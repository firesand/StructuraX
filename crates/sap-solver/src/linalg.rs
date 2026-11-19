//! Linear algebra kernels for `sap-solver`, corresponding to `csigo2_a.dll` (`Csi_sFire_*`),
//! `csigo2_d.dll` (`SFire2`), and `csigo2.dll` (`Eigen_SpacemSBR`, `Ritz_SpacemSBR`, `BuckLin_SpacemSBR`).

use thiserror::Error;

#[derive(Debug, Error, Clone)]
pub enum LinalgError {
    #[error("Matrix dimension mismatch: expected {expected}, got {actual}")]
    DimensionMismatch { expected: usize, actual: usize },
    #[error("Singular matrix encountered at pivot index {0}")]
    SingularMatrix(usize),
    /// A singular pivot of a dense auxiliary or reduced matrix (an element
    /// matrix, a free-DOF block, a shifted matrix): its index is not a global
    /// degree of freedom and must never be reported as a joint.
    #[error("Singular matrix encountered at pivot index {0} of a reduced or auxiliary matrix")]
    SingularReduced(usize),
    #[error("Eigenvalue solver failed to converge within {0} sweeps")]
    EigenConvergence(usize),
    #[error("Nonfinite value encountered in {0}")]
    NonFinite(&'static str),
    #[error(
        "Subspace eigensolution failed the Sturm sequence check: {negative} negative pivots, \
         {below} computed eigenvalues below the shift"
    )]
    EigenSturmMismatch { negative: usize, below: usize },
    #[error("Subspace eigensolution failed: {0}")]
    EigenSubspace(&'static str),
}

/// Profile (skyline) $LDL^T$ factorization of a symmetric matrix $\mathbf{K} = \mathbf{L}\mathbf{D}\mathbf{L}^T$.
///
/// The matrix stays dense and row-major, but every loop runs only over the
/// profile of each row: the first nonzero column $f_i \le i$ of row $i$ of the
/// lower triangle. Elimination creates no fill-in left of $f_i$, so skipping
/// those terms is exact (they are the terms the dense algorithm skipped through
/// its `!= 0.0` tests) and the factor is bitwise the one the dense loops gave.
///
/// Automatically stabilizes zero-stiffness DOFs (`zero_dof`).
#[derive(Debug, Clone)]
pub struct SymmetricLdlt {
    pub n: usize,
    /// Unit lower-triangular factor $\mathbf{L}$ stored row-by-row (flattened $n \times n$).
    pub l: Vec<f64>,
    /// Diagonal pivot vector $\mathbf{D}$ ($n$).
    pub d: Vec<f64>,
    /// Boolean mask indicating whether DOF $i$ had identically zero stiffness and was stabilized.
    pub zero_dof: Vec<bool>,
    /// Number of strictly negative pivots $D_{ii} < 0$ (`Csi_sFire_nSturm`).
    pub sturm_count: usize,
    /// First column of the profile of each row of $\mathbf{L}$ (`i` for a row with no off-diagonal entry).
    first: Vec<usize>,
    /// Compressed lists, per column `i`, of the rows `j > i` with `l[j][i] != 0`, increasing.
    col_ptr: Vec<usize>,
    col_rows: Vec<usize>,
}

/// First nonzero column of every row of the lower triangle of `k` (`i` if none,
/// and for zero DOFs, whose factor rows hold only the unit diagonal).
fn row_profile(k: &[f64], n: usize, zero_dof: &[bool]) -> Vec<usize> {
    (0..n)
        .map(|i| {
            if zero_dof[i] {
                return i;
            }
            k[i * n..i * n + i]
                .iter()
                .position(|&v| v != 0.0)
                .unwrap_or(i)
        })
        .collect()
}

impl SymmetricLdlt {
    /// Factorizes an $n \times n$ symmetric matrix `k` (stored as flattened row-major `n * n`).
    pub fn factorize(k: &[f64], n: usize) -> Result<Self, LinalgError> {
        Self::factorize_with_reference(k, n, None)
    }

    /// As [`Self::factorize`], with a reference diagonal for the singularity
    /// test. A pivot is normally compared with the original diagonal of its
    /// DOF. A penalty constraint inflates that diagonal by many orders of
    /// magnitude, and the DOF of a constrained pair that is eliminated second
    /// legitimately keeps only the structural stiffness, so the comparison
    /// would report a sound but flexible structure as singular. Where
    /// `reference[j]` is positive (the structural diagonal without penalties)
    /// the pivot is compared with the smaller of the two, and where it is zero
    /// with nothing; in both cases never with less than the roundoff of the
    /// arithmetic actually performed on the inflated diagonal, so a true
    /// mechanism at a constrained DOF is still detected.
    pub fn factorize_with_reference(
        k: &[f64],
        n: usize,
        reference: Option<&[f64]>,
    ) -> Result<Self, LinalgError> {
        if reference.is_some_and(|r| r.len() != n) {
            return Err(LinalgError::DimensionMismatch {
                expected: n,
                actual: reference.map_or(0, <[f64]>::len),
            });
        }
        if k.len() != n * n {
            return Err(LinalgError::DimensionMismatch {
                expected: n * n,
                actual: k.len(),
            });
        }
        if k.iter().any(|v| !v.is_finite()) {
            return Err(LinalgError::NonFinite("stiffness matrix"));
        }
        let mut l = vec![0.0; n * n];
        let mut d = vec![0.0; n];
        let mut zero_dof = vec![false; n];

        let mut orig_diag = vec![0.0f64; n];
        for i in 0..n {
            orig_diag[i] = k[i * n + i];
        }

        // Identify rows/cols that are completely zero prior to elimination (unused DOFs, e.g. 2D truss out-of-plane or released rotations)
        for i in 0..n {
            let mut row_norm = 0.0f64;
            for j in 0..n {
                row_norm = row_norm.max(k[i * n + j].abs());
            }
            if row_norm == 0.0 {
                zero_dof[i] = true;
            }
            l[i * n + i] = 1.0;
        }
        let first = row_profile(k, n, &zero_dof);

        // Row-oriented elimination. Entry (i, j) of L needs only L entries of
        // rows i and j left of column j and the pivots D_p, p < j, all of which
        // exist when row i is reached; the terms and their order (increasing p)
        // are those of the column-oriented dense algorithm, and so is the order
        // in which pivots are tested (increasing index).
        let mut inv_d = vec![0.0f64; n];
        for i in 0..n {
            if zero_dof[i] {
                d[i] = 1.0;
                continue;
            }
            let row = i * n;
            let fi = first[i];
            // Left of the profile K is zero, so L is zero there. The dense
            // algorithm stored k * (1/D) (signed zero included); keep that.
            for j in 0..fi {
                if !zero_dof[j] {
                    l[row + j] = k[row + j] * inv_d[j];
                }
            }
            for j in fi..i {
                if zero_dof[j] {
                    continue;
                }
                let mut sum = k[row + j];
                let jrow = j * n;
                for p in fi.max(first[j])..j {
                    let l_ip = l[row + p];
                    let l_jp = l[jrow + p];
                    if l_ip != 0.0 && l_jp != 0.0 {
                        sum -= l_ip * l_jp * d[p];
                    }
                }
                l[row + j] = sum * inv_d[j];
            }

            let mut dj = k[row + i];
            for p in fi..i {
                let l_jp = l[row + p];
                if l_jp != 0.0 {
                    dj -= l_jp * l_jp * d[p];
                }
            }

            // If this DOF had positive initial stiffness,
            // losing stiffness during elimination (`dj <= 1e-11 * orig_diag[j]`) or going negative
            // indicates a kinematic mechanism or unstable structure!
            if !dj.is_finite() {
                return Err(LinalgError::NonFinite("stiffness factorization"));
            }
            let scale = match reference.map(|r| r[i]) {
                Some(structural) if structural > 0.0 => orig_diag[i].abs().min(structural),
                // A DOF whose whole diagonal is penalty (for example the
                // rotation of a solid joint tied into a body constraint) has no
                // structural scale; only the roundoff floor applies to it.
                Some(_) => 0.0,
                None => orig_diag[i].abs(),
            };
            let roundoff_floor = 64.0 * f64::EPSILON * orig_diag[i].abs();
            let rel_pivot_tol = (scale * 1e-11).max(if reference.is_some() {
                roundoff_floor
            } else {
                0.0
            });
            if dj <= rel_pivot_tol {
                return Err(LinalgError::SingularMatrix(i));
            }

            d[i] = dj;
            inv_d[i] = 1.0 / dj;
        }

        // Column lists of the nonzero entries of L for the backward sweep.
        let mut col_ptr = vec![0usize; n + 1];
        for j in 0..n {
            for i in first[j]..j {
                if l[j * n + i] != 0.0 {
                    col_ptr[i + 1] += 1;
                }
            }
        }
        for i in 0..n {
            col_ptr[i + 1] += col_ptr[i];
        }
        let mut fill = col_ptr.clone();
        let mut col_rows = vec![0usize; col_ptr[n]];
        for j in 0..n {
            for i in first[j]..j {
                if l[j * n + i] != 0.0 {
                    col_rows[fill[i]] = j;
                    fill[i] += 1;
                }
            }
        }

        Ok(Self {
            n,
            l,
            d,
            zero_dof,
            sturm_count: 0,
            first,
            col_ptr,
            col_rows,
        })
    }

    /// Solves $\mathbf{K} \mathbf{x} = \mathbf{b}$ using forward, diagonal, and backward substitution.
    pub fn solve(&self, b: &[f64]) -> Result<Vec<f64>, LinalgError> {
        if b.len() != self.n {
            return Err(LinalgError::DimensionMismatch {
                expected: self.n,
                actual: b.len(),
            });
        }
        if b.iter().any(|v| !v.is_finite()) {
            return Err(LinalgError::NonFinite("load vector"));
        }
        let n = self.n;
        let mut y = vec![0.0; n];

        // Forward substitution: L * y = b
        for i in 0..n {
            if self.zero_dof[i] {
                if b[i] != 0.0 {
                    // Applied load on an unrestrained zero-stiffness DOF -> mechanism!
                    return Err(LinalgError::SingularMatrix(i));
                }
                y[i] = 0.0;
                continue;
            }
            let mut sum = b[i];
            let row_off = i * n;
            for j in self.first[i]..i {
                let l_ij = self.l[row_off + j];
                if l_ij != 0.0 {
                    sum -= l_ij * y[j];
                }
            }
            y[i] = sum;
        }

        // Diagonal scaling: D * z = y
        for i in 0..n {
            if self.zero_dof[i] {
                y[i] = 0.0;
            } else {
                y[i] /= self.d[i];
            }
        }

        // Backward substitution: L^T * x = z
        let mut x = y;
        for i in (0..n).rev() {
            if self.zero_dof[i] {
                x[i] = 0.0;
                continue;
            }
            let mut sum = x[i];
            for &j in &self.col_rows[self.col_ptr[i]..self.col_ptr[i + 1]] {
                sum -= self.l[j * n + i] * x[j];
            }
            x[i] = sum;
        }

        if x.iter().any(|v| !v.is_finite()) {
            return Err(LinalgError::NonFinite("solution vector"));
        }
        Ok(x)
    }
}

/// Maps the pivot index of a failure in a sub-matrix to the index in the full
/// matrix: `dofs[i]` is the global index of row `i` of the sub-matrix. Only
/// `SingularMatrix` carries an index of that kind.
pub fn remap_singular_index(error: LinalgError, dofs: &[usize]) -> LinalgError {
    match error {
        LinalgError::SingularMatrix(i) => match dofs.get(i) {
            Some(&global) => LinalgError::SingularMatrix(global),
            None => LinalgError::SingularReduced(i),
        },
        other => other,
    }
}

/// Result of [`scaled_stiffness_minimum`].
#[derive(Debug, Clone, Copy)]
pub struct ScaledMinimum {
    /// Rayleigh quotient `z^T S z` of the last unit vector `z`, with
    /// `S = D^{-1/2} K D^{-1/2}`: close to the smallest eigenvalue of `S`.
    pub mu: f64,
    /// Index of the entry of `z` with the largest magnitude.
    pub worst: usize,
}

/// Inverse iteration on the diagonally scaled matrix `S = D^{-1/2} K D^{-1/2}`
/// (three solves with the existing factor `ldlt` of `k`), used after a
/// successful factorization to detect a mechanism whose pivots stayed above the
/// pivot test. `D_j` is `reference[j]` where that is positive (the structural
/// diagonal, the reference of the pivot test) and `k[j][j]` otherwise;
/// `excluded` (restrained DOFs) and the zero DOFs of the factor are left out.
/// Returns `None` when no free DOF with a positive scale remains.
///
/// Not called by the analysis in wave 5: measured on mechanisms and sound
/// models, `mu` separates them without penalty constraints (largest mechanism
/// 1.7e-16, smallest sound model 1.7e-12) but not with them (a Diaphragm
/// constraint raised the mechanism's `mu` to between -1.6e-8 and +4.5e-7), see
/// the wave 5 report of D03. It is kept, with its closed-form test, for the
/// wave that removes the penalty constraints.
pub fn scaled_stiffness_minimum(
    k: &[f64],
    n: usize,
    ldlt: &SymmetricLdlt,
    reference: Option<&[f64]>,
    excluded: &[bool],
) -> Result<Option<ScaledMinimum>, LinalgError> {
    if k.len() != n * n || ldlt.n != n || excluded.len() != n {
        return Err(LinalgError::DimensionMismatch {
            expected: n,
            actual: excluded.len(),
        });
    }
    let root: Vec<f64> = (0..n)
        .map(|j| {
            let structural = reference.map_or(0.0, |r| r[j]);
            let scale = if structural > 0.0 { structural } else { k[j * n + j] };
            if excluded[j] || ldlt.zero_dof[j] || !scale.is_finite() || scale <= 0.0 {
                0.0
            } else {
                scale.sqrt()
            }
        })
        .collect();
    let free: Vec<usize> = (0..n).filter(|&j| root[j] > 0.0).collect();
    if free.is_empty() {
        return Ok(None);
    }
    let mut z = vec![0.0; n];
    let norm = (free.len() as f64).sqrt();
    for &j in &free {
        z[j] = 1.0 / norm;
    }
    for _ in 0..3 {
        let b: Vec<f64> = (0..n).map(|j| root[j] * z[j]).collect();
        let x = ldlt.solve(&b)?;
        let mut y: Vec<f64> = (0..n).map(|j| root[j] * x[j]).collect();
        let len = y.iter().map(|v| v * v).sum::<f64>().sqrt();
        if !len.is_finite() || len <= 0.0 {
            return Err(LinalgError::NonFinite("mechanism check"));
        }
        for v in &mut y {
            *v /= len;
        }
        z = y;
    }
    let w: Vec<f64> = (0..n)
        .map(|j| if root[j] > 0.0 { z[j] / root[j] } else { 0.0 })
        .collect();
    let mut mu = 0.0;
    for &i in &free {
        let row = &k[i * n..(i + 1) * n];
        let kw: f64 = free.iter().map(|&j| row[j] * w[j]).sum();
        mu += w[i] * kw;
    }
    let worst = free
        .iter()
        .copied()
        .max_by(|&a, &b| z[a].abs().total_cmp(&z[b].abs()))
        .unwrap_or(0);
    Ok(Some(ScaledMinimum { mu, worst }))
}

/// Solves a dense linear system $\mathbf{A} \mathbf{x} = \mathbf{b}$ via Gaussian elimination with partial pivoting.
pub fn solve_lu(a: &[f64], b: &[f64], n: usize) -> Result<Vec<f64>, LinalgError> {
    if a.len() != n * n || b.len() != n {
        return Err(LinalgError::DimensionMismatch {
            expected: n * n,
            actual: a.len(),
        });
    }
    let mut mat = a.to_vec();
    let mut rhs = b.to_vec();

    for k in 0..n {
        let mut max_row = k;
        let mut max_val = mat[k * n + k].abs();
        for i in (k + 1)..n {
            let val = mat[i * n + k].abs();
            if val > max_val {
                max_val = val;
                max_row = i;
            }
        }

        if max_val < 1e-18 {
            return Err(LinalgError::SingularReduced(k));
        }

        if max_row != k {
            for j in k..n {
                mat.swap(k * n + j, max_row * n + j);
            }
            rhs.swap(k, max_row);
        }

        let pivot = mat[k * n + k];
        for i in (k + 1)..n {
            let factor = mat[i * n + k] / pivot;
            mat[i * n + k] = 0.0;
            if factor.abs() > 0.0 {
                for j in (k + 1)..n {
                    mat[i * n + j] -= factor * mat[k * n + j];
                }
                rhs[i] -= factor * rhs[k];
            }
        }
    }

    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum = rhs[i];
        for j in (i + 1)..n {
            sum -= mat[i * n + j] * x[j];
        }
        x[i] = sum / mat[i * n + i];
    }
    Ok(x)
}

/// Computes the inverse of an $n \times n$ matrix `a` (flattened row-major).
pub fn invert_matrix(a: &[f64], n: usize) -> Result<Vec<f64>, LinalgError> {
    let mut inv = vec![0.0; n * n];
    let mut e = vec![0.0; n];
    for col in 0..n {
        e.fill(0.0);
        e[col] = 1.0;
        let x = solve_lu(a, &e, n)?;
        for row in 0..n {
            inv[row * n + col] = x[row];
        }
    }
    Ok(inv)
}

/// Cyclic Jacobi eigenvalue solver for a real symmetric $n \times n$ matrix $\mathbf{A}$.
///
/// Returns `(eigenvalues, eigenvectors_col_major)` where column `k` of `eigenvectors`
/// (i.e. `vecs[i * n + k]` in row-major storage) is the orthonormal eigenvector for `eigenvalues[k]`.
pub fn jacobi_symmetric_eigen(
    a_in: &[f64],
    n: usize,
    max_sweeps: usize,
) -> Result<(Vec<f64>, Vec<f64>), LinalgError> {
    if a_in.len() != n * n {
        return Err(LinalgError::DimensionMismatch {
            expected: n * n,
            actual: a_in.len(),
        });
    }
    if n == 0 {
        return Ok((Vec::new(), Vec::new()));
    }
    let mut a = a_in.to_vec();
    let mut v = vec![0.0; n * n];
    for i in 0..n {
        v[i * n + i] = 1.0;
    }

    let mut diag_norm = 0.0f64;
    for i in 0..n {
        diag_norm = diag_norm.max(a[i * n + i].abs());
    }
    let tol = (diag_norm * 1e-15).max(1e-24);

    for _sweep in 0..max_sweeps {
        let mut max_off = 0.0f64;
        for p in 0..n {
            for q in (p + 1)..n {
                max_off = max_off.max(a[p * n + q].abs());
            }
        }
        if max_off <= tol {
            let mut eigvals = vec![0.0; n];
            for i in 0..n {
                eigvals[i] = a[i * n + i];
            }
            return Ok((eigvals, v));
        }

        for p in 0..n {
            for q in (p + 1)..n {
                let apq = a[p * n + q];
                if apq.abs() <= tol * 0.01 {
                    a[p * n + q] = 0.0;
                    a[q * n + p] = 0.0;
                    continue;
                }
                let app = a[p * n + p];
                let aqq = a[q * n + q];
                let tau = (aqq - app) / (2.0 * apq);
                let t = if tau >= 0.0 {
                    1.0 / (tau + (1.0 + tau * tau).sqrt())
                } else {
                    -1.0 / (-tau + (1.0 + tau * tau).sqrt())
                };
                let c = 1.0 / (1.0 + t * t).sqrt();
                let s = t * c;

                a[p * n + p] = app - t * apq;
                a[q * n + q] = aqq + t * apq;
                a[p * n + q] = 0.0;
                a[q * n + p] = 0.0;

                for r in 0..n {
                    if r != p && r != q {
                        let arp = a[r * n + p];
                        let arq = a[r * n + q];
                        let new_rp = c * arp - s * arq;
                        let new_rq = s * arp + c * arq;
                        a[r * n + p] = new_rp;
                        a[p * n + r] = new_rp;
                        a[r * n + q] = new_rq;
                        a[q * n + r] = new_rq;
                    }
                }

                for r in 0..n {
                    let vrp = v[r * n + p];
                    let vrq = v[r * n + q];
                    v[r * n + p] = c * vrp - s * vrq;
                    v[r * n + q] = s * vrp + c * vrq;
                }
            }
        }
    }

    Err(LinalgError::EigenConvergence(max_sweeps))
}

/// Solves the generalized undamped vibration eigenvalue problem $(\mathbf{K} - \omega^2 \mathbf{M})\boldsymbol{\phi} = \mathbf{0}$
/// where $\mathbf{M}$ is a diagonal lumped mass vector (`diag_mass`).
///
/// Uses exact dynamic condensation onto the mass-carrying DOFs $D = \{i \mid M_i > 0\}$ via the
/// symmetric flexibility operator $\mathbf{F} = \mathbf{M}_D^{1/2} \mathbf{K}^{-1}_{DD} \mathbf{M}_D^{1/2}$,
/// so that $\mathbf{F} \mathbf{z}_k = \frac{1}{\omega_k^2} \mathbf{z}_k$ and $\boldsymbol{\phi}_k^T \mathbf{M} \boldsymbol{\phi}_k = 1$.
pub fn solve_modal_eigen(
    ldlt: &SymmetricLdlt,
    diag_mass: &[f64],
    max_modes: usize,
) -> Result<Vec<(f64, Vec<f64>)>, LinalgError> {
    let n = ldlt.n;
    let mass_dofs: Vec<usize> = (0..n)
        .filter(|&i| !ldlt.zero_dof[i] && diag_mass[i] > 1e-18)
        .collect();
    let nm = mass_dofs.len();
    if nm == 0 || max_modes == 0 {
        return Ok(Vec::new());
    }

    // For each mass-carrying DOF j, solve K * c_j = sqrt(M_j) * e_j
    let mut cols: Vec<Vec<f64>> = Vec::with_capacity(nm);
    let mut rhs = vec![0.0; n];
    for &dof_j in &mass_dofs {
        rhs.fill(0.0);
        rhs[dof_j] = diag_mass[dof_j].sqrt();
        let cj = ldlt.solve(&rhs)?;
        cols.push(cj);
    }

    // Assemble symmetric nm x nm dynamic flexibility matrix F_{ab} = sqrt(M_a) * c_b[dof_a]
    let mut f_mat = vec![0.0; nm * nm];
    for a in 0..nm {
        let dof_a = mass_dofs[a];
        let sqrt_ma = diag_mass[dof_a].sqrt();
        for b in a..nm {
            let val = 0.5
                * (sqrt_ma * cols[b][dof_a]
                    + diag_mass[mass_dofs[b]].sqrt() * cols[a][mass_dofs[b]]);
            f_mat[a * nm + b] = val;
            f_mat[b * nm + a] = val;
        }
    }

    let (mu_vals, z_vecs) = jacobi_symmetric_eigen(&f_mat, nm, 80)?;

    // Sort by descending mu = 1 / omega^2 (which corresponds to ascending omega^2)
    let mut indices: Vec<usize> = (0..nm).filter(|&k| mu_vals[k] > 1e-16).collect();
    indices.sort_by(|&a, &b| {
        mu_vals[b]
            .partial_cmp(&mu_vals[a])
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let num_keep = indices.len().min(max_modes);
    let mut modes = Vec::with_capacity(num_keep);

    for &idx in indices.iter().take(num_keep) {
        let mu = mu_vals[idx];
        let omega_sq = 1.0 / mu;

        // Reconstruct full N-DOF mode shape: phi = omega^2 * sum_b z_{b, idx} * c_b
        let mut phi = vec![0.0; n];
        for b in 0..nm {
            let z_b = z_vecs[b * nm + idx];
            let coeff = omega_sq * z_b;
            for i in 0..n {
                phi[i] += coeff * cols[b][i];
            }
        }

        // Normalize so phi^T * M * phi = 1.0 and largest component has positive sign
        let mut modal_mass = 0.0;
        let mut max_abs = 0.0;
        let mut max_sign = 1.0;
        for &dof_a in &mass_dofs {
            modal_mass += diag_mass[dof_a] * phi[dof_a] * phi[dof_a];
            if phi[dof_a].abs() > max_abs {
                max_abs = phi[dof_a].abs();
                max_sign = if phi[dof_a] >= 0.0 { 1.0 } else { -1.0 };
            }
        }
        if modal_mass > 1e-24 {
            let scale = max_sign / modal_mass.sqrt();
            for v in &mut phi {
                *v *= scale;
            }
        }

        modes.push((omega_sq, phi));
    }

    Ok(modes)
}

/// Largest mass-DOF count solved by dynamic condensation ([`solve_modal_eigen`]).
const SUBSPACE_MIN_MASS_DOFS: usize = 400;
/// Subspace iterations before the solver reports non-convergence.
const SUBSPACE_MAX_ITERATIONS: usize = 200;
/// Relative eigenvalue change accepted as converged.
const SUBSPACE_TOLERANCE: f64 = 1e-12;
/// Relative margin above the last wanted eigenvalue at which the Sturm check is taken.
const SUBSPACE_SHIFT_MARGIN: f64 = 1e-6;
/// Largest number of mass degrees of freedom for which the exact condensation
/// replaces a subspace iteration that could not prove its result (about a
/// minute of work at this size).
const CONDENSATION_FALLBACK_MAX_MASS_DOFS: usize = 1500;

/// Eigen modes of `k` (dense, row-major, restraints applied, factorized as
/// `ldlt`) with the diagonal mass `diag_mass`, lowest first, at most
/// `max_modes`.
///
/// Small problems (at most 400 mass DOFs, or when the modes wanted are a
/// quarter or more of them) use the exact dynamic condensation of
/// [`solve_modal_eigen`], unchanged. Larger ones use
/// [`solve_modal_eigen_subspace`], which checks its result with a Sturm
/// sequence and never returns a set with a missing mode.
pub fn solve_modal_eigen_checked(
    k: &[f64],
    ldlt: &SymmetricLdlt,
    diag_mass: &[f64],
    max_modes: usize,
) -> Result<Vec<(f64, Vec<f64>)>, LinalgError> {
    let nm = (0..ldlt.n)
        .filter(|&i| !ldlt.zero_dof[i] && diag_mass[i] > 1e-18)
        .count();
    if nm <= SUBSPACE_MIN_MASS_DOFS || 4 * max_modes >= nm {
        return solve_modal_eigen(ldlt, diag_mass, max_modes);
    }
    match solve_modal_eigen_subspace(k, ldlt, diag_mass, max_modes) {
        // The iteration could not prove its set complete. The condensation is
        // exact for every model, only slower, so it decides instead of
        // failing a model that it can solve. Its cost grows with the cube of
        // the number of mass degrees of freedom: above the bound it would run
        // for hours without a word, so there the error is returned.
        Err(
            LinalgError::EigenSturmMismatch { .. }
            | LinalgError::EigenSubspace(_)
            | LinalgError::EigenConvergence { .. },
        ) if nm <= CONDENSATION_FALLBACK_MAX_MASS_DOFS => {
            solve_modal_eigen(ldlt, diag_mass, max_modes)
        }
        other => other,
    }
}

/// Failure of one Sturm-sequence factorization.
#[derive(Debug)]
enum ShiftedFactorFailure {
    /// A pivot was exactly zero: the shift is an eigenvalue of the pencil.
    ZeroPivot,
    NonFinite,
}

/// Pivots of $\mathbf{K} - \sigma\mathbf{M}$ (diagonal `mass`, zero off the
/// mass-bearing free DOFs), factorized in skyline storage over the profile of
/// `ldlt` without any pivot refusal.
///
/// By Sylvester's law of inertia the number of negative pivots over ALL
/// equations is the number of eigenvalues of
/// $\mathbf{K}\boldsymbol{\phi}=\lambda\mathbf{M}\boldsymbol{\phi}$ below
/// $\sigma$, when $\mathbf{K}$ is positive definite on the DOFs that are not
/// `zero_dof`. Which row carries a negative pivot depends on the elimination
/// order: a massless free DOF eliminated after the mass DOFs it couples to can
/// itself carry the negative pivot (the shifted matrix is congruent to the one
/// with that DOF condensed out), so the count is NOT restricted to mass-bearing
/// equations. Restrained DOFs (unit diagonal, no coupling, no mass) always give
/// a positive pivot and zero DOFs are skipped (pivot 1) as in the factorization
/// of $\mathbf{K}$; neither changes the count.
fn shifted_pivots(
    k: &[f64],
    ldlt: &SymmetricLdlt,
    mass: &[f64],
    sigma: f64,
) -> Result<Vec<f64>, ShiftedFactorFailure> {
    let n = ldlt.n;
    let first = &ldlt.first;
    let mut offset = vec![0usize; n + 1];
    for i in 0..n {
        offset[i + 1] = offset[i] + (i - first[i]);
    }
    let mut sky = vec![0.0f64; offset[n]];
    let mut d = vec![1.0f64; n];
    for i in 0..n {
        if ldlt.zero_dof[i] {
            continue;
        }
        let fi = first[i];
        let base = offset[i];
        for j in fi..i {
            if ldlt.zero_dof[j] {
                continue;
            }
            let mut sum = k[i * n + j];
            let fj = first[j];
            let jbase = offset[j];
            for p in fi.max(fj)..j {
                let l_ip = sky[base + p - fi];
                let l_jp = sky[jbase + p - fj];
                sum -= l_ip * l_jp * d[p];
            }
            sky[base + j - fi] = sum / d[j];
        }
        let mut dii = k[i * n + i] - sigma * mass[i];
        for p in fi..i {
            let l_ip = sky[base + p - fi];
            dii -= l_ip * l_ip * d[p];
        }
        if !dii.is_finite() {
            return Err(ShiftedFactorFailure::NonFinite);
        }
        if dii == 0.0 {
            return Err(ShiftedFactorFailure::ZeroPivot);
        }
        d[i] = dii;
    }
    Ok(d)
}

/// Number of negative pivots of [`shifted_pivots`]; see there for what it counts.
fn shifted_negative_pivots(
    k: &[f64],
    ldlt: &SymmetricLdlt,
    mass: &[f64],
    sigma: f64,
) -> Result<usize, ShiftedFactorFailure> {
    Ok(shifted_pivots(k, ldlt, mass, sigma)?
        .iter()
        .filter(|&&v| v < 0.0)
        .count())
}

/// Lowest `max_modes` eigenpairs of $(\mathbf{K}-\lambda\mathbf{M})\boldsymbol\phi=\mathbf{0}$ by subspace
/// iteration with Rayleigh-Ritz projection (Bathe, *Finite Element
/// Procedures*, 1996, section 11.6) on the profile factorization `ldlt` of
/// `k`, with a Sturm-sequence check that no eigenvalue below the last
/// computed one has been missed.
///
/// $q = \min(\max(2p, p+8), n_m)$ vectors are iterated, started from
/// $\mathbf{M}\mathbf{1}$ and unit vectors at the DOFs with the largest
/// $m_{ii}/k_{ii}$. Each pass computes $\bar{X}=K^{-1}MX$, projects
/// $K_q=\bar X^T M X=\bar X^T K\bar X$ and $M_q=\bar X^T M\bar X$, reduces
/// with a Cholesky factor of $M_q$ and solves with
/// [`jacobi_symmetric_eigen`]. Converged means $|\Delta\lambda_i|\le
/// 10^{-12}\lambda_i$ for the wanted modes (and for every Ritz value within
/// $10^{-6}$ above the last one, so that a repeated eigenvalue is converged
/// together); 200 iterations without that is an error.
///
/// Sturm check: $\mathbf{K}-\sigma\mathbf{M}$ with $\sigma=\lambda_p(1+10^{-6})$
/// is factorized without pivot refusal; the negative pivots are exactly the
/// eigenvalues below $\sigma$ (the shift touches only mass-bearing free DOFs,
/// and restrained, zero and massless DOFs carry positive or skipped pivots), and
/// their number must equal the number of computed Ritz values below $\sigma$.
/// On a mismatch the iteration is repeated once with $q$ doubled, and then an
/// error is returned. An exactly zero pivot moves $\sigma$ by $10^{-10}$
/// relative.
///
/// Modes are normalized as in [`solve_modal_eigen`]: $\phi^T M\phi=1$, the
/// largest mass-DOF component positive, ascending eigenvalue.
pub fn solve_modal_eigen_subspace(
    k: &[f64],
    ldlt: &SymmetricLdlt,
    diag_mass: &[f64],
    max_modes: usize,
) -> Result<Vec<(f64, Vec<f64>)>, LinalgError> {
    let n = ldlt.n;
    if k.len() != n * n {
        return Err(LinalgError::DimensionMismatch {
            expected: n * n,
            actual: k.len(),
        });
    }
    if diag_mass.len() != n {
        return Err(LinalgError::DimensionMismatch {
            expected: n,
            actual: diag_mass.len(),
        });
    }
    let mass_dofs: Vec<usize> = (0..n)
        .filter(|&i| !ldlt.zero_dof[i] && diag_mass[i] > 1e-18)
        .collect();
    let nm = mass_dofs.len();
    if nm == 0 || max_modes == 0 {
        return Ok(Vec::new());
    }
    let p = max_modes.min(nm);
    let q = (2 * p).max(p + 8).min(nm);
    match subspace_attempt(k, ldlt, diag_mass, &mass_dofs, p, q) {
        Err(LinalgError::EigenSturmMismatch { .. }) if q < nm => {
            subspace_attempt(k, ldlt, diag_mass, &mass_dofs, p, (2 * q).min(nm))
        }
        other => other,
    }
}

fn subspace_attempt(
    k: &[f64],
    ldlt: &SymmetricLdlt,
    m: &[f64],
    mass_dofs: &[usize],
    p: usize,
    q: usize,
) -> Result<Vec<(f64, Vec<f64>)>, LinalgError> {
    use rayon::prelude::*;

    let n = ldlt.n;
    let nm = mass_dofs.len();
    let mass_at: Vec<f64> = mass_dofs.iter().map(|&i| m[i]).collect();

    // Starting vectors on the mass DOFs: M*1, then unit vectors at the DOFs
    // with the largest m_ii / k_ii.
    let mut order: Vec<usize> = (0..nm).collect();
    let ratio = |a: usize| {
        let dof = mass_dofs[a];
        let kii = k[dof * n + dof];
        if kii > 0.0 {
            mass_at[a] / kii
        } else {
            f64::INFINITY
        }
    };
    order.sort_by(|&a, &b| ratio(b).total_cmp(&ratio(a)).then(a.cmp(&b)));
    let mut x: Vec<Vec<f64>> = Vec::with_capacity(q);
    x.push(mass_at.clone());
    for &a in order.iter().take(q - 1) {
        let mut e = vec![0.0; nm];
        e[a] = 1.0;
        x.push(e);
    }

    let mut previous: Option<Vec<f64>> = None;
    for _iteration in 0..SUBSPACE_MAX_ITERATIONS {
        // Xbar = K^-1 M X, one independent solve per column.
        let xbar: Vec<Vec<f64>> = x
            .par_iter()
            .map(|col| {
                let mut rhs = vec![0.0; n];
                for (a, &dof) in mass_dofs.iter().enumerate() {
                    rhs[dof] = mass_at[a] * col[a];
                }
                ldlt.solve(&rhs)
            })
            .collect::<Result<_, _>>()?;
        let xbar_mass: Vec<Vec<f64>> = xbar
            .iter()
            .map(|col| mass_dofs.iter().map(|&dof| col[dof]).collect())
            .collect();

        // K_q = Xbar^T M X and M_q = Xbar^T M Xbar.
        let rows: Vec<(Vec<f64>, Vec<f64>)> = (0..q)
            .into_par_iter()
            .map(|j| {
                let mut a_row = vec![0.0; q];
                let mut b_row = vec![0.0; q];
                for l in 0..q {
                    let mut a_sum = 0.0;
                    let mut b_sum = 0.0;
                    for a in 0..nm {
                        let w = mass_at[a] * xbar_mass[j][a];
                        a_sum += w * x[l][a];
                        b_sum += w * xbar_mass[l][a];
                    }
                    a_row[l] = a_sum;
                    b_row[l] = b_sum;
                }
                (a_row, b_row)
            })
            .collect();
        // Unit-diagonal scaling keeps the (1/lambda^2-spread) columns comparable.
        let mut scale = vec![0.0; q];
        for j in 0..q {
            let bjj = rows[j].1[j];
            if bjj <= 0.0 || !bjj.is_finite() {
                return Err(LinalgError::EigenSubspace("iteration vectors lost rank"));
            }
            scale[j] = 1.0 / bjj.sqrt();
        }
        let mut a_mat = vec![0.0; q * q];
        let mut b_mat = vec![0.0; q * q];
        for j in 0..q {
            for l in 0..q {
                a_mat[j * q + l] = 0.5 * (rows[j].0[l] + rows[l].0[j]) * scale[j] * scale[l];
                b_mat[j * q + l] = 0.5 * (rows[j].1[l] + rows[l].1[j]) * scale[j] * scale[l];
            }
        }

        // Reduce to the standard problem with the Cholesky factor of M_q.
        let chol = cholesky_lower(&b_mat, q).ok_or(LinalgError::EigenSubspace(
            "projected mass matrix is not positive definite",
        ))?;
        let mut s_mat = vec![0.0; q * q];
        // C = L^-1 A (column by column), then S = L^-1 C^T.
        let mut c_mat = a_mat;
        for col in 0..q {
            for i in 0..q {
                let mut sum = c_mat[i * q + col];
                for t in 0..i {
                    sum -= chol[i * q + t] * c_mat[t * q + col];
                }
                c_mat[i * q + col] = sum / chol[i * q + i];
            }
        }
        for col in 0..q {
            for i in 0..q {
                let mut sum = c_mat[col * q + i];
                for t in 0..i {
                    sum -= chol[i * q + t] * s_mat[t * q + col];
                }
                s_mat[i * q + col] = sum / chol[i * q + i];
            }
        }
        for i in 0..q {
            for j in (i + 1)..q {
                let avg = 0.5 * (s_mat[i * q + j] + s_mat[j * q + i]);
                s_mat[i * q + j] = avg;
                s_mat[j * q + i] = avg;
            }
        }
        let (values, z) = jacobi_symmetric_eigen(&s_mat, q, 80)?;
        let mut idx: Vec<usize> = (0..q).collect();
        idx.sort_by(|&a, &b| values[a].total_cmp(&values[b]));
        let lambda: Vec<f64> = idx.iter().map(|&i| values[i]).collect();
        // Q = D L^-T Z, columns in ascending eigenvalue order.
        let mut q_mat = vec![0.0; q * q];
        for (out, &src) in idx.iter().enumerate() {
            for i in (0..q).rev() {
                let mut sum = z[i * q + src];
                for t in (i + 1)..q {
                    sum -= chol[t * q + i] * q_mat[t * q + out];
                }
                q_mat[i * q + out] = sum / chol[i * q + i];
            }
            for j in 0..q {
                q_mat[j * q + out] *= scale[j];
            }
        }

        let cluster_limit = lambda[p - 1] * (1.0 + SUBSPACE_SHIFT_MARGIN);
        let wanted = lambda
            .iter()
            .take_while(|&&v| v <= cluster_limit)
            .count()
            .max(p);
        let converged = previous.as_ref().is_some_and(|prev| {
            (0..wanted).all(|i| (lambda[i] - prev[i]).abs() <= SUBSPACE_TOLERANCE * lambda[i].abs())
        });

        if converged {
            return finish_subspace(k, ldlt, m, mass_dofs, p, &xbar, &q_mat, &lambda);
        }

        // Next iterate: X = Xbar Q on the mass DOFs.
        x = (0..q)
            .into_par_iter()
            .map(|l| {
                let mut col = vec![0.0; nm];
                for j in 0..q {
                    let w = q_mat[j * q + l];
                    for a in 0..nm {
                        col[a] += w * xbar_mass[j][a];
                    }
                }
                col
            })
            .collect();
        previous = Some(lambda);
    }
    Err(LinalgError::EigenConvergence(SUBSPACE_MAX_ITERATIONS))
}

/// Sturm check and mode recovery once the Ritz values have converged.
#[allow(clippy::too_many_arguments)]
fn finish_subspace(
    k: &[f64],
    ldlt: &SymmetricLdlt,
    m: &[f64],
    mass_dofs: &[usize],
    p: usize,
    xbar: &[Vec<f64>],
    q_mat: &[f64],
    lambda: &[f64],
) -> Result<Vec<(f64, Vec<f64>)>, LinalgError> {
    let n = ldlt.n;
    let q = lambda.len();
    let mut mass = vec![0.0; n];
    for &dof in mass_dofs {
        mass[dof] = m[dof];
    }
    // The shift sits above the wanted values and the ones clustered with the
    // last of them. Just above (the margin) is enough in exact arithmetic,
    // but the sign of a pivot is only reliable when the shift is farther from
    // every eigenvalue than the factorization's rounding, which for a stiff
    // model is not small against the margin. So the shift moves up into the
    // gap to the next Ritz value: half of the gap, at most one percent (the
    // distance Bathe uses). A true eigenvalue inside that gap, which the
    // iteration has not found, then shows as a mismatch, as it must.
    let cluster_limit = lambda[p - 1] * (1.0 + SUBSPACE_SHIFT_MARGIN);
    let wanted = lambda
        .iter()
        .take_while(|&&v| v <= cluster_limit)
        .count()
        .max(p);
    let top = lambda[wanted - 1];
    let mut sigma = top * (1.0 + SUBSPACE_SHIFT_MARGIN);
    if wanted < q {
        let wide = top + (0.01 * top).min(0.5 * (lambda[wanted] - top));
        if wide > sigma {
            sigma = wide;
        }
    }
    let mut negative = None;
    for _ in 0..4 {
        match shifted_negative_pivots(k, ldlt, &mass, sigma) {
            Ok(count) => {
                negative = Some(count);
                break;
            }
            Err(ShiftedFactorFailure::ZeroPivot) => sigma *= 1.0 + 1e-10,
            Err(ShiftedFactorFailure::NonFinite) => {
                return Err(LinalgError::NonFinite("shifted stiffness factorization"));
            }
        }
    }
    let negative = negative.ok_or(LinalgError::SingularReduced(0))?;
    let below = lambda.iter().filter(|&&v| v < sigma).count();
    if negative != below {
        return Err(LinalgError::EigenSturmMismatch { negative, below });
    }

    let mut modes = Vec::with_capacity(p);
    for l in 0..p {
        let mut phi = vec![0.0; n];
        for j in 0..q {
            let w = q_mat[j * q + l];
            for i in 0..n {
                phi[i] += w * xbar[j][i];
            }
        }
        // Normalize so phi^T * M * phi = 1.0 and largest component has positive sign
        let mut modal_mass = 0.0;
        let mut max_abs = 0.0;
        let mut max_sign = 1.0;
        for &dof_a in mass_dofs {
            modal_mass += m[dof_a] * phi[dof_a] * phi[dof_a];
            if phi[dof_a].abs() > max_abs {
                max_abs = phi[dof_a].abs();
                max_sign = if phi[dof_a] >= 0.0 { 1.0 } else { -1.0 };
            }
        }
        if modal_mass > 1e-24 {
            let scale = max_sign / modal_mass.sqrt();
            for v in &mut phi {
                *v *= scale;
            }
        }
        modes.push((lambda[l], phi));
    }
    Ok(modes)
}

/// Lower Cholesky factor of the symmetric positive definite `a` (row-major),
/// or `None` when a pivot is not safely positive (unit-diagonal input).
fn cholesky_lower(a: &[f64], n: usize) -> Option<Vec<f64>> {
    let mut l = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..=i {
            let mut sum = a[i * n + j];
            for t in 0..j {
                sum -= l[i * n + t] * l[j * n + t];
            }
            if i == j {
                if sum <= 1e-14 || !sum.is_finite() {
                    return None;
                }
                l[i * n + i] = sum.sqrt();
            } else {
                l[i * n + j] = sum / l[j * n + j];
            }
        }
    }
    Some(l)
}

/// Generates load-dependent Ritz vectors (`Ritz_SpacemSBR`) starting from `starting_loads`,
/// orthogonalized with respect to $\mathbf{M}$ and projected onto $(\mathbf{K}, \mathbf{M})$.
pub fn solve_ritz_vectors(
    ldlt: &SymmetricLdlt,
    diag_mass: &[f64],
    starting_loads: &[Vec<f64>],
    max_modes: usize,
) -> Result<Vec<(f64, Vec<f64>)>, LinalgError> {
    if starting_loads.is_empty() {
        return solve_modal_eigen(ldlt, diag_mass, max_modes);
    }
    let n = ldlt.n;
    let m_dot = |u: &[f64], v: &[f64]| -> f64 {
        u.iter()
            .zip(v.iter())
            .zip(diag_mass.iter())
            .map(|((&ui, &vi), &mi)| ui * vi * mi)
            .sum()
    };

    let mut basis: Vec<Vec<f64>> = Vec::with_capacity(max_modes);
    // Store w_a = K^{-1} M b_a for each M-orthonormal basis vector b_a
    let mut k_inv_m_basis: Vec<Vec<f64>> = Vec::with_capacity(max_modes);

    // Block Krylov sequence generated by K^{-1} M starting from K^{-1} F_0
    let mut current_block: Vec<Vec<f64>> = Vec::new();
    for f in starting_loads {
        if let Ok(u) = ldlt.solve(f) {
            current_block.push(u);
        }
    }

    while basis.len() < max_modes && !current_block.is_empty() {
        let mut next_block = Vec::new();
        for mut cand in current_block {
            // Modified Gram-Schmidt M-orthogonalization (twice for numerical stability)
            for _pass in 0..2 {
                for b in &basis {
                    let proj = m_dot(&cand, b);
                    for i in 0..n {
                        cand[i] -= proj * b[i];
                    }
                }
            }
            let norm_sq = m_dot(&cand, &cand);
            if norm_sq > 1e-18 {
                let inv_norm = 1.0 / norm_sq.sqrt();
                for i in 0..n {
                    cand[i] *= inv_norm;
                }
                // Next Krylov vector: K * u_next = M * cand
                let mut m_cand = vec![0.0; n];
                for i in 0..n {
                    m_cand[i] = diag_mass[i] * cand[i];
                }
                if let Ok(u_next) = ldlt.solve(&m_cand) {
                    k_inv_m_basis.push(u_next.clone());
                    next_block.push(u_next);
                } else {
                    k_inv_m_basis.push(cand.clone());
                }
                basis.push(cand);
                if basis.len() >= max_modes {
                    break;
                }
            }
        }
        current_block = next_block;
    }

    let m_count = basis.len();
    if m_count == 0 {
        return solve_modal_eigen(ldlt, diag_mass, max_modes);
    }

    // Form projected m_count x m_count flexibility matrix F_R[a, b] = b_a^T * M * (K^{-1} M b_b)
    let mut f_ritz = vec![0.0; m_count * m_count];
    for a in 0..m_count {
        for b in a..m_count {
            let val =
                0.5 * (m_dot(&basis[a], &k_inv_m_basis[b]) + m_dot(&basis[b], &k_inv_m_basis[a]));
            f_ritz[a * m_count + b] = val;
            f_ritz[b * m_count + a] = val;
        }
    }

    let (mu_vals, z_vecs) = jacobi_symmetric_eigen(&f_ritz, m_count, 80)?;
    let mut indices: Vec<usize> = (0..m_count).filter(|&k| mu_vals[k] > 1e-16).collect();
    indices.sort_by(|&a, &b| {
        mu_vals[b]
            .partial_cmp(&mu_vals[a])
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut modes = Vec::with_capacity(indices.len());
    for &idx in &indices {
        let omega_sq = 1.0 / mu_vals[idx];
        let mut phi = vec![0.0; n];
        for b in 0..m_count {
            let zb = z_vecs[b * m_count + idx];
            for i in 0..n {
                phi[i] += zb * basis[b][i];
            }
        }
        modes.push((omega_sq, phi));
    }

    Ok(modes)
}

/// Solves the linear buckling eigenvalue problem $(\mathbf{K} + \lambda \mathbf{K}_G)\boldsymbol{\phi} = \mathbf{0}$,
/// i.e. $\mathbf{K} \boldsymbol{\phi} = \lambda (-\mathbf{K}_G) \boldsymbol{\phi}$ (`BuckLin_SpacemSBR`).
///
/// Returns critical buckling load factors $\lambda_{cr}$ sorted by ascending $|\lambda_{cr}|$.
pub fn solve_buckling_eigen(
    ldlt: &SymmetricLdlt,
    kg: &[f64],
    num_modes: usize,
) -> Result<Vec<(f64, Vec<f64>)>, LinalgError> {
    let n = ldlt.n;
    let active_dofs: Vec<usize> = (0..n)
        .filter(|&i| !ldlt.zero_dof[i] && ldlt.d[i] > 1e-18)
        .collect();
    let na = active_dofs.len();
    if na == 0 || num_modes == 0 {
        return Ok(Vec::new());
    }

    // Using L D L^T = K, let L_c = L * D^{1/2} so K = L_c * L_c^T.
    // Then K * phi = lambda * (-K_G) * phi  <=>  (L_c^{-1} (-K_G) L_c^{-T}) y = (1 / lambda) y
    // where L_c^{-T} e_k is solved via L^T * z_k = e_k / sqrt(D_k).
    // Even simpler: compute Z_j = K^{-1} e_j for active DOFs, or form H = L_c^{-1} (-K_G) L_c^{-T}.
    let mut inv_lt_cols: Vec<Vec<f64>> = Vec::with_capacity(na);
    for &dof_k in &active_dofs {
        // Solve L^T * x = e_{dof_k} / sqrt(D_{dof_k})
        let mut x = vec![0.0; n];
        x[dof_k] = 1.0 / ldlt.d[dof_k].sqrt();
        for i in (0..=dof_k).rev() {
            if ldlt.zero_dof[i] {
                x[i] = 0.0;
                continue;
            }
            let mut sum = x[i];
            for j in (i + 1)..=dof_k {
                let l_ji = ldlt.l[j * n + i];
                if l_ji != 0.0 {
                    sum -= l_ji * x[j];
                }
            }
            x[i] = sum;
        }
        inv_lt_cols.push(x);
    }

    // Form symmetric matrix H_{ab} = - col_a^T * K_G * col_b
    let mut h_mat = vec![0.0; na * na];
    for b in 0..na {
        // w = -K_G * col_b
        let col_b = &inv_lt_cols[b];
        let mut w = vec![0.0; n];
        for i in 0..n {
            let mut s = 0.0;
            for j in 0..n {
                let kg_ij = kg[i * n + j];
                if kg_ij != 0.0 {
                    s -= kg_ij * col_b[j];
                }
            }
            w[i] = s;
        }
        for a in 0..=b {
            let col_a = &inv_lt_cols[a];
            let val: f64 = col_a.iter().zip(w.iter()).map(|(&ca, &wi)| ca * wi).sum();
            h_mat[a * na + b] = val;
            h_mat[b * na + a] = val;
        }
    }

    let (mu_vals, y_vecs) = jacobi_symmetric_eigen(&h_mat, na, 80)?;

    // Filter non-zero mu = 1 / lambda and sort by descending |mu| (ascending |lambda|)
    let mut max_mu = 0.0f64;
    for &m in &mu_vals {
        max_mu = max_mu.max(m.abs());
    }
    let mu_tol = (max_mu * 1e-10).max(1e-16);

    let mut indices: Vec<usize> = (0..na).filter(|&k| mu_vals[k].abs() > mu_tol).collect();
    // Prioritize positive buckling factors first, sorted by ascending |lambda| (descending |mu|)
    indices.sort_by(|&a, &b| {
        let ma = mu_vals[a];
        let mb = mu_vals[b];
        let pos_a = ma > 0.0;
        let pos_b = mb > 0.0;
        match (pos_a, pos_b) {
            (true, false) => std::cmp::Ordering::Less,
            (false, true) => std::cmp::Ordering::Greater,
            _ => mb
                .abs()
                .partial_cmp(&ma.abs())
                .unwrap_or(std::cmp::Ordering::Equal),
        }
    });

    let num_keep = indices.len().min(num_modes);
    let mut results = Vec::with_capacity(num_keep);
    for &idx in indices.iter().take(num_keep) {
        let lambda = 1.0 / mu_vals[idx];
        let mut phi = vec![0.0; n];
        for b in 0..na {
            let yb = y_vecs[b * na + idx];
            for i in 0..n {
                phi[i] += yb * inv_lt_cols[b][i];
            }
        }
        let max_v = phi.iter().fold(0.0f64, |acc, &v| acc.max(v.abs()));
        if max_v > 1e-20 {
            for v in &mut phi {
                *v /= max_v;
            }
        }
        results.push((lambda, phi));
    }

    Ok(results)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A bar (1e6) from DOF 0 to DOF 1, a ground spring at DOF 0, and DOF 2
    /// tied to DOF 1 by a 1e12 penalty. DOF 2 has no stiffness of its own, so
    /// its diagonal is all penalty while its pivot is the true stiffness seen
    /// there. Returns the matrix and the structural (penalty-free) diagonal.
    fn penalty_tied(ground_spring: f64) -> (Vec<f64>, Vec<f64>) {
        let (bar, penalty) = (1.0e6, 1.0e12);
        let k = vec![
            bar + ground_spring,
            -bar,
            0.0,
            -bar,
            bar + penalty,
            -penalty,
            0.0,
            -penalty,
            penalty,
        ];
        (k, vec![bar + ground_spring, bar, 0.0])
    }

    #[test]
    fn a_sub_matrix_pivot_index_is_mapped_or_never_a_global_index() {
        // Rows 0, 1, 2 of a sub-matrix are the global DOFs 7, 12, 30.
        let dofs = [7usize, 12, 30];
        assert!(matches!(
            remap_singular_index(LinalgError::SingularMatrix(1), &dofs),
            LinalgError::SingularMatrix(12)
        ));
        // An index outside the list cannot be a DOF of the full matrix.
        assert!(matches!(
            remap_singular_index(LinalgError::SingularMatrix(3), &dofs),
            LinalgError::SingularReduced(3)
        ));
        // Other errors pass through unchanged.
        assert!(matches!(
            remap_singular_index(LinalgError::NonFinite("x"), &dofs),
            LinalgError::NonFinite("x")
        ));
        // The dense solver of auxiliary matrices reports `SingularReduced`, so
        // its index is never taken for a joint DOF.
        let singular = [1.0, 1.0, 1.0, 1.0];
        assert!(matches!(
            solve_lu(&singular, &[1.0, 2.0], 2),
            Err(LinalgError::SingularReduced(_))
        ));
        // A massless-DOF block whose second row is a rigid-body mode: the
        // factorization failure is reported at the global DOF 12.
        let block = [1.0, -1.0, -1.0, 1.0];
        let error = SymmetricLdlt::factorize(&block, 2)
            .map_err(|e| remap_singular_index(e, &[7, 12]))
            .unwrap_err();
        assert!(matches!(error, LinalgError::SingularMatrix(12)), "{error}");
    }

    /// K = [[2, -1], [-1, 2]] has D = (2, 2), S = K / 2 with the eigenpairs
    /// 1/2 for (1, 1)/sqrt2 and 3/2 for (1, -1)/sqrt2. The start vector is
    /// the lowest eigenvector, so mu = 1/2 exactly; the free-DOF mask leaves
    /// the third DOF (restrained, unit diagonal) out.
    #[test]
    fn scaled_stiffness_minimum_of_a_two_dof_chain_is_one_half() {
        let k = [2.0, -1.0, 0.0, -1.0, 2.0, 0.0, 0.0, 0.0, 1.0];
        let ldlt = SymmetricLdlt::factorize(&k, 3).unwrap();
        let found = scaled_stiffness_minimum(&k, 3, &ldlt, None, &[false, false, true])
            .unwrap()
            .unwrap();
        assert!((found.mu - 0.5).abs() < 1e-14, "{}", found.mu);
        assert!(found.worst < 2);
        // Nothing free: no result.
        assert!(scaled_stiffness_minimum(&k, 3, &ldlt, None, &[true; 3])
            .unwrap()
            .is_none());
    }

    #[test]
    fn penalty_inflated_diagonals_do_not_hide_or_invent_mechanisms() {
        // Grounded by a spring of 2: the true stiffness at DOF 2 is about 2.
        let (k, structural) = penalty_tied(2.0);
        // Relative to its penalty diagonal that pivot looks like lost stiffness.
        assert!(matches!(
            SymmetricLdlt::factorize(&k, 3),
            Err(LinalgError::SingularMatrix(2))
        ));
        let factor = SymmetricLdlt::factorize_with_reference(&k, 3, Some(&structural)).unwrap();
        let u = factor.solve(&[0.0, 0.0, 1.0]).unwrap();
        // Springs in series: 1/2 + 1/1e6 + 1/1e12.
        let expected = 0.5 + 1.0e-6 + 1.0e-12;
        assert!((u[2] - expected).abs() < 1e-4 * expected, "{}", u[2]);

        // With no path to ground the structure is a mechanism and is still
        // reported: the pivot falls below the roundoff floor of the penalty.
        let (k, structural) = penalty_tied(0.0);
        assert!(matches!(
            SymmetricLdlt::factorize_with_reference(&k, 3, Some(&structural)),
            Err(LinalgError::SingularMatrix(_))
        ));
        // A reference of the wrong length is an error, not a silent fallback.
        assert!(SymmetricLdlt::factorize_with_reference(&k, 3, Some(&structural[..2])).is_err());
    }
    /// Free massless DOF `s`, masses `m1`, `m2`, a restrained DOF (unit
    /// diagonal, no mass) and a zero-stiffness DOF, in the order
    /// `[s, m1, R, m2, Z]`. Springs: ground-`s` 2, `s`-`m1` 2, `m1`-`m2` 1.
    /// Eliminating `s` leaves ground spring 1 at `m1`, so the unit-mass pencil
    /// is [[2, -1], [-1, 1]] with lambda = (3 -+ sqrt 5) / 2.
    fn mixed_dof_system() -> (Vec<f64>, Vec<f64>) {
        let n = 5;
        let mut k = vec![0.0; n * n];
        let mut set = |i: usize, j: usize, v: f64| {
            k[i * n + j] = v;
            k[j * n + i] = v;
        };
        set(0, 0, 4.0);
        set(0, 1, -2.0);
        set(1, 1, 3.0);
        set(1, 3, -1.0);
        set(3, 3, 1.0);
        set(2, 2, 1.0);
        (k, vec![0.0, 1.0, 0.0, 1.0, 0.0])
    }

    #[test]
    fn sturm_count_ignores_restrained_zero_and_massless_dofs() {
        let (k, mass) = mixed_dof_system();
        let ldlt = SymmetricLdlt::factorize(&k, 5).unwrap();
        assert!(ldlt.zero_dof[4] && !ldlt.zero_dof[2]);
        let lambda = [(3.0 - 5.0_f64.sqrt()) / 2.0, (3.0 + 5.0_f64.sqrt()) / 2.0];
        for (sigma, expected) in [
            (0.1, 0),
            (lambda[0] * (1.0 - 1e-6), 0),
            (lambda[0] * (1.0 + 1e-6), 1),
            (1.0, 1),
            (lambda[1] * (1.0 - 1e-6), 1),
            (lambda[1] * (1.0 + 1e-6), 2),
            (50.0, 2),
        ] {
            let negative = shifted_negative_pivots(&k, &ldlt, &mass, sigma)
                .ok()
                .unwrap_or(usize::MAX);
            assert_eq!(negative, expected, "sigma = {sigma}");
        }
        // The condensation solver sees the same two eigenvalues.
        let modes = solve_modal_eigen(&ldlt, &mass, 5).unwrap();
        assert_eq!(modes.len(), 2);
        assert!((modes[0].0 - lambda[0]).abs() < 1e-12);
        assert!((modes[1].0 - lambda[1]).abs() < 1e-12);
    }

    #[test]
    fn sturm_count_includes_a_massless_dof_eliminated_after_its_mass() {
        // Order [m1, s, m2, R, Z]: the massless DOF s comes after the mass m1
        // it is tied to. Same springs as `mixed_dof_system`
        // (ground-s 2, s-m1 2, m1-m2 1), so lambda = (3 -+ sqrt 5) / 2.
        let n = 5;
        let mut k = vec![0.0; n * n];
        let mut set = |i: usize, j: usize, v: f64| {
            k[i * n + j] = v;
            k[j * n + i] = v;
        };
        set(0, 0, 3.0);
        set(0, 1, -2.0);
        set(0, 2, -1.0);
        set(1, 1, 4.0);
        set(2, 2, 1.0);
        set(3, 3, 1.0);
        let mass = vec![1.0, 0.0, 1.0, 0.0, 0.0];
        let ldlt = SymmetricLdlt::factorize(&k, n).unwrap();
        assert!(ldlt.zero_dof[4]);
        // sigma = 2.9 lies above both eigenvalues (0.382 and 2.618). Hand
        // elimination: pivot m1 = 3 - 2.9 = 0.1; pivot s = 4 - 2^2 / 0.1 = -36
        // (negative, on the massless row); L(m2, m1) = -1 / 0.1 = -10,
        // L(s, m1) = -2 / 0.1 = -20, L(m2, s) = (0 - (-10)(-20)(0.1)) / -36
        // = 20 / 36, pivot m2 = (1 - 2.9) - 100 * 0.1 + (20/36)^2 * 36
        // = -11.9 + 400 / 36 = -0.7888...
        let pivots = shifted_pivots(&k, &ldlt, &mass, 2.9).unwrap();
        assert!((pivots[0] - 0.1).abs() < 1e-12);
        assert!(
            (pivots[1] + 36.0).abs() < 1e-9,
            "massless pivot {}",
            pivots[1]
        );
        assert!(
            (pivots[2] - (-11.9 + 400.0 / 36.0)).abs() < 1e-9,
            "m2 pivot {}",
            pivots[2]
        );
        assert_eq!(pivots[3], 1.0);
        assert_eq!(pivots[4], 1.0);
        // Two eigenvalues below 2.9: the count over all equations is 2 even
        // though the pivot of the massless row is one of the negative ones and
        // the first mass row is positive.
        let count = |sigma: f64| shifted_negative_pivots(&k, &ldlt, &mass, sigma).ok();
        assert_eq!(count(2.9), Some(2));
        assert_eq!(count(1.0), Some(1));
        assert_eq!(count(0.1), Some(0));
        let lambda = [(3.0 - 5.0_f64.sqrt()) / 2.0, (3.0 + 5.0_f64.sqrt()) / 2.0];
        assert_eq!(count(lambda[0] * (1.0 + 1e-6)), Some(1));
        assert_eq!(count(lambda[1] * (1.0 - 1e-6)), Some(1));
        assert_eq!(count(lambda[1] * (1.0 + 1e-6)), Some(2));
    }

    #[test]
    fn sturm_check_rejects_a_subspace_that_missed_a_mode() {
        // 450-DOF fixed-fixed chain (k = m = 1) beside 12 oscillators with
        // m = 1000, k = 1 + 0.01 i. The antisymmetric chain modes are
        // orthogonal to M*1 and no unit starting vector reaches the chain with
        // q = 11, so the first attempt converges to {B1, B3, A1} and misses B2
        // (lambda_2 = 4 sin^2(2 pi / 902) < every oscillator eigenvalue ~1e-3).
        let (chain, a_count) = (450usize, 12usize);
        let n = chain + a_count;
        let mut k = vec![0.0; n * n];
        let mut m = vec![1.0; n];
        for i in 0..chain {
            k[i * n + i] = 2.0;
            if i + 1 < chain {
                k[i * n + i + 1] = -1.0;
                k[(i + 1) * n + i] = -1.0;
            }
        }
        for a in 0..a_count {
            k[(chain + a) * n + chain + a] = 1.0 + 0.01 * a as f64;
            m[chain + a] = 1000.0;
        }
        let ldlt = SymmetricLdlt::factorize(&k, n).unwrap();
        let mass_dofs: Vec<usize> = (0..n).collect();
        let attempt = subspace_attempt(&k, &ldlt, &m, &mass_dofs, 3, 11);
        assert!(
            matches!(attempt, Err(LinalgError::EigenSturmMismatch { .. })),
            "first attempt must not return a set with a missing mode: {:?}",
            attempt
                .as_ref()
                .map(|modes| modes.iter().map(|m| m.0).collect::<Vec<_>>())
        );
        // With q doubled the chain is reached and the answer is exact.
        let retry = subspace_attempt(&k, &ldlt, &m, &mass_dofs, 3, 22).unwrap();
        let pi = std::f64::consts::PI;
        for (j, (lambda, _)) in retry.iter().enumerate() {
            let want = 4.0
                * ((j + 1) as f64 * pi / (2.0 * (chain as f64 + 1.0)))
                    .sin()
                    .powi(2);
            assert!((lambda - want).abs() <= 1e-9 * want, "{lambda} vs {want}");
        }
    }
}
