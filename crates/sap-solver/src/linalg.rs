//! Linear algebra kernels for `sap-solver`, corresponding to `csigo2_a.dll` (`Csi_sFire_*`),
//! `csigo2_d.dll` (`SFire2`), and `csigo2.dll` (`Eigen_SpacemSBR`, `Ritz_SpacemSBR`, `BuckLin_SpacemSBR`).

use thiserror::Error;

#[derive(Debug, Error, Clone)]
pub enum LinalgError {
    #[error("Matrix dimension mismatch: expected {expected}, got {actual}")]
    DimensionMismatch { expected: usize, actual: usize },
    #[error("Singular matrix encountered at pivot index {0}")]
    SingularMatrix(usize),
    #[error("Eigenvalue solver failed to converge within {0} sweeps")]
    EigenConvergence(usize),
    #[error("Nonfinite value encountered in {0}")]
    NonFinite(&'static str),
}

/// Active-column / dense $LDL^T$ factorization of a symmetric matrix $\mathbf{K} = \mathbf{L}\mathbf{D}\mathbf{L}^T$.
///
/// Tracks the Sturm sequence negative pivot count (`sturm_count`) for eigenvalue bounds and
/// stability diagnostics (`Csi_sFire_nSturm`), and automatically stabilizes zero-stiffness DOFs.
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

        for j in 0..n {
            if zero_dof[j] {
                d[j] = 1.0;
                continue;
            }
            let mut dj = k[j * n + j];
            for p in 0..j {
                let l_jp = l[j * n + p];
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
            let scale = match reference.map(|r| r[j]) {
                Some(structural) if structural > 0.0 => orig_diag[j].abs().min(structural),
                // A DOF whose whole diagonal is penalty (for example the
                // rotation of a solid joint tied into a body constraint) has no
                // structural scale; only the roundoff floor applies to it.
                Some(_) => 0.0,
                None => orig_diag[j].abs(),
            };
            let roundoff_floor = 64.0 * f64::EPSILON * orig_diag[j].abs();
            let rel_pivot_tol = (scale * 1e-11).max(if reference.is_some() {
                roundoff_floor
            } else {
                0.0
            });
            if dj <= rel_pivot_tol {
                return Err(LinalgError::SingularMatrix(j));
            }

            d[j] = dj;
            let inv_dj = 1.0 / dj;

            for i in (j + 1)..n {
                if zero_dof[i] {
                    continue;
                }
                let mut sum = k[i * n + j];
                for p in 0..j {
                    let l_ip = l[i * n + p];
                    let l_jp = l[j * n + p];
                    if l_ip != 0.0 && l_jp != 0.0 {
                        sum -= l_ip * l_jp * d[p];
                    }
                }
                l[i * n + j] = sum * inv_dj;
            }
        }

        Ok(Self {
            n,
            l,
            d,
            zero_dof,
            sturm_count: 0,
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
            for j in 0..i {
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
            for j in (i + 1)..n {
                let l_ji = self.l[j * n + i];
                if l_ji != 0.0 {
                    sum -= l_ji * x[j];
                }
            }
            x[i] = sum;
        }

        if x.iter().any(|v| !v.is_finite()) {
            return Err(LinalgError::NonFinite("solution vector"));
        }
        Ok(x)
    }
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
            return Err(LinalgError::SingularMatrix(k));
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
}
