//! 8-Node 3D Isoparametric Solid (Brick) Element Engine (CSI Chapter XIII).
//!
//! Implements the SAP2000 8-node hexahedral solid element with:
//! - Standard trilinear isoparametric shape functions $N_i(\xi_1, \xi_2, \xi_3)$
//!   using CSI binary corner ordering (`(-1,-1,-1)` through `(+1,+1,+1)`).
//! - Optional 9 Wilson-Taylor-Beresford (1976) incompatible bending modes
//!   $P_m(\xi_m) = 1 - \xi_m^2$ ($m = 1, 2, 3$) evaluated with the center-Jacobian
//!   patch-test modification $\nabla_{\mathbf{x}} P_m = \frac{\det \mathbf{J}_0}{\det \mathbf{J}} \mathbf{J}_0^{-T} \nabla_{\boldsymbol{\xi}} P_m$
//!   and statically condensed at the element level ($24 \times 24$ condensed stiffness).
//! - Full 3D isotropic and orthotropic constitutive law $\mathbf{C} = \mathbf{S}^{-1}$
//!   with 3D material coordinate rotation (`mat_angle_deg = [a, b, c]`) and
//!   6-component property stiffness modifiers (`modifiers[0..6]`).
//! - Consistent $2 \times 2 \times 2$ Gauss-Legendre integration for stiffness,
//!   equal-weight lumped nodal mass (with `modifiers[6]`), self-weight body forces
//!   (with `modifiers[7]`), 4-node isoparametric surface pressure loads (`Face 1..6`,
//!   positive along inward normal), and uniform temperature loads ($\Delta T$).
//! - Complete stress and internal nodal force recovery (`recover_solid_result`)
//!   including element-center, Gauss-point, and extrapolated corner stresses,
//!   principal stresses ($\sigma_{\max} \ge \sigma_{\text{mid}} \ge \sigma_{\min}$),
//!   and von Mises stress.

use sap_model::{Material, SolidObj, SolidSection};
pub use sap_model::{SolidResult, SolidStress};

use crate::linalg::{invert_matrix, jacobi_symmetric_eigen, solve_lu};
use crate::SolverError;

/// Natural coordinates $(\xi_1, \xi_2, \xi_3) \in \{-1, +1\}^3$ of the 8 solid nodes
/// following CSI Analysis Reference Manual Chapter XIII Table 29:
/// - Node 0 (`j1`): `(-1, -1, -1)`
/// - Node 1 (`j2`): `(+1, -1, -1)`
/// - Node 2 (`j3`): `(-1, +1, -1)`
/// - Node 3 (`j4`): `(+1, +1, -1)`
/// - Node 4 (`j5`): `(-1, -1, +1)`
/// - Node 5 (`j6`): `(+1, -1, +1)`
/// - Node 6 (`j7`): `(-1, +1, +1)`
/// - Node 7 (`j8`): `(+1, +1, +1)`
pub const SOLID_NODE_XI: [[f64; 3]; 8] = [
    [-1.0, -1.0, -1.0],
    [1.0, -1.0, -1.0],
    [-1.0, 1.0, -1.0],
    [1.0, 1.0, -1.0],
    [-1.0, -1.0, 1.0],
    [1.0, -1.0, 1.0],
    [-1.0, 1.0, 1.0],
    [1.0, 1.0, 1.0],
];

/// $2 \times 2 \times 2$ Gauss-Legendre quadrature abscissae ($\pm 1 / \sqrt{3}$)
/// ordered in the same binary sequence as [`SOLID_NODE_XI`].
const INV_SQRT3: f64 = 0.577_350_269_189_625_8;

/// $2 \times 2 \times 2$ Gauss points in natural coordinates $(\xi_1, \xi_2, \xi_3)$.
pub const SOLID_GAUSS_POINTS: [[f64; 3]; 8] = [
    [-INV_SQRT3, -INV_SQRT3, -INV_SQRT3],
    [INV_SQRT3, -INV_SQRT3, -INV_SQRT3],
    [-INV_SQRT3, INV_SQRT3, -INV_SQRT3],
    [INV_SQRT3, INV_SQRT3, -INV_SQRT3],
    [-INV_SQRT3, -INV_SQRT3, INV_SQRT3],
    [INV_SQRT3, -INV_SQRT3, INV_SQRT3],
    [-INV_SQRT3, INV_SQRT3, INV_SQRT3],
    [INV_SQRT3, INV_SQRT3, INV_SQRT3],
];

/// Construct a [`SolidStress`] from the 6 stress components
/// $[\sigma_{11}, \sigma_{22}, \sigma_{33}, \tau_{12}, \tau_{13}, \tau_{23}]$,
/// computing principal stresses ($\sigma_{\max} \ge \sigma_{\text{mid}} \ge \sigma_{\min}$)
/// and the von Mises equivalent stress.
pub fn solid_stress_from_components(sig: [f64; 6]) -> SolidStress {
    let [s11, s22, s33, s12, s13, s23] = sig;
    let tensor = [s11, s12, s13, s12, s22, s23, s13, s23, s33];
    let (mut eig, _) =
        jacobi_symmetric_eigen(&tensor, 3, 50).unwrap_or_else(|_| (vec![s11, s22, s33], vec![]));
    // Sort descending: s_max >= s_mid >= s_min
    if eig[0] < eig[1] {
        eig.swap(0, 1);
    }
    if eig[1] < eig[2] {
        eig.swap(1, 2);
    }
    if eig[0] < eig[1] {
        eig.swap(0, 1);
    }
    let (s_max, s_mid, s_min) = (eig[0], eig[1], eig[2]);
    let d12 = s_max - s_mid;
    let d23 = s_mid - s_min;
    let d31 = s_min - s_max;
    let von_mises = (0.5 * (d12 * d12 + d23 * d23 + d31 * d31)).sqrt();
    SolidStress {
        s11,
        s22,
        s33,
        s12,
        s13,
        s23,
        s_max,
        s_mid,
        s_min,
        von_mises,
    }
}

/// Evaluate the 8 trilinear shape functions $N_a(\xi_1, \xi_2, \xi_3)$ and their
/// natural-coordinate derivatives $\frac{\partial N_a}{\partial \xi_j}$ ($a = 0..8, j = 0..3$).
pub fn solid_shape_functions(xi1: f64, xi2: f64, xi3: f64) -> ([f64; 8], [[f64; 3]; 8]) {
    let mut n = [0.0; 8];
    let mut d_n = [[0.0; 3]; 8];
    for i in 0..8 {
        let [s1, s2, s3] = SOLID_NODE_XI[i];
        let f1 = 1.0 + s1 * xi1;
        let f2 = 1.0 + s2 * xi2;
        let f3 = 1.0 + s3 * xi3;
        n[i] = 0.125 * f1 * f2 * f3;
        d_n[i][0] = 0.125 * s1 * f2 * f3;
        d_n[i][1] = 0.125 * f1 * s2 * f3;
        d_n[i][2] = 0.125 * f1 * f2 * s3;
    }
    (n, d_n)
}

/// Compute the $3 \times 3$ coordinate Jacobian $J_{ij} = \frac{\partial x_i}{\partial \xi_j}$,
/// its determinant $\det \mathbf{J}$, and its inverse $\mathbf{J}^{-1}$ at a point with
/// natural derivatives `d_n`.
pub fn solid_jacobian(
    coords: &[[f64; 3]; 8],
    d_n: &[[f64; 3]; 8],
) -> ([[f64; 3]; 3], f64, [[f64; 3]; 3]) {
    let mut j = [[0.0; 3]; 3];
    for a in 0..8 {
        for r in 0..3 {
            for c in 0..3 {
                j[r][c] += coords[a][r] * d_n[a][c];
            }
        }
    }
    let det = j[0][0] * (j[1][1] * j[2][2] - j[1][2] * j[2][1])
        - j[0][1] * (j[1][0] * j[2][2] - j[1][2] * j[2][0])
        + j[0][2] * (j[1][0] * j[2][1] - j[1][1] * j[2][0]);

    let mut inv = [[0.0; 3]; 3];
    if det.abs() > 1e-30 {
        let inv_det = 1.0 / det;
        inv[0][0] = (j[1][1] * j[2][2] - j[1][2] * j[2][1]) * inv_det;
        inv[0][1] = (j[0][2] * j[2][1] - j[0][1] * j[2][2]) * inv_det;
        inv[0][2] = (j[0][1] * j[1][2] - j[0][2] * j[1][1]) * inv_det;

        inv[1][0] = (j[1][2] * j[2][0] - j[1][0] * j[2][2]) * inv_det;
        inv[1][1] = (j[0][0] * j[2][2] - j[0][2] * j[2][0]) * inv_det;
        inv[1][2] = (j[0][2] * j[1][0] - j[0][0] * j[1][2]) * inv_det;

        inv[2][0] = (j[1][0] * j[2][1] - j[1][1] * j[2][0]) * inv_det;
        inv[2][1] = (j[0][1] * j[2][0] - j[0][0] * j[2][1]) * inv_det;
        inv[2][2] = (j[0][0] * j[1][1] - j[0][1] * j[1][0]) * inv_det;
    }
    (j, det, inv)
}

/// Validate the 8-node solid geometry by checking that $\det \mathbf{J} > 0$ at the
/// natural origin $(0,0,0)$ and at all 8 Gauss integration points.
/// Returns the element volume $V = \sum_{g=1}^8 \det \mathbf{J}(\boldsymbol{\xi}_g)$.
pub fn validate_solid_geometry(coords: &[[f64; 3]; 8]) -> Result<f64, SolverError> {
    let (_, d_n0) = solid_shape_functions(0.0, 0.0, 0.0);
    let (_, det0, _) = solid_jacobian(coords, &d_n0);
    if !det0.is_finite() || det0 <= 1e-24 {
        return Err(SolverError::InvalidInput(format!(
            "degenerate or inverted 8-node solid element at center (det J0 = {det0:.6e})"
        )));
    }
    let mut volume = 0.0;
    for (idx, &[xi1, xi2, xi3]) in SOLID_GAUSS_POINTS.iter().enumerate() {
        let (_, d_n) = solid_shape_functions(xi1, xi2, xi3);
        let (_, det, _) = solid_jacobian(coords, &d_n);
        if !det.is_finite() || det <= 1e-24 {
            return Err(SolverError::InvalidInput(format!(
                "degenerate or inverted 8-node solid element at Gauss point {idx} (det J = {det:.6e})"
            )));
        }
        volume += det;
    }
    Ok(volume)
}

/// Build the $3 \times 3$ direction-cosine matrix $\mathbf{R}$ mapping local material axes
/// $(1, 2, 3)$ to global Cartesian axes $(X, Y, Z)$ (`x_global = R * x_material`) from
/// `mat_angle_deg = [a, b, c]` (in degrees):
/// - `a`: rotation about global $Z$
/// - `b`: rotation about intermediate $Y'$
/// - `c`: rotation about local $X''$
pub fn solid_material_rotation_matrix(mat_angle_deg: [f64; 3]) -> [[f64; 3]; 3] {
    let a = mat_angle_deg[0].to_radians();
    let b = mat_angle_deg[1].to_radians();
    let c = mat_angle_deg[2].to_radians();
    let (ca, sa) = (a.cos(), a.sin());
    let (cb, sb) = (b.cos(), b.sin());
    let (cc, sc) = (c.cos(), c.sin());

    // R = R_z(a) * R_y(b) * R_x(c)
    [
        [
            ca * cb,
            ca * sb * sc - sa * cc,
            ca * sb * cc + sa * sc,
        ],
        [
            sa * cb,
            sa * sb * sc + ca * cc,
            sa * sb * cc - ca * sc,
        ],
        [-sb, cb * sc, cb * cc],
    ]
}

/// Build the $6 \times 6$ Bond engineering-strain transformation matrix $\mathbf{T}_\varepsilon$
/// mapping global engineering strains $\boldsymbol{\varepsilon}_{\text{global}}$ to local
/// material engineering strains $\boldsymbol{\varepsilon}_{\text{local}} = \mathbf{T}_\varepsilon \boldsymbol{\varepsilon}_{\text{global}}$,
/// where strain ordering is $[\varepsilon_{11}, \varepsilon_{22}, \varepsilon_{33}, \gamma_{12}, \gamma_{13}, \gamma_{23}]^T$.
///
/// Since $\mathbf{x}_{\text{global}} = \mathbf{R} \mathbf{x}_{\text{local}}$, the local-from-global
/// direction cosines are $Q_{ij} = R_{ji}$ ($\mathbf{Q} = \mathbf{R}^T$).
pub fn bond_strain_transform_global_to_local(r_local_to_global: &[[f64; 3]; 3]) -> [[f64; 6]; 6] {
    // Q = R^T maps global vectors to local vectors: v_local = Q * v_global
    let mut q = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            q[i][j] = r_local_to_global[j][i];
        }
    }
    // Index pairs for shear rows/cols in CSI order: (0,1)=12, (0,2)=13, (1,2)=23
    let pairs = [(0usize, 1usize), (0usize, 2usize), (1usize, 2usize)];
    let mut t = [[0.0; 6]; 6];

    // Normal strain rows i = 0..3
    for i in 0..3 {
        for j in 0..3 {
            t[i][j] = q[i][j] * q[i][j];
        }
        for (col_idx, &(p, r)) in pairs.iter().enumerate() {
            t[i][3 + col_idx] = q[i][p] * q[i][r];
        }
    }

    // Engineering shear strain rows (gamma_{ab} = 2 * eps_{ab})
    for (row_idx, &(a, b)) in pairs.iter().enumerate() {
        let r = 3 + row_idx;
        for j in 0..3 {
            t[r][j] = 2.0 * q[a][j] * q[b][j];
        }
        for (col_idx, &(p, s)) in pairs.iter().enumerate() {
            t[r][3 + col_idx] = q[a][p] * q[b][s] + q[a][s] * q[b][p];
        }
    }
    t
}

/// Compute the unrotated $6 \times 6$ material constitutive matrix $\mathbf{C}_{\text{local}}$
/// (in material axes $1, 2, 3$ with stiffness modifiers applied) and local thermal strain vector
/// $\boldsymbol{\alpha}_{\text{local}} = [\alpha_1, \alpha_2, \alpha_3, 0, 0, 0]^T$.
pub fn solid_local_constitutive_matrix(
    mat: &Material,
    sec: &SolidSection,
) -> Result<([[f64; 6]; 6], [f64; 6]), SolverError> {
    let [e1, e2, e3] = mat.e_ortho.unwrap_or([mat.e, mat.e, mat.e]);
    if e1 <= 0.0 || e2 <= 0.0 || e3 <= 0.0 {
        return Err(SolverError::InvalidInput(format!(
            "material '{}' has non-positive Young's modulus ({e1}, {e2}, {e3})",
            mat.name
        )));
    }
    let [u12, u13, u23] = mat.u_ortho.unwrap_or([mat.u, mat.u, mat.u]);

    // Per CSI Reference Manual Chapter VI Eqn. 1: for isotropic materials,
    // G = E / (2 * (1 + u)) when explicit orthotropic moduli are not specified.
    let [g12, g13, g23] = match mat.g_ortho {
        Some(g_arr) => g_arr,
        None => {
            if mat.e_ortho.is_none() && mat.u_ortho.is_none() && (1.0 + mat.u).abs() > 1e-12 {
                let g_iso = mat.e / (2.0 * (1.0 + mat.u));
                [g_iso, g_iso, g_iso]
            } else if mat.g > 0.0 {
                [mat.g, mat.g, mat.g]
            } else {
                [
                    e1 / (2.0 * (1.0 + u12)),
                    e1 / (2.0 * (1.0 + u13)),
                    e2 / (2.0 * (1.0 + u23)),
                ]
            }
        }
    };
    if g12 <= 0.0 || g13 <= 0.0 || g23 <= 0.0 {
        return Err(SolverError::InvalidInput(format!(
            "material '{}' has non-positive shear modulus ({g12}, {g13}, {g23})",
            mat.name
        )));
    }

    // 3x3 normal compliance submatrix S_normal (symmetric per CSI Chapter VI Eqn. 2)
    let s_norm = [
        1.0 / e1,
        -u12 / e1,
        -u13 / e1,
        -u12 / e1,
        1.0 / e2,
        -u23 / e2,
        -u13 / e1,
        -u23 / e2,
        1.0 / e3,
    ];
    // Verify positive definiteness of S_normal before inverting
    let (eigs, _) = jacobi_symmetric_eigen(&s_norm, 3, 40)?;
    if eigs.iter().any(|&lam| lam <= 1e-18) {
        return Err(SolverError::InvalidInput(format!(
            "material '{}' compliance matrix is not positive definite (Poissons ratios {u12}, {u13}, {u23})",
            mat.name
        )));
    }
    let c_norm = invert_matrix(&s_norm, 3)?;

    let mut c_local = [[0.0; 6]; 6];
    for i in 0..3 {
        for j in 0..3 {
            c_local[i][j] = c_norm[3 * i + j];
        }
    }
    c_local[3][3] = g12;
    c_local[4][4] = g13;
    c_local[5][5] = g23;

    // Apply section stiffness modifiers (modifiers[0..6] correspond to C11, C22, C33, C12, C13, C23;
    // off-diagonal normal coupling terms are scaled by sqrt(m_i * m_j) to preserve symmetry and PD).
    let mut mods = [1.0; 6];
    for i in 0..6 {
        let m = sec.modifiers[i];
        mods[i] = if m > 0.0 { m } else { 1.0 };
    }
    for i in 0..3 {
        for j in 0..3 {
            let scale = if i == j {
                mods[i]
            } else {
                (mods[i] * mods[j]).sqrt()
            };
            c_local[i][j] *= scale;
        }
    }
    for k in 3..6 {
        c_local[k][k] *= mods[k];
    }

    let [a1, a2, a3] = mat.a_ortho.unwrap_or([mat.a, mat.a, mat.a]);
    let alpha_local = [a1, a2, a3, 0.0, 0.0, 0.0];
    Ok((c_local, alpha_local))
}

/// Compute the global $6 \times 6$ constitutive matrix $\mathbf{C}_{\text{global}} = \mathbf{T}_\varepsilon^T \mathbf{C}_{\text{local}} \mathbf{T}_\varepsilon$
/// and global thermal expansion strain vector $\boldsymbol{\alpha}_{\text{global}} = \mathbf{T}_\varepsilon^{-1} \boldsymbol{\alpha}_{\text{local}}$.
pub fn solid_constitutive_matrix(
    mat: &Material,
    sec: &SolidSection,
) -> Result<([[f64; 6]; 6], [f64; 6]), SolverError> {
    let (c_local, alpha_local) = solid_local_constitutive_matrix(mat, sec)?;
    if sec.mat_angle_deg.iter().all(|&ang| ang.abs() <= 1e-15) {
        return Ok((c_local, alpha_local));
    }

    let r_mat = solid_material_rotation_matrix(sec.mat_angle_deg);
    let t_eps = bond_strain_transform_global_to_local(&r_mat);

    // C_global = T_eps^T * C_local * T_eps
    let mut ct = [[0.0; 6]; 6];
    for i in 0..6 {
        for j in 0..6 {
            let mut s = 0.0;
            for k in 0..6 {
                s += c_local[i][k] * t_eps[k][j];
            }
            ct[i][j] = s;
        }
    }
    let mut c_global = [[0.0; 6]; 6];
    for i in 0..6 {
        for j in 0..6 {
            let mut s = 0.0;
            for k in 0..6 {
                s += t_eps[k][i] * ct[k][j];
            }
            c_global[i][j] = s;
        }
    }

    // For thermal strain: eps_local^th = T_eps * eps_global^th => eps_global^th = T_eps(R^T) * alpha_local
    let mut r_inv = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            r_inv[i][j] = r_mat[j][i];
        }
    }
    let t_eps_inv = bond_strain_transform_global_to_local(&r_inv);
    let mut alpha_global = [0.0; 6];
    for i in 0..6 {
        let mut s = 0.0;
        for j in 0..6 {
            s += t_eps_inv[i][j] * alpha_local[j];
        }
        alpha_global[i] = s;
    }

    Ok((c_global, alpha_global))
}

/// Populate the $6 \times 24$ compatible strain-displacement matrix $\mathbf{B}_u(\boldsymbol{\xi})$
/// from Cartesian shape function gradients $\nabla_{\mathbf{x}} N_a = \mathbf{J}^{-T} \nabla_{\boldsymbol{\xi}} N_a$.
fn build_bu_matrix(d_n: &[[f64; 3]; 8], inv_j: &[[f64; 3]; 3]) -> [[f64; 24]; 6] {
    let mut b_u = [[0.0; 24]; 6];
    for a in 0..8 {
        // grad_x N_a[r] = sum_c inv_j[c][r] * d_n[a][c]
        let gx = inv_j[0][0] * d_n[a][0] + inv_j[1][0] * d_n[a][1] + inv_j[2][0] * d_n[a][2];
        let gy = inv_j[0][1] * d_n[a][0] + inv_j[1][1] * d_n[a][1] + inv_j[2][1] * d_n[a][2];
        let gz = inv_j[0][2] * d_n[a][0] + inv_j[1][2] * d_n[a][1] + inv_j[2][2] * d_n[a][2];
        let col = 3 * a;
        b_u[0][col] = gx;
        b_u[1][col + 1] = gy;
        b_u[2][col + 2] = gz;
        // gamma_12 = du/dy + dv/dx
        b_u[3][col] = gy;
        b_u[3][col + 1] = gx;
        // gamma_13 = du/dz + dw/dx
        b_u[4][col] = gz;
        b_u[4][col + 2] = gx;
        // gamma_23 = dv/dz + dw/dy
        b_u[5][col + 1] = gz;
        b_u[5][col + 2] = gy;
    }
    b_u
}

/// Populate the $6 \times 9$ Wilson-Taylor-Beresford (1976) incompatible-mode strain matrix
/// $\mathbf{B}_\alpha(\boldsymbol{\xi})$ using the center-Jacobian patch-test modification:
/// $$\nabla_{\mathbf{x}} P_m(\boldsymbol{\xi}) = \frac{\det \mathbf{J}_0}{\det \mathbf{J}(\boldsymbol{\xi})} \mathbf{J}_0^{-T} \nabla_{\boldsymbol{\xi}} P_m(\boldsymbol{\xi})$$
/// where $P_m(\boldsymbol{\xi}) = 1 - \xi_m^2$ has $\frac{\partial P_m}{\partial \xi_k} = -2 \xi_m \delta_{mk}$.
fn build_ba_matrix(
    xi: [f64; 3],
    det_j: f64,
    det_j0: f64,
    inv_j0: &[[f64; 3]; 3],
) -> [[f64; 9]; 6] {
    let mut b_a = [[0.0; 9]; 6];
    let ratio = det_j0 / det_j;
    for m in 0..3 {
        let d_pm_d_xim = -2.0 * xi[m] * ratio;
        let gx = inv_j0[m][0] * d_pm_d_xim;
        let gy = inv_j0[m][1] * d_pm_d_xim;
        let gz = inv_j0[m][2] * d_pm_d_xim;
        let col = 3 * m;
        b_a[0][col] = gx;
        b_a[1][col + 1] = gy;
        b_a[2][col + 2] = gz;
        b_a[3][col] = gy;
        b_a[3][col + 1] = gx;
        b_a[4][col] = gz;
        b_a[4][col + 2] = gx;
        b_a[5][col + 1] = gz;
        b_a[5][col + 2] = gy;
    }
    b_a
}

/// Internal helper computing $\mathbf{K}_{uu}$ ($24 \times 24$), $\mathbf{K}_{u\alpha}$ ($24 \times 9$),
/// $\mathbf{K}_{\alpha\alpha}$ ($9 \times 9$), and element volume $V$.
fn compute_solid_submatrices(
    coords: &[[f64; 3]; 8],
    c_global: &[[f64; 6]; 6],
    incompatible_modes: bool,
) -> Result<
    (
        [[f64; 24]; 24],
        [[f64; 9]; 24],
        [[f64; 9]; 9],
        f64,
        f64,
        [[f64; 3]; 3],
    ),
    SolverError,
> {
    let (_, d_n0) = solid_shape_functions(0.0, 0.0, 0.0);
    let (_, det_j0, inv_j0) = solid_jacobian(coords, &d_n0);
    if !det_j0.is_finite() || det_j0 <= 1e-24 {
        return Err(SolverError::InvalidInput(format!(
            "degenerate or inverted 8-node solid element at center (det J0 = {det_j0:.6e})"
        )));
    }

    let mut k_uu = [[0.0; 24]; 24];
    let mut k_ua = [[0.0; 9]; 24];
    let mut k_aa = [[0.0; 9]; 9];
    let mut volume = 0.0;

    for (gp_idx, &xi) in SOLID_GAUSS_POINTS.iter().enumerate() {
        let (_, d_n) = solid_shape_functions(xi[0], xi[1], xi[2]);
        let (_, det_j, inv_j) = solid_jacobian(coords, &d_n);
        if !det_j.is_finite() || det_j <= 1e-24 {
            return Err(SolverError::InvalidInput(format!(
                "degenerate or inverted 8-node solid element at Gauss point {gp_idx} (det J = {det_j:.6e})"
            )));
        }
        volume += det_j; // Gauss weights are 1 * 1 * 1 = 1

        let b_u = build_bu_matrix(&d_n, &inv_j);

        // c_bu = C * B_u (6 x 24) * det_j
        let mut c_bu = [[0.0; 24]; 6];
        for r in 0..6 {
            for c in 0..24 {
                let mut s = 0.0;
                for k in 0..6 {
                    s += c_global[r][k] * b_u[k][c];
                }
                c_bu[r][c] = s * det_j;
            }
        }

        // k_uu += B_u^T * (C * B_u * det_j)
        for i in 0..24 {
            for j in i..24 {
                let mut s = 0.0;
                for r in 0..6 {
                    s += b_u[r][i] * c_bu[r][j];
                }
                k_uu[i][j] += s;
            }
        }

        if incompatible_modes {
            let b_a = build_ba_matrix(xi, det_j, det_j0, &inv_j0);
            let mut c_ba = [[0.0; 9]; 6];
            for r in 0..6 {
                for c in 0..9 {
                    let mut s = 0.0;
                    for k in 0..6 {
                        s += c_global[r][k] * b_a[k][c];
                    }
                    c_ba[r][c] = s * det_j;
                }
            }
            for i in 0..24 {
                for j in 0..9 {
                    let mut s = 0.0;
                    for r in 0..6 {
                        s += b_u[r][i] * c_ba[r][j];
                    }
                    k_ua[i][j] += s;
                }
            }
            for i in 0..9 {
                for j in i..9 {
                    let mut s = 0.0;
                    for r in 0..6 {
                        s += b_a[r][i] * c_ba[r][j];
                    }
                    k_aa[i][j] += s;
                }
            }
        }
    }

    for i in 0..24 {
        for j in 0..i {
            k_uu[i][j] = k_uu[j][i];
        }
    }
    if incompatible_modes {
        for i in 0..9 {
            for j in 0..i {
                k_aa[i][j] = k_aa[j][i];
            }
        }
    }

    Ok((k_uu, k_ua, k_aa, volume, det_j0, inv_j0))
}

fn has_collapsed_nodes(coords: &[[f64; 3]; 8]) -> bool {
    for a in 0..8 {
        for b in (a + 1)..8 {
            let d = (coords[a][0] - coords[b][0])
                .hypot(coords[a][1] - coords[b][1])
                .hypot(coords[a][2] - coords[b][2]);
            if d <= 1e-12 {
                return true;
            }
        }
    }
    false
}

/// Compute the $24 \times 24$ element stiffness matrix (statically condensed if
/// `sec.incompatible_modes` is `true` and all 8 corners are distinct) and the 24-DOF diagonal lumped mass vector
/// `[M_1x, M_1y, M_1z, ..., M_8z]` for an 8-node solid element.
pub fn compute_solid_stiffness_and_mass(
    coords: &[[f64; 3]; 8],
    mat: &Material,
    sec: &SolidSection,
) -> Result<([[f64; 24]; 24], [f64; 24]), SolverError> {
    let use_incomp = sec.incompatible_modes && !has_collapsed_nodes(coords);
    let (c_global, _) = solid_constitutive_matrix(mat, sec)?;
    let (mut k_uu, k_ua, k_aa, volume, _, _) =
        compute_solid_submatrices(coords, &c_global, use_incomp)?;

    if use_incomp {
        let mut k_aa_flat = [0.0; 81];
        for i in 0..9 {
            for j in 0..9 {
                k_aa_flat[9 * i + j] = k_aa[i][j];
            }
        }
        let k_aa_inv = invert_matrix(&k_aa_flat, 9)?;

        // Condense: K_cond = K_uu - K_ua * K_aa^{-1} * K_ua^T
        let mut k_ua_inv = [[0.0; 9]; 24];
        for i in 0..24 {
            for j in 0..9 {
                let mut s = 0.0;
                for k in 0..9 {
                    s += k_ua[i][k] * k_aa_inv[9 * k + j];
                }
                k_ua_inv[i][j] = s;
            }
        }
        for i in 0..24 {
            for j in i..24 {
                let mut s = 0.0;
                for k in 0..9 {
                    s += k_ua_inv[i][k] * k_ua[j][k];
                }
                k_uu[i][j] -= s;
                k_uu[j][i] = k_uu[i][j];
            }
        }
    }

    // Equal-weight lumped nodal mass at each of the 8 nodes (CSI Chapter XIII:
    // total element mass = volume * unit_mass * mass_modifier, distributed 1/8 per node)
    let mass_mod = if sec.modifiers[6] > 0.0 {
        sec.modifiers[6]
    } else {
        1.0
    };
    let node_mass = 0.125 * volume * mat.unit_mass * mass_mod;
    let m_diag = [node_mass; 24];

    Ok((k_uu, m_diag))
}

/// Return the 4 corner node indices (0-indexed into `0..8`) of `face` (`1..=6`) ordered
/// counter-clockwise when viewed from OUTSIDE the solid element (so that the right-hand
/// cross product $\frac{\partial \mathbf{x}}{\partial \eta_1} \times \frac{\partial \mathbf{x}}{\partial \eta_2}$
/// points OUTWARD, and its negation points INWARD into the solid per CSI surface pressure convention).
///
/// CSI Solid Face Convention (Chapter XIII, Figure 55):
/// - Face 1 (`-2` face, $\xi_2 = -1$, nodes `j1, j2, j6, j5` = `[0, 1, 5, 4]`)
/// - Face 2 (`+1` face, $\xi_1 = +1$, nodes `j2, j4, j8, j6` = `[1, 3, 7, 5]`)
/// - Face 3 (`+2` face, $\xi_2 = +1$, nodes `j3, j7, j8, j4` = `[2, 6, 7, 3]`)
/// - Face 4 (`-1` face, $\xi_1 = -1$, nodes `j1, j5, j7, j3` = `[0, 4, 6, 2]`)
/// - Face 5 (`-3` face, $\xi_3 = -1$, nodes `j1, j3, j4, j2` = `[0, 2, 3, 1]`)
/// - Face 6 (`+3` face, $\xi_3 = +1$, nodes `j5, j6, j8, j7` = `[4, 5, 7, 6]`)
pub fn solid_face_nodes(face: i32) -> Result<[usize; 4], SolverError> {
    match face {
        1 => Ok([0, 1, 5, 4]),
        2 => Ok([1, 3, 7, 5]),
        3 => Ok([2, 6, 7, 3]),
        4 => Ok([0, 4, 6, 2]),
        5 => Ok([0, 2, 3, 1]),
        6 => Ok([4, 5, 7, 6]),
        _ => Err(SolverError::InvalidInput(format!(
            "invalid solid face index {face}; expected 1..=6"
        ))),
    }
}

/// Compute the consistent 24-DOF nodal force vector `[F_1x, F_1y, F_1z, ..., F_8z]`
/// for a given `load_pat` combining:
/// 1. Self-weight body force (`self_weight_mult * mat.unit_weight * weight_modifier` along `gravity_dir`).
/// 2. Surface pressure loads (`solid.loads_surface` matching `load_pat`, where positive `val`
///    acts toward the interior of the solid element along the inward unit normal).
/// 3. Uniform temperature change (`solid.loads_temp.get(load_pat)`:
///    $\mathbf{f}_{\text{temp}} = \int_\Omega \mathbf{B}_u^T \mathbf{C} \boldsymbol{\alpha} \Delta T \, d\Omega$).
pub fn compute_solid_nodal_loads(
    coords: &[[f64; 3]; 8],
    mat: &Material,
    sec: &SolidSection,
    solid: &SolidObj,
    load_pat: &str,
    self_weight_mult: f64,
    gravity_dir: [f64; 3],
) -> Result<[f64; 24], SolverError> {
    let volume = validate_solid_geometry(coords)?;
    let mut f_elem = [0.0; 24];

    // 1. Self-weight body forces
    if self_weight_mult.abs() > 0.0 && mat.unit_weight.abs() > 0.0 {
        let weight_mod = if sec.modifiers[7] > 0.0 {
            sec.modifiers[7]
        } else {
            1.0
        };
        let node_weight = 0.125 * volume * mat.unit_weight * weight_mod * self_weight_mult;
        for a in 0..8 {
            f_elem[3 * a] += node_weight * gravity_dir[0];
            f_elem[3 * a + 1] += node_weight * gravity_dir[1];
            f_elem[3 * a + 2] += node_weight * gravity_dir[2];
        }
    }

    // 2. Surface pressure loads (2x2 Gauss integration on each loaded quadrilateral face)
    let gp_1d = [-INV_SQRT3, INV_SQRT3];
    let quad_signs = [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]];
    for surf in &solid.loads_surface {
        if surf.load_pat != load_pat || surf.val.abs() == 0.0 {
            continue;
        }
        let fnodes = solid_face_nodes(surf.face)?;
        let p = surf.val;
        for &eta1 in &gp_1d {
            for &eta2 in &gp_1d {
                let mut n_2d = [0.0; 4];
                let mut dn_deta1 = [0.0; 4];
                let mut dn_deta2 = [0.0; 4];
                for k in 0..4 {
                    let [s1, s2] = quad_signs[k];
                    n_2d[k] = 0.25 * (1.0 + s1 * eta1) * (1.0 + s2 * eta2);
                    dn_deta1[k] = 0.25 * s1 * (1.0 + s2 * eta2);
                    dn_deta2[k] = 0.25 * (1.0 + s1 * eta1) * s2;
                }
                let mut dx_d1 = [0.0; 3];
                let mut dx_d2 = [0.0; 3];
                for k in 0..4 {
                    let c = coords[fnodes[k]];
                    for r in 0..3 {
                        dx_d1[r] += c[r] * dn_deta1[k];
                        dx_d2[r] += c[r] * dn_deta2[k];
                    }
                }
                // Outward area-weighted normal n_out = dx_d1 x dx_d2;
                // Inward area-weighted normal n_in = -n_out (since positive pressure pushes INTO the solid).
                let n_in = [
                    -(dx_d1[1] * dx_d2[2] - dx_d1[2] * dx_d2[1]),
                    -(dx_d1[2] * dx_d2[0] - dx_d1[0] * dx_d2[2]),
                    -(dx_d1[0] * dx_d2[1] - dx_d1[1] * dx_d2[0]),
                ];
                for k in 0..4 {
                    let node_idx = fnodes[k];
                    let w_p = p * n_2d[k];
                    f_elem[3 * node_idx] += w_p * n_in[0];
                    f_elem[3 * node_idx + 1] += w_p * n_in[1];
                    f_elem[3 * node_idx + 2] += w_p * n_in[2];
                }
            }
        }
    }

    // 3. Uniform temperature load
    if let Some(&delta_t) = solid.loads_temp.get(load_pat) {
        if delta_t.abs() > 0.0 {
            let (c_global, alpha_global) = solid_constitutive_matrix(mat, sec)?;
            // sigma_0 = C_global * alpha_global * delta_t
            let mut sig_th = [0.0; 6];
            for i in 0..6 {
                let mut s = 0.0;
                for j in 0..6 {
                    s += c_global[i][j] * alpha_global[j];
                }
                sig_th[i] = s * delta_t;
            }
            for &[xi1, xi2, xi3] in &SOLID_GAUSS_POINTS {
                let (_, d_n) = solid_shape_functions(xi1, xi2, xi3);
                let (_, det_j, inv_j) = solid_jacobian(coords, &d_n);
                let b_u = build_bu_matrix(&d_n, &inv_j);
                for i in 0..24 {
                    let mut s = 0.0;
                    for r in 0..6 {
                        s += b_u[r][i] * sig_th[r];
                    }
                    f_elem[i] += s * det_j;
                }
            }
        }
    }

    Ok(f_elem)
}

/// Assemble a $24 \times 24$ solid element stiffness matrix into a row-major global
/// stiffness matrix `k_global` of dimension `n_dof x n_dof`.
pub fn assemble_solid_into_global(
    k_global: &mut [f64],
    n_dof: usize,
    dof_map: &[Option<usize>; 24],
    k_elem: &[[f64; 24]; 24],
) {
    for i in 0..24 {
        if let Some(gi) = dof_map[i] {
            for j in 0..24 {
                if let Some(gj) = dof_map[j] {
                    k_global[gi * n_dof + gj] += k_elem[i][j];
                }
            }
        }
    }
}

/// Recover stresses (at the element center, 8 Gauss points, and 8 corner nodes in local
/// material axes) and internal elastic nodal forces (in global axes) for an 8-node solid
/// element given nodal displacements `u_elem` and uniform temperature change `delta_t`.
pub fn recover_solid_result(
    coords: &[[f64; 3]; 8],
    mat: &Material,
    sec: &SolidSection,
    u_elem: &[f64; 24],
    delta_t: f64,
) -> Result<SolidResult, SolverError> {
    let use_incomp = sec.incompatible_modes && !has_collapsed_nodes(coords);
    let (c_local, alpha_local) = solid_local_constitutive_matrix(mat, sec)?;
    let (c_global, alpha_global) = solid_constitutive_matrix(mat, sec)?;
    let r_mat = solid_material_rotation_matrix(sec.mat_angle_deg);
    let t_eps = bond_strain_transform_global_to_local(&r_mat);

    let (k_uu, k_ua, k_aa, _, det_j0, inv_j0) =
        compute_solid_submatrices(coords, &c_global, use_incomp)?;

    // Recover incompatible mode amplitudes alpha = -K_aa^{-1} * K_ua^T * u_elem
    let mut alpha_modes = [0.0; 9];
    if use_incomp {
        let mut rhs = [0.0; 9];
        for i in 0..9 {
            let mut s = 0.0;
            for j in 0..24 {
                s += k_ua[j][i] * u_elem[j];
            }
            rhs[i] = -s;
        }
        let mut k_aa_flat = [0.0; 81];
        for i in 0..9 {
            for j in 0..9 {
                k_aa_flat[9 * i + j] = k_aa[i][j];
            }
        }
        let sol = solve_lu(&k_aa_flat, &rhs, 9)?;
        alpha_modes.copy_from_slice(&sol);
    }

    // Helper closure to evaluate local material stress at arbitrary natural coordinates xi
    let eval_stress_at_xi = |xi: [f64; 3]| -> SolidStress {
        let (_, d_n) = solid_shape_functions(xi[0], xi[1], xi[2]);
        let (_, det_j, inv_j) = solid_jacobian(coords, &d_n);
        let b_u = build_bu_matrix(&d_n, &inv_j);

        let mut eps_global = [0.0; 6];
        for r in 0..6 {
            let mut s = 0.0;
            for c in 0..24 {
                s += b_u[r][c] * u_elem[c];
            }
            eps_global[r] = s;
        }
        if use_incomp {
            let b_a = build_ba_matrix(xi, det_j, det_j0, &inv_j0);
            for r in 0..6 {
                let mut s = 0.0;
                for c in 0..9 {
                    s += b_a[r][c] * alpha_modes[c];
                }
                eps_global[r] += s;
            }
        }

        // Transform total strain to local material axes and subtract local thermal strain
        let mut eps_local = [0.0; 6];
        for r in 0..6 {
            let mut s = 0.0;
            for c in 0..6 {
                s += t_eps[r][c] * eps_global[c];
            }
            eps_local[r] = s - alpha_local[r] * delta_t;
        }

        let mut sig_local = [0.0; 6];
        for r in 0..6 {
            let mut s = 0.0;
            for c in 0..6 {
                s += c_local[r][c] * eps_local[c];
            }
            sig_local[r] = s;
        }
        solid_stress_from_components(sig_local)
    };

    let center_stress = eval_stress_at_xi([0.0, 0.0, 0.0]);

    let mut gauss_raw = [[0.0; 6]; 8];
    let mut gauss_stresses = [SolidStress::default(); 8];
    for (g, &xi) in SOLID_GAUSS_POINTS.iter().enumerate() {
        let st = eval_stress_at_xi(xi);
        gauss_stresses[g] = st;
        gauss_raw[g] = [st.s11, st.s22, st.s33, st.s12, st.s13, st.s23];
    }

    // Extrapolate from the 8 Gauss points (+/- 1/sqrt(3)) to the 8 corner nodes (+/- 1)
    // using trilinear shape functions in scaled coordinates zeta = sqrt(3) * xi.
    let sqrt3 = 3.0_f64.sqrt();
    let mut nodal_stresses = [SolidStress::default(); 8];
    for node_idx in 0..8 {
        let [s1, s2, s3] = SOLID_NODE_XI[node_idx];
        let (n_extrap, _) = solid_shape_functions(s1 * sqrt3, s2 * sqrt3, s3 * sqrt3);
        let mut sig_node = [0.0; 6];
        for g in 0..8 {
            for comp in 0..6 {
                sig_node[comp] += n_extrap[g] * gauss_raw[g][comp];
            }
        }
        nodal_stresses[node_idx] = solid_stress_from_components(sig_node);
    }

    // Internal nodal forces: f_int = K_uu * u + K_ua * alpha - f_thermal
    let mut f_int = [0.0; 24];
    for i in 0..24 {
        let mut s = 0.0;
        for j in 0..24 {
            s += k_uu[i][j] * u_elem[j];
        }
        if use_incomp {
            for j in 0..9 {
                s += k_ua[i][j] * alpha_modes[j];
            }
        }
        f_int[i] = s;
    }
    if delta_t.abs() > 0.0 {
        let mut sig_th = [0.0; 6];
        for i in 0..6 {
            let mut s = 0.0;
            for j in 0..6 {
                s += c_global[i][j] * alpha_global[j];
            }
            sig_th[i] = s * delta_t;
        }
        for &[xi1, xi2, xi3] in &SOLID_GAUSS_POINTS {
            let (_, d_n) = solid_shape_functions(xi1, xi2, xi3);
            let (_, det_j, inv_j) = solid_jacobian(coords, &d_n);
            let b_u = build_bu_matrix(&d_n, &inv_j);
            for i in 0..24 {
                let mut s = 0.0;
                for r in 0..6 {
                    s += b_u[r][i] * sig_th[r];
                }
                f_int[i] -= s * det_j;
            }
        }
    }

    let mut nodal_forces = [[0.0; 3]; 8];
    for a in 0..8 {
        nodal_forces[a] = [f_int[3 * a], f_int[3 * a + 1], f_int[3 * a + 2]];
    }

    Ok(SolidResult {
        center_stress,
        gauss_stresses,
        nodal_stresses,
        nodal_forces,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use sap_model::MatType;
    use std::collections::BTreeMap;

    fn test_material(e: f64, u: f64, a: f64) -> Material {
        Material {
            name: "MAT".to_string(),
            mat_type: MatType::Steel,
            e,
            e_ortho: None,
            u,
            u_ortho: None,
            g: e / (2.0 * (1.0 + u)),
            g_ortho: None,
            a,
            a_ortho: None,
            unit_weight: 0.0,
            unit_mass: 1.0,
            fy: 0.0,
            fu: 0.0,
            fc: 0.0,
            fys: 0.0,
            color: "Blue".to_string(),
        }
    }

    fn test_section(incompatible_modes: bool) -> SolidSection {
        SolidSection {
            name: "SSEC".to_string(),
            material: "MAT".to_string(),
            mat_angle_deg: [0.0, 0.0, 0.0],
            incompatible_modes,
            modifiers: [1.0; 8],
        }
    }

    /// SAP2000 Verification Example 5-001:
    /// MacNeal & Harder (1985) 3D Patch Test for Seven Irregular 8-Node Solid Elements
    /// inside a Unit Cube (`E = 1,000,000 psi`, `nu = 0.25`).
    #[test]
    fn sap2000_example_5_001_patch_test_with_and_without_incompatible_modes() {
        // 16 joints (1-indexed 1..=16 stored in 0..16)
        let joints: [[f64; 3]; 16] = [
            // Inner joints 1..8
            [0.249, 0.342, 0.192],
            [0.826, 0.288, 0.288],
            [0.850, 0.649, 0.263],
            [0.273, 0.750, 0.230],
            [0.320, 0.186, 0.643],
            [0.677, 0.305, 0.683],
            [0.788, 0.693, 0.644],
            [0.165, 0.745, 0.702],
            // Outer cube corners 9..16
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
            [1.0, 1.0, 1.0],
            [0.0, 1.0, 1.0],
        ];

        // 7 skewed hexahedral elements in CSI 8-node binary ordering (1-based joint IDs)
        let elements: [[usize; 8]; 7] = [
            [9, 1, 12, 4, 13, 5, 16, 8],
            [9, 10, 1, 2, 13, 14, 5, 6],
            [1, 2, 4, 3, 5, 6, 8, 7],
            [4, 3, 12, 11, 8, 7, 16, 15],
            [2, 10, 3, 11, 6, 14, 7, 15],
            [5, 6, 8, 7, 13, 14, 16, 15],
            [9, 10, 12, 11, 1, 2, 4, 3],
        ];

        let exact_disp = |p: [f64; 3]| -> [f64; 3] {
            let [x, y, z] = p;
            [
                1e-3 * (2.0 * x + y + z) / 2.0,
                1e-3 * (x + 2.0 * y + z) / 2.0,
                1e-3 * (x + y + 2.0 * z) / 2.0,
            ]
        };

        let mat = test_material(1.0e6, 0.25, 0.0);

        for incompatible_modes in [true, false] {
            let sec = test_section(incompatible_modes);
            let n_total = 16 * 3;
            let mut k_full = vec![0.0; n_total * n_total];
            let mut total_vol = 0.0;

            for conn in &elements {
                let mut coords = [[0.0; 3]; 8];
                let mut dof_map = [None; 24];
                for a in 0..8 {
                    let j_idx = conn[a] - 1;
                    coords[a] = joints[j_idx];
                    dof_map[3 * a] = Some(3 * j_idx);
                    dof_map[3 * a + 1] = Some(3 * j_idx + 1);
                    dof_map[3 * a + 2] = Some(3 * j_idx + 2);
                }
                total_vol += validate_solid_geometry(&coords).unwrap();
                let (k_e, _) = compute_solid_stiffness_and_mass(&coords, &mat, &sec).unwrap();
                assemble_solid_into_global(&mut k_full, n_total, &dof_map, &k_e);
            }
            // Total volume of the 7 solids must equal the unit cube volume 1.0
            assert!((total_vol - 1.0).abs() < 1e-12);

            // Partition into free DOFs (inner joints 0..8 -> DOFs 0..24) and prescribed DOFs (24..48)
            let mut u_all = [0.0; 48];
            for j in 8..16 {
                let u_p = exact_disp(joints[j]);
                u_all[3 * j] = u_p[0];
                u_all[3 * j + 1] = u_p[1];
                u_all[3 * j + 2] = u_p[2];
            }

            let n_free = 24;
            let mut k_ff = vec![0.0; n_free * n_free];
            let mut rhs = vec![0.0; n_free];
            for i in 0..n_free {
                for j in 0..n_free {
                    k_ff[i * n_free + j] = k_full[i * n_total + j];
                }
                let mut s = 0.0;
                for j in n_free..n_total {
                    s -= k_full[i * n_total + j] * u_all[j];
                }
                rhs[i] = s;
            }

            let u_free = solve_lu(&k_ff, &rhs, n_free).unwrap();
            for i in 0..n_free {
                u_all[i] = u_free[i];
            }

            // Verify all 8 interior joints match the exact linear field to machine precision
            for j in 0..8 {
                let expected = exact_disp(joints[j]);
                for d in 0..3 {
                    assert!(
                        (u_all[3 * j + d] - expected[d]).abs() < 1e-12,
                        "joint {} dir {} mismatch: got {}, expected {}",
                        j + 1,
                        d,
                        u_all[3 * j + d],
                        expected[d]
                    );
                }
            }
            // Specifically check SAP2000 Example 5-001 benchmark values at Joint 8:
            // Ux = 0.00089 in, Uy = 0.00118 in, Uz = 0.00116 in
            let j8_u = [u_all[21], u_all[22], u_all[23]];
            assert!((j8_u[0] - 0.0008885).abs() < 1e-9);
            assert!((j8_u[1] - 0.0011785).abs() < 1e-9);
            assert!((j8_u[2] - 0.001157).abs() < 1e-9);

            // Verify constant stress recovery across all 7 skewed elements:
            // S11 = S22 = S33 = 2000 psi, S12 = S13 = S23 = 400 psi
            for conn in &elements {
                let mut coords = [[0.0; 3]; 8];
                let mut u_elem = [0.0; 24];
                for a in 0..8 {
                    let j_idx = conn[a] - 1;
                    coords[a] = joints[j_idx];
                    u_elem[3 * a] = u_all[3 * j_idx];
                    u_elem[3 * a + 1] = u_all[3 * j_idx + 1];
                    u_elem[3 * a + 2] = u_all[3 * j_idx + 2];
                }
                let res = recover_solid_result(&coords, &mat, &sec, &u_elem, 0.0).unwrap();
                for st in std::iter::once(&res.center_stress)
                    .chain(res.gauss_stresses.iter())
                    .chain(res.nodal_stresses.iter())
                {
                    assert!((st.s11 - 2000.0).abs() < 1e-6);
                    assert!((st.s22 - 2000.0).abs() < 1e-6);
                    assert!((st.s33 - 2000.0).abs() < 1e-6);
                    assert!((st.s12 - 400.0).abs() < 1e-6);
                    assert!((st.s13 - 400.0).abs() < 1e-6);
                    assert!((st.s23 - 400.0).abs() < 1e-6);
                }
            }
        }
    }

    /// SAP2000 Verification Example 5-002:
    /// Straight Cantilever Beam (`L = 6 in`, `b = 0.1 in`, `h = 0.2 in`, `E = 10,000,000 psi`,
    /// `nu = 0.30`) modeled with 6 Solid elements under unit axial extension, unit tip moment,
    /// and unit out-of-plane tip shear.
    #[test]
    fn sap2000_example_5_002_cantilever_beam_bending_and_extension() {
        let mat = test_material(10.0e6, 0.30, 0.0);
        let sec_incomp = test_section(true);
        let sec_std = test_section(false);

        let nid = |ix: usize, iy: usize, iz: usize| -> usize { ix * 4 + iy * 2 + iz };
        let mut coords_all = [[0.0; 3]; 28];
        for ix in 0..7 {
            for iy in 0..2 {
                for iz in 0..2 {
                    coords_all[nid(ix, iy, iz)] = [ix as f64, 0.1 * (iy as f64), 0.2 * (iz as f64)];
                }
            }
        }

        let solve_cantilever = |sec: &SolidSection| -> (f64, f64, f64) {
            let n_dof = 28 * 3;
            let mut k_global = vec![0.0; n_dof * n_dof];
            for e in 0..6 {
                let enodes = [
                    nid(e, 0, 0),
                    nid(e + 1, 0, 0),
                    nid(e, 1, 0),
                    nid(e + 1, 1, 0),
                    nid(e, 0, 1),
                    nid(e + 1, 0, 1),
                    nid(e, 1, 1),
                    nid(e + 1, 1, 1),
                ];
                let mut ec = [[0.0; 3]; 8];
                let mut dof_map = [None; 24];
                for a in 0..8 {
                    ec[a] = coords_all[enodes[a]];
                    dof_map[3 * a] = Some(3 * enodes[a]);
                    dof_map[3 * a + 1] = Some(3 * enodes[a] + 1);
                    dof_map[3 * a + 2] = Some(3 * enodes[a] + 2);
                }
                let (k_e, _) = compute_solid_stiffness_and_mass(&ec, &mat, sec).unwrap();
                assemble_solid_into_global(&mut k_global, n_dof, &dof_map, &k_e);
            }

            // Minimal support at x = 0 preventing rigid-body motion while allowing free Poisson contraction
            let fixed = [
                3 * nid(0, 0, 0),
                3 * nid(0, 0, 1),
                3 * nid(0, 1, 0),
                3 * nid(0, 1, 1),
                3 * nid(0, 0, 0) + 1,
                3 * nid(0, 0, 1) + 1,
                3 * nid(0, 0, 0) + 2,
                3 * nid(0, 1, 0) + 2,
            ];
            let free: Vec<usize> = (0..n_dof).filter(|d| !fixed.contains(d)).collect();
            let nf = free.len();

            let solve_load = |f_full: &[f64]| -> Vec<f64> {
                let mut k_ff = vec![0.0; nf * nf];
                let mut rhs = vec![0.0; nf];
                for (i, &gi) in free.iter().enumerate() {
                    rhs[i] = f_full[gi];
                    for (j, &gj) in free.iter().enumerate() {
                        k_ff[i * nf + j] = k_global[gi * n_dof + gj];
                    }
                }
                let u_f = solve_lu(&k_ff, &rhs, nf).unwrap();
                let mut u_full = vec![0.0; n_dof];
                for (i, &gi) in free.iter().enumerate() {
                    u_full[gi] = u_f[i];
                }
                u_full
            };

            let mut f_axial = vec![0.0; n_dof];
            let mut f_moment = vec![0.0; n_dof];
            let mut f_shear = vec![0.0; n_dof];
            for iy in 0..2 {
                for iz in 0..2 {
                    let nt = nid(6, iy, iz);
                    f_axial[3 * nt] = 0.25; // Total Fx = 1.0 lb
                    f_shear[3 * nt + 2] = 0.25; // Total Fz = 1.0 lb
                    f_moment[3 * nt] = if iz == 0 { -2.5 } else { 2.5 }; // Couple My = 1.0 in-lb
                }
            }

            let u_ax = solve_load(&f_axial);
            let u_bm = solve_load(&f_moment);
            let u_sh = solve_load(&f_shear);

            let tip_nodes = [nid(6, 0, 0), nid(6, 0, 1), nid(6, 1, 0), nid(6, 1, 1)];
            let avg_ux = tip_nodes.iter().map(|&n| u_ax[3 * n]).sum::<f64>() / 4.0;
            let avg_uz_bm = tip_nodes.iter().map(|&n| u_bm[3 * n + 2]).sum::<f64>() / 4.0;
            let avg_uz_sh = tip_nodes.iter().map(|&n| u_sh[3 * n + 2]).sum::<f64>() / 4.0;
            (avg_ux, avg_uz_bm.abs(), avg_uz_sh)
        };

        let (ux_inc, uz_bm_inc, uz_sh_inc) = solve_cantilever(&sec_incomp);
        let (_, uz_bm_std, _) = solve_cantilever(&sec_std);

        // Exact axial extension PL / (EA) = 1.0 * 6.0 / (1e7 * 0.02) = 0.00003 in
        assert!((ux_inc - 3.0e-5).abs() < 1e-12);
        // Pure bending tip deflection ML^2 / (2EI) = 1.0 * 36 / (2 * 1e7 * 0.1 * 0.008 / 12) = 0.0270 in
        assert!((uz_bm_inc - 0.0270).abs() < 5e-5);
        // Tip shear deflection ~ 0.1072 in (vs 0.1080 theoretical)
        assert!((uz_sh_inc - 0.10715).abs() < 2e-4);
        // Confirm incompatible modes eliminate parasitic shear locking present in standard 8-node brick
        assert!(uz_bm_inc > 1.4 * uz_bm_std);
    }

    #[test]
    fn surface_pressure_and_thermal_expansion_equilibrium() {
        // 2x3x4 rectangular block with E = 1e6, nu = 0.25, alpha = 1e-5
        let (lx, ly, lz) = (2.0, 3.0, 4.0);
        let coords: [[f64; 3]; 8] = [
            [0.0, 0.0, 0.0],
            [lx, 0.0, 0.0],
            [0.0, ly, 0.0],
            [lx, ly, 0.0],
            [0.0, 0.0, lz],
            [lx, 0.0, lz],
            [0.0, ly, lz],
            [lx, ly, lz],
        ];
        let mat = test_material(1.0e6, 0.25, 1.0e-5);
        let sec = test_section(true);

        let mut loads_temp = BTreeMap::new();
        loads_temp.insert("TEMP".to_string(), 100.0);
        let solid = SolidObj {
            name: "B1".to_string(),
            points: [
                "1".into(),
                "2".into(),
                "3".into(),
                "4".into(),
                "5".into(),
                "6".into(),
                "7".into(),
                "8".into(),
            ],
            section: "SSEC".to_string(),
            loads_surface: vec![sap_model::SolidSurfaceLoad {
                load_pat: "SURF".to_string(),
                face: 6, // +Z top face (area = 2 * 3 = 6), inward normal is -Z
                val: 120.0,
            }],
            loads_temp,
            selected: true,
        };

        // Check Face 6 inward pressure: total Fz = -120 * (2 * 3) = -720, distributed equally (-180) on nodes 4,5,6,7
        let f_surf =
            compute_solid_nodal_loads(&coords, &mat, &sec, &solid, "SURF", 0.0, [0.0, 0.0, -1.0])
                .unwrap();
        for a in 0..4 {
            assert!(f_surf[3 * a + 2].abs() < 1e-12);
        }
        for a in 4..8 {
            assert!((f_surf[3 * a] - 0.0).abs() < 1e-12);
            assert!((f_surf[3 * a + 1] - 0.0).abs() < 1e-12);
            assert!((f_surf[3 * a + 2] - (-180.0)).abs() < 1e-10);
        }

        // Check free thermal expansion (delta_T = 100, alpha = 1e-5 => eps = 1e-3):
        // Under free thermal displacement u = eps * x, recovered mechanical stress and internal nodal forces are zero.
        let (k_e, _) = compute_solid_stiffness_and_mass(&coords, &mat, &sec).unwrap();
        let f_temp =
            compute_solid_nodal_loads(&coords, &mat, &sec, &solid, "TEMP", 0.0, [0.0, 0.0, -1.0])
                .unwrap();
        let mut u_th = [0.0; 24];
        for a in 0..8 {
            u_th[3 * a] = 1.0e-3 * coords[a][0];
            u_th[3 * a + 1] = 1.0e-3 * coords[a][1];
            u_th[3 * a + 2] = 1.0e-3 * coords[a][2];
        }
        for i in 0..24 {
            let mut ku_i = 0.0;
            for j in 0..24 {
                ku_i += k_e[i][j] * u_th[j];
            }
            assert!((ku_i - f_temp[i]).abs() < 1e-8);
        }
        let res_free = recover_solid_result(&coords, &mat, &sec, &u_th, 100.0).unwrap();
        assert!(res_free.center_stress.von_mises < 1e-8);
        for a in 0..8 {
            for d in 0..3 {
                assert!(res_free.nodal_forces[a][d].abs() < 1e-8);
            }
        }
    }
}
