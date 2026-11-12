//! 3D 2-Node 12-DOF Timoshenko Frame / Beam-Column / Cable element matching `CsiGo2.dll` (`Module Frame`).
//!
//! Implements:
//! - Exact local coordinate system `(1, 2, 3)` (`Go_LocalCoordLine`) and skewed joint axes (`Go_LocalCoordJoint`).
//! - Prismatic and multi-segment non-prismatic (`FramePropType::Variable`) stiffness & flexibility integration.
//! - Exact Timoshenko shear deformation ($\Phi_2, \Phi_3$) and property modifiers `[A, As2, As3, J, I22, I33, Mass, Weight]`.
//! - End length offsets ($a_I, a_J$) with rigid-zone factor $r_z \in [0, 1]$.
//! - End releases (`release_i`, `release_j`) and partial-fixity springs (`start_spring`, `end_spring`) via static condensation.
//! - Consistent fixed-end forces and internal station force/deflection recovery for self-weight, gravity,
//!   distributed/trapezoidal loads (Local, Global, Projected, Gravity, Force & Moment), point span loads,
//!   and uniform/patterned/gradient thermal loads (`FrameLoadTemperature`).
//! - Consistent cubic-Hermite geometric stiffness matrix $\mathbf{K}_G(P)$ for P-Delta and Linear Buckling.

use sap_model::{
    FrameObj, FramePropType, FrameSection, FrameStationResult, Material, PointObj, SapModel,
};

/// 5-point Gauss-Legendre quadrature nodes and weights on $[0, 1]$.
const GL5_NODES: [f64; 5] = [
    0.046_910_077_030_668_0,
    0.230_765_344_947_158_5,
    0.5,
    0.769_234_655_052_841_5,
    0.953_089_922_969_332,
];
const GL5_WEIGHTS: [f64; 5] = [
    0.118_463_442_528_094_5,
    0.239_314_335_249_683_2,
    0.284_444_444_444_444_4,
    0.239_314_335_249_683_2,
    0.118_463_442_528_094_5,
];

/// Computes the $3 \times 3$ joint-local-to-global direction cosine matrix $\mathbf{R}_p$
/// where $\mathbf{v}_{\text{global}} = \mathbf{R}_p \mathbf{v}_{\text{joint-local}}$
/// (columns of $\mathbf{R}_p$ are the joint's local unit vectors $\mathbf{j}_1, \mathbf{j}_2, \mathbf{j}_3$).
pub fn joint_local_to_global_matrix(pt: &PointObj) -> [[f64; 3]; 3] {
    if let Some(mat) = pt.local_axes_matrix {
        return mat;
    }
    let [a_deg, b_deg, c_deg] = pt.local_axes_deg;
    if a_deg.abs() < 1e-14 && b_deg.abs() < 1e-14 && c_deg.abs() < 1e-14 {
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    }
    let (sa, ca) = a_deg.to_radians().sin_cos();
    let (sb, cb) = b_deg.to_radians().sin_cos();
    let (sc, cc) = c_deg.to_radians().sin_cos();

    // R = R_Z(A) * R_{-Y}(B) * R_X(C)
    // where R_{-Y}(B) has j_1 = (cos B, 0, sin B) and j_3 = (-sin B, 0, cos B)
    let rz = [[ca, -sa, 0.0], [sa, ca, 0.0], [0.0, 0.0, 1.0]];
    let ry = [[cb, 0.0, -sb], [0.0, 1.0, 0.0], [sb, 0.0, cb]];
    let rx = [[1.0, 0.0, 0.0], [0.0, cc, -sc], [0.0, sc, cc]];

    let r_zy = mat3_mul(&rz, &ry);
    mat3_mul(&r_zy, &rx)
}

#[inline]
pub fn mat3_mul(a: &[[f64; 3]; 3], b: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut out = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
        }
    }
    out
}

#[inline]
pub fn mat3_transpose(a: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    [
        [a[0][0], a[1][0], a[2][0]],
        [a[0][1], a[1][1], a[2][1]],
        [a[0][2], a[1][2], a[2][2]],
    ]
}

/// Computes the element length $L$ and the $3 \times 3$ global-to-local rotation matrix $\mathbf{R}_{\text{elem}}$
/// whose rows are the unit local axes $\mathbf{e}_1, \mathbf{e}_2, \mathbf{e}_3$ in Global `(X, Y, Z)` coordinates
/// (`Go_LocalCoordLine`).
pub fn frame_local_axes(pi: [f64; 3], pj: [f64; 3], angle_deg: f64) -> (f64, [[f64; 3]; 3]) {
    let dx = pj[0] - pi[0];
    let dy = pj[1] - pi[1];
    let dz = pj[2] - pi[2];
    let length = (dx * dx + dy * dy + dz * dz).sqrt().max(1e-12);
    let e1 = [dx / length, dy / length, dz / length];

    let h = (e1[0] * e1[0] + e1[1] * e1[1]).sqrt();
    let (e2_0, e3_0) = if h > 1e-4 {
        // Non-vertical: local 1-2 plane is vertical, e2_0 has +Z projection, e3_0 = e1 x e2_0
        let e2 = [-e1[0] * e1[2] / h, -e1[1] * e1[2] / h, h];
        let e3 = [e1[1] / h, -e1[0] / h, 0.0];
        (e2, e3)
    } else {
        // Vertical along +-Z: local 2-axis points along Global +X, e3_0 = e1 x e2_0
        let cz = if e1[2] >= 0.0 { 1.0 } else { -1.0 };
        let e2 = [1.0, 0.0, 0.0];
        let e3 = [0.0, cz, 0.0];
        (e2, e3)
    };

    let (sb, cb) = angle_deg.to_radians().sin_cos();
    let e2 = [
        cb * e2_0[0] + sb * e3_0[0],
        cb * e2_0[1] + sb * e3_0[1],
        cb * e2_0[2] + sb * e3_0[2],
    ];
    let e3 = [
        -sb * e2_0[0] + cb * e3_0[0],
        -sb * e2_0[1] + cb * e3_0[1],
        -sb * e2_0[2] + cb * e3_0[2],
    ];

    (length, [e1, e2, e3])
}

/// Effective cross-sectional properties at a station along the frame element.
#[derive(Debug, Clone, Copy)]
pub struct SectionPropsAtStation {
    pub area: f64,
    pub as2: f64,
    pub as3: f64,
    pub j: f64,
    pub i22: f64,
    pub i33: f64,
    pub mass_per_len: f64,
    pub weight_per_len: f64,
}

/// Evaluates effective section properties at normalized coordinate $s \in [0, 1]$ along the flexible length.
pub fn eval_section_props_at(
    model: &SapModel,
    frame: &FrameObj,
    sec: &FrameSection,
    mat: &Material,
    s_norm: f64,
    lf: f64,
) -> SectionPropsAtStation {
    let mut mods = [1.0; 8];
    for k in 0..8 {
        mods[k] = sec.modifiers[k] * frame.modifiers[k];
    }

    if sec.prop_type != FramePropType::Variable || sec.non_prismatic.is_empty() {
        return SectionPropsAtStation {
            area: sec.area * mods[0],
            as2: sec.as2 * mods[1],
            as3: sec.as3 * mods[2],
            j: sec.torsional_j * mods[3],
            i22: sec.i22 * mods[4],
            i33: sec.i33 * mods[5],
            mass_per_len: sec.area * mat.unit_mass * mods[6],
            weight_per_len: sec.area * mat.unit_weight * mods[7],
        };
    }

    // Resolve non-prismatic segment lengths
    let segs = resolve_non_prismatic_segments(&sec.non_prismatic, lf);
    let x_pos = (s_norm.clamp(0.0, 1.0)) * lf;
    let mut accum = 0.0;
    for (seg, seg_len) in segs {
        if x_pos <= accum + seg_len + 1e-12 || seg_len <= 1e-12 {
            let xi = if seg_len > 1e-12 {
                ((x_pos - accum) / seg_len).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let s_start = model.frame_sections.get(&seg.start_sec).unwrap_or(sec);
            let s_end = model.frame_sections.get(&seg.end_sec).unwrap_or(sec);

            let area = (s_start.area * (1.0 - xi) + s_end.area * xi) * mods[0];
            let as2 = (s_start.as2 * (1.0 - xi) + s_end.as2 * xi) * mods[1];
            let as3 = (s_start.as3 * (1.0 - xi) + s_end.as3 * xi) * mods[2];
            let j = (s_start.torsional_j * (1.0 - xi) + s_end.torsional_j * xi) * mods[3];

            let n33 = (seg.ei33_variation.max(1)) as f64;
            let i33_raw = (s_start.i33.max(0.0).powf(1.0 / n33) * (1.0 - xi)
                + s_end.i33.max(0.0).powf(1.0 / n33) * xi)
                .powf(n33);
            let n22 = (seg.ei22_variation.max(1)) as f64;
            let i22_raw = (s_start.i22.max(0.0).powf(1.0 / n22) * (1.0 - xi)
                + s_end.i22.max(0.0).powf(1.0 / n22) * xi)
                .powf(n22);

            let raw_area = s_start.area * (1.0 - xi) + s_end.area * xi;
            return SectionPropsAtStation {
                area,
                as2,
                as3,
                j,
                i22: i22_raw * mods[4],
                i33: i33_raw * mods[5],
                mass_per_len: raw_area * mat.unit_mass * mods[6],
                weight_per_len: raw_area * mat.unit_weight * mods[7],
            };
        }
        accum += seg_len;
    }

    SectionPropsAtStation {
        area: sec.area * mods[0],
        as2: sec.as2 * mods[1],
        as3: sec.as3 * mods[2],
        j: sec.torsional_j * mods[3],
        i22: sec.i22 * mods[4],
        i33: sec.i33 * mods[5],
        mass_per_len: sec.area * mat.unit_mass * mods[6],
        weight_per_len: sec.area * mat.unit_weight * mods[7],
    }
}

pub(crate) fn resolve_non_prismatic_segments(
    segments: &[sap_model::NonPrismaticSegment],
    lf: f64,
) -> Vec<(&sap_model::NonPrismaticSegment, f64)> {
    let mut fixed_sum = 0.0;
    let mut var_sum = 0.0;
    for seg in segments {
        if seg.is_variable_length {
            var_sum += seg.length.max(0.0);
        } else {
            fixed_sum += seg.length.max(0.0);
        }
    }

    let mut out = Vec::with_capacity(segments.len());
    if var_sum > 1e-12 && fixed_sum < lf {
        let rem = lf - fixed_sum;
        for seg in segments {
            let len = if seg.is_variable_length {
                rem * (seg.length.max(0.0) / var_sum)
            } else {
                seg.length.max(0.0)
            };
            out.push((seg, len));
        }
    } else {
        let total = (fixed_sum + var_sum).max(1e-12);
        for seg in segments {
            out.push((seg, lf * (seg.length.max(0.0) / total)));
        }
    }
    out
}

/// Computes the $12 \times 12$ flexible local stiffness matrix $\mathbf{K}_f$ (before releases and rigid offsets).
pub fn compute_flexible_local_stiffness(
    model: &SapModel,
    frame: &FrameObj,
    sec: &FrameSection,
    mat: &Material,
    lf: f64,
) -> [[f64; 12]; 12] {
    let e = mat.e;
    let g = if mat.g > 1e-12 {
        mat.g
    } else {
        mat.e / (2.0 * (1.0 + mat.u))
    };

    if sec.prop_type == FramePropType::Variable && !sec.non_prismatic.is_empty() {
        return compute_non_prismatic_flexible_stiffness(model, frame, sec, mat, lf, e, g);
    }

    let p = eval_section_props_at(model, frame, sec, mat, 0.5, lf);
    let mut k = [[0.0f64; 12]; 12];

    // Axial (DOFs 0, 6)
    let ka = e * p.area / lf;
    k[0][0] = ka;
    k[0][6] = -ka;
    k[6][0] = -ka;
    k[6][6] = ka;

    // Torsion (DOFs 3, 9)
    let kt = g * p.j / lf;
    k[3][3] = kt;
    k[3][9] = -kt;
    k[9][3] = -kt;
    k[9][9] = kt;

    // Major-axis bending in 1-2 plane: DOFs (u2_I=1, r3_I=5, u2_J=7, r3_J=11)
    let phi2 = if g * p.as2 > 1e-14 {
        (12.0 * e * p.i33) / (g * p.as2 * lf * lf)
    } else {
        0.0
    };
    let c12 = (e * p.i33) / ((1.0 + phi2) * lf.powi(3));
    let k11_2 = 12.0 * c12;
    let k12_2 = 6.0 * lf * c12;
    let k22_2 = (4.0 + phi2) * lf * lf * c12;
    let k24_2 = (2.0 - phi2) * lf * lf * c12;

    k[1][1] = k11_2;
    k[1][5] = k12_2;
    k[1][7] = -k11_2;
    k[1][11] = k12_2;

    k[5][1] = k12_2;
    k[5][5] = k22_2;
    k[5][7] = -k12_2;
    k[5][11] = k24_2;

    k[7][1] = -k11_2;
    k[7][5] = -k12_2;
    k[7][7] = k11_2;
    k[7][11] = -k12_2;

    k[11][1] = k12_2;
    k[11][5] = k24_2;
    k[11][7] = -k12_2;
    k[11][11] = k22_2;

    // Minor-axis bending in 1-3 plane: DOFs (u3_I=2, r2_I=4, u3_J=8, r2_J=10)
    // Note right-hand rule: r2 = -du3/dx
    let phi3 = if g * p.as3 > 1e-14 {
        (12.0 * e * p.i22) / (g * p.as3 * lf * lf)
    } else {
        0.0
    };
    let c13 = (e * p.i22) / ((1.0 + phi3) * lf.powi(3));
    let k11_3 = 12.0 * c13;
    let k12_3 = -6.0 * lf * c13;
    let k22_3 = (4.0 + phi3) * lf * lf * c13;
    let k24_3 = (2.0 - phi3) * lf * lf * c13;

    k[2][2] = k11_3;
    k[2][4] = k12_3;
    k[2][8] = -k11_3;
    k[2][10] = k12_3;

    k[4][2] = k12_3;
    k[4][4] = k22_3;
    k[4][8] = -k12_3;
    k[4][10] = k24_3;

    k[8][2] = -k11_3;
    k[8][4] = -k12_3;
    k[8][8] = k11_3;
    k[8][10] = -k12_3;

    k[10][2] = k12_3;
    k[10][4] = k24_3;
    k[10][8] = -k12_3;
    k[10][10] = k22_3;

    k
}

fn compute_non_prismatic_flexible_stiffness(
    model: &SapModel,
    frame: &FrameObj,
    sec: &FrameSection,
    mat: &Material,
    lf: f64,
    e: f64,
    g: f64,
) -> [[f64; 12]; 12] {
    let segs = resolve_non_prismatic_segments(&sec.non_prismatic, lf);
    let mut f_ax = 0.0;
    let mut f_tor = 0.0;
    let mut f12 = [[0.0; 2]; 2]; // Flexibility at J-end in 1-2 plane: [u2_J, r3_J]
    let mut f13 = [[0.0; 2]; 2]; // Flexibility at J-end in 1-3 plane: [u3_J, r2_J]

    let mut x_start = 0.0;
    for (_seg, seg_len) in segs {
        if seg_len <= 1e-12 {
            continue;
        }
        for (&xi_g, &w_g) in GL5_NODES.iter().zip(GL5_WEIGHTS.iter()) {
            let x = x_start + xi_g * seg_len;
            let dx = w_g * seg_len;
            let arm = lf - x;
            let p = eval_section_props_at(model, frame, sec, mat, x / lf, lf);

            if e * p.area > 1e-18 {
                f_ax += dx / (e * p.area);
            }
            if g * p.j > 1e-18 {
                f_tor += dx / (g * p.j);
            }
            if e * p.i33 > 1e-18 {
                let inv_ei33 = 1.0 / (e * p.i33);
                let inv_gas2 = if g * p.as2 > 1e-14 {
                    1.0 / (g * p.as2)
                } else {
                    0.0
                };
                f12[0][0] += (arm * arm * inv_ei33 + inv_gas2) * dx;
                f12[0][1] += (arm * inv_ei33) * dx;
                f12[1][0] += (arm * inv_ei33) * dx;
                f12[1][1] += inv_ei33 * dx;
            }
            if e * p.i22 > 1e-18 {
                let inv_ei22 = 1.0 / (e * p.i22);
                let inv_gas3 = if g * p.as3 > 1e-14 {
                    1.0 / (g * p.as3)
                } else {
                    0.0
                };
                // In 1-3 plane, M2(x) = -(L - x) * F3_J + M2_J
                f13[0][0] += (arm * arm * inv_ei22 + inv_gas3) * dx;
                f13[0][1] += (-arm * inv_ei22) * dx;
                f13[1][0] += (-arm * inv_ei22) * dx;
                f13[1][1] += inv_ei22 * dx;
            }
        }
        x_start += seg_len;
    }

    let mut k = [[0.0f64; 12]; 12];
    if f_ax > 1e-18 {
        let ka = 1.0 / f_ax;
        k[0][0] = ka;
        k[0][6] = -ka;
        k[6][0] = -ka;
        k[6][6] = ka;
    }
    if f_tor > 1e-18 {
        let kt = 1.0 / f_tor;
        k[3][3] = kt;
        k[3][9] = -kt;
        k[9][3] = -kt;
        k[9][9] = kt;
    }

    // Invert 2x2 flexibility f12 and expand via equilibrium T_IJ = [[-1, 0], [-lf, -1]]
    let det12 = f12[0][0] * f12[1][1] - f12[0][1] * f12[1][0];
    if det12.abs() > 1e-24 {
        let kjj = [
            [f12[1][1] / det12, -f12[0][1] / det12],
            [-f12[1][0] / det12, f12[0][0] / det12],
        ];
        let tij = [[-1.0, 0.0], [-lf, -1.0]];
        let dofs = [1usize, 5, 7, 11];
        assemble_plane_from_kjj(&mut k, dofs, &kjj, &tij);
    }

    // Invert 2x2 flexibility f13 and expand via equilibrium T_IJ = [[-1, 0], [lf, -1]]
    let det13 = f13[0][0] * f13[1][1] - f13[0][1] * f13[1][0];
    if det13.abs() > 1e-24 {
        let kjj = [
            [f13[1][1] / det13, -f13[0][1] / det13],
            [-f13[1][0] / det13, f13[0][0] / det13],
        ];
        let tij = [[-1.0, 0.0], [lf, -1.0]];
        let dofs = [2usize, 4, 8, 10];
        assemble_plane_from_kjj(&mut k, dofs, &kjj, &tij);
    }

    k
}

fn assemble_plane_from_kjj(
    k: &mut [[f64; 12]; 12],
    dofs: [usize; 4],
    kjj: &[[f64; 2]; 2],
    tij: &[[f64; 2]; 2],
) {
    // [T * Kjj * T^T, T * Kjj; Kjj * T^T, Kjj]
    let mut tk = [[0.0; 2]; 2];
    for i in 0..2 {
        for j in 0..2 {
            tk[i][j] = tij[i][0] * kjj[0][j] + tij[i][1] * kjj[1][j];
        }
    }
    let mut tkt = [[0.0; 2]; 2];
    for i in 0..2 {
        for j in 0..2 {
            tkt[i][j] = tk[i][0] * tij[j][0] + tk[i][1] * tij[j][1];
        }
    }
    for i in 0..2 {
        for j in 0..2 {
            k[dofs[i]][dofs[j]] = tkt[i][j];
            k[dofs[i]][dofs[j + 2]] = tk[i][j];
            k[dofs[i + 2]][dofs[j]] = tk[j][i];
            k[dofs[i + 2]][dofs[j + 2]] = kjj[i][j];
        }
    }
}

/// Applies member end releases (`release_i`, `release_j`) and partial-fixity springs (`start_spring`, `end_spring`)
/// to both the $12 \times 12$ local stiffness matrix `k` and the $12 \times 1$ fixed-end force vector `fef`.
pub fn condense_releases_and_springs(
    k: &mut [[f64; 12]; 12],
    fef: &mut [f64; 12],
    frame: &FrameObj,
) {
    for dof in 0..12 {
        let (released, ks) = if dof < 6 {
            (frame.release_i[dof], frame.start_spring[dof].max(0.0))
        } else {
            (
                frame.release_j[dof - 6],
                frame.end_spring[dof - 6].max(0.0),
            )
        };
        if !released {
            continue;
        }
        let k_mm = k[dof][dof];
        let denom = k_mm + ks;
        if denom.abs() <= 1e-18 {
            for i in 0..12 {
                k[dof][i] = 0.0;
                k[i][dof] = 0.0;
            }
            fef[dof] = 0.0;
            continue;
        }

        let col_m: [f64; 12] = std::array::from_fn(|i| k[i][dof]);
        let row_m: [f64; 12] = k[dof];
        let fef_m = fef[dof];

        for i in 0..12 {
            if i == dof {
                fef[i] = (ks / denom) * fef_m;
            } else {
                fef[i] -= (col_m[i] / denom) * fef_m;
            }
        }

        for i in 0..12 {
            for j in 0..12 {
                if i == dof && j == dof {
                    k[i][j] = (ks * k_mm) / denom;
                } else if i == dof {
                    k[i][j] = (ks / denom) * row_m[j];
                } else if j == dof {
                    k[i][j] = (ks / denom) * col_m[i];
                } else {
                    k[i][j] -= (col_m[i] * row_m[j]) / denom;
                }
            }
        }
    }
}

/// Applies rigid end-zone offset transformation $\mathbf{T}_{\text{rigid}}$ ($d_I = r_z a_I, d_J = r_z a_J$)
/// to both the $12 \times 12$ stiffness matrix and $12 \times 1$ fixed-end force vector.
pub fn apply_rigid_offsets(
    k: &mut [[f64; 12]; 12],
    fef: &mut [f64; 12],
    di: f64,
    dj: f64,
) {
    if di.abs() < 1e-14 && dj.abs() < 1e-14 {
        return;
    }
    let mut t = [[0.0f64; 12]; 12];
    for i in 0..12 {
        t[i][i] = 1.0;
    }
    // At I-end: u2_fI = u2_I + di * r3_I, u3_fI = u3_I - di * r2_I
    t[1][5] = di;
    t[2][4] = -di;
    // At J-end: u2_fJ = u2_J - dj * r3_J, u3_fJ = u3_J + dj * r2_J
    t[7][11] = -dj;
    t[8][10] = dj;

    // fef_new = T^T * fef
    let old_fef = *fef;
    for i in 0..12 {
        let mut s = 0.0;
        for m in 0..12 {
            s += t[m][i] * old_fef[m];
        }
        fef[i] = s;
    }

    // k_new = T^T * k * T
    let mut kt = [[0.0f64; 12]; 12];
    for i in 0..12 {
        for j in 0..12 {
            let mut s = 0.0;
            for m in 0..12 {
                s += k[i][m] * t[m][j];
            }
            kt[i][j] = s;
        }
    }
    for i in 0..12 {
        for j in 0..12 {
            let mut s = 0.0;
            for m in 0..12 {
                s += t[m][i] * kt[m][j];
            }
            k[i][j] = s;
        }
    }
}

/// Offset of the section centroid from the line between the joints, in element
/// local axes, for a frame whose insertion point (cardinal point plus joint
/// offsets) has stiffness transformation on. `Ok([0.0; 3])` means the frame
/// is analyzed between its joints. `Err` names what is not implemented.
///
/// Cardinal points follow the CSI Analysis Reference figure "Frame Cardinal
/// Points": bottom/top are the negative/positive local 2 side and left/right
/// the negative/positive local 3 side, at half the section depth and width.
/// The centroid is at the joint plus the joint offset, minus the position of
/// the cardinal point relative to the centroid.
pub fn insertion_offset_local(model: &SapModel, frame: &FrameObj) -> Result<[f64; 3], String> {
    let no_offsets = frame.joint_offset_i == [0.0; 3] && frame.joint_offset_j == [0.0; 3];
    if !frame.stiff_transform || (frame.cardinal_point == 10 && no_offsets) {
        return Ok([0.0; 3]);
    }
    let (Some(pi), Some(pj)) = (
        model.points.get(&frame.point_i),
        model.points.get(&frame.point_j),
    ) else {
        return Ok([0.0; 3]);
    };
    let (length, e_axes) =
        frame_local_axes([pi.x, pi.y, pi.z], [pj.x, pj.y, pj.z], frame.angle_deg);

    // Position of the cardinal point relative to the centroid, in half depths
    // (local 2) and half widths (local 3).
    let (c2, c3) = match frame.cardinal_point {
        1 => (-1.0, -1.0),
        2 => (-1.0, 0.0),
        3 => (-1.0, 1.0),
        4 => (0.0, -1.0),
        5 | 10 | 11 => (0.0, 0.0),
        6 => (0.0, 1.0),
        7 => (1.0, -1.0),
        8 => (1.0, 0.0),
        9 => (1.0, 1.0),
        other => return Err(format!("cardinal point {other} is not one of 1 to 11")),
    };
    // A cable object has no frame section and no bending element to offset.
    if frame.cable_params.is_some() {
        return Err("it is a cable".to_string());
    }
    let mut centroid = [0.0f64; 2];
    if frame.cardinal_point != 10 {
        let Some(sec) = model.frame_sections.get(&frame.section) else {
            return Ok([0.0; 3]);
        };
        // Cardinal points 5 and 11 are the centroid, and the others are half
        // the outside dimensions away from it, only for a doubly symmetric
        // shape of known dimensions.
        let (depth, width) = match sec.prop_type {
            FramePropType::Rectangular | FramePropType::Box => (sec.t3, sec.t2),
            FramePropType::I if sec.t2 == sec.t2b && sec.tf == sec.tfb => (sec.t3, sec.t2),
            FramePropType::Circle | FramePropType::Pipe => (sec.t3, sec.t3),
            _ => {
                return Err(format!(
                    "cardinal point {} on section '{}', whose shape is not a doubly symmetric one with known outside dimensions",
                    frame.cardinal_point, sec.name
                ))
            }
        };
        if !(depth > 0.0 && width > 0.0) {
            return Err(format!(
                "cardinal point {} on section '{}', which has no outside dimensions",
                frame.cardinal_point, sec.name
            ));
        }
        centroid = [-c2 * depth / 2.0, -c3 * width / 2.0];
    }

    let local = |offset: [f64; 3]| {
        if frame.joint_offset_is_global {
            global_vec_to_local(offset, &e_axes)
        } else {
            offset
        }
    };
    let (at_i, at_j) = (local(frame.joint_offset_i), local(frame.joint_offset_j));
    let tolerance = 1e-9 * length.max(1e-12);
    if (0..3).any(|c| (at_i[c] - at_j[c]).abs() > tolerance) {
        return Err(
            "joint offsets that differ between its ends, which turn the member axis".to_string(),
        );
    }
    if at_i[0].abs() > tolerance {
        return Err("a joint offset along its own axis".to_string());
    }
    let offset = [0.0, at_i[1] + centroid[0], at_i[2] + centroid[1]];
    if offset == [0.0; 3] {
        return Ok(offset);
    }
    // What the transformation below does not cover.
    if frame.end_offset_i != 0.0 || frame.end_offset_j != 0.0 {
        return Err("end length offsets".to_string());
    }
    if frame.pdelta_force != 0.0 {
        return Err("an assigned P-Delta force".to_string());
    }
    if frame.auto_mesh_at_joints {
        return Err("automatic subdivision at joints".to_string());
    }
    if model
        .frame_sections
        .get(&frame.section)
        .is_some_and(|sec| sec.prop_type == FramePropType::Variable)
    {
        return Err("a non-prismatic section".to_string());
    }
    Ok(offset)
}

/// Rigid arm from each joint to the element end at `offset` (element local):
/// `u_end = u_joint + theta_joint x offset`. Returns the 3x3 block that
/// multiplies the joint rotation.
fn insertion_arm(offset: [f64; 3]) -> [[f64; 3]; 3] {
    [
        [0.0, offset[2], -offset[1]],
        [-offset[2], 0.0, offset[0]],
        [offset[1], -offset[0], 0.0],
    ]
}

/// Carries element stiffness and fixed-end forces from the element ends to
/// the joints through the rigid arms of an insertion point: `k = T^T k T`,
/// `f = T^T f`. The arms are exact rigid links, not penalty constraints.
pub fn apply_insertion_offset(k: &mut [[f64; 12]; 12], fef: &mut [f64; 12], offset: [f64; 3]) {
    if offset == [0.0; 3] {
        return;
    }
    let arm = insertion_arm(offset);
    let mut t = [[0.0f64; 12]; 12];
    for (i, row) in t.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    for end in [0, 6] {
        for i in 0..3 {
            for j in 0..3 {
                t[end + i][end + 3 + j] = arm[i][j];
            }
        }
    }
    let old_fef = *fef;
    for i in 0..12 {
        fef[i] = (0..12).map(|m| t[m][i] * old_fef[m]).sum();
    }
    let mut kt = [[0.0f64; 12]; 12];
    for i in 0..12 {
        for j in 0..12 {
            kt[i][j] = (0..12).map(|m| k[i][m] * t[m][j]).sum();
        }
    }
    for i in 0..12 {
        for j in 0..12 {
            k[i][j] = (0..12).map(|m| t[m][i] * kt[m][j]).sum();
        }
    }
}

/// The inverse view for result recovery: joint displacements and joint-level
/// end forces (element local) become those of the element ends.
pub fn insertion_to_element_ends(u: &mut [f64; 12], f: &mut [f64; 12], offset: [f64; 3]) {
    if offset == [0.0; 3] {
        return;
    }
    let arm = insertion_arm(offset);
    for end in [0, 6] {
        for i in 0..3 {
            // u_end = u_joint + arm * theta
            u[end + i] += (0..3).map(|j| arm[i][j] * u[end + 3 + j]).sum::<f64>();
            // m_joint = m_end + arm^T * F, so m_end = m_joint - arm^T * F
            f[end + 3 + i] -= (0..3).map(|j| arm[j][i] * f[end + j]).sum::<f64>();
        }
    }
}

/// Computes the consistent cubic-Hermite geometric stiffness matrix $\mathbf{K}_G(P)$ ($12 \times 12$)
/// in local coordinates for axial force $P$ (`+` = tension, `-` = compression).
pub fn compute_local_geometric_stiffness(p_axial: f64, lf: f64) -> [[f64; 12]; 12] {
    let mut kg = [[0.0f64; 12]; 12];
    if p_axial.abs() <= 1e-18 || lf <= 1e-12 {
        return kg;
    }

    let c11 = 1.2 * p_axial / lf;
    let c12 = 0.1 * p_axial;
    let c22 = (2.0 / 15.0) * p_axial * lf;
    let c24 = -(1.0 / 30.0) * p_axial * lf;

    // 1-2 plane: (u2_I=1, r3_I=5, u2_J=7, r3_J=11)
    kg[1][1] = c11;
    kg[1][5] = c12;
    kg[1][7] = -c11;
    kg[1][11] = c12;

    kg[5][1] = c12;
    kg[5][5] = c22;
    kg[5][7] = -c12;
    kg[5][11] = c24;

    kg[7][1] = -c11;
    kg[7][5] = -c12;
    kg[7][7] = c11;
    kg[7][11] = -c12;

    kg[11][1] = c12;
    kg[11][5] = c24;
    kg[11][7] = -c12;
    kg[11][11] = c22;

    // 1-3 plane: (u3_I=2, r2_I=4, u3_J=8, r2_J=10) with r2 = -du3/dx
    kg[2][2] = c11;
    kg[2][4] = -c12;
    kg[2][8] = -c11;
    kg[2][10] = -c12;

    kg[4][2] = -c12;
    kg[4][4] = c22;
    kg[4][8] = c12;
    kg[4][10] = c24;

    kg[8][2] = -c11;
    kg[8][4] = c12;
    kg[8][8] = c11;
    kg[8][10] = c12;

    kg[10][2] = -c12;
    kg[10][4] = c24;
    kg[10][8] = c12;
    kg[10][10] = c22;

    kg
}

/// Resolves a load direction code (`1..=11`) and intensity into a local 3D vector `[v1, v2, v3]`.
/// `is_distributed`: if true, applies projected length factor for `dir` in `7..=9` or `11`.
pub fn resolve_load_dir_to_local(
    dir: i32,
    val: f64,
    e_axes: &[[f64; 3]; 3],
    is_distributed: bool,
) -> [f64; 3] {
    let e1 = e_axes[0];
    let proj_x = (1.0 - e1[0] * e1[0]).max(0.0).sqrt();
    let proj_y = (1.0 - e1[1] * e1[1]).max(0.0).sqrt();
    let proj_z = (e1[0] * e1[0] + e1[1] * e1[1]).sqrt();

    match dir {
        1 => [val, 0.0, 0.0],
        2 => [0.0, val, 0.0],
        3 => [0.0, 0.0, val],
        4 => global_vec_to_local([val, 0.0, 0.0], e_axes),
        5 => global_vec_to_local([0.0, val, 0.0], e_axes),
        6 => global_vec_to_local([0.0, 0.0, val], e_axes),
        7 => {
            let pf = if is_distributed { proj_x } else { 1.0 };
            global_vec_to_local([val * pf, 0.0, 0.0], e_axes)
        }
        8 => {
            let pf = if is_distributed { proj_y } else { 1.0 };
            global_vec_to_local([0.0, val * pf, 0.0], e_axes)
        }
        9 => {
            let pf = if is_distributed { proj_z } else { 1.0 };
            global_vec_to_local([0.0, 0.0, val * pf], e_axes)
        }
        10 => global_vec_to_local([0.0, 0.0, -val], e_axes),
        11 => {
            let pf = if is_distributed { proj_z } else { 1.0 };
            global_vec_to_local([0.0, 0.0, -val * pf], e_axes)
        }
        _ => [0.0, 0.0, 0.0],
    }
}

/// Projected force intensity uses sin(theta); projected moment intensity uses
/// cos(theta), per CSI Analysis Reference Manual p.140. Projection is a length
/// magnitude and is invariant under reversal of member connectivity.
fn distributed_load_dir_to_local(dir: i32, value: f64, axes: &[[f64;3];3], my_type: i32) -> [f64;3] {
    if my_type == 2 {
        let projection_axis = match dir { 7..=9 => Some((dir-7) as usize), 11 => Some(2), _ => None };
        if let Some(axis) = projection_axis {
            let direction = if dir == 11 { 10 } else { dir-3 };
            return resolve_load_dir_to_local(direction, value*axes[0][axis].abs(), axes, false);
        }
    }
    resolve_load_dir_to_local(dir,value,axes,true)
}

#[inline]
pub fn global_vec_to_local(g: [f64; 3], e_axes: &[[f64; 3]; 3]) -> [f64; 3] {
    [
        e_axes[0][0] * g[0] + e_axes[0][1] * g[1] + e_axes[0][2] * g[2],
        e_axes[1][0] * g[0] + e_axes[1][1] * g[1] + e_axes[1][2] * g[2],
        e_axes[2][0] * g[0] + e_axes[2][1] * g[1] + e_axes[2][2] * g[2],
    ]
}

/// Adds the fixed-end force contribution of a concentrated force/moment at relative position $s \in [0, 1]$
/// on a prismatic Timoshenko beam of length $L_f$.
pub fn add_point_load_fef(
    fef: &mut [f64; 12],
    my_type: i32,
    local_vec: [f64; 3],
    s: f64,
    lf: f64,
    phi2: f64,
    phi3: f64,
) {
    let s = s.clamp(0.0, 1.0);
    let b = 1.0 - s;

    if my_type == 1 {
        // Point Force [P1, P2, P3]
        let [p1, p2, p3] = local_vec;
        // Axial
        fef[0] -= p1 * b;
        fef[6] -= p1 * s;

        // 1-2 plane (P2 -> V2_I, M3_I, V2_J, M3_J)
        if p2.abs() > 0.0 {
            let denom = 1.0 + phi2;
            fef[1] -= p2 * b * (b * (1.0 + 2.0 * s) + phi2) / denom;
            fef[5] -= p2 * lf * s * b * (b + 0.5 * phi2) / denom;
            fef[7] -= p2 * s * (s * (3.0 - 2.0 * s) + phi2) / denom;
            fef[11] += p2 * lf * s * b * (s + 0.5 * phi2) / denom;
        }

        // 1-3 plane (P3 -> V3_I, M2_I, V3_J, M2_J; note r2 = -du3/dx flips moment signs)
        if p3.abs() > 0.0 {
            let denom = 1.0 + phi3;
            fef[2] -= p3 * b * (b * (1.0 + 2.0 * s) + phi3) / denom;
            fef[4] += p3 * lf * s * b * (b + 0.5 * phi3) / denom;
            fef[8] -= p3 * s * (s * (3.0 - 2.0 * s) + phi3) / denom;
            fef[10] -= p3 * lf * s * b * (s + 0.5 * phi3) / denom;
        }
    } else if my_type == 2 {
        // Point Moment [M1, M2, M3]
        let [m1, m2, m3] = local_vec;
        // Torsion
        fef[3] -= m1 * b;
        fef[9] -= m1 * s;

        // Bending moment M3 in 1-2 plane
        if m3.abs() > 0.0 {
            let denom = 1.0 + phi2;
            let v = 6.0 * m3 * s * b / (denom * lf);
            fef[1] += v;
            fef[5] -= m3 * b * (1.0 - 3.0 * s + phi2) / denom;
            fef[7] -= v;
            fef[11] -= m3 * s * (3.0 * s - 2.0 + phi2) / denom;
        }

        // Bending moment M2 in 1-3 plane
        if m2.abs() > 0.0 {
            let denom = 1.0 + phi3;
            let v = -6.0 * m2 * s * b / (denom * lf);
            fef[2] += v;
            fef[4] -= m2 * b * (1.0 - 3.0 * s + phi3) / denom;
            fef[8] -= v;
            fef[10] -= m2 * s * (3.0 * s - 2.0 + phi3) / denom;
        }
    }
}

/// Adds the fixed-end forces of a concentrated force or moment at relative
/// position $s$ on a member whose section varies along its length.
///
/// The member is released at J: the load is carried to I by statics, the free
/// motion of J follows from the unit-load integrals of the varying section
/// (the same flexibilities the stiffness is built from), and the stiffness
/// then restrains that motion. The prismatic closed forms do not apply here.
#[allow(clippy::too_many_arguments)]
fn add_point_load_fef_non_prismatic(
    fef: &mut [f64; 12],
    model: &SapModel,
    frame: &FrameObj,
    sec: &FrameSection,
    mat: &Material,
    stiffness: &[[f64; 12]; 12],
    my_type: i32,
    local_vec: [f64; 3],
    s: f64,
    lf: f64,
) {
    let a = s.clamp(0.0, 1.0) * lf;
    let (force, moment) = match my_type {
        1 => (local_vec, [0.0; 3]),
        2 => ([0.0; 3], local_vec),
        _ => return,
    };
    let e = mat.e;
    let g = if mat.g > 1e-12 { mat.g } else { mat.e / (2.0 * (1.0 + mat.u)) };

    // Free motion of J: [u1, u2, u3, r1, r2, r3].
    let mut free = [0.0f64; 6];
    let mut start = 0.0;
    for (_, segment_length) in resolve_non_prismatic_segments(&sec.non_prismatic, lf) {
        let (lo, hi) = (start, (start + segment_length).min(a));
        start += segment_length;
        if hi <= lo {
            continue;
        }
        for (&node, &weight) in GL5_NODES.iter().zip(GL5_WEIGHTS.iter()) {
            let x = lo + node * (hi - lo);
            let dx = weight * (hi - lo);
            let props = eval_section_props_at(model, frame, sec, mat, x / lf, lf);
            // Section resultants of the load beyond x, about the section.
            let m3 = (a - x) * force[1] + moment[2];
            let m2 = -(a - x) * force[2] + moment[1];
            let arm = lf - x;
            if e * props.area > 1e-18 {
                free[0] += force[0] / (e * props.area) * dx;
            }
            if g * props.j > 1e-18 {
                free[3] += moment[0] / (g * props.j) * dx;
            }
            if e * props.i33 > 1e-18 {
                free[1] += m3 * arm / (e * props.i33) * dx;
                free[5] += m3 / (e * props.i33) * dx;
                if g * props.as2 > 1e-14 {
                    free[1] += force[1] / (g * props.as2) * dx;
                }
            }
            if e * props.i22 > 1e-18 {
                free[2] -= m2 * arm / (e * props.i22) * dx;
                free[4] += m2 / (e * props.i22) * dx;
                if g * props.as3 > 1e-14 {
                    free[2] += force[2] / (g * props.as3) * dx;
                }
            }
        }
    }

    // Force of joint I on the member with J free: minus the load carried to I.
    let at_i = [
        -force[0],
        -force[1],
        -force[2],
        -moment[0],
        -(moment[1] - a * force[2]),
        -(moment[2] + a * force[1]),
    ];
    for row in 0..6 {
        fef[row] += at_i[row];
    }
    for row in 0..12 {
        fef[row] -= (0..6).map(|c| stiffness[row][6 + c] * free[c]).sum::<f64>();
    }
}

/// Computes the uncondensed flexible fixed-end force vector $\mathbf{f}_{\text{FEF}}$ ($12 \times 1$)
/// for a given `LoadPattern` and scale factor `sf`.
pub fn compute_flexible_fef_for_pattern(
    model: &SapModel,
    frame: &FrameObj,
    sec: &FrameSection,
    mat: &Material,
    pt_i: &PointObj,
    pt_j: &PointObj,
    e_axes: &[[f64; 3]; 3],
    lf: f64,
    pat_name: &str,
    self_wt_mult: f64,
    sf: f64,
) -> [f64; 12] {
    let mut fef = [0.0f64; 12];
    if sf.abs() <= 1e-18 {
        return fef;
    }

    let e = mat.e;
    let g = if mat.g > 1e-12 {
        mat.g
    } else {
        mat.e / (2.0 * (1.0 + mat.u))
    };
    let p_mid = eval_section_props_at(model, frame, sec, mat, 0.5, lf);
    let phi2 = if g * p_mid.as2 > 1e-14 {
        (12.0 * e * p_mid.i33) / (g * p_mid.as2 * lf * lf)
    } else {
        0.0
    };
    let phi3 = if g * p_mid.as3 > 1e-14 {
        (12.0 * e * p_mid.i22) / (g * p_mid.as3 * lf * lf)
    } else {
        0.0
    };

    // A varying section needs its own fixed-end forces; the prismatic closed
    // forms would give the right load resultant with the wrong end moments.
    let non_prismatic = sec.prop_type == FramePropType::Variable && !sec.non_prismatic.is_empty();
    let varying_stiffness =
        non_prismatic.then(|| compute_flexible_local_stiffness(model, frame, sec, mat, lf));
    let add_point_load_fef =
        |fef: &mut [f64; 12], my_type: i32, local_vec: [f64; 3], s: f64, lf: f64, phi2: f64, phi3: f64| {
            match &varying_stiffness {
                Some(stiffness) => add_point_load_fef_non_prismatic(
                    fef, model, frame, sec, mat, stiffness, my_type, local_vec, s, lf,
                ),
                None => add_point_load_fef(fef, my_type, local_vec, s, lf, phi2, phi3),
            }
        };

    // 1. Self-weight (downward -Z) and Gravity multipliers
    let mut grav_mult = [0.0, 0.0, -self_wt_mult];
    if let Some(gm) = frame.loads_gravity.get(pat_name) {
        grav_mult[0] += gm[0];
        grav_mult[1] += gm[1];
        grav_mult[2] += gm[2];
    }
    if grav_mult[0].abs() > 1e-15 || grav_mult[1].abs() > 1e-15 || grav_mult[2].abs() > 1e-15 {
        let local_g_unit = global_vec_to_local(grav_mult, e_axes);
        if non_prismatic {
            // Exact non-prismatic self-weight integration
            let segs = resolve_non_prismatic_segments(&sec.non_prismatic, lf);
            let mut x_start = 0.0;
            for (_seg, seg_len) in segs {
                for (&xi_g, &w_g) in GL5_NODES.iter().zip(GL5_WEIGHTS.iter()) {
                    let x = x_start + xi_g * seg_len;
                    let s = x / lf;
                    let dx = w_g * seg_len;
                    let p_st = eval_section_props_at(model, frame, sec, mat, s, lf);
                    let w_elem = p_st.weight_per_len * dx * sf;
                    let dp = [
                        local_g_unit[0] * w_elem,
                        local_g_unit[1] * w_elem,
                        local_g_unit[2] * w_elem,
                    ];
                    add_point_load_fef(&mut fef, 1, dp, s, lf, phi2, phi3);
                }
                x_start += seg_len;
            }
        } else {
            let w_len = p_mid.weight_per_len * sf;
            for (&xi_g, &w_g) in GL5_NODES.iter().zip(GL5_WEIGHTS.iter()) {
                let d_len = w_g * lf;
                let dp = [
                    local_g_unit[0] * w_len * d_len,
                    local_g_unit[1] * w_len * d_len,
                    local_g_unit[2] * w_len * d_len,
                ];
                add_point_load_fef(&mut fef, 1, dp, xi_g, lf, phi2, phi3);
            }
        }
    }

    // 2. Distributed & Trapezoidal Loads (`loads_dist`)
    for ld in &frame.loads_dist {
        if ld.load_pat != pat_name {
            continue;
        }
        let s1 = ld.rel_dist1.clamp(0.0, 1.0);
        let s2 = ld.rel_dist2.clamp(0.0, 1.0);
        if s2 <= s1 + 1e-12 {
            continue;
        }
        let ds = s2 - s1;
        for (&xi_g, &w_g) in GL5_NODES.iter().zip(GL5_WEIGHTS.iter()) {
            let s = s1 + xi_g * ds;
            let q_val = (ld.val1 + (ld.val2 - ld.val1) * xi_g) * sf;
            let d_len = w_g * ds * lf;
            let q_local = distributed_load_dir_to_local(ld.dir, q_val * d_len, e_axes, ld.my_type);
            add_point_load_fef(&mut fef, ld.my_type, q_local, s, lf, phi2, phi3);
        }
    }

    // 3. Point Span Loads (`loads_point`)
    for lp in &frame.loads_point {
        if lp.load_pat != pat_name {
            continue;
        }
        let p_local = resolve_load_dir_to_local(lp.dir, lp.val * sf, e_axes, false);
        add_point_load_fef(&mut fef, lp.my_type, p_local, lp.rel_dist, lf, phi2, phi3);
    }

    // 4. Thermal eigenstrains: first integrate the stress-free end motion,
    // then restrain it with the actual flexible stiffness. This is compatible
    // with finite shear stiffness and variable sections; midpoint EI formulas
    // are not sufficient for a longitudinally varying thermal curvature.
    let mut thermal_free = [0.;12];
    for lt in &frame.loads_temp {
        if lt.load_pat != pat_name { continue; }
        let (pat_i, pat_j) = if let Some(pname) = &lt.pattern_name {
            (pt_i.pattern_values.get(pname).copied().unwrap_or(1.0),
             pt_j.pattern_values.get(pname).copied().unwrap_or(1.0))
        } else { (1.,1.) };
        let strain_i = mat.a * lt.val * pat_i * sf;
        let strain_j = mat.a * lt.val * pat_j * sf;
        let rotation = 0.5 * lf * (strain_i+strain_j);
        let transverse = -lf*lf*(2.*strain_i+strain_j)/6.;
        match lt.my_type {
            1 => thermal_free[6] += rotation,
            2 => { thermal_free[7] += transverse; thermal_free[11] -= rotation; }
            3 => { thermal_free[8] += transverse; thermal_free[10] += rotation; }
            _ => {}
        }
    }
    if thermal_free.iter().any(|x| *x != 0.) {
        let stiffness = compute_flexible_local_stiffness(model,frame,sec,mat,lf);
        for row in 0..12 {
            fef[row] -= (0..12).map(|col| stiffness[row][col]*thermal_free[col]).sum::<f64>();
        }
    }

    fef
}

/// Evaluates internal section forces $(P, V_2, V_3, T, M_2, M_3)$ and local deflections $(u_1, u_2, u_3)$
/// along output stations of a frame element.
pub fn recover_frame_stations(
    model: &SapModel,
    frame: &FrameObj,
    sec: &FrameSection,
    mat: &Material,
    e_axes: &[[f64; 3]; 3],
    length: f64,
    u_local: &[f64; 12],
    f_end_local: &[f64; 12],
    case_loads: &[(String, f64)],
) -> Vec<FrameStationResult> {
    let n_sta = frame.min_stations.max(3);
    let mut stations = Vec::with_capacity(n_sta);

    // End forces at x = 0 in SAP2000 internal section sign convention:
    // P(0) = -f_end[0] (+ tension, - compression)
    // V2(0) = -f_end[1]
    // V3(0) = -f_end[2]
    // T(0) = -f_end[3]
    // M2(0) = -f_end[4]
    // M3(0) = -f_end[5]
    let p_0 = -f_end_local[0];
    let v2_0 = -f_end_local[1];
    let v3_0 = -f_end_local[2];
    let t_0 = -f_end_local[3];
    let m2_0 = -f_end_local[4];
    let m3_0 = -f_end_local[5];

    let p_j = f_end_local[6];
    let v2_j = f_end_local[7];
    let v3_j = f_end_local[8];
    let t_j = f_end_local[9];
    let m2_j = f_end_local[10];
    let m3_j = f_end_local[11];
    let _ = (p_j, v2_j, v3_j, t_j);

    let mut property_breaks = vec![0.,1.];
    if sec.prop_type == FramePropType::Variable {
        let mut position=0.;
        for (_, segment_length) in resolve_non_prismatic_segments(&sec.non_prismatic,length) {
            position += segment_length;
            property_breaks.push((position/length).clamp(0.,1.));
        }
    }
    property_breaks.sort_by(f64::total_cmp);
    property_breaks.dedup();
    // Use identical segment-aware gravity quadrature for partial and full-span
    // force/moment integrals. Midpoint weight gives the wrong first moment for
    // tapered sections and cannot be used for the end-equilibrium correction.
    let gravity_integrals = |x: f64, direction: [f64;3], scale: f64| {
        let mut sum=[0.;5]; // axial, shear2, shear3, moment2, moment3
        for span in property_breaks.windows(2) {
            let lo=span[0]*length;let hi=(span[1]*length).min(x);
            if hi <= lo { continue; }
            for (&node,&weight) in GL5_NODES.iter().zip(GL5_WEIGHTS.iter()) {
                let at=lo+node*(hi-lo);
                let props=eval_section_props_at(model,frame,sec,mat,at/length,length);
                let dw=props.weight_per_len*weight*(hi-lo)*scale;
                for d in 0..3 { sum[d] += direction[d]*dw; }
                sum[3] -= (x-at)*direction[2]*dw;
                sum[4] += (x-at)*direction[1]*dw;
            }
        }
        sum
    };
    let section_forces = |s: f64| {
        let idx = usize::from(s > 0.0);
        let x = s * length;

        // Accumulate span load integrals from 0 to x
        let mut int_f = [0.0f64; 3];
        let mut int_m1 = 0.0f64;
        let mut int_m2 = 0.0f64;
        let mut int_m3 = 0.0f64;

        for (pat_name, sf) in case_loads {
            if sf.abs() <= 1e-18 {
                continue;
            }
            let self_wt_mult = model
                .load_patterns
                .get(pat_name)
                .map(|lp| lp.self_wt_multiplier)
                .unwrap_or(0.0);

            // Self-weight & gravity on [0, x]
            let mut grav_mult = [0.0, 0.0, -self_wt_mult];
            if let Some(gm) = frame.loads_gravity.get(pat_name) {
                grav_mult[0] += gm[0];
                grav_mult[1] += gm[1];
                grav_mult[2] += gm[2];
            }
            if grav_mult[0].abs() > 1e-15
                || grav_mult[1].abs() > 1e-15
                || grav_mult[2].abs() > 1e-15
            {
                let local_g = global_vec_to_local(grav_mult, e_axes);
                let integrals=gravity_integrals(x,local_g,*sf);
                for d in 0..3 { int_f[d] += integrals[d]; }
                int_m2 += integrals[3];
                int_m3 += integrals[4];
            }

            // Distributed loads on [0, x]
            for ld in &frame.loads_dist {
                if &ld.load_pat != pat_name {
                    continue;
                }
                let s1 = ld.rel_dist1.clamp(0.0, 1.0);
                let s2 = ld.rel_dist2.clamp(0.0, 1.0);
                if s <= s1 || s2 <= s1 + 1e-12 {
                    continue;
                }
                let s_hi = s.min(s2);
                let ds_active = s_hi - s1;
                for (&xi_g, &w_g) in GL5_NODES.iter().zip(GL5_WEIGHTS.iter()) {
                    let s_eval = s1 + xi_g * ds_active;
                    let x_eval = s_eval * length;
                    let frac = (s_eval - s1) / (s2 - s1);
                    let q_val = (ld.val1 + (ld.val2 - ld.val1) * frac) * sf;
                    let d_len = w_g * ds_active * length;
                    let dq = distributed_load_dir_to_local(ld.dir, q_val * d_len, e_axes, ld.my_type);
                    if ld.my_type == 1 {
                        int_f[0] += dq[0];
                        int_f[1] += dq[1];
                        int_f[2] += dq[2];
                        int_m3 += (x - x_eval) * dq[1];
                        int_m2 -= (x - x_eval) * dq[2];
                    } else if ld.my_type == 2 {
                        // An applied moment about local 2 or 3 lowers the
                        // section moment beyond it (moment equilibrium of the
                        // part of the member before the section).
                        int_m1 += dq[0];
                        int_m2 -= dq[1];
                        int_m3 -= dq[2];
                    }
                }
            }

            // Point loads on [0, x]
            for lp in &frame.loads_point {
                if &lp.load_pat != pat_name {
                    continue;
                }
                if lp.rel_dist <= s + 1e-9 && (idx > 0 || lp.rel_dist <= 1e-9) {
                    let x_p = lp.rel_dist * length;
                    let dp = resolve_load_dir_to_local(lp.dir, lp.val * sf, e_axes, false);
                    if lp.my_type == 1 {
                        int_f[0] += dp[0];
                        int_f[1] += dp[1];
                        int_f[2] += dp[2];
                        int_m3 += (x - x_p) * dp[1];
                        int_m2 -= (x - x_p) * dp[2];
                    } else if lp.my_type == 2 {
                        int_m1 += dp[0];
                        int_m2 -= dp[1];
                        int_m3 -= dp[2];
                    }
                }
            }
        }

        let p = p_0 - int_f[0];
        let v2 = v2_0 - int_f[1];
        let v3 = v3_0 - int_f[2];
        let t = t_0 - int_m1;
        // Include linear interpolation of any P-Delta end moment difference so M3(L) == m3_j and M2(L) == m2_j
        let (m3_span_l, m2_span_l) = {
            let mut m3_l = 0.0;
            let mut m2_l = 0.0;
            for (pat_name, sf) in case_loads {
                let self_wt_mult = model
                    .load_patterns
                    .get(pat_name)
                    .map(|lp| lp.self_wt_multiplier)
                    .unwrap_or(0.0);
                let mut grav_mult = [0.0, 0.0, -self_wt_mult];
                if let Some(gm) = frame.loads_gravity.get(pat_name) {
                    grav_mult[0] += gm[0];
                    grav_mult[1] += gm[1];
                    grav_mult[2] += gm[2];
                }
                let local_g = global_vec_to_local(grav_mult, e_axes);
                let integrals=gravity_integrals(length,local_g,*sf);
                m2_l += integrals[3];
                m3_l += integrals[4];
                for ld in &frame.loads_dist {
                    if &ld.load_pat == pat_name {
                        let s1 = ld.rel_dist1.clamp(0.0, 1.0);
                        let s2 = ld.rel_dist2.clamp(0.0, 1.0);
                        if s2 > s1 + 1e-12 {
                            let ds = s2 - s1;
                            for (&xi_g, &w_g) in GL5_NODES.iter().zip(GL5_WEIGHTS.iter()) {
                                let se = s1 + xi_g * ds;
                                let qv = (ld.val1 + (ld.val2 - ld.val1) * xi_g) * sf;
                                let dq = distributed_load_dir_to_local(
                                    ld.dir,
                                    qv * w_g * ds * length,
                                    e_axes,
                                    ld.my_type,
                                );
                                if ld.my_type == 1 {
                                    m3_l += (1.0 - se) * length * dq[1];
                                    m2_l -= (1.0 - se) * length * dq[2];
                                } else if ld.my_type == 2 {
                                    m2_l -= dq[1];
                                    m3_l -= dq[2];
                                }
                            }
                        }
                    }
                }
                for lp in &frame.loads_point {
                    if &lp.load_pat == pat_name {
                        let dp = resolve_load_dir_to_local(lp.dir, lp.val * sf, e_axes, false);
                        if lp.my_type == 1 {
                            m3_l += (1.0 - lp.rel_dist) * length * dp[1];
                            m2_l -= (1.0 - lp.rel_dist) * length * dp[2];
                        } else if lp.my_type == 2 {
                            m2_l -= dp[1];
                            m3_l -= dp[2];
                        }
                    }
                }
            }
            (m3_l, m2_l)
        };
        let m3_static_end = m3_0 - v2_0 * length + m3_span_l;
        let m2_static_end = m2_0 + v3_0 * length + m2_span_l;
        let pdelta_m3_corr = (m3_j - m3_static_end) * s;
        let pdelta_m2_corr = (m2_j - m2_static_end) * s;
        let m3 = m3_0 - v2_0 * x + int_m3 + pdelta_m3_corr;
        let m2 = m2_0 + v3_0 * x + int_m2 + pdelta_m2_corr;

        [p, v2, v3, t, m2, m3]
    };

    // Integrate equilibrium section forces rather than interpolating only the
    // nodal DOFs. The latter omits the particular deflection caused by span loads
    // (a fixed-fixed uniformly loaded beam would incorrectly plot as straight).
    // Split integration at load discontinuities so point and partial span loads
    // retain their exact piecewise-polynomial integrals for prismatic members.
    let mut breaks = property_breaks.clone();
    for load in &frame.loads_dist {
        breaks.extend([load.rel_dist1.clamp(0.,1.),load.rel_dist2.clamp(0.,1.)]);
    }
    for load in &frame.loads_point { breaks.push(load.rel_dist.clamp(0.,1.)); }
    breaks.sort_by(f64::total_cmp);
    breaks.dedup();
    // Thermal axial strain and transverse curvature are independent of the
    // elastic section forces. Positive gradients lengthen the positive local
    // face, giving negative transverse curvature in either bending plane.
    let thermal_at = |s: f64| {
        let mut strain = [0.;3];
        for load in &frame.loads_temp {
            if !(1..=3).contains(&load.my_type) { continue; }
            let scale: f64 = case_loads.iter().filter(|(p,_)| p == &load.load_pat).map(|(_,sf)| sf).sum();
            let profile = if let Some(pattern) = &load.pattern_name {
                let i = model.points[&frame.point_i].pattern_values.get(pattern).copied().unwrap_or(1.);
                let j = model.points[&frame.point_j].pattern_values.get(pattern).copied().unwrap_or(1.);
                i+(j-i)*s
            } else { 1. };
            let axis=(load.my_type-1) as usize;
            strain[axis] += if axis == 0 { 1. } else { -1. } * mat.a*load.val*scale*profile;
        }
        strain
    };
    let shear_modulus = if mat.g > 1e-12 { mat.g } else { mat.e/(2.*(1.+mat.u)) };
    let integrate = |s: f64| {
        let x=s*length;
        let mut integral=[0.;3];
        for span in breaks.windows(2) {
            let lo=span[0]; let hi=span[1].min(s);
            if hi <= lo { continue; }
            for (&node,&weight) in GL5_NODES.iter().zip(GL5_WEIGHTS.iter()) {
                let at=lo+node*(hi-lo);
                let ds=weight*(hi-lo)*length;
                let [p,v2,v3,_,m2,m3]=section_forces(at);
                let props=eval_section_props_at(model,frame,sec,mat,at,length);
                if props.area > 0. { integral[0] += ds*p/(mat.e*props.area); }
                if props.i33 > 0. { integral[1] += ds*(x-at*length)*m3/(mat.e*props.i33); }
                if props.i22 > 0. { integral[2] -= ds*(x-at*length)*m2/(mat.e*props.i22); }
                if props.as2 > 0. { integral[1] += ds*v2/(shear_modulus*props.as2); }
                if props.as3 > 0. { integral[2] += ds*v3/(shear_modulus*props.as3); }
                let thermal=thermal_at(at);
                integral[0] += ds*thermal[0];
                integral[1] += ds*(x-at*length)*thermal[1];
                integral[2] += ds*(x-at*length)*thermal[2];
            }
        }
        integral
    };
    // Rigid-zone kinematics need a separate mapping from flexible-face output
    // stations to joint locations; retain previous interpolation for that path.
    let integrate_deflection = frame.end_offset_i == 0. && frame.end_offset_j == 0.;
    let full = if integrate_deflection { integrate(1.) } else { [0.;3] };

    for idx in 0..n_sta {
        let s = (idx as f64) / ((n_sta - 1) as f64);
        let x = s*length;
        let [p,v2,v3,t,m2,m3] = section_forces(s);
        // Hermite interpolation of local displacements [u1, u2, u3]
        let n1 = 1.0 - 3.0 * s * s + 2.0 * s * s * s;
        let n2 = length * (s - 2.0 * s * s + s * s * s);
        let n3 = 3.0 * s * s - 2.0 * s * s * s;
        let n4 = length * (-s * s + s * s * s);

        let u1_s = (1.0 - s) * u_local[0] + s * u_local[6];
        let u2_s = n1 * u_local[1] + n2 * u_local[5] + n3 * u_local[7] + n4 * u_local[11];
        let u3_s = n1 * u_local[2] - n2 * u_local[4] + n3 * u_local[8] - n4 * u_local[10];

        stations.push(FrameStationResult {
            station: x,
            rel_dist: s,
            p,
            v2,
            v3,
            t,
            // Reported in the SAP2000 convention: a positive bending moment
            // compresses the positive 3 face (CSI Analysis Reference,
            // "Internal Force Output"). The moment vector about local 2 that
            // the element works with internally has the opposite sense; for
            // M3 the two agree.
            m2: -m2,
            m3,
            local_disp: if integrate_deflection {
                let partial=integrate(s);
                std::array::from_fn(|d| (1.-s)*u_local[d]+s*u_local[6+d]+partial[d]-s*full[d])
            } else { [u1_s,u2_s,u3_s] },
        });
    }

    stations
}
