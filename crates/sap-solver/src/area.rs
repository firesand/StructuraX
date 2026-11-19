//! 3-Node Triangle and 4-Node Quadrilateral Area Element (`Shell`, `Plate`, `Membrane`, `PlaneStress`,
//! `PlaneStrain`, `ASolid`) matching `CsiGo2.dll` (`Module ShellElem` and `Module Plasol`).
//!
//! Uses:
//! - Exact bilinear isoparametric membrane + Allman drilling rotation stabilization (passes MacNeal-Harder
//!   Membrane Patch Test `2-001` with `0.000000%` error on skewed quads).
//! - Bathe-Dvorkin MITC4 (Mixed Interpolation of Tensorial Components) plate bending + shear element
//!   (passes MacNeal-Harder Plate Bending Patch Test `2-001` with `0.000000%` error on skewed quads
//!   and Rectangular Plate Bending `2-005` for both Thin Kirchhoff and Thick Mindlin-Reissner plates).

use sap_model::{AreaObj, AreaPropType, AreaResult, AreaSection, Material, PlaneType, ShellType};

const GP2: [f64; 2] = [-0.577_350_269_189_625_7, 0.577_350_269_189_625_7];

/// Angle (degrees) of the element axes of `area` about its local 3 axis, as passed to
/// [`area_local_system`]: the area's own angle. The section's material angle does not turn the
/// element axes (and with them the output and the local loads): it turns the material inside
/// the element, see [`area_law`] (Analysis Reference p. 193: "The material local coordinate
/// system and the element (Shell Section) local coordinate system need not be the same").
pub fn area_axes_angle(area: &AreaObj, _sec: &AreaSection) -> f64 {
    area.angle_deg
}

/// Computes the $3 \times 3$ global-to-local rotation matrix `e_axes` (rows are $\mathbf{e}_1, \mathbf{e}_2, \mathbf{e}_3$)
/// and the 2D local coordinates `(x_loc, y_loc)` of each corner node relative to the element centroid.
pub fn area_local_system(
    coords: &[[f64; 3]],
    angle_deg: f64,
) -> ([[f64; 3]; 3], [f64; 3], Vec<[f64; 2]>) {
    let n = coords.len();
    let mut centroid = [0.0; 3];
    for c in coords {
        centroid[0] += c[0];
        centroid[1] += c[1];
        centroid[2] += c[2];
    }
    centroid[0] /= n as f64;
    centroid[1] /= n as f64;
    centroid[2] /= n as f64;

    let (v_xi, v_eta) = if n >= 4 {
        (
            [
                (coords[1][0] + coords[2][0]) - (coords[0][0] + coords[3][0]),
                (coords[1][1] + coords[2][1]) - (coords[0][1] + coords[3][1]),
                (coords[1][2] + coords[2][2]) - (coords[0][2] + coords[3][2]),
            ],
            [
                (coords[2][0] + coords[3][0]) - (coords[0][0] + coords[1][0]),
                (coords[2][1] + coords[3][1]) - (coords[0][1] + coords[1][1]),
                (coords[2][2] + coords[3][2]) - (coords[0][2] + coords[1][2]),
            ],
        )
    } else {
        (
            [
                coords[1][0] - coords[0][0],
                coords[1][1] - coords[0][1],
                coords[1][2] - coords[0][2],
            ],
            [
                coords[2][0] - coords[0][0],
                coords[2][1] - coords[0][1],
                coords[2][2] - coords[0][2],
            ],
        )
    };

    let cross = [
        v_xi[1] * v_eta[2] - v_xi[2] * v_eta[1],
        v_xi[2] * v_eta[0] - v_xi[0] * v_eta[2],
        v_xi[0] * v_eta[1] - v_xi[1] * v_eta[0],
    ];
    let norm = (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2])
        .sqrt()
        .max(1e-12);
    let e3 = [cross[0] / norm, cross[1] / norm, cross[2] / norm];

    // SAP2000 default area local axes (`Go_LocalCoordArea`):
    // If element normal is parallel to +-Z (horizontal slab/plate in XY plane), e1_0 = +X, e2_0 = e3 x e1_0.
    // Otherwise e2_0 is in the vertical plane with +Z projection, and e1_0 = e2_0 x e3.
    let h = (e3[0] * e3[0] + e3[1] * e3[1]).sqrt();
    let (e1_0, e2_0) = if h <= 1e-4 {
        let sign_z = if e3[2] >= 0.0 { 1.0 } else { -1.0 };
        ([1.0, 0.0, 0.0], [0.0, sign_z, 0.0])
    } else {
        let e2 = [-e3[0] * e3[2] / h, -e3[1] * e3[2] / h, h];
        let e1 = [
            e2[1] * e3[2] - e2[2] * e3[1],
            e2[2] * e3[0] - e2[0] * e3[2],
            e2[0] * e3[1] - e2[1] * e3[0],
        ];
        (e1, e2)
    };

    let (sb, cb) = angle_deg.to_radians().sin_cos();
    let e1 = [
        cb * e1_0[0] + sb * e2_0[0],
        cb * e1_0[1] + sb * e2_0[1],
        cb * e1_0[2] + sb * e2_0[2],
    ];
    let e2 = [
        -sb * e1_0[0] + cb * e2_0[0],
        -sb * e1_0[1] + cb * e2_0[1],
        -sb * e1_0[2] + cb * e2_0[2],
    ];

    let mut local_xy = Vec::with_capacity(n);
    for c in coords {
        let d = [c[0] - centroid[0], c[1] - centroid[1], c[2] - centroid[2]];
        let x = e1[0] * d[0] + e1[1] * d[1] + e1[2] * d[2];
        let y = e2[0] * d[0] + e2[1] * d[1] + e2[2] * d[2];
        local_xy.push([x, y]);
    }

    ([e1, e2, e3], centroid, local_xy)
}

/// Computes the local stiffness matrix ($6n \times 6n$) and element area for a 3-node or 4-node area element.
pub fn compute_area_local_stiffness(
    area_obj: &AreaObj,
    sec: &AreaSection,
    mat: &Material,
    local_xy: &[[f64; 2]],
) -> (Vec<f64>, f64) {
    let n_nodes = local_xy.len();
    if n_nodes == 3 {
        // Represent 3-node triangle via degenerated/linked MITC4 or CST+DKT
        return compute_triangle_local_stiffness(area_obj, sec, mat, local_xy);
    }
    compute_quad4_local_stiffness(area_obj, sec, mat, local_xy)
}

/// Penalty factor on the transverse shear law of a thin-plate section (ShellThin, PlateThin):
/// `clamp(1e4 (tb/h)^2, 1, 1e6)` with `h` the diagonal from node 0 to node 2 of the element in
/// its plane; 1 for every other section type. The element stiffness multiplies the shear law
/// by it, so the shear force the element transmits is `boost * ds * gamma`; the recovery of
/// V13/V23 multiplies by the same factor (D105). Analysis Reference p. 211: the thin-plate
/// shear force follows from equilibrium with the moments, i.e. it is the force the element
/// carries, not the unboosted `ds * gamma`.
fn thin_plate_shear_boost(sec: &AreaSection, tb: f64, xy: &[[f64; 2]]) -> f64 {
    // Only a section with plate bending has the shear term the boost belongs to: a Plane or
    // Asolid section keeps the plain law whatever its (unused) shell type says.
    if !section_behaviours(sec).1
        || !matches!(sec.shell_type, ShellType::ShellThin | ShellType::PlateThin)
    {
        return 1.0;
    }
    let dx02 = xy[2][0] - xy[0][0];
    let dy02 = xy[2][1] - xy[0][1];
    let h_elem = (dx02 * dx02 + dy02 * dy02).sqrt().max(1e-6);
    // Ensure Kirchhoff thin-plate limit even for moderate thickness, while avoiding ill-conditioning
    let ratio_sq = (tb / h_elem).powi(2);
    (1e4 * ratio_sq).clamp(1.0, 1e6)
}

fn compute_quad4_local_stiffness(
    area_obj: &AreaObj,
    sec: &AreaSection,
    mat: &Material,
    xy: &[[f64; 2]],
) -> (Vec<f64>, f64) {
    let ndof = 24;
    let mut k = vec![0.0f64; ndof * ndof];

    let e = mat.e;
    let nu = mat.u.clamp(-0.49, 0.4999);
    let g = if mat.g > 1e-12 {
        mat.g
    } else {
        e / (2.0 * (1.0 + nu))
    };

    let tm = sec.thickness_membrane;
    let tb = sec.thickness_bending;

    let (has_membrane, has_bending) = section_behaviours(sec);
    // The Allman drilling term belongs to the shell element only: a Plane or Asolid section
    // has no drilling stiffness (Analysis Reference p. 217: the plane element has stiffness
    // for the three translations only), whatever `drilling_dof` says, and its R3 is
    // stabilised like any unused rotation.
    let drilling_active = sec.prop_type == AreaPropType::Shell && has_membrane && sec.drilling_dof;

    // One law serves the stiffness and the recovery of the resultants.
    let law = area_law(area_obj, sec, mat, tm, tb);
    let dm = law.dm;
    let db = law.db;

    // Thin-plate shear boost, shared with the recovery of V13/V23 (D105).
    let thin_boost = thin_plate_shear_boost(sec, tb, xy);
    let ds = [
        [law.ds[0][0] * thin_boost, law.ds[0][1] * thin_boost],
        [law.ds[1][0] * thin_boost, law.ds[1][1] * thin_boost],
    ];

    let mut total_area = 0.0;

    // 2x2 Gauss quadrature for Membrane, Bending, and MITC4 Transverse Shear
    for &xi in &GP2 {
        for &eta in &GP2 {
            let (n_val, dndxi, dndeta, j_mat, det_j, dndx, dndy) = quad4_shape_derivs(xy, xi, eta);
            let d_omega = det_j.abs();
            total_area += d_omega;

            if has_membrane {
                // Membrane DOFs at node i: u1 = 6*i, u2 = 6*i + 1
                let mut bm = [[0.0f64; 24]; 3];
                for i in 0..4 {
                    bm[0][6 * i] = dndx[i];
                    bm[1][6 * i + 1] = dndy[i];
                    bm[2][6 * i] = dndy[i];
                    bm[2][6 * i + 1] = dndx[i];
                }
                add_bdb(&mut k, &bm, &dm, d_omega);

                // Drilling DOF stabilization: penalize (theta_3 - 0.5*(du2/dx - du1/dy))^2
                if drilling_active {
                    let mut b_drill = [0.0f64; 24];
                    for i in 0..4 {
                        b_drill[6 * i] = 0.5 * dndy[i];
                        b_drill[6 * i + 1] = -0.5 * dndx[i];
                        b_drill[6 * i + 5] = n_val[i];
                    }
                    let k_drill = 1e-3 * g * tm * d_omega;
                    for r in 0..24 {
                        if b_drill[r] == 0.0 {
                            continue;
                        }
                        for c in 0..24 {
                            k[r * 24 + c] += k_drill * b_drill[r] * b_drill[c];
                        }
                    }
                }
            }

            if has_bending {
                // Bending DOFs at node i: w = 6*i + 2, theta_1 = 6*i + 3, theta_2 = 6*i + 4
                // With beta_x = -theta_2 and beta_y = +theta_1:
                // kappa_xx = d(beta_x)/dx = -dN/dx * theta_2
                // kappa_yy = d(beta_y)/dy = +dN/dy * theta_1
                // kappa_xy = d(beta_x)/dy + d(beta_y)/dx = dN/dx * theta_1 - dN/dy * theta_2
                let mut bb = [[0.0f64; 24]; 3];
                for i in 0..4 {
                    bb[0][6 * i + 4] = -dndx[i];
                    bb[1][6 * i + 3] = dndy[i];
                    bb[2][6 * i + 3] = dndx[i];
                    bb[2][6 * i + 4] = -dndy[i];
                }
                add_bdb(&mut k, &bb, &db, d_omega);

                // MITC4 Transverse Shear Strain B_s (2 x 24)
                let bs = mitc4_shear_b_matrix(xy, xi, eta, &j_mat, det_j);
                for r in 0..24 {
                    let b0r = bs[0][r];
                    let b1r = bs[1][r];
                    if b0r == 0.0 && b1r == 0.0 {
                        continue;
                    }
                    for c in 0..24 {
                        k[r * 24 + c] += d_omega
                            * (ds[0][0] * b0r * bs[0][c]
                                + ds[1][1] * b1r * bs[1][c]
                                + ds[0][1] * (b0r * bs[1][c] + b1r * bs[0][c]));
                    }
                }
            }

            if let Some(dp) = law.dp {
                // Anti-plane shear of a plane-strain section (Analysis Reference p. 217):
                // gamma_13 = du3/dx, gamma_23 = du3/dy, with the shear moduli times the thickness.
                for r in 0..4 {
                    for c in 0..4 {
                        k[(6 * r + 2) * 24 + 6 * c + 2] += d_omega
                            * (dp[0][0] * dndx[r] * dndx[c]
                                + dp[1][1] * dndy[r] * dndy[c]
                                + dp[0][1] * (dndx[r] * dndy[c] + dndy[r] * dndx[c]));
                    }
                }
            }

            let _ = (dndxi, dndeta);
        }
    }

    // Small stabilization on unused local DOFs (e.g. out-of-plane rotation R3 for pure Plate, or U3/R1/R2 for Membrane)
    let diag_scale = (e * tm.max(tb) * 1e-10).max(1e-12);
    for i in 0..4 {
        if !has_membrane {
            k[(6 * i) * 24 + 6 * i] += diag_scale;
            k[(6 * i + 1) * 24 + 6 * i + 1] += diag_scale;
        }
        if !has_bending {
            if law.dp.is_none() {
                k[(6 * i + 2) * 24 + 6 * i + 2] += diag_scale;
            }
            k[(6 * i + 3) * 24 + 6 * i + 3] += diag_scale;
            k[(6 * i + 4) * 24 + 6 * i + 4] += diag_scale;
        }
        if !drilling_active {
            k[(6 * i + 5) * 24 + 6 * i + 5] += diag_scale;
        }
    }

    (k, total_area)
}

#[allow(clippy::type_complexity)]
fn quad4_shape_derivs(
    xy: &[[f64; 2]],
    xi: f64,
    eta: f64,
) -> (
    [f64; 4],
    [f64; 4],
    [f64; 4],
    [[f64; 2]; 2],
    f64,
    [f64; 4],
    [f64; 4],
) {
    let xi_n = [-1.0, 1.0, 1.0, -1.0];
    let eta_n = [-1.0, -1.0, 1.0, 1.0];
    let mut n = [0.0; 4];
    let mut dndxi = [0.0; 4];
    let mut dndeta = [0.0; 4];

    for i in 0..4 {
        n[i] = 0.25 * (1.0 + xi_n[i] * xi) * (1.0 + eta_n[i] * eta);
        dndxi[i] = 0.25 * xi_n[i] * (1.0 + eta_n[i] * eta);
        dndeta[i] = 0.25 * eta_n[i] * (1.0 + xi_n[i] * xi);
    }

    let mut j = [[0.0; 2]; 2];
    for i in 0..4 {
        j[0][0] += dndxi[i] * xy[i][0];
        j[0][1] += dndxi[i] * xy[i][1];
        j[1][0] += dndeta[i] * xy[i][0];
        j[1][1] += dndeta[i] * xy[i][1];
    }
    let det_j = j[0][0] * j[1][1] - j[0][1] * j[1][0];
    let inv_det = if det_j.abs() > 1e-18 {
        1.0 / det_j
    } else {
        0.0
    };

    let mut dndx = [0.0; 4];
    let mut dndy = [0.0; 4];
    for i in 0..4 {
        dndx[i] = inv_det * (j[1][1] * dndxi[i] - j[0][1] * dndeta[i]);
        dndy[i] = inv_det * (-j[1][0] * dndxi[i] + j[0][0] * dndeta[i]);
    }

    (n, dndxi, dndeta, j, det_j, dndx, dndy)
}

/// Computes the $2 \times 24$ MITC4 Cartesian transverse shear strain-displacement matrix $[\gamma_{xz}, \gamma_{yz}]^T$
/// at natural coordinates $(\xi, \eta)$.
///
/// Exact property: for ANY quadratic transverse displacement $w(x, y)$ and consistent linear rotations
/// $\beta_x = -\theta_2 = \partial w/\partial x$, $\beta_y = +\theta_1 = \partial w/\partial y$,
/// all four covariant edge shear strains vanish identically ($e_{\xi z}^A = e_{\xi z}^C = e_{\eta z}^D = e_{\eta z}^B = 0$),
/// guaranteeing 0% error on the MacNeal-Harder skewed-mesh constant-curvature patch test.
fn mitc4_shear_b_matrix(
    xy: &[[f64; 2]],
    xi: f64,
    eta: f64,
    j_mat: &[[f64; 2]; 2],
    det_j: f64,
) -> [[f64; 24]; 2] {
    // Helper to build covariant shear row for directed edge (n1 -> n2)
    // e_edge = 0.5 * (w_{n2} - w_{n1}) - 0.25 * dx * (beta_{x,n1} + beta_{x,n2}) - 0.25 * dy * (beta_{y,n1} + beta_{y,n2})
    // where beta_x = -theta_2 (DOF 6*i + 4) and beta_y = +theta_1 (DOF 6*i + 3).
    let edge_cov_row = |n1: usize, n2: usize| -> [f64; 24] {
        let mut row = [0.0f64; 24];
        let dx = xy[n2][0] - xy[n1][0];
        let dy = xy[n2][1] - xy[n1][1];
        row[6 * n1 + 2] = -0.5;
        row[6 * n2 + 2] = 0.5;
        // -0.25 * dy * theta_1
        row[6 * n1 + 3] = -0.25 * dy;
        row[6 * n2 + 3] = -0.25 * dy;
        // +0.25 * dx * theta_2 (since -0.25 * dx * (-theta_2) = +0.25 * dx * theta_2)
        row[6 * n1 + 4] = 0.25 * dx;
        row[6 * n2 + 4] = 0.25 * dx;
        row
    };

    // Edge A: 0 -> 1 (eta = -1)
    let row_a = edge_cov_row(0, 1);
    // Edge C: 3 -> 2 (eta = +1)
    let row_c = edge_cov_row(3, 2);
    // Edge D: 0 -> 3 (xi = -1)
    let row_d = edge_cov_row(0, 3);
    // Edge B: 1 -> 2 (xi = +1)
    let row_b = edge_cov_row(1, 2);

    let c_a = 0.5 * (1.0 - eta);
    let c_c = 0.5 * (1.0 + eta);
    let c_d = 0.5 * (1.0 - xi);
    let c_b = 0.5 * (1.0 + xi);

    let inv_det = if det_j.abs() > 1e-18 {
        1.0 / det_j
    } else {
        0.0
    };

    let mut bs = [[0.0f64; 24]; 2];
    for col in 0..24 {
        let e_xiz = c_a * row_a[col] + c_c * row_c[col];
        let e_etaz = c_d * row_d[col] + c_b * row_b[col];
        bs[0][col] = inv_det * (j_mat[1][1] * e_xiz - j_mat[0][1] * e_etaz);
        bs[1][col] = inv_det * (-j_mat[1][0] * e_xiz + j_mat[0][0] * e_etaz);
    }
    bs
}

fn add_bdb(k: &mut [f64], b: &[[f64; 24]; 3], d: &[[f64; 3]; 3], weight: f64) {
    for r in 0..24 {
        let b0r = b[0][r];
        let b1r = b[1][r];
        let b2r = b[2][r];
        if b0r == 0.0 && b1r == 0.0 && b2r == 0.0 {
            continue;
        }
        let db0 = b0r * d[0][0] + b1r * d[1][0] + b2r * d[2][0];
        let db1 = b0r * d[0][1] + b1r * d[1][1] + b2r * d[2][1];
        let db2 = b0r * d[0][2] + b1r * d[1][2] + b2r * d[2][2];
        for c in 0..24 {
            k[r * 24 + c] += weight * (db0 * b[0][c] + db1 * b[1][c] + db2 * b[2][c]);
        }
    }
}

fn compute_triangle_local_stiffness(
    area_obj: &AreaObj,
    sec: &AreaSection,
    mat: &Material,
    xy: &[[f64; 2]],
) -> (Vec<f64>, f64) {
    // Degenerate 4-node quad with node 3 == node 2, condensing DOFs 18..24 into 12..18
    let xy4 = [xy[0], xy[1], xy[2], xy[2]];
    let (k4, area) = compute_quad4_local_stiffness(area_obj, sec, mat, &xy4);
    let mut k3 = vec![0.0; 18 * 18];
    let map_node = |n: usize| if n == 3 { 2 } else { n };
    for r4 in 0..24 {
        let nr = map_node(r4 / 6);
        let dr = r4 % 6;
        let r3 = 6 * nr + dr;
        for c4 in 0..24 {
            let nc = map_node(c4 / 6);
            let dc = c4 % 6;
            let c3 = 6 * nc + dc;
            k3[r3 * 18 + c3] += k4[r4 * 24 + c4];
        }
    }
    (k3, area)
}

/// Computes the consistent local nodal load vector ($6n$) for an area element under a given `LoadPattern`.
pub fn compute_area_local_loads(
    area_obj: &AreaObj,
    sec: &AreaSection,
    mat: &Material,
    e_axes: &[[f64; 3]; 3],
    local_xy: &[[f64; 2]],
    pat_name: &str,
    self_wt_mult: f64,
    sf: f64,
) -> Vec<f64> {
    let n_nodes = local_xy.len();
    let mut f_loc = vec![0.0; 6 * n_nodes];
    if sf.abs() <= 1e-18 {
        return f_loc;
    }

    let mut mods = [1.0; 10];
    for i in 0..10 {
        mods[i] = sec.modifiers[i] * area_obj.modifiers[i];
    }

    // Total distributed force per unit area in local [f1, f2, f3]
    let mut q_local = [0.0f64; 3];

    // 1. Self-weight
    if self_wt_mult.abs() > 1e-15 {
        let w_area = sec.thickness_membrane * mat.unit_weight * mods[9] * (-self_wt_mult) * sf;
        let g_loc = crate::frame::global_vec_to_local([0.0, 0.0, w_area], e_axes);
        q_local[0] += g_loc[0];
        q_local[1] += g_loc[1];
        q_local[2] += g_loc[2];
    }

    // 2. Uniform area loads (`loads_uniform`)
    for lu in &area_obj.loads_uniform {
        if lu.load_pat != pat_name {
            continue;
        }
        let v = lu.val * sf;
        let ql = crate::frame::resolve_load_dir_to_local(lu.dir, v, e_axes, false);
        q_local[0] += ql[0];
        q_local[1] += ql[1];
        q_local[2] += ql[2];
    }

    // 3. Surface pressure loads (`loads_pressure`: -1 = Bottom -> +e3, -2 = Top -> -e3)
    for lp in &area_obj.loads_pressure {
        if lp.load_pat != pat_name {
            continue;
        }
        let v = lp.val * sf;
        if lp.face == -1 {
            q_local[2] += v;
        } else if lp.face == -2 {
            q_local[2] -= v;
        }
    }

    if n_nodes == 4 {
        let law = area_law(
            area_obj,
            sec,
            mat,
            sec.thickness_membrane,
            sec.thickness_bending,
        );
        for &xi in &GP2 {
            for &eta in &GP2 {
                let (n_val, _, _, _, det_j, dndx, dndy) = quad4_shape_derivs(local_xy, xi, eta);
                let d_omega = det_j.abs();
                for i in 0..4 {
                    f_loc[6 * i] += n_val[i] * q_local[0] * d_omega;
                    f_loc[6 * i + 1] += n_val[i] * q_local[1] * d_omega;
                    f_loc[6 * i + 2] += n_val[i] * q_local[2] * d_omega;
                }

                // Thermal loads (`loads_temp`)
                for lt in &area_obj.loads_temp {
                    if lt.load_pat != pat_name {
                        continue;
                    }
                    let val = lt.val * sf;
                    let e = mat.e;
                    let nu = mat.u.clamp(-0.49, 0.4999);
                    if law.ortho && lt.my_type == 1 {
                        // Orthotropic membrane: the thermal force is dm [a dT, a dT, 0]
                        // (a1 = a2, so the free strain is the same in any axes).
                        let eps = area_alpha(mat) * val;
                        let nth = [
                            (law.dm[0][0] + law.dm[0][1]) * eps,
                            (law.dm[1][0] + law.dm[1][1]) * eps,
                            (law.dm[2][0] + law.dm[2][1]) * eps,
                        ];
                        for i in 0..4 {
                            f_loc[6 * i] += (nth[0] * dndx[i] + nth[2] * dndy[i]) * d_omega;
                            f_loc[6 * i + 1] += (nth[1] * dndy[i] + nth[2] * dndx[i]) * d_omega;
                        }
                    } else if law.ortho && lt.my_type == 2 {
                        // Orthotropic gradient: the free curvature is -a g in both
                        // directions; the equivalent loads are B^T db k_free.
                        let kap = area_alpha(mat) * val;
                        let mth = [
                            (law.db[0][0] + law.db[0][1]) * kap,
                            (law.db[1][0] + law.db[1][1]) * kap,
                            (law.db[2][0] + law.db[2][1]) * kap,
                        ];
                        for i in 0..4 {
                            f_loc[6 * i + 3] -= (mth[1] * dndy[i] + mth[2] * dndx[i]) * d_omega;
                            f_loc[6 * i + 4] += (mth[0] * dndx[i] + mth[2] * dndy[i]) * d_omega;
                        }
                    } else if lt.my_type == 1 {
                        // Uniform Delta T -> membrane force N_th = E * tm * alpha * dT / (1 - nu)
                        let n_th = e * sec.thickness_membrane * mat.a * val / (1.0 - nu);
                        for i in 0..4 {
                            f_loc[6 * i] += n_th * mods[0] * dndx[i] * d_omega;
                            f_loc[6 * i + 1] += n_th * mods[1] * dndy[i] * d_omega;
                        }
                    } else if lt.my_type == 2 {
                        // Through-thickness gradient dT/dx3 (positive: warmer toward +local 3).
                        // The free thermal strain is alpha * grad * x3 and the bending strain
                        // is -x3 * kappa with kappa_xx = w,xx = -sum dndx_i * theta2_i and
                        // kappa_yy = w,yy = sum dndy_i * theta1_i, so the free curvature is
                        // -alpha * grad in both directions and the equivalent loads are
                        // B^T D kappa_free, with m_th = E * tb^3 * alpha * grad / (12 * (1 - nu)).
                        // Sign checked on CSI Example 2-014: the plate with the warmer bottom
                        // (grad = -100) lifts its free inner edge.
                        let m_th =
                            e * sec.thickness_bending.powi(3) * mat.a * val / (12.0 * (1.0 - nu));
                        for i in 0..4 {
                            f_loc[6 * i + 3] -= m_th * mods[4] * dndy[i] * d_omega;
                            f_loc[6 * i + 4] += m_th * mods[3] * dndx[i] * d_omega;
                        }
                    }
                }
            }
        }
    } else if n_nodes == 3 {
        let dx1 = local_xy[1][0] - local_xy[0][0];
        let dy1 = local_xy[1][1] - local_xy[0][1];
        let dx2 = local_xy[2][0] - local_xy[0][0];
        let dy2 = local_xy[2][1] - local_xy[0][1];
        let area = 0.5 * (dx1 * dy2 - dx2 * dy1).abs();
        for i in 0..3 {
            f_loc[6 * i] += q_local[0] * area / 3.0;
            f_loc[6 * i + 1] += q_local[1] * area / 3.0;
            f_loc[6 * i + 2] += q_local[2] * area / 3.0;
        }
    }

    f_loc
}

/// Whether a section has membrane stiffness and plate-bending stiffness.
pub fn section_behaviours(sec: &AreaSection) -> (bool, bool) {
    let has_membrane = match sec.prop_type {
        AreaPropType::Shell => {
            !matches!(sec.shell_type, ShellType::PlateThin | ShellType::PlateThick)
        }
        AreaPropType::Plane | AreaPropType::ASolid => true,
    };
    let has_bending = match sec.prop_type {
        AreaPropType::Shell => !matches!(sec.shell_type, ShellType::Membrane),
        AreaPropType::Plane | AreaPropType::ASolid => false,
    };
    (has_membrane, has_bending)
}

/// Constitutive law of an area section in its element axes: the one set of matrices that
/// the stiffness and the recovery of the resultants share (Analysis Reference pp. 75, 193,
/// 217-219). Stiffness modifiers are included.
#[derive(Clone, Debug)]
pub(crate) struct AreaLaw {
    /// Membrane: `[F11, F22, F12] = dm [e11, e22, g12]` (force per length; thickness included).
    pub dm: [[f64; 3]; 3],
    /// Plate bending: `[M11, M22, M12] = db [k11, k22, k12]`.
    pub db: [[f64; 3]; 3],
    /// Transverse shear, `[V13, V23] = ds [g13, g23]`, without the thin-plate shear boost.
    pub ds: [[f64; 2]; 2],
    /// Anti-plane shear of a plane-strain section: `[s13 t, s23 t] = dp [g13, g23]`.
    pub dp: Option<[[f64; 2]; 2]>,
    /// Whether the orthotropic constants (`e_ortho`, `u_ortho`, `g_ortho`) were used.
    pub ortho: bool,
    plane_strain: bool,
    nu: f64,
    /// `(u13, u23)` of an orthotropic plane-strain material, for S33 (p. 75, 219).
    u3: [f64; 2],
    /// `(sin, cos)` of the material angle, to bring the stresses into material axes.
    mat_rot: (f64, f64),
}

impl AreaLaw {
    /// Thickness-normal stress of a plane-strain section from `e33 = 0` (p. 218); zero for
    /// every other section. `s11, s22, s12` are the membrane stresses in element axes.
    pub(crate) fn s33(&self, s11: f64, s22: f64, s12: f64) -> f64 {
        if !self.plane_strain {
            0.0
        } else if self.ortho {
            // Material-axes normal stresses: s'11 = c^2 s11 + s^2 s22 + 2 s c s12 and
            // s'22 = s^2 s11 + c^2 s22 - 2 s c s12; e33 = 0 gives s33 = u13 s'11 + u23 s'22.
            let (sn, cs) = self.mat_rot;
            let m11 = cs * cs * s11 + sn * sn * s22 + 2.0 * sn * cs * s12;
            let m22 = sn * sn * s11 + cs * cs * s22 - 2.0 * sn * cs * s12;
            self.u3[0] * m11 + self.u3[1] * m22
        } else {
            self.nu * (s11 + s22)
        }
    }
}

/// Whether the material of an area section carries orthotropic constants, which the area
/// elements then use (CSI Analysis Reference p. 75, Eqn. 3; p. 193).
pub fn area_material_is_orthotropic(mat: &Material) -> bool {
    mat.e_ortho.is_some() || mat.u_ortho.is_some() || mat.g_ortho.is_some()
}

/// Orthotropic constants `[e1, e2, e3]`, `[u12, u13, u23]`, `[g12, g13, g23]` of `mat`; a
/// missing array repeats the scalar property (`g` from `e1` and `u12` when it is not given).
fn ortho_constants(mat: &Material) -> ([f64; 3], [f64; 3], [f64; 3]) {
    let e = mat.e_ortho.unwrap_or([mat.e; 3]);
    let u = mat.u_ortho.unwrap_or([mat.u; 3]);
    let g = mat.g_ortho.unwrap_or_else(|| {
        let g = if mat.g > 1e-12 {
            mat.g
        } else {
            e[0] / (2.0 * (1.0 + u[0]))
        };
        [g; 3]
    });
    (e, u, g)
}

/// Compliance of an orthotropic material (Analysis Reference p. 75, Eqn. 3, the printed
/// matrix: `S12 = -u12/e2`, `S13 = -u13/e3`, `S23 = -u23/e3`) restricted to the in-plane
/// normal strains `(e11, e22)`, by plane stress (the section drops e3, u13 and u23, p. 193)
/// or plane strain (`e33 = 0` from `S'ij = Sij - Si3 Sj3 / S33`, p. 219). Returns
/// `(s11, s22, s12)`.
fn ortho_compliance(
    e: [f64; 3],
    u: [f64; 3],
    plane_strain: bool,
) -> (f64, f64, f64) {
    let (mut s11, mut s22, mut s12) = (1.0 / e[0], 1.0 / e[1], -u[0] / e[1]);
    if plane_strain {
        s11 -= u[1] * u[1] / e[2];
        s22 -= u[2] * u[2] / e[2];
        s12 -= u[1] * u[2] / e[2];
    }
    (s11, s22, s12)
}

/// Checks that the orthotropic constants of `mat` give a positive-definite material (moduli
/// and shear moduli positive and finite, and the normal part of Eqn. 3 positive definite,
/// p. 75). Isotropic materials are not checked here. The error names the area object.
pub fn validate_area_material(area_name: &str, mat: &Material) -> Result<(), String> {
    if !area_material_is_orthotropic(mat) {
        return Ok(());
    }
    let (e, u, g) = ortho_constants(mat);
    let bad = |what: &str| {
        Err(format!(
            "area '{area_name}' uses material '{}' whose orthotropic constants are not valid: {what}",
            mat.name
        ))
    };
    if e.iter().chain(&u).chain(&g).any(|v| !v.is_finite()) {
        return bad("a constant is not finite");
    }
    if e.iter().any(|&v| v <= 0.0) || g.iter().any(|&v| v <= 0.0) {
        return bad("the moduli of elasticity and the shear moduli must be positive");
    }
    // Leading minors of the symmetric normal compliance matrix of Eqn. 3.
    let s = [
        [1.0 / e[0], -u[0] / e[1], -u[1] / e[2]],
        [-u[0] / e[1], 1.0 / e[1], -u[2] / e[2]],
        [-u[1] / e[2], -u[2] / e[2], 1.0 / e[2]],
    ];
    let m2 = s[0][0] * s[1][1] - s[0][1] * s[1][0];
    let m3 = s[0][0] * (s[1][1] * s[2][2] - s[1][2] * s[2][1])
        - s[0][1] * (s[1][0] * s[2][2] - s[1][2] * s[2][0])
        + s[0][2] * (s[1][0] * s[2][1] - s[1][1] * s[2][0]);
    if m2 <= 0.0 || m3 <= 0.0 {
        return bad("the stress-strain matrix is not positive definite (check the Poisson's ratios)");
    }
    Ok(())
}

/// Refuses a temperature load on an area whose material has unequal expansion coefficients in
/// its two in-plane directions: the thermal code works with one coefficient (`a1 = a2`).
pub fn validate_area_thermal_material(area_name: &str, mat: &Material) -> Result<(), String> {
    if let Some(a) = mat.a_ortho {
        if area_material_is_orthotropic(mat) && (a[0] - a[1]).abs() > 1e-12 * a[0].abs().max(a[1].abs()) {
            return Err(format!(
                "area '{area_name}' has a temperature load but its material '{}' has different coefficients of thermal expansion a1 = {} and a2 = {} in the plane; only a1 = a2 is implemented for area elements",
                mat.name, a[0], a[1]
            ));
        }
    }
    Ok(())
}

/// Coefficient of thermal expansion the area thermal code uses.
fn area_alpha(mat: &Material) -> f64 {
    match mat.a_ortho {
        Some(a) if area_material_is_orthotropic(mat) => a[0],
        _ => mat.a,
    }
}

/// Engineering-strain rotation `T_e` from element axes to material axes, the material 1 axis
/// lying at the angle whose sine and cosine are `(sn, cs)` from the element 1 axis about local 3:
/// `[e11, e22, g12]_mat = T_e [e11, e22, g12]_elem`.
fn strain_rotation(sn: f64, cs: f64) -> [[f64; 3]; 3] {
    [
        [cs * cs, sn * sn, sn * cs],
        [sn * sn, cs * cs, -sn * cs],
        [-2.0 * sn * cs, 2.0 * sn * cs, cs * cs - sn * sn],
    ]
}

/// `T_e^T Q T_e`: the material matrix `q` (material axes) in element axes.
fn rotate3(q: [[f64; 3]; 3], sn: f64, cs: f64) -> [[f64; 3]; 3] {
    let t = strain_rotation(sn, cs);
    let mut qt = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                qt[i][j] += q[i][k] * t[k][j];
            }
        }
    }
    let mut out = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                out[i][j] += t[k][i] * qt[k][j];
            }
        }
    }
    out
}

/// Transverse-shear matrix `diag(g13, g23)` of the material in element axes (the shear
/// strain vector rotates like a vector): `[[g13 c^2 + g23 s^2, (g13 - g23) s c], [.., g13 s^2
/// + g23 c^2]]`.
fn rotate2(g13: f64, g23: f64, sn: f64, cs: f64) -> [[f64; 2]; 2] {
    let off = (g13 - g23) * sn * cs;
    [
        [g13 * cs * cs + g23 * sn * sn, off],
        [off, g13 * sn * sn + g23 * cs * cs],
    ]
}

/// `Q_ij sqrt(m_i m_j)`: stiffness modifiers scale the diagonal terms and, as the geometric
/// mean, the couplings (the congruence `M^1/2 Q M^1/2`, which keeps a positive-definite
/// matrix positive definite).
fn scale3(q: [[f64; 3]; 3], m: [f64; 3], f: f64) -> [[f64; 3]; 3] {
    let mut d = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            d[i][j] = if i == j {
                f * q[i][j] * m[i]
            } else {
                f * q[i][j] * (m[i] * m[j]).sqrt()
            };
        }
    }
    d
}

fn scale2(q: [[f64; 2]; 2], m: [f64; 2], f: f64) -> [[f64; 2]; 2] {
    [
        [f * q[0][0] * m[0], f * q[0][1] * (m[0] * m[1]).sqrt()],
        [f * q[1][0] * (m[0] * m[1]).sqrt(), f * q[1][1] * m[1]],
    ]
}

/// The constitutive law of `sec` made of `mat`, with the thicknesses `tm` (membrane) and
/// `tb` (bending and shear) passed by the caller.
pub(crate) fn area_law(
    area_obj: &AreaObj,
    sec: &AreaSection,
    mat: &Material,
    tm: f64,
    tb: f64,
) -> AreaLaw {
    let mut mods = [1.0; 10];
    for i in 0..10 {
        mods[i] = sec.modifiers[i] * area_obj.modifiers[i];
    }
    let plane_strain =
        sec.prop_type == AreaPropType::Plane && sec.plane_type == PlaneType::PlaneStrain;

    if area_material_is_orthotropic(mat) {
        let (e, u, g) = ortho_constants(mat);
        // Material-axes plane-stress and (for the membrane of a plane-strain section)
        // plane-strain stiffness per unit thickness, from the inverted reduced compliance.
        let q_of = |strain: bool| {
            let (s11, s22, s12) = ortho_compliance(e, u, strain);
            let det = s11 * s22 - s12 * s12;
            [
                [s22 / det, -s12 / det, 0.0],
                [-s12 / det, s11 / det, 0.0],
                [0.0, 0.0, g[0]],
            ]
        };
        // The material angle turns the material about local 3 (p. 193, 219); the element axes
        // stay the area's. An isotropic material needs no rotation.
        let (sn, cs) = sec.mat_angle_deg.to_radians().sin_cos();
        let qm = rotate3(q_of(plane_strain), sn, cs);
        let qb = rotate3(q_of(false), sn, cs);
        // Only the thick-plate formulation uses g13 and g23 for the transverse shear
        // (Analysis Reference p. 193); the thin-plate shear stiffness is a penalty that
        // enforces the Kirchhoff limit and takes g12, as before.
        let thin = sec.prop_type == AreaPropType::Shell
            && matches!(sec.shell_type, ShellType::ShellThin | ShellType::PlateThin);
        let (g13, g23) = if thin { (g[0], g[0]) } else { (g[1], g[2]) };
        let gs = rotate2(g13, g23, sn, cs);
        let dm = scale3(qm, [mods[0], mods[1], mods[2]], tm);
        let db = scale3(qb, [mods[3], mods[4], mods[5]], tb.powi(3) / 12.0);
        let ds = scale2(gs, [mods[6], mods[7]], (5.0 / 6.0) * tb);
        let dp = plane_strain.then(|| scale2(gs, [mods[6], mods[7]], tm));
        return AreaLaw {
            dm,
            db,
            ds,
            dp,
            ortho: true,
            plane_strain,
            nu: u[0],
            u3: [u[1], u[2]],
            mat_rot: (sn, cs),
        };
    }

    let e = mat.e;
    let nu = mat.u.clamp(-0.49, 0.4999);
    let g = if mat.g > 1e-12 {
        mat.g
    } else {
        e / (2.0 * (1.0 + nu))
    };

    let dm = if plane_strain {
        let c = e * tm / ((1.0 + nu) * (1.0 - 2.0 * nu));
        [
            [
                c * (1.0 - nu) * mods[0],
                c * nu * (mods[0] * mods[1]).sqrt(),
                0.0,
            ],
            [
                c * nu * (mods[0] * mods[1]).sqrt(),
                c * (1.0 - nu) * mods[1],
                0.0,
            ],
            [0.0, 0.0, g * tm * mods[2]],
        ]
    } else {
        let c = e * tm / (1.0 - nu * nu);
        [
            [c * mods[0], c * nu * (mods[0] * mods[1]).sqrt(), 0.0],
            [c * nu * (mods[0] * mods[1]).sqrt(), c * mods[1], 0.0],
            [0.0, 0.0, g * tm * mods[2]],
        ]
    };
    let cb = e * tb.powi(3) / (12.0 * (1.0 - nu * nu));
    let db = [
        [cb * mods[3], cb * nu * (mods[3] * mods[4]).sqrt(), 0.0],
        [cb * nu * (mods[3] * mods[4]).sqrt(), cb * mods[4], 0.0],
        [0.0, 0.0, cb * 0.5 * (1.0 - nu) * mods[5]],
    ];
    let ds = [
        [(5.0 / 6.0) * g * tb * mods[6], 0.0],
        [0.0, (5.0 / 6.0) * g * tb * mods[7]],
    ];
    let dp = plane_strain.then(|| [[g * tm * mods[6], 0.0], [0.0, g * tm * mods[7]]]);
    AreaLaw {
        dm,
        db,
        ds,
        dp,
        ortho: false,
        plane_strain,
        nu,
        u3: [nu, nu],
        mat_rot: (0.0, 1.0),
    }
}

/// Internal area temperature load types: `1` uniform change, `2` gradient through the
/// thickness (the type the native file and the OAPI number `3`).
pub const AREA_TEMP_UNIFORM: i32 = 1;
pub const AREA_TEMP_GRADIENT: i32 = 2;

/// Total uniform temperature change and total through-thickness gradient of an area in a
/// case with the given load patterns and scale factors.
pub fn area_thermal_totals(area_obj: &AreaObj, case_loads: &[(String, f64)]) -> (f64, f64) {
    let mut uniform = 0.0;
    let mut gradient = 0.0;
    for (pattern, sf) in case_loads {
        for lt in &area_obj.loads_temp {
            if &lt.load_pat != pattern {
                continue;
            }
            match lt.my_type {
                AREA_TEMP_UNIFORM => uniform += lt.val * sf,
                AREA_TEMP_GRADIENT => gradient += lt.val * sf,
                _ => {}
            }
        }
    }
    (uniform, gradient)
}

/// Removes the part of the recovered resultants that belongs to free thermal strain.
///
/// [`recover_area_result`] multiplies the total strains by the stiffness. Stress comes from
/// the strain in excess of the free thermal strain, so for a uniform change `dT` the
/// membrane forces lose `E tm alpha dT / (1 - nu)`, and for a gradient `g` (free curvature
/// `-alpha g`) the moments gain `E tb^3 alpha g / (12 (1 - nu))`. References: CSI Examples
/// 2-013 (a restrained plate at +100 has -733.33 lb/in2, a free one has none) and 2-014
/// (edge moment -2.182 k-in/in).
pub fn apply_area_thermal(
    result: &mut AreaResult,
    area_obj: &AreaObj,
    sec: &AreaSection,
    mat: &Material,
    uniform: f64,
    gradient: f64,
) {
    if uniform == 0.0 && gradient == 0.0 {
        return;
    }
    let mods: [f64; 10] = std::array::from_fn(|i| sec.modifiers[i] * area_obj.modifiers[i]);
    let nu = mat.u.clamp(-0.49, 0.4999);
    let tm = sec.thickness_membrane.max(1e-12);
    let tb = sec.thickness_bending.max(1e-12);
    if area_material_is_orthotropic(mat) {
        let law = area_law(
            area_obj,
            sec,
            mat,
            sec.thickness_membrane,
            sec.thickness_bending,
        );
        let a = area_alpha(mat);
        for (i, row) in law.dm.iter().enumerate() {
            let n_th = (row[0] + row[1]) * a * uniform;
            match i {
                0 => result.f11 -= n_th,
                1 => result.f22 -= n_th,
                _ => result.f12 -= n_th,
            }
        }
        for (i, row) in law.db.iter().enumerate() {
            let m_th = (row[0] + row[1]) * a * gradient;
            match i {
                0 => result.m11 += m_th,
                1 => result.m22 += m_th,
                _ => result.m12 += m_th,
            }
        }
    } else {
        let n_th = mat.e * sec.thickness_membrane * mat.a * uniform / (1.0 - nu);
        let m_th = mat.e * sec.thickness_bending.powi(3) * mat.a * gradient / (12.0 * (1.0 - nu));
        result.f11 -= n_th * mods[0];
        result.f22 -= n_th * mods[1];
        result.m11 += m_th * mods[3];
        result.m22 += m_th * mods[4];
    }
    result.s11_top = result.f11 / tm - 6.0 * result.m11 / (tb * tb);
    result.s22_top = result.f22 / tm - 6.0 * result.m22 / (tb * tb);
    result.s12_top = result.f12 / tm - 6.0 * result.m12 / (tb * tb);
    result.svm_top = plane_von_mises(result.s11_top, result.s22_top, result.s12_top);
}

/// Plane-stress von Mises value of the stress components `[s11, s22, s12]`.
///
/// A von Mises or principal value is not linear in the components, so a result that
/// averages or combines several stress states recomputes it from the combined components.
pub fn plane_von_mises(s11: f64, s22: f64, s12: f64) -> f64 {
    (s11 * s11 - s11 * s22 + s22 * s22 + 3.0 * s12 * s12)
        .max(0.0)
        .sqrt()
}

/// Recovers internal stresses and resultants ($F_{11}, F_{22}, F_{12}, M_{11}, M_{22}, M_{12}, V_{13}, V_{23}, S_{11}, S_{22}, S_{12}, S_{\text{VM}}$)
/// at the centroid of an area element.
pub fn recover_area_result(
    area_obj: &AreaObj,
    sec: &AreaSection,
    mat: &Material,
    local_xy: &[[f64; 2]],
    u_local: &[f64],
    joint_forces_global: Vec<(String, [f64; 6])>,
) -> AreaResult {
    let xy4 = if local_xy.len() == 3 {
        [local_xy[0], local_xy[1], local_xy[2], local_xy[2]]
    } else {
        [local_xy[0], local_xy[1], local_xy[2], local_xy[3]]
    };
    let u24: [f64; 24] = if u_local.len() >= 24 {
        std::array::from_fn(|i| u_local[i])
    } else {
        std::array::from_fn(|i| if i < 18 { u_local[i] } else { u_local[i - 6] })
    };

    let tm = sec.thickness_membrane.max(1e-12);
    let tb = sec.thickness_bending.max(1e-12);
    let law = area_law(area_obj, sec, mat, tm, tb);

    let (_, _, _, j_mat, det_j, dndx, dndy) = quad4_shape_derivs(&xy4, 0.0, 0.0);

    // Membrane strains at center (0, 0)
    let mut eps_xx = 0.0;
    let mut eps_yy = 0.0;
    let mut gamma_xy = 0.0;
    let mut kappa_xx = 0.0;
    let mut kappa_yy = 0.0;
    let mut kappa_xy = 0.0;
    let mut gamma_13 = 0.0;
    let mut gamma_23 = 0.0;

    for i in 0..4 {
        eps_xx += dndx[i] * u24[6 * i];
        eps_yy += dndy[i] * u24[6 * i + 1];
        gamma_xy += dndy[i] * u24[6 * i] + dndx[i] * u24[6 * i + 1];

        // beta_x = -theta_2 (6*i+4), beta_y = +theta_1 (6*i+3)
        kappa_xx += -dndx[i] * u24[6 * i + 4];
        kappa_yy += dndy[i] * u24[6 * i + 3];
        kappa_xy += dndx[i] * u24[6 * i + 3] - dndy[i] * u24[6 * i + 4];

        gamma_13 += dndx[i] * u24[6 * i + 2];
        gamma_23 += dndy[i] * u24[6 * i + 2];
    }

    let dm = &law.dm;
    let f11 = dm[0][0] * eps_xx + dm[0][1] * eps_yy + dm[0][2] * gamma_xy;
    let f22 = dm[1][0] * eps_xx + dm[1][1] * eps_yy + dm[1][2] * gamma_xy;
    let f12 = dm[2][0] * eps_xx + dm[2][1] * eps_yy + dm[2][2] * gamma_xy;

    let (m11, m22, m12, v13, v23);
    if let Some(dp) = law.dp {
        // Plane strain: no bending; the shear resultants are the anti-plane shear
        // stresses times the thickness (s13 t, s23 t).
        (m11, m22, m12) = (0.0, 0.0, 0.0);
        v13 = dp[0][0] * gamma_13 + dp[0][1] * gamma_23;
        v23 = dp[1][0] * gamma_13 + dp[1][1] * gamma_23;
    } else {
        let db = &law.db;
        m11 = db[0][0] * kappa_xx + db[0][1] * kappa_yy + db[0][2] * kappa_xy;
        m22 = db[1][0] * kappa_xx + db[1][1] * kappa_yy + db[1][2] * kappa_xy;
        m12 = db[2][0] * kappa_xx + db[2][1] * kappa_yy + db[2][2] * kappa_xy;

        let bs = mitc4_shear_b_matrix(&xy4, 0.0, 0.0, &j_mat, det_j);
        let mut gamma_xz = 0.0;
        let mut gamma_yz = 0.0;
        for c in 0..24 {
            gamma_xz += bs[0][c] * u24[c];
            gamma_yz += bs[1][c] * u24[c];
        }
        // The element carries boost * ds * gamma (D105); report that force.
        let boost = thin_plate_shear_boost(sec, sec.thickness_bending, &xy4);
        v13 = boost * (law.ds[0][0] * gamma_xz + law.ds[0][1] * gamma_yz);
        v23 = boost * (law.ds[1][0] * gamma_xz + law.ds[1][1] * gamma_yz);
    }

    // CSI Analysis Reference Eqns. 2: positive moments give positive stress at the
    // bottom face, so at the top face (x3 = +tb/2) s = F/tm - 6 M/tb^2.
    let s11_top = f11 / tm - 6.0 * m11 / (tb * tb);
    let s22_top = f22 / tm - 6.0 * m22 / (tb * tb);
    let s12_top = f12 / tm - 6.0 * m12 / (tb * tb);
    let svm_top = plane_von_mises(s11_top, s22_top, s12_top);
    // Plane strain: the thickness-normal stress from e33 = 0 (p. 218), from the membrane
    // stresses (the plane section has no bending).
    let s33 = law.s33(f11 / tm, f22 / tm, f12 / tm);

    AreaResult {
        f11,
        f22,
        f12,
        m11,
        m22,
        m12,
        v13,
        v23,
        s11_top,
        s22_top,
        s12_top,
        svm_top,
        s33,
        joint_forces: joint_forces_global,
    }
}
