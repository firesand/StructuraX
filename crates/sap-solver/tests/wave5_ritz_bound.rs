//! Wave 5 P11 (ledger D37): the Ritz-vector solver against an independently computed spectrum.
//!
//! Reference. The test builds the stiffness of its frames itself (Euler-Bernoulli members,
//! axial EA/L, torsion GJ/L, bending 12EI/L^3 ..., written out in `plane_k` and `beam3d_k`;
//! no shear deformation, the section has `as2 = as3 = 0`), condenses the massless degrees of
//! freedom statically (exact for a massless DOF) and solves `Kc x = w^2 M x`, M diagonal and
//! positive, by a cyclic Jacobi sweep of `M^-1/2 Kc M^-1/2` (`condensed_w2`). Nothing here
//! calls solver code, and the Eigen solver is only a second witness.
//!
//! Theory (what is asserted).
//! 1. Rayleigh-Ritz on any subspace of the space on which K is positive definite gives
//!    `w_i(ritz)^2 >= w_i(exact)^2` for every i (min-max theorem; the Analysis Reference,
//!    "Ritz-Vector Analysis", printed p. 372, calls the Ritz modes an approximation of the
//!    natural ones). In floating point the bound holds up to the error of the solve with K,
//!    `tol = 100 eps kappa` with `kappa = w_max^2 / w_min^2` of the exact spectrum.
//! 2. A set of M-orthonormal vectors has at most `rank(M)` members; the manual lets the
//!    program "compute fewer modes if there are fewer mass degrees of freedom" (printed
//!    p. 373, Ritz-Vector Analysis, Number of Modes), so a request above the rank returns the
//!    rank's number of modes.
//! 3. When the Ritz basis has `rank(M)` vectors, the mass part of the basis is the whole mass
//!    space, so the Ritz problem is the exact one: every period equals the exact period.
//! 4. A stiff but positive definite model is solved (the exact problem is positive definite).
use sap_model::*;
use sap_solver::analyze_model;

const E: f64 = 1000.0;
const EI: f64 = 2000.0; // I = 2
const A: f64 = 10.0;
const GJ: f64 = E / 2.6; // J = 1

fn base_model() -> SapModel {
    let mut m = SapModel::new(sap_units::Units::KnMC);
    m.load_cases.clear();
    m.load_patterns.clear();
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.e = E;
    mat.g = E / 2.6;
    mat.unit_weight = 0.0;
    mat.unit_mass = 0.0;
    m.materials.insert("MAT".into(), mat);
    m.frame_sections.insert(
        "SEC".into(),
        FrameSection {
            name: "SEC".into(),
            material: "MAT".into(),
            area: A,
            i22: EI / E,
            i33: EI / E,
            torsional_j: 1.0,
            as2: 0.0,
            as3: 0.0,
            ..FrameSection::default()
        },
    );
    m
}

// ---- independent reference ------------------------------------------------------------------

/// Exact `w^2`, ascending, of `K x = w^2 M x` with `mass[d]` the diagonal mass of DOF d,
/// `fixed[d]` the restrained DOFs. Massless free DOFs are condensed out (Gaussian elimination
/// with partial pivoting), the rest solved by Jacobi.
fn condensed_w2(k: &[Vec<f64>], mass: &[f64], fixed: &[bool]) -> Vec<f64> {
    let n = mass.len();
    let kept: Vec<usize> = (0..n).filter(|&d| !fixed[d] && mass[d] > 0.0).collect();
    let elim: Vec<usize> = (0..n).filter(|&d| !fixed[d] && mass[d] <= 0.0).collect();
    let (ne, nk) = (elim.len(), kept.len());
    let mut kee: Vec<Vec<f64>> = elim
        .iter()
        .map(|&a| elim.iter().map(|&b| k[a][b]).collect())
        .collect();
    let mut kek: Vec<Vec<f64>> = elim
        .iter()
        .map(|&a| kept.iter().map(|&b| k[a][b]).collect())
        .collect();
    for c in 0..ne {
        let piv = (c..ne)
            .max_by(|&a, &b| kee[a][c].abs().total_cmp(&kee[b][c].abs()))
            .unwrap();
        kee.swap(c, piv);
        kek.swap(c, piv);
        for r in c + 1..ne {
            let f = kee[r][c] / kee[c][c];
            for q in c..ne {
                kee[r][q] -= f * kee[c][q];
            }
            for q in 0..nk {
                kek[r][q] -= f * kek[c][q];
            }
        }
    }
    for c in (0..ne).rev() {
        for q in 0..nk {
            let mut s = kek[c][q];
            for r in c + 1..ne {
                s -= kee[c][r] * kek[r][q];
            }
            kek[c][q] = s / kee[c][c];
        }
    }
    // kek now holds Kee^-1 Kek; Kc = Kkk - Kke Kee^-1 Kek.
    let mut kc = vec![vec![0.0; nk]; nk];
    for a in 0..nk {
        for b in 0..nk {
            let mut s = k[kept[a]][kept[b]];
            for (r, &e) in elim.iter().enumerate() {
                s -= k[kept[a]][e] * kek[r][b];
            }
            kc[a][b] = s;
        }
    }
    let minv: Vec<f64> = kept.iter().map(|&d| 1.0 / mass[d].sqrt()).collect();
    let mut s: Vec<Vec<f64>> = (0..nk)
        .map(|a| {
            (0..nk)
                .map(|b| 0.5 * (kc[a][b] + kc[b][a]) * minv[a] * minv[b])
                .collect()
        })
        .collect();
    let mut w2 = jacobi(&mut s);
    w2.sort_by(|a, b| a.total_cmp(b));
    w2
}

/// Cyclic Jacobi eigenvalues of a symmetric matrix (destroyed).
fn jacobi(a: &mut [Vec<f64>]) -> Vec<f64> {
    let n = a.len();
    for _ in 0..100 {
        let (mut off, mut diag) = (0.0, 0.0);
        for i in 0..n {
            diag += a[i][i] * a[i][i];
            for j in 0..n {
                if j != i {
                    off += a[i][j] * a[i][j];
                }
            }
        }
        if off <= 1e-30 * diag {
            break;
        }
        for p in 0..n {
            for q in p + 1..n {
                if a[p][q] == 0.0 {
                    continue;
                }
                let th = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                let t = if th == 0.0 {
                    1.0
                } else {
                    th.signum() / (th.abs() + (th * th + 1.0).sqrt())
                };
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                for row in a.iter_mut() {
                    let (x, y) = (row[p], row[q]);
                    row[p] = c * x - s * y;
                    row[q] = s * x + c * y;
                }
                for k in 0..n {
                    let (x, y) = (a[p][k], a[q][k]);
                    a[p][k] = c * x - s * y;
                    a[q][k] = s * x + c * y;
                }
            }
        }
    }
    (0..n).map(|i| a[i][i]).collect()
}

/// `t^T kl t` for a block-diagonal `t`.
fn congruence(kl: &[Vec<f64>], t: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let n = kl.len();
    let mut kg = vec![vec![0.0; n]; n];
    for i in 0..n {
        for j in 0..n {
            for p in 0..n {
                if t[p][i] == 0.0 {
                    continue;
                }
                for q in 0..n {
                    kg[i][j] += t[p][i] * kl[p][q] * t[q][j];
                }
            }
        }
    }
    kg
}

/// Plane-frame member stiffness in global (ux, uz, ry) at both ends (6x6). The sign of the
/// plane rotation against `ry` flips theta only, which leaves the eigenvalues unchanged.
fn plane_k(p1: (f64, f64), p2: (f64, f64), ea: f64) -> Vec<Vec<f64>> {
    let (dx, dz) = (p2.0 - p1.0, p2.1 - p1.1);
    let l = dx.hypot(dz);
    let (c, s) = (dx / l, dz / l);
    let a = ea / l;
    let (b1, b2, b3, b4) = (
        12.0 * EI / l.powi(3),
        6.0 * EI / l.powi(2),
        4.0 * EI / l,
        2.0 * EI / l,
    );
    #[rustfmt::skip]
    let kl = vec![
        vec![ a, 0.0, 0.0, -a, 0.0, 0.0],
        vec![0.0,  b1,  b2, 0.0, -b1,  b2],
        vec![0.0,  b2,  b3, 0.0, -b2,  b4],
        vec![-a, 0.0, 0.0,  a, 0.0, 0.0],
        vec![0.0, -b1, -b2, 0.0,  b1, -b2],
        vec![0.0,  b2,  b4, 0.0, -b2,  b3],
    ];
    let mut t = vec![vec![0.0; 6]; 6];
    for e in 0..2 {
        let o = 3 * e;
        t[o][o] = c;
        t[o][o + 1] = s;
        t[o + 1][o] = -s;
        t[o + 1][o + 1] = c;
        t[o + 2][o + 2] = 1.0;
    }
    congruence(&kl, &t)
}

/// Space-frame member stiffness (12x12 in global joint axes, DOF order u v w rx ry rz).
/// `I2 = I3`, so the choice of the local transverse axes does not matter.
fn beam3d_k(p1: [f64; 3], p2: [f64; 3], ea: f64) -> Vec<Vec<f64>> {
    let d = [p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]];
    let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    let ex = [d[0] / l, d[1] / l, d[2] / l];
    let up = if ex[2].abs() > 0.99 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 0.0, 1.0]
    };
    let cross = |a: [f64; 3], b: [f64; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let mut ey = cross(up, ex);
    let ny = (ey[0] * ey[0] + ey[1] * ey[1] + ey[2] * ey[2]).sqrt();
    ey = [ey[0] / ny, ey[1] / ny, ey[2] / ny];
    let ez = cross(ex, ey);
    let (a, t) = (ea / l, GJ / l);
    let (b1, b2, b3, b4) = (
        12.0 * EI / l.powi(3),
        6.0 * EI / l.powi(2),
        4.0 * EI / l,
        2.0 * EI / l,
    );
    let mut kl = vec![vec![0.0; 12]; 12];
    let mut put = |i: usize, j: usize, v: f64| {
        kl[i][j] = v;
        kl[j][i] = v;
    };
    put(0, 0, a);
    put(0, 6, -a);
    put(6, 6, a);
    put(3, 3, t);
    put(3, 9, -t);
    put(9, 9, t);
    // bending in the local x-y plane: v (1, 7), rz (5, 11)
    put(1, 1, b1);
    put(1, 5, b2);
    put(1, 7, -b1);
    put(1, 11, b2);
    put(5, 5, b3);
    put(5, 7, -b2);
    put(5, 11, b4);
    put(7, 7, b1);
    put(7, 11, -b2);
    put(11, 11, b3);
    // bending in the local x-z plane: w (2, 8), ry (4, 10)
    put(2, 2, b1);
    put(2, 4, -b2);
    put(2, 8, -b1);
    put(2, 10, -b2);
    put(4, 4, b3);
    put(4, 8, b2);
    put(4, 10, b4);
    put(8, 8, b1);
    put(8, 10, b2);
    put(10, 10, b3);
    let mut tm = vec![vec![0.0; 12]; 12];
    for blk in 0..4 {
        for (r, axis) in [ex, ey, ez].iter().enumerate() {
            for c in 0..3 {
                tm[3 * blk + r][3 * blk + c] = axis[c];
            }
        }
    }
    congruence(&kl, &tm)
}

/// `w^2` tolerance of the bound: `100 eps kappa`, `kappa` the condition of the exact spectrum.
fn tolerance(exact: &[f64]) -> f64 {
    100.0 * f64::EPSILON * exact[exact.len() - 1] / exact[0]
}

// ---- plane frames -------------------------------------------------------------------------------

/// A node of a plane frame: (x, z, mass on UX, mass on UZ).
type Node = (f64, f64, f64, f64);
type Members = Vec<(usize, usize, f64)>;

/// Plane frame in the global XZ plane: UY, RX, RZ restrained everywhere, `fixed` nodes fully.
fn plane_model(nodes: &[Node], members: &Members, fixed: &[usize]) -> SapModel {
    let mut m = base_model();
    for (i, &(x, z, mx, mz)) in nodes.iter().enumerate() {
        let name = format!("N{i}");
        m.add_point(name.clone(), x, 0.0, z);
        let p = m.points.get_mut(&name).unwrap();
        p.restraint = if fixed.contains(&i) {
            [true; 6]
        } else {
            [false, true, false, true, false, true]
        };
        p.mass = [mx, 0.0, mz, 0.0, 0.0, 0.0];
    }
    for (k, &(i, j, amod)) in members.iter().enumerate() {
        let name = format!("F{k}");
        m.add_frame(name.clone(), format!("N{i}"), format!("N{j}"), "SEC");
        m.frames.get_mut(&name).unwrap().modifiers[0] = amod;
    }
    m
}

fn plane_exact(nodes: &[Node], members: &Members, fixed: &[usize]) -> Vec<f64> {
    let n = 3 * nodes.len();
    let mut k = vec![vec![0.0; n]; n];
    for &(i, j, amod) in members {
        let ke = plane_k(
            (nodes[i].0, nodes[i].1),
            (nodes[j].0, nodes[j].1),
            E * A * amod,
        );
        let map = [3 * i, 3 * i + 1, 3 * i + 2, 3 * j, 3 * j + 1, 3 * j + 2];
        for a in 0..6 {
            for b in 0..6 {
                k[map[a]][map[b]] += ke[a][b];
            }
        }
    }
    let mass: Vec<f64> = nodes.iter().flat_map(|nd| [nd.2, nd.3, 0.0]).collect();
    let fx: Vec<bool> = (0..n).map(|d| fixed.contains(&(d / 3))).collect();
    condensed_w2(&k, &mass, &fx)
}

/// Cantilever column of `n` storeys, mass 2 on UX at every level above the base.
fn stack(n: usize) -> (Vec<Node>, Members, Vec<usize>) {
    let nodes = (0..=n)
        .map(|i| (0.0, 3.0 * i as f64, if i == 0 { 0.0 } else { 2.0 }, 0.0))
        .collect();
    (nodes, (0..n).map(|i| (i, i + 1, 1.0)).collect(), vec![0])
}

/// Two-storey one-bay frame, beams horizontal with axial modifier `amod`; unequal masses on UX.
fn portal(amod: f64) -> (Vec<Node>, Members, Vec<usize>) {
    let nodes = vec![
        (0.0, 0.0, 0.0, 0.0),
        (4.0, 0.0, 0.0, 0.0),
        (0.0, 3.0, 2.0, 0.0),
        (4.5, 3.0, 3.1, 0.0),
        (0.0, 6.0, 1.3, 0.0),
        (4.5, 6.0, 0.7, 0.0),
    ];
    let members = vec![
        (0, 2, 1.0),
        (1, 3, 1.0),
        (2, 4, 1.0),
        (3, 5, 1.0),
        (2, 3, amod),
        (4, 5, amod),
    ];
    (nodes, members, vec![0, 1])
}

/// `stories` x `bays` plane frame, beams with axial modifier `amod`, unequal masses on UX and UZ
/// at every joint above the base: rank `2 * stories * (bays + 1)`.
fn grid(stories: usize, bays: usize, amod: f64) -> (Vec<Node>, Members, Vec<usize>) {
    let cols = bays + 1;
    let mut nodes = Vec::new();
    for f in 0..=stories {
        for c in 0..cols {
            let x = 4.0 * c as f64 + 0.1 * f as f64 * c as f64;
            let w = 1.0 + 0.37 * f as f64 + 0.21 * c as f64;
            nodes.push((
                x,
                3.0 * f as f64,
                if f == 0 { 0.0 } else { w },
                if f == 0 { 0.0 } else { 0.6 * w },
            ));
        }
    }
    let mut members = Vec::new();
    for f in 0..=stories {
        for c in 0..cols {
            let i = f * cols + c;
            if f < stories {
                members.push((i, i + cols, 1.0));
            }
            if f > 0 && c < bays {
                members.push((i, i + 1, amod));
            }
        }
    }
    (nodes, members, (0..cols).collect())
}

// ---- 3D frame with closed floor loops --------------------------------------------------------

/// `floors` storeys, 4 columns on a 4 x 4 plan, 4 floor beams per storey (axial modifier
/// `amod`), mass `mj[f]` on UX and UY, `uz_mass` on UZ and `rot_mass` on the three rotations
/// of every floor joint. Returns the model and the exact spectrum.
fn frame3d(
    floors: usize,
    amod: f64,
    mj: &[f64],
    rot_mass: f64,
    uz_mass: f64,
) -> (SapModel, Vec<f64>) {
    let mut m = base_model();
    let corners = [(-2.0, -2.0), (2.0, -2.0), (2.0, 2.0), (-2.0, 2.0)];
    let idx = |f: usize, c: usize| f * 4 + c;
    let mut coords = Vec::new();
    let mut masses: Vec<[f64; 6]> = Vec::new();
    for f in 0..=floors {
        for (ci, (x, y)) in corners.iter().enumerate() {
            let name = format!("J{f}_{ci}");
            m.add_point(name.clone(), *x, *y, 3.0 * f as f64);
            let ms = if f == 0 {
                [0.0; 6]
            } else {
                [mj[f - 1], mj[f - 1], uz_mass, rot_mass, rot_mass, rot_mass]
            };
            let p = m.points.get_mut(&name).unwrap();
            if f == 0 {
                p.restraint = [true; 6];
            }
            p.mass = ms;
            coords.push([*x, *y, 3.0 * f as f64]);
            masses.push(ms);
        }
    }
    let mut members: Vec<(usize, usize, f64)> = Vec::new();
    for f in 1..=floors {
        for c in 0..4 {
            m.add_frame(
                format!("C{f}_{c}"),
                format!("J{}_{c}", f - 1),
                format!("J{f}_{c}"),
                "SEC",
            );
            members.push((idx(f - 1, c), idx(f, c), 1.0));
        }
        for c in 0..4 {
            let b = format!("B{f}_{c}");
            m.add_frame(
                b.clone(),
                format!("J{f}_{c}"),
                format!("J{f}_{}", (c + 1) % 4),
                "SEC",
            );
            m.frames.get_mut(&b).unwrap().modifiers[0] = amod;
            members.push((idx(f, c), idx(f, (c + 1) % 4), amod));
        }
    }
    let n = 6 * coords.len();
    let mut k = vec![vec![0.0; n]; n];
    for &(i, j, a) in &members {
        let ke = beam3d_k(coords[i], coords[j], E * A * a);
        let map: Vec<usize> = (0..6)
            .map(|d| 6 * i + d)
            .chain((0..6).map(|d| 6 * j + d))
            .collect();
        for r in 0..12 {
            for c in 0..12 {
                k[map[r]][map[c]] += ke[r][c];
            }
        }
    }
    let mass: Vec<f64> = masses.iter().flatten().copied().collect();
    let fixed: Vec<bool> = (0..n).map(|d| d < 24).collect();
    (m, condensed_w2(&k, &mass, &fixed))
}

// ---- 3D shear building with rigid diaphragms (penalty constraints) -------------------------------

/// `floors` storeys of 4 columns (plan 4 x 4), fixed-fixed columns (UZ, RX, RY, RZ restrained at
/// every joint), one diaphragm per floor, mass `mj[f]` on UX and UY of every floor joint.
/// The diaphragm leaves each floor two translations: the exact model is a shear chain with
/// storey stiffness `4 * 12 EI / h^3` and floor mass `4 mj`, once for X and once for Y.
fn diaphragm_building(mj: &[f64]) -> (SapModel, Vec<f64>) {
    let floors = mj.len();
    let mut m = base_model();
    for (ci, (x, y)) in [(-2.0, -2.0), (2.0, -2.0), (2.0, 2.0), (-2.0, 2.0)]
        .iter()
        .enumerate()
    {
        for f in 0..=floors {
            let name = format!("J{f}_{ci}");
            m.add_point(name.clone(), *x, *y, 3.0 * f as f64);
            let p = m.points.get_mut(&name).unwrap();
            p.restraint = [false, false, true, true, true, true];
            if f == 0 {
                p.restraint = [true; 6];
            } else {
                p.mass = [mj[f - 1], mj[f - 1], 0.0, 0.0, 0.0, 1.0];
                p.constraints.push(format!("D{f}"));
                m.add_frame(
                    format!("C{f}_{ci}"),
                    format!("J{}_{ci}", f - 1),
                    name,
                    "SEC",
                );
            }
        }
    }
    for f in 1..=floors {
        m.constraints.insert(
            format!("D{f}"),
            ConstraintDef {
                name: format!("D{f}"),
                constraint_type: ConstraintType::Diaphragm,
                axis: ConstraintAxis::Z,
                dofs: [true; 6],
            },
        );
    }
    let ks = 4.0 * 12.0 * EI / 27.0;
    let mut k = vec![vec![0.0; floors]; floors];
    for i in 0..floors {
        k[i][i] += ks;
        if i + 1 < floors {
            k[i][i] += ks;
            k[i][i + 1] -= ks;
            k[i + 1][i] -= ks;
        }
    }
    let mut s: Vec<Vec<f64>> = (0..floors)
        .map(|a| {
            (0..floors)
                .map(|b| k[a][b] / (4.0 * mj[a] * 4.0 * mj[b]).sqrt())
                .collect()
        })
        .collect();
    let mut one = jacobi(&mut s);
    one.sort_by(|a, b| a.total_cmp(b));
    let mut both: Vec<f64> = one.iter().flat_map(|v| [*v, *v]).collect();
    both.sort_by(|a, b| a.total_cmp(b));
    (m, both)
}

// ---- running the solver -------------------------------------------------------------------------

/// Eigenvalues `w^2` of a modal case: Ritz with the starting loads `ritz`, Eigen when `None`.
fn run(mut m: SapModel, ritz: Option<&[&str]>, n: usize) -> Result<Vec<f64>, String> {
    let mut lc = LoadCase::modal_eigen("M", n);
    if let Some(loads) = ritz {
        lc.modal_sub_type = ModalSubType::Ritz;
        for l in loads {
            lc.loads.push(LoadCaseAssignment {
                load_name: (*l).into(),
                scale_factor: 1.0,
            });
        }
    }
    m.load_cases.insert("M".into(), lc);
    analyze_model(&mut m).map_err(|e| e.to_string())?;
    Ok(m.case_results["M"]
        .modes
        .iter()
        .map(|x| x.eigenvalue)
        .collect())
}

/// Theory points 1, 2 and 3 of the header, for a Ritz run of `n` vectors on a model with `rank`
/// mass degrees of freedom and the exact spectrum `exact` (all `rank` values).
fn check(label: &str, exact: &[f64], ritz: &[f64], n: usize) {
    let rank = exact.len();
    let tol = tolerance(exact);
    assert!(!ritz.is_empty(), "{label}: no mode");
    assert!(
        ritz.len() <= n.min(rank),
        "{label}: {} modes for n = {n}, rank {rank}",
        ritz.len()
    );
    for (i, (r, x)) in ritz.iter().zip(exact).enumerate() {
        assert!(
            *r >= x * (1.0 - tol),
            "{label}: Ritz w2[{i}] = {r:e} lies below the exact {x:e} (tol {tol:e})"
        );
    }
    if ritz.len() == rank {
        for (i, (r, x)) in ritz.iter().zip(exact).enumerate() {
            assert!(
                (r - x).abs() <= x * tol,
                "{label}: complete Ritz basis, w2[{i}] = {r:e} against exact {x:e} (tol {tol:e})"
            );
        }
    }
}

#[test]
fn cantilever_stack_rank_four() {
    let (nodes, members, fixed) = stack(4);
    let exact = plane_exact(&nodes, &members, &fixed);
    assert_eq!(exact.len(), 4);
    // Closed form check of the reference: the first cantilever frequency of the uniform
    // 4-mass chain is near the one of a lumped model; here only the cantilever tip stiffness
    // 3 EI / L^3 of the single-storey case is quoted: one storey, L = 3, mass 2.
    let (n1, m1, f1) = stack(1);
    let w2 = plane_exact(&n1, &m1, &f1)[0];
    assert!((w2 - 3.0 * EI / 27.0 / 2.0).abs() < 1e-12 * w2, "{w2}");
    for n in [2, 3, 4, 5, 8] {
        for loads in [&["UX"][..], &[][..]] {
            let ritz = run(plane_model(&nodes, &members, &fixed), Some(loads), n).unwrap();
            check(&format!("stack n={n} loads={loads:?}"), &exact, &ritz, n);
        }
    }
    let eigen = run(plane_model(&nodes, &members, &fixed), None, 4).unwrap();
    for (e, x) in eigen.iter().zip(&exact) {
        assert!((e - x).abs() <= x * tolerance(&exact));
    }
}

#[test]
fn portal_with_stiff_beams() {
    for amod in [1.0, 1e3, 1e6, 1e9] {
        let (nodes, members, fixed) = portal(amod);
        let exact = plane_exact(&nodes, &members, &fixed);
        for n in [2, 4, 7] {
            let ritz = run(plane_model(&nodes, &members, &fixed), Some(&["UX"]), n).unwrap();
            check(&format!("portal amod={amod:e} n={n}"), &exact, &ritz, n);
        }
    }
}

#[test]
fn grid_with_stiff_beams_below_at_and_above_the_rank() {
    for amod in [1.0, 1e6] {
        let (nodes, members, fixed) = grid(4, 3, amod);
        let exact = plane_exact(&nodes, &members, &fixed);
        assert_eq!(exact.len(), 32);
        for n in [6, 12, 20, 31, 32, 33, 40] {
            let ritz = run(
                plane_model(&nodes, &members, &fixed),
                Some(&["UX", "UZ"]),
                n,
            )
            .unwrap();
            check(&format!("grid amod={amod:e} n={n}"), &exact, &ritz, n);
            if n >= 32 {
                assert_eq!(
                    ritz.len(),
                    32,
                    "grid amod={amod:e} n={n}: the rank's number of modes"
                );
            }
        }
    }
}

#[test]
fn space_frame_with_stiff_floor_loops() {
    let mj = [2.0, 3.0, 1.5];
    // (axial modifier, rotation mass, UZ mass, starting loads)
    let cases: [(f64, f64, f64, &[&str]); 5] = [
        (1.0, 0.0, 0.0, &["UX", "UY"]),
        (1e6, 0.0, 0.0, &["UX", "UY"]),
        (1e6, 0.0, 1.0, &["UX", "UY"]),
        (1e6, 0.0, 1.0, &["UX", "UY", "UZ"]),
        (1e9, 0.0, 0.0, &["UX", "UY"]),
    ];
    for (amod, rm, uzm, loads) in cases {
        let (_, exact) = frame3d(3, amod, &mj, rm, uzm);
        let rank = 12 * (2 + usize::from(uzm > 0.0) + 3 * usize::from(rm > 0.0));
        assert_eq!(exact.len(), rank);
        for n in [6, 12, 30, rank, rank + 1, 60] {
            let (model, _) = frame3d(3, amod, &mj, rm, uzm);
            let ritz = run(model, Some(loads), n)
                .unwrap_or_else(|e| panic!("amod={amod:e} uz={uzm} loads={loads:?} n={n}: {e}"));
            check(
                &format!("frame3d amod={amod:e} uz={uzm} loads={loads:?} n={n}"),
                &exact,
                &ritz,
                n,
            );
        }
    }
}

#[test]
fn shear_building_with_diaphragms() {
    let mj = [2.0, 3.0, 1.5];
    let (_, exact) = diaphragm_building(&mj);
    // Nominal rank of M: 4 joints x (UX, UY, RZ) x 3 floors = 36, but the diaphragms leave 6
    // finite modes; the penalty modes lie near 1e10, so the Ritz set has at most 6 members
    // below them. The bound is against the rigid-diaphragm spectrum, to the penalty error.
    for n in [3, 6, 12, 36, 40] {
        let (model, _) = diaphragm_building(&mj);
        let ritz = run(model, Some(&["UX", "UY"]), n).unwrap();
        assert!(ritz.len() <= n.min(36));
        let low: Vec<f64> = ritz.iter().copied().filter(|w| *w < 1e6).collect();
        let tol = 100.0 * f64::EPSILON * 1.85e10 / exact[0];
        for (i, (r, x)) in low.iter().zip(&exact).enumerate() {
            assert!(
                *r >= x * (1.0 - tol),
                "n={n}: w2[{i}] = {r:e} below exact {x:e}"
            );
        }
        if n >= 6 {
            assert_eq!(low.len(), 6, "n={n}");
            for (i, (r, x)) in low.iter().zip(&exact).enumerate() {
                assert!(
                    (r - x).abs() <= x * tol,
                    "n={n}: w2[{i}] = {r:e} against {x:e}"
                );
            }
        }
    }
}
