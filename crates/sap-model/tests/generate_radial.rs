//! Geometry commands: grids, quick draw, radial replicate/extrude, reversal of
//! area local 3, curved frames, selection by section, diaphragms.
//!
//! Every expected value is a closed form evaluated in this file (coordinates,
//! counts, angles); none was read back from the implementation. Lengths are
//! in the units of the model (kip-in); angles in degrees unless stated.
//!
//! Local-axis rules used by the radial tests are taken from the solver's
//! definitions (`sap-solver/src/frame.rs: frame_local_axes`,
//! `area.rs: area_local_system`, `frame.rs: joint_local_to_global_matrix`) and
//! are re-implemented below as `frame_axes`, `area_axes` and `joint_axes`. The
//! solver itself is exercised against these rules in
//! `crates/sap-solver/tests/p6_geometry_local_axes.rs`.

use sap_model::generate::{BraceKind, GridRegion, SpanDirection, VerticalPlane};
use sap_model::radial::{CurveKind, ReverseKeep};
use sap_model::{
    AreaLoadSurfacePressure, AreaLoadTemperature, AreaLoadUniform, ConstraintAxis, ConstraintDef,
    ConstraintType, EditError, FrameLoadDistributed, LinkObj, PointLoadForce, PointObj, SapModel,
    SolidObj,
};

const TOL: f64 = 1e-12;

/// Equality that also holds for models containing NaN (compares the Debug text).
fn same(a: &SapModel, b: &SapModel) -> bool {
    format!("{a:?}") == format!("{b:?}")
}

fn close(a: f64, b: f64, scale: f64) -> bool {
    (a - b).abs() <= TOL * scale.max(1.0)
}

fn find(m: &SapModel, c: [f64; 3]) -> Option<&PointObj> {
    m.points.values().find(|p| {
        (p.x - c[0]).abs() < 1e-9 && (p.y - c[1]).abs() < 1e-9 && (p.z - c[2]).abs() < 1e-9
    })
}

fn pt(m: &SapModel, name: &str) -> [f64; 3] {
    let p = &m.points[name];
    [p.x, p.y, p.z]
}

fn grid_model(nx: usize, ny: usize, nz: usize) -> SapModel {
    let mut m = SapModel::default();
    m.quick_grid([nx, ny, nz], [6.0, 5.0, 4.0], [0.0, 0.0, 0.0])
        .unwrap();
    m
}

fn full(m: &SapModel) -> GridRegion {
    let g = &m.coord_systems["GLOBAL"].grids;
    let n = |a: &str| g.iter().filter(|l| l.axis == a).count() - 1;
    GridRegion {
        x: (0, n("X")),
        y: (0, n("Y")),
        z: (0, n("Z")),
    }
}

// --- local-axis rules (mirror of the solver's definitions) -------------------

fn frame_axes(a: [f64; 3], b: [f64; 3], angle_deg: f64) -> [[f64; 3]; 3] {
    let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    let e1 = [d[0] / l, d[1] / l, d[2] / l];
    let h = e1[0].hypot(e1[1]);
    let (e2_0, e3_0) = if h > 1e-4 {
        (
            [-e1[0] * e1[2] / h, -e1[1] * e1[2] / h, h],
            [e1[1] / h, -e1[0] / h, 0.0],
        )
    } else {
        let cz = if e1[2] >= 0.0 { 1.0 } else { -1.0 };
        ([1.0, 0.0, 0.0], [0.0, cz, 0.0])
    };
    let (sb, cb) = angle_deg.to_radians().sin_cos();
    let e2 = std::array::from_fn(|k| cb * e2_0[k] + sb * e3_0[k]);
    let e3 = std::array::from_fn(|k| -sb * e2_0[k] + cb * e3_0[k]);
    [e1, e2, e3]
}

fn area_axes(c: &[[f64; 3]], angle_deg: f64) -> [[f64; 3]; 3] {
    let (v_xi, v_eta): ([f64; 3], [f64; 3]) = if c.len() >= 4 {
        (
            std::array::from_fn(|k| (c[1][k] + c[2][k]) - (c[0][k] + c[3][k])),
            std::array::from_fn(|k| (c[2][k] + c[3][k]) - (c[0][k] + c[1][k])),
        )
    } else {
        (
            std::array::from_fn(|k| c[1][k] - c[0][k]),
            std::array::from_fn(|k| c[2][k] - c[0][k]),
        )
    };
    let cr = [
        v_xi[1] * v_eta[2] - v_xi[2] * v_eta[1],
        v_xi[2] * v_eta[0] - v_xi[0] * v_eta[2],
        v_xi[0] * v_eta[1] - v_xi[1] * v_eta[0],
    ];
    let nn = (cr[0] * cr[0] + cr[1] * cr[1] + cr[2] * cr[2]).sqrt();
    let e3 = [cr[0] / nn, cr[1] / nn, cr[2] / nn];
    let h = e3[0].hypot(e3[1]);
    let (e1_0, e2_0) = if h <= 1e-4 {
        let sz = if e3[2] >= 0.0 { 1.0 } else { -1.0 };
        ([1.0, 0.0, 0.0], [0.0, sz, 0.0])
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
    let e1 = std::array::from_fn(|k| cb * e1_0[k] + sb * e2_0[k]);
    let e2 = std::array::from_fn(|k| -sb * e1_0[k] + cb * e2_0[k]);
    [e1, e2, e3]
}

/// Columns are the joint's local axes in global coordinates: Rz(A) Ry(B) Rx(C).
fn joint_axes(p: &PointObj) -> [[f64; 3]; 3] {
    if let Some(m) = p.local_axes_matrix {
        return m;
    }
    let [a, b, c] = p.local_axes_deg.map(f64::to_radians);
    let (sa, ca) = a.sin_cos();
    let (sb, cb) = b.sin_cos();
    let (sc, cc) = c.sin_cos();
    let rz = [[ca, -sa, 0.0], [sa, ca, 0.0], [0.0, 0.0, 1.0]];
    let ry = [[cb, 0.0, -sb], [0.0, 1.0, 0.0], [sb, 0.0, cb]];
    let rx = [[1.0, 0.0, 0.0], [0.0, cc, -sc], [0.0, sc, cc]];
    let mul = |x: [[f64; 3]; 3], y: [[f64; 3]; 3]| {
        let mut o = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                o[i][j] = (0..3).map(|k| x[i][k] * y[k][j]).sum();
            }
        }
        o
    };
    mul(mul(rz, ry), rx)
}

fn rot_z(v: [f64; 3], deg: f64) -> [f64; 3] {
    let (s, c) = deg.to_radians().sin_cos();
    [c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]]
}

fn vclose(a: [f64; 3], b: [f64; 3]) -> bool {
    (0..3).all(|k| (a[k] - b[k]).abs() < 1e-12)
}

// --- grid ---------------------------------------------------------------

#[test]
fn quick_grid_line_counts_ids_and_coordinates() {
    let mut m = SapModel::default();
    m.quick_grid([4, 3, 5], [2.5, 3.0, 4.0], [1.0, -2.0, 0.5])
        .unwrap();
    let g = &m.coord_systems["GLOBAL"].grids;
    let axis = |a: &str| g.iter().filter(|l| l.axis == a).collect::<Vec<_>>();
    let (x, y, z) = (axis("X"), axis("Y"), axis("Z"));
    assert_eq!((x.len(), y.len(), z.len()), (4, 3, 5));
    assert_eq!(g.len(), 12);
    for (i, l) in x.iter().enumerate() {
        assert_eq!(l.id, ["A", "B", "C", "D"][i]);
        assert!(close(l.coordinate, 1.0 + 2.5 * i as f64, 1.0));
    }
    for (i, l) in y.iter().enumerate() {
        assert_eq!(l.id, (i + 1).to_string());
        assert!(close(l.coordinate, -2.0 + 3.0 * i as f64, 1.0));
    }
    for (i, l) in z.iter().enumerate() {
        assert_eq!(l.id, format!("Z{}", i + 1));
        assert!(close(l.coordinate, 0.5 + 4.0 * i as f64, 1.0));
    }
    // More than 26 lines: Excel-style labels, all distinct.
    m.quick_grid([55, 1, 1], [1.0, 1.0, 1.0], [0.0; 3]).unwrap();
    let ids: Vec<_> = m.coord_systems["GLOBAL"]
        .grids
        .iter()
        .filter(|l| l.axis == "X")
        .map(|l| l.id.clone())
        .collect();
    assert_eq!(ids[25], "Z");
    assert_eq!(ids[26], "AA");
    assert_eq!(ids[27], "AB");
    assert_eq!(ids[51], "AZ");
    assert_eq!(ids[52], "BA");
    assert_eq!(
        ids.iter().collect::<std::collections::BTreeSet<_>>().len(),
        55
    );
    // Replaces, never appends.
    assert_eq!(m.coord_systems["GLOBAL"].grids.len(), 57);
}

#[test]
fn quick_grid_refusals_leave_the_model_unchanged() {
    let mut m = grid_model(3, 3, 2);
    let before = m.clone();
    for (counts, spacing, origin) in [
        ([0, 2, 2], [1.0; 3], [0.0; 3]),
        ([2, 0, 2], [1.0; 3], [0.0; 3]),
        ([2, 2, 1001], [1.0; 3], [0.0; 3]),
        ([2, 2, 2], [0.0, 1.0, 1.0], [0.0; 3]),
        ([2, 2, 2], [1.0, -1.0, 1.0], [0.0; 3]),
        ([2, 2, 2], [1.0, 1.0, f64::NAN], [0.0; 3]),
        ([2, 2, 2], [1.0, 1.0, f64::INFINITY], [0.0; 3]),
        ([2, 2, 2], [1.0; 3], [f64::NAN, 0.0, 0.0]),
        ([2, 2, 2], [f64::MAX, 1.0, 1.0], [f64::MAX, 0.0, 0.0]),
    ] {
        assert!(matches!(
            m.quick_grid(counts, spacing, origin),
            Err(EditError::Invalid(_))
        ));
        assert_eq!(m, before);
    }
    m.is_locked = true;
    let before = m.clone();
    assert_eq!(
        m.quick_grid([2, 2, 2], [1.0; 3], [0.0; 3]),
        Err(EditError::Locked)
    );
    assert_eq!(m, before);
}

// --- quick draw ---------------------------------------------------------

#[test]
fn beams_and_columns_counts_and_coordinates() {
    let (nx, ny, nz) = (4usize, 3usize, 4usize);
    let mut m = grid_model(nx, ny, nz);
    let region = full(&m);
    let s = m.quick_draw_beams(region, true, true, "FSEC1").unwrap();
    // beams per level = ny (nx - 1) + nx (ny - 1)
    assert_eq!(s.frames_created.len(), nz * (ny * (nx - 1) + nx * (ny - 1)));
    assert_eq!(m.points.len(), nx * ny * nz);
    for f in m.frames.values() {
        let (a, b) = (pt(&m, &f.point_i), pt(&m, &f.point_j));
        assert_eq!(a[2], b[2], "beams are horizontal");
        let (da, db) = ((b[0] - a[0]), (b[1] - a[1]));
        // exactly one bay long, from the lower to the higher coordinate
        assert!(
            (close(da, 6.0, 1.0) && db == 0.0) || (close(db, 5.0, 1.0) && da == 0.0),
            "{a:?} -> {b:?}"
        );
    }
    let before = m.clone();
    // Everything exists already: refused, unchanged.
    assert!(matches!(
        m.quick_draw_beams(region, true, true, "FSEC1"),
        Err(EditError::Invalid(_))
    ));
    assert_eq!(m, before);
    let s = m.quick_draw_columns(region, "FSEC1").unwrap();
    assert_eq!(s.frames_created.len(), (nz - 1) * nx * ny);
    assert!(s.points_created.is_empty(), "joints are reused");
    for name in &s.frames_created {
        let f = &m.frames[name];
        let (a, b) = (pt(&m, &f.point_i), pt(&m, &f.point_j));
        assert_eq!((a[0], a[1]), (b[0], b[1]));
        assert!(close(b[2] - a[2], 4.0, 1.0));
    }
    // Sub-region: x 1..=2, y 0..=1, level 1 only, beams along X.
    let mut m = grid_model(nx, ny, nz);
    let r = GridRegion {
        x: (1, 2),
        y: (0, 1),
        z: (1, 1),
    };
    let s = m.quick_draw_beams(r, true, false, "FSEC1").unwrap();
    assert_eq!(s.frames_created.len(), 2); // 2 Y lines x 1 bay
    assert_eq!(s.points_created.len(), 4);
    assert!(find(&m, [6.0, 0.0, 4.0]).is_some() && find(&m, [12.0, 5.0, 4.0]).is_some());
}

#[test]
fn quick_draw_refusals_leave_the_model_unchanged() {
    let mut m = grid_model(3, 3, 2);
    let region = full(&m);
    let before = m.clone();
    assert!(m.quick_draw_beams(region, false, false, "FSEC1").is_err());
    assert!(m.quick_draw_beams(region, true, false, "NOPE").is_err());
    assert!(m.quick_draw_columns(region, "NOPE").is_err());
    assert!(
        m.quick_draw_areas(region, "FSEC1").is_err(),
        "frame section is not an area section"
    );
    let bad = GridRegion {
        x: (0, 3),
        ..region
    };
    assert!(m.quick_draw_beams(bad, true, false, "FSEC1").is_err());
    let reversed = GridRegion {
        x: (2, 1),
        ..region
    };
    assert!(m.quick_draw_columns(reversed, "FSEC1").is_err());
    assert_eq!(m, before);
    let mut empty = SapModel::default();
    assert!(empty.quick_draw_beams(region, true, true, "FSEC1").is_err());
    m.is_locked = true;
    let before = m.clone();
    for r in [
        m.quick_draw_beams(region, true, true, "FSEC1"),
        m.quick_draw_columns(region, "FSEC1"),
        m.quick_draw_areas(region, "ASEC1"),
        m.quick_draw_braces(region, VerticalPlane::XZ, BraceKind::X, "FSEC1"),
        m.quick_draw_secondary_beams(region, SpanDirection::X, 1, "FSEC1"),
    ] {
        assert_eq!(r, Err(EditError::Locked));
    }
    assert_eq!(m, before);
}

#[test]
fn x_braces_are_pinned_diagonals_of_every_bay() {
    // grid x = 0, 6, 12; y = 0, 5; z = 0, 4, 8
    let mut m = grid_model(3, 2, 3);
    let r = GridRegion {
        x: (0, 2),
        y: (1, 1),
        z: (0, 2),
    };
    let s = m
        .quick_draw_braces(r, VerticalPlane::XZ, BraceKind::X, "FSEC1")
        .unwrap();
    assert_eq!(s.frames_created.len(), 2 * 2 * 2); // 2 x-bays, 2 storeys, 2 diagonals
    assert_eq!(m.points.len(), 3 * 3); // joints on the plane y = 5 only
    let mut ends = std::collections::BTreeSet::new();
    for name in &s.frames_created {
        let f = &m.frames[name];
        // [P, V2, V3, T, M2, M3]: M2 and M3 released, torsion kept.
        for rel in [f.release_i, f.release_j] {
            assert_eq!(rel, [false, false, false, false, true, true]);
        }
        assert!(!f.auto_mesh_at_joints);
        let (a, b) = (pt(&m, &f.point_i), pt(&m, &f.point_j));
        assert_eq!((a[1], b[1]), (5.0, 5.0));
        assert!(close((b[0] - a[0]).abs(), 6.0, 1.0) && close((b[2] - a[2]).abs(), 4.0, 1.0));
        ends.insert((a.map(|v| (v * 8.0) as i64), b.map(|v| (v * 8.0) as i64)));
    }
    assert_eq!(ends.len(), 8);
    // Bay [0,6]x[0,4]: diagonals (0,0)-(6,4) and (6,0)-(0,4).
    let has = |a: [f64; 3], b: [f64; 3]| {
        s.frames_created.iter().any(|n| {
            let f = &m.frames[n];
            let (p, q) = (pt(&m, &f.point_i), pt(&m, &f.point_j));
            (vclose(p, a) && vclose(q, b)) || (vclose(p, b) && vclose(q, a))
        })
    };
    assert!(has([0.0, 5.0, 0.0], [6.0, 5.0, 4.0]));
    assert!(has([6.0, 5.0, 0.0], [0.0, 5.0, 4.0]));
    // YZ plane at x index 2 (x = 12): braces span Y.
    let mut m = grid_model(3, 2, 2);
    let r = GridRegion {
        x: (2, 2),
        y: (0, 1),
        z: (0, 1),
    };
    let s = m
        .quick_draw_braces(r, VerticalPlane::YZ, BraceKind::X, "FSEC1")
        .unwrap();
    assert_eq!(s.frames_created.len(), 2);
    for name in &s.frames_created {
        let f = &m.frames[name];
        assert_eq!((pt(&m, &f.point_i)[0], pt(&m, &f.point_j)[0]), (12.0, 12.0));
    }
}

#[test]
fn v_and_inverted_v_braces_meet_at_mid_beam_and_mesh_that_beam() {
    for kind in [BraceKind::V, BraceKind::InvertedV] {
        let mut m = grid_model(3, 2, 2); // x 0,6,12; y 0,5; z 0,4
        let region = full(&m);
        m.quick_draw_beams(region, true, false, "FSEC1").unwrap();
        let n_before = m.frames.len();
        let s = m
            .quick_draw_braces(region, VerticalPlane::XZ, kind, "FSEC1")
            .unwrap();
        // 2 y planes x 2 x-bays x 1 storey x 2 braces
        assert_eq!(s.frames_created.len(), 8);
        assert_eq!(m.frames.len(), n_before + 8);
        let apex_z = if kind == BraceKind::V { 0.0 } else { 4.0 };
        let corner_z = 4.0 - apex_z;
        for y in [0.0, 5.0] {
            for apex_x in [3.0, 9.0] {
                let apex = find(&m, [apex_x, y, apex_z]).expect("apex at mid beam");
                let attached: Vec<_> = s
                    .frames_created
                    .iter()
                    .map(|n| &m.frames[n])
                    .filter(|f| f.point_i == apex.name || f.point_j == apex.name)
                    .collect();
                assert_eq!(attached.len(), 2);
                for f in attached {
                    let other = if f.point_i == apex.name {
                        &f.point_j
                    } else {
                        &f.point_i
                    };
                    let o = pt(&m, other);
                    assert_eq!(o[2], corner_z);
                    assert!(close((o[0] - apex_x).abs(), 3.0, 1.0));
                    assert_eq!(f.release_i, [false, false, false, false, true, true]);
                    assert_eq!(f.release_j, [false, false, false, false, true, true]);
                }
            }
        }
        // Only beams at the apex level are meshed at joints.
        for f in m.frames.values() {
            let (a, b) = (pt(&m, &f.point_i), pt(&m, &f.point_j));
            let beam = !s.frames_created.contains(&f.name);
            if beam {
                assert_eq!(
                    f.auto_mesh_at_joints,
                    a[2] == apex_z && b[2] == apex_z,
                    "{f:?}"
                );
            } else {
                assert!(!f.auto_mesh_at_joints);
            }
        }
    }
}

#[test]
fn v_brace_without_apex_beam_is_refused_without_changes() {
    let mut m = grid_model(3, 2, 2);
    let region = full(&m);
    m.quick_draw_columns(region, "FSEC1").unwrap();
    // Beams only on the TOP level: V needs one at the bottom.
    let top = GridRegion {
        z: (1, 1),
        ..region
    };
    m.quick_draw_beams(top, true, false, "FSEC1").unwrap();
    let before = m.clone();
    assert!(matches!(
        m.quick_draw_braces(region, VerticalPlane::XZ, BraceKind::V, "FSEC1"),
        Err(EditError::Invalid(_))
    ));
    assert_eq!(m, before);
    assert!(m
        .quick_draw_braces(region, VerticalPlane::XZ, BraceKind::InvertedV, "FSEC1")
        .is_ok());
}

#[test]
fn secondary_beams_split_bays_at_i_over_n_plus_1_and_mesh_the_girders() {
    // x = 0, 6, 12 ; y = 0, 5, 10 ; one level z = 0
    let mut m = grid_model(3, 3, 1);
    let region = full(&m);
    m.quick_draw_beams(region, true, true, "FSEC1").unwrap();
    assert_eq!(m.frames.len(), 12);
    let n = 3;
    let s = m
        .quick_draw_secondary_beams(region, SpanDirection::X, n, "FSEC1")
        .unwrap();
    assert_eq!(s.frames_created.len(), 4 * n);
    // y = y0 + (y1 - y0) i / (n + 1)
    for (y0, y1) in [(0.0, 5.0), (5.0, 10.0)] {
        for i in 1..=n {
            let y = y0 + (y1 - y0) * i as f64 / (n as f64 + 1.0);
            for (xa, xb) in [(0.0, 6.0), (6.0, 12.0)] {
                assert!(
                    s.frames_created.iter().any(|nm| {
                        let f = &m.frames[nm];
                        let (a, b) = (pt(&m, &f.point_i), pt(&m, &f.point_j));
                        close(a[1], y, 10.0)
                            && close(b[1], y, 10.0)
                            && close(a[0], xa, 12.0)
                            && close(b[0], xb, 12.0)
                    }),
                    "beam at y={y} from x={xa} to {xb}"
                );
            }
        }
    }
    assert_eq!(s.points_created.len(), 3 * 2 * n);
    for name in &s.frames_created {
        let f = &m.frames[name];
        assert_eq!(f.release_i, [false, false, false, false, true, true]);
        assert_eq!(f.release_j, [false, false, false, false, true, true]);
        assert!(!f.auto_mesh_at_joints);
    }
    // Girders are the beams along Y (at x = 0, 6, 12); X-direction beams are not meshed.
    for f in m
        .frames
        .values()
        .filter(|f| !s.frames_created.contains(&f.name))
    {
        let (a, b) = (pt(&m, &f.point_i), pt(&m, &f.point_j));
        assert_eq!(f.auto_mesh_at_joints, a[0] == b[0], "{a:?} -> {b:?}");
    }
    // Y span: the beams along X are the girders.
    let mut m = grid_model(3, 3, 1);
    m.quick_draw_beams(region, true, true, "FSEC1").unwrap();
    let s = m
        .quick_draw_secondary_beams(region, SpanDirection::Y, 1, "FSEC1")
        .unwrap();
    assert_eq!(s.frames_created.len(), 4);
    assert!(find(&m, [3.0, 0.0, 0.0]).is_some() && find(&m, [3.0, 5.0, 0.0]).is_some());
    for f in m
        .frames
        .values()
        .filter(|f| !s.frames_created.contains(&f.name))
    {
        let (a, b) = (pt(&m, &f.point_i), pt(&m, &f.point_j));
        assert_eq!(f.auto_mesh_at_joints, a[1] == b[1]);
    }
}

#[test]
fn secondary_beams_refuse_missing_girders_and_bad_counts() {
    let mut m = grid_model(3, 3, 1);
    let region = full(&m);
    // Girders along Y exist only at x = 0.
    let r = GridRegion {
        x: (0, 0),
        ..region
    };
    m.quick_draw_beams(r, false, true, "FSEC1").unwrap();
    let before = m.clone();
    assert!(matches!(
        m.quick_draw_secondary_beams(region, SpanDirection::X, 2, "FSEC1"),
        Err(EditError::Invalid(_))
    ));
    assert!(m
        .quick_draw_secondary_beams(region, SpanDirection::X, 0, "FSEC1")
        .is_err());
    assert!(m
        .quick_draw_secondary_beams(region, SpanDirection::X, 1001, "FSEC1")
        .is_err());
    assert_eq!(m, before);
}

#[test]
fn areas_one_quad_per_cell_with_local_3_up() {
    let mut m = grid_model(3, 4, 2);
    let region = GridRegion {
        z: (1, 1),
        ..full(&m)
    };
    let s = m.quick_draw_areas(region, "ASEC1").unwrap();
    assert_eq!(s.areas_created.len(), 2 * 3);
    assert_eq!(s.points_created.len(), 3 * 4);
    let mut cells = std::collections::BTreeSet::new();
    for name in &s.areas_created {
        let a = &m.areas[name];
        assert_eq!(a.points.len(), 4);
        let c: Vec<[f64; 3]> = a.points.iter().map(|p| pt(&m, p)).collect();
        assert!(c.iter().all(|p| p[2] == 4.0));
        // shoelace: counter-clockwise seen from +Z <=> local 3 = +Z
        let area2: f64 = (0..4)
            .map(|i| c[i][0] * c[(i + 1) % 4][1] - c[(i + 1) % 4][0] * c[i][1])
            .sum();
        assert!(
            close(area2, 2.0 * 30.0, 100.0),
            "cell area 6 x 5 = 30, got {}",
            area2 / 2.0
        );
        assert_eq!(area_axes(&c, 0.0)[2], [0.0, 0.0, 1.0]);
        cells.insert((
            (c[0][0] / 6.0).round() as i64,
            (c[0][1] / 5.0).round() as i64,
        ));
    }
    assert_eq!(cells.len(), 6);
    let before = m.clone();
    assert!(m.quick_draw_areas(region, "ASEC1").is_err());
    assert_eq!(m, before);
}

// --- selection ------------------------------------------------------------

#[test]
fn select_by_section_adds_or_replaces() {
    let mut m = grid_model(3, 2, 2);
    let mut second = m.frame_sections["FSEC1"].clone();
    second.name = "FSEC2".into();
    m.frame_sections.insert("FSEC2".into(), second);
    let region = full(&m);
    m.quick_draw_columns(region, "FSEC1").unwrap();
    m.quick_draw_beams(region, true, false, "FSEC2").unwrap();
    m.quick_draw_areas(region, "ASEC1").unwrap();
    let cols = m.frames.values().filter(|f| f.section == "FSEC1").count();
    let beams = m.frames.values().filter(|f| f.section == "FSEC2").count();
    let (nx, ny, nz) = (3usize, 2usize, 2usize);
    assert_eq!(cols, (nz - 1) * nx * ny);
    assert_eq!(beams, nz * ny * (nx - 1)); // along X only
    assert_eq!(m.select_frames_by_section("FSEC1", true).unwrap(), cols);
    assert_eq!(m.frames.values().filter(|f| f.selected).count(), cols);
    assert_eq!(m.select_frames_by_section("FSEC2", false).unwrap(), beams);
    assert_eq!(
        m.frames.values().filter(|f| f.selected).count(),
        cols + beams
    );
    assert_eq!(
        m.select_areas_by_section("ASEC1", false).unwrap(),
        nz * (nx - 1) * (ny - 1)
    );
    assert!(m.areas.values().all(|a| a.selected));
    // replace clears the other object kinds too
    assert_eq!(m.select_frames_by_section("FSEC2", true).unwrap(), beams);
    assert_eq!(m.frames.values().filter(|f| f.selected).count(), beams);
    assert!(m.areas.values().all(|a| !a.selected));
    assert_eq!(
        m.select_areas_by_section("ASEC1", true).unwrap(),
        nz * (nx - 1) * (ny - 1)
    );
    assert!(m.frames.values().all(|f| !f.selected));
}

#[test]
fn select_by_section_refusals() {
    let mut m = grid_model(2, 2, 2);
    m.quick_draw_columns(full(&m), "FSEC1").unwrap();
    m.frames.values_mut().for_each(|f| f.selected = true);
    let before = m.clone();
    assert!(matches!(
        m.select_frames_by_section("NOPE", true),
        Err(EditError::Invalid(_))
    ));
    assert!(matches!(
        m.select_areas_by_section("FSEC1", true),
        Err(EditError::Invalid(_))
    ));
    assert!(matches!(
        m.select_frames_by_section("ASEC1", false),
        Err(EditError::Invalid(_))
    ));
    assert_eq!(m, before, "a refused replace must not clear the selection");
    m.is_locked = true;
    let before = m.clone();
    assert_eq!(
        m.select_frames_by_section("FSEC1", true),
        Err(EditError::Locked)
    );
    assert_eq!(
        m.select_areas_by_section("ASEC1", true),
        Err(EditError::Locked)
    );
    assert_eq!(m, before);
}

// --- diaphragms -------------------------------------------------------------

fn building() -> SapModel {
    // 3 x 3 lines, 4 levels: z = 0, 4, 8, 12
    let mut m = grid_model(3, 3, 4);
    let region = full(&m);
    m.quick_draw_columns(region, "FSEC1").unwrap();
    m.quick_draw_beams(region, true, true, "FSEC1").unwrap();
    m
}

#[test]
fn diaphragm_one_constraint_per_level() {
    let mut m = building();
    for p in m.points.values_mut().filter(|p| p.z > 0.0) {
        p.selected = true;
    }
    let names = m.assign_diaphragm("D", true).unwrap();
    assert_eq!(names, ["D_1", "D_2", "D_3"]); // levels z = 4, 8, 12
    assert_eq!(m.constraints.len(), 3);
    for (k, name) in names.iter().enumerate() {
        let c = &m.constraints[name];
        assert_eq!(c.constraint_type, ConstraintType::Diaphragm);
        assert_eq!(c.axis, ConstraintAxis::Z);
        let members: Vec<_> = m
            .points
            .values()
            .filter(|p| p.constraints.contains(name))
            .collect();
        assert_eq!(members.len(), 9);
        assert!(members.iter().all(|p| p.z == 4.0 * (k + 1) as f64));
    }
    assert!(m
        .points
        .values()
        .all(|p| p.constraints.len() == usize::from(p.z > 0.0)));
}

#[test]
fn diaphragm_single_constraint_and_reassignment_and_removal() {
    let mut m = building();
    m.points.values_mut().for_each(|p| p.selected = true);
    assert_eq!(m.assign_diaphragm("ALL", false).unwrap(), ["ALL"]);
    assert_eq!(m.constraints.len(), 1);
    assert!(m.points.values().all(|p| p.constraints == ["ALL"]));
    // Going to per-level moves the joints out of the single constraint.
    assert_eq!(m.assign_diaphragm("L", true).unwrap().len(), 4);
    assert!(m
        .points
        .values()
        .all(|p| p.constraints.len() == 1 && p.constraints[0] != "ALL"));
    // Removal from the roof joints deletes the generated constraint that empties.
    m.points.values_mut().for_each(|p| p.selected = p.z == 12.0);
    assert_eq!(m.remove_diaphragm_assignments().unwrap(), ["L_4"]);
    assert!(m.constraints.contains_key("L_3") && !m.constraints.contains_key("L_4"));
    assert!(m
        .points
        .values()
        .filter(|p| p.z == 12.0)
        .all(|p| p.constraints.is_empty()));
    // Joints moved to a user constraint leave L_1; a name without a _<digits>
    // suffix is never deleted by removal, and removal only touches constraints
    // the selected joints were in.
    m.points.values_mut().for_each(|p| p.selected = p.z == 4.0);
    m.assign_diaphragm("USER", false).unwrap();
    assert!(m
        .points
        .values()
        .filter(|p| p.z == 4.0)
        .all(|p| p.constraints == ["USER"]));
    assert!(m.remove_diaphragm_assignments().unwrap().is_empty());
    assert!(m.constraints.contains_key("USER"));
    assert!(m
        .points
        .values()
        .filter(|p| p.z == 4.0)
        .all(|p| p.constraints.is_empty()));
    // L_3 keeps its other joints when some are removed.
    m.points
        .values_mut()
        .for_each(|p| p.selected = p.z == 8.0 && p.x == 0.0);
    assert!(m.remove_diaphragm_assignments().unwrap().is_empty());
    assert!(m.constraints.contains_key("L_3"));
}

#[test]
fn diaphragm_refusals_leave_the_model_unchanged() {
    let mut m = building();
    let before = m.clone();
    assert_eq!(
        m.assign_diaphragm("D", false),
        Err(EditError::EmptySelection)
    );
    assert_eq!(
        m.remove_diaphragm_assignments(),
        Err(EditError::EmptySelection)
    );
    m.points.values_mut().for_each(|p| p.selected = true);
    let selected = m.clone();
    for bad in ["", " D", "D ", "A\nB"] {
        assert!(matches!(
            m.assign_diaphragm(bad, false),
            Err(EditError::Invalid(_))
        ));
    }
    m.constraints.insert(
        "EQ".into(),
        ConstraintDef {
            name: "EQ".into(),
            constraint_type: ConstraintType::Equal,
            axis: ConstraintAxis::Z,
            dofs: [true; 6],
        },
    );
    let with_eq = m.clone();
    assert!(matches!(
        m.assign_diaphragm("EQ", false),
        Err(EditError::Invalid(_))
    ));
    assert_eq!(m, with_eq);
    m.constraints.insert(
        "DX".into(),
        ConstraintDef {
            name: "DX".into(),
            constraint_type: ConstraintType::Diaphragm,
            axis: ConstraintAxis::X,
            dofs: [true; 6],
        },
    );
    let with_dx = m.clone();
    assert!(matches!(
        m.assign_diaphragm("DX", false),
        Err(EditError::Unsupported(_))
    ));
    assert_eq!(m, with_dx);
    // a clash on the SECOND level name must not leave the first one assigned
    m.constraints.insert(
        "Q_2".into(),
        ConstraintDef {
            name: "Q_2".into(),
            constraint_type: ConstraintType::Body,
            axis: ConstraintAxis::Z,
            dofs: [true; 6],
        },
    );
    let with_q = m.clone();
    assert!(m.assign_diaphragm("Q", true).is_err());
    assert_eq!(m, with_q);
    m.is_locked = true;
    let locked = m.clone();
    assert_eq!(m.assign_diaphragm("D", false), Err(EditError::Locked));
    assert_eq!(m.remove_diaphragm_assignments(), Err(EditError::Locked));
    assert_eq!(m, locked);
    let _ = (before, selected);
}

// --- radial extrusion ---------------------------------------------------------

fn dist_to_axis(p: [f64; 3], c: [f64; 2]) -> f64 {
    (p[0] - c[0]).hypot(p[1] - c[1])
}

#[test]
fn full_turn_extrusion_closes_without_duplicate_joints() {
    let mut m = SapModel::default();
    let c = [3.0, -2.0];
    m.add_point("1", c[0] + 5.0, c[1], 0.0);
    m.add_point("2", c[0] + 5.0, c[1], 3.0);
    m.add_frame("L", "1", "2", "FSEC1");
    m.frames.get_mut("L").unwrap().selected = true;
    let s = m
        .extrude_lines_to_areas_radial(c, 10.0, 36, 0.0, "ASEC1", false)
        .unwrap();
    assert_eq!(s.areas_created.len(), 36);
    assert_eq!(
        m.points.len(),
        72,
        "36 joints per ring, two rings, ring 36 = ring 0"
    );
    assert_eq!(s.points_created.len(), 70);
    // no duplicate joints
    let pts: Vec<_> = m.points.values().map(|p| [p.x, p.y, p.z]).collect();
    for i in 0..pts.len() {
        for j in (i + 1)..pts.len() {
            assert!(
                (0..3).any(|k| (pts[i][k] - pts[j][k]).abs() > 1e-6),
                "duplicate joint {i} {j}"
            );
        }
    }
    // exact sin/cos coordinates
    for k in 0..36 {
        let th = (10.0 * k as f64).to_radians();
        for z in [0.0, 3.0] {
            let want = [c[0] + 5.0 * th.cos(), c[1] + 5.0 * th.sin(), z];
            let p = find(&m, want).unwrap_or_else(|| panic!("ring joint {k} z={z}"));
            assert!(close(p.x, want[0], 10.0) && close(p.y, want[1], 10.0));
        }
    }
    // Every quad is a 5-radius, 3-high panel whose normal (i->j, then rotation
    // direction: up x counter-clockwise tangent) points at the axis.
    for name in &s.areas_created {
        let a = &m.areas[name];
        assert_eq!(a.points.len(), 4);
        let cs: Vec<[f64; 3]> = a.points.iter().map(|p| pt(&m, p)).collect();
        let e3 = area_axes(&cs, 0.0)[2];
        let mid = [
            (cs[0][0] + cs[1][0] + cs[2][0] + cs[3][0]) / 4.0 - c[0],
            (cs[0][1] + cs[1][1] + cs[2][1] + cs[3][1]) / 4.0 - c[1],
        ];
        let r = mid[0].hypot(mid[1]);
        let radial = [mid[0] / r, mid[1] / r];
        assert!(
            close(e3[0] * radial[0] + e3[1] * radial[1], -1.0, 1.0),
            "normal points inward"
        );
        assert!(e3[2].abs() < 1e-12);
        assert!(cs.iter().all(|p| close(dist_to_axis(*p, c), 5.0, 5.0)));
    }
    // Source frame stays.
    assert!(m.frames.contains_key("L"));
}

#[test]
fn extrusion_of_a_horizontal_line_has_local_3_up_and_can_delete_the_source() {
    let mut m = SapModel::default();
    m.add_point("1", 2.0, 0.0, 1.0);
    m.add_point("2", 5.0, 0.0, 1.0);
    m.add_frame("L", "1", "2", "FSEC1");
    m.frames.get_mut("L").unwrap().selected = true;
    let s = m
        .extrude_lines_to_areas_radial([0.0, 0.0], 30.0, 12, 0.0, "ASEC1", true)
        .unwrap();
    assert_eq!(s.areas_created.len(), 12);
    assert_eq!(s.frames_removed, ["L"]);
    assert!(m.frames.is_empty());
    assert_eq!(m.points.len(), 24);
    for name in &s.areas_created {
        let cs: Vec<[f64; 3]> = m.areas[name].points.iter().map(|p| pt(&m, p)).collect();
        assert_eq!(area_axes(&cs, 0.0)[2], [0.0, 0.0, 1.0]);
        // annular sector area = (theta/2)(R^2 - r^2) approximated by the planar quad:
        // shoelace area of the straight-edged quad (rs, 2 radii, chords).
        let a2: f64 = (0..4)
            .map(|i| cs[i][0] * cs[(i + 1) % 4][1] - cs[(i + 1) % 4][0] * cs[i][1])
            .sum();
        let th = 30.0f64.to_radians();
        let want = 0.5 * th.sin() * (25.0 - 4.0); // two triangles of the chord quad
        assert!(close(a2 / 2.0, want, 10.0));
    }
}

#[test]
fn extrusion_triangles_at_the_axis_and_helix_rise() {
    // Frame from the axis: triangles.
    let mut m = SapModel::default();
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 4.0, 0.0, 0.0);
    m.add_frame("L", "1", "2", "FSEC1");
    m.frames.get_mut("L").unwrap().selected = true;
    let s = m
        .extrude_lines_to_areas_radial([0.0, 0.0], 45.0, 8, 0.0, "ASEC1", false)
        .unwrap();
    assert_eq!(s.areas_created.len(), 8);
    assert_eq!(m.points.len(), 1 + 8);
    assert!(s.areas_created.iter().all(|n| m.areas[n].points.len() == 3));
    for name in &s.areas_created {
        let cs: Vec<[f64; 3]> = m.areas[name].points.iter().map(|p| pt(&m, p)).collect();
        assert_eq!(area_axes(&cs, 0.0)[2], [0.0, 0.0, 1.0]);
    }
    // Helix: horizontal line r = 2..4, 12 steps of 30 deg, total rise 6.
    let mut m = SapModel::default();
    m.add_point("1", 2.0, 0.0, 0.0);
    m.add_point("2", 4.0, 0.0, 0.0);
    m.add_frame("L", "1", "2", "FSEC1");
    m.frames.get_mut("L").unwrap().selected = true;
    let s = m
        .extrude_lines_to_areas_radial([0.0, 0.0], 30.0, 12, 6.0, "ASEC1", false)
        .unwrap();
    assert_eq!(s.areas_created.len(), 12);
    assert_eq!(m.points.len(), 2 + 2 * 12, "a helix does not close");
    for k in 0..=12 {
        let th = (30.0 * k as f64).to_radians();
        let z = 6.0 * k as f64 / 12.0;
        for r in [2.0, 4.0] {
            let p =
                find(&m, [r * th.cos(), r * th.sin(), z]).unwrap_or_else(|| panic!("k={k} r={r}"));
            assert!(close(p.z, z, 6.0));
        }
    }
}

#[test]
fn extrusion_refusals_leave_the_model_unchanged() {
    let mut m = SapModel::default();
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 0.0, 0.0, 3.0);
    m.add_point("3", 4.0, 0.0, 0.0);
    m.add_frame("AX", "1", "2", "FSEC1");
    m.add_frame("L", "1", "3", "FSEC1");
    let c = [0.0, 0.0];
    let before = m.clone();
    assert_eq!(
        m.extrude_lines_to_areas_radial(c, 10.0, 4, 0.0, "ASEC1", false),
        Err(EditError::EmptySelection)
    );
    m.frames.get_mut("AX").unwrap().selected = true;
    let sel = m.clone();
    assert!(matches!(
        m.extrude_lines_to_areas_radial(c, 10.0, 4, 0.0, "ASEC1", false),
        Err(EditError::Invalid(_))
    ));
    assert!(matches!(
        m.extrude_lines_to_areas_radial(c, 10.0, 4, 2.0, "ASEC1", false),
        Err(EditError::Invalid(_))
    ));
    assert_eq!(m, sel);
    m.frames.get_mut("AX").unwrap().selected = false;
    m.frames.get_mut("L").unwrap().selected = true;
    let sel = m.clone();
    for (angle, n, rise, sec) in [
        (0.0, 4, 0.0, "ASEC1"),
        (10.0, 0, 0.0, "ASEC1"),
        (10.0, 10_001, 0.0, "ASEC1"),
        (f64::NAN, 4, 0.0, "ASEC1"),
        (10.0, 4, f64::INFINITY, "ASEC1"),
        (10.0, 4, 0.0, "NOPE"),
        (360.0, 1, 0.0, "ASEC1"),
    ] {
        assert!(
            matches!(
                m.extrude_lines_to_areas_radial(c, angle, n, rise, sec, true),
                Err(EditError::Invalid(_))
            ),
            "{angle} {n} {rise} {sec}"
        );
        assert_eq!(m, sel);
    }
    // 360 deg in 4 steps, then again: the areas exist.
    m.extrude_lines_to_areas_radial(c, 90.0, 4, 0.0, "ASEC1", false)
        .unwrap();
    let after = m.clone();
    assert!(matches!(
        m.extrude_lines_to_areas_radial(c, 90.0, 4, 0.0, "ASEC1", false),
        Err(EditError::Invalid(_))
    ));
    assert_eq!(m, after);
    m.is_locked = true;
    let locked = m.clone();
    assert_eq!(
        m.extrude_lines_to_areas_radial(c, 10.0, 4, 0.0, "ASEC1", false),
        Err(EditError::Locked)
    );
    assert_eq!(m, locked);
    let _ = before;
}

// --- radial replication --------------------------------------------------------

#[test]
fn radial_copies_have_exact_coordinates_and_transformed_angles() {
    let c = [1.0, 2.0];
    let mut m = SapModel::default();
    // joints: column base/top (vertical up), a downward column, and a beam
    for (n, p) in [
        ("1", [5.0, 2.0, 0.0]),
        ("2", [5.0, 2.0, 3.0]),
        ("3", [3.0, 5.0, 3.0]),
        ("4", [3.0, 5.0, 0.0]),
        ("5", [3.0, 2.0, 3.0]),
    ] {
        m.add_point(n, p[0], p[1], p[2]);
    }
    m.add_frame("UP", "1", "2", "FSEC1"); // vertical, pointing up
    m.add_frame("DOWN", "3", "4", "FSEC1"); // vertical, pointing down
    m.add_frame("BEAM", "2", "5", "FSEC1"); // horizontal, not vertical
    m.frames.get_mut("UP").unwrap().angle_deg = 10.0;
    m.frames.get_mut("DOWN").unwrap().angle_deg = 20.0;
    m.frames.get_mut("BEAM").unwrap().angle_deg = 30.0;
    m.points.get_mut("1").unwrap().restraint = [true; 6];
    m.points.get_mut("1").unwrap().mass = [2.0, 2.0, 3.0, 4.0, 4.0, 5.0];
    for n in ["1", "2", "3", "4", "5"] {
        m.points.get_mut(n).unwrap().selected = true;
    }
    for f in m.frames.values_mut() {
        f.selected = true;
    }
    // joint 5 with explicit local axes, 4 with Euler angles
    m.points.get_mut("4").unwrap().local_axes_deg = [15.0, 25.0, 35.0];
    m.points.get_mut("5").unwrap().local_axes_matrix =
        Some([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]);
    let src = m.clone();
    let s = m.replicate_radial(c, 90.0, 3).unwrap();
    assert_eq!(s.points_created.len(), 15);
    assert_eq!(s.frames_created.len(), 9);
    for (copy, chunk) in s.frames_created.chunks(3).enumerate() {
        let k = copy as f64 + 1.0;
        let theta = 90.0 * k;
        for (orig, new_name) in ["UP", "DOWN", "BEAM"].iter().zip(chunk) {
            let (of, nf) = (&src.frames[*orig], &m.frames[new_name]);
            let rot = |p: [f64; 3]| {
                let r = rot_z([p[0] - c[0], p[1] - c[1], p[2]], theta);
                [r[0] + c[0], r[1] + c[1], r[2]]
            };
            assert!(vclose(pt(&m, &nf.point_i), rot(pt(&src, &of.point_i))));
            assert!(vclose(pt(&m, &nf.point_j), rot(pt(&src, &of.point_j))));
            // Angle: +k theta (up), -k theta (down), unchanged (not vertical).
            let want = match *orig {
                "UP" => 10.0 + theta,
                "DOWN" => 20.0 - theta,
                _ => 30.0,
            };
            assert!(
                close(nf.angle_deg, want, 400.0),
                "{orig}: {} vs {want}",
                nf.angle_deg
            );
            // The point of the angle rules: the rotated local axes are the
            // rotated original local axes.
            let ea = frame_axes(pt(&src, &of.point_i), pt(&src, &of.point_j), of.angle_deg);
            let eb = frame_axes(pt(&m, &nf.point_i), pt(&m, &nf.point_j), nf.angle_deg);
            for a in 0..3 {
                assert!(
                    vclose(eb[a], rot_z(ea[a], theta)),
                    "{orig} axis {a} copy {k}"
                );
            }
        }
    }
    // Joint assignments are copied; Euler angle A advances by k theta,
    // matrices are left-multiplied by Rz.
    for (copy, chunk) in s.points_created.chunks(5).enumerate() {
        let theta = 90.0 * (copy as f64 + 1.0);
        for (orig, new_name) in ["1", "2", "3", "4", "5"].iter().zip(chunk) {
            let (op, np) = (&src.points[*orig], &m.points[new_name]);
            assert_eq!(np.restraint, op.restraint);
            assert_eq!(np.mass, op.mass);
            let (ra, rb) = (joint_axes(op), joint_axes(np));
            let ident = op.local_axes_matrix.is_none() && op.local_axes_deg == [0.0; 3];
            for col in 0..3 {
                let va = [ra[0][col], ra[1][col], ra[2][col]];
                let vb = [rb[0][col], rb[1][col], rb[2][col]];
                if ident {
                    assert!(vclose(vb, va), "identity axes stay (assignments symmetric)");
                } else {
                    assert!(vclose(vb, rot_z(va, theta)), "joint {orig} axis {col}");
                }
            }
        }
    }
    // Originals untouched.
    for (id, p) in &src.points {
        assert_eq!(&m.points[id], p);
    }
    for (id, f) in &src.frames {
        assert_eq!(&m.frames[id], f);
    }
}

#[test]
fn radial_copies_of_areas_rotate_local_axes_with_the_geometry() {
    let c = [0.0, 0.0];
    let mut m = SapModel::default();
    // horizontal panel, normal +Z ; same corners reversed give normal -Z ; vertical wall
    let up = [
        [2.0, 0.0, 1.0],
        [4.0, 0.0, 1.0],
        [4.0, 1.0, 1.0],
        [2.0, 1.0, 1.0],
    ];
    let down = [
        [2.0, 3.0, 5.0],
        [2.0, 4.0, 5.0],
        [4.0, 4.0, 5.0],
        [4.0, 3.0, 5.0],
    ];
    let wall = [
        [2.0, 6.0, 0.0],
        [4.0, 6.0, 0.0],
        [4.0, 6.0, 2.0],
        [2.0, 6.0, 2.0],
    ];
    let tilted = [
        [2.0, 8.0, 0.0],
        [4.0, 8.0, 0.0],
        [4.0, 9.0, 2.0],
        [2.0, 9.0, 2.0],
    ];
    let mut names = Vec::new();
    for (i, quad) in [up, down, wall, tilted].iter().enumerate() {
        let ids: Vec<String> = quad
            .iter()
            .enumerate()
            .map(|(j, p)| m.add_point(format!("p{i}{j}"), p[0], p[1], p[2]))
            .collect();
        let n = m.add_area(format!("A{i}"), ids, "ASEC1");
        m.areas.get_mut(&n).unwrap().angle_deg = 17.0 + i as f64;
        m.areas.get_mut(&n).unwrap().selected = true;
        names.push(n);
    }
    m.area_sections.get_mut("ASEC1").unwrap().mat_angle_deg = 5.0;
    let src = m.clone();
    // axes signs: up -> +Z, down -> -Z
    let axes_of = |m: &SapModel, n: &str| {
        let a = &m.areas[n];
        let cs: Vec<[f64; 3]> = a.points.iter().map(|p| pt(m, p)).collect();
        // the element axes are the area's angle alone; the section's 5 degree material
        // angle turns the material, not the axes (Analysis Reference p. 193)
        area_axes(&cs, a.angle_deg)
    };
    assert_eq!(axes_of(&src, "A0")[2], [0.0, 0.0, 1.0]);
    assert_eq!(axes_of(&src, "A1")[2], [0.0, 0.0, -1.0]);
    let s = m.replicate_radial(c, 40.0, 2).unwrap();
    assert_eq!(s.areas_created.len(), 8);
    for (copy, chunk) in s.areas_created.chunks(4).enumerate() {
        let theta = 40.0 * (copy as f64 + 1.0);
        for (i, new_name) in chunk.iter().enumerate() {
            let before = axes_of(&src, &names[i]);
            let after = axes_of(&m, new_name);
            for a in 0..3 {
                assert!(
                    vclose(after[a], rot_z(before[a], theta)),
                    "area {i} axis {a} copy {copy}"
                );
            }
            let d = m.areas[new_name].angle_deg - src.areas[&names[i]].angle_deg;
            let want = match i {
                0 => theta,  // normal +Z
                1 => -theta, // normal -Z
                _ => 0.0,    // vertical wall / tilted panel: normal not vertical
            };
            assert!(close(d, want, 100.0), "area {i}: delta {d} want {want}");
        }
    }
}

#[test]
fn radial_merges_coincident_joints_with_equal_assignments_only() {
    let c = [0.0, 0.0];
    let build = || {
        let mut m = SapModel::default();
        m.add_point("1", 4.0, 0.0, 0.0);
        m.add_point("2", 4.0, 0.0, 3.0);
        m.add_point("3", 0.0, 4.0, 0.0); // where copy 1 of "1" lands
        m.add_frame("C", "1", "2", "FSEC1");
        m.frames.get_mut("C").unwrap().selected = true;
        m
    };
    let mut m = build();
    let s = m.replicate_radial(c, 90.0, 1).unwrap();
    assert_eq!(s.points_created.len(), 1, "joint 3 is reused");
    assert_eq!(m.frames[&s.frames_created[0]].point_i, "3");
    // a restraint on the existing joint makes the overlap incompatible
    let mut m = build();
    m.points.get_mut("3").unwrap().restraint[2] = true;
    let before = m.clone();
    assert!(matches!(
        m.replicate_radial(c, 90.0, 1),
        Err(EditError::JointConflict(_))
    ));
    assert_eq!(m, before);
    // a full-circle copy duplicates the source frame
    let mut m = build();
    let before = m.clone();
    assert!(matches!(
        m.replicate_radial(c, 90.0, 4),
        Err(EditError::Invalid(_))
    ));
    assert_eq!(m, before);
    // an on-axis joint is its own copy
    let mut m = SapModel::default();
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 3.0, 0.0, 0.0);
    m.add_frame("R", "1", "2", "FSEC1");
    m.frames.get_mut("R").unwrap().selected = true;
    m.points.get_mut("1").unwrap().restraint = [true; 6];
    let s = m.replicate_radial(c, 120.0, 2).unwrap();
    assert_eq!(m.points.len(), 1 + 3);
    assert_eq!(s.frames_created.len(), 2);
    for n in &s.frames_created {
        assert_eq!(m.frames[n].point_i, "1");
    }
}

fn radial_fixture() -> SapModel {
    let mut m = SapModel::default();
    m.add_point("1", 4.0, 0.0, 0.0);
    m.add_point("2", 4.0, 0.0, 3.0);
    m.add_point("3", 6.0, 0.0, 3.0);
    m.add_point("4", 6.0, 1.0, 3.0);
    m.add_frame("C", "1", "2", "FSEC1");
    m.add_frame("B", "2", "3", "FSEC1");
    m.add_area("A", vec!["2".into(), "3".into(), "4".into()], "ASEC1");
    m.points.values_mut().for_each(|p| p.selected = true);
    m.frames.values_mut().for_each(|p| p.selected = true);
    m.areas.values_mut().for_each(|p| p.selected = true);
    m
}

fn assert_refused_unchanged(mut m: SapModel, what: &str) {
    let before = m.clone();
    let r = m.replicate_radial([0.0, 0.0], 30.0, 2);
    assert!(
        matches!(
            r,
            Err(EditError::Unsupported(_)) | Err(EditError::Invalid(_))
        ),
        "{what}: {r:?}"
    );
    assert!(same(&m, &before), "{what}: model must not change");
}

#[test]
fn radial_replication_refuses_what_is_not_rotation_invariant() {
    // control: the fixture itself is accepted, and symmetric assignments too
    let mut ok = radial_fixture();
    ok.points.get_mut("1").unwrap().restraint = [true; 6];
    ok.points.get_mut("2").unwrap().mass = [1.0, 1.0, 2.0, 0.5, 0.5, 0.7];
    ok.points
        .get_mut("2")
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: [0.0, 0.0, -5.0, 0.0, 0.0, 2.0],
        });
    ok.frames
        .get_mut("B")
        .unwrap()
        .loads_dist
        .push(FrameLoadDistributed {
            load_pat: "DEAD".into(),
            my_type: 1,
            dir: 10,
            csys: "GLOBAL".into(),
            rel_dist1: 0.0,
            rel_dist2: 1.0,
            val1: 1.0,
            val2: 1.0,
        });
    ok.areas
        .get_mut("A")
        .unwrap()
        .loads_uniform
        .push(AreaLoadUniform {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            dir: 6,
            val: 1.0,
        });
    assert!(ok.clone().replicate_radial([0.0, 0.0], 30.0, 2).is_ok());

    let cases: Vec<(&str, Box<dyn Fn(&mut SapModel)>)> = vec![
        (
            "global X force",
            Box::new(|m| {
                m.points
                    .get_mut("2")
                    .unwrap()
                    .loads_force
                    .push(PointLoadForce {
                        load_pat: "DEAD".into(),
                        csys: "GLOBAL".into(),
                        force: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    })
            }),
        ),
        (
            "global MY",
            Box::new(|m| {
                m.points
                    .get_mut("2")
                    .unwrap()
                    .loads_force
                    .push(PointLoadForce {
                        load_pat: "DEAD".into(),
                        csys: "GLOBAL".into(),
                        force: [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                    })
            }),
        ),
        (
            "local force on global-axes joint",
            Box::new(|m| {
                m.points
                    .get_mut("2")
                    .unwrap()
                    .loads_force
                    .push(PointLoadForce {
                        load_pat: "DEAD".into(),
                        csys: "LOCAL".into(),
                        force: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    })
            }),
        ),
        (
            "global force on rotated-axes joint",
            Box::new(|m| {
                let p = m.points.get_mut("2").unwrap();
                p.local_axes_deg = [10.0, 0.0, 0.0];
                p.loads_force.push(PointLoadForce {
                    load_pat: "DEAD".into(),
                    csys: "GLOBAL".into(),
                    force: [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                })
            }),
        ),
        (
            "restraint UX only",
            Box::new(|m| m.points.get_mut("1").unwrap().restraint[0] = true),
        ),
        (
            "restraint RY only",
            Box::new(|m| m.points.get_mut("1").unwrap().restraint[4] = true),
        ),
        (
            "spring UY only",
            Box::new(|m| m.points.get_mut("1").unwrap().spring[1] = 5.0),
        ),
        (
            "coupled spring",
            Box::new(|m| m.points.get_mut("1").unwrap().spring_coupled = Some([1.0; 21])),
        ),
        (
            "mass UX != UY",
            Box::new(|m| m.points.get_mut("1").unwrap().mass[0] = 2.0),
        ),
        (
            "global spring on rotated joint",
            Box::new(|m| {
                let p = m.points.get_mut("1").unwrap();
                p.local_axes_deg = [0.0, 0.0, 20.0];
                p.spring[0] = 4.0;
            }),
        ),
        (
            "frame X load",
            Box::new(|m| {
                m.frames
                    .get_mut("B")
                    .unwrap()
                    .loads_dist
                    .push(FrameLoadDistributed {
                        load_pat: "DEAD".into(),
                        my_type: 1,
                        dir: 4,
                        csys: "GLOBAL".into(),
                        rel_dist1: 0.0,
                        rel_dist2: 1.0,
                        val1: 1.0,
                        val2: 1.0,
                    })
            }),
        ),
        (
            "frame projected Y load",
            Box::new(|m| {
                m.frames
                    .get_mut("B")
                    .unwrap()
                    .loads_dist
                    .push(FrameLoadDistributed {
                        load_pat: "DEAD".into(),
                        my_type: 1,
                        dir: 8,
                        csys: "GLOBAL".into(),
                        rel_dist1: 0.0,
                        rel_dist2: 1.0,
                        val1: 1.0,
                        val2: 1.0,
                    })
            }),
        ),
        (
            "frame gravity X",
            Box::new(|m| {
                m.frames
                    .get_mut("B")
                    .unwrap()
                    .loads_gravity
                    .insert("DEAD".into(), [1.0, 0.0, -1.0]);
            }),
        ),
        (
            "area global Y load",
            Box::new(|m| {
                m.areas
                    .get_mut("A")
                    .unwrap()
                    .loads_uniform
                    .push(AreaLoadUniform {
                        load_pat: "DEAD".into(),
                        csys: "GLOBAL".into(),
                        dir: 5,
                        val: 1.0,
                    })
            }),
        ),
        (
            "global joint offset",
            Box::new(|m| {
                let f = m.frames.get_mut("B").unwrap();
                f.joint_offset_is_global = true;
                f.joint_offset_i = [0.5, 0.0, 0.0];
            }),
        ),
        (
            "frame on the axis",
            Box::new(|m| {
                m.add_point("9", 0.0, 0.0, 0.0);
                m.add_point("10", 0.0, 0.0, 5.0);
                m.add_frame("AX", "9", "10", "FSEC1");
                for n in ["9", "10"] {
                    m.points.get_mut(n).unwrap().selected = true
                }
                m.frames.get_mut("AX").unwrap().selected = true;
            }),
        ),
        (
            "selected link",
            Box::new(|m| {
                m.links.insert(
                    "L1".into(),
                    LinkObj {
                        name: "L1".into(),
                        point_i: "1".into(),
                        point_j: Some("2".into()),
                        prop: "X".into(),
                        angle_deg: 0.0,
                        selected: true,
                    },
                );
            }),
        ),
        (
            "selected solid",
            Box::new(|m| {
                let p = [
                    "1".to_string(),
                    "2".into(),
                    "3".into(),
                    "4".into(),
                    "1".into(),
                    "2".into(),
                    "3".into(),
                    "4".into(),
                ];
                m.solids.insert(
                    "S".into(),
                    SolidObj {
                        name: "S".into(),
                        points: p,
                        section: "X".into(),
                        loads_surface: vec![],
                        loads_temp: Default::default(),
                        selected: true,
                    },
                );
            }),
        ),
        (
            "non-symmetric Body constraint",
            Box::new(|m| {
                m.constraints.insert(
                    "B1".into(),
                    ConstraintDef {
                        name: "B1".into(),
                        constraint_type: ConstraintType::Body,
                        axis: ConstraintAxis::Z,
                        dofs: [true, false, true, true, true, true],
                    },
                );
                m.points.get_mut("1").unwrap().constraints.push("B1".into());
            }),
        ),
        (
            "X diaphragm",
            Box::new(|m| {
                m.constraints.insert(
                    "DX".into(),
                    ConstraintDef {
                        name: "DX".into(),
                        constraint_type: ConstraintType::Diaphragm,
                        axis: ConstraintAxis::X,
                        dofs: [true; 6],
                    },
                );
                m.points.get_mut("1").unwrap().constraints.push("DX".into());
            }),
        ),
        (
            "non-finite coordinate",
            Box::new(|m| m.points.get_mut("1").unwrap().x = f64::NAN),
        ),
    ];
    for (what, edit) in cases {
        let mut m = radial_fixture();
        edit(&mut m);
        assert_refused_unchanged(m, what);
    }
    // Parameter refusals.
    let mut m = radial_fixture();
    let before = m.clone();
    for (center, angle, copies) in [
        ([0.0, 0.0], 0.0, 2),
        ([0.0, 0.0], 30.0, 0),
        ([0.0, 0.0], 30.0, 10_001),
        ([0.0, f64::NAN], 30.0, 2),
        ([0.0, 0.0], f64::INFINITY, 2),
    ] {
        assert!(m.replicate_radial(center, angle, copies).is_err());
        assert_eq!(m, before);
    }
    let mut empty = SapModel::default();
    assert_eq!(
        empty.replicate_radial([0.0; 2], 10.0, 1),
        Err(EditError::EmptySelection)
    );
    m.is_locked = true;
    let locked = m.clone();
    assert_eq!(
        m.replicate_radial([0.0, 0.0], 30.0, 2),
        Err(EditError::Locked)
    );
    assert_eq!(m, locked);
}

// --- reverse local 3 ----------------------------------------------------------

fn reversible(quad: [[f64; 3]; 4], angle: f64, mat_angle: f64) -> SapModel {
    let mut m = SapModel::default();
    m.area_sections.get_mut("ASEC1").unwrap().mat_angle_deg = mat_angle;
    let ids: Vec<String> = quad
        .iter()
        .enumerate()
        .map(|(j, p)| m.add_point(format!("q{j}"), p[0], p[1], p[2]))
        .collect();
    let n = m.add_area("A", ids, "ASEC1");
    let a = m.areas.get_mut(&n).unwrap();
    a.angle_deg = angle;
    a.selected = true;
    a.loads_uniform = [1, 2, 3, 6, 10]
        .iter()
        .enumerate()
        .map(|(i, d)| AreaLoadUniform {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            dir: *d,
            val: 1.5 + i as f64,
        })
        .collect();
    a.loads_pressure = [-1, -2]
        .iter()
        .map(|f| AreaLoadSurfacePressure {
            load_pat: "DEAD".into(),
            face: *f,
            val: 2.0 + *f as f64,
            pattern_name: None,
        })
        .collect();
    a.loads_temp = vec![
        AreaLoadTemperature {
            load_pat: "T".into(),
            my_type: 1,
            val: 7.0,
            pattern_name: None,
        },
        AreaLoadTemperature {
            load_pat: "T".into(),
            my_type: 2,
            val: 3.0,
            pattern_name: None,
        },
    ];
    m
}

fn orientations() -> Vec<([[f64; 3]; 4], f64, f64)> {
    vec![
        // horizontal, normal +Z
        (
            [
                [0.0, 0.0, 1.0],
                [4.0, 0.0, 1.0],
                [4.0, 3.0, 1.0],
                [0.0, 3.0, 1.0],
            ],
            0.0,
            0.0,
        ),
        (
            [
                [0.0, 0.0, 1.0],
                [4.0, 0.0, 1.0],
                [4.0, 3.0, 1.0],
                [0.0, 3.0, 1.0],
            ],
            25.0,
            10.0,
        ),
        // horizontal, normal -Z
        (
            [
                [0.0, 0.0, 1.0],
                [0.0, 3.0, 1.0],
                [4.0, 3.0, 1.0],
                [4.0, 0.0, 1.0],
            ],
            35.0,
            -5.0,
        ),
        // vertical wall
        (
            [
                [0.0, 0.0, 0.0],
                [4.0, 0.0, 0.0],
                [4.0, 0.0, 3.0],
                [0.0, 0.0, 3.0],
            ],
            20.0,
            0.0,
        ),
        // inclined and skewed in plan
        (
            [
                [1.0, 1.0, 0.0],
                [5.0, 2.0, 0.0],
                [6.0, 4.0, 2.0],
                [2.0, 3.0, 2.0],
            ],
            40.0,
            12.0,
        ),
    ]
}

fn axes_of_area(m: &SapModel) -> [[f64; 3]; 3] {
    let a = &m.areas["A"];
    let cs: Vec<[f64; 3]> = a.points.iter().map(|p| pt(m, p)).collect();
    // the material angle of the section does not turn the element axes
    area_axes(&cs, a.angle_deg)
}

fn vec_of_uniform(a: &sap_model::AreaObj, e: &[[f64; 3]; 3]) -> [f64; 3] {
    // physical load vector of local-direction loads (dirs 1..3)
    let mut v = [0.0; 3];
    for l in a.loads_uniform.iter().filter(|l| (1..=3).contains(&l.dir)) {
        for k in 0..3 {
            v[k] += l.val * e[l.dir as usize - 1][k];
        }
    }
    v
}

#[test]
fn reverse_same_global_keeps_every_physical_load() {
    for (quad, angle, mat) in orientations() {
        let src = reversible(quad, angle, mat);
        let mut m = src.clone();
        assert_eq!(m.reverse_area_local_3(ReverseKeep::SameGlobalLoads), Ok(1));
        let (e0, e1) = (axes_of_area(&src), axes_of_area(&m));
        // joint order reversed
        let mut rev = src.areas["A"].points.clone();
        rev.reverse();
        assert_eq!(m.areas["A"].points, rev);
        // local 3 reversed, local 1 kept, local 2 reversed
        for k in 0..3 {
            assert!((e1[2][k] + e0[2][k]).abs() < 1e-12);
            assert!(
                (e1[0][k] - e0[0][k]).abs() < 1e-12,
                "e1 {:?} vs {:?}",
                e1[0],
                e0[0]
            );
            assert!((e1[1][k] + e0[1][k]).abs() < 1e-12);
        }
        // physical in-plane + normal load unchanged
        let (v0, v1) = (
            vec_of_uniform(&src.areas["A"], &e0),
            vec_of_uniform(&m.areas["A"], &e1),
        );
        assert!(vclose(v0, v1), "{v0:?} vs {v1:?}");
        // global loads (dirs 6, 10) unchanged in value
        for d in [6, 10] {
            let a = src.areas["A"]
                .loads_uniform
                .iter()
                .find(|l| l.dir == d)
                .unwrap()
                .val;
            let b = m.areas["A"]
                .loads_uniform
                .iter()
                .find(|l| l.dir == d)
                .unwrap()
                .val;
            assert_eq!(a, b);
        }
        // pressure: top <-> bottom with unchanged value (value = 2 + face)
        let pf = |m: &SapModel, f: i32| {
            m.areas["A"]
                .loads_pressure
                .iter()
                .find(|l| l.face == f)
                .map(|l| l.val)
        };
        assert_eq!(pf(&m, -2), Some(2.0 - 1.0)); // old bottom (-1) had 1.0
        assert_eq!(pf(&m, -1), Some(2.0 - 2.0)); // old top (-2) had 0.0
                                                 // temperature: gradient flips sign, uniform unchanged
        let t = &m.areas["A"].loads_temp;
        assert_eq!((t[0].val, t[1].val), (7.0, -3.0));
        // the angle: the area angle becomes 180 - a (tilted) or -a (vertical normal)
        assert!(m.areas["A"].selected);
    }
}

#[test]
fn reverse_same_local_keeps_numbers_and_turns_loads_with_the_axes() {
    for (quad, angle, mat) in orientations() {
        let mut src = reversible(quad, angle, mat);
        src.areas
            .get_mut("A")
            .unwrap()
            .loads_pressure
            .retain(|l| matches!(l.face, -1 | -2));
        let mut m = src.clone();
        m.reverse_area_local_3(ReverseKeep::SameLocalLoads).unwrap();
        let (e0, e1) = (axes_of_area(&src), axes_of_area(&m));
        assert_eq!(m.areas["A"].loads_uniform, src.areas["A"].loads_uniform);
        assert_eq!(m.areas["A"].loads_pressure, src.areas["A"].loads_pressure);
        assert_eq!(m.areas["A"].loads_temp, src.areas["A"].loads_temp);
        // physical vector: local 1 component same, local 2 and 3 components flipped
        let (v0, v1) = (
            vec_of_uniform(&src.areas["A"], &e0),
            vec_of_uniform(&m.areas["A"], &e1),
        );
        let l = |d: i32| {
            src.areas["A"]
                .loads_uniform
                .iter()
                .find(|x| x.dir == d)
                .unwrap()
                .val
        };
        let expected = {
            let mut v = [0.0; 3];
            for k in 0..3 {
                v[k] = l(1) * e0[0][k] - l(2) * e0[1][k] - l(3) * e0[2][k];
            }
            v
        };
        assert!(vclose(v1, expected), "{v1:?} vs {expected:?}");
        assert!(!vclose(v0, v1));
    }
}

#[test]
fn reverse_twice_restores_the_area_and_triangles_reverse_too() {
    for (quad, angle, mat) in orientations() {
        let src = reversible(quad, angle, mat);
        let mut m = src.clone();
        m.reverse_area_local_3(ReverseKeep::SameGlobalLoads)
            .unwrap();
        m.reverse_area_local_3(ReverseKeep::SameGlobalLoads)
            .unwrap();
        let (a, b) = (&src.areas["A"], &m.areas["A"]);
        assert_eq!(a.points, b.points);
        let total = |x: f64| x.rem_euclid(360.0);
        let d = (total(a.angle_deg) - total(b.angle_deg)).abs();
        assert!(
            d < 1e-9 || (360.0 - d) < 1e-9,
            "angle {} vs {}",
            a.angle_deg,
            b.angle_deg
        );
        for (x, y) in a.loads_uniform.iter().zip(&b.loads_uniform) {
            assert!((x.val - y.val).abs() < 1e-12);
        }
        assert_eq!(a.loads_pressure, b.loads_pressure);
    }
    // triangle: normal reverses, edges 1<->2
    let mut m = SapModel::default();
    for (n, p) in [
        ("1", [0.0, 0.0, 0.0]),
        ("2", [4.0, 0.0, 0.0]),
        ("3", [0.0, 3.0, 2.0]),
    ] {
        m.add_point(n, p[0], p[1], p[2]);
    }
    m.add_area("T", vec!["1".into(), "2".into(), "3".into()], "ASEC1");
    let a = m.areas.get_mut("T").unwrap();
    a.selected = true;
    a.angle_deg = 12.0;
    a.loads_pressure = vec![-1, -2]
        .into_iter()
        .map(|f| AreaLoadSurfacePressure {
            load_pat: "D".into(),
            face: f,
            val: 10.0 + f as f64,
            pattern_name: None,
        })
        .collect();
    let cs =
        |m: &SapModel| -> Vec<[f64; 3]> { m.areas["T"].points.iter().map(|p| pt(m, p)).collect() };
    let e0 = area_axes(&cs(&m), 12.0);
    m.reverse_area_local_3(ReverseKeep::SameGlobalLoads)
        .unwrap();
    assert_eq!(m.areas["T"].points, ["3", "2", "1"]);
    let e1 = area_axes(&cs(&m), m.areas["T"].angle_deg);
    for k in 0..3 {
        assert!((e1[2][k] + e0[2][k]).abs() < 1e-12 && (e1[0][k] - e0[0][k]).abs() < 1e-12);
    }
    let v = |f: i32| {
        m.areas["T"]
            .loads_pressure
            .iter()
            .find(|l| l.face == f)
            .unwrap()
            .val
    };
    assert_eq!((v(-2), v(-1)), (9.0, 8.0)); // old bottom 9, old top 8
}

#[test]
fn reverse_refusals_leave_the_model_unchanged() {
    let base = reversible(orientations()[0].0, 0.0, 0.0);
    // edge pressure cannot be transformed exactly, in either mode
    for keep in [ReverseKeep::SameLocalLoads, ReverseKeep::SameGlobalLoads] {
        let mut m = base.clone();
        m.areas
            .get_mut("A")
            .unwrap()
            .loads_pressure
            .push(AreaLoadSurfacePressure {
                load_pat: "DEAD".into(),
                face: 3,
                val: 1.0,
                pattern_name: None,
            });
        let before = m.clone();
        assert!(matches!(
            m.reverse_area_local_3(keep),
            Err(EditError::Unsupported(_))
        ));
        assert_eq!(m, before);
    }
    // unknown pressure face / dir / temperature type
    for edit in [
        (|a: &mut sap_model::AreaObj| a.loads_pressure[0].face = 9) as fn(&mut sap_model::AreaObj),
        |a| a.loads_uniform[0].dir = 42,
        |a| a.loads_temp[0].my_type = 5,
        |a| a.angle_deg = f64::NAN,
    ] {
        let mut m = base.clone();
        edit(m.areas.get_mut("A").unwrap());
        let before = m.clone();
        assert!(m
            .reverse_area_local_3(ReverseKeep::SameGlobalLoads)
            .is_err());
        assert!(same(&m, &before));
    }
    // 5-joint area
    let mut m = base.clone();
    m.add_point("x", 9.0, 9.0, 1.0);
    m.areas.get_mut("A").unwrap().points.push("x".into());
    let before = m.clone();
    assert!(matches!(
        m.reverse_area_local_3(ReverseKeep::SameGlobalLoads),
        Err(EditError::Unsupported(_))
    ));
    assert_eq!(m, before);
    // missing section, no selection, locked
    let mut m = base.clone();
    m.areas.get_mut("A").unwrap().section = "NOPE".into();
    let before = m.clone();
    assert!(m
        .reverse_area_local_3(ReverseKeep::SameGlobalLoads)
        .is_err());
    assert_eq!(m, before);
    let mut m = base.clone();
    m.areas.get_mut("A").unwrap().selected = false;
    assert_eq!(
        m.reverse_area_local_3(ReverseKeep::SameGlobalLoads),
        Err(EditError::EmptySelection)
    );
    let mut m = base;
    m.is_locked = true;
    let before = m.clone();
    assert_eq!(
        m.reverse_area_local_3(ReverseKeep::SameGlobalLoads),
        Err(EditError::Locked)
    );
    assert_eq!(m, before);
    // a later bad area must not leave an earlier one reversed
    let mut m = reversible(orientations()[0].0, 0.0, 0.0);
    for (n, p) in [
        ("a", [0.0, 0.0, 9.0]),
        ("b", [1.0, 0.0, 9.0]),
        ("c", [0.0, 1.0, 9.0]),
    ] {
        m.add_point(n, p[0], p[1], p[2]);
    }
    let mut bad = sap_model::AreaObj::new("Z", vec!["a".into(), "b".into(), "c".into()], "ASEC1");
    bad.selected = true;
    bad.loads_uniform.push(AreaLoadUniform {
        load_pat: "D".into(),
        csys: "GLOBAL".into(),
        dir: 77,
        val: 1.0,
    });
    m.areas.insert("Z".into(), bad);
    m.areas
        .get_mut("A")
        .unwrap()
        .loads_pressure
        .retain(|l| matches!(l.face, -1 | -2));
    let before = m.clone();
    assert!(m
        .reverse_area_local_3(ReverseKeep::SameGlobalLoads)
        .is_err());
    assert_eq!(m, before);
}

// --- curved frames --------------------------------------------------------------

fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[test]
fn circular_curve_has_equal_angle_steps_on_the_circle() {
    // Known circle: centre C, radius R, in the plane spanned by ex and ey.
    let c = [1.0, 2.0, 3.0];
    let r = 5.0;
    let ex = [1.0, 0.0, 0.0];
    let (s30, c30) = 30.0f64.to_radians().sin_cos();
    let ey = [0.0, c30, s30];
    let on = |deg: f64| -> [f64; 3] {
        let (s, co) = deg.to_radians().sin_cos();
        std::array::from_fn(|k| c[k] + r * (co * ex[k] + s * ey[k]))
    };
    for (a0, a1, a2, n) in [
        (20.0, 100.0, 200.0, 6usize),
        (200.0, 100.0, 20.0, 6),
        (10.0, 300.0, 350.0, 7),
    ] {
        let mut m = SapModel::default();
        let s = m
            .curved_frame(on(a0), on(a2), on(a1), CurveKind::Circular, n, "FSEC1")
            .unwrap();
        assert_eq!(s.frames_created.len(), n);
        assert_eq!(m.points.len(), n + 1);
        // chain from start to end
        let mut order = vec![m.frames[&s.frames_created[0]].point_i.clone()];
        for f in &s.frames_created {
            let f = &m.frames[f];
            assert_eq!(&f.point_i, order.last().unwrap());
            order.push(f.point_j.clone());
        }
        let step = (a2 - a0) / n as f64;
        // Sweep from start to end through the third point: ccw if a1 between a0 and a2 going up.
        for (k, id) in order.iter().enumerate() {
            let want = on(a0 + step * k as f64);
            let got = pt(&m, id);
            let d = [got[0] - c[0], got[1] - c[1], got[2] - c[2]];
            assert!(close((dot3(d, d)).sqrt(), r, r), "k={k} on circle");
            assert!(
                dot3(d, [0.0, -s30, c30]).abs() < 1e-12 * r,
                "k={k} in plane"
            );
            assert!(
                (0..3).all(|q| (got[q] - want[q]).abs() < 1e-12 * r),
                "k={k} {got:?} vs {want:?}"
            );
        }
    }
}

#[test]
fn parabolic_curve_uses_equal_chord_steps() {
    // y = 3 s (10 - s) / (4 * 6) through (0,0), (4,3), (10,0)
    let mut m = SapModel::default();
    let s = m
        .curved_frame(
            [0.0; 3],
            [10.0, 0.0, 0.0],
            [4.0, 3.0, 0.0],
            CurveKind::Parabolic,
            5,
            "FSEC1",
        )
        .unwrap();
    assert_eq!(s.frames_created.len(), 5);
    for k in 0..=5 {
        let x = 2.0 * k as f64;
        let y = 3.0 * x * (10.0 - x) / 24.0;
        let p = find(&m, [x, y, 0.0]).unwrap_or_else(|| panic!("k={k}"));
        assert!(close(p.y, y, 10.0));
    }
    // tilted chord in 3D: u = (0.6, 0.8, 0), third point 4 along the chord, 2.5 above
    let mut m = SapModel::default();
    let a = [1.0, 2.0, 3.0];
    let b = [7.0, 10.0, 3.0];
    let t = [a[0] + 4.0 * 0.6, a[1] + 4.0 * 0.8, 3.0 + 2.5];
    m.curved_frame(a, b, t, CurveKind::Parabolic, 4, "FSEC1")
        .unwrap();
    for k in 0..=4 {
        let s = 10.0 * k as f64 / 4.0;
        let h = 2.5 * s * (10.0 - s) / 24.0;
        let want = [a[0] + 0.6 * s, a[1] + 0.8 * s, 3.0 + h];
        assert!(find(&m, want).is_some(), "k={k} {want:?}");
    }
}

#[test]
fn curved_frame_reuses_existing_end_joints_and_refusals_leave_the_model() {
    let mut m = SapModel::default();
    m.add_point("S", 0.0, 0.0, 0.0);
    m.points.get_mut("S").unwrap().restraint = [true; 6];
    m.curved_frame(
        [0.0; 3],
        [10.0, 0.0, 0.0],
        [5.0, 4.0, 0.0],
        CurveKind::Circular,
        4,
        "FSEC1",
    )
    .unwrap();
    assert_eq!(m.points.len(), 5);
    assert_eq!(m.points["S"].restraint, [true; 6]);
    let mut m = SapModel::default();
    let before = m.clone();
    let cases: Vec<(&str, [f64; 3], [f64; 3], [f64; 3], CurveKind, usize, &str)> = vec![
        (
            "collinear circular",
            [0.0; 3],
            [10.0, 0.0, 0.0],
            [5.0, 0.0, 0.0],
            CurveKind::Circular,
            4,
            "FSEC1",
        ),
        (
            "collinear parabolic",
            [0.0; 3],
            [10.0, 0.0, 0.0],
            [5.0, 0.0, 0.0],
            CurveKind::Parabolic,
            4,
            "FSEC1",
        ),
        (
            "third = start",
            [0.0; 3],
            [10.0, 0.0, 0.0],
            [0.0; 3],
            CurveKind::Circular,
            4,
            "FSEC1",
        ),
        (
            "start = end",
            [0.0; 3],
            [0.0; 3],
            [0.0, 1.0, 0.0],
            CurveKind::Circular,
            4,
            "FSEC1",
        ),
        (
            "parabola third beyond end",
            [0.0; 3],
            [10.0, 0.0, 0.0],
            [12.0, 3.0, 0.0],
            CurveKind::Parabolic,
            4,
            "FSEC1",
        ),
        (
            "parabola third at end coordinate",
            [0.0; 3],
            [10.0, 0.0, 0.0],
            [10.0, 3.0, 0.0],
            CurveKind::Parabolic,
            4,
            "FSEC1",
        ),
        (
            "parabola third before start",
            [0.0; 3],
            [10.0, 0.0, 0.0],
            [-1.0, 3.0, 0.0],
            CurveKind::Parabolic,
            4,
            "FSEC1",
        ),
        (
            "one segment",
            [0.0; 3],
            [10.0, 0.0, 0.0],
            [5.0, 3.0, 0.0],
            CurveKind::Circular,
            1,
            "FSEC1",
        ),
        (
            "too many",
            [0.0; 3],
            [10.0, 0.0, 0.0],
            [5.0, 3.0, 0.0],
            CurveKind::Circular,
            10_001,
            "FSEC1",
        ),
        (
            "missing section",
            [0.0; 3],
            [10.0, 0.0, 0.0],
            [5.0, 3.0, 0.0],
            CurveKind::Circular,
            4,
            "NOPE",
        ),
        (
            "non-finite",
            [0.0; 3],
            [10.0, 0.0, 0.0],
            [5.0, f64::NAN, 0.0],
            CurveKind::Circular,
            4,
            "FSEC1",
        ),
    ];
    for (what, a, b, t, kind, n, sec) in cases {
        let r = m.curved_frame(a, b, t, kind, n, sec);
        assert!(matches!(r, Err(EditError::Invalid(_))), "{what}: {r:?}");
        assert_eq!(m, before, "{what}");
    }
    // existing frame in the way
    m.curved_frame(
        [0.0; 3],
        [10.0, 0.0, 0.0],
        [5.0, 3.0, 0.0],
        CurveKind::Parabolic,
        2,
        "FSEC1",
    )
    .unwrap();
    let after = m.clone();
    assert!(m
        .curved_frame(
            [0.0; 3],
            [10.0, 0.0, 0.0],
            [5.0, 3.0, 0.0],
            CurveKind::Parabolic,
            2,
            "FSEC1"
        )
        .is_err());
    assert_eq!(m, after);
    m.is_locked = true;
    let locked = m.clone();
    assert_eq!(
        m.curved_frame(
            [0.0; 3],
            [10.0, 0.0, 0.0],
            [5.0, 3.0, 0.0],
            CurveKind::Circular,
            4,
            "FSEC1"
        ),
        Err(EditError::Locked)
    );
    assert_eq!(m, locked);
}

#[test]
fn accepted_geometry_edits_clear_stale_results_and_selection_edits_do_not() {
    let mut m = grid_model(2, 2, 2);
    m.case_results.insert("DEAD".into(), Default::default());
    m.quick_draw_columns(full(&m), "FSEC1").unwrap();
    assert!(m.case_results.is_empty());
    m.case_results.insert("DEAD".into(), Default::default());
    m.select_frames_by_section("FSEC1", true).unwrap();
    assert!(!m.case_results.is_empty());
}

// --- further negative tests ------------------------------------------------------

#[test]
fn per_level_diaphragm_never_spans_two_levels() {
    let mut m = building();
    m.points.values_mut().for_each(|p| p.selected = p.z > 0.0);
    let names = m.assign_diaphragm("D", true).unwrap();
    assert_eq!(names, ["D_1", "D_2", "D_3"]);
    // idempotent with the same selection
    let again = m.clone();
    assert_eq!(m.assign_diaphragm("D", true).unwrap(), names);
    assert_eq!(m, again);
    // Selecting only the level z = 8 would name it D_1, which holds z = 4 joints.
    m.points.values_mut().for_each(|p| p.selected = p.z == 8.0);
    let before = m.clone();
    assert!(matches!(
        m.assign_diaphragm("D", true),
        Err(EditError::Invalid(_))
    ));
    assert_eq!(m, before);
}

#[test]
fn zero_area_extrusion_steps_are_refused() {
    for (a, b) in [
        ([2.0, 0.0, 0.0], [4.0, 0.0, 0.0]),
        ([0.0; 3], [4.0, 0.0, 0.0]),
    ] {
        let mut m = SapModel::default();
        m.add_point("1", a[0], a[1], a[2]);
        m.add_point("2", b[0], b[1], b[2]);
        m.add_frame("L", "1", "2", "FSEC1");
        m.frames.get_mut("L").unwrap().selected = true;
        let before = m.clone();
        // a half turn in one step: the four joints are collinear
        assert!(matches!(
            m.extrude_lines_to_areas_radial([0.0, 0.0], 180.0, 1, 0.0, "ASEC1", false),
            Err(EditError::Invalid(_))
        ));
        assert_eq!(m, before);
        // two half-turn steps give a degenerate first step as well
        assert!(m
            .extrude_lines_to_areas_radial([0.0, 0.0], 180.0, 2, 0.0, "ASEC1", false)
            .is_err());
        assert_eq!(m, before);
    }
}

fn selected_everything(m: &mut SapModel) {
    m.points.values_mut().for_each(|p| p.selected = true);
    m.frames.values_mut().for_each(|p| p.selected = true);
    m.areas.values_mut().for_each(|p| p.selected = true);
}

#[test]
fn radial_replication_refuses_displacements_point_loads_and_constraints() {
    let base = || {
        let mut m = radial_fixture();
        selected_everything(&mut m);
        m
    };
    let run = |mut m: SapModel| {
        let before = m.clone();
        let r = m.replicate_radial([0.0, 0.0], 30.0, 2);
        if r.is_err() {
            assert_eq!(m, before, "model must not change");
        }
        r
    };
    // joint displacement load with X/Y (or RX/RY) components, global axes
    for k in [0usize, 1, 3, 4] {
        let mut m = base();
        let mut displ = [0.0; 6];
        displ[k] = 0.1;
        m.points
            .get_mut("2")
            .unwrap()
            .loads_displ
            .push(sap_model::PointLoadDispl {
                load_pat: "DEAD".into(),
                csys: "GLOBAL".into(),
                displ,
            });
        assert!(matches!(run(m), Err(EditError::Unsupported(_))), "dof {k}");
    }
    // a Z displacement is fine
    let mut m = base();
    m.points
        .get_mut("2")
        .unwrap()
        .loads_displ
        .push(sap_model::PointLoadDispl {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            displ: [0.0, 0.0, 0.1, 0.0, 0.0, 0.2],
        });
    assert!(run(m).is_ok());
    // frame point loads
    for (dir, ok) in [
        (4, false),
        (5, false),
        (7, false),
        (6, true),
        (10, true),
        (2, true),
    ] {
        let mut m = base();
        m.frames
            .get_mut("B")
            .unwrap()
            .loads_point
            .push(sap_model::FrameLoadPoint {
                load_pat: "DEAD".into(),
                my_type: 1,
                dir,
                csys: "GLOBAL".into(),
                rel_dist: 0.5,
                val: 1.0,
            });
        let r = run(m);
        assert_eq!(r.is_ok(), ok, "dir {dir}: {r:?}");
        if !ok {
            assert!(matches!(r, Err(EditError::Unsupported(_))));
        }
    }
    // a load in a user coordinate system is not assumed to be global
    let mut m = base();
    m.frames
        .get_mut("B")
        .unwrap()
        .loads_dist
        .push(FrameLoadDistributed {
            load_pat: "DEAD".into(),
            my_type: 1,
            dir: 6,
            csys: "SKEW".into(),
            rel_dist1: 0.0,
            rel_dist2: 1.0,
            val1: 1.0,
            val2: 1.0,
        });
    assert!(matches!(run(m), Err(EditError::Unsupported(_))));
    // constraints
    let mut m = base();
    m.constraints.insert(
        "PL".into(),
        ConstraintDef {
            name: "PL".into(),
            constraint_type: ConstraintType::Plate,
            axis: ConstraintAxis::Z,
            dofs: [true; 6],
        },
    );
    m.points.get_mut("1").unwrap().constraints.push("PL".into());
    assert!(matches!(run(m), Err(EditError::Unsupported(_))));
    let mut m = base();
    m.points
        .get_mut("1")
        .unwrap()
        .constraints
        .push("GHOST".into());
    assert!(matches!(run(m), Err(EditError::Invalid(_))));
    // a Z diaphragm is accepted and the copies stay in it, as in linear replication
    let mut m = base();
    m.constraints.insert(
        "DZ".into(),
        ConstraintDef {
            name: "DZ".into(),
            constraint_type: ConstraintType::Diaphragm,
            axis: ConstraintAxis::Z,
            dofs: [true; 6],
        },
    );
    m.points.get_mut("1").unwrap().constraints.push("DZ".into());
    m.points.get_mut("2").unwrap().constraints.push("DZ".into());
    m.replicate_radial([0.0, 0.0], 30.0, 1).unwrap();
    // 2 originals + 2 copies, all in DZ
    assert_eq!(
        m.points
            .values()
            .filter(|p| p.constraints == ["DZ"])
            .count(),
        4
    );
}

#[test]
fn radial_replication_refuses_collapsing_and_duplicate_objects() {
    // frame whose ends are 1e-10 apart: the copies of both ends merge
    let mut m = SapModel::default();
    m.add_point("1", 4.0, 0.0, 0.0);
    m.add_point("2", 4.0 + 1e-10, 0.0, 0.0);
    m.add_frame("F", "1", "2", "FSEC1");
    selected_everything(&mut m);
    let before = m.clone();
    assert!(matches!(
        m.replicate_radial([0.0, 0.0], 90.0, 1),
        Err(EditError::Invalid(_))
    ));
    assert_eq!(m, before);
    // area with two joints 1e-10 apart
    let mut m = SapModel::default();
    m.add_point("1", 4.0, 0.0, 0.0);
    m.add_point("2", 4.0 + 1e-10, 0.0, 0.0);
    m.add_point("3", 4.0, 2.0, 0.0);
    m.add_area("A", vec!["1".into(), "2".into(), "3".into()], "ASEC1");
    selected_everything(&mut m);
    let before = m.clone();
    assert!(matches!(
        m.replicate_radial([0.0, 0.0], 90.0, 1),
        Err(EditError::Invalid(_))
    ));
    assert_eq!(m, before);
    // an area already exists where the copy would go
    let mut m = SapModel::default();
    for (n, p) in [
        ("1", [4.0, 0.0, 0.0]),
        ("2", [6.0, 0.0, 0.0]),
        ("3", [4.0, 2.0, 0.0]),
    ] {
        m.add_point(n, p[0], p[1], p[2]);
    }
    for (n, p) in [
        ("a", [0.0, 4.0, 0.0]),
        ("b", [0.0, 6.0, 0.0]),
        ("c", [-2.0, 4.0, 0.0]),
    ] {
        m.add_point(n, p[0], p[1], p[2]);
    }
    m.add_area("A", vec!["1".into(), "2".into(), "3".into()], "ASEC1");
    m.add_area("E", vec!["a".into(), "b".into(), "c".into()], "ASEC1");
    for n in ["1", "2", "3"] {
        m.points.get_mut(n).unwrap().selected = true;
    }
    m.areas.get_mut("A").unwrap().selected = true;
    let before = m.clone();
    assert!(matches!(
        m.replicate_radial([0.0, 0.0], 90.0, 1),
        Err(EditError::Invalid(_))
    ));
    assert_eq!(m, before);
}

#[test]
fn bad_grid_definitions_are_refused_by_quick_draw() {
    let mut m = grid_model(3, 3, 2);
    let region = full(&m);
    // duplicate coordinates on an axis
    m.coord_systems.get_mut("GLOBAL").unwrap().grids[1].coordinate = 0.0;
    let before = m.clone();
    assert!(matches!(
        m.quick_draw_beams(region, true, true, "FSEC1"),
        Err(EditError::Invalid(_))
    ));
    assert_eq!(m, before);
    // unknown axis label
    let mut m = grid_model(3, 3, 2);
    m.coord_systems.get_mut("GLOBAL").unwrap().grids[0].axis = "W".into();
    let before = m.clone();
    assert!(matches!(
        m.quick_draw_columns(region, "FSEC1"),
        Err(EditError::Invalid(_))
    ));
    assert_eq!(m, before);
    // non-finite coordinate
    let mut m = grid_model(3, 3, 2);
    m.coord_systems.get_mut("GLOBAL").unwrap().grids[0].coordinate = f64::NAN;
    assert!(m.quick_draw_columns(region, "FSEC1").is_err());
}

/// An orthotropic material with a nonzero material angle would mirror under a reversal of
/// local 3 (the material 1 axis turns about the new local 3), so the command is refused and
/// the model is left as it was; with a zero material angle, or an isotropic material, it is
/// accepted and the section keeps its material angle.
#[test]
fn reverse_refuses_an_orthotropic_material_with_a_material_angle() {
    let quad = orientations()[1].0;
    let ortho = |angle: f64| {
        let mut m = reversible(quad, 25.0, angle);
        let sec = m.area_sections["ASEC1"].material.clone();
        let mat = m.materials.get_mut(&sec).unwrap();
        mat.e_ortho = Some([30000.0, 15000.0, 15000.0]);
        mat.u_ortho = Some([0.2, 0.2, 0.2]);
        mat.g_ortho = Some([6600.0, 6600.0, 6600.0]);
        m
    };
    let mut m = ortho(30.0);
    let before = m.clone();
    match m.reverse_area_local_3(ReverseKeep::SameGlobalLoads) {
        Err(EditError::Unsupported(msg)) => {
            assert!(msg.contains("A") && msg.contains("material angle"), "{msg}")
        }
        other => panic!("expected a refusal, got {other:?}"),
    }
    assert_eq!(m, before);
    // zero material angle: accepted, the material angle stays zero
    let mut m = ortho(0.0);
    assert_eq!(m.reverse_area_local_3(ReverseKeep::SameGlobalLoads), Ok(1));
    assert_eq!(m.area_sections["ASEC1"].mat_angle_deg, 0.0);
    // isotropic material with a material angle: accepted and the section is untouched
    let mut m = reversible(quad, 25.0, 30.0);
    assert_eq!(m.reverse_area_local_3(ReverseKeep::SameGlobalLoads), Ok(1));
    assert_eq!(m.area_sections["ASEC1"].mat_angle_deg, 30.0);
}
