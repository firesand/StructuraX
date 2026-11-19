//! Grid line editing, the 2D template on typed grid lines, divide with a
//! length ratio and break at intersections.
//!
//! Units are arbitrary (the commands are geometric). Every expected value is a
//! closed form evaluated in this file, except `equal_division_is_bitwise_unchanged`:
//! its literals were captured from `divide_selected_frames` BEFORE that command
//! was routed through the common `split_at` (a refactoring guard, not a
//! reference). Solver-based checks (beam deflections) are in
//! `crates/sap-solver/tests/grid_divide_break_solver.rs`.

use sap_model::generate::GridAxis;
use sap_model::hinge::FrameHinge;
use sap_model::{
    EditError, FrameLoadDistributed, FrameLoadPoint, FrameType2D, LinkObj, SapModel, SolidObj,
};
use std::collections::BTreeSet;

const TOL: f64 = 1e-12;

fn line(id: &str, c: f64) -> (String, f64) {
    (id.to_string(), c)
}

fn grid(m: &SapModel, axis: &str) -> Vec<(String, f64)> {
    m.coord_systems["GLOBAL"]
        .grids
        .iter()
        .filter(|g| g.axis == axis)
        .map(|g| (g.id.clone(), g.coordinate))
        .collect()
}

fn pt(m: &SapModel, name: &str) -> [f64; 3] {
    let p = &m.points[name];
    [p.x, p.y, p.z]
}

fn ends(m: &SapModel, f: &str) -> ([f64; 3], [f64; 3]) {
    (pt(m, &m.frames[f].point_i), pt(m, &m.frames[f].point_j))
}

// ---------------------------------------------------------------------------
// Equal division: captured before the refactoring
// ---------------------------------------------------------------------------

fn divisible_frame() -> SapModel {
    let mut m = SapModel::default();
    m.add_point("1", 0.1, 0.2, 0.3);
    m.add_point("2", 7.3, 2.9, 1.1);
    m.add_frame("beam", "1", "2", "FSEC1");
    let f = m.frames.get_mut("beam").unwrap();
    f.selected = true;
    f.release_i[5] = true;
    f.release_j[4] = true;
    f.start_spring[1] = 11.0;
    f.end_spring[2] = 13.0;
    f.loads_dist.push(FrameLoadDistributed {
        load_pat: "DEAD".into(),
        my_type: 1,
        dir: 6,
        csys: "GLOBAL".into(),
        rel_dist1: 0.2,
        rel_dist2: 0.9,
        val1: 3.0,
        val2: 5.5,
    });
    for (rel_dist, val) in [(0.45, 7.0), (1.0, 2.0)] {
        f.loads_point.push(FrameLoadPoint {
            load_pat: "DEAD".into(),
            my_type: 1,
            dir: 6,
            csys: "GLOBAL".into(),
            rel_dist,
            val,
        });
    }
    m
}

fn dump(m: &SapModel) -> String {
    let mut out = String::new();
    for (k, p) in &m.points {
        out += &format!("P {k} {:?} {:?} {:?}\n", p.x, p.y, p.z);
    }
    for (k, f) in &m.frames {
        out += &format!(
            "F {k} {} {} {:?} {:?} {:?} {:?} d={:?} p={:?}\n",
            f.point_i,
            f.point_j,
            f.release_i,
            f.release_j,
            f.start_spring,
            f.end_spring,
            f.loads_dist
                .iter()
                .map(|l| (l.rel_dist1, l.rel_dist2, l.val1, l.val2))
                .collect::<Vec<_>>(),
            f.loads_point
                .iter()
                .map(|l| (l.rel_dist, l.val))
                .collect::<Vec<_>>()
        );
    }
    out
}

const EQUAL_3: &str = "\
P 1 0.1 0.2 0.3
P 2 7.3 2.9 1.1
P 3 2.5 1.0999999999999999 0.5666666666666667
P 4 4.8999999999999995 1.9999999999999998 0.8333333333333333
F beam 1 3 [false, false, false, false, false, true] [false, false, false, false, false, false] [0.0, 11.0, 0.0, 0.0, 0.0, 0.0] [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] d=[(0.6000000000000001, 1.0, 3.0, 3.4761904761904763)] p=[]
F 1 3 4 [false, false, false, false, false, false] [false, false, false, false, false, false] [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] d=[(0.0, 1.0, 3.4761904761904763, 4.666666666666666)] p=[(0.3500000000000001, 7.0)]
F 2 4 2 [false, false, false, false, false, false] [false, false, false, false, true, false] [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] [0.0, 0.0, 13.0, 0.0, 0.0, 0.0] d=[(0.0, 0.7000000000000002, 4.666666666666666, 5.5)] p=[(1.0, 2.0)]
";

const EQUAL_7: &str = "\
P 1 0.1 0.2 0.3
P 2 7.3 2.9 1.1
P 3 1.1285714285714286 0.5857142857142856 0.41428571428571426
P 4 2.157142857142857 0.9714285714285713 0.5285714285714286
P 5 3.1857142857142855 1.357142857142857 0.6428571428571428
P 6 4.2142857142857135 1.7428571428571427 0.7571428571428571
P 7 5.242857142857143 2.1285714285714286 0.8714285714285714
P 8 6.2714285714285705 2.5142857142857142 0.9857142857142858
F beam 1 3 [false, false, false, false, false, true] [false, false, false, false, false, false] [0.0, 11.0, 0.0, 0.0, 0.0, 0.0] [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] d=[] p=[]
F 1 3 4 [false, false, false, false, false, false] [false, false, false, false, false, false] [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] d=[(0.40000000000000013, 1.0, 3.0, 3.3061224489795915)] p=[]
F 2 4 5 [false, false, false, false, false, false] [false, false, false, false, false, false] [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] d=[(0.0, 1.0, 3.3061224489795915, 3.816326530612245)] p=[]
F 3 5 6 [false, false, false, false, false, false] [false, false, false, false, false, false] [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] d=[(0.0, 1.0, 3.816326530612245, 4.326530612244898)] p=[(0.15000000000000024, 7.0)]
F 4 6 7 [false, false, false, false, false, false] [false, false, false, false, false, false] [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] d=[(0.0, 1.0000000000000004, 4.326530612244898, 4.836734693877551)] p=[]
F 5 7 8 [false, false, false, false, false, false] [false, false, false, false, false, false] [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] d=[(0.0, 0.9999999999999996, 4.836734693877551, 5.346938775510203)] p=[]
F 6 8 2 [false, false, false, false, false, false] [false, false, false, false, true, false] [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] [0.0, 0.0, 13.0, 0.0, 0.0, 0.0] d=[(0.0, 0.3000000000000005, 5.346938775510203, 5.5)] p=[(1.0000000000000004, 2.0)]
";

#[test]
fn equal_division_is_bitwise_unchanged() {
    for (n, expected) in [(3usize, EQUAL_3), (7, EQUAL_7)] {
        let mut m = divisible_frame();
        m.divide_selected_frames(n).unwrap();
        let (got, want) = (dump(&m), expected);
        assert_eq!(got, want, "{n} segments");
    }
}

#[test]
fn ratio_one_is_the_equal_division() {
    let mut a = divisible_frame();
    let mut b = divisible_frame();
    a.divide_selected_frames(5).unwrap();
    b.divide_selected_frames_ratio(5, 1.0).unwrap();
    assert_eq!(a, b);
}

// ---------------------------------------------------------------------------
// Grid line editing
// ---------------------------------------------------------------------------

#[test]
fn set_grid_lines_replaces_one_axis_and_keeps_the_others() {
    let mut m = SapModel::default();
    m.quick_grid([3, 2, 2], [6.0, 5.0, 4.0], [0.0; 3]).unwrap();
    let y_before = grid(&m, "Y");
    let z_before = grid(&m, "Z");
    m.coord_systems.get_mut("GLOBAL").unwrap().grids[1].visible = false;
    m.set_grid_lines(
        GridAxis::X,
        &[line("A", 0.0), line("Gate", 2.5), line("B", 10.0), line("C", 11.0)],
    )
    .unwrap();
    assert_eq!(
        grid(&m, "X"),
        vec![line("A", 0.0), line("Gate", 2.5), line("B", 10.0), line("C", 11.0)]
    );
    assert_eq!(grid(&m, "Y"), y_before);
    assert_eq!(grid(&m, "Z"), z_before);
    // The X lines stay where the old ones were: first in the list.
    assert_eq!(m.coord_systems["GLOBAL"].grids[0].axis, "X");
    // A kept id keeps its visibility; the new id gets the defaults.
    let g = &m.coord_systems["GLOBAL"].grids;
    assert!(!g.iter().find(|l| l.id == "B").unwrap().visible);
    assert!(g.iter().find(|l| l.id == "Gate").unwrap().visible);
    assert_eq!(g.iter().find(|l| l.id == "Gate").unwrap().bubble_loc, "End");
    // The order of the lines is the order given: the coordinates need not ascend.
    m.set_grid_lines(GridAxis::Z, &[line("Roof", 12.0), line("Base", 0.0)])
        .unwrap();
    assert_eq!(grid(&m, "Z"), vec![line("Roof", 12.0), line("Base", 0.0)]);
    // Z lines created on a model with none go last.
    let mut two_d = SapModel::default();
    two_d.set_grid_lines(GridAxis::Y, &[]).unwrap();
    two_d
        .set_grid_lines(GridAxis::X, &[line("1", 0.0), line("2", 3.0)])
        .unwrap();
    two_d.set_grid_lines(GridAxis::Z, &[line("Z1", 0.0)]).unwrap();
    two_d.set_grid_lines(GridAxis::X, &[]).unwrap();
    assert_eq!(grid(&two_d, "X"), vec![]);
    assert_eq!(grid(&two_d, "Z"), vec![line("Z1", 0.0)]);
}

#[test]
fn set_grid_lines_refusals_leave_the_model_unchanged() {
    let mut m = SapModel::default();
    m.quick_grid([3, 2, 2], [6.0, 5.0, 4.0], [0.0; 3]).unwrap();
    let before = m.clone();
    let many: Vec<_> = (0..1001).map(|i| line(&format!("L{i}"), i as f64)).collect();
    let cases: Vec<(&str, Vec<(String, f64)>, &str)> = vec![
        ("duplicate id", vec![line("A", 0.0), line("A", 1.0)], "twice"),
        ("blank id", vec![line("A", 0.0), line("  ", 1.0)], "no id"),
        ("same coordinate", vec![line("A", 1.0), line("B", 1.0)], "same coordinate"),
        (
            "within tolerance",
            vec![line("A", 1.0), line("B", 1.0 + 1e-12)],
            "same coordinate",
        ),
        ("nan", vec![line("A", f64::NAN)], "nonfinite"),
        ("infinite", vec![line("A", 0.0), line("B", f64::INFINITY)], "nonfinite"),
        ("too many", many, "at most"),
    ];
    for (what, lines, text) in cases {
        let error = m.set_grid_lines(GridAxis::X, &lines).unwrap_err();
        assert!(matches!(error, EditError::Invalid(_)), "{what}: {error:?}");
        assert!(error.to_string().contains(text), "{what}: {error}");
        assert_eq!(m, before, "{what}");
    }
    m.is_locked = true;
    let locked = m.clone();
    assert_eq!(
        m.set_grid_lines(GridAxis::X, &[line("A", 0.0)]),
        Err(EditError::Locked)
    );
    assert_eq!(m, locked);
}

// ---------------------------------------------------------------------------
// 2D template on typed grid lines
// ---------------------------------------------------------------------------

#[test]
fn custom_portal_has_the_geometry_of_the_pushover_frame() {
    // The frame of video_pushover_workflow.rs, retyped: column lines x = 0, 4,
    // 10, 14, floors at 3 m, four storeys.
    let xs = [0.0, 4.0, 10.0, 14.0];
    let zs = [0.0, 3.0, 6.0, 9.0, 12.0];
    let mut m = SapModel::default();
    m.generate_2d_frame_custom(FrameType2D::PortalFrame, &xs, &zs, true, "FSEC1", "FSEC1", "FSEC1", 0.0)
        .unwrap();
    assert_eq!(m.points.len(), 20);
    assert_eq!(m.frames.len(), 28);
    let fixed: Vec<_> = m.points.values().filter(|p| p.restraint == [true; 6]).collect();
    assert_eq!(fixed.len(), 4);
    assert!(fixed.iter().all(|p| p.z == 0.0));
    // Joints: every (x, 0, z).
    let joints: BTreeSet<[u64; 3]> = m
        .points
        .values()
        .map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()])
        .collect();
    let mut expected = BTreeSet::new();
    for x in xs {
        for z in zs {
            expected.insert([x.to_bits(), 0.0f64.to_bits(), z.to_bits()]);
        }
    }
    assert_eq!(joints, expected);
    // Frames: 16 columns (I below, J above) and 12 beams (I left, J right).
    let got: BTreeSet<_> = m
        .frames
        .keys()
        .map(|f| {
            let (a, b) = ends(&m, f);
            [a.map(f64::to_bits), b.map(f64::to_bits)]
        })
        .collect();
    let mut want = BTreeSet::new();
    for x in xs {
        for k in 1..zs.len() {
            want.insert([[x, 0.0, zs[k - 1]], [x, 0.0, zs[k]]].map(|p| p.map(f64::to_bits)));
        }
    }
    for k in 1..zs.len() {
        for i in 1..xs.len() {
            want.insert([[xs[i - 1], 0.0, zs[k]], [xs[i], 0.0, zs[k]]].map(|p| p.map(f64::to_bits)));
        }
    }
    assert_eq!(want.len(), 28);
    assert_eq!(got, want);
    // Grid lines are the typed ones.
    assert_eq!(
        grid(&m, "X"),
        vec![line("A", 0.0), line("B", 4.0), line("C", 10.0), line("D", 14.0)]
    );
    assert_eq!(grid(&m, "Z").len(), 5);
    assert_eq!(m.model_name, "2D PortalFrame (4 Stories, 3 Bays)");
}

fn assert_same_template(a: &SapModel, b: &SapModel, tol: f64, what: &str) {
    assert_eq!(a.points.len(), b.points.len(), "{what}: joints");
    assert_eq!(a.frames.len(), b.frames.len(), "{what}: frames");
    for (name, p) in &a.points {
        let q = &b.points[name];
        for (u, v) in [(p.x, q.x), (p.y, q.y), (p.z, q.z)] {
            assert!((u - v).abs() <= tol * v.abs().max(1.0), "{what}: joint {name}");
        }
        assert_eq!(p.restraint, q.restraint, "{what}: joint {name}");
    }
    for (name, f) in &a.frames {
        let g = &b.frames[name];
        assert_eq!((&f.point_i, &f.point_j, &f.section), (&g.point_i, &g.point_j, &g.section), "{what}: frame {name}");
        assert_eq!((f.release_i, f.release_j), (g.release_i, g.release_j), "{what}: frame {name}");
    }
    assert_eq!(a.model_name, b.model_name, "{what}");
    assert_eq!(
        a.coord_systems["GLOBAL"].grids.len(),
        b.coord_systems["GLOBAL"].grids.len(),
        "{what}: grid"
    );
}

#[test]
fn equally_spaced_custom_lines_equal_the_equal_bay_template() {
    let xs = [-360.0, -120.0, 120.0, 360.0];
    let zs = [0.0, 144.0, 288.0, 432.0];
    for restraint in [true, false] {
        let mut reference = SapModel::default();
        reference.generate_2d_frame(FrameType2D::PortalFrame, 3, 144.0, 3, 240.0, restraint, "FSEC1", "FSEC1", "FSEC1", 0.0);
        let mut custom = SapModel::default();
        custom
            .generate_2d_frame_custom(FrameType2D::PortalFrame, &xs, &zs, restraint, "FSEC1", "FSEC1", "FSEC1", 0.0)
            .unwrap();
        // Bitwise: same joints, frames and grid lines, `model_name` included.
        assert_eq!(custom.points, reference.points);
        assert_eq!(custom.frames, reference.frames);
        assert_eq!(custom.coord_systems["GLOBAL"].grids, reference.coord_systems["GLOBAL"].grids);
        assert_eq!(custom.model_name, reference.model_name);
    }
    // Sections that are not defined fall back as in the equal-bay template.
    let mut reference = SapModel::default();
    reference.generate_2d_frame(FrameType2D::PortalFrame, 3, 144.0, 3, 240.0, true, "W18X50", "W14X90", "W14X90", 0.0);
    let mut custom = SapModel::default();
    custom
        .generate_2d_frame_custom(FrameType2D::PortalFrame, &xs, &zs, true, "W18X50", "W14X90", "W14X90", 0.0)
        .unwrap();
    assert_eq!(custom.frames, reference.frames);
}

#[test]
fn grid_line_ids_follow_the_equal_bay_template_and_stay_unique_beyond_26_lines() {
    // 26 bays: 27 X lines. The equal-bay template wraps around the alphabet
    // (its 27th X line is "A" again); the custom template continues with "AA"
    // so that no id repeats.
    let xs: Vec<f64> = (0..27).map(|i| 3.0 * f64::from(i)).collect();
    let zs = [0.0, 4.0];
    let mut reference = SapModel::default();
    reference.generate_2d_frame(FrameType2D::PortalFrame, 1, 4.0, 26, 3.0, true, "FSEC1", "FSEC1", "FSEC1", 0.0);
    let mut custom = SapModel::default();
    custom
        .generate_2d_frame_custom(FrameType2D::PortalFrame, &xs, &zs, true, "FSEC1", "FSEC1", "FSEC1", 0.0)
        .unwrap();
    let ids = |m: &SapModel| -> Vec<String> {
        m.coord_systems["GLOBAL"].grids.iter().map(|g| g.id.clone()).collect()
    };
    assert_eq!(ids(&reference)[26], "A");
    assert_eq!(ids(&custom)[26], "AA");
    assert_eq!(ids(&custom)[..26], ids(&reference)[..26]);
    assert_eq!(ids(&custom)[27..], ids(&reference)[27..]);
    let distinct: std::collections::BTreeSet<String> = ids(&custom).into_iter().collect();
    assert_eq!(distinct.len(), ids(&custom).len());
    // The start of the equal-bay template is at -39, so only the spacing and
    // the ids are compared, not the coordinates.
    let spacing = |m: &SapModel| -> Vec<u64> {
        let x: Vec<f64> = m.coord_systems["GLOBAL"].grids.iter().filter(|g| g.axis == "X").map(|g| g.coordinate).collect();
        x.windows(2).map(|w| (w[1] - w[0]).to_bits()).collect()
    };
    assert_eq!(spacing(&custom), spacing(&reference));
    assert_eq!(custom.frames.len(), reference.frames.len());
}

#[test]
fn braced_custom_lines_equal_the_equal_bay_template_to_1e_12() {
    let xs = [-360.0, -120.0, 120.0, 360.0];
    let zs = [0.0, 144.0, 288.0, 432.0];
    for (kind, ecc) in [
        (FrameType2D::ConcentricBraced, 0.0),
        (FrameType2D::EccentricBraced, 30.0),
        (FrameType2D::EccentricBraced, 1.0),
        (FrameType2D::EccentricBraced, 500.0),
    ] {
        let mut reference = SapModel::default();
        reference.generate_2d_frame(kind, 3, 144.0, 3, 240.0, true, "FSEC1", "FSEC1", "FSEC1", ecc);
        let mut custom = SapModel::default();
        custom
            .generate_2d_frame_custom(kind, &xs, &zs, true, "FSEC1", "FSEC1", "FSEC1", ecc)
            .unwrap();
        assert_same_template(&custom, &reference, 1e-12, &format!("{kind:?} e = {ecc}"));
    }
}

#[test]
fn braced_custom_bays_use_their_own_midpoint_and_link_length() {
    // Bays of 4 and 6 and an eccentricity of 0.5: the link is clamp(0.5, 0.1 b,
    // 0.4 b) long, i.e. 0.5 in the first bay (0.4..1.6) and 0.6 in the second
    // (0.6..2.4); it is centered on the bay.
    let mut m = SapModel::default();
    m.generate_2d_frame_custom(FrameType2D::EccentricBraced, &[0.0, 4.0, 10.0], &[0.0, 3.0], true, "FSEC1", "FSEC1", "FSEC1", 0.5)
        .unwrap();
    let on_beam: Vec<f64> = {
        let mut xs: Vec<f64> = m.points.values().filter(|p| p.z == 3.0).map(|p| p.x).collect();
        xs.sort_by(f64::total_cmp);
        xs
    };
    let want = [0.0, 1.75, 2.25, 4.0, 6.7, 7.3, 10.0];
    assert_eq!(on_beam.len(), want.len());
    for (g, w) in on_beam.iter().zip(want) {
        assert!((g - w).abs() <= TOL * 10.0, "{on_beam:?}");
    }
    // Concentric: the brace joint is the middle of each bay (2 and 7).
    let mut m = SapModel::default();
    m.generate_2d_frame_custom(FrameType2D::ConcentricBraced, &[0.0, 4.0, 10.0], &[0.0, 3.0], false, "FSEC1", "FSEC1", "FSEC1", 0.0)
        .unwrap();
    let mid: Vec<f64> = m.points.values().filter(|p| p.z == 3.0 && p.x != 0.0 && p.x != 4.0 && p.x != 10.0).map(|p| p.x).collect();
    assert_eq!(mid.len(), 2);
    assert!(mid.contains(&2.0) && mid.contains(&7.0), "{mid:?}");
}

#[test]
fn custom_template_refusals_leave_the_model_unchanged() {
    let mut m = SapModel::default();
    m.generate_beam(2, 5.0, true, "FSEC1");
    m.points.get_mut("1").unwrap().selected = true;
    let before = m.clone();
    let good = [0.0, 4.0];
    let cases: Vec<(&str, Vec<f64>, Vec<f64>, f64, &str)> = vec![
        ("one X line", vec![0.0], good.to_vec(), 0.0, "at least two X"),
        ("no Z line", good.to_vec(), vec![], 0.0, "at least two Z"),
        ("equal X lines", vec![0.0, 0.0, 4.0], good.to_vec(), 0.0, "increase strictly"),
        ("decreasing Z", good.to_vec(), vec![3.0, 0.0], 0.0, "increase strictly"),
        ("nan X", vec![0.0, f64::NAN], good.to_vec(), 0.0, "finite"),
        ("infinite Z", good.to_vec(), vec![0.0, f64::INFINITY], 0.0, "finite"),
        ("nan eccentricity", good.to_vec(), good.to_vec(), f64::NAN, "eccentricity"),
        ("too many lines", (0..1001).map(f64::from).collect(), good.to_vec(), 0.0, "at most 1000"),
    ];
    for (what, x, z, e, text) in cases {
        let error = m
            .generate_2d_frame_custom(FrameType2D::EccentricBraced, &x, &z, true, "FSEC1", "FSEC1", "FSEC1", e)
            .unwrap_err();
        assert!(error.to_string().contains(text), "{what}: {error}");
        assert_eq!(m, before, "{what}");
    }
}

// ---------------------------------------------------------------------------
// Divide with a length ratio
// ---------------------------------------------------------------------------

fn beam_along_x(length: f64) -> SapModel {
    let mut m = SapModel::default();
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", length, 0.0, 0.0);
    m.add_frame("1", "1", "2", "FSEC1");
    m.frames.get_mut("1").unwrap().selected = true;
    m
}

fn xs_of(m: &SapModel) -> Vec<f64> {
    let mut xs: Vec<f64> = m.points.values().map(|p| p.x).collect();
    xs.sort_by(f64::total_cmp);
    xs
}

#[test]
fn ratio_division_makes_a_geometric_series() {
    // L = 10, n = 4, r = 8: q = 8^(1/3) = 2, l1 = 10 (2 - 1) / (2^4 - 1) = 2/3,
    // lengths 2/3, 4/3, 8/3, 16/3 and joints at 2/3, 2, 14/3.
    let mut m = beam_along_x(10.0);
    let summary = m.divide_selected_frames_ratio(4, 8.0).unwrap();
    assert_eq!(summary.frames_created.len(), 3);
    assert_eq!(summary.points_created.len(), 3);
    let xs = xs_of(&m);
    let want = [0.0, 2.0 / 3.0, 2.0, 14.0 / 3.0, 10.0];
    for (g, w) in xs.iter().zip(want) {
        assert!((g - w).abs() <= 1e-12 * 10.0, "{xs:?}");
    }
    // The chain runs from the I end to the J end and keeps the name "1" first.
    let mut at = 0.0;
    let mut name = "1".to_string();
    let mut lengths = Vec::new();
    loop {
        let (a, b) = ends(&m, &name);
        assert!((a[0] - at).abs() < 1e-12);
        lengths.push(b[0] - a[0]);
        at = b[0];
        match m.frames.values().find(|f| f.point_i == m.frames[&name].point_j) {
            Some(next) => name = next.name.clone(),
            None => break,
        }
    }
    assert_eq!(lengths.len(), 4);
    for (g, w) in lengths.iter().zip([2.0 / 3.0, 4.0 / 3.0, 8.0 / 3.0, 16.0 / 3.0]) {
        assert!((g - w).abs() <= 1e-12 * 10.0, "{lengths:?}");
    }
    assert!((lengths[3] / lengths[0] - 8.0).abs() < 1e-12);
}

#[test]
fn ratio_below_one_shortens_toward_the_j_end() {
    // r = 1/8: lengths 16/3, 8/3, 4/3, 2/3.
    let mut m = beam_along_x(10.0);
    m.divide_selected_frames_ratio(4, 0.125).unwrap();
    let xs = xs_of(&m);
    let want = [0.0, 16.0 / 3.0, 8.0, 28.0 / 3.0, 10.0];
    for (g, w) in xs.iter().zip(want) {
        assert!((g - w).abs() <= 1e-12 * 10.0, "{xs:?}");
    }
    // Three pieces with r = 4 (q = 2): l1 = L / 7 and joints at 1/7, 3/7.
    let mut m = beam_along_x(7.0);
    m.divide_selected_frames_ratio(3, 4.0).unwrap();
    let xs = xs_of(&m);
    for (g, w) in xs.iter().zip([0.0, 1.0, 3.0, 7.0]) {
        assert!((g - w).abs() <= 1e-12 * 7.0, "{xs:?}");
    }
}

#[test]
fn ratio_division_distributes_loads_like_the_equal_division() {
    // A uniform load over the whole frame stays uniform on every piece, a point
    // load at 0.3 falls on exactly one piece at the matching relative distance.
    let mut m = beam_along_x(10.0);
    let f = m.frames.get_mut("1").unwrap();
    f.loads_dist.push(FrameLoadDistributed {
        load_pat: "D".into(),
        my_type: 1,
        dir: 6,
        csys: "GLOBAL".into(),
        rel_dist1: 0.0,
        rel_dist2: 1.0,
        val1: 2.0,
        val2: 2.0,
    });
    f.loads_point.push(FrameLoadPoint {
        load_pat: "D".into(),
        my_type: 1,
        dir: 6,
        csys: "GLOBAL".into(),
        rel_dist: 0.3,
        val: 5.0,
    });
    m.divide_selected_frames_ratio(4, 8.0).unwrap();
    let total: f64 = m
        .frames
        .values()
        .flat_map(|f| {
            let (a, b) = ends(&m, &f.name);
            let l = b[0] - a[0];
            f.loads_dist.iter().map(move |d| 0.5 * (d.val1 + d.val2) * (d.rel_dist2 - d.rel_dist1) * l)
        })
        .sum();
    assert!((total - 20.0).abs() < 1e-12, "{total}");
    let points: Vec<_> = m.frames.values().filter(|f| !f.loads_point.is_empty()).collect();
    assert_eq!(points.len(), 1);
    // 0.3 L = 3 lies in the third piece, 2 < 3 < 14/3: rel = (3 - 2) / (8/3).
    let (a, _) = ends(&m, &points[0].name);
    assert!((a[0] - 2.0).abs() < 1e-12);
    assert!((points[0].loads_point[0].rel_dist - 3.0 / 8.0).abs() < 1e-12);
    for f in m.frames.values() {
        for l in &f.loads_dist {
            assert!((0.0..=1.0).contains(&l.rel_dist1) && (0.0..=1.0).contains(&l.rel_dist2));
        }
    }
}

#[test]
fn ratio_division_refusals_leave_the_model_unchanged() {
    let mut m = beam_along_x(10.0);
    let before = m.clone();
    for (what, n, r) in [
        ("one segment", 1usize, 2.0),
        ("too many", 10_001, 2.0),
        ("zero ratio", 4, 0.0),
        ("negative ratio", 4, -2.0),
        ("nan ratio", 4, f64::NAN),
        ("infinite ratio", 4, f64::INFINITY),
        ("unrepresentable ratio", 4, 1e300),
        ("segments shorter than the tolerance", 50, 1e12),
    ] {
        assert!(matches!(m.divide_selected_frames_ratio(n, r), Err(EditError::Invalid(_))), "{what}");
        assert_eq!(m, before, "{what}");
    }
    // Hinged frames are refused as the equal division refuses them.
    let mut hinged = beam_along_x(10.0);
    hinged.frames.get_mut("1").unwrap().hinges.push(FrameHinge { prop: "H".into(), rel_dist: 0.5 });
    let before = hinged.clone();
    let e = hinged.divide_selected_frames_ratio(4, 2.0).unwrap_err();
    assert!(matches!(e, EditError::Unsupported(_)) && e.to_string().contains("frame 1"), "{e}");
    assert_eq!(hinged, before);
    // The other refusals of the equal division apply to the ratio division.
    let setups: Vec<(&str, Box<dyn Fn(&mut SapModel)>)> = vec![
        ("end offset", Box::new(|m| m.frames.get_mut("1").unwrap().end_offset_i = 0.5)),
        ("cardinal point", Box::new(|m| m.frames.get_mut("1").unwrap().cardinal_point = 1)),
        ("limits", Box::new(|m| m.frames.get_mut("1").unwrap().tc_limits = Some((true, 0.0, false, 0.0)))),
        (
            "cable",
            Box::new(|m| {
                m.frames.get_mut("1").unwrap().cable_params = Some(sap_model::CableParams {
                    undeformed_length: Some(10.0),
                    target_tension_i: None,
                    horizontal_tension: None,
                    num_segments: 4,
                })
            }),
        ),
        (
            "non-prismatic section",
            Box::new(|m| m.frame_sections.get_mut("FSEC1").unwrap().prop_type = sap_model::FramePropType::Variable),
        ),
    ];
    for (what, setup) in setups {
        let mut m = beam_along_x(10.0);
        setup(&mut m);
        let before = m.clone();
        assert!(matches!(m.divide_selected_frames_ratio(4, 2.0), Err(EditError::Unsupported(_))), "{what}");
        assert_eq!(m, before, "{what}");
    }
    // Nothing selected, and a locked model.
    let mut none = beam_along_x(10.0);
    none.frames.get_mut("1").unwrap().selected = false;
    let before = none.clone();
    assert_eq!(none.divide_selected_frames_ratio(4, 2.0), Err(EditError::EmptySelection));
    assert_eq!(none, before);
    let mut locked = beam_along_x(10.0);
    locked.is_locked = true;
    let before = locked.clone();
    assert_eq!(locked.divide_selected_frames_ratio(4, 2.0), Err(EditError::Locked));
    assert_eq!(locked, before);
}

// ---------------------------------------------------------------------------
// Break at intersections
// ---------------------------------------------------------------------------

/// Beams (0,0,0)-(10,0,0) named "1" and (5,-5,0)-(5,5,0) named "2", selected.
fn crossing() -> SapModel {
    let mut m = SapModel::default();
    for (n, c) in [("1", [0.0, 0.0, 0.0]), ("2", [10.0, 0.0, 0.0]), ("3", [5.0, -5.0, 0.0]), ("4", [5.0, 5.0, 0.0])] {
        m.add_point(n, c[0], c[1], c[2]);
    }
    m.add_frame("1", "1", "2", "FSEC1");
    m.add_frame("2", "3", "4", "FSEC1");
    for f in m.frames.values_mut() {
        f.selected = true;
    }
    m
}

#[test]
fn crossing_frames_split_both_at_one_shared_joint() {
    let mut m = crossing();
    let summary = m.break_selected_frames_at_intersections().unwrap();
    assert_eq!(summary.points_created, vec!["5".to_string()]);
    assert_eq!(summary.frames_created.len(), 2);
    assert_eq!(m.points.len(), 5);
    assert_eq!(m.frames.len(), 4);
    assert_eq!(pt(&m, "5"), [5.0, 0.0, 0.0]);
    for f in m.frames.values() {
        assert!(f.point_i == "5" || f.point_j == "5", "{f:?}");
    }
    // The first piece keeps the name; the pieces run end to end.
    assert_eq!((m.frames["1"].point_i.as_str(), m.frames["1"].point_j.as_str()), ("1", "5"));
    assert_eq!((m.frames["2"].point_i.as_str(), m.frames["2"].point_j.as_str()), ("3", "5"));
    let others: BTreeSet<_> = m
        .frames
        .values()
        .filter(|f| f.name != "1" && f.name != "2")
        .map(|f| (f.point_i.clone(), f.point_j.clone()))
        .collect();
    assert_eq!(others, BTreeSet::from([("5".to_string(), "2".to_string()), ("5".to_string(), "4".to_string())]));
}

#[test]
fn loads_of_a_broken_frame_follow_the_split() {
    let mut m = crossing();
    m.frames.get_mut("1").unwrap().loads_point.push(FrameLoadPoint {
        load_pat: "P".into(),
        my_type: 1,
        dir: 6,
        csys: "GLOBAL".into(),
        rel_dist: 0.5,
        val: 10.0,
    });
    m.break_selected_frames_at_intersections().unwrap();
    // The load at the split goes to the right piece, at its start.
    let loaded: Vec<_> = m.frames.values().filter(|f| !f.loads_point.is_empty()).collect();
    assert_eq!(loaded.len(), 1);
    assert_eq!(loaded[0].point_i, "5");
    assert_eq!(loaded[0].loads_point[0].rel_dist, 0.0);
}

#[test]
fn a_t_junction_breaks_only_the_frame_that_is_crossed() {
    // Beam 2 ends on the interior of beam 1.
    let mut m = crossing();
    m.points.get_mut("3").unwrap().y = 0.0;
    m.points.get_mut("3").unwrap().x = 5.0;
    m.points.get_mut("4").unwrap().y = 5.0;
    let summary = m.break_selected_frames_at_intersections().unwrap();
    assert!(summary.points_created.is_empty(), "the end joint of beam 2 is the break joint");
    assert_eq!(summary.frames_created.len(), 1);
    assert_eq!(m.frames["2"].point_i, "3");
    assert_eq!(m.frames["2"].point_j, "4");
    assert_eq!((m.frames["1"].point_i.as_str(), m.frames["1"].point_j.as_str()), ("1", "3"));
    let other = m.frames.values().find(|f| f.name != "1" && f.name != "2").unwrap();
    assert_eq!((other.point_i.as_str(), other.point_j.as_str()), ("3", "2"));
}

#[test]
fn a_selected_joint_on_the_span_splits_the_frame() {
    let mut m = SapModel::default();
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 12.0, 0.0, 0.0);
    m.add_point("3", 4.0, 0.0, 0.0);
    m.add_point("4", 8.0, 1e-12, 0.0); // within the tolerance of the span
    m.add_point("5", 6.0, 0.5, 0.0); // off the span
    m.add_frame("1", "1", "2", "FSEC1");
    m.frames.get_mut("1").unwrap().selected = true;
    for j in ["3", "4", "5"] {
        m.points.get_mut(j).unwrap().selected = true;
    }
    let summary = m.break_selected_frames_at_intersections().unwrap();
    assert!(summary.points_created.is_empty());
    assert_eq!(m.frames.len(), 3);
    let chain: Vec<_> = {
        let mut v: Vec<_> = m.frames.values().map(|f| (f.point_i.clone(), f.point_j.clone())).collect();
        v.sort();
        v
    };
    assert_eq!(
        chain,
        vec![
            ("1".to_string(), "3".to_string()),
            ("3".to_string(), "4".to_string()),
            ("4".to_string(), "2".to_string())
        ]
    );
    // An end joint, selected, does not split.
    let mut m = SapModel::default();
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 12.0, 0.0, 0.0);
    m.add_frame("1", "1", "2", "FSEC1");
    m.frames.get_mut("1").unwrap().selected = true;
    m.points.get_mut("1").unwrap().selected = true;
    let before = m.clone();
    assert!(matches!(m.break_selected_frames_at_intersections(), Err(EditError::Invalid(_))));
    assert_eq!(m, before);
}

#[test]
fn only_selected_frames_and_real_intersections_break() {
    // An unselected crossing frame does not break the selected one; a frame
    // that passes the other at distance 1 (skew) does not either.
    let mut m = crossing();
    m.frames.get_mut("2").unwrap().selected = false;
    let before = m.clone();
    assert!(matches!(m.break_selected_frames_at_intersections(), Err(EditError::Invalid(_))));
    assert_eq!(m, before);
    let mut skew = crossing();
    for n in ["3", "4"] {
        skew.points.get_mut(n).unwrap().z = 1.0;
    }
    let before = skew.clone();
    assert!(matches!(skew.break_selected_frames_at_intersections(), Err(EditError::Invalid(_))));
    assert_eq!(skew, before);
    // A skew crossing within the tolerance does break.
    let mut close = crossing();
    for n in ["3", "4"] {
        close.points.get_mut(n).unwrap().z = 1e-12;
    }
    close.break_selected_frames_at_intersections().unwrap();
    assert_eq!(close.frames.len(), 4);
}

#[test]
fn three_frames_through_one_point_share_one_joint() {
    let mut m = crossing();
    m.add_point("5", 0.0, -5.0, 0.0);
    m.add_point("6", 10.0, 5.0, 0.0);
    m.add_frame("3", "5", "6", "FSEC1");
    m.frames.get_mut("3").unwrap().selected = true;
    let summary = m.break_selected_frames_at_intersections().unwrap();
    assert_eq!(summary.points_created.len(), 1);
    assert_eq!(summary.frames_created.len(), 3);
    assert_eq!(m.frames.len(), 6);
    let center = &summary.points_created[0];
    assert_eq!(m.frames.values().filter(|f| &f.point_i == center || &f.point_j == center).count(), 6);
}

#[test]
fn break_refusals_leave_the_model_unchanged() {
    let hinged = |m: &mut SapModel| {
        m.frames.get_mut("1").unwrap().hinges.push(FrameHinge { prop: "H".into(), rel_dist: 0.5 });
    };
    let cases: Vec<(&str, Box<dyn Fn(&mut SapModel)>, &str)> = vec![
        ("hinged frame", Box::new(hinged), "hinge"),
        (
            "offset frame",
            Box::new(|m| m.frames.get_mut("2").unwrap().end_offset_i = 0.5),
            "offsets",
        ),
        (
            "joint offset",
            Box::new(|m| m.frames.get_mut("1").unwrap().joint_offset_j = [0.0, 0.0, 0.1]),
            "offsets",
        ),
        (
            "cable",
            Box::new(|m| {
                m.frames.get_mut("1").unwrap().cable_params = Some(sap_model::CableParams {
                    undeformed_length: Some(10.0),
                    target_tension_i: None,
                    horizontal_tension: None,
                    num_segments: 4,
                })
            }),
            "cable",
        ),
        (
            "tension/compression limits",
            Box::new(|m| m.frames.get_mut("1").unwrap().tc_limits = Some((true, 0.0, false, 0.0))),
            "limits",
        ),
        (
            "cardinal point",
            Box::new(|m| m.frames.get_mut("2").unwrap().cardinal_point = 1),
            "cardinal point",
        ),
        (
            "non-prismatic section",
            Box::new(|m| {
                m.frame_sections.get_mut("FSEC1").unwrap().prop_type = sap_model::FramePropType::Variable
            }),
            "nonprismatic",
        ),
        (
            "selected area",
            Box::new(|m| {
                m.add_point("5", 0.0, 5.0, 0.0);
                m.add_area("A", vec!["1".into(), "2".into(), "4".into(), "5".into()], "ASEC1");
                m.areas.get_mut("A").unwrap().selected = true;
            }),
            "area or solid edges is not implemented",
        ),
        (
            "selected solid",
            Box::new(|m| {
                let p = std::array::from_fn(|i| ["1", "2", "3", "4"][i % 4].to_string());
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
            "area or solid edges is not implemented",
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
            "links",
        ),
        (
            "missing section",
            Box::new(|m| m.frames.get_mut("1").unwrap().section = "NOPE".into()),
            "NOPE",
        ),
        ("locked", Box::new(|m| m.is_locked = true), "locked"),
        (
            "collinear overlap",
            Box::new(|m| {
                m.add_point("5", 3.0, 0.0, 0.0);
                m.add_point("6", 8.0, 0.0, 0.0);
                m.add_frame("3", "5", "6", "FSEC1");
                m.frames.get_mut("3").unwrap().selected = true;
            }),
            "overlap",
        ),
        (
            "nothing selected",
            Box::new(|m| m.frames.values_mut().for_each(|f| f.selected = false)),
            "no supported objects",
        ),
        (
            "a piece would duplicate a frame",
            Box::new(|m| {
                // An unselected frame already joins joint 1 to the crossing point.
                m.add_point("5", 5.0, 0.0, 0.0);
                m.add_frame("3", "1", "5", "FSEC1");
            }),
            "duplicate",
        ),
    ];
    for (what, setup, text) in cases {
        let mut m = crossing();
        setup(&mut m);
        let before = m.clone();
        let error = m.break_selected_frames_at_intersections().unwrap_err();
        assert!(error.to_string().contains(text), "{what}: {error}");
        assert_eq!(m, before, "{what}");
    }
}

#[test]
fn break_clears_stale_results_only_when_it_succeeds() {
    use sap_model::CaseResults;
    let mut m = crossing();
    m.case_results.insert("OLD".into(), CaseResults::default());
    m.is_locked = false;
    let mut broken = m.clone();
    broken.break_selected_frames_at_intersections().unwrap();
    assert!(broken.case_results.is_empty());
    let mut refused = m.clone();
    refused.frames.values_mut().for_each(|f| f.selected = false);
    let _ = refused.break_selected_frames_at_intersections();
    assert!(!refused.case_results.is_empty());
}
