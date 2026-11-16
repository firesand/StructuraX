//! Drawing at coordinates: `draw_frames`, `draw_areas`, `check_area_corners`.
//!
//! Units kN, m. Every refusal must leave the model equal to its state before.

use sap_model::draw::check_area_corners;
use sap_model::{EditError, SapModel};
use sap_units::Units;

fn model() -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    m.quick_grid([3, 3, 2], [4.0, 5.0, 3.0], [0.0; 3]).unwrap();
    m
}

fn unchanged_after_refusal<T: std::fmt::Debug>(
    m: &mut SapModel,
    f: impl FnOnce(&mut SapModel) -> Result<T, EditError>,
) -> EditError {
    let before = m.clone();
    let err = f(m).expect_err("the command must be refused");
    assert_eq!(*m, before, "a refused command must not change the model");
    err
}

#[test]
fn frames_run_from_the_first_point_and_reuse_joints() {
    let mut m = model();
    let s = m
        .draw_frames(
            &[
                ([0.0, 0.0, 0.0], [4.0, 0.0, 0.0]),
                ([4.0, 0.0, 0.0], [4.0, 0.0, 3.0]),
            ],
            "FSEC1",
            false,
        )
        .unwrap();
    assert_eq!(s.frames_created.len(), 2);
    assert_eq!(s.points_created.len(), 3, "the shared joint is reused");
    let f = &m.frames[&s.frames_created[0]];
    let (i, j) = (&m.points[&f.point_i], &m.points[&f.point_j]);
    assert_eq!([i.x, i.y, i.z], [0.0, 0.0, 0.0]);
    assert_eq!([j.x, j.y, j.z], [4.0, 0.0, 0.0]);
    assert_eq!(f.release_i, [false; 6]);
    // A point within the merge tolerance reuses the existing joint.
    let n = m.points.len();
    m.draw_frames(&[([4.0 + 1e-12, 0.0, 3.0], [8.0, 0.0, 3.0])], "FSEC1", false)
        .unwrap();
    assert_eq!(m.points.len(), n + 1);
}

#[test]
fn pinned_frames_release_m2_and_m3_at_both_ends_only() {
    let mut m = model();
    let s = m
        .draw_frames(&[([0.0, 0.0, 0.0], [0.0, 0.0, 3.0])], "FSEC1", true)
        .unwrap();
    let f = &m.frames[&s.frames_created[0]];
    for r in [f.release_i, f.release_j] {
        assert_eq!(r, [false, false, false, false, true, true]);
    }
}

#[test]
fn drawing_clears_stale_results_like_the_quick_draw_commands() {
    let mut m = model();
    m.case_results.insert("DEAD".into(), Default::default());
    m.draw_frames(&[([0.0, 0.0, 0.0], [4.0, 0.0, 0.0])], "FSEC1", false)
        .unwrap();
    assert!(m.case_results.is_empty());
    m.case_results.insert("DEAD".into(), Default::default());
    m.draw_areas(
        &[vec![
            [0.0, 0.0, 0.0],
            [4.0, 0.0, 0.0],
            [4.0, 5.0, 0.0],
            [0.0, 5.0, 0.0],
        ]],
        "ASEC1",
    )
    .unwrap();
    assert!(m.case_results.is_empty());
}

#[test]
fn frame_refusals_leave_the_model_unchanged() {
    let seg = ([0.0, 0.0, 0.0], [4.0, 0.0, 0.0]);
    // Locked model.
    let mut m = model();
    m.is_locked = true;
    let e = unchanged_after_refusal(&mut m, |m| m.draw_frames(&[seg], "FSEC1", false));
    assert_eq!(e, EditError::Locked);
    // Missing section.
    let mut m = model();
    let e = unchanged_after_refusal(&mut m, |m| m.draw_frames(&[seg], "NOPE", false));
    assert!(e.to_string().contains("NOPE"), "{e}");
    // Zero length, also after joint reuse (the second end merges with the first).
    let e = unchanged_after_refusal(&mut m, |m| {
        m.draw_frames(
            &[([1.0, 1.0, 1.0], [1.0, 1.0, 1.0 + 1e-13])],
            "FSEC1",
            false,
        )
    });
    assert!(e.to_string().contains("zero length"), "{e}");
    // A valid segment followed by a bad one creates nothing.
    unchanged_after_refusal(&mut m, |m| {
        m.draw_frames(&[seg, ([2.0, 2.0, 2.0], [2.0, 2.0, 2.0])], "FSEC1", false)
    });
    // Nothing created: the frame exists.
    m.draw_frames(&[seg], "FSEC1", false).unwrap();
    let e = unchanged_after_refusal(&mut m, |m| m.draw_frames(&[(seg.1, seg.0)], "FSEC1", false));
    assert!(e.to_string().contains("creates no objects"), "{e}");
    // Nonfinite coordinates and an empty list.
    unchanged_after_refusal(&mut m, |m| {
        m.draw_frames(&[([f64::NAN, 0.0, 0.0], [1.0, 0.0, 0.0])], "FSEC1", false)
    });
    unchanged_after_refusal(&mut m, |m| m.draw_frames(&[], "FSEC1", false));
}

#[test]
fn an_area_keeps_the_given_corner_order() {
    let mut m = model();
    let corners = vec![
        [0.0, 0.0, 3.0],
        [4.0, 0.0, 3.0],
        [4.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
    ];
    let s = m.draw_areas(std::slice::from_ref(&corners), "ASEC1").unwrap();
    let a = &m.areas[&s.areas_created[0]];
    let drawn: Vec<[f64; 3]> = a
        .points
        .iter()
        .map(|p| [m.points[p].x, m.points[p].y, m.points[p].z])
        .collect();
    assert_eq!(drawn, corners);
    // A triangle is accepted.
    m.draw_areas(
        &[vec![[0.0, 5.0, 0.0], [4.0, 5.0, 0.0], [4.0, 10.0, 0.0]]],
        "ASEC1",
    )
    .unwrap();
}

#[test]
fn area_refusals_leave_the_model_unchanged() {
    let sq = vec![
        [0.0, 0.0, 0.0],
        [4.0, 0.0, 0.0],
        [4.0, 5.0, 0.0],
        [0.0, 5.0, 0.0],
    ];
    let mut m = model();
    m.is_locked = true;
    let e = unchanged_after_refusal(&mut m, |m| m.draw_areas(std::slice::from_ref(&sq), "ASEC1"));
    assert_eq!(e, EditError::Locked);

    let mut m = model();
    let e = unchanged_after_refusal(&mut m, |m| m.draw_areas(std::slice::from_ref(&sq), "NOPE"));
    assert!(e.to_string().contains("NOPE"), "{e}");

    // Five corners, two corners.
    let five = vec![
        [0.0, 0.0, 0.0],
        [4.0, 0.0, 0.0],
        [6.0, 2.0, 0.0],
        [4.0, 5.0, 0.0],
        [0.0, 5.0, 0.0],
    ];
    let e = unchanged_after_refusal(&mut m, |m| m.draw_areas(std::slice::from_ref(&five), "ASEC1"));
    assert!(e.to_string().contains("3 or 4"), "{e}");
    unchanged_after_refusal(&mut m, |m| m.draw_areas(&[sq[..2].to_vec()], "ASEC1"));

    // A repeated joint after reuse.
    let mut repeated = sq.clone();
    repeated[3] = [4.0 + 1e-12, 0.0, 0.0];
    let e = unchanged_after_refusal(&mut m, |m| m.draw_areas(std::slice::from_ref(&repeated), "ASEC1"));
    assert!(e.to_string().contains("repeats joint"), "{e}");

    // Folded (bow-tie) and degenerate (collinear) quadrilaterals.
    let bowtie = vec![
        [0.0, 0.0, 0.0],
        [4.0, 5.0, 0.0],
        [4.0, 0.0, 0.0],
        [0.0, 5.0, 0.0],
    ];
    unchanged_after_refusal(&mut m, |m| m.draw_areas(std::slice::from_ref(&bowtie), "ASEC1"));
    let line = vec![
        [0.0, 0.0, 0.0],
        [4.0, 0.0, 0.0],
        [8.0, 0.0, 0.0],
        [12.0, 0.0, 0.0],
    ];
    unchanged_after_refusal(&mut m, |m| m.draw_areas(std::slice::from_ref(&line), "ASEC1"));

    // An existing area with the same joint set, in any order, and a duplicate
    // inside one command.
    m.draw_areas(std::slice::from_ref(&sq), "ASEC1").unwrap();
    let mut shuffled = sq.clone();
    shuffled.reverse();
    let e = unchanged_after_refusal(&mut m, |m| m.draw_areas(std::slice::from_ref(&shuffled), "ASEC1"));
    assert!(e.to_string().contains("existing area"), "{e}");
    let other = vec![
        [4.0, 0.0, 0.0],
        [8.0, 0.0, 0.0],
        [8.0, 5.0, 0.0],
        [4.0, 5.0, 0.0],
    ];
    unchanged_after_refusal(&mut m, |m| {
        m.draw_areas(&[other.clone(), other.clone()], "ASEC1")
    });
    unchanged_after_refusal(&mut m, |m| m.draw_areas(&[], "ASEC1"));
}

#[test]
fn corner_check_follows_the_convexity_and_projection_rules() {
    let ok = |c: &[[f64; 3]]| check_area_corners(c).is_ok();
    // Triangle and rectangles in three planes.
    assert!(ok(&[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]));
    assert!(ok(&[
        [0.0, 0.0, 0.0],
        [4.0, 0.0, 0.0],
        [4.0, 0.0, 3.0],
        [0.0, 0.0, 3.0]
    ]));
    assert!(ok(&[
        [0.0, 0.0, 0.0],
        [0.0, 4.0, 0.0],
        [0.0, 4.0, 3.0],
        [0.0, 0.0, 3.0]
    ]));
    // Clockwise order is accepted: the sign only has to be uniform.
    assert!(ok(&[
        [0.0, 0.0, 0.0],
        [0.0, 5.0, 0.0],
        [4.0, 5.0, 0.0],
        [4.0, 0.0, 0.0]
    ]));
    // A slightly warped quadrilateral is projected on its mean plane.
    assert!(ok(&[
        [0.0, 0.0, 0.0],
        [4.0, 0.0, 0.1],
        [4.0, 5.0, 0.0],
        [0.0, 5.0, 0.1]
    ]));
    // Concave (dart), bow-tie, repeated point, collinear, all equal, nonfinite.
    assert!(!ok(&[
        [0.0, 0.0, 0.0],
        [4.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 4.0, 0.0]
    ]));
    assert!(!ok(&[
        [0.0, 0.0, 0.0],
        [4.0, 5.0, 0.0],
        [4.0, 0.0, 0.0],
        [0.0, 5.0, 0.0]
    ]));
    assert!(!ok(&[
        [0.0, 0.0, 0.0],
        [4.0, 0.0, 0.0],
        [4.0, 0.0, 0.0],
        [0.0, 5.0, 0.0]
    ]));
    assert!(!ok(&[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]));
    assert!(!ok(&[[1.0; 3], [1.0; 3], [1.0; 3]]));
    assert!(!ok(&[
        [0.0, 0.0, 0.0],
        [f64::NAN, 0.0, 0.0],
        [0.0, 1.0, 0.0]
    ]));
    // Counts.
    assert!(!ok(&[[0.0; 3], [1.0, 0.0, 0.0]]));
    assert!(!ok(&[
        [0.0, 0.0, 0.0],
        [4.0, 0.0, 0.0],
        [6.0, 2.0, 0.0],
        [4.0, 5.0, 0.0],
        [0.0, 5.0, 0.0]
    ]));
}
