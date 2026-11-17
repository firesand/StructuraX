//! P16, D56: a frame with an end offset has its output stations from the
//! support face on; the deformed shape and the moment diagram follow.

use super::wave5_windows_tests::{flat, Run};
use super::*;
use eframe::egui::{epaint::Shape, Color32, Pos2};
use sap_model::PointLoadForce;

const GREEN: Color32 = Color32::from_rgb(80, 255, 160);

/// A cantilever along X, 4 m, fixed at joint 1 with a rigid offset of 0.5 m
/// at that end, loaded by 10 kN downward at the tip.
fn cantilever() -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 4.0, 0.0, 0.0);
    m.points["1"].restraint = [true; 6];
    m.add_frame("F", "1", "2", "FSEC1");
    let f = &mut m.frames["F"];
    f.end_offset_i = 0.5;
    f.rigid_zone_factor = 1.0;
    m.points["2"].loads_force.push(PointLoadForce {
        load_pat: "DEAD".into(),
        csys: "GLOBAL".into(),
        force: [0.0, 0.0, -10.0, 0.0, 0.0, 0.0],
    });
    sap_solver::analyze_model(&mut m).expect("analysis");
    m
}

fn segments(run: &Run, color: Color32) -> Vec<[Pos2; 2]> {
    flat(&run.last)
        .into_iter()
        .filter_map(|s| match s {
            Shape::LineSegment { points, stroke } if stroke.color == eframe::epaint::ColorMode::Solid(color) => Some(points),
            _ => None,
        })
        .collect()
}

#[test]
fn the_deformed_shape_runs_from_the_joint_through_the_face_to_the_tip() {
    let m = cantilever();
    let stations = &m.case_results["DEAD"].frame_forces["F"];
    // P13: the first station is at the support face, 0.5 m from joint I.
    assert!((stations[0].station - 0.5).abs() < 1e-9, "{}", stations[0].station);
    let mut run = Run::new(m);
    run.app.left_proj = ViewProjection::ElevationXZ;
    run.app.display_mode = DisplayMode::Deformed;
    run.app.active_case = "DEAD".into();
    run.settle();
    let scale = f64::from(run.app.deform_scale);
    let tip = run.app.model.case_results["DEAD"].joint_displacements["2"];
    let drawn_tip = run.screen([4.0 + scale * tip[0], 0.0, scale * tip[2]]);
    let joint = run.screen([0.0, 0.0, 0.0]);
    let face = run.screen([0.5, 0.0, 0.0]);
    let segs = segments(&run, GREEN);
    let near = |a: Pos2, b: Pos2| (a - b).length() < 0.6;
    // The rigid arm: from the (fixed, undisplaced) joint to the face.
    assert!(segs.iter().any(|s| near(s[0], joint) && near(s[1], face)), "{segs:?}");
    // The curve ends at the displaced tip, which is below the joint's level.
    assert!(segs.iter().any(|s| near(s[1], drawn_tip)), "{segs:?}");
    assert!(drawn_tip.y > joint.y + 5.0, "the tip moved down on screen");
}

#[test]
fn the_moment_diagram_starts_at_the_face_with_nothing_inside_the_offset() {
    let m = cantilever();
    let mut run = Run::new(m);
    run.app.left_proj = ViewProjection::ElevationXZ;
    run.app.display_mode = DisplayMode::FrameDiagram(FrameForceComponent::MomentM3);
    run.app.active_case = "DEAD".into();
    run.settle();
    let joint = run.screen([0.0, 0.0, 0.0]);
    let face = run.screen([0.5, 0.0, 0.0]);
    // The ordinates are drawn from points of the member axis: none lies
    // between the joint and the face.
    let from_axis: Vec<Pos2> = flat(&run.last)
        .into_iter()
        .filter_map(|s| match s {
            Shape::LineSegment { points, stroke } if (stroke.width - 1.2).abs() < 1e-3 => Some(points[0]),
            _ => None,
        })
        .collect();
    assert!(!from_axis.is_empty());
    let leftmost = from_axis.iter().map(|p| p.x).fold(f32::INFINITY, f32::min);
    assert!((leftmost - face.x).abs() < 0.6, "starts at {leftmost}, face at {}", face.x);
    assert!(leftmost > joint.x + 5.0);
}
