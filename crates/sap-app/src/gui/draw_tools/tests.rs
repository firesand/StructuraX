//! Tests of the drawing tools. The view is fixed: a plan with its centre at
//! (15, 7.5, 9.6), 20 px per metre and the screen centre at (500, 400), over
//! the grid of 6 x 4 x 7 lines at 6, 5 and 3.2 m. Grid lines then fall at
//! screen x = 200, 320, 440, 560, 680, 800 and y = 550, 450, 350, 250 in the
//! plan; in an elevation z = 0 is at y = 592 and z = 3.2 at y = 528.
//! Units kN, m.

use super::super::view::PlaneAxis;
use super::super::SapGuiApp;
use super::*;
use eframe::egui::Vec2;
use sap_units::Units;
use std::path::PathBuf;

mod review;

fn grid_model() -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    m.quick_grid([6, 4, 7], [6.0, 5.0, 3.2], [0.0; 3]).unwrap();
    m
}

fn plan() -> ViewTransform {
    ViewTransform {
        proj: ViewProjection::PlanXY,
        center: [15.0, 7.5, 9.6],
        scale: 20.0,
        screen_center: Pos2::new(500.0, 400.0),
        yaw: 0.0,
        pitch: 0.0,
    }
}

fn elevation(proj: ViewProjection) -> ViewTransform {
    ViewTransform { proj, ..plan() }
}

/// The plane of `proj` at grid line `index` of its normal axis (the stored value).
fn plane_at(m: &SapModel, proj: ViewProjection, index: usize) -> PlaneView {
    let normal = PlaneView::of(proj, Some(0.0)).unwrap().normal;
    let c = grid_axes(m).unwrap()[normal.index()][index];
    PlaneView::of(proj, Some(c)).unwrap()
}

fn click(x: f32, y: f32) -> PointerEvent {
    PointerEvent::Click {
        pos: Pos2::new(x, y),
        double: false,
    }
}

fn window(x0: f32, y0: f32, x1: f32, y1: f32) -> PointerEvent {
    PointerEvent::Window {
        from: Pos2::new(x0, y0),
        to: Pos2::new(x1, y1),
    }
}

fn applied(o: &DrawOutcome) -> &Result<String, String> {
    match o {
        DrawOutcome::Applied(r) => r,
        other => panic!("expected an applied outcome, got {other:?}"),
    }
}

fn corners_of(m: &SapModel, area: &str) -> Vec<[f64; 3]> {
    m.areas[area]
        .points
        .iter()
        .map(|p| [m.points[p].x, m.points[p].y, m.points[p].z])
        .collect()
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

// ---- pure resolution with the fixed view -----------------------------------

#[test]
fn a_click_picks_the_nearest_grid_segment_from_the_lower_to_the_higher_end() {
    let m = grid_model();
    let z2 = grid_axes(&m).unwrap()[2][2];
    let plane = plane_at(&m, ViewProjection::PlanXY, 2);
    let mut st = State::default();
    // 2 px from the X segment (6, 5)-(12, 5), about 60 px from the nearest Y one.
    let r = resolve(&mut st, ToolMode::QuickFrame, &plan(), Some(plane), click(380.0, 452.0), &m);
    assert_eq!(
        r,
        Resolution::Command {
            command: DrawCommand::Frames {
                segments: vec![([6.0, 5.0, z2], [12.0, 5.0, z2])]
            },
            then: Then::Keep
        }
    );
    // A Y segment: x = 12 (screen 440), between y = 5 and 10 (screen 450, 350).
    let r = resolve(&mut st, ToolMode::QuickFrame, &plan(), Some(plane), click(443.0, 400.0), &m);
    assert_eq!(
        r,
        Resolution::Command {
            command: DrawCommand::Frames {
                segments: vec![([12.0, 5.0, z2], [12.0, 10.0, z2])]
            },
            then: Then::Keep
        }
    );
    // Equidistant: on the intersection (6, 5) the X segment to the left wins
    // (lower axis, then lower grid index).
    let r = resolve(&mut st, ToolMode::QuickFrame, &plan(), Some(plane), click(320.0, 450.0), &m);
    assert_eq!(
        r,
        Resolution::Command {
            command: DrawCommand::Frames {
                segments: vec![([0.0, 5.0, z2], [6.0, 5.0, z2])]
            },
            then: Then::Keep
        }
    );
    // Farther than 8 px from every segment: nothing to draw.
    let r = resolve(&mut st, ToolMode::QuickFrame, &plan(), Some(plane), click(380.0, 400.0), &m);
    assert!(matches!(r, Resolution::Outcome(DrawOutcome::Status(_))), "{r:?}");
}

#[test]
fn a_window_takes_every_segment_with_both_ends_inside_in_either_direction() {
    let mut m = grid_model();
    let plane = plane_at(&m, ViewProjection::PlanXY, 2);
    let mut left_to_right = State::default();
    let a = resolve(
        &mut left_to_right,
        ToolMode::QuickFrame,
        &plan(),
        Some(plane),
        window(300.0, 300.0, 720.0, 520.0),
        &m,
    );
    let Resolution::Command {
        command: DrawCommand::Frames { segments },
        ..
    } = a.clone()
    else {
        panic!("{a:?}");
    };
    // X lines 6..24 (4 lines, 3 bays) on the 2 Y lines 5 and 10, plus 4 Y segments.
    assert_eq!(segments.len(), 3 * 2 + 4);
    let b = resolve(
        &mut State::default(),
        ToolMode::QuickFrame,
        &plan(),
        Some(plane),
        window(720.0, 520.0, 300.0, 300.0),
        &m,
    );
    assert_eq!(a, b);
    let out = on_event(
        &mut State::default(),
        ToolMode::QuickFrame,
        &plan(),
        Some(plane),
        window(300.0, 300.0, 720.0, 520.0),
        &mut m,
    );
    assert!(applied(&out).is_ok(), "{out:?}");
    assert_eq!(m.frames.len(), 10);
    // Nothing fully inside: a hint, no change.
    let before = m.clone();
    let out = on_event(
        &mut State::default(),
        ToolMode::QuickFrame,
        &plan(),
        Some(plane),
        window(330.0, 460.0, 430.0, 440.0),
        &mut m,
    );
    assert!(matches!(out, DrawOutcome::Status(_)), "{out:?}");
    assert_eq!(m, before);
}

#[test]
fn quick_area_resolves_to_the_cell_under_the_click() {
    let m = grid_model();
    let z2 = grid_axes(&m).unwrap()[2][2];
    let plane = plane_at(&m, ViewProjection::PlanXY, 2);
    let r = resolve(&mut State::default(), ToolMode::QuickArea, &plan(), Some(plane), click(380.0, 400.0), &m);
    assert_eq!(
        r,
        Resolution::Command {
            command: DrawCommand::Areas {
                corner_sets: vec![vec![
                    [6.0, 5.0, z2],
                    [12.0, 5.0, z2],
                    [12.0, 10.0, z2],
                    [6.0, 10.0, z2]
                ]]
            },
            then: Then::Keep
        }
    );
    // Outside the grid.
    let r = resolve(&mut State::default(), ToolMode::QuickArea, &plan(), Some(plane), click(100.0, 100.0), &m);
    assert!(matches!(r, Resolution::Outcome(DrawOutcome::Status(_))), "{r:?}");
}

#[test]
fn a_click_in_an_elevation_picks_the_column_between_grid_lines() {
    let m = grid_model();
    let plane = plane_at(&m, ViewProjection::ElevationXZ, 1);
    assert_eq!(plane.coordinate, 5.0);
    let view = elevation(ViewProjection::ElevationXZ);
    // z = 0 is at 592, z = 3.2 at 528, x = 6 at 320.
    assert_eq!(view.project([6.0, 5.0, 0.0]), Pos2::new(320.0, 592.0));
    assert_eq!(view.project([6.0, 5.0, 3.2]), Pos2::new(320.0, 528.0));
    let r = resolve(&mut State::default(), ToolMode::QuickFrame, &view, Some(plane), click(322.0, 560.0), &m);
    assert_eq!(
        r,
        Resolution::Command {
            command: DrawCommand::Frames {
                segments: vec![([6.0, 5.0, 0.0], [6.0, 5.0, 3.2])]
            },
            then: Then::Keep
        }
    );
}

#[test]
fn a_rectangle_in_an_elevation_faces_the_viewer() {
    let mut m = grid_model();
    let plane = plane_at(&m, ViewProjection::ElevationXZ, 0);
    assert_eq!(plane.coordinate, 0.0);
    let view = elevation(ViewProjection::ElevationXZ);
    let mut st = State::default();
    let first = on_event(&mut st, ToolMode::RectArea, &view, Some(plane), click(200.0, 592.0), &mut m);
    assert!(matches!(first, DrawOutcome::Status(_)), "{first:?}");
    let out = on_event(&mut st, ToolMode::RectArea, &view, Some(plane), click(320.0, 528.0), &mut m);
    assert!(applied(&out).is_ok(), "{out:?}");
    let a = m.areas.keys().next().unwrap().clone();
    let c = corners_of(&m, &a);
    assert_eq!(
        c,
        vec![[0.0, 0.0, 0.0], [6.0, 0.0, 0.0], [6.0, 0.0, 3.2], [0.0, 0.0, 3.2]]
    );
    // (6, 0, 0) x (0, 0, 3.2) = (0, -19.2, 0): local 3 is -Y, toward a viewer at -Y.
    let n = cross(sub(c[1], c[0]), sub(c[3], c[0]));
    assert!(n[0] == 0.0 && n[2] == 0.0 && (n[1] + 19.2).abs() < 1e-12, "{n:?}");
    // The Y-Z elevation faces +X, the plan +Z.
    let yz = elevation(ViewProjection::ElevationYZ);
    let plane = plane_at(&m, ViewProjection::ElevationYZ, 0);
    let out = on_event(&mut State::default(), ToolMode::QuickArea, &yz, Some(plane), click(380.0, 570.0), &mut m);
    assert!(applied(&out).is_ok(), "{out:?}");
    let c = corners_of(&m, "2");
    assert!(cross(sub(c[1], c[0]), sub(c[3], c[0]))[0] > 0.0, "{c:?}");
}

#[test]
fn a_free_point_is_unprojected_and_a_snapped_point_takes_the_stored_value() {
    let mut m = grid_model();
    let z2 = grid_axes(&m).unwrap()[2][2];
    let plane = plane_at(&m, ViewProjection::PlanXY, 2);
    // Free: 390, 450 is 50 px from the nearest intersection; exact in binary.
    assert_eq!(
        pick_point(&m, &plan(), Some(plane), Pos2::new(390.0, 450.0)),
        Ok([9.5, 5.0, z2])
    );
    // Snapped to the grid intersection (6, 5): 1 px off, stored value returned.
    assert_eq!(
        pick_point(&m, &plan(), Some(plane), Pos2::new(321.0, 451.0)),
        Ok([6.0, 5.0, z2])
    );
    // 11 px off: free again.
    let far = pick_point(&m, &plan(), Some(plane), Pos2::new(331.0, 450.0)).unwrap();
    assert_ne!(far, [6.0, 5.0, z2]);
    // A joint of the plane wins over the grid; a joint of another plane is ignored.
    m.add_point("J", 6.3, 5.2, z2);
    m.add_point("K", 9.0, 2.5, 0.0);
    assert_eq!(
        pick_point(&m, &plan(), Some(plane), Pos2::new(329.0, 446.0)),
        Ok([6.3, 5.2, z2])
    );
    // K would project at (380, 500).
    assert_eq!(plan().project([9.0, 2.5, 0.0]), Pos2::new(380.0, 500.0));
    assert_eq!(
        pick_point(&m, &plan(), Some(plane), Pos2::new(380.0, 500.0)),
        Ok([9.0, 2.5, z2])
    );
}

#[test]
fn without_a_plane_only_joints_snap_and_quick_tools_are_refused() {
    let mut m = grid_model();
    m.add_point("A", 3.0, 4.0, 5.0);
    let persp = ViewTransform {
        proj: ViewProjection::Perspective3D,
        yaw: 0.4,
        pitch: 0.5,
        ..plan()
    };
    let s = persp.project([3.0, 4.0, 5.0]);
    assert_eq!(
        pick_point(&m, &persp, None, s + Vec2::new(3.0, -2.0)),
        Ok([3.0, 4.0, 5.0])
    );
    assert!(pick_point(&m, &persp, None, s + Vec2::new(40.0, 40.0)).is_err());
    // The whole-model 2D view has no plane either.
    assert!(pick_point(&m, &plan(), None, Pos2::new(390.0, 450.0)).is_err());
    for tool in [
        ToolMode::QuickFrame,
        ToolMode::QuickArea,
        ToolMode::QuickBrace,
        ToolMode::QuickSecondary,
        ToolMode::RectArea,
    ] {
        let before = m.clone();
        let out = on_event(&mut State::default(), tool, &persp, None, click(500.0, 400.0), &mut m);
        let err = applied(&out).clone().unwrap_err();
        assert!(err.contains("2D plane view"), "{tool:?}: {err}");
        let out = on_event(&mut State::default(), tool, &plan(), None, window(0.0, 0.0, 999.0, 999.0), &mut m);
        assert!(applied(&out).is_err(), "{tool:?}");
        assert_eq!(m, before);
    }
}

// ---- identity with the index commands --------------------------------------

#[test]
fn a_plan_window_equals_quick_draw_beams_at_that_level() {
    let mut drawn = grid_model();
    let mut reference = grid_model();
    let plane = plane_at(&drawn, ViewProjection::PlanXY, 2);
    let out = on_event(
        &mut State::default(),
        ToolMode::QuickFrame,
        &plan(),
        Some(plane),
        window(150.0, 200.0, 850.0, 600.0),
        &mut drawn,
    );
    assert!(applied(&out).is_ok(), "{out:?}");
    reference
        .quick_draw_beams(
            GridRegion {
                x: (0, 5),
                y: (0, 3),
                z: (2, 2),
            },
            true,
            true,
            "FSEC1",
        )
        .unwrap();
    assert_eq!(drawn, reference);
    assert_eq!(drawn.frames.len(), 5 * 4 + 6 * 3);
}

#[test]
fn an_elevation_window_equals_columns_plus_the_beams_along_its_axis() {
    // The window takes every segment with both ends inside, so the segments
    // on the lowest Z line are included: columns plus beams along X.
    let mut drawn = grid_model();
    let mut reference = grid_model();
    let plane = plane_at(&drawn, ViewProjection::ElevationXZ, 1);
    let view = elevation(ViewProjection::ElevationXZ);
    let out = on_event(
        &mut State::default(),
        ToolMode::QuickFrame,
        &view,
        Some(plane),
        window(150.0, 150.0, 850.0, 650.0),
        &mut drawn,
    );
    assert!(applied(&out).is_ok(), "{out:?}");
    let region = GridRegion {
        x: (0, 5),
        y: (1, 1),
        z: (0, 6),
    };
    reference.quick_draw_beams(region, true, false, "FSEC1").unwrap();
    reference.quick_draw_columns(region, "FSEC1").unwrap();
    assert_eq!(drawn, reference);
    assert_eq!(drawn.frames.len(), 5 * 7 + 6 * 6);
    // Drawing the same window again creates nothing and changes nothing.
    let before = drawn.clone();
    let out = on_event(
        &mut State::default(),
        ToolMode::QuickFrame,
        &view,
        Some(plane),
        window(150.0, 150.0, 850.0, 650.0),
        &mut drawn,
    );
    assert!(applied(&out).is_err());
    assert_eq!(drawn, before);
}

#[test]
fn a_plan_window_of_areas_equals_quick_draw_areas_and_skips_existing_cells() {
    let mut drawn = grid_model();
    let mut reference = grid_model();
    let plane = plane_at(&drawn, ViewProjection::PlanXY, 2);
    // One cell first, then the whole plan: the existing cell is skipped, as
    // `quick_draw_areas` skips an existing area.
    let one = on_event(&mut State::default(), ToolMode::QuickArea, &plan(), Some(plane), click(380.0, 400.0), &mut drawn);
    assert!(applied(&one).is_ok());
    let out = on_event(
        &mut State::default(),
        ToolMode::QuickArea,
        &plan(),
        Some(plane),
        window(150.0, 200.0, 850.0, 600.0),
        &mut drawn,
    );
    assert!(applied(&out).is_ok(), "{out:?}");
    let region = GridRegion {
        x: (1, 2),
        y: (1, 2),
        z: (2, 2),
    };
    reference.quick_draw_areas(region, "ASEC1").unwrap();
    assert_eq!(drawn.areas.len(), 15);
    assert_eq!(drawn.areas["1"], reference.areas["1"]);
    // Every cell once: same corner sets as `quick_draw_areas` over the plan.
    let mut full = grid_model();
    full.quick_draw_areas(
        GridRegion {
            x: (0, 5),
            y: (0, 3),
            z: (2, 2),
        },
        "ASEC1",
    )
    .unwrap();
    let key = |m: &SapModel| -> std::collections::BTreeSet<Vec<String>> {
        m.areas
            .keys()
            .map(|a| {
                corners_of(m, a)
                    .iter()
                    .map(|c| format!("{c:?}"))
                    .collect::<Vec<_>>()
            })
            .collect()
    };
    assert_eq!(key(&drawn), key(&full));
    // A click on a cell that has its area is refused and changes nothing.
    let before = drawn.clone();
    let out = on_event(&mut State::default(), ToolMode::QuickArea, &plan(), Some(plane), click(380.0, 400.0), &mut drawn);
    assert!(applied(&out).is_err(), "{out:?}");
    assert_eq!(drawn, before);
}

#[test]
fn braces_and_secondary_beams_equal_the_index_commands() {
    // Braces in the X-Z elevation at y = 5: cell x 12..18, z 9.6..12.8.
    let base = |m: &mut SapModel| {
        let region = GridRegion {
            x: (0, 5),
            y: (1, 1),
            z: (0, 6),
        };
        m.quick_draw_beams(region, true, false, "FSEC1").unwrap();
        m.quick_draw_columns(region, "FSEC1").unwrap();
    };
    let mut drawn = grid_model();
    let mut reference = grid_model();
    base(&mut drawn);
    base(&mut reference);
    let plane = plane_at(&drawn, ViewProjection::ElevationXZ, 1);
    let view = elevation(ViewProjection::ElevationXZ);
    let mut st = State {
        brace_kind: BraceKind::InvertedV,
        ..State::default()
    };
    let out = on_event(&mut st, ToolMode::QuickBrace, &view, Some(plane), click(500.0, 370.0), &mut drawn);
    assert!(applied(&out).is_ok(), "{out:?}");
    reference
        .quick_draw_braces(
            GridRegion {
                x: (2, 3),
                y: (1, 1),
                z: (3, 4),
            },
            VerticalPlane::XZ,
            BraceKind::InvertedV,
            "FSEC1",
        )
        .unwrap();
    assert_eq!(drawn, reference);
    // A window over two storeys of two bays: x 12..24 (440..680), z 3.2..9.6.
    let out = on_event(&mut st, ToolMode::QuickBrace, &view, Some(plane), window(430.0, 300.0, 690.0, 560.0), &mut drawn);
    assert!(applied(&out).is_ok(), "{out:?}");
    reference
        .quick_draw_braces(
            GridRegion {
                x: (2, 4),
                y: (1, 1),
                z: (1, 4),
            },
            VerticalPlane::XZ,
            BraceKind::InvertedV,
            "FSEC1",
        )
        .unwrap();
    assert_eq!(drawn, reference);
    // Braces in a plan are refused.
    let plan_plane = plane_at(&drawn, ViewProjection::PlanXY, 2);
    let before = drawn.clone();
    let out = on_event(&mut st, ToolMode::QuickBrace, &plan(), Some(plan_plane), click(380.0, 400.0), &mut drawn);
    assert!(applied(&out).is_err());
    assert_eq!(drawn, before);

    // Secondary beams in the plan at z2: cell x 6..12, y 5..10.
    let girders = |m: &mut SapModel| {
        m.quick_draw_beams(
            GridRegion {
                x: (0, 5),
                y: (0, 3),
                z: (2, 2),
            },
            true,
            true,
            "FSEC1",
        )
        .unwrap();
    };
    let mut drawn = grid_model();
    let mut reference = grid_model();
    girders(&mut drawn);
    girders(&mut reference);
    let mut st = State {
        span: SpanDirection::Y,
        secondary_count: 2,
        ..State::default()
    };
    let out = on_event(&mut st, ToolMode::QuickSecondary, &plan(), Some(plan_plane), click(380.0, 400.0), &mut drawn);
    assert!(applied(&out).is_ok(), "{out:?}");
    reference
        .quick_draw_secondary_beams(
            GridRegion {
                x: (1, 2),
                y: (1, 2),
                z: (2, 2),
            },
            SpanDirection::Y,
            2,
            "FSEC1",
        )
        .unwrap();
    assert_eq!(drawn, reference);
    // In an elevation they are refused.
    let before = drawn.clone();
    let out = on_event(&mut st, ToolMode::QuickSecondary, &view, Some(plane), click(500.0, 370.0), &mut drawn);
    assert!(applied(&out).is_err());
    assert_eq!(drawn, before);
}

#[test]
fn the_properties_choose_the_section_and_the_pinned_ends() {
    let mut m = grid_model();
    let mut sec = m.frame_sections["FSEC1"].clone();
    sec.name = "FSEC2".into();
    m.frame_sections.insert("FSEC2".into(), sec);
    let plane = plane_at(&m, ViewProjection::PlanXY, 2);
    let mut st = State {
        frame_section: "FSEC2".into(),
        pinned: true,
        ..State::default()
    };
    let out = on_event(&mut st, ToolMode::QuickFrame, &plan(), Some(plane), click(380.0, 452.0), &mut m);
    assert!(applied(&out).is_ok(), "{out:?}");
    let f = m.frames.values().next().unwrap();
    assert_eq!(f.section, "FSEC2");
    assert_eq!(f.release_i, [false, false, false, false, true, true]);
    assert_eq!(f.release_j, [false, false, false, false, true, true]);
}

// ---- polyline tools ---------------------------------------------------------

#[test]
fn polyframe_adds_a_frame_per_click_and_ends_on_the_same_point_enter_or_double_click() {
    let mut m = grid_model();
    let plane = plane_at(&m, ViewProjection::PlanXY, 2);
    let mut st = State::default();
    let go = |st: &mut State, m: &mut SapModel, e| on_event(st, ToolMode::PolyFrame, &plan(), Some(plane), e, m);
    assert!(matches!(go(&mut st, &mut m, click(200.0, 550.0)), DrawOutcome::Status(_)));
    assert!(m.frames.is_empty());
    assert!(applied(&go(&mut st, &mut m, click(320.0, 550.0))).is_ok());
    assert!(applied(&go(&mut st, &mut m, click(320.0, 450.0))).is_ok());
    assert_eq!(m.frames.len(), 2);
    // End I is the first picked point.
    let f = &m.frames["1"];
    assert_eq!((m.points[&f.point_i].x, m.points[&f.point_j].x), (0.0, 6.0));
    // A click on the same point ends the sequence; the next click starts anew.
    assert!(matches!(go(&mut st, &mut m, click(320.0, 450.0)), DrawOutcome::Status(_)));
    assert!(matches!(go(&mut st, &mut m, click(440.0, 450.0)), DrawOutcome::Status(_)));
    assert_eq!(m.frames.len(), 2);
    // Enter ends it, too.
    assert!(matches!(go(&mut st, &mut m, PointerEvent::Enter), DrawOutcome::Status(_)));
    assert!(matches!(go(&mut st, &mut m, click(560.0, 450.0)), DrawOutcome::Status(_)));
    assert_eq!(m.frames.len(), 2, "no frame from the first point of a sequence");
    // A double click ends without adding a frame.
    let dbl = PointerEvent::Click {
        pos: Pos2::new(560.0, 350.0),
        double: true,
    };
    assert!(matches!(go(&mut st, &mut m, dbl), DrawOutcome::Status(_)));
    assert_eq!(m.frames.len(), 2);
    // Escape leaves the tool and drops the pending point.
    assert!(matches!(go(&mut st, &mut m, click(560.0, 450.0)), DrawOutcome::Status(_)));
    assert_eq!(go(&mut st, &mut m, PointerEvent::Escape), DrawOutcome::Finish);
    assert!(matches!(go(&mut st, &mut m, click(680.0, 450.0)), DrawOutcome::Status(_)));
    assert_eq!(m.frames.len(), 2);
}

#[test]
fn a_refused_polyframe_segment_keeps_the_pending_point_and_the_model() {
    let mut m = grid_model();
    let plane = plane_at(&m, ViewProjection::PlanXY, 2);
    let mut st = State::default();
    on_event(&mut st, ToolMode::PolyFrame, &plan(), Some(plane), click(200.0, 550.0), &mut m);
    st.frame_section = "NOPE".into();
    let before = m.clone();
    let out = on_event(&mut st, ToolMode::PolyFrame, &plan(), Some(plane), click(320.0, 550.0), &mut m);
    assert!(applied(&out).clone().unwrap_err().contains("NOPE"), "{out:?}");
    assert_eq!(m, before);
    st.frame_section = "FSEC1".into();
    let out = on_event(&mut st, ToolMode::PolyFrame, &plan(), Some(plane), click(320.0, 550.0), &mut m);
    assert!(applied(&out).is_ok(), "the first point was kept: {out:?}");
    assert_eq!(m.frames.len(), 1);
    // The picked points belong to a tool and a view: another one drops them.
    on_event(&mut st, ToolMode::PolyFrame, &plan(), Some(plane), click(440.0, 550.0), &mut m);
    assert_eq!(m.frames.len(), 2);
    let out = on_event(&mut st, ToolMode::PolyArea, &plan(), Some(plane), PointerEvent::Enter, &mut m);
    assert!(applied(&out).clone().unwrap_err().contains("at least 3"), "{out:?}");
}

#[test]
fn a_zero_length_frame_is_refused_by_the_model_through_the_tool() {
    let mut m = grid_model();
    let before = m.clone();
    let p = [3.0, 3.0, 3.0];
    let cmd = DrawCommand::Frames {
        segments: vec![(p, p)],
    };
    let err = apply(&State::default(), &cmd, &mut m).unwrap_err();
    assert!(err.contains("zero length"), "{err}");
    assert_eq!(m, before);
    // A locked model is refused; so is a model without any frame section.
    m.is_locked = true;
    let cmd = DrawCommand::Frames {
        segments: vec![([0.0; 3], [1.0, 0.0, 0.0])],
    };
    let before = m.clone();
    assert!(apply(&State::default(), &cmd, &mut m).unwrap_err().contains("locked"));
    m.is_locked = false;
    m.frame_sections.clear();
    let before2 = m.clone();
    assert!(apply(&State::default(), &cmd, &mut m).unwrap_err().contains("frame section"));
    assert_eq!(m, before2);
    assert_ne!(before, before2);
}

#[test]
fn polyarea_closes_on_the_first_point_enter_or_a_double_click() {
    let mut m = grid_model();
    let z2 = grid_axes(&m).unwrap()[2][2];
    let plane = plane_at(&m, ViewProjection::PlanXY, 2);
    let mut st = State::default();
    let go = |st: &mut State, m: &mut SapModel, e| on_event(st, ToolMode::PolyArea, &plan(), Some(plane), e, m);
    // Enter with four points.
    for (x, y) in [(200.0, 550.0), (320.0, 550.0), (320.0, 450.0), (200.0, 450.0)] {
        assert!(matches!(go(&mut st, &mut m, click(x, y)), DrawOutcome::Status(_)));
    }
    assert!(applied(&go(&mut st, &mut m, PointerEvent::Enter)).is_ok());
    assert_eq!(
        corners_of(&m, "1"),
        vec![[0.0, 0.0, z2], [6.0, 0.0, z2], [6.0, 5.0, z2], [0.0, 5.0, z2]]
    );
    // A triangle closed by a click on its first point.
    for (x, y) in [(320.0, 550.0), (440.0, 550.0), (440.0, 450.0)] {
        go(&mut st, &mut m, click(x, y));
    }
    assert!(applied(&go(&mut st, &mut m, click(320.0, 550.0))).is_ok());
    assert_eq!(m.areas.len(), 2);
    // Closed by a double click: the click that opens the double already added the point.
    for (x, y) in [(440.0, 450.0), (560.0, 450.0), (560.0, 350.0)] {
        go(&mut st, &mut m, click(x, y));
    }
    let dbl = PointerEvent::Click {
        pos: Pos2::new(560.0, 350.0),
        double: true,
    };
    assert!(applied(&go(&mut st, &mut m, dbl)).is_ok());
    assert_eq!(m.areas.len(), 3);
}

#[test]
fn polyarea_refusals_leave_the_model_unchanged() {
    let mut m = grid_model();
    let plane = plane_at(&m, ViewProjection::PlanXY, 2);
    let go = |st: &mut State, m: &mut SapModel, e| on_event(st, ToolMode::PolyArea, &plan(), Some(plane), e, m);
    let pick = |st: &mut State, m: &mut SapModel, pts: &[(f32, f32)]| {
        for (x, y) in pts {
            go(st, m, click(*x, *y));
        }
    };
    // Fewer than three points.
    let mut st = State::default();
    pick(&mut st, &mut m, &[(200.0, 550.0), (320.0, 550.0)]);
    let before = m.clone();
    let out = go(&mut st, &mut m, PointerEvent::Enter);
    assert!(applied(&out).clone().unwrap_err().contains("at least 3"), "{out:?}");
    assert_eq!(m, before);
    // A fifth point.
    pick(
        &mut st,
        &mut m,
        &[(200.0, 550.0), (320.0, 550.0), (440.0, 450.0), (320.0, 350.0)],
    );
    let out = go(&mut st, &mut m, click(200.0, 350.0));
    assert!(applied(&out).clone().unwrap_err().contains("at most 4"), "{out:?}");
    assert_eq!(m, before);
    go(&mut st, &mut m, PointerEvent::Escape);
    // A folded (bow-tie) quadrilateral.
    pick(
        &mut st,
        &mut m,
        &[(200.0, 550.0), (320.0, 450.0), (320.0, 550.0), (200.0, 450.0)],
    );
    let out = go(&mut st, &mut m, PointerEvent::Enter);
    assert!(applied(&out).clone().unwrap_err().contains("folded"), "{out:?}");
    assert_eq!(m, before);
    // An existing area.
    pick(
        &mut st,
        &mut m,
        &[(200.0, 550.0), (320.0, 550.0), (320.0, 450.0), (200.0, 450.0)],
    );
    assert!(applied(&go(&mut st, &mut m, PointerEvent::Enter)).is_ok());
    let with_area = m.clone();
    pick(
        &mut st,
        &mut m,
        &[(320.0, 450.0), (200.0, 450.0), (200.0, 550.0), (320.0, 550.0)],
    );
    let out = go(&mut st, &mut m, PointerEvent::Enter);
    assert!(applied(&out).clone().unwrap_err().contains("existing area"), "{out:?}");
    assert_eq!(m, with_area);
    // A locked model.
    m.is_locked = true;
    let locked = m.clone();
    pick(
        &mut st,
        &mut m,
        &[(320.0, 550.0), (440.0, 550.0), (440.0, 450.0)],
    );
    let out = go(&mut st, &mut m, PointerEvent::Enter);
    assert!(applied(&out).clone().unwrap_err().contains("locked"), "{out:?}");
    assert_eq!(m, locked);
    // A plane off the grid, and a missing area section.
    m.is_locked = false;
    let off = PlaneView::of(ViewProjection::PlanXY, Some(1.0)).unwrap();
    let before = m.clone();
    let out = on_event(&mut State::default(), ToolMode::QuickArea, &plan(), Some(off), click(380.0, 400.0), &mut m);
    assert!(applied(&out).clone().unwrap_err().contains("not a grid plane"), "{out:?}");
    let mut st = State {
        area_section: "NOPE".into(),
        ..State::default()
    };
    let out = on_event(&mut st, ToolMode::QuickArea, &plan(), Some(plane), click(380.0, 400.0), &mut m);
    assert!(applied(&out).clone().unwrap_err().contains("NOPE"), "{out:?}");
    assert_eq!(m, before);
}

// ---- reactions of a portal drawn with PolyFrame ------------------------------

fn portal_reactions(m: &mut SapModel) -> [[f64; 6]; 2] {
    m.load_patterns.get_mut("DEAD").unwrap().self_wt_multiplier = 0.0;
    for j in ["1", "4"] {
        m.points.get_mut(j).unwrap().restraint = [true; 6];
    }
    m.points
        .get_mut("2")
        .unwrap()
        .loads_force
        .push(sap_model::PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: [10.0, 0.0, -5.0, 0.0, 0.0, 0.0],
        });
    sap_solver::analyze_model(m).expect("analysis");
    let r = &m.case_results["DEAD"].joint_reactions;
    [r["1"], r["4"]]
}

#[test]
fn a_portal_drawn_with_polyframe_gives_the_reactions_of_the_code_built_portal() {
    let mut drawn = grid_model();
    let plane = plane_at(&drawn, ViewProjection::ElevationXZ, 0);
    let view = elevation(ViewProjection::ElevationXZ);
    let mut st = State::default();
    // (0,0,0) -> (0,0,3.2) -> (6,0,3.2) -> (6,0,0), snapped to the grid.
    for (x, y) in [(200.0, 592.0), (200.0, 528.0), (320.0, 528.0), (320.0, 592.0)] {
        on_event(&mut st, ToolMode::PolyFrame, &view, Some(plane), click(x, y), &mut drawn);
    }
    assert_eq!(drawn.frames.len(), 3);

    let mut coded = SapModel::new(Units::KnMC);
    coded.add_point("1", 0.0, 0.0, 0.0);
    coded.add_point("2", 0.0, 0.0, 3.2);
    coded.add_point("3", 6.0, 0.0, 3.2);
    coded.add_point("4", 6.0, 0.0, 0.0);
    coded.add_frame("1", "1", "2", "FSEC1");
    coded.add_frame("2", "2", "3", "FSEC1");
    coded.add_frame("3", "3", "4", "FSEC1");
    for (name, p) in &coded.points {
        let q = &drawn.points[name];
        assert_eq!([p.x, p.y, p.z], [q.x, q.y, q.z]);
    }
    for (name, f) in &coded.frames {
        assert_eq!(
            (&f.point_i, &f.point_j),
            (&drawn.frames[name].point_i, &drawn.frames[name].point_j)
        );
    }
    let a = portal_reactions(&mut drawn);
    let b = portal_reactions(&mut coded);
    assert!(a[0][0].abs() > 1.0, "the portal must carry the lateral load");
    // Statics: the horizontal reactions balance the 10 kN load.
    assert!((a[0][0] + a[1][0] + 10.0).abs() < 1e-9);
    for k in 0..2 {
        for d in 0..6 {
            assert!((a[k][d] - b[k][d]).abs() <= 1e-12 * (1.0 + b[k][d].abs()), "{k} {d}");
        }
    }
}

// ---- preview and escape --------------------------------------------------------

#[test]
fn the_preview_shows_the_segment_cell_and_pending_points() {
    let m = grid_model();
    let plane = plane_at(&m, ViewProjection::PlanXY, 2);
    let st = State::default();
    let hover = Some(Pos2::new(380.0, 452.0));
    assert_eq!(preview(&st, ToolMode::QuickFrame, &plan(), Some(plane), hover, &m).len(), 1);
    assert_eq!(preview(&st, ToolMode::QuickArea, &plan(), Some(plane), Some(Pos2::new(380.0, 400.0)), &m).len(), 1);
    assert!(preview(&st, ToolMode::QuickFrame, &plan(), None, hover, &m).is_empty());
    assert!(preview(&st, ToolMode::QuickFrame, &plan(), Some(plane), None, &m).is_empty());
    // A pending point of PolyFrame adds the rubber-band line.
    let mut st = State::default();
    let mut scratch = m.clone();
    on_event(&mut st, ToolMode::PolyFrame, &plan(), Some(plane), click(200.0, 550.0), &mut scratch);
    let shapes = preview(&st, ToolMode::PolyFrame, &plan(), Some(plane), Some(Pos2::new(320.0, 550.0)), &m);
    assert_eq!(shapes.len(), 2, "snap marker and line");
    // In another tool the pending point is not used.
    let other = preview(&st, ToolMode::PolyArea, &plan(), Some(plane), Some(Pos2::new(320.0, 550.0)), &m);
    assert_eq!(other.len(), 1);
}

// ---- headless rendering ------------------------------------------------------

fn texts(out: &egui::FullOutput) -> Vec<String> {
    fn walk(s: &egui::epaint::Shape, acc: &mut Vec<String>) {
        match s {
            egui::epaint::Shape::Text(t) => acc.push(t.galley.job.text.clone()),
            egui::epaint::Shape::Vec(v) => v.iter().for_each(|s| walk(s, acc)),
            _ => {}
        }
    }
    let mut acc = Vec::new();
    for c in &out.shapes {
        walk(&c.shape, &mut acc);
    }
    acc
}

fn input(time: f64, events: Vec<egui::Event>) -> egui::RawInput {
    egui::RawInput {
        screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1800.0, 1400.0))),
        time: Some(time),
        events,
        ..Default::default()
    }
}

#[test]
fn the_properties_window_renders_the_options_of_each_tool() {
    let m = grid_model();
    let cases = [
        (ToolMode::QuickFrame, vec!["Section", "Pinned ends"]),
        (ToolMode::PolyFrame, vec!["Section", "Pinned ends"]),
        (ToolMode::QuickBrace, vec!["Section", "Brace"]),
        (ToolMode::QuickSecondary, vec!["Section", "Beams per bay", "Span"]),
        (ToolMode::QuickArea, vec!["Area section"]),
        (ToolMode::RectArea, vec!["Area section"]),
        (ToolMode::PolyArea, vec!["Area section"]),
    ];
    for (k, (tool, labels)) in cases.into_iter().enumerate() {
        let ctx = egui::Context::default();
        let mut st = State::default();
        let mut shown = Vec::new();
        for f in 0..3 {
            let out = ctx.run(input(f64::from(f), Vec::new()), |ctx| {
                render_properties(ctx, &mut st, tool, &m);
            });
            shown = texts(&out);
        }
        assert!(
            ctx.memory(|mem| mem.area_rect(egui::Id::new("Properties of Object")))
                .is_some(),
            "{tool:?}: window not shown"
        );
        for label in labels {
            assert!(
                shown.iter().any(|t| t == label),
                "case {k} {tool:?}: no label {label:?} in {shown:?}"
            );
        }
        // The empty choices are filled with the first section of the model.
        assert_eq!(st.frame_section, "FSEC1");
        assert_eq!(st.area_section, "ASEC1");
    }
}

#[test]
fn the_properties_window_edits_the_state() {
    let m = grid_model();
    let ctx = egui::Context::default();
    let mut st = State::default();
    let run = |st: &mut State, time: f64, events: Vec<egui::Event>| {
        ctx.run(input(time, events), |ctx| {
            render_properties(ctx, st, ToolMode::PolyFrame, &m);
        })
    };
    run(&mut st, 0.0, Vec::new());
    run(&mut st, 1.0, Vec::new());
    let out = run(&mut st, 2.0, Vec::new());
    // Click the check box: its label is "Release M2 and M3 at both ends".
    let pos = out
        .shapes
        .iter()
        .find_map(|c| find_text(&c.shape, "Release M2 and M3 at both ends"))
        .expect("check box label");
    let press = vec![
        egui::Event::PointerMoved(pos),
        egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed: true,
            modifiers: Default::default(),
        },
    ];
    let release = vec![egui::Event::PointerButton {
        pos,
        button: egui::PointerButton::Primary,
        pressed: false,
        modifiers: Default::default(),
    }];
    run(&mut st, 3.0, press);
    run(&mut st, 3.1, release);
    assert!(st.pinned);
}

fn find_text(s: &egui::epaint::Shape, text: &str) -> Option<Pos2> {
    match s {
        egui::epaint::Shape::Text(t) if t.galley.job.text == text => {
            Some(t.visual_bounding_rect().center())
        }
        egui::epaint::Shape::Vec(v) => v.iter().find_map(|s| find_text(s, text)),
        _ => None,
    }
}

#[test]
fn a_real_click_in_the_viewport_with_the_quick_frame_tool_draws_exactly_one_beam() {
    let mut app = SapGuiApp::new(Some(PathBuf::from("/nonexistent/draw-tools.s2k")));
    app.model = grid_model();
    let z2 = grid_axes(&app.model).unwrap()[2][2];
    app.dual_pane = false;
    app.left_proj = ViewProjection::PlanXY;
    app.left_plane = Some(z2);
    app.tool_mode = ToolMode::QuickFrame;
    let ctx = egui::Context::default();
    // Two frames to settle the layout, then read the projection of the pane.
    for t in 0..2 {
        let _ = ctx.run(input(f64::from(t), Vec::new()), |ctx| app.render_ui(ctx));
    }
    let (view, plane) = app.last_view[0].expect("the viewport has rendered");
    assert_eq!(plane.map(|p| p.normal), Some(PlaneAxis::Z));
    assert_eq!(plane.map(|p| p.coordinate), Some(z2));
    // The middle of the X segment (12, 5)-(18, 5).
    let pos = view.project([15.0, 5.0, z2]);
    let press = vec![
        egui::Event::PointerMoved(pos),
        egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed: true,
            modifiers: Default::default(),
        },
    ];
    let release = vec![egui::Event::PointerButton {
        pos,
        button: egui::PointerButton::Primary,
        pressed: false,
        modifiers: Default::default(),
    }];
    let _ = ctx.run(input(2.0, press), |ctx| app.render_ui(ctx));
    let _ = ctx.run(input(2.1, release), |ctx| app.render_ui(ctx));
    assert_eq!(app.model.frames.len(), 1, "{}", app.status_msg);
    let f = app.model.frames.values().next().unwrap();
    let (i, j) = (&app.model.points[&f.point_i], &app.model.points[&f.point_j]);
    assert_eq!([i.x, i.y, i.z], [12.0, 5.0, z2]);
    assert_eq!([j.x, j.y, j.z], [18.0, 5.0, z2]);
    assert!(app.status_msg.starts_with("Drew 1 frame"), "{}", app.status_msg);
}
