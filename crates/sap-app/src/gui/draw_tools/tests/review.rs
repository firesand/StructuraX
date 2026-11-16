//! Further tests: stored level of a typed plane, the Y-Z elevation, a fresh
//! quick-area window, and the second click of a double click.

use super::*;

#[test]
fn a_plane_typed_off_the_grid_by_rounding_still_gives_the_stored_level() {
    let mut m = grid_model();
    let z2 = grid_axes(&m).unwrap()[2][2];
    let typed = PlaneView::of(ViewProjection::PlanXY, Some(z2 + 1e-13)).unwrap();
    assert_ne!(typed.coordinate, z2);
    // Free point and snapped grid intersection: the stored level on the normal axis.
    assert_eq!(
        pick_point(&m, &plan(), Some(typed), Pos2::new(390.0, 450.0)),
        Ok([9.5, 5.0, z2])
    );
    assert_eq!(
        pick_point(&m, &plan(), Some(typed), Pos2::new(321.0, 451.0)),
        Ok([6.0, 5.0, z2])
    );
    // The rectangle and the quick tools use it, too.
    let mut st = State::default();
    on_event(
        &mut st,
        ToolMode::RectArea,
        &plan(),
        Some(typed),
        click(200.0, 550.0),
        &mut m,
    );
    let out = on_event(
        &mut st,
        ToolMode::RectArea,
        &plan(),
        Some(typed),
        click(320.0, 450.0),
        &mut m,
    );
    assert!(applied(&out).is_ok(), "{out:?}");
    assert!(corners_of(&m, "1").iter().all(|c| c[2] == z2));
    let out = on_event(
        &mut State::default(),
        ToolMode::QuickFrame,
        &plan(),
        Some(typed),
        click(380.0, 452.0),
        &mut m,
    );
    assert!(applied(&out).is_ok(), "{out:?}");
    assert!(m.points.values().all(|p| p.z == z2));
}

#[test]
fn the_y_z_elevation_matches_the_index_commands() {
    let region = GridRegion {
        x: (1, 1),
        y: (0, 3),
        z: (0, 6),
    };
    let mut drawn = grid_model();
    let mut reference = grid_model();
    let plane = plane_at(&drawn, ViewProjection::ElevationYZ, 1);
    assert_eq!(plane.coordinate, 6.0);
    let view = elevation(ViewProjection::ElevationYZ);
    // y = 0..15 at screen x 350..650, z = 0..19.2 at screen y 592..208.
    let out = on_event(
        &mut State::default(),
        ToolMode::QuickFrame,
        &view,
        Some(plane),
        window(300.0, 150.0, 700.0, 650.0),
        &mut drawn,
    );
    assert!(applied(&out).is_ok(), "{out:?}");
    reference
        .quick_draw_beams(region, false, true, "FSEC1")
        .unwrap();
    reference.quick_draw_columns(region, "FSEC1").unwrap();
    assert_eq!(drawn, reference);
    assert_eq!(drawn.frames.len(), 3 * 7 + 4 * 6);
    // Braces: cell y 5..10, z 9.6..12.8.
    let mut st = State {
        brace_kind: BraceKind::V,
        ..State::default()
    };
    let out = on_event(
        &mut st,
        ToolMode::QuickBrace,
        &view,
        Some(plane),
        click(500.0, 370.0),
        &mut drawn,
    );
    assert!(applied(&out).is_ok(), "{out:?}");
    reference
        .quick_draw_braces(
            GridRegion {
                x: (1, 1),
                y: (1, 2),
                z: (3, 4),
            },
            VerticalPlane::YZ,
            BraceKind::V,
            "FSEC1",
        )
        .unwrap();
    assert_eq!(drawn, reference);
}

#[test]
fn a_quick_area_window_on_a_fresh_model_equals_quick_draw_areas() {
    let mut drawn = grid_model();
    let mut reference = grid_model();
    let plane = plane_at(&drawn, ViewProjection::PlanXY, 2);
    let out = on_event(
        &mut State::default(),
        ToolMode::QuickArea,
        &plan(),
        Some(plane),
        window(150.0, 200.0, 850.0, 600.0),
        &mut drawn,
    );
    assert!(applied(&out).is_ok(), "{out:?}");
    reference
        .quick_draw_areas(
            GridRegion {
                x: (0, 5),
                y: (0, 3),
                z: (2, 2),
            },
            "ASEC1",
        )
        .unwrap();
    assert_eq!(drawn, reference);
}

#[test]
fn the_second_click_of_a_double_click_does_not_undo_the_first() {
    let dbl = |x: f32, y: f32| PointerEvent::Click {
        pos: Pos2::new(x, y),
        double: true,
    };
    // PolyArea closed by a click on its first point: the double click that
    // follows finds nothing pending.
    let mut m = grid_model();
    let plane = plane_at(&m, ViewProjection::PlanXY, 2);
    let mut st = State::default();
    let area = ToolMode::PolyArea;
    for (x, y) in [(200.0, 550.0), (320.0, 550.0), (320.0, 450.0)] {
        on_event(&mut st, area, &plan(), Some(plane), click(x, y), &mut m);
    }
    let out = on_event(&mut st, area, &plan(), Some(plane), click(200.0, 550.0), &mut m);
    assert!(applied(&out).is_ok(), "{out:?}");
    let before = m.clone();
    let out = on_event(&mut st, area, &plan(), Some(plane), dbl(200.0, 550.0), &mut m);
    assert_eq!(out, DrawOutcome::Nothing);
    assert_eq!(m, before);
    // PolyFrame ended by a click on the same point.
    let poly = ToolMode::PolyFrame;
    let mut st = State::default();
    on_event(&mut st, poly, &plan(), Some(plane), click(200.0, 550.0), &mut m);
    on_event(&mut st, poly, &plan(), Some(plane), click(200.0, 550.0), &mut m);
    let out = on_event(&mut st, poly, &plan(), Some(plane), dbl(200.0, 550.0), &mut m);
    assert_eq!(out, DrawOutcome::Nothing);
    // RectArea: a double click neither completes nor restarts a rectangle.
    let rect = ToolMode::RectArea;
    let mut st = State::default();
    on_event(&mut st, rect, &plan(), Some(plane), click(440.0, 550.0), &mut m);
    let out = on_event(&mut st, rect, &plan(), Some(plane), dbl(440.0, 550.0), &mut m);
    assert_eq!(out, DrawOutcome::Nothing);
    let out = on_event(&mut st, rect, &plan(), Some(plane), click(560.0, 450.0), &mut m);
    assert!(applied(&out).is_ok(), "the first corner was kept: {out:?}");
    let out = on_event(&mut st, rect, &plan(), Some(plane), dbl(560.0, 450.0), &mut m);
    assert_eq!(out, DrawOutcome::Nothing);
    // Quick tools: the repeated click would be refused as a duplicate.
    let quick = ToolMode::QuickFrame;
    let mut st = State::default();
    let out = on_event(&mut st, quick, &plan(), Some(plane), click(380.0, 452.0), &mut m);
    assert!(applied(&out).is_ok());
    let out = on_event(&mut st, quick, &plan(), Some(plane), dbl(380.0, 452.0), &mut m);
    assert_eq!(out, DrawOutcome::Nothing);
}
