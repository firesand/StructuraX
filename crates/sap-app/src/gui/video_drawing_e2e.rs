//! End-to-end tests of the drawing workflow of the tutorial video, driven
//! through the GUI's own functions. They live inside the crate because the
//! application is a binary without a library target.
//!
//! The first group checks the join between the viewport (which turns pointer
//! and key input into events) and the drawing tools (which turn events into
//! model commands): each test sends real pointer or key input through
//! `render_ui` and compares the model with objects written out by hand.

mod building;
mod chooser;
mod frame_template;
mod models;
mod part4;

use super::view::{PlaneView, ViewTransform};
use super::{SapGuiApp, ToolMode, ViewProjection};
use eframe::egui::{self, Pos2, Rect, Vec2};
use sap_model::SapModel;
use sap_units::Units;
use std::collections::BTreeSet;
use std::path::PathBuf;

/// The grid of the video's first model: 6 x 4 x 7 lines at 6, 5 and 3.2 m.
fn grid_app(proj: ViewProjection, plane: f64, tool: ToolMode) -> (SapGuiApp, egui::Context, f64) {
    let mut app = SapGuiApp::new(Some(PathBuf::from("/nonexistent/video-drawing.s2k")));
    let mut model = SapModel::new(Units::KnMC);
    model
        .quick_grid([6, 4, 7], [6.0, 5.0, 3.2], [0.0; 3])
        .unwrap();
    app.model = model;
    app.dual_pane = false;
    app.left_proj = proj;
    app.left_plane = Some(plane);
    app.tool_mode = tool;
    let ctx = egui::Context::default();
    let mut time = 0.0;
    // Two frames settle the layout before the projection is read.
    for _ in 0..2 {
        frame(&mut app, &ctx, &mut time, Vec::new());
    }
    (app, ctx, time)
}

fn frame(app: &mut SapGuiApp, ctx: &egui::Context, time: &mut f64, events: Vec<egui::Event>) {
    *time += 0.05;
    let input = egui::RawInput {
        screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1800.0, 1400.0))),
        time: Some(*time),
        events,
        ..Default::default()
    };
    let _ = ctx.run(input, |ctx| app.render_ui(ctx));
}

fn button(pos: Pos2, pressed: bool) -> egui::Event {
    egui::Event::PointerButton {
        pos,
        button: egui::PointerButton::Primary,
        pressed,
        modifiers: Default::default(),
    }
}

fn click(app: &mut SapGuiApp, ctx: &egui::Context, time: &mut f64, pos: Pos2) {
    frame(app, ctx, time, vec![egui::Event::PointerMoved(pos), button(pos, true)]);
    frame(app, ctx, time, vec![button(pos, false)]);
    // Past the double-click interval, so the next click is a single one.
    *time += 1.0;
}

fn drag(app: &mut SapGuiApp, ctx: &egui::Context, time: &mut f64, from: Pos2, to: Pos2) {
    let mid = from + (to - from) * 0.5;
    frame(app, ctx, time, vec![egui::Event::PointerMoved(from)]);
    frame(app, ctx, time, vec![button(from, true)]);
    frame(app, ctx, time, vec![egui::Event::PointerMoved(mid)]);
    frame(app, ctx, time, vec![egui::Event::PointerMoved(to)]);
    frame(app, ctx, time, vec![button(to, false)]);
    frame(app, ctx, time, Vec::new());
    *time += 1.0;
}

fn key(app: &mut SapGuiApp, ctx: &egui::Context, time: &mut f64, key: egui::Key) {
    let event = |pressed| egui::Event::Key {
        key,
        physical_key: None,
        pressed,
        repeat: false,
        modifiers: Default::default(),
    };
    frame(app, ctx, time, vec![event(true)]);
    frame(app, ctx, time, vec![event(false)]);
}

fn pane(app: &SapGuiApp) -> (ViewTransform, PlaneView) {
    let (view, plane) = app.last_view[0].expect("the pane has been drawn");
    (view, plane.expect("the pane shows a plane"))
}

/// Coordinates in millimetres, so that end pairs can be compared as sets.
fn mm(c: [f64; 3]) -> [i64; 3] {
    c.map(|v| (v * 1000.0).round() as i64)
}

fn frame_ends(model: &SapModel) -> BTreeSet<([i64; 3], [i64; 3])> {
    model
        .frames
        .values()
        .map(|f| {
            let (i, j) = (&model.points[&f.point_i], &model.points[&f.point_j]);
            (mm([i.x, i.y, i.z]), mm([j.x, j.y, j.z]))
        })
        .collect()
}

#[test]
fn a_window_dragged_with_the_quick_frame_tool_draws_the_grid_segments_inside_it() {
    let z2 = 6.4;
    let (mut app, ctx, mut time) = grid_app(ViewProjection::PlanXY, z2, ToolMode::QuickFrame);
    let (view, _) = pane(&app);
    // Model window x 5..26, y 1.5..12.5: X lines 6, 12, 18, 24 and Y lines 5, 10.
    drag(
        &mut app,
        &ctx,
        &mut time,
        view.project([5.0, 12.5, z2]),
        view.project([26.0, 1.5, z2]),
    );
    let mut want = BTreeSet::new();
    for y in [5.0, 10.0] {
        for x in [6.0, 12.0, 18.0] {
            want.insert((mm([x, y, z2]), mm([x + 6.0, y, z2])));
        }
    }
    for x in [6.0, 12.0, 18.0, 24.0] {
        want.insert((mm([x, 5.0, z2]), mm([x, 10.0, z2])));
    }
    assert_eq!(want.len(), 10);
    assert_eq!(frame_ends(&app.model), want, "{}", app.status_msg);
    assert_eq!(app.tool_mode, ToolMode::QuickFrame, "the tool stays active");
    // Every joint took the stored grid coordinate, not a screen round trip.
    for p in app.model.points.values() {
        assert_eq!(p.z, z2);
        assert_eq!((p.x / 6.0).fract(), 0.0);
        assert_eq!((p.y / 5.0).fract(), 0.0);
    }
    // The same window again creates nothing and says so.
    drag(
        &mut app,
        &ctx,
        &mut time,
        view.project([5.0, 12.5, z2]),
        view.project([26.0, 1.5, z2]),
    );
    assert_eq!(app.model.frames.len(), 10);
    assert!(app.status_msg.starts_with("Not done"), "{}", app.status_msg);
}

#[test]
fn a_window_dragged_right_to_left_with_the_quick_area_tool_draws_the_cells_inside_it() {
    let z1 = 3.2;
    let (mut app, ctx, mut time) = grid_app(ViewProjection::PlanXY, z1, ToolMode::QuickArea);
    let (view, _) = pane(&app);
    drag(
        &mut app,
        &ctx,
        &mut time,
        view.project([26.0, 1.5, z1]),
        view.project([5.0, 12.5, z1]),
    );
    // Cells [6,12], [12,18], [18,24] x [5,10]; corners counter-clockwise seen
    // from above, starting at the lower left, so local 3 is +Z.
    assert_eq!(app.model.areas.len(), 3, "{}", app.status_msg);
    let mut cells = BTreeSet::new();
    for a in app.model.areas.values() {
        let c: Vec<[f64; 3]> = a
            .points
            .iter()
            .map(|p| {
                let p = &app.model.points[p];
                [p.x, p.y, p.z]
            })
            .collect();
        assert_eq!(c.len(), 4);
        let normal_z = (c[1][0] - c[0][0]) * (c[3][1] - c[0][1])
            - (c[1][1] - c[0][1]) * (c[3][0] - c[0][0]);
        assert_eq!(normal_z, 30.0, "area {} of 6 m x 5 m faces up", a.name);
        cells.insert(mm(c[0]));
    }
    let want: BTreeSet<_> = [6.0, 12.0, 18.0].map(|x| mm([x, 5.0, z1])).into();
    assert_eq!(cells, want);
}

#[test]
fn a_narrow_window_in_an_elevation_draws_one_column_line() {
    let y1 = 5.0;
    let (mut app, ctx, mut time) = grid_app(ViewProjection::ElevationXZ, y1, ToolMode::QuickFrame);
    let (view, plane) = pane(&app);
    assert_eq!(plane.title(), "X-Z Plane @ Y=5");
    // x 5..7 holds the grid line x = 6 only; z -1..20 holds all seven levels.
    drag(
        &mut app,
        &ctx,
        &mut time,
        view.project([5.0, y1, 20.0]),
        view.project([7.0, y1, -1.0]),
    );
    let want: BTreeSet<_> = (0..6)
        .map(|k| {
            let (z0, z1) = (k as f64 * 3.2, (k + 1) as f64 * 3.2);
            (mm([6.0, y1, z0]), mm([6.0, y1, z1]))
        })
        .collect();
    assert_eq!(frame_ends(&app.model), want, "{}", app.status_msg);
}

#[test]
fn clicks_enter_and_escape_drive_the_frame_through_points_tool() {
    let z0 = 0.0;
    let (mut app, ctx, mut time) = grid_app(ViewProjection::PlanXY, z0, ToolMode::PolyFrame);
    let (view, _) = pane(&app);
    // Three clicks near grid intersections (2 px off, inside the snap radius).
    for c in [[0.0, 0.0], [6.0, 0.0], [6.0, 5.0]] {
        let at = view.project([c[0], c[1], z0]) + Vec2::new(2.0, -2.0);
        click(&mut app, &ctx, &mut time, at);
    }
    let want: BTreeSet<_> = [
        (mm([0.0, 0.0, z0]), mm([6.0, 0.0, z0])),
        (mm([6.0, 0.0, z0]), mm([6.0, 5.0, z0])),
    ]
    .into();
    assert_eq!(frame_ends(&app.model), want, "{}", app.status_msg);
    // Enter ends the sequence: the next click starts a new one, so it draws
    // nothing by itself.
    key(&mut app, &ctx, &mut time, egui::Key::Enter);
    assert_eq!(app.tool_mode, ToolMode::PolyFrame);
    click(&mut app, &ctx, &mut time, view.project([18.0, 10.0, z0]));
    assert_eq!(app.model.frames.len(), 2, "{}", app.status_msg);
    // Escape drops the pending point and returns to the pointer.
    key(&mut app, &ctx, &mut time, egui::Key::Escape);
    assert_eq!(app.tool_mode, ToolMode::Select);
    app.tool_mode = ToolMode::PolyFrame;
    click(&mut app, &ctx, &mut time, view.project([24.0, 10.0, z0]));
    assert_eq!(app.model.frames.len(), 2, "no frame from the dropped point");
}

#[test]
fn a_selection_window_in_a_plane_view_takes_only_what_the_plane_shows() {
    let z2 = 6.4;
    let (mut app, ctx, mut time) = grid_app(ViewProjection::PlanXY, z2, ToolMode::Select);
    // Beams at two levels and the columns between them.
    let region = |z: (usize, usize)| sap_model::generate::GridRegion {
        x: (0, 5),
        y: (0, 3),
        z,
    };
    let section = app.model.frame_sections.keys().next().cloned().unwrap();
    app.model
        .quick_draw_beams(region((1, 2)), true, true, &section)
        .unwrap();
    app.model
        .quick_draw_columns(region((0, 2)), &section)
        .unwrap();
    frame(&mut app, &ctx, &mut time, Vec::new());
    let (view, _) = pane(&app);
    drag(
        &mut app,
        &ctx,
        &mut time,
        view.project([-1.0, 16.0, z2]),
        view.project([31.0, -1.0, z2]),
    );
    let selected: Vec<_> = app.model.frames.values().filter(|f| f.selected).collect();
    // 5 x 4 beams along X and 3 x 6 along Y at this level; no column and no
    // beam of the level below, although they project into the same window.
    assert_eq!(selected.len(), 38);
    for f in selected {
        for p in [&f.point_i, &f.point_j] {
            assert_eq!(app.model.points[p].z, z2);
        }
    }
    assert_eq!(app.model.points.values().filter(|p| p.selected).count(), 24);
}
