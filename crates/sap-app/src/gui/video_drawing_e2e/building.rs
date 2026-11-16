//! Test A: the part 1 building drawn with the mouse in plane views against
//! the same building built with the index commands.

use super::models::*;
use super::{click, drag, frame, grid_app, mm, pane};
use crate::gui::{SapGuiApp, ToolMode, ViewProjection};
use eframe::egui;
use sap_model::generate::{BraceKind, GridRegion, VerticalPlane};
use sap_model::SapModel;
use sap_solver::analyze_model;

type JointSig = ([i64; 3], [bool; 6]);
type FrameSig = ([i64; 3], [i64; 3], String, [bool; 6], [bool; 6], bool);
type AreaSig = (Vec<[i64; 3]>, String);

fn joint_sigs(m: &SapModel) -> Vec<JointSig> {
    let mut v: Vec<_> = m.points.values().map(|p| (mm([p.x, p.y, p.z]), p.restraint)).collect();
    v.sort();
    v
}

fn frame_sigs(m: &SapModel) -> Vec<FrameSig> {
    let mut v: Vec<_> = m
        .frames
        .values()
        .map(|f| {
            let (i, j) = (&m.points[&f.point_i], &m.points[&f.point_j]);
            (
                mm([i.x, i.y, i.z]),
                mm([j.x, j.y, j.z]),
                f.section.clone(),
                f.release_i,
                f.release_j,
                f.auto_mesh_at_joints,
            )
        })
        .collect();
    v.sort();
    v
}

fn area_sigs(m: &SapModel) -> Vec<AreaSig> {
    let mut v: Vec<_> = m
        .areas
        .values()
        .map(|a| {
            let corners = a
                .points
                .iter()
                .map(|p| {
                    let p = &m.points[p];
                    mm([p.x, p.y, p.z])
                })
                .collect();
            (corners, a.section.clone())
        })
        .collect();
    v.sort();
    v
}

/// Shows the plane `value` of `proj` with `tool` and lets the layout settle.
fn show(
    app: &mut SapGuiApp,
    ctx: &egui::Context,
    time: &mut f64,
    proj: ViewProjection,
    value: f64,
    tool: ToolMode,
) {
    app.left_proj = proj;
    app.left_plane = Some(value);
    app.tool_mode = tool;
    for _ in 0..2 {
        frame(app, ctx, time, Vec::new());
    }
}

/// The index-command reference of the building.
fn reference() -> SapModel {
    let mut m = building_base();
    let all = GridRegion { x: (0, NX - 1), y: (0, NY - 1), z: (0, NZ - 1) };
    let floors = GridRegion { z: (1, NZ - 1), ..all };
    m.quick_draw_columns(all, "HE300B").unwrap();
    m.quick_draw_beams(floors, true, true, "IPE330").unwrap();
    for k in 0..NZ - 1 {
        let bay = GridRegion { x: (2, 3), y: (1, 1), z: (k, k + 1) };
        m.quick_draw_braces(bay, VerticalPlane::XZ, BraceKind::InvertedV, "IPE270").unwrap();
    }
    m.quick_draw_areas(floors, "SLAB").unwrap();
    for p in m.points.values_mut().filter(|p| p.z.abs() < 1e-9) {
        p.restraint = [true; 6];
    }
    m
}

/// Draws the building with pointer input only (plus the Fixed restraint
/// command of the app on a window selection).
fn drawn() -> SapGuiApp {
    let (mut app, ctx, mut time) = grid_app(ViewProjection::PlanXY, 0.0, ToolMode::Select);
    app.model = building_base();
    let line = |n: usize, d: f64| -> Vec<f64> { (0..n).map(|i| i as f64 * d).collect() };
    let (xs, ys, zs) = (line(NX, DX), line(NY, DY), line(NZ, DZ));
    assert_eq!((xs.len(), ys.len(), zs.len()), (NX, NY, NZ));
    let (x_end, y_end) = (xs[NX - 1], ys[NY - 1]);

    // 1. Beams: one QuickFrame window over the grid at each plan z1..z6.
    app.draw_tools.frame_section = "IPE330".into();
    for &z in &zs[1..] {
        show(&mut app, &ctx, &mut time, ViewProjection::PlanXY, z, ToolMode::QuickFrame);
        let (view, _) = pane(&app);
        drag(
            &mut app,
            &ctx,
            &mut time,
            view.project([-0.5, y_end + 0.5, z]),
            view.project([x_end + 0.5, -0.5, z]),
        );
    }
    let per_level = NY * (NX - 1) + NX * (NY - 1);
    assert_eq!(app.model.frames.len(), per_level * (NZ - 1), "{}", app.status_msg);

    // 2. Columns: one narrow window per column line in each X-Z elevation.
    app.draw_tools.frame_section = "HE300B".into();
    for &y in &ys {
        show(&mut app, &ctx, &mut time, ViewProjection::ElevationXZ, y, ToolMode::QuickFrame);
        let (view, _) = pane(&app);
        for &x in &xs {
            drag(
                &mut app,
                &ctx,
                &mut time,
                view.project([x - 0.8, y, zs[NZ - 1] + 0.8]),
                view.project([x + 0.8, y, -0.8]),
            );
        }
    }
    assert_eq!(
        app.model.frames.len(),
        per_level * (NZ - 1) + NX * NY * (NZ - 1),
        "{}",
        app.status_msg
    );

    // 3. Inverted-V braces in the bay x 12..18 of the elevation y = 5: one
    //    click per storey.
    app.draw_tools.frame_section = "IPE270".into();
    app.draw_tools.brace_kind = BraceKind::InvertedV;
    show(&mut app, &ctx, &mut time, ViewProjection::ElevationXZ, ys[1], ToolMode::QuickBrace);
    let (view, _) = pane(&app);
    for k in 0..NZ - 1 {
        let at = view.project([0.5 * (xs[2] + xs[3]), ys[1], 0.5 * (zs[k] + zs[k + 1])]);
        click(&mut app, &ctx, &mut time, at);
        assert!(
            app.status_msg.starts_with("Drew 2 brace"),
            "storey {}: {}",
            k + 1,
            app.status_msg
        );
    }

    // 4. Slabs: one QuickArea window per plan.
    app.draw_tools.area_section = "SLAB".into();
    for &z in &zs[1..] {
        show(&mut app, &ctx, &mut time, ViewProjection::PlanXY, z, ToolMode::QuickArea);
        let (view, _) = pane(&app);
        drag(
            &mut app,
            &ctx,
            &mut time,
            view.project([-0.5, y_end + 0.5, z]),
            view.project([x_end + 0.5, -0.5, z]),
        );
    }
    assert_eq!(app.model.areas.len(), (NX - 1) * (NY - 1) * (NZ - 1), "{}", app.status_msg);

    // 5. Supports: a selection window over the plan at z0, then Fixed.
    show(&mut app, &ctx, &mut time, ViewProjection::PlanXY, zs[0], ToolMode::Select);
    let (view, _) = pane(&app);
    drag(
        &mut app,
        &ctx,
        &mut time,
        view.project([-0.5, y_end + 0.5, zs[0]]),
        view.project([x_end + 0.5, -0.5, zs[0]]),
    );
    assert_eq!(app.model.points.values().filter(|p| p.selected).count(), NX * NY);
    assert_eq!(app.model.frames.values().filter(|f| f.selected).count(), 0);
    app.assign_restraint_to_selected([true; 6], "Fixed");
    assert!(app.status_msg.contains("Fixed restraint to 24 joints"), "{}", app.status_msg);
    app
}

/// Largest component, the scale of a vector of reactions.
fn scale(v: &[f64; 6]) -> f64 {
    v.iter().fold(0.0_f64, |a, b| a.max(b.abs()))
}

#[test]
fn the_part_1_building_drawn_with_the_mouse_equals_the_one_built_with_index_commands() {
    let app = drawn();
    let mut reference = reference();
    let mut model = app.model.clone();

    // --- the sets of objects ----------------------------------------------
    // The grid joints and one apex joint per brace pair.
    assert_eq!(model.points.len(), NX * NY * NZ + (NZ - 1));
    assert_eq!(model.frames.len(), reference.frames.len());
    assert_eq!(model.areas.len(), reference.areas.len());
    assert_eq!(joint_sigs(&model), joint_sigs(&reference), "joints with restraints");
    assert_eq!(frame_sigs(&model), frame_sigs(&reference), "frames");
    assert_eq!(area_sigs(&model), area_sigs(&reference), "areas");
    // 144 columns, 6 x 31 beams, 12 braces; the six beams under the braces
    // are flagged for meshing at the apex.
    let braces = model.frames.values().filter(|f| f.section == "IPE270").count();
    assert_eq!(braces, 2 * (NZ - 1));
    assert_eq!(model.frames.values().filter(|f| f.auto_mesh_at_joints).count(), NZ - 1);
    assert!(model
        .frames
        .values()
        .filter(|f| f.section == "IPE270")
        .all(|f| f.release_i[4] && f.release_i[5] && f.release_j[4] && f.release_j[5]));

    // --- the same loads, mass source and cases, then the analysis ----------
    let drawn_only = model.clone();
    building_loads_and_cases(&mut model);
    building_loads_and_cases(&mut reference);
    analyze_model(&mut model).unwrap();
    analyze_model(&mut reference).unwrap();
    for case in ["DEAD", "SDEAD", "LIVE", "EX"] {
        let (a, b) = (
            model.case_results[case].base_reactions,
            reference.case_results[case].base_reactions,
        );
        let s = scale(&b);
        assert!(s > 1.0, "{case}: no reaction");
        for k in 0..6 {
            assert!(
                (a[k] - b[k]).abs() <= 1e-9 * s,
                "{case} component {k}: drawn {} against reference {}",
                a[k],
                b[k]
            );
        }
    }
    for case in ["MODAL", "EIGEN"] {
        let (a, b) = (&model.case_results[case].modes, &reference.case_results[case].modes);
        assert!(a.len() >= 3 && a.len() == b.len(), "{case}: {} against {} modes", a.len(), b.len());
        for k in 0..3 {
            assert!(
                (a[k].period_sec - b[k].period_sec).abs() <= 1e-8 * b[k].period_sec,
                "{case} period {}: {} against {}",
                k + 1,
                a[k].period_sec,
                b[k].period_sec
            );
        }
    }

    // --- S2K round trip of the drawn model ---------------------------------
    let text = sap_format::write_s2k(&model);
    let back = sap_format::parse_s2k(&text).unwrap();
    assert_eq!(back.points.len(), model.points.len());
    assert_eq!(back.frames.len(), model.frames.len());
    assert_eq!(back.areas.len(), model.areas.len());
    // One joint, one frame and one area survive with their data.
    let joint = (mm([12.0, 5.0, 0.0]), [true; 6]);
    assert!(joint_sigs(&back).contains(&joint));
    // The apex of the first inverted V is at the top of the bay (x 15, z 3.2);
    // the brace runs from the bottom left corner to it.
    let first_brace = (
        mm([12.0, 5.0, 0.0]),
        mm([15.0, 5.0, 3.2]),
        "IPE270".to_string(),
        [false, false, false, false, true, true],
        [false, false, false, false, true, true],
        false,
    );
    assert!(frame_sigs(&back).contains(&first_brace), "{first_brace:?}");
    let slab = (
        vec![
            mm([6.0, 5.0, 3.2]),
            mm([12.0, 5.0, 3.2]),
            mm([12.0, 10.0, 3.2]),
            mm([6.0, 10.0, 3.2]),
        ],
        "SLAB".to_string(),
    );
    assert!(area_sigs(&back).contains(&slab), "{slab:?}");
    assert_eq!(frame_sigs(&back), frame_sigs(&model));
    assert_eq!(area_sigs(&back), area_sigs(&model));
    assert_eq!(joint_sigs(&back), joint_sigs(&model));

    // --- native .sdb export of this model -----------------------------------
    // The experimental native writer (not the private container) refuses the
    // analyzed building, and for the first wave's data (Ritz controls, mass
    // source, automatic seismic load), not for anything that was drawn.
    let path = std::env::temp_dir().join(format!("structurax_p7_building_{}.sdb", std::process::id()));
    let _ = std::fs::remove_file(&path);
    let written = sap_format::save_native_sdb_file(&model, &path);
    let _ = std::fs::remove_file(&path);
    let refusal = written.expect_err("native export of the first wave's data").to_string();
    assert!(
        ["ritz", "mass_source", "auto_seismic"].iter().any(|k| refusal.to_lowercase().contains(k)),
        "{refusal}"
    );
    // The drawn objects alone (before loads, mass source and cases) are
    // written, and the file reads back with the same objects.
    let _ = std::fs::remove_file(&path);
    let written = sap_format::save_native_sdb_file(&drawn_only, &path);
    let read = written.map(|()| sap_format::load_sdb_file(&path));
    let _ = std::fs::remove_file(&path);
    match read {
        Ok(native) => {
            let native = native.expect("native import");
            assert_eq!(frame_sigs(&native), frame_sigs(&drawn_only));
            assert_eq!(area_sigs(&native), area_sigs(&drawn_only));
            assert_eq!(joint_sigs(&native), joint_sigs(&drawn_only));
        }
        Err(error) => panic!("native export of the drawn objects was refused: {error}"),
    }
}
