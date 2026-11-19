//! P16, D53: what File > Open resets and what it keeps (tests through the
//! command function the Open button calls and through the painted status bar).

use super::wave5_windows_tests::Run;
use super::*;
use std::path::PathBuf;

fn temp_file(name: &str) -> PathBuf {
    std::env::temp_dir().join(format!("structurax_p16_{}_{name}", std::process::id()))
}

/// The default 3D building, analysed (so locked, with results), shown as a
/// shell stress, with the right pane on a plan.
fn analysed_app_with_display_state(right_plane: Option<f64>) -> SapGuiApp {
    let mut app = SapGuiApp::new(None);
    app.execute_analysis();
    assert!(app.model.is_locked && !app.model.case_results.is_empty(), "{}", app.status_msg);
    app.dual_pane = true;
    app.right_proj = ViewProjection::PlanXY;
    app.right_plane = right_plane;
    app.display_mode = DisplayMode::ShellContour(shell_output::ShellQuantity::LEGACY[0]);
    app.active_mode_idx = 3;
    app.deform_auto = false;
    app.animate_deformed = true;
    app
}

#[test]
fn opening_an_s2k_resets_results_lock_display_and_keeps_a_plane_of_the_new_grid() {
    // The saved file is the default building's S2K: it holds no results.
    let path = temp_file("building.s2k");
    let saved = SapGuiApp::new(None);
    let levels = view::plane_levels(&saved.model, view::PlaneAxis::Z);
    assert!(levels.len() >= 2, "{levels:?}");
    save_s2k_file_with_report(&saved.model, path.to_str().unwrap()).expect("save");

    let mut app = analysed_app_with_display_state(Some(levels[1]));
    assert!(app.open_model_file(path.to_str().unwrap()), "{}", app.status_msg);
    assert!(!app.model.is_locked, "a file without results is unlocked");
    assert!(app.model.case_results.is_empty());
    assert_eq!(app.display_mode, DisplayMode::Undeformed);
    assert!(app.deform_auto && !app.animate_deformed);
    assert_eq!(app.active_mode_idx, 0);
    assert!(app.status_msg.starts_with("Opened "), "{}", app.status_msg);
    // The plan at a level the new model has stays; the pane keeps its plan.
    assert_eq!(app.right_proj, ViewProjection::PlanXY);
    let kept = app.right_plane.expect("the plane of a level of the new model stays");
    assert!((kept - levels[1]).abs() < 1e-9);

    // A plan at a coordinate the new model does not have: the whole model.
    let mut app = analysed_app_with_display_state(Some(levels[1] + 0.123));
    assert!(app.open_model_file(path.to_str().unwrap()));
    assert_eq!(app.right_plane, None);

    // Painted: the status bar says Unlocked and no result is named.
    let mut run = Run::new(SapModel::new(Units::KnMC));
    run.app = app;
    run.settle();
    assert!(run.has_text("🔓 Unlocked"), "{:?}", run.all_texts());
    assert!(!run.has_text_containing("has no results"), "{:?}", run.all_texts());
    let _ = std::fs::remove_file(&path);
}

#[test]
fn opening_a_file_that_cannot_be_read_changes_nothing() {
    let mut app = analysed_app_with_display_state(Some(3.0));
    let joints = app.model.points.len();
    assert!(!app.open_model_file("/nonexistent/p16/none.s2k"));
    assert!(app.status_msg.starts_with("Open error"), "{}", app.status_msg);
    assert!(app.model.is_locked);
    assert_eq!(app.model.points.len(), joints);
    assert!(matches!(app.display_mode, DisplayMode::ShellContour(_)));
    assert_eq!(app.right_plane, Some(3.0));
}

#[test]
fn a_planar_model_opens_the_2d_pane_on_its_plane() {
    // Four joints of a portal in the X-Z plane at Y = 2.5 m.
    let mut m = SapModel::new(Units::KnMC);
    for (i, (x, z)) in [(0.0, 0.0), (6.0, 0.0), (0.0, 4.0), (6.0, 4.0)].iter().enumerate() {
        m.add_point(format!("{}", i + 1), *x, 2.5, *z);
    }
    let path = temp_file("portal.s2k");
    save_s2k_file_with_report(&m, path.to_str().unwrap()).expect("save");
    // The right pane was a plan at Z = 0 (what the hand check saw): a line.
    let mut app = SapGuiApp::new(None);
    app.dual_pane = true;
    app.right_proj = ViewProjection::PlanXY;
    app.right_plane = Some(0.0);
    assert!(app.open_model_file(path.to_str().unwrap()), "{}", app.status_msg);
    assert_eq!(app.right_proj, ViewProjection::ElevationXZ);
    assert_eq!(app.right_plane, Some(2.5));
    assert_eq!(app.left_proj, ViewProjection::Perspective3D, "the 3D pane stays");
    let _ = std::fs::remove_file(&path);
}

#[test]
fn the_2d_frame_template_opens_its_2d_pane_on_the_frame_and_a_3d_template_does_not() {
    let mut app = SapGuiApp::new(None);
    app.dual_pane = true;
    app.right_proj = ViewProjection::PlanXY;
    app.right_plane = Some(0.0);
    for (choice, planar) in [(TemplateChoice::Frame2D, true), (TemplateChoice::Frame3D, false)] {
        app.tpl_choice = choice;
        let mut m = SapModel::new(Units::KipInF);
        app.generate_template(&mut m, 240.0, 144.0, 240.0);
        app.replace_model(m, "template".into(), Vec::new());
        if planar {
            // The frame lies in one plane (all joints at one Y): the elevation.
            let y = app.model.points.values().next().unwrap().y;
            assert!(app.model.points.values().all(|p| (p.y - y).abs() < 1e-9));
            assert_eq!(app.right_proj, ViewProjection::ElevationXZ);
            assert_eq!(app.right_plane, Some(y));
            // Back to the plan the hand check had, for the 3D template.
            app.right_proj = ViewProjection::PlanXY;
            app.right_plane = Some(0.0);
        } else {
            // A plan at Z = 0 is a level of a building: it stays.
            assert_eq!(app.right_proj, ViewProjection::PlanXY);
            assert_eq!(app.right_plane, Some(0.0));
        }
    }
}
