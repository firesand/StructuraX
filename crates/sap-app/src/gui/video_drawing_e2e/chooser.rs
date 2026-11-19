//! Test D: one path from the file chooser to a drawn frame. The import
//! window's Browse opens the chooser, a click chooses the `.pro` file, two
//! shapes are selected with a click and a Ctrl+click, "Import selected" runs,
//! and one of the shapes is the section of a frame drawn with PolyFrame.

use super::{frame, grid_app, key, pane};
use crate::gui::{SapGuiApp, ToolMode, ViewProjection};
use eframe::egui::{self, Modifiers, Pos2, Rect, Vec2};
use sap_section::ProDatabase;
use sap_units::Units;

/// A synthetic `.pro` file: header, then one record per entry (as the tests
/// of `section_import.rs` build one; unit code 1 is inches).
fn pro_bytes(records: &[(&str, &str, f32)]) -> Vec<u8> {
    let mut bytes = vec![b' '; 120 * (records.len() + 1)];
    for (offset, value) in [(40, 7.0_f32), (44, 1.0_f32), (48, records.len() as f32)] {
        bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }
    for (n, (name, shape, scale)) in records.iter().enumerate() {
        let base = 120 * (n + 1);
        bytes[base..base + name.len()].copy_from_slice(name.as_bytes());
        bytes[base + 36..base + 36 + shape.len()].copy_from_slice(shape.as_bytes());
        let mut values = [0.0_f32; 20];
        values[0] = 10.0 * scale;
        values[1] = 5.0 * scale;
        values[2] = 0.5 * scale;
        values[3] = 0.25 * scale;
        values[5] = 12.0 * scale * scale;
        values[6] = 3.0 * scale.powi(4);
        values[7] = 200.0 * scale.powi(4);
        values[8] = 50.0 * scale.powi(4);
        values[9] = 4.0 * scale * scale;
        values[10] = 5.0 * scale * scale;
        values[11] = 40.0 * scale.powi(3);
        values[12] = 15.0 * scale.powi(3);
        for (k, v) in values.iter().enumerate() {
            let at = base + 40 + 4 * k;
            bytes[at..at + 4].copy_from_slice(&v.to_le_bytes());
        }
    }
    bytes
}

fn frame_out(
    app: &mut SapGuiApp,
    ctx: &egui::Context,
    time: &mut f64,
    modifiers: Modifiers,
    events: Vec<egui::Event>,
) -> egui::FullOutput {
    *time += 0.3;
    let input = egui::RawInput {
        screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1800.0, 1400.0))),
        time: Some(*time),
        modifiers,
        events,
        ..Default::default()
    };
    ctx.run(input, |ctx| app.render_ui(ctx))
}

fn text_center(out: &egui::FullOutput, label: &str) -> Option<Pos2> {
    out.shapes.iter().find_map(|shape| match &shape.shape {
        egui::epaint::Shape::Text(t) if t.galley.job.text == label => {
            Some(t.pos + t.galley.size() * 0.5)
        }
        _ => None,
    })
}

/// A real pointer click on the text `label` of the window that shows it.
fn click_text(
    app: &mut SapGuiApp,
    ctx: &egui::Context,
    time: &mut f64,
    label: &str,
    modifiers: Modifiers,
) {
    let out = frame_out(app, ctx, time, Modifiers::NONE, Vec::new());
    let pos = text_center(&out, label).unwrap_or_else(|| panic!("no '{label}' drawn"));
    let event = |pressed| egui::Event::PointerButton {
        pos,
        button: egui::PointerButton::Primary,
        pressed,
        modifiers,
    };
    frame_out(app, ctx, time, modifiers, vec![egui::Event::PointerMoved(pos), event(true)]);
    frame_out(app, ctx, time, modifiers, vec![event(false)]);
    frame_out(app, ctx, time, Modifiers::NONE, Vec::new());
    *time += 1.0;
}

#[test]
fn a_shape_chosen_through_browse_is_imported_and_becomes_the_section_of_a_drawn_frame() {
    let dir = std::env::temp_dir().join(format!("structurax_p7_chooser_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let bytes = pro_bytes(&[("IPE200", "W", 1.0), ("IPE220", "W", 1.1), ("UPN100", "C", 0.9)]);
    let file = dir.join("video_shapes.pro");
    std::fs::write(&file, &bytes).unwrap();

    let (mut app, ctx, mut time) = grid_app(ViewProjection::PlanXY, 0.0, ToolMode::Select);
    // The window opens from the Edit-menu entry; Browse starts in the folder
    // of the typed path, so the test does not depend on the working directory.
    app.panel_section_import.open = true;
    app.panel_section_import.path = dir.display().to_string();
    frame(&mut app, &ctx, &mut time, Vec::new());

    click_text(&mut app, &ctx, &mut time, "Browse...", Modifiers::NONE);
    click_text(&mut app, &ctx, &mut time, "video_shapes.pro", Modifiers::NONE);
    click_text(&mut app, &ctx, &mut time, "Choose", Modifiers::NONE);
    // The next frame takes the choice and loads the file.
    frame(&mut app, &ctx, &mut time, Vec::new());
    assert!(app.panel_section_import.path.ends_with("video_shapes.pro"));
    let db = app.panel_section_import.db.as_ref().expect("the chosen file is loaded");
    assert_eq!(db.sections.len(), 3, "{}", app.status_msg);

    // Two shapes: a click and a Ctrl+click.
    click_text(&mut app, &ctx, &mut time, "IPE220", Modifiers::NONE);
    click_text(&mut app, &ctx, &mut time, "UPN100", Modifiers::CTRL);
    assert_eq!(app.panel_section_import.selected.len(), 2);
    click_text(&mut app, &ctx, &mut time, "Import selected", Modifiers::NONE);
    assert!(app.status_msg.contains("2 section(s)"), "{}", app.status_msg);
    assert!(app.model.frame_sections.contains_key("IPE220"));
    assert!(app.model.frame_sections.contains_key("UPN100"));
    assert!(!app.model.frame_sections.contains_key("IPE200"), "only the selected shapes");
    let material = app.panel_section_import.material.clone();
    assert!(app.model.materials.contains_key(&material));

    // Close the windows, then draw a frame with the imported shape.
    app.panel_section_import.open = false;
    app.draw_tools.frame_section = "UPN100".into();
    app.left_proj = ViewProjection::PlanXY;
    app.left_plane = Some(0.0);
    app.tool_mode = ToolMode::PolyFrame;
    for _ in 0..2 {
        frame(&mut app, &ctx, &mut time, Vec::new());
    }
    let (view, _) = pane(&app);
    for c in [[0.0, 0.0], [6.0, 0.0]] {
        let at = view.project([c[0], c[1], 0.0]);
        super::click(&mut app, &ctx, &mut time, at);
    }
    key(&mut app, &ctx, &mut time, egui::Key::Enter);
    assert_eq!(app.model.frames.len(), 1, "{}", app.status_msg);
    let drawn = app.model.frames.values().next().unwrap();
    assert_eq!(drawn.section, "UPN100");

    // Its section is the single-shape scaled read of the same file.
    let single = ProDatabase::from_bytes(&bytes, "video_shapes.pro".into())
        .unwrap()
        .get_section_scaled("UPN100", Units::KnMC, &material)
        .unwrap();
    let section = &app.model.frame_sections[&drawn.section];
    assert_eq!(section.name, single.name);
    assert_eq!(section.area.to_bits(), single.area.to_bits());
    // 12 * 0.9^2 in2 in m2: the file's area scaled by the inch.
    assert!((section.area - 12.0 * 0.81 * 0.0254 * 0.0254).abs() <= 1e-6 * section.area);
    let _ = std::fs::remove_dir_all(&dir);
}
