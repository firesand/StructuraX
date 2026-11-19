//! Concrete design of a section made in the section dialog, and the eccentricity overwrite through
//! save to S2K and reopen with the file window.

use super::ext::unlock;
use super::harness::Run;
use crate::gui::CustomSectionShape;
use sap_model::{
    FrameLoadDistributed, FramePropType, FrameSection, LoadCase, LoadPattern, LoadPatternType,
    MatType, Material, SapModel,
};
use sap_units::Units;

/// A cantilever of 3 m (kN, m), a library-like concrete without a yield stress and a rebar material
/// with one, a uniform load of 20 kN/m on the cantilever. The frame is selected, as the dialog
/// assigns its section to the selected frames. Section TMP is replaced by the dialog's.
fn cantilever() -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    m.load_cases.clear();
    m.load_patterns.clear();
    m.frame_sections.clear();
    let mut conc = Material::default_steel_kip_in();
    conc.name = "CONC".into();
    conc.mat_type = MatType::Concrete;
    conc.e = 2.5e7;
    conc.u = 0.2;
    conc.g = 2.5e7 / 2.4;
    conc.fc = 30_000.0;
    conc.fy = 0.0;
    conc.fu = 0.0;
    conc.unit_weight = 0.0;
    conc.unit_mass = 0.0;
    m.materials.insert("CONC".into(), conc);
    let mut rebar = Material::default_steel_kip_in();
    rebar.name = "REBAR".into();
    rebar.mat_type = MatType::Rebar;
    rebar.e = 2.0e8;
    rebar.fy = 420_000.0;
    rebar.fu = 620_000.0;
    rebar.unit_weight = 0.0;
    rebar.unit_mass = 0.0;
    m.materials.insert("REBAR".into(), rebar);
    m.frame_sections.insert("TMP".into(), FrameSection { name: "TMP".into(), material: "CONC".into(), prop_type: FramePropType::General, area: 0.15, i33: 3.1e-3, i22: 1.1e-3, torsional_j: 1.0e-3, ..FrameSection::default() });
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 3.0, 0.0, 0.0);
    m.points.get_mut("1").unwrap().restraint = [true; 6];
    m.add_frame("F1", "1", "2", "TMP");
    m.frames.get_mut("F1").unwrap().selected = true;
    m.load_patterns.insert(
        "DEAD".into(),
        LoadPattern { name: "DEAD".into(), pat_type: LoadPatternType::Dead, self_wt_multiplier: 0.0 },
    );
    m.load_cases.insert("DEAD".into(), LoadCase::linear_static("DEAD", "DEAD", 1.0));
    m.frames.get_mut("F1").unwrap().loads_dist.push(FrameLoadDistributed {
        load_pat: "DEAD".into(),
        my_type: 1,
        dir: 6,
        csys: "GLOBAL".into(),
        rel_dist1: 0.0,
        rel_dist2: 1.0,
        val1: -20.0,
        val2: -20.0,
    });
    m
}

/// Sets the eccentricity overwrite of D1 to `distance` in the seismic window (its automatic tab is
/// open and scrolled to the bottom): the number is typed into the field left of "Set overwrite".
fn set_overwrite(run: &mut Run, distance: f64) {
    use eframe::egui;
    let button = run.find("Set overwrite");
    let field = egui::Pos2::new(button.left() - 25.0, button.center().y);
    run.click(field);
    let ctrl = egui::Modifiers { ctrl: true, command: true, ..Default::default() };
    let key = |key, pressed, modifiers| egui::Event::Key { key, physical_key: None, pressed, repeat: false, modifiers };
    run.frame(vec![key(egui::Key::A, true, ctrl), key(egui::Key::A, false, ctrl)]);
    run.frame(vec![egui::Event::Text(distance.to_string())]);
    run.frame(vec![key(egui::Key::Enter, true, Default::default()), key(egui::Key::Enter, false, Default::default())]);
    run.settle();
    run.click_text("Set overwrite");
}

#[test]
fn the_eccentricity_overwrite_survives_save_to_s2k_and_reopen_and_moves_the_moment() {
    use super::rows::{analyze_and_show_report, define_in_the_window, painted_moment, storey_forces, two_storeys};
    let mut run = Run::new(two_storeys());
    define_in_the_window(&mut run, "X", 0.05);
    // The overwrite of D1 (the lower diaphragm) to 0.4 m; D2 keeps ratio x width = 0.05 x 6 = 0.3 m.
    super::ext::scroll_window(&mut run, "Mass Source and Auto Seismic Loads", 4000.0);
    set_overwrite(&mut run, 0.4);
    assert_eq!(run.app.model.auto_seismic_eccentricity["EQ"]["D1"], 0.4, "{}", run.app.status_msg);
    // File > Open / Save, save as S2K, then reopen the file with the same window.
    let path = std::env::temp_dir().join(format!("structurax_w5e2e_{}_ecc.s2k", std::process::id()));
    let _ = std::fs::remove_file(&path);
    super::ext::menu(&mut run, &["File", "Open / Save Model (.s2k / .sdb)... (Ctrl+O)"]);
    run.app.file_path_input = path.display().to_string();
    run.settle();
    run.click_text("💾 Save as .S2K");
    assert!(run.app.status_msg.starts_with("Saved S2K to "), "{}", run.app.status_msg);
    assert!(path.is_file());
    // Lose the overwrite in the application, so that only the file can bring it back.
    run.app.model.auto_seismic_eccentricity.clear();
    super::ext::menu(&mut run, &["File", "Open / Save Model (.s2k / .sdb)... (Ctrl+O)"]);
    run.app.file_path_input = path.display().to_string();
    run.settle();
    run.click_text("📂 Open Model");
    assert!(run.app.status_msg.starts_with("Opened "), "{}", run.app.status_msg);
    let _ = std::fs::remove_file(&path);
    assert_eq!(
        run.app.model.auto_seismic_eccentricity.get("EQ").and_then(|t| t.get("D1")).copied(),
        Some(0.4),
        "the overwrite came back from the file"
    );
    // The reopened model analysed: the moment of D1 uses the overwrite, that of D2 the ratio.
    super::ext::analyze(&mut run);
    super::ext::menu(&mut run, &["Define", "Mass Source and Auto Seismic Loads..."]);
    run.click_text("Auto seismic (ASCE 7-10)");
    analyze_and_show_report(&mut run);
    let forces = storey_forces();
    for (d, e, f) in [("D1", 0.4, forces[0]), ("D2", 0.3, forces[1])] {
        let (_, _, pe, pm) = painted_moment(&run, d).unwrap_or_else(|| panic!("no line of {d}: {:?}", run.shown()));
        // X load: the couple about Z is -e F (project choice); four decimals are printed.
        assert!((pe - e).abs() <= 5.0e-5, "{d}: e {pe}");
        assert!((pm + e * f).abs() <= 5.0e-5, "{d}: Mz {pm} against {}", -e * f);
    }
    assert!(run.texts().iter().any(|t| t.contains("(overwrite)")), "{:?}", run.shown());
}

/// The dialog, with a rectangular 0.30 x 0.50 m section named `name` of the concrete CONC, with or
/// without the reinforcement metadata (the rebar material is the one the combo offers).
fn add_section(run: &mut Run, name: &str, rebar: bool) {
    run.app.custom_sec_material = "CONC".into();
    run.app.show_section_modal = true;
    run.app.custom_sec_name = name.into();
    run.app.custom_sec_shape = CustomSectionShape::Rectangular;
    (run.app.custom_sec_d, run.app.custom_sec_b) = (0.50, 0.30);
    run.app.custom_sec_cover = 0.05;
    run.settle();
    if rebar != run.app.custom_sec_rebar_enabled {
        run.click_text("Include Concrete Reinforcement Metadata");
    }
    assert_eq!(run.app.custom_sec_rebar_enabled, rebar);
    run.click_text("Add Custom Section (& Assign to Selected Frames)");
}

#[test]
fn concrete_design_of_a_dialog_section_with_a_rebar_material_runs_and_one_without_is_refused_by_name() {
    let mut run = Run::new(cantilever());
    // With the rebar material: the dialog's combo offers REBAR (the only material with a yield
    // stress) and the section stores it.
    add_section(&mut run, "RB1", true);
    let sec = run.app.model.frame_sections.get("RB1").unwrap_or_else(|| panic!("{}", run.app.status_msg)).clone();
    assert_eq!(sec.concrete_rebar.as_ref().map(|c| c.rebar_mat_long.as_str()), Some("REBAR"));
    assert_eq!(run.app.model.frames["F1"].section, "RB1");
    run.app.show_section_modal = false;
    run.settle();
    run.click_text("🧱 Concrete Design");
    assert!(run.app.status_msg.starts_with("Concrete design ("), "{}", run.app.status_msg);
    assert!(run.app.status_msg.contains("complete: 1 members"), "{}", run.app.status_msg);
    let result = &run.app.model.concrete_design_results["F1"];
    assert_eq!(result.section_name, "RB1");
    // The hogging moment at the fixed end is w L^2 / 2 = 20 x 9 / 2 = 90 kN-m of the DEAD case; the
    // factored demand is at least that, so the required top steel is at least Mu / (phi fy d) with
    // phi = 0.9, fy = 420000 kN/m2 (the rebar material's, not the concrete's 0) and d at most
    // 0.50 - 0.05 = 0.45 m: 90 / (0.9 x 420000 x 0.45) = 5.29e-4 m2. A lower bound, not the code's
    // value, which depends on the code's load factors and cover details.
    assert!(result.as_top >= 5.29e-4, "As top {} m2", result.as_top);
    // Without it: the same frame, a section without reinforcement metadata, whose concrete has no
    // yield stress. The design is refused and the message names the frame, the section and the
    // material (no yield stress is assumed).
    unlock(&mut run);
    assert!(!run.app.model.is_locked);
    add_section(&mut run, "RB0", false);
    assert_eq!(run.app.model.frames["F1"].section, "RB0", "{}", run.app.status_msg);
    run.app.show_section_modal = false;
    run.settle();
    run.click_text("🧱 Concrete Design");
    let msg = run.app.status_msg.clone();
    // D127: the refused member is recorded and named in the status message; the run completes.
    assert!(msg.starts_with("Concrete design (") && msg.contains("not designed (first: "), "{msg}");
    for part in ["'F1'", "'RB0'", "'CONC'", "yield stress"] {
        assert!(msg.contains(part), "the refusal must name {part}: {msg}");
    }
    assert!(run.app.model.concrete_design_results.is_empty(), "a refused member has no result");
    assert!(run.app.model.concrete_design_errors.contains_key("F1"));
}
