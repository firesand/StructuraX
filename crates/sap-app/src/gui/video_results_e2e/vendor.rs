//! Opt-in vendor tests (`STRUCTURAX_VENDOR_FIXTURES`, skipped silently without
//! the corpus): Example 2-010 (cylinder under internal pressure) through the
//! Shells form and the joint read-out, Example 1-022 (the frame with seven
//! modes) through the modal tables window.
//!
//! Independent sides: the printed values of `problem_2_010.pdf` (sigma11 58.85 and
//! 59.87 k/in2, U1 0.12175 and 0.12387 in) and of `problem_1_022.pdf` (the mass
//! ratios 0.800 ... 0.003 and the seven periods), and the closed form
//! sigma = p R cos(pi/N) / t, U1 = R sigma / E (R = 60 in, t = 1 in,
//! E = 29000 k/in2, p = 1 k/in2, N = 16 and 48), which reproduces all four
//! printed values: 58.8471, 59.8715 k/in2 and 0.121753, 0.123872 in.

use super::super::modal_tables;
use super::super::result_display;
use super::super::shell_output::{self, Face, ShellQuantity, ShellStress};
use super::cylinder::KSI_TO_MPA;
use super::harness::{close, Run};
use sap_model::SapModel;
use std::path::PathBuf;

fn vendor_dir() -> Option<PathBuf> {
    std::env::var_os("STRUCTURAX_VENDOR_FIXTURES").map(PathBuf::from)
}

fn load(file: &str) -> Option<SapModel> {
    let path = vendor_dir()?.join(file);
    if !path.is_file() {
        eprintln!("skipped: {} not found", path.display());
        return None;
    }
    Some(sap_format::load_sdb_file(&path).unwrap_or_else(|e| panic!("{file}: {e}")))
}

#[test]
fn vendor_example_2_010_through_the_shells_form_and_the_read_out() {
    let pi = std::f64::consts::PI;
    let cases = [
        ("example_2_010a_thin.sdb", 16.0, 58.85, 0.12175),
        ("example_2_010a_thick.sdb", 16.0, 58.85, 0.12175),
        ("example_2_010b_thin.sdb", 48.0, 59.87, 0.12387),
        ("example_2_010b_thick.sdb", 48.0, 59.87, 0.12387),
    ];
    for (file, n, printed_sigma, printed_u1) in cases {
        let Some(model) = load(file) else { continue };
        let sigma = 60.0 * (pi / n).cos();
        let u1 = 60.0 * sigma / 29_000.0;
        let mut run = Run::new(model);
        assert_eq!(run.app.model.present_units, sap_units::Units::KipInF, "{file}");
        assert!(run.app.execute_analysis(), "{file}: {}", run.app.status_msg);
        let case = run.app.model.case_results.keys().next().unwrap().clone();
        run.app.active_case = case.clone();
        run.settle();
        // The values of the form: top-face S11 of every area, against the printed
        // number (two decimals) and the closed form.
        let q = ShellQuantity::Stress(ShellStress::S11, Face::Top);
        let m = &run.app.model;
        for area in m.areas.keys() {
            let v = shell_output::area_value(m, &case, area, q).unwrap();
            assert!((v - printed_sigma).abs() <= 0.005, "{file} {area}: {v} against the printed {printed_sigma}");
            close(&format!("{file} {area} S11"), v, sigma, 1e-6);
        }
        // The form, through pointer clicks: S11, top face, display units N, mm, C.
        run.click_text("Display");
        run.click_text("Show Forces/Stresses");
        run.click_text("Shells...");
        run.click_text("Shell Stresses");
        run.click_text("S11");
        run.click_text("Top Face");
        run.click_text("Kip, in, F");
        run.click_text("N, mm, C");
        run.click_text("OK");
        run.settle();
        assert!(run.has_text("Stress S11 (Top) [N/mm2]"), "{file}: {:?}", run.shown());
        // The legend range of the uniform field is the single value, in MPa.
        let tick = format!("{:.3e}", sigma * KSI_TO_MPA);
        assert!(run.has_text(&tick), "{file}: tick {tick} in {:?}", run.shown());
        // Read-out of a joint at mid height: U1 in the joint's local axis 1 (radial).
        let joint = run
            .app
            .model
            .points
            .values()
            .find(|p| (p.z - 100.0).abs() < 1e-6 && !p.restraint[0])
            .unwrap_or_else(|| panic!("{file}: no free joint at mid height"))
            .name
            .clone();
        let (header, lines) = result_display::readout_lines(
            &run.app.model,
            &case,
            &joint,
            Default::default(),
            run.app.model.present_units,
        );
        assert!(header.contains(&joint));
        let shown: f64 = lines.iter().find(|(l, _)| l == "U1 [in]").unwrap().1.parse().unwrap();
        assert!((shown - u1).abs() <= 5.0e-7, "{file}: read-out {shown} against {u1}");
        assert!((shown - printed_u1).abs() <= 5.0e-6, "{file}: read-out {shown} against the printed {printed_u1}");
        // U3 at the top (z = 200): the Poisson contraction -nu sigma H / E = -0.12175 and
        // -0.12387 in, which the vendor prints as well.
        let top = run
            .app
            .model
            .points
            .values()
            .find(|p| (p.z - 200.0).abs() < 1e-6 && !p.restraint[2])
            .unwrap_or_else(|| panic!("{file}: no free joint at the top"))
            .name
            .clone();
        let (_, top_lines) = result_display::readout_lines(
            &run.app.model,
            &case,
            &top,
            Default::default(),
            run.app.model.present_units,
        );
        let u3: f64 = top_lines.iter().find(|(l, _)| l == "U3 [in]").unwrap().1.parse().unwrap();
        let want_u3 = -0.3 * sigma * 200.0 / 29_000.0;
        assert!((u3 - want_u3).abs() <= 5.0e-7, "{file}: U3 at the top {u3} against {want_u3}");
        assert!((u3 + printed_u1).abs() <= 5.0e-6, "{file}: U3 at the top {u3} against the printed {}", -printed_u1);
        // Painted by the floating window.
        run.app.results.readout_units = None;
        run.app.display_mode = super::super::DisplayMode::Deformed;
        run.app.results.open_readout(&joint, false).unwrap();
        run.settle();
        assert!(run.has_text("Joint Displacements"), "{file}");
        assert!(run.has_text(&format!("{shown:.6}")), "{file}: {:?}", run.shown());
    }
}

#[test]
fn vendor_example_1_022_through_the_modal_tables_window() {
    let Some(mut m) = load("example_1_022.sdb") else { return };
    // The documented change of the solver's own test of this example
    // (`vendor_example_1_022_modal_time_history_against_the_published_values`): only
    // MODAL runs, the penalty diaphragms are replaced by an axial-area modifier of
    // 1e6 on the beam sections (a Modal case with the imported penalty constraints
    // is refused by the eigen solver).
    for name in ["LAT", "SPECSRSS", "SPECCQC", "MHIST1", "DHIST1", "MHIST2", "DHIST2"] {
        m.load_cases[name].run_flag = false;
    }
    m.constraints.clear();
    for point in m.points.values_mut() {
        point.constraints.clear();
    }
    for beam in ["W24X110", "W24X130", "W24X160"] {
        m.frame_sections[beam].modifiers[0] = 1.0e6;
    }
    let mut run = Run::new(m);
    assert!(run.app.execute_analysis(), "{}", run.app.status_msg);
    run.settle();
    let cr = &run.app.model.case_results["MODAL"];
    // Published (problem_1_022.pdf): participating mass ratios and periods of the 7 modes.
    let mass = [0.800, 0.113, 0.042, 0.021, 0.014, 0.007, 0.003];
    let periods = [1.2732, 0.4313, 0.2420, 0.1602, 0.1190, 0.0951, 0.0795];
    assert_eq!(cr.modes.len(), 7);
    let rows = modal_tables::mass_ratio_rows("MODAL", cr);
    for (i, row) in rows.iter().enumerate() {
        assert!((row.ratio[0] - mass[i]).abs() < 6e-4, "UX of mode {}: {}", i + 1, row.ratio[0]);
        assert!((row.period - periods[i]).abs() < 6e-5, "T{}: {}", i + 1, row.period);
    }
    // All seven modes of the seven-storey shear building: the whole mass.
    close("SumUX", rows[6].cumulative[0], 1.0, 1e-9);
    let items = modal_tables::item_rows("MODAL", cr);
    let ux = items.iter().find(|r| r.item == "UX").unwrap();
    close("UX dynamic %", ux.dynamic_percent, 100.0, 1e-7);
    // The window, opened as the Analyze menu opens it: the vendor field names,
    // the totals line, and the cells of every mode (period, UX) as painted.
    run.app.panel_modal_tables.open = true;
    run.settle();
    assert!(run.has_text("Modal Results Tables"));
    for field in modal_tables::MASS_RATIO_FIELDS {
        assert!(run.has_text(field), "field {field}");
    }
    assert!(run.has_text(&modal_tables::mass_ratio_totals(&rows)), "{:?}", run.shown());
    for (i, row) in rows.iter().enumerate() {
        let cells = modal_tables::mass_ratio_cells(row, 5);
        assert!(run.has_text(&cells[3]), "period of mode {}", i + 1);
        assert!(run.has_text(&cells[4]), "UX of mode {}", i + 1);
    }
    assert!(run.texts().iter().any(|t| t.contains("UX 100.00 %")), "{:?}", run.shown());
}
