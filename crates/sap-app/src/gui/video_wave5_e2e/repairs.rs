//! Repairs of wave 5 seen through the application: Max and Min rows of a combination with a
//! response-spectrum case, the unstable joint named on the status line, a plane-strain area, an
//! orthotropic area with a material angle, rigid end zones, concrete design of a section made in
//! the dialog, and the eccentricity overwrite through save and reopen.
//!
//! Every model is analysed with Analyze > Run Analysis and read through a window or the status
//! line; the references are closed forms written out in the tests.

use super::ext::{analyze, menu};
use super::harness::{texts, Run};
use super::video::storey;
use crate::gui::{TableTab, ViewProjection};
use eframe::egui::Rect;
use sap_model::{
    ComboType, DynamicFunction, LoadCase, LoadCaseType, LoadCombo, LoadPattern, LoadPatternType,
    PointLoadForce, SapModel,
};

/// The single storey of `video.rs` with a static case LL (a lateral force that moves the roof by
/// 3.5 m) and a response-spectrum case QUAKE (that moves it by 2.0 m): the numbers of the manual's
/// worked example, Analysis Reference p. 353 (COMB1 = LL + QUAKE and COMB2 = envelope).
///
/// Closed forms (kN, m; k = 4 x 12 EI / h^3 = 101250 kN/m, omega^2 = k/M = 40.5 with M = 2500 t):
/// LL: H = 3.5 k = 354375 kN gives u = H / k = 3.5 m. QUAKE: a flat spectrum Sa = 81 m/s2 gives the
/// roof u = Sa / omega^2 = 81 / 40.5 = 2.0 m (all the mass takes part in the sway mode).
fn storey_with_spectrum() -> SapModel {
    let mut m = storey();
    m.load_cases.shift_remove("HIST");
    m.functions.shift_remove("STEP");
    m.load_patterns.insert(
        "LL".into(),
        LoadPattern { name: "LL".into(), pat_type: LoadPatternType::Live, self_wt_multiplier: 0.0 },
    );
    for j in ["T1", "T2", "T3", "T4"] {
        m.points.get_mut(j).unwrap().loads_force.push(PointLoadForce {
            load_pat: "LL".into(),
            csys: "GLOBAL".into(),
            force: [3.5 * 101250.0 / 4.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        });
    }
    m.load_cases.insert("LL".into(), LoadCase::linear_static("LL", "LL", 1.0));
    m.functions.insert(
        "SPEC".into(),
        DynamicFunction {
            name: "SPEC".into(),
            is_time_history: false,
            damping_ratio: 0.05,
            points: vec![(0.0, 81.0), (10.0, 81.0)],
        },
    );
    let mut q = LoadCase::linear_static("QUAKE", "LL", 1.0);
    q.case_type = LoadCaseType::ResponseSpectrum;
    q.loads.clear();
    q.initial_case = Some("MODAL".into());
    q.spectrum_loads = vec![("SPEC".into(), "U1".into(), 1.0)];
    m.load_cases.insert("QUAKE".into(), q);
    for (name, ty) in [("COMB1", ComboType::LinearAdd), ("COMB2", ComboType::Envelope)] {
        m.load_combos.insert(
            name.into(),
            LoadCombo { name: name.into(), combo_type: ty, cases: vec![("LL".into(), 1.0), ("QUAKE".into(), 1.0)] },
        );
    }
    m
}

/// The rectangle of the painted text `value` that sits on the same row as the painted text `tag`
/// ("Max" or "Min") of a table.
fn row_has(run: &Run, tag: &str, value: &str) -> bool {
    let all = texts(&run.last);
    let tags: Vec<Rect> = all.iter().filter(|(t, _)| t == tag).map(|(_, r)| *r).collect();
    all.iter().any(|(t, r)| t == value && tags.iter().any(|g| (g.center().y - r.center().y).abs() < 3.0 && g.left() < r.left()))
}

#[test]
fn a_combination_with_a_response_spectrum_case_shows_a_max_row_and_a_min_row_in_the_tables_window() {
    let mut run = Run::new(storey_with_spectrum());
    run.app.left_proj = ViewProjection::Perspective3D;
    analyze(&mut run);
    assert!(run.app.model.is_locked, "{}", run.app.status_msg);
    run.app.tables_tab = TableTab::JointDisplacements;
    menu(&mut run, &["Display", "Show Database Tables... (Ctrl+T)"]);
    // The cases: the static roof move 3.5 and the spectrum's 2.0 (printed with 5 decimals, so half
    // a unit of the last digit, 5e-6, is allowed).
    for (case, want) in [("LL", 3.5), ("QUAKE", 2.0)] {
        run.app.active_case = case.into();
        run.settle();
        let u = run.app.model.case_results[case].joint_displacements["T1"][0];
        assert!((u - want).abs() <= 5.0e-6, "{case}: roof U1 {u} against {want}");
        assert!(run.has_text(&format!("{want:.5}")), "{case}: {:?}", run.shown());
        assert!(!run.has_text("Step Type"), "a single-valued case has no Max/Min column");
    }
    // COMB1 = LL + QUAKE: 3.5 + 2.0 = 5.5 and 3.5 - 2.0 = 1.5 (p. 353).
    run.app.active_case = "COMB1".into();
    run.settle();
    assert!(run.has_text("Step Type"), "{:?}", run.shown());
    assert!(row_has(&run, "Max", "5.50000"), "Max row: {:?}", run.shown());
    assert!(row_has(&run, "Min", "1.50000"), "Min row: {:?}", run.shown());
    // COMB2 = envelope of LL and QUAKE: max(3.5, 2.0) = 3.5 and min(3.5, -2.0) = -2.0 (p. 353).
    run.app.active_case = "COMB2".into();
    run.settle();
    assert!(row_has(&run, "Max", "3.50000"), "Max row: {:?}", run.shown());
    assert!(row_has(&run, "Min", "-2.00000"), "Min row: {:?}", run.shown());
}
