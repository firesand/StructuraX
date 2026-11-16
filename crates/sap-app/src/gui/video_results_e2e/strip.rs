//! Shell stress components and faces through the Shells form, on the
//! cantilever plate strip of the third wave (kN, m): 4 x 1 elements, L = 4,
//! t = 0.2, root fixed, nu = 0, pattern D: FX = 25 and MY = +5 on each of the two
//! tip joints. The model is built in code (not through the application's
//! commands), solved by the application's `execute_analysis`, and the numbers
//! the form shows are compared with the statics of the strip:
//!
//!   F11 = 2 x 25 / 1 m = 50 kN/m, M11 = -2 x 5 / 1 m = -10 kN-m/m (hogging)
//!   top    S11 = F11/t - 6 M11/t^2 = 250 + 1500 =  1750 kN/m2 =  1.75 N/mm2
//!   bottom S11 = F11/t + 6 M11/t^2 = 250 - 1500 = -1250 kN/m2 = -1.25 N/mm2
//!   (Analysis Reference Eqns. 2: positive moment, positive bottom stress);
//!   S22 = S12 = 0 (nu = 0, no shear), so SMAX bottom = 0, SMIN bottom = -1250 kN/m2.
//!   Pattern L: MY = -5 on each tip joint gives F11 = 0, M11 = +10, so its top
//!   S11 = -1500 kN/m2 and the SRSS of D and L has top S11 = sqrt(1750^2 + 1500^2)
//!   = 2304.886 kN/m2 (the combination rule applied to the stored top-face stress).

use super::super::contours;
use super::super::display_units::{self, Dimension};
use super::super::result_display::{self, no_contour_fill, Contour};
use super::super::shell_output::{self, Face, ShellQuantity, ShellStress};
use super::super::DisplayMode;
use super::harness::{close, polygon_fills, Run};
use sap_model::{
    AreaSection, ComboType, DynamicFunction, LoadCase, LoadCaseType, LoadCombo, LoadPattern,
    LoadPatternType, Material, PointLoadForce, SapModel,
};
use sap_units::Units;

const L: f64 = 4.0;
const T: f64 = 0.2;
const E: f64 = 2.5e7;

pub fn strip() -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.e = E;
    mat.u = 0.0;
    mat.g = E / 2.0;
    mat.unit_weight = 0.0;
    mat.unit_mass = 2.5;
    m.materials.insert("MAT".into(), mat);
    let mut s = AreaSection::default();
    s.name = "PL".into();
    s.material = "MAT".into();
    s.thickness_membrane = T;
    s.thickness_bending = T;
    m.area_sections.insert("PL".into(), s);
    m.load_cases.clear();
    m.load_patterns.clear();
    for name in ["D", "L"] {
        m.load_patterns.insert(
            name.into(),
            LoadPattern { name: name.into(), pat_type: LoadPatternType::Dead, self_wt_multiplier: 0.0 },
        );
        m.load_cases.insert(name.into(), LoadCase::linear_static(name, name, 1.0));
    }
    for i in 0..=4 {
        for (j, y) in [0.0, 1.0].iter().enumerate() {
            let name = format!("J{i}{j}");
            m.add_point(name.clone(), i as f64 * L / 4.0, *y, 0.0);
            if i == 0 {
                m.points[&name].restraint = [true; 6];
            }
            if i == 4 {
                for (pat, f) in
                    [("D", [25.0, 0., 0., 0., 5.0, 0.]), ("L", [0., 0., 0., 0., -5.0, 0.])]
                {
                    m.points[&name].loads_force.push(PointLoadForce {
                        load_pat: pat.into(),
                        csys: "GLOBAL".into(),
                        force: f,
                    });
                }
            }
        }
    }
    for i in 0..4 {
        m.add_area(
            format!("A{i}"),
            vec![format!("J{i}0"), format!("J{}0", i + 1), format!("J{}1", i + 1), format!("J{i}1")],
            "PL",
        );
    }
    // A response-spectrum case (needs mass and a modal case) for the dropped-results message.
    m.load_cases.insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 6));
    m.functions.insert(
        "SPEC".into(),
        DynamicFunction {
            name: "SPEC".into(),
            is_time_history: false,
            damping_ratio: 0.05,
            points: vec![(0.0, 1.0), (10.0, 1.0)],
        },
    );
    let mut rs = LoadCase::linear_static("RS", "D", 1.0);
    rs.case_type = LoadCaseType::ResponseSpectrum;
    rs.loads.clear();
    rs.initial_case = Some("MODAL".into());
    rs.spectrum_loads = vec![("SPEC".into(), "U1".into(), 1.0)];
    m.load_cases.insert("RS".into(), rs);
    for (name, ty, items) in [
        ("D+RS", ComboType::LinearAdd, vec![("D", 1.0), ("RS", 1.0)]),
        ("SRSS1", ComboType::Srss, vec![("D", 1.0), ("L", 1.0)]),
        ("D+L", ComboType::LinearAdd, vec![("D", 1.0), ("L", 1.0)]),
    ] {
        m.load_combos.insert(
            name.into(),
            LoadCombo {
                name: name.into(),
                combo_type: ty,
                cases: items.into_iter().map(|(n, f)| (n.to_string(), f)).collect(),
            },
        );
    }
    m
}

fn analysed() -> Run {
    let mut run = Run::new(strip());
    run.app.active_case = "D".into();
    assert!(run.app.execute_analysis(), "{}", run.app.status_msg);
    run.settle();
    run
}

/// Display > Show Forces/Stresses > Shells..., then the stress `component`
/// (label of the form), the face `face` (label), the range and the display
/// units N, mm, C, then OK. All through pointer clicks and typed text.
fn apply_stress(run: &mut Run, case: &str, component: &str, face: &str, range: (&str, &str)) {
    run.app.active_case = case.into();
    run.settle();
    run.click_text("Display");
    run.click_text("Show Forces/Stresses");
    run.click_text("Shells...");
    assert!(run.app.results.shells.open);
    run.click_text("Shell Stresses");
    run.click_text(component);
    run.click_text(face);
    // Always typed, so that a range of an earlier call is replaced or cleared.
    run.type_into("Min", range.0);
    run.type_into("Max", range.1);
    run.click_text("kN, m, C");
    run.click_text("N, mm, C");
    run.click_text("OK");
    assert!(!run.app.results.shells.open, "{}", run.app.results.shells.error);
    run.settle();
}

/// The ordinary (non-contour) fill of an unselected area in a plan view.
fn plain_fill() -> eframe::egui::Color32 {
    eframe::egui::Color32::from_rgba_unmultiplied(60, 120, 180, 55)
}

fn fill(value: f64, lo: f64, hi: f64) -> eframe::egui::Color32 {
    contours::value_color(value, lo, hi, result_display::CONTOUR_ALPHA)
}

#[test]
fn the_form_shows_s11_top_and_bottom_in_mpa_with_the_ticks_and_colours_of_the_range() {
    let mut run = analysed();
    // The statics of the strip, from the solver's stored resultants (independent
    // of the form): F11 = 50, M11 = -10.
    for a in ["A0", "A1", "A2", "A3"] {
        let ar = &run.app.model.case_results["D"].area_results[a];
        close("F11", ar.f11, 50.0, 1e-9);
        close("M11", ar.m11, -10.0, 1e-9);
    }
    // Top face, range -2 .. 3 N/mm2 typed in the form: 1.75 and -1.25 are then
    // mid-ramp values (0.75 and 0.15 of the range), not the clamped end colours.
    apply_stress(&mut run, "D", "S11", "Top Face", ("-2", "3"));
    assert_eq!(
        run.app.display_mode,
        DisplayMode::ShellContour(ShellQuantity::Stress(ShellStress::S11, Face::Top))
    );
    assert_eq!(
        run.app.results.contour,
        Contour::Shell { visible: false, range: (-2.0, 3.0), units: Units::NMmC }
    );
    // Painted: the title with the unit of the form (N/mm2 = MPa) and the end ticks
    // -2.000 and 3.000 (span 5: three decimals); every area has the top-face value
    // 1.75 = 0.75 of the range.
    assert!(run.has_text("Stress S11 (Top) [N/mm2]"), "{:?}", run.shown());
    assert!(run.has_text("-2.000") && run.has_text("3.000"), "{:?}", run.shown());
    let fills = polygon_fills(&run.last);
    let top = fill(1750.0, -2000.0, 3000.0);
    assert_eq!(fills.iter().filter(|c| **c == top).count(), 4, "{fills:?}");
    for end in [-2000.0, 3000.0] {
        assert_ne!(top, fill(end, -2000.0, 3000.0), "the top value is not at an end of the ramp");
    }
    assert!(!fills.contains(&fill(-1250.0, -2000.0, 3000.0)));
    // Bottom face: -1.25 N/mm2 = 0.15 of the range.
    apply_stress(&mut run, "D", "S11", "Bottom Face", ("-2", "3"));
    assert!(run.has_text("Stress S11 (Bottom) [N/mm2]"), "{:?}", run.shown());
    let fills = polygon_fills(&run.last);
    let bottom = fill(-1250.0, -2000.0, 3000.0);
    assert_ne!(bottom, top);
    assert_ne!(bottom, fill(-2000.0, -2000.0, 3000.0));
    assert_eq!(fills.iter().filter(|c| **c == bottom).count(), 4, "{fills:?}");
    // The values in N/mm2 from the pure function behind the legend: stored model
    // units kN/m2 times the application's display factor, which is the physical
    // constant 1e-3 (1 kN/m2 = 1e3 N / 1e6 mm2).
    let spec = result_display::contour_spec(
        run.app.display_mode,
        run.app.results.contour,
        run.app.model.present_units,
    )
    .unwrap();
    let fc =
        result_display::frame_contour(&run.app.model, "D", spec, Default::default(), [0.0, 0.0, 1.0]);
    for e in fc.table.entries.values() {
        let factor = display_units::factor(Dimension::Stress, Units::KnMC, Units::NMmC);
        close("factor kN/m2 -> N/mm2", factor, 1.0e-3, 1e-12);
        close("bottom S11 in N/mm2", e.as_ref().unwrap()[0] * factor, -1.25, 1e-9);
    }
    // The model is untouched: still kN, m.
    assert_eq!(run.app.model.present_units, Units::KnMC);
}

#[test]
fn smax_and_smin_on_the_bottom_face_are_zero_and_minus_1_25_mpa() {
    let mut run = analysed();
    // Principal stresses of (s11, s22, s12) = (-1250, 0, 0) on the bottom face:
    // Mohr's circle gives 0 and -1250 kN/m2.
    let m = &run.app.model;
    for a in ["A0", "A1", "A2", "A3"] {
        let v = |q| shell_output::area_value(m, "D", a, q).unwrap();
        let tol = 1e-9 * 1750.0;
        assert!(v(ShellQuantity::Stress(ShellStress::SMax, Face::Bottom)).abs() <= tol);
        assert!((v(ShellQuantity::Stress(ShellStress::SMin, Face::Bottom)) + 1250.0).abs() <= tol);
        assert!((v(ShellQuantity::Stress(ShellStress::Svm, Face::Bottom)) - 1250.0).abs() <= tol);
    }
    // Through the form, range -2 .. 1: SMIN bottom (-1.25) is a quarter up the scale,
    // SMAX bottom (0) two thirds up.
    apply_stress(&mut run, "D", "SMIN", "Bottom Face", ("-2", "1"));
    assert!(run.has_text("Stress SMIN (Bottom) [N/mm2]"), "{:?}", run.shown());
    assert!(run.has_text("-2.000") && run.has_text("1.000"), "{:?}", run.shown());
    let fills = polygon_fills(&run.last);
    assert_eq!(fills.iter().filter(|c| **c == fill(-1250.0, -2000.0, 1000.0)).count(), 4);
    apply_stress(&mut run, "D", "SMAX", "Bottom Face", ("-2", "1"));
    assert!(run.has_text("Stress SMAX (Bottom) [N/mm2]"), "{:?}", run.shown());
    let fills = polygon_fills(&run.last);
    assert_eq!(fills.iter().filter(|c| **c == fill(0.0, -2000.0, 1000.0)).count(), 4);
}

#[test]
fn a_combination_with_a_response_spectrum_case_drops_its_shell_results_and_says_why() {
    let mut run = analysed();
    // Source of the rule: the solver reports shell results of a combination only
    // when every case does; the response-spectrum case RS has none.
    assert!(run.app.model.case_results["RS"].area_results.is_empty());
    assert!(run.app.model.case_results["D+RS"].area_results.is_empty());
    assert!(!run.app.model.case_results["D+L"].area_results.is_empty());
    apply_stress(&mut run, "D+RS", "S11", "Top Face", ("", ""));
    // The legend area paints the message of `shell_output::missing_results_message`.
    let want = "'D+RS' has no shell results: case 'RS' (ResponseSpectrum) carries no shell results";
    assert!(run.has_text(want), "{:?}", run.shown());
    assert!(run.has_text("Stress S11 (Top) [N/mm2]"));
    // No area is coloured by a ramp: no mesh and no ramp fill, because there is no range.
    assert!(!polygon_fills(&run.last).contains(&fill(1750.0, -1250.0, 1750.0)));
    // The combination without the spectrum case still has them: D + L has
    // F11 = 50, M11 = -10 + 10 = 0: top S11 = bottom S11 = 250 kN/m2 = 0.25 N/mm2.
    apply_stress(&mut run, "D+L", "S11", "Top Face", ("", ""));
    assert!(run.text_starting("'D+L' has no").is_none(), "{:?}", run.shown());
    let m = &run.app.model;
    close(
        "D+L top S11",
        shell_output::area_value(m, "D+L", "A1", ShellQuantity::Stress(ShellStress::S11, Face::Top))
            .unwrap(),
        250.0,
        1e-9,
    );
}

#[test]
fn smax_of_an_srss_combination_is_refused_in_the_legend_and_s11_top_follows_the_srss_rule() {
    let mut run = analysed();
    // SMAX needs a single-valued result (Analysis Reference p. 214).
    apply_stress(&mut run, "SRSS1", "SMAX", "Top Face", ("", ""));
    let note = run
        .texts()
        .into_iter()
        .find(|t| t.starts_with("4 area(s)"))
        .unwrap_or_else(|| panic!("no refusal in {:?}", run.shown()));
    assert!(note.contains("SMAX") && note.contains("'SRSS1'") && note.contains("multi-valued"), "{note}");
    assert!(note.contains("p. 214"), "{note}");
    // With no value at all there is no range: the four areas get the fill of an
    // area without a value, no ramp colour is painted, and the legend carries the
    // reason.
    let fills = polygon_fills(&run.last);
    assert_eq!(fills.iter().filter(|c| **c == no_contour_fill()).count(), 4, "{fills:?}");
    assert!(!fills.contains(&fill(0.0, -1.0, 1.0)) && !fills.contains(&plain_fill()));
    // A bottom-face stress is refused the same way, but the stored top-face S11 is
    // available: sqrt(1750^2 + 1500^2) kN/m2 = 2.304886 N/mm2.
    apply_stress(&mut run, "SRSS1", "S11", "Bottom Face", ("", ""));
    assert!(run.text_starting("4 area(s)").is_some_and(|n| n.contains("multi-valued")));
    apply_stress(&mut run, "SRSS1", "S11", "Top Face", ("-1", "3"));
    assert!(run.text_starting("4 area(s)").is_none(), "{:?}", run.shown());
    let want = (1750.0_f64.powi(2) + 1500.0_f64.powi(2)).sqrt();
    close("SRSS top S11", want, 2304.886114, 1e-9);
    let got = shell_output::area_value(
        &run.app.model,
        "SRSS1",
        "A2",
        ShellQuantity::Stress(ShellStress::S11, Face::Top),
    )
    .unwrap();
    close("SRSS top S11 of the application", got, want, 1e-9);
    // Range -1 .. 3: ticks -1.000 .. 3.000 and the colour of 2.3049.
    assert!(run.has_text("-1.000") && run.has_text("3.000"), "{:?}", run.shown());
    let fills = polygon_fills(&run.last);
    assert_eq!(fills.iter().filter(|c| **c == fill(want, -1000.0, 3000.0)).count(), 4, "{fills:?}");
}
