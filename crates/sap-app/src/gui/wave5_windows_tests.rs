//! Tests of the application windows of wave 5, package P6: the automatic scale
//! and the view title (D32, D33), the Max/Min rows of the result tables and the
//! frame diagram of a multi-valued combination, the message of an empty table
//! (D15), the unequal-flange I-section dialog (D18), the Object Information
//! window (G07), Grid Only (G36) and the Create Video menu.
//!
//! The application is run through `render_ui` with synthetic pointer input; the
//! checked values are the strings and shapes it paints and the model it
//! changes. The independent side of each comparison is named in the test.

use super::contours::StepSelection;
use super::result_display::scaling;
use super::video_export::{self, FramePose, VideoKind, VideoSettings};
use super::video_menu;
use super::*;
use eframe::egui::{epaint::Shape, Pos2, Rect};
use sap_model::{
    AreaResult, ComboType, FrameLoadDistributed, FrameLoadPoint, FrameStationResult, LoadCase,
    LoadCaseType, LoadCombo, ModalModeResult, PointLoadForce,
};
use std::path::PathBuf;

pub(super) const SCREEN: Vec2 = Vec2::new(1800.0, 1400.0);

pub(super) fn flat(out: &egui::FullOutput) -> Vec<Shape> {
    fn walk(s: &Shape, into: &mut Vec<Shape>) {
        match s {
            Shape::Vec(v) => v.iter().for_each(|x| walk(x, into)),
            other => into.push(other.clone()),
        }
    }
    let mut all = Vec::new();
    for c in &out.shapes {
        walk(&c.shape, &mut all);
    }
    all
}

pub(super) fn texts(out: &egui::FullOutput) -> Vec<(String, Rect)> {
    flat(out)
        .into_iter()
        .filter_map(|s| match s {
            Shape::Text(t) => Some((t.galley.job.text.clone(), Rect::from_min_size(t.pos, t.galley.size()))),
            _ => None,
        })
        .collect()
}

pub(super) struct Run {
    pub(super) app: SapGuiApp,
    pub(super) ctx: egui::Context,
    pub(super) time: f64,
    pub(super) last: egui::FullOutput,
    pub(super) screen: Vec2,
}

impl Run {
    pub(super) fn new(model: SapModel) -> Self {
        let mut app = SapGuiApp::new(Some(PathBuf::from("/nonexistent/wave5-windows.s2k")));
        app.model = model;
        app.dual_pane = false;
        app.left_proj = ViewProjection::PlanXY;
        app.left_plane = None;
        let mut run = Self { app, ctx: egui::Context::default(), time: 0.0, last: Default::default(), screen: SCREEN };
        run.settle();
        run
    }

    pub(super) fn frame(&mut self, events: Vec<egui::Event>) {
        self.time += 0.05;
        let input = egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, self.screen)),
            time: Some(self.time),
            events,
            ..Default::default()
        };
        self.last = self.ctx.run(input, |ctx| self.app.render_ui(ctx));
    }

    pub(super) fn settle(&mut self) {
        for _ in 0..3 {
            self.frame(vec![]);
        }
    }

    pub(super) fn click_at(&mut self, pos: Pos2, b: egui::PointerButton) {
        let ev = |pressed| egui::Event::PointerButton { pos, button: b, pressed, modifiers: Default::default() };
        self.frame(vec![egui::Event::PointerMoved(pos), ev(true)]);
        self.frame(vec![ev(false)]);
        self.time += 1.0;
        self.settle();
    }

    pub(super) fn click(&mut self, pos: Pos2) {
        self.click_at(pos, egui::PointerButton::Primary);
    }

    pub(super) fn right_click(&mut self, pos: Pos2) {
        self.click_at(pos, egui::PointerButton::Secondary);
    }

    pub(super) fn all_texts(&self) -> Vec<String> {
        texts(&self.last).into_iter().map(|t| t.0).collect()
    }

    pub(super) fn has_text(&self, label: &str) -> bool {
        self.all_texts().iter().any(|t| t == label)
    }

    pub(super) fn has_text_containing(&self, part: &str) -> bool {
        self.all_texts().iter().any(|t| t.contains(part))
    }

    pub(super) fn find(&self, label: &str) -> Rect {
        texts(&self.last)
            .into_iter()
            .find(|(t, _)| t == label)
            .unwrap_or_else(|| panic!("no text '{label}' in {:?}", self.all_texts()))
            .1
    }

    pub(super) fn click_text(&mut self, label: &str) {
        let r = self.find(label);
        self.click(r.center());
    }

    pub(super) fn screen(&self, p: [f64; 3]) -> Pos2 {
        self.app.last_view[0].expect("pane drawn").0.project(p)
    }
}

pub(super) fn close(what: &str, got: f64, want: f64, rel: f64) {
    assert!(
        (got - want).abs() <= rel * want.abs().max(1e-300),
        "{what}: got {got:e}, want {want:e}"
    );
}

// ------------------------------------------------------------ D32 and D33

/// Two joints 3-4-0 apart (diagonal 5 m) and a modal result whose first mode
/// moves joint 2 by (30, 0, 40) m: a mass-normalised mode of a light model.
pub(super) fn modal_model() -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    m.load_cases.clear();
    m.load_cases.insert("M".into(), LoadCase::linear_static("M", "D", 1.0));
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 3.0, 4.0, 0.0);
    let mut cr = CaseResults { case_name: "M".into(), ..Default::default() };
    for (n, shape) in [(1, [30.0, 0.0, 40.0, 0.0, 0.0, 0.0]), (2, [3.0, 0.0, 4.0, 0.0, 0.0, 0.0])] {
        cr.modes.push(ModalModeResult {
            mode_num: n,
            period_sec: 0.5 / n as f64,
            freq_hz: 2.0 * n as f64,
            mode_shape: [("2".to_string(), shape)].into_iter().collect(),
            ..ModalModeResult::default()
        });
    }
    m.case_results.insert("M".into(), cr);
    m
}

#[test]
fn the_automatic_scale_keeps_a_large_mode_on_the_screen_and_a_typed_factor_wins() {
    let mut run = Run::new(modal_model());
    run.app.active_case = "M".into();
    run.app.display_mode = DisplayMode::Deformed;
    run.settle();
    // Closed form: the largest translation is |(30, 0, 40)| = 50 m, the
    // diagonal 5 m, so the factor is 0.1 * 5 / 50 = 0.01 (a project fraction).
    assert!(run.app.deform_auto);
    close("auto factor", f64::from(run.app.deform_scale), 0.01, 1e-6);
    // The joint is drawn 0.5 m from where it was, not 50 * 25 m.
    close("drawn", f64::from(run.app.deform_scale) * 50.0, scaling::AUTO_FRACTION * 5.0, 1e-6);
    // Mode 2 has a tenth of the amplitude: ten times the factor.
    run.app.active_mode_idx = 1;
    run.settle();
    close("mode 2", f64::from(run.app.deform_scale), 0.1, 1e-6);
    // A factor the user sets (the slider writes the same field) is user-defined.
    run.app.deform_scale = 7.0;
    run.settle();
    assert!(!run.app.deform_auto);
    assert_eq!(run.app.deform_scale, 7.0);
    // The form: Auto selected again, OK (the form opens with the mode on
    // screen, mode 2), and the factor follows the result.
    run.click_text("Display");
    run.click_text("Show Deformed Shape...");
    assert!(run.app.results.deformed.open && !run.app.results.deformed.auto);
    assert!(run.has_text("Scaling") && run.has_text("Auto") && run.has_text("Scale Factor"));
    run.click_text("Auto");
    assert!(run.app.results.deformed.auto);
    run.settle();
    run.click_text("OK");
    assert!(run.app.deform_auto);
    assert_eq!(run.app.active_mode_idx, 1);
    close("after the form", f64::from(run.app.deform_scale), 0.1, 1e-6);
}

#[test]
fn the_view_title_names_the_mode_with_its_period_and_frequency() {
    let mut run = Run::new(modal_model());
    run.app.active_case = "M".into();
    run.app.display_mode = DisplayMode::Deformed;
    run.app.active_mode_idx = 1;
    run.settle();
    assert!(
        run.has_text_containing("[Result: M]  Mode 2, Period = 0.25000 s, Frequency = 4.0000 Hz"),
        "{:?}",
        run.all_texts()
    );
    // Not a deformed shape: the title names the case only.
    run.app.display_mode = DisplayMode::Undeformed;
    run.settle();
    assert!(!run.has_text_containing("Mode 2, Period"));
    assert!(run.has_text_containing("[Result: M]"));
}

// ------------------------------------------------------ Max/Min and D15

fn area_result(f11: f64, m11: f64, svm: f64) -> AreaResult {
    AreaResult { f11, m11, svm_top: svm, ..AreaResult::default() }
}

/// An area with a combination "COMB" whose maximum has F11 = 7 and minimum
/// F11 = -4 (as D + RS and D - RS with D = 1.5 and RS = 5.5 would).
fn combo_model() -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    m.load_cases.clear();
    m.load_cases.insert("D".into(), LoadCase::linear_static("D", "D", 1.0));
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 6.0, 0.0, 0.0);
    m.add_point("3", 6.0, 6.0, 0.0);
    m.add_point("4", 0.0, 6.0, 0.0);
    m.add_area("A1", ["1", "2", "3", "4"].map(String::from).to_vec(), "SH");
    m.add_frame("F1", "1", "2", "FS");
    m.load_combos.insert(
        "COMB".into(),
        LoadCombo { name: "COMB".into(), combo_type: ComboType::Envelope, cases: vec![("D".into(), 1.0)] },
    );
    let mut d = CaseResults { case_name: "D".into(), ..Default::default() };
    d.area_results.insert("A1".into(), area_result(1.5, 0.0, 1.0));
    m.case_results.insert("D".into(), d);
    let mut max = CaseResults { case_name: "COMB".into(), ..Default::default() };
    max.area_results.insert("A1".into(), area_result(7.0, 2.0, 0.0));
    max.frame_forces.insert("F1".into(), stations([12.0, 9.0, 3.0]));
    let mut min = CaseResults { case_name: "COMB".into(), ..Default::default() };
    min.area_results.insert("A1".into(), area_result(-4.0, -1.0, 0.0));
    min.frame_forces.insert("F1".into(), stations([-8.0, -6.0, -2.0]));
    max.combo_min = Some(Box::new(min));
    m.case_results.insert("COMB".into(), max);
    m.is_locked = true;
    m
}

fn stations(m3: [f64; 3]) -> Vec<FrameStationResult> {
    m3.iter()
        .enumerate()
        .map(|(k, v)| FrameStationResult {
            station: 3.0 * k as f64,
            rel_dist: 0.5 * k as f64,
            m3: *v,
            ..FrameStationResult::default()
        })
        .collect()
}

#[test]
fn a_multi_valued_result_has_a_max_and_a_min_row_per_object_and_na_for_derived_columns() {
    use super::result_tables::table_text;
    let m = combo_model();
    let t = table_text(&m, "COMB", TableTab::AreaResults).unwrap();
    assert_eq!(t.headers[..2], ["Area".to_string(), "Step Type".to_string()]);
    assert_eq!(t.headers.last().unwrap(), "SVM Top");
    assert_eq!(t.rows.len(), 2);
    // The stored values, to the 4 decimals the table prints.
    assert_eq!(t.rows[0][..4], ["A1", "Max", "7.0000", "0.0000"].map(String::from));
    assert_eq!(t.rows[1][..4], ["A1", "Min", "-4.0000", "0.0000"].map(String::from));
    assert_eq!(t.rows[0].last().unwrap(), "n/a");
    assert_eq!(t.rows[1].last().unwrap(), "n/a");
    assert_eq!(t.rows[0][5], "2.0000");
    assert_eq!(t.rows[1][5], "-1.0000");
    // Frame forces: Max and Min per station, the station before the Step Type.
    let f = table_text(&m, "COMB", TableTab::FrameForces).unwrap();
    assert_eq!(f.headers[..3], ["Frame", "Station", "Step Type"].map(String::from));
    assert_eq!(f.rows.len(), 6);
    assert_eq!(f.rows[2][..3], ["F1", "3.00", "Max"].map(String::from));
    assert_eq!(f.rows[3][..3], ["F1", "3.00", "Min"].map(String::from));
    assert_eq!(f.rows[2].last().unwrap(), "9.000");
    assert_eq!(f.rows[3].last().unwrap(), "-6.000");
    // A single-valued result has neither the column nor "n/a".
    let mut single = m.clone();
    single.case_results.get_mut("COMB").unwrap().combo_min = None;
    let s = table_text(&single, "COMB", TableTab::AreaResults).unwrap();
    assert!(!s.headers.contains(&"Step Type".to_string()));
    assert_eq!(s.rows.len(), 1);
    assert_eq!(s.rows[0].last().unwrap(), "0.0000");
    // No results: no table; a design tab is not a result table.
    assert!(table_text(&m, "NOPE", TableTab::AreaResults).is_none());
    assert!(table_text(&m, "COMB", TableTab::SteelDesign).is_none());
}

#[test]
fn the_tables_window_paints_the_max_and_min_rows() {
    let mut run = Run::new(combo_model());
    run.app.active_case = "COMB".into();
    run.app.tables_tab = TableTab::AreaResults;
    run.app.show_tables_modal = true;
    run.settle();
    for s in ["Step Type", "Max", "Min", "7.0000", "-4.0000", "n/a"] {
        assert!(run.has_text(s), "{s} in {:?}", run.all_texts());
    }
}

#[test]
fn an_empty_area_table_of_a_combination_names_the_case_that_dropped_it() {
    // D + RS: the response-spectrum case has no shell results, so the
    // combination has none (CSI Analysis Reference p. 213 and the solver).
    let mut m = combo_model();
    let mut rs = LoadCase::linear_static("RS", "D", 1.0);
    rs.case_type = LoadCaseType::ResponseSpectrum;
    m.load_cases.insert("RS".into(), rs);
    m.load_combos.insert(
        "DRS".into(),
        LoadCombo { name: "DRS".into(), combo_type: ComboType::LinearAdd, cases: vec![("D".into(), 1.0), ("RS".into(), 1.0)] },
    );
    m.case_results.insert("D".into(), CaseResults { case_name: "D".into(), ..Default::default() });
    m.case_results.insert("RS".into(), CaseResults { case_name: "RS".into(), ..Default::default() });
    m.case_results.insert("DRS".into(), CaseResults { case_name: "DRS".into(), ..Default::default() });
    for tab in [TableTab::AreaResults, TableTab::SolidStresses, TableTab::LinkForces] {
        let t = super::result_tables::table_text(&m, "DRS", tab).unwrap();
        let msg = t.empty.expect("empty table");
        assert!(msg.contains("'DRS' has no"), "{msg}");
        assert!(msg.contains("'RS'") && msg.contains("ResponseSpectrum"), "{msg}");
    }
    // Painted through the window.
    let mut run = Run::new(m);
    run.app.active_case = "DRS".into();
    run.app.tables_tab = TableTab::AreaResults;
    run.app.show_tables_modal = true;
    run.settle();
    assert!(run.has_text_containing("case 'RS' (ResponseSpectrum) carries no shell results"), "{:?}", run.all_texts());
    // A model without areas keeps the plain text.
    let mut plain = combo_model();
    plain.case_results.get_mut("COMB").unwrap().area_results.clear();
    let t = super::result_tables::table_text(&plain, "COMB", TableTab::AreaResults).unwrap();
    assert_eq!(t.empty.unwrap(), "No area/shell results recorded for this case.");
}

#[test]
fn the_frame_diagram_of_a_multi_valued_combination_draws_both_curves() {
    let mut run = Run::new(combo_model());
    run.app.active_case = "COMB".into();
    run.app.display_mode = DisplayMode::FrameDiagram(FrameForceComponent::MomentM3);
    run.settle();
    // The peak labels: M3 is drawn on the tension side (negated), so the Max
    // curve (M3 = 12) peaks at -12 and the Min curve (M3 = -8) at 8.
    assert!(run.has_text("Max -12"), "{:?}", run.all_texts());
    assert!(run.has_text("Min 8"), "{:?}", run.all_texts());
    // A single-valued result draws one curve and no tag.
    let mut single = combo_model();
    single.case_results.get_mut("COMB").unwrap().combo_min = None;
    let mut run = Run::new(single);
    run.app.active_case = "COMB".into();
    run.app.display_mode = DisplayMode::FrameDiagram(FrameForceComponent::MomentM3);
    run.settle();
    assert!(run.has_text("12"), "{:?}", run.all_texts());
    assert!(!run.has_text_containing("Max ") && !run.has_text_containing("Min "));
}

// ------------------------------------------------------------------ D18

#[test]
fn the_i_section_dialog_makes_the_unequal_flange_section_and_refuses_a_wrong_dimension() {
    let mut run = Run::new(SapModel::new(Units::KnMC));
    run.app.custom_sec_material = run.app.model.materials.keys().next().cloned().expect("a material");
    run.app.show_section_modal = true;
    run.app.custom_sec_name = "UNEQ".into();
    run.app.custom_sec_shape = CustomSectionShape::WideFlangeI;
    // Reference R3: d 20, top flange 10 x 1, bottom flange 6 x 0.5, web 0.4.
    (run.app.custom_sec_d, run.app.custom_sec_b, run.app.custom_sec_tf) = (20.0, 10.0, 1.0);
    (run.app.custom_sec_bb, run.app.custom_sec_tfb, run.app.custom_sec_tw) = (6.0, 0.5, 0.4);
    run.app.custom_sec_symmetric = false;
    run.settle();
    for s in ["Top flange width t2:", "Bottom flange width t2b:", "Bottom flange thickness tfb:"] {
        assert!(run.has_text(s), "{s}");
    }
    let button = "Add Custom Section (& Assign to Selected Frames)";
    run.click_text(button);
    let sec = run.app.model.frame_sections.get("UNEQ").unwrap_or_else(|| panic!("{}", run.app.status_msg));
    // Written out here: three rectangles (b, h, y from the bottom).
    let parts = [(6.0, 0.5, 0.25), (0.4, 18.5, 9.75), (10.0, 1.0, 19.5)];
    let area: f64 = parts.iter().map(|(b, h, _)| b * h).sum();
    let ybar = parts.iter().map(|(b, h, y)| b * h * y).sum::<f64>() / area;
    let i33: f64 = parts.iter().map(|(b, h, y)| b * h * h * h / 12.0 + b * h * (y - ybar).powi(2)).sum();
    close("A", sec.area, 20.4, 1e-9);
    close("I33", sec.i33, i33, 1e-9);
    close("I33 printed", sec.i33, 1199.942647, 1e-9);
    close("I22", sec.i22, 92.432, 1e-9);
    // A wrong dimension is refused with its name and adds nothing.
    run.app.custom_sec_name = "BAD".into();
    run.app.custom_sec_tw = 7.0; // wider than the bottom flange (6)
    run.settle();
    run.click_text(button);
    assert!(run.app.status_msg.contains("bottom flange width t2b"), "{}", run.app.status_msg);
    assert!(!run.app.model.frame_sections.contains_key("BAD"));
    // Symmetric (the default): the bottom flange follows the top one.
    run.app.custom_sec_symmetric = true;
    run.app.custom_sec_tw = 0.4;
    run.app.custom_sec_name = "SYM".into();
    run.settle();
    run.click_text(button);
    let sym = &run.app.model.frame_sections["SYM"];
    let hw = 20.0 - 2.0;
    let i33 = 2.0 * (10.0 * 1.0 / 12.0 + 10.0 * (10.0_f64 - 0.5).powi(2)) + 0.4 * hw * hw * hw / 12.0;
    close("symmetric I33", sym.i33, i33, 1e-9);
}

// ------------------------------------------------------------------ G07

fn loaded_model() -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    m.load_cases.clear();
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 6.0, 0.0, 0.0);
    m.add_point("3", 6.0, 6.0, 0.0);
    m.add_point("4", 0.0, 6.0, 0.0);
    m.add_frame("F1", "1", "2", "FS");
    m.add_area("A1", ["1", "2", "3", "4"].map(String::from).to_vec(), "SH");
    let f = m.frames.get_mut("F1").unwrap();
    f.loads_dist.push(FrameLoadDistributed {
        load_pat: "DEAD".into(),
        my_type: 1,
        dir: 6,
        csys: "GLOBAL".into(),
        rel_dist1: 0.0,
        rel_dist2: 1.0,
        val1: -10.0,
        val2: -10.0,
    });
    f.loads_point.push(FrameLoadPoint {
        load_pat: "LIVE".into(),
        my_type: 1,
        dir: 6,
        csys: "GLOBAL".into(),
        rel_dist: 0.5,
        val: -25.0,
    });
    m.points.get_mut("3").unwrap().loads_force.push(PointLoadForce {
        load_pat: "DEAD".into(),
        csys: "GLOBAL".into(),
        force: [0.0, 0.0, -5.0, 0.0, 0.0, 0.0],
    });
    m.areas.get_mut("A1").unwrap().loads_uniform.push(sap_model::AreaLoadUniform {
        load_pat: "LIVE".into(),
        csys: "GLOBAL".into(),
        dir: 6,
        val: -2.0,
    });
    m
}

#[test]
fn the_load_lines_show_the_stored_values_in_display_units() {
    use super::object_info::{load_lines, ObjectRef};
    let m = loaded_model();
    let f = load_lines(&m, &ObjectRef::Frame("F1".into()), Units::KnMC).unwrap();
    assert_eq!(f.len(), 2);
    assert_eq!(
        f[0].cells(),
        ["DEAD", "Force per length", "GLOBAL", "Global Z", "-10 kN/m at 0 m to -10 kN/m at 6 m"].map(String::from)
    );
    assert_eq!(
        f[1].cells(),
        ["LIVE", "Point force", "GLOBAL", "Global Z", "-25 kN at 3 m"].map(String::from)
    );
    // N, mm: 1 kN/m = 1 N/mm, 1 kN = 1000 N, 1 m = 1000 mm (exact factors).
    let n = load_lines(&m, &ObjectRef::Frame("F1".into()), Units::NMmC).unwrap();
    assert_eq!(n[0].values, "-10 N/mm at 0 mm to -10 N/mm at 6000 mm");
    assert_eq!(n[1].values, "-25000 N at 3000 mm");
    let j = load_lines(&m, &ObjectRef::Joint("3".into()), Units::KnMC).unwrap();
    assert_eq!(
        j[0].values,
        "F1 = 0 kN, F2 = 0 kN, F3 = -5 kN, M1 = 0 kN-m, M2 = 0 kN-m, M3 = 0 kN-m"
    );
    let a = load_lines(&m, &ObjectRef::Area("A1".into()), Units::KnMC).unwrap();
    assert_eq!(a[0].cells(), ["LIVE", "Uniform", "GLOBAL", "Global Z", "-2 kN/m2"].map(String::from));
    // A kip, in view of the same area: 1 kN/m2 = 1/6.894757293168361 psi... in ksi.
    let ksi = load_lines(&m, &ObjectRef::Area("A1".into()), Units::KipInF).unwrap();
    let want = -2.0 * Units::KnMC.stress_factor_to(Units::KipInF);
    let got: f64 = ksi[0].values.split(' ').next().unwrap().parse().unwrap();
    assert!((got - want).abs() <= 1e-6 * want.abs() + 1e-6, "{got} {want}");
    assert!(ksi[0].values.ends_with("kip/in2"));
    // Refusals.
    assert!(load_lines(&m, &ObjectRef::Frame("NOPE".into()), Units::KnMC).is_err());
}

#[test]
fn a_right_click_in_the_undeformed_display_opens_the_object_window_with_the_loads() {
    let mut run = Run::new(loaded_model());
    run.app.display_mode = DisplayMode::Undeformed;
    run.settle();
    // Joint 3, then the frame F1 at its middle, then the area at its centre.
    run.right_click(run.screen([6.0, 6.0, 0.0]));
    assert!(run.has_text("Object Information"));
    assert!(run.has_text("Joint 3 - Loads"), "{:?}", run.all_texts());
    assert!(run.has_text("F1 = 0 kN, F2 = 0 kN, F3 = -5 kN, M1 = 0 kN-m, M2 = 0 kN-m, M3 = 0 kN-m"));
    run.right_click(run.screen([3.0, 0.0, 0.0]));
    assert!(run.has_text("Frame F1 - Loads"));
    assert!(run.has_text("-10 kN/m at 0 m to -10 kN/m at 6 m"));
    assert!(run.has_text("-25 kN at 3 m"));
    assert!(run.has_text("DEAD") && run.has_text("LIVE"));
    run.right_click(run.screen([3.0, 3.0, 0.0]));
    assert!(run.has_text("Area A1 - Loads"));
    assert!(run.has_text("-2 kN/m2"));
    // An object without loads says so.
    run.right_click(run.screen([0.0, 6.0, 0.0]));
    assert!(run.has_text("Joint 4 - Loads") && run.has_text("No loads are assigned to this object."));
    // The existing right-click of the result displays stays: on a frame of a
    // result display the member station window opens, not this one.
    let mut run = Run::new(combo_model());
    run.app.active_case = "COMB".into();
    run.app.display_mode = DisplayMode::FrameDiagram(FrameForceComponent::MomentM3);
    run.settle();
    run.right_click(run.screen([3.0, 0.0, 0.0]));
    assert!(!run.has_text("Object Information"));
    assert_eq!(run.app.inspected_frame.as_deref(), Some("F1"));
}

#[test]
fn the_pick_prefers_a_joint_then_a_frame_then_an_area() {
    use super::object_info::{pick, ObjectRef};
    let joints = vec![("J".to_string(), Pos2::new(10.0, 10.0))];
    let frames = vec![("F".to_string(), Pos2::new(0.0, 12.0), Pos2::new(100.0, 12.0))];
    let areas = vec![("A".to_string(), vec![Pos2::new(0.0, 0.0), Pos2::new(100.0, 0.0), Pos2::new(100.0, 100.0), Pos2::new(0.0, 100.0)])];
    assert_eq!(pick(&joints, &frames, &areas, Pos2::new(12.0, 12.0)), Some(ObjectRef::Joint("J".into())));
    assert_eq!(pick(&joints, &frames, &areas, Pos2::new(50.0, 15.0)), Some(ObjectRef::Frame("F".into())));
    assert_eq!(pick(&joints, &frames, &areas, Pos2::new(50.0, 50.0)), Some(ObjectRef::Area("A".into())));
    assert_eq!(pick(&joints, &frames, &areas, Pos2::new(150.0, 50.0)), None);
}

// ------------------------------------------------------------------ G36

#[test]
fn grid_only_gives_a_blank_model_and_the_quick_grid_lines_form() {
    let mut run = Run::new(SapModel::new(Units::KnMC));
    // Something to be replaced: Grid Only starts from scratch.
    run.app.model.add_point("OLD", 1.0, 1.0, 1.0);
    run.click_text("File");
    run.click_text("New Model from Templates... (Ctrl+N)");
    assert!(run.app.show_new_template_modal);
    run.click_text("▦ Grid Only");
    assert!(!run.app.show_new_template_modal);
    assert!(run.app.model.points.is_empty());
    assert_eq!(run.app.model.present_units, Units::KnMC);
    assert!(run.app.panel_build.open);
    run.settle();
    run.click_text("Create grid lines");
    let grids = &run.app.model.coord_systems["GLOBAL"].grids;
    // The form's counts (4, 4, 4) become 4 X, 4 Y and 4 Z grid lines; no object.
    for axis in ["X", "Y", "Z"] {
        assert_eq!(grids.iter().filter(|g| g.axis == axis).count(), 4, "{axis}");
    }
    assert!(run.app.model.points.is_empty() && run.app.model.frames.is_empty() && run.app.model.areas.is_empty());
}

// ------------------------------------------------------------ Create Video

/// A frame F1 1-2 and a column F2 2-3 whose top joint (3) moves 0.2 m per saved
/// step of a 5-step history (0 to 0.4 s); the model is locked (analysed).
fn history_model() -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    m.load_cases.clear();
    m.load_cases.insert("H".into(), LoadCase::linear_static("H", "D", 1.0));
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 6.0, 0.0, 0.0);
    m.add_point("3", 6.0, 0.0, 4.0);
    m.add_frame("F1", "1", "2", "FS");
    m.add_frame("F2", "2", "3", "FS");
    let mut cr = CaseResults { case_name: "H".into(), ..Default::default() };
    cr.time_steps = (0..5).map(|k| 0.1 * k as f64).collect();
    for j in ["1", "2", "3"] {
        let amp = if j == "3" { 0.2 } else { 0.0 };
        cr.time_history_disp
            .insert(j.to_string(), (0..5).map(|k| [amp * k as f64, 0.0, 0.0, 0.0, 0.0, 0.0]).collect());
        cr.joint_displacements.insert(j.to_string(), [amp * 4.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    }
    m.case_results.insert("H".into(), cr);
    m.is_locked = true;
    m
}

fn settings(path: &std::path::Path) -> VideoSettings {
    VideoSettings {
        kind: VideoKind::MultiStep { case: "H".into(), start: 0.0, end: 0.4, increment: 0.1 },
        width: 64,
        height: 48,
        fps: 10,
        magnification: 10.0,
        path: path.display().to_string(),
    }
}

fn temp_avi(tag: &str) -> PathBuf {
    std::env::temp_dir().join(format!("structurax_p6_{}_{tag}.avi", std::process::id()))
}

#[test]
fn the_create_video_menu_opens_both_forms() {
    let mut run = Run::new(history_model());
    run.click_text("File");
    run.click_text("Create Video");
    run.click_text("Multi-Step Animation Video...");
    assert!(run.app.video_form.is_open());
    assert!(run.has_text("Create Multi-Step Animation Video"), "{:?}", run.all_texts());
    run.app.video_form.close();
    run.settle();
    run.click_text("File");
    run.click_text("Create Video");
    run.click_text("Cyclic Animation Video...");
    assert!(run.app.video_form.is_open());
    assert!(run.has_text("Create Cyclic Animation Video"), "{:?}", run.all_texts());
}

#[test]
fn the_poses_come_in_order_and_a_mode_number_is_one_based() {
    let m = history_model();
    let path = temp_avi("poses");
    let s = settings(&path);
    let poses = video_export::frame_plan(&m, &s).unwrap();
    let steps: Vec<_> = poses.iter().map(|p| p.step_idx.unwrap()).collect();
    assert_eq!(steps, vec![0, 1, 2, 3, 4]);
    // Each pose is shown as a saved step, with the form's magnification as a
    // user-defined factor.
    for (k, p) in poses.iter().enumerate() {
        let d = video_menu::pose_display(&s, p);
        assert_eq!((d.use_step, d.step_idx, d.animate, d.scale), (true, k, false, 10.0));
    }
    let cyc = VideoSettings {
        kind: VideoKind::Cyclic { case: "H".into(), mode: 2, cycles: 1, frames_per_cycle: 4 },
        ..s
    };
    let pose = FramePose { step_idx: None, phase: std::f64::consts::FRAC_PI_2 };
    let d = video_menu::pose_display(&cyc, &pose);
    assert_eq!((d.mode_idx, d.use_step, d.animate), (1, false, true));
    close("phase", f64::from(d.phase), std::f64::consts::FRAC_PI_2, 1e-6);
}

#[test]
fn nothing_is_recorded_for_a_locked_out_or_result_less_state() {
    let path = temp_avi("refused");
    let _ = std::fs::remove_file(&path);
    let mut app = SapGuiApp::new(Some(PathBuf::from("/nonexistent/wave5-windows.s2k")));
    app.model = history_model();
    app.model.is_locked = false;
    let msg = app.create_video(&settings(&path));
    assert!(msg.contains("not locked"), "{msg}");
    assert!(!path.exists());
    app.model.is_locked = true;
    let missing = VideoSettings {
        kind: VideoKind::MultiStep { case: "NOPE".into(), start: 0.0, end: 0.4, increment: 0.1 },
        ..settings(&path)
    };
    let msg = app.create_video(&missing);
    assert!(msg.contains("'NOPE' has no results"), "{msg}");
    assert!(!path.exists());
}

#[test]
fn an_error_of_the_recorder_reaches_the_status_line_and_no_file_is_left() {
    let mut app = SapGuiApp::new(Some(PathBuf::from("/nonexistent/wave5-windows.s2k")));
    app.model = history_model();
    // The writer cannot create the file: the message of `save`.
    let dir = std::env::temp_dir().join(format!("structurax_p6_missing_{}", std::process::id()));
    let bad_path = dir.join("v.avi");
    let msg = app.create_video(&settings(&bad_path));
    assert!(msg.starts_with("Create Video: cannot write"), "{msg}");
    // A frame size the format refuses: the message of the plan/record.
    let too_small = VideoSettings { width: 1, ..settings(&temp_avi("small")) };
    let msg = app.create_video(&too_small);
    assert!(msg.starts_with("Create Video: frame size 1 x 48"), "{msg}");
    assert!(!temp_avi("small").exists());
}

/// Chunks `00db` (frame data) of an AVI file.
fn frame_chunks(bytes: &[u8]) -> Vec<&[u8]> {
    let mut out = Vec::new();
    let mut i = 0;
    while i + 8 <= bytes.len() {
        if &bytes[i..i + 4] == b"00db" {
            let n = u32::from_le_bytes(bytes[i + 4..i + 8].try_into().unwrap()) as usize;
            if i + 8 + n <= bytes.len() && n == 192 * 48 {
                out.push(&bytes[i + 8..i + 8 + n]);
                i += 8 + n;
                continue;
            }
        }
        i += 1;
    }
    out
}

#[test]
fn a_video_of_a_history_through_the_menu_path_has_the_layout_length_and_moving_frames() {
    let path = temp_avi("history");
    let _ = std::fs::remove_file(&path);
    let mut app = SapGuiApp::new(Some(PathBuf::from("/nonexistent/wave5-windows.s2k")));
    app.model = history_model();
    app.dual_pane = false;
    app.active_case = "H".into();
    app.display_mode = DisplayMode::Undeformed;
    app.deform_scale = 3.0;
    let before = (app.active_case.clone(), app.display_mode, app.deform_scale, app.use_time_step);
    let msg = app.create_video(&settings(&path));
    assert_eq!(msg, format!("Video written (5 frames): {}", path.display()));
    // The display the user had comes back.
    assert_eq!(before, (app.active_case.clone(), app.display_mode, app.deform_scale, app.use_time_step));
    let bytes = std::fs::read(&path).unwrap();
    // The length of 5 frames of 64 x 48 as the layout of the writer gives it
    // (the writer's own test checks its sizes against the format).
    let blank = video_export::RgbImage { width: 64, height: 48, pixels: vec![[0; 3]; 64 * 48] };
    let reference = video_export::avi_bytes(&vec![blank; 5], 10).unwrap();
    assert_eq!(bytes.len(), reference.len());
    let frames = frame_chunks(&bytes);
    assert_eq!(frames.len(), 5);
    assert!(frames.windows(2).all(|w| w[0] != w[1]), "every frame differs from the next: the joint moves");
    std::fs::remove_file(path).unwrap();
}

#[test]
fn the_selection_of_the_scaling_helper_is_used_for_a_step_of_a_history() {
    // The step selection of a history frame reaches the automatic scale: the
    // last step moves joint 3 by 0.8 m (0.2 per step), the first not at all.
    let m = history_model();
    let cr = &m.case_results["H"];
    let last = StepSelection { mode_idx: 0, use_step: true, step_idx: 4 };
    close("last", scaling::max_translation(&m, cr, last), 0.8, 1e-12);
    let first = StepSelection { mode_idx: 0, use_step: true, step_idx: 0 };
    assert_eq!(scaling::max_translation(&m, cr, first), 0.0);
    assert_eq!(scaling::view_subtitle(cr, last).unwrap(), "Step 4, Time = 0.4000 s");
}
