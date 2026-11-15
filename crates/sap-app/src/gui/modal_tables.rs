//! Modal period, mass-ratio and load-participation tables, and the Ritz
//! starting-load controls of the analysis case editor.
//!
//! Source: CSI Analysis Reference Manual, "Modal Analysis Output" (printed
//! pp. 375-381) and "Ritz-Vector Analysis" (pp. 372-375).

use eframe::egui;
use sap_model::ritz::{RitzLoadControl, ACCELERATION_LOADS};
use sap_model::{CaseResults, LoadCase, LoadCaseAssignment, ModalSubType, SapModel};

/// Window state. It is reset when another model is loaded.
#[derive(Default)]
pub(super) struct State {
    pub(super) open: bool,
    /// Modal case shown; empty selects the first case with modes.
    pub(super) case: String,
    /// 0: participating mass ratios, 1: load participation.
    pub(super) tab: usize,
}

/// One row of "Modal Participating Mass Ratios".
#[derive(Debug, Clone, PartialEq)]
pub(super) struct MassRatioRow {
    pub mode: usize,
    pub period: f64,
    pub ratio: [f64; 3],
    pub cumulative: [f64; 3],
}

/// One row of the static/dynamic load participation table.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct ParticipationRow {
    pub load: String,
    pub mode: usize,
    pub static_ratio: f64,
    pub static_cumulative: f64,
    pub dynamic_ratio: f64,
    pub dynamic_cumulative: f64,
}

/// Periods and UX, UY, UZ participating mass ratios with their cumulative sums.
pub(super) fn mass_ratio_rows(results: &CaseResults) -> Vec<MassRatioRow> {
    let mut sums = [0.0; 3];
    results
        .modes
        .iter()
        .map(|mode| {
            let ratio = [
                mode.modal_mass_ratios[0],
                mode.modal_mass_ratios[1],
                mode.modal_mass_ratios[2],
            ];
            for d in 0..3 {
                sums[d] += ratio[d];
            }
            MassRatioRow {
                mode: mode.mode_num,
                period: mode.period_sec,
                ratio,
                cumulative: sums,
            }
        })
        .collect()
}

/// Static and dynamic participation of every starting load, mode by mode.
pub(super) fn participation_rows(results: &CaseResults) -> Vec<ParticipationRow> {
    let mut rows = Vec::new();
    for load in &results.load_participation {
        for (n, mode) in results.modes.iter().enumerate() {
            let at = |v: &Vec<f64>| v.get(n).copied().unwrap_or(0.0);
            rows.push(ParticipationRow {
                load: load.load_name.clone(),
                mode: mode.mode_num,
                static_ratio: at(&load.static_ratio),
                static_cumulative: at(&load.static_cumulative),
                dynamic_ratio: at(&load.dynamic_ratio),
                dynamic_cumulative: at(&load.dynamic_cumulative),
            });
        }
    }
    rows
}

fn mass_ratio_text(rows: &[MassRatioRow]) -> String {
    let mut text = String::from("Mode\tPeriod (s)\tUX\tUY\tUZ\tSumUX\tSumUY\tSumUZ\n");
    for r in rows {
        text.push_str(&format!(
            "{}\t{:.6e}\t{:.6}\t{:.6}\t{:.6}\t{:.6}\t{:.6}\t{:.6}\n",
            r.mode,
            r.period,
            r.ratio[0],
            r.ratio[1],
            r.ratio[2],
            r.cumulative[0],
            r.cumulative[1],
            r.cumulative[2]
        ));
    }
    text
}

fn participation_text(rows: &[ParticipationRow]) -> String {
    let mut text = String::from("Load\tMode\tStatic %\tSum static %\tDynamic %\tSum dynamic %\n");
    for r in rows {
        text.push_str(&format!(
            "{}\t{}\t{:.4}\t{:.4}\t{:.4}\t{:.4}\n",
            r.load,
            r.mode,
            100.0 * r.static_ratio,
            100.0 * r.static_cumulative,
            100.0 * r.dynamic_ratio,
            100.0 * r.dynamic_cumulative
        ));
    }
    text
}

/// Names of the modal cases that have results.
fn modal_cases_with_modes(model: &SapModel) -> Vec<String> {
    model
        .case_results
        .iter()
        .filter(|(_, r)| !r.modes.is_empty())
        .map(|(name, _)| name.clone())
        .collect()
}

/// The table text for `case` in `tab`, or why there is none.
pub(super) fn table_text(model: &SapModel, case: &str, tab: usize) -> Result<String, String> {
    let results = model
        .case_results
        .get(case)
        .filter(|r| !r.modes.is_empty())
        .ok_or_else(|| format!("Case '{case}' has no modal results; run the analysis first."))?;
    Ok(if tab == 0 {
        mass_ratio_text(&mass_ratio_rows(results))
    } else {
        participation_text(&participation_rows(results))
    })
}

/// Draws the window when it is open. Returns the outcome of a command the
/// user ran in this frame: a status message, or why it was refused.
pub(super) fn render(
    ctx: &egui::Context,
    state: &mut State,
    model: &mut SapModel,
) -> Option<Result<String, String>> {
    if !state.open {
        return None;
    }
    let mut open = true;
    let mut outcome = None;
    egui::Window::new("Modal Results Tables")
        .open(&mut open)
        .default_width(620.0)
        .vscroll(true)
        .show(ctx, |ui| {
            let cases = modal_cases_with_modes(model);
            if !cases.contains(&state.case) {
                state.case = cases.first().cloned().unwrap_or_default();
            }
            ui.horizontal(|ui| {
                ui.label("Modal case:");
                egui::ComboBox::from_id_salt("modal_tables_case")
                    .selected_text(state.case.as_str())
                    .show_ui(ui, |ui| {
                        for name in &cases {
                            ui.selectable_value(&mut state.case, name.clone(), name);
                        }
                    });
                ui.selectable_value(&mut state.tab, 0, "Modal Participating Mass Ratios");
                ui.selectable_value(&mut state.tab, 1, "Load Participation Ratios");
            });
            let Some(results) = model.case_results.get(&state.case).filter(|r| !r.modes.is_empty())
            else {
                ui.label("No modal results. Run the analysis with a Modal case first.");
                return;
            };
            if state.tab == 0 {
                ui.label("Period in seconds; participating mass ratios and their cumulative sums for the global translations.");
                egui::Grid::new("modal_mass_ratio_grid").striped(true).show(ui, |ui| {
                    for h in ["Mode", "Period", "UX", "UY", "UZ", "SumUX", "SumUY", "SumUZ"] {
                        ui.strong(h);
                    }
                    ui.end_row();
                    for r in mass_ratio_rows(results) {
                        ui.label(r.mode.to_string());
                        ui.label(format!("{:.5}", r.period));
                        for v in r.ratio.iter().chain(&r.cumulative) {
                            ui.label(format!("{v:.5}"));
                        }
                        ui.end_row();
                    }
                });
            } else if results.load_participation.is_empty() {
                ui.label("This case has no load participation results.");
            } else {
                ui.label("Static ratio: share of the strain energy of the static solution. Dynamic ratio: share of the load's mass-weighted norm. Percent.");
                egui::Grid::new("modal_participation_grid").striped(true).show(ui, |ui| {
                    for h in ["Load", "Mode", "Static %", "Sum static %", "Dynamic %", "Sum dynamic %"] {
                        ui.strong(h);
                    }
                    ui.end_row();
                    for r in participation_rows(results) {
                        ui.label(&r.load);
                        ui.label(r.mode.to_string());
                        for v in [r.static_ratio, r.static_cumulative, r.dynamic_ratio, r.dynamic_cumulative] {
                            ui.label(format!("{:.3}", 100.0 * v));
                        }
                        ui.end_row();
                    }
                });
            }
            if ui.button("Copy table").clicked() {
                outcome = Some(table_text(model, &state.case, state.tab).map(|text| {
                    ctx.copy_text(text);
                    format!("Copied the table of case '{}' to the clipboard.", state.case)
                }));
            }
        });
    state.open = open;
    outcome
}

// ---- Ritz starting-load controls of the case editor ----

/// Sets the cycle limit (`0` unlimited) and the dynamic participation target
/// in percent (`0` none) of a listed starting load. A load with neither is
/// stored without a control.
pub(super) fn set_control(
    case: &mut LoadCase,
    load: &str,
    max_cycles: usize,
    target_percent: f64,
) -> Result<(), String> {
    if !case.loads.iter().any(|l| l.load_name == load) {
        return Err(format!("'{load}' is not a starting load of this case."));
    }
    if !target_percent.is_finite() || !(0.0..=100.0).contains(&target_percent) {
        return Err(format!(
            "The target for '{load}' must be between 0 and 100 percent."
        ));
    }
    case.ritz_controls.retain(|c| c.load_name != load);
    if max_cycles != 0 || target_percent > 0.0 {
        case.ritz_controls.push(RitzLoadControl {
            load_name: load.to_string(),
            max_cycles,
            target_dynamic_participation: target_percent / 100.0,
        });
    }
    Ok(())
}

/// Adds a starting load (a pattern of the model or UX, UY, UZ) with scale 1.
pub(super) fn add_starting_load(
    model: &SapModel,
    case: &mut LoadCase,
    name: &str,
) -> Result<(), String> {
    let is_acceleration = ACCELERATION_LOADS
        .iter()
        .any(|(label, _)| label.eq_ignore_ascii_case(name));
    if is_acceleration && model.load_patterns.contains_key(name) {
        return Err(format!(
            "'{name}' is both a load pattern and an acceleration; rename the pattern."
        ));
    }
    if !is_acceleration && !model.load_patterns.contains_key(name) {
        return Err(format!(
            "'{name}' is neither a load pattern nor UX, UY, UZ."
        ));
    }
    if case.loads.iter().any(|l| l.load_name == name) {
        return Err(format!("'{name}' is already a starting load."));
    }
    case.loads.push(LoadCaseAssignment {
        load_name: name.to_string(),
        scale_factor: 1.0,
    });
    Ok(())
}

/// Removes starting load `index` and its generation control.
pub(super) fn remove_starting_load(case: &mut LoadCase, index: usize) {
    if index < case.loads.len() {
        let removed = case.loads.remove(index);
        case.ritz_controls
            .retain(|c| c.load_name != removed.load_name);
    }
}

/// Starting loads, cycles and target percent per load for a Ritz case.
pub(super) fn ritz_load_controls(ui: &mut egui::Ui, model: &SapModel, case: &mut LoadCase) {
    if case.modal_sub_type != ModalSubType::Ritz {
        return;
    }
    ui.label("Starting loads (load patterns or acceleration loads UX, UY, UZ), in generation order; cycles 0 = unlimited, target % 0 = none:");
    let mut remove = None;
    let mut edits = Vec::new();
    for (i, load) in case.loads.iter_mut().enumerate() {
        let (cycles0, target0) = control_of(&case.ritz_controls, &load.load_name);
        let (mut cycles, mut target) = (cycles0, target0);
        ui.horizontal(|ui| {
            ui.label(&load.load_name);
            ui.label("scale");
            ui.add(egui::DragValue::new(&mut load.scale_factor).speed(0.1));
            ui.label("cycles");
            ui.add(egui::DragValue::new(&mut cycles).range(0..=10000));
            ui.label("target %");
            ui.add(
                egui::DragValue::new(&mut target)
                    .range(0.0..=100.0)
                    .speed(1.0),
            );
            if ui.button("Remove").clicked() {
                remove = Some(i);
            }
        });
        if cycles != cycles0 || target != target0 {
            edits.push((load.load_name.clone(), cycles, target));
        }
    }
    for (name, cycles, target) in edits {
        let _ = set_control(case, &name, cycles, target);
    }
    if let Some(i) = remove {
        remove_starting_load(case, i);
    }
    ui.horizontal(|ui| {
        let candidates = ["UX", "UY", "UZ"]
            .into_iter()
            .map(str::to_owned)
            .chain(model.load_patterns.keys().cloned());
        let mut chosen = String::new();
        egui::ComboBox::from_id_salt("ritz_add_starting_load")
            .selected_text("Add starting load")
            .show_ui(ui, |ui| {
                for name in candidates {
                    ui.selectable_value(&mut chosen, name.clone(), name);
                }
            });
        if !chosen.is_empty() {
            // A refused addition (duplicate, ambiguous name) leaves the case unchanged;
            // the case definition is validated again when it is saved.
            let _ = add_starting_load(model, case, &chosen);
        }
    });
}

/// Cycle limit and target (percent) of `load` among `controls`.
fn control_of(controls: &[RitzLoadControl], load: &str) -> (usize, f64) {
    controls
        .iter()
        .find(|c| c.load_name == load)
        .map_or((0, 0.0), |c| {
            (c.max_cycles, 100.0 * c.target_dynamic_participation)
        })
}

#[cfg(test)]
mod tests {
    use super::*;
    use sap_model::ritz::LoadParticipation;
    use sap_model::{LoadPattern, LoadPatternType, ModalModeResult};
    use sap_units::Units;

    fn mode(n: usize, period: f64, mass: [f64; 3]) -> ModalModeResult {
        ModalModeResult {
            mode_num: n,
            period_sec: period,
            freq_hz: 1.0 / period,
            circ_freq_rad_s: 2.0 * std::f64::consts::PI / period,
            eigenvalue: 0.0,
            participation_factors: [0.0; 6],
            modal_mass_ratios: [mass[0], mass[1], mass[2], 0.0, 0.0, 0.0],
            mode_shape: Default::default(),
        }
    }

    fn results() -> CaseResults {
        CaseResults {
            modes: vec![
                mode(1, 1.0, [0.5, 0.0, 0.0]),
                mode(2, 0.5, [0.25, 0.1, 0.0]),
            ],
            load_participation: vec![LoadParticipation {
                load_name: "UX".into(),
                is_acceleration: true,
                vectors_generated: 2,
                static_ratio: vec![0.9, 0.1],
                static_cumulative: vec![0.9, 1.0],
                dynamic_ratio: vec![0.5, 0.25],
                dynamic_cumulative: vec![0.5, 0.75],
            }],
            ..Default::default()
        }
    }

    #[test]
    fn mass_ratio_rows_accumulate_each_direction() {
        let rows = mass_ratio_rows(&results());
        assert_eq!(rows.len(), 2);
        assert_eq!(rows[1].cumulative, [0.75, 0.1, 0.0]);
        assert_eq!(rows[0].period, 1.0);
    }

    #[test]
    fn participation_rows_pair_each_mode_with_each_load() {
        let rows = participation_rows(&results());
        assert_eq!(rows.len(), 2);
        assert_eq!(rows[1].dynamic_cumulative, 0.75);
        assert_eq!(rows[0].static_ratio, 0.9);
        let text = participation_text(&rows);
        assert!(text.starts_with("Load\tMode"));
        assert!(text.contains("UX\t2\t10.0000\t100.0000\t25.0000\t75.0000"));
    }

    #[test]
    fn table_text_refuses_a_case_without_modes() {
        let model = SapModel::new(Units::KipInF);
        assert!(table_text(&model, "NONE", 0)
            .unwrap_err()
            .contains("no modal results"));
    }

    fn model_with_pattern() -> SapModel {
        let mut model = SapModel::new(Units::KipInF);
        model.load_patterns.insert(
            "LAT".into(),
            LoadPattern {
                name: "LAT".into(),
                pat_type: LoadPatternType::Other,
                self_wt_multiplier: 0.0,
            },
        );
        model
    }

    #[test]
    fn starting_load_controls_edit_the_case() {
        let model = model_with_pattern();
        let mut case = LoadCase::modal_eigen("R", 6);
        case.modal_sub_type = ModalSubType::Ritz;
        add_starting_load(&model, &mut case, "UX").unwrap();
        add_starting_load(&model, &mut case, "LAT").unwrap();
        set_control(&mut case, "UX", 3, 90.0).unwrap();
        assert_eq!(control_of(&case.ritz_controls, "UX"), (3, 90.0));
        assert!((case.ritz_controls[0].target_dynamic_participation - 0.9).abs() < 1e-15);
        // zeroing both removes the control
        set_control(&mut case, "UX", 0, 0.0).unwrap();
        assert!(case.ritz_controls.is_empty());
        set_control(&mut case, "LAT", 1, 0.0).unwrap();
        remove_starting_load(&mut case, 1);
        assert!(case.ritz_controls.is_empty());
        assert_eq!(case.loads.len(), 1);
    }

    #[test]
    fn starting_load_edits_refuse_bad_input_and_leave_the_case_unchanged() {
        let mut model = model_with_pattern();
        let mut case = LoadCase::modal_eigen("R", 6);
        case.modal_sub_type = ModalSubType::Ritz;
        add_starting_load(&model, &mut case, "UX").unwrap();
        let before = case.clone();
        assert!(add_starting_load(&model, &mut case, "UX")
            .unwrap_err()
            .contains("already"));
        assert!(add_starting_load(&model, &mut case, "NOPE")
            .unwrap_err()
            .contains("neither"));
        assert!(set_control(&mut case, "LAT", 1, 0.0)
            .unwrap_err()
            .contains("not a starting load"));
        for bad in [-1.0, 100.5, f64::NAN] {
            assert!(set_control(&mut case, "UX", 1, bad)
                .unwrap_err()
                .contains("between 0 and 100"));
        }
        model.load_patterns.insert(
            "UY".into(),
            LoadPattern {
                name: "UY".into(),
                pat_type: LoadPatternType::Other,
                self_wt_multiplier: 0.0,
            },
        );
        assert!(add_starting_load(&model, &mut case, "UY")
            .unwrap_err()
            .contains("rename"));
        assert_eq!(case, before);
    }

    #[test]
    fn ritz_controls_render_in_the_case_editor() {
        let model = model_with_pattern();
        let mut case = LoadCase::modal_eigen("R", 6);
        case.modal_sub_type = ModalSubType::Ritz;
        add_starting_load(&model, &mut case, "UX").unwrap();
        set_control(&mut case, "UX", 2, 80.0).unwrap();
        let ctx = egui::Context::default();
        let out = ctx.run(egui::RawInput::default(), |ctx| {
            egui::CentralPanel::default().show(ctx, |ui| ritz_load_controls(ui, &model, &mut case));
        });
        assert!(!out.shapes.is_empty());
        assert_eq!(case.ritz_controls.len(), 1);
    }

    #[test]
    fn window_renders_both_tables_and_the_copy_button_reports() {
        use egui::{Pos2, Rect, Vec2};
        let mut model = SapModel::new(Units::KipInF);
        model.case_results.insert("MODAL".into(), results());
        let mut state = State {
            open: true,
            ..State::default()
        };
        let ctx = egui::Context::default();
        let input = |time: f64| egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1000.0, 800.0))),
            time: Some(time),
            ..Default::default()
        };
        let mut time = 0.0;
        for tab in [0, 1] {
            state.tab = tab;
            time += 1.0;
            let out = ctx.run(input(time), |ctx| {
                render(ctx, &mut state, &mut model);
            });
            assert!(!out.shapes.is_empty());
            assert_eq!(state.case, "MODAL");
        }
        // Find the button and click it.
        time += 1.0;
        let out = ctx.run(input(time), |ctx| {
            render(ctx, &mut state, &mut model);
        });
        let pos = out
            .shapes
            .iter()
            .find_map(|shape| match &shape.shape {
                egui::epaint::Shape::Text(t) if t.galley.job.text == "Copy table" => {
                    Some(t.pos + t.galley.size() * 0.5)
                }
                _ => None,
            })
            .expect("Copy table button is visible");
        let mut down = input(time + 1.0);
        down.events = vec![
            egui::Event::PointerMoved(pos),
            egui::Event::PointerButton {
                pos,
                button: egui::PointerButton::Primary,
                pressed: true,
                modifiers: Default::default(),
            },
        ];
        let mut seen = None;
        let _ = ctx.run(down, |ctx| {
            seen = seen.take().or(render(ctx, &mut state, &mut model));
        });
        let mut up = input(time + 1.1);
        up.events = vec![egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed: false,
            modifiers: Default::default(),
        }];
        let _ = ctx.run(up, |ctx| {
            seen = seen.take().or(render(ctx, &mut state, &mut model));
        });
        let message = seen.expect("button click returns an outcome").unwrap();
        assert!(message.contains("MODAL"));
    }
}
