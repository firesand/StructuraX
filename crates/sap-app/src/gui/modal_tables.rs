//! Modal results tables ("Modal Participating Mass Ratios", "Modal Load
//! Participation Ratios", the per-mode participation table and the Ritz target
//! lines) and the Ritz starting-load controls of the analysis case editor.
//!
//! Source: CSI Analysis Reference Manual, "Modal Analysis Output" (printed
//! pp. 375-381) and "Ritz-Vector Analysis" (pp. 372-375). The 16 field names
//! of the mass-ratio table are those of the vendor's default format file
//! (table "Modal Participating Mass Ratios"); the load-participation table
//! follows the OAPI topic `ModalLoadParticipationRatios` (LoadCase, ItemType,
//! Item, static and dynamic percent), whose database field names are not in
//! the corpus, so the header strings of that table are a project choice.

use eframe::egui;
use sap_model::ritz::{RitzLoadControl, ACCELERATION_LOADS};
use sap_model::{CaseResults, LoadCase, LoadCaseAssignment, ModalSubType, SapModel};

/// Window state. It is reset when another model is loaded.
#[derive(Default)]
pub(super) struct State {
    pub(super) open: bool,
    /// Modal case shown; empty selects the first case with modes.
    pub(super) case: String,
    /// 0: "Modal Participating Mass Ratios", 1: "Modal Load Participation
    /// Ratios", 2: participation by mode.
    pub(super) tab: usize,
}

/// Titles of the three tabs, in tab order.
pub(super) const TAB_TITLES: [&str; 3] = [
    "Modal Participating Mass Ratios",
    "Modal Load Participation Ratios",
    "Load participation by mode",
];

/// The 16 fields of the vendor table "Modal Participating Mass Ratios".
pub(super) const MASS_RATIO_FIELDS: [&str; 16] = [
    "OutputCase",
    "StepType",
    "StepNum",
    "Period",
    "UX",
    "UY",
    "UZ",
    "SumUX",
    "SumUY",
    "SumUZ",
    "RX",
    "RY",
    "RZ",
    "SumRX",
    "SumRY",
    "SumRZ",
];

/// Header strings of the load participation table (project choice, see above).
pub(super) const ITEM_FIELDS: [&str; 5] = ["OutputCase", "ItemType", "Item", "Static %", "Dynamic %"];

/// Header strings of the per-mode participation table.
pub(super) const BY_MODE_FIELDS: [&str; 6] = [
    "Load",
    "Mode",
    "Static %",
    "Sum static %",
    "Dynamic %",
    "Sum dynamic %",
];

/// One row of "Modal Participating Mass Ratios": one mode.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct MassRatioRow {
    pub case: String,
    pub mode: usize,
    pub period: f64,
    /// UX, UY, UZ, RX, RY, RZ.
    pub ratio: [f64; 6],
    pub cumulative: [f64; 6],
}

/// One row of "Modal Load Participation Ratios": one item (an acceleration
/// UX..RZ or a load pattern) with its static and dynamic participation in
/// percent after the last mode.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct ItemRow {
    pub case: String,
    /// "Acceleration" or "Load Pattern".
    pub item_type: &'static str,
    pub item: String,
    pub static_percent: f64,
    pub dynamic_percent: f64,
}

/// One row of the per-mode participation table.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct ParticipationRow {
    pub load: String,
    pub mode: usize,
    pub static_ratio: f64,
    pub static_cumulative: f64,
    pub dynamic_ratio: f64,
    pub dynamic_cumulative: f64,
}

/// Periods and the six participating mass ratios with their cumulative sums.
pub(super) fn mass_ratio_rows(case: &str, results: &CaseResults) -> Vec<MassRatioRow> {
    let mut sums = [0.0; 6];
    results
        .modes
        .iter()
        .map(|mode| {
            for d in 0..6 {
                sums[d] += mode.modal_mass_ratios[d];
            }
            MassRatioRow {
                case: case.to_string(),
                mode: mode.mode_num,
                period: mode.period_sec,
                ratio: mode.modal_mass_ratios,
                cumulative: sums,
            }
        })
        .collect()
}

/// The 16 cells of a mass-ratio row, in the order of [`MASS_RATIO_FIELDS`].
/// `digits` is the number of decimals of the period and the ratios.
pub(super) fn mass_ratio_cells(row: &MassRatioRow, digits: usize) -> Vec<String> {
    let f = |v: f64| format!("{v:.digits$}");
    let mut cells = vec![
        row.case.clone(),
        "Mode".to_string(),
        row.mode.to_string(),
        f(row.period),
    ];
    // Field order: UX UY UZ SumUX SumUY SumUZ RX RY RZ SumRX SumRY SumRZ.
    cells.extend(row.ratio[..3].iter().map(|v| f(*v)));
    cells.extend(row.cumulative[..3].iter().map(|v| f(*v)));
    cells.extend(row.ratio[3..].iter().map(|v| f(*v)));
    cells.extend(row.cumulative[3..].iter().map(|v| f(*v)));
    cells
}

/// "Totals" line under the mass-ratio table: the cumulative ratio of every
/// direction after the last mode, in percent (a complete model gives 100).
pub(super) fn mass_ratio_totals(rows: &[MassRatioRow]) -> String {
    let Some(last) = rows.last() else {
        return "Totals: no modes".to_string();
    };
    let labels = ["UX", "UY", "UZ", "RX", "RY", "RZ"];
    let parts: Vec<String> = labels
        .iter()
        .zip(last.cumulative)
        .map(|(l, v)| format!("{l} {:.2} %", 100.0 * v))
        .collect();
    format!("Totals after {} modes: {}", rows.len(), parts.join(", "))
}

/// One row per item of the load participation results: static and dynamic
/// percent are 100 times the last cumulative value (zero for a case with no
/// modes).
pub(super) fn item_rows(case: &str, results: &CaseResults) -> Vec<ItemRow> {
    results
        .load_participation
        .iter()
        .map(|load| ItemRow {
            case: case.to_string(),
            item_type: if load.is_acceleration {
                "Acceleration"
            } else {
                "Load Pattern"
            },
            item: load.load_name.clone(),
            static_percent: 100.0 * load.static_cumulative.last().copied().unwrap_or(0.0),
            dynamic_percent: 100.0 * load.dynamic_cumulative.last().copied().unwrap_or(0.0),
        })
        .collect()
}

/// The five cells of an item row, in the order of [`ITEM_FIELDS`].
pub(super) fn item_cells(row: &ItemRow, digits: usize) -> Vec<String> {
    vec![
        row.case.clone(),
        row.item_type.to_string(),
        row.item.clone(),
        format!("{:.digits$}", row.static_percent),
        format!("{:.digits$}", row.dynamic_percent),
    ]
}

/// Static and dynamic participation of every item, mode by mode.
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

/// The cells of a per-mode row in percent, in the order of [`BY_MODE_FIELDS`].
pub(super) fn participation_cells(row: &ParticipationRow, digits: usize) -> Vec<String> {
    let mut cells = vec![row.load.clone(), row.mode.to_string()];
    cells.extend(
        [
            row.static_ratio,
            row.static_cumulative,
            row.dynamic_ratio,
            row.dynamic_cumulative,
        ]
        .iter()
        .map(|v| format!("{:.digits$}", 100.0 * v)),
    );
    cells
}

/// Ritz target lines of case `case`: one per starting load, naming the target
/// (or "no target"), the dynamic participation the modes reached in percent
/// and whether the target was reached, with the number of vectors made by
/// that load and in all. Empty for a case that is not a Ritz case with
/// starting loads or has no results.
pub(super) fn ritz_target_lines(model: &SapModel, case: &str) -> Vec<String> {
    let (Some(lc), Some(results)) = (model.load_cases.get(case), model.case_results.get(case))
    else {
        return Vec::new();
    };
    if lc.modal_sub_type != ModalSubType::Ritz {
        return Vec::new();
    }
    lc.loads
        .iter()
        .filter_map(|assignment| {
            let name = &assignment.load_name;
            let part = results
                .load_participation
                .iter()
                .find(|p| &p.load_name == name)?;
            let reached = *part.dynamic_cumulative.last()?;
            let target = lc
                .ritz_controls
                .iter()
                .find(|c| &c.load_name == name)
                .map_or(0.0, |c| c.target_dynamic_participation);
            let counts = format!(
                "{} vectors in all, {} from this load",
                results.modes.len(),
                part.vectors_generated
            );
            Some(if target > 0.0 {
                let verdict = if reached >= target - 1e-9 {
                    "target reached"
                } else {
                    "target not reached"
                };
                format!(
                    "{name}: target {:.1} %, reached {:.1} %: {verdict} ({counts})",
                    100.0 * target,
                    100.0 * reached
                )
            } else {
                format!(
                    "{name}: no target, dynamic participation {:.1} % ({counts})",
                    100.0 * reached
                )
            })
        })
        .collect()
}

fn tsv(headers: &[&str], rows: impl Iterator<Item = Vec<String>>) -> String {
    let mut text = headers.join("\t");
    text.push('\n');
    for row in rows {
        text.push_str(&row.join("\t"));
        text.push('\n');
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

/// The table text for `case` in `tab` (see [`TAB_TITLES`]), tab-separated with
/// the vendor header strings, or why there is none.
pub(super) fn table_text(model: &SapModel, case: &str, tab: usize) -> Result<String, String> {
    let results = model
        .case_results
        .get(case)
        .filter(|r| !r.modes.is_empty())
        .ok_or_else(|| format!("Case '{case}' has no modal results; run the analysis first."))?;
    Ok(match tab {
        0 => tsv(
            &MASS_RATIO_FIELDS,
            mass_ratio_rows(case, results)
                .iter()
                .map(|r| mass_ratio_cells(r, 6)),
        ),
        1 => tsv(
            &ITEM_FIELDS,
            item_rows(case, results).iter().map(|r| item_cells(r, 4)),
        ),
        _ => tsv(
            &BY_MODE_FIELDS,
            participation_rows(results)
                .iter()
                .map(|r| participation_cells(r, 4)),
        ),
    })
}

fn grid(ui: &mut egui::Ui, id: &str, headers: &[&str], rows: Vec<Vec<String>>) {
    egui::ScrollArea::horizontal()
        .id_salt(id)
        .show(ui, |ui| {
            egui::Grid::new(id).striped(true).show(ui, |ui| {
                for h in headers {
                    ui.strong(*h);
                }
                ui.end_row();
                for row in rows {
                    for cell in row {
                        ui.label(cell);
                    }
                    ui.end_row();
                }
            });
        });
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
        .default_width(900.0)
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
            });
            ui.horizontal(|ui| {
                for (i, title) in TAB_TITLES.iter().enumerate() {
                    ui.selectable_value(&mut state.tab, i, *title);
                }
            });
            let Some(results) = model.case_results.get(&state.case).filter(|r| !r.modes.is_empty())
            else {
                ui.label("No modal results. Run the analysis with a Modal case first.");
                return;
            };
            match state.tab {
                0 => {
                    ui.label("Period in seconds; participating mass ratios and their cumulative sums for the global translations and for the rotations about the centre of mass.");
                    let rows = mass_ratio_rows(&state.case, results);
                    ui.label(mass_ratio_totals(&rows));
                    grid(
                        ui,
                        "modal_mass_ratio_grid",
                        &MASS_RATIO_FIELDS,
                        rows.iter().map(|r| mass_ratio_cells(r, 5)).collect(),
                    );
                }
                1 => {
                    if results.load_participation.is_empty() {
                        ui.label("This case has no load participation results.");
                    } else {
                        ui.label("Static and dynamic participation of each item after the last mode, in percent: the six unit accelerations that act on mass, and the load patterns of the case.");
                        grid(
                            ui,
                            "modal_item_grid",
                            &ITEM_FIELDS,
                            item_rows(&state.case, results)
                                .iter()
                                .map(|r| item_cells(r, 3))
                                .collect(),
                        );
                    }
                    for line in ritz_target_lines(model, &state.case) {
                        ui.label(line);
                    }
                }
                _ => {
                    if results.load_participation.is_empty() {
                        ui.label("This case has no load participation results.");
                    } else {
                        ui.label("Static ratio: share of the strain energy of the static solution. Dynamic ratio: share of the load's mass-weighted norm. Percent.");
                        grid(
                            ui,
                            "modal_participation_grid",
                            &BY_MODE_FIELDS,
                            participation_rows(results)
                                .iter()
                                .map(|r| participation_cells(r, 3))
                                .collect(),
                        );
                    }
                }
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

    fn mode(n: usize, period: f64, mass: [f64; 6]) -> ModalModeResult {
        ModalModeResult {
            mode_num: n,
            period_sec: period,
            freq_hz: 1.0 / period,
            circ_freq_rad_s: 2.0 * std::f64::consts::PI / period,
            eigenvalue: 0.0,
            participation_factors: [0.0; 6],
            modal_mass_ratios: mass,
            mode_shape: Default::default(),
        }
    }

    fn item(name: &str, accel: bool, vectors: usize, stat: [f64; 2], dynamic: [f64; 2]) -> LoadParticipation {
        LoadParticipation {
            load_name: name.into(),
            is_acceleration: accel,
            vectors_generated: vectors,
            static_ratio: vec![stat[0], stat[1] - stat[0]],
            static_cumulative: stat.to_vec(),
            dynamic_ratio: vec![dynamic[0], dynamic[1] - dynamic[0]],
            dynamic_cumulative: dynamic.to_vec(),
        }
    }

    fn results() -> CaseResults {
        CaseResults {
            modes: vec![
                mode(1, 1.0, [0.5, 0.0, 0.0, 0.0, 0.125, 0.5]),
                mode(2, 0.5, [0.25, 0.1, 0.0, 0.0, 0.25, 0.25]),
            ],
            load_participation: vec![
                item("UX", true, 2, [0.9, 1.0], [0.5, 0.75]),
                item("LAT", false, 1, [0.5, 1.0], [0.25, 0.5]),
                item("RZ", true, 0, [0.5, 0.75], [0.5, 0.75]),
            ],
            ..Default::default()
        }
    }

    #[test]
    fn mass_ratio_rows_accumulate_each_direction() {
        let rows = mass_ratio_rows("MODAL", &results());
        assert_eq!(rows.len(), 2);
        assert_eq!(rows[1].cumulative, [0.75, 0.1, 0.0, 0.0, 0.375, 0.75]);
        assert_eq!(rows[0].period, 1.0);
        assert_eq!(rows[0].case, "MODAL");
    }

    #[test]
    fn mass_ratio_cells_follow_the_16_vendor_fields() {
        assert_eq!(MASS_RATIO_FIELDS.len(), 16);
        assert_eq!(
            MASS_RATIO_FIELDS.join(" "),
            "OutputCase StepType StepNum Period UX UY UZ SumUX SumUY SumUZ RX RY RZ SumRX SumRY SumRZ"
        );
        let rows = mass_ratio_rows("MODAL", &results());
        // Mode 2: ratios UX .25, UY .1, UZ 0, RX 0, RY .25, RZ .25; sums
        // .75, .1, 0, 0, .375, .75.
        assert_eq!(
            mass_ratio_cells(&rows[1], 3),
            [
                "MODAL", "Mode", "2", "0.500", "0.250", "0.100", "0.000", "0.750", "0.100",
                "0.000", "0.000", "0.250", "0.250", "0.000", "0.375", "0.750"
            ]
        );
        assert_eq!(mass_ratio_cells(&rows[0], 3).len(), MASS_RATIO_FIELDS.len());
    }

    #[test]
    fn mass_ratio_totals_state_the_last_cumulative_values_in_percent() {
        let rows = mass_ratio_rows("MODAL", &results());
        assert_eq!(
            mass_ratio_totals(&rows),
            "Totals after 2 modes: UX 75.00 %, UY 10.00 %, UZ 0.00 %, RX 0.00 %, RY 37.50 %, RZ 75.00 %"
        );
        assert_eq!(mass_ratio_totals(&[]), "Totals: no modes");
    }

    #[test]
    fn item_rows_list_every_item_with_its_final_cumulative_percent() {
        let rows = item_rows("MODAL", &results());
        let got: Vec<(&str, &str, f64, f64)> = rows
            .iter()
            .map(|r| (r.item_type, r.item.as_str(), r.static_percent, r.dynamic_percent))
            .collect();
        assert_eq!(
            got,
            [
                ("Acceleration", "UX", 100.0, 75.0),
                ("Load Pattern", "LAT", 100.0, 50.0),
                ("Acceleration", "RZ", 75.0, 75.0),
            ]
        );
        assert_eq!(item_cells(&rows[1], 1), ["MODAL", "Load Pattern", "LAT", "100.0", "50.0"]);
        assert_eq!(ITEM_FIELDS.len(), item_cells(&rows[0], 1).len());
        // A case with an empty cumulative vector reports zero, not a panic.
        let mut empty = results();
        empty.load_participation[0].dynamic_cumulative.clear();
        assert_eq!(item_rows("M", &empty)[0].dynamic_percent, 0.0);
    }

    #[test]
    fn participation_rows_pair_each_mode_with_each_load() {
        let rows = participation_rows(&results());
        assert_eq!(rows.len(), 6);
        assert_eq!(rows[1].dynamic_cumulative, 0.75);
        assert_eq!(rows[0].static_ratio, 0.9);
        assert_eq!(
            participation_cells(&rows[1], 4),
            ["UX", "2", "10.0000", "100.0000", "25.0000", "75.0000"]
        );
        assert_eq!(BY_MODE_FIELDS.len(), participation_cells(&rows[1], 4).len());
    }

    #[test]
    fn table_text_uses_the_vendor_headers_for_every_tab() {
        let mut model = SapModel::new(Units::KipInF);
        model.case_results.insert("MODAL".into(), results());
        let t0 = table_text(&model, "MODAL", 0).unwrap();
        let lines: Vec<&str> = t0.lines().collect();
        assert_eq!(lines.len(), 3);
        assert_eq!(
            lines[0],
            "OutputCase\tStepType\tStepNum\tPeriod\tUX\tUY\tUZ\tSumUX\tSumUY\tSumUZ\tRX\tRY\tRZ\tSumRX\tSumRY\tSumRZ"
        );
        assert!(lines[2].starts_with("MODAL\tMode\t2\t0.500000\t0.250000\t0.100000"));
        let t1 = table_text(&model, "MODAL", 1).unwrap();
        assert_eq!(
            t1,
            "OutputCase\tItemType\tItem\tStatic %\tDynamic %\n\
             MODAL\tAcceleration\tUX\t100.0000\t75.0000\n\
             MODAL\tLoad Pattern\tLAT\t100.0000\t50.0000\n\
             MODAL\tAcceleration\tRZ\t75.0000\t75.0000\n"
        );
        let t2 = table_text(&model, "MODAL", 2).unwrap();
        assert!(t2.starts_with("Load\tMode\tStatic %\tSum static %\tDynamic %\tSum dynamic %\n"));
        assert!(t2.contains("UX\t2\t10.0000\t100.0000\t25.0000\t75.0000"));
    }

    #[test]
    fn table_text_refuses_a_case_without_modes() {
        let model = SapModel::new(Units::KipInF);
        for tab in 0..3 {
            assert!(table_text(&model, "NONE", tab)
                .unwrap_err()
                .contains("no modal results"));
        }
    }

    /// A Ritz case R with starting loads UX (target 70 %), LAT (target 90 %)
    /// and a third load with no target; results as in `results()`.
    fn ritz_model() -> SapModel {
        let mut model = SapModel::new(Units::KipInF);
        let mut case = LoadCase::modal_eigen("R", 6);
        case.modal_sub_type = ModalSubType::Ritz;
        for name in ["UX", "LAT", "RZ"] {
            case.loads.push(LoadCaseAssignment {
                load_name: name.into(),
                scale_factor: 1.0,
            });
        }
        for (name, target) in [("UX", 0.7), ("LAT", 0.9)] {
            case.ritz_controls.push(RitzLoadControl {
                load_name: name.into(),
                max_cycles: 0,
                target_dynamic_participation: target,
            });
        }
        model.load_cases.insert("R".into(), case);
        model.case_results.insert("R".into(), results());
        model
    }

    #[test]
    fn ritz_target_lines_say_reached_or_not_reached() {
        let model = ritz_model();
        assert_eq!(
            ritz_target_lines(&model, "R"),
            [
                "UX: target 70.0 %, reached 75.0 %: target reached (2 vectors in all, 2 from this load)",
                "LAT: target 90.0 %, reached 50.0 %: target not reached (2 vectors in all, 1 from this load)",
                "RZ: no target, dynamic participation 75.0 % (2 vectors in all, 0 from this load)",
            ]
        );
        // Not a Ritz case, or no such case: no lines.
        let mut eigen = ritz_model();
        eigen.load_cases["R"].modal_sub_type = ModalSubType::Eigen;
        assert!(ritz_target_lines(&eigen, "R").is_empty());
        assert!(ritz_target_lines(&model, "NONE").is_empty());
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
        // One cycle is a control; no target and no cycle limit is none.
        assert_eq!(control_of(&case.ritz_controls, "LAT"), (1, 0.0));
        assert_eq!(case.ritz_controls.len(), 1);
        assert_eq!(case.loads.len(), 2, "a control is not a starting load");
        remove_starting_load(&mut case, 1);
        // Removing a load removes its control and only that load.
        assert!(case.ritz_controls.is_empty());
        assert_eq!(case.loads.len(), 1);
        assert_eq!(case.loads[0].load_name, "UX");
        // An index past the list changes nothing.
        let before = case.clone();
        remove_starting_load(&mut case, 7);
        assert_eq!(case, before);
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
        let mut time = 0.0;
        let mut frame = |case: &mut LoadCase, events: Vec<egui::Event>| {
            time += 0.5;
            ctx.run(
                egui::RawInput {
                    screen_rect: Some(egui::Rect::from_min_size(egui::Pos2::ZERO, egui::Vec2::new(900.0, 400.0))),
                    time: Some(time),
                    events,
                    ..Default::default()
                },
                |ctx| {
                    egui::CentralPanel::default().show(ctx, |ui| ritz_load_controls(ui, &model, case));
                },
            )
        };
        let _ = frame(&mut case, vec![]);
        let out = frame(&mut case, vec![]);
        // The row of the starting load shows its name, the control's cycles (2) and
        // target (80 %), as DragValues print them, and a Remove button.
        let shown = shape_texts(&out);
        for want in ["UX", "scale", "cycles", "target %", "Remove", "2", "80"] {
            assert!(shown.iter().any(|t| t == want), "{want} is drawn: {shown:?}");
        }
        assert_eq!(case.ritz_controls.len(), 1);
        // A click on Remove removes the load and its control.
        let at = out
            .shapes
            .iter()
            .find_map(|s| match &s.shape {
                egui::epaint::Shape::Text(t) if t.galley.job.text == "Remove" => Some(t.pos + t.galley.size() * 0.5),
                _ => None,
            })
            .expect("Remove button");
        let button = |pressed| egui::Event::PointerButton {
            pos: at,
            button: egui::PointerButton::Primary,
            pressed,
            modifiers: Default::default(),
        };
        let _ = frame(&mut case, vec![egui::Event::PointerMoved(at), button(true)]);
        let _ = frame(&mut case, vec![button(false)]);
        assert!(case.loads.is_empty(), "{:?}", case.loads);
        assert!(case.ritz_controls.is_empty());
    }

    fn shape_texts(out: &egui::FullOutput) -> Vec<String> {
        out.shapes
            .iter()
            .filter_map(|shape| match &shape.shape {
                egui::epaint::Shape::Text(t) => Some(t.galley.job.text.clone()),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn window_shows_the_ritz_target_lines_on_the_load_participation_tab() {
        let mut model = ritz_model();
        let mut state = State {
            open: true,
            tab: 1,
            case: "R".into(),
        };
        let ctx = egui::Context::default();
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::Vec2::new(1000.0, 800.0),
            )),
            ..Default::default()
        };
        let mut out = egui::FullOutput::default();
        for frame in 0..2 {
            let mut input = input.clone();
            input.time = Some(f64::from(frame));
            out = ctx.run(input, |ctx| {
                render(ctx, &mut state, &mut model);
            });
        }
        let texts = shape_texts(&out);
        for line in ritz_target_lines(&model, "R") {
            assert!(texts.contains(&line), "{line} in {texts:?}");
        }
    }

    #[test]
    fn window_renders_all_tabs_and_the_copy_button_reports() {
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
        // Each tab draws its own title, its vendor header strings and its rows.
        let wanted: [(&[&str], &[&str]); 3] = [
            (&MASS_RATIO_FIELDS, &["Totals after 2 modes: UX 75.00 %, UY 10.00 %, UZ 0.00 %, RX 0.00 %, RY 37.50 %, RZ 75.00 %", "Mode", "0.50000"]),
            (&ITEM_FIELDS, &["Acceleration", "Load Pattern", "LAT", "75.000"]),
            (&BY_MODE_FIELDS, &["UX", "10.000", "100.000"]),
        ];
        for (tab, (headers, cells)) in wanted.iter().enumerate() {
            state.tab = tab;
            // The first frame of a window only measures it.
            let mut out = egui::FullOutput::default();
            for _ in 0..2 {
                time += 1.0;
                out = ctx.run(input(time), |ctx| {
                    render(ctx, &mut state, &mut model);
                });
            }
            assert!(!out.shapes.is_empty());
            assert_eq!(state.case, "MODAL");
            let texts = shape_texts(&out);
            for title in TAB_TITLES {
                assert!(texts.iter().any(|t| t == title), "tab title {title}");
            }
            for h in headers.iter().chain(*cells) {
                assert!(texts.iter().any(|t| t == h), "tab {tab}: text {h:?} in {texts:?}");
            }
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
