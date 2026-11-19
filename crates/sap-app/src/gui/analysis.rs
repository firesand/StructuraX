//! Native case/function editor over the validated API definitions.
use super::*;
use sap_model::hinge::{HingeUnloading, ModeLoad, ResultsSaved};
use sap_model::{
    DisplacementControl, DynamicFunction, HistoryLoadKind, LoadCaseAssignment, LoadCaseType, ModalComboMethod,
    ModalSubType, NLGeomType, TimeIntegrationMethod,
};

pub(super) struct AnalysisEditor {
    pub open: bool,
    pub tab: usize,
    case: LoadCase,
    function: DynamicFunction,
    samples: String,
}
impl Default for AnalysisEditor {
    fn default() -> Self {
        Self {
            open: false,
            tab: 0,
            case: LoadCase::linear_static("STATIC1", "DEAD", 1.0),
            function: DynamicFunction {
                name: "FUNCTION1".into(),
                is_time_history: true,
                damping_ratio: 0.05,
                points: vec![(0.0, 0.0), (1.0, 1.0)],
            },
            samples: "0 0\n1 1".into(),
        }
    }
}
fn parse_samples(text: &str) -> Result<Vec<(f64, f64)>, String> {
    let mut points = Vec::new();
    for (idx, line) in text.lines().enumerate() {
        let line = line.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        if points.len() >= 100_000 {
            return Err("Function editor accepts at most 100000 samples.".into());
        }
        let parts: Vec<_> = line
            .split(|c: char| c.is_whitespace() || c == ',')
            .filter(|s| !s.is_empty())
            .collect();
        if parts.len() != 2 {
            return Err(format!("Line {}: enter exactly two numbers.", idx + 1));
        }
        let x = parts[0]
            .parse::<f64>()
            .map_err(|_| format!("Line {}: invalid first number.", idx + 1))?;
        let y = parts[1]
            .parse::<f64>()
            .map_err(|_| format!("Line {}: invalid second number.", idx + 1))?;
        points.push((x, y));
    }
    Ok(points)
}
fn save_function(model: &mut SapModel, state: &AnalysisEditor) -> Result<String, String> {
    let mut function = state.function.clone();
    function.points = parse_samples(&state.samples)?;
    sap_api::set_function_definition(model, function).map_err(|e| e.to_string())?;
    Ok(format!("Saved function {}.", state.function.name))
}
fn delete_case(model: &mut SapModel, name: &str) -> Result<String, String> {
    if model.is_locked {
        return Err("Unlock the model before deleting cases.".into());
    }
    if !model.load_cases.contains_key(name) {
        return Err(format!("Case '{name}' does not exist."));
    }
    if model
        .load_cases
        .values()
        .any(|c| {
            c.name != name
                && (c.initial_case.as_deref() == Some(name)
                    || c.modal_case.as_deref() == Some(name))
        })
    {
        return Err(format!(
            "Case '{name}' is referenced as initial/modal source by another case."
        ));
    }
    if model
        .load_combos
        .values()
        .any(|combo| combo.cases.iter().any(|(c, _)| c == name))
    {
        return Err(format!(
            "Case '{name}' is referenced by a load combination; update or delete the combination first."
        ));
    }
    model.load_cases.shift_remove(name);
    model.invalidate_results();
    Ok(format!("Deleted case {name}."))
}
fn delete_function(model: &mut SapModel, name: &str) -> Result<String, String> {
    if model.is_locked {
        return Err("Unlock the model before deleting functions.".into());
    }
    if !model.functions.contains_key(name) {
        return Err(format!("Function '{name}' does not exist."));
    }
    if model.load_cases.values().any(|c| {
        c.spectrum_loads.iter().any(|(f, _, _)| f == name)
            || c.history_loads.iter().any(|(f, _, _)| f == name)
    }) {
        return Err(format!(
            "Function '{name}' is referenced by an analysis case."
        ));
    }
    model.functions.shift_remove(name);
    model.invalidate_results();
    Ok(format!("Deleted function {name}."))
}
fn new_case(kind: LoadCaseType, name: &str) -> LoadCase {
    let mut case = LoadCase::linear_static(name, "DEAD", 1.0);
    case.case_type = kind;
    if matches!(
        kind,
        LoadCaseType::Modal
            | LoadCaseType::ResponseSpectrum
            | LoadCaseType::LinearHistory
            | LoadCaseType::LinearDynamic
            | LoadCaseType::NonlinearHistory
            | LoadCaseType::NonlinearDynamic
    ) {
        case.loads.clear();
    }
    if kind == LoadCaseType::Modal {
        case = LoadCase::modal_eigen(name, 12);
    }
    // Without a modal case (direct integration, the default of a new case) a
    // history case takes no modal damping; `case_controls` sets a damping
    // ratio when a Modal case is chosen for superposition.
    if matches!(
        kind,
        LoadCaseType::LinearHistory | LoadCaseType::NonlinearHistory
    ) {
        case.modal_damping = 0.0;
    }
    case
}
fn text_choice(
    ui: &mut egui::Ui,
    id: impl std::hash::Hash,
    value: &mut String,
    choices: impl Iterator<Item = String>,
) {
    egui::ComboBox::from_id_salt(id)
        .selected_text(value.as_str())
        .show_ui(ui, |ui| {
            for name in choices {
                ui.selectable_value(value, name.clone(), name);
            }
        });
}
fn static_loads(ui: &mut egui::Ui, model: &SapModel, case: &mut LoadCase) {
    let mut remove = None;
    for (i, load) in case.loads.iter_mut().enumerate() {
        ui.horizontal(|ui| {
            text_choice(
                ui,
                ("case_pattern", i),
                &mut load.load_name,
                model.load_patterns.keys().cloned(),
            );
            ui.add(egui::DragValue::new(&mut load.scale_factor).speed(0.1));
            if ui.button("Remove").clicked() {
                remove = Some(i);
            }
        });
    }
    if let Some(i) = remove {
        case.loads.remove(i);
    }
    if ui.button("Add Pattern").clicked() {
        case.loads.push(LoadCaseAssignment {
            load_name: model
                .load_patterns
                .keys()
                .next()
                .cloned()
                .unwrap_or_default(),
            scale_factor: 1.0,
        });
    }
}
/// Load assignments of a time-history case. Each entry is either a ground
/// acceleration (U1, U2, U3) or a load pattern, and the case stores which
/// (D77): the form writes exactly the kind it shows, whatever the name.
fn history_load_entries(ui: &mut egui::Ui, model: &SapModel, case: &mut LoadCase) {
    let mut remove = None;
    let mut edits: Vec<(usize, String, HistoryLoadKind)> = Vec::new();
    for i in 0..case.history_loads.len() {
        let kind = case.history_kind(i);
        let (function, target, scale) = &mut case.history_loads[i];
        ui.horizontal(|ui| {
            text_choice(
                ui,
                ("case_function", true, i),
                function,
                model
                    .functions
                    .values()
                    .filter(|f| f.is_time_history)
                    .map(|f| f.name.clone()),
            );
            let shown = match kind {
                HistoryLoadKind::Acceleration => format!("Accel {target}"),
                HistoryLoadKind::Pattern => format!("Pattern {target}"),
                HistoryLoadKind::Unspecified => format!("{target} (type not stated)"),
            };
            egui::ComboBox::from_id_salt(("case_target", true, i))
                .selected_text(shown)
                .show_ui(ui, |ui| {
                    for dir in ["U1", "U2", "U3"] {
                        let chosen = kind == HistoryLoadKind::Acceleration && target == dir;
                        if ui.selectable_label(chosen, format!("Accel {dir}")).clicked() {
                            edits.push((i, dir.to_string(), HistoryLoadKind::Acceleration));
                        }
                    }
                    for name in model.load_patterns.keys() {
                        let chosen = kind == HistoryLoadKind::Pattern && target == name;
                        if ui.selectable_label(chosen, format!("Pattern {name}")).clicked() {
                            edits.push((i, name.clone(), HistoryLoadKind::Pattern));
                        }
                    }
                });
            ui.add(egui::DragValue::new(scale).speed(0.1));
            if ui.button("Remove").clicked() {
                remove = Some(i);
            }
        });
    }
    let mut kinds: Vec<HistoryLoadKind> = (0..case.history_loads.len())
        .map(|i| case.history_kind(i))
        .collect();
    for (i, name, kind) in edits {
        case.history_loads[i].1 = name;
        kinds[i] = kind;
    }
    case.history_load_kinds = kinds;
    case.normalize_history_kinds();
    if let Some(i) = remove {
        case.remove_history_load(i);
    }
    if ui.button("Add Function Assignment").clicked() {
        let function = model
            .functions
            .values()
            .find(|f| f.is_time_history)
            .map(|f| f.name.clone())
            .unwrap_or_default();
        case.push_history_load(function, "U1", HistoryLoadKind::Acceleration, 1.0);
    }
}
fn dynamic_loads(
    ui: &mut egui::Ui,
    model: &SapModel,
    loads: &mut Vec<(String, String, f64)>,
    history: bool,
) {
    let mut remove = None;
    for (i, (function, target, scale)) in loads.iter_mut().enumerate() {
        ui.horizontal(|ui| {
            text_choice(
                ui,
                ("case_function", history, i),
                function,
                model
                    .functions
                    .values()
                    .filter(|f| f.is_time_history == history)
                    .map(|f| f.name.clone()),
            );
            text_choice(
                ui,
                ("case_target", history, i),
                target,
                ["U1", "U2", "U3"]
                    .into_iter()
                    .map(str::to_owned)
                    .chain(model.load_patterns.keys().filter(|_| history).cloned()),
            );
            ui.add(egui::DragValue::new(scale).speed(0.1));
            if ui.button("Remove").clicked() {
                remove = Some(i);
            }
        });
    }
    if let Some(i) = remove {
        loads.remove(i);
    }
    if ui.button("Add Function Assignment").clicked() {
        loads.push((
            model
                .functions
                .values()
                .find(|f| f.is_time_history == history)
                .map(|f| f.name.clone())
                .unwrap_or_default(),
            "U1".into(),
            1.0,
        ));
    }
}
/// How a linear history case is solved: direct integration or superposition of
/// the modes of a Modal case. Choosing a Modal case clears the settings that
/// modal superposition refuses (Rayleigh damping, initial case) and gives a
/// zero damping ratio the usual 5 %; going back to direct integration resets
/// the damping ratio that direct integration refuses.
fn history_solution_choice(ui: &mut egui::Ui, model: &SapModel, case: &mut LoadCase) {
    let before = case.modal_case.clone();
    let mut source = before.clone().unwrap_or_default();
    ui.horizontal(|ui| {
        ui.label("Solution:");
        egui::ComboBox::from_id_salt("analysis_history_solution")
            .selected_text(if source.is_empty() {
                "Direct integration".to_string()
            } else {
                format!("Modal superposition of {source}")
            })
            .show_ui(ui, |ui| {
                ui.selectable_value(&mut source, String::new(), "Direct integration");
                for (name, modal) in &model.load_cases {
                    if modal.case_type == LoadCaseType::Modal && name != &case.name {
                        ui.selectable_value(&mut source, name.clone(), format!("Modal: {name}"));
                    }
                }
            });
    });
    let chosen = (!source.is_empty()).then_some(source);
    if chosen != before {
        match &chosen {
            Some(_) => {
                case.rayleigh_damping = (0.0, 0.0);
                case.initial_case = None;
                if case.modal_damping == 0.0 {
                    case.modal_damping = 0.05;
                }
            }
            None => case.modal_damping = 0.0,
        }
        case.modal_case = chosen;
    }
}
// ---------------------------------------------------------------------------
// Nonlinear static case commands. They edit the case being drafted, check
// what they can against the model, and leave the draft unchanged when they
// refuse. Saving the draft runs the full `sap_api::validate_case_definition`.
// ---------------------------------------------------------------------------

/// Names of the Accel loads of a nonlinear static case.
pub(super) const ACCEL_NAMES: [&str; 3] = ["UX", "UY", "UZ"];

const DOF_LABELS: [&str; 6] = ["U1", "U2", "U3", "R1", "R2", "R3"];

fn is_accel(load: &LoadCaseAssignment) -> bool {
    sap_model::ritz::acceleration_direction(&load.load_name).is_some()
}

/// Nonlinear static cases the draft can continue from: not itself, and none
/// whose own chain of initial cases leads back to it.
pub(super) fn eligible_initial_cases(model: &SapModel, case: &LoadCase) -> Vec<String> {
    model
        .load_cases
        .values()
        .filter(|c| {
            c.case_type == LoadCaseType::NonlinearStatic
                && c.name != case.name
                && initial_chain_is_clear(model, case, &c.name)
        })
        .map(|c| c.name.clone())
        .collect()
}

/// True when following the initial cases from `start` never reaches `case`.
fn initial_chain_is_clear(model: &SapModel, case: &LoadCase, start: &str) -> bool {
    // Walk by name: the draft may not be in the model yet, but a stored case
    // can already continue from its name.
    let mut seen: Vec<&str> = Vec::new();
    let mut current = Some(start);
    while let Some(name) = current {
        if name == case.name {
            return false;
        }
        if seen.contains(&name) {
            return true; // a cycle that does not involve the draft
        }
        seen.push(name);
        current = model
            .load_cases
            .get(name)
            .and_then(|c| c.initial_case.as_deref());
    }
    true
}

/// Chooses the case the draft continues from, or none.
pub(super) fn set_initial_case(
    model: &SapModel,
    case: &mut LoadCase,
    initial: Option<&str>,
) -> Result<(), String> {
    if case.case_type != LoadCaseType::NonlinearStatic {
        return Err("Only a nonlinear static case continues from an initial case here.".into());
    }
    let Some(name) = initial else {
        case.initial_case = None;
        return Ok(());
    };
    if name == case.name {
        return Err("A case cannot continue from itself.".into());
    }
    let Some(source) = model.load_cases.get(name) else {
        return Err(format!("Case '{name}' does not exist."));
    };
    if source.case_type != LoadCaseType::NonlinearStatic {
        return Err(format!(
            "Case '{name}' is not a nonlinear static case; only a nonlinear static case can be an initial case."
        ));
    }
    if !initial_chain_is_clear(model, case, name) {
        return Err(format!(
            "Case '{name}' continues from '{}' through its own initial cases; that would be a cycle.",
            case.name
        ));
    }
    case.initial_case = Some(name.to_string());
    Ok(())
}

/// Load application control: the load pattern is scaled until the joint
/// displacement has moved by `target` (a positive magnitude).
pub(super) fn set_displacement_control(
    model: &SapModel,
    case: &mut LoadCase,
    joint: &str,
    dof: usize,
    target: f64,
    conjugate: bool,
) -> Result<(), String> {
    if case.case_type != LoadCaseType::NonlinearStatic {
        return Err("Only a nonlinear static case has load application control.".into());
    }
    // P-Delta with displacement control is analyzed for models with frame
    // hinges and for Accel or Mode loads (sixth tutorial-video wave, G14);
    // the solver refuses the other combinations by name when the case runs.
    if case.nl_geom == NLGeomType::LargeDisplacements {
        return Err("Displacement control is not available with large displacements; set the geometry to small displacements or P-Delta first.".into());
    }
    if !model.points.contains_key(joint) {
        return Err(format!("Joint '{joint}' does not exist."));
    }
    if dof >= 6 {
        return Err("The monitored degree of freedom must be U1, U2, U3, R1, R2 or R3.".into());
    }
    if !target.is_finite() || target <= 0.0 {
        return Err("The target displacement must be a positive number.".into());
    }
    case.displacement_control = Some(DisplacementControl {
        target,
        monitored: Some((joint.to_string(), dof)),
        conjugate,
    });
    Ok(())
}

/// Back to full load application.
pub(super) fn set_full_load(case: &mut LoadCase) {
    case.displacement_control = None;
}

/// Saved states of a nonlinear static case.
pub(super) fn set_results_saved(case: &mut LoadCase, saved: ResultsSaved) -> Result<(), String> {
    if case.case_type != LoadCaseType::NonlinearStatic {
        return Err("Only a nonlinear static case has saved states to choose.".into());
    }
    if let ResultsSaved::MultipleStates {
        min_saved,
        max_saved,
        ..
    } = saved
    {
        if min_saved == 0 || max_saved < min_saved || max_saved > 100_000 {
            return Err("Multiple saved states need 1 <= minimum <= maximum <= 100000.".into());
        }
    }
    case.nl_static.results_saved = saved;
    Ok(())
}

/// Maximum total and null steps.
pub(super) fn set_step_limits(
    case: &mut LoadCase,
    max_total: usize,
    max_null: usize,
) -> Result<(), String> {
    if case.case_type != LoadCaseType::NonlinearStatic {
        return Err("Only a nonlinear static case has step limits.".into());
    }
    if !(1..=1_000_000).contains(&max_total) || !(1..=1_000_000).contains(&max_null) {
        return Err("Maximum total and null steps must each be 1 to 1000000.".into());
    }
    case.nl_static.max_total_steps = max_total;
    case.nl_static.max_null_steps = max_null;
    Ok(())
}

pub(super) fn unloading_label(method: HingeUnloading) -> &'static str {
    match method {
        HingeUnloading::UnloadEntireStructure => "Unload entire structure",
        HingeUnloading::ApplyLocalRedistribution => "Apply local redistribution",
        HingeUnloading::RestartUsingSecantStiffness => "Restart using secant stiffness",
    }
}

/// Hinge unloading method. Only "unload entire structure" is implemented.
pub(super) fn set_hinge_unloading(
    case: &mut LoadCase,
    method: HingeUnloading,
) -> Result<(), String> {
    if method != HingeUnloading::UnloadEntireStructure {
        return Err(format!(
            "{} is not implemented; only \"unload entire structure\" is.",
            unloading_label(method)
        ));
    }
    case.nl_static.hinge_unloading = method;
    Ok(())
}

fn require_nl_static(case: &LoadCase) -> Result<(), String> {
    if case.case_type == LoadCaseType::NonlinearStatic {
        Ok(())
    } else {
        Err("Only a nonlinear static case takes Accel and Mode loads here.".into())
    }
}

fn refuse_if_mode_loads(case: &LoadCase) -> Result<(), String> {
    if case.nl_static.mode_loads.is_empty() {
        Ok(())
    } else {
        Err("The case has Mode loads, which cannot be mixed with other loads; remove them first."
            .into())
    }
}

/// Adds a load pattern row.
pub(super) fn add_pattern_load(
    model: &SapModel,
    case: &mut LoadCase,
    pattern: &str,
    scale: f64,
) -> Result<(), String> {
    require_nl_static(case)?;
    refuse_if_mode_loads(case)?;
    if !model.load_patterns.contains_key(pattern) {
        return Err(format!("Load pattern '{pattern}' does not exist."));
    }
    if !scale.is_finite() {
        return Err("The scale factor must be a finite number.".into());
    }
    case.loads.push(LoadCaseAssignment {
        load_name: pattern.to_string(),
        scale_factor: scale,
    });
    Ok(())
}

/// Adds an acceleration load row (UX, UY or UZ).
pub(super) fn add_accel_load(
    model: &SapModel,
    case: &mut LoadCase,
    direction: &str,
    scale: f64,
) -> Result<(), String> {
    require_nl_static(case)?;
    refuse_if_mode_loads(case)?;
    if !ACCEL_NAMES.contains(&direction) {
        return Err(format!(
            "'{direction}' is not an acceleration load; use UX, UY or UZ."
        ));
    }
    if !scale.is_finite() {
        return Err("The scale factor must be a finite number.".into());
    }
    if let Some(clash) = model
        .load_patterns
        .keys()
        .find(|p| sap_model::ritz::acceleration_direction(p).is_some())
    {
        return Err(format!(
            "Load pattern '{clash}' is named like an acceleration load; rename it before using Accel loads."
        ));
    }
    case.loads.push(LoadCaseAssignment {
        load_name: direction.to_string(),
        scale_factor: scale,
    });
    Ok(())
}

/// Modal cases that can supply Mode loads: run, and not the draft.
pub(super) fn modal_cases_for_modes(model: &SapModel, case: &LoadCase) -> Vec<String> {
    model
        .load_cases
        .values()
        .filter(|c| c.case_type == LoadCaseType::Modal && c.run_flag && c.name != case.name)
        .map(|c| c.name.clone())
        .collect()
}

/// Adds a Mode load: mode number of the Modal case, with a scale factor.
pub(super) fn add_mode_load(
    model: &SapModel,
    case: &mut LoadCase,
    modal_case: &str,
    mode: usize,
    scale: f64,
) -> Result<(), String> {
    require_nl_static(case)?;
    if !case.loads.is_empty() {
        return Err("Mode loads cannot be mixed with load patterns or Accel loads; remove those rows first.".into());
    }
    let modal = model
        .load_cases
        .get(modal_case)
        .filter(|m| m.case_type == LoadCaseType::Modal && m.name != case.name)
        .ok_or_else(|| format!("'{modal_case}' is not an existing Modal case."))?;
    if !modal.run_flag {
        return Err(format!("Modal case '{modal_case}' is not set to run."));
    }
    if case.modal_case.as_deref().is_some_and(|m| m != modal_case)
        && !case.nl_static.mode_loads.is_empty()
    {
        return Err("All Mode loads of a case take their modes from one Modal case.".into());
    }
    if mode == 0 || mode > modal.max_modes {
        return Err(format!(
            "Mode {mode} is outside 1 to {} (the maximum modes of '{modal_case}').",
            modal.max_modes
        ));
    }
    if !scale.is_finite() {
        return Err("The scale factor must be a finite number.".into());
    }
    if case.nl_static.mode_loads.iter().any(|l| l.mode == mode) {
        return Err(format!(
            "Mode {mode} is already loaded; change its scale factor instead."
        ));
    }
    case.modal_case = Some(modal_case.to_string());
    case.nl_static.mode_loads.push(ModeLoad {
        mode,
        scale_factor: scale,
    });
    Ok(())
}

/// Removes a Mode load row; the modal case is cleared with the last one.
pub(super) fn remove_mode_load(case: &mut LoadCase, index: usize) -> Result<(), String> {
    if index >= case.nl_static.mode_loads.len() {
        return Err("There is no such Mode load row.".into());
    }
    case.nl_static.mode_loads.remove(index);
    if case.nl_static.mode_loads.is_empty() {
        case.modal_case = None;
    }
    Ok(())
}

/// "Stiffness to Use" of a modal or linear static case (v19 help topic of
/// that name): the unstressed state, or the stiffness at the end of a
/// nonlinear static case. Whether the hinges of that case allow it is decided
/// when the case runs (sixth tutorial-video wave, G48).
pub(super) fn set_stiffness_case(
    model: &SapModel,
    case: &mut LoadCase,
    source: Option<&str>,
) -> Result<(), String> {
    if !matches!(
        case.case_type,
        LoadCaseType::Modal | LoadCaseType::LinearStatic
    ) {
        return Err("Only a modal or a linear static case chooses a stiffness here.".into());
    }
    let Some(name) = source else {
        case.initial_case = None;
        return Ok(());
    };
    if name == case.name {
        return Err("A case cannot use its own stiffness.".into());
    }
    match model.load_cases.get(name) {
        None => Err(format!("Case '{name}' does not exist.")),
        Some(c) if c.case_type != LoadCaseType::NonlinearStatic => Err(format!(
            "Case '{name}' is not a nonlinear static case; only the stiffness at the end of a nonlinear static case can be used."
        )),
        Some(_) => {
            case.initial_case = Some(name.to_string());
            Ok(())
        }
    }
}

/// The "Stiffness to Use" row of a modal or linear static case.
fn stiffness_controls(ui: &mut egui::Ui, model: &SapModel, case: &mut LoadCase) {
    const ZERO: &str = "Zero initial conditions - unstressed state";
    let sources: Vec<String> = model
        .load_cases
        .values()
        .filter(|c| c.case_type == LoadCaseType::NonlinearStatic && c.name != case.name)
        .map(|c| c.name.clone())
        .collect();
    let mut choice = case.initial_case.clone();
    ui.horizontal(|ui| {
        ui.label("Stiffness to use:");
        egui::ComboBox::from_id_salt("stiffness_to_use")
            .selected_text(match &choice {
                Some(name) => format!("At end of nonlinear case {name}"),
                None => ZERO.to_string(),
            })
            .show_ui(ui, |ui| {
                ui.selectable_value(&mut choice, None, ZERO);
                for name in &sources {
                    ui.selectable_value(
                        &mut choice,
                        Some(name.clone()),
                        format!("At end of nonlinear case {name}"),
                    );
                }
            });
    });
    if choice != case.initial_case {
        if let Err(error) = set_stiffness_case(model, case, choice.as_deref()) {
            ui.colored_label(Color32::YELLOW, format!("Not changed: {error}"));
        }
    }
    if case.initial_case.is_some() {
        ui.weak("The geometric stiffness of that case is used only if it considered P-Delta. In a model with hinges the case runs only if no hinge has yielded at the end of that case.");
    }
}

/// The load rows and settings of a nonlinear static case. Returns the reason
/// when a command the user triggered was refused.
fn nl_static_controls(ui: &mut egui::Ui, model: &SapModel, case: &mut LoadCase) -> Option<String> {
    let mut error: Option<String> = None;
    let mut note = |r: Result<(), String>| {
        if let Err(e) = r {
            error = Some(e);
        }
    };

    ui.label("Initial conditions:");
    let eligible = eligible_initial_cases(model, case);
    let mut choice = case.initial_case.clone();
    ui.horizontal(|ui| {
        ui.label("Continue from case:");
        egui::ComboBox::from_id_salt("nl_initial_case")
            .selected_text(choice.as_deref().unwrap_or("Zero initial conditions"))
            .show_ui(ui, |ui| {
                ui.selectable_value(&mut choice, None, "Zero initial conditions");
                for name in &eligible {
                    ui.selectable_value(&mut choice, Some(name.clone()), name);
                }
            });
    });
    if choice != case.initial_case {
        note(set_initial_case(model, case, choice.as_deref()));
    }

    ui.separator();
    ui.label("Loads applied:");
    let mut remove_load = None;
    let mut remove_mode = None;
    let patterns: Vec<String> = model.load_patterns.keys().cloned().collect();
    for (i, load) in case.loads.iter_mut().enumerate() {
        ui.horizontal(|ui| {
            if is_accel(load) {
                ui.label("Accel");
                egui::ComboBox::from_id_salt(("nl_accel", i))
                    .selected_text(load.load_name.as_str())
                    .show_ui(ui, |ui| {
                        for name in ACCEL_NAMES {
                            ui.selectable_value(&mut load.load_name, name.to_string(), name);
                        }
                    });
            } else {
                ui.label("Load Pattern");
                text_choice(
                    ui,
                    ("nl_pattern", i),
                    &mut load.load_name,
                    patterns.iter().cloned(),
                );
            }
            ui.add(egui::DragValue::new(&mut load.scale_factor).speed(0.1));
            if ui.button("Remove").clicked() {
                remove_load = Some(i);
            }
        });
    }
    let mut modal_case = case.modal_case.clone().unwrap_or_default();
    let modals = modal_cases_for_modes(model, case);
    for (i, load) in case.nl_static.mode_loads.iter_mut().enumerate() {
        ui.horizontal(|ui| {
            ui.label("Mode");
            ui.add(egui::DragValue::new(&mut load.mode).range(1..=10_000));
            ui.label("of modal case");
            text_choice(ui, ("nl_mode_case", i), &mut modal_case, modals.iter().cloned());
            ui.add(egui::DragValue::new(&mut load.scale_factor).speed(0.1));
            if ui.button("Remove").clicked() {
                remove_mode = Some(i);
            }
        });
    }
    if !case.nl_static.mode_loads.is_empty() {
        case.modal_case = Some(modal_case);
    }
    if let Some(i) = remove_load {
        case.loads.remove(i);
    }
    if let Some(i) = remove_mode {
        note(remove_mode_load(case, i));
    }
    ui.horizontal(|ui| {
        if ui.button("Add Load Pattern").clicked() {
            let first = patterns.first().cloned().unwrap_or_default();
            note(add_pattern_load(model, case, &first, 1.0));
        }
        if ui.button("Add Accel").clicked() {
            note(add_accel_load(model, case, "UX", 1.0));
        }
        if ui.button("Add Mode").clicked() {
            let modal = case
                .modal_case
                .clone()
                .or_else(|| modals.first().cloned())
                .unwrap_or_default();
            let next = case
                .nl_static
                .mode_loads
                .iter()
                .map(|l| l.mode)
                .max()
                .map_or(1, |m| m + 1);
            note(add_mode_load(model, case, &modal, next, 1.0));
        }
    });
    ui.label("Mode load: joint force = scale x omega^2 x mass x mode shape, modes of a Modal case that is run. It cannot be mixed with other loads.");

    ui.separator();
    ui.horizontal(|ui| {
        ui.label("Geometry:");
        ui.selectable_value(&mut case.nl_geom, NLGeomType::None, "Small displacement");
        ui.selectable_value(&mut case.nl_geom, NLGeomType::PDelta, "P-Delta");
    });
    ui.horizontal(|ui| {
        ui.label("Iterations:");
        ui.add(egui::DragValue::new(&mut case.max_iterations).range(1..=10000));
        ui.label("Tolerance:");
        ui.add(
            egui::DragValue::new(&mut case.convergence_tol)
                .speed(1e-7)
                .max_decimals(12),
        );
    });

    ui.separator();
    ui.label("Load application:");
    let mut displacement = case.displacement_control.is_some();
    ui.horizontal(|ui| {
        if ui.selectable_label(!displacement, "Full load").clicked() {
            displacement = false;
        }
        if ui
            .selectable_label(displacement, "Displacement control")
            .clicked()
        {
            displacement = true;
        }
    });
    if !displacement {
        set_full_load(case);
    } else {
        let (mut joint, mut dof, mut target, mut conjugate) = match &case.displacement_control {
            Some(c) => (
                c.monitored.as_ref().map(|(j, _)| j.clone()),
                c.monitored.as_ref().map_or(0, |(_, d)| *d),
                c.target,
                c.conjugate,
            ),
            None => (model.points.keys().next().cloned(), 0, 1.0, true),
        };
        let before = (joint.clone(), dof, target, conjugate);
        ui.horizontal(|ui| {
            ui.label("Monitored joint");
            let mut name = joint.clone().unwrap_or_default();
            text_choice(ui, "nl_monitor_joint", &mut name, model.points.keys().cloned());
            joint = Some(name).filter(|n| !n.is_empty());
            ui.label("DOF");
            egui::ComboBox::from_id_salt("nl_monitor_dof")
                .selected_text(DOF_LABELS[dof.min(5)])
                .show_ui(ui, |ui| {
                    for (i, label) in DOF_LABELS.iter().enumerate() {
                        ui.selectable_value(&mut dof, i, *label);
                    }
                });
            ui.label("Target displacement");
            ui.add(egui::DragValue::new(&mut target).speed(0.01).max_decimals(8));
        });
        ui.checkbox(&mut conjugate, "Use conjugate displacement");
        let unchanged =
            case.displacement_control.is_some() && before == (joint.clone(), dof, target, conjugate);
        if !unchanged {
            match &joint {
                Some(j) => note(set_displacement_control(
                    model, case, j, dof, target, conjugate,
                )),
                None => note(Err("The model has no joint to monitor.".into())),
            }
        }
    }

    ui.separator();
    ui.label("Results saved:");
    let was_multiple = matches!(
        case.nl_static.results_saved,
        ResultsSaved::MultipleStates { .. }
    );
    let mut multiple = was_multiple;
    ui.horizontal(|ui| {
        ui.selectable_value(&mut multiple, false, "Final state only");
        ui.selectable_value(&mut multiple, true, "Multiple states");
    });
    if multiple != was_multiple {
        note(set_results_saved(
            case,
            if multiple {
                ResultsSaved::multiple_default()
            } else {
                ResultsSaved::FinalState
            },
        ));
    } else if let ResultsSaved::MultipleStates {
        mut min_saved,
        mut max_saved,
        mut positive_only,
    } = case.nl_static.results_saved
    {
        ui.horizontal(|ui| {
            ui.label("Minimum number of saved states");
            ui.add(egui::DragValue::new(&mut min_saved).range(1..=100_000));
            ui.label("Maximum");
            ui.add(egui::DragValue::new(&mut max_saved).range(1..=100_000));
            ui.checkbox(&mut positive_only, "Save positive increments only");
        });
        let new = ResultsSaved::MultipleStates {
            min_saved,
            max_saved,
            positive_only,
        };
        if new != case.nl_static.results_saved {
            note(set_results_saved(case, new));
        }
    }
    let (mut total, mut null) = (
        case.nl_static.max_total_steps,
        case.nl_static.max_null_steps,
    );
    ui.horizontal(|ui| {
        ui.label("Maximum total steps");
        ui.add(egui::DragValue::new(&mut total).range(1..=1_000_000));
        ui.label("Maximum null steps");
        ui.add(egui::DragValue::new(&mut null).range(1..=1_000_000));
    });
    if (total, null)
        != (
            case.nl_static.max_total_steps,
            case.nl_static.max_null_steps,
        )
    {
        note(set_step_limits(case, total, null));
    }
    ui.label("Hinge unloading method:");
    for method in [
        HingeUnloading::UnloadEntireStructure,
        HingeUnloading::ApplyLocalRedistribution,
        HingeUnloading::RestartUsingSecantStiffness,
    ] {
        ui.horizontal(|ui| {
            if method == HingeUnloading::UnloadEntireStructure {
                let selected = case.nl_static.hinge_unloading == method;
                if ui
                    .selectable_label(selected, unloading_label(method))
                    .clicked()
                {
                    note(set_hinge_unloading(case, method));
                }
            } else {
                ui.add_enabled(false, egui::Button::new(unloading_label(method)));
                ui.weak("not available: not implemented");
            }
        });
    }
    error
}

fn case_controls(ui: &mut egui::Ui, model: &SapModel, case: &mut LoadCase) {
    ui.horizontal(|ui| {
        ui.label("Name:");
        ui.text_edit_singleline(&mut case.name);
        ui.checkbox(&mut case.run_flag, "Run this case");
    });
    let mut kind = case.case_type;
    egui::ComboBox::from_id_salt("analysis_case_type")
        .selected_text(format!("{kind:?}"))
        .show_ui(ui, |ui| {
            for value in [
                LoadCaseType::LinearStatic,
                LoadCaseType::NonlinearStatic,
                LoadCaseType::Modal,
                LoadCaseType::Buckling,
                LoadCaseType::ResponseSpectrum,
                LoadCaseType::LinearHistory,
                LoadCaseType::LinearDynamic,
                LoadCaseType::NonlinearHistory,
                LoadCaseType::NonlinearDynamic,
            ] {
                ui.selectable_value(&mut kind, value, format!("{value:?}"));
            }
        });
    if kind != case.case_type {
        *case = new_case(kind, &case.name);
    }
    ui.label(
        "Changing case type resets its settings. Saving an existing name replaces that definition.",
    );
    match case.case_type {
        LoadCaseType::NonlinearStatic => {
            if let Some(error) = nl_static_controls(ui, model, case) {
                ui.colored_label(Color32::YELLOW, format!("Not changed: {error}"));
            }
            ui.label("Element applicability and convergence are checked when analysis runs.");
        }
        LoadCaseType::LinearStatic | LoadCaseType::Buckling => {
            if case.case_type == LoadCaseType::LinearStatic {
                stiffness_controls(ui, model, case);
            }
            static_loads(ui, model, case);
            if case.case_type == LoadCaseType::Buckling {
                ui.horizontal(|ui| {
                    ui.label("Buckling modes:");
                    ui.add(egui::DragValue::new(&mut case.num_buckling_modes).range(1..=10000));
                });
            }
        }
        LoadCaseType::Modal => {
            stiffness_controls(ui, model, case);
            let old_subtype = case.modal_sub_type;
            ui.horizontal(|ui| {
                ui.selectable_value(&mut case.modal_sub_type, ModalSubType::Eigen, "Eigen");
                ui.selectable_value(&mut case.modal_sub_type, ModalSubType::Ritz, "Ritz");
                ui.label("Maximum modes:");
                ui.add(egui::DragValue::new(&mut case.max_modes).range(1..=10000));
            });
            if old_subtype != case.modal_sub_type && case.modal_sub_type == ModalSubType::Eigen {
                case.loads.clear();
                case.ritz_controls.clear();
            }
            super::modal_tables::ritz_load_controls(ui, model, case);
        }
        LoadCaseType::ResponseSpectrum => {
            let mut source = case.initial_case.clone().unwrap_or_default();
            ui.horizontal(|ui| {
                ui.label("Modal source:");
                text_choice(
                    ui,
                    "analysis_modal_source",
                    &mut source,
                    model
                        .load_cases
                        .values()
                        .filter(|c| c.case_type == LoadCaseType::Modal && c.name != case.name)
                        .map(|c| c.name.clone()),
                );
            });
            case.initial_case = if source.is_empty() {
                None
            } else {
                Some(source)
            };
            ui.horizontal(|ui| {
                for method in [
                    ModalComboMethod::Cqc,
                    ModalComboMethod::Srss,
                    ModalComboMethod::AbsSum,
                ] {
                    ui.selectable_value(&mut case.modal_combo, method, format!("{method:?}"));
                }
                ui.label("Damping ratio:");
                ui.add(egui::DragValue::new(&mut case.modal_damping).speed(0.005));
            });
            dynamic_loads(ui, model, &mut case.spectrum_loads, false);
            ui.label(
                "Function ordinates × scale are spectral acceleration in current model units.",
            );
        }
        LoadCaseType::LinearHistory
        | LoadCaseType::LinearDynamic
        | LoadCaseType::NonlinearHistory
        | LoadCaseType::NonlinearDynamic => {
            if case.case_type == LoadCaseType::LinearHistory {
                history_solution_choice(ui, model, case);
            }
            let superposed = case.modal_case.is_some();
            if superposed {
                ui.horizontal(|ui| {
                    ui.label("Modal damping ratio (constant for all modes, 0 <= ratio < 1):");
                    ui.add(
                        egui::DragValue::new(&mut case.modal_damping)
                            .speed(0.005)
                            .range(0.0..=0.99),
                    );
                });
            } else if matches!(
                case.case_type,
                LoadCaseType::LinearHistory | LoadCaseType::NonlinearHistory
            ) {
                // An imported modal history may carry a modal damping ratio. It is
                // shown because analysis refuses the case until it is zero.
                ui.horizontal(|ui| {
                    ui.label("Modal damping ratio (direct integration takes none; must be 0, use Rayleigh damping):");
                    ui.add(egui::DragValue::new(&mut case.modal_damping).speed(0.005));
                });
            }
            if matches!(
                case.case_type,
                LoadCaseType::NonlinearHistory | LoadCaseType::NonlinearDynamic
            ) {
                ui.label("Nonlinear link history from rest; continuation and geometric nonlinearity are unavailable.");
                ui.horizontal(|ui| {
                    ui.label("Maximum iterations per step:");
                    ui.add(egui::DragValue::new(&mut case.max_iterations).range(1..=10000));
                    ui.label("Convergence tolerance:");
                    ui.add(
                        egui::DragValue::new(&mut case.convergence_tol)
                            .speed(1e-7)
                            .max_decimals(12),
                    );
                });
            }
            ui.horizontal(|ui| {
                ui.label("Steps:");
                ui.add(egui::DragValue::new(&mut case.num_time_steps).range(1..=1_000_000));
                ui.label("dt (seconds):");
                ui.add(
                    egui::DragValue::new(&mut case.time_step_dt)
                        .speed(0.001)
                        .max_decimals(10),
                );
            });
            if superposed {
                ui.label("Each mode is integrated exactly for loads that vary linearly between the output steps and the function points; integrator and Rayleigh settings do not apply.");
            } else {
            egui::ComboBox::from_id_salt("analysis_integrator")
                .selected_text(format!("{:?}", case.time_integration))
                .show_ui(ui, |ui| {
                    for method in [
                        TimeIntegrationMethod::NewmarkAverage,
                        TimeIntegrationMethod::NewmarkLinear,
                        TimeIntegrationMethod::HilberHughesTaylor,
                        TimeIntegrationMethod::WilsonTheta,
                    ] {
                        ui.selectable_value(
                            &mut case.time_integration,
                            method,
                            format!("{method:?}"),
                        );
                    }
                });
            if case.time_integration == TimeIntegrationMethod::HilberHughesTaylor {
                ui.horizontal(|ui| {
                    ui.label("HHT alpha [-1/3,0]:");
                    ui.add(egui::DragValue::new(&mut case.hht_alpha).speed(0.01));
                });
            }
            if case.time_integration == TimeIntegrationMethod::WilsonTheta {
                ui.horizontal(|ui| {
                    ui.label("Wilson theta (>=1.37):");
                    ui.add(egui::DragValue::new(&mut case.wilson_theta).speed(0.01));
                });
            }
            ui.horizontal(|ui| {
                ui.label("Rayleigh mass alpha:");
                ui.add(egui::DragValue::new(&mut case.rayleigh_damping.0).speed(0.001));
                ui.label("Stiffness beta:");
                ui.add(egui::DragValue::new(&mut case.rayleigh_damping.1).speed(0.001));
            });
            }
            history_load_entries(ui, model, case);
            ui.label("Accel U1/U2/U3 are global base acceleration. Pattern entries apply the joint forces and the frame distributed and point loads of that pattern, whatever its name. Ordinate × scale multiplies the selected excitation.");
            ui.label(
                "History patterns cannot contain self-weight, frame temperature or gravity loads, area or solid loads, or support motion; saving names the pattern and the load type.",
            );
        }
        _ => {
            ui.label("This imported case type is not editable here. Choose a supported type to create a replacement.");
        }
    }
}
impl SapGuiApp {
    pub(super) fn render_analysis_editor(&mut self, ctx: &egui::Context) {
        let mut state = std::mem::take(&mut self.analysis_editor);
        let mut open = state.open;
        if open {
            egui::Window::new("Analysis Cases and Functions").open(&mut open).default_width(690.0).default_height(760.0).vscroll(true).show(ctx,|ui| {
                ui.horizontal(|ui|{ui.selectable_value(&mut state.tab,0,"Cases");ui.selectable_value(&mut state.tab,1,"Functions");});
                ui.label(format!("Model units: {}. Time and period are seconds.",self.model.present_units.s2k_label()));
                if self.model.is_locked {ui.colored_label(Color32::YELLOW,"Use Edit > Unlock Model before saving definitions.");}
                let mut outcome=None;
                if state.tab==0 {
                    egui::ComboBox::from_id_salt("analysis_read_case").selected_text("Read existing case").show_ui(ui,|ui| {
                        for (name,case) in &self.model.load_cases {if ui.button(name).clicked() {state.case=case.clone();ui.close_menu();}}
                    });
                    case_controls(ui,&self.model,&mut state.case);
                    ui.horizontal(|ui| {
                        if ui.button("Save Analysis Case").clicked() {
                            outcome = Some(
                                sap_api::set_case_definition(&mut self.model, state.case.clone())
                                    .map(|_| format!("Saved case {}.", state.case.name))
                                    .map_err(|e| e.to_string()),
                            );
                        }
                        if ui.button("Delete Case").clicked() {
                            outcome = Some(delete_case(&mut self.model, &state.case.name));
                        }
                    });
                    ui.separator();
                    ui.label("Cases selected to run:");
                    let mut toggle=None;
                    for (name,case) in &self.model.load_cases {let mut run=case.run_flag;if ui.checkbox(&mut run,name).changed() {toggle=Some((name.clone(),run));}}
                    if let Some((name,run))=toggle {
                        if self.model.is_locked {outcome=Some(Err("Unlock the model before changing run selection.".into()));}
                        else {self.model.load_cases[&name].run_flag=run;self.model.invalidate_results();outcome=Some(Ok(format!("Case {name}: run={run}.")));if state.case.name==name {state.case.run_flag=run;}}
                    }
                    if !self.model.unmodeled_source.is_empty() {
                        ui.separator();
                        ui.colored_label(Color32::YELLOW, "Imported data that is not modeled. Affected cases will not analyze until an entry is discarded:");
                        let mut discard = None;
                        for (index, entry) in self.model.unmodeled_source.iter().enumerate() {
                            let scope = match &entry.scope {
                                sap_model::UnmodeledScope::AllCases => "all cases".to_string(),
                                sap_model::UnmodeledScope::NonlinearCases => "nonlinear cases".to_string(),
                                sap_model::UnmodeledScope::DynamicCases => "modal, spectrum and history cases".to_string(),
                                sap_model::UnmodeledScope::AllButLinearStatic => "all cases except linear static from zero".to_string(),
                                sap_model::UnmodeledScope::LoadPattern(name) => format!("cases using load pattern {name}"),
                                sap_model::UnmodeledScope::Case(name) => format!("case {name}"),
                            };
                            ui.horizontal(|ui| {
                                ui.label(format!("{} ({scope})", entry.description));
                                if ui.button("Discard and analyze without it").clicked() {
                                    discard = Some(index);
                                }
                            });
                        }
                        if let Some(index) = discard {
                            outcome = Some(
                                sap_api::discard_unmodeled_source(&mut self.model, index)
                                    .map(|entry| format!("Discarded: {}.", entry.description))
                                    .map_err(|e| e.to_string()),
                            );
                        }
                    }
                } else {
                    egui::ComboBox::from_id_salt("analysis_read_function").selected_text("Read existing function").show_ui(ui,|ui| {
                        for (name,function) in &self.model.functions {if ui.button(name).clicked() {state.function=function.clone();state.samples=function.points.iter().map(|(x,y)|format!("{x} {y}")).collect::<Vec<_>>().join("\n");ui.close_menu();}}
                    });
                    ui.horizontal(|ui| {ui.label("Name:");ui.text_edit_singleline(&mut state.function.name);});
                    ui.horizontal(|ui| {ui.selectable_value(&mut state.function.is_time_history,true,"Time history");ui.selectable_value(&mut state.function.is_time_history,false,"Response spectrum");});
                    if !state.function.is_time_history {ui.horizontal(|ui| {ui.label("Reference damping ratio:");ui.add(egui::DragValue::new(&mut state.function.damping_ratio).speed(0.005));});}
                    ui.label("One pair per line: time/value or period/spectral acceleration. Spaces or commas; # starts a comment.");
                    ui.add(egui::TextEdit::multiline(&mut state.samples).code_editor().desired_rows(9).desired_width(f32::INFINITY));
                    ui.horizontal(|ui| {
                        if ui.button("Save Dynamic Function").clicked() {
                            outcome = Some(save_function(&mut self.model, &state));
                        }
                        if ui.button("Delete Function").clicked() {
                            outcome = Some(delete_function(&mut self.model, &state.function.name));
                        }
                    });
                }
                if let Some(result)=outcome {self.status_msg=match result {Ok(message)=>{self.inspected_frame=None;self.sync_active_case();message},Err(error)=>format!("Definition error: {error}")};}
                ui.separator();ui.label(&self.status_msg);
            });
        }
        state.open = open;
        self.analysis_editor = state;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn sample_text_and_type_switch_do_not_silently_drop_bad_input() {
        assert_eq!(
            parse_samples("# time,value\n0, 0\n1 2 # peak\n").unwrap(),
            vec![(0., 0.), (1., 2.)]
        );
        assert!(parse_samples("0 1 2").is_err());
        let mut model = SapModel::new(Units::KipInF);
        model.case_results.insert("old".into(), Default::default());
        let before = model.clone();
        let mut editor = AnalysisEditor::default();
        editor.samples = "0 0\n0 1".into();
        assert!(save_function(&mut model, &editor).is_err());
        assert_eq!(model, before);
        editor.samples = "0 0\n1 2".into();
        save_function(&mut model, &editor).unwrap();
        assert_eq!(
            model.functions["FUNCTION1"].points,
            vec![(0., 0.), (1., 2.)]
        );
        assert!(model.case_results.is_empty());
        let history = new_case(LoadCaseType::LinearHistory, "H");
        assert!(history.loads.is_empty());
        assert!(history.spectrum_loads.is_empty());
    }
    #[test]
    fn a_modal_case_named_by_a_modal_history_cannot_be_deleted() {
        let mut model = SapModel::new(Units::KipInF);
        model
            .load_cases
            .insert("MODES".into(), LoadCase::modal_eigen("MODES", 3));
        let mut history = new_case(LoadCaseType::LinearHistory, "H");
        history.modal_case = Some("MODES".into());
        model.load_cases.insert("H".into(), history);
        let error = delete_case(&mut model, "MODES").unwrap_err();
        assert!(error.contains("referenced"), "{error}");
        assert!(model.load_cases.contains_key("MODES"));
        // The dependent case itself can go, and then the modal case can too.
        delete_case(&mut model, "H").unwrap();
        delete_case(&mut model, "MODES").unwrap();
    }
    #[test]
    fn modal_history_case_renders_with_a_modal_source() {
        let mut app = SapGuiApp::new(Some(PathBuf::from("/nonexistent/modal-history.s2k")));
        app.model = SapModel::new(Units::KipInF);
        app.analysis_editor.open = true;
        let mut case = new_case(LoadCaseType::LinearHistory, "H");
        case.modal_case = Some("MODAL".into());
        case.modal_damping = 0.05;
        app.analysis_editor.case = case;
        let ctx = egui::Context::default();
        let mut out = None;
        for time in [1., 2., 3.] {
            out = Some(ctx.run(
                egui::RawInput {
                    screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1400., 1200.))),
                    time: Some(time),
                    ..Default::default()
                },
                |ctx| app.render_analysis_editor(ctx),
            ));
        }
        let out = out.unwrap();
        let texts: Vec<String> = out
            .shapes
            .iter()
            .filter_map(|shape| match &shape.shape {
                egui::epaint::Shape::Text(text) => Some(text.galley.job.text.clone()),
                _ => None,
            })
            .collect();
        assert!(texts.iter().any(|t| t.contains("Modal damping ratio (constant")));
        assert!(texts.iter().any(|t| t.contains("Modal superposition of MODAL")));
        assert_eq!(
            app.analysis_editor.case.modal_case.as_deref(),
            Some("MODAL")
        );
    }
    #[test]
    fn modal_cases_as_the_editor_creates_them_save_and_analyze() {
        // A spring-supported mass: one mode, omega^2 = k / m.
        let mut model = SapModel::new(Units::KipInF);
        model.active_dofs = [true, false, false, false, false, false];
        model.add_point("1", 0.0, 0.0, 0.0);
        model.points["1"].spring[0] = 40.0;
        model.points["1"].mass[0] = 0.1;
        let period = 2.0 * std::f64::consts::PI * (0.1f64 / 40.0).sqrt();

        // The Eigen case a new Modal case starts as.
        let eigen = new_case(LoadCaseType::Modal, "EIG");
        assert!(eigen.loads.is_empty() && eigen.modal_sub_type == ModalSubType::Eigen);
        sap_api::set_case_definition(&mut model, eigen).unwrap();

        // Switching it to Ritz without a starting load is refused at save,
        // with the reason, and nothing is stored.
        let mut ritz = new_case(LoadCaseType::Modal, "RITZ");
        ritz.modal_sub_type = ModalSubType::Ritz;
        let error = sap_api::set_case_definition(&mut model, ritz.clone()).unwrap_err();
        assert!(error.to_string().contains("starting load"), "{error}");
        assert!(!model.load_cases.contains_key("RITZ"));

        // With the X acceleration as its starting load it saves.
        ritz.loads.push(LoadCaseAssignment { load_name: "UX".into(), scale_factor: 1.0 });
        sap_api::set_case_definition(&mut model, ritz).unwrap();

        sap_solver::analyze_model(&mut model).unwrap();
        for case in ["MODAL", "EIG", "RITZ"] {
            let modes = &model.case_results[case].modes;
            assert_eq!(modes.len(), 1, "{case}");
            assert!((modes[0].period_sec - period).abs() < 1e-12 * period, "{case}");
        }
    }

    #[test]
    fn editor_renders_case_types_and_pointer_click_saves_function() {
        let mut app = SapGuiApp::new(Some(PathBuf::from("/nonexistent/analysis-editor.s2k")));
        app.model = SapModel::new(Units::KipInF);
        app.analysis_editor.open = true;
        let ctx = egui::Context::default();
        let input = |time| egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1400., 1200.))),
            time: Some(time),
            ..Default::default()
        };
        let mut time = 0.;
        for kind in [
            LoadCaseType::LinearStatic,
            LoadCaseType::NonlinearStatic,
            LoadCaseType::Modal,
            LoadCaseType::Buckling,
            LoadCaseType::ResponseSpectrum,
            LoadCaseType::LinearHistory,
            LoadCaseType::NonlinearHistory,
            LoadCaseType::NonlinearDynamic,
        ] {
            app.analysis_editor.case = new_case(kind, "TEST");
            time += 1.;
            let out = ctx.run(input(time), |ctx| app.render_analysis_editor(ctx));
            assert!(!out.shapes.is_empty());
        }
        app.analysis_editor.tab = 1;
        time += 1.;
        let _ = ctx.run(input(time), |ctx| app.render_analysis_editor(ctx));
        time += 1.;
        let out = ctx.run(input(time), |ctx| app.render_analysis_editor(ctx));
        let pos = out
            .shapes
            .iter()
            .find_map(|shape| {
                if let egui::epaint::Shape::Text(text) = &shape.shape {
                    if text.galley.job.text == "Save Dynamic Function" {
                        Some(text.pos + text.galley.size() * 0.5)
                    } else {
                        None
                    }
                } else {
                    None
                }
            })
            .expect("function save button visible");
        let mut down = input(time + 1.);
        down.events = vec![
            egui::Event::PointerMoved(pos),
            egui::Event::PointerButton {
                pos,
                button: egui::PointerButton::Primary,
                pressed: true,
                modifiers: Default::default(),
            },
        ];
        let _ = ctx.run(down, |ctx| app.render_analysis_editor(ctx));
        let mut up = input(time + 1.1);
        up.events = vec![egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed: false,
            modifiers: Default::default(),
        }];
        let _ = ctx.run(up, |ctx| app.render_analysis_editor(ctx));
        assert!(app.model.functions.contains_key("FUNCTION1"));
    }

    // -- nonlinear static (pushover) case commands -------------------------

    fn push_model() -> SapModel {
        let mut m = SapModel::new(Units::KipInF);
        m.add_point("J", 0.0, 0.0, 0.0);
        m.add_point("K", 0.0, 0.0, 3.0);
        m.load_cases
            .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 4));
        let mut n1 = new_case(LoadCaseType::NonlinearStatic, "GRAV");
        n1.loads[0].load_name = "DEAD".into();
        m.load_cases.insert("GRAV".into(), n1);
        m
    }

    fn draft(name: &str) -> LoadCase {
        new_case(LoadCaseType::NonlinearStatic, name)
    }

    #[test]
    fn initial_case_commands_follow_the_rules() {
        let mut m = push_model();
        let mut c = draft("PUSH");
        assert_eq!(eligible_initial_cases(&m, &c), vec!["GRAV".to_string()]);
        set_initial_case(&m, &mut c, Some("GRAV")).unwrap();
        assert_eq!(c.initial_case.as_deref(), Some("GRAV"));
        set_initial_case(&m, &mut c, None).unwrap();
        assert!(c.initial_case.is_none());
        // Refusals leave the draft unchanged.
        for (initial, needle) in [
            ("PUSH", "itself"),
            ("GHOST", "does not exist"),
            ("MODAL", "not a nonlinear static case"),
        ] {
            let before = c.clone();
            let error = set_initial_case(&m, &mut c, Some(initial)).unwrap_err();
            assert!(error.contains(needle), "{error}");
            assert_eq!(c, before);
        }
        // A cycle: GRAV continues from PUSH, so PUSH cannot continue from GRAV,
        // and the chooser does not offer it.
        let mut grav = m.load_cases["GRAV"].clone();
        grav.initial_case = Some("PUSH".into());
        m.load_cases.insert("GRAV".into(), grav);
        assert!(eligible_initial_cases(&m, &c).is_empty());
        let error = set_initial_case(&m, &mut c, Some("GRAV")).unwrap_err();
        assert!(error.contains("cycle"), "{error}");
        // Not a nonlinear static draft.
        let mut lin = new_case(LoadCaseType::LinearStatic, "L");
        assert!(set_initial_case(&m, &mut lin, Some("GRAV")).is_err());
        assert!(lin.initial_case.is_none());
    }

    #[test]
    fn the_stiffness_of_a_modal_or_linear_case_may_come_from_a_nonlinear_static_case() {
        // v19 help "Stiffness to Use". `push_model` has the nonlinear static
        // case GRAV; a draft may name it, and nothing else.
        let m = push_model();
        for kind in [LoadCaseType::Modal, LoadCaseType::LinearStatic] {
            let mut c = new_case(kind, "X");
            set_stiffness_case(&m, &mut c, Some("GRAV")).unwrap();
            assert_eq!(c.initial_case.as_deref(), Some("GRAV"));
            // The definition is one the command layer accepts.
            sap_api::validate_case_definition(&m, &c).unwrap();
            set_stiffness_case(&m, &mut c, None).unwrap();
            assert!(c.initial_case.is_none());
            assert!(set_stiffness_case(&m, &mut c, Some("GHOST"))
                .unwrap_err()
                .contains("does not exist"));
            assert!(set_stiffness_case(&m, &mut c, Some("X"))
                .unwrap_err()
                .contains("own stiffness"));
            assert!(c.initial_case.is_none());
        }
        // A source that is not a nonlinear static case is refused.
        let mut with_lin = m.clone();
        with_lin
            .load_cases
            .insert("LIN".into(), new_case(LoadCaseType::LinearStatic, "LIN"));
        let mut c = new_case(LoadCaseType::Modal, "X");
        assert!(set_stiffness_case(&with_lin, &mut c, Some("LIN"))
            .unwrap_err()
            .contains("not a nonlinear static case"));
        // Other case types have no such choice.
        let mut buckling = new_case(LoadCaseType::Buckling, "B");
        assert!(set_stiffness_case(&m, &mut buckling, Some("GRAV")).is_err());
    }

    #[test]
    fn load_application_results_and_step_commands_validate() {
        let m = push_model();
        let mut c = draft("PUSH");
        set_displacement_control(&m, &mut c, "K", 0, 6.0, true).unwrap();
        assert_eq!(
            c.displacement_control,
            Some(DisplacementControl {
                target: 6.0,
                monitored: Some(("K".into(), 0)),
                conjugate: true
            })
        );
        let before = c.clone();
        for (joint, dof, target) in [
            ("GHOST", 0, 1.0),
            ("K", 6, 1.0),
            ("K", 0, 0.0),
            ("K", 0, -2.0),
            ("K", 0, f64::NAN),
        ] {
            assert!(set_displacement_control(&m, &mut c, joint, dof, target, false).is_err());
            assert_eq!(c, before);
        }
        // Large displacements: refused by the form. P-Delta: accepted since
        // the sixth wave (the solver decides whether the model carries it).
        c.nl_geom = NLGeomType::LargeDisplacements;
        let error = set_displacement_control(&m, &mut c, "K", 1, 1.0, false).unwrap_err();
        assert!(error.contains("large displacements"), "{error}");
        c.nl_geom = NLGeomType::PDelta;
        set_displacement_control(&m, &mut c, "K", 1, 1.0, false).unwrap();
        assert_eq!(
            c.displacement_control.as_ref().and_then(|d| d.monitored.clone()),
            Some(("K".to_string(), 1))
        );
        c.nl_geom = NLGeomType::None;
        set_full_load(&mut c);
        assert!(c.displacement_control.is_none());
        let mut lin = new_case(LoadCaseType::LinearStatic, "L");
        assert!(set_displacement_control(&m, &mut lin, "K", 0, 1.0, true).is_err());

        // Results saved.
        let multiple = ResultsSaved::MultipleStates {
            min_saved: 6,
            max_saved: 10,
            positive_only: false,
        };
        set_results_saved(&mut c, multiple).unwrap();
        assert_eq!(c.nl_static.results_saved, multiple);
        for (min_saved, max_saved) in [(0, 5), (7, 6), (1, 100_001)] {
            let bad = ResultsSaved::MultipleStates {
                min_saved,
                max_saved,
                positive_only: true,
            };
            assert!(set_results_saved(&mut c, bad).is_err());
            assert_eq!(c.nl_static.results_saved, multiple);
        }
        set_results_saved(&mut c, ResultsSaved::FinalState).unwrap();
        assert!(set_results_saved(&mut lin, multiple).is_err());

        // Step limits.
        set_step_limits(&mut c, 500, 80).unwrap();
        assert_eq!((c.nl_static.max_total_steps, c.nl_static.max_null_steps), (500, 80));
        for (total, null) in [(0, 80), (500, 0), (1_000_001, 80), (500, 1_000_001)] {
            assert!(set_step_limits(&mut c, total, null).is_err());
        }
        assert_eq!((c.nl_static.max_total_steps, c.nl_static.max_null_steps), (500, 80));
        assert!(set_step_limits(&mut lin, 10, 10).is_err());

        // Hinge unloading: only the first method.
        set_hinge_unloading(&mut c, HingeUnloading::UnloadEntireStructure).unwrap();
        for method in [
            HingeUnloading::ApplyLocalRedistribution,
            HingeUnloading::RestartUsingSecantStiffness,
        ] {
            let error = set_hinge_unloading(&mut c, method).unwrap_err();
            assert!(error.contains("not implemented"), "{error}");
            assert_eq!(
                c.nl_static.hinge_unloading,
                HingeUnloading::UnloadEntireStructure
            );
        }
    }

    #[test]
    fn load_row_commands_add_accel_and_mode_and_refuse_mixing() {
        let mut m = push_model();
        let mut c = draft("PUSH");
        // Pattern row.
        assert_eq!(c.loads.len(), 1);
        add_pattern_load(&m, &mut c, "DEAD", 2.0).unwrap();
        assert!(add_pattern_load(&m, &mut c, "GHOST", 1.0).is_err());
        assert!(add_pattern_load(&m, &mut c, "DEAD", f64::NAN).is_err());
        // Accel rows: UX, UY, UZ only.
        add_accel_load(&m, &mut c, "UY", -1.0).unwrap();
        for bad in ["RX", "U4", "ux", "DEAD"] {
            assert!(add_accel_load(&m, &mut c, bad, 1.0).is_err(), "{bad}");
        }
        assert!(add_accel_load(&m, &mut c, "UX", f64::INFINITY).is_err());
        assert_eq!(c.loads.len(), 3);
        assert!(is_accel(&c.loads[2]) && !is_accel(&c.loads[0]));
        // Mode rows are refused while other loads exist, and the other way round.
        let before = c.clone();
        let error = add_mode_load(&m, &mut c, "MODAL", 1, 1.0).unwrap_err();
        assert!(error.contains("cannot be mixed"), "{error}");
        assert_eq!(c, before);
        c.loads.clear();
        add_mode_load(&m, &mut c, "MODAL", 2, 1.5).unwrap();
        assert_eq!(c.modal_case.as_deref(), Some("MODAL"));
        assert_eq!(c.nl_static.mode_loads[0].mode, 2);
        let before = c.clone();
        assert!(add_pattern_load(&m, &mut c, "DEAD", 1.0).unwrap_err().contains("Mode loads"));
        assert!(add_accel_load(&m, &mut c, "UX", 1.0).unwrap_err().contains("Mode loads"));
        for (modal, mode, scale, needle) in [
            ("GHOST", 1, 1.0, "not an existing Modal"),
            ("GRAV", 1, 1.0, "not an existing Modal"),
            ("MODAL", 0, 1.0, "outside 1 to 4"),
            ("MODAL", 5, 1.0, "outside 1 to 4"),
            ("MODAL", 2, 1.0, "already loaded"),
            ("MODAL", 3, f64::NAN, "finite"),
        ] {
            let error = add_mode_load(&m, &mut c, modal, mode, scale).unwrap_err();
            assert!(error.contains(needle), "{error}");
            assert_eq!(c, before);
        }
        add_mode_load(&m, &mut c, "MODAL", 3, 1.0).unwrap();
        assert_eq!(c.nl_static.mode_loads.len(), 2);
        // The selector offers run Modal cases only.
        assert_eq!(modal_cases_for_modes(&m, &c), vec!["MODAL".to_string()]);
        m.load_cases.get_mut("MODAL").unwrap().run_flag = false;
        assert!(modal_cases_for_modes(&m, &c).is_empty());
        let mut other = draft("OTHER");
        other.loads.clear();
        assert!(add_mode_load(&m, &mut other, "MODAL", 1, 1.0).unwrap_err().contains("not set to run"));
        // Removing the last Mode row clears the modal case.
        remove_mode_load(&mut c, 0).unwrap();
        assert_eq!(c.modal_case.as_deref(), Some("MODAL"));
        remove_mode_load(&mut c, 0).unwrap();
        assert!(c.modal_case.is_none() && c.nl_static.mode_loads.is_empty());
        assert!(remove_mode_load(&mut c, 0).is_err());
        // Other case types take none of these rows.
        let mut lin = new_case(LoadCaseType::LinearStatic, "L");
        assert!(add_accel_load(&m, &mut lin, "UX", 1.0).is_err());
        assert!(add_mode_load(&m, &mut lin, "MODAL", 1, 1.0).is_err());
        assert!(add_pattern_load(&m, &mut lin, "DEAD", 1.0).is_err());
    }

    #[test]
    fn a_pushover_chain_drafted_with_the_commands_saves_and_a_locked_model_refuses() {
        let mut m = push_model();
        let mut c = draft("PUSH");
        set_initial_case(&m, &mut c, Some("GRAV")).unwrap();
        set_displacement_control(&m, &mut c, "K", 0, 6.0, true).unwrap();
        set_results_saved(
            &mut c,
            ResultsSaved::MultipleStates {
                min_saved: 6,
                max_saved: 10,
                positive_only: false,
            },
        )
        .unwrap();
        set_step_limits(&mut c, 200, 50).unwrap();
        add_accel_load(&m, &mut c, "UX", 1.0).unwrap();
        // Locked: refused, the model is unchanged.
        m.is_locked = true;
        let before = m.clone();
        let error = sap_api::set_case_definition(&mut m, c.clone()).unwrap_err();
        assert!(error.to_string().contains("locked"), "{error}");
        assert_eq!(m, before);
        m.is_locked = false;
        sap_api::set_case_definition(&mut m, c.clone()).unwrap();
        assert_eq!(m.load_cases["PUSH"], c);
        // A Mode-load case too.
        let mut modes = draft("MODEPUSH");
        modes.loads.clear();
        add_mode_load(&m, &mut modes, "MODAL", 1, 1.0).unwrap();
        sap_api::set_case_definition(&mut m, modes.clone()).unwrap();
        assert_eq!(m.load_cases["MODEPUSH"], modes);
        // The initial case cannot be deleted while PUSH continues from it.
        assert!(delete_case(&mut m, "GRAV").unwrap_err().contains("referenced"));
    }

    fn all_texts(app: &mut SapGuiApp, passes: usize) -> Vec<String> {
        let ctx = egui::Context::default();
        let mut out = None;
        for pass in 0..passes {
            out = Some(ctx.run(
                egui::RawInput {
                    screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1400., 2400.))),
                    time: Some(1.0 + pass as f64),
                    ..Default::default()
                },
                |ctx| app.render_analysis_editor(ctx),
            ));
        }
        out.unwrap()
            .shapes
            .iter()
            .filter_map(|s| match &s.shape {
                egui::epaint::Shape::Text(t) => Some(t.galley.job.text.clone()),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn nonlinear_static_editor_renders_every_new_control() {
        let mut app = SapGuiApp::new(Some(PathBuf::from("/nonexistent/pushover.s2k")));
        app.model = push_model();
        app.analysis_editor.open = true;
        let mut c = draft("PUSH");
        set_initial_case(&app.model, &mut c, Some("GRAV")).unwrap();
        set_displacement_control(&app.model, &mut c, "K", 0, 6.0, true).unwrap();
        set_results_saved(&mut c, ResultsSaved::multiple_default()).unwrap();
        add_accel_load(&app.model, &mut c, "UX", 1.0).unwrap();
        app.analysis_editor.case = c.clone();
        let t = all_texts(&mut app, 8);
        for needle in [
            "Continue from case:",
            "Loads applied:",
            "Load Pattern",
            "Accel",
            "Add Load Pattern",
            "Add Accel",
            "Add Mode",
            "Full load",
            "Displacement control",
            "Monitored joint",
            "Target displacement",
            "Use conjugate displacement",
            "Final state only",
            "Multiple states",
            "Minimum number of saved states",
            "Save positive increments only",
            "Maximum total steps",
            "Maximum null steps",
            "Unload entire structure",
            "Apply local redistribution",
            "Restart using secant stiffness",
        ] {
            assert!(t.iter().any(|s| s == needle || s.contains(needle)), "{needle}");
        }
        assert!(t.iter().any(|s| s == "GRAV"), "initial case shown");
        assert_eq!(
            t.iter().filter(|s| s.contains("not available: not implemented")).count(),
            2
        );
        // The rendering did not alter a valid draft.
        assert_eq!(app.analysis_editor.case, c);

        // A Mode draft shows its row and modal case.
        let mut modes = draft("MODEPUSH");
        modes.loads.clear();
        add_mode_load(&app.model, &mut modes, "MODAL", 2, 1.0).unwrap();
        app.analysis_editor.case = modes.clone();
        let t = all_texts(&mut app, 8);
        assert!(t.iter().any(|s| s == "of modal case"));
        assert!(t.iter().any(|s| s == "MODAL"));
        assert_eq!(app.analysis_editor.case, modes);
    }

    #[test]
    fn a_click_on_add_accel_adds_a_row_and_a_refusal_is_shown() {
        let mut app = SapGuiApp::new(Some(PathBuf::from("/nonexistent/pushover.s2k")));
        app.model = push_model();
        app.analysis_editor.open = true;
        app.analysis_editor.case = draft("PUSH");
        let ctx = egui::Context::default();
        let input = |time: f64| egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1400., 2400.))),
            time: Some(time),
            ..Default::default()
        };
        let press = |app: &mut SapGuiApp, label: &str, time: f64| {
            let _ = ctx.run(input(time), |ctx| app.render_analysis_editor(ctx));
            let out = ctx.run(input(time + 0.1), |ctx| app.render_analysis_editor(ctx));
            let pos = out
                .shapes
                .iter()
                .find_map(|s| match &s.shape {
                    egui::epaint::Shape::Text(t) if t.galley.job.text == label => {
                        Some(t.pos + t.galley.size() * 0.5)
                    }
                    _ => None,
                })
                .unwrap_or_else(|| panic!("'{label}' not visible"));
            let mut down = input(time + 0.2);
            down.events = vec![
                egui::Event::PointerMoved(pos),
                egui::Event::PointerButton {
                    pos,
                    button: egui::PointerButton::Primary,
                    pressed: true,
                    modifiers: Default::default(),
                },
            ];
            let _ = ctx.run(down, |ctx| app.render_analysis_editor(ctx));
            let mut up = input(time + 0.3);
            up.events = vec![egui::Event::PointerButton {
                pos,
                button: egui::PointerButton::Primary,
                pressed: false,
                modifiers: Default::default(),
            }];
            let _ = ctx.run(up, |ctx| app.render_analysis_editor(ctx));
        };
        press(&mut app, "Add Accel", 1.0);
        assert_eq!(app.analysis_editor.case.loads.len(), 2);
        assert!(is_accel(&app.analysis_editor.case.loads[1]));
        // Add Mode is refused while load rows exist; the draft is unchanged.
        let before = app.analysis_editor.case.clone();
        press(&mut app, "Add Mode", 10.0);
        assert_eq!(app.analysis_editor.case, before);
        // Choosing Displacement control sets the default monitored joint.
        press(&mut app, "Displacement control", 20.0);
        let control = app.analysis_editor.case.displacement_control.clone().unwrap();
        assert_eq!(control.monitored, Some(("J".to_string(), 0)));
        assert!(control.conjugate);
        press(&mut app, "Full load", 30.0);
        assert!(app.analysis_editor.case.displacement_control.is_none());
    }

    /// D77: the history form shows and writes the kind of each entry. A
    /// pattern named U1 and the acceleration U1 are different rows, both survive
    /// a frame unchanged, and "Add Function Assignment" appends an acceleration
    /// without disturbing the kinds of the rows above it.
    #[test]
    fn history_form_shows_and_keeps_the_kind_of_each_entry() {
        let mut model = SapModel::new(Units::KnMC);
        model.load_patterns.insert(
            "U1".into(),
            sap_model::LoadPattern {
                name: "U1".into(),
                pat_type: sap_model::LoadPatternType::Other,
                self_wt_multiplier: 0.0,
            },
        );
        let mut case = LoadCase::linear_static("H", "U1", 1.0);
        case.case_type = LoadCaseType::LinearHistory;
        case.loads.clear();
        case.push_history_load("F", "U1", HistoryLoadKind::Pattern, 1.0);
        case.push_history_load("F", "U1", HistoryLoadKind::Acceleration, 2.0);
        let ctx = egui::Context::default();
        let mut time = 0.0;
        let mut frame = |case: &mut LoadCase, events: Vec<egui::Event>| {
            time += 0.5;
            ctx.run(
                egui::RawInput {
                    screen_rect: Some(egui::Rect::from_min_size(
                        egui::Pos2::ZERO,
                        egui::Vec2::new(900.0, 400.0),
                    )),
                    time: Some(time),
                    events,
                    ..Default::default()
                },
                |ctx| {
                    egui::CentralPanel::default().show(ctx, |ui| {
                        history_load_entries(ui, &model, case);
                    });
                },
            )
        };
        let _ = frame(&mut case, vec![]);
        let out = frame(&mut case, vec![]);
        let shown: Vec<String> = out
            .shapes
            .iter()
            .filter_map(|s| match &s.shape {
                egui::epaint::Shape::Text(t) => Some(t.galley.job.text.clone()),
                _ => None,
            })
            .collect();
        for want in ["Pattern U1", "Accel U1"] {
            assert!(shown.iter().any(|t| t == want), "{want} in {shown:?}");
        }
        assert_eq!(case.history_kind(0), HistoryLoadKind::Pattern);
        assert_eq!(case.history_kind(1), HistoryLoadKind::Acceleration);
        let at = out
            .shapes
            .iter()
            .find_map(|s| match &s.shape {
                egui::epaint::Shape::Text(t) if t.galley.job.text == "Add Function Assignment" => {
                    Some(t.pos + t.galley.size() * 0.5)
                }
                _ => None,
            })
            .expect("Add button");
        let button = |pressed| egui::Event::PointerButton {
            pos: at,
            button: egui::PointerButton::Primary,
            pressed,
            modifiers: Default::default(),
        };
        let _ = frame(&mut case, vec![egui::Event::PointerMoved(at), button(true)]);
        let _ = frame(&mut case, vec![button(false)]);
        assert_eq!(case.history_loads.len(), 3);
        assert_eq!(case.history_kind(0), HistoryLoadKind::Pattern);
        assert_eq!(case.history_kind(1), HistoryLoadKind::Acceleration);
        assert_eq!(case.history_kind(2), HistoryLoadKind::Acceleration);
        assert_eq!(case.history_loads[2].1, "U1");
    }
}
