//! Validated native analysis definitions. These are not Windows COM signatures.
use sap_model::{
    DynamicFunction, LoadCase, LoadCaseType, ModalSubType, NLGeomType, SapModel,
    TimeIntegrationMethod,
};

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum DefinitionError {
    #[error("model is locked; unlock it before editing")]
    Locked,
    #[error("invalid definition: {0}")]
    Invalid(String),
}
fn invalid(reason: impl Into<String>) -> DefinitionError {
    DefinitionError::Invalid(reason.into())
}
fn name(value: &str) -> Result<(), DefinitionError> {
    if !super::is_valid_name(value) {
        return Err(invalid(
            "name must be nonempty, trimmed, and contain no quotes or control characters",
        ));
    }
    Ok(())
}
fn unlocked(model: &SapModel) -> Result<(), DefinitionError> {
    if model.is_locked {
        Err(DefinitionError::Locked)
    } else {
        Ok(())
    }
}
fn validate_function(function: &DynamicFunction) -> Result<(), DefinitionError> {
    name(&function.name)?;
    if !function.damping_ratio.is_finite() || !(0.0..1.0).contains(&function.damping_ratio) {
        return Err(invalid("function damping ratio must be in [0,1)"));
    }
    if function.points.len() < 2
        || function
            .points
            .iter()
            .any(|(x, y)| !x.is_finite() || !y.is_finite() || *x < 0.0)
        || function.points.windows(2).any(|p| p[0].0 >= p[1].0)
    {
        return Err(invalid(
            "provide at least two finite samples with nonnegative, strictly increasing abscissae",
        ));
    }
    if !function.is_time_history && function.points.iter().any(|(_, y)| *y < 0.0) {
        return Err(invalid("response spectrum ordinates must be nonnegative"));
    }
    Ok(())
}
/// Save a function atomically, preserving existing references and results on error.
pub fn set_function_definition(
    model: &mut SapModel,
    function: DynamicFunction,
) -> Result<(), DefinitionError> {
    unlocked(model)?;
    validate_function(&function)?;
    for case in model.load_cases.values() {
        if (!function.is_time_history
            && case
                .history_loads
                .iter()
                .any(|(f, _, _)| f == &function.name))
            || (function.is_time_history
                && case
                    .spectrum_loads
                    .iter()
                    .any(|(f, _, _)| f == &function.name))
        {
            return Err(invalid(format!(
                "function kind conflicts with case '{}'",
                case.name
            )));
        }
    }
    model.functions.insert(function.name.clone(), function);
    model.invalidate_results();
    Ok(())
}
/// Ritz starting loads and their generation controls (CSI Analysis Reference,
/// "Ritz-Vector Analysis"): starting loads are load patterns or the
/// accelerations `UX`, `UY`, `UZ`; a pattern with one of those names would be
/// ambiguous; controls name listed loads and targets lie in `[0, 1]`.
fn validate_ritz_controls(model: &SapModel, case: &LoadCase) -> Result<(), DefinitionError> {
    for (i, load) in case.loads.iter().enumerate() {
        if sap_model::ritz::acceleration_direction(&load.load_name).is_some()
            && model.load_patterns.contains_key(&load.load_name)
        {
            return Err(invalid(format!(
                "'{}' is both a load pattern and an acceleration load name; rename the pattern",
                load.load_name
            )));
        }
        if case.loads[..i].iter().any(|l| l.load_name == load.load_name) {
            return Err(invalid(format!(
                "starting load '{}' is listed more than once",
                load.load_name
            )));
        }
    }
    if case.modal_sub_type != ModalSubType::Ritz && !case.ritz_controls.is_empty() {
        return Err(invalid("Eigen cases take no Ritz generation controls"));
    }
    for (i, control) in case.ritz_controls.iter().enumerate() {
        if !case
            .loads
            .iter()
            .any(|l| l.load_name == control.load_name)
        {
            return Err(invalid(format!(
                "generation control names '{}', which is not a listed starting load",
                control.load_name
            )));
        }
        let target = control.target_dynamic_participation;
        if !target.is_finite() || !(0.0..=1.0).contains(&target) {
            return Err(invalid(format!(
                "dynamic participation target {target} for '{}' must be in [0, 1]",
                control.load_name
            )));
        }
        if case.ritz_controls[..i]
            .iter()
            .any(|c| c.load_name == control.load_name)
        {
            return Err(invalid(format!(
                "two generation controls name '{}'",
                control.load_name
            )));
        }
    }
    Ok(())
}
/// Validate definitions represented by this native editor; solver applicability checks still apply.
/// The initial case of a nonlinear static case: an existing, different
/// nonlinear static case, with no cycle through the initial cases.
fn validate_static_initial_case(
    model: &SapModel,
    case: &LoadCase,
    initial: &str,
) -> Result<(), DefinitionError> {
    let first = model
        .load_cases
        .get(initial)
        .filter(|c| c.name != case.name)
        .ok_or_else(|| invalid(format!("initial case '{initial}' is not an existing, different case")))?;
    if first.case_type != LoadCaseType::NonlinearStatic {
        return Err(invalid(format!(
            "initial case '{initial}' is not a nonlinear static case; a nonlinear static case can only continue from another nonlinear static case"
        )));
    }
    // Follow the chain by name as the model would hold it with this
    // definition: a stored case may already continue from this name even
    // when the case itself is new.
    let mut seen = vec![case.name.as_str()];
    let mut current = Some(first.name.as_str());
    while let Some(next) = current {
        if seen.contains(&next) {
            return Err(invalid(format!(
                "initial case '{initial}' leads back to '{next}': initial cases must not form a cycle"
            )));
        }
        seen.push(next);
        current = model
            .load_cases
            .get(next)
            .and_then(|c| c.initial_case.as_deref());
    }
    Ok(())
}

/// Settings particular to nonlinear static cases: saved states, step limits,
/// hinge unloading, and Mode loads.
fn validate_nl_static(model: &SapModel, case: &LoadCase) -> Result<(), DefinitionError> {
    use sap_model::hinge::{HingeUnloading, ResultsSaved};
    let params = &case.nl_static;
    if let ResultsSaved::MultipleStates {
        min_saved,
        max_saved,
        ..
    } = params.results_saved
    {
        if min_saved == 0 || max_saved < min_saved || max_saved > 100_000 {
            return Err(invalid(
                "multiple saved states require 1 <= minimum <= maximum <= 100000",
            ));
        }
    }
    if params.max_total_steps == 0 || params.max_total_steps > 1_000_000 {
        return Err(invalid("maximum total steps must be 1..1000000"));
    }
    if params.max_null_steps == 0 || params.max_null_steps > 1_000_000 {
        return Err(invalid("maximum null steps must be 1..1000000"));
    }
    if params.hinge_unloading != HingeUnloading::UnloadEntireStructure {
        return Err(invalid(format!(
            "hinge unloading {:?} is not implemented; only \"unload entire structure\" is",
            params.hinge_unloading
        )));
    }
    if params.mode_loads.is_empty() {
        if case.modal_case.is_some() {
            return Err(invalid(
                "a modal case is named but the case has no Mode load",
            ));
        }
        return Ok(());
    }
    if !case.loads.is_empty() {
        return Err(invalid(
            "Mode loads cannot be mixed with load patterns or Accel loads in one case",
        ));
    }
    let modal_name = case
        .modal_case
        .as_ref()
        .ok_or_else(|| invalid("Mode loads need the Modal case that supplies the modes"))?;
    let modal = model
        .load_cases
        .get(modal_name)
        .filter(|m| m.case_type == LoadCaseType::Modal && m.name != case.name)
        .ok_or_else(|| invalid("select an existing Modal case for the Mode loads"))?;
    if !modal.run_flag {
        return Err(invalid("the Modal case of the Mode loads must be run"));
    }
    for (i, load) in params.mode_loads.iter().enumerate() {
        if load.mode == 0 || load.mode > modal.max_modes {
            return Err(invalid(format!(
                "Mode load {}: mode {} is outside 1..{} (the maximum modes of '{modal_name}')",
                i + 1,
                load.mode,
                modal.max_modes
            )));
        }
        if !load.scale_factor.is_finite() {
            return Err(invalid(format!(
                "Mode load {}: scale factor is not finite",
                i + 1
            )));
        }
        if params.mode_loads[..i].iter().any(|l| l.mode == load.mode) {
            return Err(invalid(format!(
                "Mode {} is loaded twice; combine the scale factors",
                load.mode
            )));
        }
    }
    Ok(())
}
pub fn validate_case_definition(model: &SapModel, case: &LoadCase) -> Result<(), DefinitionError> {
    name(&case.name)?;
    if model.load_combos.contains_key(&case.name) {
        return Err(invalid("case name collides with a load combination"));
    }
    if !matches!(
        case.case_type,
        LoadCaseType::LinearStatic
            | LoadCaseType::NonlinearStatic
            | LoadCaseType::Modal
            | LoadCaseType::Buckling
            | LoadCaseType::ResponseSpectrum
            | LoadCaseType::LinearHistory
            | LoadCaseType::LinearDynamic
            | LoadCaseType::NonlinearHistory
            | LoadCaseType::NonlinearDynamic
    ) {
        return Err(invalid("case type is not supported by this editor"));
    }
    if let Some(initial) = &case.initial_case {
        match case.case_type {
            // A spectrum case names its modal source here.
            LoadCaseType::ResponseSpectrum => {}
            LoadCaseType::NonlinearStatic => validate_static_initial_case(model, case, initial)?,
            _ => {
                return Err(invalid(
                    "initial-state cases are supported only by nonlinear static cases (and as the modal source of a response-spectrum case)",
                ));
            }
        }
    }
    if case.case_type != LoadCaseType::NonlinearStatic
        && model.load_cases.values().any(|c| {
            c.case_type == LoadCaseType::NonlinearStatic
                && c.initial_case.as_ref() == Some(&case.name)
        })
    {
        return Err(invalid(
            "case is the initial case of a nonlinear static case and must stay a nonlinear static case",
        ));
    }
    if case.case_type != LoadCaseType::NonlinearStatic && !case.nl_static.is_default() {
        return Err(invalid(
            "saved states, step limits and modal loads belong to nonlinear static cases",
        ));
    }
    if case.case_type != LoadCaseType::Modal
        && model.load_cases.values().any(|c| {
            c.case_type == LoadCaseType::ResponseSpectrum
                && c.initial_case.as_ref() == Some(&case.name)
        })
    {
        return Err(invalid(
            "case is referenced as a modal source by a response-spectrum case",
        ));
    }
    if case.case_type != LoadCaseType::Modal
        && model
            .load_cases
            .values()
            .any(|c| c.modal_case.as_ref() == Some(&case.name))
    {
        return Err(invalid(
            "case is referenced as a modal source by a modal time-history case",
        ));
    }
    if case.case_type == LoadCaseType::NonlinearStatic {
        validate_nl_static(model, case)?;
    }
    if let Some(modal_name) = case
        .modal_case
        .as_ref()
        .filter(|_| case.case_type != LoadCaseType::NonlinearStatic)
    {
        if case.case_type != LoadCaseType::LinearHistory {
            return Err(invalid(
                "only a linear modal time-history case can superpose the modes of a modal case, or supply the modes of a nonlinear static case with Mode loads",
            ));
        }
        let modal = model
            .load_cases
            .get(modal_name)
            .filter(|m| m.case_type == LoadCaseType::Modal && m.name != case.name)
            .ok_or_else(|| invalid("select an existing, different Modal case for superposition"))?;
        if !modal.run_flag {
            return Err(invalid("the Modal case for superposition must be run"));
        }
        if !case.modal_damping.is_finite() || !(0.0..1.0).contains(&case.modal_damping) {
            return Err(invalid("modal damping ratio must satisfy 0 <= ratio < 1"));
        }
        if case.rayleigh_damping != (0.0, 0.0) {
            return Err(invalid(
                "modal superposition uses the modal damping ratio; Rayleigh coefficients must be zero",
            ));
        }
    }
    if let Some(control) = &case.displacement_control {
        let monitored_ok = control
            .monitored
            .as_ref()
            .is_some_and(|(point, dof)| model.points.contains_key(point) && *dof < 6);
        if case.case_type != LoadCaseType::NonlinearStatic
            || case.nl_geom != NLGeomType::None
            || !control.target.is_finite()
            || control.target <= 0.0
            || !monitored_ok
        {
            return Err(invalid(
                "displacement control needs a small-displacement nonlinear static case, a positive finite target and an existing monitored joint displacement",
            ));
        }
    }
    for load in &case.loads {
        let ritz_acceleration = case.case_type == LoadCaseType::Modal
            && case.modal_sub_type == ModalSubType::Ritz
            && sap_model::ritz::acceleration_direction(&load.load_name).is_some()
            && load.scale_factor.is_finite();
        if ritz_acceleration {
            continue;
        }
        // Nonlinear static: Accel loads UX, UY, UZ (not rotations).
        if case.case_type == LoadCaseType::NonlinearStatic
            && sap_model::ritz::acceleration_direction(&load.load_name).is_some()
        {
            if !load.scale_factor.is_finite() {
                return Err(invalid("nonfinite scale factor of an Accel load"));
            }
            if let Some(clash) = model
                .load_patterns
                .keys()
                .find(|p| sap_model::ritz::acceleration_direction(p).is_some())
            {
                return Err(invalid(format!(
                    "load pattern '{clash}' is named like an acceleration load; rename it before using Accel loads"
                )));
            }
            continue;
        }
        if !model.load_patterns.contains_key(&load.load_name) || !load.scale_factor.is_finite() {
            return Err(invalid(format!(
                "undefined static load pattern or nonfinite scale: '{}'",
                load.load_name
            )));
        }
    }
    if matches!(
        case.case_type,
        LoadCaseType::ResponseSpectrum
            | LoadCaseType::LinearHistory
            | LoadCaseType::LinearDynamic
            | LoadCaseType::NonlinearHistory
            | LoadCaseType::NonlinearDynamic
    ) && case.nl_geom != NLGeomType::None
    {
        return Err(invalid(
            "dynamic/spectrum cases cannot request geometric nonlinearity",
        ));
    }
    if case.case_type != LoadCaseType::Modal && !case.ritz_controls.is_empty() {
        return Err(invalid("only modal Ritz cases take generation controls"));
    }
    match case.case_type {
        LoadCaseType::LinearStatic | LoadCaseType::NonlinearStatic | LoadCaseType::Buckling => {
            if case.loads.is_empty()
                && !(case.case_type == LoadCaseType::NonlinearStatic
                    && !case.nl_static.mode_loads.is_empty())
            {
                return Err(invalid("provide at least one static load pattern"));
            }
            if !case.history_loads.is_empty() || !case.spectrum_loads.is_empty() {
                return Err(invalid("static case cannot carry dynamic assignments"));
            }
            if case.case_type == LoadCaseType::NonlinearStatic {
                if !matches!(case.nl_geom, NLGeomType::None | NLGeomType::PDelta) {
                    return Err(invalid("large-displacement analysis is not implemented"));
                }
                if case.max_iterations == 0
                    || case.max_iterations > 10000
                    || !case.convergence_tol.is_finite()
                    || case.convergence_tol <= 0.0
                    || case.convergence_tol >= 1.0
                {
                    return Err(invalid(
                        "nonlinear iteration limit must be 1..10000 and tolerance in (0,1)",
                    ));
                }
            } else if case.nl_geom != NLGeomType::None {
                return Err(invalid(
                    "geometric nonlinearity requires a nonlinear-static case",
                ));
            }
            if case.case_type == LoadCaseType::Buckling
                && (case.num_buckling_modes == 0 || case.num_buckling_modes > 10000)
            {
                return Err(invalid("buckling mode count must be 1..10000"));
            }
        }
        LoadCaseType::Modal => {
            if case.min_modes != 1 {
                return Err(invalid(
                    "minimum mode count is not an implemented control; use 1",
                ));
            }
            if case.modal_sub_type == ModalSubType::Eigen && !case.loads.is_empty() {
                return Err(invalid(
                    "Eigen cases cannot have starting load patterns; select Ritz",
                ));
            }
            if case.max_modes == 0
                || case.max_modes > 10000
                || case.min_modes == 0
                || case.min_modes > case.max_modes
            {
                return Err(invalid(
                    "require 1 <= minimum modes <= maximum modes <= 10000",
                ));
            }
            if case.shift_freq_hz != 0.0 {
                return Err(invalid("nonzero modal frequency shift is not implemented"));
            }
            if case.modal_sub_type == ModalSubType::Ritz && case.loads.is_empty() {
                return Err(invalid(
                    "Ritz analysis requires a starting load: a load pattern or UX, UY, UZ",
                ));
            }
            validate_ritz_controls(model, case)?;
            if !case.history_loads.is_empty()
                || !case.spectrum_loads.is_empty()
                || case.nl_geom != NLGeomType::None
            {
                return Err(invalid(
                    "modal case contains inapplicable dynamic/nonlinear assignments",
                ));
            }
        }
        LoadCaseType::ResponseSpectrum => {
            if case.spectrum_loads.is_empty()
                || !case.loads.is_empty()
                || !case.history_loads.is_empty()
            {
                return Err(invalid("spectrum case requires spectrum assignments only"));
            }
            if !case.modal_damping.is_finite() || !(0.0..=0.99).contains(&case.modal_damping) {
                return Err(invalid("modal damping must be in [0,0.99]"));
            }
            let modal = case
                .initial_case
                .as_ref()
                .and_then(|n| model.load_cases.get(n))
                .ok_or_else(|| invalid("select an existing modal source case"))?;
            if modal.case_type != LoadCaseType::Modal || modal.name == case.name {
                return Err(invalid("spectrum source must be a different modal case"));
            }
            for (f, dir, sf) in &case.spectrum_loads {
                let function = model
                    .functions
                    .get(f)
                    .ok_or_else(|| invalid(format!("missing spectrum function '{f}'")))?;
                validate_function(function)?;
                let valid_dir = dir == dir.trim()
                    && matches!(
                        dir.to_ascii_uppercase().as_str(),
                        "U1" | "UX" | "X" | "1" | "U2" | "UY" | "Y" | "2" | "U3" | "UZ" | "Z" | "3"
                    );
                if function.is_time_history || !valid_dir || !sf.is_finite() {
                    return Err(invalid(
                        "spectrum requires spectrum functions, directions U1/UX/X/1, U2/UY/Y/2, or U3/UZ/Z/3, and finite scales",
                    ));
                }
            }
        }
        LoadCaseType::LinearHistory
        | LoadCaseType::LinearDynamic
        | LoadCaseType::NonlinearHistory
        | LoadCaseType::NonlinearDynamic => {
            if matches!(
                case.case_type,
                LoadCaseType::NonlinearHistory | LoadCaseType::NonlinearDynamic
            ) && (case.max_iterations == 0
                || case.max_iterations > 10000
                || !case.convergence_tol.is_finite()
                || case.convergence_tol <= 0.0
                || case.convergence_tol >= 1.0)
            {
                return Err(invalid(
                    "nonlinear iteration limit must be 1..10000 and tolerance in (0,1)",
                ));
            }
            if matches!(
                case.case_type,
                LoadCaseType::LinearHistory | LoadCaseType::NonlinearHistory
            ) && case.modal_damping != 0.0
                && case.modal_case.is_none()
            {
                return Err(invalid(
                    "modal damping is not implemented for history cases; set it to zero and use Rayleigh damping, or use a direct-integration case type",
                ));
            }
            if case.history_loads.is_empty()
                || !case.loads.is_empty()
                || !case.spectrum_loads.is_empty()
            {
                return Err(invalid("history case requires history assignments only"));
            }
            if case.num_time_steps == 0
                || case.num_time_steps > 1_000_000
                || !case.time_step_dt.is_finite()
                || case.time_step_dt <= 0.0
                || !(case.time_step_dt * case.num_time_steps as f64).is_finite()
            {
                return Err(invalid(
                    "require 1..1000000 time steps and a finite positive step size/duration",
                ));
            }
            if [case.rayleigh_damping.0, case.rayleigh_damping.1]
                .iter()
                .any(|x| !x.is_finite() || *x < 0.0)
            {
                return Err(invalid(
                    "Rayleigh damping coefficients must be finite and nonnegative",
                ));
            }
            match case.time_integration {
                TimeIntegrationMethod::HilberHughesTaylor
                    if !case.hht_alpha.is_finite()
                        || !(-1.0 / 3.0..=0.0).contains(&case.hht_alpha) =>
                {
                    return Err(invalid("HHT alpha must be in [-1/3,0]"))
                }
                TimeIntegrationMethod::WilsonTheta
                    if !case.wilson_theta.is_finite() || case.wilson_theta < 1.37 =>
                {
                    return Err(invalid("Wilson theta must be at least 1.37"))
                }
                _ => {}
            }
            for (f, target, sf) in &case.history_loads {
                let function = model
                    .functions
                    .get(f)
                    .ok_or_else(|| invalid(format!("missing history function '{f}'")))?;
                validate_function(function)?;
                if !function.is_time_history || !sf.is_finite() {
                    return Err(invalid(
                        "history requires time-history functions and finite scales",
                    ));
                }
                if target == target.trim()
                    && matches!(
                        target.to_ascii_uppercase().as_str(),
                        "U1" | "U2" | "U3" | "UX" | "UY" | "UZ" | "X" | "Y" | "Z"
                    )
                {
                    continue;
                }
                let pattern = model
                    .load_patterns
                    .get(target)
                    .ok_or_else(|| invalid(format!("missing history pattern '{target}'")))?;
                if pattern.self_wt_multiplier != 0.0
                    || model.frames.values().any(|f| {
                        f.loads_dist.iter().any(|l| &l.load_pat == target)
                            || f.loads_point.iter().any(|l| &l.load_pat == target)
                            || f.loads_temp.iter().any(|l| &l.load_pat == target)
                            || f.loads_gravity.contains_key(target)
                    })
                    || model.areas.values().any(|a| {
                        a.loads_uniform.iter().any(|l| &l.load_pat == target)
                            || a.loads_pressure.iter().any(|l| &l.load_pat == target)
                            || a.loads_temp.iter().any(|l| &l.load_pat == target)
                    })
                    || model.solids.values().any(|s| {
                        s.loads_surface.iter().any(|l| &l.load_pat == target)
                            || s.loads_temp.contains_key(target)
                    })
                    || model
                        .points
                        .values()
                        .any(|p| p.loads_displ.iter().any(|l| &l.load_pat == target))
                {
                    return Err(invalid("history patterns currently support joint forces only, without self-weight, element loads or support motion"));
                }
            }
        }
        _ => unreachable!(),
    }
    Ok(())
}
/// Remove one entry of the model's unmodeled source data. This is the explicit
/// decision to analyze the affected cases without that imported data; nothing
/// removes an entry implicitly.
pub fn discard_unmodeled_source(
    model: &mut SapModel,
    index: usize,
) -> Result<sap_model::UnmodeledSourceData, DefinitionError> {
    unlocked(model)?;
    if index >= model.unmodeled_source.len() {
        return Err(invalid("no unmodeled source entry at that position"));
    }
    let entry = model.unmodeled_source.remove(index);
    model.invalidate_results();
    Ok(entry)
}
pub fn set_case_definition(model: &mut SapModel, case: LoadCase) -> Result<(), DefinitionError> {
    unlocked(model)?;
    validate_case_definition(model, &case)?;
    model.load_cases.insert(case.name.clone(), case);
    model.invalidate_results();
    Ok(())
}

impl super::cLoadCases<'_> {
    pub fn set_definition(&mut self, case: LoadCase) -> i32 {
        if set_case_definition(&mut self.api.inner, case).is_ok() {
            super::OAPI_OK
        } else {
            super::OAPI_ERR
        }
    }
    pub fn get_definition(&self, name: &str) -> Option<LoadCase> {
        self.api.inner.load_cases.get(name).cloned()
    }
    pub fn set_run_flag(&mut self, name: &str, run: bool) -> i32 {
        if self.api.inner.is_locked {
            return super::OAPI_ERR;
        }
        let Some(case) = self.api.inner.load_cases.get_mut(name) else {
            return super::OAPI_ERR;
        };
        case.run_flag = run;
        self.api.inner.invalidate_results();
        super::OAPI_OK
    }
}
pub struct cFunctions<'a> {
    pub(crate) api: &'a mut super::cSapModel,
}
impl cFunctions<'_> {
    pub fn set_definition(&mut self, function: DynamicFunction) -> i32 {
        if set_function_definition(&mut self.api.inner, function).is_ok() {
            super::OAPI_OK
        } else {
            super::OAPI_ERR
        }
    }
    pub fn get_definition(&self, name: &str) -> Option<DynamicFunction> {
        self.api.inner.functions.get(name).cloned()
    }
}

#[cfg(test)]
mod ritz_definition_tests {
    use super::*;
    use sap_model::ritz::RitzLoadControl;
    use sap_model::{LoadCaseAssignment, LoadPattern, LoadPatternType};

    fn ritz(loads: &[&str]) -> LoadCase {
        let mut case = LoadCase::modal_eigen("R", 6);
        case.modal_sub_type = ModalSubType::Ritz;
        case.loads = loads
            .iter()
            .map(|name| LoadCaseAssignment {
                load_name: (*name).into(),
                scale_factor: 1.0,
            })
            .collect();
        case
    }
    fn control(name: &str, target: f64) -> RitzLoadControl {
        RitzLoadControl {
            load_name: name.into(),
            max_cycles: 0,
            target_dynamic_participation: target,
        }
    }
    fn refused(model: &SapModel, case: &LoadCase, needle: &str) {
        let error = validate_case_definition(model, case)
            .unwrap_err()
            .to_string();
        assert!(error.contains(needle), "'{error}' should contain '{needle}'");
    }

    #[test]
    fn ritz_accepts_patterns_accelerations_and_controls() {
        let model = SapModel::default();
        let mut case = ritz(&["DEAD", "UX", "uz", "U2"]);
        case.ritz_controls = vec![control("UX", 0.9), control("DEAD", 0.0)];
        assert_eq!(validate_case_definition(&model, &case), Ok(()));
    }

    #[test]
    fn ritz_refusals() {
        let mut model = SapModel::default();
        refused(&model, &ritz(&["U4"]), "undefined static load pattern");
        // unknown name that is neither a pattern nor UX/UY/UZ
        refused(&model, &ritz(&["NOPE"]), "undefined static load pattern");
        // no starting load
        refused(&model, &ritz(&[]), "requires a starting load");
        // duplicate
        refused(&model, &ritz(&["DEAD", "DEAD"]), "more than once");
        // control for an unlisted load
        let mut case = ritz(&["DEAD"]);
        case.ritz_controls = vec![control("UX", 0.5)];
        refused(&model, &case, "not a listed starting load");
        // target outside [0, 1], including NaN
        for bad in [-0.1, 1.01, f64::NAN] {
            let mut case = ritz(&["DEAD"]);
            case.ritz_controls = vec![control("DEAD", bad)];
            refused(&model, &case, "must be in [0, 1]");
        }
        // two controls for one load
        let mut case = ritz(&["DEAD"]);
        case.ritz_controls = vec![control("DEAD", 0.5), control("DEAD", 0.6)];
        refused(&model, &case, "two generation controls");
        // Eigen cases take no controls and no starting loads
        let mut eigen = LoadCase::modal_eigen("E", 6);
        eigen.ritz_controls = vec![control("UX", 0.5)];
        refused(&model, &eigen, "Eigen");
        let mut eigen = LoadCase::modal_eigen("E", 6);
        eigen.loads = ritz(&["DEAD"]).loads;
        refused(&model, &eigen, "Eigen");
        // controls on a non-modal case
        let mut other = LoadCase::linear_static("S", "DEAD", 1.0);
        other.ritz_controls = vec![control("DEAD", 0.5)];
        refused(&model, &other, "only modal Ritz cases");
        // acceleration names are not valid starting loads outside Ritz cases
        let mut stat = LoadCase::linear_static("S", "DEAD", 1.0);
        stat.loads.push(LoadCaseAssignment {
            load_name: "UX".into(),
            scale_factor: 1.0,
        });
        refused(&model, &stat, "undefined static load pattern");
        // a pattern literally named UX is ambiguous
        model.load_patterns.insert(
            "UX".into(),
            LoadPattern {
                name: "UX".into(),
                pat_type: LoadPatternType::Other,
                self_wt_multiplier: 0.0,
            },
        );
        refused(&model, &ritz(&["UX"]), "both a load pattern and an acceleration");
    }
}
