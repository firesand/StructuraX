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
    if value.trim().is_empty() || value.chars().any(char::is_control) {
        return Err(invalid(
            "name must be nonempty and contain no control characters",
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
/// Validate definitions represented by this native editor; solver applicability checks still apply.
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
    if case.case_type != LoadCaseType::ResponseSpectrum && case.initial_case.is_some() {
        return Err(invalid(
            "initial-state cases are not supported by this editor",
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
    for load in &case.loads {
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
    match case.case_type {
        LoadCaseType::LinearStatic | LoadCaseType::NonlinearStatic | LoadCaseType::Buckling => {
            if case.loads.is_empty() {
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
                return Err(invalid("Ritz analysis requires a starting load pattern"));
            }
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
                if function.is_time_history
                    || !matches!(dir.as_str(), "U1" | "U2" | "U3")
                    || !sf.is_finite()
                {
                    return Err(invalid("spectrum requires spectrum functions, directions U1/U2/U3 and finite scales"));
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
                if matches!(
                    target.trim().to_ascii_uppercase().as_str(),
                    "U1" | "U2" | "U3" | "UX" | "UY" | "UZ" | "X" | "Y" | "Z"
                ) {
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
