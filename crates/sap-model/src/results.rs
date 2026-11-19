//! Result-state queries shared by the solver and the GUI.

use crate::{CaseStatus, ComboType, LoadCase, LoadCaseType, SapModel};

/// Whether `name` (case or combination) is single valued (Analysis Reference
/// pp. 213, 352). Moved unchanged from `sap-app/src/gui/shell_output.rs`.
///
/// True when `name` has one value per quantity: a case other than a
/// response-spectrum, moving-load, steady-state or power-spectral-density
/// case, or a linear combination of such results (CSI Analysis Reference
/// Manual, pp. 214 and 352). Envelope, SRSS, absolute-sum and range-sum
/// combinations are multi-valued (the solver adds a range-sum linearly, but
/// the vendor reports its range), and so is a name that is neither a case nor
/// a combination.
pub fn is_single_valued(model: &SapModel, name: &str) -> bool {
    single_valued_inner(model, name, 0)
}

fn single_valued_inner(model: &SapModel, name: &str, depth: usize) -> bool {
    if depth > model.load_combos.len() {
        return false; // a cycle
    }
    if let Some(combo) = model.load_combos.get(name) {
        return combo.combo_type == ComboType::LinearAdd
            && combo
                .cases
                .iter()
                .all(|(item, _)| single_valued_inner(model, item, depth + 1));
    }
    match model.load_cases.get(name) {
        // Response-spectrum, moving-load, steady-state and power-spectral-density
        // results are ranges, not single states.
        Some(case) => !matches!(
            case.case_type,
            LoadCaseType::ResponseSpectrum
                | LoadCaseType::MovingLoad
                | LoadCaseType::SteadyState
                | LoadCaseType::PowerSpectralDensity
        ),
        None => false,
    }
}

/// The names that are both a load case and a load combination of `model` (D79), in
/// the order of the combinations. A case and a combination live in one namespace for
/// results (`case_results` is keyed by name) and for the contributors of a combination,
/// so such a name cannot be told apart: the result of the case would be read as that of
/// the combination, or the reverse. Every entry point that can create or load a model
/// refuses these names (S2K and native readers, the database tables, the API) and the
/// solver refuses to analyze one before any case is evaluated.
pub fn case_combo_name_collisions(model: &SapModel) -> Vec<String> {
    model
        .load_combos
        .keys()
        .filter(|n| model.load_cases.contains_key(*n))
        .cloned()
        .collect()
}

/// The diagnostic for [`case_combo_name_collisions`], naming each name and both objects;
/// `None` when no name is shared.
pub fn case_combo_collision_message(model: &SapModel) -> Option<String> {
    let names = case_combo_name_collisions(model);
    if names.is_empty() {
        return None;
    }
    let list = names
        .iter()
        .map(|n| {
            let case_type = model
                .load_cases
                .get(n)
                .map_or_else(String::new, |c| format!("{:?}", c.case_type));
            let n_contrib = model.load_combos.get(n).map_or(0, |c| c.cases.len());
            format!(
                "'{n}' is both the load case '{n}' ({case_type}) and the load combination \
                 '{n}' ({n_contrib} contributors)"
            )
        })
        .collect::<Vec<_>>()
        .join("; ");
    Some(format!(
        "{list}: a load case and a load combination must not share a name, because their \
         results and the contributors of a combination are identified by name alone"
    ))
}

/// The reason why the case or combination `name` has no results although it was
/// requested (D78), as recorded by the last analysis. `None` for a name with
/// results, a name never requested and any name while the model holds no results.
pub fn unavailable_reason<'a>(model: &'a SapModel, name: &str) -> Option<&'a str> {
    if !model.is_locked || model.case_results.contains_key(name) {
        return None;
    }
    model.unavailable_results.get(name).map(String::as_str)
}

/// `"'NAME' is unavailable: reason"` for [`unavailable_reason`].
pub fn unavailable_message(model: &SapModel, name: &str) -> Option<String> {
    unavailable_reason(model, name).map(|r| format!("'{name}' is unavailable: {r}"))
}

/// Why the combination `combo` cannot be evaluated from the results that exist, as
/// the clause that follows `"combination 'NAME' is unavailable: "`; `None` when every
/// contributor has results (or is a combination that can itself be evaluated). A
/// contributor is required whatever its scale factor, a missing contributor is never
/// treated as zero, and the contributors that exist are never combined without it.
pub fn combo_gap(model: &SapModel, combo: &crate::LoadCombo) -> Option<String> {
    combo_gap_in(model, combo, &mut vec![combo.name.clone()])
}

fn combo_gap_in(
    model: &SapModel,
    combo: &crate::LoadCombo,
    path: &mut Vec<String>,
) -> Option<String> {
    if combo.cases.is_empty() {
        return Some("it has no contributing case or combination".to_string());
    }
    for (src, _) in &combo.cases {
        // A Modal or Buckling case is combined one mode at a time (Analysis Reference
        // p. 351), and a combination here names no mode: its stored shape of the first
        // mode (no forces) would enter the sum as if it were a response.
        if let Some(case) = model.load_cases.get(src) {
            if matches!(case.case_type, LoadCaseType::Modal | LoadCaseType::Buckling) {
                return Some(format!(
                    "its contributor case '{src}' is a {:?} case, whose results can be \
                     combined only one mode at a time and a combination here cannot name a mode",
                    case.case_type
                ));
            }
        }
        // D80: a time-history case keeps its forces, reactions and area results at one
        // step only (the series exists for joint displacements alone), so the maximum and
        // minimum over time of what a combination publishes are not known. The snapshot
        // would be published as both bounds of an envelope and as a state of a sum.
        if is_time_history_contributor(model, src) {
            return Some(format!(
                "its contributor case '{src}' is a time-history case: combination '{}' would \
                 need the maximum and minimum over time of every quantity, and only a \
                 snapshot at the step of largest displacement is stored for forces, \
                 reactions and area results, so the extrema over time are not stored",
                path.last().map_or("", String::as_str)
            ));
        }
        if let Some(results) = model.case_results.get(src) {
            // G16: the states of a case that is not finished are not its response to the
            // load, and a combination of them would claim a response that was not computed.
            if let CaseStatus::NotFinished { reason, .. } = &results.status {
                return Some(format!(
                    "its contributor case '{src}' is not finished ({reason})"
                ));
            }
            continue;
        }
        if let Some(case) = model.load_cases.get(src) {
            return Some(if !case.run_flag {
                format!("its contributor case '{src}' was not run (RunCase=No)")
            } else if let Some(r) = model.unavailable_results.get(src) {
                format!("its contributor case '{src}' is unavailable ({r})")
            } else {
                format!("its contributor case '{src}' has no results")
            });
        }
        if let Some(sub) = model.load_combos.get(src) {
            if path.contains(src) {
                return Some(format!(
                    "it depends on itself through the combinations {} -> '{src}'",
                    path.iter()
                        .map(|p| format!("'{p}'"))
                        .collect::<Vec<_>>()
                        .join(" -> ")
                ));
            }
            path.push(src.clone());
            let inner = combo_gap_in(model, sub, path);
            path.pop();
            if let Some(inner) = inner {
                return Some(format!(
                    "its contributor combination '{src}' is unavailable ({inner})"
                ));
            }
            continue;
        }
        return Some(format!(
            "its contributor '{src}' is neither a load case nor a combination of the model"
        ));
    }
    None
}

/// Whether the case `name` is a time-history case (D80): of a history type, or holding a
/// stored time series.
pub fn is_time_history_contributor(model: &SapModel, name: &str) -> bool {
    model.load_cases.get(name).is_some_and(|c| {
        matches!(
            c.case_type,
            LoadCaseType::LinearHistory
                | LoadCaseType::NonlinearHistory
                | LoadCaseType::LinearDynamic
                | LoadCaseType::NonlinearDynamic
        )
    }) || model
        .case_results
        .get(name)
        .is_some_and(|r| !r.time_steps.is_empty())
}

/// One accumulation step of the combination rules of Analysis Reference pp. 351-352,
/// shared by the solver (`combos.rs`) and by design: `acc` after one more contributor
/// with the value pair `(max, min)` of one quantity, scaled by `sf`, for the maximum
/// (`maximum`) or the minimum sense. `range` marks a contributor whose pair is
/// `(+|max|, -|max|)` (response-spectrum, moving-load, steady-state and PSD cases:
/// "the maximum value used is the positive computed value, and the minimum value is
/// just the negative of the maximum", p. 351). A negative `sf` swaps the pair.
///
/// - additive: sum of the maxima and sum of the minima;
/// - range: sum of the positive maxima and sum of the negative minima;
/// - absolute: sum of the larger absolute values, the minimum its negative;
/// - SRSS: accumulates the squares of the larger absolute values (take the square
///   root at the end, negated for the minimum);
/// - envelope: maximum of the maxima, minimum of the minima (`is_first` starts it).
pub fn combo_step(
    combo_type: ComboType,
    maximum: bool,
    acc: f64,
    pair: (f64, f64),
    range: bool,
    sf: f64,
    is_first: bool,
) -> f64 {
    let (hi, lo) = pair;
    let (a, b) = if range { (hi.abs(), -hi.abs()) } else { (hi, lo) };
    let (p, q) = (a * sf, b * sf);
    let (pmax, pmin) = (p.max(q), p.min(q));
    match (combo_type, maximum) {
        (ComboType::LinearAdd, true) => acc + pmax,
        (ComboType::LinearAdd, false) => acc + pmin,
        (ComboType::RangeAdd, true) => acc + pmax.max(0.0),
        (ComboType::RangeAdd, false) => acc + pmin.min(0.0),
        (ComboType::AbsAdd, true) => acc + pmax.abs().max(pmin.abs()),
        (ComboType::AbsAdd, false) => acc - pmax.abs().max(pmin.abs()),
        (ComboType::Srss, _) => acc + pmax.abs().max(pmin.abs()).powi(2),
        (ComboType::Envelope, true) => {
            if is_first {
                pmax
            } else {
                acc.max(pmax)
            }
        }
        (ComboType::Envelope, false) => {
            if is_first {
                pmin
            } else {
                acc.min(pmin)
            }
        }
    }
}

/// Whether the results of load case `name` are a pair `(+v, -v)` (response-spectrum,
/// moving-load, steady-state and power-spectral-density cases).
pub fn is_range_case(model: &SapModel, name: &str) -> bool {
    model.load_cases.get(name).is_some_and(|c| {
        matches!(
            c.case_type,
            LoadCaseType::ResponseSpectrum
                | LoadCaseType::MovingLoad
                | LoadCaseType::SteadyState
                | LoadCaseType::PowerSpectralDensity
        )
    })
}

/// The cases whose results stop short of their target (v19 "Not Finished"), in the order of
/// the results, with the reason and the number of saved states each kept. Empty while the
/// model holds no results.
pub fn unfinished_cases(model: &SapModel) -> Vec<(String, CaseStatus)> {
    if !model.is_locked {
        return Vec::new();
    }
    model
        .case_results
        .iter()
        .filter(|(name, r)| model.load_cases.contains_key(*name) && !r.status.is_finished())
        .map(|(name, r)| (name.clone(), r.status.clone()))
        .collect()
}

/// The cases that could not start because a prerequisite case did not finish (v19 "Could
/// Not Start"), with the reason, which begins with `could not start:`. They have no results.
pub fn could_not_start_cases(model: &SapModel) -> Vec<(String, String)> {
    if !model.is_locked {
        return Vec::new();
    }
    model
        .unavailable_results
        .iter()
        .filter(|(name, reason)| {
            model.load_cases.contains_key(*name) && reason.starts_with(COULD_NOT_START)
        })
        .map(|(n, r)| (n.clone(), r.clone()))
        .collect()
}

/// The start of the reason of a case that could not start.
pub const COULD_NOT_START: &str = "could not start:";

/// `Some("'NAME' is not finished: reason")` for a case that has results that stop short of
/// its target. Every consumer that would use such results as a complete response refuses
/// the case with this text.
pub fn unfinished_message(model: &SapModel, name: &str) -> Option<String> {
    if !model.is_locked {
        return None;
    }
    match &model.case_results.get(name)?.status {
        CaseStatus::Finished => None,
        CaseStatus::NotFinished { reason, steps } => Some(format!(
            "case '{name}' is not finished ({reason}; {steps} saved steps kept)"
        )),
    }
}

/// Why the case `lc` cannot start: a prerequisite it continues from or takes its stiffness
/// or modes from (its initial case, its modal case) is not finished or could not start.
/// The text begins with [`COULD_NOT_START`] and names the prerequisite. `None` when every
/// prerequisite has complete results.
pub fn prerequisite_gap(model: &SapModel, lc: &LoadCase) -> Option<String> {
    for (role, dep) in [("initial case", &lc.initial_case), ("modal case", &lc.modal_case)] {
        let Some(dep) = dep else { continue };
        if let Some(results) = model.case_results.get(dep) {
            if let CaseStatus::NotFinished { reason, .. } = &results.status {
                return Some(format!(
                    "{COULD_NOT_START} its {role} '{dep}' is not finished ({reason})"
                ));
            }
        } else if let Some(reason) = model.unavailable_results.get(dep) {
            return Some(format!(
                "{COULD_NOT_START} its {role} '{dep}' is unavailable ({reason})"
            ));
        }
    }
    None
}

/// The v19 OAPI `GetCaseStatus` code of load case `name`: 1 not run, 2 could not start,
/// 3 not finished, 4 finished.
pub fn case_status_code(model: &SapModel, name: &str) -> i32 {
    if !model.is_locked {
        return 1;
    }
    match model.case_results.get(name) {
        Some(r) if model.load_cases.contains_key(name) => {
            if r.status.is_finished() {
                4
            } else {
                3
            }
        }
        _ => match model.unavailable_results.get(name) {
            Some(reason) if reason.starts_with(COULD_NOT_START) => 2,
            _ => 1,
        },
    }
}
