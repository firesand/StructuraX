//! Result-state queries shared by the solver and the GUI.

use crate::{ComboType, LoadCaseType, SapModel};

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
