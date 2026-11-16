//! Loads of nonlinear static cases beyond the load patterns: acceleration
//! (Accel) and modal (Mode) loads. Source: CSI Analysis Reference Manual,
//! "Nonlinear Static Analysis", load types of a static case.

use indexmap::IndexMap;
use sap_model::{LoadCase, SapModel};

use crate::SolverError;

fn is_acceleration(name: &str) -> bool {
    sap_model::ritz::acceleration_direction(name).is_some()
}

/// Whether a case applies an Accel or a Mode load.
pub(crate) fn uses_special_loads(lc: &LoadCase) -> bool {
    lc.loads.iter().any(|l| is_acceleration(&l.load_name)) || !lc.nl_static.mode_loads.is_empty()
}

/// The load patterns of a case with their scale factors, without
/// acceleration loads.
pub(crate) fn pattern_loads(lc: &LoadCase) -> Vec<(String, f64)> {
    lc.loads
        .iter()
        .filter(|l| !is_acceleration(&l.load_name))
        .map(|l| (l.load_name.clone(), l.scale_factor))
        .collect()
}

/// The joint-local force vector of the Accel and Mode loads of a case at unit
/// load factor, or `None` when it has none.
///
/// * Accel (direction `r`, a unit translation in a global direction, scale
///   factor `sf`): `f = -sf * M * r`, with `M` the lumped joint masses of the
///   analysis (element, additional and mass-source mass), written in joint
///   local axes. The acceleration has the sign of `sf` and the force acts
///   against it, as in a ground-acceleration history.
/// * Mode (mode `n` of the Modal case `lc.modal_case`, scale factor `sf`):
///   `f = sf * omega_n^2 * M * phi_n` with `phi_n` the mode shape normalized
///   to `phi^T M phi = 1` (CSI Analysis Reference, modal analysis), in joint
///   local axes. For a linear structure the displacement it produces is
///   `sf * phi_n`. The sign is that of the stored mode shape.
///
/// A rotational acceleration, a load pattern named like an acceleration
/// load, a Mode load mixed with other loads, and a Mode load whose modal case
/// is missing, not Modal, not run or without that mode are refused.
pub(crate) fn special_loads(
    model: &SapModel,
    lc: &LoadCase,
    pt_index: &IndexMap<String, usize>,
    joint_rot: &[[[f64; 3]; 3]],
    diag_mass: &[f64],
    ndof: usize,
) -> Result<Option<Vec<f64>>, SolverError> {
    let case = &lc.name;
    let mut total: Option<Vec<f64>> = None;
    for load in &lc.loads {
        let Some(dir) = sap_model::ritz::acceleration_direction(&load.load_name) else {
            continue;
        };
        if model.load_patterns.contains_key(&load.load_name) {
            return Err(SolverError::UnsupportedInput(format!(
                "case '{case}': load '{}' is both a load pattern and an acceleration load name; rename the load pattern",
                load.load_name
            )));
        }
        if !load.scale_factor.is_finite() {
            return Err(SolverError::InvalidInput(format!(
                "case '{case}': acceleration load '{}' has a nonfinite scale factor",
                load.load_name
            )));
        }
        let f = crate::ritz_modal::acceleration_load(diag_mass, joint_rot, dir, load.scale_factor);
        add_into(total.get_or_insert_with(|| vec![0.0; ndof]), &f);
    }
    if lc.nl_static.mode_loads.is_empty() {
        return Ok(total);
    }
    if !lc.loads.is_empty() {
        return Err(SolverError::UnsupportedInput(format!(
            "case '{case}' mixes Mode loads with other loads; the scale of a modal load depends on the mode normalization, so a Mode load must be the only load of its case"
        )));
    }
    let modal_name = lc.modal_case.as_ref().ok_or_else(|| {
        SolverError::InvalidInput(format!(
            "case '{case}' has Mode loads but names no modal case"
        ))
    })?;
    let modal = model.case_results.get(modal_name).ok_or_else(|| {
        SolverError::InvalidInput(format!(
            "case '{case}' needs the modes of '{modal_name}', which has not been analysed"
        ))
    })?;
    for load in &lc.nl_static.mode_loads {
        if !load.scale_factor.is_finite() {
            return Err(SolverError::InvalidInput(format!(
                "case '{case}': Mode load {} has a nonfinite scale factor",
                load.mode
            )));
        }
        let mode = load
            .mode
            .checked_sub(1)
            .and_then(|i| modal.modes.get(i))
            .ok_or_else(|| {
                SolverError::InvalidInput(format!(
                    "case '{case}': Mode load {} is outside the {} modes of '{modal_name}'",
                    load.mode,
                    modal.modes.len()
                ))
            })?;
        let w = mode.circ_freq_rad_s;
        if !w.is_finite() || w <= 0.0 {
            return Err(SolverError::UnsupportedInput(format!(
                "case '{case}': mode {} of '{modal_name}' has circular frequency {w}; a Mode load needs a positive frequency",
                load.mode
            )));
        }
        let mut phi = vec![0.0; ndof];
        for (point, disp) in &mode.mode_shape {
            if let Some(&idx) = pt_index.get(point) {
                phi[6 * idx..6 * idx + 6].copy_from_slice(disp);
            }
        }
        let mu: f64 = phi.iter().zip(diag_mass).map(|(f, m)| m * f * f).sum();
        // The modes are stored with phi^T M phi = 1 over every joint of the
        // analysis, including the joints a subdivision creates. A shape that
        // is not, is not the shape of this analysis model: refuse it instead
        // of rescaling a load vector that would miss part of the mass.
        if !mu.is_finite() || (mu - 1.0).abs() > 1e-6 {
            return Err(SolverError::InvalidInput(format!(
                "case '{case}': mode {} of '{modal_name}' is not normalized to the mass of the analysis model (phi' M phi = {mu})",
                load.mode
            )));
        }
        let scale = load.scale_factor * w * w;
        let f: Vec<f64> = phi
            .iter()
            .zip(diag_mass)
            .map(|(f, m)| scale * m * f)
            .collect();
        add_into(total.get_or_insert_with(|| vec![0.0; ndof]), &f);
    }
    Ok(total)
}

fn add_into(acc: &mut [f64], f: &[f64]) {
    for (a, b) in acc.iter_mut().zip(f) {
        *a += b;
    }
}
