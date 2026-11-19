//! Mass from the model mass source.
//!
//! CSI Analysis Reference, "Mass Source" (pp. 334-337). The mass of a model is
//! element self mass, plus additional mass, plus the mass of specified load
//! patterns. The first two are gated in place where the elements are
//! assembled (`assemble_elastic_stiffness_and_mass`), so that the default
//! source keeps its exact floating-point result. This module validates the
//! source and adds the third contribution.
//!
//! For each joint the combined joint-equivalent load of the patterns, scaled
//! by their multipliers, is rotated to global axes. Its component along
//! negative global Z, divided by the gravitational acceleration, is added to
//! all three translational degrees of freedom of the joint: downward loads
//! make positive mass, upward loads negative mass.
//!
//! Differences from the vendor: where the net mass at a free translation comes
//! out negative the vendor sets it to zero and writes a warning to the
//! analysis log. There is no warning channel here, so the analysis is refused.
//! Temperature loads in a mass pattern are refused (the manual says
//! self-equilibrating loads should not be used for mass).

use crate::{assemble_static_loads, SolverError};
use indexmap::IndexMap;
use sap_model::{PointObj, SapModel};

fn is_named_system(csys: &str) -> bool {
    !(csys.eq_ignore_ascii_case("GLOBAL") || csys.eq_ignore_ascii_case("LOCAL"))
}

/// Refuses a mass source that cannot be evaluated exactly.
pub(crate) fn validate(model: &SapModel) -> Result<(), SolverError> {
    for (pattern, factor) in &model.mass_source.load_patterns {
        if !factor.is_finite() {
            return Err(SolverError::InvalidInput(format!(
                "mass source multiplier of pattern '{pattern}' is not finite"
            )));
        }
        if !model.load_patterns.contains_key(pattern) {
            return Err(SolverError::InvalidInput(format!(
                "mass source names unknown load pattern '{pattern}'"
            )));
        }
        if model.auto_seismic.contains_key(pattern) {
            return Err(SolverError::UnsupportedInput(format!(
                "mass source uses pattern '{pattern}', which is an automatic lateral load; its force depends on the mass"
            )));
        }
        // Loads of patterns that no case uses are not normalized to global
        // axes before the analysis, so named systems cannot be honored here.
        let unsupported = |what: &str, owner: &str| {
            SolverError::UnsupportedInput(format!(
                "mass source pattern '{pattern}': {what} on '{owner}' is not supported in a mass pattern"
            ))
        };
        for (name, p) in &model.points {
            for load in p.loads_force.iter().filter(|l| &l.load_pat == pattern) {
                if load.force.iter().any(|x| !x.is_finite()) {
                    return Err(SolverError::InvalidInput(format!(
                        "joint '{name}' has nonfinite force in mass pattern '{pattern}'"
                    )));
                }
                if is_named_system(&load.csys) {
                    return Err(unsupported("a joint load in a named coordinate system", name));
                }
            }
        }
        for (name, f) in &model.frames {
            if f.loads_temp.iter().any(|l| &l.load_pat == pattern) {
                return Err(unsupported("a temperature load", name));
            }
            if f.loads_dist
                .iter()
                .any(|l| &l.load_pat == pattern && is_named_system(&l.csys))
                || f.loads_point
                    .iter()
                    .any(|l| &l.load_pat == pattern && is_named_system(&l.csys))
            {
                return Err(unsupported("a load in a named coordinate system", name));
            }
        }
        for (name, a) in &model.areas {
            if a.loads_temp.iter().any(|l| &l.load_pat == pattern) {
                return Err(unsupported("a temperature load", name));
            }
            if a.loads_uniform.iter().any(|l| {
                &l.load_pat == pattern
                    && l.val != 0.0
                    && (is_named_system(&l.csys) || (7..=9).contains(&l.dir) || l.dir == 11)
            }) {
                return Err(unsupported(
                    "a uniform load in a named system or projected direction",
                    name,
                ));
            }
            if a.loads_pressure.iter().any(|l| {
                &l.load_pat == pattern
                    && l.val != 0.0
                    && (!matches!(l.face, -1 | -2)
                        || l.pattern_name.as_ref().is_some_and(|p| !p.is_empty()))
            }) {
                return Err(unsupported("an edge or patterned pressure", name));
            }
        }
    }
    Ok(())
}

/// Adds the mass of the specified load patterns to the assembled joint-local
/// mass vector. Does nothing for a source without patterns.
#[allow(clippy::too_many_arguments)]
pub(crate) fn add_pattern_mass(
    model: &SapModel,
    work_points: &IndexMap<String, PointObj>,
    work_areas: &IndexMap<String, sap_model::AreaObj>,
    pt_index: &IndexMap<String, usize>,
    joint_rot: &[[[f64; 3]; 3]],
    ndof: usize,
    diag_mass: &mut [f64],
) -> Result<(), SolverError> {
    let patterns = &model.mass_source.load_patterns;
    if patterns.is_empty() {
        return Ok(());
    }
    let g = model.present_units.gravity_acceleration();
    let loads = assemble_static_loads(
        model,
        patterns,
        work_points,
        work_areas,
        pt_index,
        joint_rot,
        &IndexMap::new(),
        ndof,
    )?;
    for (idx, rot) in joint_rot.iter().enumerate() {
        // Global vertical component of the joint load: the load in local
        // components is rotated by the local-to-global matrix.
        let fz: f64 = (0..3)
            .map(|d| rot[2][d] * (loads.f_ext_joint[6 * idx + d] - loads.f_fef_joint[6 * idx + d]))
            .sum();
        let mass = -fz / g;
        for d in 0..3 {
            diag_mass[6 * idx + d] += mass;
        }
    }
    // Net negative mass at a free translation is refused. Rounding noise
    // from cancelling loads, relative to the largest mass, is set to zero.
    let scale = diag_mass.iter().fold(0.0f64, |a, m| a.max(m.abs()));
    let tol = 1e-12 * scale;
    for (idx, pt) in work_points.values().enumerate() {
        for d in 0..3 {
            if pt.restraint[d] || !model.active_dofs[d] {
                continue;
            }
            let m = &mut diag_mass[6 * idx + d];
            if *m < -tol {
                return Err(SolverError::UnsupportedInput(format!(
                    "mass source gives joint '{}' a net negative translational mass {:e} in local direction {}; the vendor would set it to zero with a warning",
                    pt.name,
                    *m,
                    d + 1
                )));
            }
            if *m < 0.0 {
                *m = 0.0;
            }
        }
    }
    Ok(())
}
