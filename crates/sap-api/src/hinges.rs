//! Frame hinge assignments and pushover results in the shape of the vendor
//! OAPI (`cFrameObj.GetHingeAssigns`, static nonlinear case hinge unloading),
//! and the static pushover curve.
//!
//! Sources: OAPI help "GetHingeAssigns" and "SetHingeUnloading (Static
//! Nonlinear)"; CSI Analysis Reference, "Static Pushover Analysis" and
//! "Frame Hinge Properties". The columns of [`PushoverPoint`] are the ones of
//! the vendor pushover table (base force, displacement, hinge counts by state
//! and by status, total hinges).
//!
//! Base shear. The reactions of a saved state are the sum of all support
//! reactions in global axes (`NonlinearStaticStep::base_reactions`). The base
//! shear of a point is minus the component of that force along the global
//! direction of the monitored joint DOF (the column of the joint's local-to-
//! global rotation that belongs to the monitored DOF):
//! `V = -(R . e)`. A reaction opposes the load, so a push in the positive
//! direction of the monitored DOF gives a positive base shear, and a push in
//! the negative direction gives a negative one together with a negative
//! displacement. It includes the reaction to everything the case carries
//! (the loads of earlier cases of its chain and its own), as the vendor curve
//! does for a case that continues a gravity case.

use sap_model::hinge::{HingeBehavior, HingeState, HingeStatus, HingeUnloading};
use sap_model::{LoadCaseType, SapModel};
use sap_solver::frame::joint_local_to_global_matrix;

/// One saved state of a pushover curve.
#[derive(Debug, Clone, PartialEq)]
pub struct PushoverPoint {
    /// Saved state number, starting at 0 (the state at the start of the case).
    pub step: usize,
    /// Total value of the monitored joint displacement (model length unit).
    pub displacement: f64,
    /// Base shear along the monitored direction (model force unit); see the
    /// module documentation for the sign.
    pub base_shear: f64,
    /// Hinge counts by state: A-B, B-C, C-D, D-E, beyond E.
    pub a_to_b: usize,
    pub b_to_c: usize,
    pub c_to_d: usize,
    pub d_to_e: usize,
    pub beyond_e: usize,
    /// Hinge counts by performance status: A-IO (the states A-B and B-IO),
    /// IO-LS, LS-CP, beyond CP.
    pub a_to_io: usize,
    pub io_to_ls: usize,
    pub ls_to_cp: usize,
    pub beyond_cp: usize,
    /// All hinges of the model.
    pub total: usize,
}

/// The status of the results behind [`pushover_curve`] (G16): `Some((reason, saved steps))`
/// when the case stopped before its target and the curve is the part it reached; `None`
/// for a case that finished or has no results.
pub fn pushover_not_finished(model: &SapModel, case: &str) -> Option<(String, usize)> {
    match &model.case_results.get(case)?.status {
        sap_model::CaseStatus::NotFinished { reason, steps } => Some((reason.clone(), *steps)),
        sap_model::CaseStatus::Finished => None,
    }
}

/// The pushover curve of a nonlinear static case: one point per saved state. A case that
/// is not finished gives the states it reached (see [`pushover_not_finished`]).
///
/// Refused: an unknown case, a case that is not nonlinear static, a case
/// without displacement control, a monitored DOF that is a rotation (a base
/// shear against a rotation is not defined), a case without results, and a
/// case with fewer than two saved states.
pub fn pushover_curve(model: &SapModel, case: &str) -> Result<Vec<PushoverPoint>, String> {
    let lc = model
        .load_cases
        .get(case)
        .ok_or_else(|| format!("case '{case}' does not exist"))?;
    // G16: a case that could not start has no results, and says why.
    if let Some(reason) = sap_model::results::unavailable_reason(model, case) {
        return Err(format!("case '{case}' has no results: {reason}"));
    }
    if lc.case_type != LoadCaseType::NonlinearStatic {
        return Err(format!("case '{case}' is not a nonlinear static case"));
    }
    let control = lc.displacement_control.as_ref().ok_or_else(|| {
        format!("case '{case}' does not use displacement control; a pushover curve needs a monitored displacement")
    })?;
    let (joint, dof) = control
        .monitored
        .as_ref()
        .ok_or_else(|| format!("case '{case}' monitors a displacement that is not a joint DOF"))?;
    if *dof >= 3 {
        return Err(format!(
            "case '{case}' monitors a rotation of joint '{joint}'; the pushover curve needs a translation"
        ));
    }
    let point = model
        .points
        .get(joint)
        .ok_or_else(|| format!("monitored joint '{joint}' of case '{case}' does not exist"))?;
    let results = model
        .case_results
        .get(case)
        .filter(|r| !r.static_steps.is_empty())
        .ok_or_else(|| format!("case '{case}' has no saved states; run the analysis first"))?;
    if results.static_steps.len() < 2 {
        return Err(format!(
            "case '{case}' saved a single state; select Multiple States in its Results Saved options"
        ));
    }
    let rot = joint_local_to_global_matrix(point);
    let direction = [rot[0][*dof], rot[1][*dof], rot[2][*dof]];
    Ok(results
        .static_steps
        .iter()
        .map(|step| {
            let mut p = PushoverPoint {
                step: step.step,
                displacement: step.monitored_displacement.unwrap_or(f64::NAN),
                base_shear: -(0..3)
                    .map(|i| step.base_reactions[i] * direction[i])
                    .sum::<f64>(),
                a_to_b: 0,
                b_to_c: 0,
                c_to_d: 0,
                d_to_e: 0,
                beyond_e: 0,
                a_to_io: 0,
                io_to_ls: 0,
                ls_to_cp: 0,
                beyond_cp: 0,
                total: step.hinges.len(),
            };
            for h in &step.hinges {
                match h.state {
                    HingeState::AToB => p.a_to_b += 1,
                    HingeState::BToC => p.b_to_c += 1,
                    HingeState::CToD => p.c_to_d += 1,
                    HingeState::DToE => p.d_to_e += 1,
                    HingeState::BeyondE => p.beyond_e += 1,
                }
                match h.status {
                    HingeStatus::AToB | HingeStatus::BToIO => p.a_to_io += 1,
                    HingeStatus::IOToLS => p.io_to_ls += 1,
                    HingeStatus::LSToCP => p.ls_to_cp += 1,
                    HingeStatus::BeyondCP => p.beyond_cp += 1,
                }
            }
            p
        })
        .collect())
}

/// The result of `cFrameObj.GetHingeAssigns`: parallel arrays, one entry per
/// hinge of the frame in assignment order.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct HingeAssigns {
    pub number_hinges: usize,
    /// Hinge number on the frame, starting at 1.
    pub hinge_num: Vec<i32>,
    /// Generated property name, `{frame}H{number}`.
    pub prop: Vec<String>,
    /// 1 P, 2 V2, 3 V3, 4 T, 5 M2, 6 M3 (7 to 11 are coupled and fiber hinges,
    /// which are not modeled).
    pub my_type: Vec<i32>,
    /// 1 force controlled, 2 deformation controlled.
    pub behavior: Vec<i32>,
    /// Name of the defined hinge property the hinge was assigned (the vendor
    /// reports "Auto" for auto hinges, which are not modeled).
    pub source: Vec<String>,
    /// Relative distance from End I.
    pub rd: Vec<f64>,
}

/// Hinge assignments of a frame object. Refused: an unknown frame, and a
/// hinge whose property is not defined.
pub fn get_hinge_assigns(model: &SapModel, frame: &str) -> Result<HingeAssigns, String> {
    let obj = model
        .frames
        .get(frame)
        .ok_or_else(|| format!("frame '{frame}' does not exist"))?;
    let mut out = HingeAssigns {
        number_hinges: obj.hinges.len(),
        ..Default::default()
    };
    for (i, h) in obj.hinges.iter().enumerate() {
        let prop = model.hinge_props.get(&h.prop).ok_or_else(|| {
            format!(
                "hinge {} of frame '{frame}' uses the undefined hinge property '{}'",
                i + 1,
                h.prop
            )
        })?;
        out.hinge_num.push(i as i32 + 1);
        out.prop
            .push(sap_model::hinge::generated_name(frame, i + 1));
        out.my_type.push(prop.dof.oapi_code());
        out.behavior.push(match prop.behavior {
            HingeBehavior::ForceControlled => 1,
            HingeBehavior::DeformationControlled => 2,
        });
        out.source.push(h.prop.clone());
        out.rd.push(h.rel_dist);
    }
    Ok(out)
}

/// `cCaseStaticNonlinear.GetHingeUnloading`: 1 unload entire structure,
/// 2 apply local redistribution, 3 restart using secant stiffness.
pub fn get_hinge_unloading(model: &SapModel, case: &str) -> Result<i32, String> {
    let lc = nonlinear_static_case(model, case)?;
    Ok(match lc.nl_static.hinge_unloading {
        HingeUnloading::UnloadEntireStructure => 1,
        HingeUnloading::ApplyLocalRedistribution => 2,
        HingeUnloading::RestartUsingSecantStiffness => 3,
    })
}

/// `cCaseStaticNonlinear.SetHingeUnloading`. Only option 1 is implemented;
/// 2 and 3 are refused, other codes are invalid. A locked model is refused.
pub fn set_hinge_unloading(model: &mut SapModel, case: &str, option: i32) -> Result<(), String> {
    nonlinear_static_case(model, case)?;
    match option {
        1 => {}
        2 | 3 => {
            return Err(format!(
                "hinge unloading option {option} is not implemented; only 1 (unload entire structure) is"
            ))
        }
        _ => return Err(format!("hinge unloading option {option} is not 1, 2 or 3")),
    }
    if model.is_locked {
        return Err("the model is locked; unlock it to change the case".into());
    }
    if let Some(lc) = model.load_cases.get_mut(case) {
        lc.nl_static.hinge_unloading = HingeUnloading::UnloadEntireStructure;
    }
    Ok(())
}

fn nonlinear_static_case<'a>(
    model: &'a SapModel,
    case: &str,
) -> Result<&'a sap_model::LoadCase, String> {
    let lc = model
        .load_cases
        .get(case)
        .ok_or_else(|| format!("case '{case}' does not exist"))?;
    if lc.case_type != LoadCaseType::NonlinearStatic {
        return Err(format!("case '{case}' is not a nonlinear static case"));
    }
    Ok(lc)
}
