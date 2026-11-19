//! Steel design with Auto Select section lists and the design verification
//! commands.
//!
//! Source: CSI Steel Frame Design Manual AISC 360-10, section 2.1: the sections
//! of the list are ordered by increasing weight per unit length and checked
//! one after the other exactly like an individual section; the first (lightest)
//! one that passes is chosen; when none passes the heaviest is used and
//! flagged as overstressed. A section passes when its demand/capacity ratio is
//! at most the D/C ratio limit (default 0.95, table B-1) and it satisfies the
//! other requirements. Appendix E: the analysis section and the design section
//! are tracked separately and must be verified to agree before design is
//! complete.
//!
//! The list itself, its median and the analysis-section bookkeeping live in
//! `sap_model::auto_select`.

use crate::steel::{design_frame_with_section, SteelCode, SteelMemberOverwrite};
use crate::DesignError;
use sap_model::auto_select::{self as list, is_auto_list};
use sap_model::{FrameObj, FrameSection, SapModel, SteelDesignResult};
use std::collections::HashMap;

/// A frame section that is an Auto Select list with members. A list without
/// members keeps the behavior of any other unsupported section.
pub(crate) fn is_auto_list_section(section: &FrameSection) -> bool {
    is_auto_list(section) && !section.auto_select_list.is_empty()
}

fn refusal(message: impl Into<String>) -> DesignError {
    DesignError::Workflow(message.into())
}

/// A copy of `model` in which every frame assigned an Auto Select list has the
/// member its last analysis used (the median if it was never recorded). The
/// effective length factors of neighbouring members are computed on it.
pub(crate) fn analysis_view(model: &SapModel) -> Result<SapModel, DesignError> {
    let mut view = model.clone();
    for (name, frame) in &model.frames {
        let Some(section) = model.frame_sections.get(&frame.section) else {
            continue;
        };
        if !is_auto_list_section(section) {
            continue;
        }
        let members = list::list_members(model, &frame.section)
            .map_err(|e| refusal(format!("frame '{name}': {e}")))?;
        let recorded = frame
            .auto_select
            .as_ref()
            .and_then(|s| s.analysis.as_ref())
            .filter(|a| members.iter().any(|m| &m.name == *a))
            .cloned();
        let member = match recorded {
            Some(m) => m,
            None => list::median_member(model, &frame.section)
                .map_err(|e| refusal(format!("frame '{name}': {e}")))?,
        };
        if let Some(f) = view.frames.get_mut(name) {
            f.section = member;
        }
    }
    Ok(view)
}

/// Chooses the section of the list `list_name` for one frame from the forces
/// of the design combinations. Returns the result of the chosen section and
/// its name, or `None` when no combination has forces for the frame.
///
/// D127: a candidate whose design is a member refusal is passed over (the strict form, `strict`,
/// returns its error instead, as before); the fallback "heaviest, OVERSTRESSED" is the heaviest
/// candidate that was designed, and the result carries one note naming the sections passed over.
/// When every candidate is refused the frame is refused with `AutoSelectNoSection`.
#[allow(clippy::too_many_arguments)]
pub(crate) fn design_frame_from_list(
    model: &SapModel,
    k_model: &SapModel,
    frame_name: &str,
    frame: &FrameObj,
    list_name: &str,
    overwrites: &HashMap<String, SteelMemberOverwrite>,
    code: SteelCode,
    combo_forces: &[(String, Vec<(String, crate::steel::FrameStates)>)],
    dc_limit: f64,
    strict: bool,
) -> Result<Option<(SteelDesignResult, String)>, DesignError> {
    let candidates = list::members_by_weight(model, list_name)
        .map_err(|e| refusal(format!("frame '{frame_name}': {e}")))?;
    let mut last: Option<(SteelDesignResult, String)> = None;
    // The sections passed over, with the text of the first reason.
    let mut passed_over: Vec<String> = Vec::new();
    let mut first_reason: Option<String> = None;
    let note = |passed_over: &[String], first: &Option<String>| -> Option<String> {
        let first = first.as_ref()?;
        let count = passed_over.len();
        let names = passed_over
            .iter()
            .enumerate()
            .map(|(i, n)| if i == 0 { format!("{n} ({first})") } else { n.clone() })
            .collect::<Vec<_>>()
            .join(", ");
        let plural = if count == 1 { "section" } else { "sections" };
        Some(format!(
            "Auto Select passed over {count} {plural} that {} not designed: {names}",
            if count == 1 { "is" } else { "are" }
        ))
    };
    for sec in candidates {
        let mat = model
            .materials
            .get(&sec.material)
            .ok_or_else(|| refusal(format!("section '{}' has no material", sec.name)))?;
        let designed = match design_frame_with_section(
            model,
            k_model,
            frame_name,
            frame,
            sec,
            mat,
            overwrites,
            code,
            combo_forces,
        ) {
            Ok(designed) => designed,
            Err(e) if !strict && e.is_member_refusal() => {
                if first_reason.is_none() {
                    first_reason = Some(e.to_string());
                }
                passed_over.push(sec.name.clone());
                continue;
            }
            Err(e) => return Err(e),
        };
        let Some(mut result) = designed else {
            return Ok(None);
        };
        if result.status == "OK" && result.total_ratio <= dc_limit {
            if let Some(n) = note(&passed_over, &first_reason) {
                result.notes.push(n);
            }
            return Ok(Some((result, sec.name.clone())));
        }
        last = Some((result, sec.name.clone()));
    }
    // No section is acceptable: the heaviest that was designed is used and flagged overstressed.
    match last {
        Some((mut result, name)) => {
            result.status = "OVERSTRESSED".to_string();
            if let Some(n) = note(&passed_over, &first_reason) {
                result.notes.push(n);
            }
            Ok(Some((result, name)))
        }
        None if !passed_over.is_empty() => Err(DesignError::AutoSelectNoSection {
            frame: frame_name.to_string(),
            list: list_name.to_string(),
            first: first_reason.unwrap_or_default(),
        }),
        None => Ok(None),
    }
}

fn require_locked_with_results(model: &SapModel, what: &str) -> Result<(), DesignError> {
    if !model.is_locked || model.case_results.is_empty() {
        return Err(refusal(format!(
            "{what} requires a locked model with analysis results; run the analysis first"
        )));
    }
    Ok(())
}

/// Frames whose last analysis section differs from the current design
/// section (design manual appendix E), in model order. Frames without an Auto
/// Select list always agree. Requires a locked model with results.
///
/// D127: a frame of an Auto Select list that the last design refused (it is in
/// `steel_design_errors`) has no design section, so it cannot agree with its analysis section and
/// is listed too.
pub fn verify_analysis_vs_design(model: &SapModel) -> Result<Vec<String>, DesignError> {
    require_locked_with_results(model, "verifying analysis against design sections")?;
    let mut out = Vec::new();
    for (name, frame) in &model.frames {
        let Some(section) = model.frame_sections.get(&frame.section) else {
            continue;
        };
        if !is_auto_list(section) {
            continue;
        }
        let Some(state) = &frame.auto_select else {
            return Err(refusal(format!(
                "frame '{name}' has no recorded analysis section; run the analysis again"
            )));
        };
        let analysis = state.analysis.as_deref().ok_or_else(|| {
            refusal(format!(
                "frame '{name}' has no recorded analysis section; run the analysis again"
            ))
        })?;
        if model.steel_design_errors.contains_key(name) {
            out.push(name.clone());
            continue;
        }
        let design = if state.design.is_empty() {
            list::median_member(model, &frame.section)
                .map_err(|e| refusal(format!("frame '{name}': {e}")))?
        } else {
            state.design.clone()
        };
        if analysis != design {
            out.push(name.clone());
        }
    }
    Ok(out)
}

/// Frames whose design result is not acceptable: status other than "OK" or a
/// ratio above the D/C ratio limit. The status string itself still turns to
/// "OVERSTRESSED" only above 1.0, so the limit is checked here as well.
/// Requires a locked model with results and a completed design.
///
/// D127: a member the design refused (`steel_design_errors`) did not pass: its frame is returned
/// after the frames of the results, and a model whose members are all refused has no results but
/// is still verified.
pub fn verify_all_members_passed(model: &SapModel) -> Result<Vec<String>, DesignError> {
    require_locked_with_results(model, "verifying that all members passed")?;
    if model.steel_design_results.is_empty() && model.steel_design_errors.is_empty() {
        return Err(refusal(
            "verifying that all members passed requires steel design results; run the design first",
        ));
    }
    let limit = model.steel_design_prefs.dc_ratio_limit;
    Ok(model
        .steel_design_results
        .iter()
        .filter(|(_, r)| r.status != "OK" || r.total_ratio > limit)
        .map(|(name, _)| name.clone())
        .chain(model.steel_design_errors.keys().cloned())
        .collect())
}
