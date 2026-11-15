//! Auto Select section lists: which member of a list a frame is analyzed and
//! designed with, and the steel design preferences that govern the choice.
//!
//! Source: CSI Steel Frame Design Manual AISC 360-10, section 2.1 (design with
//! Auto Select lists: sections are ordered by weight per unit length, the
//! median section by weight is used for the first analysis) and Appendix E
//! (analysis sections versus design sections; table B-1 for the D/C ratio
//! limit, default 0.95).
//!
//! How a list is stored: a frame section with `prop_type == Auto` whose
//! `auto_select_list` names the member sections; a frame is assigned the list
//! by naming it in `FrameObj::section`.
//!
//! Weight per unit length of a member is `area * unit_weight` of its material
//! (property modifiers are ignored). The median of an even-sized list is the
//! lower of the two middle members, and members of equal weight keep their
//! list order (the sort is stable). Both are choices of this implementation:
//! the manual does not say how an even count or a tie is resolved.

use crate::{FramePropType, FrameSection, MatType, SapModel};
use serde::{Deserialize, Serialize};

/// Sections of a frame whose `section` names an Auto Select list.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct AutoSelectState {
    /// Member of the list chosen by the last design; empty before any design.
    pub design: String,
    /// Member of the list the last analysis used.
    pub analysis: Option<String>,
}

/// Steel frame design preferences.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SteelDesignPreferences {
    /// Largest demand/capacity ratio a member may have to pass (manual
    /// table B-1: at most 1.0, default 0.95).
    pub dc_ratio_limit: f64,
}

impl Default for SteelDesignPreferences {
    fn default() -> Self {
        Self {
            dc_ratio_limit: 0.95,
        }
    }
}

impl SteelDesignPreferences {
    pub fn is_default(&self) -> bool {
        *self == Self::default()
    }

    /// The limit must be a positive number not above 1.0 (table B-1).
    pub fn validate(&self) -> Result<(), String> {
        let v = self.dc_ratio_limit;
        if v.is_finite() && v > 0.0 && v <= 1.0 {
            Ok(())
        } else {
            Err(format!(
                "D/C ratio limit {v} is outside (0, 1.0] (steel design manual, table B-1)"
            ))
        }
    }
}

/// True for a frame section that is an Auto Select list.
pub fn is_auto_list(section: &FrameSection) -> bool {
    section.prop_type == FramePropType::Auto
}

/// Weight per unit length of a section: area times the unit weight of its material.
pub fn weight_per_length(model: &SapModel, section: &FrameSection) -> Result<f64, String> {
    let material = model.materials.get(&section.material).ok_or_else(|| {
        format!(
            "section '{}' names the missing material '{}'",
            section.name, section.material
        )
    })?;
    let w = section.area * material.unit_weight;
    if w.is_finite() && w > 0.0 {
        Ok(w)
    } else {
        Err(format!(
            "section '{}' has no positive weight per length (area {}, unit weight {})",
            section.name, section.area, material.unit_weight
        ))
    }
}

/// The member sections of the list `list`, in list order. Refuses an unknown
/// or non-list section, an empty list, a missing member, a member that is
/// itself an Auto Select list and a member whose material is not steel.
pub fn list_members<'a>(model: &'a SapModel, list: &str) -> Result<Vec<&'a FrameSection>, String> {
    let section = model
        .frame_sections
        .get(list)
        .ok_or_else(|| format!("unknown section '{list}'"))?;
    if !is_auto_list(section) {
        return Err(format!("section '{list}' is not an Auto Select list"));
    }
    if section.auto_select_list.is_empty() {
        return Err(format!("Auto Select list '{list}' is empty"));
    }
    let mut members = Vec::with_capacity(section.auto_select_list.len());
    for name in &section.auto_select_list {
        let member = model.frame_sections.get(name).ok_or_else(|| {
            format!("Auto Select list '{list}' names the missing section '{name}'")
        })?;
        if is_auto_list(member) {
            return Err(format!(
                "Auto Select list '{list}' contains the Auto Select list '{name}'; lists cannot be nested"
            ));
        }
        let material = model.materials.get(&member.material).ok_or_else(|| {
            format!(
                "list member '{name}' names the missing material '{}'",
                member.material
            )
        })?;
        if material.mat_type != MatType::Steel {
            return Err(format!(
                "list member '{name}' of '{list}' has the non-steel material '{}'",
                material.name
            ));
        }
        weight_per_length(model, member)?;
        members.push(member);
    }
    Ok(members)
}

/// The members of `list` in ascending order of weight per length; members of
/// equal weight keep their list order.
pub fn members_by_weight<'a>(
    model: &'a SapModel,
    list: &str,
) -> Result<Vec<&'a FrameSection>, String> {
    let mut members = list_members(model, list)?;
    let mut keyed = Vec::with_capacity(members.len());
    for m in members.drain(..) {
        keyed.push((weight_per_length(model, m)?, m));
    }
    keyed.sort_by(|a, b| a.0.total_cmp(&b.0));
    Ok(keyed.into_iter().map(|(_, m)| m).collect())
}

/// The median member by weight per length; for an even count the lower of the
/// two middle members.
pub fn median_member(model: &SapModel, list: &str) -> Result<String, String> {
    let sorted = members_by_weight(model, list)?;
    Ok(sorted[(sorted.len() - 1) / 2].name.clone())
}

/// The member a frame assigned the list is analyzed with: the chosen design
/// section if there is one, else the median member.
fn analysis_member(
    model: &SapModel,
    list: &str,
    state: Option<&AutoSelectState>,
) -> Result<String, String> {
    match state.map(|s| s.design.as_str()).filter(|d| !d.is_empty()) {
        Some(design) => {
            let members = list_members(model, list)?;
            if members.iter().any(|m| m.name == design) {
                Ok(design.to_string())
            } else {
                Err(format!(
                    "design section '{design}' is not a member of Auto Select list '{list}'"
                ))
            }
        }
        None => median_member(model, list),
    }
}

/// Replaces, in an analysis working copy, the section of every frame assigned
/// an Auto Select list by the member of the list it is analyzed with. A frame
/// whose section is missing is left to the solver's own validation.
pub fn resolve_analysis_sections(model: &mut SapModel) -> Result<(), String> {
    let mut chosen = Vec::new();
    for (name, frame) in &model.frames {
        let Some(section) = model.frame_sections.get(&frame.section) else {
            continue;
        };
        if !is_auto_list(section) {
            continue;
        }
        let member = analysis_member(model, &frame.section, frame.auto_select.as_ref())
            .map_err(|e| format!("frame '{name}': {e}"))?;
        chosen.push((name.clone(), member));
    }
    for (name, member) in chosen {
        if let Some(frame) = model.frames.get_mut(&name) {
            frame.section = member;
        }
    }
    Ok(())
}

/// Records on the model which list member each such frame was analyzed with.
/// A frame that is no longer assigned a list loses its Auto Select state.
pub fn record_analysis_sections(model: &mut SapModel) {
    let mut updates = Vec::new();
    for (name, frame) in &model.frames {
        let is_list = model
            .frame_sections
            .get(&frame.section)
            .is_some_and(is_auto_list);
        if is_list {
            if let Ok(member) = analysis_member(model, &frame.section, frame.auto_select.as_ref()) {
                updates.push((name.clone(), Some(member)));
            }
        } else if frame.auto_select.is_some() {
            updates.push((name.clone(), None));
        }
    }
    for (name, member) in updates {
        if let Some(frame) = model.frames.get_mut(&name) {
            match member {
                Some(member) => {
                    let state = frame
                        .auto_select
                        .get_or_insert_with(AutoSelectState::default);
                    state.analysis = Some(member);
                }
                None => frame.auto_select = None,
            }
        }
    }
}

fn editable(model: &SapModel) -> Result<(), String> {
    if model.is_locked {
        Err("the model is locked; unlock it before changing Auto Select lists".into())
    } else {
        Ok(())
    }
}

impl SapModel {
    /// Defines (or redefines) the Auto Select list `name` from existing steel
    /// sections. Everything is validated before the model changes.
    pub fn define_auto_select_list(
        &mut self,
        name: &str,
        members: &[String],
    ) -> Result<(), String> {
        editable(self)?;
        if name.trim().is_empty() {
            return Err("an Auto Select list needs a name".into());
        }
        if let Some(existing) = self.frame_sections.get(name) {
            if !is_auto_list(existing) {
                return Err(format!(
                    "'{name}' is already a section that is not an Auto Select list"
                ));
            }
        }
        let mut trial = self.clone();
        let mut list = FrameSection {
            name: name.to_string(),
            prop_type: FramePropType::Auto,
            auto_select_list: members.to_vec(),
            ..FrameSection::default()
        };
        if let Some(existing) = self.frame_sections.get(name) {
            list = existing.clone();
            list.auto_select_list = members.to_vec();
        }
        trial.frame_sections.insert(name.to_string(), list.clone());
        list_members(&trial, name)?;
        self.frame_sections.insert(name.to_string(), list);
        self.steel_design_results.clear();
        Ok(())
    }

    /// Assigns the Auto Select list `list` to the frames `frames`. As in the
    /// design manual (appendix E) the analysis and design sections both start
    /// as the median member by weight.
    pub fn assign_auto_select_list(&mut self, list: &str, frames: &[String]) -> Result<(), String> {
        editable(self)?;
        if frames.is_empty() {
            return Err("no frames to assign the Auto Select list to".into());
        }
        for f in frames {
            if !self.frames.contains_key(f) {
                return Err(format!("unknown frame '{f}'"));
            }
        }
        let median = median_member(self, list)?;
        for f in frames {
            if let Some(frame) = self.frames.get_mut(f) {
                frame.section = list.to_string();
                frame.auto_select = Some(AutoSelectState {
                    design: median.clone(),
                    analysis: Some(median.clone()),
                });
            }
        }
        self.steel_design_results.clear();
        Ok(())
    }

    /// Sets the D/C ratio limit. Altering a design preference deletes the
    /// design results (design manual appendix E).
    pub fn set_dc_ratio_limit(&mut self, limit: f64) -> Result<(), String> {
        let prefs = SteelDesignPreferences {
            dc_ratio_limit: limit,
        };
        prefs.validate()?;
        if self.steel_design_prefs != prefs {
            self.steel_design_prefs = prefs;
            self.steel_design_results.clear();
        }
        Ok(())
    }
}
