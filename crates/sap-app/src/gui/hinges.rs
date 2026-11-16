//! Frame hinge property, hinge assignment and hinge overwrite panels.
//!
//! Source: CSI Analysis Reference, "Frame Hinge Properties". Every command
//! validates its input before it changes the model and refuses while the
//! model is locked. Only the hinges the solver analyzes can be defined:
//! user-defined, uncoupled, deformation controlled. The other kinds the
//! vendor program offers are listed disabled with the reason.

use eframe::egui;
use sap_model::hinge::{
    FrameHinge, FrameHingeOverwrite, HingeAcceptance, HingeBehavior, HingeBeyondE, HingeDof,
    HingePoint, HingeProp,
};
use sap_model::{FramePropType, SapModel};

/// Tab shown when the window opens from Define > Hinge Properties.
pub(super) const TAB_PROPERTIES: usize = 0;
/// Tab shown when the window opens from Assign > Frame > Hinges.
pub(super) const TAB_ASSIGN: usize = 1;
/// Tab shown when the window opens from Assign > Frame > Hinge Overwrites.
pub(super) const TAB_OVERWRITES: usize = 2;

/// Hinge kinds of the vendor form that are not analyzed, with the reason.
pub(super) const UNANALYZED_KINDS: [(&str, &str); 4] = [
    (
        "Coupled P-M2-M3 and its subsets (P-M2, P-M3, M2-M3)",
        "interaction surfaces are not implemented; define uncoupled hinges",
    ),
    ("Fiber P-M2-M3", "fiber hinges are not implemented"),
    (
        "Automatic (table based, ASCE 41 / FEMA 356)",
        "automatic hinge tables are not implemented; enter the curve",
    ),
    (
        "Force controlled (brittle)",
        "force controlled hinges are not implemented; use deformation controlled",
    ),
];

/// Window state. It is reset when another model is loaded.
pub(super) struct State {
    pub(super) open: bool,
    pub(super) tab: usize,
    /// Property the draft was read from; `None` for a new property.
    pub(super) editing: Option<String>,
    pub(super) draft: HingeProp,
    pub(super) assign_prop: String,
    pub(super) assign_distances: String,
    pub(super) assign_replace: bool,
    pub(super) overwrite: FrameHingeOverwrite,
}

/// The curve a new property starts from: normalized force 1 at B, a plateau
/// with a small strain-hardening step to C, a drop to D and a residual to E.
pub(super) fn default_draft() -> HingeProp {
    HingeProp::symmetric(
        "HINGE1",
        HingeDof::M3,
        [
            HingePoint::new(1.0, 0.0),
            HingePoint::new(1.1, 1.0),
            HingePoint::new(0.2, 1.0),
            HingePoint::new(0.2, 2.0),
        ],
    )
}

impl Default for State {
    fn default() -> Self {
        Self {
            open: false,
            tab: TAB_PROPERTIES,
            editing: None,
            draft: default_draft(),
            assign_prop: String::new(),
            assign_distances: "0, 1".into(),
            assign_replace: false,
            overwrite: FrameHingeOverwrite::default(),
        }
    }
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

fn locked(model: &SapModel) -> Result<(), String> {
    if model.is_locked {
        Err("The model is locked. Unlock it before changing hinges.".into())
    } else {
        Ok(())
    }
}

fn valid_name(name: &str) -> Result<(), String> {
    if name.is_empty()
        || name != name.trim()
        || name.contains('"')
        || name.chars().any(char::is_control)
    {
        return Err(
            "The property name must be nonempty, trimmed, and contain no quotes or control characters."
                .into(),
        );
    }
    if name.eq_ignore_ascii_case("auto") {
        return Err(
            "'Auto' is the name the vendor program gives generated hinges; choose another name."
                .into(),
        );
    }
    Ok(())
}

/// Frames that use the property, in model order.
pub(super) fn frames_using(model: &SapModel, prop: &str) -> Vec<String> {
    model
        .frames
        .values()
        .filter(|f| f.hinges.iter().any(|h| h.prop == prop))
        .map(|f| f.name.clone())
        .collect()
}

/// The property as the model stores it: a symmetric property carries the
/// mirror of its positive branch, scale factors and acceptance criteria.
fn normalized(mut prop: HingeProp) -> HingeProp {
    if prop.symmetric {
        prop.negative = prop
            .positive
            .map(|p| HingePoint::new(-p.force, -p.deformation));
        prop.force_sf_negative = prop.force_sf;
        prop.deformation_sf_negative = prop.deformation_sf;
        prop.acceptance_negative = HingeAcceptance {
            io: -prop.acceptance_positive.io,
            ls: -prop.acceptance_positive.ls,
            cp: -prop.acceptance_positive.cp,
        };
    }
    prop
}

fn check_property(prop: &HingeProp) -> Result<(), String> {
    valid_name(&prop.name)?;
    if prop.behavior == HingeBehavior::ForceControlled {
        return Err(format!(
            "Hinge property '{}' is force controlled; {}.",
            prop.name, UNANALYZED_KINDS[3].1
        ));
    }
    if prop.beyond_e.is_none() {
        return Err(format!(
            "Hinge property '{}': choose what the hinge does beyond point E (drops to zero or is extrapolated).",
            prop.name
        ));
    }
    prop.validate()
}

/// Defines a new hinge property.
pub(super) fn add_property(model: &mut SapModel, prop: HingeProp) -> Result<String, String> {
    locked(model)?;
    check_property(&prop)?;
    if model.hinge_props.contains_key(&prop.name) {
        return Err(format!(
            "Hinge property '{}' already exists; modify it instead.",
            prop.name
        ));
    }
    let name = prop.name.clone();
    model.hinge_props.insert(name.clone(), normalized(prop));
    model.invalidate_results();
    Ok(format!("Defined hinge property '{name}'."))
}

/// Replaces the definition of an existing hinge property (same name).
pub(super) fn modify_property(model: &mut SapModel, prop: HingeProp) -> Result<String, String> {
    locked(model)?;
    check_property(&prop)?;
    if !model.hinge_props.contains_key(&prop.name) {
        return Err(format!(
            "Hinge property '{}' is not defined; add it first.",
            prop.name
        ));
    }
    let name = prop.name.clone();
    model.hinge_props.insert(name.clone(), normalized(prop));
    model.invalidate_results();
    Ok(format!("Modified hinge property '{name}'."))
}

/// Deletes a hinge property that no frame uses.
pub(super) fn delete_property(model: &mut SapModel, name: &str) -> Result<String, String> {
    locked(model)?;
    if !model.hinge_props.contains_key(name) {
        return Err(format!("Hinge property '{name}' is not defined."));
    }
    let users = frames_using(model, name);
    if let Some(first) = users.first() {
        return Err(format!(
            "Hinge property '{name}' is used by {} frame(s), for example frame '{first}'; remove those hinges first.",
            users.len()
        ));
    }
    model.hinge_props.shift_remove(name);
    model.invalidate_results();
    Ok(format!("Deleted hinge property '{name}'."))
}

fn selected_frames(model: &SapModel) -> Result<Vec<String>, String> {
    let frames: Vec<String> = model
        .frames
        .values()
        .filter(|f| f.selected)
        .map(|f| f.name.clone())
        .collect();
    if frames.is_empty() {
        Err("No frames are selected.".into())
    } else {
        Ok(frames)
    }
}

/// Parses relative distances separated by commas, semicolons or spaces.
pub(super) fn parse_distances(text: &str) -> Result<Vec<f64>, String> {
    let mut out = Vec::new();
    for token in text
        .split(|c: char| c == ',' || c == ';' || c.is_whitespace())
        .filter(|t| !t.is_empty())
    {
        out.push(
            token
                .parse::<f64>()
                .map_err(|_| format!("'{token}' is not a number."))?,
        );
    }
    if out.is_empty() {
        return Err("Enter at least one relative distance.".into());
    }
    Ok(out)
}

/// A condition of the frame that makes the analysis refuse its hinges, as
/// the solver states it; `None` when the frame has none of them.
fn analysis_obstacle(model: &SapModel, frame: &sap_model::FrameObj) -> Option<&'static str> {
    let section_varies = model
        .frame_sections
        .get(&frame.section)
        .is_some_and(|s| s.prop_type == FramePropType::Variable && !s.non_prismatic.is_empty());
    if frame.release_i.iter().chain(&frame.release_j).any(|r| *r)
        || frame
            .start_spring
            .iter()
            .chain(&frame.end_spring)
            .any(|k| *k != 0.0)
    {
        Some("end releases or partial fixity")
    } else if frame.end_offset_i != 0.0 || frame.end_offset_j != 0.0 {
        Some("end length offsets")
    } else if section_varies {
        Some("a non-prismatic section")
    } else if frame.cable_params.is_some() {
        Some("a cable")
    } else if frame.tc_limits.is_some_and(|(c, _, t, _)| c || t) {
        Some("a tension or compression limit")
    } else {
        None
    }
}

/// Assigns hinges of one property at the given relative distances (from
/// end I, in `[0, 1]`) to the selected frames, adding to the hinges they have
/// or replacing all of them.
pub(super) fn assign_hinges(
    model: &mut SapModel,
    prop: &str,
    distances: &[f64],
    replace: bool,
) -> Result<String, String> {
    locked(model)?;
    let Some(property) = model.hinge_props.get(prop) else {
        return Err(format!(
            "Hinge property '{prop}' is not defined; define it first."
        ));
    };
    let dof = property.dof;
    if distances.is_empty() {
        return Err("Enter at least one relative distance.".into());
    }
    for (k, d) in distances.iter().enumerate() {
        if !d.is_finite() || !(0.0..=1.0).contains(d) {
            return Err(format!(
                "Relative distance {d} is outside the object; use a value from 0 to 1 measured from end I."
            ));
        }
        if distances[..k].iter().any(|e| (e - d).abs() <= 1e-9) {
            return Err(format!(
                "Relative distance {d} is given twice; one property can be assigned once at a position."
            ));
        }
    }
    let frames = selected_frames(model)?;
    if !replace {
        for name in &frames {
            for existing in &model.frames[name].hinges {
                let same_dof = model
                    .hinge_props
                    .get(&existing.prop)
                    .is_some_and(|p| p.dof == dof);
                if same_dof
                    && distances
                        .iter()
                        .any(|d| (d - existing.rel_dist).abs() <= 1e-9)
                {
                    return Err(format!(
                        "Frame '{name}' already has a {} hinge ('{}') at relative distance {}.",
                        dof.label(),
                        existing.prop,
                        existing.rel_dist
                    ));
                }
            }
        }
    }
    let mut notes = Vec::new();
    for name in &frames {
        if let Some(reason) = analysis_obstacle(model, &model.frames[name]) {
            notes.push(format!(
                "frame '{name}' has {reason}, which the analysis refuses together with hinges"
            ));
        }
        let frame = model.frames.get_mut(name).expect("selected frame exists");
        if replace {
            frame.hinges.clear();
        }
        for d in distances {
            frame.hinges.push(FrameHinge {
                prop: prop.to_string(),
                rel_dist: *d,
            });
        }
    }
    model.invalidate_results();
    let mut message = format!(
        "{} {} hinge(s) '{prop}' on {} frame(s).",
        if replace { "Replaced with" } else { "Added" },
        distances.len(),
        frames.len()
    );
    if !notes.is_empty() {
        message.push_str(&format!(" Note: {}.", notes.join("; ")));
    }
    Ok(message)
}

/// Removes every hinge from the selected frames.
pub(super) fn remove_hinges(model: &mut SapModel) -> Result<String, String> {
    locked(model)?;
    let frames = selected_frames(model)?;
    let mut removed = 0;
    for name in &frames {
        let frame = model.frames.get_mut(name).expect("selected frame exists");
        removed += frame.hinges.len();
        frame.hinges.clear();
    }
    if removed == 0 {
        return Err("The selected frames have no hinges.".into());
    }
    model.invalidate_results();
    Ok(format!(
        "Removed {removed} hinge(s) from {} frame(s).",
        frames.len()
    ))
}

/// Assigns a hinge overwrite to the selected frames.
pub(super) fn assign_overwrite(
    model: &mut SapModel,
    overwrite: FrameHingeOverwrite,
) -> Result<String, String> {
    locked(model)?;
    if !overwrite.relative_length.is_finite()
        || overwrite.relative_length <= 0.0
        || overwrite.relative_length > 1.0
    {
        return Err(format!(
            "The hinge length {} must lie in (0, 1] of the frame length.",
            overwrite.relative_length
        ));
    }
    if overwrite.do_not_drop_load {
        return Err(
            "\"Do not allow hinges to drop loads\" is stored by the vendor program but not analyzed; leave it off."
                .into(),
        );
    }
    let frames = selected_frames(model)?;
    for name in &frames {
        model
            .frames
            .get_mut(name)
            .expect("selected frame exists")
            .hinge_overwrite = Some(overwrite.clone());
    }
    model.invalidate_results();
    Ok(format!(
        "Assigned the hinge overwrite (subdivide {}, length {}) to {} frame(s).",
        if overwrite.auto_subdivide {
            "on"
        } else {
            "off"
        },
        overwrite.relative_length,
        frames.len()
    ))
}

/// Removes the hinge overwrite from the selected frames.
pub(super) fn remove_overwrite(model: &mut SapModel) -> Result<String, String> {
    locked(model)?;
    let frames = selected_frames(model)?;
    let mut removed = 0;
    for name in &frames {
        if model
            .frames
            .get_mut(name)
            .expect("selected frame exists")
            .hinge_overwrite
            .take()
            .is_some()
        {
            removed += 1;
        }
    }
    if removed == 0 {
        return Err("The selected frames have no hinge overwrite.".into());
    }
    model.invalidate_results();
    Ok(format!(
        "Removed the hinge overwrite from {removed} frame(s)."
    ))
}

// ---------------------------------------------------------------------------
// Window
// ---------------------------------------------------------------------------

fn disabled_kinds(ui: &mut egui::Ui) {
    for (kind, reason) in UNANALYZED_KINDS {
        ui.horizontal(|ui| {
            ui.add_enabled(false, egui::Button::new(kind));
            ui.weak(format!("not available: {reason}"));
        });
    }
}

fn point_rows(ui: &mut egui::Ui, id: &str, points: &mut [HingePoint; 4]) {
    egui::Grid::new(id).show(ui, |ui| {
        ui.label("Point");
        ui.label("Force");
        ui.label("Deformation");
        ui.end_row();
        for (label, p) in ["B", "C", "D", "E"].iter().zip(points.iter_mut()) {
            ui.label(*label);
            ui.add(egui::DragValue::new(&mut p.force).speed(0.01));
            ui.add(egui::DragValue::new(&mut p.deformation).speed(0.001));
            ui.end_row();
        }
    });
}

fn branch_scale(
    ui: &mut egui::Ui,
    force_sf: &mut f64,
    deformation_sf: &mut f64,
    acceptance: &mut HingeAcceptance,
) {
    ui.horizontal(|ui| {
        ui.label("Force scale factor");
        ui.add(egui::DragValue::new(force_sf).speed(0.1));
        ui.label("Deformation scale factor");
        ui.add(egui::DragValue::new(deformation_sf).speed(0.001));
    });
    ui.horizontal(|ui| {
        ui.label("Acceptance IO, LS, CP (table deformations, all 0 for none)");
        ui.add(egui::DragValue::new(&mut acceptance.io).speed(0.01));
        ui.add(egui::DragValue::new(&mut acceptance.ls).speed(0.01));
        ui.add(egui::DragValue::new(&mut acceptance.cp).speed(0.01));
    });
}

fn properties_tab(
    ui: &mut egui::Ui,
    state: &mut State,
    model: &mut SapModel,
    outcome: &mut Option<Result<String, String>>,
) {
    ui.label("Hinge property (uncoupled, deformation controlled, user defined):");
    ui.horizontal(|ui| {
        egui::ComboBox::from_id_salt("hinge_prop_read")
            .selected_text(state.editing.as_deref().unwrap_or("New property"))
            .show_ui(ui, |ui| {
                if ui
                    .selectable_label(state.editing.is_none(), "New property")
                    .clicked()
                {
                    state.editing = None;
                    state.draft = default_draft();
                }
                for (name, prop) in &model.hinge_props {
                    if ui
                        .selectable_label(state.editing.as_deref() == Some(name), name)
                        .clicked()
                    {
                        state.editing = Some(name.clone());
                        state.draft = prop.clone();
                    }
                }
            });
        ui.label(format!(
            "{} defined. Force and moment in model units ({}).",
            model.hinge_props.len(),
            model.present_units.s2k_label()
        ));
    });
    ui.horizontal(|ui| {
        ui.label("Name");
        let editable = state.editing.is_none();
        ui.add_enabled(
            editable,
            egui::TextEdit::singleline(&mut state.draft.name).desired_width(140.0),
        );
        ui.label("Degree of freedom");
        egui::ComboBox::from_id_salt("hinge_prop_dof")
            .selected_text(state.draft.dof.label())
            .show_ui(ui, |ui| {
                for dof in HingeDof::ALL {
                    ui.selectable_value(&mut state.draft.dof, dof, dof.label());
                }
            });
    });
    disabled_kinds(ui);
    ui.separator();
    ui.label("Positive branch (points B, C, D, E; point A is the origin):");
    point_rows(ui, "hinge_pos_points", &mut state.draft.positive);
    let State { draft, .. } = state;
    branch_scale(
        ui,
        &mut draft.force_sf,
        &mut draft.deformation_sf,
        &mut draft.acceptance_positive,
    );
    ui.checkbox(
        &mut state.draft.symmetric,
        "Symmetric (the negative branch mirrors the positive one)",
    );
    if !state.draft.symmetric {
        ui.label("Negative branch (entered with signs, values 0 or below):");
        point_rows(ui, "hinge_neg_points", &mut state.draft.negative);
        let State { draft, .. } = state;
        branch_scale(
            ui,
            &mut draft.force_sf_negative,
            &mut draft.deformation_sf_negative,
            &mut draft.acceptance_negative,
        );
    }
    let mut beyond = state.draft.beyond_e.unwrap_or_default();
    ui.horizontal(|ui| {
        ui.label("Load carrying capacity beyond point E:");
        ui.selectable_value(&mut beyond, HingeBeyondE::DropsToZero, "Drops to zero");
        ui.selectable_value(&mut beyond, HingeBeyondE::Extrapolated, "Is extrapolated");
    });
    state.draft.beyond_e = Some(beyond);
    ui.horizontal(|ui| {
        if ui.button("Add New Property").clicked() {
            *outcome = Some(add_property(model, state.draft.clone()).inspect(|_| {
                state.editing = Some(state.draft.name.clone());
            }));
        }
        if ui.button("Modify Property").clicked() {
            *outcome = Some(match &state.editing {
                Some(name) if *name == state.draft.name => {
                    modify_property(model, state.draft.clone())
                }
                _ => Err("Choose an existing property to modify.".into()),
            });
        }
        if ui.button("Delete Property").clicked() {
            *outcome = Some(match state.editing.clone() {
                Some(name) => delete_property(model, &name).inspect(|_| {
                    state.editing = None;
                    state.draft = default_draft();
                }),
                None => Err("Choose an existing property to delete.".into()),
            });
        }
    });
}

fn assign_tab(
    ui: &mut egui::Ui,
    state: &mut State,
    model: &mut SapModel,
    outcome: &mut Option<Result<String, String>>,
) {
    let selected: Vec<&sap_model::FrameObj> =
        model.frames.values().filter(|f| f.selected).collect();
    ui.label(format!("{} frame(s) selected.", selected.len()));
    for frame in selected.iter().take(8) {
        let list: Vec<String> = frame
            .hinges
            .iter()
            .map(|h| format!("{} at {}", h.prop, h.rel_dist))
            .collect();
        ui.label(format!(
            "{}: {}",
            frame.name,
            if list.is_empty() {
                "no hinges".to_string()
            } else {
                list.join(", ")
            }
        ));
    }
    ui.separator();
    ui.horizontal(|ui| {
        ui.label("Property");
        egui::ComboBox::from_id_salt("hinge_assign_prop")
            .selected_text(state.assign_prop.as_str())
            .show_ui(ui, |ui| {
                for name in model.hinge_props.keys() {
                    ui.selectable_value(&mut state.assign_prop, name.clone(), name);
                }
            });
    });
    ui.horizontal(|ui| {
        ui.label("Relative distances from end I (0 to 1, separated by commas)");
        ui.text_edit_singleline(&mut state.assign_distances);
    });
    ui.checkbox(
        &mut state.assign_replace,
        "Replace the existing hinges (otherwise add to them)",
    );
    ui.horizontal(|ui| {
        if ui.button("Assign Hinges to Selected Frames").clicked() {
            *outcome =
                Some(parse_distances(&state.assign_distances).and_then(|d| {
                    assign_hinges(model, &state.assign_prop, &d, state.assign_replace)
                }));
        }
        if ui.button("Remove Hinges from Selected Frames").clicked() {
            *outcome = Some(remove_hinges(model));
        }
    });
}

fn overwrites_tab(
    ui: &mut egui::Ui,
    state: &mut State,
    model: &mut SapModel,
    outcome: &mut Option<Result<String, String>>,
) {
    ui.label(format!(
        "{} frame(s) selected.",
        model.frames.values().filter(|f| f.selected).count()
    ));
    ui.checkbox(
        &mut state.overwrite.auto_subdivide,
        "Auto subdivide line objects at hinges",
    );
    ui.horizontal(|ui| {
        ui.label("Relative length of the element containing a hinge (0 to 1)");
        ui.add(
            egui::DragValue::new(&mut state.overwrite.relative_length)
                .speed(0.005)
                .max_decimals(6),
        );
    });
    ui.horizontal(|ui| {
        ui.add_enabled(
            false,
            egui::Checkbox::new(
                &mut state.overwrite.do_not_drop_load,
                "Do not allow hinges to drop loads",
            ),
        );
        ui.weak("not available: stored by the vendor program, not analyzed");
    });
    state.overwrite.do_not_drop_load = false;
    ui.horizontal(|ui| {
        if ui.button("Assign Overwrite to Selected Frames").clicked() {
            *outcome = Some(assign_overwrite(model, state.overwrite.clone()));
        }
        if ui.button("Remove Overwrite from Selected Frames").clicked() {
            *outcome = Some(remove_overwrite(model));
        }
    });
}

pub(super) fn render(
    ctx: &egui::Context,
    state: &mut State,
    model: &mut SapModel,
) -> Option<Result<String, String>> {
    if !state.open {
        return None;
    }
    let mut open = true;
    let mut outcome = None;
    egui::Window::new("Frame Hinges")
        .open(&mut open)
        .default_width(560.0)
        .vscroll(true)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.selectable_value(&mut state.tab, TAB_PROPERTIES, "Hinge Properties");
                ui.selectable_value(&mut state.tab, TAB_ASSIGN, "Assign Hinges");
                ui.selectable_value(&mut state.tab, TAB_OVERWRITES, "Hinge Overwrites");
            });
            if model.is_locked {
                ui.colored_label(
                    egui::Color32::YELLOW,
                    "The model is locked. Unlock it before changing hinges.",
                );
            }
            ui.separator();
            match state.tab {
                TAB_PROPERTIES => properties_tab(ui, state, model, &mut outcome),
                TAB_ASSIGN => assign_tab(ui, state, model, &mut outcome),
                _ => overwrites_tab(ui, state, model, &mut outcome),
            }
        });
    state.open = open;
    outcome
}

#[cfg(test)]
mod tests {
    use super::*;
    use eframe::egui::{Pos2, Rect, Vec2};
    use sap_units::Units;

    fn model() -> SapModel {
        let mut m = SapModel::new(Units::KnMC);
        m.add_point("1", 0.0, 0.0, 0.0);
        m.add_point("2", 4.0, 0.0, 0.0);
        m.add_point("3", 8.0, 0.0, 0.0);
        m.add_frame("A", "1", "2", "FSEC1");
        m.add_frame("B", "2", "3", "FSEC1");
        m
    }

    fn select(m: &mut SapModel, names: &[&str]) {
        for f in m.frames.values_mut() {
            f.selected = names.contains(&f.name.as_str());
        }
    }

    fn texts(out: &egui::FullOutput) -> Vec<String> {
        out.shapes
            .iter()
            .filter_map(|s| match &s.shape {
                egui::epaint::Shape::Text(t) => Some(t.galley.job.text.clone()),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn property_add_modify_delete_validate_before_changing() {
        let mut m = model();
        // Bad curves and names change nothing.
        let mut bad = default_draft();
        bad.positive[0].force = 0.0;
        assert!(add_property(&mut m, bad).unwrap_err().contains("point B"));
        let mut bad = default_draft();
        bad.positive[2].deformation = 0.5; // decreases from C to D
        assert!(add_property(&mut m, bad).is_err());
        let mut bad = default_draft();
        bad.force_sf = 0.0;
        assert!(add_property(&mut m, bad).is_err());
        let mut bad = default_draft();
        bad.name = " x".into();
        assert!(add_property(&mut m, bad).is_err());
        let mut bad = default_draft();
        bad.name = "auto".into();
        assert!(add_property(&mut m, bad).is_err());
        let mut bad = default_draft();
        bad.acceptance_positive = HingeAcceptance {
            io: 3.0,
            ls: 2.0,
            cp: 1.0,
        };
        assert!(add_property(&mut m, bad).is_err());
        let mut bad = default_draft();
        bad.beyond_e = None;
        assert!(add_property(&mut m, bad).is_err());
        let mut bad = default_draft();
        bad.behavior = HingeBehavior::ForceControlled;
        assert!(add_property(&mut m, bad)
            .unwrap_err()
            .contains("force controlled"));
        assert!(m.hinge_props.is_empty());

        add_property(&mut m, default_draft()).unwrap();
        assert!(add_property(&mut m, default_draft())
            .unwrap_err()
            .contains("already exists"));
        // Modify: a non-symmetric property keeps its own negative branch.
        let mut p = default_draft();
        p.symmetric = false;
        p.negative = [
            HingePoint::new(-0.8, 0.0),
            HingePoint::new(-0.9, -1.0),
            HingePoint::new(-0.1, -1.0),
            HingePoint::new(-0.1, -2.0),
        ];
        p.force_sf_negative = 2.0;
        p.beyond_e = Some(HingeBeyondE::Extrapolated);
        modify_property(&mut m, p.clone()).unwrap();
        assert_eq!(m.hinge_props["HINGE1"], p);
        // An invalid negative branch is refused, a symmetric one is not checked.
        let mut q = p.clone();
        q.negative[1].deformation = 1.0; // positive deformation on the negative branch
        assert!(modify_property(&mut m, q.clone()).is_err());
        assert_eq!(m.hinge_props["HINGE1"], p);
        q.symmetric = true;
        modify_property(&mut m, q).unwrap();
        // Symmetric storage mirrors the positive branch and its factors.
        let stored = &m.hinge_props["HINGE1"];
        assert_eq!(stored.negative[1], HingePoint::new(-1.1, -1.0));
        assert_eq!(stored.force_sf_negative, stored.force_sf);
        // Modify of an undefined property is refused.
        let mut other = default_draft();
        other.name = "OTHER".into();
        assert!(modify_property(&mut m, other)
            .unwrap_err()
            .contains("not defined"));
        delete_property(&mut m, "HINGE1").unwrap();
        assert!(delete_property(&mut m, "HINGE1").is_err());
        assert!(m.hinge_props.is_empty());
    }

    #[test]
    fn deleting_a_used_property_is_refused_naming_a_frame() {
        let mut m = model();
        add_property(&mut m, default_draft()).unwrap();
        select(&mut m, &["B"]);
        assign_hinges(&mut m, "HINGE1", &[0.0, 1.0], false).unwrap();
        let error = delete_property(&mut m, "HINGE1").unwrap_err();
        assert!(error.contains("frame 'B'"), "{error}");
        assert!(m.hinge_props.contains_key("HINGE1"));
        remove_hinges(&mut m).unwrap();
        delete_property(&mut m, "HINGE1").unwrap();
    }

    #[test]
    fn results_are_invalidated_by_every_change() {
        let mut m = model();
        m.case_results.insert("OLD".into(), Default::default());
        add_property(&mut m, default_draft()).unwrap();
        assert!(m.case_results.is_empty());
        for step in 0..4 {
            m.case_results.insert("OLD".into(), Default::default());
            select(&mut m, &["A"]);
            match step {
                0 => assign_hinges(&mut m, "HINGE1", &[0.5], false).unwrap(),
                1 => assign_overwrite(&mut m, FrameHingeOverwrite::default()).unwrap(),
                2 => remove_overwrite(&mut m).unwrap(),
                _ => remove_hinges(&mut m).unwrap(),
            };
            assert!(m.case_results.is_empty(), "step {step}");
        }
    }

    #[test]
    fn hinge_assignment_adds_replaces_and_refuses() {
        let mut m = model();
        add_property(&mut m, default_draft()).unwrap();
        let mut shear = default_draft();
        shear.name = "V".into();
        shear.dof = HingeDof::V2;
        add_property(&mut m, shear).unwrap();

        // Nothing selected, undefined property, bad distances: nothing changes.
        assert!(assign_hinges(&mut m, "HINGE1", &[0.5], false)
            .unwrap_err()
            .contains("selected"));
        select(&mut m, &["A", "B"]);
        let before = m.clone();
        assert!(assign_hinges(&mut m, "GHOST", &[0.5], false)
            .unwrap_err()
            .contains("not defined"));
        for bad in [-0.1, 1.0001, f64::NAN, f64::INFINITY] {
            assert!(
                assign_hinges(&mut m, "HINGE1", &[bad], false).is_err(),
                "{bad}"
            );
        }
        assert!(assign_hinges(&mut m, "HINGE1", &[0.2, 0.2], false).is_err());
        assert!(assign_hinges(&mut m, "HINGE1", &[], false).is_err());
        assert_eq!(m, before);

        // Add at both ends of both frames, endpoints included.
        assign_hinges(&mut m, "HINGE1", &[0.0, 1.0], false).unwrap();
        assert_eq!(
            m.frames["A"].hinges,
            vec![
                FrameHinge {
                    prop: "HINGE1".into(),
                    rel_dist: 0.0
                },
                FrameHinge {
                    prop: "HINGE1".into(),
                    rel_dist: 1.0
                }
            ]
        );
        // A second hinge of the same degree of freedom at the same place is
        // refused, naming the frame, and the other frame is not changed either.
        select(&mut m, &["B"]);
        m.frames.get_mut("B").unwrap().hinges.clear();
        select(&mut m, &["A", "B"]);
        let before = m.clone();
        let error = assign_hinges(&mut m, "HINGE1", &[1.0], false).unwrap_err();
        assert!(error.contains("Frame 'A'"), "{error}");
        assert_eq!(m, before);
        // A shear hinge at the same place is another degree of freedom: added.
        assign_hinges(&mut m, "V", &[1.0], false).unwrap();
        assert_eq!(m.frames["A"].hinges.len(), 3);
        assert_eq!(m.frames["B"].hinges.len(), 1);
        // Replace drops the old hinges.
        select(&mut m, &["A"]);
        assign_hinges(&mut m, "HINGE1", &[0.5], true).unwrap();
        assert_eq!(
            m.frames["A"].hinges,
            vec![FrameHinge {
                prop: "HINGE1".into(),
                rel_dist: 0.5
            }]
        );
        assert_eq!(m.frames["B"].hinges.len(), 1);
        // Removing from a frame without hinges is refused.
        select(&mut m, &["A"]);
        remove_hinges(&mut m).unwrap();
        assert!(remove_hinges(&mut m).is_err());
    }

    #[test]
    fn assignment_notes_frames_the_analysis_refuses() {
        let mut m = model();
        add_property(&mut m, default_draft()).unwrap();
        m.frames.get_mut("A").unwrap().release_j[5] = true;
        m.frames.get_mut("B").unwrap().end_offset_i = 0.1;
        select(&mut m, &["A", "B"]);
        let message = assign_hinges(&mut m, "HINGE1", &[0.5], false).unwrap();
        assert!(message.contains("frame 'A' has end releases"), "{message}");
        assert!(
            message.contains("frame 'B' has end length offsets"),
            "{message}"
        );
        // The hinges are assigned all the same; the solver states the refusal.
        assert_eq!(m.frames["A"].hinges.len(), 1);
    }

    #[test]
    fn distance_text_parses_or_names_the_bad_token() {
        assert_eq!(
            parse_distances("0, 0.5;1  0.25").unwrap(),
            vec![0.0, 0.5, 1.0, 0.25]
        );
        assert!(parse_distances("0.5, abc").unwrap_err().contains("'abc'"));
        assert!(parse_distances(" , ").is_err());
    }

    #[test]
    fn overwrite_assign_and_remove() {
        let mut m = model();
        select(&mut m, &[]);
        assert!(assign_overwrite(&mut m, FrameHingeOverwrite::default())
            .unwrap_err()
            .contains("selected"));
        select(&mut m, &["A"]);
        for length in [0.0, -0.1, 1.5, f64::NAN] {
            let o = FrameHingeOverwrite {
                relative_length: length,
                ..Default::default()
            };
            assert!(assign_overwrite(&mut m, o).is_err(), "{length}");
        }
        let o = FrameHingeOverwrite {
            do_not_drop_load: true,
            ..Default::default()
        };
        assert!(assign_overwrite(&mut m, o)
            .unwrap_err()
            .contains("not analyzed"));
        assert!(m.frames["A"].hinge_overwrite.is_none());
        let o = FrameHingeOverwrite {
            auto_subdivide: false,
            relative_length: 0.1,
            do_not_drop_load: false,
        };
        assign_overwrite(&mut m, o.clone()).unwrap();
        assert_eq!(m.frames["A"].hinge_overwrite, Some(o));
        assert!(m.frames["B"].hinge_overwrite.is_none());
        remove_overwrite(&mut m).unwrap();
        assert!(m.frames["A"].hinge_overwrite.is_none());
        assert!(remove_overwrite(&mut m)
            .unwrap_err()
            .contains("no hinge overwrite"));
    }

    #[test]
    fn every_command_refuses_while_locked() {
        let mut m = model();
        add_property(&mut m, default_draft()).unwrap();
        select(&mut m, &["A"]);
        assign_hinges(&mut m, "HINGE1", &[0.5], false).unwrap();
        assign_overwrite(&mut m, FrameHingeOverwrite::default()).unwrap();
        m.is_locked = true;
        let before = m.clone();
        let mut other = default_draft();
        other.name = "N".into();
        assert!(add_property(&mut m, other).unwrap_err().contains("locked"));
        assert!(modify_property(&mut m, default_draft())
            .unwrap_err()
            .contains("locked"));
        assert!(delete_property(&mut m, "HINGE1")
            .unwrap_err()
            .contains("locked"));
        assert!(assign_hinges(&mut m, "HINGE1", &[0.1], false)
            .unwrap_err()
            .contains("locked"));
        assert!(remove_hinges(&mut m).unwrap_err().contains("locked"));
        assert!(assign_overwrite(&mut m, FrameHingeOverwrite::default())
            .unwrap_err()
            .contains("locked"));
        assert!(remove_overwrite(&mut m).unwrap_err().contains("locked"));
        assert_eq!(m, before);
    }

    /// A model defined only through the commands of the panels runs through
    /// the solver: a cantilever with an elastic-perfectly-plastic M3 hinge at
    /// its base, pushed by displacement control. Once the hinge yields the
    /// base moment is Mp, so the tip load is Mp / L (statics).
    #[test]
    fn a_pushover_defined_with_the_commands_collapses_at_mp_over_l() {
        use super::super::analysis as cases;
        use sap_model::hinge::ResultsSaved;
        use sap_model::{LoadCase, LoadCaseType, PointLoadForce};
        let (mp, length, target) = (100.0, 100.0, 5.0); // kip-in, in, in
        let mut m = SapModel::new(Units::KipInF);
        m.load_patterns.get_mut("DEAD").unwrap().self_wt_multiplier = 0.0;
        m.add_point("1", 0.0, 0.0, 0.0);
        m.add_point("2", length, 0.0, 0.0);
        m.points.get_mut("1").unwrap().restraint = [true; 6];
        m.points
            .get_mut("2")
            .unwrap()
            .loads_force
            .push(PointLoadForce {
                load_pat: "DEAD".into(),
                csys: "GLOBAL".into(),
                force: [0.0, 0.0, -1.0, 0.0, 0.0, 0.0],
            });
        m.add_frame("C", "1", "2", "FSEC1");

        let mut prop = HingeProp::symmetric(
            "MP",
            HingeDof::M3,
            [
                HingePoint::new(1.0, 0.0),
                HingePoint::new(1.0, 1.0),
                HingePoint::new(1.0, 1.0),
                HingePoint::new(1.0, 2.0),
            ],
        );
        prop.force_sf = mp;
        add_property(&mut m, prop).unwrap();
        select(&mut m, &["C"]);
        assign_hinges(&mut m, "MP", &[0.0], false).unwrap();
        assign_overwrite(&mut m, FrameHingeOverwrite::default()).unwrap();

        let mut push = LoadCase::linear_static("PUSH", "DEAD", 1.0);
        push.case_type = LoadCaseType::NonlinearStatic;
        cases::set_displacement_control(&m, &mut push, "2", 2, target, true).unwrap();
        cases::set_results_saved(
            &mut push,
            ResultsSaved::MultipleStates {
                min_saved: 6,
                max_saved: 10,
                positive_only: true,
            },
        )
        .unwrap();
        m.load_cases.clear();
        sap_api::set_case_definition(&mut m, push).unwrap();
        sap_solver::analyze_model(&mut m).unwrap();

        let steps = &m.case_results["PUSH"].static_steps;
        assert!(steps.len() >= 2, "{} saved states", steps.len());
        let last = steps.last().unwrap();
        let expected = mp / length; // kip
        assert!(
            (last.load_factor - expected).abs() < 1e-6 * expected,
            "load factor {} vs Mp/L {expected}",
            last.load_factor
        );
        let tip = last.joint_displacements["2"][2];
        assert!((tip + target).abs() < 1e-6 * target, "tip {tip}");
        assert_eq!(last.hinges.len(), 1);
        assert!(
            (last.hinges[0].force.abs() - mp).abs() < 1e-6 * mp,
            "hinge moment {}",
            last.hinges[0].force
        );
    }

    fn render_tab(
        state: &mut State,
        m: &mut SapModel,
        ctx: &egui::Context,
        time: f64,
    ) -> egui::FullOutput {
        ctx.run(
            egui::RawInput {
                screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1400., 1400.))),
                time: Some(time),
                ..Default::default()
            },
            |ctx| {
                render(ctx, state, m);
            },
        )
    }

    #[test]
    fn each_tab_renders_headless_and_lists_the_disabled_kinds() {
        let ctx = egui::Context::default();
        let mut m = model();
        add_property(&mut m, default_draft()).unwrap();
        select(&mut m, &["A"]);
        assign_hinges(&mut m, "HINGE1", &[0.5], false).unwrap();
        let mut state = State {
            open: true,
            ..State::default()
        };
        let mut time = 0.0;
        for tab in [TAB_PROPERTIES, TAB_ASSIGN, TAB_OVERWRITES] {
            state.tab = tab;
            // Two passes: the first lays the window out.
            time += 1.0;
            let _ = render_tab(&mut state, &mut m, &ctx, time);
            time += 1.0;
            let out = render_tab(&mut state, &mut m, &ctx, time);
            let t = texts(&out);
            assert!(!t.is_empty());
            match tab {
                TAB_PROPERTIES => {
                    for (kind, reason) in UNANALYZED_KINDS {
                        assert!(t.iter().any(|s| s == kind), "{kind}");
                        assert!(t.iter().any(|s| s.contains(reason)), "{reason}");
                    }
                    assert!(t.iter().any(|s| s == "Add New Property"));
                    assert!(t.iter().any(|s| s.contains("Symmetric")));
                    assert!(t.iter().any(|s| s.contains("beyond point E")));
                }
                TAB_ASSIGN => {
                    assert!(t.iter().any(|s| s.contains("1 frame(s) selected")));
                    assert!(t.iter().any(|s| s.contains("HINGE1 at 0.5")));
                }
                _ => {
                    assert!(t
                        .iter()
                        .any(|s| s.contains("Do not allow hinges to drop loads")));
                    assert!(t.iter().any(|s| s.contains("not analyzed")));
                }
            }
        }
        // Negative branch shown when the draft is not symmetric.
        state.tab = TAB_PROPERTIES;
        state.draft.symmetric = false;
        time += 1.0;
        let _ = render_tab(&mut state, &mut m, &ctx, time);
        time += 1.0;
        let out = render_tab(&mut state, &mut m, &ctx, time);
        assert!(texts(&out).iter().any(|s| s.contains("Negative branch")));
        // Locked: the window says so.
        m.is_locked = true;
        time += 1.0;
        let out = render_tab(&mut state, &mut m, &ctx, time);
        assert!(texts(&out)
            .iter()
            .any(|s| s.contains("The model is locked")));
    }

    fn click(
        state: &mut State,
        m: &mut SapModel,
        ctx: &egui::Context,
        label: &str,
        time: f64,
    ) -> Option<Result<String, String>> {
        let input = |t: f64| egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1400., 1400.))),
            time: Some(t),
            ..Default::default()
        };
        let _ = ctx.run(input(time), |ctx| {
            render(ctx, state, m);
        });
        let out = ctx.run(input(time + 0.1), |ctx| {
            render(ctx, state, m);
        });
        let pos = out
            .shapes
            .iter()
            .find_map(|s| match &s.shape {
                egui::epaint::Shape::Text(t) if t.galley.job.text == label => {
                    Some(t.pos + t.galley.size() * 0.5)
                }
                _ => None,
            })
            .unwrap_or_else(|| panic!("button '{label}' not visible"));
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
        let _ = ctx.run(down, |ctx| {
            render(ctx, state, m);
        });
        let mut up = input(time + 0.3);
        up.events = vec![egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed: false,
            modifiers: Default::default(),
        }];
        let mut outcome = None;
        let _ = ctx.run(up, |ctx| {
            outcome = render(ctx, state, m);
        });
        outcome
    }

    #[test]
    fn a_click_defines_a_property_and_a_click_on_a_locked_model_is_refused() {
        let ctx = egui::Context::default();
        let mut m = model();
        let mut state = State {
            open: true,
            ..State::default()
        };
        let outcome = click(&mut state, &mut m, &ctx, "Add New Property", 1.0);
        assert!(matches!(outcome, Some(Ok(_))), "{outcome:?}");
        assert!(m.hinge_props.contains_key("HINGE1"));
        assert_eq!(state.editing.as_deref(), Some("HINGE1"));
        m.is_locked = true;
        let outcome = click(&mut state, &mut m, &ctx, "Delete Property", 10.0);
        assert!(
            matches!(&outcome, Some(Err(e)) if e.contains("locked")),
            "{outcome:?}"
        );
        assert!(m.hinge_props.contains_key("HINGE1"));
    }
}
