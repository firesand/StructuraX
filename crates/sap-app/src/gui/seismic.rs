//! Mass source and automatic seismic load panels.
//!
//! The mass source (CSI Analysis Reference, "Mass Source") and the
//! 2012 IBC / ASCE 7-10 automatic lateral load (CSI Lateral Loads Manual,
//! section 2.14) are model definitions. Every command validates its input
//! before it changes the model and refuses while the model is locked.

use eframe::egui;
use sap_model::seismic::{AutoSeismicAsce710, AutoSeismicReport, SeismicDirection, SeismicPeriod};
use sap_model::{ConstraintAxis, ConstraintType, LoadPatternType, SapModel};

#[derive(Clone, Copy, PartialEq)]
enum PeriodKind {
    Approximate,
    User,
}

/// Window state. It is reset when another model is loaded.
pub(super) struct State {
    pub(super) open: bool,
    tab: usize,
    mass_pattern: String,
    mass_factor: f64,
    auto_pattern: String,
    /// Pattern the draft was last loaded from.
    loaded_for: Option<String>,
    draft: AutoSeismicAsce710,
    period_kind: PeriodKind,
    ct: f64,
    x: f64,
    user_t: f64,
    fa: f64,
    fv: f64,
    use_z_min: bool,
    use_z_max: bool,
    z_min: f64,
    z_max: f64,
    ecc_diaphragm: String,
    ecc_distance: f64,
}

impl Default for State {
    fn default() -> Self {
        Self {
            open: false,
            tab: 0,
            mass_pattern: String::new(),
            mass_factor: 1.0,
            auto_pattern: String::new(),
            loaded_for: None,
            draft: AutoSeismicAsce710::default(),
            period_kind: PeriodKind::Approximate,
            ct: 0.02,
            x: 0.75,
            user_t: 1.0,
            fa: 1.0,
            fv: 1.0,
            use_z_min: false,
            use_z_max: false,
            z_min: 0.0,
            z_max: 0.0,
            ecc_diaphragm: String::new(),
            ecc_distance: 0.0,
        }
    }
}

impl State {
    /// D125: the lengths the draft holds, in the model's present units, follow a
    /// change of units (the elevations of the story range and the eccentricity
    /// distance); ratios, periods and factors do not change.
    pub(super) fn scale_lengths(&mut self, factor: f64) {
        self.z_min *= factor;
        self.z_max *= factor;
        self.ecc_distance *= factor;
    }
}

fn locked(model: &SapModel) -> Result<(), String> {
    if model.is_locked {
        Err("The model is locked. Unlock it before changing the mass source or automatic loads.".into())
    } else {
        Ok(())
    }
}

/// Switches the element self mass and additional mass contributions.
pub(super) fn set_mass_contributions(
    model: &mut SapModel,
    element_self_mass: bool,
    additional_mass: bool,
) -> Result<String, String> {
    locked(model)?;
    model.mass_source.element_self_mass = element_self_mass;
    model.mass_source.additional_mass = additional_mass;
    Ok(format!(
        "Mass source: element self mass {}, additional mass {}.",
        if element_self_mass { "on" } else { "off" },
        if additional_mass { "on" } else { "off" }
    ))
}

/// Adds a load pattern to the mass source, or changes its multiplier.
pub(super) fn set_mass_pattern(
    model: &mut SapModel,
    pattern: &str,
    factor: f64,
) -> Result<String, String> {
    locked(model)?;
    if !model.load_patterns.contains_key(pattern) {
        return Err(format!("Load pattern '{pattern}' does not exist."));
    }
    if !factor.is_finite() {
        return Err("The multiplier must be a finite number.".into());
    }
    if model.auto_seismic.contains_key(pattern) {
        return Err(format!(
            "'{pattern}' is an automatic lateral load; its force depends on the mass, so it cannot be a mass source."
        ));
    }
    match model
        .mass_source
        .load_patterns
        .iter_mut()
        .find(|(p, _)| p == pattern)
    {
        Some(entry) => entry.1 = factor,
        None => model
            .mass_source
            .load_patterns
            .push((pattern.to_string(), factor)),
    }
    Ok(format!("Mass source includes '{pattern}' x {factor}."))
}

pub(super) fn remove_mass_pattern(model: &mut SapModel, pattern: &str) -> Result<String, String> {
    locked(model)?;
    let before = model.mass_source.load_patterns.len();
    model.mass_source.load_patterns.retain(|(p, _)| p != pattern);
    if model.mass_source.load_patterns.len() == before {
        return Err(format!("'{pattern}' is not in the mass source."));
    }
    Ok(format!("Removed '{pattern}' from the mass source."))
}

fn positive(name: &str, v: f64) -> Result<(), String> {
    if v.is_finite() && v > 0.0 {
        Ok(())
    } else {
        Err(format!("{name} must be a positive number."))
    }
}

/// Stores the automatic seismic definition of a Quake pattern.
pub(super) fn define_auto_seismic(
    model: &mut SapModel,
    pattern: &str,
    def: AutoSeismicAsce710,
) -> Result<String, String> {
    locked(model)?;
    let Some(pat) = model.load_patterns.get(pattern) else {
        return Err(format!("Load pattern '{pattern}' does not exist."));
    };
    if pat.pat_type != LoadPatternType::Quake {
        return Err(format!("'{pattern}' is not a Quake pattern."));
    }
    if model
        .mass_source
        .load_patterns
        .iter()
        .any(|(p, _)| p == pattern)
    {
        return Err(format!(
            "'{pattern}' is in the mass source; remove it there first."
        ));
    }
    match def.period {
        SeismicPeriod::Approximate { ct, x } => {
            positive("Ct", ct)?;
            positive("x", x)?;
        }
        SeismicPeriod::User { t } => positive("The period T", t)?,
        SeismicPeriod::ProgramCalculated => {
            return Err("The program-calculated period is not implemented.".into())
        }
    }
    for (name, v) in [
        ("R", def.r),
        ("Omega0", def.omega0),
        ("Cd", def.cd),
        ("Ie", def.ie),
        ("TL", def.tl),
    ] {
        positive(name, v)?;
    }
    for (name, v) in [("Ss", def.ss), ("S1", def.s1)] {
        if !v.is_finite() || v < 0.0 {
            return Err(format!("{name} must be finite and not negative."));
        }
    }
    for (name, v) in [("Fa", def.fa), ("Fv", def.fv)] {
        match v {
            Some(v) => positive(name, v)?,
            None => {
                return Err(format!(
                    "{name} from the site class is not implemented; enter it directly."
                ))
            }
        }
    }
    if !def.eccentricity_ratio.is_finite() {
        return Err("The eccentricity ratio must be finite.".into());
    }
    if let (Some(lo), Some(hi)) = (def.z_min, def.z_max) {
        if !(lo.is_finite() && hi.is_finite() && hi > lo) {
            return Err("The top elevation must be above the bottom elevation.".into());
        }
    } else if [def.z_min, def.z_max]
        .into_iter()
        .flatten()
        .any(|z| !z.is_finite())
    {
        return Err("Elevations must be finite.".into());
    }
    model.auto_seismic.insert(pattern.to_string(), def);
    Ok(format!(
        "'{pattern}' is an ASCE 7-10 automatic lateral load."
    ))
}

/// Overwrites the eccentricity of one diaphragm for an automatic lateral load
/// with a distance (Lateral Loads Manual, section 2.2.3): the eccentric moment
/// is then distance times the lateral force on the diaphragm instead of ratio
/// times width times force. The distance is in the model length unit, measured
/// perpendicular to the load.
pub(super) fn set_eccentricity_overwrite(
    model: &mut SapModel,
    pattern: &str,
    diaphragm: &str,
    distance: f64,
) -> Result<String, String> {
    locked(model)?;
    if !model.auto_seismic.contains_key(pattern) {
        return Err(format!("'{pattern}' has no automatic lateral definition."));
    }
    let Some(constraint) = model.constraints.get(diaphragm) else {
        return Err(format!("Constraint '{diaphragm}' does not exist."));
    };
    if constraint.constraint_type != ConstraintType::Diaphragm
        || constraint.axis != ConstraintAxis::Z
    {
        return Err(format!(
            "'{diaphragm}' is not a diaphragm constraint about the Z axis; eccentricities apply to those only."
        ));
    }
    if !distance.is_finite() {
        return Err("The eccentricity distance must be a finite number.".into());
    }
    model
        .auto_seismic_eccentricity
        .entry(pattern.to_string())
        .or_default()
        .insert(diaphragm.to_string(), distance);
    Ok(format!(
        "Eccentricity of diaphragm '{diaphragm}' in '{pattern}' is overwritten with {distance}."
    ))
}

/// Removes an eccentricity overwrite.
pub(super) fn remove_eccentricity_overwrite(
    model: &mut SapModel,
    pattern: &str,
    diaphragm: &str,
) -> Result<String, String> {
    locked(model)?;
    let removed = model
        .auto_seismic_eccentricity
        .get_mut(pattern)
        .and_then(|t| t.shift_remove(diaphragm))
        .is_some();
    if !removed {
        return Err(format!(
            "Diaphragm '{diaphragm}' has no eccentricity overwrite in '{pattern}'."
        ));
    }
    if model.auto_seismic_eccentricity[pattern].is_empty() {
        model.auto_seismic_eccentricity.shift_remove(pattern);
    }
    Ok(format!(
        "Eccentricity of diaphragm '{diaphragm}' in '{pattern}' is no longer overwritten."
    ))
}

/// The lines that show the eccentric moments of a computed report.
pub(super) fn moment_lines(report: &AutoSeismicReport) -> Vec<String> {
    report
        .diaphragm_moments
        .iter()
        .map(|m| {
            format!(
                "Diaphragm {}: F = {}, width = {}, e = {}{}, Mz = {} at joint {}",
                m.diaphragm,
                super::view::quantity_text(m.force),
                super::view::quantity_text(m.width),
                super::view::quantity_text(m.eccentricity),
                if m.overwritten { " (overwrite)" } else { "" },
                super::view::quantity_text(m.moment_z),
                m.joint
            )
        })
        .collect()
}

pub(super) fn remove_auto_seismic(model: &mut SapModel, pattern: &str) -> Result<String, String> {
    locked(model)?;
    if model.auto_seismic.shift_remove(pattern).is_none() {
        return Err(format!("'{pattern}' has no automatic lateral definition."));
    }
    model.auto_seismic_eccentricity.shift_remove(pattern);
    Ok(format!("'{pattern}' is no longer an automatic lateral load."))
}

/// The report of the first case that applied the pattern.
pub(super) fn first_report<'a>(
    model: &'a SapModel,
    pattern: &str,
) -> Option<(&'a str, &'a AutoSeismicReport)> {
    model.case_results.iter().find_map(|(case, r)| {
        r.auto_lateral
            .iter()
            .find(|a| a.pattern == pattern)
            .map(|a| (case.as_str(), a))
    })
}

fn sync_draft(state: &mut State, model: &SapModel) {
    if state.loaded_for.as_deref() == Some(state.auto_pattern.as_str()) {
        return;
    }
    state.loaded_for = Some(state.auto_pattern.clone());
    if let Some(def) = model.auto_seismic.get(&state.auto_pattern) {
        state.draft = def.clone();
        match def.period {
            SeismicPeriod::Approximate { ct, x } => {
                state.period_kind = PeriodKind::Approximate;
                state.ct = ct;
                state.x = x;
            }
            SeismicPeriod::User { t } => {
                state.period_kind = PeriodKind::User;
                state.user_t = t;
            }
            SeismicPeriod::ProgramCalculated => {}
        }
        state.fa = def.fa.unwrap_or(1.0);
        state.fv = def.fv.unwrap_or(1.0);
        state.use_z_min = def.z_min.is_some();
        state.use_z_max = def.z_max.is_some();
        state.z_min = def.z_min.unwrap_or(0.0);
        state.z_max = def.z_max.unwrap_or(0.0);
    }
}

fn pattern_combo(ui: &mut egui::Ui, id: &str, model: &SapModel, current: &mut String) {
    if !model.load_patterns.contains_key(current.as_str()) {
        *current = model.load_patterns.keys().next().cloned().unwrap_or_default();
    }
    egui::ComboBox::from_id_salt(id)
        .selected_text(current.as_str())
        .show_ui(ui, |ui| {
            for name in model.load_patterns.keys() {
                ui.selectable_value(current, name.clone(), name);
            }
        });
}

fn mass_tab(ui: &mut egui::Ui, state: &mut State, model: &mut SapModel) -> Option<Result<String, String>> {
    let mut outcome = None;
    ui.label("Mass = element self mass + additional mass + load patterns / g. Downward load gives positive mass; the mass acts in all three translations.");
    let (mut own, mut added) = (
        model.mass_source.element_self_mass,
        model.mass_source.additional_mass,
    );
    let changed = ui.checkbox(&mut own, "Element self mass").changed()
        | ui.checkbox(&mut added, "Additional mass (joint and area mass)").changed();
    if changed {
        outcome = Some(set_mass_contributions(model, own, added));
    }
    ui.separator();
    ui.label("Specified load patterns (multiplier of the pattern load):");
    let rows = model.mass_source.load_patterns.clone();
    for (pattern, factor) in rows {
        ui.horizontal(|ui| {
            ui.label(format!("{pattern} x {factor}"));
            if ui.button("Remove").clicked() {
                outcome = Some(remove_mass_pattern(model, &pattern));
            }
        });
    }
    ui.horizontal(|ui| {
        pattern_combo(ui, "ms_pattern", model, &mut state.mass_pattern);
        ui.add(egui::DragValue::new(&mut state.mass_factor).speed(0.05));
        if ui.button("Add to mass source").clicked() {
            outcome = Some(set_mass_pattern(model, &state.mass_pattern, state.mass_factor));
        }
    });
    if model.is_locked {
        ui.colored_label(egui::Color32::YELLOW, "The model is locked; unlock it to edit.");
    }
    outcome
}

fn auto_tab(ui: &mut egui::Ui, state: &mut State, model: &mut SapModel) -> Option<Result<String, String>> {
    let mut outcome = None;
    ui.label("Lengths are in the model length unit; Ct is always in English units (hn in feet); Ss and S1 in g; periods in seconds.");
    ui.horizontal(|ui| {
        ui.label("Quake pattern");
        pattern_combo(ui, "as_pattern", model, &mut state.auto_pattern);
    });
    sync_draft(state, model);
    let defined = model.auto_seismic.contains_key(&state.auto_pattern);
    ui.label(if defined {
        "This pattern is an automatic lateral load."
    } else {
        "This pattern has no automatic lateral definition."
    });
    ui.horizontal(|ui| {
        ui.label("Direction");
        ui.selectable_value(&mut state.draft.direction, SeismicDirection::X, "X");
        ui.selectable_value(&mut state.draft.direction, SeismicDirection::Y, "Y");
        ui.label("Eccentricity ratio");
        ui.add(egui::DragValue::new(&mut state.draft.eccentricity_ratio).speed(0.01));
    });
    ui.label("Where a Z diaphragm carries load, a moment about Z of ratio x width x force is applied (none without diaphragms). Sign: the manual does not state it; by the project's choice a positive eccentricity moves the force toward +Y for an X load and +X for a Y load, giving -e F about Z for X and +e F for Y.");
    ui.horizontal(|ui| {
        ui.label("Period");
        ui.selectable_value(&mut state.period_kind, PeriodKind::Approximate, "Approximate");
        ui.selectable_value(&mut state.period_kind, PeriodKind::User, "User defined");
        match state.period_kind {
            PeriodKind::Approximate => {
                ui.label("Ct");
                ui.add(egui::DragValue::new(&mut state.ct).speed(0.001));
                ui.label("x");
                ui.add(egui::DragValue::new(&mut state.x).speed(0.01));
            }
            PeriodKind::User => {
                ui.label("T (s)");
                ui.add(egui::DragValue::new(&mut state.user_t).speed(0.01));
            }
        }
    });
    egui::Grid::new("as_factors").show(ui, |ui| {
        for (label, v) in [
            ("R", &mut state.draft.r),
            ("Omega0", &mut state.draft.omega0),
            ("Cd", &mut state.draft.cd),
            ("Ie", &mut state.draft.ie),
        ] {
            ui.label(label);
            ui.add(egui::DragValue::new(v).speed(0.05));
            ui.end_row();
        }
        for (label, v) in [
            ("Ss (g)", &mut state.draft.ss),
            ("S1 (g)", &mut state.draft.s1),
            ("TL (s)", &mut state.draft.tl),
            ("Fa", &mut state.fa),
            ("Fv", &mut state.fv),
        ] {
            ui.label(label);
            ui.add(egui::DragValue::new(v).speed(0.05));
            ui.end_row();
        }
        ui.checkbox(&mut state.use_z_min, "Bottom elevation");
        ui.add_enabled(state.use_z_min, egui::DragValue::new(&mut state.z_min));
        ui.end_row();
        ui.checkbox(&mut state.use_z_max, "Top elevation");
        ui.add_enabled(state.use_z_max, egui::DragValue::new(&mut state.z_max));
        ui.end_row();
    });
    ui.label("Fa and Fv are entered directly; the site-class tables are not available.");
    ui.horizontal(|ui| {
        if ui.button("Set automatic lateral load").clicked() {
            let mut def = state.draft.clone();
            def.period = match state.period_kind {
                PeriodKind::Approximate => SeismicPeriod::Approximate {
                    ct: state.ct,
                    x: state.x,
                },
                PeriodKind::User => SeismicPeriod::User { t: state.user_t },
            };
            def.fa = Some(state.fa);
            def.fv = Some(state.fv);
            def.z_min = state.use_z_min.then_some(state.z_min);
            def.z_max = state.use_z_max.then_some(state.z_max);
            outcome = Some(define_auto_seismic(model, &state.auto_pattern, def));
        }
        if ui.button("Remove definition").clicked() {
            outcome = Some(remove_auto_seismic(model, &state.auto_pattern));
        }
    });
    if model.is_locked {
        ui.colored_label(egui::Color32::YELLOW, "The model is locked; unlock it to edit.");
    }
    ui.separator();
    ui.label("Eccentricity overwrite: a distance (model length unit, perpendicular to the load) instead of the ratio, per Z diaphragm.");
    let rows: Vec<(String, f64)> = model
        .auto_seismic_eccentricity
        .get(&state.auto_pattern)
        .map(|t| t.iter().map(|(d, v)| (d.clone(), *v)).collect())
        .unwrap_or_default();
    for (diaphragm, distance) in rows {
        ui.horizontal(|ui| {
            ui.label(format!("{diaphragm}: e = {distance}"));
            if ui.button("Remove overwrite").clicked() {
                outcome = Some(remove_eccentricity_overwrite(model, &state.auto_pattern, &diaphragm));
            }
        });
    }
    let diaphragms: Vec<String> = model
        .constraints
        .iter()
        .filter(|(_, c)| c.constraint_type == ConstraintType::Diaphragm && c.axis == ConstraintAxis::Z)
        .map(|(n, _)| n.clone())
        .collect();
    if diaphragms.is_empty() {
        ui.label("The model has no Z diaphragm constraint, so eccentricities do not apply.");
    } else {
        if !diaphragms.contains(&state.ecc_diaphragm) {
            state.ecc_diaphragm = diaphragms[0].clone();
        }
        ui.horizontal(|ui| {
            egui::ComboBox::from_id_salt("as_ecc_diaphragm")
                .selected_text(state.ecc_diaphragm.as_str())
                .show_ui(ui, |ui| {
                    for name in &diaphragms {
                        ui.selectable_value(&mut state.ecc_diaphragm, name.clone(), name);
                    }
                });
            ui.add(egui::DragValue::new(&mut state.ecc_distance).speed(0.05));
            if ui.button("Set overwrite").clicked() {
                outcome = Some(set_eccentricity_overwrite(
                    model,
                    &state.auto_pattern,
                    &state.ecc_diaphragm,
                    state.ecc_distance,
                ));
            }
        });
    }
    ui.separator();
    let units = model.present_units.s2k_label();
    match first_report(model, &state.auto_pattern) {
        None => {
            ui.label("Run the analysis to see the computed load.");
        }
        Some((case, r)) => {
            ui.label(format!("Computed in case '{case}' ({units}), pattern scale 1:"));
            ui.label(format!(
                "T = {:.5} s, k = {:.4}, SDS = {:.5} g, SD1 = {:.5} g",
                r.period, r.k, r.sds, r.sd1
            ));
            ui.label(format!(
                "Cs = {:.6} (Eqn. {}), W = {:.6}, V = {:.6}",
                r.cs, r.governing_equation, r.weight, r.base_shear
            ));
            for line in moment_lines(r) {
                ui.label(line);
            }
            egui::Grid::new("as_forces").show(ui, |ui| {
                for h in ["Joint", "z", "Weight", "Force"] {
                    ui.strong(h);
                }
                ui.end_row();
                for f in &r.joint_forces {
                    ui.label(&f.joint);
                    ui.label(super::view::quantity_text(f.z));
                    ui.label(super::view::quantity_text(f.weight));
                    ui.label(super::view::quantity_text(f.force));
                    ui.end_row();
                }
            });
        }
    }
    outcome
}

/// Height the window opens with: it fits a 900 px screen, and the content
/// scrolls inside it, so that every control is reachable.
const WINDOW_HEIGHT: f32 = 600.0;

/// Draws the window when it is open. Returns the outcome of a command the
/// user ran in this frame: a status message, or why it was refused.
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
    egui::Window::new("Mass Source and Auto Seismic Loads")
        .open(&mut open)
        .default_size([560.0, WINDOW_HEIGHT])
        .max_height((ctx.screen_rect().height() - 60.0).max(200.0))
        .vscroll(true)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.selectable_value(&mut state.tab, 0, "Mass source");
                ui.selectable_value(&mut state.tab, 1, "Auto seismic (ASCE 7-10)");
            });
            ui.separator();
            outcome = if state.tab == 0 {
                mass_tab(ui, state, model)
            } else {
                auto_tab(ui, state, model)
            };
        });
    state.open = open;
    outcome
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_change_of_units_rescales_the_lengths_of_the_draft() {
        // D125: 120 in = 3.048 m (120 * 0.0254); 240 in = 6.096 m; 36 in = 0.9144 m.
        let mut app = crate::gui::SapGuiApp::new(Some(std::path::PathBuf::from(
            "/nonexistent/sap-review-model.s2k",
        )));
        app.model = SapModel::new(Units::KipInF);
        app.panel_seismic.z_min = 120.0;
        app.panel_seismic.z_max = 240.0;
        app.panel_seismic.ecc_distance = 36.0;
        app.panel_seismic.fa = 1.2;
        app.change_units(Units::KnMC);
        assert!((app.panel_seismic.z_min - 3.048).abs() < 1e-12, "{}", app.panel_seismic.z_min);
        assert!((app.panel_seismic.z_max - 6.096).abs() < 1e-12);
        assert!((app.panel_seismic.ecc_distance - 0.9144).abs() < 1e-12);
        // Negative: a factor is not a length.
        assert_eq!(app.panel_seismic.fa, 1.2);
    }
    use eframe::egui::{Pos2, Rect, Vec2};
    use sap_model::LoadPattern;
    use sap_units::Units;

    fn model() -> SapModel {
        let mut m = SapModel::new(Units::KnMC);
        for (name, kind) in [
            ("LIVE", LoadPatternType::Live),
            ("EQX", LoadPatternType::Quake),
        ] {
            m.load_patterns.insert(
                name.into(),
                LoadPattern {
                    name: name.into(),
                    pat_type: kind,
                    self_wt_multiplier: 0.0,
                },
            );
        }
        m
    }

    fn definition() -> AutoSeismicAsce710 {
        AutoSeismicAsce710 {
            fa: Some(1.0),
            fv: Some(1.0),
            ..Default::default()
        }
    }

    #[test]
    fn mass_source_commands_validate_before_changing() {
        let mut m = model();
        assert!(set_mass_pattern(&mut m, "NOPE", 1.0).is_err());
        assert!(set_mass_pattern(&mut m, "LIVE", f64::NAN).is_err());
        assert!(m.mass_source.is_default());
        set_mass_pattern(&mut m, "LIVE", 0.25).unwrap();
        set_mass_pattern(&mut m, "LIVE", 0.5).unwrap();
        assert_eq!(m.mass_source.load_patterns, vec![("LIVE".to_string(), 0.5)]);
        set_mass_contributions(&mut m, false, true).unwrap();
        assert!(!m.mass_source.element_self_mass && m.mass_source.additional_mass);
        assert!(remove_mass_pattern(&mut m, "OTHER").is_err());
        remove_mass_pattern(&mut m, "LIVE").unwrap();
        assert!(m.mass_source.load_patterns.is_empty());
        // An automatic lateral pattern cannot be a mass source, and vice versa.
        define_auto_seismic(&mut m, "EQX", definition()).unwrap();
        assert!(set_mass_pattern(&mut m, "EQX", 1.0).is_err());
    }

    #[test]
    fn auto_seismic_commands_validate_before_changing() {
        let mut m = model();
        assert!(define_auto_seismic(&mut m, "LIVE", definition()).is_err());
        assert!(define_auto_seismic(&mut m, "GHOST", definition()).is_err());
        for edit in [
            (|d: &mut AutoSeismicAsce710| d.r = 0.0) as fn(&mut AutoSeismicAsce710),
            |d| d.ie = -1.0,
            |d| d.tl = 0.0,
            |d| d.fa = None,
            |d| d.period = SeismicPeriod::ProgramCalculated,
            |d| d.period = SeismicPeriod::User { t: 0.0 },
            |d| {
                d.z_min = Some(5.0);
                d.z_max = Some(1.0)
            },
        ] {
            let mut d = definition();
            edit(&mut d);
            assert!(define_auto_seismic(&mut m, "EQX", d).is_err());
            assert!(m.auto_seismic.is_empty());
        }
        define_auto_seismic(&mut m, "EQX", definition()).unwrap();
        assert!(m.auto_seismic.contains_key("EQX"));
        // A pattern in the mass source cannot become automatic.
        let mut other = model();
        set_mass_pattern(&mut other, "EQX", 1.0).unwrap();
        assert!(define_auto_seismic(&mut other, "EQX", definition()).is_err());
        remove_auto_seismic(&mut m, "EQX").unwrap();
        assert!(remove_auto_seismic(&mut m, "EQX").is_err());
    }

    #[test]
    fn eccentricity_overwrite_commands_validate_and_clean_up() {
        use sap_model::ConstraintDef;
        let mut m = model();
        let make = |name: &str, kind, axis| ConstraintDef {
            name: name.into(),
            constraint_type: kind,
            axis,
            dofs: [true, true, false, false, false, true],
        };
        m.constraints.insert("D1".into(), make("D1", ConstraintType::Diaphragm, ConstraintAxis::Z));
        m.constraints.insert("D2".into(), make("D2", ConstraintType::Diaphragm, ConstraintAxis::X));
        m.constraints.insert("B1".into(), make("B1", ConstraintType::Body, ConstraintAxis::Z));
        // No automatic definition yet.
        assert!(set_eccentricity_overwrite(&mut m, "EQX", "D1", 0.4).is_err());
        define_auto_seismic(&mut m, "EQX", definition()).unwrap();
        for (name, distance) in [("GHOST", 1.0), ("D2", 1.0), ("B1", 1.0), ("D1", f64::NAN)] {
            assert!(set_eccentricity_overwrite(&mut m, "EQX", name, distance).is_err(), "{name}");
            assert!(m.auto_seismic_eccentricity.is_empty());
        }
        set_eccentricity_overwrite(&mut m, "EQX", "D1", 0.4).unwrap();
        assert_eq!(m.auto_seismic_eccentricity["EQX"]["D1"], 0.4);
        assert!(remove_eccentricity_overwrite(&mut m, "EQX", "D2").is_err());
        remove_eccentricity_overwrite(&mut m, "EQX", "D1").unwrap();
        assert!(m.auto_seismic_eccentricity.is_empty());
        // Removing the definition removes its overwrites; a locked model refuses.
        set_eccentricity_overwrite(&mut m, "EQX", "D1", 0.4).unwrap();
        m.is_locked = true;
        assert!(set_eccentricity_overwrite(&mut m, "EQX", "D1", 0.5).is_err());
        assert!(remove_eccentricity_overwrite(&mut m, "EQX", "D1").is_err());
        m.is_locked = false;
        remove_auto_seismic(&mut m, "EQX").unwrap();
        assert!(m.auto_seismic_eccentricity.is_empty());
    }

    #[test]
    fn moment_lines_show_every_number_of_the_report() {
        use sap_model::seismic::AutoSeismicDiaphragmMoment;
        let report = AutoSeismicReport {
            diaphragm_moments: vec![AutoSeismicDiaphragmMoment {
                diaphragm: "D1".into(),
                force: 100.0,
                width: 6.0,
                eccentricity: 0.3,
                overwritten: true,
                moment_z: -30.0,
                joint: "2".into(),
            }],
            ..Default::default()
        };
        assert_eq!(
            moment_lines(&report),
            vec!["Diaphragm D1: F = 100, width = 6, e = 0.3 (overwrite), Mz = -30 at joint 2"]
        );
        assert!(moment_lines(&AutoSeismicReport::default()).is_empty());
    }

    #[test]
    fn commands_refuse_while_locked() {
        let mut m = model();
        m.is_locked = true;
        assert!(set_mass_contributions(&mut m, false, false).is_err());
        assert!(set_mass_pattern(&mut m, "LIVE", 1.0).is_err());
        assert!(define_auto_seismic(&mut m, "EQX", definition()).is_err());
        assert!(m.mass_source.is_default() && m.auto_seismic.is_empty());
    }

    fn button_center(out: &egui::FullOutput, label: &str) -> Option<Pos2> {
        out.shapes.iter().find_map(|shape| {
            if let egui::epaint::Shape::Text(text) = &shape.shape {
                (text.galley.job.text == label).then(|| text.pos + text.galley.size() * 0.5)
            } else {
                None
            }
        })
    }

    #[test]
    fn window_renders_both_tabs_and_a_click_changes_the_model() {
        let mut state = State {
            open: true,
            ..State::default()
        };
        let mut m = model();
        let ctx = egui::Context::default();
        let input = |time: f64| egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1400., 1200.))),
            time: Some(time),
            ..Default::default()
        };
        // Both tabs draw.
        for (i, tab) in [1usize, 0].into_iter().enumerate() {
            state.tab = tab;
            let out = ctx.run(input(1.0 + i as f64), |ctx| {
                render(ctx, &mut state, &mut m);
            });
            assert!(!out.shapes.is_empty());
        }
        // Click "Add to mass source" on the first tab.
        let _ = ctx.run(input(3.0), |ctx| {
            render(ctx, &mut state, &mut m);
        });
        let out = ctx.run(input(4.0), |ctx| {
            render(ctx, &mut state, &mut m);
        });
        let pos = button_center(&out, "Add to mass source").expect("button visible");
        let mut down = input(5.0);
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
            render(ctx, &mut state, &mut m);
        });
        let mut up = input(5.1);
        up.events = vec![egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed: false,
            modifiers: Default::default(),
        }];
        let mut outcome = None;
        let _ = ctx.run(up, |ctx| {
            outcome = render(ctx, &mut state, &mut m);
        });
        assert!(matches!(outcome, Some(Ok(_))), "{outcome:?}");
        assert_eq!(m.mass_source.load_patterns.len(), 1);
    }
}
