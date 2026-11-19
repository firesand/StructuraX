//! Display > Show Hinge Results: a read-only window for one hinge of a
//! nonlinear static case that saved multiple states.
//!
//! Source: v19 help topic "Show Hinge Results" (fields: Select Hinge, Show
//! Hinge on Selected Frame Only, Frame Object, Relative Distance, Hinge
//! Behavior, Units, Select Load Case and Step, Hinge DOF, Hinge State, Hinge
//! Status, plot control, Horiz and Vert boxes) and CSI Analysis Reference,
//! "Frame Hinge Properties". Only `static_steps[*].hinges` and
//! `hinge_props` are read; the model is never changed.
//!
//! The drawn backbone is the curve as entered: B to E of both branches, table
//! values times the scale factors, point B moved to zero plastic deformation
//! (only the deformation beyond B is used). The path is the force against the
//! plastic deformation of the saved states 0 to the step. The solver limits
//! the negative slope of a bending hinge where strength is lost (v19
//! Analysis Reference p. 152, "Strength Loss"), so the path leaves the drawn
//! backbone there; the window says so.

use super::display_units::{self, Dimension};
use super::pushover::{nice_ticks, tick_text};
use eframe::egui;
use sap_model::hinge::{HingeBehavior, HingeDof, HingeProp, HingeState, HingeStatus};
use sap_model::{LoadCaseType, SapModel};
use sap_units::Units;

/// Window state. It is reset when another model is loaded.
pub(super) struct State {
    pub(super) open: bool,
    /// Generated name of the hinge shown (`{frame}H{n}`); empty or unknown
    /// selects the first listed hinge.
    pub(super) hinge: String,
    /// "Show Hinge on Selected Frame Only".
    pub(super) only_selected: bool,
    /// Case shown; empty or unknown selects the first nonlinear static case
    /// with saved states.
    pub(super) case: String,
    /// Index into the saved states of the case.
    pub(super) step: usize,
    /// Display units; `None` shows the units of the model.
    pub(super) units: Option<Units>,
    pub(super) show_backbone: bool,
    pub(super) show_path: bool,
    pub(super) backbone_color: egui::Color32,
}

impl Default for State {
    fn default() -> Self {
        Self {
            open: false,
            hinge: String::new(),
            only_selected: false,
            case: String::new(),
            step: 0,
            units: None,
            show_backbone: true,
            show_path: true,
            backbone_color: egui::Color32::from_rgb(255, 170, 60),
        }
    }
}

impl State {
    /// The Display menu command: opens the window, with the selected-frame
    /// filter on when frames are selected (v19 help).
    pub(super) fn show(&mut self, model: &SapModel) {
        self.open = true;
        self.only_selected = model.frames.values().any(|f| f.selected);
    }
}

/// Everything the window shows for one hinge at one saved state, in model
/// units.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct HingeData {
    pub name: String,
    pub frame: String,
    pub rel_dist: f64,
    pub prop: String,
    pub dof: HingeDof,
    /// `None` when the property no longer exists.
    pub behavior: Option<HingeBehavior>,
    /// Number of the saved state shown (`NonlinearStaticStep::step`).
    pub step_number: usize,
    pub force: f64,
    pub plastic: f64,
    pub max_force: f64,
    pub min_force: f64,
    pub max_plastic: f64,
    pub min_plastic: f64,
    pub state: HingeState,
    pub status: HingeStatus,
    pub acceptance_given: bool,
    /// (plastic deformation, force) of the saved states 0 to the step.
    pub path: Vec<(f64, f64)>,
    /// The entered curve as (plastic deformation, force), from -E to -B and
    /// from B to E; empty without the property.
    pub backbone: Vec<(f64, f64)>,
}

/// Nonlinear static cases with at least one saved state, in model order.
pub(super) fn cases_with_states(model: &SapModel) -> Vec<String> {
    model
        .load_cases
        .iter()
        .filter(|(name, lc)| {
            lc.case_type == LoadCaseType::NonlinearStatic
                && model
                    .case_results
                    .get(*name)
                    .is_some_and(|r| !r.static_steps.is_empty())
        })
        .map(|(name, _)| name.clone())
        .collect()
}

/// Generated names of the hinges of a case, in the order of the results; with
/// `only_selected` only those of selected frames.
pub(super) fn hinge_names(model: &SapModel, case: &str, only_selected: bool) -> Vec<String> {
    let Some(step) = model
        .case_results
        .get(case)
        .and_then(|r| r.static_steps.last())
    else {
        return Vec::new();
    };
    step.hinges
        .iter()
        .filter(|h| !only_selected || model.frames.get(&h.frame).is_some_and(|f| f.selected))
        .map(|h| h.name.clone())
        .collect()
}

/// The drawn backbone of a property: the entered B, C, D, E of the negative
/// branch (mirror of the positive one when symmetric) and of the positive
/// branch, table values times the scale factors, point B at zero plastic
/// deformation. Forces and deformations carry the sign of their branch.
pub(super) fn backbone_points(prop: &HingeProp) -> Vec<(f64, f64)> {
    let branch = |points: &[sap_model::hinge::HingePoint; 4], fsf: f64, dsf: f64, sign: f64| {
        let d_b = points[0].deformation.abs() * dsf;
        points
            .iter()
            .map(|p| (sign * (p.deformation.abs() * dsf - d_b), sign * p.force.abs() * fsf))
            .collect::<Vec<_>>()
    };
    let positive = branch(&prop.positive, prop.force_sf, prop.deformation_sf, 1.0);
    let mut negative = if prop.symmetric {
        branch(&prop.positive, prop.force_sf, prop.deformation_sf, -1.0)
    } else {
        branch(
            &prop.negative,
            prop.force_sf_negative,
            prop.deformation_sf_negative,
            -1.0,
        )
    };
    negative.reverse();
    negative.extend(positive);
    negative
}

/// The data of hinge `name` at saved state `step` (an index; past the last
/// state selects the last one) of `case`.
pub(super) fn hinge_data(
    model: &SapModel,
    case: &str,
    name: &str,
    step: usize,
) -> Result<HingeData, String> {
    let results = model
        .case_results
        .get(case)
        .filter(|r| !r.static_steps.is_empty())
        .ok_or_else(|| format!("case '{case}' has no saved states"))?;
    let k = step.min(results.static_steps.len() - 1);
    let find = |i: usize| {
        results.static_steps[i]
            .hinges
            .iter()
            .find(|h| h.name == name)
    };
    let now = find(k).ok_or_else(|| format!("case '{case}' has no hinge '{name}' at saved state {k}"))?;
    let mut data = HingeData {
        name: name.to_string(),
        frame: now.frame.clone(),
        rel_dist: now.rel_dist,
        prop: now.prop.clone(),
        dof: now.dof,
        behavior: None,
        step_number: results.static_steps[k].step,
        force: now.force,
        plastic: now.plastic_deformation,
        max_force: f64::NEG_INFINITY,
        min_force: f64::INFINITY,
        max_plastic: f64::NEG_INFINITY,
        min_plastic: f64::INFINITY,
        state: now.state,
        status: now.status,
        acceptance_given: false,
        path: Vec::new(),
        backbone: Vec::new(),
    };
    for i in 0..=k {
        if let Some(h) = find(i) {
            data.path.push((h.plastic_deformation, h.force));
            data.max_force = data.max_force.max(h.force);
            data.min_force = data.min_force.min(h.force);
            data.max_plastic = data.max_plastic.max(h.plastic_deformation);
            data.min_plastic = data.min_plastic.min(h.plastic_deformation);
        }
    }
    if let Some(prop) = model.hinge_props.get(&now.prop) {
        data.behavior = Some(prop.behavior);
        data.acceptance_given = prop.acceptance_positive.is_given()
            || (!prop.symmetric && prop.acceptance_negative.is_given());
        data.backbone = backbone_points(prop);
    }
    Ok(data)
}

fn state_text(state: HingeState) -> &'static str {
    match state {
        HingeState::AToB => "A to B",
        HingeState::BToC => "B to C",
        HingeState::CToD => "C to D",
        HingeState::DToE => "D to E",
        HingeState::BeyondE => ">E",
    }
}

fn status_text(status: HingeStatus) -> &'static str {
    match status {
        HingeStatus::AToB => "A to B",
        HingeStatus::BToIO => "B to IO",
        HingeStatus::IOToLS => "IO to LS",
        HingeStatus::LSToCP => "LS to CP",
        HingeStatus::BeyondCP => ">CP",
    }
}

/// The single line the window prints about the slope limit.
pub(super) const STRENGTH_LOSS_NOTE: &str = "Where a bending hinge loses strength the solver limits the negative slope (v19 Analysis Reference p. 152, Strength Loss), so the path leaves the drawn backbone there.";

fn dimensions(dof: HingeDof) -> (Dimension, Dimension) {
    if dof.is_rotation() {
        (Dimension::Moment, Dimension::Rotation)
    } else {
        (Dimension::Force, Dimension::Length)
    }
}

struct Series<'a> {
    xy: &'a [(f64, f64)],
    color: egui::Color32,
    dots: bool,
}

/// Draws the series, the axes through zero and the current point; returns
/// the data coordinates under the pointer.
fn draw_plot(
    ui: &mut egui::Ui,
    series: &[Series],
    current: Option<(f64, f64)>,
    x_label: &str,
    y_label: &str,
) -> Option<(f64, f64)> {
    let size = egui::vec2(ui.available_width().clamp(300.0, 720.0), 240.0);
    let (rect, response) = ui.allocate_exact_size(size, egui::Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(16, 20, 28));
    let finite = |xy: &[(f64, f64)]| -> Vec<(f64, f64)> {
        xy.iter()
            .copied()
            .filter(|(x, y)| x.is_finite() && y.is_finite())
            .collect()
    };
    let all: Vec<(f64, f64)> = series.iter().flat_map(|s| finite(s.xy)).collect();
    if all.is_empty() {
        return None;
    }
    let (mut x0, mut x1, mut y0, mut y1) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
    for (x, y) in &all {
        x0 = x0.min(*x);
        x1 = x1.max(*x);
        y0 = y0.min(*y);
        y1 = y1.max(*y);
    }
    if x1 - x0 <= 0.0 {
        x1 = x0 + 1.0;
    }
    if y1 - y0 <= 0.0 {
        y1 = y0 + 1.0;
    }
    let inner = egui::Rect::from_min_max(
        rect.min + egui::vec2(66.0, 24.0),
        rect.max - egui::vec2(18.0, 38.0),
    );
    let to_screen = |(x, y): (f64, f64)| {
        egui::pos2(
            inner.left() + ((x - x0) / (x1 - x0)) as f32 * inner.width(),
            inner.bottom() - ((y - y0) / (y1 - y0)) as f32 * inner.height(),
        )
    };
    let text_color = egui::Color32::from_gray(200);
    let font = egui::FontId::monospace(11.0);
    let grid = egui::Stroke::new(1.0_f32, egui::Color32::from_gray(40));
    for v in nice_ticks(x0, x1, 5) {
        let x = to_screen((v, 0.0)).x;
        painter.line_segment([egui::pos2(x, inner.top()), egui::pos2(x, inner.bottom())], grid);
        painter.text(
            egui::pos2(x, inner.bottom() + 3.0),
            egui::Align2::CENTER_TOP,
            tick_text(v),
            font.clone(),
            text_color,
        );
    }
    for v in nice_ticks(y0, y1, 5) {
        let y = to_screen((0.0, v)).y;
        painter.line_segment([egui::pos2(inner.left(), y), egui::pos2(inner.right(), y)], grid);
        painter.text(
            egui::pos2(inner.left() - 4.0, y),
            egui::Align2::RIGHT_CENTER,
            tick_text(v),
            font.clone(),
            text_color,
        );
    }
    let axis = egui::Stroke::new(1.0_f32, egui::Color32::from_gray(110));
    let origin = to_screen((0.0, 0.0));
    painter.line_segment(
        [egui::pos2(inner.left(), origin.y), egui::pos2(inner.right(), origin.y)],
        axis,
    );
    painter.line_segment(
        [egui::pos2(origin.x, inner.top()), egui::pos2(origin.x, inner.bottom())],
        axis,
    );
    for s in series {
        let line: Vec<egui::Pos2> = finite(s.xy).into_iter().map(to_screen).collect();
        if line.len() >= 2 {
            painter.add(egui::Shape::line(line.clone(), egui::Stroke::new(1.5_f32, s.color)));
        }
        if s.dots {
            for p in line {
                painter.circle_filled(p, 2.5, s.color);
            }
        }
    }
    if let Some(c) = current.filter(|(x, y)| x.is_finite() && y.is_finite()) {
        let p = to_screen(c);
        painter.circle_stroke(p, 6.0, egui::Stroke::new(2.0_f32, egui::Color32::from_rgb(255, 230, 120)));
    }
    painter.text(
        egui::pos2(inner.center().x, rect.bottom() - 4.0),
        egui::Align2::CENTER_BOTTOM,
        x_label,
        font.clone(),
        text_color,
    );
    painter.text(
        egui::pos2(rect.left() + 6.0, rect.top() + 4.0),
        egui::Align2::LEFT_TOP,
        y_label,
        font,
        text_color,
    );
    response.hover_pos().filter(|p| inner.contains(*p)).map(|p| {
        (
            x0 + f64::from((p.x - inner.left()) / inner.width()) * (x1 - x0),
            y0 + f64::from((inner.bottom() - p.y) / inner.height()) * (y1 - y0),
        )
    })
}

fn units_combo(ui: &mut egui::Ui, units: &mut Units) {
    egui::ComboBox::from_id_salt("hinge_results_units")
        .selected_text(units.s2k_label())
        .show_ui(ui, |ui| {
            for u in Units::ALL {
                ui.selectable_value(units, u, u.s2k_label());
            }
        });
}

fn property_definition(ui: &mut egui::Ui, prop: &HingeProp) {
    ui.label(format!(
        "{} ({}), {}. Table values; scale factors force {}, deformation {}{}. Read only.",
        prop.name,
        prop.dof.label(),
        if prop.symmetric { "symmetric" } else { "not symmetric" },
        tick_text(prop.force_sf),
        tick_text(prop.deformation_sf),
        if prop.symmetric {
            String::new()
        } else {
            format!(
                " (negative: {}, {})",
                tick_text(prop.force_sf_negative),
                tick_text(prop.deformation_sf_negative)
            )
        }
    ));
    egui::Grid::new("hinge_results_prop_grid").striped(true).show(ui, |ui| {
        for h in ["Point", "Force", "Deformation", "Force (-)", "Deformation (-)"] {
            ui.strong(h);
        }
        ui.end_row();
        for (i, label) in ["B", "C", "D", "E"].iter().enumerate() {
            ui.label(*label);
            ui.label(tick_text(prop.positive[i].force));
            ui.label(tick_text(prop.positive[i].deformation));
            let neg = if prop.symmetric {
                sap_model::hinge::HingePoint::new(-prop.positive[i].force, -prop.positive[i].deformation)
            } else {
                prop.negative[i]
            };
            ui.label(tick_text(neg.force));
            ui.label(tick_text(neg.deformation));
            ui.end_row();
        }
    });
    if prop.acceptance_positive.is_given() {
        ui.label(format!(
            "Acceptance criteria (deformation): IO {}, LS {}, CP {}",
            tick_text(prop.acceptance_positive.io),
            tick_text(prop.acceptance_positive.ls),
            tick_text(prop.acceptance_positive.cp)
        ));
    }
}

/// The content of the window.
fn contents(ui: &mut egui::Ui, state: &mut State, model: &SapModel) {
    let cases = cases_with_states(model);
    if !cases.contains(&state.case) {
        state.case = cases.first().cloned().unwrap_or_default();
    }
    if cases.is_empty() {
        ui.label("No nonlinear static results. Assign hinges, define a nonlinear static case whose Results Saved type is Multiple States, and run the analysis first.");
        for (name, reason) in sap_model::results::could_not_start_cases(model) {
            ui.colored_label(
                egui::Color32::from_rgb(255, 170, 90),
                format!("Case '{name}': {reason}"),
            );
        }
        return;
    }
    let display = state.units.unwrap_or(model.present_units);
    ui.horizontal(|ui| {
        ui.label("Load case:");
        egui::ComboBox::from_id_salt("hinge_results_case")
            .selected_text(state.case.as_str())
            .show_ui(ui, |ui| {
                for name in &cases {
                    ui.selectable_value(&mut state.case, name.clone(), name);
                }
            });
        ui.label("Units:");
        let mut units = display;
        units_combo(ui, &mut units);
        if units != display {
            state.units = Some(units);
        }
    });
    // G16: the states of a case that stopped before its target are the ones it reached.
    if let Some((reason, kept)) = sap_api::hinges::pushover_not_finished(model, &state.case) {
        ui.colored_label(
            egui::Color32::from_rgb(255, 170, 90),
            format!("Not finished: {reason} ({kept} saved steps kept)"),
        );
    }
    let steps = model.case_results[&state.case].static_steps.len();
    state.step = state.step.min(steps - 1);
    ui.horizontal(|ui| {
        ui.label("Step:");
        if ui.add_enabled(state.step > 0, egui::Button::new("<")).clicked() {
            state.step -= 1;
        }
        ui.add(egui::Slider::new(&mut state.step, 0..=steps - 1).integer());
        if ui.add_enabled(state.step + 1 < steps, egui::Button::new(">")).clicked() {
            state.step += 1;
        }
        ui.label(format!("of {} saved states (0 to {})", steps, steps - 1));
    });
    ui.horizontal(|ui| {
        ui.checkbox(&mut state.only_selected, "Show hinge on selected frame only");
    });
    let names = hinge_names(model, &state.case, state.only_selected);
    if names.is_empty() {
        ui.label(if state.only_selected {
            "No hinge on the selected frames in this case. Clear the check box to list all hinges."
        } else {
            "This case has no hinges."
        });
        return;
    }
    if !names.contains(&state.hinge) {
        state.hinge = names[0].clone();
    }
    ui.horizontal(|ui| {
        ui.label("Select hinge:");
        egui::ComboBox::from_id_salt("hinge_results_hinge")
            .selected_text(state.hinge.as_str())
            .show_ui(ui, |ui| {
                for name in &names {
                    ui.selectable_value(&mut state.hinge, name.clone(), name);
                }
            });
    });
    let data = match hinge_data(model, &state.case, &state.hinge, state.step) {
        Ok(data) => data,
        Err(error) => {
            ui.colored_label(egui::Color32::from_rgb(255, 120, 120), error);
            return;
        }
    };
    if let Some(prop) = model.hinge_props.get(&data.prop) {
        egui::CollapsingHeader::new("Hinge property definition (read only)")
            .id_salt("hinge_results_prop")
            .show(ui, |ui| property_definition(ui, prop));
    }
    let (force_dim, def_dim) = dimensions(data.dof);
    let ff = display_units::factor(force_dim, model.present_units, display);
    let df = display_units::factor(def_dim, model.present_units, display);
    let (force_unit, def_unit) = (
        display_units::label(force_dim, display),
        display_units::label(def_dim, display),
    );
    let num = |v: f64| super::view::significant_text(v, 8);
    let (force_name, def_name) = if data.dof.is_rotation() {
        ("Moment", "Plastic rotation")
    } else {
        ("Force", "Plastic displacement")
    };
    ui.separator();
    ui.strong("Hinge location and behavior");
    egui::Grid::new("hinge_results_location").show(ui, |ui| {
        ui.label("Frame object");
        ui.label(&data.frame);
        ui.end_row();
        ui.label("Relative distance");
        ui.label(super::view::number_text(data.rel_dist));
        ui.end_row();
        ui.label("Hinge behavior");
        ui.label(match data.behavior {
            Some(HingeBehavior::DeformationControlled) => "Deformation controlled",
            Some(HingeBehavior::ForceControlled) => "Force controlled",
            None => "property not found",
        });
        ui.end_row();
        ui.label("Hinge property");
        ui.label(&data.prop);
        ui.end_row();
    });
    ui.separator();
    ui.strong(format!("Current hinge data (state {})", data.step_number));
    ui.horizontal(|ui| {
        ui.label("Hinge DOF:");
        // An uncoupled hinge acts in one degree of freedom.
        let mut dof = data.dof;
        egui::ComboBox::from_id_salt("hinge_results_dof")
            .selected_text(data.dof.label())
            .show_ui(ui, |ui| {
                ui.selectable_value(&mut dof, data.dof, data.dof.label());
            });
        ui.label("(one degree of freedom per hinge)");
    });
    egui::Grid::new("hinge_results_values").striped(true).show(ui, |ui| {
        ui.strong("");
        ui.strong(format!("{force_name} [{force_unit}]"));
        ui.strong(format!("{def_name} [{def_unit}]"));
        ui.end_row();
        ui.label("Current");
        ui.label(num(data.force * ff));
        ui.label(num(data.plastic * df));
        ui.end_row();
        ui.label("Maximum to this step");
        ui.label(num(data.max_force * ff));
        ui.label(num(data.max_plastic * df));
        ui.end_row();
        ui.label("Minimum to this step");
        ui.label(num(data.min_force * ff));
        ui.label(num(data.min_plastic * df));
        ui.end_row();
    });
    egui::Grid::new("hinge_results_state").show(ui, |ui| {
        ui.label("Hinge state");
        ui.label(state_text(data.state));
        ui.end_row();
        ui.label("Hinge status");
        ui.label(if data.acceptance_given {
            status_text(data.status).to_string()
        } else {
            format!("{} (no acceptance criteria given)", status_text(data.status))
        });
        ui.end_row();
    });
    ui.separator();
    ui.strong("Plot control parameters");
    ui.horizontal(|ui| {
        ui.checkbox(&mut state.show_backbone, "Backbone");
        ui.color_edit_button_srgba(&mut state.backbone_color);
        ui.checkbox(&mut state.show_path, "Path of the saved states");
    });
    let path: Vec<(f64, f64)> = data.path.iter().map(|(q, f)| (q * df, f * ff)).collect();
    let backbone: Vec<(f64, f64)> =
        data.backbone.iter().map(|(q, f)| (q * df, f * ff)).collect();
    let mut series = Vec::new();
    if state.show_backbone {
        series.push(Series {
            xy: &backbone,
            color: state.backbone_color,
            dots: false,
        });
    }
    if state.show_path {
        series.push(Series {
            xy: &path,
            color: egui::Color32::from_rgb(110, 200, 255),
            dots: true,
        });
    }
    ui.label("Backbone as entered, B to E of both branches, point B at zero plastic deformation. The ring marks the current step.");
    ui.label(STRENGTH_LOSS_NOTE);
    let hover = draw_plot(
        ui,
        &series,
        Some((data.plastic * df, data.force * ff)),
        &format!("{} [{def_unit}]", def_name.to_lowercase()),
        &format!("{} [{force_unit}]", force_name.to_lowercase()),
    );
    ui.label(match hover {
        Some((x, y)) => format!("Horiz: {}   Vert: {}", num(x), num(y)),
        None => "Horiz and Vert: move the pointer over the plot.".to_string(),
    });
}

pub(super) fn render(
    ctx: &egui::Context,
    state: &mut State,
    model: &SapModel,
) -> Option<Result<String, String>> {
    if !state.open {
        return None;
    }
    let mut open = true;
    egui::Window::new("Hinge Results")
        .open(&mut open)
        .default_width(760.0)
        .default_height(900.0)
        .vscroll(true)
        .show(ctx, |ui| contents(ui, state, model));
    state.open = open;
    None
}

#[cfg(test)]
mod tests {
    use super::super::pushover::example_1_026_model;
    use super::*;

    fn analyzed() -> SapModel {
        let mut m = example_1_026_model();
        sap_solver::analyze_model(&mut m).unwrap();
        m
    }

    /// G16: the window says that the case stopped before its target.
    #[test]
    fn a_case_that_is_not_finished_is_marked_in_the_window() {
        let ctx = egui::Context::default();
        let mut state = State::default();
        state.open = true;
        let finished = analyzed();
        let t = texts(&frames(&ctx, &mut state, &finished));
        assert!(!t.iter().any(|s| s.starts_with("Not finished")), "{t:?}");
        let mut model = example_1_026_model();
        model.load_cases.get_mut("PUSH").unwrap().nl_static.max_total_steps = 5;
        sap_solver::analyze_model(&mut model).unwrap();
        let mut state = State::default();
        state.open = true;
        let t = texts(&frames(&ctx, &mut state, &model));
        assert!(
            t.iter().any(|s| s.starts_with("Not finished: ") && s.contains("total steps")),
            "{t:?}"
        );
    }

    fn texts(out: &egui::FullOutput) -> Vec<String> {
        out.shapes
            .iter()
            .filter_map(|shape| match &shape.shape {
                egui::epaint::Shape::Text(text) => Some(text.galley.job.text.clone()),
                _ => None,
            })
            .collect()
    }

    fn input(time: f64) -> egui::RawInput {
        egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(1200.0, 2600.0),
            )),
            time: Some(time),
            ..Default::default()
        }
    }

    /// The contents in a panel as tall as the screen (the window of the
    /// program scrolls, and a test cannot scroll it); a closed window draws
    /// nothing.
    fn frames(ctx: &egui::Context, state: &mut State, model: &SapModel) -> egui::FullOutput {
        let mut out = egui::FullOutput::default();
        for t in [0.0, 1.0, 2.0] {
            out = ctx.run(input(t), |ctx| {
                if state.open {
                    egui::CentralPanel::default().show(ctx, |ui| contents(ui, state, model));
                }
            });
        }
        out
    }

    #[test]
    fn the_window_itself_opens_with_its_title_and_closes() {
        let model = analyzed();
        let mut state = State::default();
        let ctx = egui::Context::default();
        let run = |state: &mut State| {
            let mut out = egui::FullOutput::default();
            for t in [0.0, 1.0, 2.0] {
                out = ctx.run(input(t), |ctx| {
                    assert!(render(ctx, state, &model).is_none());
                });
            }
            texts(&out)
        };
        assert!(run(&mut state).is_empty());
        state.open = true;
        let t = run(&mut state);
        assert!(t.iter().any(|s| s == "Hinge Results"), "{t:?}");
        assert!(t.iter().any(|s| s == "Load case:"), "{t:?}");
    }

    #[test]
    fn the_path_equals_the_stored_hinge_results_of_every_saved_state() {
        let m = analyzed();
        let steps = &m.case_results["PUSH"].static_steps;
        assert_eq!(steps.len(), 10);
        for name in ["1H1", "1H2"] {
            for k in 0..steps.len() {
                let d = hinge_data(&m, "PUSH", name, k).unwrap();
                assert_eq!(d.path.len(), k + 1);
                for (i, (q, f)) in d.path.iter().enumerate() {
                    let h = steps[i].hinges.iter().find(|h| h.name == name).unwrap();
                    assert_eq!((*q, *f), (h.plastic_deformation, h.force), "{name} state {i}");
                }
            }
        }
        // The generated names are those of the stored results, with the
        // moment hinge first as assigned.
        assert_eq!(hinge_names(&m, "PUSH", false), ["1H1", "1H2"]);
        let d = hinge_data(&m, "PUSH", "1H1", 3).unwrap();
        assert_eq!(
            (d.frame.as_str(), d.rel_dist, d.prop.as_str(), d.dof),
            ("1", 0.0, "Moment", HingeDof::M3)
        );
        assert_eq!(d.behavior, Some(HingeBehavior::DeformationControlled));
        // A step past the end selects the last state.
        assert_eq!(hinge_data(&m, "PUSH", "1H1", 99).unwrap().step_number, 9);
        assert!(hinge_data(&m, "PUSH", "1H9", 0).is_err());
        assert!(hinge_data(&m, "NOPE", "1H1", 0).is_err());
    }

    #[test]
    fn the_moments_at_the_published_points_are_p_times_the_arm() {
        // Example 1-026 (v19 verification suite), model A, kip and in. The
        // published base-shear points are P = 60, 80 and 20 kip at saved
        // states 1, 6 and 8 (as in `example_1_026_a_follows_the_published_points`
        // of the solver); the hinge is at the fixed end of the 24 in
        // cantilever, so the moments are P x 24 = 1440, 1920 and 480 kip in.
        // The 1440 and 1920 are also the entered B and C forces of the
        // property; the 480 is its D force.
        let m = analyzed();
        for (k, moment) in [(1, 1440.0), (6, 1920.0), (8, 480.0)] {
            let d = hinge_data(&m, "PUSH", "1H1", k).unwrap();
            assert!((d.force - moment).abs() <= 1e-9, "state {k}: {} vs {moment}", d.force);
            assert_eq!(*d.path.last().unwrap(), (d.plastic, d.force));
        }
    }

    #[test]
    fn maximum_and_minimum_run_over_the_states_up_to_the_step() {
        let m = analyzed();
        let steps = &m.case_results["PUSH"].static_steps;
        let d = hinge_data(&m, "PUSH", "1H1", 9).unwrap();
        // Largest plastic rotation: the final state of the solver unit test
        // `example_1_026_a_follows_the_published_points` (target 2.0 in) has
        // q = 0.066409694 rad (1e-8 there).
        assert!((d.max_plastic - 0.066409694).abs() <= 1e-8, "{}", d.max_plastic);
        let plastic: Vec<f64> = steps
            .iter()
            .map(|s| s.hinges.iter().find(|h| h.name == "1H1").unwrap().plastic_deformation)
            .collect();
        assert_eq!(d.max_plastic, plastic.iter().copied().fold(f64::MIN, f64::max));
        assert_eq!(d.min_plastic, 0.0, "the first state has no plastic deformation");
        // Strength is lost after the peak, so the maximum force is 1920 and
        // the final one 480 (entered C and D forces).
        assert!((d.max_force - 1920.0).abs() <= 1e-9);
        assert!((d.force - 480.0).abs() <= 1e-9);
        // At an early step the maximum covers only the states up to it.
        let early = hinge_data(&m, "PUSH", "1H1", 1).unwrap();
        assert!((early.max_force - 1440.0).abs() <= 1e-9);
        assert!(early.max_plastic < d.max_plastic);
        assert_eq!(early.path.len(), 2);
    }

    #[test]
    fn state_and_status_follow_the_stored_results_per_step() {
        let m = analyzed();
        let steps = &m.case_results["PUSH"].static_steps;
        // Plastic rotation 0 at state 0 and 1, then B to C, C to D, D to E
        // (`markers_follow_the_hinge_states_of_example_1_026_a_per_saved_state`
        // of the pushover window lists the classes of each state).
        let want = [
            HingeState::AToB,
            HingeState::AToB,
            HingeState::BToC,
            HingeState::BToC,
            HingeState::BToC,
            HingeState::BToC,
            HingeState::CToD,
            HingeState::CToD,
            HingeState::DToE,
            HingeState::DToE,
        ];
        for (k, w) in want.iter().enumerate() {
            let d = hinge_data(&m, "PUSH", "1H1", k).unwrap();
            let stored = steps[k].hinges.iter().find(|h| h.name == "1H1").unwrap();
            assert_eq!((d.state, d.status), (stored.state, stored.status));
            assert_eq!(d.state, *w, "state of 1H1 at saved state {k}");
        }
        assert_eq!(state_text(HingeState::BeyondE), ">E");
        assert_eq!(status_text(HingeStatus::LSToCP), "LS to CP");
        assert!(!hinge_data(&m, "PUSH", "1H1", 0).unwrap().acceptance_given);
    }

    #[test]
    fn the_drawn_backbone_is_the_entered_curve_with_b_at_zero() {
        let m = analyzed();
        let d = hinge_data(&m, "PUSH", "1H1", 9).unwrap();
        // Entered curve of Example 1-026 A (B, C, D, E as force, deformation):
        // (1440, 0) (1920, 0.04) (480, 0.04) (480, 0.4); plotted as
        // (deformation, force) with B at zero deformation, both branches.
        let want = [
            (-0.4, -480.0),
            (-0.04, -480.0),
            (-0.04, -1920.0),
            (0.0, -1440.0),
            (0.0, 1440.0),
            (0.04, 1920.0),
            (0.04, 480.0),
            (0.4, 480.0),
        ];
        assert_eq!(d.backbone.len(), want.len());
        for (g, w) in d.backbone.iter().zip(want) {
            assert!((g.0 - w.0).abs() <= 1e-12 && (g.1 - w.1).abs() <= 1e-12, "{g:?} vs {w:?}");
        }
        // Scale factors multiply, and B moves to zero for a table whose B
        // deformation is not zero; a non-symmetric property uses its own
        // negative branch.
        let mut prop = m.hinge_props["Moment"].clone();
        prop.positive[0].deformation = 0.01;
        prop.positive[1].deformation = 0.05;
        prop.force_sf = 2.0;
        prop.deformation_sf = 10.0;
        let b = backbone_points(&prop);
        assert_eq!(b[4], (0.0, 2880.0));
        assert!((b[5].0 - 0.4).abs() <= 1e-12 && (b[5].1 - 3840.0).abs() <= 1e-9);
        assert_eq!(b[3], (0.0, -2880.0));
        prop.symmetric = false;
        prop.force_sf_negative = 3.0;
        prop.negative[0] = sap_model::hinge::HingePoint::new(-100.0, -0.0);
        prop.negative[1] = sap_model::hinge::HingePoint::new(-200.0, -0.5);
        let b = backbone_points(&prop);
        assert_eq!(b[3], (0.0, -300.0));
        assert_eq!(b[2], (-0.5, -600.0));
        // The path follows the slope-limited branch (v19 AR p. 152): its
        // plastic rotation at the drop is not the drawn 0.04 at force 480.
        // The drawn drop from C to D is vertical at 0.04; a saved state after
        // the peak (state 7) lies at a deformation beyond 0.04 with a force
        // between D and C.
        let after_peak = hinge_data(&m, "PUSH", "1H1", 7).unwrap();
        let (q, f) = *after_peak.path.last().unwrap();
        assert!(q > 0.04 && f < 1920.0 && f > 480.0, "{q} {f}");
    }

    #[test]
    fn the_window_shows_the_fields_and_never_changes_the_model() {
        let mut model = analyzed();
        let before = model.clone();
        let mut state = State::default();
        state.open = true;
        state.step = 9;
        let ctx = egui::Context::default();
        let t = texts(&frames(&ctx, &mut state, &model));
        for want in [
            "Frame object",
            "Relative distance",
            "Hinge behavior",
            "Deformation controlled",
            "Hinge state",
            "D to E",
            "Hinge status",
            "Maximum to this step",
            "Minimum to this step",
            "Plastic rotation [rad]",
            "Moment [kip-in]",
            STRENGTH_LOSS_NOTE,
        ] {
            assert!(t.iter().any(|s| s == want), "missing {want:?}: {t:?}");
        }
        assert_eq!(state.hinge, "1H1");
        assert_eq!(state.case, "PUSH");
        assert_eq!(model, before, "the window must not edit the model");
        // Another hinge, in other units: the shear hinge in N and mm.
        state.hinge = "1H2".into();
        state.units = Some(Units::NMmC);
        let t = texts(&frames(&ctx, &mut state, &model));
        assert!(t.iter().any(|s| s == "Force [N]"), "{t:?}");
        assert!(t.iter().any(|s| s == "Plastic displacement [mm]"), "{t:?}");
        // The filter on selected frames: none selected lists nothing.
        state.only_selected = true;
        let t = texts(&frames(&ctx, &mut state, &model));
        assert!(t.iter().any(|s| s.contains("No hinge on the selected frames")), "{t:?}");
        model.frames.get_mut("1").unwrap().selected = true;
        assert_eq!(hinge_names(&model, "PUSH", true), ["1H1", "1H2"]);
        assert_eq!(
            hinge_names(&{ let mut m = model.clone(); m.frames.get_mut("1").unwrap().selected = false; m }, "PUSH", true),
            Vec::<String>::new()
        );
        // The menu command checks the filter box when frames are selected.
        let mut s = State::default();
        s.show(&model);
        assert!(s.open && s.only_selected);
        model.frames.get_mut("1").unwrap().selected = false;
        s.show(&model);
        assert!(!s.only_selected);
    }

    #[test]
    fn a_case_without_saved_states_gets_an_explanation_and_a_closed_window_draws_nothing() {
        let mut model = example_1_026_model();
        let mut state = State::default();
        state.open = true;
        let ctx = egui::Context::default();
        let t = texts(&frames(&ctx, &mut state, &model));
        assert!(t.iter().any(|s| s.contains("No nonlinear static results")), "{t:?}");
        // Results with one saved state at least are needed; a case that
        // saved only its final state has steps too, but a model without
        // hinges lists none.
        let mut analyzed = analyzed();
        analyzed.frames.get_mut("1").unwrap().hinges.clear();
        for step in analyzed.case_results.get_mut("PUSH").unwrap().static_steps.iter_mut() {
            step.hinges.clear();
        }
        let t = texts(&frames(&ctx, &mut state, &analyzed));
        assert!(t.iter().any(|s| s.contains("This case has no hinges")), "{t:?}");
        model.load_cases.clear();
        state.open = false;
        assert!(texts(&frames(&ctx, &mut state, &model)).is_empty());
    }

    #[test]
    fn the_plot_handles_degenerate_series() {
        let ctx = egui::Context::default();
        let _ = ctx.run(input(0.0), |ctx| {
            egui::CentralPanel::default().show(ctx, |ui| {
                let one = [(0.0, 0.0)];
                let bad = [(f64::NAN, 1.0), (1.0, 2.0)];
                let s = |xy| Series { xy, color: egui::Color32::WHITE, dots: true };
                draw_plot(ui, &[], None, "x", "y");
                draw_plot(ui, &[s(&one)], Some((0.0, 0.0)), "x", "y");
                draw_plot(ui, &[s(&bad)], Some((f64::NAN, 0.0)), "x", "y");
            });
        });
    }
}
