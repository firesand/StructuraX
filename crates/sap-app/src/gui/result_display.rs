//! The result forms of the viewport: "Display > Show Deformed Shape..." (with
//! displacement contours on area objects), "Display > Show Forces/Stresses >
//! Shells...", their legend, the joint read-out of a right-click and the
//! animation through the steps of a result.
//!
//! Everything a user reads comes from a pure function of this file (form
//! validation, value tables, legend text, read-out text); `gui.rs` only holds
//! the state, the menu entries and the drawing calls. Sources: the help topics
//! "Show Deformed Shape" and "Show Forces/Stresses (Shells)" of the vendor
//! (components, range 0/0 = automatic, display units, "Joint displacements can
//! not be displayed during animation") and the CSI Analysis Reference Manual,
//! p. 24 (joint output axes).
//!
//! Display units: a form shows its values in units of its own; the model, its
//! results and its units selector are never touched (see `display_units`).

use std::collections::BTreeMap;

use eframe::egui::{self, Color32, FontId, Pos2, Rect, Vec2};
use sap_model::{CaseResults, SapModel};
use sap_units::Units;

use super::contours::{
    self, ContourRange, DisplacementComponent, JointReadout, StepSelection, LEGEND_INTERVALS,
};
use super::display_units::{self, Dimension};
use super::shell_output::{self, Face, ResultKind, ShellQuantity, ShellResultant, ShellStress};
use super::view::ViewTransform;
use super::{DisplayMode, SapGuiApp, ViewProjection};

#[cfg(test)]
mod tests;

/// Seconds a step of a multi-step result stays on screen during an animation.
pub(super) const SECONDS_PER_STEP: f64 = 0.05;
/// Edge divisions of a contour mesh (a quad is drawn as 4 x 4 cells).
pub(super) const SUBDIVISIONS: usize = 4;
/// Radius, in pixels, within which a right-click picks a joint (the radius of
/// the left-click pick).
pub(super) const JOINT_PICK_RADIUS: f32 = 10.0;
/// Opacity of contour fills (the legacy shell contour used 145).
pub(super) const CONTOUR_ALPHA: u8 = 145;
/// Fill of an area whose value could not be obtained while a contour is shown
/// (the colour the legacy shell contour used for it): no ramp colour.
pub(super) fn no_contour_fill() -> Color32 {
    Color32::from_rgba_unmultiplied(60, 120, 180, 65)
}
pub(super) const ANIMATION_REFUSED: &str =
    "Joint displacements can not be displayed during animation.";

// ------------------------------------------------------------------ state

/// What the contour of the viewport shows, as applied by a form.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub(super) enum Contour {
    /// No form contour: the legacy displays.
    #[default]
    Off,
    /// Displacement contour on the deformed shape (`DisplayMode::Deformed`).
    Displacement {
        component: DisplacementComponent,
        /// Minimum and maximum in display units; `(0, 0)` is automatic.
        range: (f64, f64),
        units: Units,
    },
    /// Shell contour of `DisplayMode::ShellContour(q)`; with `visible` the
    /// face of a stress is resolved per area from the view.
    Shell {
        visible: bool,
        range: (f64, f64),
        units: Units,
    },
}

/// State of the result forms; reset with the model.
#[derive(Clone, Debug, Default)]
pub(super) struct State {
    pub deformed: DeformedForm,
    pub shells: ShellForm,
    pub contour: Contour,
    /// Joint whose displacements the floating read-out shows.
    pub readout_joint: Option<String>,
    /// Display units of the read-out (the last deformed-shape form's).
    pub readout_units: Option<Units>,
    /// Input time at which the running animation started.
    pub anim_start: Option<f64>,
    /// True while an animation steps through a multi-step result (the sine
    /// scaling is then off).
    pub stepping: bool,
}

impl State {
    /// A right-click picked `joint`: show its read-out, or refuse while animating.
    pub fn open_readout(&mut self, joint: &str, animating: bool) -> Result<(), String> {
        if animating {
            self.readout_joint = None;
            return Err(ANIMATION_REFUSED.to_string());
        }
        self.readout_joint = Some(joint.to_string());
        Ok(())
    }
}

// ------------------------------------------------------------- parsing

/// `(min, max)` from the two text fields of a form: an empty field is 0, so an
/// untouched form asks for the automatic range. Not finite, or not a number, is
/// an error naming the field.
pub(super) fn parse_range(min: &str, max: &str) -> Result<(f64, f64), String> {
    let one = |text: &str, what: &str| -> Result<f64, String> {
        let t = text.trim();
        if t.is_empty() {
            return Ok(0.0);
        }
        t.parse::<f64>()
            .ok()
            .filter(|v| v.is_finite())
            .ok_or_else(|| format!("the contour {what} '{t}' is not a number"))
    };
    let range = (one(min, "minimum")?, one(max, "maximum")?);
    // The same rule the contour applies: a given range needs min < max.
    contours::contour_range(&[], range)?;
    Ok(range)
}

// ---------------------------------------------------------------- forms

pub(super) const DISPLACEMENT_COMPONENTS: [(DisplacementComponent, &str); 6] = [
    (DisplacementComponent::Ux, "Ux"),
    (DisplacementComponent::Uy, "Uy"),
    (DisplacementComponent::Uz, "Uz"),
    (DisplacementComponent::Area1, "Area 1 Axis"),
    (DisplacementComponent::Area2, "Area 2 Axis"),
    (DisplacementComponent::Area3, "Area 3 Axis"),
];

pub(super) fn displacement_label(c: DisplacementComponent) -> &'static str {
    DISPLACEMENT_COMPONENTS
        .iter()
        .find(|(k, _)| *k == c)
        .map_or("", |(_, l)| l)
}

/// Draft of "Show Deformed Shape...".
#[derive(Clone, Debug)]
pub(super) struct DeformedForm {
    pub open: bool,
    pub case: String,
    /// One-based mode number of a modal or buckling result.
    pub mode: usize,
    /// Zero-based step of a multi-step result.
    pub step: usize,
    pub scale: f32,
    pub contour: bool,
    pub component: DisplacementComponent,
    pub min: String,
    pub max: String,
    pub units: Units,
    pub error: String,
}

impl Default for DeformedForm {
    fn default() -> Self {
        Self {
            open: false,
            case: String::new(),
            mode: 1,
            step: 0,
            scale: 1.0,
            contour: false,
            component: DisplacementComponent::Uz,
            min: String::new(),
            max: String::new(),
            units: Units::default(),
            error: String::new(),
        }
    }
}

/// What a valid deformed-shape form sets in the application.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct DeformedApply {
    pub case: String,
    /// Zero-based.
    pub mode_idx: usize,
    pub use_step: bool,
    pub step_idx: usize,
    pub scale: f32,
    pub contour: Contour,
    pub units: Units,
}

/// Case and combination names that have results, in model order.
pub(super) fn result_names(model: &SapModel) -> Vec<String> {
    model.case_results.keys().cloned().collect()
}

impl DeformedForm {
    pub fn open_with(&mut self, case: &str, scale: f32, units: Units) {
        self.open = true;
        self.case = case.to_string();
        self.scale = scale;
        self.units = units;
        self.error.clear();
    }

    /// Validates the form against the model; nothing is changed.
    pub fn apply(&self, model: &SapModel) -> Result<DeformedApply, String> {
        let cr = model
            .case_results
            .get(&self.case)
            .ok_or_else(|| format!("'{}' has no results; run the analysis first", self.case))?;
        if !(self.scale.is_finite() && self.scale > 0.0) {
            return Err("the scale factor must be a positive number".into());
        }
        let (mut mode_idx, mut use_step, mut step_idx) = (0, false, 0);
        if !cr.modes.is_empty() {
            if self.mode == 0 || self.mode > cr.modes.len() {
                return Err(format!(
                    "mode {} does not exist: '{}' has {} modes",
                    self.mode,
                    self.case,
                    cr.modes.len()
                ));
            }
            mode_idx = self.mode - 1;
        } else {
            let n = contours::step_count(cr);
            if n >= 2 {
                if self.step >= n {
                    return Err(format!(
                        "step {} does not exist: '{}' has steps 0 to {}",
                        self.step,
                        self.case,
                        n - 1
                    ));
                }
                use_step = true;
                step_idx = self.step;
            }
        }
        let range = parse_range(&self.min, &self.max)?;
        let contour = if self.contour {
            Contour::Displacement { component: self.component, range, units: self.units }
        } else {
            Contour::Off
        };
        Ok(DeformedApply {
            case: self.case.clone(),
            mode_idx,
            use_step,
            step_idx,
            scale: self.scale,
            contour,
            units: self.units,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum ShellKind {
    Resultant,
    Stress,
}

pub(super) const RESULTANT_ITEMS: [(ShellResultant, &str); 14] = [
    (ShellResultant::F11, "F11"),
    (ShellResultant::F22, "F22"),
    (ShellResultant::F12, "F12"),
    (ShellResultant::FMax, "FMAX"),
    (ShellResultant::FMin, "FMIN"),
    (ShellResultant::Fvm, "FVM"),
    (ShellResultant::M11, "M11"),
    (ShellResultant::M22, "M22"),
    (ShellResultant::M12, "M12"),
    (ShellResultant::MMax, "MMAX"),
    (ShellResultant::MMin, "MMIN"),
    (ShellResultant::V13, "V13"),
    (ShellResultant::V23, "V23"),
    (ShellResultant::VMax, "VMAX"),
];

pub(super) const STRESS_ITEMS: [(ShellStress, &str); 9] = [
    (ShellStress::S11, "S11"),
    (ShellStress::S22, "S22"),
    (ShellStress::S12, "S12"),
    (ShellStress::SMax, "SMAX"),
    (ShellStress::SMin, "SMIN"),
    (ShellStress::Svm, "SVM"),
    (ShellStress::S13, "S13"),
    (ShellStress::S23, "S23"),
    (ShellStress::SMaxV, "SMAXV"),
];

/// The output-face choices of the form.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum FaceChoice {
    Visible,
    Fixed(Face),
}

pub(super) const FACE_CHOICES: [(FaceChoice, &str); 6] = [
    (FaceChoice::Visible, "Visible Face"),
    (FaceChoice::Fixed(Face::Top), "Top Face"),
    (FaceChoice::Fixed(Face::Bottom), "Bottom Face"),
    (FaceChoice::Fixed(Face::Maximum), "Max"),
    (FaceChoice::Fixed(Face::Minimum), "Min"),
    (FaceChoice::Fixed(Face::AbsMaximum), "Absolute Max"),
];

/// Draft of "Show Forces/Stresses > Shells...".
#[derive(Clone, Debug)]
pub(super) struct ShellForm {
    pub open: bool,
    pub case: String,
    pub kind: ShellKind,
    pub resultant: ShellResultant,
    pub stress: ShellStress,
    pub face: FaceChoice,
    pub min: String,
    pub max: String,
    pub units: Units,
    pub error: String,
}

impl Default for ShellForm {
    fn default() -> Self {
        Self {
            open: false,
            case: String::new(),
            kind: ShellKind::Resultant,
            resultant: ShellResultant::F11,
            stress: ShellStress::S11,
            face: FaceChoice::Visible,
            min: String::new(),
            max: String::new(),
            units: Units::default(),
            error: String::new(),
        }
    }
}

/// What a valid shell form sets in the application.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct ShellApply {
    pub case: String,
    pub quantity: ShellQuantity,
    pub contour: Contour,
}

impl ShellForm {
    pub fn open_with(&mut self, case: &str, units: Units) {
        self.open = true;
        self.case = case.to_string();
        self.units = units;
        self.error.clear();
    }

    /// The quantity the form names. With the visible face a stress carries
    /// `Face::Top` here; the face is resolved per area when the values are
    /// tabulated.
    pub fn quantity(&self) -> ShellQuantity {
        match self.kind {
            ShellKind::Resultant => ShellQuantity::Resultant(self.resultant),
            ShellKind::Stress => {
                let face = match self.face {
                    FaceChoice::Visible => Face::Top,
                    FaceChoice::Fixed(f) => f,
                };
                ShellQuantity::Stress(self.stress, face)
            }
        }
    }

    pub fn apply(&self, model: &SapModel) -> Result<ShellApply, String> {
        if !model.case_results.contains_key(&self.case) {
            return Err(format!("'{}' has no results; run the analysis first", self.case));
        }
        let range = parse_range(&self.min, &self.max)?;
        let quantity = self.quantity();
        let visible = self.kind == ShellKind::Stress && self.face == FaceChoice::Visible;
        Ok(ShellApply {
            case: self.case.clone(),
            quantity,
            contour: Contour::Shell { visible, range, units: self.units },
        })
    }
}

// ------------------------------------------------------------ view direction

/// Unit vector from the model toward the eye for a pane, read from
/// `ViewTransform::project`: with screen right `r` and screen up `u` (both
/// unit vectors in model axes), the eye lies along `r x u`.
///
/// Plan (`x` right, `y` up): +Z. Elevation X-Z (`x` right, `z` up): -Y.
/// Elevation Y-Z (`y` right, `z` up): +X. In 3D, from the rotation of
/// `project` (`x1 = x cos(yaw) - y sin(yaw)` to the right, up `z cos(pitch) +
/// y1 sin(pitch)`): right x up = `(-sin(yaw) cos(pitch), -cos(yaw) cos(pitch),
/// sin(pitch))`; a positive pitch, the default, looks from above.
pub(super) fn toward_viewer(view: &ViewTransform) -> [f64; 3] {
    match view.proj {
        ViewProjection::PlanXY => [0.0, 0.0, 1.0],
        ViewProjection::ElevationXZ => [0.0, -1.0, 0.0],
        ViewProjection::ElevationYZ => [1.0, 0.0, 0.0],
        ViewProjection::Perspective3D => {
            let (sy, cy) = view.yaw.sin_cos();
            let (sp, cp) = view.pitch.sin_cos();
            [f64::from(-sy * cp), f64::from(-cy * cp), f64::from(sp)]
        }
    }
}

// --------------------------------------------------------------- the table

/// Per-area contour values of one frame of one pane, computed once and read by
/// the range, the legend and the fill.
#[derive(Clone, Debug, Default)]
pub(super) struct Table {
    /// One value per area (a shell, flat) or one per corner joint (a
    /// displacement), in model units; an error names why there is none.
    pub entries: BTreeMap<String, Result<Vec<f64>, String>>,
    /// Range in model units; `None` when there is nothing to colour.
    pub range: Option<ContourRange>,
    /// What the legend area says besides the scale: missing results, how many
    /// areas have no value and the first reason, an invalid range.
    pub messages: Vec<String>,
}

fn finish(
    entries: BTreeMap<String, Result<Vec<f64>, String>>,
    user: (f64, f64),
    mut messages: Vec<String>,
) -> Table {
    let failed: Vec<&String> = entries.values().filter_map(|e| e.as_ref().err()).collect();
    if let Some(first) = failed.first() {
        messages.push(format!(
            "{} area(s) are drawn without a contour colour; first reason: {first}",
            failed.len()
        ));
    }
    let values: Vec<f64> = entries
        .values()
        .filter_map(|e| e.as_ref().ok())
        .flatten()
        .copied()
        .collect();
    let range = match contours::contour_range(&values, user) {
        Ok(Some(r)) => Some(r),
        Ok(None) => {
            if messages.is_empty() {
                messages.push("There is no finite value to contour.".into());
            }
            None
        }
        Err(e) => {
            messages.push(e);
            None
        }
    };
    Table { entries, range, messages }
}

fn no_results(result: &str) -> Table {
    Table {
        messages: vec![format!("'{result}' has no results; run the analysis first.")],
        ..Table::default()
    }
}

/// The user's range, given in display units, in model units.
fn model_range(range: (f64, f64), factor: f64) -> (f64, f64) {
    if range == (0.0, 0.0) || !(factor.is_finite() && factor > 0.0) {
        return range;
    }
    (range.0 / factor, range.1 / factor)
}

/// Value of every area for the shell quantity `q`; with `visible` the face of
/// a stress is the one that looks toward `toward_viewer`.
pub(super) fn shell_table(
    model: &SapModel,
    result: &str,
    q: ShellQuantity,
    visible: bool,
    toward: [f64; 3],
    user_model: (f64, f64),
) -> Table {
    let Some(cr) = model.case_results.get(result) else {
        return no_results(result);
    };
    let mut messages = Vec::new();
    if cr.area_results.is_empty() {
        messages.push(
            shell_output::missing_results_message(model, result, ResultKind::Area)
                .unwrap_or_else(|| format!("'{result}' has no shell results.")),
        );
        return finish(BTreeMap::new(), user_model, messages);
    }
    if let Some(m) = shell_output::missing_results_message(model, result, ResultKind::Area) {
        messages.push(m);
    }
    let resolve_face = visible && matches!(q, ShellQuantity::Stress(..));
    let mut entries = BTreeMap::new();
    for aid in model.areas.keys() {
        let quantity = match q {
            ShellQuantity::Stress(s, _) if resolve_face => contours::area_local_axes(model, aid)
                .map(|axes| ShellQuantity::Stress(s, shell_output::visible_face(axes[2], toward))),
            _ => Ok(q),
        };
        let value = quantity.and_then(|qq| shell_output::area_value(model, result, aid, qq));
        entries.insert(aid.clone(), value.map(|v| vec![v]));
    }
    finish(entries, user_model, messages)
}

/// Displacement component at the corner joints of every area.
pub(super) fn displacement_table(
    model: &SapModel,
    result: &str,
    comp: DisplacementComponent,
    sel: StepSelection,
    user_model: (f64, f64),
) -> Table {
    let Some(cr) = model.case_results.get(result) else {
        return no_results(result);
    };
    let mut entries = BTreeMap::new();
    for (aid, area) in &model.areas {
        let value = if (3..=4).contains(&area.points.len()) {
            contours::area_vertex_values(model, cr, aid, comp, sel)
        } else {
            Err(format!(
                "area '{aid}' has {} joints; a displacement contour needs three or four",
                area.points.len()
            ))
        };
        entries.insert(aid.clone(), value);
    }
    finish(entries, user_model, Vec::new())
}

// ------------------------------------------------------------------ legend

/// Tick labels of one scale: fixed notation with as many decimals as the
/// span needs for about four significant digits, scientific notation when a
/// tick is below 0.001 or at least 1e6 in magnitude (or more than six decimals
/// would be needed).
pub(super) fn tick_labels(ticks: &[f64]) -> Vec<String> {
    let (Some(&first), Some(&last)) = (ticks.first(), ticks.last()) else {
        return Vec::new();
    };
    let span = (last - first).abs();
    let max_abs = ticks.iter().fold(0.0_f64, |m, t| m.max(t.abs()));
    let decimals = if span > 0.0 {
        (3.0 - span.log10().floor()).clamp(0.0, 12.0) as usize
    } else {
        3
    };
    let scientific = max_abs >= 1e6 || (max_abs > 0.0 && max_abs < 1e-3) || decimals > 6;
    ticks
        .iter()
        .map(|t| {
            if !t.is_finite() {
                "-".to_string()
            } else if scientific {
                format!("{t:.3e}")
            } else {
                format!("{t:.decimals$}")
            }
        })
        .collect()
}

/// A single number of the read-out: five significant digits.
pub(super) fn number_text(v: f64) -> String {
    if !v.is_finite() {
        "-".to_string()
    } else if v != 0.0 && (v.abs() < 1e-3 || v.abs() >= 1e6) {
        format!("{v:.5e}")
    } else {
        format!("{v:.6}")
    }
}

/// The legend: title, bands, tick labels and the messages of the table.
#[derive(Clone, Debug, Default, PartialEq)]
pub(super) struct Legend {
    /// Label and unit, for example `Von Mises Stress SVM (Visible Face) [N/mm2]`.
    pub title: String,
    /// Range in model units (colours are taken in model units).
    pub range: Option<ContourRange>,
    /// Value at the middle of each band, in model units, lowest first.
    pub bands: Vec<f64>,
    /// `LEGEND_INTERVALS + 1` labels in display units, lowest first.
    pub ticks: Vec<String>,
    pub messages: Vec<String>,
}

pub(super) fn build_legend(label: &str, dim: Dimension, model_units: Units, display: Units, table: &Table) -> Legend {
    let factor = display_units::factor(dim, model_units, display);
    let unit = display_units::label(dim, display);
    let (mut bands, mut ticks) = (Vec::new(), Vec::new());
    if let Some(r) = table.range {
        bands = (0..LEGEND_INTERVALS)
            .map(|k| r.lo + (r.hi - r.lo) * (k as f64 + 0.5) / LEGEND_INTERVALS as f64)
            .collect();
        let shown: Vec<f64> = contours::legend_ticks(r, LEGEND_INTERVALS)
            .iter()
            .map(|t| t * factor)
            .collect();
        ticks = tick_labels(&shown);
    }
    Legend {
        title: format!("{label} [{unit}]"),
        range: table.range,
        bands,
        ticks,
        messages: table.messages.clone(),
    }
}

/// The legend title of a shell quantity; with the visible face a stress ends
/// with `(Visible Face)` in place of its face.
pub(super) fn shell_title(q: ShellQuantity, visible: bool) -> String {
    match q {
        ShellQuantity::Stress(s, _) if visible && !matches!(s, ShellStress::S13 | ShellStress::S23 | ShellStress::SMaxV) => {
            // The Bottom label ends with "(Bottom)", also for SVM.
            let label = ShellQuantity::Stress(s, Face::Bottom).label();
            label.trim_end_matches("(Bottom)").to_string() + "(Visible Face)"
        }
        _ => q.label(),
    }
}

/// Table and legend of one pane for one frame.
#[derive(Clone, Debug)]
pub(super) struct FrameContour {
    pub table: Table,
    pub legend: Legend,
}

/// What the contour of the current display is, from the display mode and the
/// applied form.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Target {
    Displacement(DisplacementComponent),
    Shell { quantity: ShellQuantity, visible: bool },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct ContourSpec {
    pub target: Target,
    /// Range in display units.
    pub range: (f64, f64),
    pub units: Units,
}

/// The contour to draw. A shell contour without an applied form is the legacy
/// one (model units, automatic range, the quantity's own face).
pub(super) fn contour_spec(mode: DisplayMode, contour: Contour, model_units: Units) -> Option<ContourSpec> {
    match (mode, contour) {
        (DisplayMode::Deformed, Contour::Displacement { component, range, units }) => {
            Some(ContourSpec { target: Target::Displacement(component), range, units })
        }
        (DisplayMode::ShellContour(q), Contour::Shell { visible, range, units }) => Some(ContourSpec {
            target: Target::Shell { quantity: q, visible },
            range,
            units,
        }),
        (DisplayMode::ShellContour(q), _) => Some(ContourSpec {
            target: Target::Shell { quantity: q, visible: false },
            range: (0.0, 0.0),
            units: model_units,
        }),
        _ => None,
    }
}

/// Table and legend for `spec` in a pane looking toward `toward`.
pub(super) fn frame_contour(
    model: &SapModel,
    result: &str,
    spec: ContourSpec,
    sel: StepSelection,
    toward: [f64; 3],
) -> FrameContour {
    let (dim, label) = match spec.target {
        Target::Displacement(c) => (Dimension::Length, format!("Displacement {}", displacement_label(c))),
        Target::Shell { quantity, visible } => (quantity.dimension(), shell_title(quantity, visible)),
    };
    let factor = display_units::factor(dim, model.present_units, spec.units);
    let user = model_range(spec.range, factor);
    let table = match spec.target {
        Target::Displacement(c) => displacement_table(model, result, c, sel, user),
        Target::Shell { quantity, visible } => shell_table(model, result, quantity, visible, toward, user),
    };
    let legend = build_legend(&label, dim, model.present_units, spec.units, &table);
    FrameContour { table, legend }
}

/// How the viewport fills an area while a contour is shown.
#[derive(Clone, Debug)]
pub(super) enum AreaFill {
    /// The ordinary fill (nothing to colour).
    Plain,
    /// One colour for the whole polygon.
    Flat(Color32),
    /// A mesh on the polygon, to be drawn instead of a fill.
    Mesh(egui::Mesh),
}

impl FrameContour {
    /// Fill of area `aid` drawn as the screen polygon `poly` (the corner
    /// joints in order). A flat value gives one colour, corner values a mesh
    /// of `SUBDIVISIONS` cells per edge; an area without a value has
    /// `no_contour_fill`, never the colour of zero.
    pub fn fill(&self, aid: &str, poly: &[Pos2]) -> AreaFill {
        // An area whose value is refused is marked as such even when no area
        // has a value and there is therefore no range.
        if matches!(self.table.entries.get(aid), Some(Err(_))) {
            return AreaFill::Flat(no_contour_fill());
        }
        let Some(range) = self.table.range else {
            return AreaFill::Plain;
        };
        match self.table.entries.get(aid) {
            None => AreaFill::Plain,
            Some(Err(_)) => AreaFill::Flat(no_contour_fill()),
            Some(Ok(v)) if v.len() == 1 => {
                AreaFill::Flat(contours::value_color(v[0], range.lo, range.hi, CONTOUR_ALPHA))
            }
            Some(Ok(v)) if v.len() == poly.len() && (3..=4).contains(&v.len()) => {
                AreaFill::Mesh(contours::colored_polygon(poly, v, range, CONTOUR_ALPHA, SUBDIVISIONS))
            }
            Some(Ok(_)) => AreaFill::Flat(no_contour_fill()),
        }
    }
}

/// Paints the legend in the top-right corner of `rect`.
pub(super) fn draw_legend(painter: &egui::Painter, rect: Rect, legend: &Legend) {
    const W: f32 = 230.0;
    const BAND_H: f32 = 16.0;
    let text_col = Color32::from_rgb(210, 225, 245);
    let tick_col = Color32::from_rgb(180, 200, 230);
    let note_col = Color32::from_rgb(255, 190, 120);
    let left = rect.right() - W - 8.0;
    let top = rect.top() + 8.0;
    let title = painter.layout(legend.title.clone(), FontId::proportional(11.0), text_col, W - 12.0);
    let notes: Vec<_> = legend
        .messages
        .iter()
        .map(|m| painter.layout(m.clone(), FontId::proportional(10.0), note_col, W - 12.0))
        .collect();
    let bands_h = if legend.range.is_some() { BAND_H * LEGEND_INTERVALS as f32 + 8.0 } else { 0.0 };
    let notes_h: f32 = notes.iter().map(|g| g.size().y + 4.0).sum();
    let height = 8.0 + title.size().y + 4.0 + bands_h + notes_h + 4.0;
    painter.rect_filled(
        Rect::from_min_size(Pos2::new(left, top), Vec2::new(W, height)),
        4.0,
        Color32::from_rgba_unmultiplied(12, 16, 24, 220),
    );
    let mut y = top + 6.0;
    let title_h = title.size().y;
    painter.galley(Pos2::new(left + 6.0, y), title, text_col);
    y += title_h + 6.0;
    if let Some(r) = legend.range {
        let n = legend.bands.len();
        // Highest band on top, lowest at the bottom.
        let bottom = y + BAND_H * n as f32;
        for (k, v) in legend.bands.iter().enumerate() {
            let band_top = bottom - BAND_H * (k as f32 + 1.0);
            painter.rect_filled(
                Rect::from_min_size(Pos2::new(left + 6.0, band_top), Vec2::new(18.0, BAND_H)),
                0.0,
                contours::value_color(*v, r.lo, r.hi, 230),
            );
        }
        for (k, label) in legend.ticks.iter().enumerate() {
            painter.text(
                Pos2::new(left + 30.0, bottom - BAND_H * k as f32),
                egui::Align2::LEFT_CENTER,
                label,
                FontId::proportional(10.0),
                tick_col,
            );
        }
        y = bottom + 8.0;
    }
    for g in notes {
        let h = g.size().y;
        painter.galley(Pos2::new(left + 6.0, y), g, note_col);
        y += h + 4.0;
    }
}

// --------------------------------------------------------- deformed shape

/// Displacement `scale * (UX, UY, UZ)` of a joint in global axes. The stored
/// result is in the joint's local axes (Analysis Reference p. 24), so a joint
/// with local axes is moved along its rotated axes. A non-finite component
/// counts as zero.
pub(super) fn displaced_point(
    model: &SapModel,
    cr: Option<&CaseResults>,
    joint: &str,
    base: [f64; 3],
    scale: f64,
    sel: StepSelection,
) -> [f64; 3] {
    let Some(u) = cr.and_then(|cr| contours::joint_displacement_global(model, cr, joint, sel)) else {
        return base;
    };
    let d = |i: usize| if u[i].is_finite() { u[i] * scale } else { 0.0 };
    [base[0] + d(0), base[1] + d(1), base[2] + d(2)]
}

/// The step an animation shows `now - start` seconds after it began, for a
/// result with steps; `None` for a result without (modal or single state).
pub(super) fn step_animation(cr: Option<&CaseResults>, start: f64, now: f64) -> Option<usize> {
    let n = contours::step_count(cr?);
    (n >= 2).then(|| contours::animation_step(now - start, n, SECONDS_PER_STEP))
}

/// The closest of `points` within `radius` of `at`.
pub(super) fn closest_within(points: &[(String, Pos2)], at: Pos2, radius: f32) -> Option<String> {
    points
        .iter()
        .map(|(id, p)| (id, p.distance(at)))
        .filter(|(_, d)| *d < radius)
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(id, _)| id.clone())
}

/// Lines of the joint read-out window: a header and one `label`, `value` pair
/// per component; an error is one line.
pub(super) fn readout_lines(
    model: &SapModel,
    result: &str,
    joint: &str,
    sel: StepSelection,
    display: Units,
) -> (String, Vec<(String, String)>) {
    let header = format!("Joint {joint} (result {result}), local axes");
    match contours::joint_readout(model, result, joint, sel, display) {
        Ok(JointReadout { labels, values, .. }) => (
            header,
            labels.iter().cloned().zip(values.iter().map(|v| number_text(*v))).collect(),
        ),
        Err(e) => (header, vec![("".to_string(), e)]),
    }
}

// --------------------------------------------------------------- windows

fn units_combo(ui: &mut egui::Ui, id: &str, units: &mut Units) {
    egui::ComboBox::from_id_salt(id)
        .selected_text(units.s2k_label())
        .show_ui(ui, |ui| {
            for code in 1..=16 {
                if let Ok(u) = Units::from_code(code) {
                    ui.selectable_value(units, u, u.s2k_label());
                }
            }
        });
}

fn case_combo(ui: &mut egui::Ui, id: &str, model: &SapModel, case: &mut String) {
    let names = result_names(model);
    if names.is_empty() {
        ui.label("No results: run the analysis (F5).");
        return;
    }
    egui::ComboBox::from_id_salt(id)
        .selected_text(case.as_str())
        .show_ui(ui, |ui| {
            for n in &names {
                ui.selectable_value(case, n.clone(), n);
            }
        });
}

fn range_fields(ui: &mut egui::Ui, min: &mut String, max: &mut String) {
    ui.horizontal(|ui| {
        ui.label("Min");
        ui.add(egui::TextEdit::singleline(min).desired_width(70.0));
        ui.label("Max");
        ui.add(egui::TextEdit::singleline(max).desired_width(70.0));
    });
    ui.label("Min = Max = 0 selects the automatic range.");
}

impl SapGuiApp {
    pub(super) fn open_deformed_form(&mut self) {
        let (case, scale, units) = (self.active_case.clone(), self.deform_scale, self.model.present_units);
        let form = &mut self.results.deformed;
        form.open_with(&case, scale, units);
        if let Contour::Displacement { component, range, units } = self.results.contour {
            let form = &mut self.results.deformed;
            form.contour = true;
            form.component = component;
            form.units = units;
            form.min = if range.0 == 0.0 { String::new() } else { range.0.to_string() };
            form.max = if range.1 == 0.0 { String::new() } else { range.1.to_string() };
        }
    }

    pub(super) fn open_shell_form(&mut self) {
        let (case, units) = (self.active_case.clone(), self.model.present_units);
        self.results.shells.open_with(&case, units);
    }

    fn apply_deformed(&mut self, a: DeformedApply) {
        self.active_case = a.case;
        self.sync_active_case();
        self.active_mode_idx = a.mode_idx;
        self.use_time_step = a.use_step;
        self.active_step_idx = a.step_idx;
        self.deform_scale = a.scale;
        self.display_mode = DisplayMode::Deformed;
        self.results.contour = a.contour;
        self.results.readout_units = Some(a.units);
        self.status_msg = format!("Deformed shape of '{}'.", self.active_case);
    }

    fn apply_shells(&mut self, a: ShellApply) {
        self.active_case = a.case;
        self.sync_active_case();
        self.display_mode = DisplayMode::ShellContour(a.quantity);
        self.results.contour = a.contour;
        self.status_msg = format!("{} of '{}'.", a.quantity.label(), self.active_case);
    }

    /// The two forms and the joint read-out.
    pub(super) fn render_result_forms(&mut self, ctx: &egui::Context) {
        if self.results.deformed.open {
            self.deformed_form_window(ctx);
        }
        if self.results.shells.open {
            self.shell_form_window(ctx);
        }
        self.joint_readout_window(ctx);
    }

    fn deformed_form_window(&mut self, ctx: &egui::Context) {
        let mut form = self.results.deformed.clone();
        let mut open = true;
        let (mut ok, mut cancel) = (false, false);
        let model = &self.model;
        let cr = model.case_results.get(&form.case);
        let modes = cr.map_or(0, |c| c.modes.len());
        let steps = cr.map_or(0, contours::step_count);
        egui::Window::new("Show Deformed Shape")
            .open(&mut open)
            .resizable(false)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Case/Combo Name");
                    case_combo(ui, "deformed_case", model, &mut form.case);
                });
                if modes > 0 {
                    ui.horizontal(|ui| {
                        ui.label(format!("Mode (1 to {modes})"));
                        ui.add(egui::DragValue::new(&mut form.mode).range(1..=modes));
                    });
                } else if steps >= 2 {
                    ui.horizontal(|ui| {
                        ui.label(format!("Step (0 to {})", steps - 1));
                        ui.add(egui::DragValue::new(&mut form.step).range(0..=steps - 1));
                    });
                }
                ui.horizontal(|ui| {
                    ui.label("Scale Factor");
                    ui.add(egui::DragValue::new(&mut form.scale).speed(1.0));
                });
                ui.separator();
                ui.checkbox(&mut form.contour, "Draw Displacement Contours on Area Objects");
                ui.add_enabled_ui(form.contour, |ui| {
                    ui.horizontal_wrapped(|ui| {
                        for (c, label) in DISPLACEMENT_COMPONENTS {
                            ui.radio_value(&mut form.component, c, label);
                        }
                    });
                    range_fields(ui, &mut form.min, &mut form.max);
                    ui.horizontal(|ui| {
                        ui.label("Display Units");
                        units_combo(ui, "deformed_units", &mut form.units);
                    });
                });
                ui.label("Joint displacements are drawn in global axes.");
                if !form.error.is_empty() {
                    ui.colored_label(Color32::from_rgb(255, 120, 100), &form.error);
                }
                ui.horizontal(|ui| {
                    ok = ui.button("OK").clicked();
                    cancel = ui.button("Cancel").clicked();
                });
            });
        if ok {
            match form.apply(&self.model) {
                Ok(a) => {
                    self.apply_deformed(a);
                    form.open = false;
                    form.error.clear();
                }
                Err(e) => form.error = e,
            }
        }
        if cancel || !open {
            form.open = false;
        }
        self.results.deformed = form;
    }

    fn shell_form_window(&mut self, ctx: &egui::Context) {
        let mut form = self.results.shells.clone();
        let mut open = true;
        let (mut ok, mut cancel) = (false, false);
        let model = &self.model;
        egui::Window::new("Show Forces/Stresses (Shells)")
            .open(&mut open)
            .resizable(false)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Case/Combo Name");
                    case_combo(ui, "shell_case", model, &mut form.case);
                });
                ui.horizontal(|ui| {
                    ui.radio_value(&mut form.kind, ShellKind::Resultant, "Resultant Forces");
                    ui.radio_value(&mut form.kind, ShellKind::Stress, "Shell Stresses");
                });
                ui.horizontal_wrapped(|ui| match form.kind {
                    ShellKind::Resultant => {
                        for (r, label) in RESULTANT_ITEMS {
                            ui.selectable_value(&mut form.resultant, r, label);
                        }
                    }
                    ShellKind::Stress => {
                        for (s, label) in STRESS_ITEMS {
                            ui.selectable_value(&mut form.stress, s, label);
                        }
                    }
                });
                if form.kind == ShellKind::Stress {
                    ui.label("Output");
                    ui.horizontal_wrapped(|ui| {
                        for (c, label) in FACE_CHOICES {
                            ui.radio_value(&mut form.face, c, label);
                        }
                    });
                    ui.label("S13, S23 and SMAXV have no face.");
                }
                ui.separator();
                range_fields(ui, &mut form.min, &mut form.max);
                ui.horizontal(|ui| {
                    ui.label("Display Units");
                    units_combo(ui, "shell_units", &mut form.units);
                });
                ui.label("Element-centre values, no stress averaging.");
                if !form.error.is_empty() {
                    ui.colored_label(Color32::from_rgb(255, 120, 100), &form.error);
                }
                ui.horizontal(|ui| {
                    ok = ui.button("OK").clicked();
                    cancel = ui.button("Cancel").clicked();
                });
            });
        if ok {
            match form.apply(&self.model) {
                Ok(a) => {
                    self.apply_shells(a);
                    form.open = false;
                    form.error.clear();
                }
                Err(e) => form.error = e,
            }
        }
        if cancel || !open {
            form.open = false;
        }
        self.results.shells = form;
    }

    fn joint_readout_window(&mut self, ctx: &egui::Context) {
        let Some(joint) = self.results.readout_joint.clone() else {
            return;
        };
        if self.animate_deformed {
            // Refused while animating (the help of the Deformed Shape form).
            self.results.readout_joint = None;
            return;
        }
        let sel = StepSelection {
            mode_idx: self.active_mode_idx,
            use_step: self.use_time_step,
            step_idx: self.active_step_idx,
        };
        let units = self.results.readout_units.unwrap_or(self.model.present_units);
        let (header, lines) = readout_lines(&self.model, &self.active_case, &joint, sel, units);
        let mut open = true;
        egui::Window::new("Joint Displacements")
            .open(&mut open)
            .resizable(false)
            .show(ctx, |ui| {
                ui.label(header);
                egui::Grid::new("joint_readout_grid").show(ui, |ui| {
                    for (label, value) in &lines {
                        ui.label(label);
                        ui.monospace(value);
                        ui.end_row();
                    }
                });
            });
        if !open {
            self.results.readout_joint = None;
        }
    }
}
