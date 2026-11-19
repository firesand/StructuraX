//! The "Object Information" window (G07): a right-click on an object of the
//! undeformed display opens a floating window with the Loads tab of the object.
//!
//! Source: the help topics "Right Mouse Button" ("Right click on an object in a
//! display window to open a form with information about that object. The type
//! of information shown depends on what is being displayed in the window"),
//! "Point Information - Loads Tab", "Line Information Form", "Area Information
//! Form" and "Show Load Assigns" (pattern, load type, coordinate system,
//! direction). The window lists every load record the object stores, as the
//! model holds it, in the display units of the window (the model's units until
//! the user picks others). Nothing is changed by the window.
//!
//! Every displayed string is made by the pure functions of this file
//! ([`pick`], [`load_lines`], [`LoadLine::cells`], [`header`]).

use eframe::egui::{self, Pos2};
use sap_model::SapModel;
use sap_units::Units;

use super::display_units::{self, Dimension};
use super::{point_in_polygon, point_to_segment_dist};

/// Radius, in pixels, within which a right-click picks a joint.
pub(super) const JOINT_RADIUS: f32 = 10.0;
/// Distance, in pixels, within which a right-click picks a frame.
pub(super) const FRAME_RADIUS: f32 = 8.0;

/// The object a window describes.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) enum ObjectRef {
    Joint(String),
    Frame(String),
    Area(String),
}

impl ObjectRef {
    pub fn kind(&self) -> &'static str {
        match self {
            Self::Joint(_) => "Joint",
            Self::Frame(_) => "Frame",
            Self::Area(_) => "Area",
        }
    }

    pub fn name(&self) -> &str {
        match self {
            Self::Joint(n) | Self::Frame(n) | Self::Area(n) => n,
        }
    }
}

/// Window state; reset with the model.
#[derive(Clone, Debug, Default)]
pub(super) struct State {
    pub target: Option<ObjectRef>,
    /// Display units; the model's when `None`.
    pub units: Option<Units>,
}

/// The object under a right-click at `at`: the closest joint within
/// [`JOINT_RADIUS`], else the closest frame within [`FRAME_RADIUS`], else an
/// area whose polygon holds the point (the last one drawn, which lies on top).
pub(super) fn pick(
    joints: &[(String, Pos2)],
    frames: &[(String, Pos2, Pos2)],
    areas: &[(String, Vec<Pos2>)],
    at: Pos2,
) -> Option<ObjectRef> {
    if let Some((id, _)) = joints
        .iter()
        .map(|(id, p)| (id, p.distance(at)))
        .filter(|(_, d)| *d < JOINT_RADIUS)
        .min_by(|a, b| a.1.total_cmp(&b.1))
    {
        return Some(ObjectRef::Joint(id.clone()));
    }
    if let Some((id, _)) = frames
        .iter()
        .map(|(id, a, b)| (id, point_to_segment_dist(at, *a, *b)))
        .filter(|(_, d)| *d < FRAME_RADIUS)
        .min_by(|a, b| a.1.total_cmp(&b.1))
    {
        return Some(ObjectRef::Frame(id.clone()));
    }
    areas
        .iter()
        .rev()
        .find(|(_, poly)| poly.len() >= 3 && point_in_polygon(at, poly))
        .map(|(id, _)| ObjectRef::Area(id.clone()))
}

/// One load record of an object, as the table lists it.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct LoadLine {
    pub pattern: String,
    /// "Force", "Moment", "Displacement", "Gravity", "Temperature", "Surface Pressure", ...
    pub kind: String,
    pub csys: String,
    pub direction: String,
    /// The values, with units, and for a frame load the distances.
    pub values: String,
}

impl LoadLine {
    /// The cells of the table row, in the order of [`COLUMNS`].
    pub fn cells(&self) -> [String; 5] {
        [
            self.pattern.clone(),
            self.kind.clone(),
            self.csys.clone(),
            self.direction.clone(),
            self.values.clone(),
        ]
    }
}

pub(super) const COLUMNS: [&str; 5] =
    ["Load Pattern", "Load Type", "Coordinate System", "Direction", "Value"];

/// Direction code of a frame or area load (`FrameLoadDistributed::dir`).
pub(super) fn direction_name(dir: i32) -> String {
    match dir {
        1..=3 => format!("Local {dir}"),
        4 => "Global X".into(),
        5 => "Global Y".into(),
        6 => "Global Z".into(),
        7 => "Projected X".into(),
        8 => "Projected Y".into(),
        9 => "Projected Z".into(),
        10 => "Gravity".into(),
        11 => "Projected Gravity".into(),
        other => format!("Direction {other}"),
    }
}

fn face_name(face: i32) -> String {
    match face {
        -2 => "Top".into(),
        -1 => "Bottom".into(),
        1..=4 => format!("Edge {face}"),
        other => format!("Face {other}"),
    }
}

/// A number as the window prints it: at least six decimals and six
/// significant digits, no trailing zeros (six decimals alone would print a load
/// of 1e-7 MN as 0).
fn num(v: f64) -> String {
    if !v.is_finite() {
        return "-".into();
    }
    super::view::decimals_or_digits(v, 6, 6)
}

/// `value` of dimension `dim`, converted from `model` to `display` units, with
/// its unit label.
fn with_unit(value: f64, dim: Dimension, model: Units, display: Units) -> String {
    format!(
        "{} {}",
        num(value * display_units::factor(dim, model, display)),
        display_units::label(dim, display)
    )
}

fn temp_label(u: Units) -> &'static str {
    match u.temp_unit() {
        sap_units::TempUnit::F => "F",
        sap_units::TempUnit::C => "C",
    }
}

fn six(
    names: [&str; 6],
    v: &[f64; 6],
    model: Units,
    display: Units,
    translation: Dimension,
    rotation: Dimension,
) -> String {
    names
        .iter()
        .zip(v)
        .enumerate()
        .map(|(i, (n, x))| {
            let dim = if i < 3 { translation } else { rotation };
            format!("{n} = {}", with_unit(*x, dim, model, display))
        })
        .collect::<Vec<_>>()
        .join(", ")
}

/// The load records of `obj`, in the order the model stores them, with values
/// in `display` units; the reason when the object does not exist.
pub(super) fn load_lines(model: &SapModel, obj: &ObjectRef, display: Units) -> Result<Vec<LoadLine>, String> {
    let mu = model.present_units;
    let mut lines = Vec::new();
    match obj {
        ObjectRef::Joint(name) => {
            let p = model
                .points
                .get(name)
                .ok_or_else(|| format!("joint '{name}' does not exist"))?;
            for l in &p.loads_force {
                lines.push(LoadLine {
                    pattern: l.load_pat.clone(),
                    kind: "Force".into(),
                    csys: l.csys.clone(),
                    direction: "F1, F2, F3, M1, M2, M3".into(),
                    values: six(
                        ["F1", "F2", "F3", "M1", "M2", "M3"],
                        &l.force,
                        mu,
                        display,
                        Dimension::Force,
                        Dimension::Moment,
                    ),
                });
            }
            for l in &p.loads_displ {
                lines.push(LoadLine {
                    pattern: l.load_pat.clone(),
                    kind: "Displacement".into(),
                    csys: l.csys.clone(),
                    direction: "U1, U2, U3, R1, R2, R3".into(),
                    values: six(
                        ["U1", "U2", "U3", "R1", "R2", "R3"],
                        &l.displ,
                        mu,
                        display,
                        Dimension::Length,
                        Dimension::Rotation,
                    ),
                });
            }
        }
        ObjectRef::Frame(name) => {
            let f = model
                .frames
                .get(name)
                .ok_or_else(|| format!("frame '{name}' does not exist"))?;
            let length = match (model.points.get(&f.point_i), model.points.get(&f.point_j)) {
                (Some(a), Some(b)) => (b.x - a.x).hypot(b.y - a.y).hypot(b.z - a.z),
                _ => 0.0,
            };
            let at = |rel: f64| with_unit(rel * length, Dimension::Length, mu, display);
            for l in &f.loads_dist {
                let (dim, kind) = if l.my_type == 2 {
                    (Dimension::MomentPerLength, "Moment per length")
                } else {
                    (Dimension::ForcePerLength, "Force per length")
                };
                lines.push(LoadLine {
                    pattern: l.load_pat.clone(),
                    kind: kind.into(),
                    csys: l.csys.clone(),
                    direction: direction_name(l.dir),
                    values: format!(
                        "{} at {} to {} at {}",
                        with_unit(l.val1, dim, mu, display),
                        at(l.rel_dist1),
                        with_unit(l.val2, dim, mu, display),
                        at(l.rel_dist2)
                    ),
                });
            }
            for l in &f.loads_point {
                let (dim, kind) = if l.my_type == 2 {
                    (Dimension::Moment, "Point moment")
                } else {
                    (Dimension::Force, "Point force")
                };
                lines.push(LoadLine {
                    pattern: l.load_pat.clone(),
                    kind: kind.into(),
                    csys: l.csys.clone(),
                    direction: direction_name(l.dir),
                    values: format!("{} at {}", with_unit(l.val, dim, mu, display), at(l.rel_dist)),
                });
            }
            for l in &f.loads_temp {
                let (kind, dir) = match l.my_type {
                    1 => ("Temperature", "Uniform"),
                    2 => ("Temperature", "Gradient along local 2"),
                    _ => ("Temperature", "Gradient along local 3"),
                };
                let k = mu.temp_delta_factor_to(display);
                let value = if l.my_type == 1 {
                    format!("{} {}", num(l.val * k), temp_label(display))
                } else {
                    format!(
                        "{} {}/{}",
                        num(l.val * k / mu.length_factor_to(display)),
                        temp_label(display),
                        display.length_unit().symbol()
                    )
                };
                lines.push(LoadLine {
                    pattern: l.load_pat.clone(),
                    kind: kind.into(),
                    csys: "-".into(),
                    direction: dir.into(),
                    values: value,
                });
            }
            for (pat, g) in &f.loads_gravity {
                lines.push(LoadLine {
                    pattern: pat.clone(),
                    kind: "Gravity".into(),
                    csys: "GLOBAL".into(),
                    direction: "Global X, Y, Z".into(),
                    values: format!("multipliers X = {}, Y = {}, Z = {}", num(g[0]), num(g[1]), num(g[2])),
                });
            }
        }
        ObjectRef::Area(name) => {
            let a = model
                .areas
                .get(name)
                .ok_or_else(|| format!("area '{name}' does not exist"))?;
            for l in &a.loads_uniform {
                lines.push(LoadLine {
                    pattern: l.load_pat.clone(),
                    kind: "Uniform".into(),
                    csys: l.csys.clone(),
                    direction: direction_name(l.dir),
                    values: with_unit(l.val, Dimension::Stress, mu, display),
                });
            }
            for l in &a.loads_pressure {
                lines.push(LoadLine {
                    pattern: l.load_pat.clone(),
                    kind: "Surface Pressure".into(),
                    csys: "Local".into(),
                    direction: face_name(l.face),
                    values: with_unit(l.val, Dimension::Stress, mu, display),
                });
            }
            for l in &a.loads_temp {
                let k = mu.temp_delta_factor_to(display);
                let (dir, value) = if l.my_type == 1 {
                    ("Uniform", format!("{} {}", num(l.val * k), temp_label(display)))
                } else {
                    (
                        "Gradient through the thickness",
                        format!(
                            "{} {}/{}",
                            num(l.val * k / mu.length_factor_to(display)),
                            temp_label(display),
                            display.length_unit().symbol()
                        ),
                    )
                };
                lines.push(LoadLine {
                    pattern: l.load_pat.clone(),
                    kind: "Temperature".into(),
                    csys: "-".into(),
                    direction: dir.into(),
                    values: value,
                });
            }
        }
    }
    Ok(lines)
}

/// The header line of the window.
pub(super) fn header(obj: &ObjectRef) -> String {
    format!("{} {} - Loads", obj.kind(), obj.name())
}

/// What the table says when the object has no load.
pub(super) const NO_LOADS: &str = "No loads are assigned to this object.";

/// Draws the window while it has a target; closes it (drops the target) when the
/// user closes it or the object is gone.
pub(super) fn window(ctx: &egui::Context, state: &mut State, model: &SapModel) {
    let Some(target) = state.target.clone() else {
        return;
    };
    let display = state.units.unwrap_or(model.present_units);
    let lines = match load_lines(model, &target, display) {
        Ok(l) => l,
        Err(_) => {
            // Deleted or replaced since the click.
            state.target = None;
            return;
        }
    };
    let mut open = true;
    egui::Window::new("Object Information")
        .open(&mut open)
        .resizable(false)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                // The only tab of this window.
                let _ = ui.selectable_label(true, "Loads");
                ui.label(header(&target));
            });
            ui.horizontal(|ui| {
                ui.label("Units");
                egui::ComboBox::from_id_salt("object_info_units")
                    .selected_text(display.s2k_label())
                    .show_ui(ui, |ui| {
                        for code in 1..=16 {
                            if let Ok(u) = Units::from_code(code) {
                                ui.selectable_value(&mut state.units, Some(u), u.s2k_label());
                            }
                        }
                    });
            });
            ui.separator();
            if lines.is_empty() {
                ui.label(NO_LOADS);
                return;
            }
            egui::Grid::new("object_info_loads").striped(true).spacing([12.0, 4.0]).show(ui, |ui| {
                for c in COLUMNS {
                    ui.strong(c);
                }
                ui.end_row();
                for line in &lines {
                    for cell in line.cells() {
                        ui.label(cell);
                    }
                    ui.end_row();
                }
            });
        });
    if !open {
        state.target = None;
    }
}
