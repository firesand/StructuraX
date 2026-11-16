//! Contour colours, ranges, legend ticks and vertex-coloured meshes, and the
//! joint displacement components the deformed-shape display draws.
//!
//! Pure functions: the viewport only calls them. Joint results are stored in
//! each joint's local axes (CSI Analysis Reference Manual, p. 24); drawing in
//! the global system therefore needs [`joint_displacement_global`].

use eframe::egui::{self, Color32, Pos2};
use sap_model::{AreaObj, CaseResults, SapModel};
use sap_solver::area::area_local_system;
use sap_solver::frame::joint_local_to_global_matrix;
use sap_units::Units;

use super::display_units::{self, Dimension};

#[cfg(test)]
mod tests;

/// Colour of `val` on the ramp from `lo` (blue) to `hi` (red). Values outside
/// the range take the end colours; a non-finite input gives a neutral colour.
pub(super) fn value_color(val: f64, lo: f64, hi: f64, alpha: u8) -> Color32 {
    if !val.is_finite() || !lo.is_finite() || !hi.is_finite() {
        return Color32::from_rgba_unmultiplied(90, 110, 140, alpha);
    }
    let span = hi - lo;
    let t = if span.abs() > 1e-15 {
        ((val - lo) / span).clamp(0.0, 1.0) as f32
    } else {
        0.5
    };
    let (r, g, b) = if t < 0.25 {
        let s = t / 0.25;
        (40.0, 90.0 + 150.0 * s, 245.0)
    } else if t < 0.50 {
        let s = (t - 0.25) / 0.25;
        (40.0 + 40.0 * s, 240.0, 245.0 * (1.0 - s) + 90.0 * s)
    } else if t < 0.75 {
        let s = (t - 0.50) / 0.25;
        (80.0 + 175.0 * s, 240.0 - 20.0 * s, 90.0 * (1.0 - s))
    } else {
        let s = (t - 0.75) / 0.25;
        (255.0, 220.0 * (1.0 - 0.72 * s), 45.0 * (1.0 - s))
    };
    Color32::from_rgba_unmultiplied(r as u8, g as u8, b as u8, alpha)
}

/// Which stored displacement of a result the display shows.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub(super) struct StepSelection {
    /// Mode index (zero-based) of a modal or buckling-shape result.
    pub mode_idx: usize,
    /// Show the step `step_idx` of a multi-step result instead of its final state.
    pub use_step: bool,
    pub step_idx: usize,
}

/// Displacement `[U1, U2, U3, R1, R2, R3]` of `joint` in its local axes, as
/// stored: the selected mode shape of a modal result; the selected step of a
/// multi-step static or history result when `use_step` is set (falling back
/// to the final state); otherwise the final state.
pub(super) fn joint_displacement_local(
    cr: &CaseResults,
    joint: &str,
    sel: StepSelection,
) -> Option<[f64; 6]> {
    if !cr.modes.is_empty() {
        cr.modes
            .get(sel.mode_idx.min(cr.modes.len() - 1))
            .and_then(|m| m.mode_shape.get(joint))
            .copied()
    } else if sel.use_step && !cr.static_steps.is_empty() {
        cr.static_steps
            .get(sel.step_idx.min(cr.static_steps.len() - 1))
            .and_then(|st| st.joint_displacements.get(joint))
            .copied()
            .or_else(|| cr.joint_displacements.get(joint).copied())
    } else if sel.use_step && !cr.time_steps.is_empty() {
        cr.time_history_disp
            .get(joint)
            .and_then(|steps| {
                if steps.is_empty() {
                    None
                } else {
                    steps.get(sel.step_idx.min(steps.len() - 1)).copied()
                }
            })
            .or_else(|| cr.joint_displacements.get(joint).copied())
    } else {
        cr.joint_displacements.get(joint).copied()
    }
}

/// The same displacement as `[UX, UY, UZ, RX, RY, RZ]` in the global system.
///
/// Stored results are in the joint's local axes. `joint_local_to_global_matrix`
/// returns `R` with `v_global = R · v_local` (its columns are the joint's local
/// axes; the solver forms `e_axes · R` to go from joint-local to area-local), so
/// the translations and, the same way, the rotations are `R · u`. `None` when the
/// joint has no stored result or is not in the model.
pub(super) fn joint_displacement_global(
    model: &SapModel,
    cr: &CaseResults,
    joint: &str,
    sel: StepSelection,
) -> Option<[f64; 6]> {
    let local = joint_displacement_local(cr, joint, sel)?;
    let point = model.points.get(joint)?;
    let r = joint_local_to_global_matrix(point);
    let mut out = [0.0; 6];
    for block in 0..2 {
        for i in 0..3 {
            out[3 * block + i] = (0..3).map(|k| r[i][k] * local[3 * block + k]).sum();
        }
    }
    Some(out)
}

/// Displacement component a contour shows.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum DisplacementComponent {
    Ux,
    Uy,
    Uz,
    /// Along the local 1, 2 or 3 axis of the area being drawn.
    Area1,
    Area2,
    Area3,
}

/// Number of corner joints the solver reads for an area (three or four).
fn corner_count(area: &AreaObj) -> usize {
    area.points.len().min(4)
}

/// Local axes of an area as rows `[e1, e2, e3]` in global components: the
/// solver's own system, `area_local_system` with the area's angle plus the
/// material angle of its section (as `analyze_model` and `recover_area_result`
/// call it).
pub(super) fn area_local_axes(model: &SapModel, area: &str) -> Result<[[f64; 3]; 3], String> {
    let a = model
        .areas
        .get(area)
        .ok_or_else(|| format!("area '{area}' does not exist"))?;
    if a.points.len() < 3 {
        return Err(format!("area '{area}' has fewer than three joints"));
    }
    let sec = model
        .area_sections
        .get(&a.section)
        .ok_or_else(|| format!("area '{area}' uses the undefined section '{}'", a.section))?;
    let mut coords = Vec::new();
    for p in a.points.iter().take(corner_count(a)) {
        let pt = model
            .points
            .get(p)
            .ok_or_else(|| format!("area '{area}' refers to the missing joint '{p}'"))?;
        coords.push([pt.x, pt.y, pt.z]);
    }
    let (axes, _, _) = area_local_system(&coords, a.angle_deg + sec.mat_angle_deg);
    Ok(axes)
}

/// Value of `comp` at each corner joint of `area`, in corner order and model
/// units (the first three or four joints of the area; an auto-meshed area uses
/// its own corners). A corner joint without a result is an error naming it,
/// never a zero.
pub(super) fn area_vertex_values(
    model: &SapModel,
    cr: &CaseResults,
    area: &str,
    comp: DisplacementComponent,
    sel: StepSelection,
) -> Result<Vec<f64>, String> {
    let axes = match comp {
        DisplacementComponent::Area1
        | DisplacementComponent::Area2
        | DisplacementComponent::Area3 => Some(area_local_axes(model, area)?),
        _ => None,
    };
    let a = model
        .areas
        .get(area)
        .ok_or_else(|| format!("area '{area}' does not exist"))?;
    let mut out = Vec::new();
    for p in a.points.iter().take(corner_count(a)) {
        let u = joint_displacement_global(model, cr, p, sel).ok_or_else(|| {
            format!("area '{area}': corner joint '{p}' has no displacement result")
        })?;
        let dot = |e: &[f64; 3]| e[0] * u[0] + e[1] * u[1] + e[2] * u[2];
        out.push(match (comp, &axes) {
            (DisplacementComponent::Ux, _) => u[0],
            (DisplacementComponent::Uy, _) => u[1],
            (DisplacementComponent::Uz, _) => u[2],
            (DisplacementComponent::Area1, Some(e)) => dot(&e[0]),
            (DisplacementComponent::Area2, Some(e)) => dot(&e[1]),
            (_, Some(e)) => dot(&e[2]),
            (_, None) => unreachable!("area components always carry axes"),
        });
    }
    Ok(out)
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct ContourRange {
    pub lo: f64,
    pub hi: f64,
}

/// Contour range. `user == (0, 0)` asks for the automatic range over the
/// finite `values`; any other pair must satisfy `lo < hi` and is used as given.
/// `Ok(None)` when automatic and there is no finite value.
pub(super) fn contour_range(
    values: &[f64],
    user: (f64, f64),
) -> Result<Option<ContourRange>, String> {
    if user != (0.0, 0.0) {
        return if user.0 < user.1 && user.0.is_finite() && user.1.is_finite() {
            Ok(Some(ContourRange { lo: user.0, hi: user.1 }))
        } else {
            Err("the contour minimum must be below the maximum".into())
        };
    }
    let mut it = values.iter().copied().filter(|v| v.is_finite());
    let Some(first) = it.next() else {
        return Ok(None);
    };
    let (lo, hi) = it.fold((first, first), |(lo, hi), v| (lo.min(v), hi.max(v)));
    Ok(Some(ContourRange { lo, hi }))
}

/// Number of colour bands of a legend.
pub(super) const LEGEND_INTERVALS: usize = 10;

/// `intervals + 1` equally spaced tick values from `range.lo` to `range.hi`
/// (the last exactly `hi`); empty when `intervals` is zero.
pub(super) fn legend_ticks(range: ContourRange, intervals: usize) -> Vec<f64> {
    if intervals == 0 {
        return Vec::new();
    }
    (0..=intervals)
        .map(|k| {
            if k == intervals {
                range.hi
            } else {
                range.lo + (range.hi - range.lo) * k as f64 / intervals as f64
            }
        })
        .collect()
}

/// A mesh that fills the polygon `points` (three or four screen points, in
/// order around the polygon) with colours interpolated from the corner `values`,
/// each edge divided into `subdivisions` parts. Quads are bilinear, triangles
/// barycentric; the colour of a vertex is `value_color` of its interpolated
/// value. Empty for `subdivisions == 0`, other corner counts, or a `values`
/// slice of a different length than `points`.
pub(super) fn colored_polygon(
    points: &[Pos2],
    values: &[f64],
    range: ContourRange,
    alpha: u8,
    subdivisions: usize,
) -> egui::Mesh {
    let mut mesh = egui::Mesh::default();
    let s = subdivisions;
    let n = points.len();
    if s == 0 || !(n == 3 || n == 4) || values.len() != n {
        return mesh;
    }
    let mut push = |pos: Pos2, val: f64| {
        mesh.colored_vertex(pos, value_color(val, range.lo, range.hi, alpha));
    };
    if n == 4 {
        for j in 0..=s {
            let v = j as f32 / s as f32;
            let vd = j as f64 / s as f64;
            for i in 0..=s {
                let u = i as f32 / s as f32;
                let ud = i as f64 / s as f64;
                let w = [(1.0 - u) * (1.0 - v), u * (1.0 - v), u * v, (1.0 - u) * v];
                let wd = [
                    (1.0 - ud) * (1.0 - vd),
                    ud * (1.0 - vd),
                    ud * vd,
                    (1.0 - ud) * vd,
                ];
                let mut pos = Pos2::ZERO;
                let mut val = 0.0;
                for k in 0..4 {
                    pos += points[k].to_vec2() * w[k];
                    val += values[k] * wd[k];
                }
                push(pos, val);
            }
        }
        let idx = |i: usize, j: usize| (j * (s + 1) + i) as u32;
        for j in 0..s {
            for i in 0..s {
                mesh.add_triangle(idx(i, j), idx(i + 1, j), idx(i + 1, j + 1));
                mesh.add_triangle(idx(i, j), idx(i + 1, j + 1), idx(i, j + 1));
            }
        }
    } else {
        // Row j holds s + 1 - j vertices; vertex (i, j) has weights
        // (1 - i/s - j/s, i/s, j/s) on the corners 0, 1, 2.
        let offset = |j: usize| (0..j).map(|r| s + 1 - r).sum::<usize>();
        for j in 0..=s {
            for i in 0..=(s - j) {
                let w1 = i as f64 / s as f64;
                let w2 = j as f64 / s as f64;
                let w0 = 1.0 - w1 - w2;
                let wf = [w0 as f32, w1 as f32, w2 as f32];
                let mut pos = Pos2::ZERO;
                for k in 0..3 {
                    pos += points[k].to_vec2() * wf[k];
                }
                push(pos, w0 * values[0] + w1 * values[1] + w2 * values[2]);
            }
        }
        let idx = |i: usize, j: usize| (offset(j) + i) as u32;
        for j in 0..s {
            for i in 0..(s - j) {
                mesh.add_triangle(idx(i, j), idx(i + 1, j), idx(i, j + 1));
                if i + j + 1 < s {
                    mesh.add_triangle(idx(i + 1, j), idx(i + 1, j + 1), idx(i, j + 1));
                }
            }
        }
    }
    mesh
}

/// What a right-click on a joint shows.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct JointReadout {
    pub joint: String,
    /// `U1 [mm]` and so on, one per component.
    pub labels: [String; 6],
    /// Joint-local components in the display units.
    pub values: [f64; 6],
}

/// Joint-local `U1..R3` of `joint` in result `result` (Analysis Reference p. 24):
/// translations converted from the model's units to `display`, rotations in rad.
/// An unknown result or joint, or a joint without a result, is an error.
pub(super) fn joint_readout(
    model: &SapModel,
    result: &str,
    joint: &str,
    sel: StepSelection,
    display: Units,
) -> Result<JointReadout, String> {
    let cr = model
        .case_results
        .get(result)
        .ok_or_else(|| format!("no results for '{result}'"))?;
    if !model.points.contains_key(joint) {
        return Err(format!("joint '{joint}' does not exist"));
    }
    let local = joint_displacement_local(cr, joint, sel)
        .ok_or_else(|| format!("joint '{joint}' has no displacement result in '{result}'"))?;
    let f = display_units::factor(Dimension::Length, model.present_units, display);
    let len = display_units::label(Dimension::Length, display);
    let rot = display_units::label(Dimension::Rotation, display);
    let names = ["U1", "U2", "U3", "R1", "R2", "R3"];
    let labels =
        std::array::from_fn(|i| format!("{} [{}]", names[i], if i < 3 { &len } else { &rot }));
    let values = std::array::from_fn(|i| if i < 3 { local[i] * f } else { local[i] });
    Ok(JointReadout { joint: joint.to_string(), labels, values })
}

/// Number of steps a result can be animated through (history output steps or
/// saved static steps); zero for a single-state result (including one saved
/// static step) and for modal results.
pub(super) fn step_count(cr: &CaseResults) -> usize {
    if !cr.modes.is_empty() {
        return 0;
    }
    let n = if !cr.static_steps.is_empty() {
        cr.static_steps.len()
    } else if !cr.time_history_disp.is_empty() {
        cr.time_steps.len()
    } else {
        0
    };
    if n >= 2 {
        n
    } else {
        0
    }
}

/// Step shown `elapsed_s` seconds into an animation that loops over
/// `n_steps` steps at `seconds_per_step` each: `floor(elapsed / period) mod n`.
/// Zero when there is nothing to loop over or the times are not positive finite.
pub(super) fn animation_step(elapsed_s: f64, n_steps: usize, seconds_per_step: f64) -> usize {
    if n_steps == 0
        || !elapsed_s.is_finite()
        || elapsed_s < 0.0
        || !seconds_per_step.is_finite()
        || seconds_per_step <= 0.0
    {
        return 0;
    }
    let k = (elapsed_s / seconds_per_step).floor();
    (k % n_steps as f64) as usize
}
