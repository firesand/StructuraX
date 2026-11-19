//! View geometry shared by the viewport, the drawing tools and window
//! selection: the projection of a pane, plane views (a plan at a level or an
//! elevation at a grid line) and the objects that lie in such a plane.
//!
//! Source for plane views: SAP2000 help, "Set 2-D View" (choose the X-Y, X-Z
//! or Y-Z plane and the coordinate of the plane; the arrows step to the next
//! plane). Only the front face of a plane is shown.

use super::ViewProjection;
use eframe::egui::{Color32, Pos2, Shape, Stroke};
use sap_model::SapModel;
use std::collections::BTreeSet;

/// A global axis, as the normal of a view plane.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum PlaneAxis {
    X,
    Y,
    Z,
}

impl PlaneAxis {
    pub(super) fn index(self) -> usize {
        match self {
            Self::X => 0,
            Self::Y => 1,
            Self::Z => 2,
        }
    }

    pub(super) fn label(self) -> &'static str {
        match self {
            Self::X => "X",
            Self::Y => "Y",
            Self::Z => "Z",
        }
    }
}

/// A view plane normal to a global axis at a coordinate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct PlaneView {
    pub normal: PlaneAxis,
    pub coordinate: f64,
}

impl PlaneView {
    /// The plane of a 2D projection at `coordinate`; `None` for the 3D view
    /// and for a 2D view of the whole model (`coordinate` is `None`).
    pub(super) fn of(proj: ViewProjection, coordinate: Option<f64>) -> Option<Self> {
        let normal = match proj {
            ViewProjection::Perspective3D => return None,
            ViewProjection::PlanXY => PlaneAxis::Z,
            ViewProjection::ElevationXZ => PlaneAxis::Y,
            ViewProjection::ElevationYZ => PlaneAxis::X,
        };
        coordinate
            .filter(|c| c.is_finite())
            .map(|coordinate| Self { normal, coordinate })
    }

    pub(super) fn contains(&self, p: [f64; 3], tol: f64) -> bool {
        (p[self.normal.index()] - self.coordinate).abs() <= tol
    }

    /// Indices of the horizontal and the vertical screen axis of the plane.
    pub(super) fn in_plane_axes(&self) -> (usize, usize) {
        match self.normal {
            PlaneAxis::Z => (0, 1),
            PlaneAxis::Y => (0, 2),
            PlaneAxis::X => (1, 2),
        }
    }

    /// For example `X-Z Plane @ Y=5`.
    pub(super) fn title(&self) -> String {
        let (a, b) = match self.normal {
            PlaneAxis::Z => ("X", "Y"),
            PlaneAxis::Y => ("X", "Z"),
            PlaneAxis::X => ("Y", "Z"),
        };
        format!(
            "{a}-{b} Plane @ {}={}",
            self.normal.label(),
            number_text(self.coordinate)
        )
    }
}

/// Significant digits of a plane coordinate or a converted length in a title
/// or a text field (the sums of a grid spacing, `3.2 * 3`, carry roundoff).
pub(super) const NUMBER_DIGITS: i32 = 10;

/// `v` with at most [`NUMBER_DIGITS`] significant digits and no trailing zeros:
/// `9.600000000000001` is `9.6`, `6` is `6`, `-0.0` is `0`.
pub(super) fn number_text(v: f64) -> String {
    significant_text(v, NUMBER_DIGITS)
}

/// A quantity of the explorer, the tables and the labels of the drawing: at
/// least [`QUANTITY_DECIMALS`] decimals and at least [`QUANTITY_DIGITS`]
/// significant digits, no trailing zeros. A fixed number of decimals makes a
/// number meaningless after a change of units: the same section reads
/// `I33=0.0` in kN, m and `I33=7931.0` in kip, in.
pub(super) fn quantity_text(v: f64) -> String {
    decimals_or_digits(v, QUANTITY_DECIMALS, QUANTITY_DIGITS)
}

/// Decimals every printed quantity keeps (a value of the size of a force in
/// kN or a length in m has its habitual digits).
pub(super) const QUANTITY_DECIMALS: i32 = 4;
/// Significant digits every printed quantity keeps (a small value in a large
/// unit: a moment of inertia in m4, a load in MN).
pub(super) const QUANTITY_DIGITS: i32 = 5;

/// `v` with at least `min_decimals` decimals and at least `min_digits`
/// significant digits, no trailing zeros; scientific notation (with
/// `min_digits` digits) for a magnitude below 1e-6 or from 1e15.
pub(super) fn decimals_or_digits(v: f64, min_decimals: i32, min_digits: i32) -> String {
    if !v.is_finite() {
        return v.to_string();
    }
    if v == 0.0 {
        return "0".into();
    }
    let magnitude = v.abs().log10().floor() as i32;
    if !(-6..15).contains(&magnitude) {
        return significant_text(v, min_digits);
    }
    let decimals = min_decimals.max(min_digits - 1 - magnitude) as usize;
    let text = format!("{v:.decimals$}");
    let text = if text.contains('.') {
        text.trim_end_matches('0').trim_end_matches('.').to_string()
    } else {
        text
    };
    if text == "-0" { "0".into() } else { text }
}

/// `v`, or zero when it is below a billionth of `peak` (the roundoff of a
/// quantity that is exactly zero, such as the V3 of a plane frame).
pub(super) fn without_roundoff(v: f64, peak: f64) -> f64 {
    if v.abs() < 1e-9 * peak { 0.0 } else { v }
}

/// `v` with at most `digits` significant digits and no trailing zeros; a value
/// whose magnitude is below 1e-6 or at least `10^(digits + 2)` is written in
/// scientific notation (`1.5e-9`).
pub(super) fn significant_text(v: f64, digits: i32) -> String {
    if !v.is_finite() {
        return v.to_string();
    }
    if v == 0.0 {
        return "0".into();
    }
    let magnitude = v.abs().log10().floor() as i32;
    if !(-6..digits + 2).contains(&magnitude) {
        let text = format!("{:.*e}", (digits - 1) as usize, v);
        // "1.500000000e7" -> "1.5e7"
        let (mantissa, exponent) = text.split_once('e').unwrap_or((&text, "0"));
        let mantissa = if mantissa.contains('.') {
            mantissa.trim_end_matches('0').trim_end_matches('.')
        } else {
            mantissa
        };
        return format!("{mantissa}e{exponent}");
    }
    let decimals = (digits - 1 - magnitude).max(0) as usize;
    let text = format!("{v:.decimals$}");
    let text = if text.contains('.') {
        text.trim_end_matches('0').trim_end_matches('.').to_string()
    } else {
        text
    };
    if text == "-0" { "0".into() } else { text }
}

/// The projection of one viewport pane in one frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct ViewTransform {
    pub proj: ViewProjection,
    /// Model point shown at `screen_center`.
    pub center: [f64; 3],
    /// Pixels per model length. The viewport stores the `f32` scale it
    /// computed, widened, so that the projection rounds as it always did.
    pub scale: f64,
    pub screen_center: Pos2,
    pub yaw: f32,
    pub pitch: f32,
}

impl ViewTransform {
    /// Screen position of a model point.
    pub(super) fn project(&self, p: [f64; 3]) -> Pos2 {
        if !p[0].is_finite() || !p[1].is_finite() || !p[2].is_finite() {
            return self.screen_center;
        }
        let dx = ((p[0] - self.center[0]) * self.scale).clamp(-50_000.0, 50_000.0) as f32;
        let dy = ((p[1] - self.center[1]) * self.scale).clamp(-50_000.0, 50_000.0) as f32;
        let dz = ((p[2] - self.center[2]) * self.scale).clamp(-50_000.0, 50_000.0) as f32;
        let (sx, sy) = match self.proj {
            ViewProjection::Perspective3D => {
                let (syaw, cyaw) = self.yaw.sin_cos();
                let (sp, cp) = self.pitch.sin_cos();
                let x1 = dx * cyaw - dy * syaw;
                let y1 = dx * syaw + dy * cyaw;
                // Screen up is `z cos(pitch) + y1 sin(pitch)`: a positive pitch
                // looks down on the model, and at 90 degrees with zero yaw the
                // picture is the plan view (x right, y up). The eye is then at
                // `right x up = (-sin yaw cos pitch, -cos yaw cos pitch, sin pitch)`.
                let z2 = dz * cp + y1 * sp;
                (x1, -z2)
            }
            ViewProjection::ElevationXZ => (dx, -dz),
            ViewProjection::PlanXY => (dx, -dy),
            ViewProjection::ElevationYZ => (dy, -dz),
        };
        Pos2::new(self.screen_center.x + sx, self.screen_center.y + sy)
    }

    /// Model point of a screen position on `plane`. `None` in the 3D view, for
    /// a plane that is not the plane of this projection, and for a degenerate
    /// scale. Screen positions are `f32`, so the result is not exact: a point
    /// snapped to a joint or a grid line must take that object's stored
    /// coordinate, not this value.
    pub(super) fn unproject(&self, s: Pos2, plane: PlaneView) -> Option<[f64; 3]> {
        if PlaneView::of(self.proj, Some(plane.coordinate))?.normal != plane.normal {
            return None;
        }
        if !(self.scale.is_finite() && self.scale > 0.0) {
            return None;
        }
        let (h, v) = plane.in_plane_axes();
        let mut p = [0.0; 3];
        p[plane.normal.index()] = plane.coordinate;
        p[h] = self.center[h] + f64::from(s.x - self.screen_center.x) / self.scale;
        p[v] = self.center[v] - f64::from(s.y - self.screen_center.y) / self.scale;
        p.iter().all(|c| c.is_finite()).then_some(p)
    }
}

/// Tolerance for "lies in the view plane": the joint-merge tolerance of the
/// model commands, `1e-9 * max(1, largest |coordinate|)`.
pub(super) fn plane_tolerance(model: &SapModel) -> f64 {
    sap_model::generate::geometry_tolerance(model, &[])
}

fn sorted_distinct(mut values: Vec<f64>, tol: f64) -> Vec<f64> {
    values.retain(|v| v.is_finite());
    values.sort_by(f64::total_cmp);
    let mut out: Vec<f64> = Vec::with_capacity(values.len());
    for v in values {
        if out.last().is_none_or(|last| v - last > tol) {
            out.push(v);
        }
    }
    out
}

/// Coordinates of the GLOBAL grid lines of one axis, increasing, without
/// repeats (within the plane tolerance). The values are the stored ones.
pub(super) fn grid_coordinates(model: &SapModel, axis: PlaneAxis) -> Vec<f64> {
    let values = model
        .coord_systems
        .get("GLOBAL")
        .map(|cs| {
            cs.grids
                .iter()
                .filter(|g| g.axis.eq_ignore_ascii_case(axis.label()))
                .map(|g| g.coordinate)
                .collect()
        })
        .unwrap_or_default();
    sorted_distinct(values, plane_tolerance(model))
}

/// The planes a view can step through along `axis`: the grid coordinates, or
/// the distinct joint coordinates when that axis has no grid line.
pub(super) fn plane_levels(model: &SapModel, axis: PlaneAxis) -> Vec<f64> {
    let grid = grid_coordinates(model, axis);
    if !grid.is_empty() {
        return grid;
    }
    let values = model
        .points
        .values()
        .map(|p| [p.x, p.y, p.z][axis.index()])
        .collect();
    sorted_distinct(values, plane_tolerance(model))
}

/// The global plane that holds every joint of the model, when there is one:
/// X-Z at the common Y first, then X-Y at the common Z, then Y-Z at the common
/// X. `None` for a model with fewer than two joints or joints in no one plane.
pub(super) fn joint_plane(model: &SapModel) -> Option<PlaneView> {
    if model.points.len() < 2 {
        return None;
    }
    let tol = plane_tolerance(model);
    for normal in [PlaneAxis::Y, PlaneAxis::Z, PlaneAxis::X] {
        let k = normal.index();
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for p in model.points.values() {
            let c = [p.x, p.y, p.z][k];
            lo = lo.min(c);
            hi = hi.max(c);
        }
        if hi - lo <= tol {
            return Some(PlaneView { normal, coordinate: lo });
        }
    }
    None
}

/// The next plane above (`up`) or below `current` in `levels` (increasing).
/// From no plane, up gives the first level and down the last. `None` when
/// there is no further level.
pub(super) fn step_level(levels: &[f64], current: Option<f64>, up: bool, tol: f64) -> Option<f64> {
    match (current, up) {
        (None, true) => levels.first().copied(),
        (None, false) => levels.last().copied(),
        (Some(c), true) => levels.iter().copied().find(|l| *l > c + tol),
        (Some(c), false) => levels.iter().rev().copied().find(|l| *l < c - tol),
    }
}

/// Corner indices of the six faces of a solid object (D121). The corner order of
/// the model is binary: corner k has bit 0 = xi1, bit 1 = xi2, bit 2 = xi3
/// (Analysis Reference, Figure 55; `sap-solver/src/solid.rs`). Each face holds the
/// four corners that share one bit, in the order of the solver's face list
/// (`solid_face_nodes`): faces 1 to 6 are -2, +1, +2, -1, -3, +3. One constant for
/// drawing, plane outlines and window picking.
pub(super) const SOLID_FACES: [[usize; 4]; 6] = [
    [0, 1, 5, 4],
    [1, 3, 7, 5],
    [2, 6, 7, 3],
    [0, 4, 6, 2],
    [0, 2, 3, 1],
    [4, 5, 7, 6],
];

/// The objects that lie in a view plane.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub(super) struct Members {
    pub points: BTreeSet<String>,
    pub frames: BTreeSet<String>,
    pub links: BTreeSet<String>,
    pub areas: BTreeSet<String>,
    /// Solid name and index into [`SOLID_FACES`] of each face in the plane.
    pub solid_faces: Vec<(String, usize)>,
}

/// Joints in the plane, frames and links with every end joint in it, areas
/// with every corner in it, and solid faces with their four corners in it.
/// An object that names a missing joint is not a member.
pub(super) fn plane_members(model: &SapModel, plane: PlaneView, tol: f64) -> Members {
    let mut m = Members::default();
    for (name, p) in &model.points {
        if plane.contains([p.x, p.y, p.z], tol) {
            m.points.insert(name.clone());
        }
    }
    for (name, f) in &model.frames {
        if m.points.contains(&f.point_i) && m.points.contains(&f.point_j) {
            m.frames.insert(name.clone());
        }
    }
    for (name, l) in &model.links {
        if m.points.contains(&l.point_i) && l.point_j.as_ref().is_none_or(|j| m.points.contains(j))
        {
            m.links.insert(name.clone());
        }
    }
    for (name, a) in &model.areas {
        if !a.points.is_empty() && a.points.iter().all(|p| m.points.contains(p)) {
            m.areas.insert(name.clone());
        }
    }
    for (name, s) in &model.solids {
        for (k, face) in SOLID_FACES.iter().enumerate() {
            if face.iter().all(|c| m.points.contains(&s.points[*c])) {
                m.solid_faces.push((name.clone(), k));
            }
        }
    }
    m
}

/// Bounding box of the model for the viewport: the joints and the GLOBAL
/// grid lines of each axis, so that a model of grid lines only can be shown.
/// An axis with neither joints nor grid lines collapses to 0; the box
/// `[-100, -100, 0]..[100, 100, 100]` is used only when the model has no joint
/// and no grid line at all.
pub(super) fn model_bounds(model: &SapModel) -> ([f64; 3], [f64; 3]) {
    let mut min_p = [f64::INFINITY; 3];
    let mut max_p = [f64::NEG_INFINITY; 3];
    let mut any = false;
    let mut take = |axis: usize, v: f64| {
        if v.is_finite() {
            min_p[axis] = min_p[axis].min(v);
            max_p[axis] = max_p[axis].max(v);
            any = true;
        }
    };
    for pt in model.points.values() {
        if pt.x.is_finite() && pt.y.is_finite() && pt.z.is_finite() {
            take(0, pt.x);
            take(1, pt.y);
            take(2, pt.z);
        }
    }
    if let Some(cs) = model.coord_systems.get("GLOBAL") {
        for g in &cs.grids {
            for (k, axis) in [PlaneAxis::X, PlaneAxis::Y, PlaneAxis::Z].iter().enumerate() {
                if g.axis.eq_ignore_ascii_case(axis.label()) {
                    take(k, g.coordinate);
                }
            }
        }
    }
    if !any {
        return ([-100.0, -100.0, 0.0], [100.0, 100.0, 100.0]);
    }
    for k in 0..3 {
        if !min_p[k].is_finite() {
            min_p[k] = 0.0;
            max_p[k] = 0.0;
        }
    }
    (min_p, max_p)
}

/// One grid line of a plane view, with the id shown in its bubble.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct GridSegment {
    pub id: String,
    pub a: [f64; 3],
    pub b: [f64; 3],
    /// The bubble sits at `a` when true, at `b` otherwise.
    pub bubble_at_a: bool,
}

/// The visible GLOBAL grid lines of both in-plane axes of `plane`, at the
/// plane. A line of one axis spans the grid lines of the other in-plane axis
/// (lowest to highest); when that axis has fewer than two lines it spans
/// `bounds` along it.
pub(super) fn plane_grid_segments(
    model: &SapModel,
    plane: PlaneView,
    bounds: ([f64; 3], [f64; 3]),
) -> Vec<GridSegment> {
    let Some(cs) = model.coord_systems.get("GLOBAL") else {
        return Vec::new();
    };
    let (h, v) = plane.in_plane_axes();
    let axes = [PlaneAxis::X, PlaneAxis::Y, PlaneAxis::Z];
    let extent = |k: usize| -> (f64, f64) {
        let coords: Vec<f64> = cs
            .grids
            .iter()
            .filter(|g| g.axis.eq_ignore_ascii_case(axes[k].label()) && g.coordinate.is_finite())
            .map(|g| g.coordinate)
            .collect();
        let lo = coords.iter().copied().fold(f64::INFINITY, f64::min);
        let hi = coords.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        if coords.len() >= 2 && hi > lo {
            (lo, hi)
        } else {
            (bounds.0[k], bounds.1[k])
        }
    };
    let mut out = Vec::new();
    for g in &cs.grids {
        if !g.visible || !g.coordinate.is_finite() {
            continue;
        }
        let (along, across) = if g.axis.eq_ignore_ascii_case(axes[h].label()) {
            (h, v)
        } else if g.axis.eq_ignore_ascii_case(axes[v].label()) {
            (v, h)
        } else {
            continue;
        };
        let (lo, hi) = extent(across);
        let mut a = [0.0; 3];
        a[plane.normal.index()] = plane.coordinate;
        a[along] = g.coordinate;
        let mut b = a;
        a[across] = lo;
        b[across] = hi;
        out.push(GridSegment {
            id: g.id.clone(),
            a,
            b,
            bubble_at_a: g.bubble_loc.eq_ignore_ascii_case("Start"),
        });
    }
    out
}

/// Input of the "Set 2D View" dialog.
#[derive(Debug, Clone)]
pub(super) struct Set2dForm {
    pub open: bool,
    /// 0: left pane, 1: right pane.
    pub pane: usize,
    pub normal: PlaneAxis,
    pub coordinate_text: String,
}

impl Default for Set2dForm {
    fn default() -> Self {
        Self {
            open: false,
            pane: 1,
            normal: PlaneAxis::Y,
            coordinate_text: "0".into(),
        }
    }
}

impl PlaneAxis {
    /// The 2D projection whose view plane is normal to this axis.
    pub(super) fn projection(self) -> ViewProjection {
        match self {
            Self::Z => ViewProjection::PlanXY,
            Self::Y => ViewProjection::ElevationXZ,
            Self::X => ViewProjection::ElevationYZ,
        }
    }
}

/// A filled polygon with an outline, as shapes that cannot spike.
///
/// `Shape::convex_polygon` strokes a closed path with mitred corners and
/// feathers its fill along corner normals; both grow without limit where an
/// edge turns back on itself, which a foreshortened or deformed sliver of an
/// area does (a quadrilateral seen edge-on). The tessellated stroke then
/// reaches across the whole view (D41). Here the fill is a plain fan without
/// outline and every edge is a segment of its own; a corner is a butt join.
/// A polygon with fewer than three corners draws its edges only.
pub(super) fn outlined_polygon(points: &[Pos2], fill: Color32, stroke: Stroke) -> Vec<Shape> {
    let mut shapes = Vec::with_capacity(points.len() + 1);
    let finite = |p: &Pos2| p.x.is_finite() && p.y.is_finite();
    if points.len() >= 3 && fill.a() > 0 && points.iter().all(finite) {
        shapes.push(Shape::Mesh(fan(points, fill)));
    }
    if stroke.width > 0.0 && stroke.color.a() > 0 && points.len() >= 2 {
        let edges = if points.len() == 2 { 1 } else { points.len() };
        for k in 0..edges {
            let (a, b) = (points[k], points[(k + 1) % points.len()]);
            if a != b && finite(&a) && finite(&b) {
                shapes.push(Shape::line_segment([a, b], stroke));
            }
        }
    }
    shapes
}

/// The triangle fan of a polygon from its first corner (exact for a convex
/// polygon, the fill the viewport always used for areas). No anti-aliasing
/// fringe: that is what grows spikes.
fn fan(points: &[Pos2], fill: Color32) -> eframe::egui::epaint::Mesh {
    let mut mesh = eframe::egui::epaint::Mesh::default();
    for p in points {
        mesh.colored_vertex(*p, fill);
    }
    for k in 1..points.len() as u32 - 1 {
        mesh.add_triangle(0, k, k + 1);
    }
    mesh
}

/// The fill colour of a shape made by `outlined_polygon` for its fill (a
/// fan of one colour from the first corner), for the tests that read the
/// drawn shapes.
#[cfg(test)]
pub(super) fn polygon_fill_of(shape: &Shape) -> Option<Color32> {
    let Shape::Mesh(m) = shape else {
        return None;
    };
    let first = m.vertices.first()?;
    let n = m.vertices.len();
    let fan_like = n >= 3
        && m.indices.len() == 3 * (n - 2)
        && m.indices.chunks(3).all(|t| t[0] == 0)
        && m.vertices.iter().all(|v| v.color == first.color);
    fan_like.then_some(first.color)
}

#[cfg(test)]
mod tests {
    use super::*;
    use sap_units::Units;

    /// A unit cube whose corner k sits at (bit 0, bit 1, bit 2) of k, the
    /// binary corner order of the solid element (Analysis Reference, Figure 55).
    fn unit_cube_model() -> SapModel {
        let mut m = SapModel::new(Units::KnMC);
        for k in 0..8usize {
            m.add_point(
                format!("P{k}"),
                (k & 1) as f64,
                ((k >> 1) & 1) as f64,
                ((k >> 2) & 1) as f64,
            );
        }
        let points: [String; 8] = std::array::from_fn(|k| format!("P{k}"));
        m.solids.insert(
            "S1".to_string(),
            sap_model::SolidObj {
                name: "S1".to_string(),
                points,
                section: "SOLID1".to_string(),
                loads_surface: Vec::new(),
                loads_temp: Default::default(),
                selected: false,
            },
        );
        m
    }

    #[test]
    fn each_solid_face_holds_the_four_corners_that_share_one_bit() {
        // Hand check on the binary order: a face of the cube is the set of four
        // corners with one bit fixed, and consecutive corners of the loop
        // differ in exactly one bit (they are joined by an edge).
        let mut seen = BTreeSet::new();
        for face in SOLID_FACES {
            let fixed: Vec<(u32, bool)> = (0..3)
                .filter_map(|bit| {
                    let first = face[0] >> bit & 1;
                    face.iter()
                        .all(|c| c >> bit & 1 == first)
                        .then_some((bit as u32, first == 1))
                })
                .collect();
            assert_eq!(fixed.len(), 1, "face {face:?} must fix exactly one bit");
            seen.insert(fixed[0]);
            let distinct: BTreeSet<usize> = face.iter().copied().collect();
            assert_eq!(distinct.len(), 4, "{face:?}");
            for i in 0..4 {
                let differ = (face[i] ^ face[(i + 1) % 4]).count_ones();
                assert_eq!(differ, 1, "corners {} and {} of {face:?}", face[i], face[(i + 1) % 4]);
            }
        }
        // All six faces: each bit fixed at 0 and at 1.
        assert_eq!(seen.len(), 6);
    }

    #[test]
    fn a_plane_through_corners_0_2_4_6_finds_the_minus_1_face() {
        // Corners 0, 2, 4, 6 are the ones with bit 0 clear: the plane x = 0.
        let m = unit_cube_model();
        let plane = PlaneView {
            normal: PlaneAxis::X,
            coordinate: 0.0,
        };
        let members = plane_members(&m, plane, 1e-9);
        let faces: Vec<_> = members
            .solid_faces
            .iter()
            .map(|(name, k)| (name.as_str(), SOLID_FACES[*k]))
            .collect();
        // Before the repair no face of the list lay in this plane.
        assert_eq!(faces.len(), 1);
        assert_eq!(faces[0].0, "S1");
        let mut corners = faces[0].1;
        corners.sort_unstable();
        assert_eq!(corners, [0, 2, 4, 6]);
        // Negative: a plane that touches no corner finds none.
        let off = PlaneView {
            normal: PlaneAxis::X,
            coordinate: 0.5,
        };
        assert!(plane_members(&m, off, 1e-9).solid_faces.is_empty());
    }

    fn view_3d(yaw: f32, pitch: f32) -> ViewTransform {
        ViewTransform {
            proj: ViewProjection::Perspective3D,
            center: [0.0; 3],
            scale: 100.0,
            screen_center: Pos2::new(400.0, 300.0),
            yaw,
            pitch,
        }
    }

    #[test]
    fn the_3d_view_is_right_handed_and_looks_down_at_a_positive_pitch() {
        // Looking straight down with zero yaw is the plan view: x to the
        // right, y up the screen (screen y grows downward).
        let top = view_3d(0.0, std::f32::consts::FRAC_PI_2);
        let plan = ViewTransform {
            proj: ViewProjection::PlanXY,
            ..top
        };
        for p in [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.3, -0.7, 0.0]] {
            let (a, b) = (top.project(p), plan.project(p));
            assert!((a.x - b.x).abs() < 1e-3 && (a.y - b.y).abs() < 1e-3, "{p:?}: {a:?} {b:?}");
        }
        // Screen right and up as model vectors; right x up points at the eye.
        // At the default yaw and pitch the eye is above the model (z > 0), and
        // a point farther from the eye along the ground is higher on screen.
        for (yaw, pitch) in [(-0.65f32, 0.42f32), (0.0, 0.42), (1.1, 0.9), (2.5, 0.1)] {
            let v = view_3d(yaw, pitch);
            let o = v.project([0.0; 3]);
            let axis = |p: [f64; 3]| {
                let s = v.project(p);
                // (right, up) components on screen of a unit model vector
                [f64::from(s.x - o.x) / 100.0, f64::from(o.y - s.y) / 100.0]
            };
            let (ex, ey, ez) = (axis([1.0, 0.0, 0.0]), axis([0.0, 1.0, 0.0]), axis([0.0, 0.0, 1.0]));
            let right = [ex[0], ey[0], ez[0]];
            let up = [ex[1], ey[1], ez[1]];
            let eye = [
                right[1] * up[2] - right[2] * up[1],
                right[2] * up[0] - right[0] * up[2],
                right[0] * up[1] - right[1] * up[0],
            ];
            let (sy, cy) = f64::from(yaw).sin_cos();
            let (sp, cp) = f64::from(pitch).sin_cos();
            let want = [-sy * cp, -cy * cp, sp];
            for k in 0..3 {
                assert!((eye[k] - want[k]).abs() < 1e-5, "yaw {yaw} pitch {pitch}: {eye:?} {want:?}");
            }
            assert!(eye[2] > 0.0, "a positive pitch looks from above");
            // +Z is up the screen.
            assert!(ez[1] > 0.0);
        }
    }

    #[test]
    fn bounds_include_grid_lines_and_fall_back_per_axis() {
        // Grid lines only: x 0..10, y 0..5, z 0..6.4.
        let mut grid_only = SapModel::new(Units::KnMC);
        grid_only
            .quick_grid([3, 2, 3], [5.0, 5.0, 3.2], [0.0; 3])
            .unwrap();
        assert_eq!(
            model_bounds(&grid_only),
            ([0.0, 0.0, 0.0], [10.0, 5.0, 6.4])
        );
        // A joint outside the grid widens the box.
        grid_only.add_point("1", -2.0, 1.0, 9.0);
        assert_eq!(
            model_bounds(&grid_only),
            ([-2.0, 0.0, 0.0], [10.0, 5.0, 9.0])
        );
        // Nothing at all keeps the old default box.
        let empty = SapModel::new(Units::KnMC);
        assert_eq!(
            model_bounds(&empty),
            ([-100.0, -100.0, 0.0], [100.0, 100.0, 100.0])
        );
    }

    #[test]
    fn a_plane_view_draws_the_grid_lines_of_both_in_plane_axes() {
        let mut model = SapModel::new(Units::KnMC);
        model
            .quick_grid([3, 2, 3], [5.0, 4.0, 3.2], [0.0; 3])
            .unwrap();
        let b = model_bounds(&model);
        let plan = PlaneView::of(ViewProjection::PlanXY, Some(3.2)).unwrap();
        let segs = plane_grid_segments(&model, plan, b);
        let ids: Vec<&str> = segs.iter().map(|s| s.id.as_str()).collect();
        assert_eq!(ids, ["A", "B", "C", "1", "2"]);
        // "B" is the line x = 5 from y = 0 to y = 4 at z = 3.2; "2" is y = 4.
        assert_eq!(segs[1].a, [5.0, 0.0, 3.2]);
        assert_eq!(segs[1].b, [5.0, 4.0, 3.2]);
        assert_eq!(segs[4].a, [0.0, 4.0, 3.2]);
        assert_eq!(segs[4].b, [10.0, 4.0, 3.2]);
        assert!(!segs[0].bubble_at_a && segs[3].bubble_at_a);
        // Elevation X-Z at y = 4: the X lines span Z, the Z lines span X.
        let xz = PlaneView::of(ViewProjection::ElevationXZ, Some(4.0)).unwrap();
        let segs = plane_grid_segments(&model, xz, b);
        let ids: Vec<&str> = segs.iter().map(|s| s.id.as_str()).collect();
        assert_eq!(ids, ["A", "B", "C", "Z1", "Z2", "Z3"]);
        assert_eq!(segs[0].a, [0.0, 4.0, 0.0]);
        assert_eq!(segs[0].b, [0.0, 4.0, 6.4]);
        assert_eq!(segs[5].b, [10.0, 4.0, 6.4]);
        // Hidden lines are not drawn.
        model
            .coord_systems
            .get_mut("GLOBAL")
            .unwrap()
            .grids
            .iter_mut()
            .for_each(|g| g.visible = g.id != "B");
        assert_eq!(plane_grid_segments(&model, plan, b).len(), 4);
    }

    fn plan() -> ViewTransform {
        ViewTransform {
            proj: ViewProjection::PlanXY,
            center: [15.0, 7.5, 9.6],
            scale: 20.0,
            screen_center: Pos2::new(500.0, 400.0),
            yaw: 0.0,
            pitch: 0.0,
        }
    }

    #[test]
    fn projection_and_its_inverse_agree_on_each_plane() {
        // Plan: x = 0 is 15 m left of the centre, 300 px at 20 px/m; y grows
        // upward on the screen.
        let v = plan();
        assert_eq!(v.project([0.0, 0.0, 6.4]), Pos2::new(200.0, 550.0));
        let z = PlaneView::of(ViewProjection::PlanXY, Some(6.4)).unwrap();
        assert_eq!(v.unproject(Pos2::new(390.0, 450.0), z), Some([9.5, 5.0, 6.4]));
        // Elevation X-Z at y = 5: z = 0 is 9.6 m below the centre, 192 px.
        let xz = ViewTransform {
            proj: ViewProjection::ElevationXZ,
            ..v
        };
        assert_eq!(xz.project([6.0, 5.0, 0.0]), Pos2::new(320.0, 592.0));
        let y = PlaneView::of(ViewProjection::ElevationXZ, Some(5.0)).unwrap();
        assert_eq!(xz.unproject(Pos2::new(320.0, 592.0), y), Some([6.0, 5.0, 0.0]));
        // Elevation Y-Z at x = 12.
        let yz = ViewTransform {
            proj: ViewProjection::ElevationYZ,
            ..v
        };
        let x = PlaneView::of(ViewProjection::ElevationYZ, Some(12.0)).unwrap();
        let s = yz.project([12.0, 10.0, 3.2]);
        assert_eq!(s, Pos2::new(550.0, 528.0));
        let back = yz.unproject(s, x).unwrap();
        assert_eq!((back[0], back[1]), (12.0, 10.0));
        assert!((back[2] - 3.2).abs() < 1e-12);
        // A plane of another projection and the 3D view have no inverse.
        assert_eq!(v.unproject(Pos2::new(0.0, 0.0), y), None);
        let persp = ViewTransform {
            proj: ViewProjection::Perspective3D,
            ..v
        };
        assert_eq!(persp.unproject(Pos2::new(0.0, 0.0), z), None);
        assert_eq!(PlaneView::of(ViewProjection::Perspective3D, Some(0.0)), None);
        assert_eq!(PlaneView::of(ViewProjection::PlanXY, None), None);
        assert_eq!(y.title(), "X-Z Plane @ Y=5");
    }

    #[test]
    fn levels_step_through_the_grid_and_stop_at_its_ends() {
        let mut model = SapModel::new(Units::KnMC);
        model
            .quick_grid([2, 2, 3], [6.0, 5.0, 3.2], [0.0; 3])
            .unwrap();
        let levels = plane_levels(&model, PlaneAxis::Z);
        assert_eq!(levels, vec![0.0, 3.2, 6.4]);
        let tol = plane_tolerance(&model);
        assert_eq!(step_level(&levels, None, true, tol), Some(0.0));
        assert_eq!(step_level(&levels, None, false, tol), Some(6.4));
        assert_eq!(step_level(&levels, Some(3.2), true, tol), Some(6.4));
        assert_eq!(step_level(&levels, Some(3.2), false, tol), Some(0.0));
        assert_eq!(step_level(&levels, Some(6.4), true, tol), None);
        assert_eq!(step_level(&levels, Some(0.0), false, tol), None);
        // Without grid lines the joint coordinates are the levels.
        let mut bare = SapModel::new(Units::KnMC);
        bare.add_point("1", 0.0, 0.0, 4.0);
        bare.add_point("2", 0.0, 0.0, 1.0);
        bare.add_point("3", 3.0, 0.0, 4.0);
        assert_eq!(plane_levels(&bare, PlaneAxis::Z), vec![1.0, 4.0]);
    }

    #[test]
    fn plane_members_are_the_objects_with_every_joint_in_the_plane() {
        let mut model = SapModel::new(Units::KnMC);
        model.add_point("1", 0.0, 0.0, 0.0);
        model.add_point("2", 4.0, 0.0, 0.0);
        model.add_point("3", 4.0, 0.0, 3.0);
        model.add_point("4", 0.0, 0.0, 3.0);
        model.add_point("5", 0.0, 5.0, 3.0);
        model.add_frame("B", "4", "3", "FSEC1");
        model.add_frame("C", "1", "4", "FSEC1");
        model.add_frame("Y", "4", "5", "FSEC1");
        model.add_area("A", vec!["1".into(), "2".into(), "3".into(), "4".into()], "ASEC1");
        let tol = plane_tolerance(&model);
        let elevation = PlaneView::of(ViewProjection::ElevationXZ, Some(0.0)).unwrap();
        let m = plane_members(&model, elevation, tol);
        assert_eq!(m.points.len(), 4);
        assert_eq!(m.frames, BTreeSet::from(["B".to_string(), "C".to_string()]));
        assert_eq!(m.areas, BTreeSet::from(["A".to_string()]));
        let level = PlaneView::of(ViewProjection::PlanXY, Some(3.0)).unwrap();
        let m = plane_members(&model, level, tol);
        assert_eq!(m.frames, BTreeSet::from(["B".to_string(), "Y".to_string()]));
        assert!(m.areas.is_empty());
    }

    use eframe::egui;

    fn tessellated_vertices(shapes: Vec<Shape>) -> Vec<Pos2> {
        let ctx = egui::Context::default();
        let _ = ctx.run(egui::RawInput::default(), |_| {});
        let clip = egui::Rect::from_min_max(Pos2::new(-5000.0, -5000.0), Pos2::new(5000.0, 5000.0));
        let clipped = shapes
            .into_iter()
            .map(|shape| egui::epaint::ClippedShape { clip_rect: clip, shape })
            .collect();
        ctx.tessellate(clipped, 1.0)
            .iter()
            .filter_map(|p| match &p.primitive {
                egui::epaint::Primitive::Mesh(m) => Some(m.vertices.iter().map(|v| v.pos).collect::<Vec<_>>()),
                _ => None,
            })
            .flatten()
            .collect()
    }

    /// D41: edge-on quadrilaterals (the shell dome's top rings seen from the
    /// side, or a deformed sliver) draw inside their own bounding box.
    #[test]
    fn an_outlined_polygon_stays_inside_its_corners_even_when_an_edge_turns_back() {
        let slivers: [&[Pos2]; 3] = [
            // The quad of the dome's first ring in the 3D pane (pixels).
            &[Pos2::new(634.2, 452.2), Pos2::new(649.4, 456.8), Pos2::new(579.4, 435.0), Pos2::new(577.3, 434.6)],
            // A triangle whose third corner lies on the first edge.
            &[Pos2::new(100.0, 100.0), Pos2::new(200.0, 120.0), Pos2::new(150.0, 110.0)],
            // Coincident corners.
            &[Pos2::new(10.0, 10.0), Pos2::new(10.0, 10.0), Pos2::new(60.0, 10.5), Pos2::new(60.0, 10.5)],
        ];
        for poly in slivers {
            let (mut lo, mut hi) = (Pos2::new(f32::MAX, f32::MAX), Pos2::new(f32::MIN, f32::MIN));
            for p in poly {
                lo = Pos2::new(lo.x.min(p.x), lo.y.min(p.y));
                hi = Pos2::new(hi.x.max(p.x), hi.y.max(p.y));
            }
            let shapes = outlined_polygon(poly, Color32::from_rgba_unmultiplied(60, 120, 180, 55), Stroke::new(1.2_f32, Color32::WHITE));
            let vertices = tessellated_vertices(shapes);
            assert!(!vertices.is_empty());
            for v in &vertices {
                assert!(
                    v.x >= lo.x - 1.5 && v.x <= hi.x + 1.5 && v.y >= lo.y - 1.5 && v.y <= hi.y + 1.5,
                    "{v:?} outside {lo:?}..{hi:?} of {poly:?}"
                );
            }
        }
    }

    #[test]
    fn an_outlined_polygon_has_a_fill_fan_and_one_segment_per_distinct_edge() {
        let sq = [Pos2::new(0.0, 0.0), Pos2::new(10.0, 0.0), Pos2::new(10.0, 10.0), Pos2::new(0.0, 10.0)];
        let shapes = outlined_polygon(&sq, Color32::RED, Stroke::new(1.0_f32, Color32::WHITE));
        assert_eq!(shapes.iter().filter(|s| matches!(s, Shape::Mesh(m) if m.indices.len() == 6)).count(), 1);
        assert_eq!(shapes.iter().filter(|s| matches!(s, Shape::LineSegment { .. })).count(), 4);
        // No fill: outline only; no stroke: fill only; fewer than three corners: edges only.
        let outline = outlined_polygon(&sq, Color32::TRANSPARENT, Stroke::new(1.0_f32, Color32::WHITE));
        assert!(outline.iter().all(|s| matches!(s, Shape::LineSegment { .. })) && outline.len() == 4);
        assert_eq!(outlined_polygon(&sq, Color32::RED, Stroke::NONE).len(), 1);
        assert_eq!(outlined_polygon(&sq[..2], Color32::RED, Stroke::new(1.0_f32, Color32::WHITE)).len(), 1);
        // A corner that is not finite draws no fill and no edge through it.
        let bad = [Pos2::new(0.0, 0.0), Pos2::new(f32::NAN, 0.0), Pos2::new(5.0, 5.0)];
        let drawn = outlined_polygon(&bad, Color32::RED, Stroke::new(1.0_f32, Color32::WHITE));
        assert!(drawn.iter().all(|s| matches!(s, Shape::LineSegment { points, .. } if points.iter().all(|p| p.x.is_finite()))));
    }

}
