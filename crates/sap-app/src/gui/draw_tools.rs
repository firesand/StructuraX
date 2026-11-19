//! Mouse drawing tools: quick draw on the grid of a plane view, frames and
//! areas through picked points.
//!
//! The tools are pure functions of the pointer event, the view of the pane and
//! the model, so that tests can drive them without a window:
//! [`resolve`] turns an event into a [`DrawCommand`] (or an outcome that needs
//! no command), and [`on_event`] runs that command on the model.
//!
//! Project choices the vendor help does not settle:
//! - the pick radius for snapping to a joint or a grid intersection is 10 px,
//!   and a click selects the nearest grid segment within 8 px;
//! - a window with the quick frame tool, dragged in either direction, takes
//!   every grid segment with both ends inside (the help documents windows for
//!   areas only);
//! - an area has three or four corners, the solver's limit.

use super::view::{plane_members, plane_tolerance, PlaneView, ViewTransform};
use super::{point_to_segment_dist, ToolMode, ViewProjection};
use eframe::egui::{self, Color32, Pos2, Rect, Stroke};
use sap_model::generate::{BraceKind, GridRegion, SpanDirection, VerticalPlane};
use sap_model::SapModel;
use std::collections::HashSet;

/// Pixel radius for snapping to a joint or a grid intersection.
const SNAP_PX: f32 = 10.0;
/// Pixel distance within which a click picks a grid segment.
const SEGMENT_PX: f32 = 8.0;

/// Options and the points picked so far.
#[derive(Debug, Clone)]
pub(super) struct State {
    /// Frame section of new frames, braces and secondary beams; empty means
    /// the first section of the model.
    pub frame_section: String,
    /// Area section of new areas; empty means the first section of the model.
    pub area_section: String,
    /// Release M2 and M3 at both ends of frames drawn by the frame tools.
    pub pinned: bool,
    pub brace_kind: BraceKind,
    pub secondary_count: usize,
    pub span: SpanDirection,
    /// Points picked for the current frame sequence, polygon or rectangle.
    pending: Vec<[f64; 3]>,
    /// The tool and view `pending` was picked in; another one discards it.
    context: Option<(ToolMode, Option<PlaneView>, ViewProjection)>,
}

impl State {
    /// The tool and view in which points are waiting, if any are.
    pub(super) fn picked_in(&self) -> Option<(ToolMode, Option<PlaneView>, ViewProjection)> {
        if self.pending.is_empty() {
            None
        } else {
            self.context
        }
    }

    /// Forget the points picked so far (their coordinates are in other units
    /// after a change of units).
    pub(super) fn clear_pending(&mut self) {
        self.pending.clear();
        self.context = None;
    }
}

impl Default for State {
    fn default() -> Self {
        Self {
            frame_section: String::new(),
            area_section: String::new(),
            pinned: false,
            brace_kind: BraceKind::X,
            secondary_count: 1,
            span: SpanDirection::X,
            pending: Vec::new(),
            context: None,
        }
    }
}

/// What the pointer or the keyboard did in a viewport pane.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum PointerEvent {
    Click { pos: Pos2, double: bool },
    /// A finished drag from `from` to `to` (screen positions). Sent by the
    /// viewport's window routing.
    Window { from: Pos2, to: Pos2 },
    Enter,
    Escape,
}

/// The model command a tool resolved an event to.
#[derive(Debug, Clone, PartialEq)]
pub(super) enum DrawCommand {
    Frames { segments: Vec<([f64; 3], [f64; 3])> },
    Areas { corner_sets: Vec<Vec<[f64; 3]>> },
    Braces { region: GridRegion, plane: VerticalPlane },
    Secondary { region: GridRegion },
}

/// What the viewport does after an event.
#[derive(Debug, Clone, PartialEq)]
pub(super) enum DrawOutcome {
    /// The event meant nothing to the tool.
    Nothing,
    /// A hint for the status line (for example "click the next point").
    Status(String),
    /// A command ran: its message, or why it was refused.
    Applied(Result<String, String>),
    /// The tool is done; the viewport returns to the Select tool.
    Finish,
}

/// What the picked points become once a command has succeeded.
#[derive(Debug, Clone, PartialEq)]
pub(super) enum Then {
    Keep,
    Clear,
    Set(Vec<[f64; 3]>),
}

/// The resolution of one event.
#[derive(Debug, Clone, PartialEq)]
pub(super) enum Resolution {
    Outcome(DrawOutcome),
    Command { command: DrawCommand, then: Then },
}

fn refused(why: impl Into<String>) -> Resolution {
    Resolution::Outcome(DrawOutcome::Applied(Err(why.into())))
}

fn hint(text: impl Into<String>) -> Resolution {
    Resolution::Outcome(DrawOutcome::Status(text.into()))
}

/// Whether `tool` is handled by this module.
pub(super) fn is_draw_tool(tool: ToolMode) -> bool {
    matches!(
        tool,
        ToolMode::QuickFrame
            | ToolMode::PolyFrame
            | ToolMode::QuickArea
            | ToolMode::RectArea
            | ToolMode::PolyArea
            | ToolMode::QuickBrace
            | ToolMode::QuickSecondary
    )
}

// ---------------------------------------------------------------------------
// The grid of a view plane
// ---------------------------------------------------------------------------

/// Sorted GLOBAL grid coordinates of X, Y and Z, as the quick draw commands
/// index them.
fn grid_axes(model: &SapModel) -> Result<[Vec<f64>; 3], String> {
    let cs = model
        .coord_systems
        .get("GLOBAL")
        .ok_or("the model has no GLOBAL coordinate system")?;
    let mut axes: [Vec<f64>; 3] = Default::default();
    for g in &cs.grids {
        let k = match g.axis.as_str() {
            "X" => 0,
            "Y" => 1,
            "Z" => 2,
            other => return Err(format!("grid line {} has axis {other}", g.id)),
        };
        if !g.coordinate.is_finite() {
            return Err(format!("grid line {} has a nonfinite coordinate", g.id));
        }
        axes[k].push(g.coordinate);
    }
    for a in &mut axes {
        a.sort_by(f64::total_cmp);
        if a.windows(2).any(|w| w[1] - w[0] <= 0.0) {
            return Err("duplicate grid line coordinates".into());
        }
    }
    Ok(axes)
}

/// The grid lines of a grid plane: the two in-plane axes and the plane's own
/// line index along its normal.
struct PlaneGrid {
    h: usize,
    v: usize,
    n: usize,
    n_index: usize,
    gh: Vec<f64>,
    gv: Vec<f64>,
    gn: f64,
}

impl PlaneGrid {
    fn new(model: &SapModel, view: &ViewTransform, plane: PlaneView) -> Result<Self, String> {
        if PlaneView::of(view.proj, Some(plane.coordinate)).map(|p| p.normal) != Some(plane.normal)
        {
            return Err("this view does not show that plane".into());
        }
        let axes = grid_axes(model)?;
        let (h, v) = plane.in_plane_axes();
        let n = plane.normal.index();
        let tol = plane_tolerance(model);
        let n_index = axes[n]
            .iter()
            .position(|c| (c - plane.coordinate).abs() <= tol)
            .ok_or("the view plane is not a grid plane")?;
        if axes[h].len() < 2 || axes[v].len() < 2 {
            return Err("the plane has fewer than two grid lines in a direction".into());
        }
        Ok(Self {
            h,
            v,
            n,
            n_index,
            gh: axes[h].clone(),
            gv: axes[v].clone(),
            gn: axes[n][n_index],
        })
    }

    fn point(&self, i: usize, j: usize) -> [f64; 3] {
        let mut p = [0.0; 3];
        p[self.h] = self.gh[i];
        p[self.v] = self.gv[j];
        p[self.n] = self.gn;
        p
    }

    fn screen(&self, view: &ViewTransform, i: usize, j: usize) -> Pos2 {
        view.project(self.point(i, j))
    }

    /// Screen x of every h line and screen y of every v line.
    fn line_screens(&self, view: &ViewTransform) -> (Vec<f32>, Vec<f32>) {
        let xs = (0..self.gh.len())
            .map(|i| self.screen(view, i, 0).x)
            .collect();
        let ys = (0..self.gv.len())
            .map(|j| self.screen(view, 0, j).y)
            .collect();
        (xs, ys)
    }

    /// Every segment between adjacent grid intersections, in the order of the
    /// index commands: along the horizontal axis with the vertical line as the
    /// outer loop; then along the vertical axis, with the vertical position as
    /// the outer loop in an elevation (as `quick_draw_columns`) and the
    /// horizontal line in a plan (as `quick_draw_beams`).
    fn segments(&self) -> Vec<Segment> {
        let (nh, nv) = (self.gh.len(), self.gv.len());
        let mut out = Vec::with_capacity(nh * nv * 2);
        for j in 0..nv {
            for i in 0..nh - 1 {
                out.push(Segment {
                    a: (i, j),
                    b: (i + 1, j),
                });
            }
        }
        if self.n == 2 {
            for i in 0..nh {
                for j in 0..nv - 1 {
                    out.push(Segment {
                        a: (i, j),
                        b: (i, j + 1),
                    });
                }
            }
        } else {
            for j in 0..nv - 1 {
                for i in 0..nh {
                    out.push(Segment {
                        a: (i, j),
                        b: (i, j + 1),
                    });
                }
            }
        }
        out
    }

    /// The grid cell whose screen rectangle contains `pos`.
    fn cell_at(&self, view: &ViewTransform, pos: Pos2) -> Option<(usize, usize)> {
        let (xs, ys) = self.line_screens(view);
        for i in 0..xs.len() - 1 {
            for j in 0..ys.len() - 1 {
                let r = Rect::from_two_pos(
                    Pos2::new(xs[i], ys[j]),
                    Pos2::new(xs[i + 1], ys[j + 1]),
                );
                if r.contains(pos) {
                    return Some((i, j));
                }
            }
        }
        None
    }

    /// Index ranges of the grid lines inside a window (`None` for an axis
    /// without a line inside).
    fn lines_in(&self, view: &ViewTransform, rect: Rect) -> (Vec<usize>, Vec<usize>) {
        let (xs, ys) = self.line_screens(view);
        let i = (0..xs.len())
            .filter(|&i| rect.min.x <= xs[i] && xs[i] <= rect.max.x)
            .collect();
        let j = (0..ys.len())
            .filter(|&j| rect.min.y <= ys[j] && ys[j] <= rect.max.y)
            .collect();
        (i, j)
    }

    /// The nearest segment within [`SEGMENT_PX`] of `pos`. Ties go to the
    /// earlier segment: the horizontal axis, then the lower grid index.
    fn segment_at(&self, view: &ViewTransform, pos: Pos2) -> Option<Segment> {
        let mut best: Option<(f32, Segment)> = None;
        for s in self.segments() {
            let d = point_to_segment_dist(
                pos,
                self.screen(view, s.a.0, s.a.1),
                self.screen(view, s.b.0, s.b.1),
            );
            if d <= SEGMENT_PX && best.is_none_or(|(b, _)| d < b - 1e-6) {
                best = Some((d, s));
            }
        }
        best.map(|(_, s)| s)
    }

    /// Corners of cell (`i`, `j`), counter-clockwise as the viewer sees the
    /// plane: local 3 faces the viewer (+Z in a plan, -Y in an X-Z elevation
    /// because (+X) x (+Z) = -Y, +X in a Y-Z elevation).
    fn cell_corners(&self, i: usize, j: usize) -> Vec<[f64; 3]> {
        vec![
            self.point(i, j),
            self.point(i + 1, j),
            self.point(i + 1, j + 1),
            self.point(i, j + 1),
        ]
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Segment {
    a: (usize, usize),
    b: (usize, usize),
}

/// A block of grid cells: index ranges of the cells, inclusive.
#[derive(Debug, Clone, Copy)]
struct CellBlock {
    i: (usize, usize),
    j: (usize, usize),
}

impl CellBlock {
    /// The grid lines bounding the cells, as the region of an index command.
    fn lines_i(&self) -> (usize, usize) {
        (self.i.0, self.i.1 + 1)
    }

    fn lines_j(&self) -> (usize, usize) {
        (self.j.0, self.j.1 + 1)
    }
}

/// The cells under a click or inside a window.
fn cells_for(
    grid: &PlaneGrid,
    view: &ViewTransform,
    event: PointerEvent,
) -> Option<CellBlock> {
    match event {
        PointerEvent::Click { pos, .. } => grid.cell_at(view, pos).map(|(i, j)| CellBlock {
            i: (i, i),
            j: (j, j),
        }),
        PointerEvent::Window { from, to } => {
            let (is, js) = grid.lines_in(view, Rect::from_two_pos(from, to));
            // A cell is inside when both its lines are: runs of consecutive
            // line indices of length two or more.
            let run = |v: &[usize]| -> Option<(usize, usize)> {
                let first = *v.first()?;
                let mut last = first;
                for &k in &v[1..] {
                    if k == last + 1 {
                        last = k;
                    }
                }
                (last > first).then_some((first, last - 1))
            };
            Some(CellBlock {
                i: run(&is)?,
                j: run(&js)?,
            })
        }
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// Picking points
// ---------------------------------------------------------------------------

/// The point a click at `pos` picks. In a plane view: the nearest joint of
/// that plane within [`SNAP_PX`], else the nearest grid intersection of the
/// plane within [`SNAP_PX`], else the free point under the pointer. Elsewhere:
/// the nearest joint within [`SNAP_PX`]. A snapped point takes the stored
/// coordinate of the joint or grid line; only a free point is unprojected.
fn pick_point(
    model: &SapModel,
    view: &ViewTransform,
    plane: Option<PlaneView>,
    pos: Pos2,
) -> Result<[f64; 3], String> {
    let nearest_joint = |names: &mut dyn Iterator<Item = &String>| -> Option<[f64; 3]> {
        let mut best: Option<(f32, [f64; 3])> = None;
        for name in names {
            let p = &model.points[name];
            let c = [p.x, p.y, p.z];
            let d = view.project(c).distance(pos);
            if d <= SNAP_PX && best.is_none_or(|(b, _)| d < b) {
                best = Some((d, c));
            }
        }
        best.map(|(_, c)| c)
    };
    let Some(plane) = plane else {
        return nearest_joint(&mut model.points.keys())
            .ok_or_else(|| "no joint under the pointer; in this view points snap to joints only".into());
    };
    if PlaneView::of(view.proj, Some(plane.coordinate)).map(|p| p.normal) != Some(plane.normal) {
        return Err("this view does not show that plane".into());
    }
    let members = plane_members(model, plane, plane_tolerance(model));
    if let Some(c) = nearest_joint(&mut members.points.iter()) {
        return Ok(c);
    }
    let level = plane_level(model, plane);
    if let Ok(axes) = grid_axes(model) {
        let (h, v) = plane.in_plane_axes();
        let at = |a: f64, b: f64| {
            let mut p = [0.0; 3];
            p[h] = a;
            p[v] = b;
            p[plane.normal.index()] = level;
            p
        };
        let near_h: Vec<f64> = axes[h]
            .iter()
            .copied()
            .filter(|a| (view.project(at(*a, axes[v].first().copied().unwrap_or(0.0))).x - pos.x).abs() <= SNAP_PX)
            .collect();
        let near_v: Vec<f64> = axes[v]
            .iter()
            .copied()
            .filter(|b| (view.project(at(axes[h].first().copied().unwrap_or(0.0), *b)).y - pos.y).abs() <= SNAP_PX)
            .collect();
        let mut best: Option<(f32, [f64; 3])> = None;
        for a in &near_h {
            for b in &near_v {
                let c = at(*a, *b);
                let d = view.project(c).distance(pos);
                if d <= SNAP_PX && best.is_none_or(|(bd, _)| d < bd) {
                    best = Some((d, c));
                }
            }
        }
        if let Some((_, c)) = best {
            return Ok(c);
        }
    }
    let mut p = view
        .unproject(pos, plane)
        .ok_or_else(|| "the pointer is outside the drawable area".to_string())?;
    p[plane.normal.index()] = level;
    Ok(p)
}

/// The coordinate of `plane` along its normal: the stored value of the grid
/// line the plane lies on (within the plane tolerance), else its own value. A
/// plane typed in Set 2D View can differ from the grid line in the last bits.
fn plane_level(model: &SapModel, plane: PlaneView) -> f64 {
    let tol = plane_tolerance(model);
    grid_axes(model)
        .ok()
        .and_then(|axes| {
            axes[plane.normal.index()]
                .iter()
                .copied()
                .find(|c| (c - plane.coordinate).abs() <= tol)
        })
        .unwrap_or(plane.coordinate)
}

fn same_point(model: &SapModel, a: [f64; 3], b: [f64; 3]) -> bool {
    let tol = sap_model::generate::geometry_tolerance(model, &[a, b]);
    (0..3).all(|k| (a[k] - b[k]).abs() <= tol)
}

// ---------------------------------------------------------------------------
// Resolving an event
// ---------------------------------------------------------------------------

fn context_of(
    tool: ToolMode,
    view: &ViewTransform,
    plane: Option<PlaneView>,
) -> (ToolMode, Option<PlaneView>, ViewProjection) {
    (tool, plane, view.proj)
}

/// Turn one event into a command or an outcome. Changes only the picked
/// points of `state`; the model is read, never written.
pub(super) fn resolve(
    state: &mut State,
    tool: ToolMode,
    view: &ViewTransform,
    plane: Option<PlaneView>,
    event: PointerEvent,
    model: &SapModel,
) -> Resolution {
    let ctx = context_of(tool, view, plane);
    if state.context != Some(ctx) {
        state.pending.clear();
        state.context = Some(ctx);
    }
    if event == PointerEvent::Escape {
        state.pending.clear();
        return Resolution::Outcome(DrawOutcome::Finish);
    }
    match tool {
        ToolMode::QuickFrame
        | ToolMode::QuickArea
        | ToolMode::QuickBrace
        | ToolMode::QuickSecondary => resolve_quick(tool, view, plane, event, model),
        ToolMode::PolyFrame => resolve_poly_frame(state, view, plane, event, model),
        ToolMode::RectArea => resolve_rect_area(state, view, plane, event, model),
        ToolMode::PolyArea => resolve_poly_area(state, view, plane, event, model),
        _ => Resolution::Outcome(DrawOutcome::Nothing),
    }
}

fn resolve_quick(
    tool: ToolMode,
    view: &ViewTransform,
    plane: Option<PlaneView>,
    event: PointerEvent,
    model: &SapModel,
) -> Resolution {
    // Enter ends nothing here; the second click of a double click repeats the
    // first one, which has drawn already.
    if matches!(
        event,
        PointerEvent::Enter | PointerEvent::Click { double: true, .. }
    ) {
        return Resolution::Outcome(DrawOutcome::Nothing);
    }
    let Some(plane) = plane else {
        return refused(
            "quick draw works in a 2D plane view of a grid plane (View > Set 2D View); not in 3D or a whole-model 2D view",
        );
    };
    let grid = match PlaneGrid::new(model, view, plane) {
        Ok(g) => g,
        Err(e) => return refused(e),
    };
    match tool {
        ToolMode::QuickFrame => {
            let segments: Vec<Segment> = match event {
                PointerEvent::Click { pos, .. } => {
                    match grid.segment_at(view, pos) {
                        Some(s) => vec![s],
                        None => return hint("no grid segment under the pointer"),
                    }
                }
                PointerEvent::Window { from, to } => {
                    let (is, js) = grid.lines_in(view, Rect::from_two_pos(from, to));
                    let (is, js): (HashSet<usize>, HashSet<usize>) =
                        (is.into_iter().collect(), js.into_iter().collect());
                    grid.segments()
                        .into_iter()
                        .filter(|s| {
                            is.contains(&s.a.0)
                                && is.contains(&s.b.0)
                                && js.contains(&s.a.1)
                                && js.contains(&s.b.1)
                        })
                        .collect()
                }
                _ => return Resolution::Outcome(DrawOutcome::Nothing),
            };
            if segments.is_empty() {
                return hint("no grid segment lies fully inside the window");
            }
            Resolution::Command {
                command: DrawCommand::Frames {
                    segments: segments
                        .iter()
                        .map(|s| (grid.point(s.a.0, s.a.1), grid.point(s.b.0, s.b.1)))
                        .collect(),
                },
                then: Then::Keep,
            }
        }
        ToolMode::QuickArea => {
            let Some(block) = cells_for(&grid, view, event) else {
                return hint("no grid cell under the pointer or inside the window");
            };
            let tol = plane_tolerance(model);
            let existing = existing_area_keys(model, tol);
            let mut sets = Vec::new();
            let mut skipped = 0;
            for i in block.i.0..=block.i.1 {
                for j in block.j.0..=block.j.1 {
                    let corners = grid.cell_corners(i, j);
                    if existing.contains(&area_key(&corners, tol)) {
                        skipped += 1;
                    } else {
                        sets.push(corners);
                    }
                }
            }
            if sets.is_empty() {
                return refused(format!(
                    "quick draw areas creates no objects ({skipped} of the cells have an area already)"
                ));
            }
            Resolution::Command {
                command: DrawCommand::Areas { corner_sets: sets },
                then: Then::Keep,
            }
        }
        ToolMode::QuickBrace => {
            if plane.normal.index() == 2 {
                return refused("braces are drawn in an elevation (an X-Z or Y-Z plane view)");
            }
            let Some(block) = cells_for(&grid, view, event) else {
                return hint("no grid cell under the pointer or inside the window");
            };
            // In an X-Z elevation the horizontal axis is X and Y is fixed; in
            // a Y-Z elevation the horizontal axis is Y and X is fixed.
            let (vp, region) = if grid.n == 1 {
                (
                    VerticalPlane::XZ,
                    GridRegion {
                        x: block.lines_i(),
                        y: (grid.n_index, grid.n_index),
                        z: block.lines_j(),
                    },
                )
            } else {
                (
                    VerticalPlane::YZ,
                    GridRegion {
                        x: (grid.n_index, grid.n_index),
                        y: block.lines_i(),
                        z: block.lines_j(),
                    },
                )
            };
            Resolution::Command {
                command: DrawCommand::Braces { region, plane: vp },
                then: Then::Keep,
            }
        }
        ToolMode::QuickSecondary => {
            if plane.normal.index() != 2 {
                return refused("secondary beams are drawn in a plan (an X-Y plane view)");
            }
            let Some(block) = cells_for(&grid, view, event) else {
                return hint("no grid cell under the pointer or inside the window");
            };
            let region = GridRegion {
                x: block.lines_i(),
                y: block.lines_j(),
                z: (grid.n_index, grid.n_index),
            };
            Resolution::Command {
                command: DrawCommand::Secondary { region },
                then: Then::Keep,
            }
        }
        _ => Resolution::Outcome(DrawOutcome::Nothing),
    }
}

fn coord_key(c: [f64; 3], tol: f64) -> [i64; 3] {
    std::array::from_fn(|k| (c[k] / tol).round() as i64)
}

/// The corners of an area as a sorted list of quantized coordinates.
fn area_key(corners: &[[f64; 3]], tol: f64) -> Vec<[i64; 3]> {
    let mut k: Vec<[i64; 3]> = corners.iter().map(|c| coord_key(*c, tol)).collect();
    k.sort_unstable();
    k
}

fn existing_area_keys(model: &SapModel, tol: f64) -> HashSet<Vec<[i64; 3]>> {
    model
        .areas
        .values()
        .filter_map(|a| {
            let corners: Option<Vec<[f64; 3]>> = a
                .points
                .iter()
                .map(|p| model.points.get(p).map(|p| [p.x, p.y, p.z]))
                .collect();
            corners.map(|c| area_key(&c, tol))
        })
        .collect()
}

fn pick_or_refuse(
    model: &SapModel,
    view: &ViewTransform,
    plane: Option<PlaneView>,
    pos: Pos2,
) -> Result<[f64; 3], Resolution> {
    pick_point(model, view, plane, pos).map_err(refused)
}

fn resolve_poly_frame(
    state: &mut State,
    view: &ViewTransform,
    plane: Option<PlaneView>,
    event: PointerEvent,
    model: &SapModel,
) -> Resolution {
    match event {
        PointerEvent::Enter => {
            state.pending.clear();
            hint("frame sequence ended")
        }
        PointerEvent::Window { .. } => Resolution::Outcome(DrawOutcome::Nothing),
        PointerEvent::Click { pos, double } => {
            if double {
                // The second click of a double click: the first one has
                // already been handled (it may have ended the sequence).
                if state.pending.is_empty() {
                    return Resolution::Outcome(DrawOutcome::Nothing);
                }
                state.pending.clear();
                return hint("frame sequence ended");
            }
            let p = match pick_or_refuse(model, view, plane, pos) {
                Ok(p) => p,
                Err(r) => return r,
            };
            match state.pending.last().copied() {
                None => {
                    state.pending = vec![p];
                    hint("click the next point (Enter or a double click ends the sequence)")
                }
                Some(last) if same_point(model, last, p) => {
                    state.pending.clear();
                    hint("frame sequence ended")
                }
                Some(last) => Resolution::Command {
                    command: DrawCommand::Frames {
                        segments: vec![(last, p)],
                    },
                    then: Then::Set(vec![p]),
                },
            }
        }
        PointerEvent::Escape => Resolution::Outcome(DrawOutcome::Finish),
    }
}

fn resolve_rect_area(
    state: &mut State,
    view: &ViewTransform,
    plane: Option<PlaneView>,
    event: PointerEvent,
    model: &SapModel,
) -> Resolution {
    let Some(plane) = plane else {
        return refused("rectangular areas are drawn in a 2D plane view (View > Set 2D View)");
    };
    match event {
        PointerEvent::Enter => {
            state.pending.clear();
            Resolution::Outcome(DrawOutcome::Nothing)
        }
        PointerEvent::Window { .. } | PointerEvent::Escape => {
            Resolution::Outcome(DrawOutcome::Nothing)
        }
        // The second click of a double click repeats the first.
        PointerEvent::Click { double: true, .. } => Resolution::Outcome(DrawOutcome::Nothing),
        PointerEvent::Click { pos, .. } => {
            let p = match pick_or_refuse(model, view, Some(plane), pos) {
                Ok(p) => p,
                Err(r) => return r,
            };
            let Some(first) = state.pending.first().copied() else {
                state.pending = vec![p];
                return hint("click the opposite corner");
            };
            let (h, v) = plane.in_plane_axes();
            let level = plane_level(model, plane);
            let tol = sap_model::generate::geometry_tolerance(model, &[first, p]);
            let (h0, h1) = (first[h].min(p[h]), first[h].max(p[h]));
            let (v0, v1) = (first[v].min(p[v]), first[v].max(p[v]));
            if h1 - h0 <= tol || v1 - v0 <= tol {
                state.pending.clear();
                return refused("the two corners lie on one line and define no rectangle");
            }
            let at = |a: f64, b: f64| {
                let mut c = [0.0; 3];
                c[h] = a;
                c[v] = b;
                c[plane.normal.index()] = level;
                c
            };
            Resolution::Command {
                command: DrawCommand::Areas {
                    corner_sets: vec![vec![at(h0, v0), at(h1, v0), at(h1, v1), at(h0, v1)]],
                },
                then: Then::Clear,
            }
        }
    }
}

fn close_polygon(state: &mut State) -> Resolution {
    let points = std::mem::take(&mut state.pending);
    if points.len() < 3 {
        return refused(format!(
            "an area needs at least 3 points, {} picked",
            points.len()
        ));
    }
    Resolution::Command {
        command: DrawCommand::Areas {
            corner_sets: vec![points],
        },
        then: Then::Clear,
    }
}

fn resolve_poly_area(
    state: &mut State,
    view: &ViewTransform,
    plane: Option<PlaneView>,
    event: PointerEvent,
    model: &SapModel,
) -> Resolution {
    match event {
        PointerEvent::Enter => close_polygon(state),
        PointerEvent::Window { .. } | PointerEvent::Escape => {
            Resolution::Outcome(DrawOutcome::Nothing)
        }
        PointerEvent::Click { pos, double } => {
            if double {
                if state.pending.is_empty() {
                    return Resolution::Outcome(DrawOutcome::Nothing);
                }
                return close_polygon(state);
            }
            let p = match pick_or_refuse(model, view, plane, pos) {
                Ok(p) => p,
                Err(r) => return r,
            };
            if let (Some(first), Some(last)) =
                (state.pending.first().copied(), state.pending.last().copied())
            {
                if same_point(model, first, p) || same_point(model, last, p) {
                    return close_polygon(state);
                }
            }
            if state.pending.len() >= 4 {
                return refused(
                    "an area has at most 4 points (the solver accepts 3 or 4); press Enter to finish it",
                );
            }
            state.pending.push(p);
            hint(format!(
                "area point {} picked: click the next point, or Enter to finish",
                state.pending.len()
            ))
        }
    }
}

// ---------------------------------------------------------------------------
// Running a command
// ---------------------------------------------------------------------------

fn frame_section(state: &State, model: &SapModel) -> Result<String, String> {
    if !state.frame_section.is_empty() {
        return Ok(state.frame_section.clone());
    }
    model
        .frame_sections
        .keys()
        .next()
        .cloned()
        .ok_or_else(|| "define a frame section first".to_string())
}

fn area_section(state: &State, model: &SapModel) -> Result<String, String> {
    if !state.area_section.is_empty() {
        return Ok(state.area_section.clone());
    }
    model
        .area_sections
        .keys()
        .next()
        .cloned()
        .ok_or_else(|| "define an area section first".to_string())
}

/// Run `command` on the model with the options of `state`. The model is
/// unchanged when the result is an error.
pub(super) fn apply(
    state: &State,
    command: &DrawCommand,
    model: &mut SapModel,
) -> Result<String, String> {
    match command {
        DrawCommand::Frames { segments } => {
            let section = frame_section(state, model)?;
            let s = model
                .draw_frames(segments, &section, state.pinned)
                .map_err(|e| e.to_string())?;
            Ok(format!("Drew {} frame(s) of section {section}", s.frames_created.len()))
        }
        DrawCommand::Areas { corner_sets } => {
            let section = area_section(state, model)?;
            let s = model
                .draw_areas(corner_sets, &section)
                .map_err(|e| e.to_string())?;
            Ok(format!("Drew {} area(s) of section {section}", s.areas_created.len()))
        }
        DrawCommand::Braces { region, plane } => {
            let section = frame_section(state, model)?;
            let s = model
                .quick_draw_braces(*region, *plane, state.brace_kind, &section)
                .map_err(|e| e.to_string())?;
            Ok(format!("Drew {} brace(s) of section {section}", s.frames_created.len()))
        }
        DrawCommand::Secondary { region } => {
            let section = frame_section(state, model)?;
            let s = model
                .quick_draw_secondary_beams(*region, state.span, state.secondary_count, &section)
                .map_err(|e| e.to_string())?;
            Ok(format!(
                "Drew {} secondary beam(s) of section {section}",
                s.frames_created.len()
            ))
        }
    }
}

/// Handle one event of `tool` in a pane with projection `view` showing
/// `plane` (`None`: the 3D view or a 2D view of the whole model).
pub(super) fn on_event(
    state: &mut State,
    tool: ToolMode,
    view: &ViewTransform,
    plane: Option<PlaneView>,
    event: PointerEvent,
    model: &mut SapModel,
) -> DrawOutcome {
    match resolve(state, tool, view, plane, event, model) {
        Resolution::Outcome(o) => o,
        Resolution::Command { command, then } => {
            let result = apply(state, &command, model);
            if result.is_ok() {
                match then {
                    Then::Keep => {}
                    Then::Clear => state.pending.clear(),
                    Then::Set(p) => state.pending = p,
                }
            }
            DrawOutcome::Applied(result)
        }
    }
}

// ---------------------------------------------------------------------------
// Preview
// ---------------------------------------------------------------------------

/// Shapes that preview what the next click at `hover` would do.
pub(super) fn preview(
    state: &State,
    tool: ToolMode,
    view: &ViewTransform,
    plane: Option<PlaneView>,
    hover: Option<Pos2>,
    model: &SapModel,
) -> Vec<egui::Shape> {
    let mut shapes = Vec::new();
    let Some(hover) = hover else {
        return shapes;
    };
    let accent = Color32::from_rgb(255, 210, 70);
    let fill = Color32::from_rgba_unmultiplied(255, 210, 70, 50);
    let pending: &[[f64; 3]] = if state.context == Some(context_of(tool, view, plane)) {
        &state.pending
    } else {
        &[]
    };
    match tool {
        ToolMode::QuickFrame => {
            if let Some(p) = plane {
                if let Ok(grid) = PlaneGrid::new(model, view, p) {
                    if let Some(s) = grid.segment_at(view, hover) {
                        shapes.push(egui::Shape::line_segment(
                            [
                                grid.screen(view, s.a.0, s.a.1),
                                grid.screen(view, s.b.0, s.b.1),
                            ],
                            Stroke::new(3.0_f32, accent),
                        ));
                    }
                }
            }
        }
        ToolMode::QuickArea | ToolMode::QuickBrace | ToolMode::QuickSecondary => {
            if let Some(p) = plane {
                if let Ok(grid) = PlaneGrid::new(model, view, p) {
                    if let Some((i, j)) = grid.cell_at(view, hover) {
                        let poly: Vec<Pos2> = grid
                            .cell_corners(i, j)
                            .into_iter()
                            .map(|c| view.project(c))
                            .collect();
                        shapes.push(egui::Shape::convex_polygon(
                            poly,
                            fill,
                            Stroke::new(1.5_f32, accent),
                        ));
                    }
                }
            }
        }
        ToolMode::PolyFrame | ToolMode::PolyArea | ToolMode::RectArea => {
            let snapped = pick_point(model, view, plane, hover).ok();
            let mut path: Vec<Pos2> = pending.iter().map(|c| view.project(*c)).collect();
            if let Some(c) = snapped {
                let s = view.project(c);
                shapes.push(egui::Shape::circle_stroke(s, SNAP_PX, Stroke::new(1.5_f32, accent)));
                if tool == ToolMode::RectArea {
                    if let Some(first) = pending.first() {
                        shapes.push(egui::Shape::rect_stroke(
                            Rect::from_two_pos(view.project(*first), s),
                            0.0,
                            Stroke::new(1.5_f32, accent),
                        ));
                    }
                } else {
                    path.push(s);
                }
            }
            if tool != ToolMode::RectArea && path.len() >= 2 {
                if tool == ToolMode::PolyArea && path.len() >= 3 {
                    path.push(path[0]);
                }
                shapes.push(egui::Shape::line(path, Stroke::new(1.5_f32, accent)));
            }
        }
        _ => {}
    }
    shapes
}

// ---------------------------------------------------------------------------
// Properties window
// ---------------------------------------------------------------------------

/// The floating "Properties of Object" window of the active tool.
pub(super) fn render_properties(
    ctx: &egui::Context,
    state: &mut State,
    tool: ToolMode,
    model: &SapModel,
) {
    if state.frame_section.is_empty() || !model.frame_sections.contains_key(&state.frame_section) {
        if let Some(first) = model.frame_sections.keys().next() {
            state.frame_section.clone_from(first);
        }
    }
    if state.area_section.is_empty() || !model.area_sections.contains_key(&state.area_section) {
        if let Some(first) = model.area_sections.keys().next() {
            state.area_section.clone_from(first);
        }
    }
    let frames = matches!(
        tool,
        ToolMode::QuickFrame | ToolMode::PolyFrame | ToolMode::QuickBrace | ToolMode::QuickSecondary
    );
    let areas = matches!(
        tool,
        ToolMode::QuickArea | ToolMode::RectArea | ToolMode::PolyArea
    );
    egui::Window::new("Properties of Object")
        .anchor(egui::Align2::RIGHT_BOTTOM, [-8.0, -36.0])
        .resizable(false)
        .collapsible(false)
        .show(ctx, |ui| {
            egui::Grid::new("draw_tool_properties")
                .num_columns(2)
                .show(ui, |ui| {
                    if frames {
                        ui.label("Section");
                        egui::ComboBox::from_id_salt("draw_frame_section")
                            .selected_text(state.frame_section.clone())
                            .show_ui(ui, |ui| {
                                for name in model.frame_sections.keys() {
                                    ui.selectable_value(
                                        &mut state.frame_section,
                                        name.clone(),
                                        name,
                                    );
                                }
                            });
                        ui.end_row();
                    }
                    if areas {
                        ui.label("Area section");
                        egui::ComboBox::from_id_salt("draw_area_section")
                            .selected_text(state.area_section.clone())
                            .show_ui(ui, |ui| {
                                for name in model.area_sections.keys() {
                                    ui.selectable_value(
                                        &mut state.area_section,
                                        name.clone(),
                                        name,
                                    );
                                }
                            });
                        ui.end_row();
                    }
                    if matches!(tool, ToolMode::QuickFrame | ToolMode::PolyFrame) {
                        ui.label("Pinned ends");
                        ui.checkbox(&mut state.pinned, "Release M2 and M3 at both ends");
                        ui.end_row();
                    }
                    if tool == ToolMode::QuickBrace {
                        ui.label("Brace");
                        egui::ComboBox::from_id_salt("draw_brace_kind")
                            .selected_text(brace_label(state.brace_kind))
                            .show_ui(ui, |ui| {
                                for kind in [BraceKind::X, BraceKind::V, BraceKind::InvertedV] {
                                    ui.selectable_value(
                                        &mut state.brace_kind,
                                        kind,
                                        brace_label(kind),
                                    );
                                }
                            });
                        ui.end_row();
                        ui.label("");
                        ui.label("Braces are pinned at both ends");
                        ui.end_row();
                    }
                    if tool == ToolMode::QuickSecondary {
                        ui.label("Beams per bay");
                        ui.add(egui::DragValue::new(&mut state.secondary_count).range(1..=1000));
                        ui.end_row();
                        ui.label("Span");
                        egui::ComboBox::from_id_salt("draw_secondary_span")
                            .selected_text(span_label(state.span))
                            .show_ui(ui, |ui| {
                                for span in [SpanDirection::X, SpanDirection::Y] {
                                    ui.selectable_value(&mut state.span, span, span_label(span));
                                }
                            });
                        ui.end_row();
                        ui.label("");
                        ui.label("Secondary beams are pinned at both ends");
                        ui.end_row();
                    }
                });
            ui.separator();
            ui.small(match tool {
                ToolMode::QuickFrame => "Click a grid segment, or drag a window over several.",
                ToolMode::PolyFrame => "Click points; Enter or a double click ends the sequence.",
                ToolMode::QuickArea => "Click a grid cell, or drag a window over several.",
                ToolMode::RectArea => "Click two opposite corners.",
                ToolMode::PolyArea => "Click 3 or 4 points; Enter or a double click closes it.",
                ToolMode::QuickBrace => "Click a grid cell of an elevation, or drag a window.",
                ToolMode::QuickSecondary => "Click a grid cell of a plan, or drag a window.",
                _ => "",
            });
        });
}

fn brace_label(kind: BraceKind) -> &'static str {
    match kind {
        BraceKind::X => "X",
        BraceKind::V => "V",
        BraceKind::InvertedV => "Inverted V",
    }
}

fn span_label(span: SpanDirection) -> &'static str {
    match span {
        SpanDirection::X => "Along X",
        SpanDirection::Y => "Along Y",
    }
}

#[cfg(test)]
mod tests;
