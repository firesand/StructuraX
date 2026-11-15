//! Radial replication and extrusion, reversal of area local axes and curved
//! frames.
//!
//! Radial commands rotate about a VERTICAL axis through `(center[0],
//! center[1])`; a positive angle is counter-clockwise seen from above (CSI
//! Extrude and Replicate help topics). Axes parallel to X or Y and arbitrary 3D
//! lines are not offered.
//!
//! Local axes follow the solver's definitions (`sap-solver`: `frame_local_axes`,
//! `area_local_system`, `joint_local_to_global_matrix`), which this module
//! mirrors because the model crate cannot depend on the solver:
//!
//! * Frame: local 1 runs i to j. If the horizontal part `h` of the unit
//!   direction is above 1e-4, local 2 is `(-e1x e1z / h, -e1y e1z / h, h)` and
//!   `angle_deg` is unchanged by a rotation about Z. For a vertical frame
//!   (`h <= 1e-4`) local 2 is `+X` rotated by `angle_deg` about local 1 and
//!   local 3 starts at `(0, +-1, 0)` (sign of `e1z`). Rotating the frame by
//!   `t` about Z therefore adds `t` to `angle_deg` of a frame pointing up and
//!   `-t` to one pointing down.
//! * Area: local 3 is the normalised cross product of the diagonals (see
//!   `area_local_system`). With a vertical normal (`h <= 1e-4`) local 1 starts
//!   at `+X` and local 2 at `(0, sign(e3z), 0)`; the angle rotates local 1
//!   towards local 2. A rotation by `t` about Z adds `t` to `angle_deg` for
//!   `e3z >= 0` and `-t` for `e3z < 0`. A non-vertical normal needs no change.
//! * Joint: `R = Rz(A) Ry(B) Rx(C)`, so a rotation by `t` about Z adds `t` to
//!   `A`; an explicit matrix is left-multiplied by `Rz(t)`.

use crate::editing::{coords, finite, invalid, same_assignments};
use crate::generate::{geometry_tolerance, Stage, MAX_GENERATED};
use crate::{AreaObj, ConstraintType, EditError, EditSummary, FrameObj, PointObj, SapModel};
use std::collections::{BTreeSet, HashMap};

/// What stays fixed when the local 3 axis of an area is reversed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReverseKeep {
    /// Every assignment keeps its direction in the global system.
    SameGlobalLoads,
    /// Every assignment keeps its local direction, so it turns with the axes.
    SameLocalLoads,
}

/// Shape of a curved frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CurveKind {
    /// Circular arc through the three points.
    Circular,
    /// Parabola through the three points over the chord coordinate.
    Parabolic,
}

/// `(sin, cos)` of `degrees`, exact at multiples of 90 degrees.
fn sin_cos_deg(degrees: f64) -> (f64, f64) {
    let r = degrees.rem_euclid(360.0);
    if r == 0.0 {
        (0.0, 1.0)
    } else if r == 90.0 {
        (1.0, 0.0)
    } else if r == 180.0 {
        (0.0, -1.0)
    } else if r == 270.0 {
        (-1.0, 0.0)
    } else {
        degrees.to_radians().sin_cos()
    }
}

fn rotate_about(center: [f64; 2], p: [f64; 3], sc: (f64, f64)) -> [f64; 3] {
    let (s, c) = sc;
    let (dx, dy) = (p[0] - center[0], p[1] - center[1]);
    [
        center[0] + dx * c - dy * s,
        center[1] + dx * s + dy * c,
        p[2],
    ]
}

fn is_global(csys: &str) -> bool {
    csys.eq_ignore_ascii_case("GLOBAL")
}

fn identity_axes(p: &PointObj) -> bool {
    p.local_axes_matrix.is_none() && p.local_axes_deg.iter().all(|a| a.abs() < 1e-14)
}

/// Vertical-axis rotation class of a frame: `Some(+1)` pointing up, `Some(-1)`
/// pointing down, `None` not vertical (solver threshold).
fn vertical_frame_sign(a: [f64; 3], b: [f64; 3]) -> Option<f64> {
    let d: [f64; 3] = std::array::from_fn(|k| b[k] - a[k]);
    let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt().max(1e-12);
    let e1 = [d[0] / len, d[1] / len, d[2] / len];
    let h = (e1[0] * e1[0] + e1[1] * e1[1]).sqrt();
    if h > 1e-4 {
        None
    } else if e1[2] >= 0.0 {
        Some(1.0)
    } else {
        Some(-1.0)
    }
}

/// Local 3 axis of an area as the solver computes it (joint order only).
fn area_normal(c: &[[f64; 3]]) -> [f64; 3] {
    let (v_xi, v_eta) = if c.len() >= 4 {
        (
            std::array::from_fn::<f64, 3, _>(|k| (c[1][k] + c[2][k]) - (c[0][k] + c[3][k])),
            std::array::from_fn::<f64, 3, _>(|k| (c[2][k] + c[3][k]) - (c[0][k] + c[1][k])),
        )
    } else {
        (
            std::array::from_fn::<f64, 3, _>(|k| c[1][k] - c[0][k]),
            std::array::from_fn::<f64, 3, _>(|k| c[2][k] - c[0][k]),
        )
    };
    let cross = [
        v_xi[1] * v_eta[2] - v_xi[2] * v_eta[1],
        v_xi[2] * v_eta[0] - v_xi[0] * v_eta[2],
        v_xi[0] * v_eta[1] - v_xi[1] * v_eta[0],
    ];
    let norm = (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2])
        .sqrt()
        .max(1e-12);
    [cross[0] / norm, cross[1] / norm, cross[2] / norm]
}

/// True when the joints do not span a plane area (parallel diagonals or edges,
/// or a repeated or collinear set).
fn area_degenerate(c: &[[f64; 3]]) -> bool {
    let (v_xi, v_eta): ([f64; 3], [f64; 3]) = if c.len() >= 4 {
        (
            std::array::from_fn(|k| (c[1][k] + c[2][k]) - (c[0][k] + c[3][k])),
            std::array::from_fn(|k| (c[2][k] + c[3][k]) - (c[0][k] + c[1][k])),
        )
    } else {
        (
            std::array::from_fn(|k| c[1][k] - c[0][k]),
            std::array::from_fn(|k| c[2][k] - c[0][k]),
        )
    };
    let cross = [
        v_xi[1] * v_eta[2] - v_xi[2] * v_eta[1],
        v_xi[2] * v_eta[0] - v_xi[0] * v_eta[2],
        v_xi[0] * v_eta[1] - v_xi[1] * v_eta[0],
    ];
    let norm = |v: [f64; 3]| (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    norm(cross) <= 1e-9 * norm(v_xi) * norm(v_eta)
}

/// `Some(+1)` / `Some(-1)` for a vertical normal pointing up / down (solver
/// threshold), `None` otherwise.
fn vertical_normal_sign(e3: [f64; 3]) -> Option<f64> {
    if (e3[0] * e3[0] + e3[1] * e3[1]).sqrt() <= 1e-4 {
        Some(if e3[2] >= 0.0 { 1.0 } else { -1.0 })
    } else {
        None
    }
}

/// Directions of frame and area loads that are unchanged by a rotation about Z:
/// local 1..3, global Z (6), projected Z (9), gravity (10), projected gravity (11).
fn load_dir_invariant(dir: i32, csys: &str) -> bool {
    match dir {
        1..=3 => true,
        6 | 9 | 10 | 11 => is_global(csys),
        _ => false,
    }
}

fn point_checks(model: &SapModel, p: &PointObj) -> Result<(), EditError> {
    let name = &p.name;
    let unsupported = |what: &str| {
        Err(EditError::Unsupported(format!(
            "joint {name}: {what} is not invariant under rotation about Z"
        )))
    };
    let asym = |v: &[f64; 6]| v[0] != v[1] || v[3] != v[4];
    let identity = identity_axes(p);
    if identity
        && (p.restraint[0] != p.restraint[1]
            || p.restraint[3] != p.restraint[4]
            || asym(&p.mass)
            || asym(&p.spring)
            || p.spring_coupled.is_some())
    {
        return unsupported(
            "a restraint, spring or mass that differs between X and Y (or RX and RY) on global axes",
        );
    }
    if !identity && !p.spring_is_local && (asym(&p.spring) || p.spring_coupled.is_some()) {
        return unsupported("a global-axes spring that differs between X and Y or is coupled");
    }
    for l in &p.loads_force {
        let local = l.csys.eq_ignore_ascii_case("LOCAL");
        if (identity || !local) && [0, 1, 3, 4].iter().any(|&k| l.force[k] != 0.0) {
            return unsupported("a joint force or moment with X or Y components (MX, MY)");
        }
    }
    for l in &p.loads_displ {
        let local = l.csys.eq_ignore_ascii_case("LOCAL");
        if (identity || !local) && [0, 1, 3, 4].iter().any(|&k| l.displ[k] != 0.0) {
            return unsupported("a joint displacement with X or Y components (RX, RY)");
        }
    }
    for cname in &p.constraints {
        let c = model
            .constraints
            .get(cname)
            .ok_or_else(|| invalid(format!("joint {name} names unknown constraint {cname}")))?;
        let symmetric = c.dofs[0] == c.dofs[1] && c.dofs[3] == c.dofs[4];
        let ok = match c.constraint_type {
            ConstraintType::Diaphragm => c.axis == crate::ConstraintAxis::Z,
            ConstraintType::Body
            | ConstraintType::Weld
            | ConstraintType::Equal
            | ConstraintType::Local => symmetric,
            _ => false,
        };
        if !ok {
            return Err(EditError::Unsupported(format!(
                "joint {name}: constraint {cname} is not invariant under rotation about Z"
            )));
        }
    }
    Ok(())
}

fn frame_checks(
    f: &FrameObj,
    tol: f64,
    center: [f64; 2],
    model: &SapModel,
) -> Result<(), EditError> {
    let name = &f.name;
    let bad = |what: &str| {
        Err(EditError::Unsupported(format!(
            "frame {name}: {what} is not invariant under rotation about Z"
        )))
    };
    if f.joint_offset_is_global
        && f.joint_offset_i
            .iter()
            .chain(&f.joint_offset_j)
            .any(|v| *v != 0.0)
    {
        return bad("a global joint offset");
    }
    if f.loads_gravity.values().any(|g| g[0] != 0.0 || g[1] != 0.0) {
        return bad("a gravity multiplier with X or Y component");
    }
    if f.loads_dist
        .iter()
        .any(|l| !load_dir_invariant(l.dir, &l.csys))
    {
        return bad("a distributed load in a global or projected X/Y direction");
    }
    if f.loads_point
        .iter()
        .any(|l| !load_dir_invariant(l.dir, &l.csys))
    {
        return bad("a point load in a global or projected X/Y direction");
    }
    let (pi, pj) = (&model.points[&f.point_i], &model.points[&f.point_j]);
    let on_axis = |p: &PointObj| (p.x - center[0]).abs() <= tol && (p.y - center[1]).abs() <= tol;
    if on_axis(pi) && on_axis(pj) {
        return Err(invalid(format!("frame {name} lies on the rotation axis")));
    }
    Ok(())
}

fn area_load_checks(a: &AreaObj) -> Result<(), EditError> {
    if a.loads_uniform
        .iter()
        .any(|l| !load_dir_invariant(l.dir, &l.csys))
    {
        return Err(EditError::Unsupported(format!(
            "area {}: a uniform load in a global or projected X/Y direction is not invariant under rotation about Z",
            a.name
        )));
    }
    Ok(())
}

/// Canonicalise the angle of `p` so that it compares equal to `q` when the two
/// differ by whole turns, or by rounding.
fn match_axes(p: &mut PointObj, q: &PointObj) {
    let d = (p.local_axes_deg[0] - q.local_axes_deg[0]).rem_euclid(360.0);
    if d < 1e-9 || 360.0 - d < 1e-9 {
        p.local_axes_deg[0] = q.local_axes_deg[0];
    }
    if let (Some(a), Some(b)) = (&mut p.local_axes_matrix, &q.local_axes_matrix) {
        let diff = (0..3)
            .flat_map(|i| (0..3).map(move |j| (i, j)))
            .map(|(i, j)| (a[i][j] - b[i][j]).abs())
            .fold(0.0, f64::max);
        if diff < 1e-12 {
            *a = *b;
        }
    }
}

impl SapModel {
    /// Replicate the selection about a vertical axis through `center`: copy `k`
    /// (1..=`copies`) is rotated by `k * angle_deg`, counter-clockwise seen from
    /// above. Assignments are copied; the local axes of joints, vertical frames
    /// and areas with a vertical normal are rotated with the geometry (see the
    /// module documentation). Joints that coincide with an existing joint
    /// within `1e-9 * max(1, largest |coordinate|)` are merged when their
    /// assignments agree, otherwise the command is refused. Named constraints
    /// are kept verbatim, as in [`SapModel::replicate_selected`].
    ///
    /// Refused as not invariant under the rotation: joint forces, moments or
    /// displacements with X/Y (MX/MY) components in global axes; restraints,
    /// springs or masses that differ between X and Y (RX and RY) on joints with
    /// global axes; coupled springs; frame or area loads in global or projected
    /// X/Y directions; global joint offsets; gravity multipliers with X/Y
    /// components; constraints other than Z diaphragms and symmetric Body, Weld,
    /// Equal and Local; selected links or solids; frames lying on the axis.
    pub fn replicate_radial(
        &mut self,
        center: [f64; 2],
        angle_deg: f64,
        copies: usize,
    ) -> Result<EditSummary, EditError> {
        let ids = self.selected_joints()?;
        finite(center)?;
        finite([angle_deg])?;
        if angle_deg == 0.0 || !(1..=10_000).contains(&copies) {
            return Err(invalid("nonzero angle and 1..=10000 copies required"));
        }
        let frames: Vec<FrameObj> = self
            .frames
            .values()
            .filter(|f| f.selected)
            .cloned()
            .collect();
        let areas: Vec<AreaObj> = self
            .areas
            .values()
            .filter(|a| a.selected)
            .cloned()
            .collect();
        let count = ids
            .len()
            .checked_add(frames.len())
            .and_then(|v| v.checked_add(areas.len()))
            .and_then(|v| v.checked_mul(copies));
        if count.is_none_or(|n| n > MAX_GENERATED) {
            return Err(invalid("edit exceeds one million generated objects"));
        }
        let reach = ids
            .iter()
            .map(|id| {
                let p = &self.points[id];
                (p.x - center[0]).abs().max((p.y - center[1]).abs())
            })
            .fold(0.0, f64::max);
        let extra = [[
            center[0].abs() + 2.0 * reach,
            center[1].abs() + 2.0 * reach,
            0.0,
        ]];
        let tol = geometry_tolerance(self, &extra);
        for id in &ids {
            point_checks(self, &self.points[id])?;
        }
        for f in &frames {
            frame_checks(f, tol, center, self)?;
        }
        for a in &areas {
            area_load_checks(a)?;
        }
        // Rotation class of each source frame and area, from the source geometry.
        let frame_sign: Vec<Option<f64>> = frames
            .iter()
            .map(|f| {
                vertical_frame_sign(
                    coords(&self.points[&f.point_i]),
                    coords(&self.points[&f.point_j]),
                )
            })
            .collect();
        let area_sign: Vec<Option<f64>> = areas
            .iter()
            .map(|a| {
                let c: Vec<[f64; 3]> = a.points.iter().map(|p| coords(&self.points[p])).collect();
                vertical_normal_sign(area_normal(&c))
            })
            .collect();

        let mut stage = Stage::new(self, &extra);
        for copy in 1..=copies {
            let theta = angle_deg * copy as f64;
            let sc = sin_cos_deg(theta);
            let mut mapping: HashMap<String, String> = HashMap::new();
            for id in &ids {
                let mut p = self.points[id].clone();
                let moved = rotate_about(center, coords(&p), sc);
                finite(moved)?;
                (p.x, p.y, p.z) = (moved[0], moved[1], moved[2]);
                if !identity_axes(&self.points[id]) {
                    if let Some(m) = &mut p.local_axes_matrix {
                        let old = *m;
                        let (s, c) = sc;
                        let rz = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]];
                        for i in 0..3 {
                            for j in 0..3 {
                                m[i][j] = rz[i][0] * old[0][j]
                                    + rz[i][1] * old[1][j]
                                    + rz[i][2] * old[2][j];
                            }
                        }
                    } else {
                        p.local_axes_deg[0] += theta;
                    }
                }
                let target = if let Some(target) = stage.find_joint(moved) {
                    let q = &stage.model.points[&target];
                    match_axes(&mut p, q);
                    if !same_assignments(&p, q) {
                        return Err(EditError::JointConflict(target));
                    }
                    target
                } else {
                    p.selected = true;
                    stage.insert_joint(p)?
                };
                mapping.insert(id.clone(), target);
            }
            for (original, sign) in frames.iter().zip(&frame_sign) {
                let mut f = original.clone();
                f.point_i = mapping[&original.point_i].clone();
                f.point_j = mapping[&original.point_j].clone();
                if f.point_i == f.point_j {
                    return Err(invalid(
                        "replication collapses a frame at coordinate precision",
                    ));
                }
                if let Some(s) = sign {
                    f.angle_deg += s * theta;
                }
                f.selected = true;
                if stage.insert_frame(f)?.is_none() {
                    return Err(invalid("replication would duplicate an existing frame"));
                }
            }
            for (original, sign) in areas.iter().zip(&area_sign) {
                let mut a = original.clone();
                a.points = original.points.iter().map(|p| mapping[p].clone()).collect();
                if a.points.iter().collect::<BTreeSet<_>>().len() != a.points.len() {
                    return Err(invalid(
                        "replication collapses an area at coordinate precision",
                    ));
                }
                if let Some(s) = sign {
                    a.angle_deg += s * theta;
                }
                a.selected = true;
                if stage.insert_area(a)?.is_none() {
                    return Err(invalid("replication would duplicate an existing area"));
                }
            }
        }
        if stage.summary.points_created.is_empty()
            && stage.summary.frames_created.is_empty()
            && stage.summary.areas_created.is_empty()
        {
            return Err(invalid("replication creates no objects"));
        }
        Ok(stage.commit(self))
    }

    /// Extrude every selected frame into quadrilateral areas by rotating it
    /// `n` times by `angle_deg` about a vertical axis through `center` while
    /// raising it by `total_rise / n` per step (a helicoid for a nonzero rise).
    /// Step `k` creates the area `(i[k-1], j[k-1], j[k], i[k])`, where `i[k]`
    /// and `j[k]` are the ends after `k` steps; its local 3 axis follows the
    /// right-hand rule applied to the direction `i->j` followed by the
    /// direction of rotation. An end on the axis (with no rise) gives a
    /// triangle. New joints reuse existing joints at the same location, so a
    /// full turn closes onto the first joints. With `delete_source` the selected
    /// frames are removed afterwards. Refused: a frame with both ends on the
    /// axis; a step that collapses an area; an area that already exists.
    pub fn extrude_lines_to_areas_radial(
        &mut self,
        center: [f64; 2],
        angle_deg: f64,
        n: usize,
        total_rise: f64,
        section: &str,
        delete_source: bool,
    ) -> Result<EditSummary, EditError> {
        self.check_edit()?;
        finite(center)?;
        finite([angle_deg, total_rise])?;
        if angle_deg == 0.0 || !(1..=10_000).contains(&n) {
            return Err(invalid("nonzero angle and 1..=10000 steps required"));
        }
        if !self.area_sections.contains_key(section) {
            return Err(invalid(format!("missing area section {section}")));
        }
        let selected: Vec<FrameObj> = self
            .frames
            .values()
            .filter(|f| f.selected)
            .cloned()
            .collect();
        if selected.is_empty() {
            return Err(EditError::EmptySelection);
        }
        if selected
            .len()
            .checked_mul(n)
            .is_none_or(|c| c > MAX_GENERATED)
        {
            return Err(invalid("edit exceeds one million generated objects"));
        }
        let mut reach = 0.0f64;
        for f in &selected {
            for id in [&f.point_i, &f.point_j] {
                let p = self
                    .points
                    .get(id)
                    .ok_or_else(|| EditError::MissingPoint(id.clone()))?;
                finite(coords(p))?;
                reach = reach.max((p.x - center[0]).abs().max((p.y - center[1]).abs()));
            }
        }
        let extra = [[
            center[0].abs() + 2.0 * reach,
            center[1].abs() + 2.0 * reach,
            total_rise.abs(),
        ]];
        let mut stage = Stage::new(self, &extra);
        let tol = stage.tol();
        let mut rings: HashMap<String, Vec<String>> = HashMap::new();
        for f in &selected {
            let (pi, pj) = (&self.points[&f.point_i], &self.points[&f.point_j]);
            let on_axis =
                |p: &PointObj| (p.x - center[0]).abs() <= tol && (p.y - center[1]).abs() <= tol;
            if on_axis(pi) && on_axis(pj) {
                return Err(invalid(format!(
                    "frame {} has both ends on the rotation axis",
                    f.name
                )));
            }
            for id in [&f.point_i, &f.point_j] {
                if rings.contains_key(id) {
                    continue;
                }
                let p0 = coords(&self.points[id]);
                let mut ring = vec![id.clone()];
                for k in 1..=n {
                    let sc = sin_cos_deg(angle_deg * k as f64);
                    let mut c = rotate_about(center, p0, sc);
                    c[2] = p0[2] + total_rise * k as f64 / n as f64;
                    finite(c)?;
                    ring.push(stage.add_joint(c)?);
                }
                rings.insert(id.clone(), ring);
            }
            let (ri, rj) = (&rings[&f.point_i], &rings[&f.point_j]);
            for k in 1..=n {
                let (i0, j0, j1, i1) = (&ri[k - 1], &rj[k - 1], &rj[k], &ri[k]);
                let pts: Vec<String> = if i0 == i1 && j0 == j1 {
                    return Err(invalid("extrusion step collapses an area"));
                } else if i0 == i1 {
                    vec![i0.clone(), j0.clone(), j1.clone()]
                } else if j0 == j1 {
                    vec![i0.clone(), j0.clone(), i1.clone()]
                } else {
                    vec![i0.clone(), j0.clone(), j1.clone(), i1.clone()]
                };
                if pts.iter().collect::<BTreeSet<_>>().len() != pts.len() {
                    return Err(invalid("extrusion step collapses an area"));
                }
                let corners: Vec<[f64; 3]> =
                    pts.iter().map(|p| coords(&stage.model.points[p])).collect();
                if area_degenerate(&corners) {
                    return Err(invalid(
                        "extrusion step has zero area (its joints are collinear)",
                    ));
                }
                match stage.add_area(pts, section)? {
                    Some(name) => {
                        stage
                            .model
                            .areas
                            .get_mut(&name)
                            .expect("just inserted")
                            .selected = true;
                    }
                    None => return Err(invalid("extrusion would duplicate an existing area")),
                }
            }
        }
        if delete_source {
            for f in &selected {
                stage.model.frames.shift_remove(&f.name);
                stage.summary.frames_removed.push(f.name.clone());
            }
        }
        Ok(stage.commit(self))
    }

    /// Reverse the joint order of every selected area, which reverses its local
    /// 3 axis (`[p0..pn]` becomes `[pn..p0]`). The in-plane local 1 axis keeps
    /// its direction and local 2 reverses (the angle is replaced by `180 - a`
    /// for a normal that is not vertical and `-a` for a vertical normal, `a`
    /// being the section material angle plus `angle_deg`).
    ///
    /// Assignments that depend on local 3 are transformed:
    ///
    /// * `SameGlobalLoads`: uniform loads in local 2 and 3 change sign, local 1
    ///   and all global/gravity loads are unchanged; surface pressure on the
    ///   top face (-2) moves to the bottom face (-1) and vice versa with the same
    ///   value; a through-thickness temperature gradient (type 2) changes sign.
    /// * `SameLocalLoads`: every assignment keeps its numbers and local labels
    ///   and so turns with the axes.
    ///
    /// Edge pressures (faces 1..4) are refused in both modes: how an edge label
    /// follows the reversed joint order is not defined by the solver (which does
    /// not implement edge pressure) or the help topics.
    ///
    /// Modifiers, mesh divisions, mass and the other assignments do not depend
    /// on the sense of local 3. Only 3- and 4-joint areas are accepted.
    /// Returns the number of areas reversed.
    pub fn reverse_area_local_3(&mut self, keep: ReverseKeep) -> Result<usize, EditError> {
        self.check_edit()?;
        let names: Vec<String> = self
            .areas
            .values()
            .filter(|a| a.selected)
            .map(|a| a.name.clone())
            .collect();
        if names.is_empty() {
            return Err(EditError::EmptySelection);
        }
        let mut staged = self.clone();
        for name in &names {
            let a = &self.areas[name];
            let unsupported =
                |what: &str| Err(EditError::Unsupported(format!("area {name}: {what}")));
            if !(3..=4).contains(&a.points.len()) {
                return unsupported("only 3- and 4-joint areas can be reversed");
            }
            finite([a.angle_deg])?;
            let sec = self
                .area_sections
                .get(&a.section)
                .ok_or_else(|| invalid(format!("area {name}: missing section {}", a.section)))?;
            finite([sec.mat_angle_deg])?;
            let old: Vec<[f64; 3]> = a
                .points
                .iter()
                .map(|p| {
                    self.points
                        .get(p)
                        .map(coords)
                        .ok_or_else(|| EditError::MissingPoint(p.clone()))
                })
                .collect::<Result<_, _>>()?;
            if old.iter().any(|c| c.iter().any(|v| !v.is_finite())) {
                return Err(invalid("non-finite joint coordinate"));
            }
            let mut new_pts = old.clone();
            new_pts.reverse();
            let (e_old, e_new) = (area_normal(&old), area_normal(&new_pts));
            let class = match (vertical_normal_sign(e_old), vertical_normal_sign(e_new)) {
                (None, None) => None,
                (Some(a), Some(b)) if a == -b => Some(()),
                _ => {
                    return unsupported("local 3 is too close to vertical to classify its reversal")
                }
            };
            let mut area = a.clone();
            area.points.reverse();
            let total = a.angle_deg + sec.mat_angle_deg;
            let reversed_total = if class.is_some() {
                -total
            } else {
                180.0 - total
            };
            area.angle_deg = reversed_total - sec.mat_angle_deg;
            finite([area.angle_deg])?;
            // Edge numbering under reversal is not evidenced by the solver (it
            // refuses edge pressures) or by the help topics, so edge pressures
            // are refused in both modes rather than guessed.
            if area
                .loads_pressure
                .iter()
                .any(|l| !matches!(l.face, -1 | -2))
            {
                return unsupported("edge pressure cannot be transformed exactly");
            }
            match keep {
                ReverseKeep::SameGlobalLoads => {
                    for l in &mut area.loads_uniform {
                        if matches!(l.dir, 2 | 3) {
                            l.val = -l.val;
                        }
                        if !(1..=11).contains(&l.dir) {
                            return unsupported("uniform load in an unknown direction");
                        }
                    }
                    for l in &mut area.loads_pressure {
                        l.face = match l.face {
                            -1 => -2,
                            -2 => -1,
                            _ => return unsupported("pressure on an unknown face"),
                        };
                    }
                    for l in &mut area.loads_temp {
                        match l.my_type {
                            2 => l.val = -l.val,
                            1 => {}
                            _ => return unsupported("temperature load of unknown type"),
                        }
                    }
                }
                ReverseKeep::SameLocalLoads => {
                    if area
                        .loads_uniform
                        .iter()
                        .any(|l| !(1..=11).contains(&l.dir))
                    {
                        return unsupported("uniform load in an unknown direction");
                    }
                    if area.loads_temp.iter().any(|l| !matches!(l.my_type, 1 | 2)) {
                        return unsupported("temperature load of unknown type");
                    }
                }
            }
            staged.areas.insert(name.clone(), area);
        }
        staged.invalidate_results();
        *self = staged;
        Ok(names.len())
    }

    /// Draw `n` straight frames approximating a circular or parabolic curve
    /// from `start` to `end` through `third_point` (CSI "Curved Frame
    /// Geometry"). Frames run from `start` to `end`; existing joints at the
    /// computed points are reused.
    ///
    /// * `Circular`: the arc of the circle through the three points that runs
    ///   from `start` to `end` and contains `third_point`, divided into `n`
    ///   equal angles about the circle centre.
    /// * `Parabolic`: with `u` the chord direction, `L` the chord length and `t`
    ///   the chord coordinate of `third_point`, the curve is
    ///   `start + s u + w s (L - s) / (t (L - t))` where `w` is the part of
    ///   `third_point - start` perpendicular to the chord; the `n + 1` points use
    ///   equal chord steps `s = k L / n`.
    ///
    /// Refused: fewer than 2 segments; collinear points; a parabolic third point
    /// whose chord coordinate is not strictly between the ends.
    pub fn curved_frame(
        &mut self,
        start: [f64; 3],
        end: [f64; 3],
        third_point: [f64; 3],
        kind: CurveKind,
        n: usize,
        section: &str,
    ) -> Result<EditSummary, EditError> {
        self.check_edit()?;
        finite(start)?;
        finite(end)?;
        finite(third_point)?;
        if !(2..=10_000).contains(&n) {
            return Err(invalid("2..=10000 segments required"));
        }
        if !self.frame_sections.contains_key(section) {
            return Err(invalid(format!("missing frame section {section}")));
        }
        let sub = |a: [f64; 3], b: [f64; 3]| -> [f64; 3] { std::array::from_fn(|k| a[k] - b[k]) };
        let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        let cross = |a: [f64; 3], b: [f64; 3]| -> [f64; 3] {
            [
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0],
            ]
        };
        let norm = |a: [f64; 3]| dot(a, a).sqrt();
        let ab = sub(third_point, start);
        let ac = sub(end, start);
        let nrm = cross(ab, ac);
        if norm(ab) == 0.0 || norm(ac) == 0.0 || norm(nrm) <= 1e-12 * norm(ab) * norm(ac) {
            return Err(invalid(
                "the three points of a curved frame must not be collinear",
            ));
        }
        let mut pts = vec![start];
        match kind {
            CurveKind::Circular => {
                let (ab2, ac2, nn) = (dot(ab, ab), dot(ac, ac), dot(nrm, nrm));
                // Circumcentre (Euclidean geometry, 3D form).
                let t1 = cross(cross(ab, ac), ab);
                let t2 = cross(cross(ac, ab), ac);
                let center: [f64; 3] =
                    std::array::from_fn(|k| start[k] + (ac2 * t1[k] + ab2 * t2[k]) / (2.0 * nn));
                let r_vec = sub(start, center);
                let radius = norm(r_vec);
                let ex: [f64; 3] = std::array::from_fn(|k| r_vec[k] / radius);
                let nhat: [f64; 3] = {
                    let l = norm(nrm);
                    std::array::from_fn(|k| nrm[k] / l)
                };
                let ey = cross(nhat, ex);
                let ang = |p: [f64; 3]| {
                    let v = sub(p, center);
                    dot(v, ey)
                        .atan2(dot(v, ex))
                        .rem_euclid(std::f64::consts::TAU)
                };
                let sweep = ang(end);
                if !(sweep.is_finite() && sweep > 0.0) {
                    return Err(invalid("circular curve has no sweep"));
                }
                for k in 1..n {
                    let phi = sweep * k as f64 / n as f64;
                    let (s, c) = phi.sin_cos();
                    pts.push(std::array::from_fn(|q| {
                        center[q] + radius * (c * ex[q] + s * ey[q])
                    }));
                }
            }
            CurveKind::Parabolic => {
                let len = norm(ac);
                let u: [f64; 3] = std::array::from_fn(|k| ac[k] / len);
                let t = dot(ab, u);
                if !(t > 0.0 && t < len) {
                    return Err(invalid(
                        "the third point of a parabolic curve must lie strictly between the ends along the chord",
                    ));
                }
                let w: [f64; 3] = std::array::from_fn(|k| ab[k] - t * u[k]);
                let denom = t * (len - t);
                for k in 1..n {
                    let s = len * k as f64 / n as f64;
                    let off = s * (len - s) / denom;
                    pts.push(std::array::from_fn(|q| start[q] + s * u[q] + w[q] * off));
                }
            }
        }
        pts.push(end);
        for p in &pts {
            finite(*p)?;
        }
        let mut stage = Stage::new(self, &pts);
        let mut ids = Vec::with_capacity(pts.len());
        for p in &pts {
            ids.push(stage.add_joint(*p)?);
        }
        for w in ids.windows(2) {
            if stage.add_frame(&w[0], &w[1], section, false)?.is_none() {
                return Err(invalid("curved frame would duplicate an existing frame"));
            }
        }
        Ok(stage.commit(self))
    }
}
