//! Transactional model editing. These are native Rust operations, not COM contracts.
use crate::{next_numeric_name, FrameObj, FramePropType, PointObj, SapModel};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct EditSummary {
    pub points_created: Vec<String>,
    pub frames_created: Vec<String>,
    pub areas_created: Vec<String>,
    pub frames_removed: Vec<String>,
    pub points_moved: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum EditError {
    #[error("model is locked; unlock it before editing")]
    Locked,
    #[error("no supported objects are selected")]
    EmptySelection,
    #[error("invalid edit: {0}")]
    Invalid(String),
    #[error("unsupported edit assignment: {0}")]
    Unsupported(String),
    #[error("missing joint {0}")]
    MissingPoint(String),
    #[error("joint overlap has incompatible assignments: {0}")]
    JointConflict(String),
}

pub(crate) fn invalid(message: impl Into<String>) -> EditError {
    EditError::Invalid(message.into())
}
pub(crate) fn finite(values: impl IntoIterator<Item = f64>) -> Result<(), EditError> {
    if values.into_iter().all(f64::is_finite) {
        Ok(())
    } else {
        Err(invalid("non-finite value"))
    }
}
pub(crate) fn coords(p: &PointObj) -> [f64; 3] {
    [p.x, p.y, p.z]
}
pub(crate) fn validate_point(p: &PointObj) -> Result<(), EditError> {
    finite(coords(p))?;
    finite(p.spring)?;
    finite(p.mass)?;
    finite(p.local_axes_deg)?;
    if let Some(v) = p.spring_coupled {
        finite(v)?;
    }
    if let Some(v) = p.local_axes_matrix {
        finite(v.into_iter().flatten())?;
    }
    finite(p.pattern_values.values().copied())?;
    for l in &p.loads_force {
        finite(l.force)?;
    }
    for l in &p.loads_displ {
        finite(l.displ)?;
    }
    Ok(())
}
pub(crate) fn validate_frame(f: &FrameObj) -> Result<(), EditError> {
    finite([
        f.angle_deg,
        f.end_offset_i,
        f.end_offset_j,
        f.rigid_zone_factor,
        f.pdelta_force,
    ])?;
    finite(f.modifiers)?;
    finite(f.start_spring)?;
    finite(f.end_spring)?;
    finite(f.joint_offset_i)?;
    finite(f.joint_offset_j)?;
    if let Some((_, c, _, t)) = f.tc_limits {
        finite([c, t])?;
    }
    if let Some(c) = &f.cable_params {
        finite(
            [
                c.undeformed_length,
                c.target_tension_i,
                c.horizontal_tension,
            ]
            .into_iter()
            .flatten(),
        )?;
    }
    for l in &f.loads_dist {
        finite([l.rel_dist1, l.rel_dist2, l.val1, l.val2])?;
        if !(0.0..=1.0).contains(&l.rel_dist1)
            || !(0.0..=1.0).contains(&l.rel_dist2)
            || l.rel_dist1 >= l.rel_dist2
        {
            return Err(invalid(format!(
                "frame {} has invalid distributed load interval",
                f.name
            )));
        }
    }
    for l in &f.loads_point {
        finite([l.rel_dist, l.val])?;
        if !(0.0..=1.0).contains(&l.rel_dist) {
            return Err(invalid("point load station outside frame"));
        }
    }
    for l in &f.loads_temp {
        finite([l.val])?;
    }
    for v in f.loads_gravity.values() {
        finite(*v)?;
    }
    Ok(())
}
pub(crate) fn same_assignments(a: &PointObj, b: &PointObj) -> bool {
    let mut normalized = a.clone();
    normalized.name.clone_from(&b.name);
    normalized.x = b.x;
    normalized.y = b.y;
    normalized.z = b.z;
    normalized.selected = b.selected;
    normalized == *b
}

impl SapModel {
    /// Clear stale analysis and design output after an accepted edit.
    pub fn invalidate_results(&mut self) {
        self.case_results.clear();
        self.unavailable_results.clear();
        self.steel_design_results.clear();
        self.steel_design_errors.clear();
        self.concrete_design_results.clear();
        self.concrete_design_errors.clear();
    }

    pub(crate) fn check_edit(&self) -> Result<(), EditError> {
        if self.is_locked {
            return Err(EditError::Locked);
        }
        Ok(())
    }

    pub(crate) fn selected_joints(&self) -> Result<BTreeSet<String>, EditError> {
        self.check_edit()?;
        if self.links.values().any(|v| v.selected) || self.solids.values().any(|v| v.selected) {
            return Err(EditError::Unsupported("selected links or solids".into()));
        }
        let mut ids: BTreeSet<_> = self
            .points
            .iter()
            .filter(|(_, p)| p.selected)
            .map(|(id, _)| id.clone())
            .collect();
        for f in self.frames.values().filter(|f| f.selected) {
            validate_frame(f)?;
            ids.extend([f.point_i.clone(), f.point_j.clone()]);
        }
        for a in self.areas.values().filter(|a| a.selected) {
            finite([a.angle_deg])?;
            finite(a.modifiers)?;
            for l in &a.loads_uniform {
                finite([l.val])?;
            }
            for l in &a.loads_pressure {
                finite([l.val])?;
            }
            for l in &a.loads_temp {
                finite([l.val])?;
            }
            ids.extend(a.points.iter().cloned());
        }
        if ids.is_empty() {
            return Err(EditError::EmptySelection);
        }
        for id in &ids {
            validate_point(
                self.points
                    .get(id)
                    .ok_or_else(|| EditError::MissingPoint(id.clone()))?,
            )?;
        }
        for f in self.frames.values().filter(|f| f.selected) {
            let a = coords(&self.points[&f.point_i]);
            let b = coords(&self.points[&f.point_j]);
            let d: [f64; 3] = std::array::from_fn(|k| b[k] - a[k]);
            finite(d)?;
            if a == b {
                return Err(invalid(format!("frame {} has zero length", f.name)));
            }
        }
        for a in self.areas.values().filter(|a| a.selected) {
            if a.points.len() < 3
                || a.points.iter().collect::<BTreeSet<_>>().len() != a.points.len()
            {
                return Err(invalid(format!("area {} has invalid topology", a.name)));
            }
        }
        Ok(ids)
    }

    /// Translate selected joints and joints referenced by selected frames/areas once.
    /// Shared joints remain shared: unselected objects attached to them follow the move.
    /// Distinct joints may not be moved onto each other; no automatic merge is performed.
    pub fn move_selected(&mut self, delta: [f64; 3]) -> Result<EditSummary, EditError> {
        let ids = self.selected_joints()?;
        finite(delta)?;
        if delta == [0.0; 3] {
            return Err(invalid("translation must be nonzero"));
        }
        let mut staged = self.clone();
        for id in &ids {
            let p = staged.points.get_mut(id).expect("selection validated");
            let old = coords(p);
            p.x += delta[0];
            p.y += delta[1];
            p.z += delta[2];
            finite(coords(p))?;
            if coords(p) == old {
                return Err(invalid("translation is below coordinate precision"));
            }
        }
        for id in &ids {
            let p = &staged.points[id];
            if staged
                .points
                .iter()
                .any(|(other, q)| other != id && coords(p) == coords(q))
            {
                return Err(EditError::JointConflict(id.clone()));
            }
        }
        staged.invalidate_results();
        *self = staged;
        Ok(EditSummary {
            points_moved: ids.into_iter().collect(),
            ..Default::default()
        })
    }

    /// Make `copies` translated copies at delta, 2*delta, ... . All object and
    /// joint assignments are cloned; originals retain selection and copies are selected.
    /// Exact coincident joints are reused only if all assignments agree. Named
    /// constraints are preserved verbatim, so copies remain in the original constraint.
    pub fn replicate_selected(
        &mut self,
        delta: [f64; 3],
        copies: usize,
    ) -> Result<EditSummary, EditError> {
        let ids = self.selected_joints()?;
        finite(delta)?;
        if delta == [0.0; 3] || !(1..=10_000).contains(&copies) {
            return Err(invalid("nonzero translation and 1..=10000 copies required"));
        }
        let frames: Vec<_> = self
            .frames
            .values()
            .filter(|f| f.selected)
            .cloned()
            .collect();
        let areas: Vec<_> = self
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
        if count.is_none_or(|n| n > 1_000_000) {
            return Err(invalid("edit exceeds one million generated objects"));
        }
        let mut staged = self.clone();
        let mut summary = EditSummary::default();
        for copy in 1..=copies {
            let mut mapping = BTreeMap::new();
            for id in &ids {
                let mut p = self.points[id].clone();
                let old = coords(&p);
                p.x += delta[0] * copy as f64;
                p.y += delta[1] * copy as f64;
                p.z += delta[2] * copy as f64;
                finite(coords(&p))?;
                if coords(&p) == old {
                    return Err(invalid("translation is below coordinate precision"));
                }
                let overlap = staged.points.iter().find(|(_, q)| coords(q) == coords(&p));
                let target = if let Some((target, q)) = overlap {
                    if !same_assignments(&p, q) {
                        return Err(EditError::JointConflict(target.clone()));
                    }
                    target.clone()
                } else {
                    p.name = next_numeric_name(staged.points.keys());
                    p.selected = true;
                    let target = p.name.clone();
                    summary.points_created.push(target.clone());
                    staged.points.insert(target.clone(), p);
                    target
                };
                mapping.insert(id.clone(), target);
            }
            for original in &frames {
                let mut f = original.clone();
                f.point_i = mapping[&original.point_i].clone();
                f.point_j = mapping[&original.point_j].clone();
                if coords(&staged.points[&f.point_i]) == coords(&staged.points[&f.point_j]) {
                    return Err(invalid(
                        "replication collapses a frame at coordinate precision",
                    ));
                }
                if staged.frames.values().any(|other| {
                    (other.point_i == f.point_i && other.point_j == f.point_j)
                        || (other.point_i == f.point_j && other.point_j == f.point_i)
                }) {
                    return Err(invalid("replication would duplicate an existing frame"));
                }
                f.name = next_numeric_name(staged.frames.keys());
                f.selected = true;
                summary.frames_created.push(f.name.clone());
                staged.frames.insert(f.name.clone(), f);
            }
            for original in &areas {
                let mut a = original.clone();
                a.points = original.points.iter().map(|p| mapping[p].clone()).collect();
                if a.points.iter().enumerate().any(|(i, p)| {
                    a.points[..i]
                        .iter()
                        .any(|q| coords(&staged.points[p]) == coords(&staged.points[q]))
                }) {
                    return Err(invalid(
                        "replication collapses an area at coordinate precision",
                    ));
                }
                let set: BTreeSet<_> = a.points.iter().collect();
                if staged.areas.values().any(|other| {
                    other.points.len() == a.points.len()
                        && other.points.iter().collect::<BTreeSet<_>>() == set
                }) {
                    return Err(invalid("replication would duplicate an existing area"));
                }
                a.name = next_numeric_name(staged.areas.keys());
                a.selected = true;
                summary.areas_created.push(a.name.clone());
                staged.areas.insert(a.name.clone(), a);
            }
        }
        if summary.points_created.is_empty()
            && summary.frames_created.is_empty()
            && summary.areas_created.is_empty()
        {
            return Err(invalid("replication creates no objects"));
        }
        staged.invalidate_results();
        *self = staged;
        Ok(summary)
    }

    /// Refusals shared by every command that replaces a frame by pieces, and
    /// the frame's end `a`, its end-to-end vector and its length.
    fn check_divisible(&self, f: &FrameObj) -> Result<([f64; 3], [f64; 3], f64), EditError> {
        validate_frame(f)?;
        let section = self
            .frame_sections
            .get(&f.section)
            .ok_or_else(|| invalid(format!("missing section {}", f.section)))?;
        if section.prop_type == FramePropType::Variable
            || !section.non_prismatic.is_empty()
            || f.cable_params.is_some()
            || f.tc_limits.is_some()
            || f.end_offset_i != 0.0
            || f.end_offset_j != 0.0
            || f.joint_offset_i != [0.0; 3]
            || f.joint_offset_j != [0.0; 3]
            || f.cardinal_point != 10
            || f.loads_temp.iter().any(|l| l.pattern_name.is_some())
        {
            return Err(EditError::Unsupported(format!("frame {}: nonprismatic/cable/limits/offsets/cardinal point/patterned temperature", f.name)));
        }
        // Every piece is a copy of the object, so its hinges would be
        // repeated on each piece at the same relative distance.
        if !f.hinges.is_empty() || f.hinge_overwrite.is_some() {
            return Err(EditError::Unsupported(format!(
                "frame {}: hinge assignments or a hinge overwrite",
                f.name
            )));
        }
        let pi = self
            .points
            .get(&f.point_i)
            .ok_or_else(|| EditError::MissingPoint(f.point_i.clone()))?;
        let pj = self
            .points
            .get(&f.point_j)
            .ok_or_else(|| EditError::MissingPoint(f.point_j.clone()))?;
        validate_point(pi)?;
        validate_point(pj)?;
        let a = coords(pi);
        let b = coords(pj);
        let distance: [f64; 3] = std::array::from_fn(|k| b[k] - a[k]);
        finite(distance)?;
        let length = distance[0].hypot(distance[1]).hypot(distance[2]);
        if !length.is_finite() || length <= 0.0 {
            return Err(invalid("zero/nonfinite frame length"));
        }
        Ok((a, distance, length))
    }

    /// Replace selected prismatic frames with equal geometric segments. Original
    /// endpoint joints/assignments survive; intermediate joints have no assignments.
    /// End releases/springs stay at the original ends. Distributed loads are clipped
    /// and interpolated, point loads occur on exactly one child (right child at a
    /// split), uniform temperature and gravity assignments apply to each child.
    pub fn divide_selected_frames(&mut self, segments: usize) -> Result<EditSummary, EditError> {
        self.check_edit()?;
        if !(2..=10_000).contains(&segments) {
            return Err(invalid("2..=10000 segments required"));
        }
        let rels: Vec<f64> = (1..segments)
            .map(|i| i as f64 / segments as f64)
            .collect();
        self.split_at(&rels)
    }

    /// Divide each selected frame into `segments` pieces whose lengths form a
    /// geometric series with `last_to_first` the ratio of the last length to
    /// the first (SAP2000 "Divide into specified number of frames" with a
    /// last/first length ratio). A ratio of 1 is the equal division.
    ///
    /// With `q = r^(1/(n-1))` the pieces are `l1 q^k` and the interior joints
    /// lie at the relative distances `(q^k - 1) / (q^n - 1)`, k = 1..n-1.
    pub fn divide_selected_frames_ratio(
        &mut self,
        segments: usize,
        last_to_first: f64,
    ) -> Result<EditSummary, EditError> {
        self.check_edit()?;
        if !(2..=10_000).contains(&segments) {
            return Err(invalid("2..=10000 segments required"));
        }
        if !last_to_first.is_finite() || last_to_first <= 0.0 {
            return Err(invalid(
                "the last/first length ratio must be positive and finite",
            ));
        }
        if last_to_first == 1.0 {
            return self.divide_selected_frames(segments);
        }
        let ln_q = last_to_first.ln() / (segments - 1) as f64;
        let denominator = (segments as f64 * ln_q).exp_m1();
        let rels: Vec<f64> = (1..segments)
            .map(|k| (k as f64 * ln_q).exp_m1() / denominator)
            .collect();
        if rels
            .iter()
            .any(|r| !r.is_finite() || !(0.0..1.0).contains(r))
            || rels.windows(2).any(|w| w[1] <= w[0])
            || rels[0] <= 0.0
        {
            return Err(invalid(format!(
                "ratio {last_to_first} over {segments} segments is not representable"
            )));
        }
        self.split_at(&rels)
    }

    /// Replace each selected frame by pieces that end at the relative distances
    /// `rels` (strictly increasing, inside 0..1) from its I end.
    fn split_at(&mut self, rels: &[f64]) -> Result<EditSummary, EditError> {
        self.check_edit()?;
        let segments = rels.len() + 1;
        let selected: Vec<_> = self
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
            .checked_mul(segments)
            .is_none_or(|n| n > 1_000_000)
        {
            return Err(invalid("edit exceeds one million generated frames"));
        }
        // Equal pieces keep the arithmetic of the equal division (a product by
        // the number of segments); other pieces divide by their own span.
        let uniform = rels
            .iter()
            .enumerate()
            .all(|(i, r)| *r == (i + 1) as f64 / segments as f64);
        let mut bounds = vec![0.0];
        bounds.extend_from_slice(rels);
        bounds.push(1.0);
        let mut staged = self.clone();
        let mut summary = EditSummary::default();
        for f in &selected {
            let (a, distance, length) = self.check_divisible(f)?;
            if !uniform {
                let tol = crate::generate::geometry_tolerance(self, &[]);
                if bounds.windows(2).any(|w| (w[1] - w[0]) * length <= tol) {
                    return Err(invalid(format!(
                        "frame {}: a segment is shorter than the coordinate tolerance",
                        f.name
                    )));
                }
            }
            let mut joints = vec![f.point_i.clone()];
            for (i, rel) in rels.iter().enumerate() {
                let c = std::array::from_fn(|k| a[k] + distance[k] * rel);
                finite(c)?;
                // Reuse existing geometric joints (their assignments remain unchanged).
                let id = if let Some((id, _)) = staged.points.iter().find(|(_, p)| coords(p) == c) {
                    id.clone()
                } else {
                    let id = next_numeric_name(staged.points.keys());
                    let mut p = PointObj::new(&id, c[0], c[1], c[2]);
                    p.selected = true;
                    staged.points.insert(id.clone(), p);
                    summary.points_created.push(id.clone());
                    id
                };
                if joints.contains(&id) || id == f.point_j {
                    return Err(invalid("segments collapse at coordinate precision"));
                }
                debug_assert_eq!(joints.len(), i + 1);
                joints.push(id);
            }
            joints.push(f.point_j.clone());
            make_children(&mut staged, f, &joints, &bounds, uniform, &mut summary)?;
        }
        staged.invalidate_results();
        *self = staged;
        Ok(summary)
    }

    /// Break the selected frames where selected joints lie on them and where
    /// they meet other selected frames (SAP2000 "Break at intersections with
    /// selected joints, frames"). A frame splits at a selected joint that lies
    /// within the geometry tolerance of its interior, and at the point where
    /// another selected frame comes within the tolerance of it, also when that
    /// frame ends there (a T-junction splits only the frame that is crossed).
    /// Pieces share the joint at the break; the joint of an intersection is
    /// created unless one exists there. Selected areas, solids and links are
    /// refused, as are collinear overlapping frames and frames the divide
    /// command refuses (checked on the frames that would split).
    pub fn break_selected_frames_at_intersections(&mut self) -> Result<EditSummary, EditError> {
        self.check_edit()?;
        if self.areas.values().any(|a| a.selected) || self.solids.values().any(|s| s.selected) {
            return Err(EditError::Unsupported(
                "breaking at area or solid edges is not implemented".into(),
            ));
        }
        if self.links.values().any(|l| l.selected) {
            return Err(EditError::Unsupported(
                "breaking with selected links is not implemented".into(),
            ));
        }
        let selected: Vec<_> = self
            .frames
            .values()
            .filter(|f| f.selected)
            .cloned()
            .collect();
        if selected.is_empty() {
            return Err(EditError::EmptySelection);
        }
        let joints: Vec<(String, [f64; 3])> = self
            .points
            .iter()
            .filter(|(_, p)| p.selected)
            .map(|(n, p)| (n.clone(), coords(p)))
            .collect();
        for (n, c) in &joints {
            finite(*c).map_err(|_| invalid(format!("joint {n} has a nonfinite coordinate")))?;
        }
        let mut ends = Vec::with_capacity(selected.len());
        for f in &selected {
            let pi = self
                .points
                .get(&f.point_i)
                .ok_or_else(|| EditError::MissingPoint(f.point_i.clone()))?;
            let pj = self
                .points
                .get(&f.point_j)
                .ok_or_else(|| EditError::MissingPoint(f.point_j.clone()))?;
            let (a, b) = (coords(pi), coords(pj));
            let u = sub(b, a);
            finite(a)?;
            finite(u)?;
            if norm(u) <= 0.0 || !norm(u).is_finite() {
                return Err(invalid(format!("frame {} has zero length", f.name)));
            }
            ends.push((a, u));
        }
        let tol = crate::generate::geometry_tolerance(self, &[]);

        // Plan on the original geometry: per frame, the relative distance of
        // every break and where it goes.
        let mut plan: Vec<Vec<(f64, Split)>> = vec![Vec::new(); selected.len()];
        for (k, f) in selected.iter().enumerate() {
            let (a, u) = ends[k];
            let len = norm(u);
            let interior = |s: f64| s * len > tol && (1.0 - s) * len > tol;
            for (name, c) in &joints {
                if *name == f.point_i || *name == f.point_j {
                    continue;
                }
                let s = dot(sub(*c, a), u) / dot(u, u);
                let foot: [f64; 3] = std::array::from_fn(|d| a[d] + u[d] * s);
                if interior(s) && norm(sub(*c, foot)) <= tol {
                    plan[k].push((s, Split::Joint(name.clone())));
                }
            }
            for (m, g) in selected.iter().enumerate() {
                if m == k {
                    continue;
                }
                let (c0, v) = ends[m];
                let lv = norm(v);
                let sin = norm(cross(u, v)) / (len * lv);
                if sin * len.min(lv) <= tol {
                    // Parallel: only a collinear overlap is a problem.
                    let off = |p: [f64; 3]| {
                        let s = dot(sub(p, a), u) / dot(u, u);
                        let foot: [f64; 3] = std::array::from_fn(|d| a[d] + u[d] * s);
                        (s, norm(sub(p, foot)))
                    };
                    let (s0, d0) = off(c0);
                    let (s1, d1) = off(std::array::from_fn(|d| c0[d] + v[d]));
                    if d0 <= tol && d1 <= tol {
                        let overlap = (s0.max(s1).min(1.0) - s0.min(s1).max(0.0)) * len;
                        if overlap > tol {
                            return Err(invalid(format!(
                                "frames {} and {} overlap along a common line; they cannot be broken",
                                f.name, g.name
                            )));
                        }
                    }
                    continue;
                }
                let (s, _t, dist) = segment_closest(a, u, c0, v);
                if dist <= tol && interior(s) {
                    let p = std::array::from_fn(|d| a[d] + u[d] * s);
                    plan[k].push((s, Split::Point(p)));
                }
            }
            plan[k].sort_by(|x, y| x.0.total_cmp(&y.0));
            // Merge breaks that fall on the same place; two different selected
            // joints that close are a refusal.
            let mut merged: Vec<(f64, Split)> = Vec::new();
            for (s, split) in std::mem::take(&mut plan[k]) {
                if let Some((s0, last)) = merged.last_mut() {
                    if (s - *s0) * len <= tol {
                        match (&*last, &split) {
                            (Split::Joint(x), Split::Joint(y)) if x != y => {
                                return Err(invalid(format!(
                                    "frame {}: joints {x} and {y} lie within the tolerance of each other",
                                    f.name
                                )));
                            }
                            (Split::Point(_), Split::Joint(_)) => {
                                *s0 = s;
                                *last = split;
                            }
                            _ => {}
                        }
                        continue;
                    }
                }
                merged.push((s, split));
            }
            plan[k] = merged;
        }
        if plan.iter().all(Vec::is_empty) {
            return Err(invalid(
                "no selected joint lies on a selected frame and no selected frames intersect",
            ));
        }
        let count: usize = plan.iter().map(|p| p.len() + 1).sum();
        if count > 1_000_000 {
            return Err(invalid("edit exceeds one million generated frames"));
        }

        let mut stage = crate::generate::Stage::new(self, &[]);
        let mut pairs: BTreeSet<(String, String)> = self
            .frames
            .values()
            .map(|f| pair(&f.point_i, &f.point_j))
            .collect();
        for (k, f) in selected.iter().enumerate() {
            if plan[k].is_empty() {
                continue;
            }
            self.check_divisible(f)?;
            let mut names = vec![f.point_i.clone()];
            let mut bounds = vec![0.0];
            for (s, split) in &plan[k] {
                let name = match split {
                    Split::Joint(name) => name.clone(),
                    Split::Point(p) => {
                        let before = stage.summary.points_created.len();
                        let name = stage.add_joint(*p)?;
                        if stage.summary.points_created.len() > before {
                            stage
                                .model
                                .points
                                .get_mut(&name)
                                .expect("joint just created")
                                .selected = true;
                        }
                        name
                    }
                };
                if names.contains(&name) || name == f.point_j {
                    return Err(invalid(format!(
                        "frame {}: breaks collapse at coordinate precision",
                        f.name
                    )));
                }
                names.push(name);
                bounds.push(*s);
            }
            names.push(f.point_j.clone());
            bounds.push(1.0);
            pairs.remove(&pair(&f.point_i, &f.point_j));
            for w in names.windows(2) {
                if !pairs.insert(pair(&w[0], &w[1])) {
                    return Err(invalid(format!(
                        "frame {}: a piece would duplicate an existing frame between joints {} and {}",
                        f.name, w[0], w[1]
                    )));
                }
            }
            let mut summary = std::mem::take(&mut stage.summary);
            let result = make_children(&mut stage.model, f, &names, &bounds, false, &mut summary);
            stage.summary = summary;
            result?;
        }
        Ok(stage.commit(self))
    }
}

/// Where a frame breaks: at an existing joint, or at a point that gets a joint.
#[derive(Debug, Clone)]
enum Split {
    Joint(String),
    Point([f64; 3]),
}

fn pair(a: &str, b: &str) -> (String, String) {
    if a <= b {
        (a.to_string(), b.to_string())
    } else {
        (b.to_string(), a.to_string())
    }
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|k| a[k] - b[k])
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn norm(a: [f64; 3]) -> f64 {
    a[0].hypot(a[1]).hypot(a[2])
}

/// Closest points of the segments `a + s u` and `c + t v` (s, t in 0..1, neither
/// segment degenerate and not parallel): the parameters and the distance.
fn segment_closest(a: [f64; 3], u: [f64; 3], c: [f64; 3], v: [f64; 3]) -> (f64, f64, f64) {
    let r = sub(a, c);
    let (aa, e, f) = (dot(u, u), dot(v, v), dot(v, r));
    let (cc, b) = (dot(u, r), dot(u, v));
    let denominator = aa * e - b * b;
    let mut s = if denominator > 0.0 {
        ((b * f - cc * e) / denominator).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let mut t = (b * s + f) / e;
    if t < 0.0 {
        t = 0.0;
        s = (-cc / aa).clamp(0.0, 1.0);
    } else if t > 1.0 {
        t = 1.0;
        s = ((b - cc) / aa).clamp(0.0, 1.0);
    }
    let p: [f64; 3] = std::array::from_fn(|k| a[k] + u[k] * s);
    let q: [f64; 3] = std::array::from_fn(|k| c[k] + v[k] * t);
    (s, t, norm(sub(p, q)))
}

/// Replace `f` by the pieces between consecutive `joints`, which lie at the
/// relative distances `bounds` (first 0, last 1). The first piece keeps the
/// name. `uniform` bounds are the equal division, rescaled by a product with
/// the number of pieces as that command always did.
fn make_children(
    staged: &mut SapModel,
    f: &FrameObj,
    joints: &[String],
    bounds: &[f64],
    uniform: bool,
    summary: &mut EditSummary,
) -> Result<(), EditError> {
    let segments = joints.len() - 1;
    // Keep the original name for the first child; reserve all original names.
    for i in 0..segments {
        let lo = bounds[i];
        let hi = bounds[i + 1];
        let rescale = |x: f64| {
            if uniform {
                (x - lo) * segments as f64
            } else {
                ((x - lo) / (hi - lo)).clamp(0.0, 1.0)
            }
        };
        let mut child = f.clone();
        child.name = if i == 0 {
            f.name.clone()
        } else {
            next_numeric_name(staged.frames.keys())
        };
        child.point_i = joints[i].clone();
        child.point_j = joints[i + 1].clone();
        if i != 0 {
            child.release_i = [false; 6];
            child.start_spring = [0.0; 6];
        }
        if i != segments - 1 {
            child.release_j = [false; 6];
            child.end_spring = [0.0; 6];
        }
        child.loads_dist.clear();
        for l in &f.loads_dist {
            let start = lo.max(l.rel_dist1);
            let end = hi.min(l.rel_dist2);
            if end > start {
                let value = |s| {
                    l.val1 + (l.val2 - l.val1) * ((s - l.rel_dist1) / (l.rel_dist2 - l.rel_dist1))
                };
                let mut load = l.clone();
                load.rel_dist1 = rescale(start);
                load.rel_dist2 = rescale(end);
                load.val1 = value(start);
                load.val2 = value(end);
                finite([load.val1, load.val2])?;
                child.loads_dist.push(load);
            }
        }
        child.loads_point = f
            .loads_point
            .iter()
            .filter(|l| l.rel_dist >= lo && (l.rel_dist < hi || i == segments - 1))
            .map(|l| {
                let mut load = l.clone();
                load.rel_dist = rescale(l.rel_dist);
                load
            })
            .collect();
        if i != 0 {
            summary.frames_created.push(child.name.clone());
        }
        staged.frames.insert(child.name.clone(), child);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        AreaLoadSurfacePressure, CaseResults, FrameLoadDistributed, FrameLoadPoint, PointLoadForce,
    };

    fn frame_model() -> SapModel {
        let mut m = SapModel::default();
        m.add_point("1", 0.0, 0.0, 0.0);
        m.add_point("2", 12.0, 0.0, 0.0);
        m.add_frame("beam", "1", "2", "FSEC1");
        m.frames.get_mut("beam").unwrap().selected = true;
        m.case_results.insert("OLD".into(), CaseResults::default());
        m
    }

    #[test]
    fn move_shared_joints_once_preserves_connectivity_assignments_and_unrelated_points() {
        let mut m = frame_model();
        m.add_point("3", 24.0, 0.0, 0.0);
        m.add_frame("other", "2", "3", "FSEC1");
        m.points.get_mut("2").unwrap().selected = true;
        m.points.get_mut("1").unwrap().restraint = [true; 6];
        let other = m.frames["other"].clone();
        let p3 = m.points["3"].clone();
        let summary = m.move_selected([0.0, 5.0, -1.0]).unwrap();
        assert_eq!(summary.points_moved, ["1", "2"]);
        assert_eq!(coords(&m.points["2"]), [12.0, 5.0, -1.0]);
        assert_eq!(m.points["1"].restraint, [true; 6]);
        assert_eq!(m.frames["other"], other);
        assert_eq!(m.points["3"], p3);
        assert!(m.case_results.is_empty());
    }

    #[test]
    fn failed_and_locked_moves_leave_results_and_geometry_unchanged() {
        let mut m = frame_model();
        m.add_point("3", 0.0, 5.0, 0.0);
        let before = m.clone();
        assert!(matches!(
            m.move_selected([0.0, 5.0, 0.0]),
            Err(EditError::JointConflict(_))
        ));
        assert_eq!(m, before);
        assert!(m.move_selected([f64::INFINITY, 0.0, 0.0]).is_err());
        assert_eq!(m, before);
        m.is_locked = true;
        let before = m.clone();
        assert_eq!(m.move_selected([0.0, 1.0, 0.0]), Err(EditError::Locked));
        assert_eq!(m, before);
    }

    #[test]
    fn replicate_frame_area_and_joint_assignments_with_one_shared_mapping_per_copy() {
        let mut m = frame_model();
        m.add_point("3", 12.0, 0.0, 12.0);
        m.add_point("4", 0.0, 0.0, 12.0);
        m.add_area(
            "panel",
            vec!["1".into(), "2".into(), "3".into(), "4".into()],
            "ASEC1",
        );
        let panel = m.areas.get_mut("panel").unwrap();
        panel.selected = true;
        panel.modifiers[1] = 0.0;
        panel.auto_mesh = (3, 2);
        panel.loads_pressure.push(AreaLoadSurfacePressure {
            load_pat: "DEAD".into(),
            face: -2,
            val: 2.0,
            pattern_name: None,
        });
        let p = m.points.get_mut("1").unwrap();
        p.restraint = [true; 6];
        p.mass[0] = 4.0;
        p.loads_force.push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: [2.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        });
        m.frames.get_mut("beam").unwrap().release_j[5] = true;
        let old_points = m.points.clone();
        let old_frame = m.frames["beam"].clone();
        let summary = m.replicate_selected([0.0, 24.0, 0.0], 2).unwrap();
        assert_eq!(summary.points_created.len(), 8);
        assert_eq!(summary.frames_created.len(), 2);
        assert_eq!(summary.areas_created.len(), 2);
        for (id, original) in old_points {
            assert_eq!(m.points[&id], original);
        }
        assert_eq!(m.frames["beam"], old_frame);
        for (i, fid) in summary.frames_created.iter().enumerate() {
            let f = &m.frames[fid];
            let a = &m.areas[&summary.areas_created[i]];
            assert_eq!([&f.point_i, &f.point_j], [&a.points[0], &a.points[1]]);
            assert_eq!(m.points[&f.point_i].restraint, [true; 6]);
            assert_eq!(m.points[&f.point_i].mass[0], 4.0);
            assert_eq!(m.points[&f.point_i].loads_force.len(), 1);
            assert!(f.release_j[5]);
            assert_eq!(a.modifiers[1], 0.0);
            assert_eq!(a.auto_mesh, (3, 2));
            assert_eq!(a.loads_pressure[0].val, 2.0);
        }
    }

    #[test]
    fn replicas_join_existing_equal_joints_and_reject_conflicts_atomically() {
        let mut m = frame_model();
        let summary = m.replicate_selected([12.0, 0.0, 0.0], 2).unwrap();
        assert_eq!(summary.points_created.len(), 2);
        assert_eq!(m.frames[&summary.frames_created[0]].point_i, "2");
        assert_eq!(
            m.frames[&summary.frames_created[0]].point_j,
            m.frames[&summary.frames_created[1]].point_i
        );
        let mut m = frame_model();
        m.points.get_mut("1").unwrap().restraint[0] = true;
        let before = m.clone();
        assert!(matches!(
            m.replicate_selected([12.0, 0.0, 0.0], 2),
            Err(EditError::JointConflict(_))
        ));
        assert_eq!(m, before);
    }

    #[test]
    fn partial_trapezoids_conserve_force_moment_and_boundary_point_loads_once() {
        let mut m = frame_model();
        let f = m.frames.get_mut("beam").unwrap();
        f.loads_dist.push(FrameLoadDistributed {
            load_pat: "DEAD".into(),
            my_type: 1,
            dir: 6,
            csys: "GLOBAL".into(),
            rel_dist1: 0.125,
            rel_dist2: 0.875,
            val1: 2.0,
            val2: 8.0,
        });
        f.loads_point = [0.0, 0.25, 0.5, 1.0]
            .map(|s| FrameLoadPoint {
                load_pat: "DEAD".into(),
                my_type: 1,
                dir: 6,
                csys: "GLOBAL".into(),
                rel_dist: s,
                val: 5.0,
            })
            .to_vec();
        f.release_i[5] = true;
        f.start_spring[5] = 7.0;
        f.release_j[4] = true;
        f.end_spring[4] = 8.0;
        let summary = m.divide_selected_frames(4).unwrap();
        assert_eq!(summary.points_created.len(), 3);
        assert_eq!(summary.frames_created.len(), 3);
        assert_eq!(m.frames.len(), 4);
        let mut total = 0.0;
        let mut moment = 0.0;
        let mut points = 0;
        for f in m.frames.values() {
            let start = m.points[&f.point_i].x;
            let length = m.points[&f.point_j].x - start;
            for l in &f.loads_dist {
                let x = start + length * l.rel_dist1;
                let span = length * (l.rel_dist2 - l.rel_dist1);
                let force = span * (l.val1 + l.val2) / 2.0;
                total += force;
                moment += x * force + span * span * (l.val1 + 2.0 * l.val2) / 6.0;
            }
            points += f.loads_point.len();
            if f.point_i == "1" {
                assert!(f.release_i[5]);
                assert_eq!(f.start_spring[5], 7.0);
            } else {
                assert_eq!(f.release_i, [false; 6]);
                assert_eq!(f.start_spring, [0.0; 6]);
            }
            if f.point_j == "2" {
                assert!(f.release_j[4]);
                assert_eq!(f.end_spring[4], 8.0);
            } else {
                assert_eq!(f.release_j, [false; 6]);
                assert_eq!(f.end_spring, [0.0; 6]);
            }
        }
        assert!((total - 45.0).abs() < 1e-12);
        assert!((moment - 310.5).abs() < 1e-12);
        assert_eq!(points, 4);
    }

    #[test]
    fn dividing_multiple_frames_is_atomic_when_later_assignment_is_unsupported() {
        let mut m = frame_model();
        m.add_point("3", 24.0, 0.0, 0.0);
        m.add_frame("second", "2", "3", "FSEC1");
        let f = m.frames.get_mut("second").unwrap();
        f.selected = true;
        f.end_offset_i = 1.0;
        let before = m.clone();
        assert!(matches!(
            m.divide_selected_frames(3),
            Err(EditError::Unsupported(_))
        ));
        assert_eq!(m, before);
    }

    #[test]
    fn dividing_a_frame_with_hinges_or_a_hinge_overwrite_is_refused() {
        use crate::hinge::{FrameHinge, FrameHingeOverwrite};
        // The unchanged model divides, so each refusal is caused by the one
        // assignment it adds.
        frame_model().divide_selected_frames(2).unwrap();
        let mut hinged = frame_model();
        let frame = hinged.frames.values_mut().find(|f| f.selected).unwrap();
        let name = frame.name.clone();
        frame.hinges.push(FrameHinge {
            prop: "H".into(),
            rel_dist: 0.0,
        });
        let mut overwritten = frame_model();
        overwritten
            .frames
            .values_mut()
            .find(|f| f.selected)
            .unwrap()
            .hinge_overwrite = Some(FrameHingeOverwrite::default());
        for mut m in [hinged, overwritten] {
            let before = m.clone();
            match m.divide_selected_frames(2) {
                Err(EditError::Unsupported(message)) => {
                    assert!(message.contains(&name) && message.contains("hinge"), "{message}")
                }
                other => panic!("{other:?}"),
            }
            assert_eq!(m, before);
        }
    }

    #[test]
    fn division_reuses_existing_intermediate_joint_without_cloning_endpoint_assignments() {
        let mut m = frame_model();
        m.add_point("3", 6.0, 0.0, 0.0);
        m.points.get_mut("3").unwrap().restraint[2] = true;
        m.points.get_mut("1").unwrap().restraint = [true; 6];
        let summary = m.divide_selected_frames(2).unwrap();
        assert!(summary.points_created.is_empty());
        assert_eq!(m.frames["beam"].point_j, "3");
        assert_eq!(
            m.points["3"].restraint,
            [false, false, true, false, false, false]
        );
    }
}

#[cfg(test)]
mod numerical_edge_tests {
    use super::*;
    use crate::CaseResults;

    #[test]
    fn replication_overflow_after_a_valid_copy_keeps_original_state_and_outputs() {
        let mut m = SapModel::default();
        m.add_point("1", 0.0, 0.0, 0.0);
        m.add_point("2", 12.0, 0.0, 0.0);
        m.add_frame("beam", "1", "2", "FSEC1");
        m.frames.get_mut("beam").unwrap().selected = true;
        m.case_results.insert("OLD".into(), CaseResults::default());
        let before = m.clone();
        assert!(m
            .replicate_selected([0.0, 0.0, f64::MAX * 0.75], 2)
            .is_err());
        assert_eq!(m, before);
    }
}
