//! Automatic frame subdivision for analysis.
//!
//! A frame object assigned a number of segments, or subdivision at the joints
//! that lie on it, is analyzed as several frame elements in a row. The working
//! copy of the model is expanded before assembly and every result container is
//! folded back afterwards, so results are reported for the object and only for
//! joints the model defines.

use std::collections::HashSet;

use indexmap::IndexMap;
use sap_model::{
    CaseResults, FrameObj, FramePropType, FrameSection, FrameStationResult, NonPrismaticSegment,
    PointObj, SapModel,
};

use crate::frame::resolve_non_prismatic_segments;
use crate::SolverError;

/// Largest accepted number of automatic segments on one frame object.
pub(crate) const MAX_SEGMENTS: usize = 10_000;

/// One element of a subdivided frame object.
struct Piece {
    frame: String,
    /// Distance from the object's I joint to the element's I joint.
    start: f64,
    /// The element starts at a joint created for the subdivision, which
    /// carries no load or support, so forces are continuous across it.
    starts_at_created_joint: bool,
}

/// What [`expand`] did, for [`collapse`].
#[derive(Default)]
pub(crate) struct FrameMesh {
    /// Frame object names in model order.
    order: Vec<String>,
    /// Subdivided object -> its elements in order, and its length.
    parents: IndexMap<String, (Vec<Piece>, f64)>,
    /// Joints created for the subdivision.
    internal_joints: HashSet<String>,
}

fn unsupported(frame: &str, what: &str) -> SolverError {
    SolverError::UnsupportedInput(format!(
        "frame '{frame}' is subdivided automatically for analysis and {what}, a combination that is not implemented; remove the automatic subdivision or split the frame into separate objects"
    ))
}

/// Relative positions in `(0, 1)` of the joints lying on the frame.
fn joints_on_span<'a>(
    frame: &FrameObj,
    start: &PointObj,
    axis: [f64; 3],
    length: f64,
    candidates: impl Iterator<Item = (&'a str, [f64; 3])>,
) -> Vec<(f64, String)> {
    // A joint is on the frame when it is within one millionth of the frame
    // length of its axis, and that far inside both ends.
    let tolerance = 1e-6 * length;
    let mut found: Vec<(f64, String)> = candidates
        .filter(|(name, _)| *name != frame.point_i && *name != frame.point_j)
        .filter_map(|(name, at)| {
            let rel = [at[0] - start.x, at[1] - start.y, at[2] - start.z];
            let along = (0..3).map(|c| rel[c] * axis[c]).sum::<f64>() / length;
            let off2: f64 = (0..3)
                .map(|c| (rel[c] - along * axis[c] / length).powi(2))
                .sum();
            (along > tolerance && along < length - tolerance && off2 <= tolerance * tolerance)
                .then(|| (along / length, name.to_string()))
        })
        .collect();
    found.sort_by(|a, b| a.0.total_cmp(&b.0));
    found
}

/// Replaces every automatically subdivided frame object in `model` by its
/// elements. `mesh_points` are the joints area auto-meshing will create, which
/// a frame subdivided at joints would have to connect to.
pub(crate) fn expand(
    model: &mut SapModel,
    mesh_points: &[(String, [f64; 3])],
) -> Result<FrameMesh, SolverError> {
    let mut mesh = FrameMesh {
        order: model.frames.keys().cloned().collect(),
        ..FrameMesh::default()
    };
    if model
        .frames
        .values()
        .all(|f| f.auto_mesh_segments <= 1 && !f.auto_mesh_at_joints)
    {
        return Ok(mesh);
    }

    let mut frames = IndexMap::with_capacity(model.frames.len());
    // Created joints with the frame they belong to and its tolerance.
    let mut created: Vec<(String, f64)> = Vec::new();
    let mut new_points: Vec<PointObj> = Vec::new();
    let mut new_sections: Vec<FrameSection> = Vec::new();
    for (name, frame) in &model.frames {
        let segments = frame.auto_mesh_segments.max(1);
        let (Some(pi), Some(pj)) = (
            model.points.get(&frame.point_i),
            model.points.get(&frame.point_j),
        ) else {
            frames.insert(name.clone(), frame.clone());
            continue;
        };
        let axis = [pj.x - pi.x, pj.y - pi.y, pj.z - pi.z];
        let length = axis.iter().map(|c| c * c).sum::<f64>().sqrt();
        if (segments == 1 && !frame.auto_mesh_at_joints) || length <= 0.0 {
            frames.insert(name.clone(), frame.clone());
            continue;
        }

        // Joints of the elements: (relative position, joint name).
        let mut joints: Vec<(f64, String)> = vec![(0.0, frame.point_i.clone())];
        if frame.auto_mesh_at_joints {
            if !joints_on_span(
                frame,
                pi,
                axis,
                length,
                mesh_points.iter().map(|(n, at)| (n.as_str(), *at)),
            )
            .is_empty()
            {
                return Err(unsupported(
                    name,
                    "it passes through joints created by automatic area meshing",
                ));
            }
            let on_span = joints_on_span(
                frame,
                pi,
                axis,
                length,
                model.points.values().map(|p| (p.name.as_str(), [p.x, p.y, p.z])),
            );
            if !on_span.is_empty() && segments > 1 {
                return Err(unsupported(
                    name,
                    "has both a number of segments and joints lying on it",
                ));
            }
            joints.extend(on_span);
        }
        if segments > 1 {
            for k in 1..segments {
                let t = k as f64 / segments as f64;
                let joint = format!("__fmesh_{name}_{k}");
                if model.points.contains_key(&joint) {
                    return Err(SolverError::InvalidInput(format!(
                        "joint name '{joint}' is reserved for the automatic subdivision of frame '{name}'"
                    )));
                }
                let mut point = PointObj::new(
                    joint.clone(),
                    pi.x + t * axis[0],
                    pi.y + t * axis[1],
                    pi.z + t * axis[2],
                );
                // A temperature load scaled by a joint pattern varies linearly
                // between the object's ends; give the new joints that value.
                for load in &frame.loads_temp {
                    if let Some(pattern) = &load.pattern_name {
                        let vi = pi.pattern_values.get(pattern).copied().unwrap_or(1.0);
                        let vj = pj.pattern_values.get(pattern).copied().unwrap_or(1.0);
                        point.pattern_values.insert(pattern.clone(), vi + (vj - vi) * t);
                    }
                }
                new_points.push(point);
                created.push((name.clone(), 1e-6 * length));
                mesh.internal_joints.insert(joint.clone());
                joints.push((t, joint));
            }
        }
        joints.push((1.0, frame.point_j.clone()));
        if joints.len() == 2 {
            frames.insert(name.clone(), frame.clone());
            continue;
        }

        if frame.cable_params.is_some() {
            return Err(unsupported(name, "is a cable"));
        }
        if frame.tc_limits.is_some_and(|(c, _, t, _)| c || t) {
            return Err(unsupported(name, "has a tension or compression limit"));
        }
        if frame.end_offset_i != 0.0 || frame.end_offset_j != 0.0 {
            return Err(unsupported(name, "has end length offsets"));
        }
        let section = model.frame_sections.get(&frame.section);
        let non_prismatic = section
            .filter(|s| s.prop_type == FramePropType::Variable && !s.non_prismatic.is_empty());

        let pieces_n = joints.len() - 1;
        let stations = (frame.min_stations.max(2) - 1).div_ceil(pieces_n) + 1;
        let mut pieces = Vec::with_capacity(pieces_n);
        for (k, pair) in joints.windows(2).enumerate() {
            let ((ta, ja), (tb, jb)) = (&pair[0], &pair[1]);
            let piece_name = format!("{name}__seg_{k}");
            if model.frames.contains_key(&piece_name) {
                return Err(SolverError::InvalidInput(format!(
                    "frame name '{piece_name}' is reserved for the automatic subdivision of frame '{name}'"
                )));
            }
            let mut piece = frame.clone();
            piece.name = piece_name.clone();
            piece.point_i = ja.clone();
            piece.point_j = jb.clone();
            piece.auto_mesh_segments = 0;
            piece.auto_mesh_at_joints = false;
            piece.min_stations = stations;
            // End releases and partial fixity belong to the object's ends.
            if k > 0 {
                piece.release_i = [false; 6];
                piece.start_spring = [0.0; 6];
            }
            if k + 1 < pieces_n {
                piece.release_j = [false; 6];
                piece.end_spring = [0.0; 6];
            }
            let span = tb - ta;
            piece.loads_dist = frame
                .loads_dist
                .iter()
                .filter_map(|load| {
                    let (s1, s2) = (load.rel_dist1.clamp(0.0, 1.0), load.rel_dist2.clamp(0.0, 1.0));
                    let (lo, hi) = (s1.max(*ta), s2.min(*tb));
                    if s2 <= s1 + 1e-12 || hi <= lo {
                        return None;
                    }
                    let value = |s: f64| load.val1 + (load.val2 - load.val1) * (s - s1) / (s2 - s1);
                    let mut part = load.clone();
                    part.rel_dist1 = ((lo - ta) / span).clamp(0.0, 1.0);
                    part.rel_dist2 = ((hi - ta) / span).clamp(0.0, 1.0);
                    part.val1 = value(lo);
                    part.val2 = value(hi);
                    (part.rel_dist2 > part.rel_dist1 + 1e-12).then_some(part)
                })
                .collect();
            // A point load on an element boundary goes to the element it ends.
            piece.loads_point = frame
                .loads_point
                .iter()
                .filter_map(|load| {
                    let s = load.rel_dist.clamp(0.0, 1.0);
                    let after_start = if k == 0 { s >= *ta } else { s > *ta };
                    (after_start && s <= *tb).then(|| {
                        let mut part = load.clone();
                        part.rel_dist = ((s - ta) / span).clamp(0.0, 1.0);
                        part
                    })
                })
                .collect();
            if let Some(section) = non_prismatic {
                let (clipped, ends) =
                    clip_non_prismatic(model, section, &piece_name, length, ta * length, tb * length);
                piece.section = clipped.name.clone();
                new_sections.extend(ends);
                new_sections.push(clipped);
            }
            frames.insert(piece_name.clone(), piece);
            pieces.push(Piece {
                frame: piece_name,
                start: ta * length,
                starts_at_created_joint: mesh.internal_joints.contains(ja),
            });
        }
        mesh.parents.insert(name.clone(), (pieces, length));
    }

    refuse_coincident_joints(model, mesh_points, &new_points, &created)?;

    model.frames = frames;
    for point in new_points {
        model.points.insert(point.name.clone(), point);
    }
    for section in new_sections {
        model.frame_sections.insert(section.name.clone(), section);
    }
    Ok(mesh)
}

/// A joint created for a subdivision is connected to its own frame only. If
/// it lands on a joint of the model, on a joint area meshing will create, or
/// on a joint created for another frame, the two would stay unconnected, and
/// whether the source program merges them is not established. Refuse.
fn refuse_coincident_joints(
    model: &SapModel,
    mesh_points: &[(String, [f64; 3])],
    new_points: &[PointObj],
    created: &[(String, f64)],
) -> Result<(), SolverError> {
    if new_points.is_empty() {
        return Ok(());
    }
    // (x, y, z, description, owning frame and tolerance if created)
    struct Site<'a> {
        at: [f64; 3],
        what: String,
        owner: Option<(&'a str, f64)>,
    }
    let mut sites: Vec<Site> = Vec::with_capacity(new_points.len() + model.points.len());
    for (point, (frame, tolerance)) in new_points.iter().zip(created) {
        sites.push(Site {
            at: [point.x, point.y, point.z],
            what: format!("a joint created for the subdivision of frame '{frame}'"),
            owner: Some((frame.as_str(), *tolerance)),
        });
    }
    for point in model.points.values() {
        sites.push(Site {
            at: [point.x, point.y, point.z],
            what: format!("joint '{}'", point.name),
            owner: None,
        });
    }
    for (_, at) in mesh_points {
        sites.push(Site {
            at: *at,
            what: "a joint created by automatic area meshing".to_string(),
            owner: None,
        });
    }
    sites.sort_by(|a, b| a.at[0].total_cmp(&b.at[0]));
    let largest = created.iter().fold(0.0f64, |m, (_, t)| m.max(*t));
    for (index, site) in sites.iter().enumerate() {
        for other in &sites[index + 1..] {
            if other.at[0] - site.at[0] > largest {
                break;
            }
            let (created_site, against) = match (site.owner, other.owner) {
                (Some(owner), Some(second)) if owner.0 == second.0 => continue,
                (Some(owner), _) => (owner, other),
                (None, Some(owner)) => (owner, site),
                (None, None) => continue,
            };
            let tolerance = created_site.1.max(other.owner.or(site.owner).map_or(0.0, |o| o.1));
            let distance2: f64 = (0..3).map(|c| (site.at[c] - other.at[c]).powi(2)).sum();
            if distance2 <= tolerance * tolerance {
                return Err(unsupported(
                    created_site.0,
                    &format!("one of its new joints falls on {}", against.what),
                ));
            }
        }
    }
    Ok(())
}

/// The part `[from, to]` (distances along the object) of a non-prismatic
/// section as a section of its own. Area, shear areas and torsion constant
/// vary linearly and each moment of inertia as a power of a linear function,
/// so the part of a segment is a segment of the same kind between the
/// properties at its cut points.
fn clip_non_prismatic(
    model: &SapModel,
    section: &FrameSection,
    piece: &str,
    length: f64,
    from: f64,
    to: f64,
) -> (FrameSection, Vec<FrameSection>) {
    let mut clipped = section.clone();
    clipped.name = format!("{}__for_{piece}", section.name);
    clipped.non_prismatic.clear();
    let mut ends = Vec::new();
    let mut position = 0.0f64;
    for (index, (segment, segment_length)) in
        resolve_non_prismatic_segments(&section.non_prismatic, length)
            .into_iter()
            .enumerate()
    {
        let (lo, hi) = (position.max(from), (position + segment_length).min(to));
        let segment_start = position;
        position += segment_length;
        if hi <= lo || segment_length <= 0.0 {
            continue;
        }
        let start = model.frame_sections.get(&segment.start_sec).unwrap_or(section);
        let end = model.frame_sections.get(&segment.end_sec).unwrap_or(section);
        let mut names = [String::new(), String::new()];
        for (side, at) in [lo, hi].into_iter().enumerate() {
            let xi = ((at - segment_start) / segment_length).clamp(0.0, 1.0);
            let linear = |a: f64, b: f64| a * (1.0 - xi) + b * xi;
            let power = |a: f64, b: f64, variation: i32| {
                let n = f64::from(variation.max(1));
                linear(a.max(0.0).powf(1.0 / n), b.max(0.0).powf(1.0 / n)).powf(n)
            };
            let mut cut = start.clone();
            cut.name = format!("{}__cut_{index}_{side}", clipped.name);
            cut.prop_type = FramePropType::General;
            cut.non_prismatic.clear();
            cut.area = linear(start.area, end.area);
            cut.as2 = linear(start.as2, end.as2);
            cut.as3 = linear(start.as3, end.as3);
            cut.torsional_j = linear(start.torsional_j, end.torsional_j);
            cut.i33 = power(start.i33, end.i33, segment.ei33_variation);
            cut.i22 = power(start.i22, end.i22, segment.ei22_variation);
            names[side] = cut.name.clone();
            ends.push(cut);
        }
        let [start_sec, end_sec] = names;
        clipped.non_prismatic.push(NonPrismaticSegment {
            start_sec,
            end_sec,
            length: hi - lo,
            is_variable_length: false,
            ei33_variation: segment.ei33_variation,
            ei22_variation: segment.ei22_variation,
        });
    }
    (clipped, ends)
}

/// Folds the results of the elements back onto their frame objects and drops
/// the joints created for the subdivision from every result container.
pub(crate) fn collapse(results: &mut IndexMap<String, CaseResults>, mesh: &FrameMesh) {
    if mesh.parents.is_empty() {
        return;
    }
    let internal = &mesh.internal_joints;
    for case in results.values_mut() {
        if !case.frame_forces.is_empty() {
            let mut by_element = std::mem::take(&mut case.frame_forces);
            for name in &mesh.order {
                let stations = match mesh.parents.get(name) {
                    None => by_element.shift_remove(name),
                    Some((pieces, length)) => merge_stations(pieces, *length, &mut by_element),
                };
                if let Some(stations) = stations {
                    case.frame_forces.insert(name.clone(), stations);
                }
            }
        }
        case.joint_displacements.retain(|joint, _| !internal.contains(joint));
        case.joint_reactions.retain(|joint, _| !internal.contains(joint));
        case.time_history_disp.retain(|joint, _| !internal.contains(joint));
        for mode in &mut case.modes {
            mode.mode_shape.retain(|joint, _| !internal.contains(joint));
        }
        if let Some(state) = &mut case.transient_state {
            let keep: Vec<bool> = state.point_order.iter().map(|p| !internal.contains(p)).collect();
            let points = keep.len();
            for vector in [
                &mut state.u,
                &mut state.v,
                &mut state.a,
                &mut state.previous_force,
            ] {
                if points > 0 && vector.len() % points == 0 {
                    let per_point = vector.len() / points;
                    let mut index = 0;
                    vector.retain(|_| {
                        let kept = keep[index / per_point];
                        index += 1;
                        kept
                    });
                }
            }
            state.point_order.retain(|p| !internal.contains(p));
        }
    }
}

/// Stations of the elements in order, measured along the object. The station
/// two elements share is reported once at a joint created for the
/// subdivision and twice (before and after) at a joint of the model, where a
/// load or support can make the forces jump. The rule does not look at the
/// forces, so a frame has the same stations in every case and combination.
fn merge_stations(
    pieces: &[Piece],
    length: f64,
    by_element: &mut IndexMap<String, Vec<FrameStationResult>>,
) -> Option<Vec<FrameStationResult>> {
    let mut merged: Vec<FrameStationResult> = Vec::new();
    let mut found_all = true;
    for piece in pieces {
        let Some(stations) = by_element.shift_remove(&piece.frame) else {
            found_all = false;
            continue;
        };
        for (index, mut station) in stations.into_iter().enumerate() {
            station.station += piece.start;
            station.rel_dist = (station.station / length).clamp(0.0, 1.0);
            if index == 0 && piece.starts_at_created_joint && !merged.is_empty() {
                continue;
            }
            merged.push(station);
        }
    }
    (found_all && !merged.is_empty()).then_some(merged)
}
