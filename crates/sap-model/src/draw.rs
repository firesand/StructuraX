//! Drawing objects at typed or picked coordinates.
//!
//! The commands follow the rules of [`crate::generate`]: they work on a staged
//! copy, refuse a locked model, reuse a joint that already exists at a
//! requested coordinate and leave the model unchanged when refused. Accepted
//! commands clear stale analysis and design results.
//!
//! The area criterion ([`check_area_corners`]) is the one the solver applies
//! to every area before it analyzes (`validate_area_geometry` in
//! `sap-solver`): three or four corners, and the signed cross products of
//! consecutive edges, taken in the plane of the element, nonzero and of one
//! sign. `sap-model` cannot depend on the solver, so the test is repeated here
//! and `crates/sap-solver/tests/draw_area_criterion.rs` holds the two to the
//! same outcome.

use crate::editing::{finite, invalid};
use crate::generate::Stage;
use crate::{EditError, EditSummary, SapModel};

impl SapModel {
    /// Draw one frame per segment, from the first point (end I) to the second.
    /// `pinned` releases M2 and M3 at both ends. A frame between two joints
    /// that already have one is skipped; the command is refused when it
    /// creates nothing, when a segment has zero length (after reusing
    /// existing joints), for a missing section and for a locked model.
    pub fn draw_frames(
        &mut self,
        segments: &[([f64; 3], [f64; 3])],
        section: &str,
        pinned: bool,
    ) -> Result<EditSummary, EditError> {
        self.check_edit()?;
        if segments.is_empty() {
            return Err(invalid("no frame segments to draw"));
        }
        self.require_frame_section(section)?;
        let mut extra = Vec::with_capacity(2 * segments.len());
        for (k, (a, b)) in segments.iter().enumerate() {
            finite(a.iter().chain(b.iter()).copied())
                .map_err(|_| invalid(format!("frame segment {} has a nonfinite coordinate", k + 1)))?;
            extra.push(*a);
            extra.push(*b);
        }
        let mut stage = Stage::new(self, &extra);
        for (k, (a, b)) in segments.iter().enumerate() {
            let i = stage.add_joint(*a)?;
            let j = stage.add_joint(*b)?;
            if i == j {
                return Err(invalid(format!(
                    "frame segment {} has zero length (both ends are joint {i})",
                    k + 1
                )));
            }
            stage.add_frame(&i, &j, section, pinned)?;
        }
        self.finish_draw(stage, "draw frames")
    }

    /// Draw one area per corner set, with the joints in the given order (local
    /// axis 3 follows the right-hand rule over that order). Each set has three
    /// or four corners and must pass [`check_area_corners`] at the stored
    /// joint coordinates. Refused: a repeated joint, an area that exists
    /// already (also within this command), a missing area section, a locked
    /// model.
    pub fn draw_areas(
        &mut self,
        corner_sets: &[Vec<[f64; 3]>],
        section: &str,
    ) -> Result<EditSummary, EditError> {
        self.check_edit()?;
        if corner_sets.is_empty() {
            return Err(invalid("no areas to draw"));
        }
        if !self.area_sections.contains_key(section) {
            return Err(invalid(format!("missing area section {section}")));
        }
        let mut extra = Vec::new();
        for (k, set) in corner_sets.iter().enumerate() {
            if !(3..=4).contains(&set.len()) {
                return Err(invalid(format!(
                    "area {} has {} corners; the solver accepts 3 or 4",
                    k + 1,
                    set.len()
                )));
            }
            for c in set {
                finite(*c).map_err(|_| {
                    invalid(format!("area {} has a nonfinite coordinate", k + 1))
                })?;
                extra.push(*c);
            }
        }
        let mut stage = Stage::new(self, &extra);
        for (k, set) in corner_sets.iter().enumerate() {
            let mut ids = Vec::with_capacity(set.len());
            for c in set {
                ids.push(stage.add_joint(*c)?);
            }
            for (a, id) in ids.iter().enumerate() {
                if ids[..a].contains(id) {
                    return Err(invalid(format!(
                        "area {} repeats joint {id}: two corners coincide",
                        k + 1
                    )));
                }
            }
            let stored: Vec<[f64; 3]> = ids
                .iter()
                .map(|id| {
                    let p = &stage.model.points[id];
                    [p.x, p.y, p.z]
                })
                .collect();
            check_area_corners(&stored)
                .map_err(|e| invalid(format!("area {}: {e}", k + 1)))?;
            if stage.add_area(ids.clone(), section)?.is_none() {
                return Err(invalid(format!(
                    "area {} duplicates an existing area on joints {}",
                    k + 1,
                    ids.join(", ")
                )));
            }
        }
        self.finish_draw(stage, "draw areas")
    }
}

/// Whether `corners` (in order) form an area the analysis accepts: three or
/// four distinct points, planar enough to project, and convex without
/// folding. This is the solver's criterion (`validate_area_geometry`): the
/// corners are taken relative to the first and divided by their largest
/// absolute coordinate, projected on the plane of the element, and the cross
/// products of consecutive edges must all exceed `1e-12` in magnitude and
/// share one sign.
pub fn check_area_corners(corners: &[[f64; 3]]) -> Result<(), EditError> {
    let refuse = |why: &str| {
        Err(invalid(format!(
            "area corners are degenerate, nonfinite or folded ({why}); use distinct corners forming a valid triangle or convex quadrilateral"
        )))
    };
    if !(3..=4).contains(&corners.len()) {
        return Err(invalid(format!(
            "an area has 3 or 4 corners, not {}",
            corners.len()
        )));
    }
    let origin = corners[0];
    let mut coords: Vec<[f64; 3]> = corners
        .iter()
        .map(|c| [c[0] - origin[0], c[1] - origin[1], c[2] - origin[2]])
        .collect();
    if coords.iter().flatten().any(|v| !v.is_finite()) {
        return refuse("nonfinite coordinate");
    }
    let scale = coords.iter().flatten().fold(0.0_f64, |s, v| s.max(v.abs()));
    if scale == 0.0 {
        return refuse("all corners coincide");
    }
    for c in &mut coords {
        for v in c {
            *v /= scale;
        }
    }
    let xy = local_xy(&coords);
    let n = xy.len();
    let mut orientation = 0.0_f64;
    for i in 0..n {
        let a = xy[i];
        let b = xy[(i + 1) % n];
        let c = xy[(i + 2) % n];
        let cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]);
        if !cross.is_finite() || cross.abs() <= 1e-12 {
            return refuse("zero-area corner");
        }
        if orientation == 0.0 {
            orientation = cross.signum();
        } else if cross.signum() != orientation {
            return refuse("corners turn in both directions");
        }
    }
    Ok(())
}

/// Local 2D coordinates of the corners about their centroid, as the solver's
/// `area_local_system` computes them for a zero local-axis angle.
fn local_xy(coords: &[[f64; 3]]) -> Vec<[f64; 2]> {
    let n = coords.len();
    let mut centroid = [0.0; 3];
    for c in coords {
        for k in 0..3 {
            centroid[k] += c[k];
        }
    }
    for v in &mut centroid {
        *v /= n as f64;
    }
    let (v_xi, v_eta) = if n >= 4 {
        (
            std::array::from_fn(|k| (coords[1][k] + coords[2][k]) - (coords[0][k] + coords[3][k])),
            std::array::from_fn(|k| (coords[2][k] + coords[3][k]) - (coords[0][k] + coords[1][k])),
        )
    } else {
        (
            std::array::from_fn(|k| coords[1][k] - coords[0][k]),
            std::array::from_fn(|k| coords[2][k] - coords[0][k]),
        )
    };
    let v_xi: [f64; 3] = v_xi;
    let v_eta: [f64; 3] = v_eta;
    let cross = [
        v_xi[1] * v_eta[2] - v_xi[2] * v_eta[1],
        v_xi[2] * v_eta[0] - v_xi[0] * v_eta[2],
        v_xi[0] * v_eta[1] - v_xi[1] * v_eta[0],
    ];
    let norm = (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2])
        .sqrt()
        .max(1e-12);
    let e3 = [cross[0] / norm, cross[1] / norm, cross[2] / norm];
    let h = (e3[0] * e3[0] + e3[1] * e3[1]).sqrt();
    let (e1, e2) = if h <= 1e-4 {
        let sign_z = if e3[2] >= 0.0 { 1.0 } else { -1.0 };
        ([1.0, 0.0, 0.0], [0.0, sign_z, 0.0])
    } else {
        let e2 = [-e3[0] * e3[2] / h, -e3[1] * e3[2] / h, h];
        let e1 = [
            e2[1] * e3[2] - e2[2] * e3[1],
            e2[2] * e3[0] - e2[0] * e3[2],
            e2[0] * e3[1] - e2[1] * e3[0],
        ];
        (e1, e2)
    };
    coords
        .iter()
        .map(|c| {
            let d = [c[0] - centroid[0], c[1] - centroid[1], c[2] - centroid[2]];
            [
                e1[0] * d[0] + e1[1] * d[1] + e1[2] * d[2],
                e2[0] * d[0] + e2[1] * d[1] + e2[2] * d[2],
            ]
        })
        .collect()
}
