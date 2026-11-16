//! Window selection and "Get Previous Selection".
//!
//! Source: SAP2000 help, "Pointer/Window" (dragged left to right a window
//! selects the objects entirely inside it, right to left also the objects it
//! crosses; windows add to the selection; only displayed objects are
//! selectable) and "Get Previous Selection".

use super::view::{plane_members, plane_tolerance, PlaneView, ViewTransform, SOLID_FACES};
use eframe::egui::{Pos2, Rect};
use sap_model::SapModel;
use std::collections::BTreeSet;

/// The names of the selected objects.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub(super) struct Snapshot {
    pub points: BTreeSet<String>,
    pub frames: BTreeSet<String>,
    pub areas: BTreeSet<String>,
    pub solids: BTreeSet<String>,
    pub links: BTreeSet<String>,
}

impl Snapshot {
    pub(super) fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub(super) fn len(&self) -> usize {
        self.points.len()
            + self.frames.len()
            + self.areas.len()
            + self.solids.len()
            + self.links.len()
    }

    /// Whether the two selections share at least one object.
    fn overlaps(&self, other: &Snapshot) -> bool {
        !self.points.is_disjoint(&other.points)
            || !self.frames.is_disjoint(&other.frames)
            || !self.areas.is_disjoint(&other.areas)
            || !self.solids.is_disjoint(&other.solids)
            || !self.links.is_disjoint(&other.links)
    }
}

/// What is needed to restore the previous selection.
#[derive(Debug, Clone, Default)]
pub(super) struct State {
    /// The selection seen by the last call of [`track`].
    last: Snapshot,
    /// The last non-empty selection that was cleared or entirely replaced.
    previous: Option<Snapshot>,
}

/// The current selection of `model`.
pub(super) fn snapshot(model: &SapModel) -> Snapshot {
    fn names<'a, T: 'a>(
        items: impl Iterator<Item = (&'a String, &'a T)>,
        selected: impl Fn(&T) -> bool,
    ) -> BTreeSet<String> {
        items
            .filter(|(_, o)| selected(o))
            .map(|(n, _)| n.clone())
            .collect()
    }
    Snapshot {
        points: names(model.points.iter(), |o| o.selected),
        frames: names(model.frames.iter(), |o| o.selected),
        areas: names(model.areas.iter(), |o| o.selected),
        solids: names(model.solids.iter(), |o| o.selected),
        links: names(model.links.iter(), |o| o.selected),
    }
}

/// Called once per frame: remembers a selection when it is cleared or
/// replaced. A selection that only grows or shrinks (some object stays
/// selected) is the same selection edited, not a new one.
pub(super) fn track(state: &mut State, model: &SapModel) {
    let current = snapshot(model);
    if current == state.last {
        return;
    }
    if !state.last.is_empty() && (current.is_empty() || !current.overlaps(&state.last)) {
        state.previous = Some(std::mem::take(&mut state.last));
    }
    state.last = current;
}

/// Replace the selection by the previous one. Names of objects that no longer
/// exist are dropped; when none is left, or nothing was stored, the model is
/// left as it is and the reason is returned.
pub(super) fn restore_previous(state: &State, model: &mut SapModel) -> Result<String, String> {
    let Some(previous) = &state.previous else {
        return Err("there is no previous selection".into());
    };
    let alive = Snapshot {
        points: previous
            .points
            .iter()
            .filter(|n| model.points.contains_key(*n))
            .cloned()
            .collect(),
        frames: previous
            .frames
            .iter()
            .filter(|n| model.frames.contains_key(*n))
            .cloned()
            .collect(),
        areas: previous
            .areas
            .iter()
            .filter(|n| model.areas.contains_key(*n))
            .cloned()
            .collect(),
        solids: previous
            .solids
            .iter()
            .filter(|n| model.solids.contains_key(*n))
            .cloned()
            .collect(),
        links: previous
            .links
            .iter()
            .filter(|n| model.links.contains_key(*n))
            .cloned()
            .collect(),
    };
    if alive.is_empty() {
        return Err("none of the previously selected objects exists any more".into());
    }
    for (name, o) in model.points.iter_mut() {
        o.selected = alive.points.contains(name);
    }
    for (name, o) in model.frames.iter_mut() {
        o.selected = alive.frames.contains(name);
    }
    for (name, o) in model.areas.iter_mut() {
        o.selected = alive.areas.contains(name);
    }
    for (name, o) in model.solids.iter_mut() {
        o.selected = alive.solids.contains(name);
    }
    for (name, o) in model.links.iter_mut() {
        o.selected = alive.links.contains(name);
    }
    let dropped = previous.len() - alive.len();
    Ok(if dropped == 0 {
        format!("Previous selection restored: {} objects.", alive.len())
    } else {
        format!(
            "Previous selection restored: {} objects ({dropped} no longer exist).",
            alive.len()
        )
    })
}

fn inside(r: Rect, p: Pos2) -> bool {
    r.min.x <= p.x && p.x <= r.max.x && r.min.y <= p.y && p.y <= r.max.y
}

fn orient(a: Pos2, b: Pos2, c: Pos2) -> f32 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}

/// Whether the closed segments `a`-`b` and `c`-`d` share a point.
fn segments_meet(a: Pos2, b: Pos2, c: Pos2, d: Pos2) -> bool {
    let on = |p: Pos2, q: Pos2, r: Pos2| {
        r.x >= p.x.min(q.x) && r.x <= p.x.max(q.x) && r.y >= p.y.min(q.y) && r.y <= p.y.max(q.y)
    };
    let (o1, o2, o3, o4) = (orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b));
    if ((o1 > 0.0) != (o2 > 0.0)) && o1 != 0.0 && o2 != 0.0
        && ((o3 > 0.0) != (o4 > 0.0)) && o3 != 0.0 && o4 != 0.0
    {
        return true;
    }
    (o1 == 0.0 && on(a, b, c))
        || (o2 == 0.0 && on(a, b, d))
        || (o3 == 0.0 && on(c, d, a))
        || (o4 == 0.0 && on(c, d, b))
}

fn segment_touches(r: Rect, a: Pos2, b: Pos2) -> bool {
    if inside(r, a) || inside(r, b) {
        return true;
    }
    let corners = [r.left_top(), r.right_top(), r.right_bottom(), r.left_bottom()];
    (0..4).any(|k| segments_meet(a, b, corners[k], corners[(k + 1) % 4]))
}

/// Whether the closed polygon `poly` (at least three corners) and the
/// rectangle share a point.
fn polygon_touches(r: Rect, poly: &[Pos2]) -> bool {
    if poly.iter().any(|p| inside(r, *p)) {
        return true;
    }
    if (0..poly.len()).any(|k| segment_touches(r, poly[k], poly[(k + 1) % poly.len()])) {
        return true;
    }
    // The rectangle lies inside a big polygon.
    super::point_in_polygon(r.center(), poly)
}

/// Add the objects in the screen window `from`..`to` to the selection and
/// return how many were added. Dragged left to right the window takes objects
/// fully inside it, right to left also those it crosses (SAP2000 help,
/// "Pointer/Window"). In a plane view only the objects in the plane count and
/// solids are not selectable (their faces in the plane are only outlines).
pub(super) fn select_window(
    model: &mut SapModel,
    view: &ViewTransform,
    plane: Option<PlaneView>,
    from: Pos2,
    to: Pos2,
) -> usize {
    let rect = Rect::from_two_pos(from, to);
    let crossing = to.x < from.x;
    let members = plane.map(|p| plane_members(model, p, plane_tolerance(model)));
    let shown = |set: fn(&super::view::Members) -> &BTreeSet<String>, name: &String| {
        members.as_ref().is_none_or(|m| set(m).contains(name))
    };
    let at = |name: &String, model: &SapModel| {
        model.points.get(name).map(|p| view.project([p.x, p.y, p.z]))
    };
    let mut added = 0;
    let mut hit = Vec::new();

    for (name, p) in &model.points {
        if !p.selected && shown(|m| &m.points, name) && inside(rect, view.project([p.x, p.y, p.z])) {
            hit.push(name.clone());
        }
    }
    for name in hit.drain(..) {
        if let Some(o) = model.points.get_mut(&name) {
            o.selected = true;
            added += 1;
        }
    }

    for (name, f) in &model.frames {
        if f.selected || !shown(|m| &m.frames, name) {
            continue;
        }
        let (Some(a), Some(b)) = (at(&f.point_i, model), at(&f.point_j, model)) else {
            continue;
        };
        let take = if crossing {
            segment_touches(rect, a, b)
        } else {
            inside(rect, a) && inside(rect, b)
        };
        if take {
            hit.push(name.clone());
        }
    }
    for name in hit.drain(..) {
        if let Some(o) = model.frames.get_mut(&name) {
            o.selected = true;
            added += 1;
        }
    }

    for (name, l) in &model.links {
        if l.selected || !shown(|m| &m.links, name) {
            continue;
        }
        let Some(a) = at(&l.point_i, model) else {
            continue;
        };
        let b = match &l.point_j {
            Some(j) => match at(j, model) {
                Some(b) => b,
                None => continue,
            },
            None => a,
        };
        let take = if crossing {
            segment_touches(rect, a, b)
        } else {
            inside(rect, a) && inside(rect, b)
        };
        if take {
            hit.push(name.clone());
        }
    }
    for name in hit.drain(..) {
        if let Some(o) = model.links.get_mut(&name) {
            o.selected = true;
            added += 1;
        }
    }

    for (name, a) in &model.areas {
        if a.selected || !shown(|m| &m.areas, name) {
            continue;
        }
        let poly: Vec<Pos2> = a.points.iter().filter_map(|p| at(p, model)).collect();
        if poly.len() != a.points.len() || poly.len() < 3 {
            continue;
        }
        let take = if crossing {
            polygon_touches(rect, &poly)
        } else {
            poly.iter().all(|p| inside(rect, *p))
        };
        if take {
            hit.push(name.clone());
        }
    }
    for name in hit.drain(..) {
        if let Some(o) = model.areas.get_mut(&name) {
            o.selected = true;
            added += 1;
        }
    }

    if plane.is_none() {
        for (name, s) in &model.solids {
            if s.selected {
                continue;
            }
            let corners: Vec<Pos2> = s.points.iter().filter_map(|p| at(p, model)).collect();
            if corners.len() != s.points.len() || corners.len() != 8 {
                continue;
            }
            let take = if crossing {
                SOLID_FACES.iter().any(|f| {
                    let face = [corners[f[0]], corners[f[1]], corners[f[2]], corners[f[3]]];
                    polygon_touches(rect, &face)
                })
            } else {
                corners.iter().all(|p| inside(rect, *p))
            };
            if take {
                hit.push(name.clone());
            }
        }
        for name in hit.drain(..) {
            if let Some(o) = model.solids.get_mut(&name) {
                o.selected = true;
                added += 1;
            }
        }
    }
    added
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gui::ViewProjection;
    use sap_units::Units;

    fn portal() -> SapModel {
        // Two bays, one storey, in the X-Z plane at y = 0.
        let mut m = SapModel::new(Units::KnMC);
        for (i, x) in [0.0, 6.0, 12.0].iter().enumerate() {
            m.add_point(format!("B{i}"), *x, 0.0, 0.0);
            m.add_point(format!("T{i}"), *x, 0.0, 4.0);
            m.add_frame(format!("C{i}"), format!("B{i}"), format!("T{i}"), "FSEC1");
        }
        m.add_frame("G1", "T0", "T1", "FSEC1");
        m.add_frame("G2", "T1", "T2", "FSEC1");
        m
    }

    fn elevation() -> ViewTransform {
        ViewTransform {
            proj: ViewProjection::ElevationXZ,
            center: [6.0, 0.0, 2.0],
            scale: 20.0,
            screen_center: Pos2::new(500.0, 400.0),
            yaw: 0.0,
            pitch: 0.0,
        }
    }

    fn names(m: &SapModel) -> (Vec<String>, Vec<String>) {
        let s = snapshot(m);
        (s.frames.into_iter().collect(), s.points.into_iter().collect())
    }

    #[test]
    fn a_window_dragged_right_takes_what_is_inside_and_left_also_what_it_crosses() {
        let v = elevation();
        // Bay 1 is x = 0..6, z = 0..4; the window runs from x = -1 to 9, so
        // beam G2 (x = 6..12) leaves it at x = 9.
        let a = v.project([-1.0, 0.0, -1.0]);
        let b = v.project([9.0, 0.0, 5.0]);
        let (lo, hi) = (a.min(b), a.max(b));
        let (left, right) = (Pos2::new(lo.x, lo.y), Pos2::new(hi.x, hi.y));
        let mut m = portal();
        let plane = PlaneView::of(ViewProjection::ElevationXZ, Some(0.0));
        let n = select_window(&mut m, &v, plane, left, right);
        let (frames, points) = names(&m);
        assert_eq!(frames, vec!["C0", "C1", "G1"]);
        assert_eq!(points, vec!["B0", "B1", "T0", "T1"]);
        assert_eq!(n, 7);
        // Right to left also takes the beam that crosses the window edge;
        // windows add, and the second call counts only what is new.
        let mut m = portal();
        let n = select_window(&mut m, &v, plane, right, left);
        let (frames, points) = names(&m);
        assert_eq!(frames, vec!["C0", "C1", "G1", "G2"]);
        assert_eq!(points.len(), 4);
        assert_eq!(n, 8);
        assert_eq!(select_window(&mut m, &v, plane, right, left), 0);
    }

    #[test]
    fn a_plane_view_selects_only_objects_in_its_plane() {
        let v = ViewTransform {
            proj: ViewProjection::PlanXY,
            center: [6.0, 0.0, 2.0],
            ..elevation()
        };
        let mut m = portal();
        // In plan at z = 4 only the top joints and the two beams are shown.
        let plane = PlaneView::of(ViewProjection::PlanXY, Some(4.0));
        let a = v.project([-1.0, -1.0, 4.0]);
        let b = v.project([13.0, 1.0, 4.0]);
        select_window(&mut m, &v, plane, a, b);
        let (frames, points) = names(&m);
        assert_eq!(frames, vec!["G1", "G2"]);
        assert_eq!(points, vec!["T0", "T1", "T2"]);
        // Without a plane the same window takes the joints below as well.
        let mut m = portal();
        select_window(&mut m, &v, None, a, b);
        assert_eq!(names(&m).1.len(), 6);
    }

    #[test]
    fn crossing_tests_see_a_window_inside_a_large_area_and_a_segment_through_a_window() {
        let mut m = SapModel::new(Units::KnMC);
        m.add_point("1", 0.0, 0.0, 0.0);
        m.add_point("2", 10.0, 0.0, 0.0);
        m.add_point("3", 10.0, 10.0, 0.0);
        m.add_point("4", 0.0, 10.0, 0.0);
        m.add_area("A", vec!["1".into(), "2".into(), "3".into(), "4".into()], "ASEC1");
        m.add_frame("F", "1", "3", "FSEC1");
        let v = ViewTransform {
            proj: ViewProjection::PlanXY,
            center: [5.0, 5.0, 0.0],
            scale: 20.0,
            screen_center: Pos2::new(500.0, 400.0),
            yaw: 0.0,
            pitch: 0.0,
        };
        // A small window in the middle of the area, away from every joint.
        let c = v.project([5.0, 5.0, 0.0]);
        let (from, to) = (c + eframe::egui::vec2(3.0, 3.0), c - eframe::egui::vec2(3.0, 3.0));
        let n = select_window(&mut m, &v, None, from, to);
        assert!(m.areas["A"].selected && m.frames["F"].selected);
        assert_eq!(n, 2);
        // Left to right it takes neither.
        let mut m2 = m.clone();
        for a in m2.areas.values_mut() {
            a.selected = false;
        }
        for f in m2.frames.values_mut() {
            f.selected = false;
        }
        assert_eq!(select_window(&mut m2, &v, None, to, from), 0);
    }

    #[test]
    fn the_previous_selection_is_the_last_one_cleared_or_replaced() {
        let mut m = portal();
        let mut st = State::default();
        track(&mut st, &m);
        assert!(restore_previous(&st, &mut m).is_err());
        // {C0, C1}, built up in two steps, then cleared.
        m.frames.get_mut("C0").unwrap().selected = true;
        track(&mut st, &m);
        m.frames.get_mut("C1").unwrap().selected = true;
        track(&mut st, &m);
        assert!(restore_previous(&st, &mut m).is_err(), "still the current one");
        for f in m.frames.values_mut() {
            f.selected = false;
        }
        track(&mut st, &m);
        assert!(snapshot(&m).is_empty());
        let msg = restore_previous(&st, &mut m).unwrap();
        assert!(msg.contains('2'), "{msg}");
        assert_eq!(names(&m).0, vec!["C0", "C1"]);
        // Replacing by a disjoint selection stores the one replaced.
        track(&mut st, &m);
        for f in m.frames.values_mut() {
            f.selected = false;
        }
        m.frames.get_mut("G1").unwrap().selected = true;
        track(&mut st, &m);
        let _ = restore_previous(&st, &mut m).unwrap();
        assert_eq!(names(&m).0, vec!["C0", "C1"]);
        // Dropping one object of three is an edit, not a replacement.
        track(&mut st, &m);
        m.frames.get_mut("C1").unwrap().selected = false;
        track(&mut st, &m);
        track(&mut st, &m);
        assert_eq!(names(&m).0, vec!["C0"]);
        let _ = restore_previous(&st, &mut m).unwrap();
        assert_eq!(names(&m).0, vec!["G1"], "the edit did not replace the stored selection");
    }

    #[test]
    fn restoring_drops_names_that_no_longer_exist_and_refuses_when_none_remain() {
        let mut m = portal();
        let mut st = State::default();
        m.frames.get_mut("C0").unwrap().selected = true;
        m.frames.get_mut("G2").unwrap().selected = true;
        track(&mut st, &m);
        for f in m.frames.values_mut() {
            f.selected = false;
        }
        track(&mut st, &m);
        m.frames.shift_remove("G2");
        let msg = restore_previous(&st, &mut m).unwrap();
        assert!(msg.contains("no longer exist"), "{msg}");
        assert_eq!(names(&m).0, vec!["C0"]);
        // Nothing left to restore: the model is not touched.
        for f in m.frames.values_mut() {
            f.selected = false;
        }
        m.frames.shift_remove("C0");
        m.points.get_mut("B0").unwrap().selected = true;
        assert!(restore_previous(&st, &mut m).is_err());
        assert_eq!(names(&m).1, vec!["B0"], "a refused restore leaves the selection");
    }
}
