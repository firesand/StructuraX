//! Grid definition, object generation on grid intersections, selection by
//! property and diaphragm assignment.
//!
//! Every command works on a staged copy of the model and replaces the model
//! only when it succeeds, so a refused command leaves the model unchanged. Every
//! command refuses a locked model. Accepted geometry edits clear stale analysis
//! and design results; defining grid lines and selecting objects change no
//! analysis input and leave results alone.
//!
//! Naming follows [`SapModel::replicate_selected`]: new objects take the lowest
//! unused numeric name, and a joint that already exists at a requested
//! coordinate (within `1e-9 * max(1, largest |coordinate|)`) is reused.

use crate::editing::{finite, invalid};
use crate::{
    AreaObj, ConstraintAxis, ConstraintDef, ConstraintType, EditError, EditSummary, FrameObj,
    GridLine, PointObj, SapModel,
};
use std::collections::{BTreeSet, HashMap, HashSet};

/// Upper bound on objects one command may create.
pub(crate) const MAX_GENERATED: usize = 1_000_000;
/// Upper bound on grid lines per direction.
const MAX_GRID_LINES: usize = 1000;

/// Tolerance for treating two coordinates as the same point:
/// `1e-9 * max(1, largest |coordinate|)` over the model and `extra`.
pub(crate) fn geometry_tolerance(model: &SapModel, extra: &[[f64; 3]]) -> f64 {
    let mut scale = 1.0f64;
    for p in model.points.values() {
        scale = scale.max(p.x.abs()).max(p.y.abs()).max(p.z.abs());
    }
    for c in extra {
        for v in c {
            scale = scale.max(v.abs());
        }
    }
    1e-9 * scale
}

/// Hands out the lowest unused numeric names, in increasing order.
pub(crate) struct NameAlloc {
    occupied: HashSet<String>,
    next: usize,
}

impl NameAlloc {
    pub(crate) fn new<'a>(names: impl IntoIterator<Item = &'a String>) -> Self {
        Self {
            occupied: names.into_iter().cloned().collect(),
            next: 1,
        }
    }
    pub(crate) fn alloc(&mut self) -> String {
        loop {
            let name = self.next.to_string();
            self.next += 1;
            if self.occupied.insert(name.clone()) {
                return name;
            }
        }
    }
}

fn pair_key(a: &str, b: &str) -> (String, String) {
    if a <= b {
        (a.to_string(), b.to_string())
    } else {
        (b.to_string(), a.to_string())
    }
}

/// A staged copy of the model with a spatial index of its joints.
pub(crate) struct Stage {
    pub(crate) model: SapModel,
    pub(crate) summary: EditSummary,
    tol: f64,
    cells: HashMap<[i64; 3], Vec<String>>,
    points: NameAlloc,
    frames: NameAlloc,
    areas: NameAlloc,
    frame_pairs: HashMap<(String, String), String>,
    area_sets: HashSet<Vec<String>>,
    created: usize,
}

impl Stage {
    /// `extra` lists coordinates the command may create, so that the tolerance
    /// scale covers them.
    pub(crate) fn new(source: &SapModel, extra: &[[f64; 3]]) -> Self {
        let tol = geometry_tolerance(source, extra);
        let mut stage = Self {
            model: source.clone(),
            summary: EditSummary::default(),
            tol,
            cells: HashMap::new(),
            points: NameAlloc::new(source.points.keys()),
            frames: NameAlloc::new(source.frames.keys()),
            areas: NameAlloc::new(source.areas.keys()),
            frame_pairs: HashMap::new(),
            area_sets: HashSet::new(),
            created: 0,
        };
        for (name, p) in &source.points {
            let key = stage.cell_key([p.x, p.y, p.z]);
            stage.cells.entry(key).or_default().push(name.clone());
        }
        for f in source.frames.values() {
            stage
                .frame_pairs
                .insert(pair_key(&f.point_i, &f.point_j), f.name.clone());
        }
        for a in source.areas.values() {
            let mut set = a.points.clone();
            set.sort();
            stage.area_sets.insert(set);
        }
        stage
    }

    pub(crate) fn tol(&self) -> f64 {
        self.tol
    }

    fn cell_key(&self, c: [f64; 3]) -> [i64; 3] {
        std::array::from_fn(|k| (c[k] / self.tol).floor() as i64)
    }

    fn bump(&mut self) -> Result<(), EditError> {
        self.created += 1;
        if self.created > MAX_GENERATED {
            return Err(invalid("edit exceeds one million generated objects"));
        }
        Ok(())
    }

    /// The joint at `c`, if one exists within the tolerance (nearest wins).
    pub(crate) fn find_joint(&self, c: [f64; 3]) -> Option<String> {
        let k = self.cell_key(c);
        let mut best: Option<(f64, &String)> = None;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let Some(names) = self.cells.get(&[k[0] + dx, k[1] + dy, k[2] + dz]) else {
                        continue;
                    };
                    for name in names {
                        let p = &self.model.points[name];
                        let d = [p.x - c[0], p.y - c[1], p.z - c[2]];
                        if d.iter().all(|v| v.abs() <= self.tol) {
                            let dist = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
                            if best.is_none_or(|(b, n)| dist < b || (dist == b && name < n)) {
                                best = Some((dist, name));
                            }
                        }
                    }
                }
            }
        }
        best.map(|(_, n)| n.clone())
    }

    /// Reuse the joint at `c` or create a blank one.
    pub(crate) fn add_joint(&mut self, c: [f64; 3]) -> Result<String, EditError> {
        finite(c)?;
        if let Some(name) = self.find_joint(c) {
            return Ok(name);
        }
        self.insert_joint(PointObj::new("", c[0], c[1], c[2]))
    }

    /// Insert `p` as a new joint under a fresh numeric name.
    pub(crate) fn insert_joint(&mut self, mut p: PointObj) -> Result<String, EditError> {
        self.bump()?;
        let name = self.points.alloc();
        p.name.clone_from(&name);
        let key = self.cell_key([p.x, p.y, p.z]);
        self.cells.entry(key).or_default().push(name.clone());
        self.model.points.insert(name.clone(), p);
        self.summary.points_created.push(name.clone());
        Ok(name)
    }

    pub(crate) fn find_frame(&self, a: &str, b: &str) -> Option<String> {
        self.frame_pairs.get(&pair_key(a, b)).cloned()
    }

    /// Create a frame from `i` to `j`; `None` when a frame already joins them.
    pub(crate) fn add_frame(
        &mut self,
        i: &str,
        j: &str,
        section: &str,
        pinned: bool,
    ) -> Result<Option<String>, EditError> {
        if i == j {
            return Err(invalid("frame with identical end joints"));
        }
        if self.find_frame(i, j).is_some() {
            return Ok(None);
        }
        self.bump()?;
        let name = self.frames.alloc();
        let mut f = FrameObj::new(name.clone(), i, j, section);
        if pinned {
            // Release index convention: [P, V2, V3, T, M2, M3]. Torsion stays
            // fixed so that the member cannot spin about its own axis.
            f.release_i[4] = true;
            f.release_i[5] = true;
            f.release_j[4] = true;
            f.release_j[5] = true;
        }
        self.frame_pairs.insert(pair_key(i, j), name.clone());
        self.model.frames.insert(name.clone(), f);
        self.summary.frames_created.push(name.clone());
        Ok(Some(name))
    }

    /// Insert a fully built frame under a fresh name.
    pub(crate) fn insert_frame(&mut self, mut f: FrameObj) -> Result<Option<String>, EditError> {
        if self.find_frame(&f.point_i, &f.point_j).is_some() {
            return Ok(None);
        }
        self.bump()?;
        let name = self.frames.alloc();
        f.name.clone_from(&name);
        self.frame_pairs
            .insert(pair_key(&f.point_i, &f.point_j), name.clone());
        self.model.frames.insert(name.clone(), f);
        self.summary.frames_created.push(name.clone());
        Ok(Some(name))
    }

    /// Create an area; `None` when an area with the same joint set exists.
    pub(crate) fn add_area(
        &mut self,
        points: Vec<String>,
        section: &str,
    ) -> Result<Option<String>, EditError> {
        let mut set = points.clone();
        set.sort();
        if set.windows(2).any(|w| w[0] == w[1]) {
            return Err(invalid("area with a repeated joint"));
        }
        if !self.area_sets.insert(set) {
            return Ok(None);
        }
        self.bump()?;
        let name = self.areas.alloc();
        self.model
            .areas
            .insert(name.clone(), AreaObj::new(name.clone(), points, section));
        self.summary.areas_created.push(name.clone());
        Ok(Some(name))
    }

    /// Insert a fully built area under a fresh name (no duplicate check beyond
    /// the joint-set test of [`Stage::add_area`]).
    pub(crate) fn insert_area(&mut self, mut a: AreaObj) -> Result<Option<String>, EditError> {
        let mut set = a.points.clone();
        set.sort();
        if !self.area_sets.insert(set) {
            return Ok(None);
        }
        self.bump()?;
        let name = self.areas.alloc();
        a.name.clone_from(&name);
        self.model.areas.insert(name.clone(), a);
        self.summary.areas_created.push(name.clone());
        Ok(Some(name))
    }

    /// Replace `target` with the staged model and drop stale results.
    pub(crate) fn commit(self, target: &mut SapModel) -> EditSummary {
        let Stage { model, summary, .. } = self;
        *target = model;
        target.invalidate_results();
        summary
    }
}

/// Inclusive index ranges into the sorted GLOBAL grid lines of each direction
/// (index 0 is the lowest coordinate).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GridRegion {
    pub x: (usize, usize),
    pub y: (usize, usize),
    pub z: (usize, usize),
}

/// Bracing pattern in one bay, drawn in a vertical grid plane.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BraceKind {
    /// Two diagonals crossing at the middle of the bay, not connected there.
    X,
    /// Two diagonals from the top corners meeting at the middle of the beam at
    /// the bottom of the bay.
    V,
    /// Two diagonals from the bottom corners meeting at the middle of the beam
    /// at the top of the bay.
    InvertedV,
}

/// Vertical grid plane in which braces are drawn.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VerticalPlane {
    /// Plane parallel to X and Z, one plane per Y grid line of the region.
    XZ,
    /// Plane parallel to Y and Z, one plane per X grid line of the region.
    YZ,
}

/// Direction in which secondary beams span.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpanDirection {
    X,
    Y,
}

/// Excel-style label: A..Z, AA, AB, ...
fn column_label(mut i: usize) -> String {
    let mut out = Vec::new();
    loop {
        out.push(b'A' + (i % 26) as u8);
        if i < 26 {
            break;
        }
        i = i / 26 - 1;
    }
    out.reverse();
    String::from_utf8(out).expect("ascii")
}

impl SapModel {
    /// Replace the GLOBAL grid lines by a regular Cartesian grid (CSI "Quick
    /// Grid Lines"). `counts` are numbers of grid LINES in X, Y and Z (not
    /// spaces), `spacing` the distance between lines, and `origin` the location
    /// of the first line of each direction. Line `i` of a direction lies at
    /// `origin + i * spacing`. Lines are named A, B, ... (X), 1, 2, ... (Y) and
    /// Z1, Z2, ... (Z).
    pub fn quick_grid(
        &mut self,
        counts: [usize; 3],
        spacing: [f64; 3],
        origin: [f64; 3],
    ) -> Result<(), EditError> {
        self.check_edit()?;
        if counts.iter().any(|c| !(1..=MAX_GRID_LINES).contains(c)) {
            return Err(invalid(format!(
                "grid line counts must be 1..={MAX_GRID_LINES}"
            )));
        }
        finite(spacing)?;
        finite(origin)?;
        if spacing.iter().any(|s| *s <= 0.0) {
            return Err(invalid("grid spacing must be positive"));
        }
        let mut grids = Vec::with_capacity(counts.iter().sum());
        for i in 0..counts[0] {
            grids.push(GridLine {
                axis: "X".into(),
                id: column_label(i),
                coordinate: origin[0] + i as f64 * spacing[0],
                visible: true,
                bubble_loc: "End".into(),
            });
        }
        for i in 0..counts[1] {
            grids.push(GridLine {
                axis: "Y".into(),
                id: (i + 1).to_string(),
                coordinate: origin[1] + i as f64 * spacing[1],
                visible: true,
                bubble_loc: "Start".into(),
            });
        }
        for i in 0..counts[2] {
            grids.push(GridLine {
                axis: "Z".into(),
                id: format!("Z{}", i + 1),
                coordinate: origin[2] + i as f64 * spacing[2],
                visible: true,
                bubble_loc: "Start".into(),
            });
        }
        if grids.iter().any(|g| !g.coordinate.is_finite()) {
            return Err(invalid("grid coordinate overflow"));
        }
        let cs = self
            .coord_systems
            .get_mut("GLOBAL")
            .ok_or_else(|| invalid("model has no GLOBAL coordinate system"))?;
        cs.grids = grids;
        Ok(())
    }

    /// Sorted (ascending) GLOBAL grid coordinates of X, Y and Z.
    fn grid_axes(&self) -> Result<[Vec<f64>; 3], EditError> {
        let cs = self
            .coord_systems
            .get("GLOBAL")
            .ok_or_else(|| invalid("model has no GLOBAL coordinate system"))?;
        let mut axes: [Vec<f64>; 3] = Default::default();
        for g in &cs.grids {
            let k = match g.axis.as_str() {
                "X" => 0,
                "Y" => 1,
                "Z" => 2,
                other => return Err(invalid(format!("grid line {} has axis {other}", g.id))),
            };
            finite([g.coordinate])?;
            axes[k].push(g.coordinate);
        }
        for a in &mut axes {
            a.sort_by(f64::total_cmp);
            if a.windows(2).any(|w| w[1] - w[0] <= 0.0) {
                return Err(invalid("duplicate grid line coordinates"));
            }
        }
        Ok(axes)
    }

    fn checked_region(axes: &[Vec<f64>; 3], r: &GridRegion) -> Result<(), EditError> {
        for (k, (lo, hi)) in [r.x, r.y, r.z].into_iter().enumerate() {
            if lo > hi || hi >= axes[k].len() {
                return Err(invalid(format!(
                    "grid index range {lo}..={hi} is outside the {} defined {} grid lines",
                    axes[k].len(),
                    ["X", "Y", "Z"][k]
                )));
            }
        }
        Ok(())
    }

    fn require_frame_section(&self, name: &str) -> Result<(), EditError> {
        if self.frame_sections.contains_key(name) {
            Ok(())
        } else {
            Err(invalid(format!("missing frame section {name}")))
        }
    }

    fn finish_draw(&mut self, stage: Stage, what: &str) -> Result<EditSummary, EditError> {
        if stage.summary.frames_created.is_empty() && stage.summary.areas_created.is_empty() {
            return Err(invalid(format!(
                "{what} creates no objects (all exist already)"
            )));
        }
        Ok(stage.commit(self))
    }

    /// Draw beams on the GLOBAL grid at every Z grid line of `region`: along X
    /// when `along_x` (one per Y line, per X bay) and/or along Y when
    /// `along_y`. Beams run from the lower to the higher coordinate. Existing
    /// frames between the same joints are skipped.
    pub fn quick_draw_beams(
        &mut self,
        region: GridRegion,
        along_x: bool,
        along_y: bool,
        section: &str,
    ) -> Result<EditSummary, EditError> {
        self.check_edit()?;
        if !along_x && !along_y {
            return Err(invalid("select beams along X, along Y or both"));
        }
        self.require_frame_section(section)?;
        let axes = self.grid_axes()?;
        Self::checked_region(&axes, &region)?;
        let mut stage = Stage::new(self, &[]);
        let (x, y, z) = (&axes[0], &axes[1], &axes[2]);
        for iz in region.z.0..=region.z.1 {
            if along_x {
                for iy in region.y.0..=region.y.1 {
                    for ix in region.x.0..region.x.1 {
                        let a = stage.add_joint([x[ix], y[iy], z[iz]])?;
                        let b = stage.add_joint([x[ix + 1], y[iy], z[iz]])?;
                        stage.add_frame(&a, &b, section, false)?;
                    }
                }
            }
            if along_y {
                for ix in region.x.0..=region.x.1 {
                    for iy in region.y.0..region.y.1 {
                        let a = stage.add_joint([x[ix], y[iy], z[iz]])?;
                        let b = stage.add_joint([x[ix], y[iy + 1], z[iz]])?;
                        stage.add_frame(&a, &b, section, false)?;
                    }
                }
            }
        }
        self.finish_draw(stage, "quick draw beams")
    }

    /// Draw columns between consecutive Z grid lines of `region`, at every X
    /// and Y grid line of the region (bottom joint is `i`).
    pub fn quick_draw_columns(
        &mut self,
        region: GridRegion,
        section: &str,
    ) -> Result<EditSummary, EditError> {
        self.check_edit()?;
        self.require_frame_section(section)?;
        let axes = self.grid_axes()?;
        Self::checked_region(&axes, &region)?;
        let mut stage = Stage::new(self, &[]);
        let (x, y, z) = (&axes[0], &axes[1], &axes[2]);
        for iz in region.z.0..region.z.1 {
            for ix in region.x.0..=region.x.1 {
                for iy in region.y.0..=region.y.1 {
                    let a = stage.add_joint([x[ix], y[iy], z[iz]])?;
                    let b = stage.add_joint([x[ix], y[iy], z[iz + 1]])?;
                    stage.add_frame(&a, &b, section, false)?;
                }
            }
        }
        self.finish_draw(stage, "quick draw columns")
    }

    /// Draw pinned braces (M2 and M3 released at both ends) in every bay of a
    /// vertical grid plane. `XZ` uses every Y line of the region as a plane and
    /// the X and Z ranges as bays; `YZ` the converse. For `V` and `InvertedV`
    /// the apex joint lies at the middle of the beam at the bottom (V) or top
    /// (inverted V) of the bay; that beam must already exist between the two
    /// bay corners and gets `auto_mesh_at_joints = true` so that it is
    /// subdivided at the apex.
    pub fn quick_draw_braces(
        &mut self,
        region: GridRegion,
        plane: VerticalPlane,
        kind: BraceKind,
        section: &str,
    ) -> Result<EditSummary, EditError> {
        self.check_edit()?;
        self.require_frame_section(section)?;
        let axes = self.grid_axes()?;
        Self::checked_region(&axes, &region)?;
        let (h_axis, f_axis, h_range, f_range) = match plane {
            VerticalPlane::XZ => (0, 1, region.x, region.y),
            VerticalPlane::YZ => (1, 0, region.y, region.x),
        };
        let at = |h: f64, f: f64, z: f64| -> [f64; 3] {
            if h_axis == 0 {
                [h, f, z]
            } else {
                [f, h, z]
            }
        };
        let mut stage = Stage::new(self, &[]);
        let (hs, fs, zs) = (&axes[h_axis], &axes[f_axis], &axes[2]);
        for fi in f_range.0..=f_range.1 {
            let f = fs[fi];
            for hi in h_range.0..h_range.1 {
                for zi in region.z.0..region.z.1 {
                    let (h0, h1, z0, z1) = (hs[hi], hs[hi + 1], zs[zi], zs[zi + 1]);
                    let bl = stage.add_joint(at(h0, f, z0))?;
                    let br = stage.add_joint(at(h1, f, z0))?;
                    let tl = stage.add_joint(at(h0, f, z1))?;
                    let tr = stage.add_joint(at(h1, f, z1))?;
                    match kind {
                        BraceKind::X => {
                            stage.add_frame(&bl, &tr, section, true)?;
                            stage.add_frame(&br, &tl, section, true)?;
                        }
                        BraceKind::V | BraceKind::InvertedV => {
                            let (zb, e0, e1, c0, c1) = if kind == BraceKind::V {
                                (z0, &bl, &br, &tl, &tr)
                            } else {
                                (z1, &tl, &tr, &bl, &br)
                            };
                            let beam = stage.find_frame(e0, e1).ok_or_else(|| {
                                invalid(format!(
                                    "{kind:?} brace needs an existing beam between joints {e0} and {e1} at its apex level"
                                ))
                            })?;
                            let apex = stage.add_joint(at(h0 + 0.5 * (h1 - h0), f, zb))?;
                            stage.add_frame(c0, &apex, section, true)?;
                            stage.add_frame(c1, &apex, section, true)?;
                            stage
                                .model
                                .frames
                                .get_mut(&beam)
                                .expect("frame just found")
                                .auto_mesh_at_joints = true;
                        }
                    }
                }
            }
        }
        self.finish_draw(stage, "quick draw braces")
    }

    /// Draw `count` pinned secondary beams in every grid cell of `region` at
    /// every Z line of the region. A beam spanning X runs from the X line at
    /// one side of the cell to the other, at Y = y0 + (y1 - y0) * i / (count + 1),
    /// i = 1..=count; spanning Y is the converse. The two girders it frames into
    /// must exist between the cell corners; they get `auto_mesh_at_joints =
    /// true`. Beams are pinned with M2 and M3 released at both ends.
    pub fn quick_draw_secondary_beams(
        &mut self,
        region: GridRegion,
        span: SpanDirection,
        count: usize,
        section: &str,
    ) -> Result<EditSummary, EditError> {
        self.check_edit()?;
        if !(1..=1000).contains(&count) {
            return Err(invalid("1..=1000 secondary beams per bay required"));
        }
        self.require_frame_section(section)?;
        let axes = self.grid_axes()?;
        Self::checked_region(&axes, &region)?;
        let mut stage = Stage::new(self, &[]);
        let (x, y, z) = (&axes[0], &axes[1], &axes[2]);
        for iz in region.z.0..=region.z.1 {
            for ix in region.x.0..region.x.1 {
                for iy in region.y.0..region.y.1 {
                    let (x0, x1, y0, y1, zz) = (x[ix], x[ix + 1], y[iy], y[iy + 1], z[iz]);
                    // Girders are the cell sides perpendicular to the beams.
                    let girders = match span {
                        SpanDirection::X => {
                            [([x0, y0, zz], [x0, y1, zz]), ([x1, y0, zz], [x1, y1, zz])]
                        }
                        SpanDirection::Y => {
                            [([x0, y0, zz], [x1, y0, zz]), ([x0, y1, zz], [x1, y1, zz])]
                        }
                    };
                    let mut girder_names = Vec::new();
                    for (a, b) in girders {
                        let found = stage
                            .find_joint(a)
                            .zip(stage.find_joint(b))
                            .and_then(|(a, b)| stage.find_frame(&a, &b));
                        girder_names.push(found.ok_or_else(|| {
                            invalid(format!(
                                "secondary beams need an existing girder between {a:?} and {b:?}"
                            ))
                        })?);
                    }
                    for i in 1..=count {
                        let t = i as f64 / (count + 1) as f64;
                        let (a, b) = match span {
                            SpanDirection::X => {
                                let yy = y0 + (y1 - y0) * t;
                                ([x0, yy, zz], [x1, yy, zz])
                            }
                            SpanDirection::Y => {
                                let xx = x0 + (x1 - x0) * t;
                                ([xx, y0, zz], [xx, y1, zz])
                            }
                        };
                        let ja = stage.add_joint(a)?;
                        let jb = stage.add_joint(b)?;
                        stage.add_frame(&ja, &jb, section, true)?;
                    }
                    for g in girder_names {
                        stage
                            .model
                            .frames
                            .get_mut(&g)
                            .expect("frame just found")
                            .auto_mesh_at_joints = true;
                    }
                }
            }
        }
        self.finish_draw(stage, "quick draw secondary beams")
    }

    /// Draw one quadrilateral area per grid cell of `region` at every Z line of
    /// the region. Joints run (x0,y0), (x1,y0), (x1,y1), (x0,y1): counter
    /// clockwise seen from above, so the local 3 axis points to +Z.
    pub fn quick_draw_areas(
        &mut self,
        region: GridRegion,
        section: &str,
    ) -> Result<EditSummary, EditError> {
        self.check_edit()?;
        if !self.area_sections.contains_key(section) {
            return Err(invalid(format!("missing area section {section}")));
        }
        let axes = self.grid_axes()?;
        Self::checked_region(&axes, &region)?;
        let mut stage = Stage::new(self, &[]);
        let (x, y, z) = (&axes[0], &axes[1], &axes[2]);
        for iz in region.z.0..=region.z.1 {
            for ix in region.x.0..region.x.1 {
                for iy in region.y.0..region.y.1 {
                    let corners = [
                        [x[ix], y[iy], z[iz]],
                        [x[ix + 1], y[iy], z[iz]],
                        [x[ix + 1], y[iy + 1], z[iz]],
                        [x[ix], y[iy + 1], z[iz]],
                    ];
                    let mut ids = Vec::with_capacity(4);
                    for c in corners {
                        ids.push(stage.add_joint(c)?);
                    }
                    stage.add_area(ids, section)?;
                }
            }
        }
        self.finish_draw(stage, "quick draw areas")
    }

    /// Select the frames assigned to frame section `name`. With `replace` the
    /// selection of every other object is cleared first; otherwise the frames
    /// are added to the selection. Returns the number of frames selected.
    pub fn select_frames_by_section(
        &mut self,
        name: &str,
        replace: bool,
    ) -> Result<usize, EditError> {
        self.check_edit()?;
        if !self.frame_sections.contains_key(name) {
            return Err(invalid(format!("unknown frame section {name}")));
        }
        if replace {
            self.clear_selection();
        }
        let mut n = 0;
        for f in self.frames.values_mut().filter(|f| f.section == name) {
            f.selected = true;
            n += 1;
        }
        Ok(n)
    }

    /// Select the areas assigned to area section `name`; see
    /// [`SapModel::select_frames_by_section`].
    pub fn select_areas_by_section(
        &mut self,
        name: &str,
        replace: bool,
    ) -> Result<usize, EditError> {
        self.check_edit()?;
        if !self.area_sections.contains_key(name) {
            return Err(invalid(format!("unknown area section {name}")));
        }
        if replace {
            self.clear_selection();
        }
        let mut n = 0;
        for a in self.areas.values_mut().filter(|a| a.section == name) {
            a.selected = true;
            n += 1;
        }
        Ok(n)
    }

    fn clear_selection(&mut self) {
        self.points.values_mut().for_each(|p| p.selected = false);
        self.frames.values_mut().for_each(|p| p.selected = false);
        self.areas.values_mut().for_each(|p| p.selected = false);
        self.solids.values_mut().for_each(|p| p.selected = false);
        self.links.values_mut().for_each(|p| p.selected = false);
    }

    /// Assign the selected JOINTS to a Diaphragm constraint with axis Z.
    ///
    /// With `per_level` false all selected joints join the one constraint
    /// `name`. With `per_level` true the joints are grouped by Z coordinate
    /// (equal within the geometry tolerance) and each group joins its own
    /// constraint `{name}_{k}`, k = 1.. from the lowest level. (SAP2000 keeps a
    /// single constraint name for its "different diaphragm at each Z level"
    /// option and creates the levels internally; this model has no such
    /// per-level marker, so the levels are explicit constraints.) A constraint
    /// that already exists must be a Diaphragm with axis Z; it is reused. A
    /// joint already in another Diaphragm constraint leaves it. Returns the
    /// constraint names assigned, in order.
    pub fn assign_diaphragm(
        &mut self,
        name: &str,
        per_level: bool,
    ) -> Result<Vec<String>, EditError> {
        self.check_edit()?;
        if name.trim().is_empty() || name != name.trim() || name.chars().any(char::is_control) {
            return Err(invalid(
                "diaphragm name must be non-empty text without surrounding spaces",
            ));
        }
        let ids: Vec<String> = self
            .points
            .iter()
            .filter(|(_, p)| p.selected)
            .map(|(id, _)| id.clone())
            .collect();
        if ids.is_empty() {
            return Err(EditError::EmptySelection);
        }
        for id in &ids {
            finite([self.points[id].z])?;
        }
        let groups: Vec<(String, Vec<String>)> = if per_level {
            let tol = geometry_tolerance(self, &[]);
            let mut sorted = ids.clone();
            sorted.sort_by(|a, b| self.points[a].z.total_cmp(&self.points[b].z));
            let mut levels: Vec<(f64, Vec<String>)> = Vec::new();
            for id in sorted {
                let z = self.points[&id].z;
                match levels.last_mut() {
                    Some((z0, members)) if (z - *z0).abs() <= tol => members.push(id),
                    _ => levels.push((z, vec![id])),
                }
            }
            levels
                .into_iter()
                .enumerate()
                .map(|(k, (_, members))| (format!("{name}_{}", k + 1), members))
                .collect()
        } else {
            vec![(name.to_string(), ids)]
        };
        for (cname, _) in &groups {
            if let Some(c) = self.constraints.get(cname) {
                if c.constraint_type != ConstraintType::Diaphragm {
                    return Err(invalid(format!(
                        "constraint {cname} exists and is not a Diaphragm"
                    )));
                }
                if c.axis != ConstraintAxis::Z {
                    return Err(EditError::Unsupported(format!(
                        "constraint {cname} is a Diaphragm with an axis other than Z"
                    )));
                }
            }
        }
        if per_level {
            // An existing per-level constraint must not already hold joints of
            // another elevation, or one diaphragm would span two levels.
            let tol = geometry_tolerance(self, &[]);
            for (cname, members) in &groups {
                if !self.constraints.contains_key(cname) {
                    continue;
                }
                let z = self.points[&members[0]].z;
                if let Some((id, _)) = self.points.iter().find(|(_, p)| {
                    !p.selected && p.constraints.contains(cname) && (p.z - z).abs() > tol
                }) {
                    return Err(invalid(format!(
                        "constraint {cname} already holds joint {id} at another elevation"
                    )));
                }
            }
        }
        let mut staged = self.clone();
        let diaphragms: BTreeSet<String> = staged
            .constraints
            .values()
            .filter(|c| c.constraint_type == ConstraintType::Diaphragm)
            .map(|c| c.name.clone())
            .collect();
        let mut names = Vec::new();
        for (cname, members) in &groups {
            staged
                .constraints
                .entry(cname.clone())
                .or_insert_with(|| ConstraintDef {
                    name: cname.clone(),
                    constraint_type: ConstraintType::Diaphragm,
                    axis: ConstraintAxis::Z,
                    dofs: [true; 6],
                });
            for id in members {
                let p = staged.points.get_mut(id).expect("selected joint");
                p.constraints
                    .retain(|c| !diaphragms.contains(c) || c == cname);
                if !p.constraints.contains(cname) {
                    p.constraints.push(cname.clone());
                }
            }
            names.push(cname.clone());
        }
        staged.invalidate_results();
        *self = staged;
        Ok(names)
    }

    /// Remove every Diaphragm constraint assignment from the selected joints
    /// and delete the constraints that this leaves unused and whose names end
    /// in `_<digits>` (the names [`SapModel::assign_diaphragm`] generates for
    /// per-level diaphragms). Returns the names of the deleted constraints.
    pub fn remove_diaphragm_assignments(&mut self) -> Result<Vec<String>, EditError> {
        self.check_edit()?;
        if !self.points.values().any(|p| p.selected) {
            return Err(EditError::EmptySelection);
        }
        let mut staged = self.clone();
        let diaphragms: BTreeSet<String> = staged
            .constraints
            .values()
            .filter(|c| c.constraint_type == ConstraintType::Diaphragm)
            .map(|c| c.name.clone())
            .collect();
        let mut touched = BTreeSet::new();
        for p in staged.points.values_mut().filter(|p| p.selected) {
            p.constraints.retain(|c| {
                if diaphragms.contains(c) {
                    touched.insert(c.clone());
                    false
                } else {
                    true
                }
            });
        }
        let mut deleted = Vec::new();
        for c in touched {
            let generated = c.rsplit_once('_').is_some_and(|(stem, k)| {
                !stem.is_empty() && !k.is_empty() && k.bytes().all(|b| b.is_ascii_digit())
            });
            let used = staged.points.values().any(|p| p.constraints.contains(&c));
            if generated && !used && staged.constraints[&c].axis == ConstraintAxis::Z {
                staged.constraints.shift_remove(&c);
                deleted.push(c);
            }
        }
        staged.invalidate_results();
        *self = staged;
        Ok(deleted)
    }
}
