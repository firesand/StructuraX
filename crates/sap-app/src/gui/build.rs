//! Grid, quick-draw, radial and selection commands.
//!
//! Every command is a plain function that takes the model and typed input and
//! returns a status message or the reason it was refused. A refused command
//! leaves the model unchanged (the model methods stage their edit), and every
//! command refuses while the model is locked.

use eframe::egui;
use sap_model::generate::{BraceKind, GridAxis, GridRegion, SpanDirection, VerticalPlane};
use sap_model::radial::{CurveKind, ReverseKeep};
use sap_model::{EditSummary, SapModel};

#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
enum DrawKind {
    #[default]
    Beams,
    Columns,
    Braces,
    SecondaryBeams,
    Areas,
}

impl DrawKind {
    const ALL: [Self; 5] = [
        Self::Beams,
        Self::Columns,
        Self::Braces,
        Self::SecondaryBeams,
        Self::Areas,
    ];
    fn label(self) -> &'static str {
        match self {
            Self::Beams => "Beams",
            Self::Columns => "Columns",
            Self::Braces => "Braces",
            Self::SecondaryBeams => "Secondary beams",
            Self::Areas => "Areas (one per grid cell)",
        }
    }
}

#[derive(Clone, Debug)]
struct QuickDrawInput {
    kind: DrawKind,
    /// Inclusive grid line index ranges `[first, last]` (0 = lowest coordinate).
    x: [usize; 2],
    y: [usize; 2],
    z: [usize; 2],
    along_x: bool,
    along_y: bool,
    plane: VerticalPlane,
    brace: BraceKind,
    span: SpanDirection,
    count: usize,
    frame_section: String,
    area_section: String,
}

impl Default for QuickDrawInput {
    fn default() -> Self {
        Self {
            kind: DrawKind::Beams,
            x: [0, 1],
            y: [0, 1],
            z: [0, 1],
            along_x: true,
            along_y: true,
            plane: VerticalPlane::XZ,
            brace: BraceKind::X,
            span: SpanDirection::X,
            count: 1,
            frame_section: String::new(),
            area_section: String::new(),
        }
    }
}

#[derive(Clone, Debug)]
struct RadialInput {
    center: [f64; 2],
    angle_deg: f64,
    copies: usize,
    steps: usize,
    rise: f64,
    area_section: String,
    delete_source: bool,
}

impl Default for RadialInput {
    fn default() -> Self {
        Self {
            center: [0.0; 2],
            angle_deg: 30.0,
            copies: 1,
            steps: 12,
            rise: 0.0,
            area_section: String::new(),
            delete_source: false,
        }
    }
}

#[derive(Clone, Debug)]
struct CurvedInput {
    start: [f64; 3],
    end: [f64; 3],
    third: [f64; 3],
    kind: CurveKind,
    segments: usize,
    section: String,
}

impl Default for CurvedInput {
    fn default() -> Self {
        Self {
            start: [0.0; 3],
            end: [10.0, 0.0, 0.0],
            third: [5.0, 0.0, 2.0],
            kind: CurveKind::Circular,
            segments: 8,
            section: String::new(),
        }
    }
}

#[derive(Clone, Debug)]
struct SelectInput {
    frame_section: String,
    area_section: String,
    replace: bool,
    diaphragm: String,
    per_level: bool,
}

impl Default for SelectInput {
    fn default() -> Self {
        Self {
            frame_section: String::new(),
            area_section: String::new(),
            replace: true,
            diaphragm: "DIAPH".into(),
            per_level: true,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
enum GridEditMode {
    /// One `id coordinate` per line of the text.
    #[default]
    Lines,
    /// Spacings after a first coordinate; the ids are made up.
    Spacings,
}

/// Input of the grid line editor (one direction at a time).
#[derive(Clone, Debug)]
struct GridEditInput {
    axis: GridAxis,
    mode: GridEditMode,
    text: String,
    /// Coordinate of the first line in [`GridEditMode::Spacings`].
    origin: f64,
}

impl Default for GridEditInput {
    fn default() -> Self {
        Self {
            axis: GridAxis::X,
            mode: GridEditMode::Lines,
            text: String::new(),
            origin: 0.0,
        }
    }
}

/// Window state. It is reset when another model is loaded.
pub(super) struct State {
    pub(super) open: bool,
    tab: usize,
    grid_counts: [usize; 3],
    grid_spacing: [f64; 3],
    grid_origin: [f64; 3],
    grid_edit: GridEditInput,
    draw: QuickDrawInput,
    radial: RadialInput,
    curved: CurvedInput,
    select: SelectInput,
}

/// Tab of the grid commands (quick grid and the grid line editor).
pub(super) const TAB_GRID: usize = 0;

impl State {
    /// Open the window on `tab`.
    pub(super) fn show_tab(&mut self, tab: usize) {
        self.open = true;
        self.tab = tab;
    }
}

/// Positive lengths from a list separated by commas or blanks.
pub(super) fn parse_lengths(text: &str) -> Result<Vec<f64>, String> {
    let mut lengths = Vec::new();
    for token in text
        .split(|c: char| c == ',' || c.is_whitespace())
        .filter(|t| !t.is_empty())
    {
        let value: f64 = token
            .parse()
            .map_err(|_| format!("'{token}' is not a number"))?;
        if !value.is_finite() || value <= 0.0 {
            return Err(format!("'{token}' is not a positive length"));
        }
        lengths.push(value);
    }
    if lengths.is_empty() {
        return Err("enter at least one length".into());
    }
    Ok(lengths)
}

/// Grid coordinates from an origin and spacings: `origin`, then one more
/// coordinate per spacing.
pub(super) fn cumulative(origin: f64, spacings: &[f64]) -> Vec<f64> {
    let mut coordinates = Vec::with_capacity(spacings.len() + 1);
    let mut at = origin;
    coordinates.push(at);
    for spacing in spacings {
        at += spacing;
        coordinates.push(at);
    }
    coordinates
}

/// Id of the `i`-th line made up for a direction, as the quick grid names
/// them: A, B, ..., Z, AA, ... for X; 1, 2, ... for Y; Z1, Z2, ... for Z.
fn made_up_id(axis: GridAxis, i: usize) -> String {
    match axis {
        GridAxis::X => {
            let mut letters = Vec::new();
            let mut n = i;
            loop {
                letters.push(b'A' + (n % 26) as u8);
                if n < 26 {
                    break;
                }
                n = n / 26 - 1;
            }
            letters.reverse();
            String::from_utf8(letters).expect("ascii")
        }
        GridAxis::Y => (i + 1).to_string(),
        GridAxis::Z => format!("Z{}", i + 1),
    }
}

/// Grid lines typed as one `id coordinate` per line (blank lines are
/// skipped; blanks, tabs and commas separate the two values).
fn parse_grid_lines(text: &str) -> Result<Vec<(String, f64)>, String> {
    let mut lines = Vec::new();
    for (n, line) in text.lines().enumerate() {
        let tokens: Vec<&str> = line
            .split(|c: char| c == ',' || c.is_whitespace())
            .filter(|t| !t.is_empty())
            .collect();
        match tokens.as_slice() {
            [] => {}
            [id, coordinate] => {
                let value: f64 = coordinate.parse().map_err(|_| {
                    format!("line {}: '{coordinate}' is not a number", n + 1)
                })?;
                lines.push(((*id).to_string(), value));
            }
            _ => {
                return Err(format!(
                    "line {}: expected an id and a coordinate, e.g. 'A 0'",
                    n + 1
                ))
            }
        }
    }
    Ok(lines)
}

/// The grid lines of one direction as the editor's text.
fn grid_lines_text(model: &SapModel, axis: GridAxis) -> String {
    model
        .coord_systems
        .get("GLOBAL")
        .map(|cs| {
            cs.grids
                .iter()
                .filter(|g| g.axis == axis.label())
                .map(|g| format!("{} {}\n", g.id, g.coordinate))
                .collect()
        })
        .unwrap_or_default()
}

impl Default for State {
    fn default() -> Self {
        Self {
            open: false,
            tab: 0,
            grid_counts: [4, 4, 4],
            grid_spacing: [20.0, 20.0, 12.0],
            grid_origin: [0.0; 3],
            grid_edit: GridEditInput::default(),
            draw: QuickDrawInput::default(),
            radial: RadialInput::default(),
            curved: CurvedInput::default(),
            select: SelectInput::default(),
        }
    }
}

// --- commands --------------------------------------------------------------

fn guard(model: &SapModel) -> Result<(), String> {
    if model.is_locked {
        Err("The model is locked. Use Edit > Unlock Model before changing the geometry.".into())
    } else {
        Ok(())
    }
}

fn created(what: &str, s: &EditSummary) -> String {
    format!(
        "{what}: created {} joints, {} frames, {} areas.",
        s.points_created.len(),
        s.frames_created.len(),
        s.areas_created.len()
    )
}

fn apply_grid(
    model: &mut SapModel,
    counts: [usize; 3],
    spacing: [f64; 3],
    origin: [f64; 3],
) -> Result<String, String> {
    guard(model)?;
    model
        .quick_grid(counts, spacing, origin)
        .map(|()| {
            format!(
                "Defined {} x {} x {} grid lines.",
                counts[0], counts[1], counts[2]
            )
        })
        .map_err(|e| e.to_string())
}

fn apply_grid_lines(model: &mut SapModel, input: &GridEditInput) -> Result<String, String> {
    guard(model)?;
    let lines = match input.mode {
        GridEditMode::Lines => parse_grid_lines(&input.text)?,
        GridEditMode::Spacings => {
            if !input.origin.is_finite() {
                return Err("the first coordinate must be finite".into());
            }
            let spacings = parse_lengths(&input.text)?;
            cumulative(input.origin, &spacings)
                .into_iter()
                .enumerate()
                .map(|(i, c)| (made_up_id(input.axis, i), c))
                .collect()
        }
    };
    let axis = input.axis.label();
    model
        .set_grid_lines(input.axis, &lines)
        .map(|()| {
            if lines.is_empty() {
                format!("Removed the {axis} grid lines.")
            } else {
                format!("Defined {} {axis} grid lines.", lines.len())
            }
        })
        .map_err(|e| e.to_string())
}

fn apply_quick_draw(model: &mut SapModel, d: &QuickDrawInput) -> Result<String, String> {
    guard(model)?;
    let region = GridRegion {
        x: (d.x[0], d.x[1]),
        y: (d.y[0], d.y[1]),
        z: (d.z[0], d.z[1]),
    };
    let fs = d.frame_section.as_str();
    let result = match d.kind {
        DrawKind::Beams => model.quick_draw_beams(region, d.along_x, d.along_y, fs),
        DrawKind::Columns => model.quick_draw_columns(region, fs),
        DrawKind::Braces => model.quick_draw_braces(region, d.plane, d.brace, fs),
        DrawKind::SecondaryBeams => model.quick_draw_secondary_beams(region, d.span, d.count, fs),
        DrawKind::Areas => model.quick_draw_areas(region, &d.area_section),
    };
    result
        .map(|s| created(d.kind.label(), &s))
        .map_err(|e| e.to_string())
}

fn apply_replicate_radial(model: &mut SapModel, r: &RadialInput) -> Result<String, String> {
    guard(model)?;
    model
        .replicate_radial(r.center, r.angle_deg, r.copies)
        .map(|s| created("Radial replicate", &s))
        .map_err(|e| e.to_string())
}

fn apply_extrude_radial(model: &mut SapModel, r: &RadialInput) -> Result<String, String> {
    guard(model)?;
    model
        .extrude_lines_to_areas_radial(
            r.center,
            r.angle_deg,
            r.steps,
            r.rise,
            &r.area_section,
            r.delete_source,
        )
        .map(|s| {
            format!(
                "{} Removed {} source frames.",
                created("Radial extrusion", &s),
                s.frames_removed.len()
            )
        })
        .map_err(|e| e.to_string())
}

fn apply_reverse(model: &mut SapModel, keep: ReverseKeep) -> Result<String, String> {
    guard(model)?;
    model
        .reverse_area_local_3(keep)
        .map(|n| format!("Reversed local 3 of {n} areas."))
        .map_err(|e| e.to_string())
}

fn apply_curved(model: &mut SapModel, c: &CurvedInput) -> Result<String, String> {
    guard(model)?;
    model
        .curved_frame(c.start, c.end, c.third, c.kind, c.segments, &c.section)
        .map(|s| created("Curved frame", &s))
        .map_err(|e| e.to_string())
}

fn apply_select_frames(model: &mut SapModel, s: &SelectInput) -> Result<String, String> {
    guard(model)?;
    model
        .select_frames_by_section(&s.frame_section, s.replace)
        .map(|n| format!("Selected {n} frames with section {}.", s.frame_section))
        .map_err(|e| e.to_string())
}

fn apply_select_areas(model: &mut SapModel, s: &SelectInput) -> Result<String, String> {
    guard(model)?;
    model
        .select_areas_by_section(&s.area_section, s.replace)
        .map(|n| format!("Selected {n} areas with section {}.", s.area_section))
        .map_err(|e| e.to_string())
}

fn apply_diaphragm(model: &mut SapModel, s: &SelectInput) -> Result<String, String> {
    guard(model)?;
    model
        .assign_diaphragm(s.diaphragm.trim(), s.per_level)
        .map(|names| {
            format!(
                "Assigned the selected joints to diaphragms {}.",
                names.join(", ")
            )
        })
        .map_err(|e| e.to_string())
}

fn apply_remove_diaphragm(model: &mut SapModel) -> Result<String, String> {
    guard(model)?;
    model
        .remove_diaphragm_assignments()
        .map(|d| {
            format!(
                "Removed diaphragm assignments; deleted {} unused constraints.",
                d.len()
            )
        })
        .map_err(|e| e.to_string())
}

// --- window ----------------------------------------------------------------

fn triplet(ui: &mut egui::Ui, labels: &[&str], values: &mut [f64]) {
    ui.horizontal(|ui| {
        for (label, value) in labels.iter().zip(values) {
            ui.label(*label);
            ui.add(egui::DragValue::new(value).speed(0.1));
        }
    });
}

fn range(ui: &mut egui::Ui, label: &str, v: &mut [usize; 2]) {
    ui.horizontal(|ui| {
        ui.label(label);
        ui.add(egui::DragValue::new(&mut v[0]).prefix("from "));
        ui.add(egui::DragValue::new(&mut v[1]).prefix("to "));
    });
}

fn section_combo(ui: &mut egui::Ui, id: &str, label: &str, current: &mut String, names: &[String]) {
    if !names.is_empty() && !names.contains(current) {
        current.clone_from(&names[0]);
    }
    ui.horizontal(|ui| {
        ui.label(label);
        egui::ComboBox::from_id_salt(id)
            .selected_text(current.as_str())
            .show_ui(ui, |ui| {
                for n in names {
                    ui.selectable_value(current, n.clone(), n);
                }
            });
    });
}

/// Draws the window when it is open. Returns the outcome of a command the
/// user ran in this frame: a status message, or why it was refused.
pub(super) fn render(
    ctx: &egui::Context,
    state: &mut State,
    model: &mut SapModel,
) -> Option<Result<String, String>> {
    if !state.open {
        return None;
    }
    let frame_names: Vec<String> = model.frame_sections.keys().cloned().collect();
    let area_names: Vec<String> = model.area_sections.keys().cloned().collect();
    let units = model.present_units.s2k_label();
    let locked = model.is_locked;
    let mut outcome = None;
    let mut open = state.open;
    egui::Window::new("Grid, Quick Draw, Radial and Selection")
        .open(&mut open)
        .default_width(560.0)
        .vscroll(true)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                for (i, t) in [
                    "Grid",
                    "Quick Draw",
                    "Radial / Extrude",
                    "Curved Frame",
                    "Select / Diaphragm",
                ]
                .into_iter()
                .enumerate()
                {
                    ui.selectable_value(&mut state.tab, i, t);
                }
            });
            ui.label(format!(
                "Lengths are in the model length unit ({units}); angles are degrees, counter-clockwise seen from above."
            ));
            if locked {
                ui.colored_label(
                    egui::Color32::YELLOW,
                    "The model is locked: commands are refused. Use Edit > Unlock Model.",
                );
            }
            ui.separator();
            match state.tab {
                0 => {
                    ui.label("Regular Cartesian grid (replaces the GLOBAL grid lines).");
                    ui.horizontal(|ui| {
                        for (l, v) in ["Lines in X", "Y", "Z"].iter().zip(&mut state.grid_counts) {
                            ui.label(*l);
                            ui.add(egui::DragValue::new(v).range(1..=1000));
                        }
                    });
                    triplet(ui, &["Spacing X", "Y", "Z"], &mut state.grid_spacing);
                    triplet(ui, &["First line X", "Y", "Z"], &mut state.grid_origin);
                    if ui.button("Create grid lines").clicked() {
                        outcome = Some(apply_grid(
                            model,
                            state.grid_counts,
                            state.grid_spacing,
                            state.grid_origin,
                        ));
                    }
                    ui.separator();
                    ui.label("Edit the grid lines of one direction (the other directions stay).");
                    let g = &mut state.grid_edit;
                    ui.horizontal(|ui| {
                        for axis in GridAxis::ALL {
                            ui.selectable_value(&mut g.axis, axis, axis.label());
                        }
                        ui.separator();
                        ui.selectable_value(&mut g.mode, GridEditMode::Lines, "Ids and coordinates");
                        ui.selectable_value(&mut g.mode, GridEditMode::Spacings, "Spacings");
                    });
                    match g.mode {
                        GridEditMode::Lines => {
                            ui.label("One line per grid line: id, then coordinate (e.g. 'A 0'). An empty list removes the direction.");
                        }
                        GridEditMode::Spacings => {
                            ui.horizontal(|ui| {
                                ui.label("First line at");
                                ui.add(egui::DragValue::new(&mut g.origin).speed(0.1));
                            });
                            ui.label("Spacings separated by commas or blanks (e.g. '4, 6, 4'); ids are made up (A.., 1.., Z1..).");
                        }
                    }
                    ui.add(egui::TextEdit::multiline(&mut g.text).desired_rows(4).desired_width(f32::INFINITY));
                    ui.horizontal(|ui| {
                        if ui.button("Read current lines").clicked() {
                            g.mode = GridEditMode::Lines;
                            g.text = grid_lines_text(model, g.axis);
                        }
                        if ui.button("Apply grid lines").clicked() {
                            outcome = Some(apply_grid_lines(model, g));
                        }
                    });
                }
                1 => {
                    let d = &mut state.draw;
                    ui.label("Objects on GLOBAL grid intersections. Ranges are grid line indices, 0 = first line.");
                    ui.horizontal(|ui| {
                        for k in DrawKind::ALL {
                            ui.selectable_value(&mut d.kind, k, k.label());
                        }
                    });
                    range(ui, "X lines", &mut d.x);
                    range(ui, "Y lines", &mut d.y);
                    range(ui, "Z lines (levels)", &mut d.z);
                    if d.kind == DrawKind::Areas {
                        section_combo(ui, "build_draw_asec", "Area section", &mut d.area_section, &area_names);
                    } else {
                        section_combo(ui, "build_draw_fsec", "Frame section", &mut d.frame_section, &frame_names);
                    }
                    match d.kind {
                        DrawKind::Beams => {
                            ui.checkbox(&mut d.along_x, "Beams along X");
                            ui.checkbox(&mut d.along_y, "Beams along Y");
                        }
                        DrawKind::Braces => {
                            ui.horizontal(|ui| {
                                ui.selectable_value(&mut d.plane, VerticalPlane::XZ, "XZ planes");
                                ui.selectable_value(&mut d.plane, VerticalPlane::YZ, "YZ planes");
                            });
                            ui.horizontal(|ui| {
                                ui.selectable_value(&mut d.brace, BraceKind::X, "X");
                                ui.selectable_value(&mut d.brace, BraceKind::V, "V");
                                ui.selectable_value(&mut d.brace, BraceKind::InvertedV, "Inverted V");
                            });
                            ui.label("Pinned (M2, M3 released). V and inverted V need the beam at the apex level.");
                        }
                        DrawKind::SecondaryBeams => {
                            ui.horizontal(|ui| {
                                ui.selectable_value(&mut d.span, SpanDirection::X, "Span X");
                                ui.selectable_value(&mut d.span, SpanDirection::Y, "Span Y");
                                ui.label("Beams per bay");
                                ui.add(egui::DragValue::new(&mut d.count).range(1..=1000));
                            });
                            ui.label("Pinned. The supporting girders must exist; they are meshed at the new joints.");
                        }
                        _ => {}
                    }
                    if ui.button("Draw").clicked() {
                        outcome = Some(apply_quick_draw(model, &state.draw));
                    }
                }
                2 => {
                    let r = &mut state.radial;
                    ui.label("Rotation about a vertical axis through (X, Y).");
                    triplet(ui, &["Axis X", "Axis Y"], &mut r.center);
                    ui.horizontal(|ui| {
                        ui.label("Angle step (deg)");
                        ui.add(egui::DragValue::new(&mut r.angle_deg).speed(0.5));
                    });
                    ui.separator();
                    ui.label("Replicate the selected objects");
                    ui.horizontal(|ui| {
                        ui.label("Copies");
                        ui.add(egui::DragValue::new(&mut state.radial.copies).range(1..=10_000));
                        if ui.button("Replicate radially").clicked() {
                            outcome = Some(apply_replicate_radial(model, &state.radial));
                        }
                    });
                    let r = &mut state.radial;
                    ui.separator();
                    ui.label("Extrude the selected frames to areas");
                    ui.horizontal(|ui| {
                        ui.label("Steps");
                        ui.add(egui::DragValue::new(&mut r.steps).range(1..=10_000));
                        ui.label("Total rise");
                        ui.add(egui::DragValue::new(&mut r.rise).speed(0.1));
                    });
                    section_combo(ui, "build_extrude_asec", "Area section", &mut r.area_section, &area_names);
                    ui.checkbox(&mut r.delete_source, "Delete source frames");
                    if ui.button("Extrude to areas").clicked() {
                        outcome = Some(apply_extrude_radial(model, &state.radial));
                    }
                    ui.separator();
                    ui.label("Reverse local 3 of the selected areas");
                    ui.horizontal(|ui| {
                        if ui.button("Keep assignments in same global orientation").clicked() {
                            outcome = Some(apply_reverse(model, ReverseKeep::SameGlobalLoads));
                        }
                        if ui.button("Keep assignments in same local orientation").clicked() {
                            outcome = Some(apply_reverse(model, ReverseKeep::SameLocalLoads));
                        }
                    });
                }
                3 => {
                    let c = &mut state.curved;
                    ui.horizontal(|ui| {
                        ui.selectable_value(&mut c.kind, CurveKind::Circular, "Circular");
                        ui.selectable_value(&mut c.kind, CurveKind::Parabolic, "Parabolic");
                    });
                    triplet(ui, &["Start X", "Y", "Z"], &mut c.start);
                    triplet(ui, &["End X", "Y", "Z"], &mut c.end);
                    triplet(ui, &["Third point X", "Y", "Z"], &mut c.third);
                    ui.horizontal(|ui| {
                        ui.label("Segments");
                        ui.add(egui::DragValue::new(&mut c.segments).range(2..=10_000));
                    });
                    section_combo(ui, "build_curved_fsec", "Frame section", &mut c.section, &frame_names);
                    ui.label("Circular: equal angles. Parabolic: equal steps along the chord; the third point must lie between the ends.");
                    if ui.button("Draw curved frame").clicked() {
                        outcome = Some(apply_curved(model, &state.curved));
                    }
                }
                _ => {
                    let s = &mut state.select;
                    ui.label("Select by section property");
                    ui.checkbox(&mut s.replace, "Replace the current selection");
                    section_combo(ui, "build_sel_fsec", "Frame section", &mut s.frame_section, &frame_names);
                    if ui.button("Select frames").clicked() {
                        outcome = Some(apply_select_frames(model, &state.select));
                    }
                    let s = &mut state.select;
                    section_combo(ui, "build_sel_asec", "Area section", &mut s.area_section, &area_names);
                    if ui.button("Select areas").clicked() {
                        outcome = Some(apply_select_areas(model, &state.select));
                    }
                    let s = &mut state.select;
                    ui.separator();
                    ui.label("Diaphragm constraint (axis Z) for the selected joints");
                    ui.horizontal(|ui| {
                        ui.label("Name");
                        ui.text_edit_singleline(&mut s.diaphragm);
                    });
                    ui.checkbox(&mut s.per_level, "One constraint per Z level (name_1, name_2, ...)");
                    ui.horizontal(|ui| {
                        if ui.button("Assign diaphragm").clicked() {
                            outcome = Some(apply_diaphragm(model, &state.select));
                        }
                        if ui.button("Remove diaphragm assignments").clicked() {
                            outcome = Some(apply_remove_diaphragm(model));
                        }
                    });
                }
            }
        });
    state.open = open;
    outcome
}

#[cfg(test)]
mod tests {
    use super::*;
    use egui::{Pos2, Rect, Vec2};

    fn locked_model_is_unchanged(f: impl Fn(&mut SapModel) -> Result<String, String>) {
        let mut m = SapModel::default();
        m.quick_grid([3, 3, 2], [6.0, 5.0, 4.0], [0.0; 3]).unwrap();
        m.is_locked = true;
        let before = m.clone();
        let r = f(&mut m);
        assert!(r.as_ref().is_err_and(|e| e.contains("locked")), "{r:?}");
        assert_eq!(m, before);
    }

    #[test]
    fn every_command_refuses_a_locked_model() {
        let d = QuickDrawInput {
            frame_section: "FSEC1".into(),
            area_section: "ASEC1".into(),
            ..Default::default()
        };
        locked_model_is_unchanged(|m| apply_grid(m, [2, 2, 2], [1.0; 3], [0.0; 3]));
        for kind in DrawKind::ALL {
            let d = QuickDrawInput { kind, ..d.clone() };
            locked_model_is_unchanged(move |m| apply_quick_draw(m, &d));
        }
        let r = RadialInput {
            area_section: "ASEC1".into(),
            ..Default::default()
        };
        locked_model_is_unchanged(|m| apply_replicate_radial(m, &r));
        locked_model_is_unchanged(|m| apply_extrude_radial(m, &r));
        locked_model_is_unchanged(|m| apply_reverse(m, ReverseKeep::SameGlobalLoads));
        let c = CurvedInput {
            section: "FSEC1".into(),
            ..Default::default()
        };
        locked_model_is_unchanged(|m| apply_curved(m, &c));
        let s = SelectInput {
            frame_section: "FSEC1".into(),
            area_section: "ASEC1".into(),
            ..Default::default()
        };
        locked_model_is_unchanged(|m| apply_select_frames(m, &s));
        locked_model_is_unchanged(|m| apply_select_areas(m, &s));
        locked_model_is_unchanged(|m| apply_diaphragm(m, &s));
        locked_model_is_unchanged(apply_remove_diaphragm);
    }

    #[test]
    fn commands_report_counts_and_refusals() {
        let mut m = SapModel::default();
        assert_eq!(
            apply_grid(&mut m, [4, 3, 2], [6.0, 5.0, 4.0], [0.0; 3]).unwrap(),
            "Defined 4 x 3 x 2 grid lines."
        );
        let d = QuickDrawInput {
            kind: DrawKind::Columns,
            x: [0, 3],
            y: [0, 2],
            z: [0, 1],
            frame_section: "FSEC1".into(),
            ..Default::default()
        };
        let (nx, ny, nz) = (4usize, 3usize, 2usize);
        let (joints, columns) = (nx * ny * nz, (nz - 1) * nx * ny);
        let msg = apply_quick_draw(&mut m, &d).unwrap();
        assert_eq!(
            msg,
            format!("Columns: created {joints} joints, {columns} frames, 0 areas.")
        );
        assert_eq!(m.frames.len(), columns);
        let before = m.clone();
        let again = apply_quick_draw(&mut m, &d);
        assert!(again.is_err());
        assert_eq!(m, before);
        let bad = QuickDrawInput {
            frame_section: "NOPE".into(),
            ..d
        };
        assert!(apply_quick_draw(&mut m, &bad).is_err());
        assert_eq!(m, before);
        // select + diaphragm through the command functions
        let s = SelectInput {
            frame_section: "FSEC1".into(),
            ..Default::default()
        };
        assert_eq!(
            apply_select_frames(&mut m, &s).unwrap(),
            format!("Selected {columns} frames with section FSEC1.")
        );
        assert!(apply_diaphragm(&mut m, &s).is_err(), "no joints selected");
        m.points.values_mut().for_each(|p| p.selected = true);
        let msg = apply_diaphragm(&mut m, &s).unwrap();
        assert!(msg.contains("DIAPH_1, DIAPH_2"), "{msg}");
        assert_eq!(m.constraints.len(), 2);
    }

    fn input(time: f64) -> egui::RawInput {
        egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1400., 1200.))),
            time: Some(time),
            ..Default::default()
        }
    }

    fn button_center(out: &egui::FullOutput, label: &str) -> Option<Pos2> {
        out.shapes.iter().find_map(|shape| {
            if let egui::epaint::Shape::Text(text) = &shape.shape {
                (text.galley.job.text == label).then(|| text.pos + text.galley.size() * 0.5)
            } else {
                None
            }
        })
    }

    #[test]
    fn window_renders_every_tab_and_a_click_creates_the_grid() {
        let mut state = State {
            open: true,
            ..Default::default()
        };
        let mut model = SapModel::default();
        let ctx = egui::Context::default();
        let mut time = 0.0;
        for tab in 0..5 {
            state.tab = tab;
            for _ in 0..2 {
                time += 1.0;
                let out = ctx.run(input(time), |ctx| {
                    assert!(render(ctx, &mut state, &mut model).is_none());
                });
                assert!(!out.shapes.is_empty(), "tab {tab} draws");
            }
        }
        // Tab 0: click "Create grid lines".
        state.tab = 0;
        state.grid_counts = [3, 2, 2];
        time += 1.0;
        let _ = ctx.run(input(time), |ctx| {
            render(ctx, &mut state, &mut model);
        });
        time += 1.0;
        let out = ctx.run(input(time), |ctx| {
            render(ctx, &mut state, &mut model);
        });
        let pos = button_center(&out, "Create grid lines").expect("button visible");
        let mut down = input(time + 1.0);
        down.events = vec![
            egui::Event::PointerMoved(pos),
            egui::Event::PointerButton {
                pos,
                button: egui::PointerButton::Primary,
                pressed: true,
                modifiers: Default::default(),
            },
        ];
        let _ = ctx.run(down, |ctx| {
            render(ctx, &mut state, &mut model);
        });
        let mut up = input(time + 1.1);
        up.events = vec![egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed: false,
            modifiers: Default::default(),
        }];
        let mut outcome = None;
        let _ = ctx.run(up, |ctx| {
            outcome = render(ctx, &mut state, &mut model);
        });
        assert_eq!(
            outcome,
            Some(Ok("Defined 3 x 2 x 2 grid lines.".to_string()))
        );
        assert_eq!(model.coord_systems["GLOBAL"].grids.len(), 7);
        // A closed window draws nothing and runs nothing.
        state.open = false;
        let out = ctx.run(input(time + 3.0), |ctx| {
            assert!(render(ctx, &mut state, &mut model).is_none());
        });
        assert!(out.shapes.is_empty());
    }

    #[test]
    fn lengths_parse_and_accumulate_into_grid_coordinates() {
        assert_eq!(parse_lengths("4, 6, 4").unwrap(), vec![4.0, 6.0, 4.0]);
        assert_eq!(parse_lengths("4 6\t4,\n3.5").unwrap(), vec![4.0, 6.0, 4.0, 3.5]);
        assert_eq!(cumulative(0.0, &[4.0, 6.0, 4.0]), vec![0.0, 4.0, 10.0, 14.0]);
        assert_eq!(cumulative(-360.0, &[240.0, 240.0, 240.0]), vec![-360.0, -120.0, 120.0, 360.0]);
        assert_eq!(cumulative(2.5, &[]), vec![2.5]);
        for (text, what) in [
            ("", "at least one"),
            (" , ", "at least one"),
            ("4, x", "'x' is not a number"),
            ("4, 0", "'0' is not a positive"),
            ("4, -2", "'-2' is not a positive"),
            ("inf", "'inf' is not a positive"),
            ("NaN", "'NaN' is not a positive"),
        ] {
            let error = parse_lengths(text).unwrap_err();
            assert!(error.contains(what), "{text:?}: {error}");
        }
    }

    #[test]
    fn grid_line_editor_command_sets_typed_ids_and_spacings() {
        let mut m = SapModel::default();
        m.quick_grid([3, 2, 2], [6.0, 5.0, 4.0], [0.0; 3]).unwrap();
        let y_before = m.coord_systems["GLOBAL"].grids.iter().filter(|g| g.axis == "Y").cloned().collect::<Vec<_>>();
        let typed = GridEditInput {
            text: "A 0\nGate, 2.5\n\n  B  10\n".into(),
            ..Default::default()
        };
        assert_eq!(apply_grid_lines(&mut m, &typed).unwrap(), "Defined 3 X grid lines.");
        let x: Vec<_> = m.coord_systems["GLOBAL"].grids.iter().filter(|g| g.axis == "X").map(|g| (g.id.clone(), g.coordinate)).collect();
        assert_eq!(x, vec![("A".to_string(), 0.0), ("Gate".to_string(), 2.5), ("B".to_string(), 10.0)]);
        let y_after: Vec<_> = m.coord_systems["GLOBAL"].grids.iter().filter(|g| g.axis == "Y").cloned().collect();
        assert_eq!(y_after, y_before);
        // The editor text of the current lines is what was typed (ids and numbers).
        assert_eq!(grid_lines_text(&m, GridAxis::X), "A 0\nGate 2.5\nB 10\n");
        // Spacings: Y lines at 1, 5, 11, 12 with made-up ids.
        let spacings = GridEditInput {
            axis: GridAxis::Y,
            mode: GridEditMode::Spacings,
            text: "4 6, 1".into(),
            origin: 1.0,
        };
        assert_eq!(apply_grid_lines(&mut m, &spacings).unwrap(), "Defined 4 Y grid lines.");
        let y: Vec<_> = m.coord_systems["GLOBAL"].grids.iter().filter(|g| g.axis == "Y").map(|g| (g.id.clone(), g.coordinate)).collect();
        assert_eq!(y, vec![("1".to_string(), 1.0), ("2".to_string(), 5.0), ("3".to_string(), 11.0), ("4".to_string(), 12.0)]);
        // X ids made up past Z: AA follows Z.
        assert_eq!(made_up_id(GridAxis::X, 25), "Z");
        assert_eq!(made_up_id(GridAxis::X, 26), "AA");
        assert_eq!(made_up_id(GridAxis::Z, 0), "Z1");
        // An empty list removes the direction.
        let none = GridEditInput { axis: GridAxis::Z, ..Default::default() };
        assert_eq!(apply_grid_lines(&mut m, &none).unwrap(), "Removed the Z grid lines.");
        assert!(m.coord_systems["GLOBAL"].grids.iter().all(|g| g.axis != "Z"));
    }

    #[test]
    fn grid_line_editor_refusals_leave_the_model_unchanged() {
        let mut m = SapModel::default();
        m.quick_grid([3, 2, 2], [6.0, 5.0, 4.0], [0.0; 3]).unwrap();
        let before = m.clone();
        for (text, mode, what) in [
            ("A 0\nA 5", GridEditMode::Lines, "twice"),
            ("A 0\nB 0", GridEditMode::Lines, "same coordinate"),
            ("A 0\nB", GridEditMode::Lines, "line 2: expected an id and a coordinate"),
            ("A 0 7", GridEditMode::Lines, "line 1: expected"),
            ("A zero", GridEditMode::Lines, "line 1: 'zero' is not a number"),
            ("A NaN", GridEditMode::Lines, "nonfinite"),
            ("4, 0", GridEditMode::Spacings, "not a positive"),
            ("", GridEditMode::Spacings, "at least one"),
        ] {
            let input = GridEditInput { text: text.into(), mode, ..Default::default() };
            let error = apply_grid_lines(&mut m, &input).unwrap_err();
            assert!(error.contains(what), "{text:?}: {error}");
            assert_eq!(m, before, "{text:?}");
        }
        let infinite = GridEditInput { mode: GridEditMode::Spacings, text: "1".into(), origin: f64::INFINITY, ..Default::default() };
        assert!(apply_grid_lines(&mut m, &infinite).is_err());
        assert_eq!(m, before);
        locked_model_is_unchanged(|m| {
            apply_grid_lines(m, &GridEditInput { text: "A 0".into(), ..Default::default() })
        });
    }

    #[test]
    fn edit_grid_lines_menu_entry_opens_the_grid_tab() {
        let mut state = State::default();
        state.tab = 3;
        state.show_tab(TAB_GRID);
        assert!(state.open);
        assert_eq!(state.tab, TAB_GRID);
    }

    #[test]
    fn grid_tab_renders_the_editor_and_apply_defines_the_typed_lines() {
        let mut state = State::default();
        state.show_tab(TAB_GRID);
        state.grid_edit.text = "A 0\nB 3\nC 9\n".into();
        let mut model = SapModel::default();
        let ctx = egui::Context::default();
        let mut time = 1.0;
        for _ in 0..2 {
            time += 1.0;
            let _ = ctx.run(input(time), |ctx| {
                render(ctx, &mut state, &mut model);
            });
        }
        let out = ctx.run(input(time + 1.0), |ctx| {
            render(ctx, &mut state, &mut model);
        });
        for label in ["Apply grid lines", "Read current lines", "Create grid lines", "Ids and coordinates", "Spacings"] {
            assert!(button_center(&out, label).is_some(), "{label} is drawn");
        }
        let pos = button_center(&out, "Apply grid lines").unwrap();
        let mut down = input(time + 2.0);
        down.events = vec![
            egui::Event::PointerMoved(pos),
            egui::Event::PointerButton { pos, button: egui::PointerButton::Primary, pressed: true, modifiers: Default::default() },
        ];
        let _ = ctx.run(down, |ctx| {
            render(ctx, &mut state, &mut model);
        });
        let mut up = input(time + 2.1);
        up.events = vec![egui::Event::PointerButton { pos, button: egui::PointerButton::Primary, pressed: false, modifiers: Default::default() }];
        let mut outcome = None;
        let _ = ctx.run(up, |ctx| {
            outcome = render(ctx, &mut state, &mut model);
        });
        assert_eq!(outcome, Some(Ok("Defined 3 X grid lines.".to_string())));
        let x: Vec<_> = model.coord_systems["GLOBAL"].grids.iter().map(|g| (g.id.clone(), g.coordinate)).collect();
        assert_eq!(x, vec![("A".to_string(), 0.0), ("B".to_string(), 3.0), ("C".to_string(), 9.0)]);
    }
}
