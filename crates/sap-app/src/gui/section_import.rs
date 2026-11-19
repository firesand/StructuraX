//! Define > Import Sections from .PRO (multiple): choose several shapes of a section
//! file at once (drag, Shift and Ctrl selection as in the vendor form), then
//! import them in one atomic command.

use super::file_browser;
use eframe::egui;
use sap_model::{FramePropType, MatType, SapModel};
use sap_section::ProDatabase;
use std::collections::BTreeSet;

/// The shape types of a `.pro` file, as the filter offers them.
const SHAPES: [(&str, FramePropType); 11] = [
    ("I / Wide Flange", FramePropType::I),
    ("Channel", FramePropType::Channel),
    ("Tee", FramePropType::T),
    ("Angle", FramePropType::Angle),
    ("Double Angle", FramePropType::DblAngle),
    ("Double Channel", FramePropType::DbChannel),
    ("Box / Tube", FramePropType::Box),
    ("Pipe", FramePropType::Pipe),
    ("Rectangular", FramePropType::Rectangular),
    ("Circular", FramePropType::Circle),
    ("General", FramePropType::General),
];

#[derive(Debug, Clone, Default)]
pub(super) struct State {
    pub open: bool,
    /// The `.pro` file name or path as typed.
    pub path: String,
    /// The loaded database, if any.
    pub db: Option<ProDatabase>,
    /// Index into `SHAPES`, or all shapes.
    pub shape: Option<usize>,
    pub name_filter: String,
    /// Selected sections, as indices into the database.
    pub selected: BTreeSet<usize>,
    /// The anchor of a Shift selection, as an index into the database.
    pub anchor: Option<usize>,
    /// Row (position in the shown list) where a drag began.
    drag_from: Option<usize>,
    pub material: String,
    pub replace: bool,
    pub list_name: String,
    /// Names imported by the last successful import, in database order.
    pub imported: Vec<String>,
}

/// Selection after a click on row `clicked` of a list: a plain click selects
/// only that row; Shift selects the range from the anchor (added to the
/// selection when Ctrl is held too); Ctrl toggles the row. Returns the new
/// selection and the new anchor.
pub(super) fn click_selection(
    sel: &BTreeSet<usize>,
    anchor: Option<usize>,
    clicked: usize,
    shift: bool,
    ctrl: bool,
) -> (BTreeSet<usize>, Option<usize>) {
    if shift {
        let from = anchor.unwrap_or(clicked);
        let range = drag_selection(from, clicked);
        let mut out = if ctrl { sel.clone() } else { BTreeSet::new() };
        out.extend(range);
        (out, Some(from))
    } else if ctrl {
        let mut out = sel.clone();
        if !out.remove(&clicked) {
            out.insert(clicked);
        }
        (out, Some(clicked))
    } else {
        (BTreeSet::from([clicked]), Some(clicked))
    }
}

/// A drag from row `from` to row `to` selects every row between them.
pub(super) fn drag_selection(from: usize, to: usize) -> BTreeSet<usize> {
    (from.min(to)..=from.max(to)).collect()
}

/// Indices of the sections of `db` that pass the filters, in database order:
/// the shape type (all when `None`) and a case-insensitive name substring.
pub(super) fn matching_rows(
    db: &ProDatabase,
    shape: Option<FramePropType>,
    name_filter: &str,
) -> Vec<usize> {
    let text = name_filter.trim().to_uppercase();
    db.sections
        .iter()
        .enumerate()
        .filter(|(_, s)| shape.is_none_or(|t| s.prop_type == t))
        .filter(|(_, s)| text.is_empty() || s.name.to_uppercase().contains(&text))
        .map(|(i, _)| i)
        .collect()
}

/// Imports the shapes `names` of `db` into the model, scaled to the model's
/// units and given `material`. Atomic: either every shape is imported or the
/// model is unchanged. Names match without regard to case and are stored
/// under the name in the file. Refused: an empty selection, a locked model, a
/// missing material, an unknown shape, an existing section name without
/// `replace`, and replacing an Auto Select list. Replacing a section deletes
/// the analysis and design results, as any change to a section does.
pub(super) fn import_sections(
    model: &mut SapModel,
    db: &ProDatabase,
    names: &[String],
    material: &str,
    replace: bool,
) -> Result<String, String> {
    if names.is_empty() {
        return Err("no sections are selected".into());
    }
    if model.is_locked {
        return Err("the model is locked; unlock it before importing sections".into());
    }
    if !model.materials.contains_key(material) {
        return Err(format!(
            "the material '{material}' is not defined in the model"
        ));
    }
    let mut sections = Vec::new();
    for name in names {
        let section = db
            .get_section_scaled(name, model.present_units, material)
            .map_err(|e| {
                format!(
                    "section '{name}' cannot be imported from {}: {e}",
                    db.file_name
                )
            })?;
        if sections
            .iter()
            .any(|s: &sap_model::FrameSection| s.name == section.name)
        {
            continue;
        }
        match model.frame_sections.get(&section.name) {
            Some(existing) if existing.prop_type == FramePropType::Auto => {
                return Err(format!(
                    "'{}' is an Auto Select list and cannot be replaced by a shape",
                    section.name
                ));
            }
            Some(_) if !replace => {
                return Err(format!(
                    "the section '{}' already exists; enable replace to overwrite it",
                    section.name
                ));
            }
            _ => {}
        }
        sections.push(section);
    }
    let replaced = sections
        .iter()
        .filter(|s| model.frame_sections.contains_key(&s.name))
        .count();
    if replaced > 0 {
        // D124: one way to clear results (reasons included).
        model.invalidate_results();
    }
    let listed: Vec<&str> = sections.iter().take(6).map(|s| s.name.as_str()).collect();
    let mut message = format!(
        "Imported {} section(s) from {} with material {material}: {}",
        sections.len(),
        db.file_name,
        listed.join(", ")
    );
    if sections.len() > listed.len() {
        message.push_str(", ...");
    }
    if replaced > 0 {
        message.push_str(&format!(" ({replaced} replaced)"));
    }
    for section in sections {
        model.frame_sections.insert(section.name.clone(), section);
    }
    Ok(message)
}

fn load_database(path: &str) -> Result<ProDatabase, String> {
    let name = path.trim();
    if name.is_empty() {
        return Err("type the name or path of a .pro file".into());
    }
    let resolved = super::resolve_pro_database_path(name)
        .ok_or_else(|| format!("cannot find the .pro database '{name}'"))?;
    ProDatabase::from_file(&resolved).map_err(|e| format!("failed to read '{name}': {e}"))
}

fn default_material(model: &SapModel) -> String {
    if model.materials.contains_key("A992Fy50") {
        return "A992Fy50".into();
    }
    model
        .materials
        .values()
        .find(|m| m.mat_type == MatType::Steel)
        .or_else(|| model.materials.values().next())
        .map(|m| m.name.clone())
        .unwrap_or_default()
}

/// Draws the window when it is open. Returns the outcome of a command the
/// user ran in this frame: a status message, or why it was refused.
pub(super) fn render(
    ctx: &egui::Context,
    state: &mut State,
    model: &mut SapModel,
    files: &mut file_browser::State,
) -> Option<Result<String, String>> {
    if !state.open {
        return None;
    }
    let mut outcome = None;
    if let Some(path) = file_browser::take_choice(files, file_browser::Purpose::SectionFile) {
        state.path = path.display().to_string();
        outcome = Some(load_into(state));
    }
    if state.material.is_empty() || !model.materials.contains_key(&state.material) {
        state.material = default_material(model);
    }
    let mut open = state.open;
    egui::Window::new("Import Sections from .PRO (multiple)")
        .open(&mut open)
        .default_width(520.0)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label("Database (.pro):");
                ui.text_edit_singleline(&mut state.path);
                if ui.button("Browse...").clicked() {
                    let start = super::resolve_pro_database_path(state.path.trim())
                        .unwrap_or_else(|| std::path::PathBuf::from(state.path.trim()));
                    file_browser::open_for(
                        files,
                        file_browser::Purpose::SectionFile,
                        &start,
                        &["pro"],
                        false,
                    );
                }
                if ui.button("Load").clicked() {
                    outcome = Some(load_into(state));
                }
            });
            let Some(db) = state.db.clone() else {
                ui.label("Load a .pro file to list its shapes.");
                return;
            };
            ui.label(format!(
                "{}: {} shapes, lengths in {}",
                db.file_name,
                db.sections.len(),
                db.length_unit.symbol()
            ));
            ui.horizontal(|ui| {
                ui.label("Shape type:");
                egui::ComboBox::from_id_salt("section_import_shape")
                    .selected_text(state.shape.map_or("All", |i| SHAPES[i].0))
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut state.shape, None, "All");
                        for (i, (label, _)) in SHAPES.iter().enumerate() {
                            ui.selectable_value(&mut state.shape, Some(i), *label);
                        }
                    });
                ui.label("Name contains:");
                ui.text_edit_singleline(&mut state.name_filter);
            });
            let visible = matching_rows(&db, state.shape.map(|i| SHAPES[i].1), &state.name_filter);
            // Only shown rows stay selected, so an import never takes hidden shapes.
            let shown: BTreeSet<usize> = visible.iter().copied().collect();
            state.selected.retain(|i| shown.contains(i));
            if state.anchor.is_some_and(|a| !shown.contains(&a)) {
                state.anchor = None;
            }
            ui.horizontal(|ui| {
                if ui.button("Select all shown").clicked() {
                    state.selected = shown.clone();
                }
                if ui.button("Clear selection").clicked() {
                    state.selected.clear();
                    state.anchor = None;
                }
                ui.label(format!(
                    "{} selected of {} shown",
                    state.selected.len(),
                    visible.len()
                ));
            });
            ui.label("Click, drag, Shift+click (range) or Ctrl+click (toggle) to choose shapes.");
            shape_list(ui, state, &db, &visible);
            ui.separator();
            ui.horizontal(|ui| {
                ui.label("Material:");
                egui::ComboBox::from_id_salt("section_import_material")
                    .selected_text(state.material.clone())
                    .show_ui(ui, |ui| {
                        for name in model.materials.keys() {
                            ui.selectable_value(&mut state.material, name.clone(), name);
                        }
                    });
                ui.checkbox(&mut state.replace, "Replace existing sections");
            });
            if ui.button("Import selected").clicked() {
                let names: Vec<String> = state
                    .selected
                    .iter()
                    .map(|&i| db.sections[i].name.clone())
                    .collect();
                let result = import_sections(model, &db, &names, &state.material, state.replace);
                if result.is_ok() {
                    state.imported = names;
                }
                outcome = Some(result);
            }
            ui.horizontal(|ui| {
                ui.label("Auto Select list:");
                ui.text_edit_singleline(&mut state.list_name);
                if ui
                    .add_enabled(
                        !state.imported.is_empty(),
                        egui::Button::new("Define Auto Select list from imported"),
                    )
                    .clicked()
                {
                    outcome = Some(
                        model
                            .define_auto_select_list(state.list_name.trim(), &state.imported)
                            .map(|()| {
                                format!(
                                    "Defined Auto Select list '{}' with {} sections.",
                                    state.list_name.trim(),
                                    state.imported.len()
                                )
                            }),
                    );
                }
            });
        });
    state.open = open;
    outcome
}

fn load_into(state: &mut State) -> Result<String, String> {
    let db = load_database(&state.path)?;
    let message = format!("Loaded {} shapes from {}", db.sections.len(), db.file_name);
    state.db = Some(db);
    state.selected.clear();
    state.anchor = None;
    state.imported.clear();
    Ok(message)
}

/// The scrolling shape list with click, Ctrl, Shift and drag selection.
fn shape_list(ui: &mut egui::Ui, state: &mut State, db: &ProDatabase, visible: &[usize]) {
    let row_height = ui.text_style_height(&egui::TextStyle::Body) + 4.0;
    let mut clicked: Option<usize> = None;
    let mut rects: Vec<(usize, egui::Rect)> = Vec::new();
    let mut drag_started: Option<usize> = None;
    egui::ScrollArea::vertical()
        .max_height(220.0)
        .auto_shrink([false, true])
        .show_rows(ui, row_height, visible.len(), |ui, range| {
            for position in range {
                let index = visible[position];
                let section = &db.sections[index];
                let (rect, response) = ui.allocate_exact_size(
                    egui::vec2(ui.available_width(), row_height),
                    egui::Sense::click_and_drag(),
                );
                if state.selected.contains(&index) {
                    ui.painter()
                        .rect_filled(rect, 2.0, ui.visuals().selection.bg_fill);
                }
                let color = ui.visuals().text_color();
                ui.painter().text(
                    rect.left_center() + egui::vec2(4.0, 0.0),
                    egui::Align2::LEFT_CENTER,
                    &section.name,
                    egui::TextStyle::Body.resolve(ui.style()),
                    color,
                );
                let kind = SHAPES
                    .iter()
                    .find(|(_, t)| *t == section.prop_type)
                    .map_or("Other", |(l, _)| *l);
                ui.painter().text(
                    rect.right_center() - egui::vec2(4.0, 0.0),
                    egui::Align2::RIGHT_CENTER,
                    kind,
                    egui::TextStyle::Small.resolve(ui.style()),
                    ui.visuals().weak_text_color(),
                );
                rects.push((position, rect));
                if response.clicked() {
                    clicked = Some(position);
                }
                if response.drag_started() {
                    drag_started = Some(position);
                }
            }
        });
    if drag_started.is_some() {
        state.drag_from = drag_started;
    }
    let to_positions = |state: &State| -> BTreeSet<usize> {
        visible
            .iter()
            .enumerate()
            .filter(|(_, i)| state.selected.contains(i))
            .map(|(p, _)| p)
            .collect()
    };
    if let Some(position) = clicked {
        let (shift, ctrl) =
            ui.input(|i| (i.modifiers.shift, i.modifiers.ctrl || i.modifiers.command));
        let anchor = state
            .anchor
            .and_then(|a| visible.iter().position(|&v| v == a));
        let (positions, anchor) =
            click_selection(&to_positions(state), anchor, position, shift, ctrl);
        state.selected = positions.iter().map(|&p| visible[p]).collect();
        state.anchor = anchor.map(|p| visible[p]);
        state.drag_from = None;
    } else if let Some(from) = state.drag_from {
        if ui.input(|i| i.pointer.any_released()) {
            let pointer = ui.input(|i| i.pointer.latest_pos());
            let to = pointer.and_then(|p| {
                rects
                    .iter()
                    .find(|(_, r)| r.y_range().contains(p.y))
                    .map(|(position, _)| *position)
            });
            if let Some(to) = to {
                state.selected = drag_selection(from, to)
                    .iter()
                    .map(|&p| visible[p])
                    .collect();
                state.anchor = Some(visible[from]);
            }
            state.drag_from = None;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use eframe::egui::{Modifiers, Pos2, Rect, Vec2};
    use sap_units::Units;
    use std::path::Path;

    /// A synthetic `.pro` file: header, then one record per entry.
    fn pro_bytes(unit_code: f32, records: &[(&str, &str, f32)]) -> Vec<u8> {
        let mut bytes = vec![b' '; 120 * (records.len() + 1)];
        for (offset, value) in [(40, 7.0_f32), (44, unit_code), (48, records.len() as f32)] {
            bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        }
        for (n, (name, shape, scale)) in records.iter().enumerate() {
            let base = 120 * (n + 1);
            bytes[base..base + name.len()].copy_from_slice(name.as_bytes());
            bytes[base + 36..base + 36 + shape.len()].copy_from_slice(shape.as_bytes());
            let mut values = [0.0_f32; 20];
            values[0] = 10.0 * scale; // depth
            values[1] = 5.0 * scale; // width
            values[2] = 0.5 * scale;
            values[3] = 0.25 * scale;
            values[5] = 12.0 * scale * scale; // area
            values[6] = 3.0 * scale.powi(4); // J
            values[7] = 200.0 * scale.powi(4); // I33
            values[8] = 50.0 * scale.powi(4); // I22
            values[9] = 4.0 * scale * scale;
            values[10] = 5.0 * scale * scale;
            values[11] = 40.0 * scale.powi(3);
            values[12] = 15.0 * scale.powi(3);
            for (k, v) in values.iter().enumerate() {
                let at = base + 40 + 4 * k;
                bytes[at..at + 4].copy_from_slice(&v.to_le_bytes());
            }
        }
        bytes
    }

    fn database() -> ProDatabase {
        let records = [
            ("IPE200", "W", 1.0),
            ("IPE220", "W", 1.1),
            ("IPE240", "W", 1.2),
            ("HEA200", "W", 1.3),
            ("UPN100", "C", 0.9),
            ("UPN120", "C", 0.95),
            ("TUBE50", "B", 0.5),
            ("PIPE60", "P", 0.6),
            ("ANG40", "L", 0.4),
        ];
        // Unit code 1 is inches; the models below are in kN, m.
        ProDatabase::from_bytes(&pro_bytes(1.0, &records), "synthetic.pro".into()).unwrap()
    }

    fn names(list: &[&str]) -> Vec<String> {
        list.iter().map(|s| s.to_string()).collect()
    }

    fn set(items: &[usize]) -> BTreeSet<usize> {
        items.iter().copied().collect()
    }

    #[test]
    fn click_rules_plain_shift_ctrl_and_drag() {
        let none = BTreeSet::new();
        // A plain click selects only the clicked row and anchors there.
        let (sel, anchor) = click_selection(&none, None, 3, false, false);
        assert_eq!((sel.clone(), anchor), (set(&[3]), Some(3)));
        let (sel, anchor) = click_selection(&set(&[1, 2, 8]), Some(2), 5, false, false);
        assert_eq!((sel, anchor), (set(&[5]), Some(5)));
        // Shift selects the range from the anchor, either way round.
        let (sel, anchor) = click_selection(&set(&[3]), Some(3), 6, true, false);
        assert_eq!((sel, anchor), (set(&[3, 4, 5, 6]), Some(3)));
        let (sel, anchor) = click_selection(&set(&[3, 4, 5, 6]), Some(3), 1, true, false);
        assert_eq!((sel, anchor), (set(&[1, 2, 3]), Some(3)));
        // Shift without an anchor selects the clicked row alone.
        assert_eq!(
            click_selection(&none, None, 4, true, false),
            (set(&[4]), Some(4))
        );
        // Ctrl toggles a row and moves the anchor to it.
        let (sel, anchor) = click_selection(&set(&[1, 2]), Some(1), 5, false, true);
        assert_eq!((sel.clone(), anchor), (set(&[1, 2, 5]), Some(5)));
        let (sel, anchor) = click_selection(&sel, anchor, 2, false, true);
        assert_eq!((sel, anchor), (set(&[1, 5]), Some(2)));
        // Ctrl+Shift adds the range to the selection.
        let (sel, anchor) = click_selection(&set(&[0, 9]), Some(3), 5, true, true);
        assert_eq!((sel, anchor), (set(&[0, 3, 4, 5, 9]), Some(3)));
        // A drag from row i to row j selects i..=j.
        assert_eq!(drag_selection(2, 5), set(&[2, 3, 4, 5]));
        assert_eq!(drag_selection(5, 2), set(&[2, 3, 4, 5]));
        assert_eq!(drag_selection(4, 4), set(&[4]));
    }

    #[test]
    fn filters_by_shape_type_and_name_without_regard_to_case() {
        let db = database();
        assert_eq!(matching_rows(&db, None, "").len(), 9);
        assert_eq!(matching_rows(&db, Some(FramePropType::I), ""), [0, 1, 2, 3]);
        assert_eq!(matching_rows(&db, Some(FramePropType::Channel), ""), [4, 5]);
        assert_eq!(matching_rows(&db, None, "ipe"), [0, 1, 2, 7]); // PIPE60 too
        assert_eq!(matching_rows(&db, Some(FramePropType::I), " 200"), [0, 3]);
        assert_eq!(matching_rows(&db, Some(FramePropType::Pipe), "ipe"), [7]);
        assert_eq!(matching_rows(&db, Some(FramePropType::Angle), ""), [8]);
        assert!(matching_rows(&db, Some(FramePropType::Box), "pipe").is_empty());
    }

    fn model_kn_m() -> SapModel {
        SapModel::new(Units::KnMC)
    }

    fn steel(model: &SapModel) -> String {
        model
            .materials
            .values()
            .find(|m| m.mat_type == MatType::Steel)
            .unwrap()
            .name
            .clone()
    }

    #[test]
    fn replacing_a_section_clears_the_reasons_a_result_is_unavailable_too() {
        // D124: the replace path cleared the results but kept `unavailable_results`.
        let db = database();
        let mut model = model_kn_m();
        let material = steel(&model);
        import_sections(&mut model, &db, &names(&["IPE200"]), &material, false).unwrap();
        model
            .unavailable_results
            .insert("COMBO1".into(), "stale reason".into());
        model
            .case_results
            .insert("stale".into(), Default::default());
        import_sections(&mut model, &db, &names(&["IPE200"]), &material, true).unwrap();
        assert!(model.unavailable_results.is_empty());
        assert!(model.case_results.is_empty());
        // Negative: an import that replaces nothing leaves the results alone.
        model
            .unavailable_results
            .insert("COMBO1".into(), "kept".into());
        import_sections(&mut model, &db, &names(&["IPE220"]), &material, false).unwrap();
        assert_eq!(model.unavailable_results.len(), 1);
    }

    #[test]
    fn n_imports_equal_n_single_scaled_reads_field_by_field() {
        let db = database();
        let mut model = model_kn_m();
        let material = steel(&model);
        let chosen = names(&["IPE220", "upn100", "TUBE50", "ANG40"]);
        let message = import_sections(&mut model, &db, &chosen, &material, false).unwrap();
        assert!(message.contains("4 section(s)"), "{message}");
        for name in &chosen {
            let single = db
                .get_section_scaled(name, model.present_units, &material)
                .unwrap();
            let imported = &model.frame_sections[&single.name];
            assert_eq!(imported, &single, "{name}");
            // Bitwise, field by field, for the scaled properties.
            for (a, b) in [
                (imported.area, single.area),
                (imported.i33, single.i33),
                (imported.i22, single.i22),
                (imported.torsional_j, single.torsional_j),
                (imported.z33, single.z33),
                (imported.t3, single.t3),
            ] {
                assert_eq!(a.to_bits(), b.to_bits(), "{name}");
            }
        }
        // 10 in = 0.254 m and I33 scales with the fourth power.
        let ipe = &model.frame_sections["UPN100"];
        assert!((ipe.t3 - 9.0 * 0.0254).abs() < 1e-12);
        // Case-insensitive names are stored under the file's name, once.
        assert!(!model.frame_sections.contains_key("upn100"));
        let mut again = model_kn_m();
        import_sections(
            &mut again,
            &db,
            &names(&["IPE200", "ipe200", "IPE200"]),
            &material,
            false,
        )
        .unwrap();
        assert_eq!(
            again
                .frame_sections
                .keys()
                .filter(|k| *k == "IPE200")
                .count(),
            1
        );
    }

    #[test]
    fn a_refused_import_leaves_the_model_unchanged() {
        let db = database();
        let mut model = model_kn_m();
        let material = steel(&model);
        import_sections(&mut model, &db, &names(&["IPE200"]), &material, false).unwrap();
        let before = model.clone();
        let check = |model: &mut SapModel, list: &[&str], material: &str, replace, expect: &str| {
            let snapshot = model.clone();
            let error = import_sections(model, &db, &names(list), material, replace).unwrap_err();
            assert!(error.contains(expect), "{error}");
            assert_eq!(*model, snapshot, "unchanged after: {error}");
        };
        // A name that is not in the file, after valid ones.
        let mut m = before.clone();
        check(
            &mut m,
            &["IPE220", "IPE999", "IPE240"],
            &material,
            false,
            "IPE999",
        );
        // An existing name without replace (the valid ones before it are not kept).
        check(&mut m, &["IPE220", "IPE200"], &material, false, "IPE200");
        // A missing material.
        check(&mut m, &["IPE220"], "NOSUCH", false, "NOSUCH");
        // An empty selection.
        check(&mut m, &[], &material, false, "no sections");
        // A locked model.
        let mut locked = before.clone();
        locked.is_locked = true;
        check(&mut locked, &["IPE220"], &material, false, "locked");
        // An Auto Select list cannot be replaced by a shape.
        let mut with_list = before.clone();
        with_list
            .define_auto_select_list("IPE220", &names(&["IPE200"]))
            .unwrap();
        check(&mut with_list, &["IPE220"], &material, true, "IPE220");
    }

    #[test]
    fn replace_overwrites_and_clears_results() {
        let db = database();
        let mut model = model_kn_m();
        let material = steel(&model);
        import_sections(&mut model, &db, &names(&["IPE200"]), &material, false).unwrap();
        model.frame_sections.get_mut("IPE200").unwrap().area = 99.0;
        model.case_results.insert("DEAD".into(), Default::default());
        let message = import_sections(
            &mut model,
            &db,
            &names(&["IPE200", "IPE220"]),
            &material,
            true,
        )
        .unwrap();
        assert!(message.contains("1 replaced"), "{message}");
        let single = db
            .get_section_scaled("IPE200", model.present_units, &material)
            .unwrap();
        assert_eq!(model.frame_sections["IPE200"], single);
        assert!(model.frame_sections.contains_key("IPE220"));
        assert!(model.case_results.is_empty());
        // A pure addition keeps results.
        model.case_results.insert("DEAD".into(), Default::default());
        import_sections(&mut model, &db, &names(&["IPE240"]), &material, false).unwrap();
        assert!(!model.case_results.is_empty());
    }

    #[test]
    fn imported_shapes_define_an_auto_select_list() {
        let db = database();
        let mut model = model_kn_m();
        let material = steel(&model);
        let all: Vec<String> = matching_rows(&db, Some(FramePropType::I), "IPE")
            .into_iter()
            .map(|i| db.sections[i].name.clone())
            .collect();
        import_sections(&mut model, &db, &all, &material, false).unwrap();
        model.define_auto_select_list("IPES", &all).unwrap();
        assert_eq!(model.frame_sections["IPES"].auto_select_list, all);
    }

    fn input(time: f64, modifiers: Modifiers) -> egui::RawInput {
        egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1200., 900.))),
            time: Some(time),
            modifiers,
            ..Default::default()
        }
    }

    fn text_center(out: &egui::FullOutput, label: &str) -> Option<Pos2> {
        out.shapes.iter().find_map(|shape| {
            if let egui::epaint::Shape::Text(text) = &shape.shape {
                (text.galley.job.text == label).then(|| text.pos + text.galley.size() * 0.5)
            } else {
                None
            }
        })
    }

    struct Harness {
        ctx: egui::Context,
        state: State,
        model: SapModel,
        files: file_browser::State,
        time: f64,
    }

    impl Harness {
        fn new() -> Self {
            let mut state = State {
                open: true,
                db: Some(database()),
                ..Default::default()
            };
            state.path = "synthetic.pro".into();
            let mut h = Self {
                ctx: egui::Context::default(),
                state,
                model: model_kn_m(),
                files: Default::default(),
                time: 0.0,
            };
            for _ in 0..3 {
                h.frame(Modifiers::NONE, vec![]);
            }
            h
        }

        fn frame_outcome(
            &mut self,
            modifiers: Modifiers,
            events: Vec<egui::Event>,
        ) -> (egui::FullOutput, Option<Result<String, String>>) {
            self.time += 0.3;
            let mut raw = input(self.time, modifiers);
            raw.events = events;
            let mut outcome = None;
            let out = self.ctx.run(raw, |ctx| {
                outcome = render(ctx, &mut self.state, &mut self.model, &mut self.files);
            });
            (out, outcome)
        }

        fn frame(&mut self, modifiers: Modifiers, events: Vec<egui::Event>) -> egui::FullOutput {
            self.frame_outcome(modifiers, events).0
        }

        fn button(pos: Pos2, pressed: bool, modifiers: Modifiers) -> Vec<egui::Event> {
            vec![
                egui::Event::PointerMoved(pos),
                egui::Event::PointerButton {
                    pos,
                    button: egui::PointerButton::Primary,
                    pressed,
                    modifiers,
                },
            ]
        }

        /// A real pointer click on the text `label`; returns the outcome of
        /// the release frame.
        fn click(&mut self, label: &str, modifiers: Modifiers) -> Option<Result<String, String>> {
            let out = self.frame(Modifiers::NONE, vec![]);
            let pos = text_center(&out, label).unwrap_or_else(|| panic!("no '{label}' drawn"));
            self.frame(modifiers, Self::button(pos, true, modifiers));
            let (_, outcome) = self.frame_outcome(modifiers, Self::button(pos, false, modifiers));
            self.frame(Modifiers::NONE, vec![]);
            outcome
        }

        fn selected_names(&self) -> Vec<String> {
            let db = self.state.db.as_ref().unwrap();
            self.state
                .selected
                .iter()
                .map(|&i| db.sections[i].name.clone())
                .collect()
        }
    }

    #[test]
    fn window_plain_ctrl_and_shift_clicks_select_rows_and_import_runs() {
        let mut h = Harness::new();
        h.click("IPE220", Modifiers::NONE);
        assert_eq!(h.selected_names(), ["IPE220"]);
        h.click("UPN100", Modifiers::CTRL);
        assert_eq!(h.selected_names(), ["IPE220", "UPN100"]);
        // Ctrl again on a selected row removes it.
        h.click("IPE220", Modifiers::CTRL);
        assert_eq!(h.selected_names(), ["UPN100"]);
        // Shift selects from the anchor (UPN100... anchor moved to IPE220 by the toggle).
        h.click("HEA200", Modifiers::SHIFT);
        assert_eq!(h.selected_names(), ["IPE220", "IPE240", "HEA200"]);
        // A plain click replaces the selection again.
        h.click("TUBE50", Modifiers::NONE);
        assert_eq!(h.selected_names(), ["TUBE50"]);
        h.click("PIPE60", Modifiers::SHIFT);
        assert_eq!(h.selected_names(), ["TUBE50", "PIPE60"]);
        // Import the selection with a real click on the button.
        let outcome = h
            .click("Import selected", Modifiers::NONE)
            .expect("outcome");
        let message = outcome.unwrap();
        assert!(message.contains("2 section(s)"), "{message}");
        assert!(h.model.frame_sections.contains_key("TUBE50"));
        assert!(h.model.frame_sections.contains_key("PIPE60"));
        assert_eq!(h.state.imported, ["TUBE50", "PIPE60"]);
        // The same import again is refused (the names exist) and reported.
        let outcome = h
            .click("Import selected", Modifiers::NONE)
            .expect("outcome");
        assert!(outcome.unwrap_err().contains("already exists"));
    }

    #[test]
    fn window_drag_selects_a_range_and_filters_hide_rows() {
        let mut h = Harness::new();
        let out = h.frame(Modifiers::NONE, vec![]);
        let from = text_center(&out, "IPE220").unwrap();
        let to = text_center(&out, "HEA200").unwrap();
        h.frame(
            Modifiers::NONE,
            Harness::button(from, true, Modifiers::NONE),
        );
        h.frame(Modifiers::NONE, vec![egui::Event::PointerMoved(to)]);
        h.frame(Modifiers::NONE, Harness::button(to, false, Modifiers::NONE));
        assert_eq!(h.selected_names(), ["IPE220", "IPE240", "HEA200"]);
        // A name filter hides rows and drops them from the selection.
        h.state.name_filter = "ipe2".into();
        h.frame(Modifiers::NONE, vec![]);
        let out = h.frame(Modifiers::NONE, vec![]);
        assert!(text_center(&out, "HEA200").is_none());
        assert_eq!(h.selected_names(), ["IPE220", "IPE240"]);
    }

    #[test]
    fn closed_window_draws_nothing_and_without_a_database_it_says_so() {
        let mut h = Harness::new();
        h.state.open = false;
        let out = h.frame(Modifiers::NONE, vec![]);
        assert!(out.shapes.is_empty());
        h.state.open = true;
        h.state.db = None;
        h.frame(Modifiers::NONE, vec![]);
        let out = h.frame(Modifiers::NONE, vec![]);
        assert!(text_center(&out, "Load a .pro file to list its shapes.").is_some());
        // Loading a missing file reports an error and keeps the window state.
        h.state.path = "/nonexistent/structurax/none.pro".into();
        let outcome = h.click("Load", Modifiers::NONE).expect("outcome");
        assert!(outcome.unwrap_err().contains("none.pro"));
        assert!(h.state.db.is_none());
    }

    #[test]
    fn a_chosen_file_is_loaded_through_the_chooser_state() {
        let dir =
            std::env::temp_dir().join(format!("structurax_section_import_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("mine.pro");
        std::fs::write(&file, pro_bytes(3.0, &[("X1", "W", 1.0), ("X2", "W", 2.0)])).unwrap();
        let mut h = Harness::new();
        h.state.db = None;
        file_browser::open_for(
            &mut h.files,
            file_browser::Purpose::SectionFile,
            &file,
            &["pro"],
            false,
        );
        // Drive the chooser window to a choice with real clicks.
        let ctx = egui::Context::default();
        let mut time = 0.0;
        let mut files = std::mem::take(&mut h.files);
        let mut shown = None;
        for _ in 0..3 {
            time += 0.3;
            let out = ctx.run(input(time, Modifiers::NONE), |ctx| {
                file_browser::render(ctx, &mut files)
            });
            shown = Some(out);
        }
        let pos = text_center(shown.as_ref().unwrap(), "mine.pro").unwrap();
        for pressed in [true, false] {
            time += 0.3;
            let mut raw = input(time, Modifiers::NONE);
            raw.events = Harness::button(pos, pressed, Modifiers::NONE);
            let _ = ctx.run(raw, |ctx| file_browser::render(ctx, &mut files));
        }
        time += 0.3;
        let out = ctx.run(input(time, Modifiers::NONE), |ctx| {
            file_browser::render(ctx, &mut files)
        });
        let choose = text_center(&out, "Choose").unwrap();
        for pressed in [true, false] {
            time += 0.3;
            let mut raw = input(time, Modifiers::NONE);
            raw.events = Harness::button(choose, pressed, Modifiers::NONE);
            let _ = ctx.run(raw, |ctx| file_browser::render(ctx, &mut files));
        }
        h.files = files;
        let (_, outcome) = h.frame_outcome(Modifiers::NONE, vec![]);
        let message = outcome.expect("the chosen file is loaded").unwrap();
        assert!(message.contains("2 shapes"), "{message}");
        assert_eq!(h.state.db.as_ref().unwrap().sections.len(), 2);
        assert!(h.state.path.ends_with("mine.pro"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    // ---- Opt-in vendor corpus: euro.pro -------------------------------------

    fn vendor_euro() -> Option<std::path::PathBuf> {
        let dir = std::path::PathBuf::from(std::env::var_os("STRUCTURAX_VENDOR_FIXTURES")?);
        let path = dir.join("euro.pro");
        path.exists().then_some(path)
    }

    #[test]
    fn vendor_euro_ipe_shapes_import_with_exact_scaled_values() {
        let Some(path) = vendor_euro() else {
            eprintln!("skipped: STRUCTURAX_VENDOR_FIXTURES has no euro.pro");
            return;
        };
        let db = ProDatabase::from_file(Path::new(&path)).unwrap();
        assert_eq!(db.length_unit, sap_units::LengthUnit::Mm);
        // The name filter is a substring match (166 names contain "IPE");
        // the 58 IPE shapes are the names that start with it.
        let ipe: Vec<usize> = matching_rows(&db, None, "IPE")
            .into_iter()
            .filter(|&i| db.sections[i].name.starts_with("IPE"))
            .collect();
        assert_eq!(ipe.len(), 58);
        let mut model = model_kn_m();
        let material = steel(&model);
        let all: Vec<String> = ipe.iter().map(|&i| db.sections[i].name.clone()).collect();
        import_sections(&mut model, &db, &all, &material, false).unwrap();
        // The record values are mm units; the model is kN, m, so fl = 0.001.
        let fl = 0.001_f64;
        let s = &model.frame_sections["IPE300"];
        assert!((s.area - 5380.0 * fl * fl).abs() < 1e-12, "{}", s.area);
        assert!((s.i33 - 8.356e7 * fl.powi(4)).abs() < 1e-12, "{}", s.i33);
        assert!((s.i22 - 6.04e6 * fl.powi(4)).abs() < 1e-12, "{}", s.i22);
        assert!((s.z33 - 628000.0 * fl.powi(3)).abs() < 1e-12, "{}", s.z33);
        assert!((s.z22 - 125000.0 * fl.powi(3)).abs() < 1e-12, "{}", s.z22);
        assert!(
            (s.torsional_j - 199000.0 * fl.powi(4)).abs() < 1e-12,
            "{}",
            s.torsional_j
        );
        // The plan's rounded values.
        assert!((s.area - 5.38e-3).abs() < 1e-12);
        assert!((s.i33 - 8.356e-5).abs() < 1e-12);
        assert!((s.z33 - 6.28e-4).abs() < 1e-12);
        assert!((s.torsional_j - 1.99e-7).abs() < 1e-12);
        model.define_auto_select_list("EURO_IPE", &all).unwrap();
        assert_eq!(model.frame_sections["EURO_IPE"].auto_select_list.len(), 58);
    }
}
