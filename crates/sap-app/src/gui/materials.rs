//! Define > Materials from Library: pick a region, type, standard and grade from the
//! vendor's material library files and add it as a material.
//!
//! The files (`CSiMaterialLibrary*.xml`, see `sap_format::material_library`)
//! are read at run time from a folder, found like the `.pro` section files or
//! chosen with Browse. Nothing from them ships with the program.

use super::file_browser;
use eframe::egui;
use sap_format::material_library::{
    fold_type, library_files_in, library_material, read_material_library_dir, LibraryEntry,
    MaterialLibrary,
};
use sap_model::SapModel;
use std::path::{Path, PathBuf};

#[derive(Debug, Clone, Default)]
pub(super) struct State {
    pub open: bool,
    /// The folder of the library files, as typed.
    dir: String,
    /// What the last load of `dir` gave.
    library: Option<Result<MaterialLibrary, String>>,
    /// The folder was searched for once.
    searched: bool,
    region: String,
    type_label: String,
    standard: String,
    grade: String,
    /// The name of the new material.
    name: String,
    /// The entry `name` was filled in for.
    name_for: String,
    /// The outcome of the last Add.
    message: Option<Result<String, String>>,
}

/// The folders searched for library files, like the section files of
/// `resolve_pro_database_path`: `materials` and `extracted/files` under the
/// working directory, then next to the program and in the shared folders.
fn candidate_dirs() -> Vec<PathBuf> {
    let mut dirs = vec![PathBuf::from("materials"), PathBuf::from("extracted/files")];
    if let Ok(exe) = std::env::current_exe() {
        if let Some(exe_dir) = exe.parent() {
            dirs.push(exe_dir.join("materials"));
            dirs.push(exe_dir.join("../share/structurax/materials"));
            dirs.push(exe_dir.join("../Resources/materials"));
        }
    }
    if let Ok(home) = std::env::var("HOME") {
        dirs.push(PathBuf::from(home).join(".local/share/structurax/materials"));
    }
    dirs.push(PathBuf::from("/usr/local/share/structurax/materials"));
    dirs.push(PathBuf::from("/usr/share/structurax/materials"));
    dirs
}

/// The first candidate folder that holds a library file.
fn discover_dir() -> Option<PathBuf> {
    candidate_dirs()
        .into_iter()
        .find(|dir| library_files_in(dir).is_ok_and(|files| !files.is_empty()))
}

/// Reads the library of `state.dir` and resets the choice.
fn load(state: &mut State) {
    let dir = state.dir.trim().to_string();
    state.library =
        Some(read_material_library_dir(Path::new(&dir)).map_err(|error| error.to_string()));
    state.region.clear();
    state.type_label.clear();
    state.standard.clear();
    state.grade.clear();
    state.name_for.clear();
    state.message = None;
    normalize(state);
}

/// Keeps region, type, standard and grade valid for the library: a value that
/// is not offered is replaced by the first one that is. The first region is
/// "United States" when the library has it.
fn normalize(state: &mut State) {
    let Some(Ok(library)) = &state.library else {
        return;
    };
    let regions = library.regions();
    if !regions.contains(&state.region) {
        state.region = regions
            .iter()
            .find(|r| r.as_str() == "United States")
            .or(regions.first())
            .cloned()
            .unwrap_or_default();
    }
    let types = library.type_labels(&state.region);
    if !types
        .iter()
        .any(|t| fold_type(t) == fold_type(&state.type_label))
    {
        state.type_label = types.first().cloned().unwrap_or_default();
    }
    let standards = library.standards(&state.region, &state.type_label);
    if !standards.contains(&state.standard) {
        state.standard = standards.first().cloned().unwrap_or_default();
    }
    let grades = library.grades(&state.region, &state.type_label, &state.standard);
    if !grades.iter().any(|e| e.grade == state.grade) {
        state.grade = grades
            .iter()
            .find(|e| e.is_default)
            .or(grades.first())
            .map(|e| e.grade.clone())
            .unwrap_or_default();
    }
    let key = format!(
        "{}|{}|{}|{}",
        state.region,
        fold_type(&state.type_label),
        state.standard,
        state.grade
    );
    if state.name_for != key {
        if let Some(entry) = library.find(
            &state.region,
            &state.type_label,
            &state.standard,
            &state.grade,
        ) {
            state.name = entry.name.clone();
        }
        state.name_for = key;
    }
}

/// Which library material to add.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct Pick {
    pub region: String,
    pub type_label: String,
    pub standard: String,
    pub grade: String,
}

/// Adds the library material `pick` to the model under `name`, converted to
/// the present units. A refused command leaves the model unchanged.
///
/// Refused: a locked model; a region no file defines; a material the library
/// does not have; a name that is empty or already used; and everything
/// `library_material` refuses (lightweight concrete, shear strength reduction
/// factors other than 1, aluminum, cold-formed and unknown types, missing
/// values).
pub(super) fn add_material(
    model: &mut SapModel,
    library: &MaterialLibrary,
    pick: &Pick,
    name: &str,
) -> Result<String, String> {
    if model.is_locked {
        return Err("Model is locked. Use Edit > Unlock Model before editing.".into());
    }
    if !library.regions().contains(&pick.region) {
        return Err(format!("No library file defines region '{}'.", pick.region));
    }
    let Some(entry) = library.find(&pick.region, &pick.type_label, &pick.standard, &pick.grade)
    else {
        return Err(format!(
            "The library has no {} material '{}' in standard '{}' of region '{}'.",
            pick.type_label, pick.grade, pick.standard, pick.region
        ));
    };
    let name = name.trim();
    if model.materials.contains_key(name) {
        return Err(format!("A material named '{name}' already exists."));
    }
    let material = library_material(entry, model.present_units, name)?;
    model.materials.insert(name.to_string(), material);
    model.case_results.clear();
    model.steel_design_results.clear();
    model.concrete_design_results.clear();
    Ok(format!(
        "Added material {name} from the library ({} / {} / {}).",
        entry.region, entry.standard, entry.grade
    ))
}

fn entry_summary(ui: &mut egui::Ui, entry: &LibraryEntry) {
    let v = &entry.values;
    let show = |x: Option<f64>| x.map_or("-".to_string(), |x| format!("{x}"));
    ui.label(format!(
        "File {} (line {}), units {}",
        entry.source,
        entry.line,
        entry.units.s2k_label()
    ));
    ui.label(format!(
        "E {}   nu {}   alpha {}",
        show(v.e),
        show(v.nu),
        show(v.alpha)
    ));
    ui.label(format!(
        "weight density {}   mass density {}",
        show(v.weight_density),
        show(v.mass_density)
    ));
    ui.label(format!(
        "Fy {}   Fu {}   f'c {}",
        show(v.fy),
        show(v.fu),
        show(v.fc)
    ));
    if v.lightweight {
        ui.colored_label(
            egui::Color32::LIGHT_RED,
            "Lightweight concrete: not supported.",
        );
    }
    if let Some(r) = v.shear_reduction.filter(|r| *r != 1.0) {
        ui.colored_label(
            egui::Color32::LIGHT_RED,
            format!("Shear strength reduction factor {r}: not supported."),
        );
    }
}

fn combo(
    ui: &mut egui::Ui,
    id: &str,
    label: &str,
    value: &mut String,
    options: &[String],
    display: impl Fn(&String) -> String,
) {
    ui.horizontal(|ui| {
        ui.label(label);
        egui::ComboBox::from_id_salt(id)
            .width(260.0)
            .selected_text(display(value))
            .show_ui(ui, |ui| {
                for option in options {
                    ui.selectable_value(value, option.clone(), display(option));
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
    files: &mut file_browser::State,
) -> Option<Result<String, String>> {
    if !state.open {
        return None;
    }
    if let Some(path) = file_browser::take_choice(files, file_browser::Purpose::MaterialLibraryDir)
    {
        state.dir = path.display().to_string();
        load(state);
    }
    if state.library.is_none() && !state.searched {
        state.searched = true;
        if let Some(dir) = discover_dir() {
            state.dir = dir.display().to_string();
            load(state);
        }
    }
    let mut open = state.open;
    let mut outcome = None;
    egui::Window::new("Materials from Library")
        .open(&mut open)
        .default_width(520.0)
        .vscroll(true)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label("Library folder");
                ui.text_edit_singleline(&mut state.dir);
                if ui.button("Browse...").clicked() {
                    let start = if state.dir.trim().is_empty() {
                        PathBuf::from(".")
                    } else {
                        PathBuf::from(state.dir.trim())
                    };
                    file_browser::open_for(
                        files,
                        file_browser::Purpose::MaterialLibraryDir,
                        &start,
                        &[],
                        true,
                    );
                }
                if ui.button("Load").clicked() {
                    load(state);
                }
            });
            ui.label(
                "Reads CSiMaterialLibrary*.xml (and extension-less files whose name starts \
                 with csimateriallibrary) in alphabetical order; a later file replaces an \
                 earlier material with the same region, standard, type and grade.",
            );
            ui.separator();
            let library = match &state.library {
                None => {
                    ui.label("No library loaded. Choose the folder and press Load.");
                    return;
                }
                Some(Err(error)) => {
                    ui.colored_label(egui::Color32::LIGHT_RED, error);
                    return;
                }
                Some(Ok(library)) => library.clone(),
            };
            ui.label(format!(
                "{} materials in {} regions.",
                library.entries.len(),
                library.regions().len()
            ));
            let mut region = state.region.clone();
            let mut type_label = state.type_label.clone();
            let mut standard = state.standard.clone();
            let mut grade = state.grade.clone();
            combo(
                ui,
                "matlib_region",
                "Region",
                &mut region,
                &library.regions(),
                |s| s.clone(),
            );
            combo(
                ui,
                "matlib_type",
                "Material type",
                &mut type_label,
                &library.type_labels(&region),
                |s| s.clone(),
            );
            combo(
                ui,
                "matlib_standard",
                "Standard",
                &mut standard,
                &library.standards(&region, &type_label),
                |s| s.clone(),
            );
            let grades: Vec<String> = library
                .grades(&region, &type_label, &standard)
                .iter()
                .map(|e| e.grade.clone())
                .collect();
            combo(ui, "matlib_grade", "Grade", &mut grade, &grades, |s| {
                s.clone()
            });
            state.region = region;
            state.type_label = type_label;
            state.standard = standard;
            state.grade = grade;
            ui.separator();
            if let Some(entry) = library.find(
                &state.region,
                &state.type_label,
                &state.standard,
                &state.grade,
            ) {
                entry_summary(ui, entry);
            }
            ui.horizontal(|ui| {
                ui.label("Material name");
                ui.text_edit_singleline(&mut state.name);
            });
            if ui.button("Add Material").clicked() {
                let pick = Pick {
                    region: state.region.clone(),
                    type_label: state.type_label.clone(),
                    standard: state.standard.clone(),
                    grade: state.grade.clone(),
                };
                let result = add_material(model, &library, &pick, &state.name);
                state.message = Some(result.clone());
                outcome = Some(result);
            }
            match &state.message {
                Some(Ok(message)) => {
                    ui.label(message);
                }
                Some(Err(message)) => {
                    ui.colored_label(egui::Color32::LIGHT_RED, message);
                }
                None => {}
            }
        });
    state.open = open;
    normalize(state);
    outcome
}

#[cfg(test)]
mod tests {
    use super::*;
    use sap_format::material_library::parse_material_library;
    use sap_model::MatType;
    use sap_units::Units;

    const XML: &str = r#"<?xml version="1.0"?>
<propertyFile>
  <control lengthUnits="in" forceUnits="Kip"/>
  <region name="United States">
    <collection type="Materials">
      <standard name="ASTM A36">
        <material type="Steel" grade="Grade 36" name="A36" isDefault="true">
          <weightDensity>0.000283564814814815</weightDensity>
          <massDensity>0.000000734455290352564</massDensity>
          <modulusOfElasticity>29000</modulusOfElasticity>
          <poissonsRatio>0.3</poissonsRatio>
          <coefficientOfThermalExpansion>0.0000065</coefficientOfThermalExpansion>
          <minimumYieldStress>36</minimumYieldStress>
          <minimumTensileStress>58</minimumTensileStress>
        </material>
        <material type="aluminum" grade="6061" name="AL">
          <weightDensity>0.000098</weightDensity>
          <massDensity>0.000000254</massDensity>
          <modulusOfElasticity>10100</modulusOfElasticity>
          <poissonsRatio>0.33</poissonsRatio>
          <coefficientOfThermalExpansion>0.000013</coefficientOfThermalExpansion>
        </material>
      </standard>
      <standard name="ACI 318">
        <material type="concrete" grade="4000" name="4000Psi">
          <weightDensity>0.00008680555556</weightDensity>
          <massDensity>0.0000002248332521</massDensity>
          <modulusOfElasticity>3604.9965</modulusOfElasticity>
          <poissonsRatio>0.2</poissonsRatio>
          <coefficientOfThermalExpansion>0.0000055</coefficientOfThermalExpansion>
          <compressiveStrength>4</compressiveStrength>
          <isLightWeight>false</isLightWeight>
          <shearStrengthReductionFactor>1</shearStrengthReductionFactor>
        </material>
        <material type="concrete" grade="4000LW" name="4000PsiLW">
          <weightDensity>0.00006365740741</weightDensity>
          <massDensity>0.0000001648777182</massDensity>
          <modulusOfElasticity>2407.8696</modulusOfElasticity>
          <poissonsRatio>0.2</poissonsRatio>
          <coefficientOfThermalExpansion>0.0000055</coefficientOfThermalExpansion>
          <compressiveStrength>4</compressiveStrength>
          <isLightWeight>true</isLightWeight>
          <shearStrengthReductionFactor>0.85</shearStrengthReductionFactor>
        </material>
      </standard>
    </collection>
  </region>
</propertyFile>
"#;

    fn library() -> MaterialLibrary {
        let mut library = MaterialLibrary::default();
        library.merge(parse_material_library(XML, "us.xml").unwrap());
        library
    }

    fn pick(ty: &str, standard: &str, grade: &str) -> Pick {
        Pick {
            region: "United States".into(),
            type_label: ty.into(),
            standard: standard.into(),
            grade: grade.into(),
        }
    }

    fn close(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol * a.abs().max(b.abs())
    }

    #[test]
    fn add_converts_to_the_present_units_and_refuses_leaving_the_model_unchanged() {
        let lib = library();
        let mut model = SapModel::new(Units::KnMC);
        let message = add_material(
            &mut model,
            &lib,
            &pick("Steel", "ASTM A36", "Grade 36"),
            "A36",
        )
        .unwrap();
        assert!(message.contains("A36"));
        let m = &model.materials["A36"];
        assert_eq!(m.mat_type, MatType::Steel);
        let ksi = 6_894.757_293_168;
        assert!(close(m.fy, 36.0 * ksi, 1e-9));
        assert!(close(m.fu, 58.0 * ksi, 1e-9));
        assert!(close(m.e, 29000.0 * ksi, 1e-9));
        // Concrete: type given in another case, fy stays 0.
        add_material(&mut model, &lib, &pick("CONCRETE", "ACI 318", "4000"), "C4").unwrap();
        assert!(close(model.materials["C4"].fc, 4.0 * ksi, 1e-9));
        assert_eq!(model.materials["C4"].fy, 0.0);
        // In kip and in the values are the file's.
        let mut kip = SapModel::new(Units::KipInF);
        add_material(
            &mut kip,
            &lib,
            &pick("Steel", "ASTM A36", "Grade 36"),
            "A36b",
        )
        .unwrap();
        assert_eq!(kip.materials["A36b"].e, 29000.0);
        assert_eq!(kip.materials["A36b"].unit_weight, 0.000283564814814815);

        // Refusals; each leaves the materials as they were.
        let before = model.materials.clone();
        let refuse = |model: &mut SapModel, p: Pick, name: &str, word: &str| {
            let before = model.materials.clone();
            let error = add_material(model, &lib, &p, name).unwrap_err();
            assert!(error.contains(word), "{error}");
            assert_eq!(model.materials, before);
        };
        refuse(
            &mut model,
            pick("Steel", "ASTM A36", "Grade 36"),
            "A36",
            "already exists",
        );
        let mut missing = pick("Steel", "ASTM A36", "Grade 36");
        missing.region = "Atlantis".into();
        refuse(
            &mut model,
            missing,
            "X",
            "library file defines region 'Atlantis'",
        );
        let mut us_gone = SapModel::new(Units::KnMC);
        let empty = MaterialLibrary::default();
        let error = add_material(
            &mut us_gone,
            &empty,
            &pick("Steel", "ASTM A36", "Grade 36"),
            "X",
        )
        .unwrap_err();
        assert!(
            error.contains("No library file defines region 'United States'"),
            "{error}"
        );
        refuse(
            &mut model,
            pick("concrete", "ACI 318", "4000LW"),
            "LW",
            "lightweight",
        );
        refuse(
            &mut model,
            pick("aluminum", "ASTM A36", "6061"),
            "AL",
            "not supported",
        );
        refuse(
            &mut model,
            pick("steel", "ASTM A36", "Grade 99"),
            "X",
            "no steel material",
        );
        refuse(
            &mut model,
            pick("steel", "ASTM A36", "Grade 36"),
            "  ",
            "name",
        );
        assert_eq!(model.materials.len(), before.len());
        model.is_locked = true;
        refuse(
            &mut model,
            pick("steel", "ASTM A36", "Grade 36"),
            "A36c",
            "locked",
        );
    }

    #[test]
    fn selection_follows_the_library_and_fills_in_the_name() {
        let mut state = State::default();
        state.library = Some(Ok(library()));
        normalize(&mut state);
        assert_eq!(state.region, "United States");
        assert_eq!(state.type_label, "Steel");
        assert_eq!(state.standard, "ASTM A36");
        assert_eq!(state.grade, "Grade 36");
        assert_eq!(state.name, "A36");
        state.type_label = "concrete".into();
        normalize(&mut state);
        assert_eq!(state.standard, "ACI 318");
        assert_eq!(state.grade, "4000");
        assert_eq!(state.name, "4000Psi");
        state.region = "Nowhere".into();
        normalize(&mut state);
        assert_eq!(state.region, "United States");
    }

    #[test]
    fn a_folder_is_read_by_load_and_a_bad_one_reports_why() {
        let dir =
            std::env::temp_dir().join(format!("structurax_materials_gui_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("csimateriallibraryunitedstat"), XML).unwrap();
        let mut state = State::default();
        state.dir = dir.display().to_string();
        load(&mut state);
        let lib = state.library.as_ref().unwrap().as_ref().unwrap();
        assert_eq!(lib.entries.len(), 4);
        std::fs::write(dir.join("csimateriallibrarybad.xml"), "<a>\n</b>").unwrap();
        load(&mut state);
        let error = state.library.as_ref().unwrap().as_ref().unwrap_err();
        assert!(error.contains("csimateriallibrarybad.xml") && error.contains("line 2"));
        let _ = std::fs::remove_dir_all(&dir);
        load(&mut state);
        assert!(state.library.as_ref().unwrap().is_err());
    }

    fn input(time: f64) -> egui::RawInput {
        egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::Vec2::new(1000.0, 900.0),
            )),
            time: Some(time),
            ..Default::default()
        }
    }

    fn text_shapes(out: &egui::FullOutput) -> Vec<(String, egui::Pos2)> {
        out.shapes
            .iter()
            .filter_map(|s| match &s.shape {
                egui::epaint::Shape::Text(t) => {
                    Some((t.galley.job.text.clone(), t.pos + t.galley.size() * 0.5))
                }
                _ => None,
            })
            .collect()
    }

    #[test]
    fn headless_render_shows_the_window_and_the_add_button_adds_the_material() {
        let ctx = egui::Context::default();
        let mut files = file_browser::State::default();
        let mut model = SapModel::new(Units::KipInF);
        let mut state = State::default();
        // Closed: nothing is drawn and nothing happens.
        let out = ctx.run(input(1.0), |ctx| {
            assert!(render(ctx, &mut state, &mut model, &mut files).is_none());
        });
        assert!(text_shapes(&out).is_empty());

        state.open = true;
        state.searched = true;
        state.library = Some(Ok(library()));
        normalize(&mut state);
        state.name = "MYA36".into();
        let _ = ctx.run(input(2.0), |ctx| {
            render(ctx, &mut state, &mut model, &mut files);
        });
        let out = ctx.run(input(3.0), |ctx| {
            render(ctx, &mut state, &mut model, &mut files);
        });
        let texts = text_shapes(&out);
        assert!(texts.iter().any(|(t, _)| t == "Materials from Library"));
        assert!(texts
            .iter()
            .any(|(t, _)| t.contains("4 materials in 1 regions")));
        assert!(texts.iter().any(|(t, _)| t.contains("E 29000")));
        let (_, pos) = texts
            .iter()
            .find(|(t, _)| t == "Add Material")
            .expect("the Add Material button is drawn")
            .clone();
        let mut down = input(4.0);
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
            render(ctx, &mut state, &mut model, &mut files);
        });
        let mut up = input(4.1);
        up.events = vec![egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed: false,
            modifiers: Default::default(),
        }];
        let mut outcome = None;
        let _ = ctx.run(up, |ctx| {
            outcome = render(ctx, &mut state, &mut model, &mut files);
        });
        assert!(matches!(outcome, Some(Ok(_))), "{outcome:?}");
        assert_eq!(model.materials["MYA36"].fy, 36.0);
        assert_eq!(model.materials["MYA36"].fu, 58.0);
    }
}
