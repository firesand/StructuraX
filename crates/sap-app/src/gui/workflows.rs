//! Model-editing workflows with validation before mutation.
use super::*;
use sap_model::{
    AreaLoadSurfacePressure, AreaSection, CableParams, ComboType, LinkObj, LinkProp, LinkPropType,
    LoadCombo, MatType, ShellType,
};

pub(super) struct Workflows {
    pub open: bool,
    pub tab: usize,
    delta: [f64; 3],
    copies: usize,
    segments: usize,
    pattern: String,
    pattern_type: LoadPatternType,
    self_weight: f64,
    force: [f64; 6],
    distributed: f64,
    frame_load_dir: i32,
    pressure: f64,
    face: i32,
    replace_loads: bool,
    area: AreaSection,
    mesh: [usize; 2],
    release_i: [bool; 6],
    release_j: [bool; 6],
    tc_has_comp: bool,
    tc_comp_limit: f64,
    tc_has_tens: bool,
    tc_tens_limit: f64,
    cable_mode: usize,
    cable_value: f64,
    cable_segments: usize,
    combo_name: String,
    combo_type: ComboType,
    terms: Vec<(String, f64)>,
    material: sap_model::Material,
    material_units: Option<Units>,
    frame_section: String,
    link_prop: LinkProp,
    link_name: String,
    link_point_i: String,
    link_point_j: String,
    link_grounded: bool,
    link_angle_deg: f64,
}
impl Default for Workflows {
    fn default() -> Self {
        Self {
            open: false,
            tab: 0,
            delta: [0.0; 3],
            copies: 1,
            segments: 2,
            pattern: "DEAD".into(),
            pattern_type: LoadPatternType::Dead,
            self_weight: 0.0,
            force: [0.0; 6],
            distributed: 0.0,
            frame_load_dir: 6,
            pressure: 0.0,
            face: -2,
            replace_loads: true,
            area: AreaSection::default(),
            mesh: [1, 1],
            release_i: [false; 6],
            release_j: [false; 6],
            tc_has_comp: true,
            tc_comp_limit: 0.0,
            tc_has_tens: false,
            tc_tens_limit: 0.0,
            cable_mode: 0,
            cable_value: 240.0,
            cable_segments: 16,
            combo_name: "COMB1".into(),
            combo_type: ComboType::LinearAdd,
            terms: vec![("DEAD".into(), 1.0)],
            material: sap_model::Material::default_steel_kip_in(),
            material_units: None,
            frame_section: "FSEC1".into(),
            link_prop: LinkProp::default(),
            link_name: "L1".into(),
            link_point_i: "1".into(),
            link_point_j: "2".into(),
            link_grounded: false,
            link_angle_deg: 0.0,
        }
    }
}

impl Workflows {
    pub(super) fn scale_for_units(&mut self, from: Units, to: Units, model: &SapModel) {
        if from == to {
            return;
        }
        let fl = from.length_factor_to(to);
        let ff = from.force_factor_to(to);
        let f_mass = from.mass_factor_to(to);
        for d in &mut self.delta {
            *d *= fl;
        }
        for f in &mut self.force[..3] {
            *f *= ff;
        }
        for m in &mut self.force[3..] {
            *m *= ff * fl;
        }
        self.distributed *= ff / fl;
        self.pressure *= ff / (fl * fl);
        self.area.thickness_membrane *= fl;
        self.area.thickness_bending *= fl;
        self.tc_comp_limit *= ff;
        self.tc_tens_limit *= ff;
        if self.cable_mode == 1 {
            self.cable_value *= fl;
        } else if self.cable_mode >= 2 {
            self.cable_value *= ff;
        }
        for d in 0..3 {
            self.link_prop.ke[d] *= ff / fl;
            self.link_prop.ce[d] *= ff / fl;
            self.link_prop.k_nl[d] *= ff / fl;
            if matches!(
                self.link_prop.link_type,
                LinkPropType::Gap | LinkPropType::Hook
            ) {
                self.link_prop.param1[d] *= fl;
            } else if self.link_prop.link_type == LinkPropType::Damper {
                self.link_prop.param1[d] *= ff / fl;
            } else {
                self.link_prop.param1[d] *= ff;
            }
        }
        for d in 3..6 {
            self.link_prop.ke[d] *= ff * fl;
            self.link_prop.ce[d] *= ff * fl;
            self.link_prop.k_nl[d] *= ff * fl;
            if !matches!(
                self.link_prop.link_type,
                LinkPropType::Gap | LinkPropType::Hook
            ) {
                self.link_prop.param1[d] *= ff * fl;
            }
        }
        self.link_prop.dj2 *= fl;
        self.link_prop.dj3 *= fl;
        self.link_prop.weight *= ff;
        self.link_prop.mass *= f_mass;
        self.material_units = None;
        initialize_material(self, model);
    }
}
fn editable(model: &SapModel) -> Result<(), String> {
    if model.is_locked {
        Err("Model is locked. Use Edit > Unlock Model before editing.".into())
    } else {
        Ok(())
    }
}
fn invalidate(model: &mut SapModel) {
    model.case_results.clear();
    model.steel_design_results.clear();
    model.concrete_design_results.clear();
}
fn named(name: &str) -> Result<(), String> {
    if name.trim().is_empty() || name.chars().any(char::is_control) {
        Err("Name must be nonempty and contain no control characters.".into())
    } else {
        Ok(())
    }
}
fn load_pattern(model: &SapModel, name: &str) -> Result<(), String> {
    if model.load_patterns.contains_key(name) {
        Ok(())
    } else {
        Err(format!(
            "Load pattern '{name}' does not exist. Define it first."
        ))
    }
}
fn define_pattern(
    model: &mut SapModel,
    name: &str,
    kind: LoadPatternType,
    weight: f64,
) -> Result<String, String> {
    editable(model)?;
    named(name)?;
    if !weight.is_finite() {
        return Err("Self-weight multiplier must be finite.".into());
    }
    if model.load_combos.contains_key(name) {
        return Err("Pattern name collides with a combination name.".into());
    }
    if let Some(case) = model.load_cases.get(name) {
        if case.case_type != sap_model::LoadCaseType::LinearStatic
            || case.loads.len() != 1
            || case.loads[0].load_name != name
            || case.loads[0].scale_factor != 1.0
        {
            return Err(format!("Case '{name}' already exists with different settings; choose another pattern name."));
        }
    }
    model.load_patterns.insert(
        name.into(),
        LoadPattern {
            name: name.into(),
            pat_type: kind,
            self_wt_multiplier: weight,
        },
    );
    model
        .load_cases
        .entry(name.into())
        .or_insert_with(|| LoadCase::linear_static(name, name, 1.0));
    invalidate(model);
    Ok(format!(
        "Saved pattern {name}; created matching static case if absent."
    ))
}
fn define_combo(
    model: &mut SapModel,
    name: &str,
    kind: ComboType,
    terms: &[(String, f64)],
) -> Result<String, String> {
    editable(model)?;
    named(name)?;
    if model.load_cases.contains_key(name) {
        return Err("Combination name conflicts with an analysis case.".into());
    }
    if terms.is_empty() {
        return Err("Add at least one case or combination.".into());
    }
    for (n, sf) in terms {
        if !sf.is_finite() {
            return Err("Combination factors must be finite.".into());
        }
        if !model.load_cases.contains_key(n) && !model.load_combos.contains_key(n) && n != name {
            return Err(format!("Unknown case or combination '{n}'."));
        }
    }
    fn visit(
        model: &SapModel,
        name: &str,
        root: &str,
        terms: &[(String, f64)],
        stack: &mut Vec<String>,
    ) -> Result<(), String> {
        if model.load_cases.contains_key(name) {
            return Ok(());
        }
        if stack.iter().any(|s| s == name) {
            return Err(format!("Combination dependency cycle at '{name}'."));
        }
        let rows = if name == root {
            terms
        } else {
            &model
                .load_combos
                .get(name)
                .ok_or_else(|| format!("Missing combination '{name}'."))?
                .cases
        };
        stack.push(name.into());
        for (child, _) in rows {
            visit(model, child, root, terms, stack)?;
        }
        stack.pop();
        Ok(())
    }
    visit(model, name, name, terms, &mut Vec::new())?;
    model.load_combos.insert(
        name.into(),
        LoadCombo {
            name: name.into(),
            combo_type: kind,
            cases: terms.to_vec(),
        },
    );
    invalidate(model);
    Ok(format!("Saved combination {name}."))
}
fn save_area(model: &mut SapModel, area: &AreaSection) -> Result<String, String> {
    editable(model)?;
    named(&area.name)?;
    if area.prop_type != sap_model::AreaPropType::Shell || !area.mat_angle_deg.is_finite() {
        return Err("This editor requires a shell property with a finite material angle.".into());
    }
    if !model.materials.contains_key(&area.material) {
        return Err("Choose an existing material.".into());
    }
    if !area.thickness_membrane.is_finite()
        || area.thickness_membrane <= 0.0
        || !area.thickness_bending.is_finite()
        || area.thickness_bending <= 0.0
    {
        return Err("Shell thicknesses must be finite and positive.".into());
    }
    if area.modifiers.iter().any(|m| !m.is_finite() || *m < 0.0) {
        return Err("Modifiers must be finite and nonnegative.".into());
    }
    model.area_sections.insert(area.name.clone(), area.clone());
    invalidate(model);
    Ok(format!("Saved area section {}.", area.name))
}
fn material_default(model: &SapModel, kind: MatType) -> Material {
    let defaults = SapModel::new(model.present_units);
    let key = if kind == MatType::Concrete {
        "4000Psi"
    } else {
        "A992Fy50"
    };
    let mut material = defaults.materials[key].clone();
    material.mat_type = kind;
    material
}
fn initialize_material(state: &mut Workflows, model: &SapModel) {
    if state.material_units != Some(model.present_units) {
        state.material = model
            .materials
            .get(&state.material.name)
            .or_else(|| model.materials.values().next())
            .cloned()
            .unwrap_or_else(|| material_default(model, MatType::Steel));
        state.material_units = Some(model.present_units);
    }
}
fn change_material_type(state: &mut Workflows, model: &SapModel, kind: MatType) {
    let name = state.material.name.clone();
    state.material = material_default(model, kind);
    state.material.name = name;
    state.material_units = Some(model.present_units);
}
fn save_material(model: &mut SapModel, input: &sap_model::Material) -> Result<String, String> {
    editable(model)?;
    named(&input.name)?;
    if input.e_ortho.is_some()
        || input.u_ortho.is_some()
        || input.g_ortho.is_some()
        || input.a_ortho.is_some()
    {
        return Err("This editor supports isotropic materials only.".into());
    }
    if !matches!(
        input.mat_type,
        MatType::Steel | MatType::Concrete | MatType::NoDesign
    ) {
        return Err("This editor supports steel, concrete, and no-design materials.".into());
    }
    if [
        input.e,
        input.u,
        input.a,
        input.unit_weight,
        input.unit_mass,
        input.fy,
        input.fu,
        input.fc,
        input.fys,
    ]
    .iter()
    .any(|v| !v.is_finite())
        || input.e <= 0.0
        || input.u <= -1.0
        || input.u >= 0.5
        || input.unit_weight < 0.0
        || input.unit_mass < 0.0
    {
        return Err("Require finite properties, E > 0, -1 < Poisson ratio < 0.5, and nonnegative densities.".into());
    }
    if input.mat_type == MatType::Steel && (input.fy <= 0.0 || input.fu < input.fy) {
        return Err("Steel requires Fu >= Fy > 0.".into());
    }
    if input.mat_type == MatType::Concrete
        && (input.fc <= 0.0 || input.fy <= 0.0 || input.fys <= 0.0)
    {
        return Err("Concrete and reinforcement strengths must be positive.".into());
    }
    let mut material = input.clone();
    material.g = material.e / (2.0 * (1.0 + material.u));
    if !material.g.is_finite() {
        return Err("Derived shear modulus overflows.".into());
    }
    model.materials.insert(material.name.clone(), material);
    invalidate(model);
    Ok(format!("Saved isotropic material {}.", input.name))
}
fn assign_frame_section(model: &mut SapModel, name: &str) -> Result<String, String> {
    editable(model)?;
    let section = model
        .frame_sections
        .get(name)
        .ok_or("Select an existing frame section.")?;
    if !model.materials.contains_key(&section.material) {
        return Err("Section refers to a missing material.".into());
    }
    let count = model.frames.values().filter(|f| f.selected).count();
    if count == 0 {
        return Err("Select at least one frame.".into());
    }
    for frame in model.frames.values_mut().filter(|f| f.selected) {
        frame.section = name.into();
    }
    invalidate(model);
    Ok(format!("Assigned {name} to {count} frames."))
}
fn assign_area(model: &mut SapModel, name: &str, mesh: [usize; 2]) -> Result<String, String> {
    editable(model)?;
    if !model.area_sections.contains_key(name) {
        return Err("Save or select an existing area section first.".into());
    }
    if mesh.iter().any(|n| *n == 0 || *n > 100) {
        return Err("Mesh divisions must be in 1..=100.".into());
    }
    let selected: Vec<_> = model.areas.values().filter(|a| a.selected).collect();
    if selected.is_empty() {
        return Err("Select at least one area.".into());
    }
    if selected
        .iter()
        .any(|a| a.points.len() != 4 && mesh != [1, 1])
    {
        return Err("Subdivision is available for four-node areas only.".into());
    }
    let count = selected.len();
    for a in model.areas.values_mut().filter(|a| a.selected) {
        a.section = name.into();
        a.auto_mesh = (mesh[0], mesh[1]);
    }
    invalidate(model);
    Ok(format!("Assigned section and mesh to {count} areas."))
}
fn assign_releases(model: &mut SapModel, i: [bool; 6], j: [bool; 6]) -> Result<String, String> {
    editable(model)?;
    let count = model.frames.values().filter(|f| f.selected).count();
    if count == 0 {
        return Err("Select at least one frame.".into());
    }
    for f in model.frames.values_mut().filter(|f| f.selected) {
        f.release_i = i;
        f.release_j = j;
        f.start_spring = [0.0; 6];
        f.end_spring = [0.0; 6];
    }
    invalidate(model);
    Ok(format!(
        "Assigned full releases to {count} frames (partial-fixity springs reset)."
    ))
}
fn assign_tc_limits(
    model: &mut SapModel,
    limits: Option<(bool, f64, bool, f64)>,
) -> Result<String, String> {
    editable(model)?;
    let count = model.frames.values().filter(|f| f.selected).count();
    if count == 0 {
        return Err("Select at least one frame.".into());
    }
    if let Some((has_c, comp, has_t, tens)) = limits {
        if !has_c && !has_t {
            return Err("Enable at least one tension or compression limit, or click Clear.".into());
        }
        if !comp.is_finite() || !tens.is_finite() {
            return Err("Tension and compression limits must be finite.".into());
        }
        if has_c && comp > 0.0 {
            return Err(
                "Compression limit must be <= 0 (e.g. 0.0 for tension-only bracing).".into(),
            );
        }
        if has_t && tens < 0.0 {
            return Err(
                "Tension limit must be >= 0 (e.g. 0.0 for compression-only struts).".into(),
            );
        }
        if model
            .frames
            .values()
            .any(|f| f.selected && f.cable_params.is_some())
        {
            return Err("Clear catenary cable parameters before assigning T/C limits.".into());
        }
    }
    for f in model.frames.values_mut().filter(|f| f.selected) {
        f.tc_limits = limits;
    }
    invalidate(model);
    Ok(if limits.is_some() {
        format!("Assigned tension/compression limits to {count} frames.")
    } else {
        format!("Cleared tension/compression limits on {count} frames.")
    })
}
fn assign_cable_params(
    model: &mut SapModel,
    mode: usize,
    value: f64,
    segments: usize,
) -> Result<String, String> {
    editable(model)?;
    let count = model.frames.values().filter(|f| f.selected).count();
    if count == 0 {
        return Err("Select at least one frame.".into());
    }
    let spec = if mode == 0 {
        None
    } else {
        if !value.is_finite() || value <= 0.0 {
            return Err("Cable length or tension parameter must be finite and positive.".into());
        }
        if !(1..=200).contains(&segments) {
            return Err("Cable segments must be in 1..=200.".into());
        }
        if model
            .frames
            .values()
            .any(|f| f.selected && f.tc_limits.is_some())
        {
            return Err("Clear frame T/C limits before converting to a catenary cable.".into());
        }
        Some(match mode {
            1 => CableParams {
                undeformed_length: Some(value),
                target_tension_i: None,
                horizontal_tension: None,
                num_segments: segments,
            },
            2 => CableParams {
                undeformed_length: None,
                target_tension_i: None,
                horizontal_tension: Some(value),
                num_segments: segments,
            },
            _ => CableParams {
                undeformed_length: None,
                target_tension_i: Some(value),
                horizontal_tension: None,
                num_segments: segments,
            },
        })
    };
    for f in model.frames.values_mut().filter(|f| f.selected) {
        f.cable_params = spec.clone();
    }
    invalidate(model);
    Ok(if spec.is_some() {
        format!("Assigned catenary cable parameters to {count} frames.")
    } else {
        format!("Restored standard beam-column formulation on {count} frames.")
    })
}
fn delete_combo(model: &mut SapModel, name: &str) -> Result<String, String> {
    editable(model)?;
    if !model.load_combos.contains_key(name) {
        return Err(format!("Combination '{name}' does not exist."));
    }
    if model
        .load_combos
        .values()
        .any(|c| c.name != name && c.cases.iter().any(|(child, _)| child == name))
    {
        return Err(format!(
            "Combination '{name}' is referenced by another combination."
        ));
    }
    model.load_combos.shift_remove(name);
    invalidate(model);
    Ok(format!("Deleted combination {name}."))
}
#[derive(Clone, Copy)]
enum LoadAssignment {
    Joint([f64; 6]),
    FrameDir(i32, f64),
    Pressure(i32, f64),
}
fn assign_load(
    model: &mut SapModel,
    pattern: &str,
    load: LoadAssignment,
    replace: bool,
) -> Result<String, String> {
    editable(model)?;
    load_pattern(model, pattern)?;
    let (count, finite) = match load {
        LoadAssignment::Joint(f) => (
            model.points.values().filter(|p| p.selected).count(),
            f.iter().all(|v| v.is_finite()),
        ),
        LoadAssignment::FrameDir(_, w) => (
            model.frames.values().filter(|p| p.selected).count(),
            w.is_finite(),
        ),
        LoadAssignment::Pressure(face, p) => {
            if ![-1, -2].contains(&face) {
                return Err("Choose the top or bottom face.".into());
            }
            (
                model.areas.values().filter(|p| p.selected).count(),
                p.is_finite(),
            )
        }
    };
    if count == 0 {
        return Err("No objects of the requested type are selected.".into());
    }
    if !finite {
        return Err("Loads must be finite.".into());
    }
    match load {
        LoadAssignment::Joint(force) => {
            for p in model.points.values_mut().filter(|p| p.selected) {
                if replace {
                    p.loads_force.retain(|l| l.load_pat != pattern);
                }
                p.loads_force.push(PointLoadForce {
                    load_pat: pattern.into(),
                    csys: "GLOBAL".into(),
                    force,
                });
            }
        }
        LoadAssignment::FrameDir(dir, w) => {
            let csys = if (1..=3).contains(&dir) {
                "LOCAL"
            } else {
                "GLOBAL"
            };
            for f in model.frames.values_mut().filter(|f| f.selected) {
                if replace {
                    f.loads_dist.retain(|l| l.load_pat != pattern);
                }
                f.loads_dist.push(FrameLoadDistributed {
                    load_pat: pattern.into(),
                    my_type: 1,
                    dir,
                    csys: csys.into(),
                    rel_dist1: 0.0,
                    rel_dist2: 1.0,
                    val1: w,
                    val2: w,
                });
            }
        }
        LoadAssignment::Pressure(face, val) => {
            for a in model.areas.values_mut().filter(|a| a.selected) {
                if replace {
                    a.loads_pressure
                        .retain(|l| l.load_pat != pattern || l.face != face);
                }
                a.loads_pressure.push(AreaLoadSurfacePressure {
                    load_pat: pattern.into(),
                    face,
                    val,
                    pattern_name: None,
                });
            }
        }
    }
    invalidate(model);
    Ok(format!("Assigned {pattern} loads to {count} objects."))
}
fn vector(ui: &mut egui::Ui, labels: &[&str], values: &mut [f64]) {
    ui.horizontal(|ui| {
        for (label, value) in labels.iter().zip(values) {
            ui.label(*label);
            ui.add(egui::DragValue::new(value).speed(0.1));
        }
    });
}
impl SapGuiApp {
    pub(super) fn open_workflow(&mut self, tab: usize) {
        self.workflows.open = true;
        self.workflows.tab = tab;
        initialize_material(&mut self.workflows, &self.model);
        if tab == 5 {
            if let Some(material) = self.model.materials.values().next() {
                self.workflows.material = material.clone();
            }
        }
    }
    pub(super) fn render_workflows(&mut self, ctx: &egui::Context) {
        let mut state = std::mem::take(&mut self.workflows);
        initialize_material(&mut state, &self.model);
        let mut open = state.open;
        if open {
            egui::Window::new("Model Workflows").open(&mut open).default_width(680.0).vscroll(true).show(ctx,|ui| {
            ui.horizontal_wrapped(|ui| {for (i,title) in ["Geometry","Loads","Areas","Releases / T-C / Cable","Combinations","Materials","Links"].iter().enumerate() {ui.selectable_value(&mut state.tab,i,*title);}});
            ui.label(format!("Units: {}. Selected: {} joints, {} frames, {} areas, {} solids, {} links.",self.model.present_units.s2k_label(),self.model.points.values().filter(|p|p.selected).count(),self.model.frames.values().filter(|p|p.selected).count(),self.model.areas.values().filter(|p|p.selected).count(),self.model.solids.values().filter(|s|s.selected).count(),self.model.links.values().filter(|l|l.selected).count()));
            if self.model.is_locked {ui.colored_label(Color32::YELLOW,"Unlock the model from the Edit menu before applying changes.");}
            ui.separator();
            let mut outcome:Option<Result<String,String>>=None;
            match state.tab {
                0=>{
                    egui::ComboBox::from_id_salt("wf_frame_section").selected_text(&state.frame_section).show_ui(ui,|ui| {for name in self.model.frame_sections.keys() {ui.selectable_value(&mut state.frame_section,name.clone(),name);}});
                    if ui.button("Assign Section to Selected Frames").clicked() {outcome=Some(assign_frame_section(&mut self.model,&state.frame_section));}
                    ui.separator();
                    ui.label("Move or linearly replicate selected joints, frames and areas.");
                    vector(ui,&["dX","dY","dZ"],&mut state.delta);
                    ui.label("Moving shared joints also moves attached unselected elements.");
                    if ui.button("Move Selected").clicked() {outcome=Some(self.model.move_selected(state.delta).map(|r|format!("Moved {} joints.",r.points_moved.len())).map_err(|e|e.to_string()));}
                    ui.horizontal(|ui| {ui.label("Additional copies:");ui.add(egui::DragValue::new(&mut state.copies).range(1..=100));});
                    ui.label("Copies retain assignments. Coincident joints are merged only when their assignments agree.");
                    if ui.button("Replicate Selected").clicked() {outcome=Some(self.model.replicate_selected(state.delta,state.copies).map(|r|format!("Created {} joints, {} frames, {} areas.",r.points_created.len(),r.frames_created.len(),r.areas_created.len())).map_err(|e|e.to_string()));}
                    ui.separator();ui.horizontal(|ui| {ui.label("Equal frame segments:");ui.add(egui::DragValue::new(&mut state.segments).range(2..=100));});
                    if ui.button("Divide Selected Frames").clicked() {outcome=Some(self.model.divide_selected_frames(state.segments).map(|r|format!("Divided selected frames; created {} additional segments and {} joints.",r.frames_created.len(),r.points_created.len())).map_err(|e|e.to_string()));}
                },
                1=>{
                    ui.horizontal(|ui| {ui.label("Pattern:");ui.text_edit_singleline(&mut state.pattern);});
                    egui::ComboBox::from_id_salt("wf_pattern_type").selected_text(format!("{:?}",state.pattern_type)).show_ui(ui,|ui| {for kind in [LoadPatternType::Dead,LoadPatternType::SuperDead,LoadPatternType::Live,LoadPatternType::Wind,LoadPatternType::Quake,LoadPatternType::Snow,LoadPatternType::Temperature,LoadPatternType::Other] {ui.selectable_value(&mut state.pattern_type,kind,format!("{kind:?}"));}});
                    ui.horizontal(|ui| {ui.label("Self-weight multiplier:");ui.add(egui::DragValue::new(&mut state.self_weight).speed(0.1));});
                    ui.horizontal(|ui| {
                        if ui.button("Read Pattern").clicked() {if let Some(p)=self.model.load_patterns.get(&state.pattern) {state.pattern_type=p.pat_type;state.self_weight=p.self_wt_multiplier;} else {outcome=Some(Err("Pattern does not exist.".into()));}}
                        if ui.button("Save Pattern / Create Case if Missing").clicked() {outcome=Some(define_pattern(&mut self.model,&state.pattern,state.pattern_type,state.self_weight));}
                    });
                    ui.separator();ui.checkbox(&mut state.replace_loads,"Replace this assignment type in this pattern (pressure: same face)");
                    ui.label("Joint forces and moments in GLOBAL axes:");vector(ui,&["FX","FY","FZ"],&mut state.force[..3]);vector(ui,&["MX","MY","MZ"],&mut state.force[3..]);
                    if ui.button("Assign to Selected Joints").clicked() {outcome=Some(assign_load(&mut self.model,&state.pattern,LoadAssignment::Joint(state.force),state.replace_loads));}
                    ui.horizontal(|ui| {
                        ui.label("Frame uniform load:");
                        ui.add(egui::DragValue::new(&mut state.distributed).speed(0.01));
                        let dir_label = match state.frame_load_dir {
                            4 => "Global X",
                            5 => "Global Y",
                            6 => "Global Z",
                            10 => "Gravity (-Z)",
                            2 => "Local 2",
                            3 => "Local 3",
                            _ => "Global Z",
                        };
                        egui::ComboBox::from_id_salt("wf_frame_load_dir")
                            .selected_text(dir_label)
                            .show_ui(ui, |ui| {
                                for (code, lbl) in [
                                    (6, "Global Z"),
                                    (10, "Gravity (-Z)"),
                                    (4, "Global X"),
                                    (5, "Global Y"),
                                    (2, "Local 2"),
                                    (3, "Local 3"),
                                ] {
                                    ui.selectable_value(&mut state.frame_load_dir, code, lbl);
                                }
                            });
                    });
                    if ui.button("Assign to Selected Frames").clicked() {outcome=Some(assign_load(&mut self.model,&state.pattern,LoadAssignment::FrameDir(state.frame_load_dir,state.distributed),state.replace_loads));}
                    ui.separator();ui.horizontal(|ui| {ui.label("Area pressure:");ui.add(egui::DragValue::new(&mut state.pressure).speed(0.001));ui.selectable_value(&mut state.face,-2,"Top");ui.selectable_value(&mut state.face,-1,"Bottom");});
                    ui.label("Positive pressure pushes into the selected face: top -local 3, bottom +local 3.");
                    if ui.button("Assign to Selected Areas").clicked() {outcome=Some(assign_load(&mut self.model,&state.pattern,LoadAssignment::Pressure(state.face,state.pressure),state.replace_loads));}
                },
                2=>{
                    ui.horizontal(|ui| {ui.label("Section:");ui.text_edit_singleline(&mut state.area.name);});
                    egui::ComboBox::from_id_salt("wf_existing_area").selected_text("Read existing section").show_ui(ui,|ui| {for (n,a) in &self.model.area_sections {if ui.button(n).clicked() {state.area=a.clone();ui.close_menu();}}});
                    egui::ComboBox::from_id_salt("wf_area_material").selected_text(&state.area.material).show_ui(ui,|ui| {for n in self.model.materials.keys() {ui.selectable_value(&mut state.area.material,n.clone(),n);}});
                    egui::ComboBox::from_id_salt("wf_shell_type").selected_text(format!("{:?}",state.area.shell_type)).show_ui(ui,|ui| {for kind in [ShellType::ShellThin,ShellType::ShellThick,ShellType::Membrane,ShellType::PlateThin,ShellType::PlateThick] {ui.selectable_value(&mut state.area.shell_type,kind,format!("{kind:?}"));}});
                    ui.horizontal(|ui| {ui.label("Membrane thickness:");ui.add(egui::DragValue::new(&mut state.area.thickness_membrane).speed(0.1).range(1e-6..=1e6));ui.label("Bending:");ui.add(egui::DragValue::new(&mut state.area.thickness_bending).speed(0.1).range(1e-6..=1e6));});
                    ui.label("Section modifiers (affect every area using this property):");
                    for (labels,values) in ["F11","F22","F12","M11","M22","M12","V13","V23","Mass","Weight"].chunks(5).zip(state.area.modifiers.chunks_mut(5)) {vector(ui,labels,values);}
                    if ui.button("Save Area Section").clicked() {outcome=Some(save_area(&mut self.model,&state.area));}
                    ui.horizontal(|ui| {ui.label("Mesh divisions:");for n in &mut state.mesh {ui.add(egui::DragValue::new(n).range(1..=100));}});
                    if ui.button("Assign Section and Mesh to Selected Areas").clicked() {outcome=Some(assign_area(&mut self.model,&state.area.name,state.mesh));}
                },
                3=>{
                    ui.label("Checked means a full release. Applying resets existing partial-fixity springs.");
                    for (end,values) in [("I-end",&mut state.release_i),("J-end",&mut state.release_j)] {ui.horizontal(|ui| {ui.label(end);for (label,value) in ["P","V2","V3","T","M2","M3"].iter().zip(values) {ui.checkbox(value,*label);}});}
                    ui.horizontal(|ui| {
                        if ui.button("Read First Selected Frame").clicked() {
                            if let Some(f)=self.model.frames.values().find(|f|f.selected) {
                                state.release_i=f.release_i;state.release_j=f.release_j;
                                if let Some((hc,c,ht,t))=f.tc_limits {state.tc_has_comp=hc;state.tc_comp_limit=c;state.tc_has_tens=ht;state.tc_tens_limit=t;}
                                if let Some(cp)=&f.cable_params {
                                    state.cable_segments=cp.num_segments;
                                    if let Some(l0)=cp.undeformed_length {state.cable_mode=1;state.cable_value=l0;}
                                    else if let Some(h)=cp.horizontal_tension {state.cable_mode=2;state.cable_value=h;}
                                    else if let Some(ti)=cp.target_tension_i {state.cable_mode=3;state.cable_value=ti;}
                                } else {state.cable_mode=0;}
                            } else {outcome=Some(Err("Select a frame first.".into()));}
                        }
                        if ui.button("Assign Releases to Selected Frames").clicked() {outcome=Some(assign_releases(&mut self.model,state.release_i,state.release_j));}
                    });
                    ui.separator();
                    ui.label("Nonlinear Frame Tension / Compression Axial Limits (evaluated in NonlinearStatic cases):");
                    ui.horizontal(|ui| {
                        ui.checkbox(&mut state.tc_has_comp, "Compression limit (<= 0):");
                        ui.add(egui::DragValue::new(&mut state.tc_comp_limit).speed(0.5));
                        ui.checkbox(&mut state.tc_has_tens, "Tension limit (>= 0):");
                        ui.add(egui::DragValue::new(&mut state.tc_tens_limit).speed(0.5));
                    });
                    ui.horizontal(|ui| {
                        if ui.button("Assign T/C Limits").clicked() {
                            outcome = Some(assign_tc_limits(&mut self.model, Some((state.tc_has_comp, state.tc_comp_limit, state.tc_has_tens, state.tc_tens_limit))));
                        }
                        if ui.button("Clear T/C Limits").clicked() {
                            outcome = Some(assign_tc_limits(&mut self.model, None));
                        }
                    });
                    ui.separator();
                    ui.label("Catenary Cable Specification (evaluated in NonlinearStatic cases):");
                    ui.horizontal(|ui| {
                        let mode_lbl = match state.cable_mode {
                            0 => "Standard Beam-Column (Not a Cable)",
                            1 => "Undeformed Length L0",
                            2 => "Horizontal Tension H",
                            _ => "Target Tension at I-end Ti",
                        };
                        egui::ComboBox::from_id_salt("wf_cable_mode").selected_text(mode_lbl).show_ui(ui, |ui| {
                            ui.selectable_value(&mut state.cable_mode, 0, "Standard Beam-Column (Not a Cable)");
                            ui.selectable_value(&mut state.cable_mode, 1, "Undeformed Length L0");
                            ui.selectable_value(&mut state.cable_mode, 2, "Horizontal Tension H");
                            ui.selectable_value(&mut state.cable_mode, 3, "Target Tension at I-end Ti");
                        });
                        if state.cable_mode != 0 {
                            ui.label("Value:");
                            ui.add(egui::DragValue::new(&mut state.cable_value).speed(1.0).range(1e-6..=1e12));
                            ui.label("Segments:");
                            ui.add(egui::DragValue::new(&mut state.cable_segments).range(1..=200));
                        }
                    });
                    if ui.button("Apply Cable / Frame Formulation").clicked() {
                        outcome = Some(assign_cable_params(&mut self.model, state.cable_mode, state.cable_value, state.cable_segments));
                    }
                },
                4=>{
                    ui.horizontal(|ui| {ui.label("Combination:");ui.text_edit_singleline(&mut state.combo_name);});
                    egui::ComboBox::from_id_salt("wf_existing_combo").selected_text("Read existing combination").show_ui(ui,|ui| {for (n,c) in &self.model.load_combos {if ui.button(n).clicked() {state.combo_name=n.clone();state.combo_type=c.combo_type;state.terms=c.cases.clone();ui.close_menu();}}});
                    egui::ComboBox::from_id_salt("wf_combo_type").selected_text(format!("{:?}",state.combo_type)).show_ui(ui,|ui| {for kind in [ComboType::LinearAdd,ComboType::Envelope,ComboType::AbsAdd,ComboType::Srss,ComboType::RangeAdd] {ui.selectable_value(&mut state.combo_type,kind,format!("{kind:?}"));}});
                    let mut remove=None;
                    for (i,(name,sf)) in state.terms.iter_mut().enumerate() {ui.horizontal(|ui| {
                        egui::ComboBox::from_id_salt(("wf_combo_term",i)).selected_text(name.as_str()).show_ui(ui,|ui| {for n in self.model.load_cases.keys().chain(self.model.load_combos.keys()) {ui.selectable_value(name,n.clone(),n);}});
                        ui.add(egui::DragValue::new(sf).speed(0.1));if ui.button("Remove").clicked() {remove=Some(i);}
                    });}
                    if let Some(i)=remove {state.terms.remove(i);}
                    ui.horizontal(|ui| {
                        if ui.button("Add Term").clicked() {state.terms.push((self.model.load_cases.keys().next().cloned().unwrap_or_default(),1.0));}
                        if ui.button("Save Combination").clicked() {outcome=Some(define_combo(&mut self.model,&state.combo_name,state.combo_type,&state.terms));}
                        if ui.button("Delete Combination").clicked() {outcome=Some(delete_combo(&mut self.model,&state.combo_name));}
                    });
                },
                5=>{
                    ui.label("Isotropic material properties in the displayed model units. Saving an existing name updates every section using it.");
                    egui::ComboBox::from_id_salt("wf_material_existing").selected_text("Read existing material").show_ui(ui,|ui| {for (name,material) in &self.model.materials {if ui.button(name).clicked() {state.material=material.clone();ui.close_menu();}}});
                    ui.horizontal(|ui| {ui.label("Name:");ui.text_edit_singleline(&mut state.material.name);});
                    let mut kind=state.material.mat_type;
                    egui::ComboBox::from_id_salt("wf_material_kind").selected_text(format!("{kind:?}")).show_ui(ui,|ui| {for choice in [MatType::Steel,MatType::Concrete,MatType::NoDesign] {ui.selectable_value(&mut kind,choice,format!("{choice:?}"));}});
                    if kind != state.material.mat_type {change_material_type(&mut state,&self.model,kind);}
                    ui.label("Changing material type reloads defaults in current units; edit the values below before saving.");
                    egui::Grid::new("wf_material_props").num_columns(2).show(ui,|ui| {
                        for (label,value) in [("Young modulus E",&mut state.material.e),("Poisson ratio",&mut state.material.u),("Thermal expansion",&mut state.material.a),("Weight per volume",&mut state.material.unit_weight),("Mass per volume",&mut state.material.unit_mass),("Yield strength Fy",&mut state.material.fy),("Tensile strength Fu",&mut state.material.fu),("Concrete strength Fc",&mut state.material.fc),("Shear rebar strength Fys",&mut state.material.fys)] {
                            ui.label(label);ui.add(egui::DragValue::new(value).speed(0.001).max_decimals(12));ui.end_row();
                        }
                    });
                    ui.label("Shear modulus is calculated as E / [2(1 + Poisson ratio)]. Mass and weight densities are independent inputs.");
                    if ui.button("Save Material").clicked() {outcome=Some(save_material(&mut self.model,&state.material));}
                },
                _=>{
                    ui.label("Define Linear, Damper, Gap or Hook link properties and place 1-joint (grounded) or 2-joint link elements.");
                    egui::ComboBox::from_id_salt("wf_link_existing").selected_text("Read existing link property").show_ui(ui,|ui| {
                        for (name,prop) in &self.model.link_props {if ui.button(name).clicked() {state.link_prop=prop.clone();ui.close_menu();}}
                    });
                    ui.horizontal(|ui| {
                        ui.label("Property Name:");
                        ui.text_edit_singleline(&mut state.link_prop.name);
                        let mut ltype = state.link_prop.link_type;
                        egui::ComboBox::from_id_salt("wf_link_type").selected_text(format!("{ltype:?}")).show_ui(ui,|ui| {
                            for choice in [LinkPropType::Linear, LinkPropType::Damper, LinkPropType::Gap, LinkPropType::Hook] {
                                ui.selectable_value(&mut ltype, choice, format!("{choice:?}"));
                            }
                        });
                        if ltype != state.link_prop.link_type {
                            state.link_prop.link_type = ltype;
                            if ltype == LinkPropType::Linear {
                                state.link_prop.dof_nonlinear = [false; 6];
                            }
                        }
                    });
                    let dof_labels = ["U1", "U2", "U3", "R1", "R2", "R3"];
                    ui.horizontal(|ui| {
                        ui.label("Active DOFs:");
                        for (lbl, flag) in dof_labels.iter().zip(&mut state.link_prop.dof_active) {
                            ui.checkbox(flag, *lbl);
                        }
                    });
                    ui.horizontal(|ui| {
                        ui.label("Fixed DOFs:");
                        for (lbl, flag) in dof_labels.iter().zip(&mut state.link_prop.dof_fixed) {
                            ui.checkbox(flag, *lbl);
                        }
                    });
                    if state.link_prop.link_type != LinkPropType::Linear {
                        ui.horizontal(|ui| {
                            ui.label("Nonlinear DOFs:");
                            for (lbl, flag) in dof_labels.iter().zip(&mut state.link_prop.dof_nonlinear) {
                                ui.checkbox(flag, *lbl);
                            }
                        });
                    }
                    ui.label("Effective stiffness ke [U1..R3]:");
                    vector(ui, &dof_labels, &mut state.link_prop.ke);
                    ui.label("Effective damping ce [U1..R3]:");
                    vector(ui, &dof_labels, &mut state.link_prop.ce);
                    if matches!(
                        state.link_prop.link_type,
                        LinkPropType::Damper | LinkPropType::Gap | LinkPropType::Hook
                    ) {
                        let knl_lbl = if state.link_prop.link_type == LinkPropType::Damper {
                            "Maxwell series stiffness k_nl [U1..R3]:"
                        } else {
                            "Nonlinear stiffness k_nl [U1..R3]:"
                        };
                        let p1_lbl = if state.link_prop.link_type == LinkPropType::Damper {
                            "Nonlinear damping coeff c_nl [U1..R3]:"
                        } else {
                            "Initial opening [U1..R3]:"
                        };
                        ui.label(knl_lbl);
                        vector(ui, &dof_labels, &mut state.link_prop.k_nl);
                        ui.label(p1_lbl);
                        vector(ui, &dof_labels, &mut state.link_prop.param1);
                    }
                    if ui.button("Save Link Property").clicked() {
                        let mut prop = state.link_prop.clone();
                        prop.nonlinear_data_complete = true;
                        outcome = Some(sap_api::set_link_property(&mut self.model, prop.clone()).map(|_| format!("Saved link property {}.", prop.name)));
                    }
                    ui.separator();
                    ui.label("Create or Update Link Element:");
                    ui.horizontal(|ui| {
                        ui.label("Link ID:");
                        ui.text_edit_singleline(&mut state.link_name);
                        ui.checkbox(&mut state.link_grounded, "1-Joint Grounded Link");
                    });
                    ui.horizontal(|ui| {
                        ui.label("Joint I:");
                        egui::ComboBox::from_id_salt("wf_link_pi").selected_text(&state.link_point_i).show_ui(ui,|ui| {
                            for pid in self.model.points.keys() {ui.selectable_value(&mut state.link_point_i, pid.clone(), pid);}
                        });
                        if !state.link_grounded {
                            ui.label("Joint J:");
                            egui::ComboBox::from_id_salt("wf_link_pj").selected_text(&state.link_point_j).show_ui(ui,|ui| {
                                for pid in self.model.points.keys() {ui.selectable_value(&mut state.link_point_j, pid.clone(), pid);}
                            });
                        }
                        ui.label("Roll angle (deg):");
                        ui.add(egui::DragValue::new(&mut state.link_angle_deg).speed(1.0));
                    });
                    if ui.button("Save Link Element").clicked() {
                        let link = LinkObj {
                            name: state.link_name.clone(),
                            point_i: state.link_point_i.clone(),
                            point_j: if state.link_grounded { None } else { Some(state.link_point_j.clone()) },
                            prop: state.link_prop.name.clone(),
                            angle_deg: state.link_angle_deg,
                            selected: false,
                        };
                        let lname = link.name.clone();
                        outcome = Some(sap_api::set_link_object(&mut self.model, link).map(|_| format!("Saved link element {lname}.")));
                    }
                }
            }
            if let Some(result)=outcome {match result {Ok(message)=>{self.status_msg=message;self.draw_first_point=None;self.draw_area_points.clear();self.inspected_frame=None;self.sync_active_case();},Err(error)=>self.status_msg=format!("Edit error: {error}")}}
            ui.separator();ui.label(&self.status_msg);
        });
        }
        state.open = open;
        self.workflows = state;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sap_format::{parse_s2k, write_s2k};
    fn model() -> SapModel {
        let mut m = SapModel::new(Units::KipInF);
        m.add_point("1", 0.0, 0.0, 0.0);
        m.add_point("2", 120.0, 0.0, 0.0);
        m.add_point("3", 120.0, 120.0, 0.0);
        m.add_point("4", 0.0, 120.0, 0.0);
        m.add_frame("F", "1", "2", "FSEC1");
        m.frames["F"].selected = true;
        m.add_area(
            "A",
            vec!["1".into(), "2".into(), "3".into(), "4".into()],
            "ASEC1",
        );
        m.areas["A"].selected = true;
        m.points["2"].selected = true;
        m
    }
    #[test]
    fn empty_metric_model_and_type_switch_use_scaled_material_defaults() {
        let mut m = SapModel::new(Units::KnMC);
        m.materials.clear();
        let mut state = Workflows::default();
        initialize_material(&mut state, &m);
        assert!((state.material.e - 199947961.501).abs() < 1.0);
        state.material.name = "NEW".into();
        change_material_type(&mut state, &m, MatType::Concrete);
        assert_eq!(state.material.name, "NEW");
        assert!((state.material.e - 24821126.255).abs() < 1.0);
        assert!((state.material.unit_weight - 23.563).abs() < 0.01);
        save_material(&mut m, &state.material).unwrap();
        m.convert_units_in_place(Units::KipInF);
        initialize_material(&mut state, &m);
        assert!((state.material.e - 3600.0).abs() < 1e-9);
        assert!((state.material.fc - 4.0).abs() < 1e-9);
    }
    #[test]
    fn material_edit_roundtrips_and_rejects_invalid_values_atomically() {
        let mut m = model();
        let mut material = m.materials.values().next().unwrap().clone();
        material.name = "CUSTOM".into();
        material.e = 31000.0;
        material.u = 0.25;
        material.fy = 55.0;
        material.fu = 75.0;
        m.case_results.insert("old".into(), Default::default());
        save_material(&mut m, &material).unwrap();
        assert!(m.case_results.is_empty());
        assert_eq!(m.materials["CUSTOM"].g, 12400.0);
        let restored = parse_s2k(&write_s2k(&m)).unwrap();
        assert_eq!(restored.materials["CUSTOM"], m.materials["CUSTOM"]);
        m.case_results.insert("old".into(), Default::default());
        let before = m.clone();
        material.u = 0.5;
        assert!(save_material(&mut m, &material).is_err());
        assert_eq!(m, before);
        material.u = 0.25;
        material.fu = 40.0;
        assert!(save_material(&mut m, &material).is_err());
        assert_eq!(m, before);
        assert!(assign_frame_section(&mut m, "missing").is_err());
        assert_eq!(m, before);
        let mut section = m.frame_sections["FSEC1"].clone();
        section.name = "CUSTOM_SEC".into();
        section.material = "CUSTOM".into();
        m.frame_sections.insert(section.name.clone(), section);
        assign_frame_section(&mut m, "CUSTOM_SEC").unwrap();
        assert_eq!(m.frames["F"].section, "CUSTOM_SEC");
        assert!(m.case_results.is_empty());
    }
    #[test]
    fn assignments_and_definitions_survive_text_save_and_reopen() {
        let mut m = model();
        define_pattern(&mut m, "LIVE", LoadPatternType::Live, 0.0).unwrap();
        assign_load(
            &mut m,
            "LIVE",
            LoadAssignment::Joint([1.0, 2.0, -3.0, 4.0, 5.0, 6.0]),
            true,
        )
        .unwrap();
        assign_load(&mut m, "LIVE", LoadAssignment::FrameDir(6, -0.2), true).unwrap();
        assign_load(&mut m, "LIVE", LoadAssignment::Pressure(-2, 0.005), true).unwrap();
        let mut section = m.area_sections["ASEC1"].clone();
        section.name = "ROOF".into();
        section.modifiers[1] = 0.0;
        section.thickness_membrane = 5.0;
        section.thickness_bending = 5.0;
        save_area(&mut m, &section).unwrap();
        assign_area(&mut m, "ROOF", [3, 4]).unwrap();
        let releases = [false, false, false, false, false, true];
        assign_releases(&mut m, releases, releases).unwrap();
        define_combo(
            &mut m,
            "ULS",
            ComboType::LinearAdd,
            &[("DEAD".into(), 1.2), ("LIVE".into(), 1.6)],
        )
        .unwrap();
        let restored = parse_s2k(&write_s2k(&m)).unwrap();
        assert_eq!(restored.load_patterns["LIVE"], m.load_patterns["LIVE"]);
        assert_eq!(restored.load_cases["LIVE"], m.load_cases["LIVE"]);
        assert_eq!(restored.load_combos["ULS"], m.load_combos["ULS"]);
        assert_eq!(restored.points["2"].loads_force, m.points["2"].loads_force);
        assert_eq!(restored.frames["F"].loads_dist, m.frames["F"].loads_dist);
        assert_eq!(restored.frames["F"].release_i, releases);
        assert_eq!(restored.area_sections["ROOF"], section);
        assert_eq!(restored.areas["A"].auto_mesh, (3, 4));
        assert_eq!(
            restored.areas["A"].loads_pressure,
            m.areas["A"].loads_pressure
        );
    }
    #[test]
    fn invalid_or_locked_edits_preserve_inputs_and_prior_results() {
        let mut m = model();
        m.case_results.insert("old".into(), Default::default());
        let before = m.clone();
        assert!(assign_load(&mut m, "MISSING", LoadAssignment::FrameDir(6, 2.0), true).is_err());
        assert!(assign_load(&mut m, "DEAD", LoadAssignment::Joint([f64::NAN; 6]), true).is_err());
        assert!(assign_area(&mut m, "ASEC1", [0, 1]).is_err());
        assert!(define_pattern(&mut m, "BAD", LoadPatternType::Dead, f64::NAN).is_err());
        assert!(define_pattern(&mut m, "MODAL", LoadPatternType::Live, 0.0).is_err());
        assert_eq!(m, before);
        m.is_locked = true;
        let before = m.clone();
        assert!(define_combo(&mut m, "C", ComboType::LinearAdd, &[("DEAD".into(), 1.0)]).is_err());
        assert!(assign_releases(&mut m, [true; 6], [true; 6]).is_err());
        assert_eq!(m, before);
        m.is_locked = false;
        define_combo(&mut m, "A", ComboType::LinearAdd, &[("DEAD".into(), 1.0)]).unwrap();
        define_combo(&mut m, "B", ComboType::LinearAdd, &[("A".into(), 1.0)]).unwrap();
        let before = m.clone();
        assert!(define_combo(&mut m, "A", ComboType::LinearAdd, &[("B".into(), 1.0)]).is_err());
        assert_eq!(m, before);
    }
    #[test]
    fn replace_loads_is_scoped_and_success_clears_results() {
        let mut m = model();
        define_pattern(&mut m, "LIVE", LoadPatternType::Live, 0.0).unwrap();
        assign_load(&mut m, "DEAD", LoadAssignment::FrameDir(6, -1.0), false).unwrap();
        assign_load(&mut m, "LIVE", LoadAssignment::FrameDir(6, -2.0), false).unwrap();
        m.case_results.insert("old".into(), Default::default());
        assign_load(&mut m, "LIVE", LoadAssignment::FrameDir(6, -3.0), true).unwrap();
        assert_eq!(m.frames["F"].loads_dist.len(), 2);
        assert_eq!(m.frames["F"].loads_dist[0].load_pat, "DEAD");
        assert_eq!(m.frames["F"].loads_dist[1].val1, -3.0);
        assert!(m.case_results.is_empty());
    }
    #[test]
    fn workflow_window_renders_all_tabs_and_click_creates_pattern() {
        let mut app = SapGuiApp::new(Some(PathBuf::from("/nonexistent/workflow-test.s2k")));
        app.model = model();
        app.workflows.open = true;
        let ctx = egui::Context::default();
        let input = |time| egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1200.0, 1000.0))),
            time: Some(time),
            ..Default::default()
        };
        for tab in 0..7 {
            app.workflows.tab = tab;
            let _ = ctx.run(input(tab as f64 + 1.0), |ctx| app.render_workflows(ctx));
            let out = ctx.run(input(tab as f64 + 1.5), |ctx| app.render_workflows(ctx));
            assert!(!out.shapes.is_empty());
        }
        app.workflows.tab = 1;
        app.workflows.pattern = "WIND".into();
        app.workflows.pattern_type = LoadPatternType::Wind;
        let _ = ctx.run(input(7.0), |ctx| app.render_workflows(ctx));
        let out = ctx.run(input(8.0), |ctx| app.render_workflows(ctx));
        let pos = out
            .shapes
            .iter()
            .find_map(|shape| {
                if let egui::epaint::Shape::Text(text) = &shape.shape {
                    if text.galley.job.text == "Save Pattern / Create Case if Missing" {
                        return Some(text.pos + text.galley.size() * 0.5);
                    }
                }
                None
            })
            .expect("save-pattern button must be visible");
        let mut down = input(9.0);
        down.events = vec![
            egui::Event::PointerMoved(pos),
            egui::Event::PointerButton {
                pos,
                button: egui::PointerButton::Primary,
                pressed: true,
                modifiers: Default::default(),
            },
        ];
        let _ = ctx.run(down, |ctx| app.render_workflows(ctx));
        let mut up = input(9.1);
        up.events = vec![egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed: false,
            modifiers: Default::default(),
        }];
        let _ = ctx.run(up, |ctx| app.render_workflows(ctx));
        assert_eq!(
            app.model.load_patterns["WIND"].pat_type,
            LoadPatternType::Wind
        );
        assert!(app.model.load_cases.contains_key("WIND"));
    }
}
