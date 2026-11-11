//! Interactive StructuraX Desktop CAD GUI (`eframe`/`egui`), providing a 3D/2D structural
//! engineering workspace with ribbon/menus, Model Explorer, synchronized 3D + 2D viewports,
//! Parametric Structural Templates wizard, `.pro` section database importer, interactive
//! member diagram inspector, Database Tables viewer, and Verification Benchmark Suite runner.

mod workflows;
mod analysis;

use eframe::egui::{self, Color32, FontId, Pos2, Rect, RichText, Sense, Shape, Stroke, Vec2};
use sap_design::{try_run_concrete_design, try_run_steel_design};
use sap_format::{
    load_model_with_report, save_native_sdb_file, save_s2k_file_with_report,
    save_sdb_file_with_report,
};
use sap_model::{
    CaseResults, ConcreteDesignResult, ConcreteRebarConfig, FrameLoadDistributed, FrameLoadPoint,
    FrameSection, FrameType2D, FrameType3D, LoadCase, LoadPattern, LoadPatternType, MatType,
    Material, PointLoadForce, SapModel, SteelDesignResult,
};
use sap_section::{
    make_box_tube, make_circle, make_i_section, make_pipe, make_rectangular, ProDatabase,
};
use sap_solver::{
    analyze_model,
    benchmarks::{run_verification_suite, BenchmarkVerificationRow},
    frame::frame_local_axes,
};
use sap_units::Units;
use std::collections::BTreeMap;
use std::path::PathBuf;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ToolMode {
    Select,
    DrawPoint,
    DrawFrame,
    DrawArea,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DisplayMode {
    Undeformed,
    Deformed,
    Reactions,
    FrameDiagram(FrameForceComponent),
    ShellContour(ShellComponent),
    SolidContour(SolidComponent),
    DesignRatioSteel,
    DesignRatioConcrete,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameForceComponent {
    AxialP,
    ShearV2,
    ShearV3,
    TorsionT,
    MomentM2,
    MomentM3,
}

impl FrameForceComponent {
    pub fn label(self) -> &'static str {
        match self {
            Self::AxialP => "Axial Force P",
            Self::ShearV2 => "Shear V2",
            Self::ShearV3 => "Shear V3",
            Self::TorsionT => "Torsion T",
            Self::MomentM2 => "Moment M2 (Minor)",
            Self::MomentM3 => "Moment M3 (Major)",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShellComponent {
    F11,
    F22,
    M11,
    M22,
    SvmTop,
}

impl ShellComponent {
    pub fn label(self) -> &'static str {
        match self {
            Self::F11 => "Membrane F11",
            Self::F22 => "Membrane F22",
            Self::M11 => "Plate Moment M11",
            Self::M22 => "Plate Moment M22",
            Self::SvmTop => "Von Mises Stress SVM",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SolidComponent {
    S11,
    S22,
    S33,
    VonMises,
}

impl SolidComponent {
    pub fn label(self) -> &'static str {
        match self {
            Self::S11 => "Normal Stress S11",
            Self::S22 => "Normal Stress S22",
            Self::S33 => "Normal Stress S33",
            Self::VonMises => "Von Mises Stress SVM",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CustomSectionShape {
    WideFlangeI,
    Rectangular,
    Circle,
    Pipe,
    BoxTube,
}

impl CustomSectionShape {
    pub fn label(self) -> &'static str {
        match self {
            Self::WideFlangeI => "Wide-Flange (I)",
            Self::Rectangular => "Rectangular (Solid)",
            Self::Circle => "Circular (Solid)",
            Self::Pipe => "Circular Pipe (Hollow)",
            Self::BoxTube => "Rectangular Box / HSS",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TableTab {
    JointDisplacements,
    JointReactions,
    BaseReactions,
    FrameForces,
    AreaResults,
    SolidStresses,
    LinkForces,
    ModalPeriods,
    BucklingFactors,
    SteelDesign,
    ConcreteDesign,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ViewProjection {
    Perspective3D,
    PlanXY,
    ElevationXZ,
    ElevationYZ,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TemplateChoice {
    Beam,
    Frame2D,
    Truss2D,
    Frame3D,
    ShearWall,
    SphericalDome,
}

pub struct SapGuiApp {
    pub model: SapModel,
    pub status_msg: String,
    pub import_warnings: Vec<String>,
    pub export_warnings: Vec<String>,
    pub tool_mode: ToolMode,
    pub display_mode: DisplayMode,
    pub active_case: String,
    pub active_mode_idx: usize,
    pub active_step_idx: usize,
    pub use_time_step: bool,
    pub deform_scale: f32,
    pub animate_deformed: bool,
    pub anim_phase: f32,
    pub diagram_scale: f32,

    // Viewports
    pub dual_pane: bool,
    pub left_proj: ViewProjection,
    pub right_proj: ViewProjection,
    pub yaw_rad: f32,
    pub pitch_rad: f32,
    pub zoom: f32,
    pub pan: Vec2,

    // Display options
    pub show_grids: bool,
    pub show_joint_labels: bool,
    pub show_frame_labels: bool,
    pub show_area_labels: bool,
    pub show_solid_labels: bool,
    pub show_link_labels: bool,
    pub show_restraints: bool,
    pub show_loads: bool,
    pub show_extruded: bool,

    // Interactive drawing state
    pub draw_first_point: Option<String>,
    pub draw_area_points: Vec<String>,
    pub new_pt_coords: [f64; 3],

    // Inspector & Dialogs
    pub inspected_frame: Option<String>,
    pub show_new_template_modal: bool,
    pub show_tables_modal: bool,
    pub tables_tab: TableTab,
    pub show_verification_modal: bool,
    pub show_section_modal: bool,
    pub show_assign_load_modal: bool,
    pub show_file_modal: bool,
    pub file_path_input: String,

    // Template dialog parameters
    pub tpl_choice: TemplateChoice,
    pub tpl_frame2d_type: FrameType2D,
    pub tpl_frame3d_type: FrameType3D,
    pub tpl_num_spans_or_bays: usize,
    pub tpl_span_or_bay_len: f64,
    pub tpl_num_stories: usize,
    pub tpl_story_height: f64,
    pub tpl_num_bays_y: usize,
    pub tpl_bay_width_y: f64,
    pub tpl_restraint: bool,

    // Section dialog parameters
    pub pro_file_name: String,
    pub pro_filter: String,
    pub pro_available_names: Vec<String>,
    pub custom_sec_shape: CustomSectionShape,
    pub custom_sec_material: String,
    pub custom_sec_name: String,
    pub custom_sec_d: f64,
    pub custom_sec_b: f64,
    pub custom_sec_tf: f64,
    pub custom_sec_tw: f64,
    pub custom_sec_rebar_enabled: bool,
    pub custom_sec_is_column: bool,
    pub custom_sec_cover: f64,
    pub custom_sec_rebar_area: f64,

    workflows: workflows::Workflows,
    analysis_editor: analysis::AnalysisEditor,

    // Verification suite cache
    pub verification_rows: Vec<BenchmarkVerificationRow>,
}

impl SapGuiApp {
    pub fn new(initial_file: Option<PathBuf>) -> Self {
        let mut model = SapModel::new(Units::KipInF);
        // Load W14X90 and W18X50 from aisc14.pro if available, then generate a default 2-story 2-bay 3D frame
        let pro_path = resolve_pro_database_path("aisc14.pro");
        let mut pro_names = Vec::new();
        if let Some(db) = pro_path.and_then(|path| ProDatabase::from_file(path).ok()) {
            pro_names = db.sections.iter().map(|s| s.name.clone()).collect();
            if let Ok(w14) = db.get_section_scaled("W14X90", Units::KipInF, "A992Fy50") {
                model.frame_sections.insert("W14X90".to_string(), w14);
            }
            if let Ok(w18) = db.get_section_scaled("W18X50", Units::KipInF, "A992Fy50") {
                model.frame_sections.insert("W18X50".to_string(), w18);
            }
        }
        let mut import_warnings = Vec::new();
        let mut status_msg =
            "Ready — Default 3D Building Frame loaded (Press F5 or click 'Run Analysis')"
                .to_string();

        if let Some(path) = initial_file {
            match load_model_with_report(&path) {
                Ok(loaded) => {
                    status_msg = format!("Loaded model from {}", path.display());
                    if !loaded.warnings.is_empty() {
                        status_msg.push_str(&format!(
                            " — Import warnings: {}",
                            loaded.warnings.join("; ")
                        ));
                    }
                    import_warnings = loaded.warnings;
                    model = loaded.model;
                }
                Err(error) => {
                    status_msg = format!("Cannot open {}: {error}", path.display());
                }
            }
        } else {
            model.generate_3d_frame(
                FrameType3D::OpenFrame,
                2,
                144.0,
                2,
                240.0,
                1,
                240.0,
                true,
                "W18X50",
                "W14X90",
                "ASEC1",
            );
            // Add default gravity distributed load for instant analysis & design
            for f in model.frames.values_mut() {
                let pi = &model.points[&f.point_i];
                let pj = &model.points[&f.point_j];
                if (pi.z - pj.z).abs() < 1e-3 {
                    f.loads_dist.push(FrameLoadDistributed {
                        load_pat: "DEAD".to_string(),
                        my_type: 1,
                        dir: 6,
                        csys: "GLOBAL".to_string(),
                        rel_dist1: 0.0,
                        rel_dist2: 1.0,
                        val1: -0.10,
                        val2: -0.10,
                    });
                }
            }
            match analyze_model(&mut model) {
                Ok(()) => {
                    if let Err(error) = try_run_steel_design(&mut model) {
                        status_msg = format!("Default model analyzed; design unavailable: {error}");
                    }
                }
                Err(error) => status_msg = format!("Default model analysis failed: {error}"),
            }
        }

        let active_case = model
            .load_cases
            .keys()
            .chain(model.load_combos.keys())
            .next()
            .cloned()
            .unwrap_or_else(|| "DEAD".to_string());
        let default_mat = model
            .materials
            .keys()
            .next()
            .cloned()
            .unwrap_or_else(|| "A992Fy50".to_string());

        let mut app = Self {
            model,
            status_msg,
            import_warnings,
            export_warnings: Vec::new(),
            tool_mode: ToolMode::Select,
            display_mode: DisplayMode::FrameDiagram(FrameForceComponent::MomentM3),
            active_case,
            active_mode_idx: 0,
            active_step_idx: 0,
            use_time_step: false,
            deform_scale: 25.0,
            animate_deformed: false,
            anim_phase: 0.0,
            diagram_scale: 1.0,
            dual_pane: true,
            left_proj: ViewProjection::Perspective3D,
            right_proj: ViewProjection::ElevationXZ,
            yaw_rad: -0.65,
            pitch_rad: 0.42,
            zoom: 1.0,
            pan: Vec2::ZERO,
            show_grids: true,
            show_joint_labels: true,
            show_frame_labels: false,
            show_area_labels: false,
            show_solid_labels: false,
            show_link_labels: true,
            show_restraints: true,
            show_loads: true,
            show_extruded: false,
            draw_first_point: None,
            draw_area_points: Vec::new(),
            new_pt_coords: [0.0, 0.0, 0.0],
            inspected_frame: None,
            show_new_template_modal: false,
            show_tables_modal: false,
            tables_tab: TableTab::JointDisplacements,
            show_verification_modal: false,
            show_section_modal: false,
            show_assign_load_modal: false,
            show_file_modal: false,
            file_path_input: String::new(),
            tpl_choice: TemplateChoice::Frame2D,
            tpl_frame2d_type: FrameType2D::PortalFrame,
            tpl_frame3d_type: FrameType3D::OpenFrame,
            tpl_num_spans_or_bays: 3,
            tpl_span_or_bay_len: 240.0,
            tpl_num_stories: 3,
            tpl_story_height: 144.0,
            tpl_num_bays_y: 2,
            tpl_bay_width_y: 240.0,
            tpl_restraint: true,
            pro_file_name: "aisc14.pro".to_string(),
            pro_filter: "W14".to_string(),
            pro_available_names: pro_names,
            custom_sec_shape: CustomSectionShape::WideFlangeI,
            custom_sec_material: default_mat,
            custom_sec_name: "W24X76_CUSTOM".to_string(),
            custom_sec_d: 24.0,
            custom_sec_b: 9.0,
            custom_sec_tf: 0.68,
            custom_sec_tw: 0.44,
            custom_sec_rebar_enabled: false,
            custom_sec_is_column: false,
            custom_sec_cover: 2.5,
            custom_sec_rebar_area: 4.0,
            workflows: workflows::Workflows::default(),
            analysis_editor: analysis::AnalysisEditor::default(),
            verification_rows: Vec::new(),
        };
        if app.model.present_units != Units::KipInF {
            app.scale_gui_cached_units(Units::KipInF, app.model.present_units);
        }
        app
    }

    pub(super) fn sync_active_case(&mut self) {
        if !self.model.load_cases.contains_key(&self.active_case)
            && !self.model.load_combos.contains_key(&self.active_case)
        {
            self.active_case = self
                .model
                .load_cases
                .keys()
                .chain(self.model.load_combos.keys())
                .next()
                .cloned()
                .unwrap_or_else(|| "DEAD".to_string());
        }
        if let Some(cr) = self.model.case_results.get(&self.active_case) {
            if !cr.modes.is_empty() {
                self.active_mode_idx = self.active_mode_idx.min(cr.modes.len() - 1);
            } else {
                self.active_mode_idx = 0;
            }
            if !cr.time_steps.is_empty() {
                self.active_step_idx = self.active_step_idx.min(cr.time_steps.len() - 1);
            } else {
                self.active_step_idx = 0;
                self.use_time_step = false;
            }
        } else {
            self.active_mode_idx = 0;
            self.active_step_idx = 0;
            self.use_time_step = false;
        }
        if let Some(fid) = &self.inspected_frame {
            if !self.model.frames.contains_key(fid) {
                self.inspected_frame = None;
            }
        }
        if !self.model.materials.contains_key(&self.custom_sec_material) {
            if let Some(first_mat) = self.model.materials.keys().next().cloned() {
                self.custom_sec_material = first_mat;
            }
        }
    }

    fn replace_model(
        &mut self,
        new_model: SapModel,
        status: String,
        import_warnings: Vec<String>,
    ) {
        let old_u = self.model.present_units;
        let new_u = new_model.present_units;
        self.model = new_model;
        if old_u != new_u {
            self.scale_gui_cached_units(old_u, new_u);
            self.workflows.scale_for_units(old_u, new_u, &self.model);
        } else {
            self.workflows = workflows::Workflows::default();
            self.workflows
                .scale_for_units(Units::KipInF, new_u, &self.model);
        }
        self.analysis_editor = analysis::AnalysisEditor::default();
        self.import_warnings = import_warnings;
        self.export_warnings.clear();
        self.draw_first_point = None;
        self.draw_area_points.clear();
        self.inspected_frame = None;
        self.active_mode_idx = 0;
        self.active_step_idx = 0;
        self.use_time_step = false;
        self.status_msg = status;
        self.sync_active_case();
    }

    fn scale_gui_cached_units(&mut self, from: Units, to: Units) {
        if from == to {
            return;
        }
        let fl = from.length_factor_to(to);
        self.tpl_span_or_bay_len *= fl;
        self.tpl_story_height *= fl;
        self.tpl_bay_width_y *= fl;
        for c in &mut self.new_pt_coords {
            *c *= fl;
        }
        self.custom_sec_d *= fl;
        self.custom_sec_b *= fl;
        self.custom_sec_tf *= fl;
        self.custom_sec_tw *= fl;
        self.custom_sec_cover *= fl;
        self.custom_sec_rebar_area *= fl * fl;
    }

    fn ensure_unlocked(&mut self) -> bool {
        if self.model.is_locked {
            self.status_msg =
                "Model is locked. Click 'Edit > Unlock Model' before modifying the model."
                    .to_string();
            false
        } else {
            true
        }
    }

    fn invalidate_results(&mut self) {
        self.model.is_locked = false;
        self.model.case_results.clear();
        self.model.steel_design_results.clear();
        self.model.concrete_design_results.clear();
    }

    fn delete_selected_objects(&mut self) {
        self.invalidate_results();
        self.model.points.retain(|_, point| !point.selected);
        let points = &self.model.points;
        self.model.frames.retain(|_, frame| {
            !frame.selected
                && points.contains_key(&frame.point_i)
                && points.contains_key(&frame.point_j)
        });
        self.model.areas.retain(|_, area| {
            !area.selected && area.points.iter().all(|point| points.contains_key(point))
        });
        self.model.solids.retain(|_, solid| {
            !solid.selected && solid.points.iter().all(|point| points.contains_key(point))
        });
        self.model.links.retain(|_, link| {
            !link.selected
                && points.contains_key(&link.point_i)
                && link
                    .point_j
                    .as_ref()
                    .is_none_or(|point| points.contains_key(point))
        });
        self.draw_first_point = None;
        self.draw_area_points.clear();
        self.inspected_frame = None;
        self.status_msg =
            "Deleted selected objects and elements attached to deleted joints; results cleared."
                .into();
    }

    fn execute_analysis(&mut self) -> bool {
        match analyze_model(&mut self.model) {
            Ok(()) => {
                self.sync_active_case();
                self.status_msg = format!(
                    "Analysis Complete: {} joints, {} frames, {} areas, {} solids, {} links solved across {} results.",
                    self.model.points.len(),
                    self.model.frames.len(),
                    self.model.areas.len(),
                    self.model.solids.len(),
                    self.model.links.len(),
                    self.model.case_results.len()
                );
                true
            }
            Err(e) => {
                self.status_msg = format!("Analysis Error: {e}");
                false
            }
        }
    }

    fn execute_design(&mut self, steel: bool) {
        // Re-analyze the current inputs: an edited model must not reuse old forces.
        if !self.execute_analysis() {
            return;
        }
        if steel {
            if let Err(error) = try_run_steel_design(&mut self.model) {
                self.status_msg = format!("Design Error: {error}");
                return;
            }
            self.display_mode = DisplayMode::DesignRatioSteel;
            let count = self.model.steel_design_results.len();
            let failed = self
                .model
                .steel_design_results
                .values()
                .filter(|r| r.status != "OK")
                .count();
            self.status_msg = if count == 0 {
                "No applicable steel members with analysis forces were available for design."
                    .to_string()
            } else {
                format!(
                    "Steel design ({}) complete: {count} members ({} OK, {failed} failed).",
                    self.model.steel_design_code,
                    count - failed
                )
            };
        } else {
            if let Err(error) = try_run_concrete_design(&mut self.model) {
                self.status_msg = format!("Design Error: {error}");
                return;
            }
            self.display_mode = DisplayMode::DesignRatioConcrete;
            let count = self.model.concrete_design_results.len();
            let failed = self
                .model
                .concrete_design_results
                .values()
                .filter(|r| r.status != "OK")
                .count();
            self.status_msg = if count == 0 {
                "No applicable concrete members with analysis forces were available for design."
                    .to_string()
            } else {
                format!(
                    "Concrete design ({}) complete: {count} members ({} OK, {failed} failed).",
                    self.model.concrete_design_code,
                    count - failed
                )
            };
        }
    }

    fn assign_restraint_to_selected(&mut self, restraint: [bool; 6], label: &str) {
        if !self.ensure_unlocked() {
            return;
        }
        let count = self.model.points.values().filter(|p| p.selected).count();
        if count == 0 {
            self.status_msg =
                "Select at least one joint before assigning support restraints.".to_string();
            return;
        }
        self.invalidate_results();
        for p in self.model.points.values_mut().filter(|p| p.selected) {
            p.restraint = restraint;
        }
        self.status_msg = format!("Assigned {label} restraint to {count} joints.");
    }

    fn render_top_menu_and_ribbon(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        // Keyboard shortcuts (ignored while typing in text fields)
        if !ctx.wants_keyboard_input() {
            let (f4, f5, f6, f7, del, esc, ctrl_n, ctrl_o, ctrl_t) = ctx.input(|i| {
                let cmd = i.modifiers.command || i.modifiers.ctrl;
                (
                    i.key_pressed(egui::Key::F4),
                    i.key_pressed(egui::Key::F5),
                    i.key_pressed(egui::Key::F6),
                    i.key_pressed(egui::Key::F7),
                    i.key_pressed(egui::Key::Delete),
                    i.key_pressed(egui::Key::Escape),
                    cmd && i.key_pressed(egui::Key::N),
                    cmd && (i.key_pressed(egui::Key::O) || i.key_pressed(egui::Key::S)),
                    cmd && i.key_pressed(egui::Key::T),
                )
            });
            if f5 {
                self.execute_analysis();
            }
            if f4 {
                self.display_mode = DisplayMode::Undeformed;
                self.animate_deformed = false;
            }
            if f6 {
                self.display_mode = DisplayMode::Deformed;
            }
            if f7 {
                self.display_mode = DisplayMode::Reactions;
            }
            if del && self.ensure_unlocked() {
                let any_selected = self.model.points.values().any(|p| p.selected)
                    || self.model.frames.values().any(|f| f.selected)
                    || self.model.areas.values().any(|a| a.selected)
                    || self.model.solids.values().any(|s| s.selected)
                    || self.model.links.values().any(|l| l.selected);
                if any_selected {
                    self.delete_selected_objects();
                } else {
                    self.status_msg = "No selected objects to delete.".to_string();
                }
            }
            if esc {
                self.draw_first_point = None;
                self.draw_area_points.clear();
                self.tool_mode = ToolMode::Select;
            }
            if ctrl_n {
                self.show_new_template_modal = true;
            }
            if ctrl_o {
                self.show_file_modal = true;
            }
            if ctrl_t {
                self.show_tables_modal = true;
            }
        }

        // 1. Classic SAP2000 v19 Menu Bar
        egui::menu::bar(ui, |ui| {
            ui.menu_button("File", |ui| {
                if ui.button("New Model from Templates... (Ctrl+N)").clicked() {
                    self.show_new_template_modal = true;
                    ui.close_menu();
                }
                if ui.button("New Blank Model").clicked() {
                    let u = self.model.present_units;
                    self.replace_model(
                        SapModel::new(u),
                        "Initialized blank model.".to_string(),
                        Vec::new(),
                    );
                    ui.close_menu();
                }
                ui.separator();
                if ui.button("Open / Save Model (.s2k / .sdb)... (Ctrl+O)").clicked() {
                    self.show_file_modal = true;
                    ui.close_menu();
                }
                ui.menu_button("Load Verification Benchmark", |ui| {
                    if ui
                        .button("Example 1-001: 2D Frame (7 Load Cases)")
                        .clicked()
                    {
                        self.load_benchmark_1_001();
                        ui.close_menu();
                    }
                    if ui
                        .button("Example 1-004: Rotated Local Axes (30 deg)")
                        .clicked()
                    {
                        self.load_benchmark_1_004();
                        ui.close_menu();
                    }
                    if ui
                        .button("Example 1-016: P-Delta Tension Stiffening")
                        .clicked()
                    {
                        self.load_benchmark_1_016();
                        ui.close_menu();
                    }
                });
            });

            ui.menu_button("Edit", |ui| {
                if ui.button("Move / Replicate / Divide Selected...").clicked() {
                    self.open_workflow(0);
                    ui.close_menu();
                }
                if ui.button("Select All Objects").clicked() {
                    for p in self.model.points.values_mut() { p.selected = true; }
                    for f in self.model.frames.values_mut() { f.selected = true; }
                    for a in self.model.areas.values_mut() { a.selected = true; }
                    for s in self.model.solids.values_mut() { s.selected = true; }
                    for l in self.model.links.values_mut() { l.selected = true; }
                    ui.close_menu();
                }
                if ui.button("Clear Selection").clicked() {
                    for p in self.model.points.values_mut() { p.selected = false; }
                    for f in self.model.frames.values_mut() { f.selected = false; }
                    for a in self.model.areas.values_mut() { a.selected = false; }
                    for s in self.model.solids.values_mut() { s.selected = false; }
                    for l in self.model.links.values_mut() { l.selected = false; }
                    ui.close_menu();
                }
                if ui.button("Delete Selected Objects (Del)").clicked() {
                    if self.ensure_unlocked() {
                        let any_selected = self.model.points.values().any(|p| p.selected)
                            || self.model.frames.values().any(|f| f.selected)
                            || self.model.areas.values().any(|a| a.selected)
                            || self.model.solids.values().any(|s| s.selected)
                            || self.model.links.values().any(|l| l.selected);
                        if any_selected {
                            self.delete_selected_objects();
                        } else {
                            self.status_msg = "No selected objects to delete.".to_string();
                        }
                    }
                    ui.close_menu();
                }
                if ui.button("Unlock Model").clicked() {
                    self.invalidate_results();
                    self.status_msg = "Model unlocked.".to_string();
                    ui.close_menu();
                }
            });

            ui.menu_button("View", |ui| {
                ui.checkbox(&mut self.dual_pane, "Dual-Pane Viewport (3D + 2D)");
                ui.separator();
                if ui.button("Set Left View: 3D Perspective").clicked() {
                    self.left_proj = ViewProjection::Perspective3D;
                    ui.close_menu();
                }
                if ui.button("Set Right View: XZ Elevation").clicked() {
                    self.right_proj = ViewProjection::ElevationXZ;
                    ui.close_menu();
                }
                if ui.button("Set Right View: XY Plan").clicked() {
                    self.right_proj = ViewProjection::PlanXY;
                    ui.close_menu();
                }
                if ui.button("Set Right View: YZ Elevation").clicked() {
                    self.right_proj = ViewProjection::ElevationYZ;
                    ui.close_menu();
                }
                ui.separator();
                ui.checkbox(&mut self.show_grids, "Show Grid Lines & Bubbles");
                ui.checkbox(&mut self.show_joint_labels, "Show Joint Labels");
                ui.checkbox(&mut self.show_frame_labels, "Show Frame Labels");
                ui.checkbox(&mut self.show_area_labels, "Show Area Labels");
                ui.checkbox(&mut self.show_solid_labels, "Show Solid Labels");
                ui.checkbox(&mut self.show_link_labels, "Show Link Labels");
                ui.checkbox(&mut self.show_restraints, "Show Support Restraints");
                ui.checkbox(&mut self.show_loads, "Show Applied Loads");
                ui.checkbox(&mut self.show_extruded, "Show Extruded Profile View");
                if ui.button("Reset Camera / Fit View").clicked() {
                    self.zoom = 1.0;
                    self.pan = Vec2::ZERO;
                    self.yaw_rad = -0.65;
                    self.pitch_rad = 0.42;
                    ui.close_menu();
                }
            });

            ui.menu_button("Define", |ui| {
                for (label, tab) in [("Analysis Cases...", 0), ("Dynamic Functions...", 1)] {
                    if ui.button(label).clicked() { self.analysis_editor.open=true; self.analysis_editor.tab=tab; ui.close_menu(); }
                }
                for (label, tab) in [
                    ("Materials...", 5),
                    ("Load Patterns...", 1),
                    ("Area Sections and Modifiers...", 2),
                    ("Link Properties & Elements...", 6),
                    ("Load Combinations...", 4),
                ] {
                    if ui.button(label).clicked() { self.open_workflow(tab); ui.close_menu(); }
                }
                if ui.button("Frame Sections & .PRO Library...").clicked() {
                    self.show_section_modal = true;
                    ui.close_menu();
                }
                if ui.button("Database Tables Explorer... (Ctrl+T)").clicked() {
                    self.show_tables_modal = true;
                    ui.close_menu();
                }
            });

            ui.menu_button("Assign", |ui| {
                for (label, tab) in [
                    ("Frame Section...", 0),
                    ("Area Section and Mesh...", 2),
                    ("Frame Releases / T-C Limits / Cable...", 3),
                    ("Link Elements...", 6),
                ] {
                    if ui.button(label).clicked() { self.open_workflow(tab); ui.close_menu(); }
                }
                if ui.button("Assign Joint / Frame / Area Loads...").clicked() {
                    self.show_assign_load_modal = true;
                    ui.close_menu();
                }
                ui.separator();
                if ui.button("Restraint: Fixed Support (All 6 DOFs)").clicked() {
                    self.assign_restraint_to_selected([true; 6], "Fixed");
                    ui.close_menu();
                }
                if ui
                    .button("Restraint: Pinned Support (UX, UY, UZ)")
                    .clicked()
                {
                    self.assign_restraint_to_selected(
                        [true, true, true, false, false, false],
                        "Pinned",
                    );
                    ui.close_menu();
                }
                if ui.button("Restraint: Roller Support (UZ only)").clicked() {
                    self.assign_restraint_to_selected(
                        [false, false, true, false, false, false],
                        "Roller",
                    );
                    ui.close_menu();
                }
                if ui.button("Restraint: Free (Release All)").clicked() {
                    self.assign_restraint_to_selected([false; 6], "Free");
                    ui.close_menu();
                }
            });

            ui.menu_button("Analyze", |ui| {
                if ui.button("▶ Run Analysis (F5)").clicked() {
                    self.execute_analysis();
                    ui.close_menu();
                }
                ui.separator();
                if ui.button("Set Active DOFs: 3D Space Frame").clicked() {
                    if self.ensure_unlocked() {
                        self.invalidate_results();
                        self.model.active_dofs = [true; 6];
                        self.status_msg = "Active DOFs set to 3D Space Frame.".to_string();
                    }
                    ui.close_menu();
                }
                if ui.button("Set Active DOFs: 2D XZ Plane Frame").clicked() {
                    if self.ensure_unlocked() {
                        self.invalidate_results();
                        self.model.active_dofs = [true, false, true, false, true, false];
                        self.status_msg = "Active DOFs set to 2D XZ Plane Frame.".to_string();
                    }
                    ui.close_menu();
                }
                if ui.button("Set Active DOFs: 2D XY Plane Grid").clicked() {
                    if self.ensure_unlocked() {
                        self.invalidate_results();
                        self.model.active_dofs = [false, false, true, true, true, false];
                        self.status_msg = "Active DOFs set to 2D XY Plane Grid.".to_string();
                    }
                    ui.close_menu();
                }
            });

            ui.menu_button("Display", |ui| {
                if ui.button("Show Undeformed Shape (F4)").clicked() {
                    self.display_mode = DisplayMode::Undeformed;
                    self.animate_deformed = false;
                    ui.close_menu();
                }
                if ui.button("Show Deformed Shape (F6)").clicked() {
                    self.display_mode = DisplayMode::Deformed;
                    ui.close_menu();
                }
                if ui.button("Show Support Reactions (F7)").clicked() {
                    self.display_mode = DisplayMode::Reactions;
                    ui.close_menu();
                }
                ui.separator();
                for comp in [
                    FrameForceComponent::AxialP,
                    FrameForceComponent::ShearV2,
                    FrameForceComponent::ShearV3,
                    FrameForceComponent::TorsionT,
                    FrameForceComponent::MomentM2,
                    FrameForceComponent::MomentM3,
                ] {
                    if ui
                        .button(format!("Frame Diagram: {}", comp.label()))
                        .clicked()
                    {
                        self.display_mode = DisplayMode::FrameDiagram(comp);
                        ui.close_menu();
                    }
                }
                ui.separator();
                for scomp in [
                    ShellComponent::F11,
                    ShellComponent::F22,
                    ShellComponent::M11,
                    ShellComponent::M22,
                    ShellComponent::SvmTop,
                ] {
                    if ui
                        .button(format!("Shell Contour: {}", scomp.label()))
                        .clicked()
                    {
                        self.display_mode = DisplayMode::ShellContour(scomp);
                        ui.close_menu();
                    }
                }
                ui.separator();
                for sol_comp in [
                    SolidComponent::S11,
                    SolidComponent::S22,
                    SolidComponent::S33,
                    SolidComponent::VonMises,
                ] {
                    if ui
                        .button(format!("Solid Contour: {}", sol_comp.label()))
                        .clicked()
                    {
                        self.display_mode = DisplayMode::SolidContour(sol_comp);
                        ui.close_menu();
                    }
                }
                ui.separator();
                if ui.button("Show Database Tables... (Ctrl+T)").clicked() {
                    self.show_tables_modal = true;
                    ui.close_menu();
                }
            });

            ui.menu_button("Design", |ui| {
                for code in ["AISC 360-10", "AISC 360-10 ASD", "Eurocode 3-2005"] {
                    if ui
                        .button(format!("▶ Start Steel Frame Design ({code})"))
                        .clicked()
                    {
                        self.model.steel_design_code = code.to_string();
                        self.execute_design(true);
                        ui.close_menu();
                    }
                }
                ui.separator();
                for code in ["ACI 318-14", "Eurocode 2-2004"] {
                    if ui
                        .button(format!("▶ Start Concrete Frame Design ({code})"))
                        .clicked()
                    {
                        self.model.concrete_design_code = code.to_string();
                        self.execute_design(false);
                        ui.close_menu();
                    }
                }
            });

            ui.menu_button("Help", |ui| {
                if ui
                    .button("✓ Run Structural Verification Benchmark Suite...")
                    .clicked()
                {
                    self.verification_rows = run_verification_suite();
                    self.show_verification_modal = true;
                    ui.close_menu();
                }
            });
        });

        ui.separator();

        // 2. Quick-Access StructuraX Ribbon Toolbar
        ui.horizontal_wrapped(|ui| {
            if ui.button("📄 New Template").clicked() {
                self.show_new_template_modal = true;
            }
            if ui.button("📂 Open / Save").clicked() {
                self.show_file_modal = true;
            }
            if ui.button("📐 Sections (.PRO)").clicked() {
                self.show_section_modal = true;
            }
            if ui.button("⬇ Assign Loads").clicked() {
                self.show_assign_load_modal = true;
            }
            ui.separator();
            if ui
                .button(
                    RichText::new("▶ Run Analysis")
                        .color(Color32::from_rgb(80, 220, 120))
                        .strong(),
                )
                .clicked()
            {
                self.execute_analysis();
            }
            if ui
                .button(RichText::new("🏗 Steel Design").color(Color32::from_rgb(100, 190, 255)))
                .clicked()
            {
                self.execute_design(true);
            }
            if ui
                .button(RichText::new("🧱 Concrete Design").color(Color32::from_rgb(240, 180, 90)))
                .clicked()
            {
                self.execute_design(false);
            }
            ui.separator();
            ui.selectable_value(
                &mut self.display_mode,
                DisplayMode::Undeformed,
                "Undeformed",
            );
            ui.selectable_value(&mut self.display_mode, DisplayMode::Deformed, "Deformed");
            ui.selectable_value(&mut self.display_mode, DisplayMode::Reactions, "Reactions");
            ui.selectable_value(
                &mut self.display_mode,
                DisplayMode::FrameDiagram(FrameForceComponent::AxialP),
                "P",
            );
            ui.selectable_value(
                &mut self.display_mode,
                DisplayMode::FrameDiagram(FrameForceComponent::ShearV2),
                "V2",
            );
            ui.selectable_value(
                &mut self.display_mode,
                DisplayMode::FrameDiagram(FrameForceComponent::MomentM3),
                "M3",
            );
            ui.selectable_value(
                &mut self.display_mode,
                DisplayMode::FrameDiagram(FrameForceComponent::TorsionT),
                "T",
            );
            ui.selectable_value(
                &mut self.display_mode,
                DisplayMode::DesignRatioSteel,
                "Steel D/C",
            );
            ui.selectable_value(
                &mut self.display_mode,
                DisplayMode::DesignRatioConcrete,
                "Conc Design",
            );
            ui.separator();
            if ui.button("📊 Tables").clicked() {
                self.show_tables_modal = true;
            }
            if ui.button("✓ Verify Suite").clicked() {
                self.verification_rows = run_verification_suite();
                self.show_verification_modal = true;
            }
        });
    }

    fn render_left_explorer(&mut self, ui: &mut egui::Ui) {
        self.sync_active_case();
        ui.heading("Model Explorer");
        if !self.import_warnings.is_empty() {
            ui.collapsing(
                format!("Import warnings ({})", self.import_warnings.len()),
                |ui| {
                    for warning in &self.import_warnings {
                        ui.label(warning);
                    }
                },
            );
        }
        if !self.export_warnings.is_empty() {
            ui.collapsing(format!("Export warnings ({})", self.export_warnings.len()), |ui| {
                for warning in &self.export_warnings {
                    ui.label(warning);
                }
            });
        }
        ui.label(RichText::new(&self.model.model_name).strong());
        ui.separator();

        // Drawing Tool Mode Selector
        ui.label(RichText::new("CAD Tool Mode:").strong());
        ui.horizontal_wrapped(|ui| {
            ui.selectable_value(&mut self.tool_mode, ToolMode::Select, "🖱 Select");
            ui.selectable_value(&mut self.tool_mode, ToolMode::DrawPoint, "• Joint");
            ui.selectable_value(&mut self.tool_mode, ToolMode::DrawFrame, "╱ Frame");
            ui.selectable_value(&mut self.tool_mode, ToolMode::DrawArea, "▭ Area");
        });
        if self.tool_mode == ToolMode::DrawPoint {
            ui.horizontal(|ui| {
                ui.label("X:");
                ui.add(egui::DragValue::new(&mut self.new_pt_coords[0]).speed(1.0));
                ui.label("Y:");
                ui.add(egui::DragValue::new(&mut self.new_pt_coords[1]).speed(1.0));
                ui.label("Z:");
                ui.add(egui::DragValue::new(&mut self.new_pt_coords[2]).speed(1.0));
            });
            if ui.button("+ Add Joint at (X,Y,Z)").clicked() && self.ensure_unlocked() {
                if self.new_pt_coords.iter().all(|v| v.is_finite()) {
                    let id = sap_model::next_numeric_name(self.model.points.keys());
                    self.invalidate_results();
                    self.model.add_point(
                        &id,
                        self.new_pt_coords[0],
                        self.new_pt_coords[1],
                        self.new_pt_coords[2],
                    );
                    self.status_msg = format!("Added Joint {id}");
                } else {
                    self.status_msg = "Joint coordinates must be finite.".to_string();
                }
            }
        }
        ui.separator();

        egui::ScrollArea::vertical().show(ui, |ui| {
            egui::CollapsingHeader::new(format!("Joints ({})", self.model.points.len()))
                .default_open(false)
                .show(ui, |ui| {
                    for (id, pt) in &mut self.model.points {
                        let lbl = format!("Jt {id}: ({:.1}, {:.1}, {:.1})", pt.x, pt.y, pt.z);
                        ui.checkbox(&mut pt.selected, lbl);
                    }
                });

            egui::CollapsingHeader::new(format!("Frames ({})", self.model.frames.len()))
                .default_open(true)
                .show(ui, |ui| {
                    for (id, fr) in &mut self.model.frames {
                        ui.horizontal(|ui| {
                            let tag = if fr.cable_params.is_some() {
                                "Cable"
                            } else {
                                "Fr"
                            };
                            ui.checkbox(
                                &mut fr.selected,
                                format!("{tag} {id}: {}->{} [{}]", fr.point_i, fr.point_j, fr.section),
                            );
                            if ui
                                .small_button("🔍")
                                .on_hover_text("Inspect Station Diagrams")
                                .clicked()
                            {
                                self.inspected_frame = Some(id.clone());
                            }
                        });
                    }
                });

            egui::CollapsingHeader::new(format!("Areas / Shells ({})", self.model.areas.len()))
                .default_open(false)
                .show(ui, |ui| {
                    for (id, ar) in &mut self.model.areas {
                        ui.checkbox(
                            &mut ar.selected,
                            format!("Area {id}: {:?} [{}]", ar.points, ar.section),
                        );
                    }
                });

            if !self.model.solids.is_empty() {
                egui::CollapsingHeader::new(format!("Solids ({})", self.model.solids.len()))
                    .default_open(false)
                    .show(ui, |ui| {
                        for (id, sol) in &mut self.model.solids {
                            ui.checkbox(
                                &mut sol.selected,
                                format!("Solid {id} [{}]", sol.section),
                            );
                        }
                    });
            }

            if !self.model.links.is_empty() {
                egui::CollapsingHeader::new(format!("Links ({})", self.model.links.len()))
                    .default_open(false)
                    .show(ui, |ui| {
                        for (id, lnk) in &mut self.model.links {
                            let end_j = lnk.point_j.as_deref().unwrap_or("Ground");
                            ui.checkbox(
                                &mut lnk.selected,
                                format!("Link {id}: {}->{} [{}]", lnk.point_i, end_j, lnk.prop),
                            );
                        }
                    });
            }

            egui::CollapsingHeader::new(format!(
                "Frame Sections ({})",
                self.model.frame_sections.len()
            ))
            .default_open(true)
            .show(ui, |ui| {
                for (name, sec) in &self.model.frame_sections {
                    ui.label(format!(
                        "• {name} ({:?}): A={:.2}, I33={:.1}",
                        sec.prop_type, sec.area, sec.i33
                    ));
                }
            });

            egui::CollapsingHeader::new(format!("Load Cases ({})", self.model.load_cases.len()))
                .default_open(true)
                .show(ui, |ui| {
                    for (name, lc) in &self.model.load_cases {
                        let is_act = &self.active_case == name;
                        let run_tag = if lc.run_flag { "" } else { " [Off]" };
                        if ui
                            .selectable_label(is_act, format!("{name} ({:?}){run_tag}", lc.case_type))
                            .clicked()
                        {
                            self.active_case = name.clone();
                            self.active_mode_idx = 0;
                            self.active_step_idx = 0;
                        }
                    }
                });

            if !self.model.load_combos.is_empty() {
                egui::CollapsingHeader::new(format!(
                    "Load Combinations ({})",
                    self.model.load_combos.len()
                ))
                .default_open(true)
                .show(ui, |ui| {
                    for (name, combo) in &self.model.load_combos {
                        let is_act = &self.active_case == name;
                        if ui
                            .selectable_label(
                                is_act,
                                format!("{name} ({:?})", combo.combo_type),
                            )
                            .clicked()
                        {
                            self.active_case = name.clone();
                            self.active_mode_idx = 0;
                            self.active_step_idx = 0;
                        }
                    }
                });
            }

            if let Some(cr) = self.model.case_results.get(&self.active_case) {
                if !cr.modes.is_empty() {
                    egui::CollapsingHeader::new(format!(
                        "Modal Periods ({} modes)",
                        cr.modes.len()
                    ))
                    .default_open(true)
                    .show(ui, |ui| {
                        for (idx, m) in cr.modes.iter().enumerate() {
                            let lbl = format!(
                                "Mode {}: T={:.4}s (f={:.2}Hz)",
                                m.mode_num, m.period_sec, m.freq_hz
                            );
                            if ui
                                .selectable_label(self.active_mode_idx == idx, lbl)
                                .clicked()
                            {
                                self.active_mode_idx = idx;
                                self.display_mode = DisplayMode::Deformed;
                            }
                        }
                    });
                }

                if !cr.buckling_factors.is_empty() {
                    egui::CollapsingHeader::new(format!(
                        "Buckling Factors ({} modes)",
                        cr.buckling_factors.len()
                    ))
                    .default_open(true)
                    .show(ui, |ui| {
                        for (idx, factor) in cr.buckling_factors.iter().enumerate() {
                            ui.label(format!("Mode {}: λ = {:.5}", idx + 1, factor));
                        }
                    });
                }

                if !cr.time_steps.is_empty() {
                    egui::CollapsingHeader::new(format!(
                        "Time-History Steps ({})",
                        cr.time_steps.len()
                    ))
                    .default_open(true)
                    .show(ui, |ui| {
                        if ui
                            .checkbox(&mut self.use_time_step, "Scrub Deformed Time Step")
                            .changed()
                            && self.use_time_step
                        {
                            self.display_mode = DisplayMode::Deformed;
                        }
                        let max_idx = cr.time_steps.len().saturating_sub(1);
                        let cur_idx = self.active_step_idx.min(max_idx);
                        let t_sec = cr.time_steps.get(cur_idx).copied().unwrap_or(0.0);
                        ui.label(format!("Step {cur_idx}/{max_idx}: t = {t_sec:.4} s"));
                        if max_idx > 0
                            && ui
                                .add(egui::Slider::new(&mut self.active_step_idx, 0..=max_idx))
                                .changed()
                        {
                            self.use_time_step = true;
                            self.display_mode = DisplayMode::Deformed;
                        }
                    });
                }
            }

            if !self.model.steel_design_results.is_empty() {
                egui::CollapsingHeader::new(format!(
                    "Steel Design ({})",
                    self.model.steel_design_code
                ))
                .default_open(true)
                .show(ui, |ui| {
                    for (fid, res) in &self.model.steel_design_results {
                        let col = steel_result_color(res);
                        ui.horizontal(|ui| {
                            ui.colored_label(
                                col,
                                format!(
                                    "Fr {fid} ({}): D/C = {:.3} [{}, {}]",
                                    res.section_name, res.total_ratio, res.eq_name, res.status
                                ),
                            );
                        });
                    }
                });
            }

            if !self.model.concrete_design_results.is_empty() {
                egui::CollapsingHeader::new(format!(
                    "Concrete Design ({})",
                    self.model.concrete_design_code
                ))
                .default_open(true)
                .show(ui, |ui| {
                    for (fid, res) in &self.model.concrete_design_results {
                        let col = concrete_result_color(res);
                        let detail = if res.is_column {
                            format!("Col PMM = {:.3} [{}]", res.pmm_ratio, res.status)
                        } else {
                            format!(
                                "Beam As = {:.2}/{:.2} [{}]",
                                res.as_top, res.as_bot, res.status
                            )
                        };
                        ui.colored_label(col, format!("Fr {fid} ({}): {detail}", res.section_name));
                    }
                });
            }
        });
    }

    fn render_viewport(&mut self, ui: &mut egui::Ui, proj: ViewProjection, title: &str) {
        let avail = ui.available_size();
        let (rect, response) = ui.allocate_exact_size(avail, Sense::click_and_drag());
        let painter = ui.painter_at(rect);

        // Dark CAD background matching SAP2000 dark theme
        painter.rect_filled(rect, 0.0, Color32::from_rgb(16, 20, 28));
        painter.rect_stroke(
            rect,
            0.0,
            Stroke::new(1.0_f32, Color32::from_rgb(55, 65, 85)),
        );

        // Camera manipulation
        if response.dragged_by(egui::PointerButton::Secondary)
            || response.dragged_by(egui::PointerButton::Middle)
            || (response.dragged_by(egui::PointerButton::Primary) && ui.input(|i| i.modifiers.alt))
        {
            let delta = response.drag_delta();
            if proj == ViewProjection::Perspective3D {
                self.yaw_rad += delta.x * 0.01;
                self.pitch_rad = (self.pitch_rad + delta.y * 0.01).clamp(-1.5, 1.5);
            } else {
                self.pan += delta;
            }
        } else if response.dragged_by(egui::PointerButton::Primary)
            && self.tool_mode == ToolMode::Select
        {
            let delta = response.drag_delta();
            if ui.input(|i| i.modifiers.shift) || proj != ViewProjection::Perspective3D {
                self.pan += delta;
            } else {
                self.yaw_rad += delta.x * 0.008;
                self.pitch_rad = (self.pitch_rad + delta.y * 0.008).clamp(-1.5, 1.5);
            }
        }

        if response.hovered() {
            let scroll = ui.input(|i| i.raw_scroll_delta.y);
            if scroll.abs() > 0.1 {
                let factor = (1.0 + scroll * 0.0015).clamp(0.8, 1.25);
                self.zoom = (self.zoom * factor).clamp(0.1, 20.0);
            }
        }

        // Compute finite model bounding box without f32 overflow
        let (min_pt, max_pt) = self.compute_bounds();
        let center_3d = [
            0.5 * (min_pt[0] + max_pt[0]),
            0.5 * (min_pt[1] + max_pt[1]),
            0.5 * (min_pt[2] + max_pt[2]),
        ];
        let dx_b = max_pt[0] - min_pt[0];
        let dy_b = max_pt[1] - min_pt[1];
        let dz_b = max_pt[2] - min_pt[2];
        let span_f64 = dx_b.hypot(dy_b).hypot(dz_b).max(10.0);
        let min_dim = rect.width().min(rect.height()).max(1.0) as f64;
        let base_scale = ((min_dim * 0.62 / span_f64) * (self.zoom as f64)).clamp(1e-9, 1e6) as f32;
        let screen_center = rect.center() + self.pan;
        let is_planar_xz = dy_b.abs() < 1e-6 && self.model.points.len() >= 2;

        let project = |p: [f64; 3]| -> Pos2 {
            if !p[0].is_finite() || !p[1].is_finite() || !p[2].is_finite() {
                return screen_center;
            }
            let dx = ((p[0] - center_3d[0]) * (base_scale as f64)).clamp(-50_000.0, 50_000.0) as f32;
            let dy = ((p[1] - center_3d[1]) * (base_scale as f64)).clamp(-50_000.0, 50_000.0) as f32;
            let dz = ((p[2] - center_3d[2]) * (base_scale as f64)).clamp(-50_000.0, 50_000.0) as f32;
            let (sx, sy) = match proj {
                ViewProjection::Perspective3D => {
                    let (syaw, cyaw) = self.yaw_rad.sin_cos();
                    let (sp, cp) = self.pitch_rad.sin_cos();
                    let x1 = dx * cyaw - dy * syaw;
                    let y1 = dx * syaw + dy * cyaw;
                    let z2 = dz * cp - y1 * sp;
                    (x1, -z2)
                }
                ViewProjection::ElevationXZ => (dx, -dz),
                ViewProjection::PlanXY => (dx, -dy),
                ViewProjection::ElevationYZ => (dy, -dz),
            };
            Pos2::new(screen_center.x + sx, screen_center.y + sy)
        };

        // Active static load-pattern multipliers for load arrow visualization
        let mut active_pat_scales: BTreeMap<String, f64> = BTreeMap::new();
        if let Some(lc) = self.model.load_cases.get(&self.active_case) {
            for assign in &lc.loads {
                if assign.scale_factor.is_finite() && assign.scale_factor.abs() > 1e-12 {
                    *active_pat_scales
                        .entry(assign.load_name.clone())
                        .or_insert(0.0) += assign.scale_factor;
                }
            }
        } else if self.model.load_patterns.contains_key(&self.active_case) {
            active_pat_scales.insert(self.active_case.clone(), 1.0);
        }

        // Animation phase & deformed displacement scale
        let anim_mult = if self.animate_deformed {
            self.anim_phase.sin() as f64
        } else {
            1.0
        };
        let def_scale = (self.deform_scale as f64) * anim_mult;
        let case_res = self.model.case_results.get(&self.active_case);
        let active_mode_idx = self.active_mode_idx;
        let use_time_step = self.use_time_step;
        let active_step_idx = self.active_step_idx;

        let displaced_point = |pid: &str, base: [f64; 3]| -> [f64; 3] {
            if let Some(cr) = case_res {
                let d = joint_disp_3d(
                    cr,
                    pid,
                    def_scale,
                    active_mode_idx,
                    use_time_step,
                    active_step_idx,
                );
                [base[0] + d[0], base[1] + d[1], base[2] + d[2]]
            } else {
                base
            }
        };

        // Precompute normalized contour range for ShellContour / SolidContour
        let contour_range: Option<(&str, f64, f64)> = match (self.display_mode, case_res) {
            (DisplayMode::ShellContour(scomp), Some(cr)) if !cr.area_results.is_empty() => {
                let mut lo = f64::INFINITY;
                let mut hi = f64::NEG_INFINITY;
                for ar in cr.area_results.values() {
                    let v = match scomp {
                        ShellComponent::F11 => ar.f11,
                        ShellComponent::F22 => ar.f22,
                        ShellComponent::M11 => ar.m11,
                        ShellComponent::M22 => ar.m22,
                        ShellComponent::SvmTop => ar.svm_top,
                    };
                    if v.is_finite() {
                        lo = lo.min(v);
                        hi = hi.max(v);
                    }
                }
                (lo <= hi).then_some((scomp.label(), lo, hi))
            }
            (DisplayMode::SolidContour(sol_comp), Some(cr)) if !cr.solid_results.is_empty() => {
                let mut lo = f64::INFINITY;
                let mut hi = f64::NEG_INFINITY;
                for sr in cr.solid_results.values() {
                    let s = &sr.center_stress;
                    let v = match sol_comp {
                        SolidComponent::S11 => s.s11,
                        SolidComponent::S22 => s.s22,
                        SolidComponent::S33 => s.s33,
                        SolidComponent::VonMises => s.von_mises,
                    };
                    if v.is_finite() {
                        lo = lo.min(v);
                        hi = hi.max(v);
                    }
                }
                (lo <= hi).then_some((sol_comp.label(), lo, hi))
            }
            _ => None,
        };

        // 1. Draw Grid Lines & Bubbles (orthogonal on base planes, never diagonal in 3D)
        if self.show_grids {
            if let Some(cs) = self.model.coord_systems.get("GLOBAL") {
                let grid_col = Color32::from_rgba_unmultiplied(70, 85, 110, 120);
                for g in &cs.grids {
                    if !g.visible || !g.coordinate.is_finite() {
                        continue;
                    }
                    let (p_a, p_b) = match g.axis.as_str() {
                        "X" => {
                            if proj == ViewProjection::ElevationXZ || is_planar_xz {
                                (
                                    [g.coordinate, min_pt[1], min_pt[2]],
                                    [g.coordinate, min_pt[1], max_pt[2]],
                                )
                            } else {
                                (
                                    [g.coordinate, min_pt[1], min_pt[2]],
                                    [g.coordinate, max_pt[1], min_pt[2]],
                                )
                            }
                        }
                        "Y" => {
                            if proj == ViewProjection::ElevationYZ {
                                (
                                    [min_pt[0], g.coordinate, min_pt[2]],
                                    [min_pt[0], g.coordinate, max_pt[2]],
                                )
                            } else {
                                (
                                    [min_pt[0], g.coordinate, min_pt[2]],
                                    [max_pt[0], g.coordinate, min_pt[2]],
                                )
                            }
                        }
                        "Z" => {
                            if proj == ViewProjection::ElevationYZ {
                                (
                                    [min_pt[0], min_pt[1], g.coordinate],
                                    [min_pt[0], max_pt[1], g.coordinate],
                                )
                            } else {
                                (
                                    [min_pt[0], min_pt[1], g.coordinate],
                                    [max_pt[0], min_pt[1], g.coordinate],
                                )
                            }
                        }
                        _ => continue,
                    };
                    let sa = project(p_a);
                    let sb = project(p_b);
                    if sa.distance(sb) > 1.0 {
                        painter.line_segment([sa, sb], Stroke::new(1.0_f32, grid_col));
                        painter.circle_stroke(sb, 8.0, Stroke::new(1.0_f32, grid_col));
                        painter.text(
                            sb,
                            egui::Align2::CENTER_CENTER,
                            &g.id,
                            FontId::proportional(10.0),
                            Color32::from_rgb(150, 170, 200),
                        );
                    }
                }
            }
        }

        // 2. Draw 3D Hexahedral Solid Objects
        const HEX_FACES: [[usize; 4]; 6] = [
            [0, 1, 2, 3],
            [4, 5, 6, 7],
            [0, 1, 5, 4],
            [1, 2, 6, 5],
            [2, 3, 7, 6],
            [3, 0, 4, 7],
        ];
        for (sid, solid) in &self.model.solids {
            let mut corners = [[0.0_f64; 3]; 8];
            let mut valid = true;
            for (k, pid) in solid.points.iter().enumerate() {
                if let Some(pt) = self.model.points.get(pid) {
                    let base = [pt.x, pt.y, pt.z];
                    corners[k] = if self.display_mode == DisplayMode::Deformed {
                        displaced_point(pid, base)
                    } else {
                        base
                    };
                } else {
                    valid = false;
                    break;
                }
            }
            if !valid {
                continue;
            }
            let fill_col = if let (
                DisplayMode::SolidContour(sol_comp),
                Some(cr),
                Some((_, lo, hi)),
            ) = (self.display_mode, case_res, contour_range)
            {
                if let Some(sr) = cr.solid_results.get(sid) {
                    let s = &sr.center_stress;
                    let v = match sol_comp {
                        SolidComponent::S11 => s.s11,
                        SolidComponent::S22 => s.s22,
                        SolidComponent::S33 => s.s33,
                        SolidComponent::VonMises => s.von_mises,
                    };
                    normalized_contour_color(v, lo, hi, 115)
                } else {
                    Color32::from_rgba_unmultiplied(110, 130, 175, 45)
                }
            } else if solid.selected {
                Color32::from_rgba_unmultiplied(255, 200, 60, 80)
            } else {
                Color32::from_rgba_unmultiplied(110, 145, 190, 42)
            };
            let stroke_col = if self.display_mode == DisplayMode::Deformed {
                Color32::from_rgb(80, 255, 160)
            } else if solid.selected {
                Color32::from_rgb(255, 215, 0)
            } else {
                Color32::from_rgb(130, 175, 225)
            };
            for face in &HEX_FACES {
                let fpoly = vec![
                    project(corners[face[0]]),
                    project(corners[face[1]]),
                    project(corners[face[2]]),
                    project(corners[face[3]]),
                ];
                painter.add(Shape::convex_polygon(
                    fpoly,
                    fill_col,
                    Stroke::new(1.0_f32, stroke_col),
                ));
            }
            if self.show_solid_labels {
                let mut c = [0.0; 3];
                for p in &corners {
                    c[0] += p[0] * 0.125;
                    c[1] += p[1] * 0.125;
                    c[2] += p[2] * 0.125;
                }
                painter.text(
                    project(c),
                    egui::Align2::CENTER_CENTER,
                    format!("S{sid}"),
                    FontId::proportional(10.5),
                    Color32::from_rgb(180, 220, 255),
                );
            }
        }

        // 3. Draw Area / Shell Objects (+ Deformed Mesh + Applied Area Loads)
        let mut area_screen_polys: Vec<(String, Vec<Pos2>)> = Vec::new();
        for (aid, area) in &self.model.areas {
            let mut poly_undef = Vec::with_capacity(area.points.len());
            let mut poly_def = Vec::with_capacity(area.points.len());
            let mut world_pts = Vec::with_capacity(area.points.len());
            for pid in &area.points {
                if let Some(pt) = self.model.points.get(pid) {
                    let base = [pt.x, pt.y, pt.z];
                    world_pts.push(base);
                    poly_undef.push(project(base));
                    if self.display_mode == DisplayMode::Deformed {
                        poly_def.push(project(displaced_point(pid, base)));
                    }
                }
            }
            if poly_undef.len() >= 3 {
                if self.display_mode == DisplayMode::Deformed {
                    painter.add(Shape::convex_polygon(
                        poly_undef.clone(),
                        Color32::TRANSPARENT,
                        Stroke::new(1.0_f32, Color32::from_rgba_unmultiplied(95, 115, 140, 90)),
                    ));
                }
                let active_poly = if self.display_mode == DisplayMode::Deformed && poly_def.len() >= 3
                {
                    &poly_def
                } else {
                    &poly_undef
                };
                let fill_col = if let (
                    DisplayMode::ShellContour(scomp),
                    Some(cr),
                    Some((_, lo, hi)),
                ) = (self.display_mode, case_res, contour_range)
                {
                    if let Some(ar) = cr.area_results.get(aid) {
                        let val = match scomp {
                            ShellComponent::F11 => ar.f11,
                            ShellComponent::F22 => ar.f22,
                            ShellComponent::M11 => ar.m11,
                            ShellComponent::M22 => ar.m22,
                            ShellComponent::SvmTop => ar.svm_top,
                        };
                        normalized_contour_color(val, lo, hi, 145)
                    } else {
                        Color32::from_rgba_unmultiplied(60, 120, 180, 65)
                    }
                } else if area.selected {
                    Color32::from_rgba_unmultiplied(255, 200, 60, 90)
                } else if self.display_mode == DisplayMode::Deformed {
                    Color32::from_rgba_unmultiplied(70, 220, 150, 55)
                } else {
                    Color32::from_rgba_unmultiplied(60, 120, 180, 55)
                };
                let edge_col = if self.display_mode == DisplayMode::Deformed {
                    Color32::from_rgb(80, 255, 160)
                } else if area.selected {
                    Color32::from_rgb(255, 215, 0)
                } else {
                    Color32::from_rgb(100, 160, 220)
                };
                painter.add(Shape::convex_polygon(
                    active_poly.clone(),
                    fill_col,
                    Stroke::new(1.2_f32, edge_col),
                ));

                let mut cx = 0.0;
                let mut cy = 0.0;
                for p in active_poly {
                    cx += p.x;
                    cy += p.y;
                }
                let centroid_scr =
                    Pos2::new(cx / active_poly.len() as f32, cy / active_poly.len() as f32);

                if self.show_area_labels {
                    painter.text(
                        centroid_scr,
                        egui::Align2::CENTER_CENTER,
                        format!("A{aid}"),
                        FontId::proportional(11.0),
                        Color32::from_rgb(160, 220, 255),
                    );
                }

                // Area uniform & pressure loads at centroid
                if self.show_loads && self.display_mode == DisplayMode::Undeformed {
                    let mut c3 = [0.0; 3];
                    for wp in &world_pts {
                        c3[0] += wp[0] / world_pts.len() as f64;
                        c3[1] += wp[1] / world_pts.len() as f64;
                        c3[2] += wp[2] / world_pts.len() as f64;
                    }
                    let normal = area_unit_normal(&world_pts);
                    for ul in &area.loads_uniform {
                        if let Some(&sf) = active_pat_scales.get(&ul.load_pat) {
                            let v = ul.val * sf;
                            if v.abs() > 1e-12 {
                                let dir = area_load_dir_3d(ul.dir, normal, v);
                                draw_world_force_arrow(&painter, &project, c3, dir, span_f64);
                            }
                        }
                    }
                    for pl in &area.loads_pressure {
                        if let Some(&sf) = active_pat_scales.get(&pl.load_pat) {
                            let v = pl.val * sf;
                            if v.abs() > 1e-12 {
                                // Top face (-2) pushes along -local 3; Bottom face (-1) pushes along +local 3
                                let sign = if pl.face == -2 { -v.signum() } else { v.signum() };
                                let dir = [normal[0] * sign, normal[1] * sign, normal[2] * sign];
                                draw_world_force_arrow(&painter, &project, c3, dir, span_f64);
                            }
                        }
                    }
                }

                area_screen_polys.push((aid.clone(), poly_undef));
            }
        }

        // 4. Draw Link / Spring-Damper Elements (1-Joint Grounded & 2-Joint Links)
        let mut link_screen_segs: Vec<(String, Pos2, Pos2)> = Vec::new();
        for (lid, link) in &self.model.links {
            let Some(pi) = self.model.points.get(&link.point_i) else {
                continue;
            };
            let base_i = [pi.x, pi.y, pi.z];
            let p_i = if self.display_mode == DisplayMode::Deformed {
                displaced_point(&link.point_i, base_i)
            } else {
                base_i
            };
            let s_i = project(p_i);
            let link_col = if link.selected {
                Color32::from_rgb(255, 215, 0)
            } else if self.display_mode == DisplayMode::Deformed {
                Color32::from_rgb(80, 255, 160)
            } else {
                Color32::from_rgb(235, 130, 255)
            };
            if let Some(pj_name) = &link.point_j {
                let Some(pj) = self.model.points.get(pj_name) else {
                    continue;
                };
                let base_j = [pj.x, pj.y, pj.z];
                let p_j = if self.display_mode == DisplayMode::Deformed {
                    displaced_point(pj_name, base_j)
                } else {
                    base_j
                };
                let s_j = project(p_j);
                painter.line_segment([s_i, s_j], Stroke::new(2.2_f32, link_col));
                let mid = Pos2::new(0.5 * (s_i.x + s_j.x), 0.5 * (s_i.y + s_j.y));
                painter.rect_stroke(
                    Rect::from_center_size(mid, Vec2::new(9.0, 9.0)),
                    0.0,
                    Stroke::new(1.5_f32, link_col),
                );
                if self.show_link_labels {
                    painter.text(
                        mid + Vec2::new(0.0, -7.0),
                        egui::Align2::CENTER_BOTTOM,
                        format!("L{lid}"),
                        FontId::proportional(10.0),
                        link_col,
                    );
                }
                link_screen_segs.push((lid.clone(), s_i, s_j));
            } else {
                // 1-joint grounded spring-damper symbol
                let ground_end = s_i + Vec2::new(0.0, 16.0);
                painter.line_segment([s_i, ground_end], Stroke::new(2.0_f32, link_col));
                painter.line_segment(
                    [
                        ground_end + Vec2::new(-6.0, 0.0),
                        ground_end + Vec2::new(6.0, 0.0),
                    ],
                    Stroke::new(2.0_f32, link_col),
                );
                if self.show_link_labels {
                    painter.text(
                        ground_end + Vec2::new(8.0, 0.0),
                        egui::Align2::LEFT_CENTER,
                        format!("L{lid}"),
                        FontId::proportional(10.0),
                        link_col,
                    );
                }
                link_screen_segs.push((lid.clone(), s_i, ground_end));
            }
        }

        // Compute finite max diagram value for normalization if in FrameDiagram mode
        let mut max_diag_val = 1e-6f64;
        if let DisplayMode::FrameDiagram(comp) = self.display_mode {
            if let Some(cr) = case_res {
                for stations in cr.frame_forces.values() {
                    for st in stations {
                        let v = match comp {
                            FrameForceComponent::AxialP => st.p.abs(),
                            FrameForceComponent::ShearV2 => st.v2.abs(),
                            FrameForceComponent::ShearV3 => st.v3.abs(),
                            FrameForceComponent::TorsionT => st.t.abs(),
                            FrameForceComponent::MomentM2 => st.m2.abs(),
                            FrameForceComponent::MomentM3 => st.m3.abs(),
                        };
                        if v.is_finite() && v > max_diag_val {
                            max_diag_val = v;
                        }
                    }
                }
            }
        }

        // 5. Draw Frame Elements, Deformed Curves, Force/Moment Diagrams, or Design Colors
        let mut frame_screen_segs: Vec<(String, Pos2, Pos2)> = Vec::new();
        for (fid, frame) in &self.model.frames {
            let (Some(pi), Some(pj)) = (
                self.model.points.get(&frame.point_i),
                self.model.points.get(&frame.point_j),
            ) else {
                continue;
            };
            let p1 = [pi.x, pi.y, pi.z];
            let p2 = [pj.x, pj.y, pj.z];
            let s1 = project(p1);
            let s2 = project(p2);
            frame_screen_segs.push((fid.clone(), s1, s2));

            let (line_col, width): (Color32, f32) = match self.display_mode {
                DisplayMode::DesignRatioSteel => {
                    if let Some(res) = self.model.steel_design_results.get(fid) {
                        (steel_result_color(res), 3.5)
                    } else {
                        (Color32::from_rgb(130, 150, 175), 2.0)
                    }
                }
                DisplayMode::DesignRatioConcrete => {
                    if let Some(res) = self.model.concrete_design_results.get(fid) {
                        (concrete_result_color(res), 3.5)
                    } else {
                        (Color32::from_rgb(130, 150, 175), 2.0)
                    }
                }
                DisplayMode::Deformed => (Color32::from_rgba_unmultiplied(110, 125, 145, 110), 1.2),
                _ => {
                    if frame.selected {
                        (Color32::from_rgb(255, 215, 0), 3.2)
                    } else if frame.cable_params.is_some() {
                        (Color32::from_rgb(120, 235, 210), 2.2)
                    } else {
                        (Color32::from_rgb(120, 200, 255), 2.2)
                    }
                }
            };

            if self.show_extruded {
                let sec_d = self
                    .model
                    .frame_sections
                    .get(&frame.section)
                    .map(|s| s.t3)
                    .unwrap_or(12.0);
                let (_, e_axes) = frame_local_axes(p1, p2, frame.angle_deg);
                let e2 = e_axes[1];
                let top_i = project([
                    p1[0] + 0.5 * sec_d * e2[0],
                    p1[1] + 0.5 * sec_d * e2[1],
                    p1[2] + 0.5 * sec_d * e2[2],
                ]);
                let bot_i = project([
                    p1[0] - 0.5 * sec_d * e2[0],
                    p1[1] - 0.5 * sec_d * e2[1],
                    p1[2] - 0.5 * sec_d * e2[2],
                ]);
                let top_j = project([
                    p2[0] + 0.5 * sec_d * e2[0],
                    p2[1] + 0.5 * sec_d * e2[1],
                    p2[2] + 0.5 * sec_d * e2[2],
                ]);
                let bot_j = project([
                    p2[0] - 0.5 * sec_d * e2[0],
                    p2[1] - 0.5 * sec_d * e2[1],
                    p2[2] - 0.5 * sec_d * e2[2],
                ]);
                painter.add(Shape::convex_polygon(
                    vec![top_i, top_j, bot_j, bot_i],
                    Color32::from_rgba_unmultiplied(100, 180, 240, 65),
                    Stroke::new(1.5_f32, line_col),
                ));
            } else {
                painter.line_segment([s1, s2], Stroke::new(width, line_col));
            }

            // Frame label or Steel/Concrete Design label at midpoint
            let mid_s = Pos2::new(0.5 * (s1.x + s2.x), 0.5 * (s1.y + s2.y));
            if self.show_frame_labels {
                painter.text(
                    mid_s + Vec2::new(0.0, -8.0),
                    egui::Align2::CENTER_BOTTOM,
                    format!("{fid} ({})", frame.section),
                    FontId::proportional(10.5),
                    Color32::from_rgb(200, 230, 255),
                );
            }
            if self.display_mode == DisplayMode::DesignRatioSteel {
                if let Some(res) = self.model.steel_design_results.get(fid) {
                    let lbl = if res.status == "OK" {
                        format!("{:.3}", res.total_ratio)
                    } else {
                        format!("{:.3} ({})", res.total_ratio, res.status)
                    };
                    painter.text(
                        mid_s + Vec2::new(0.0, -6.0),
                        egui::Align2::CENTER_BOTTOM,
                        lbl,
                        FontId::proportional(11.0),
                        steel_result_color(res),
                    );
                }
            } else if self.display_mode == DisplayMode::DesignRatioConcrete {
                if let Some(res) = self.model.concrete_design_results.get(fid) {
                    let lbl = if res.is_column {
                        if res.status == "OK" {
                            format!("PMM {:.3}", res.pmm_ratio)
                        } else {
                            format!("PMM {:.3} ({})", res.pmm_ratio, res.status)
                        }
                    } else if res.status == "OK" {
                        format!("As {:.2}/{:.2}", res.as_top, res.as_bot)
                    } else {
                        format!("As {:.2}/{:.2} ({})", res.as_top, res.as_bot, res.status)
                    };
                    painter.text(
                        mid_s + Vec2::new(0.0, -6.0),
                        egui::Align2::CENTER_BOTTOM,
                        lbl,
                        FontId::proportional(10.5),
                        concrete_result_color(res),
                    );
                }
            }

            // Oriented distributed & point load arrows on frame
            if self.show_loads && self.display_mode == DisplayMode::Undeformed {
                let (_, e_axes) = frame_local_axes(p1, p2, frame.angle_deg);
                for ld in &frame.loads_dist {
                    if let Some(&sf) = active_pat_scales.get(&ld.load_pat) {
                        let r1 = ld.rel_dist1.clamp(0.0, 1.0);
                        let r2 = ld.rel_dist2.clamp(0.0, 1.0);
                        if r2 > r1 {
                            for k in 0..=4 {
                                let frac = k as f64 / 4.0;
                                let s = r1 + frac * (r2 - r1);
                                let w = (ld.val1 + frac * (ld.val2 - ld.val1)) * sf;
                                if w.abs() > 1e-12 {
                                    let px = [
                                        p1[0] + s * (p2[0] - p1[0]),
                                        p1[1] + s * (p2[1] - p1[1]),
                                        p1[2] + s * (p2[2] - p1[2]),
                                    ];
                                    let dir = frame_load_dir_3d(ld.dir, &e_axes, w);
                                    draw_world_force_arrow(&painter, &project, px, dir, span_f64);
                                }
                            }
                        }
                    }
                }
                for lp in &frame.loads_point {
                    if let Some(&sf) = active_pat_scales.get(&lp.load_pat) {
                        let fval = lp.val * sf;
                        if fval.abs() > 1e-12 {
                            let s = lp.rel_dist.clamp(0.0, 1.0);
                            let px = [
                                p1[0] + s * (p2[0] - p1[0]),
                                p1[1] + s * (p2[1] - p1[1]),
                                p1[2] + s * (p2[2] - p1[2]),
                            ];
                            let dir = frame_load_dir_3d(lp.dir, &e_axes, fval);
                            draw_world_force_arrow(&painter, &project, px, dir, span_f64);
                        }
                    }
                }
            }

            // Deformed shape curve (supports Modal, Time-History scrubbing, Station curves, and Buckling joint fallback)
            if self.display_mode == DisplayMode::Deformed {
                if let Some(cr) = case_res {
                    let use_stations = cr.modes.is_empty()
                        && !(use_time_step && !cr.time_steps.is_empty())
                        && cr.frame_forces.contains_key(fid);
                    if use_stations {
                        if let Some(stations) = cr.frame_forces.get(fid) {
                            let (_, e_axes) = frame_local_axes(p1, p2, frame.angle_deg);
                            let mut pts = Vec::with_capacity(stations.len());
                            for st in stations {
                                let s = st.rel_dist;
                                let base = [
                                    p1[0] + s * (p2[0] - p1[0]),
                                    p1[1] + s * (p2[1] - p1[1]),
                                    p1[2] + s * (p2[2] - p1[2]),
                                ];
                                let [u1, u2, u3] = st.local_disp;
                                let dg = [
                                    e_axes[0][0] * u1 + e_axes[1][0] * u2 + e_axes[2][0] * u3,
                                    e_axes[0][1] * u1 + e_axes[1][1] * u2 + e_axes[2][1] * u3,
                                    e_axes[0][2] * u1 + e_axes[1][2] * u2 + e_axes[2][2] * u3,
                                ];
                                pts.push(project([
                                    base[0] + def_scale * dg[0],
                                    base[1] + def_scale * dg[1],
                                    base[2] + def_scale * dg[2],
                                ]));
                            }
                            for w in pts.windows(2) {
                                painter.line_segment(
                                    [w[0], w[1]],
                                    Stroke::new(2.6_f32, Color32::from_rgb(80, 255, 160)),
                                );
                            }
                        }
                    } else {
                        let d1 = project(displaced_point(&frame.point_i, p1));
                        let d2 = project(displaced_point(&frame.point_j, p2));
                        painter.line_segment(
                            [d1, d2],
                            Stroke::new(2.6_f32, Color32::from_rgb(80, 255, 160)),
                        );
                    }
                }
            }

            // Internal Force / Moment Diagram along Frame (NaN-safe)
            if let DisplayMode::FrameDiagram(comp) = self.display_mode {
                if let Some(cr) = case_res {
                    if let Some(stations) = cr.frame_forces.get(fid) {
                        let (_, e_axes) = frame_local_axes(p1, p2, frame.angle_deg);
                        let dir_3d = match comp {
                            FrameForceComponent::MomentM2 | FrameForceComponent::ShearV3 => {
                                e_axes[2]
                            }
                            _ => e_axes[1],
                        };
                        let diag_world_scale =
                            span_f64 * 0.12 * (self.diagram_scale as f64) / max_diag_val;
                        let mut outline = Vec::with_capacity(stations.len());
                        for st in stations {
                            let s = st.rel_dist;
                            let base = [
                                p1[0] + s * (p2[0] - p1[0]),
                                p1[1] + s * (p2[1] - p1[1]),
                                p1[2] + s * (p2[2] - p1[2]),
                            ];
                            let val = match comp {
                                FrameForceComponent::AxialP => st.p,
                                FrameForceComponent::ShearV2 => st.v2,
                                FrameForceComponent::ShearV3 => st.v3,
                                FrameForceComponent::TorsionT => st.t,
                                FrameForceComponent::MomentM2 => st.m2,
                                FrameForceComponent::MomentM3 => -st.m3, // Plot on tension side per SAP2000 convention
                            };
                            if !val.is_finite() {
                                continue;
                            }
                            let offset_pt = [
                                base[0] + val * diag_world_scale * dir_3d[0],
                                base[1] + val * diag_world_scale * dir_3d[1],
                                base[2] + val * diag_world_scale * dir_3d[2],
                            ];
                            let b_scr = project(base);
                            let o_scr = project(offset_pt);
                            let col = if val >= 0.0 {
                                Color32::from_rgba_unmultiplied(255, 190, 60, 160)
                            } else {
                                Color32::from_rgba_unmultiplied(255, 95, 95, 160)
                            };
                            painter.line_segment([b_scr, o_scr], Stroke::new(1.2_f32, col));
                            outline.push((o_scr, val));
                        }
                        for w in outline.windows(2) {
                            let col = if w[0].1 >= 0.0 {
                                Color32::from_rgb(255, 200, 70)
                            } else {
                                Color32::from_rgb(255, 100, 100)
                            };
                            painter.line_segment([w[0].0, w[1].0], Stroke::new(2.0_f32, col));
                        }
                        if let Some((peak_pos, peak_val)) = outline
                            .iter()
                            .filter(|(pos, val)| pos.x.is_finite() && pos.y.is_finite() && val.is_finite())
                            .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
                        {
                            if peak_val.abs() > 1e-4 {
                                painter.text(
                                    *peak_pos,
                                    egui::Align2::CENTER_BOTTOM,
                                    format!("{:.2}", peak_val.abs()),
                                    FontId::proportional(10.5),
                                    Color32::from_rgb(255, 235, 150),
                                );
                            }
                        }
                    }
                }
            }
        }

        // 6. Draw Joints, Restraints, Applied Joint Loads, Reactions, and Hit-Test Picking
        let click_pos = if response.clicked() {
            response.interact_pointer_pos()
        } else {
            None
        };
        let right_click_pos = if response.secondary_clicked() {
            response.interact_pointer_pos()
        } else {
            None
        };
        let mut closest_joint: Option<(String, f32)> = None;

        for (pid, pt) in &self.model.points {
            let base = [pt.x, pt.y, pt.z];
            let sp = project(base);
            if let Some(cp) = click_pos {
                let dist = cp.distance(sp);
                if dist < 10.0 && closest_joint.as_ref().is_none_or(|(_, d)| dist < *d) {
                    closest_joint = Some((pid.clone(), dist));
                }
            }

            if self.display_mode == DisplayMode::Deformed {
                painter.circle_filled(sp, 2.5, Color32::from_rgba_unmultiplied(150, 165, 185, 100));
                let sp_def = project(displaced_point(pid, base));
                let pt_col = if pt.selected {
                    Color32::from_rgb(255, 230, 50)
                } else {
                    Color32::from_rgb(100, 255, 175)
                };
                painter.circle_filled(sp_def, 3.8, pt_col);
            } else {
                let pt_col = if pt.selected {
                    Color32::from_rgb(255, 230, 50)
                } else {
                    Color32::from_rgb(230, 240, 255)
                };
                painter.circle_filled(sp, 3.8, pt_col);
            }

            if self.show_joint_labels {
                painter.text(
                    sp + Vec2::new(6.0, -6.0),
                    egui::Align2::LEFT_BOTTOM,
                    pid,
                    FontId::proportional(11.0),
                    Color32::from_rgb(210, 225, 245),
                );
            }

            // Applied Joint Point Forces
            if self.show_loads && self.display_mode == DisplayMode::Undeformed {
                let mut net_f = [0.0; 3];
                for pl in &pt.loads_force {
                    if let Some(&sf) = active_pat_scales.get(&pl.load_pat) {
                        net_f[0] += pl.force[0] * sf;
                        net_f[1] += pl.force[1] * sf;
                        net_f[2] += pl.force[2] * sf;
                    }
                }
                let mag = net_f[0].hypot(net_f[1]).hypot(net_f[2]);
                if mag > 1e-12 {
                    let dir = [net_f[0] / mag, net_f[1] / mag, net_f[2] / mag];
                    draw_world_force_arrow(&painter, &project, base, dir, span_f64);
                }
            }

            // Support restraint symbols (ignore pure out-of-plane 2D guide restraints)
            if self.show_restraints {
                let mut eff_r = [false; 6];
                for d in 0..6 {
                    eff_r[d] = pt.restraint[d] && self.model.active_dofs[d];
                }
                if is_planar_xz && !eff_r[0] && !eff_r[2] && !eff_r[4] {
                    // Pure out-of-plane guide restraint [UY, RX, RZ] on a 2D XZ frame
                    eff_r = [false; 6];
                }
                if eff_r.iter().any(|&r| r) {
                    let trans_count = (0..3).filter(|&d| eff_r[d]).count();
                    let rot_restrained = (3..6).any(|d| eff_r[d]);
                    let in_plane_pinned = eff_r[0] && eff_r[2];
                    if (trans_count >= 2 || in_plane_pinned) && rot_restrained {
                        painter.rect_filled(
                            Rect::from_center_size(sp + Vec2::new(0.0, 5.0), Vec2::new(14.0, 7.0)),
                            0.0,
                            Color32::from_rgb(80, 210, 140),
                        );
                    } else if trans_count >= 2 || in_plane_pinned {
                        painter.add(Shape::convex_polygon(
                            vec![sp, sp + Vec2::new(-7.0, 10.0), sp + Vec2::new(7.0, 10.0)],
                            Color32::from_rgba_unmultiplied(80, 210, 140, 140),
                            Stroke::new(1.5_f32, Color32::from_rgb(80, 210, 140)),
                        ));
                    } else {
                        painter.circle_stroke(
                            sp + Vec2::new(0.0, 6.0),
                            5.5,
                            Stroke::new(1.5_f32, Color32::from_rgb(80, 210, 140)),
                        );
                    }
                }
            }

            // All 6-DOF Support Reactions display
            if self.display_mode == DisplayMode::Reactions {
                if let Some(cr) = case_res {
                    if let Some(r) = cr.joint_reactions.get(pid) {
                        let names = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"];
                        let parts: Vec<String> = names
                            .iter()
                            .zip(r.iter())
                            .filter(|(_, v)| v.is_finite() && v.abs() > 1e-4)
                            .map(|(n, v)| format!("{n}={v:.2}"))
                            .collect();
                        if !parts.is_empty() {
                            let fmag = r[0].hypot(r[1]).hypot(r[2]);
                            if fmag > 1e-6 {
                                let u_dir = [r[0] / fmag, r[1] / fmag, r[2] / fmag];
                                let tail_3d = [
                                    base[0] - u_dir[0] * span_f64 * 0.06,
                                    base[1] - u_dir[1] * span_f64 * 0.06,
                                    base[2] - u_dir[2] * span_f64 * 0.06,
                                ];
                                let tail = project(tail_3d);
                                let vec_scr = sp - tail;
                                if vec_scr.length() > 3.0 {
                                    painter.arrow(
                                        tail,
                                        vec_scr,
                                        Stroke::new(2.0_f32, Color32::from_rgb(120, 255, 140)),
                                    );
                                }
                            } else {
                                painter.arrow(
                                    sp + Vec2::new(0.0, 22.0),
                                    Vec2::new(0.0, -20.0),
                                    Stroke::new(2.0_f32, Color32::from_rgb(120, 255, 140)),
                                );
                            }
                            painter.text(
                                sp + Vec2::new(8.0, 14.0),
                                egui::Align2::LEFT_TOP,
                                parts.join(", "),
                                FontId::proportional(10.0),
                                Color32::from_rgb(140, 255, 160),
                            );
                        }
                    }
                }
            }
        }

        // Right-click on frame opens Member Station Diagram Inspector
        if let Some(rcp) = right_click_pos {
            if let Some((fid, _)) = frame_screen_segs
                .iter()
                .map(|(fid, a, b)| (fid, point_to_segment_dist(rcp, *a, *b)))
                .filter(|(_, d)| *d < 8.0)
                .min_by(|a, b| a.1.total_cmp(&b.1))
            {
                self.inspected_frame = Some(fid.clone());
            }
        }

        // Left-click selection or interactive CAD drawing
        if let Some(cp) = click_pos {
            match self.tool_mode {
                ToolMode::Select => {
                    if let Some((jid, _)) = closest_joint {
                        if let Some(pt) = self.model.points.get_mut(&jid) {
                            pt.selected = !pt.selected;
                        }
                    } else if let Some((fid, _)) = frame_screen_segs
                        .iter()
                        .map(|(fid, a, b)| (fid, point_to_segment_dist(cp, *a, *b)))
                        .filter(|(_, d)| *d < 8.0)
                        .min_by(|a, b| a.1.total_cmp(&b.1))
                    {
                        if let Some(fr) = self.model.frames.get_mut(fid) {
                            fr.selected = !fr.selected;
                        }
                    } else if let Some((lid, _)) = link_screen_segs
                        .iter()
                        .map(|(lid, a, b)| (lid, point_to_segment_dist(cp, *a, *b)))
                        .filter(|(_, d)| *d < 8.0)
                        .min_by(|a, b| a.1.total_cmp(&b.1))
                    {
                        if let Some(lnk) = self.model.links.get_mut(lid) {
                            lnk.selected = !lnk.selected;
                        }
                    } else if let Some((aid, _)) = area_screen_polys
                        .iter()
                        .rev()
                        .find(|(_, poly)| point_in_polygon(cp, poly))
                    {
                        if let Some(ar) = self.model.areas.get_mut(aid) {
                            ar.selected = !ar.selected;
                        }
                    }
                }
                ToolMode::DrawFrame => {
                    if let Some((jid, _)) = closest_joint {
                        if self.ensure_unlocked() {
                            if let Some(first) = self.draw_first_point.take() {
                                if first != jid {
                                    let fid = sap_model::next_numeric_name(self.model.frames.keys());
                                    let sec = self
                                        .model
                                        .frame_sections
                                        .keys()
                                        .next()
                                        .cloned()
                                        .unwrap_or_else(|| "FSEC1".to_string());
                                    self.invalidate_results();
                                    self.model.add_frame(&fid, &first, &jid, &sec);
                                    self.status_msg =
                                        format!("Drew Frame {fid} from Joint {first} to {jid}");
                                }
                            } else {
                                self.draw_first_point = Some(jid.clone());
                                self.status_msg = format!(
                                    "Selected start Joint {jid} — click end Joint to create Frame."
                                );
                            }
                        }
                    }
                }
                ToolMode::DrawArea => {
                    if let Some((jid, _)) = closest_joint {
                        if self.ensure_unlocked() {
                            if !self.draw_area_points.contains(&jid) {
                                self.draw_area_points.push(jid.clone());
                            }
                            if self.draw_area_points.len() == 4 {
                                let aid = sap_model::next_numeric_name(self.model.areas.keys());
                                let asec = self
                                    .model
                                    .area_sections
                                    .keys()
                                    .next()
                                    .cloned()
                                    .unwrap_or_else(|| "ASEC1".to_string());
                                let pts = std::mem::take(&mut self.draw_area_points);
                                self.invalidate_results();
                                self.model.add_area(&aid, pts, &asec);
                                self.status_msg = format!("Drew 4-Node Area {aid}");
                            } else {
                                self.status_msg = format!(
                                    "Area corner {}/4 selected (Joint {jid})",
                                    self.draw_area_points.len()
                                );
                            }
                        }
                    }
                }
                ToolMode::DrawPoint => {}
            }
        }

        // Contour Legend Overlay (when viewing ShellContour or SolidContour)
        if let Some((label, lo, hi)) = contour_range {
            let leg_rect = Rect::from_min_size(
                Pos2::new(rect.right() - 175.0, rect.top() + 8.0),
                Vec2::new(165.0, 42.0),
            );
            if rect.width() > 240.0 {
                painter.rect_filled(
                    leg_rect,
                    4.0,
                    Color32::from_rgba_unmultiplied(12, 16, 24, 210),
                );
                painter.text(
                    leg_rect.left_top() + Vec2::new(6.0, 4.0),
                    egui::Align2::LEFT_TOP,
                    label,
                    FontId::proportional(10.0),
                    Color32::from_rgb(210, 225, 245),
                );
                let steps = 16;
                let bar_w = (leg_rect.width() - 12.0) / steps as f32;
                for i in 0..steps {
                    let t = (i as f64 + 0.5) / steps as f64;
                    let v = lo + t * (hi - lo);
                    let sub = Rect::from_min_size(
                        Pos2::new(leg_rect.left() + 6.0 + i as f32 * bar_w, leg_rect.top() + 18.0),
                        Vec2::new(bar_w + 0.5, 8.0),
                    );
                    painter.rect_filled(sub, 0.0, normalized_contour_color(v, lo, hi, 230));
                }
                painter.text(
                    Pos2::new(leg_rect.left() + 6.0, leg_rect.bottom() - 2.0),
                    egui::Align2::LEFT_BOTTOM,
                    format!("{lo:.3e}"),
                    FontId::proportional(9.5),
                    Color32::from_rgb(180, 200, 230),
                );
                painter.text(
                    Pos2::new(leg_rect.right() - 6.0, leg_rect.bottom() - 2.0),
                    egui::Align2::RIGHT_BOTTOM,
                    format!("{hi:.3e}"),
                    FontId::proportional(9.5),
                    Color32::from_rgb(180, 200, 230),
                );
            }
        }

        // Viewport header overlay
        painter.text(
            rect.left_top() + Vec2::new(10.0, 8.0),
            egui::Align2::LEFT_TOP,
            format!("{title}  [Result: {}]", self.active_case),
            FontId::proportional(12.5),
            Color32::from_rgb(200, 220, 255),
        );
    }

    fn compute_bounds(&self) -> ([f64; 3], [f64; 3]) {
        let mut min_p = [f64::INFINITY; 3];
        let mut max_p = [f64::NEG_INFINITY; 3];
        let mut count = 0usize;
        for pt in self.model.points.values() {
            if pt.x.is_finite() && pt.y.is_finite() && pt.z.is_finite() {
                min_p[0] = min_p[0].min(pt.x);
                min_p[1] = min_p[1].min(pt.y);
                min_p[2] = min_p[2].min(pt.z);
                max_p[0] = max_p[0].max(pt.x);
                max_p[1] = max_p[1].max(pt.y);
                max_p[2] = max_p[2].max(pt.z);
                count += 1;
            }
        }
        if count == 0 {
            ([-100.0, -100.0, 0.0], [100.0, 100.0, 100.0])
        } else {
            (min_p, max_p)
        }
    }

    fn load_benchmark_1_001(&mut self) {
        let mut m = SapModel::new(Units::KipInF);
        m.model_name = "StructuraX Verification Example 1-001 (2D Frame)".to_string();
        m.load_patterns.clear();
        m.load_cases.clear();
        let mut mat = Material::default_concrete_kip_in();
        mat.name = "CONC".to_string();
        mat.e = 3600.0;
        mat.u = 0.2;
        mat.g = 1500.0;
        mat.unit_weight = 0.15 / 1728.0;
        m.materials.insert("CONC".to_string(), mat);

        let mut sec = make_rectangular("FSEC1", "CONC", 12.0, 12.0);
        sec.as2 = 0.0;
        sec.as3 = 0.0;
        sec.modifiers[0] = 1000.0;
        m.frame_sections.insert("FSEC1".to_string(), sec);

        m.add_point("1", 0.0, 0.0, 0.0);
        m.points.get_mut("1").unwrap().restraint = [true, true, true, true, false, true];
        m.add_point("2", 0.0, 0.0, 120.0);
        m.points.get_mut("2").unwrap().restraint = [false, true, false, true, false, true];
        m.add_point("3", 96.0, 0.0, 192.0);
        m.points.get_mut("3").unwrap().restraint = [true, true, false, true, false, true];
        m.add_point("4", -48.0, 0.0, 120.0);
        m.points.get_mut("4").unwrap().restraint = [false, true, false, true, false, true];

        m.add_frame("1", "1", "2", "FSEC1");
        m.add_frame("2", "2", "3", "FSEC1");
        m.add_frame("3", "4", "2", "FSEC1");

        for i in 1..=7 {
            let lp = format!("LCASE{i}");
            m.load_patterns.insert(
                lp.clone(),
                LoadPattern {
                    name: lp.clone(),
                    pat_type: LoadPatternType::Other,
                    self_wt_multiplier: if i == 1 { 1.0 } else { 0.0 },
                },
            );
            m.load_cases
                .insert(lp.clone(), LoadCase::linear_static(&lp, &lp, 1.0));
        }
        m.frames
            .get_mut("3")
            .unwrap()
            .loads_dist
            .push(FrameLoadDistributed {
                load_pat: "LCASE2".to_string(),
                my_type: 1,
                dir: 6,
                csys: "GLOBAL".to_string(),
                rel_dist1: 0.0,
                rel_dist2: 1.0,
                val1: -1.8 / 12.0,
                val2: -1.8 / 12.0,
            });
        m.points
            .get_mut("4")
            .unwrap()
            .loads_force
            .push(PointLoadForce {
                load_pat: "LCASE2".to_string(),
                csys: "GLOBAL".to_string(),
                force: [0.0, 0.0, -10.0, 0.0, 0.0, 0.0],
            });
        m.points
            .get_mut("2")
            .unwrap()
            .loads_force
            .push(PointLoadForce {
                load_pat: "LCASE3".to_string(),
                csys: "GLOBAL".to_string(),
                force: [0.0, 0.0, -17.2, 0.0, -54.4 * 12.0, 0.0],
            });
        m.frames
            .get_mut("2")
            .unwrap()
            .loads_dist
            .push(FrameLoadDistributed {
                load_pat: "LCASE4".to_string(),
                my_type: 1,
                dir: 9,
                csys: "GLOBAL".to_string(),
                rel_dist1: 0.0,
                rel_dist2: 1.0,
                val1: -2.0 / 12.0,
                val2: -2.0 / 12.0,
            });
        m.frames
            .get_mut("1")
            .unwrap()
            .loads_dist
            .push(FrameLoadDistributed {
                load_pat: "LCASE5".to_string(),
                my_type: 1,
                dir: 2,
                csys: "LOCAL".to_string(),
                rel_dist1: 0.0,
                rel_dist2: 1.0,
                val1: 2.0 / 12.0,
                val2: 2.0 / 12.0,
            });
        m.frames
            .get_mut("2")
            .unwrap()
            .loads_dist
            .push(FrameLoadDistributed {
                load_pat: "LCASE5".to_string(),
                my_type: 1,
                dir: 2,
                csys: "LOCAL".to_string(),
                rel_dist1: 0.0,
                rel_dist2: 1.0,
                val1: -2.0 / 12.0,
                val2: -2.0 / 12.0,
            });
        m.frames
            .get_mut("1")
            .unwrap()
            .loads_dist
            .push(FrameLoadDistributed {
                load_pat: "LCASE6".to_string(),
                my_type: 1,
                dir: 2,
                csys: "LOCAL".to_string(),
                rel_dist1: 0.0,
                rel_dist2: 1.0,
                val1: 0.9984 / 12.0,
                val2: 0.3744 / 12.0,
            });
        m.frames
            .get_mut("2")
            .unwrap()
            .loads_dist
            .push(FrameLoadDistributed {
                load_pat: "LCASE6".to_string(),
                my_type: 1,
                dir: 2,
                csys: "LOCAL".to_string(),
                rel_dist1: 0.0,
                rel_dist2: 1.0,
                val1: -0.3744 / 12.0,
                val2: 0.0,
            });
        m.frames
            .get_mut("2")
            .unwrap()
            .loads_point
            .push(FrameLoadPoint {
                load_pat: "LCASE7".to_string(),
                my_type: 1,
                dir: 2,
                csys: "LOCAL".to_string(),
                rel_dist: 0.5,
                val: -15.0,
            });

        self.replace_model(
            m,
            "Loaded Verification Example 1-001.".to_string(),
            Vec::new(),
        );
        self.active_case = "LCASE5".to_string();
        self.execute_analysis();
    }

    fn load_benchmark_1_004(&mut self) {
        let mut m = SapModel::new(Units::KipInF);
        m.model_name = "StructuraX Verification Example 1-004 (Rotated Local Axes 30 deg)".to_string();
        m.load_patterns.clear();
        m.load_cases.clear();
        let mut sec = FrameSection::default();
        sec.name = "W12X106".to_string();
        sec.area = 31.2;
        sec.i33 = 933.0;
        sec.i22 = 301.0;
        sec.as2 = 0.0;
        sec.as3 = 0.0;
        m.frame_sections.insert("W12X106".to_string(), sec);
        m.add_point("1", 0.0, 0.0, 0.0);
        m.points.get_mut("1").unwrap().restraint = [true; 6];
        m.add_point("2", 144.0, 0.0, 0.0);
        m.add_frame("1", "1", "2", "W12X106");
        m.frames.get_mut("1").unwrap().angle_deg = 30.0;

        for name in ["LC1", "LC2", "LC3"] {
            m.load_patterns.insert(
                name.to_string(),
                LoadPattern {
                    name: name.to_string(),
                    pat_type: LoadPatternType::Other,
                    self_wt_multiplier: 0.0,
                },
            );
            m.load_cases
                .insert(name.to_string(), LoadCase::linear_static(name, name, 1.0));
        }
        m.frames
            .get_mut("1")
            .unwrap()
            .loads_dist
            .push(FrameLoadDistributed {
                load_pat: "LC1".to_string(),
                my_type: 1,
                dir: 6,
                csys: "GLOBAL".to_string(),
                rel_dist1: 0.0,
                rel_dist2: 1.0,
                val1: -0.01,
                val2: -0.01,
            });
        m.points
            .get_mut("2")
            .unwrap()
            .loads_force
            .push(PointLoadForce {
                load_pat: "LC2".to_string(),
                csys: "GLOBAL".to_string(),
                force: [0.0, 0.0, -1.0, 0.0, 0.0, 0.0],
            });
        m.points
            .get_mut("2")
            .unwrap()
            .loads_force
            .push(PointLoadForce {
                load_pat: "LC3".to_string(),
                csys: "GLOBAL".to_string(),
                force: [0.0, 0.0, 0.0, 0.0, 240.0, 0.0],
            });
        self.replace_model(
            m,
            "Loaded Verification Example 1-004.".to_string(),
            Vec::new(),
        );
        self.active_case = "LC1".to_string();
        self.execute_analysis();
    }

    fn load_benchmark_1_016(&mut self) {
        let mut m = SapModel::new(Units::KipInF);
        m.model_name =
            "StructuraX Verification Example 1-016 (P-Delta Tension Stiffening)".to_string();
        let mut sec = make_rectangular("ROD", "A992Fy50", 3.0, 3.0);
        sec.as2 = 0.0;
        m.frame_sections.insert("ROD".to_string(), sec);
        m.add_point("1", 0.0, 0.0, 0.0);
        m.points.get_mut("1").unwrap().restraint = [true, true, true, true, false, true];
        m.add_point("2", 150.0, 0.0, 0.0);
        m.points.get_mut("2").unwrap().restraint = [false, true, false, true, false, true];
        m.add_point("3", 300.0, 0.0, 0.0);
        m.points.get_mut("3").unwrap().restraint = [false, true, true, true, false, true];
        m.add_frame("1", "1", "2", "ROD");
        m.add_frame("2", "2", "3", "ROD");
        for fid in ["1", "2"] {
            m.frames
                .get_mut(fid)
                .unwrap()
                .loads_dist
                .push(FrameLoadDistributed {
                    load_pat: "DEAD".to_string(),
                    my_type: 1,
                    dir: 6,
                    csys: "GLOBAL".to_string(),
                    rel_dist1: 0.0,
                    rel_dist2: 1.0,
                    val1: -0.002,
                    val2: -0.002,
                });
        }
        m.points
            .get_mut("3")
            .unwrap()
            .loads_force
            .push(PointLoadForce {
                load_pat: "DEAD".to_string(),
                csys: "GLOBAL".to_string(),
                force: [20.25, 0.0, 0.0, 0.0, 0.0, 0.0],
            });
        if let Some(lc) = m.load_cases.get_mut("DEAD") {
            lc.case_type = sap_model::LoadCaseType::NonlinearStatic;
            lc.nl_geom = sap_model::NLGeomType::PDelta;
        }
        self.replace_model(
            m,
            "Loaded Verification Example 1-016.".to_string(),
            Vec::new(),
        );
        self.active_case = "DEAD".to_string();
        self.execute_analysis();
    }

    fn render_modals(&mut self, ctx: &egui::Context) {
        // 1. New Model from Parametric Templates Wizard
        let mut open_tpl = self.show_new_template_modal;
        let mut close_tpl = false;
        if open_tpl {
            egui::Window::new("New Model — StructuraX Parametric Structural Templates")
                .open(&mut open_tpl)
                .resizable(false)
                .show(ctx, |ui| {
                    ui.horizontal(|ui| {
                        ui.selectable_value(&mut self.tpl_choice, TemplateChoice::Beam, "📏 Beam");
                        ui.selectable_value(
                            &mut self.tpl_choice,
                            TemplateChoice::Frame2D,
                            "🏗 2D Frame",
                        );
                        ui.selectable_value(
                            &mut self.tpl_choice,
                            TemplateChoice::Truss2D,
                            "🔺 2D Truss",
                        );
                        ui.selectable_value(
                            &mut self.tpl_choice,
                            TemplateChoice::Frame3D,
                            "🏢 3D Building",
                        );
                        ui.selectable_value(
                            &mut self.tpl_choice,
                            TemplateChoice::ShearWall,
                            "🧱 Shear Wall",
                        );
                        ui.selectable_value(
                            &mut self.tpl_choice,
                            TemplateChoice::SphericalDome,
                            "🌐 Shell Dome",
                        );
                    });
                    ui.separator();

                    egui::Grid::new("tpl_grid")
                        .num_columns(2)
                        .spacing([14.0, 8.0])
                        .show(ui, |ui| {
                            if self.tpl_choice == TemplateChoice::Frame2D {
                                ui.label("2D Frame Type:");
                                egui::ComboBox::from_id_salt("f2d_type")
                                    .selected_text(format!("{:?}", self.tpl_frame2d_type))
                                    .show_ui(ui, |ui| {
                                        ui.selectable_value(
                                            &mut self.tpl_frame2d_type,
                                            FrameType2D::PortalFrame,
                                            "Portal Frame",
                                        );
                                        ui.selectable_value(
                                            &mut self.tpl_frame2d_type,
                                            FrameType2D::ConcentricBraced,
                                            "Concentric Braced (CBF)",
                                        );
                                        ui.selectable_value(
                                            &mut self.tpl_frame2d_type,
                                            FrameType2D::EccentricBraced,
                                            "Eccentric Braced (EBF)",
                                        );
                                    });
                                ui.end_row();
                            }
                            if self.tpl_choice == TemplateChoice::Frame3D {
                                ui.label("3D Building Type:");
                                egui::ComboBox::from_id_salt("f3d_type")
                                    .selected_text(format!("{:?}", self.tpl_frame3d_type))
                                    .show_ui(ui, |ui| {
                                        ui.selectable_value(
                                            &mut self.tpl_frame3d_type,
                                            FrameType3D::OpenFrame,
                                            "Open Space Frame",
                                        );
                                        ui.selectable_value(
                                            &mut self.tpl_frame3d_type,
                                            FrameType3D::PerimeterFrame,
                                            "Perimeter Moment Frame",
                                        );
                                        ui.selectable_value(
                                            &mut self.tpl_frame3d_type,
                                            FrameType3D::BeamSlab,
                                            "Beam-Slab Building",
                                        );
                                        ui.selectable_value(
                                            &mut self.tpl_frame3d_type,
                                            FrameType3D::FlatPlate,
                                            "Flat Plate Slab",
                                        );
                                    });
                                ui.end_row();
                            }

                            ui.label("Number of Spans / X-Bays / Divs:");
                            ui.add(
                                egui::DragValue::new(&mut self.tpl_num_spans_or_bays).range(1..=24),
                            );
                            ui.end_row();

                            ui.label("Span / Bay Width X / Radius:");
                            ui.add(
                                egui::DragValue::new(&mut self.tpl_span_or_bay_len)
                                    .range(1e-3..=1e9)
                                    .speed(6.0),
                            );
                            ui.end_row();

                            if self.tpl_choice != TemplateChoice::Beam {
                                ui.label("Number of Stories / Z-Divs:");
                                ui.add(
                                    egui::DragValue::new(&mut self.tpl_num_stories).range(1..=30),
                                );
                                ui.end_row();

                                ui.label("Story / Truss Height Z:");
                                ui.add(
                                    egui::DragValue::new(&mut self.tpl_story_height)
                                        .range(1e-3..=1e9)
                                        .speed(6.0),
                                );
                                ui.end_row();
                            }

                            if self.tpl_choice == TemplateChoice::Frame3D {
                                ui.label("Number of Y-Bays:");
                                ui.add(
                                    egui::DragValue::new(&mut self.tpl_num_bays_y).range(1..=20),
                                );
                                ui.end_row();

                                ui.label("Bay Width Y:");
                                ui.add(
                                    egui::DragValue::new(&mut self.tpl_bay_width_y)
                                        .range(1e-3..=1e9)
                                        .speed(6.0),
                                );
                                ui.end_row();
                            }

                            ui.label("Base Restraints:");
                            ui.checkbox(&mut self.tpl_restraint, "Restrain Base Supports");
                            ui.end_row();
                        });

                    ui.separator();
                    if ui
                        .button("✓ Generate Structural Model & Run Analysis")
                        .clicked()
                    {
                        let units = self.model.present_units;
                        let mut m = SapModel::new(units);
                        for std_sec in ["W14X90", "W18X50"] {
                            if let Some(sec) = self.model.frame_sections.get(std_sec) {
                                m.frame_sections.insert(std_sec.to_string(), sec.clone());
                            }
                        }
                        if !m.frame_sections.contains_key("W14X90")
                            || !m.frame_sections.contains_key("W18X50")
                        {
                            if let Some(db) = resolve_pro_database_path("aisc14.pro")
                                .and_then(|p| ProDatabase::from_file(p).ok())
                            {
                                for std_sec in ["W14X90", "W18X50"] {
                                    if let Ok(sec) =
                                        db.get_section_scaled(std_sec, units, "A992Fy50")
                                    {
                                        m.frame_sections.insert(std_sec.to_string(), sec);
                                    }
                                }
                            }
                        }
                        let span = self.tpl_span_or_bay_len.max(1e-3);
                        let story_h = self.tpl_story_height.max(1e-3);
                        let bay_y = self.tpl_bay_width_y.max(1e-3);
                        match self.tpl_choice {
                            TemplateChoice::Beam => m.generate_beam(
                                self.tpl_num_spans_or_bays,
                                span,
                                self.tpl_restraint,
                                "W18X50",
                            ),
                            TemplateChoice::Frame2D => m.generate_2d_frame(
                                self.tpl_frame2d_type,
                                self.tpl_num_stories,
                                story_h,
                                self.tpl_num_spans_or_bays,
                                span,
                                self.tpl_restraint,
                                "W18X50",
                                "W14X90",
                                "W14X90",
                                (0.15 * span).max(1e-3),
                            ),
                            TemplateChoice::Truss2D => m.generate_2d_truss(
                                self.tpl_num_spans_or_bays,
                                span,
                                story_h,
                                self.tpl_restraint,
                                "W14X90",
                                "W14X90",
                            ),
                            TemplateChoice::Frame3D => m.generate_3d_frame(
                                self.tpl_frame3d_type,
                                self.tpl_num_stories,
                                story_h,
                                self.tpl_num_spans_or_bays,
                                span,
                                self.tpl_num_bays_y,
                                bay_y,
                                self.tpl_restraint,
                                "W18X50",
                                "W14X90",
                                "ASEC1",
                            ),
                            TemplateChoice::ShearWall => m.generate_wall(
                                self.tpl_num_spans_or_bays,
                                span,
                                self.tpl_num_stories,
                                story_h,
                                self.tpl_restraint,
                                "ASEC1",
                            ),
                            TemplateChoice::SphericalDome => m.generate_dome_shell(
                                span,
                                45.0,
                                self.tpl_num_stories.max(3),
                                self.tpl_num_spans_or_bays.max(8),
                                self.tpl_restraint,
                                "ASEC1",
                            ),
                        }
                        self.replace_model(
                            m,
                            format!("Generated {:?} parametric model.", self.tpl_choice),
                            Vec::new(),
                        );
                        self.execute_analysis();
                        close_tpl = true;
                    }
                });
        }
        self.show_new_template_modal = open_tpl && !close_tpl;

        // 2. Structural Verification Benchmark Suite Window
        let mut open_ver = self.show_verification_modal;
        if open_ver {
            egui::Window::new("StructuraX Reference Verification Benchmarks")
                .open(&mut open_ver)
                .default_width(720.0)
                .show(ctx, |ui| {
                    let passed = self.verification_rows.iter().filter(|r| r.passed).count();
                    let total = self.verification_rows.len();
                    let all_passed = total > 0 && passed == total;
                    let percent = if total == 0 {
                        0.0
                    } else {
                        100.0 * passed as f64 / total as f64
                    };
                    ui.label(
                        RichText::new(format!(
                            "Verification Status: {passed} / {total} reference checks passed ({percent:.1}%)"
                        ))
                        .color(if all_passed {
                            Color32::from_rgb(90, 235, 130)
                        } else {
                            Color32::from_rgb(255, 120, 100)
                        })
                        .strong(),
                    );
                    ui.label("Analytical and structural mechanics reference benchmarks across frame, shell, solid, cable, and nonlinear dynamic systems.");
                    ui.separator();
                    egui::ScrollArea::vertical().max_height(420.0).show(ui, |ui| {
                        egui::Grid::new("ver_grid")
                            .striped(true)
                            .spacing([16.0, 6.0])
                            .show(ui, |ui| {
                                ui.strong("Problem");
                                ui.strong("Load Case");
                                ui.strong("Parameter");
                                ui.strong("Target Ref");
                                ui.strong("StructuraX");
                                ui.strong("% Diff");
                                ui.strong("Status");
                                ui.end_row();

                                for row in &self.verification_rows {
                                    ui.label(&row.example);
                                    ui.label(&row.load_case);
                                    ui.label(&row.parameter);
                                    ui.monospace(format!("{:.5}", row.sap2000_ref));
                                    ui.monospace(format!("{:.5}", row.computed));
                                    ui.monospace(format!("{:.4}%", row.pct_diff));
                                    ui.colored_label(
                                        if row.passed {
                                            Color32::from_rgb(80, 235, 130)
                                        } else {
                                            Color32::RED
                                        },
                                        if row.passed { "PASS" } else { "FAIL" },
                                    );
                                    ui.end_row();
                                }
                            });
                    });
                });
        }
        self.show_verification_modal = open_ver;

        // 3. Member Station Diagram Inspector (Right-click or magnifying glass on Frame)
        if let Some(fid) = self.inspected_frame.clone() {
            let mut open_insp = true;
            egui::Window::new(format!(
                "Frame {fid} — Internal Force, Moment & Deflection Diagrams [{}]",
                self.active_case
            ))
            .open(&mut open_insp)
            .default_width(560.0)
            .show(ctx, |ui| {
                if let Some(cr) = self.model.case_results.get(&self.active_case) {
                    if let Some(stations) = cr.frame_forces.get(&fid) {
                        egui::ScrollArea::vertical().max_height(360.0).show(ui, |ui| {
                            egui::Grid::new("station_grid")
                                .striped(true)
                                .spacing([14.0, 4.0])
                                .show(ui, |ui| {
                                    ui.strong("Station");
                                    ui.strong("P (Axial)");
                                    ui.strong("V2 (Shear)");
                                    ui.strong("V3");
                                    ui.strong("T (Torque)");
                                    ui.strong("M2 (Minor)");
                                    ui.strong("M3 (Major)");
                                    ui.strong("u2 (Defl)");
                                    ui.end_row();
                                    for st in stations {
                                        ui.monospace(format!("{:.2}", st.station));
                                        ui.monospace(format!("{:.3}", st.p));
                                        ui.monospace(format!("{:.3}", st.v2));
                                        ui.monospace(format!("{:.3}", st.v3));
                                        ui.monospace(format!("{:.3}", st.t));
                                        ui.monospace(format!("{:.3}", st.m2));
                                        ui.monospace(format!("{:.3}", st.m3));
                                        ui.monospace(format!("{:.5}", st.local_disp[1]));
                                        ui.end_row();
                                    }
                                });
                        });
                    } else {
                        ui.label("No frame station forces for the selected case/combination.");
                    }
                } else {
                    ui.label("Run analysis (F5) to view station results.");
                }
            });
            if !open_insp {
                self.inspected_frame = None;
            }
        }

        // 4. Comprehensive Database Tables Modal (`Ctrl+T`)
        let mut open_tbl = self.show_tables_modal;
        if open_tbl {
            egui::Window::new(format!(
                "StructuraX Database Tables [Active Case/Combo: {}]",
                self.active_case
            ))
            .open(&mut open_tbl)
            .default_width(780.0)
            .show(ctx, |ui| {
                ui.horizontal_wrapped(|ui| {
                    for (tab, lbl) in [
                        (TableTab::JointDisplacements, "Joint Displ"),
                        (TableTab::JointReactions, "Joint React"),
                        (TableTab::BaseReactions, "Base React"),
                        (TableTab::FrameForces, "Frame Forces"),
                        (TableTab::AreaResults, "Shell / Area"),
                        (TableTab::SolidStresses, "Solid Stress"),
                        (TableTab::LinkForces, "Link Forces"),
                        (TableTab::ModalPeriods, "Modal"),
                        (TableTab::BucklingFactors, "Buckling"),
                        (TableTab::SteelDesign, "Steel Design"),
                        (TableTab::ConcreteDesign, "Concrete Design"),
                    ] {
                        ui.selectable_value(&mut self.tables_tab, tab, lbl);
                    }
                });
                ui.separator();

                egui::ScrollArea::vertical()
                    .max_height(420.0)
                    .show(ui, |ui| match self.tables_tab {
                        TableTab::SteelDesign => {
                            if self.model.steel_design_results.is_empty() {
                                ui.label("No steel design results — click 'Design > Steel Frame Design' first.");
                            } else {
                                egui::Grid::new("tbl_steel_design")
                                    .striped(true)
                                    .spacing([12.0, 4.0])
                                    .show(ui, |ui| {
                                        for h in [
                                            "Frame",
                                            "Section",
                                            "Code",
                                            "Combo",
                                            "Station",
                                            "Total D/C",
                                            "Axial",
                                            "Major M",
                                            "Minor M",
                                            "Shear V2",
                                            "Eq",
                                            "Status",
                                        ] {
                                            ui.strong(h);
                                        }
                                        ui.end_row();
                                        for res in self.model.steel_design_results.values() {
                                            ui.label(&res.frame_name);
                                            ui.label(&res.section_name);
                                            ui.label(&res.design_code);
                                            ui.label(&res.governing_combo);
                                            ui.monospace(format!("{:.2}", res.governing_station));
                                            ui.monospace(format!("{:.4}", res.total_ratio));
                                            ui.monospace(format!("{:.4}", res.pr_ratio));
                                            ui.monospace(format!("{:.4}", res.mr_major_ratio));
                                            ui.monospace(format!("{:.4}", res.mr_minor_ratio));
                                            ui.monospace(format!("{:.4}", res.shear_major_ratio));
                                            ui.label(&res.eq_name);
                                            ui.colored_label(steel_result_color(res), &res.status);
                                            ui.end_row();
                                        }
                                    });
                            }
                        }
                        TableTab::ConcreteDesign => {
                            if self.model.concrete_design_results.is_empty() {
                                ui.label("No concrete design results — click 'Design > Concrete Frame Design' first.");
                            } else {
                                egui::Grid::new("tbl_conc_design")
                                    .striped(true)
                                    .spacing([12.0, 4.0])
                                    .show(ui, |ui| {
                                        for h in [
                                            "Frame",
                                            "Section",
                                            "Type",
                                            "Code",
                                            "Combo",
                                            "As Top",
                                            "As Bot",
                                            "Av/s",
                                            "PMM Ratio",
                                            "Status",
                                        ] {
                                            ui.strong(h);
                                        }
                                        ui.end_row();
                                        for res in self.model.concrete_design_results.values() {
                                            ui.label(&res.frame_name);
                                            ui.label(&res.section_name);
                                            ui.label(if res.is_column { "Column" } else { "Beam" });
                                            ui.label(&res.design_code);
                                            ui.label(&res.governing_combo);
                                            ui.monospace(format!("{:.4}", res.as_top));
                                            ui.monospace(format!("{:.4}", res.as_bot));
                                            ui.monospace(format!("{:.5}", res.av_over_s));
                                            ui.monospace(if res.is_column {
                                                format!("{:.4}", res.pmm_ratio)
                                            } else {
                                                "-".to_string()
                                            });
                                            ui.colored_label(
                                                concrete_result_color(res),
                                                &res.status,
                                            );
                                            ui.end_row();
                                        }
                                    });
                            }
                        }
                        other_tab => {
                            let Some(cr) = self.model.case_results.get(&self.active_case) else {
                                ui.label("No analysis results for active case/combo — press F5 to run analysis.");
                                return;
                            };
                            match other_tab {
                                TableTab::JointDisplacements => {
                                    egui::Grid::new("disp_tbl")
                                        .striped(true)
                                        .spacing([14.0, 4.0])
                                        .show(ui, |ui| {
                                            for h in [
                                                "Joint", "U1", "U2", "U3", "R1 (rad)", "R2 (rad)",
                                                "R3 (rad)",
                                            ] {
                                                ui.strong(h);
                                            }
                                            ui.end_row();
                                            for (jid, u) in &cr.joint_displacements {
                                                ui.label(jid);
                                                for v in u {
                                                    ui.monospace(format!("{v:.5}"));
                                                }
                                                ui.end_row();
                                            }
                                        });
                                }
                                TableTab::JointReactions => {
                                    egui::Grid::new("react_tbl")
                                        .striped(true)
                                        .spacing([14.0, 4.0])
                                        .show(ui, |ui| {
                                            for h in ["Joint", "F1", "F2", "F3", "M1", "M2", "M3"]
                                            {
                                                ui.strong(h);
                                            }
                                            ui.end_row();
                                            for (jid, r) in &cr.joint_reactions {
                                                ui.label(jid);
                                                for v in r {
                                                    ui.monospace(format!("{v:.4}"));
                                                }
                                                ui.end_row();
                                            }
                                        });
                                }
                                TableTab::BaseReactions => {
                                    egui::Grid::new("base_react_tbl")
                                        .striped(true)
                                        .spacing([16.0, 4.0])
                                        .show(ui, |ui| {
                                            for h in [
                                                "Case/Combo",
                                                "Global FX",
                                                "Global FY",
                                                "Global FZ",
                                                "Global MX",
                                                "Global MY",
                                                "Global MZ",
                                            ] {
                                                ui.strong(h);
                                            }
                                            ui.end_row();
                                            ui.label(&self.active_case);
                                            for v in &cr.base_reactions {
                                                ui.monospace(format!("{v:.4}"));
                                            }
                                            ui.end_row();
                                        });
                                }
                                TableTab::FrameForces => {
                                    if cr.frame_forces.is_empty() {
                                        ui.label("No frame forces recorded for this case.");
                                    } else {
                                        egui::Grid::new("frame_force_tbl")
                                            .striped(true)
                                            .spacing([12.0, 4.0])
                                            .show(ui, |ui| {
                                                for h in [
                                                    "Frame", "Station", "P", "V2", "V3", "T", "M2",
                                                    "M3",
                                                ] {
                                                    ui.strong(h);
                                                }
                                                ui.end_row();
                                                for (fid, stations) in &cr.frame_forces {
                                                    for st in stations {
                                                        ui.label(fid);
                                                        ui.monospace(format!("{:.2}", st.station));
                                                        ui.monospace(format!("{:.3}", st.p));
                                                        ui.monospace(format!("{:.3}", st.v2));
                                                        ui.monospace(format!("{:.3}", st.v3));
                                                        ui.monospace(format!("{:.3}", st.t));
                                                        ui.monospace(format!("{:.3}", st.m2));
                                                        ui.monospace(format!("{:.3}", st.m3));
                                                        ui.end_row();
                                                    }
                                                }
                                            });
                                    }
                                }
                                TableTab::AreaResults => {
                                    if cr.area_results.is_empty() {
                                        ui.label("No area/shell results recorded for this case.");
                                    } else {
                                        egui::Grid::new("area_res_tbl")
                                            .striped(true)
                                            .spacing([12.0, 4.0])
                                            .show(ui, |ui| {
                                                for h in [
                                                    "Area", "F11", "F22", "F12", "M11", "M22",
                                                    "M12", "V13", "V23", "SVM Top",
                                                ] {
                                                    ui.strong(h);
                                                }
                                                ui.end_row();
                                                for (aid, ar) in &cr.area_results {
                                                    ui.label(aid);
                                                    for v in [
                                                        ar.f11, ar.f22, ar.f12, ar.m11, ar.m22,
                                                        ar.m12, ar.v13, ar.v23, ar.svm_top,
                                                    ] {
                                                        ui.monospace(format!("{v:.4}"));
                                                    }
                                                    ui.end_row();
                                                }
                                            });
                                    }
                                }
                                TableTab::SolidStresses => {
                                    if cr.solid_results.is_empty() {
                                        ui.label("No 3D solid stresses recorded for this case.");
                                    } else {
                                        egui::Grid::new("solid_res_tbl")
                                            .striped(true)
                                            .spacing([12.0, 4.0])
                                            .show(ui, |ui| {
                                                for h in [
                                                    "Solid", "S11", "S22", "S33", "S12", "S13",
                                                    "S23", "Von Mises",
                                                ] {
                                                    ui.strong(h);
                                                }
                                                ui.end_row();
                                                for (sid, sr) in &cr.solid_results {
                                                    let s = &sr.center_stress;
                                                    ui.label(sid);
                                                    for v in [
                                                        s.s11,
                                                        s.s22,
                                                        s.s33,
                                                        s.s12,
                                                        s.s13,
                                                        s.s23,
                                                        s.von_mises,
                                                    ] {
                                                        ui.monospace(format!("{v:.4}"));
                                                    }
                                                    ui.end_row();
                                                }
                                            });
                                    }
                                }
                                TableTab::LinkForces => {
                                    if cr.link_forces.is_empty() {
                                        ui.label("No link forces recorded for this case.");
                                    } else {
                                        egui::Grid::new("link_force_tbl")
                                            .striped(true)
                                            .spacing([14.0, 4.0])
                                            .show(ui, |ui| {
                                                for h in ["Link", "P", "V2", "V3", "T", "M2", "M3"]
                                                {
                                                    ui.strong(h);
                                                }
                                                ui.end_row();
                                                for (lid, lf) in &cr.link_forces {
                                                    ui.label(lid);
                                                    for v in lf {
                                                        ui.monospace(format!("{v:.4}"));
                                                    }
                                                    ui.end_row();
                                                }
                                            });
                                    }
                                }
                                TableTab::ModalPeriods => {
                                    if cr.modes.is_empty() {
                                        ui.label("Active case is not a Modal case (or has 0 solved modes).");
                                    } else {
                                        egui::Grid::new("modal_tbl")
                                            .striped(true)
                                            .spacing([14.0, 4.0])
                                            .show(ui, |ui| {
                                                for h in [
                                                    "Mode",
                                                    "Period (s)",
                                                    "Freq (Hz)",
                                                    "Eigenvalue",
                                                    "Mass UX",
                                                    "Mass UY",
                                                    "Mass UZ",
                                                ] {
                                                    ui.strong(h);
                                                }
                                                ui.end_row();
                                                for m in &cr.modes {
                                                    ui.monospace(format!("{}", m.mode_num));
                                                    ui.monospace(format!("{:.5}", m.period_sec));
                                                    ui.monospace(format!("{:.4}", m.freq_hz));
                                                    ui.monospace(format!("{:.4}", m.eigenvalue));
                                                    ui.monospace(format!(
                                                        "{:.4}",
                                                        m.modal_mass_ratios[0]
                                                    ));
                                                    ui.monospace(format!(
                                                        "{:.4}",
                                                        m.modal_mass_ratios[1]
                                                    ));
                                                    ui.monospace(format!(
                                                        "{:.4}",
                                                        m.modal_mass_ratios[2]
                                                    ));
                                                    ui.end_row();
                                                }
                                            });
                                    }
                                }
                                TableTab::BucklingFactors => {
                                    if cr.buckling_factors.is_empty() {
                                        ui.label("Active case is not a Buckling case (or has 0 buckling modes).");
                                    } else {
                                        egui::Grid::new("buckling_tbl")
                                            .striped(true)
                                            .spacing([16.0, 4.0])
                                            .show(ui, |ui| {
                                                ui.strong("Buckling Mode");
                                                ui.strong("Critical Factor λ");
                                                ui.end_row();
                                                for (idx, factor) in
                                                    cr.buckling_factors.iter().enumerate()
                                                {
                                                    ui.monospace(format!("{}", idx + 1));
                                                    ui.monospace(format!("{factor:.6}"));
                                                    ui.end_row();
                                                }
                                            });
                                    }
                                }
                                TableTab::SteelDesign | TableTab::ConcreteDesign => unreachable!(),
                            }
                        }
                    });
            });
        }
        self.show_tables_modal = open_tbl;

        // 5. Frame Sections & Binary `.pro` Importer Modal
        let mut open_sec = self.show_section_modal;
        if open_sec {
            egui::Window::new("Define Frame Sections & Import from .PRO Section Database")
                .open(&mut open_sec)
                .default_width(540.0)
                .show(ctx, |ui| {
                    let mat_names: Vec<String> = self.model.materials.keys().cloned().collect();
                    if !mat_names.contains(&self.custom_sec_material) {
                        if let Some(first) = mat_names.first() {
                            self.custom_sec_material = first.clone();
                        }
                    }
                    ui.horizontal(|ui| {
                        ui.label("Section Material:");
                        egui::ComboBox::from_id_salt("sec_mat_combo")
                            .selected_text(&self.custom_sec_material)
                            .show_ui(ui, |ui| {
                                for mname in &mat_names {
                                    ui.selectable_value(
                                        &mut self.custom_sec_material,
                                        mname.clone(),
                                        mname,
                                    );
                                }
                            });
                    });
                    ui.separator();

                    ui.heading("Import from Standard .PRO Section Library");
                    ui.horizontal(|ui| {
                        ui.label("Database (.pro):");
                        ui.text_edit_singleline(&mut self.pro_file_name);
                        if ui.button("Load .PRO Index").clicked() {
                            if let Some(p) = resolve_pro_database_path(&self.pro_file_name) {
                                match ProDatabase::from_file(p) {
                                    Ok(db) => {
                                        self.pro_available_names =
                                            db.sections.iter().map(|s| s.name.clone()).collect();
                                        self.status_msg = format!(
                                            "Loaded {} sections from {}",
                                            self.pro_available_names.len(),
                                            self.pro_file_name
                                        );
                                    }
                                    Err(err) => {
                                        self.status_msg = format!(
                                            "Failed to parse {}: {err}",
                                            self.pro_file_name
                                        );
                                    }
                                }
                            } else {
                                self.status_msg =
                                    format!("Cannot find .PRO database '{}'", self.pro_file_name);
                            }
                        }
                    });
                    ui.horizontal(|ui| {
                        ui.label("Search Filter:");
                        ui.text_edit_singleline(&mut self.pro_filter);
                    });
                    egui::ScrollArea::vertical()
                        .max_height(145.0)
                        .show(ui, |ui| {
                            let filt = self.pro_filter.to_ascii_uppercase();
                            let matching_names: Vec<_> = self
                                .pro_available_names
                                .iter()
                                .filter(|n| n.to_ascii_uppercase().contains(&filt))
                                .take(50)
                                .cloned()
                                .collect();
                            for sname in &matching_names {
                                if ui
                                    .button(format!("+ Import {sname} & Assign to Selected"))
                                    .clicked()
                                    && self.ensure_unlocked()
                                {
                                    if let Some(p) = resolve_pro_database_path(&self.pro_file_name)
                                    {
                                        if let Ok(db) = ProDatabase::from_file(p) {
                                            if let Ok(sec) = db.get_section_scaled(
                                                sname,
                                                self.model.present_units,
                                                &self.custom_sec_material,
                                            ) {
                                                self.invalidate_results();
                                                self.model
                                                    .frame_sections
                                                    .insert(sname.clone(), sec);
                                                for f in self
                                                    .model
                                                    .frames
                                                    .values_mut()
                                                    .filter(|f| f.selected)
                                                {
                                                    f.section = sname.clone();
                                                }
                                                self.status_msg = format!(
                                                    "Imported section {sname} ({}) from {}",
                                                    self.custom_sec_material, self.pro_file_name
                                                );
                                            }
                                        }
                                    }
                                }
                            }
                        });
                    ui.separator();
                    ui.heading("Create Parametric Frame Section");
                    egui::Grid::new("cust_sec")
                        .num_columns(2)
                        .spacing([12.0, 6.0])
                        .show(ui, |ui| {
                            ui.label("Name:");
                            ui.text_edit_singleline(&mut self.custom_sec_name);
                            ui.end_row();

                            ui.label("Shape:");
                            egui::ComboBox::from_id_salt("cust_sec_shape")
                                .selected_text(self.custom_sec_shape.label())
                                .show_ui(ui, |ui| {
                                    for sh in [
                                        CustomSectionShape::WideFlangeI,
                                        CustomSectionShape::Rectangular,
                                        CustomSectionShape::Circle,
                                        CustomSectionShape::Pipe,
                                        CustomSectionShape::BoxTube,
                                    ] {
                                        ui.selectable_value(
                                            &mut self.custom_sec_shape,
                                            sh,
                                            sh.label(),
                                        );
                                    }
                                });
                            ui.end_row();

                            match self.custom_sec_shape {
                                CustomSectionShape::WideFlangeI | CustomSectionShape::BoxTube => {
                                    ui.label("Depth t3:");
                                    ui.add(
                                        egui::DragValue::new(&mut self.custom_sec_d)
                                            .range(1e-4..=1e6),
                                    );
                                    ui.end_row();
                                    ui.label("Width t2:");
                                    ui.add(
                                        egui::DragValue::new(&mut self.custom_sec_b)
                                            .range(1e-4..=1e6),
                                    );
                                    ui.end_row();
                                    ui.label("Flange Thick tf:");
                                    ui.add(
                                        egui::DragValue::new(&mut self.custom_sec_tf)
                                            .range(1e-4..=1e6),
                                    );
                                    ui.end_row();
                                    ui.label("Web Thick tw:");
                                    ui.add(
                                        egui::DragValue::new(&mut self.custom_sec_tw)
                                            .range(1e-4..=1e6),
                                    );
                                    ui.end_row();
                                }
                                CustomSectionShape::Rectangular => {
                                    ui.label("Depth t3:");
                                    ui.add(
                                        egui::DragValue::new(&mut self.custom_sec_d)
                                            .range(1e-4..=1e6),
                                    );
                                    ui.end_row();
                                    ui.label("Width t2:");
                                    ui.add(
                                        egui::DragValue::new(&mut self.custom_sec_b)
                                            .range(1e-4..=1e6),
                                    );
                                    ui.end_row();
                                }
                                CustomSectionShape::Circle => {
                                    ui.label("Diameter d:");
                                    ui.add(
                                        egui::DragValue::new(&mut self.custom_sec_d)
                                            .range(1e-4..=1e6),
                                    );
                                    ui.end_row();
                                }
                                CustomSectionShape::Pipe => {
                                    ui.label("Outer Diameter d:");
                                    ui.add(
                                        egui::DragValue::new(&mut self.custom_sec_d)
                                            .range(1e-4..=1e6),
                                    );
                                    ui.end_row();
                                    ui.label("Wall Thick tw:");
                                    ui.add(
                                        egui::DragValue::new(&mut self.custom_sec_tw)
                                            .range(1e-4..=1e6),
                                    );
                                    ui.end_row();
                                }
                            }
                        });

                    let is_conc_mat = self
                        .model
                        .materials
                        .get(&self.custom_sec_material)
                        .is_some_and(|m| m.mat_type == MatType::Concrete);
                    if is_conc_mat
                        || matches!(
                            self.custom_sec_shape,
                            CustomSectionShape::Rectangular | CustomSectionShape::Circle
                        )
                    {
                        ui.checkbox(
                            &mut self.custom_sec_rebar_enabled,
                            "Include Concrete Reinforcement Metadata",
                        );
                        if self.custom_sec_rebar_enabled {
                            ui.horizontal(|ui| {
                                ui.checkbox(&mut self.custom_sec_is_column, "Column (PMM)");
                                ui.label("Cover:");
                                ui.add(
                                    egui::DragValue::new(&mut self.custom_sec_cover)
                                        .range(1e-4..=1e5),
                                );
                                if self.custom_sec_is_column {
                                    ui.label("Total As:");
                                    ui.add(
                                        egui::DragValue::new(&mut self.custom_sec_rebar_area)
                                            .range(1e-4..=1e6),
                                    );
                                }
                            });
                        }
                    }

                    if ui
                        .button("Add Custom Section (& Assign to Selected Frames)")
                        .clicked()
                        && self.ensure_unlocked()
                    {
                        let name = self.custom_sec_name.trim().to_string();
                        let d = self.custom_sec_d;
                        let b = self.custom_sec_b;
                        let tf = self.custom_sec_tf;
                        let tw = self.custom_sec_tw;
                        let geom_err = if name.is_empty() {
                            Some("Section name cannot be empty.")
                        } else if !self.model.materials.contains_key(&self.custom_sec_material) {
                            Some("Select an existing material before creating a section.")
                        } else {
                            match self.custom_sec_shape {
                                CustomSectionShape::WideFlangeI => {
                                    if !(d > 0.0
                                        && b > 0.0
                                        && tf > 0.0
                                        && tw > 0.0
                                        && 2.0 * tf < d
                                        && tw < b)
                                    {
                                        Some("Invalid I-section dimensions: require 2*tf < t3 and tw < t2.")
                                    } else {
                                        None
                                    }
                                }
                                CustomSectionShape::BoxTube => {
                                    if !(d > 0.0
                                        && b > 0.0
                                        && tf > 0.0
                                        && tw > 0.0
                                        && 2.0 * tf < d
                                        && 2.0 * tw < b)
                                    {
                                        Some("Invalid Box/HSS dimensions: require 2*tf < t3 and 2*tw < t2.")
                                    } else {
                                        None
                                    }
                                }
                                CustomSectionShape::Rectangular => {
                                    if !(d > 0.0 && b > 0.0) {
                                        Some("Rectangular depth and width must be positive.")
                                    } else {
                                        None
                                    }
                                }
                                CustomSectionShape::Circle => {
                                    if !d.is_finite() || d <= 0.0 {
                                        Some("Circular diameter must be positive.")
                                    } else {
                                        None
                                    }
                                }
                                CustomSectionShape::Pipe => {
                                    if !(d > 0.0 && tw > 0.0 && 2.0 * tw < d) {
                                        Some("Invalid Pipe dimensions: require 2*tw < outer diameter.")
                                    } else {
                                        None
                                    }
                                }
                            }
                        };
                        let rebar_err = if geom_err.is_none() && self.custom_sec_rebar_enabled {
                            if !(self.custom_sec_cover > 0.0 && 2.0 * self.custom_sec_cover < d) {
                                Some("Concrete cover must satisfy 0 < 2*cover < depth.")
                            } else if self.custom_sec_is_column
                                && (!self.custom_sec_rebar_area.is_finite()
                                    || self.custom_sec_rebar_area <= 0.0)
                            {
                                Some("Column longitudinal rebar area must be positive.")
                            } else {
                                None
                            }
                        } else {
                            None
                        };

                        if let Some(err) = geom_err.or(rebar_err) {
                            self.status_msg = err.to_string();
                        } else {
                            let mat = self.custom_sec_material.clone();
                            let mut sec = match self.custom_sec_shape {
                                CustomSectionShape::WideFlangeI => {
                                    make_i_section(&name, &mat, d, b, tf, tw, b, tf)
                                }
                                CustomSectionShape::Rectangular => {
                                    make_rectangular(&name, &mat, d, b)
                                }
                                CustomSectionShape::Circle => make_circle(&name, &mat, d),
                                CustomSectionShape::Pipe => make_pipe(&name, &mat, d, tw),
                                CustomSectionShape::BoxTube => {
                                    make_box_tube(&name, &mat, d, b, tf, tw)
                                }
                            };
                            if self.custom_sec_rebar_enabled {
                                sec.concrete_rebar = Some(ConcreteRebarConfig {
                                    is_column: self.custom_sec_is_column,
                                    rebar_mat_long: mat.clone(),
                                    rebar_mat_confine: mat.clone(),
                                    cover_top: self.custom_sec_cover,
                                    cover_bot: self.custom_sec_cover,
                                    num_bars_3dir: 3,
                                    num_bars_2dir: 3,
                                    total_rebar_area: if self.custom_sec_is_column {
                                        self.custom_sec_rebar_area
                                    } else {
                                        0.0
                                    },
                                });
                            }
                            self.invalidate_results();
                            self.model.frame_sections.insert(name.clone(), sec);
                            let mut assigned = 0usize;
                            for f in self.model.frames.values_mut().filter(|f| f.selected) {
                                f.section = name.clone();
                                assigned += 1;
                            }
                            self.status_msg = format!(
                                "Added section {name} ({}, {mat}) and assigned to {assigned} selected frames.",
                                self.custom_sec_shape.label()
                            );
                        }
                    }
                });
        }
        self.show_section_modal = open_sec;

        // 6. Open / Save File Modal
        let mut open_file = self.show_file_modal;
        let mut close_file = false;
        if open_file {
            egui::Window::new("Open / Save Structural Model (.s2k / .$2k / .sdb)")
                .open(&mut open_file)
                .default_width(540.0)
                .show(ctx, |ui| {
                    ui.label("File Path:");
                    ui.label("S2K preserves source-only data when it can be associated safely. Binary .SDB stream export writes the 64-section binary container.");
                    ui.text_edit_singleline(&mut self.file_path_input);
                    ui.horizontal(|ui| {
                        if ui.button("📂 Open Model").clicked() {
                            match load_model_with_report(&self.file_path_input) {
                                Ok(m) => {
                                    let mut status = format!("Opened {}", self.file_path_input);
                                    if !m.warnings.is_empty() {
                                        status.push_str(&format!(
                                            " — Import warnings: {}",
                                            m.warnings.join("; ")
                                        ));
                                    }
                                    self.replace_model(m.model, status, m.warnings);
                                    close_file = true;
                                }
                                Err(e) => {
                                    self.status_msg = format!("Open error: {e}");
                                }
                            }
                        }
                        if ui.button("💾 Save as .S2K").clicked() {
                            self.export_warnings.clear();
                            match save_s2k_file_with_report(&self.model, &self.file_path_input) {
                                Err(error) => self.status_msg = format!("Save error: {error}"),
                                Ok(warnings) => {
                                    self.status_msg =
                                        format!("Saved S2K to {}", self.file_path_input);
                                    if !warnings.is_empty() {
                                        self.status_msg.push_str(&format!(
                                            " — Export warnings: {}",
                                            warnings.join("; ")
                                        ));
                                    }
                                    self.export_warnings = warnings;
                                    close_file = true;
                                }
                            }
                        }
                        if ui.button("💾 Export Binary .SDB (new file)").clicked() {
                            self.export_warnings.clear();
                            match save_native_sdb_file(&self.model, &self.file_path_input) {
                                Err(error) => {
                                    self.status_msg = format!("Export SDB error: {error}")
                                }
                                Ok(()) => {
                                    self.export_warnings.push(
                                        "Experimental native SDB export: vendor application acceptance is unverified; retain the original source.".into(),
                                    );
                                    self.status_msg = format!(
                                        "Exported experimental binary stream .SDB to {} — vendor application acceptance is unverified",
                                        self.file_path_input
                                    );
                                    close_file = true;
                                }
                            }
                        }
                        if ui.button("💾 Save Private .SDB").clicked() {
                            self.export_warnings.clear();
                            match save_sdb_file_with_report(&self.model, &self.file_path_input) {
                                Err(error) => self.status_msg = format!("Save SDB error: {error}"),
                                Ok(warnings) => {
                                    self.status_msg = format!(
                                        "Saved private StructuraX container to {}.",
                                        self.file_path_input
                                    );
                                    if !warnings.is_empty() {
                                        self.status_msg.push_str(&format!(
                                            " — Export warnings: {}",
                                            warnings.join("; ")
                                        ));
                                    }
                                    self.export_warnings = warnings;
                                    close_file = true;
                                }
                            }
                        }
                    });
                });
        }
        self.show_file_modal = open_file && !close_file;

        if self.show_assign_load_modal {
            self.open_workflow(1);
            self.show_assign_load_modal = false;
        }
        self.render_workflows(ctx);
        self.render_analysis_editor(ctx);
    }

    pub(crate) fn render_ui(&mut self, ctx: &egui::Context) {
        if self.animate_deformed {
            self.anim_phase += 0.08;
            ctx.request_repaint();
        }

        // Top Menu & Ribbon
        egui::TopBottomPanel::top("sap_top_panel").show(ctx, |ui| {
            self.render_top_menu_and_ribbon(ctx, ui);
        });

        // Bottom Status Bar + SAP2000 Bottom-Right Units Selector
        egui::TopBottomPanel::bottom("sap_bottom_status").show(ctx, |ui| {
            ui.horizontal(|ui| {
                let lock_txt = if self.model.is_locked {
                    "🔒 Locked"
                } else {
                    "🔓 Unlocked"
                };
                ui.label(RichText::new(lock_txt).strong());
                ui.separator();
                ui.label(&self.status_msg);

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    // Bottom-Right Live Units Selector (1..=16)
                    let mut cur_u = self.model.present_units;
                    egui::ComboBox::from_id_salt("units_selector")
                        .selected_text(format!("Units: {}", cur_u.s2k_label()))
                        .show_ui(ui, |ui| {
                            for code in 1..=16 {
                                if let Ok(u) = Units::from_code(code) {
                                    if ui.selectable_value(&mut cur_u, u, u.s2k_label()).clicked()
                                        && u != self.model.present_units
                                        && self.ensure_unlocked()
                                    {
                                        let old_u = self.model.present_units;
                                        self.scale_gui_cached_units(old_u, u);
                                        self.model.convert_units_in_place(u);
                                        self.workflows.scale_for_units(old_u, u, &self.model);
                                        self.sync_active_case();
                                        self.status_msg = format!(
                                            "Converted model units to {}; results cleared, run analysis again.",
                                            u.s2k_label()
                                        );
                                    }
                                }
                            }
                        });
                    ui.separator();
                    ui.checkbox(&mut self.animate_deformed, "Animate");
                    ui.add(
                        egui::Slider::new(&mut self.deform_scale, 1.0..=500.0)
                            .text("Deform Scale"),
                    );
                    ui.add(
                        egui::Slider::new(&mut self.diagram_scale, 0.1..=10.0)
                            .text("Diagram Scale"),
                    );
                });
            });
        });

        // Left Model Explorer Tree
        egui::SidePanel::left("sap_left_explorer")
            .default_width(275.0)
            .resizable(true)
            .show(ctx, |ui| {
                self.render_left_explorer(ui);
            });

        // Central Synchronized 3D + 2D CAD Viewports
        egui::CentralPanel::default().show(ctx, |ui| {
            if self.dual_pane {
                ui.columns(2, |cols| {
                    let l_proj = self.left_proj;
                    let r_proj = self.right_proj;
                    self.render_viewport(&mut cols[0], l_proj, "3D Perspective View");
                    self.render_viewport(&mut cols[1], r_proj, "2D Plan / Elevation View");
                });
            } else {
                let l_proj = self.left_proj;
                self.render_viewport(ui, l_proj, "3D Perspective View");
            }
        });

        self.render_modals(ctx);
    }
}

impl eframe::App for SapGuiApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.render_ui(ctx);
    }
}

fn resolve_pro_database_path(pro_name: &str) -> Option<PathBuf> {
    let direct = PathBuf::from(pro_name);
    if direct.is_file() {
        return Some(direct);
    }

    let mut candidates = Vec::new();
    candidates.push(PathBuf::from("sections").join(pro_name));
    candidates.push(PathBuf::from("extracted/files").join(pro_name));

    if let Ok(exe) = std::env::current_exe() {
        if let Some(exe_dir) = exe.parent() {
            candidates.push(exe_dir.join("sections").join(pro_name));
            candidates.push(exe_dir.join("../share/structurax/sections").join(pro_name));
            candidates.push(exe_dir.join("../Resources/sections").join(pro_name));
        }
    }

    if let Ok(home) = std::env::var("HOME") {
        candidates.push(
            PathBuf::from(home)
                .join(".local/share/structurax/sections")
                .join(pro_name),
        );
    }

    candidates.push(PathBuf::from("/usr/local/share/structurax/sections").join(pro_name));
    candidates.push(PathBuf::from("/usr/share/structurax/sections").join(pro_name));

    candidates.into_iter().find(|p| p.is_file())
}

fn dc_ratio_color(ratio: f64) -> Color32 {
    if ratio <= 0.50 {
        Color32::from_rgb(70, 195, 255) // Cyan / Low
    } else if ratio <= 0.70 {
        Color32::from_rgb(80, 230, 120) // Green
    } else if ratio <= 0.90 {
        Color32::from_rgb(245, 220, 70) // Yellow
    } else if ratio <= 1.00 {
        Color32::from_rgb(255, 145, 50) // Orange
    } else {
        Color32::from_rgb(255, 65, 65) // Red (Overstressed > 1.0)
    }
}

fn steel_result_color(res: &SteelDesignResult) -> Color32 {
    if res.status != "OK" || !res.total_ratio.is_finite() {
        Color32::from_rgb(255, 65, 65)
    } else {
        dc_ratio_color(res.total_ratio)
    }
}

fn concrete_result_color(res: &ConcreteDesignResult) -> Color32 {
    if res.status != "OK" {
        Color32::from_rgb(255, 65, 65)
    } else if res.is_column {
        if res.pmm_ratio.is_finite() {
            dc_ratio_color(res.pmm_ratio)
        } else {
            Color32::from_rgb(255, 65, 65)
        }
    } else {
        Color32::from_rgb(80, 230, 120)
    }
}

fn normalized_contour_color(val: f64, lo: f64, hi: f64, alpha: u8) -> Color32 {
    if !val.is_finite() || !lo.is_finite() || !hi.is_finite() {
        return Color32::from_rgba_unmultiplied(90, 110, 140, alpha);
    }
    let span = hi - lo;
    let t = if span.abs() > 1e-15 {
        ((val - lo) / span).clamp(0.0, 1.0) as f32
    } else {
        0.5
    };
    let (r, g, b) = if t < 0.25 {
        let s = t / 0.25;
        (40.0, 90.0 + 150.0 * s, 245.0)
    } else if t < 0.50 {
        let s = (t - 0.25) / 0.25;
        (40.0 + 40.0 * s, 240.0, 245.0 * (1.0 - s) + 90.0 * s)
    } else if t < 0.75 {
        let s = (t - 0.50) / 0.25;
        (80.0 + 175.0 * s, 240.0 - 20.0 * s, 90.0 * (1.0 - s))
    } else {
        let s = (t - 0.75) / 0.25;
        (255.0, 220.0 * (1.0 - 0.72 * s), 45.0 * (1.0 - s))
    };
    Color32::from_rgba_unmultiplied(r as u8, g as u8, b as u8, alpha)
}

fn joint_disp_3d(
    cr: &CaseResults,
    pid: &str,
    scale: f64,
    mode_idx: usize,
    use_step: bool,
    step_idx: usize,
) -> [f64; 3] {
    let raw = if !cr.modes.is_empty() {
        cr.modes
            .get(mode_idx.min(cr.modes.len() - 1))
            .and_then(|m| m.mode_shape.get(pid))
            .copied()
    } else if use_step && !cr.time_steps.is_empty() {
        cr.time_history_disp
            .get(pid)
            .and_then(|steps| {
                if steps.is_empty() {
                    None
                } else {
                    steps.get(step_idx.min(steps.len() - 1)).copied()
                }
            })
            .or_else(|| cr.joint_displacements.get(pid).copied())
    } else {
        cr.joint_displacements.get(pid).copied()
    };
    if let Some(u) = raw {
        let dx = if u[0].is_finite() { u[0] * scale } else { 0.0 };
        let dy = if u[1].is_finite() { u[1] * scale } else { 0.0 };
        let dz = if u[2].is_finite() { u[2] * scale } else { 0.0 };
        [dx, dy, dz]
    } else {
        [0.0; 3]
    }
}

fn area_unit_normal(pts: &[[f64; 3]]) -> [f64; 3] {
    if pts.len() < 3 {
        return [0.0, 0.0, 1.0];
    }
    let u = [
        pts[1][0] - pts[0][0],
        pts[1][1] - pts[0][1],
        pts[1][2] - pts[0][2],
    ];
    let v = [
        pts[2][0] - pts[0][0],
        pts[2][1] - pts[0][1],
        pts[2][2] - pts[0][2],
    ];
    let nx = u[1] * v[2] - u[2] * v[1];
    let ny = u[2] * v[0] - u[0] * v[2];
    let nz = u[0] * v[1] - u[1] * v[0];
    let len = nx.hypot(ny).hypot(nz);
    if len > 1e-12 && len.is_finite() {
        [nx / len, ny / len, nz / len]
    } else {
        [0.0, 0.0, 1.0]
    }
}

fn area_load_dir_3d(dir: i32, normal: [f64; 3], val: f64) -> [f64; 3] {
    let s = val.signum();
    match dir {
        3 => [normal[0] * s, normal[1] * s, normal[2] * s],
        4 | 7 => [s, 0.0, 0.0],
        5 | 8 => [0.0, s, 0.0],
        6 | 9 => [0.0, 0.0, s],
        10 | 11 => [0.0, 0.0, -s],
        _ => [0.0, 0.0, s],
    }
}

fn frame_load_dir_3d(dir: i32, e_axes: &[[f64; 3]; 3], val: f64) -> [f64; 3] {
    let s = val.signum();
    match dir {
        1 => [e_axes[0][0] * s, e_axes[0][1] * s, e_axes[0][2] * s],
        2 => [e_axes[1][0] * s, e_axes[1][1] * s, e_axes[1][2] * s],
        3 => [e_axes[2][0] * s, e_axes[2][1] * s, e_axes[2][2] * s],
        4 | 7 => [s, 0.0, 0.0],
        5 | 8 => [0.0, s, 0.0],
        6 | 9 => [0.0, 0.0, s],
        10 | 11 => [0.0, 0.0, -s],
        _ => [0.0, 0.0, s],
    }
}

fn draw_world_force_arrow(
    painter: &egui::Painter,
    project: &dyn Fn([f64; 3]) -> Pos2,
    tip_world: [f64; 3],
    unit_dir: [f64; 3],
    span: f64,
) {
    let arrow_len = span * 0.055;
    let tail_world = [
        tip_world[0] - unit_dir[0] * arrow_len,
        tip_world[1] - unit_dir[1] * arrow_len,
        tip_world[2] - unit_dir[2] * arrow_len,
    ];
    let tip = project(tip_world);
    let tail = project(tail_world);
    let vec_scr = tip - tail;
    if vec_scr.length() > 2.5 {
        painter.arrow(
            tail,
            vec_scr,
            Stroke::new(1.5_f32, Color32::from_rgb(255, 175, 55)),
        );
    } else {
        painter.circle_filled(tip, 2.5, Color32::from_rgb(255, 175, 55));
    }
}

fn point_to_segment_dist(p: Pos2, a: Pos2, b: Pos2) -> f32 {
    let ab = b - a;
    let len_sq = ab.length_sq();
    if !len_sq.is_finite() || len_sq <= 1e-6 {
        return p.distance(a);
    }
    let t = ((p - a).dot(ab) / len_sq).clamp(0.0, 1.0);
    p.distance(a + ab * t)
}

fn point_in_polygon(p: Pos2, poly: &[Pos2]) -> bool {
    if poly.len() < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = poly.len() - 1;
    for i in 0..poly.len() {
        let pi = poly[i];
        let pj = poly[j];
        if ((pi.y > p.y) != (pj.y > p.y))
            && (p.x < (pj.x - pi.x) * (p.y - pi.y) / (pj.y - pi.y) + pi.x)
        {
            inside = !inside;
        }
        j = i;
    }
    inside
}

#[cfg(test)]
mod tests {
    use super::*;
    use sap_model::{ComboType, FrameStationResult, LoadCombo};

    fn blank_app() -> SapGuiApp {
        let mut app = SapGuiApp::new(Some(PathBuf::from("/nonexistent/sap-review-model.s2k")));
        app.model = SapModel::new(Units::KipInF);
        app
    }

    #[test]
    fn startup_file_errors_remain_visible() {
        let app = blank_app();
        assert!(app.status_msg.starts_with("Cannot open"));
    }

    #[test]
    fn failed_analysis_prevents_design_success() {
        let mut app = blank_app();
        app.execute_design(true);
        assert!(
            app.status_msg.starts_with("Analysis Error:"),
            "{}",
            app.status_msg
        );
        assert!(app.model.steel_design_results.is_empty());
        assert!(!app.model.is_locked);
    }

    #[test]
    fn deleting_joint_removes_connected_members_and_old_results() {
        let mut app = blank_app();
        app.model.add_point("1", 0.0, 0.0, 0.0);
        app.model.add_point("2", 1.0, 0.0, 0.0);
        app.model.add_point("3", 0.0, 1.0, 0.0);
        app.model.add_frame("1", "1", "2", "FSEC1");
        app.model.add_frame("3", "2", "3", "FSEC1");
        app.model
            .add_area("1", vec!["1".into(), "2".into(), "3".into()], "ASEC1");
        app.model.points.get_mut("1").unwrap().selected = true;
        app.model
            .case_results
            .insert("stale".into(), Default::default());
        app.model.is_locked = true;
        app.delete_selected_objects();
        assert_eq!(
            app.model.frames.keys().cloned().collect::<Vec<_>>(),
            vec!["3"]
        );
        assert!(app.model.areas.is_empty());
        assert!(app.model.case_results.is_empty());
        assert!(!app.model.is_locked);
        let name = sap_model::next_numeric_name(app.model.frames.keys());
        app.model.add_frame(name, "2", "3", "FSEC1");
        assert_eq!(app.model.frames.len(), 2);
        assert_eq!(app.model.frames["3"].point_i, "2");
    }

    #[test]
    fn design_colors_flag_overstressed_beams_and_slender_steel_members() {
        let red = Color32::from_rgb(255, 65, 65);
        let green = Color32::from_rgb(80, 230, 120);

        let beam_fail = ConcreteDesignResult {
            frame_name: "B1".into(),
            section_name: "RCBEAM".into(),
            design_code: "ACI 318-14".into(),
            governing_combo: "DEAD".into(),
            is_column: false,
            as_top: 2.5,
            as_bot: 1.8,
            av_over_s: 0.12,
            pmm_ratio: 0.0,
            min_rebar_area: 0.6,
            status: "OVERSTRESSED".into(),
        };
        assert_eq!(concrete_result_color(&beam_fail), red);

        let mut beam_ok = beam_fail.clone();
        beam_ok.status = "OK".into();
        assert_eq!(concrete_result_color(&beam_ok), green);

        let steel_slender = SteelDesignResult {
            frame_name: "C1".into(),
            section_name: "W14X90".into(),
            design_code: "AISC 360-10".into(),
            governing_combo: "DEAD".into(),
            governing_station: 0.0,
            total_ratio: 0.42,
            pr_ratio: 0.42,
            mr_major_ratio: 0.0,
            mr_minor_ratio: 0.0,
            shear_major_ratio: 0.0,
            shear_minor_ratio: 0.0,
            phi_pnc: 500.0,
            phi_pnt: 600.0,
            phi_mn33: 1000.0,
            phi_mn22: 500.0,
            phi_vn2: 200.0,
            phi_vn3: 150.0,
            status: "SLENDER".into(),
            eq_name: "KL/r>200".into(),
        };
        assert_eq!(steel_result_color(&steel_slender), red);
    }

    #[test]
    fn viewport_handles_nan_station_results_and_extreme_coordinates() {
        let mut app = blank_app();
        app.model.add_point("1", 0.0, 0.0, 0.0);
        app.model.add_point("2", 1.0e150, 0.0, 0.0);
        app.model.add_frame("1", "1", "2", "FSEC1");

        let mut cr = CaseResults {
            case_name: "DEAD".into(),
            ..Default::default()
        };
        cr.frame_forces.insert(
            "1".into(),
            vec![
                FrameStationResult {
                    station: 0.0,
                    rel_dist: 0.0,
                    m3: f64::NAN,
                    ..Default::default()
                },
                FrameStationResult {
                    station: 1.0,
                    rel_dist: 1.0,
                    m3: 125.0,
                    ..Default::default()
                },
            ],
        );
        app.model.case_results.insert("DEAD".into(), cr);
        app.active_case = "DEAD".into();
        app.display_mode = DisplayMode::FrameDiagram(FrameForceComponent::MomentM3);
        app.show_tables_modal = true;
        app.show_section_modal = true;
        app.show_new_template_modal = true;

        let ctx = egui::Context::default();
        let _ = ctx.run(egui::RawInput::default(), |ctx| {
            app.render_ui(ctx);
        });
    }

    #[test]
    fn replace_model_and_unit_scaling_keep_gui_state_consistent() {
        let mut app = blank_app();
        app.model.load_combos.insert(
            "OLD_COMB".into(),
            LoadCombo {
                name: "OLD_COMB".into(),
                combo_type: ComboType::LinearAdd,
                cases: vec![("DEAD".into(), 1.4)],
            },
        );
        app.inspected_frame = Some("STALE_FRAME".into());
        app.active_case = "OLD_COMB".into();
        app.new_pt_coords = [120.0, 0.0, 240.0];

        let mut next = SapModel::new(Units::KnMC);
        next.add_point("1", 0.0, 0.0, 0.0);
        app.replace_model(next, "Switched to SI".into(), Vec::new());

        assert!(app.model.load_combos.is_empty());
        assert!(app.inspected_frame.is_none());
        assert_eq!(app.active_case, "DEAD");
        // 240 in -> 6.096 m
        assert!((app.tpl_span_or_bay_len - 6.096).abs() < 1e-6);
        assert!((app.new_pt_coords[0] - 3.048).abs() < 1e-6);
    }

    #[test]
    fn restraint_assignment_guards_locked_model_and_empty_selection() {
        let mut app = blank_app();
        app.model.add_point("1", 0.0, 0.0, 0.0);
        app.model
            .case_results
            .insert("DEAD".into(), CaseResults::default());

        // 0 selected joints -> must not clear case_results
        app.assign_restraint_to_selected([true; 6], "Fixed");
        assert_eq!(app.model.case_results.len(), 1);

        // Locked model -> must not clear case_results or mutate restraint
        app.model.points.get_mut("1").unwrap().selected = true;
        app.model.is_locked = true;
        app.assign_restraint_to_selected([true; 6], "Fixed");
        assert_eq!(app.model.case_results.len(), 1);
        assert_eq!(app.model.points["1"].restraint, [false; 6]);
    }
}
