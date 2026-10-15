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
    FrameLoadDistributed, FrameLoadPoint, FrameSection, FrameType2D, FrameType3D, LoadCase,
    LoadPattern, LoadPatternType, Material, PointLoadForce, SapModel,
};
use sap_section::{make_i_section, make_rectangular, ProDatabase};
use sap_solver::{
    analyze_model,
    benchmarks::{run_verification_suite, BenchmarkVerificationRow},
    frame::frame_local_axes,
};
use sap_units::Units;
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
    pub custom_sec_name: String,
    pub custom_sec_d: f64,
    pub custom_sec_b: f64,
    pub custom_sec_tf: f64,
    pub custom_sec_tw: f64,

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
            // Add default gravity distributed load + lateral seismic/wind load for instant analysis & design
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
            .next()
            .cloned()
            .unwrap_or_else(|| "DEAD".to_string());

        Self {
            model,
            status_msg,
            import_warnings,
            export_warnings: Vec::new(),
            tool_mode: ToolMode::Select,
            display_mode: DisplayMode::FrameDiagram(FrameForceComponent::MomentM3),
            active_case,
            active_mode_idx: 0,
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
            show_restraints: true,
            show_loads: true,
            show_extruded: false,
            draw_first_point: None,
            draw_area_points: Vec::new(),
            new_pt_coords: [0.0, 0.0, 0.0],
            inspected_frame: None,
            show_new_template_modal: false,
            show_tables_modal: false,
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
            custom_sec_name: "W24X76_CUSTOM".to_string(),
            custom_sec_d: 24.0,
            custom_sec_b: 9.0,
            custom_sec_tf: 0.68,
            custom_sec_tw: 0.44,
            workflows: workflows::Workflows::default(),
            analysis_editor: analysis::AnalysisEditor::default(),
            verification_rows: Vec::new(),
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
                self.status_msg = format!(
                    "Analysis Complete: {} joints, {} frames, {} areas solved across {} cases.",
                    self.model.points.len(),
                    self.model.frames.len(),
                    self.model.areas.len(),
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
        let count = if steel {
            if let Err(error) = try_run_steel_design(&mut self.model) {
                self.status_msg = format!("Design Error: {error}");
                return;
            }
            self.display_mode = DisplayMode::DesignRatioSteel;
            self.model.steel_design_results.len()
        } else {
            if let Err(error) = try_run_concrete_design(&mut self.model) {
                self.status_msg = format!("Design Error: {error}");
                return;
            }
            self.display_mode = DisplayMode::DesignRatioConcrete;
            self.model.concrete_design_results.len()
        };
        self.status_msg = if count == 0 {
            "No applicable members with analysis forces were available for design.".to_string()
        } else {
            format!("Design calculations completed for {count} members.")
        };
    }

    fn render_top_menu_and_ribbon(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        // Keyboard shortcuts
        if ctx.input(|i| i.key_pressed(egui::Key::F5)) {
            self.execute_analysis();
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
                    self.model = SapModel::new(u);
                    self.import_warnings.clear();
        self.export_warnings.clear();
                    self.status_msg = "Initialized blank model.".to_string();
                    ui.close_menu();
                }
                ui.separator();
                if ui.button("Open / Save Model (.s2k / .sdb)...").clicked() {
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
                if ui.button("Select All Joints, Frames and Areas").clicked() {
                    for p in self.model.points.values_mut() { p.selected = true; }
                    for f in self.model.frames.values_mut() { f.selected = true; }
                    for a in self.model.areas.values_mut() { a.selected = true; }
                    ui.close_menu();
                }
                if ui.button("Clear Selection").clicked() {
                    for p in self.model.points.values_mut() { p.selected = false; }
                    for f in self.model.frames.values_mut() { f.selected = false; }
                    for a in self.model.areas.values_mut() { a.selected = false; }
                    ui.close_menu();
                }
                if ui.button("Delete Selected Objects (Del)").clicked() {
                    self.delete_selected_objects();
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
                for (label, tab) in [("Materials...", 5), ("Load Patterns...", 1), ("Area Sections and Modifiers...", 2), ("Load Combinations...", 4)] {
                    if ui.button(label).clicked() { self.open_workflow(tab); ui.close_menu(); }
                }
                if ui.button("Frame Sections & .PRO Library...").clicked() {
                    self.show_section_modal = true;
                    ui.close_menu();
                }
                if ui.button("Database Tables Explorer...").clicked() {
                    self.show_tables_modal = true;
                    ui.close_menu();
                }
            });

            ui.menu_button("Assign", |ui| {
                for (label, tab) in [("Frame Section...", 0), ("Area Section and Mesh...", 2), ("Frame End Releases...", 3)] {
                    if ui.button(label).clicked() { self.open_workflow(tab); ui.close_menu(); }
                }
                if ui.button("Assign Joint / Frame / Area Loads...").clicked() {
                    self.show_assign_load_modal = true;
                    ui.close_menu();
                }
                ui.separator();
                if ui.button("Restraint: Fixed Support (All 6 DOFs)").clicked() {
                    self.invalidate_results();
                    for p in self.model.points.values_mut().filter(|p| p.selected) {
                        p.restraint = [true; 6];
                    }
                    ui.close_menu();
                }
                if ui
                    .button("Restraint: Pinned Support (UX, UY, UZ)")
                    .clicked()
                {
                    self.invalidate_results();
                    for p in self.model.points.values_mut().filter(|p| p.selected) {
                        p.restraint = [true, true, true, false, false, false];
                    }
                    ui.close_menu();
                }
                if ui.button("Restraint: Roller Support (UZ only)").clicked() {
                    self.invalidate_results();
                    for p in self.model.points.values_mut().filter(|p| p.selected) {
                        p.restraint = [false, false, true, false, false, false];
                    }
                    ui.close_menu();
                }
                if ui.button("Restraint: Free (Release All)").clicked() {
                    self.invalidate_results();
                    for p in self.model.points.values_mut().filter(|p| p.selected) {
                        p.restraint = [false; 6];
                    }
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
                    self.invalidate_results();
                    self.model.active_dofs = [true; 6];
                    ui.close_menu();
                }
                if ui.button("Set Active DOFs: 2D XZ Plane Frame").clicked() {
                    self.invalidate_results();
                    self.model.active_dofs = [true, false, true, false, true, false];
                    ui.close_menu();
                }
                if ui.button("Set Active DOFs: 2D XY Plane Grid").clicked() {
                    self.invalidate_results();
                    self.model.active_dofs = [false, false, true, true, true, false];
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
                if ui.button("Show Database Tables... (Ctrl+T)").clicked() {
                    self.show_tables_modal = true;
                    ui.close_menu();
                }
            });

            ui.menu_button("Design", |ui| {
                if ui
                    .button("▶ Start Steel Frame Design (AISC 360-10)")
                    .clicked()
                {
                    self.model.steel_design_code = "AISC 360-10".to_string();
                    self.execute_design(true);
                    ui.close_menu();
                }
                if ui
                    .button("▶ Start Steel Frame Design (Eurocode 3-2005)")
                    .clicked()
                {
                    self.model.steel_design_code = "Eurocode 3-2005".to_string();
                    self.execute_design(true);
                    ui.close_menu();
                }
                ui.separator();
                if ui
                    .button("▶ Start Concrete Frame Design (ACI 318-14)")
                    .clicked()
                {
                    self.model.concrete_design_code = "ACI 318-14".to_string();
                    self.execute_design(false);
                    ui.close_menu();
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
                "D/C Ratio",
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
            ui.collapsing(format!("Export warnings ({})",self.export_warnings.len()),|ui| {
                for warning in &self.export_warnings {ui.label(warning);}
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
            if ui.button("+ Add Joint at (X,Y,Z)").clicked() {
                let id = sap_model::next_numeric_name(self.model.points.keys());
                self.invalidate_results();
                self.model.add_point(
                    &id,
                    self.new_pt_coords[0],
                    self.new_pt_coords[1],
                    self.new_pt_coords[2],
                );
                self.status_msg = format!("Added Joint {id}");
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
                            ui.checkbox(
                                &mut fr.selected,
                                format!("Fr {id}: {}->{} [{}]", fr.point_i, fr.point_j, fr.section),
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
                        if ui
                            .selectable_label(is_act, format!("{name} ({:?})", lc.case_type))
                            .clicked()
                        {
                            self.active_case = name.clone();
                        }
                    }
                });

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
            }

            if !self.model.steel_design_results.is_empty() {
                egui::CollapsingHeader::new(format!(
                    "Steel Design ({})",
                    self.model.steel_design_code
                ))
                .default_open(true)
                .show(ui, |ui| {
                    for (fid, res) in &self.model.steel_design_results {
                        let col = dc_ratio_color(res.total_ratio);
                        ui.horizontal(|ui| {
                            ui.colored_label(
                                col,
                                format!(
                                    "Fr {fid} ({}): D/C = {:.3} [{}]",
                                    res.section_name, res.total_ratio, res.eq_name
                                ),
                            );
                        });
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

        // Compute model bounding box
        let (min_pt, max_pt) = self.compute_bounds();
        let center_3d = [
            0.5 * (min_pt[0] + max_pt[0]),
            0.5 * (min_pt[1] + max_pt[1]),
            0.5 * (min_pt[2] + max_pt[2]),
        ];
        let span = ((max_pt[0] - min_pt[0]).powi(2)
            + (max_pt[1] - min_pt[1]).powi(2)
            + (max_pt[2] - min_pt[2]).powi(2))
        .sqrt()
        .max(10.0) as f32;
        let base_scale = (rect.width().min(rect.height()) * 0.62 / span) * self.zoom;
        let screen_center = rect.center() + self.pan;

        let project = |p: [f64; 3]| -> Pos2 {
            let dx = (p[0] - center_3d[0]) as f32;
            let dy = (p[1] - center_3d[1]) as f32;
            let dz = (p[2] - center_3d[2]) as f32;
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
            Pos2::new(
                screen_center.x + sx * base_scale,
                screen_center.y + sy * base_scale,
            )
        };

        // 1. Draw Grid Lines & Bubbles
        if self.show_grids {
            if let Some(cs) = self.model.coord_systems.get("GLOBAL") {
                let grid_col = Color32::from_rgba_unmultiplied(70, 85, 110, 120);
                for g in &cs.grids {
                    if !g.visible {
                        continue;
                    }
                    let (p_a, p_b) = match g.axis.as_str() {
                        "X" => (
                            [g.coordinate, min_pt[1], min_pt[2]],
                            [g.coordinate, max_pt[1], max_pt[2]],
                        ),
                        "Y" => (
                            [min_pt[0], g.coordinate, min_pt[2]],
                            [max_pt[0], g.coordinate, min_pt[2]],
                        ),
                        "Z" => (
                            [min_pt[0], 0.0, g.coordinate],
                            [max_pt[0], 0.0, g.coordinate],
                        ),
                        _ => continue,
                    };
                    let sa = project(p_a);
                    let sb = project(p_b);
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

        // 2. Draw Area / Shell Objects
        let case_res = self.model.case_results.get(&self.active_case);
        for (aid, area) in &self.model.areas {
            let mut poly = Vec::with_capacity(area.points.len());
            for pid in &area.points {
                if let Some(pt) = self.model.points.get(pid) {
                    poly.push(project([pt.x, pt.y, pt.z]));
                }
            }
            if poly.len() >= 3 {
                let fill_col = if let DisplayMode::ShellContour(scomp) = self.display_mode {
                    if let Some(cr) = case_res {
                        if let Some(ar) = cr.area_results.get(aid) {
                            let val = match scomp {
                                ShellComponent::F11 => ar.f11,
                                ShellComponent::F22 => ar.f22,
                                ShellComponent::M11 => ar.m11,
                                ShellComponent::M22 => ar.m22,
                                ShellComponent::SvmTop => ar.svm_top,
                            };
                            contour_color(val)
                        } else {
                            Color32::from_rgba_unmultiplied(60, 120, 180, 65)
                        }
                    } else {
                        Color32::from_rgba_unmultiplied(60, 120, 180, 65)
                    }
                } else if area.selected {
                    Color32::from_rgba_unmultiplied(255, 200, 60, 90)
                } else {
                    Color32::from_rgba_unmultiplied(60, 120, 180, 55)
                };
                painter.add(Shape::convex_polygon(
                    poly.clone(),
                    fill_col,
                    Stroke::new(1.2_f32, Color32::from_rgb(100, 160, 220)),
                ));
                if self.show_area_labels {
                    let mut cx = 0.0;
                    let mut cy = 0.0;
                    for p in &poly {
                        cx += p.x;
                        cy += p.y;
                    }
                    let c = Pos2::new(cx / poly.len() as f32, cy / poly.len() as f32);
                    painter.text(
                        c,
                        egui::Align2::CENTER_CENTER,
                        format!("A{aid}"),
                        FontId::proportional(11.0),
                        Color32::from_rgb(160, 220, 255),
                    );
                }
            }
        }

        // Compute max diagram value for normalization if in FrameDiagram mode
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
                        if v > max_diag_val {
                            max_diag_val = v;
                        }
                    }
                }
            }
        }

        // Animation phase multiplier
        let anim_mult = if self.animate_deformed {
            self.anim_phase.sin() as f64
        } else {
            1.0
        };

        // 3. Draw Frame Elements, Deformed Curves, Force/Moment Diagrams, or Design Colors
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

            // Base line color
            let (line_col, width): (Color32, f32) = match self.display_mode {
                DisplayMode::DesignRatioSteel => {
                    if let Some(res) = self.model.steel_design_results.get(fid) {
                        (dc_ratio_color(res.total_ratio), 3.5)
                    } else {
                        (Color32::from_rgb(130, 150, 175), 2.0)
                    }
                }
                DisplayMode::DesignRatioConcrete => {
                    if let Some(res) = self.model.concrete_design_results.get(fid) {
                        (dc_ratio_color(res.pmm_ratio.max(0.25)), 3.5)
                    } else {
                        (Color32::from_rgb(130, 150, 175), 2.0)
                    }
                }
                DisplayMode::Deformed => (Color32::from_rgba_unmultiplied(110, 125, 145, 110), 1.2),
                _ => {
                    if frame.selected {
                        (Color32::from_rgb(255, 215, 0), 3.2)
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

            // Frame label or D/C label at midpoint
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
                    painter.text(
                        mid_s + Vec2::new(0.0, -6.0),
                        egui::Align2::CENTER_BOTTOM,
                        format!("{:.3}", res.total_ratio),
                        FontId::proportional(11.0),
                        dc_ratio_color(res.total_ratio),
                    );
                }
            }

            // Applied distributed load arrows on frame
            if self.show_loads && self.display_mode == DisplayMode::Undeformed {
                for ld in &frame.loads_dist {
                    if ld.load_pat == self.active_case {
                        for k in 0..=4 {
                            let t = k as f64 / 4.0;
                            let px = [
                                p1[0] + t * (p2[0] - p1[0]),
                                p1[1] + t * (p2[1] - p1[1]),
                                p1[2] + t * (p2[2] - p1[2]),
                            ];
                            let tip = project(px);
                            let tail = tip + Vec2::new(0.0, -18.0);
                            painter.arrow(
                                tail,
                                tip - tail,
                                Stroke::new(1.3_f32, Color32::from_rgb(255, 165, 60)),
                            );
                        }
                    }
                }
            }

            // Deformed shape curve
            if self.display_mode == DisplayMode::Deformed {
                if let Some(cr) = case_res {
                    let scale = (self.deform_scale as f64) * anim_mult;
                    if !cr.modes.is_empty() {
                        let m_idx = self.active_mode_idx.min(cr.modes.len() - 1);
                        let ms = &cr.modes[m_idx].mode_shape;
                        let ui_v = ms.get(&frame.point_i).copied().unwrap_or([0.0; 6]);
                        let uj_v = ms.get(&frame.point_j).copied().unwrap_or([0.0; 6]);
                        let d1 = project([
                            p1[0] + scale * ui_v[0],
                            p1[1] + scale * ui_v[1],
                            p1[2] + scale * ui_v[2],
                        ]);
                        let d2 = project([
                            p2[0] + scale * uj_v[0],
                            p2[1] + scale * uj_v[1],
                            p2[2] + scale * uj_v[2],
                        ]);
                        painter.line_segment(
                            [d1, d2],
                            Stroke::new(2.6_f32, Color32::from_rgb(80, 255, 160)),
                        );
                    } else if let Some(stations) = cr.frame_forces.get(fid) {
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
                                base[0] + scale * dg[0],
                                base[1] + scale * dg[1],
                                base[2] + scale * dg[2],
                            ]));
                        }
                        for w in pts.windows(2) {
                            painter.line_segment(
                                [w[0], w[1]],
                                Stroke::new(2.6_f32, Color32::from_rgb(80, 255, 160)),
                            );
                        }
                    }
                }
            }

            // Internal Force / Moment Diagram along Frame
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
                            (span as f64) * 0.12 * (self.diagram_scale as f64) / max_diag_val;
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
                            .max_by(|a, b| a.1.abs().partial_cmp(&b.1.abs()).unwrap())
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

        // 4. Draw Joints, Restraints, Reactions, and Handle Interactive Click Selection / Drawing
        let click_pos = if response.clicked() {
            response.interact_pointer_pos()
        } else {
            None
        };
        let mut clicked_joint: Option<String> = None;

        for (pid, pt) in &mut self.model.points {
            let sp = project([pt.x, pt.y, pt.z]);
            if let Some(cp) = click_pos {
                if cp.distance(sp) < 10.0 {
                    clicked_joint = Some(pid.clone());
                }
            }

            let pt_col = if pt.selected {
                Color32::from_rgb(255, 230, 50)
            } else {
                Color32::from_rgb(230, 240, 255)
            };
            painter.circle_filled(sp, 3.8, pt_col);

            if self.show_joint_labels {
                painter.text(
                    sp + Vec2::new(6.0, -6.0),
                    egui::Align2::LEFT_BOTTOM,
                    pid,
                    FontId::proportional(11.0),
                    Color32::from_rgb(210, 225, 245),
                );
            }

            // Support restraint symbols
            if self.show_restraints && pt.restraint.iter().any(|&r| r) {
                let is_fixed = pt.restraint[0] && pt.restraint[2] && pt.restraint[4];
                let is_pinned = pt.restraint[0] && pt.restraint[2] && !pt.restraint[4];
                if is_fixed {
                    painter.rect_filled(
                        Rect::from_center_size(sp + Vec2::new(0.0, 5.0), Vec2::new(14.0, 7.0)),
                        0.0,
                        Color32::from_rgb(80, 210, 140),
                    );
                } else if is_pinned {
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

            // Support Reactions display
            if self.display_mode == DisplayMode::Reactions {
                if let Some(cr) = case_res {
                    if let Some(r) = cr.joint_reactions.get(pid) {
                        if r[0].abs() > 1e-4 || r[2].abs() > 1e-4 || r[4].abs() > 1e-4 {
                            let txt = format!("Fx={:.2}, Fz={:.2}, My={:.1}", r[0], r[2], r[4]);
                            painter.arrow(
                                sp + Vec2::new(0.0, 24.0),
                                Vec2::new(0.0, -22.0),
                                Stroke::new(2.0_f32, Color32::from_rgb(120, 255, 140)),
                            );
                            painter.text(
                                sp + Vec2::new(8.0, 18.0),
                                egui::Align2::LEFT_TOP,
                                txt,
                                FontId::proportional(10.5),
                                Color32::from_rgb(140, 255, 160),
                            );
                        }
                    }
                }
            }
        }

        if let Some(jid) = clicked_joint {
            match self.tool_mode {
                ToolMode::Select => {
                    if let Some(pt) = self.model.points.get_mut(&jid) {
                        pt.selected = !pt.selected;
                    }
                }
                ToolMode::DrawFrame => {
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
                ToolMode::DrawArea => {
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
                _ => {}
            }
        }

        // Viewport header overlay
        painter.text(
            rect.left_top() + Vec2::new(10.0, 8.0),
            egui::Align2::LEFT_TOP,
            format!("{title}  [Case: {}]", self.active_case),
            FontId::proportional(12.5),
            Color32::from_rgb(200, 220, 255),
        );
    }

    fn compute_bounds(&self) -> ([f64; 3], [f64; 3]) {
        if self.model.points.is_empty() {
            return ([-100.0, -100.0, 0.0], [100.0, 100.0, 100.0]);
        }
        let mut min_p = [f64::INFINITY; 3];
        let mut max_p = [f64::NEG_INFINITY; 3];
        for pt in self.model.points.values() {
            min_p[0] = min_p[0].min(pt.x);
            min_p[1] = min_p[1].min(pt.y);
            min_p[2] = min_p[2].min(pt.z);
            max_p[0] = max_p[0].max(pt.x);
            max_p[1] = max_p[1].max(pt.y);
            max_p[2] = max_p[2].max(pt.z);
        }
        (min_p, max_p)
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

        self.model = m;
        self.import_warnings.clear();
        self.export_warnings.clear();
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
        self.model = m;
        self.import_warnings.clear();
        self.export_warnings.clear();
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
        self.model = m;
        self.import_warnings.clear();
        self.export_warnings.clear();
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
                            ui.add(egui::DragValue::new(&mut self.tpl_span_or_bay_len).speed(6.0));
                            ui.end_row();

                            if self.tpl_choice != TemplateChoice::Beam {
                                ui.label("Number of Stories / Z-Divs:");
                                ui.add(
                                    egui::DragValue::new(&mut self.tpl_num_stories).range(1..=30),
                                );
                                ui.end_row();

                                ui.label("Story / Truss Height Z:");
                                ui.add(egui::DragValue::new(&mut self.tpl_story_height).speed(6.0));
                                ui.end_row();
                            }

                            if self.tpl_choice == TemplateChoice::Frame3D {
                                ui.label("Number of Y-Bays:");
                                ui.add(
                                    egui::DragValue::new(&mut self.tpl_num_bays_y).range(1..=20),
                                );
                                ui.end_row();

                                ui.label("Bay Width Y:");
                                ui.add(egui::DragValue::new(&mut self.tpl_bay_width_y).speed(6.0));
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
                        match self.tpl_choice {
                            TemplateChoice::Beam => self.model.generate_beam(
                                self.tpl_num_spans_or_bays,
                                self.tpl_span_or_bay_len,
                                self.tpl_restraint,
                                "W18X50",
                            ),
                            TemplateChoice::Frame2D => self.model.generate_2d_frame(
                                self.tpl_frame2d_type,
                                self.tpl_num_stories,
                                self.tpl_story_height,
                                self.tpl_num_spans_or_bays,
                                self.tpl_span_or_bay_len,
                                self.tpl_restraint,
                                "W18X50",
                                "W14X90",
                                "W14X90",
                                36.0,
                            ),
                            TemplateChoice::Truss2D => self.model.generate_2d_truss(
                                self.tpl_num_spans_or_bays,
                                self.tpl_span_or_bay_len,
                                self.tpl_story_height,
                                self.tpl_restraint,
                                "W14X90",
                                "W14X90",
                            ),
                            TemplateChoice::Frame3D => self.model.generate_3d_frame(
                                self.tpl_frame3d_type,
                                self.tpl_num_stories,
                                self.tpl_story_height,
                                self.tpl_num_spans_or_bays,
                                self.tpl_span_or_bay_len,
                                self.tpl_num_bays_y,
                                self.tpl_bay_width_y,
                                self.tpl_restraint,
                                "W18X50",
                                "W14X90",
                                "ASEC1",
                            ),
                            TemplateChoice::ShearWall => self.model.generate_wall(
                                self.tpl_num_spans_or_bays,
                                self.tpl_span_or_bay_len,
                                self.tpl_num_stories,
                                self.tpl_story_height,
                                self.tpl_restraint,
                                "ASEC1",
                            ),
                            TemplateChoice::SphericalDome => self.model.generate_dome_shell(
                                self.tpl_span_or_bay_len,
                                45.0,
                                self.tpl_num_stories.max(3),
                                self.tpl_num_spans_or_bays.max(8),
                                self.tpl_restraint,
                                "ASEC1",
                            ),
                        }
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
                    let percent = if total == 0 { 0.0 } else { 100.0 * passed as f64 / total as f64 };
                    ui.label(
                        RichText::new(format!(
                            "Verification Status: {passed} / {total} reference checks passed ({percent:.1}%)"
                        ))
                        .color(if all_passed { Color32::from_rgb(90, 235, 130) } else { Color32::from_rgb(255, 120, 100) })
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
            .default_width(540.0)
            .show(ctx, |ui| {
                if let Some(cr) = self.model.case_results.get(&self.active_case) {
                    if let Some(stations) = cr.frame_forces.get(&fid) {
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
                    } else {
                        ui.label("Run analysis (F5) to view station results.");
                    }
                } else {
                    ui.label("Run analysis (F5) to view station results.");
                }
            });
            if !open_insp {
                self.inspected_frame = None;
            }
        }

        // 4. Database Tables Modal (`Ctrl+T`)
        let mut open_tbl = self.show_tables_modal;
        if open_tbl {
            egui::Window::new(format!(
                "StructuraX Database Tables — Joint Displacements & Reactions [{}]",
                self.active_case
            ))
            .open(&mut open_tbl)
            .default_width(680.0)
            .show(ctx, |ui| {
                if let Some(cr) = self.model.case_results.get(&self.active_case) {
                    egui::ScrollArea::vertical()
                        .max_height(380.0)
                        .show(ui, |ui| {
                            ui.heading("TABLE: Joint Displacements");
                            egui::Grid::new("disp_tbl")
                                .striped(true)
                                .spacing([14.0, 4.0])
                                .show(ui, |ui| {
                                    ui.strong("Joint");
                                    ui.strong("U1");
                                    ui.strong("U2");
                                    ui.strong("U3");
                                    ui.strong("R1 (rad)");
                                    ui.strong("R2 (rad)");
                                    ui.strong("R3 (rad)");
                                    ui.end_row();
                                    for (jid, u) in &cr.joint_displacements {
                                        ui.label(jid);
                                        for v in u {
                                            ui.monospace(format!("{v:.5}"));
                                        }
                                        ui.end_row();
                                    }
                                });
                            ui.separator();
                            ui.heading("TABLE: Joint Support Reactions");
                            egui::Grid::new("react_tbl")
                                .striped(true)
                                .spacing([14.0, 4.0])
                                .show(ui, |ui| {
                                    ui.strong("Joint");
                                    ui.strong("F1");
                                    ui.strong("F2");
                                    ui.strong("F3");
                                    ui.strong("M1");
                                    ui.strong("M2");
                                    ui.strong("M3");
                                    ui.end_row();
                                    for (jid, r) in &cr.joint_reactions {
                                        ui.label(jid);
                                        for v in r {
                                            ui.monospace(format!("{v:.3}"));
                                        }
                                        ui.end_row();
                                    }
                                });
                        });
                } else {
                    ui.label("No analysis results available — press F5 to run analysis.");
                }
            });
        }
        self.show_tables_modal = open_tbl;

        // 5. Frame Sections & Binary `.pro` Importer Modal
        let mut open_sec = self.show_section_modal;
        if open_sec {
            egui::Window::new("Define Frame Sections & Import from .PRO Section Database")
                .open(&mut open_sec)
                .default_width(520.0)
                .show(ctx, |ui| {
                    ui.heading("Import from Standard .PRO Section Library");
                    ui.horizontal(|ui| {
                        ui.label("Database (.pro):");
                        ui.text_edit_singleline(&mut self.pro_file_name);
                        if ui.button("Load .PRO Index").clicked() {
                            if let Some(p) = resolve_pro_database_path(&self.pro_file_name) {
                                if let Ok(db) = ProDatabase::from_file(p) {
                                    self.pro_available_names =
                                        db.sections.iter().map(|s| s.name.clone()).collect();
                                }
                            }
                        }
                    });
                    ui.horizontal(|ui| {
                        ui.label("Search Prefix:");
                        ui.text_edit_singleline(&mut self.pro_filter);
                    });
                    egui::ScrollArea::vertical()
                        .max_height(150.0)
                        .show(ui, |ui| {
                            let filt = self.pro_filter.to_ascii_uppercase();
                            let matching_names: Vec<_> = self
                                .pro_available_names
                                .iter()
                                .filter(|n| n.to_ascii_uppercase().contains(&filt))
                                .take(40)
                                .cloned()
                                .collect();
                            for sname in &matching_names {
                                if ui
                                    .button(format!("+ Import {sname} & Assign to Selected"))
                                    .clicked()
                                {
                                    if let Some(p) = resolve_pro_database_path(&self.pro_file_name) {
                                        if let Ok(db) = ProDatabase::from_file(p) {
                                            if let Ok(sec) = db.get_section_scaled(
                                                sname,
                                                self.model.present_units,
                                                "A992Fy50",
                                            ) {
                                                self.invalidate_results();
                                                self.model.frame_sections.insert(sname.clone(), sec);
                                                for f in self
                                                    .model
                                                    .frames
                                                    .values_mut()
                                                    .filter(|f| f.selected)
                                                {
                                                    f.section = sname.clone();
                                                }
                                                self.status_msg = format!(
                                                    "Imported section {sname} from {}",
                                                    self.pro_file_name
                                                );
                                            }
                                        }
                                    }
                                }
                            }
                        });
                    ui.separator();
                    ui.heading("Create Parametric Wide-Flange (I) Section");
                    egui::Grid::new("cust_sec").num_columns(2).show(ui, |ui| {
                        ui.label("Name:");
                        ui.text_edit_singleline(&mut self.custom_sec_name);
                        ui.end_row();
                        ui.label("Depth t3:");
                        ui.add(egui::DragValue::new(&mut self.custom_sec_d));
                        ui.end_row();
                        ui.label("Flange Width t2:");
                        ui.add(egui::DragValue::new(&mut self.custom_sec_b));
                        ui.end_row();
                        ui.label("Flange Thick tf:");
                        ui.add(egui::DragValue::new(&mut self.custom_sec_tf));
                        ui.end_row();
                        ui.label("Web Thick tw:");
                        ui.add(egui::DragValue::new(&mut self.custom_sec_tw));
                        ui.end_row();
                    });
                    if ui.button("Add Custom I-Section").clicked() {
                        let sec = make_i_section(
                            &self.custom_sec_name,
                            "A992Fy50",
                            self.custom_sec_d,
                            self.custom_sec_b,
                            self.custom_sec_tf,
                            self.custom_sec_tw,
                            self.custom_sec_b,
                            self.custom_sec_tf,
                        );
                        self.invalidate_results();
                        self.model
                            .frame_sections
                            .insert(self.custom_sec_name.clone(), sec);
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
                                    self.import_warnings = m.warnings.clone();
                                    self.export_warnings.clear();
                                    self.model = m.model;
                                    self.active_case = self.model.load_cases.keys().next().cloned().unwrap_or_default();
                                    self.draw_first_point = None;
                                    self.draw_area_points.clear();
                                    self.inspected_frame = None;
                                    self.status_msg = format!("Opened {}", self.file_path_input);
                                    if !m.warnings.is_empty() {
                                        self.status_msg.push_str(&format!(" — Import warnings: {}", m.warnings.join("; ")));
                                    }
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
                                Err(error)=>self.status_msg=format!("Save error: {error}"),
                                Ok(warnings)=>{
                                    self.status_msg=format!("Saved S2K to {}",self.file_path_input);
                                    if !warnings.is_empty() {self.status_msg.push_str(&format!(" — Export warnings: {}",warnings.join("; ")));}
                                    self.export_warnings=warnings;
                                    close_file=true;
                                }
                            }
                        }
                        if ui.button("💾 Export Binary .SDB").clicked() {
                            self.export_warnings.clear();
                            match save_native_sdb_file(&self.model, &self.file_path_input) {
                                Err(error) => self.status_msg = format!("Export SDB error: {error}"),
                                Ok(()) => {
                                    self.status_msg = format!(
                                        "Exported binary stream .SDB to {}",
                                        self.file_path_input
                                    );
                                    close_file = true;
                                }
                            }
                        }
                        if ui.button("💾 Save Private .SDB").clicked() {
                            self.export_warnings.clear();
                            match save_sdb_file_with_report(&self.model, &self.file_path_input) {
                                Err(error)=>self.status_msg=format!("Save SDB error: {error}"),
                                Ok(warnings)=>{
                                    self.status_msg=format!("Saved private StructuraX container to {}.",self.file_path_input);
                                    if !warnings.is_empty() {self.status_msg.push_str(&format!(" — Export warnings: {}",warnings.join("; ")));}
                                    self.export_warnings=warnings;
                                    close_file=true;
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
}

impl eframe::App for SapGuiApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
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
                                    if ui.selectable_value(&mut cur_u, u, u.s2k_label()).clicked() {
                                        self.model.convert_units_in_place(u);
                                        self.status_msg =
                                            format!("Converted model units to {}; results cleared, run analysis again.", u.s2k_label());
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

fn contour_color(val: f64) -> Color32 {
    let t = (val.abs() * 10.0).tanh() as f32;
    Color32::from_rgba_unmultiplied(
        (60.0 + 190.0 * t) as u8,
        (140.0 * (1.0 - 0.5 * t)) as u8,
        (230.0 * (1.0 - t)) as u8,
        140,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

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
}
