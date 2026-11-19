//! Interactive StructuraX Desktop CAD GUI (`eframe`/`egui`), providing a 3D/2D structural
//! engineering workspace with ribbon/menus, Model Explorer, synchronized 3D + 2D viewports,
//! Parametric Structural Templates wizard, `.pro` section database importer, interactive
//! member diagram inspector, Database Tables viewer, and Verification Benchmark Suite runner.

mod analysis;
mod build;
mod contours;
mod display_units;
mod draw_tools;
mod file_browser;
mod hinges;
mod i_section;
mod materials;
mod messages;
mod modal_tables;
mod hinge_results;
mod pushover;
mod result_display;
mod result_tables;
mod section_import;
mod seismic;
mod selection;
mod shell_output;
mod steel_design;
mod time_history;
#[cfg(test)]
mod video_drawing_e2e;
mod video_export;
mod video_menu;
mod object_info;
#[cfg(test)]
mod video_results_e2e;
#[cfg(test)]
mod video_wave5_e2e;
#[cfg(test)]
mod p16_messages_tests;
#[cfg(test)]
mod p16_misc_tests;
#[cfg(test)]
mod p16_offset_tests;
#[cfg(test)]
mod p16_open_tests;
#[cfg(test)]
mod wave5_hand_check_tests;
#[cfg(test)]
mod wave5_windows_tests;
mod view;
mod workflows;

use eframe::egui::{self, Color32, FontId, Pos2, Rect, RichText, Sense, Shape, Stroke, Vec2};
use sap_design::{try_run_concrete_design, try_run_steel_design};
use sap_format::{
    load_model_with_report, save_native_sdb_file, save_s2k_file_with_report,
    save_sdb_file_with_report,
};
use sap_model::{
    ConcreteDesignResult, ConcreteRebarConfig, FrameLoadDistributed, FrameLoadPoint,
    FrameSection, FrameType2D, FrameType3D, LoadCase, LoadPattern, LoadPatternType, MatType,
    Material, PointLoadForce, SapModel, SteelDesignResult,
};
#[cfg(test)]
use sap_model::CaseResults;
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
use view::{PlaneView, ViewTransform};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ToolMode {
    Select,
    DrawPoint,
    DrawFrame,
    DrawArea,
    /// Click or window grid segments of a plane view.
    QuickFrame,
    /// Frames through picked points.
    PolyFrame,
    /// Click or window grid cells of a plane view.
    QuickArea,
    /// Two opposite corners in a plane view.
    RectArea,
    /// An area through picked points.
    PolyArea,
    /// Click or window grid cells of an elevation.
    QuickBrace,
    /// Click or window grid cells of a plan.
    QuickSecondary,
}

/// What a primary drag in a viewport pane does; decided when the drag starts
/// from the modifier keys, the projection and the tool.
#[derive(Debug, Clone, Copy, PartialEq)]
enum DragKind {
    /// Rubber band: a selection window, or a window for the active draw tool.
    Window,
    Pan,
    /// Rotate the 3D view by this many radians per pixel.
    Rotate(f32),
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct PaneDrag {
    pane: usize,
    kind: DragKind,
    from: Pos2,
    to: Pos2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DisplayMode {
    Undeformed,
    Deformed,
    Reactions,
    FrameDiagram(FrameForceComponent),
    ShellContour(shell_output::ShellQuantity),
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
    /// Scale factor of the deformed shape in use. With `deform_auto` it is set
    /// every frame from the model and the result (`result_display::scaling`).
    pub deform_scale: f32,
    /// "Scaling: Auto" of the Show Deformed Shape form. A change of
    /// `deform_scale` that the automatic scale did not make (the slider, a
    /// typed value) is a user-defined factor and turns this off.
    pub deform_auto: bool,
    /// Window of the right-click on an object of the undeformed display.
    object_info: object_info::State,
    /// Form of File > Create Video.
    video_form: video_export::VideoForm,
    /// Counter that every analysis, unlock and model replacement increments;
    /// cached traces of the plot window are valid for one value of it.
    pub results_revision: u64,
    /// The value the automatic scale last set (or found) in `deform_scale`.
    deform_auto_written: Option<f32>,
    pub animate_deformed: bool,
    pub anim_phase: f32,
    /// Result forms, contour and joint read-out (see `result_display`).
    results: result_display::State,
    pub diagram_scale: f32,

    // Viewports
    pub dual_pane: bool,
    pub left_proj: ViewProjection,
    pub right_proj: ViewProjection,
    pub yaw_rad: f32,
    pub pitch_rad: f32,
    pub zoom: f32,
    pub pan: Vec2,
    /// Coordinate of the plane a 2D pane shows (left pane, right pane);
    /// `None` shows the whole model in that projection.
    pub left_plane: Option<f64>,
    pub right_plane: Option<f64>,
    /// Projection and plane each pane used in the last frame.
    last_view: [Option<(ViewTransform, Option<PlaneView>)>; 2],
    /// The pane the pointer was last in; Plane Up/Down and the keyboard act
    /// on it.
    active_pane: usize,
    /// The primary drag in progress, decided when it started.
    pane_drag: Option<PaneDrag>,
    set_2d: view::Set2dForm,

    // Display options
    pub show_grids: bool,
    pub show_joint_labels: bool,
    pub show_frame_labels: bool,
    pub show_area_labels: bool,
    pub show_solid_labels: bool,
    pub show_link_labels: bool,
    pub show_hinge_labels: bool,
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
    /// 2D frame on typed bay widths and storey heights.
    pub tpl_custom_grid: bool,
    pub tpl_bays_text: String,
    pub tpl_storeys_text: String,

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
    /// Bottom flange of an I-section (width t2b and thickness tfb); equal to
    /// the top flange while `custom_sec_symmetric`.
    pub custom_sec_bb: f64,
    pub custom_sec_tfb: f64,
    pub custom_sec_symmetric: bool,
    pub custom_sec_rebar_enabled: bool,
    /// Rebar material of the longitudinal and the confinement bars of a custom
    /// section (a model material with a yield stress).
    pub custom_sec_rebar_material: String,
    pub custom_sec_is_column: bool,
    pub custom_sec_cover: f64,
    pub custom_sec_rebar_area: f64,

    workflows: workflows::Workflows,
    analysis_editor: analysis::AnalysisEditor,
    panel_build: build::State,
    draw_tools: draw_tools::State,
    selection: selection::State,
    file_browser: file_browser::State,
    panel_materials: materials::State,
    panel_section_import: section_import::State,
    panel_hinges: hinges::State,
    panel_modal_tables: modal_tables::State,
    panel_pushover: pushover::State,
    panel_hinge_results: hinge_results::State,
    panel_seismic: seismic::State,
    panel_steel_design: steel_design::State,
    panel_time_history: time_history::State,
    /// The status messages of the session (the Messages window).
    messages: messages::State,

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
            deform_auto: true,
            video_form: video_export::VideoForm::default(),
            object_info: object_info::State::default(),
            results_revision: 0,
            deform_auto_written: Some(25.0),
            animate_deformed: false,
            anim_phase: 0.0,
            results: result_display::State::default(),
            diagram_scale: 1.0,
            dual_pane: true,
            left_proj: ViewProjection::Perspective3D,
            right_proj: ViewProjection::ElevationXZ,
            yaw_rad: -0.65,
            pitch_rad: 0.42,
            zoom: 1.0,
            pan: Vec2::ZERO,
            left_plane: None,
            right_plane: None,
            last_view: [None; 2],
            active_pane: 1,
            pane_drag: None,
            set_2d: view::Set2dForm::default(),
            show_grids: true,
            show_joint_labels: true,
            show_frame_labels: false,
            show_area_labels: false,
            show_solid_labels: false,
            show_link_labels: true,
            show_hinge_labels: false,
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
            tpl_custom_grid: false,
            tpl_bays_text: "240, 240, 240".to_string(),
            tpl_storeys_text: "144, 144, 144".to_string(),
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
            custom_sec_bb: 9.0,
            custom_sec_tfb: 0.68,
            custom_sec_symmetric: true,
            custom_sec_rebar_enabled: false,
            custom_sec_rebar_material: String::new(),
            custom_sec_is_column: false,
            custom_sec_cover: 2.5,
            custom_sec_rebar_area: 4.0,
            workflows: workflows::Workflows::default(),
            analysis_editor: analysis::AnalysisEditor::default(),
            panel_build: build::State::default(),
            draw_tools: draw_tools::State::default(),
            selection: selection::State::default(),
            file_browser: file_browser::State::default(),
            panel_materials: materials::State::default(),
            panel_section_import: section_import::State::default(),
            panel_hinges: hinges::State::default(),
            panel_modal_tables: modal_tables::State::default(),
            panel_pushover: pushover::State::default(),
            panel_hinge_results: hinge_results::State::default(),
            panel_seismic: seismic::State::default(),
            panel_steel_design: steel_design::State::default(),
            panel_time_history: time_history::State::default(),
            messages: messages::State::default(),
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
            // Time-history steps and saved states of a nonlinear static case
            // share the step slider; a case has one kind or the other.
            let step_count = cr.time_steps.len().max(cr.static_steps.len());
            if step_count > 0 {
                self.active_step_idx = self.active_step_idx.min(step_count - 1);
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
        if self
            .results
            .readout_joint
            .as_ref()
            .is_some_and(|j| !self.model.points.contains_key(j))
        {
            self.results.readout_joint = None;
        }
        if !self.model.materials.contains_key(&self.custom_sec_material) {
            if let Some(first_mat) = self.model.materials.keys().next().cloned() {
                self.custom_sec_material = first_mat;
            }
        }
    }

    fn replace_model(&mut self, new_model: SapModel, status: String, import_warnings: Vec<String>) {
        let old_u = self.model.present_units;
        let new_u = new_model.present_units;
        self.results_revision += 1;
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
        self.panel_build = build::State::default();
        self.draw_tools = draw_tools::State::default();
        self.selection = selection::State::default();
        self.file_browser = file_browser::State::default();
        self.panel_materials = materials::State::default();
        self.panel_section_import = section_import::State::default();
        // A plane view stays where the new model has that plane; a model whose
        // joints all lie in one global plane opens its 2D panes on it.
        self.fit_planes_to_model();
        self.last_view = [None; 2];
        self.pane_drag = None;
        self.active_pane = 0;
        // The view fits the pane to the bounds of the model (`render_viewport`)
        // at zoom 1 and no pan: a new model starts there, not at the zoom and
        // pan the old one was looked at with ("Reset Camera / Fit View").
        self.zoom = 1.0;
        self.pan = Vec2::ZERO;
        self.set_2d.open = false;
        if draw_tools::is_draw_tool(self.tool_mode) {
            self.tool_mode = ToolMode::Select;
        }
        self.panel_hinges = hinges::State::default();
        self.panel_modal_tables = modal_tables::State::default();
        self.panel_pushover = pushover::State::default();
        self.panel_hinge_results = hinge_results::State::default();
        self.panel_seismic = seismic::State::default();
        self.panel_steel_design = steel_design::State::default();
        self.panel_time_history = time_history::State::default();
        self.results = result_display::State::default();
        self.object_info = object_info::State::default();
        // Nothing of the old model's results or of the way they were shown
        // outlives it: a file has no results (an S2K or a template is
        // unlocked in the undeformed display), and a result display of the old
        // model (a shell stress, a mode of the deformed shape) would show "no
        // results" for the new one.
        if self.model.case_results.is_empty() {
            self.model.is_locked = false;
        }
        self.display_mode = DisplayMode::Undeformed;
        self.deform_auto = true;
        self.deform_auto_written = None;
        self.animate_deformed = false;
        self.anim_phase = 0.0;
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

    /// File > Open: reads the model at `path` (an S2K or an SDB) and replaces
    /// the current one (`replace_model`). Returns whether the file was opened;
    /// a file that cannot be read leaves the model and the display unchanged.
    fn open_model_file(&mut self, path: &str) -> bool {
        match load_model_with_report(path) {
            Ok(m) => {
                let mut status = format!("Opened {path}");
                if !m.warnings.is_empty() {
                    status.push_str(&format!(" — Import warnings: {}", m.warnings.join("; ")));
                }
                self.replace_model(m.model, status, m.warnings);
                true
            }
            Err(e) => {
                self.status_msg = format!("Open error: {e}");
                false
            }
        }
    }

    /// Keeps the plane of each 2D pane whose coordinate is a plane of the
    /// current model (a grid line, or a joint coordinate where the axis has no
    /// grid) and shows the whole model in the others. A model whose joints all
    /// lie in one global plane (a 2D frame, a truss, a slab) puts every 2D pane
    /// on that plane.
    fn fit_planes_to_model(&mut self) {
        let tol = view::plane_tolerance(&self.model);
        let coplanar = view::joint_plane(&self.model);
        let panes = if self.dual_pane { 2 } else { 1 };
        for pane in 0..panes {
            let (proj, plane) = if pane == 0 {
                (&mut self.left_proj, &mut self.left_plane)
            } else {
                (&mut self.right_proj, &mut self.right_plane)
            };
            if *proj == ViewProjection::Perspective3D {
                *plane = None;
                continue;
            }
            if let Some(joint_plane) = coplanar {
                *proj = joint_plane.normal.projection();
                *plane = Some(joint_plane.coordinate);
                continue;
            }
            *plane = PlaneView::of(*proj, *plane).filter(|old| {
                view::plane_levels(&self.model, old.normal)
                    .iter()
                    .any(|level| (level - old.coordinate).abs() <= tol)
            }).map(|old| old.coordinate);
        }
        if !self.dual_pane {
            self.right_plane = None;
        }
    }

    /// The units selector: converts the model and every length the window
    /// state holds (see `scale_gui_cached_units`). Refused on a locked model.
    fn change_units(&mut self, u: Units) {
        if u == self.model.present_units || !self.ensure_unlocked() {
            return;
        }
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
        self.custom_sec_bb *= fl;
        self.custom_sec_tfb *= fl;
        self.custom_sec_cover *= fl;
        self.custom_sec_rebar_area *= fl * fl;
        // Plane views are at a coordinate in the old units; points picked by
        // the draw tools and the typed coordinate of Set 2D View too.
        self.left_plane = self.left_plane.map(|c| c * fl);
        self.right_plane = self.right_plane.map(|c| c * fl);
        self.draw_tools.clear_pending();
        self.set_2d.coordinate_text.clear();
        self.draw_first_point = None;
        self.draw_area_points.clear();
        self.tpl_bays_text = build::scale_lengths_text(&self.tpl_bays_text, fl);
        self.tpl_storeys_text = build::scale_lengths_text(&self.tpl_storeys_text, fl);
        // D125: the drafts of the seismic window and of the Build window.
        self.panel_seismic.scale_lengths(fl);
        self.panel_build.scale_lengths(fl);
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

    /// D124: the one way the application clears results: the model's own
    /// `invalidate_results` (results and the reasons a result is unavailable)
    /// and a new `results_revision`, which the plot caches key on.
    fn invalidate_results(&mut self) {
        self.results_revision += 1;
        self.model.is_locked = false;
        self.model.invalidate_results();
    }

    /// Sizes of the result stores of the model: case results, reasons a result
    /// is unavailable, steel and concrete design results.
    fn result_sizes(&self) -> [usize; 4] {
        [
            self.model.case_results.len(),
            self.model.unavailable_results.len(),
            self.model.steel_design_results.len(),
            self.model.concrete_design_results.len(),
        ]
    }

    /// D124: runs window code that edits the model through `&mut SapModel`
    /// functions (panels, workflows, the analysis editor). Those call
    /// `SapModel::invalidate_results` and cannot reach `results_revision`, so a
    /// store that held entries before and is empty after counts as a clear and
    /// bumps the revision.
    fn track_result_clears<R>(&mut self, run: impl FnOnce(&mut Self) -> R) -> R {
        let before = self.result_sizes();
        let out = run(self);
        let after = self.result_sizes();
        if (0..4).any(|i| before[i] > 0 && after[i] == 0) {
            self.results_revision += 1;
        }
        out
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
        self.results_revision += 1;
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
                // D78: results that could not be produced are named, not dropped silently.
                for (name, reason) in &self.model.unavailable_results {
                    self.status_msg
                        .push_str(&format!(" Unavailable: '{name}' ({reason})."));
                }
                // G16: a case that stopped before its target keeps the states it reached.
                for (name, status) in sap_model::results::unfinished_cases(&self.model) {
                    if let sap_model::CaseStatus::NotFinished { reason, steps } = status {
                        self.status_msg.push_str(&format!(
                            " Not finished: '{name}' ({reason}; {steps} saved steps kept)."
                        ));
                    }
                }
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

    fn select_all_objects(&mut self) {
        for p in self.model.points.values_mut() {
            p.selected = true;
        }
        for f in self.model.frames.values_mut() {
            f.selected = true;
        }
        for a in self.model.areas.values_mut() {
            a.selected = true;
        }
        for s in self.model.solids.values_mut() {
            s.selected = true;
        }
        for l in self.model.links.values_mut() {
            l.selected = true;
        }
    }

    fn clear_selection(&mut self) {
        for p in self.model.points.values_mut() {
            p.selected = false;
        }
        for f in self.model.frames.values_mut() {
            f.selected = false;
        }
        for a in self.model.areas.values_mut() {
            a.selected = false;
        }
        for s in self.model.solids.values_mut() {
            s.selected = false;
        }
        for l in self.model.links.values_mut() {
            l.selected = false;
        }
    }

    /// Send Enter or Escape to the active draw tool, in the view the pointer
    /// was last in. Without a drawn frame there is no view to give.
    fn route_draw_key(&mut self, event: draw_tools::PointerEvent) {
        // Points picked in one pane are discarded by an event of another
        // view, so the key goes to the pane they were picked in, when that
        // pane is drawn; otherwise to the pane the pointer was last in.
        let first = self.active_pane.min(1);
        let picked_in = self.draw_tools.picked_in();
        let pane = [first, 1 - first]
            .into_iter()
            .find(|p| {
                picked_in.is_some_and(|(tool, plane, proj)| {
                    tool == self.tool_mode
                        && self.last_view[*p].is_some_and(|(v, pl)| v.proj == proj && pl == plane)
                })
            })
            .unwrap_or(first);
        let Some((view, plane)) = self.last_view[pane] else {
            return;
        };
        let outcome = draw_tools::on_event(
            &mut self.draw_tools,
            self.tool_mode,
            &view,
            plane,
            event,
            &mut self.model,
        );
        self.apply_draw_outcome(outcome);
    }

    fn pane_projection(&self, pane: usize) -> ViewProjection {
        if pane == 0 {
            self.left_proj
        } else {
            self.right_proj
        }
    }

    fn pane_plane(&self, pane: usize) -> Option<f64> {
        if pane == 0 {
            self.left_plane
        } else {
            self.right_plane
        }
    }

    /// Show `proj` in a pane. The plane of the old projection does not carry
    /// over (its coordinate belongs to another axis), so the pane shows the
    /// whole model, unless the projection is unchanged.
    fn set_pane_projection(&mut self, pane: usize, proj: ViewProjection) {
        let pane = pane.min(1);
        if self.pane_projection(pane) == proj {
            return;
        }
        if pane == 0 {
            self.left_proj = proj;
            self.left_plane = None;
        } else {
            self.right_proj = proj;
            self.right_plane = None;
        }
    }

    /// Show the plane normal to `normal` at `coordinate` in a pane.
    fn set_plane_view(&mut self, pane: usize, normal: view::PlaneAxis, coordinate: f64) {
        if pane.min(1) == 0 {
            self.left_proj = normal.projection();
            self.left_plane = Some(coordinate);
        } else {
            self.right_proj = normal.projection();
            self.right_plane = Some(coordinate);
        }
    }

    /// The 2D pane Plane Up and Plane Down act on: the pane the pointer was
    /// last in, else the other one.
    fn plane_pane(&self) -> Option<usize> {
        let first = self.active_pane.min(1);
        [first, 1 - first]
            .into_iter()
            .filter(|p| *p == 0 || self.dual_pane)
            .find(|p| self.pane_projection(*p) != ViewProjection::Perspective3D)
    }

    /// Plane Up / Plane Down: the next plane of the grid (or of the joint
    /// coordinates) in the plane view. At the last plane the view stays.
    fn step_plane(&mut self, up: bool) {
        let Some(pane) = self.plane_pane() else {
            self.status_msg = "Plane Up / Down needs a plan or an elevation pane.".into();
            return;
        };
        let Some(normal) = PlaneView::of(self.pane_projection(pane), Some(0.0)).map(|p| p.normal)
        else {
            return;
        };
        let levels = view::plane_levels(&self.model, normal);
        let tol = view::plane_tolerance(&self.model);
        match view::step_level(&levels, self.pane_plane(pane), up, tol) {
            Some(c) => {
                self.set_plane_view(pane, normal, c);
                if let Some(plane) = PlaneView::of(self.pane_projection(pane), Some(c)) {
                    self.status_msg = plane.title();
                }
            }
            None => {
                self.status_msg = format!(
                    "No {} {} plane: the view stays.",
                    if up { "higher" } else { "lower" },
                    normal.label()
                );
            }
        }
    }

    /// Leave every plane view: both panes show the whole model again.
    fn whole_model_view(&mut self) {
        self.left_plane = None;
        self.right_plane = None;
        self.status_msg = "Showing the whole model.".into();
    }

    fn open_set_2d_view(&mut self) {
        let pane = if self.dual_pane { self.plane_pane().unwrap_or(1) } else { 0 };
        let proj = self.pane_projection(pane);
        if let Some(plane) = PlaneView::of(proj, Some(0.0)) {
            self.set_2d.normal = plane.normal;
        }
        self.set_2d.pane = pane;
        if let Some(c) = self.pane_plane(pane) {
            self.set_2d.coordinate_text = view::number_text(c);
        }
        self.set_2d.open = true;
    }

    /// The "Set 2D View" dialog: plane, coordinate and pane.
    fn render_set_2d_view(&mut self, ctx: &egui::Context) {
        if !self.set_2d.open {
            return;
        }
        let levels = view::plane_levels(&self.model, self.set_2d.normal);
        let mut open = true;
        let mut apply = false;
        let mut close = false;
        egui::Window::new("Set 2D View")
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .show(ctx, |ui| {
                if self.dual_pane {
                    ui.horizontal(|ui| {
                        ui.label("Pane:");
                        ui.radio_value(&mut self.set_2d.pane, 0, "Left");
                        ui.radio_value(&mut self.set_2d.pane, 1, "Right");
                    });
                }
                ui.horizontal(|ui| {
                    ui.label("Plane:");
                    for (axis, label) in [
                        (view::PlaneAxis::Z, "X-Y"),
                        (view::PlaneAxis::Y, "X-Z"),
                        (view::PlaneAxis::X, "Y-Z"),
                    ] {
                        ui.radio_value(&mut self.set_2d.normal, axis, label);
                    }
                });
                ui.horizontal(|ui| {
                    ui.label(format!("{} =", self.set_2d.normal.label()));
                    ui.text_edit_singleline(&mut self.set_2d.coordinate_text);
                    if !levels.is_empty() {
                        egui::ComboBox::from_id_salt("set_2d_levels")
                            .selected_text("grid")
                            .show_ui(ui, |ui| {
                                for l in &levels {
                                    if ui.selectable_label(false, view::number_text(*l)).clicked() {
                                        self.set_2d.coordinate_text = view::number_text(*l);
                                    }
                                }
                            });
                    }
                });
                ui.label("Front face of the plane only.");
                ui.horizontal(|ui| {
                    if ui.button("OK").clicked() {
                        apply = true;
                    }
                    if ui.button("Cancel").clicked() {
                        close = true;
                    }
                });
            });
        if apply {
            match self.set_2d.coordinate_text.trim().parse::<f64>() {
                Ok(c) if c.is_finite() => {
                    let pane = if self.dual_pane { self.set_2d.pane } else { 0 };
                    self.set_plane_view(pane, self.set_2d.normal, c);
                    if let Some(plane) = PlaneView::of(self.pane_projection(pane), Some(c)) {
                        self.status_msg = plane.title();
                    }
                    close = true;
                }
                _ => {
                    self.status_msg = format!(
                        "Not done: '{}' is not a coordinate.",
                        self.set_2d.coordinate_text
                    );
                }
            }
        }
        if close || !open {
            self.set_2d.open = false;
        }
    }

    fn render_top_menu_and_ribbon(&mut self, ctx: &egui::Context, ui: &mut egui::Ui) {
        // Keyboard shortcuts (ignored while typing in text fields)
        if !ctx.wants_keyboard_input() {
            let (f4, f5, f6, f7, del, esc, ctrl_n, ctrl_o, ctrl_t, enter) = ctx.input(|i| {
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
                    i.key_pressed(egui::Key::Enter),
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
                self.results.contour = result_display::Contour::Off;
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
            if enter && draw_tools::is_draw_tool(self.tool_mode) {
                self.route_draw_key(draw_tools::PointerEvent::Enter);
            }
            if esc {
                // The tool ends its sequence first; then the Select tool returns.
                if draw_tools::is_draw_tool(self.tool_mode) {
                    self.route_draw_key(draw_tools::PointerEvent::Escape);
                }
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
                ui.menu_button("Create Video", |ui| {
                    if ui.button("Multi-Step Animation Video...").clicked() {
                        self.video_form.open(&self.model, false);
                        ui.close_menu();
                    }
                    if ui.button("Cyclic Animation Video...").clicked() {
                        self.video_form.open(&self.model, true);
                        ui.close_menu();
                    }
                });
                ui.separator();
                if ui
                    .button("Open / Save Model (.s2k / .sdb)... (Ctrl+O)")
                    .clicked()
                {
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
                if ui
                    .button("Grid, Quick Draw, Radial, Extrude, Select by Property...")
                    .clicked()
                {
                    self.panel_build.open = true;
                    ui.close_menu();
                }
                if ui.button("Edit Grid Lines...").clicked() {
                    self.panel_build.show_tab(build::TAB_GRID);
                    ui.close_menu();
                }
                if ui.button("Divide / Break Frames...").clicked() {
                    self.open_workflow(0);
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

            ui.menu_button("Select", |ui| {
                if ui.button("Select All Objects").clicked() {
                    self.select_all_objects();
                    ui.close_menu();
                }
                if ui.button("Clear Selection").clicked() {
                    self.clear_selection();
                    ui.close_menu();
                }
                if ui.button("Get Previous Selection").clicked() {
                    self.status_msg = match selection::restore_previous(
                        &self.selection,
                        &mut self.model,
                    ) {
                        Ok(message) => message,
                        Err(error) => format!("Not done: {error}"),
                    };
                    ui.close_menu();
                }
            });

            ui.menu_button("View", |ui| {
                ui.checkbox(&mut self.dual_pane, "Dual-Pane Viewport (3D + 2D)");
                ui.separator();
                for (pane, name) in [(0usize, "Left"), (1, "Right")] {
                    ui.menu_button(format!("{name} Pane"), |ui| {
                        for (label, proj) in [
                            ("3D Perspective", ViewProjection::Perspective3D),
                            ("XY Plan", ViewProjection::PlanXY),
                            ("XZ Elevation", ViewProjection::ElevationXZ),
                            ("YZ Elevation", ViewProjection::ElevationYZ),
                        ] {
                            if ui.button(label).clicked() {
                                self.set_pane_projection(pane, proj);
                                ui.close_menu();
                            }
                        }
                    });
                }
                ui.separator();
                if ui.button("Set 2D View...").clicked() {
                    self.open_set_2d_view();
                    ui.close_menu();
                }
                if ui.button("Plane Up").clicked() {
                    self.step_plane(true);
                    ui.close_menu();
                }
                if ui.button("Plane Down").clicked() {
                    self.step_plane(false);
                    ui.close_menu();
                }
                if ui.button("Whole Model (leave plane view)").clicked() {
                    self.whole_model_view();
                    ui.close_menu();
                }
                ui.separator();
                ui.checkbox(&mut self.show_grids, "Show Grid Lines & Bubbles");
                ui.checkbox(&mut self.show_joint_labels, "Show Joint Labels");
                ui.checkbox(&mut self.show_frame_labels, "Show Frame Labels");
                ui.checkbox(&mut self.show_area_labels, "Show Area Labels");
                ui.checkbox(&mut self.show_solid_labels, "Show Solid Labels");
                ui.checkbox(&mut self.show_link_labels, "Show Link Labels");
                ui.checkbox(&mut self.show_hinge_labels, "Show Hinge Labels");
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
                    if ui.button(label).clicked() {
                        self.analysis_editor.open = true;
                        self.analysis_editor.tab = tab;
                        ui.close_menu();
                    }
                }
                for (label, tab) in [
                    ("Materials...", 5),
                    ("Load Patterns...", 1),
                    ("Area Sections and Modifiers...", 2),
                    ("Link Properties & Elements...", 6),
                    ("Load Combinations...", 4),
                ] {
                    if ui.button(label).clicked() {
                        self.open_workflow(tab);
                        ui.close_menu();
                    }
                }
                if ui.button("Materials from Library...").clicked() {
                    self.panel_materials.open = true;
                    ui.close_menu();
                }
                if ui.button("Frame Sections & .PRO Library...").clicked() {
                    self.show_section_modal = true;
                    ui.close_menu();
                }
                if ui
                    .button("Import Sections from .PRO (multiple)...")
                    .clicked()
                {
                    self.panel_section_import.open = true;
                    ui.close_menu();
                }
                if ui.button("Mass Source and Auto Seismic Loads...").clicked() {
                    self.panel_seismic.open = true;
                    ui.close_menu();
                }
                if ui.button("Hinge Properties...").clicked() {
                    self.panel_hinges.open = true;
                    self.panel_hinges.tab = hinges::TAB_PROPERTIES;
                    ui.close_menu();
                }
                if ui.button("Functions from File and Plot Functions...").clicked() {
                    self.panel_time_history.open = true;
                    ui.close_menu();
                }
                if ui.button("Database Tables Explorer... (Ctrl+T)").clicked() {
                    self.show_tables_modal = true;
                    ui.close_menu();
                }
            });

            ui.menu_button("Draw", |ui| {
                for (label, tool) in [
                    ("Select / Pointer", ToolMode::Select),
                    ("Draw Special Joint", ToolMode::DrawPoint),
                    ("Draw Frame between Two Joints", ToolMode::DrawFrame),
                    ("Draw Frame through Points", ToolMode::PolyFrame),
                    ("Quick Draw Frame (grid segment)", ToolMode::QuickFrame),
                    ("Quick Draw Braces (grid cell)", ToolMode::QuickBrace),
                    (
                        "Quick Draw Secondary Beams (grid cell)",
                        ToolMode::QuickSecondary,
                    ),
                    ("Draw Area through Four Joints", ToolMode::DrawArea),
                    ("Draw Poly Area", ToolMode::PolyArea),
                    ("Draw Rectangular Area", ToolMode::RectArea),
                    ("Quick Draw Area (grid cell)", ToolMode::QuickArea),
                ] {
                    if ui.button(label).clicked() {
                        self.tool_mode = tool;
                        ui.close_menu();
                    }
                }
            });

            ui.menu_button("Assign", |ui| {
                if ui.button("Frame Hinges...").clicked() {
                    self.panel_hinges.open = true;
                    self.panel_hinges.tab = hinges::TAB_ASSIGN;
                    ui.close_menu();
                }
                if ui.button("Frame Hinge Overwrites...").clicked() {
                    self.panel_hinges.open = true;
                    self.panel_hinges.tab = hinges::TAB_OVERWRITES;
                    ui.close_menu();
                }
                for (label, tab) in [
                    ("Frame Section...", 0),
                    ("Area Section and Mesh...", 2),
                    ("Frame Releases / T-C Limits / Cable...", 3),
                    ("Link Elements...", 6),
                ] {
                    if ui.button(label).clicked() {
                        self.open_workflow(tab);
                        ui.close_menu();
                    }
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
                if ui.button("Modal Periods and Participation Tables...").clicked() {
                    self.panel_modal_tables.open = true;
                    ui.close_menu();
                }
                if ui.button("Show Plot Functions...").clicked() {
                    self.panel_time_history.open = true;
                    ui.close_menu();
                }
                if ui.button("Show Static Pushover Curve...").clicked() {
                    self.panel_pushover.open = true;
                    ui.close_menu();
                }
                if ui.button("Show Hinge Results...").clicked() {
                    self.panel_hinge_results.show(&self.model);
                    ui.close_menu();
                }
                if ui.button("Show Undeformed Shape (F4)").clicked() {
                    self.display_mode = DisplayMode::Undeformed;
                    self.animate_deformed = false;
                    ui.close_menu();
                }
                if ui.button("Show Deformed Shape (F6)").clicked() {
                    self.display_mode = DisplayMode::Deformed;
                    self.results.contour = result_display::Contour::Off;
                    ui.close_menu();
                }
                if ui.button("Show Deformed Shape...").clicked() {
                    self.open_deformed_form();
                    ui.close_menu();
                }
                ui.menu_button("Show Forces/Stresses", |ui| {
                    if ui.button("Shells...").clicked() {
                        self.open_shell_form();
                        ui.close_menu();
                    }
                });
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
                for scomp in shell_output::ShellQuantity::LEGACY {
                    if ui
                        .button(format!("Shell Contour: {}", scomp.label()))
                        .clicked()
                    {
                        self.display_mode = DisplayMode::ShellContour(scomp);
                        self.results.contour = result_display::Contour::Off;
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
                if ui
                    .button("Steel Auto Select, Preferences, Design Combos, Verify...")
                    .clicked()
                {
                    self.panel_steel_design.open = true;
                    ui.close_menu();
                }
                ui.separator();
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
                if ui.button("Messages...").clicked() {
                    self.messages.open = true;
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
            ui.collapsing(
                format!("Export warnings ({})", self.export_warnings.len()),
                |ui| {
                    for warning in &self.export_warnings {
                        ui.label(warning);
                    }
                },
            );
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
                        let lbl = format!(
                            "Jt {id}: ({}, {}, {})",
                            view::quantity_text(pt.x),
                            view::quantity_text(pt.y),
                            view::quantity_text(pt.z)
                        );
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
                                format!(
                                    "{tag} {id}: {}->{} [{}]",
                                    fr.point_i, fr.point_j, fr.section
                                ),
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
                            ui.checkbox(&mut sol.selected, format!("Solid {id} [{}]", sol.section));
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
                        "• {name} ({:?}): A={}, I33={}",
                        sec.prop_type,
                        view::quantity_text(sec.area),
                        view::quantity_text(sec.i33)
                    ));
                }
            });

            egui::CollapsingHeader::new(format!("Load Cases ({})", self.model.load_cases.len()))
                .default_open(true)
                .show(ui, |ui| {
                    for (name, lc) in &self.model.load_cases {
                        let is_act = &self.active_case == name;
                        let run_tag = if !lc.run_flag {
                            " [Off]"
                        } else if sap_model::results::unavailable_reason(&self.model, name).is_some() {
                            " [could not start]" // G16: no results, the reason is the tooltip
                        } else if sap_model::results::unfinished_message(&self.model, name).is_some() {
                            " [not finished]"
                        } else {
                            ""
                        };
                        let mut resp = ui.selectable_label(
                            is_act,
                            format!("{name} ({:?}){run_tag}", lc.case_type),
                        );
                        if let Some(note) = sap_model::results::unavailable_message(&self.model, name)
                            .or_else(|| sap_model::results::unfinished_message(&self.model, name))
                        {
                            resp = resp.on_hover_text(note);
                        }
                        if resp.clicked() {
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
                        // D78: a combination without complete results is not offered
                        // as if it had results; choosing it shows the reason.
                        let gap = sap_model::results::unavailable_reason(&self.model, name);
                        let label = if gap.is_some() {
                            format!("{name} ({:?}) - unavailable", combo.combo_type)
                        } else {
                            format!("{name} ({:?})", combo.combo_type)
                        };
                        let mut resp = ui.selectable_label(is_act, label);
                        if let Some(reason) = gap {
                            resp = resp.on_hover_text(format!("No results: {reason}"));
                        }
                        if resp.clicked() {
                            self.active_case = name.clone();
                            self.active_mode_idx = 0;
                            self.active_step_idx = 0;
                        }
                    }
                    if let Some(msg) =
                        sap_model::results::unavailable_message(&self.model, &self.active_case)
                    {
                        ui.colored_label(egui::Color32::from_rgb(200, 60, 40), msg);
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

                if !cr.static_steps.is_empty() {
                    egui::CollapsingHeader::new(format!(
                        "Nonlinear Static Steps ({})",
                        cr.static_steps.len()
                    ))
                    .default_open(true)
                    .show(ui, |ui| {
                        if ui
                            .checkbox(&mut self.use_time_step, "Scrub Deformed Saved State")
                            .changed()
                            && self.use_time_step
                        {
                            self.display_mode = DisplayMode::Deformed;
                        }
                        let max_idx = cr.static_steps.len().saturating_sub(1);
                        let cur_idx = self.active_step_idx.min(max_idx);
                        let step = &cr.static_steps[cur_idx];
                        let monitored = step
                            .monitored_displacement
                            .map(|d| format!(", monitored {d:.5}"))
                            .unwrap_or_default();
                        ui.label(format!(
                            "State {cur_idx}/{max_idx}: load factor {:.5}{monitored}",
                            step.load_factor
                        ));
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
                                "Beam As = {}/{} [{}]",
                                view::quantity_text(res.as_top),
                                view::quantity_text(res.as_bot),
                                res.status
                            )
                        };
                        ui.colored_label(col, format!("Fr {fid} ({}): {detail}", res.section_name));
                    }
                });
            }
        });
    }

    /// Draws pane `pane` (0: left or only pane, 1: right pane).
    fn render_viewport(&mut self, ui: &mut egui::Ui, pane: usize, title: &str) {
        let pane = pane.min(1);
        let (proj, plane_coordinate) = if pane == 0 {
            (self.left_proj, self.left_plane)
        } else {
            (self.right_proj, self.right_plane)
        };
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

        if response.hovered() || response.dragged() {
            self.active_pane = pane;
        }

        // Camera manipulation. A middle or right drag pans a 2D pane and
        // rotates the 3D one. A primary drag is decided once, when it starts:
        // in 2D it draws a window (Select tool or a draw tool) and Shift or
        // Alt pan; in 3D it rotates (Shift pans, Alt rotates faster) and Ctrl
        // draws a window.
        if response.dragged_by(egui::PointerButton::Secondary)
            || response.dragged_by(egui::PointerButton::Middle)
        {
            let delta = response.drag_delta();
            if proj == ViewProjection::Perspective3D {
                self.yaw_rad += delta.x * 0.01;
                self.pitch_rad = (self.pitch_rad + delta.y * 0.01).clamp(-1.5, 1.5);
            } else {
                self.pan += delta;
            }
        }
        if response.drag_started_by(egui::PointerButton::Primary) {
            let (alt, shift, ctrl, press) = ui.input(|i| {
                (
                    i.modifiers.alt,
                    i.modifiers.shift,
                    i.modifiers.ctrl || i.modifiers.command,
                    i.pointer.press_origin(),
                )
            });
            let window_tool =
                self.tool_mode == ToolMode::Select || draw_tools::is_draw_tool(self.tool_mode);
            let kind = if proj == ViewProjection::Perspective3D {
                if alt {
                    Some(DragKind::Rotate(0.01))
                } else if ctrl && window_tool {
                    Some(DragKind::Window)
                } else if self.tool_mode == ToolMode::Select {
                    Some(if shift {
                        DragKind::Pan
                    } else {
                        DragKind::Rotate(0.008)
                    })
                } else {
                    None
                }
            } else if alt || shift {
                Some(DragKind::Pan)
            } else if window_tool {
                Some(DragKind::Window)
            } else {
                None
            };
            let from = press.or(response.interact_pointer_pos());
            self.pane_drag = kind.zip(from).map(|(kind, from)| PaneDrag {
                pane,
                kind,
                from,
                to: from,
            });
        }
        if response.dragged_by(egui::PointerButton::Primary) {
            let delta = response.drag_delta();
            let pos = response.interact_pointer_pos();
            if let Some(d) = self.pane_drag.as_mut().filter(|d| d.pane == pane) {
                match d.kind {
                    DragKind::Pan => self.pan += delta,
                    DragKind::Rotate(rate) => {
                        self.yaw_rad += delta.x * rate;
                        self.pitch_rad = (self.pitch_rad + delta.y * rate).clamp(-1.5, 1.5);
                    }
                    DragKind::Window => {
                        if let Some(p) = pos {
                            d.to = p;
                        }
                    }
                }
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

        let view = ViewTransform {
            proj,
            center: center_3d,
            scale: f64::from(base_scale),
            screen_center,
            yaw: self.yaw_rad,
            pitch: self.pitch_rad,
        };
        let plane = PlaneView::of(proj, plane_coordinate);
        self.last_view[pane] = Some((view, plane));
        let project = |p: [f64; 3]| -> Pos2 { view.project(p) };
        // In a plane view only the objects in the plane are drawn and picked.
        let members = plane.map(|pl| {
            view::plane_members(&self.model, pl, view::plane_tolerance(&self.model))
        });
        let in_view = |pick: fn(&view::Members) -> &std::collections::BTreeSet<String>,
                       name: &String|
         -> bool { members.as_ref().is_none_or(|m| pick(m).contains(name)) };

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

        self.update_deform_scale();
        // Animation phase & deformed displacement scale
        let anim_mult = if self.animate_deformed && !self.results.stepping {
            self.anim_phase.sin() as f64
        } else {
            1.0
        };
        let def_scale = (self.deform_scale as f64) * anim_mult;
        let case_res = self.model.case_results.get(&self.active_case);
        let active_mode_idx = self.active_mode_idx;
        let use_time_step = self.use_time_step;
        let active_step_idx = self.active_step_idx;

        // Joint results are stored in joint-local axes: the shape is drawn from
        // their global components.
        let step_sel = contours::StepSelection {
            mode_idx: active_mode_idx,
            use_step: use_time_step,
            step_idx: active_step_idx,
        };
        // What of the result the view shows, for its title (D33).
        let subtitle = case_res
            .filter(|_| self.display_mode == DisplayMode::Deformed)
            .and_then(|cr| result_display::scaling::view_subtitle(cr, step_sel));
        let model_ref = &self.model;
        let displaced_point = |pid: &str, base: [f64; 3]| -> [f64; 3] {
            result_display::displaced_point(model_ref, case_res, pid, base, def_scale, step_sel)
        };

        // Contour of the shell forms and the displacement contour: one value
        // table and legend per pane and frame.
        let frame_contour = result_display::contour_spec(
            self.display_mode,
            self.results.contour,
            self.model.present_units,
        )
        .map(|spec| {
            result_display::frame_contour(
                &self.model,
                &self.active_case,
                spec,
                step_sel,
                result_display::toward_viewer(&view),
            )
        });

        // Precompute normalized contour range for SolidContour
        let contour_range: Option<(&str, f64, f64)> = match (self.display_mode, case_res) {
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
            if let Some(pl) = plane {
                let grid_col = Color32::from_rgba_unmultiplied(70, 85, 110, 120);
                for g in view::plane_grid_segments(&self.model, pl, (min_pt, max_pt)) {
                    let sa = project(g.a);
                    let sb = project(g.b);
                    if sa.distance(sb) > 1.0 {
                        painter.line_segment([sa, sb], Stroke::new(1.0_f32, grid_col));
                        let bubble = if g.bubble_at_a { sa } else { sb };
                        painter.circle_stroke(bubble, 8.0, Stroke::new(1.0_f32, grid_col));
                        painter.text(
                            bubble,
                            egui::Align2::CENTER_CENTER,
                            &g.id,
                            FontId::proportional(10.0),
                            Color32::from_rgb(150, 170, 200),
                        );
                    }
                }
            } else if let Some(cs) = self.model.coord_systems.get("GLOBAL") {
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
        if let Some(m) = &members {
            // Solid faces in the plane are outlines; solids are not selectable.
            for (sid, k) in &m.solid_faces {
                let Some(solid) = self.model.solids.get(sid) else {
                    continue;
                };
                let face: Vec<Pos2> = view::SOLID_FACES[*k]
                    .iter()
                    .filter_map(|c| self.model.points.get(&solid.points[*c]))
                    .map(|p| project([p.x, p.y, p.z]))
                    .collect();
                if face.len() == 4 {
                    painter.extend(view::outlined_polygon(
                        &face,
                        Color32::TRANSPARENT,
                        Stroke::new(1.0_f32, Color32::from_rgb(130, 175, 225)),
                    ));
                }
            }
        }
        for (sid, solid) in &self.model.solids {
            if members.is_some() {
                continue;
            }
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
            let fill_col =
                if let (DisplayMode::SolidContour(sol_comp), Some(cr), Some((_, lo, hi))) =
                    (self.display_mode, case_res, contour_range)
                {
                    if let Some(sr) = cr.solid_results.get(sid) {
                        let s = &sr.center_stress;
                        let v = match sol_comp {
                            SolidComponent::S11 => s.s11,
                            SolidComponent::S22 => s.s22,
                            SolidComponent::S33 => s.s33,
                            SolidComponent::VonMises => s.von_mises,
                        };
                        contours::value_color(v, lo, hi, 115)
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
            for face in &view::SOLID_FACES {
                let fpoly = vec![
                    project(corners[face[0]]),
                    project(corners[face[1]]),
                    project(corners[face[2]]),
                    project(corners[face[3]]),
                ];
                painter.extend(view::outlined_polygon(
                    &fpoly,
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
            if !in_view(|m| &m.areas, aid) {
                continue;
            }
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
                    painter.extend(view::outlined_polygon(
                        &poly_undef,
                        Color32::TRANSPARENT,
                        Stroke::new(1.0_f32, Color32::from_rgba_unmultiplied(95, 115, 140, 90)),
                    ));
                }
                let active_poly =
                    if self.display_mode == DisplayMode::Deformed && poly_def.len() >= 3 {
                        &poly_def
                    } else {
                        &poly_undef
                    };
                let contour_fill = frame_contour
                    .as_ref()
                    .map(|fc| fc.fill(aid, active_poly))
                    .unwrap_or(result_display::AreaFill::Plain);
                let fill_col =
                    if let result_display::AreaFill::Flat(colour) = &contour_fill {
                        *colour
                    } else if matches!(contour_fill, result_display::AreaFill::Mesh(_)) {
                        Color32::TRANSPARENT
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
                if let result_display::AreaFill::Mesh(mesh) = contour_fill {
                    painter.add(Shape::mesh(mesh));
                }
                painter.extend(view::outlined_polygon(
                    active_poly,
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
                                let sign = if pl.face == -2 {
                                    -v.signum()
                                } else {
                                    v.signum()
                                };
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
            if !in_view(|m| &m.links, lid) {
                continue;
            }
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
                // A multi-valued combination draws its maximum and its minimum
                // on one scale.
                for stations in result_tables::max_min_results(cr)
                    .into_iter()
                    .flat_map(|(_, r)| r.frame_forces.values())
                {
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
        // Yielded hinges of the active case at the shown saved state (the
        // final state unless a saved state is scrubbed).
        let show_hinge_legend =
            case_res.is_some_and(|cr| !cr.static_steps.is_empty() || !cr.hinge_results.is_empty());
        let hinge_marks = if self.display_mode == DisplayMode::Deformed {
            let scrubbed = case_res
                .is_some_and(|cr| use_time_step && !cr.static_steps.is_empty())
                .then_some(active_step_idx);
            pushover::hinge_markers(&self.model, &self.active_case, scrubbed)
        } else {
            Vec::new()
        };
        for (fid, frame) in &self.model.frames {
            if !in_view(|m| &m.frames, fid) {
                continue;
            }
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
                painter.extend(view::outlined_polygon(
                    &[top_i, top_j, bot_j, bot_i],
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
            if self.show_hinge_labels {
                hinges::draw_view_labels(&painter, &self.model, fid, s1, s2);
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
                        format!(
                            "As {}/{}",
                            view::quantity_text(res.as_top),
                            view::quantity_text(res.as_bot)
                        )
                    } else {
                        format!(
                            "As {}/{} ({})",
                            view::quantity_text(res.as_top),
                            view::quantity_text(res.as_bot),
                            res.status
                        )
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
                    // A saved state of a nonlinear static case has its own stations.
                    let station_source = if use_time_step && !cr.static_steps.is_empty() {
                        &cr.static_steps[active_step_idx.min(cr.static_steps.len() - 1)]
                            .frame_forces
                    } else {
                        &cr.frame_forces
                    };
                    let use_stations = cr.modes.is_empty()
                        && !(use_time_step && !cr.time_steps.is_empty())
                        && station_source.contains_key(fid);
                    if use_stations {
                        if let Some(stations) = station_source.get(fid) {
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
                            // With end offsets the stations start at the support
                            // faces: the rigid arms are straight segments from
                            // the displaced joints to the first and the last
                            // station (D56).
                            if stations.first().is_some_and(|st| st.rel_dist > 1e-9) {
                                pts.insert(0, project(displaced_point(&frame.point_i, p1)));
                            }
                            if stations.last().is_some_and(|st| st.rel_dist < 1.0 - 1e-9) {
                                pts.push(project(displaced_point(&frame.point_j, p2)));
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

            // Hinge markers at their relative position on the deformed frame.
            for (_, rel, class) in hinge_marks.iter().filter(|(f, _, _)| f == fid) {
                let a = displaced_point(&frame.point_i, p1);
                let b = displaced_point(&frame.point_j, p2);
                let at = project([
                    a[0] + rel * (b[0] - a[0]),
                    a[1] + rel * (b[1] - a[1]),
                    a[2] + rel * (b[2] - a[2]),
                ]);
                painter.circle_filled(at, 5.5, pushover::class_color(*class));
                painter.circle_stroke(at, 5.5, Stroke::new(1.0_f32, Color32::from_rgb(10, 12, 18)));
            }

            // Internal Force / Moment Diagram along Frame (NaN-safe)
            if let DisplayMode::FrameDiagram(comp) = self.display_mode {
                if let Some(cr) = case_res {
                    for (tag, res) in result_tables::max_min_results(cr) {
                        let Some(stations) = res.frame_forces.get(fid) else {
                            continue;
                        };
                        // The minimum of a multi-valued combination has its own colours.
                        let is_min = tag == "Min";
                        let (pos_fill, neg_fill, pos_line, neg_line) = if is_min {
                            (
                                Color32::from_rgba_unmultiplied(90, 190, 255, 160),
                                Color32::from_rgba_unmultiplied(190, 130, 255, 160),
                                Color32::from_rgb(110, 200, 255),
                                Color32::from_rgb(200, 150, 255),
                            )
                        } else {
                            (
                                Color32::from_rgba_unmultiplied(255, 190, 60, 160),
                                Color32::from_rgba_unmultiplied(255, 95, 95, 160),
                                Color32::from_rgb(255, 200, 70),
                                Color32::from_rgb(255, 100, 100),
                            )
                        };
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
                            let col = if val >= 0.0 { pos_fill } else { neg_fill };
                            painter.line_segment([b_scr, o_scr], Stroke::new(1.2_f32, col));
                            outline.push((o_scr, val));
                        }
                        for w in outline.windows(2) {
                            let col = if w[0].1 >= 0.0 { pos_line } else { neg_line };
                            painter.line_segment([w[0].0, w[1].0], Stroke::new(2.0_f32, col));
                        }
                        if let Some((peak_pos, peak_val)) = outline
                            .iter()
                            .filter(|(pos, val)| {
                                pos.x.is_finite() && pos.y.is_finite() && val.is_finite()
                            })
                            .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
                        {
                            if peak_val.abs() > 1e-4 {
                                painter.text(
                                    *peak_pos,
                                    egui::Align2::CENTER_BOTTOM,
                                    if tag.is_empty() {
                                        view::quantity_text(peak_val.abs())
                                    } else {
                                        format!("{tag} {}", view::quantity_text(*peak_val))
                                    },
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
        // Joints as drawn on the deformed shape, for the right-click read-out.
        let mut joint_screen: Vec<(String, Pos2)> = Vec::new();

        for (pid, pt) in &self.model.points {
            if !in_view(|m| &m.points, pid) {
                continue;
            }
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
                if right_click_pos.is_some() {
                    joint_screen.push((pid.clone(), sp_def));
                }
                let pt_col = if pt.selected {
                    Color32::from_rgb(255, 230, 50)
                } else {
                    Color32::from_rgb(100, 255, 175)
                };
                painter.circle_filled(sp_def, 3.8, pt_col);
            } else {
                if right_click_pos.is_some() {
                    joint_screen.push((pid.clone(), sp));
                }
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
                        // Components below a millionth of the largest are
                        // roundoff, whatever the units.
                        let rmax = r.iter().filter(|v| v.is_finite()).fold(0.0_f64, |a, v| a.max(v.abs()));
                        let parts: Vec<String> = names
                            .iter()
                            .zip(r.iter())
                            .filter(|(_, v)| v.is_finite() && v.abs() > 1e-6 * rmax)
                            .map(|(n, v)| format!("{n}={}", view::quantity_text(*v)))
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

        // A click elsewhere closes the joint read-out.
        if click_pos.is_some() {
            self.results.readout_joint = None;
        }
        // Right-click: on the deformed shape a joint within
        // `JOINT_PICK_RADIUS` of the pointer wins and opens the Joint
        // Displacements read-out; otherwise (and in every other display) a
        // frame within 8 px opens the Member Station Diagram Inspector.
        let joint_pick = right_click_pos
            .filter(|_| self.display_mode == DisplayMode::Deformed && case_res.is_some())
            .and_then(|rcp| {
                result_display::closest_within(
                    &joint_screen,
                    rcp,
                    result_display::JOINT_PICK_RADIUS,
                )
            });
        if let Some(jid) = joint_pick {
            self.status_msg = match self.results.open_readout(&jid, self.animate_deformed) {
                Ok(()) => format!("Joint {jid}: displacements shown."),
                Err(refused) => refused,
            };
        } else if let Some(rcp) = right_click_pos.filter(|_| self.display_mode == DisplayMode::Undeformed) {
            // The undeformed display: the Object Information window of the
            // joint, frame or area under the pointer (G07).
            self.results.readout_joint = None;
            if let Some(obj) =
                object_info::pick(&joint_screen, &frame_screen_segs, &area_screen_polys, rcp)
            {
                self.status_msg = format!("{} {}: loads shown.", obj.kind(), obj.name());
                self.object_info.target = Some(obj);
            }
        } else if let Some(rcp) = right_click_pos {
            self.results.readout_joint = None;
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
                                    let fid =
                                        sap_model::next_numeric_name(self.model.frames.keys());
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
                tool => {
                    let outcome = draw_tools::on_event(
                        &mut self.draw_tools,
                        tool,
                        &view,
                        plane,
                        draw_tools::PointerEvent::Click {
                            pos: cp,
                            double: response.double_clicked(),
                        },
                        &mut self.model,
                    );
                    self.apply_draw_outcome(outcome);
                }
            }
        }

        // A finished window: select, or hand it to the active draw tool. The
        // old DrawPoint / DrawFrame / DrawArea tools ignore windows.
        if response.drag_stopped_by(egui::PointerButton::Primary)
            && self.pane_drag.is_some_and(|d| d.pane == pane)
        {
            if let Some(mut d) = self.pane_drag.take() {
                if let Some(p) = response.interact_pointer_pos() {
                    d.to = p;
                }
                if d.kind == DragKind::Window && d.from.distance(d.to) >= 2.0 {
                    if self.tool_mode == ToolMode::Select {
                        let n = selection::select_window(
                            &mut self.model,
                            &view,
                            plane,
                            d.from,
                            d.to,
                        );
                        self.status_msg = format!("Window selected {n} more object(s).");
                    } else if draw_tools::is_draw_tool(self.tool_mode) {
                        let outcome = draw_tools::on_event(
                            &mut self.draw_tools,
                            self.tool_mode,
                            &view,
                            plane,
                            draw_tools::PointerEvent::Window {
                                from: d.from,
                                to: d.to,
                            },
                            &mut self.model,
                        );
                        self.apply_draw_outcome(outcome);
                    }
                }
            }
        }

        // Preview of what the next click of a draw tool would do.
        if draw_tools::is_draw_tool(self.tool_mode) {
            let shapes = draw_tools::preview(
                &self.draw_tools,
                self.tool_mode,
                &view,
                plane,
                response.hover_pos(),
                &self.model,
            );
            painter.extend(shapes);
        }

        // Rubber band of a window in progress: solid when dragged left to
        // right (takes what is inside), dashed right to left (also takes what
        // it crosses).
        if let Some(d) = self
            .pane_drag
            .filter(|d| d.pane == pane && d.kind == DragKind::Window)
        {
            let band = Rect::from_two_pos(d.from, d.to);
            let col = Color32::from_rgb(255, 235, 150);
            if d.to.x < d.from.x {
                let path = [
                    band.left_top(),
                    band.right_top(),
                    band.right_bottom(),
                    band.left_bottom(),
                    band.left_top(),
                ];
                painter.extend(Shape::dashed_line(&path, Stroke::new(1.2_f32, col), 6.0, 4.0));
            } else {
                painter.rect_filled(band, 0.0, Color32::from_rgba_unmultiplied(255, 235, 150, 25));
                painter.rect_stroke(band, 0.0, Stroke::new(1.2_f32, col));
            }
        }

        // Legend of the shell forms and of the displacement contour.
        if let Some(fc) = &frame_contour {
            if rect.width() > 240.0 {
                result_display::draw_legend(&painter, rect, &fc.legend);
            }
        }

        // Contour Legend Overlay (when viewing SolidContour)
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
                        Pos2::new(
                            leg_rect.left() + 6.0 + i as f32 * bar_w,
                            leg_rect.top() + 18.0,
                        ),
                        Vec2::new(bar_w + 0.5, 8.0),
                    );
                    painter.rect_filled(sub, 0.0, contours::value_color(v, lo, hi, 230));
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

        // Hinge legend, in the order of the manual.
        if self.display_mode == DisplayMode::Deformed && rect.width() > 240.0 && show_hinge_legend {
            pushover::draw_legend(&painter, rect);
        }

        // Viewport header overlay: the plane of a plane view names the pane.
        let title = plane.map_or_else(|| title.to_string(), |pl| pl.title());
        // A title wider than the pane (a narrow pane, a small video frame)
        // wraps onto further lines instead of running past the edge.
        let header = painter.layout(
            result_display::scaling::view_title(
                &title,
                &result_display::scaling::case_label(&self.model, &self.active_case),
                subtitle,
            ),
            FontId::proportional(12.5),
            Color32::from_rgb(200, 220, 255),
            (rect.width() - 20.0).max(40.0),
        );
        painter.galley(
            rect.left_top() + Vec2::new(10.0, 8.0),
            header,
            Color32::from_rgb(200, 220, 255),
        );
        // Coordinates under the pointer in a plane view.
        if let (Some(pl), Some(hover)) = (plane, response.hover_pos()) {
            if let Some(p) = view.unproject(hover, pl) {
                let (h, v) = pl.in_plane_axes();
                let axis = ["X", "Y", "Z"];
                painter.text(
                    rect.left_bottom() + Vec2::new(10.0, -8.0),
                    egui::Align2::LEFT_BOTTOM,
                    format!("{} = {:.4}   {} = {:.4}", axis[h], p[h], axis[v], p[v]),
                    FontId::proportional(11.0),
                    Color32::from_rgb(170, 190, 220),
                );
            }
        }
    }

    /// New Model > Grid Only (G36): a blank model in the current units and the
    /// Quick Grid Lines form of the Build window.
    fn start_grid_only(&mut self) {
        let u = self.model.present_units;
        self.replace_model(
            SapModel::new(u),
            format!("Grid Only: blank model in {}; define the grid lines.", u.s2k_label()),
            Vec::new(),
        );
        self.panel_build.show_tab(build::TAB_GRID);
    }

    fn compute_bounds(&self) -> ([f64; 3], [f64; 3]) {
        view::model_bounds(&self.model)
    }

    /// Sets `deform_scale` from the model and the shown result while the
    /// automatic scale is on and a deformed shape is shown (D32). A factor that
    /// is not the one set here was changed by the user: the scale is then
    /// user-defined.
    fn update_deform_scale(&mut self) {
        if !self.deform_auto {
            return;
        }
        if self.deform_auto_written.is_some_and(|w| w != self.deform_scale) {
            self.deform_auto = false;
            return;
        }
        if self.display_mode != DisplayMode::Deformed {
            return;
        }
        let Some(cr) = self.model.case_results.get(&self.active_case) else {
            return;
        };
        let sel = contours::StepSelection {
            mode_idx: self.active_mode_idx,
            use_step: self.use_time_step,
            step_idx: self.active_step_idx,
        };
        let (min, max) = self.compute_bounds();
        let f = result_display::scaling::auto_scale_for(&self.model, cr, sel, min, max);
        let f = (f as f32).clamp(f32::MIN_POSITIVE, f32::MAX / 4.0);
        self.deform_scale = f;
        self.deform_auto_written = Some(f);
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
        m.model_name =
            "StructuraX Verification Example 1-004 (Rotated Local Axes 30 deg)".to_string();
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

    /// The equal-spacing templates of the New Model dialog.
    fn generate_template(&self, m: &mut SapModel, span: f64, story_h: f64, bay_y: f64) {
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
    }

    /// The 2D frame template on typed bay widths and storey heights, with the
    /// origin at the first grid line.
    fn generate_custom_2d_frame(&self, m: &mut SapModel) -> Result<(), String> {
        let bays = build::parse_lengths(&self.tpl_bays_text)?;
        let storeys = build::parse_lengths(&self.tpl_storeys_text)?;
        let narrowest = bays.iter().copied().fold(f64::INFINITY, f64::min);
        m.generate_2d_frame_custom(
            self.tpl_frame2d_type,
            &build::cumulative(0.0, &bays),
            &build::cumulative(0.0, &storeys),
            self.tpl_restraint,
            "W18X50",
            "W14X90",
            "W14X90",
            (0.15 * narrowest).max(1e-3),
        )
        .map_err(|e| e.to_string())
    }

    fn apply_draw_outcome(&mut self, outcome: draw_tools::DrawOutcome) {
        match outcome {
            draw_tools::DrawOutcome::Nothing => {}
            draw_tools::DrawOutcome::Status(message) => self.status_msg = message,
            draw_tools::DrawOutcome::Applied(Ok(message)) => {
                self.status_msg = message;
                self.sync_active_case();
            }
            draw_tools::DrawOutcome::Applied(Err(error)) => {
                self.status_msg = format!("Not done: {error}");
            }
            draw_tools::DrawOutcome::Finish => self.tool_mode = ToolMode::Select,
        }
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
                        // The help: "selecting the Grid Only button will access the
                        // Quick Grid Lines form. Use this option to 'start a model
                        // from scratch.'"
                        if ui.button("▦ Grid Only").clicked() {
                            self.start_grid_only();
                            close_tpl = true;
                        }
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
                                ui.label("Grid:");
                                ui.checkbox(
                                    &mut self.tpl_custom_grid,
                                    "Use custom grid spacing",
                                );
                                ui.end_row();
                                if self.tpl_custom_grid {
                                    ui.label("Bay widths along X:");
                                    ui.text_edit_singleline(&mut self.tpl_bays_text);
                                    ui.end_row();
                                    ui.label("Storey heights along Z:");
                                    ui.text_edit_singleline(&mut self.tpl_storeys_text);
                                    ui.end_row();
                                }
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
                        let custom = self.tpl_choice == TemplateChoice::Frame2D
                            && self.tpl_custom_grid;
                        let generated = if custom {
                            self.generate_custom_2d_frame(&mut m)
                        } else {
                            self.generate_template(&mut m, span, story_h, bay_y);
                            Ok(())
                        };
                        match generated {
                            Ok(()) => {
                                self.replace_model(
                                    m,
                                    format!("Generated {:?} parametric model.", self.tpl_choice),
                                    Vec::new(),
                                );
                                self.execute_analysis();
                                close_tpl = true;
                            }
                            Err(error) => self.status_msg = format!("Not done: {error}"),
                        }
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
                        egui::ScrollArea::vertical()
                            .max_height(360.0)
                            .show(ui, |ui| {
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
                                        // The forces of a plane frame that are
                                        // exactly zero carry roundoff: a value
                                        // below a billionth of the largest of
                                        // its table is printed 0.
                                        let peak = stations
                                            .iter()
                                            .flat_map(|st| [st.p, st.v2, st.v3, st.t, st.m2, st.m3])
                                            .filter(|v| v.is_finite())
                                            .fold(0.0_f64, |a, v| a.max(v.abs()));
                                        let q = |v: f64| view::quantity_text(view::without_roundoff(v, peak));
                                        for st in stations {
                                            ui.monospace(view::quantity_text(st.station));
                                            ui.monospace(q(st.p));
                                            ui.monospace(q(st.v2));
                                            ui.monospace(q(st.v3));
                                            ui.monospace(q(st.t));
                                            ui.monospace(q(st.m2));
                                            ui.monospace(q(st.m3));
                                            ui.monospace(view::quantity_text(st.local_disp[1]));
                                            ui.end_row();
                                        }
                                    });
                            });
                    } else {
                        ui.label("No frame station forces for the selected case/combination.");
                    }
                } else {
                    ui.label(
                        sap_model::results::unavailable_message(&self.model, &self.active_case)
                            .unwrap_or_else(|| "Run analysis (F5) to view station results.".to_string()),
                    );
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
                                            ui.monospace(view::quantity_text(res.governing_station));
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
                                            "As Side",
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
                                            ui.monospace(view::quantity_text(res.as_top));
                                            ui.monospace(view::quantity_text(res.as_bot));
                                            ui.monospace(view::quantity_text(res.as_side));
                                            ui.monospace(view::quantity_text(res.av_over_s));
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
                            // Every cell comes from `result_tables::table_text`.
                            let Some(table) =
                                result_tables::table_text(&self.model, &self.active_case, other_tab)
                            else {
                                // D78: say why a combination has no results.
                                ui.label(
                                    sap_model::results::unavailable_message(
                                        &self.model,
                                        &self.active_case,
                                    )
                                    .unwrap_or_else(|| {
                                        "No analysis results for active case/combo — press F5 to run analysis."
                                            .to_string()
                                    }),
                                );
                                return;
                            };
                            if let Some(message) = &table.empty {
                                ui.label(message);
                                return;
                            }
                            egui::Grid::new(table.grid)
                                .striped(true)
                                .spacing([14.0, 4.0])
                                .show(ui, |ui| {
                                    for h in &table.headers {
                                        ui.strong(h);
                                    }
                                    ui.end_row();
                                    for row in &table.rows {
                                        for (c, cell) in row.iter().enumerate() {
                                            if (c == 0 && !table.first_is_number)
                                                || table.headers[c] == result_tables::STEP_TYPE
                                            {
                                                ui.label(cell);
                                            } else {
                                                ui.monospace(cell);
                                            }
                                        }
                                        ui.end_row();
                                    }
                                });
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
                                CustomSectionShape::WideFlangeI => {
                                    ui.label("Depth t3:");
                                    ui.add(
                                        egui::DragValue::new(&mut self.custom_sec_d)
                                            .range(1e-4..=1e6),
                                    );
                                    ui.end_row();
                                    ui.label("Top flange width t2:");
                                    ui.add(
                                        egui::DragValue::new(&mut self.custom_sec_b)
                                            .range(1e-4..=1e6),
                                    );
                                    ui.end_row();
                                    ui.label("Top flange thickness tf:");
                                    ui.add(
                                        egui::DragValue::new(&mut self.custom_sec_tf)
                                            .range(1e-4..=1e6),
                                    );
                                    ui.end_row();
                                    ui.label("Bottom flange:");
                                    ui.checkbox(
                                        &mut self.custom_sec_symmetric,
                                        "Same as the top flange",
                                    );
                                    ui.end_row();
                                    if self.custom_sec_symmetric {
                                        self.custom_sec_bb = self.custom_sec_b;
                                        self.custom_sec_tfb = self.custom_sec_tf;
                                    }
                                    ui.label("Bottom flange width t2b:");
                                    ui.add_enabled(
                                        !self.custom_sec_symmetric,
                                        egui::DragValue::new(&mut self.custom_sec_bb)
                                            .range(1e-4..=1e6),
                                    );
                                    ui.end_row();
                                    ui.label("Bottom flange thickness tfb:");
                                    ui.add_enabled(
                                        !self.custom_sec_symmetric,
                                        egui::DragValue::new(&mut self.custom_sec_tfb)
                                            .range(1e-4..=1e6),
                                    );
                                    ui.end_row();
                                    ui.label("Web thickness tw:");
                                    ui.add(
                                        egui::DragValue::new(&mut self.custom_sec_tw)
                                            .range(1e-4..=1e6),
                                    );
                                    ui.end_row();
                                }
                                CustomSectionShape::BoxTube => {
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
                            let candidates = rebar_materials(&self.model);
                            if candidates.is_empty() {
                                ui.colored_label(
                                    egui::Color32::YELLOW,
                                    "The model has no material with a yield stress to use as rebar. Add one from the material library (Define > Materials from Library...).",
                                );
                            } else {
                                if !candidates.contains(&self.custom_sec_rebar_material) {
                                    self.custom_sec_rebar_material =
                                        default_rebar_material(&self.model, &candidates);
                                }
                                ui.horizontal(|ui| {
                                    ui.label("Rebar Material (longitudinal and confinement):");
                                    egui::ComboBox::from_id_salt("sec_rebar_mat_combo")
                                        .selected_text(&self.custom_sec_rebar_material)
                                        .show_ui(ui, |ui| {
                                            for mname in &candidates {
                                                ui.selectable_value(
                                                    &mut self.custom_sec_rebar_material,
                                                    mname.clone(),
                                                    mname,
                                                );
                                            }
                                        });
                                });
                            }
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
                        let i_dims = i_section::IDimensions {
                            d,
                            bt: b,
                            tt: tf,
                            bb: self.custom_sec_bb,
                            tb: self.custom_sec_tfb,
                            tw,
                        };
                        let i_err = (self.custom_sec_shape == CustomSectionShape::WideFlangeI)
                            .then(|| i_section::validate(&i_dims).err())
                            .flatten();
                        let geom_err = if let Some(e) = i_err.as_deref() {
                            Some(e)
                        } else if name.is_empty() {
                            Some("Section name cannot be empty.")
                        } else if !self.model.materials.contains_key(&self.custom_sec_material) {
                            Some("Select an existing material before creating a section.")
                        } else {
                            match self.custom_sec_shape {
                                // Checked, with the field named, by `i_section::validate`.
                                CustomSectionShape::WideFlangeI => None,
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
                                Some("Concrete cover must satisfy 0 < 2*cover < depth.".to_string())
                            } else if self.custom_sec_is_column
                                && (!self.custom_sec_rebar_area.is_finite()
                                    || self.custom_sec_rebar_area <= 0.0)
                            {
                                Some("Column longitudinal rebar area must be positive.".to_string())
                            } else {
                                rebar_material_error(&self.model, &self.custom_sec_rebar_material)
                            }
                        } else {
                            None
                        };

                        if let Some(err) = geom_err.map(str::to_string).or(rebar_err) {
                            self.status_msg = err;
                        } else {
                            let mat = self.custom_sec_material.clone();
                            let mut sec = match self.custom_sec_shape {
                                CustomSectionShape::WideFlangeI => {
                                    make_i_section(&name, &mat, d, b, tf, tw, i_dims.bb, i_dims.tb)
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
                                    rebar_mat_long: self.custom_sec_rebar_material.clone(),
                                    rebar_mat_confine: self.custom_sec_rebar_material.clone(),
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
                                "Added section {name} ({}, {mat}{}) and assigned to {assigned} selected frames.",
                                self.custom_sec_shape.label(),
                                if self.custom_sec_rebar_enabled {
                                    format!(", rebar {}", self.custom_sec_rebar_material)
                                } else {
                                    String::new()
                                }
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
                    ui.horizontal(|ui| {
                        ui.text_edit_singleline(&mut self.file_path_input);
                        if ui.button("Browse...").clicked() {
                            file_browser::open_for(
                                &mut self.file_browser,
                                file_browser::Purpose::ModelFile,
                                std::path::Path::new(&self.file_path_input),
                                &["s2k", "$2k", "sdb"],
                                false,
                            );
                        }
                    });
                    if let Some(path) = file_browser::take_choice(
                        &mut self.file_browser,
                        file_browser::Purpose::ModelFile,
                    ) {
                        self.file_path_input = path.display().to_string();
                    }
                    ui.horizontal(|ui| {
                        if ui.button("📂 Open Model").clicked() {
                            let path = self.file_path_input.clone();
                            close_file = self.open_model_file(&path);
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
        self.track_result_clears(|app| {
            app.render_workflows(ctx);
            app.render_analysis_editor(ctx);
            app.render_panels(ctx);
        });
    }

    /// Windows of the panel modules. A panel returns the outcome of a command
    /// the user ran in it; a panel that changes the model definition refuses
    /// while the model is locked, so no result can outlive its model.
    fn render_panels(&mut self, ctx: &egui::Context) {
        self.panel_time_history.revision = self.results_revision;
        let outcomes = [
            build::render(ctx, &mut self.panel_build, &mut self.model),
            seismic::render(ctx, &mut self.panel_seismic, &mut self.model),
            steel_design::render(ctx, &mut self.panel_steel_design, &mut self.model),
            time_history::render(
                ctx,
                &mut self.panel_time_history,
                &mut self.model,
                &mut self.file_browser,
            ),
            materials::render(
                ctx,
                &mut self.panel_materials,
                &mut self.model,
                &mut self.file_browser,
            ),
            section_import::render(
                ctx,
                &mut self.panel_section_import,
                &mut self.model,
                &mut self.file_browser,
            ),
            modal_tables::render(ctx, &mut self.panel_modal_tables, &mut self.model),
            hinges::render(ctx, &mut self.panel_hinges, &mut self.model),
            pushover::render(ctx, &mut self.panel_pushover, &mut self.model),
            hinge_results::render(ctx, &mut self.panel_hinge_results, &self.model),
        ];
        for outcome in outcomes.into_iter().flatten() {
            self.status_msg = match outcome {
                Ok(message) => message,
                Err(error) => format!("Not done: {error}"),
            };
        }
        file_browser::render(ctx, &mut self.file_browser);
        if draw_tools::is_draw_tool(self.tool_mode) {
            draw_tools::render_properties(ctx, &mut self.draw_tools, self.tool_mode, &self.model);
        }
    }

    pub(crate) fn render_ui(&mut self, ctx: &egui::Context) {
        selection::track(&mut self.selection, &self.model);
        if !self.dual_pane {
            self.last_view[1] = None;
            self.active_pane = 0;
        }
        if self.animate_deformed {
            // A result with steps is animated through its steps; the input
            // time of the frame drives it. Modal results keep the sine.
            let now = ctx.input(|i| i.time);
            let start = *self.results.anim_start.get_or_insert(now);
            let step = result_display::step_animation(
                self.model.case_results.get(&self.active_case),
                start,
                now,
            );
            self.results.stepping = step.is_some();
            if let Some(step) = step {
                self.active_step_idx = step;
                self.use_time_step = true;
            } else {
                self.anim_phase += 0.08;
            }
            ctx.request_repaint();
        } else {
            self.results.anim_start = None;
            self.results.stepping = false;
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
                // The status line stops where the scale controls and the units
                // selector on the right begin (a long message overlapped them).
                let room = (ui.available_width() - STATUS_RIGHT_CONTROLS_WIDTH).max(120.0);
                ui.allocate_ui_with_layout(
                    egui::vec2(room, ui.spacing().interact_size.y),
                    egui::Layout::left_to_right(egui::Align::Center),
                    |ui| {
                        // A label cut to the line shows the whole text as its
                        // tooltip; the Messages window (Help) lists the session's.
                        ui.add(egui::Label::new(self.status_msg.as_str()).truncate());
                    },
                );

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    // Bottom-Right Live Units Selector (1..=16)
                    let mut cur_u = self.model.present_units;
                    egui::ComboBox::from_id_salt("units_selector")
                        .selected_text(format!("Units: {}", cur_u.s2k_label()))
                        .show_ui(ui, |ui| {
                            for code in 1..=16 {
                                if let Ok(u) = Units::from_code(code) {
                                    if ui.selectable_value(&mut cur_u, u, u.s2k_label()).clicked() {
                                        self.change_units(u);
                                    }
                                }
                            }
                        });
                    ui.separator();
                    ui.checkbox(&mut self.animate_deformed, "Animate");
                    if ui.checkbox(&mut self.deform_auto, "Auto scale").changed() {
                        self.deform_auto_written = Some(self.deform_scale);
                    }
                    ui.add(
                        egui::Slider::new(&mut self.deform_scale, 1e-3..=1e6)
                            .logarithmic(true)
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
                    self.render_viewport(&mut cols[0], 0, "3D Perspective View");
                    self.render_viewport(&mut cols[1], 1, "2D Plan / Elevation View");
                });
            } else {
                self.render_viewport(ui, 0, "3D Perspective View");
            }
        });

        self.render_modals(ctx);
        self.render_result_forms(ctx);
        object_info::window(ctx, &mut self.object_info, &self.model);
        self.render_video_form(ctx);
        self.render_set_2d_view(ctx);
        self.messages.observe(&self.status_msg);
        messages::window(ctx, &mut self.messages);
    }
}

impl eframe::App for SapGuiApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.render_ui(ctx);
    }
}

/// Width the status bar keeps free for the controls on its right.
const STATUS_RIGHT_CONTROLS_WIDTH: f32 = 780.0;

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

/// The model materials that can be named as the rebar of a section: those
/// with a positive yield stress (concrete design refuses a section whose rebar
/// material has none and whose concrete has none).
pub(super) fn rebar_materials(model: &SapModel) -> Vec<String> {
    model
        .materials
        .iter()
        .filter(|(_, m)| m.fy.is_finite() && m.fy > 0.0)
        .map(|(name, _)| name.clone())
        .collect()
}

/// The rebar material a dialog offers first: one of rebar type, else the first.
fn default_rebar_material(model: &SapModel, candidates: &[String]) -> String {
    candidates
        .iter()
        .find(|n| model.materials.get(*n).is_some_and(|m| m.mat_type == MatType::Rebar))
        .or_else(|| candidates.first())
        .cloned()
        .unwrap_or_default()
}

/// Why `name` cannot be the rebar material of a section, if it cannot.
fn rebar_material_error(model: &SapModel, name: &str) -> Option<String> {
    if rebar_materials(model).is_empty() {
        return Some(
            "The model has no material with a yield stress to use as rebar; add one from the material library (Define > Materials from Library...)."
                .to_string(),
        );
    }
    match model.materials.get(name) {
        None => Some("Select an existing rebar material for the reinforcement.".to_string()),
        Some(m) if !(m.fy.is_finite() && m.fy > 0.0) => {
            Some(format!("Rebar material '{name}' has no yield stress."))
        }
        Some(_) => None,
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

/// The expression the deformed shape used before joint-local results were
/// rotated to global axes; kept as the reference of the tests.
#[cfg(test)]
fn joint_disp_3d(
    cr: &CaseResults,
    pid: &str,
    scale: f64,
    mode_idx: usize,
    use_step: bool,
    step_idx: usize,
) -> [f64; 3] {
    let raw = contours::joint_displacement_local(
        cr,
        pid,
        contours::StepSelection {
            mode_idx,
            use_step,
            step_idx,
        },
    );
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

    /// D78: ULS = 1.2 D + 1.6 L with L not run: the analysis message and the design
    /// message name the combination and the case, the list and the tables refuse it.
    #[test]
    fn unavailable_combination_is_named_in_analysis_and_design_messages() {
        let text = "TABLE: \"PROGRAM CONTROL\"\n CurrUnits=\"kN, m, C\"\n\
TABLE: \"JOINT COORDINATES\"\n Joint=J XorR=0 Y=0 Z=0\n\
TABLE: \"JOINT RESTRAINT ASSIGNMENTS\"\n Joint=J U2=Yes U3=Yes R1=Yes R2=Yes R3=Yes\n\
TABLE: \"JOINT SPRING ASSIGNMENTS 1 - UNCOUPLED\"\n Joint=J U1=100\n\
TABLE: \"LOAD PATTERN DEFINITIONS\"\n LoadPat=D DesignType=Dead SelfWtMult=0\n LoadPat=L DesignType=Live SelfWtMult=0\n\
TABLE: \"JOINT LOADS - FORCE\"\n Joint=J LoadPat=D F1=10\n Joint=J LoadPat=L F1=30\n\
TABLE: \"LOAD CASE DEFINITIONS\"\n Case=D Type=LinStatic RunCase=Yes\n Case=L Type=LinStatic RunCase=No\n\
TABLE: \"COMBINATION DEFINITIONS\"\n ComboName=ULS ComboType=\"Linear Add\" CaseName=D ScaleFactor=1.2\n \
ComboName=ULS ComboType=\"Linear Add\" CaseName=L ScaleFactor=1.6\nEND TABLE DATA\n";
        let mut app = blank_app();
        app.model = sap_format::s2k::parse_s2k(text).unwrap();
        assert!(app.execute_analysis());
        assert!(
            app.status_msg.contains("'ULS'") && app.status_msg.contains("'L'"),
            "{}",
            app.status_msg
        );
        assert!(app.model.case_results.contains_key("D"));
        assert!(!app.model.case_results.contains_key("ULS"));
        app.active_case = "ULS".into();
        assert!(result_tables::table_text(&app.model, "ULS", TableTab::JointDisplacements).is_none());
        let msg = sap_model::results::unavailable_message(&app.model, "ULS").unwrap();
        assert!(msg.contains("'ULS'") && msg.contains("'L'"), "{msg}");
        app.execute_design(true);
        assert!(
            app.status_msg.starts_with("Design Error:")
                && app.status_msg.contains("'ULS'")
                && app.status_msg.contains("'L'"),
            "{}",
            app.status_msg
        );
        assert!(app.model.steel_design_results.is_empty());
    }

    /// G16: the run message names a case that is not finished (with the reason and the
    /// number of saved steps) and a case that could not start; the view title says so.
    #[test]
    fn the_run_message_names_cases_that_are_not_finished_and_could_not_start() {
        let mut app = blank_app();
        let mut model = pushover::example_1_026_model();
        model.load_cases.get_mut("PUSH").unwrap().nl_static.max_total_steps = 5;
        let mut next = model.load_cases["PUSH"].clone();
        next.name = "NEXT".into();
        next.initial_case = Some("PUSH".into());
        model.load_cases.insert("NEXT".into(), next);
        app.model = model;
        assert!(app.execute_analysis());
        let msg = &app.status_msg;
        assert!(
            msg.contains("Not finished: 'PUSH' (") && msg.contains("total steps") && msg.contains("saved steps kept"),
            "{msg}"
        );
        assert!(msg.contains("Unavailable: 'NEXT' (could not start:") && msg.contains("'PUSH'"), "{msg}");
        assert!(!app.model.case_results.contains_key("NEXT"));
        assert_eq!(
            result_display::scaling::case_label(&app.model, "PUSH"),
            "PUSH (not finished)"
        );
        assert_eq!(result_display::scaling::case_label(&app.model, "NEXT"), "NEXT");
        assert!(result_display::result_names(&app.model).iter().all(|n| n != "NEXT"));
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
            as_side: 0.0,
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
            notes: Vec::new(),
        };
        assert_eq!(steel_result_color(&steel_slender), red);
    }

    #[test]
    fn window_code_that_clears_results_bumps_the_revision_and_drops_the_reasons() {
        // D124: a template rebuilds the model through `invalidate_results`; the
        // application sees the stores go from full to empty and bumps the revision.
        let mut app = blank_app();
        app.model
            .case_results
            .insert("stale".into(), Default::default());
        app.model
            .unavailable_results
            .insert("COMBO1".into(), "stale reason".into());
        let before = app.results_revision;
        app.track_result_clears(|app| app.model.generate_beam(2, 5.0, true, "FSEC1"));
        assert!(app.model.case_results.is_empty());
        assert!(app.model.unavailable_results.is_empty());
        assert_eq!(app.results_revision, before + 1);
        // Negative: code that clears nothing leaves the revision alone.
        app.track_result_clears(|app| app.model.add_point("Z", 0.0, 0.0, 9.0));
        assert_eq!(app.results_revision, before + 1);
        // The application's own function clears the reasons and bumps as well.
        app.model
            .unavailable_results
            .insert("COMBO2".into(), "stale".into());
        app.invalidate_results();
        assert!(app.model.unavailable_results.is_empty());
        assert_eq!(app.results_revision, before + 2);
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
    fn the_five_panel_windows_render_inside_the_application_before_and_after_analysis() {
        let mut app = blank_app();
        app.model.generate_beam(2, 120.0, true, "FSEC1");
        let titles = |out: &egui::FullOutput| -> Vec<String> {
            out.shapes
                .iter()
                .filter_map(|shape| match &shape.shape {
                    egui::epaint::Shape::Text(text) => Some(text.galley.job.text.clone()),
                    _ => None,
                })
                .collect()
        };
        let input = |time| egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1800., 1400.))),
            time: Some(time),
            ..Default::default()
        };
        let ctx = egui::Context::default();
        let closed = ctx.run(input(0.0), |ctx| app.render_ui(ctx));
        let before = titles(&closed).len();

        app.panel_build.open = true;
        app.panel_hinges.open = true;
        app.panel_modal_tables.open = true;
        app.panel_pushover.open = true;
        app.panel_seismic.open = true;
        app.panel_steel_design.open = true;
        app.panel_time_history.open = true;
        let _ = ctx.run(input(1.0), |ctx| app.render_ui(ctx));
        let open = ctx.run(input(2.0), |ctx| app.render_ui(ctx));
        assert!(
            titles(&open).len() > before + 5,
            "the open panels must draw their own content"
        );

        // With results in the model the panels that read them still render.
        app.execute_analysis();
        assert!(app.model.is_locked, "{}", app.status_msg);
        let _ = ctx.run(input(3.0), |ctx| app.render_ui(ctx));
        let locked = ctx.run(input(4.0), |ctx| app.render_ui(ctx));
        assert!(titles(&locked).len() > before + 5);
        assert!(app.model.is_locked, "rendering must not unlock or edit the model");

        // Loading another model closes the panels.
        app.replace_model(SapModel::new(Units::KipInF), "x".into(), Vec::new());
        assert!(
            !(app.panel_build.open
                || app.panel_hinges.open
                || app.panel_modal_tables.open
                || app.panel_pushover.open
                || app.panel_seismic.open
                || app.panel_steel_design.open
                || app.panel_time_history.open)
        );
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

    // ---- Saved states of nonlinear static cases (Example 1-026 A) ----

    fn pushover_app() -> SapGuiApp {
        let mut app = blank_app();
        app.model = pushover::example_1_026_model();
        app.active_case = "PUSH".into();
        app.display_mode = DisplayMode::Deformed;
        // One viewport, so each marker is drawn once.
        app.dual_pane = false;
        app
    }

    /// Shapes of one frame, flattened.
    fn flat_shapes(out: &egui::FullOutput) -> Vec<egui::epaint::Shape> {
        fn walk(shape: &egui::epaint::Shape, into: &mut Vec<egui::epaint::Shape>) {
            match shape {
                egui::epaint::Shape::Vec(v) => v.iter().for_each(|s| walk(s, into)),
                other => into.push(other.clone()),
            }
        }
        let mut all = Vec::new();
        for clipped in &out.shapes {
            walk(&clipped.shape, &mut all);
        }
        all
    }

    /// Colours of the hinge markers (radius 5.5 filled circles in a class
    /// colour) and the number of legend entries drawn.
    fn marker_colours(out: &egui::FullOutput) -> (Vec<egui::Color32>, usize) {
        use sap_model::hinge::HingeDisplayClass as C;
        let class_colours: Vec<egui::Color32> =
            C::ALL.iter().map(|c| pushover::class_color(*c)).collect();
        let mut markers = Vec::new();
        let mut legend = 0;
        for shape in flat_shapes(out) {
            match shape {
                egui::epaint::Shape::Circle(c) if class_colours.contains(&c.fill) => {
                    if (c.radius - 5.5).abs() < 1e-6 {
                        markers.push(c.fill);
                    } else if (c.radius - 4.0).abs() < 1e-6 {
                        legend += 1;
                    }
                }
                _ => {}
            }
        }
        (markers, legend)
    }

    fn view(app: &mut SapGuiApp, ctx: &egui::Context, time: f64) -> egui::FullOutput {
        let input = egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1800., 1400.))),
            time: Some(time),
            ..Default::default()
        };
        ctx.run(input, |ctx| app.render_ui(ctx))
    }

    #[test]
    fn hinge_markers_and_legend_appear_after_analysis_and_follow_the_saved_state() {
        use sap_model::hinge::HingeDisplayClass as C;
        let col = |c: C| pushover::class_color(c);
        let mut app = pushover_app();
        let ctx = egui::Context::default();
        // Before the analysis: no markers, no legend.
        let _ = view(&mut app, &ctx, 0.0);
        let (markers, legend) = marker_colours(&view(&mut app, &ctx, 1.0));
        assert!(markers.is_empty() && legend == 0);

        app.execute_analysis();
        assert!(app.model.is_locked, "{}", app.status_msg);
        assert_eq!(app.model.case_results["PUSH"].static_steps.len(), 10);
        // Final state: the moment hinge is D-E, the shear hinge C-D; the
        // legend has seven entries.
        let (markers, legend) = marker_colours(&view(&mut app, &ctx, 2.0));
        assert_eq!(markers, vec![col(C::DToE), col(C::CToD)]);
        assert_eq!(legend, 7);
        // The legend names the classes in the order of the manual.
        let out = view(&mut app, &ctx, 3.0);
        let labels: Vec<String> = flat_shapes(&out)
            .into_iter()
            .filter_map(|s| match s {
                egui::epaint::Shape::Text(t) => Some(t.galley.job.text.clone()),
                _ => None,
            })
            .collect();
        let at = |l: &str| labels.iter().position(|t| t == l).unwrap_or(usize::MAX);
        let order: Vec<usize> = [
            "B to IO", "IO to LS", "LS to CP", "CP to C", "C to D", "D to E", ">E",
        ]
        .iter()
        .map(|l| at(l))
        .collect();
        assert!(order.iter().all(|i| *i != usize::MAX), "{labels:?}");
        assert!(
            order.windows(2).all(|w| w[0] < w[1]),
            "legend order {order:?}"
        );
        // Scrubbing the saved states changes the markers.
        app.use_time_step = true;
        for (step, want) in [
            (0usize, vec![]),
            (1, vec![]),
            (2, vec![col(C::BToIO)]),
            (4, vec![col(C::BToIO), col(C::BToIO)]),
            (6, vec![col(C::CToD), col(C::CToD)]),
            (8, vec![col(C::DToE), col(C::CToD)]),
        ] {
            app.active_step_idx = step;
            let (markers, legend) = marker_colours(&view(&mut app, &ctx, 4.0 + step as f64));
            assert_eq!(markers, want, "markers at saved state {step}");
            assert_eq!(legend, 7);
        }
        // Another display mode draws neither markers nor legend.
        app.display_mode = DisplayMode::Undeformed;
        let (markers, legend) = marker_colours(&view(&mut app, &ctx, 20.0));
        assert!(markers.is_empty() && legend == 0);
    }

    #[test]
    fn the_saved_state_slider_drives_the_deformed_shape_from_static_steps() {
        let mut app = pushover_app();
        app.execute_analysis();
        let cr = &app.model.case_results["PUSH"];
        let tip = |step: usize, use_step: bool| joint_disp_3d(cr, "2", 1.0, 0, use_step, step)[2];
        // Without scrubbing the final state is shown; with it the saved state.
        assert_eq!(tip(0, false), cr.joint_displacements["2"][2]);
        assert!((tip(0, false) - 2.0).abs() < 1e-12);
        for step in 0..cr.static_steps.len() {
            assert_eq!(
                tip(step, true),
                cr.static_steps[step].joint_displacements["2"][2]
            );
        }
        assert!((tip(6, true) - 1.3846694).abs() < 5e-7);
        // The scale factor applies and an index past the end is clamped.
        let scaled = joint_disp_3d(cr, "2", 10.0, 0, true, 99)[2];
        assert!((scaled - 20.0).abs() < 1e-9);
        // The slider range follows the saved states and is reset with the case.
        app.active_step_idx = 99;
        app.use_time_step = true;
        app.sync_active_case();
        assert_eq!(app.active_step_idx, 9);
        assert!(app.use_time_step);
        app.model.case_results.shift_remove("PUSH");
        app.sync_active_case();
        assert_eq!(app.active_step_idx, 0);
        assert!(!app.use_time_step);
    }

    #[test]
    fn the_explorer_lists_the_saved_states_of_a_nonlinear_static_case() {
        let mut app = pushover_app();
        app.execute_analysis();
        let ctx = egui::Context::default();
        let mut found = false;
        for t in 0..3 {
            let out = view(&mut app, &ctx, t as f64);
            found |= flat_shapes(&out).into_iter().any(|s| match s {
                egui::epaint::Shape::Text(t) => {
                    t.galley.job.text.contains("Nonlinear Static Steps (10)")
                }
                _ => false,
            });
        }
        assert!(found, "the explorer must show the saved-state header");
    }

    // ---- Plane views, window selection and previous selection ----

    use sap_model::generate::GridRegion;

    /// The Part 1 grid (6 x 4 x 7 lines at 6, 5 and 3.2 m) with columns, beams
    /// and slab areas, drawn with the index commands. Beams and slabs start at
    /// the second Z line, as in a building whose base level has columns only.
    fn part1_model() -> SapModel {
        let mut m = SapModel::new(Units::KnMC);
        m.quick_grid([6, 4, 7], [6.0, 5.0, 3.2], [0.0; 3]).unwrap();
        let all = GridRegion {
            x: (0, 5),
            y: (0, 3),
            z: (0, 6),
        };
        let floors = GridRegion { z: (1, 6), ..all };
        m.quick_draw_columns(all, "FSEC1").unwrap();
        m.quick_draw_beams(floors, true, true, "FSEC1").unwrap();
        m.quick_draw_areas(floors, "ASEC1").unwrap();
        m
    }

    /// Two bays, one storey, in the X-Z plane at y = 0 (columns C1..C3, beams
    /// B1 and B2).
    fn portal_model() -> SapModel {
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

    fn one_pane(app: &mut SapGuiApp, proj: ViewProjection, plane: Option<f64>) {
        app.dual_pane = false;
        app.left_proj = proj;
        app.left_plane = plane;
    }

    fn texts(out: &egui::FullOutput) -> Vec<(String, Pos2)> {
        flat_shapes(out)
            .into_iter()
            .filter_map(|s| match s {
                egui::epaint::Shape::Text(t) => Some((t.galley.job.text.clone(), t.pos)),
                _ => None,
            })
            .collect()
    }

    fn frame_labels(out: &egui::FullOutput) -> Vec<String> {
        let mut v: Vec<String> = texts(out)
            .into_iter()
            .map(|(t, _)| t)
            .filter(|t| t.ends_with(" (FSEC1)"))
            .collect();
        v.sort();
        v
    }

    fn input_with(time: f64, events: Vec<egui::Event>, modifiers: egui::Modifiers) -> egui::RawInput {
        egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1800., 1400.))),
            time: Some(time),
            events,
            modifiers,
            ..Default::default()
        }
    }

    /// One frame with pointer events.
    fn frame_with(
        app: &mut SapGuiApp,
        ctx: &egui::Context,
        time: &mut f64,
        events: Vec<egui::Event>,
        modifiers: egui::Modifiers,
    ) -> egui::FullOutput {
        *time += 0.05;
        ctx.run(input_with(*time, events, modifiers), |ctx| app.render_ui(ctx))
    }

    fn button(pos: Pos2, b: egui::PointerButton, pressed: bool, m: egui::Modifiers) -> egui::Event {
        egui::Event::PointerButton {
            pos,
            button: b,
            pressed,
            modifiers: m,
        }
    }

    /// A pointer drag from `from` to `to`: press, two moves, release. Returns
    /// the output of the frame after the first move (drag in progress).
    fn drag(
        app: &mut SapGuiApp,
        ctx: &egui::Context,
        time: &mut f64,
        from: Pos2,
        to: Pos2,
        b: egui::PointerButton,
        m: egui::Modifiers,
    ) -> egui::FullOutput {
        let mid = from + (to - from) * 0.5;
        frame_with(app, ctx, time, vec![egui::Event::PointerMoved(from)], m);
        frame_with(app, ctx, time, vec![button(from, b, true, m)], m);
        let _ = frame_with(app, ctx, time, vec![egui::Event::PointerMoved(mid)], m);
        let during = frame_with(app, ctx, time, vec![egui::Event::PointerMoved(to)], m);
        frame_with(app, ctx, time, vec![button(to, b, false, m)], m);
        frame_with(app, ctx, time, vec![], m);
        during
    }

    fn pane0(app: &SapGuiApp) -> (ViewTransform, Option<PlaneView>) {
        app.last_view[0].expect("the pane has been drawn")
    }

    #[test]
    fn plane_levels_and_steps_follow_the_grid() {
        let m = part1_model();
        let z = [0, 1, 2, 3, 4, 5, 6].map(|i| i as f64 * 3.2);
        let levels = view::plane_levels(&m, view::PlaneAxis::Z);
        assert_eq!(levels, z.to_vec(), "the seven stored grid floats");
        let tol = view::plane_tolerance(&m);
        assert_eq!(view::step_level(&levels, None, true, tol), Some(z[0]));
        assert_eq!(view::step_level(&levels, Some(z[6]), true, tol), None);
        assert_eq!(view::step_level(&levels, Some(z[2]), false, tol), Some(z[1]));
    }

    #[test]
    fn a_floor_plan_holds_its_beams_and_slabs_and_an_elevation_its_columns_and_beams() {
        let m = part1_model();
        let tol = view::plane_tolerance(&m);
        let z2 = 2.0 * 3.2;
        let plan = PlaneView::of(ViewProjection::PlanXY, Some(z2)).unwrap();
        let members = view::plane_members(&m, plan, tol);
        // 5 bays x 4 lines along X, 3 bays x 6 lines along Y; no column.
        assert_eq!(members.frames.len(), 20 + 18);
        assert_eq!(members.areas.len(), 5 * 3);
        assert_eq!(members.points.len(), 6 * 4);
        let elevation = PlaneView::of(ViewProjection::ElevationXZ, Some(5.0)).unwrap();
        // 6 columns x 6 storeys + 5 beams x 6 floors.
        assert_eq!(view::plane_members(&m, elevation, tol).frames.len(), 36 + 30);
    }

    #[test]
    fn a_plane_view_draws_exactly_the_frames_in_its_plane() {
        let mut app = blank_app();
        app.model = part1_model();
        app.show_frame_labels = true;
        let z2 = 2.0 * 3.2;
        let ctx = egui::Context::default();
        // Whole model first: every frame is labelled.
        one_pane(&mut app, ViewProjection::PlanXY, None);
        let _ = view(&mut app, &ctx, 0.0);
        let all = frame_labels(&view(&mut app, &ctx, 1.0));
        assert_eq!(all.len(), app.model.frames.len());
        // The plane at z2.
        one_pane(&mut app, ViewProjection::PlanXY, Some(z2));
        let _ = view(&mut app, &ctx, 2.0);
        let out = view(&mut app, &ctx, 3.0);
        let plan = PlaneView::of(ViewProjection::PlanXY, Some(z2)).unwrap();
        let members = view::plane_members(&app.model, plan, view::plane_tolerance(&app.model));
        let mut want: Vec<String> = members
            .frames
            .iter()
            .map(|n| format!("{n} (FSEC1)"))
            .collect();
        want.sort();
        assert_eq!(want.len(), 38);
        assert_eq!(frame_labels(&out), want);
        // The header names the plane.
        assert!(
            texts(&out).iter().any(|(t, _)| t.starts_with("X-Y Plane @ Z=6.4")),
            "{:?}",
            texts(&out).iter().map(|(t, _)| t.clone()).collect::<Vec<_>>()
        );
        // Another display mode does not bring the other frames back.
        app.display_mode = DisplayMode::Reactions;
        let out = view(&mut app, &ctx, 4.0);
        assert_eq!(frame_labels(&out), want);
    }

    #[test]
    fn a_window_in_an_elevation_follows_the_help_rule_in_either_direction() {
        let mut app = blank_app();
        app.model = portal_model();
        one_pane(&mut app, ViewProjection::ElevationXZ, Some(0.0));
        let ctx = egui::Context::default();
        let _ = view(&mut app, &ctx, 0.0);
        let (v, plane) = pane0(&app);
        assert!(plane.is_some());
        // Bay 1 is x = 0..6; the window ends at x = 9, inside beam 2.
        let a = v.project([-1.0, 0.0, 5.0]);
        let b = v.project([9.0, 0.0, -1.0]);
        let mut m = app.model.clone();
        selection::select_window(&mut m, &v, plane, a, b);
        let s = selection::snapshot(&m);
        assert_eq!(
            s.frames.iter().map(String::as_str).collect::<Vec<_>>(),
            ["C0", "C1", "G1"]
        );
        assert_eq!(s.points.len(), 4);
        let mut m = app.model.clone();
        selection::select_window(&mut m, &v, plane, b, a);
        let s = selection::snapshot(&m);
        assert_eq!(
            s.frames.iter().map(String::as_str).collect::<Vec<_>>(),
            ["C0", "C1", "G1", "G2"]
        );
        assert_eq!(s.points.len(), 4);
    }

    #[test]
    fn the_previous_selection_survives_clearing_in_the_application() {
        let mut app = blank_app();
        app.model = portal_model();
        let ctx = egui::Context::default();
        let _ = view(&mut app, &ctx, 0.0);
        assert!(selection::restore_previous(&app.selection, &mut app.model).is_err());
        app.model.frames.get_mut("C0").unwrap().selected = true;
        app.model.frames.get_mut("G1").unwrap().selected = true;
        let _ = view(&mut app, &ctx, 1.0);
        app.clear_selection();
        let _ = view(&mut app, &ctx, 2.0);
        assert!(selection::snapshot(&app.model).is_empty());
        let msg = selection::restore_previous(&app.selection, &mut app.model).unwrap();
        assert!(msg.contains('2'), "{msg}");
        let s = selection::snapshot(&app.model);
        assert_eq!(s.frames.iter().map(String::as_str).collect::<Vec<_>>(), ["C0", "G1"]);
        // A new model forgets it.
        app.replace_model(portal_model(), "x".into(), Vec::new());
        assert!(selection::restore_previous(&app.selection, &mut app.model).is_err());
    }

    #[test]
    fn a_primary_drag_in_a_2d_pane_selects_and_shift_or_middle_drag_pans() {
        let mut app = blank_app();
        app.model = portal_model();
        one_pane(&mut app, ViewProjection::ElevationXZ, Some(0.0));
        let ctx = egui::Context::default();
        let mut t = 0.0;
        let none = egui::Modifiers::NONE;
        let _ = frame_with(&mut app, &ctx, &mut t, vec![], none);
        let _ = frame_with(&mut app, &ctx, &mut t, vec![], none);
        let (v, _) = pane0(&app);
        let a = v.project([-1.0, 0.0, 5.0]);
        let b = v.project([9.0, 0.0, -1.0]);
        let pan0 = app.pan;

        // Left to right: a solid band while dragging, then bay 1 is selected.
        let during = drag(&mut app, &ctx, &mut t, a, b, egui::PointerButton::Primary, none);
        let band = Color32::from_rgba_unmultiplied(255, 235, 150, 25);
        assert!(
            flat_shapes(&during)
                .iter()
                .any(|s| matches!(s, egui::epaint::Shape::Rect(r) if r.fill == band)),
            "a solid rubber band"
        );
        assert_eq!(app.pan, pan0, "a window drag does not pan");
        let s = selection::snapshot(&app.model);
        assert_eq!(
            s.frames.iter().map(String::as_str).collect::<Vec<_>>(),
            ["C0", "C1", "G1"]
        );
        assert!(app.status_msg.contains("selected"), "{}", app.status_msg);

        // Right to left adds beam 2; the band is dashed.
        let during = drag(&mut app, &ctx, &mut t, b, a, egui::PointerButton::Primary, none);
        assert!(
            !flat_shapes(&during)
                .iter()
                .any(|s| matches!(s, egui::epaint::Shape::Rect(r) if r.fill == band)),
            "a dashed band has no fill"
        );
        let s = selection::snapshot(&app.model);
        assert!(s.frames.contains("G2"));
        assert_eq!(app.pan, pan0);

        // Shift drag pans and leaves the selection as it was.
        let before = selection::snapshot(&app.model);
        let shift = egui::Modifiers::SHIFT;
        let _ = drag(&mut app, &ctx, &mut t, Pos2::new(900.0, 600.0), Pos2::new(1000.0, 650.0), egui::PointerButton::Primary, shift);
        assert_ne!(app.pan, pan0, "shift drag pans");
        assert_eq!(selection::snapshot(&app.model), before);
        // Middle drag pans too.
        let pan1 = app.pan;
        let _ = drag(&mut app, &ctx, &mut t, Pos2::new(900.0, 600.0), Pos2::new(800.0, 560.0), egui::PointerButton::Middle, none);
        assert_ne!(app.pan, pan1, "middle drag pans");
        assert_eq!(selection::snapshot(&app.model), before);
    }

    #[test]
    fn in_3d_a_drag_rotates_and_a_ctrl_drag_draws_a_window() {
        let mut app = blank_app();
        app.model = portal_model();
        app.dual_pane = false;
        let ctx = egui::Context::default();
        let mut t = 0.0;
        let none = egui::Modifiers::NONE;
        let _ = frame_with(&mut app, &ctx, &mut t, vec![], none);
        let _ = frame_with(&mut app, &ctx, &mut t, vec![], none);
        let (v, _) = pane0(&app);
        let yaw0 = app.yaw_rad;
        let c = v.screen_center;
        let _ = drag(&mut app, &ctx, &mut t, c, c + Vec2::new(60.0, 0.0), egui::PointerButton::Primary, none);
        assert_ne!(app.yaw_rad, yaw0, "a primary drag rotates the 3D view");
        assert!(selection::snapshot(&app.model).is_empty());
        let yaw1 = app.yaw_rad;
        // Ctrl+drag around the whole model selects it (left to right).
        let (v, _) = pane0(&app);
        let pts: Vec<Pos2> = app
            .model
            .points
            .values()
            .map(|p| v.project([p.x, p.y, p.z]))
            .collect();
        let lo = pts.iter().fold(Pos2::new(f32::MAX, f32::MAX), |a, p| a.min(*p));
        let hi = pts.iter().fold(Pos2::new(f32::MIN, f32::MIN), |a, p| a.max(*p));
        let ctrl = egui::Modifiers::CTRL;
        let _ = drag(&mut app, &ctx, &mut t, lo - Vec2::splat(15.0), hi + Vec2::splat(15.0), egui::PointerButton::Primary, ctrl);
        assert_eq!(app.yaw_rad, yaw1, "ctrl drag does not rotate");
        assert_eq!(selection::snapshot(&app.model).frames.len(), 5);
    }

    #[test]
    fn a_click_in_a_plane_view_cannot_pick_what_the_plane_hides() {
        let mut app = blank_app();
        app.model = portal_model();
        // In plan the column's two joints project to the same point; at z = 4
        // only the top joint is in the view.
        one_pane(&mut app, ViewProjection::PlanXY, Some(4.0));
        let ctx = egui::Context::default();
        let mut t = 0.0;
        let none = egui::Modifiers::NONE;
        let _ = frame_with(&mut app, &ctx, &mut t, vec![], none);
        let _ = frame_with(&mut app, &ctx, &mut t, vec![], none);
        let (v, plane) = pane0(&app);
        assert_eq!(plane.map(|p| p.title()), Some("X-Y Plane @ Z=4".to_string()));
        let at = v.project([0.0, 0.0, 4.0]);
        let _ = frame_with(&mut app, &ctx, &mut t, vec![egui::Event::PointerMoved(at)], none);
        frame_with(&mut app, &ctx, &mut t, vec![button(at, egui::PointerButton::Primary, true, none)], none);
        frame_with(&mut app, &ctx, &mut t, vec![button(at, egui::PointerButton::Primary, false, none)], none);
        frame_with(&mut app, &ctx, &mut t, vec![], none);
        let s = selection::snapshot(&app.model);
        assert_eq!(s.points.iter().map(String::as_str).collect::<Vec<_>>(), ["T0"]);
        // Plane Up from z = 4 has no higher level here (joint coordinates 0, 4
        // are the levels), so the view stays.
        app.right_proj = ViewProjection::Perspective3D;
        app.dual_pane = true;
        app.active_pane = 0;
        app.step_plane(true);
        assert_eq!(app.left_plane, Some(4.0));
        assert!(app.status_msg.contains("No higher"), "{}", app.status_msg);
        app.step_plane(false);
        assert_eq!(app.left_plane, Some(0.0));
    }

    #[test]
    fn plane_up_down_set_view_and_whole_model_drive_the_panes() {
        let mut app = blank_app();
        app.model = part1_model();
        // Left 3D, right XZ elevation (the defaults): the right pane is it.
        assert_eq!(app.plane_pane(), Some(1));
        app.step_plane(true);
        assert_eq!(app.right_plane, Some(0.0), "from the whole model up gives the first plane");
        assert_eq!(app.status_msg, "X-Z Plane @ Y=0");
        app.step_plane(true);
        assert_eq!(app.right_plane, Some(5.0));
        app.step_plane(false);
        app.step_plane(false);
        assert_eq!(app.right_plane, Some(0.0), "the view stays at the lowest plane");
        assert!(app.status_msg.contains("No lower"));
        // Set 2D View: a plan at a typed coordinate in the left pane.
        app.set_plane_view(0, view::PlaneAxis::Z, 3.2);
        assert_eq!(app.left_proj, ViewProjection::PlanXY);
        assert_eq!(app.left_plane, Some(3.2));
        // Changing the projection of a pane drops its plane; the same
        // projection keeps it.
        app.set_pane_projection(0, ViewProjection::PlanXY);
        assert_eq!(app.left_plane, Some(3.2));
        app.set_pane_projection(0, ViewProjection::ElevationYZ);
        assert_eq!(app.left_plane, None);
        // Either pane can show a plan or an elevation.
        app.set_pane_projection(1, ViewProjection::Perspective3D);
        assert_eq!(app.right_proj, ViewProjection::Perspective3D);
        app.set_plane_view(0, view::PlaneAxis::X, 12.0);
        app.right_plane = Some(1.0);
        app.whole_model_view();
        assert_eq!((app.left_plane, app.right_plane), (None, None));
        // Only a 3D pane: Plane Up has nothing to act on.
        app.left_proj = ViewProjection::Perspective3D;
        app.step_plane(true);
        assert!(app.status_msg.contains("needs a plan"), "{}", app.status_msg);
    }

    #[test]
    fn the_set_2d_view_dialog_renders_and_applies_a_typed_coordinate() {
        let mut app = blank_app();
        app.model = part1_model();
        app.open_set_2d_view();
        assert!(app.set_2d.open);
        let ctx = egui::Context::default();
        let _ = view(&mut app, &ctx, 0.0);
        let out = view(&mut app, &ctx, 0.5);
        assert!(texts(&out).iter().any(|(t, _)| t == "Front face of the plane only."));
        // A refused coordinate leaves the view alone.
        app.set_2d.coordinate_text = "abc".into();
        app.set_2d.normal = view::PlaneAxis::Z;
        app.set_2d.pane = 0;
        // Click OK.
        let ok = texts(&view(&mut app, &ctx, 1.0))
            .into_iter()
            .find(|(t, _)| t == "OK")
            .map(|(_, p)| p + Vec2::new(6.0, 6.0))
            .expect("OK button");
        let mut t = 2.0;
        let none = egui::Modifiers::NONE;
        frame_with(&mut app, &ctx, &mut t, vec![egui::Event::PointerMoved(ok)], none);
        frame_with(&mut app, &ctx, &mut t, vec![button(ok, egui::PointerButton::Primary, true, none)], none);
        frame_with(&mut app, &ctx, &mut t, vec![button(ok, egui::PointerButton::Primary, false, none)], none);
        assert!(app.status_msg.contains("not a coordinate"), "{}", app.status_msg);
        assert_eq!(app.left_plane, None);
        // A valid coordinate is applied: the plane, the pane's projection, the
        // status line and the closed dialog.
        assert!(app.set_2d.open);
        app.set_2d.coordinate_text = " 6.4 ".into();
        let ok = texts(&frame_with(&mut app, &ctx, &mut t, vec![], none))
            .into_iter()
            .find(|(t, _)| t == "OK")
            .map(|(_, p)| p + Vec2::new(6.0, 6.0))
            .expect("OK button");
        frame_with(&mut app, &ctx, &mut t, vec![egui::Event::PointerMoved(ok)], none);
        frame_with(&mut app, &ctx, &mut t, vec![button(ok, egui::PointerButton::Primary, true, none)], none);
        frame_with(&mut app, &ctx, &mut t, vec![button(ok, egui::PointerButton::Primary, false, none)], none);
        assert_eq!(app.left_plane, Some(6.4), "{}", app.status_msg);
        assert_eq!(app.left_proj, ViewProjection::PlanXY);
        assert_eq!(app.status_msg, "X-Y Plane @ Z=6.4");
        assert!(!app.set_2d.open);
        // And the pane now draws the frames of that plane only (the 38 of the
        // second storey level of the model).
        let plane = PlaneView::of(app.left_proj, app.left_plane).unwrap();
        let members = view::plane_members(&app.model, plane, view::plane_tolerance(&app.model));
        assert_eq!(members.frames.len(), 38);
    }

    #[test]
    fn escape_returns_to_select_and_enter_is_routed_while_a_draw_tool_is_active() {
        // Poly Area acts on Enter: three picked points become an area. Enter that
        // was not routed to the tool would leave the model alone.
        let mut app = blank_app();
        app.model = SapModel::new(Units::KnMC);
        app.model.quick_grid([3, 3, 2], [6.0, 5.0, 3.2], [0.0; 3]).unwrap();
        one_pane(&mut app, ViewProjection::PlanXY, Some(0.0));
        app.tool_mode = ToolMode::PolyArea;
        let ctx = egui::Context::default();
        let mut t = 0.0;
        let none = egui::Modifiers::NONE;
        for _ in 0..3 {
            let _ = frame_with(&mut app, &ctx, &mut t, vec![], none);
        }
        let (v, _) = pane0(&app);
        for p in [[0.0, 0.0, 0.0], [6.0, 0.0, 0.0], [6.0, 5.0, 0.0]] {
            click_at_pos(&mut app, &ctx, &mut t, v.project(p));
        }
        assert!(app.model.areas.is_empty());
        press_key(&mut app, &ctx, &mut t, egui::Key::Enter);
        assert_eq!(app.model.areas.len(), 1, "Enter closes the polygon: {}", app.status_msg);
        assert_eq!(app.tool_mode, ToolMode::PolyArea, "Enter keeps the tool");
        // Escape ends a sequence in progress without drawing it, then returns to Select.
        for p in [[0.0, 0.0, 0.0], [12.0, 0.0, 0.0], [12.0, 10.0, 0.0]] {
            click_at_pos(&mut app, &ctx, &mut t, v.project(p));
        }
        press_key(&mut app, &ctx, &mut t, egui::Key::Escape);
        assert_eq!(app.tool_mode, ToolMode::Select);
        assert_eq!(app.model.areas.len(), 1, "the picked points were discarded, not drawn");
        press_key(&mut app, &ctx, &mut t, egui::Key::Enter);
        assert_eq!(app.model.areas.len(), 1, "Enter without a draw tool draws nothing");
    }

    #[test]
    fn a_draw_tool_window_is_routed_and_the_old_draw_tools_ignore_windows() {
        // A grid of 3 x 3 lines at z = 0 and no frames: a Quick Frame window over
        // all of it draws its 12 segments; the old tools do nothing with a window.
        let mut app = blank_app();
        app.model = SapModel::new(Units::KnMC);
        app.model.quick_grid([3, 3, 1], [6.0, 5.0, 3.2], [0.0; 3]).unwrap();
        one_pane(&mut app, ViewProjection::PlanXY, Some(0.0));
        let ctx = egui::Context::default();
        let mut t = 0.0;
        let none = egui::Modifiers::NONE;
        for _ in 0..3 {
            let _ = frame_with(&mut app, &ctx, &mut t, vec![], none);
        }
        let (v, _) = pane0(&app);
        let a = v.project([-1.0, -1.0, 0.0]);
        let b = v.project([13.0, 11.0, 0.0]);
        let before = app.pan;
        // The old tools: no frames, no selection, no pan.
        for tool in [ToolMode::DrawFrame, ToolMode::DrawArea] {
            app.tool_mode = tool;
            let _ = drag(&mut app, &ctx, &mut t, a, b, egui::PointerButton::Primary, none);
            assert_eq!(app.pan, before);
            assert!(selection::snapshot(&app.model).is_empty(), "{tool:?} must not select");
            assert!(app.model.frames.is_empty() && app.model.areas.is_empty(), "{tool:?}");
        }
        // Rectangle Area: a window is not a rectangle.
        app.tool_mode = ToolMode::RectArea;
        let _ = drag(&mut app, &ctx, &mut t, a, b, egui::PointerButton::Primary, none);
        assert!(selection::snapshot(&app.model).is_empty());
        assert!(app.model.frames.is_empty() && app.model.areas.is_empty());
        // Quick Frame: the window is routed to the tool and draws the segments
        // that lie inside it, and neither selects nor pans.
        app.tool_mode = ToolMode::QuickFrame;
        let _ = drag(&mut app, &ctx, &mut t, a, b, egui::PointerButton::Primary, none);
        assert_eq!(app.pan, before);
        assert!(selection::snapshot(&app.model).is_empty(), "a draw tool window must not select");
        assert_eq!(app.model.frames.len(), 12, "{}", app.status_msg);
        // Quick Area: four cells.
        app.tool_mode = ToolMode::QuickArea;
        let _ = drag(&mut app, &ctx, &mut t, a, b, egui::PointerButton::Primary, none);
        assert_eq!(app.model.areas.len(), 4, "{}", app.status_msg);
    }

    #[test]
    fn an_empty_model_with_a_grid_shows_its_bubbles_and_is_framed_by_the_grid() {
        let mut app = blank_app();
        app.model = SapModel::new(Units::KnMC);
        app.model.quick_grid([6, 4, 2], [6.0, 5.0, 3.2], [0.0; 3]).unwrap();
        let (lo, hi) = app.compute_bounds();
        assert_eq!((lo, hi), ([0.0, 0.0, 0.0], [30.0, 15.0, 3.2]));
        let ctx = egui::Context::default();
        for plane in [None, Some(0.0)] {
            one_pane(&mut app, ViewProjection::PlanXY, plane);
            let _ = view(&mut app, &ctx, 0.0);
            let out = view(&mut app, &ctx, 1.0);
            let (v, _) = pane0(&app);
            // Bubbles are drawn inside the viewport, to the right of the explorer.
            let drawn: Vec<String> = texts(&out)
                .into_iter()
                .filter(|(_, p)| p.x > 300.0)
                .map(|(t, _)| t)
                .collect();
            for id in ["A", "B", "C", "D", "E", "F", "1", "2", "3", "4"] {
                assert!(drawn.iter().any(|t| t == id), "bubble {id} missing for {plane:?}: {drawn:?}");
            }
            // The grid is centred in the pane, not in the old +-100 box.
            let c = v.project([15.0, 7.5, 0.0]);
            assert!((c - v.screen_center).length() < 1.0);
        }
    }

    // ------------------------------------------------------------ D34

    #[test]
    fn the_custom_grid_text_follows_the_units() {
        let mut app = blank_app();
        app.replace_model(SapModel::new(Units::KnMC), "x".into(), Vec::new());
        // The kip-in defaults 240 in and 144 in, in metres (1 in = 0.0254 m).
        assert!((app.tpl_span_or_bay_len - 6.096).abs() < 1e-9, "{}", app.tpl_span_or_bay_len);
        let bays = build::parse_lengths(&app.tpl_bays_text).unwrap();
        let storeys = build::parse_lengths(&app.tpl_storeys_text).unwrap();
        assert_eq!(bays.len(), 3);
        assert!(bays.iter().all(|b| (b - 6.096).abs() < 1e-9), "{bays:?}");
        assert!(storeys.iter().all(|s| (s - 3.6576).abs() < 1e-9), "{storeys:?}");
        assert_eq!(app.tpl_bays_text, "6.096, 6.096, 6.096");
    }

    #[test]
    fn the_plane_title_of_the_fourth_level_is_not_printed_with_roundoff() {
        let mut m = SapModel::new(Units::KnMC);
        m.quick_grid([2, 2, 7], [6.0, 5.0, 3.2], [0.0; 3]).unwrap();
        let levels = view::plane_levels(&m, view::PlaneAxis::Z);
        let title = PlaneView::of(ViewProjection::PlanXY, Some(levels[3])).unwrap().title();
        assert_eq!(title, "X-Y Plane @ Z=9.6");
        assert_eq!(view::number_text(0.1 + 0.2), "0.3");
        assert_eq!(view::number_text(-0.0), "0");
        assert_eq!(view::number_text(6.0), "6");
        assert_eq!(view::number_text(1234.5678), "1234.5678");
        assert_eq!(view::number_text(1.5e-9), "1.5e-9");
        assert_eq!(view::number_text(1.5e15), "1.5e15");
        assert_eq!(view::number_text(f64::INFINITY), "inf");
    }

    #[test]
    fn a_plane_view_and_picked_points_survive_a_unit_change() {
        let mut app = blank_app();
        app.model = part1_model();
        one_pane(&mut app, ViewProjection::PlanXY, Some(2.0 * 3.2));
        let count = |app: &SapGuiApp| {
            let plane = PlaneView::of(app.left_proj, app.left_plane).unwrap();
            (plane.title(), view::plane_members(&app.model, plane, view::plane_tolerance(&app.model)).frames.len())
        };
        assert_eq!(count(&app), ("X-Y Plane @ Z=6.4".to_string(), 38));
        // A Poly Frame point waits in this plane, and a typed coordinate in Set 2D View.
        app.tool_mode = ToolMode::PolyFrame;
        let ctx = egui::Context::default();
        let mut t = 0.0;
        let none = egui::Modifiers::NONE;
        let _ = frame_with(&mut app, &ctx, &mut t, vec![], none);
        let (v, _) = pane0(&app);
        let p = v.project([0.0, 0.0, 6.4]);
        for ev in [vec![egui::Event::PointerMoved(p)], vec![button(p, egui::PointerButton::Primary, true, none)], vec![button(p, egui::PointerButton::Primary, false, none)]] {
            let _ = frame_with(&mut app, &ctx, &mut t, ev, none);
        }
        assert!(app.draw_tools.picked_in().is_some(), "{}", app.status_msg);
        app.set_2d.coordinate_text = "6.4".into();
        app.change_units(Units::NMmC);
        // 6.4 m = 6400 mm; the same 38 frames lie in it, and nothing waits.
        assert_eq!(count(&app), ("X-Y Plane @ Z=6400".to_string(), 38));
        assert!(app.draw_tools.picked_in().is_none());
        assert!(app.set_2d.coordinate_text.is_empty());
        // And back.
        app.change_units(Units::KnMC);
        assert_eq!(count(&app), ("X-Y Plane @ Z=6.4".to_string(), 38));
        // A locked model keeps everything.
        app.model.is_locked = true;
        app.change_units(Units::NMmC);
        assert_eq!(app.model.present_units, Units::KnMC);
        assert_eq!(app.left_plane, Some(6.4));
    }

    fn poly_area_app() -> (SapGuiApp, egui::Context, f64) {
        let mut app = blank_app();
        app.model = SapModel::new(Units::KnMC);
        app.model.quick_grid([3, 3, 2], [6.0, 5.0, 3.2], [0.0; 3]).unwrap();
        app.dual_pane = true;
        app.left_proj = ViewProjection::Perspective3D;
        app.right_proj = ViewProjection::PlanXY;
        app.right_plane = Some(0.0);
        app.tool_mode = ToolMode::PolyArea;
        let ctx = egui::Context::default();
        let mut t = 0.0;
        for _ in 0..3 {
            let _ = frame_with(&mut app, &ctx, &mut t, vec![], egui::Modifiers::NONE);
        }
        (app, ctx, t)
    }

    fn click_at_pos(app: &mut SapGuiApp, ctx: &egui::Context, t: &mut f64, at: Pos2) {
        let none = egui::Modifiers::NONE;
        let _ = frame_with(app, ctx, t, vec![egui::Event::PointerMoved(at)], none);
        let _ = frame_with(app, ctx, t, vec![button(at, egui::PointerButton::Primary, true, none)], none);
        let _ = frame_with(app, ctx, t, vec![button(at, egui::PointerButton::Primary, false, none)], none);
        *t += 1.0;
        let _ = frame_with(app, ctx, t, vec![], none);
    }

    fn press_key(app: &mut SapGuiApp, ctx: &egui::Context, t: &mut f64, key: egui::Key) {
        let none = egui::Modifiers::NONE;
        let ev = egui::Event::Key { key, physical_key: None, pressed: true, repeat: false, modifiers: none };
        let _ = frame_with(app, ctx, t, vec![ev], none);
    }

    /// D34 (review item 5): Enter closes the polygon wherever the pointer is.
    #[test]
    fn enter_closes_the_polygon_in_the_pane_it_was_picked_in_wherever_the_pointer_is() {
        for move_to_other_pane in [false, true] {
            let (mut app, ctx, mut t) = poly_area_app();
            let (v, plane) = app.last_view[1].expect("right pane drawn");
            assert!(plane.is_some());
            for p in [[0.0, 0.0, 0.0], [6.0, 0.0, 0.0], [6.0, 5.0, 0.0]] {
                click_at_pos(&mut app, &ctx, &mut t, v.project(p));
            }
            if move_to_other_pane {
                let (v0, _) = app.last_view[0].expect("left pane drawn");
                let none = egui::Modifiers::NONE;
                let _ = frame_with(&mut app, &ctx, &mut t, vec![egui::Event::PointerMoved(v0.screen_center)], none);
                let _ = frame_with(&mut app, &ctx, &mut t, vec![], none);
                assert_eq!(app.active_pane, 0, "the pointer is over the 3D pane");
            }
            press_key(&mut app, &ctx, &mut t, egui::Key::Enter);
            assert_eq!(app.model.areas.len(), 1, "pointer moved to the other pane: {move_to_other_pane}: {}", app.status_msg);
            let area = app.model.areas.values().next().unwrap();
            assert_eq!(area.points.len(), 3);
        }
    }

    // ------------------------------------------------------------ D40

    /// Clicks the first text equal to `label` of the last frame; returns the
    /// output of the frame after the click.
    fn click_label(app: &mut SapGuiApp, ctx: &egui::Context, t: &mut f64, label: &str) -> egui::FullOutput {
        let none = egui::Modifiers::NONE;
        let out = frame_with(app, ctx, t, vec![], none);
        let at = texts(&out)
            .into_iter()
            .find(|(l, _)| l == label)
            .map(|(_, p)| p + Vec2::new(6.0, 6.0))
            .unwrap_or_else(|| panic!("no text '{label}'"));
        let _ = frame_with(app, ctx, t, vec![egui::Event::PointerMoved(at)], none);
        let _ = frame_with(app, ctx, t, vec![button(at, egui::PointerButton::Primary, true, none)], none);
        let _ = frame_with(app, ctx, t, vec![button(at, egui::PointerButton::Primary, false, none)], none);
        *t += 1.0;
        frame_with(app, ctx, t, vec![], none)
    }

    const ADD_SECTION: &str = "Add Custom Section (& Assign to Selected Frames)";

    /// A cantilever column of a library-like concrete (fc, no fy) in kip-in,
    /// a rebar material named A615 and a second steel; the section dialog open
    /// for a rectangular section, all frames selected.
    fn concrete_dialog_app() -> (SapGuiApp, egui::Context, f64) {
        let mut app = blank_app();
        app.model.generate_beam(2, 120.0, true, "FSEC1");
        let mut concrete = Material::default_concrete_kip_in();
        concrete.name = "C4".into();
        concrete.fy = 0.0;
        concrete.fys = 0.0;
        let mut rebar = Material::default_steel_kip_in();
        rebar.name = "A615".into();
        rebar.mat_type = MatType::Rebar;
        rebar.fy = 60.0;
        app.model.materials.insert("C4".into(), concrete);
        app.model.materials.insert("A615".into(), rebar);
        for f in app.model.frames.values_mut() {
            f.selected = true;
        }
        app.show_section_modal = true;
        app.custom_sec_shape = CustomSectionShape::Rectangular;
        app.custom_sec_material = "C4".into();
        app.custom_sec_name = "RC1".into();
        app.custom_sec_d = 24.0;
        app.custom_sec_b = 12.0;
        let ctx = egui::Context::default();
        let mut t = 0.0;
        for _ in 0..3 {
            let _ = frame_with(&mut app, &ctx, &mut t, vec![], egui::Modifiers::NONE);
        }
        (app, ctx, t)
    }

    #[test]
    fn the_rebar_materials_are_the_model_materials_with_a_yield_stress() {
        let (app, _, _) = concrete_dialog_app();
        // A992Fy50 (fy 50), the default concrete (fy 60) and A615; the library-like
        // concrete C4 has none.
        let names = rebar_materials(&app.model);
        assert!(names.contains(&"A615".to_string()) && !names.contains(&"C4".to_string()), "{names:?}");
        assert_eq!(default_rebar_material(&app.model, &names), "A615", "a rebar type comes first");
        assert!(rebar_material_error(&app.model, "A615").is_none());
        assert!(rebar_material_error(&app.model, "C4").unwrap().contains("no yield stress"));
        assert!(rebar_material_error(&app.model, "NOPE").unwrap().contains("existing rebar material"));
        let mut none = app.model.clone();
        none.materials.retain(|_, m| m.fy == 0.0);
        assert!(rebar_material_error(&none, "C4").unwrap().contains("material library"));
    }

    #[test]
    fn a_section_made_in_the_dialog_with_a_rebar_material_is_designed_and_one_without_is_refused() {
        // With: a library-like concrete (fy 0) and the rebar material chosen in the dialog.
        let (mut app, ctx, mut t) = concrete_dialog_app();
        let _ = click_label(&mut app, &ctx, &mut t, "Include Concrete Reinforcement Metadata");
        assert!(app.custom_sec_rebar_enabled);
        let out = frame_with(&mut app, &ctx, &mut t, vec![], egui::Modifiers::NONE);
        assert!(texts(&out).iter().any(|(l, _)| l.starts_with("Rebar Material")));
        // The combo opens on a click and offers the materials with a yield stress.
        let _ = click_label(&mut app, &ctx, &mut t, "A615");
        let out = frame_with(&mut app, &ctx, &mut t, vec![], egui::Modifiers::NONE);
        let shown: Vec<String> = texts(&out).into_iter().map(|(l, _)| l).collect();
        assert!(shown.iter().any(|l| l == "A992Fy50"), "{shown:?}");
        let _ = click_label(&mut app, &ctx, &mut t, "A992Fy50");
        assert_eq!(app.custom_sec_rebar_material, "A992Fy50");
        let _ = click_label(&mut app, &ctx, &mut t, "A992Fy50");
        let _ = click_label(&mut app, &ctx, &mut t, "A615");
        assert_eq!(app.custom_sec_rebar_material, "A615");
        let _ = click_label(&mut app, &ctx, &mut t, ADD_SECTION);
        assert!(app.status_msg.contains("Added section RC1") && app.status_msg.contains("rebar A615"), "{}", app.status_msg);
        let cfg = app.model.frame_sections["RC1"].concrete_rebar.clone().expect("reinforcement");
        assert_eq!((cfg.rebar_mat_long.as_str(), cfg.rebar_mat_confine.as_str()), ("A615", "A615"));
        assert!(app.model.frames.values().all(|f| f.section == "RC1"));
        app.execute_design(false);
        assert!(app.status_msg.starts_with("Concrete design"), "{}", app.status_msg);
        assert!(!app.model.concrete_design_results.is_empty());

        // Without reinforcement metadata the same section is refused at design,
        // naming the frame, the section and the concrete.
        let (mut app, ctx, mut t) = concrete_dialog_app();
        let _ = click_label(&mut app, &ctx, &mut t, ADD_SECTION);
        assert!(app.status_msg.contains("Added section RC1"), "{}", app.status_msg);
        assert!(app.model.frame_sections["RC1"].concrete_rebar.is_none());
        let frame = app.model.frames.keys().next().unwrap().clone();
        app.execute_design(false);
        let msg = app.status_msg.clone();
        assert!(msg.starts_with("Design Error"), "{msg}");
        assert!(msg.contains(&format!("frame '{frame}'")) && msg.contains("section 'RC1'") && msg.contains("material 'C4'"), "{msg}");
        assert!(app.model.concrete_design_results.is_empty());
    }

    #[test]
    fn the_dialog_says_when_the_model_has_no_rebar_material_and_refuses_the_section() {
        let (mut app, ctx, mut t) = concrete_dialog_app();
        app.model.materials.retain(|_, m| m.fy == 0.0);
        let _ = click_label(&mut app, &ctx, &mut t, "Include Concrete Reinforcement Metadata");
        let out = frame_with(&mut app, &ctx, &mut t, vec![], egui::Modifiers::NONE);
        let shown: Vec<String> = texts(&out).into_iter().map(|(l, _)| l).collect();
        assert!(shown.iter().any(|l| l.contains("no material with a yield stress") && l.contains("material library")), "{shown:?}");
        let _ = click_label(&mut app, &ctx, &mut t, ADD_SECTION);
        assert!(app.status_msg.contains("material library"), "{}", app.status_msg);
        assert!(!app.model.frame_sections.contains_key("RC1"));
    }

    /// Frames of the real window size (the dome spikes showed at 1440 x 900).
    fn window_frame(app: &mut SapGuiApp, ctx: &egui::Context, t: &mut f64) -> egui::FullOutput {
        *t += 0.05;
        let input = egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1440.0, 900.0))),
            time: Some(*t),
            ..Default::default()
        };
        ctx.run(input, |ctx| app.render_ui(ctx))
    }

    /// Tessellated vertices of everything in the central panel that is not text.
    fn central_vertices(ctx: &egui::Context, out: &egui::FullOutput) -> Vec<Pos2> {
        fn walk(s: &egui::epaint::Shape, clip: Rect, into: &mut Vec<egui::epaint::ClippedShape>) {
            match s {
                egui::epaint::Shape::Vec(v) => v.iter().for_each(|x| walk(x, clip, into)),
                // The pane's own frame and the labels are not what is looked for.
                egui::epaint::Shape::Text(_) | egui::epaint::Shape::Rect(_) => {}
                o => into.push(egui::epaint::ClippedShape { clip_rect: clip, shape: o.clone() }),
            }
        }
        let mut singles = Vec::new();
        for c in &out.shapes {
            if c.clip_rect.left() > 270.0 && c.clip_rect.top() > 60.0 {
                walk(&c.shape, c.clip_rect, &mut singles);
            }
        }
        ctx.tessellate(singles, 1.0)
            .iter()
            .filter_map(|p| match &p.primitive {
                egui::epaint::Primitive::Mesh(m) => Some(m.vertices.iter().map(|v| v.pos).collect::<Vec<_>>()),
                _ => None,
            })
            .flatten()
            .collect()
    }

    /// D41: the first mode of the Shell Dome template (3 x 8 divisions, R = 240
    /// in) shows its areas, deformed and not, inside the box of its joints. A
    /// ring quadrilateral seen edge-on, or a deformed sliver, used to be
    /// stroked with a mitre that ran across the whole view, from the apex.
    #[test]
    fn the_shell_dome_draws_no_spikes_in_the_undeformed_and_the_modal_views() {
        let mut app = blank_app();
        app.tpl_choice = TemplateChoice::SphericalDome;
        let mut m = SapModel::new(app.model.present_units);
        let (span, storey, bay) = (app.tpl_span_or_bay_len, app.tpl_story_height, app.tpl_bay_width_y);
        app.generate_template(&mut m, span, storey, bay);
        app.replace_model(m, "dome".into(), Vec::new());
        assert!(app.execute_analysis(), "{}", app.status_msg);
        app.dual_pane = false;
        let ctx = egui::Context::default();
        let mut t = 0.0;
        for deformed in [false, true] {
            if deformed {
                app.active_case = "MODAL".into();
                app.sync_active_case();
                app.display_mode = DisplayMode::Deformed;
            }
            for _ in 0..4 {
                let _ = window_frame(&mut app, &ctx, &mut t);
            }
            let out = window_frame(&mut app, &ctx, &mut t);
            let (v, _) = pane0(&app);
            let case = app.model.case_results.get(&app.active_case);
            let sel = contours::StepSelection { mode_idx: app.active_mode_idx, use_step: false, step_idx: 0 };
            let (mut lo, mut hi) = (Pos2::new(f32::MAX, f32::MAX), Pos2::new(f32::MIN, f32::MIN));
            for (pid, pt) in &app.model.points {
                let base = [pt.x, pt.y, pt.z];
                let shown = result_display::displaced_point(&app.model, case, pid, base, f64::from(app.deform_scale), sel);
                for q in [v.project(base), v.project(if deformed { shown } else { base })] {
                    lo = Pos2::new(lo.x.min(q.x), lo.y.min(q.y));
                    hi = Pos2::new(hi.x.max(q.x), hi.y.max(q.y));
                }
            }
            // Joint circles, supports and the like reach a few tens of pixels.
            let margin = 40.0;
            let vertices = central_vertices(&ctx, &out);
            assert!(vertices.len() > 500, "the dome is drawn: {}", vertices.len());
            let outside: Vec<&Pos2> = vertices
                .iter()
                .filter(|p| p.x < lo.x - margin || p.x > hi.x + margin || p.y < lo.y - margin || p.y > hi.y + margin)
                .collect();
            assert!(
                outside.is_empty(),
                "deformed {deformed}: {} vertices outside {lo:?}..{hi:?}, first {:?}",
                outside.len(),
                outside.first()
            );
        }
    }

    /// Follow-up of the windows package: a new or replaced model is shown fitted
    /// to its pane (zoom 1, no pan), the camera angle stays.
    #[test]
    fn a_replaced_model_is_fitted_to_its_pane() {
        let mut app = blank_app();
        app.zoom = 4.5;
        app.pan = Vec2::new(120.0, -80.0);
        app.yaw_rad = 0.3;
        app.replace_model(SapModel::new(Units::KnMC), "x".into(), Vec::new());
        assert_eq!((app.zoom, app.pan), (1.0, Vec2::ZERO));
        assert_eq!(app.yaw_rad, 0.3, "the angle of the camera is the user's");
        // The pane then shows the model inside itself: the dome of the template
        // has its whole bounding box inside the pane at zoom 1.
        let mut m = SapModel::new(Units::KipInF);
        m.generate_dome_shell(240.0, 45.0, 3, 8, true, "ASEC1");
        app.replace_model(m, "dome".into(), Vec::new());
        app.dual_pane = false;
        let ctx = egui::Context::default();
        let mut t = 0.0;
        for _ in 0..3 {
            let _ = window_frame(&mut app, &ctx, &mut t);
        }
        let (v, _) = pane0(&app);
        let (lo, hi) = app.compute_bounds();
        for z in [lo[2], hi[2]] {
            for (x, y) in [(lo[0], lo[1]), (hi[0], hi[1]), (lo[0], hi[1]), (hi[0], lo[1])] {
                let p = v.project([x, y, z]);
                assert!(p.x > 283.0 && p.x < 1433.0 && p.y > 81.0 && p.y < 870.0, "{p:?} is outside the pane");
            }
        }
    }
}
