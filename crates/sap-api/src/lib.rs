//! Selected typed Rust facades following the SAP2000 v19 object hierarchy.
//! This crate is not a COM server or a complete implementation of the vendor API.
//!
//! All mutating and query methods follow the CSI OAPI convention of returning `0` (`OAPI_OK`)
//! on success and non-zero on error.

#![allow(non_camel_case_types)]

mod definitions;
mod links;
pub use definitions::{
    cFunctions, set_case_definition, set_function_definition, validate_case_definition,
    DefinitionError,
};
pub use links::{cLinkObj, cPropLink, set_link_object, set_link_property};

use sap_design::{try_run_concrete_design, try_run_steel_design, ConcreteCode, SteelCode};
use sap_format::{
    load_model_with_report, save_native_sdb_file, save_s2k_file_with_report,
    save_sdb_file_with_report,
};
use sap_model::{
    AreaLoadSurfacePressure, AreaLoadUniform, AreaSection, CableParams, ComboType,
    ConcreteDesignResult, EditError, EditSummary, FrameLoadDistributed, FrameLoadPoint,
    FrameLoadTemperature, FramePropType, FrameSection, FrameStationResult, FrameType2D,
    FrameType3D, LoadCase, LoadCaseAssignment, LoadCaseType, LoadCombo, LoadPattern,
    LoadPatternType, MatType, ModalModeResult, NLGeomType, PointLoadDispl, PointLoadForce,
    SapModel, ShellType, SolidObj, SolidResult, SolidSection, SolidSurfaceLoad, SteelDesignResult,
};
use sap_section::{
    make_box_tube, make_circle, make_i_section, make_pipe, make_rectangular, ProDatabase,
};
use sap_solver::analyze_model;
use sap_units::Units;
use std::path::{Path, PathBuf};

pub const OAPI_OK: i32 = 0;
pub const OAPI_ERR: i32 = 1;

/// Root OAPI controller (`SAP2000v19.cOAPI` / `SapObject`).
#[derive(Debug, Clone)]
pub struct SapObject {
    pub sap_model: cSapModel,
    pub visible: bool,
    pub current_file: Option<PathBuf>,
}

impl Default for SapObject {
    fn default() -> Self {
        Self::new()
    }
}

impl SapObject {
    pub fn new() -> Self {
        Self {
            sap_model: cSapModel::new(Units::KipInF),
            visible: true,
            current_file: None,
        }
    }

    /// Starts the SAP2000 application engine (`cOAPI.ApplicationStart`).
    pub fn application_start(&mut self, units: Units, visible: bool, file_name: &str) -> i32 {
        self.visible = visible;
        self.current_file = None;
        self.sap_model.initialize_new_model(units);
        if !file_name.trim().is_empty() {
            let result = self.sap_model.file().open_file(file_name);
            if result == OAPI_OK {
                self.current_file = self.sap_model.last_file_path.clone();
            }
            return result;
        }
        OAPI_OK
    }

    /// Exits the application, optionally saving the active model (`cOAPI.ApplicationExit`).
    pub fn application_exit(&mut self, file_save: bool) -> i32 {
        if file_save {
            // cFile owns the active destination, including Save As after startup.
            let Some(path) = self.sap_model.last_file_path.clone() else {
                return OAPI_ERR;
            };
            let Some(path) = path.to_str() else {
                return OAPI_ERR;
            };
            return self.sap_model.file().save(path);
        }
        OAPI_OK
    }

    /// Returns the OAPI version number (`19.0`).
    pub fn get_oapi_version_number(&self) -> f64 {
        19.0
    }
}

/// Root Structural Model API (`SAP2000v19.cSapModel`).
#[derive(Debug, Clone)]
pub struct cSapModel {
    pub inner: SapModel,
    pub selected_cases: Vec<String>,
    pub selected_combos: Vec<String>,
    pub pro_search_dir: PathBuf,
    pub last_file_path: Option<PathBuf>,
    pub import_warnings: Vec<String>,
    pub export_warnings: Vec<String>,
    pub last_file_error: Option<String>,
}

impl Default for cSapModel {
    fn default() -> Self {
        Self::new(Units::KipInF)
    }
}

impl cSapModel {
    pub fn new(units: Units) -> Self {
        Self {
            inner: SapModel::new(units),
            selected_cases: vec!["DEAD".to_string()],
            selected_combos: Vec::new(),
            pro_search_dir: std::env::var_os("STRUCTURAX_SECTION_DIR")
                .map(PathBuf::from)
                .unwrap_or_else(|| PathBuf::from("sections")),
            last_file_path: None,
            import_warnings: Vec::new(),
            export_warnings: Vec::new(),
            last_file_error: None,
        }
    }

    pub fn initialize_new_model(&mut self, units: Units) -> i32 {
        self.inner = SapModel::new(units);
        self.selected_cases = vec!["DEAD".to_string()];
        self.selected_combos.clear();
        self.last_file_path = None;
        self.import_warnings.clear();
        self.export_warnings.clear();
        self.last_file_error = None;
        OAPI_OK
    }

    fn prepare_edit(&mut self) -> bool {
        if self.inner.is_locked {
            return false;
        }
        self.inner.case_results.clear();
        self.inner.steel_design_results.clear();
        self.inner.concrete_design_results.clear();
        true
    }

    pub fn set_present_units(&mut self, units: Units) -> i32 {
        self.inner.convert_units_in_place(units);
        OAPI_OK
    }

    pub fn get_present_units(&self) -> Units {
        self.inner.present_units
    }

    pub fn set_model_is_locked(&mut self, locked: bool) -> i32 {
        if !locked {
            self.inner.case_results.clear();
            self.inner.steel_design_results.clear();
            self.inner.concrete_design_results.clear();
        }
        self.inner.is_locked = locked;
        OAPI_OK
    }

    pub fn get_model_is_locked(&self) -> bool {
        self.inner.is_locked
    }

    pub fn get_version(&self) -> (&'static str, f64) {
        ("19.0.0", 19.0)
    }

    pub fn edit(&mut self) -> cEdit<'_> {
        cEdit { api: self }
    }

    pub fn file(&mut self) -> cFile<'_> {
        cFile { api: self }
    }

    pub fn point_obj(&mut self) -> cPointObj<'_> {
        cPointObj { api: self }
    }

    pub fn frame_obj(&mut self) -> cFrameObj<'_> {
        cFrameObj { api: self }
    }

    pub fn area_obj(&mut self) -> cAreaObj<'_> {
        cAreaObj { api: self }
    }

    pub fn solid_obj(&mut self) -> cSolidObj<'_> {
        cSolidObj { api: self }
    }

    pub fn prop_link(&mut self) -> cPropLink<'_> {
        cPropLink { api: self }
    }
    pub fn link_obj(&mut self) -> cLinkObj<'_> {
        cLinkObj { api: self }
    }

    pub fn prop_material(&mut self) -> cPropMaterial<'_> {
        cPropMaterial { api: self }
    }

    pub fn prop_frame(&mut self) -> cPropFrame<'_> {
        cPropFrame { api: self }
    }

    pub fn prop_area(&mut self) -> cPropArea<'_> {
        cPropArea { api: self }
    }

    pub fn prop_solid(&mut self) -> cPropSolid<'_> {
        cPropSolid { api: self }
    }

    pub fn load_patterns(&mut self) -> cLoadPatterns<'_> {
        cLoadPatterns { api: self }
    }

    pub fn load_cases(&mut self) -> cLoadCases<'_> {
        cLoadCases { api: self }
    }

    pub fn functions(&mut self) -> cFunctions<'_> {
        cFunctions { api: self }
    }

    pub fn resp_combo(&mut self) -> cRespCombo<'_> {
        cRespCombo { api: self }
    }

    pub fn analyze(&mut self) -> cAnalyze<'_> {
        cAnalyze { api: self }
    }

    pub fn results(&mut self) -> cAnalysisResults<'_> {
        cAnalysisResults { api: self }
    }

    pub fn design_steel(&mut self) -> cDesignSteel<'_> {
        cDesignSteel { api: self }
    }

    pub fn design_concrete(&mut self) -> cDesignConcrete<'_> {
        cDesignConcrete { api: self }
    }
}

/// Native transactional editing facade. Typed errors preserve the rejection reason;
/// these signatures are not a claim of Windows COM compatibility.
pub struct cEdit<'a> {
    api: &'a mut cSapModel,
}

impl cEdit<'_> {
    pub fn move_selected(&mut self, delta: [f64; 3]) -> Result<EditSummary, EditError> {
        self.api.inner.move_selected(delta)
    }
    pub fn replicate_selected(
        &mut self,
        delta: [f64; 3],
        copies: usize,
    ) -> Result<EditSummary, EditError> {
        self.api.inner.replicate_selected(delta, copies)
    }
    pub fn divide_selected_frames(&mut self, segments: usize) -> Result<EditSummary, EditError> {
        self.api.inner.divide_selected_frames(segments)
    }
}

/// File I/O and Parametric Template Controller (`SAP2000v19.cFile`).
pub struct cFile<'a> {
    api: &'a mut cSapModel,
}

impl cFile<'_> {
    pub fn new_blank(&mut self) -> i32 {
        let u = self.api.inner.present_units;
        self.api.initialize_new_model(u)
    }

    pub fn open_file(&mut self, file_name: &str) -> i32 {
        let path = Path::new(file_name);
        let res = load_model_with_report(path);
        match res {
            Ok(report) => {
                self.api.import_warnings = report.warnings;
                self.api.export_warnings.clear();
                self.api.last_file_error = None;
                let model = report.model;
                self.api.selected_cases = model.load_cases.keys().cloned().collect();
                self.api.selected_combos.clear();
                self.api.inner = model;
                self.api.last_file_path = Some(path.to_path_buf());
                OAPI_OK
            }
            Err(error) => {
                self.api.last_file_error = Some(error.to_string());
                OAPI_ERR
            }
        }
    }

    pub fn save(&mut self, file_name: &str) -> i32 {
        let path = if file_name.trim().is_empty() {
            match &self.api.last_file_path {
                Some(p) => p.clone(),
                None => PathBuf::from("model.s2k"),
            }
        } else {
            PathBuf::from(file_name)
        };
        let ext = path
            .extension()
            .and_then(|e| e.to_str())
            .unwrap_or("")
            .to_ascii_lowercase();
        self.api.export_warnings.clear();
        let res = if ext == "sdb" {
            save_sdb_file_with_report(&self.api.inner, &path)
        } else {
            save_s2k_file_with_report(&self.api.inner, &path)
        };
        match res {
            Ok(warnings) => {
                self.api.export_warnings = warnings;
                self.api.last_file_error = None;
                self.api.last_file_path = Some(path);
                OAPI_OK
            }
            Err(error) => {
                self.api.last_file_error = Some(error.to_string());
                OAPI_ERR
            }
        }
    }

    /// Exports the supported model subset as an experimental native `cBinaryStream` file.
    /// Unsupported/lossy state rejects before writing; vendor runtime acceptance is unverified.
    pub fn save_native_sdb(&mut self, file_name: &str) -> i32 {
        if file_name.trim().is_empty() {
            self.api.last_file_error = Some("Empty native SDB destination path".to_string());
            return OAPI_ERR;
        }
        let path = PathBuf::from(file_name);
        self.api.export_warnings.clear();
        match save_native_sdb_file(&self.api.inner, &path) {
            Ok(()) => {
                self.api.export_warnings.push(
                    "Experimental native SDB export: vendor application acceptance is unverified; retain the original source.".into(),
                );
                self.api.last_file_error = None;
                self.api.last_file_path = Some(path);
                OAPI_OK
            }
            Err(error) => {
                self.api.last_file_error = Some(error.to_string());
                OAPI_ERR
            }
        }
    }

    pub fn new_beam(
        &mut self,
        num_spans: usize,
        span_length: f64,
        restraint: bool,
        beam: &str,
    ) -> i32 {
        self.api
            .inner
            .generate_beam(num_spans, span_length, restraint, beam);
        OAPI_OK
    }

    #[allow(clippy::too_many_arguments)]
    pub fn new_2d_frame(
        &mut self,
        temp_type: FrameType2D,
        num_stories: usize,
        story_height: f64,
        num_bays: usize,
        bay_width: f64,
        restraint: bool,
        beam: &str,
        column: &str,
        brace: &str,
    ) -> i32 {
        self.api.inner.generate_2d_frame(
            temp_type,
            num_stories,
            story_height,
            num_bays,
            bay_width,
            restraint,
            beam,
            column,
            brace,
            0.0,
        );
        OAPI_OK
    }

    pub fn new_2d_truss(
        &mut self,
        num_divs: usize,
        div_len: f64,
        height: f64,
        restraint: bool,
        chord: &str,
        web: &str,
    ) -> i32 {
        self.api
            .inner
            .generate_2d_truss(num_divs, div_len, height, restraint, chord, web);
        OAPI_OK
    }

    #[allow(clippy::too_many_arguments)]
    pub fn new_3d_frame(
        &mut self,
        temp_type: FrameType3D,
        num_stories: usize,
        story_height: f64,
        num_bays_x: usize,
        bay_width_x: f64,
        num_bays_y: usize,
        bay_width_y: f64,
        restraint: bool,
        beam: &str,
        column: &str,
        area_sec: &str,
    ) -> i32 {
        self.api.inner.generate_3d_frame(
            temp_type,
            num_stories,
            story_height,
            num_bays_x,
            bay_width_x,
            num_bays_y,
            bay_width_y,
            restraint,
            beam,
            column,
            area_sec,
        );
        OAPI_OK
    }

    pub fn new_wall(
        &mut self,
        num_x: usize,
        width_x: f64,
        num_z: usize,
        height_z: f64,
        restraint: bool,
        area_sec: &str,
    ) -> i32 {
        self.api
            .inner
            .generate_wall(num_x, width_x, num_z, height_z, restraint, area_sec);
        OAPI_OK
    }
}

/// Point / Joint Object API (`SAP2000v19.cPointObj`).
pub struct cPointObj<'a> {
    api: &'a mut cSapModel,
}

impl cPointObj<'_> {
    pub fn add_cartesian(&mut self, x: f64, y: f64, z: f64, user_name: &str) -> (i32, String) {
        if self.api.inner.is_locked
            || [x, y, z].iter().any(|v| !v.is_finite())
            || (!user_name.trim().is_empty() && self.api.inner.points.contains_key(user_name))
        {
            return (OAPI_ERR, String::new());
        }
        let name = if user_name.trim().is_empty() {
            sap_model::next_numeric_name(self.api.inner.points.keys())
        } else {
            user_name.to_owned()
        };
        self.api.inner.add_point(&name, x, y, z);
        self.api.inner.invalidate_results();
        (OAPI_OK, name)
    }

    fn edit_point(&mut self, name: &str, edit: impl FnOnce(&mut sap_model::PointObj)) -> i32 {
        if self.api.inner.is_locked {
            return OAPI_ERR;
        }
        let Some(point) = self.api.inner.points.get_mut(name) else {
            return OAPI_ERR;
        };
        edit(point);
        self.api.inner.invalidate_results();
        OAPI_OK
    }

    pub fn set_restraint(&mut self, name: &str, value: [bool; 6]) -> i32 {
        self.edit_point(name, |p| p.restraint = value)
    }
    pub fn get_restraint(&self, name: &str) -> Option<[bool; 6]> {
        self.api.inner.points.get(name).map(|p| p.restraint)
    }
    pub fn set_spring(&mut self, name: &str, k: [f64; 6]) -> i32 {
        if k.iter().any(|v| !v.is_finite() || *v < 0.) {
            return OAPI_ERR;
        }
        self.edit_point(name, |p| p.spring = k)
    }
    pub fn set_load_force(
        &mut self,
        name: &str,
        load_pat: &str,
        f: [f64; 6],
        replace: bool,
    ) -> i32 {
        if !self.api.inner.load_patterns.contains_key(load_pat) || f.iter().any(|v| !v.is_finite())
        {
            return OAPI_ERR;
        }
        self.edit_point(name, |p| {
            if replace {
                p.loads_force.retain(|l| l.load_pat != load_pat);
            }
            p.loads_force.push(PointLoadForce {
                load_pat: load_pat.into(),
                csys: "GLOBAL".into(),
                force: f,
            });
        })
    }
    pub fn set_load_displ(
        &mut self,
        name: &str,
        load_pat: &str,
        u: [f64; 6],
        replace: bool,
    ) -> i32 {
        if !self.api.inner.load_patterns.contains_key(load_pat) || u.iter().any(|v| !v.is_finite())
        {
            return OAPI_ERR;
        }
        self.edit_point(name, |p| {
            if replace {
                p.loads_displ.retain(|l| l.load_pat != load_pat);
            }
            p.loads_displ.push(PointLoadDispl {
                load_pat: load_pat.into(),
                csys: "GLOBAL".into(),
                displ: u,
            });
        })
    }
    pub fn set_mass(&mut self, name: &str, m: [f64; 6], replace: bool) -> i32 {
        let Some(point) = self.api.inner.points.get(name) else {
            return OAPI_ERR;
        };
        if m.iter().any(|v| !v.is_finite() || *v < 0.) {
            return OAPI_ERR;
        }
        let result = if replace {
            m
        } else {
            std::array::from_fn(|k| point.mass[k] + m[k])
        };
        if result.iter().any(|v| !v.is_finite() || *v < 0.) {
            return OAPI_ERR;
        }
        self.edit_point(name, |p| p.mass = result)
    }
    pub fn count(&self) -> usize {
        self.api.inner.points.len()
    }
}

/// Frame / Cable Object API (`SAP2000v19.cFrameObj`).
pub struct cFrameObj<'a> {
    api: &'a mut cSapModel,
}

impl cFrameObj<'_> {
    fn edit_frame(&mut self, name: &str, edit: impl FnOnce(&mut sap_model::FrameObj)) -> i32 {
        if self.api.inner.is_locked {
            return OAPI_ERR;
        }
        let Some(frame) = self.api.inner.frames.get_mut(name) else {
            return OAPI_ERR;
        };
        edit(frame);
        self.api.inner.invalidate_results();
        OAPI_OK
    }
    fn valid_load(&self, load_pat: &str, my_type: i32, dir: i32, csys: &str) -> bool {
        let valid_system = csys.eq_ignore_ascii_case("GLOBAL")
            || csys.eq_ignore_ascii_case("LOCAL")
            || self
                .api
                .inner
                .coord_systems
                .get(csys)
                .is_some_and(|system| {
                    system
                        .origin
                        .iter()
                        .chain(&system.angles_deg)
                        .all(|v| v.is_finite())
                });
        self.api.inner.load_patterns.contains_key(load_pat)
            && (1..=2).contains(&my_type)
            && (1..=11).contains(&dir)
            && valid_system
    }
    pub fn add_by_point(
        &mut self,
        point1: &str,
        point2: &str,
        prop_name: &str,
        user_name: &str,
    ) -> (i32, String) {
        let sec = if prop_name.trim().is_empty() {
            "FSEC1"
        } else {
            prop_name
        };
        let Some(i) = self.api.inner.points.get(point1) else {
            return (OAPI_ERR, String::new());
        };
        let Some(j) = self.api.inner.points.get(point2) else {
            return (OAPI_ERR, String::new());
        };
        let length = (j.x - i.x).hypot(j.y - i.y).hypot(j.z - i.z);
        if self.api.inner.is_locked
            || !length.is_finite()
            || length <= 0.
            || !self.api.inner.frame_sections.contains_key(sec)
            || (!user_name.trim().is_empty() && self.api.inner.frames.contains_key(user_name))
        {
            return (OAPI_ERR, String::new());
        }
        let name = if user_name.trim().is_empty() {
            sap_model::next_numeric_name(self.api.inner.frames.keys())
        } else {
            user_name.to_owned()
        };
        self.api.inner.add_frame(&name, point1, point2, sec);
        self.api.inner.invalidate_results();
        (OAPI_OK, name)
    }
    #[allow(clippy::too_many_arguments)]
    pub fn add_by_coord(
        &mut self,
        xi: f64,
        yi: f64,
        zi: f64,
        xj: f64,
        yj: f64,
        zj: f64,
        prop_name: &str,
        user_name: &str,
    ) -> (i32, String) {
        let sec = if prop_name.trim().is_empty() {
            "FSEC1"
        } else {
            prop_name
        };
        let length = (xj - xi).hypot(yj - yi).hypot(zj - zi);
        if self.api.inner.is_locked
            || [xi, yi, zi, xj, yj, zj].iter().any(|v| !v.is_finite())
            || !length.is_finite()
            || length <= 0.
            || !self.api.inner.frame_sections.contains_key(sec)
            || (!user_name.trim().is_empty() && self.api.inner.frames.contains_key(user_name))
        {
            return (OAPI_ERR, String::new());
        }
        if [[xi, yi, zi], [xj, yj, zj]].iter().any(|xyz| {
            self.api
                .inner
                .points
                .values()
                .filter(|p| [p.x, p.y, p.z] == *xyz)
                .take(2)
                .count()
                > 1
        }) {
            return (OAPI_ERR, String::new());
        }
        // Validate everything before creating either endpoint. Only exact coordinate
        // matches merge; a fixed unit-independent epsilon can collapse short members.
        let p1 = self.find_or_create_point(xi, yi, zi);
        let p2 = self.find_or_create_point(xj, yj, zj);
        self.add_by_point(&p1, &p2, sec, user_name)
    }
    fn find_or_create_point(&mut self, x: f64, y: f64, z: f64) -> String {
        if let Some((id, _)) = self
            .api
            .inner
            .points
            .iter()
            .find(|(_, p)| [p.x, p.y, p.z] == [x, y, z])
        {
            return id.clone();
        }
        let id = sap_model::next_numeric_name(self.api.inner.points.keys());
        self.api.inner.add_point(&id, x, y, z);
        id
    }
    pub fn set_section(&mut self, name: &str, prop_name: &str) -> i32 {
        if !self.api.inner.frame_sections.contains_key(prop_name) {
            return OAPI_ERR;
        }
        self.edit_frame(name, |f| f.section = prop_name.into())
    }
    pub fn set_releases(
        &mut self,
        name: &str,
        ii: [bool; 6],
        jj: [bool; 6],
        start_val: [f64; 6],
        end_val: [f64; 6],
    ) -> i32 {
        if start_val
            .iter()
            .chain(&end_val)
            .any(|v| !v.is_finite() || *v < 0.)
            || (0..6).any(|k| (!ii[k] && start_val[k] != 0.) || (!jj[k] && end_val[k] != 0.))
        {
            return OAPI_ERR;
        }
        self.edit_frame(name, |f| {
            f.release_i = ii;
            f.release_j = jj;
            f.start_spring = start_val;
            f.end_spring = end_val;
        })
    }
    #[allow(clippy::too_many_arguments)]
    pub fn set_load_distributed(
        &mut self,
        name: &str,
        load_pat: &str,
        my_type: i32,
        dir: i32,
        dist1: f64,
        dist2: f64,
        val1: f64,
        val2: f64,
        csys: &str,
        replace: bool,
    ) -> i32 {
        if !self.valid_load(load_pat, my_type, dir, csys)
            || [dist1, dist2, val1, val2].iter().any(|v| !v.is_finite())
            || !(0.0..=1.).contains(&dist1)
            || !(0.0..=1.).contains(&dist2)
            || dist1 >= dist2
        {
            return OAPI_ERR;
        }
        self.edit_frame(name, |f| {
            if replace {
                f.loads_dist.retain(|l| l.load_pat != load_pat);
            }
            f.loads_dist.push(FrameLoadDistributed {
                load_pat: load_pat.into(),
                my_type,
                dir,
                csys: csys.into(),
                rel_dist1: dist1,
                rel_dist2: dist2,
                val1,
                val2,
            });
        })
    }
    #[allow(clippy::too_many_arguments)]
    pub fn set_load_point(
        &mut self,
        name: &str,
        load_pat: &str,
        my_type: i32,
        dir: i32,
        dist: f64,
        val: f64,
        csys: &str,
        replace: bool,
    ) -> i32 {
        if !self.valid_load(load_pat, my_type, dir, csys)
            || !dist.is_finite()
            || !val.is_finite()
            || !(0.0..=1.).contains(&dist)
        {
            return OAPI_ERR;
        }
        self.edit_frame(name, |f| {
            if replace {
                f.loads_point.retain(|l| l.load_pat != load_pat);
            }
            f.loads_point.push(FrameLoadPoint {
                load_pat: load_pat.into(),
                my_type,
                dir,
                csys: csys.into(),
                rel_dist: dist,
                val,
            });
        })
    }
    pub fn set_load_temperature(
        &mut self,
        name: &str,
        load_pat: &str,
        my_type: i32,
        val: f64,
        pat_name: Option<String>,
        replace: bool,
    ) -> i32 {
        let Some(frame) = self.api.inner.frames.get(name) else {
            return OAPI_ERR;
        };
        if !self.api.inner.load_patterns.contains_key(load_pat)
            || !val.is_finite()
            || !(1..=3).contains(&my_type)
            || pat_name.as_ref().is_some_and(|pattern| {
                pattern.is_empty()
                    || [&frame.point_i, &frame.point_j].into_iter().any(|id| {
                        self.api
                            .inner
                            .points
                            .get(id)
                            .and_then(|p| p.pattern_values.get(pattern))
                            .is_none_or(|v| !v.is_finite())
                    })
            })
        {
            return OAPI_ERR;
        }
        self.edit_frame(name, |f| {
            if replace {
                f.loads_temp.retain(|l| l.load_pat != load_pat);
            }
            f.loads_temp.push(FrameLoadTemperature {
                load_pat: load_pat.into(),
                my_type,
                val,
                pattern_name: pat_name,
            });
        })
    }
    pub fn set_end_length_offset(
        &mut self,
        name: &str,
        length1: f64,
        length2: f64,
        rz: f64,
    ) -> i32 {
        let Some(frame) = self.api.inner.frames.get(name) else {
            return OAPI_ERR;
        };
        let Some(i) = self.api.inner.points.get(&frame.point_i) else {
            return OAPI_ERR;
        };
        let Some(j) = self.api.inner.points.get(&frame.point_j) else {
            return OAPI_ERR;
        };
        let length = (j.x - i.x).hypot(j.y - i.y).hypot(j.z - i.z);
        if [length1, length2, rz, length]
            .iter()
            .any(|v| !v.is_finite())
            || length1 < 0.
            || length2 < 0.
            || !(0.0..=1.).contains(&rz)
            || length1 + length2 >= length
        {
            return OAPI_ERR;
        }
        self.edit_frame(name, |f| {
            f.end_offset_i = length1;
            f.end_offset_j = length2;
            f.rigid_zone_factor = rz;
        })
    }
    pub fn set_local_axes(&mut self, name: &str, ang: f64) -> i32 {
        if !ang.is_finite() {
            return OAPI_ERR;
        }
        self.edit_frame(name, |f| f.angle_deg = ang)
    }
    pub fn set_modifiers(&mut self, name: &str, modifiers: [f64; 8]) -> i32 {
        if modifiers.iter().any(|v| !v.is_finite() || *v < 0.) {
            return OAPI_ERR;
        }
        self.edit_frame(name, |f| f.modifiers = modifiers)
    }
    pub fn set_tc_limits(
        &mut self,
        name: &str,
        limit_compression_exists: bool,
        limit_compression: f64,
        limit_tension_exists: bool,
        limit_tension: f64,
    ) -> i32 {
        if !limit_compression.is_finite()
            || !limit_tension.is_finite()
            || (limit_compression_exists && limit_compression > 0.0)
            || (limit_tension_exists && limit_tension < 0.0)
        {
            return OAPI_ERR;
        }
        self.edit_frame(name, |f| {
            if limit_compression_exists || limit_tension_exists {
                f.tc_limits = Some((
                    limit_compression_exists,
                    limit_compression,
                    limit_tension_exists,
                    limit_tension,
                ));
            } else {
                f.tc_limits = None;
            }
        })
    }
    pub fn set_cable_data(
        &mut self,
        name: &str,
        undeformed_length: Option<f64>,
        target_tension_i: Option<f64>,
        horizontal_tension: Option<f64>,
        num_segments: usize,
    ) -> i32 {
        if num_segments == 0
            || undeformed_length.is_some_and(|v| !v.is_finite() || v <= 0.0)
            || target_tension_i.is_some_and(|v| !v.is_finite() || v <= 0.0)
            || horizontal_tension.is_some_and(|v| !v.is_finite() || v <= 0.0)
        {
            return OAPI_ERR;
        }
        self.edit_frame(name, |f| {
            f.cable_params = Some(CableParams {
                undeformed_length,
                target_tension_i,
                horizontal_tension,
                num_segments,
            });
        })
    }
    pub fn count(&self) -> usize {
        self.api.inner.frames.len()
    }
}

/// 3D Solid Object API (`SAP2000v19.cSolidObj`).
pub struct cSolidObj<'a> {
    api: &'a mut cSapModel,
}

impl cSolidObj<'_> {
    pub fn add_by_point(
        &mut self,
        points: [String; 8],
        prop_name: &str,
        user_name: &str,
    ) -> (i32, String) {
        let sec = if prop_name.trim().is_empty() {
            "SSEC1"
        } else {
            prop_name
        };
        if self.api.inner.is_locked
            || !self.api.inner.solid_sections.contains_key(sec)
            || points
                .iter()
                .any(|p| !self.api.inner.points.contains_key(p))
            || (!user_name.trim().is_empty() && self.api.inner.solids.contains_key(user_name))
        {
            return (OAPI_ERR, String::new());
        }
        let name = if user_name.trim().is_empty() {
            sap_model::next_numeric_name(self.api.inner.solids.keys())
        } else {
            user_name.to_owned()
        };
        // Use the solver's geometric contract before inserting or invalidating results.
        // Collapsed wedge/tetrahedral mappings remain allowed only when their
        // center and integration-point Jacobians satisfy that same contract.
        let coords = points.each_ref().map(|id| {
            let p = &self.api.inner.points[id];
            [p.x, p.y, p.z]
        });
        if sap_solver::solid::validate_solid_geometry(&coords).is_err() {
            return (OAPI_ERR, String::new());
        }
        self.api.inner.solids.insert(
            name.clone(),
            SolidObj {
                name: name.clone(),
                points,
                section: sec.to_owned(),
                loads_surface: Vec::new(),
                loads_temp: std::collections::BTreeMap::new(),
                selected: false,
            },
        );
        self.api.inner.invalidate_results();
        (OAPI_OK, name)
    }

    pub fn set_load_surface_pressure(
        &mut self,
        name: &str,
        load_pat: &str,
        face: i32,
        val: f64,
        replace: bool,
    ) -> i32 {
        if self.api.inner.is_locked
            || !self.api.inner.solids.contains_key(name)
            || !self.api.inner.load_patterns.contains_key(load_pat)
            || !(1..=6).contains(&face)
            || !val.is_finite()
        {
            return OAPI_ERR;
        }
        let solid = self
            .api
            .inner
            .solids
            .get_mut(name)
            .expect("validated solid");
        if replace {
            solid
                .loads_surface
                .retain(|l| l.load_pat != load_pat || l.face != face);
        }
        solid.loads_surface.push(SolidSurfaceLoad {
            load_pat: load_pat.to_owned(),
            face,
            val,
        });
        self.api.inner.invalidate_results();
        OAPI_OK
    }

    pub fn set_load_temperature(
        &mut self,
        name: &str,
        load_pat: &str,
        val: f64,
        replace: bool,
    ) -> i32 {
        if self.api.inner.is_locked
            || !self.api.inner.solids.contains_key(name)
            || !self.api.inner.load_patterns.contains_key(load_pat)
            || !val.is_finite()
        {
            return OAPI_ERR;
        }
        let solid = self
            .api
            .inner
            .solids
            .get_mut(name)
            .expect("validated solid");
        let prospective = if replace {
            val
        } else {
            solid.loads_temp.get(load_pat).copied().unwrap_or(0.0) + val
        };
        if !prospective.is_finite() {
            return OAPI_ERR;
        }
        solid.loads_temp.insert(load_pat.to_owned(), prospective);
        self.api.inner.invalidate_results();
        OAPI_OK
    }

    pub fn count(&self) -> usize {
        self.api.inner.solids.len()
    }
}

/// 3D Solid Property API (`SAP2000v19.cPropSolid`).
pub struct cPropSolid<'a> {
    api: &'a mut cSapModel,
}

impl cPropSolid<'_> {
    pub fn set_prop(
        &mut self,
        name: &str,
        mat_prop: &str,
        a: f64,
        b: f64,
        c: f64,
        incompatible: bool,
    ) -> i32 {
        if self.api.inner.is_locked
            || name.trim().is_empty()
            || !self.api.inner.materials.contains_key(mat_prop)
            || [a, b, c].iter().any(|v| !v.is_finite())
        {
            return OAPI_ERR;
        }
        self.api.inner.solid_sections.insert(
            name.to_owned(),
            SolidSection {
                name: name.to_owned(),
                material: mat_prop.to_owned(),
                mat_angle_deg: [a, b, c],
                incompatible_modes: incompatible,
                modifiers: [1.0; 8],
            },
        );
        self.api.inner.invalidate_results();
        OAPI_OK
    }

    pub fn count(&self) -> usize {
        self.api.inner.solid_sections.len()
    }
}

/// Area / Shell Object API (`SAP2000v19.cAreaObj`).
pub struct cAreaObj<'a> {
    api: &'a mut cSapModel,
}

impl cAreaObj<'_> {
    pub fn set_modifiers(&mut self, name: &str, modifiers: [f64; 10]) -> i32 {
        if self.api.inner.is_locked
            || !self.api.inner.areas.contains_key(name)
            || modifiers.iter().any(|v| !v.is_finite() || *v < 0.0)
        {
            return OAPI_ERR;
        }
        self.api
            .inner
            .areas
            .get_mut(name)
            .expect("validated area")
            .modifiers = modifiers;
        self.api.inner.invalidate_results();
        OAPI_OK
    }

    /// Uniform pressure on -1 bottom or -2 top. Edge-face and joint-pattern
    /// pressure are rejected because their solver formulation is not implemented.
    pub fn set_load_surface_pressure(
        &mut self,
        name: &str,
        load_pat: &str,
        face: i32,
        val: f64,
        pattern_name: Option<&str>,
        replace: bool,
    ) -> i32 {
        if self.api.inner.is_locked
            || !self.api.inner.areas.contains_key(name)
            || !val.is_finite()
            || !self.api.inner.load_patterns.contains_key(load_pat)
            || ![-1, -2].contains(&face)
            || pattern_name.is_some()
        {
            return OAPI_ERR;
        }
        let area = self.api.inner.areas.get_mut(name).expect("validated area");
        if replace {
            area.loads_pressure
                .retain(|l| l.load_pat != load_pat || l.face != face);
        }
        area.loads_pressure.push(AreaLoadSurfacePressure {
            load_pat: load_pat.into(),
            face,
            val,
            pattern_name: pattern_name.map(str::to_owned),
        });
        self.api.inner.invalidate_results();
        OAPI_OK
    }

    pub fn add_by_point(
        &mut self,
        points: Vec<String>,
        prop_name: &str,
        user_name: &str,
    ) -> (i32, String) {
        if !self.api.prepare_edit() {
            return (OAPI_ERR, String::new());
        }
        let name = if user_name.trim().is_empty() {
            sap_model::next_numeric_name(self.api.inner.areas.keys())
        } else {
            user_name.to_string()
        };
        self.api.inner.add_area(&name, points, prop_name);
        (OAPI_OK, name)
    }

    pub fn set_load_uniform(
        &mut self,
        name: &str,
        load_pat: &str,
        val: f64,
        dir: i32,
        csys: &str,
        replace: bool,
    ) -> i32 {
        if !self.api.prepare_edit() {
            return OAPI_ERR;
        }
        if let Some(a) = self.api.inner.areas.get_mut(name) {
            if replace {
                a.loads_uniform.retain(|l| l.load_pat != load_pat);
            }
            a.loads_uniform.push(AreaLoadUniform {
                load_pat: load_pat.to_string(),
                csys: csys.to_string(),
                dir,
                val,
            });
            OAPI_OK
        } else {
            OAPI_ERR
        }
    }

    pub fn count(&self) -> usize {
        self.api.inner.areas.len()
    }
}

/// Material Property API (`SAP2000v19.cPropMaterial`).
pub struct cPropMaterial<'a> {
    api: &'a mut cSapModel,
}

impl cPropMaterial<'_> {
    /// Create an isotropic steel, normal-weight concrete, or no-design material.
    /// Other material families need dedicated defaults/constitutive setters.
    pub fn set_material(&mut self, name: &str, mat_type: MatType) -> i32 {
        if self.api.inner.is_locked
            || name.trim().is_empty()
            || !matches!(
                mat_type,
                MatType::Steel | MatType::Concrete | MatType::NoDesign
            )
        {
            return OAPI_ERR;
        }
        // Use the same unit conversion as model initialization. The raw factory
        // helpers describe kip/in/F values and must not be copied into SI models.
        let defaults = SapModel::new(self.api.inner.present_units);
        let key = if mat_type == MatType::Concrete {
            "4000Psi"
        } else {
            "A992Fy50"
        };
        let mut mat = defaults.materials[key].clone();
        mat.name = name.into();
        mat.mat_type = mat_type;
        self.api.inner.materials.insert(name.into(), mat);
        self.api.inner.invalidate_results();
        OAPI_OK
    }

    pub fn set_mp_isotropic(&mut self, name: &str, e: f64, u: f64, a: f64, g: f64) -> i32 {
        if self.api.inner.is_locked
            || [e, u, a, g].iter().any(|v| !v.is_finite())
            || e <= 0.
            || !(-1.0..0.5).contains(&u)
            || u == -1.
            || g < 0.
        {
            return OAPI_ERR;
        }
        let shear = if g == 0. { e / (2. * (1. + u)) } else { g };
        if !shear.is_finite() || shear <= 0. {
            return OAPI_ERR;
        }
        let Some(mat) = self.api.inner.materials.get_mut(name) else {
            return OAPI_ERR;
        };
        mat.e = e;
        mat.u = u;
        mat.a = a;
        mat.g = shear;
        mat.e_ortho = None;
        mat.u_ortho = None;
        mat.g_ortho = None;
        mat.a_ortho = None;
        self.api.inner.invalidate_results();
        OAPI_OK
    }

    pub fn set_weight_and_mass(&mut self, name: &str, unit_weight: f64, unit_mass: f64) -> i32 {
        if self.api.inner.is_locked
            || [unit_weight, unit_mass]
                .iter()
                .any(|v| !v.is_finite() || *v < 0.)
        {
            return OAPI_ERR;
        }
        let Some(mat) = self.api.inner.materials.get_mut(name) else {
            return OAPI_ERR;
        };
        mat.unit_weight = unit_weight;
        mat.unit_mass = unit_mass;
        self.api.inner.invalidate_results();
        OAPI_OK
    }

    /// Expected strengths are not represented separately. Zero or nominal values
    /// carry no additional information; distinct expected strengths are rejected.
    pub fn set_osteel(&mut self, name: &str, fy: f64, fu: f64, efy: f64, efu: f64) -> i32 {
        if self.api.inner.is_locked
            || [fy, fu, efy, efu].iter().any(|v| !v.is_finite())
            || fy <= 0.
            || fu < fy
            || (efy != 0. && efy != fy)
            || (efu != 0. && efu != fu)
        {
            return OAPI_ERR;
        }
        let Some(mat) = self.api.inner.materials.get_mut(name) else {
            return OAPI_ERR;
        };
        if mat.mat_type != MatType::Steel {
            return OAPI_ERR;
        }
        mat.fy = fy;
        mat.fu = fu;
        self.api.inner.invalidate_results();
        OAPI_OK
    }
    /// Lightweight concrete behavior is not represented; reject it explicitly.
    pub fn set_oconcrete(&mut self, name: &str, fc: f64, is_lightweight: bool) -> i32 {
        if self.api.inner.is_locked || !fc.is_finite() || fc <= 0. || is_lightweight {
            return OAPI_ERR;
        }
        let Some(mat) = self.api.inner.materials.get_mut(name) else {
            return OAPI_ERR;
        };
        if mat.mat_type != MatType::Concrete {
            return OAPI_ERR;
        }
        mat.fc = fc;
        self.api.inner.invalidate_results();
        OAPI_OK
    }
}

/// Frame Section Property API (`SAP2000v19.cPropFrame`).
pub struct cPropFrame<'a> {
    api: &'a mut cSapModel,
}

impl cPropFrame<'_> {
    pub fn set_rectangle(&mut self, name: &str, mat_prop: &str, t3: f64, t2: f64) -> i32 {
        if !self.api.prepare_edit() {
            return OAPI_ERR;
        }
        let sec = make_rectangular(name, mat_prop, t3, t2);
        self.api.inner.frame_sections.insert(name.to_string(), sec);
        OAPI_OK
    }

    #[allow(clippy::too_many_arguments)]
    pub fn set_i_section(
        &mut self,
        name: &str,
        mat_prop: &str,
        t3: f64,
        t2: f64,
        tf: f64,
        tw: f64,
        t2b: f64,
        tfb: f64,
    ) -> i32 {
        if !self.api.prepare_edit() {
            return OAPI_ERR;
        }
        let sec = make_i_section(name, mat_prop, t3, t2, tf, tw, t2b, tfb);
        self.api.inner.frame_sections.insert(name.to_string(), sec);
        OAPI_OK
    }

    pub fn set_pipe(&mut self, name: &str, mat_prop: &str, t3: f64, tw: f64) -> i32 {
        if !self.api.prepare_edit() {
            return OAPI_ERR;
        }
        let sec = make_pipe(name, mat_prop, t3, tw);
        self.api.inner.frame_sections.insert(name.to_string(), sec);
        OAPI_OK
    }

    pub fn set_tube(
        &mut self,
        name: &str,
        mat_prop: &str,
        t3: f64,
        t2: f64,
        tf: f64,
        tw: f64,
    ) -> i32 {
        if !self.api.prepare_edit() {
            return OAPI_ERR;
        }
        let sec = make_box_tube(name, mat_prop, t3, t2, tf, tw);
        self.api.inner.frame_sections.insert(name.to_string(), sec);
        OAPI_OK
    }

    pub fn set_circle(&mut self, name: &str, mat_prop: &str, t3: f64) -> i32 {
        if !self.api.prepare_edit() {
            return OAPI_ERR;
        }
        let sec = make_circle(name, mat_prop, t3);
        self.api.inner.frame_sections.insert(name.to_string(), sec);
        OAPI_OK
    }

    #[allow(clippy::too_many_arguments)]
    pub fn set_general(
        &mut self,
        name: &str,
        mat_prop: &str,
        t3: f64,
        t2: f64,
        area: f64,
        as2: f64,
        as3: f64,
        torsional_j: f64,
        i22: f64,
        i33: f64,
        s22: f64,
        s33: f64,
        z22: f64,
        z33: f64,
        r22: f64,
        r33: f64,
    ) -> i32 {
        if !self.api.prepare_edit() {
            return OAPI_ERR;
        }
        let sec = FrameSection {
            name: name.to_string(),
            material: mat_prop.to_string(),
            prop_type: FramePropType::General,
            t3,
            t2,
            area,
            as2,
            as3,
            torsional_j,
            i22,
            i33,
            s22,
            s33,
            z22,
            z33,
            r22,
            r33,
            ..Default::default()
        };
        self.api.inner.frame_sections.insert(name.to_string(), sec);
        OAPI_OK
    }

    /// Imports a section property from a `.pro` database file (`cPropFrame.ImportProp`).
    pub fn import_prop(
        &mut self,
        name: &str,
        mat_prop: &str,
        file_name: &str,
        sect_name: &str,
    ) -> i32 {
        if !self.api.prepare_edit() {
            return OAPI_ERR;
        }
        let mut path = PathBuf::from(file_name);
        if !path.exists() {
            path = self.api.pro_search_dir.join(file_name.to_ascii_lowercase());
        }
        let Ok(db) = ProDatabase::from_file(&path) else {
            return OAPI_ERR;
        };
        let Ok(mut sec) = db.get_section_scaled(sect_name, self.api.inner.present_units, mat_prop)
        else {
            return OAPI_ERR;
        };
        sec.name = name.to_string();
        self.api.inner.frame_sections.insert(name.to_string(), sec);
        OAPI_OK
    }
}

/// Area Section Property API (`SAP2000v19.cPropArea`).
pub struct cPropArea<'a> {
    api: &'a mut cSapModel,
}

impl cPropArea<'_> {
    pub fn set_modifiers(&mut self, name: &str, modifiers: [f64; 10]) -> i32 {
        if self.api.inner.is_locked
            || !self.api.inner.area_sections.contains_key(name)
            || modifiers.iter().any(|v| !v.is_finite() || *v < 0.0)
        {
            return OAPI_ERR;
        }
        self.api
            .inner
            .area_sections
            .get_mut(name)
            .expect("validated section")
            .modifiers = modifiers;
        self.api.inner.invalidate_results();
        OAPI_OK
    }

    pub fn set_shell(
        &mut self,
        name: &str,
        shell_type: ShellType,
        mat_prop: &str,
        thickness: f64,
        bending: f64,
    ) -> i32 {
        if !self.api.prepare_edit() {
            return OAPI_ERR;
        }
        let mut sec = AreaSection::default();
        sec.name = name.to_string();
        sec.material = mat_prop.to_string();
        sec.shell_type = shell_type;
        sec.thickness_membrane = thickness;
        sec.thickness_bending = bending;
        self.api.inner.area_sections.insert(name.to_string(), sec);
        OAPI_OK
    }
}

/// Load Patterns API (`SAP2000v19.cLoadPatterns`).
pub struct cLoadPatterns<'a> {
    api: &'a mut cSapModel,
}

impl cLoadPatterns<'_> {
    pub fn add(
        &mut self,
        name: &str,
        my_type: LoadPatternType,
        self_wt_multiplier: f64,
        add_load_case: bool,
    ) -> i32 {
        if !self.api.prepare_edit() {
            return OAPI_ERR;
        }
        self.api.inner.load_patterns.insert(
            name.to_string(),
            LoadPattern {
                name: name.to_string(),
                pat_type: my_type,
                self_wt_multiplier,
            },
        );
        if add_load_case {
            self.api
                .inner
                .load_cases
                .insert(name.to_string(), LoadCase::linear_static(name, name, 1.0));
            if !self.api.selected_cases.contains(&name.to_string()) {
                self.api.selected_cases.push(name.to_string());
            }
        }
        OAPI_OK
    }

    pub fn set_self_wt_multiplier(&mut self, name: &str, self_wt_multiplier: f64) -> i32 {
        if !self.api.prepare_edit() {
            return OAPI_ERR;
        }
        if let Some(lp) = self.api.inner.load_patterns.get_mut(name) {
            lp.self_wt_multiplier = self_wt_multiplier;
            OAPI_OK
        } else {
            OAPI_ERR
        }
    }
}

/// Load Cases API (`SAP2000v19.cLoadCases`).
pub struct cLoadCases<'a> {
    api: &'a mut cSapModel,
}

impl cLoadCases<'_> {
    pub fn set_linear_static(&mut self, name: &str, loads: Vec<(String, f64)>) -> i32 {
        let mut case = LoadCase::linear_static(name, "", 1.0);
        case.loads = loads
            .into_iter()
            .map(|(load_name, scale_factor)| LoadCaseAssignment {
                load_name,
                scale_factor,
            })
            .collect();
        self.set_definition(case)
    }
    pub fn set_modal_eigen(&mut self, name: &str, max_modes: usize) -> i32 {
        self.set_definition(LoadCase::modal_eigen(name, max_modes))
    }
    pub fn set_pdelta(&mut self, name: &str, loads: Vec<(String, f64)>) -> i32 {
        let mut case = LoadCase::linear_static(name, "", 1.0);
        case.case_type = LoadCaseType::NonlinearStatic;
        case.nl_geom = NLGeomType::PDelta;
        case.loads = loads
            .into_iter()
            .map(|(load_name, scale_factor)| LoadCaseAssignment {
                load_name,
                scale_factor,
            })
            .collect();
        self.set_definition(case)
    }
    pub fn set_buckling(&mut self, name: &str, loads: Vec<(String, f64)>, num_modes: usize) -> i32 {
        let mut case = LoadCase::linear_static(name, "", 1.0);
        case.case_type = LoadCaseType::Buckling;
        case.num_buckling_modes = num_modes;
        case.loads = loads
            .into_iter()
            .map(|(load_name, scale_factor)| LoadCaseAssignment {
                load_name,
                scale_factor,
            })
            .collect();
        self.set_definition(case)
    }
}

/// Response Combinations API (`SAP2000v19.cRespCombo`).
pub struct cRespCombo<'a> {
    api: &'a mut cSapModel,
}

impl cRespCombo<'_> {
    pub fn add(&mut self, name: &str, combo_type: ComboType) -> i32 {
        if !self.api.prepare_edit() {
            return OAPI_ERR;
        }
        self.api.inner.load_combos.insert(
            name.to_string(),
            LoadCombo {
                name: name.to_string(),
                combo_type,
                cases: Vec::new(),
            },
        );
        OAPI_OK
    }

    pub fn set_case_list(&mut self, name: &str, case_name: &str, sf: f64) -> i32 {
        if !self.api.prepare_edit() {
            return OAPI_ERR;
        }
        if let Some(combo) = self.api.inner.load_combos.get_mut(name) {
            combo.cases.push((case_name.to_string(), sf));
            OAPI_OK
        } else {
            OAPI_ERR
        }
    }
}

/// Analysis Execution API (`SAP2000v19.cAnalyze`).
pub struct cAnalyze<'a> {
    api: &'a mut cSapModel,
}

impl cAnalyze<'_> {
    pub fn run_analysis(&mut self) -> i32 {
        match analyze_model(&mut self.api.inner) {
            Ok(()) => OAPI_OK,
            Err(_) => OAPI_ERR,
        }
    }

    pub fn set_run_case_flag(&mut self, name: &str, run: bool, all: bool) -> i32 {
        if all {
            for lc in self.api.inner.load_cases.values_mut() {
                lc.run_flag = run;
            }
            OAPI_OK
        } else if let Some(lc) = self.api.inner.load_cases.get_mut(name) {
            lc.run_flag = run;
            OAPI_OK
        } else {
            OAPI_ERR
        }
    }

    pub fn delete_results(&mut self) -> i32 {
        self.api.set_model_is_locked(false)
    }
}

/// Analysis Results Extraction API (`SAP2000v19.cAnalysisResults`).
pub struct cAnalysisResults<'a> {
    api: &'a mut cSapModel,
}

impl cAnalysisResults<'_> {
    pub fn setup_deselect_all_cases_and_combos_for_output(&mut self) -> i32 {
        self.api.selected_cases.clear();
        self.api.selected_combos.clear();
        OAPI_OK
    }

    pub fn setup_set_case_selected_for_output(&mut self, name: &str, selected: bool) -> i32 {
        self.api.selected_cases.retain(|c| c != name);
        if selected {
            self.api.selected_cases.push(name.to_string());
        }
        OAPI_OK
    }

    pub fn setup_set_combo_selected_for_output(&mut self, name: &str, selected: bool) -> i32 {
        self.api.selected_combos.retain(|c| c != name);
        if selected {
            self.api.selected_combos.push(name.to_string());
        }
        OAPI_OK
    }

    /// Queries joint displacements `[U1, U2, U3, R1, R2, R3]` across selected output cases (`cAnalysisResults.JointDispl`).
    pub fn joint_displ(&self, name: &str) -> Vec<(String, [f64; 6])> {
        let mut out = Vec::new();
        for cname in self
            .api
            .selected_cases
            .iter()
            .chain(self.api.selected_combos.iter())
        {
            if let Some(cr) = self.api.inner.case_results.get(cname) {
                if let Some(&u) = cr.joint_displacements.get(name) {
                    out.push((cname.clone(), u));
                }
            }
        }
        out
    }

    /// Queries support reactions `[F1, F2, F3, M1, M2, M3]` across selected output cases (`cAnalysisResults.JointReact`).
    pub fn joint_react(&self, name: &str) -> Vec<(String, [f64; 6])> {
        let mut out = Vec::new();
        for cname in self
            .api
            .selected_cases
            .iter()
            .chain(self.api.selected_combos.iter())
        {
            if let Some(cr) = self.api.inner.case_results.get(cname) {
                if let Some(&r) = cr.joint_reactions.get(name) {
                    out.push((cname.clone(), r));
                }
            }
        }
        out
    }

    /// Queries internal frame station forces across selected output cases (`cAnalysisResults.FrameForce`).
    pub fn frame_force(&self, name: &str) -> Vec<(String, Vec<FrameStationResult>)> {
        let mut out = Vec::new();
        for cname in self
            .api
            .selected_cases
            .iter()
            .chain(self.api.selected_combos.iter())
        {
            if let Some(cr) = self.api.inner.case_results.get(cname) {
                if let Some(st) = cr.frame_forces.get(name) {
                    out.push((cname.clone(), st.clone()));
                }
            }
        }
        out
    }

    /// Queries modal periods and frequencies across selected modal cases (`cAnalysisResults.ModalPeriod`).
    pub fn modal_period(&self) -> Vec<(String, Vec<ModalModeResult>)> {
        let mut out = Vec::new();
        for cname in &self.api.selected_cases {
            if let Some(cr) = self.api.inner.case_results.get(cname) {
                if !cr.modes.is_empty() {
                    out.push((cname.clone(), cr.modes.clone()));
                }
            }
        }
        out
    }

    /// Queries 3D solid stresses (`cAnalysisResults.SolidStress`).
    pub fn solid_stress(&self, name: &str) -> Vec<(String, SolidResult)> {
        let mut out = Vec::new();
        for cname in self
            .api
            .selected_cases
            .iter()
            .chain(self.api.selected_combos.iter())
        {
            if let Some(cr) = self.api.inner.case_results.get(cname) {
                if let Some(sr) = cr.solid_results.get(name) {
                    out.push((cname.clone(), sr.clone()));
                }
            }
        }
        out
    }
}

/// Steel Frame Design API (`SAP2000v19.cDesignSteel`).
pub struct cDesignSteel<'a> {
    api: &'a mut cSapModel,
}

impl cDesignSteel<'_> {
    pub fn set_code(&mut self, code_name: &str) -> i32 {
        if SteelCode::try_from_code_str(code_name).is_err() {
            return OAPI_ERR;
        }
        self.api.inner.steel_design_results.clear();
        self.api.inner.steel_design_code = code_name.to_string();
        OAPI_OK
    }

    pub fn get_code(&self) -> &str {
        &self.api.inner.steel_design_code
    }

    pub fn start_design(&mut self) -> i32 {
        match try_run_steel_design(&mut self.api.inner) {
            Ok(()) => OAPI_OK,
            Err(_) => OAPI_ERR,
        }
    }

    pub fn get_summary_results(&self, name: &str) -> Option<SteelDesignResult> {
        self.api.inner.steel_design_results.get(name).cloned()
    }
}

/// Concrete Frame Design API (`SAP2000v19.cDesignConcrete`).
pub struct cDesignConcrete<'a> {
    api: &'a mut cSapModel,
}

impl cDesignConcrete<'_> {
    pub fn set_code(&mut self, code_name: &str) -> i32 {
        if ConcreteCode::try_from_code_str(code_name).is_err() {
            return OAPI_ERR;
        }
        self.api.inner.concrete_design_results.clear();
        self.api.inner.concrete_design_code = code_name.to_string();
        OAPI_OK
    }

    pub fn get_code(&self) -> &str {
        &self.api.inner.concrete_design_code
    }

    pub fn start_design(&mut self) -> i32 {
        match try_run_concrete_design(&mut self.api.inner) {
            Ok(()) => OAPI_OK,
            Err(_) => OAPI_ERR,
        }
    }

    pub fn get_summary_results(&self, name: &str) -> Option<ConcreteDesignResult> {
        self.api.inner.concrete_design_results.get(name).cloned()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sap_format::load_s2k_file;

    #[test]
    fn api_edits_respect_lock_and_generated_names_do_not_collide() {
        let mut api = cSapModel::default();
        api.inner.add_point("1", 0.0, 0.0, 0.0);
        api.inner.add_point("3", 3.0, 0.0, 0.0);
        let (code, name) = api.point_obj().add_cartesian(2.0, 0.0, 0.0, "");
        assert_eq!(code, OAPI_OK);
        assert_eq!(name, "2");
        assert_eq!(api.inner.points["3"].x, 3.0);
        api.inner.is_locked = true;
        let before = api.inner.clone();
        assert_eq!(api.point_obj().set_restraint("1", [true; 6]), OAPI_ERR);
        assert_eq!(
            api.frame_obj().add_by_point("1", "3", "FSEC1", "").0,
            OAPI_ERR
        );
        assert_eq!(api.inner, before);
        api.set_model_is_locked(false);
        assert_eq!(api.point_obj().set_restraint("1", [true; 6]), OAPI_OK);
    }

    #[test]
    fn api_rejects_unknown_design_codes_without_relabeling_outputs() {
        let mut api = cSapModel::default();
        let steel = api.inner.steel_design_code.clone();
        let concrete = api.inner.concrete_design_code.clone();
        assert_eq!(api.design_steel().set_code("AISC typo"), OAPI_ERR);
        assert_eq!(api.design_concrete().set_code("ACI typo"), OAPI_ERR);
        assert_eq!(api.inner.steel_design_code, steel);
        assert_eq!(api.inner.concrete_design_code, concrete);
        api.inner.steel_design_code = "invalid".into();
        assert_eq!(api.design_steel().start_design(), OAPI_ERR);
        assert!(api.inner.steel_design_results.is_empty());
    }

    #[test]
    fn save_on_exit_uses_latest_file_destination() {
        let dir = std::env::temp_dir().join(format!("sap-api-save-exit-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let original = dir.join("original.s2k");
        let renamed = dir.join("renamed.s2k");
        let mut app = SapObject::new();
        assert_eq!(app.application_exit(true), OAPI_ERR);
        assert_eq!(
            app.sap_model.file().save(original.to_str().unwrap()),
            OAPI_OK
        );
        assert_eq!(
            app.application_start(Units::KipInF, false, original.to_str().unwrap()),
            OAPI_OK
        );
        assert_eq!(
            app.sap_model.file().save(renamed.to_str().unwrap()),
            OAPI_OK
        );
        app.sap_model
            .inner
            .add_point("saved-on-exit", 1.0, 2.0, 3.0);
        assert_eq!(app.application_exit(true), OAPI_OK);
        assert!(load_s2k_file(&renamed)
            .unwrap()
            .points
            .contains_key("saved-on-exit"));
        assert!(!load_s2k_file(&original)
            .unwrap()
            .points
            .contains_key("saved-on-exit"));
        assert_eq!(app.sap_model.file().new_blank(), OAPI_OK);
        assert!(app.sap_model.last_file_path.is_none());
        assert_eq!(app.application_exit(true), OAPI_ERR);
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn unlocking_invalidates_all_analysis_and_design_outputs() {
        let mut api = cSapModel::default();
        api.inner
            .case_results
            .insert("stale".into(), Default::default());
        api.inner.steel_design_results.insert(
            "stale".into(),
            SteelDesignResult {
                frame_name: "stale".into(),
                section_name: String::new(),
                design_code: String::new(),
                governing_combo: String::new(),
                governing_station: 0.0,
                total_ratio: 0.0,
                pr_ratio: 0.0,
                mr_major_ratio: 0.0,
                mr_minor_ratio: 0.0,
                shear_major_ratio: 0.0,
                shear_minor_ratio: 0.0,
                phi_pnc: 0.0,
                phi_pnt: 0.0,
                phi_mn33: 0.0,
                phi_mn22: 0.0,
                phi_vn2: 0.0,
                phi_vn3: 0.0,
                status: "OK".into(),
                eq_name: String::new(),
            },
        );
        api.inner.concrete_design_results.insert(
            "stale".into(),
            ConcreteDesignResult {
                frame_name: "stale".into(),
                section_name: String::new(),
                design_code: String::new(),
                governing_combo: String::new(),
                is_column: false,
                as_top: 0.0,
                as_bot: 0.0,
                av_over_s: 0.0,
                pmm_ratio: 0.0,
                min_rebar_area: 0.0,
                status: "OK".into(),
            },
        );
        api.inner.is_locked = true;
        assert_eq!(api.analyze().delete_results(), OAPI_OK);
        assert!(!api.inner.is_locked);
        assert!(api.inner.case_results.is_empty());
        assert!(api.inner.steel_design_results.is_empty());
        assert!(api.inner.concrete_design_results.is_empty());
    }

    #[test]
    fn test_oapi_end_to_end_workflow() {
        let mut sap = SapObject::new();
        assert_eq!(sap.application_start(Units::KipInF, false, ""), OAPI_OK);

        // Define a synthetic section so this workflow needs no vendor database.
        assert_eq!(
            sap.sap_model
                .prop_frame()
                .set_i_section("TEST_I", "A992Fy50", 14.0, 14.0, 0.7, 0.45, 14.0, 0.7),
            OAPI_OK
        );

        // Generate a 2-story 2-bay 2D portal frame using cFile.New2DFrame
        assert_eq!(
            sap.sap_model.file().new_2d_frame(
                FrameType2D::PortalFrame,
                2,
                144.0,
                2,
                240.0,
                true,
                "TEST_I",
                "TEST_I",
                "TEST_I",
            ),
            OAPI_OK
        );

        // Add lateral wind load pattern & point load at top story
        assert_eq!(
            sap.sap_model
                .load_patterns()
                .add("WIND", LoadPatternType::Wind, 0.0, true),
            OAPI_OK
        );
        assert_eq!(
            sap.sap_model.point_obj().set_load_force(
                "3",
                "WIND",
                [25.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                true
            ),
            OAPI_OK
        );

        // Run analysis via cAnalyze.RunAnalysis
        assert_eq!(sap.sap_model.analyze().run_analysis(), OAPI_OK);
        assert!(sap.sap_model.get_model_is_locked());

        // Query displacement at Joint 3 under WIND
        sap.sap_model
            .results()
            .setup_deselect_all_cases_and_combos_for_output();
        sap.sap_model
            .results()
            .setup_set_case_selected_for_output("WIND", true);
        let disp = sap.sap_model.results().joint_displ("3");
        assert_eq!(disp.len(), 1);
        assert!(disp[0].1[0] > 0.0);

        // Run AISC 360-10 Steel Frame Design
        assert_eq!(
            sap.sap_model.design_steel().set_code("AISC 360-10"),
            OAPI_OK
        );
        assert_eq!(sap.sap_model.design_steel().start_design(), OAPI_OK);
        let summary = sap
            .sap_model
            .design_steel()
            .get_summary_results("1")
            .unwrap();
        assert!(summary.total_ratio > 0.0);
    }

    #[test]
    fn test_oapi_solid_cable_and_native_sdb_export() {
        let mut api = cSapModel::new(Units::KipInF);
        // Explicitly choose the native writer's lossless steel-only subset.
        api.inner.materials.shift_remove("4000Psi");
        api.inner.area_sections.clear();
        assert_eq!(
            api.prop_solid()
                .set_prop("BRICK1", "A992Fy50", 0.0, 0.0, 0.0, true),
            OAPI_OK
        );
        for (id, (x, y, z)) in [
            ("1", (0.0, 0.0, 0.0)),
            ("2", (10.0, 0.0, 0.0)),
            ("3", (0.0, 10.0, 0.0)),
            ("4", (10.0, 10.0, 0.0)),
            ("5", (0.0, 0.0, 10.0)),
            ("6", (10.0, 0.0, 10.0)),
            ("7", (0.0, 10.0, 10.0)),
            ("8", (10.0, 10.0, 10.0)),
        ] {
            assert_eq!(api.point_obj().add_cartesian(x, y, z, id).0, OAPI_OK);
        }
        for id in ["1", "2", "3", "4"] {
            assert_eq!(api.point_obj().set_restraint(id, [true; 6]), OAPI_OK);
        }
        let (code, sname) = api.solid_obj().add_by_point(
            [
                "1".into(),
                "2".into(),
                "3".into(),
                "4".into(),
                "5".into(),
                "6".into(),
                "7".into(),
                "8".into(),
            ],
            "BRICK1",
            "S1",
        );
        assert_eq!(code, OAPI_OK);
        assert_eq!(sname, "S1");
        assert_eq!(
            api.solid_obj()
                .set_load_surface_pressure("S1", "DEAD", 6, 1.0, true),
            OAPI_OK
        );
        assert_eq!(api.analyze().run_analysis(), OAPI_OK);
        let solid_res = api.results().solid_stress("S1");
        assert_eq!(solid_res.len(), 1);
        assert!(solid_res[0].1.center_stress.s33 < 0.0);

        let out = std::env::temp_dir().join(format!("sap-api-native-{}.sdb", std::process::id()));
        assert_eq!(api.file().save_native_sdb(out.to_str().unwrap()), OAPI_OK,
            "{:?}", api.last_file_error);
        assert!(api.export_warnings.iter().any(|w| w.contains("Experimental")));
        let doc = sap_format::inspect_native_sdb_file(&out).unwrap();
        assert_eq!(doc.inventory().parsed_bytes, doc.inventory().stream_bytes);
        std::fs::remove_file(out).unwrap();
    }
}
