use sap_model::CableParams;

use crate::{cSapModel, is_valid_csys, is_valid_optional_user_name, OAPI_ERR, OAPI_OK};

/// Cable Object API (`SAP2000v19.cCableObj`).
pub struct cCableObj<'a> {
    pub(crate) api: &'a mut cSapModel,
}

impl<'a> cCableObj<'a> {
    pub fn new(api: &'a mut cSapModel) -> Self {
        Self { api }
    }

    /// Returns the number of cable objects in the model.
    pub fn count(&self) -> usize {
        self.api
            .inner
            .frames
            .values()
            .filter(|f| f.cable_params.is_some())
            .count()
    }

    /// Returns the names of all cable objects in the model.
    pub fn get_name_list(&self) -> (i32, Vec<String>) {
        let names = self
            .api
            .inner
            .frames
            .iter()
            .filter(|(_, f)| f.cable_params.is_some())
            .map(|(k, _)| k.clone())
            .collect();
        (OAPI_OK, names)
    }

    /// Adds a cable object between two existing points.
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
        let chord = (j.x - i.x).hypot(j.y - i.y).hypot(j.z - i.z);
        if self.api.inner.is_locked
            || point1 == point2
            || !chord.is_finite()
            || chord <= 0.0
            || !self.api.inner.frame_sections.contains_key(sec)
            || !is_valid_optional_user_name(user_name)
            || (!user_name.is_empty() && self.api.inner.frames.contains_key(user_name))
        {
            return (OAPI_ERR, String::new());
        }

        let name = if user_name.is_empty() {
            sap_model::next_numeric_name(self.api.inner.frames.keys())
        } else {
            user_name.to_owned()
        };

        let mut cable = sap_model::FrameObj::new(&name, point1, point2, sec);
        cable.cable_params = Some(CableParams {
            undeformed_length: Some(chord),
            target_tension_i: None,
            horizontal_tension: None,
            num_segments: 1,
        });

        self.api.inner.frames.insert(name.clone(), cable);
        self.api.inner.invalidate_results();
        (OAPI_OK, name)
    }

    /// Adds a cable object between two coordinate points, auto-generating endpoint joints.
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
            || length <= 0.0
            || !self.api.inner.frame_sections.contains_key(sec)
            || !is_valid_optional_user_name(user_name)
            || (!user_name.is_empty() && self.api.inner.frames.contains_key(user_name))
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

        let before = self.api.inner.clone();
        let p1 = self.find_or_create_point(xi, yi, zi);
        let p2 = self.find_or_create_point(xj, yj, zj);
        let res = self.add_by_point(&p1, &p2, sec, user_name);
        if res.0 != OAPI_OK {
            self.api.inner = before;
            return (OAPI_ERR, String::new());
        }
        res
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

    /// Retrieves endpoint joint names `(Point1, Point2)` for the specified cable.
    pub fn get_points(&self, name: &str) -> (i32, String, String) {
        if let Some(f) = self.api.inner.frames.get(name) {
            if f.cable_params.is_some() {
                return (OAPI_OK, f.point_i.clone(), f.point_j.clone());
            }
        }
        (OAPI_ERR, String::new(), String::new())
    }

    /// Modifies endpoint joint connectivity for the specified cable.
    pub fn set_points(&mut self, name: &str, point1: &str, point2: &str) -> i32 {
        if self.api.inner.is_locked || point1 == point2 {
            return OAPI_ERR;
        }
        let (Some(p1), Some(p2)) = (
            self.api.inner.points.get(point1),
            self.api.inner.points.get(point2),
        ) else {
            return OAPI_ERR;
        };
        let chord = (p2.x - p1.x).hypot(p2.y - p1.y).hypot(p2.z - p1.z);
        if !chord.is_finite() || chord <= 0.0 {
            return OAPI_ERR;
        }

        let Some(f) = self.api.inner.frames.get_mut(name) else {
            return OAPI_ERR;
        };
        if f.cable_params.is_none() {
            return OAPI_ERR;
        }
        f.point_i = point1.to_string();
        f.point_j = point2.to_string();
        self.api.inner.invalidate_results();
        OAPI_OK
    }

    /// Retrieves the assigned section property name for the cable.
    pub fn get_section(&self, name: &str) -> (i32, String) {
        if let Some(f) = self.api.inner.frames.get(name) {
            if f.cable_params.is_some() {
                return (OAPI_OK, f.section.clone());
            }
        }
        (OAPI_ERR, String::new())
    }

    /// Assigns a section property to the cable.
    pub fn set_section(&mut self, name: &str, prop_name: &str) -> i32 {
        if self.api.inner.is_locked || !self.api.inner.frame_sections.contains_key(prop_name) {
            return OAPI_ERR;
        }
        let Some(f) = self.api.inner.frames.get_mut(name) else {
            return OAPI_ERR;
        };
        if f.cable_params.is_none() {
            return OAPI_ERR;
        }
        f.section = prop_name.to_string();
        self.api.inner.invalidate_results();
        OAPI_OK
    }

    /// Retrieves axial tension and compression limits.
    pub fn get_tension_compression_limits(&self, name: &str) -> (i32, bool, f64, bool, f64) {
        if let Some(f) = self.api.inner.frames.get(name) {
            if f.cable_params.is_some() {
                if let Some((c_exists, c, t_exists, t)) = f.tc_limits {
                    return (OAPI_OK, c_exists, c, t_exists, t);
                }
                return (OAPI_OK, false, 0.0, false, 0.0);
            }
        }
        (OAPI_ERR, false, 0.0, false, 0.0)
    }

    pub fn get_tc_limits(&self, name: &str) -> (i32, bool, f64, bool, f64) {
        self.get_tension_compression_limits(name)
    }

    /// Sets axial tension and compression limits.
    pub fn set_tension_compression_limits(
        &mut self,
        name: &str,
        limit_compression_exists: bool,
        limit_compression: f64,
        limit_tension_exists: bool,
        limit_tension: f64,
    ) -> i32 {
        if self.api.inner.is_locked
            || !limit_compression.is_finite()
            || !limit_tension.is_finite()
            || (limit_compression_exists && limit_compression > 0.0)
            || (limit_tension_exists && limit_tension < 0.0)
        {
            return OAPI_ERR;
        }
        let Some(f) = self.api.inner.frames.get_mut(name) else {
            return OAPI_ERR;
        };
        if f.cable_params.is_none() {
            return OAPI_ERR;
        }
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
        self.api.inner.invalidate_results();
        OAPI_OK
    }

    pub fn set_tc_limits(
        &mut self,
        name: &str,
        limit_compression_exists: bool,
        limit_compression: f64,
        limit_tension_exists: bool,
        limit_tension: f64,
    ) -> i32 {
        self.set_tension_compression_limits(
            name,
            limit_compression_exists,
            limit_compression,
            limit_tension_exists,
            limit_tension,
        )
    }

    /// Retrieves unstressed (undeformed) arc length L0.
    pub fn get_unstressed_length(&self, name: &str) -> (i32, f64) {
        if let Some(f) = self.api.inner.frames.get(name) {
            if let Some(ref cp) = f.cable_params {
                if let Some(l0) = cp.undeformed_length {
                    return (OAPI_OK, l0);
                }
                // If undeformed_length is not explicitly set, calculate chord length
                if let (Some(pi), Some(pj)) = (
                    self.api.inner.points.get(&f.point_i),
                    self.api.inner.points.get(&f.point_j),
                ) {
                    let chord = (pj.x - pi.x).hypot(pj.y - pi.y).hypot(pj.z - pi.z);
                    return (OAPI_OK, chord);
                }
            }
        }
        (OAPI_ERR, 0.0)
    }

    /// Sets explicit unstressed length L0, clearing competing tension targets to maintain SDB invariance.
    pub fn set_unstressed_length(&mut self, name: &str, length: f64) -> i32 {
        if self.api.inner.is_locked || !length.is_finite() || length <= 0.0 {
            return OAPI_ERR;
        }
        let Some(f) = self.api.inner.frames.get_mut(name) else {
            return OAPI_ERR;
        };
        if let Some(ref mut cp) = f.cable_params {
            cp.undeformed_length = Some(length);
            cp.target_tension_i = None;
            cp.horizontal_tension = None;
            self.api.inner.invalidate_results();
            OAPI_OK
        } else {
            OAPI_ERR
        }
    }

    /// Retrieves full cable geometry parameters.
    pub fn get_cable_data(
        &self,
        name: &str,
    ) -> (i32, Option<f64>, Option<f64>, Option<f64>, usize) {
        if let Some(f) = self.api.inner.frames.get(name) {
            if let Some(ref cp) = f.cable_params {
                return (
                    OAPI_OK,
                    cp.undeformed_length,
                    cp.target_tension_i,
                    cp.horizontal_tension,
                    cp.num_segments,
                );
            }
        }
        (OAPI_ERR, None, None, None, 0)
    }

    /// Sets full cable geometry specification. Enforces num_segments >= 1 and at most one active target.
    pub fn set_cable_data(
        &mut self,
        name: &str,
        undeformed_length: Option<f64>,
        target_tension_i: Option<f64>,
        horizontal_tension: Option<f64>,
        num_segments: usize,
    ) -> i32 {
        let targets = [undeformed_length, target_tension_i, horizontal_tension];
        if self.api.inner.is_locked
            || !(1..=i32::MAX as usize).contains(&num_segments)
            || targets
                .iter()
                .flatten()
                .any(|v| !v.is_finite() || *v <= 0.0)
            || targets.iter().flatten().count() > 1
        {
            return OAPI_ERR;
        }
        let Some(f) = self.api.inner.frames.get_mut(name) else {
            return OAPI_ERR;
        };
        if f.cable_params.is_none() {
            return OAPI_ERR;
        }
        f.cable_params = Some(CableParams {
            undeformed_length,
            target_tension_i,
            horizontal_tension,
            num_segments,
        });
        self.api.inner.invalidate_results();
        OAPI_OK
    }

    /// Deletes a cable object from the model.
    pub fn delete(&mut self, name: &str) -> i32 {
        if self.api.inner.is_locked {
            return OAPI_ERR;
        }
        let Some(f) = self.api.inner.frames.get(name) else {
            return OAPI_ERR;
        };
        if f.cable_params.is_none() {
            return OAPI_ERR;
        }
        self.api.inner.frames.shift_remove(name);
        self.api.inner.invalidate_results();
        OAPI_OK
    }

    /// Gets property modifiers `[f64; 8]`.
    pub fn get_modifiers(&self, name: &str) -> (i32, [f64; 8]) {
        if let Some(f) = self.api.inner.frames.get(name) {
            if f.cable_params.is_some() {
                return (OAPI_OK, f.modifiers);
            }
        }
        (OAPI_ERR, [0.0; 8])
    }

    /// Sets property modifiers `[f64; 8]`.
    pub fn set_modifiers(&mut self, name: &str, modifiers: [f64; 8]) -> i32 {
        if self.api.inner.is_locked || modifiers.iter().any(|v| !v.is_finite() || *v < 0.0) {
            return OAPI_ERR;
        }
        let Some(f) = self.api.inner.frames.get_mut(name) else {
            return OAPI_ERR;
        };
        if f.cable_params.is_none() {
            return OAPI_ERR;
        }
        f.modifiers = modifiers;
        self.api.inner.invalidate_results();
        OAPI_OK
    }

    /// Gets GUI selection state.
    pub fn get_selected(&self, name: &str) -> (i32, bool) {
        if let Some(f) = self.api.inner.frames.get(name) {
            if f.cable_params.is_some() {
                return (OAPI_OK, f.selected);
            }
        }
        (OAPI_ERR, false)
    }

    /// Sets GUI selection state.
    pub fn set_selected(&mut self, name: &str, selected: bool) -> i32 {
        let Some(f) = self.api.inner.frames.get_mut(name) else {
            return OAPI_ERR;
        };
        if f.cable_params.is_none() {
            return OAPI_ERR;
        }
        f.selected = selected;
        OAPI_OK
    }

    /// Assigns distributed loads to the cable.
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
        if self.api.inner.is_locked
            || !self.api.inner.load_patterns.contains_key(load_pat)
            || !(1..=2).contains(&my_type)
            || !(1..=11).contains(&dir)
            || [dist1, dist2, val1, val2].iter().any(|v| !v.is_finite())
            || !(0.0..=1.0).contains(&dist1)
            || !(0.0..=1.0).contains(&dist2)
            || dist1 >= dist2
            || !is_valid_csys(&self.api.inner, csys)
        {
            return OAPI_ERR;
        }
        let Some(f) = self.api.inner.frames.get_mut(name) else {
            return OAPI_ERR;
        };
        if f.cable_params.is_none() {
            return OAPI_ERR;
        }
        if replace {
            f.loads_dist.retain(|l| l.load_pat != load_pat);
        }
        f.loads_dist.push(sap_model::FrameLoadDistributed {
            load_pat: load_pat.into(),
            my_type,
            dir,
            csys: csys.into(),
            rel_dist1: dist1,
            rel_dist2: dist2,
            val1,
            val2,
        });
        self.api.inner.invalidate_results();
        OAPI_OK
    }

    /// Assigns uniform temperature loads to the cable.
    pub fn set_load_temperature(
        &mut self,
        name: &str,
        load_pat: &str,
        my_type: i32,
        val: f64,
        joint_pattern: Option<&str>,
        replace: bool,
    ) -> i32 {
        if self.api.inner.is_locked
            || !val.is_finite()
            || !self.api.inner.load_patterns.contains_key(load_pat)
            || my_type != 1
            || joint_pattern.is_some()
        {
            return OAPI_ERR;
        }
        let Some(f) = self.api.inner.frames.get_mut(name) else {
            return OAPI_ERR;
        };
        if f.cable_params.is_none() {
            return OAPI_ERR;
        }
        if replace {
            f.loads_temp.retain(|l| l.load_pat != load_pat);
        }
        f.loads_temp.push(sap_model::FrameLoadTemperature {
            load_pat: load_pat.into(),
            my_type,
            val,
            pattern_name: None,
        });
        self.api.inner.invalidate_results();
        OAPI_OK
    }
}
