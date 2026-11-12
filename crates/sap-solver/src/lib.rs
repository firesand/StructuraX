//! Core FEM structural analysis engine (`sap-solver`), cleanly re-implementing the mathematical
//! formulations of `CsiGo2.dll`, `csigo2_a.dll`, and `CSI.SAPFire.dll`.

pub mod area;
pub mod benchmarks;
pub mod cable;
mod coordinates;
mod dynamics;
pub mod frame;
mod frame_mesh;
pub mod linalg;
mod link;
mod nonlinear;
pub mod solid;
mod supported_subset;

use indexmap::IndexMap;
use sap_model::{
    AreaObj, CaseResults, ComboType, ConstraintAxis, ConstraintType, FrameStationResult, LoadCase,
    LoadCaseType, ModalComboMethod, ModalModeResult, ModalSubType, NLGeomType, PointObj, SapModel,
    SolidStress,
};
use std::f64::consts::PI;
use thiserror::Error;

use crate::area::{
    area_local_system, compute_area_local_loads, compute_area_local_stiffness, recover_area_result,
};
use crate::frame::{
    apply_insertion_offset, apply_rigid_offsets, compute_flexible_fef_for_pattern,
    compute_flexible_local_stiffness, insertion_offset_local, insertion_to_element_ends,
    compute_local_geometric_stiffness, condense_releases_and_springs, eval_section_props_at,
    frame_local_axes, joint_local_to_global_matrix, mat3_mul, mat3_transpose,
    recover_frame_stations, resolve_load_dir_to_local,
};
use crate::linalg::{
    solve_buckling_eigen, solve_modal_eigen, solve_ritz_vectors, LinalgError, SymmetricLdlt,
};
use crate::solid::{
    compute_solid_nodal_loads, compute_solid_stiffness_and_mass, recover_solid_result,
    solid_constitutive_matrix, solid_face_nodes, validate_solid_geometry,
};

#[derive(Debug, Error)]
pub enum SolverError {
    #[error("Linear algebra error: {0}")]
    Linalg(#[from] LinalgError),
    #[error("Model has no joints to analyze")]
    EmptyModel,
    #[error("Referenced point '{0}' not found in model")]
    PointNotFound(String),
    #[error("Referenced section '{0}' not found in model")]
    SectionNotFound(String),
    #[error("Referenced material '{0}' not found in model")]
    MaterialNotFound(String),
    #[error("Unsupported analysis input: {0}")]
    UnsupportedInput(String),
    #[error("Nonlinear case '{case}' did not converge after {iterations} iterations (residual {residual:e}): {reason}")]
    NonConvergence {
        case: String,
        iterations: usize,
        residual: f64,
        reason: String,
    },
    #[error("Invalid analysis input: {0}")]
    InvalidInput(String),
}

/// Runs all active `LoadCase`s and evaluates all `LoadCombo`s in `model`,
/// populating `model.case_results` and locking the model (`model.is_locked = true`).
pub fn analyze_model(model: &mut SapModel) -> Result<(), SolverError> {
    // A failed re-analysis must never leave old or partially computed results usable.
    model.is_locked = false;
    model.case_results.clear();
    model.steel_design_results.clear();
    model.concrete_design_results.clear();
    // Analyze a working copy so coordinate normalization never rewrites assignments.
    let mut working = model.clone();
    let result =
        coordinates::normalize_loads(&mut working).and_then(|()| analyze_model_inner(&mut working));
    if result.is_ok() {
        model.case_results = working.case_results;
        model.is_locked = working.is_locked;
    }
    if result.is_err() {
        model.case_results.clear();
    }
    result
}

fn analyze_model_inner(model: &mut SapModel) -> Result<(), SolverError> {
    if model.points.is_empty() {
        return Err(SolverError::EmptyModel);
    }
    refuse_unmodeled_source(model)?;
    validate_analysis_inputs(model)?;
    let ordered_cases = analysis_case_order(model)?;

    // Replace automatically subdivided frame objects by their elements. The
    // results are folded back onto the objects before returning.
    let area_mesh_joints: Vec<(String, [f64; 3])> =
        if model
            .frames
            .values()
            .any(|f| f.auto_mesh_at_joints || f.auto_mesh_segments > 1)
        {
            expand_auto_meshed_areas(model)
                .0
                .into_iter()
                .filter(|(name, _)| !model.points.contains_key(name))
                .map(|(name, p)| (name, [p.x, p.y, p.z]))
                .collect()
        } else {
            Vec::new()
        };
    let frame_mesh = frame_mesh::expand(model, &area_mesh_joints)?;

    // Expand any auto-meshed AreaObjs (`auto_mesh.0 > 1 || auto_mesh.1 > 1`) into a working point/area list
    let (work_points, work_areas, orig_area_map) = expand_auto_meshed_areas(model);

    let num_pts = work_points.len();
    let ndof = num_pts * 6;
    let pt_index: IndexMap<String, usize> = work_points
        .keys()
        .enumerate()
        .map(|(idx, name)| (name.clone(), idx))
        .collect();

    // Precompute joint local-to-global rotation matrices R_p (3x3)
    let joint_rot: Vec<[[f64; 3]; 3]> = work_points
        .values()
        .map(joint_local_to_global_matrix)
        .collect();

    // Assemble base elastic structural stiffness K_struct (without grounded springs or restraints)
    // and lumped diagonal mass vector diag_mass (in joint-local coordinates).
    let (k_struct, diag_mass) = assemble_elastic_stiffness_and_mass(
        model,
        &work_points,
        &work_areas,
        &pt_index,
        &joint_rot,
        ndof,
    )?;

    // Build K_base = K_struct + grounded springs + multipoint constraints
    let k_base =
        add_springs_and_constraints(model, &work_points, &pt_index, &joint_rot, &k_struct, ndof);

    for case_name in ordered_cases {
        let lc = model.load_cases.get(&case_name).unwrap().clone();
        let res = match lc.case_type {
            LoadCaseType::LinearStatic | LoadCaseType::LinearStaticMultiStep => {
                run_linear_static_case(
                    model,
                    &lc,
                    &work_points,
                    &work_areas,
                    &orig_area_map,
                    &pt_index,
                    &joint_rot,
                    &k_struct,
                    &k_base,
                    ndof,
                )?
            }
            LoadCaseType::NonlinearStatic => run_nonlinear_static_case(
                model,
                &lc,
                &work_points,
                &work_areas,
                &orig_area_map,
                &pt_index,
                &joint_rot,
                &k_struct,
                &k_base,
                ndof,
            )?,
            LoadCaseType::Modal => run_modal_case(
                model,
                &lc,
                &work_points,
                &pt_index,
                &joint_rot,
                &k_base,
                &diag_mass,
                ndof,
            )?,
            LoadCaseType::Buckling => run_buckling_case(
                model,
                &lc,
                &work_points,
                &work_areas,
                &orig_area_map,
                &pt_index,
                &joint_rot,
                &k_struct,
                &k_base,
                ndof,
            )?,
            LoadCaseType::ResponseSpectrum => run_response_spectrum_case(
                model,
                &lc,
                &work_points,
                &work_areas,
                &orig_area_map,
                &pt_index,
                &joint_rot,
                &k_struct,
                &k_base,
                &diag_mass,
                ndof,
            )?,
            LoadCaseType::LinearHistory
            | LoadCaseType::NonlinearHistory
            | LoadCaseType::LinearDynamic
            | LoadCaseType::NonlinearDynamic => run_time_history_case(
                model,
                &lc,
                &work_points,
                &work_areas,
                &orig_area_map,
                &pt_index,
                &joint_rot,
                &k_struct,
                &k_base,
                &diag_mass,
                ndof,
            )?,
            _ => {
                return Err(SolverError::UnsupportedInput(format!(
                    "case '{}' has type {:?}",
                    case_name, lc.case_type
                )))
            }
        };
        model.case_results.insert(case_name, res);
    }

    // Evaluate all LoadCombos
    evaluate_load_combos(model);
    frame_mesh::collapse(&mut model.case_results, &frame_mesh);

    model.is_locked = true;
    Ok(())
}

/// The insertion offset of a frame, with the reason as a solver error if the
/// assignment is one the element does not implement. Validation refuses those
/// first; this keeps every later use from falling back to a centroidal frame.
fn frame_insertion_offset(
    model: &SapModel,
    frame: &sap_model::FrameObj,
) -> Result<[f64; 3], SolverError> {
    insertion_offset_local(model, frame).map_err(|reason| {
        SolverError::UnsupportedInput(format!(
            "frame '{}' has an insertion point with stiffness transformation and {reason}",
            frame.name
        ))
    })
}

/// Reject requests that the implemented algorithms cannot represent. Returning an
/// apparently valid linear result for these requests is more harmful than failing.
fn validate_analysis_inputs(model: &SapModel) -> Result<(), SolverError> {
    for solid in model.solids.values() {
        let sec = model
            .solid_sections
            .get(&solid.section)
            .ok_or_else(|| SolverError::SectionNotFound(solid.section.clone()))?;
        let mat = model
            .materials
            .get(&sec.material)
            .ok_or_else(|| SolverError::MaterialNotFound(sec.material.clone()))?;
        solid_constitutive_matrix(mat, sec)?;
        let mut coords = [[0.0; 3]; 8];
        for (i, pname) in solid.points.iter().enumerate() {
            let pt = model
                .points
                .get(pname)
                .ok_or_else(|| SolverError::PointNotFound(pname.clone()))?;
            coords[i] = [pt.x, pt.y, pt.z];
        }
        validate_solid_geometry(&coords)?;
        for surf in &solid.loads_surface {
            solid_face_nodes(surf.face)?;
            if !surf.val.is_finite() {
                return Err(SolverError::InvalidInput(format!(
                    "solid '{}' has nonfinite surface pressure",
                    solid.name
                )));
            }
        }
        for dt in solid.loads_temp.values() {
            if !dt.is_finite() {
                return Err(SolverError::InvalidInput(format!(
                    "solid '{}' has nonfinite temperature load",
                    solid.name
                )));
            }
        }
    }
    validate_active_loads(model)?;
    link::validate(model)?;
    for (pname, pt) in &model.points {
        if pt.spring.iter().any(|x| !x.is_finite())
            || pt
                .spring_coupled
                .as_ref()
                .is_some_and(|c| c.iter().any(|x| !x.is_finite()))
        {
            return Err(SolverError::InvalidInput(format!(
                "joint '{pname}' has nonfinite spring stiffness"
            )));
        }
    }
    for area in model.areas.values() {
        if !(3..=4).contains(&area.points.len()) {
            return Err(SolverError::UnsupportedInput(format!(
                "area '{}' requires 3 or 4 nodes",
                area.name
            )));
        }
        // Validate before auto-meshing, whose working-mesh expansion needs all corners.
        for point in &area.points {
            if !model.points.contains_key(point) {
                return Err(SolverError::PointNotFound(point.clone()));
            }
        }
        validate_area_geometry(model, area)?;
        // An axisymmetric-solid section would otherwise be assembled as a
        // plane-stress membrane, which is a different problem.
        if model
            .area_sections
            .get(&area.section)
            .is_some_and(|section| section.prop_type == sap_model::AreaPropType::ASolid)
        {
            return Err(SolverError::UnsupportedInput(format!(
                "area '{}' uses an axisymmetric-solid section, which is not implemented",
                area.name
            )));
        }
        // The area element has no incompatible bending modes. A plane section
        // that asks for them (the vendor default) would be solved without.
        if model.area_sections.get(&area.section).is_some_and(|section| {
            section.prop_type == sap_model::AreaPropType::Plane && section.incompatible_modes
        }) {
            return Err(SolverError::UnsupportedInput(format!(
                "area '{}' uses a plane section with incompatible bending modes, which are not implemented for area elements",
                area.name
            )));
        }
        if !area.mass_per_area.is_finite() || area.mass_per_area < 0.0 {
            return Err(SolverError::InvalidInput(format!(
                "area '{}' added mass per area must be finite and nonnegative",
                area.name
            )));
        }
    }
    let has_cables = model.frames.values().any(|f| f.cable_params.is_some());
    for frame in model.frames.values() {
        // An insertion point (cardinal point plus joint offsets) with
        // stiffness transformation sets the element off from its joints on
        // rigid arms. That is implemented for a parallel offset in linear
        // static cases; everything else is refused, not solved as if the
        // frame were centroidal.
        match insertion_offset_local(model, frame) {
            Ok(offset) if offset != [0.0; 3] => {
                if let Some(case) = model.load_cases.values().find(|lc| {
                    lc.run_flag
                        && !(matches!(
                            lc.case_type,
                            LoadCaseType::LinearStatic | LoadCaseType::LinearStaticMultiStep
                        ) && lc.initial_case.is_none())
                }) {
                    return Err(SolverError::UnsupportedInput(format!(
                        "frame '{}' has an insertion point with stiffness transformation, which is implemented for linear static cases with no initial case only; case '{}' is not one (mass and geometric stiffness of the offset are not modeled). Turn the case off, or use cardinal point 10 with zero joint offsets, or turn the transformation off to analyze the frame at its centroid (S2K table \"FRAME INSERTION POINT ASSIGNMENTS\")",
                        frame.name, case.name
                    )));
                }
            }
            Ok(_) => {}
            Err(reason) => {
                return Err(SolverError::UnsupportedInput(format!(
                    "frame '{}' has an insertion point with stiffness transformation and {reason}, a combination that is not implemented; use cardinal point 10 with zero joint offsets, or turn the transformation off to analyze the frame at its centroid (S2K table \"FRAME INSERTION POINT ASSIGNMENTS\", columns CardinalPt, StiffTrans and the offset columns)",
                    frame.name
                )));
            }
        }
        if frame.auto_mesh_segments > frame_mesh::MAX_SEGMENTS {
            return Err(SolverError::InvalidInput(format!(
                "frame '{}' asks for {} automatic segments; at most {} are accepted",
                frame.name,
                frame.auto_mesh_segments,
                frame_mesh::MAX_SEGMENTS
            )));
        }
        if let Some(cp) = &frame.cable_params {
            if cp
                .undeformed_length
                .is_some_and(|l| !l.is_finite() || l <= 0.0)
                || cp
                    .target_tension_i
                    .is_some_and(|t| !t.is_finite() || t <= 0.0)
                || cp
                    .horizontal_tension
                    .is_some_and(|h| !h.is_finite() || h <= 0.0)
            {
                return Err(SolverError::InvalidInput(format!(
                    "cable '{}' has nonpositive or nonfinite cable parameter",
                    frame.name
                )));
            }
        }
        if let Some((has_c, c_lim, has_t, t_lim)) = frame.tc_limits {
            if (has_c && !c_lim.is_finite()) || (has_t && (!t_lim.is_finite() || t_lim < 0.0)) {
                return Err(SolverError::InvalidInput(format!(
                    "frame '{}' has invalid tension/compression limit",
                    frame.name
                )));
            }
        }
        let pi = model
            .points
            .get(&frame.point_i)
            .ok_or_else(|| SolverError::PointNotFound(frame.point_i.clone()))?;
        let pj = model
            .points
            .get(&frame.point_j)
            .ok_or_else(|| SolverError::PointNotFound(frame.point_j.clone()))?;
        let length = ((pj.x - pi.x).powi(2) + (pj.y - pi.y).powi(2) + (pj.z - pi.z).powi(2)).sqrt();
        if !length.is_finite() || length <= 0.0 {
            return Err(SolverError::InvalidInput(format!(
                "frame '{}' has invalid length",
                frame.name
            )));
        }
    }
    for (name, lc) in model.load_cases.iter().filter(|(_, lc)| lc.run_flag) {
        if lc.loads.iter().any(|load| !load.scale_factor.is_finite()) {
            return Err(SolverError::InvalidInput(format!(
                "case '{name}' has nonfinite load scale"
            )));
        }

        if !matches!(
            lc.case_type,
            LoadCaseType::LinearStatic
                | LoadCaseType::NonlinearStatic
                | LoadCaseType::Modal
                | LoadCaseType::Buckling
                | LoadCaseType::ResponseSpectrum
                | LoadCaseType::LinearHistory
                | LoadCaseType::NonlinearHistory
                | LoadCaseType::LinearDynamic
                | LoadCaseType::NonlinearDynamic
        ) {
            return Err(SolverError::UnsupportedInput(format!(
                "case '{name}' has type {:?}",
                lc.case_type
            )));
        }
        if matches!(
            lc.case_type,
            LoadCaseType::NonlinearStatic
                | LoadCaseType::NonlinearHistory
                | LoadCaseType::NonlinearDynamic
        ) {
            if lc.max_iterations == 0
                || !lc.convergence_tol.is_finite()
                || lc.convergence_tol <= 0.
                || lc.convergence_tol >= 1.
            {
                return Err(SolverError::InvalidInput(format!("case '{name}' needs positive nonlinear iteration count and finite relative tolerance in (0,1)")));
            }
            for object in model.links.values() {
                let prop = &model.link_props[&object.prop];
                if !prop.nonlinear_data_complete {
                    return Err(SolverError::UnsupportedInput(format!(
                        "link '{}' has incomplete imported nonlinear constitutive data",
                        object.name
                    )));
                }
                if !matches!(
                    prop.link_type,
                    sap_model::LinkPropType::Linear
                        | sap_model::LinkPropType::Gap
                        | sap_model::LinkPropType::Hook
                        | sap_model::LinkPropType::Damper
                        | sap_model::LinkPropType::PlasticWen
                ) {
                    return Err(SolverError::UnsupportedInput(format!(
                        "nonlinear link constitutive iteration for '{}' is not implemented",
                        object.name
                    )));
                }
                for d in 0..6 {
                    if prop.dof_nonlinear[d] && prop.dof_active[d] && !prop.dof_fixed[d] {
                        if !matches!(
                            prop.link_type,
                            sap_model::LinkPropType::Gap
                                | sap_model::LinkPropType::Hook
                                | sap_model::LinkPropType::Damper
                                | sap_model::LinkPropType::PlasticWen
                        ) {
                            return Err(SolverError::InvalidInput(format!(
                                "link '{}' nonlinear flags require a supported nonlinear property",
                                object.name
                            )));
                        }
                        if prop.link_type == sap_model::LinkPropType::PlasticWen {
                            nonlinear::wen_law_for(prop, d, &object.name)?;
                            if model
                                .frames
                                .values()
                                .any(|f| f.tc_limits.is_some_and(|(c, _, t, _)| c || t))
                            {
                                return Err(SolverError::UnsupportedInput(format!(
                                    "link '{}' combines plastic Wen behavior with frame tension/compression limits, which is not implemented",
                                    object.name
                                )));
                            }
                        }
                        if matches!(
                            prop.link_type,
                            sap_model::LinkPropType::Gap | sap_model::LinkPropType::Hook
                        ) && (!prop.k_nl[d].is_finite()
                            || prop.k_nl[d] < 0.
                            || !prop.param1[d].is_finite()
                            || prop.param1[d] < 0.)
                        {
                            return Err(SolverError::InvalidInput(format!("link '{}' gap/hook stiffness and opening must be finite and nonnegative", object.name)));
                        }
                        if prop.link_type == sap_model::LinkPropType::Damper
                            && (!prop.k_nl[d].is_finite()
                                || prop.k_nl[d] <= 0.
                                || !prop.param1[d].is_finite()
                                || prop.param1[d] < 0.
                                || !prop.param2[d].is_finite()
                                || prop.param2[d] <= 0.)
                        {
                            return Err(SolverError::InvalidInput(format!(
                                "link '{}' damper stiffness, damping, and exponent must be finite and valid",
                                object.name
                            )));
                        }
                        if lc.case_type == LoadCaseType::NonlinearStatic
                            && lc.nl_geom != NLGeomType::None
                        {
                            return Err(SolverError::UnsupportedInput(format!(
                                "case '{name}' combines geometric nonlinearity and nonlinear links"
                            )));
                        }
                    }
                }
            }
        }
        if !matches!(
            lc.case_type,
            LoadCaseType::NonlinearStatic
                | LoadCaseType::NonlinearHistory
                | LoadCaseType::NonlinearDynamic
        ) && lc.initial_case.as_ref().is_some_and(|initial| {
            model.load_cases.get(initial).is_some_and(|c| {
                matches!(
                    c.case_type,
                    LoadCaseType::NonlinearStatic
                        | LoadCaseType::NonlinearHistory
                        | LoadCaseType::NonlinearDynamic
                )
            })
        }) && model
            .link_props
            .values()
            .any(|p| p.dof_nonlinear.iter().any(|flag| *flag))
        {
            return Err(SolverError::UnsupportedInput(format!("case '{name}' requires continuation from a nonlinear link tangent, which is not stored")));
        }
        // Geometric stiffness is assembled from frame axial forces only. With
        // area or solid elements present, their membrane and continuum forces
        // would be left out of a buckling or P-Delta solution.
        // The cases that assemble it: buckling; nonlinear static with
        // geometric nonlinearity; and linear static and modal cases with an
        // initial case, which take geometric stiffness from that case's frame
        // forces whatever its type.
        let uses_geometric_stiffness = lc.case_type == LoadCaseType::Buckling
            || lc.nl_geom != NLGeomType::None
            || (lc.initial_case.is_some()
                && matches!(
                    lc.case_type,
                    LoadCaseType::LinearStatic
                        | LoadCaseType::LinearStaticMultiStep
                        | LoadCaseType::Modal
                ));
        if uses_geometric_stiffness && (!model.areas.is_empty() || !model.solids.is_empty()) {
            return Err(SolverError::UnsupportedInput(format!(
                "case '{name}' needs geometric stiffness, which is implemented for frame elements only; the model has area or solid elements"
            )));
        }
        // A static case applies load patterns by name. A name that is not a
        // load pattern would contribute nothing, so it must not pass silently:
        // imported acceleration loads (U1..R3) arrive here as such names.
        if matches!(
            lc.case_type,
            LoadCaseType::LinearStatic
                | LoadCaseType::NonlinearStatic
                | LoadCaseType::LinearStaticMultiStep
                | LoadCaseType::Buckling
        ) {
            for load in &lc.loads {
                if model.load_patterns.contains_key(&load.load_name) {
                    continue;
                }
                let upper = load.load_name.trim().to_ascii_uppercase();
                return Err(
                    if matches!(
                        upper.as_str(),
                        "U1" | "U2" | "U3" | "R1" | "R2" | "R3" | "U4" | "U5" | "U6" | "UX"
                            | "UY" | "UZ" | "RX" | "RY" | "RZ"
                    ) {
                        SolverError::UnsupportedInput(format!(
                            "case '{name}' applies acceleration load '{}'; acceleration loads in static cases are not implemented",
                            load.load_name
                        ))
                    } else {
                        SolverError::InvalidInput(format!(
                            "case '{name}' references missing load pattern '{}'",
                            load.load_name
                        ))
                    },
                );
            }
        }
        if let Some(control) = &lc.displacement_control {
            if lc.case_type != LoadCaseType::NonlinearStatic {
                return Err(SolverError::InvalidInput(format!(
                    "case '{name}' uses displacement control, which applies only to nonlinear static cases"
                )));
            }
            if !control.target.is_finite() || control.target <= 0. {
                return Err(SolverError::InvalidInput(format!(
                    "case '{name}' displacement-control target must be finite and positive"
                )));
            }
            let Some((point, dof)) = &control.monitored else {
                return Err(SolverError::UnsupportedInput(format!(
                    "case '{name}' uses displacement control without a representable monitored joint displacement"
                )));
            };
            if !model.points.contains_key(point) || *dof >= 6 {
                return Err(SolverError::InvalidInput(format!(
                    "case '{name}' monitors a missing joint or invalid degree of freedom"
                )));
            }
            if lc.nl_geom != NLGeomType::None
                || has_cables
                || model
                    .frames
                    .values()
                    .any(|f| f.tc_limits.is_some_and(|(c, _, t, _)| c || t))
            {
                return Err(SolverError::UnsupportedInput(format!(
                    "case '{name}' combines displacement control with geometric nonlinearity, cables or frame tension/compression limits, which is not implemented"
                )));
            }
        }
        if lc.nl_geom == NLGeomType::LargeDisplacements && !has_cables {
            return Err(SolverError::UnsupportedInput(format!(
                "case '{name}' requests large displacements"
            )));
        }
        if lc.case_type == LoadCaseType::LinearHistory && lc.initial_case.is_some() {
            return Err(SolverError::UnsupportedInput(format!(
                "initial state for modal history case '{name}' is not implemented"
            )));
        }
        if lc.case_type == LoadCaseType::LinearDynamic {
            if let Some(init_name) = &lc.initial_case {
                if model.load_cases.get(init_name).is_some_and(|c| {
                    !matches!(
                        c.case_type,
                        LoadCaseType::LinearDynamic
                            | LoadCaseType::NonlinearHistory
                            | LoadCaseType::NonlinearDynamic
                    )
                }) {
                    return Err(SolverError::UnsupportedInput(format!(
                        "initial state for case '{name}' from non-transient case '{init_name}' is not implemented"
                    )));
                }
            }
        }
        if matches!(
            lc.case_type,
            LoadCaseType::LinearHistory
                | LoadCaseType::NonlinearHistory
                | LoadCaseType::LinearDynamic
                | LoadCaseType::NonlinearDynamic
        ) {
            // LinearHistory and NonlinearHistory are the vendor's modal-superposition
            // history types. They are integrated here directly over all degrees of
            // freedom with Rayleigh and link damping only, so a modal damping ratio
            // cannot be honored and must not be dropped.
            if matches!(
                lc.case_type,
                LoadCaseType::LinearHistory | LoadCaseType::NonlinearHistory
            ) && lc.modal_damping != 0.0
            {
                return Err(SolverError::UnsupportedInput(format!(
                    "case '{name}' is a modal history type with modal damping ratio {}; modal damping is not implemented for history cases. Set it to zero and use Rayleigh damping, or use a direct-integration case type",
                    lc.modal_damping
                )));
            }
            dynamics::validate_parameters(lc)?;
            for (_, pattern, scale) in &lc.history_loads {
                if !scale.is_finite() {
                    return Err(SolverError::InvalidInput(format!(
                        "case '{name}' has a nonfinite history load scale"
                    )));
                }
                if matches!(
                    pattern.trim().to_ascii_uppercase().as_str(),
                    "U1" | "UX" | "X" | "U2" | "UY" | "Y" | "U3" | "UZ" | "Z"
                ) {
                    continue;
                }
                let pat = model.load_patterns.get(pattern).ok_or_else(|| {
                    SolverError::InvalidInput(format!(
                        "case '{name}' references missing history load pattern '{pattern}'"
                    ))
                })?;
                let unsupported_member_loads = model.frames.values().any(|f| {
                    f.loads_temp.iter().any(|l| &l.load_pat == pattern)
                        || f.loads_gravity.contains_key(pattern)
                }) || model.areas.values().any(|a| {
                    a.loads_uniform.iter().any(|l| &l.load_pat == pattern)
                        || a.loads_pressure.iter().any(|l| &l.load_pat == pattern)
                        || a.loads_temp.iter().any(|l| &l.load_pat == pattern)
                });
                let motion_loads = model
                    .points
                    .values()
                    .any(|p| p.loads_displ.iter().any(|l| &l.load_pat == pattern));
                if pat.self_wt_multiplier != 0.0 || unsupported_member_loads || motion_loads {
                    return Err(SolverError::UnsupportedInput(format!("history pattern '{pattern}' contains self-weight, element loads, or support motion; only joint forces are implemented")));
                }
            }
        }
        let functions = lc
            .history_loads
            .iter()
            .map(|(f, _, _)| f)
            .chain(lc.spectrum_loads.iter().map(|(f, _, _)| f));
        for function in functions {
            let f = model.functions.get(function).ok_or_else(|| {
                SolverError::InvalidInput(format!(
                    "case '{name}' references missing function '{function}'"
                ))
            })?;
            if f.points.is_empty()
                || f.points
                    .iter()
                    .any(|(x, y)| !x.is_finite() || !y.is_finite())
                || f.points.windows(2).any(|p| p[0].0 >= p[1].0)
            {
                return Err(SolverError::InvalidInput(format!(
                    "function '{function}' needs finite, strictly increasing samples"
                )));
            }
        }
    }
    supported_subset::validate(model)?;
    Ok(())
}

fn validate_active_loads(model: &SapModel) -> Result<(), SolverError> {
    let used_patterns: std::collections::BTreeSet<&str> = model
        .load_cases
        .values()
        .filter(|case| case.run_flag)
        .flat_map(|case| {
            case.loads
                .iter()
                .filter(|load| load.scale_factor != 0.0)
                .map(|load| load.load_name.as_str())
                .chain(
                    case.history_loads
                        .iter()
                        .filter(|(_, _, scale)| *scale != 0.0)
                        .map(|(_, pattern, _)| pattern.as_str()),
                )
        })
        .collect();
    for (name, point) in &model.points {
        if point.loads_force.iter().any(|load| {
            used_patterns.contains(load.load_pat.as_str())
                && load.force.iter().any(|x| !x.is_finite())
        }) || point.loads_displ.iter().any(|load| {
            used_patterns.contains(load.load_pat.as_str())
                && load.displ.iter().any(|x| !x.is_finite())
        }) {
            return Err(SolverError::InvalidInput(format!(
                "joint '{name}' has nonfinite applied force or displacement"
            )));
        }
    }
    for (name, area) in &model.areas {
        for load in &area.loads_pressure {
            if used_patterns.contains(load.load_pat.as_str())
                && load.val != 0.
                && (!matches!(load.face, -1 | -2)
                    || load.pattern_name.as_ref().is_some_and(|p| !p.is_empty()))
            {
                return Err(SolverError::UnsupportedInput(format!("area '{name}' has edge or patterned pressure; only uniform top/bottom pressure is implemented")));
            }
        }
        for load in &area.loads_uniform {
            if used_patterns.contains(load.load_pat.as_str())
                && load.val != 0.
                && ((7..=9).contains(&load.dir) || load.dir == 11)
            {
                return Err(SolverError::UnsupportedInput(format!("area '{name}' has projected uniform loading; surface projection is not implemented")));
            }
        }
    }
    Ok(())
}

fn validate_area_geometry(model: &SapModel, area: &AreaObj) -> Result<(), SolverError> {
    let invalid = || {
        SolverError::InvalidInput(format!(
        "area '{}' has degenerate, nonfinite, or folded geometry; use distinct corners forming a valid triangle or convex quadrilateral",
        area.name
    ))
    };
    let origin = &model.points[&area.points[0]];
    let mut coords: Vec<[f64; 3]> = area
        .points
        .iter()
        .map(|name| {
            let point = &model.points[name];
            [point.x - origin.x, point.y - origin.y, point.z - origin.z]
        })
        .collect();
    if coords.iter().flatten().any(|v| !v.is_finite()) {
        return Err(invalid());
    }
    // Normalize before testing geometry so the criterion does not depend on the
    // model's length units, and the local-axis normalization stays well scaled.
    let scale = coords.iter().flatten().fold(0.0_f64, |s, v| s.max(v.abs()));
    if scale == 0.0 {
        return Err(invalid());
    }
    for coord in &mut coords {
        for value in coord {
            *value /= scale;
        }
    }
    let (_, _, xy) = area_local_system(&coords, 0.0);
    let n = xy.len();
    let mut orientation = 0.0_f64;
    // For a bilinear quad these signed corner cross products have the same
    // signs as its corner Jacobians. A zero or sign reversal means a singular
    // or folded mapping; abs(det J) in integration must not conceal it.
    for i in 0..n {
        let a = xy[i];
        let b = xy[(i + 1) % n];
        let c = xy[(i + 2) % n];
        let cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]);
        if !cross.is_finite() || cross.abs() <= 1e-12 {
            return Err(invalid());
        }
        if orientation == 0.0 {
            orientation = cross.signum();
        } else if cross.signum() != orientation {
            return Err(invalid());
        }
    }
    Ok(())
}

/// Imported data that changes results but is not modeled must stop analysis of
/// the cases it affects, before any other check reports a secondary symptom.
fn refuse_unmodeled_source(model: &SapModel) -> Result<(), SolverError> {
    for (name, case) in &model.load_cases {
        if !case.run_flag {
            continue;
        }
        let entries = model.unmodeled_source_for_case(name);
        if !entries.is_empty() {
            let descriptions: Vec<&str> = entries.iter().map(|e| e.description.as_str()).collect();
            return Err(SolverError::UnsupportedInput(format!(
                "case '{name}' depends on imported source data that is not modeled: {}. Remove the entry from the model's unmodeled source data to analyze without it",
                descriptions.join("; ")
            )));
        }
    }
    Ok(())
}

fn analysis_case_order(model: &SapModel) -> Result<Vec<String>, SolverError> {
    // Preserve the default static/modal/dynamic preference while honoring explicit
    // dependencies even when the input file lists the dependent case first.
    let mut pending: Vec<String> = model
        .load_cases
        .iter()
        .filter(|(_, lc)| lc.run_flag)
        .map(|(name, _)| name.clone())
        .collect();
    pending.sort_by_key(|name| match model.load_cases[name].case_type {
        LoadCaseType::LinearStatic | LoadCaseType::NonlinearStatic => 0,
        LoadCaseType::Modal => 1,
        _ => 2,
    });
    for name in &pending {
        if let Some(dep) = &model.load_cases[name].initial_case {
            if !model.load_cases.get(dep).is_some_and(|lc| lc.run_flag) {
                return Err(SolverError::InvalidInput(format!(
                    "case '{name}' requires missing or disabled initial case '{dep}'"
                )));
            }
            if model.load_cases[name].case_type == LoadCaseType::ResponseSpectrum
                && model.load_cases[dep].case_type != LoadCaseType::Modal
            {
                return Err(SolverError::InvalidInput(format!(
                    "spectrum case '{name}' requires a modal initial case"
                )));
            }
        }
    }
    let mut ordered = Vec::with_capacity(pending.len());
    while !pending.is_empty() {
        let ready = pending.iter().position(|name| {
            model.load_cases[name]
                .initial_case
                .as_ref()
                .is_none_or(|dep| ordered.contains(dep))
        });
        let Some(index) = ready else {
            return Err(SolverError::InvalidInput(format!(
                "cyclic initial-case dependencies: {}",
                pending.join(", ")
            )));
        };
        ordered.push(pending.remove(index));
    }
    Ok(ordered)
}

/// Expands any 4-node `AreaObj` with `auto_mesh = (n1, n2)` ($n_1 > 1$ or $n_2 > 1$) into sub-elements
/// and interpolates boundary restraints along restrained edges.
#[allow(clippy::type_complexity)]
fn expand_auto_meshed_areas(
    model: &SapModel,
) -> (
    IndexMap<String, PointObj>,
    IndexMap<String, AreaObj>,
    IndexMap<String, Vec<String>>,
) {
    let mut work_points = model.points.clone();
    let mut work_areas = IndexMap::new();
    let mut orig_area_map: IndexMap<String, Vec<String>> = IndexMap::new();

    for (aname, area) in &model.areas {
        let (n1, n2) = area.auto_mesh;
        if (n1 <= 1 && n2 <= 1) || area.points.len() != 4 {
            work_areas.insert(aname.clone(), area.clone());
            orig_area_map.insert(aname.clone(), vec![aname.clone()]);
            continue;
        }

        let n1 = n1.max(1);
        let n2 = n2.max(1);
        let p0 = match model.points.get(&area.points[0]) {
            Some(p) => p.clone(),
            None => continue,
        };
        let p1 = match model.points.get(&area.points[1]) {
            Some(p) => p.clone(),
            None => continue,
        };
        let p2 = match model.points.get(&area.points[2]) {
            Some(p) => p.clone(),
            None => continue,
        };
        let p3 = match model.points.get(&area.points[3]) {
            Some(p) => p.clone(),
            None => continue,
        };

        let mut grid_pts = vec![vec![String::new(); n2 + 1]; n1 + 1];
        for i in 0..=n1 {
            let u = (i as f64) / (n1 as f64);
            for j in 0..=n2 {
                let v = (j as f64) / (n2 as f64);
                if i == 0 && j == 0 {
                    grid_pts[i][j] = p0.name.clone();
                } else if i == n1 && j == 0 {
                    grid_pts[i][j] = p1.name.clone();
                } else if i == n1 && j == n2 {
                    grid_pts[i][j] = p2.name.clone();
                } else if i == 0 && j == n2 {
                    grid_pts[i][j] = p3.name.clone();
                } else {
                    let s0 = (1.0 - u) * (1.0 - v);
                    let s1 = u * (1.0 - v);
                    let s2 = u * v;
                    let s3 = (1.0 - u) * v;
                    let x = s0 * p0.x + s1 * p1.x + s2 * p2.x + s3 * p3.x;
                    let y = s0 * p0.y + s1 * p1.y + s2 * p2.y + s3 * p3.y;
                    let z = s0 * p0.z + s1 * p1.z + s2 * p2.z + s3 * p3.z;
                    let sub_name = format!("__mesh_{}_{}_{}", aname, i, j);
                    let mut sub_pt = PointObj::new(sub_name.clone(), x, y, z);

                    // Interpolate edge restraints: if on an edge, inherit DOFs restrained at both ends of that edge;
                    // also inherit DOFs restrained at all 4 corners (e.g. in-plane membrane DOFs on a pure plate).
                    for dof in 0..6 {
                        let all_four = p0.restraint[dof]
                            && p1.restraint[dof]
                            && p2.restraint[dof]
                            && p3.restraint[dof];
                        let edge_bot = j == 0 && p0.restraint[dof] && p1.restraint[dof];
                        let edge_top = j == n2 && p3.restraint[dof] && p2.restraint[dof];
                        let edge_left = i == 0 && p0.restraint[dof] && p3.restraint[dof];
                        let edge_right = i == n1 && p1.restraint[dof] && p2.restraint[dof];
                        sub_pt.restraint[dof] =
                            all_four || edge_bot || edge_top || edge_left || edge_right;
                    }
                    work_points.insert(sub_name.clone(), sub_pt);
                    grid_pts[i][j] = sub_name;
                }
            }
        }

        let mut sub_names = Vec::with_capacity(n1 * n2);
        for i in 0..n1 {
            for j in 0..n2 {
                let sub_aname = format!("{}__sub_{}_{}", aname, i, j);
                let mut sub_area = area.clone();
                sub_area.name = sub_aname.clone();
                sub_area.auto_mesh = (1, 1);
                sub_area.points = vec![
                    grid_pts[i][j].clone(),
                    grid_pts[i + 1][j].clone(),
                    grid_pts[i + 1][j + 1].clone(),
                    grid_pts[i][j + 1].clone(),
                ];
                work_areas.insert(sub_aname.clone(), sub_area);
                sub_names.push(sub_aname);
            }
        }
        orig_area_map.insert(aname.clone(), sub_names);
    }

    (work_points, work_areas, orig_area_map)
}

/// Assembles the unrestrained elastic structural stiffness matrix `K_struct` ($N \times N$)
/// and lumped diagonal mass vector `diag_mass` ($N$) in joint-local coordinates.
fn assemble_elastic_stiffness_and_mass(
    model: &SapModel,
    work_points: &IndexMap<String, PointObj>,
    work_areas: &IndexMap<String, AreaObj>,
    pt_index: &IndexMap<String, usize>,
    joint_rot: &[[[f64; 3]; 3]],
    ndof: usize,
) -> Result<(Vec<f64>, Vec<f64>), SolverError> {
    let mut k_struct = vec![0.0f64; ndof * ndof];
    let mut diag_mass = vec![0.0f64; ndof];

    // 1. Joint lumped masses
    for (idx, pt) in work_points.values().enumerate() {
        for d in 0..6 {
            diag_mass[6 * idx + d] += pt.mass[d];
        }
    }

    // 2. Frame elements
    for frame in model.frames.values() {
        let idx_i = *pt_index
            .get(&frame.point_i)
            .ok_or_else(|| SolverError::PointNotFound(frame.point_i.clone()))?;
        let idx_j = *pt_index
            .get(&frame.point_j)
            .ok_or_else(|| SolverError::PointNotFound(frame.point_j.clone()))?;
        let pt_i = &work_points[idx_i];
        let pt_j = &work_points[idx_j];

        let sec = model
            .frame_sections
            .get(&frame.section)
            .ok_or_else(|| SolverError::SectionNotFound(frame.section.clone()))?;
        let mat = model
            .materials
            .get(&sec.material)
            .ok_or_else(|| SolverError::MaterialNotFound(sec.material.clone()))?;

        let (length, e_axes) = frame_local_axes(
            [pt_i.x, pt_i.y, pt_i.z],
            [pt_j.x, pt_j.y, pt_j.z],
            frame.angle_deg,
        );
        let rz = frame.rigid_zone_factor.clamp(0.0, 1.0);
        let di = rz * frame.end_offset_i;
        let dj = rz * frame.end_offset_j;
        let lf = (length - di - dj).max(1e-6 * length);

        let mut k_loc = compute_flexible_local_stiffness(model, frame, sec, mat, lf);
        if frame.pdelta_force.abs() > 1e-18 {
            let kg_loc = compute_local_geometric_stiffness(frame.pdelta_force, lf);
            for r in 0..12 {
                for c in 0..12 {
                    k_loc[r][c] += kg_loc[r][c];
                }
            }
        }
        let mut dummy_fef = [0.0; 12];
        condense_releases_and_springs(&mut k_loc, &mut dummy_fef, frame);
        apply_rigid_offsets(&mut k_loc, &mut dummy_fef, di, dj);
        let insertion = frame_insertion_offset(model, frame)?;
        apply_insertion_offset(&mut k_loc, &mut dummy_fef, insertion);

        // Transform from element-local to joint-local at I and J:
        // u_elem = T_elem_joint * u_joint where T_p = e_axes * R_p
        let ti = mat3_mul(&e_axes, &joint_rot[idx_i]);
        let tj = mat3_mul(&e_axes, &joint_rot[idx_j]);
        assemble_12x12_into_global(&mut k_struct, ndof, idx_i, idx_j, &k_loc, &ti, &tj);

        // Lumped frame translational mass (50% at I, 50% at J)
        let p_mid = eval_section_props_at(model, frame, sec, mat, 0.5, length);
        let m_half = 0.5 * p_mid.mass_per_len * length;
        for d in 0..3 {
            diag_mass[6 * idx_i + d] += m_half;
            diag_mass[6 * idx_j + d] += m_half;
        }
    }

    // 3. Area elements
    for area in work_areas.values() {
        if area.points.len() < 3 {
            continue;
        }
        let sec = model
            .area_sections
            .get(&area.section)
            .ok_or_else(|| SolverError::SectionNotFound(area.section.clone()))?;
        let mat = model
            .materials
            .get(&sec.material)
            .ok_or_else(|| SolverError::MaterialNotFound(sec.material.clone()))?;

        let n_nodes = area.points.len().min(4);
        let mut node_indices = Vec::with_capacity(n_nodes);
        let mut coords = Vec::with_capacity(n_nodes);
        for pname in area.points.iter().take(n_nodes) {
            let idx = *pt_index
                .get(pname)
                .ok_or_else(|| SolverError::PointNotFound(pname.clone()))?;
            let pt = &work_points[idx];
            node_indices.push(idx);
            coords.push([pt.x, pt.y, pt.z]);
        }

        let (e_axes, _centroid, local_xy) =
            area_local_system(&coords, area.angle_deg + sec.mat_angle_deg);
        let (k_loc, elem_area) = compute_area_local_stiffness(area, sec, mat, &local_xy);

        assemble_area_into_global(
            &mut k_struct,
            ndof,
            &node_indices,
            &k_loc,
            &e_axes,
            joint_rot,
        );

        let mod_mass = sec.modifiers[8] * area.modifiers[8];
        let m_node = ((sec.thickness_membrane * mat.unit_mass * mod_mass + area.mass_per_area)
            * elem_area)
            / (n_nodes as f64);
        for &idx in &node_indices {
            for d in 0..3 {
                diag_mass[6 * idx + d] += m_node;
            }
        }
    }

    // 4. Link springs in element axes, including shear/rotation coupling.
    for link in model.links.values() {
        let prop = &model.link_props[&link.prop];
        let op = link::operator(link, prop, work_points, pt_index, joint_rot);
        link::assemble(&mut k_struct, ndof, &op, link::stiffness(prop));
        for name in std::iter::once(&link.point_i).chain(link.point_j.iter()) {
            for d in 0..3 {
                diag_mass[6 * pt_index[name] + d] += 0.5 * prop.mass;
            }
        }
    }

    // 5. 3D Solid (8-node brick) elements
    for solid in model.solids.values() {
        let sec = model
            .solid_sections
            .get(&solid.section)
            .ok_or_else(|| SolverError::SectionNotFound(solid.section.clone()))?;
        let mat = model
            .materials
            .get(&sec.material)
            .ok_or_else(|| SolverError::MaterialNotFound(sec.material.clone()))?;
        let mut node_indices = [0usize; 8];
        let mut coords = [[0.0; 3]; 8];
        for (i, pname) in solid.points.iter().enumerate() {
            let idx = *pt_index
                .get(pname)
                .ok_or_else(|| SolverError::PointNotFound(pname.clone()))?;
            let pt = &work_points[idx];
            node_indices[i] = idx;
            coords[i] = [pt.x, pt.y, pt.z];
        }
        let (k_solid_glob, m_solid_diag) = compute_solid_stiffness_and_mass(&coords, mat, sec)?;
        assemble_solid_into_joint_local(
            &mut k_struct,
            ndof,
            &node_indices,
            &k_solid_glob,
            joint_rot,
        );
        for (a, &idx) in node_indices.iter().enumerate() {
            for d in 0..3 {
                diag_mass[6 * idx + d] += m_solid_diag[3 * a + d];
            }
        }
    }

    Ok((k_struct, diag_mass))
}

fn assemble_solid_into_joint_local(
    k_global: &mut [f64],
    ndof: usize,
    node_indices: &[usize; 8],
    k_elem_glob: &[[f64; 24]; 24],
    joint_rot: &[[[f64; 3]; 3]],
) {
    for a in 0..8 {
        let idx_a = node_indices[a];
        let ra = &joint_rot[idx_a];
        for b in 0..8 {
            let idx_b = node_indices[b];
            let rb = &joint_rot[idx_b];
            // Block K_joint(a, b) = R_a^T * K_glob(a, b) * R_b
            for i in 0..3 {
                let gi = 6 * idx_a + i;
                for j in 0..3 {
                    let gj = 6 * idx_b + j;
                    let mut s = 0.0;
                    for p in 0..3 {
                        for q in 0..3 {
                            s += ra[p][i] * k_elem_glob[3 * a + p][3 * b + q] * rb[q][j];
                        }
                    }
                    k_global[gi * ndof + gj] += s;
                }
            }
        }
    }
}

pub(crate) fn assemble_12x12_into_global(
    k_global: &mut [f64],
    ndof: usize,
    idx_i: usize,
    idx_j: usize,
    k_loc: &[[f64; 12]; 12],
    ti: &[[f64; 3]; 3],
    tj: &[[f64; 3]; 3],
) {
    // Build 12x12 transformation T_12 from joint-local to element-local:
    // diag(ti, ti, tj, tj)
    let mut t12 = [[0.0f64; 12]; 12];
    for r in 0..3 {
        for c in 0..3 {
            t12[r][c] = ti[r][c];
            t12[r + 3][c + 3] = ti[r][c];
            t12[r + 6][c + 6] = tj[r][c];
            t12[r + 9][c + 9] = tj[r][c];
        }
    }

    let mut kt = [[0.0f64; 12]; 12];
    for i in 0..12 {
        for j in 0..12 {
            let mut s = 0.0;
            for m in 0..12 {
                s += k_loc[i][m] * t12[m][j];
            }
            kt[i][j] = s;
        }
    }

    let g_dofs: [usize; 12] = [
        6 * idx_i,
        6 * idx_i + 1,
        6 * idx_i + 2,
        6 * idx_i + 3,
        6 * idx_i + 4,
        6 * idx_i + 5,
        6 * idx_j,
        6 * idx_j + 1,
        6 * idx_j + 2,
        6 * idx_j + 3,
        6 * idx_j + 4,
        6 * idx_j + 5,
    ];

    for i in 0..12 {
        let gi = g_dofs[i];
        for j in 0..12 {
            let gj = g_dofs[j];
            let mut s = 0.0;
            for m in 0..12 {
                s += t12[m][i] * kt[m][j];
            }
            k_global[gi * ndof + gj] += s;
        }
    }
}

fn assemble_area_into_global(
    k_global: &mut [f64],
    ndof: usize,
    node_indices: &[usize],
    k_loc: &[f64],
    e_axes: &[[f64; 3]; 3],
    joint_rot: &[[[f64; 3]; 3]],
) {
    let n_nodes = node_indices.len();
    let m = 6 * n_nodes;
    let mut t_mat = vec![0.0f64; m * m];
    for (a, &pt_idx) in node_indices.iter().enumerate() {
        let ta = mat3_mul(e_axes, &joint_rot[pt_idx]);
        for r in 0..3 {
            for c in 0..3 {
                t_mat[(6 * a + r) * m + (6 * a + c)] = ta[r][c];
                t_mat[(6 * a + 3 + r) * m + (6 * a + 3 + c)] = ta[r][c];
            }
        }
    }

    let mut kt = vec![0.0f64; m * m];
    for i in 0..m {
        for j in 0..m {
            let mut s = 0.0;
            for p in 0..m {
                s += k_loc[i * m + p] * t_mat[p * m + j];
            }
            kt[i * m + j] = s;
        }
    }

    let mut g_dofs = Vec::with_capacity(m);
    for &idx in node_indices {
        for d in 0..6 {
            g_dofs.push(6 * idx + d);
        }
    }

    for i in 0..m {
        let gi = g_dofs[i];
        for j in 0..m {
            let gj = g_dofs[j];
            let mut s = 0.0;
            for p in 0..m {
                s += t_mat[p * m + i] * kt[p * m + j];
            }
            k_global[gi * ndof + gj] += s;
        }
    }
}

/// Returns the 6x6 grounded spring stiffness matrix of `pt` expressed in the joint-local
/// DOF basis `[U1, U2, U3, R1, R2, R3]` (`v_global = R_p * v_local`).
///
/// - `pt.spring_coupled` (`[f64; 21]`) is unpacked in column-major upper-triangular order:
///   `[U1U1, U1U2, U2U2, U1U3, U2U3, U3U3, ...]`, matching SAP2000 `.s2k`/`.sdb`.
/// - When `!pt.spring_is_local`, `K_global` is transformed into joint-local DOFs via
///   `K_local = R_6^T * K_global * R_6` where `R_6 = diag(R_p, R_p)`.
fn joint_spring_matrix_local(pt: &PointObj, r_p: &[[f64; 3]; 3]) -> Option<[[f64; 6]; 6]> {
    let ks = if let Some(coupled) = &pt.spring_coupled {
        let mut ks6 = [[0.0f64; 6]; 6];
        let mut pos = 0;
        for c in 0..6 {
            for r in 0..=c {
                ks6[r][c] = coupled[pos];
                ks6[c][r] = coupled[pos];
                pos += 1;
            }
        }
        ks6
    } else if pt.spring.iter().any(|&s| s > 0.0) {
        let mut ks6 = [[0.0f64; 6]; 6];
        for d in 0..6 {
            if pt.spring[d] > 0.0 {
                ks6[d][d] = pt.spring[d];
            }
        }
        ks6
    } else {
        return None;
    };

    if pt.spring_is_local {
        return Some(ks);
    }

    let mut r6 = [[0.0f64; 6]; 6];
    for a in 0..3 {
        for b in 0..3 {
            r6[a][b] = r_p[a][b];
            r6[a + 3][b + 3] = r_p[a][b];
        }
    }
    let mut tmp = [[0.0f64; 6]; 6];
    for m in 0..6 {
        for j in 0..6 {
            let mut s = 0.0;
            for n in 0..6 {
                s += ks[m][n] * r6[n][j];
            }
            tmp[m][j] = s;
        }
    }
    let mut k_loc = [[0.0f64; 6]; 6];
    for i in 0..6 {
        for j in 0..6 {
            let mut s = 0.0;
            for m in 0..6 {
                s += r6[m][i] * tmp[m][j];
            }
            k_loc[i][j] = s;
        }
    }
    Some(k_loc)
}

/// Adds grounded joint springs (`pt.spring`, `pt.spring_coupled`) and multipoint constraints (`model.constraints`)
/// to `k_struct`.
fn add_springs_and_constraints(
    model: &SapModel,
    work_points: &IndexMap<String, PointObj>,
    _pt_index: &IndexMap<String, usize>,
    joint_rot: &[[[f64; 3]; 3]],
    k_struct: &[f64],
    ndof: usize,
) -> Vec<f64> {
    let mut k = k_struct.to_vec();

    // 1. Grounded joint springs
    for (idx, pt) in work_points.values().enumerate() {
        if let Some(ks_loc) = joint_spring_matrix_local(pt, &joint_rot[idx]) {
            for r in 0..6 {
                for c in 0..6 {
                    k[(6 * idx + r) * ndof + (6 * idx + c)] += ks_loc[r][c];
                }
            }
        }
    }

    // 2. Multipoint constraints (Equal, Diaphragm, Body, Plate, Rod, Beam, Weld)
    let mut max_diag = 1e6f64;
    for i in 0..ndof {
        max_diag = max_diag.max(k[i * ndof + i].abs());
    }
    let penalty = max_diag * 1e6;

    for (cname, cdef) in &model.constraints {
        let mut constrained_pts: Vec<usize> = Vec::new();
        for (p_idx, pt) in work_points.values().enumerate() {
            if pt.constraints.iter().any(|c| c == cname) {
                constrained_pts.push(p_idx);
            }
        }
        if constrained_pts.len() < 2 {
            continue;
        }
        let master = constrained_pts[0];
        let pm = &work_points[master];

        for &slave in &constrained_pts[1..] {
            let ps = &work_points[slave];
            let dx = ps.x - pm.x;
            let dy = ps.y - pm.y;
            let dz = ps.z - pm.z;

            match cdef.constraint_type {
                ConstraintType::Equal | ConstraintType::Local => {
                    for d in 0..6 {
                        if cdef.dofs[d] {
                            add_penalty_constraint(
                                &mut k,
                                ndof,
                                &[(6 * slave + d, 1.0), (6 * master + d, -1.0)],
                                penalty,
                            );
                        }
                    }
                }
                ConstraintType::Diaphragm => {
                    // Rigid in plane perpendicular to cdef.axis (default Z -> XY plane: UX, UY, RZ)
                    match cdef.axis {
                        ConstraintAxis::Z | ConstraintAxis::AutoAxis => {
                            // u_xS - u_xM + dy * r_zM = 0
                            add_penalty_constraint(
                                &mut k,
                                ndof,
                                &[(6 * slave, 1.0), (6 * master, -1.0), (6 * master + 5, dy)],
                                penalty,
                            );
                            // u_yS - u_yM - dx * r_zM = 0
                            add_penalty_constraint(
                                &mut k,
                                ndof,
                                &[
                                    (6 * slave + 1, 1.0),
                                    (6 * master + 1, -1.0),
                                    (6 * master + 5, -dx),
                                ],
                                penalty,
                            );
                            // r_zS - r_zM = 0
                            add_penalty_constraint(
                                &mut k,
                                ndof,
                                &[(6 * slave + 5, 1.0), (6 * master + 5, -1.0)],
                                penalty,
                            );
                        }
                        ConstraintAxis::X => {
                            // Rigid in YZ plane: UY, UZ, RX
                            // u_yS - u_yM + dz * r_xM = 0
                            add_penalty_constraint(
                                &mut k,
                                ndof,
                                &[
                                    (6 * slave + 1, 1.0),
                                    (6 * master + 1, -1.0),
                                    (6 * master + 3, dz),
                                ],
                                penalty,
                            );
                            // u_zS - u_zM - dy * r_xM = 0
                            add_penalty_constraint(
                                &mut k,
                                ndof,
                                &[
                                    (6 * slave + 2, 1.0),
                                    (6 * master + 2, -1.0),
                                    (6 * master + 3, -dy),
                                ],
                                penalty,
                            );
                            // r_xS - r_xM = 0
                            add_penalty_constraint(
                                &mut k,
                                ndof,
                                &[(6 * slave + 3, 1.0), (6 * master + 3, -1.0)],
                                penalty,
                            );
                        }
                        ConstraintAxis::Y => {
                            // Rigid in XZ plane: UX, UZ, RY
                            // u_xS - u_xM - dz * r_yM = 0
                            add_penalty_constraint(
                                &mut k,
                                ndof,
                                &[(6 * slave, 1.0), (6 * master, -1.0), (6 * master + 4, -dz)],
                                penalty,
                            );
                            // u_zS - u_zM + dx * r_yM = 0
                            add_penalty_constraint(
                                &mut k,
                                ndof,
                                &[
                                    (6 * slave + 2, 1.0),
                                    (6 * master + 2, -1.0),
                                    (6 * master + 4, dx),
                                ],
                                penalty,
                            );
                            // r_yS - r_yM = 0
                            add_penalty_constraint(
                                &mut k,
                                ndof,
                                &[(6 * slave + 4, 1.0), (6 * master + 4, -1.0)],
                                penalty,
                            );
                        }
                    }
                }
                ConstraintType::Body | ConstraintType::Weld => {
                    // 3D rigid body: u_S = u_M + theta_M x r_{MS}
                    // u_xS - u_xM - dz * r_yM + dy * r_zM = 0
                    if cdef.dofs[0] {
                        add_penalty_constraint(
                            &mut k,
                            ndof,
                            &[
                                (6 * slave, 1.0),
                                (6 * master, -1.0),
                                (6 * master + 4, -dz),
                                (6 * master + 5, dy),
                            ],
                            penalty,
                        );
                    }
                    // u_yS - u_yM + dz * r_xM - dx * r_zM = 0
                    if cdef.dofs[1] {
                        add_penalty_constraint(
                            &mut k,
                            ndof,
                            &[
                                (6 * slave + 1, 1.0),
                                (6 * master + 1, -1.0),
                                (6 * master + 3, dz),
                                (6 * master + 5, -dx),
                            ],
                            penalty,
                        );
                    }
                    // u_zS - u_zM - dy * r_xM + dx * r_yM = 0
                    if cdef.dofs[2] {
                        add_penalty_constraint(
                            &mut k,
                            ndof,
                            &[
                                (6 * slave + 2, 1.0),
                                (6 * master + 2, -1.0),
                                (6 * master + 3, -dy),
                                (6 * master + 4, dx),
                            ],
                            penalty,
                        );
                    }
                    for d in 3..6 {
                        if cdef.dofs[d] {
                            add_penalty_constraint(
                                &mut k,
                                ndof,
                                &[(6 * slave + d, 1.0), (6 * master + d, -1.0)],
                                penalty,
                            );
                        }
                    }
                }
                ConstraintType::Plate => {
                    // Plate bending in XY plane: UZ, RX, RY
                    add_penalty_constraint(
                        &mut k,
                        ndof,
                        &[
                            (6 * slave + 2, 1.0),
                            (6 * master + 2, -1.0),
                            (6 * master + 3, -dy),
                            (6 * master + 4, dx),
                        ],
                        penalty,
                    );
                    add_penalty_constraint(
                        &mut k,
                        ndof,
                        &[(6 * slave + 3, 1.0), (6 * master + 3, -1.0)],
                        penalty,
                    );
                    add_penalty_constraint(
                        &mut k,
                        ndof,
                        &[(6 * slave + 4, 1.0), (6 * master + 4, -1.0)],
                        penalty,
                    );
                }
                _ => {
                    for d in 0..6 {
                        if cdef.dofs[d] {
                            add_penalty_constraint(
                                &mut k,
                                ndof,
                                &[(6 * slave + d, 1.0), (6 * master + d, -1.0)],
                                penalty,
                            );
                        }
                    }
                }
            }
        }
    }

    k
}

fn add_penalty_constraint(k: &mut [f64], ndof: usize, terms: &[(usize, f64)], penalty: f64) {
    let norm_sq: f64 = terms.iter().map(|(_, c)| c * c).sum();
    if norm_sq <= 1e-30 {
        return;
    }
    let scale = penalty / norm_sq;
    for &(i, ci) in terms {
        for &(j, cj) in terms {
            k[i * ndof + j] += scale * ci * cj;
        }
    }
}

/// Assembles the global geometric stiffness matrix $\mathbf{K}_G$ ($N \times N$) given a map of frame axial forces.
///
/// Note: When a frame has end releases or partial fixity springs, condensation is nonlinear in the
/// stiffness matrix ($\text{cond}(K_e + K_G) \neq \text{cond}(K_e) + \text{cond}(K_G)$). Therefore we
/// condense $K_e + K_G$ and subtract condensed $K_e$ so that $K_e^{\text{cond}} + K_G^{\text{cond}} = (K_e + K_G)^{\text{cond}}$.
fn assemble_global_geometric_stiffness(
    model: &SapModel,
    work_points: &IndexMap<String, PointObj>,
    pt_index: &IndexMap<String, usize>,
    joint_rot: &[[[f64; 3]; 3]],
    frame_axial: &IndexMap<String, f64>,
    subtract_initial_pdelta: bool,
    ndof: usize,
) -> Result<Vec<f64>, SolverError> {
    let mut kg_global = vec![0.0f64; ndof * ndof];
    for (fname, frame) in &model.frames {
        let p_target = frame_axial
            .get(fname)
            .copied()
            .unwrap_or(frame.pdelta_force);
        let p_base = if subtract_initial_pdelta {
            frame.pdelta_force
        } else {
            0.0
        };
        let p_delta = p_target - p_base;
        if p_delta.abs() <= 1e-18 {
            continue;
        }
        let idx_i = *pt_index
            .get(&frame.point_i)
            .ok_or_else(|| SolverError::PointNotFound(frame.point_i.clone()))?;
        let idx_j = *pt_index
            .get(&frame.point_j)
            .ok_or_else(|| SolverError::PointNotFound(frame.point_j.clone()))?;
        let pt_i = &work_points[idx_i];
        let pt_j = &work_points[idx_j];

        let sec = model
            .frame_sections
            .get(&frame.section)
            .ok_or_else(|| SolverError::SectionNotFound(frame.section.clone()))?;
        let mat = model
            .materials
            .get(&sec.material)
            .ok_or_else(|| SolverError::MaterialNotFound(sec.material.clone()))?;

        let (length, e_axes) = frame_local_axes(
            [pt_i.x, pt_i.y, pt_i.z],
            [pt_j.x, pt_j.y, pt_j.z],
            frame.angle_deg,
        );
        let rz = frame.rigid_zone_factor.clamp(0.0, 1.0);
        let di = rz * frame.end_offset_i;
        let dj = rz * frame.end_offset_j;
        let lf = (length - di - dj).max(1e-6 * length);

        let has_release = frame.release_i.iter().any(|&r| r) || frame.release_j.iter().any(|&r| r);
        let mut kg_eff = if has_release {
            let ke_raw = compute_flexible_local_stiffness(model, frame, sec, mat, lf);
            let mut k_base_loc = ke_raw;
            if p_base.abs() > 1e-18 {
                let kg_b = compute_local_geometric_stiffness(p_base, lf);
                for r in 0..12 {
                    for c in 0..12 {
                        k_base_loc[r][c] += kg_b[r][c];
                    }
                }
            }
            let mut k_tot_loc = ke_raw;
            if p_target.abs() > 1e-18 {
                let kg_t = compute_local_geometric_stiffness(p_target, lf);
                for r in 0..12 {
                    for c in 0..12 {
                        k_tot_loc[r][c] += kg_t[r][c];
                    }
                }
            }
            let mut dummy1 = [0.0; 12];
            let mut dummy2 = [0.0; 12];
            condense_releases_and_springs(&mut k_base_loc, &mut dummy1, frame);
            condense_releases_and_springs(&mut k_tot_loc, &mut dummy2, frame);
            let mut diff = [[0.0f64; 12]; 12];
            for r in 0..12 {
                for c in 0..12 {
                    diff[r][c] = k_tot_loc[r][c] - k_base_loc[r][c];
                }
            }
            diff
        } else {
            compute_local_geometric_stiffness(p_delta, lf)
        };

        let mut dummy_fef = [0.0; 12];
        apply_rigid_offsets(&mut kg_eff, &mut dummy_fef, di, dj);

        let ti = mat3_mul(&e_axes, &joint_rot[idx_i]);
        let tj = mat3_mul(&e_axes, &joint_rot[idx_j]);
        assemble_12x12_into_global(&mut kg_global, ndof, idx_i, idx_j, &kg_eff, &ti, &tj);
    }
    Ok(kg_global)
}

/// Runs a Linear Static analysis case (`LoadCaseType::LinearStatic`).
#[allow(clippy::too_many_arguments)]
fn run_linear_static_case(
    model: &SapModel,
    lc: &LoadCase,
    work_points: &IndexMap<String, PointObj>,
    work_areas: &IndexMap<String, AreaObj>,
    orig_area_map: &IndexMap<String, Vec<String>>,
    pt_index: &IndexMap<String, usize>,
    joint_rot: &[[[f64; 3]; 3]],
    k_struct: &[f64],
    k_base: &[f64],
    ndof: usize,
) -> Result<CaseResults, SolverError> {
    // Check if an initial_case (e.g. P-Delta preload) or NLGeom P-Delta is specified
    let mut k_case = k_base.to_vec();
    let mut k_struct_case = k_struct.to_vec();
    let mut initial_axial: IndexMap<String, f64> = IndexMap::new();

    if let Some(init_name) = &lc.initial_case {
        if let Some(init_res) = model.case_results.get(init_name) {
            for (fname, stations) in &init_res.frame_forces {
                if !stations.is_empty() {
                    let p_avg = 0.5 * (stations.first().unwrap().p + stations.last().unwrap().p);
                    initial_axial.insert(fname.clone(), p_avg);
                }
            }
        }
    }

    if !initial_axial.is_empty() {
        let kg_init = assemble_global_geometric_stiffness(
            model,
            work_points,
            pt_index,
            joint_rot,
            &initial_axial,
            true,
            ndof,
        )?;
        for i in 0..(ndof * ndof) {
            k_case[i] += kg_init[i];
            k_struct_case[i] += kg_init[i];
        }
    }

    let case_loads: Vec<(String, f64)> = lc
        .loads
        .iter()
        .map(|a| (a.load_name.clone(), a.scale_factor))
        .collect();

    solve_static_with_stiffness(
        model,
        &lc.name,
        &case_loads,
        work_points,
        work_areas,
        orig_area_map,
        pt_index,
        joint_rot,
        &k_struct_case,
        &k_case,
        &initial_axial,
        None,
        ndof,
    )
}

fn accumulated_nonlinear_static_loads(model: &SapModel, lc: &LoadCase) -> Vec<(String, f64)> {
    let mut chain = Vec::new();
    let mut cur = Some(lc);
    let mut visited = std::collections::BTreeSet::new();
    while let Some(c) = cur {
        if !visited.insert(c.name.clone()) {
            break;
        }
        chain.push(c);
        cur = c
            .initial_case
            .as_ref()
            .and_then(|name| model.load_cases.get(name))
            .filter(|prev| {
                matches!(
                    prev.case_type,
                    LoadCaseType::NonlinearStatic
                        | LoadCaseType::NonlinearHistory
                        | LoadCaseType::NonlinearDynamic
                        | LoadCaseType::LinearDynamic
                )
            });
    }
    chain.reverse();
    let mut loads = Vec::new();
    for c in chain {
        if matches!(
            c.case_type,
            LoadCaseType::NonlinearHistory
                | LoadCaseType::NonlinearDynamic
                | LoadCaseType::LinearDynamic
        ) {
            let t_end = (c.num_time_steps as f64) * c.time_step_dt;
            for (func_name, pat_or_acc, scale_factor) in &c.history_loads {
                let upper = pat_or_acc.trim().to_ascii_uppercase();
                if !matches!(
                    upper.as_str(),
                    "U1" | "UX" | "X" | "U2" | "UY" | "Y" | "U3" | "UZ" | "Z"
                ) {
                    let f_end = eval_dynamic_function(model, func_name, t_end);
                    let sf = scale_factor * f_end;
                    if sf.abs() > 1e-18 {
                        loads.push((pat_or_acc.clone(), sf));
                    }
                }
            }
        } else {
            // A completed displacement-controlled case applied its pattern at
            // the load factor it reached, not at the nominal scale factor.
            let factor = c
                .displacement_control
                .as_ref()
                .and_then(|_| model.case_results.get(&c.name))
                .and_then(|result| result.displacement_control_factor)
                .unwrap_or(1.0);
            for a in &c.loads {
                loads.push((a.load_name.clone(), factor * a.scale_factor));
            }
        }
    }
    loads
}

/// Loads already on the structure when a nonlinear static case starts.
fn initial_chain_loads(model: &SapModel, lc: &LoadCase) -> Vec<(String, f64)> {
    lc.initial_case
        .as_ref()
        .and_then(|name| model.load_cases.get(name))
        .filter(|prev| {
            matches!(
                prev.case_type,
                LoadCaseType::NonlinearStatic
                    | LoadCaseType::NonlinearHistory
                    | LoadCaseType::NonlinearDynamic
                    | LoadCaseType::LinearDynamic
            )
        })
        .map_or_else(Vec::new, |prev| accumulated_nonlinear_static_loads(model, prev))
}

/// Final load factor of a path-dependent or displacement-controlled nonlinear
/// static case; see `nonlinear::System::load_path_factor`.
#[allow(clippy::too_many_arguments)]
fn nonlinear_static_load_factor(
    model: &SapModel,
    lc: &LoadCase,
    work_points: &IndexMap<String, PointObj>,
    work_areas: &IndexMap<String, AreaObj>,
    pt_index: &IndexMap<String, usize>,
    joint_rot: &[[[f64; 3]; 3]],
    k_total: &[f64],
    ndof: usize,
) -> Result<f64, SolverError> {
    let own_loads: Vec<(String, f64)> = lc
        .loads
        .iter()
        .map(|a| (a.load_name.clone(), a.scale_factor))
        .collect();
    let no_axial = IndexMap::new();
    let assemble = |loads: &[(String, f64)]| {
        assemble_static_loads(
            model,
            loads,
            work_points,
            work_areas,
            pt_index,
            joint_rot,
            &no_axial,
            ndof,
        )
    };
    let prev = assemble(&initial_chain_loads(model, lc))?;
    let own = assemble(&own_loads)?;
    let fixed: Vec<bool> = prev
        .is_fixed
        .iter()
        .zip(&own.is_fixed)
        .map(|(a, b)| *a || *b)
        .collect();
    let initial_hysteresis: &[sap_model::LinkHysteresisState] = lc
        .initial_case
        .as_ref()
        .and_then(|name| model.case_results.get(name))
        .map_or(&[], |result| result.link_hysteresis.as_slice());
    let system = nonlinear::System::new_with_frame_p0(
        model,
        work_points,
        pt_index,
        joint_rot,
        k_total,
        &IndexMap::new(),
        initial_hysteresis,
        ndof,
    )?;
    let monitor = match &lc.displacement_control {
        Some(control) => {
            let (point, dof) = control.monitored.as_ref().ok_or_else(|| {
                SolverError::UnsupportedInput(format!(
                    "case '{}' uses displacement control without a representable monitored joint displacement",
                    lc.name
                ))
            })?;
            let index = *pt_index
                .get(point)
                .ok_or_else(|| SolverError::PointNotFound(point.clone()))?;
            Some((6 * index + dof, control.target))
        }
        None => None,
    };
    let u_init = initial_displacement_vector(model, lc, work_points, ndof);
    system.load_path_factor(
        &nonlinear::StageLoads {
            rhs_prev: &prev.rhs,
            rhs_own: &own.rhs,
            u_prev: &prev.u_fixed_val,
            u_own: &own.u_fixed_val,
            fixed: &fixed,
        },
        u_init.as_deref(),
        lc,
        monitor,
    )
}

fn initial_displacement_vector(
    model: &SapModel,
    lc: &LoadCase,
    work_points: &IndexMap<String, PointObj>,
    ndof: usize,
) -> Option<Vec<f64>> {
    let init_name = lc.initial_case.as_ref()?;
    let init_res = model.case_results.get(init_name)?;
    if let Some(ts) = &init_res.transient_state {
        if ts.u.len() == ndof
            && ts.point_order.len() == work_points.len()
            && ts
                .point_order
                .iter()
                .zip(work_points.keys())
                .all(|(a, b)| a == b)
        {
            return Some(ts.u.clone());
        }
        let mut u = vec![0.0; ndof];
        let ts_idx: IndexMap<&str, usize> = ts
            .point_order
            .iter()
            .enumerate()
            .map(|(i, s)| (s.as_str(), i))
            .collect();
        for (idx, pname) in work_points.keys().enumerate() {
            if let Some(&src_i) = ts_idx.get(pname.as_str()) {
                if 6 * src_i + 6 <= ts.u.len() {
                    u[6 * idx..6 * idx + 6].copy_from_slice(&ts.u[6 * src_i..6 * src_i + 6]);
                }
            }
        }
        return Some(u);
    }
    let mut u = vec![0.0; ndof];
    for (idx, pname) in work_points.keys().enumerate() {
        let d_opt = init_res
            .time_history_disp
            .get(pname)
            .and_then(|trace| trace.last())
            .or_else(|| init_res.joint_displacements.get(pname));
        if let Some(d) = d_opt {
            for k in 0..6 {
                u[6 * idx + k] = d[k];
            }
        }
    }
    Some(u)
}

#[allow(clippy::too_many_arguments)]
fn run_cable_nonlinear_static_case(
    model: &SapModel,
    lc: &LoadCase,
    case_loads: &[(String, f64)],
    work_points: &IndexMap<String, PointObj>,
    pt_index: &IndexMap<String, usize>,
    joint_rot: &[[[f64; 3]; 3]],
) -> Result<CaseResults, SolverError> {
    let num_pts = work_points.len();
    let mut restraints = vec![[false; 3]; num_pts];
    let mut nodal_loads = vec![[0.0; 3]; num_pts];
    let mut u_init = vec![[0.0; 3]; num_pts];
    let mut prescribed = vec![[false; 3]; num_pts];

    if let Some(init_name) = &lc.initial_case {
        if let Some(init_res) = model.case_results.get(init_name) {
            for (idx, pname) in work_points.keys().enumerate() {
                if let Some(d) = init_res.joint_displacements.get(pname) {
                    u_init[idx] = [d[0], d[1], d[2]];
                }
            }
        }
    }

    for (idx, pt) in work_points.values().enumerate() {
        for d in 0..3 {
            restraints[idx][d] = !model.active_dofs[d] || pt.restraint[d];
            if restraints[idx][d] {
                u_init[idx][d] = 0.0;
            }
        }
        let r_p = &joint_rot[idx];
        for (pat_name, sf) in case_loads {
            if sf.abs() <= 1e-18 {
                continue;
            }
            for lf in &pt.loads_force {
                if &lf.load_pat != pat_name {
                    continue;
                }
                let f_glob = if lf.csys.eq_ignore_ascii_case("LOCAL") {
                    [
                        r_p[0][0] * lf.force[0] + r_p[0][1] * lf.force[1] + r_p[0][2] * lf.force[2],
                        r_p[1][0] * lf.force[0] + r_p[1][1] * lf.force[1] + r_p[1][2] * lf.force[2],
                        r_p[2][0] * lf.force[0] + r_p[2][1] * lf.force[1] + r_p[2][2] * lf.force[2],
                    ]
                } else {
                    [lf.force[0], lf.force[1], lf.force[2]]
                };
                for d in 0..3 {
                    nodal_loads[idx][d] += f_glob[d] * sf;
                }
            }
            for ld in &pt.loads_displ {
                if &ld.load_pat != pat_name {
                    continue;
                }
                let u_glob = if ld.csys.eq_ignore_ascii_case("LOCAL") {
                    [
                        r_p[0][0] * ld.displ[0] + r_p[0][1] * ld.displ[1] + r_p[0][2] * ld.displ[2],
                        r_p[1][0] * ld.displ[0] + r_p[1][1] * ld.displ[1] + r_p[1][2] * ld.displ[2],
                        r_p[2][0] * ld.displ[0] + r_p[2][1] * ld.displ[1] + r_p[2][2] * ld.displ[2],
                    ]
                } else {
                    [ld.displ[0], ld.displ[1], ld.displ[2]]
                };
                for d in 0..3 {
                    if u_glob[d].abs() > 1e-18 {
                        restraints[idx][d] = true;
                        prescribed[idx][d] = true;
                        u_init[idx][d] += u_glob[d] * sf;
                    }
                }
            }
        }
    }

    let mut cable_elems = Vec::new();
    let mut cable_names = Vec::new();
    for (fname, frame) in &model.frames {
        let idx_i = pt_index[&frame.point_i];
        let idx_j = pt_index[&frame.point_j];
        let pt_i = &work_points[idx_i];
        let pt_j = &work_points[idx_j];
        let sec = model
            .frame_sections
            .get(&frame.section)
            .ok_or_else(|| SolverError::SectionNotFound(frame.section.clone()))?;
        let mat = model
            .materials
            .get(&sec.material)
            .ok_or_else(|| SolverError::MaterialNotFound(sec.material.clone()))?;
        let x_i = [pt_i.x, pt_i.y, pt_i.z];
        let x_j = [pt_j.x, pt_j.y, pt_j.z];
        let (chord_len, e_axes) = frame_local_axes(x_i, x_j, frame.angle_deg);

        let area = sec.area * sec.modifiers[0] * frame.modifiers[0];
        let e = mat.e;
        let alpha = mat.a;
        let weight_per_len = mat.unit_weight * sec.area * sec.modifiers[7] * frame.modifiers[7];

        let mut w_vec = [0.0; 3];
        let mut delta_t = 0.0;
        for (pat_name, sf) in case_loads {
            let sw_mult = model
                .load_patterns
                .get(pat_name)
                .map(|lp| lp.self_wt_multiplier)
                .unwrap_or(0.0);
            w_vec[2] -= sw_mult * sf * weight_per_len;
            if let Some(g) = frame.loads_gravity.get(pat_name) {
                for d in 0..3 {
                    w_vec[d] += g[d] * weight_per_len * sf;
                }
            }
            for ld in &frame.loads_dist {
                if &ld.load_pat == pat_name && ld.my_type == 1 {
                    let val = 0.5 * (ld.val1 + ld.val2) * sf;
                    let loc_v = resolve_load_dir_to_local(ld.dir, val, &e_axes, true);
                    for g in 0..3 {
                        w_vec[g] += e_axes[0][g] * loc_v[0]
                            + e_axes[1][g] * loc_v[1]
                            + e_axes[2][g] * loc_v[2];
                    }
                }
            }
            for lt in &frame.loads_temp {
                if &lt.load_pat == pat_name && lt.my_type == 1 {
                    delta_t += lt.val * sf;
                }
            }
        }

        let elem = if let Some(cp) = &frame.cable_params {
            if let Some(l0) = cp.undeformed_length {
                cable::CableElement::from_undeformed_length(
                    x_i, x_j, area, e, alpha, l0, w_vec, delta_t,
                )?
            } else if let Some(target_ti) = cp.target_tension_i {
                let chord = [x_j[0] - x_i[0], x_j[1] - x_i[1], x_j[2] - x_i[2]];
                let shaped = cable::CableElement::from_target_tension_i(
                    chord,
                    e * area,
                    target_ti,
                    weight_per_len,
                )?;
                cable::CableElement::from_undeformed_length(
                    x_i, x_j, area, e, alpha, shaped.l0, w_vec, delta_t,
                )?
            } else if let Some(h_target) = cp.horizontal_tension {
                cable::CableElement::from_horizontal_tension(
                    x_i, x_j, area, e, alpha, h_target, w_vec, delta_t,
                )?
            } else {
                cable::CableElement::from_undeformed_length(
                    x_i, x_j, area, e, alpha, chord_len, w_vec, delta_t,
                )?
            }
        } else {
            cable::CableElement::from_undeformed_length(
                x_i, x_j, area, e, alpha, chord_len, w_vec, delta_t,
            )?
        };
        cable_elems.push((idx_i, idx_j, elem));
        cable_names.push(fname.clone());
    }

    let (u_nodes, cable_states) = cable::solve_cable_network(
        num_pts,
        &cable_elems,
        &restraints,
        &nodal_loads,
        Some(&u_init),
        lc.max_iterations,
        lc.convergence_tol,
    )?;

    let mut joint_displacements = IndexMap::new();
    let mut joint_reactions = IndexMap::new();
    let mut base_reactions = [0.0f64; 6];
    let mut f_int_global = vec![[0.0f64; 3]; num_pts];

    for ((idx_i, idx_j, _), st) in cable_elems.iter().zip(&cable_states) {
        for d in 0..3 {
            f_int_global[*idx_i][d] += st.f_i[d];
            f_int_global[*idx_j][d] += st.f_j[d];
        }
    }

    for (idx, (pname, pt)) in work_points.iter().enumerate() {
        let r_p_t = mat3_transpose(&joint_rot[idx]);
        let ug = u_nodes[idx];
        let u_loc = [
            r_p_t[0][0] * ug[0] + r_p_t[0][1] * ug[1] + r_p_t[0][2] * ug[2],
            r_p_t[1][0] * ug[0] + r_p_t[1][1] * ug[1] + r_p_t[1][2] * ug[2],
            r_p_t[2][0] * ug[0] + r_p_t[2][1] * ug[1] + r_p_t[2][2] * ug[2],
            0.0,
            0.0,
            0.0,
        ];
        joint_displacements.insert(pname.clone(), u_loc);

        if pt.restraint.iter().any(|&r| r) || restraints[idx].iter().any(|&r| r) {
            let mut rg = [0.0; 3];
            for d in 0..3 {
                if pt.restraint[d] || restraints[idx][d] {
                    rg[d] = f_int_global[idx][d] - nodal_loads[idx][d];
                }
            }
            let reac = [
                r_p_t[0][0] * rg[0] + r_p_t[0][1] * rg[1] + r_p_t[0][2] * rg[2],
                r_p_t[1][0] * rg[0] + r_p_t[1][1] * rg[1] + r_p_t[1][2] * rg[2],
                r_p_t[2][0] * rg[0] + r_p_t[2][1] * rg[1] + r_p_t[2][2] * rg[2],
                0.0,
                0.0,
                0.0,
            ];
            if pt.restraint.iter().any(|&r| r) || prescribed[idx].iter().any(|r| *r) {
                joint_reactions.insert(pname.clone(), reac);
                for g in 0..3 {
                    base_reactions[g] += rg[g];
                }
                base_reactions[3] += pt.y * rg[2] - pt.z * rg[1];
                base_reactions[4] += pt.z * rg[0] - pt.x * rg[2];
                base_reactions[5] += pt.x * rg[1] - pt.y * rg[0];
            }
        }
    }

    let mut frame_forces = IndexMap::new();
    for ((fname, (_, _, elem)), st) in cable_names
        .iter()
        .zip(cable_elems.iter())
        .zip(&cable_states)
    {
        frame_forces.insert(
            fname.clone(),
            vec![
                FrameStationResult {
                    station: 0.0,
                    rel_dist: 0.0,
                    p: st.tension_i,
                    ..Default::default()
                },
                FrameStationResult {
                    station: elem.l0,
                    rel_dist: 1.0,
                    p: st.tension_j,
                    ..Default::default()
                },
            ],
        );
    }

    Ok(CaseResults {
        case_name: lc.name.clone(),
        joint_displacements,
        joint_reactions,
        frame_forces,
        base_reactions,
        ..Default::default()
    })
}

/// Runs a Nonlinear Static analysis case (`LoadCaseType::NonlinearStatic` with P-Delta, Cables, Links, or T/C limits).
#[allow(clippy::too_many_arguments)]
fn run_nonlinear_static_case(
    model: &SapModel,
    lc: &LoadCase,
    work_points: &IndexMap<String, PointObj>,
    work_areas: &IndexMap<String, AreaObj>,
    orig_area_map: &IndexMap<String, Vec<String>>,
    pt_index: &IndexMap<String, usize>,
    joint_rot: &[[[f64; 3]; 3]],
    k_struct: &[f64],
    k_base: &[f64],
    ndof: usize,
) -> Result<CaseResults, SolverError> {
    let case_loads = accumulated_nonlinear_static_loads(model, lc);

    if model.frames.values().any(|f| f.cable_params.is_some()) {
        return run_cable_nonlinear_static_case(
            model,
            lc,
            &case_loads,
            work_points,
            pt_index,
            joint_rot,
        );
    }

    let has_nl_links = model.links.values().any(|object| {
        let prop = &model.link_props[&object.prop];
        (0..6).any(|d| prop.dof_nonlinear[d] && prop.dof_active[d] && !prop.dof_fixed[d])
    });
    let has_tc_frames = model
        .frames
        .values()
        .any(|f| f.tc_limits.is_some_and(|(c, _, t, _)| c || t));

    let has_hysteretic_links = model.links.values().any(|object| {
        let prop = &model.link_props[&object.prop];
        prop.link_type == sap_model::LinkPropType::PlasticWen
            && (0..6).any(|d| prop.dof_nonlinear[d] && prop.dof_active[d] && !prop.dof_fixed[d])
    });

    if has_nl_links || has_tc_frames || lc.displacement_control.is_some() {
        // Assemble the linear part without nonlinear effective springs from the
        // outset. Subtracting a large ke afterward can lose small true stiffness
        // terms through cancellation; ke must not influence nonlinear response.
        let mut constitutive_model = model.clone();
        for prop in constitutive_model.link_props.values_mut() {
            for d in 0..6 {
                if prop.dof_nonlinear[d] && prop.dof_active[d] && !prop.dof_fixed[d] {
                    prop.ke[d] = 0.;
                }
            }
        }
        let (linear_struct, _) = assemble_elastic_stiffness_and_mass(
            &constitutive_model,
            work_points,
            work_areas,
            pt_index,
            joint_rot,
            ndof,
        )?;
        let linear_base = add_springs_and_constraints(
            &constitutive_model,
            work_points,
            pt_index,
            joint_rot,
            &linear_struct,
            ndof,
        );
        let mut case_loads = case_loads;
        let mut reached_factor = None;
        if has_hysteretic_links || lc.displacement_control.is_some() {
            let factor = nonlinear_static_load_factor(
                &constitutive_model,
                lc,
                work_points,
                work_areas,
                pt_index,
                joint_rot,
                &linear_base,
                ndof,
            )?;
            if lc.displacement_control.is_some() {
                case_loads = initial_chain_loads(model, lc);
                case_loads.extend(
                    lc.loads
                        .iter()
                        .map(|a| (a.load_name.clone(), factor * a.scale_factor)),
                );
                reached_factor = Some(factor);
            }
        }
        let mut result = solve_static_with_stiffness(
            &constitutive_model,
            &lc.name,
            &case_loads,
            work_points,
            work_areas,
            orig_area_map,
            pt_index,
            joint_rot,
            &linear_struct,
            &linear_base,
            &IndexMap::new(),
            Some(lc),
            ndof,
        )?;
        result.displacement_control_factor = reached_factor;
        return Ok(result);
    }

    let mut current_axial: IndexMap<String, f64> = IndexMap::new();
    for (fname, fr) in &model.frames {
        if fr.pdelta_force.abs() > 1e-18 {
            current_axial.insert(fname.clone(), fr.pdelta_force);
        }
    }

    let mut last_res = solve_static_with_stiffness(
        model,
        &lc.name,
        &case_loads,
        work_points,
        work_areas,
        orig_area_map,
        pt_index,
        joint_rot,
        k_struct,
        k_base,
        &current_axial,
        None,
        ndof,
    )?;

    if lc.nl_geom == NLGeomType::None {
        return Ok(last_res);
    }
    // A solve counts as one iteration. Honor the requested budget and tolerance;
    // never publish the last iterate as converged merely because it was last.
    for iteration in 1..=lc.max_iterations {
        let mut max_diff = 0.0f64;
        let mut force_scale = 0.0f64;
        for (fname, stations) in &last_res.frame_forces {
            if !stations.is_empty() {
                let p_new = 0.5 * (stations.first().unwrap().p + stations.last().unwrap().p);
                let p_old = current_axial.get(fname).copied().unwrap_or(0.0);
                max_diff = max_diff.max((p_new - p_old).abs());
                force_scale = force_scale.max(p_new.abs()).max(p_old.abs());
                current_axial.insert(fname.clone(), p_new);
            }
        }
        if max_diff <= lc.convergence_tol * force_scale {
            return Ok(last_res);
        }
        if iteration == lc.max_iterations {
            return Err(SolverError::NonConvergence {
                case: lc.name.clone(),
                iterations: iteration,
                residual: max_diff,
                reason: "P-Delta axial-force iteration limit reached".into(),
            });
        }
        let kg = assemble_global_geometric_stiffness(
            model,
            work_points,
            pt_index,
            joint_rot,
            &current_axial,
            true,
            ndof,
        )?;
        let mut k_iter = k_base.to_vec();
        let mut k_struct_iter = k_struct.to_vec();
        for i in 0..(ndof * ndof) {
            k_iter[i] += kg[i];
            k_struct_iter[i] += kg[i];
        }
        last_res = solve_static_with_stiffness(
            model,
            &lc.name,
            &case_loads,
            work_points,
            work_areas,
            orig_area_map,
            pt_index,
            joint_rot,
            &k_struct_iter,
            &k_iter,
            &current_axial,
            None,
            ndof,
        )?;
    }
    unreachable!()
}

/// Merge support forces and their global resultant about the model origin.
fn add_reaction_contributions(
    contributions: IndexMap<String, [f64; 6]>,
    points: &IndexMap<String, PointObj>,
    indices: &IndexMap<String, usize>,
    rotations: &[[[f64; 3]; 3]],
    reactions: &mut IndexMap<String, [f64; 6]>,
    base: &mut [f64; 6],
) {
    for (name, delta) in contributions {
        let point = &points[&name];
        let r = rotations[indices[&name]];
        let reaction = reactions.entry(name).or_insert([0.; 6]);
        for d in 0..6 {
            reaction[d] += delta[d];
        }
        let global: [f64; 6] =
            std::array::from_fn(|i| (0..3).map(|j| r[i % 3][j] * delta[3 * (i / 3) + j]).sum());
        for d in 0..6 {
            base[d] += global[d];
        }
        base[3] += point.y * global[2] - point.z * global[1];
        base[4] += point.z * global[0] - point.x * global[2];
        base[5] += point.x * global[1] - point.y * global[0];
    }
}

/// Nodal loads, fixed-end forces and prescribed displacements of a scaled set
/// of load patterns. Every quantity is linear in the pattern scale factors.
#[allow(clippy::type_complexity)]
struct StaticLoads {
    f_ext_joint: Vec<f64>,
    f_fef_joint: Vec<f64>,
    prescribed_u: Vec<Option<f64>>,
    spring_ground_u: Vec<f64>,
    frame_fef_map: IndexMap<String, ([f64; 12], [[f64; 12]; 12], [[f64; 3]; 3], f64)>,
    frame_p0_map: IndexMap<String, f64>,
    rhs: Vec<f64>,
    is_fixed: Vec<bool>,
    u_fixed_val: Vec<f64>,
}

#[allow(clippy::too_many_arguments)]
fn assemble_static_loads(
    model: &SapModel,
    case_loads: &[(String, f64)],
    work_points: &IndexMap<String, PointObj>,
    work_areas: &IndexMap<String, AreaObj>,
    pt_index: &IndexMap<String, usize>,
    joint_rot: &[[[f64; 3]; 3]],
    frame_axial: &IndexMap<String, f64>,
    ndof: usize,
) -> Result<StaticLoads, SolverError> {
    let mut f_ext_joint = vec![0.0f64; ndof];
    let mut f_fef_joint = vec![0.0f64; ndof];
    let mut prescribed_u: Vec<Option<f64>> = vec![None; ndof];
    let mut spring_ground_u = vec![0.0f64; ndof];

    // 1. Joint Force Loads (`loads_force`) and Joint Displacement Loads (`loads_displ`)
    for (idx, pt) in work_points.values().enumerate() {
        let r_p = &joint_rot[idx];
        let r_p_t = mat3_transpose(r_p);

        for (pat_name, sf) in case_loads {
            if sf.abs() <= 1e-18 {
                continue;
            }
            for lf in &pt.loads_force {
                if &lf.load_pat != pat_name {
                    continue;
                }
                let f_scaled = [
                    lf.force[0] * sf,
                    lf.force[1] * sf,
                    lf.force[2] * sf,
                    lf.force[3] * sf,
                    lf.force[4] * sf,
                    lf.force[5] * sf,
                ];
                if lf.csys.eq_ignore_ascii_case("LOCAL") {
                    for d in 0..6 {
                        f_ext_joint[6 * idx + d] += f_scaled[d];
                    }
                } else {
                    // Transform from Global to Joint-Local: f_joint = R_p^T * f_global
                    for d in 0..3 {
                        f_ext_joint[6 * idx + d] += r_p_t[d][0] * f_scaled[0]
                            + r_p_t[d][1] * f_scaled[1]
                            + r_p_t[d][2] * f_scaled[2];
                        f_ext_joint[6 * idx + 3 + d] += r_p_t[d][0] * f_scaled[3]
                            + r_p_t[d][1] * f_scaled[4]
                            + r_p_t[d][2] * f_scaled[5];
                    }
                }
            }

            for ld in &pt.loads_displ {
                if &ld.load_pat != pat_name {
                    continue;
                }
                let u_scaled = [
                    ld.displ[0] * sf,
                    ld.displ[1] * sf,
                    ld.displ[2] * sf,
                    ld.displ[3] * sf,
                    ld.displ[4] * sf,
                    ld.displ[5] * sf,
                ];
                let u_j = if ld.csys.eq_ignore_ascii_case("LOCAL") {
                    u_scaled
                } else {
                    let mut uj = [0.0; 6];
                    for d in 0..3 {
                        uj[d] = r_p_t[d][0] * u_scaled[0]
                            + r_p_t[d][1] * u_scaled[1]
                            + r_p_t[d][2] * u_scaled[2];
                        uj[d + 3] = r_p_t[d][0] * u_scaled[3]
                            + r_p_t[d][1] * u_scaled[4]
                            + r_p_t[d][2] * u_scaled[5];
                    }
                    uj
                };

                let ks_loc_opt = joint_spring_matrix_local(pt, &joint_rot[idx]);
                for d in 0..6 {
                    if u_j[d].abs() > 1e-18 {
                        let gdof = 6 * idx + d;
                        let has_spring_dof = ks_loc_opt
                            .as_ref()
                            .is_some_and(|ks| ks[d].iter().any(|&v| v.abs() > 0.0));
                        if pt.restraint[d] {
                            let prev = prescribed_u[gdof].unwrap_or(0.0);
                            prescribed_u[gdof] = Some(prev + u_j[d]);
                        } else if has_spring_dof {
                            // Grounded spring settlement/rotation (`Example 1-005` Models B & D)
                            spring_ground_u[gdof] += u_j[d];
                        } else {
                            // Also allow prescribed displacement on unrestrained node if explicitly requested
                            let prev = prescribed_u[gdof].unwrap_or(0.0);
                            prescribed_u[gdof] = Some(prev + u_j[d]);
                        }
                    }
                }
            }
        }
    }

    // Link mass/weight is halved between ends, including the grounded end.
    for link in model.links.values() {
        let prop = &model.link_props[&link.prop];
        let weight: f64 = case_loads
            .iter()
            .map(|(pat, sf)| {
                model
                    .load_patterns
                    .get(pat)
                    .map_or(0., |p| p.self_wt_multiplier * sf * prop.weight)
            })
            .sum();
        for name in std::iter::once(&link.point_i).chain(link.point_j.iter()) {
            let idx = pt_index[name];
            for d in 0..3 {
                f_ext_joint[6 * idx + d] -= 0.5 * weight * joint_rot[idx][2][d];
            }
        }
    }

    // 2. Frame Fixed-End Forces (`f_fef`)
    let mut frame_fef_map: IndexMap<String, ([f64; 12], [[f64; 12]; 12], [[f64; 3]; 3], f64)> =
        IndexMap::new();
    let mut frame_p0_map: IndexMap<String, f64> = IndexMap::new();
    for (fname, frame) in &model.frames {
        let idx_i = pt_index[&frame.point_i];
        let idx_j = pt_index[&frame.point_j];
        let pt_i = &work_points[idx_i];
        let pt_j = &work_points[idx_j];
        let sec = &model.frame_sections[&frame.section];
        let mat = &model.materials[&sec.material];

        let (length, e_axes) = frame_local_axes(
            [pt_i.x, pt_i.y, pt_i.z],
            [pt_j.x, pt_j.y, pt_j.z],
            frame.angle_deg,
        );
        let rz = frame.rigid_zone_factor.clamp(0.0, 1.0);
        let di = rz * frame.end_offset_i;
        let dj = rz * frame.end_offset_j;
        let lf = (length - di - dj).max(1e-6 * length);

        let mut k_loc = compute_flexible_local_stiffness(model, frame, sec, mat, lf);
        let p_ax = frame_axial
            .get(fname)
            .copied()
            .unwrap_or(frame.pdelta_force);
        if p_ax.abs() > 1e-18 {
            let kg_loc = compute_local_geometric_stiffness(p_ax, lf);
            for r in 0..12 {
                for c in 0..12 {
                    k_loc[r][c] += kg_loc[r][c];
                }
            }
        }

        let mut fef_total = [0.0f64; 12];
        for (pat_name, sf) in case_loads {
            let self_wt_mult = model
                .load_patterns
                .get(pat_name)
                .map(|lp| lp.self_wt_multiplier)
                .unwrap_or(0.0);
            let fef_pat = compute_flexible_fef_for_pattern(
                model,
                frame,
                sec,
                mat,
                pt_i,
                pt_j,
                &e_axes,
                lf,
                pat_name,
                self_wt_mult,
                *sf,
            );
            for m in 0..12 {
                fef_total[m] += fef_pat[m];
            }
        }

        condense_releases_and_springs(&mut k_loc, &mut fef_total, frame);
        apply_rigid_offsets(&mut k_loc, &mut fef_total, di, dj);
        let insertion = frame_insertion_offset(model, frame)?;
        apply_insertion_offset(&mut k_loc, &mut fef_total, insertion);
        frame_p0_map.insert(fname.clone(), 0.5 * (fef_total[6] - fef_total[0]));

        // Transform fef_total from element-local to joint-local: f_joint = T^T * fef_local
        let ti = mat3_mul(&e_axes, &joint_rot[idx_i]);
        let tj = mat3_mul(&e_axes, &joint_rot[idx_j]);
        for c in 0..3 {
            let mut fi_f = 0.0;
            let mut fi_m = 0.0;
            let mut fj_f = 0.0;
            let mut fj_m = 0.0;
            for r in 0..3 {
                fi_f += ti[r][c] * fef_total[r];
                fi_m += ti[r][c] * fef_total[r + 3];
                fj_f += tj[r][c] * fef_total[r + 6];
                fj_m += tj[r][c] * fef_total[r + 9];
            }
            f_fef_joint[6 * idx_i + c] += fi_f;
            f_fef_joint[6 * idx_i + 3 + c] += fi_m;
            f_fef_joint[6 * idx_j + c] += fj_f;
            f_fef_joint[6 * idx_j + 3 + c] += fj_m;
        }

        frame_fef_map.insert(fname.clone(), (fef_total, k_loc, e_axes, length));
    }

    // 3. Area Nodal Loads
    for area in work_areas.values() {
        if area.points.len() < 3 {
            continue;
        }
        let sec = &model.area_sections[&area.section];
        let mat = &model.materials[&sec.material];
        let n_nodes = area.points.len().min(4);
        let mut node_indices = Vec::with_capacity(n_nodes);
        let mut coords = Vec::with_capacity(n_nodes);
        for pname in area.points.iter().take(n_nodes) {
            let idx = pt_index[pname];
            let pt = &work_points[idx];
            node_indices.push(idx);
            coords.push([pt.x, pt.y, pt.z]);
        }
        let (e_axes, _, local_xy) = area_local_system(&coords, area.angle_deg + sec.mat_angle_deg);

        for (pat_name, sf) in case_loads {
            let self_wt_mult = model
                .load_patterns
                .get(pat_name)
                .map(|lp| lp.self_wt_multiplier)
                .unwrap_or(0.0);
            let f_area_loc = compute_area_local_loads(
                area,
                sec,
                mat,
                &e_axes,
                &local_xy,
                pat_name,
                self_wt_mult,
                *sf,
            );
            for (a, &pt_idx) in node_indices.iter().enumerate() {
                let ta = mat3_mul(&e_axes, &joint_rot[pt_idx]);
                for c in 0..3 {
                    let mut ff = 0.0;
                    let mut fm = 0.0;
                    for r in 0..3 {
                        ff += ta[r][c] * f_area_loc[6 * a + r];
                        fm += ta[r][c] * f_area_loc[6 * a + 3 + r];
                    }
                    f_fef_joint[6 * pt_idx + c] -= ff;
                    f_fef_joint[6 * pt_idx + 3 + c] -= fm;
                }
            }
        }
    }

    // 3b. 3D Solid Nodal Loads (self-weight, surface pressure, uniform temperature)
    for solid in model.solids.values() {
        let sec = &model.solid_sections[&solid.section];
        let mat = &model.materials[&sec.material];
        let mut node_indices = [0usize; 8];
        let mut coords = [[0.0; 3]; 8];
        for (i, pname) in solid.points.iter().enumerate() {
            let idx = pt_index[pname];
            let pt = &work_points[idx];
            node_indices[i] = idx;
            coords[i] = [pt.x, pt.y, pt.z];
        }
        for (pat_name, sf) in case_loads {
            if sf.abs() <= 1e-18 {
                continue;
            }
            let self_wt_mult = model
                .load_patterns
                .get(pat_name)
                .map(|lp| lp.self_wt_multiplier)
                .unwrap_or(0.0);
            let f_solid_glob = compute_solid_nodal_loads(
                &coords,
                mat,
                sec,
                solid,
                pat_name,
                self_wt_mult,
                [0.0, 0.0, -1.0],
            )?;
            for (a, &pt_idx) in node_indices.iter().enumerate() {
                let ra = &joint_rot[pt_idx];
                for c in 0..3 {
                    let mut ff = 0.0;
                    for r in 0..3 {
                        ff += ra[r][c] * f_solid_glob[3 * a + r];
                    }
                    f_fef_joint[6 * pt_idx + c] -= ff * (*sf);
                }
            }
        }
    }

    // 4. Form right-hand side F_rhs = f_ext - f_fef + K_spring * u_ground
    let mut rhs = vec![0.0f64; ndof];
    for i in 0..ndof {
        rhs[i] = f_ext_joint[i] - f_fef_joint[i];
    }
    for (idx, pt) in work_points.values().enumerate() {
        if let Some(ks_loc) = joint_spring_matrix_local(pt, &joint_rot[idx]) {
            for r in 0..6 {
                let mut f_sp = 0.0;
                for c in 0..6 {
                    f_sp += ks_loc[r][c] * spring_ground_u[6 * idx + c];
                }
                rhs[6 * idx + r] += f_sp;
            }
        }
    }

    // 5. Apply restraints, inactive global DOFs (`model.active_dofs`), and prescribed displacements
    let mut is_fixed = vec![false; ndof];
    let mut u_fixed_val = vec![0.0f64; ndof];
    for (idx, pt) in work_points.values().enumerate() {
        for d in 0..6 {
            let gdof = 6 * idx + d;
            if !model.active_dofs[d] || pt.restraint[d] || prescribed_u[gdof].is_some() {
                is_fixed[gdof] = true;
                u_fixed_val[gdof] = prescribed_u[gdof].unwrap_or(0.0);
            }
        }
    }

    Ok(StaticLoads {
        f_ext_joint,
        f_fef_joint,
        prescribed_u,
        spring_ground_u,
        frame_fef_map,
        frame_p0_map,
        rhs,
        is_fixed,
        u_fixed_val,
    })
}

/// Core static solver given assembled stiffness matrices `k_struct` and `k_total`.
#[allow(clippy::too_many_arguments)]
fn solve_static_with_stiffness(
    model: &SapModel,
    case_name: &str,
    case_loads: &[(String, f64)],
    work_points: &IndexMap<String, PointObj>,
    work_areas: &IndexMap<String, AreaObj>,
    orig_area_map: &IndexMap<String, Vec<String>>,
    pt_index: &IndexMap<String, usize>,
    joint_rot: &[[[f64; 3]; 3]],
    k_struct: &[f64],
    k_total: &[f64],
    frame_axial: &IndexMap<String, f64>,
    nonlinear_case: Option<&LoadCase>,
    ndof: usize,
) -> Result<CaseResults, SolverError> {
    let StaticLoads {
        f_ext_joint,
        f_fef_joint,
        prescribed_u,
        spring_ground_u,
        frame_fef_map,
        frame_p0_map,
        mut rhs,
        is_fixed,
        u_fixed_val,
    } = assemble_static_loads(
        model,
        case_loads,
        work_points,
        work_areas,
        pt_index,
        joint_rot,
        frame_axial,
        ndof,
    )?;

    // Path-dependent link state inherited from the initial case, if any.
    let initial_hysteresis: &[sap_model::LinkHysteresisState] = nonlinear_case
        .and_then(|case| case.initial_case.as_ref())
        .and_then(|name| model.case_results.get(name))
        .map_or(&[], |result| result.link_hysteresis.as_slice());
    let nonlinear_system = nonlinear_case
        .map(|_| {
            nonlinear::System::new_with_frame_p0(
                model,
                work_points,
                pt_index,
                joint_rot,
                k_total,
                &frame_p0_map,
                initial_hysteresis,
                ndof,
            )
        })
        .transpose()?;
    let u_joint = if let (Some(system), Some(case)) = (&nonlinear_system, nonlinear_case) {
        let u_init = initial_displacement_vector(model, case, work_points, ndof);
        system.solve_from(&rhs, &is_fixed, &u_fixed_val, u_init.as_deref(), case)?
    } else {
        let mut k_mod = k_total.to_vec();
        for i in 0..ndof {
            if is_fixed[i] {
                let ui = u_fixed_val[i];
                if ui.abs() > 0.0 {
                    for f in 0..ndof {
                        if !is_fixed[f] {
                            rhs[f] -= k_mod[f * ndof + i] * ui;
                        }
                    }
                }
                for j in 0..ndof {
                    k_mod[i * ndof + j] = 0.0;
                    k_mod[j * ndof + i] = 0.0;
                }
                k_mod[i * ndof + i] = 1.0;
                rhs[i] = ui;
            }
        }

        // Constraint penalties sit on the diagonal of `k_total` but not of
        // `k_struct`; the singularity test must not be relative to them.
        let structural_diagonal: Vec<f64> = (0..ndof).map(|i| k_struct[i * ndof + i]).collect();
        let ldlt =
            SymmetricLdlt::factorize_with_reference(&k_mod, ndof, Some(&structural_diagonal))?;
        ldlt.solve(&rhs)?
    };
    let nonlinear_correction = nonlinear_system
        .as_ref()
        .map_or_else(|| vec![0.; ndof], |system| system.correction(&u_joint));
    let hysteretic_forces = nonlinear_system
        .as_ref()
        .map(|system| system.hysteretic_forces(&u_joint))
        .unwrap_or_default();
    let link_hysteresis = nonlinear_system
        .as_ref()
        .map(|system| system.hysteretic_states(&u_joint))
        .unwrap_or_default();

    // 6. Recover Joint Displacements and Support/Spring Reactions
    let mut joint_displacements = IndexMap::new();
    let mut joint_reactions = IndexMap::new();
    let mut base_reactions = [0.0f64; 6];

    for (idx, (pname, pt)) in work_points.iter().enumerate() {
        let disp: [f64; 6] = std::array::from_fn(|d| u_joint[6 * idx + d]);
        joint_displacements.insert(pname.clone(), disp);

        let ks_loc_opt = joint_spring_matrix_local(pt, &joint_rot[idx]);
        let has_support = pt.restraint.iter().any(|&r| r)
            || ks_loc_opt.is_some()
            || (0..6).any(|d| prescribed_u[6 * idx + d].is_some());
        if has_support && model.points.contains_key(pname) {
            let mut reac = [0.0f64; 6];
            for d in 0..6 {
                let gdof = 6 * idx + d;
                if pt.restraint[d] || prescribed_u[gdof].is_some() {
                    // Internal nodal force - external nodal force
                    let mut f_int = f_fef_joint[gdof] + nonlinear_correction[gdof];
                    for j in 0..ndof {
                        f_int += k_struct[gdof * ndof + j] * u_joint[j];
                    }
                    reac[d] = f_int - f_ext_joint[gdof];
                } else if let Some(ks_loc) = &ks_loc_opt {
                    // Grounded spring reaction on structure: R = -K_spring * (u - u_ground)
                    let mut f_sp = 0.0;
                    for c in 0..6 {
                        f_sp += ks_loc[d][c] * (u_joint[6 * idx + c] - spring_ground_u[6 * idx + c]);
                    }
                    reac[d] = -f_sp;
                }
            }
            joint_reactions.insert(pname.clone(), reac);

            // Transform joint reaction to global for base_reactions sum (including r x F about origin)
            let r_p = &joint_rot[idx];
            let mut r_glob_f = [0.0f64; 3];
            let mut r_glob_m = [0.0f64; 3];
            for g in 0..3 {
                r_glob_f[g] = r_p[g][0] * reac[0] + r_p[g][1] * reac[1] + r_p[g][2] * reac[2];
                r_glob_m[g] = r_p[g][0] * reac[3] + r_p[g][1] * reac[4] + r_p[g][2] * reac[5];
                base_reactions[g] += r_glob_f[g];
                base_reactions[g + 3] += r_glob_m[g];
            }
            base_reactions[3] += pt.y * r_glob_f[2] - pt.z * r_glob_f[1];
            base_reactions[4] += pt.z * r_glob_f[0] - pt.x * r_glob_f[2];
            base_reactions[5] += pt.x * r_glob_f[1] - pt.y * r_glob_f[0];
        }
    }

    add_reaction_contributions(
        link::grounded_reactions(
            model,
            work_points,
            pt_index,
            joint_rot,
            &u_joint,
            None,
            nonlinear_case.is_some(),
            &hysteretic_forces,
        ),
        work_points,
        pt_index,
        joint_rot,
        &mut joint_reactions,
        &mut base_reactions,
    );

    // 7. Recover Frame Internal Station Forces & Deflections
    let mut frame_forces = IndexMap::new();
    for (fname, frame) in &model.frames {
        let idx_i = pt_index[&frame.point_i];
        let idx_j = pt_index[&frame.point_j];
        let sec = &model.frame_sections[&frame.section];
        let mat = &model.materials[&sec.material];
        let (fef_loc, k_loc, e_axes, length) = &frame_fef_map[fname];

        let ti = mat3_mul(e_axes, &joint_rot[idx_i]);
        let tj = mat3_mul(e_axes, &joint_rot[idx_j]);

        let mut u_loc = [0.0f64; 12];
        for r in 0..3 {
            for c in 0..3 {
                u_loc[r] += ti[r][c] * u_joint[6 * idx_i + c];
                u_loc[r + 3] += ti[r][c] * u_joint[6 * idx_i + 3 + c];
                u_loc[r + 6] += tj[r][c] * u_joint[6 * idx_j + c];
                u_loc[r + 9] += tj[r][c] * u_joint[6 * idx_j + 3 + c];
            }
        }

        let mut f_end_loc = *fef_loc;
        for r in 0..12 {
            for c in 0..12 {
                f_end_loc[r] += k_loc[r][c] * u_loc[c];
            }
        }
        // Station forces and deflections are those of the element, whose
        // ends an insertion point sets off from the joints.
        insertion_to_element_ends(
            &mut u_loc,
            &mut f_end_loc,
            frame_insertion_offset(model, frame)?,
        );
        if nonlinear_case.is_some() {
            if let Some((has_c, c_lim, has_t, t_lim)) = frame.tc_limits {
                if has_c || has_t {
                    let mut k_tc = *k_loc;
                    let _ = cable::apply_frame_tc_limits_local(
                        &mut f_end_loc,
                        &mut k_tc,
                        cable::FrameTcLimits {
                            tension_limit: has_t.then_some(t_lim),
                            compression_limit: has_c.then_some(c_lim),
                        },
                        0.0,
                    );
                }
            }
        }

        let stations = recover_frame_stations(
            model, frame, sec, mat, e_axes, *length, &u_loc, &f_end_loc, case_loads,
        );
        frame_forces.insert(fname.clone(), stations);
    }

    // 8. Recover Area Internal Forces & Stresses
    let mut sub_area_results = IndexMap::new();
    for (aname, area) in work_areas {
        if area.points.len() < 3 {
            continue;
        }
        let sec = &model.area_sections[&area.section];
        let mat = &model.materials[&sec.material];
        let n_nodes = area.points.len().min(4);
        let mut node_indices = Vec::with_capacity(n_nodes);
        let mut coords = Vec::with_capacity(n_nodes);
        for pname in area.points.iter().take(n_nodes) {
            let idx = pt_index[pname];
            let pt = &work_points[idx];
            node_indices.push(idx);
            coords.push([pt.x, pt.y, pt.z]);
        }
        let (e_axes, _, local_xy) = area_local_system(&coords, area.angle_deg + sec.mat_angle_deg);

        let mut u_loc = vec![0.0f64; 6 * n_nodes];
        for (a, &pt_idx) in node_indices.iter().enumerate() {
            let ta = mat3_mul(&e_axes, &joint_rot[pt_idx]);
            for r in 0..3 {
                for c in 0..3 {
                    u_loc[6 * a + r] += ta[r][c] * u_joint[6 * pt_idx + c];
                    u_loc[6 * a + 3 + r] += ta[r][c] * u_joint[6 * pt_idx + 3 + c];
                }
            }
        }

        let ares = recover_area_result(area, sec, mat, &local_xy, &u_loc, Vec::new());
        sub_area_results.insert(aname.clone(), ares);
    }

    let mut area_results = IndexMap::new();
    for (orig_name, sub_list) in orig_area_map {
        if sub_list.len() == 1 {
            if let Some(r) = sub_area_results.get(&sub_list[0]) {
                area_results.insert(orig_name.clone(), r.clone());
            }
        } else if !sub_list.is_empty() {
            // Average over sub-elements for auto-meshed area
            let mut avg = sap_model::AreaResult::default();
            let count = sub_list.len() as f64;
            for sname in sub_list {
                if let Some(r) = sub_area_results.get(sname) {
                    avg.f11 += r.f11 / count;
                    avg.f22 += r.f22 / count;
                    avg.f12 += r.f12 / count;
                    avg.m11 += r.m11 / count;
                    avg.m22 += r.m22 / count;
                    avg.m12 += r.m12 / count;
                    avg.v13 += r.v13 / count;
                    avg.v23 += r.v23 / count;
                    avg.s11_top += r.s11_top / count;
                    avg.s22_top += r.s22_top / count;
                    avg.s12_top += r.s12_top / count;
                    avg.svm_top += r.svm_top / count;
                }
            }
            area_results.insert(orig_name.clone(), avg);
        }
    }

    // 8b. Recover 3D Solid Internal Stresses & Nodal Forces
    let mut solid_results = IndexMap::new();
    for (sname, solid) in &model.solids {
        let sec = &model.solid_sections[&solid.section];
        let mat = &model.materials[&sec.material];
        let mut coords = [[0.0; 3]; 8];
        let mut u_elem = [0.0; 24];
        for (a, pname) in solid.points.iter().enumerate() {
            let idx = pt_index[pname];
            let pt = &work_points[idx];
            coords[a] = [pt.x, pt.y, pt.z];
            let ra = &joint_rot[idx];
            for r in 0..3 {
                u_elem[3 * a + r] = ra[r][0] * u_joint[6 * idx]
                    + ra[r][1] * u_joint[6 * idx + 1]
                    + ra[r][2] * u_joint[6 * idx + 2];
            }
        }
        let mut delta_t = 0.0;
        for (pat_name, sf) in case_loads {
            if let Some(&dt) = solid.loads_temp.get(pat_name) {
                delta_t += dt * (*sf);
            }
        }
        let sres = recover_solid_result(&coords, mat, sec, &u_elem, delta_t)?;
        solid_results.insert(sname.clone(), sres);
    }

    // 9. Link section resultants at the J end in element local axes.
    let mut link_forces = IndexMap::new();
    for (lname, link) in &model.links {
        let prop = &model.link_props[&link.prop];
        let op = link::operator(link, prop, work_points, pt_index, joint_rot);
        link_forces.insert(
            lname.clone(),
            link::forces_for_case(
                prop,
                &op,
                &u_joint,
                nonlinear_case.is_some(),
                &link::hysteretic_overrides(&hysteretic_forces, lname),
            ),
        );
    }

    Ok(CaseResults {
        case_name: case_name.to_string(),
        joint_displacements,
        joint_reactions,
        frame_forces,
        area_results,
        solid_results,
        link_forces,
        modes: Vec::new(),
        buckling_factors: Vec::new(),
        time_steps: Vec::new(),
        time_history_disp: IndexMap::new(),
        base_reactions,
        transient_state: None,
        link_hysteresis,
        displacement_control_factor: None,
    })
}

/// Runs a Modal Eigen or Ritz analysis case (`LoadCaseType::Modal`).
#[allow(clippy::too_many_arguments)]
fn run_modal_case(
    model: &SapModel,
    lc: &LoadCase,
    work_points: &IndexMap<String, PointObj>,
    pt_index: &IndexMap<String, usize>,
    joint_rot: &[[[f64; 3]; 3]],
    k_base: &[f64],
    diag_mass: &[f64],
    ndof: usize,
) -> Result<CaseResults, SolverError> {
    let mut k_mod = k_base.to_vec();
    let mut m_mod = diag_mass.to_vec();

    // Include P-Delta geometric stiffness if initial_case is set
    if let Some(init_name) = &lc.initial_case {
        if let Some(init_res) = model.case_results.get(init_name) {
            let mut init_axial = IndexMap::new();
            for (fname, stations) in &init_res.frame_forces {
                if !stations.is_empty() {
                    let p_avg = 0.5 * (stations.first().unwrap().p + stations.last().unwrap().p);
                    init_axial.insert(fname.clone(), p_avg);
                }
            }
            let kg = assemble_global_geometric_stiffness(
                model,
                work_points,
                pt_index,
                joint_rot,
                &init_axial,
                true,
                ndof,
            )?;
            for i in 0..(ndof * ndof) {
                k_mod[i] += kg[i];
            }
        }
    }

    // Apply restraints and active_dofs
    let mut is_fixed = vec![false; ndof];
    for (idx, pt) in work_points.values().enumerate() {
        for d in 0..6 {
            let gdof = 6 * idx + d;
            if !model.active_dofs[d] || pt.restraint[d] {
                is_fixed[gdof] = true;
                for j in 0..ndof {
                    k_mod[gdof * ndof + j] = 0.0;
                    k_mod[j * ndof + gdof] = 0.0;
                }
                k_mod[gdof * ndof + gdof] = 1.0;
                m_mod[gdof] = 0.0;
            }
        }
    }

    let ldlt = SymmetricLdlt::factorize(&k_mod, ndof)?;
    let raw_modes = if lc.modal_sub_type == ModalSubType::Ritz {
        // Build 3 unit-acceleration starting load vectors (UX, UY, UZ) + any explicit loads in lc.loads
        let mut starting_loads = Vec::new();
        for dir in 0..3 {
            let mut f_acc = vec![0.0; ndof];
            let mut has_m = false;
            for idx in 0..work_points.len() {
                let gdof = 6 * idx + dir;
                if !is_fixed[gdof] && m_mod[gdof] > 1e-18 {
                    f_acc[gdof] = m_mod[gdof];
                    has_m = true;
                }
            }
            if has_m {
                starting_loads.push(f_acc);
            }
        }
        solve_ritz_vectors(&ldlt, &m_mod, &starting_loads, lc.max_modes)?
    } else {
        solve_modal_eigen(&ldlt, &m_mod, lc.max_modes)?
    };

    // Total unrestrained mass along global X, Y, Z for modal mass ratio calculation
    let mut total_mass = [0.0f64; 6];
    for i in 0..ndof {
        let d = i % 6;
        total_mass[d] += m_mod[i];
    }

    let mut modes = Vec::with_capacity(raw_modes.len());
    for (m_idx, (omega_sq, phi)) in raw_modes.into_iter().enumerate() {
        let omega = omega_sq.max(0.0).sqrt();
        let freq_hz = omega / (2.0 * PI);
        let period_sec = if freq_hz > 1e-12 { 1.0 / freq_hz } else { 0.0 };

        let mut pf = [0.0f64; 6];
        for i in 0..ndof {
            let d = i % 6;
            pf[d] += m_mod[i] * phi[i];
        }
        let mut mmr = [0.0f64; 6];
        for d in 0..6 {
            if total_mass[d] > 1e-18 {
                mmr[d] = (pf[d] * pf[d]) / total_mass[d];
            }
        }

        let mut mode_shape = IndexMap::new();
        for (idx, pname) in work_points.keys().enumerate() {
            let disp: [f64; 6] = std::array::from_fn(|d| phi[6 * idx + d]);
            mode_shape.insert(pname.clone(), disp);
        }

        modes.push(ModalModeResult {
            mode_num: m_idx + 1,
            period_sec,
            freq_hz,
            circ_freq_rad_s: omega,
            eigenvalue: omega_sq,
            participation_factors: pf,
            modal_mass_ratios: mmr,
            mode_shape,
        });
    }

    let first_disp = modes
        .first()
        .map(|m| m.mode_shape.clone())
        .unwrap_or_default();

    Ok(CaseResults {
        case_name: lc.name.clone(),
        joint_displacements: first_disp,
        modes,
        ..Default::default()
    })
}

/// Runs a Linear Buckling eigenvalue case (`LoadCaseType::Buckling`).
#[allow(clippy::too_many_arguments)]
fn run_buckling_case(
    model: &SapModel,
    lc: &LoadCase,
    work_points: &IndexMap<String, PointObj>,
    work_areas: &IndexMap<String, AreaObj>,
    orig_area_map: &IndexMap<String, Vec<String>>,
    pt_index: &IndexMap<String, usize>,
    joint_rot: &[[[f64; 3]; 3]],
    k_struct: &[f64],
    k_base: &[f64],
    ndof: usize,
) -> Result<CaseResults, SolverError> {
    // Reference axial forces. A case with no initial case buckles under its
    // own loads: every assigned pattern at its scale factor, solved here as a
    // linear static state. With an initial case, that case's forces are the
    // reference; with neither, the assigned frame P-Delta forces are.
    let mut ref_axial: IndexMap<String, f64> = IndexMap::new();
    let mut take_axial = |results: &CaseResults| {
        for (fname, stations) in &results.frame_forces {
            if let (Some(first), Some(last)) = (stations.first(), stations.last()) {
                ref_axial.insert(fname.clone(), 0.5 * (first.p + last.p));
            }
        }
    };
    if let Some(initial) = &lc.initial_case {
        if let Some(results) = model.case_results.get(initial) {
            take_axial(results);
        }
    } else if !lc.loads.is_empty() {
        let mut reference = LoadCase::linear_static(lc.name.clone(), "", 1.0);
        reference.loads = lc.loads.clone();
        let results = run_linear_static_case(
            model,
            &reference,
            work_points,
            work_areas,
            orig_area_map,
            pt_index,
            joint_rot,
            k_struct,
            k_base,
            ndof,
        )?;
        take_axial(&results);
    }

    if ref_axial.is_empty() {
        for (fname, fr) in &model.frames {
            if fr.pdelta_force.abs() > 1e-18 {
                ref_axial.insert(fname.clone(), fr.pdelta_force);
            }
        }
    }

    let mut kg = assemble_global_geometric_stiffness(
        model,
        work_points,
        pt_index,
        joint_rot,
        &ref_axial,
        false,
        ndof,
    )?;
    let mut k_mod = k_base.to_vec();

    for (idx, pt) in work_points.values().enumerate() {
        for d in 0..6 {
            let gdof = 6 * idx + d;
            if !model.active_dofs[d] || pt.restraint[d] {
                for j in 0..ndof {
                    k_mod[gdof * ndof + j] = 0.0;
                    k_mod[j * ndof + gdof] = 0.0;
                    kg[gdof * ndof + j] = 0.0;
                    kg[j * ndof + gdof] = 0.0;
                }
                k_mod[gdof * ndof + gdof] = 1.0;
            }
        }
    }

    let ldlt = SymmetricLdlt::factorize(&k_mod, ndof)?;
    let buck_modes = solve_buckling_eigen(&ldlt, &kg, lc.num_buckling_modes.max(1))?;
    let buckling_factors: Vec<f64> = buck_modes.iter().map(|(lam, _)| *lam).collect();

    let mut joint_displacements = IndexMap::new();
    if let Some((_, first_phi)) = buck_modes.first() {
        for (idx, pname) in work_points.keys().enumerate() {
            let disp: [f64; 6] = std::array::from_fn(|d| first_phi[6 * idx + d]);
            joint_displacements.insert(pname.clone(), disp);
        }
    }

    Ok(CaseResults {
        case_name: lc.name.clone(),
        joint_displacements,
        buckling_factors,
        ..Default::default()
    })
}

/// Runs a Response Spectrum analysis case (`LoadCaseType::ResponseSpectrum`) with CQC, SRSS, or AbsSum modal combination.
#[allow(clippy::too_many_arguments)]
fn run_response_spectrum_case(
    model: &SapModel,
    lc: &LoadCase,
    work_points: &IndexMap<String, PointObj>,
    work_areas: &IndexMap<String, AreaObj>,
    orig_area_map: &IndexMap<String, Vec<String>>,
    pt_index: &IndexMap<String, usize>,
    joint_rot: &[[[f64; 3]; 3]],
    k_struct: &[f64],
    k_base: &[f64],
    diag_mass: &[f64],
    ndof: usize,
) -> Result<CaseResults, SolverError> {
    // Find solved modal case or run one on the fly
    let modal_res = if let Some(init_name) = &lc.initial_case {
        model.case_results.get(init_name).cloned()
    } else {
        model
            .case_results
            .values()
            .find(|r| !r.modes.is_empty())
            .cloned()
    };

    let modal_res = match modal_res {
        Some(mr) => mr,
        None => {
            let temp_modal = LoadCase::modal_eigen("__TEMP_MODAL", lc.max_modes.max(12));
            run_modal_case(
                model,
                &temp_modal,
                work_points,
                pt_index,
                joint_rot,
                k_base,
                diag_mass,
                ndof,
            )?
        }
    };

    let modes = &modal_res.modes;
    let n_modes = modes.len();
    if n_modes == 0 {
        return Ok(CaseResults {
            case_name: lc.name.clone(),
            ..Default::default()
        });
    }

    // For each mode m, compute modal coordinate amplitude q_m = sum_{dir} (Gamma_{m,dir} * Sa(T_m) * sf) / omega_m^2
    let mut modal_u_vecs: Vec<Vec<f64>> = Vec::with_capacity(n_modes);
    let mut omegas: Vec<f64> = Vec::with_capacity(n_modes);

    for mode in modes {
        let omega = mode.circ_freq_rad_s.max(1e-9);
        let t_sec = mode.period_sec;
        omegas.push(omega);

        let mut q_m = 0.0f64;
        for (func_name, dir_str, sf) in &lc.spectrum_loads {
            let dir_idx = match dir_str.trim().to_ascii_uppercase().as_str() {
                "U1" | "UX" | "X" | "1" => 0,
                "U2" | "UY" | "Y" | "2" => 1,
                "U3" | "UZ" | "Z" | "3" => 2,
                _ => 0,
            };
            let sa = eval_dynamic_function(model, func_name, t_sec);
            q_m += mode.participation_factors[dir_idx] * sa * (*sf) / (omega * omega);
        }

        let mut u_m = vec![0.0f64; ndof];
        for (idx, pname) in work_points.keys().enumerate() {
            if let Some(ms) = mode.mode_shape.get(pname) {
                for d in 0..6 {
                    u_m[6 * idx + d] = q_m * ms[d];
                }
            }
        }
        modal_u_vecs.push(u_m);
    }

    // Combine modal responses using CQC, SRSS, or AbsSum
    let xi = lc.modal_damping.clamp(0.0, 0.99);
    let cqc_corr = |wi: f64, wj: f64| -> f64 {
        let r = wi / wj.max(1e-12);
        let num = 8.0 * xi * xi * (1.0 + r) * r.powf(1.5);
        let den = (1.0 - r * r).powi(2) + 4.0 * xi * xi * r * (1.0 + r).powi(2);
        if den > 1e-18 {
            (num / den).clamp(-1.0, 1.0)
        } else {
            1.0
        }
    };

    let combine_scalars = |vals: &[f64]| -> f64 {
        match lc.modal_combo {
            ModalComboMethod::AbsSum => vals.iter().map(|v| v.abs()).sum(),
            ModalComboMethod::Srss => vals.iter().map(|v| v * v).sum::<f64>().sqrt(),
            ModalComboMethod::Cqc => {
                let mut sum = 0.0f64;
                for i in 0..n_modes {
                    for j in 0..n_modes {
                        let rho = if i == j {
                            1.0
                        } else {
                            cqc_corr(omegas[i], omegas[j])
                        };
                        sum += rho * vals[i] * vals[j];
                    }
                }
                sum.max(0.0).sqrt()
            }
        }
    };

    // Combine joint displacements and reactions mode-by-mode
    let mut joint_displacements = IndexMap::new();
    for (idx, pname) in work_points.keys().enumerate() {
        let mut comb_d = [0.0f64; 6];
        for d in 0..6 {
            let modal_vals: Vec<f64> = (0..n_modes).map(|m| modal_u_vecs[m][6 * idx + d]).collect();
            comb_d[d] = combine_scalars(&modal_vals);
        }
        joint_displacements.insert(pname.clone(), comb_d);
    }

    // Recover modal reactions and frame forces and combine them with CQC/SRSS
    let mut modal_case_results: Vec<CaseResults> = Vec::with_capacity(n_modes);
    for u_m in &modal_u_vecs {
        let mr = recover_results_from_displacement_vector(
            model,
            work_points,
            work_areas,
            orig_area_map,
            pt_index,
            joint_rot,
            k_struct,
            u_m,
            ndof,
        );
        modal_case_results.push(mr);
    }

    let mut joint_reactions = IndexMap::new();
    for pname in model.points.keys() {
        if modal_case_results
            .first()
            .is_some_and(|r| r.joint_reactions.contains_key(pname))
        {
            let mut cr = [0.0f64; 6];
            for d in 0..6 {
                let vals: Vec<f64> = modal_case_results
                    .iter()
                    .map(|r| r.joint_reactions[pname][d])
                    .collect();
                cr[d] = combine_scalars(&vals);
            }
            joint_reactions.insert(pname.clone(), cr);
        }
    }

    let mut base_reactions = [0.0f64; 6];
    for d in 0..6 {
        let vals: Vec<f64> = modal_case_results
            .iter()
            .map(|r| r.base_reactions[d])
            .collect();
        base_reactions[d] = combine_scalars(&vals);
    }

    let mut frame_forces = IndexMap::new();
    for fname in model.frames.keys() {
        if let Some(first_sta) = modal_case_results
            .first()
            .and_then(|r| r.frame_forces.get(fname))
        {
            let n_sta = first_sta.len();
            let mut comb_sta = Vec::with_capacity(n_sta);
            for s_idx in 0..n_sta {
                let p_v: Vec<f64> = modal_case_results
                    .iter()
                    .map(|r| r.frame_forces[fname][s_idx].p)
                    .collect();
                let v2_v: Vec<f64> = modal_case_results
                    .iter()
                    .map(|r| r.frame_forces[fname][s_idx].v2)
                    .collect();
                let v3_v: Vec<f64> = modal_case_results
                    .iter()
                    .map(|r| r.frame_forces[fname][s_idx].v3)
                    .collect();
                let t_v: Vec<f64> = modal_case_results
                    .iter()
                    .map(|r| r.frame_forces[fname][s_idx].t)
                    .collect();
                let m2_v: Vec<f64> = modal_case_results
                    .iter()
                    .map(|r| r.frame_forces[fname][s_idx].m2)
                    .collect();
                let m3_v: Vec<f64> = modal_case_results
                    .iter()
                    .map(|r| r.frame_forces[fname][s_idx].m3)
                    .collect();

                comb_sta.push(FrameStationResult {
                    station: first_sta[s_idx].station,
                    rel_dist: first_sta[s_idx].rel_dist,
                    p: combine_scalars(&p_v),
                    v2: combine_scalars(&v2_v),
                    v3: combine_scalars(&v3_v),
                    t: combine_scalars(&t_v),
                    m2: combine_scalars(&m2_v),
                    m3: combine_scalars(&m3_v),
                    local_disp: [0.0; 3],
                });
            }
            frame_forces.insert(fname.clone(), comb_sta);
        }
    }

    Ok(CaseResults {
        case_name: lc.name.clone(),
        joint_displacements,
        joint_reactions,
        frame_forces,
        base_reactions,
        ..Default::default()
    })
}

fn assemble_static_pattern_rhs(
    model: &SapModel,
    case_loads: &[(String, f64)],
    work_points: &IndexMap<String, PointObj>,
    work_areas: &IndexMap<String, AreaObj>,
    pt_index: &IndexMap<String, usize>,
    joint_rot: &[[[f64; 3]; 3]],
    ndof: usize,
) -> Result<Vec<f64>, SolverError> {
    let mut rhs = vec![0.0; ndof];
    for (idx, pt) in work_points.values().enumerate() {
        let r_p_t = mat3_transpose(&joint_rot[idx]);
        for (pat_name, sf) in case_loads {
            if sf.abs() <= 1e-18 {
                continue;
            }
            for lf in &pt.loads_force {
                if &lf.load_pat != pat_name {
                    continue;
                }
                let f_scaled = [
                    lf.force[0] * sf,
                    lf.force[1] * sf,
                    lf.force[2] * sf,
                    lf.force[3] * sf,
                    lf.force[4] * sf,
                    lf.force[5] * sf,
                ];
                if lf.csys.eq_ignore_ascii_case("LOCAL") {
                    for d in 0..6 {
                        rhs[6 * idx + d] += f_scaled[d];
                    }
                } else {
                    for d in 0..3 {
                        rhs[6 * idx + d] += r_p_t[d][0] * f_scaled[0]
                            + r_p_t[d][1] * f_scaled[1]
                            + r_p_t[d][2] * f_scaled[2];
                        rhs[6 * idx + 3 + d] += r_p_t[d][0] * f_scaled[3]
                            + r_p_t[d][1] * f_scaled[4]
                            + r_p_t[d][2] * f_scaled[5];
                    }
                }
            }
        }
    }
    for frame in model.frames.values() {
        let idx_i = pt_index[&frame.point_i];
        let idx_j = pt_index[&frame.point_j];
        let pt_i = &work_points[idx_i];
        let pt_j = &work_points[idx_j];
        let sec = &model.frame_sections[&frame.section];
        let mat = &model.materials[&sec.material];
        let (length, e_axes) = frame_local_axes(
            [pt_i.x, pt_i.y, pt_i.z],
            [pt_j.x, pt_j.y, pt_j.z],
            frame.angle_deg,
        );
        let rz = frame.rigid_zone_factor.clamp(0.0, 1.0);
        let di = rz * frame.end_offset_i;
        let dj = rz * frame.end_offset_j;
        let lf = (length - di - dj).max(1e-6 * length);
        let mut k_loc = compute_flexible_local_stiffness(model, frame, sec, mat, lf);
        let mut fef_total = [0.0f64; 12];
        for (pat_name, sf) in case_loads {
            let self_wt_mult = model
                .load_patterns
                .get(pat_name)
                .map(|lp| lp.self_wt_multiplier)
                .unwrap_or(0.0);
            let fef_pat = compute_flexible_fef_for_pattern(
                model,
                frame,
                sec,
                mat,
                pt_i,
                pt_j,
                &e_axes,
                lf,
                pat_name,
                self_wt_mult,
                *sf,
            );
            for m in 0..12 {
                fef_total[m] += fef_pat[m];
            }
        }
        condense_releases_and_springs(&mut k_loc, &mut fef_total, frame);
        apply_rigid_offsets(&mut k_loc, &mut fef_total, di, dj);
        let insertion = frame_insertion_offset(model, frame)?;
        apply_insertion_offset(&mut k_loc, &mut fef_total, insertion);
        let ti = mat3_mul(&e_axes, &joint_rot[idx_i]);
        let tj = mat3_mul(&e_axes, &joint_rot[idx_j]);
        for c in 0..3 {
            let mut fi_f = 0.0;
            let mut fi_m = 0.0;
            let mut fj_f = 0.0;
            let mut fj_m = 0.0;
            for r in 0..3 {
                fi_f += ti[r][c] * fef_total[r];
                fi_m += ti[r][c] * fef_total[r + 3];
                fj_f += tj[r][c] * fef_total[r + 6];
                fj_m += tj[r][c] * fef_total[r + 9];
            }
            rhs[6 * idx_i + c] -= fi_f;
            rhs[6 * idx_i + 3 + c] -= fi_m;
            rhs[6 * idx_j + c] -= fj_f;
            rhs[6 * idx_j + 3 + c] -= fj_m;
        }
    }
    let _ = work_areas;
    Ok(rhs)
}

/// Runs a Direct-Integration Time-History case (`Newmark-beta` / `HHT-alpha`).
#[allow(clippy::too_many_arguments)]
fn run_time_history_case(
    model: &SapModel,
    lc: &LoadCase,
    work_points: &IndexMap<String, PointObj>,
    work_areas: &IndexMap<String, AreaObj>,
    orig_area_map: &IndexMap<String, Vec<String>>,
    pt_index: &IndexMap<String, usize>,
    joint_rot: &[[[f64; 3]; 3]],
    k_struct: &[f64],
    k_base: &[f64],
    diag_mass: &[f64],
    ndof: usize,
) -> Result<CaseResults, SolverError> {
    let n_steps = lc.num_time_steps;
    let dt = lc.time_step_dt;
    let mut is_fixed = vec![false; ndof];
    for (idx, point) in work_points.values().enumerate() {
        for d in 0..6 {
            is_fixed[6 * idx + d] = !model.active_dofs[d] || point.restraint[d];
        }
    }

    // Precompute spatial load vectors for each history_loads entry
    let mut spatial_loads: Vec<(String, Vec<f64>)> = Vec::new();
    for (func_name, pat_or_acc, sf) in &lc.history_loads {
        let mut f_vec = vec![0.0f64; ndof];
        let upper = pat_or_acc.trim().to_ascii_uppercase();
        if matches!(
            upper.as_str(),
            "U1" | "UX" | "X" | "U2" | "UY" | "Y" | "U3" | "UZ" | "Z"
        ) {
            let dir_idx = match upper.as_str() {
                "U1" | "UX" | "X" => 0,
                "U2" | "UY" | "Y" => 1,
                _ => 2,
            };
            for idx in 0..work_points.len() {
                for d in 0..3 {
                    let gdof = 6 * idx + d;
                    f_vec[gdof] = -diag_mass[gdof] * joint_rot[idx][dir_idx][d] * (*sf);
                }
            }
        } else {
            f_vec = assemble_static_pattern_rhs(
                model,
                &[(pat_or_acc.clone(), *sf)],
                work_points,
                work_areas,
                pt_index,
                joint_rot,
                ndof,
            )?;
        }
        spatial_loads.push((func_name.clone(), f_vec));
    }

    let f_base_init = if let Some(init_name) = &lc.initial_case {
        if let Some(init_lc) = model.load_cases.get(init_name) {
            if matches!(
                init_lc.case_type,
                LoadCaseType::NonlinearStatic
                    | LoadCaseType::NonlinearHistory
                    | LoadCaseType::NonlinearDynamic
                    | LoadCaseType::LinearDynamic
            ) {
                let init_loads = accumulated_nonlinear_static_loads(model, init_lc);
                assemble_static_pattern_rhs(
                    model,
                    &init_loads,
                    work_points,
                    work_areas,
                    pt_index,
                    joint_rot,
                    ndof,
                )?
            } else {
                vec![0.0; ndof]
            }
        } else {
            vec![0.0; ndof]
        }
    } else {
        vec![0.0; ndof]
    };

    let load_at = |t: f64, derivative: bool| {
        let mut force = if derivative {
            vec![0.0; ndof]
        } else {
            f_base_init.clone()
        };
        for (function, spatial) in &spatial_loads {
            let value = if derivative {
                eval_dynamic_function_slope(model, function, t)
            } else {
                eval_dynamic_function(model, function, t)
            };
            for i in 0..ndof {
                force[i] += value * spatial[i];
            }
        }
        force
    };
    let element_damping = link::damping(model, work_points, pt_index, joint_rot, ndof);
    let is_nonlinear_history = matches!(
        lc.case_type,
        LoadCaseType::NonlinearHistory | LoadCaseType::NonlinearDynamic
    );
    let is_direct_history = matches!(
        lc.case_type,
        LoadCaseType::LinearDynamic
            | LoadCaseType::NonlinearHistory
            | LoadCaseType::NonlinearDynamic
    );

    let seed_ts = if let Some(init_name) = &lc.initial_case {
        let init_res = model.case_results.get(init_name);
        if let Some(ts) = init_res.and_then(|r| r.transient_state.as_ref()) {
            let order_matches = ts.point_order.len() == work_points.len()
                && ts
                    .point_order
                    .iter()
                    .zip(work_points.keys())
                    .all(|(a, b)| a == b);
            if !order_matches {
                return Err(SolverError::InvalidInput(format!(
                    "case '{}' initial_case '{}' has incompatible transient point order",
                    lc.name, init_name
                )));
            }
            Some(ts)
        } else if model.load_cases.get(init_name).is_some_and(|c| {
            matches!(
                c.case_type,
                LoadCaseType::LinearDynamic
                    | LoadCaseType::NonlinearHistory
                    | LoadCaseType::NonlinearDynamic
            )
        }) {
            return Err(SolverError::InvalidInput(format!(
                "case '{}' initial_case '{}' is missing persisted transient state",
                lc.name, init_name
            )));
        } else {
            None
        }
    } else {
        None
    };

    let mut u_peak = vec![0.0f64; ndof];
    let mut v_peak = vec![0.0f64; ndof];
    let mut a_peak = vec![0.0f64; ndof];
    let mut t_peak = 0.0;
    let mut peak_norm = 0.0f64;
    let mut nl_link_peak: IndexMap<(String, usize), f64> = IndexMap::new();
    let mut terminal_state: Option<sap_model::TransientTerminalState> = None;
    let mut terminal_hysteresis: Vec<sap_model::LinkHysteresisState> = Vec::new();
    // Linear history borrows the assembled matrices. Only nonlinear history
    // owns a separately assembled constitutive base for reaction recovery.
    let mut reaction_base = std::borrow::Cow::Borrowed(k_base);
    let mut reaction_struct = std::borrow::Cow::Borrowed(k_struct);

    let mut time_steps = Vec::with_capacity(n_steps + 1);
    let mut time_history_disp: IndexMap<String, Vec<[f64; 6]>> = IndexMap::new();
    for pname in model.points.keys() {
        time_history_disp.insert(pname.clone(), Vec::with_capacity(n_steps + 1));
    }

    if is_nonlinear_history {
        let mut constitutive_model = model.clone();
        for prop in constitutive_model.link_props.values_mut() {
            for d in 0..6 {
                if prop.dof_nonlinear[d] && prop.dof_active[d] && !prop.dof_fixed[d] {
                    prop.ke[d] = 0.;
                }
            }
        }
        let (linear_struct, _) = assemble_elastic_stiffness_and_mass(
            &constitutive_model,
            work_points,
            work_areas,
            pt_index,
            joint_rot,
            ndof,
        )?;
        let linear_base = add_springs_and_constraints(
            &constitutive_model,
            work_points,
            pt_index,
            joint_rot,
            &linear_struct,
            ndof,
        );
        reaction_base = std::borrow::Cow::Owned(linear_base);
        reaction_struct = std::borrow::Cow::Owned(linear_struct);
        let u_init = initial_displacement_vector(model, lc, work_points, ndof);
        let f0 = load_at(0.0, false);
        let mut integrator = dynamics::NonlinearStepper::new(
            model,
            work_points,
            pt_index,
            joint_rot,
            &reaction_base,
            diag_mass,
            &is_fixed,
            lc,
            &f0,
            &element_damping,
            u_init.as_deref(),
            lc.initial_case
                .as_ref()
                .and_then(|name| model.case_results.get(name))
                .map_or(&[], |result| result.link_hysteresis.as_slice()),
        )?;
        if let Some(ts) = seed_ts {
            integrator.seed_transient_state(ts, &f0)?;
        }
        for step in 0..=n_steps {
            let t = (step as f64) * dt;
            time_steps.push(t);
            if step > 0 {
                integrator.step(&load_at(t, false), lc)?;
            }
            let u = &integrator.state.u;
            let norm: f64 = u.iter().map(|x| x * x).sum();
            if norm >= peak_norm {
                peak_norm = norm;
                u_peak.clone_from(u);
                v_peak.clone_from(&integrator.state.v);
                a_peak.clone_from(&integrator.state.a);
                t_peak = t;
                nl_link_peak = integrator.current_nonlinear_link_forces();
            }
            for (pname, trace) in &mut time_history_disp {
                let idx = pt_index[pname];
                let disp: [f64; 6] = std::array::from_fn(|d| u[6 * idx + d]);
                trace.push(disp);
            }
        }
        terminal_hysteresis = integrator.terminal_hysteresis();
        if is_direct_history {
            let previous_force = integrator.previous_force().to_vec();
            let nonlinear_link_forces = integrator.terminal_link_force_states();
            terminal_state = Some(sap_model::TransientTerminalState {
                point_order: work_points.keys().cloned().collect(),
                u: integrator.state.u,
                v: integrator.state.v,
                a: integrator.state.a,
                previous_force,
                nonlinear_link_forces,
            });
        }
    } else {
        let f0 = load_at(0.0, false);
        let mut integrator = dynamics::LinearStepper::with_element_damping(
            k_base,
            diag_mass,
            &is_fixed,
            lc,
            &f0,
            &load_at(0.0, true),
            &element_damping,
        )?;
        if let Some(ts) = seed_ts {
            integrator.seed_transient_state(ts, &f0)?;
        }
        for step in 0..=n_steps {
            let t = (step as f64) * dt;
            time_steps.push(t);

            if step > 0 {
                integrator.step(&load_at(t, false), &load_at(t, true))?;
            }
            let u = &integrator.state.u;

            let norm: f64 = u.iter().map(|x| x * x).sum();
            if norm >= peak_norm {
                peak_norm = norm;
                u_peak.clone_from(u);
                v_peak.clone_from(&integrator.state.v);
                a_peak.clone_from(&integrator.state.a);
                t_peak = t;
            }

            for (pname, trace) in &mut time_history_disp {
                let idx = pt_index[pname];
                let disp: [f64; 6] = std::array::from_fn(|d| u[6 * idx + d]);
                trace.push(disp);
            }
        }
        if is_direct_history {
            let previous_force = integrator.previous_force().to_vec();
            terminal_state = Some(sap_model::TransientTerminalState {
                point_order: work_points.keys().cloned().collect(),
                u: integrator.state.u,
                v: integrator.state.v,
                a: integrator.state.a,
                previous_force,
                nonlinear_link_forces: Vec::new(),
            });
        }
    }

    let mut res = recover_results_from_displacement_vector(
        model,
        work_points,
        work_areas,
        orig_area_map,
        pt_index,
        joint_rot,
        &reaction_struct,
        &u_peak,
        ndof,
    );
    // Recover the simultaneous physical reaction, not just its elastic part.
    // Applied loads at fixed DOFs are retained in load_at even though the stepper
    // suppresses their displacement. Rayleigh and link damping both contribute.
    let mut nonlinear_nodal = vec![0.; ndof];
    for object in model.links.values() {
        let prop = &model.link_props[&object.prop];
        let op = link::operator(object, prop, work_points, pt_index, joint_rot);
        for d in 0..6 {
            if let Some(force) = nl_link_peak.get(&(object.name.clone(), d)) {
                for (i, b) in op.dofs.iter().zip(&op.b[d]) {
                    nonlinear_nodal[*i] += force * b;
                }
            }
        }
    }
    let applied_peak = load_at(t_peak, false);
    let (alpha_mass, beta_stiffness) = lc.rayleigh_damping;
    for (name, reaction) in &mut res.joint_reactions {
        let idx = pt_index[name];
        let point = &work_points[name];
        let ks_loc_opt = joint_spring_matrix_local(point, &joint_rot[idx]);
        let mut delta = [0.; 6];
        for d in 0..6 {
            let i = 6 * idx + d;
            if point.restraint[d] {
                let internal: f64 = (0..ndof)
                    .map(|j| {
                        reaction_base[i * ndof + j] * u_peak[j]
                            + (element_damping[i * ndof + j]
                                + beta_stiffness * reaction_base[i * ndof + j])
                                * v_peak[j]
                    })
                    .sum();
                let physical = internal
                    + nonlinear_nodal[i]
                    + alpha_mass * diag_mass[i] * v_peak[i]
                    + diag_mass[i] * a_peak[i]
                    - applied_peak[i];
                delta[d] = physical - reaction[d];
            } else if let Some(ks_loc) = &ks_loc_opt {
                let mut f_damp = 0.0;
                for c in 0..6 {
                    f_damp += ks_loc[d][c] * v_peak[6 * idx + c];
                }
                delta[d] = -beta_stiffness * f_damp;
            }
            reaction[d] += delta[d];
        }
        let r = joint_rot[idx];
        let global: [f64; 6] =
            std::array::from_fn(|i| (0..3).map(|j| r[i % 3][j] * delta[3 * (i / 3) + j]).sum());
        for d in 0..6 {
            res.base_reactions[d] += global[d];
        }
        res.base_reactions[3] += point.y * global[2] - point.z * global[1];
        res.base_reactions[4] += point.z * global[0] - point.x * global[2];
        res.base_reactions[5] += point.x * global[1] - point.y * global[0];
    }
    res.case_name = lc.name.clone();
    res.time_steps = time_steps;
    res.time_history_disp = time_history_disp;
    res.transient_state = terminal_state;
    res.link_hysteresis = terminal_hysteresis;
    let mut grounded = IndexMap::<String, [f64; 6]>::new();
    for (name, object) in &model.links {
        let prop = &model.link_props[&object.prop];
        let op = link::operator(object, prop, work_points, pt_index, joint_rot);
        let snapshot = std::array::from_fn(|d| nl_link_peak.get(&(name.clone(), d)).copied());
        let raw = link::dynamic_spring_forces(prop, &op, &u_peak, &v_peak, &snapshot, 0.);
        res.link_forces
            .insert(name.clone(), link::section_at_j(prop, raw));
        if object.point_j.is_none() {
            let support_raw =
                link::dynamic_spring_forces(prop, &op, &u_peak, &v_peak, &snapshot, beta_stiffness);
            let force = grounded.entry(object.point_i.clone()).or_insert([0.; 6]);
            for (c, value) in force.iter_mut().enumerate() {
                *value -= (0..6).map(|d| op.b[d][c] * support_raw[d]).sum::<f64>();
            }
        }
    }
    add_reaction_contributions(
        grounded,
        work_points,
        pt_index,
        joint_rot,
        &mut res.joint_reactions,
        &mut res.base_reactions,
    );
    Ok(res)
}

#[allow(clippy::too_many_arguments)]
fn recover_results_from_displacement_vector(
    model: &SapModel,
    work_points: &IndexMap<String, PointObj>,
    work_areas: &IndexMap<String, AreaObj>,
    orig_area_map: &IndexMap<String, Vec<String>>,
    pt_index: &IndexMap<String, usize>,
    joint_rot: &[[[f64; 3]; 3]],
    k_struct: &[f64],
    u_joint: &[f64],
    ndof: usize,
) -> CaseResults {
    let mut joint_displacements = IndexMap::new();
    let mut joint_reactions = IndexMap::new();
    let mut base_reactions = [0.0f64; 6];

    for (idx, (pname, pt)) in work_points.iter().enumerate() {
        let disp: [f64; 6] = std::array::from_fn(|d| u_joint[6 * idx + d]);
        joint_displacements.insert(pname.clone(), disp);

        let ks_loc_opt = joint_spring_matrix_local(pt, &joint_rot[idx]);
        let has_support = pt.restraint.iter().any(|&r| r) || ks_loc_opt.is_some();
        if has_support && model.points.contains_key(pname) {
            let mut reac = [0.0f64; 6];
            for d in 0..6 {
                let gdof = 6 * idx + d;
                if pt.restraint[d] {
                    let mut f_int = 0.0;
                    for j in 0..ndof {
                        f_int += k_struct[gdof * ndof + j] * u_joint[j];
                    }
                    reac[d] = f_int;
                } else if let Some(ks_loc) = &ks_loc_opt {
                    let mut f_sp = 0.0;
                    for c in 0..6 {
                        f_sp += ks_loc[d][c] * u_joint[6 * idx + c];
                    }
                    reac[d] = -f_sp;
                }
            }
            joint_reactions.insert(pname.clone(), reac);

            let r_p = &joint_rot[idx];
            let mut r_glob_f = [0.0f64; 3];
            let mut r_glob_m = [0.0f64; 3];
            for g in 0..3 {
                r_glob_f[g] = r_p[g][0] * reac[0] + r_p[g][1] * reac[1] + r_p[g][2] * reac[2];
                r_glob_m[g] = r_p[g][0] * reac[3] + r_p[g][1] * reac[4] + r_p[g][2] * reac[5];
                base_reactions[g] += r_glob_f[g];
                base_reactions[g + 3] += r_glob_m[g];
            }
            base_reactions[3] += pt.y * r_glob_f[2] - pt.z * r_glob_f[1];
            base_reactions[4] += pt.z * r_glob_f[0] - pt.x * r_glob_f[2];
            base_reactions[5] += pt.x * r_glob_f[1] - pt.y * r_glob_f[0];
        }
    }

    let mut frame_forces = IndexMap::new();
    for (fname, frame) in &model.frames {
        let idx_i = pt_index[&frame.point_i];
        let idx_j = pt_index[&frame.point_j];
        let pt_i = &work_points[idx_i];
        let pt_j = &work_points[idx_j];
        let sec = &model.frame_sections[&frame.section];
        let mat = &model.materials[&sec.material];

        let (length, e_axes) = frame_local_axes(
            [pt_i.x, pt_i.y, pt_i.z],
            [pt_j.x, pt_j.y, pt_j.z],
            frame.angle_deg,
        );
        let rz = frame.rigid_zone_factor.clamp(0.0, 1.0);
        let di = rz * frame.end_offset_i;
        let dj = rz * frame.end_offset_j;
        let lf = (length - di - dj).max(1e-6 * length);

        let mut k_loc = compute_flexible_local_stiffness(model, frame, sec, mat, lf);
        let mut dummy_fef = [0.0; 12];
        condense_releases_and_springs(&mut k_loc, &mut dummy_fef, frame);
        apply_rigid_offsets(&mut k_loc, &mut dummy_fef, di, dj);
        // This path serves history and spectrum cases, which validation
        // refuses for a frame with an offset insertion point, so the offset
        // is zero here.
        let insertion = insertion_offset_local(model, frame).unwrap_or([0.0; 3]);
        apply_insertion_offset(&mut k_loc, &mut dummy_fef, insertion);

        let ti = mat3_mul(&e_axes, &joint_rot[idx_i]);
        let tj = mat3_mul(&e_axes, &joint_rot[idx_j]);

        let mut u_loc = [0.0f64; 12];
        for r in 0..3 {
            for c in 0..3 {
                u_loc[r] += ti[r][c] * u_joint[6 * idx_i + c];
                u_loc[r + 3] += ti[r][c] * u_joint[6 * idx_i + 3 + c];
                u_loc[r + 6] += tj[r][c] * u_joint[6 * idx_j + c];
                u_loc[r + 9] += tj[r][c] * u_joint[6 * idx_j + 3 + c];
            }
        }

        let mut f_end_loc = [0.0f64; 12];
        for r in 0..12 {
            for c in 0..12 {
                f_end_loc[r] += k_loc[r][c] * u_loc[c];
            }
        }
        insertion_to_element_ends(&mut u_loc, &mut f_end_loc, insertion);

        let stations = recover_frame_stations(
            model,
            frame,
            sec,
            mat,
            &e_axes,
            length,
            &u_loc,
            &f_end_loc,
            &[],
        );
        frame_forces.insert(fname.clone(), stations);
    }

    let _ = (work_areas, orig_area_map);

    CaseResults {
        case_name: String::new(),
        joint_displacements,
        joint_reactions,
        frame_forces,
        base_reactions,
        ..Default::default()
    }
}

/// Right derivative of the piecewise-linear load function. The function is
/// constant outside its samples, matching eval_dynamic_function's convention.
fn eval_dynamic_function_slope(model: &SapModel, func_name: &str, x: f64) -> f64 {
    let Some(func) = model.functions.get(func_name) else {
        return 0.0;
    };
    for window in func.points.windows(2) {
        let (x0, y0) = window[0];
        let (x1, y1) = window[1];
        if x >= x0 && x < x1 {
            return (y1 - y0) / (x1 - x0);
        }
    }
    0.0
}

fn eval_dynamic_function(model: &SapModel, func_name: &str, x: f64) -> f64 {
    let Some(func) = model.functions.get(func_name) else {
        return 1.0;
    };
    let pts = &func.points;
    if pts.is_empty() {
        return 0.0;
    }
    if x <= pts[0].0 {
        return pts[0].1;
    }
    if x >= pts[pts.len() - 1].0 {
        return pts[pts.len() - 1].1;
    }
    for w in pts.windows(2) {
        let (x0, y0) = w[0];
        let (x1, y1) = w[1];
        if x >= x0 && x <= x1 {
            let dx = x1 - x0;
            if dx.abs() <= 1e-18 {
                return y0;
            }
            return y0 + (y1 - y0) * ((x - x0) / dx);
        }
    }
    pts[pts.len() - 1].1
}

/// Evaluates all `LoadCombo`s in `model.load_combos` in topological order and stores
/// combined `CaseResults` in `model.case_results` (safely skipping cyclic or empty combos).
fn evaluate_load_combos(model: &mut SapModel) {
    let n_combos = model.load_combos.len();
    if n_combos == 0 {
        return;
    }
    let combo_idx_map: IndexMap<String, usize> = model
        .load_combos
        .keys()
        .enumerate()
        .map(|(idx, name)| (name.clone(), idx))
        .collect();
    let mut in_degree = vec![0usize; n_combos];
    let mut dependents: Vec<Vec<usize>> = vec![Vec::new(); n_combos];
    for (idx, combo) in model.load_combos.values().enumerate() {
        let mut seen_deps = std::collections::BTreeSet::new();
        for (src_name, _) in &combo.cases {
            if model.case_results.contains_key(src_name) {
                continue;
            }
            if let Some(&dep_idx) = combo_idx_map.get(src_name) {
                if seen_deps.insert(dep_idx) {
                    in_degree[idx] += 1;
                    if dep_idx != idx {
                        dependents[dep_idx].push(idx);
                    }
                }
            }
        }
    }

    let mut ready = std::collections::BinaryHeap::new();
    for (idx, &deg) in in_degree.iter().enumerate() {
        if deg == 0 {
            ready.push(std::cmp::Reverse(idx));
        }
    }
    let mut topo_order = Vec::with_capacity(n_combos);
    while let Some(std::cmp::Reverse(idx)) = ready.pop() {
        topo_order.push(idx);
        for &next in &dependents[idx] {
            in_degree[next] -= 1;
            if in_degree[next] == 0 {
                ready.push(std::cmp::Reverse(next));
            }
        }
    }

    for combo_idx in topo_order {
        let combo = model.load_combos[combo_idx].clone();
        let mut comb_res = CaseResults {
            case_name: combo.name.clone(),
            ..Default::default()
        };

        let combine_op = |acc: f64, val: f64, sf: f64, is_first: bool| -> f64 {
            match combo.combo_type {
                ComboType::LinearAdd | ComboType::RangeAdd => acc + val * sf,
                ComboType::AbsAdd => acc + (val * sf).abs(),
                ComboType::Srss => acc + (val * sf).powi(2),
                ComboType::Envelope => {
                    let v = val * sf;
                    if is_first || v.abs() > acc.abs() {
                        v
                    } else {
                        acc
                    }
                }
            }
        };
        let finalize_op = |acc: f64| -> f64 {
            if combo.combo_type == ComboType::Srss {
                acc.max(0.0).sqrt()
            } else {
                acc
            }
        };

        let combine_stress = |dst: &mut SolidStress, src: &SolidStress, sf: f64, is_first: bool| {
            dst.s11 = combine_op(dst.s11, src.s11, sf, is_first);
            dst.s22 = combine_op(dst.s22, src.s22, sf, is_first);
            dst.s33 = combine_op(dst.s33, src.s33, sf, is_first);
            dst.s12 = combine_op(dst.s12, src.s12, sf, is_first);
            dst.s13 = combine_op(dst.s13, src.s13, sf, is_first);
            dst.s23 = combine_op(dst.s23, src.s23, sf, is_first);
            dst.s_max = combine_op(dst.s_max, src.s_max, sf, is_first);
            dst.s_mid = combine_op(dst.s_mid, src.s_mid, sf, is_first);
            dst.s_min = combine_op(dst.s_min, src.s_min, sf, is_first);
            dst.von_mises = combine_op(dst.von_mises, src.von_mises, sf, is_first);
        };
        let finalize_stress = |dst: &mut SolidStress| {
            dst.s11 = finalize_op(dst.s11);
            dst.s22 = finalize_op(dst.s22);
            dst.s33 = finalize_op(dst.s33);
            dst.s12 = finalize_op(dst.s12);
            dst.s13 = finalize_op(dst.s13);
            dst.s23 = finalize_op(dst.s23);
            dst.s_max = finalize_op(dst.s_max);
            dst.s_mid = finalize_op(dst.s_mid);
            dst.s_min = finalize_op(dst.s_min);
            dst.von_mises = finalize_op(dst.von_mises);
        };

        let mut contributed = 0usize;
        for (src_name, sf) in &combo.cases {
            let Some(src) = model.case_results.get(src_name).cloned() else {
                continue;
            };
            let is_first = contributed == 0;
            contributed += 1;

            for (pname, d) in &src.joint_displacements {
                let key_first = is_first || !comb_res.joint_displacements.contains_key(pname);
                let entry = comb_res
                    .joint_displacements
                    .entry(pname.clone())
                    .or_insert([0.0; 6]);
                for k in 0..6 {
                    entry[k] = combine_op(entry[k], d[k], *sf, key_first);
                }
            }

            for (pname, r) in &src.joint_reactions {
                let key_first = is_first || !comb_res.joint_reactions.contains_key(pname);
                let entry = comb_res
                    .joint_reactions
                    .entry(pname.clone())
                    .or_insert([0.0; 6]);
                for k in 0..6 {
                    entry[k] = combine_op(entry[k], r[k], *sf, key_first);
                }
            }

            for k in 0..6 {
                comb_res.base_reactions[k] = combine_op(
                    comb_res.base_reactions[k],
                    src.base_reactions[k],
                    *sf,
                    is_first,
                );
            }

            for (fname, stations) in &src.frame_forces {
                let key_first = is_first || !comb_res.frame_forces.contains_key(fname);
                let entry = comb_res
                    .frame_forces
                    .entry(fname.clone())
                    .or_insert_with(|| {
                        stations
                            .iter()
                            .map(|s| FrameStationResult {
                                station: s.station,
                                rel_dist: s.rel_dist,
                                ..Default::default()
                            })
                            .collect()
                    });
                for (dst_s, src_s) in entry.iter_mut().zip(stations.iter()) {
                    dst_s.p = combine_op(dst_s.p, src_s.p, *sf, key_first);
                    dst_s.v2 = combine_op(dst_s.v2, src_s.v2, *sf, key_first);
                    dst_s.v3 = combine_op(dst_s.v3, src_s.v3, *sf, key_first);
                    dst_s.t = combine_op(dst_s.t, src_s.t, *sf, key_first);
                    dst_s.m2 = combine_op(dst_s.m2, src_s.m2, *sf, key_first);
                    dst_s.m3 = combine_op(dst_s.m3, src_s.m3, *sf, key_first);
                    for k in 0..3 {
                        dst_s.local_disp[k] =
                            combine_op(dst_s.local_disp[k], src_s.local_disp[k], *sf, key_first);
                    }
                }
            }

            for (aname, ar) in &src.area_results {
                let key_first = is_first || !comb_res.area_results.contains_key(aname);
                let entry = comb_res.area_results.entry(aname.clone()).or_default();
                entry.f11 = combine_op(entry.f11, ar.f11, *sf, key_first);
                entry.f22 = combine_op(entry.f22, ar.f22, *sf, key_first);
                entry.f12 = combine_op(entry.f12, ar.f12, *sf, key_first);
                entry.m11 = combine_op(entry.m11, ar.m11, *sf, key_first);
                entry.m22 = combine_op(entry.m22, ar.m22, *sf, key_first);
                entry.m12 = combine_op(entry.m12, ar.m12, *sf, key_first);
                entry.v13 = combine_op(entry.v13, ar.v13, *sf, key_first);
                entry.v23 = combine_op(entry.v23, ar.v23, *sf, key_first);
                entry.s11_top = combine_op(entry.s11_top, ar.s11_top, *sf, key_first);
                entry.s22_top = combine_op(entry.s22_top, ar.s22_top, *sf, key_first);
                entry.s12_top = combine_op(entry.s12_top, ar.s12_top, *sf, key_first);
                entry.svm_top = combine_op(entry.svm_top, ar.svm_top, *sf, key_first);
            }

            for (sname, sr) in &src.solid_results {
                let key_first = is_first || !comb_res.solid_results.contains_key(sname);
                let entry = comb_res.solid_results.entry(sname.clone()).or_default();
                combine_stress(&mut entry.center_stress, &sr.center_stress, *sf, key_first);
                for g in 0..8 {
                    combine_stress(
                        &mut entry.gauss_stresses[g],
                        &sr.gauss_stresses[g],
                        *sf,
                        key_first,
                    );
                    combine_stress(
                        &mut entry.nodal_stresses[g],
                        &sr.nodal_stresses[g],
                        *sf,
                        key_first,
                    );
                }
                for k in 0..8 {
                    for d in 0..3 {
                        entry.nodal_forces[k][d] = combine_op(
                            entry.nodal_forces[k][d],
                            sr.nodal_forces[k][d],
                            *sf,
                            key_first,
                        );
                    }
                }
            }

            for (lname, lf) in &src.link_forces {
                let key_first = is_first || !comb_res.link_forces.contains_key(lname);
                let entry = comb_res
                    .link_forces
                    .entry(lname.clone())
                    .or_insert([0.0; 6]);
                for k in 0..6 {
                    entry[k] = combine_op(entry[k], lf[k], *sf, key_first);
                }
            }
        }

        if contributed == 0 {
            continue;
        }

        if combo.combo_type == ComboType::Srss {
            for d in comb_res.joint_displacements.values_mut() {
                for v in d {
                    *v = finalize_op(*v);
                }
            }
            for r in comb_res.joint_reactions.values_mut() {
                for v in r {
                    *v = finalize_op(*v);
                }
            }
            for v in &mut comb_res.base_reactions {
                *v = finalize_op(*v);
            }
            for stations in comb_res.frame_forces.values_mut() {
                for s in stations {
                    s.p = finalize_op(s.p);
                    s.v2 = finalize_op(s.v2);
                    s.v3 = finalize_op(s.v3);
                    s.t = finalize_op(s.t);
                    s.m2 = finalize_op(s.m2);
                    s.m3 = finalize_op(s.m3);
                    for v in &mut s.local_disp {
                        *v = finalize_op(*v);
                    }
                }
            }
            for ar in comb_res.area_results.values_mut() {
                ar.f11 = finalize_op(ar.f11);
                ar.f22 = finalize_op(ar.f22);
                ar.f12 = finalize_op(ar.f12);
                ar.m11 = finalize_op(ar.m11);
                ar.m22 = finalize_op(ar.m22);
                ar.m12 = finalize_op(ar.m12);
                ar.v13 = finalize_op(ar.v13);
                ar.v23 = finalize_op(ar.v23);
                ar.s11_top = finalize_op(ar.s11_top);
                ar.s22_top = finalize_op(ar.s22_top);
                ar.s12_top = finalize_op(ar.s12_top);
                ar.svm_top = finalize_op(ar.svm_top);
            }
            for sr in comb_res.solid_results.values_mut() {
                finalize_stress(&mut sr.center_stress);
                for g in 0..8 {
                    finalize_stress(&mut sr.gauss_stresses[g]);
                    finalize_stress(&mut sr.nodal_stresses[g]);
                }
                for nf in &mut sr.nodal_forces {
                    for v in nf {
                        *v = finalize_op(*v);
                    }
                }
            }
            for lf in comb_res.link_forces.values_mut() {
                for v in lf {
                    *v = finalize_op(*v);
                }
            }
        }

        model.case_results.insert(combo.name.clone(), comb_res);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sap_model::{
        ConstraintAxis, ConstraintDef, ConstraintType, FrameLoadDistributed, LoadCase,
        ModalSubType, NLGeomType, PointLoadForce,
    };
    use sap_units::Units;

    #[test]
    fn test_finding_1_unstable_pinned_cantilever_rejected() {
        // A beam pinned at joint 1 (RY free) with a free tip at joint 2 under vertical load
        // is a kinematic mechanism and MUST return an error.
        let mut model = SapModel::new(Units::KipInF);
        model.add_point("1", 0.0, 0.0, 0.0);
        model.points.get_mut("1").unwrap().restraint = [true, true, true, true, false, true];
        model.add_point("2", 120.0, 0.0, 0.0);
        model
            .points
            .get_mut("2")
            .unwrap()
            .loads_force
            .push(PointLoadForce {
                load_pat: "DEAD".into(),
                csys: "GLOBAL".into(),
                force: [0.0, 0.0, -10.0, 0.0, 0.0, 0.0],
            });
        model.add_frame("1", "1", "2", "FSEC1");

        let res = analyze_model(&mut model);
        assert!(
            res.is_err(),
            "Expected unstable pinned cantilever mechanism to fail, but got Ok"
        );
    }

    #[test]
    fn test_finding_2_m2_moment_diagram_in_1_3_plane() {
        // Simply supported beam of length L = 120 in loaded in local 3 direction (Global Y)
        // with uniform load w = 0.2 kip/in -> mid-span M2 must have magnitude w * L^2 / 8 = 360 kip-in.
        let mut model = SapModel::new(Units::KipInF);
        if let Some(pat) = model.load_patterns.get_mut("DEAD") {
            pat.self_wt_multiplier = 0.0;
        }
        model.add_point("1", 0.0, 0.0, 0.0);
        model.points.get_mut("1").unwrap().restraint = [true, true, true, true, true, false];
        model.add_point("2", 120.0, 0.0, 0.0);
        model.points.get_mut("2").unwrap().restraint = [false, true, true, true, true, false];
        model.add_frame("1", "1", "2", "FSEC1");
        model
            .frames
            .get_mut("1")
            .unwrap()
            .loads_dist
            .push(FrameLoadDistributed {
                load_pat: "DEAD".into(),
                my_type: 1,
                dir: 3, // Local 3
                csys: "LOCAL".into(),
                rel_dist1: 0.0,
                rel_dist2: 1.0,
                val1: 0.2,
                val2: 0.2,
            });

        analyze_model(&mut model).expect("Simple beam in 1-3 plane should solve");
        let stations = &model.case_results["DEAD"].frame_forces["1"];
        let mid = stations
            .iter()
            .find(|s| (s.rel_dist - 0.5).abs() < 1e-6)
            .expect("mid-span station");
        let expected_m2 = 0.2 * 120.0_f64.powi(2) / 8.0; // 360.0
        assert!(
            (mid.m2.abs() - expected_m2).abs() < 1e-3,
            "Expected |M2| at mid-span = {}, got {}",
            expected_m2,
            mid.m2
        );
    }

    #[test]
    fn test_finding_3_and_4_pdelta_with_releases_and_pdelta_force() {
        // Cantilever column with end release at tip and P-Delta analysis
        let mut model = SapModel::new(Units::KipInF);
        if let Some(pat) = model.load_patterns.get_mut("DEAD") {
            pat.self_wt_multiplier = 0.0;
        }
        model.add_point("1", 0.0, 0.0, 0.0);
        model.points.get_mut("1").unwrap().restraint = [true; 6];
        model.add_point("2", 0.0, 0.0, 120.0);
        model
            .points
            .get_mut("2")
            .unwrap()
            .loads_force
            .push(PointLoadForce {
                load_pat: "DEAD".into(),
                csys: "GLOBAL".into(),
                force: [5.0, 0.0, -50.0, 0.0, 0.0, 0.0],
            });
        model.add_frame("1", "1", "2", "FSEC1");
        model.frames.get_mut("1").unwrap().release_j[5] = true;

        let mut pd_case = LoadCase::linear_static("PDELTA", "DEAD", 1.0);
        pd_case.case_type = LoadCaseType::NonlinearStatic;
        pd_case.nl_geom = NLGeomType::PDelta;
        model.load_cases.insert("PDELTA".into(), pd_case);

        analyze_model(&mut model).expect("P-Delta analysis should succeed");
        let ux_lin = model.case_results["DEAD"].joint_displacements["2"][0];
        let ux_pd = model.case_results["PDELTA"].joint_displacements["2"][0];
        assert!(
            ux_pd > ux_lin,
            "Compressive P-Delta must amplify lateral displacement: ux_pd={} vs ux_lin={}",
            ux_pd,
            ux_lin
        );
    }

    #[test]
    fn test_finding_5_base_reaction_moments_and_ritz_and_diaphragm() {
        let mut model = SapModel::new(Units::KipInF);
        if let Some(pat) = model.load_patterns.get_mut("DEAD") {
            pat.self_wt_multiplier = 0.0;
        }
        // Portal frame with supports at x = 0 and x = 100
        model.add_point("1", 0.0, 0.0, 0.0);
        model.points.get_mut("1").unwrap().restraint = [true; 6];
        model.add_point("2", 100.0, 0.0, 0.0);
        model.points.get_mut("2").unwrap().restraint = [true; 6];
        model.add_point("3", 0.0, 0.0, 100.0);
        model.points.get_mut("3").unwrap().mass = [0.05, 0.05, 0.05, 0.0, 0.0, 0.0];
        model
            .points
            .get_mut("3")
            .unwrap()
            .constraints
            .push("DIAPH1".into());
        model.add_point("4", 100.0, 0.0, 100.0);
        model.points.get_mut("4").unwrap().mass = [0.05, 0.05, 0.05, 0.0, 0.0, 0.0];
        model
            .points
            .get_mut("4")
            .unwrap()
            .constraints
            .push("DIAPH1".into());
        model
            .points
            .get_mut("4")
            .unwrap()
            .loads_force
            .push(PointLoadForce {
                load_pat: "DEAD".into(),
                csys: "GLOBAL".into(),
                force: [0.0, 0.0, -20.0, 0.0, 0.0, 0.0],
            });

        model.add_frame("C1", "1", "3", "FSEC1");
        model.add_frame("C2", "2", "4", "FSEC1");
        model.add_frame("B1", "3", "4", "FSEC1");

        // Diaphragm about Y axis (XZ plane)
        model.constraints.insert(
            "DIAPH1".into(),
            ConstraintDef {
                name: "DIAPH1".into(),
                constraint_type: ConstraintType::Diaphragm,
                axis: ConstraintAxis::Y,
                dofs: [true; 6],
            },
        );

        let mut ritz_case = LoadCase::modal_eigen("RITZ", 2);
        ritz_case.modal_sub_type = ModalSubType::Ritz;
        model.load_cases.insert("RITZ".into(), ritz_case);

        analyze_model(&mut model).expect("Frame with Y-diaphragm and Ritz case should solve");
        let br = model.case_results["DEAD"].base_reactions;
        // Total vertical reaction Fz = +20 kips at x = 100 -> global moment about origin My = -x * Fz = -2000 kip-in
        assert!((br[2] - 20.0).abs() < 1e-3, "Fz = {}", br[2]);
        assert!(
            (br[4] - (-2000.0)).abs() < 1e-2,
            "Expected base reaction My = -2000.0 (including r x F), got {}",
            br[4]
        );
        assert!(!model.case_results["RITZ"].modes.is_empty());
    }
}
