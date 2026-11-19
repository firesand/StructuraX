//! Explicit boundaries for the wave-4 formulations. Unsupported assignments must
//! fail before assembly instead of being silently discarded by a narrower path.
use crate::SolverError;
use sap_model::{LoadCaseType, NLGeomType, SapModel};

pub(crate) fn validate(model: &SapModel) -> Result<(), SolverError> {
    let unsupported = |reason: String| SolverError::UnsupportedInput(reason);
    let has_cables = model.frames.values().any(|f| f.cable_params.is_some());
    for case in model.load_cases.values().filter(|c| c.run_flag) {
        let history = matches!(
            case.case_type,
            LoadCaseType::LinearHistory
                | LoadCaseType::LinearDynamic
                | LoadCaseType::NonlinearHistory
                | LoadCaseType::NonlinearDynamic
        );
        if let Some(initial) = case
            .initial_case
            .as_ref()
            .and_then(|n| model.load_cases.get(n))
        {
            if matches!(
                initial.case_type,
                LoadCaseType::NonlinearHistory
                    | LoadCaseType::NonlinearDynamic
                    | LoadCaseType::LinearDynamic
                    | LoadCaseType::LinearHistory
            ) && !matches!(
                case.case_type,
                LoadCaseType::NonlinearStatic
                    | LoadCaseType::NonlinearHistory
                    | LoadCaseType::NonlinearDynamic
                    | LoadCaseType::LinearDynamic
                    | LoadCaseType::LinearHistory
            ) {
                return Err(unsupported(format!(
                    "case '{}' requires history continuation into {:?}, which is not supported",
                    case.name, case.case_type
                )));
            }
        }
        if matches!(
            case.case_type,
            LoadCaseType::NonlinearHistory | LoadCaseType::NonlinearDynamic
        ) && case.time_integration == sap_model::TimeIntegrationMethod::WilsonTheta
            && model.link_props.values().any(|p| {
                p.link_type == sap_model::LinkPropType::Damper
                    && (0..6).any(|d| p.dof_active[d] && p.dof_nonlinear[d] && !p.dof_fixed[d])
            })
        {
            return Err(unsupported(format!(
                "case '{}' Wilson-theta Maxwell internal-state integration is not implemented",
                case.name
            )));
        }
        if matches!(
            case.case_type,
            LoadCaseType::NonlinearHistory | LoadCaseType::NonlinearDynamic
        ) && case.time_integration == sap_model::TimeIntegrationMethod::WilsonTheta
            && model.link_props.values().any(|p| {
                p.link_type == sap_model::LinkPropType::PlasticWen
                    && (0..6).any(|d| p.dof_active[d] && p.dof_nonlinear[d] && !p.dof_fixed[d])
            })
        {
            return Err(unsupported(format!(
                "case '{}' Wilson-theta integration of plastic Wen links is not implemented",
                case.name
            )));
        }
        if history && case.nl_geom != NLGeomType::None {
            return Err(unsupported(format!(
                "case '{}' requests geometric nonlinearity in history integration",
                case.name
            )));
        }
        if history {
            let mut patterns: Vec<String> = case
                .history_loads
                .iter()
                .enumerate()
                .filter(|(i, (_, _, sf))| *sf != 0. && case.history_entry_is_pattern(model, *i))
                .map(|(_, (_, p, _))| p.clone())
                .collect();
            if let Some(initial) = case
                .initial_case
                .as_ref()
                .and_then(|n| model.load_cases.get(n))
            {
                patterns.extend(
                    super::accumulated_nonlinear_static_loads(model, initial)?
                        .into_iter()
                        .filter(|(_, sf)| *sf != 0.)
                        .map(|(p, _)| p),
                );
            }
            if model.solids.values().any(|s| {
                s.loads_surface
                    .iter()
                    .any(|l| patterns.contains(&l.load_pat) && l.val != 0.)
                    || s.loads_temp
                        .iter()
                        .any(|(p, t)| patterns.contains(p) && *t != 0.)
            }) {
                return Err(unsupported(format!(
                    "case '{}' has solid surface or temperature history loading",
                    case.name
                )));
            }
            if model.points.values().any(|p| {
                p.loads_displ
                    .iter()
                    .any(|l| patterns.contains(&l.load_pat) && l.displ.iter().any(|x| *x != 0.))
            }) {
                return Err(unsupported(format!(
                    "case '{}' cannot continue prescribed support motion into history integration",
                    case.name
                )));
            }
        }
        if has_cables {
            if case.case_type != LoadCaseType::NonlinearStatic {
                return Err(unsupported(format!("case '{}' contains cables; only cable-only nonlinear static analysis is implemented", case.name)));
            }
            if model.frames.values().any(|f| f.cable_params.is_none())
                || !model.links.is_empty()
                || !model.areas.is_empty()
                || !model.solids.is_empty()
            {
                return Err(unsupported(
                    "cable-only analysis cannot mix ordinary frames, links, areas or solids".into(),
                ));
            }
            if model.points.values().any(|p| {
                p.spring.iter().any(|s| *s != 0.)
                    || p.spring_coupled.is_some()
                    || !p.constraints.is_empty()
                    || p.local_axes_deg.iter().any(|a| *a != 0.)
                    || p.local_axes_matrix.is_some()
            }) {
                return Err(unsupported("cable-only analysis does not support joint springs, multipoint constraints or rotated joint axes".into()));
            }
            let loads = super::accumulated_nonlinear_static_loads(model, case)?;
            let active = |p: &str| loads.iter().any(|(name, sf)| name == p && *sf != 0.);
            for p in model.points.values() {
                if p.loads_force
                    .iter()
                    .any(|l| active(&l.load_pat) && l.force[3..].iter().any(|x| *x != 0.))
                    || p.loads_displ
                        .iter()
                        .any(|l| active(&l.load_pat) && l.displ[3..].iter().any(|x| *x != 0.))
                {
                    return Err(unsupported(
                        "cable-only analysis cannot apply joint moments or prescribed rotations"
                            .into(),
                    ));
                }
            }
            for f in model.frames.values() {
                let cp = f.cable_params.as_ref().expect("checked cable-only model");
                if cp
                    .undeformed_length
                    .is_some_and(|v| !v.is_finite() || v <= 0.0)
                    || cp
                        .target_tension_i
                        .is_some_and(|v| !v.is_finite() || v <= 0.0)
                    || cp
                        .horizontal_tension
                        .is_some_and(|v| !v.is_finite() || v <= 0.0)
                {
                    return Err(SolverError::InvalidInput(format!(
                        "cable '{}' shape initialization parameter must be finite and positive",
                        f.name
                    )));
                }
                let spec_count = usize::from(cp.undeformed_length.is_some())
                    + usize::from(cp.target_tension_i.is_some())
                    + usize::from(cp.horizontal_tension.is_some());
                if spec_count > 1 {
                    return Err(unsupported(format!(
                        "cable '{}' specifies multiple competing shape parameters (undeformed length, target end tension, or horizontal tension)",
                        f.name
                    )));
                }
                if f.release_i.iter().chain(&f.release_j).any(|r| *r)
                    || f.end_offset_i != 0.
                    || f.end_offset_j != 0.
                    || f.joint_offset_i
                        .iter()
                        .chain(&f.joint_offset_j)
                        .any(|x| *x != 0.)
                    || f.tc_limits
                        .is_some_and(|(c, lim_c, t, _)| (c && lim_c != 0.0) || t)
                    || model
                        .frame_sections
                        .get(&f.section)
                        .is_some_and(|s| !s.non_prismatic.is_empty())
                {
                    return Err(unsupported(format!("cable '{}' has releases, offsets, member limits or nonprismatic properties", f.name)));
                }
                if f.loads_point
                    .iter()
                    .any(|l| active(&l.load_pat) && l.val != 0.)
                    || f.loads_dist.iter().any(|l| {
                        active(&l.load_pat)
                            && (l.val1 != 0. || l.val2 != 0.)
                            && (l.my_type != 1
                                || l.rel_dist1 != 0.
                                || l.rel_dist2 != 1.
                                || l.val1 != l.val2)
                    })
                    || f.loads_temp.iter().any(|l| {
                        active(&l.load_pat)
                            && l.val != 0.
                            && (l.my_type != 1 || l.pattern_name.is_some())
                    })
                {
                    return Err(unsupported(format!("cable '{}' supports only full-span uniform force loads and uniform temperature; split the cable at point or changing loads", f.name)));
                }
            }
        }
        for f in model
            .frames
            .values()
            .filter(|f| f.cable_params.is_none() && f.tc_limits.is_some_and(|(c, _, t, _)| c || t))
        {
            if history
                || (case.case_type == LoadCaseType::NonlinearStatic
                    && case.nl_geom != NLGeomType::None)
            {
                return Err(unsupported(format!(
                    "frame '{}' member limits require small-displacement nonlinear static analysis",
                    f.name
                )));
            }
            if case.case_type != LoadCaseType::NonlinearStatic {
                continue;
            }
            if f.release_i.iter().chain(&f.release_j).any(|r| *r)
                || f.end_offset_i != 0.
                || f.end_offset_j != 0.
                || f.joint_offset_i
                    .iter()
                    .chain(&f.joint_offset_j)
                    .any(|x| *x != 0.)
                || model
                    .frame_sections
                    .get(&f.section)
                    .is_some_and(|s| !s.non_prismatic.is_empty())
            {
                return Err(unsupported(format!("frame '{}' member limits with releases, offsets or nonprismatic properties are not implemented", f.name)));
            }
            if let Some(reason) = tc_axial_span_load_refusal(model, case, f)? {
                return Err(unsupported(reason));
            }
        }
    }
    Ok(())
}

/// D111 (G75): the tension/compression limits of a frame are treated per extension and
/// shortening (Analysis Reference p. 133, which says nothing about span loads). The solver
/// estimates the extreme axial forces of a member as `p_avg +- |W|/2`, which is exact only when
/// the axial span load keeps one sign, and the limit law is discontinuous when the resultant
/// `|W|` exceeds `T + |C|`. Both cases are refused by name instead of approximated.
fn tc_axial_span_load_refusal(
    model: &SapModel,
    case: &sap_model::LoadCase,
    frame: &sap_model::FrameObj,
) -> Result<Option<String>, SolverError> {
    use crate::frame::{frame_local_axes, resolve_load_dir_to_local};
    let (Some(pi), Some(pj)) = (
        model.points.get(&frame.point_i),
        model.points.get(&frame.point_j),
    ) else {
        return Ok(None);
    };
    let (length, e_axes) = frame_local_axes(
        [pi.x, pi.y, pi.z],
        [pj.x, pj.y, pj.z],
        frame.angle_deg,
    );
    let weight_per_len = model
        .frame_sections
        .get(&frame.section)
        .and_then(|sec| {
            model.materials.get(&sec.material).map(|mat| {
                mat.unit_weight * sec.area * sec.modifiers[7] * frame.modifiers[7]
            })
        })
        .unwrap_or(0.0);
    // Axial components (local 1) of the active span loads: the values that set the sign, and
    // the resultant W.
    let mut values: Vec<f64> = Vec::new();
    let mut resultant = 0.0f64;
    for (pattern, scale) in super::accumulated_nonlinear_static_loads(model, case)? {
        if scale == 0.0 {
            continue;
        }
        let self_weight = model
            .load_patterns
            .get(&pattern)
            .map_or(0.0, |lp| lp.self_wt_multiplier);
        if self_weight != 0.0 {
            // self weight acts along -Z: component on local 1
            let q = -e_axes[0][2] * self_weight * weight_per_len * scale;
            values.push(q);
            resultant += q * length;
        }
        if let Some(g) = frame.loads_gravity.get(&pattern) {
            let q: f64 = (0..3).map(|d| e_axes[0][d] * g[d]).sum::<f64>() * weight_per_len * scale;
            values.push(q);
            resultant += q * length;
        }
        for ld in frame
            .loads_dist
            .iter()
            .filter(|l| l.load_pat == pattern && l.my_type == 1)
        {
            let q1 = resolve_load_dir_to_local(ld.dir, ld.val1 * scale, &e_axes, true)[0];
            let q2 = resolve_load_dir_to_local(ld.dir, ld.val2 * scale, &e_axes, true)[0];
            values.push(q1);
            values.push(q2);
            resultant += 0.5 * (q1 + q2) * (ld.rel_dist2 - ld.rel_dist1) * length;
        }
        for lp in frame
            .loads_point
            .iter()
            .filter(|l| l.load_pat == pattern && l.my_type == 1)
        {
            let p = resolve_load_dir_to_local(lp.dir, lp.val * scale, &e_axes, false)[0];
            values.push(p);
            resultant += p;
        }
    }
    let largest = values.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    let eps = 1e-12 * largest;
    let positive = values.iter().any(|v| *v > eps);
    let negative = values.iter().any(|v| *v < -eps);
    if positive && negative {
        return Ok(Some(format!(
            "frame '{}' has tension/compression limits and axial span loads of both signs in case '{}': the extreme axial force inside the member is not implemented (G75)",
            frame.name, case.name
        )));
    }
    if let Some((true, c, true, t)) = frame.tc_limits {
        if resultant.abs() > t.abs() + c.abs() * (1.0 + 1e-12) {
            return Ok(Some(format!(
                "frame '{}' has tension limit {} and compression limit {} but the axial span load of case '{}' has resultant {}, more than their sum: the member has no admissible state (G75)",
                frame.name,
                t.abs(),
                c.abs(),
                case.name,
                resultant.abs()
            )));
        }
    }
    Ok(None)
}
