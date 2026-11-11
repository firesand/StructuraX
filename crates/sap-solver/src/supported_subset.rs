//! Explicit boundaries for the wave-4 formulations. Unsupported assignments must
//! fail before assembly instead of being silently discarded by a narrower path.
use crate::{frame, SolverError};
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
            if initial.case_type == LoadCaseType::LinearHistory {
                return Err(unsupported(format!(
                    "case '{}' requires modal history continuation from '{}', which is not supported",
                    case.name, initial.name
                )));
            }
            if matches!(
                initial.case_type,
                LoadCaseType::NonlinearHistory
                    | LoadCaseType::NonlinearDynamic
                    | LoadCaseType::LinearDynamic
            ) && !matches!(
                case.case_type,
                LoadCaseType::NonlinearStatic
                    | LoadCaseType::NonlinearHistory
                    | LoadCaseType::NonlinearDynamic
                    | LoadCaseType::LinearDynamic
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
                .filter(|(_, _, sf)| *sf != 0.)
                .map(|(_, p, _)| p.clone())
                .collect();
            if let Some(initial) = case
                .initial_case
                .as_ref()
                .and_then(|n| model.load_cases.get(n))
            {
                patterns.extend(
                    super::accumulated_nonlinear_static_loads(model, initial)
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
            let loads = super::accumulated_nonlinear_static_loads(model, case);
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
                if cp.undeformed_length.is_some_and(|v| !v.is_finite() || v <= 0.0)
                    || cp.target_tension_i.is_some_and(|v| !v.is_finite() || v <= 0.0)
                    || cp.horizontal_tension.is_some_and(|v| !v.is_finite() || v <= 0.0)
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
            let pi = model
                .points
                .get(&f.point_i)
                .ok_or_else(|| SolverError::PointNotFound(f.point_i.clone()))?;
            let pj = model
                .points
                .get(&f.point_j)
                .ok_or_else(|| SolverError::PointNotFound(f.point_j.clone()))?;
            let (_, axes) =
                frame::frame_local_axes([pi.x, pi.y, pi.z], [pj.x, pj.y, pj.z], f.angle_deg);
            let axial = |dir, value| {
                frame::resolve_load_dir_to_local(dir, value, &axes, true)[0].abs()
                    > 1e-14 * value.abs()
            };
            let loads = super::accumulated_nonlinear_static_loads(model, case);
            let active = |p: &str| loads.iter().any(|(name, sf)| name == p && *sf != 0.);
            if f.loads_dist.iter().any(|l| {
                active(&l.load_pat)
                    && l.my_type == 1
                    && (axial(l.dir, l.val1) || axial(l.dir, l.val2))
            }) || f
                .loads_point
                .iter()
                .any(|l| active(&l.load_pat) && l.my_type == 1 && axial(l.dir, l.val))
                || loads.iter().any(|(p, sf)| {
                    *sf != 0.
                        && ((model
                            .load_patterns
                            .get(p)
                            .is_some_and(|pat| pat.self_wt_multiplier != 0.)
                            && axes[0][2].abs() > 1e-14)
                            || f.loads_gravity.get(p).is_some_and(|g| {
                                (0..3).map(|i| axes[0][i] * g[i]).sum::<f64>().abs() > 1e-14
                            }))
                })
            {
                return Err(unsupported(format!(
                    "frame '{}' member limits with axial span loads are not implemented",
                    f.name
                )));
            }
        }
    }
    Ok(())
}
