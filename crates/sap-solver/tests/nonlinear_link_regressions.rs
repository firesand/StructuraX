use sap_model::*;
use sap_solver::analyze_model;

fn close(a: f64, b: f64) {
    assert!((a - b).abs() < 2e-7 * (1. + b.abs()), "{a} != {b}");
}
fn axial(kind: LinkPropType, opening: f64, load: f64, parallel: f64) -> SapModel {
    let mut m = SapModel::default();
    m.load_patterns["DEAD"].self_wt_multiplier = 0.;
    m.load_cases.clear();
    let mut case = LoadCase::linear_static("NL", "DEAD", 1.);
    case.case_type = LoadCaseType::NonlinearStatic;
    case.convergence_tol = 1e-10;
    m.load_cases.insert("NL".into(), case);
    m.active_dofs = [true, false, false, false, false, false];
    m.add_point("I", 0., 0., 0.);
    m.add_point("J", 3., 0., 0.);
    m.points["I"].restraint = [true; 6];
    m.points["J"].spring[0] = parallel;
    m.points["J"].loads_force.push(PointLoadForce {
        load_pat: "DEAD".into(),
        csys: "GLOBAL".into(),
        force: [load, 0., 0., 0., 0., 0.],
    });
    let mut prop = LinkProp {
        name: "P".into(),
        link_type: kind,
        ..LinkProp::default()
    };
    prop.dof_nonlinear[0] = true;
    prop.ke[0] = 13.;
    prop.k_nl[0] = 100.;
    prop.param1[0] = opening;
    m.link_props.insert("P".into(), prop);
    m.links.insert(
        "L".into(),
        LinkObj {
            name: "L".into(),
            point_i: "I".into(),
            point_j: Some("J".into()),
            prop: "P".into(),
            angle_deg: 0.,
            selected: false,
        },
    );
    m
}

#[test]
fn gap_and_hook_open_close_with_exact_parallel_spring_equilibrium() {
    for (kind, sign) in [(LinkPropType::Hook, 1.), (LinkPropType::Gap, -1.)] {
        for load in [-20., 0., 5., 30.] {
            let load = load * sign;
            let mut m = axial(kind, 0.2, load, 50.);
            analyze_model(&mut m).unwrap();
            let r = &m.case_results["NL"];
            let closed = load * sign > 10.;
            let u = if closed {
                (load + sign * 100. * 0.2) / 150.
            } else {
                load / 50.
            };
            let force = if closed { 100. * (u - sign * 0.2) } else { 0. };
            close(r.joint_displacements["J"][0], u);
            close(r.link_forces["L"][0], force);
            close(r.joint_reactions["I"][0], -force);
            close(r.base_reactions[0], -load);
        }
    }
}

#[test]
fn initially_open_gap_without_parallel_stiffness_closes_without_artificial_force() {
    for (kind, sign) in [(LinkPropType::Hook, 1.), (LinkPropType::Gap, -1.)] {
        for opening in [0., 0.2, 100.] {
            for effective in [0., 13., 1e20] {
                let mut m = axial(kind, opening, sign * 0.1, 0.);
                m.link_props["P"].ke[0] = effective;
                analyze_model(&mut m).unwrap();
                let r = &m.case_results["NL"];
                close(r.joint_displacements["J"][0], sign * (opening + 0.001));
                close(r.link_forces["L"][0], sign * 0.1);
                close(r.base_reactions[0], -sign * 0.1);
            }
        }
    }
}

#[test]
fn nonlinear_dof_flags_distinguish_linear_behavior_and_zero_nonlinear_stiffness() {
    let mut m = axial(LinkPropType::Gap, 0.2, 20., 50.);
    m.link_props["P"].dof_nonlinear[0] = false;
    analyze_model(&mut m).unwrap();
    close(m.case_results["NL"].joint_displacements["J"][0], 20. / 63.);
    m.link_props["P"].dof_nonlinear[0] = true;
    m.link_props["P"].k_nl[0] = 0.;
    analyze_model(&mut m).unwrap();
    close(m.case_results["NL"].joint_displacements["J"][0], 0.4);
    close(m.case_results["NL"].link_forces["L"][0], 0.);
    // An unstressed linear case uses ke even when nonlinear properties are active.
    m.load_cases["NL"].case_type = LoadCaseType::LinearStatic;
    analyze_model(&mut m).unwrap();
    close(m.case_results["NL"].joint_displacements["J"][0], 20. / 63.);
}

#[test]
fn contact_activated_by_rotation_recovers_offset_moments_in_rotated_joint_axes() {
    let mut m = axial(LinkPropType::Gap, 0.1, 0., 0.);
    m.active_dofs = [true; 6];
    m.points["J"].restraint = [true; 6];
    m.points["J"].local_axes_deg = [33., 15., -10.];
    m.points["J"].loads_displ.push(PointLoadDispl {
        load_pat: "DEAD".into(),
        csys: "GLOBAL".into(),
        displ: [0., 0., -0.5, 0., 0.2, 0.],
    });
    let p = &mut m.link_props["P"];
    p.dof_active = [false, true, false, false, false, false];
    p.dof_nonlinear = [false, true, false, false, false, false];
    p.k_nl[1] = 100.;
    p.param1[1] = 0.1;
    p.dj2 = 0.7;
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["NL"];
    close(r.link_forces["L"][1], -26.);
    close(r.link_forces["L"][5], 18.2);
    for reaction in r.base_reactions {
        close(reaction, 0.);
    }
}

fn coupled_contacts() -> SapModel {
    let mut m = axial(LinkPropType::Hook, 0.2, 0., 10.);
    m.active_dofs = [true, true, false, false, false, false];
    m.points["J"].x = 1.;
    m.points["J"].spring[1] = 10.;
    m.add_point("I2", 0., -1., 0.);
    m.points["I2"].restraint = [true; 6];
    let mut p = m.link_props["P"].clone();
    p.name = "P2".into();
    p.param1[0] = 0.1;
    m.link_props.insert("P2".into(), p);
    m.links.insert(
        "L2".into(),
        LinkObj {
            name: "L2".into(),
            point_i: "I2".into(),
            point_j: Some("J".into()),
            prop: "P2".into(),
            angle_deg: 0.,
            selected: false,
        },
    );
    let diagonal_force = 100. * (0.2 / 2f64.sqrt() - 0.1) / 2f64.sqrt();
    m.points["J"].loads_force[0].force =
        [24. + diagonal_force, -2. + diagonal_force, 0., 0., 0., 0.];
    m
}

#[test]
fn coupled_contacts_solve_vector_equilibrium_and_fail_cleanly_on_iteration_limit() {
    let mut m = coupled_contacts();
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["NL"];
    close(r.joint_displacements["J"][0], 0.4);
    close(r.joint_displacements["J"][1], -0.2);
    close(r.link_forces["L"][0], 20.);
    close(r.link_forces["L2"][0], 100. * (0.2 / 2f64.sqrt() - 0.1));
    m.load_cases["NL"].max_iterations = 1;
    assert!(analyze_model(&mut m)
        .unwrap_err()
        .to_string()
        .contains("iteration limit"));
    assert!(!m.is_locked);
    assert!(m.case_results.is_empty());
}

#[test]
fn unsupported_contact_load_direction_and_invalid_parameters_fail_without_stale_results() {
    let mut m = axial(LinkPropType::Hook, 0.2, 20., 0.);
    analyze_model(&mut m).unwrap();
    m.points["J"].loads_force[0].force[0] = -20.;
    let error = analyze_model(&mut m).unwrap_err().to_string();
    assert!(error.contains("no reachable resisting contact"), "{error}");
    assert!(!m.is_locked);
    assert!(m.case_results.is_empty());
    for (k, opening) in [(-1., 0.), (1., -1.), (f64::NAN, 0.), (1., f64::INFINITY)] {
        let mut m = axial(LinkPropType::Hook, opening, 20., 10.);
        m.link_props["P"].k_nl[0] = k;
        assert!(analyze_model(&mut m)
            .unwrap_err()
            .to_string()
            .contains("finite and nonnegative"));
    }
    for (iterations, tolerance) in [(0, 1e-8), (2, 0.), (2, f64::NAN)] {
        let mut m = axial(LinkPropType::Hook, 0., 20., 10.);
        m.load_cases["NL"].max_iterations = iterations;
        m.load_cases["NL"].convergence_tol = tolerance;
        assert!(analyze_model(&mut m)
            .unwrap_err()
            .to_string()
            .contains("positive nonlinear iteration"));
    }
}

#[test]
fn grounded_gap_support_reaction_and_global_moment_balance_are_reported() {
    let mut m = axial(LinkPropType::Gap, 0.2, 0., 0.);
    m.active_dofs = [false, false, true, false, false, false];
    m.links["L"].point_i = "J".into();
    m.links["L"].point_j = None;
    m.points["J"].spring[2] = 50.;
    m.points["J"].loads_force[0].force[2] = -20.;
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["NL"];
    close(r.joint_displacements["J"][2], -40. / 150.);
    close(r.link_forces["L"][0], -20. / 3.);
    close(r.joint_reactions["J"][2], 20.);
    close(r.base_reactions[2], 20.);
    close(r.base_reactions[4], -60.);
}

#[test]
fn nonlinear_free_equilibrium_tolerance_ignores_unrelated_restrained_load_magnitude() {
    let mut m = axial(LinkPropType::Hook, 0.2, 30., 50.);
    m.points["I"].loads_force.push(PointLoadForce {
        load_pat: "DEAD".into(),
        csys: "GLOBAL".into(),
        force: [1e20, 0., 0., 0., 0., 0.],
    });
    analyze_model(&mut m).unwrap();
    close(m.case_results["NL"].joint_displacements["J"][0], 1. / 3.);
    for scale in [1e-9, 1e9] {
        let mut m = axial(LinkPropType::Hook, 0.2, 30. * scale, 50. * scale);
        m.link_props["P"].k_nl[0] = 100. * scale;
        analyze_model(&mut m).unwrap();
        close(m.case_results["NL"].joint_displacements["J"][0], 1. / 3.);
    }
}

#[test]
fn pdelta_honors_iteration_limit_and_returns_error_instead_of_last_unconverged_result() {
    let mut m = axial(LinkPropType::Hook, 0., 10., 0.);
    m.links.clear();
    m.add_frame("F", "I", "J", "FSEC1");
    m.active_dofs = [true; 6];
    m.points["J"].loads_force[0].force[1] = 1.;
    m.load_cases["NL"].nl_geom = NLGeomType::PDelta;
    m.load_cases["NL"].max_iterations = 1;
    let error = analyze_model(&mut m).unwrap_err().to_string();
    assert!(
        error.contains("P-Delta axial-force iteration limit"),
        "{error}"
    );
    assert!(!m.is_locked);
    assert!(m.case_results.is_empty());
    m.load_cases["NL"].max_iterations = 2;
    m.load_cases["NL"].convergence_tol = 1e-12;
    analyze_model(&mut m).unwrap();
    close(m.case_results["NL"].frame_forces["F"][0].p, 10.);
}

#[test]
fn geometric_contact_and_unstored_nonlinear_tangent_continuation_remain_explicit_errors() {
    let mut m = axial(LinkPropType::Hook, 0., 20., 10.);
    m.load_cases["NL"].nl_geom = NLGeomType::PDelta;
    assert!(analyze_model(&mut m)
        .unwrap_err()
        .to_string()
        .contains("combines geometric"));
    m.load_cases["NL"].nl_geom = NLGeomType::None;
    let mut follow = LoadCase::linear_static("FOLLOW", "DEAD", 1.);
    follow.initial_case = Some("NL".into());
    m.load_cases.insert("FOLLOW".into(), follow);
    assert!(analyze_model(&mut m)
        .unwrap_err()
        .to_string()
        .contains("nonlinear link tangent"));
}

#[test]
fn incomplete_imported_contact_data_cannot_silently_become_linear_or_zero_opening() {
    for flags in [false, true] {
        let mut m = axial(LinkPropType::Gap, 0.2, -20., 10.);
        m.link_props["P"].nonlinear_data_complete = false;
        m.link_props["P"].dof_nonlinear[0] = flags;
        assert!(analyze_model(&mut m)
            .unwrap_err()
            .to_string()
            .contains("incomplete imported nonlinear"));
        assert!(!m.is_locked);
        assert!(m.case_results.is_empty());
        m.load_cases["NL"].case_type = LoadCaseType::LinearStatic;
        analyze_model(&mut m).unwrap();
        close(m.case_results["NL"].joint_displacements["J"][0], -20. / 23.);
    }
}

#[test]
fn loaded_cantilever_closes_ground_gap_and_matches_beam_compatibility_at_every_station() {
    let mut m = axial(LinkPropType::Gap, 0.1, 0., 0.);
    m.active_dofs = [true; 6];
    m.points["J"].x = 10.;
    m.add_frame("BEAM", "I", "J", "FSEC1");
    let section = &mut m.frame_sections["FSEC1"];
    section.i33 = 1.;
    section.i22 = 1.;
    section.as2 = 0.;
    section.as3 = 0.;
    m.materials[&section.material].e = 2000.;
    m.frames["BEAM"].loads_dist.push(FrameLoadDistributed {
        load_pat: "DEAD".into(),
        csys: "GLOBAL".into(),
        my_type: 1,
        dir: 6,
        rel_dist1: 0.,
        rel_dist2: 1.,
        val1: -1.,
        val2: -1.,
    });
    m.links["L"].point_i = "J".into();
    m.links["L"].point_j = None;
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["NL"];
    let deflection = (6. * 0.625 + 100. * 0.1) / 106.;
    let contact = 100. * (deflection - 0.1);
    close(r.joint_displacements["J"][2], -deflection);
    close(r.link_forces["L"][0], -contact);
    close(r.base_reactions[2], 10.);
    close(r.base_reactions[4], -50.);
    for station in &r.frame_forces["BEAM"] {
        let x = station.station;
        let exact = -x * x * (600. - 40. * x + x * x) / (24. * 2000.)
            + contact * x * x * (30. - x) / (6. * 2000.);
        close(station.local_disp[1], exact);
    }
}

#[test]
fn prescribed_unrestrained_contact_dof_reports_actuator_reaction() {
    let mut m = axial(LinkPropType::Hook, 0.2, 0., 0.);
    m.points["J"].loads_displ.push(PointLoadDispl {
        load_pat: "DEAD".into(),
        csys: "GLOBAL".into(),
        displ: [0.3, 0., 0., 0., 0., 0.],
    });
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["NL"];
    close(r.link_forces["L"][0], 10.);
    close(r.joint_reactions["I"][0], -10.);
    close(r.joint_reactions["J"][0], 10.);
    close(r.base_reactions[0], 0.);
}

#[test]
fn nonfinite_contact_loads_or_prescribed_displacements_do_not_produce_false_convergence() {
    for force in [f64::NAN, f64::INFINITY] {
        let mut m = axial(LinkPropType::Hook, 0., force, 10.);
        assert!(analyze_model(&mut m)
            .unwrap_err()
            .to_string()
            .contains("nonfinite"));
        assert!(!m.is_locked);
        assert!(m.case_results.is_empty());
    }
    let mut m = axial(LinkPropType::Hook, 0., 0., 10.);
    m.points["J"].loads_displ.push(PointLoadDispl {
        load_pat: "DEAD".into(),
        csys: "GLOBAL".into(),
        displ: [f64::NAN, 0., 0., 0., 0., 0.],
    });
    assert!(analyze_model(&mut m)
        .unwrap_err()
        .to_string()
        .contains("nonfinite"));
}
