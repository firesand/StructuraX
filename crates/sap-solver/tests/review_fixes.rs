use sap_model::*;
use sap_solver::{analyze_model, SolverError};

fn close(a: f64, b: f64) {
    assert!((a - b).abs() <= 1e-8 * (1. + b.abs()), "{a} != {b}");
}
fn base() -> SapModel {
    let mut m = SapModel::default();
    m.load_patterns["DEAD"].self_wt_multiplier = 0.;
    m.load_cases.clear();
    m.active_dofs = [true, false, false, false, false, false];
    m.add_point("I", 0., 0., 0.);
    m.add_point("J", 10., 0., 0.);
    m.points["I"].restraint = [true; 6];
    m
}
fn force(m: &mut SapModel, f: [f64; 6]) {
    m.points["J"].loads_force.push(PointLoadForce {
        load_pat: "DEAD".into(),
        csys: "GLOBAL".into(),
        force: f,
    });
}
fn history(m: &mut SapModel, name: &str, steps: usize) {
    m.functions.insert(
        "RAMP".into(),
        DynamicFunction {
            name: "RAMP".into(),
            is_time_history: true,
            damping_ratio: 0.,
            points: vec![(0., 0.), (1., 1.)],
        },
    );
    let mut c = LoadCase::linear_static(name, "DEAD", 1.);
    c.loads.clear();
    c.case_type = LoadCaseType::NonlinearHistory;
    c.history_loads = vec![("RAMP".into(), "DEAD".into(), 1.)];
    c.time_step_dt = 0.01;
    c.num_time_steps = steps;
    c.convergence_tol = 1e-10;
    m.load_cases.insert(name.into(), c);
}
fn link(m: &mut SapModel, kind: LinkPropType, ke: f64, grounded: bool) {
    let mut prop = LinkProp {
        name: "P".into(),
        link_type: kind,
        ..LinkProp::default()
    };
    prop.dof_nonlinear[0] = true;
    prop.ke[0] = ke;
    prop.k_nl[0] = 100.;
    m.link_props.insert("P".into(), prop);
    m.links.insert(
        "L".into(),
        LinkObj {
            name: "L".into(),
            point_i: if grounded { "J" } else { "I" }.into(),
            point_j: (!grounded).then(|| "J".into()),
            prop: "P".into(),
            angle_deg: 0.,
            selected: false,
        },
    );
}
fn bar(m: &mut SapModel) {
    let mut mat = Material::default_steel_kip_in();
    mat.name = "M".into();
    mat.e = 1000.;
    mat.unit_mass = 0.;
    mat.unit_weight = 0.;
    m.materials.insert("M".into(), mat);
    m.frame_sections.insert(
        "S".into(),
        FrameSection {
            name: "S".into(),
            material: "M".into(),
            area: 1.,
            ..FrameSection::default()
        },
    );
    m.add_frame("B", "I", "J", "S");
    let mut c = LoadCase::linear_static("NL", "DEAD", 1.);
    c.case_type = LoadCaseType::NonlinearStatic;
    m.load_cases.insert("NL".into(), c);
}
fn cable() -> SapModel {
    let mut m = base();
    bar(&mut m);
    m.points["J"].restraint = [true; 6];
    m.frames["B"].cable_params = Some(CableParams {
        undeformed_length: Some(12.),
        target_tension_i: None,
        horizontal_tension: None,
        num_segments: 1,
    });
    m.frames["B"].loads_dist.push(FrameLoadDistributed {
        load_pat: "DEAD".into(),
        my_type: 1,
        dir: 6,
        csys: "GLOBAL".into(),
        rel_dist1: 0.,
        rel_dist2: 1.,
        val1: -1.,
        val2: -1.,
    });
    m
}
fn rejected(m: &mut SapModel, phrase: &str) {
    let error = analyze_model(m).unwrap_err();
    assert!(matches!(error, SolverError::UnsupportedInput(_)), "{error}");
    assert!(error.to_string().contains(phrase), "{error}");
    assert!(!m.is_locked);
    assert!(m.case_results.is_empty());
}

#[test]
fn nonlinear_history_support_force_matches_constitutive_force_independent_of_ke() {
    let mut reference = None;
    for ke in [0., 13., 1e12] {
        let mut m = base();
        force(&mut m, [10., 0., 0., 0., 0., 0.]);
        m.points["J"].mass[0] = 1.;
        link(&mut m, LinkPropType::Hook, ke, false);
        m.link_props["P"].ce[0] = 0.7;
        history(&mut m, "H", 50);
        m.load_cases["H"].rayleigh_damping = (0., 0.03);
        analyze_model(&mut m).unwrap();
        let r = &m.case_results["H"];
        close(r.joint_reactions["I"][0], -r.link_forces["L"][0]);
        let u = r.joint_displacements["J"][0];
        if let Some(expected) = reference {
            close(u, expected);
        } else {
            reference = Some(u);
        }
        // For a ramp and c=0.7, exact response follows the independent linear
        // oscillator while the Hook stays in tension.
        assert!(u > 0.05 && u < 0.06);
    }
}

#[test]
fn grounded_maxwell_history_reactions_include_internal_state() {
    let mut m = base();
    m.active_dofs = [false, false, true, false, false, false];
    m.points["J"].spring[2] = 50.;
    m.points["J"].mass[2] = 1.;
    force(&mut m, [0., 0., 5., 0., 0., 0.]);
    link(&mut m, LinkPropType::Damper, 1e6, true);
    m.link_props["P"].param1[0] = 10.;
    m.link_props["P"].param2[0] = 1.;
    history(&mut m, "H", 50);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["H"];
    let f = r.link_forces["L"][0];
    assert!(f > 0.1);
    close(
        r.joint_reactions["J"][2],
        -50. * r.joint_displacements["J"][2] - f,
    );
    close(r.base_reactions[2], r.joint_reactions["J"][2]);
}

#[test]
fn history_origin_continuation_fails_without_stale_results() {
    let mut m = base();
    force(&mut m, [1., 0., 0., 0., 0., 0.]);
    m.points["J"].mass[0] = 1.;
    m.points["J"].spring[0] = 100.;
    history(&mut m, "H1", 10);
    analyze_model(&mut m).unwrap();
    history(&mut m, "H2", 10);
    m.load_cases["H2"].history_loads.clear();
    m.load_cases["H2"].initial_case = Some("H1".into());
    rejected(&mut m, "history continuation");
    m.load_cases["H2"].case_type = LoadCaseType::NonlinearStatic;
    rejected(&mut m, "history continuation");
}

#[test]
fn disconnected_cable_cannot_discard_the_ordinary_structure() {
    let mut m = base();
    bar(&mut m);
    force(&mut m, [10., 0., 0., 0., 0., 0.]);
    m.points["J"].spring[0] = 100.;
    analyze_model(&mut m).unwrap();
    close(m.case_results["NL"].joint_displacements["J"][0], 0.05);
    m.add_point("C", 0., 0., 10.);
    m.add_point("D", 10., 0., 10.);
    m.points["C"].restraint = [true; 6];
    m.points["D"].restraint = [true; 6];
    m.add_frame("CABLE", "C", "D", "S");
    m.frames["CABLE"].cable_params = Some(CableParams {
        undeformed_length: Some(10.),
        target_tension_i: None,
        horizontal_tension: None,
        num_segments: 1,
    });
    rejected(&mut m, "cannot mix");
}

#[test]
fn cable_supported_uniform_loading_and_unsupported_assignments_are_distinct() {
    let original = cable();
    let mut m = original.clone();
    analyze_model(&mut m).unwrap();
    close(m.case_results["NL"].base_reactions[2], 12.);
    m = original.clone();
    m.frames["B"].loads_dist[0].rel_dist2 = 0.5;
    rejected(&mut m, "full-span uniform");
    m = original.clone();
    m.frames["B"].loads_dist[0].val2 = -2.;
    rejected(&mut m, "full-span uniform");
    m = original.clone();
    m.frames["B"].loads_point.push(FrameLoadPoint {
        load_pat: "DEAD".into(),
        my_type: 1,
        dir: 6,
        csys: "GLOBAL".into(),
        rel_dist: 0.5,
        val: -1.,
    });
    rejected(&mut m, "full-span uniform");
    m = original.clone();
    m.frames["B"]
        .cable_params
        .as_mut()
        .unwrap()
        .target_tension_i = Some(5.);
    rejected(&mut m, "target end tension");
    m = original.clone();
    m.points["J"].spring[0] = 100.;
    rejected(&mut m, "joint springs");
    m = original.clone();
    m.points["J"].local_axes_deg[0] = 30.;
    rejected(&mut m, "rotated joint axes");
    m = original;
    m.load_cases["NL"].case_type = LoadCaseType::LinearStatic;
    rejected(&mut m, "cable-only nonlinear static");
}

#[test]
fn member_limits_accept_nodal_axial_loading_but_reject_unrepresented_span_loads() {
    let mut m = base();
    bar(&mut m);
    m.frames["B"].tc_limits = Some((true, 0., false, 0.));
    m.points["J"].spring[0] = 50.;
    force(&mut m, [10., 0., 0., 0., 0., 0.]);
    analyze_model(&mut m).unwrap();
    close(m.case_results["NL"].joint_displacements["J"][0], 10. / 150.);
    m.points["J"].loads_force[0].force[0] = -10.;
    analyze_model(&mut m).unwrap();
    close(m.case_results["NL"].joint_displacements["J"][0], -0.2);
    close(m.case_results["NL"].frame_forces["B"][0].p, 0.);
    m.frames["B"].loads_dist.push(FrameLoadDistributed {
        load_pat: "DEAD".into(),
        my_type: 1,
        dir: 1,
        csys: "LOCAL".into(),
        rel_dist1: 0.,
        rel_dist2: 1.,
        val1: 1.,
        val2: 1.,
    });
    rejected(&mut m, "axial span loads");
    m.frames["B"].loads_dist.clear();
    m.frames["B"].release_i[0] = true;
    rejected(&mut m, "releases");
}

#[test]
fn solid_zero_mass_and_weight_are_exact_and_invalid_modifiers_reject() {
    use sap_solver::solid::{compute_solid_nodal_loads, compute_solid_stiffness_and_mass};
    let coords = [
        [0., 0., 0.],
        [1., 0., 0.],
        [0., 1., 0.],
        [1., 1., 0.],
        [0., 0., 1.],
        [1., 0., 1.],
        [0., 1., 1.],
        [1., 1., 1.],
    ];
    let mat = Material::default_steel_kip_in();
    let mut sec = SolidSection::default();
    sec.modifiers[6] = 0.;
    sec.modifiers[7] = 0.;
    let (_, mass) = compute_solid_stiffness_and_mass(&coords, &mat, &sec).unwrap();
    assert!(mass.iter().all(|v| *v == 0.));
    let solid = SolidObj {
        name: "S".into(),
        points: std::array::from_fn(|i| i.to_string()),
        section: "S".into(),
        loads_surface: vec![],
        loads_temp: Default::default(),
        selected: false,
    };
    let f =
        compute_solid_nodal_loads(&coords, &mat, &sec, &solid, "DEAD", 1., [0., 0., -1.]).unwrap();
    assert!(f.iter().all(|v| *v == 0.));
    for (i, value) in [(6, -1.), (7, f64::NAN), (0, 0.), (3, f64::INFINITY)] {
        let mut invalid = sec.clone();
        invalid.modifiers[i] = value;
        assert!(compute_solid_stiffness_and_mass(&coords, &mat, &invalid).is_err());
        assert!(compute_solid_nodal_loads(
            &coords,
            &mat,
            &invalid,
            &solid,
            "DEAD",
            1.,
            [0., 0., -1.]
        )
        .is_err());
    }
}

#[test]
fn nonlinear_history_iteration_budget_is_honored() {
    let mut m = base();
    m.active_dofs = [true, true, false, false, false, false];
    m.points["J"].mass = [1., 1., 0., 0., 0., 0.];
    force(&mut m, [20., 30., 0., 0., 0., 0.]);
    link(&mut m, LinkPropType::Hook, 0., false);
    m.link_props["P"].param1[0] = 0.01;
    m.add_point("K", 0., -10., 0.);
    m.points["K"].restraint = [true; 6];
    let mut q = m.link_props["P"].clone();
    q.name = "Q".into();
    q.k_nl[0] = 300.;
    q.param1[0] = 0.03;
    m.link_props.insert("Q".into(), q);
    m.links.insert(
        "L2".into(),
        LinkObj {
            name: "L2".into(),
            point_i: "K".into(),
            point_j: Some("J".into()),
            prop: "Q".into(),
            angle_deg: 0.,
            selected: false,
        },
    );
    history(&mut m, "H", 5);
    m.load_cases["H"].time_step_dt = 0.1;
    m.load_cases["H"].max_iterations = 1;
    let error = analyze_model(&mut m).unwrap_err();
    assert!(
        matches!(error, SolverError::NonConvergence { iterations: 1, .. }),
        "{error}"
    );
    assert!(m.case_results.is_empty());
    m.load_cases["H"].max_iterations = 100;
    analyze_model(&mut m).unwrap();
}

#[test]
fn cable_iteration_limit_and_relative_equilibrium_are_enforced() {
    use sap_solver::cable::{solve_cable_network, CableElement};
    let element = CableElement::from_undeformed_length(
        [0.; 3],
        [10., 0., 0.],
        1.,
        1000.,
        0.,
        12.,
        [0., 0., -1.],
        0.,
    )
    .unwrap();
    let elements = [(0, 1, element)];
    let fixed = [[true; 3], [true, true, false]];
    let load = [[0.; 3], [0., 0., -10.]];
    let error = solve_cable_network(2, &elements, &fixed, &load, None, 1, 1e-10).unwrap_err();
    assert!(
        matches!(error, SolverError::NonConvergence { iterations: 1, .. }),
        "{error}"
    );
    let (_, states) = solve_cable_network(2, &elements, &fixed, &load, None, 100, 1e-10).unwrap();
    close(states[0].f_j[2], -10.);
    close(states[0].f_i[2] + states[0].f_j[2], 12.);
}

#[test]
fn unsupported_maxwell_wilson_state_update_is_explicit() {
    let mut m = base();
    force(&mut m, [1., 0., 0., 0., 0., 0.]);
    m.points["J"].mass[0] = 1.;
    link(&mut m, LinkPropType::Damper, 0., false);
    m.link_props["P"].param1[0] = 10.;
    m.link_props["P"].param2[0] = 1.;
    history(&mut m, "H", 10);
    m.load_cases["H"].time_integration = TimeIntegrationMethod::WilsonTheta;
    rejected(&mut m, "Wilson-theta Maxwell");
}

#[test]
fn nonlinear_shear_link_section_moment_uses_single_offset_conversion() {
    let mut m = base();
    m.points["J"].mass[0] = 1.;
    m.points["J"].restraint[5] = true;
    force(&mut m, [10., 0., 0., 0., 0., 0.]);
    link(&mut m, LinkPropType::Hook, 13., true);
    let p = &mut m.link_props["P"];
    p.dof_active = [false, true, false, false, false, false];
    p.dof_nonlinear = [false, true, false, false, false, false];
    p.ke[1] = 13.;
    p.k_nl[1] = 100.;
    p.dj2 = 2.;
    history(&mut m, "H", 50);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["H"];
    let shear = r.link_forces["L"][1];
    assert!(shear > 1.);
    close(shear, 100. * r.joint_displacements["J"][0]);
    close(r.link_forces["L"][5], -2. * shear);
    close(r.joint_reactions["J"][0], -shear);
}

#[test]
fn static_to_history_initial_acceleration_includes_new_step_force() {
    let mut m = base();
    force(&mut m, [1., 0., 0., 0., 0., 0.]);
    m.points["J"].mass[0] = 1.;
    m.points["J"].spring[0] = 100.;
    let mut preload = LoadCase::linear_static("PRE", "DEAD", 1.);
    preload.case_type = LoadCaseType::NonlinearStatic;
    m.load_cases.insert("PRE".into(), preload);
    history(&mut m, "H", 10);
    m.functions["RAMP"].points = vec![(0., 1.), (1., 1.)];
    m.load_cases["H"].initial_case = Some("PRE".into());
    analyze_model(&mut m).unwrap();
    let trace = &m.case_results["H"].time_history_disp["J"];
    close(trace[0][0], 0.01);
    let exact = 0.02 - 0.01 * 1.0_f64.cos();
    assert!((trace.last().unwrap()[0] - exact).abs() < 1e-5);
}

#[test]
fn cable_prescribed_free_joint_includes_actuator_reaction() {
    let mut m = cable();
    m.frames["B"].loads_dist.clear();
    m.frames["B"]
        .cable_params
        .as_mut()
        .unwrap()
        .undeformed_length = Some(10.);
    m.points["J"].restraint = [false; 6];
    m.points["J"].loads_displ.push(PointLoadDispl {
        load_pat: "DEAD".into(),
        csys: "GLOBAL".into(),
        displ: [0.1, 0., 0., 0., 0., 0.],
    });
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["NL"];
    close(r.joint_displacements["J"][0], 0.1);
    close(r.joint_reactions["J"][0], 10.);
    close(r.joint_reactions["I"][0], -10.);
    close(r.base_reactions[0], 0.);
}
