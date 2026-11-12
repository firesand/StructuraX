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
    // Modal history types are integrated directly; they take no modal damping.
    c.modal_damping = 0.;
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
    m.load_cases["H1"].case_type = LoadCaseType::LinearHistory;
    // Modal history types are integrated directly; they take no modal damping.
    m.load_cases["H1"].modal_damping = 0.;
    analyze_model(&mut m).unwrap();
    history(&mut m, "H2", 10);
    m.load_cases["H2"].history_loads.clear();
    m.load_cases["H2"].initial_case = Some("H1".into());
    rejected(&mut m, "modal history continuation");
    m.load_cases["H2"].case_type = LoadCaseType::NonlinearStatic;
    rejected(&mut m, "modal history continuation");

    // Direct-integration into modal LinearHistory must also remain rejected.
    m.load_cases["H1"].case_type = LoadCaseType::LinearDynamic;
    m.load_cases["H2"].case_type = LoadCaseType::LinearHistory;
    // Modal history types are integrated directly; they take no modal damping.
    m.load_cases["H2"].modal_damping = 0.;
    rejected(&mut m, "modal history");
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

#[test]
fn nonlinear_case_relative_tolerance_must_be_between_zero_and_one() {
    for case_type in [
        LoadCaseType::NonlinearStatic,
        LoadCaseType::NonlinearHistory,
    ] {
        for tolerance in [0., 1., 2., f64::NAN, f64::INFINITY] {
            let mut m = base();
            m.points["J"].spring[0] = 100.;
            m.points["J"].mass[0] = 1.;
            force(&mut m, [1., 0., 0., 0., 0., 0.]);
            history(&mut m, "H", 1);
            m.load_cases["H"].case_type = case_type;
            m.load_cases["H"].convergence_tol = tolerance;
            let error = analyze_model(&mut m).unwrap_err();
            assert!(matches!(error, SolverError::InvalidInput(_)), "{error}");
            assert!(
                error.to_string().contains("relative tolerance in (0,1)"),
                "{error}"
            );
            assert!(m.case_results.is_empty());
        }
    }
}

#[test]
fn linear_dynamic_split_continuation_matches_uninterrupted_and_analytical_sdof() {
    // Underdamped SDOF: m = 2, k = 50 (wn = 5 rad/s), Rayleigh c = 0.4*2 + 0.008*50 = 1.2 (zeta = 0.06)
    let mass = 2.0_f64;
    let k = 50.0_f64;
    let alpha_m = 0.4_f64;
    let beta_k = 0.008_f64;
    let c_damp = alpha_m * mass + beta_k * k;
    let f0 = 10.0_f64;
    let dt = 0.002_f64;
    let n1 = 500_usize;
    let n2 = 500_usize;

    let exact_u = |t: f64| -> f64 {
        let decay = c_damp / (2.0 * mass);
        let wd = (k / mass - decay * decay).sqrt();
        (f0 / k) * (1.0 - (-decay * t).exp() * ((wd * t).cos() + (decay / wd) * (wd * t).sin()))
    };

    for (method, hht_alpha) in [
        (TimeIntegrationMethod::NewmarkAverage, 0.0),
        (TimeIntegrationMethod::HilberHughesTaylor, -0.1),
    ] {
        let mut m = base();
        m.points["J"].mass[0] = mass;
        m.points["J"].spring[0] = k;
        force(&mut m, [f0, 0., 0., 0., 0., 0.]);
        m.functions.insert(
            "STEP".into(),
            DynamicFunction {
                name: "STEP".into(),
                is_time_history: true,
                damping_ratio: 0.,
                points: vec![(0., 1.), (10., 1.)],
            },
        );

        let mut full = LoadCase::linear_static("FULL", "DEAD", 1.);
        full.loads.clear();
        full.case_type = LoadCaseType::LinearDynamic;
        full.time_integration = method;
        full.hht_alpha = hht_alpha;
        full.rayleigh_damping = (alpha_m, beta_k);
        full.time_step_dt = dt;
        full.num_time_steps = n1 + n2;
        full.history_loads = vec![("STEP".into(), "DEAD".into(), 1.)];

        let mut hist1 = full.clone();
        hist1.name = "HIST1".into();
        hist1.num_time_steps = n1;

        let mut hist2 = full.clone();
        hist2.name = "HIST2".into();
        hist2.num_time_steps = n2;
        hist2.initial_case = Some("HIST1".into());
        // Sustained step load from HIST1 is carried automatically via f_base_init.
        hist2.history_loads.clear();

        m.load_cases.insert("FULL".into(), full);
        m.load_cases.insert("HIST1".into(), hist1);
        m.load_cases.insert("HIST2".into(), hist2);
        analyze_model(&mut m).unwrap();

        let tr_full = &m.case_results["FULL"].time_history_disp["J"];
        let tr_h2 = &m.case_results["HIST2"].time_history_disp["J"];
        assert_eq!(tr_h2.len(), n2 + 1);
        for s in 0..=n2 {
            let diff = (tr_h2[s][0] - tr_full[n1 + s][0]).abs();
            assert!(
                diff < 1e-13,
                "method={method:?} step {s}: split={} vs full={} (diff={diff:e})",
                tr_h2[s][0],
                tr_full[n1 + s][0]
            );
        }

        let ts_full = m.case_results["FULL"].transient_state.as_ref().unwrap();
        let ts_h2 = m.case_results["HIST2"].transient_state.as_ref().unwrap();
        for i in 0..ts_full.u.len() {
            assert!((ts_h2.u[i] - ts_full.u[i]).abs() < 1e-13);
            assert!((ts_h2.v[i] - ts_full.v[i]).abs() < 1e-12);
            assert!((ts_h2.a[i] - ts_full.a[i]).abs() < 1e-12);
        }

        let t_end = ((n1 + n2) as f64) * dt;
        let u_ana = exact_u(t_end);
        assert!(
            (tr_h2[n2][0] - u_ana).abs() < 5e-5,
            "method={method:?} terminal u={} vs analytical={u_ana}",
            tr_h2[n2][0]
        );
    }
}

#[test]
fn nonlinear_gap_hook_maxwell_split_history_and_terminal_static_continuation() {
    let mut m = base();
    m.points["J"].mass[0] = 1.0;
    m.points["J"].spring[0] = 25.0;
    m.add_point("G_END", 0., 0., 0.);
    m.points["G_END"].restraint = [true; 6];
    m.add_point("H_END", 0., 0., 0.);
    m.points["H_END"].restraint = [true; 6];
    m.add_point("D_END", 0., 0., 0.);
    m.points["D_END"].restraint = [true; 6];

    let mut gap_prop = LinkProp {
        name: "GAP_P".into(),
        link_type: LinkPropType::Gap,
        ..LinkProp::default()
    };
    gap_prop.dof_nonlinear[0] = true;
    gap_prop.k_nl[0] = 150.0;
    gap_prop.param1[0] = 0.03;
    m.link_props.insert("GAP_P".into(), gap_prop);

    let mut hook_prop = LinkProp {
        name: "HOOK_P".into(),
        link_type: LinkPropType::Hook,
        ..LinkProp::default()
    };
    hook_prop.dof_nonlinear[0] = true;
    hook_prop.k_nl[0] = 120.0;
    hook_prop.param1[0] = 0.02;
    m.link_props.insert("HOOK_P".into(), hook_prop);

    let mut damp_prop = LinkProp {
        name: "DAMP_P".into(),
        link_type: LinkPropType::Damper,
        ..LinkProp::default()
    };
    damp_prop.dof_nonlinear[0] = true;
    damp_prop.k_nl[0] = 200.0;
    damp_prop.param1[0] = 18.0;
    damp_prop.param2[0] = 1.0;
    m.link_props.insert("DAMP_P".into(), damp_prop);

    for (lname, p_i, prop) in [
        ("LGAP", "G_END", "GAP_P"),
        ("LHOOK", "H_END", "HOOK_P"),
        ("LDAMP", "D_END", "DAMP_P"),
    ] {
        m.links.insert(
            lname.into(),
            LinkObj {
                name: lname.into(),
                point_i: p_i.into(),
                point_j: Some("J".into()),
                prop: prop.into(),
                angle_deg: 0.,
                selected: false,
            },
        );
    }

    force(&mut m, [1.0, 0., 0., 0., 0., 0.]);
    // Full load profile over [0, 6.0s]: big positive pulse (+35), negative reversal (-12),
    // then constant -5.0 from t=1.5s..6.0s so peak |u| >> |u(t_end)| and sign differs!
    // Split at t_split = 0.75s (right in the middle of active Maxwell damper force & reversal):
    // At t = 0.75s, linear interpolation between (0.5, 35.0) and (1.0, -12.0) is 11.5.
    let full_pts = vec![
        (0.0, 0.0),
        (0.25, 35.0),
        (0.50, 35.0),
        (0.75, 11.5),
        (1.00, -12.0),
        (1.50, -5.0),
        (6.00, -5.0),
    ];
    let f_split = 11.5_f64;
    let inc_pts = vec![
        (0.0, 0.0),
        (0.25, -12.0 - f_split),
        (0.75, -5.0 - f_split),
        (5.25, -5.0 - f_split),
    ];
    m.functions.insert(
        "F_FULL".into(),
        DynamicFunction {
            name: "F_FULL".into(),
            is_time_history: true,
            damping_ratio: 0.,
            points: full_pts,
        },
    );
    m.functions.insert(
        "F_INC".into(),
        DynamicFunction {
            name: "F_INC".into(),
            is_time_history: true,
            damping_ratio: 0.,
            points: inc_pts,
        },
    );

    let dt = 0.005_f64;
    let n1 = 150_usize; // 0.75 s
    let n2 = 1050_usize; // 5.25 s (total 6.00 s)

    let mut h_full = LoadCase::linear_static("H_FULL", "DEAD", 1.);
    h_full.loads.clear();
    h_full.case_type = LoadCaseType::NonlinearHistory;
    // Modal history types are integrated directly; they take no modal damping.
    h_full.modal_damping = 0.;
    h_full.time_integration = TimeIntegrationMethod::HilberHughesTaylor;
    h_full.hht_alpha = -0.1;
    h_full.rayleigh_damping = (2.5, 0.02);
    h_full.time_step_dt = dt;
    h_full.num_time_steps = n1 + n2;
    h_full.max_iterations = 80;
    h_full.convergence_tol = 1e-12;
    h_full.history_loads = vec![("F_FULL".into(), "DEAD".into(), 1.0)];

    let mut h1 = h_full.clone();
    h1.name = "H1".into();
    h1.num_time_steps = n1;

    let mut h2 = h_full.clone();
    h2.name = "H2".into();
    h2.num_time_steps = n2;
    h2.initial_case = Some("H1".into());
    h2.history_loads = vec![("F_INC".into(), "DEAD".into(), 1.0)];

    // NLSTAT continues from H2 with ZERO incremental load and max_iterations = 1:
    // because H2 has damped to terminal static equilibrium u(t_end) < 0 (Gap closed),
    // while H2's envelope joint_displacements["J"] holds an earlier positive/larger peak,
    // NLSTAT can converge in <= 1 iteration ONLY if seeded from terminal ts.u!
    let mut nlstat = LoadCase::linear_static("NLSTAT", "DEAD", 0.0);
    nlstat.loads.clear();
    nlstat.case_type = LoadCaseType::NonlinearStatic;
    nlstat.initial_case = Some("H2".into());
    nlstat.max_iterations = 1;
    nlstat.convergence_tol = 1e-9;

    m.load_cases.insert("H_FULL".into(), h_full);
    m.load_cases.insert("H1".into(), h1);
    m.load_cases.insert("H2".into(), h2);
    m.load_cases.insert("NLSTAT".into(), nlstat);
    analyze_model(&mut m).unwrap();

    // Verify at split point t=0.75s that Maxwell damper carries substantial nonzero force.
    let ts_h1 = m.case_results["H1"].transient_state.as_ref().unwrap();
    let f_damp_split = ts_h1
        .nonlinear_link_forces
        .iter()
        .find(|s| s.link == "LDAMP")
        .unwrap()
        .force;
    assert!(
        f_damp_split.abs() > 1.0,
        "Maxwell damper must be actively loaded at split, got {f_damp_split}"
    );

    let tr_full = &m.case_results["H_FULL"].time_history_disp["J"];
    let tr_h2 = &m.case_results["H2"].time_history_disp["J"];
    for s in 0..=n2 {
        let diff = (tr_h2[s][0] - tr_full[n1 + s][0]).abs();
        assert!(
            diff < 1e-12,
            "step {s}: H2={} vs H_FULL={} (diff={diff:e})",
            tr_h2[s][0],
            tr_full[n1 + s][0]
        );
    }

    // Verify H2 peak displacement differs substantially from terminal displacement,
    // and NLSTAT converged to terminal u(t_end) (-5 = 25*u + 150*(u + 0.03) => u = -0.5/175? Wait:
    // Gap compression at J: d = u_J - u_G = u_J < -0.03 => f_gap = 150*(u_J + 0.03),
    // 25*u_J + 150*(u_J + 0.03) = -5 => 175*u_J = -9.5 + ... wait: check exact static equilibrium!)
    let u_h2_peak = m.case_results["H2"].joint_displacements["J"][0];
    let u_h2_end = tr_h2.last().unwrap()[0];
    assert!(
        (u_h2_peak - u_h2_end).abs() > 0.05,
        "peak {u_h2_peak} and terminal {u_h2_end} must be distinct"
    );
    let u_nlstat = m.case_results["NLSTAT"].joint_displacements["J"][0];
    // Static equilibrium under sustained F = -5.0:
    // Gap opening = 0.03: at u = -0.0542857, d = -0.0542857 < -0.03,
    // 25*u + 150*(u + 0.03) = 175*u + 4.5 = -5.0 => u = -9.5 / 175.0
    let u_static_exact = -9.5 / 175.0;
    assert!((u_h2_end - u_static_exact).abs() < 1e-6);
    assert!((u_nlstat - u_static_exact).abs() < 1e-10);
}

#[test]
fn topological_nested_load_combinations_and_skipped_first_envelope_case() {
    let mut m = base();
    bar(&mut m);
    m.load_cases.clear();

    m.load_patterns.insert(
        "LIVE".into(),
        LoadPattern {
            name: "LIVE".into(),
            pat_type: LoadPatternType::Live,
            self_wt_multiplier: 0.,
        },
    );
    force(&mut m, [20., 0., 0., 0., 0., 0.]); // DEAD -> u = 20 / 100 = 0.2
    m.points["J"].loads_force.push(PointLoadForce {
        load_pat: "LIVE".into(),
        csys: "GLOBAL".into(),
        force: [-50., 0., 0., 0., 0., 0.], // LIVE -> u = -0.5
    });

    let mut unrun = LoadCase::linear_static("UNRUN", "DEAD", 999.0);
    unrun.run_flag = false;
    m.load_cases.insert("UNRUN".into(), unrun);
    m.load_cases
        .insert("CASE_A".into(), LoadCase::linear_static("CASE_A", "DEAD", 1.0));
    m.load_cases
        .insert("CASE_B".into(), LoadCase::linear_static("CASE_B", "LIVE", 1.0));

    // Define C1 (outer) BEFORE C2 (inner) in IndexMap!
    m.load_combos.insert(
        "C1".into(),
        LoadCombo {
            name: "C1".into(),
            combo_type: ComboType::LinearAdd,
            cases: vec![("C2".into(), 2.0), ("CASE_A".into(), 0.5)],
        },
    );
    m.load_combos.insert(
        "C2".into(),
        LoadCombo {
            name: "C2".into(),
            combo_type: ComboType::LinearAdd,
            cases: vec![("CASE_A".into(), 1.0), ("CASE_B".into(), 1.5)],
        },
    );
    // Envelope with skipped non-run first case and negative governing case
    m.load_combos.insert(
        "ENV_SKIP".into(),
        LoadCombo {
            name: "ENV_SKIP".into(),
            combo_type: ComboType::Envelope,
            cases: vec![
                ("UNRUN".into(), 1.0),
                ("CASE_B".into(), 0.2), // -0.10
                ("CASE_A".into(), 0.3), // +0.06 -> |-0.10| > |0.06|, so -0.10 governs
            ],
        },
    );
    // Combo with 0 solved constituent cases must NOT be inserted
    m.load_combos.insert(
        "EMPTY_COMBO".into(),
        LoadCombo {
            name: "EMPTY_COMBO".into(),
            combo_type: ComboType::LinearAdd,
            cases: vec![("UNRUN".into(), 1.0), ("MISSING".into(), 2.0)],
        },
    );
    // Cyclic combos must be safely skipped without hanging
    m.load_combos.insert(
        "CYC_A".into(),
        LoadCombo {
            name: "CYC_A".into(),
            combo_type: ComboType::LinearAdd,
            cases: vec![("CYC_B".into(), 1.0)],
        },
    );
    m.load_combos.insert(
        "CYC_B".into(),
        LoadCombo {
            name: "CYC_B".into(),
            combo_type: ComboType::LinearAdd,
            cases: vec![("CYC_A".into(), 1.0)],
        },
    );

    analyze_model(&mut m).unwrap();

    // CASE_A: u = 0.2, P = 20; CASE_B: u = -0.5, P = -50
    // C2 = 1.0*CASE_A + 1.5*CASE_B => u = 0.2 - 0.75 = -0.55, P = 20 - 75 = -55
    // C1 = 2.0*C2 + 0.5*CASE_A => u = -1.10 + 0.10 = -1.00, P = -110 + 10 = -100
    close(m.case_results["C2"].joint_displacements["J"][0], -0.55);
    close(m.case_results["C1"].joint_displacements["J"][0], -1.00);
    close(m.case_results["C1"].frame_forces["B"][0].p, -100.0);
    close(m.case_results["C1"].base_reactions[0], 100.0);

    // ENV_SKIP: first contributed case is 0.2 * CASE_B (-0.10), which beats 0.3 * CASE_A (+0.06)
    close(m.case_results["ENV_SKIP"].joint_displacements["J"][0], -0.10);
    close(m.case_results["ENV_SKIP"].frame_forces["B"][0].p, -10.0);

    assert!(!m.case_results.contains_key("EMPTY_COMBO"));
    assert!(!m.case_results.contains_key("CYC_A"));
    assert!(!m.case_results.contains_key("CYC_B"));
}

#[test]
fn coupled_21_term_joint_spring_column_major_and_rotated_axes_equilibrium() {
    // 6x6 SPD spring matrix with distinct diagonals and off-diagonal couplings
    // including U1-U3 (0,2), U1-R2 (0,4), U2-R1 (1,3), U2-U3 (1,2).
    let ks: [[f64; 6]; 6] = [
        [200.0, 15.0, -35.0, 0.0, 28.0, 0.0],
        [15.0, 180.0, 22.0, -19.0, 0.0, 11.0],
        [-35.0, 22.0, 250.0, 0.0, -14.0, 0.0],
        [0.0, -19.0, 0.0, 140.0, 9.0, -12.0],
        [28.0, 0.0, -14.0, 9.0, 160.0, 7.0],
        [0.0, 11.0, 0.0, -12.0, 7.0, 130.0],
    ];
    // Pack in SAP2000 column-major upper-triangular order: for c in 0..6 { for r in 0..=c }
    let mut coupled = [0.0f64; 21];
    let mut pos = 0;
    for (c, col) in ks.iter().enumerate() {
        for &val in col.iter().take(c + 1) {
            coupled[pos] = val;
            pos += 1;
        }
    }
    assert_eq!(pos, 21);
    assert_eq!(coupled[3], -35.0); // U1-U3
    assert_eq!(coupled[10], 28.0); // U1-R2

    // Solve Ks * u_expected = f_vec by Gaussian elimination for closed-form reference
    let solve6 = |mat: [[f64; 6]; 6], rhs: [f64; 6]| -> [f64; 6] {
        let mut a = [[0.0; 7]; 6];
        for r in 0..6 {
            for c in 0..6 {
                a[r][c] = mat[r][c];
            }
            a[r][6] = rhs[r];
        }
        for k in 0..6 {
            let piv = a[k][k];
            for item in a[k].iter_mut().skip(k) {
                *item /= piv;
            }
            for r in 0..6 {
                if r != k {
                    let f = a[r][k];
                    let row_k: [f64; 7] = a[k];
                    for c in k..7 {
                        a[r][c] -= f * row_k[c];
                    }
                }
            }
        }
        std::array::from_fn(|r| a[r][6])
    };

    let f_applied = [12.0, -7.0, 19.0, 4.0, -9.0, 6.0];
    let u_expected = solve6(ks, f_applied);

    // Case A: spring_is_local = true with rotated joint axes & LOCAL applied load
    {
        let mut m = SapModel::default();
        m.load_patterns["DEAD"].self_wt_multiplier = 0.0;
        m.load_cases.clear();
        m.active_dofs = [true; 6];
        m.add_point("J", 3.0, -4.0, 5.0);
        let pt = m.points.get_mut("J").unwrap();
        pt.local_axes_deg = [35.0, -20.0, 50.0];
        pt.spring_is_local = true;
        pt.spring_coupled = Some(coupled);
        pt.loads_force.push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "LOCAL".into(),
            force: f_applied,
        });
        m.load_cases
            .insert("L".into(), LoadCase::linear_static("L", "DEAD", 1.0));
        analyze_model(&mut m).unwrap();

        let u_got = m.case_results["L"].joint_displacements["J"];
        let r_got = m.case_results["L"].joint_reactions["J"];
        for d in 0..6 {
            assert!(
                (u_got[d] - u_expected[d]).abs() < 1e-12,
                "local coupled u[{d}]: got {}, expected {}",
                u_got[d],
                u_expected[d]
            );
            assert!(
                (r_got[d] + f_applied[d]).abs() < 1e-12,
                "local coupled reaction[{d}]: got {}, expected {}",
                r_got[d],
                -f_applied[d]
            );
        }
    }

    // Case B: spring_is_local = false (Global coupled spring) on a joint with rotated 3D local axes!
    // Global displacement u_global = Ks^-1 * F_global MUST be invariant to joint local axis rotation!
    for (idx_case, angles) in [[0.0, 0.0, 0.0], [35.0, -20.0, 50.0]].iter().enumerate() {
        let mut m = SapModel::default();
        m.load_patterns["DEAD"].self_wt_multiplier = 0.0;
        m.load_cases.clear();
        m.active_dofs = [true; 6];
        m.add_point("J", 3.0, -4.0, 5.0);
        let pt = m.points.get_mut("J").unwrap();
        pt.local_axes_deg = *angles;
        pt.spring_is_local = false;
        pt.spring_coupled = Some(coupled);
        pt.loads_force.push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: f_applied,
        });
        m.load_cases
            .insert("G".into(), LoadCase::linear_static("G", "DEAD", 1.0));
        analyze_model(&mut m).unwrap();

        let res = &m.case_results["G"];
        if idx_case == 0 {
            let u_unrotated = res.joint_displacements["J"];
            for d in 0..6 {
                assert!((u_unrotated[d] - u_expected[d]).abs() < 1e-12);
            }
        } else {
            let r_p = sap_solver::frame::joint_local_to_global_matrix(&m.points["J"]);
            let u_loc = res.joint_displacements["J"];
            let r_loc = res.joint_reactions["J"];
            let mut u_glob = [0.0; 6];
            let mut r_glob = [0.0; 6];
            for g in 0..3 {
                u_glob[g] = r_p[g][0] * u_loc[0] + r_p[g][1] * u_loc[1] + r_p[g][2] * u_loc[2];
                u_glob[g + 3] =
                    r_p[g][0] * u_loc[3] + r_p[g][1] * u_loc[4] + r_p[g][2] * u_loc[5];
                r_glob[g] = r_p[g][0] * r_loc[0] + r_p[g][1] * r_loc[1] + r_p[g][2] * r_loc[2];
                r_glob[g + 3] =
                    r_p[g][0] * r_loc[3] + r_p[g][1] * r_loc[4] + r_p[g][2] * r_loc[5];
            }
            for d in 0..6 {
                assert!(
                    (u_glob[d] - u_expected[d]).abs() < 1e-12,
                    "rotated global-spring u_glob[{d}]: got {}, expected {}",
                    u_glob[d],
                    u_expected[d]
                );
                assert!(
                    (r_glob[d] + f_applied[d]).abs() < 1e-12,
                    "rotated global-spring r_glob[{d}]: got {}, expected {}",
                    r_glob[d],
                    -f_applied[d]
                );
            }
        }
        // Global base reactions must balance F_applied + r x F_applied about origin
        let br = res.base_reactions;
        assert!((br[0] + f_applied[0]).abs() < 1e-11);
        assert!((br[1] + f_applied[1]).abs() < 1e-11);
        assert!((br[2] + f_applied[2]).abs() < 1e-11);
        let mx = f_applied[3] + (-4.0 * f_applied[2] - 5.0 * f_applied[1]);
        let my = f_applied[4] + (5.0 * f_applied[0] - 3.0 * f_applied[2]);
        let mz = f_applied[5] + (3.0 * f_applied[1] - (-4.0) * f_applied[0]);
        assert!((br[3] + mx).abs() < 1e-11);
        assert!((br[4] + my).abs() < 1e-11);
        assert!((br[5] + mz).abs() < 1e-11);
    }
}

#[test]
fn cable_from_target_tension_i_closed_form_and_catenary_inversion() {
    use sap_solver::cable::CableElement;

    // 1. Weightless straight cable: exact L0 = L_c / (1 + T_I / EA)
    let elem_w0 = CableElement::from_target_tension_i([30.0, 40.0, 120.0], 10_000.0, 250.0, 0.0)
        .expect("weightless target_tension_i");
    let expected_l0 = 130.0 / (1.0 + 250.0 / 10_000.0);
    assert!((elem_w0.l0 - expected_l0).abs() < 1e-13);
    let st_w0 = elem_w0.evaluate([0.0; 3], [0.0; 3]).unwrap();
    assert!((st_w0.tension_i - 250.0).abs() < 1e-11);

    // 2. Vertical cable with J above I (dz > 0) and J below I (dz < 0)
    let elem_up = CableElement::from_target_tension_i([0.0, 0.0, 80.0], 10_000.0, 25.0, 0.05)
        .expect("vertical upward target_tension_i");
    let st_up = elem_up.evaluate([0.0; 3], [0.0; 3]).unwrap();
    assert!((st_up.tension_i - 25.0).abs() < 1e-11);
    assert!((st_up.tension_j - (25.0 + 0.05 * elem_up.l0)).abs() < 1e-11);

    let elem_dn = CableElement::from_target_tension_i([0.0, 0.0, -80.0], 10_000.0, 25.0, 0.05)
        .expect("vertical downward target_tension_i");
    let st_dn = elem_dn.evaluate([0.0; 3], [0.0; 3]).unwrap();
    assert!((st_dn.tension_i - 25.0).abs() < 1e-11);
    assert!((st_dn.tension_j - (25.0 - 0.05 * elem_dn.l0)).abs() < 1e-11);

    let err_vert =
        CableElement::from_target_tension_i([0.0, 0.0, -80.0], 10_000.0, 3.0, 0.05).unwrap_err();
    assert!(
        err_vert
            .to_string()
            .contains("below minimum achievable catenary tension"),
        "{err_vert}"
    );

    // 3. Horizontal and inclined 3D catenaries: exact inversion < 1e-11 and T_I < T_min rejection
    for (chord, ea, ti, w_self) in [
        ([200.0, 0.0, 0.0], 50_000.0, 30.0, 0.02),
        ([120.0, 90.0, 45.0], 30_000.0, 40.0, 0.015),
        ([120.0, 90.0, -45.0], 30_000.0, 40.0, 0.015),
    ] {
        let elem = CableElement::from_target_tension_i(chord, ea, ti, w_self).unwrap();
        let st = elem.evaluate([0.0; 3], [0.0; 3]).unwrap();
        assert!(
            (st.tension_i - ti).abs() < 1e-11,
            "chord={chord:?}: got T_I={}, expected {ti}",
            st.tension_i
        );
    }
    let err_cat =
        CableElement::from_target_tension_i([200.0, 0.0, 0.0], 50_000.0, 0.5, 0.02).unwrap_err();
    assert!(
        err_cat
            .to_string()
            .contains("below minimum achievable catenary tension"),
        "{err_cat}"
    );

    // 4. Full model test: SAP2000 Verification Example 7-003 (3x3 Prestressed Cable Net,
    //    100 ft grid spacing, interior nodes at z = 10 ft, boundary supports at z = 40 ft (f = 30 ft),
    //    A = 0.227 in^2, E = 12,000 ksi, w_self = 0.0001 kip/ft,
    //    target_tension_i = 5.325 kips on 8 inclined members and 5.459 kips on 4 horizontal members,
    //    -8.0 kip vertical load at interior nodes 1..4)
    let mut net = SapModel::new(sap_units::Units::KipFtF);
    net.load_patterns.clear();
    net.load_cases.clear();
    net.load_patterns.insert(
        "SELF".into(),
        LoadPattern {
            name: "SELF".into(),
            pat_type: LoadPatternType::Dead,
            self_wt_multiplier: 1.0,
        },
    );
    net.load_patterns.insert(
        "POINT".into(),
        LoadPattern {
            name: "POINT".into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
    let mut mat = Material::default_steel_kip_in();
    mat.name = "CABLE_MAT".into();
    mat.e = 12_000.0 * 144.0; // kips/ft^2
    mat.a = 0.0;
    let area_ft2 = 0.227 / 144.0; // ft^2
    mat.unit_weight = 0.0001 / area_ft2; // kip/ft^3 -> weight_per_len = 0.0001 kip/ft
    mat.unit_mass = 0.0;
    net.materials.insert("CABLE_MAT".into(), mat);
    net.frame_sections.insert(
        "CAB_SEC".into(),
        FrameSection {
            name: "CAB_SEC".into(),
            material: "CABLE_MAT".into(),
            area: area_ft2,
            ..FrameSection::default()
        },
    );

    net.add_point("1", 100.0, 100.0, 10.0);
    net.add_point("2", 200.0, 100.0, 10.0);
    net.add_point("3", 100.0, 200.0, 10.0);
    net.add_point("4", 200.0, 200.0, 10.0);
    for (sname, x, y) in [
        ("5", 100.0, 0.0),
        ("6", 200.0, 0.0),
        ("7", 0.0, 100.0),
        ("8", 300.0, 100.0),
        ("9", 0.0, 200.0),
        ("10", 300.0, 200.0),
        ("11", 100.0, 300.0),
        ("12", 200.0, 300.0),
    ] {
        net.add_point(sname, x, y, 40.0);
        net.points.get_mut(sname).unwrap().restraint = [true; 6];
    }
    for j in ["1", "2", "3", "4"] {
        net.points.get_mut(j).unwrap().loads_force.push(PointLoadForce {
            load_pat: "POINT".into(),
            csys: "GLOBAL".into(),
            force: [0.0, 0.0, -8.0, 0.0, 0.0, 0.0],
        });
    }
    let inclined = [
        ("1", "7", "1"),
        ("4", "9", "3"),
        ("9", "5", "1"),
        ("12", "6", "2"),
        ("3", "8", "2"),
        ("6", "10", "4"),
        ("10", "4", "12"),
        ("7", "3", "11"),
    ];
    for (cname, pi, pj) in inclined {
        net.add_frame(cname, pi, pj, "CAB_SEC");
        let f = net.frames.get_mut(cname).unwrap();
        f.tc_limits = Some((true, 0.0, false, 0.0)); // intrinsic zero-compression limit from SAP2000
        f.cable_params = Some(CableParams {
            undeformed_length: None,
            target_tension_i: Some(5.325),
            horizontal_tension: None,
            num_segments: 1,
        });
    }
    let horizontal = [
        ("2", "1", "2"),
        ("5", "3", "4"),
        ("11", "2", "4"),
        ("8", "3", "1"),
    ];
    for (cname, pi, pj) in horizontal {
        net.add_frame(cname, pi, pj, "CAB_SEC");
        let f = net.frames.get_mut(cname).unwrap();
        f.tc_limits = Some((true, 0.0, false, 0.0));
        f.cable_params = Some(CableParams {
            undeformed_length: None,
            target_tension_i: Some(5.459),
            horizontal_tension: None,
            num_segments: 1,
        });
    }
    let mut self_case = LoadCase::linear_static("SELF", "SELF", 1.0);
    self_case.case_type = LoadCaseType::NonlinearStatic;
    self_case.nl_geom = NLGeomType::LargeDisplacements;
    self_case.convergence_tol = 1e-10;
    net.load_cases.insert("SELF".into(), self_case);

    let mut point_case = LoadCase::linear_static("POINT", "POINT", 1.0);
    point_case.case_type = LoadCaseType::NonlinearStatic;
    point_case.nl_geom = NLGeomType::LargeDisplacements;
    point_case.initial_case = Some("SELF".into());
    point_case.convergence_tol = 1e-10;
    net.load_cases.insert("POINT".into(), point_case);

    // Competing shape specs (undeformed_length + target_tension_i) must be explicitly rejected
    let mut bad_net = net.clone();
    bad_net
        .frames
        .get_mut("1")
        .unwrap()
        .cable_params
        .as_mut()
        .unwrap()
        .undeformed_length = Some(100.0);
    let err_dup = analyze_model(&mut bad_net).unwrap_err();
    assert!(err_dup.to_string().contains("target end tension"), "{err_dup}");

    analyze_model(&mut net).unwrap();
    let u1 = net.case_results["POINT"].joint_displacements["1"];
    // SAP2000 Example 7-003 reference: Ux = -0.1328 ft, Uy = -0.1328 ft, Uz = -1.4745 ft
    assert!((u1[0] - (-0.1328)).abs() < 2e-4, "Ux(1) = {}", u1[0]);
    assert!((u1[1] - (-0.1328)).abs() < 2e-4, "Uy(1) = {}", u1[1]);
    assert!((u1[2] - (-1.4745)).abs() < 2e-4, "Uz(1) = {}", u1[2]);
}
