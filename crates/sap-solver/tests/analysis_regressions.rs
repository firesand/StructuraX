use sap_model::{
    DynamicFunction, LoadCase, LoadCaseType, PointLoadForce, SapModel, TimeIntegrationMethod,
};
use sap_solver::analyze_model;

fn oscillator() -> SapModel {
    let mut model = SapModel::default();
    model.load_cases.clear();
    model
        .load_patterns
        .get_mut("DEAD")
        .unwrap()
        .self_wt_multiplier = 0.0;
    model.active_dofs = [true, false, false, false, false, false];
    model.add_point("M", 0.0, 0.0, 0.0);
    let point = model.points.get_mut("M").unwrap();
    point.mass[0] = 1.0;
    point.spring[0] = 4.0;
    point.loads_force.push(PointLoadForce {
        load_pat: "DEAD".into(),
        csys: "GLOBAL".into(),
        force: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    });
    model.functions.insert(
        "STEP".into(),
        DynamicFunction {
            name: "STEP".into(),
            is_time_history: true,
            damping_ratio: 0.0,
            points: vec![(0.0, 1.0), (1.0, 1.0)],
        },
    );
    let mut case = LoadCase::linear_static("HISTORY", "DEAD", 1.0);
    case.case_type = LoadCaseType::LinearDynamic;
    case.history_loads = vec![("STEP".into(), "DEAD".into(), 1.0)];
    case.time_step_dt = 0.1;
    case.num_time_steps = 1;
    model.load_cases.insert("HISTORY".into(), case);
    model
}

#[test]
fn newmark_step_load_obeys_initial_equilibrium() {
    // m=1, k=4, F(0)=F(dt)=1, u(0)=v(0)=0 implies a(0)=1.
    // Average acceleration gives u(dt)=2/(4/dt^2 + k), not half this.
    let mut model = oscillator();
    analyze_model(&mut model).unwrap();
    let u = model.case_results["HISTORY"].time_history_disp["M"][1][0];
    assert!((u - 2.0 / 404.0).abs() < 1e-14, "u(dt) = {u}");
    // Independent continuous solution: u(t)=(1-cos(2t))/4.
    model.load_cases["HISTORY"].time_step_dt = 0.001;
    model.load_cases["HISTORY"].num_time_steps = 1000;
    analyze_model(&mut model).unwrap();
    let u = model.case_results["HISTORY"].time_history_disp["M"][1000][0];
    assert!((u - (1.0 - 2.0_f64.cos()) / 4.0).abs() < 2e-7);
}

#[test]
fn hht_and_wilson_solve_complete_model_histories() {
    for method in [
        TimeIntegrationMethod::HilberHughesTaylor,
        TimeIntegrationMethod::WilsonTheta,
    ] {
        let mut model = oscillator();
        let settings = &mut model.load_cases["HISTORY"];
        settings.time_integration = method;
        settings.hht_alpha = -0.2;
        settings.wilson_theta = 1.4;
        settings.time_step_dt = 0.001;
        settings.num_time_steps = 1000;
        analyze_model(&mut model).unwrap();
        let result = &model.case_results["HISTORY"];
        assert_eq!(result.time_steps.len(), 1001);
        assert!((result.time_steps[1000] - 1.0).abs() < 1e-14);
        let u = result.time_history_disp["M"][1000][0];
        let exact = (1.0 - 2.0_f64.cos()) / 4.0;
        assert!((u - exact).abs() < 2e-6, "{method:?}: {u} versus {exact}");
        assert!(model.is_locked);
    }
}

#[test]
fn tip_moment_history_initializes_massless_frame_rotation_consistently() {
    // Euler-Bernoulli cantilever, L=10, EI=100*(2/3), tip translation mass=1.
    // Condensing its massless rotation gives k=3EI/L^3=.2 and the applied
    // unit moment gives equivalent translation load 3M/(2L)=.15.
    for method in [
        TimeIntegrationMethod::NewmarkAverage,
        TimeIntegrationMethod::NewmarkLinear,
        TimeIntegrationMethod::HilberHughesTaylor,
        TimeIntegrationMethod::WilsonTheta,
    ] {
        let mut model = SapModel::default();
        model.load_cases.clear();
        model.load_patterns["DEAD"].self_wt_multiplier = 0.0;
        let mut material = sap_model::Material::default_steel_kip_in();
        material.name = "M".into();
        material.e = 100.0;
        material.g = 40.0;
        material.unit_mass = 0.0;
        material.unit_weight = 0.0;
        model.materials.insert("M".into(), material);
        let mut section = sap_section::make_rectangular("S", "M", 2.0, 1.0);
        section.as2 = 0.0;
        section.as3 = 0.0;
        model.frame_sections.insert("S".into(), section);
        model.add_point("BASE", 0.0, 0.0, 0.0);
        model.points["BASE"].restraint = [true; 6];
        model.add_point("TIP", 10.0, 0.0, 0.0);
        model.points["TIP"].restraint = [true, true, false, true, false, true];
        model.points["TIP"].mass[2] = 1.0;
        model.points["TIP"].loads_force.push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: [0.0, 0.0, 0.0, 0.0, -1.0, 0.0],
        });
        model.add_frame("BEAM", "BASE", "TIP", "S");
        model.functions.insert(
            "STEP".into(),
            DynamicFunction {
                name: "STEP".into(),
                is_time_history: true,
                damping_ratio: 0.0,
                points: vec![(0.0, 1.0), (2.0, 1.0)],
            },
        );
        let mut settings = LoadCase::linear_static("H", "DEAD", 1.0);
        settings.case_type = LoadCaseType::LinearDynamic;
        settings.time_integration = method;
        settings.hht_alpha = -0.2;
        settings.time_step_dt = 0.001;
        settings.num_time_steps = 1000;
        settings.history_loads = vec![("STEP".into(), "DEAD".into(), 1.0)];
        model.load_cases.insert("H".into(), settings);
        analyze_model(&mut model).unwrap();
        let trace = &model.case_results["H"].time_history_disp["TIP"];
        assert!(trace[0][2].abs() < 1e-14);
        assert!((trace[0][4] + 0.0375).abs() < 1e-12);
        let exact_u = 0.75 * (1.0 - 0.2_f64.sqrt().cos());
        assert!((trace[1000][2] - exact_u).abs() < 1e-6, "{method:?}");
        assert!((trace[1000][4] + 0.15 * exact_u + 0.0375).abs() < 1e-6);
    }
}

#[test]
fn unsupported_analysis_requests_fail_without_results() {
    for case_type in [
        LoadCaseType::MovingLoad,
        LoadCaseType::SteadyState,
        LoadCaseType::PowerSpectralDensity,
        LoadCaseType::HyperStatic,
        LoadCaseType::LinearStaticMultiStep,
    ] {
        let mut model = oscillator();
        model.load_cases["HISTORY"].case_type = case_type;
        assert!(analyze_model(&mut model)
            .unwrap_err()
            .to_string()
            .contains("Unsupported"));
        assert!(!model.is_locked);
        assert!(model.case_results.is_empty());
    }
    for method in [
        TimeIntegrationMethod::WilsonTheta,
        TimeIntegrationMethod::HilberHughesTaylor,
    ] {
        let mut model = oscillator();
        model.load_cases["HISTORY"].time_integration = method;
        model.load_cases["HISTORY"].wilson_theta = 0.5;
        model.load_cases["HISTORY"].hht_alpha = 0.5;
        assert!(analyze_model(&mut model).is_err());
    }
}

#[test]
fn missing_function_and_invalid_timestep_do_not_silently_change_loading() {
    let mut model = oscillator();
    model.functions.clear();
    assert!(analyze_model(&mut model)
        .unwrap_err()
        .to_string()
        .contains("missing function"));
    for dt in [0.0, -0.1, f64::NAN, f64::INFINITY] {
        let mut model = oscillator();
        model.load_cases["HISTORY"].time_step_dt = dt;
        assert!(analyze_model(&mut model).is_err());
    }
}

#[test]
fn failed_reanalysis_clears_previous_and_partial_results() {
    let mut model = oscillator();
    analyze_model(&mut model).unwrap();
    assert!(model.is_locked);
    model.load_cases["HISTORY"].time_step_dt = 0.0;
    assert!(analyze_model(&mut model).is_err());
    assert!(!model.is_locked);
    assert!(model.case_results.is_empty());

    // Fail during a later solve, after an earlier case has already succeeded.
    model.load_cases.clear();
    model.active_dofs = [true; 6];
    let mut fail_pattern = model.load_patterns["DEAD"].clone();
    fail_pattern.name = "FAIL".into();
    model.load_patterns.insert("FAIL".into(), fail_pattern);
    model.points["M"].loads_force.push(PointLoadForce {
        load_pat: "FAIL".into(),
        csys: "GLOBAL".into(),
        force: [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
    });
    model
        .load_cases
        .insert("OK".into(), LoadCase::linear_static("OK", "DEAD", 1.0));
    model
        .load_cases
        .insert("FAIL".into(), LoadCase::linear_static("FAIL", "FAIL", 1.0));
    assert!(analyze_model(&mut model).is_err());
    assert!(!model.is_locked);
    assert!(model.case_results.is_empty());
}

#[test]
fn history_forces_and_acceleration_follow_rotated_joint_axes() {
    for acceleration in [false, true] {
        let mut model = oscillator();
        model.active_dofs = [true; 6];
        let point = model.points.get_mut("M").unwrap();
        point.local_axes_matrix = Some([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]);
        point.spring_is_local = true;
        point.restraint = [false, true, true, true, true, true];
        point.loads_force[0].force = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0];
        if acceleration {
            model.load_cases["HISTORY"].history_loads[0].1 = "U2".into();
        }
        analyze_model(&mut model).unwrap();
        let u = model.case_results["HISTORY"].time_history_disp["M"][1][0];
        let expected = if acceleration {
            -2.0 / 404.0
        } else {
            2.0 / 404.0
        };
        assert!(
            (u - expected).abs() < 1e-14,
            "acceleration={acceleration}, u={u}"
        );
    }
}

#[test]
fn history_self_weight_is_rejected_instead_of_ignored() {
    let mut model = oscillator();
    model.load_patterns["DEAD"].self_wt_multiplier = 1.0;
    assert!(analyze_model(&mut model)
        .unwrap_err()
        .to_string()
        .contains("only joint forces"));
}

#[test]
fn invalid_frame_and_auto_mesh_connectivity_are_rejected() {
    let mut model = oscillator();
    model.add_frame("ZERO", "M", "M", "FSEC1");
    assert!(analyze_model(&mut model)
        .unwrap_err()
        .to_string()
        .contains("invalid length"));
    model.frames.clear();
    let mut area = sap_model::AreaObj::new(
        "A",
        vec!["M".into(), "M".into(), "M".into(), "MISSING".into()],
        "ASEC1",
    );
    area.auto_mesh = (2, 2);
    model.areas.insert("A".into(), area);
    assert!(analyze_model(&mut model)
        .unwrap_err()
        .to_string()
        .contains("MISSING"));
}

#[test]
fn invalid_area_geometry_is_rejected_before_loads_can_disappear() {
    for corners in [
        vec![(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)],
        vec![(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)],
        vec![(0.0, 0.0), (1.0, 0.0), (0.0, 0.0), (0.0, 1.0)],
        vec![(0.0, 0.0), (1.0, 1.0), (0.0, 1.0), (1.0, 0.0)],
        vec![(0.0, 0.0), (1.0, 0.0), (0.2, 0.2), (0.0, 1.0)],
    ] {
        let mut model = SapModel::default();
        let mut names = Vec::new();
        for (i, (x, y)) in corners.into_iter().enumerate() {
            let name = i.to_string();
            model.add_point(&name, x, y, 0.0);
            model.points[&name].restraint = [true; 6];
            names.push(name);
        }
        model.add_area("BAD", names, "ASEC1");
        model.areas["BAD"]
            .loads_uniform
            .push(sap_model::AreaLoadUniform {
                load_pat: "DEAD".into(),
                csys: "GLOBAL".into(),
                dir: 6,
                val: -10.0,
            });
        let error = analyze_model(&mut model).unwrap_err().to_string();
        assert!(error.contains("geometry"), "{error}");
        assert!(!model.is_locked);
        assert!(model.case_results.is_empty());
    }
    // The guard accepts either node orientation, including after auto-meshing.
    for reverse in [false, true] {
        let mut model = SapModel::default();
        for (name, x, y) in [
            ("1", 0.0, 0.0),
            ("2", 2.0, 0.0),
            ("3", 1.5, 1.0),
            ("4", 0.0, 1.0),
        ] {
            model.add_point(name, x, y, 0.0);
            model.points[name].restraint = [true; 6];
        }
        let mut names: Vec<String> = ["1", "2", "3", "4"].map(String::from).into();
        if reverse {
            names.reverse();
        }
        model.add_area("GOOD", names, "ASEC1");
        model.areas["GOOD"].auto_mesh = (2, 2);
        analyze_model(&mut model).unwrap();
        assert!(model.is_locked);
    }
}

#[test]
fn ldlt_checks_finiteness_and_is_invariant_to_uniform_scaling() {
    use sap_solver::linalg::SymmetricLdlt;
    let tiny = SymmetricLdlt::factorize(&[1e-24], 1).unwrap();
    assert!((tiny.solve(&[1e-24]).unwrap()[0] - 1.0).abs() < 1e-14);
    let empty = SymmetricLdlt::factorize(&[0.0], 1).unwrap();
    assert!(
        empty.solve(&[1e-12]).is_err(),
        "nonzero load on a mechanism must not be discarded"
    );
    assert!(SymmetricLdlt::factorize(&[f64::NAN], 1).is_err());
    let regular = SymmetricLdlt::factorize(&[1.0], 1).unwrap();
    assert!(regular.solve(&[f64::INFINITY]).is_err());
}

#[test]
fn missing_named_load_coordinate_systems_are_rejected_on_all_static_load_paths() {
    for kind in 0..5 {
        for pattern in ["DEAD", "UNUSED"] {
            let mut model = SapModel::default();
            for (name, x, y) in [
                ("1", 0.0, 0.0),
                ("2", 1.0, 0.0),
                ("3", 1.0, 1.0),
                ("4", 0.0, 1.0),
            ] {
                model.add_point(name, x, y, 0.0);
                model.points[name].restraint = [true; 6];
            }
            model.add_frame("F", "1", "2", "FSEC1");
            model.add_area("A", ["1", "2", "3", "4"].map(String::from).into(), "ASEC1");
            let csys = "ROTATED_GRID".to_string();
            let load_pat = pattern.to_string();
            match kind {
                0 => model.points["1"].loads_force.push(PointLoadForce {
                    load_pat,
                    csys,
                    force: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                }),
                1 => model.points["1"]
                    .loads_displ
                    .push(sap_model::PointLoadDispl {
                        load_pat,
                        csys,
                        displ: [0.01, 0.0, 0.0, 0.0, 0.0, 0.0],
                    }),
                2 => model.frames["F"]
                    .loads_dist
                    .push(sap_model::FrameLoadDistributed {
                        load_pat,
                        csys,
                        my_type: 1,
                        dir: 6,
                        rel_dist1: 0.0,
                        rel_dist2: 1.0,
                        val1: -1.0,
                        val2: -1.0,
                    }),
                3 => model.frames["F"]
                    .loads_point
                    .push(sap_model::FrameLoadPoint {
                        load_pat,
                        csys,
                        my_type: 1,
                        dir: 6,
                        rel_dist: 0.5,
                        val: -1.0,
                    }),
                _ => model.areas["A"]
                    .loads_uniform
                    .push(sap_model::AreaLoadUniform {
                        load_pat,
                        csys,
                        dir: 6,
                        val: -1.0,
                    }),
            }
            if pattern == "DEAD" {
                let error = analyze_model(&mut model).unwrap_err().to_string();
                assert!(error.contains("ROTATED_GRID"), "{kind}: {error}");
                assert!(model.case_results.is_empty());
                assert!(!model.is_locked);
            } else {
                analyze_model(&mut model).unwrap();
            }
        }
    }
}

#[test]
fn initial_case_is_solved_before_dependent_and_invalid_dependencies_fail() {
    let mut model = oscillator();
    model.load_cases.clear();
    let mut dependent = LoadCase::linear_static("DEPENDENT", "DEAD", 1.0);
    dependent.initial_case = Some("PRELOAD".into());
    model.load_cases.insert("DEPENDENT".into(), dependent);
    model.load_cases.insert(
        "PRELOAD".into(),
        LoadCase::linear_static("PRELOAD", "DEAD", 1.0),
    );
    analyze_model(&mut model).unwrap();
    assert_eq!(
        model
            .case_results
            .keys()
            .map(String::as_str)
            .collect::<Vec<_>>(),
        vec!["PRELOAD", "DEPENDENT"]
    );
    model.load_cases["PRELOAD"].initial_case = Some("DEPENDENT".into());
    assert!(analyze_model(&mut model)
        .unwrap_err()
        .to_string()
        .contains("cyclic"));
    model.load_cases["PRELOAD"].initial_case = None;
    model.load_cases["PRELOAD"].run_flag = false;
    assert!(analyze_model(&mut model)
        .unwrap_err()
        .to_string()
        .contains("disabled"));
}
