use sap_model::*;
use sap_solver::analyze_model;

fn close(a: f64, b: f64) {
    assert!((a - b).abs() < 1e-8 * (1. + b.abs()), "{a} != {b}");
}
fn model() -> SapModel {
    let mut m = SapModel::default();
    m.load_patterns["DEAD"].self_wt_multiplier = 0.;
    m.load_cases.clear();
    m.load_cases
        .insert("S".into(), LoadCase::linear_static("S", "DEAD", 1.));
    m
}
fn grid(m: &mut SapModel, z: f64) {
    m.coord_systems.insert(
        "GRID".into(),
        CoordSystem {
            name: "GRID".into(),
            origin: [12., -6., 8.],
            angles_deg: [z, 0., 0.],
            grids: vec![],
        },
    );
}
fn force(values: [f64; 6], csys: &str) -> PointLoadForce {
    PointLoadForce {
        load_pat: "DEAD".into(),
        csys: csys.into(),
        force: values,
    }
}
fn compare(a: &SapModel, b: &SapModel) {
    let ar = &a.case_results["S"];
    let br = &b.case_results["S"];
    for (name, disp) in &ar.joint_displacements {
        for d in 0..6 {
            close(disp[d], br.joint_displacements[name][d]);
        }
    }
    for d in 0..6 {
        close(ar.base_reactions[d], br.base_reactions[d]);
    }
    for (name, stations) in &ar.frame_forces {
        for (a, b) in stations.iter().zip(&br.frame_forces[name]) {
            for (x, y) in [a.p, a.v2, a.v3, a.t, a.m2, a.m3]
                .into_iter()
                .zip([b.p, b.v2, b.v3, b.t, b.m2, b.m3])
            {
                close(x, y);
            }
        }
    }
}

#[test]
fn named_joint_force_moment_and_support_motion_equal_explicit_global_vectors() {
    for motion in [false, true] {
        let mut a = model();
        grid(&mut a, 90.);
        a.add_point("A", 0., 0., 0.);
        a.add_point("B", 2., 0., 0.);
        a.points["A"].restraint = [true; 6];
        a.add_frame("F", "A", "B", "FSEC1");
        // A second rotation at the joints checks composition of both transforms.
        a.points["B"].local_axes_deg = [30., 0., 0.];
        if motion {
            a.points["A"].loads_displ.push(PointLoadDispl {
                load_pat: "DEAD".into(),
                csys: "GRID".into(),
                displ: [0.01, 0.02, 0.03, 0.001, 0.002, 0.003],
            });
        } else {
            a.points["B"]
                .loads_force
                .push(force([1., 2., 3., 4., 5., 6.], "GRID"));
        }
        let mut b = a.clone();
        if motion {
            b.points["A"].loads_displ[0].csys = "GLOBAL".into();
            b.points["A"].loads_displ[0].displ = [-0.02, 0.01, 0.03, -0.002, 0.001, 0.003];
        } else {
            b.points["B"].loads_force[0] = force([-2., 1., 3., -5., 4., 6.], "GLOBAL");
        }
        analyze_model(&mut a).unwrap();
        analyze_model(&mut b).unwrap();
        compare(&a, &b);
        if !motion {
            assert_eq!(a.points["B"].loads_force[0].csys, "GRID");
        }
    }
}

#[test]
fn projected_named_distributed_load_and_named_point_moment_preserve_equilibrium() {
    let mut a = model();
    grid(&mut a, 45.);
    a.add_point("A", 0., 0., 0.);
    a.add_point("B", 4., 0., 0.);
    a.points["A"].restraint = [true; 6];
    a.add_frame("F", "A", "B", "FSEC1");
    a.frames["F"].loads_dist.push(FrameLoadDistributed {
        load_pat: "DEAD".into(),
        csys: "GRID".into(),
        my_type: 1,
        dir: 7,
        rel_dist1: 0.,
        rel_dist2: 1.,
        val1: 2.,
        val2: 4.,
    });
    a.frames["F"].loads_point.push(FrameLoadPoint {
        load_pat: "DEAD".into(),
        csys: "GRID".into(),
        my_type: 2,
        dir: 4,
        rel_dist: 0.3,
        val: 2f64.sqrt(),
    });
    let mut b = a.clone();
    b.frames["F"].loads_dist.clear();
    b.frames["F"].loads_point.clear();
    for dir in [4, 5] {
        let mut ld = a.frames["F"].loads_dist[0].clone();
        ld.csys = "GLOBAL".into();
        ld.dir = dir;
        ld.val1 = 1.;
        ld.val2 = 2.;
        b.frames["F"].loads_dist.push(ld);
        let mut lp = a.frames["F"].loads_point[0].clone();
        lp.csys = "GLOBAL".into();
        lp.dir = dir;
        lp.val = 1.;
        b.frames["F"].loads_point.push(lp);
    }
    analyze_model(&mut a).unwrap();
    analyze_model(&mut b).unwrap();
    compare(&a, &b);
    close(a.case_results["S"].base_reactions[0], -6.);
    close(a.case_results["S"].base_reactions[1], -6.);
}

#[test]
fn named_area_load_rotates_with_grid_and_preserves_total_force() {
    let mut a = model();
    grid(&mut a, 90.);
    for (name, x, y) in [("A", 0., 0.), ("B", 2., 0.), ("C", 2., 3.), ("D", 0., 3.)] {
        a.add_point(name, x, y, 0.);
        a.points[name].restraint = [true; 6];
    }
    a.add_area(
        "AREA",
        vec!["A".into(), "B".into(), "C".into(), "D".into()],
        "ASEC1",
    );
    a.areas["AREA"].loads_uniform.push(AreaLoadUniform {
        load_pat: "DEAD".into(),
        csys: "GRID".into(),
        dir: 4,
        val: 7.,
    });
    let mut b = a.clone();
    b.areas["AREA"].loads_uniform[0].csys = "GLOBAL".into();
    b.areas["AREA"].loads_uniform[0].dir = 5;
    analyze_model(&mut a).unwrap();
    analyze_model(&mut b).unwrap();
    compare(&a, &b);
    close(a.case_results["S"].base_reactions[1], -42.);
}

fn link_model(grounded: bool) -> SapModel {
    let mut m = model();
    m.add_point("I", 0., 0., 0.);
    let j = if grounded {
        "I"
    } else {
        m.add_point("J", 3., 0., 0.);
        m.points["I"].restraint = [true; 6];
        "J"
    };
    m.points[j].mass = [2.; 6];
    let p = LinkProp {
        name: "P".into(),
        dof_active: [true; 6],
        ke: [20., 30., 40., 50., 60., 70.],
        ..LinkProp::default()
    };
    m.link_props.insert("P".into(), p);
    m.links.insert(
        "L".into(),
        LinkObj {
            name: "L".into(),
            point_i: "I".into(),
            point_j: if grounded { None } else { Some("J".into()) },
            prop: "P".into(),
            angle_deg: 0.,
            selected: false,
        },
    );
    m
}

#[test]
fn link_offsets_rotation_and_joint_axes_match_independent_strain_solution() {
    let mut m = link_model(false);
    // local 1=X, local 2=Y, local 3=Z after -90-degree rotation.
    m.links["L"].angle_deg = -90.;
    m.points["J"].local_axes_deg = [90., 0., 0.];
    m.link_props["P"].dj2 = 0.7;
    m.link_props["P"].dj3 = 0.4;
    // Chosen global J displacement [1,2,3,.1,.2,.3]. Generalized strains:
    // [1,2-.7*.3,3+.4*.2,.1,-.2,.3]. J load = B_j^T K strain.
    let f = [
        20.,
        30. * 1.79,
        40. * 3.08,
        5.,
        60. * 0.2 + 0.4 * 40. * 3.08,
        70. * 0.3 - 0.7 * 30. * 1.79,
    ];
    m.points["J"].loads_force.push(force(f, "GLOBAL"));
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["S"];
    for (a, b) in r.joint_displacements["J"]
        .iter()
        .zip([2., -1., 3., 0.2, -0.1, 0.3])
    {
        close(*a, b);
    }
    for (a, b) in
        r.link_forces["L"]
            .iter()
            .zip([20., 53.7, 123.2, 5., -12. - 0.4 * 123.2, 21. - 0.7 * 53.7])
    {
        close(*a, b);
    }
    for d in 0..3 {
        close(r.base_reactions[d], -f[d]);
    }
    close(r.base_reactions[4], -f[4] + 3. * f[2]);
    close(r.base_reactions[5], -f[5] - 3. * f[1]);
}

#[test]
fn grounded_link_axes_half_mass_and_weight_follow_csi_convention() {
    let mut m = link_model(true);
    m.points["I"].restraint = [true; 6];
    m.link_props["P"].weight = 10.;
    m.load_patterns["DEAD"].self_wt_multiplier = 1.;
    analyze_model(&mut m).unwrap();
    close(m.case_results["S"].base_reactions[2], 5.);
    m.points["I"].restraint = [false; 6];
    m.load_patterns["DEAD"].self_wt_multiplier = 0.;
    m.points["I"]
        .loads_force
        .push(force([3., 4., 2., 0., 0., 0.], "GLOBAL"));
    analyze_model(&mut m).unwrap();
    // Default zero-length axes: 1=Z, 2=X, 3=Y.
    for (a, b) in m.case_results["S"].joint_displacements["I"]
        .iter()
        .zip([0.1, 0.1, 0.1, 0., 0., 0.])
    {
        close(*a, b);
    }
    for (a, b) in m.case_results["S"].link_forces["L"]
        .iter()
        .zip([2., 3., 4., 0., 0., 0.])
    {
        close(*a, b);
    }
}

#[test]
fn viscous_link_history_matches_closed_form_for_all_integrators_and_named_load() {
    for method in [
        TimeIntegrationMethod::NewmarkAverage,
        TimeIntegrationMethod::NewmarkLinear,
        TimeIntegrationMethod::HilberHughesTaylor,
        TimeIntegrationMethod::WilsonTheta,
    ] {
        let mut m = link_model(true);
        grid(&mut m, 90.);
        m.active_dofs = [true, false, false, false, false, false];
        m.link_props["P"].ke[1] = 8.;
        m.link_props["P"].ce[1] = 0.8;
        m.link_props["P"].mass = 2.;
        m.points["I"].mass = [1.; 6]; // effective m=1+2/2=2
        m.points["I"]
            .loads_force
            .push(force([0., -3., 0., 0., 0., 0.], "GRID"));
        m.functions.insert(
            "STEP".into(),
            DynamicFunction {
                name: "STEP".into(),
                is_time_history: true,
                damping_ratio: 0.,
                points: vec![(0., 1.), (10., 1.)],
            },
        );
        let c = &mut m.load_cases["S"];
        c.case_type = LoadCaseType::LinearDynamic;
        c.time_integration = method;
        c.time_step_dt = 0.005;
        c.num_time_steps = 400;
        c.history_loads = vec![("STEP".into(), "DEAD".into(), 1.)];
        analyze_model(&mut m).unwrap();
        let r = &m.case_results["S"];
        let wd = 3.96f64.sqrt();
        for (t, u) in r.time_steps.iter().zip(&r.time_history_disp["I"]) {
            let exact =
                3. / 8. * (1. - (-0.2 * t).exp() * ((wd * t).cos() + 0.2 / wd * (wd * t).sin()));
            assert!(
                (u[0] - exact).abs() < 5e-5,
                "{method:?} t={t} {} vs {exact}",
                u[0]
            );
        }
    }
}

#[test]
fn invalid_link_reference_and_massless_dashpot_fail_without_results() {
    let mut m = link_model(true);
    m.links["L"].prop = "MISSING".into();
    assert!(analyze_model(&mut m)
        .unwrap_err()
        .to_string()
        .contains("missing property"));
    let mut m = link_model(true);
    m.points["I"].mass = [0.; 6];
    m.link_props["P"].ce = [1.; 6];
    let c = &mut m.load_cases["S"];
    c.case_type = LoadCaseType::LinearDynamic;
    assert!(analyze_model(&mut m)
        .unwrap_err()
        .to_string()
        .contains("DAE"));
    assert!(m.case_results.is_empty());
}

#[test]
fn three_angle_named_basis_is_right_handed_and_translates_no_moment() {
    let mut m = model();
    grid(&mut m, 90.);
    m.coord_systems["GRID"].angles_deg = [90., 90., 0.];
    m.add_point("A", 0., 0., 0.);
    m.points["A"].restraint = [true; 6];
    m.points["A"]
        .loads_force
        .push(force([1., 2., 3., 4., 5., 6.], "GRID"));
    analyze_model(&mut m).unwrap();
    for (a, b) in m.case_results["S"]
        .base_reactions
        .iter()
        .zip([2., -3., 1., 5., -6., 4.])
    {
        close(*a, b);
    }
    m.coord_systems["GRID"].angles_deg[1] = f64::NAN;
    assert!(analyze_model(&mut m)
        .unwrap_err()
        .to_string()
        .contains("nonfinite"));
    assert!(m.case_results.is_empty());
}

#[test]
fn finite_link_rigid_body_rotation_produces_no_strain_or_force() {
    let mut m = link_model(false);
    m.link_props["P"].dj2 = 0.7;
    m.link_props["P"].dj3 = 1.2;
    for (name, u) in [
        ("I", [0.2, 0.3, 0.4, 0.1, 0.2, 0.3]),
        ("J", [0.2, 1.2, -0.2, 0.1, 0.2, 0.3]),
    ] {
        m.points[name].restraint = [true; 6];
        m.points[name].loads_displ.push(PointLoadDispl {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            displ: u,
        });
    }
    analyze_model(&mut m).unwrap();
    for f in m.case_results["S"].link_forces["L"] {
        close(f, 0.);
    }
    for f in m.case_results["S"].base_reactions {
        close(f, 0.);
    }
}

#[test]
fn rotated_nonproportional_link_damping_matches_two_independent_modal_responses() {
    let mut m = link_model(true);
    m.active_dofs = [true, true, false, false, false, false];
    m.links["L"].angle_deg = 45.;
    m.link_props["P"].ke[1] = 8.;
    m.link_props["P"].ke[2] = 18.;
    m.link_props["P"].ce[1] = 0.8;
    m.link_props["P"].ce[2] = 1.2;
    m.points["I"]
        .loads_force
        .push(force([3., 0., 0., 0., 0., 0.], "GLOBAL"));
    m.functions.insert(
        "STEP".into(),
        DynamicFunction {
            name: "STEP".into(),
            is_time_history: true,
            damping_ratio: 0.,
            points: vec![(0., 1.), (10., 1.)],
        },
    );
    let c = &mut m.load_cases["S"];
    c.case_type = LoadCaseType::LinearDynamic;
    c.time_step_dt = 0.002;
    c.num_time_steps = 1000;
    c.history_loads = vec![("STEP".into(), "DEAD".into(), 1.)];
    analyze_model(&mut m).unwrap();
    for (t, u) in m.case_results["S"]
        .time_steps
        .iter()
        .zip(&m.case_results["S"].time_history_disp["I"])
    {
        let response = |k: f64, c: f64| {
            let a = c / 4.;
            let w = (k / 2. - a * a).sqrt();
            3. / (2f64.sqrt() * k) * (1. - (-a * t).exp() * ((w * t).cos() + a / w * (w * t).sin()))
        };
        let a = response(8., 0.8);
        let b = -response(18., 1.2);
        assert!((u[0] - (a - b) / 2f64.sqrt()).abs() < 1e-5);
        assert!((u[1] - (a + b) / 2f64.sqrt()).abs() < 1e-5);
    }
}

#[test]
fn unsupported_pressure_and_nonlinear_constitutive_inputs_are_not_silently_linearized() {
    for (face, pattern) in [(1, None), (-1, Some("VARYING".into()))] {
        let mut m = model();
        for (name, x, y) in [("A", 0., 0.), ("B", 1., 0.), ("C", 0., 1.)] {
            m.add_point(name, x, y, 0.);
            m.points[name].restraint = [true; 6];
        }
        m.add_area("AREA", vec!["A".into(), "B".into(), "C".into()], "ASEC1");
        m.areas["AREA"]
            .loads_pressure
            .push(AreaLoadSurfacePressure {
                load_pat: "DEAD".into(),
                face,
                val: 1.,
                pattern_name: pattern,
            });
        assert!(analyze_model(&mut m)
            .unwrap_err()
            .to_string()
            .contains("pressure"));
    }
    let mut m = link_model(true);
    m.link_props["P"].link_type = LinkPropType::Gap;
    // Effective stiffness remains valid for an unstressed linear case.
    analyze_model(&mut m).unwrap();
    m.load_cases["S"].case_type = LoadCaseType::NonlinearStatic;
    // Plastic Wen is implemented (plastic_wen_regressions.rs); every other
    // family beyond Gap/Hook/Damper must still be refused.
    for kind in [
        LinkPropType::MultiLinearElastic,
        LinkPropType::MultiLinearPlastic,
        LinkPropType::RubberIsolator,
        LinkPropType::FrictionIsolator,
        LinkPropType::TCFrictionIsolator,
        LinkPropType::TriplePendulumIsolator,
    ] {
        m.link_props["P"].link_type = kind;
        assert!(analyze_model(&mut m)
            .unwrap_err()
            .to_string()
            .contains("constitutive iteration"));
        assert!(m.case_results.is_empty());
    }
}

#[test]
fn ramp_history_support_reaction_contains_viscous_force_and_direct_support_load() {
    for beta in [0., 0.1] {
        let mut m = link_model(false);
        m.active_dofs = [true, false, false, false, false, false];
        m.link_props["P"].ke[0] = 8.;
        m.link_props["P"].ce[0] = 0.8;
        m.points["J"]
            .loads_force
            .push(force([3., 0., 0., 0., 0., 0.], "GLOBAL"));
        m.points["I"]
            .loads_force
            .push(force([2., 0., 0., 0., 0., 0.], "GLOBAL"));
        m.functions.insert(
            "RAMP".into(),
            DynamicFunction {
                name: "RAMP".into(),
                is_time_history: true,
                damping_ratio: 0.,
                points: vec![(0., 0.), (10., 10.)],
            },
        );
        let c = &mut m.load_cases["S"];
        c.case_type = LoadCaseType::LinearDynamic;
        c.time_step_dt = 0.001;
        c.num_time_steps = 1000;
        c.rayleigh_damping = (0., beta);
        c.history_loads = vec![("RAMP".into(), "DEAD".into(), 1.)];
        analyze_model(&mut m).unwrap();
        let r = &m.case_results["S"];
        // The displacement increases throughout [0,1], so the summary is at t=1.
        close(
            r.joint_displacements["J"][0],
            r.time_history_disp["J"].last().unwrap()[0],
        );
        let u = r.joint_displacements["J"][0];
        let elastic = 8. * u;
        let link = r.link_forces["L"][0];
        let speed = (link - elastic) / 0.8;
        close(r.joint_reactions["I"][0], -link - beta * 8. * speed - 2.);
        close(r.base_reactions[0], r.joint_reactions["I"][0]);
        // Independent closed-form acceleration for F(t)=3t, m=2, k=8.
        let c = 0.8 + beta * 8.;
        let decay = c / 4.;
        let wd = (4. - decay * decay).sqrt();
        let acceleration = 3. / (2. * wd) * (-decay).exp() * wd.sin();
        assert!((r.base_reactions[0] + 5. - 2. * acceleration).abs() < 2e-6);
    }
}

#[test]
fn projected_moments_use_cosine_and_projected_forces_use_sine_before_global_split() {
    for named in [false, true] {
        for reverse in [false, true] {
            for kind in [1, 2] {
                let mut a = model();
                grid(&mut a, 30.);
                let (x, y) = if named {
                    (4., 0.)
                } else {
                    (4. * 30f64.to_radians().cos(), 4. * 30f64.to_radians().sin())
                };
                a.add_point("I", 0., 0., 0.);
                a.add_point("J", x, y, 0.);
                a.points["I"].restraint = [true; 6];
                a.add_frame(
                    "F",
                    if reverse { "J" } else { "I" },
                    if reverse { "I" } else { "J" },
                    "FSEC1",
                );
                a.frames["F"].loads_dist.push(FrameLoadDistributed {
                    load_pat: "DEAD".into(),
                    csys: if named { "GRID" } else { "GLOBAL" }.into(),
                    my_type: kind,
                    dir: 7,
                    rel_dist1: 0.,
                    rel_dist2: 1.,
                    val1: 2.,
                    val2: 2.,
                });
                let mut b = a.clone();
                b.frames["F"].loads_dist.clear();
                let projection = if kind == 1 {
                    0.5
                } else {
                    30f64.to_radians().cos()
                };
                for (dir, component) in if named {
                    vec![(4, 30f64.to_radians().cos()), (5, 0.5)]
                } else {
                    vec![(4, 1.)]
                } {
                    b.frames["F"].loads_dist.push(FrameLoadDistributed {
                        load_pat: "DEAD".into(),
                        csys: "GLOBAL".into(),
                        my_type: kind,
                        dir,
                        rel_dist1: 0.,
                        rel_dist2: 1.,
                        val1: 2. * projection * component,
                        val2: 2. * projection * component,
                    });
                }
                analyze_model(&mut a).unwrap();
                analyze_model(&mut b).unwrap();
                compare(&a, &b);
                if kind == 2 {
                    close(
                        a.case_results["S"].base_reactions[3],
                        -8. * projection * if named { 30f64.to_radians().cos() } else { 1. },
                    );
                } else {
                    close(
                        a.case_results["S"].base_reactions[0],
                        -8. * projection * if named { 30f64.to_radians().cos() } else { 1. },
                    );
                }
            }
        }
    }
}

#[test]
fn grounded_viscous_link_history_reports_support_resultant_with_no_joint_restraint() {
    let mut m = link_model(true);
    m.active_dofs = [true, false, false, false, false, false];
    m.link_props["P"].ke[1] = 8.;
    m.link_props["P"].ce[1] = 0.8;
    m.points["I"].y = 3.;
    m.points["I"]
        .loads_force
        .push(force([3., 0., 0., 0., 0., 0.], "GLOBAL"));
    m.functions.insert(
        "RAMP".into(),
        DynamicFunction {
            name: "RAMP".into(),
            is_time_history: true,
            damping_ratio: 0.,
            points: vec![(0., 0.), (10., 10.)],
        },
    );
    let c = &mut m.load_cases["S"];
    c.case_type = LoadCaseType::LinearDynamic;
    c.time_step_dt = 0.001;
    c.num_time_steps = 1000;
    c.history_loads = vec![("RAMP".into(), "DEAD".into(), 1.)];
    c.rayleigh_damping = (0., 0.1);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["S"];
    let link = r.link_forces["L"][1];
    let speed = (link - 8. * r.joint_displacements["I"][0]) / 0.8;
    close(r.joint_reactions["I"][0], -link - 0.8 * speed);
    close(r.base_reactions[0], -link - 0.8 * speed);
    close(r.base_reactions[5], -3. * r.base_reactions[0]);
}
