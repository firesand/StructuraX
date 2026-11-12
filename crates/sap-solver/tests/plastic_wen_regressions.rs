//! Plastic Wen link and displacement-controlled nonlinear static regressions.
//!
//! Constitutive reference: CSI Analysis Reference Manual (July 2016), "Wen
//! Plasticity Property": `f = ratio k d + (1 - ratio) yield z`,
//! `dz/dd = (k/yield)(1 - |z|^exp)` while `dd z > 0`, `k/yield` otherwise.
//! Published values: CSI Software Verification Example 6-008, "Link - Plastic
//! Wen Link" (k = 100 kip/in, yield = 50 kip, ratio = 0.1, exp = 1; kip, in).
//! The models below are independent single- and three-joint models in those
//! units; they are not the vendor file, whose link gravity load and advanced
//! link axes are not imported.
use sap_model::*;
use sap_solver::analyze_model;

const K: f64 = 100.0;
const YIELD: f64 = 50.0;
const RATIO: f64 = 0.1;

fn close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance * (1.0 + expected.abs()),
        "{actual} != {expected} (relative tolerance {tolerance})"
    );
}

/// One axial Wen link from fixed joint I to joint J, loaded by a unit force at J.
fn sdof(exponent: f64) -> SapModel {
    let mut m = SapModel::default();
    m.load_patterns["DEAD"].self_wt_multiplier = 0.;
    m.load_cases.clear();
    m.active_dofs = [true, false, false, false, false, false];
    m.add_point("I", 0., 0., 0.);
    m.add_point("J", 3., 0., 0.);
    m.points["I"].restraint = [true; 6];
    m.points["J"].loads_force.push(PointLoadForce {
        load_pat: "DEAD".into(),
        csys: "GLOBAL".into(),
        force: [1., 0., 0., 0., 0., 0.],
    });
    let mut prop = LinkProp {
        name: "W".into(),
        link_type: LinkPropType::PlasticWen,
        ..LinkProp::default()
    };
    prop.dof_nonlinear[0] = true;
    // The effective stiffness must not influence a nonlinear case.
    prop.ke[0] = 13.;
    prop.k_nl[0] = K;
    prop.param1[0] = YIELD;
    prop.param2[0] = RATIO;
    prop.param3[0] = exponent;
    m.link_props.insert("W".into(), prop);
    m.links.insert(
        "L".into(),
        LinkObj {
            name: "L".into(),
            point_i: "I".into(),
            point_j: Some("J".into()),
            prop: "W".into(),
            angle_deg: 0.,
            selected: false,
        },
    );
    m
}

fn static_case(m: &mut SapModel, name: &str, scale: f64, initial: Option<&str>) {
    let mut case = LoadCase::linear_static(name, "DEAD", scale);
    case.case_type = LoadCaseType::NonlinearStatic;
    case.convergence_tol = 1e-12;
    case.max_iterations = 200;
    case.initial_case = initial.map(str::to_owned);
    m.load_cases.insert(name.into(), case);
}

fn push_to(m: &mut SapModel, name: &str, target: f64) {
    m.load_cases[name].displacement_control = Some(DisplacementControl {
        target,
        monitored: Some(("J".into(), 0)),
        conjugate: false,
    });
}

/// Independent fourth-order integration of the Wen state along a deformation
/// path made of straight segments, with 200 000 steps per segment.
fn reference_z(exponent: f64, path: &[f64]) -> f64 {
    let c = K / YIELD;
    let mut z = 0.0f64;
    for pair in path.windows(2) {
        let n = 200_000;
        let h = (pair[1] - pair[0]) / n as f64;
        let rate = |z: f64| {
            if h * z > 0. {
                c * (1. - z.abs().powf(exponent))
            } else {
                c
            }
        };
        for _ in 0..n {
            let k1 = rate(z);
            let k2 = rate(z + 0.5 * h * k1);
            let k3 = rate(z + 0.5 * h * k2);
            let k4 = rate(z + h * k3);
            z += h / 6. * (k1 + 2. * k2 + 2. * k3 + k4);
        }
    }
    z
}

fn wen_force(d: f64, z: f64) -> f64 {
    RATIO * K * d + (1. - RATIO) * YIELD * z
}

#[test]
fn example_6_008_published_link_forces_under_displacement_control() {
    // NLSTAT1: push from rest. Published link force at each deformation.
    for (deformation, published) in [(0.2, 16.836), (0.8, 43.915), (9.6, 141.000)] {
        let mut m = sdof(1.);
        static_case(&mut m, "NLSTAT1", 1., None);
        push_to(&mut m, "NLSTAT1", deformation);
        analyze_model(&mut m).unwrap();
        let r = &m.case_results["NLSTAT1"];
        close(r.joint_displacements["J"][0], deformation, 1e-10);
        // Published to three decimals.
        assert!((r.link_forces["L"][0] - published).abs() < 5e-4);
        let exact = wen_force(deformation, 1. - (-K / YIELD * deformation).exp());
        close(r.link_forces["L"][0], exact, 1e-10);
        // Unit load pattern: the reached load factor is the link force.
        close(r.displacement_control_factor.unwrap(), exact, 1e-10);
        close(r.joint_reactions["I"][0], -exact, 1e-10);
        close(r.base_reactions[0], -exact, 1e-10);
    }

    // NLSTAT2 continues from +10 in and pushes back; the target is the
    // distance moved from the start of the case.
    for (moved, deformation, published) in
        [(0.4, 9.6, 105.000), (1.4, 8.6, 48.438), (19.6, -9.6, -141.000)]
    {
        let mut m = sdof(1.);
        static_case(&mut m, "NLSTAT1", 1., None);
        push_to(&mut m, "NLSTAT1", 10.);
        static_case(&mut m, "NLSTAT2", -1., Some("NLSTAT1"));
        push_to(&mut m, "NLSTAT2", moved);
        analyze_model(&mut m).unwrap();
        let r = &m.case_results["NLSTAT2"];
        close(r.joint_displacements["J"][0], deformation, 1e-9);
        assert!(
            (r.link_forces["L"][0] - published).abs() < 5e-4,
            "{} != {published}",
            r.link_forces["L"][0]
        );
        close(r.base_reactions[0], -r.link_forces["L"][0], 1e-9);
        // The inherited load is +145 kip (factor of NLSTAT1); NLSTAT2 removes
        // `factor` kip of it.
        let inherited = m.case_results["NLSTAT1"].displacement_control_factor.unwrap();
        close(
            inherited - r.displacement_control_factor.unwrap(),
            r.link_forces["L"][0],
            1e-9,
        );
        let z = m.case_results["NLSTAT2"].link_hysteresis[0].z;
        close(z, reference_z(1., &[0., 10., deformation]), 1e-8);
    }
}

#[test]
fn load_control_inverts_the_closed_form_law_and_unloads_elastically() {
    for exponent in [1., 2.] {
        let virgin = |d: f64| {
            let x = K / YIELD * d;
            let z = if exponent == 1. { 1. - (-x).exp() } else { x.tanh() };
            wen_force(d, z)
        };
        for deformation in [0.05, 0.5, 2., 7.] {
            let mut m = sdof(exponent);
            static_case(&mut m, "PUSH", virgin(deformation), None);
            // Remove 30 kip: |dz| = (k/yield) |dd| < z, so the unloading is elastic
            // with the initial stiffness k.
            static_case(&mut m, "BACK", -30., Some("PUSH"));
            analyze_model(&mut m).unwrap();
            let push = &m.case_results["PUSH"];
            close(push.joint_displacements["J"][0], deformation, 1e-9);
            close(push.link_forces["L"][0], virgin(deformation), 1e-10);
            assert!(push.displacement_control_factor.is_none());
            let back = &m.case_results["BACK"];
            if deformation >= 0.5 {
                close(back.joint_displacements["J"][0], deformation - 30. / K, 1e-9);
            }
            close(back.link_forces["L"][0], virgin(deformation) - 30., 1e-9);
        }
    }
}

#[test]
fn general_exponents_match_an_independent_integration_of_the_state_equation() {
    for exponent in [1., 1.5, 2., 3., 7.25, 20.] {
        for path in [vec![0.0f64, 0.3], vec![0., 4., 3.2], vec![0., 1.2, -2.5]] {
            let mut m = sdof(exponent);
            let mut previous: Option<String> = None;
            for (index, pair) in path.windows(2).enumerate() {
                let name = format!("S{index}");
                static_case(
                    &mut m,
                    &name,
                    (pair[1] - pair[0]).signum(),
                    previous.as_deref(),
                );
                push_to(&mut m, &name, (pair[1] - pair[0]).abs());
                previous = Some(name);
            }
            analyze_model(&mut m).unwrap();
            let last = &m.case_results[previous.as_ref().unwrap()];
            let end = *path.last().unwrap();
            let z = reference_z(exponent, &path);
            close(last.joint_displacements["J"][0], end, 1e-9);
            close(last.link_hysteresis[0].z, z, 2e-8);
            close(last.link_forces["L"][0], wen_force(end, z), 2e-8);
        }
    }
}

#[test]
fn splitting_a_monotonic_push_into_cases_does_not_change_the_state() {
    for exponent in [1., 2., 3.5] {
        let mut whole = sdof(exponent);
        static_case(&mut whole, "A", 1., None);
        push_to(&mut whole, "A", 6.);
        analyze_model(&mut whole).unwrap();

        let mut split = sdof(exponent);
        static_case(&mut split, "A", 1., None);
        push_to(&mut split, "A", 0.37);
        static_case(&mut split, "B", 1., Some("A"));
        push_to(&mut split, "B", 2.13);
        static_case(&mut split, "C", 1., Some("B"));
        push_to(&mut split, "C", 3.5);
        analyze_model(&mut split).unwrap();

        let (a, c) = (&whole.case_results["A"], &split.case_results["C"]);
        close(c.joint_displacements["J"][0], 6., 1e-9);
        close(c.link_forces["L"][0], a.link_forces["L"][0], 1e-9);
        close(c.link_hysteresis[0].z, a.link_hysteresis[0].z, 1e-9);
    }
}

#[test]
fn linear_cases_use_the_effective_stiffness_of_a_wen_link() {
    let mut m = sdof(1.);
    m.load_cases
        .insert("LIN".into(), LoadCase::linear_static("LIN", "DEAD", 26.));
    analyze_model(&mut m).unwrap();
    close(m.case_results["LIN"].joint_displacements["J"][0], 2., 1e-12);
    assert!(m.case_results["LIN"].link_hysteresis.is_empty());
}

#[test]
fn displacement_control_scales_linear_and_contact_models() {
    // Linear: grounded spring of 50 kip/in, unit load, target 2 in -> factor 100.
    let mut m = sdof(1.);
    m.link_props["W"].dof_nonlinear[0] = false;
    m.link_props["W"].ke[0] = 50.;
    static_case(&mut m, "PUSH", 1., None);
    push_to(&mut m, "PUSH", 2.);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["PUSH"];
    close(r.joint_displacements["J"][0], 2., 1e-10);
    close(r.displacement_control_factor.unwrap(), 100., 1e-10);
    close(r.link_forces["L"][0], 100., 1e-10);

    // A following load-controlled case adds its own load to the reached 100 kip.
    static_case(&mut m, "MORE", 25., Some("PUSH"));
    analyze_model(&mut m).unwrap();
    close(m.case_results["MORE"].joint_displacements["J"][0], 2.5, 1e-10);

    // Hook with a 0.2 in opening in parallel with a 50 kip/in spring:
    // u = 1 in needs 50*1 + 100*(1 - 0.2) = 130 kip.
    let mut m = sdof(1.);
    m.link_props["W"].link_type = LinkPropType::Hook;
    m.link_props["W"].param1[0] = 0.2;
    m.points["J"].spring[0] = 50.;
    static_case(&mut m, "PUSH", 1., None);
    push_to(&mut m, "PUSH", 1.);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["PUSH"];
    close(r.joint_displacements["J"][0], 1., 1e-10);
    close(r.displacement_control_factor.unwrap(), 130., 1e-9);
    close(r.link_forces["L"][0], 80., 1e-9);
}

#[test]
fn displacement_control_approaches_a_perfectly_plastic_capacity_from_below() {
    // ratio = 0: f = 50 (1 - exp(-2 d)) never exceeds 50 kip, so larger load
    // factors have no equilibrium. Reaching d = 2 in needs 50 (1 - exp(-4)) kip.
    let mut m = sdof(1.);
    m.link_props["W"].param2[0] = 0.;
    static_case(&mut m, "PUSH", 1., None);
    push_to(&mut m, "PUSH", 2.);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["PUSH"];
    let exact = YIELD * (1. - (-4.0f64).exp());
    close(r.joint_displacements["J"][0], 2., 1e-8);
    close(r.displacement_control_factor.unwrap(), exact, 1e-9);
    close(r.link_forces["L"][0], exact, 1e-9);

    // With a 100 kip unit pattern even the first trial factor has no equilibrium.
    let mut m = sdof(1.);
    m.link_props["W"].param2[0] = 0.;
    m.points["J"].loads_force[0].force[0] = 100.;
    static_case(&mut m, "PUSH", 1., None);
    push_to(&mut m, "PUSH", 2.);
    assert!(analyze_model(&mut m).is_err());
    assert!(m.case_results.is_empty());
}

/// Joint A is pushed against a stiff hook while joint B keeps moving, so the
/// Wen deformation `u_A - u_B` first grows and then shrinks within one case.
fn reversing_model() -> SapModel {
    let mut m = sdof(1.);
    m.points.clear();
    m.links.clear();
    m.add_point("G", 0., 0., 0.);
    m.add_point("A", 1., 0., 0.);
    m.add_point("B", 2., 0., 0.);
    m.points["G"].restraint = [true; 6];
    m.points["B"].spring[0] = 100.;
    for (point, force) in [("A", 20.), ("B", 200.)] {
        m.points[point].loads_force.push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: [force, 0., 0., 0., 0., 0.],
        });
    }
    let mut hook = LinkProp {
        name: "H".into(),
        link_type: LinkPropType::Hook,
        ..LinkProp::default()
    };
    hook.dof_nonlinear[0] = true;
    hook.k_nl[0] = 1e4;
    hook.param1[0] = 0.5;
    m.link_props.insert("H".into(), hook);
    for (name, i, j, prop) in [("L", "B", "A", "W"), ("S", "G", "A", "H")] {
        m.links.insert(
            name.into(),
            LinkObj {
                name: name.into(),
                point_i: i.into(),
                point_j: Some(j.into()),
                prop: prop.into(),
                angle_deg: 0.,
                selected: false,
            },
        );
    }
    m
}

#[test]
fn a_wen_deformation_reversal_inside_one_case_is_rejected() {
    let mut m = reversing_model();
    static_case(&mut m, "ALL", 1., None);
    let error = analyze_model(&mut m).unwrap_err().to_string();
    assert!(error.contains("reverses within the load case"), "{error}");
    assert!(m.case_results.is_empty());

    // Before the hook engages (u_A = 0.5 in at about 21% of the load) the
    // deformation is monotonic and statically determinate: |f_link| = 20 * 0.1.
    // The link runs from B to A, against +X, so the push on A compresses it.
    let mut m = reversing_model();
    static_case(&mut m, "PART", 0.1, None);
    analyze_model(&mut m).unwrap();
    close(m.case_results["PART"].link_forces["L"][0], -2., 1e-9);
}

#[test]
fn invalid_or_unsupported_wen_and_control_inputs_are_rejected() {
    let rejected = |m: &mut SapModel, text: &str| {
        let error = analyze_model(m).unwrap_err().to_string();
        assert!(error.contains(text), "{error}");
        assert!(m.case_results.is_empty());
    };
    for (edit, text) in [
        (
            (|p: &mut LinkProp| p.param2[0] = 1.5) as fn(&mut LinkProp),
            "post-yield ratio",
        ),
        (|p: &mut LinkProp| p.param1[0] = 0., "yield > 0"),
        (|p: &mut LinkProp| p.param3[0] = 0.5, "yield exponent"),
        (|p: &mut LinkProp| p.param3[0] = 25., "yield exponent"),
        (|p: &mut LinkProp| p.k_nl[0] = f64::NAN, "stiffness > 0"),
    ] {
        let mut m = sdof(1.);
        static_case(&mut m, "S", 1., None);
        edit(&mut m.link_props["W"]);
        rejected(&mut m, text);
    }

    let mut m = sdof(1.);
    static_case(&mut m, "S", 1., None);
    m.load_cases["S"].displacement_control = Some(DisplacementControl {
        target: 1.,
        monitored: None,
        conjugate: true,
    });
    rejected(&mut m, "representable monitored joint displacement");
    push_to(&mut m, "S", -1.);
    rejected(&mut m, "finite and positive");
    push_to(&mut m, "S", 1.);
    m.load_cases["S"].displacement_control.as_mut().unwrap().monitored = Some(("I".into(), 0));
    rejected(&mut m, "restrained or inactive");
    m.load_cases["S"].displacement_control.as_mut().unwrap().monitored = Some(("Q".into(), 0));
    rejected(&mut m, "missing joint");
    push_to(&mut m, "S", 1.);
    m.load_cases["S"].case_type = LoadCaseType::LinearStatic;
    rejected(&mut m, "only to nonlinear static");

    // A load that cannot move the monitored displacement.
    let mut m = sdof(1.);
    m.points["J"].loads_force.clear();
    static_case(&mut m, "S", 1., None);
    push_to(&mut m, "S", 1.);
    rejected(&mut m, "does not move the monitored displacement");

    // Perfectly plastic link: 60 kip exceeds the 50 kip yield plateau.
    let mut m = sdof(1.);
    m.link_props["W"].param2[0] = 0.;
    static_case(&mut m, "S", 60., None);
    assert!(analyze_model(&mut m).is_err());
    assert!(m.case_results.is_empty());
}

fn history_case(m: &mut SapModel, name: &str, steps: usize, dt: f64) {
    let mut case = LoadCase::linear_static(name, "DEAD", 1.);
    case.loads.clear();
    case.case_type = LoadCaseType::NonlinearDynamic;
    // Some tests retype this case to the modal history type, which takes no
    // modal damping.
    case.modal_damping = 0.;
    case.history_loads = vec![("F".into(), "DEAD".into(), 1.)];
    case.time_step_dt = dt;
    case.num_time_steps = steps;
    case.convergence_tol = 1e-12;
    case.max_iterations = 200;
    m.load_cases.insert(name.into(), case);
}

/// Independent RK4 integration of `m u'' + f(u, z) = p(t)` with the Wen state
/// equation, for a piecewise-linear `p(t)`.
fn reference_history(mass: f64, exponent: f64, load: &[(f64, f64)], t_end: f64) -> (f64, f64, f64) {
    let p = |t: f64| {
        let i = load.windows(2).position(|w| t <= w[1].0).unwrap_or(load.len() - 2);
        let (a, b) = (load[i], load[i + 1]);
        a.1 + (b.1 - a.1) * (t - a.0) / (b.0 - a.0)
    };
    let c = K / YIELD;
    let rhs = |t: f64, y: [f64; 3]| -> [f64; 3] {
        let (u, v, z) = (y[0], y[1], y[2]);
        let dz = if v * z > 0. {
            c * v * (1. - z.abs().powf(exponent))
        } else {
            c * v
        };
        [v, (p(t) - wen_force(u, z)) / mass, dz]
    };
    let n = 400_000usize;
    let h = t_end / n as f64;
    let mut y = [0.0f64; 3];
    for step in 0..n {
        let t = step as f64 * h;
        let add = |y: [f64; 3], k: [f64; 3], s: f64| [y[0] + s * k[0], y[1] + s * k[1], y[2] + s * k[2]];
        let k1 = rhs(t, y);
        let k2 = rhs(t + 0.5 * h, add(y, k1, 0.5 * h));
        let k3 = rhs(t + 0.5 * h, add(y, k2, 0.5 * h));
        let k4 = rhs(t + h, add(y, k3, h));
        for i in 0..3 {
            y[i] += h / 6. * (k1[i] + 2. * k2[i] + 2. * k3[i] + k4[i]);
        }
    }
    (y[0], y[2], wen_force(y[0], y[2]))
}

#[test]
fn direct_integration_history_converges_to_an_independent_wen_oscillator_solution() {
    // 2 kip-s2/in on the Wen link: elastic period 2 pi sqrt(2/100) = 0.889 s.
    // The load rises to 80 kip (above the 50 kip yield), reverses to -60 kip
    // and returns to zero, so the response yields in both directions.
    let load = [(0., 0.), (0.5, 80.), (1.2, -60.), (1.6, 0.), (2., 0.)];
    // Every accepted integrator and both nonlinear history case types. HHT
    // with alpha = -0.05 adds algorithmic damping but stays second order.
    for (exponent, case_type, integration, alpha) in [
        (1., LoadCaseType::NonlinearDynamic, TimeIntegrationMethod::NewmarkAverage, 0.),
        (2., LoadCaseType::NonlinearDynamic, TimeIntegrationMethod::NewmarkAverage, 0.),
        (2., LoadCaseType::NonlinearHistory, TimeIntegrationMethod::NewmarkAverage, 0.),
        (1., LoadCaseType::NonlinearDynamic, TimeIntegrationMethod::NewmarkLinear, 0.),
        (2., LoadCaseType::NonlinearHistory, TimeIntegrationMethod::NewmarkLinear, 0.),
        (1., LoadCaseType::NonlinearHistory, TimeIntegrationMethod::HilberHughesTaylor, -0.05),
        (2., LoadCaseType::NonlinearDynamic, TimeIntegrationMethod::HilberHughesTaylor, -0.05),
    ] {
        let (u_ref, z_ref, f_ref) = reference_history(2., exponent, &load, 2.);
        let mut errors = Vec::new();
        for steps in [1_000usize, 2_000, 4_000] {
            let mut m = sdof(exponent);
            m.points["J"].mass[0] = 2.;
            m.functions.insert(
                "F".into(),
                DynamicFunction {
                    name: "F".into(),
                    is_time_history: true,
                    damping_ratio: 0.,
                    points: load.to_vec(),
                },
            );
            history_case(&mut m, "H", steps, 2. / steps as f64);
            m.load_cases["H"].case_type = case_type;
            m.load_cases["H"].time_integration = integration;
            m.load_cases["H"].hht_alpha = alpha;
            analyze_model(&mut m).unwrap();
            let r = &m.case_results["H"];
            let u_end = r.time_history_disp["J"].last().unwrap()[0];
            let state = &r.link_hysteresis[0];
            close(state.deformation, u_end, 1e-12);
            errors.push((u_end - u_ref).abs());
            if steps == 4_000 {
                // Time-integration error at dt = 0.5 ms.
                let label = format!("{integration:?} {case_type:?} exp {exponent}");
                assert!(
                    (u_end - u_ref).abs() < 2e-5 * u_ref.abs().max(1.),
                    "{label}: {u_end} {u_ref}"
                );
                assert!((state.z - z_ref).abs() < 2e-5, "{label}: {} {z_ref}", state.z);
                let f_end = wen_force(state.deformation, state.z);
                assert!((f_end - f_ref).abs() < 2e-5 * f_ref.abs().max(1.), "{label}");
            }
        }
        // Second-order time integration: a fourfold smaller step cuts the error
        // about sixteenfold; require at least eightfold.
        assert!(
            errors[2] < errors[0] / 8.,
            "{integration:?} {case_type:?}: no second-order convergence: {errors:?}"
        );
    }
}

#[test]
fn hysteretic_state_continues_between_static_and_history_cases() {
    // Push statically past yield, then run a history that only holds the load.
    // The inherited state must be in equilibrium: no motion at all.
    let mut m = sdof(1.);
    m.points["J"].mass[0] = 2.;
    static_case(&mut m, "PUSH", 100., None);
    m.functions.insert(
        "F".into(),
        DynamicFunction {
            name: "F".into(),
            is_time_history: true,
            damping_ratio: 0.,
            points: vec![(0., 0.), (1., 0.)],
        },
    );
    history_case(&mut m, "HOLD", 200, 0.005);
    m.load_cases["HOLD"].initial_case = Some("PUSH".into());
    // Remove 30 kip statically after the history: elastic unloading by 0.3 in.
    static_case(&mut m, "BACK", -30., Some("HOLD"));
    analyze_model(&mut m).unwrap();
    let pushed = m.case_results["PUSH"].joint_displacements["J"][0];
    let z = m.case_results["PUSH"].link_hysteresis[0].z;
    close(wen_force(pushed, z), 100., 1e-10);
    for step in &m.case_results["HOLD"].time_history_disp["J"] {
        close(step[0], pushed, 1e-9);
    }
    close(m.case_results["HOLD"].link_hysteresis[0].z, z, 1e-9);
    close(
        m.case_results["BACK"].joint_displacements["J"][0],
        pushed - 0.3,
        1e-8,
    );
    close(m.case_results["BACK"].link_forces["L"][0], 70., 1e-8);

    // Wilson-theta integration of a Wen link is outside the verified subset.
    m.load_cases["HOLD"].time_integration = TimeIntegrationMethod::WilsonTheta;
    let error = analyze_model(&mut m).unwrap_err().to_string();
    assert!(error.contains("Wilson-theta integration of plastic Wen"), "{error}");
}

#[test]
fn unit_conversion_preserves_the_displacement_controlled_wen_response() {
    use sap_units::Units;
    // Example 6-008 values in kip-in: 141 kip at 9.6 in.
    let mut m = sdof(1.);
    assert_eq!(m.present_units, Units::KipInF);
    static_case(&mut m, "PUSH", 1., None);
    push_to(&mut m, "PUSH", 9.6);
    m.convert_units_in_place(Units::KnMC);
    // 1 in = 0.0254 m exactly; 1 kip = 4.4482216152605 kN.
    close(
        m.load_cases["PUSH"].displacement_control.as_ref().unwrap().target,
        9.6 * 0.0254,
        1e-12,
    );
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["PUSH"];
    close(r.joint_displacements["J"][0], 9.6 * 0.0254, 1e-9);
    close(r.link_forces["L"][0], 141.0 * 4.4482216152605, 1e-6);
}
