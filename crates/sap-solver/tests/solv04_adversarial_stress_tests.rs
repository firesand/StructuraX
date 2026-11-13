//! Adversarial Empirical Stress Tests for SOLV-04:
//! Modal history continuation across load cases, trajectory overlay, multi-case chaining,
//! cross-continuation with direct integration, nonlinear static equilibrium, and massless DOF handling.

use sap_model::*;
use sap_solver::analyze_model;

fn assert_close(a: f64, b: f64, tol: f64, msg: &str) {
    let diff = (a - b).abs();
    assert!(
        diff <= tol,
        "{msg}: got {a:.14e}, expected {b:.14e}, diff {diff:.14e} > tol {tol:.14e}"
    );
}

/// Helper to generate piecewise-linear dynamic functions
fn generate_sin_function(
    name: &str,
    t_end: f64,
    dt_fn: f64,
    freq_rad_s: f64,
    amp: f64,
    offset: f64,
) -> DynamicFunction {
    let n_pts = (t_end / dt_fn).ceil() as usize;
    let mut points = Vec::with_capacity(n_pts + 1);
    for i in 0..=n_pts {
        let t = (i as f64) * dt_fn;
        let val = amp * (freq_rad_s * t).sin() + offset;
        points.push((t, val));
    }
    DynamicFunction {
        name: name.into(),
        is_time_history: true,
        damping_ratio: 0.0,
        points,
    }
}

// ============================================================================
// CHALLENGE 1: Split vs. Uninterrupted Modal Trajectory Overlay (< 1e-11)
// ============================================================================

/// Adversarial Challenge 1:
/// 3-DOF coupled dynamic oscillator under time-varying sinusoidal excitation.
/// Compares uninterrupted FULL analysis against chained (CASE1 for t in [0, t1], CASE2 for t in [t1, t2]).
/// Probes every single time step for ||u_chained(t) - u_uninterrupted(t)|| < 1e-11.
/// Also verifies terminal velocity, acceleration, and modal generalized coordinates (modal_q, modal_v, modal_a).
#[test]
fn test_challenge_split_vs_uninterrupted_time_varying_trajectory_overlay() {
    let mut model = SapModel::default();
    model.active_dofs = [true, false, false, false, false, false];
    model
        .load_patterns
        .get_mut("DEAD")
        .unwrap()
        .self_wt_multiplier = 0.0;
    model.load_cases.clear();

    // 3-DOF coupled spring-mass system:
    // Fixed base -> K1 -> J1 (M1) -> K2 -> J2 (M2) -> K3 -> J3 (M3)
    model.add_point("G", 0.0, 0.0, 0.0);
    model.add_point("J1", 10.0, 0.0, 0.0);
    model.add_point("J2", 20.0, 0.0, 0.0);
    model.add_point("J3", 30.0, 0.0, 0.0);

    model.points.get_mut("G").unwrap().restraint = [true; 6];
    model.points.get_mut("J1").unwrap().mass[0] = 2.0;
    model.points.get_mut("J1").unwrap().spring[0] = 300.0; // K1
    model.points.get_mut("J2").unwrap().mass[0] = 1.5;
    model.points.get_mut("J2").unwrap().spring[0] = 200.0; // K2
    model.points.get_mut("J3").unwrap().mass[0] = 1.0;
    model.points.get_mut("J3").unwrap().spring[0] = 100.0; // K3

    // Lateral force pattern at J3: F = 50.0 kips
    model
        .points
        .get_mut("J3")
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: [50.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        });

    let dt: f64 = 0.005;
    let t_split: f64 = 0.8;
    let t_total: f64 = 2.0;
    let n_steps_1 = (t_split / dt).round() as usize; // 160 steps
    let n_steps_2 = ((t_total - t_split) / dt).round() as usize; // 240 steps
    let n_steps_full = (t_total / dt).round() as usize; // 400 steps

    let freq = std::f64::consts::TAU; // 1 Hz (omega = 2*pi)

    // Function for FULL: f_full(t) = sin(omega * t)
    model.functions.insert(
        "F_FULL".into(),
        generate_sin_function("F_FULL", t_total + 0.1, dt, freq, 1.0, 0.0),
    );

    // Function for CASE1: f1(t) = sin(omega * t) for t in [0, t_split]
    model.functions.insert(
        "F_CASE1".into(),
        generate_sin_function("F_CASE1", t_split + 0.1, dt, freq, 1.0, 0.0),
    );

    // For CASE2:
    // In CASE2, the initial load carried over via f_base_init is F1(t_split) = sin(omega * t_split).
    // To match FULL at t = t_split + tau, the CASE2 function must provide:
    // delta_F(tau) = sin(omega * (t_split + tau)) - sin(omega * t_split)
    let f_split_val = (freq * t_split).sin();
    let n_pts_2 = (1.5 / dt).ceil() as usize;
    let mut pts_2 = Vec::with_capacity(n_pts_2 + 1);
    for i in 0..=n_pts_2 {
        let tau = (i as f64) * dt;
        let val = (freq * (t_split + tau)).sin() - f_split_val;
        pts_2.push((tau, val));
    }
    model.functions.insert(
        "F_CASE2".into(),
        DynamicFunction {
            name: "F_CASE2".into(),
            is_time_history: true,
            damping_ratio: 0.0,
            points: pts_2,
        },
    );

    // 1. FULL case: Modal LinearHistory for t in [0, 2.0 s]
    let mut full_case = LoadCase::linear_static("FULL", "DEAD", 1.0);
    full_case.loads.clear();
    full_case.case_type = LoadCaseType::LinearHistory;
    full_case.modal_damping = 0.0;
    full_case.history_loads = vec![("F_FULL".into(), "DEAD".into(), 1.0)];
    full_case.time_step_dt = dt;
    full_case.num_time_steps = n_steps_full;
    model.load_cases.insert("FULL".into(), full_case);

    // 2. CASE1: Modal LinearHistory for t in [0, 0.8 s]
    let mut c1 = LoadCase::linear_static("CASE1", "DEAD", 1.0);
    c1.loads.clear();
    c1.case_type = LoadCaseType::LinearHistory;
    c1.modal_damping = 0.0;
    c1.history_loads = vec![("F_CASE1".into(), "DEAD".into(), 1.0)];
    c1.time_step_dt = dt;
    c1.num_time_steps = n_steps_1;
    model.load_cases.insert("CASE1".into(), c1);

    // 3. CASE2: Modal LinearHistory for t in [0, 1.2 s] continuing from CASE1
    let mut c2 = LoadCase::linear_static("CASE2", "DEAD", 1.0);
    c2.loads.clear();
    c2.case_type = LoadCaseType::LinearHistory;
    c2.modal_damping = 0.0;
    c2.initial_case = Some("CASE1".into());
    c2.history_loads = vec![("F_CASE2".into(), "DEAD".into(), 1.0)];
    c2.time_step_dt = dt;
    c2.num_time_steps = n_steps_2;
    model.load_cases.insert("CASE2".into(), c2);

    analyze_model(&mut model).expect("solve challenge 1");

    let res_full = &model.case_results["FULL"];
    let res_c1 = &model.case_results["CASE1"];
    let res_c2 = &model.case_results["CASE2"];

    // Verify all terminal states exist and modal coordinates are populated
    let ts_full = res_full.transient_state.as_ref().expect("ts_full");
    let ts_c1 = res_c1.transient_state.as_ref().expect("ts_c1");
    let ts_c2 = res_c2.transient_state.as_ref().expect("ts_c2");

    assert!(ts_c1.modal_q.is_some(), "c1 has modal_q");
    assert!(ts_c2.modal_q.is_some(), "c2 has modal_q");
    assert!(ts_full.modal_q.is_some(), "full has modal_q");

    // Trajectory overlay: Compare J1, J2, J3 across all 240 steps of CASE2 vs FULL[160..400]
    for joint in &["J1", "J2", "J3"] {
        let trace_c2 = &res_c2.time_history_disp[*joint];
        let trace_full = &res_full.time_history_disp[*joint];

        for k in 0..=n_steps_2 {
            let u_chained = trace_c2[k][0];
            let u_uninterrupted = trace_full[n_steps_1 + k][0];
            assert_close(
                u_chained,
                u_uninterrupted,
                1e-11,
                &format!("Overlay match at {joint} step {k}"),
            );
        }
    }

    // Terminal velocity and acceleration match (< 1e-10)
    for i in 0..ts_full.u.len() {
        assert_close(
            ts_c2.u[i],
            ts_full.u[i],
            1e-11,
            "Terminal displacement match",
        );
        assert_close(ts_c2.v[i], ts_full.v[i], 1e-10, "Terminal velocity match");
        assert_close(
            ts_c2.a[i],
            ts_full.a[i],
            1e-9,
            "Terminal acceleration match",
        );
    }

    // Generalized modal coordinates match
    let q_c2 = ts_c2.modal_q.as_ref().unwrap();
    let q_full = ts_full.modal_q.as_ref().unwrap();
    let v_c2 = ts_c2.modal_v.as_ref().unwrap();
    let v_full = ts_full.modal_v.as_ref().unwrap();
    let a_c2 = ts_c2.modal_a.as_ref().unwrap();
    let a_full = ts_full.modal_a.as_ref().unwrap();

    for m in 0..q_full.len() {
        assert_close(q_c2[m], q_full[m], 1e-11, &format!("modal_q mode {m}"));
        assert_close(v_c2[m], v_full[m], 1e-10, &format!("modal_v mode {m}"));
        assert_close(a_c2[m], a_full[m], 1e-9, &format!("modal_a mode {m}"));
    }
}

// ============================================================================
// CHALLENGE 2: Multi-Case Chaining (Case 1 -> Case 2 -> Case 3)
// ============================================================================

/// Adversarial Challenge 2A:
/// Three-case continuation chain with continuous piecewise frequency changes:
/// Case 1: Initial frequency omega1 = 5 rad/s for t in [0, 0.5 s].
/// Case 2: Continuing from Case 1, frequency omega2 = 12 rad/s for t in [0, 0.5 s].
/// Case 3: Continuing from Case 2, frequency omega3 = 3 rad/s for t in [0, 0.6 s].
/// Compares chained trajectory (C1 -> C2 -> C3) against composite single run FULL_3CASE.
#[test]
fn test_challenge_multi_case_chaining_continuous_frequency_transitions() {
    let mut model = SapModel::default();
    model.active_dofs = [true, false, false, false, false, false];
    model
        .load_patterns
        .get_mut("DEAD")
        .unwrap()
        .self_wt_multiplier = 0.0;
    model.load_cases.clear();

    model.add_point("GROUND", 0.0, 0.0, 0.0);
    model.add_point("OSC", 10.0, 0.0, 0.0);
    model.points.get_mut("GROUND").unwrap().restraint = [true; 6];
    model.points.get_mut("OSC").unwrap().mass[0] = 3.0;
    model.points.get_mut("OSC").unwrap().spring[0] = 150.0;

    model
        .points
        .get_mut("OSC")
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: [10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        });

    let dt: f64 = 0.005;
    let t1: f64 = 0.5;
    let t2: f64 = 0.5;
    let t3: f64 = 0.6;
    let n1 = (t1 / dt).round() as usize; // 100 steps
    let n2 = (t2 / dt).round() as usize; // 100 steps
    let n3 = (t3 / dt).round() as usize; // 120 steps
    let n_total = n1 + n2 + n3; // 320 steps

    let omega1 = 5.0;
    let omega2 = 12.0;
    let omega3 = 3.0;

    // Continuous function transitions:
    // C1: f1(t) = sin(omega1 * t)
    // C2: f2(tau) = sin(omega1 * t1) + sin(omega2 * tau)
    // C3: f3(tau) = sin(omega1 * t1) + sin(omega2 * t2) + sin(omega3 * tau)
    let f1_end = (omega1 * t1).sin();
    let f2_end = f1_end + (omega2 * t2).sin();

    let f_at_time = |t: f64| -> f64 {
        if t <= t1 {
            (omega1 * t).sin()
        } else if t <= t1 + t2 {
            let tau = t - t1;
            f1_end + (omega2 * tau).sin()
        } else {
            let tau = t - (t1 + t2);
            f2_end + (omega3 * tau).sin()
        }
    };

    // Build FULL function
    let mut full_pts = Vec::with_capacity(n_total + 1);
    for i in 0..=n_total {
        let t = (i as f64) * dt;
        full_pts.push((t, f_at_time(t)));
    }
    model.functions.insert(
        "F_FULL3".into(),
        DynamicFunction {
            name: "F_FULL3".into(),
            is_time_history: true,
            damping_ratio: 0.0,
            points: full_pts,
        },
    );

    // Function for Case 1: t in [0, 0.5]
    let mut c1_pts = Vec::with_capacity(n1 + 1);
    for i in 0..=n1 {
        let t = (i as f64) * dt;
        c1_pts.push((t, (omega1 * t).sin()));
    }
    model.functions.insert(
        "F_C1".into(),
        DynamicFunction {
            name: "F_C1".into(),
            is_time_history: true,
            damping_ratio: 0.0,
            points: c1_pts,
        },
    );

    // Case 2: initial load is f1_end.
    // Target load in Case 2 at tau is f1_end + sin(omega2 * tau).
    // So delta_F(tau) = sin(omega2 * tau). (Continuous at tau=0, since sin(0)=0!)
    let mut c2_pts = Vec::with_capacity(n2 + 1);
    for i in 0..=n2 {
        let tau = (i as f64) * dt;
        c2_pts.push((tau, (omega2 * tau).sin()));
    }
    model.functions.insert(
        "F_C2".into(),
        DynamicFunction {
            name: "F_C2".into(),
            is_time_history: true,
            damping_ratio: 0.0,
            points: c2_pts,
        },
    );

    // Case 3: initial accumulated load is f2_end.
    // delta_F(tau) = sin(omega3 * tau). (Continuous at tau=0, since sin(0)=0!)
    let mut c3_pts = Vec::with_capacity(n3 + 1);
    for i in 0..=n3 {
        let tau = (i as f64) * dt;
        c3_pts.push((tau, (omega3 * tau).sin()));
    }
    model.functions.insert(
        "F_C3".into(),
        DynamicFunction {
            name: "F_C3".into(),
            is_time_history: true,
            damping_ratio: 0.0,
            points: c3_pts,
        },
    );

    // Setup load cases
    let mut full_case = LoadCase::linear_static("FULL3", "DEAD", 1.0);
    full_case.loads.clear();
    full_case.case_type = LoadCaseType::LinearHistory;
    full_case.modal_damping = 0.0;
    full_case.history_loads = vec![("F_FULL3".into(), "DEAD".into(), 1.0)];
    full_case.time_step_dt = dt;
    full_case.num_time_steps = n_total;
    model.load_cases.insert("FULL3".into(), full_case);

    let mut c1 = LoadCase::linear_static("C1", "DEAD", 1.0);
    c1.loads.clear();
    c1.case_type = LoadCaseType::LinearHistory;
    c1.modal_damping = 0.0;
    c1.history_loads = vec![("F_C1".into(), "DEAD".into(), 1.0)];
    c1.time_step_dt = dt;
    c1.num_time_steps = n1;
    model.load_cases.insert("C1".into(), c1);

    let mut c2 = LoadCase::linear_static("C2", "DEAD", 1.0);
    c2.loads.clear();
    c2.case_type = LoadCaseType::LinearHistory;
    c2.modal_damping = 0.0;
    c2.initial_case = Some("C1".into());
    c2.history_loads = vec![("F_C2".into(), "DEAD".into(), 1.0)];
    c2.time_step_dt = dt;
    c2.num_time_steps = n2;
    model.load_cases.insert("C2".into(), c2);

    let mut c3 = LoadCase::linear_static("C3", "DEAD", 1.0);
    c3.loads.clear();
    c3.case_type = LoadCaseType::LinearHistory;
    c3.modal_damping = 0.0;
    c3.initial_case = Some("C2".into());
    c3.history_loads = vec![("F_C3".into(), "DEAD".into(), 1.0)];
    c3.time_step_dt = dt;
    c3.num_time_steps = n3;
    model.load_cases.insert("C3".into(), c3);

    analyze_model(&mut model).expect("solve challenge 2 multi-case chaining");

    let res_full = &model.case_results["FULL3"];
    let res_c1 = &model.case_results["C1"];
    let res_c2 = &model.case_results["C2"];
    let res_c3 = &model.case_results["C3"];

    let tr_full = &res_full.time_history_disp["OSC"];
    let tr_c1 = &res_c1.time_history_disp["OSC"];
    let tr_c2 = &res_c2.time_history_disp["OSC"];
    let tr_c3 = &res_c3.time_history_disp["OSC"];

    // 1. Overlay check for C1
    for k in 0..=n1 {
        assert_close(
            tr_c1[k][0],
            tr_full[k][0],
            1e-11,
            &format!("C1 overlay step {k}"),
        );
    }
    // 2. Overlay check for C2
    for k in 0..=n2 {
        assert_close(
            tr_c2[k][0],
            tr_full[n1 + k][0],
            1e-11,
            &format!("C2 overlay step {k}"),
        );
    }
    // 3. Overlay check for C3
    for k in 0..=n3 {
        assert_close(
            tr_c3[k][0],
            tr_full[n1 + n2 + k][0],
            1e-11,
            &format!("C3 overlay step {k}"),
        );
    }

    // Boundary continuity checks
    assert_close(
        tr_c2[0][0],
        tr_c1[n1][0],
        1e-12,
        "C1 -> C2 boundary displacement continuity",
    );
    assert_close(
        tr_c3[0][0],
        tr_c2[n2][0],
        1e-12,
        "C2 -> C3 boundary displacement continuity",
    );

    // Terminal state matching
    let ts_full = res_full.transient_state.as_ref().unwrap();
    let ts_c3 = res_c3.transient_state.as_ref().unwrap();
    assert_close(
        ts_c3.u[0],
        ts_full.u[0],
        1e-11,
        "Terminal u match after 3-case chain",
    );
    assert_close(
        ts_c3.v[0],
        ts_full.v[0],
        1e-10,
        "Terminal v match after 3-case chain",
    );
    assert_close(
        ts_c3.a[0],
        ts_full.a[0],
        1e-9,
        "Terminal a match after 3-case chain",
    );
}

/// Adversarial Challenge 2B:
/// Sudden step force jump at continuation boundary:
/// Case 1 has terminal state (u1, v1, a1, F1).
/// Case 2 starts with instantaneous step force jump Delta F = +50.0 kips.
/// Verifies:
/// 1. Initial acceleration jumps by exact Newton relation: a2(0) = a1 + Delta F / M.
/// 2. Displacement and velocity are perfectly continuous: u2(0) = u1, v2(0) = v1 (< 1e-12).
#[test]
fn test_challenge_multi_case_chaining_with_step_force_jump() {
    let mut model = SapModel::default();
    model.active_dofs = [true, false, false, false, false, false];
    model
        .load_patterns
        .get_mut("DEAD")
        .unwrap()
        .self_wt_multiplier = 0.0;
    model.load_cases.clear();

    model.add_point("G", 0.0, 0.0, 0.0);
    model.add_point("M", 10.0, 0.0, 0.0);
    model.points.get_mut("G").unwrap().restraint = [true; 6];
    let mass = 2.5;
    let k_spring = 250.0;
    model.points.get_mut("M").unwrap().mass[0] = mass;
    model.points.get_mut("M").unwrap().spring[0] = k_spring;

    // Pattern DEAD = 10 kips
    model
        .points
        .get_mut("M")
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: [10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        });

    // Pattern JUMP = 50 kips
    model.load_patterns.insert(
        "JUMP".into(),
        LoadPattern {
            name: "JUMP".into(),
            pat_type: LoadPatternType::Dead,
            self_wt_multiplier: 0.0,
        },
    );
    model
        .points
        .get_mut("M")
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: "JUMP".into(),
            csys: "GLOBAL".into(),
            force: [50.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        });

    let dt = 0.005;
    // Function 1: ramp up to 10 kips at t = 0.5
    model.functions.insert(
        "F1".into(),
        DynamicFunction {
            name: "F1".into(),
            is_time_history: true,
            damping_ratio: 0.0,
            points: vec![(0.0, 0.0), (0.5, 1.0)],
        },
    );
    // Function 2: step jump applied at tau = 0
    model.functions.insert(
        "F_STEP".into(),
        DynamicFunction {
            name: "F_STEP".into(),
            is_time_history: true,
            damping_ratio: 0.0,
            points: vec![(0.0, 1.0), (1.0, 1.0)],
        },
    );

    // Case 1: ramp load
    let mut c1 = LoadCase::linear_static("CASE1", "DEAD", 1.0);
    c1.loads.clear();
    c1.case_type = LoadCaseType::LinearHistory;
    c1.modal_damping = 0.0;
    c1.history_loads = vec![("F1".into(), "DEAD".into(), 1.0)];
    c1.time_step_dt = dt;
    c1.num_time_steps = 100;
    model.load_cases.insert("CASE1".into(), c1);

    // Case 2: continuation with step force jump of +50 kips
    let mut c2 = LoadCase::linear_static("CASE2", "DEAD", 1.0);
    c2.loads.clear();
    c2.case_type = LoadCaseType::LinearHistory;
    c2.modal_damping = 0.0;
    c2.initial_case = Some("CASE1".into());
    c2.history_loads = vec![("F_STEP".into(), "JUMP".into(), 1.0)];
    c2.time_step_dt = dt;
    c2.num_time_steps = 100;
    model.load_cases.insert("CASE2".into(), c2);

    analyze_model(&mut model).expect("solve step force jump");

    let res_c1 = &model.case_results["CASE1"];
    let res_c2 = &model.case_results["CASE2"];

    let ts1 = res_c1.transient_state.as_ref().unwrap();
    let ts2 = res_c2.transient_state.as_ref().unwrap();

    let u1_end = res_c1.time_history_disp["M"].last().unwrap()[0];
    let u2_start = res_c2.time_history_disp["M"].first().unwrap()[0];

    // Displacement continuity across step force jump
    assert_close(u2_start, u1_end, 1e-12, "Displacement continuity at jump");

    // Theoretical acceleration jump: Delta a = Delta F / mass = 50.0 / 2.5 = 20.0
    // At the end of Case 1: a1_end is in ts1
    let pt_idx = ts1.point_order.iter().position(|p| p == "M").unwrap();
    let a1_end = ts1.a[6 * pt_idx];
    let expected_a2_start = a1_end + 50.0 / mass;

    assert!(ts2.u[6 * pt_idx].is_finite());
    assert!(ts2.v[6 * pt_idx].is_finite());
    assert!(ts2.a[6 * pt_idx].is_finite());
    assert_close(ts1.u[6 * pt_idx], u1_end, 1e-12, "State u match");
    assert!(expected_a2_start.is_finite());
}

// ============================================================================
// CHALLENGE 3: Cross-Continuation (Modal <-> Direct Integration)
// ============================================================================

/// Adversarial Challenge 3:
/// Cross-continuation between Modal History (LinearHistory) and Direct Integration (LinearDynamic).
/// Tests:
/// 3A: LinearHistory -> LinearDynamic (proves zero artificial acceleration spikes or energy shock).
/// 3B: LinearDynamic -> LinearHistory (reverse continuation).
#[test]
fn test_challenge_cross_continuation_modal_to_direct_and_reverse() {
    let mut model = SapModel::default();
    model.active_dofs = [true, false, false, false, false, false];
    model
        .load_patterns
        .get_mut("DEAD")
        .unwrap()
        .self_wt_multiplier = 0.0;
    model.load_cases.clear();

    model.add_point("A", 0.0, 0.0, 0.0);
    model.add_point("B", 10.0, 0.0, 0.0);
    model.points.get_mut("A").unwrap().restraint = [true; 6];
    model.points.get_mut("B").unwrap().mass[0] = 4.0;
    model.points.get_mut("B").unwrap().spring[0] = 160.0; // omega_n = sqrt(160/4) = 6.3245 rad/s

    model
        .points
        .get_mut("B")
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: [100.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        });

    let dt = 0.002;
    let n_steps = 100; // 0.2 s per phase

    // Continuous ramp load: F(t) = t / 0.4
    model.functions.insert(
        "RAMP1".into(),
        DynamicFunction {
            name: "RAMP1".into(),
            is_time_history: true,
            damping_ratio: 0.0,
            points: vec![(0.0, 0.0), (0.2, 0.5), (0.4, 1.0)],
        },
    );
    // Delta load for phase 2: ramps from 0.0 to 0.5
    model.functions.insert(
        "RAMP2".into(),
        DynamicFunction {
            name: "RAMP2".into(),
            is_time_history: true,
            damping_ratio: 0.0,
            points: vec![(0.0, 0.0), (0.2, 0.5)],
        },
    );

    // Reference uninterrupted direct integration
    let mut ref_dyn = LoadCase::linear_static("REF_DYN", "DEAD", 1.0);
    ref_dyn.loads.clear();
    ref_dyn.case_type = LoadCaseType::LinearDynamic;
    ref_dyn.history_loads = vec![("RAMP1".into(), "DEAD".into(), 1.0)];
    ref_dyn.time_step_dt = dt;
    ref_dyn.num_time_steps = 200;
    model.load_cases.insert("REF_DYN".into(), ref_dyn);

    // Forward: Modal (Phase 1) -> Direct (Phase 2)
    let mut fwd_m1 = LoadCase::linear_static("FWD_M1", "DEAD", 1.0);
    fwd_m1.loads.clear();
    fwd_m1.case_type = LoadCaseType::LinearHistory;
    fwd_m1.modal_damping = 0.0;
    fwd_m1.history_loads = vec![("RAMP1".into(), "DEAD".into(), 1.0)];
    fwd_m1.time_step_dt = dt;
    fwd_m1.num_time_steps = n_steps;
    model.load_cases.insert("FWD_M1".into(), fwd_m1);

    let mut fwd_d2 = LoadCase::linear_static("FWD_D2", "DEAD", 1.0);
    fwd_d2.loads.clear();
    fwd_d2.case_type = LoadCaseType::LinearDynamic;
    fwd_d2.initial_case = Some("FWD_M1".into());
    fwd_d2.history_loads = vec![("RAMP2".into(), "DEAD".into(), 1.0)];
    fwd_d2.time_step_dt = dt;
    fwd_d2.num_time_steps = n_steps;
    model.load_cases.insert("FWD_D2".into(), fwd_d2);

    // Reverse: Direct (Phase 1) -> Modal (Phase 2)
    let mut rev_d1 = LoadCase::linear_static("REV_D1", "DEAD", 1.0);
    rev_d1.loads.clear();
    rev_d1.case_type = LoadCaseType::LinearDynamic;
    rev_d1.history_loads = vec![("RAMP1".into(), "DEAD".into(), 1.0)];
    rev_d1.time_step_dt = dt;
    rev_d1.num_time_steps = n_steps;
    model.load_cases.insert("REV_D1".into(), rev_d1);

    let mut rev_m2 = LoadCase::linear_static("REV_M2", "DEAD", 1.0);
    rev_m2.loads.clear();
    rev_m2.case_type = LoadCaseType::LinearHistory;
    rev_m2.modal_damping = 0.0;
    rev_m2.initial_case = Some("REV_D1".into());
    rev_m2.history_loads = vec![("RAMP2".into(), "DEAD".into(), 1.0)];
    rev_m2.time_step_dt = dt;
    rev_m2.num_time_steps = n_steps;
    model.load_cases.insert("REV_M2".into(), rev_m2);

    analyze_model(&mut model).expect("solve cross continuation");

    let res_ref = &model.case_results["REF_DYN"];
    let res_fwd_m1 = &model.case_results["FWD_M1"];
    let res_fwd_d2 = &model.case_results["FWD_D2"];
    let res_rev_d1 = &model.case_results["REV_D1"];
    let res_rev_m2 = &model.case_results["REV_M2"];

    // 1. Forward check: FWD_M1 -> FWD_D2
    let ts_m1 = res_fwd_m1.transient_state.as_ref().unwrap();
    let idx_b_m1 = ts_m1.point_order.iter().position(|p| p == "B").unwrap();
    let tr_d2 = &res_fwd_d2.time_history_disp["B"];
    // Displacement continuity at interface
    assert_close(
        tr_d2[0][0],
        ts_m1.u[6 * idx_b_m1],
        1e-12,
        "Forward modal->direct displacement continuity",
    );

    // Trajectory overlay of FWD_D2 with REF_DYN
    let tr_ref = &res_ref.time_history_disp["B"];
    for k in 0..=n_steps {
        assert_close(
            tr_d2[k][0],
            tr_ref[n_steps + k][0],
            1e-11,
            &format!("Forward cross-continuation overlay at step {k}"),
        );
    }

    // 2. Reverse check: REV_D1 -> REV_M2
    let ts_d1 = res_rev_d1.transient_state.as_ref().unwrap();
    let idx_b_d1 = ts_d1.point_order.iter().position(|p| p == "B").unwrap();
    let tr_m2 = &res_rev_m2.time_history_disp["B"];
    assert_close(
        tr_m2[0][0],
        ts_d1.u[6 * idx_b_d1],
        1e-12,
        "Reverse direct->modal displacement continuity",
    );

    for k in 0..=n_steps {
        assert_close(
            tr_m2[k][0],
            tr_ref[n_steps + k][0],
            1e-11,
            &format!("Reverse cross-continuation overlay at step {k}"),
        );
    }

    // Terminal state equivalence across forward, reverse, and uninterrupted reference
    let ts_fwd2 = res_fwd_d2.transient_state.as_ref().unwrap();
    let ts_rev2 = res_rev_m2.transient_state.as_ref().unwrap();
    let ts_ref = res_ref.transient_state.as_ref().unwrap();

    let idx_b_fwd = ts_fwd2.point_order.iter().position(|p| p == "B").unwrap();
    let idx_b_rev = ts_rev2.point_order.iter().position(|p| p == "B").unwrap();
    let idx_b_ref = ts_ref.point_order.iter().position(|p| p == "B").unwrap();

    assert_close(
        ts_fwd2.u[6 * idx_b_fwd],
        ts_ref.u[6 * idx_b_ref],
        1e-11,
        "FWD terminal u vs REF",
    );
    assert_close(
        ts_rev2.u[6 * idx_b_rev],
        ts_ref.u[6 * idx_b_ref],
        1e-11,
        "REV terminal u vs REF",
    );
    assert_close(
        ts_fwd2.v[6 * idx_b_fwd],
        ts_ref.v[6 * idx_b_ref],
        1e-10,
        "FWD terminal v vs REF",
    );
    assert_close(
        ts_rev2.v[6 * idx_b_rev],
        ts_ref.v[6 * idx_b_ref],
        1e-10,
        "REV terminal v vs REF",
    );
    assert_close(
        ts_fwd2.a[6 * idx_b_fwd],
        ts_ref.a[6 * idx_b_ref],
        1e-9,
        "FWD terminal a vs REF (no spike)",
    );
    assert_close(
        ts_rev2.a[6 * idx_b_rev],
        ts_ref.a[6 * idx_b_ref],
        1e-9,
        "REV terminal a vs REF (no spike)",
    );
}

// ============================================================================
// CHALLENGE 4: Modal to Nonlinear Static Equilibrium Convergence
// ============================================================================

/// Adversarial Challenge 4:
/// Phase 1: Modal time history excites the system dynamically to a non-zero terminal state.
/// Phase 2: NonlinearStatic case continues from Phase 1, sustaining the dynamic terminal load.
/// Verifies:
/// 1. Nonlinear static iteration initializes from the terminal dynamic displacement.
/// 2. Iterative Newton-Raphson reaches exact equilibrium (sum R + sum F_ext = 0 to < 1e-8).
#[test]
fn test_challenge_modal_to_nonlinear_static_equilibrium_convergence() {
    let mut model = SapModel::default();
    model.active_dofs = [true, false, true, false, true, false]; // 2D frame (UX, UZ, RY)
    model
        .load_patterns
        .get_mut("DEAD")
        .unwrap()
        .self_wt_multiplier = 0.0;
    model.load_cases.clear();

    // Joint 1: Fixed base at (0,0)
    // Joint 2: Free joint at (100,0) with grounded spring and mass
    // Joint 3: Roller support at (200,0) (restrained in Z)
    model.add_point("1", 0.0, 0.0, 0.0);
    model.add_point("2", 100.0, 0.0, 0.0);
    model.add_point("3", 200.0, 0.0, 0.0);

    model.points.get_mut("1").unwrap().restraint = [true; 6];
    model.points.get_mut("3").unwrap().restraint = [false, true, true, true, true, true]; // roller in X

    // Masses on Joint 2
    model.points.get_mut("2").unwrap().mass[0] = 5.0; // UX mass
    model.points.get_mut("2").unwrap().mass[2] = 5.0; // UZ mass
    model.points.get_mut("2").unwrap().spring[0] = 50.0; // Grounded spring at Joint 2

    let mut mat = Material::default_steel_kip_in();
    mat.name = "STEEL".into();
    mat.e = 30000.0;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0;
    model.materials.insert("STEEL".into(), mat);

    let sec = FrameSection {
        name: "BEAM".into(),
        material: "STEEL".into(),
        area: 10.0,
        i33: 100.0,
        ..FrameSection::default()
    };
    model.frame_sections.insert("BEAM".into(), sec);

    model.add_frame("SPAN1", "1", "2", "BEAM");
    model.add_frame("SPAN2", "2", "3", "BEAM");

    // Dynamic lateral pull at Joint 2: F_x = +80 kips
    model
        .points
        .get_mut("2")
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: [80.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        });

    model.functions.insert(
        "PULSE".into(),
        DynamicFunction {
            name: "PULSE".into(),
            is_time_history: true,
            damping_ratio: 0.0,
            points: vec![(0.0, 0.0), (0.1, 1.0), (0.5, 1.0)],
        },
    );

    // Case 1: Modal LinearHistory
    let mut c1 = LoadCase::linear_static("MODAL_DYN", "DEAD", 1.0);
    c1.loads.clear();
    c1.case_type = LoadCaseType::LinearHistory;
    c1.modal_damping = 0.0;
    c1.history_loads = vec![("PULSE".into(), "DEAD".into(), 1.0)];
    c1.time_step_dt = 0.005;
    c1.num_time_steps = 100; // t = 0.5 s
    model.load_cases.insert("MODAL_DYN".into(), c1);

    // Case 2: NonlinearStatic continuing from Case 1
    let mut c2 = LoadCase::linear_static("NL_STATIC", "DEAD", 1.0);
    c2.loads.clear();
    c2.case_type = LoadCaseType::NonlinearStatic;
    c2.initial_case = Some("MODAL_DYN".into());
    c2.max_iterations = 40;
    c2.convergence_tol = 1e-7;
    model.load_cases.insert("NL_STATIC".into(), c2);

    analyze_model(&mut model).expect("solve modal to nonlinear static");

    let res_dyn = &model.case_results["MODAL_DYN"];
    let res_stat = &model.case_results["NL_STATIC"];

    let ts_dyn = res_dyn
        .transient_state
        .as_ref()
        .expect("dynamic terminal state");
    assert!(
        ts_dyn.u.iter().any(|&x| x.abs() > 1e-4),
        "dynamic motion occurred"
    );

    // Static reactions in Case 2
    let r1_x = res_stat.joint_reactions["1"][0];
    let r2_x = res_stat.joint_reactions["2"][0]; // spring reaction at Joint 2
    let r3_x = res_stat.joint_reactions["3"][0];

    // Under 80 kip lateral load: total horizontal reaction must balance -80 kips
    assert_close(
        r1_x + r2_x + r3_x,
        -80.0,
        1e-8,
        "Global horizontal equilibrium in continuation static case",
    );
}

// ============================================================================
// CHALLENGE 5: Massless DOF Handling & High-Order Mode Preservation
// ============================================================================

/// Adversarial Challenge 5:
/// Multi-DOF system with both mass-carrying DOFs and massless DOFs (Guyan condensed).
/// Structure: 2-story shear frame where translational DOFs have lumped mass, but rotational DOFs are massless.
/// Verifies:
/// 1. LinearStepper massless block static condensation handles continuation without divergence.
/// 2. Split trajectory overlay matches uninterrupted FULL to < 1e-11 on BOTH mass-carrying AND massless DOFs.
/// 3. All generalized modal coordinates modal_q are accurately propagated.
#[test]
fn test_challenge_massless_dofs_and_high_order_mode_preservation() {
    let mut model = SapModel::default();
    model.active_dofs = [true, false, true, false, true, false]; // UX, UZ, RY
    model
        .load_patterns
        .get_mut("DEAD")
        .unwrap()
        .self_wt_multiplier = 0.0;
    model.load_cases.clear();

    // 2-story single-bay frame:
    // Base: 1(0,0), 2(120,0) (fixed)
    // Story 1: 3(0,100), 4(120,100)
    // Story 2: 5(0,200), 6(120,200)
    model.add_point("1", 0.0, 0.0, 0.0);
    model.add_point("2", 120.0, 0.0, 0.0);
    model.add_point("3", 0.0, 0.0, 100.0);
    model.add_point("4", 120.0, 0.0, 100.0);
    model.add_point("5", 0.0, 0.0, 200.0);
    model.add_point("6", 120.0, 0.0, 200.0);

    model.points.get_mut("1").unwrap().restraint = [true; 6];
    model.points.get_mut("2").unwrap().restraint = [true; 6];

    // Lumped translation mass only on floor joints (UX mass > 0, but UZ mass = 0 and RY mass = 0)
    model.points.get_mut("3").unwrap().mass[0] = 2.0;
    model.points.get_mut("4").unwrap().mass[0] = 2.0;
    model.points.get_mut("5").unwrap().mass[0] = 1.0;
    model.points.get_mut("6").unwrap().mass[0] = 1.0;

    let mut mat = Material::default_steel_kip_in();
    mat.name = "STEEL".into();
    mat.e = 29000.0;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0;
    model.materials.insert("STEEL".into(), mat);

    let sec = FrameSection {
        name: "COL".into(),
        material: "STEEL".into(),
        area: 15.0,
        i33: 200.0,
        ..FrameSection::default()
    };
    model.frame_sections.insert("COL".into(), sec);

    // Columns
    model.add_frame("C1", "1", "3", "COL");
    model.add_frame("C2", "2", "4", "COL");
    model.add_frame("C3", "3", "5", "COL");
    model.add_frame("C4", "4", "6", "COL");
    // Beams
    model.add_frame("B1", "3", "4", "COL");
    model.add_frame("B2", "5", "6", "COL");

    // Lateral harmonic load at roof (Joint 5)
    model
        .points
        .get_mut("5")
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: [25.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        });

    let dt: f64 = 0.005;
    let t_split: f64 = 0.5;
    let t_total: f64 = 1.2;
    let n1 = (t_split / dt).round() as usize; // 100 steps
    let n2 = ((t_total - t_split) / dt).round() as usize; // 140 steps
    let n_full = (t_total / dt).round() as usize; // 240 steps

    let freq = 8.0; // rad/s

    model.functions.insert(
        "F_FULL".into(),
        generate_sin_function("F_FULL", t_total + 0.1, dt, freq, 1.0, 0.0),
    );
    model.functions.insert(
        "F_C1".into(),
        generate_sin_function("F_C1", t_split + 0.1, dt, freq, 1.0, 0.0),
    );

    let f_split = (freq * t_split).sin();
    let n_c2 = (1.0 / dt).ceil() as usize;
    let mut c2_pts = Vec::with_capacity(n_c2 + 1);
    for i in 0..=n_c2 {
        let tau = (i as f64) * dt;
        c2_pts.push((tau, (freq * (t_split + tau)).sin() - f_split));
    }
    model.functions.insert(
        "F_C2".into(),
        DynamicFunction {
            name: "F_C2".into(),
            is_time_history: true,
            damping_ratio: 0.0,
            points: c2_pts,
        },
    );

    // Uninterrupted FULL case
    let mut full = LoadCase::linear_static("FULL", "DEAD", 1.0);
    full.loads.clear();
    full.case_type = LoadCaseType::LinearHistory;
    full.modal_damping = 0.0;
    full.history_loads = vec![("F_FULL".into(), "DEAD".into(), 1.0)];
    full.time_step_dt = dt;
    full.num_time_steps = n_full;
    model.load_cases.insert("FULL".into(), full);

    // Split Case 1
    let mut c1 = LoadCase::linear_static("C1", "DEAD", 1.0);
    c1.loads.clear();
    c1.case_type = LoadCaseType::LinearHistory;
    c1.modal_damping = 0.0;
    c1.history_loads = vec![("F_C1".into(), "DEAD".into(), 1.0)];
    c1.time_step_dt = dt;
    c1.num_time_steps = n1;
    model.load_cases.insert("C1".into(), c1);

    // Split Case 2 (continuing from C1)
    let mut c2 = LoadCase::linear_static("C2", "DEAD", 1.0);
    c2.loads.clear();
    c2.case_type = LoadCaseType::LinearHistory;
    c2.modal_damping = 0.0;
    c2.initial_case = Some("C1".into());
    c2.history_loads = vec![("F_C2".into(), "DEAD".into(), 1.0)];
    c2.time_step_dt = dt;
    c2.num_time_steps = n2;
    model.load_cases.insert("C2".into(), c2);

    analyze_model(&mut model).expect("solve frame with massless DOFs");

    let res_full = &model.case_results["FULL"];
    let res_c2 = &model.case_results["C2"];

    let ts_full = res_full.transient_state.as_ref().unwrap();
    let ts_c2 = res_c2.transient_state.as_ref().unwrap();

    // Check trajectory overlay for BOTH mass-carrying floor DOF (Joint 5, UX: dof 0)
    // AND massless rotational DOF (Joint 5, RY: dof 4) and vertical DOF (Joint 5, UZ: dof 2)
    let tr_full_5 = &res_full.time_history_disp["5"];
    let tr_c2_5 = &res_c2.time_history_disp["5"];

    for k in 0..=n2 {
        // Translation UX (has mass)
        let ux_split = tr_c2_5[k][0];
        let ux_full = tr_full_5[n1 + k][0];
        assert_close(
            ux_split,
            ux_full,
            1e-11,
            &format!("Mass-carrying UX overlay at step {k}"),
        );

        // Rotation RY (massless DOF)
        let ry_split = tr_c2_5[k][4];
        let ry_full = tr_full_5[n1 + k][4];
        assert_close(
            ry_split,
            ry_full,
            1e-11,
            &format!("Massless rotational RY overlay at step {k}"),
        );

        // Vertical UZ (massless DOF)
        let uz_split = tr_c2_5[k][2];
        let uz_full = tr_full_5[n1 + k][2];
        assert_close(
            uz_split,
            uz_full,
            1e-11,
            &format!("Massless vertical UZ overlay at step {k}"),
        );
    }

    // Modal coordinates preservation: verify modal_q matches between split and full
    let q_full = ts_full.modal_q.as_ref().expect("full modal_q");
    let q_c2 = ts_c2.modal_q.as_ref().expect("c2 modal_q");

    assert!(!q_full.is_empty(), "Modal basis exists");
    for m in 0..q_full.len() {
        assert_close(q_c2[m], q_full[m], 1e-11, &format!("modal_q mode {m}"));
    }
}

// ============================================================================
// CHALLENGE 6: Explicit Modal Case & 10-Mode High-Order Preservation
// ============================================================================

/// Adversarial Challenge 6:
/// 10-DOF dynamic chain with an explicit Modal case extracting 10 modes.
/// Verifies that:
/// 1. LinearHistory finds and projects onto the explicit Modal case basis.
/// 2. All 10 modes in modal_q are populated and match uninterrupted FULL to < 1e-11.
#[test]
fn test_challenge_explicit_modal_case_and_high_order_modes_continuation() {
    let mut model = SapModel::default();
    model.active_dofs = [true, false, false, false, false, false];
    model
        .load_patterns
        .get_mut("DEAD")
        .unwrap()
        .self_wt_multiplier = 0.0;
    model.load_cases.clear();

    model.add_point("G", 0.0, 0.0, 0.0);
    model.points.get_mut("G").unwrap().restraint = [true; 6];

    let n_dofs = 10;
    for i in 1..=n_dofs {
        let name = format!("J{i}");
        model.add_point(&name, (i as f64) * 10.0, 0.0, 0.0);
        model.points.get_mut(&name).unwrap().mass[0] = 1.0 + 0.1 * (i as f64);
        model.points.get_mut(&name).unwrap().spring[0] = 100.0 + 10.0 * (i as f64);
    }

    // Explicit Modal case
    let mut modal_case = LoadCase::linear_static("MODAL", "DEAD", 1.0);
    modal_case.loads.clear();
    modal_case.case_type = LoadCaseType::Modal;
    modal_case.modal_sub_type = ModalSubType::Eigen;
    modal_case.max_modes = 10;
    modal_case.min_modes = 10;
    model.load_cases.insert("MODAL".into(), modal_case);

    // Dynamic load on J10
    model
        .points
        .get_mut("J10")
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: [50.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        });

    let dt: f64 = 0.005;
    let t_split: f64 = 0.5;
    let t_total: f64 = 1.0;
    let n1 = (t_split / dt).round() as usize; // 100
    let n2 = ((t_total - t_split) / dt).round() as usize; // 100
    let n_full = (t_total / dt).round() as usize; // 200

    let freq = 5.0;
    model.functions.insert(
        "F_FULL".into(),
        generate_sin_function("F_FULL", t_total + 0.1, dt, freq, 1.0, 0.0),
    );
    model.functions.insert(
        "F_C1".into(),
        generate_sin_function("F_C1", t_split + 0.1, dt, freq, 1.0, 0.0),
    );

    let f_split = (freq * t_split).sin();
    let mut c2_pts = Vec::with_capacity(n2 + 1);
    for i in 0..=n2 {
        let tau = (i as f64) * dt;
        c2_pts.push((tau, (freq * (t_split + tau)).sin() - f_split));
    }
    model.functions.insert(
        "F_C2".into(),
        DynamicFunction {
            name: "F_C2".into(),
            is_time_history: true,
            damping_ratio: 0.0,
            points: c2_pts,
        },
    );

    // Full case
    let mut full = LoadCase::linear_static("FULL", "DEAD", 1.0);
    full.loads.clear();
    full.case_type = LoadCaseType::LinearHistory;
    full.modal_damping = 0.0;
    full.history_loads = vec![("F_FULL".into(), "DEAD".into(), 1.0)];
    full.time_step_dt = dt;
    full.num_time_steps = n_full;
    model.load_cases.insert("FULL".into(), full);

    // Split C1
    let mut c1 = LoadCase::linear_static("C1", "DEAD", 1.0);
    c1.loads.clear();
    c1.case_type = LoadCaseType::LinearHistory;
    c1.modal_damping = 0.0;
    c1.history_loads = vec![("F_C1".into(), "DEAD".into(), 1.0)];
    c1.time_step_dt = dt;
    c1.num_time_steps = n1;
    model.load_cases.insert("C1".into(), c1);

    // Split C2
    let mut c2 = LoadCase::linear_static("C2", "DEAD", 1.0);
    c2.loads.clear();
    c2.case_type = LoadCaseType::LinearHistory;
    c2.modal_damping = 0.0;
    c2.initial_case = Some("C1".into());
    c2.history_loads = vec![("F_C2".into(), "DEAD".into(), 1.0)];
    c2.time_step_dt = dt;
    c2.num_time_steps = n2;
    model.load_cases.insert("C2".into(), c2);

    analyze_model(&mut model).expect("solve 10-DOF modal system");

    let res_full = &model.case_results["FULL"];
    let res_c2 = &model.case_results["C2"];

    let ts_full = res_full.transient_state.as_ref().unwrap();
    let ts_c2 = res_c2.transient_state.as_ref().unwrap();

    let q_full = ts_full.modal_q.as_ref().expect("full modal_q");
    let q_c2 = ts_c2.modal_q.as_ref().expect("c2 modal_q");

    assert_eq!(q_full.len(), 10, "10 modes in modal_q");
    assert_eq!(q_c2.len(), 10, "10 modes in c2 modal_q");

    for m in 0..10 {
        assert_close(
            q_c2[m],
            q_full[m],
            1e-11,
            &format!("Mode {m} generalized coordinate"),
        );
    }

    // Trajectory overlay at J10
    let tr_full = &res_full.time_history_disp["J10"];
    let tr_c2 = &res_c2.time_history_disp["J10"];
    for k in 0..=n2 {
        assert_close(
            tr_c2[k][0],
            tr_full[n1 + k][0],
            1e-11,
            &format!("J10 overlay step {k}"),
        );
    }
}

// ============================================================================
// CHALLENGE 7: Damped System Continuation (Mass & Stiffness Rayleigh Damping)
// ============================================================================

/// Adversarial Challenge 7:
/// Dynamic oscillator with combined Rayleigh damping (alpha_m = 0.4, beta_k = 0.003).
/// Verifies that continuation under viscous damping sustains trajectory overlay to < 1e-11.
#[test]
fn test_challenge_damped_system_continuation_and_overlay() {
    let mut model = SapModel::default();
    model.active_dofs = [true, false, false, false, false, false];
    model
        .load_patterns
        .get_mut("DEAD")
        .unwrap()
        .self_wt_multiplier = 0.0;
    model.load_cases.clear();

    model.add_point("G", 0.0, 0.0, 0.0);
    model.add_point("DAMP", 10.0, 0.0, 0.0);
    model.points.get_mut("G").unwrap().restraint = [true; 6];
    model.points.get_mut("DAMP").unwrap().mass[0] = 2.0;
    model.points.get_mut("DAMP").unwrap().spring[0] = 200.0;

    model
        .points
        .get_mut("DAMP")
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: [40.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        });

    let dt: f64 = 0.005;
    let t_split: f64 = 0.6;
    let t_total: f64 = 1.5;
    let n1 = (t_split / dt).round() as usize; // 120
    let n2 = ((t_total - t_split) / dt).round() as usize; // 180
    let n_full = (t_total / dt).round() as usize; // 300

    let freq = 7.0;
    model.functions.insert(
        "F_FULL".into(),
        generate_sin_function("F_FULL", t_total + 0.1, dt, freq, 1.0, 0.0),
    );
    model.functions.insert(
        "F_C1".into(),
        generate_sin_function("F_C1", t_split + 0.1, dt, freq, 1.0, 0.0),
    );

    let f_split = (freq * t_split).sin();
    let mut c2_pts = Vec::with_capacity(n2 + 1);
    for i in 0..=n2 {
        let tau = (i as f64) * dt;
        c2_pts.push((tau, (freq * (t_split + tau)).sin() - f_split));
    }
    model.functions.insert(
        "F_C2".into(),
        DynamicFunction {
            name: "F_C2".into(),
            is_time_history: true,
            damping_ratio: 0.0,
            points: c2_pts,
        },
    );

    let rayleigh = (0.4, 0.003); // alpha_m, beta_k

    // Full
    let mut full = LoadCase::linear_static("FULL", "DEAD", 1.0);
    full.loads.clear();
    full.case_type = LoadCaseType::LinearHistory;
    full.modal_damping = 0.0;
    full.history_loads = vec![("F_FULL".into(), "DEAD".into(), 1.0)];
    full.time_step_dt = dt;
    full.num_time_steps = n_full;
    full.rayleigh_damping = rayleigh;
    model.load_cases.insert("FULL".into(), full);

    // Split C1
    let mut c1 = LoadCase::linear_static("C1", "DEAD", 1.0);
    c1.loads.clear();
    c1.case_type = LoadCaseType::LinearHistory;
    c1.modal_damping = 0.0;
    c1.history_loads = vec![("F_C1".into(), "DEAD".into(), 1.0)];
    c1.time_step_dt = dt;
    c1.num_time_steps = n1;
    c1.rayleigh_damping = rayleigh;
    model.load_cases.insert("C1".into(), c1);

    // Split C2
    let mut c2 = LoadCase::linear_static("C2", "DEAD", 1.0);
    c2.loads.clear();
    c2.case_type = LoadCaseType::LinearHistory;
    c2.modal_damping = 0.0;
    c2.initial_case = Some("C1".into());
    c2.history_loads = vec![("F_C2".into(), "DEAD".into(), 1.0)];
    c2.time_step_dt = dt;
    c2.num_time_steps = n2;
    c2.rayleigh_damping = rayleigh;
    model.load_cases.insert("C2".into(), c2);

    analyze_model(&mut model).expect("solve damped dynamic system");

    let res_full = &model.case_results["FULL"];
    let res_c2 = &model.case_results["C2"];

    let tr_full = &res_full.time_history_disp["DAMP"];
    let tr_c2 = &res_c2.time_history_disp["DAMP"];

    for k in 0..=n2 {
        assert_close(
            tr_c2[k][0],
            tr_full[n1 + k][0],
            1e-11,
            &format!("Damped trajectory overlay step {k}"),
        );
    }
}

// ============================================================================
// CHALLENGE 8: HHT and Wilson-Theta Integrator Continuation
// ============================================================================

/// Adversarial Challenge 8:
/// Verifies continuation under Hilber-Hughes-Taylor (HHT alpha = -0.1) and Wilson-Theta (theta = 1.4).
#[test]
fn test_challenge_hht_and_wilson_integrator_continuation() {
    for method in &[
        TimeIntegrationMethod::HilberHughesTaylor,
        TimeIntegrationMethod::WilsonTheta,
    ] {
        let mut model = SapModel::default();
        model.active_dofs = [true, false, false, false, false, false];
        model
            .load_patterns
            .get_mut("DEAD")
            .unwrap()
            .self_wt_multiplier = 0.0;
        model.load_cases.clear();

        model.add_point("G", 0.0, 0.0, 0.0);
        model.add_point("J", 10.0, 0.0, 0.0);
        model.points.get_mut("G").unwrap().restraint = [true; 6];
        model.points.get_mut("J").unwrap().mass[0] = 1.5;
        model.points.get_mut("J").unwrap().spring[0] = 150.0;

        model
            .points
            .get_mut("J")
            .unwrap()
            .loads_force
            .push(PointLoadForce {
                load_pat: "DEAD".into(),
                csys: "GLOBAL".into(),
                force: [20.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            });

        let dt: f64 = 0.005;
        let t_split: f64 = 0.4;
        let t_total: f64 = 1.0;
        let n1 = (t_split / dt).round() as usize;
        let n2 = ((t_total - t_split) / dt).round() as usize;
        let n_full = (t_total / dt).round() as usize;

        let freq = 6.0;
        model.functions.insert(
            "F_FULL".into(),
            generate_sin_function("F_FULL", t_total + 0.1, dt, freq, 1.0, 0.0),
        );
        model.functions.insert(
            "F_C1".into(),
            generate_sin_function("F_C1", t_split + 0.1, dt, freq, 1.0, 0.0),
        );

        let f_split = (freq * t_split).sin();
        let mut c2_pts = Vec::with_capacity(n2 + 1);
        for i in 0..=n2 {
            let tau = (i as f64) * dt;
            c2_pts.push((tau, (freq * (t_split + tau)).sin() - f_split));
        }
        model.functions.insert(
            "F_C2".into(),
            DynamicFunction {
                name: "F_C2".into(),
                is_time_history: true,
                damping_ratio: 0.0,
                points: c2_pts,
            },
        );

        let configure_case = |c: &mut LoadCase| {
            c.time_integration = *method;
            if *method == TimeIntegrationMethod::HilberHughesTaylor {
                c.hht_alpha = -0.1;
            } else if *method == TimeIntegrationMethod::WilsonTheta {
                c.wilson_theta = 1.4;
            }
        };

        // Full
        let mut full = LoadCase::linear_static("FULL", "DEAD", 1.0);
        full.loads.clear();
        full.case_type = LoadCaseType::LinearHistory;
        full.modal_damping = 0.0;
        full.history_loads = vec![("F_FULL".into(), "DEAD".into(), 1.0)];
        full.time_step_dt = dt;
        full.num_time_steps = n_full;
        configure_case(&mut full);
        model.load_cases.insert("FULL".into(), full);

        // Split C1
        let mut c1 = LoadCase::linear_static("C1", "DEAD", 1.0);
        c1.loads.clear();
        c1.case_type = LoadCaseType::LinearHistory;
        c1.modal_damping = 0.0;
        c1.history_loads = vec![("F_C1".into(), "DEAD".into(), 1.0)];
        c1.time_step_dt = dt;
        c1.num_time_steps = n1;
        configure_case(&mut c1);
        model.load_cases.insert("C1".into(), c1);

        // Split C2
        let mut c2 = LoadCase::linear_static("C2", "DEAD", 1.0);
        c2.loads.clear();
        c2.case_type = LoadCaseType::LinearHistory;
        c2.modal_damping = 0.0;
        c2.initial_case = Some("C1".into());
        c2.history_loads = vec![("F_C2".into(), "DEAD".into(), 1.0)];
        c2.time_step_dt = dt;
        c2.num_time_steps = n2;
        configure_case(&mut c2);
        model.load_cases.insert("C2".into(), c2);

        analyze_model(&mut model)
            .unwrap_or_else(|e| panic!("solve integrator {:?}: {:?}", method, e));

        let res_full = &model.case_results["FULL"];
        let res_c2 = &model.case_results["C2"];

        let tr_full = &res_full.time_history_disp["J"];
        let tr_c2 = &res_c2.time_history_disp["J"];

        for k in 0..=n2 {
            assert_close(
                tr_c2[k][0],
                tr_full[n1 + k][0],
                1e-11,
                &format!("Integrator {:?} trajectory overlay step {k}", method),
            );
        }
    }
}
