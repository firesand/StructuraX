//! `ground_acceleration_trace` and the accelerations of a modal history.
//!
//! Units are consistent (kip, in, s). Every expected value is a closed form
//! written next to the assertion. Absolute acceleration is the relative
//! acceleration (joint-local axes, as the solver stores it) plus the ground
//! acceleration `a_g` seen in those axes, `Rᵀ·a_g`, where `R` is
//! `sap_solver::frame::joint_local_to_global_matrix` (`v_global = R·v_local`).

use sap_model::*;
use sap_solver::{analyze_model, ground_acceleration_trace};

fn bare_model(active_dofs: [bool; 6]) -> SapModel {
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    m.active_dofs = active_dofs;
    m.load_patterns.insert(
        "P".into(),
        LoadPattern {
            name: "P".into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
    m
}

fn function(m: &mut SapModel, name: &str, points: Vec<(f64, f64)>) {
    m.functions.insert(
        name.into(),
        DynamicFunction {
            name: name.into(),
            is_time_history: true,
            damping_ratio: 0.0,
            points,
        },
    );
}

fn modal_case(m: &mut SapModel, n_modes: usize) {
    m.load_cases
        .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", n_modes));
}

fn modal_history(m: &mut SapModel, xi: f64, dt: f64, steps: usize, loads: Vec<(&str, &str, f64)>) {
    let mut case = LoadCase::linear_static("HIST", "P", 1.0);
    case.case_type = LoadCaseType::LinearHistory;
    case.loads.clear();
    case.modal_case = Some("MODAL".into());
    case.modal_damping = xi;
    case.time_step_dt = dt;
    case.num_time_steps = steps;
    case.history_loads = loads
        .into_iter()
        .map(|(f, p, s)| (f.to_string(), p.to_string(), s))
        .collect();
    m.load_cases.insert("HIST".into(), case);
}

fn sdof(mass: f64, k: f64) -> SapModel {
    let mut m = bare_model([true, false, false, false, false, false]);
    m.add_point("J", 0.0, 0.0, 0.0);
    m.points["J"].mass[0] = mass;
    m.points["J"].spring[0] = k;
    m
}

fn rel_accel(m: &SapModel, joint: &str) -> Vec<[f64; 6]> {
    m.case_results["HIST"].time_history_accel[joint].clone()
}

// ---------------------------------------------------------------------------
// The trace
// ---------------------------------------------------------------------------

#[test]
fn trace_is_scale_factor_times_function_at_the_output_steps() {
    let mut m = sdof(2.0, 50.0);
    // TRI: (0,0), (1,1), (2,0); TWO: zero before 0.5, 3 + 2(t - 0.5) on
    // [0.5, 1.5], held at 5 after 1.5. dt = 0.25 (exact in binary), 12 steps.
    function(&mut m, "TRI", vec![(0.0, 0.0), (1.0, 1.0), (2.0, 0.0)]);
    function(&mut m, "TWO", vec![(0.5, 3.0), (1.5, 5.0)]);
    function(&mut m, "ONE", vec![(0.0, 1.0), (9.0, 1.0)]);
    modal_case(&mut m, 1);
    // U1 0.01*TRI; "Y" alias 2*TWO in Y; "U3" 4*ONE and "uz" -1*ONE add in Z;
    // a pattern entry adds nothing.
    modal_history(
        &mut m,
        0.05,
        0.25,
        12,
        vec![
            ("TRI", "U1", 0.01),
            ("TWO", "Y", 2.0),
            ("ONE", "U3", 4.0),
            ("ONE", "uz", -1.0),
            ("TRI", "P", 7.0),
        ],
    );
    let trace = ground_acceleration_trace(&m, "HIST").unwrap();
    assert_eq!(trace.len(), 13);
    for (k, a) in trace.iter().enumerate() {
        let t = k as f64 * 0.25;
        let tri = if t <= 1.0 {
            t
        } else if t <= 2.0 {
            2.0 - t
        } else {
            0.0
        };
        let two = if t < 0.5 {
            0.0
        } else if t <= 1.5 {
            3.0 + 2.0 * (t - 0.5)
        } else {
            5.0
        };
        assert!((a[0] - 0.01 * tri).abs() < 1e-15, "ax[{k}] {}", a[0]);
        assert!((a[1] - 2.0 * two).abs() < 1e-15, "ay[{k}] {}", a[1]);
        assert!((a[2] - 3.0).abs() < 1e-15, "az[{k}] {}", a[2]);
    }
    // Spot values: t = 0.5 -> 0.005; t = 1.5 -> TWO = 5 -> y = 10.
    assert!((trace[2][0] - 0.005).abs() < 1e-15);
    assert!((trace[6][1] - 10.0).abs() < 1e-15);
}

#[test]
fn trace_refuses_what_it_cannot_define() {
    let mut m = sdof(2.0, 50.0);
    function(&mut m, "ONE", vec![(0.0, 1.0), (9.0, 1.0)]);
    modal_case(&mut m, 1);
    modal_history(&mut m, 0.05, 0.1, 5, vec![("ONE", "UX", 1.0)]);
    assert!(ground_acceleration_trace(&m, "HIST").is_ok());
    // Missing case.
    let e = ground_acceleration_trace(&m, "NOPE")
        .unwrap_err()
        .to_string();
    assert!(e.contains("NOPE"), "{e}");
    // A modal case is not a history case.
    let e = ground_acceleration_trace(&m, "MODAL")
        .unwrap_err()
        .to_string();
    assert!(e.contains("MODAL"), "{e}");
    // Direct integration: no modal case.
    let mut direct = m.clone();
    direct.load_cases["HIST"].modal_case = None;
    let e = ground_acceleration_trace(&direct, "HIST")
        .unwrap_err()
        .to_string();
    assert!(e.contains("modal"), "{e}");
    // Missing function.
    let mut missing = m.clone();
    missing.functions.clear();
    let e = ground_acceleration_trace(&missing, "HIST")
        .unwrap_err()
        .to_string();
    assert!(e.contains("ONE"), "{e}");
}

// ---------------------------------------------------------------------------
// Damped and undamped single mass
// ---------------------------------------------------------------------------

/// m = 2, k = 50: omega = 5, ground step a0 = 1 held, dt 0.02, 500 steps.
fn closed_form_model(xi: f64) -> SapModel {
    let mut m = sdof(2.0, 50.0);
    function(&mut m, "STEP", vec![(0.0, 1.0), (10.0, 1.0)]);
    modal_case(&mut m, 1);
    modal_history(&mut m, xi, 0.02, 500, vec![("STEP", "UX", 1.0)]);
    analyze_model(&mut m).unwrap();
    m
}

#[test]
fn damped_single_mass_absolute_acceleration_matches_the_closed_form() {
    // Closed form (Chopra, Dynamics of Structures, ch. 5, step of base
    // acceleration): with omega = 5, xi = 0.05, s = sqrt(1 - xi^2), wd = omega s,
    //   u  = -(a0/w^2)[1 - e^{-xi w t}(cos wd t + xi/s sin wd t)]
    //   du = -(a0/w^2)(w/s) e^{-xi w t} sin wd t
    //   a_abs = -w^2 u - 2 xi w du      (from u'' + 2 xi w u' + w^2 u = -a0).
    let (w, xi, a0) = (5.0f64, 0.05f64, 1.0f64);
    let s = (1.0 - xi * xi).sqrt();
    let wd = w * s;
    let m = closed_form_model(xi);
    let trace = ground_acceleration_trace(&m, "HIST").unwrap();
    let rel = rel_accel(&m, "J");
    let time = &m.case_results["HIST"].time_steps;
    assert_eq!(trace.len(), 501);
    assert_eq!(rel.len(), 501);
    let mut abs_expected = Vec::new();
    let mut u_peak = 0.0f64;
    for &t in time {
        let e = (-xi * w * t).exp();
        let u = -(a0 / (w * w)) * (1.0 - e * ((wd * t).cos() + xi / s * (wd * t).sin()));
        let du = -(a0 / (w * w)) * (w / s) * e * (wd * t).sin();
        u_peak = u_peak.max(u.abs());
        abs_expected.push(-w * w * u - 2.0 * xi * w * du);
    }
    // Peak |u| = (a0/w^2)(1 + exp(-pi xi/s)) = 0.04 * 1.854... = 0.0741787 (the
    // peak lies at t = pi/wd, not an output time, hence the slack).
    assert!((u_peak - 0.0741787).abs() < 1e-4, "{u_peak}");
    let peak = abs_expected.iter().fold(0.0f64, |a, b| a.max(b.abs()));
    for k in 0..=500 {
        let abs = rel[k][0] + trace[k][0];
        assert!(
            (abs - abs_expected[k]).abs() <= 1e-9 * peak,
            "step {k}: {abs} vs {}",
            abs_expected[k]
        );
    }
}

#[test]
fn undamped_single_mass_absolute_acceleration_is_minus_omega_squared_u() {
    let m = closed_form_model(0.0);
    let trace = ground_acceleration_trace(&m, "HIST").unwrap();
    let rel = rel_accel(&m, "J");
    let u = &m.case_results["HIST"].time_history_disp["J"];
    let peak = (0..=500).fold(0.0f64, |a, k| a.max((25.0 * u[k][0]).abs()));
    for k in 0..=500 {
        let abs = rel[k][0] + trace[k][0];
        assert!((abs + 25.0 * u[k][0]).abs() <= 1e-9 * peak, "step {k}");
    }
}

// ---------------------------------------------------------------------------
// Two masses, all modes
// ---------------------------------------------------------------------------

#[test]
fn two_mass_chain_with_all_modes_satisfies_m_abs_plus_k_u_equals_zero() {
    // J1 (m1) with a ground spring k1, joined to J2 (m2) by a spring k2.
    // Undamped, all modes: M(a_rel + a_g) + K u = 0 with
    // K = [[k1 + k2, -k2], [-k2, k2]].
    let (m1, m2, k1, k2) = (2.0, 1.5, 300.0, 200.0);
    let mut m = bare_model([true, false, false, false, false, false]);
    m.add_point("J1", 10.0, 0.0, 0.0);
    m.add_point("J2", 20.0, 0.0, 0.0);
    m.points["J1"].mass[0] = m1;
    m.points["J2"].mass[0] = m2;
    m.points["J1"].spring[0] = k1;
    let mut prop = LinkProp::default();
    prop.name = "LK".into();
    prop.ke[0] = k2;
    m.link_props.insert("LK".into(), prop);
    m.links.insert(
        "L".into(),
        LinkObj {
            name: "L".into(),
            point_i: "J1".into(),
            point_j: Some("J2".into()),
            prop: "LK".into(),
            angle_deg: 0.0,
            selected: false,
        },
    );
    function(&mut m, "TRI", vec![(0.0, 0.0), (0.3, 1.0), (0.9, -0.5)]);
    modal_case(&mut m, 2);
    modal_history(&mut m, 0.0, 0.05, 100, vec![("TRI", "UX", 3.0)]);
    analyze_model(&mut m).unwrap();
    let trace = ground_acceleration_trace(&m, "HIST").unwrap();
    let r1 = rel_accel(&m, "J1");
    let r2 = rel_accel(&m, "J2");
    let u1 = &m.case_results["HIST"].time_history_disp["J1"];
    let u2 = &m.case_results["HIST"].time_history_disp["J2"];
    // Force scale: the largest inertia term m1 * 3 * 1 (|TRI| <= 1).
    let scale = m1 * 3.0;
    for k in 0..=100 {
        let a1 = r1[k][0] + trace[k][0];
        let a2 = r2[k][0] + trace[k][0];
        let f1 = m1 * a1 + (k1 + k2) * u1[k][0] - k2 * u2[k][0];
        let f2 = m2 * a2 - k2 * u1[k][0] + k2 * u2[k][0];
        assert!(
            f1.abs() <= 1e-9 * scale && f2.abs() <= 1e-9 * scale,
            "{k}: {f1} {f2}"
        );
    }
}

// ---------------------------------------------------------------------------
// Rotated joint
// ---------------------------------------------------------------------------

/// One joint with the local axes turned 30 degrees about Z, isotropic mass
/// m = 2 and spring k = 50 in the three translations (omega = 5), ground
/// acceleration (1.0, 0.5, 0)·STEP in GLOBAL X and Y.
fn rotated_model() -> SapModel {
    let mut m = bare_model([true, true, true, false, false, false]);
    m.add_point("J", 0.0, 0.0, 0.0);
    m.points["J"].mass = [2.0, 2.0, 2.0, 0.0, 0.0, 0.0];
    m.points["J"].spring = [50.0, 50.0, 50.0, 0.0, 0.0, 0.0];
    m.points["J"].local_axes_deg = [30.0, 0.0, 0.0];
    function(&mut m, "STEP", vec![(0.0, 1.0), (10.0, 1.0)]);
    modal_case(&mut m, 3);
    modal_history(
        &mut m,
        0.0,
        0.02,
        200,
        vec![("STEP", "UX", 1.0), ("STEP", "UY", 0.5)],
    );
    analyze_model(&mut m).unwrap();
    m
}

#[test]
fn rotated_joint_stores_local_components_and_the_trace_is_global() {
    // R = Rz(30): j1 = (cos30, sin30, 0), j2 = (-sin30, cos30, 0), v_g = R v_l,
    // so v_l = R^T v_g: (v_l1, v_l2) = (c vx + s vy, -s vx + c vy).
    // Isotropic undamped system, global ground a_g = (1, 0.5, 0) STEP:
    //   u_g = -(1 - cos wt)/w^2 (1, 0.5, 0),  a_abs_g = -w^2 u_g.
    let (c, s) = (30f64.to_radians().cos(), 30f64.to_radians().sin());
    let w = 5.0f64;
    let m = rotated_model();
    let r = sap_solver::frame::joint_local_to_global_matrix(&m.points["J"]);
    // The documented orientation: columns are the local axes.
    assert!((r[0][0] - c).abs() < 1e-15 && (r[1][0] - s).abs() < 1e-15);
    assert!((r[0][1] + s).abs() < 1e-15 && (r[1][1] - c).abs() < 1e-15);
    let trace = ground_acceleration_trace(&m, "HIST").unwrap();
    let rel = rel_accel(&m, "J");
    let u = &m.case_results["HIST"].time_history_disp["J"];
    let time = &m.case_results["HIST"].time_steps;
    for (k, &t) in time.iter().enumerate() {
        // The ground trace is global.
        assert!((trace[k][0] - 1.0).abs() < 1e-15 && (trace[k][1] - 0.5).abs() < 1e-15);
        let g = -(1.0 - (w * t).cos()) / (w * w);
        let (ugx, ugy) = (g, 0.5 * g);
        let abs_g = (-w * w * ugx, -w * w * ugy);
        // Displacement in local axes.
        assert!((u[k][0] - (c * ugx + s * ugy)).abs() < 1e-12, "u1 {k}");
        assert!((u[k][1] - (-s * ugx + c * ugy)).abs() < 1e-12, "u2 {k}");
        // Absolute acceleration in local axes = relative(local) + R^T a_g.
        let ag_l = (
            c * trace[k][0] + s * trace[k][1],
            -s * trace[k][0] + c * trace[k][1],
        );
        let abs_l = (rel[k][0] + ag_l.0, rel[k][1] + ag_l.1);
        assert!(
            (abs_l.0 - (c * abs_g.0 + s * abs_g.1)).abs() < 1e-12,
            "a1 {k}"
        );
        assert!(
            (abs_l.1 - (-s * abs_g.0 + c * abs_g.1)).abs() < 1e-12,
            "a2 {k}"
        );
        assert!(rel[k][2].abs() < 1e-12);
    }
}

// ---------------------------------------------------------------------------
// Restrained joint
// ---------------------------------------------------------------------------

#[test]
fn restrained_joint_moves_with_the_ground() {
    let mut m = sdof(2.0, 50.0);
    m.add_point("B", 5.0, 0.0, 0.0);
    m.points["B"].restraint[0] = true;
    m.points["B"].mass[0] = 1.0;
    function(&mut m, "STEP", vec![(0.0, 1.0), (10.0, 1.0)]);
    modal_case(&mut m, 1);
    modal_history(&mut m, 0.05, 0.02, 50, vec![("STEP", "UX", 1.5)]);
    analyze_model(&mut m).unwrap();
    let trace = ground_acceleration_trace(&m, "HIST").unwrap();
    let rel = rel_accel(&m, "B");
    for k in 0..=50 {
        // The relative acceleration of a restrained joint is zero, so its
        // absolute acceleration is the ground acceleration, here 1.5.
        assert!(rel[k][0].abs() < 1e-15, "{k}");
        assert!((rel[k][0] + trace[k][0] - 1.5).abs() < 1e-15, "{k}");
    }
}
