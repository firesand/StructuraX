//! Mode-superposition time history (`LinearHistory` with `modal_case`).
//!
//! Every expected value is a closed form evaluated in the test (CSI Analysis
//! Reference, "Linear Time-History Analysis"; Chopra, Dynamics of Structures,
//! ch. 4-5) or a published value typed with its source. Units are consistent
//! (kip, in, s) unless noted.

use sap_model::*;
use sap_solver::analyze_model;
use std::path::PathBuf;

// ---------------------------------------------------------------------------
// Model helpers
// ---------------------------------------------------------------------------

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

fn force(m: &mut SapModel, joint: &str, f: [f64; 6]) {
    m.points[joint].loads_force.push(PointLoadForce {
        load_pat: "P".into(),
        csys: "GLOBAL".into(),
        force: f,
    });
}

/// One joint with mass `mass` and a spring `k` to ground in X (U1 only).
fn sdof(mass: f64, k: f64) -> SapModel {
    let mut m = bare_model([true, false, false, false, false, false]);
    m.add_point("J", 0.0, 0.0, 0.0);
    m.points["J"].mass[0] = mass;
    m.points["J"].spring[0] = k;
    m
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

fn assert_close_to_peak(actual: &[f64], expected: &[f64], tol: f64, what: &str) {
    assert_eq!(actual.len(), expected.len(), "{what}: length");
    let peak = expected.iter().fold(0.0f64, |a, b| a.max(b.abs()));
    assert!(peak > 0.0, "{what}: expected response is zero");
    for (i, (a, e)) in actual.iter().zip(expected).enumerate() {
        assert!(
            (a - e).abs() <= tol * peak,
            "{what}[{i}]: {a} vs {e} (peak {peak}, tol {tol})"
        );
    }
}

fn trace(m: &SapModel, joint: &str, dof: usize) -> Vec<f64> {
    m.case_results["HIST"].time_history_disp[joint]
        .iter()
        .map(|u| u[dof])
        .collect()
}

// ---------------------------------------------------------------------------
// Closed forms
// ---------------------------------------------------------------------------

/// Response of an SDOF oscillator (stiffness 1 factored out: multiply by 1/k)
/// to the force `t` for `t >= 0` from rest:
/// `t − 2ξ/ω + e^{−ξωt}[(2ξ/ω) cos ωd t − ((1 − 2ξ²)/ωd) sin ωd t]`
/// (the particular solution of `ü + 2ξωu̇ + ω²u = ω² t` plus the free
/// vibration that makes `u(0) = u̇(0) = 0`).
fn ramp_unit(t: f64, omega: f64, xi: f64) -> f64 {
    if t <= 0.0 {
        return 0.0;
    }
    let wd = omega * (1.0 - xi * xi).sqrt();
    t - 2.0 * xi / omega
        + (-xi * omega * t).exp()
            * (2.0 * xi / omega * (wd * t).cos() - (1.0 - 2.0 * xi * xi) / wd * (wd * t).sin())
}

/// Unit step response `1 − e^{−ξωt}(cos ωd t + (ξω/ωd) sin ωd t)` (times F/k).
fn step_unit(t: f64, omega: f64, xi: f64) -> f64 {
    if t < 0.0 {
        return 0.0;
    }
    let wd = omega * (1.0 - xi * xi).sqrt();
    1.0 - (-xi * omega * t).exp() * ((wd * t).cos() + xi * omega / wd * (wd * t).sin())
}

// ---------------------------------------------------------------------------
// 1. Damped SDOF under a ramp-then-constant force
// ---------------------------------------------------------------------------

#[test]
fn damped_sdof_ramp_then_constant_force_matches_the_closed_form() {
    // m = 2 kip s²/in, k = 8 kip/in: ω = 2 rad/s. F0 = 3 kip is reached at
    // t_r = 1.5 s and held (the function is constant after its last point).
    let (mass, k, f0, tr, xi) = (2.0, 8.0, 3.0, 1.5, 0.05);
    let omega = f64::sqrt(k / mass);
    let mut m = sdof(mass, k);
    force(&mut m, "J", [f0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    function(&mut m, "RAMP", vec![(0.0, 0.0), (tr, 1.0)]);
    modal_case(&mut m, 1);
    modal_history(&mut m, xi, 0.1, 100, vec![("RAMP", "P", 1.0)]);
    analyze_model(&mut m).unwrap();

    let r = &m.case_results["HIST"];
    assert_eq!(r.time_steps.len(), 101);
    for (i, t) in r.time_steps.iter().enumerate() {
        assert!((t - i as f64 * 0.1).abs() < 1e-12);
    }
    // F(t) = (F0/tr)[t − (t − tr)H(t − tr)], so u = (F0/(k tr))[U(t) − U(t − tr)].
    let expected: Vec<f64> = r
        .time_steps
        .iter()
        .map(|&t| f0 / (k * tr) * (ramp_unit(t, omega, xi) - ramp_unit(t - tr, omega, xi)))
        .collect();
    assert_close_to_peak(&trace(&m, "J", 0), &expected, 1e-9, "u(t)");
}

#[test]
fn function_before_its_first_point_is_zero() {
    // A step that starts at t0 = 0.55 s (off the output grid) and at 0.5 s (on it).
    let (mass, k, xi) = (2.0, 8.0, 0.05);
    let omega = f64::sqrt(k / mass);
    for t0 in [0.5, 0.55] {
        let mut m = sdof(mass, k);
        force(&mut m, "J", [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        function(&mut m, "LATE", vec![(t0, 1.0), (100.0, 1.0)]);
        modal_case(&mut m, 1);
        modal_history(&mut m, xi, 0.1, 80, vec![("LATE", "P", 1.0)]);
        analyze_model(&mut m).unwrap();
        let r = &m.case_results["HIST"];
        let expected: Vec<f64> = r
            .time_steps
            .iter()
            .map(|&t| step_unit(t - t0, omega, xi) / k)
            .collect();
        assert_close_to_peak(&trace(&m, "J", 0), &expected, 1e-9, &format!("t0 = {t0}"));
    }
}

// ---------------------------------------------------------------------------
// 2. Two degrees of freedom, undamped step
// ---------------------------------------------------------------------------

#[test]
fn two_dof_undamped_step_matches_the_modal_closed_form() {
    // J1 (m1) has a ground spring k1 and is joined to J2 (m2) by a link k2.
    let (m1, m2, k1, k2) = (2.0, 1.5, 300.0, 200.0);
    let (f1, f2) = (2.0, 1.0);
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
    force(&mut m, "J1", [f1, 0.0, 0.0, 0.0, 0.0, 0.0]);
    force(&mut m, "J2", [f2, 0.0, 0.0, 0.0, 0.0, 0.0]);
    function(&mut m, "STEP", vec![(0.0, 1.0), (100.0, 1.0)]);
    modal_case(&mut m, 2);
    modal_history(&mut m, 0.0, 0.05, 200, vec![("STEP", "P", 1.0)]);
    analyze_model(&mut m).unwrap();

    // Eigenproblem K φ = ω² M φ solved here: with y = M^{1/2}φ it is the
    // symmetric 2x2 problem S y = λ y, S = M^{-1/2} K M^{1/2}^{-1}.
    let a = (k1 + k2) / m1;
    let c = k2 / m2;
    let b = -k2 / (m1 * m2).sqrt();
    let mean = 0.5 * (a + c);
    let radius = ((0.5 * (a - c)).powi(2) + b * b).sqrt();
    let modes: Vec<(f64, [f64; 2])> = [mean - radius, mean + radius]
        .into_iter()
        .map(|lambda| {
            let (y1, y2) = (b, lambda - a);
            let norm = (y1 * y1 + y2 * y2).sqrt();
            (lambda, [y1 / norm / m1.sqrt(), y2 / norm / m2.sqrt()])
        })
        .collect();
    let r = &m.case_results["HIST"];
    let mut expected1 = Vec::new();
    let mut expected2 = Vec::new();
    for &t in &r.time_steps {
        let (mut u1, mut u2) = (0.0, 0.0);
        for (lambda, phi) in &modes {
            let gamma = phi[0] * f1 + phi[1] * f2;
            let w = lambda.sqrt();
            let q = gamma / lambda * (1.0 - (w * t).cos());
            u1 += phi[0] * q;
            u2 += phi[1] * q;
        }
        expected1.push(u1);
        expected2.push(u2);
    }
    assert_close_to_peak(&trace(&m, "J1", 0), &expected1, 1e-9, "u1(t)");
    assert_close_to_peak(&trace(&m, "J2", 0), &expected2, 1e-9, "u2(t)");
}

// ---------------------------------------------------------------------------
// 3. Function step and output step do not coincide
// ---------------------------------------------------------------------------

#[test]
fn function_samples_every_0_03_s_with_output_every_0_02_s() {
    // m = 1, k = 400: ω = 20 rad/s. A triangular pulse sampled every 0.03 s:
    // up from 0 to 1 over 0.09 s, down to 0 at 0.21 s. Slope changes at
    // 0, 0.09 and 0.21 s (none of the last two is an output time).
    let (mass, k, f0, xi) = (1.0, 400.0, 5.0, 0.02);
    let omega = f64::sqrt(k / mass);
    let values = [0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0, 0.75, 0.5, 0.25, 0.0];
    let samples: Vec<(f64, f64)> = values
        .iter()
        .enumerate()
        .map(|(i, v)| (i as f64 * 0.03, *v))
        .collect();
    let mut m = sdof(mass, k);
    force(&mut m, "J", [f0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    function(&mut m, "PULSE", samples);
    modal_case(&mut m, 1);
    modal_history(&mut m, xi, 0.02, 60, vec![("PULSE", "P", 1.0)]);
    analyze_model(&mut m).unwrap();

    let s_up = 1.0 / 0.09;
    let s_down = -0.25 / 0.03;
    let changes = [(0.0, s_up), (0.09, s_down - s_up), (0.21, -s_down)];
    let r = &m.case_results["HIST"];
    let expected: Vec<f64> = r
        .time_steps
        .iter()
        .map(|&t| {
            changes
                .iter()
                .map(|(tj, ds)| ds * ramp_unit(t - tj, omega, xi))
                .sum::<f64>()
                * f0
                / k
        })
        .collect();
    assert_close_to_peak(&trace(&m, "J", 0), &expected, 1e-9, "pulse response");
}

// ---------------------------------------------------------------------------
// 4. Ground acceleration, relative displacement and relative acceleration
// ---------------------------------------------------------------------------

#[test]
fn ground_acceleration_step_gives_relative_displacement_and_acceleration() {
    // ü + 2ξωu̇ + ω²u = −a_g with a_g = 1 (unit of acceleration) for t >= 0.
    let (mass, k, xi): (f64, f64, f64) = (2.0, 50.0, 0.1);
    let omega = f64::sqrt(k / mass);
    let wd = omega * (1.0 - xi * xi).sqrt();
    let mut m = sdof(mass, k);
    function(&mut m, "AG", vec![(0.0, 1.0), (100.0, 1.0)]);
    modal_case(&mut m, 1);
    modal_history(&mut m, xi, 0.02, 150, vec![("AG", "UX", 1.0)]);
    analyze_model(&mut m).unwrap();

    let r = &m.case_results["HIST"];
    let u_exact = |t: f64| -step_unit(t, omega, xi) / (omega * omega);
    let v_exact = |t: f64| -(-xi * omega * t).exp() * (wd * t).sin() / wd;
    let expected_u: Vec<f64> = r.time_steps.iter().map(|&t| u_exact(t)).collect();
    assert_close_to_peak(&trace(&m, "J", 0), &expected_u, 1e-9, "u(t)");
    let expected_a: Vec<f64> = r
        .time_steps
        .iter()
        .map(|&t| -omega * omega * u_exact(t) - 2.0 * xi * omega * v_exact(t) - 1.0)
        .collect();
    let accel: Vec<f64> = r.time_history_accel["J"].iter().map(|a| a[0]).collect();
    assert_close_to_peak(&accel, &expected_a, 1e-9, "relative acceleration");
    // At t = 0 the structure is at rest, so the relative acceleration is −a_g.
    assert!((accel[0] + 1.0).abs() < 1e-12);
    // Other degrees of freedom stay at zero.
    assert!(r.time_history_disp["J"].iter().all(|u| u[1..] == [0.0; 5]));
}

// ---------------------------------------------------------------------------
// 5. Snapshot results recovered at the peak of a cantilever column
// ---------------------------------------------------------------------------

#[test]
fn cantilever_column_snapshot_forces_follow_the_static_stiffness_relations() {
    // 100 in column, flexural only (no shear area, axial stiffness x100000),
    // no self mass; tip mass 0.5 and a tip force F. The only mass DOF is the
    // tip translation, so the mode is the static deflection shape.
    let (e, i_x, len, mass, f, xi) = (29000.0, 110.0, 100.0, 0.5, 2.0, 0.3);
    let build = |dynamic: bool| {
        let mut m = bare_model([true, false, true, false, true, false]);
        let mut material = Material::default_steel_kip_in();
        material.name = "S".into();
        material.e = e;
        material.unit_mass = 0.0;
        material.unit_weight = 0.0;
        m.materials.insert("S".into(), material);
        let section = FrameSection {
            name: "C".into(),
            material: "S".into(),
            prop_type: FramePropType::General,
            area: 10.0,
            i33: i_x,
            i22: i_x,
            torsional_j: 2.0 * i_x,
            as2: 0.0,
            as3: 0.0,
            modifiers: [100_000.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            ..FrameSection::default()
        };
        m.frame_sections.insert("C".into(), section);
        m.add_point("A", 0.0, 0.0, 0.0);
        m.add_point("B", 0.0, 0.0, len);
        m.points["A"].restraint = [true; 6];
        m.points["B"].mass[0] = mass;
        m.add_frame("F", "A", "B", "C");
        force(&mut m, "B", [f, 0.0, 0.0, 0.0, 0.0, 0.0]);
        m.load_cases
            .insert("S".into(), LoadCase::linear_static("S", "P", 1.0));
        if dynamic {
            function(&mut m, "STEP", vec![(0.0, 1.0), (100.0, 1.0)]);
            modal_case(&mut m, 1);
            // ω = sqrt(3EI/(L³ m)) = 4.37 rad/s, T = 1.44 s; 20 s of response
            // leaves e^(−ξωt) = e^(−26) of the transient.
            modal_history(&mut m, xi, 0.02, 1000, vec![("STEP", "P", 1.0)]);
        }
        analyze_model(&mut m).unwrap();
        m
    };
    let m = build(true);
    let stat = build(false);
    let k_tip = 3.0 * e * i_x / len.powi(3);
    let r = &m.case_results["HIST"];

    // Quasi-static end state: u = F L³/(3EI) (ξ = 0.3 damps the transient).
    let trace_x = trace(&m, "B", 0);
    let u_static = f / k_tip;
    assert!(
        (trace_x.last().unwrap() - u_static).abs() < 1e-6 * u_static,
        "{} vs {u_static}",
        trace_x.last().unwrap()
    );
    assert!((stat.case_results["S"].joint_displacements["B"][0] - u_static).abs() < 1e-9);

    // The snapshot is the step of largest displacement norm.
    let (peak_index, peak) = trace_x.iter().enumerate().fold((0, 0.0f64), |acc, (i, u)| {
        if u.abs() >= acc.1 {
            (i, u.abs())
        } else {
            acc
        }
    });
    let _ = peak_index;
    let u_snapshot = r.joint_displacements["B"][0];
    assert!((u_snapshot.abs() - peak).abs() < 1e-12 * peak);
    assert!(peak > u_static, "step response overshoots the static value");

    // Elastic recovery: the base moment and reaction scale with the tip
    // displacement exactly like the static case (ratio from the static run,
    // which is solved by the existing static path).
    let sr = &stat.case_results["S"];
    let base = |res: &CaseResults| {
        let s = &res.frame_forces["F"][0];
        (s.m2.abs().max(s.m3.abs()), s.v2.abs().max(s.v3.abs()))
    };
    let (m_static, v_static) = base(sr);
    assert!((m_static - f * len).abs() < 1e-8 * f * len);
    let (m_dyn, v_dyn) = base(r);
    let ratio = u_snapshot / u_static;
    assert!((m_dyn - ratio.abs() * m_static).abs() < 1e-9 * m_static * ratio.abs());
    assert!((v_dyn - ratio.abs() * v_static).abs() < 1e-9 * v_static * ratio.abs());
    // Reaction: R = K u at the base (no mass or load there) = −k u.
    let rx = r.joint_reactions["A"][0];
    assert!((rx + k_tip * u_snapshot).abs() < 1e-9 * k_tip * u_snapshot.abs());
    assert!((r.base_reactions[0] - rx).abs() < 1e-12 * rx.abs());
    // Acceleration trace has the same length as the displacement trace.
    assert_eq!(r.time_history_accel["B"].len(), r.time_steps.len());
}

#[test]
fn traces_of_an_automatically_subdivided_frame_name_only_model_joints() {
    let mut m = bare_model([true, false, true, false, true, false]);
    let mut material = Material::default_steel_kip_in();
    material.name = "S".into();
    material.unit_mass = 0.0;
    material.unit_weight = 0.0;
    m.materials.insert("S".into(), material);
    m.frame_sections.insert(
        "C".into(),
        FrameSection {
            name: "C".into(),
            material: "S".into(),
            prop_type: FramePropType::General,
            area: 10.0,
            i33: 110.0,
            i22: 110.0,
            as2: 0.0,
            as3: 0.0,
            ..FrameSection::default()
        },
    );
    m.add_point("A", 0.0, 0.0, 0.0);
    m.add_point("B", 0.0, 0.0, 100.0);
    m.points["A"].restraint = [true; 6];
    m.points["B"].mass[0] = 0.5;
    m.add_frame("F", "A", "B", "C");
    m.frames["F"].auto_mesh_segments = 3;
    force(&mut m, "B", [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    function(&mut m, "STEP", vec![(0.0, 1.0), (100.0, 1.0)]);
    modal_case(&mut m, 1);
    modal_history(&mut m, 0.05, 0.1, 20, vec![("STEP", "P", 1.0)]);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["HIST"];
    let joints: Vec<&String> = r.time_history_disp.keys().collect();
    assert_eq!(joints, ["A", "B"]);
    assert_eq!(r.time_history_accel.keys().collect::<Vec<_>>(), joints);
    assert_eq!(r.frame_forces.keys().collect::<Vec<_>>(), ["F"]);
}

// ---------------------------------------------------------------------------
// Dependency order
// ---------------------------------------------------------------------------

#[test]
fn modal_case_is_analysed_before_a_history_case_listed_ahead_of_it() {
    let (mass, k, xi) = (2.0, 8.0, 0.05);
    let mut m = sdof(mass, k);
    force(&mut m, "J", [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    function(&mut m, "STEP", vec![(0.0, 1.0), (100.0, 1.0)]);
    modal_history(&mut m, xi, 0.1, 10, vec![("STEP", "P", 1.0)]);
    modal_case(&mut m, 1); // inserted after HIST
    assert_eq!(m.load_cases.keys().next().unwrap(), "HIST");
    analyze_model(&mut m).unwrap();
    assert_eq!(m.case_results["HIST"].time_steps.len(), 11);
}

// ---------------------------------------------------------------------------
// Refusals
// ---------------------------------------------------------------------------

fn valid() -> SapModel {
    let mut m = sdof(2.0, 8.0);
    force(&mut m, "J", [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    function(&mut m, "STEP", vec![(0.0, 1.0), (100.0, 1.0)]);
    modal_case(&mut m, 1);
    modal_history(&mut m, 0.05, 0.1, 10, vec![("STEP", "P", 1.0)]);
    m
}

fn refused(mut m: SapModel, needle: &str) {
    let error = analyze_model(&mut m)
        .expect_err("must be refused")
        .to_string();
    assert!(error.contains(needle), "'{error}' lacks '{needle}'");
    assert!(m.case_results.is_empty(), "results survive a refusal");
    assert!(!m.is_locked);
}

#[test]
fn the_valid_baseline_analyses() {
    let mut m = valid();
    analyze_model(&mut m).unwrap();
}

#[test]
fn rayleigh_damping_together_with_a_modal_case_is_refused() {
    let mut m = valid();
    m.load_cases["HIST"].rayleigh_damping = (0.1, 0.0);
    refused(m, "Rayleigh");
}

#[test]
fn an_initial_case_together_with_a_modal_case_is_refused() {
    let mut m = valid();
    m.load_cases["HIST"].initial_case = Some("MODAL".into());
    refused(m, "starts at rest");
}

#[test]
fn nonlinear_and_direct_history_cases_cannot_name_a_modal_case() {
    for case_type in [
        LoadCaseType::NonlinearHistory,
        LoadCaseType::NonlinearDynamic,
        LoadCaseType::LinearDynamic,
        LoadCaseType::LinearStatic,
    ] {
        let mut m = valid();
        m.load_cases["HIST"].case_type = case_type;
        refused(m, "only a LinearHistory case");
    }
}

#[test]
fn the_modal_case_must_exist_be_modal_run_and_have_modes() {
    let mut m = valid();
    m.load_cases["HIST"].modal_case = Some("NOPE".into());
    refused(m, "missing case 'NOPE'");

    let mut m = valid();
    m.load_cases
        .insert("S".into(), LoadCase::linear_static("S", "P", 1.0));
    m.load_cases["HIST"].modal_case = Some("S".into());
    refused(m, "not a Modal case");

    let mut m = valid();
    m.load_cases["MODAL"].run_flag = false;
    refused(m, "not flagged to run");

    let mut m = valid();
    m.load_cases["MODAL"].max_modes = 0;
    refused(m, "requests no modes");

    // A history case cannot be its own modal case.
    let mut m = valid();
    m.load_cases["HIST"].modal_case = Some("HIST".into());
    refused(m, "not a Modal case");
}

#[test]
fn modal_damping_must_be_in_zero_to_one() {
    for xi in [1.0, 1.5, -0.01, f64::NAN, f64::INFINITY] {
        let mut m = valid();
        m.load_cases["HIST"].modal_damping = xi;
        refused(m, "0 <= ratio < 1");
    }
    // Zero damping is allowed.
    let mut m = valid();
    m.load_cases["HIST"].modal_damping = 0.0;
    analyze_model(&mut m).unwrap();
}

#[test]
fn viscous_link_damping_is_refused_for_modal_superposition() {
    let mut m = valid();
    m.add_point("K", 5.0, 0.0, 0.0);
    m.points["K"].mass[0] = 1.0;
    m.points["K"].spring[0] = 4.0;
    let mut prop = LinkProp::default();
    prop.name = "D".into();
    prop.ce[0] = 0.5;
    m.link_props.insert("D".into(), prop);
    m.links.insert(
        "L".into(),
        LinkObj {
            name: "L".into(),
            point_i: "J".into(),
            point_j: Some("K".into()),
            prop: "D".into(),
            angle_deg: 0.0,
            selected: false,
        },
    );
    refused(m, "viscous damping");
}

#[test]
fn existing_history_validation_still_applies_to_modal_history() {
    // Self weight in a history pattern.
    let mut m = valid();
    m.load_patterns["P"].self_wt_multiplier = 1.0;
    refused(m, "self-weight");
    // Missing function.
    let mut m = valid();
    m.load_cases["HIST"].history_loads[0].0 = "NOFUNC".into();
    refused(m, "missing function");
    // Missing pattern.
    let mut m = valid();
    m.load_cases["HIST"].history_loads[0].1 = "NOPAT".into();
    refused(m, "missing history load pattern");
    // Time step.
    let mut m = valid();
    m.load_cases["HIST"].time_step_dt = 0.0;
    refused(m, "time step");
}

#[test]
fn direct_integration_with_modal_damping_is_still_refused_and_without_it_runs() {
    // modal_case None: today's rule is unchanged.
    let mut m = valid();
    m.load_cases["HIST"].modal_case = None;
    m.load_cases["HIST"].modal_damping = 0.05;
    refused(m, "modal damping");
    let mut m = valid();
    m.load_cases["HIST"].modal_case = None;
    m.load_cases["HIST"].modal_damping = 0.0;
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["HIST"];
    assert!(r.time_history_accel.is_empty());
}

// ---------------------------------------------------------------------------
// Opt-in vendor: CSI Example 1-022 (STRUCTURAX_VENDOR_FIXTURES)
// ---------------------------------------------------------------------------

fn vendor_dir() -> Option<PathBuf> {
    std::env::var_os("STRUCTURAX_VENDOR_FIXTURES").map(PathBuf::from)
}

/// The vendor model and the El Centro record, or `None` (test skipped).
fn example_1_022() -> Option<SapModel> {
    use sap_format::function_file::{
        read_function_file, FunctionFileOptions, ItemFormat, ValueLayout,
    };
    let dir = vendor_dir()?;
    let (sdb, record) = (dir.join("example_1_022.sdb"), dir.join("elcentro"));
    if !sdb.is_file() || !record.is_file() {
        eprintln!(
            "skipped: example_1_022.sdb or elcentro not found in {}",
            dir.display()
        );
        return None;
    }
    let mut m = sap_format::load_sdb_file(&sdb).expect("import example_1_022.sdb");
    // The file links the digitized El Centro N-S record (ground acceleration in g,
    // three (time, value) pairs per line) instead of storing it.
    let points = read_function_file(
        &record,
        &FunctionFileOptions {
            header_lines: 0,
            prefix_chars: 0,
            points_per_line: 3,
            format: ItemFormat::Free,
            layout: ValueLayout::Pairs,
        },
    )
    .expect("read elcentro");
    m.functions["ELC_TH"].points = points;
    Some(m)
}

fn close_rel(actual: f64, expected: f64, tol: f64, what: &str) {
    assert!(
        (actual - expected).abs() <= tol * expected.abs(),
        "{what}: {actual} vs {expected} (relative tolerance {tol})"
    );
}

/// CSI Software Verification Example 1-022, static lateral case LAT, as
/// imported (with its seven diaphragm constraints): printed SAP2000 and
/// independent values Ux(joint 22) = 1.45076 in, axial force in frame 1 =
/// 69.99 kip, moment in frame 1 at joint 1 = 2324.68 kip-in.
#[test]
fn vendor_example_1_022_static_lateral_case_as_imported() {
    let Some(mut m) = example_1_022() else { return };
    for name in [
        "MODAL", "SPECSRSS", "SPECCQC", "MHIST1", "DHIST1", "MHIST2", "DHIST2",
    ] {
        m.load_cases[name].run_flag = false;
    }
    analyze_model(&mut m).unwrap();
    let lat = &m.case_results["LAT"];
    // Penalty diaphragms leave about 5e-5 relative flexibility.
    close_rel(
        lat.joint_displacements["22"][0],
        1.45076,
        1e-4,
        "Ux at joint 22",
    );
    let station = &lat.frame_forces["1"][0];
    assert!(
        (station.p.abs() - 69.99).abs() < 0.01,
        "axial {}",
        station.p
    );
    close_rel(
        station.m3.abs(),
        2324.68,
        1e-4,
        "moment in frame 1 at joint 1",
    );
}

/// Example 1-022 periods and modal time history. The model is the one the
/// problem describes, with one stated change: the problem's diaphragm
/// constraints "eliminate all axial deformations in the beams", and here that is
/// modeled by an axial-area modifier of 1e6 on the beam sections instead,
/// because a Modal case with the imported penalty constraints is refused by the
/// eigen solver (see the ignored test below).
///
/// Published (problem_1_022.pdf, "Results comparison"): periods 1.2732, 0.4313,
/// 0.2420, 0.1602, 0.1190, 0.0951, 0.0795 s; participating mass ratios 0.800,
/// 0.113, 0.042, 0.021, 0.014, 0.007, 0.003; modal time history MHIST1 (5 %
/// damping, 0.02 s output, 8 s) maximum Ux at joint 22 = 5.486 in (SAP2000)
/// and 5.46 in (independent, sampled every 0.1 s).
#[test]
fn vendor_example_1_022_modal_time_history_against_the_published_values() {
    let Some(mut m) = example_1_022() else { return };
    for name in ["SPECSRSS", "SPECCQC", "DHIST1", "MHIST2", "DHIST2"] {
        m.load_cases[name].run_flag = false;
    }
    m.constraints.clear();
    for point in m.points.values_mut() {
        point.constraints.clear();
    }
    for beam in ["W24X110", "W24X130", "W24X160"] {
        m.frame_sections[beam].modifiers[0] = 1.0e6;
    }
    // MHIST1 as defined in the file: El Centro in U1 x 386.4 in/s², 0.02 s
    // output for 8 s, modal damping 5 %. Only its modal case is chosen here.
    assert_eq!(m.load_cases["MHIST1"].modal_damping, 0.05);
    assert_eq!(m.load_cases["MHIST1"].history_loads[0].2, 386.4);
    m.load_cases["MHIST1"].modal_case = Some("MODAL".into());
    analyze_model(&mut m).unwrap();

    let modal = &m.case_results["MODAL"];
    let periods = [1.2732, 0.4313, 0.2420, 0.1602, 0.1190, 0.0951, 0.0795];
    let mass = [0.800, 0.113, 0.042, 0.021, 0.014, 0.007, 0.003];
    assert_eq!(modal.modes.len(), 7);
    for (i, mode) in modal.modes.iter().enumerate() {
        // Published to four decimals / three decimals.
        assert!(
            (mode.period_sec - periods[i]).abs() < 6e-5,
            "T{}: {}",
            i + 1,
            mode.period_sec
        );
        assert!(
            (mode.modal_mass_ratios[0] - mass[i]).abs() < 6e-4,
            "mass {}",
            i + 1
        );
    }
    close_rel(
        m.case_results["LAT"].joint_displacements["22"][0],
        1.45076,
        1e-4,
        "LAT Ux at joint 22 (rigid-beam model)",
    );

    let history = &m.case_results["MHIST1"];
    assert_eq!(history.time_steps.len(), 401);
    assert!((history.time_steps[400] - 8.0).abs() < 1e-12);
    let peak = history.time_history_disp["22"]
        .iter()
        .fold(0.0f64, |a, u| a.max(u[0].abs()));
    // Within 1 % of the SAP2000 value and of the independent value.
    close_rel(peak, 5.486, 0.01, "max |Ux| joint 22 vs SAP2000 5.486 in");
    close_rel(peak, 5.46, 0.01, "max |Ux| joint 22 vs independent 5.46 in");
    // Axial force in frame 1 at the step of peak displacement (published
    // maximum over the record: 263.0 kip).
    let axial = history.frame_forces["1"][0].p.abs();
    close_rel(axial, 263.0, 0.01, "axial force in frame 1");
    // At t = 0 the structure is at rest and the record starts at 0.0108 g, so
    // the relative acceleration of a lumped mass is −0.0108 * 386.4 in/s².
    let a0 = history.time_history_accel["22"][0][0];
    close_rel(a0, -0.0108 * 386.4, 1e-9, "relative acceleration at t = 0");
}

/// The model exactly as imported, with its diaphragm constraints. The modal
/// case factorizes the penalty-constrained stiffness against the structural
/// diagonal, as the static path does; without that the constrained model was
/// reported as singular.
#[test]
fn vendor_example_1_022_as_imported_modal_history() {
    let Some(mut m) = example_1_022() else { return };
    for name in ["SPECSRSS", "SPECCQC", "DHIST1", "MHIST2", "DHIST2"] {
        m.load_cases[name].run_flag = false;
    }
    m.load_cases["MHIST1"].modal_case = Some("MODAL".into());
    analyze_model(&mut m).unwrap();
    let peak = m.case_results["MHIST1"].time_history_disp["22"]
        .iter()
        .fold(0.0f64, |a, u| a.max(u[0].abs()));
    close_rel(peak, 5.486, 0.01, "max |Ux| joint 22");
}
