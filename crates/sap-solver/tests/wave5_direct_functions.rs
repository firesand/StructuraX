//! Wave 5 step 1e (D23): direct-integration time histories read their load
//! function with the one rule of the modal histories, zero before the first point.
//!
//! Reference: a function with points (1.0, 1.0) and (5.0, 1.0) is defined from
//! t = 1 s; before its first point it contributes no load (the rule of
//! `time_function_value`, shared with the modal histories: "zero before the first
//! point, constant after the last"). A system at rest that receives no load stays
//! at rest, so every displacement for t < 1 s is exactly zero. At the first step
//! with a load, Newmark's average-acceleration method from rest gives
//! u = p / (k + 4 m / dt^2) (Chopra, Dynamics of Structures, Table 5.4.2: the
//! effective stiffness k_hat = k + 4 m / dt^2 and p_hat = p when u, v, a = 0).

use sap_model::{
    DynamicFunction, LoadCase, LoadCaseType, PointLoadForce, SapModel, TimeIntegrationMethod,
};
use sap_solver::analyze_model;

/// One joint, UX free, mass 1, spring 4, 1 kip along X in pattern DEAD.
fn oscillator(points: Vec<(f64, f64)>) -> SapModel {
    let mut model = SapModel::default();
    model.load_cases.clear();
    model.load_patterns.get_mut("DEAD").unwrap().self_wt_multiplier = 0.0;
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
        "LATE".into(),
        DynamicFunction {
            name: "LATE".into(),
            is_time_history: true,
            damping_ratio: 0.0,
            points,
        },
    );
    let mut case = LoadCase::linear_static("HISTORY", "DEAD", 1.0);
    case.case_type = LoadCaseType::LinearDynamic;
    case.time_integration = TimeIntegrationMethod::NewmarkAverage;
    case.history_loads = vec![("LATE".into(), "DEAD".into(), 1.0)];
    case.time_step_dt = 0.01;
    case.num_time_steps = 300;
    model.load_cases.insert("HISTORY".into(), case);
    model
}

#[test]
fn r11_a_function_that_starts_at_one_second_loads_nothing_before_it() {
    let mut model = oscillator(vec![(1.0, 1.0), (5.0, 1.0)]);
    analyze_model(&mut model).unwrap();
    let result = &model.case_results["HISTORY"];
    let u = &result.time_history_disp["M"];
    assert_eq!(u.len(), 301);
    for k in 0..100 {
        let t = result.time_steps[k];
        assert!(t < 1.0);
        assert_eq!(u[k][0], 0.0, "t = {t}");
    }
    // t = 1.0 is the first point: the load is on from there (right-continuous).
    let dt = 0.01;
    let first = 1.0 / (4.0 + 4.0 / (dt * dt));
    assert!((u[100][0] - first).abs() <= 1e-12 * first, "{} against {first}", u[100][0]);
    assert!(u[200][0] != 0.0);
}

#[test]
fn a_function_that_starts_at_zero_still_loads_from_the_first_step() {
    let mut model = oscillator(vec![(0.0, 1.0), (5.0, 1.0)]);
    analyze_model(&mut model).unwrap();
    let u = &model.case_results["HISTORY"].time_history_disp["M"];
    // F(0) = F(dt) = 1 and rest: a(0) = 1 and u(dt) = 2 / (4 / dt^2 + k) (the
    // existing initial-equilibrium test of the integrator, derived from the same
    // Newmark recursion).
    let dt = 0.01;
    let expected = 2.0 / (4.0 / (dt * dt) + 4.0);
    assert!((u[1][0] - expected).abs() <= 1e-12 * expected, "{} against {expected}", u[1][0]);
}

#[test]
fn a_missing_function_is_an_error_naming_it() {
    let mut model = oscillator(vec![(1.0, 1.0), (5.0, 1.0)]);
    model.load_cases["HISTORY"].history_loads = vec![("NO_SUCH_FUNCTION".into(), "DEAD".into(), 1.0)];
    let error = analyze_model(&mut model).unwrap_err().to_string();
    assert!(error.contains("NO_SUCH_FUNCTION") && error.contains("HISTORY"), "{error}");
    assert!(model.case_results.is_empty());
}
