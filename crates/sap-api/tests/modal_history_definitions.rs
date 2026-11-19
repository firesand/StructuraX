//! Definition rules for modal (mode-superposition) time-history cases.

use sap_api::{cSapModel, set_case_definition, set_function_definition, OAPI_OK};
use sap_model::*;
use sap_units::Units;

fn oscillator() -> cSapModel {
    let mut api = cSapModel::new(Units::KipInF);
    api.inner.load_cases.clear();
    api.inner
        .load_patterns
        .get_mut("DEAD")
        .unwrap()
        .self_wt_multiplier = 0.0;
    api.inner.add_point("J", 0., 0., 0.);
    let p = api.inner.points.get_mut("J").unwrap();
    p.mass[0] = 2.;
    p.spring[0] = 8.;
    p.restraint = [false, true, true, true, true, true];
    p.loads_force.push(PointLoadForce {
        load_pat: "DEAD".into(),
        csys: "GLOBAL".into(),
        force: [4., 0., 0., 0., 0., 0.],
    });
    api
}

fn step() -> DynamicFunction {
    DynamicFunction {
        name: "STEP".into(),
        is_time_history: true,
        damping_ratio: 0.0,
        points: vec![(0., 1.), (10., 1.)],
    }
}

fn modal_history(damping: f64) -> LoadCase {
    let mut h = LoadCase::linear_static("H", "DEAD", 1.);
    h.case_type = LoadCaseType::LinearHistory;
    h.modal_damping = damping;
    h.loads.clear();
    h.num_time_steps = 200;
    h.time_step_dt = 0.005;
    h.history_loads = vec![("STEP".into(), "DEAD".into(), 1.)];
    h.modal_case = Some("MODES".into());
    h
}

fn model_with_modes() -> cSapModel {
    let mut a = oscillator();
    assert_eq!(a.load_cases().set_modal_eigen("MODES", 1), OAPI_OK);
    set_function_definition(&mut a.inner, step()).unwrap();
    a
}

#[test]
fn modal_history_with_damping_is_accepted_and_runs() {
    let mut a = model_with_modes();
    set_case_definition(&mut a.inner, modal_history(0.05)).unwrap();
    assert_eq!(a.analyze().run_analysis(), OAPI_OK);
    // ω = sqrt(8/2) = 2 rad/s. The static value is F/k = 4/8 = 0.5 in; the
    // undamped overshoot is 1.0, and 5 % damping keeps the peak below that.
    let peak = a.inner.case_results["H"].time_history_disp["J"]
        .iter()
        .fold(0.0f64, |m, u| m.max(u[0]));
    assert!(peak > 0.5 && peak < 1.0, "{peak}");
}

#[test]
fn modal_history_definition_rules() {
    let mut a = model_with_modes();
    let before = a.inner.clone();
    // Modal damping on a case without a modal case is still refused.
    let mut c = modal_history(0.05);
    c.modal_case = None;
    assert!(set_case_definition(&mut a.inner, c).is_err());
    // Missing, self-referencing and non-modal sources.
    let mut c = modal_history(0.05);
    c.modal_case = Some("NOPE".into());
    assert!(set_case_definition(&mut a.inner, c).is_err());
    let mut c = modal_history(0.05);
    c.modal_case = Some("H".into());
    assert!(set_case_definition(&mut a.inner, c).is_err());
    a.inner
        .load_cases
        .insert("S".into(), LoadCase::linear_static("S", "DEAD", 1.));
    let mut c = modal_history(0.05);
    c.modal_case = Some("S".into());
    assert!(set_case_definition(&mut a.inner, c).is_err());
    // Damping outside [0, 1), Rayleigh coefficients and nonlinear types.
    for xi in [1.0, -0.1, f64::NAN] {
        assert!(set_case_definition(&mut a.inner, modal_history(xi)).is_err());
    }
    let mut c = modal_history(0.05);
    c.rayleigh_damping = (0.1, 0.0);
    assert!(set_case_definition(&mut a.inner, c).is_err());
    let mut c = modal_history(0.05);
    c.case_type = LoadCaseType::NonlinearHistory;
    assert!(set_case_definition(&mut a.inner, c).is_err());
    let mut c = modal_history(0.05);
    c.case_type = LoadCaseType::LinearStatic;
    assert!(set_case_definition(&mut a.inner, c).is_err());
    // A disabled modal case cannot be the source.
    a.inner.load_cases["MODES"].run_flag = false;
    assert!(set_case_definition(&mut a.inner, modal_history(0.05)).is_err());
    a.inner.load_cases["MODES"].run_flag = true;
    a.inner.load_cases.shift_remove("S");
    assert_eq!(a.inner, before, "refusals leave the model unchanged");
}

#[test]
fn a_modal_case_used_by_a_history_cannot_change_type() {
    let mut a = model_with_modes();
    set_case_definition(&mut a.inner, modal_history(0.05)).unwrap();
    let mut changed = LoadCase::linear_static("MODES", "DEAD", 1.);
    changed.case_type = LoadCaseType::LinearStatic;
    assert!(set_case_definition(&mut a.inner, changed).is_err());
    assert_eq!(a.inner.load_cases["MODES"].case_type, LoadCaseType::Modal);
}
