use sap_api::{cSapModel, set_case_definition, set_function_definition, OAPI_ERR, OAPI_OK};
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
fn function(name: &str, history: bool) -> DynamicFunction {
    DynamicFunction {
        name: name.into(),
        is_time_history: history,
        damping_ratio: 0.05,
        points: vec![(0., 1.), (10., 1.)],
    }
}
fn history() -> LoadCase {
    let mut h = LoadCase::linear_static("H", "DEAD", 1.);
    h.case_type = LoadCaseType::LinearHistory;
    // Modal history types are integrated directly; they take no modal damping.
    h.modal_damping = 0.;
    h.loads.clear();
    h.num_time_steps = 200;
    h.time_step_dt = 0.005;
    h.history_loads = vec![("STEP".into(), "DEAD".into(), 1.)];
    h
}
#[test]
fn history_defined_by_api_roundtrips_and_matches_closed_form_response() {
    let mut a = oscillator();
    assert_eq!(
        a.functions().set_definition(function("STEP", true)),
        OAPI_OK
    );
    assert_eq!(a.load_cases().set_definition(history()), OAPI_OK);
    let loaded = sap_format::parse_s2k(&sap_format::write_s2k(&a.inner)).unwrap();
    assert_eq!(loaded.load_cases["H"], a.inner.load_cases["H"]);
    assert_eq!(loaded.functions["STEP"], a.inner.functions["STEP"]);
    a.inner = loaded;
    assert_eq!(a.analyze().run_analysis(), OAPI_OK);
    let result = &a.inner.case_results["H"];
    assert_eq!(result.time_steps.len(), 201);
    for (t, u) in result.time_steps.iter().zip(&result.time_history_disp["J"]) {
        let exact = 0.5 * (1. - (2. * t).cos());
        assert!((u[0] - exact).abs() < 1e-5, "{t}: {} vs {exact}", u[0]);
    }
}
#[test]
fn spectrum_editor_selects_real_modal_dependency_and_known_oscillator_response() {
    let mut a = oscillator();
    assert_eq!(a.load_cases().set_modal_eigen("MODES", 1), OAPI_OK);
    let mut f = function("SA", false);
    for (_, y) in &mut f.points {
        *y = 2.;
    }
    assert_eq!(a.functions().set_definition(f), OAPI_OK);
    let mut s = LoadCase::linear_static("S", "DEAD", 1.);
    s.loads.clear();
    s.case_type = LoadCaseType::ResponseSpectrum;
    s.initial_case = Some("MODES".into());
    s.spectrum_loads = vec![("SA".into(), "U1".into(), 1.)];
    assert_eq!(a.load_cases().set_definition(s), OAPI_OK);
    assert_eq!(a.analyze().run_analysis(), OAPI_OK);
    assert!((a.inner.case_results["MODES"].modes[0].period_sec - std::f64::consts::PI).abs() < 1e-9);
    assert!((a.inner.case_results["S"].joint_displacements["J"][0] - 0.5).abs() < 1e-9);
}
#[test]
fn invalid_definitions_and_reference_type_changes_are_atomic() {
    let mut a = oscillator();
    set_function_definition(&mut a.inner, function("STEP", true)).unwrap();
    set_case_definition(&mut a.inner, history()).unwrap();
    a.inner
        .case_results
        .insert("OLD".into(), Default::default());
    let before = a.inner.clone();
    let mut f = function("STEP", false);
    assert!(set_function_definition(&mut a.inner, f.clone()).is_err());
    f.name = "BAD".into();
    f.points[1].0 = 0.;
    assert!(set_function_definition(&mut a.inner, f).is_err());
    let mut c = history();
    c.time_step_dt = f64::NAN;
    assert!(set_case_definition(&mut a.inner, c).is_err());
    let mut c = history();
    c.nl_geom = NLGeomType::PDelta;
    assert!(set_case_definition(&mut a.inner, c).is_err());
    let mut c = history();
    c.history_loads[0].0 = "MISSING".into();
    assert!(set_case_definition(&mut a.inner, c).is_err());
    let mut c = LoadCase::modal_eigen("M", 2);
    c.loads = vec![LoadCaseAssignment {
        load_name: "DEAD".into(),
        scale_factor: 1.,
    }];
    assert!(set_case_definition(&mut a.inner, c).is_err());
    assert_eq!(a.load_cases().set_modal_eigen("M", 0), OAPI_ERR);
    assert_eq!(
        a.load_cases()
            .set_linear_static("S", vec![("MISSING".into(), 1.)]),
        OAPI_ERR
    );
    assert_eq!(a.inner, before);
    a.inner.is_locked = true;
    let before = a.inner.clone();
    assert_eq!(a.load_cases().set_run_flag("H", false), OAPI_ERR);
    assert!(set_function_definition(&mut a.inner, function("F", true)).is_err());
    assert_eq!(a.inner, before);
}
#[test]
fn history_editor_rejects_unsupported_pattern_sources_and_parameter_ranges() {
    let mut a = oscillator();
    set_function_definition(&mut a.inner, function("STEP", true)).unwrap();
    for method in [
        TimeIntegrationMethod::NewmarkAverage,
        TimeIntegrationMethod::NewmarkLinear,
        TimeIntegrationMethod::HilberHughesTaylor,
        TimeIntegrationMethod::WilsonTheta,
    ] {
        let mut c = history();
        c.time_integration = method;
        set_case_definition(&mut a.inner, c).unwrap();
    }
    let mut c = history();
    c.time_integration = TimeIntegrationMethod::WilsonTheta;
    c.wilson_theta = 1.0;
    assert!(set_case_definition(&mut a.inner, c).is_err());
    let mut c = history();
    c.time_integration = TimeIntegrationMethod::HilberHughesTaylor;
    c.hht_alpha = -0.4;
    assert!(set_case_definition(&mut a.inner, c).is_err());
    a.inner.load_patterns["DEAD"].self_wt_multiplier = 1.;
    let before = a.inner.clone();
    assert!(set_case_definition(&mut a.inner, history()).is_err());
    assert_eq!(a.inner, before);
}

#[test]
fn modal_history_damping_and_unmodeled_source_need_explicit_decisions() {
    use sap_api::{discard_unmodeled_source, validate_case_definition};
    let mut api = oscillator();
    assert_eq!(api.functions().set_definition(function("STEP", true)), OAPI_OK);
    // The general 0.05 default is a spectrum setting; a modal history type
    // cannot honor it and the definition is refused, not silently accepted.
    let mut h = history();
    h.modal_damping = 0.05;
    assert!(validate_case_definition(&api.inner, &h)
        .unwrap_err()
        .to_string()
        .contains("modal damping"));
    assert_eq!(api.load_cases().set_definition(h.clone()), OAPI_ERR);
    h.modal_damping = 0.;
    assert_eq!(api.load_cases().set_definition(h.clone()), OAPI_OK);
    // The direct-integration type does not use the ratio.
    h.name = "D".into();
    h.case_type = LoadCaseType::LinearDynamic;
    h.modal_damping = 0.05;
    assert_eq!(api.load_cases().set_definition(h), OAPI_OK);

    api.inner.unmodeled_source.push(UnmodeledSourceData {
        scope: UnmodeledScope::NonlinearCases,
        description: "2 frame hinge assignment(s)".into(),
    });
    api.inner
        .case_results
        .insert("OLD".into(), Default::default());
    let before = api.inner.clone();
    assert!(discard_unmodeled_source(&mut api.inner, 1).is_err());
    api.inner.is_locked = true;
    let locked = api.inner.clone();
    assert!(discard_unmodeled_source(&mut api.inner, 0).is_err());
    assert_eq!(api.inner, locked);
    api.inner.is_locked = false;
    assert_eq!(api.inner, before);
    let entry = discard_unmodeled_source(&mut api.inner, 0).unwrap();
    assert_eq!(entry.description, "2 frame hinge assignment(s)");
    assert!(api.inner.unmodeled_source.is_empty());
    // The model changed in a way that affects results.
    assert!(api.inner.case_results.is_empty());
}
