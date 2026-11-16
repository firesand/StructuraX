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
    assert!(
        (a.inner.case_results["MODES"].modes[0].period_sec - std::f64::consts::PI).abs() < 1e-9
    );
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

// ---------------------------------------------------------------------------
// Nonlinear static pushover definitions: initial case, saved states, step
// limits, Accel and Mode loads. Each relaxation has its negative tests.
// ---------------------------------------------------------------------------

fn nl_static(name: &str) -> LoadCase {
    let mut c = LoadCase::linear_static(name, "DEAD", 1.);
    c.case_type = LoadCaseType::NonlinearStatic;
    c
}

fn refused(api: &cSapModel, case: LoadCase, needle: &str) {
    let error = sap_api::validate_case_definition(&api.inner, &case)
        .expect_err(needle)
        .to_string();
    assert!(error.contains(needle), "{error} should contain {needle}");
}

#[test]
fn nonlinear_static_initial_case_is_accepted_for_nonlinear_static_cases_only() {
    let mut api = oscillator();
    set_case_definition(&mut api.inner, nl_static("N1")).unwrap();
    let mut n2 = nl_static("N2");
    n2.initial_case = Some("N1".into());
    set_case_definition(&mut api.inner, n2.clone()).unwrap();
    assert_eq!(
        api.inner.load_cases["N2"].initial_case.as_deref(),
        Some("N1")
    );
    // A third link of the chain.
    let mut n3 = nl_static("N3");
    n3.initial_case = Some("N2".into());
    set_case_definition(&mut api.inner, n3).unwrap();

    // Refused: missing, self, a case that is not nonlinear static, a cycle.
    let mut c = nl_static("N4");
    c.initial_case = Some("GHOST".into());
    refused(&api, c, "not an existing");
    let mut c = nl_static("N4");
    c.initial_case = Some("N4".into());
    refused(&api, c, "not an existing, different");
    set_case_definition(&mut api.inner, LoadCase::linear_static("LIN", "DEAD", 1.)).unwrap();
    let mut c = nl_static("N4");
    c.initial_case = Some("LIN".into());
    refused(&api, c, "not a nonlinear static case");
    api.inner
        .load_cases
        .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 2));
    let mut c = nl_static("N4");
    c.initial_case = Some("MODAL".into());
    refused(&api, c, "not a nonlinear static case");
    let mut n1 = nl_static("N1");
    n1.initial_case = Some("N3".into());
    refused(&api, n1, "cycle");
    // A stored case may already name a case that is not defined yet.
    let mut dangling = nl_static("N5");
    dangling.initial_case = Some("NEW".into());
    api.inner.load_cases.insert("N5".into(), dangling);
    let mut new = nl_static("NEW");
    new.initial_case = Some("N5".into());
    refused(&api, new, "cycle");
    api.inner.load_cases.shift_remove("N5");
    // Redefining N1 with itself unchanged is fine.
    set_case_definition(&mut api.inner, nl_static("N1")).unwrap();

    // Other case types still refuse an initial case.
    let mut lin = LoadCase::linear_static("LIN2", "DEAD", 1.);
    lin.initial_case = Some("N1".into());
    refused(&api, lin, "initial-state cases are supported only");
    let mut h = history();
    h.initial_case = Some("N1".into());
    refused(&api, h, "initial-state cases are supported only");
    let mut modal = LoadCase::modal_eigen("M2", 2);
    modal.initial_case = Some("N1".into());
    refused(&api, modal, "initial-state cases are supported only");

    // A case that others continue from must stay nonlinear static.
    refused(
        &api,
        LoadCase::linear_static("N1", "DEAD", 1.),
        "initial case of a nonlinear static",
    );
    assert_eq!(
        api.load_cases()
            .set_definition(LoadCase::linear_static("N1", "DEAD", 1.)),
        OAPI_ERR
    );
}

#[test]
fn saved_states_step_limits_and_unloading_are_checked() {
    use sap_model::hinge::{HingeUnloading, ResultsSaved};
    let mut api = oscillator();
    let mut c = nl_static("N");
    c.nl_static.results_saved = ResultsSaved::MultipleStates {
        min_saved: 6,
        max_saved: 10,
        positive_only: false,
    };
    c.nl_static.max_total_steps = 200;
    c.nl_static.max_null_steps = 50;
    set_case_definition(&mut api.inner, c.clone()).unwrap();
    assert_eq!(api.inner.load_cases["N"].nl_static, c.nl_static);

    for (min_saved, max_saved) in [(0, 10), (11, 10), (1, 100_001)] {
        let mut bad = c.clone();
        bad.nl_static.results_saved = ResultsSaved::MultipleStates {
            min_saved,
            max_saved,
            positive_only: true,
        };
        refused(&api, bad, "multiple saved states");
    }
    for (total, null) in [(0, 50), (1_000_001, 50), (200, 0), (200, 1_000_001)] {
        let mut bad = c.clone();
        bad.nl_static.max_total_steps = total;
        bad.nl_static.max_null_steps = null;
        refused(&api, bad, "steps must be");
    }
    for method in [
        HingeUnloading::ApplyLocalRedistribution,
        HingeUnloading::RestartUsingSecantStiffness,
    ] {
        let mut bad = c.clone();
        bad.nl_static.hinge_unloading = method;
        refused(&api, bad, "only \"unload entire structure\"");
    }
    // These belong to nonlinear static cases only.
    let mut lin = LoadCase::linear_static("L", "DEAD", 1.);
    lin.nl_static.max_total_steps = 10;
    refused(&api, lin, "belong to nonlinear static cases");
    let mut dynamic = history();
    dynamic.nl_static.results_saved = ResultsSaved::multiple_default();
    refused(&api, dynamic, "belong to nonlinear static cases");
}

#[test]
fn displacement_control_definition_keeps_its_checks() {
    let mut api = oscillator();
    let mut c = nl_static("PUSH");
    let control = DisplacementControl {
        target: 0.5,
        monitored: Some(("J".into(), 0)),
        conjugate: true,
    };
    c.displacement_control = Some(control.clone());
    set_case_definition(&mut api.inner, c.clone()).unwrap();
    for bad in [
        DisplacementControl {
            target: 0.,
            ..control.clone()
        },
        DisplacementControl {
            target: f64::NAN,
            ..control.clone()
        },
        DisplacementControl {
            monitored: Some(("GHOST".into(), 0)),
            ..control.clone()
        },
        DisplacementControl {
            monitored: Some(("J".into(), 6)),
            ..control.clone()
        },
        DisplacementControl {
            monitored: None,
            ..control.clone()
        },
    ] {
        let mut b = c.clone();
        b.displacement_control = Some(bad);
        refused(&api, b, "displacement control needs");
    }
    let mut pdelta = c;
    pdelta.nl_geom = NLGeomType::PDelta;
    refused(&api, pdelta, "displacement control needs");
}

#[test]
fn accel_loads_are_accepted_for_nonlinear_static_cases_only() {
    let mut api = oscillator();
    let mut c = nl_static("ACC");
    c.loads = vec![LoadCaseAssignment {
        load_name: "UX".into(),
        scale_factor: -1.,
    }];
    set_case_definition(&mut api.inner, c.clone()).unwrap();
    // Pattern and Accel rows can be mixed.
    c.loads.push(LoadCaseAssignment {
        load_name: "DEAD".into(),
        scale_factor: 2.,
    });
    set_case_definition(&mut api.inner, c.clone()).unwrap();
    // Not finite.
    let mut bad = c.clone();
    bad.loads[0].scale_factor = f64::INFINITY;
    refused(&api, bad, "nonfinite");
    // Rotational accelerations and unknown names are undefined patterns.
    for name in ["RX", "U4", "UW", "UXX"] {
        let mut bad = c.clone();
        bad.loads[0].load_name = name.into();
        refused(&api, bad, "undefined static load pattern");
    }
    // Linear static and buckling cases still refuse them.
    for kind in [LoadCaseType::LinearStatic, LoadCaseType::Buckling] {
        let mut bad = c.clone();
        bad.name = "OTHER".into();
        bad.case_type = kind;
        refused(&api, bad, "undefined static load pattern");
    }
    // A pattern named like an acceleration load is ambiguous: refused.
    api.inner.load_patterns.insert(
        "UY".into(),
        LoadPattern {
            name: "UY".into(),
            pat_type: LoadPatternType::Live,
            self_wt_multiplier: 0.,
        },
    );
    refused(&api, c, "named like an acceleration load");
}

fn with_modal(api: &mut cSapModel, max_modes: usize, run: bool) {
    let mut m = LoadCase::modal_eigen("MODAL", max_modes);
    m.run_flag = run;
    api.inner.load_cases.insert("MODAL".into(), m);
}

fn mode_case(mode: usize, sf: f64) -> LoadCase {
    let mut c = nl_static("MODEPUSH");
    c.loads.clear();
    c.modal_case = Some("MODAL".into());
    c.nl_static.mode_loads = vec![sap_model::hinge::ModeLoad {
        mode,
        scale_factor: sf,
    }];
    c
}

#[test]
fn mode_loads_need_a_run_modal_case_and_a_valid_mode() {
    let mut api = oscillator();
    with_modal(&mut api, 3, true);
    // A case with a Mode load and no load pattern is valid.
    set_case_definition(&mut api.inner, mode_case(2, 1.5)).unwrap();
    let stored = &api.inner.load_cases["MODEPUSH"];
    assert_eq!(stored.modal_case.as_deref(), Some("MODAL"));
    assert_eq!(stored.nl_static.mode_loads[0].mode, 2);
    assert!(stored.loads.is_empty());
    // Two different modes.
    let mut two = mode_case(1, 1.);
    two.nl_static.mode_loads.push(sap_model::hinge::ModeLoad {
        mode: 3,
        scale_factor: 1.,
    });
    set_case_definition(&mut api.inner, two).unwrap();

    refused(&api, mode_case(0, 1.), "outside 1..3");
    refused(&api, mode_case(4, 1.), "outside 1..3");
    refused(&api, mode_case(1, f64::NAN), "not finite");
    let mut dup = mode_case(1, 1.);
    dup.nl_static.mode_loads.push(sap_model::hinge::ModeLoad {
        mode: 1,
        scale_factor: 2.,
    });
    refused(&api, dup, "loaded twice");
    let mut no_modal = mode_case(1, 1.);
    no_modal.modal_case = None;
    refused(&api, no_modal, "need the Modal case");
    let mut ghost = mode_case(1, 1.);
    ghost.modal_case = Some("GHOST".into());
    refused(&api, ghost, "existing Modal case");
    set_case_definition(&mut api.inner, nl_static("OTHERNL")).unwrap();
    let mut wrong = mode_case(1, 1.);
    wrong.modal_case = Some("OTHERNL".into());
    refused(&api, wrong, "existing Modal case");
    // Mode mixed with a load pattern or an Accel load.
    for name in ["DEAD", "UX"] {
        let mut mixed = mode_case(1, 1.);
        mixed.loads = vec![LoadCaseAssignment {
            load_name: name.into(),
            scale_factor: 1.,
        }];
        refused(&api, mixed, "cannot be mixed");
    }
    // A modal case without a Mode load is a leftover.
    let mut leftover = nl_static("LEFT");
    leftover.modal_case = Some("MODAL".into());
    refused(&api, leftover, "no Mode load");
    // Mode loads of other case types.
    let mut lin = mode_case(1, 1.);
    lin.case_type = LoadCaseType::LinearStatic;
    refused(&api, lin, "belong to nonlinear static cases");
    // Modal superposition of a linear history still works.
    set_function_definition(&mut api.inner, function("STEP", true)).unwrap();
    let mut hist = history();
    hist.modal_case = Some("MODAL".into());
    hist.modal_damping = 0.05;
    set_case_definition(&mut api.inner, hist).unwrap();
    // The Modal case cannot become another type while a pushover uses it.
    refused(
        &api,
        LoadCase::linear_static("MODAL", "DEAD", 1.),
        "modal source",
    );
    // A modal case that is not run cannot supply modes.
    with_modal(&mut api, 3, false);
    refused(&api, mode_case(1, 1.), "must be run");
}

#[test]
fn pushover_definitions_refuse_while_locked_and_change_nothing() {
    let mut api = oscillator();
    with_modal(&mut api, 3, true);
    set_case_definition(&mut api.inner, nl_static("N1")).unwrap();
    api.inner.is_locked = true;
    let before = api.inner.clone();
    let mut n2 = nl_static("N2");
    n2.initial_case = Some("N1".into());
    assert!(set_case_definition(&mut api.inner, n2.clone()).is_err());
    assert_eq!(api.load_cases().set_definition(n2), OAPI_ERR);
    assert!(set_case_definition(&mut api.inner, mode_case(1, 1.)).is_err());
    assert_eq!(api.inner, before);
}
