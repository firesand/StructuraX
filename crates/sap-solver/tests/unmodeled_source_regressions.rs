//! Fail-closed handling of imported data that is recorded as not modeled, and
//! of modal damping on the modal-superposition history case types.
//!
//! The model is one spring-supported joint with unit mass (kip, in, s), so every
//! accepted case has a hand result: static u = P / k.
use sap_model::*;
use sap_solver::analyze_model;

fn model() -> SapModel {
    let mut m = SapModel::default();
    m.load_patterns["DEAD"].self_wt_multiplier = 0.;
    m.load_patterns.insert(
        "LIVE".into(),
        LoadPattern {
            name: "LIVE".into(),
            pat_type: LoadPatternType::Live,
            self_wt_multiplier: 0.,
        },
    );
    m.load_cases.clear();
    m.active_dofs = [true, false, false, false, false, false];
    m.add_point("J", 0., 0., 0.);
    m.points["J"].spring[0] = 100.;
    m.points["J"].mass[0] = 1.;
    for (pattern, force) in [("DEAD", 10.), ("LIVE", 30.)] {
        m.points["J"].loads_force.push(PointLoadForce {
            load_pat: pattern.into(),
            csys: "GLOBAL".into(),
            force: [force, 0., 0., 0., 0., 0.],
        });
    }
    m.functions.insert(
        "RAMP".into(),
        DynamicFunction {
            name: "RAMP".into(),
            is_time_history: true,
            damping_ratio: 0.,
            points: vec![(0., 0.), (1., 1.)],
        },
    );
    m.load_cases
        .insert("LIN".into(), LoadCase::linear_static("LIN", "DEAD", 1.));
    m.load_cases
        .insert("LIVE".into(), LoadCase::linear_static("LIVE", "LIVE", 1.));
    let mut nl = LoadCase::linear_static("NL", "DEAD", 1.);
    nl.case_type = LoadCaseType::NonlinearStatic;
    m.load_cases.insert("NL".into(), nl);
    // Continues from the nonlinear case and adds the live load.
    let mut next = LoadCase::linear_static("NEXT", "LIVE", 1.);
    next.case_type = LoadCaseType::NonlinearStatic;
    next.initial_case = Some("NL".into());
    m.load_cases.insert("NEXT".into(), next);
    let mut history = LoadCase::linear_static("HIST", "LIVE", 1.);
    history.loads.clear();
    history.case_type = LoadCaseType::LinearDynamic;
    history.history_loads = vec![("RAMP".into(), "LIVE".into(), 1.)];
    history.num_time_steps = 10;
    history.time_step_dt = 0.01;
    m.load_cases.insert("HIST".into(), history);
    m
}

fn entry(scope: UnmodeledScope) -> UnmodeledSourceData {
    UnmodeledSourceData {
        scope,
        description: "test data that is not modeled".into(),
    }
}

/// Runs only `enabled` and reports whether analysis was refused for unmodeled data.
fn refused(base: &SapModel, enabled: &[&str]) -> bool {
    let mut m = base.clone();
    for (name, case) in &mut m.load_cases {
        case.run_flag = enabled.contains(&name.as_str());
    }
    match analyze_model(&mut m) {
        Ok(()) => {
            assert_eq!(m.case_results.len(), enabled.len());
            false
        }
        Err(error) => {
            let text = error.to_string();
            assert!(
                text.contains("test data that is not modeled")
                    && text.contains("Remove the entry"),
                "{text}"
            );
            assert!(m.case_results.is_empty());
            true
        }
    }
}

#[test]
fn each_scope_refuses_exactly_the_cases_it_affects() {
    let clean = model();
    for case in ["LIN", "LIVE", "NL", "HIST"] {
        assert!(!refused(&clean, &[case]));
    }
    assert!(!refused(&clean, &["NL", "NEXT"]));

    let mut all = model();
    all.unmodeled_source.push(entry(UnmodeledScope::AllCases));
    for case in ["LIN", "LIVE", "NL", "HIST"] {
        assert!(refused(&all, &[case]));
    }

    let mut nonlinear = model();
    nonlinear
        .unmodeled_source
        .push(entry(UnmodeledScope::NonlinearCases));
    assert!(!refused(&nonlinear, &["LIN"]));
    assert!(!refused(&nonlinear, &["LIVE"]));
    assert!(!refused(&nonlinear, &["HIST"]));
    assert!(refused(&nonlinear, &["NL"]));
    assert!(refused(&nonlinear, &["NL", "NEXT"]));

    // Mass data: history (and modal/spectrum) cases, not static ones.
    let mut dynamic = model();
    dynamic
        .unmodeled_source
        .push(entry(UnmodeledScope::DynamicCases));
    assert!(!refused(&dynamic, &["LIN"]));
    assert!(!refused(&dynamic, &["NL", "NEXT"]));
    assert!(refused(&dynamic, &["HIST"]));
    dynamic
        .load_cases
        .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 1));
    assert!(refused(&dynamic, &["MODAL"]));
    // A static case that continues from a history inherits its dependence.
    dynamic.load_cases["NL"].initial_case = Some("HIST".into());
    assert_eq!(dynamic.unmodeled_source_for_case("NL").len(), 1);
    assert_eq!(dynamic.unmodeled_source_for_case("NEXT").len(), 1);
    assert!(dynamic.unmodeled_source_for_case("LIN").is_empty());

    // Data that only a linear static solution from zero is indifferent to.
    let mut refinement = model();
    refinement
        .unmodeled_source
        .push(entry(UnmodeledScope::AllButLinearStatic));
    assert!(!refused(&refinement, &["LIN"]));
    assert!(!refused(&refinement, &["LIVE"]));
    assert!(refused(&refinement, &["NL"]));
    assert!(refused(&refinement, &["HIST"]));
    // A linear static case that starts from another case is not "from zero".
    refinement.load_cases["LIVE"].initial_case = Some("LIN".into());
    assert_eq!(refinement.unmodeled_source_for_case("LIVE").len(), 1);
    assert!(refinement.unmodeled_source_for_case("LIN").is_empty());

    // A load pattern: static users, history users and cases that inherit it.
    let mut pattern = model();
    pattern
        .unmodeled_source
        .push(entry(UnmodeledScope::LoadPattern("LIVE".into())));
    assert!(!refused(&pattern, &["LIN"]));
    assert!(!refused(&pattern, &["NL"]));
    assert!(refused(&pattern, &["LIVE"]));
    assert!(refused(&pattern, &["HIST"]));
    assert!(refused(&pattern, &["NL", "NEXT"]));
    let mut inherited = model();
    inherited
        .unmodeled_source
        .push(entry(UnmodeledScope::LoadPattern("DEAD".into())));
    // NEXT applies LIVE itself but starts from NL, which applied DEAD.
    assert!(refused(&inherited, &["NL", "NEXT"]));
    assert!(!refused(&inherited, &["LIVE"]));
    // The lookup itself, without the initial case being what trips first.
    assert_eq!(inherited.unmodeled_source_for_case("NEXT").len(), 1);
    assert!(inherited.unmodeled_source_for_case("LIVE").is_empty());
    assert!(inherited.unmodeled_source_for_case("HIST").is_empty());
    // A zero scale factor does not apply the pattern.
    inherited.load_cases["LIN"].loads[0].scale_factor = 0.;
    assert!(!refused(&inherited, &["LIN"]));

    // One case and whatever continues from it.
    let mut case = model();
    case.unmodeled_source
        .push(entry(UnmodeledScope::Case("NL".into())));
    assert!(!refused(&case, &["LIN"]));
    assert!(refused(&case, &["NL"]));
    assert!(refused(&case, &["NL", "NEXT"]));
}

#[test]
fn discarding_the_entry_is_what_allows_analysis() {
    let mut m = model();
    m.unmodeled_source.push(entry(UnmodeledScope::AllCases));
    for case in m.load_cases.values_mut() {
        case.run_flag = case.name == "LIN";
    }
    assert!(analyze_model(&mut m).is_err());
    // Saving keeps the entry; it is not a transient import note.
    let mut reopened = sap_format::parse_s2k(&sap_format::write_s2k(&m)).unwrap();
    assert_eq!(reopened.unmodeled_source, m.unmodeled_source);
    assert!(analyze_model(&mut reopened).is_err());

    reopened.unmodeled_source.clear();
    analyze_model(&mut reopened).unwrap();
    // u = P / k = 10 kip / 100 kip/in.
    let u = reopened.case_results["LIN"].joint_displacements["J"][0];
    assert!((u - 0.1).abs() < 1e-12);
}

#[test]
fn modal_history_types_refuse_a_modal_damping_ratio() {
    for case_type in [LoadCaseType::LinearHistory, LoadCaseType::NonlinearHistory] {
        let mut m = model();
        for case in m.load_cases.values_mut() {
            case.run_flag = case.name == "HIST";
        }
        m.load_cases["HIST"].case_type = case_type;
        // `LoadCase::linear_static` carries the 0.05 default meant for spectra.
        assert_eq!(m.load_cases["HIST"].modal_damping, 0.05);
        let error = analyze_model(&mut m).unwrap_err().to_string();
        assert!(
            error.contains("HIST") && error.contains("modal damping ratio 0.05"),
            "{error}"
        );
        assert!(m.case_results.is_empty());

        m.load_cases["HIST"].modal_damping = f64::NAN;
        assert!(analyze_model(&mut m).is_err());

        // Zero modal damping: the same direct integration as the
        // direct-integration type, step for step.
        m.load_cases["HIST"].modal_damping = 0.;
        analyze_model(&mut m).unwrap();
        let modal_typed = m.case_results["HIST"].time_history_disp["J"].clone();
        m.load_cases["HIST"].case_type = if case_type == LoadCaseType::LinearHistory {
            LoadCaseType::LinearDynamic
        } else {
            LoadCaseType::NonlinearDynamic
        };
        // Direct-integration types do not read the modal damping ratio at all.
        m.load_cases["HIST"].modal_damping = 0.05;
        analyze_model(&mut m).unwrap();
        assert_eq!(m.case_results["HIST"].time_history_disp["J"], modal_typed);
    }
}

#[test]
fn static_cases_refuse_load_names_that_are_not_load_patterns() {
    for case_type in [
        LoadCaseType::LinearStatic,
        LoadCaseType::NonlinearStatic,
        LoadCaseType::Buckling,
    ] {
        for (load, expected) in [
            // An imported acceleration load arrives as a direction name.
            ("U1", "acceleration load 'U1'"),
            ("UZ", "acceleration load 'UZ'"),
            ("WIND", "missing load pattern 'WIND'"),
        ] {
            let mut m = model();
            for case in m.load_cases.values_mut() {
                case.run_flag = case.name == "LIN";
            }
            m.load_cases["LIN"].case_type = case_type;
            // The valid pattern alone would give u = 0.1 in; the unknown name
            // must not be dropped to leave exactly that result.
            m.load_cases["LIN"].loads.push(LoadCaseAssignment {
                load_name: load.into(),
                scale_factor: 1.,
            });
            let error = analyze_model(&mut m).unwrap_err().to_string();
            assert!(error.contains(expected), "{case_type:?} {load}: {error}");
            assert!(m.case_results.is_empty());
        }
    }
    // A load pattern that happens to be named like a direction is a pattern.
    let mut m = model();
    m.load_patterns.insert(
        "U1".into(),
        LoadPattern {
            name: "U1".into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.,
        },
    );
    for case in m.load_cases.values_mut() {
        case.run_flag = case.name == "LIN";
    }
    m.load_cases["LIN"].loads.push(LoadCaseAssignment {
        load_name: "U1".into(),
        scale_factor: 1.,
    });
    analyze_model(&mut m).unwrap();
    assert!((m.case_results["LIN"].joint_displacements["J"][0] - 0.1).abs() < 1e-12);
}
