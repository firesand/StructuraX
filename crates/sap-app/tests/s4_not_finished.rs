//! G16 (wave 6, stage S4): a nonlinear static case that stops early keeps what it reached
//! and is "not finished" (the v19 words, help "Set Load Cases to Run"); a case that depends
//! on it "could not start"; the other cases run. This file checks the consumers of the
//! result: the command line (stderr, JSON, exit status), steel design, the API.
//!
//! Model (kip, in): a W14X90 steel cantilever, L = 120 in, a tip load pattern P, an
//! elastic-perfectly-plastic moment hinge at the root with a yield moment of 2000 kip in.
//! The limit load of the root is 2000 / 120 = 16.667 kip (closed form), so a push of 100 kip
//! by load control cannot reach its target: the case keeps the state at P = 16.667 kip
//! (tolerance 1e-9, exact arithmetic of the event stepper) and is not finished. A push of
//! 10 kip is below the limit and finishes.

use sap_design::{try_run_steel_design, DesignError};
use sap_model::hinge::*;
use sap_model::*;
use sap_section::make_i_section;
use sap_solver::analyze_model;
use sap_units::Units;

const MP: f64 = 2000.0;
const LEN: f64 = 120.0;

fn model(push: f64) -> SapModel {
    let mut m = SapModel::new(Units::KipInF);
    m.load_cases.clear();
    m.load_patterns.clear();
    m.load_patterns.insert(
        "P".into(),
        LoadPattern {
            name: "P".into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
    let sec = make_i_section("W14X90", "A992Fy50", 14.0, 14.5, 0.710, 0.440, 14.5, 0.710);
    m.frame_sections.insert(sec.name.clone(), sec);
    m.add_point("A", 0.0, 0.0, 0.0);
    m.add_point("B", LEN, 0.0, 0.0);
    m.points.get_mut("A").unwrap().restraint = [true; 6];
    m.points.get_mut("B").unwrap().loads_force.push(PointLoadForce {
        load_pat: "P".into(),
        csys: "GLOBAL".into(),
        force: [0.0, 0.0, -1.0, 0.0, 0.0, 0.0],
    });
    m.add_frame("BM", "A", "B", "W14X90");
    let mut prop = HingeProp::symmetric(
        "H",
        HingeDof::M3,
        [
            HingePoint::new(1.0, 0.0),
            HingePoint::new(1.0, 0.01),
            HingePoint::new(1.0, 0.02),
            HingePoint::new(1.0, 0.03),
        ],
    );
    prop.force_sf = MP;
    prop.force_sf_negative = MP;
    m.hinge_props.insert("H".into(), prop);
    m.frames.get_mut("BM").unwrap().hinges = vec![FrameHinge {
        prop: "H".into(),
        rel_dist: 0.0,
    }];
    let mut case = LoadCase::linear_static("PUSH", "P", push);
    case.case_type = LoadCaseType::NonlinearStatic;
    case.nl_static.results_saved = ResultsSaved::MultipleStates {
        min_saved: 4,
        max_saved: 10,
        positive_only: false,
    };
    m.load_cases.insert("PUSH".into(), case);
    // A linear case of the same model, and a case that continues from the push.
    m.load_cases
        .insert("LIN".into(), LoadCase::linear_static("LIN", "P", 1.0));
    let mut next = LoadCase::linear_static("NEXT", "P", 1.0);
    next.case_type = LoadCaseType::NonlinearStatic;
    next.initial_case = Some("PUSH".into());
    m.load_cases.insert("NEXT".into(), next);
    m
}

fn temp_dir(tag: &str) -> std::path::PathBuf {
    let d = std::env::temp_dir().join(format!("structurax_s4_{tag}_{}", std::process::id()));
    std::fs::create_dir_all(&d).unwrap();
    d
}

fn cli(args: &[&str]) -> std::process::Output {
    std::process::Command::new(env!("CARGO_BIN_EXE_structurax"))
        .args(args)
        .output()
        .unwrap()
}

#[test]
fn the_push_keeps_the_state_at_the_limit_load_and_is_not_finished() {
    let mut m = model(100.0);
    analyze_model(&mut m).expect("not finished is not an error");
    let push = &m.case_results["PUSH"];
    let CaseStatus::NotFinished { reason, steps } = &push.status else {
        panic!("expected NotFinished, got {:?}", push.status);
    };
    assert!(reason.contains("limit"), "{reason}");
    assert_eq!(*steps, push.static_steps.len());
    // Closed form: the root yields at P = Mp / L = 2000 / 120 kip; the last state is that
    // one (scale 100, load factor 1/6).
    let load = push.displacement_control_factor.unwrap() * 100.0;
    assert!((load - MP / LEN).abs() < 1e-9 * (MP / LEN), "{load}");
    assert!(m.case_results["LIN"].status.is_finished());
    assert!(m.unavailable_results["NEXT"].starts_with("could not start:"));
    // A push below the limit finishes and its dependant starts.
    let mut ok = model(10.0);
    analyze_model(&mut ok).unwrap();
    assert!(ok.case_results["PUSH"].status.is_finished());
    assert!(ok.case_results.contains_key("NEXT"));
    assert!(ok.unavailable_results.is_empty());
}

#[test]
fn cli_names_each_case_that_is_not_finished_and_writes_the_status() {
    let dir = temp_dir("cli");
    let input = dir.join("push.s2k");
    let output = dir.join("push.json");
    let (i, o) = (input.to_str().unwrap(), output.to_str().unwrap());
    std::fs::write(&input, sap_format::write_s2k(&model(100.0))).unwrap();
    let out = cli(&["analyze", i, "--output", o]);
    let err = String::from_utf8_lossy(&out.stderr);
    // Exit status 0, like an unavailable combination: the other results are valid.
    assert!(out.status.success(), "{err}");
    let line = err
        .lines()
        .find(|l| l.starts_with("Warning: case 'PUSH' is not finished"))
        .unwrap_or_else(|| panic!("no line for PUSH: {err}"));
    assert!(line.contains("limit") && line.contains("saved steps kept"), "{line}");
    assert!(
        err.lines()
            .any(|l| l.contains("'NEXT'") && l.contains("could not start") && l.contains("'PUSH'")),
        "{err}"
    );
    let json: serde_json::Value = serde_json::from_slice(&std::fs::read(&output).unwrap()).unwrap();
    let status = &json["case_results"]["PUSH"]["status"]["NotFinished"];
    assert!(status["reason"].as_str().unwrap().contains("limit"), "{status}");
    assert!(status["steps"].as_u64().unwrap() >= 1);
    assert!(json["case_results"]["LIN"].get("status").is_none(), "finished: no status field");
    assert!(json["case_results"].get("NEXT").is_none());
    assert!(json["unavailable_results"]["NEXT"]
        .as_str()
        .unwrap()
        .starts_with("could not start:"));
    // Positive control: a push that finishes reports nothing.
    std::fs::write(&input, sap_format::write_s2k(&model(10.0))).unwrap();
    let out = cli(&["analyze", i, "--output", o]);
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(out.status.success() && !err.contains("not finished"), "{err}");
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn steel_design_refuses_a_case_that_is_not_finished_by_name() {
    let mut m = model(100.0);
    analyze_model(&mut m).unwrap();
    let err = try_run_steel_design(&mut m).unwrap_err();
    match &err {
        DesignError::UnfinishedCase { case, reason, .. } => {
            assert_eq!(case, "PUSH");
            assert!(reason.contains("limit"), "{reason}");
        }
        other => panic!("wrong error {other:?}"),
    }
    assert!(err.to_string().contains("'PUSH'"), "{err}");
    assert!(m.steel_design_results.is_empty(), "no partial ratios");
    let err = sap_design::try_run_concrete_design(&mut m).unwrap_err();
    assert!(
        matches!(&err, DesignError::UnfinishedCase { case, .. } if case == "PUSH"),
        "{err}"
    );
    assert!(m.concrete_design_results.is_empty());
    // A combination that contains the case is unavailable, and design refuses it by name.
    m.load_combos.insert(
        "C1".into(),
        LoadCombo {
            name: "C1".into(),
            combo_type: ComboType::LinearAdd,
            cases: vec![("PUSH".into(), 1.0), ("LIN".into(), 1.0)],
        },
    );
    analyze_model(&mut m).unwrap();
    assert!(!m.case_results.contains_key("C1"));
    assert!(m.unavailable_results["C1"].contains("'PUSH' is not finished"));
    match try_run_steel_design(&mut m).unwrap_err() {
        DesignError::UnavailableCombination { combo, reason, .. } => {
            assert_eq!(combo, "C1");
            assert!(reason.contains("'PUSH'") && reason.contains("not finished"), "{reason}");
        }
        other => panic!("wrong error {other:?}"),
    }
    // Positive control: a combination of the cases that finished designs.
    m.load_combos["C1"].cases = vec![("LIN".into(), 1.0)];
    analyze_model(&mut m).unwrap();
    try_run_steel_design(&mut m).expect("design of a combination of finished cases");
    assert!(!m.steel_design_results.is_empty());
}

#[test]
fn the_api_reports_the_v19_status_codes_and_refuses_the_combination() {
    let mut api = sap_api::cSapModel::new(Units::KipInF);
    api.inner = model(100.0);
    api.inner.load_combos.insert(
        "C1".into(),
        LoadCombo {
            name: "C1".into(),
            combo_type: ComboType::LinearAdd,
            cases: vec![("PUSH".into(), 1.0)],
        },
    );
    // Before the run every case is "not run" (1).
    assert!(api.analyze().get_case_status().iter().all(|(_, code)| *code == 1));
    assert_eq!(api.analyze().run_analysis(), 0);
    let codes: std::collections::BTreeMap<String, i32> =
        api.analyze().get_case_status().into_iter().collect();
    assert_eq!(codes["LIN"], 4);
    assert_eq!(codes["PUSH"], 3);
    assert_eq!(codes["NEXT"], 2);
    assert!(api.analyze().case_status_reason("PUSH").unwrap().contains("limit"));
    assert!(api.analyze().case_status_reason("NEXT").unwrap().starts_with("could not start:"));
    assert!(api.analyze().case_status_reason("LIN").is_none());
    // The combination that names the case is unavailable and cannot be selected.
    assert_ne!(api.results().setup_set_combo_selected_for_output("C1", true), 0);
    assert!(api
        .analyze()
        .unavailable_results()
        .iter()
        .any(|(name, reason)| name == "C1" && reason.contains("'PUSH'")));
    // A case that could not start has no results: it cannot be selected after the run, and
    // a getter refuses when it was selected before (it does not return the other cases only).
    assert_ne!(api.results().setup_set_case_selected_for_output("NEXT", true), 0);
    api.results().setup_deselect_all_cases_and_combos_for_output();
    api.results().setup_set_case_selected_for_output("LIN", true);
    let (code, rows) = api.results().base_react();
    assert_eq!(code, 0);
    assert_eq!(rows.len(), 1);
    api.inner.invalidate_results();
    api.results().setup_set_case_selected_for_output("NEXT", true);
    api.results().setup_set_case_selected_for_output("LIN", true);
    assert_eq!(api.analyze().run_analysis(), 0);
    assert_ne!(
        api.results().base_react().0,
        0,
        "NEXT could not start: refuse, do not return LIN alone"
    );
    // The curve of the case that is not finished is the part it reached, and says so; the
    // case that could not start has no curve.
    // PUSH is load controlled, so its curve is refused for that reason, not for the status.
    let e = sap_api::pushover_curve(&api.inner, "PUSH").unwrap_err();
    assert!(e.contains("displacement control"), "{e}");
    let (reason, kept) = sap_api::pushover_not_finished(&api.inner, "PUSH").unwrap();
    assert!(reason.contains("limit"), "{reason}");
    assert_eq!(kept, api.inner.case_results["PUSH"].static_steps.len());
    let e = sap_api::pushover_curve(&api.inner, "NEXT").unwrap_err();
    assert!(e.contains("could not start"), "{e}");
}

#[test]
fn results_saved_before_this_stage_read_as_finished() {
    // `status` has a serde default: a result written without it is a finished case.
    let mut m = model(10.0);
    analyze_model(&mut m).unwrap();
    let mut json: serde_json::Value = serde_json::to_value(&m).unwrap();
    assert!(json["case_results"]["PUSH"].get("status").is_none());
    json["case_results"]["PUSH"]
        .as_object_mut()
        .unwrap()
        .remove("status");
    let back: SapModel = serde_json::from_value(json).unwrap();
    assert!(back.case_results["PUSH"].status.is_finished());
}
