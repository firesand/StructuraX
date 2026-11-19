//! D78: a load combination one of whose contributors has no results is unavailable.
//!
//! Reference (closed form, kN, m): D = 10 kN and L = 30 kN act on a 100 kN/m spring,
//! ULS = 1.2 D + 1.6 L = 1.2*10 + 1.6*30 = 60 kN, so the reaction magnitude is
//! 60 kN (tolerance 1e-9 kN, exact arithmetic of a one-degree-of-freedom model).
//! With L not run the combination must not exist under any name (12 kN is the
//! partial value 1.2*10 that was published before), the reason must name the
//! combination and the case, and every result that does not depend on L stays.

use sap_design::{try_run_concrete_design, try_run_steel_design, DesignError};
use sap_format::s2k::parse_s2k;
use sap_model::*;
use sap_section::make_i_section;
use sap_solver::analyze_model;
use sap_units::Units;

fn uls_text(run_l: bool, extra_combos: &str) -> String {
    format!(
        r#"TABLE: "PROGRAM CONTROL"
 CurrUnits="kN, m, C"
TABLE: "JOINT COORDINATES"
 Joint=J XorR=0 Y=0 Z=0
TABLE: "JOINT RESTRAINT ASSIGNMENTS"
 Joint=J U2=Yes U3=Yes R1=Yes R2=Yes R3=Yes
TABLE: "JOINT SPRING ASSIGNMENTS 1 - UNCOUPLED"
 Joint=J U1=100
TABLE: "LOAD PATTERN DEFINITIONS"
 LoadPat=D DesignType=Dead SelfWtMult=0
 LoadPat=L DesignType=Live SelfWtMult=0
TABLE: "JOINT LOADS - FORCE"
 Joint=J LoadPat=D F1=10
 Joint=J LoadPat=L F1=30
TABLE: "LOAD CASE DEFINITIONS"
 Case=D Type=LinStatic RunCase=Yes
 Case=L Type=LinStatic RunCase={}
TABLE: "COMBINATION DEFINITIONS"
 ComboName=ULS ComboType="Linear Add" CaseName=D ScaleFactor=1.2
 ComboName=ULS ComboType="Linear Add" CaseName=L ScaleFactor=1.6
 ComboName=SERV ComboType="Linear Add" CaseName=D ScaleFactor=1
{extra_combos}END TABLE DATA
"#,
        if run_l { "Yes" } else { "No" }
    )
}

fn run(text: &str) -> SapModel {
    let mut m = parse_s2k(text).expect("parse");
    analyze_model(&mut m).expect("analysis succeeds; only the combination is unavailable");
    m
}

fn fx(m: &SapModel, name: &str) -> f64 {
    m.case_results[name].base_reactions[0]
}

#[test]
fn positive_control_with_l_on_gives_60_kn() {
    let m = run(&uls_text(true, ""));
    assert!((fx(&m, "ULS").abs() - 60.0).abs() < 1e-9, "{}", fx(&m, "ULS"));
    assert!(m.unavailable_results.is_empty());
}

#[test]
fn disabled_contributor_makes_the_combination_unavailable_and_keeps_the_rest() {
    let m = run(&uls_text(false, ""));
    assert!(
        !m.case_results.contains_key("ULS"),
        "ULS must not be published (the partial value would be 12 kN)"
    );
    let reason = &m.unavailable_results["ULS"];
    assert!(reason.contains("'L'") && reason.contains("RunCase=No"), "{reason}");
    let msg = sap_model::results::unavailable_message(&m, "ULS").unwrap();
    assert!(msg.contains("'ULS'") && msg.contains("'L'"), "{msg}");
    // Unrelated results stay: D = 10 kN, and SERV = 1.0 D = 10 kN does not depend on L.
    assert!((fx(&m, "D").abs() - 10.0).abs() < 1e-9);
    assert!((fx(&m, "SERV").abs() - 10.0).abs() < 1e-9);
    assert!(!m.unavailable_results.contains_key("SERV"));
}

#[test]
fn nested_combination_of_an_unavailable_one_is_unavailable_at_any_depth() {
    // ULS2 = 1.0 ULS + 1.0 D; ULS3 = 1.0 ULS2 (depth two); both with L on and off.
    let nested = " ComboName=ULS2 ComboType=\"Linear Add\" CaseName=ULS ScaleFactor=1\n \
        ComboName=ULS2 ComboType=\"Linear Add\" CaseName=D ScaleFactor=1\n \
        ComboName=ULS3 ComboType=\"Linear Add\" CaseName=ULS2 ScaleFactor=1\n";
    let on = run(&uls_text(true, nested));
    assert!((fx(&on, "ULS2").abs() - 70.0).abs() < 1e-9); // 60 + 10 kN
    assert!((fx(&on, "ULS3").abs() - 70.0).abs() < 1e-9);
    let off = run(&uls_text(false, nested));
    for name in ["ULS2", "ULS3"] {
        assert!(!off.case_results.contains_key(name), "{name} must not exist");
        let r = &off.unavailable_results[name];
        assert!(r.contains("'ULS'") && r.contains("'L'"), "{name}: {r}");
    }
    assert!(off.unavailable_results["ULS3"].contains("'ULS2'"));
}

#[test]
fn envelope_and_range_combinations_over_an_unavailable_one_are_unavailable() {
    let extra = " ComboName=ENV ComboType=Envelope CaseName=ULS ScaleFactor=1\n \
        ComboName=ENV ComboType=Envelope CaseName=D ScaleFactor=1\n \
        ComboName=ENV2 ComboType=Envelope CaseName=SERV ScaleFactor=1\n \
        ComboName=ENV2 ComboType=Envelope CaseName=D ScaleFactor=1\n";
    let on = run(&uls_text(true, extra));
    // Envelope of ULS (-60 kN) and D (-10 kN): the minimum is -60 kN.
    let env = &on.case_results["ENV"];
    let lo = env.combo_min.as_deref().unwrap().base_reactions[0];
    assert!((lo + 60.0).abs() < 1e-9 && (env.base_reactions[0] + 10.0).abs() < 1e-9);
    let off = run(&uls_text(false, extra));
    assert!(!off.case_results.contains_key("ENV"));
    let r = &off.unavailable_results["ENV"];
    assert!(r.contains("'ULS'") && r.contains("'L'"), "{r}");
    // An envelope that does not depend on L remains usable.
    assert!(off.case_results.contains_key("ENV2"));
}

#[test]
fn zero_scale_factor_does_not_excuse_a_missing_contributor() {
    // The vendor behaviour for a zero factor is not established: the contributor is required.
    let extra = " ComboName=Z ComboType=\"Linear Add\" CaseName=D ScaleFactor=1\n \
        ComboName=Z ComboType=\"Linear Add\" CaseName=L ScaleFactor=0\n";
    let off = run(&uls_text(false, extra));
    assert!(!off.case_results.contains_key("Z"));
    assert!(off.unavailable_results["Z"].contains("'L'"));
    let on = run(&uls_text(true, extra));
    assert!((fx(&on, "Z").abs() - 10.0).abs() < 1e-9);
}

#[test]
fn undefined_contributor_circular_and_empty_combinations_are_unavailable() {
    let extra = " ComboName=GHOST ComboType=\"Linear Add\" CaseName=D ScaleFactor=1\n \
        ComboName=GHOST ComboType=\"Linear Add\" CaseName=NOPE ScaleFactor=1\n \
        ComboName=A ComboType=\"Linear Add\" CaseName=B ScaleFactor=1\n \
        ComboName=B ComboType=\"Linear Add\" CaseName=A ScaleFactor=1\n \
        ComboName=DEPA ComboType=\"Linear Add\" CaseName=A ScaleFactor=1\n";
    let m = run(&uls_text(true, extra));
    assert!(m.unavailable_results["GHOST"].contains("'NOPE'"));
    for name in ["A", "B", "DEPA"] {
        assert!(!m.case_results.contains_key(name), "{name}");
        assert!(m.unavailable_results.contains_key(name), "{name}");
    }
    assert!(m.unavailable_results["A"].contains("itself"));
    // The valid ones are untouched.
    assert!((fx(&m, "ULS").abs() - 60.0).abs() < 1e-9);
}

#[test]
fn reanalysis_after_enabling_the_case_clears_the_diagnostic() {
    let mut m = parse_s2k(&uls_text(false, "")).unwrap();
    analyze_model(&mut m).unwrap();
    assert!(m.unavailable_results.contains_key("ULS"));
    m.invalidate_results();
    m.is_locked = false;
    m.load_cases.get_mut("L").unwrap().run_flag = true;
    analyze_model(&mut m).unwrap();
    assert!(m.unavailable_results.is_empty());
    assert!((fx(&m, "ULS").abs() - 60.0).abs() < 1e-9);
}

#[test]
fn the_api_refuses_to_select_an_unavailable_combination_and_lists_it() {
    let mut api = sap_api::cSapModel::new(Units::KnMC);
    api.inner = parse_s2k(&uls_text(false, "")).unwrap();
    assert_eq!(api.analyze().run_analysis(), 0);
    let list = api.analyze().unavailable_results();
    assert_eq!(list.len(), 1);
    assert_eq!(list[0].0, "ULS");
    assert_ne!(api.results().setup_set_combo_selected_for_output("ULS", true), 0);
    assert_eq!(api.results().setup_set_combo_selected_for_output("SERV", true), 0);
}

// ---- design -----------------------------------------------------------------

/// A 120 in steel cantilever (kip, in) with dead and live gravity loads and
/// ULS = 1.2 D + 1.6 L; L is run or not.
fn cantilever(run_l: bool) -> SapModel {
    let mut m = SapModel::new(Units::KipInF);
    m.load_cases.clear();
    m.load_patterns.clear();
    for p in ["D", "L"] {
        m.load_patterns.insert(
            p.into(),
            LoadPattern {
                name: p.into(),
                pat_type: LoadPatternType::Other,
                self_wt_multiplier: 0.0,
            },
        );
        m.load_cases.insert(p.into(), LoadCase::linear_static(p, p, 1.0));
    }
    m.load_cases.get_mut("L").unwrap().run_flag = run_l;
    let sec = make_i_section("W14X90", "A992Fy50", 14.0, 14.5, 0.710, 0.440, 14.5, 0.710);
    m.frame_sections.insert(sec.name.clone(), sec);
    m.add_point("A", 0.0, 0.0, 0.0);
    m.add_point("B", 120.0, 0.0, 0.0);
    m.points.get_mut("A").unwrap().restraint = [true; 6];
    m.add_frame("BM", "A", "B", "W14X90");
    for (p, w) in [("D", -0.05), ("L", -0.10)] {
        m.frames.get_mut("BM").unwrap().loads_dist.push(FrameLoadDistributed {
            load_pat: p.into(),
            my_type: 1,
            dir: 6,
            csys: "GLOBAL".into(),
            rel_dist1: 0.0,
            rel_dist2: 1.0,
            val1: w,
            val2: w,
        });
    }
    m.load_combos.insert(
        "ULS".into(),
        LoadCombo {
            name: "ULS".into(),
            combo_type: ComboType::LinearAdd,
            cases: vec![("D".into(), 1.2), ("L".into(), 1.6)],
        },
    );
    m
}

#[test]
fn steel_design_positive_control_runs_with_l_on() {
    let mut m = cantilever(true);
    analyze_model(&mut m).unwrap();
    try_run_steel_design(&mut m).expect("design with every contributor present");
    assert!(!m.steel_design_results.is_empty());
}

#[test]
fn steel_and_concrete_design_refuse_a_combination_with_a_disabled_contributor() {
    let mut m = cantilever(false);
    analyze_model(&mut m).unwrap();
    assert!(m.case_results.contains_key("D") && !m.case_results.contains_key("ULS"));
    let err = try_run_steel_design(&mut m).unwrap_err();
    match &err {
        DesignError::UnavailableCombination { combo, reason, .. } => {
            assert_eq!(combo, "ULS");
            assert!(reason.contains("'L'"), "{reason}");
        }
        other => panic!("wrong error {other:?}"),
    }
    assert!(err.to_string().contains("'ULS'") && err.to_string().contains("'L'"));
    assert!(m.steel_design_results.is_empty(), "no partial ratios");
    let err = try_run_concrete_design(&mut m).unwrap_err();
    assert!(matches!(err, DesignError::UnavailableCombination { .. }), "{err}");
}

#[test]
fn design_refuses_a_combination_added_after_analysis_that_names_a_missing_case() {
    // Not evaluated by the solver (defined after the run), so design would synthesize it
    // from its contributors; it must not do so with L missing.
    let mut m = cantilever(false);
    m.load_combos.clear();
    analyze_model(&mut m).unwrap();
    m.load_combos.insert(
        "LATE".into(),
        LoadCombo {
            name: "LATE".into(),
            combo_type: ComboType::LinearAdd,
            cases: vec![("D".into(), 1.2), ("L".into(), 1.6)],
        },
    );
    let err = try_run_steel_design(&mut m).unwrap_err().to_string();
    assert!(err.contains("'LATE'") && err.contains("'L'"), "{err}");
    // With the contributor complete the same synthesis is allowed.
    m.load_combos.get_mut("LATE").unwrap().cases = vec![("D".into(), 1.2)];
    try_run_steel_design(&mut m).expect("complete synthesized combination");
}

#[test]
fn design_refuses_a_nested_combination_of_an_unavailable_one() {
    let mut m = cantilever(false);
    m.load_combos.insert(
        "TOP".into(),
        LoadCombo {
            name: "TOP".into(),
            combo_type: ComboType::LinearAdd,
            cases: vec![("ULS".into(), 1.0), ("D".into(), 1.0)],
        },
    );
    m.load_combos.shift_remove("ULS");
    m.load_combos.insert(
        "ULS".into(),
        LoadCombo {
            name: "ULS".into(),
            combo_type: ComboType::LinearAdd,
            cases: vec![("D".into(), 1.2), ("L".into(), 1.6)],
        },
    );
    analyze_model(&mut m).unwrap();
    assert!(!m.case_results.contains_key("TOP"));
    let err = try_run_steel_design(&mut m).unwrap_err().to_string();
    assert!(err.contains("combination") && err.contains("'L'"), "{err}");
}

// ---- more contributor kinds ---------------------------------------------------

#[test]
fn range_combination_over_an_unavailable_one_is_unavailable() {
    let extra = " ComboName=RNG ComboType=\"Range Add\" CaseName=ULS ScaleFactor=1\n \
        ComboName=RNG ComboType=\"Range Add\" CaseName=D ScaleFactor=1\n";
    let on = run(&uls_text(true, extra));
    assert!(on.case_results.contains_key("RNG"));
    let off = run(&uls_text(false, extra));
    assert!(!off.case_results.contains_key("RNG"));
    let r = &off.unavailable_results["RNG"];
    assert!(r.contains("'ULS'") && r.contains("'L'"), "{r}");
}

#[test]
fn modal_contributor_cannot_be_combined_without_a_mode() {
    // A modal case M beside D. A combination names no mode, so the stored first-mode
    // shape must not enter the sum as if it were a response.
    let mut m = parse_s2k(&uls_text(true, "")).unwrap();
    m.load_cases.insert("M".into(), {
        let mut c = LoadCase::linear_static("M", "D", 1.0);
        c.case_type = LoadCaseType::Modal;
        c.loads.clear();
        c
    });
    m.load_combos.insert(
        "DM".into(),
        LoadCombo {
            name: "DM".into(),
            combo_type: ComboType::LinearAdd,
            cases: vec![("D".into(), 1.0), ("M".into(), 1.0)],
        },
    );
    m.points.get_mut("J").unwrap().mass = [1.0; 6];
    analyze_model(&mut m).expect("the modal case solves");
    assert!(!m.case_results.contains_key("DM"));
    let r = &m.unavailable_results["DM"];
    assert!(r.contains("'M'") && r.contains("Modal"), "{r}");
    assert!((fx(&m, "ULS").abs() - 60.0).abs() < 1e-9);
}

#[test]
fn case_naming_a_missing_pattern_refuses_the_run_and_publishes_nothing() {
    let mut m = parse_s2k(&uls_text(true, "")).unwrap();
    m.load_cases
        .insert("Q".into(), LoadCase::linear_static("Q", "NOPAT", 1.0));
    let err = analyze_model(&mut m).unwrap_err().to_string();
    assert!(err.contains("'Q'") && err.contains("'NOPAT'"), "{err}");
    assert!(m.case_results.is_empty() && !m.is_locked);
}

// ---- the command line (the harness runs the built binary) ----------------------

fn temp_dir(tag: &str) -> std::path::PathBuf {
    let d = std::env::temp_dir().join(format!("structurax_p1_{tag}_{}", std::process::id()));
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
fn cli_names_the_unavailable_combination_and_keeps_the_other_results() {
    let dir = temp_dir("cli");
    let input = dir.join("uls.s2k");
    let output = dir.join("uls.json");
    let (i, o) = (input.to_str().unwrap(), output.to_str().unwrap());
    std::fs::write(&input, uls_text(false, "")).unwrap();
    let out = cli(&["analyze", i, "--output", o]);
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(out.status.success(), "the run is not discarded: {err}");
    assert!(
        err.contains("'ULS'") && err.contains("'L'") && err.contains("unavailable"),
        "{err}"
    );
    let json: serde_json::Value = serde_json::from_slice(&std::fs::read(&output).unwrap()).unwrap();
    assert!(json["case_results"].get("ULS").is_none());
    assert!(json["case_results"].get("D").is_some());
    assert!(json["unavailable_results"]["ULS"].as_str().unwrap().contains("'L'"));
    // Positive control: with L on, ULS exists (-60 kN) and nothing is reported.
    std::fs::write(&input, uls_text(true, "")).unwrap();
    let out = cli(&["analyze", i, "--output", o]);
    assert!(out.status.success() && !String::from_utf8_lossy(&out.stderr).contains("unavailable"));
    let json: serde_json::Value = serde_json::from_slice(&std::fs::read(&output).unwrap()).unwrap();
    let fx = json["case_results"]["ULS"]["base_reactions"][0].as_f64().unwrap();
    assert!((fx + 60.0).abs() < 1e-9, "{fx}");
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn cli_design_of_an_unavailable_combination_fails_naming_it() {
    let dir = temp_dir("clid");
    let input = dir.join("uls.s2k");
    std::fs::write(&input, uls_text(false, "")).unwrap();
    let out = cli(&["analyze", input.to_str().unwrap(), "--steel-code", "AISC 360-10"]);
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(!out.status.success(), "{err}");
    assert!(err.contains("'ULS'") && err.contains("'L'"), "{err}");
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn api_combination_selected_before_the_run_is_refused_by_the_getters() {
    let mut api = sap_api::cSapModel::new(Units::KnMC);
    api.inner = parse_s2k(&uls_text(false, "")).unwrap();
    assert_eq!(api.results().setup_set_combo_selected_for_output("ULS", true), 0);
    assert_eq!(api.analyze().run_analysis(), 0);
    assert_ne!(
        api.results().base_react().0,
        0,
        "ULS has no results: refuse, do not return D alone"
    );
    api.results().setup_deselect_all_cases_and_combos_for_output();
    api.results().setup_set_combo_selected_for_output("SERV", true);
    let (code, rows) = api.results().base_react();
    assert_eq!(code, 0);
    assert!((rows[0].1[0].abs() - 10.0).abs() < 1e-9);
}
