//! D79 (a load case and a load combination with one name) and D80 (a combination that
//! would take a single history snapshot for an envelope).
//!
//! Reference (kN, m): D = 10 kN and L = 30 kN on a 100 kN/m spring (tolerance 1e-9).
//! `combo-healthy`: ULS = 1.2 D + 1.6 L, SERV = 1.0 D, OK = 2 SERV = 20 kN.
//! `combo-missing`: L not run; ULS and everything that depends on it have no results,
//! D = 10 kN and SERV = 10 kN stay. `combo-shadow`: a case D and a combination D.
//! `combo-history`: D = +0.1 m and a triangular pulse H with 101 stored samples
//! spanning -0.1409388942 .. +0.1415610469 m (a snapshot at the step of largest
//! displacement is +0.1415610469 m); the envelope of H would be that pair, and D + H
//! would span -0.0409388942 .. +0.2415610469 m. The extrema over time of the forces
//! and reactions are not stored, so these combinations are withheld.

use sap_design::{try_run_concrete_design, try_run_steel_design, DesignError};
use sap_format::s2k::parse_s2k;
use sap_model::*;
use sap_section::make_i_section;
use sap_solver::analyze_model;
use sap_units::Units;

const HEADER: &str = r#"TABLE: "PROGRAM CONTROL"
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
"#;

fn shadow_text() -> String {
    format!(
        "{HEADER}TABLE: \"LOAD CASE DEFINITIONS\"\n Case=D Type=LinStatic RunCase=Yes\n \
         Case=L Type=LinStatic RunCase=No\nTABLE: \"COMBINATION DEFINITIONS\"\n \
         ComboName=ULS ComboType=\"Linear Add\" CaseName=D ScaleFactor=1.2\n \
         ComboName=ULS ComboType=\"Linear Add\" CaseName=L ScaleFactor=1.6\n \
         ComboName=SERV ComboType=\"Linear Add\" CaseName=D ScaleFactor=1\n \
         ComboName=D ComboType=\"Linear Add\" CaseName=L ScaleFactor=1\n \
         ComboName=TOP ComboType=\"Linear Add\" CaseName=D ScaleFactor=2\nEND TABLE DATA\n"
    )
}

fn healthy_text(run_l: bool) -> String {
    format!(
        "{HEADER}TABLE: \"LOAD CASE DEFINITIONS\"\n Case=D Type=LinStatic RunCase=Yes\n \
         Case=L Type=LinStatic RunCase={}\nTABLE: \"COMBINATION DEFINITIONS\"\n \
         ComboName=ULS ComboType=\"Linear Add\" CaseName=D ScaleFactor=1.2\n \
         ComboName=ULS ComboType=\"Linear Add\" CaseName=L ScaleFactor=1.6\n \
         ComboName=SERV ComboType=\"Linear Add\" CaseName=D ScaleFactor=1\n \
         ComboName=OK ComboType=\"Linear Add\" CaseName=SERV ScaleFactor=2\n \
         ComboName=X0 ComboType=\"Linear Add\" CaseName=ULS ScaleFactor=1\n \
         ComboName=X1 ComboType=\"Linear Add\" CaseName=X0 ScaleFactor=1\nEND TABLE DATA\n",
        if run_l { "Yes" } else { "No" }
    )
}

fn history_text() -> String {
    format!(
        "{}{HEADER_HISTORY}",
        r#"TABLE: "PROGRAM CONTROL"
 CurrUnits="kN, m, C"
TABLE: "JOINT COORDINATES"
 Joint=J XorR=0 Y=0 Z=0
TABLE: "JOINT ADDED MASS ASSIGNMENTS"
 Joint=J Mass1=1
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
"#
    )
}

const HEADER_HISTORY: &str = r#"TABLE: "LOAD CASE DEFINITIONS"
 Case=D Type=LinStatic RunCase=Yes
 Case=L Type=LinStatic RunCase=No
 Case=H Type=LinDynamic RunCase=Yes NumSteps=100 StepSize=0.02
TABLE: "FUNCTION - TIME HISTORY - USER"
 Name=F Time=0 Value=0
 Name=F Time=0.2 Value=1
 Name=F Time=0.4 Value=0
TABLE: "CASE - TIME HISTORY 2 - LOAD ASSIGNMENTS"
 Case=H Function=F LoadType="Load pattern" LoadName=D SF=1
TABLE: "COMBINATION DEFINITIONS"
 ComboName=ENVELOPE ComboType=Envelope CaseName=H ScaleFactor=1
 ComboName=SUM ComboType="Linear Add" CaseName=D ScaleFactor=1
 ComboName=SUM ComboType="Linear Add" CaseName=H ScaleFactor=1
 ComboName=NESTED ComboType="Linear Add" CaseName=SUM ScaleFactor=1
 ComboName=DONLY ComboType="Linear Add" CaseName=D ScaleFactor=3
END TABLE DATA
"#;

fn fx(m: &SapModel, name: &str) -> f64 {
    m.case_results[name].base_reactions[0]
}

// ---- D79: names -------------------------------------------------------------------

#[test]
fn the_s2k_reader_refuses_a_case_and_a_combination_with_one_name() {
    let err = parse_s2k(&shadow_text()).unwrap_err().to_string();
    assert!(err.contains("'D'"), "{err}");
    assert!(err.contains("load case") && err.contains("load combination"), "{err}");
    assert!(err.contains("LinearStatic"), "names the case type: {err}");
}

#[test]
fn analyze_model_refuses_the_collision_and_publishes_nothing_for_a_model_built_otherwise() {
    // A model that did not come from the S2K reader (the API, the editor, a native file).
    let mut m = parse_s2k(&healthy_text(false)).unwrap();
    m.load_combos.insert(
        "D".into(),
        LoadCombo {
            name: "D".into(),
            combo_type: ComboType::LinearAdd,
            cases: vec![("L".into(), 1.0)],
        },
    );
    m.load_combos.insert(
        "TOP".into(),
        LoadCombo {
            name: "TOP".into(),
            combo_type: ComboType::LinearAdd,
            cases: vec![("D".into(), 2.0)],
        },
    );
    let err = analyze_model(&mut m).unwrap_err().to_string();
    assert!(err.contains("'D'") && err.contains("load case") && err.contains("load combination"), "{err}");
    // Nothing is published: TOP must not exist (the case value -20 kN was published before),
    // neither must D, SERV or the model be locked.
    assert!(m.case_results.is_empty() && !m.is_locked);
    assert!(!m.case_results.contains_key("TOP"));
}

#[test]
fn design_cannot_consume_the_shadowed_name_or_top() {
    // Stored results of a model analyzed before the name was shared: the combination 'D'
    // would be skipped by design because a result named 'D' exists.
    let mut m = parse_s2k(&healthy_text(true)).unwrap();
    analyze_model(&mut m).unwrap();
    m.load_combos.insert(
        "D".into(),
        LoadCombo {
            name: "D".into(),
            combo_type: ComboType::LinearAdd,
            cases: vec![("L".into(), 1.0)],
        },
    );
    let err = try_run_steel_design(&mut m).unwrap_err();
    match &err {
        DesignError::UnavailableCombination { combo, reason, .. } => {
            assert_eq!(combo, "D");
            assert!(reason.contains("load case") && reason.contains("load combination"), "{reason}");
        }
        other => panic!("wrong error {other:?}"),
    }
    assert!(try_run_concrete_design(&mut m).is_err());
    assert!(m.steel_design_results.is_empty());
}

#[test]
fn the_cli_refuses_the_shadow_model_and_writes_no_output() {
    let dir = std::env::temp_dir().join(format!("mb_shadow_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let (input, output) = (dir.join("shadow.s2k"), dir.join("shadow.json"));
    std::fs::write(&input, shadow_text()).unwrap();
    let out = std::process::Command::new(env!("CARGO_BIN_EXE_structurax"))
        .args(["analyze", input.to_str().unwrap(), "--output", output.to_str().unwrap()])
        .output()
        .unwrap();
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(!out.status.success(), "{err}");
    assert!(err.contains("'D'") && err.contains("load case") && err.contains("load combination"), "{err}");
    assert!(!output.exists(), "TOP must not be published in any file");
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn positive_control_healthy_siblings_and_nested_parents_survive() {
    let mut m = parse_s2k(&healthy_text(true)).unwrap();
    analyze_model(&mut m).unwrap();
    assert!(m.unavailable_results.is_empty());
    assert!((fx(&m, "ULS") + 60.0).abs() < 1e-9, "1.2*10 + 1.6*30 = 60 kN");
    assert!((fx(&m, "OK") + 20.0).abs() < 1e-9);
    assert!((fx(&m, "X1") + 60.0).abs() < 1e-9, "nested parent of ULS");
}

#[test]
fn missing_contributor_propagates_and_independent_siblings_survive() {
    let mut m = parse_s2k(&healthy_text(false)).unwrap();
    analyze_model(&mut m).unwrap();
    for gone in ["ULS", "X0", "X1"] {
        assert!(!m.case_results.contains_key(gone), "{gone}");
        assert!(m.unavailable_results[gone].contains("'L'"), "{gone}");
    }
    for kept in ["D", "SERV", "OK"] {
        assert!((fx(&m, kept).abs() - [10.0, 10.0, 20.0][["D", "SERV", "OK"].iter().position(|k| *k == kept).unwrap()]).abs() < 1e-9);
    }
}

// ---- D80: history -----------------------------------------------------------------

#[test]
fn history_combinations_are_withheld_not_published_from_one_snapshot() {
    let mut m = parse_s2k(&history_text()).unwrap();
    analyze_model(&mut m).unwrap();
    // The stored series is what proves the snapshot is not an envelope.
    let h = &m.case_results["H"];
    assert_eq!(h.time_steps.len(), 101);
    let series: Vec<f64> = h.time_history_disp["J"].iter().map(|u| u[0]).collect();
    let (lo, hi) = series.iter().fold((f64::MAX, f64::MIN), |(l, h), v| (l.min(*v), h.max(*v)));
    assert!((lo + 0.1409388942).abs() < 1e-6 && (hi - 0.1415610469).abs() < 1e-6, "{lo} {hi}");
    // Neither Envelope(H) nor D + H nor a parent of it is published: not +0.1415610469 as
    // a minimum, and not a sum of a static state and a snapshot.
    for name in ["ENVELOPE", "SUM", "NESTED"] {
        assert!(!m.case_results.contains_key(name), "{name} must be withheld");
        assert!(m.unavailable_results.contains_key(name), "{name}");
    }
    let reason = &m.unavailable_results["ENVELOPE"];
    assert!(reason.contains("'ENVELOPE'") && reason.contains("'H'"), "{reason}");
    assert!(reason.contains("extrema over time are not stored"), "{reason}");
    assert!(m.unavailable_results["SUM"].contains("'SUM'"));
    assert!(m.unavailable_results["NESTED"].contains("'SUM'"));
    // Positive control: a combination of the static case alone is unaffected (3 * -10 kN).
    assert!((fx(&m, "DONLY") + 30.0).abs() < 1e-9);
    assert!(!m.unavailable_results.contains_key("DONLY"));
}

#[test]
fn api_getters_and_tables_report_a_history_combination_as_unavailable() {
    let mut api = sap_api::cSapModel::new(Units::KnMC);
    api.inner = parse_s2k(&history_text()).unwrap();
    api.results().setup_deselect_all_cases_and_combos_for_output();
    assert_eq!(api.results().setup_set_combo_selected_for_output("ENVELOPE", true), 0);
    assert_eq!(api.analyze().run_analysis(), 0);
    assert_eq!(api.analyze().unavailable_results().iter().filter(|(n, _)| n == "ENVELOPE").count(), 1);
    assert_ne!(api.results().base_react().0, 0);
    assert!(api.results().joint_displ("J").is_empty());
    let (code, _, rows, _) = api
        .database_tables()
        .get_table_for_display_array_vec("JOINT DISPLACEMENTS", &["OutputCase"], "ALL");
    assert_ne!(code, 0);
    assert_eq!(rows, 0);
    // Selecting it after the run is refused by name.
    assert_ne!(api.results().setup_set_combo_selected_for_output("SUM", true), 0);
}

fn cantilever_with_history_combo() -> SapModel {
    let mut m = SapModel::new(Units::KipInF);
    m.load_cases.clear();
    m.load_patterns.clear();
    m.load_patterns.insert(
        "D".into(),
        LoadPattern {
            name: "D".into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
    m.load_cases.insert("D".into(), LoadCase::linear_static("D", "D", 1.0));
    let sec = make_i_section("W14X90", "A992Fy50", 14.0, 14.5, 0.710, 0.440, 14.5, 0.710);
    m.frame_sections.insert(sec.name.clone(), sec);
    m.add_point("A", 0.0, 0.0, 0.0);
    m.add_point("B", 120.0, 0.0, 0.0);
    m.points.get_mut("A").unwrap().restraint = [true; 6];
    m.add_frame("BM", "A", "B", "W14X90");
    m.frames.get_mut("BM").unwrap().loads_dist.push(FrameLoadDistributed {
        load_pat: "D".into(),
        my_type: 1,
        dir: 6,
        csys: "GLOBAL".into(),
        rel_dist1: 0.0,
        rel_dist2: 1.0,
        val1: -0.05,
        val2: -0.05,
    });
    m.load_combos.insert(
        "DS".into(),
        LoadCombo {
            name: "DS".into(),
            combo_type: ComboType::LinearAdd,
            cases: vec![("D".into(), 1.4)],
        },
    );
    m
}

#[test]
fn design_refuses_a_combination_with_a_history_contributor_by_name() {
    // Positive control: the static combination designs.
    let mut m = cantilever_with_history_combo();
    analyze_model(&mut m).unwrap();
    try_run_steel_design(&mut m).expect("static combination designs");
    // A history case H with the results of D stored under its name (a snapshot, as the
    // solver stores a history) and a combination that lists it.
    let mut h = LoadCase::linear_static("H", "D", 1.0);
    h.case_type = LoadCaseType::LinearHistory;
    m.load_cases.insert("H".into(), h);
    let snapshot = m.case_results["D"].clone();
    m.case_results.insert("H".into(), snapshot);
    m.load_combos.insert(
        "ENVH".into(),
        LoadCombo {
            name: "ENVH".into(),
            combo_type: ComboType::Envelope,
            cases: vec![("H".into(), 1.0)],
        },
    );
    let err = try_run_steel_design(&mut m).unwrap_err();
    match &err {
        DesignError::UnavailableCombination { combo, reason, .. } => {
            assert_eq!(combo, "ENVH");
            assert!(reason.contains("'H'") && reason.contains("not stored"), "{reason}");
        }
        other => panic!("wrong error {other:?}"),
    }
    assert!(matches!(
        try_run_concrete_design(&mut m).unwrap_err(),
        DesignError::UnavailableCombination { .. }
    ));
}
