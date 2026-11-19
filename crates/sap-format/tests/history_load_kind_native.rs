//! D77 in the native SDB: a history record's `m_ltype` (1 load pattern, 2
//! acceleration) is read into the stored kind and written from it, so a pattern
//! named U1 and an acceleration U1 stay different through a save and reopen.

use sap_format::sdb::native::pack_model_to_native_sdb;
use sap_format::extract_sdb_to_model;
use sap_model::*;

fn model(kind: HistoryLoadKind, load: &str) -> SapModel {
    let mut m = SapModel::default();
    m.add_point("A", 0., 0., 0.);
    m.add_point("B", 12., 0., 0.);
    m.add_frame("F", "A", "B", "FSEC1");
    m.load_patterns.insert(
        "U1".into(),
        LoadPattern {
            name: "U1".into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.,
        },
    );
    m.functions.insert(
        "RAMP".into(),
        DynamicFunction {
            name: "RAMP".into(),
            is_time_history: true,
            damping_ratio: 0.,
            points: vec![(0., 0.), (1., 1.)],
        },
    );
    let mut modal = LoadCase::linear_static("MODES", "DEAD", 1.0);
    modal.case_type = LoadCaseType::Modal;
    modal.loads.clear();
    m.load_cases.insert("MODES".into(), modal);
    let mut history = LoadCase::linear_static("MHIST", "DEAD", 1.0);
    history.case_type = LoadCaseType::LinearHistory;
    history.loads.clear();
    history.push_history_load("RAMP", load, kind, 1.);
    history.modal_damping = 0.05;
    history.modal_case = Some("MODES".into());
    m.load_cases.insert("MHIST".into(), history);
    m
}

fn back(m: &SapModel) -> SapModel {
    extract_sdb_to_model(&pack_model_to_native_sdb(m).unwrap()).unwrap()
}

#[test]
fn a_pattern_named_like_an_acceleration_stays_a_pattern() {
    let b = back(&model(HistoryLoadKind::Pattern, "U1"));
    assert_eq!(b.load_cases["MHIST"].history_kind(0), HistoryLoadKind::Pattern);
    assert_eq!(b.load_cases["MHIST"].history_loads[0].1, "U1");
}

#[test]
fn an_acceleration_stays_an_acceleration_beside_a_pattern_of_the_same_name() {
    // A pattern U1 exists in the model; the entry is the acceleration U1.
    let b = back(&model(HistoryLoadKind::Acceleration, "U1"));
    assert_eq!(b.load_cases["MHIST"].history_kind(0), HistoryLoadKind::Acceleration);
    assert_eq!(b.load_cases["MHIST"].history_loads[0].1, "U1");
    // The record type is in the file, so the reader needs no name to decide.
    let b2 = back(&b);
    assert_eq!(b2.load_cases["MHIST"].history_kind(0), HistoryLoadKind::Acceleration);
}

#[test]
fn a_legacy_entry_that_could_be_either_is_refused_by_the_native_writer() {
    let err = pack_model_to_native_sdb(&model(HistoryLoadKind::Unspecified, "U1"))
        .unwrap_err()
        .to_string();
    assert!(
        err.contains("MHIST") && err.contains("'U1'") && err.contains("load pattern"),
        "{err}"
    );
}
