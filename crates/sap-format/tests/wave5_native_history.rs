//! Wave 5 P4 (D07): the modal case of a native modal-history case record round-trips, and the
//! combinations the writer does not carry are still refused.

use sap_format::sdb::native::{inspect_native_sdb, pack_model_to_native_sdb};
use sap_format::{extract_sdb_to_model, FormatError};
use sap_model::*;

fn model_with_history() -> SapModel {
    let mut m = SapModel::default();
    m.add_point("A", 0., 0., 0.);
    m.add_point("B", 12., 0., 0.);
    m.add_frame("F", "A", "B", "FSEC1");
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
    history.push_history_load("RAMP", "DEAD", HistoryLoadKind::Pattern, 1.);
    history.modal_damping = 0.05;
    history.modal_case = Some("MODES".into());
    m.load_cases.insert("MHIST".into(), history);
    m
}

/// `m_modalcase` is written as the 1-based index of the Modal case among the analysis cases and
/// read back as its name (the index convention of the vendor files: Example 6-001 MHISTA has
/// type 5 with modalcase 2 naming MODAL).
#[test]
fn a_linear_history_keeps_its_modal_case_through_the_native_writer() {
    let m = model_with_history();
    let bytes = pack_model_to_native_sdb(&m).unwrap();
    let back = extract_sdb_to_model(&bytes).unwrap();
    assert_eq!(back.load_cases["MHIST"].modal_case.as_deref(), Some("MODES"));
    assert_eq!(back.load_cases["MHIST"].modal_damping, 0.05);
    assert_eq!(back.load_cases["MODES"].modal_case, None);
    assert!(back.unmodeled_source.is_empty(), "{:?}", back.unmodeled_source);
    assert_eq!(back.load_cases["MHIST"], m.load_cases["MHIST"]);

    // The index is in the record itself: m_modalcase of MHIST holds the position of MODES.
    let doc = inspect_native_sdb(&bytes).unwrap();
    let position = |name: &str| m.load_cases.keys().position(|k| k == name).unwrap() as i32 + 1;
    let modal_fields: Vec<_> = doc
        .records()
        .iter()
        .flat_map(|r| r.fields.iter())
        .filter(|f| f.name.as_deref() == Some("m_modalcase"))
        .collect();
    assert_eq!(modal_fields.len(), m.load_cases.len());
    let stream = &bytes.decompressed_stream;
    let values: Vec<i32> = modal_fields
        .iter()
        .map(|f| i32::from_le_bytes(stream[f.offset..f.offset + 4].try_into().unwrap()))
        .collect();
    let expected: Vec<i32> = m
        .load_cases
        .keys()
        .map(|k| if k == "MHIST" { position("MODES") } else { 0 })
        .collect();
    assert_eq!(values, expected);
}

/// A history without a modal case (direct integration, Rayleigh damping) is written with
/// index 0 and comes back without a modal case and without an unmodeled entry.
#[test]
fn a_direct_integration_history_roundtrips_without_a_modal_case() {
    let mut m = model_with_history();
    m.load_cases["MHIST"].modal_case = None;
    m.load_cases["MHIST"].modal_damping = 0.;
    m.load_cases["MHIST"].rayleigh_damping = (0.0741, 0.0002);
    let back = extract_sdb_to_model(&pack_model_to_native_sdb(&m).unwrap()).unwrap();
    assert_eq!(back.load_cases["MHIST"], m.load_cases["MHIST"]);
    assert!(back.unmodeled_source.is_empty());
}

/// Negative: a modal case that is missing or is not a Modal case, and a modal case on any other
/// case type, are still refused by the export guard, naming the case.
#[test]
fn modal_cases_the_writer_does_not_carry_are_refused_by_name() {
    let refused = |m: &SapModel, field: &str| {
        let before = m.clone();
        let error = pack_model_to_native_sdb(m).unwrap_err();
        assert!(matches!(&error, FormatError::UnsupportedNativeExport { .. }), "{error}");
        assert!(error.to_string().contains(field), "{error}");
        assert_eq!(*m, before);
    };
    let mut m = model_with_history();
    m.load_cases["MHIST"].modal_case = Some("NOSUCH".into());
    refused(&m, "load_cases.MHIST.modal_case");
    let mut m = model_with_history();
    m.load_cases["MHIST"].modal_case = Some("DEAD".into());
    refused(&m, "load_cases.MHIST.modal_case");
    let mut m = model_with_history();
    m.load_cases["MHIST"].case_type = LoadCaseType::NonlinearHistory;
    refused(&m, "load_cases.MHIST.modal_case");
    let mut m = model_with_history();
    m.load_cases["MODES"].modal_case = Some("MODES".into());
    refused(&m, "load_cases.MODES.modal_case");
}
