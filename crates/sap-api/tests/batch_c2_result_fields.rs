//! Batch C2 (D130): a stored concrete design result written before the torsion fields existed
//! still loads. This test lives here because `sap-design` has no JSON dependency and the batch
//! may not add one.
use sap_model::ConcreteDesignResult;

#[test]
fn a_stored_result_without_the_torsion_fields_loads_with_zeros_and_no_notes() {
    let json = r#"{"frame_name":"F","section_name":"S","design_code":"ACI 318-14",
        "governing_combo":"C","is_column":false,"as_top":1.0,"as_bot":2.0,"as_side":0.0,
        "av_over_s":0.05,"pmm_ratio":0.0,"min_rebar_area":0.5,"status":"OK"}"#;
    let r: ConcreteDesignResult = serde_json::from_str(json).unwrap();
    assert_eq!(r.at_over_s, 0.0);
    assert_eq!(r.al_torsion, 0.0);
    assert!(r.notes.is_empty());
    // an empty list is not written back; a note is
    assert!(!serde_json::to_string(&r).unwrap().contains("notes"));
    let mut with_note = r.clone();
    with_note.notes.push("n".into());
    let back: ConcreteDesignResult =
        serde_json::from_str(&serde_json::to_string(&with_note).unwrap()).unwrap();
    assert_eq!(back.notes, vec!["n".to_string()]);
}
