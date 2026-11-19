//! Wave 5 P10: the seismic eccentricity overwrite table in S2K (D39) and the refusal of the native
//! writer for it, and the Ritz starting-load record of type 4 of a native Modal case (D36).

use sap_format::sdb::native::{inspect_native_sdb, pack_model_to_native_sdb};
use sap_format::{extract_sdb_to_model, parse_s2k, write_s2k, FormatError, SdbFile};
use sap_model::*;
use sap_units::Units;

const TABLE: &str = "RUST AUTO SEISMIC ECCENTRICITY";

fn building() -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    m.load_patterns.insert(
        "EQ".into(),
        LoadPattern {
            name: "EQ".into(),
            pat_type: LoadPatternType::Quake,
            self_wt_multiplier: 0.0,
        },
    );
    m.constraints.insert(
        "D1".into(),
        ConstraintDef {
            name: "D1".into(),
            constraint_type: ConstraintType::Diaphragm,
            axis: ConstraintAxis::Z,
            dofs: [true, true, false, false, false, true],
        },
    );
    m.constraints.insert(
        "D2".into(),
        ConstraintDef {
            name: "D2".into(),
            constraint_type: ConstraintType::Diaphragm,
            axis: ConstraintAxis::Z,
            dofs: [true, true, false, false, false, true],
        },
    );
    m
}

fn with_overwrites() -> SapModel {
    let mut m = building();
    let table = m.auto_seismic_eccentricity.entry("EQ".into()).or_default();
    table.insert("D1".into(), 0.4);
    table.insert("D2".into(), -0.1234567890123);
    m
}

fn import_error(text: &str, needles: &[&str]) {
    let error = parse_s2k(text).unwrap_err();
    assert!(matches!(error, FormatError::ParseS2k(_)), "{error}");
    for needle in needles {
        assert!(error.to_string().contains(needle), "'{needle}' missing from: {error}");
    }
}

/// Text of a model that has the table with one extra row appended to it.
fn text_with_row(row: &str) -> String {
    let text = write_s2k(&with_overwrites());
    let header = format!("TABLE:  \"{TABLE}\"\n");
    assert_eq!(text.matches(&header).count(), 1, "{text}");
    text.replacen(&header, &format!("{header}{row}\n"), 1)
}

#[test]
fn the_overwrite_table_round_trips_exactly_and_is_written_only_when_present() {
    let m = with_overwrites();
    let text = write_s2k(&m);
    assert!(text.contains(TABLE), "{text}");
    let back = parse_s2k(&text).unwrap();
    assert_eq!(back.auto_seismic_eccentricity, m.auto_seismic_eccentricity);
    assert_eq!(back, parse_s2k(&write_s2k(&back)).unwrap());
    // Insertion order of the diaphragms survives.
    let keys: Vec<_> = back.auto_seismic_eccentricity["EQ"].keys().cloned().collect();
    assert_eq!(keys, ["D1", "D2"]);

    // No overwrite, no table; an empty inner table counts as none.
    assert!(!write_s2k(&building()).contains(TABLE));
    let mut empty = building();
    empty.auto_seismic_eccentricity.entry("EQ".into()).or_default();
    assert!(!write_s2k(&empty).contains(TABLE));
}

/// An imported model keeps its source text; edits of the overwrites must still reach the export.
#[test]
fn an_edited_or_removed_overwrite_is_what_the_next_export_writes() {
    let mut m = parse_s2k(&write_s2k(&with_overwrites())).unwrap();
    m.auto_seismic_eccentricity["EQ"]["D1"] = 0.75;
    let back = parse_s2k(&write_s2k(&m)).unwrap();
    assert_eq!(back.auto_seismic_eccentricity["EQ"]["D1"], 0.75);
    assert_eq!(back.auto_seismic_eccentricity["EQ"]["D2"], -0.1234567890123);
    m.auto_seismic_eccentricity.clear();
    let text = write_s2k(&m);
    assert!(!text.contains("Distance="), "{text}");
    assert!(parse_s2k(&text).unwrap().auto_seismic_eccentricity.is_empty());
}

#[test]
fn the_distance_is_a_length_in_the_units_of_the_file() {
    // 0.4 m written in kN, m is 400 mm in kN, mm: the model conversion scales it as a length and
    // the file states the units it is written in.
    let mut m = with_overwrites();
    m.convert_units_in_place(Units::KnMmC);
    assert_eq!(m.auto_seismic_eccentricity["EQ"]["D1"], 0.4 * 1000.0);
    let text = write_s2k(&m);
    assert!(text.contains("Distance=400"), "{text}");
    let back = parse_s2k(&text).unwrap();
    assert_eq!(back.present_units, Units::KnMmC);
    assert_eq!(back.auto_seismic_eccentricity["EQ"]["D1"], 400.0);
}

#[test]
fn a_row_naming_an_unknown_pattern_or_constraint_is_refused_with_the_line() {
    let row = "   LoadPat=NOSUCH   Diaphragm=D1   Distance=0.1";
    import_error(&text_with_row(row), &[TABLE, "NOSUCH", row.trim()]);
    let row = "   LoadPat=EQ   Diaphragm=GHOST   Distance=0.1";
    import_error(&text_with_row(row), &[TABLE, "GHOST", row.trim()]);
    // A pattern that exists but is not a constraint's name, and the reverse.
    let row = "   LoadPat=D1   Diaphragm=EQ   Distance=0.1";
    import_error(&text_with_row(row), &[TABLE, row.trim()]);
}

#[test]
fn malformed_rows_are_refused_naming_the_table() {
    import_error(&text_with_row("   LoadPat=EQ   Diaphragm=D1"), &[TABLE, "Distance"]);
    import_error(
        &text_with_row("   LoadPat=EQ   Diaphragm=D1   Distance=abc"),
        &[TABLE, "abc"],
    );
    import_error(
        &text_with_row("   LoadPat=EQ   Diaphragm=D1   Distance=NaN"),
        &[TABLE, "NaN"],
    );
    import_error(&text_with_row("   Diaphragm=D1   Distance=0.1"), &[TABLE, "LoadPat"]);
    import_error(&text_with_row("   LoadPat=EQ   Distance=0.1"), &[TABLE, "Diaphragm"]);
    // The same diaphragm twice for one pattern is ambiguous.
    import_error(
        &text_with_row("   LoadPat=EQ   Diaphragm=D1   Distance=0.1"),
        &[TABLE, "D1", "repeats"],
    );
}

#[test]
fn the_native_writer_refuses_overwrites_naming_the_pattern() {
    let m = with_overwrites();
    let before = m.clone();
    let error = pack_model_to_native_sdb(&m).unwrap_err();
    assert!(
        matches!(&error, FormatError::UnsupportedNativeExport { field, .. } if field == "auto_seismic_eccentricity.EQ"),
        "{error}"
    );
    assert!(error.to_string().contains("S2K"), "{error}");
    assert_eq!(m, before);
    // Without the overwrites the same model is written.
    pack_model_to_native_sdb(&building()).unwrap();
}

// ---- D36 ----------------------------------------------------------------------------------

/// A native file of one Ritz Modal case `MODAL` with one load assignment, whose record type and
/// load index are then overwritten in the stream.
fn native_with_modal_assignment(ltype: i32, load: i32) -> SdbFile {
    let mut m = SapModel::default();
    m.add_point("A", 0., 0., 0.);
    m.add_point("B", 12., 0., 0.);
    m.add_frame("F", "A", "B", "FSEC1");
    m.load_cases.clear();
    let mut modal = LoadCase::linear_static("MODAL", "DEAD", 1.0);
    modal.case_type = LoadCaseType::Modal;
    modal.modal_sub_type = ModalSubType::Ritz;
    modal.loads = vec![LoadCaseAssignment { load_name: "DEAD".into(), scale_factor: 1.0 }];
    m.load_cases.insert("MODAL".into(), modal);
    let mut file = pack_model_to_native_sdb(&m).unwrap();
    let doc = inspect_native_sdb(&file).unwrap();
    let record = doc
        .records()
        .iter()
        .find(|r| {
            r.fields.iter().any(|f| f.name.as_deref() == Some("m_load"))
                && r.fields.iter().any(|f| f.name.as_deref() == Some("m_ltype"))
        })
        .expect("the load assignment record");
    for (name, value) in [("m_ltype", ltype), ("m_load", load)] {
        let field = record
            .fields
            .iter()
            .find(|f| f.name.as_deref() == Some(name))
            .unwrap();
        assert_eq!(field.length, 4, "{name}");
        file.decompressed_stream[field.offset..field.offset + 4]
            .copy_from_slice(&value.to_le_bytes());
    }
    file
}

#[test]
fn the_pattern_starting_load_of_a_modal_case_still_imports_without_an_entry() {
    let m = extract_sdb_to_model(&native_with_modal_assignment(1, 1)).unwrap();
    assert!(m.unmodeled_source.is_empty(), "{:?}", m.unmodeled_source);
    assert_eq!(m.load_cases["MODAL"].loads.len(), 1);
}

#[test]
fn the_link_nonlinear_starting_vectors_are_recorded_naming_the_case() {
    let m = extract_sdb_to_model(&native_with_modal_assignment(4, -1)).unwrap();
    assert_eq!(m.unmodeled_source.len(), 1, "{:?}", m.unmodeled_source);
    let entry = &m.unmodeled_source[0];
    assert_eq!(entry.scope, UnmodeledScope::Case("MODAL".into()));
    for needle in ["type 4", "link element nonlinear", "'MODAL'"] {
        assert!(entry.description.contains(needle), "{}", entry.description);
    }
    assert!(m.load_cases["MODAL"].loads.is_empty());
}

#[test]
fn any_other_starting_load_type_of_a_modal_case_is_recorded_naming_the_type() {
    for ltype in [0, 3, 5, 7] {
        let m = extract_sdb_to_model(&native_with_modal_assignment(ltype, 3)).unwrap();
        assert_eq!(m.unmodeled_source.len(), 1, "type {ltype}: {:?}", m.unmodeled_source);
        let entry = &m.unmodeled_source[0];
        assert_eq!(entry.scope, UnmodeledScope::Case("MODAL".into()));
        assert!(
            entry.description.contains(&format!("type {ltype}")),
            "{}",
            entry.description
        );
    }
}

/// The entry survives S2K and refuses the case at analysis through the existing mechanism; the
/// native writer refuses the model.
#[test]
fn the_entry_survives_s2k_and_stops_the_native_writer() {
    let m = extract_sdb_to_model(&native_with_modal_assignment(4, -1)).unwrap();
    let back = parse_s2k(&write_s2k(&m)).unwrap();
    assert_eq!(back.unmodeled_source, m.unmodeled_source);
    assert_eq!(back.unmodeled_source_for_case("MODAL").len(), 1);
    assert!(matches!(
        pack_model_to_native_sdb(&m),
        Err(FormatError::UnsupportedNativeExport { .. })
    ));
}
