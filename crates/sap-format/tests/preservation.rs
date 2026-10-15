use sap_format::{
    extract_sdb_to_model, pack_model_to_sdb, parse_s2k, parse_s2k_with_report,
    save_s2k_file_with_report, try_write_s2k_with_report, FormatError,
};
use sap_model::{LinkProp, LinkPropType, SapModel};
use sap_units::Units;

const SOURCE: &str = "$ original banner\r\nTABLE: \"PROGRAM CONTROL\"\r\n CurrUnits=\"Kip, in, F\" VendorMode=\"keep me\"\r\nTABLE: \"JOINT COORDINATES\"\r\n Joint=\"A space\" GlobalX=1 GlobalY=2 GlobalZ=3 VendorNote=\"a \"\"quote\"\" and λ\" _\r\n   VendorNumber=12.500\r\n Joint=B XorR=7 Y=8 Z=9 VendorNote=second\r\nTABLE: \"VENDOR OPAQUE TABLE\"\r\n ; exact comment\r\n Weird=\"first\" Weird=\"duplicate\"\r\n   raw syntax without equal signs\r\nTABLE: \"VENDOR OPAQUE TABLE\"\r\n Key=\"another occurrence\"\r\nEND TABLE DATA\r\n$ tail\r\n";

#[test]
fn unedited_roundtrip_retains_every_source_byte_duplicate_table_and_opaque_field() {
    let report = parse_s2k_with_report(SOURCE).unwrap();
    assert!(report
        .warnings
        .iter()
        .any(|w| w.contains("unknown columns") && w.contains("VENDORNOTE")));
    assert!(report
        .warnings
        .iter()
        .any(|w| w.contains("VENDOR OPAQUE TABLE") && w.contains("preserved")));
    assert_eq!(
        try_write_s2k_with_report(&report.model).unwrap().text,
        SOURCE
    );
    let json = serde_json::to_string(&report.model).unwrap();
    let restored: SapModel = serde_json::from_str(&json).unwrap();
    assert_eq!(try_write_s2k_with_report(&restored).unwrap().text, SOURCE);
    let private = pack_model_to_sdb(&restored).unwrap();
    let restored = extract_sdb_to_model(&private).unwrap();
    assert_eq!(try_write_s2k_with_report(&restored).unwrap().text, SOURCE);
}

#[test]
fn coordinate_edit_removes_stale_alias_and_reattaches_exact_unknown_tokens_once() {
    let mut model = parse_s2k(SOURCE).unwrap();
    model.points["A space"].x = 42.;
    let export = try_write_s2k_with_report(&model).unwrap();
    assert!(export.text.contains("VendorNote=\"a \"\"quote\"\" and λ\""));
    assert!(export.text.contains("VendorNumber=12.500"));
    assert!(!export.text.contains("GlobalX=1"));
    assert_eq!(export.text.matches("VendorNumber=12.500").count(), 1);
    assert!(export.text.contains("TABLE: \"VENDOR OPAQUE TABLE\"\r\n ; exact comment\r\n Weird=\"first\" Weird=\"duplicate\"\r\n   raw syntax without equal signs\r\n"));
    let reopened = parse_s2k(&export.text).unwrap();
    assert_eq!(reopened.points["A space"].x, 42.);
    assert_eq!(reopened.points["A space"].y, 2.);
    assert_eq!(reopened.points["B"].x, 7.);
    assert_eq!(
        try_write_s2k_with_report(&reopened).unwrap().text,
        export.text
    );
}

#[test]
fn new_and_deleted_rows_do_not_receive_other_owners_unknown_fields() {
    let mut model = parse_s2k(SOURCE).unwrap();
    model.points.shift_remove("B");
    model.add_point("NEW", 10., 11., 12.);
    let export = try_write_s2k_with_report(&model).unwrap();
    assert!(!export.text.contains("VendorNote=second"));
    assert!(export.warnings.iter().any(|w| w.contains("deleted owner")));
    let newrow = export
        .text
        .lines()
        .find(|line| line.contains("Joint=NEW"))
        .unwrap();
    assert!(!newrow.contains("Vendor"));
    assert_eq!(parse_s2k(&export.text).unwrap().points.len(), 2);
}

#[test]
fn unknown_column_owner_ambiguity_is_rejected_before_overwriting_a_file() {
    let source="TABLE: \"JOINT COORDINATES\"\n Joint=A XorR=0 Y=0 Z=0 Xtra=one\n Joint=A XorR=1 Y=0 Z=0 Xtra=two\n";
    let mut model = parse_s2k(source).unwrap();
    assert_eq!(try_write_s2k_with_report(&model).unwrap().text, source);
    model.points["A"].x = 2.;
    assert!(matches!(
        try_write_s2k_with_report(&model),
        Err(FormatError::PreservationConflict(_))
    ));
    let path = std::env::temp_dir().join(format!("rea-preservation-{}.s2k", std::process::id()));
    std::fs::write(&path, "original destination").unwrap();
    assert!(save_s2k_file_with_report(&model, &path).is_err());
    assert_eq!(
        std::fs::read_to_string(&path).unwrap(),
        "original destination"
    );
    std::fs::remove_file(path).unwrap();
}

#[test]
fn unknown_dimensioned_data_blocks_unit_conversion_until_explicitly_discarded() {
    let mut model = parse_s2k(SOURCE).unwrap();
    model.convert_units_in_place(Units::KnMC);
    assert!(try_write_s2k_with_report(&model)
        .unwrap_err()
        .to_string()
        .contains("unit changes"));
    model.s2k_preservation = None;
    let reopened = parse_s2k(&try_write_s2k_with_report(&model).unwrap().text).unwrap();
    assert_eq!(reopened.present_units, Units::KnMC);
    assert!((reopened.points["A space"].x - 0.0254).abs() < 1e-12);
}

#[test]
fn linear_table_unverified_nonlinear_flags_are_not_inferred_from_nonzero_stiffness() {
    let source="TABLE: \"LINK PROPERTY DEFINITIONS 01 - GENERAL\"\n Link=G LinkType=Gap\nTABLE: \"LINK PROPERTY DEFINITIONS 02 - LINEAR\"\n Link=G DOF=U1 Active=Yes Fixed=No Ke=2 Knl=10 Param1=3\nTABLE: \"LINK PROPERTY DEFINITIONS 04 - GAP\"\n Link=G DOF=U1 Nonlinear=Yes Opening=4\n";
    let report = parse_s2k_with_report(source).unwrap();
    let prop = &report.model.link_props["G"];
    assert_eq!(prop.dof_nonlinear, [false; 6]);
    assert!(!prop.nonlinear_data_complete);
    assert!(report.warnings.iter().any(|w| w.contains("incomplete")));
    assert_eq!(
        try_write_s2k_with_report(&report.model).unwrap().text,
        source
    );
}

#[test]
fn explicit_reconstruction_nonlinear_flags_zero_stiffness_and_completeness_roundtrip() {
    let mut model = SapModel::default();
    let mut prop = LinkProp {
        name: "G".into(),
        link_type: LinkPropType::Gap,
        ..LinkProp::default()
    };
    prop.dof_nonlinear[0] = true;
    prop.k_nl[0] = 0.;
    prop.param1[0] = 3.;
    model.link_props.insert("G".into(), prop);
    let export = try_write_s2k_with_report(&model).unwrap();
    assert!(export
        .warnings
        .iter()
        .any(|w| w.contains("reconstruction extension")));
    let reopened = parse_s2k(&export.text).unwrap();
    assert!(reopened.link_props["G"].dof_nonlinear[0]);
    assert!(reopened.link_props["G"].nonlinear_data_complete);
    assert_eq!(reopened.link_props["G"].k_nl[0], 0.);
    model.link_props["G"].nonlinear_data_complete = false;
    let text = try_write_s2k_with_report(&model).unwrap().text;
    assert!(!parse_s2k(&text).unwrap().link_props["G"].nonlinear_data_complete);
}

#[test]
fn uninterpreted_fields_that_writer_fabricates_are_preserved_without_duplicate_keys() {
    let source="TABLE: \"MATERIAL PROPERTIES 01 - GENERAL\"\n Material=S Type=Steel\nTABLE: \"MATERIAL PROPERTIES 03A - STEEL DATA\"\n Material=S Fy=50 Fu=65 EffFy=57.25 EffFu=76.75 Vendor=\"actual strengths\"\n";
    let mut model = parse_s2k(source).unwrap();
    model.materials["S"].fy = 60.;
    let export = try_write_s2k_with_report(&model).unwrap();
    assert!(export.text.contains("EffFy=57.25"));
    assert!(export.text.contains("EffFu=76.75"));
    let row = export
        .text
        .lines()
        .find(|line| line.contains("Vendor=\"actual strengths\""))
        .unwrap();
    assert_eq!(row.matches("EffFy=").count(), 1);
    assert_eq!(parse_s2k(&export.text).unwrap().materials["S"].fy, 60.);
}

#[test]
fn changing_absolute_load_intervals_writes_one_canonical_relative_interval() {
    let source="TABLE: \"JOINT COORDINATES\"\n Joint=A XorR=0 Y=0 Z=0\n Joint=B XorR=12 Y=0 Z=0\nTABLE: \"CONNECTIVITY - FRAME\"\n Frame=F JointI=A JointJ=B\nTABLE: \"FRAME LOADS - POINT\"\n Frame=F LoadPat=DEAD Type=Force Dir=Gravity DistType=AbsDist AbsDist=3 Force=2 Vendor=\"span identity\"\n";
    let mut model = parse_s2k(source).unwrap();
    model.frames["F"].loads_point[0].rel_dist = 0.5;
    let export = try_write_s2k_with_report(&model).unwrap();
    let row = export
        .text
        .lines()
        .find(|line| line.contains("Vendor=\"span identity\""))
        .unwrap();
    assert!(!row.contains("AbsDist="));
    assert!(row.contains("RelDist=0.5"));
    assert_eq!(
        parse_s2k(&export.text).unwrap().frames["F"].loads_point[0].rel_dist,
        0.5
    );
}

#[test]
fn unprojected_orphan_rows_are_diagnosed_and_cannot_be_erased_by_a_known_load_edit() {
    let source="TABLE: \"JOINT COORDINATES\"\n Joint=A XorR=0 Y=0 Z=0\nTABLE: \"JOINT LOADS - FORCE\"\n Joint=A LoadPat=DEAD F1=1\n Joint=MISSING LoadPat=DEAD F1=99\n";
    let report = parse_s2k_with_report(source).unwrap();
    assert!(report
        .warnings
        .iter()
        .any(|w| w.contains("source records absent")));
    assert_eq!(
        try_write_s2k_with_report(&report.model).unwrap().text,
        source
    );
    let mut model = report.model;
    model.points["A"].loads_force[0].force[0] = 2.;
    assert!(try_write_s2k_with_report(&model)
        .unwrap_err()
        .to_string()
        .contains("unprojected record"));
}

#[test]
fn function_ordinate_and_sample_aliases_never_override_edited_canonical_values() {
    for (table, samples, ordinate) in [
        ("FUNCTION - TIME HISTORY - USER", "Time", "Accel"),
        ("FUNCTION - TIME HISTORY - USER", "Period", "Accel"),
        ("FUNCTION - RESPONSE SPECTRUM - USER", "Time", "Value"),
    ] {
        let source=format!("TABLE: \"{table}\"\n Name=F {samples}=0 {ordinate}=1 Note=first\n Name=F {samples}=1 {ordinate}=3 Note=second\n");
        let mut model = parse_s2k(&source).unwrap();
        model.functions["F"].points[0].1 = 2.;
        let export = try_write_s2k_with_report(&model).unwrap();
        let reopened = parse_s2k(&export.text).unwrap();
        assert_eq!(reopened.functions["F"].points, vec![(0., 2.), (1., 3.)]);
        assert!(export.text.contains("Note=first"));
        assert!(export.text.contains("Note=second"));
    }
}
