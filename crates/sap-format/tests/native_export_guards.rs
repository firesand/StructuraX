use sap_format::sdb::native::{inspect_native_sdb, pack_model_to_native_sdb, save_native_sdb_file};
use sap_format::{extract_sdb_to_model, parse_s2k, write_s2k, FormatError};
use sap_model::*;
use std::sync::atomic::{AtomicU64, Ordering};

static TEMP_SEQUENCE: AtomicU64 = AtomicU64::new(0);

fn supported_model() -> SapModel {
    let mut m = SapModel::default();
    m.add_point("A", 0., 0., 0.);
    m.add_point("B", 12., 0., 0.);
    m.add_frame("F", "A", "B", "FSEC1");
    m
}
fn assert_rejected_unchanged(m: &SapModel, expected: &str) {
    let before = m.clone();
    let error = pack_model_to_native_sdb(m).unwrap_err();
    assert!(
        matches!(&error, FormatError::UnsupportedNativeExport { .. }),
        "{error}"
    );
    assert!(
        error.to_string().contains(expected),
        "{error} missing {expected}"
    );
    assert_eq!(*m, before);
    let path = std::env::temp_dir().join(format!(
        "structurax-native-guard-{}-{}-{}.sdb",
        std::process::id(),
        TEMP_SEQUENCE.fetch_add(1, Ordering::Relaxed),
        expected.replace(['.', '[', ']', '/'], "_")
    ));
    std::fs::write(&path, b"existing destination stays intact").unwrap();
    assert!(save_native_sdb_file(m, &path).is_err());
    assert_eq!(
        std::fs::read(&path).unwrap(),
        b"existing destination stays intact"
    );
    std::fs::remove_file(path).unwrap();
}
#[test]
fn supported_steel_model_exports_with_independent_topology_and_assignment_checks() {
    let mut m = supported_model();
    m.points["A"].restraint = [true; 6];
    m.points["B"].loads_force.push(PointLoadForce {
        load_pat: "DEAD".into(),
        csys: "GLOBAL".into(),
        force: [0., 0., -5., 0., 0., 0.],
    });
    m.frames["F"].loads_temp.push(FrameLoadTemperature {
        load_pat: "DEAD".into(),
        my_type: 1,
        val: 10.,
        pattern_name: None,
    });
    let native = pack_model_to_native_sdb(&m).unwrap();
    let doc = inspect_native_sdb(&native).unwrap();
    assert_eq!(doc.inventory().parsed_bytes, doc.inventory().stream_bytes);
    let r = extract_sdb_to_model(&native).unwrap();
    assert_eq!(r.frames["F"].point_i, "A");
    assert_eq!(r.frames["F"].point_j, "B");
    assert_eq!(r.points["A"].restraint, [true; 6]);
    assert_eq!(r.points["B"].loads_force[0].force[2], -5.);
    assert_eq!(r.frames["F"].loads_temp[0].pattern_name, None);
    assert_eq!(r.frames["F"].loads_temp[0].val, 10.);
}
#[test]
fn opaque_source_metadata_is_not_silently_discarded() {
    let m=parse_s2k("TABLE: \"JOINT COORDINATES\"\n Joint=A XorR=0 Y=0 Z=0 VendorNote=keep\nTABLE: \"OPAQUE VENDOR\"\n Value=21\n").unwrap();
    assert_rejected_unchanged(&m, "s2k_preservation");
}
#[test]
fn handwritten_recognized_s2k_exports_without_discarding_source_metadata() {
    let input = r#"$ Handwritten source; spelling and quoting need not be canonical.
TABLE: "JOINT COORDINATES"
 Joint="A" XorR=0 Y=0 Z=0
 Joint=B XorR=12 Y=0 Z=0
TABLE: "CONNECTIVITY - FRAME"
 Frame=F JointI=A JointJ=B
TABLE: "FRAME SECTION ASSIGNMENTS"
 Frame=F AnalSect=FSEC1
TABLE: "JOINT LOADS - FORCE"
END TABLE DATA
"#;
    let mut m = parse_s2k(input).unwrap();
    let original = m.clone();
    assert!(m.s2k_preservation.is_some());
    let native = pack_model_to_native_sdb(&m).unwrap();
    assert_eq!(m, original, "export must retain the caller's source text");
    let restored = extract_sdb_to_model(&native).unwrap();
    assert_eq!(restored.frames["F"].point_i, "A");
    assert_eq!(restored.frames["F"].point_j, "B");
    assert_eq!(restored.materials["4000Psi"], m.materials["4000Psi"]);
    // A supported edit remains eligible; the provenance baseline describes the
    // imported source, while the native comparison checks the current model.
    m.points["B"].x = 18.;
    let edited = extract_sdb_to_model(&pack_model_to_native_sdb(&m).unwrap()).unwrap();
    assert_eq!(edited.points["B"].x, 18.);
}
#[test]
fn native_material_color_bytes_and_yield_fallback_are_explicit() {
    let m = supported_model();
    let native = pack_model_to_native_sdb(&m).unwrap();
    let doc = inspect_native_sdb(&native).unwrap();
    let colors: Vec<_> = doc
        .records()
        .iter()
        .filter(|record| record.class_id == "S_Material")
        .map(|record| {
            let field = record
                .fields
                .iter()
                .find(|field| field.name.as_deref() == Some("m_color"))
                .unwrap();
            i32::from_le_bytes(
                native.decompressed_stream[field.offset..field.offset + 4]
                    .try_into()
                    .unwrap(),
            )
        })
        .collect();
    assert_eq!(colors, [0x00ff0000, 0x0000ff00]);
    let restored = extract_sdb_to_model(&native).unwrap();
    assert_eq!(restored.materials["4000Psi"].fys, 60.);
    assert_eq!(restored.materials["4000Psi"].color, "Green");
    for color in [
        "Black",
        "Red",
        "Yellow",
        "Magenta",
        "Cyan",
        "White",
        "Gray",
        "ColorRef:123456",
    ] {
        let mut edited = m.clone();
        edited.materials["A992Fy50"].color = color.into();
        edited.materials["A992Fy50"].fy = 0.;
        edited.materials["A992Fy50"].fys = 0.;
        let restored = extract_sdb_to_model(&pack_model_to_native_sdb(&edited).unwrap()).unwrap();
        assert_eq!(restored.materials["A992Fy50"].color, color);
        assert_eq!(restored.materials["A992Fy50"].fys, 0.);
    }
}
#[test]
fn source_syntax_changes_to_generated_s2k_remain_exportable() {
    let source = format!("$ alternate whitespace\n{}", write_s2k(&supported_model()));
    let parsed = parse_s2k(&source).unwrap();
    assert!(parsed.s2k_preservation.is_some());
    assert!(pack_model_to_native_sdb(&parsed).is_ok());
}
#[test]
fn translated_named_coordinates_with_authoritative_globals_remain_exportable() {
    let source = r#"TABLE: "COORDINATE SYSTEMS"
 Name=GLOBAL X=0 Y=0 Z=0
 Name=CS1 X=10 Y=20 Z=30
TABLE: "JOINT COORDINATES"
 Joint=A CoordSys=CS1 CoordType=Cartesian XorR=0 Y=0 Z=0 GlobalX=10 GlobalY=20 GlobalZ=30
 Joint=B CoordSys=CS1 CoordType=Cartesian XorR=12 Y=0 Z=0 GlobalX=22 GlobalY=20 GlobalZ=30
TABLE: "CONNECTIVITY - FRAME"
 Frame=F JointI=A JointJ=B
TABLE: "FRAME LOADS - POINT"
 Frame=F LoadPat=DEAD CoordSys=GLOBAL Dir=4 RelDist=0.5 Force=2
TABLE: "MATERIAL PROPERTIES 01 - GENERAL"
 Material=A992Fy50 Type=Steel
 Material=4000Psi Type=Conc
"#;
    let parsed = parse_s2k(source).unwrap();
    let restored = extract_sdb_to_model(&pack_model_to_native_sdb(&parsed).unwrap()).unwrap();
    assert_eq!(restored.coord_systems["CS1"].origin, [10., 20., 30.]);
    assert_eq!((restored.points["A"].x, restored.points["B"].x), (10., 22.));
    assert_eq!(restored.points["A"].y, 20.);
    assert_eq!(restored.points["A"].z, 30.);
    assert_eq!(restored.frames["F"].loads_point[0].dir, 4);
    assert_eq!(restored.materials["4000Psi"].mat_type, MatType::Concrete);
}
#[test]
fn translated_row_never_whitelists_another_rows_unverified_coordinate_metadata() {
    for unverified in [
        "CoordSys=ROTATED CoordType=Cartesian",
        "CoordSys=GLOBAL CoordType=Cylindrical",
    ] {
        let source = format!(
            r#"TABLE: "COORDINATE SYSTEMS"
 Name=GLOBAL X=0 Y=0 Z=0
 Name=CS1 X=10 Y=20 Z=30
 Name=ROTATED X=0 Y=0 Z=0 AboutZ=90
TABLE: "JOINT COORDINATES"
 Joint=A CoordSys=CS1 CoordType=Cartesian XorR=0 Y=0 Z=0 GlobalX=10 GlobalY=20 GlobalZ=30
 Joint=B {unverified} XorR=0 Y=0 Z=0 GlobalX=0 GlobalY=0 GlobalZ=0
"#
        );
        let parsed = parse_s2k(&source).unwrap();
        assert_rejected_unchanged(&parsed, "s2k_preservation");
    }
}
#[test]
fn individually_opaque_tables_columns_and_unprojected_rows_still_reject() {
    for source in [
        "TABLE: \"JOINT COORDINATES\"\n Joint=A XorR=0 Y=0 Z=0 VendorNote=keep\n",
        "TABLE: \"JOINT COORDINATES\"\n Joint=A XorR=0 Y=0 Z=0\nTABLE: \"OPAQUE VENDOR\"\n Value=21\n",
        "TABLE: \"JOINT COORDINATES\"\n Joint=A XorR=0 Y=0 Z=0\nTABLE: \"JOINT LOADS - FORCE\"\n Joint=MISSING LoadPat=DEAD F1=99\n",
        "TABLE: \"JOINT COORDINATES\"\n Joint=A XorR=0 Y=0 Z=0\n Joint=A XorR=9 Y=0 Z=0\n",
        "TABLE: \"JOINT COORDINATES\"\n Joint=A XorR=0 GlobalX=9 Y=0 Z=0\n",
        "Program VendorAssignment=keep\nTABLE: \"JOINT COORDINATES\"\n Joint=A XorR=0 Y=0 Z=0\n",
        "TABLE: \"JOINT COORDINATES\"\n Joint=A XorR=0 Y=0 Z=0\nEND TABLE DATA\n VendorAssignment=keep\n",
        "TABLE: \"MATERIAL PROPERTIES 01 - GENERAL\"\n Material=A992Fy50 Type=UnknownMaterial\n",
        "TABLE: \"MATERIAL PROPERTIES 01 - GENERAL\"\n Material=A992Fy50 Type=Steel SymType=Orthotropic\n",
    ] {
        let m = parse_s2k(source).unwrap();
        assert_rejected_unchanged(&m, "s2k_preservation");
    }
}
#[test]
fn original_physical_assignment_losses_are_rejected_before_write() {
    let mut m = supported_model();
    m.project_info.insert("Owner".into(), "keep".into());
    assert_rejected_unchanged(&m, "project_info");
    let mut m = supported_model();
    m.points["B"].spring_coupled = Some([2.; 21]);
    assert_rejected_unchanged(&m, "spring_coupled");
    let mut m = supported_model();
    m.points["B"].local_axes_matrix = Some([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]);
    assert_rejected_unchanged(&m, "local_axes_matrix");
    let mut m = supported_model();
    m.materials["A992Fy50"].e_ortho = Some([29000., 22000., 13000.]);
    assert_rejected_unchanged(&m, "orthotropic");
}
#[test]
fn incomplete_nonlinear_source_cannot_be_promoted_by_native_roundtrip() {
    for active in [false, true] {
        let mut m = supported_model();
        let mut p = LinkProp {
            name: "G".into(),
            link_type: LinkPropType::Gap,
            nonlinear_data_complete: false,
            ..LinkProp::default()
        };
        p.dof_nonlinear[0] = active;
        p.k_nl[0] = 100.;
        p.param1[0] = 1.;
        m.link_props.insert("G".into(), p);
        assert_rejected_unchanged(&m, "nonlinear_data_complete");
    }
}
#[test]
fn missing_references_are_not_substituted_or_dropped() {
    let mut m = supported_model();
    m.frames["F"].point_j = "MISSING".into();
    assert_rejected_unchanged(&m, "MISSING");
    let mut m = supported_model();
    m.frames["F"].section = "MISSING".into();
    assert_rejected_unchanged(&m, "MISSING");
    let mut m = supported_model();
    m.frame_sections["FSEC1"].material = "MISSING".into();
    assert_rejected_unchanged(&m, "MISSING");
    let mut m = supported_model();
    m.points["B"].loads_force.push(PointLoadForce {
        load_pat: "MISSING".into(),
        csys: "GLOBAL".into(),
        force: [1.; 6],
    });
    assert_rejected_unchanged(&m, "MISSING");
    let mut m = supported_model();
    m.points["B"].loads_force.push(PointLoadForce {
        load_pat: "DEAD".into(),
        csys: "MISSING".into(),
        force: [1.; 6],
    });
    assert_rejected_unchanged(&m, "MISSING");
    let mut m = supported_model();
    m.points["B"].constraints.push("MISSING".into());
    assert_rejected_unchanged(&m, "MISSING");
    let mut m = supported_model();
    m.load_cases["DEAD"].initial_case = Some("MISSING".into());
    assert_rejected_unchanged(&m, "MISSING");
    let mut m = supported_model();
    m.load_combos.insert(
        "C".into(),
        LoadCombo {
            name: "C".into(),
            combo_type: ComboType::LinearAdd,
            cases: vec![("MISSING".into(), 1.)],
        },
    );
    assert_rejected_unchanged(&m, "MISSING");
}
#[test]
fn semantic_backstop_rejects_other_unimplemented_fields_and_late_failure_keeps_file() {
    let mut m = supported_model();
    m.points["B"].special_point = true;
    assert_rejected_unchanged(&m, "special_point");
    let mut m = supported_model();
    m.materials["A992Fy50"].color = "VendorPurple".into();
    assert_rejected_unchanged(&m, "color");
    let mut m = supported_model();
    m.materials["4000Psi"].fys = 72.;
    assert_rejected_unchanged(&m, "fys");
}
#[test]
fn nonfinite_numeric_values_and_index_truncation_are_rejected() {
    let mut m = supported_model();
    m.points["B"].x = f64::INFINITY;
    assert_rejected_unchanged(&m, "nonfinite");
    let mut m = supported_model();
    m.frames["F"].cable_params = Some(CableParams {
        undeformed_length: Some(f64::INFINITY),
        target_tension_i: None,
        horizontal_tension: None,
        num_segments: 1,
    });
    assert_rejected_unchanged(&m, "finite");
    let mut m = supported_model();
    m.frames["F"].cable_params = Some(CableParams {
        undeformed_length: Some(12.),
        target_tension_i: None,
        horizontal_tension: None,
        num_segments: usize::MAX,
    });
    assert_rejected_unchanged(&m, "num_segments");
}

#[test]
fn optional_nan_cable_quantity_is_rejected_without_becoming_none_or_touching_file() {
    let mut m = supported_model();
    m.frames["F"].cable_params = Some(CableParams {
        undeformed_length: Some(f64::NAN),
        target_tension_i: None,
        horizontal_tension: None,
        num_segments: 1,
    });
    let bits = m.frames["F"]
        .cable_params
        .as_ref()
        .unwrap()
        .undeformed_length
        .unwrap()
        .to_bits();
    let path =
        std::env::temp_dir().join(format!("structurax-native-nan-{}.sdb", std::process::id()));
    std::fs::write(&path, b"unchanged").unwrap();
    let error = save_native_sdb_file(&m, &path).unwrap_err();
    assert!(error.to_string().contains("finite"));
    assert_eq!(
        m.frames["F"]
            .cable_params
            .as_ref()
            .unwrap()
            .undeformed_length
            .unwrap()
            .to_bits(),
        bits
    );
    assert_eq!(std::fs::read(&path).unwrap(), b"unchanged");
    std::fs::remove_file(path).unwrap();
}

#[test]
fn generated_class_versions_match_independently_observed_v19_tags() {
    let mut m = supported_model();
    let prop = LinkProp {
        name: "L".into(),
        ..LinkProp::default()
    };
    m.link_props.insert("L".into(), prop);
    let native = pack_model_to_native_sdb(&m).unwrap();
    let doc = inspect_native_sdb(&native).unwrap();
    for (class, expected) in [
        ("RUNARRAY", 2),
        ("S_MaterialPropertiesSteel", 2),
        ("NLLINKPROPARRAY", 3),
        ("LOADPATTERNARRAY", 3),
        ("SOLUTIONCONTROLARRAY", 3),
        ("ANALYSISCASEARRAY", 3),
    ] {
        let record = doc.records().iter().find(|r| r.class_id == class).unwrap();
        let field = record
            .fields
            .iter()
            .find(|f| f.name.as_deref() == Some("ClassVersion"))
            .unwrap();
        let actual = i32::from_le_bytes(
            native.decompressed_stream[field.offset..field.offset + 4]
                .try_into()
                .unwrap(),
        );
        assert_eq!(actual, expected, "{class}");
    }
}
