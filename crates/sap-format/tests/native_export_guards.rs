use sap_format::sdb::native::{inspect_native_sdb, pack_model_to_native_sdb, save_native_sdb_file};
use sap_format::{extract_sdb_to_model, parse_s2k, FormatError};
use sap_model::*;

fn supported_model() -> SapModel {
    let mut m = SapModel::default();
    m.materials.shift_remove("4000Psi");
    m.area_sections.clear();
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
        "structurax-native-guard-{}-{}.sdb",
        std::process::id(),
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
    m.materials["A992Fy50"].color = "Green".into();
    assert_rejected_unchanged(&m, "color");
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
