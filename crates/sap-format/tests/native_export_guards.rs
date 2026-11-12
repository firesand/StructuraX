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
    m.points["B"].spring = [10., 0., 0., 0., 0., 0.];
    m.points["B"].spring_coupled = Some([2.; 21]);
    assert_rejected_unchanged(&m, "spring_coupled");
    let mut m = supported_model();
    let mut diag_only = [0.; 21];
    diag_only[0] = 25.;
    m.points["B"].spring_coupled = Some(diag_only);
    assert_rejected_unchanged(&m, "spring_coupled");
    let mut m = supported_model();
    m.points["B"].local_axes_matrix = Some([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]);
    assert_rejected_unchanged(&m, "local_axes_matrix");
    let mut m = supported_model();
    m.materials["A992Fy50"].e_ortho = Some([29000., 22000., 13000.]);
    assert_rejected_unchanged(&m, "orthotropic");
    let mut m = supported_model();
    {
        let mat = m.materials.get_mut("A992Fy50").unwrap();
        mat.e_ortho = Some([28000., 22000., 13000.]);
        mat.g_ortho = Some([mat.g, 8500., 5000.]);
        mat.u_ortho = Some([mat.u, 0.25, 0.20]);
        mat.a_ortho = Some([mat.a, 7.0e-6, 8.0e-6]);
    }
    assert_rejected_unchanged(&m, "orthotropic");
    let mut m = supported_model();
    m.frames["F"].cable_params = Some(CableParams {
        undeformed_length: Some(11.95),
        target_tension_i: Some(50.0),
        horizontal_tension: None,
        num_segments: 4,
    });
    assert_rejected_unchanged(&m, "cable_params");
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

#[test]
fn coupled_and_local_joint_springs_roundtrip_through_native_sdb() {
    let mut coupled = [0.0; 21];
    for (idx, slot) in coupled.iter_mut().enumerate() {
        *slot = (idx as f64 + 1.0) * 12.5;
    }

    for is_local in [false, true] {
        let mut m = supported_model();
        m.points["A"].spring = [100.0, 0.0, 250.0, 0.0, 80000.0, 0.0];
        m.points["A"].spring_is_local = is_local;
        m.points["B"].spring_coupled = Some(coupled);
        m.points["B"].spring_is_local = !is_local;

        let native = pack_model_to_native_sdb(&m).unwrap();
        let doc = inspect_native_sdb(&native).unwrap();
        assert_eq!(doc.inventory().parsed_bytes, doc.inventory().stream_bytes);

        let spring_records: Vec<_> = doc
            .records()
            .iter()
            .filter(|r| r.class_id == "TYSpring")
            .collect();
        assert_eq!(spring_records.len(), 2);
        for (rec, expected_local) in spring_records.iter().zip([is_local, !is_local]) {
            let ver_field = rec
                .fields
                .iter()
                .find(|f| f.name.as_deref() == Some("ClassVersion"))
                .unwrap();
            let ver = i32::from_le_bytes(
                native.decompressed_stream[ver_field.offset..ver_field.offset + 4]
                    .try_into()
                    .unwrap(),
            );
            assert_eq!(ver, 1);

            let csys_field = rec
                .fields
                .iter()
                .find(|f| f.name.as_deref() == Some("m_csys"))
                .unwrap();
            let csys = i32::from_le_bytes(
                native.decompressed_stream[csys_field.offset..csys_field.offset + 4]
                    .try_into()
                    .unwrap(),
            );
            assert_eq!(csys, if expected_local { -1 } else { 1 });
        }

        let restored = extract_sdb_to_model(&native).unwrap();
        assert_eq!(restored, m);
        assert_eq!(parse_s2k(&write_s2k(&restored)).unwrap(), m);
    }
}

#[test]
fn complete_orthotropic_material_roundtrips_through_native_sdb() {
    let mut m = supported_model();
    {
        let mat = m.materials.get_mut("A992Fy50").unwrap();
        mat.e = 29000.0;
        mat.e_ortho = Some([29000.0, 18000.0, 12000.0]);
        mat.g = 11153.846;
        mat.g_ortho = Some([11153.846, 7500.0, 4800.0]);
        mat.u = 0.30;
        mat.u_ortho = Some([0.30, 0.24, 0.18]);
        mat.a = 6.5e-6;
        mat.a_ortho = Some([6.5e-6, 7.2e-6, 8.1e-6]);
    }

    let native = pack_model_to_native_sdb(&m).unwrap();
    let doc = inspect_native_sdb(&native).unwrap();
    assert_eq!(doc.inventory().parsed_bytes, doc.inventory().stream_bytes);

    let first_mat = doc
        .records()
        .iter()
        .find(|r| r.class_id == "S_Material")
        .unwrap();
    let sym_field = first_mat
        .fields
        .iter()
        .find(|f| f.name.as_deref() == Some("m_SymType"))
        .unwrap();
    let sym = i32::from_le_bytes(
        native.decompressed_stream[sym_field.offset..sym_field.offset + 4]
            .try_into()
            .unwrap(),
    );
    assert_eq!(sym, 1);

    let restored = extract_sdb_to_model(&native).unwrap();
    assert_eq!(restored, m);
    assert_eq!(parse_s2k(&write_s2k(&restored)).unwrap(), m);
}

#[test]
fn cable_m_cable_type_modes_roundtrip_through_native_sdb() {
    let specs = [
        (
            8_i32,
            CableParams {
                undeformed_length: Some(11.95),
                target_tension_i: None,
                horizontal_tension: None,
                num_segments: 8,
            },
        ),
        (
            3_i32,
            CableParams {
                undeformed_length: None,
                target_tension_i: Some(52.5),
                horizontal_tension: None,
                num_segments: 12,
            },
        ),
        (
            5_i32,
            CableParams {
                undeformed_length: None,
                target_tension_i: None,
                horizontal_tension: Some(41.25),
                num_segments: 16,
            },
        ),
    ];

    for (expected_code, params) in specs {
        let mut m = supported_model();
        let mut cab_sec = FrameSection::default();
        cab_sec.name = "CAB1".into();
        cab_sec.material = "A992Fy50".into();
        cab_sec.prop_type = FramePropType::Circle;
        cab_sec.area = 0.5;
        cab_sec.t3 = (4.0 * 0.5 / std::f64::consts::PI).sqrt();
        m.frame_sections.insert("CAB1".into(), cab_sec);
        m.frames["F"].section = "CAB1".into();
        m.frames["F"].cable_params = Some(params.clone());

        let native = pack_model_to_native_sdb(&m).unwrap();
        let doc = inspect_native_sdb(&native).unwrap();
        let cable_rec = doc
            .records()
            .iter()
            .find(|r| r.class_id == "S_CableData")
            .unwrap();
        let ctype_field = cable_rec
            .fields
            .iter()
            .find(|f| f.name.as_deref() == Some("m_CableType"))
            .unwrap();
        let actual_code = i32::from_le_bytes(
            native.decompressed_stream[ctype_field.offset..ctype_field.offset + 4]
                .try_into()
                .unwrap(),
        );
        assert_eq!(actual_code, expected_code);

        let restored = extract_sdb_to_model(&native).unwrap();
        assert_eq!(restored.frames["F"].cable_params, Some(params));
        assert_eq!(restored, m);
    }
}

fn wen_pushover_model() -> SapModel {
    let mut m = supported_model();
    m.points["A"].restraint = [true; 6];
    let mut prop = LinkProp {
        name: "PLAW".into(),
        link_type: LinkPropType::PlasticWen,
        ..LinkProp::default()
    };
    prop.dof_active = [false, true, false, false, false, false];
    prop.dof_nonlinear[1] = true;
    prop.ke = [0.; 6];
    prop.k_nl[1] = 100.;
    prop.param1[1] = 50.;
    prop.param2[1] = 0.1;
    prop.param3 = [1., 1., 2., 2., 2., 2.];
    m.link_props.insert("PLAW".into(), prop);
    m.links.insert(
        "L".into(),
        LinkObj {
            name: "L".into(),
            point_i: "B".into(),
            point_j: None,
            prop: "PLAW".into(),
            angle_deg: 0.,
            selected: false,
        },
    );
    let mut push = LoadCase::linear_static("PUSH", "DEAD", 1.);
    push.case_type = LoadCaseType::NonlinearStatic;
    push.displacement_control = Some(DisplacementControl {
        target: 10.,
        monitored: Some(("B".into(), 2)),
        conjugate: true,
    });
    m.load_cases.insert("PUSH".into(), push);
    let mut back = LoadCase::linear_static("BACK", "DEAD", -1.);
    back.case_type = LoadCaseType::NonlinearStatic;
    back.initial_case = Some("PUSH".into());
    back.displacement_control = Some(DisplacementControl {
        target: 20.,
        monitored: Some(("B".into(), 2)),
        conjugate: false,
    });
    m.load_cases.insert("BACK".into(), back);
    m
}

#[test]
fn wen_exponent_and_displacement_control_roundtrip_natively_and_through_s2k() {
    let m = wen_pushover_model();
    let native = pack_model_to_native_sdb(&m).unwrap();
    let doc = inspect_native_sdb(&native).unwrap();
    assert_eq!(doc.inventory().parsed_bytes, doc.inventory().stream_bytes);
    // One monitored-displacement record per displacement-controlled case.
    assert_eq!(doc.inventory().classes["TYNLCDisp"], 2);
    let r = extract_sdb_to_model(&native).unwrap();
    assert_eq!(r.link_props["PLAW"], m.link_props["PLAW"]);
    assert_eq!(r.load_cases["PUSH"], m.load_cases["PUSH"]);
    assert_eq!(r.load_cases["BACK"], m.load_cases["BACK"]);
    assert!(r.load_cases["DEAD"].displacement_control.is_none());

    let text = write_s2k(&m);
    assert!(text.contains("RustDisplTarget=10") && text.contains("RustDisplDOF=U3"));
    assert!(text.contains("Param3=2"));
    let s = parse_s2k(&text).unwrap();
    assert_eq!(s.link_props["PLAW"], m.link_props["PLAW"]);
    assert_eq!(s.load_cases["PUSH"], m.load_cases["PUSH"]);
    assert_eq!(s.load_cases["BACK"], m.load_cases["BACK"]);
    assert!(parse_s2k(&text.replace("RustDisplDOF=U3", "RustDisplDOF=UZ")).is_err());

    // A model reopened from that S2K text keeps its source for preservation;
    // the extension columns must count as represented data, so native
    // synthesis and a second S2K save still work and agree.
    let native_again = pack_model_to_native_sdb(&s).unwrap();
    let r2 = extract_sdb_to_model(&native_again).unwrap();
    assert_eq!(r2.link_props["PLAW"], m.link_props["PLAW"]);
    assert_eq!(r2.load_cases["PUSH"], m.load_cases["PUSH"]);
    let report = sap_format::try_write_s2k_with_report(&s).unwrap();
    let again = parse_s2k(&report.text).unwrap();
    assert_eq!(again.link_props["PLAW"], m.link_props["PLAW"]);
    assert_eq!(again.load_cases["BACK"], m.load_cases["BACK"]);
    assert!(!report
        .warnings
        .iter()
        .any(|w| w.contains("Param3") || w.contains("RustDispl")));
}

#[test]
fn unrepresentable_displacement_control_and_parameter_rows_are_not_exported() {
    let mut m = wen_pushover_model();
    m.load_cases["PUSH"].displacement_control.as_mut().unwrap().monitored = None;
    assert_rejected_unchanged(&m, "load_cases.PUSH.displacement_control");
    let mut m = wen_pushover_model();
    m.load_cases["PUSH"].displacement_control.as_mut().unwrap().monitored =
        Some(("MISSING".into(), 2));
    assert_rejected_unchanged(&m, "load_cases.PUSH.displacement_control");
    let mut m = wen_pushover_model();
    m.load_cases["PUSH"].displacement_control.as_mut().unwrap().monitored = Some(("B".into(), 6));
    assert_rejected_unchanged(&m, "load_cases.PUSH.displacement_control");
    let mut m = wen_pushover_model();
    m.load_cases["PUSH"].case_type = LoadCaseType::LinearStatic;
    assert_rejected_unchanged(&m, "load_cases.PUSH.displacement_control");
    // The third parameter row is only written for the Wen law; elsewhere it
    // would be lost on reimport, so the roundtrip check refuses the export.
    let mut m = wen_pushover_model();
    m.link_props["PLAW"].link_type = LinkPropType::Gap;
    assert_rejected_unchanged(&m, "link_props.PLAW.param3");
}

fn unmodeled(scope: UnmodeledScope, description: &str) -> UnmodeledSourceData {
    UnmodeledSourceData {
        scope,
        description: description.into(),
    }
}

#[test]
fn unmodeled_source_entries_survive_s2k_and_block_native_synthesis() {
    let mut m = supported_model();
    m.unmodeled_source = vec![
        unmodeled(
            UnmodeledScope::AllCases,
            "advanced local axes of link '1' (only the roll angle is imported)",
        ),
        unmodeled(UnmodeledScope::NonlinearCases, "2 frame hinge assignment(s)"),
        unmodeled(UnmodeledScope::DynamicCases, "2 frame added-mass assignments"),
        unmodeled(
            UnmodeledScope::LoadPattern("WIND X-1".into()),
            "gravity load on link \"A\" in load pattern 'WIND X-1'",
        ),
        unmodeled(
            UnmodeledScope::Case("NLMHIST1".into()),
            "per-mode or interpolated modal damping of case 'NLMHIST1'",
        ),
    ];
    let text = write_s2k(&m);
    assert!(text.contains("TABLE:  \"RUST UNMODELED SOURCE DATA\""));
    let reopened = parse_s2k(&text).unwrap();
    assert_eq!(reopened.unmodeled_source, m.unmodeled_source);
    // A second save from the reopened model (with its retained source) agrees.
    let again = parse_s2k(&sap_format::try_write_s2k_with_report(&reopened).unwrap().text).unwrap();
    assert_eq!(again.unmodeled_source, m.unmodeled_source);

    // Discarding one entry from a model that still retains its S2K source: the
    // save must drop that row, keep the others, and not bring the row back.
    // (A source that differs from the canonical text is retained; a comment
    // line is enough.)
    let mut edited = parse_s2k(&format!("$ handwritten note\n{text}")).unwrap();
    assert!(edited.s2k_preservation.is_some());
    assert_eq!(edited.unmodeled_source, m.unmodeled_source);
    let discarded = edited.unmodeled_source.remove(1);
    assert_eq!(discarded.description, "2 frame hinge assignment(s)");
    let saved = sap_format::try_write_s2k_with_report(&edited).unwrap();
    assert!(!saved.text.contains("frame hinge assignment"));
    let after = parse_s2k(&saved.text).unwrap();
    assert_eq!(after.unmodeled_source, edited.unmodeled_source);
    assert_eq!(after.unmodeled_source.len(), 4);
    // Discarding all of them removes the table.
    edited.unmodeled_source.clear();
    let saved = sap_format::try_write_s2k_with_report(&edited).unwrap();
    assert!(!saved.text.contains("RUST UNMODELED SOURCE DATA"));
    assert!(parse_s2k(&saved.text).unwrap().unmodeled_source.is_empty());

    // Synthesis would write a file that silently lacks the data.
    assert_rejected_unchanged(&m, "unmodeled_source");
    assert!(pack_model_to_native_sdb(&reopened)
        .unwrap_err()
        .to_string()
        .contains("unmodeled_source"));
    // Discarding the entries is what makes the model exportable.
    m.unmodeled_source.clear();
    pack_model_to_native_sdb(&m).unwrap();

    // A row that cannot be understood must not read as "nothing is missing".
    for (from, to) in [
        ("Scope=Nonlinear", "Scope=Sometimes"),
        ("Scope=Pattern   Target=\"WIND X-1\"", "Scope=Pattern   Target=\"\""),
        ("Description=\"2 frame hinge assignment(s)\"", "Description=\"\""),
    ] {
        assert!(text.contains(from), "{from}");
        assert!(parse_s2k(&text.replace(from, to)).is_err(), "{to}");
    }
}

/// A modal history whose damping is carried as Rayleigh coefficients is written
/// with the "direct coefficients" damping type, so a reader does not take the
/// coefficients for inactive data next to a zero constant ratio.
#[test]
fn proportional_modal_history_damping_keeps_its_native_damping_type() {
    let mut m = supported_model();
    m.functions.insert(
        "RAMP".into(),
        DynamicFunction {
            name: "RAMP".into(),
            is_time_history: true,
            damping_ratio: 0.,
            points: vec![(0., 0.), (1., 1.)],
        },
    );
    let mut history = LoadCase::linear_static("MHIST", "DEAD", 1.);
    history.loads.clear();
    history.case_type = LoadCaseType::LinearHistory;
    history.history_loads = vec![("RAMP".into(), "DEAD".into(), 1.)];
    history.modal_damping = 0.;
    history.rayleigh_damping = (0.0741, 0.0002);
    m.load_cases.insert("MHIST".into(), history);
    let recovered = extract_sdb_to_model(&pack_model_to_native_sdb(&m).unwrap()).unwrap();
    assert_eq!(recovered.load_cases["MHIST"], m.load_cases["MHIST"]);

    // A constant ratio together with coefficients has no single native
    // damping type; the readback differs and the export is refused.
    m.load_cases["MHIST"].modal_damping = 0.02;
    assert_rejected_unchanged(&m, "load_cases.MHIST.modal_damping");
}

#[test]
fn s2k_modal_history_cases_default_to_zero_modal_damping() {
    let mut m = supported_model();
    let mut history = LoadCase::linear_static("H", "DEAD", 1.);
    history.case_type = LoadCaseType::LinearHistory;
    history.modal_damping = 0.;
    m.load_cases.insert("H".into(), history);
    let mut spectrum_default = LoadCase::linear_static("S", "DEAD", 1.);
    spectrum_default.case_type = LoadCaseType::NonlinearStatic;
    m.load_cases.insert("S".into(), spectrum_default);
    let text = write_s2k(&m);
    // A handwritten row that states no ConstDamp: a modal history gets none,
    // every other type keeps the general 0.05 default.
    let without: String = text
        .lines()
        .map(|line| match line.find("ConstDamp=") {
            Some(start) => {
                let rest = &line[start..];
                let end = rest.find("   ").map_or(line.len(), |i| start + i + 3);
                format!("{}{}", &line[..start], &line[end..])
            }
            None => line.to_string(),
        })
        .collect::<Vec<_>>()
        .join("\n");
    assert!(!without.contains("ConstDamp"));
    let parsed = parse_s2k(&without).unwrap();
    assert_eq!(parsed.load_cases["H"].modal_damping, 0.0);
    assert_eq!(parsed.load_cases["S"].modal_damping, 0.05);
    // A stated ratio is kept, so analysis can refuse it instead of losing it.
    let stated = parse_s2k(&text.replacen("ConstDamp=0 ", "ConstDamp=0.02 ", 1)).unwrap();
    assert_eq!(
        stated.load_cases.values().filter(|c| c.modal_damping == 0.02).count(),
        1
    );
}

/// Object-level area assignments are written to the sections where the corpus
/// keeps them and read back from there: an N x N auto-mesh (`ShMesh`), added
/// mass (`ShMass`) and property modifiers (`ShModifier`).
#[test]
fn area_mesh_mass_and_modifiers_roundtrip_natively_and_through_s2k() {
    let mut m = supported_model();
    m.add_point("C", 12., 12., 0.);
    m.add_point("D", 0., 12., 0.);
    let mut area = AreaObj::new(
        "A1",
        vec!["A".into(), "B".into(), "C".into(), "D".into()],
        m.area_sections.keys().next().unwrap().clone(),
    );
    area.auto_mesh = (4, 4);
    area.mass_per_area = 7.35e-7;
    area.modifiers[6] = 1000.;
    area.modifiers[7] = 1000.;
    m.areas.insert("A1".into(), area);
    // A second, plain area gets no records.
    m.add_point("E", 24., 0., 0.);
    m.add_point("F", 24., 12., 0.);
    m.areas.insert(
        "A2".into(),
        AreaObj::new(
            "A2",
            vec!["B".into(), "E".into(), "F".into(), "C".into()],
            m.area_sections.keys().next().unwrap().clone(),
        ),
    );
    let native = pack_model_to_native_sdb(&m).unwrap();
    let classes = &inspect_native_sdb(&native).unwrap().inventory().classes.clone();
    assert_eq!(
        (classes["TYShMesh"], classes["TYSMass"], classes["TYShModifier"]),
        (1, 1, 1)
    );
    let r = extract_sdb_to_model(&native).unwrap();
    assert_eq!(r.areas["A1"], m.areas["A1"]);
    assert_eq!(r.areas["A2"], m.areas["A2"]);
    assert!(r.unmodeled_source.is_empty());
    let s = parse_s2k(&write_s2k(&m)).unwrap();
    assert_eq!(s.areas["A1"], m.areas["A1"]);

    // Unequal counts have no verified native edge order.
    m.areas["A1"].auto_mesh = (2, 4);
    assert_rejected_unchanged(&m, "areas.A1.auto_mesh");
    assert_eq!(parse_s2k(&write_s2k(&m)).unwrap().areas["A1"].auto_mesh, (2, 4));
}

/// Native area-section type codes follow the corpus, not the OAPI enumeration:
/// thin and thick shells and plane stress/strain have observed codes; plate,
/// membrane and layered sections do not and are refused.
#[test]
fn area_section_types_roundtrip_or_are_refused() {
    for (prop_type, shell_type, plane_type) in [
        (AreaPropType::Shell, ShellType::ShellThin, PlaneType::PlaneStress),
        (AreaPropType::Shell, ShellType::ShellThick, PlaneType::PlaneStress),
        (AreaPropType::Plane, ShellType::ShellThin, PlaneType::PlaneStress),
        (AreaPropType::Plane, ShellType::ShellThin, PlaneType::PlaneStrain),
        (AreaPropType::ASolid, ShellType::ShellThin, PlaneType::PlaneStress),
    ] {
        let mut m = supported_model();
        let name = m.area_sections.keys().next().unwrap().clone();
        m.area_sections[&name].prop_type = prop_type;
        m.area_sections[&name].shell_type = shell_type;
        m.area_sections[&name].plane_type = plane_type;
        let r = extract_sdb_to_model(&pack_model_to_native_sdb(&m).unwrap()).unwrap();
        assert_eq!(r.area_sections[&name], m.area_sections[&name]);
        let s = parse_s2k(&write_s2k(&m)).unwrap();
        assert_eq!(s.area_sections[&name], m.area_sections[&name]);
    }
    for shell_type in [
        ShellType::PlateThin,
        ShellType::PlateThick,
        ShellType::Membrane,
        ShellType::ShellLayered,
    ] {
        let mut m = supported_model();
        let name = m.area_sections.keys().next().unwrap().clone();
        m.area_sections[&name].shell_type = shell_type;
        assert_rejected_unchanged(&m, &format!("area_sections.{name}.shell_type"));
    }
}

/// Automatic frame subdivision is stored on the line object: the enabling
/// flag, subdivision at joints, and the segment count. It roundtrips natively
/// and through S2K, and frames without it are written as before.
#[test]
fn frame_auto_subdivision_roundtrips_natively_and_through_s2k() {
    let mut m = supported_model();
    let frame = m.frames.keys().next().unwrap().clone();
    let (i, j, section) = {
        let f = &m.frames[frame.as_str()];
        (f.point_i.clone(), f.point_j.clone(), f.section.clone())
    };
    m.add_frame("SEG", i.clone(), j.clone(), section.clone());
    m.frames["SEG"].auto_mesh_segments = 16;
    m.add_frame("JNT", i.clone(), j.clone(), section.clone());
    m.frames["JNT"].auto_mesh_at_joints = true;
    m.add_frame("BOTH", i, j, section);
    m.frames["BOTH"].auto_mesh_segments = 10;
    m.frames["BOTH"].auto_mesh_at_joints = true;

    let native = pack_model_to_native_sdb(&m).unwrap();
    let r = extract_sdb_to_model(&native).unwrap();
    assert!(r.unmodeled_source.is_empty(), "{:?}", r.unmodeled_source);
    let s = parse_s2k(&write_s2k(&m)).unwrap();
    for model in [&r, &s] {
        for (name, original) in &m.frames {
            let read = &model.frames[name];
            assert_eq!(
                (read.auto_mesh_segments, read.auto_mesh_at_joints),
                (original.auto_mesh_segments, original.auto_mesh_at_joints),
                "{name}"
            );
        }
        assert_eq!(
            (model.frames[frame.as_str()].auto_mesh_segments, model.frames[frame.as_str()].auto_mesh_at_joints),
            (0, false)
        );
    }
    // The S2K columns appear only on the frames that have the assignment.
    let text = write_s2k(&m);
    assert_eq!(text.matches("MeshSegments=").count(), 2);
    assert_eq!(text.matches("MeshAtJoints=Yes").count(), 2);

    // A count that does not fit the native integer is refused, not truncated.
    m.frames["SEG"].auto_mesh_segments = i32::MAX as usize + 1;
    assert_rejected_unchanged(&m, "frames.SEG.auto_mesh_segments");
}

/// Insertion-point joint offsets are written to `FOffset` in the form the
/// corpus shows (element local, equal at both ends, a 3 x 2 array) and read
/// back; the forms with no native evidence are refused, and S2K carries all.
#[test]
fn frame_joint_offsets_roundtrip_natively_in_the_verified_form_only() {
    let mut m = supported_model();
    m.frames["F"].cardinal_point = 1;
    m.frames["F"].stiff_transform = true;
    m.frames["F"].joint_offset_is_global = false;
    m.frames["F"].joint_offset_i = [0.0, 12.0, -1.5];
    m.frames["F"].joint_offset_j = [0.0, 12.0, -1.5];
    // Partial fixity shares the per-end array layout.
    m.frames["F"].release_j[5] = true;
    m.frames["F"].end_spring[5] = 250.0;

    let native = pack_model_to_native_sdb(&m).unwrap();
    let classes = &inspect_native_sdb(&native).unwrap().inventory().classes.clone();
    assert_eq!((classes["TYFOffset"], classes["TYFFixity"]), (1, 1));
    let r = extract_sdb_to_model(&native).unwrap();
    assert_eq!(r.frames["F"], m.frames["F"]);
    assert!(r.unmodeled_source.is_empty());
    assert_eq!(parse_s2k(&write_s2k(&m)).unwrap().frames["F"], m.frames["F"]);

    // Offsets that differ between the ends, or are given in the global
    // system, have no verified native form.
    let mut unequal = m.clone();
    unequal.frames["F"].joint_offset_j = [0.0, 6.0, -1.5];
    assert_rejected_unchanged(&unequal, "frames.F.joint_offset_j");
    assert_eq!(parse_s2k(&write_s2k(&unequal)).unwrap().frames["F"], unequal.frames["F"]);
    let mut global = m.clone();
    global.frames["F"].joint_offset_is_global = true;
    assert_rejected_unchanged(&global, "frames.F.joint_offset_is_global");
    assert_eq!(parse_s2k(&write_s2k(&global)).unwrap().frames["F"], global.frames["F"]);
}
