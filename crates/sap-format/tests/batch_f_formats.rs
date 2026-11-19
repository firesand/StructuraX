//! Batch F format repairs (D114 and later items): reader and writer behaviour
//! with hand-written expected values. Vendor-fixture tests skip when the corpus
//! is absent; none of its bytes or numbers are stored in the repository except
//! the expected values read from the planner's decode.

use sap_format::sdb::native::pack_model_to_native_sdb;
use sap_format::{extract_sdb_to_model, read_sdb_file};
use sap_model::*;
use std::path::PathBuf;

fn vendor(name: &str) -> Option<SapModel> {
    let p = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../extracted/files").join(name);
    if !p.exists() {
        return None;
    }
    Some(extract_sdb_to_model(&read_sdb_file(&p).expect("read vendor file")).expect("extract"))
}

#[test]
fn d114_vendor_rebar_materials_take_the_rebar_record_yield() {
    // aci_318_14_ex001 (kip, ft): A615Gr60 is 60 ksi = 60 * 144 = 8640 kip/ft2.
    if let Some(m) = vendor("aci_318_14_ex001.sdb") {
        let r = &m.materials["A615Gr60"];
        assert_eq!(r.mat_type, MatType::Rebar);
        assert!((r.fy - 8640.0).abs() < 1e-6, "fy {}", r.fy);
        assert!((r.fys - 8640.0).abs() < 1e-6, "fys {}", r.fys);
        let sec = &m.frame_sections["Conc Bm 10x16"];
        let cfg = sec.concrete_rebar.as_ref().expect("section rebar record");
        assert_eq!(cfg.rebar_mat_long, "A615Gr60");
        assert_eq!(cfg.rebar_mat_confine, "A615Gr60");
        assert!(!cfg.is_column);
        assert_eq!(cfg.total_rebar_area, 0.0);
    }
    // en_2_2004_ex001 (kN, m): 460 MPa = 460000 kN/m2 (before 344737.89).
    if let Some(m) = vendor("en_2_2004_ex001.sdb") {
        assert!((m.materials["A615Gr60-460"].fy - 460000.0).abs() < 1e-3);
    }
    // csa_a23.3_14_ex001: 400 MPa = 400000 kN/m2.
    if let Some(m) = vendor("csa_a23.3_14_ex001.sdb") {
        assert!((m.materials["A615Gr60-400"].fy - 400000.0).abs() < 1e-3);
    }
    // ss_cp_65_1999_ex001: shear rebar of "300x600 Con Bm" is another material.
    if let Some(m) = vendor("ss_cp_65_1999_ex001.sdb") {
        let cfg = m.frame_sections["300x600 Con Bm"]
            .concrete_rebar
            .as_ref()
            .expect("section rebar record");
        assert_eq!(cfg.rebar_mat_confine, "A615Gr40-250");
        assert_ne!(cfg.rebar_mat_long, cfg.rebar_mat_confine);
    }
}

#[test]
fn d114_native_round_trip_of_concrete_fys_and_rebar_fy() {
    // Concrete 4000Psi with fys 60 (independent of its fy 0 template) and a
    // Rebar material with fy = fys = 60 read back equal.
    let mut m = SapModel::default();
    m.add_point("A", 0.0, 0.0, 0.0);
    m.add_point("B", 10.0, 0.0, 0.0);
    m.add_frame("F1", "A", "B", "FSEC1");
    let mut rebar = m.materials["A992Fy50"].clone();
    rebar.name = "Rb".into();
    rebar.mat_type = MatType::Rebar;
    rebar.fy = 60.0;
    rebar.fys = 60.0;
    rebar.fu = 90.0;
    m.materials.insert("Rb".into(), rebar);
    let native = pack_model_to_native_sdb(&m).expect("export");
    let back = extract_sdb_to_model(&native).expect("import");
    assert_eq!(back.materials["Rb"].fy, 60.0);
    assert_eq!(back.materials["Rb"].fu, 90.0);
    assert_eq!(back, m);
    // Negative: a Rebar material with fys != fy is refused natively.
    m.materials.get_mut("Rb").unwrap().fys = 40.0;
    assert!(pack_model_to_native_sdb(&m).is_err());
}

#[test]
fn d114_section_rebar_record_round_trips_natively() {
    let mut m = SapModel::default();
    m.add_point("A", 0.0, 0.0, 0.0);
    m.add_point("B", 10.0, 0.0, 0.0);
    m.add_frame("F1", "A", "B", "FSEC1");
    let mut conc = m.materials["A992Fy50"].clone();
    conc.name = "Conc".into();
    conc.mat_type = MatType::Concrete;
    m.materials.insert("Conc".into(), conc);
    let mut rebar = m.materials["A992Fy50"].clone();
    rebar.name = "Rb".into();
    rebar.mat_type = MatType::Rebar;
    rebar.fy = 60.0;
    rebar.fys = 60.0;
    m.materials.insert("Rb".into(), rebar);
    let mut beam = m.frame_sections["FSEC1"].clone();
    beam.name = "BM".into();
    beam.material = "Conc".into();
    beam.concrete_rebar = Some(ConcreteRebarConfig {
        is_column: false,
        rebar_mat_long: "Rb".into(),
        rebar_mat_confine: "Rb".into(),
        cover_top: 2.5,
        cover_bot: 3.0,
        num_bars_3dir: 5,
        num_bars_2dir: 3,
        total_rebar_area: 0.0,
    });
    m.frame_sections.insert("BM".into(), beam.clone());
    let mut col = beam;
    col.name = "COL".into();
    col.concrete_rebar = Some(ConcreteRebarConfig {
        is_column: true,
        cover_top: 1.5,
        cover_bot: 1.5,
        ..col.concrete_rebar.clone().unwrap()
    });
    m.frame_sections.insert("COL".into(), col);
    let back = extract_sdb_to_model(&pack_model_to_native_sdb(&m).expect("export")).expect("import");
    assert_eq!(back, m);
    // A total rebar area (bar size table not modelled) has no native form: refused.
    m.frame_sections.get_mut("BM").unwrap().concrete_rebar.as_mut().unwrap().total_rebar_area = 4.0;
    assert!(pack_model_to_native_sdb(&m).is_err());
}

#[test]
fn d114_vendor_concrete_models_still_export_and_read_back_equal() {
    if let Some(m) = vendor("aci_318_14_ex001.sdb") {
        let back = extract_sdb_to_model(&pack_model_to_native_sdb(&m).expect("export")).unwrap();
        assert_eq!(
            back.frame_sections["Conc Bm 10x16"].concrete_rebar,
            m.frame_sections["Conc Bm 10x16"].concrete_rebar
        );
        assert_eq!(back.materials["A615Gr60"].fy, m.materials["A615Gr60"].fy);
    }
}

fn area_text(table: &str, row: &str) -> String {
    let mut m = SapModel::default();
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 10.0, 0.0, 0.0);
    m.add_point("3", 10.0, 10.0, 0.0);
    m.add_point("4", 0.0, 10.0, 0.0);
    m.add_area("A1", ["1", "2", "3", "4"].map(String::from).to_vec(), "ASEC1");
    let text = sap_format::write_s2k(&m);
    let table_text = format!("TABLE:  \"{table}\"\n   {row}\n\n");
    text.replace("END TABLE DATA", &format!("{table_text}END TABLE DATA"))
}

#[test]
fn d119_vendor_area_stiffness_modifiers_are_read() {
    let text = area_text(
        "AREA STIFFNESS MODIFIERS",
        "Area=A1   f11=0.5   f22=0.6   f12=0.7   m11=0.8   m22=0.9   m12=0.95   v13=0.3   v23=0.4   MassMod=1.1   WeightMod=1.2",
    );
    let m = sap_format::parse_s2k(&text).expect("parse");
    assert_eq!(
        m.areas["A1"].modifiers,
        [0.5, 0.6, 0.7, 0.8, 0.9, 0.95, 0.3, 0.4, 1.1, 1.2]
    );
    assert!(m.unmodeled_source.is_empty(), "{:?}", m.unmodeled_source);
    // Own round trip: the writer emits the vendor table and reads it back equal.
    let out = sap_format::write_s2k(&m);
    assert!(out.contains("TABLE:  \"AREA STIFFNESS MODIFIERS\""), "{out}");
    assert!(!out.contains("AREA PROPERTY MODIFIERS"));
    assert_eq!(sap_format::parse_s2k(&out).unwrap(), m);
}

#[test]
fn d119_area_modifier_column_that_is_not_modelled_is_recorded_not_applied() {
    let text = area_text("AREA STIFFNESS MODIFIERS", "Area=A1   F11Mod=0.5");
    let m = sap_format::parse_s2k(&text).expect("parse");
    assert_eq!(m.areas["A1"].modifiers, [1.0; 10]);
    assert!(
        m.unmodeled_source.iter().any(|u| u.scope == UnmodeledScope::AllCases
            && u.description.contains("AREA STIFFNESS MODIFIERS")
            && u.description.contains("F11MOD")),
        "{:?}",
        m.unmodeled_source
    );
    // The project's own earlier table name is still read.
    let old = area_text("AREA PROPERTY MODIFIERS", "Area=A1   F11Mod=0.5");
    let m = sap_format::parse_s2k(&old).unwrap();
    assert_eq!(m.areas["A1"].modifiers[0], 0.5);
}

#[test]
fn d119_vendor_p_delta_force_table_is_recorded_as_not_interpreted() {
    let text = area_text(
        "FRAME P-DELTA FORCE ASSIGNMENTS",
        "Frame=F1   CoordSys=GLOBAL   Direction=Z   Force=10",
    );
    let m = sap_format::parse_s2k(&text).expect("parse");
    assert!(m.unmodeled_source.iter().any(|u| u.scope == UnmodeledScope::AllCases
        && u.description.contains("vendor P-Delta force table not interpreted")));
    // The project's own spelling is still read and has no entry.
    let own = area_text("FRAME PDELTA FORCE ASSIGNMENTS", "Frame=F1   Force=10");
    let m = sap_format::parse_s2k(&own).unwrap();
    assert!(m.unmodeled_source.is_empty(), "{:?}", m.unmodeled_source);
}

#[test]
fn d119_joint_local_axes_are_written_under_the_vendor_name() {
    let mut m = SapModel::default();
    m.add_point("1", 0.0, 0.0, 0.0);
    m.points.get_mut("1").unwrap().local_axes_deg = [30.0, 0.0, 0.0];
    let out = sap_format::write_s2k(&m);
    assert!(out.contains("TABLE:  \"JOINT LOCAL AXES ASSIGNMENTS 1 - TYPICAL\""), "{out}");
    assert!(!out.contains("ASSIGNMENTS 1 - STANDARD"));
    assert_eq!(sap_format::parse_s2k(&out).unwrap().points["1"].local_axes_deg[0], 30.0);
}

fn rs_case_text(row: &str) -> String {
    let mut m = SapModel::default();
    m.load_cases.insert(
        "RSX".into(),
        LoadCase::linear_static("RSX", "DEAD", 1.0),
    );
    let text = sap_format::write_s2k(&m);
    let table = format!("TABLE:  \"CASE - RESPONSE SPECTRUM 1 - GENERAL\"\n   {row}\n\n");
    text.replace("END TABLE DATA", &format!("{table}END TABLE DATA"))
}

#[test]
fn d122_s2k_direction_combination_other_than_srss_is_recorded() {
    let abs = sap_format::parse_s2k(&rs_case_text("Case=RSX   ModalCombo=CQC   DirCombo=ABS"))
        .expect("parse");
    assert!(
        abs.unmodeled_source.iter().any(|u| u.scope == UnmodeledScope::Case("RSX".into())
            && u.description.contains("'RSX'")
            && u.description.contains("ABS")),
        "{:?}",
        abs.unmodeled_source
    );
    for none in ["Case=RSX   ModalCombo=CQC   DirCombo=SRSS", "Case=RSX   ModalCombo=CQC", "Case=RSX   DirCombo=srss"] {
        let m = sap_format::parse_s2k(&rs_case_text(none)).expect("parse");
        assert!(m.unmodeled_source.is_empty(), "{none}: {:?}", m.unmodeled_source);
        assert_eq!(m.load_cases["RSX"].case_type, LoadCaseType::ResponseSpectrum);
    }
}

fn field_bytes<'a>(
    doc: &sap_format::sdb::native::NativeSdbDocument,
    stream: &'a [u8],
    class_id: &str,
    name: &str,
    width: usize,
) -> &'a [u8] {
    let record = doc
        .records()
        .iter()
        .find(|r| r.class_id == class_id)
        .unwrap_or_else(|| panic!("no {class_id} record"));
    let field = record
        .fields
        .iter()
        .find(|f| f.name.as_deref() == Some(name))
        .unwrap_or_else(|| panic!("no {name} in {class_id}"));
    &stream[field.offset..field.offset + width]
}

#[test]
fn d123_native_writer_writes_the_vendor_constants() {
    use sap_format::sdb::native::inspect_native_sdb;
    // kip-in model with a cable and a link.
    let mut m = SapModel::default();
    m.add_point("A", 0.0, 0.0, 0.0);
    m.add_point("B", 100.0, 0.0, 0.0);
    m.add_frame("F", "A", "B", "CAB1");
    let mut cab = FrameSection::default();
    cab.name = "CAB1".into();
    cab.prop_type = FramePropType::Circle;
    cab.area = 0.5;
    cab.t3 = (4.0 * 0.5 / std::f64::consts::PI).sqrt();
    m.frame_sections.insert("CAB1".into(), cab);
    m.frames["F"].cable_params = Some(CableParams {
        undeformed_length: Some(99.0),
        target_tension_i: None,
        horizontal_tension: None,
        num_segments: 4,
    });
    let prop = LinkProp {
        name: "LP".into(),
        ..LinkProp::default()
    };
    m.link_props.insert("LP".into(), prop);
    m.links.insert(
        "L".into(),
        LinkObj {
            name: "L".into(),
            point_i: "B".into(),
            point_j: None,
            prop: "LP".into(),
            angle_deg: 0.0,
            selected: false,
        },
    );
    let native = pack_model_to_native_sdb(&m).expect("export");
    let doc = inspect_native_sdb(&native).unwrap();
    let s = &native.decompressed_stream;
    let i32_of = |b: &[u8]| i32::from_le_bytes(b.try_into().unwrap());
    let f64_of = |b: &[u8]| f64::from_le_bytes(b.try_into().unwrap());
    // 3 in 336/336 corpus files.
    assert_eq!(i32_of(field_bytes(&doc, s, "S_ProjectStartUp", "m_GlobalUp", 4)), 3);
    // g in in/s2: 9.80665 m/s2 / 0.0254 m/in = 386.08858267716536.
    assert_eq!(
        f64_of(field_bytes(&doc, s, "RUNARRAY", "m_Gravity", 8)),
        386.08858267716536
    );
    // A cable is line type 2 (1 is a curved frame); all 15 corpus cables have 2.
    assert_eq!(i32_of(field_bytes(&doc, s, "S_LINEOBJECTS", "m_LineType", 4)), 2);
    // 1.0 in all 82 corpus links.
    assert_eq!(f64_of(field_bytes(&doc, s, "S_NLLINKOBJECTS", "m_NPRMultiplier", 8)), 1.0);
    // Round trip stays equal.
    assert_eq!(extract_sdb_to_model(&native).unwrap(), m);
}

#[test]
fn d123_gravity_follows_the_length_unit_of_the_model() {
    use sap_format::sdb::native::inspect_native_sdb;
    let mut m = SapModel::new(sap_units::Units::KnMC);
    m.add_point("A", 0.0, 0.0, 0.0);
    let native = pack_model_to_native_sdb(&m).expect("export");
    let doc = inspect_native_sdb(&native).unwrap();
    let g = f64::from_le_bytes(
        field_bytes(&doc, &native.decompressed_stream, "RUNARRAY", "m_Gravity", 8)
            .try_into()
            .unwrap(),
    );
    assert_eq!(g, 9.80665);
}

#[test]
fn d123_a_cable_written_with_line_type_1_by_an_earlier_version_is_still_read() {
    let mut m = SapModel::default();
    m.add_point("A", 0.0, 0.0, 0.0);
    m.add_point("B", 100.0, 0.0, 0.0);
    m.add_frame("F", "A", "B", "CAB1");
    let mut cab = FrameSection::default();
    cab.name = "CAB1".into();
    cab.prop_type = FramePropType::Circle;
    cab.area = 0.5;
    cab.t3 = 0.8;
    m.frame_sections.insert("CAB1".into(), cab);
    m.frames["F"].cable_params = Some(CableParams {
        undeformed_length: Some(99.0),
        target_tension_i: None,
        horizontal_tension: None,
        num_segments: 4,
    });
    let mut native = pack_model_to_native_sdb(&m).expect("export");
    let doc = sap_format::sdb::native::inspect_native_sdb(&native).unwrap();
    let record = doc.records().iter().find(|r| r.class_id == "S_LINEOBJECTS").unwrap();
    let field = record.fields.iter().find(|f| f.name.as_deref() == Some("m_LineType")).unwrap();
    assert_eq!(native.decompressed_stream[field.offset], 2);
    native.decompressed_stream[field.offset] = 1;
    let back = extract_sdb_to_model(&native).expect("line type 1 is still read as a cable");
    assert_eq!(back.frames["F"].cable_params, m.frames["F"].cable_params);
}

// ------------------------------------------------------------------ D141

/// `rs_case_text` plus one row of the load assignment table of the same case.
fn rs_case_with_load(general: &str, load: &str) -> String {
    let table = format!(
        "TABLE:  \"CASE - RESPONSE SPECTRUM 2 - LOAD ASSIGNMENTS\"\n   Case=RSX   LoadType=Acceleration   LoadName=Accel   CoordDir=U1   Function=F1   TransAccSF=1   {load}\n\n"
    );
    rs_case_text(general).replace("END TABLE DATA", &format!("{table}END TABLE DATA"))
}

fn rsx_notes(text: &str) -> Vec<String> {
    sap_format::parse_s2k(text)
        .expect("parse")
        .unmodeled_source
        .iter()
        .filter(|u| u.scope == UnmodeledScope::Case("RSX".into()))
        .map(|u| u.description.clone())
        .collect()
}

/// The model holds the modal combinations CQC, SRSS and Absolute. The v19 program also
/// offers GMC, NRC 10 percent and Double Sum (API help, SetModalComb_1); a case that names
/// one of them was read as CQC without a word. It is now recorded for that case, which
/// refuses its analysis.
#[test]
fn d141_s2k_modal_combination_outside_the_model_is_recorded() {
    let notes = rsx_notes(&rs_case_text("Case=RSX   ModalCombo=GMC"));
    assert!(
        notes.iter().any(|n| n.contains("'RSX'") && n.contains("ModalCombo 'GMC'")),
        "{notes:?}"
    );
    for known in ["CQC", "SRSS", "ABS", "srss"] {
        let notes = rsx_notes(&rs_case_text(&format!("Case=RSX   ModalCombo={known}")));
        assert!(notes.is_empty(), "{known}: {notes:?}");
    }
    assert!(rsx_notes(&rs_case_text("Case=RSX   ConstDamp=0.05")).is_empty());
}

/// A spectrum load at an angle, or in a coordinate system other than GLOBAL, has no place
/// in the case: it is recorded instead of being applied along the global axis.
#[test]
fn d141_s2k_spectrum_load_with_an_angle_or_a_coordinate_system_is_recorded() {
    let general = "Case=RSX   ModalCombo=CQC";
    let notes = rsx_notes(&rs_case_with_load(general, "Angle=30"));
    assert!(notes.iter().any(|n| n.contains("Angle 30")), "{notes:?}");
    let notes = rsx_notes(&rs_case_with_load(general, "CoordSys=CSYS1"));
    assert!(notes.iter().any(|n| n.contains("CoordSys 'CSYS1'")), "{notes:?}");
    assert!(rsx_notes(&rs_case_with_load(general, "Angle=0   CoordSys=GLOBAL")).is_empty());
    assert!(rsx_notes(&rs_case_with_load(general, "")).is_empty());
}

/// Vendor fixture (skips without the corpus): Example 1-024 has four response-spectrum
/// cases; "RS-10PCT" uses the NRC 10 percent method (`m_modc` 5), which was read as CQC.
#[test]
fn d141_vendor_example_1_024_ten_percent_case_is_recorded() {
    let Some(model) = vendor("example_1_024.sdb") else {
        return;
    };
    let notes: Vec<String> = model
        .unmodeled_source_for_case("RS-10PCT")
        .into_iter()
        .map(|e| e.description.clone())
        .collect();
    assert!(
        notes.iter().any(|n| n.contains("NRC 10 percent")),
        "{notes:?}"
    );
    for case in ["RS-CQC", "RS-SRSS", "RS-ABS"] {
        assert!(
            model
                .unmodeled_source_for_case(case)
                .iter()
                .all(|e| !e.description.contains("combines modes")),
            "{case}"
        );
    }
}
