//! Batch F: database-table round trips (D115 and later items). Expected values
//! are hand calculations written out in each test.

use sap_api::{cSapModel, OAPI_OK};
use sap_units::Units;

/// Read a table for editing, stage it unchanged and apply it.
fn read_stage_apply(api: &mut cSapModel, key: &str) {
    let (ret, fields, n, data) = api.database_tables().get_table_for_editing_array_vec(key, "");
    assert_eq!(ret, OAPI_OK, "read {key}");
    let mut ver = 1;
    assert_eq!(
        api.database_tables()
            .set_table_for_editing_array(key, &mut ver, &fields, n, &data),
        OAPI_OK,
        "stage {key}"
    );
    assert_eq!(apply(api), OAPI_OK);
}

fn apply(api: &mut cSapModel) -> i32 {
    let (mut fatal, mut err, mut warn, mut info, mut log) = (0, 0, 0, 0, String::new());
    api.database_tables()
        .apply_edited_tables(true, &mut fatal, &mut err, &mut warn, &mut info, &mut log)
}

const MECH: &str = "MATERIAL PROPERTIES 02 - BASIC MECHANICAL PROPERTIES";

#[test]
fn d115_material_unit_mass_round_trips_bitwise_kip_in() {
    let mut api = cSapModel::new(Units::KipInF);
    // Default A992Fy50 in kip-in: unit weight 0.490 kip/ft3 = 0.490/1728 kip/in3;
    // mass density = weight / g with g = 386.08858267716536 in/s2.
    let expected = 0.490 / 1728.0 / 386.08858267716536;
    let before = api.inner.materials["A992Fy50"].unit_mass;
    assert!((before - expected).abs() < 1e-18, "{before} vs {expected}");
    read_stage_apply(&mut api, MECH);
    // Before the repair the cell read "0.000001" and the mass grew by 36 %.
    assert_eq!(api.inner.materials["A992Fy50"].unit_mass.to_bits(), before.to_bits());
}

#[test]
fn d115_n_mm_material_mass_is_not_zeroed() {
    let mut api = cSapModel::new(Units::NMmC);
    let before = api.inner.materials["A992Fy50"].unit_mass;
    // 76.9729 kN/m3 = 7.69729e-5 N/mm3; mass = weight / 9806.65 mm/s2 = 7.849e-9.
    assert!((before - 7.849047e-9).abs() < 1e-14, "{before}");
    read_stage_apply(&mut api, MECH);
    let after = api.inner.materials["A992Fy50"].unit_mass;
    assert_eq!(after.to_bits(), before.to_bits());
    assert!(after > 0.0);
}

#[test]
fn d115_small_section_constants_round_trip_in_metres() {
    let mut api = cSapModel::new(Units::KnMC);
    {
        let s = api.inner.frame_sections.get_mut("FSEC1").unwrap();
        s.torsional_j = 1.5e-6; // m4
        s.i22 = 4e-7; // m4
    }
    read_stage_apply(&mut api, "FRAME SECTION PROPERTIES 01 - GENERAL");
    let s = &api.inner.frame_sections["FSEC1"];
    assert_eq!(s.torsional_j.to_bits(), 1.5e-6f64.to_bits());
    assert_eq!(s.i22.to_bits(), 4e-7f64.to_bits());
}

#[test]
fn d115_integer_like_values_are_unchanged() {
    let mut api = cSapModel::new(Units::KipInF);
    let cell = |api: &mut cSapModel, table: &str, field: &str| -> String {
        let (_, fields, n, data) = api.database_tables().get_table_for_editing_array_vec(table, "");
        let m = fields.iter().position(|f| f == "Material").unwrap();
        let c = fields.iter().position(|f| f == field).unwrap();
        (0..n as usize)
            .map(|r| &data[r * fields.len()..(r + 1) * fields.len()])
            .find(|row| row[m] == "A992Fy50")
            .map(|row| row[c].clone())
            .unwrap()
    };
    assert_eq!(cell(&mut api, MECH, "E1"), "29000");
    assert_eq!(cell(&mut api, MECH, "Fy"), "50");
}

const SEC: &str = "FRAME SECTION PROPERTIES 01 - GENERAL";

#[test]
fn d116_new_section_with_all_columns_takes_shape_and_shear_areas() {
    let mut api = cSapModel::new(Units::KnMC);
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            SEC,
            &[
                "SectionName", "Material", "Shape", "t3", "t2", "tf", "tw", "Area", "TorsConst",
                "I33", "I22", "AS2", "AS3", "S33", "S22", "Z33", "Z22", "R33", "R22"
            ],
            1,
            &[
                "R1", "A992Fy50", "Rectangular", "0.4", "0.2", "0", "0", "0.08", "1.5e-3",
                "1.0667e-3", "2.6667e-4", "0.0667", "0.0667", "5.3e-3", "2.7e-3", "8e-3", "4e-3",
                "0.115", "0.058"
            ],
        ),
        OAPI_OK
    );
    assert_eq!(apply(&mut api), OAPI_OK);
    let s = &api.inner.frame_sections["R1"];
    assert_eq!(s.prop_type, sap_model::FramePropType::Rectangular);
    assert_eq!(s.as2, 0.0667);
    assert_eq!(s.as3, 0.0667);
    assert_eq!(s.t3, 0.4);
}

#[test]
fn d116_existing_section_keeps_omitted_columns() {
    let mut api = cSapModel::new(Units::KipInF);
    let before = api.inner.frame_sections["FSEC1"].clone();
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            SEC,
            &["SectionName", "Material", "Area"],
            1,
            &["FSEC1", "A992Fy50", "20.5"],
        ),
        OAPI_OK
    );
    assert_eq!(apply(&mut api), OAPI_OK);
    let s = &api.inner.frame_sections["FSEC1"];
    assert_eq!(s.area, 20.5);
    for (a, b) in [
        (s.i33, before.i33),
        (s.i22, before.i22),
        (s.torsional_j, before.torsional_j),
        (s.t3, before.t3),
        (s.t2, before.t2),
        (s.tf, before.tf),
        (s.tw, before.tw),
        (s.as2, before.as2),
        (s.as3, before.as3),
    ] {
        assert_eq!(a.to_bits(), b.to_bits());
    }
    assert_eq!(s.prop_type, before.prop_type);
}

#[test]
fn d116_new_section_with_missing_columns_or_unknown_shape_is_refused() {
    let mut api = cSapModel::new(Units::KipInF);
    let before = api.inner.clone();
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            SEC,
            &["SectionName", "Material"],
            1,
            &["R2", "A992Fy50"],
        ),
        OAPI_OK
    );
    let (mut fatal, mut err, mut warn, mut info, mut log) = (0, 0, 0, 0, String::new());
    assert_ne!(
        api.database_tables()
            .apply_edited_tables(true, &mut fatal, &mut err, &mut warn, &mut info, &mut log),
        OAPI_OK
    );
    assert!(log.contains("'R2'") && log.contains("Area") && log.contains("Shape"), "{log}");
    assert_eq!(api.inner, before);
    // An unknown Shape on an existing section is refused as well.
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            SEC,
            &["SectionName", "Shape"],
            1,
            &["FSEC1", "Hexagon"],
        ),
        OAPI_OK
    );
    assert_ne!(apply(&mut api), OAPI_OK);
    assert_eq!(api.inner, before);
}

#[test]
fn d117_new_material_is_converted_to_the_model_units() {
    let mut api = cSapModel::new(Units::KnMC);
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "MATERIAL PROPERTIES 01 - GENERAL",
            &["Material", "Type"],
            1,
            &["M1", "Steel"],
        ),
        OAPI_OK
    );
    assert_eq!(apply(&mut api), OAPI_OK);
    let m = &api.inner.materials["M1"];
    // 50 ksi = 50 * 6894.757 = 344737.86 kN/m2; 29000 ksi = 199947961.5 kN/m2;
    // 0.490 kip/ft3 = 0.490 * 157.08746 = 76.97286 kN/m3.
    assert!((m.fy - 344737.86).abs() < 0.05, "fy {}", m.fy);
    assert!((m.e - 199947961.5).abs() < 5.0, "e {}", m.e);
    assert!((m.unit_weight - 76.97286).abs() < 1e-4, "w {}", m.unit_weight);
}

#[test]
fn d117_unknown_material_type_is_refused() {
    let mut api = cSapModel::new(Units::KipInF);
    let before = api.inner.clone();
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "MATERIAL PROPERTIES 01 - GENERAL",
            &["Material", "Type"],
            1,
            &["M1", "Unobtainium"],
        ),
        OAPI_OK
    );
    assert_ne!(apply(&mut api), OAPI_OK);
    assert_eq!(api.inner, before);
}

#[test]
fn d117_all_fourteen_load_case_types_survive_a_round_trip() {
    use sap_model::{LoadCase, LoadCaseType as T};
    let types = [
        T::LinearStatic, T::NonlinearStatic, T::Modal, T::ResponseSpectrum, T::LinearHistory,
        T::NonlinearHistory, T::LinearDynamic, T::NonlinearDynamic, T::MovingLoad, T::Buckling,
        T::SteadyState, T::PowerSpectralDensity, T::LinearStaticMultiStep, T::HyperStatic,
    ];
    let mut api = cSapModel::new(Units::KipInF);
    api.inner.load_cases.clear();
    for (i, t) in types.iter().enumerate() {
        let mut lc = LoadCase::linear_static(format!("C{i}"), "DEAD", 1.0);
        lc.case_type = *t;
        api.inner.load_cases.insert(format!("C{i}"), lc);
    }
    read_stage_apply(&mut api, "LOAD CASE DEFINITIONS");
    for (i, t) in types.iter().enumerate() {
        assert_eq!(api.inner.load_cases[&format!("C{i}")].case_type, *t, "case C{i}");
    }
}

#[test]
fn d117_load_case_type_omitted_or_unknown() {
    let mut api = cSapModel::new(Units::KipInF);
    let n = api.inner.load_cases.len();
    // A new case without Type is refused; an unknown name is refused.
    for (fields, row) in [
        (vec!["Case"], vec!["NEW"]),
        (vec!["Case", "Type"], vec!["NEW", "Bogus"]),
    ] {
        assert_eq!(
            api.database_tables()
                .set_table_for_editing_array_str("LOAD CASE DEFINITIONS", &fields, 1, &row),
            OAPI_OK
        );
        assert_ne!(apply(&mut api), OAPI_OK);
        assert_eq!(api.inner.load_cases.len(), n);
    }
    // An omitted Type keeps an existing case's type.
    api.inner.load_cases.get_mut("DEAD").unwrap().case_type = sap_model::LoadCaseType::Modal;
    assert_eq!(
        api.database_tables()
            .set_table_for_editing_array_str("LOAD CASE DEFINITIONS", &["Case"], 1, &["DEAD"]),
        OAPI_OK
    );
    assert_eq!(apply(&mut api), OAPI_OK);
    assert_eq!(api.inner.load_cases["DEAD"].case_type, sap_model::LoadCaseType::Modal);
}

#[test]
fn d118_load_pattern_keeps_multiplier_and_new_pattern_takes_its_type() {
    use sap_model::LoadPatternType;
    let mut api = cSapModel::new(Units::KipInF);
    api.inner.load_patterns.get_mut("DEAD").unwrap().self_wt_multiplier = 1.0;
    // SelfWtMult omitted: the existing multiplier stays (before: 0).
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "LOAD PATTERN DEFINITIONS",
            &["LoadPat", "DesignType"],
            2,
            &["DEAD", "Dead", "WIND1", "Wind"],
        ),
        OAPI_OK
    );
    assert_eq!(apply(&mut api), OAPI_OK);
    assert_eq!(api.inner.load_patterns["DEAD"].self_wt_multiplier, 1.0);
    // The new pattern takes its type from DesignType (before: Dead), multiplier 0.
    assert_eq!(api.inner.load_patterns["WIND1"].pat_type, LoadPatternType::Wind);
    assert_eq!(api.inner.load_patterns["WIND1"].self_wt_multiplier, 0.0);
    // Negative: an unknown DesignType refuses the edit.
    let before = api.inner.clone();
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "LOAD PATTERN DEFINITIONS",
            &["LoadPat", "DesignType"],
            1,
            &["X", "Gale"],
        ),
        OAPI_OK
    );
    assert_ne!(apply(&mut api), OAPI_OK);
    assert_eq!(api.inner, before);
}

#[test]
fn d118_combinations_that_refer_to_each_other_are_refused() {
    let mut api = cSapModel::new(Units::KipInF);
    let before = api.inner.clone();
    // A = B, B = A.
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "COMBINATION DEFINITIONS",
            &["ComboName", "CaseName", "ScaleFactor"],
            2,
            &["A", "B", "1.0", "B", "A", "1.0"],
        ),
        OAPI_OK
    );
    assert_ne!(apply(&mut api), OAPI_OK);
    assert_eq!(api.inner, before);
    // A self-reference is a cycle too.
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "COMBINATION DEFINITIONS",
            &["ComboName", "CaseName", "ScaleFactor"],
            1,
            &["S", "S", "1.0"],
        ),
        OAPI_OK
    );
    assert_ne!(apply(&mut api), OAPI_OK);
    assert_eq!(api.inner, before);
    // Negative: a chain without a cycle is accepted (B = DEAD, A = B).
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "COMBINATION DEFINITIONS",
            &["ComboName", "CaseName", "ScaleFactor"],
            2,
            &["A", "B", "1.0", "B", "DEAD", "1.5"],
        ),
        OAPI_OK
    );
    assert_eq!(apply(&mut api), OAPI_OK);
    assert_eq!(api.inner.load_combos["A"].cases, vec![("B".to_string(), 1.0)]);
}

#[test]
fn d118_open_file_drops_tables_staged_for_the_previous_model() {
    let mut api = cSapModel::new(Units::KipInF);
    let dir = std::env::temp_dir().join(format!("structurax-d118-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("other.s2k");
    let other = sap_format::write_s2k(&sap_model::SapModel::new(Units::KipInF));
    std::fs::write(&path, other).unwrap();
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "JOINT COORDINATES",
            &["Joint", "XorR", "Y", "Z"],
            1,
            &["STAGED", "1", "2", "3"],
        ),
        OAPI_OK
    );
    assert_eq!(api.file().open_file(path.to_str().unwrap()), OAPI_OK);
    assert!(api.staged_tables.is_empty());
    let _ = apply(&mut api);
    assert!(!api.inner.points.contains_key("STAGED"));
    let _ = std::fs::remove_dir_all(&dir);
}
