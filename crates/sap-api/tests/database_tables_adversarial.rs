//! Adversarial stress test suite for `cDatabaseTables` facade in `sap-api`.
//! Authored by Challenger M4-2 (`teamwork_challenger_m4_2`).

use sap_api::{cSapModel, OAPI_ERR, OAPI_OK};
use sap_units::Units;

fn setup_clean_model() -> cSapModel {
    cSapModel::new(Units::KipInF)
}

fn setup_base_model() -> cSapModel {
    let mut api = cSapModel::new(Units::KipInF);
    api.point_obj().add_cartesian(0.0, 0.0, 0.0, "1");
    api.point_obj().add_cartesian(144.0, 0.0, 0.0, "2");
    api.frame_obj().add_by_point("1", "2", "FSEC1", "F1");
    api
}

// =========================================================================
// 1. Malformed 1D Array Data & Record Dimensions
// =========================================================================

#[test]
fn test_adversarial_malformed_1d_array_dimensions_and_records() {
    let mut api = setup_base_model();

    // 1.1 Array length not an exact multiple of field keys
    // 3 fields ("Joint", "XorR", "Y"), 2 records -> requires 6 elements. Provide 5 elements.
    let fields = vec!["Joint".to_string(), "XorR".to_string(), "Y".to_string()];
    let data_5 = vec![
        "1".into(),
        "0.0".into(),
        "0.0".into(),
        "2".into(),
        "100.0".into(),
    ];
    let mut ver = 1;
    assert_eq!(
        api.database_tables().set_table_for_editing_array(
            "Joint Coordinates",
            &mut ver,
            &fields,
            2,
            &data_5,
        ),
        OAPI_ERR,
        "Array length smaller than num_records * fields must be rejected"
    );

    // Provide 7 elements (too many)
    let data_7 = vec![
        "1".into(),
        "0.0".into(),
        "0.0".into(),
        "2".into(),
        "100.0".into(),
        "50.0".into(),
        "EXTRA".into(),
    ];
    assert_eq!(
        api.database_tables().set_table_for_editing_array(
            "Joint Coordinates",
            &mut ver,
            &fields,
            2,
            &data_7,
        ),
        OAPI_ERR,
        "Array length larger than num_records * fields must be rejected"
    );

    // 1.2 Negative number_records
    assert_eq!(
        api.database_tables().set_table_for_editing_array(
            "Joint Coordinates",
            &mut ver,
            &fields,
            -1,
            &[],
        ),
        OAPI_ERR,
        "Negative record count must be rejected"
    );

    // 1.3 Empty field list with positive record count
    assert_eq!(
        api.database_tables().set_table_for_editing_array(
            "Joint Coordinates",
            &mut ver,
            &[],
            3,
            &[],
        ),
        OAPI_ERR,
        "Non-zero records with zero fields must be rejected"
    );

    // 1.4 Missing primary key in fields
    let no_key_fields = vec!["XorR".to_string(), "Y".to_string(), "Z".to_string()];
    let no_key_data = vec!["0.0".into(), "0.0".into(), "0.0".into()];
    assert_eq!(
        api.database_tables().set_table_for_editing_array(
            "Joint Coordinates",
            &mut ver,
            &no_key_fields,
            1,
            &no_key_data,
        ),
        OAPI_ERR,
        "Fields missing primary key 'Joint' must be rejected"
    );

    // Primary key missing for FRAME SECTION PROPERTIES 01 - GENERAL ("SectionName")
    let no_sec_fields = vec!["Material".to_string(), "Area".to_string()];
    let no_sec_data = vec!["A992Fy50".into(), "10.0".into()];
    assert_eq!(
        api.database_tables().set_table_for_editing_array(
            "Frame Section Properties 01 - General",
            &mut ver,
            &no_sec_fields,
            1,
            &no_sec_data,
        ),
        OAPI_ERR,
        "Fields missing primary key 'SectionName' must be rejected"
    );

    // 1.5 Zero records with matching empty data (legal staging of empty table)
    assert_eq!(
        api.database_tables().set_table_for_editing_array(
            "Joint Coordinates",
            &mut ver,
            &fields,
            0,
            &[],
        ),
        OAPI_OK,
        "Zero records with zero data elements and valid primary key must succeed"
    );
    assert_eq!(
        api.database_tables().apply_edited_tables_simple(false),
        OAPI_OK
    );
}

// =========================================================================
// 2. Malformed Field Values (Coordinates, Properties, Names)
// =========================================================================

#[test]
fn test_adversarial_malformed_field_values_rejection() {
    let mut api = setup_base_model();
    let before = api.inner.clone();

    // 2.1 Non-numeric strings in coordinates: "NaN", "Infinity", "-inf", "NOT_A_NUMBER", empty
    let adversarial_coords = [
        ("J_NAN", "NaN", "0.0", "0.0"),
        ("J_INF", "+Infinity", "0.0", "0.0"),
        ("J_NEGINF", "-inf", "0.0", "0.0"),
        ("J_STR", "abc_xyz", "0.0", "0.0"),
        ("J_EMPTY", "", "0.0", "0.0"),
    ];

    for (name, x, y, z) in adversarial_coords {
        assert_eq!(
            api.database_tables().set_table_for_editing_array_str(
                "Joint Coordinates",
                &["Joint", "XorR", "Y", "Z"],
                1,
                &[name, x, y, z],
            ),
            OAPI_OK,
            "Staging accepts string data"
        );
        let ret = api.database_tables().apply_edited_tables_simple(true);
        assert_eq!(
            ret, OAPI_ERR,
            "Applying non-numeric or non-finite coordinate '{}' for joint '{}' must fail",
            x, name
        );
        assert_eq!(
            api.inner, before,
            "Model must remain pristine after failure"
        );
    }

    // 2.2 Invalid entity names: empty string, whitespace only, quotes, control chars
    let bad_names = ["", "   ", "\"BAD_QUOTE\"", "BAD\nNAME", "BAD\tNAME"];
    for bad_name in bad_names {
        assert_eq!(
            api.database_tables().set_table_for_editing_array_str(
                "Joint Coordinates",
                &["Joint", "XorR", "Y", "Z"],
                1,
                &[bad_name, "10.0", "20.0", "30.0"],
            ),
            OAPI_OK
        );
        assert_eq!(
            api.database_tables().apply_edited_tables_simple(true),
            OAPI_ERR,
            "Invalid joint name '{}' must fail validation on apply",
            bad_name
        );
        assert_eq!(api.inner, before);
    }

    // 2.3 Coincident frame joints (JointI == JointJ)
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Connectivity - Frame",
            &["Frame", "JointI", "JointJ"],
            1,
            &["F_ZERO_LEN", "1", "1"],
        ),
        OAPI_OK
    );
    assert_eq!(
        api.database_tables().apply_edited_tables_simple(true),
        OAPI_ERR,
        "Coincident frame endpoints must be rejected"
    );
    assert_eq!(api.inner, before);

    // 2.4 Unphysical section properties: non-numeric Area, zero Area, negative Area
    let bad_section_areas = ["NOT_A_FLOAT", "0.0", "-15.0", "NaN"];
    for bad_area in bad_section_areas {
        assert_eq!(
            api.database_tables().set_table_for_editing_array_str(
                "Frame Section Properties 01 - General",
                &["SectionName", "Material", "Area", "I33", "I22"],
                1,
                &["SEC_BAD", "A992Fy50", bad_area, "100.0", "50.0"],
            ),
            OAPI_OK
        );
        assert_eq!(
            api.database_tables().apply_edited_tables_simple(true),
            OAPI_ERR,
            "Unphysical or non-numeric section area '{}' must fail",
            bad_area
        );
        assert_eq!(api.inner, before);
    }

    // Negative / zero moment of inertia I33
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Frame Section Properties 01 - General",
            &["SectionName", "Material", "Area", "I33", "I22"],
            1,
            &["SEC_BAD_I", "A992Fy50", "10.0", "-50.0", "50.0"],
        ),
        OAPI_OK
    );
    assert_eq!(
        api.database_tables().apply_edited_tables_simple(true),
        OAPI_ERR,
        "Negative I33 must fail"
    );
    assert_eq!(api.inner, before);

    // 2.5 Unphysical material properties: zero/negative E1, Poisson's ratio >= 0.5 or < 0
    let bad_mechanics = [
        ("E1", "0.0"),
        ("E1", "-29000.0"),
        ("E1", "NaN"),
        ("U12", "0.5"), // nu >= 0.5 unphysical for elastic isotropic
        ("U12", "0.75"),
        ("U12", "-0.1"),
    ];

    for (prop, val) in bad_mechanics {
        assert_eq!(
            api.database_tables().set_table_for_editing_array_str(
                "Material Properties 02 - Basic Mechanical Properties",
                &["Material", prop],
                1,
                &["A992Fy50", val],
            ),
            OAPI_OK
        );
        assert_eq!(
            api.database_tables().apply_edited_tables_simple(true),
            OAPI_ERR,
            "Unphysical material property {}='{}' must fail",
            prop,
            val
        );
        assert_eq!(api.inner, before);
    }
}

// =========================================================================
// 3. Referential Integrity & Inverted Topological Order Execution
// =========================================================================

#[test]
fn test_adversarial_inverted_topological_order_all_tables() {
    let mut api = setup_clean_model();
    assert!(api.inner.points.is_empty());
    assert!(api.inner.frames.is_empty());

    // Stage ALL 10 definition tables in completely REVERSED dependency order:
    // 10. Combos (references cases)
    // 9. Load Cases (references patterns)
    // 8. Load Patterns
    // 7. Frame Section Assignments (references frames and sections)
    // 6. Frame Connectivity (references joints and sections)
    // 5. Joint Restraints (references joints)
    // 4. Joint Coordinates
    // 3. Frame Section Properties (references materials)
    // 2. Material Mechanical (references materials)
    // 1. Material General

    // 10. Combination Definitions
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Combination Definitions",
            &["ComboName", "ComboType", "CaseName", "ScaleFactor"],
            1,
            &["COMB_TEST", "LinearAdd", "LC_GRAVITY", "1.4"],
        ),
        OAPI_OK
    );

    // 9. Load Case Definitions
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Load Case Definitions",
            &["Case", "Type"],
            1,
            &["LC_GRAVITY", "LinearStatic"],
        ),
        OAPI_OK
    );

    // 8. Load Pattern Definitions
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Load Pattern Definitions",
            &["LoadPat", "DesignType", "SelfWtMult"],
            1,
            &["DEAD_PAT", "DEAD", "1.0"],
        ),
        OAPI_OK
    );

    // 7. Frame Section Assignments
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Frame Section Assignments",
            &["Frame", "AnalSect"],
            1,
            &["F_TOP", "SEC_W14"],
        ),
        OAPI_OK
    );

    // 6. Frame Connectivity
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Connectivity - Frame",
            &["Frame", "JointI", "JointJ"],
            1,
            &["F_TOP", "JT_A", "JT_B"],
        ),
        OAPI_OK
    );

    // 5. Joint Restraints
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Joint Restraint Assignments",
            &["Joint", "U1", "U2", "U3", "R1", "R2", "R3"],
            1,
            &["JT_A", "Yes", "Yes", "Yes", "Yes", "Yes", "Yes"],
        ),
        OAPI_OK
    );

    // 4. Joint Coordinates
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Joint Coordinates",
            &["Joint", "XorR", "Y", "Z"],
            2,
            &["JT_A", "0.0", "0.0", "0.0", "JT_B", "180.0", "0.0", "0.0",],
        ),
        OAPI_OK
    );

    // 3. Frame Section Properties
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Frame Section Properties 01 - General",
            &["SectionName", "Material", "Area", "I33", "I22"],
            1,
            &["SEC_W14", "MAT_ADV", "20.0", "400.0", "150.0"],
        ),
        OAPI_OK
    );

    // 2. Material Mechanical
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Material Properties 02 - Basic Mechanical Properties",
            &["Material", "E1", "U12", "Fy"],
            1,
            &["MAT_ADV", "29500.0", "0.28", "50.0"],
        ),
        OAPI_OK
    );

    // 1. Material General
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Material Properties 01 - General",
            &["Material", "Type"],
            1,
            &["MAT_ADV", "Steel"],
        ),
        OAPI_OK
    );

    assert_eq!(api.staged_tables.len(), 10, "All 10 tables staged");

    // Commit all staged tables together:
    // Internal topological sort MUST reorder and execute in exact order:
    // Material General -> Material Mech -> Section Props -> Joints -> Restraints -> Frame -> Assignments -> Patterns -> Cases -> Combos
    let mut fatal = 0;
    let mut err = 0;
    let mut warn = 0;
    let mut info = 0;
    let mut log = String::new();

    let res = api
        .database_tables()
        .apply_edited_tables(true, &mut fatal, &mut err, &mut warn, &mut info, &mut log);

    assert_eq!(
        res, OAPI_OK,
        "Apply must succeed despite inverted staging order. Log:\n{}",
        log
    );
    assert_eq!(fatal, 0);

    // Verify model state after commit
    // 1. Material
    let mat = api
        .inner
        .materials
        .get("MAT_ADV")
        .expect("Material MAT_ADV must exist");
    assert_eq!(mat.e, 29500.0);
    assert_eq!(mat.u, 0.28);
    assert_eq!(mat.fy, 50.0);

    // 2. Section
    let sec = api
        .inner
        .frame_sections
        .get("SEC_W14")
        .expect("Section SEC_W14 must exist");
    assert_eq!(sec.material, "MAT_ADV");
    assert_eq!(sec.area, 20.0);
    assert_eq!(sec.i33, 400.0);

    // 3. Joints
    let ja = api.inner.points.get("JT_A").expect("JT_A must exist");
    let jb = api.inner.points.get("JT_B").expect("JT_B must exist");
    assert_eq!((ja.x, ja.y, ja.z), (0.0, 0.0, 0.0));
    assert_eq!((jb.x, jb.y, jb.z), (180.0, 0.0, 0.0));
    assert_eq!(ja.restraint, [true; 6]);

    // 4. Frame & Section Assignment
    let frame = api.inner.frames.get("F_TOP").expect("F_TOP must exist");
    assert_eq!(frame.point_i, "JT_A");
    assert_eq!(frame.point_j, "JT_B");
    assert_eq!(frame.section, "SEC_W14");

    // 5. Pattern, Case, Combo
    assert!(api.inner.load_patterns.contains_key("DEAD_PAT"));
    assert!(api.inner.load_cases.contains_key("LC_GRAVITY"));
    let combo = api
        .inner
        .load_combos
        .get("COMB_TEST")
        .expect("Combo COMB_TEST must exist");
    assert_eq!(combo.cases.len(), 1);
    assert_eq!(combo.cases[0], ("LC_GRAVITY".to_string(), 1.4));

    // Staging buffer must be empty after successful commit
    assert!(api.staged_tables.is_empty());
}

// =========================================================================
// 4. Transactional Atomicity Under Injected Catastrophic Errors
// =========================================================================

#[test]
fn test_adversarial_transactional_atomicity_multi_table_failures() {
    let mut api = setup_base_model();
    let initial_snapshot = api.inner.clone();

    // 4.1 Injected error in Table 1: Section references non-existent material "GHOST_MAT"
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Frame Section Properties 01 - General",
            &["SectionName", "Material", "Area", "I33", "I22"],
            1,
            &["SEC_FAIL", "GHOST_MAT", "10.0", "100.0", "50.0"],
        ),
        OAPI_OK
    );
    // Also stage valid joints in the same batch
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Joint Coordinates",
            &["Joint", "XorR", "Y", "Z"],
            1,
            &["J_NEW", "50.0", "50.0", "50.0"],
        ),
        OAPI_OK
    );

    let mut fatal = 0;
    let mut err = 0;
    let mut warn = 0;
    let mut info = 0;
    let mut log = String::new();
    assert_eq!(
        api.database_tables()
            .apply_edited_tables(true, &mut fatal, &mut err, &mut warn, &mut info, &mut log,),
        OAPI_ERR
    );
    assert!(fatal > 0);
    assert!(log.contains("GHOST_MAT"));

    // Check complete rollback: J_NEW must NOT have been committed, SEC_FAIL must not exist!
    assert_eq!(
        api.inner, initial_snapshot,
        "Model must revert 100% on missing material"
    );
    assert!(
        api.staged_tables.is_empty(),
        "Staged tables must be cleared"
    );

    // 4.2 Injected error in Table 2: Frame references non-existent joint "MISSING_JT"
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Joint Coordinates",
            &["Joint", "XorR", "Y", "Z"],
            1,
            &["J_VALID", "10.0", "0.0", "0.0"],
        ),
        OAPI_OK
    );
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Connectivity - Frame",
            &["Frame", "JointI", "JointJ"],
            1,
            &["F_BAD", "J_VALID", "MISSING_JT"],
        ),
        OAPI_OK
    );
    assert_eq!(
        api.database_tables().apply_edited_tables_simple(true),
        OAPI_ERR
    );
    assert_eq!(
        api.inner, initial_snapshot,
        "Model must revert 100% on missing joint"
    );
    assert!(api.staged_tables.is_empty());

    // 4.3 Injected error in Table 3: Frame section assignment references non-existent section
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Frame Section Assignments",
            &["Frame", "AnalSect"],
            1,
            &["F1", "NON_EXISTENT_SECTION"],
        ),
        OAPI_OK
    );
    assert_eq!(
        api.database_tables().apply_edited_tables_simple(true),
        OAPI_ERR
    );
    assert_eq!(
        api.inner, initial_snapshot,
        "Model must revert 100% on missing section assignment"
    );
    assert!(api.staged_tables.is_empty());

    // 4.4 Injected error in Table 4: Joint restraint assignment references non-existent joint
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Joint Restraint Assignments",
            &["Joint", "U1", "U2", "U3"],
            1,
            &["NO_SUCH_JOINT", "Yes", "Yes", "Yes"],
        ),
        OAPI_OK
    );
    assert_eq!(
        api.database_tables().apply_edited_tables_simple(true),
        OAPI_ERR
    );
    assert_eq!(
        api.inner, initial_snapshot,
        "Model must revert 100% on missing joint restraint"
    );
    assert!(api.staged_tables.is_empty());
}

// =========================================================================
// 5. Result Tables Read-Only Protection & Variations
// =========================================================================

#[test]
fn test_adversarial_result_tables_read_only_protection() {
    let mut api = setup_base_model();

    let result_table_keys = [
        "JOINT DISPLACEMENTS",
        "JOINT REACTIONS",
        "BASE REACTIONS",
        "ELEMENT FORCES - FRAMES",
        "MODAL PERIODS AND FREQUENCIES",
        "BUCKLING FACTORS",
        "STEEL DESIGN 1 - SUMMARY DATA",
        "CONCRETE DESIGN 1 - COLUMN SUMMARY DATA",
        "CONCRETE DESIGN 2 - BEAM SUMMARY DATA",
    ];

    let mut ver = 0;
    let mut included = Vec::new();
    let mut num = 0;
    let mut data = Vec::new();

    for key in result_table_keys {
        // Test exact key
        assert_eq!(
            api.database_tables().get_table_for_editing_array(
                key,
                "",
                &mut ver,
                &mut included,
                &mut num,
                &mut data,
            ),
            OAPI_ERR,
            "Result table '{}' must be rejected by GetTableForEditingArray",
            key
        );

        // Test lowercase variation
        let lower = key.to_lowercase();
        assert_eq!(
            api.database_tables().get_table_for_editing_array(
                &lower,
                "",
                &mut ver,
                &mut included,
                &mut num,
                &mut data,
            ),
            OAPI_ERR,
            "Lowercase result table '{}' must be rejected by GetTableForEditingArray",
            lower
        );

        // Test "TABLE: \"...\"" prefix variation
        let prefixed = format!("TABLE: \"{}\"", key);
        assert_eq!(
            api.database_tables().get_table_for_editing_array(
                &prefixed,
                "",
                &mut ver,
                &mut included,
                &mut num,
                &mut data,
            ),
            OAPI_ERR,
            "Prefixed result table '{}' must be rejected by GetTableForEditingArray",
            prefixed
        );

        // Test SetTableForEditingArray rejects result table
        assert_eq!(
            api.database_tables().set_table_for_editing_array_str(
                key,
                &["Joint", "U1"],
                1,
                &["1", "0.5"],
            ),
            OAPI_ERR,
            "Result table '{}' must be rejected by SetTableForEditingArray",
            key
        );
        assert!(api.staged_tables.is_empty());
    }
}

// =========================================================================
// 6. Locked Model Immunity & Staging Rejection
// =========================================================================

#[test]
fn test_adversarial_locked_model_protection() {
    let mut api = setup_base_model();

    // 6.1 SetTableForEditingArray while model is locked
    api.set_model_is_locked(true);
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Joint Coordinates",
            &["Joint", "XorR", "Y", "Z"],
            1,
            &["1", "100.0", "100.0", "100.0"],
        ),
        OAPI_ERR,
        "Staging on locked model must be rejected"
    );
    assert!(api.staged_tables.is_empty());

    // 6.2 Pre-stage while unlocked, lock model, then call ApplyEditedTables
    api.set_model_is_locked(false);
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Joint Coordinates",
            &["Joint", "XorR", "Y", "Z"],
            1,
            &["1", "100.0", "100.0", "100.0"],
        ),
        OAPI_OK
    );
    assert_eq!(api.staged_tables.len(), 1);

    // Lock model before applying
    api.set_model_is_locked(true);
    let mut fatal = 0;
    let mut err = 0;
    let mut warn = 0;
    let mut info = 0;
    let mut log = String::new();

    let res = api
        .database_tables()
        .apply_edited_tables(true, &mut fatal, &mut err, &mut warn, &mut info, &mut log);

    assert_eq!(res, OAPI_ERR, "Apply on locked model must return OAPI_ERR");
    assert!(fatal > 0, "Must increment fatal error count");
    assert!(
        log.contains("Model is locked"),
        "Log must mention locked model"
    );

    // Coordinates of Joint 1 must remain (0, 0, 0)
    let p1 = api.inner.points.get("1").unwrap();
    assert_eq!((p1.x, p1.y, p1.z), (0.0, 0.0, 0.0));

    // Unlock and apply should now succeed
    api.set_model_is_locked(false);
    assert_eq!(
        api.database_tables().apply_edited_tables_simple(true),
        OAPI_OK
    );
    let p1_after = api.inner.points.get("1").unwrap();
    assert_eq!((p1_after.x, p1_after.y, p1_after.z), (100.0, 100.0, 100.0));
}

// =========================================================================
// 7. Numerical Verification: Table-Imported Cantilever Benchmark
// =========================================================================

#[test]
fn test_adversarial_numerical_verification_from_scratch_cantilever() {
    let mut api = setup_clean_model();

    // Beam parameters:
    // L = 100 in, E = 30000 ksi, b = 2 in, h = 4 in
    // A = 8.0 in^2, I33 = 10.6666667 in^4, I22 = 2.6666667 in^4, AS2 = 0.0 (Euler-Bernoulli)
    // Transverse tip load Pz = -5.0 kips, Axial tip load Px = 40.0 kips
    // Theoretical tip deflection:
    // Delta_z = Pz * L^3 / (3 * E * I33) = 5.0 * 100^3 / (3 * 30000 * (128/12)) = 5.208333 in (downward)
    // Delta_x = Px * L / (E * A) = 40.0 * 100 / (30000 * 8.0) = 0.0166667 in (extension)
    // Reactions: Fx = -40.0 kips, Fz = +5.0 kips, My = 5.0 * 100 = +500.0 kip-in

    // 1. Stage Material
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Material Properties 01 - General",
            &["Material", "Type"],
            1,
            &["STEEL_30K", "Steel"],
        ),
        OAPI_OK
    );
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Material Properties 02 - Basic Mechanical Properties",
            &["Material", "E1", "UnitWeight"],
            1,
            &["STEEL_30K", "30000.0", "0.0"],
        ),
        OAPI_OK
    );

    // 2. Stage Section (AS2 = 0, AS3 = 0 to isolate pure bending from shear deformation)
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Frame Section Properties 01 - General",
            &[
                "SectionName",
                "Material",
                "Area",
                "I33",
                "I22",
                "AS2",
                "AS3"
            ],
            1,
            &[
                "RECT_2X4",
                "STEEL_30K",
                "8.0",
                "10.666667",
                "2.666667",
                "0.0",
                "0.0"
            ],
        ),
        OAPI_OK
    );

    // 3. Stage Joints
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Joint Coordinates",
            &["Joint", "XorR", "Y", "Z"],
            2,
            &["1", "0.0", "0.0", "0.0", "2", "100.0", "0.0", "0.0"],
        ),
        OAPI_OK
    );

    // 4. Stage Restraints: Joint 1 Fixed Base
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Joint Restraint Assignments",
            &["Joint", "U1", "U2", "U3", "R1", "R2", "R3"],
            1,
            &["1", "Yes", "Yes", "Yes", "Yes", "Yes", "Yes"],
        ),
        OAPI_OK
    );

    // 5. Stage Frame & Section Assignment
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Connectivity - Frame",
            &["Frame", "JointI", "JointJ"],
            1,
            &["CANT_BEAM", "1", "2"],
        ),
        OAPI_OK
    );
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Frame Section Assignments",
            &["Frame", "AnalSect"],
            1,
            &["CANT_BEAM", "RECT_2X4"],
        ),
        OAPI_OK
    );

    // 6. Stage Pattern & Case (SelfWtMult = 0)
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Load Pattern Definitions",
            &["LoadPat", "SelfWtMult"],
            1,
            &["DEAD", "0.0"],
        ),
        OAPI_OK
    );
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Load Case Definitions",
            &["Case", "Type"],
            1,
            &["DEAD", "LinearStatic"],
        ),
        OAPI_OK
    );

    // Apply all tables
    assert_eq!(
        api.database_tables().apply_edited_tables_simple(true),
        OAPI_OK
    );

    // Apply combined point load at tip Joint 2
    api.point_obj()
        .set_load_force("2", "DEAD", [40.0, 0.0, -5.0, 0.0, 0.0, 0.0], true);

    // Solve analysis
    let solve_res = sap_solver::analyze_model(&mut api.inner);
    if let Err(ref e) = solve_res {
        eprintln!("analyze_model error: {:?}", e);
    }
    assert!(solve_res.is_ok());
    assert_eq!(api.analyze().run_analysis(), OAPI_OK);

    // --- Query Displacements Table ---
    let (code, dcols, num_d, ddata) = api.database_tables().get_table_for_display_array_vec(
        "Joint Displacements",
        &["Joint", "U1", "U3", "R2"],
        "",
    );
    assert_eq!(code, OAPI_OK);
    assert!(num_d >= 2);

    let j_idx = dcols.iter().position(|c| c == "Joint").unwrap();
    let u1_idx = dcols.iter().position(|c| c == "U1").unwrap();
    let u3_idx = dcols.iter().position(|c| c == "U3").unwrap();
    let r2_idx = dcols.iter().position(|c| c == "R2").unwrap();

    let mut tip_u1 = 0.0;
    let mut tip_u3 = 0.0;
    let mut tip_r2 = 0.0;
    for r in 0..num_d as usize {
        if ddata[r * dcols.len() + j_idx] == "2" {
            tip_u1 = ddata[r * dcols.len() + u1_idx].parse::<f64>().unwrap();
            tip_u3 = ddata[r * dcols.len() + u3_idx].parse::<f64>().unwrap();
            tip_r2 = ddata[r * dcols.len() + r2_idx].parse::<f64>().unwrap();
        }
    }

    // Verify axial elongation: Delta_x = +0.016667 in (within 0.1%)
    let expected_u1 = 40.0 * 100.0 / (30000.0 * 8.0);
    let err_u1 = (tip_u1 - expected_u1).abs() / expected_u1;
    assert!(
        err_u1 < 0.001,
        "Tip axial displacement error {} exceeds 0.1%: got {}, expected {}",
        err_u1,
        tip_u1,
        expected_u1
    );

    // Verify transverse deflection:
    // With AS2 = 0.0 parsed and assigned, shear deformation is 0.0 and beam matches pure Euler-Bernoulli theory:
    // Delta_bending = P * L^3 / (3 * E * I33) = 5.0 * 100^3 / (3 * 30000 * (128/12)) = 5.2083333 in
    let expected_bending = -5.0 * 100.0f64.powi(3) / (3.0 * 30000.0 * (128.0 / 12.0));
    let expected_u3_total = expected_bending;
    let err_u3 = (tip_u3 - expected_u3_total).abs() / expected_u3_total.abs();
    assert!(
        err_u3 < 0.001,
        "Tip vertical deflection error {} exceeds 0.1%: got {}, expected {}",
        err_u3,
        tip_u3,
        expected_u3_total
    );

    // Verify tip slope theta_y = P * L^2 / (2 * E * I) = 5.0 * 100^2 / (2 * 30000 * 10.666667) = 0.078125 rad
    let expected_r2 = 5.0 * 100.0f64.powi(2) / (2.0 * 30000.0 * (128.0 / 12.0));
    let err_r2 = (tip_r2.abs() - expected_r2).abs() / expected_r2;
    assert!(
        err_r2 < 0.001,
        "Tip rotation error {} exceeds 0.1%: got {}, expected {}",
        err_r2,
        tip_r2,
        expected_r2
    );

    // --- Query Joint Reactions Table ---
    let (code, rcols, num_r, rdata) = api.database_tables().get_table_for_display_array_vec(
        "Joint Reactions",
        &["Joint", "F1", "F3", "M2"],
        "",
    );
    assert_eq!(code, OAPI_OK);
    let rj_idx = rcols.iter().position(|c| c == "Joint").unwrap();
    let f1_idx = rcols.iter().position(|c| c == "F1").unwrap();
    let f3_idx = rcols.iter().position(|c| c == "F3").unwrap();
    let m2_idx = rcols.iter().position(|c| c == "M2").unwrap();

    let mut base_f1 = 0.0;
    let mut base_f3 = 0.0;
    let mut base_m2 = 0.0;
    for r in 0..num_r as usize {
        if rdata[r * rcols.len() + rj_idx] == "1" {
            base_f1 = rdata[r * rcols.len() + f1_idx].parse::<f64>().unwrap();
            base_f3 = rdata[r * rcols.len() + f3_idx].parse::<f64>().unwrap();
            base_m2 = rdata[r * rcols.len() + m2_idx].parse::<f64>().unwrap();
        }
    }

    assert!(
        (base_f1 - (-40.0)).abs() < 1e-3,
        "Base F1 reaction must be -40 kips, got {}",
        base_f1
    );
    assert!(
        (base_f3 - 5.0).abs() < 1e-3,
        "Base F3 reaction must be +5.0 kips, got {}",
        base_f3
    );
    assert!(
        (base_m2 - (-500.0)).abs() < 1e-2,
        "Base M2 reaction must be -500 kip-in, got {}",
        base_m2
    );

    // --- Query Base Reactions Table ---
    let (code, bcols, num_b, bdata) = api.database_tables().get_table_for_display_array_vec(
        "Base Reactions",
        &["GlobalFX", "GlobalFZ", "GlobalMY"],
        "",
    );
    assert_eq!(code, OAPI_OK);
    assert_eq!(num_b, 1);
    let fx_idx = bcols.iter().position(|c| c == "GlobalFX").unwrap();
    let fz_idx = bcols.iter().position(|c| c == "GlobalFZ").unwrap();
    let my_idx = bcols.iter().position(|c| c == "GlobalMY").unwrap();

    let g_fx = bdata[fx_idx].parse::<f64>().unwrap();
    let g_fz = bdata[fz_idx].parse::<f64>().unwrap();
    let g_my = bdata[my_idx].parse::<f64>().unwrap();

    assert!((g_fx - (-40.0)).abs() < 1e-3);
    assert!((g_fz - 5.0).abs() < 1e-3);
    assert!((g_my - (-500.0)).abs() < 1e-2);
}

// =========================================================================
// 8. Numerical Verification: Table-Imported Simply Supported Beam
// =========================================================================

#[test]
fn test_adversarial_numerical_verification_simply_supported_beam() {
    let mut api = setup_clean_model();

    // Simply supported beam:
    // Span L = 240 in, E = 29000 ksi
    // J1 at (0, 0, 0) pinned: [true, true, true, false, false, false]
    // J2 at (120, 0, 0) midspan node
    // J3 at (240, 0, 0) roller: [false, true, true, false, false, false]
    // F1 from J1 to J2, F2 from J2 to J3
    // W8x31: A = 9.13 in^2, I33 = 110.0 in^4, AS2 = 0.0 (Euler-Bernoulli)
    // Point load at midspan J2: Pz = -12.0 kips
    // Theoretical midspan deflection:
    // Delta = P * L^3 / (48 * E * I) = 12.0 * 240^3 / (48 * 29000 * 110) = 1.0833856 in
    // End reactions: Fz1 = +6.0 kips, Fz3 = +6.0 kips
    // Midspan moment: M3 = P * L / 4 = 12.0 * 240 / 4 = 720.0 kip-in

    // 1. Stage Materials
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Material Properties 01 - General",
            &["Material", "Type"],
            1,
            &["A992", "Steel"],
        ),
        OAPI_OK
    );
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Material Properties 02 - Basic Mechanical Properties",
            &["Material", "E1", "UnitWeight"],
            1,
            &["A992", "29000.0", "0.0"],
        ),
        OAPI_OK
    );

    // 2. Stage Section
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Frame Section Properties 01 - General",
            &["SectionName", "Material", "Area", "I33", "I22", "AS2"],
            1,
            &["W8X31", "A992", "9.13", "110.0", "37.1", "0.0"],
        ),
        OAPI_OK
    );

    // 3. Stage Joints
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Joint Coordinates",
            &["Joint", "XorR", "Y", "Z"],
            3,
            &[
                "J1", "0.0", "0.0", "0.0", "J2", "120.0", "0.0", "0.0", "J3", "240.0", "0.0",
                "0.0",
            ],
        ),
        OAPI_OK
    );

    // 4. Stage Restraints: J1 pinned (U1, U2, U3, R1), J3 roller (U2, U3, R1)
    // In 3D space, beam in X-Z plane must restrain out-of-plane translation U2 and torsion R1 to avoid rigid body singularity
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Joint Restraint Assignments",
            &["Joint", "U1", "U2", "U3", "R1", "R2", "R3"],
            2,
            &[
                "J1", "Yes", "Yes", "Yes", "Yes", "No", "No", "J3", "No", "Yes", "Yes", "Yes",
                "No", "No",
            ],
        ),
        OAPI_OK
    );

    // 5. Stage Frames & Section Assignments
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Connectivity - Frame",
            &["Frame", "JointI", "JointJ"],
            2,
            &["F1", "J1", "J2", "F2", "J2", "J3",],
        ),
        OAPI_OK
    );
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Frame Section Assignments",
            &["Frame", "AnalSect"],
            2,
            &["F1", "W8X31", "F2", "W8X31",],
        ),
        OAPI_OK
    );

    // 6. Stage Pattern & Case
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Load Pattern Definitions",
            &["LoadPat", "SelfWtMult"],
            1,
            &["DEAD", "0.0"],
        ),
        OAPI_OK
    );
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Load Case Definitions",
            &["Case", "Type"],
            1,
            &["DEAD", "LinearStatic"],
        ),
        OAPI_OK
    );

    // Apply all staged tables
    assert_eq!(
        api.database_tables().apply_edited_tables_simple(true),
        OAPI_OK
    );

    // Apply point load at midspan J2
    api.point_obj()
        .set_load_force("J2", "DEAD", [0.0, 0.0, -12.0, 0.0, 0.0, 0.0], true);

    // Solve analysis
    let solve_res = sap_solver::analyze_model(&mut api.inner);
    if let Err(ref e) = solve_res {
        eprintln!("analyze_model error: {:?}", e);
    }
    assert!(solve_res.is_ok());
    assert_eq!(api.analyze().run_analysis(), OAPI_OK);

    // Verify midspan deflection via Joint Displacements table
    let (code, cols, num_d, ddata) = api.database_tables().get_table_for_display_array_vec(
        "Joint Displacements",
        &["Joint", "U3"],
        "",
    );
    assert_eq!(code, OAPI_OK);
    let j_idx = cols.iter().position(|c| c == "Joint").unwrap();
    let u3_idx = cols.iter().position(|c| c == "U3").unwrap();

    let mut midspan_u3 = 0.0;
    for r in 0..num_d as usize {
        if ddata[r * cols.len() + j_idx] == "J2" {
            midspan_u3 = ddata[r * cols.len() + u3_idx].parse::<f64>().unwrap();
        }
    }

    // Theoretical midspan deflection with AS2 = 0.0 (Euler-Bernoulli, shear deformation disabled):
    // Delta_bending = P * L^3 / (48 * E * I) = 12.0 * 240^3 / (48 * 29000 * 110) = 1.0833856 in
    let expected_bending = -12.0 * 240.0f64.powi(3) / (48.0 * 29000.0 * 110.0);
    let expected_delta_total = expected_bending;
    let rel_err = (midspan_u3 - expected_delta_total).abs() / expected_delta_total.abs();
    assert!(
        rel_err < 0.001,
        "Midspan deflection error {} exceeds 0.1%: got {}, expected {}",
        rel_err,
        midspan_u3,
        expected_delta_total
    );

    // Verify end reactions via Joint Reactions table
    let (code, rcols, num_r, rdata) = api.database_tables().get_table_for_display_array_vec(
        "Joint Reactions",
        &["Joint", "F3"],
        "",
    );
    assert_eq!(code, OAPI_OK);
    let rj_idx = rcols.iter().position(|c| c == "Joint").unwrap();
    let f3_idx = rcols.iter().position(|c| c == "F3").unwrap();

    let mut r1_f3 = 0.0;
    let mut r3_f3 = 0.0;
    for r in 0..num_r as usize {
        let jt = &rdata[r * rcols.len() + rj_idx];
        let f3 = rdata[r * rcols.len() + f3_idx].parse::<f64>().unwrap();
        if jt == "J1" {
            r1_f3 = f3;
        } else if jt == "J3" {
            r3_f3 = f3;
        }
    }

    assert!(
        (r1_f3 - 6.0).abs() < 1e-3,
        "J1 reaction must be 6.0 kips, got {}",
        r1_f3
    );
    assert!(
        (r3_f3 - 6.0).abs() < 1e-3,
        "J3 reaction must be 6.0 kips, got {}",
        r3_f3
    );
}

// =========================================================================
// 9. Adversarial Vulnerability Probes: Schema Coverage & Parity Gaps
// =========================================================================

#[test]
fn test_adversarial_vulnerability_dropped_section_shape_and_shear_properties() {
    let mut api = setup_clean_model();

    // Stage material
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Material Properties 01 - General",
            &["Material", "Type"],
            1,
            &["MAT1", "Steel"],
        ),
        OAPI_OK
    );

    // Stage Section Properties specifying custom AS2, AS3, S33, S22, Z33, Z22, R33, R22
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Frame Section Properties 01 - General",
            &[
                "SectionName",
                "Material",
                "Area",
                "I33",
                "I22",
                "AS2",
                "AS3",
                "S33",
                "S22",
                "Z33",
                "Z22",
                "R33",
                "R22"
            ],
            1,
            &[
                "SEC_CUSTOM",
                "MAT1",
                "15.0",
                "300.0",
                "120.0",
                "4.5",
                "5.5",
                "40.0",
                "20.0",
                "48.0",
                "24.0",
                "4.4",
                "2.8"
            ],
        ),
        OAPI_OK
    );

    assert_eq!(
        api.database_tables().apply_edited_tables_simple(true),
        OAPI_OK
    );

    let sec = api
        .inner
        .frame_sections
        .get("SEC_CUSTOM")
        .expect("SEC_CUSTOM must exist");
    assert_eq!(sec.area, 15.0);
    assert_eq!(sec.i33, 300.0);
    assert_eq!(sec.i22, 120.0);

    // VULNERABILITY PROBE: Check if AS2 and AS3 were actually persisted or discarded
    let as2_persisted = (sec.as2 - 4.5).abs() < 1e-4;
    let as3_persisted = (sec.as3 - 5.5).abs() < 1e-4;
    eprintln!(
        "PROBE RESULT: sec.as2 = {} (expected 4.5, persisted={}), sec.as3 = {} (expected 5.5, persisted={})",
        sec.as2, as2_persisted, sec.as3, as3_persisted
    );
    assert!(as2_persisted, "sec.as2 must be 4.5, got {}", sec.as2);
    assert!(as3_persisted, "sec.as3 must be 5.5, got {}", sec.as3);
    assert!(
        (sec.s33 - 40.0).abs() < 1e-4,
        "sec.s33 must be 40.0, got {}",
        sec.s33
    );
    assert!(
        (sec.s22 - 20.0).abs() < 1e-4,
        "sec.s22 must be 20.0, got {}",
        sec.s22
    );
    assert!(
        (sec.z33 - 48.0).abs() < 1e-4,
        "sec.z33 must be 48.0, got {}",
        sec.z33
    );
    assert!(
        (sec.z22 - 24.0).abs() < 1e-4,
        "sec.z22 must be 24.0, got {}",
        sec.z22
    );
    assert!(
        (sec.r33 - 4.4).abs() < 1e-4,
        "sec.r33 must be 4.4, got {}",
        sec.r33
    );
    assert!(
        (sec.r22 - 2.8).abs() < 1e-4,
        "sec.r22 must be 2.8, got {}",
        sec.r22
    );
}

#[test]
fn test_adversarial_vulnerability_dropped_material_fys_unitmass_a1() {
    let mut api = setup_clean_model();

    // Stage Material General
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Material Properties 01 - General",
            &["Material", "Type"],
            1,
            &["MAT_REBAR", "Rebar"],
        ),
        OAPI_OK
    );

    // Stage Mechanical with M1 parity fields Fys, UnitMass, A1
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Material Properties 02 - Basic Mechanical Properties",
            &["Material", "E1", "Fy", "Fys", "UnitMass", "A1"],
            1,
            &["MAT_REBAR", "29000.0", "60.0", "75.0", "0.000735", "6.5e-6"],
        ),
        OAPI_OK
    );

    assert_eq!(
        api.database_tables().apply_edited_tables_simple(true),
        OAPI_OK
    );

    let mat = api
        .inner
        .materials
        .get("MAT_REBAR")
        .expect("MAT_REBAR must exist");
    assert_eq!(mat.e, 29000.0);
    assert_eq!(mat.fy, 60.0);

    // VULNERABILITY PROBE: Check if Fys, UnitMass, and A1 were persisted
    let fys_persisted = (mat.fys - 75.0).abs() < 1e-4;
    let unit_mass_persisted = (mat.unit_mass - 0.000735).abs() < 1e-6;
    let a1_persisted = (mat.a - 6.5e-6).abs() < 1e-9;
    eprintln!(
        "PROBE RESULT: mat.fys = {} (expected 75.0, persisted={}), mat.unit_mass = {} (expected 0.000735, persisted={}), mat.a = {} (expected 6.5e-6, persisted={})",
        mat.fys, fys_persisted, mat.unit_mass, unit_mass_persisted, mat.a, a1_persisted
    );
    assert!(fys_persisted, "mat.fys must be 75.0, got {}", mat.fys);
    assert!(
        unit_mass_persisted,
        "mat.unit_mass must be 0.000735, got {}",
        mat.unit_mass
    );
    assert!(a1_persisted, "mat.a must be 6.5e-6, got {}", mat.a);
}

#[test]
fn test_adversarial_vulnerability_material_properties_silent_pass_on_malformed_float() {
    let mut api = setup_clean_model();

    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Material Properties 01 - General",
            &["Material", "Type"],
            1,
            &["MAT_TEST", "Steel"],
        ),
        OAPI_OK
    );

    // Inject non-numeric string in E1
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Material Properties 02 - Basic Mechanical Properties",
            &["Material", "E1"],
            1,
            &["MAT_TEST", "CORRUPT_NOT_A_FLOAT"],
        ),
        OAPI_OK
    );

    let mut fatal = 0;
    let mut err = 0;
    let mut warn = 0;
    let mut info = 0;
    let mut log = String::new();

    let res = api
        .database_tables()
        .apply_edited_tables(true, &mut fatal, &mut err, &mut warn, &mut info, &mut log);

    eprintln!(
        "PROBE RESULT: Malformed E1 apply status = {} (OAPI_OK={}, OAPI_ERR={}), fatal errors = {}",
        res, OAPI_OK, OAPI_ERR, fatal
    );
    assert_eq!(
        res, OAPI_ERR,
        "Malformed E1 must cause apply_edited_tables to return OAPI_ERR"
    );
    assert!(fatal > 0, "Fatal error count must be > 0");
}

#[test]
fn test_adversarial_vulnerability_combo_case_duplication_on_repeated_apply() {
    let mut api = setup_clean_model();

    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Load Pattern Definitions",
            &["LoadPat", "SelfWtMult"],
            1,
            &["DEAD", "1.0"],
        ),
        OAPI_OK
    );
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Load Case Definitions",
            &["Case", "Type"],
            1,
            &["DEAD", "LinearStatic"],
        ),
        OAPI_OK
    );
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Combination Definitions",
            &["ComboName", "ComboType", "CaseName", "ScaleFactor"],
            1,
            &["COMB1", "LinearAdd", "DEAD", "1.4"],
        ),
        OAPI_OK
    );

    // Apply first time
    assert_eq!(
        api.database_tables().apply_edited_tables_simple(true),
        OAPI_OK
    );
    let c1 = api.inner.load_combos.get("COMB1").unwrap();
    assert_eq!(c1.cases.len(), 1);

    // Stage and apply second time (e.g. updating combinations)
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Combination Definitions",
            &["ComboName", "ComboType", "CaseName", "ScaleFactor"],
            1,
            &["COMB1", "LinearAdd", "DEAD", "1.4"],
        ),
        OAPI_OK
    );
    assert_eq!(
        api.database_tables().apply_edited_tables_simple(true),
        OAPI_OK
    );

    let c2 = api.inner.load_combos.get("COMB1").unwrap();
    eprintln!(
        "PROBE RESULT: COMB1 cases count after re-apply = {} (expected 1, actual cases={:?})",
        c2.cases.len(),
        c2.cases
    );
    assert_eq!(
        c2.cases.len(),
        1,
        "Repeated apply must not duplicate combo cases"
    );
    assert_eq!(c2.cases[0], ("DEAD".to_string(), 1.4));
}

#[test]
fn test_adversarial_vulnerability_load_case_hardcodes_dead_pattern() {
    let mut api = setup_clean_model();

    // Define pattern "LIVE"
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Load Pattern Definitions",
            &["LoadPat", "SelfWtMult"],
            1,
            &["LIVE", "0.0"],
        ),
        OAPI_OK
    );

    // Define load case "LIVE_CASE"
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Load Case Definitions",
            &["Case", "Type"],
            1,
            &["LIVE_CASE", "LinearStatic"],
        ),
        OAPI_OK
    );

    assert_eq!(
        api.database_tables().apply_edited_tables_simple(true),
        OAPI_OK
    );

    let lc = api.inner.load_cases.get("LIVE_CASE").unwrap();
    eprintln!(
        "PROBE RESULT: LIVE_CASE loads = {:?} (associated with pattern LIVE)",
        lc.loads
    );
    assert_eq!(lc.loads.len(), 1);
    assert_eq!(
        lc.loads[0].load_name, "LIVE",
        "Load case must associate with pattern 'LIVE'"
    );
}
