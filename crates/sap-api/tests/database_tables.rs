use sap_api::{cSapModel, OAPI_ERR, OAPI_OK};
use sap_units::Units;

fn setup_base_model() -> cSapModel {
    let mut api = cSapModel::new(Units::KipInF);
    api.point_obj().add_cartesian(0.0, 0.0, 0.0, "1");
    api.point_obj().add_cartesian(144.0, 0.0, 0.0, "2");
    api.frame_obj().add_by_point("1", "2", "FSEC1", "F1");
    api
}

#[test]
fn test_database_tables_discovery_and_metadata() {
    let mut api = setup_base_model();

    // 1. GetAvailableTables
    let mut num_tables = 0;
    let mut keys = Vec::new();
    let mut names = Vec::new();
    let mut types = Vec::new();
    assert_eq!(
        api.database_tables().get_available_tables(
            &mut num_tables,
            &mut keys,
            &mut names,
            &mut types
        ),
        OAPI_OK
    );
    assert!(num_tables >= 15);
    assert_eq!(keys.len(), num_tables as usize);
    assert_eq!(names.len(), num_tables as usize);
    assert_eq!(types.len(), num_tables as usize);

    assert!(keys.contains(&"JOINT COORDINATES".to_string()));
    assert!(keys.contains(&"CONNECTIVITY - FRAME".to_string()));
    assert!(keys.contains(&"FRAME SECTION ASSIGNMENTS".to_string()));
    assert!(keys.contains(&"MATERIAL PROPERTIES 01 - GENERAL".to_string()));
    assert!(keys.contains(&"FRAME SECTION PROPERTIES 01 - GENERAL".to_string()));
    assert!(keys.contains(&"JOINT DISPLACEMENTS".to_string()));
    assert!(keys.contains(&"ELEMENT FORCES - FRAMES".to_string()));

    // 2. GetAllTables and IsEmpty status
    let mut empty_flags = Vec::new();
    assert_eq!(
        api.database_tables().get_all_tables_key_info(
            &mut num_tables,
            &mut keys,
            &mut names,
            &mut types,
            &mut empty_flags
        ),
        OAPI_OK
    );
    assert_eq!(empty_flags.len(), keys.len());

    let idx_joints = keys.iter().position(|k| k == "JOINT COORDINATES").unwrap();
    assert!(!empty_flags[idx_joints]); // Populated with joints "1" and "2"

    let idx_disp = keys
        .iter()
        .position(|k| k == "JOINT DISPLACEMENTS")
        .unwrap();
    assert!(empty_flags[idx_disp]); // Analysis not run yet

    // 3. GetAllFieldsInTable
    let mut version = 0;
    let mut num_fields = 0;
    let mut fkeys = Vec::new();
    let mut fnames = Vec::new();
    let mut descs = Vec::new();
    let mut units = Vec::new();
    let mut importable = Vec::new();

    assert_eq!(
        api.database_tables().get_all_fields_in_table(
            "JOINT COORDINATES",
            &mut version,
            &mut num_fields,
            &mut fkeys,
            &mut fnames,
            &mut descs,
            &mut units,
            &mut importable,
        ),
        OAPI_OK
    );
    assert!(num_fields >= 7);
    assert!(fkeys.contains(&"Joint".to_string()));
    assert!(fkeys.contains(&"XorR".to_string()));
    assert!(fkeys.contains(&"Y".to_string()));
    assert!(fkeys.contains(&"Z".to_string()));

    let jt_idx = fkeys.iter().position(|k| k == "Joint").unwrap();
    assert!(importable[jt_idx]);

    // Reject unknown table key
    assert_eq!(
        api.database_tables().get_all_fields_in_table(
            "NON_EXISTENT_TABLE",
            &mut version,
            &mut num_fields,
            &mut fkeys,
            &mut fnames,
            &mut descs,
            &mut units,
            &mut importable,
        ),
        OAPI_ERR
    );
}

#[test]
fn test_get_table_for_display_array_and_field_filtering() {
    let mut api = setup_base_model();

    // 1. All fields display extraction
    let mut version = 0;
    let mut included = Vec::new();
    let mut num_records = 0;
    let mut data = Vec::new();

    assert_eq!(
        api.database_tables().get_table_for_display_array(
            "Joint Coordinates",
            &[],
            "All",
            &mut version,
            &mut included,
            &mut num_records,
            &mut data,
        ),
        OAPI_OK
    );
    assert_eq!(num_records, 2);
    let k = included.len();
    assert_eq!(data.len(), (num_records as usize) * k);

    // Row 0 should be Joint 1 at (0, 0, 0)
    let jt_col = included.iter().position(|f| f == "Joint").unwrap();
    let x_col = included.iter().position(|f| f == "XorR").unwrap();
    assert_eq!(data[jt_col], "1");
    assert_eq!(data[x_col].parse::<f64>().unwrap(), 0.0);

    // Row 1 should be Joint 2 at (144, 0, 0)
    assert_eq!(data[k + jt_col], "2");
    assert_eq!(data[k + x_col].parse::<f64>().unwrap(), 144.0);

    // 2. Specific field filtering
    let filter_req = vec!["Z".to_string(), "Joint".to_string()];
    assert_eq!(
        api.database_tables().get_table_for_display_array(
            "JOINT COORDINATES",
            &filter_req,
            "",
            &mut version,
            &mut included,
            &mut num_records,
            &mut data,
        ),
        OAPI_OK
    );
    assert_eq!(included, vec!["Z".to_string(), "Joint".to_string()]);
    assert_eq!(num_records, 2);
    assert_eq!(data.len(), 4); // 2 rows * 2 cols
    assert_eq!(data[0].parse::<f64>().unwrap(), 0.0);
    assert_eq!(data[1], "1");
    assert_eq!(data[2].parse::<f64>().unwrap(), 0.0);
    assert_eq!(data[3], "2");

    // 3. Case-insensitivity & "TABLE: " prefix
    assert_eq!(
        api.database_tables().get_table_for_display_array(
            "table: \"connectivity - frame\"",
            &[],
            "ALL",
            &mut version,
            &mut included,
            &mut num_records,
            &mut data,
        ),
        OAPI_OK
    );
    assert_eq!(num_records, 1);
    let f_col = included.iter().position(|f| f == "Frame").unwrap();
    assert_eq!(data[f_col], "F1");
}

#[test]
fn test_get_table_for_editing_array_rejects_results() {
    let mut api = setup_base_model();

    let mut version = 0;
    let mut included = Vec::new();
    let mut num_records = 0;
    let mut data = Vec::new();

    // Definition table succeeds
    assert_eq!(
        api.database_tables().get_table_for_editing_array(
            "Joint Coordinates",
            "",
            &mut version,
            &mut included,
            &mut num_records,
            &mut data,
        ),
        OAPI_OK
    );
    assert_eq!(num_records, 2);

    // Result table rejected with OAPI_ERR
    assert_eq!(
        api.database_tables().get_table_for_editing_array(
            "Joint Displacements",
            "",
            &mut version,
            &mut included,
            &mut num_records,
            &mut data,
        ),
        OAPI_ERR
    );
}

#[test]
fn test_set_table_for_editing_array_validation() {
    let mut api = setup_base_model();

    // 1. Dimension mismatch rejected (2 records * 3 fields = 6 data elements, but 5 provided)
    let fields = vec!["Joint".to_string(), "XorR".to_string(), "Y".to_string()];
    let bad_data = vec![
        "1".to_string(),
        "0.0".to_string(),
        "0.0".to_string(),
        "2".to_string(),
        "100.0".to_string(),
    ];
    let mut ver = 1;
    assert_eq!(
        api.database_tables().set_table_for_editing_array(
            "Joint Coordinates",
            &mut ver,
            &fields,
            2,
            &bad_data,
        ),
        OAPI_ERR
    );
    assert!(api.staged_tables.is_empty());

    // 2. Missing primary key rejected ("XorR", "Y", "Z" without "Joint")
    let no_key_fields = vec!["XorR".to_string(), "Y".to_string(), "Z".to_string()];
    let no_key_data = vec!["0.0".to_string(), "0.0".to_string(), "0.0".to_string()];
    assert_eq!(
        api.database_tables().set_table_for_editing_array(
            "Joint Coordinates",
            &mut ver,
            &no_key_fields,
            1,
            &no_key_data,
        ),
        OAPI_ERR
    );

    // 3. Staging read-only results table rejected
    assert_eq!(
        api.database_tables().set_table_for_editing_array(
            "Joint Displacements",
            &mut ver,
            &fields,
            1,
            &["1".into(), "0".into(), "0".into()],
        ),
        OAPI_ERR
    );

    // 4. Locked model rejects staging
    api.set_model_is_locked(true);
    let valid_data = vec!["1".to_string(), "10.0".to_string(), "20.0".to_string()];
    assert_eq!(
        api.database_tables().set_table_for_editing_array(
            "Joint Coordinates",
            &mut ver,
            &fields,
            1,
            &valid_data,
        ),
        OAPI_ERR
    );
    api.set_model_is_locked(false);

    // 5. Valid staging succeeds and persists in staged_tables
    assert_eq!(
        api.database_tables().set_table_for_editing_array(
            "Joint Coordinates",
            &mut ver,
            &fields,
            1,
            &valid_data,
        ),
        OAPI_OK
    );
    assert_eq!(api.staged_tables.len(), 1);

    // 6. Cancel clears staged tables
    assert_eq!(api.database_tables().cancel_table_editing(), OAPI_OK);
    assert!(api.staged_tables.is_empty());
}

#[test]
fn test_apply_edited_tables_modifies_geometry_and_invalidates_results() {
    let mut api = setup_base_model();

    // Populate mock results on unlocked model
    api.inner
        .case_results
        .insert("DEAD".into(), sap_model::CaseResults::default());
    assert!(!api.inner.case_results.is_empty());

    // Stage update to Joint "2" coordinates
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Joint Coordinates",
            &["Joint", "XorR", "Y", "Z"],
            1,
            &["2", "200.0", "50.0", "25.0"],
        ),
        OAPI_OK
    );

    // Apply edits
    let mut fatal = 0;
    let mut err = 0;
    let mut warn = 0;
    let mut info = 0;
    let mut log = String::new();
    assert_eq!(
        api.database_tables()
            .apply_edited_tables(true, &mut fatal, &mut err, &mut warn, &mut info, &mut log,),
        OAPI_OK
    );
    assert_eq!(fatal, 0);

    // Verify coordinates updated
    let p2 = api.inner.points.get("2").unwrap();
    assert_eq!(p2.x, 200.0);
    assert_eq!(p2.y, 50.0);
    assert_eq!(p2.z, 25.0);

    // Verify analysis results invalidated
    assert!(api.inner.case_results.is_empty());
    assert!(api.staged_tables.is_empty());
}

#[test]
fn test_apply_edited_tables_dependency_ordering() {
    let mut api = cSapModel::new(Units::KipInF);

    // Stage in REVERSE dependency order:
    // Frame Connectivity -> Joint Coordinates -> Frame Section -> Material
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Connectivity - Frame",
            &["Frame", "JointI", "JointJ"],
            1,
            &["BEAM1", "J1", "J2"],
        ),
        OAPI_OK
    );

    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Joint Coordinates",
            &["Joint", "XorR", "Y", "Z"],
            2,
            &["J1", "0.0", "0.0", "0.0", "J2", "120.0", "0.0", "0.0"],
        ),
        OAPI_OK
    );

    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Frame Section Properties 01 - General",
            &["SectionName", "Material", "Area", "I33", "I22"],
            1,
            &["SEC_NEW", "MAT_STEEL", "15.0", "200.0", "80.0"],
        ),
        OAPI_OK
    );

    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Material Properties 01 - General",
            &["Material", "Type"],
            1,
            &["MAT_STEEL", "Steel"],
        ),
        OAPI_OK
    );

    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Frame Section Assignments",
            &["Frame", "AnalSect"],
            1,
            &["BEAM1", "SEC_NEW"],
        ),
        OAPI_OK
    );

    // Apply all staged tables together: dependency order ensures
    // Material -> Section -> Joints -> Frame -> Section Assignment succeeds!
    assert_eq!(
        api.database_tables().apply_edited_tables_simple(true),
        OAPI_OK
    );

    assert!(api.inner.materials.contains_key("MAT_STEEL"));
    assert!(api.inner.frame_sections.contains_key("SEC_NEW"));
    assert!(api.inner.points.contains_key("J1"));
    assert!(api.inner.points.contains_key("J2"));
    let frame = api.inner.frames.get("BEAM1").unwrap();
    assert_eq!(frame.point_i, "J1");
    assert_eq!(frame.point_j, "J2");
    assert_eq!(frame.section, "SEC_NEW");
}

#[test]
fn test_apply_edited_tables_atomic_rollback_on_failure() {
    let mut api = setup_base_model();
    let before = api.inner.clone();

    // 1. Stage frame with non-existent joint
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Connectivity - Frame",
            &["Frame", "JointI", "JointJ"],
            1,
            &["BAD_FRAME", "1", "MISSING_JOINT"],
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
    assert!(log.contains("not found"));

    // Entire model must remain 100% unchanged
    assert_eq!(api.inner, before);
    assert!(api.staged_tables.is_empty());

    // 2. Stage joint coordinate with corrupt float
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Joint Coordinates",
            &["Joint", "XorR", "Y", "Z"],
            2,
            &[
                "1",
                "10.0",
                "10.0",
                "10.0",
                "2",
                "NOT_A_NUMBER",
                "0.0",
                "0.0"
            ],
        ),
        OAPI_OK
    );
    assert_eq!(
        api.database_tables().apply_edited_tables_simple(true),
        OAPI_ERR
    );
    assert_eq!(api.inner, before);
}

#[test]
fn test_database_tables_results_extraction_and_e2e_edit_analyze() {
    let mut api = setup_base_model();

    // Disable self-weight multiplier to isolate point load reaction
    if let Some(lp) = api.inner.load_patterns.get_mut("DEAD") {
        lp.self_wt_multiplier = 0.0;
    }

    // Restrain Joint 1 (fixed cantilever)
    api.point_obj().set_restraint("1", [true; 6]);

    // Apply tip point load at Joint 2
    api.point_obj()
        .set_load_force("2", "DEAD", [0.0, 0.0, -10.0, 0.0, 0.0, 0.0], true);

    // Run initial analysis
    assert_eq!(api.analyze().run_analysis(), OAPI_OK);

    // Extract Results Tables
    let (code, cols, num_disp, disp_data) =
        api.database_tables()
            .get_table_for_display_array_vec("Joint Displacements", &[], "All");
    assert_eq!(code, OAPI_OK);
    assert!(num_disp >= 2);
    let u3_col = cols.iter().position(|c| c == "U3").unwrap();
    let jt_col = cols.iter().position(|c| c == "Joint").unwrap();

    let mut u3_tip_initial = 0.0;
    for r in 0..num_disp as usize {
        if disp_data[r * cols.len() + jt_col] == "2" {
            u3_tip_initial = disp_data[r * cols.len() + u3_col].parse::<f64>().unwrap();
        }
    }
    assert!(
        u3_tip_initial < 0.0,
        "Cantilever tip should deflect downward"
    );

    // Extract Reactions Table
    let (code, _rcols, num_react, react_data) = api
        .database_tables()
        .get_table_for_display_array_vec("Joint Reactions", &["Joint", "F3"], "");
    assert_eq!(code, OAPI_OK);
    assert!(num_react >= 1);
    let f3_val = react_data[1].parse::<f64>().unwrap();
    assert!(
        (f3_val - 10.0).abs() < 1e-3,
        "Support reaction must balance 10 kip load"
    );

    // Extract Base Reactions
    let (code, bcols, num_base, base_data) =
        api.database_tables()
            .get_table_for_display_array_vec("Base Reactions", &[], "");
    assert_eq!(code, OAPI_OK);
    assert_eq!(num_base, 1);
    let fz_col = bcols.iter().position(|c| c == "GlobalFZ").unwrap();
    assert!((base_data[fz_col].parse::<f64>().unwrap() - 10.0).abs() < 1e-3);

    // Unlock model to allow interactive database editing
    assert_eq!(api.set_model_is_locked(false), OAPI_OK);

    // EXTEND cantilever member via Database Tables:
    // Change Joint 2 X from 144.0 to 180.0
    assert_eq!(
        api.database_tables().set_table_for_editing_array_str(
            "Joint Coordinates",
            &["Joint", "XorR", "Y", "Z"],
            1,
            &["2", "180.0", "0.0", "0.0"],
        ),
        OAPI_OK
    );
    assert_eq!(
        api.database_tables().apply_edited_tables_simple(true),
        OAPI_OK
    );

    // Re-run analysis on updated geometry
    assert_eq!(api.analyze().run_analysis(), OAPI_OK);

    // Query new tip deflection
    let (_, cols, num_disp_new, disp_data_new) = api
        .database_tables()
        .get_table_for_display_array_vec("Joint Displacements", &[], "");
    let mut u3_tip_extended = 0.0;
    for r in 0..num_disp_new as usize {
        if disp_data_new[r * cols.len() + jt_col] == "2" {
            u3_tip_extended = disp_data_new[r * cols.len() + u3_col]
                .parse::<f64>()
                .unwrap();
        }
    }

    // Extended cantilever deflection (delta ~ L^3) must be strictly greater in magnitude
    assert!(
        u3_tip_extended.abs() > u3_tip_initial.abs(),
        "Extended cantilever tip deflection ({}) must exceed original ({})",
        u3_tip_extended,
        u3_tip_initial
    );
}
