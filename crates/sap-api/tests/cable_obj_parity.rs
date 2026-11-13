use sap_api::{cSapModel, OAPI_ERR, OAPI_OK};
use sap_units::Units;

fn setup_base_model() -> cSapModel {
    let mut api = cSapModel::new(Units::KipInF);
    api.point_obj().add_cartesian(0.0, 0.0, 0.0, "P1");
    api.point_obj().add_cartesian(120.0, 0.0, 0.0, "P2");
    api.point_obj().add_cartesian(0.0, 100.0, 50.0, "P3");
    api.point_obj().add_cartesian(120.0, 100.0, 50.0, "P4");
    api
}

#[test]
fn test_cable_creation_by_point_and_coord() {
    let mut api = setup_base_model();

    // 1. Add by point with auto-generated name
    let (code1, name1) = api.cable_obj().add_by_point("P1", "P2", "FSEC1", "");
    assert_eq!(code1, OAPI_OK);
    assert!(!name1.is_empty());
    assert_eq!(api.cable_obj().count(), 1);

    // 2. Add by point with custom name
    let (code2, name2) = api.cable_obj().add_by_point("P3", "P4", "FSEC1", "C_MAIN");
    assert_eq!(code2, OAPI_OK);
    assert_eq!(name2, "C_MAIN");
    assert_eq!(api.cable_obj().count(), 2);

    let (list_code, names) = api.cable_obj().get_name_list();
    assert_eq!(list_code, OAPI_OK);
    assert!(names.contains(&name1));
    assert!(names.contains(&"C_MAIN".to_string()));

    // 3. Add by coordinate (auto-generating new joints)
    let (code3, name3) = api
        .cable_obj()
        .add_by_coord(10.0, 20.0, 30.0, 110.0, 120.0, 130.0, "FSEC1", "C_COORD");
    assert_eq!(code3, OAPI_OK);
    assert_eq!(name3, "C_COORD");
    assert_eq!(api.cable_obj().count(), 3);

    // Verify endpoints of coord cable
    let (pts_code, p1, p2) = api.cable_obj().get_points("C_COORD");
    assert_eq!(pts_code, OAPI_OK);
    assert_ne!(p1, p2);
    assert!(api.inner.points.contains_key(&p1));
    assert!(api.inner.points.contains_key(&p2));

    // 4. Add by coordinate with matching coords merges joints without duplicate creation
    let pt_count_before = api.inner.points.len();
    let (code4, name4) = api
        .cable_obj()
        .add_by_coord(0.0, 0.0, 0.0, 120.0, 0.0, 0.0, "FSEC1", "C_MERGE");
    assert_eq!(code4, OAPI_OK);
    assert_eq!(name4, "C_MERGE");
    assert_eq!(api.inner.points.len(), pt_count_before); // Merged into existing P1 & P2
}

#[test]
fn test_cable_creation_rejections_and_atomicity() {
    let mut api = setup_base_model();

    // 1. Missing point
    let before = api.inner.clone();
    let (code, name) = api
        .cable_obj()
        .add_by_point("P1", "NON_EXISTENT", "FSEC1", "BAD1");
    assert_eq!(code, OAPI_ERR);
    assert!(name.is_empty());
    assert_eq!(api.inner, before);

    // 2. Coincident points
    let (code, name) = api.cable_obj().add_by_point("P1", "P1", "FSEC1", "BAD2");
    assert_eq!(code, OAPI_ERR);
    assert!(name.is_empty());
    assert_eq!(api.inner, before);

    // 3. Missing section
    let (code, name) = api
        .cable_obj()
        .add_by_point("P1", "P2", "NON_EXISTENT_SEC", "BAD3");
    assert_eq!(code, OAPI_ERR);
    assert!(name.is_empty());
    assert_eq!(api.inner, before);

    // 4. Orphan joint prevention: add_by_coord with missing section or invalid name
    let pt_count_before = api.inner.points.len();
    let (code, _) = api.cable_obj().add_by_coord(
        99.0,
        88.0,
        77.0,
        199.0,
        188.0,
        177.0,
        "NON_EXISTENT_SEC",
        "BAD_ORPHAN",
    );
    assert_eq!(code, OAPI_ERR);
    assert_eq!(api.inner.points.len(), pt_count_before);
    assert_eq!(api.inner, before);

    // 5. Zero-length coord rejected
    let (code, _) = api
        .cable_obj()
        .add_by_coord(50.0, 50.0, 50.0, 50.0, 50.0, 50.0, "FSEC1", "BAD_ZERO");
    assert_eq!(code, OAPI_ERR);
    assert_eq!(api.inner, before);

    // 6. Name collision with existing frame or cable
    api.frame_obj().add_by_point("P1", "P2", "FSEC1", "FRAME1");
    let (code, _) = api.cable_obj().add_by_point("P3", "P4", "FSEC1", "FRAME1");
    assert_eq!(code, OAPI_ERR);
}

#[test]
fn test_cable_connectivity_and_section() {
    let mut api = setup_base_model();
    let (code, name) = api.cable_obj().add_by_point("P1", "P2", "FSEC1", "CABLE1");
    assert_eq!(code, OAPI_OK);

    // Get points
    let (p_code, p1, p2) = api.cable_obj().get_points(&name);
    assert_eq!(p_code, OAPI_OK);
    assert_eq!(p1, "P1");
    assert_eq!(p2, "P2");

    // Modify points to P3 and P4
    assert_eq!(api.cable_obj().set_points(&name, "P3", "P4"), OAPI_OK);
    let (_, p1_new, p2_new) = api.cable_obj().get_points(&name);
    assert_eq!(p1_new, "P3");
    assert_eq!(p2_new, "P4");

    // Rejection on invalid set_points
    assert_eq!(api.cable_obj().set_points(&name, "P3", "P3"), OAPI_ERR);
    assert_eq!(api.cable_obj().set_points(&name, "P3", "MISSING"), OAPI_ERR);

    // Section inspection and update
    let (s_code, sec) = api.cable_obj().get_section(&name);
    assert_eq!(s_code, OAPI_OK);
    assert_eq!(sec, "FSEC1");

    // Add a new section and assign it
    api.prop_frame()
        .set_i_section("CABLE_SEC", "A992Fy50", 10.0, 10.0, 0.5, 0.5, 10.0, 0.5);
    assert_eq!(api.cable_obj().set_section(&name, "CABLE_SEC"), OAPI_OK);
    assert_eq!(api.cable_obj().get_section(&name).1, "CABLE_SEC");

    // Reject unknown section
    assert_eq!(api.cable_obj().set_section(&name, "NON_EXISTENT"), OAPI_ERR);
    assert_eq!(api.cable_obj().get_section(&name).1, "CABLE_SEC");
}

#[test]
fn test_cable_tc_limits() {
    let mut api = setup_base_model();
    api.cable_obj().add_by_point("P1", "P2", "FSEC1", "C1");

    // Default: no limits
    let (code, c_ex, c_lim, t_ex, t_lim) = api.cable_obj().get_tension_compression_limits("C1");
    assert_eq!(code, OAPI_OK);
    assert!(!c_ex);
    assert_eq!(c_lim, 0.0);
    assert!(!t_ex);
    assert_eq!(t_lim, 0.0);

    // Set tension and compression limits (compression must be <= 0, tension must be >= 0)
    assert_eq!(
        api.cable_obj()
            .set_tension_compression_limits("C1", true, -5.0, true, 200.0),
        OAPI_OK
    );
    let (code, c_ex, c_lim, t_ex, t_lim) = api.cable_obj().get_tc_limits("C1");
    assert_eq!(code, OAPI_OK);
    assert!(c_ex);
    assert_eq!(c_lim, -5.0);
    assert!(t_ex);
    assert_eq!(t_lim, 200.0);

    // Clear limits
    assert_eq!(
        api.cable_obj().set_tc_limits("C1", false, 0.0, false, 0.0),
        OAPI_OK
    );
    let (_, c_ex, _, t_ex, _) = api.cable_obj().get_tc_limits("C1");
    assert!(!c_ex);
    assert!(!t_ex);

    // Invalid limits rejected atomically
    let before = api.inner.clone();
    assert_eq!(
        api.cable_obj().set_tc_limits("C1", true, 10.0, false, 0.0), // positive compression
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj().set_tc_limits("C1", false, 0.0, true, -10.0), // negative tension
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj()
            .set_tc_limits("C1", true, f64::NAN, false, 0.0),
        OAPI_ERR
    );
    assert_eq!(api.inner, before);
}

#[test]
fn test_cable_unstressed_length_and_cable_data() {
    let mut api = setup_base_model();
    // P1=(0,0,0) to P2=(120,0,0) chord = 120.0
    api.cable_obj().add_by_point("P1", "P2", "FSEC1", "C1");

    // Unstressed length defaults to chord
    let (code, l0) = api.cable_obj().get_unstressed_length("C1");
    assert_eq!(code, OAPI_OK);
    assert!((l0 - 120.0).abs() < 1e-9);

    // Set custom unstressed length (slack cable)
    assert_eq!(api.cable_obj().set_unstressed_length("C1", 125.0), OAPI_OK);
    let (_, l0_new) = api.cable_obj().get_unstressed_length("C1");
    assert!((l0_new - 125.0).abs() < 1e-9);

    // Set invalid unstressed length rejected
    assert_eq!(api.cable_obj().set_unstressed_length("C1", -10.0), OAPI_ERR);
    assert_eq!(api.cable_obj().set_unstressed_length("C1", 0.0), OAPI_ERR);
    assert_eq!(
        api.cable_obj().set_unstressed_length("C1", f64::NAN),
        OAPI_ERR
    );
    assert_eq!(api.cable_obj().get_unstressed_length("C1").1, 125.0);

    // Inspect cable data
    let (code, l_opt, ti_opt, h_opt, segs) = api.cable_obj().get_cable_data("C1");
    assert_eq!(code, OAPI_OK);
    assert_eq!(l_opt, Some(125.0));
    assert_eq!(ti_opt, None);
    assert_eq!(h_opt, None);
    assert_eq!(segs, 1);

    // Set target tension I (switches governing parameter and enforces at most 1 active specification)
    assert_eq!(
        api.cable_obj()
            .set_cable_data("C1", None, Some(50.0), None, 5),
        OAPI_OK
    );
    let (_, l_opt, ti_opt, h_opt, segs) = api.cable_obj().get_cable_data("C1");
    assert_eq!(l_opt, None);
    assert_eq!(ti_opt, Some(50.0));
    assert_eq!(h_opt, None);
    assert_eq!(segs, 5);

    // Enforce SDB invariant: rejecting multiple simultaneous active targets
    assert_eq!(
        api.cable_obj()
            .set_cable_data("C1", Some(120.0), Some(50.0), None, 5),
        OAPI_ERR
    );
    // Enforce num_segments >= 1
    assert_eq!(
        api.cable_obj()
            .set_cable_data("C1", Some(120.0), None, None, 0),
        OAPI_ERR
    );
}

#[test]
fn test_cable_modifiers_selection_loads_and_delete() {
    let mut api = setup_base_model();
    api.cable_obj().add_by_point("P1", "P2", "FSEC1", "C1");

    // Selection
    assert!(!api.cable_obj().get_selected("C1").1);
    assert_eq!(api.cable_obj().set_selected("C1", true), OAPI_OK);
    assert!(api.cable_obj().get_selected("C1").1);

    // Modifiers
    let (mod_code, mods) = api.cable_obj().get_modifiers("C1");
    assert_eq!(mod_code, OAPI_OK);
    assert_eq!(mods, [1.0; 8]);

    let new_mods = [1.5, 1.0, 1.0, 0.8, 1.0, 1.0, 1.0, 1.0];
    assert_eq!(api.cable_obj().set_modifiers("C1", new_mods), OAPI_OK);
    assert_eq!(api.cable_obj().get_modifiers("C1").1, new_mods);

    // Negative modifier rejected
    assert_eq!(
        api.cable_obj()
            .set_modifiers("C1", [-1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),
        OAPI_ERR
    );

    // Distributed load
    assert_eq!(
        api.cable_obj()
            .set_load_distributed("C1", "DEAD", 1, 10, 0.0, 1.0, -0.05, -0.05, "GLOBAL", true),
        OAPI_OK
    );

    // Temperature load
    assert_eq!(
        api.cable_obj()
            .set_load_temperature("C1", "DEAD", 1, 35.0, None, true),
        OAPI_OK
    );

    // Deletion
    assert_eq!(api.cable_obj().delete("C1"), OAPI_OK);
    assert_eq!(api.cable_obj().count(), 0);
    assert!(!api
        .cable_obj()
        .get_name_list()
        .1
        .contains(&"C1".to_string()));
    assert_eq!(api.cable_obj().delete("C1"), OAPI_ERR);
}

#[test]
fn test_namespace_partitioning_frame_vs_cable() {
    let mut api = setup_base_model();

    // Create 1 frame and 1 cable
    assert_eq!(
        api.frame_obj()
            .add_by_point("P1", "P2", "FSEC1", "FRAME_1")
            .0,
        OAPI_OK
    );
    assert_eq!(
        api.cable_obj()
            .add_by_point("P3", "P4", "FSEC1", "CABLE_1")
            .0,
        OAPI_OK
    );

    // Frame count and name list must only include FRAME_1
    assert_eq!(api.frame_obj().count(), 1);
    let (_, frame_names) = api.frame_obj().get_name_list();
    assert_eq!(frame_names, vec!["FRAME_1".to_string()]);

    // Cable count and name list must only include CABLE_1
    assert_eq!(api.cable_obj().count(), 1);
    let (_, cable_names) = api.cable_obj().get_name_list();
    assert_eq!(cable_names, vec!["CABLE_1".to_string()]);

    // Symmetry query tests on frame_obj
    let (code, p1, p2) = api.frame_obj().get_points("FRAME_1");
    assert_eq!(code, OAPI_OK);
    assert_eq!(p1, "P1");
    assert_eq!(p2, "P2");
    assert_eq!(api.frame_obj().get_section("FRAME_1").1, "FSEC1");

    // Cross-querying cable through frame_obj or frame through cable_obj is segregated
    assert_eq!(api.cable_obj().get_points("FRAME_1").0, OAPI_ERR);
    assert_eq!(api.frame_obj().get_points("CABLE_1").0, OAPI_ERR);

    // cFrameObj::delete cannot delete a cable
    assert_eq!(api.frame_obj().delete("CABLE_1"), OAPI_ERR);
    assert_eq!(api.cable_obj().count(), 1);

    // cCableObj::delete cannot delete a frame
    assert_eq!(api.cable_obj().delete("FRAME_1"), OAPI_ERR);
    assert_eq!(api.frame_obj().count(), 1);

    // cFrameObj mutators must reject cable elements
    assert_eq!(api.frame_obj().set_section("CABLE_1", "FSEC1"), OAPI_ERR);
    assert_eq!(
        api.frame_obj()
            .set_releases("CABLE_1", [false; 6], [false; 6], [0.0; 6], [0.0; 6]),
        OAPI_ERR
    );
    assert_eq!(
        api.frame_obj()
            .set_load_distributed("CABLE_1", "DEAD", 1, 10, 0.0, 1.0, -1.0, -1.0, "GLOBAL", true),
        OAPI_ERR
    );
    assert_eq!(
        api.frame_obj()
            .set_load_point("CABLE_1", "DEAD", 1, 10, 0.5, -1.0, "GLOBAL", true),
        OAPI_ERR
    );
    assert_eq!(
        api.frame_obj()
            .set_load_temperature("CABLE_1", "DEAD", 1, 20.0, None, true),
        OAPI_ERR
    );
    assert_eq!(
        api.frame_obj()
            .set_end_length_offset("CABLE_1", 1.0, 1.0, 0.5),
        OAPI_ERR
    );
    assert_eq!(api.frame_obj().set_local_axes("CABLE_1", 45.0), OAPI_ERR);
    assert_eq!(api.frame_obj().set_modifiers("CABLE_1", [1.0; 8]), OAPI_ERR);
    assert_eq!(
        api.frame_obj()
            .set_tc_limits("CABLE_1", true, -1.0, true, 1.0),
        OAPI_ERR
    );
    assert_eq!(
        api.frame_obj()
            .set_tension_compression_limits("CABLE_1", true, -1.0, true, 1.0),
        OAPI_ERR
    );

    // cAnalysisResults::cable_force must reject non-cable elements
    assert!(api.results().cable_force("FRAME_1").is_empty());
    assert!(api.results().cable_force("NON_EXISTENT").is_empty());
}

#[test]
fn test_lock_state_and_results_invalidation() {
    let mut api = setup_base_model();
    api.cable_obj().add_by_point("P1", "P2", "FSEC1", "C1");

    // Lock model
    assert_eq!(api.set_model_is_locked(true), OAPI_OK);
    let before = api.inner.clone();

    // All mutating operations must return OAPI_ERR and preserve bitwise equality
    assert_eq!(
        api.cable_obj().add_by_point("P3", "P4", "FSEC1", "C2").0,
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj()
            .add_by_coord(1., 2., 3., 4., 5., 6., "FSEC1", "C3")
            .0,
        OAPI_ERR
    );
    assert_eq!(api.cable_obj().set_points("C1", "P3", "P4"), OAPI_ERR);
    assert_eq!(api.cable_obj().set_section("C1", "FSEC1"), OAPI_ERR);
    assert_eq!(
        api.cable_obj()
            .set_tc_limits("C1", true, -10.0, true, 100.0),
        OAPI_ERR
    );
    assert_eq!(api.cable_obj().set_unstressed_length("C1", 150.0), OAPI_ERR);
    assert_eq!(
        api.cable_obj()
            .set_cable_data("C1", Some(120.0), None, None, 2),
        OAPI_ERR
    );
    assert_eq!(api.cable_obj().delete("C1"), OAPI_ERR);
    assert_eq!(api.inner, before);
}

#[test]
fn test_cable_analysis_and_native_sdb_export() {
    let mut api = setup_base_model();

    // Pin supports at P1 and P2
    api.point_obj()
        .set_restraint("P1", [true, true, true, false, false, false]);
    api.point_obj()
        .set_restraint("P2", [true, true, true, false, false, false]);

    // Create cable with slack unstressed length
    api.cable_obj()
        .add_by_point("P1", "P2", "FSEC1", "MAIN_CABLE");
    api.cable_obj().set_unstressed_length("MAIN_CABLE", 121.0);
    api.cable_obj().set_load_distributed(
        "MAIN_CABLE",
        "DEAD",
        1,
        10,
        0.0,
        1.0,
        -0.01,
        -0.01,
        "GLOBAL",
        true,
    );

    // Disable MODAL case and set DEAD case to NonlinearStatic for catenary cable analysis
    api.analyze().set_run_case_flag("MODAL", false, false);
    if let Some(lc) = api.inner.load_cases.get_mut("DEAD") {
        lc.case_type = sap_model::LoadCaseType::NonlinearStatic;
    }

    assert_eq!(api.analyze().run_analysis(), OAPI_OK);

    // Query cable station tension via cable_force
    let forces = api.results().cable_force("MAIN_CABLE");
    assert!(!forces.is_empty());
    let stations = &forces[0].1;
    assert!(!stations.is_empty());

    // Native SDB export guard verification
    let out_path = std::env::temp_dir().join(format!("cable_export_{}.sdb", std::process::id()));
    assert_eq!(
        api.file().save_native_sdb(out_path.to_str().unwrap()),
        OAPI_OK,
        "Native SDB export failed with: {:?}",
        api.last_file_error
    );

    // Clean up temporary artifact
    if out_path.exists() {
        let _ = std::fs::remove_file(out_path);
    }
}
