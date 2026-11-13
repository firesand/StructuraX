use sap_api::{cSapModel, OAPI_ERR, OAPI_OK};
use sap_format::{extract_sdb_to_model, pack_model_to_native_sdb};
use sap_model::LoadCaseType;
use sap_units::Units;

fn create_test_model() -> cSapModel {
    let mut api = cSapModel::new(Units::KipInF);
    api.point_obj().add_cartesian(0.0, 0.0, 0.0, "P1");
    api.point_obj().add_cartesian(100.0, 0.0, 0.0, "P2");
    api.point_obj().add_cartesian(0.0, 100.0, 0.0, "P3");
    api.point_obj().add_cartesian(100.0, 100.0, 0.0, "P4");
    api
}

#[test]
fn test_stress_target_1_coincident_joints_and_degenerate_geometry() {
    let mut api = create_test_model();

    // 1.1 add_by_point with identical joint names
    let before = api.inner.clone();
    let (code, name) = api
        .cable_obj()
        .add_by_point("P1", "P1", "FSEC1", "BAD_SAME_PT");
    assert_eq!(
        code, OAPI_ERR,
        "add_by_point with identical endpoints must return OAPI_ERR"
    );
    assert!(name.is_empty());
    assert_eq!(
        api.inner, before,
        "State must be unchanged on identical endpoints"
    );

    // 1.2 add_by_point with distinct joint names but coincident coordinates (0,0,0) and (0,0,0)
    api.point_obj().add_cartesian(0.0, 0.0, 0.0, "P_COINCIDENT");
    let (code, name) =
        api.cable_obj()
            .add_by_point("P1", "P_COINCIDENT", "FSEC1", "BAD_COINCIDENT");
    assert_eq!(
        code, OAPI_ERR,
        "add_by_point with coincident coordinates must return OAPI_ERR"
    );
    assert!(name.is_empty());

    // 1.3 add_by_coord with identical endpoints (length = 0.0)
    let pt_count_before = api.inner.points.len();
    let (code, name) = api
        .cable_obj()
        .add_by_coord(50.0, 50.0, 50.0, 50.0, 50.0, 50.0, "FSEC1", "ZERO_LEN");
    assert_eq!(
        code, OAPI_ERR,
        "add_by_coord with identical coordinates must return OAPI_ERR"
    );
    assert!(name.is_empty());
    assert_eq!(
        api.inner.points.len(),
        pt_count_before,
        "No orphan points on zero-length coord creation"
    );

    // 1.4 Ambiguous coincident joint check in add_by_coord
    // We already have P1 and P_COINCIDENT at (0, 0, 0).
    // Attempting add_by_coord starting at (0, 0, 0) must detect ambiguous joint identity and reject.
    let (code, _) =
        api.cable_obj()
            .add_by_coord(0.0, 0.0, 0.0, 200.0, 0.0, 0.0, "FSEC1", "AMBIGUOUS");
    assert_eq!(
        code, OAPI_ERR,
        "add_by_coord with ambiguous coincident joints must return OAPI_ERR"
    );

    // 1.5 Non-finite coordinates in add_by_coord
    let pt_count_before = api.inner.points.len();
    for (xi, yi, zi, xj, yj, zj) in [
        (f64::NAN, 0.0, 0.0, 10.0, 0.0, 0.0),
        (0.0, f64::INFINITY, 0.0, 10.0, 0.0, 0.0),
        (0.0, 0.0, f64::NEG_INFINITY, 10.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, f64::NAN, 0.0, 0.0),
    ] {
        let (code, _) = api
            .cable_obj()
            .add_by_coord(xi, yi, zi, xj, yj, zj, "FSEC1", "NON_FINITE");
        assert_eq!(
            code, OAPI_ERR,
            "add_by_coord with non-finite coords must return OAPI_ERR"
        );
        assert_eq!(
            api.inner.points.len(),
            pt_count_before,
            "No orphan points on non-finite coords"
        );
    }

    // 1.6 Degenerate sub-micron cable geometry (length < 1e-6)
    // Add point with distance 1e-7 from P1
    api.point_obj().add_cartesian(1e-7, 0.0, 0.0, "P_TINY");
    let (code, tiny_cable) = api
        .cable_obj()
        .add_by_point("P1", "P_TINY", "FSEC1", "TINY_CABLE");
    assert_eq!(
        code, OAPI_OK,
        "Sub-micron cable with positive chord is created"
    );
    let (l0_code, l0) = api.cable_obj().get_unstressed_length(&tiny_cable);
    assert_eq!(l0_code, OAPI_OK);
    assert!(
        (l0 - 1e-7).abs() < 1e-12,
        "Unstressed length matches sub-micron chord"
    );

    // Verify SDB export of sub-micron cable succeeds
    let sdb_bytes = pack_model_to_native_sdb(&api.inner);
    assert!(
        sdb_bytes.is_ok(),
        "Native SDB export with sub-micron cable must succeed"
    );

    // 1.7 set_points with coincident or identical joints
    let (code, valid_cable) = api.cable_obj().add_by_point("P1", "P2", "FSEC1", "VALID_C");
    assert_eq!(code, OAPI_OK);
    // Setting identical joints
    assert_eq!(
        api.cable_obj().set_points(&valid_cable, "P1", "P1"),
        OAPI_ERR
    );
    // Setting coincident joints
    assert_eq!(
        api.cable_obj()
            .set_points(&valid_cable, "P1", "P_COINCIDENT"),
        OAPI_ERR
    );
    // Setting nonexistent joint
    assert_eq!(
        api.cable_obj()
            .set_points(&valid_cable, "P1", "NON_EXISTENT"),
        OAPI_ERR
    );
    // Verify valid_cable still points to P1 and P2
    let (_, p1, p2) = api.cable_obj().get_points(&valid_cable);
    assert_eq!(p1, "P1");
    assert_eq!(p2, "P2");
}

#[test]
fn test_stress_target_2_conflicting_and_extreme_targets() {
    let mut api = create_test_model();
    let (code, name) = api
        .cable_obj()
        .add_by_point("P1", "P2", "FSEC1", "C_TARGETS");
    assert_eq!(code, OAPI_OK);

    // 2.1 Negative and zero unstressed length
    assert_eq!(
        api.cable_obj().set_unstressed_length(&name, -10.0),
        OAPI_ERR
    );
    assert_eq!(api.cable_obj().set_unstressed_length(&name, -0.0), OAPI_ERR);
    assert_eq!(api.cable_obj().set_unstressed_length(&name, 0.0), OAPI_ERR);
    assert_eq!(
        api.cable_obj().set_unstressed_length(&name, f64::NAN),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj().set_unstressed_length(&name, f64::INFINITY),
        OAPI_ERR
    );

    // Verify length remains initial chord (100.0)
    let (_, l0) = api.cable_obj().get_unstressed_length(&name);
    assert!((l0 - 100.0).abs() < 1e-9);

    // 2.2 Zero segments rejection in set_cable_data
    assert_eq!(
        api.cable_obj()
            .set_cable_data(&name, Some(110.0), None, None, 0),
        OAPI_ERR
    );

    // 2.3 Segment count overflow (> i32::MAX)
    assert_eq!(
        api.cable_obj()
            .set_cable_data(&name, Some(110.0), None, None, usize::MAX),
        OAPI_ERR
    );
    #[cfg(target_pointer_width = "64")]
    assert_eq!(
        api.cable_obj()
            .set_cable_data(&name, Some(110.0), None, None, (i32::MAX as usize) + 1),
        OAPI_ERR
    );

    // 2.4 Conflicting targets in set_cable_data:
    // undeformed_length + target_tension_i
    assert_eq!(
        api.cable_obj()
            .set_cable_data(&name, Some(110.0), Some(50.0), None, 2),
        OAPI_ERR,
        "Simultaneous undeformed_length and target_tension_i must reject"
    );
    // undeformed_length + horizontal_tension
    assert_eq!(
        api.cable_obj()
            .set_cable_data(&name, Some(110.0), None, Some(40.0), 2),
        OAPI_ERR,
        "Simultaneous undeformed_length and horizontal_tension must reject"
    );
    // target_tension_i + horizontal_tension
    assert_eq!(
        api.cable_obj()
            .set_cable_data(&name, None, Some(50.0), Some(40.0), 2),
        OAPI_ERR,
        "Simultaneous target_tension_i and horizontal_tension must reject"
    );
    // all three
    assert_eq!(
        api.cable_obj()
            .set_cable_data(&name, Some(110.0), Some(50.0), Some(40.0), 2),
        OAPI_ERR,
        "All three targets must reject"
    );

    // 2.5 Negative or zero tension values in set_cable_data
    assert_eq!(
        api.cable_obj()
            .set_cable_data(&name, None, Some(-5.0), None, 2),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj()
            .set_cable_data(&name, None, Some(0.0), None, 2),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj()
            .set_cable_data(&name, None, None, Some(-5.0), 2),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj()
            .set_cable_data(&name, None, None, Some(0.0), 2),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj()
            .set_cable_data(&name, None, Some(f64::NAN), None, 2),
        OAPI_ERR
    );

    // 2.6 Mutual exclusivity & target override:
    // Transition from undeformed length -> target tension
    assert_eq!(
        api.cable_obj()
            .set_cable_data(&name, None, Some(75.0), None, 4),
        OAPI_OK
    );
    let (_, l_opt, ti_opt, h_opt, segs) = api.cable_obj().get_cable_data(&name);
    assert_eq!(l_opt, None, "Undeformed length must be cleared");
    assert_eq!(ti_opt, Some(75.0));
    assert_eq!(h_opt, None);
    assert_eq!(segs, 4);

    // Verify SDB export with target_tension_i works cleanly
    let sdb_bytes = pack_model_to_native_sdb(&api.inner);
    assert!(
        sdb_bytes.is_ok(),
        "Native SDB export with target_tension_i must succeed"
    );

    // Transition from target tension -> horizontal tension
    assert_eq!(
        api.cable_obj()
            .set_cable_data(&name, None, None, Some(60.0), 3),
        OAPI_OK
    );
    let (_, l_opt, ti_opt, h_opt, segs) = api.cable_obj().get_cable_data(&name);
    assert_eq!(l_opt, None);
    assert_eq!(ti_opt, None, "Target tension must be cleared");
    assert_eq!(h_opt, Some(60.0));
    assert_eq!(segs, 3);

    // Verify SDB export with horizontal_tension works cleanly
    let sdb_bytes = pack_model_to_native_sdb(&api.inner);
    assert!(
        sdb_bytes.is_ok(),
        "Native SDB export with horizontal_tension must succeed"
    );

    // Transition from horizontal tension -> set_unstressed_length
    assert_eq!(api.cable_obj().set_unstressed_length(&name, 105.0), OAPI_OK);
    let (_, l_opt, ti_opt, h_opt, _) = api.cable_obj().get_cable_data(&name);
    assert_eq!(l_opt, Some(105.0));
    assert_eq!(
        ti_opt, None,
        "Tension must be cleared when set_unstressed_length is called"
    );
    assert_eq!(
        h_opt, None,
        "Horiz tension must be cleared when set_unstressed_length is called"
    );

    // 2.7 All targets None (num_segments >= 1)
    assert_eq!(
        api.cable_obj().set_cable_data(&name, None, None, None, 5),
        OAPI_OK
    );
    let (_, l_opt, ti_opt, h_opt, segs) = api.cable_obj().get_cable_data(&name);
    assert_eq!(l_opt, None);
    assert_eq!(ti_opt, None);
    assert_eq!(h_opt, None);
    assert_eq!(segs, 5);
    // get_unstressed_length falls back to chord length
    let (code, chord_l) = api.cable_obj().get_unstressed_length(&name);
    assert_eq!(code, OAPI_OK);
    assert!((chord_l - 100.0).abs() < 1e-9);

    let sdb_bytes = pack_model_to_native_sdb(&api.inner);
    assert!(
        sdb_bytes.is_ok(),
        "Native SDB export with all None targets must succeed"
    );
}

#[test]
fn test_stress_target_3_point_reassignment_deletion_and_lifecycle() {
    let mut api = create_test_model();
    let (code, name) = api
        .cable_obj()
        .add_by_point("P1", "P2", "FSEC1", "LIFECYCLE_C");
    assert_eq!(code, OAPI_OK);

    // 3.1 Rapid point reassignment in a cyclic polygon
    let path = [
        ("P2", "P3"),
        ("P3", "P4"),
        ("P4", "P1"),
        ("P1", "P3"),
        ("P2", "P4"),
    ];
    for (start, end) in path {
        assert_eq!(api.cable_obj().set_points(&name, start, end), OAPI_OK);
        let (_, p1, p2) = api.cable_obj().get_points(&name);
        assert_eq!(p1, start);
        assert_eq!(p2, end);
    }

    // 3.2 Interleaved failed reassignments do not alter current connectivity
    let (_, curr_p1, curr_p2) = api.cable_obj().get_points(&name);
    assert_eq!(api.cable_obj().set_points(&name, "P1", "P1"), OAPI_ERR);
    assert_eq!(
        api.cable_obj().set_points(&name, "MISSING_A", "MISSING_B"),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj().set_points(&name, "P1", "MISSING_B"),
        OAPI_ERR
    );
    let (_, p1, p2) = api.cable_obj().get_points(&name);
    assert_eq!(p1, curr_p1);
    assert_eq!(p2, curr_p2);

    // 3.3 Atomicity of add_by_coord: failed coordinate insertion leaves NO orphan points
    let points_snapshot = api.inner.points.clone();
    // Failed due to invalid section
    let (code, _) = api.cable_obj().add_by_coord(
        123.4,
        567.8,
        910.1,
        223.4,
        667.8,
        910.1,
        "NONEXISTENT_SEC",
        "FAILED_C1",
    );
    assert_eq!(code, OAPI_ERR);
    assert_eq!(
        api.inner.points, points_snapshot,
        "No orphan points left after invalid section failure"
    );

    // Failed due to invalid user name
    let (code, _) = api.cable_obj().add_by_coord(
        333.0,
        444.0,
        555.0,
        666.0,
        777.0,
        888.0,
        "FSEC1",
        "BAD\"NAME",
    );
    assert_eq!(code, OAPI_ERR);
    assert_eq!(
        api.inner.points, points_snapshot,
        "No orphan points left after invalid name failure"
    );

    // Failed due to duplicate user name
    let (code, _) = api
        .cable_obj()
        .add_by_coord(333.0, 444.0, 555.0, 666.0, 777.0, 888.0, "FSEC1", &name);
    assert_eq!(code, OAPI_ERR);
    assert_eq!(
        api.inner.points, points_snapshot,
        "No orphan points left after duplicate name failure"
    );

    // 3.4 Deletion and post-deletion access
    assert_eq!(api.cable_obj().delete(&name), OAPI_OK);
    assert_eq!(api.cable_obj().count(), 0);

    // Double delete
    assert_eq!(api.cable_obj().delete(&name), OAPI_ERR);

    // Every single query and mutator must return OAPI_ERR on deleted cable
    assert_eq!(api.cable_obj().get_points(&name).0, OAPI_ERR);
    assert_eq!(api.cable_obj().set_points(&name, "P1", "P2"), OAPI_ERR);
    assert_eq!(api.cable_obj().get_section(&name).0, OAPI_ERR);
    assert_eq!(api.cable_obj().set_section(&name, "FSEC1"), OAPI_ERR);
    assert_eq!(
        api.cable_obj().get_tension_compression_limits(&name).0,
        OAPI_ERR
    );
    assert_eq!(api.cable_obj().get_tc_limits(&name).0, OAPI_ERR);
    assert_eq!(
        api.cable_obj()
            .set_tension_compression_limits(&name, true, -1.0, true, 1.0),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj().set_tc_limits(&name, true, -1.0, true, 1.0),
        OAPI_ERR
    );
    assert_eq!(api.cable_obj().get_unstressed_length(&name).0, OAPI_ERR);
    assert_eq!(
        api.cable_obj().set_unstressed_length(&name, 100.0),
        OAPI_ERR
    );
    assert_eq!(api.cable_obj().get_cable_data(&name).0, OAPI_ERR);
    assert_eq!(
        api.cable_obj()
            .set_cable_data(&name, Some(100.0), None, None, 1),
        OAPI_ERR
    );
    assert_eq!(api.cable_obj().get_modifiers(&name).0, OAPI_ERR);
    assert_eq!(api.cable_obj().set_modifiers(&name, [1.0; 8]), OAPI_ERR);
    assert_eq!(api.cable_obj().get_selected(&name).0, OAPI_ERR);
    assert_eq!(api.cable_obj().set_selected(&name, true), OAPI_ERR);
    assert_eq!(
        api.cable_obj()
            .set_load_distributed(&name, "DEAD", 1, 10, 0.0, 1.0, -1.0, -1.0, "GLOBAL", true),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj()
            .set_load_temperature(&name, "DEAD", 1, 20.0, None, true),
        OAPI_ERR
    );
}

#[test]
fn test_stress_target_4_concurrent_frame_cable_operations_and_namespace_isolation() {
    let mut api = create_test_model();

    // 4.1 Strict name collisions across frames and cables
    assert_eq!(
        api.frame_obj()
            .add_by_point("P1", "P2", "FSEC1", "MEMBER_SHARED")
            .0,
        OAPI_OK
    );
    // Attempt to add cable with same name
    assert_eq!(
        api.cable_obj()
            .add_by_point("P3", "P4", "FSEC1", "MEMBER_SHARED")
            .0,
        OAPI_ERR,
        "Cable creation with existing frame name must reject"
    );

    assert_eq!(
        api.cable_obj()
            .add_by_point("P3", "P4", "FSEC1", "CABLE_UNIQUE")
            .0,
        OAPI_OK
    );
    // Attempt to add frame with existing cable name
    assert_eq!(
        api.frame_obj()
            .add_by_point("P1", "P3", "FSEC1", "CABLE_UNIQUE")
            .0,
        OAPI_ERR,
        "Frame creation with existing cable name must reject"
    );

    // 4.2 Interleaved auto-generated names
    // Next numeric names should not collide and must be partitioned
    let (code_f, f_auto) = api.frame_obj().add_by_point("P2", "P4", "FSEC1", "");
    assert_eq!(code_f, OAPI_OK);
    let (code_c, c_auto) = api.cable_obj().add_by_point("P1", "P4", "FSEC1", "");
    assert_eq!(code_c, OAPI_OK);
    assert_ne!(
        f_auto, c_auto,
        "Frame and cable auto-generated names must be distinct"
    );

    // Total counts
    assert_eq!(api.frame_obj().count(), 2); // MEMBER_SHARED and f_auto
    assert_eq!(api.cable_obj().count(), 2); // CABLE_UNIQUE and c_auto

    let (_, frame_names) = api.frame_obj().get_name_list();
    let (_, cable_names) = api.cable_obj().get_name_list();
    assert_eq!(frame_names.len(), 2);
    assert_eq!(cable_names.len(), 2);
    assert!(frame_names.contains(&"MEMBER_SHARED".to_string()));
    assert!(frame_names.contains(&f_auto));
    assert!(!frame_names.contains(&"CABLE_UNIQUE".to_string()));
    assert!(!frame_names.contains(&c_auto));

    assert!(cable_names.contains(&"CABLE_UNIQUE".to_string()));
    assert!(cable_names.contains(&c_auto));
    assert!(!cable_names.contains(&"MEMBER_SHARED".to_string()));
    assert!(!cable_names.contains(&f_auto));

    // 4.3 Query segregation
    // Querying cable through frame_obj
    assert_eq!(api.frame_obj().get_points("CABLE_UNIQUE").0, OAPI_ERR);
    assert_eq!(api.frame_obj().get_section("CABLE_UNIQUE").0, OAPI_ERR);
    assert_eq!(api.frame_obj().get_tc_limits("CABLE_UNIQUE").0, OAPI_ERR);

    // Querying frame through cable_obj
    assert_eq!(api.cable_obj().get_points("MEMBER_SHARED").0, OAPI_ERR);
    assert_eq!(api.cable_obj().get_section("MEMBER_SHARED").0, OAPI_ERR);
    assert_eq!(api.cable_obj().get_tc_limits("MEMBER_SHARED").0, OAPI_ERR);
    assert_eq!(
        api.cable_obj().get_unstressed_length("MEMBER_SHARED").0,
        OAPI_ERR
    );
    assert_eq!(api.cable_obj().get_cable_data("MEMBER_SHARED").0, OAPI_ERR);
    assert_eq!(api.cable_obj().get_modifiers("MEMBER_SHARED").0, OAPI_ERR);
    assert_eq!(api.cable_obj().get_selected("MEMBER_SHARED").0, OAPI_ERR);

    // 4.4 Deletion segregation
    assert_eq!(api.frame_obj().delete("CABLE_UNIQUE"), OAPI_ERR);
    assert_eq!(api.cable_obj().count(), 2);

    assert_eq!(api.cable_obj().delete("MEMBER_SHARED"), OAPI_ERR);
    assert_eq!(api.frame_obj().count(), 2);

    // 4.5 Mutation segregation on cable_obj targeting frames
    assert_eq!(
        api.cable_obj().set_points("MEMBER_SHARED", "P1", "P4"),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj().set_section("MEMBER_SHARED", "FSEC1"),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj()
            .set_tc_limits("MEMBER_SHARED", true, -10.0, true, 100.0),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj()
            .set_unstressed_length("MEMBER_SHARED", 110.0),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj()
            .set_cable_data("MEMBER_SHARED", Some(110.0), None, None, 1),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj().set_modifiers("MEMBER_SHARED", [1.0; 8]),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj().set_selected("MEMBER_SHARED", true),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj().set_load_distributed(
            "MEMBER_SHARED",
            "DEAD",
            1,
            10,
            0.0,
            1.0,
            -1.0,
            -1.0,
            "GLOBAL",
            true
        ),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj()
            .set_load_temperature("MEMBER_SHARED", "DEAD", 1, 20.0, None, true),
        OAPI_ERR
    );

    // 4.6 Investigation of frame_obj setters invoked on a cable
    // Notice: cFrameObj::get_section, get_points, get_tc_limits and delete reject cables.
    // Frame setters route through edit_frame which enforces cable_params.is_none().
    let frame_set_sec = api.frame_obj().set_section("CABLE_UNIQUE", "FSEC1");
    // With cable isolation guard in place, frame setters reject cables with OAPI_ERR:
    assert_eq!(frame_set_sec, OAPI_ERR);
    assert_eq!(api.cable_obj().get_section("CABLE_UNIQUE").1, "FSEC1");
}

#[test]
fn test_stress_target_5_multi_cable_catenary_network_and_native_sdb_roundtrip() {
    let mut api = cSapModel::new(Units::KipInF);

    // Define a 3D catenary cable network: 4 elevated anchor supports and 4 internal sagging nodes
    // Anchors (restrained)
    api.point_obj().add_cartesian(0.0, 0.0, 40.0, "A1");
    api.point_obj().add_cartesian(100.0, 0.0, 40.0, "A2");
    api.point_obj().add_cartesian(0.0, 100.0, 40.0, "A3");
    api.point_obj().add_cartesian(100.0, 100.0, 40.0, "A4");
    for a in ["A1", "A2", "A3", "A4"] {
        api.point_obj().set_restraint(a, [true; 6]);
    }

    // Internal nodes
    api.point_obj().add_cartesian(30.0, 30.0, 20.0, "J1");
    api.point_obj().add_cartesian(70.0, 30.0, 20.0, "J2");
    api.point_obj().add_cartesian(30.0, 70.0, 20.0, "J3");
    api.point_obj().add_cartesian(70.0, 70.0, 20.0, "J4");

    // Sections
    api.prop_frame()
        .set_i_section("CSEC_MAIN", "A992Fy50", 12.0, 8.0, 0.5, 0.5, 8.0, 0.5);
    api.prop_frame()
        .set_i_section("CSEC_TIE", "A992Fy50", 8.0, 6.0, 0.4, 0.4, 6.0, 0.4);

    // 8 distinct cables connecting the network with heterogeneous parameters:
    // C1: A1 - J1 (undeformed length taut)
    api.cable_obj().add_by_point("A1", "J1", "CSEC_MAIN", "C1");
    api.cable_obj()
        .set_cable_data("C1", Some(46.0), None, None, 4);
    api.cable_obj().set_tc_limits("C1", true, 0.0, false, 0.0);

    // C2: A2 - J2 (target tension)
    api.cable_obj().add_by_point("A2", "J2", "CSEC_MAIN", "C2");
    api.cable_obj()
        .set_cable_data("C2", None, Some(80.0), None, 6);

    // C3: A3 - J3 (horizontal tension)
    api.cable_obj().add_by_point("A3", "J3", "CSEC_MAIN", "C3");
    api.cable_obj()
        .set_cable_data("C3", None, None, Some(60.0), 4);

    // C4: A4 - J4 (undeformed length taut)
    api.cable_obj().add_by_point("A4", "J4", "CSEC_MAIN", "C4");
    api.cable_obj().set_unstressed_length("C4", 46.0);

    // C5: J1 - J2 (tie with distributed gravity load)
    api.cable_obj().add_by_point("J1", "J2", "CSEC_TIE", "C5");
    api.cable_obj()
        .set_load_distributed("C5", "DEAD", 1, 10, 0.0, 1.0, -0.05, -0.05, "GLOBAL", true);
    api.cable_obj()
        .set_modifiers("C5", [0.8, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]);

    // C6: J3 - J4 (tie with temperature load)
    api.cable_obj().add_by_point("J3", "J4", "CSEC_TIE", "C6");
    api.cable_obj()
        .set_load_temperature("C6", "DEAD", 1, 30.0, None, true);

    // C7: J1 - J3 (cross tie taut)
    api.cable_obj().add_by_point("J1", "J3", "CSEC_TIE", "C7");
    api.cable_obj()
        .set_cable_data("C7", Some(39.5), None, None, 3);

    // C8: J2 - J4 (cross tie taut)
    api.cable_obj().add_by_point("J2", "J4", "CSEC_TIE", "C8");
    api.cable_obj()
        .set_cable_data("C8", Some(39.5), None, None, 3);

    assert_eq!(api.cable_obj().count(), 8);

    // Configure NonlinearStatic analysis for catenary equilibrium
    api.analyze().set_run_case_flag("MODAL", false, false);
    if let Some(lc) = api.inner.load_cases.get_mut("DEAD") {
        lc.case_type = LoadCaseType::NonlinearStatic;
        lc.max_iterations = 400;
        lc.convergence_tol = 1e-4;
    }
    let solve_res = sap_solver::analyze_model(&mut api.inner);
    println!("analyze_model error: {:?}", solve_res);
    assert!(
        solve_res.is_ok(),
        "Nonlinear catenary analysis on multi-cable network must solve: {:?}",
        solve_res
    );

    // Verify forces queryable for cables
    for c_name in ["C1", "C2", "C3", "C4", "C5", "C6", "C7", "C8"] {
        let forces = api.results().cable_force(c_name);
        assert!(
            !forces.is_empty(),
            "Forces must be computed for cable {c_name}"
        );
    }

    // Native SDB export roundtrip
    let packed =
        pack_model_to_native_sdb(&api.inner).expect("Native SDB serialization must succeed");
    let recovered = extract_sdb_to_model(&packed).expect("Native SDB deserialization must succeed");

    // Verify all 8 cables and their properties roundtrip without data loss
    assert_eq!(recovered.frames.len(), 8);
    for c_name in ["C1", "C2", "C3", "C4", "C5", "C6", "C7", "C8"] {
        let orig = &api.inner.frames[c_name];
        let restored = &recovered.frames[c_name];

        assert_eq!(
            orig.point_i, restored.point_i,
            "Cable {c_name} point_i mismatch"
        );
        assert_eq!(
            orig.point_j, restored.point_j,
            "Cable {c_name} point_j mismatch"
        );
        assert_eq!(
            orig.section, restored.section,
            "Cable {c_name} section mismatch"
        );
        assert_eq!(
            orig.cable_params, restored.cable_params,
            "Cable {c_name} cable_params mismatch"
        );
        assert_eq!(
            orig.tc_limits, restored.tc_limits,
            "Cable {c_name} tc_limits mismatch"
        );
        assert_eq!(
            orig.modifiers, restored.modifiers,
            "Cable {c_name} modifiers mismatch"
        );
        assert_eq!(
            orig.loads_dist, restored.loads_dist,
            "Cable {c_name} loads_dist mismatch"
        );
        assert_eq!(
            orig.loads_temp, restored.loads_temp,
            "Cable {c_name} loads_temp mismatch"
        );
    }

    // Export through cSapModel file API to disk and verify roundtrip
    let temp_sdb = std::env::temp_dir().join(format!("catenary_net_{}.sdb", std::process::id()));
    let save_res = api.file().save_native_sdb(temp_sdb.to_str().unwrap());
    assert_eq!(
        save_res, OAPI_OK,
        "save_native_sdb file API must succeed: {:?}",
        api.last_file_error
    );

    // Read back via file open API
    let mut reloaded_api = cSapModel::new(Units::KipInF);
    let open_res = reloaded_api.file().open_file(temp_sdb.to_str().unwrap());
    assert_eq!(
        open_res, OAPI_OK,
        "open_file on exported native SDB must succeed"
    );
    assert_eq!(reloaded_api.cable_obj().count(), 8);

    if temp_sdb.exists() {
        let _ = std::fs::remove_file(temp_sdb);
    }
}

#[test]
fn test_stress_target_6_locked_model_mutations() {
    let mut api = create_test_model();
    api.cable_obj()
        .add_by_point("P1", "P2", "FSEC1", "C_LOCKED");

    // Lock model
    assert_eq!(api.set_model_is_locked(true), OAPI_OK);
    assert!(api.get_model_is_locked());
    let snapshot = api.inner.clone();

    // 6.1 Geometric mutators
    assert_eq!(
        api.cable_obj().add_by_point("P3", "P4", "FSEC1", "NEW_C").0,
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj()
            .add_by_coord(10.0, 20.0, 30.0, 40.0, 50.0, 60.0, "FSEC1", "NEW_COORD")
            .0,
        OAPI_ERR
    );
    assert_eq!(api.cable_obj().set_points("C_LOCKED", "P3", "P4"), OAPI_ERR);

    // 6.2 Property and limit mutators
    assert_eq!(api.cable_obj().set_section("C_LOCKED", "FSEC1"), OAPI_ERR);
    assert_eq!(
        api.cable_obj()
            .set_tension_compression_limits("C_LOCKED", true, -1.0, true, 1.0),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj()
            .set_tc_limits("C_LOCKED", true, -1.0, true, 1.0),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj().set_unstressed_length("C_LOCKED", 115.0),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj()
            .set_cable_data("C_LOCKED", Some(115.0), None, None, 2),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj().set_modifiers("C_LOCKED", [1.5; 8]),
        OAPI_ERR
    );

    // 6.3 Loading mutators
    assert_eq!(
        api.cable_obj()
            .set_load_distributed("C_LOCKED", "DEAD", 1, 10, 0.0, 1.0, -1.0, -1.0, "GLOBAL", true),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj()
            .set_load_temperature("C_LOCKED", "DEAD", 1, 25.0, None, true),
        OAPI_ERR
    );

    // 6.4 Deletion
    assert_eq!(api.cable_obj().delete("C_LOCKED"), OAPI_ERR);

    // Strict model state invariance: every rejected mutation leaves model completely unchanged
    assert_eq!(
        api.inner, snapshot,
        "Locked model state must remain completely unaltered across all rejected mutations"
    );

    // 6.5 Query operations still function correctly while locked
    assert_eq!(api.cable_obj().count(), 1);
    let (code, p1, p2) = api.cable_obj().get_points("C_LOCKED");
    assert_eq!(code, OAPI_OK);
    assert_eq!(p1, "P1");
    assert_eq!(p2, "P2");

    let (code, sec) = api.cable_obj().get_section("C_LOCKED");
    assert_eq!(code, OAPI_OK);
    assert_eq!(sec, "FSEC1");

    let (code, l0) = api.cable_obj().get_unstressed_length("C_LOCKED");
    assert_eq!(code, OAPI_OK);
    assert!((l0 - 100.0).abs() < 1e-9);

    // 6.6 Selection is permitted while locked (view state)
    assert_eq!(api.cable_obj().set_selected("C_LOCKED", true), OAPI_OK);
    assert!(api.cable_obj().get_selected("C_LOCKED").1);
}

#[test]
fn test_cable_loads_modifiers_and_results_invalidation() {
    let mut api = create_test_model();
    api.cable_obj().add_by_point("P1", "P2", "FSEC1", "C_LOADS");

    // Distributed loads edge cases
    // Invalid load patterns
    assert_eq!(
        api.cable_obj().set_load_distributed(
            "C_LOADS",
            "NONEXISTENT",
            1,
            10,
            0.0,
            1.0,
            -1.0,
            -1.0,
            "GLOBAL",
            true
        ),
        OAPI_ERR
    );
    // Invalid my_type (0 or 3)
    assert_eq!(
        api.cable_obj()
            .set_load_distributed("C_LOADS", "DEAD", 0, 10, 0.0, 1.0, -1.0, -1.0, "GLOBAL", true),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj()
            .set_load_distributed("C_LOADS", "DEAD", 3, 10, 0.0, 1.0, -1.0, -1.0, "GLOBAL", true),
        OAPI_ERR
    );
    // Invalid direction (< 1 or > 11)
    assert_eq!(
        api.cable_obj()
            .set_load_distributed("C_LOADS", "DEAD", 1, 0, 0.0, 1.0, -1.0, -1.0, "GLOBAL", true),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj()
            .set_load_distributed("C_LOADS", "DEAD", 1, 12, 0.0, 1.0, -1.0, -1.0, "GLOBAL", true),
        OAPI_ERR
    );
    // Inverted or out of range distances
    assert_eq!(
        api.cable_obj()
            .set_load_distributed("C_LOADS", "DEAD", 1, 10, 0.8, 0.2, -1.0, -1.0, "GLOBAL", true),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj()
            .set_load_distributed("C_LOADS", "DEAD", 1, 10, -0.1, 1.0, -1.0, -1.0, "GLOBAL", true),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj()
            .set_load_distributed("C_LOADS", "DEAD", 1, 10, 0.0, 1.1, -1.0, -1.0, "GLOBAL", true),
        OAPI_ERR
    );
    // Non-finite load values
    assert_eq!(
        api.cable_obj().set_load_distributed(
            "C_LOADS",
            "DEAD",
            1,
            10,
            0.0,
            1.0,
            f64::NAN,
            -1.0,
            "GLOBAL",
            true
        ),
        OAPI_ERR
    );
    // Invalid csys
    assert_eq!(
        api.cable_obj().set_load_distributed(
            "C_LOADS",
            "DEAD",
            1,
            10,
            0.0,
            1.0,
            -1.0,
            -1.0,
            "NONEXISTENT_CSYS",
            true
        ),
        OAPI_ERR
    );

    // Temperature loads edge cases
    assert_eq!(
        api.cable_obj()
            .set_load_temperature("C_LOADS", "NONEXISTENT", 1, 30.0, None, true),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj()
            .set_load_temperature("C_LOADS", "DEAD", 2, 30.0, None, true),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj()
            .set_load_temperature("C_LOADS", "DEAD", 1, f64::NAN, None, true),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj()
            .set_load_temperature("C_LOADS", "DEAD", 1, 30.0, Some("PATTERN1"), true),
        OAPI_ERR,
        "Cables do not support joint pattern gradients in temperature loads"
    );

    // Modifiers validation
    assert_eq!(
        api.cable_obj()
            .set_modifiers("C_LOADS", [-0.5, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj()
            .set_modifiers("C_LOADS", [f64::NAN, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),
        OAPI_ERR
    );
    assert_eq!(
        api.cable_obj().set_modifiers(
            "C_LOADS",
            [f64::INFINITY, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        ),
        OAPI_ERR
    );

    // Results invalidation verification
    // 1. Setup simple solvable cable
    api.point_obj().set_restraint("P1", [true; 6]);
    api.point_obj().set_restraint("P2", [true; 6]);
    api.analyze().set_run_case_flag("MODAL", false, false);
    if let Some(lc) = api.inner.load_cases.get_mut("DEAD") {
        lc.case_type = LoadCaseType::NonlinearStatic;
    }
    assert_eq!(api.analyze().run_analysis(), OAPI_OK);
    assert!(
        !api.inner.case_results.is_empty(),
        "Results must exist after analysis"
    );
    assert!(api.get_model_is_locked(), "Analysis locks the model");

    // set_selected must succeed while locked and NOT invalidate results
    assert_eq!(api.cable_obj().set_selected("C_LOADS", true), OAPI_OK);
    assert!(
        !api.inner.case_results.is_empty(),
        "Selection must preserve analysis results"
    );

    // All mutators must reject while locked
    assert_eq!(api.cable_obj().set_modifiers("C_LOADS", [0.9; 8]), OAPI_ERR);
    assert_eq!(
        api.cable_obj().set_unstressed_length("C_LOADS", 102.0),
        OAPI_ERR
    );

    // Unlocking clears results
    assert_eq!(api.set_model_is_locked(false), OAPI_OK);
    assert!(
        api.inner.case_results.is_empty(),
        "Unlocking clears analysis results"
    );

    // When unlocked, manually inject a dummy case result to test invalidate_results() on cable mutators
    api.inner
        .case_results
        .insert("DUMMY".to_string(), sap_model::CaseResults::default());
    assert!(!api.inner.case_results.is_empty());
    assert_eq!(api.cable_obj().set_modifiers("C_LOADS", [0.9; 8]), OAPI_OK);
    assert!(
        api.inner.case_results.is_empty(),
        "set_modifiers must call invalidate_results()"
    );

    api.inner
        .case_results
        .insert("DUMMY".to_string(), sap_model::CaseResults::default());
    assert_eq!(
        api.cable_obj().set_unstressed_length("C_LOADS", 102.0),
        OAPI_OK
    );
    assert!(
        api.inner.case_results.is_empty(),
        "set_unstressed_length must call invalidate_results()"
    );

    api.inner
        .case_results
        .insert("DUMMY".to_string(), sap_model::CaseResults::default());
    assert_eq!(api.cable_obj().set_section("C_LOADS", "FSEC1"), OAPI_OK);
    assert!(
        api.inner.case_results.is_empty(),
        "set_section must call invalidate_results()"
    );

    api.inner
        .case_results
        .insert("DUMMY".to_string(), sap_model::CaseResults::default());
    assert_eq!(
        api.cable_obj()
            .set_tc_limits("C_LOADS", false, 0.0, false, 0.0),
        OAPI_OK
    );
    assert!(
        api.inner.case_results.is_empty(),
        "set_tc_limits must call invalidate_results()"
    );

    api.inner
        .case_results
        .insert("DUMMY".to_string(), sap_model::CaseResults::default());
    assert_eq!(
        api.cable_obj()
            .set_cable_data("C_LOADS", Some(101.0), None, None, 1),
        OAPI_OK
    );
    assert!(
        api.inner.case_results.is_empty(),
        "set_cable_data must call invalidate_results()"
    );
}
