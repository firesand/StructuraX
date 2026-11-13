use sap_api::{cSapModel, SapObject, OAPI_ERR, OAPI_OK};
use sap_model::*;
use sap_units::Units;

fn populated_model_with_results() -> cSapModel {
    let mut api = cSapModel::new(Units::KipInF);
    assert_eq!(
        api.point_obj().add_cartesian(0.0, 0.0, 0.0, "P1").0,
        OAPI_OK
    );
    assert_eq!(
        api.point_obj().add_cartesian(120.0, 0.0, 0.0, "P2").0,
        OAPI_OK
    );
    assert_eq!(
        api.point_obj().add_cartesian(120.0, 120.0, 0.0, "P3").0,
        OAPI_OK
    );
    assert_eq!(
        api.point_obj().add_cartesian(0.0, 120.0, 0.0, "P4").0,
        OAPI_OK
    );
    assert_eq!(
        api.frame_obj().add_by_point("P1", "P2", "FSEC1", "F1").0,
        OAPI_OK
    );
    assert_eq!(
        api.area_obj()
            .add_by_point(
                vec!["P1".into(), "P2".into(), "P3".into(), "P4".into()],
                "ASEC1",
                "A1",
            )
            .0,
        OAPI_OK
    );
    assert_eq!(
        api.resp_combo().add("COMB_EXISTING", ComboType::LinearAdd),
        OAPI_OK
    );

    api.inner
        .case_results
        .insert("SOLVED".into(), CaseResults::default());
    api.inner.steel_design_results.insert(
        "F1".into(),
        SteelDesignResult {
            frame_name: "F1".into(),
            section_name: "FSEC1".into(),
            design_code: "AISC 360-10".into(),
            governing_combo: "DEAD".into(),
            governing_station: 0.0,
            total_ratio: 0.42,
            pr_ratio: 0.1,
            mr_major_ratio: 0.32,
            mr_minor_ratio: 0.0,
            shear_major_ratio: 0.05,
            shear_minor_ratio: 0.0,
            phi_pnc: 100.0,
            phi_pnt: 120.0,
            phi_mn33: 500.0,
            phi_mn22: 200.0,
            phi_vn2: 80.0,
            phi_vn3: 80.0,
            status: "OK".into(),
            eq_name: "H1-1b".into(),
        },
    );
    api.inner.concrete_design_results.insert(
        "F1".into(),
        ConcreteDesignResult {
            frame_name: "F1".into(),
            section_name: "FSEC1".into(),
            design_code: "ACI 318-14".into(),
            governing_combo: "DEAD".into(),
            is_column: false,
            as_top: 1.2,
            as_bot: 1.5,
            as_side: 0.0,
            av_over_s: 0.05,
            pmm_ratio: 0.0,
            min_rebar_area: 0.8,
            status: "OK".into(),
        },
    );
    api
}

#[test]
fn legacy_mutators_validate_before_prepare_edit_and_preserve_results_and_lock() {
    let mut api = populated_model_with_results();
    assert_eq!(
        api.point_obj().add_cartesian(60.0, 0.0, 0.0, "PMID").0,
        OAPI_OK
    );
    // Re-seed results after adding collinear helper joint PMID
    api.inner
        .case_results
        .insert("SOLVED".into(), CaseResults::default());
    api.inner.steel_design_results.insert(
        "F1".into(),
        populated_model_with_results().inner.steel_design_results["F1"].clone(),
    );
    api.inner.concrete_design_results.insert(
        "F1".into(),
        populated_model_with_results().inner.concrete_design_results["F1"].clone(),
    );

    let before = api.inner.clone();

    // 1. cAreaObj::add_by_point invalid cases
    for bad_user in [" ", " A2", "A2 ", "A\"2", "A\n2", "A1"] {
        assert_eq!(
            api.area_obj()
                .add_by_point(
                    vec!["P1".into(), "P2".into(), "P3".into()],
                    "ASEC1",
                    bad_user
                )
                .0,
            OAPI_ERR,
            "accepted bad user_name {bad_user:?}"
        );
    }
    assert_eq!(
        api.area_obj()
            .add_by_point(vec!["P1".into(), "P2".into(), "P3".into()], "NO_SEC", "A2")
            .0,
        OAPI_ERR
    );
    assert_eq!(
        api.area_obj()
            .add_by_point(vec!["P1".into(), "P2".into()], "ASEC1", "A2")
            .0,
        OAPI_ERR
    );
    assert_eq!(
        api.area_obj()
            .add_by_point(
                vec![
                    "P1".into(),
                    "P2".into(),
                    "P3".into(),
                    "P4".into(),
                    "PMID".into()
                ],
                "ASEC1",
                "A2"
            )
            .0,
        OAPI_ERR
    );
    assert_eq!(
        api.area_obj()
            .add_by_point(
                vec!["P1".into(), "NO_JOINT".into(), "P3".into()],
                "ASEC1",
                "A2"
            )
            .0,
        OAPI_ERR
    );
    assert_eq!(
        api.area_obj()
            .add_by_point(vec!["P1".into(), "P2".into(), "P1".into()], "ASEC1", "A2")
            .0,
        OAPI_ERR
    );
    // Collinear triangle (P1 - PMID - P2 along X-axis)
    assert_eq!(
        api.area_obj()
            .add_by_point(vec!["P1".into(), "PMID".into(), "P2".into()], "ASEC1", "A2")
            .0,
        OAPI_ERR
    );
    // Bowtie / self-intersecting quad (P1 - P3 - P2 - P4)
    assert_eq!(
        api.area_obj()
            .add_by_point(
                vec!["P1".into(), "P3".into(), "P2".into(), "P4".into()],
                "ASEC1",
                "A2"
            )
            .0,
        OAPI_ERR
    );

    // 2. cAreaObj::set_load_uniform invalid cases
    assert_eq!(
        api.area_obj()
            .set_load_uniform("NO_AREA", "DEAD", -0.01, 6, "GLOBAL", true),
        OAPI_ERR
    );
    assert_eq!(
        api.area_obj()
            .set_load_uniform("A1", "NO_PAT", -0.01, 6, "GLOBAL", true),
        OAPI_ERR
    );
    assert_eq!(
        api.area_obj()
            .set_load_uniform("A1", "DEAD", f64::NAN, 6, "GLOBAL", true),
        OAPI_ERR
    );
    assert_eq!(
        api.area_obj()
            .set_load_uniform("A1", "DEAD", -0.01, 0, "GLOBAL", true),
        OAPI_ERR
    );
    assert_eq!(
        api.area_obj()
            .set_load_uniform("A1", "DEAD", -0.01, 12, "GLOBAL", true),
        OAPI_ERR
    );
    assert_eq!(
        api.area_obj()
            .set_load_uniform("A1", "DEAD", -0.01, 6, "NO_CSYS", true),
        OAPI_ERR
    );

    // 3. cPropFrame invalid cases across all 7 setters
    assert_eq!(
        api.prop_frame()
            .set_rectangle(" BAD ", "A992Fy50", 12.0, 6.0),
        OAPI_ERR
    );
    assert_eq!(
        api.prop_frame().set_rectangle("R1", "NO_MAT", 12.0, 6.0),
        OAPI_ERR
    );
    assert_eq!(
        api.prop_frame().set_rectangle("R1", "A992Fy50", 0.0, 6.0),
        OAPI_ERR
    );
    assert_eq!(
        api.prop_frame().set_rectangle("R1", "A992Fy50", 12.0, -1.0),
        OAPI_ERR
    );
    assert_eq!(
        api.prop_frame()
            .set_i_section("I1", "A992Fy50", 10.0, 8.0, 5.0, 0.5, 8.0, 5.0),
        OAPI_ERR
    );
    assert_eq!(
        api.prop_frame()
            .set_i_section("I1", "A992Fy50", 14.0, 8.0, 0.7, 6.0, 6.0, 0.7),
        OAPI_ERR
    );
    assert_eq!(
        api.prop_frame().set_pipe("PIPE1", "A992Fy50", 10.0, 5.0),
        OAPI_ERR
    );
    assert_eq!(
        api.prop_frame()
            .set_tube("TUBE1", "A992Fy50", 10.0, 8.0, 5.0, 0.5),
        OAPI_ERR
    );
    assert_eq!(
        api.prop_frame()
            .set_tube("TUBE1", "A992Fy50", 10.0, 8.0, 0.5, 4.0),
        OAPI_ERR
    );
    assert_eq!(
        api.prop_frame().set_circle("CIR1", "A992Fy50", 0.0),
        OAPI_ERR
    );
    assert_eq!(
        api.prop_frame().set_general(
            "GEN1", "A992Fy50", 10.0, 10.0, 0.0, 1.0, 1.0, 1.0, 10.0, 10.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0
        ),
        OAPI_ERR
    );
    assert_eq!(
        api.prop_frame().set_general(
            "GEN1", "A992Fy50", 10.0, 10.0, 10.0, -1.0, 1.0, 1.0, 10.0, 10.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0
        ),
        OAPI_ERR
    );
    assert_eq!(
        api.prop_frame()
            .import_prop("IMP1", "A992Fy50", "/nonexistent/sections.pro", "W14X90"),
        OAPI_ERR
    );
    assert_eq!(
        api.prop_frame()
            .import_prop("IMP1", "NO_MAT", "AISC14.pro", "W14X90"),
        OAPI_ERR
    );

    // Also verify missing section inside a valid .pro database rejects before prepare_edit()
    let pro_path = std::env::temp_dir().join(format!("sap-api-synth-{}.pro", std::process::id()));
    {
        let mut bytes = vec![0u8; 240];
        bytes[0x28..0x2c].copy_from_slice(&7.0f32.to_le_bytes()); // version 7.0
        bytes[0x2c..0x30].copy_from_slice(&1.0f32.to_le_bytes()); // 1.0 = inch
        bytes[0x30..0x34].copy_from_slice(&1.0f32.to_le_bytes()); // 1 record
        let rec = &mut bytes[120..240];
        rec[..40].fill(b' ');
        rec[..6].copy_from_slice(b"W14X90");
        rec[36..37].copy_from_slice(b"W");
        let mut put = |k: usize, val: f32| {
            rec[40 + k * 4..44 + k * 4].copy_from_slice(&val.to_le_bytes());
        };
        put(0, 14.0); // t3
        put(1, 14.5); // t2
        put(2, 0.71); // tf
        put(3, 0.44); // tw
        put(5, 26.5); // area
        put(6, 4.06); // J
        put(7, 999.0); // I33
        put(8, 362.0); // I22
        put(9, 6.16); // As2
        put(10, 17.2); // As3
        put(11, 157.0); // Z33
        put(12, 75.6); // Z22
        std::fs::write(&pro_path, bytes).unwrap();
    }
    assert_eq!(
        api.prop_frame().import_prop(
            "IMP1",
            "A992Fy50",
            pro_path.to_str().unwrap(),
            "NO_SUCH_SECTION"
        ),
        OAPI_ERR
    );

    // 4. cPropArea::set_shell invalid cases
    assert_eq!(
        api.prop_area()
            .set_shell(" S1 ", ShellType::ShellThin, "4000Psi", 6.0, 6.0),
        OAPI_ERR
    );
    assert_eq!(
        api.prop_area()
            .set_shell("S1", ShellType::ShellThin, "NO_MAT", 6.0, 6.0),
        OAPI_ERR
    );
    assert_eq!(
        api.prop_area()
            .set_shell("S1", ShellType::ShellThin, "4000Psi", 0.0, 6.0),
        OAPI_ERR
    );
    assert_eq!(
        api.prop_area()
            .set_shell("S1", ShellType::ShellThin, "4000Psi", 6.0, -1.0),
        OAPI_ERR
    );

    // 5. cLoadPatterns::add & set_self_wt_multiplier invalid cases (including MODAL protection)
    assert_eq!(
        api.load_patterns()
            .add(" LP ", LoadPatternType::Live, 0.0, true),
        OAPI_ERR
    );
    assert_eq!(
        api.load_patterns()
            .add("LP", LoadPatternType::Live, f64::INFINITY, true),
        OAPI_ERR
    );
    assert_eq!(
        api.load_patterns()
            .add("COMB_EXISTING", LoadPatternType::Live, 0.0, false),
        OAPI_ERR
    );
    // Must NOT overwrite existing Modal case "MODAL"
    assert_eq!(
        api.load_patterns()
            .add("MODAL", LoadPatternType::Other, 0.0, true),
        OAPI_ERR
    );
    assert_eq!(api.inner.load_cases["MODAL"].case_type, LoadCaseType::Modal);
    assert_eq!(
        api.load_patterns().set_self_wt_multiplier("NO_PAT", 1.0),
        OAPI_ERR
    );
    assert_eq!(
        api.load_patterns().set_self_wt_multiplier("DEAD", f64::NAN),
        OAPI_ERR
    );

    // 6. cRespCombo::add & set_case_list invalid cases (excluding cycles tested next)
    assert_eq!(api.resp_combo().add(" C1 ", ComboType::LinearAdd), OAPI_ERR);
    assert_eq!(api.resp_combo().add("DEAD", ComboType::LinearAdd), OAPI_ERR);
    assert_eq!(
        api.resp_combo().add("MODAL", ComboType::LinearAdd),
        OAPI_ERR
    );
    assert_eq!(
        api.resp_combo().set_case_list("NO_COMBO", "DEAD", 1.0),
        OAPI_ERR
    );
    assert_eq!(
        api.resp_combo()
            .set_case_list("COMB_EXISTING", "NO_CASE", 1.0),
        OAPI_ERR
    );
    assert_eq!(
        api.resp_combo()
            .set_case_list("COMB_EXISTING", "DEAD", f64::NAN),
        OAPI_ERR
    );
    // Self-cycle COMB_EXISTING -> COMB_EXISTING
    assert_eq!(
        api.resp_combo()
            .set_case_list("COMB_EXISTING", "COMB_EXISTING", 1.0),
        OAPI_ERR
    );

    // Every single failed mutator must have preserved the model and all results!
    assert_eq!(api.inner, before);
    assert!(!api.inner.case_results.is_empty());
    assert!(!api.inner.steel_design_results.is_empty());
    assert!(!api.inner.concrete_design_results.is_empty());

    // Now test multi-step combo cycle C1 -> C2 -> C1 preserving newly seeded results
    assert_eq!(api.resp_combo().add("C1", ComboType::LinearAdd), OAPI_OK);
    assert_eq!(api.resp_combo().add("C2", ComboType::LinearAdd), OAPI_OK);
    assert_eq!(api.resp_combo().set_case_list("C1", "C2", 1.0), OAPI_OK);
    api.inner
        .case_results
        .insert("SOLVED".into(), CaseResults::default());
    let before_cycle = api.inner.clone();
    assert_eq!(api.resp_combo().set_case_list("C2", "C1", 1.0), OAPI_ERR);
    assert_eq!(api.inner, before_cycle);

    // Now lock the model and verify ALL legacy mutators reject even valid inputs without unlocking or wiping results!
    api.inner.is_locked = true;
    let locked_before = api.inner.clone();
    assert_eq!(
        api.area_obj()
            .add_by_point(vec!["P1".into(), "P2".into(), "P3".into()], "ASEC1", "A_OK")
            .0,
        OAPI_ERR
    );
    assert_eq!(
        api.area_obj()
            .set_load_uniform("A1", "DEAD", -0.05, 6, "GLOBAL", true),
        OAPI_ERR
    );
    assert_eq!(
        api.prop_frame()
            .set_rectangle("R_OK", "A992Fy50", 12.0, 6.0),
        OAPI_ERR
    );
    assert_eq!(
        api.prop_frame()
            .import_prop("IMP_OK", "A992Fy50", pro_path.to_str().unwrap(), "W14X90"),
        OAPI_ERR
    );
    assert_eq!(
        api.prop_area()
            .set_shell("S_OK", ShellType::ShellThin, "4000Psi", 6.0, 6.0),
        OAPI_ERR
    );
    assert_eq!(
        api.load_patterns()
            .add("LIVE_OK", LoadPatternType::Live, 0.0, true),
        OAPI_ERR
    );
    assert_eq!(
        api.load_patterns().set_self_wt_multiplier("DEAD", 0.5),
        OAPI_ERR
    );
    assert_eq!(api.resp_combo().add("C_OK", ComboType::Envelope), OAPI_ERR);
    assert_eq!(api.resp_combo().set_case_list("C2", "DEAD", 1.2), OAPI_ERR);
    assert_eq!(api.inner, locked_before);
    assert!(api.get_model_is_locked());
    assert!(api.inner.case_results.contains_key("SOLVED"));

    // Unlock and verify valid import_prop succeeds and invalidates results cleanly.
    assert_eq!(api.set_model_is_locked(false), OAPI_OK);
    assert_eq!(
        api.prop_frame()
            .import_prop("IMP_OK", "A992Fy50", pro_path.to_str().unwrap(), "W14X90"),
        OAPI_OK
    );
    assert!(api.inner.frame_sections.contains_key("IMP_OK"));
    assert!(api.inner.case_results.is_empty());
    let _ = std::fs::remove_file(&pro_path);
}

#[test]
fn file_templates_after_open_clear_path_and_preservation_and_validate_inputs() {
    let dir =
        std::env::temp_dir().join(format!("sap-api-template-lifecycle-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let orig_path = dir.join("opened_vendor.s2k");
    let orig_s2k = "TABLE: \"PROGRAM CONTROL\"\n CurrUnits=\"Kip, in, F\"\nTABLE: \"JOINT COORDINATES\"\n Joint=J1 XorR=0 Y=0 Z=0\nTABLE: \"OPAQUE VENDOR\"\n Key=42\nEND TABLE DATA\n";
    std::fs::write(&orig_path, orig_s2k).unwrap();

    let mut app = SapObject::new();
    assert_eq!(
        app.application_start(Units::KipInF, false, orig_path.to_str().unwrap()),
        OAPI_OK
    );
    assert_eq!(app.current_file(), Some(orig_path.as_path()));
    assert!(app.sap_model.inner.s2k_preservation.is_some());

    // Invalid template parameters must reject BEFORE resetting the opened model or path!
    assert_eq!(
        app.sap_model.file().new_beam(0, 120.0, true, "FSEC1"),
        OAPI_ERR
    );
    assert_eq!(
        app.sap_model.file().new_beam(2, -50.0, true, "FSEC1"),
        OAPI_ERR
    );
    assert_eq!(
        app.sap_model.file().new_2d_frame(
            FrameType2D::PortalFrame,
            0,
            144.0,
            2,
            240.0,
            true,
            "FSEC1",
            "FSEC1",
            "FSEC1"
        ),
        OAPI_ERR
    );
    assert_eq!(
        app.sap_model
            .file()
            .new_2d_truss(0, 120.0, 96.0, true, "FSEC1", "FSEC1"),
        OAPI_ERR
    );
    assert_eq!(
        app.sap_model.file().new_3d_frame(
            FrameType3D::OpenFrame,
            1,
            144.0,
            0,
            240.0,
            1,
            240.0,
            true,
            "FSEC1",
            "FSEC1",
            "ASEC1"
        ),
        OAPI_ERR
    );
    assert_eq!(
        app.sap_model
            .file()
            .new_wall(0, 48.0, 2, 48.0, true, "ASEC1"),
        OAPI_ERR
    );
    assert_eq!(
        app.sap_model
            .file()
            .new_spherical_dome(-100.0, 45.0, 8, 4, true, "ASEC1"),
        OAPI_ERR
    );
    assert_eq!(
        app.sap_model
            .file()
            .new_spherical_dome(100.0, 95.0, 8, 4, true, "ASEC1"),
        OAPI_ERR
    );
    assert_eq!(app.current_file(), Some(orig_path.as_path()));
    assert!(app.sap_model.inner.s2k_preservation.is_some());
    assert!(app.sap_model.inner.points.contains_key("J1"));

    // Valid new_beam resets path, s2k_preservation, warnings, and errors
    assert_eq!(
        app.sap_model.file().new_beam(2, 120.0, true, "FSEC1"),
        OAPI_OK
    );
    assert!(app.sap_model.last_file_path.is_none());
    assert!(app.current_file().is_none());
    assert!(app.sap_model.inner.s2k_preservation.is_none());
    assert!(!app.sap_model.inner.points.contains_key("J1"));

    // Implicit save("") and application_exit(true) must NOT overwrite orig_path!
    assert_eq!(app.sap_model.file().save(""), OAPI_ERR);
    assert_eq!(app.application_exit(true), OAPI_ERR);
    assert_eq!(std::fs::read_to_string(&orig_path).unwrap(), orig_s2k);

    // Test new_spherical_dome template end-to-end
    assert_eq!(
        app.sap_model
            .file()
            .new_spherical_dome(240.0, 60.0, 8, 3, true, "ASEC1"),
        OAPI_OK
    );
    assert!(app.sap_model.area_obj().count() >= 24);
    assert_eq!(app.sap_model.analyze().run_analysis(), OAPI_OK);
    let (code, shell_res) = app.sap_model.results().area_force_shell("1");
    assert_eq!(code, OAPI_OK);
    assert_eq!(shell_res.len(), 1);

    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn locked_model_rejects_set_run_case_flag_and_unit_conversion() {
    let mut api = cSapModel::new(Units::KipInF);
    assert_eq!(api.file().new_beam(1, 120.0, true, "FSEC1"), OAPI_OK);
    assert_eq!(api.analyze().run_analysis(), OAPI_OK);
    assert!(api.get_model_is_locked());

    let before = api.inner.clone();
    assert_eq!(
        api.analyze().set_run_case_flag("DEAD", false, false),
        OAPI_ERR
    );
    assert_eq!(api.analyze().set_run_case_flag("", false, true), OAPI_ERR);
    assert_eq!(api.set_present_units(Units::KnMC), OAPI_ERR);
    // Same-unit call is a no-op and allowed
    assert_eq!(api.set_present_units(Units::KipInF), OAPI_OK);
    assert_eq!(api.inner, before);

    // Unlock and verify missing case rejection + result invalidation on mutation
    assert_eq!(api.analyze().delete_results(), OAPI_OK);
    assert_eq!(
        api.analyze()
            .set_run_case_flag("NO_SUCH_CASE", false, false),
        OAPI_ERR
    );
    api.inner
        .case_results
        .insert("STALE".into(), CaseResults::default());
    assert_eq!(
        api.analyze().set_run_case_flag("MODAL", false, false),
        OAPI_OK
    );
    assert!(!api.inner.load_cases["MODAL"].run_flag);
    assert!(api.inner.case_results.is_empty());

    api.inner
        .case_results
        .insert("STALE2".into(), CaseResults::default());
    assert_eq!(api.set_present_units(Units::KnMC), OAPI_OK);
    assert_eq!(api.get_present_units(), Units::KnMC);
    assert!(api.inner.case_results.is_empty());
}

#[test]
fn damper_link_spectrum_directions_and_expanded_results_queries() {
    // Part A: Damper link property validation + nonlinear time-history + link_force / time_history_joint_displ / base_react
    let mut api = cSapModel::new(Units::KipInF);
    api.inner.load_cases.clear();
    api.inner.load_patterns["DEAD"].self_wt_multiplier = 0.0;

    assert_eq!(api.point_obj().add_cartesian(0.0, 0.0, 0.0, "I").0, OAPI_OK);
    assert_eq!(
        api.point_obj().add_cartesian(10.0, 0.0, 0.0, "J").0,
        OAPI_OK
    );
    assert_eq!(api.point_obj().set_restraint("I", [true; 6]), OAPI_OK);
    assert_eq!(
        api.point_obj()
            .set_restraint("J", [false, true, true, true, true, true]),
        OAPI_OK
    );
    assert_eq!(
        api.point_obj()
            .set_mass("J", [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], true),
        OAPI_OK
    );
    assert_eq!(
        api.point_obj()
            .set_spring("J", [50.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        OAPI_OK
    );
    assert_eq!(
        api.point_obj()
            .set_load_force("J", "DEAD", [10.0, 0.0, 0.0, 0.0, 0.0, 0.0], true),
        OAPI_OK
    );

    // Reject untrimmed / quote names and unsupported nonlinear damper exponents
    let mut bad_damper = LinkProp {
        name: " DAMP ".into(),
        link_type: LinkPropType::Damper,
        ..Default::default()
    };
    bad_damper.dof_nonlinear[0] = true;
    bad_damper.k_nl[0] = 200.0;
    bad_damper.param1[0] = 20.0;
    assert_eq!(api.prop_link().set_definition(bad_damper.clone()), OAPI_ERR);
    bad_damper.name = "DAMP".into();
    bad_damper.param2[0] = 0.5;
    assert_eq!(api.prop_link().set_definition(bad_damper.clone()), OAPI_ERR);
    bad_damper.param2[0] = 1.0;
    bad_damper.k_nl[0] = 0.0;
    assert_eq!(api.prop_link().set_definition(bad_damper), OAPI_ERR);

    // Valid linear Maxwell damper (default param2 = 0.0 canonicalized to 1.0 on active nonlinear DOF)
    let mut damper = LinkProp {
        name: "DAMP".into(),
        link_type: LinkPropType::Damper,
        ..Default::default()
    };
    damper.dof_nonlinear[0] = true;
    damper.ke[0] = 100.0;
    damper.k_nl[0] = 200.0;
    damper.param1[0] = 25.0;
    assert_eq!(api.prop_link().set_definition(damper), OAPI_OK);
    assert_eq!(
        api.prop_link().get_definition("DAMP").unwrap().param2[0],
        1.0
    );

    // Reject untrimmed user_name on cLinkObj
    assert_eq!(
        api.link_obj()
            .add_by_point("I", Some("J"), "DAMP", " L1 ")
            .0,
        OAPI_ERR
    );
    assert_eq!(
        api.link_obj().add_by_point("I", Some("J"), "DAMP", "L1").0,
        OAPI_OK
    );

    // Before analysis, result queries return OAPI_ERR
    assert_eq!(api.results().link_force("L1").0, OAPI_ERR);
    assert_eq!(api.results().time_history_joint_displ("J").0, OAPI_ERR);
    assert_eq!(api.results().base_react().0, OAPI_ERR);

    assert_eq!(
        api.functions().set_definition(DynamicFunction {
            name: "STEP".into(),
            is_time_history: true,
            damping_ratio: 0.0,
            points: vec![(0.0, 0.0), (0.05, 1.0), (2.0, 1.0)],
        }),
        OAPI_OK
    );
    let mut nlh = LoadCase::linear_static("NLH", "DEAD", 1.0);
    nlh.case_type = LoadCaseType::NonlinearHistory;
    // Modal history types are integrated directly; they take no modal damping.
    nlh.modal_damping = 0.;
    nlh.loads.clear();
    nlh.history_loads = vec![("STEP".into(), "DEAD".into(), 1.0)];
    nlh.num_time_steps = 40;
    nlh.time_step_dt = 0.01;
    assert_eq!(api.load_cases().set_definition(nlh), OAPI_OK);

    // Also add Modal + ResponseSpectrum using direction alias "UX"
    assert_eq!(api.load_cases().set_modal_eigen("MOD", 1), OAPI_OK);
    assert_eq!(
        api.functions().set_definition(DynamicFunction {
            name: "SPEC_FN".into(),
            is_time_history: false,
            damping_ratio: 0.05,
            points: vec![(0.0, 5.0), (10.0, 5.0)],
        }),
        OAPI_OK
    );
    for alias in ["UX", "X", "1", "UY", "Y", "2", "UZ", "Z", "3"] {
        let mut rs_test = LoadCase::linear_static("RS", "DEAD", 1.0);
        rs_test.loads.clear();
        rs_test.case_type = LoadCaseType::ResponseSpectrum;
        rs_test.initial_case = Some("MOD".into());
        rs_test.spectrum_loads = vec![("SPEC_FN".into(), alias.into(), 1.0)];
        assert_eq!(
            api.load_cases().set_definition(rs_test),
            OAPI_OK,
            "failed spectrum direction alias {alias}"
        );
    }
    let mut rs = LoadCase::linear_static("RS", "DEAD", 1.0);
    rs.loads.clear();
    rs.case_type = LoadCaseType::ResponseSpectrum;
    rs.initial_case = Some("MOD".into());
    rs.spectrum_loads = vec![("SPEC_FN".into(), "UX".into(), 1.0)];
    assert_eq!(api.load_cases().set_definition(rs), OAPI_OK);

    assert_eq!(api.analyze().run_analysis(), OAPI_OK);

    api.results()
        .setup_deselect_all_cases_and_combos_for_output();
    api.results()
        .setup_set_case_selected_for_output("NLH", true);
    api.results()
        .setup_set_case_selected_for_output("MOD", true);
    api.results().setup_set_case_selected_for_output("RS", true);

    let (code, lf) = api.results().link_force("L1");
    assert_eq!(code, OAPI_OK);
    assert!(lf.iter().any(|(c, f)| c == "NLH" && f[0].abs() > 1e-4));
    assert_eq!(api.results().link_force("MISSING").0, OAPI_ERR);

    let (code, th) = api.results().time_history_joint_displ("J");
    assert_eq!(code, OAPI_OK);
    assert_eq!(th.len(), 1);
    assert_eq!(th[0].0, "NLH");
    assert_eq!(th[0].1.len(), 41);
    assert_eq!(th[0].2.len(), 41);
    assert!(th[0].2.last().unwrap()[0] > 0.0);

    let (code, br) = api.results().base_react();
    assert_eq!(code, OAPI_OK);
    // MOD is skipped because eigenvalue modes have no static/dynamic base reaction
    assert!(br.iter().any(|(c, _)| c == "NLH"));
    assert!(br.iter().any(|(c, _)| c == "RS"));
    assert!(!br.iter().any(|(c, _)| c == "MOD"));

    let (code, mmr) = api.results().modal_participating_mass_ratios();
    assert_eq!(code, OAPI_OK);
    assert_eq!(mmr.len(), 1);
    assert_eq!(mmr[0].0, "MOD");
    assert!((mmr[0].1.modal_mass_ratios[0] - 1.0).abs() < 1e-6);

    // Part B: Euler column buckling query via buckling_factor()
    let mut buck_api = cSapModel::new(Units::KipInF);
    buck_api.inner.load_patterns["DEAD"].self_wt_multiplier = 0.0;
    assert_eq!(
        buck_api.point_obj().add_cartesian(0.0, 0.0, 0.0, "B1").0,
        OAPI_OK
    );
    assert_eq!(
        buck_api.point_obj().add_cartesian(0.0, 0.0, 60.0, "B2").0,
        OAPI_OK
    );
    assert_eq!(
        buck_api.point_obj().add_cartesian(0.0, 0.0, 120.0, "B3").0,
        OAPI_OK
    );
    assert_eq!(buck_api.point_obj().set_restraint("B1", [true; 6]), OAPI_OK);
    assert_eq!(
        buck_api
            .point_obj()
            .set_load_force("B3", "DEAD", [0.0, 0.0, -1.0, 0.0, 0.0, 0.0], true),
        OAPI_OK
    );
    assert_eq!(
        buck_api
            .frame_obj()
            .add_by_point("B1", "B2", "FSEC1", "C1")
            .0,
        OAPI_OK
    );
    assert_eq!(
        buck_api
            .frame_obj()
            .add_by_point("B2", "B3", "FSEC1", "C2")
            .0,
        OAPI_OK
    );
    assert_eq!(
        buck_api
            .load_cases()
            .set_buckling("BUCK1", vec![("DEAD".into(), 1.0)], 2),
        OAPI_OK
    );
    assert_eq!(buck_api.analyze().run_analysis(), OAPI_OK);
    buck_api
        .results()
        .setup_set_case_selected_for_output("BUCK1", true);
    let (code, bf) = buck_api.results().buckling_factor();
    assert_eq!(code, OAPI_OK);
    assert!(!bf.is_empty());
    assert_eq!(bf[0].0, "BUCK1");
    assert_eq!(bf[0].1, 1);
    assert!(bf[0].2 > 0.0);
}

#[test]
fn object_management_get_name_list_and_validated_delete() {
    let mut api = cSapModel::new(Units::KipInF);
    assert_eq!(
        api.prop_solid()
            .set_prop("BRICK", "A992Fy50", 0.0, 0.0, 0.0, false),
        OAPI_OK
    );
    let pts: [String; 8] = std::array::from_fn(|i| format!("P{}", i + 1));
    for (i, id) in pts.iter().enumerate() {
        let x = ((i & 1) as f64) * 10.0;
        let y = (((i >> 1) & 1) as f64) * 10.0;
        let z = (((i >> 2) & 1) as f64) * 10.0;
        assert_eq!(api.point_obj().add_cartesian(x, y, z, id).0, OAPI_OK);
    }
    assert_eq!(
        api.frame_obj().add_by_point("P1", "P2", "FSEC1", "F1").0,
        OAPI_OK
    );
    assert_eq!(
        api.area_obj()
            .add_by_point(
                vec!["P1".into(), "P2".into(), "P4".into(), "P3".into()],
                "ASEC1",
                "A1"
            )
            .0,
        OAPI_OK
    );
    assert_eq!(
        api.solid_obj().add_by_point(pts.clone(), "BRICK", "S1").0,
        OAPI_OK
    );
    assert_eq!(api.prop_link().set_definition(LinkProp::default()), OAPI_OK);
    assert_eq!(
        api.link_obj().add_by_point("P1", None, "LIN1", "L1").0,
        OAPI_OK
    );

    assert_eq!(api.point_obj().count(), 8);
    assert_eq!(api.point_obj().get_name_list(), (OAPI_OK, pts.to_vec()));
    assert_eq!(
        api.frame_obj().get_name_list(),
        (OAPI_OK, vec!["F1".to_string()])
    );
    assert_eq!(
        api.area_obj().get_name_list(),
        (OAPI_OK, vec!["A1".to_string()])
    );
    assert_eq!(
        api.solid_obj().get_name_list(),
        (OAPI_OK, vec!["S1".to_string()])
    );
    assert_eq!(
        api.link_obj().get_name_list(),
        (OAPI_OK, vec!["L1".to_string()])
    );

    // Deleting a joint referenced by elements must fail without wiping results
    api.inner
        .case_results
        .insert("KEEP".into(), CaseResults::default());
    assert_eq!(api.point_obj().delete("P1"), OAPI_ERR);
    assert!(api.inner.case_results.contains_key("KEEP"));

    // Locked model rejects all deletes
    api.inner.is_locked = true;
    assert_eq!(api.frame_obj().delete("F1"), OAPI_ERR);
    assert_eq!(api.area_obj().delete("A1"), OAPI_ERR);
    assert_eq!(api.solid_obj().delete("S1"), OAPI_ERR);
    assert_eq!(api.link_obj().delete("L1"), OAPI_ERR);
    api.inner.is_locked = false;

    // Delete referencing objects one by one, checking P1 stays protected until the last reference is gone
    assert_eq!(api.solid_obj().delete("S1"), OAPI_OK);
    assert!(api.inner.case_results.is_empty());
    assert_eq!(api.point_obj().delete("P1"), OAPI_ERR);

    assert_eq!(api.area_obj().delete("A1"), OAPI_OK);
    assert_eq!(api.point_obj().delete("P1"), OAPI_ERR);

    assert_eq!(api.frame_obj().delete("F1"), OAPI_OK);
    assert_eq!(api.point_obj().delete("P1"), OAPI_ERR);

    assert_eq!(api.link_obj().delete("L1"), OAPI_OK);
    // Now P1 is completely unreferenced and can be deleted!
    api.inner
        .case_results
        .insert("KEEP2".into(), CaseResults::default());
    assert_eq!(api.point_obj().delete("P1"), OAPI_OK);
    assert!(api.inner.case_results.is_empty());
    assert_eq!(api.point_obj().count(), 7);
    assert_eq!(api.point_obj().delete("P1"), OAPI_ERR);
}
