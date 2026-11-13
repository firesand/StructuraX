#![cfg(feature = "vendor-fixtures")]

use std::path::{Path, PathBuf};

use sap_format::{
    extract_sdb_to_model, inspect_native_sdb, load_sdb_file, pack_model_to_native_sdb,
};
use sap_model::LoadCaseType;
use sap_solver::analyze_model;

fn vendor_dir() -> PathBuf {
    std::env::var_os("STRUCTURAX_VENDOR_FIXTURES")
        .map(PathBuf::from)
        .unwrap_or_else(|| Path::new(env!("CARGO_MANIFEST_DIR")).join("../../extracted/files"))
}

#[test]
fn vendor_sdb_example_5_001_solid_patch_test_comp_and_incomp() {
    let dir = vendor_dir();
    for fname in ["example_5_001_incomp.sdb", "example_5_001_comp.sdb"] {
        let path = dir.join(fname);
        assert!(path.exists(), "missing vendor file {fname}");
        let mut model = load_sdb_file(&path).expect(fname);
        analyze_model(&mut model).expect(fname);

        let res = model
            .case_results
            .get("PATCH")
            .or_else(|| model.case_results.values().next())
            .expect("PATCH results");
        let u8 = res.joint_displacements["8"];
        assert!((u8[0] - 0.00089).abs() < 1e-4, "{fname} ux(8) = {}", u8[0]);
        assert!((u8[1] - 0.00118).abs() < 1e-4, "{fname} uy(8) = {}", u8[1]);
        assert!((u8[2] - 0.00116).abs() < 1e-4, "{fname} uz(8) = {}", u8[2]);

        for (sname, sr) in &res.solid_results {
            let c = &sr.center_stress;
            assert!(
                (c.s11 - 2000.0).abs() < 1e-2,
                "{fname} solid {sname} s11 = {}",
                c.s11
            );
            assert!(
                (c.s22 - 2000.0).abs() < 1e-2,
                "{fname} solid {sname} s22 = {}",
                c.s22
            );
            assert!(
                (c.s33 - 2000.0).abs() < 1e-2,
                "{fname} solid {sname} s33 = {}",
                c.s33
            );
            assert!(
                (c.s12 - 400.0).abs() < 1e-2,
                "{fname} solid {sname} s12 = {}",
                c.s12
            );
            assert!(
                (c.s13 - 400.0).abs() < 1e-2,
                "{fname} solid {sname} s13 = {}",
                c.s13
            );
            assert!(
                (c.s23 - 400.0).abs() < 1e-2,
                "{fname} solid {sname} s23 = {}",
                c.s23
            );
        }
    }
}

#[test]
fn vendor_sdb_example_5_002_and_5_005_solid_beams() {
    let dir = vendor_dir();
    let p_502 = dir.join("example_5_002a1_incomp.sdb");
    let mut m_502 = load_sdb_file(&p_502).expect("load example_5_002a1_incomp.sdb");
    analyze_model(&mut m_502).expect("solve example_5_002a1_incomp.sdb");
    assert!(!m_502.case_results.is_empty());
    for res in m_502.case_results.values() {
        assert!(!res.solid_results.is_empty());
    }

    let p_505 = dir.join("example_5_005a.sdb");
    let mut m_505 = load_sdb_file(&p_505).expect("load example_5_005a.sdb");
    analyze_model(&mut m_505).expect("solve example_5_005a.sdb");
    assert!(!m_505.case_results.is_empty());
}

#[test]
fn vendor_sdb_example_6_003_gap_link_static_and_direct_history() {
    use sap_model::{
        DynamicFunction, FrameLoadPoint, FrameSection, LinkObj, LinkProp, LinkPropType, LoadCase,
        LoadPattern, LoadPatternType, Material, PointLoadForce, SapModel,
    };
    use sap_units::Units;

    let dir = vendor_dir();
    let path = dir.join("example_6_003a.sdb");
    let raw = sap_format::read_sdb_file(&path).expect("read example_6_003a.sdb");
    let doc = inspect_native_sdb(&raw).expect("inspect example_6_003a.sdb");
    assert_eq!(doc.inventory().parsed_bytes, doc.inventory().stream_bytes);

    // Verify both Model A (ke = 0) and Model B (ke = 200,000) of SAP2000 Verification Example 6-003
    for ke_val in [0.0, 200_000.0] {
        let mut model = SapModel::new(Units::KipInF);
        model.active_dofs = [true, false, true, false, true, false];
        model.load_patterns.clear();
        model.load_cases.clear();

        let mut mat = Material::default_steel_kip_in();
        mat.name = "STEEL".to_string();
        mat.e = 29_900.0;
        mat.u = 0.3;
        mat.g = 11_500.0;
        mat.unit_weight = 0.0;
        mat.unit_mass = 0.0;
        model.materials.insert("STEEL".to_string(), mat);

        let mut sec = FrameSection::default();
        sec.name = "FSEC1".to_string();
        sec.material = "STEEL".to_string();
        sec.area = 10.0;
        sec.i33 = 100.0;
        sec.as2 = 2.0;
        model.frame_sections.insert("FSEC1".to_string(), sec);

        model.add_point("1", 0.0, 0.0, 0.0);
        model.points.get_mut("1").unwrap().restraint = [true; 6];
        model.add_point("2", 144.0, 0.0, 0.0);
        model.points.get_mut("2").unwrap().mass = [0.0, 0.0, 0.001, 0.0, 0.0, 0.0];
        model.add_point("3", 0.0, 0.0, 144.0);
        model.points.get_mut("3").unwrap().mass = [0.3, 0.0, 0.1, 0.0, 0.0, 0.0];
        model.add_point("4", 144.0, 0.0, 144.0);
        model.points.get_mut("4").unwrap().mass = [0.3, 0.0, 0.1, 0.0, 0.0, 0.0];

        model.add_frame("1", "1", "3", "FSEC1");
        model.add_frame("2", "2", "4", "FSEC1");
        model.add_frame("3", "3", "4", "FSEC1");

        let mut gap = LinkProp {
            name: "GAP1".to_string(),
            link_type: LinkPropType::Gap,
            ..LinkProp::default()
        };
        gap.dof_active[0] = true;
        gap.dof_nonlinear[0] = true;
        gap.nonlinear_data_complete = true;
        gap.ke[0] = ke_val;
        gap.k_nl[0] = 200_000.0;
        gap.param1[0] = 0.0;
        model.link_props.insert("GAP1".to_string(), gap);
        model.links.insert(
            "1".to_string(),
            LinkObj {
                name: "1".to_string(),
                point_i: "2".to_string(),
                point_j: None,
                prop: "GAP1".to_string(),
                angle_deg: 0.0,
                selected: false,
            },
        );

        for pat in ["P", "V"] {
            model.load_patterns.insert(
                pat.to_string(),
                LoadPattern {
                    name: pat.to_string(),
                    pat_type: LoadPatternType::Other,
                    self_wt_multiplier: 0.0,
                },
            );
        }
        model
            .frames
            .get_mut("3")
            .unwrap()
            .loads_point
            .push(FrameLoadPoint {
                load_pat: "P".to_string(),
                my_type: 1,
                dir: 6,
                csys: "GLOBAL".to_string(),
                rel_dist: 0.5,
                val: -10.0,
            });
        model
            .points
            .get_mut("4")
            .unwrap()
            .loads_force
            .push(PointLoadForce {
                load_pat: "V".to_string(),
                csys: "GLOBAL".to_string(),
                force: [-20.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            });

        model.functions.insert(
            "RAMP20".to_string(),
            DynamicFunction {
                name: "RAMP20".to_string(),
                is_time_history: true,
                damping_ratio: 0.0,
                points: vec![(0.0, 0.0), (20.0, 1.0), (100.0, 1.0)],
            },
        );
        model.functions.insert(
            "RAMP40".to_string(),
            DynamicFunction {
                name: "RAMP40".to_string(),
                is_time_history: true,
                damping_ratio: 0.0,
                points: vec![(0.0, 0.0), (40.0, 1.0), (100.0, 1.0)],
            },
        );

        let mut nlstat1 = LoadCase::linear_static("NLSTAT1", "P", 1.0);
        nlstat1.case_type = LoadCaseType::NonlinearStatic;
        model.load_cases.insert("NLSTAT1".to_string(), nlstat1);

        let mut nlstat2 = LoadCase::linear_static("NLSTAT2", "V", 1.0);
        nlstat2.case_type = LoadCaseType::NonlinearStatic;
        nlstat2.initial_case = Some("NLSTAT1".to_string());
        model.load_cases.insert("NLSTAT2".to_string(), nlstat2);

        let w1 = 2.0 * std::f64::consts::PI / 1.0;
        let w2 = 2.0 * std::f64::consts::PI / 3.4;
        let zeta = 0.999;
        let am = 2.0 * zeta * w1 * w2 / (w1 + w2);
        let bk = 2.0 * zeta / (w1 + w2);

        let mut nldhist1 = LoadCase::linear_static("NLDHIST1", "P", 1.0);
        // The vendor case is a direct-integration history (NLDHIST).
        nldhist1.case_type = LoadCaseType::NonlinearDynamic;
        nldhist1.loads.clear();
        nldhist1.history_loads = vec![("RAMP20".to_string(), "P".to_string(), 1.0)];
        nldhist1.num_time_steps = 400;
        nldhist1.time_step_dt = 0.1;
        nldhist1.rayleigh_damping = (am, bk);
        nldhist1.max_iterations = 100;
        nldhist1.convergence_tol = 1e-6;
        model
            .load_cases
            .insert("NLDHIST1".to_string(), nldhist1.clone());

        let mut nlstat3 = LoadCase::linear_static("NLSTAT3", "V", 1.0);
        nlstat3.case_type = LoadCaseType::NonlinearStatic;
        nlstat3.initial_case = Some("NLDHIST1".to_string());
        model.load_cases.insert("NLSTAT3".to_string(), nlstat3);

        let mut nldhist2 = nldhist1.clone();
        nldhist2.name = "NLDHIST2".to_string();
        nldhist2.initial_case = Some("NLDHIST1".to_string());
        nldhist2.history_loads = vec![("RAMP40".to_string(), "V".to_string(), 1.0)];
        nldhist2.num_time_steps = 800;
        model
            .load_cases
            .insert("NLDHIST2".to_string(), nldhist2.clone());

        let mut nldhist3 = nldhist2;
        nldhist3.name = "NLDHIST3".to_string();
        nldhist3.initial_case = Some("NLSTAT1".to_string());
        model.load_cases.insert("NLDHIST3".to_string(), nldhist3);

        // Modal continuation verification
        {
            let mut modal_target = model.clone();
            let mut nlmhist2 = LoadCase::linear_static("NLMHIST2", "V", 1.0);
            nlmhist2.case_type = LoadCaseType::LinearHistory;
            // Modal history types are integrated directly; they take no modal damping.
            nlmhist2.modal_damping = 0.;
            nlmhist2.loads.clear();
            nlmhist2.initial_case = Some("NLDHIST1".to_string());
            nlmhist2.history_loads = vec![("RAMP40".to_string(), "V".to_string(), 1.0)];
            nlmhist2.num_time_steps = 800;
            nlmhist2.time_step_dt = 0.1;
            modal_target
                .load_cases
                .insert("NLMHIST2".to_string(), nlmhist2);
            let err = analyze_model(&mut modal_target).unwrap_err();
            assert!(err.to_string().contains("NLMHIST2"), "{err}");
            assert!(modal_target.case_results.is_empty());

            let mut modal_origin = model.clone();
            let mut nlmhist1 = LoadCase::linear_static("NLMHIST1", "P", 1.0);
            nlmhist1.case_type = LoadCaseType::LinearHistory;
            // Modal history types are integrated directly; they take no modal damping.
            nlmhist1.modal_damping = 0.;
            nlmhist1.loads.clear();
            nlmhist1.history_loads = vec![("RAMP20".to_string(), "P".to_string(), 1.0)];
            nlmhist1.num_time_steps = 400;
            nlmhist1.time_step_dt = 0.1;
            let mut cont = LoadCase::linear_static("NLMHIST2", "V", 1.0);
            cont.case_type = LoadCaseType::NonlinearHistory;
            // Modal history types are integrated directly; they take no modal damping.
            cont.modal_damping = 0.;
            cont.loads.clear();
            cont.initial_case = Some("NLMHIST1".to_string());
            cont.history_loads = vec![("RAMP40".to_string(), "V".to_string(), 1.0)];
            cont.num_time_steps = 800;
            cont.time_step_dt = 0.1;
            modal_origin
                .load_cases
                .insert("NLMHIST1".to_string(), nlmhist1);
            modal_origin.load_cases.insert("NLMHIST2".to_string(), cont);
            analyze_model(&mut modal_origin)
                .expect("modal_origin solves with NLMHIST1 continuation");
            assert!(modal_origin.case_results["NLMHIST1"]
                .transient_state
                .is_some());
            assert!(modal_origin.case_results["NLMHIST2"]
                .transient_state
                .is_some());
        }

        // Direct-integration history continuation (`NLSTAT3` and `NLDHIST2` from `NLDHIST1`)
        // now succeeds using persisted TransientTerminalState!
        analyze_model(&mut model).expect("solve all Example 6-003 static & direct-history cases");

        assert!(model.case_results["NLDHIST1"].transient_state.is_some());
        assert!(model.case_results["NLDHIST2"].transient_state.is_some());

        let f_nlstat1 = model.case_results["NLSTAT1"].link_forces["1"][0];
        assert!(
            (f_nlstat1 - (-4.53378)).abs() < 0.01,
            "NLSTAT1 gap force = {f_nlstat1}"
        );
        let f_nldhist1 = model.case_results["NLDHIST1"].link_forces["1"][0];
        assert!(
            (f_nldhist1 - (-4.53378)).abs() < 0.02,
            "NLDHIST1 gap force = {f_nldhist1}"
        );

        for cname in ["NLSTAT2", "NLSTAT3"] {
            let uz = model.case_results[cname].joint_displacements["2"][2];
            assert!(
                (uz - 3.9169).abs() < 0.01,
                "{cname} Joint 2 uplift Uz = {uz}"
            );
        }
        for cname in ["NLDHIST2", "NLDHIST3"] {
            let uz_end = model.case_results[cname].time_history_disp["2"]
                .last()
                .unwrap()[2];
            assert!(
                (uz_end - 3.9169).abs() < 0.01,
                "{cname} final Joint 2 uplift Uz = {uz_end}"
            );
        }
    }
}

/// Example 6-008 stores the plastic Wen parameters and the two displacement-
/// controlled pushes. Its loading is a link gravity load and its link axes are
/// an advanced assignment; neither is modeled. Import records both, analysis
/// names them as the reason it refuses, and native synthesis refuses to write a
/// file without them. The published values are checked on an independent model
/// in `plastic_wen_regressions.rs`.
#[test]
fn vendor_sdb_example_6_008_wen_parameters_and_displacement_control() {
    let path = vendor_dir().join("example_6_008.sdb");
    assert!(path.exists(), "missing vendor file example_6_008.sdb");
    let mut model = load_sdb_file(&path).expect("example_6_008");
    let prop = &model.link_props["PLAW1"];
    assert_eq!(prop.link_type, sap_model::LinkPropType::PlasticWen);
    assert!(prop.nonlinear_data_complete);
    assert_eq!(prop.dof_nonlinear, [false, true, false, false, false, false]);
    // U2: k = 100 kip/in, yield = 50 kip, ratio = 0.1, exponent = 1.
    assert_eq!(
        (prop.k_nl[1], prop.param1[1], prop.param2[1], prop.param3[1]),
        (100.0, 50.0, 0.1, 1.0)
    );
    for (case, target) in [("NLSTAT1", 10.0), ("NLSTAT2", 20.0)] {
        let control = model.load_cases[case].displacement_control.as_ref().unwrap();
        assert_eq!(control.target, target);
        assert_eq!(control.monitored, Some(("1".to_string(), 2)));
    }
    assert_eq!(
        model.load_cases["NLSTAT2"].initial_case.as_deref(),
        Some("NLSTAT1")
    );
    use sap_model::UnmodeledScope;
    let unmodeled: Vec<(&UnmodeledScope, &str)> = model
        .unmodeled_source
        .iter()
        .map(|entry| (&entry.scope, entry.description.as_str()))
        .collect();
    assert_eq!(
        unmodeled,
        [
            (
                &UnmodeledScope::AllCases,
                "advanced local axes of link '1' (only the roll angle is imported)"
            ),
            (
                &UnmodeledScope::LoadPattern("1".into()),
                "gravity load on link '1' in load pattern '1'"
            ),
        ]
    );
    let error = analyze_model(&mut model).unwrap_err().to_string();
    assert!(
        error.contains("advanced local axes of link '1'")
            && error.contains("gravity load on link '1'"),
        "{error}"
    );
    assert!(model.case_results.is_empty());
    let export = pack_model_to_native_sdb(&model).unwrap_err().to_string();
    assert!(export.contains("unmodeled_source"), "{export}");

    // Discarding the entries is an explicit decision. What is left is the
    // imported model without its load: the push then has nothing to scale.
    model.unmodeled_source.clear();
    let error = analyze_model(&mut model).unwrap_err().to_string();
    assert!(
        error.contains("does not move the monitored displacement"),
        "{error}"
    );
    // The represented data survives native synthesis.
    let recovered = extract_sdb_to_model(&pack_model_to_native_sdb(&model).unwrap()).unwrap();
    assert_eq!(recovered.link_props["PLAW1"], model.link_props["PLAW1"]);
    assert_eq!(recovered.load_cases["NLSTAT1"], model.load_cases["NLSTAT1"]);
    assert_eq!(recovered.load_cases["NLSTAT2"], model.load_cases["NLSTAT2"]);
}

/// Imported data that is recorded as not modeled refuses exactly the cases it
/// affects. Example 1-026 assigns two frame hinges: its nonlinear static
/// pushover is refused, and a linear case of the same model still runs.
/// Example 6-007 has a modal history with per-mode damping overrides and a
/// direct-integration history: only the modal one is refused.
#[test]
fn vendor_sdb_unmodeled_hinges_and_modal_damping_refuse_only_affected_cases() {
    use sap_model::{LoadCase, UnmodeledScope};
    let dir = vendor_dir();

    for fname in ["example_1_026a.sdb", "example_1_026b.sdb"] {
        let mut model = load_sdb_file(dir.join(fname)).expect(fname);
        assert_eq!(model.unmodeled_source.len(), 1, "{fname}");
        assert_eq!(model.unmodeled_source[0].scope, UnmodeledScope::NonlinearCases);
        assert_eq!(
            model.unmodeled_source[0].description,
            "2 frame hinge assignment(s)"
        );
        let error = analyze_model(&mut model).unwrap_err().to_string();
        assert!(error.contains("frame hinge assignment"), "{fname}: {error}");
        assert!(model.case_results.is_empty());

        // Hinges do not act in a linear case that starts from zero.
        let pattern = model.load_patterns.keys().next().unwrap().clone();
        for case in model.load_cases.values_mut() {
            case.run_flag = false;
        }
        model
            .load_cases
            .insert("LIN".into(), LoadCase::linear_static("LIN", pattern, 1.0));
        analyze_model(&mut model).expect(fname);
        assert!(model.case_results.contains_key("LIN"));
    }

    let mut model = load_sdb_file(dir.join("example_6_007.sdb")).expect("example_6_007");
    assert_eq!(
        model.unmodeled_source[0].scope,
        UnmodeledScope::Case("NLMHIST1".into())
    );
    assert_eq!(model.load_cases["NLMHIST1"].modal_damping, 0.0102);
    let error = analyze_model(&mut model).unwrap_err().to_string();
    assert!(
        error.contains("NLMHIST1") && error.contains("modal damping"),
        "{error}"
    );
    // Removing the override entry still leaves the constant modal damping
    // ratio, which a history case cannot honor either.
    model.unmodeled_source.clear();
    let error = analyze_model(&mut model).unwrap_err().to_string();
    assert!(
        error.contains("modal damping ratio 0.0102"),
        "{error}"
    );
    assert!(model.case_results.is_empty());

    // Mass/stiffness-proportional modal damping is Rayleigh damping, which the
    // direct integration applies: Example 6-011a stores ratio 0.0059 at 1 and
    // 10 Hz and the matching coefficients.
    let model = load_sdb_file(dir.join("example_6_011a.sdb")).expect("example_6_011a");
    let proportional: Vec<_> = model
        .load_cases
        .values()
        .filter(|c| c.case_type == LoadCaseType::NonlinearHistory && c.rayleigh_damping.0 != 0.0)
        .collect();
    assert_eq!(proportional.len(), 1);
    assert_eq!(proportional[0].modal_damping, 0.0);
    let (w1, w2) = (2.0 * std::f64::consts::PI, 20.0 * std::f64::consts::PI);
    let (a, b) = proportional[0].rayleigh_damping;
    assert!((a - 2.0 * 0.0059 * w1 * w2 / (w1 + w2)).abs() < 1e-12);
    assert!((b - 2.0 * 0.0059 / (w1 + w2)).abs() < 1e-15);
}

#[test]
fn vendor_sdb_example_6_004_hook_link_thermal_contraction() {
    let dir = vendor_dir();
    let path = dir.join("example_6_004.sdb");
    let mut model = load_sdb_file(&path).expect("load example_6_004.sdb");
    analyze_model(&mut model).expect("solve example_6_004.sdb");

    let res = model
        .case_results
        .get("NLSTAT1")
        .or_else(|| model.case_results.values().next())
        .expect("Example 6-004 results");
    let uz_1 = res.joint_displacements["1"][2];
    let f_link = res.link_forces["1"][0];
    let p_frame = res.frame_forces["1"][0].p;

    assert!((uz_1 - 0.0491772).abs() < 1e-4, "Uz(1) = {uz_1}");
    assert!((f_link - 45.886).abs() < 0.05, "Link force = {f_link}");
    assert!(
        (p_frame - 45.886).abs() < 0.05,
        "Frame axial force = {p_frame}"
    );
}

#[test]
fn vendor_sdb_example_6_005_maxwell_damper_harmonic_history() {
    let dir = vendor_dir();
    let path = dir.join("example_6_005a.sdb");
    let mut model = load_sdb_file(&path).expect("load example_6_005a.sdb");
    model.load_cases.retain(|_, lc| {
        matches!(
            lc.case_type,
            LoadCaseType::NonlinearHistory | LoadCaseType::NonlinearDynamic
        )
    });
    analyze_model(&mut model).expect("solve example_6_005a.sdb");

    let res = &model.case_results["NLDHIST1"];
    let trace = &res.time_history_disp["1"];
    // dt = 0.01 s: t = 0.50 s -> step 50, t = 5.00 s -> step 500, t = 11.00 s -> step 1100
    let uz_050 = trace[50][2];
    let uz_500 = trace[500][2];
    let uz_1100 = trace[1100][2];
    let uz_max = trace.iter().map(|d| d[2].abs()).fold(0.0_f64, f64::max);

    assert!((uz_050 - (-0.10483)).abs() < 2e-3, "Uz(0.50s) = {uz_050}");
    assert!((uz_500 - (-0.88833)).abs() < 5e-3, "Uz(5.00s) = {uz_500}");
    assert!((uz_1100 - 0.99453).abs() < 5e-3, "Uz(11.00s) = {uz_1100}");
    assert!(
        (uz_max - 1.0000).abs() < 1e-2,
        "Steady-state max Uz = {uz_max}"
    );
}

#[test]
fn vendor_sdb_examples_7_001_7_002_7_003_cables() {
    let dir = vendor_dir();

    // Example 7-001: 6 catenary cables C1..C6 (Peyrot & Goulois 1979)
    // C1..C6 correspond to cases DEADTEMP, DISP1, DISP2, DISP3, DISP4, DISP5 at Lower Support Joint 2!
    let mut m_701 = load_sdb_file(dir.join("example_7_001.sdb")).expect("load example_7_001.sdb");
    analyze_model(&mut m_701).expect("solve example_7_001.sdb");
    for (cname, exp_rx, exp_rz, tol) in [
        ("DEADTEMP", 4_258_491.0, -2_555_045.0, 50.0),
        ("DISP1", 504.10, -328.87, 0.05),
        ("DISP2", 22.15, 15.73, 0.02),
        ("DISP3", 9.17, 19.24, 0.02),
        ("DISP4", 3.06, 19.93, 0.02),
        ("DISP5", 0.00, 20.02, 0.02),
    ] {
        let r2 = m_701.case_results[cname].joint_reactions["2"];
        assert!(
            (r2[0] - exp_rx).abs() <= tol,
            "7-001 {cname} Rx(2) = {}, expected {exp_rx}",
            r2[0]
        );
        assert!(
            (r2[2] - exp_rz).abs() <= tol,
            "7-001 {cname} Rz(2) = {}, expected {exp_rz}",
            r2[2]
        );
    }

    // Example 7-002: Two-segment cable under SELF + POINT + COMB1 (Tibert 1999)
    let mut m_702 = load_sdb_file(dir.join("example_7_002.sdb")).expect("load example_7_002.sdb");
    analyze_model(&mut m_702).expect("solve example_7_002.sdb");
    let u2_comb1 = m_702.case_results["COMB1"].joint_displacements["2"];
    assert!(
        (u2_comb1[0] - (-2.8194)).abs() < 1e-3,
        "7-002 COMB1 Ux(2) = {}",
        u2_comb1[0]
    );
    assert!(
        (u2_comb1[2] - (-18.4567)).abs() < 1e-3,
        "7-002 COMB1 Uz(2) = {}",
        u2_comb1[2]
    );

    // Example 7-003: Prestressed 3x3 Cable Net initialized via target_tension_i (Tibert 1999)
    let mut m_703 = load_sdb_file(dir.join("example_7_003.sdb")).expect("load example_7_003.sdb");
    assert_eq!(m_703.frames.len(), 12);
    for f in m_703.frames.values() {
        let cp = f.cable_params.as_ref().expect("7-003 cable_params");
        assert!(cp.target_tension_i.is_some());
        assert!(cp.undeformed_length.is_none());
    }

    // Competing shape specs remain explicitly rejected
    let mut m_703_bad = m_703.clone();
    m_703_bad
        .frames
        .get_mut("1")
        .unwrap()
        .cable_params
        .as_mut()
        .unwrap()
        .undeformed_length = Some(100.0);
    let err_dup = analyze_model(&mut m_703_bad).unwrap_err();
    assert!(
        err_dup.to_string().contains("target end tension"),
        "{err_dup}"
    );
    assert!(m_703_bad.case_results.is_empty());

    analyze_model(&mut m_703).expect("solve example_7_003.sdb");
    let pt_res = &m_703.case_results["POINT"].joint_displacements;
    // SAP2000 Verification Example 7-003 reference:
    // Node 1: Ux = -0.1328 ft, Uy = -0.1328 ft, Uz = -1.4745 ft (symmetric across Nodes 1..4)
    for (jname, sx, sy) in [
        ("1", -1.0, -1.0),
        ("2", 1.0, -1.0),
        ("3", -1.0, 1.0),
        ("4", 1.0, 1.0),
    ] {
        let u = pt_res[jname];
        assert!(
            (u[0] - sx * 0.1328).abs() < 2e-4,
            "7-003 POINT Ux({jname}) = {}",
            u[0]
        );
        assert!(
            (u[1] - sy * 0.1328).abs() < 2e-4,
            "7-003 POINT Uy({jname}) = {}",
            u[1]
        );
        assert!(
            (u[2] - (-1.4745)).abs() < 2e-4,
            "7-003 POINT Uz({jname}) = {}",
            u[2]
        );
    }
}

#[test]
fn native_sdb_export_reimport_preserves_analysis_results() {
    let dir = vendor_dir();
    let mut orig = load_sdb_file(dir.join("example_6_004.sdb")).expect("load 6-004");
    let native_sdb = pack_model_to_native_sdb(&orig).expect("pack native sdb");
    let doc = inspect_native_sdb(&native_sdb).expect("inspect native sdb");
    assert_eq!(doc.inventory().parsed_bytes, doc.inventory().stream_bytes);

    let mut reimported = extract_sdb_to_model(&native_sdb).expect("extract native sdb");
    analyze_model(&mut orig).expect("solve orig");
    analyze_model(&mut reimported).expect("solve reimported");

    let uz_orig = orig.case_results["NLSTAT1"].joint_displacements["1"][2];
    let uz_reimp = reimported.case_results["NLSTAT1"].joint_displacements["1"][2];
    assert!(
        (uz_orig - uz_reimp).abs() < 1e-9,
        "uz_orig = {uz_orig}, uz_reimp = {uz_reimp}"
    );
}

/// Native sections that the importer does not interpret are recorded per kind
/// and refuse only the cases they affect. Files and their recorded data:
/// Example 2-006 (area gravity loads in one pattern), 1-015 (frame added mass),
/// 1-009a (a tendon object and its prestress load), 2-008a (area auto-mesh and
/// added mass), 1-013a (frame line springs).
#[test]
fn vendor_sdb_skipped_sections_are_recorded_and_scoped() {
    use sap_model::{LoadCase, SapModel, UnmodeledScope};
    let dir = vendor_dir();
    let entries = |model: &SapModel| -> Vec<(UnmodeledScope, String)> {
        model
            .unmodeled_source
            .iter()
            .map(|e| (e.scope.clone(), e.description.clone()))
            .collect()
    };
    let only = |model: &mut SapModel, enabled: &[&str]| {
        for (name, case) in &mut model.load_cases {
            case.run_flag = enabled.contains(&name.as_str());
        }
    };

    // A load: only the case that applies its pattern is refused.
    let mut model = load_sdb_file(dir.join("example_2_006_thick.sdb")).expect("2_006");
    assert_eq!(
        entries(&model),
        [(
            UnmodeledScope::LoadPattern("GRAV".into()),
            "36 area gravity loads in load pattern 'GRAV'".to_string()
        )]
    );
    let error = analyze_model(&mut model).unwrap_err().to_string();
    assert!(error.contains("'GRAV'") && error.contains("area gravity loads"), "{error}");
    assert!(model.case_results.is_empty());
    only(&mut model, &["SELF", "UNIF"]);
    analyze_model(&mut model).expect("2_006 without the gravity-load case");
    assert_eq!(model.case_results.len(), 2);

    // Added mass: the modal case is refused, a static case is not.
    let mut model = load_sdb_file(dir.join("example_1_015.sdb")).expect("1_015");
    assert_eq!(
        entries(&model),
        [(
            UnmodeledScope::DynamicCases,
            "2 frame added-mass assignments".to_string()
        )]
    );
    assert!(model.frames.values().all(|f| f.auto_mesh_segments == 120));
    only(&mut model, &["MODAL1"]);
    let error = analyze_model(&mut model).unwrap_err().to_string();
    assert!(error.contains("frame added-mass"), "{error}");
    only(&mut model, &[]);
    model
        .load_cases
        .insert("STATIC".into(), LoadCase::linear_static("STATIC", "P", 1.0));
    analyze_model(&mut model).expect("1_015 static");
    assert!(model.case_results.contains_key("STATIC"));

    // A tendon object is not a frame member. It is recorded and left out
    // (it used to arrive as a W18X35 beam); nothing may be analyzed.
    let mut model = load_sdb_file(dir.join("example_1_009a.sdb")).expect("1_009a");
    assert_eq!(model.frames.keys().collect::<Vec<_>>(), ["1"]);
    assert_eq!(
        entries(&model),
        [
            (
                UnmodeledScope::AllCases,
                "tendon object '2' (not imported)".to_string()
            ),
            (
                UnmodeledScope::LoadPattern("PRE".into()),
                "1 tendon prestress load in load pattern 'PRE'".to_string()
            ),
        ]
    );
    only(&mut model, &["DEAD"]);
    let error = analyze_model(&mut model).unwrap_err().to_string();
    assert!(error.contains("tendon object '2'"), "{error}");

    // An auto-mesh whose options the model cannot express stays refused. In
    // Example 2-016 all four corners are restrained in Uz and the vendor mesh
    // restrains only the new edge joints, not the interior ones.
    let mut model = load_sdb_file(dir.join("example_2_016a_thick.sdb")).expect("2_016a");
    assert_eq!(
        entries(&model),
        [(
            UnmodeledScope::AllCases,
            "auto-mesh assignment of area '1' (16 x 16, type 1) with options that are not modeled"
                .to_string()
        )]
    );
    assert!(model.areas.values().all(|a| a.auto_mesh == (1, 1)));
    assert!(analyze_model(&mut model).is_err());

    // Frame auto-subdivision is modeled, so it is no longer recorded. The
    // design examples that subdivide at a joint are refused for other
    // reasons. The beam is inserted at its top center with stiffness
    // transformation; that is implemented for linear static cases, and the
    // file also runs a modal case.
    let mut model = load_sdb_file(dir.join("aci_318_08_ex001.sdb")).expect("aci_318_08_ex001");
    assert!(entries(&model).is_empty());
    assert!(model.frames["1"].auto_mesh_at_joints);
    assert_eq!(
        (model.frames["1"].cardinal_point, model.frames["1"].stiff_transform),
        (8, true)
    );
    let error = analyze_model(&mut model).unwrap_err().to_string();
    assert!(
        error.contains("frame '1' has an insertion point") && error.contains("subdivision at joints"),
        "{error}"
    );
    model.frames["1"].auto_mesh_at_joints = false;
    let error = analyze_model(&mut model).unwrap_err().to_string();
    assert!(error.contains("case 'MODAL' is not one"), "{error}");
    // With only the static case, and equally at its centroid, the beam is
    // free to spin about its axis and to slide sideways.
    only(&mut model, &["Ultimate"]);
    let error = analyze_model(&mut model).unwrap_err().to_string();
    assert!(error.contains("Singular matrix"), "{error}");
    model.frames["1"].stiff_transform = false;
    let error = analyze_model(&mut model).unwrap_err().to_string();
    assert!(error.contains("Singular matrix"), "{error}");

    let model = load_sdb_file(dir.join("example_1_014b.sdb")).expect("1_014b");
    assert!(entries(&model).is_empty());
    assert_eq!(model.frames["1"].auto_mesh_segments, 2);
    assert!(!model.frames["1"].auto_mesh_at_joints);

    let mut model = load_sdb_file(dir.join("example_1_013a.sdb")).expect("1_013a");
    assert_eq!(
        entries(&model),
        [(
            UnmodeledScope::AllCases,
            "2 frame line-spring assignments".to_string()
        )]
    );
    let error = analyze_model(&mut model).unwrap_err().to_string();
    assert!(error.contains("frame line-spring"), "{error}");
    assert!(pack_model_to_native_sdb(&model)
        .unwrap_err()
        .to_string()
        .contains("unmodeled_source"));
}

const EXAMPLE_2_008_MASS_PER_AREA: f64 = 7.35e-7;

fn example_2_008_periods(file: &str, mesh: Option<usize>) -> (sap_model::SapModel, Vec<f64>) {
    let mut model = load_sdb_file(vendor_dir().join(file)).expect(file);
    assert!(model.unmodeled_source.is_empty(), "{file}: {:?}", model.unmodeled_source);
    if let Some(n) = mesh {
        for area in model.areas.values_mut() {
            area.auto_mesh = (n, n);
        }
    }
    analyze_model(&mut model).expect(file);
    let periods = model.case_results["MODAL"]
        .modes
        .iter()
        .take(5)
        .map(|mode| mode.period_sec)
        .collect();
    (model, periods)
}

fn within(actual: &[f64], reference: &[f64], tolerance: f64, label: &str) {
    assert_eq!(actual.len(), reference.len(), "{label}");
    for (mode, (a, r)) in actual.iter().zip(reference).enumerate() {
        assert!(
            ((a - r) / r).abs() <= tolerance,
            "{label} mode {}: {a} vs {r} ({:+.2}%)",
            mode + 1,
            100.0 * (a - r) / r
        );
    }
}

/// CSI Software Verification Example 2-008, "Shell - Cantilever Plate
/// Eigenvalue Problem" (kip, in, s): a 24 x 24 x 1 in plate, E = 29000,
/// nu = 0.3, clamped along one edge, whose only mass is an area mass assignment
/// of 7.35e-7 kip-s2/in3. The files carry the N x N auto-mesh, the area mass
/// and (Model D) the area stiffness modifiers in their own native sections.
///
/// Published SAP2000 periods of the first five modes, 10 x 10 mesh:
/// thin  0.01743 0.00717 0.00288 0.00228 0.00200
/// thick 0.01746 0.00721 0.00290 0.00230 0.00202
/// Independent (Harris and Crede 1976): 0.01723 0.00704 0.00281 0.00219 0.00193.
#[test]
fn vendor_sdb_example_2_008_plate_periods_from_imported_mesh_mass_and_modifiers() {
    use sap_model::ShellType;
    let independent = [0.01723, 0.00704, 0.00281, 0.00219, 0.00193];

    let (thin_model, thin) = example_2_008_periods("example_2_008a_thin.sdb", None);
    let (thick_model, thick) = example_2_008_periods("example_2_008a_thick.sdb", None);
    for (model, shell_type) in [
        (&thin_model, ShellType::ShellThin),
        (&thick_model, ShellType::ShellThick),
    ] {
        let area = model.areas.values().next().unwrap();
        assert_eq!(area.auto_mesh, (10, 10));
        assert_eq!(area.mass_per_area, EXAMPLE_2_008_MASS_PER_AREA);
        assert_eq!(area.modifiers, [1.0; 10]);
        assert_eq!(model.area_sections[&area.section].shell_type, shell_type);
    }
    // Against the published SAP2000 values: 1.5 % (this element is not the
    // vendor element; the published values themselves are 1.2-5.0 % above the
    // independent ones at this mesh).
    within(&thin, &[0.01743, 0.00717, 0.00288, 0.00228, 0.00200], 0.015, "thin 10x10");
    within(&thick, &[0.01746, 0.00721, 0.00290, 0.00230, 0.00202], 0.015, "thick 10x10");
    within(&thin, &independent, 0.04, "thin 10x10 vs independent");
    within(&thick, &independent, 0.05, "thick 10x10 vs independent");
    // Shear deformation makes the thick plate slightly more flexible.
    for (mode, (thin, thick)) in thin.iter().zip(&thick).enumerate() {
        assert!(thick > thin, "mode {}: thick {thick} <= thin {thin}", mode + 1);
        assert!((thick - thin) / thin < 0.02);
    }

    // Model D: the thick plate with shear stiffness modifiers V13 = V23 = 1000,
    // which the verification manual reports as identical to the thin plate.
    // The file uses a 40 x 40 mesh, too large for this dense solver in a test;
    // the same model is solved on the 10 x 10 mesh and compared with the thin
    // 10 x 10 model.
    let model_d = load_sdb_file(vendor_dir().join("example_2_008d_thick.sdb")).expect("2_008d");
    let area = model_d.areas.values().next().unwrap();
    assert_eq!(area.auto_mesh, (40, 40));
    assert_eq!(area.mass_per_area, EXAMPLE_2_008_MASS_PER_AREA);
    assert_eq!(
        area.modifiers,
        [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1000.0, 1000.0, 1.0, 1.0]
    );
    assert_eq!(
        model_d.area_sections[&area.section].shell_type,
        ShellType::ShellThick
    );
    let (_, stiffened) = example_2_008_periods("example_2_008d_thick.sdb", Some(10));
    within(&stiffened, &thin, 1e-4, "Model D on 10x10 vs thin 10x10");

    // Example 2-018 sets the bending modifiers M11 = M22 = M12 = 1000 on both
    // of its areas. (Its large-displacement case is refused by the solver.)
    let model = load_sdb_file(vendor_dir().join("example_2_018_thick.sdb")).expect("2_018");
    assert_eq!(model.areas.len(), 2);
    for area in model.areas.values() {
        assert_eq!(
            area.modifiers,
            [1.0, 1.0, 1.0, 1000.0, 1000.0, 1000.0, 1.0, 1.0, 1.0, 1.0]
        );
    }
}

/// The 20 x 20 models of Example 2-008. Published SAP2000 periods:
/// thin  0.01737 0.00710 0.00284 0.00223 0.00196
/// thick 0.01740 0.00714 0.00286 0.00225 0.00198
/// About a minute per model in a debug build, so not run by default:
/// `cargo test -p sap-solver --features vendor-fixtures --test wave4_vendor_regressions -- --ignored`.
#[test]
#[ignore = "slow: two 2646-DOF dense modal solutions"]
fn vendor_sdb_example_2_008_plate_periods_on_the_20_x_20_mesh() {
    let (model, thin) = example_2_008_periods("example_2_008b_thin.sdb", None);
    assert_eq!(model.areas.values().next().unwrap().auto_mesh, (20, 20));
    let (_, thick) = example_2_008_periods("example_2_008b_thick.sdb", None);
    within(&thin, &[0.01737, 0.00710, 0.00284, 0.00223, 0.00196], 0.006, "thin 20x20");
    within(&thick, &[0.01740, 0.00714, 0.00286, 0.00225, 0.00198], 0.006, "thick 20x20");
    for (thin, thick) in thin.iter().zip(&thick) {
        assert!(thick > thin);
    }
}

/// CSI Software Verification Example 3-001, "Plane - Patch Test With
/// Prescribed Displacements" (lb, in): E = 1e6, nu = 0.25, thickness 0.001.
/// Edge displacements Ux = (x + y/2)/1000, Uy = (y + x/2)/1000 impose constant
/// strains of 1e-3, so the plane-stress stresses are
/// sigma_xx = sigma_yy = E (1 + nu) 1e-3 / (1 - nu^2) = 1333.33 psi and
/// tau_xy = G 1e-3 = 400 psi, and interior joints follow the same field.
/// The file's section is a plane-stress section; it used to import as a shell.
#[test]
fn vendor_sdb_example_3_001_plane_stress_patch_test() {
    use sap_model::{AreaPropType, PlaneType};
    for (file, incompatible) in [
        ("example_3_001_comp.sdb", false),
        ("example_3_001_incomp.sdb", true),
    ] {
        let mut model = load_sdb_file(vendor_dir().join(file)).expect(file);
        assert!(model.unmodeled_source.is_empty(), "{file}");
        let section = model.area_sections.values().next().unwrap();
        assert_eq!(section.prop_type, AreaPropType::Plane, "{file}");
        assert_eq!(section.plane_type, PlaneType::PlaneStress, "{file}");
        assert_eq!(section.incompatible_modes, incompatible, "{file}");
        assert_eq!(section.thickness_membrane, 0.001, "{file}");
        if incompatible {
            // The area element has no incompatible bending modes; the model
            // that asks for them is refused instead of solved without.
            let error = analyze_model(&mut model).unwrap_err().to_string();
            assert!(error.contains("incompatible bending modes"), "{error}");
            assert!(model.case_results.is_empty());
            continue;
        }
        analyze_model(&mut model).expect(file);
        let result = &model.case_results["MEMBRANE"];
        for (joint, point) in &model.points {
            let u = result.joint_displacements[joint];
            let expected = [
                (point.x + 0.5 * point.y) / 1000.0,
                (point.y + 0.5 * point.x) / 1000.0,
            ];
            for d in 0..2 {
                assert!(
                    (u[d] - expected[d]).abs() < 1e-12,
                    "{file} joint {joint} U{}: {} vs {}",
                    d + 1,
                    u[d],
                    expected[d]
                );
            }
        }
        // Membrane forces per unit length: stress times the 0.001 in thickness.
        for (area, forces) in &result.area_results {
            for (value, expected) in [
                (forces.f11, 1.333_333_333_333_333_3),
                (forces.f22, 1.333_333_333_333_333_3),
                (forces.f12, 0.4),
            ] {
                assert!(
                    (value - expected).abs() < 1e-9,
                    "{file} area {area}: {value} vs {expected}"
                );
            }
        }
    }
}

/// CSI Software Verification Example 5-002, "Solid - Straight Beam With Static
/// Loads", Model A (6 x 1 x 1 rectangular solids; lb, in). Published SAP2000
/// averages over tip joints 7, 14, 21 and 28:
///
/// | case | quantity | with incompatible modes | without |
/// | 1 axial | Ux | 2.926e-5 | 2.914e-5 |
/// | 2 Vz    | Uz | 0.1057   | 0.0100   |
/// | 3 Vy    | Uy | 0.4203   | 0.0109   |
///
/// The tenfold bending difference identifies which file has the option on:
/// the native flag is 1 when it is off. The tip joints are tied by a body
/// constraint, enforced here by penalty; the "incomp" model is flexible enough
/// that the factorization's singularity test must not be relative to that
/// penalty.
#[test]
fn vendor_sdb_example_5_002_incompatible_modes_flag_matches_published_deflections() {
    for (file, incompatible, published) in [
        ("example_5_002a1_incomp.sdb", true, [2.926e-5, 0.1057, 0.4203]),
        ("example_5_002a1_comp.sdb", false, [2.914e-5, 0.0100, 0.0109]),
    ] {
        let mut model = load_sdb_file(vendor_dir().join(file)).expect(file);
        assert!(model
            .solid_sections
            .values()
            .all(|section| section.incompatible_modes == incompatible));
        assert_eq!(model.constraints.len(), 1, "{file}: tip body constraint");
        analyze_model(&mut model).expect(file);
        for (case, dof, expected) in [("1", 0, published[0]), ("2", 2, published[1]), ("3", 1, published[2])] {
            let tip = ["7", "14", "21", "28"]
                .iter()
                .map(|joint| model.case_results[case].joint_displacements[*joint][dof])
                .sum::<f64>()
                / 4.0;
            // Published to four significant digits.
            assert!(
                ((tip - expected) / expected).abs() < 0.01,
                "{file} case {case}: tip {tip} vs published {expected}"
            );
        }
    }
}

/// Automatic frame subdivision, end to end from the vendor model files:
/// every published SAP2000 value of the examples that use it.
#[test]
fn vendor_sdb_frame_auto_subdivision_examples_match_published_values() {
    let dir = vendor_dir();
    let solved = |fname: &str| {
        let mut model = load_sdb_file(dir.join(fname)).unwrap_or_else(|e| panic!("{fname}: {e}"));
        assert!(model.unmodeled_source.is_empty(), "{fname}: {:?}", model.unmodeled_source);
        analyze_model(&mut model).unwrap_or_else(|e| panic!("{fname}: {e}"));
        // Results name the model's own joints and frames only.
        for results in model.case_results.values() {
            assert!(results.joint_displacements.keys().all(|j| model.points.contains_key(j)));
            assert!(results.frame_forces.keys().all(|f| model.frames.contains_key(f)));
        }
        model
    };

    // Example 1-014: cantilever periods (s) per number of elements.
    let periods: [(&str, usize, &[f64]); 7] = [
        ("a", 0, &[0.054547, 0.036364]),
        ("b", 2, &[0.042333, 0.028222, 0.008218, 0.005479]),
        ("c", 4, &[0.039090, 0.026060, 0.006651, 0.004434, 0.002511]),
        ("d", 6, &[0.038485, 0.025657, 0.006330, 0.004220, 0.002321]),
        ("e", 8, &[0.038273, 0.025516, 0.006214, 0.004143, 0.002254]),
        ("f", 10, &[0.038175, 0.025450, 0.006160, 0.004107, 0.002222]),
        ("g", 96, &[0.038003, 0.025335, 0.006065, 0.004043, 0.002166]),
    ];
    for (letter, segments, published) in periods {
        let fname = format!("example_1_014{letter}.sdb");
        let model = solved(&fname);
        assert_eq!(model.frames["1"].auto_mesh_segments, segments, "{fname}");
        let modes = &model.case_results["MODAL1"].modes;
        assert_eq!(modes.len(), published.len(), "{fname}");
        for (mode, expected) in modes.iter().zip(published) {
            assert!(
                (mode.period_sec - expected).abs() < 6e-7,
                "{fname} mode {}: {} vs {expected}",
                mode.mode_num,
                mode.period_sec
            );
        }
    }

    // Example 1-019: buckling load of a rigid frame (kip).
    for (letter, segments, published) in [("a", 0, 280.98), ("b", 2, 280.24), ("c", 4, 280.19)] {
        let fname = format!("example_1_019{letter}.sdb");
        let model = solved(&fname);
        assert!(model.frames.values().all(|f| f.auto_mesh_segments == segments), "{fname}");
        let factor = model.case_results["BUCK1"].buckling_factors[0];
        assert!((factor - published).abs() < 6e-3, "{fname}: {factor} vs {published}");
    }

    // Example 1-016: midpoint Uz (in) and My (kip-in) of a tie rod, without
    // tension, with an assigned P-Delta force, and with a P-Delta case.
    for (letter, segments, uz, my) in [
        ("a", 0, -1.04167, 22.500),
        ("b", 0, -0.54555, 11.453),
        ("c", 2, -0.54343, 11.495),
        ("d", 16, -0.54330, 11.498),
        ("e", 0, -0.54555, 11.453),
        ("f", 2, -0.54343, 11.495),
        ("g", 16, -0.54330, 11.498),
    ] {
        let fname = format!("example_1_016{letter}.sdb");
        let model = solved(&fname);
        assert!(model.frames.values().all(|f| f.auto_mesh_segments == segments), "{fname}");
        let results = &model.case_results["1"];
        let got = results.joint_displacements["2"][2];
        assert!((got - uz).abs() < 6e-6, "{fname}: Uz {got} vs {uz}");
        let station = results.frame_forces["1"].last().unwrap();
        assert!((station.rel_dist - 1.0).abs() < 1e-12);
        assert!((station.m3.abs() - my).abs() < 6e-4, "{fname}: My {} vs {my}", station.m3);
    }

    // Example 1-017: frequencies of a tensioned string (Hz).
    for (letter, segments, published) in
        [("a", 10, [74.579, 148.93, 222.06]), ("b", 100, [74.587, 149.18, 223.80])]
    {
        let fname = format!("example_1_017{letter}.sdb");
        let model = solved(&fname);
        assert_eq!(model.frames["1"].auto_mesh_segments, segments, "{fname}");
        let modes = &model.case_results["MODAL"].modes;
        assert_eq!(modes.len(), 3, "{fname}");
        for (mode, expected) in modes.iter().zip(published) {
            let tolerance = if expected < 100.0 { 6e-4 } else { 6e-3 };
            assert!(
                (mode.freq_hz - expected).abs() < tolerance,
                "{fname} mode {}: {} vs {expected}",
                mode.mode_num,
                mode.freq_hz
            );
        }
    }

    // Example 1-006: non-prismatic cantilever, one element and ten.
    for (letter, segments) in [("a", 0), ("b", 10)] {
        let fname = format!("example_1_006{letter}.sdb");
        let model = solved(&fname);
        assert_eq!(model.frames["1"].auto_mesh_segments, segments, "{fname}");
        let fixed_end = &model.case_results["1"].frame_forces["1"][0];
        assert!((fixed_end.m3 + 184.950).abs() < 6e-4, "{fname}: My {}", fixed_end.m3);
        for (index, expected) in [0.09087, 0.43120, -0.14335, 0.07987, 0.03742, 0.07634]
            .into_iter()
            .enumerate()
        {
            let case = (index + 2).to_string();
            let got = model.case_results[&case].joint_displacements["2"][index];
            assert!((got - expected).abs() < 6e-6, "{fname} case {case}: {got} vs {expected}");
        }
    }
}

/// Example 1-011 (frame insertion point), end to end from the vendor file:
/// cardinal point 1, joint offsets read from `FOffset`, and every published
/// value.
#[test]
fn vendor_sdb_example_1_011_insertion_point_matches_published_values() {
    let dir = vendor_dir();
    let mut model = load_sdb_file(dir.join("example_1_011.sdb")).expect("1_011");
    assert!(model.unmodeled_source.is_empty(), "{:?}", model.unmodeled_source);
    let frame = &model.frames["1"];
    assert_eq!((frame.cardinal_point, frame.stiff_transform), (1, true));
    assert!(!frame.joint_offset_is_global);
    assert_eq!(frame.joint_offset_i, [0.0, 12.0, 0.0]);
    assert_eq!(frame.joint_offset_j, [0.0, 12.0, 0.0]);
    analyze_model(&mut model).expect("1_011");

    // Case 1, 10 kip compression on the end joint: no fixed-end moment,
    // M3 = -210 and M2 = -60 kip-in along the beam.
    let case1 = &model.case_results["1"];
    let reaction = case1.joint_reactions["1"];
    assert!(reaction[4].abs() < 1e-6 && reaction[5].abs() < 1e-6, "{reaction:?}");
    for station in &case1.frame_forces["1"] {
        assert!((station.m3 + 210.0).abs() < 1e-6, "{station:?}");
        assert!((station.m2 + 60.0).abs() < 1e-6, "{station:?}");
    }
    // Case 2, the same load on the end of the frame object: fixed-end
    // My = 210 and Mz = 60 kip-in, no moment in the beam.
    let case2 = &model.case_results["2"];
    let reaction = case2.joint_reactions["1"];
    assert!((reaction[4] - 210.0).abs() < 1e-6, "{reaction:?}");
    assert!((reaction[5] - 60.0).abs() < 1e-6, "{reaction:?}");
    for station in &case2.frame_forces["1"] {
        assert!(station.m3.abs() < 1e-6 && station.m2.abs() < 1e-6, "{station:?}");
    }
}

/// Per-end assignment records are written with the array shape and field
/// values the vendor files have: the bytes of the record synthesized from the
/// imported model equal the bytes of the source record.
#[test]
fn vendor_sdb_per_end_assignment_records_are_written_byte_for_byte() {
    use sap_format::sdb::{read_sdb_file, SdbFile};
    let dir = vendor_dir();
    let records = |sdb: &SdbFile, class: &str| -> Vec<Vec<u8>> {
        inspect_native_sdb(sdb)
            .unwrap()
            .records()
            .iter()
            .filter(|r| r.class_id == class)
            .map(|r| sdb.decompressed_stream[r.offset..r.offset + r.length].to_vec())
            .collect()
    };
    // Joint offsets (3 x 2) in Example 1-011, partial fixity (6 x 2) in 1-008.
    for (fname, class) in [("example_1_011.sdb", "TYFOffset"), ("example_1_008.sdb", "TYFFixity")] {
        let source = read_sdb_file(dir.join(fname)).unwrap_or_else(|e| panic!("{fname}: {e}"));
        let model = extract_sdb_to_model(&source).unwrap_or_else(|e| panic!("{fname}: {e}"));
        let written = pack_model_to_native_sdb(&model).unwrap_or_else(|e| panic!("{fname}: {e}"));
        let (original, synthesized) = (records(&source, class), records(&written, class));
        assert_eq!(original.len(), 1, "{fname} {class}");
        assert_eq!(synthesized, original, "{fname} {class}");
    }
}
