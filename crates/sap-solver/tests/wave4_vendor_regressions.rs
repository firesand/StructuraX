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
        nldhist1.case_type = LoadCaseType::NonlinearHistory;
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
            nlmhist1.loads.clear();
            nlmhist1.history_loads = vec![("RAMP20".to_string(), "P".to_string(), 1.0)];
            nlmhist1.num_time_steps = 400;
            nlmhist1.time_step_dt = 0.1;
            let mut cont = LoadCase::linear_static("NLMHIST2", "V", 1.0);
            cont.case_type = LoadCaseType::NonlinearHistory;
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
