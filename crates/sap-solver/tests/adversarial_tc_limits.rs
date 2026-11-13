//! Empirical Adversarial Stress Test Suite for SOLV-01
//! (Frame Tension/Compression limits under extreme span loads, boundary conditions, and load reversals)

use sap_model::*;
use sap_solver::analyze_model;

fn assert_close(a: f64, b: f64, tol: f64, msg: &str) {
    let diff = (a - b).abs();
    assert!(
        diff <= tol,
        "{msg}: got {a}, expected {b}, diff {diff} > tol {tol}"
    );
}

// ============================================================================
// 1. Extreme Distributed Loads: W_axial >> T_lim
// ============================================================================

#[test]
fn test_extreme_span_load_tension_cutoff() {
    let mut model = SapModel::default();
    model.active_dofs = [true, false, false, false, false, false];
    model
        .load_patterns
        .get_mut("DEAD")
        .unwrap()
        .self_wt_multiplier = 0.0;
    model.load_cases.clear();

    model.add_point("A", 0.0, 0.0, 0.0);
    model.add_point("B", 100.0, 0.0, 0.0);
    model.points.get_mut("A").unwrap().restraint = [true; 6];
    model.points.get_mut("B").unwrap().spring[0] = 500.0; // parallel spring

    let mut mat = Material::default_steel_kip_in();
    mat.name = "STEEL".into();
    mat.e = 30000.0;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0;
    model.materials.insert("STEEL".into(), mat);

    let sec = FrameSection {
        name: "BAR".into(),
        material: "STEEL".into(),
        area: 10.0, // EA/L = 3000 kip/in
        ..FrameSection::default()
    };
    model.frame_sections.insert("BAR".into(), sec);

    model.add_frame("BAR1", "A", "B", "BAR");
    // T_lim = 50 kips, C_lim = None
    model.frames.get_mut("BAR1").unwrap().tc_limits = Some((false, 0.0, true, 50.0));

    // Massive distributed axial load: q_x = 100 kip/in -> W_axial = 10,000 kips >> T_lim (50 kips)
    model
        .frames
        .get_mut("BAR1")
        .unwrap()
        .loads_dist
        .push(FrameLoadDistributed {
            load_pat: "DEAD".into(),
            my_type: 1,
            dir: 1,
            csys: "LOCAL".into(),
            rel_dist1: 0.0,
            rel_dist2: 1.0,
            val1: 100.0,
            val2: 100.0,
        });

    let mut nl_case = LoadCase::linear_static("NL_EXTREME_T", "DEAD", 1.0);
    nl_case.case_type = LoadCaseType::NonlinearStatic;
    model.load_cases.insert("NL_EXTREME_T".into(), nl_case);

    analyze_model(&mut model).expect("solve extreme tension span load");

    let res = &model.case_results["NL_EXTREME_T"];
    let r_a = res.joint_reactions["A"][0];
    let r_b = res.joint_reactions["B"][0];

    // 1. Exact tension cutoff at support A: R_A = -50.000 kips
    assert_close(r_a, -50.0, 1e-6, "Support reaction at A clamped to -T_lim");

    // 2. Global static equilibrium: R_A + R_B + 10,000 = 0 to machine precision
    assert_close(r_a + r_b + 10000.0, 0.0, 1e-9, "Global X equilibrium");

    // 3. Station forces: P(x) <= 50.0 kips everywhere across member
    let stations = &res.frame_forces["BAR1"];
    assert!(!stations.is_empty());
    for sta in stations {
        assert!(
            sta.p <= 50.0 + 1e-6,
            "Internal station tension exceeds T_lim: p = {}",
            sta.p
        );
    }
    // Peak tension is at x = 0: P(0) = 50.0 kips
    assert_close(stations.first().unwrap().p, 50.0, 1e-6, "Station 0 tension");
    // Station at x = L: P(L) = 50 - 10000 = -9950 kips
    assert_close(
        stations.last().unwrap().p,
        -9950.0,
        1e-6,
        "Station L tension",
    );
}

// ============================================================================
// 2. Extreme Distributed Loads: |W_axial| >> C_lim
// ============================================================================

#[test]
fn test_extreme_span_load_compression_cutoff() {
    let mut model = SapModel::default();
    model.active_dofs = [true, false, false, false, false, false];
    model
        .load_patterns
        .get_mut("DEAD")
        .unwrap()
        .self_wt_multiplier = 0.0;
    model.load_cases.clear();

    model.add_point("A", 0.0, 0.0, 0.0);
    model.add_point("B", 100.0, 0.0, 0.0);
    model.points.get_mut("A").unwrap().restraint = [true; 6];
    model.points.get_mut("B").unwrap().spring[0] = 500.0;

    let mut mat = Material::default_steel_kip_in();
    mat.name = "STEEL".into();
    mat.e = 30000.0;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0;
    model.materials.insert("STEEL".into(), mat);

    let sec = FrameSection {
        name: "BAR".into(),
        material: "STEEL".into(),
        area: 10.0,
        ..FrameSection::default()
    };
    model.frame_sections.insert("BAR".into(), sec);

    model.add_frame("BAR1", "A", "B", "BAR");
    // C_lim = 60 kips, T_lim = None
    model.frames.get_mut("BAR1").unwrap().tc_limits = Some((true, 60.0, false, 0.0));

    // Massive compressive distributed axial load: q_x = -100 kip/in -> W_axial = -10,000 kips >> C_lim (60 kips)
    model
        .frames
        .get_mut("BAR1")
        .unwrap()
        .loads_dist
        .push(FrameLoadDistributed {
            load_pat: "DEAD".into(),
            my_type: 1,
            dir: 1,
            csys: "LOCAL".into(),
            rel_dist1: 0.0,
            rel_dist2: 1.0,
            val1: -100.0,
            val2: -100.0,
        });

    let mut nl_case = LoadCase::linear_static("NL_EXTREME_C", "DEAD", 1.0);
    nl_case.case_type = LoadCaseType::NonlinearStatic;
    model.load_cases.insert("NL_EXTREME_C".into(), nl_case);

    analyze_model(&mut model).expect("solve extreme compression span load");

    let res = &model.case_results["NL_EXTREME_C"];
    let r_a = res.joint_reactions["A"][0];
    let r_b = res.joint_reactions["B"][0];

    // 1. Support reaction at A: R_A = +60.000 kips
    assert_close(r_a, 60.0, 1e-6, "Support reaction at A clamped to +C_lim");

    // 2. Global static equilibrium: R_A + R_B - 10,000 = 0
    assert_close(r_a + r_b - 10000.0, 0.0, 1e-9, "Global X equilibrium");

    // 3. Station forces: P(x) >= -60.0 kips everywhere across member
    let stations = &res.frame_forces["BAR1"];
    assert!(!stations.is_empty());
    for sta in stations {
        assert!(
            sta.p >= -60.0 - 1e-6,
            "Internal station compression exceeds C_lim: p = {}",
            sta.p
        );
    }
    // Most compressive station is at x = 0: P(0) = -60.0 kips
    assert_close(
        stations.first().unwrap().p,
        -60.0,
        1e-6,
        "Station 0 compression",
    );
    // Station at x = L: P(L) = -60 - (-10000) = +9940 kips (tension)
    assert_close(
        stations.last().unwrap().p,
        9940.0,
        1e-6,
        "Station L tension",
    );
}

// ============================================================================
// 3. Zero Limits: Compression-Only Member (T_lim = 0) under Massive Tension
// ============================================================================

#[test]
fn test_zero_limit_compression_only_under_tension() {
    let mut model = SapModel::default();
    model.active_dofs = [true, false, false, false, false, false];
    model
        .load_patterns
        .get_mut("DEAD")
        .unwrap()
        .self_wt_multiplier = 0.0;
    model.load_cases.clear();

    model.add_point("A", 0.0, 0.0, 0.0);
    model.add_point("B", 100.0, 0.0, 0.0);
    model.points.get_mut("A").unwrap().restraint = [true; 6];
    model.points.get_mut("B").unwrap().spring[0] = 500.0;

    let mut mat = Material::default_steel_kip_in();
    mat.name = "STEEL".into();
    mat.e = 30000.0;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0;
    model.materials.insert("STEEL".into(), mat);

    let sec = FrameSection {
        name: "BAR".into(),
        material: "STEEL".into(),
        area: 10.0,
        ..FrameSection::default()
    };
    model.frame_sections.insert("BAR".into(), sec);

    model.add_frame("BAR1", "A", "B", "BAR");
    // Compression-only member: T_lim = 0.0, C_lim = None
    model.frames.get_mut("BAR1").unwrap().tc_limits = Some((false, 0.0, true, 0.0));

    // Tensile span distributed load: q_x = +10 kip/in -> 1000 kips + nodal pull 500 kips
    model
        .frames
        .get_mut("BAR1")
        .unwrap()
        .loads_dist
        .push(FrameLoadDistributed {
            load_pat: "DEAD".into(),
            my_type: 1,
            dir: 1,
            csys: "LOCAL".into(),
            rel_dist1: 0.0,
            rel_dist2: 1.0,
            val1: 10.0,
            val2: 10.0,
        });
    model
        .points
        .get_mut("B")
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: [500.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        });

    let mut nl_case = LoadCase::linear_static("NL_C_ONLY", "DEAD", 1.0);
    nl_case.case_type = LoadCaseType::NonlinearStatic;
    model.load_cases.insert("NL_C_ONLY".into(), nl_case);

    analyze_model(&mut model).expect("solve compression only under tension");

    let res = &model.case_results["NL_C_ONLY"];
    let r_a = res.joint_reactions["A"][0];
    let r_b = res.joint_reactions["B"][0];

    // Total external load: 1000 + 500 = 1500 kips
    // Member cannot take tension: support A reaction must be clamped at 0.0
    assert_close(r_a, 0.0, 1e-6, "Support A reaction for T_lim = 0");
    // Spring carries all 1500 kips
    assert_close(
        r_b,
        -1500.0,
        1e-6,
        "Spring reaction carries entire tension load",
    );
    // Equilibrium: R_A + R_B + 1500 = 0
    assert_close(r_a + r_b + 1500.0, 0.0, 1e-9, "Equilibrium");

    // All station forces must have p <= 1e-6 (zero tension everywhere)
    for sta in &res.frame_forces["BAR1"] {
        assert!(
            sta.p <= 1e-6,
            "Tension found in compression-only member: p = {}",
            sta.p
        );
    }
}

// ============================================================================
// 4. Sign Reversibility: +w1 vs -w1 Symmetry
// ============================================================================

#[test]
fn test_sign_reversibility_symmetric_limits() {
    let build_model = |load_sign: f64| {
        let mut model = SapModel::default();
        model.active_dofs = [true, false, false, false, false, false];
        model
            .load_patterns
            .get_mut("DEAD")
            .unwrap()
            .self_wt_multiplier = 0.0;
        model.load_cases.clear();

        model.add_point("A", 0.0, 0.0, 0.0);
        model.add_point("B", 100.0, 0.0, 0.0);
        model.points.get_mut("A").unwrap().restraint = [true; 6];
        model.points.get_mut("B").unwrap().spring[0] = 1000.0;

        let mut mat = Material::default_steel_kip_in();
        mat.name = "STEEL".into();
        mat.e = 30000.0;
        mat.unit_mass = 0.0;
        mat.unit_weight = 0.0;
        model.materials.insert("STEEL".into(), mat);

        let sec = FrameSection {
            name: "BAR".into(),
            material: "STEEL".into(),
            area: 10.0,
            ..FrameSection::default()
        };
        model.frame_sections.insert("BAR".into(), sec);

        model.add_frame("BAR1", "A", "B", "BAR");
        // Symmetric limits: T_lim = 80, C_lim = 80
        model.frames.get_mut("BAR1").unwrap().tc_limits = Some((true, 80.0, true, 80.0));

        model
            .frames
            .get_mut("BAR1")
            .unwrap()
            .loads_dist
            .push(FrameLoadDistributed {
                load_pat: "DEAD".into(),
                my_type: 1,
                dir: 1,
                csys: "LOCAL".into(),
                rel_dist1: 0.0,
                rel_dist2: 1.0,
                val1: 1.5 * load_sign,
                val2: 1.5 * load_sign,
            });
        model
            .points
            .get_mut("B")
            .unwrap()
            .loads_force
            .push(PointLoadForce {
                load_pat: "DEAD".into(),
                csys: "GLOBAL".into(),
                force: [50.0 * load_sign, 0.0, 0.0, 0.0, 0.0, 0.0],
            });

        let mut nl_case = LoadCase::linear_static("NL", "DEAD", 1.0);
        nl_case.case_type = LoadCaseType::NonlinearStatic;
        model.load_cases.insert("NL".into(), nl_case);

        analyze_model(&mut model).expect("solve sign reversibility");
        model
    };

    let model_pos = build_model(1.0);
    let model_neg = build_model(-1.0);

    let res_pos = &model_pos.case_results["NL"];
    let res_neg = &model_neg.case_results["NL"];

    let ra_pos = res_pos.joint_reactions["A"][0];
    let ra_neg = res_neg.joint_reactions["A"][0];
    let rb_pos = res_pos.joint_reactions["B"][0];
    let rb_neg = res_neg.joint_reactions["B"][0];
    let ub_pos = res_pos.joint_displacements["B"][0];
    let ub_neg = res_neg.joint_displacements["B"][0];

    // Check exact anti-symmetry
    assert_close(ra_pos, -ra_neg, 1e-9, "Reaction A sign reversibility");
    assert_close(rb_pos, -rb_neg, 1e-9, "Reaction B sign reversibility");
    assert_close(ub_pos, -ub_neg, 1e-9, "Displacement B sign reversibility");

    let st_pos = &res_pos.frame_forces["BAR1"];
    let st_neg = &res_neg.frame_forces["BAR1"];
    assert_eq!(st_pos.len(), st_neg.len());
    for (p_pos, p_neg) in st_pos.iter().zip(st_neg.iter()) {
        assert_close(p_pos.p, -p_neg.p, 1e-9, "Station force sign reversibility");
    }
}

// ============================================================================
// 5. Non-Uniform Point Loads on Span
// ============================================================================

#[test]
fn test_non_uniform_axial_point_loads_on_span() {
    let mut model = SapModel::default();
    model.active_dofs = [true, false, false, false, false, false];
    model
        .load_patterns
        .get_mut("DEAD")
        .unwrap()
        .self_wt_multiplier = 0.0;
    model.load_cases.clear();

    model.add_point("A", 0.0, 0.0, 0.0);
    model.add_point("B", 100.0, 0.0, 0.0);
    model.points.get_mut("A").unwrap().restraint = [true; 6];
    model.points.get_mut("B").unwrap().spring[0] = 1000.0;

    let mut mat = Material::default_steel_kip_in();
    mat.name = "STEEL".into();
    mat.e = 30000.0;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0;
    model.materials.insert("STEEL".into(), mat);

    let sec = FrameSection {
        name: "BAR".into(),
        material: "STEEL".into(),
        area: 10.0,
        ..FrameSection::default()
    };
    model.frame_sections.insert("BAR".into(), sec);

    model.add_frame("BAR1", "A", "B", "BAR");
    model.frames.get_mut("BAR1").unwrap().min_stations = 21;
    // T_lim = 100 kips, C_lim = None
    model.frames.get_mut("BAR1").unwrap().tc_limits = Some((false, 0.0, true, 100.0));

    // Two non-uniform point loads along span:
    // P1 = +150 kips at s = 0.3 (x = 30)
    // P2 = +80 kips at s = 0.7 (x = 70)
    model
        .frames
        .get_mut("BAR1")
        .unwrap()
        .loads_point
        .push(FrameLoadPoint {
            load_pat: "DEAD".into(),
            my_type: 1,
            dir: 1,
            csys: "LOCAL".into(),
            rel_dist: 0.3,
            val: 150.0,
        });
    model
        .frames
        .get_mut("BAR1")
        .unwrap()
        .loads_point
        .push(FrameLoadPoint {
            load_pat: "DEAD".into(),
            my_type: 1,
            dir: 1,
            csys: "LOCAL".into(),
            rel_dist: 0.7,
            val: 80.0,
        });

    let mut nl_case = LoadCase::linear_static("NL_PTS", "DEAD", 1.0);
    nl_case.case_type = LoadCaseType::NonlinearStatic;
    model.load_cases.insert("NL_PTS".into(), nl_case);

    analyze_model(&mut model).expect("solve non-uniform point loads");

    let res = &model.case_results["NL_PTS"];
    let r_a = res.joint_reactions["A"][0];
    let r_b = res.joint_reactions["B"][0];

    // Total external load: 150 + 80 = 230 kips
    assert_close(r_a + r_b + 230.0, 0.0, 1e-9, "Global X equilibrium");

    // Tension limit check: R_A should clamp to -100.0
    assert_close(r_a, -100.0, 1e-6, "Reaction at A clamped to -100.0");
    assert_close(
        r_b,
        -130.0,
        1e-6,
        "Spring reaction at B carries remainder 130.0",
    );

    let stations = &res.frame_forces["BAR1"];
    for sta in stations {
        assert!(
            sta.p <= 100.0 + 1e-6,
            "Station tension exceeds 100.0: p = {}",
            sta.p
        );
    }
}

// ============================================================================
// 6. Trapezoidal Non-Uniform Span Load
// ============================================================================

#[test]
fn test_trapezoidal_non_uniform_axial_span_load() {
    let mut model = SapModel::default();
    model.active_dofs = [true, false, false, false, false, false];
    model
        .load_patterns
        .get_mut("DEAD")
        .unwrap()
        .self_wt_multiplier = 0.0;
    model.load_cases.clear();

    model.add_point("A", 0.0, 0.0, 0.0);
    model.add_point("B", 100.0, 0.0, 0.0);
    model.points.get_mut("A").unwrap().restraint = [true; 6];
    model.points.get_mut("B").unwrap().spring[0] = 1000.0;

    let mut mat = Material::default_steel_kip_in();
    mat.name = "STEEL".into();
    mat.e = 30000.0;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0;
    model.materials.insert("STEEL".into(), mat);

    let sec = FrameSection {
        name: "BAR".into(),
        material: "STEEL".into(),
        area: 10.0,
        ..FrameSection::default()
    };
    model.frame_sections.insert("BAR".into(), sec);

    model.add_frame("BAR1", "A", "B", "BAR");
    model.frames.get_mut("BAR1").unwrap().min_stations = 21;
    // T_lim = 120 kips, C_lim = None
    model.frames.get_mut("BAR1").unwrap().tc_limits = Some((false, 0.0, true, 120.0));

    // Trapezoidal load: q(0) = 1.0 kip/in, q(1) = 3.0 kip/in
    // Average q = 2.0 kip/in -> Total W_axial = 200 kips
    model
        .frames
        .get_mut("BAR1")
        .unwrap()
        .loads_dist
        .push(FrameLoadDistributed {
            load_pat: "DEAD".into(),
            my_type: 1,
            dir: 1,
            csys: "LOCAL".into(),
            rel_dist1: 0.0,
            rel_dist2: 1.0,
            val1: 1.0,
            val2: 3.0,
        });

    let mut nl_case = LoadCase::linear_static("NL_TRAP", "DEAD", 1.0);
    nl_case.case_type = LoadCaseType::NonlinearStatic;
    model.load_cases.insert("NL_TRAP".into(), nl_case);

    analyze_model(&mut model).expect("solve trapezoidal span load");

    let res = &model.case_results["NL_TRAP"];
    let r_a = res.joint_reactions["A"][0];
    let r_b = res.joint_reactions["B"][0];

    // Total load = 200 kips
    assert_close(
        r_a + r_b + 200.0,
        0.0,
        1e-9,
        "Equilibrium under trapezoidal load",
    );
    // Station forces <= 120.0
    for sta in &res.frame_forces["BAR1"] {
        assert!(
            sta.p <= 120.0 + 1e-6,
            "Station tension exceeds 120.0: p = {}",
            sta.p
        );
    }
}

// ============================================================================
// 7. Fixed-Fixed Bar under Distributed Axial Span Load
// ============================================================================

#[test]
fn test_fixed_fixed_bar_equilibrium() {
    let mut model = SapModel::default();
    model.active_dofs = [true, false, false, false, false, false];
    model
        .load_patterns
        .get_mut("DEAD")
        .unwrap()
        .self_wt_multiplier = 0.0;
    model.load_cases.clear();

    model.add_point("A", 0.0, 0.0, 0.0);
    model.add_point("B", 100.0, 0.0, 0.0);
    // Both ends fully restrained
    model.points.get_mut("A").unwrap().restraint = [true; 6];
    model.points.get_mut("B").unwrap().restraint = [true; 6];

    let mut mat = Material::default_steel_kip_in();
    mat.name = "STEEL".into();
    mat.e = 30000.0;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0;
    model.materials.insert("STEEL".into(), mat);

    let sec = FrameSection {
        name: "BAR".into(),
        material: "STEEL".into(),
        area: 10.0,
        ..FrameSection::default()
    };
    model.frame_sections.insert("BAR".into(), sec);

    model.add_frame("BAR1", "A", "B", "BAR");
    // Tension limit = 200.0
    model.frames.get_mut("BAR1").unwrap().tc_limits = Some((false, 0.0, true, 200.0));

    // Distributed axial load: q = 5.0 kip/in -> W_axial = 500 kips
    model
        .frames
        .get_mut("BAR1")
        .unwrap()
        .loads_dist
        .push(FrameLoadDistributed {
            load_pat: "DEAD".into(),
            my_type: 1,
            dir: 1,
            csys: "LOCAL".into(),
            rel_dist1: 0.0,
            rel_dist2: 1.0,
            val1: 5.0,
            val2: 5.0,
        });

    let mut nl_case = LoadCase::linear_static("NL_FIXED", "DEAD", 1.0);
    nl_case.case_type = LoadCaseType::NonlinearStatic;
    model.load_cases.insert("NL_FIXED".into(), nl_case);

    analyze_model(&mut model).expect("solve fixed-fixed bar");

    let res = &model.case_results["NL_FIXED"];
    let r_a = res.joint_reactions["A"][0];
    let r_b = res.joint_reactions["B"][0];

    // Global equilibrium MUST be satisfied identically
    assert_close(r_a + r_b + 500.0, 0.0, 1e-9, "Fixed-fixed equilibrium");

    // All stations must satisfy tension limit
    for sta in &res.frame_forces["BAR1"] {
        assert!(
            sta.p <= 200.0 + 1e-6,
            "Fixed-fixed bar station tension exceeds 200: p = {}",
            sta.p
        );
    }
}

// ============================================================================
// 8. X-Braced Frame with Axial Span Loads and Reversal
// ============================================================================

#[test]
fn test_x_braced_frame_span_loads_and_reversal() {
    let run_x_brace = |lateral_force: f64| {
        let mut model = SapModel::default();
        model.active_dofs = [true, false, true, false, true, false]; // 2D X-Z
        model
            .load_patterns
            .get_mut("DEAD")
            .unwrap()
            .self_wt_multiplier = 0.0;
        model.load_cases.clear();

        model.add_point("B1", 0.0, 0.0, 0.0);
        model.add_point("B2", 120.0, 0.0, 0.0);
        model.add_point("T1", 0.0, 0.0, 120.0);
        model.add_point("T2", 120.0, 0.0, 120.0);

        model.points.get_mut("B1").unwrap().restraint = [true; 6];
        model.points.get_mut("B2").unwrap().restraint = [true; 6];

        let mut mat = Material::default_steel_kip_in();
        mat.name = "STEEL".into();
        mat.e = 30000.0;
        model.materials.insert("STEEL".into(), mat);

        let sec_frame = FrameSection {
            name: "FRAME_SEC".into(),
            material: "STEEL".into(),
            area: 25.0,
            i33: 1500.0,
            i22: 1500.0,
            ..FrameSection::default()
        };
        let sec_brace = FrameSection {
            name: "BRACE_SEC".into(),
            material: "STEEL".into(),
            area: 5.0,
            i33: 10.0,
            i22: 10.0,
            ..FrameSection::default()
        };
        model.frame_sections.insert("FRAME_SEC".into(), sec_frame);
        model.frame_sections.insert("BRACE_SEC".into(), sec_brace);

        model.add_frame("C1", "B1", "T1", "FRAME_SEC");
        model.add_frame("C2", "B2", "T2", "FRAME_SEC");
        model.add_frame("BEAM", "T1", "T2", "FRAME_SEC");

        // Diagonals are tension-only: C_lim = 0.0
        model.add_frame("D1", "B1", "T2", "BRACE_SEC");
        model.add_frame("D2", "B2", "T1", "BRACE_SEC");
        model.frames.get_mut("D1").unwrap().tc_limits = Some((true, 0.0, false, 0.0));
        model.frames.get_mut("D2").unwrap().tc_limits = Some((true, 0.0, false, 0.0));

        // Symmetric lateral push/pull divided equally between T1 and T2
        model
            .points
            .get_mut("T1")
            .unwrap()
            .loads_force
            .push(PointLoadForce {
                load_pat: "DEAD".into(),
                csys: "GLOBAL".into(),
                force: [lateral_force * 0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
            });
        model
            .points
            .get_mut("T2")
            .unwrap()
            .loads_force
            .push(PointLoadForce {
                load_pat: "DEAD".into(),
                csys: "GLOBAL".into(),
                force: [lateral_force * 0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
            });

        let mut nl_case = LoadCase::linear_static("NL_X", "DEAD", 1.0);
        nl_case.case_type = LoadCaseType::NonlinearStatic;
        model.load_cases.insert("NL_X".into(), nl_case);

        analyze_model(&mut model).expect("solve x-brace frame");
        model
    };

    let model_push = run_x_brace(60.0);
    let model_pull = run_x_brace(-60.0);

    let res_push = &model_push.case_results["NL_X"];
    let res_pull = &model_pull.case_results["NL_X"];

    // Base shear equilibrium
    let r_push = res_push.joint_reactions["B1"][0] + res_push.joint_reactions["B2"][0];
    let r_pull = res_pull.joint_reactions["B1"][0] + res_pull.joint_reactions["B2"][0];
    assert_close(r_push + 60.0, 0.0, 1e-9, "Base shear push equilibrium");
    assert_close(r_pull - 60.0, 0.0, 1e-9, "Base shear pull equilibrium");

    // Under push: D1 is in tension (carries lateral shear), D2 is compressed -> clamped to p >= 0
    let d1_push = res_push.frame_forces["D1"].first().unwrap().p;
    let d2_push = res_push.frame_forces["D2"].first().unwrap().p;
    assert!(d1_push > 10.0, "D1 in tension under push");
    assert!(d2_push >= -1e-9, "D2 carries zero compression under push");

    // Under pull: D2 is in tension, D1 is compressed -> clamped to p >= 0
    let d1_pull = res_pull.frame_forces["D1"].first().unwrap().p;
    let d2_pull = res_pull.frame_forces["D2"].first().unwrap().p;
    assert!(d2_pull > 10.0, "D2 in tension under pull");
    assert!(d1_pull >= -1e-9, "D1 carries zero compression under pull");

    // Exact anti-symmetric duality between push and pull
    assert_close(
        d1_push,
        d2_pull,
        1e-6,
        "D1 push tension matches D2 pull tension",
    );
    assert_close(
        d2_push,
        d1_pull,
        1e-6,
        "Zero compression matches between braces",
    );
}

// ============================================================================
// 9. Opposing Span Loads (Net W_axial = 0, but internal forces non-zero)
// ============================================================================

#[test]
fn test_opposing_span_loads_internal_limits() {
    let mut model = SapModel::default();
    model.active_dofs = [true, false, false, false, false, false];
    model
        .load_patterns
        .get_mut("DEAD")
        .unwrap()
        .self_wt_multiplier = 0.0;
    model.load_cases.clear();

    model.add_point("A", 0.0, 0.0, 0.0);
    model.add_point("B", 100.0, 0.0, 0.0);
    model.points.get_mut("A").unwrap().restraint = [true; 6];
    model.points.get_mut("B").unwrap().spring[0] = 500.0;

    let mut mat = Material::default_steel_kip_in();
    mat.name = "STEEL".into();
    mat.e = 30000.0;
    model.materials.insert("STEEL".into(), mat);

    let sec = FrameSection {
        name: "BAR".into(),
        material: "STEEL".into(),
        area: 10.0,
        ..FrameSection::default()
    };
    model.frame_sections.insert("BAR".into(), sec);

    model.add_frame("BAR1", "A", "B", "BAR");
    model.frames.get_mut("BAR1").unwrap().min_stations = 21;
    // T_lim = 100 kips, C_lim = 100 kips
    model.frames.get_mut("BAR1").unwrap().tc_limits = Some((true, 100.0, true, 100.0));

    // +2 kip/in on [0.0, 0.5] (100 kips) and -2 kip/in on [0.5, 1.0] (-100 kips) -> net W_axial = 0
    // Internal variation = 100 kips <= T_lim + C_lim (200 kips)
    model
        .frames
        .get_mut("BAR1")
        .unwrap()
        .loads_dist
        .push(FrameLoadDistributed {
            load_pat: "DEAD".into(),
            my_type: 1,
            dir: 1,
            csys: "LOCAL".into(),
            rel_dist1: 0.0,
            rel_dist2: 0.5,
            val1: 2.0,
            val2: 2.0,
        });
    model
        .frames
        .get_mut("BAR1")
        .unwrap()
        .loads_dist
        .push(FrameLoadDistributed {
            load_pat: "DEAD".into(),
            my_type: 1,
            dir: 1,
            csys: "LOCAL".into(),
            rel_dist1: 0.5,
            rel_dist2: 1.0,
            val1: -2.0,
            val2: -2.0,
        });

    let mut nl_case = LoadCase::linear_static("NL_OPP", "DEAD", 1.0);
    nl_case.case_type = LoadCaseType::NonlinearStatic;
    model.load_cases.insert("NL_OPP".into(), nl_case);

    analyze_model(&mut model).expect("solve opposing span loads");

    let res = &model.case_results["NL_OPP"];
    let r_a = res.joint_reactions["A"][0];
    let r_b = res.joint_reactions["B"][0];

    // Net applied load is 0, so R_A + R_B = 0
    assert_close(r_a + r_b, 0.0, 1e-9, "Equilibrium under net zero span load");

    // All stations must satisfy limits
    for sta in &res.frame_forces["BAR1"] {
        assert!(
            sta.p <= 100.0 + 1e-6,
            "Tension limit violated: p = {}",
            sta.p
        );
        assert!(
            sta.p >= -100.0 - 1e-6,
            "Compression limit violated: p = {}",
            sta.p
        );
    }
}

// ============================================================================
// 10. Self-Weight Column Compression Capping
// ============================================================================

#[test]
fn test_self_weight_column_compression_cutoff() {
    let mut model = SapModel::default();
    model.active_dofs = [false, false, true, false, false, false]; // 1D vertical Z
                                                                   // Enable self-weight multiplier = 1.0
    model
        .load_patterns
        .get_mut("DEAD")
        .unwrap()
        .self_wt_multiplier = 1.0;
    model.load_cases.clear();

    model.add_point("BASE", 0.0, 0.0, 0.0);
    model.add_point("TOP", 0.0, 0.0, 100.0);
    model.points.get_mut("BASE").unwrap().restraint = [true; 6];
    model.points.get_mut("TOP").unwrap().spring[2] = 200.0; // parallel vertical spring

    let mut mat = Material::default_steel_kip_in();
    mat.name = "HEAVY_MAT".into();
    mat.e = 30000.0;
    mat.unit_weight = 0.5; // 0.5 kip/in^3
    mat.unit_mass = 0.0;
    model.materials.insert("HEAVY_MAT".into(), mat);

    let sec = FrameSection {
        name: "COL_SEC".into(),
        material: "HEAVY_MAT".into(),
        area: 10.0, // weight_per_len = 10 * 0.5 = 5 kip/in -> Total column weight = 500 kips
        ..FrameSection::default()
    };
    model.frame_sections.insert("COL_SEC".into(), sec);

    model.add_frame("COL", "BASE", "TOP", "COL_SEC");
    // Compression limit = 150 kips, no tension limit
    model.frames.get_mut("COL").unwrap().tc_limits = Some((true, 150.0, false, 0.0));

    let mut nl_case = LoadCase::linear_static("NL_SW", "DEAD", 1.0);
    nl_case.case_type = LoadCaseType::NonlinearStatic;
    model.load_cases.insert("NL_SW".into(), nl_case);

    analyze_model(&mut model).expect("solve self-weight column");

    let res = &model.case_results["NL_SW"];
    let r_base = res.joint_reactions["BASE"][2];
    let r_top = res.joint_reactions["TOP"][2];

    // Total downward self-weight is 500 kips -> R_BASE + R_TOP = +500 kips
    assert_close(
        r_base + r_top,
        500.0,
        1e-8,
        "Self-weight vertical equilibrium",
    );

    // Column base reaction is capped at C_lim = 150 kips
    assert_close(
        r_base,
        150.0,
        1e-4,
        "Base reaction capped at compression limit",
    );
    // Spring carries remaining 350 kips
    assert_close(
        r_top,
        350.0,
        1e-4,
        "Top spring reaction carries remaining load",
    );

    // All stations must satisfy compression limit: P(z) >= -150.0
    for sta in &res.frame_forces["COL"] {
        assert!(
            sta.p >= -150.0 - 1e-6,
            "Station compression exceeds limit under self-weight: p = {}",
            sta.p
        );
    }
}

// ============================================================================
// 11. Multi-Spring Stiffness Dynamic Range (1e-4 to 1e8)
// ============================================================================

#[test]
fn test_spring_stiffness_dynamic_range() {
    for &ks in &[1e-3, 1.0, 1e3, 1e6, 1e9] {
        let mut model = SapModel::default();
        model.active_dofs = [true, false, false, false, false, false];
        model
            .load_patterns
            .get_mut("DEAD")
            .unwrap()
            .self_wt_multiplier = 0.0;
        model.load_cases.clear();

        model.add_point("A", 0.0, 0.0, 0.0);
        model.add_point("B", 100.0, 0.0, 0.0);
        model.points.get_mut("A").unwrap().restraint = [true; 6];
        model.points.get_mut("B").unwrap().spring[0] = ks;

        let mut mat = Material::default_steel_kip_in();
        mat.name = "STEEL".into();
        mat.e = 30000.0;
        model.materials.insert("STEEL".into(), mat);

        let sec = FrameSection {
            name: "BAR".into(),
            material: "STEEL".into(),
            area: 10.0,
            ..FrameSection::default()
        };
        model.frame_sections.insert("BAR".into(), sec);

        model.add_frame("BAR1", "A", "B", "BAR");
        model.frames.get_mut("BAR1").unwrap().tc_limits = Some((false, 0.0, true, 80.0));

        model
            .frames
            .get_mut("BAR1")
            .unwrap()
            .loads_dist
            .push(FrameLoadDistributed {
                load_pat: "DEAD".into(),
                my_type: 1,
                dir: 1,
                csys: "LOCAL".into(),
                rel_dist1: 0.0,
                rel_dist2: 1.0,
                val1: 2.0,
                val2: 2.0,
            });

        let mut nl_case = LoadCase::linear_static("NL", "DEAD", 1.0);
        nl_case.case_type = LoadCaseType::NonlinearStatic;
        model.load_cases.insert("NL".into(), nl_case);

        analyze_model(&mut model).unwrap_or_else(|_| panic!("solve with ks = {ks}"));

        let res = &model.case_results["NL"];
        let ra = res.joint_reactions["A"][0];
        let rb = res.joint_reactions["B"][0];

        // Total load = 200 kips. R_A + R_B + 200 = 0 to high precision
        assert_close(
            ra + rb + 200.0,
            0.0,
            1e-6,
            &format!("Equilibrium for ks = {ks}"),
        );
        assert_close(ra, -80.0, 1e-4, &format!("Tension cutoff for ks = {ks}"));
        assert_close(rb, -120.0, 1e-4, &format!("Spring reaction for ks = {ks}"));
    }
}

// ============================================================================
// 12. High Limit Linear Elastic Parity Match
// ============================================================================

#[test]
fn test_high_limit_linear_parity_exact() {
    let mut model = SapModel::default();
    model.active_dofs = [true, false, false, false, false, false];
    model
        .load_patterns
        .get_mut("DEAD")
        .unwrap()
        .self_wt_multiplier = 0.0;
    model.load_cases.clear();

    model.add_point("A", 0.0, 0.0, 0.0);
    model.add_point("B", 100.0, 0.0, 0.0);
    model.points.get_mut("A").unwrap().restraint = [true; 6];
    model.points.get_mut("B").unwrap().spring[0] = 500.0;

    let mut mat = Material::default_steel_kip_in();
    mat.name = "STEEL".into();
    mat.e = 30000.0;
    model.materials.insert("STEEL".into(), mat);

    let sec = FrameSection {
        name: "BAR".into(),
        material: "STEEL".into(),
        area: 10.0,
        ..FrameSection::default()
    };
    model.frame_sections.insert("BAR".into(), sec);

    model.add_frame("BAR1", "A", "B", "BAR");
    // Limits set to 1e12 (effectively infinite)
    model.frames.get_mut("BAR1").unwrap().tc_limits = Some((true, 1e12, true, 1e12));

    model
        .frames
        .get_mut("BAR1")
        .unwrap()
        .loads_dist
        .push(FrameLoadDistributed {
            load_pat: "DEAD".into(),
            my_type: 1,
            dir: 1,
            csys: "LOCAL".into(),
            rel_dist1: 0.0,
            rel_dist2: 1.0,
            val1: 2.5,
            val2: 2.5,
        });
    model
        .points
        .get_mut("B")
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: [75.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        });

    model
        .load_cases
        .insert("LIN".into(), LoadCase::linear_static("LIN", "DEAD", 1.0));
    let mut nl_case = LoadCase::linear_static("NL", "DEAD", 1.0);
    nl_case.case_type = LoadCaseType::NonlinearStatic;
    model.load_cases.insert("NL".into(), nl_case);

    analyze_model(&mut model).expect("solve high limit parity");

    let res_lin = &model.case_results["LIN"];
    let res_nl = &model.case_results["NL"];

    let u_lin = res_lin.joint_displacements["B"][0];
    let u_nl = res_nl.joint_displacements["B"][0];
    assert_close(
        u_nl,
        u_lin,
        1e-12,
        "Displacement matches linear static to 1e-12",
    );

    let ra_lin = res_lin.joint_reactions["A"][0];
    let ra_nl = res_nl.joint_reactions["A"][0];
    assert_close(
        ra_nl,
        ra_lin,
        1e-10,
        "Support reaction matches linear static to 1e-10",
    );

    let rb_lin = res_lin.joint_reactions["B"][0];
    let rb_nl = res_nl.joint_reactions["B"][0];
    assert_close(
        rb_nl,
        rb_lin,
        1e-10,
        "Spring reaction matches linear static to 1e-10",
    );
}

// ============================================================================
// 13. Portal Frame Equilibrium & Load Shedding with Varying Beam Stiffness
// ============================================================================

#[test]
fn test_portal_frame_load_shedding_across_stiffness_spectrum() {
    for &beam_i33 in &[10.0, 1e3, 1e5, 1e7] {
        let mut model = SapModel::default();
        model.active_dofs = [true, false, true, false, true, false]; // 2D X-Z frame
        model
            .load_patterns
            .get_mut("DEAD")
            .unwrap()
            .self_wt_multiplier = 0.0;
        model.load_cases.clear();

        model.add_point("BASE1", 0.0, 0.0, 0.0);
        model.add_point("TOP1", 0.0, 0.0, 100.0);
        model.add_point("BASE2", 100.0, 0.0, 0.0);
        model.add_point("TOP2", 100.0, 0.0, 100.0);

        model.points.get_mut("BASE1").unwrap().restraint = [true; 6];
        model.points.get_mut("BASE2").unwrap().restraint = [true; 6];

        let mut mat = Material::default_steel_kip_in();
        mat.name = "STEEL".into();
        mat.e = 30000.0;
        model.materials.insert("STEEL".into(), mat);

        let sec_col = FrameSection {
            name: "COL_SEC".into(),
            material: "STEEL".into(),
            area: 20.0,
            i33: 1000.0,
            i22: 1000.0,
            ..FrameSection::default()
        };
        let sec_beam = FrameSection {
            name: "BEAM_SEC".into(),
            material: "STEEL".into(),
            area: 50.0,
            i33: beam_i33,
            i22: beam_i33,
            ..FrameSection::default()
        };
        model.frame_sections.insert("COL_SEC".into(), sec_col);
        model.frame_sections.insert("BEAM_SEC".into(), sec_beam);

        model.add_frame("COL1", "BASE1", "TOP1", "COL_SEC");
        model.add_frame("COL2", "BASE2", "TOP2", "COL_SEC");
        model.add_frame("BEAM", "TOP1", "TOP2", "BEAM_SEC");

        // Column 1 compression limit = 80 kips
        model.frames.get_mut("COL1").unwrap().tc_limits = Some((true, 80.0, false, 0.0));

        // Downward load on Column 1: w = 1.0 kip/in (100 kips) + top nodal load 100 kips = 200 kips total
        model
            .frames
            .get_mut("COL1")
            .unwrap()
            .loads_dist
            .push(FrameLoadDistributed {
                load_pat: "DEAD".into(),
                my_type: 1,
                dir: 1,
                csys: "LOCAL".into(),
                rel_dist1: 0.0,
                rel_dist2: 1.0,
                val1: -1.0,
                val2: -1.0,
            });
        model
            .points
            .get_mut("TOP1")
            .unwrap()
            .loads_force
            .push(PointLoadForce {
                load_pat: "DEAD".into(),
                csys: "GLOBAL".into(),
                force: [0.0, 0.0, -100.0, 0.0, 0.0, 0.0],
            });

        let mut nl_case = LoadCase::linear_static("NL_PORTAL", "DEAD", 1.0);
        nl_case.case_type = LoadCaseType::NonlinearStatic;
        model.load_cases.insert("NL_PORTAL".into(), nl_case);

        analyze_model(&mut model)
            .unwrap_or_else(|_| panic!("solve portal frame with beam_i33 = {beam_i33}"));

        let res = &model.case_results["NL_PORTAL"];
        let r1_z = res.joint_reactions["BASE1"][2];
        let r2_z = res.joint_reactions["BASE2"][2];

        // Total vertical load = 200 kips. Sum of reactions MUST equal 200 kips
        assert_close(
            r1_z + r2_z,
            200.0,
            1e-8,
            &format!("Vertical equilibrium for beam_i33 = {beam_i33}"),
        );

        // Column 1 vertical base reaction must NOT exceed 80 kips
        assert!(
            r1_z <= 80.0 + 1e-3,
            "Column 1 base reaction exceeds 80 kips for beam_i33 {beam_i33}: r1_z = {r1_z}"
        );
        // Column 2 must carry whatever is shed
        assert!(
            r2_z >= 120.0 - 1e-3,
            "Column 2 does not carry shed load for beam_i33 {beam_i33}: r2_z = {r2_z}"
        );
    }
}
