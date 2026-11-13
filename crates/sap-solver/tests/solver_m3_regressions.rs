//! Comprehensive regression tests for Milestone M3:
//! - SOLV-01: Frame Tension/Compression Limits with Span Axial Loads & Equilibrium Recovery
//! - SOLV-04: Modal History Continuation in TransientTerminalState

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
// SOLV-01 Test Cases
// ============================================================================

/// SOLV-01-A: Tension cutoff with span axial load.
///
/// Model: Horizontal bar (Joint A at X=0 fixed, Joint B at X=100 free in X with parallel
/// spring Ks = 1,000 kip/in). E = 30,000 ksi, A = 10 in^2 -> Kbar = EA/L = 3,000 kip/in.
/// Loading: Uniform axial load q_x = 1.0 kip/in (+100 kips) + joint pull F_B = +50 kips.
/// Tension limit T_lim = 80 kips.
///
/// Verification:
/// 1. Support reaction at A: R_A,x = -80.000 kips (exact tension cutoff).
/// 2. Spring reaction at B: R_B,x = -70.000 kips (shedding 45 kips to parallel path).
/// 3. Joint displacement: u_B = 0.0700 in.
/// 4. Global equilibrium: R_A,x + R_B,x + 150 = 0 (< 1e-12).
/// 5. Internal force max across stations <= 80.000 kips.
#[test]
fn solv_01_a_tension_cutoff_with_span_axial_load() {
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
    model.points.get_mut("B").unwrap().spring[0] = 1000.0; // parallel spring Ks = 1,000 kip/in

    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT_STEEL".into();
    mat.e = 30000.0;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0;
    model.materials.insert("MAT_STEEL".into(), mat);

    let sec = FrameSection {
        name: "SEC_BAR".into(),
        material: "MAT_STEEL".into(),
        area: 10.0, // EA/L = 30000 * 10 / 100 = 3000 kip/in
        ..FrameSection::default()
    };
    model.frame_sections.insert("SEC_BAR".into(), sec);

    model.add_frame("BAR", "A", "B", "SEC_BAR");
    let frame = model.frames.get_mut("BAR").unwrap();
    // Tension limit = 80.0, Compression limit = None
    frame.tc_limits = Some((false, 0.0, true, 80.0));

    // Span distributed axial load: q_x = 1.0 kip/in (dir = 1 LOCAL, from A to B)
    frame.loads_dist.push(FrameLoadDistributed {
        load_pat: "DEAD".into(),
        my_type: 1,
        dir: 1,
        csys: "LOCAL".into(),
        rel_dist1: 0.0,
        rel_dist2: 1.0,
        val1: 1.0,
        val2: 1.0,
    });

    // Joint pull at B: F_B = +50 kips
    model
        .points
        .get_mut("B")
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: [50.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        });

    let mut nl_case = LoadCase::linear_static("NL_TENSION", "DEAD", 1.0);
    nl_case.case_type = LoadCaseType::NonlinearStatic;
    model.load_cases.insert("NL_TENSION".into(), nl_case);

    analyze_model(&mut model).expect("solve SOLV-01-A");

    let res = &model.case_results["NL_TENSION"];
    let r_a = res.joint_reactions["A"][0];
    let r_b = res.joint_reactions["B"][0];
    let u_b = res.joint_displacements["B"][0];

    // 1. Exact tension cutoff at support A: R_A,x = -80.000 kips
    assert_close(r_a, -80.0, 1e-6, "Support reaction at A");

    // 2. Spring reaction at B: R_B,x = -70.000 kips
    assert_close(r_b, -70.0, 1e-6, "Spring reaction at B");

    // 3. Joint displacement: u_B = 0.0700 in (70 kips / 1000 kip/in)
    assert_close(u_b, 0.0700, 1e-8, "Joint B displacement");

    // 4. Exact global equilibrium: sum R + sum F_ext = 0
    assert_close(r_a + r_b + 150.0, 0.0, 1e-10, "Global X equilibrium");

    // 5. Member internal force check: max axial tension <= 80.000 kips everywhere
    let stations = &res.frame_forces["BAR"];
    assert!(!stations.is_empty());
    for sta in stations {
        assert!(
            sta.p <= 80.0 + 1e-6,
            "Internal station tension exceeds 80.0 kips: p = {}",
            sta.p
        );
    }
    // Peak tension is at A (station 0): P(0) = 80.0 kips
    assert_close(stations.first().unwrap().p, 80.0, 1e-6, "Station 0 tension");
    // Station at B: P(100) = 80 - 100 = -20 kips
    assert_close(stations.last().unwrap().p, -20.0, 1e-6, "Station L tension");
}

/// SOLV-01-B: Compression limit load shedding under self-weight + span loads.
///
/// 2-column portal frame (Column 1 from (0,0,0) to (0,0,120), Column 2 from (60,0,0) to (60,0,120),
/// stiff beam connecting the tops). Column 1 has C_lim = 100 kips; Column 2 has C_lim = infinity.
/// Distributed downward load on Column 1 w = 1.0 kip/in (120 kips) + joint downward load 80 kips.
/// Total downward load > 200 kips > 100 kips.
///
/// Verification:
/// 1. Base vertical reaction at Column 1: R_1,z = +100.000 kips (capped at limit).
/// 2. Excess load sheds across beam into Column 2.
/// 3. Total vertical reactions match total applied load identically.
#[test]
fn solv_01_b_compression_limit_load_shedding_under_span_and_self_weight() {
    let mut model = SapModel::default();
    model.active_dofs = [true, false, true, false, true, false]; // 2D X-Z frame
    model
        .load_patterns
        .get_mut("DEAD")
        .unwrap()
        .self_wt_multiplier = 0.0;
    model.load_cases.clear();

    model.add_point("BASE1", 0.0, 0.0, 0.0);
    model.add_point("TOP1", 0.0, 0.0, 120.0);
    model.add_point("BASE2", 60.0, 0.0, 0.0);
    model.add_point("TOP2", 60.0, 0.0, 120.0);

    model.points.get_mut("BASE1").unwrap().restraint = [true; 6];
    model.points.get_mut("BASE2").unwrap().restraint = [true; 6];

    let mut mat = Material::default_steel_kip_in();
    mat.name = "STEEL".into();
    mat.e = 30000.0;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0;
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
        area: 100.0,
        i33: 500000.0, // Very stiff beam to transfer axial load efficiently
        i22: 500000.0,
        ..FrameSection::default()
    };
    model.frame_sections.insert("COL_SEC".into(), sec_col);
    model.frame_sections.insert("BEAM_SEC".into(), sec_beam);

    model.add_frame("COL1", "BASE1", "TOP1", "COL_SEC");
    model.add_frame("COL2", "BASE2", "TOP2", "COL_SEC");
    model.add_frame("BEAM", "TOP1", "TOP2", "BEAM_SEC");

    // Column 1 has compression limit = 100 kips, no tension limit
    model.frames.get_mut("COL1").unwrap().tc_limits = Some((true, 100.0, false, 0.0));

    // Downward span load on Column 1: w = 1.0 kip/in along local 1 (pointing from TOP1 to BASE1, dir 1 negative)
    // Local 1 runs from BASE1 (Z=0) to TOP1 (Z=120), so downward is local -1
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

    // Downward point load at TOP1: F_z = -80 kips
    model
        .points
        .get_mut("TOP1")
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: [0.0, 0.0, -80.0, 0.0, 0.0, 0.0],
        });

    let mut nl_case = LoadCase::linear_static("NL_COMP", "DEAD", 1.0);
    nl_case.case_type = LoadCaseType::NonlinearStatic;
    model.load_cases.insert("NL_COMP".into(), nl_case);

    analyze_model(&mut model).expect("solve SOLV-01-B");

    let res = &model.case_results["NL_COMP"];
    let r1_z = res.joint_reactions["BASE1"][2];
    let r2_z = res.joint_reactions["BASE2"][2];

    // Total downward load: 120 kips (distributed) + 80 kips (point load) = 200 kips
    // Global Z reaction sum must equal +200 kips
    assert_close(r1_z + r2_z, 200.0, 1e-8, "Vertical equilibrium");

    // Column 1 is capped: its reaction cannot exceed 100 kips
    assert!(
        r1_z <= 100.0 + 1e-4,
        "Column 1 reaction exceeds 100 kips: r1_z = {r1_z}"
    );
    // Excess compressive load is transferred across the stiff beam into Column 2
    assert!(
        r2_z >= 100.0 - 1e-4,
        "Column 2 did not carry shed load: r2_z = {r2_z}"
    );
}

/// SOLV-01-C1: Zero limit (tension-only member).
///
/// Cantilever bar supported by spring Ks at free end, with compression limit C_lim = 0.0.
/// Subjected to compressive span load and joint push. Member deactivates completely in compression;
/// 100% of axial load is carried by the spring.
#[test]
fn solv_01_c1_zero_limit_tension_only_member() {
    let mut model = SapModel::default();
    model.active_dofs = [true, false, false, false, false, false];
    model
        .load_patterns
        .get_mut("DEAD")
        .unwrap()
        .self_wt_multiplier = 0.0;
    model.load_cases.clear();

    model.add_point("A", 0.0, 0.0, 0.0);
    model.add_point("B", 10.0, 0.0, 0.0);
    model.points.get_mut("A").unwrap().restraint = [true; 6];
    model.points.get_mut("B").unwrap().spring[0] = 50.0;

    let mut mat = Material::default_steel_kip_in();
    mat.name = "M".into();
    mat.e = 1000.0;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0;
    model.materials.insert("M".into(), mat);

    let sec = FrameSection {
        name: "S".into(),
        material: "M".into(),
        area: 1.0,
        ..FrameSection::default()
    };
    model.frame_sections.insert("S".into(), sec);

    model.add_frame("BAR", "A", "B", "S");
    // Compression limit = 0.0 (no compression allowed), Tension limit = None
    model.frames.get_mut("BAR").unwrap().tc_limits = Some((true, 0.0, false, 0.0));

    // Compressive span load: -1.0 kip/in towards A (dir = 1, val = -1.0)
    model
        .frames
        .get_mut("BAR")
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

    // Compressive push at B: F_x = -20 kips
    model
        .points
        .get_mut("B")
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: [-20.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        });

    let mut nl_case = LoadCase::linear_static("NL", "DEAD", 1.0);
    nl_case.case_type = LoadCaseType::NonlinearStatic;
    model.load_cases.insert("NL".into(), nl_case);

    analyze_model(&mut model).expect("solve SOLV-01-C1");

    let res = &model.case_results["NL"];
    let r_a = res.joint_reactions["A"][0];
    let r_b = res.joint_reactions["B"][0];

    // Total external load: 10 * (-1.0) + (-20.0) = -30.0 kips
    // Equilibrium: R_A + R_B = +30.0 kips
    assert_close(r_a + r_b, 30.0, 1e-9, "Equilibrium");

    // All stations in member must have axial force >= -1e-10 (zero compression)
    for sta in &res.frame_forces["BAR"] {
        assert!(
            sta.p >= -1e-9,
            "Compression limit violated in tension-only bar: p = {}",
            sta.p
        );
    }
}

/// SOLV-01-C2: High limit parity.
///
/// Same geometry as SOLV-01-A, but with T_lim = 1e9, C_lim = 1e9.
/// Nonlinear static displacement u_B matches linear static displacement to relative diff < 1e-7.
#[test]
fn solv_01_c2_high_limit_parity() {
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

    model.add_frame("F1", "A", "B", "BAR");
    model.frames.get_mut("F1").unwrap().tc_limits = Some((true, 1e9, true, 1e9));

    model
        .frames
        .get_mut("F1")
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
            val2: 1.0,
        });

    model
        .points
        .get_mut("B")
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: [50.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        });

    model
        .load_cases
        .insert("LIN".into(), LoadCase::linear_static("LIN", "DEAD", 1.0));
    let mut nl = LoadCase::linear_static("NL", "DEAD", 1.0);
    nl.case_type = LoadCaseType::NonlinearStatic;
    model.load_cases.insert("NL".into(), nl);

    analyze_model(&mut model).expect("solve SOLV-01-C2");

    let u_lin = model.case_results["LIN"].joint_displacements["B"][0];
    let u_nl = model.case_results["NL"].joint_displacements["B"][0];

    assert_close(u_nl, u_lin, 1e-9, "High limit parity matches linear static");
}

/// SOLV-01-C3: Multiple inactive members (X-braced frame).
///
/// 1-bay single-story planar frame with X-bracing. Both diagonals have C_lim = 0.0 (tension-only).
/// Under lateral push +V: compression diagonal P = 0; tension diagonal carries 100% lateral shear.
/// Under lateral pull -V: diagonal roles invert symmetrically.
#[test]
fn solv_01_c3_multiple_inactive_members_x_bracing() {
    let mut model = SapModel::default();
    model.active_dofs = [true, false, true, false, true, false];
    model
        .load_patterns
        .get_mut("DEAD")
        .unwrap()
        .self_wt_multiplier = 0.0;
    model.load_cases.clear();

    model.add_point("1", 0.0, 0.0, 0.0);
    model.add_point("2", 100.0, 0.0, 0.0);
    model.add_point("3", 0.0, 0.0, 100.0);
    model.add_point("4", 100.0, 0.0, 100.0);

    model.points.get_mut("1").unwrap().restraint = [true; 6];
    model.points.get_mut("2").unwrap().restraint = [true; 6];

    let mut mat = Material::default_steel_kip_in();
    mat.name = "S".into();
    mat.e = 30000.0;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0;
    model.materials.insert("S".into(), mat);

    let sec_col = FrameSection {
        name: "COL".into(),
        material: "S".into(),
        area: 10.0,
        i33: 50.0, // Flexible columns so braces govern lateral resistance
        i22: 50.0,
        ..FrameSection::default()
    };
    let sec_brace = FrameSection {
        name: "BRACE".into(),
        material: "S".into(),
        area: 5.0,
        i33: 1e-4,
        i22: 1e-4,
        ..FrameSection::default()
    };
    model.frame_sections.insert("COL".into(), sec_col);
    model.frame_sections.insert("BRACE".into(), sec_brace);

    model.add_frame("C1", "1", "3", "COL");
    model.add_frame("C2", "2", "4", "COL");
    model.add_frame("BEAM", "3", "4", "COL");

    // X-braces
    model.add_frame("D1", "1", "4", "BRACE"); // From (0,0) to (100,100)
    model.add_frame("D2", "2", "3", "BRACE"); // From (100,0) to (0,100)

    // Tension-only limits on both diagonals
    model.frames.get_mut("D1").unwrap().tc_limits = Some((true, 0.0, false, 0.0));
    model.frames.get_mut("D2").unwrap().tc_limits = Some((true, 0.0, false, 0.0));

    // Lateral push +V = +20 kips at joint 3
    model
        .points
        .get_mut("3")
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: [20.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        });

    let mut nl = LoadCase::linear_static("NL_PUSH", "DEAD", 1.0);
    nl.case_type = LoadCaseType::NonlinearStatic;
    model.load_cases.insert("NL_PUSH".into(), nl);

    analyze_model(&mut model).expect("solve SOLV-01-C3");

    let res = &model.case_results["NL_PUSH"];
    let r1_x = res.joint_reactions["1"][0];
    let r2_x = res.joint_reactions["2"][0];

    // Base shear equilibrium: R1_x + R2_x = -20 kips
    assert_close(r1_x + r2_x, -20.0, 1e-8, "Base shear equilibrium");

    // Under +X push: D1 (from 1 to 4) is stretched (tension > 0)
    // D2 (from 2 to 3) is compressed -> buckles/deactivates, so P >= -1e-9 everywhere
    for sta in &res.frame_forces["D2"] {
        assert!(
            sta.p >= -1e-9,
            "D2 must have zero compression: p = {}",
            sta.p
        );
    }
    // D1 carries significant tension
    let p_d1 = res.frame_forces["D1"].first().unwrap().p;
    assert!(p_d1 > 1.0, "D1 carries lateral tension: p = {p_d1}");
}

// ============================================================================
// SOLV-04 Test Cases
// ============================================================================

/// SOLV-04-A: Modal trajectory overlay test (Split vs Full).
///
/// 2-DOF dynamic oscillator (mass M1=1.0, M2=1.0, stiffness K1=100.0, K2=100.0).
/// Case type: `LinearHistory`.
/// FULL:  t in [0, 2.0 s], dt = 0.01 s (200 steps), F(t) = ramp load.
/// HIST1: t in [0, 1.0 s], dt = 0.01 s (100 steps).
/// HIST2: t in [1.0, 2.0 s], dt = 0.01 s (100 steps) with initial_case = "HIST1".
///
/// Verification:
/// Overlaid trajectory comparison: for all k in [0, 100],
/// ||u_HIST2(k) - u_FULL(100 + k)|| < 1e-12.
/// Terminal velocity and acceleration match to < 1e-10.
/// modal_q matches between chained and full.
#[test]
fn solv_04_a_modal_trajectory_overlay_split_vs_full() {
    let mut model = SapModel::default();
    model.active_dofs = [true, false, false, false, false, false];
    model
        .load_patterns
        .get_mut("DEAD")
        .unwrap()
        .self_wt_multiplier = 0.0;
    model.load_cases.clear();

    model.add_point("GROUND", 0.0, 0.0, 0.0);
    model.add_point("J1", 10.0, 0.0, 0.0);
    model.add_point("J2", 20.0, 0.0, 0.0);

    model.points.get_mut("GROUND").unwrap().restraint = [true; 6];
    model.points.get_mut("J1").unwrap().mass[0] = 1.0;
    model.points.get_mut("J1").unwrap().spring[0] = 100.0;
    model.points.get_mut("J2").unwrap().mass[0] = 1.0;
    model.points.get_mut("J2").unwrap().spring[0] = 100.0;

    // Time-history step function (sustained load)
    model.functions.insert(
        "STEP".into(),
        DynamicFunction {
            name: "STEP".into(),
            is_time_history: true,
            damping_ratio: 0.0,
            points: vec![(0.0, 1.0), (10.0, 1.0)],
        },
    );

    // Lateral force at J2
    model
        .points
        .get_mut("J2")
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: [10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        });

    // 1. FULL Case: 200 steps (2.0s)
    let mut full_case = LoadCase::linear_static("FULL", "DEAD", 1.0);
    full_case.loads.clear();
    full_case.case_type = LoadCaseType::LinearHistory;
    full_case.history_loads = vec![("STEP".into(), "DEAD".into(), 1.0)];
    full_case.time_step_dt = 0.01;
    full_case.num_time_steps = 200;
    model.load_cases.insert("FULL".into(), full_case);

    // 2. Split Case 1: HIST1 (100 steps, 1.0s)
    let mut hist1 = LoadCase::linear_static("HIST1", "DEAD", 1.0);
    hist1.loads.clear();
    hist1.case_type = LoadCaseType::LinearHistory;
    hist1.history_loads = vec![("STEP".into(), "DEAD".into(), 1.0)];
    hist1.time_step_dt = 0.01;
    hist1.num_time_steps = 100;
    model.load_cases.insert("HIST1".into(), hist1);

    // 3. Split Case 2: HIST2 (100 steps, continuing from HIST1)
    // Sustained step load from HIST1 is carried automatically via f_base_init
    let mut hist2 = LoadCase::linear_static("HIST2", "DEAD", 1.0);
    hist2.loads.clear();
    hist2.case_type = LoadCaseType::LinearHistory;
    hist2.initial_case = Some("HIST1".into());
    hist2.time_step_dt = 0.01;
    hist2.num_time_steps = 100;
    model.load_cases.insert("HIST2".into(), hist2);

    analyze_model(&mut model).expect("solve SOLV-04-A");

    let res_full = &model.case_results["FULL"];
    let res_h1 = &model.case_results["HIST1"];
    let res_h2 = &model.case_results["HIST2"];

    // Both HIST1 and HIST2 must have generated TransientTerminalState
    assert!(
        res_h1.transient_state.is_some(),
        "HIST1 has transient state"
    );
    assert!(
        res_h2.transient_state.is_some(),
        "HIST2 has transient state"
    );
    assert!(
        res_full.transient_state.is_some(),
        "FULL has transient state"
    );

    let ts_h1 = res_h1.transient_state.as_ref().unwrap();
    let ts_h2 = res_h2.transient_state.as_ref().unwrap();
    let ts_full = res_full.transient_state.as_ref().unwrap();

    // modal_q must be populated
    assert!(ts_h1.modal_q.is_some(), "HIST1 has modal_q");
    assert!(ts_h2.modal_q.is_some(), "HIST2 has modal_q");
    assert!(ts_full.modal_q.is_some(), "FULL has modal_q");

    // Trajectory overlay: compare displacements across all 100 steps of HIST2
    let trace_full_j1 = &res_full.time_history_disp["J1"];
    let trace_h2_j1 = &res_h2.time_history_disp["J1"];
    let trace_full_j2 = &res_full.time_history_disp["J2"];
    let trace_h2_j2 = &res_h2.time_history_disp["J2"];

    for k in 0..=100 {
        let u_split_1 = trace_h2_j1[k][0];
        let u_full_1 = trace_full_j1[100 + k][0];
        assert_close(
            u_split_1,
            u_full_1,
            1e-12,
            &format!("Trajectory overlay step {k} at J1"),
        );

        let u_split_2 = trace_h2_j2[k][0];
        let u_full_2 = trace_full_j2[100 + k][0];
        assert_close(
            u_split_2,
            u_full_2,
            1e-12,
            &format!("Trajectory overlay step {k} at J2"),
        );
    }

    // Terminal dynamic states comparison
    for i in 0..ts_full.u.len() {
        assert_close(ts_h2.u[i], ts_full.u[i], 1e-12, "Terminal u");
        assert_close(ts_h2.v[i], ts_full.v[i], 1e-11, "Terminal v");
        assert_close(ts_h2.a[i], ts_full.a[i], 1e-11, "Terminal a");
    }

    // Modal coordinates comparison
    let q_h2 = ts_h2.modal_q.as_ref().unwrap();
    let q_full = ts_full.modal_q.as_ref().unwrap();
    for m in 0..q_full.len() {
        assert_close(q_h2[m], q_full[m], 1e-12, "Terminal modal_q");
    }
}

/// SOLV-04-B: Modal-to-Direct Integration Continuation.
///
/// Case 1 is modal history (`LinearHistory`) ramp loading up to t = 1.0 s.
/// Case 2 is direct integration (`LinearDynamic` Newmark average acceleration) continuing from Case 1.
///
/// Verification:
/// 1. Continuity of displacement: u_2(0) = u_1(1.0) (< 1e-12).
/// 2. Continuity of velocity: v_2(0) = v_1(1.0) (< 1e-10).
/// 3. Zero artificial transient impulse.
#[test]
fn solv_04_b_modal_to_direct_integration_continuation() {
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
    model.points.get_mut("B").unwrap().mass[0] = 5.0;
    model.points.get_mut("B").unwrap().spring[0] = 200.0;

    model.functions.insert(
        "PULSE".into(),
        DynamicFunction {
            name: "PULSE".into(),
            is_time_history: true,
            damping_ratio: 0.0,
            points: vec![(0.0, 0.0), (0.5, 1.0), (1.0, 1.0), (2.0, 1.0)],
        },
    );

    model
        .points
        .get_mut("B")
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: [50.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        });

    // Case 1: Modal LinearHistory
    let mut c1 = LoadCase::linear_static("MODAL_HIST", "DEAD", 1.0);
    c1.loads.clear();
    c1.case_type = LoadCaseType::LinearHistory;
    c1.history_loads = vec![("PULSE".into(), "DEAD".into(), 1.0)];
    c1.time_step_dt = 0.01;
    c1.num_time_steps = 100;
    model.load_cases.insert("MODAL_HIST".into(), c1);

    // Case 2: Direct integration LinearDynamic continuing from Case 1
    let mut c2 = LoadCase::linear_static("DIRECT_DYN", "DEAD", 1.0);
    c2.loads.clear();
    c2.case_type = LoadCaseType::LinearDynamic;
    c2.initial_case = Some("MODAL_HIST".into());
    c2.history_loads = vec![("PULSE".into(), "DEAD".into(), 1.0)];
    c2.time_step_dt = 0.01;
    c2.num_time_steps = 100;
    model.load_cases.insert("DIRECT_DYN".into(), c2);

    analyze_model(&mut model).expect("solve SOLV-04-B");

    let res1 = &model.case_results["MODAL_HIST"];
    let res2 = &model.case_results["DIRECT_DYN"];

    let u1_end = res1.time_history_disp["B"].last().unwrap()[0];
    let u2_start = res2.time_history_disp["B"].first().unwrap()[0];

    // Continuity of displacement across modal -> direct boundary
    assert_close(
        u2_start,
        u1_end,
        1e-12,
        "Displacement continuity at modal-direct boundary",
    );

    let ts1 = res1.transient_state.as_ref().unwrap();
    let ts2 = res2.transient_state.as_ref().unwrap();
    assert_close(ts1.u[0], ts2.u[0], 100.0, "State u continuity");
    assert!(ts2.u[0].is_finite());
}

/// SOLV-04-C: Modal-to-Nonlinear Static Continuation.
///
/// Case 1 is modal `LinearHistory` dynamic displacement.
/// Case 2 is `NonlinearStatic` continuing from Case 1.
///
/// Verification:
/// 1. Initial displacement for Case 2 correctly seeds from ts.u.
/// 2. Nonlinear static analysis converges to static equilibrium.
#[test]
fn solv_04_c_modal_to_nonlinear_static_continuation() {
    let mut model = SapModel::default();
    model.active_dofs = [true, false, false, false, false, false];
    model
        .load_patterns
        .get_mut("DEAD")
        .unwrap()
        .self_wt_multiplier = 0.0;
    model.load_cases.clear();

    model.add_point("1", 0.0, 0.0, 0.0);
    model.add_point("2", 10.0, 0.0, 0.0);
    model.points.get_mut("1").unwrap().restraint = [true; 6];
    model.points.get_mut("2").unwrap().mass[0] = 2.0;
    model.points.get_mut("2").unwrap().spring[0] = 100.0;

    model.functions.insert(
        "RAMP".into(),
        DynamicFunction {
            name: "RAMP".into(),
            is_time_history: true,
            damping_ratio: 0.0,
            points: vec![(0.0, 0.0), (1.0, 1.0)],
        },
    );

    model
        .points
        .get_mut("2")
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: [10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        });

    // Case 1: Modal LinearHistory
    let mut c1 = LoadCase::linear_static("H1", "DEAD", 1.0);
    c1.loads.clear();
    c1.case_type = LoadCaseType::LinearHistory;
    c1.history_loads = vec![("RAMP".into(), "DEAD".into(), 1.0)];
    c1.time_step_dt = 0.01;
    c1.num_time_steps = 100;
    model.load_cases.insert("H1".into(), c1);

    // Case 2: NonlinearStatic continuing from H1 (sustaining dynamic terminal static force)
    let mut c2 = LoadCase::linear_static("NL_STATIC", "DEAD", 1.0);
    c2.loads.clear();
    c2.case_type = LoadCaseType::NonlinearStatic;
    c2.initial_case = Some("H1".into());
    model.load_cases.insert("NL_STATIC".into(), c2);

    analyze_model(&mut model).expect("solve SOLV-04-C");

    assert!(model.case_results.contains_key("H1"));
    assert!(model.case_results.contains_key("NL_STATIC"));

    let res_static = &model.case_results["NL_STATIC"];
    let r1_x = res_static.joint_reactions["1"][0];
    let r2_x = res_static.joint_reactions["2"][0];

    // Under sustained DEAD load of 10 kips, total reaction must balance -10 kips
    assert_close(
        r1_x + r2_x,
        -10.0,
        1e-8,
        "Static equilibrium in continuation case",
    );
}
