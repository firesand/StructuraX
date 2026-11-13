//! Empirical Challenger Adversarial Test Suite for Milestone M2:
//! DSGN-01: Concrete Beam Minor Flexure (M2) under diverse loading,
//! boundary, overstress, aspect ratio, and unit conditions.

use sap_design::{
    concrete::{design_aci_beam_flexure, design_ec2_beam},
    try_run_concrete_design,
};
use sap_model::{CaseResults, ConcreteRebarConfig, FrameStationResult, SapModel};
use sap_section::make_rectangular;
use sap_units::Units;

/// Helper to build a concrete beam model with a specified cross-section and unit system.
fn create_test_beam_model(
    code: &str,
    units: Units,
    section_name: &str,
    h: f64,
    b: f64,
    cover: Option<f64>,
) -> (SapModel, String) {
    let mut model = SapModel::new(units);
    model.concrete_design_code = code.to_string();

    let mut sec = make_rectangular(section_name, "4000Psi", h, b);
    if let Some(c) = cover {
        sec.concrete_rebar = Some(ConcreteRebarConfig {
            is_column: false,
            rebar_mat_long: "4000Psi".to_string(),
            rebar_mat_confine: "4000Psi".to_string(),
            cover_top: c,
            cover_bot: c,
            num_bars_3dir: 3,
            num_bars_2dir: 3,
            total_rebar_area: 0.0,
        });
    }
    model.frame_sections.insert(sec.name.clone(), sec);

    let span = 120.0 * Units::KipInF.length_factor_to(units);
    model.add_point("N1", 0.0, 0.0, 0.0);
    model.add_point("N2", span, 0.0, 0.0);
    model.add_frame("BM1", "N1", "N2", section_name);

    (model, "BM1".to_string())
}

// =========================================================================
// 1. PURE MINOR FLEXURE (M2 >> 0, M3 = 0) UNDER ACI 318-14 & EC2
// =========================================================================

#[test]
fn test_adversarial_pure_minor_flexure_aci_and_ec2() {
    for code in ["ACI 318-14", "Eurocode 2-2004"] {
        // Test both positive and negative M2
        for &m2_val in &[1400.0, -1400.0] {
            let (mut model, frame_name) =
                create_test_beam_model(code, Units::KipInF, "B24x14", 24.0, 14.0, Some(2.5));

            let mut cr = CaseResults {
                case_name: "COMBO_M2".into(),
                ..Default::default()
            };
            cr.frame_forces.insert(
                frame_name.clone(),
                vec![FrameStationResult {
                    station: 60.0,
                    rel_dist: 0.5,
                    m3: 0.0,
                    m2: m2_val,
                    ..Default::default()
                }],
            );
            model.case_results.insert("COMBO_M2".into(), cr);

            try_run_concrete_design(&mut model).expect("design must succeed");
            let res = &model.concrete_design_results[&frame_name];

            assert_eq!(
                res.status, "OK",
                "code {code}, m2={m2_val} should be within normal capacity"
            );
            assert!(
                res.as_side > 0.0,
                "code {code}, m2={m2_val} must produce as_side > 0"
            );
            assert_eq!(
                res.as_top, 0.0,
                "code {code}, m2={m2_val} must have as_top == 0 when M3 == 0"
            );
            assert_eq!(
                res.as_bot, 0.0,
                "code {code}, m2={m2_val} must have as_bot == 0 when M3 == 0"
            );
            assert_eq!(
                res.pmm_ratio, 0.0,
                "code {code} concrete beam pmm_ratio must remain 0.0"
            );

            // Independent Oracle Verification:
            // For B24x14: width in minor bending is h = 24.0 in, depth is b = 14.0 in.
            // With cover = 2.5 in, effective depth d_2 = 14.0 - 2.5 = 11.5 in.
            if code == "ACI 318-14" {
                let (oracle_as, _, over) = design_aci_beam_flexure(
                    1400.0,
                    24.0,
                    11.5,
                    2.5,
                    4.0,
                    60.0,
                    29000.0,
                    Units::KipInF,
                );
                assert!(!over);
                let actual = res.as_side;
                assert!(
                    (actual - oracle_as).abs() < 1e-9,
                    "ACI as_side {actual} must match analytical oracle {oracle_as}"
                );
            } else {
                let (oracle_as, _, _, over) =
                    design_ec2_beam(1400.0, 0.0, 24.0, 11.5, 2.5, 4.0, 60.0, 60.0, Units::KipInF);
                assert!(!over);
                let actual = res.as_side;
                assert!(
                    (actual - oracle_as).abs() < 1e-9,
                    "EC2 as_side {actual} must match analytical oracle {oracle_as}"
                );
            }
        }
    }
}

// =========================================================================
// 2. BIAXIAL FLEXURE: INDEPENDENT SIZING OF As_top, As_bot, and As_side
// =========================================================================

#[test]
fn test_adversarial_biaxial_reinforcement_isolation() {
    for code in ["ACI 318-14", "Eurocode 2-2004"] {
        // Step A: Pure major positive bending M3 = 2000 kip-in
        let (mut model_m3, frame_m3) =
            create_test_beam_model(code, Units::KipInF, "B24x14", 24.0, 14.0, Some(2.5));
        let mut cr_m3 = CaseResults {
            case_name: "M3_ONLY".into(),
            ..Default::default()
        };
        cr_m3.frame_forces.insert(
            frame_m3.clone(),
            vec![FrameStationResult {
                station: 60.0,
                rel_dist: 0.5,
                m3: 2000.0,
                m2: 0.0,
                ..Default::default()
            }],
        );
        model_m3.case_results.insert("M3_ONLY".into(), cr_m3);
        try_run_concrete_design(&mut model_m3).unwrap();
        let baseline_as_bot = model_m3.concrete_design_results[&frame_m3].as_bot;
        let baseline_as_top = model_m3.concrete_design_results[&frame_m3].as_top;
        assert!(baseline_as_bot > 0.0);
        assert_eq!(baseline_as_top, 0.0);

        // Step B: Pure minor bending M2 = 1100 kip-in
        let (mut model_m2, frame_m2) =
            create_test_beam_model(code, Units::KipInF, "B24x14", 24.0, 14.0, Some(2.5));
        let mut cr_m2 = CaseResults {
            case_name: "M2_ONLY".into(),
            ..Default::default()
        };
        cr_m2.frame_forces.insert(
            frame_m2.clone(),
            vec![FrameStationResult {
                station: 60.0,
                rel_dist: 0.5,
                m3: 0.0,
                m2: 1100.0,
                ..Default::default()
            }],
        );
        model_m2.case_results.insert("M2_ONLY".into(), cr_m2);
        try_run_concrete_design(&mut model_m2).unwrap();
        let baseline_as_side = model_m2.concrete_design_results[&frame_m2].as_side;
        assert!(baseline_as_side > 0.0);

        // Step C: Concurrent Biaxial flexure M3 = 2000, M2 = 1100 + Shears V2 = 35, V3 = 20
        let (mut model_bi, frame_bi) =
            create_test_beam_model(code, Units::KipInF, "B24x14", 24.0, 14.0, Some(2.5));
        let mut cr_bi = CaseResults {
            case_name: "BIAXIAL".into(),
            ..Default::default()
        };
        cr_bi.frame_forces.insert(
            frame_bi.clone(),
            vec![FrameStationResult {
                station: 60.0,
                rel_dist: 0.5,
                m3: 2000.0,
                m2: 1100.0,
                v2: 35.0,
                v3: 20.0,
                ..Default::default()
            }],
        );
        model_bi.case_results.insert("BIAXIAL".into(), cr_bi);
        try_run_concrete_design(&mut model_bi).unwrap();
        let res_bi = &model_bi.concrete_design_results[&frame_bi];

        // Strict isolation: biaxial must preserve exact uncoupled flexural reinforcement
        assert!(
            (res_bi.as_bot - baseline_as_bot).abs() < 1e-9,
            "code {code}: as_bot under biaxial must equal pure major as_bot"
        );
        assert!(
            (res_bi.as_side - baseline_as_side).abs() < 1e-9,
            "code {code}: as_side under biaxial must equal pure minor as_side"
        );
        assert_eq!(
            res_bi.as_top, 0.0,
            "code {code}: as_top must remain 0 when M3 is positive sagging moment"
        );
        assert!(
            res_bi.av_over_s > 0.0,
            "code {code}: shear stirrups must be sized"
        );

        // Step D: Negative major moment M3 = -1800 (hogging) + M2 = 1100
        let (mut model_neg, frame_neg) =
            create_test_beam_model(code, Units::KipInF, "B24x14", 24.0, 14.0, Some(2.5));
        let mut cr_neg = CaseResults {
            case_name: "NEG_M3".into(),
            ..Default::default()
        };
        cr_neg.frame_forces.insert(
            frame_neg.clone(),
            vec![FrameStationResult {
                station: 60.0,
                rel_dist: 0.5,
                m3: -1800.0,
                m2: 1100.0,
                ..Default::default()
            }],
        );
        model_neg.case_results.insert("NEG_M3".into(), cr_neg);
        try_run_concrete_design(&mut model_neg).unwrap();
        let res_neg = &model_neg.concrete_design_results[&frame_neg];

        assert!(
            res_neg.as_top > 0.0,
            "code {code}: as_top must be > 0 for negative M3"
        );
        assert_eq!(
            res_neg.as_bot, 0.0,
            "code {code}: as_bot must be 0 for pure negative M3"
        );
        assert!(
            (res_neg.as_side - baseline_as_side).abs() < 1e-9,
            "code {code}: as_side must remain identical regardless of M3 sign"
        );
    }
}

// =========================================================================
// 3. BOUNDARY CONDITION M2 = 0 MUST YIELD As_side = 0 (NEVER MINIMUM STEEL)
// =========================================================================

#[test]
fn test_adversarial_boundary_zero_m2() {
    for code in ["ACI 318-14", "Eurocode 2-2004"] {
        for &m2_zero_candidate in &[0.0, -0.0, 1e-15, -1e-14, 1e-13] {
            let (mut model, frame_name) =
                create_test_beam_model(code, Units::KipInF, "B24x14", 24.0, 14.0, Some(2.5));
            let mut cr = CaseResults {
                case_name: "COMBO_ZERO".into(),
                ..Default::default()
            };
            cr.frame_forces.insert(
                frame_name.clone(),
                vec![FrameStationResult {
                    station: 60.0,
                    rel_dist: 0.5,
                    m3: 1500.0,
                    m2: m2_zero_candidate,
                    ..Default::default()
                }],
            );
            model.case_results.insert("COMBO_ZERO".into(), cr);

            try_run_concrete_design(&mut model).unwrap();
            let res = &model.concrete_design_results[&frame_name];

            assert_eq!(
                res.as_side, 0.0,
                "code {code}, M2={m2_zero_candidate}: as_side must be exactly 0.0, never As_min"
            );
            assert!(
                res.min_rebar_area > 0.0,
                "code {code}: section min_rebar_area should be reported"
            );
        }

        // Entirely unloaded beam
        let (mut model_unloaded, frame_unloaded) =
            create_test_beam_model(code, Units::KipInF, "B24x14", 24.0, 14.0, Some(2.5));
        let mut cr_unloaded = CaseResults {
            case_name: "DEAD_ZERO".into(),
            ..Default::default()
        };
        cr_unloaded.frame_forces.insert(
            frame_unloaded.clone(),
            vec![FrameStationResult {
                station: 60.0,
                rel_dist: 0.5,
                m3: 0.0,
                m2: 0.0,
                v2: 0.0,
                v3: 0.0,
                p: 0.0,
                t: 0.0,
                ..Default::default()
            }],
        );
        model_unloaded
            .case_results
            .insert("DEAD_ZERO".into(), cr_unloaded);
        try_run_concrete_design(&mut model_unloaded).unwrap();
        let res_unloaded = &model_unloaded.concrete_design_results[&frame_unloaded];

        assert_eq!(res_unloaded.as_top, 0.0);
        assert_eq!(res_unloaded.as_bot, 0.0);
        assert_eq!(res_unloaded.as_side, 0.0);
        if code == "ACI 318-14" {
            assert_eq!(res_unloaded.av_over_s, 0.0);
        } else {
            // Eurocode 2 EN 1992-1-1 9.2.2 requires minimum shear stirrups Asw/s,min throughout beams
            assert!(res_unloaded.av_over_s > 0.0);
        }
        assert_eq!(res_unloaded.status, "OK");
    }
}

// =========================================================================
// 4. EXTREME OVERSTRESS (M2 > phi * Mn2,max) FLAGGING OVERSTRESSED
// =========================================================================

#[test]
fn test_adversarial_extreme_overstress_m2() {
    for code in ["ACI 318-14", "Eurocode 2-2004"] {
        // Normal B24x14:
        // Singly reinforced minor moment capacity:
        // ACI: c_max = 0.375 * 11.5 = 4.3125 in, a_max = 0.85 * 4.3125 = 3.6656 in
        // phi_Mn_max = 0.90 * 0.85 * 4.0 * 3.6656 * 24.0 * (11.5 - 0.5 * 3.6656) = 2603 kip-in.
        // Doubly reinforced upper bound = 2.5 * phi_Mn_max ~= 6500 kip-in.

        // Test 1: Doubly reinforced within bounds (M2 = 3500 kip-in) -> status "OK"
        {
            let (mut model, frame_name) =
                create_test_beam_model(code, Units::KipInF, "B24x14", 24.0, 14.0, Some(2.5));
            let mut cr = CaseResults {
                case_name: "DOUBLY_REINF".into(),
                ..Default::default()
            };
            cr.frame_forces.insert(
                frame_name.clone(),
                vec![FrameStationResult {
                    station: 60.0,
                    rel_dist: 0.5,
                    m3: 0.0,
                    m2: 3500.0,
                    ..Default::default()
                }],
            );
            model.case_results.insert("DOUBLY_REINF".into(), cr);
            try_run_concrete_design(&mut model).unwrap();
            let res = &model.concrete_design_results[&frame_name];
            assert_eq!(
                res.status, "OK",
                "code {code}: moderate doubly reinforced M2 should pass"
            );
            assert!(res.as_side > 0.0);
        }

        // Test 2: Catastrophic minor overstress (M2 = 25000 kip-in) -> status "OVERSTRESSED"
        {
            let (mut model, frame_name) =
                create_test_beam_model(code, Units::KipInF, "B24x14", 24.0, 14.0, Some(2.5));
            let mut cr = CaseResults {
                case_name: "EXTREME_M2".into(),
                ..Default::default()
            };
            cr.frame_forces.insert(
                frame_name.clone(),
                vec![FrameStationResult {
                    station: 60.0,
                    rel_dist: 0.5,
                    m3: 0.0,
                    m2: 25000.0,
                    ..Default::default()
                }],
            );
            model.case_results.insert("EXTREME_M2".into(), cr);
            try_run_concrete_design(&mut model).unwrap();
            let res = &model.concrete_design_results[&frame_name];
            assert_eq!(
                res.status, "OVERSTRESSED",
                "code {code}: extreme M2 must trigger OVERSTRESSED status"
            );
        }

        // Test 3: Minor overstress with zero major forces (M3=0, V2=0, V3=0)
        // Verifies that minor flexure alone can transition the member to OVERSTRESSED
        {
            let (mut model, frame_name) =
                create_test_beam_model(code, Units::KipInF, "B24x14", 24.0, 14.0, Some(2.5));
            let mut cr = CaseResults {
                case_name: "PURE_M2_CRUSH".into(),
                ..Default::default()
            };
            cr.frame_forces.insert(
                frame_name.clone(),
                vec![FrameStationResult {
                    station: 60.0,
                    rel_dist: 0.5,
                    m3: 0.0,
                    m2: 30000.0,
                    ..Default::default()
                }],
            );
            model.case_results.insert("PURE_M2_CRUSH".into(), cr);
            try_run_concrete_design(&mut model).unwrap();
            let res = &model.concrete_design_results[&frame_name];
            assert_eq!(
                res.status, "OVERSTRESSED",
                "code {code}: minor flexure alone must flag OVERSTRESSED"
            );
        }
    }
}

// =========================================================================
// 5. NON-STANDARD ASPECT RATIOS (TALL THIN, SHALLOW WIDE, SYMMETRICAL SQUARE)
// =========================================================================

#[test]
fn test_adversarial_aspect_ratios() {
    for code in ["ACI 318-14", "Eurocode 2-2004"] {
        // Aspect Ratio 1: Tall Thin Beam (h = 48 in, b = 6 in, cover = 2.0 in)
        // Minor flexural depth is tiny (b = 6 in, d = 4 in).
        {
            let (mut model, frame_name) =
                create_test_beam_model(code, Units::KipInF, "TALL_48x6", 48.0, 6.0, Some(2.0));
            let mut cr = CaseResults {
                case_name: "TALL_BEAM".into(),
                ..Default::default()
            };
            // Modest M2 of 150 kip-in
            cr.frame_forces.insert(
                frame_name.clone(),
                vec![FrameStationResult {
                    station: 60.0,
                    rel_dist: 0.5,
                    m3: 0.0,
                    m2: 150.0,
                    ..Default::default()
                }],
            );
            model.case_results.insert("TALL_BEAM".into(), cr);
            try_run_concrete_design(&mut model).unwrap();
            let res = &model.concrete_design_results[&frame_name];
            assert_eq!(res.status, "OK");
            assert!(res.as_side > 0.0);

            // Now apply excessive M2 = 2500 kip-in on this 6-inch thin beam -> must overstress
            let mut cr_over = CaseResults {
                case_name: "TALL_BEAM_OVER".into(),
                ..Default::default()
            };
            cr_over.frame_forces.insert(
                frame_name.clone(),
                vec![FrameStationResult {
                    station: 60.0,
                    rel_dist: 0.5,
                    m3: 0.0,
                    m2: 2500.0,
                    ..Default::default()
                }],
            );
            model.case_results.insert("TALL_BEAM_OVER".into(), cr_over);
            try_run_concrete_design(&mut model).unwrap();
            let res_over = &model.concrete_design_results[&frame_name];
            assert_eq!(res_over.status, "OVERSTRESSED");
        }

        // Aspect Ratio 2: Shallow Wide Beam (h = 8 in, b = 36 in, cover = 1.5 in)
        // Minor flexural depth is large (b = 36 in, d = 34.5 in, b_eff = 8 in).
        {
            let (mut model, frame_name) =
                create_test_beam_model(code, Units::KipInF, "WIDE_8x36", 8.0, 36.0, Some(1.5));
            let mut cr = CaseResults {
                case_name: "WIDE_BEAM".into(),
                ..Default::default()
            };
            // 2500 kip-in minor moment is easily carried by the 36-in wide depth
            cr.frame_forces.insert(
                frame_name.clone(),
                vec![FrameStationResult {
                    station: 60.0,
                    rel_dist: 0.5,
                    m3: 0.0,
                    m2: 2500.0,
                    ..Default::default()
                }],
            );
            model.case_results.insert("WIDE_BEAM".into(), cr);
            try_run_concrete_design(&mut model).unwrap();
            let res = &model.concrete_design_results[&frame_name];
            assert_eq!(res.status, "OK");
            assert!(res.as_side > 0.0);
        }

        // Aspect Ratio 3: Symmetrical Square Beam (h = 20 in, b = 20 in, cover = 2.5 in)
        // With identical dimensions and cover, pure M3 = M and pure M2 = M must produce
        // identical steel areas: As_bot(M3=M, M2=0) == As_side(M3=0, M2=M).
        {
            let test_moment = 1600.0;
            // Model A: pure M3
            let (mut model_a, frame_a) =
                create_test_beam_model(code, Units::KipInF, "SQ_20x20", 20.0, 20.0, Some(2.5));
            let mut cr_a = CaseResults {
                case_name: "M3".into(),
                ..Default::default()
            };
            cr_a.frame_forces.insert(
                frame_a.clone(),
                vec![FrameStationResult {
                    station: 60.0,
                    rel_dist: 0.5,
                    m3: test_moment,
                    m2: 0.0,
                    ..Default::default()
                }],
            );
            model_a.case_results.insert("M3".into(), cr_a);
            try_run_concrete_design(&mut model_a).unwrap();
            let as_bot_sq = model_a.concrete_design_results[&frame_a].as_bot;

            // Model B: pure M2
            let (mut model_b, frame_b) =
                create_test_beam_model(code, Units::KipInF, "SQ_20x20", 20.0, 20.0, Some(2.5));
            let mut cr_b = CaseResults {
                case_name: "M2".into(),
                ..Default::default()
            };
            cr_b.frame_forces.insert(
                frame_b.clone(),
                vec![FrameStationResult {
                    station: 60.0,
                    rel_dist: 0.5,
                    m3: 0.0,
                    m2: test_moment,
                    ..Default::default()
                }],
            );
            model_b.case_results.insert("M2".into(), cr_b);
            try_run_concrete_design(&mut model_b).unwrap();
            let as_side_sq = model_b.concrete_design_results[&frame_b].as_side;

            assert!(
                (as_bot_sq - as_side_sq).abs() < 1e-12,
                "code {code}: symmetry requirement violated! as_bot({as_bot_sq}) != as_side({as_side_sq})"
            );
        }
    }
}

// =========================================================================
// 6. UNIT SYSTEMS INVARIANCE (KipInF, KnMmC, KnMC)
// =========================================================================

#[test]
fn test_adversarial_unit_systems_invariance() {
    // Physical geometry:
    // Section: h = 24 in = 609.6 mm = 0.6096 m
    //          b = 14 in = 355.6 mm = 0.3556 m
    // Cover:   c = 2.5 in = 63.5 mm = 0.0635 m
    // Concrete fc = 4.0 ksi = 27.579028 MPa = 27579.028 kN/m^2
    // Steel fy = 60.0 ksi = 413.685419 MPa = 413685.419 kN/m^2
    // Moment M2 = 1200.0 kip-in = 135.581795 kN-m = 135581795.0 N-mm

    for code in ["ACI 318-14", "Eurocode 2-2004"] {
        // Run 1: KipInF
        let (mut model_us, frame_us) =
            create_test_beam_model(code, Units::KipInF, "B_US", 24.0, 14.0, Some(2.5));
        let mut cr_us = CaseResults {
            case_name: "RUN".into(),
            ..Default::default()
        };
        cr_us.frame_forces.insert(
            frame_us.clone(),
            vec![FrameStationResult {
                station: 60.0,
                rel_dist: 0.5,
                m3: 0.0,
                m2: 1200.0,
                ..Default::default()
            }],
        );
        model_us.case_results.insert("RUN".into(), cr_us);
        try_run_concrete_design(&mut model_us).unwrap();
        let as_side_in2 = model_us.concrete_design_results[&frame_us].as_side;

        // Run 2: KnMmC
        let (mut model_si_mm, frame_si_mm) =
            create_test_beam_model(code, Units::KnMmC, "B_SIMM", 609.6, 355.6, Some(63.5));
        // Ensure material fc, fy match
        if let Some(mat) = model_si_mm.materials.get_mut("4000Psi") {
            mat.fc = 4.0 * Units::KipInF.stress_factor_to(Units::KnMmC);
            mat.fy = 60.0 * Units::KipInF.stress_factor_to(Units::KnMmC);
        }
        let mut cr_si_mm = CaseResults {
            case_name: "RUN".into(),
            ..Default::default()
        };
        let m2_kn_mm = 1200.0 * Units::KipInF.moment_factor_to(Units::KnMmC);
        cr_si_mm.frame_forces.insert(
            frame_si_mm.clone(),
            vec![FrameStationResult {
                station: 1524.0,
                rel_dist: 0.5,
                m3: 0.0,
                m2: m2_kn_mm,
                ..Default::default()
            }],
        );
        model_si_mm.case_results.insert("RUN".into(), cr_si_mm);
        try_run_concrete_design(&mut model_si_mm).unwrap();
        let as_side_mm2 = model_si_mm.concrete_design_results[&frame_si_mm].as_side;

        // Run 3: KnMC
        let (mut model_si_m, frame_si_m) =
            create_test_beam_model(code, Units::KnMC, "B_SIM", 0.6096, 0.3556, Some(0.0635));
        if let Some(mat) = model_si_m.materials.get_mut("4000Psi") {
            mat.fc = 4.0 * Units::KipInF.stress_factor_to(Units::KnMC);
            mat.fy = 60.0 * Units::KipInF.stress_factor_to(Units::KnMC);
        }
        let mut cr_si_m = CaseResults {
            case_name: "RUN".into(),
            ..Default::default()
        };
        let m2_kn_m = 1200.0 * Units::KipInF.moment_factor_to(Units::KnMC);
        cr_si_m.frame_forces.insert(
            frame_si_m.clone(),
            vec![FrameStationResult {
                station: 1.524,
                rel_dist: 0.5,
                m3: 0.0,
                m2: m2_kn_m,
                ..Default::default()
            }],
        );
        model_si_m.case_results.insert("RUN".into(), cr_si_m);
        try_run_concrete_design(&mut model_si_m).unwrap();
        let as_side_m2 = model_si_m.concrete_design_results[&frame_si_m].as_side;

        // Convert all to in^2
        let converted_from_mm2 = as_side_mm2 / (25.4 * 25.4);
        let converted_from_m2 = as_side_m2 / (0.0254 * 0.0254);

        let err_mm = (converted_from_mm2 - as_side_in2).abs() / as_side_in2;
        let err_m = (converted_from_m2 - as_side_in2).abs() / as_side_in2;

        assert!(
            err_mm < 1e-4,
            "code {code}: KnMmC converted area {converted_from_mm2} differs from KipInF {as_side_in2} by {err_mm}"
        );
        assert!(
            err_m < 1e-4,
            "code {code}: KnMC converted area {converted_from_m2} differs from KipInF {as_side_in2} by {err_m}"
        );
    }
}

// =========================================================================
// 7. MULTI-STATION PROFILE AND MULTI-COMBINATION ENVELOPE
// =========================================================================

#[test]
fn test_adversarial_multi_station_and_multi_combo_envelope() {
    for code in ["ACI 318-14", "Eurocode 2-2004"] {
        let (mut model, frame_name) =
            create_test_beam_model(code, Units::KipInF, "B24x14", 24.0, 14.0, Some(2.5));

        // Combo 1: High positive major moment M3 = 2400, M2 = 0
        let mut cr1 = CaseResults {
            case_name: "COMBO_M3_MAX".into(),
            ..Default::default()
        };
        cr1.frame_forces.insert(
            frame_name.clone(),
            vec![
                FrameStationResult {
                    station: 0.0,
                    rel_dist: 0.0,
                    m3: 0.0,
                    v2: 50.0,
                    ..Default::default()
                },
                FrameStationResult {
                    station: 60.0,
                    rel_dist: 0.5,
                    m3: 2400.0,
                    v2: 0.0,
                    ..Default::default()
                },
                FrameStationResult {
                    station: 120.0,
                    rel_dist: 1.0,
                    m3: 0.0,
                    v2: -50.0,
                    ..Default::default()
                },
            ],
        );
        model.case_results.insert("COMBO_M3_MAX".into(), cr1);

        // Combo 2: High minor moment M2 = 1500 (parabolic variation across 3 stations), M3 = 0
        let mut cr2 = CaseResults {
            case_name: "COMBO_M2_MAX".into(),
            ..Default::default()
        };
        cr2.frame_forces.insert(
            frame_name.clone(),
            vec![
                FrameStationResult {
                    station: 0.0,
                    rel_dist: 0.0,
                    m2: 300.0,
                    v3: 30.0,
                    ..Default::default()
                },
                FrameStationResult {
                    station: 60.0,
                    rel_dist: 0.5,
                    m2: 1500.0,
                    v3: 0.0,
                    ..Default::default()
                },
                FrameStationResult {
                    station: 120.0,
                    rel_dist: 1.0,
                    m2: -400.0,
                    v3: -30.0,
                    ..Default::default()
                },
            ],
        );
        model.case_results.insert("COMBO_M2_MAX".into(), cr2);

        // Combo 3: High shear forces V2 = 65, V3 = 45, moderate moments
        let mut cr3 = CaseResults {
            case_name: "COMBO_SHEAR_MAX".into(),
            ..Default::default()
        };
        cr3.frame_forces.insert(
            frame_name.clone(),
            vec![FrameStationResult {
                station: 0.0,
                rel_dist: 0.0,
                m3: 500.0,
                m2: 200.0,
                v2: 65.0,
                v3: 45.0,
                ..Default::default()
            }],
        );
        model.case_results.insert("COMBO_SHEAR_MAX".into(), cr3);

        try_run_concrete_design(&mut model).unwrap();
        let res = &model.concrete_design_results[&frame_name];

        assert_eq!(res.status, "OK");

        // Verify that as_bot is sized by COMBO_M3_MAX (M3=2400)
        let expected_as_bot = if code == "ACI 318-14" {
            design_aci_beam_flexure(2400.0, 14.0, 21.5, 2.5, 4.0, 60.0, 29000.0, Units::KipInF).0
        } else {
            design_ec2_beam(2400.0, 0.0, 14.0, 21.5, 2.5, 4.0, 60.0, 60.0, Units::KipInF).0
        };
        assert!(
            (res.as_bot - expected_as_bot).abs() < 1e-9,
            "as_bot must envelope to M3=2400"
        );

        // Verify that as_side is sized by COMBO_M2_MAX midspan station (M2=1500)
        let expected_as_side = if code == "ACI 318-14" {
            design_aci_beam_flexure(1500.0, 24.0, 11.5, 2.5, 4.0, 60.0, 29000.0, Units::KipInF).0
        } else {
            design_ec2_beam(1500.0, 0.0, 24.0, 11.5, 2.5, 4.0, 60.0, 60.0, Units::KipInF).0
        };
        assert!(
            (res.as_side - expected_as_side).abs() < 1e-9,
            "as_side must envelope to M2=1500"
        );

        // Verify that av_over_s envelopes the highest shear
        assert!(res.av_over_s > 0.0);
    }
}

// =========================================================================
// 8. ADVERSARIAL EDGE CASES & ROBUSTNESS HARNESS
// =========================================================================

#[test]
fn test_adversarial_code_aliases_and_unsupported_codes() {
    // Permutations of valid code names
    for valid_alias in &[
        "ACI318-14",
        "aci 318-14",
        "ACI 318",
        "ACI",
        "Eurocode 2",
        "EC2",
        "eurocode 2-2004",
        "EN 1992-1-1",
    ] {
        let (mut model, frame_name) =
            create_test_beam_model(valid_alias, Units::KipInF, "B24x14", 24.0, 14.0, Some(2.5));
        let mut cr = CaseResults {
            case_name: "C1".into(),
            ..Default::default()
        };
        cr.frame_forces.insert(
            frame_name.clone(),
            vec![FrameStationResult {
                station: 60.0,
                rel_dist: 0.5,
                m2: 500.0,
                ..Default::default()
            }],
        );
        model.case_results.insert("C1".into(), cr);
        let res = try_run_concrete_design(&mut model);
        assert!(
            res.is_ok(),
            "code alias '{valid_alias}' must be recognized as valid"
        );
        let design_res = &model.concrete_design_results[&frame_name];
        assert!(
            design_res.as_side > 0.0,
            "as_side must be populated for code '{valid_alias}'"
        );
    }

    // Invalid / unsupported code name must fail closed
    let (mut model_invalid, _) = create_test_beam_model(
        "BS8110-1997",
        Units::KipInF,
        "B24x14",
        24.0,
        14.0,
        Some(2.5),
    );
    let err = try_run_concrete_design(&mut model_invalid);
    assert!(
        err.is_err(),
        "unsupported code BS8110-1997 must fail closed"
    );
    assert!(model_invalid.concrete_design_results.is_empty());
}

#[test]
fn test_adversarial_extreme_cover_and_empty_stations() {
    // Case 1: Extreme cover exceeding half the section dimension
    // Cover = 8 in on a 14 in wide section (half width is 7 in)
    let (mut model_cover, frame_cover) =
        create_test_beam_model("ACI 318-14", Units::KipInF, "B24x14", 24.0, 14.0, Some(8.0));
    let mut cr_cover = CaseResults {
        case_name: "COVER_TEST".into(),
        ..Default::default()
    };
    cr_cover.frame_forces.insert(
        frame_cover.clone(),
        vec![FrameStationResult {
            station: 60.0,
            rel_dist: 0.5,
            m2: 300.0,
            ..Default::default()
        }],
    );
    model_cover
        .case_results
        .insert("COVER_TEST".into(), cr_cover);
    // Must execute cleanly without NaN or panic
    let res = try_run_concrete_design(&mut model_cover);
    assert!(res.is_ok());
    let r = &model_cover.concrete_design_results[&frame_cover];
    assert!(r.as_side.is_finite());
    assert!(r.as_side > 0.0);

    // Case 2: Frame present in model but has empty stations list
    // StructuraX fail-closed design contract requires analysis forces:
    // missing demand must fail closed with DesignError::MissingDemand rather than
    // reporting false 0.0 steel as valid.
    let (mut model_empty, frame_empty) =
        create_test_beam_model("ACI 318-14", Units::KipInF, "B24x14", 24.0, 14.0, Some(2.5));
    let mut cr_empty = CaseResults {
        case_name: "EMPTY_STATION".into(),
        ..Default::default()
    };
    cr_empty.frame_forces.insert(frame_empty.clone(), vec![]);
    model_empty
        .case_results
        .insert("EMPTY_STATION".into(), cr_empty);
    let res_empty = try_run_concrete_design(&mut model_empty);
    assert!(
        res_empty.is_err(),
        "empty stations must return DesignError::MissingDemand"
    );
    assert!(model_empty.concrete_design_results.is_empty());
}

#[test]
fn test_adversarial_nan_demand_fails_closed() {
    let (mut model_nan, frame_nan) =
        create_test_beam_model("ACI 318-14", Units::KipInF, "B24x14", 24.0, 14.0, Some(2.5));
    let mut cr_nan = CaseResults {
        case_name: "NAN_FORCE".into(),
        ..Default::default()
    };
    cr_nan.frame_forces.insert(
        frame_nan.clone(),
        vec![FrameStationResult {
            station: 60.0,
            rel_dist: 0.5,
            m2: f64::NAN,
            ..Default::default()
        }],
    );
    model_nan.case_results.insert("NAN_FORCE".into(), cr_nan);
    let res = try_run_concrete_design(&mut model_nan);
    assert!(
        res.is_err(),
        "NaN force demand must be rejected by demand validation"
    );
    assert!(
        model_nan.concrete_design_results.is_empty(),
        "no stale or partial results when demand validation fails"
    );
}
