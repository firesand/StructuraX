//! Comprehensive regression and verification test suite for Milestone M2:
//! - DSGN-01: Concrete beam minor-axis flexure ($M_2$) for ACI 318-14 and Eurocode 2-2004.
//! - DSGN-03: Eurocode 3 local-3 shear ($V_3$) and multi-axis interaction with high-shear moment reductions ($\rho_2, \rho_3$).

use sap_design::{
    concrete::{design_aci_beam_flexure, design_ec2_beam},
    shear::{compute_ec3_major_shear, compute_ec3_minor_shear},
    try_run_concrete_design, try_run_steel_design, DesignError,
};
use sap_model::{CaseResults, FramePropType, FrameStationResult, SapModel};
use sap_section::{make_i_section, make_pipe, make_rectangular};
use sap_units::Units;

fn setup_concrete_model(code: &str, units: Units) -> (SapModel, String, String) {
    let mut model = SapModel::new(units);
    model.concrete_design_code = code.to_string();

    let sec = make_rectangular("BEAM_24x14", "4000Psi", 24.0, 14.0);
    model.frame_sections.insert(sec.name.clone(), sec);
    model.add_point("1", 0.0, 0.0, 0.0);
    model.add_point("2", 120.0, 0.0, 0.0);
    model.add_frame("B1", "1", "2", "BEAM_24x14");

    (model, "B1".to_string(), "BEAM_24x14".to_string())
}

#[test]
fn test_dsgn01_concrete_beam_pure_major_flexure() {
    for code in ["ACI 318-14", "Eurocode 2-2004"] {
        let (mut model, frame_name, _) = setup_concrete_model(code, Units::KipInF);
        let mut cr = CaseResults {
            case_name: "COMBO1".into(),
            ..Default::default()
        };
        cr.frame_forces.insert(
            frame_name.clone(),
            vec![FrameStationResult {
                station: 60.0,
                rel_dist: 0.5,
                m3: 2000.0,
                m2: 0.0,
                ..Default::default()
            }],
        );
        model.case_results.insert("COMBO1".into(), cr);

        try_run_concrete_design(&mut model).unwrap();
        let res = &model.concrete_design_results[&frame_name];
        assert_eq!(res.status, "OK");
        assert!(res.as_bot > 0.0, "code: {code} as_bot should be > 0");
        assert_eq!(
            res.as_side, 0.0,
            "code: {code} as_side must be 0 when M2 = 0"
        );
    }
}

#[test]
fn test_dsgn01_concrete_beam_pure_minor_flexure() {
    for code in ["ACI 318-14", "Eurocode 2-2004"] {
        let (mut model, frame_name, _) = setup_concrete_model(code, Units::KipInF);
        let mut cr = CaseResults {
            case_name: "COMBO1".into(),
            ..Default::default()
        };
        cr.frame_forces.insert(
            frame_name.clone(),
            vec![FrameStationResult {
                station: 60.0,
                rel_dist: 0.5,
                m3: 0.0,
                m2: 1200.0,
                ..Default::default()
            }],
        );
        model.case_results.insert("COMBO1".into(), cr);

        try_run_concrete_design(&mut model).unwrap();
        let res = &model.concrete_design_results[&frame_name];
        assert_eq!(res.status, "OK");
        assert!(res.as_side > 0.0, "code: {code} as_side should be > 0");
        assert_eq!(
            res.as_top, 0.0,
            "code: {code} as_top should be 0 for pure minor flexure without M3"
        );
        assert_eq!(
            res.as_bot, 0.0,
            "code: {code} as_bot should be 0 for pure minor flexure without M3"
        );

        // Verify that as_side matches hand calculation for the minor axis:
        // Section: h_eff = 24 in, depth = 14 in, cover = 2.5 in -> d2 = 11.5 in
        if code == "ACI 318-14" {
            let (expected_as, _, _) =
                design_aci_beam_flexure(1200.0, 24.0, 11.5, 2.5, 4.0, 60.0, 29000.0, Units::KipInF);
            assert!(
                (res.as_side - expected_as).abs() < 1e-10,
                "ACI as_side parity"
            );
        } else {
            let (expected_as, _, _, _) =
                design_ec2_beam(1200.0, 0.0, 24.0, 11.5, 2.5, 4.0, 60.0, 60.0, Units::KipInF);
            assert!(
                (res.as_side - expected_as).abs() < 1e-10,
                "EC2 as_side parity"
            );
        }
    }
}

#[test]
fn test_dsgn01_concrete_beam_combined_biaxial_flexure() {
    for code in ["ACI 318-14", "Eurocode 2-2004"] {
        let (mut model, frame_name, _) = setup_concrete_model(code, Units::KipInF);
        let mut cr = CaseResults {
            case_name: "COMBO1".into(),
            ..Default::default()
        };
        cr.frame_forces.insert(
            frame_name.clone(),
            vec![FrameStationResult {
                station: 60.0,
                rel_dist: 0.5,
                m3: 1800.0,
                m2: 900.0,
                v2: 30.0,
                v3: 15.0,
                ..Default::default()
            }],
        );
        model.case_results.insert("COMBO1".into(), cr);

        try_run_concrete_design(&mut model).unwrap();
        let res = &model.concrete_design_results[&frame_name];
        assert_eq!(res.status, "OK");
        assert!(res.as_bot > 0.0);
        assert!(res.as_side > 0.0);
        assert!(res.av_over_s > 0.0);

        // Orthogonal independence check:
        // as_bot is governed by M3=1800, as_side is governed by M2=900
        if code == "ACI 318-14" {
            let (expected_as_bot, _, _) =
                design_aci_beam_flexure(1800.0, 14.0, 21.5, 2.5, 4.0, 60.0, 29000.0, Units::KipInF);
            let (expected_as_side, _, _) =
                design_aci_beam_flexure(900.0, 24.0, 11.5, 2.5, 4.0, 60.0, 29000.0, Units::KipInF);
            assert!((res.as_bot - expected_as_bot).abs() < 1e-10);
            assert!((res.as_side - expected_as_side).abs() < 1e-10);
        }
    }
}

#[test]
fn test_dsgn01_concrete_beam_overstressed_minor_flexure() {
    let (mut model, frame_name, _) = setup_concrete_model("ACI 318-14", Units::KipInF);
    let mut cr = CaseResults {
        case_name: "OVERSTRESS".into(),
        ..Default::default()
    };
    // Extreme M2 demand that exceeds crushing limit
    cr.frame_forces.insert(
        frame_name.clone(),
        vec![FrameStationResult {
            station: 60.0,
            rel_dist: 0.5,
            m3: 100.0,
            m2: 15000.0,
            ..Default::default()
        }],
    );
    model.case_results.insert("OVERSTRESS".into(), cr);

    try_run_concrete_design(&mut model).unwrap();
    let res = &model.concrete_design_results[&frame_name];
    assert_eq!(res.status, "OVERSTRESSED");
}

fn setup_steel_model() -> (SapModel, String, String) {
    let mut model = SapModel::new(Units::KipInF);
    model.steel_design_code = "Eurocode 3-2005".to_string();

    let sec = make_i_section("W14X90", "A992Fy50", 14.0, 10.0, 0.71, 0.44, 10.0, 0.71);
    model.frame_sections.insert(sec.name.clone(), sec);
    model.add_point("1", 0.0, 0.0, 0.0);
    model.add_point("2", 120.0, 0.0, 0.0);
    model.add_frame("C1", "1", "2", "W14X90");

    (model, "C1".to_string(), "W14X90".to_string())
}

#[test]
fn test_dsgn03_ec3_multi_axis_concurrent_demands_succeed() {
    let (mut model, frame_name, _) = setup_steel_model();
    let mut cr = CaseResults {
        case_name: "MULTI".into(),
        ..Default::default()
    };
    cr.frame_forces.insert(
        frame_name.clone(),
        vec![FrameStationResult {
            station: 60.0,
            rel_dist: 0.5,
            p: -100.0,
            v2: 25.0,
            v3: 15.0,
            m3: 1200.0,
            m2: 300.0,
            t: 0.0,
            ..Default::default()
        }],
    );
    model.case_results.insert("MULTI".into(), cr);

    // Previously returned UnsupportedEc3ShearInteraction; now must succeed
    try_run_steel_design(&mut model).unwrap();
    let res = &model.steel_design_results[&frame_name];
    assert_eq!(res.status, "OK");
    assert!(res.total_ratio > 0.0 && res.total_ratio < 1.0);
    assert!(res.phi_vn3 > 0.0);
    assert_eq!(res.eq_name, "EC3-6.2");
}

#[test]
fn test_dsgn03_ec3_high_shear_reductions_and_governing_equations() {
    let (mut model, frame_name, sec_name) = setup_steel_model();
    let sec = &model.frame_sections[&sec_name];
    let mat = &model.materials["A992Fy50"];

    let v_pl_2 = compute_ec3_major_shear(sec, mat).unwrap();
    let v_pl_3 = compute_ec3_minor_shear(sec, mat).unwrap();

    // 1. High V2 (> 0.5 Vpl,2) triggers rho2 and EC3-6.2.6+6.2.8(3) when combined interaction governs
    let mut cr1 = CaseResults {
        case_name: "HIGH_V2".into(),
        ..Default::default()
    };
    cr1.frame_forces.insert(
        frame_name.clone(),
        vec![FrameStationResult {
            station: 60.0,
            rel_dist: 0.5,
            p: -50.0,
            v2: 0.60 * v_pl_2,
            v3: 0.10 * v_pl_3,
            m3: 3500.0,
            m2: 500.0,
            ..Default::default()
        }],
    );
    model.case_results.insert("HIGH_V2".into(), cr1);
    try_run_steel_design(&mut model).unwrap();
    let res1 = &model.steel_design_results[&frame_name];
    assert_eq!(res1.eq_name, "EC3-6.2.6+6.2.8(3)");
    assert!(res1.total_ratio > res1.shear_major_ratio);

    // 2. High V3 (> 0.5 Vpl,3) triggers rho3 and EC3-6.2.6+6.2.8(3)
    let mut cr2 = CaseResults {
        case_name: "HIGH_V3".into(),
        ..Default::default()
    };
    cr2.frame_forces.insert(
        frame_name.clone(),
        vec![FrameStationResult {
            station: 60.0,
            rel_dist: 0.5,
            p: -50.0,
            v2: 0.10 * v_pl_2,
            v3: 0.80 * v_pl_3,
            m3: 400.0,
            m2: 200.0,
            ..Default::default()
        }],
    );
    model.case_results.clear();
    model.case_results.insert("HIGH_V3".into(), cr2);
    try_run_steel_design(&mut model).unwrap();
    let res2 = &model.steel_design_results[&frame_name];
    assert!(
        res2.eq_name == "EC3-6.2.6+6.2.8(3)" || res2.eq_name == "EC3-6.2.6 (V3)",
        "Expected high shear or shear governing eq, got: {}",
        res2.eq_name
    );

    // 3. Pure governing shear equation checks
    let mut cr_v3_gov = CaseResults {
        case_name: "V3_GOV".into(),
        ..Default::default()
    };
    cr_v3_gov.frame_forces.insert(
        frame_name.clone(),
        vec![FrameStationResult {
            station: 60.0,
            rel_dist: 0.5,
            v3: 1.20 * v_pl_3, // Overstressed pure minor shear
            ..Default::default()
        }],
    );
    model.case_results.clear();
    model.case_results.insert("V3_GOV".into(), cr_v3_gov);
    try_run_steel_design(&mut model).unwrap();
    let res_v3 = &model.steel_design_results[&frame_name];
    assert_eq!(res_v3.eq_name, "EC3-6.2.6 (V3)");
    assert_eq!(res_v3.status, "OVERSTRESSED");
    assert!((res_v3.total_ratio - 1.20).abs() < 1e-10);
}

#[test]
fn test_dsgn03_ec3_pipe_and_circle_srss_interaction() {
    let mut model = SapModel::new(Units::KipInF);
    model.steel_design_code = "Eurocode 3-2005".to_string();

    let pipe = make_pipe("PIPE_10x0.5", "A992Fy50", 10.0, 0.5);
    model.frame_sections.insert(pipe.name.clone(), pipe);
    model.add_point("1", 0.0, 0.0, 0.0);
    model.add_point("2", 120.0, 0.0, 0.0);
    model.add_frame("P1", "1", "2", "PIPE_10x0.5");

    let mut cr = CaseResults {
        case_name: "SRSS_PIPE".into(),
        ..Default::default()
    };
    cr.frame_forces.insert(
        "P1".into(),
        vec![FrameStationResult {
            station: 60.0,
            rel_dist: 0.5,
            p: -50.0,
            m3: 300.0,
            m2: 400.0,
            ..Default::default()
        }],
    );
    model.case_results.insert("SRSS_PIPE".into(), cr);

    try_run_steel_design(&mut model).unwrap();
    let res = &model.steel_design_results["P1"];
    assert_eq!(res.status, "OK");

    // Circular section interaction uses SRSS of normalized moments:
    // Pr / Pc + sqrt((M3 / Mc3)^2 + (M2 / Mc2)^2)
    let pc = res.phi_pnc;
    let mc = res.phi_mn33; // for pipe, mc3 == mc2
    let expected_srss_ratio = (50.0 / pc) + ((300.0 / mc).powi(2) + (400.0 / mc).powi(2)).sqrt();
    assert!(
        (res.total_ratio - expected_srss_ratio).abs() < 1e-9,
        "Pipe SRSS ratio parity: got {}, expected {}",
        res.total_ratio,
        expected_srss_ratio
    );
}

#[test]
fn test_dsgn03_unsupported_shapes_fail_closed_with_minor_shear() {
    let mut model = SapModel::new(Units::KipInF);
    model.steel_design_code = "Eurocode 3-2005".to_string();

    let mut auto_sec = make_rectangular("AUTO_SEC", "A992Fy50", 10.0, 10.0);
    auto_sec.prop_type = FramePropType::Auto;
    model.frame_sections.insert(auto_sec.name.clone(), auto_sec);
    model.add_point("1", 0.0, 0.0, 0.0);
    model.add_point("2", 120.0, 0.0, 0.0);
    model.add_frame("F1", "1", "2", "AUTO_SEC");

    let mut cr = CaseResults {
        case_name: "COMBO".into(),
        ..Default::default()
    };
    cr.frame_forces.insert(
        "F1".into(),
        vec![FrameStationResult {
            station: 60.0,
            rel_dist: 0.5,
            v3: 10.0,
            ..Default::default()
        }],
    );
    model.case_results.insert("COMBO".into(), cr);

    assert!(matches!(
        try_run_steel_design(&mut model),
        Err(DesignError::UnsupportedEc3MinorShearSection { .. })
    ));
}
