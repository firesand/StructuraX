pub mod concrete;
mod error;
pub mod shear;
pub use shear::{
    compute_aisc_minor_shear, compute_ec3_major_shear, compute_ec3_minor_shear,
    compute_ec3_rectangular_minor_shear,
};
pub mod steel;

pub use error::DesignError;

pub use concrete::{
    compute_whitney_beta1, design_aci_beam_flexure, design_aci_beam_shear, design_ec2_beam,
    evaluate_column_biaxial_pmm_ratio, generate_column_pm_curve, interpolate_phi_mn_at_pu,
    is_concrete_column, run_concrete_design, try_run_concrete_design, ConcreteCode, PmCurvePoint,
};
pub use steel::{
    check_compactness_aisc, compute_aisc_360_capacities, compute_aisc_torsional_capacity,
    compute_cb_from_stations, compute_ec3_capacities, compute_k_braced, compute_k_sway,
    compute_member_k_factors, compute_slender_q_factor, evaluate_station_interaction,
    frame_clear_length, run_steel_design, run_steel_design_with_overwrites, try_run_steel_design,
    try_run_steel_design_with_overwrites, CompactnessClass, SectionCompactness, SteelCode,
    SteelMemberOverwrite,
};

#[cfg(test)]
mod tests {
    use super::*;
    use sap_model::{
        CaseResults, ComboType, ConcreteRebarConfig, FramePropType, FrameStationResult, LoadCombo,
        SapModel,
    };
    use sap_section::{
        make_box_tube, make_circle, make_i_section, make_pipe, make_rectangular, ProDatabase,
    };
    use sap_units::Units;
    use std::path::Path;

    fn model_with_steel_frame() -> SapModel {
        let mut model = SapModel::new(Units::KipInF);
        let section = make_i_section("TEST_W", "A992Fy50", 14.0, 10.0, 0.5, 0.3, 10.0, 0.5);
        model.frame_sections.insert(section.name.clone(), section);
        model.add_point("I", 0.0, 0.0, 0.0);
        model.add_point("J", 0.0, 0.0, 120.0);
        model.add_frame("STEEL", "I", "J", "TEST_W");
        model
    }

    #[test]
    fn checked_code_parsers_reject_unknown_names() {
        assert!(matches!(
            SteelCode::try_from_code_str("AISC 360-10"),
            Ok(SteelCode::Aisc360Lrfd)
        ));
        assert!(matches!(
            SteelCode::try_from_code_str("AISC 360-10 ASD"),
            Ok(SteelCode::Aisc360Asd)
        ));
        assert!(SteelCode::try_from_code_str("AISC-ish maybe").is_err());
        assert!(matches!(
            ConcreteCode::try_from_code_str("ACI 318-14"),
            Ok(ConcreteCode::Aci318)
        ));
        assert!(matches!(
            ConcreteCode::try_from_code_str("Eurocode 2-2004"),
            Ok(ConcreteCode::Eurocode2)
        ));
        assert!(ConcreteCode::try_from_code_str("ACI? hopefully").is_err());

        let mut steel_model = model_with_steel_frame();
        steel_model.steel_design_code = "AISC-ish maybe".to_string();
        assert!(matches!(
            try_run_steel_design(&mut steel_model),
            Err(DesignError::UnsupportedSteelCode(_))
        ));
        assert!(steel_model.steel_design_results.is_empty());

        let mut concrete_model = SapModel::new(Units::KipInF);
        concrete_model.concrete_design_code = "ACI? hopefully".to_string();
        assert!(matches!(
            try_run_concrete_design(&mut concrete_model),
            Err(DesignError::UnsupportedConcreteCode(_))
        ));
    }

    #[test]
    fn checked_design_rejects_missing_force_demand() {
        let mut steel_model = model_with_steel_frame();
        assert!(matches!(
            try_run_steel_design(&mut steel_model),
            Err(DesignError::MissingDemand {
                discipline: "steel",
                ..
            })
        ));
        assert!(steel_model.steel_design_results.is_empty());

        let mut concrete_model = SapModel::new(Units::KipInF);
        let section = make_rectangular("TEST_RC", "4000Psi", 24.0, 14.0);
        concrete_model
            .frame_sections
            .insert(section.name.clone(), section);
        concrete_model.add_point("I", 0.0, 0.0, 0.0);
        concrete_model.add_point("J", 0.0, 0.0, 120.0);
        concrete_model.add_frame("CONCRETE", "I", "J", "TEST_RC");
        assert!(matches!(
            try_run_concrete_design(&mut concrete_model),
            Err(DesignError::MissingDemand {
                discipline: "concrete",
                ..
            })
        ));
        assert!(concrete_model.concrete_design_results.is_empty());
    }

    #[test]
    fn concrete_design_rejects_nonzero_local3_shear_instead_of_ignoring_it() {
        let mut model = SapModel::new(Units::KipInF);
        let section = make_rectangular("TEST_RC", "4000Psi", 24.0, 14.0);
        model.frame_sections.insert(section.name.clone(), section);
        model.add_point("I", 0.0, 0.0, 0.0);
        model.add_point("J", 0.0, 0.0, 120.0);
        model.add_frame("CONCRETE", "I", "J", "TEST_RC");
        let mut cr = CaseResults {
            case_name: "DEAD".into(),
            ..Default::default()
        };
        // Minor-axis shear V3 = 45 kips on 24x14 section (bw3 = 24 in, d3 = 14 - 2.5 = 11.5 in)
        // phi * Vc3 = 0.75 * 2 * sqrt(4000) * 24 * 11.5 / 1000 = 26.1836 kips
        cr.frame_forces.insert(
            "CONCRETE".into(),
            vec![FrameStationResult {
                v3: 45.0,
                ..Default::default()
            }],
        );
        model.case_results.insert("DEAD".into(), cr);

        try_run_concrete_design(&mut model).unwrap();
        let res = &model.concrete_design_results["CONCRETE"];
        let (expected_av_s3, _) = design_aci_beam_shear(45.0, 24.0, 11.5, 4.0, 60.0, Units::KipInF);
        assert!((res.av_over_s - expected_av_s3).abs() < 1e-10);
        assert!(res.av_over_s > 0.0);
        assert_eq!(res.status, "OK");
    }

    #[test]
    fn checked_steel_design_checks_local_3_shear_and_preserves_ec3_guard() {
        let mut model = model_with_steel_frame();
        let mut case = CaseResults {
            case_name: "DEAD".to_string(),
            ..Default::default()
        };
        case.frame_forces.insert(
            "STEEL".to_string(),
            vec![FrameStationResult {
                v3: 12.0,
                ..Default::default()
            }],
        );
        model.case_results.insert("DEAD".to_string(), case);

        try_run_steel_design(&mut model).unwrap();
        let result = &model.steel_design_results["STEEL"];
        let expected_aisc_mn22 = compute_aisc_360_capacities(
            &model.frame_sections["TEST_W"],
            &model.materials["A992Fy50"],
            120.0,
            120.0,
            120.0,
            1.0,
            1.0,
            1.0,
            false,
        )
        .3;
        assert!((result.phi_mn22 - expected_aisc_mn22).abs() < 1e-10);
        assert!((result.phi_vn3 - 270.0).abs() < 1e-10);
        assert!((result.shear_minor_ratio - 12.0 / 270.0).abs() < 1e-10);
        assert_eq!(result.total_ratio, result.shear_minor_ratio);
        assert_eq!(result.eq_name, "G7/G2-1 (V3)");

        model
            .case_results
            .get_mut("DEAD")
            .unwrap()
            .frame_forces
            .get_mut("STEEL")
            .unwrap()
            .push(FrameStationResult {
                station: 60.0,
                rel_dist: 0.5,
                v3: -540.0,
                ..Default::default()
            });
        try_run_steel_design(&mut model).unwrap();
        let result = &model.steel_design_results["STEEL"];
        assert_eq!(result.governing_station, 60.0);
        assert!((result.total_ratio - 2.0).abs() < 1e-12);
        assert_eq!(result.status, "OVERSTRESSED");

        // EC3 local-3 shear on an I-section is now supported per EN 1993-1-1 6.2.6(3).
        model.steel_design_code = "Eurocode 3-2005".into();
        try_run_steel_design(&mut model).unwrap();
        let result = &model.steel_design_results["STEEL"];
        assert!((result.phi_vn3 - 500.0 / 3.0_f64.sqrt()).abs() < 1e-10);
        assert_eq!(result.eq_name, "EC3-6.2.6 (V3)");
        assert_eq!(result.status, "OVERSTRESSED");

        model.frame_sections.get_mut("TEST_W").unwrap().prop_type = FramePropType::Auto;
        assert!(matches!(
            try_run_steel_design(&mut model),
            Err(DesignError::UnsupportedEc3MinorShearSection { .. })
        ));
        assert!(model.steel_design_results.is_empty());
    }

    #[test]
    fn ec3_rectangular_v3_shear_and_high_shear_minor_bending_are_checked() {
        let mut model = SapModel::new(Units::KipInF);
        model.steel_design_code = "Eurocode 3-2005".to_string();
        let section = make_rectangular("RECT", "A992Fy50", 8.0, 4.0);
        model.frame_sections.insert(section.name.clone(), section);
        model.add_point("I", 0.0, 0.0, 0.0);
        model.add_point("J", 0.0, 0.0, 120.0);
        model.add_frame("R1", "I", "J", "RECT");

        let v_pl_rd = 32.0 * 50.0 / 3.0_f64.sqrt();
        let m_pl_rd = 32.0 * 50.0;
        let mut cr = CaseResults {
            case_name: "DEAD".into(),
            ..Default::default()
        };
        cr.frame_forces.insert(
            "R1".into(),
            vec![FrameStationResult {
                station: 60.0,
                rel_dist: 0.5,
                v3: 0.75 * v_pl_rd,
                m2: 0.875 * m_pl_rd,
                ..Default::default()
            }],
        );
        model.case_results.insert("DEAD".into(), cr);
        try_run_steel_design(&mut model).unwrap();

        let result = &model.steel_design_results["R1"];
        // EC3 6.2.8(3): rho=(2*0.75-1)^2=0.25; Mv,Rd=(1-rho)Mpl,Rd.
        assert!((result.phi_vn3 - v_pl_rd).abs() < 1e-10);
        assert!((result.shear_minor_ratio - 0.75).abs() < 1e-12);
        assert!((result.mr_minor_ratio - 0.875 / 0.75).abs() < 1e-12);
        assert!((result.phi_mn22 - 1200.0).abs() < 1e-10);
        assert!((1400.0 / result.phi_mn22 - result.mr_minor_ratio).abs() < 1e-12);
        assert!((result.total_ratio - 0.875 / 0.75).abs() < 1e-12);
        assert_eq!(result.eq_name, "EC3-6.2.6+6.2.8(3)");
        assert_eq!(result.status, "OVERSTRESSED");
    }

    #[test]
    fn ec3_minor_shear_interaction_rejects_unsupported_shapes_and_force_combinations() {
        let mut model = model_with_steel_frame();
        model.steel_design_code = "Eurocode 3-2005".into();
        let mut cr = CaseResults {
            case_name: "DEAD".into(),
            ..Default::default()
        };
        cr.frame_forces.insert(
            "STEEL".into(),
            vec![FrameStationResult {
                v3: 1.0,
                ..Default::default()
            }],
        );
        model.case_results.insert("DEAD".into(), cr);
        // EC3 6.2.6(3) now supports I sections for local-3 shear:
        try_run_steel_design(&mut model).unwrap();
        let res = &model.steel_design_results["STEEL"];
        assert!((res.phi_vn3 - 500.0 / 3.0_f64.sqrt()).abs() < 1e-10);

        // Truly unsupported shapes still fail closed:
        model.frame_sections.get_mut("TEST_W").unwrap().prop_type = FramePropType::General;
        assert!(matches!(
            try_run_steel_design(&mut model),
            Err(DesignError::UnsupportedEc3MinorShearSection { .. })
        ));

        // Combined multi-axis forces (P and V3) are now supported per EN 1993-1-1 6.2:
        let rect = make_rectangular("RECT", "A992Fy50", 8.0, 4.0);
        model.frame_sections.insert(rect.name.clone(), rect);
        model.frames.get_mut("STEEL").unwrap().section = "RECT".into();
        model
            .case_results
            .get_mut("DEAD")
            .unwrap()
            .frame_forces
            .get_mut("STEEL")
            .unwrap()[0]
            .p = 1.0;
        try_run_steel_design(&mut model).unwrap();
        let res_rect = &model.steel_design_results["STEEL"];
        assert_eq!(res_rect.status, "OK");
        assert!(res_rect.total_ratio > 0.0);
    }

    #[test]
    fn steel_code_rejects_other_materials_and_closed_section_torsion() {
        let mut model = model_with_steel_frame();
        let mut case = CaseResults::default();
        case.frame_forces
            .insert("STEEL".into(), vec![FrameStationResult::default()]);
        model.case_results.insert("DEAD".into(), case);
        // Start with a valid result and ensure each failed rerun clears it.
        try_run_steel_design(&mut model).unwrap();
        model.materials.get_mut("A992Fy50").unwrap().mat_type = sap_model::MatType::Aluminum;
        assert!(matches!(
            try_run_steel_design(&mut model),
            Err(DesignError::UnsupportedSteelMaterial { .. })
        ));
        assert!(model.steel_design_results.is_empty());
        model.materials.get_mut("A992Fy50").unwrap().mat_type = sap_model::MatType::ColdFormed;
        assert!(matches!(
            try_run_steel_design(&mut model),
            Err(DesignError::UnsupportedSteelMaterial { .. })
        ));
        model.materials.get_mut("A992Fy50").unwrap().mat_type = sap_model::MatType::Steel;
        model.frame_sections.get_mut("TEST_W").unwrap().prop_type = sap_model::FramePropType::Box;
        model
            .case_results
            .get_mut("DEAD")
            .unwrap()
            .frame_forces
            .get_mut("STEEL")
            .unwrap()[0]
            .t = 500.0;
        // With valid wall thickness on Box section, AISC H3-6 closed-section torsion succeeds:
        try_run_steel_design(&mut model).unwrap();
        assert_eq!(model.steel_design_results["STEEL"].eq_name, "H3-6");

        // When wall thickness t <= 0 is missing (or under EC3), closed-section torsion is rejected:
        model.frame_sections.get_mut("TEST_W").unwrap().tf = 0.0;
        assert!(matches!(
            try_run_steel_design(&mut model),
            Err(DesignError::UnsupportedClosedSectionTorsion { .. })
        ));
        assert!(model.steel_design_results.is_empty());
    }

    #[test]
    fn checked_design_rejects_nonfinite_demand_and_station_values() {
        let mut model = model_with_steel_frame();
        let mut case = CaseResults {
            case_name: "DEAD".to_string(),
            ..Default::default()
        };
        case.frame_forces.insert(
            "STEEL".to_string(),
            vec![FrameStationResult {
                station: f64::NAN,
                ..Default::default()
            }],
        );
        model.case_results.insert("DEAD".to_string(), case);

        assert!(matches!(
            try_run_steel_design(&mut model),
            Err(DesignError::NonFiniteDemand {
                component: "station",
                ..
            })
        ));
        assert!(model.steel_design_results.is_empty());

        model
            .case_results
            .get_mut("DEAD")
            .unwrap()
            .frame_forces
            .get_mut("STEEL")
            .unwrap()[0] = FrameStationResult {
            m3: f64::INFINITY,
            ..Default::default()
        };
        assert!(matches!(
            try_run_steel_design(&mut model),
            Err(DesignError::NonFiniteDemand {
                component: "M3",
                ..
            })
        ));
        assert!(model.steel_design_results.is_empty());
    }

    #[test]
    fn test_aisc360_w14x90_column_beam_column_sfd_benchmark() {
        let mut model = SapModel::new(Units::KipInF);
        model.steel_design_code = "AISC 360-10".to_string();

        // Load W14X90 from aisc14.pro if available, or construct exact AISC 14th Ed properties
        let pro_path = Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../extracted/files/aisc14.pro"
        ));
        let w14x90 = if pro_path.exists() {
            let db = ProDatabase::from_file(pro_path).unwrap();
            db.get_section_scaled("W14X90", Units::KipInF, "A992Fy50")
                .unwrap()
        } else {
            let mut sec =
                make_i_section("W14X90", "A992Fy50", 14.0, 14.5, 0.710, 0.440, 14.5, 0.710);
            sec.area = 26.5;
            sec.i33 = 999.0;
            sec.i22 = 362.0;
            sec.s33 = 143.0;
            sec.s22 = 49.9;
            sec.z33 = 157.0;
            sec.z22 = 75.6;
            sec.r33 = 6.14;
            sec.r22 = 3.70;
            sec.torsional_j = 4.06;
            sec
        };
        model.frame_sections.insert("W14X90".to_string(), w14x90);

        // 14 ft (168 in) column from (0,0,0) to (0,0,168)
        model.add_point("1", 0.0, 0.0, 0.0);
        model.add_point("2", 0.0, 0.0, 168.0);
        model.add_frame("C1", "1", "2", "W14X90");

        // Define two load combinations:
        // COMB1: High axial compression (Pr/Pc >= 0.2 -> triggers H1-1a)
        //   Pu = -400 kips (compression), Mu33 = 2500 kip-in, Mu22 = 600 kip-in, Vu2 = 50 kips
        let mut cr1 = CaseResults {
            case_name: "LC1".to_string(),
            ..Default::default()
        };
        cr1.frame_forces.insert(
            "C1".to_string(),
            vec![
                FrameStationResult {
                    station: 0.0,
                    rel_dist: 0.0,
                    p: -400.0,
                    v2: 50.0,
                    v3: 0.0,
                    t: 0.0,
                    m2: 600.0,
                    m3: 2500.0,
                    local_disp: [0.0; 3],
                },
                FrameStationResult {
                    station: 84.0,
                    rel_dist: 0.5,
                    p: -400.0,
                    v2: 50.0,
                    v3: 0.0,
                    t: 0.0,
                    m2: 600.0,
                    m3: 2500.0,
                    local_disp: [0.0; 3],
                },
                FrameStationResult {
                    station: 168.0,
                    rel_dist: 1.0,
                    p: -400.0,
                    v2: 50.0,
                    v3: 0.0,
                    t: 0.0,
                    m2: 600.0,
                    m3: 2500.0,
                    local_disp: [0.0; 3],
                },
            ],
        );
        model.case_results.insert("LC1".to_string(), cr1);
        model.load_combos.insert(
            "COMB_H1_1A".to_string(),
            LoadCombo {
                name: "COMB_H1_1A".to_string(),
                combo_type: ComboType::LinearAdd,
                cases: vec![("LC1".to_string(), 1.0)],
            },
        );

        run_steel_design(&mut model);

        let res = model
            .steel_design_results
            .get("C1")
            .expect("C1 design result must exist");
        assert_eq!(res.section_name, "W14X90");
        assert_eq!(res.governing_combo, "COMB_H1_1A");
        assert_eq!(res.eq_name, "H1-1a");
        assert_eq!(res.status, "OK");

        // Verify capacities against AISC 360-10 hand calculation:
        // phi_t * Pnt = 0.90 * 50 * 26.5 = 1192.5 kips
        assert!((res.phi_pnt - 1192.5).abs() < 15.0);
        // KL/r_y = 168 / 3.70 = 45.405 -> Fe = pi^2 * 29000 / 45.405^2 = 138.8 ksi
        // Fcr = 0.658^(50/138.8) * 50 = 43.01 ksi -> phi_c * Pnc = 0.90 * 43.01 * 26.5 = 1025.8 kips
        assert!(res.phi_pnc > 980.0 && res.phi_pnc < 1060.0);
        // Pr / Pc = 400 / 1025.8 = 0.390 >= 0.2
        assert!((res.pr_ratio - 400.0 / res.phi_pnc).abs() < 1e-6);
        let expected_total =
            res.pr_ratio + (8.0 / 9.0) * (2500.0 / res.phi_mn33 + 600.0 / res.phi_mn22);
        assert!((res.total_ratio - expected_total).abs() < 1e-6);
        assert!(res.total_ratio > 0.70 && res.total_ratio < 0.95);

        // Now test H1-1b when axial force is small (Pu = -50 kips -> Pr/Pc < 0.2)
        model
            .case_results
            .get_mut("LC1")
            .unwrap()
            .frame_forces
            .get_mut("C1")
            .unwrap()
            .iter_mut()
            .for_each(|s| s.p = -50.0);
        run_steel_design(&mut model);
        let res_b = model.steel_design_results.get("C1").unwrap();
        assert_eq!(res_b.eq_name, "H1-1b");
        let expected_b =
            (50.0 / (2.0 * res_b.phi_pnc)) + (2500.0 / res_b.phi_mn33 + 600.0 / res_b.phi_mn22);
        assert!((res_b.total_ratio - expected_b).abs() < 1e-6);
    }

    #[test]
    fn test_cb_and_alignment_chart_k_factors() {
        // Uniform moment -> Cb = 1.0
        let uniform = vec![
            FrameStationResult {
                station: 0.0,
                rel_dist: 0.0,
                m3: 100.0,
                ..Default::default()
            },
            FrameStationResult {
                station: 50.0,
                rel_dist: 0.5,
                m3: 100.0,
                ..Default::default()
            },
            FrameStationResult {
                station: 100.0,
                rel_dist: 1.0,
                m3: 100.0,
                ..Default::default()
            },
        ];
        assert!((compute_cb_from_stations(&uniform) - 1.0).abs() < 1e-9);

        // Linearly varying moment from 0 at I to M_max at J -> MA=0.25, MB=0.50, MC=0.75
        // Cb = 12.5 / (2.5 + 3*0.25 + 4*0.50 + 3*0.75) = 12.5 / 7.5 = 1.6666667
        let linear = vec![
            FrameStationResult {
                station: 0.0,
                rel_dist: 0.0,
                m3: 0.0,
                ..Default::default()
            },
            FrameStationResult {
                station: 100.0,
                rel_dist: 1.0,
                m3: 100.0,
                ..Default::default()
            },
        ];
        assert!((compute_cb_from_stations(&linear) - (12.5 / 7.5)).abs() < 1e-6);

        // Alignment chart K-factors:
        // Fixed-fixed braced: GA=0, GB=0 -> K = 0.5
        assert!((compute_k_braced(0.0, 0.0) - 0.5).abs() < 1e-4);
        // Pinned-pinned braced: GA=1e7, GB=1e7 -> K = 1.0
        assert!((compute_k_braced(1e7, 1e7) - 1.0).abs() < 1e-4);
        // Sway frame with GA=1.0, GB=1.0 -> K ≈ 1.32
        let k_sway = compute_k_sway(1.0, 1.0);
        assert!(k_sway > 1.30 && k_sway < 1.34);
    }

    #[test]
    fn test_eurocode3_steel_design() {
        let mut model = SapModel::new(Units::KipInF);
        model.steel_design_code = "Eurocode 3-2005".to_string();
        model.add_point("1", 0.0, 0.0, 0.0);
        model.add_point("2", 0.0, 0.0, 144.0);
        model.add_frame("B1", "1", "2", "FSEC1");

        let mut cr = CaseResults {
            case_name: "DEAD".to_string(),
            ..Default::default()
        };
        cr.frame_forces.insert(
            "B1".to_string(),
            vec![FrameStationResult {
                station: 72.0,
                rel_dist: 0.5,
                p: -150.0,
                v2: 30.0,
                m3: 1200.0,
                ..Default::default()
            }],
        );
        model.case_results.insert("DEAD".to_string(), cr);

        run_steel_design(&mut model);
        let res = model.steel_design_results.get("B1").unwrap();
        assert_eq!(res.eq_name, "EC3-6.2");
        assert!(res.phi_pnc > 0.0 && res.phi_mn33 > 0.0);
        assert!(res.total_ratio > 0.2 && res.total_ratio < 1.0);
        assert_eq!(res.status, "OK");
    }

    #[test]
    fn test_aci318_concrete_beam_and_column_cfd_benchmark() {
        let mut model = SapModel::new(Units::KipInF);
        model.concrete_design_code = "ACI 318-14".to_string();

        // 1. Concrete Beam section: 14 in wide x 24 in deep, cover = 2.5 in -> d = 21.5 in
        let mut beam_sec = make_rectangular("CBEAM14X24", "4000Psi", 24.0, 14.0);
        beam_sec.concrete_rebar = Some(ConcreteRebarConfig {
            is_column: false,
            rebar_mat_long: "4000Psi".to_string(),
            rebar_mat_confine: "4000Psi".to_string(),
            cover_top: 2.5,
            cover_bot: 2.5,
            num_bars_3dir: 0,
            num_bars_2dir: 0,
            total_rebar_area: 0.0,
        });
        model
            .frame_sections
            .insert("CBEAM14X24".to_string(), beam_sec);

        // 2. Concrete Column section: 20 in x 20 in, Ast = 8.0 in^2 (8 #9 bars)
        let mut col_sec = make_rectangular("CCOL20X20", "4000Psi", 20.0, 20.0);
        col_sec.concrete_rebar = Some(ConcreteRebarConfig {
            is_column: true,
            rebar_mat_long: "4000Psi".to_string(),
            rebar_mat_confine: "4000Psi".to_string(),
            cover_top: 2.5,
            cover_bot: 2.5,
            num_bars_3dir: 3,
            num_bars_2dir: 3,
            total_rebar_area: 8.0,
        });
        model
            .frame_sections
            .insert("CCOL20X20".to_string(), col_sec);

        // Add beam (horizontal) and column (vertical)
        model.add_point("1", 0.0, 0.0, 0.0);
        model.add_point("2", 0.0, 0.0, 144.0);
        model.add_point("3", 240.0, 0.0, 144.0);
        model.add_frame("COL1", "1", "2", "CCOL20X20");
        model.add_frame("BM1", "2", "3", "CBEAM14X24");

        // Populate factored station forces:
        // Beam BM1:
        //   End I (station 0): M3 = -3600 kip-in (top tension), V2 = 65 kips
        //   Midspan (station 120): M3 = +2400 kip-in (bottom tension), V2 = 20 kips
        // Column COL1:
        //   Base (station 0): P = -500 kips (compression), M3 = 2400 kip-in, M2 = 1200 kip-in, V2 = 30 kips
        let mut cr = CaseResults {
            case_name: "ULT".to_string(),
            ..Default::default()
        };
        cr.frame_forces.insert(
            "BM1".to_string(),
            vec![
                FrameStationResult {
                    station: 0.0,
                    rel_dist: 0.0,
                    p: 0.0,
                    v2: 65.0,
                    m3: -3600.0,
                    ..Default::default()
                },
                FrameStationResult {
                    station: 120.0,
                    rel_dist: 0.5,
                    p: 0.0,
                    v2: 20.0,
                    m3: 2400.0,
                    ..Default::default()
                },
            ],
        );
        cr.frame_forces.insert(
            "COL1".to_string(),
            vec![FrameStationResult {
                station: 0.0,
                rel_dist: 0.0,
                p: -500.0,
                v2: 30.0,
                m2: 1200.0,
                m3: 2400.0,
                ..Default::default()
            }],
        );
        model.case_results.insert("ULT".to_string(), cr);

        run_concrete_design(&mut model);

        // Verify Beam BM1 results against ACI 318-14 hand calculation (with phi_b = 0.90):
        // b = 14 in, d = 21.5 in, fc' = 4 ksi, fy = 60 ksi:
        // As_min = (200 / 60000) * 14 * 21.5 = 1.00333 in^2
        // Top moment Mu = 3600 kip-in:
        //   Rn = 3600 / (0.90 * 14 * 21.5^2) = 0.618095 ksi
        //   a = 21.5 * (1 - sqrt(1 - 2 * 0.618095 / (0.85 * 4.0))) = 4.34824 in
        //   As_top = 0.85 * 4.0 * 4.34824 * 14 / 60.0 = 3.4496 in^2
        // Bottom moment Mu = 2400 kip-in:
        //   Rn = 2400 / (0.90 * 14 * 21.5^2) = 0.412063 ksi
        //   a = 2.79053 in -> As_bot = 2.2138 in^2
        // Shear Vu = 65 kips:
        //   phi * Vc = 0.75 * 2 * sqrt(4000) * 14 * 21.5 / 1000 = 28.555 kips
        //   Av/s = (65 - 28.555) / (0.75 * 60 * 21.5) = 0.03767 in^2/in
        let bm = model
            .concrete_design_results
            .get("BM1")
            .expect("BM1 design result must exist");
        assert!(!bm.is_column);
        assert_eq!(bm.status, "OK");
        assert!((bm.min_rebar_area - 1.00333).abs() < 0.01);
        assert!((bm.as_top - 3.4496).abs() < 0.02);
        assert!((bm.as_bot - 2.2138).abs() < 0.02);
        assert!((bm.av_over_s - 0.03767).abs() < 0.002);

        // Verify Column COL1 results:
        let col = model
            .concrete_design_results
            .get("COL1")
            .expect("COL1 design result must exist");
        assert!(col.is_column);
        assert_eq!(col.status, "OK");
        assert!((col.min_rebar_area - 4.0).abs() < 1e-6); // 0.01 * 400 = 4.0 in^2
        assert!(col.pmm_ratio > 0.45 && col.pmm_ratio < 0.95);

        // Also test Eurocode 2-2004 concrete design on the same model
        model.concrete_design_code = "Eurocode 2-2004".to_string();
        run_concrete_design(&mut model);
        let bm_ec2 = model.concrete_design_results.get("BM1").unwrap();
        let col_ec2 = model.concrete_design_results.get("COL1").unwrap();
        assert_eq!(bm_ec2.status, "OK");
        assert!(bm_ec2.as_top > 3.0 && bm_ec2.as_bot > 2.0);
        assert_eq!(col_ec2.status, "OK");
        assert!(col_ec2.pmm_ratio > 0.35 && col_ec2.pmm_ratio < 0.90);
    }

    #[test]
    fn test_finding_7_and_8_circular_column_and_buckling_limit() {
        use sap_section::make_circle;

        let mut model = SapModel::new(Units::KipInF);

        // Compare 20" diameter circular column vs 20"x20" square column with identical Ast = 6.0 in^2
        let mut sq_sec = make_rectangular("SQ20", "4000Psi", 20.0, 20.0);
        let mut cir_sec = make_circle("CIR20", "4000Psi", 20.0);
        let rebar = ConcreteRebarConfig {
            is_column: true,
            rebar_mat_long: "4000Psi".into(),
            rebar_mat_confine: "4000Psi".into(),
            cover_top: 2.5,
            cover_bot: 2.5,
            num_bars_3dir: 3,
            num_bars_2dir: 3,
            total_rebar_area: 6.0,
        };
        sq_sec.concrete_rebar = Some(rebar.clone());
        cir_sec.concrete_rebar = Some(rebar);

        let (sq_curve, p0_sq, _, _) = generate_column_pm_curve(
            &sq_sec,
            4.0,
            60.0,
            29_000.0,
            Units::KipInF,
            ConcreteCode::Aci318,
            true,
        );
        let (cir_curve, p0_cir, _, _) = generate_column_pm_curve(
            &cir_sec,
            4.0,
            60.0,
            29_000.0,
            Units::KipInF,
            ConcreteCode::Aci318,
            true,
        );
        assert!(!sq_curve.is_empty() && !cir_curve.is_empty());

        // Pure axial compression capacity must reflect circular area pi*D^2/4 = 314.159 in^2 (not 400 in^2):
        // P0_cir = 0.85 * 4.0 * (pi * 100 - 6.0) + 60 * 6 = 1407.7415 kips -> phi * P0 = 0.75 * 1407.7415 = 1055.8061 kips
        let expected_phi_p0_cir =
            0.75 * (0.85 * 4.0 * (std::f64::consts::PI * 100.0 - 6.0) + 60.0 * 6.0);
        assert!(
            (p0_cir - expected_phi_p0_cir).abs() < 1e-4,
            "Expected circular column phi*P0 = {}, got {}",
            expected_phi_p0_cir,
            p0_cir
        );
        let unfactored_p0_cir = p0_cir / 0.75;
        let unfactored_p0_sq = p0_sq / 0.65;
        assert!(unfactored_p0_cir < 0.85 * unfactored_p0_sq);

        // Finding 8: Slender column with Pu >= 0.75 * Pc must fail with BucklingExceeded
        model.frame_sections.insert("CIR20".into(), cir_sec);
        model.add_point("1", 0.0, 0.0, 0.0);
        model.add_point("2", 0.0, 0.0, 600.0); // 50 ft slender column -> small Pc
        model.add_frame("COL_SLENDER", "1", "2", "CIR20");
        let mut cr = CaseResults {
            case_name: "ULT".into(),
            ..Default::default()
        };
        cr.frame_forces.insert(
            "COL_SLENDER".into(),
            vec![FrameStationResult {
                station: 0.0,
                rel_dist: 0.0,
                p: -2000.0, // Exceeds 0.75 * Pc
                m3: 500.0,
                ..Default::default()
            }],
        );
        model.case_results.insert("ULT".into(), cr);
        run_concrete_design(&mut model);
        let res = &model.concrete_design_results["COL_SLENDER"];
        assert_eq!(res.status, "BUCKLING_OVERSTRESSED");
        assert!(res.pmm_ratio >= 999.0);
    }

    #[test]
    fn test_finding_9_steel_shear_governing_and_envelope_srss_combos() {
        let mut model = SapModel::new(Units::KipInF);
        model.add_point("1", 0.0, 0.0, 0.0);
        model.add_point("2", 0.0, 0.0, 120.0);
        model.add_frame("B1", "1", "2", "FSEC1");

        // Case 1: High shear V2 = 5000 kips (fails shear) but tiny P and M
        let mut cr1 = CaseResults {
            case_name: "LC1".into(),
            ..Default::default()
        };
        cr1.frame_forces.insert(
            "B1".into(),
            vec![FrameStationResult {
                station: 0.0,
                rel_dist: 0.0,
                p: -10.0,
                v2: 5000.0,
                m3: 10.0,
                ..Default::default()
            }],
        );
        // Case 2: Moderate compression P = -200 kips
        let mut cr2 = CaseResults {
            case_name: "LC2".into(),
            ..Default::default()
        };
        cr2.frame_forces.insert(
            "B1".into(),
            vec![FrameStationResult {
                station: 0.0,
                rel_dist: 0.0,
                p: -200.0,
                v2: 10.0,
                m3: 100.0,
                ..Default::default()
            }],
        );
        model.case_results.insert("LC1".into(), cr1);
        model.case_results.insert("LC2".into(), cr2);
        model.load_combos.insert(
            "ENV1".into(),
            LoadCombo {
                name: "ENV1".into(),
                combo_type: ComboType::Envelope,
                cases: vec![("LC1".into(), 1.0), ("LC2".into(), 1.0)],
            },
        );

        run_steel_design(&mut model);
        let res = &model.steel_design_results["B1"];
        assert_eq!(
            res.status, "OVERSTRESSED",
            "Member failing in shear must report OVERSTRESSED"
        );
        assert!(res.total_ratio > 1.0);
    }

    #[test]
    fn test_aisc360_angle_and_dbl_angle_v3_shear_and_ec3_shapes() {
        let model = SapModel::new(Units::KipInF);
        let mat = &model.materials["A992Fy50"]; // E = 29000 ksi, Fy = 50 ksi

        // 1. Single Angle L6x4x1/2: b = t2 = 4.0 in, t = tf = 0.5 in, as3 = 0.0
        // b/t = 8.0 <= 1.10 * sqrt(1.2 * 29000 / 50) = 29.02895 -> Cv = 1.0
        // Av3 = 4.0 * 0.5 = 2.0 in^2, Vn3 = 0.6 * 50 * 2.0 = 60.0 kips
        // LRFD phi_v * Vn3 = 0.90 * 60.0 = 54.0 kips; ASD Vn3 / Omega_v = 60.0 / 1.67
        let mut angle = sap_model::FrameSection {
            name: "L6X4X1/2".into(),
            material: "A992Fy50".into(),
            prop_type: FramePropType::Angle,
            t3: 6.0,
            t2: 4.0,
            tf: 0.5,
            tw: 0.5,
            area: 4.75,
            i33: 17.4,
            i22: 6.27,
            s33: 4.33,
            s22: 2.0,
            z33: 7.5,
            z22: 3.5,
            r33: 1.91,
            r22: 1.15,
            torsional_j: 0.40,
            ..Default::default()
        };
        assert!((compute_aisc_minor_shear(&angle, mat, false).unwrap() - 54.0).abs() < 1e-10);
        assert!((compute_aisc_minor_shear(&angle, mat, true).unwrap() - 60.0 / 1.67).abs() < 1e-10);

        // 2. Double Angle 2L6x4x1/2: Av3 = 2 * 4.0 * 0.5 = 4.0 in^2 -> LRFD = 108.0 kips, ASD = 120.0 / 1.67
        angle.prop_type = FramePropType::DblAngle;
        assert!((compute_aisc_minor_shear(&angle, mat, false).unwrap() - 108.0).abs() < 1e-10);
        assert!(
            (compute_aisc_minor_shear(&angle, mat, true).unwrap() - 120.0 / 1.67).abs() < 1e-10
        );

        // 3. Slender-leg Angle with b/t = 40.0 > 1.37 * sqrt(1.2 * 29000 / 50) = 36.1542
        // Cv = 1.51 * 1.2 * 29000 / (40^2 * 50) = 0.65685
        let mut slender_angle = angle.clone();
        slender_angle.prop_type = FramePropType::Angle;
        slender_angle.t2 = 8.0;
        slender_angle.tf = 0.2;
        let expected_cv = (1.51 * 1.2 * 29000.0) / (40.0 * 40.0 * 50.0);
        let expected_lrfd = 0.90 * 0.60 * 50.0 * (8.0 * 0.2) * expected_cv;
        assert!(
            (compute_aisc_minor_shear(&slender_angle, mat, false).unwrap() - expected_lrfd).abs()
                < 1e-10
        );

        // 4. EC3 V3 now supports all standard shapes per EN 1993-1-1 6.2.6(3).
        let mut w_sec = make_i_section("W_EC3", "A992Fy50", 12.0, 8.0, 0.5, 0.3, 8.0, 0.5);
        w_sec.as3 = f64::NAN; // Analysis shear area must not be used as EC3 Av.
                              // For I: Av3 = 8.0 * 0.5 + 8.0 * 0.5 = 8.0 in^2 -> Vpl,Rd = 400 / sqrt(3)
        assert!(
            (compute_ec3_minor_shear(&w_sec, mat).unwrap() - 400.0 / 3.0_f64.sqrt()).abs() < 1e-10
        );

        let mut chan_sec = w_sec.clone();
        chan_sec.prop_type = FramePropType::Channel;
        // For Channel: Av3 = 2 * 8.0 * 0.5 = 8.0 in^2 -> Vpl,Rd = 400 / sqrt(3)
        assert!(
            (compute_ec3_minor_shear(&chan_sec, mat).unwrap() - 400.0 / 3.0_f64.sqrt()).abs()
                < 1e-10
        );

        let mut dbchan_sec = w_sec.clone();
        dbchan_sec.prop_type = FramePropType::DbChannel;
        // For DbChannel: Av3 = 4 * 8.0 * 0.5 = 16.0 in^2 -> Vpl,Rd = 800 / sqrt(3)
        assert!(
            (compute_ec3_minor_shear(&dbchan_sec, mat).unwrap() - 800.0 / 3.0_f64.sqrt()).abs()
                < 1e-10
        );

        let mut t_sec = w_sec.clone();
        t_sec.prop_type = FramePropType::T;
        // For T: Av3 = 8.0 * 0.5 = 4.0 in^2 -> Vpl,Rd = 200 / sqrt(3)
        assert!(
            (compute_ec3_minor_shear(&t_sec, mat).unwrap() - 200.0 / 3.0_f64.sqrt()).abs() < 1e-10
        );

        let mut box_sec = make_box_tube("BOX_EC3", "A992Fy50", 10.0, 6.0, 0.5, 0.5);
        box_sec.as3 = 1000.0; // Deliberately unlike the section's EC3 local-3 shear area.
                              // EN 1993-1-1 6.2.6(3)(f), V3 parallel to width: Av=A*b/(b+h).
        let expected_ec3_box = box_sec.area * 6.0 / (6.0 + 10.0) * 50.0 / 3.0_f64.sqrt();
        assert!((compute_ec3_minor_shear(&box_sec, mat).unwrap() - expected_ec3_box).abs() < 1e-10);
        let mut unequal_box = box_sec.clone();
        unequal_box.tw = 0.25;
        // Unequal walls box: 2 * b * tf = 2 * 6.0 * 0.5 = 6.0 in^2 -> Vpl,Rd = 300 / sqrt(3)
        let expected_unequal_box = 6.0 * 50.0 / 3.0_f64.sqrt();
        assert!(
            (compute_ec3_minor_shear(&unequal_box, mat).unwrap() - expected_unequal_box).abs()
                < 1e-10
        );

        let mut pipe_sec = make_pipe("PIPE_EC3", "A992Fy50", 10.0, 0.5);
        pipe_sec.as3 = 777.0; // Must not override EC3 6.2.6(3)(g) Av = (2/pi) * A.
        let expected_ec3_pipe =
            (2.0 / std::f64::consts::PI) * pipe_sec.area * 50.0 / 3.0_f64.sqrt();
        assert!(
            (compute_ec3_minor_shear(&pipe_sec, mat).unwrap() - expected_ec3_pipe).abs() < 1e-10
        );
        let mut circle_sec = sap_section::make_circle("CIRCLE_EC3", "A992Fy50", 10.0);
        circle_sec.as3 = 321.0;
        let expected_ec3_circle = 0.75 * circle_sec.area * 50.0 / 3.0_f64.sqrt();
        assert!(
            (compute_ec3_minor_shear(&circle_sec, mat).unwrap() - expected_ec3_circle).abs()
                < 1e-10
        );

        let mut unsupported = w_sec.clone();
        unsupported.prop_type = FramePropType::Auto;
        assert!(matches!(
            compute_ec3_minor_shear(&unsupported, mat),
            Err(DesignError::UnsupportedEc3MinorShearSection { .. })
        ));
    }

    #[test]
    fn test_aisc360_box_and_pipe_h3_torsion_and_h3_6_interaction() {
        let mut model = SapModel::new(Units::KipInF);
        model.steel_design_code = "AISC 360-10".to_string();
        let mat = model.materials["A992Fy50"].clone(); // E = 29000 ksi, Fy = 50 ksi

        // 1. Hand calculation for Pipe (Round HSS) D = 10 in, t = 0.5 in, L = 120 in:
        // C = pi * (10 - 0.5)^2 * 0.5 / 2 = pi * 90.25 * 0.25 = 70.882184 in^3
        // D/t = 20, L/D = 12 -> Fcr1 = 1.23*29000 / (sqrt(12)*20^1.25) = 243.54 ksi, Fcr2 = 0.60*29000 / 20^1.5 = 194.54 ksi
        // Compact yielding governs: Fcr = min(max(Fcr1, Fcr2), 0.6*50) = 30.0 ksi
        // Tn = 30.0 * 70.882184 = 2126.4655 kip-in
        // Tc (LRFD) = 0.90 * Tn = 1913.8190 kip-in; Tc (ASD) = Tn / 1.67 = 1273.3326 kip-in
        let pipe_sec = make_pipe("HSS10X0.5", "A992Fy50", 10.0, 0.5);
        let expected_c_pipe = 0.5 * std::f64::consts::PI * 9.5_f64.powi(2) * 0.5;
        let expected_tn_pipe = 30.0 * expected_c_pipe;
        let tc_pipe_lrfd = compute_aisc_torsional_capacity(&pipe_sec, &mat, 120.0, false).unwrap();
        let tc_pipe_asd = compute_aisc_torsional_capacity(&pipe_sec, &mat, 120.0, true).unwrap();
        assert!((tc_pipe_lrfd - 0.90 * expected_tn_pipe).abs() < 1e-8);
        assert!((tc_pipe_asd - expected_tn_pipe / 1.67).abs() < 1e-8);

        // 2. Hand calculation for Box (Rectangular HSS) H = 10 in, B = 6 in, t = 0.5 in:
        // C = 2*(6 - 0.5)*(10 - 0.5)*0.5 - 4.5*(4 - pi)*0.5^3 = 52.25 - 0.482854 = 51.767146 in^3
        // h/t = (10 - 1.5)/0.5 = 17.0 <= 2.45 * sqrt(29000/50) = 59.0025 -> Fcr = 0.6*50 = 30.0 ksi
        // Tc (LRFD) = 0.90 * 30.0 * C = 1397.7129 kip-in
        let box_sec = make_box_tube("HSS10X6X0.5", "A992Fy50", 10.0, 6.0, 0.5, 0.5);
        let expected_c_box = 2.0 * 5.5 * 9.5 * 0.5 - 4.5 * (4.0 - std::f64::consts::PI) * 0.125;
        let expected_tc_box = 0.90 * 30.0 * expected_c_box;
        let tc_box_lrfd = compute_aisc_torsional_capacity(&box_sec, &mat, 120.0, false).unwrap();
        assert!((tc_box_lrfd - expected_tc_box).abs() < 1e-8);

        // Unequal wall thicknesses use the matching clear dimensions, not min(t) twice.
        let mut unequal_box = box_sec.clone();
        unequal_box.name = "HSS10X6X0.5X0.25".into();
        unequal_box.tw = 0.25;
        unequal_box.tf = 0.50;
        let unequal_t = 0.25_f64;
        let unequal_c = 2.0 * (6.0 - 0.25) * (10.0 - 0.50) * unequal_t
            - 4.5 * (4.0 - std::f64::consts::PI) * unequal_t.powi(3);
        let unequal_expected = 0.90 * 30.0 * unequal_c;
        let unequal_tc = compute_aisc_torsional_capacity(&unequal_box, &mat, 120.0, false).unwrap();
        assert!((unequal_tc - unequal_expected).abs() < 1e-8);

        // Flat-width/flange and flat-depth/web ratios use the corresponding wall.
        // B=20, H=10, tf=1.0, tw=0.1 gives max(h/t)=max(19.7,70)=70,
        // in the inelastic branch between 2.45*sqrt(E/Fy) and 3.07*sqrt(E/Fy).
        let slender_unequal_box = make_box_tube("HSS20X10X1X0.1", "A992Fy50", 10.0, 20.0, 1.0, 0.1);
        let t = 0.1_f64;
        let c =
            2.0 * (20.0 - 0.1) * (10.0 - 1.0) * t - 4.5 * (4.0 - std::f64::consts::PI) * t.powi(3);
        let h_over_t = 70.0;
        let fcr = 0.60 * 50.0 * (2.45 * (29000.0_f64 / 50.0).sqrt()) / h_over_t;
        let expected_slender_tc = 0.90 * fcr * c;
        assert!(
            (compute_aisc_torsional_capacity(&slender_unequal_box, &mat, 120.0, false).unwrap()
                - expected_slender_tc)
                .abs()
                < 1e-8
        );

        // 3. Test AISC H3.2 Eq. H3-6 interaction in full model design run:
        model
            .frame_sections
            .insert(box_sec.name.clone(), box_sec.clone());
        model.add_point("1", 0.0, 0.0, 0.0);
        model.add_point("2", 0.0, 0.0, 120.0);
        model.add_frame("BOX_MEM", "1", "2", "HSS10X6X0.5");

        // Case A: Small torsion |Tr| = 0.15 * Tc <= 0.20 * Tc -> H3 interaction neglected, H1-1b governs
        let mut cr = CaseResults {
            case_name: "LC1".into(),
            ..Default::default()
        };
        cr.frame_forces.insert(
            "BOX_MEM".into(),
            vec![FrameStationResult {
                station: 60.0,
                rel_dist: 0.5,
                p: -50.0,
                v2: 30.0,
                v3: 10.0,
                t: 0.15 * expected_tc_box,
                m3: 500.0,
                m2: 200.0,
                ..Default::default()
            }],
        );
        model.case_results.insert("LC1".into(), cr);
        try_run_steel_design(&mut model).unwrap();
        assert_eq!(model.steel_design_results["BOX_MEM"].eq_name, "H1-1b");

        // Case B: Significant torsion |Tr| = 0.50 * Tc > 0.20 * Tc -> Eq. H3-6 governs!
        model
            .case_results
            .get_mut("LC1")
            .unwrap()
            .frame_forces
            .get_mut("BOX_MEM")
            .unwrap()[0]
            .t = 0.50 * expected_tc_box;
        try_run_steel_design(&mut model).unwrap();
        let res = &model.steel_design_results["BOX_MEM"];
        assert_eq!(res.eq_name, "H3-6");
        let expected_h3_ratio = (50.0 / res.phi_pnc + 500.0 / res.phi_mn33 + 200.0 / res.phi_mn22)
            + (res.shear_major_ratio + res.shear_minor_ratio + 0.50).powi(2);
        assert!((res.total_ratio - expected_h3_ratio).abs() < 1e-10);
        assert!((res.pr_ratio - 50.0 / res.phi_pnc).abs() < 1e-10);
        assert!((res.mr_major_ratio - 500.0 / res.phi_mn33).abs() < 1e-10);
        assert!((res.mr_minor_ratio - 200.0 / res.phi_mn22).abs() < 1e-10);
        assert!((res.shear_major_ratio - 30.0 / res.phi_vn2).abs() < 1e-10);
        assert!((res.shear_minor_ratio - 10.0 / res.phi_vn3).abs() < 1e-10);

        // Pipe H3-6 has squared individual flexural ratios (manual pp. 3-80) and
        // sums both directional shear ratios with torsion.
        let mut pipe_model = SapModel::new(Units::KipInF);
        pipe_model.steel_design_code = "AISC 360-10".into();
        let pipe = make_pipe("HSS10X0.5", "A992Fy50", 10.0, 0.5);
        pipe_model
            .frame_sections
            .insert(pipe.name.clone(), pipe.clone());
        pipe_model.add_point("PI", 0.0, 0.0, 0.0);
        pipe_model.add_point("PJ", 0.0, 0.0, 120.0);
        pipe_model.add_frame("PIPE_MEM", "PI", "PJ", &pipe.name);
        let tc = compute_aisc_torsional_capacity(&pipe, &mat, 120.0, false).unwrap();
        let mut pipe_case = CaseResults {
            case_name: "LC".into(),
            ..Default::default()
        };
        pipe_case.frame_forces.insert(
            "PIPE_MEM".into(),
            vec![FrameStationResult {
                station: 60.0,
                rel_dist: 0.5,
                p: -10.0,
                v2: 10.0,
                v3: 8.0,
                t: 0.50 * tc,
                m3: 100.0,
                m2: 80.0,
                ..Default::default()
            }],
        );
        pipe_model.case_results.insert("LC".into(), pipe_case);
        try_run_steel_design(&mut pipe_model).unwrap();
        let pipe_res = &pipe_model.steel_design_results["PIPE_MEM"];
        assert_eq!(pipe_res.eq_name, "H3-6");
        let expected_pipe_h3 = pipe_res.pr_ratio
            + pipe_res.mr_major_ratio.powi(2)
            + pipe_res.mr_minor_ratio.powi(2)
            + (pipe_res.shear_major_ratio + pipe_res.shear_minor_ratio + 0.50).powi(2);
        assert!((pipe_res.total_ratio - expected_pipe_h3).abs() < 1e-10);
    }

    #[test]
    fn test_aci318_and_ec2_concrete_minor_axis_v3_shear_and_m2_guard() {
        let mut model = SapModel::new(Units::KipInF);
        model.concrete_design_code = "ACI 318-14".to_string();

        // 14 in wide (t2) x 24 in deep (t3) concrete beam, cover = 2.5 in
        // Major shear V2 uses bw2 = 14 in, d2 = 21.5 in
        // Minor shear V3 uses bw3 = 24 in, d3 = 14 - 2.5 = 11.5 in
        let mut beam_sec = make_rectangular("RC_BM", "4000Psi", 24.0, 14.0);
        beam_sec.concrete_rebar = Some(ConcreteRebarConfig {
            is_column: false,
            rebar_mat_long: "4000Psi".into(),
            rebar_mat_confine: "4000Psi".into(),
            cover_top: 2.5,
            cover_bot: 2.5,
            num_bars_3dir: 0,
            num_bars_2dir: 0,
            total_rebar_area: 0.0,
        });
        model.frame_sections.insert("RC_BM".into(), beam_sec);

        // Circular concrete column D = 20 in -> bw = 20 in, dr = 0.8 * 20 = 16 in for both V2 and V3
        let mut cir_col = make_circle("RC_CIR20", "4000Psi", 20.0);
        cir_col.concrete_rebar = Some(ConcreteRebarConfig {
            is_column: true,
            rebar_mat_long: "4000Psi".into(),
            rebar_mat_confine: "4000Psi".into(),
            cover_top: 2.5,
            cover_bot: 2.5,
            num_bars_3dir: 3,
            num_bars_2dir: 3,
            total_rebar_area: 6.0,
        });
        model.frame_sections.insert("RC_CIR20".into(), cir_col);

        model.add_point("1", 0.0, 0.0, 0.0);
        model.add_point("2", 0.0, 0.0, 120.0);
        model.add_point("3", 240.0, 0.0, 120.0);
        model.add_frame("COL_CIR", "1", "2", "RC_CIR20");
        model.add_frame("BM_3D", "2", "3", "RC_BM");

        let mut cr = CaseResults {
            case_name: "ULT".into(),
            ..Default::default()
        };
        cr.frame_forces.insert(
            "BM_3D".into(),
            vec![FrameStationResult {
                station: 0.0,
                rel_dist: 0.0,
                v2: 20.0,
                v3: 55.0, // Minor shear governs over V2
                m3: 1200.0,
                m2: 0.0,
                ..Default::default()
            }],
        );
        cr.frame_forces.insert(
            "COL_CIR".into(),
            vec![FrameStationResult {
                station: 0.0,
                rel_dist: 0.0,
                p: -300.0,
                v2: 10.0,
                v3: 65.0, // Minor shear on circular column
                m3: 800.0,
                ..Default::default()
            }],
        );
        model.case_results.insert("ULT".into(), cr);

        // ACI 318-14 check:
        try_run_concrete_design(&mut model).unwrap();
        let bm_aci = &model.concrete_design_results["BM_3D"];
        let (expected_av_s2, _) = design_aci_beam_shear(20.0, 14.0, 21.5, 4.0, 60.0, Units::KipInF);
        let (expected_av_s3, _) = design_aci_beam_shear(55.0, 24.0, 11.5, 4.0, 60.0, Units::KipInF);
        assert!(expected_av_s3 > expected_av_s2);
        assert!((bm_aci.av_over_s - expected_av_s3).abs() < 1e-10);
        assert_eq!(bm_aci.status, "OK");

        let col_aci = &model.concrete_design_results["COL_CIR"];
        let (expected_cir_av_s3, _) =
            design_aci_beam_shear(65.0, 20.0, 16.0, 4.0, 60.0, Units::KipInF);
        assert!((col_aci.av_over_s - expected_cir_av_s3).abs() < 1e-10);
        assert_eq!(col_aci.status, "OK");

        // Eurocode 2 check on the same 3D demands:
        model.concrete_design_code = "Eurocode 2-2004".to_string();
        try_run_concrete_design(&mut model).unwrap();
        let bm_ec2 = &model.concrete_design_results["BM_3D"];
        let (_, _, expected_ec2_av_s2, _) = design_ec2_beam(
            1200.0,
            20.0,
            14.0,
            21.5,
            2.5,
            4.0,
            60.0,
            60.0,
            Units::KipInF,
        );
        let (_, _, expected_ec2_av_s3, _) =
            design_ec2_beam(0.0, 55.0, 24.0, 11.5, 2.5, 4.0, 60.0, 60.0, Units::KipInF);
        assert!((bm_ec2.av_over_s - expected_ec2_av_s2.max(expected_ec2_av_s3)).abs() < 1e-10);
        assert_eq!(bm_ec2.status, "OK");

        // M2 on beams produces side-face reinforcement as_side per DSGN-01.
        for design_code in ["ACI 318-14", "Eurocode 2-2004"] {
            model.concrete_design_code = design_code.into();
            model
                .case_results
                .get_mut("ULT")
                .unwrap()
                .frame_forces
                .get_mut("BM_3D")
                .unwrap()[0]
                .m2 = 1800.0;
            try_run_concrete_design(&mut model).unwrap();
            let res = &model.concrete_design_results["BM_3D"];
            assert!(res.as_side > 0.0);
            if design_code == "ACI 318-14" {
                let (expected_as_side, _, _) = design_aci_beam_flexure(
                    1800.0,
                    24.0,
                    11.5,
                    2.5,
                    4.0,
                    60.0,
                    29000.0,
                    Units::KipInF,
                );
                assert!((res.as_side - expected_as_side).abs() < 1e-10);
            } else {
                let (expected_as_side, _, _, _) =
                    design_ec2_beam(1800.0, 0.0, 24.0, 11.5, 2.5, 4.0, 60.0, 60.0, Units::KipInF);
                assert!((res.as_side - expected_as_side).abs() < 1e-10);
            }
            model
                .case_results
                .get_mut("ULT")
                .unwrap()
                .frame_forces
                .get_mut("BM_3D")
                .unwrap()[0]
                .m2 = 0.0;
        }

        // Verify minor flexure design across unit systems
        for (design_code, units) in [
            ("ACI 318-14", Units::KipInF),
            ("Eurocode 2-2004", Units::KnMmC),
        ] {
            let mut relative_model = model.clone();
            relative_model.concrete_design_code = design_code.into();
            let mut converted_results = relative_model.case_results.clone();
            relative_model.convert_units_in_place(units);
            let force_factor = Units::KipInF.force_factor_to(units);
            let moment_factor = Units::KipInF.moment_factor_to(units);
            let length_factor = Units::KipInF.length_factor_to(units);
            for case in converted_results.values_mut() {
                for stations in case.frame_forces.values_mut() {
                    for station in stations {
                        station.p *= force_factor;
                        station.v2 *= force_factor;
                        station.v3 *= force_factor;
                        station.t *= moment_factor;
                        station.m2 *= moment_factor;
                        station.m3 *= moment_factor;
                        station.station *= length_factor;
                        station
                            .local_disp
                            .iter_mut()
                            .for_each(|v| *v *= length_factor);
                    }
                }
            }
            relative_model.case_results = converted_results;
            let station = &mut relative_model
                .case_results
                .get_mut("ULT")
                .unwrap()
                .frame_forces
                .get_mut("BM_3D")
                .unwrap()[0];
            station.m2 = 0.0;
            try_run_concrete_design(&mut relative_model).unwrap();
            assert_eq!(relative_model.concrete_design_results["BM_3D"].as_side, 0.0);

            let station = &mut relative_model
                .case_results
                .get_mut("ULT")
                .unwrap()
                .frame_forces
                .get_mut("BM_3D")
                .unwrap()[0];
            station.m2 = 1800.0 * moment_factor;
            try_run_concrete_design(&mut relative_model).unwrap();
            assert!(relative_model.concrete_design_results["BM_3D"].as_side > 0.0);
        }

        // Verify biaxial shear overstress when V3 exceeds phi * Vn3,max under ACI 318-14:
        model.concrete_design_code = "ACI 318-14".to_string();
        model
            .case_results
            .get_mut("ULT")
            .unwrap()
            .frame_forces
            .get_mut("BM_3D")
            .unwrap()[0]
            .v3 = 500.0;
        try_run_concrete_design(&mut model).unwrap();
        assert_eq!(
            model.concrete_design_results["BM_3D"].status,
            "OVERSTRESSED"
        );
    }
}
