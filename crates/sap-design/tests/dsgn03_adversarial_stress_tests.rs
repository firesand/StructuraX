//! Adversarial empirical stress testing suite for DSGN-03:
//! Eurocode 3 local-3 shear and multi-axis interaction across all 10 supported shapes,
//! high-shear threshold transitions (0.499, 0.501, 0.99), biaxial high shear,
//! circular/pipe quadratic SRSS interaction, and boundary conditions.

use sap_design::{
    shear::{compute_ec3_major_shear, compute_ec3_minor_shear},
    try_run_steel_design, DesignError,
};
use sap_model::{CaseResults, FramePropType, FrameSection, FrameStationResult, SapModel};
use sap_section::{make_box_tube, make_circle, make_i_section, make_pipe, make_rectangular};
use sap_units::Units;

/// Helper to create a basic Eurocode 3 steel model with a single frame.
fn create_ec3_model_with_len(frame_name: &str, sec: FrameSection, length: f64) -> SapModel {
    let mut model = SapModel::new(Units::KipInF);
    model.steel_design_code = "Eurocode 3-2005".to_string();
    let sec_name = sec.name.clone();
    model.frame_sections.insert(sec_name.clone(), sec);
    model.add_point("1", 0.0, 0.0, 0.0);
    model.add_point("2", length, 0.0, 0.0);
    model.add_frame(frame_name, "1", "2", &sec_name);
    model
}

fn create_ec3_model(frame_name: &str, sec: FrameSection) -> SapModel {
    create_ec3_model_with_len(frame_name, sec, 120.0)
}

/// Helper to set station forces and run steel design.
fn run_design_with_forces(
    model: &mut SapModel,
    frame_name: &str,
    station_result: FrameStationResult,
) -> Result<sap_model::SteelDesignResult, DesignError> {
    let mut cr = CaseResults {
        case_name: "COMBO_TEST".into(),
        ..Default::default()
    };
    cr.frame_forces
        .insert(frame_name.to_string(), vec![station_result]);
    model.case_results.clear();
    model.case_results.insert("COMBO_TEST".into(), cr);
    try_run_steel_design(model)?;
    Ok(model.steel_design_results[frame_name].clone())
}

// -----------------------------------------------------------------------------
// 1. HIGH SHEAR MAJOR AXIS (V2) THRESHOLD TESTS (0.499, 0.501, 0.99)
// -----------------------------------------------------------------------------
#[test]
fn test_ec3_high_shear_v2_thresholds_and_continuity() {
    let sec = make_i_section("W14X90", "A992Fy50", 14.0, 10.0, 0.71, 0.44, 10.0, 0.71);
    // Use braced span (length = 20 in) so chi_LT = 1.0 and cross-section yield capacity
    // directly governs, isolating Section 6.2.8 high shear reductions from LTB masking.
    let mut model = create_ec3_model_with_len("F1", sec, 20.0);
    let sec_ref = &model.frame_sections["W14X90"];
    let mat = &model.materials["A992Fy50"];

    let v_pl_2 = compute_ec3_major_shear(sec_ref, mat).unwrap();
    let v_pl_3 = compute_ec3_minor_shear(sec_ref, mat).unwrap();

    let m3_demand = 3000.0;
    let m2_demand = 400.0;
    let p_demand = -50.0;

    // 1. Below threshold: V2 = 0.499 * Vpl,2 -> NO reduction
    let res_below = run_design_with_forces(
        &mut model,
        "F1",
        FrameStationResult {
            station: 10.0,
            rel_dist: 0.5,
            p: p_demand,
            v2: 0.499 * v_pl_2,
            v3: 0.10 * v_pl_3,
            m3: m3_demand,
            m2: m2_demand,
            ..Default::default()
        },
    )
    .unwrap();

    // 2. Just above threshold: V2 = 0.501 * Vpl,2 -> reduction triggers
    let res_above = run_design_with_forces(
        &mut model,
        "F1",
        FrameStationResult {
            station: 10.0,
            rel_dist: 0.5,
            p: p_demand,
            v2: 0.501 * v_pl_2,
            v3: 0.10 * v_pl_3,
            m3: m3_demand,
            m2: m2_demand,
            ..Default::default()
        },
    )
    .unwrap();

    // 3. High shear: V2 = 0.990 * Vpl,2 with high moment demand (M3=6000) so interaction governs over shear
    let res_high_interaction = run_design_with_forces(
        &mut model,
        "F1",
        FrameStationResult {
            station: 10.0,
            rel_dist: 0.5,
            p: p_demand,
            v2: 0.990 * v_pl_2,
            v3: 0.10 * v_pl_3,
            m3: 6000.0,
            m2: m2_demand,
            ..Default::default()
        },
    )
    .unwrap();

    // 4. High shear: V2 = 0.990 * Vpl,2 with low moment demand (M3=100) so shear governs over interaction
    let res_high_shear_gov = run_design_with_forces(
        &mut model,
        "F1",
        FrameStationResult {
            station: 10.0,
            rel_dist: 0.5,
            p: p_demand,
            v2: 0.990 * v_pl_2,
            v3: 0.10 * v_pl_3,
            m3: 100.0,
            m2: m2_demand,
            ..Default::default()
        },
    )
    .unwrap();

    // Assertions on equation names
    assert_eq!(
        res_below.eq_name, "EC3-6.2",
        "V2 <= 0.5 Vpl,2 must use unreduced EC3-6.2"
    );
    assert_eq!(
        res_above.eq_name, "EC3-6.2.6+6.2.8(3)",
        "V2 > 0.5 Vpl,2 must trigger EC3-6.2.6+6.2.8(3)"
    );
    assert_eq!(
        res_high_interaction.eq_name, "EC3-6.2.6+6.2.8(3)",
        "When interaction governs, eq must be EC3-6.2.6+6.2.8(3)"
    );
    assert_eq!(
        res_high_shear_gov.eq_name, "EC3-6.2.6 (V2)",
        "When shear ratio > interaction ratio, eq must be EC3-6.2.6 (V2)"
    );

    // Continuity verification across threshold:
    // Capacity at 0.501 is slightly less than at 0.499, difference should be tiny
    let delta_cap = (res_below.phi_mn33 - res_above.phi_mn33).abs();
    assert!(
        delta_cap < 0.10,
        "Continuity check across 0.5 threshold failed: delta = {}",
        delta_cap
    );
    assert!(
        res_above.phi_mn33 < res_below.phi_mn33,
        "Moment capacity at 0.501 must be reduced compared to 0.499: {} vs {}",
        res_above.phi_mn33,
        res_below.phi_mn33
    );

    // High shear reduction verification:
    // Capacity at 0.99 is significantly lower than unreduced capacity (7013.9 vs 7850)
    assert!(
        res_high_interaction.phi_mn33 < res_below.phi_mn33 * 0.92,
        "Heavy high-shear must noticeably reduce major moment capacity: unreduced={}, reduced={}",
        res_below.phi_mn33,
        res_high_interaction.phi_mn33
    );
    assert!(
        res_high_interaction.total_ratio > res_below.total_ratio,
        "Interaction ratio must increase under high shear reduction: {} vs {}",
        res_high_interaction.total_ratio,
        res_below.total_ratio
    );
}

// -----------------------------------------------------------------------------
// 2. HIGH SHEAR MINOR AXIS (V3) THRESHOLD TESTS (0.499, 0.501, 0.99)
// -----------------------------------------------------------------------------
#[test]
fn test_ec3_high_shear_v3_thresholds_and_continuity() {
    let sec = make_i_section("W14X90", "A992Fy50", 14.0, 10.0, 0.71, 0.44, 10.0, 0.71);
    let mut model = create_ec3_model("F1", sec);
    let sec_ref = &model.frame_sections["W14X90"];
    let mat = &model.materials["A992Fy50"];

    let v_pl_2 = compute_ec3_major_shear(sec_ref, mat).unwrap();
    let v_pl_3 = compute_ec3_minor_shear(sec_ref, mat).unwrap();

    let m3_demand = 800.0;
    let m2_demand = 700.0;
    let p_demand = -50.0;

    // 1. Below threshold: V3 = 0.499 * Vpl,3 -> NO reduction
    let res_below = run_design_with_forces(
        &mut model,
        "F1",
        FrameStationResult {
            station: 60.0,
            rel_dist: 0.5,
            p: p_demand,
            v2: 0.10 * v_pl_2,
            v3: 0.499 * v_pl_3,
            m3: m3_demand,
            m2: m2_demand,
            ..Default::default()
        },
    )
    .unwrap();

    // 2. Just above threshold: V3 = 0.501 * Vpl,3 -> reduction triggers
    let res_above = run_design_with_forces(
        &mut model,
        "F1",
        FrameStationResult {
            station: 60.0,
            rel_dist: 0.5,
            p: p_demand,
            v2: 0.10 * v_pl_2,
            v3: 0.501 * v_pl_3,
            m3: m3_demand,
            m2: m2_demand,
            ..Default::default()
        },
    )
    .unwrap();

    // 3. High shear: V3 = 0.990 * Vpl,3 -> significant reduction
    let res_high = run_design_with_forces(
        &mut model,
        "F1",
        FrameStationResult {
            station: 60.0,
            rel_dist: 0.5,
            p: p_demand,
            v2: 0.10 * v_pl_2,
            v3: 0.990 * v_pl_3,
            m3: m3_demand,
            m2: m2_demand,
            ..Default::default()
        },
    )
    .unwrap();

    assert_eq!(res_below.eq_name, "EC3-6.2");
    assert_eq!(res_above.eq_name, "EC3-6.2.6+6.2.8(3)");
    assert_eq!(res_high.eq_name, "EC3-6.2.6+6.2.8(3)");

    // Continuity verification across 0.5 minor shear threshold
    let delta_cap = (res_below.phi_mn22 - res_above.phi_mn22).abs();
    assert!(
        delta_cap < 0.10,
        "Minor axis threshold continuity check failed: delta = {}",
        delta_cap
    );
    assert!(
        res_above.phi_mn22 < res_below.phi_mn22,
        "Minor moment capacity at 0.501 must be reduced: {} vs {}",
        res_above.phi_mn22,
        res_below.phi_mn22
    );

    // High shear reduction verification
    assert!(
        res_high.phi_mn22 < res_below.phi_mn22 * 0.90,
        "Minor moment capacity at 0.99 must be reduced: unreduced={}, reduced={}",
        res_below.phi_mn22,
        res_high.phi_mn22
    );
    assert!(res_high.total_ratio > res_below.total_ratio);
}

// -----------------------------------------------------------------------------
// 3. BIAXIAL SIMULTANEOUS HIGH SHEAR (V2 > 0.5 Vpl,2 AND V3 > 0.5 Vpl,3)
// -----------------------------------------------------------------------------
#[test]
fn test_ec3_biaxial_simultaneous_high_shear_interaction() {
    let sec = make_i_section("W14X90", "A992Fy50", 14.0, 10.0, 0.71, 0.44, 10.0, 0.71);
    // Use braced span (length = 20 in) so chi_LT = 1.0 and both cross-section yield capacities
    // directly govern without LTB masking.
    let mut model = create_ec3_model_with_len("F1", sec, 20.0);
    let sec_ref = &model.frame_sections["W14X90"];
    let mat = &model.materials["A992Fy50"];

    let v_pl_2 = compute_ec3_major_shear(sec_ref, mat).unwrap();
    let v_pl_3 = compute_ec3_minor_shear(sec_ref, mat).unwrap();

    // Baseline: low shear in both directions (V2 = 0.2 Vpl,2, V3 = 0.2 Vpl,3)
    let res_base = run_design_with_forces(
        &mut model,
        "F1",
        FrameStationResult {
            station: 10.0,
            rel_dist: 0.5,
            p: -80.0,
            v2: 0.20 * v_pl_2,
            v3: 0.20 * v_pl_3,
            m3: 1500.0,
            m2: 400.0,
            ..Default::default()
        },
    )
    .unwrap();

    // Biaxial simultaneous high shear (V2 = 0.75 Vpl,2, V3 = 0.75 Vpl,3)
    // Demands M3=3500, M2=900 ensure interaction ratio (0.957) > shear ratio (0.750),
    // so EC3-6.2.6+6.2.8(3) governs over pure shear.
    let res_biaxial_high = run_design_with_forces(
        &mut model,
        "F1",
        FrameStationResult {
            station: 10.0,
            rel_dist: 0.5,
            p: -80.0,
            v2: 0.75 * v_pl_2,
            v3: 0.75 * v_pl_3,
            m3: 3500.0,
            m2: 900.0,
            ..Default::default()
        },
    )
    .unwrap();

    // Both moment capacities must be simultaneously reduced
    assert!(
        res_biaxial_high.phi_mn33 < res_base.phi_mn33,
        "Major capacity must be reduced: biaxial={}, base={}",
        res_biaxial_high.phi_mn33,
        res_base.phi_mn33
    );
    assert!(
        res_biaxial_high.phi_mn22 < res_base.phi_mn22,
        "Minor capacity must be reduced: biaxial={}, base={}",
        res_biaxial_high.phi_mn22,
        res_base.phi_mn22
    );
    assert_eq!(res_biaxial_high.eq_name, "EC3-6.2.6+6.2.8(3)");

    // Theoretical manual calculation check:
    // r_v2 = 0.75 -> rho2 = (2*0.75 - 1)^2 = 0.5^2 = 0.25
    // r_v3 = 0.75 -> rho3 = (2*0.75 - 1)^2 = 0.5^2 = 0.25
    // For W14X90:
    // hw = 14.0 - 2 * 0.71 = 12.58; aw = 12.58 * 0.44 = 5.5352;
    // reduced_m3 = (z33 - rho2 * aw^2 / (4 * tw)) * fy
    // reduced_m2 = (z22 - 2 * rho3 * af^2 / (4 * tf)) * fy where af = 10 * 0.71 = 7.1
    let pc = res_biaxial_high.phi_pnc;
    let expected_total =
        (80.0 / pc) + (3500.0 / res_biaxial_high.phi_mn33) + (900.0 / res_biaxial_high.phi_mn22);

    assert!(
        (res_biaxial_high.total_ratio - expected_total).abs() < 1e-6,
        "Biaxial high-shear total ratio parity mismatch: got {}, expected {}",
        res_biaxial_high.total_ratio,
        expected_total
    );

    // If moment demands are low, shear ratio (0.75) governs over interaction ratio
    let res_shear_gov = run_design_with_forces(
        &mut model,
        "F1",
        FrameStationResult {
            station: 10.0,
            rel_dist: 0.5,
            p: -80.0,
            v2: 0.75 * v_pl_2,
            v3: 0.75 * v_pl_3,
            m3: 100.0,
            m2: 100.0,
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(
        res_shear_gov.eq_name, "EC3-6.2.6 (V2)",
        "When shear ratio > interaction ratio, shear eq must govern"
    );

    // Also test an unbraced frame (length = 120 in) where Lateral Torsional Buckling
    // governs major axis (Mb,Rd < My,V,Rd) while high shear still reduces minor axis M2
    let sec_unbraced = make_i_section("W14X90_UNB", "A992Fy50", 14.0, 10.0, 0.71, 0.44, 10.0, 0.71);
    let mut model_unbraced = create_ec3_model_with_len("F_UNB", sec_unbraced, 120.0);
    let res_unb = run_design_with_forces(
        &mut model_unbraced,
        "F_UNB",
        FrameStationResult {
            station: 60.0,
            rel_dist: 0.5,
            p: -80.0,
            v2: 0.75 * v_pl_2,
            v3: 0.75 * v_pl_3,
            m3: 3000.0,
            m2: 600.0,
            ..Default::default()
        },
    )
    .unwrap();
    // In unbraced case, major axis is capped by Mb,Rd (5054.12 < 7632.40)
    assert!(
        (res_unb.phi_mn33 - 5054.116813231704).abs() < 1e-6,
        "Unbraced major capacity must be governed by Lateral Torsional Buckling Mb,Rd"
    );
    // While minor axis is still reduced by rho3 (2056.25 < 2500)
    assert!(
        res_unb.phi_mn22 < 2500.0,
        "Unbraced minor capacity must be reduced by rho3"
    );
    assert_eq!(res_unb.eq_name, "EC3-6.2.6+6.2.8(3)");
}

// -----------------------------------------------------------------------------
// 4. ALL 10 SUPPORTED SECTION TYPES UNDER COMBINED 3D FORCE DEMANDS
// -----------------------------------------------------------------------------
#[test]
fn test_ec3_all_10_supported_section_types_under_3d_demands() {
    let mat_name = "A992Fy50";

    // Helper to build sections
    let mut sections: Vec<(&'static str, FrameSection)> = Vec::new();

    // 1. I section
    let sec_i = make_i_section("SEC_I", mat_name, 14.0, 10.0, 0.6, 0.4, 10.0, 0.6);
    sections.push(("I", sec_i));

    // 2. Channel section
    let mut sec_chan = make_i_section("SEC_CHANNEL", mat_name, 12.0, 4.0, 0.5, 0.35, 4.0, 0.5);
    sec_chan.prop_type = FramePropType::Channel;
    sections.push(("Channel", sec_chan));

    // 3. Double Channel section
    let mut sec_dbchan = make_i_section("SEC_DBCHANNEL", mat_name, 12.0, 4.0, 0.5, 0.35, 4.0, 0.5);
    sec_dbchan.prop_type = FramePropType::DbChannel;
    sec_dbchan.area *= 2.0;
    sec_dbchan.i33 *= 2.0;
    sec_dbchan.i22 *= 2.0;
    sec_dbchan.s33 *= 2.0;
    sec_dbchan.s22 *= 2.0;
    sec_dbchan.z33 *= 2.0;
    sec_dbchan.z22 *= 2.0;
    sections.push(("DbChannel", sec_dbchan));

    // 4. Tee section
    let mut sec_t = make_i_section("SEC_T", mat_name, 7.0, 8.0, 0.5, 0.3, 0.0, 0.0);
    sec_t.prop_type = FramePropType::T;
    sections.push(("T", sec_t));

    // 5. Angle section
    let mut sec_angle = make_i_section("SEC_ANGLE", mat_name, 6.0, 4.0, 0.5, 0.5, 0.0, 0.0);
    sec_angle.prop_type = FramePropType::Angle;
    sections.push(("Angle", sec_angle));

    // 6. Double Angle section
    let mut sec_dbl_angle = make_i_section("SEC_DBLANGLE", mat_name, 6.0, 4.0, 0.5, 0.5, 0.0, 0.0);
    sec_dbl_angle.prop_type = FramePropType::DblAngle;
    sec_dbl_angle.area *= 2.0;
    sec_dbl_angle.i33 *= 2.0;
    sec_dbl_angle.i22 *= 2.0;
    sec_dbl_angle.s33 *= 2.0;
    sec_dbl_angle.s22 *= 2.0;
    sec_dbl_angle.z33 *= 2.0;
    sec_dbl_angle.z22 *= 2.0;
    sections.push(("DblAngle", sec_dbl_angle));

    // 7. Box section (uniform wall)
    let sec_box = make_box_tube("SEC_BOX", mat_name, 12.0, 8.0, 0.5, 0.5);
    sections.push(("Box", sec_box));

    // 8. Pipe section
    let sec_pipe = make_pipe("SEC_PIPE", mat_name, 10.0, 0.5);
    sections.push(("Pipe", sec_pipe));

    // 9. Circle section
    let sec_circle = make_circle("SEC_CIRCLE", mat_name, 8.0);
    sections.push(("Circle", sec_circle));

    // 10. Rectangular section
    let sec_rect = make_rectangular("SEC_RECT", mat_name, 10.0, 6.0);
    sections.push(("Rectangular", sec_rect));

    assert_eq!(sections.len(), 10, "All 10 shape families must be tested");

    for (shape_name, sec) in sections {
        let frame_name = format!("F_{shape_name}");
        let sec_name = sec.name.clone();
        let mut model = create_ec3_model(&frame_name, sec);

        let sec_ref = &model.frame_sections[&sec_name];
        let mat = &model.materials[mat_name];

        let v_pl_2 = compute_ec3_major_shear(sec_ref, mat)
            .unwrap_or_else(|e| panic!("compute_ec3_major_shear failed for {shape_name}: {e:?}"));
        let v_pl_3 = compute_ec3_minor_shear(sec_ref, mat)
            .unwrap_or_else(|e| panic!("compute_ec3_minor_shear failed for {shape_name}: {e:?}"));

        assert!(
            v_pl_2 > 0.0 && v_pl_2.is_finite(),
            "Invalid Vpl,2 for {shape_name}: {v_pl_2}"
        );
        assert!(
            v_pl_3 > 0.0 && v_pl_3.is_finite(),
            "Invalid Vpl,3 for {shape_name}: {v_pl_3}"
        );

        // Apply concurrent 3D force demands: P, V2, V3, M3, M2
        let st = FrameStationResult {
            station: 60.0,
            rel_dist: 0.5,
            p: -30.0,
            v2: 0.15 * v_pl_2,
            v3: 0.15 * v_pl_3,
            m3: 200.0,
            m2: 100.0,
            t: 0.0, // Open shapes have T=0; closed have no T here to avoid separate DSGN-02 check
            ..Default::default()
        };

        let res = run_design_with_forces(&mut model, &frame_name, st)
            .unwrap_or_else(|e| panic!("EC3 design run failed for {shape_name}: {e:?}"));

        assert!(
            res.total_ratio > 0.0 && res.total_ratio.is_finite(),
            "Non-finite total ratio for {shape_name}: {}",
            res.total_ratio
        );
        assert!(
            res.phi_vn2 > 0.0 && res.phi_vn2.is_finite(),
            "Non-finite phi_vn2 for {shape_name}"
        );
        assert!(
            res.phi_vn3 > 0.0 && res.phi_vn3.is_finite(),
            "Non-finite phi_vn3 for {shape_name}"
        );
        assert!(
            res.phi_mn33 > 0.0 && res.phi_mn33.is_finite(),
            "Non-finite phi_mn33 for {shape_name}"
        );
        assert!(
            res.phi_mn22 > 0.0 && res.phi_mn22.is_finite(),
            "Non-finite phi_mn22 for {shape_name}"
        );
        assert!(
            res.phi_pnc > 0.0 && res.phi_pnc.is_finite(),
            "Non-finite phi_pnc for {shape_name}"
        );
        assert!(
            res.phi_pnt > 0.0 && res.phi_pnt.is_finite(),
            "Non-finite phi_pnt for {shape_name}"
        );
        assert!(
            res.status == "OK" || res.status == "OVERSTRESSED" || res.status == "SLENDER",
            "Unexpected status for {shape_name}: {}",
            res.status
        );
    }
}

// -----------------------------------------------------------------------------
// 5. CIRCULAR & PIPE SRSS QUADRATIC MOMENT INTERACTION VERIFICATION
// -----------------------------------------------------------------------------
#[test]
fn test_ec3_circular_and_pipe_srss_quadratic_invariance() {
    let mat_name = "A992Fy50";

    for is_pipe in [true, false] {
        let (sec_name, sec) = if is_pipe {
            ("PIPE_SRSS", make_pipe("PIPE_SRSS", mat_name, 12.0, 0.5))
        } else {
            ("CIRCLE_SRSS", make_circle("CIRCLE_SRSS", mat_name, 10.0))
        };

        let mut model = create_ec3_model("F1", sec);

        // Test at multiple orientations with resultant moment M_res = 500 kip-in:
        // Angle 0 deg:   M3 = 500, M2 = 0
        // Angle 90 deg:  M3 = 0,   M2 = 500
        // Angle 36.87 deg: M3 = 400, M2 = 300 (3-4-5 triangle)
        // Angle 53.13 deg: M3 = 300, M2 = 400
        // Angle 45 deg:  M3 = 500 / sqrt(2), M2 = 500 / sqrt(2)
        let sqrt2 = 2.0_f64.sqrt();
        let moment_cases = vec![
            (500.0, 0.0),
            (0.0, 500.0),
            (400.0, 300.0),
            (300.0, 400.0),
            (500.0 / sqrt2, 500.0 / sqrt2),
        ];

        let mut baseline_ratio: Option<f64> = None;

        for (m3, m2) in moment_cases {
            let res = run_design_with_forces(
                &mut model,
                "F1",
                FrameStationResult {
                    station: 60.0,
                    rel_dist: 0.5,
                    p: -40.0,
                    m3,
                    m2,
                    ..Default::default()
                },
            )
            .unwrap();

            // Moment capacity for circular section should be isotropic (Mc3 == Mc2)
            assert!(
                (res.phi_mn33 - res.phi_mn22).abs() < 1e-6,
                "Circular section must have symmetric capacities: {} vs {}",
                res.phi_mn33,
                res.phi_mn22
            );

            let expected_ratio = (40.0 / res.phi_pnc)
                + ((m3 / res.phi_mn33).powi(2) + (m2 / res.phi_mn22).powi(2)).sqrt();

            assert!(
                (res.total_ratio - expected_ratio).abs() < 1e-8,
                "SRSS total ratio parity mismatch: got {}, expected {}",
                res.total_ratio,
                expected_ratio
            );

            // Invariance check: for all angles with resultant moment 500, the ratio must be identical!
            if let Some(base) = baseline_ratio {
                assert!(
                    (res.total_ratio - base).abs() < 1e-8,
                    "Rotational invariance failed for circular section {sec_name}: angle ratio {} != base {}",
                    res.total_ratio,
                    base
                );
            } else {
                baseline_ratio = Some(res.total_ratio);
            }
        }
    }
}

// -----------------------------------------------------------------------------
// 6. ADVERSARIAL BOUNDARY CHECKS (ZERO FORCES, EXTREME VALUES, NAN, DIV-BY-ZERO)
// -----------------------------------------------------------------------------
#[test]
fn test_ec3_adversarial_boundaries_and_extreme_conditions() {
    let sec = make_i_section("W14X90", "A992Fy50", 14.0, 10.0, 0.71, 0.44, 10.0, 0.71);
    let mut model = create_ec3_model("F1", sec);
    let sec_ref = &model.frame_sections["W14X90"];
    let mat = &model.materials["A992Fy50"];

    let v_pl_2 = compute_ec3_major_shear(sec_ref, mat).unwrap();
    let v_pl_3 = compute_ec3_minor_shear(sec_ref, mat).unwrap();

    // 1. Boundary: Completely zero demands (P=0, V2=0, V3=0, M3=0, M2=0, T=0)
    let res_zero = run_design_with_forces(
        &mut model,
        "F1",
        FrameStationResult {
            station: 60.0,
            rel_dist: 0.5,
            p: 0.0,
            v2: 0.0,
            v3: 0.0,
            m3: 0.0,
            m2: 0.0,
            t: 0.0,
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(res_zero.total_ratio, 0.0);
    assert_eq!(res_zero.pr_ratio, 0.0);
    assert_eq!(res_zero.mr_major_ratio, 0.0);
    assert_eq!(res_zero.mr_minor_ratio, 0.0);
    assert_eq!(res_zero.shear_major_ratio, 0.0);
    assert_eq!(res_zero.shear_minor_ratio, 0.0);
    assert_eq!(res_zero.status, "OK");
    assert_eq!(res_zero.eq_name, "EC3-6.2");

    // 2. Boundary: Infinitesimal demands (1e-15)
    let res_tiny = run_design_with_forces(
        &mut model,
        "F1",
        FrameStationResult {
            station: 60.0,
            rel_dist: 0.5,
            p: 1e-15,
            v2: 1e-15,
            v3: 1e-15,
            m3: 1e-15,
            m2: 1e-15,
            t: 0.0,
            ..Default::default()
        },
    )
    .unwrap();
    assert!(res_tiny.total_ratio >= 0.0 && res_tiny.total_ratio.is_finite());
    assert_eq!(res_tiny.status, "OK");

    // 3. Boundary: Massive demands exceeding plastic capacity by 10x
    let res_extreme = run_design_with_forces(
        &mut model,
        "F1",
        FrameStationResult {
            station: 60.0,
            rel_dist: 0.5,
            p: -50000.0,
            v2: 10.0 * v_pl_2,
            v3: 10.0 * v_pl_3,
            m3: 1000000.0,
            m2: 1000000.0,
            t: 0.0,
            ..Default::default()
        },
    )
    .unwrap();
    assert!(res_extreme.total_ratio.is_finite());
    assert!(!res_extreme.total_ratio.is_nan());
    assert!(res_extreme.total_ratio > 10.0);
    assert_eq!(res_extreme.status, "OVERSTRESSED");
    // Under extreme shear, capacity floor must be positive and not panic
    assert!(res_extreme.phi_mn33 > 0.0 && res_extreme.phi_mn33.is_finite());
    assert!(res_extreme.phi_mn22 > 0.0 && res_extreme.phi_mn22.is_finite());

    // 4. Boundary: Axial Tension vs Compression
    // Tension uses Nt,Rd = A * fy, Compression uses Nb,Rd <= Nt,Rd
    let res_tension = run_design_with_forces(
        &mut model,
        "F1",
        FrameStationResult {
            station: 60.0,
            rel_dist: 0.5,
            p: 100.0, // Tension
            ..Default::default()
        },
    )
    .unwrap();
    let res_compression = run_design_with_forces(
        &mut model,
        "F1",
        FrameStationResult {
            station: 60.0,
            rel_dist: 0.5,
            p: -100.0, // Compression
            ..Default::default()
        },
    )
    .unwrap();
    assert!(
        res_compression.phi_pnc <= res_tension.phi_pnt,
        "Compression resistance Nb,Rd must be <= Tension resistance Nt,Rd"
    );
    assert!(
        res_compression.total_ratio >= res_tension.total_ratio,
        "Compression ratio must be >= tension ratio for same axial magnitude"
    );

    // 5. Boundary: Sign symmetry of flexural demands
    let res_pos_m = run_design_with_forces(
        &mut model,
        "F1",
        FrameStationResult {
            station: 60.0,
            rel_dist: 0.5,
            m3: 1200.0,
            m2: 300.0,
            ..Default::default()
        },
    )
    .unwrap();
    let res_neg_m = run_design_with_forces(
        &mut model,
        "F1",
        FrameStationResult {
            station: 60.0,
            rel_dist: 0.5,
            m3: -1200.0,
            m2: -300.0,
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(
        res_pos_m.total_ratio, res_neg_m.total_ratio,
        "Moment sign symmetry must be exact"
    );

    // 6. Boundary: Sign symmetry of shear demands
    let res_pos_v = run_design_with_forces(
        &mut model,
        "F1",
        FrameStationResult {
            station: 60.0,
            rel_dist: 0.5,
            v2: 0.6 * v_pl_2,
            v3: 0.6 * v_pl_3,
            ..Default::default()
        },
    )
    .unwrap();
    let res_neg_v = run_design_with_forces(
        &mut model,
        "F1",
        FrameStationResult {
            station: 60.0,
            rel_dist: 0.5,
            v2: -0.6 * v_pl_2,
            v3: -0.6 * v_pl_3,
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(
        res_pos_v.total_ratio, res_neg_v.total_ratio,
        "Shear sign symmetry must be exact"
    );

    // 7. Boundary: Unequal wall box section
    let mut box_uneq = make_box_tube("BOX_UNEQ", "A992Fy50", 12.0, 8.0, 0.5, 0.25);
    box_uneq.tw = 0.25;
    let mut model_box = create_ec3_model("F_BOX", box_uneq);
    let res_box = run_design_with_forces(
        &mut model_box,
        "F_BOX",
        FrameStationResult {
            station: 60.0,
            rel_dist: 0.5,
            v2: 30.0,
            v3: 20.0,
            m3: 300.0,
            m2: 200.0,
            ..Default::default()
        },
    )
    .unwrap();
    assert!(res_box.total_ratio > 0.0 && res_box.total_ratio.is_finite());
    assert!(res_box.phi_vn2 > 0.0 && res_box.phi_vn3 > 0.0);

    // 8. Boundary: ColdBox and ColdPipe
    let mut cold_box = make_box_tube("COLD_BOX", "A992Fy50", 10.0, 6.0, 0.5, 0.5);
    cold_box.prop_type = FramePropType::ColdBox;
    let mut model_cold_box = create_ec3_model("F_COLD_BOX", cold_box);
    let res_cold_box = run_design_with_forces(
        &mut model_cold_box,
        "F_COLD_BOX",
        FrameStationResult {
            station: 60.0,
            rel_dist: 0.5,
            v2: 20.0,
            v3: 15.0,
            m3: 250.0,
            m2: 150.0,
            ..Default::default()
        },
    )
    .unwrap();
    assert!(res_cold_box.total_ratio > 0.0 && res_cold_box.total_ratio.is_finite());

    let mut cold_pipe = make_pipe("COLD_PIPE", "A992Fy50", 8.0, 0.4);
    cold_pipe.prop_type = FramePropType::ColdPipe;
    let mut model_cold_pipe = create_ec3_model("F_COLD_PIPE", cold_pipe);
    let res_cold_pipe = run_design_with_forces(
        &mut model_cold_pipe,
        "F_COLD_PIPE",
        FrameStationResult {
            station: 60.0,
            rel_dist: 0.5,
            v2: 20.0,
            v3: 15.0,
            m3: 250.0,
            m2: 150.0,
            ..Default::default()
        },
    )
    .unwrap();
    assert!(res_cold_pipe.total_ratio > 0.0 && res_cold_pipe.total_ratio.is_finite());
}

// -----------------------------------------------------------------------------
// 7. MULTI-STATION AND COMBO ENVELOPE GOVERNANCE
// -----------------------------------------------------------------------------
#[test]
fn test_ec3_multi_station_and_envelope_governance() {
    let sec = make_i_section("W14X90", "A992Fy50", 14.0, 10.0, 0.71, 0.44, 10.0, 0.71);
    let mut model = create_ec3_model("F_ENV", sec);
    let sec_ref = &model.frame_sections["W14X90"];
    let mat = &model.materials["A992Fy50"];

    let v_pl_2 = compute_ec3_major_shear(sec_ref, mat).unwrap();
    let v_pl_3 = compute_ec3_minor_shear(sec_ref, mat).unwrap();

    let mut cr = CaseResults {
        case_name: "MULTI_STATION".into(),
        ..Default::default()
    };

    // Station 0: x=0, high shear major V2 = 0.8 Vpl,2, low moment
    // Station 1: x=30, high shear minor V3 = 0.8 Vpl,3, low moment
    // Station 2: x=60, pure high bending (M3 = 3500, M2 = 800)
    // Station 3: x=90, low demand
    // Station 4: x=120, extreme minor shear V3 = 1.35 Vpl,3 (OVERSTRESSED)
    let stations = vec![
        FrameStationResult {
            station: 0.0,
            rel_dist: 0.0,
            v2: 0.80 * v_pl_2,
            ..Default::default()
        },
        FrameStationResult {
            station: 30.0,
            rel_dist: 0.25,
            v3: 0.80 * v_pl_3,
            ..Default::default()
        },
        FrameStationResult {
            station: 60.0,
            rel_dist: 0.50,
            m3: 2500.0,
            m2: 500.0,
            ..Default::default()
        },
        FrameStationResult {
            station: 90.0,
            rel_dist: 0.75,
            m3: 500.0,
            ..Default::default()
        },
        FrameStationResult {
            station: 120.0,
            rel_dist: 1.0,
            v3: 1.35 * v_pl_3,
            ..Default::default()
        },
    ];

    cr.frame_forces.insert("F_ENV".into(), stations);
    model.case_results.insert("MULTI_STATION".into(), cr);

    try_run_steel_design(&mut model).unwrap();
    let res = &model.steel_design_results["F_ENV"];

    assert_eq!(res.governing_station, 120.0);
    assert_eq!(res.status, "OVERSTRESSED");
    assert_eq!(res.eq_name, "EC3-6.2.6 (V3)");
    assert!((res.total_ratio - 1.35).abs() < 1e-6);

    // Now remove station 4 (the overstressed station), run again, and verify midspan station 2 governs
    let mut cr2 = CaseResults {
        case_name: "NO_OVERSTRESS".into(),
        ..Default::default()
    };
    let stations_safe = vec![
        FrameStationResult {
            station: 0.0,
            rel_dist: 0.0,
            v2: 0.60 * v_pl_2,
            ..Default::default()
        },
        FrameStationResult {
            station: 30.0,
            rel_dist: 0.25,
            v3: 0.60 * v_pl_3,
            ..Default::default()
        },
        FrameStationResult {
            station: 60.0,
            rel_dist: 0.50,
            m3: 2500.0,
            m2: 500.0,
            ..Default::default()
        },
    ];
    cr2.frame_forces.insert("F_ENV".into(), stations_safe);
    model.case_results.clear();
    model.case_results.insert("NO_OVERSTRESS".into(), cr2);

    try_run_steel_design(&mut model).unwrap();
    let res2 = &model.steel_design_results["F_ENV"];
    assert_eq!(res2.governing_station, 60.0);
    assert_eq!(res2.status, "OK");
    assert_eq!(res2.eq_name, "EC3-6.2");
}

// -----------------------------------------------------------------------------
// 8. HIGH SHEAR REDUCTION ACROSS ALL 10 STANDARD SHAPE FAMILIES
// -----------------------------------------------------------------------------
#[test]
fn test_ec3_high_shear_reduction_across_all_10_shapes() {
    let mat_name = "A992Fy50";

    let mut sections: Vec<(&'static str, FrameSection)> = Vec::new();
    sections.push((
        "I",
        make_i_section("SEC_I2", mat_name, 14.0, 10.0, 0.6, 0.4, 10.0, 0.6),
    ));

    let mut sec_chan = make_i_section("SEC_CHANNEL2", mat_name, 12.0, 4.0, 0.5, 0.35, 4.0, 0.5);
    sec_chan.prop_type = FramePropType::Channel;
    sections.push(("Channel", sec_chan));

    let mut sec_dbchan = make_i_section("SEC_DBCHANNEL2", mat_name, 12.0, 4.0, 0.5, 0.35, 4.0, 0.5);
    sec_dbchan.prop_type = FramePropType::DbChannel;
    sec_dbchan.area *= 2.0;
    sec_dbchan.i33 *= 2.0;
    sec_dbchan.i22 *= 2.0;
    sec_dbchan.s33 *= 2.0;
    sec_dbchan.s22 *= 2.0;
    sec_dbchan.z33 *= 2.0;
    sec_dbchan.z22 *= 2.0;
    sections.push(("DbChannel", sec_dbchan));

    let mut sec_t = make_i_section("SEC_T2", mat_name, 7.0, 8.0, 0.5, 0.3, 0.0, 0.0);
    sec_t.prop_type = FramePropType::T;
    sections.push(("T", sec_t));

    let mut sec_angle = make_i_section("SEC_ANGLE2", mat_name, 6.0, 4.0, 0.5, 0.5, 0.0, 0.0);
    sec_angle.prop_type = FramePropType::Angle;
    sections.push(("Angle", sec_angle));

    let mut sec_dbl_angle = make_i_section("SEC_DBLANGLE2", mat_name, 6.0, 4.0, 0.5, 0.5, 0.0, 0.0);
    sec_dbl_angle.prop_type = FramePropType::DblAngle;
    sec_dbl_angle.area *= 2.0;
    sec_dbl_angle.i33 *= 2.0;
    sec_dbl_angle.i22 *= 2.0;
    sec_dbl_angle.s33 *= 2.0;
    sec_dbl_angle.s22 *= 2.0;
    sec_dbl_angle.z33 *= 2.0;
    sec_dbl_angle.z22 *= 2.0;
    sections.push(("DblAngle", sec_dbl_angle));

    sections.push((
        "Box",
        make_box_tube("SEC_BOX2", mat_name, 12.0, 8.0, 0.5, 0.5),
    ));
    sections.push(("Pipe", make_pipe("SEC_PIPE2", mat_name, 10.0, 0.5)));
    sections.push(("Circle", make_circle("SEC_CIRCLE2", mat_name, 8.0)));
    sections.push((
        "Rectangular",
        make_rectangular("SEC_RECT2", mat_name, 10.0, 6.0),
    ));

    for (shape_name, sec) in sections {
        let frame_name = format!("F_HS_{shape_name}");
        let sec_name = sec.name.clone();
        // Use short length 10.0 to prevent LTB from masking cross-section shear reductions
        let mut model = create_ec3_model_with_len(&frame_name, sec, 10.0);
        let sec_ref = &model.frame_sections[&sec_name];
        let mat = &model.materials[mat_name];

        let v_pl_2 = compute_ec3_major_shear(sec_ref, mat).unwrap();
        let v_pl_3 = compute_ec3_minor_shear(sec_ref, mat).unwrap();

        // 1. Low shear baseline: V2 = 0.2 Vpl,2, V3 = 0.2 Vpl,3
        let res_base = run_design_with_forces(
            &mut model,
            &frame_name,
            FrameStationResult {
                station: 5.0,
                rel_dist: 0.5,
                p: -20.0,
                v2: 0.20 * v_pl_2,
                v3: 0.20 * v_pl_3,
                m3: 300.0,
                m2: 150.0,
                ..Default::default()
            },
        )
        .unwrap();

        // 2. High shear major (V2 = 0.8 Vpl,2, V3 = 0.2 Vpl,3) with high moment
        let res_hs_v2 = run_design_with_forces(
            &mut model,
            &frame_name,
            FrameStationResult {
                station: 5.0,
                rel_dist: 0.5,
                p: -20.0,
                v2: 0.80 * v_pl_2,
                v3: 0.20 * v_pl_3,
                m3: 300.0,
                m2: 150.0,
                ..Default::default()
            },
        )
        .unwrap();

        // 3. High shear minor (V2 = 0.2 Vpl,2, V3 = 0.8 Vpl,3)
        let res_hs_v3 = run_design_with_forces(
            &mut model,
            &frame_name,
            FrameStationResult {
                station: 5.0,
                rel_dist: 0.5,
                p: -20.0,
                v2: 0.20 * v_pl_2,
                v3: 0.80 * v_pl_3,
                m3: 300.0,
                m2: 150.0,
                ..Default::default()
            },
        )
        .unwrap();

        // High major shear must reduce major moment capacity
        assert!(
            res_hs_v2.phi_mn33 < res_base.phi_mn33,
            "{shape_name}: Major capacity must be reduced under high V2: hs={}, base={}",
            res_hs_v2.phi_mn33,
            res_base.phi_mn33
        );

        // High minor shear must reduce minor moment capacity
        assert!(
            res_hs_v3.phi_mn22 < res_base.phi_mn22,
            "{shape_name}: Minor capacity must be reduced under high V3: hs={}, base={}",
            res_hs_v3.phi_mn22,
            res_base.phi_mn22
        );
    }
}

// -----------------------------------------------------------------------------
// 9. AISC 360 VS EUROCODE 3 CODE ISOLATION CHECK
// -----------------------------------------------------------------------------
#[test]
fn test_ec3_vs_aisc_code_isolation() {
    let sec = make_i_section("W14X90", "A992Fy50", 14.0, 10.0, 0.71, 0.44, 10.0, 0.71);
    let mut model_ec3 = create_ec3_model("F1", sec.clone());
    model_ec3.steel_design_code = "Eurocode 3-2005".to_string();

    let mut model_aisc = create_ec3_model("F1", sec);
    model_aisc.steel_design_code = "AISC 360-10 LRFD".to_string();

    let st = FrameStationResult {
        station: 60.0,
        rel_dist: 0.5,
        p: -100.0,
        v2: 30.0,
        v3: 20.0,
        m3: 1500.0,
        m2: 400.0,
        ..Default::default()
    };

    let res_ec3 = run_design_with_forces(&mut model_ec3, "F1", st.clone()).unwrap();
    let res_aisc = run_design_with_forces(&mut model_aisc, "F1", st).unwrap();

    assert_eq!(res_ec3.design_code, "Eurocode 3-2005");
    assert_eq!(res_aisc.design_code, "AISC 360-10 LRFD");

    // EC3 equations use EC3 identifiers
    assert!(res_ec3.eq_name.starts_with("EC3"));
    // AISC equations use AISC Chapter H / G identifiers
    assert!(res_aisc.eq_name.starts_with('H') || res_aisc.eq_name.starts_with('G'));
}
