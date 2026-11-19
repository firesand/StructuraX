//! EC3 6.2.8 minor-axis moment under high minor shear for a double channel (ledger D87 with
//! D97). `t2` of a double section is the total width (v19 OAPI help, SetDblChannel: the sum
//! of the two flange widths plus the back-to-back distance), so one flange is `(t2 - dis)/2`
//! wide. Two places use that width here: the shear area Av3 (`shear.rs`, D97) and the flange
//! area in the reduced plastic moment (`steel.rs`, D87). Every expected value is a hand
//! calculation written out below, in N and mm.

use sap_design::try_run_steel_design;
use sap_model::{CaseResults, FramePropType, FrameSection, FrameStationResult, SapModel};
use sap_units::Units;

const FY: f64 = 355.0;

/// Two channels 200 deep back to back with a 10 mm gap: each flange 75 x 12, web 8, so
/// `t2` = 75 + 10 + 75 = 160 and `dis` = 10. The minor axis (2) runs through the gap.
/// Hand properties (flanges from x = 5 to 80, webs from x = 5 to 13, clear web 176):
/// A   = 2 [2 (75)(12) + 176 (8)]                                   = 6416 mm^2
/// I22 = 2 [2 (12 (75)^3/12 + 900 (42.5)^2) + 176 (8)^3/12 + 1408 (9)^2] = 8 433 114.7 mm^4
/// Wel,z = I22/80 = 105 413.9 mm^3
/// Wpl,z = 2 [2 (900)(42.5) + 1408 (9)]                              = 178 344 mm^3
/// I33 = 2 [8 (176)^3/12 + 2 (75 (12)^3/12 + 900 (94)^2)]            = 39 121 834.7 mm^4
/// Wel,y = I33/100 = 391 218.3 mm^3, Wpl,y = 2 [2 (900)(94) + 8 (176)^2/4] = 462 304 mm^3
fn double_channel() -> FrameSection {
    FrameSection {
        name: "2C200".into(),
        material: "A992Fy50".into(),
        prop_type: FramePropType::DbChannel,
        t3: 200.0,
        t2: 160.0,
        dis: 10.0,
        tf: 12.0,
        tw: 8.0,
        area: 6416.0,
        i33: 39_121_834.666_666_67,
        i22: 8_433_114.666_666_667,
        s33: 391_218.346_666_666_7,
        s22: 105_413.933_333_333_33,
        z33: 462_304.0,
        z22: 178_344.0,
        as2: 2816.0,
        as3: 3600.0,
        torsional_j: 2.0 * (2.0 * 75.0 * 12.0_f64.powi(3) + 176.0 * 8.0_f64.powi(3)) / 3.0,
        ..FrameSection::default()
    }
}

/// One 1 m member under Eurocode 3 with one station of minor shear `v3` and minor moment `m2`.
fn design(sec: FrameSection, v3: f64, m2: f64) -> sap_model::SteelDesignResult {
    let mut model = SapModel::new(Units::NMmC);
    model.steel_design_code = "Eurocode 3-2005".into();
    let name = sec.name.clone();
    model.frame_sections.insert(name.clone(), sec);
    {
        let mat = model.materials.get_mut("A992Fy50").unwrap();
        mat.fy = FY;
        mat.e = 210_000.0;
        mat.g = 210_000.0 / 2.6;
    }
    model.add_point("PI", 0.0, 0.0, 0.0);
    model.add_point("PJ", 0.0, 0.0, 1000.0);
    model.add_frame("M1", "PI", "PJ", &name);
    let mut cr = CaseResults {
        case_name: "LC".into(),
        ..Default::default()
    };
    cr.frame_forces.insert(
        "M1".into(),
        vec![FrameStationResult {
            station: 500.0,
            rel_dist: 0.5,
            v3,
            m2,
            ..Default::default()
        }],
    );
    model.case_results.insert("LC".into(), cr);
    try_run_steel_design(&mut model).unwrap();
    model.steel_design_results["M1"].clone()
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}

/// Class (vendor Tables 4-1 and 4-2, eps = sqrt(235/355) = 0.8136): flange (75 - 8)/12 = 5.58
/// <= 9 eps = 7.32 and web 176/8 = 22 <= 72 eps: Class 1, so M_c,z,Rd = Wpl,z fy.
/// Av3 = 4 bf tf = 4 (75)(12) = 3600 mm^2 (four flanges of one channel's width);
/// V_pl,3,Rd = 3600 (355)/sqrt(3) = 737 853.6 N.
/// V3 = 0.8 V_pl,3,Rd: rho = (2 (0.8) - 1)^2 = 0.36.
/// M_z,V,Rd = (Wpl,z - n rho Af^2/(4 tf)) fy with n = 4 flanges and Af = 75 (12) = 900:
/// 4 (0.36)(900)^2/(4 (12)) = 24 300 mm^3; (178 344 - 24 300)(355) = 54 685 620 N mm.
/// M2 = half of that: minor ratio 0.5.
/// Before the two repairs: Av3 = 4 (160)(12) = 7680 mm^2, so the same V3 was 0.375 of the shear
/// resistance and no reduction was made (minor ratio 27 342 810/(178 344 (355)) = 0.4319); with
/// only the shear area repaired, Af = 160 (12) = 1920 gave a reduced moment of
/// (178 344 - 110 592)(355) = 24 051 960 N mm and a ratio of 1.1368.
#[test]
fn double_channel_minor_moment_under_high_minor_shear_uses_one_flange_width() {
    let v_pl3 = 3600.0 * FY / 3.0_f64.sqrt();
    assert!(rel(v_pl3, 737_853.6) < 1e-6);
    let m_red = (178_344.0 - 24_300.0) * FY;
    assert!(rel(m_red, 54_685_620.0) < 1e-12);

    let r = design(double_channel(), 0.8 * v_pl3, 0.5 * m_red);
    assert!(rel(r.phi_vn3, v_pl3) < 1e-9, "V_pl,3 {} hand {v_pl3}", r.phi_vn3);
    assert!(rel(r.shear_minor_ratio, 0.8) < 1e-9, "{}", r.shear_minor_ratio);
    assert!(rel(r.phi_mn22, m_red) < 1e-9, "M_z,V,Rd {} hand {m_red}", r.phi_mn22);
    assert!(rel(r.mr_minor_ratio, 0.5) < 1e-9, "{}", r.mr_minor_ratio);
}

/// Negative: at V3 = 0.5 V_pl,3,Rd there is no reduction (6.2.8(2)), and the minor resistance
/// is the plastic one, Wpl,z fy = 178 344 (355) = 63 312 120 N mm.
#[test]
fn double_channel_at_half_the_shear_resistance_is_not_reduced() {
    let v_pl3 = 3600.0 * FY / 3.0_f64.sqrt();
    let m_pl = 178_344.0 * FY;
    let r = design(double_channel(), 0.5 * v_pl3, 0.5 * m_pl);
    assert!(rel(r.phi_mn22, m_pl) < 1e-9, "M_c,z,Rd {} hand {m_pl}", r.phi_mn22);
    assert!(rel(r.mr_minor_ratio, 0.5) < 1e-9, "{}", r.mr_minor_ratio);
}
