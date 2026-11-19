//! Batch C repairs (concrete design and section libraries), ledger D94-D104.
//!
//! Every expected value below is a hand calculation written out in the test, with the
//! manual page it comes from. None of them is read from a function of the crate.
use sap_design::try_run_concrete_design;
use sap_model::{CaseResults, FrameStationResult, Material, SapModel};
use sap_section::make_rectangular;
use sap_units::Units;

fn close(a: f64, b: f64, rel: f64, what: &str) {
    assert!(
        (a - b).abs() <= rel * b.abs().max(1e-300),
        "{what}: got {a}, expected {b} (relative tolerance {rel})"
    );
}

/// A vertical `t3` x `t2` concrete column (kip, in; f'c = 4 ksi, fy = fyt = 60 ksi, default
/// cover 2.5 in) of the clear length `len`, loaded with one station per entry of `stations`.
fn column_model(
    t3: f64,
    t2: f64,
    len: f64,
    code: &str,
    stations: Vec<FrameStationResult>,
) -> SapModel {
    let mut m = SapModel::new(Units::KipInF);
    m.concrete_design_code = code.into();
    let mut concrete = Material::default_concrete_kip_in();
    concrete.fy = 60.0;
    concrete.fys = 60.0;
    m.materials.insert(concrete.name.clone(), concrete);
    let section = make_rectangular("COL", "4000Psi", t3, t2);
    m.frame_sections.insert("COL".into(), section);
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 0.0, 0.0, len);
    m.add_frame("C1", "1", "2", "COL");
    let mut cr = CaseResults {
        case_name: "COMBO1".into(),
        ..Default::default()
    };
    cr.frame_forces.insert("C1".into(), stations);
    m.case_results.insert("COMBO1".into(), cr);
    m
}

fn station(p: f64, v2: f64, m2: f64, m3: f64) -> FrameStationResult {
    FrameStationResult {
        station: 0.0,
        rel_dist: 0.0,
        p,
        v2,
        m2,
        m3,
        ..Default::default()
    }
}

// ---------------------------------------------------------------- D94

/// ACI 318-14 22.5.6.1 / 22.5.7.1 (CSI manual p. 3-25), 24 x 24 in column, d = 24 - 2.5 =
/// 21.5 in, f'c = 4000 psi, fyt = 60 ksi, Vu = 60 kip, Ag = 576 in2, Acv = 24 x 21.5 = 516 in2,
/// sqrt(f'c) = 63.2456 psi.
fn column_av_s(p_sap: f64) -> f64 {
    let mut m = column_model(24.0, 24.0, 120.0, "ACI 318-14", vec![station(p_sap, 60.0, 0.0, 0.0)]);
    try_run_concrete_design(&mut m).unwrap();
    m.concrete_design_results["C1"].av_over_s
}

#[test]
fn d94_aci_column_shear_tension_reduces_vc() {
    // P = +100 kip is tension in SAP2000 sign: Pu = -100 kip.
    // Pu/Ag = -100000/576 = -173.611 psi; 1 + (-173.611)/500 = 0.652778
    // Vc = 2 x 63.2456 x 0.652778 x 516 / 1000 = 42.6064 kip; phi Vc = 31.9548 kip
    // Av/s = (60 - 31.9548) / (0.75 x 60 x 21.5) = 0.028987 in2/in
    close(column_av_s(100.0), 0.028987, 5e-5, "tension Av/s");
    let vc: f64 = 2.0 * 4000.0_f64.sqrt() * (1.0 - 100000.0 / 576.0 / 500.0) * 516.0 / 1000.0;
    close(vc, 42.6064, 1e-6, "hand Vc");
    close(
        (60.0 - 0.75 * vc) / (0.75 * 60.0 * 21.5),
        0.028987,
        1e-5,
        "hand Av/s",
    );
}

#[test]
fn d94_aci_column_shear_tension_value_to_1e6() {
    let vc = 2.0 * 4000.0_f64.sqrt() * (1.0 - 100000.0 / 576.0 / 500.0) * 516.0 / 1000.0;
    let expected = (60.0 - 0.75 * vc) / (0.75 * 60.0 * 21.5);
    close(column_av_s(100.0), expected, 1e-6, "tension Av/s (1e-6)");
}

#[test]
fn d94_aci_column_shear_zero_axial_is_the_old_beam_value() {
    // Pu = 0: Vc = 2 x 63.2456 x 516 / 1000 = 65.2694 kip, phi Vc = 48.952 kip < Vu = 60, so
    // the strength term is (60 - 48.952)/(0.75 x 60 x 21.5) = 0.011419 in2/in (the brief's
    // number), but Av,min/s = max(0.75 sqrt(f'c) = 47.43, 50)/fyt x bw = 50/60000 x 24 =
    // 0.02 in2/in (Table 9.6.3.3) is larger and governs: the reported value is 0.02, as before.
    close(column_av_s(0.0), 0.02, 1e-9, "Pu = 0 Av/s");
}

#[test]
fn d94_aci_column_shear_compression_branch() {
    // Pu = +300 kip (P = -300): Pu/Ag = 520.833 psi; 1 + 520.833/2000 = 1.260417
    // Vc = 126.491 x 1.260417 x 516 / 1000 = 82.2667 kip (cap 3.5 x 63.2456 x sqrt(2.04167) x
    // 516 = 163.2 kip not active); phi Vc = 61.70 kip > Vu = 60: the minimum 0.02 in2/in.
    close(column_av_s(-300.0), 0.02, 1e-9, "compression Av/s");
    // Vu = 100 kip > phi Vc: Av/s = (100 - 61.70)/(0.75 x 60 x 21.5)
    let vc = 2.0 * 4000.0_f64.sqrt() * (1.0 + 300000.0 / 576.0 / 2000.0) * 516.0 / 1000.0;
    close(vc, 82.2667, 1e-5, "hand Vc compression");
    let mut m = column_model(
        24.0,
        24.0,
        120.0,
        "ACI 318-14",
        vec![station(-300.0, 100.0, 0.0, 0.0)],
    );
    try_run_concrete_design(&mut m).unwrap();
    close(
        m.concrete_design_results["C1"].av_over_s,
        (100.0 - 0.75 * vc) / (0.75 * 60.0 * 21.5),
        1e-6,
        "compression Av/s at Vu 100",
    );
}

#[test]
fn d94_aci_beam_shear_unchanged() {
    // Beam rule has no axial term: 14 x 24 beam, Vu = 65 kip, d = 21.5:
    // phi Vc = 0.75 x 2 x 63.2456 x 14 x 21.5 / 1000 = 28.5553 kip,
    // Av/s = (65 - 28.5553)/(0.75 x 60 x 21.5) = 0.037667 in2/in.
    let (avs, over) =
        sap_design::design_aci_beam_shear(65.0, 14.0, 21.5, 4.0, 60.0, Units::KipInF);
    close(avs, 0.037667, 1e-4, "beam Av/s");
    assert!(!over);
}

// ---------------------------------------------------------------- D95

/// Eurocode 2 model in N, mm: fck = 30 MPa, longitudinal rebar fyk = 500 MPa, stirrups `fywk`,
/// cover 40 mm, one concrete member (`column` selects a vertical column or a horizontal beam)
/// of `t3` x `t2` mm with the given stations.
fn ec2_model(
    t3: f64,
    t2: f64,
    column: bool,
    fywk: f64,
    cover: f64,
    stations: Vec<FrameStationResult>,
) -> SapModel {
    use sap_model::{ConcreteRebarConfig, MatType};
    let mut m = SapModel::new(Units::NMmC);
    m.concrete_design_code = "Eurocode 2-2004".into();
    let mut concrete = Material::default_concrete_kip_in();
    concrete.name = "C30".into();
    concrete.fc = 30.0;
    concrete.fy = 500.0;
    concrete.fys = 500.0;
    concrete.e = 33_000.0;
    m.materials.insert("C30".into(), concrete);
    let mut long = Material::default_steel_kip_in();
    long.name = "RL".into();
    long.mat_type = MatType::Rebar;
    long.fy = 500.0;
    long.fys = 500.0;
    long.e = 200_000.0;
    m.materials.insert("RL".into(), long);
    let mut conf = Material::default_steel_kip_in();
    conf.name = "RC".into();
    conf.mat_type = MatType::Rebar;
    conf.fy = fywk;
    conf.fys = fywk;
    conf.e = 200_000.0;
    m.materials.insert("RC".into(), conf);
    let mut section = make_rectangular("SEC", "C30", t3, t2);
    section.concrete_rebar = Some(ConcreteRebarConfig {
        is_column: column,
        rebar_mat_long: "RL".into(),
        rebar_mat_confine: "RC".into(),
        cover_top: cover,
        cover_bot: cover,
        num_bars_3dir: 3,
        num_bars_2dir: 3,
        total_rebar_area: 0.02 * t3 * t2,
    });
    m.frame_sections.insert("SEC".into(), section);
    m.add_point("1", 0.0, 0.0, 0.0);
    if column {
        m.add_point("2", 0.0, 0.0, 3000.0);
    } else {
        m.add_point("2", 6000.0, 0.0, 0.0);
    }
    m.add_frame("M1", "1", "2", "SEC");
    m.frames.get_mut("M1").unwrap().concrete_framing = sap_model::ConcreteFramingType::SwayOrdinary;
    let mut cr = CaseResults {
        case_name: "COMBO1".into(),
        ..Default::default()
    };
    cr.frame_forces.insert("M1".into(), stations);
    m.case_results.insert("COMBO1".into(), cr);
    m
}

fn ec2_column(v2: f64) -> (f64, String) {
    let mut m = ec2_model(400.0, 400.0, true, 500.0, 40.0, vec![station(0.0, v2, 0.0, 0.0)]);
    try_run_concrete_design(&mut m).unwrap();
    let r = &m.concrete_design_results["M1"];
    (r.av_over_s, r.status.clone())
}

#[test]
fn d95_ec2_column_shear_uses_45_degrees() {
    // 400 x 400, cover 40: d = 360, z = 0.9 d = 324 mm, fywd = 500/1.15 = 434.783 MPa,
    // VEd = 200 kN, cot(theta) = 1 (manual p. 3-21):
    // Asw/s = 200000 / (324 x 434.783 x 1) = 1.41975 mm2/mm  (cot 2.5 gave 0.56790)
    let (avs, status) = ec2_column(200_000.0);
    close(avs, 200_000.0 / (324.0 * (500.0 / 1.15)), 1e-6, "Asw/s at 45 degrees");
    close(avs, 1.41975, 1e-5, "Asw/s hand value");
    assert_eq!(status, "OK");
}

#[test]
fn d95_ec2_column_vrd_max_at_45_degrees() {
    // V_Rd,max(45) = bw z nu1 fcd / (1 + 1) with nu1 = 0.6 (1 - 30/250) = 0.528,
    // fcd = 0.85 x 30 / 1.5 = 17 MPa: 400 x 324 x 0.528 x 17 / 2 = 581 645 N.
    let vmax: f64 = 400.0 * 324.0 * 0.528 * 17.0 / 2.0;
    close(vmax, 581_645.0, 1e-5, "hand V_Rd,max(45)");
    assert_eq!(ec2_column(600_000.0).1, "OVERSTRESSED");
    // 450 kN: before the repair V_Rd,max(cot 2.5) = 401 134 N flagged it
    assert_eq!(ec2_column(450_000.0).1, "OK");
}

// ---------------------------------------------------------------- D96

/// Column of `t3` x `t2` in, clear length `len`, E = 3605 ksi, f'c = 4 ksi (the values of the
/// hand calculation), with one station.
fn slender_column(t3: f64, t2: f64, len: f64, st: FrameStationResult) -> SapModel {
    let mut m = column_model(t3, t2, len, "ACI 318-14", vec![st]);
    let c = m.materials.get_mut("4000Psi").unwrap();
    c.e = 3605.0;
    c.fc = 4.0;
    m
}

#[test]
fn d96_pc_and_delta_per_axis_from_the_hand_numbers() {
    // t3 = 24, t2 = 12, E = 3605 ksi, l = 180 in, EI_eff = 0.4 E I / 1.4:
    // I33 = 12 x 24^3 / 12 = 13824 in4, I22 = 24 x 12^3 / 12 = 3456 in4
    // Pc33 = pi^2 x 0.4 x 3605 x 13824 / 1.4 / 180^2 = 4337.362 kip
    // Pc22 = pi^2 x 0.4 x 3605 x 3456 / 1.4 / 180^2 = 1084.341 kip
    // Pu = 488: delta33 = 1/(1 - 488/(0.75 x 4337.362)) = 1.17649
    //           delta22 = 1/(1 - 488/(0.75 x 1084.341)) = 2.50036
    let pc33 = sap_design::concrete::aci_column_pc(3605.0, 13824.0, 180.0);
    let pc22 = sap_design::concrete::aci_column_pc(3605.0, 3456.0, 180.0);
    close(pc33, 4337.362, 1e-6, "Pc33");
    close(pc22, 1084.341, 1e-6, "Pc22");
    close(sap_design::concrete::aci_moment_magnifier(488.0, pc33), 1.17649, 1e-5, "delta33");
    close(sap_design::concrete::aci_moment_magnifier(488.0, pc22), 2.50036, 1e-5, "delta22");
}

fn expected_ratio(sec_t3: f64, sec_t2: f64, pu: f64, m3: f64, m2: f64) -> f64 {
    // The magnified moments are hand numbers passed in; only the P-M curves and the contour
    // come from the crate (they are not the subject of D96).
    let sec = make_rectangular("COL", "4000Psi", sec_t3, sec_t2);
    let units = Units::KipInF;
    let (cmaj, _, pn_max, _) = sap_design::generate_column_pm_curve(
        &sec, 4.0, 60.0, 29_000.0, units, sap_design::ConcreteCode::Aci318, true,
    );
    let (cmin, _, _, _) = sap_design::generate_column_pm_curve(
        &sec, 4.0, 60.0, 29_000.0, units, sap_design::ConcreteCode::Aci318, false,
    );
    let pnt = cmaj.last().unwrap().phi_pn;
    sap_design::evaluate_column_biaxial_pmm_ratio(
        pu, m3, m2, &cmaj, &cmin, pn_max, pnt, sec_t3, sec_t2,
    )
}

#[test]
fn d96_minor_axis_moment_is_magnified_by_its_own_factor() {
    // Pu = 488 kip compression, M3 = 2000, M2 = 800 kip-in, l = 180 in, 24 x 12 column:
    // kl/r22 = 180 / 3.4641 = 51.96 > 34. M3 x 1.17649 = 2352.98, M2 x 2.50036 = 2000.29.
    let st = station(-488.0, 0.0, 800.0, 2000.0);
    let mut m = slender_column(24.0, 12.0, 180.0, st);
    try_run_concrete_design(&mut m).unwrap();
    let r = &m.concrete_design_results["C1"];
    assert_ne!(r.status, "BUCKLING_OVERSTRESSED");
    let expected = expected_ratio(24.0, 12.0, 488.0, 2000.0 * 1.17649, 800.0 * 2.50036);
    close(r.pmm_ratio, expected, 1e-4, "ratio with per-axis delta");
    // the old rule magnified both by 1.17649
    let old = expected_ratio(24.0, 12.0, 488.0, 2000.0 * 1.17649, 800.0 * 1.17649);
    assert!(r.pmm_ratio > old * 1.01, "minor-axis moment must grow beyond the old factor");
}

#[test]
fn d96_minor_axis_buckling_is_flagged() {
    // Pu = 1000 kip >= 0.75 Pc22 = 813.26 kip (0.75 Pc33 = 3253.0 kip): failure condition
    let st = station(-1000.0, 0.0, 100.0, 100.0);
    let mut m = slender_column(24.0, 12.0, 180.0, st);
    try_run_concrete_design(&mut m).unwrap();
    assert_eq!(m.concrete_design_results["C1"].status, "BUCKLING_OVERSTRESSED");
    // 700 kip < 0.75 Pc22: no failure condition
    let st = station(-700.0, 0.0, 100.0, 100.0);
    let mut m = slender_column(24.0, 12.0, 180.0, st);
    try_run_concrete_design(&mut m).unwrap();
    assert_ne!(m.concrete_design_results["C1"].status, "BUCKLING_OVERSTRESSED");
}

#[test]
fn d96_square_column_keeps_the_single_factor() {
    // 24 x 24, l = 300 in: I = 27648 in4, Pc = pi^2 x 0.4 x 3605 x 27648 / 1.4 / 300^2 =
    // 3122.90 kip, kl/r = 300 / 6.9282 = 43.3 > 34; Pu = 488: delta = 1/(1 - 488/(0.75 x
    // 3122.90)) = 1.26319 on both moments.
    let st = station(-488.0, 0.0, 1500.0, 1500.0);
    let mut m = slender_column(24.0, 24.0, 300.0, st);
    try_run_concrete_design(&mut m).unwrap();
    let expected = expected_ratio(24.0, 24.0, 488.0, 1500.0 * 1.26319, 1500.0 * 1.26319);
    close(m.concrete_design_results["C1"].pmm_ratio, expected, 1e-4, "square ratio");
}

// ---------------------------------------------------------------- D97

fn steel_section(
    kind: sap_model::FramePropType,
    t3: f64,
    t2: f64,
    dis: f64,
    t: f64,
) -> sap_model::FrameSection {
    sap_model::FrameSection {
        name: "TEST".into(),
        material: "A992Fy50".into(),
        prop_type: kind,
        t3,
        t2,
        dis,
        tf: t,
        tw: t,
        ..Default::default()
    }
}

#[test]
fn d97_double_angle_av3_uses_one_leg_width() {
    // Made-up dimensions (kip, in; Fy = 50 ksi, E = 29000 ksi). t2 is the total width
    // (v19 OAPI SetDblAngle): t2 = 10.5 = 2 x 5 + dis 0.5, so one leg is (10.5 - 0.5)/2 = 5 in.
    // Av3 = 2 x 5 x 0.5 = 5.0 in2; b/t = 10 < 1.10 sqrt(1.2 x 29000/50) = 29.0, Cv = 1.
    // AISC phi Vn3 = 0.9 x 0.6 x 50 x 5.0 = 135.0 kip (the old code gave 283.5)
    // EC3 V_pl,3 = 5.0 x 50 / sqrt(3) = 144.338 kip (the old code gave 303.109)
    let model = SapModel::new(Units::KipInF);
    let mat = &model.materials["A992Fy50"];
    let sec = steel_section(sap_model::FramePropType::DblAngle, 5.0, 10.5, 0.5, 0.5);
    close(
        sap_design::compute_aisc_minor_shear(&sec, mat, false).unwrap(),
        135.0,
        1e-9,
        "AISC double angle phi Vn3",
    );
    close(
        sap_design::compute_ec3_minor_shear(&sec, mat).unwrap(),
        5.0 * 50.0 / 3.0_f64.sqrt(),
        1e-9,
        "EC3 double angle Vpl3",
    );
    close(5.0 * 50.0 / 3.0_f64.sqrt(), 144.338, 1e-5, "hand EC3 value");
}

#[test]
fn d97_double_channel_av3_uses_one_flange_width() {
    // t2 = 8.5 = 2 x 4 + dis 0.5: bf = 4 in, tf = 0.5, Av3 = 4 x 4 x 0.5 = 8.0 in2;
    // b/t = 8 gives Cv = 1: AISC phi Vn3 = 0.9 x 0.6 x 50 x 8.0 = 216.0 kip (old 459.0),
    // EC3 8.0 x 50 / sqrt(3) = 230.940 kip (old 490.748).
    let model = SapModel::new(Units::KipInF);
    let mat = &model.materials["A992Fy50"];
    let sec = steel_section(sap_model::FramePropType::DbChannel, 10.0, 8.5, 0.5, 0.5);
    close(
        sap_design::compute_aisc_minor_shear(&sec, mat, false).unwrap(),
        216.0,
        1e-9,
        "AISC double channel phi Vn3",
    );
    close(
        sap_design::compute_ec3_minor_shear(&sec, mat).unwrap(),
        8.0 * 50.0 / 3.0_f64.sqrt(),
        1e-9,
        "EC3 double channel Vpl3",
    );
    close(8.0 * 50.0 / 3.0_f64.sqrt(), 230.940, 1e-5, "hand EC3 value");
}

#[test]
fn d97_single_members_unchanged_and_impossible_double_sections_refused() {
    let model = SapModel::new(Units::KipInF);
    let mat = &model.materials["A992Fy50"];
    // Angle t2 = 4, t = 0.5: Av3 = 2.0 in2, phi Vn = 0.9 x 0.6 x 50 x 2.0 = 54.0 kip
    let angle = steel_section(sap_model::FramePropType::Angle, 6.0, 4.0, 0.0, 0.5);
    close(sap_design::compute_aisc_minor_shear(&angle, mat, false).unwrap(), 54.0, 1e-9, "angle");
    // Channel flange 10, tf 0.5: Av3 = 2 x 10 x 0.5 = 10 in2 -> 0.9 x 0.6 x 50 x 10 = 270 kip
    let chan = steel_section(sap_model::FramePropType::Channel, 12.0, 10.0, 0.0, 0.5);
    close(sap_design::compute_aisc_minor_shear(&chan, mat, false).unwrap(), 270.0, 1e-9, "channel");
    // t2 not larger than dis: no member width, refused (not silently zero)
    let bad = steel_section(sap_model::FramePropType::DblAngle, 5.0, 0.5, 0.5, 0.5);
    assert!(sap_design::compute_aisc_minor_shear(&bad, mat, false).is_err());
    assert!(sap_design::compute_ec3_minor_shear(&bad, mat).is_err());
}

/// Optional vendor fixture (skips when `extracted` is absent): the double sections of the v7
/// library, whose own shear area `as3` is the reference read at run time (1 % tolerance). The
/// library values are not written here.
#[test]
fn d97_vendor_library_double_sections_match_their_own_as3() {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../extracted/files/aisc13.pro");
    if !path.exists() {
        return;
    }
    let db = sap_section::ProDatabase::from_file(&path).unwrap();
    let model = SapModel::new(Units::KipInF);
    let mat = &model.materials["A992Fy50"];
    for (name, kind) in [
        ("2L8X8X1-1/8X3/8", sap_model::FramePropType::DblAngle),
        ("2C15X50X3/8", sap_model::FramePropType::DbChannel),
    ] {
        let sec = db
            .sections
            .iter()
            .find(|s| s.name == name)
            .unwrap_or_else(|| panic!("{name} not in aisc13.pro"));
        assert_eq!(sec.prop_type, kind, "{name}: type (D98)");
        // EC3: V = Av3 fy / sqrt(3), so Av3 = V sqrt(3) / fy
        let v = sap_design::compute_ec3_minor_shear(sec, mat).unwrap();
        let av3 = v * 3.0_f64.sqrt() / mat.fy;
        close(av3, sec.as3, 0.01, name);
    }
}

// ---------------------------------------------------------------- D99

/// 14 x 24 in beam (kip, in; f'c = 4 ksi, fy = fyt = 60 ksi, Es = 29000 ksi) with the given
/// covers and stations, designed to ACI 318-14.
fn aci_beam(cover_top: f64, cover_bot: f64, stations: Vec<FrameStationResult>) -> SapModel {
    use sap_model::ConcreteRebarConfig;
    let mut m = column_model(24.0, 14.0, 240.0, "ACI 318-14", vec![]);
    m.frame_sections.clear();
    m.frames.clear();
    m.points.clear();
    let mut section = make_rectangular("BEAM", "4000Psi", 24.0, 14.0);
    section.concrete_rebar = Some(ConcreteRebarConfig {
        is_column: false,
        rebar_mat_long: "4000Psi".into(),
        rebar_mat_confine: "4000Psi".into(),
        cover_top,
        cover_bot,
        num_bars_3dir: 3,
        num_bars_2dir: 3,
        total_rebar_area: 0.0,
    });
    m.frame_sections.insert("BEAM".into(), section);
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 240.0, 0.0, 0.0);
    m.add_frame("B1", "1", "2", "BEAM");
    // the default framing (sway special) raises the opposite-sign strengths; ordinary has no floor
    m.frames.get_mut("B1").unwrap().concrete_framing = sap_model::ConcreteFramingType::SwayOrdinary;
    let mut cr = CaseResults {
        case_name: "COMBO1".into(),
        ..Default::default()
    };
    cr.frame_forces.insert("B1".into(), stations);
    m.case_results.clear();
    m.case_results.insert("COMBO1".into(), cr);
    m
}

/// Hand calculation of ACI 318-14 manual pp. 3-31/3-32 for the 14 x 24 beam, d = 21.5 in.
/// Returns (Mu = 1.3 phi Mn,max, As1 + As2, A's) for the compression cover `dp`.
fn aci_doubly_hand(dp: f64) -> (f64, f64, f64) {
    let (b, d, fc, fy, es, phi) = (14.0, 21.5, 4.0, 60.0, 29000.0, 0.9);
    let beta1 = 0.85;
    let c_max = 0.003 / (0.003 + 0.005) * d; // 8.0625 in
    let a_max = beta1 * c_max; // 6.853125 in
    let c = 0.85 * fc * b * a_max;
    let muc = phi * c * (d - a_max / 2.0); // 5306.142 kip-in
    let mu = 1.3 * muc;
    let mus = mu - muc;
    let fs = (es * 0.003 * (c_max - dp) / c_max).min(fy);
    let as_comp = mus / ((fs - 0.85 * fc) * (d - dp) * phi);
    let as1 = muc / (fy * (d - a_max / 2.0) * phi);
    let as2 = mus / (fy * (d - dp) * phi);
    (mu, as1 + as2, as_comp)
}

#[test]
fn d99_aci_doubly_reinforced_beam_reports_compression_steel() {
    // Hand numbers of the brief: phi Mn,max = 5306.142 kip-in, Mu = 6897.985 kip-in,
    // As = 5.43681 + 1.55150 = 6.98832 in2, A's = 1.64470 in2 (f's = 60 ksi, yields).
    let (mu, as_t, as_c) = aci_doubly_hand(2.5);
    close(mu, 6897.985, 1e-6, "hand Mu");
    close(as_t, 6.98832, 1e-6, "hand As");
    close(as_c, 1.64470, 1e-5, "hand A's");
    // Positive M3: As at the bottom, A's at the top.
    let mut m = aci_beam(2.5, 2.5, vec![station_m3(mu)]);
    try_run_concrete_design(&mut m).unwrap();
    let r = &m.concrete_design_results["B1"];
    close(r.as_bot, as_t, 1e-9, "as_bot");
    close(r.as_top, as_c, 1e-9, "as_top = A's");
    assert_eq!(r.status, "OK");
    // Negative M3 of the same size: As at the top, A's at the bottom.
    let mut m = aci_beam(2.5, 2.5, vec![station_m3(-mu)]);
    try_run_concrete_design(&mut m).unwrap();
    let r = &m.concrete_design_results["B1"];
    close(r.as_top, as_t, 1e-9, "as_top (negative M)");
    close(r.as_bot, as_c, 1e-9, "as_bot = A's (negative M)");
}

#[test]
fn d99_aci_compression_steel_below_yield() {
    // d' = 4.5 in: f's = 29000 x 0.003 x (8.0625 - 4.5)/8.0625 = 38.4419 ksi < fy,
    // A's = 2.96908 in2, As = 5.43681 + Mus/(60 x 17 x 0.9) = 7.17085 in2.
    let (mu, as_t, as_c) = aci_doubly_hand(4.5);
    close(as_c, 2.96908, 1e-5, "hand A's d' = 4.5");
    close(as_t, 7.17085, 1e-5, "hand As d' = 4.5");
    let mut m = aci_beam(4.5, 2.5, vec![station_m3(mu)]);
    try_run_concrete_design(&mut m).unwrap();
    let r = &m.concrete_design_results["B1"];
    close(r.as_top, as_c, 1e-9, "as_top = A's");
    close(r.as_bot, as_t, 1e-9, "as_bot");
}

#[test]
fn d99_singly_reinforced_beam_keeps_no_compression_steel() {
    // Mu = 0.9 phi Mn,max (inside the singly reinforced range): as_top stays 0.
    let (mu, _, _) = aci_doubly_hand(2.5);
    let mu_single = mu * 0.9 / 1.3;
    let mut m = aci_beam(2.5, 2.5, vec![station_m3(mu_single)]);
    try_run_concrete_design(&mut m).unwrap();
    let r = &m.concrete_design_results["B1"];
    assert_eq!(r.as_top, 0.0);
    assert!(r.as_bot > 0.0);
}

fn station_m3(m3: f64) -> FrameStationResult {
    station(0.0, 0.0, 0.0, m3)
}

#[test]
fn d99_ec2_doubly_reinforced_beam_reports_compression_steel() {
    // 300 x 600, cover 60 (d = 540, d' = 60), fck = 30, fyk = 500 (fyd = 434.783), eta fcd = 17:
    // M = 1.3 x 0.167 x b d^2 fck = 569.757 kNm; (x/d)lim = (1 - 0.44)/(1.25 x 1.0) = 0.448,
    // m_lim = 0.8 x 0.448 (1 - 0.4 x 0.448) = 0.294175, m = M/(b d^2 17) = 0.383118,
    // omega' = (m - m_lim)/(1 - 60/540) = 0.100061, omega = 0.3584 + 0.100061 = 0.458461,
    // x_lim = 241.92 mm, f's = 200000 x 0.0035 x (1 - 60/241.92) = 526.4 > fyd -> 434.783,
    // A's = 0.100061 x 17 x 300 x 540/434.783 = 633.805 mm2, As = 0.458461 x 17 x 300 x 540/434.783
    // = 2903.982 mm2. (The old code gave As = 2905.070 and no A's.)
    let (b, d, dp, fck, fyd) = (300.0_f64, 540.0_f64, 60.0_f64, 30.0_f64, 500.0 / 1.15);
    let mu = 1.3 * 0.167 * b * d * d * fck;
    close(mu, 569.757e6, 1e-6, "hand M");
    let x_lim_d = (1.0 - 0.44) / (1.25 * 1.0);
    let m_lim = 0.8 * x_lim_d * (1.0 - 0.4 * x_lim_d);
    let m = mu / (b * d * d * 17.0);
    let omega_p = (m - m_lim) / (1.0 - dp / d);
    let as_c = omega_p * 17.0 * b * d / fyd;
    let as_t = (0.8 * x_lim_d + omega_p) * 17.0 * b * d / fyd;
    close(m_lim, 0.294175, 1e-5, "m_lim");
    close(as_c, 633.805, 1e-6, "hand A's");
    close(as_t, 2903.982, 1e-6, "hand As");
    // through the design entry: positive M3, in N mm
    let mut model = ec2_model(600.0, 300.0, false, 500.0, 60.0, vec![station_m3(mu)]);
    try_run_concrete_design(&mut model).unwrap();
    let r = &model.concrete_design_results["M1"];
    close(r.as_bot, as_t, 1e-9, "EC2 as_bot");
    close(r.as_top, as_c, 1e-9, "EC2 as_top = A's");
    assert_eq!(r.status, "OK");
}

#[test]
fn d99_ec2_singly_reinforced_vendor_example_unchanged() {
    // en_2_2004_ex001: b = 230, d = 490, fck = 30, fyk = 460, M = 165 kNm, eta fcd = 17:
    // m = 165e6/(230 x 490^2 x 17) = 0.17576, omega = 1 - sqrt(1 - 2m) = 0.19471,
    // As = omega x 17 x 230 x 490/400 = 932.64 mm2 (the vendor prints 933 for alpha_cc = 0.85).
    let (b, d, fyd) = (230.0_f64, 490.0_f64, 460.0 / 1.15);
    let m = 165e6 / (b * d * d * 17.0);
    let omega = 1.0 - (1.0 - 2.0 * m).sqrt();
    let as_hand = omega * 17.0 * b * d / fyd;
    close(m, 0.17576, 1e-4, "hand m");
    close(as_hand, 932.64, 1e-4, "hand As");
    let (as_t, _, _, over) =
        sap_design::design_ec2_beam(165e6, 0.0, b, d, 60.0, 30.0, 460.0, 460.0, Units::NMmC);
    assert!(!over);
    // the singly reinforced path is the code's own z-form (unchanged): within 0.1 % of the omega form
    close(as_t, as_hand, 1e-3, "singly reinforced As");
    let r = sap_design::concrete::design_ec2_beam_doubly(
        165e6, 0.0, b, d, 60.0, 30.0, 460.0, 460.0, Units::NMmC,
    );
    assert_eq!(r.as_comp, 0.0);
}

// ---------------------------------------------------------------- D100 / D101

/// EC2 beam in N, mm: 300 wide, `t3` deep, cover 60 (d = t3 - 60), station with M3 = `m3`,
/// V2 = `v2`; `fywk` is the stirrup yield stress. `seismic` selects what the design combination
/// contains: 0 = a dead pattern only, 1 = a Quake pattern, 2 = a response-spectrum case,
/// 3 = a combination that nests a combination with a Quake pattern, 4 = a time-history case.
fn ec2_beam_with(t3: f64, m3: f64, v2: f64, fywk: f64, seismic: u8) -> (f64, String) {
    use sap_model::{ComboType, LoadCase, LoadCaseType, LoadCombo, LoadPattern, LoadPatternType};
    let mut m = ec2_model(t3, 300.0, false, fywk, 60.0, vec![station(0.0, v2, 0.0, m3)]);
    let pat = |name: &str, t| LoadPattern {
        name: name.into(),
        pat_type: t,
        self_wt_multiplier: 0.0,
    };
    m.load_patterns.insert("DEAD".into(), pat("DEAD", LoadPatternType::Dead));
    m.load_patterns.insert("QK".into(), pat("QK", LoadPatternType::Quake));
    let mut case = LoadCase::linear_static(
        "CASE1",
        if seismic == 1 || seismic == 3 { "QK" } else { "DEAD" },
        1.0,
    );
    if seismic == 2 {
        case.case_type = LoadCaseType::ResponseSpectrum;
    }
    if seismic == 4 {
        case.case_type = LoadCaseType::LinearHistory;
        case.loads.clear();
    }
    m.load_cases.insert("CASE1".into(), case);
    let members = if seismic == 3 {
        m.load_combos.insert(
            "SUB".into(),
            LoadCombo {
                name: "SUB".into(),
                combo_type: ComboType::LinearAdd,
                cases: vec![("CASE1".into(), 1.0)],
            },
        );
        vec![("SUB".to_string(), 1.0)]
    } else {
        vec![("CASE1".to_string(), 1.0)]
    };
    m.load_combos.insert(
        "COMBO1".into(),
        LoadCombo {
            name: "COMBO1".into(),
            combo_type: ComboType::LinearAdd,
            cases: members,
        },
    );
    let mut res = CaseResults {
        case_name: "CASE1".into(),
        ..Default::default()
    };
    res.frame_forces.insert("M1".into(), vec![station(0.0, 0.0, 0.0, 0.0)]);
    m.case_results.insert("CASE1".into(), res);
    try_run_concrete_design(&mut m).unwrap();
    let r = &m.concrete_design_results["M1"];
    (r.av_over_s, r.status.clone())
}

/// Hand numbers for the 300 x 540 beam (z = 486, fck 30, fcd = 17, nu1 = 0.528, fywd = 434.783):
/// v_Ed = V/(300 x 540), X = 0.9 nu1 fcd / v_Ed, cot = (X + sqrt(X^2 - 4))/2 in [1, 2.5].
#[test]
fn d100_strut_cot_from_the_hand_numbers() {
    let hand = |v: f64| {
        let x = 0.9 * 0.528 * 17.0 / (v / (300.0 * 540.0));
        if x < 2.0 {
            1.0
        } else {
            ((x + (x * x - 4.0).sqrt()) / 2.0).clamp(1.0, 2.5)
        }
    };
    let cot = |v: f64, seismic| {
        sap_design::concrete::ec2_beam_strut_cot(v, 300.0, 540.0, 30.0, Units::NMmC, seismic)
    };
    close(cot(500e3, false), 2.15292, 2e-5, "cot at 500 kN");
    close(cot(500e3, false), hand(500e3), 1e-9, "cot at 500 kN (formula)");
    close(cot(600e3, false), 1.52575, 2e-5, "cot at 600 kN");
    assert_eq!(cot(200e3, false), 2.5);
    assert_eq!(cot(700e3, false), 1.0);
    assert_eq!(cot(200e3, true), 1.0);
    // V_Rd,max(45) = 300 x 486 x 0.528 x 17/2 = 654.350 kN
    close(300.0 * 486.0 * 0.528 * 17.0 / 2.0, 654_350.0, 1e-5, "hand V_Rd,max(45)");
}

#[test]
fn d100_ec2_beam_shear_with_optimised_strut_angle() {
    // V = 500 kN: cot = 2.15292 (theta 24.914 deg), Asw/s = 500000/(486 x 434.783 x 2.15292)
    // = 1.09909 mm2/mm, NOT overstressed (the old fixed cot 2.5 had V_Rd,max = 451.276 kN).
    let (avs, status) = ec2_beam_with(600.0, 0.0, 500e3, 500.0, 0);
    close(avs, 1.09909, 2e-5, "Asw/s at 500 kN");
    assert_eq!(status, "OK");
    // V = 600 kN: cot = 1.52575, Asw/s = 600000/(486 x 434.783 x 1.52575) = 1.86105
    close(ec2_beam_with(600.0, 0.0, 600e3, 500.0, 0).0, 1.86105, 2e-5, "Asw/s at 600 kN");
    // V = 200 kN: cot = 2.5 (clamped), Asw/s = 200000/(486 x 434.783 x 2.5) = 0.37860
    close(ec2_beam_with(600.0, 0.0, 200e3, 500.0, 0).0, 0.37860, 2e-5, "Asw/s at 200 kN");
    // V = 700 kN > V_Rd,max(45) = 654.350 kN: cot = 1, overstressed
    assert_eq!(ec2_beam_with(600.0, 0.0, 700e3, 500.0, 0).1, "OVERSTRESSED");
}

#[test]
fn d100_seismic_combination_fixes_45_degrees() {
    // V = 200 kN with a Quake case in the combination: cot = 1, Asw/s = 200000/(486 x 434.783) = 0.94650
    for seismic in [1, 2, 3] {
        let (avs, _) = ec2_beam_with(600.0, 0.0, 200e3, 500.0, seismic);
        close(avs, 0.94650, 2e-5, &format!("seismic variant {seismic}"));
    }
    // a dead pattern or a time-history case is not seismic in this classification
    for other in [0, 4] {
        close(ec2_beam_with(600.0, 0.0, 200e3, 500.0, other).0, 0.37860, 2e-5, "not seismic");
    }
}

#[test]
fn d100_vendor_example_theta_at_the_limit() {
    // en_2_2004_ex001: bw 230, d 490, fck 30, fyk 460 (fywd 400), V_Ed = 110 kN:
    // v_Ed = 110000/(230 x 490) = 0.976 MPa, X = 8.28 -> cot clamped to 2.5 (theta 21.8 deg):
    // Asw/s = 110000/(441 x 400 x 2.5) = 0.24943 mm2/mm (vendor 249.5 mm2/m).
    let r = sap_design::concrete::design_ec2_beam_doubly(
        0.0, 110e3, 230.0, 490.0, 60.0, 30.0, 460.0, 460.0, Units::NMmC,
    );
    close(r.av_s, 110_000.0 / (441.0 * 400.0 * 2.5), 1e-9, "Asw/s vendor example");
    close(r.av_s, 0.24943, 1e-4, "Asw/s vendor example (printed)");
    assert!(!r.overstressed);
}

#[test]
fn d101_stirrup_minimum_uses_the_stirrup_yield() {
    // V = 20 kN is below V_Rd,c: Asw/s = rho_w,min bw with rho_w,min = 0.08 sqrt(30)/fywk:
    // fywk = 250: 0.08 x 5.47723/250 = 0.001753, x 300 = 0.52581 mm2/mm (before: /500 = 0.26291)
    let (avs, _) = ec2_beam_with(600.0, 0.0, 20e3, 250.0, 0);
    close(avs, 0.08 * 30.0_f64.sqrt() / 250.0 * 300.0, 1e-9, "rho_w,min bw (stirrups 250)");
    close(avs, 0.52581, 1e-4, "hand value");
    // fywk = fyk = 500 keeps the old minimum 0.26291
    close(ec2_beam_with(600.0, 0.0, 20e3, 500.0, 0).0, 0.26291, 1e-4, "stirrups = longitudinal");
}

#[test]
fn d101_size_factor_and_rho_l_in_vrd_c() {
    // Shallow beam 300 x 260, d = 200 mm: k = 1 + sqrt(200/200) = 2, rho_l = 0.02 (cap; the
    // M3 below needs more than 1200 mm2), fck = 30: V_Rd,c = (0.12 x 2 x (100 x 0.02 x 30)^(1/3))
    // x 300 x 200 = 0.93957 MPa x 60000 = 56.37 kN (v_min = 0.035 x 2^1.5 x sqrt(30) =
    // 0.5422 MPa is lower). V = 54 kN lies below it: Asw/s = rho_w,min bw = 0.26291 mm2/mm.
    // Without k and rho_l the old V_Rd,c was 0.857 bw d = 51.4 kN and Asw/s = 54000/(180 x
    // 434.783 x 2.5) = 0.27600.
    let vrdc = 0.12 * 2.0 * (100.0_f64 * 0.02 * 30.0).cbrt() * 300.0 * 200.0;
    close(vrdc, 56_370.0, 1e-3, "hand V_Rd,c");
    let (avs, _) = ec2_beam_with(260.0, 1.0e8, 54e3, 500.0, 0);
    close(avs, 0.08 * 30.0_f64.sqrt() / 500.0 * 300.0, 1e-9, "Asw/s inside V_Rd,c");
    // above V_Rd,c the computed value governs: V = 60 kN: 60000/(180 x 434.783 x 2.5) = 0.30667
    // (cot = 2.5: X = 0.9 x 0.528 x 17/(60000/60000) = 8.08)
    let (avs, _) = ec2_beam_with(260.0, 1.0e8, 60e3, 500.0, 0);
    close(avs, 60_000.0 / (180.0 * (500.0 / 1.15) * 2.5), 1e-6, "Asw/s above V_Rd,c");
}

#[test]
fn d101_v_min_with_size_factor_hand_numbers() {
    // d = 540: k = 1 + sqrt(200/540) = 1.60858, v_min = 0.035 k^1.5 sqrt(30) = 0.39110 MPa
    // (before 0.035 sqrt(30) = 0.19170); vendor example d = 490: k = 1.6389, v_min = 0.4022,
    // V_Rd,c = 0.4022 x 230 x 490 = 45.33 kN (rho_l = 0 at the support).
    let k: f64 = 1.0 + (200.0_f64 / 540.0).sqrt();
    close(0.035 * k.powf(1.5) * 30.0_f64.sqrt(), 0.39110, 1e-4, "v_min d 540");
    let k: f64 = 1.0 + (200.0_f64 / 490.0).sqrt();
    close(0.035 * k.powf(1.5) * 30.0_f64.sqrt() * 230.0 * 490.0, 45_329.0, 1e-4, "V_Rd,c example");
    // 600 x 300 beam, no moment: V just below / above 0.3911 x 300 x 540 = 63.36 kN. The
    // old v_min (0.1917 MPa -> 31.1 kN) made V = 50 kN "above V_Rd,c": Asw/s = 50000/(486 x
    // 434.783 x 2.5) = 0.09465, smaller than rho_w,min bw = 0.26291, so the result is the
    // minimum either way; the two V_Rd,c differ only for the check against the strut.
    let (avs, status) = ec2_beam_with(600.0, 0.0, 50e3, 500.0, 0);
    close(avs, 0.26291, 1e-4, "minimum");
    assert_eq!(status, "OK");
}

// ---------------------------------------------------------------- D103

fn cable_model(cable: bool, p: f64) -> SapModel {
    let mut m = aci_beam(2.5, 2.5, vec![station(p, 0.0, 0.0, 1000.0)]);
    if cable {
        m.frames.get_mut("B1").unwrap().cable_params = Some(sap_model::CableParams {
            undeformed_length: None,
            target_tension_i: None,
            horizontal_tension: None,
            num_segments: 1,
        });
    }
    m
}

#[test]
fn d103_cables_are_not_designed() {
    // The frame has a concrete material and a force state that would be designed (and, with a
    // NaN force, refused): as a cable it gets no result and no error.
    let mut m = cable_model(true, 50.0);
    try_run_concrete_design(&mut m).unwrap();
    assert!(m.concrete_design_results.is_empty());
    let mut m = cable_model(true, f64::NAN);
    try_run_concrete_design(&mut m).unwrap();
    assert!(m.concrete_design_results.is_empty());
    // a concrete material without rebar yield stress on a cable is not refused either
    let mut m = cable_model(true, 50.0);
    let c = m.materials.get_mut("4000Psi").unwrap();
    c.fy = 0.0;
    c.fys = 0.0;
    try_run_concrete_design(&mut m).unwrap();
    assert!(m.concrete_design_results.is_empty());
}

#[test]
fn d103_the_same_member_without_cable_data_is_still_designed_or_refused() {
    let mut m = cable_model(false, 50.0);
    try_run_concrete_design(&mut m).unwrap();
    assert!(m.concrete_design_results["B1"].as_bot > 0.0);
    let mut m = cable_model(false, f64::NAN);
    assert!(try_run_concrete_design(&mut m).is_err());
    let mut m = cable_model(false, 50.0);
    let c = m.materials.get_mut("4000Psi").unwrap();
    c.fy = 0.0;
    c.fys = 0.0;
    assert!(try_run_concrete_design(&mut m).is_err());
}

// ---------------------------------------------------------------- D102

fn short_column_ratio(t3: f64, t2: f64, p_sap: f64, m3: f64, m2: f64) -> f64 {
    let mut m = column_model(t3, t2, 120.0, "ACI 318-14", vec![station(p_sap, 0.0, m2, m3)]);
    try_run_concrete_design(&mut m).unwrap();
    m.concrete_design_results["C1"].pmm_ratio
}

#[test]
fn d102_minimum_eccentricity_values() {
    // e_min = 0.6 + 0.03 h in: h = 12, 24, 36 in -> 0.960, 1.320, 1.680 in (the old 0.05 h gave
    // 0.6, 1.2, 1.8); h = 600 mm -> (0.6 + 0.03 x 23.622) in = 1.30866 in = 33.2400 mm.
    let e = |h: f64| sap_design::concrete::aci_min_eccentricity(h, Units::KipInF);
    close(e(12.0), 0.960, 1e-12, "e_min 12 in");
    close(e(24.0), 1.320, 1e-12, "e_min 24 in");
    close(e(36.0), 1.680, 1e-12, "e_min 36 in");
    close(
        sap_design::concrete::aci_min_eccentricity(600.0, Units::NMmC),
        33.2400,
        1e-6,
        "e_min 600 mm",
    );
}

#[test]
fn d102_minimum_eccentricity_moment_is_applied_one_direction_at_a_time() {
    // 24 x 12 column, l = 180 in, E = 3605 ksi, Pu = 500 kip (D96: Pc33 = 4337.362, Pc22 =
    // 1084.341 kip): delta33 = 1/(1 - 500/(0.75 Pc33)), delta22 = 1/(1 - 500/(0.75 Pc22)).
    // Pu e3 = 500 x 1.32 = 660 kip-in, Pu e2 = 500 x 0.96 = 480 kip-in. The ratio is the larger
    // contour of the pairs (max(M3, Pu e3) delta33, M2 delta22) and (M3 delta33, max(M2, Pu e2) delta22).
    let pu = 500.0;
    let d33 = 1.0 / (1.0 - pu / (0.75 * 4337.362));
    let d22 = 1.0 / (1.0 - pu / (0.75 * 1084.341));
    for (m3, m2) in [(300.0, 0.0), (660.0, 0.0), (1000.0, 10.0), (50.0, 700.0), (0.0, 0.0)] {
        let a = expected_ratio(24.0, 12.0, pu, f64::max(m3, 660.0) * d33, m2 * d22);
        let b = expected_ratio(24.0, 12.0, pu, m3 * d33, f64::max(m2, 480.0) * d22);
        let mut m = slender_column(24.0, 12.0, 180.0, station(-pu, 0.0, m2, m3));
        try_run_concrete_design(&mut m).unwrap();
        close(
            m.concrete_design_results["C1"].pmm_ratio,
            a.max(b),
            1e-5,
            &format!("M3 = {m3}, M2 = {m2}"),
        );
    }
}

#[test]
fn d102_a_station_below_the_minimum_is_evaluated_with_the_minimum_and_amplified() {
    // 24 x 12 column, l = 180 in, E = 3605 ksi (D96): Pc33 = 4337.362, Pc22 = 1084.341 kip.
    // Pu = 600 kip: delta33 = 1/(1 - 600/(0.75 x 4337.362)) = 1.22616,
    // delta22 = 1/(1 - 600/(0.75 x 1084.341)) = 3.81352. e3 = 0.6 + 0.03 x 24 = 1.32 in,
    // e2 = 0.6 + 0.03 x 12 = 0.96 in; Pu e3 = 792 kip-in, Pu e2 = 576 kip-in.
    // Station M3 = 1e-6 kip-in (above the old 1e-9 bypass), M2 = 0: the pairs are
    // (792 delta33, 0) and (1e-6 delta33, 576 delta22) and the ratio is the larger contour.
    let pu = 600.0;
    let d33 = 1.0 / (1.0 - pu / (0.75 * 4337.362));
    let d22 = 1.0 / (1.0 - pu / (0.75 * 1084.341));
    close(d33, 1.22616, 1e-5, "delta33");
    close(d22, 3.81352, 1e-5, "delta22");
    let pair_a = expected_ratio(24.0, 12.0, pu, 792.0 * d33, 0.0);
    let pair_b = expected_ratio(24.0, 12.0, pu, 1e-6 * d33, 576.0 * d22);
    let mut m = slender_column(24.0, 12.0, 180.0, station(-pu, 0.0, 0.0, 1e-6));
    try_run_concrete_design(&mut m).unwrap();
    let ratio = m.concrete_design_results["C1"].pmm_ratio;
    close(ratio, pair_a.max(pair_b), 1e-5, "ratio of the pairs");
    // without the minimum the station would show the axial ratio only (600/678.6 = 0.88)
    assert!(ratio > 2.0, "ratio {ratio}");
    assert!(pair_b > pair_a);
}

#[test]
fn d102_tension_has_no_minimum_eccentricity() {
    // P = +100 kip is tension: Pu < 0. 24 x 24, Ast = 0.02 x 576 = 11.52 in2:
    // phi Pnt = 0.9 x 60 x 11.52 = 622.08 kip, axial ratio 100/622.08 = 0.160751; no moment, no minimum.
    close(short_column_ratio(24.0, 24.0, 100.0, 0.0, 0.0), 100.0 / 622.08, 1e-9, "tension ratio");
}

// ---------------------------------------------------------------- D104

fn side_steel(code_ec2: bool, m3: f64, m2: f64) -> f64 {
    if code_ec2 {
        let mut m = ec2_model(600.0, 300.0, false, 500.0, 60.0, vec![station(0.0, 0.0, m2, m3)]);
        try_run_concrete_design(&mut m).unwrap();
        m.concrete_design_results["M1"].as_side
    } else {
        let mut m = aci_beam(2.5, 2.5, vec![station(0.0, 0.0, m2, m3)]);
        try_run_concrete_design(&mut m).unwrap();
        m.concrete_design_results["B1"].as_side
    }
}

#[test]
fn d104_noise_level_m2_gives_no_side_steel() {
    // N, mm: M3 = 2e8 N mm, M2 = 1e-9 N mm is 5e-18 of M3, below the 1e-9 relative threshold.
    // The old absolute threshold 1e-12 designed it and, with EC2, reported the side-face
    // minimum steel as_min = rho_min x h x d3 (rho_min = 0.26 x 0.3 x 30^(2/3)/500 = 0.0015)
    // > 0.
    assert_eq!(side_steel(true, 2.0e8, 1e-9), 0.0);
    // kip, in: the same ratio in other units
    assert_eq!(side_steel(false, 2000.0, 1e-9), 0.0);
}

#[test]
fn d104_m2_of_a_thousandth_of_m3_is_designed_as_before() {
    // M2 = 1e-3 M3 (far above the 1e-9 threshold): M2 = 2e5 N mm gives side steel above 0
    // (EC2: at least the minimum steel 0.0015 x 300 x d3 with d3 = 300 - 60 = 240 mm = 108 mm2).
    let as_side = side_steel(true, 2.0e8, 2.0e5);
    assert!(as_side >= 0.26 * 0.3 * 30.0_f64.powf(2.0 / 3.0) / 500.0 * 600.0 * 240.0 * 0.99);
    assert!(side_steel(false, 2000.0, 2.0) > 0.0);
    // a pure M2 beam (M3 = 0) is designed too: the scale is the beam's own largest moment
    assert!(side_steel(true, 0.0, 1.0e8) > 0.0);
}
