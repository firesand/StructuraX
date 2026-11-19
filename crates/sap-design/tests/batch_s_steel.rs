//! Batch S steel-design repairs (D85-D93). Every expected value is a hand calculation
//! written out beside the assertion, in kip and in (or N and mm where stated).

use sap_design::try_run_steel_design;
use sap_model::{CaseResults, FramePropType, FrameSection, FrameStationResult, SapModel};
use sap_section::{make_box_tube, make_i_section, make_pipe};
use sap_units::Units;

/// One-member model (vertical, `len` long) with one load case holding the given stations.
fn one_member(
    units: Units,
    code: &str,
    sec: sap_model::FrameSection,
    len: f64,
    stations: Vec<FrameStationResult>,
) -> SapModel {
    let mut model = SapModel::new(units);
    model.steel_design_code = code.to_string();
    let name = sec.name.clone();
    model.frame_sections.insert(name.clone(), sec);
    model.add_point("PI", 0.0, 0.0, 0.0);
    model.add_point("PJ", 0.0, 0.0, len);
    model.add_frame("M1", "PI", "PJ", &name);
    let mut cr = CaseResults {
        case_name: "LC".into(),
        ..Default::default()
    };
    cr.frame_forces.insert("M1".into(), stations);
    model.case_results.insert("LC".into(), cr);
    model
}

fn st(p: f64, m3: f64, t: f64) -> FrameStationResult {
    FrameStationResult {
        station: 60.0,
        rel_dist: 0.5,
        p,
        m3,
        t,
        ..Default::default()
    }
}

// ---------------------------------------------------------------- D85

/// AISC H3-6 for a Pipe (vendor manual p. 3-81): Pr/Pc + sqrt[(Mr33/Mc33)^2 + (Mr22/Mc22)^2]
/// plus the square of (Vr3/Vc3 + Vr2/Vc2 + Tr/Tc), at most 1.
/// HSS 10 x 0.5, Fy 50, E 29000, L = 120 in. D/t = 20 <= 0.07 E/Fy = 40.6 (compact).
/// Z = (10^3 - 9^3)/6 = 45.16667 in^3, S = pi (10^4 - 9^4)/64 / 5 = 33.7623 in^3
/// Mp = min(50 Z, 1.6 * 50 S) = 2258.333; phi Mn = 0.9 Mp = 2032.500 kip-in.
/// C = (pi/2)(D - t)^2 t = 70.88218; Fcr = min(max(243.46, 194.54), 0.6 Fy = 30) = 30;
/// Tn = 2126.466; phi Tn = 1913.819 kip-in.
/// M3 = 0.9 phi Mn, T = 0.4 phi Tn: ratio = 0.9 + 0.4^2 = 1.06 (the squared form gave 0.97).
#[test]
fn d85_pipe_h3_6_takes_the_square_root_of_the_squared_moment_ratios() {
    let pipe = make_pipe("HSS10X0.5", "A992Fy50", 10.0, 0.5);
    let phi_mn = 0.9 * 50.0 * (10.0_f64.powi(3) - 9.0_f64.powi(3)) / 6.0;
    assert!((phi_mn - 2032.5).abs() < 1e-9);
    let phi_tn = 0.9 * 30.0 * 0.5 * std::f64::consts::PI * 9.5_f64.powi(2) * 0.5;
    assert!((phi_tn - 1913.819).abs() < 1e-3);
    let mut model = one_member(
        Units::KipInF,
        "AISC 360-10",
        pipe,
        120.0,
        vec![st(0.0, 0.9 * phi_mn, 0.4 * phi_tn)],
    );
    try_run_steel_design(&mut model).unwrap();
    let r = &model.steel_design_results["M1"];
    assert_eq!(r.eq_name, "H3-6");
    assert!((r.total_ratio - 1.06).abs() < 1e-9, "{}", r.total_ratio);
    assert_eq!(r.status, "OVERSTRESSED");
    // the two flexural ratios stay reported separately
    assert!((r.mr_major_ratio - 0.9).abs() < 1e-9);
    assert!(r.mr_minor_ratio.abs() < 1e-12);

    // Two-axis check of the radical with unequal ratios 0.3 and 0.4 and T = 0.5 phi Tn:
    // H3-6 = sqrt(0.09 + 0.16) + 0.5^2 = 0.5 + 0.25 = 0.75 (the squared form gave 0.5), which
    // exceeds H1-1b = 0.3 + 0.4 = 0.7. The pipe is axisymmetric: phi Mn22 = phi Mn33.
    let pipe = make_pipe("HSS10X0.5", "A992Fy50", 10.0, 0.5);
    let mut st2 = st(0.0, 0.3 * phi_mn, 0.5 * phi_tn);
    st2.m2 = 0.4 * phi_mn;
    let mut model = one_member(Units::KipInF, "AISC 360-10", pipe, 120.0, vec![st2]);
    try_run_steel_design(&mut model).unwrap();
    let r = &model.steel_design_results["M1"];
    assert_eq!(r.eq_name, "H3-6");
    assert!((r.total_ratio - 0.75).abs() < 1e-9, "{}", r.total_ratio);
    assert!((r.mr_major_ratio - 0.3).abs() < 1e-9);
    assert!((r.mr_minor_ratio - 0.4).abs() < 1e-9);
}

/// Negative: a Pipe with T <= 0.2 Tc keeps H1 (no H3-6).
#[test]
fn d85_pipe_with_small_torsion_keeps_h1() {
    let pipe = make_pipe("HSS10X0.5", "A992Fy50", 10.0, 0.5);
    let phi_mn = 2032.5;
    let phi_tn = 1913.819;
    let mut model = one_member(
        Units::KipInF,
        "AISC 360-10",
        pipe,
        120.0,
        vec![st(0.0, 0.9 * phi_mn, 0.15 * phi_tn)],
    );
    try_run_steel_design(&mut model).unwrap();
    let r = &model.steel_design_results["M1"];
    assert_eq!(r.eq_name, "H1-1b");
    assert!((r.total_ratio - 0.9).abs() < 1e-9);
}

// ---------------------------------------------------------------- D86

const E_KSI: f64 = 29000.0;
const FY_KSI: f64 = 50.0;

fn root_e_fy() -> f64 {
    (E_KSI / FY_KSI).sqrt() // 24.0832
}

/// AISC E7-17/E7-18 effective width `be = 1.92 t sqrt(E/f) [1 - k/(b/t) sqrt(E/f)]`.
fn be(b: f64, t: f64, f: f64, k: f64) -> f64 {
    let r = (E_KSI / f).sqrt();
    1.92 * t * r * (1.0 - k / (b / t) * r)
}

/// phi Pn = 0.9 Q 0.658^(Q Fy/Fe) Fy A with Fe = pi^2 E / (KL/r)^2 (AISC E3, E7), K = 1.
fn phi_pn(q: f64, area: f64, r_min: f64, len: f64) -> f64 {
    let fe = std::f64::consts::PI.powi(2) * E_KSI / (len / r_min).powi(2);
    assert!(q * FY_KSI / fe <= 2.25);
    0.9 * q * 0.658_f64.powf(q * FY_KSI / fe) * FY_KSI * area
}

/// Design entry for a short vertical member under a small compression; returns phi Pnc.
fn designed_phi_pnc(mut sec: FrameSection, len: f64) -> f64 {
    // FrameSection::default() carries the radii of a W18 (7.38, 1.65): give the test sections
    // their own, r = sqrt(I/A), and no bottom flange where the shape has none.
    sec.r33 = (sec.i33 / sec.area).sqrt();
    sec.r22 = (sec.i22 / sec.area).sqrt();
    if !matches!(
        sec.prop_type,
        FramePropType::I | FramePropType::Channel | FramePropType::DbChannel
    ) {
        sec.tfb = 0.0;
        sec.t2b = 0.0;
    }
    let mut model = one_member(
        Units::KipInF,
        "AISC 360-10",
        sec,
        len,
        vec![st(-1.0, 0.0, 0.0)],
    );
    try_run_steel_design(&mut model).unwrap();
    model.steel_design_results["M1"].phi_pnc
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}

/// AISC E4-5 written out: Fe = (Fa + Fez)/(2H) [1 - sqrt(1 - 4 Fa Fez H/(Fa + Fez)^2)].
fn e4_5(fa: f64, fez: f64, h: f64) -> f64 {
    (fa + fez) / (2.0 * h) * (1.0 - (1.0 - 4.0 * fa * fez * h / ((fa + fez) * (fa + fez))).sqrt())
}

/// D128 hand calculation of phi Pn with flexural-torsional buckling for the shapes of E4-5 (slender
/// tee and double angle with Fa = Fe22, channel with Fa = Fe33), K = 1, Kz Lz = `lz`:
/// r0^2 = x0^2 + y0^2 + (I22 + I33)/A (E4-11), H = 1 - (x0^2 + y0^2)/r0^2 (E4-10),
/// Fez = (pi^2 E Cw/(Kz Lz)^2 + G J)/(A r0^2), G = E/2.6, Fe = min(pi^2 E/(KL/r)max^2, E4-5),
/// phi Pn = 0.9 Q 0.658^(Q Fy/Fe) Fy A (E7-2, with Q Fy/Fe <= 2.25).
#[allow(clippy::too_many_arguments)]
fn phi_pn_e4_5(
    q: f64,
    a: f64,
    i33: f64,
    i22: f64,
    j: f64,
    x0: f64,
    y0: f64,
    cw: f64,
    lz: f64,
    kl_r_a: f64,
    kl_r_max: f64,
) -> f64 {
    let pi2e = std::f64::consts::PI.powi(2) * E_KSI;
    let r0sq = x0 * x0 + y0 * y0 + (i22 + i33) / a;
    let h = 1.0 - (x0 * x0 + y0 * y0) / r0sq;
    let fez = (pi2e * cw / (lz * lz) + E_KSI / 2.6 * j) / (a * r0sq);
    let fe = (pi2e / (kl_r_max * kl_r_max)).min(e4_5(pi2e / (kl_r_a * kl_r_a), fez, h));
    assert!(q * FY_KSI / fe <= 2.25);
    0.9 * q * 0.658_f64.powf(q * FY_KSI / fe) * FY_KSI * a
}

/// Box 12 x 12 x 3/16 (sharp corners): b = 12 - 3 (0.1875) = 11.4375, b/t = 61 > 1.40 r = 33.72;
/// be = 1.92 (0.1875) r [1 - 0.38 r / 61] = 7.3692; Aeff = A - 4 (b - be) t = 5.80817;
/// Qa = Aeff/A = 0.65560 (E7-18, f = Fy; manual p. 3-33). L = 12 in: Fe = 46235.7, phi Pn = 261.290.
#[test]
fn d86_box_gets_qa_from_the_effective_width_of_its_walls() {
    let sec = make_box_tube("BOX12", "A992Fy50", 12.0, 12.0, 0.1875, 0.1875);
    let a = 144.0 - 11.625_f64.powi(2);
    assert!((a - 8.85938).abs() < 1e-5);
    let i = (12.0_f64.powi(4) - 11.625_f64.powi(4)) / 12.0;
    assert!((i - 206.084).abs() < 1e-3);
    let b = 12.0 - 3.0 * 0.1875;
    let be_v = be(b, 0.1875, FY_KSI, 0.38);
    assert!((be_v - 7.3692).abs() < 1e-4);
    let a_eff = a - 4.0 * (b - be_v) * 0.1875;
    let q = a_eff / a;
    assert!((q - 0.65560).abs() < 1e-5);
    let expected = phi_pn(q, a, (i / a).sqrt(), 12.0);
    assert!((expected - 261.290).abs() < 0.01, "hand {expected}");
    let got = designed_phi_pnc(sec, 12.0);
    assert!(rel(got, expected) < 1e-6, "got {got} expected {expected}");
}

fn angle8() -> FrameSection {
    FrameSection {
        name: "L8X8X1/2".into(),
        material: "A992Fy50".into(),
        prop_type: FramePropType::Angle,
        t3: 8.0,
        t2: 8.0,
        tf: 0.5,
        tw: 0.5,
        area: 7.75,
        i33: 48.4375, // r = 2.5 in (a test input, not a catalogue value)
        i22: 48.4375,
        s33: 8.0,
        s22: 8.0,
        z33: 14.0,
        z22: 14.0,
        as2: 4.0,
        as3: 4.0,
        torsional_j: 0.6,
        ..FrameSection::default()
    }
}

/// Single angle 8 x 8 x 1/2: b/t = 16, between 0.45 r = 10.84 and 0.91 r = 21.92:
/// Qs = 1.34 - 0.76 (16)/24.0832 = 0.83508 (E7-11, p. 3-32). D128 (G81): under compression the
/// single angle is no longer designed (the vendor uses its principal axes, r_z and E4-6), so the
/// design is refused by name; the Q factor itself is still computed and checked directly.
#[test]
fn d86_single_angle_gets_qs_from_the_longest_leg() {
    let q = 1.34 - 0.76 * 16.0 / root_e_fy();
    assert!((q - 0.83508).abs() < 1e-5);
    let got = sap_design::steel::compute_slender_q_factor(&angle8(), E_KSI, FY_KSI, FY_KSI).unwrap();
    assert!((got - q).abs() < 1e-9, "got {got} expected {q}");
    let mut model = one_member(
        Units::KipInF,
        "AISC 360-10",
        angle8(),
        12.0,
        vec![st(-1.0, 0.0, 0.0)],
    );
    let err = try_run_steel_design(&mut model).unwrap_err();
    assert!(matches!(
        err,
        sap_design::DesignError::AiscLimitStateNotBuilt { .. }
    ));
    let msg = err.to_string();
    assert!(msg.contains("'M1'") && msg.contains("single angle"), "{msg}");
}

/// Channel t3 = 8, t2 = 4, tf = 0.2, tw = 0.3: bf/tf = 20 (E7.1 for channels, not bf/2tf = 10):
/// 0.56 r = 13.49 < 20 <= 1.03 r = 24.81: Qs = 1.415 - 0.74 (20)/24.0832 = 0.80046.
/// Web h/tw = 7.6/0.3 = 25.3 < 1.49 r = 35.88: Qa = 1.
#[test]
fn d86_channel_flange_uses_bf_over_tf() {
    let sec = FrameSection {
        name: "C8".into(),
        material: "A992Fy50".into(),
        prop_type: FramePropType::Channel,
        t3: 8.0,
        t2: 4.0,
        tf: 0.2,
        tfb: 0.2,
        tw: 0.3,
        area: 2.0 * 4.0 * 0.2 + 7.6 * 0.3, // 3.88
        i33: 3.88 * 9.0,                   // r = 3 in
        i22: 3.88 * 1.5625,                // r = 1.25 in
        s33: 8.0,
        s22: 2.0,
        z33: 9.0,
        z22: 3.0,
        as2: 2.28,
        as3: 1.6,
        torsional_j: 0.05,
        ..FrameSection::default()
    };
    let q = 1.415 - 0.74 * 20.0 / root_e_fy();
    assert!((q - 0.80046).abs() < 1e-5);
    // D128: E4-5 about the symmetry axis 3 with the channel's own shear centre (plates, no
    // fillets): b' = bf - tw/2 = 3.85, h = d - tf = 7.8, e = 3 b'^2 tf/(6 b' tf + h tw) = 1.277802,
    // xbar (back of web) = (8 0.3 0.15 + 2 3.7 0.2 2.15)/3.88 = 0.912887, x0 = xbar + e - tw/2 =
    // 2.040688, Cw = tf b'^3 h^2/12 (3 b' tf + 2 h tw)/(6 b' tf + h tw) = 58.114978, y0 = 0.
    // Published value of the hand run: r0^2 14.726909, H 0.717225, Fe33 17888.658,
    // Fez 2031.289179, Fe 1962.877914, Fcr 39.683058, phi Pn = 138.573237 (flexural alone gave
    // 139.009089). The section inherits t2b = 7.5: the channel code reads t2.
    let (bp, h): (f64, f64) = (4.0 - 0.15, 7.8);
    let e_sc: f64 = 3.0 * bp * bp * 0.2 / (6.0 * bp * 0.2 + h * 0.3);
    let xbar: f64 = (8.0 * 0.3 * 0.15 + 2.0 * 3.7 * 0.2 * (0.3 + 1.85)) / 3.88;
    let cw: f64 = 0.2 * bp.powi(3) * h * h / 12.0 * (3.0 * bp * 0.2 + 2.0 * h * 0.3)
        / (6.0 * bp * 0.2 + h * 0.3);
    assert!((e_sc - 1.277802).abs() < 1e-6 && (xbar - 0.912887).abs() < 1e-6);
    assert!((cw - 58.114978).abs() < 1e-5);
    let expected = phi_pn_e4_5(
        q,
        3.88,
        3.88 * 9.0,
        3.88 * 1.5625,
        0.05,
        xbar + e_sc - 0.15,
        0.0,
        cw,
        12.0,
        12.0 / 3.0,
        12.0 / 1.25,
    );
    assert!((expected - 138.573237).abs() < 1e-4, "hand {expected}");
    let got = designed_phi_pnc(sec, 12.0);
    assert!(rel(got, expected) < 1e-6, "got {got} expected {expected}");
    assert!(got < 139.009089, "flexural buckling alone is the old, higher value");
}

fn tee(t3: f64) -> FrameSection {
    let area = 8.0 * 0.5 + (t3 - 0.5) * 0.3;
    FrameSection {
        name: "T".into(),
        material: "A992Fy50".into(),
        prop_type: FramePropType::T,
        t3,
        t2: 8.0,
        tf: 0.5,
        tw: 0.3,
        area,
        i33: area * 4.0,  // r = 2 in
        i22: area * 2.25, // r = 1.5 in
        s33: 5.0,
        s22: 4.0,
        z33: 9.0,
        z22: 7.0,
        as2: 2.0,
        as3: 4.0,
        torsional_j: 0.3,
        ..FrameSection::default()
    }
}

/// T with a stem d = 9, tw = 0.3: d/t = 30 > 1.03 r = 24.806: Qs = 0.69 E/(Fy (d/t)^2) = 0.44467
/// (unstiffened stem, E7-15; the flange bf/2tf = 8 gives Qs = 1). With d = 6.6: d/t = 22 lies in
/// the middle branch 0.75 r = 18.06 < 22 <= 24.806: Qs = 1.908 - 1.22 (22)/24.0832 = 0.79353
/// (E7-14 with the ratio d/t; the printed page omits it, a misprint, see the code).
#[test]
fn d86_tee_stem_is_an_unstiffened_element() {
    let q = 0.69 * E_KSI / (FY_KSI * 900.0);
    assert!((q - 0.44467).abs() < 1e-5);
    // D128: slender tee, E4-5 with Fe22 and Fez (Cw = 0, x0 = 0): ybar from the flange face =
    // (8 0.5 0.25 + 8.5 0.3 (0.5 + 4.25))/6.55 = 2.001908, y0 = ybar - tf/2 = 1.751908,
    // r0^2 = y0^2 + 6.25 = 9.319183, H = 0.670660, Fez = G J/(A r0^2) = 54.818451,
    // Fe = 54.596229, Fcr = 18.749096, phi Pn = 110.525923 (flexural alone: 130.793060).
    let sec = tee(9.0);
    let ybar: f64 = (8.0 * 0.5 * 0.25 + 8.5 * 0.3 * (0.5 + 4.25)) / 6.55;
    assert!((ybar - 2.001908).abs() < 1e-6);
    let expected = phi_pn_e4_5(
        q,
        sec.area,
        sec.i33,
        sec.i22,
        0.3,
        0.0,
        ybar - 0.25,
        0.0,
        12.0,
        12.0 / 1.5,
        12.0 / 1.5,
    );
    assert!((expected - 110.525923).abs() < 1e-4, "hand {expected}");
    let got = designed_phi_pnc(sec, 12.0);
    assert!(rel(got, expected) < 1e-6, "got {got} expected {expected}");

    let q2 = 1.908 - 1.22 * 22.0 / root_e_fy();
    assert!((q2 - 0.79353).abs() < 1e-5);
    // tee(6.6): ybar = (4 0.25 + 6.1 0.3 (0.5 + 3.05))/5.83 = 1.285849, Fez 78.377121,
    // Fe 78.173341, Fcr 32.082976, phi Pn = 168.339373 (flexural alone: 207.410890).
    let sec = tee(6.6);
    let ybar: f64 = (8.0 * 0.5 * 0.25 + 6.1 * 0.3 * (0.5 + 3.05)) / 5.83;
    assert!((ybar - 1.285849).abs() < 1e-6);
    let expected = phi_pn_e4_5(
        q2,
        sec.area,
        sec.i33,
        sec.i22,
        0.3,
        0.0,
        ybar - 0.25,
        0.0,
        12.0,
        12.0 / 1.5,
        12.0 / 1.5,
    );
    assert!((expected - 168.339373).abs() < 1e-4, "hand {expected}");
    let got = designed_phi_pnc(sec, 12.0);
    assert!(rel(got, expected) < 1e-6, "got {got} expected {expected}");
}

/// Double angle 2L8x8x1/2 with dis = 3/8: t2 = 16.375 is the total width, one leg is
/// (16.375 - 0.375)/2 = 8, b/t = 16, Qs = 0.83508 (old: b = 16.375 read as one leg, Q = 1).
#[test]
fn d86_double_angle_leg_is_half_the_total_width_less_the_gap() {
    let sec = FrameSection {
        name: "2L8".into(),
        material: "A992Fy50".into(),
        prop_type: FramePropType::DblAngle,
        t3: 8.0,
        t2: 16.375,
        dis: 0.375,
        tf: 0.5,
        tw: 0.5,
        area: 15.5,
        i33: 15.5 * 6.25,  // r = 2.5 in
        i22: 15.5 * 12.25, // r = 3.5 in
        s33: 16.0,
        s22: 30.0,
        z33: 28.0,
        z22: 50.0,
        as2: 8.0,
        as3: 8.0,
        torsional_j: 1.0,
        ..FrameSection::default()
    };
    let q = 1.34 - 0.76 * 16.0 / root_e_fy();
    // D128: E4-5 with Fe22 and Fez (Cw = 0): ybar = (16 0.5 0.25 + 2 7.5 0.5 (0.5 + 3.75))/15.5 =
    // 2.185484, y0 = 1.935484, r0^2 = y0^2 + (I22 + I33)/A = 22.246098, H = 0.831607,
    // Fez = G J/(A r0^2) = 32.347380, Fe = 32.340137, Fcr = 24.322731,
    // phi Pn = 339.302102 (flexural alone: 581.651885). The flexural value uses KL/r about 3
    // (4.8) and Fe22 uses KL/r22 = 12/3.5.
    let ybar: f64 = (16.0 * 0.5 * 0.25 + 2.0 * 7.5 * 0.5 * (0.5 + 3.75)) / 15.5;
    assert!((ybar - 2.185484).abs() < 1e-6);
    let expected = phi_pn_e4_5(
        q,
        15.5,
        15.5 * 6.25,
        15.5 * 12.25,
        1.0,
        0.0,
        ybar - 0.25,
        0.0,
        12.0,
        12.0 / 3.5,
        12.0 / 2.5,
    );
    assert!((expected - 339.302102).abs() < 1e-4, "hand {expected}");
    let got = designed_phi_pnc(sec, 12.0);
    assert!(rel(got, expected) < 1e-6, "got {got} expected {expected}");
}

/// Vendor Example 002 (AISC 360-10, published phi Pn = 506.1 kip): built-up I, d = 17, bf = 8,
/// tf = 1, tw = 0.25, h = 15, h/tw = 60 > 1.49 r = 35.88 (slender web), A = 19.75,
/// Iy = 2 (1)(8)^3/12 + 15 (0.25)^3/12 = 85.3529, ry = 2.07886, K = 1, L = 180 in.
/// Fe = pi^2 29000/(180/2.07886)^2 = 38.1772; f = Fcr(Q = 1) = 0.658^(50/38.1772) 50 = 28.9004;
/// be = 1.92 (0.25) sqrt(29000/f)(1 - 0.34/60 sqrt(29000/f)) = 12.4757; Aeff = 19.75 - (15 - be)(0.25)
/// = 19.1189; Qa = 0.96805; Fcr = 0.96805 (0.658^(0.96805 (50)/38.1772)) 50 = 28.4713;
/// phi Pn = 0.9 (28.4713)(19.75) = 506.077 kip. The old code used f = Fy: Qa = 0.93649, 498.121.
#[test]
fn d86_vendor_example_002_slender_web_uses_fcr_with_q_one() {
    let sec = make_i_section("BU17", "A992Fy50", 17.0, 8.0, 1.0, 0.25, 8.0, 1.0);
    let a = 16.0 + 15.0 * 0.25;
    let iy = 2.0 * 8.0_f64.powi(3) / 12.0 + 15.0 * 0.25_f64.powi(3) / 12.0;
    assert!((iy - 85.3529).abs() < 1e-4);
    let ry = (iy / a).sqrt();
    let fe = std::f64::consts::PI.powi(2) * E_KSI / (180.0 / ry).powi(2);
    assert!((fe - 38.1772).abs() < 1e-3);
    let f = 0.658_f64.powf(FY_KSI / fe) * FY_KSI;
    assert!((f - 28.9004).abs() < 1e-3);
    let be_v = 1.92 * 0.25 * (E_KSI / f).sqrt() * (1.0 - 0.34 / 60.0 * (E_KSI / f).sqrt());
    assert!((be_v - 12.4757).abs() < 1e-3);
    let q = (a - (15.0 - be_v) * 0.25) / a;
    assert!((q - 0.96805).abs() < 1e-4);
    let expected = phi_pn(q, a, ry, 180.0);
    assert!((expected - 506.077).abs() < 0.01, "hand {expected}");
    let got = designed_phi_pnc(sec, 180.0);
    assert!(rel(got, 506.077) < 1e-4, "got {got}");
    assert!(rel(got, 506.1) < 1e-4, "got {got} against the vendor 506.1");
}

/// Negative: a compact I (flange 7.3, web 26.8, both below the limits) has Q = 1 exactly; a Pipe
/// with D/t = 100 (between 0.11 E/Fy = 63.8 and 0.45 E/Fy = 261) keeps
/// Q = 0.038 E/(Fy D/t) + 2/3 = 0.88707.
#[test]
fn d86_compact_sections_and_moderate_pipe_keep_their_q() {
    let sec = make_i_section("W14", "A992Fy50", 14.0, 14.5, 0.71, 0.44, 14.5, 0.71);
    let q = sap_design::compute_slender_q_factor(&sec, E_KSI, FY_KSI, 40.0).unwrap();
    assert_eq!(q, 1.0);
    let pipe = make_pipe("P10", "A992Fy50", 10.0, 0.1);
    let q = sap_design::compute_slender_q_factor(&pipe, E_KSI, FY_KSI, 40.0).unwrap();
    assert!((q - (0.038 * 580.0 / 100.0 + 2.0 / 3.0)).abs() < 1e-12);
    assert!((q - 0.88707).abs() < 1e-5);
}

/// Pipe D = 30, t = 0.1: D/t = 300 > 0.45 E/Fy = 261: refused by name (manual p. 3-34).
#[test]
fn d86_pipe_beyond_the_limit_is_refused_by_name() {
    let pipe = make_pipe("P30", "A992Fy50", 30.0, 0.1);
    let mut model = one_member(
        Units::KipInF,
        "AISC 360-10",
        pipe,
        120.0,
        vec![st(-1.0, 0.0, 0.0)],
    );
    let err = try_run_steel_design(&mut model).unwrap_err();
    let text = err.to_string();
    assert!(
        text.contains("'M1'") && text.contains("P30") && text.contains("too slender"),
        "{text}"
    );
    assert!(model.steel_design_results.is_empty());
}

// ---------------------------------------------------------------- D87

/// Double channel t3 = 15, t2 = 8.5 (total width), dis = 0.5, tf = 0.5, tw = 0.4, Fy 50:
/// one flange bf = (8.5 - 0.5)/2 = 4, bf/tf = 8 <= 0.38 sqrt(E/Fy) = 9.15: flange flexure Compact
/// (old: 8.5/0.5 = 17, Noncompact). Web h = 15 - 2 (0.5) = 14, h/tw = 35 <= 3.76 r = 90.6: Compact.
#[test]
fn d87_double_channel_flange_is_one_channels_width() {
    let sec = FrameSection {
        prop_type: FramePropType::DbChannel,
        t3: 15.0,
        t2: 8.5,
        dis: 0.5,
        tf: 0.5,
        tfb: 0.5,
        tw: 0.4,
        ..FrameSection::default()
    };
    let c = sap_design::check_compactness_aisc(&sec, E_KSI, FY_KSI);
    assert!(8.0 <= 0.38 * root_e_fy());
    assert_eq!(c.flange_flexure, sap_design::CompactnessClass::Compact);
    assert_eq!(c.web_flexure, sap_design::CompactnessClass::Compact);
    // bf/tf = 8 is also below the compression limit 0.56 r = 13.49
    assert_eq!(c.flange_compression, sap_design::CompactnessClass::Compact);
}

/// Double angle t3 = 8, t2 = 16.375, dis = 0.375, t = 0.5: one leg b = (16.375 - 0.375)/2 = 8,
/// b/t = 16: 0.54 r = 13.0 < 16 <= 0.91 r = 21.92: Noncompact in flexure (old: b = 16.375,
/// b/t = 32.75, Slender), and beyond 0.45 r = 10.84: slender in compression.
#[test]
fn d87_double_angle_leg_is_one_angles_width() {
    let sec = FrameSection {
        prop_type: FramePropType::DblAngle,
        t3: 8.0,
        t2: 16.375,
        dis: 0.375,
        tf: 0.5,
        tw: 0.5,
        ..FrameSection::default()
    };
    let c = sap_design::check_compactness_aisc(&sec, E_KSI, FY_KSI);
    assert!(0.54 * root_e_fy() < 16.0 && 16.0 <= 0.91 * root_e_fy());
    assert_eq!(c.flange_flexure, sap_design::CompactnessClass::Noncompact);
    assert_eq!(c.flange_compression, sap_design::CompactnessClass::Slender);
}

/// Negative: a single Channel keeps bf = t2 (bf/tf = 4/0.2 = 20, Noncompact in flexure:
/// 0.38 r = 9.15 < 20 <= 1.0 r = 24.08) and a single Angle keeps its legs (b/t = 16).
#[test]
fn d87_channel_and_single_angle_are_unchanged() {
    let ch = FrameSection {
        prop_type: FramePropType::Channel,
        t3: 8.0,
        t2: 4.0,
        t2b: 0.0,
        tf: 0.2,
        tfb: 0.2,
        tw: 0.3,
        ..FrameSection::default()
    };
    let c = sap_design::check_compactness_aisc(&ch, E_KSI, FY_KSI);
    assert_eq!(c.flange_flexure, sap_design::CompactnessClass::Noncompact);
    let c = sap_design::check_compactness_aisc(&angle8(), E_KSI, FY_KSI);
    assert_eq!(c.flange_flexure, sap_design::CompactnessClass::Noncompact);
}

// ---------------------------------------------------------------- D88

/// N-mm model with S355-like steel (Fy 345 MPa, E 200000 MPa) and a 4 m member.
fn nmm_member(code: &str, sec: FrameSection, stations: Vec<FrameStationResult>) -> SapModel {
    let mut model = one_member(Units::NMmC, code, sec, 4000.0, stations);
    let mat = model.materials.get_mut("A992Fy50").unwrap();
    mat.fy = 345.0;
    mat.e = 200_000.0;
    mat.g = 200_000.0 / 2.6;
    model
}

/// Unequal-flange I: top 250 x 16, bottom 150 x 12, web 400 x 8 (d = 428 mm). The true warping
/// constant Cw = If1 If2/(If1 + If2) h0^2 = 4.978e11 mm^6 is 0.48 of the 1.038e12 the equal-flange
/// formula gives (If1 = 16 (250)^3/12 = 2.0833e7, If2 = 12 (150)^3/12 = 3.375e6, h0 = 414):
/// the design refuses it by name rather than use the doubly symmetric value (D88, G70).
#[test]
fn d88_unequal_flange_i_is_refused_under_aisc_and_ec3_with_m3() {
    let sec = make_i_section("UF", "A992Fy50", 428.0, 250.0, 16.0, 8.0, 150.0, 12.0);
    let if1 = 16.0 * 250.0_f64.powi(3) / 12.0;
    let if2 = 12.0 * 150.0_f64.powi(3) / 12.0;
    let cw_true = if1 * if2 / (if1 + if2) * 414.0_f64.powi(2);
    let cw_code = 0.25 * (if1 + if2) * 414.0_f64.powi(2);
    assert!((cw_true - 4.978e11).abs() < 2e8, "{cw_true}");
    assert!((cw_code - 1.038e12).abs() < 2e9, "{cw_code}");

    // AISC LRFD: refused with an axial force alone (E4 uses Cw) and with a moment.
    for station in [st(-100_000.0, 0.0, 0.0), st(0.0, 100.0e6, 0.0)] {
        let mut model = nmm_member("AISC 360-10", sec.clone(), vec![station]);
        let err = try_run_steel_design(&mut model).unwrap_err();
        let text = err.to_string();
        assert!(text.contains("'M1'") && text.contains("UF"), "{text}");
        assert!(text.contains("F4 / Cw for unequal flanges not implemented"), "{text}");
        assert!(model.steel_design_results.is_empty());
    }
    // EC3: refused when a station has M3 (Iw) ...
    let mut model = nmm_member("Eurocode 3-2005", sec.clone(), vec![st(0.0, 100.0e6, 0.0)]);
    let text = try_run_steel_design(&mut model).unwrap_err().to_string();
    assert!(text.contains("'M1'") && text.contains("Iw for unequal flanges not implemented"), "{text}");
    // ... and designed with an axial tension alone (no Iw is used).
    let mut model = nmm_member("Eurocode 3-2005", sec, vec![st(100_000.0, 0.0, 0.0)]);
    try_run_steel_design(&mut model).unwrap();
    // tension ratio 100 kN / (A fy) with A = 250 (16) + 150 (12) + 400 (8) = 9000 mm^2:
    // 100000 / (9000 * 345) = 0.032206
    let r = &model.steel_design_results["M1"];
    assert!((r.total_ratio - 100_000.0 / (9000.0 * 345.0)).abs() < 1e-9, "{}", r.total_ratio);
}

/// Negative: equal flanges (t2b = t2, tfb = tf) design as before. The zero bottom flange
/// (t2b = tfb = 0, read as "equal" by the rest of the code) is not refused as unequal; the shear
/// check of `shear.rs` reads t2b itself and refuses a zero width for its own reason (not D88).
#[test]
fn d88_equal_flange_i_still_designs() {
    let sec = make_i_section("EQ", "A992Fy50", 432.0, 250.0, 16.0, 8.0, 250.0, 16.0);
    for code in ["AISC 360-10", "Eurocode 3-2005"] {
        let mut model = nmm_member(code, sec.clone(), vec![st(0.0, 100.0e6, 0.0)]);
        try_run_steel_design(&mut model).unwrap();
        assert!(model.steel_design_results["M1"].total_ratio > 0.0);
    }
    let mut zero = sec;
    zero.t2b = 0.0;
    zero.tfb = 0.0;
    let mut model = nmm_member("AISC 360-10", zero, vec![st(0.0, 100.0e6, 0.0)]);
    if let Err(err) = try_run_steel_design(&mut model) {
        assert!(!err.to_string().contains("unequal flanges"), "{err}");
    }
}

// ---------------------------------------------------------------- D89

fn st3(p: f64, m3: f64, m2: f64) -> FrameStationResult {
    FrameStationResult {
        m2,
        ..st(p, m3, 0.0)
    }
}

/// Welded I 400 x 250, tf 11, tw 12 (Fy 345 MPa, E 200000 MPa, N and mm), L = 4 m, one station with
/// N = -500 kN and M2 = 10 kNm. D89 refused this until the Annex B factors were built (G71); it is now
/// designed by EC3 6.3.3(4). Hand calculation (single station: Cm = 1; flange (250 - 12)/2/11 = 10.82
/// is Class 3, 10 eps = 8.25 < 10.82 <= 14 eps = 11.55: elastic set and W_el):
/// A = 10036 mm^2, I33 = 2.62132e8, I22 = 2.87003e7, W_el,z = 229602 mm^3; lambda_y = 0.32721 (curve a),
/// lambda_z = 0.98888 (curve b), chi_y = 0.97105, chi_z = 0.60410; N_pl = 10036 x 345 = 3462420 N;
/// n_y = 0.148713, n_z = 0.239047; kzz = kyz = 1 + 0.6 x 0.98888 x 0.239047 = 1.141833 (cap 1.143428);
/// m_z = 10e6/(229602 x 345) = 0.126242; eq 6.61 = 0.148713 + 1.141833 x 0.126242 = 0.292861 (hand
/// script, fy 345), eq 6.62 = 0.239047 + 1.141833 x 0.126242 = 0.383195; cross section 0.270650.
/// The channel (no Annex B column) is refused by name. Compression alone and tension with a moment are
/// designed as before.
#[test]
fn d89_ec3_compression_with_bending_is_designed_by_annex_b_and_a_channel_is_refused() {
    let sec = make_i_section("WI", "A992Fy50", 400.0, 250.0, 11.0, 12.0, 250.0, 11.0);
    let code = "Eurocode 3-2005";

    let mut model = nmm_member(code, sec.clone(), vec![st3(-500_000.0, 0.0, 10.0e6)]);
    try_run_steel_design(&mut model).unwrap();
    let r = &model.steel_design_results["M1"];
    assert_eq!(r.eq_name, "EC3-6.3.3(4) eq 6.62");
    assert!((r.total_ratio - 0.383195).abs() < 1e-5, "{}", r.total_ratio);
    // M3 instead of M2 is designed as well
    let mut model = nmm_member(code, sec.clone(), vec![st3(-500_000.0, 100.0e6, 0.0)]);
    assert!(try_run_steel_design(&mut model).is_ok());

    // a Channel (no column in Annex B) with compression and a moment is refused by name
    let mut chan = sec.clone();
    chan.prop_type = FramePropType::Channel;
    chan.name = "CH".into();
    chan.t2 = 150.0;
    let mut model = nmm_member(code, chan, vec![st3(-500_000.0, 0.0, 10.0e6)]);
    let text = try_run_steel_design(&mut model).unwrap_err().to_string();
    assert!(
        text.contains("'M1'") && text.contains("combination 'LC'") && text.contains("Annex B"),
        "{text}"
    );
    assert!(model.steel_design_results.is_empty());

    // compression without a moment: designed
    let mut model = nmm_member(code, sec.clone(), vec![st3(-500_000.0, 0.0, 0.0)]);
    try_run_steel_design(&mut model).unwrap();
    assert!(model.steel_design_results["M1"].total_ratio > 0.0);

    // tension with a moment: designed (n = 0 in Annex B gives k <= 1, the code is conservative)
    let mut model = nmm_member(code, sec, vec![st3(500_000.0, 100.0e6, 0.0)]);
    try_run_steel_design(&mut model).unwrap();
    let r = &model.steel_design_results["M1"];
    assert!(r.total_ratio >= 500_000.0 / (10036.0 * 345.0) - 1e-9, "{}", r.total_ratio);
}

/// Negative: AISC reads the same compression and moment (H1-1a/b), it is not refused.
#[test]
fn d89_aisc_compression_with_bending_is_still_designed() {
    let sec = make_i_section("WI", "A992Fy50", 400.0, 250.0, 11.0, 12.0, 250.0, 11.0);
    let mut model = nmm_member("AISC 360-10", sec, vec![st3(-500_000.0, 0.0, 10.0e6)]);
    try_run_steel_design(&mut model).unwrap();
}

// ---------------------------------------------------------------- D90

const EC3: &str = "Eurocode 3-2005";

/// S355 (fy 355 MPa, E 210000 MPa), N and mm, member `len` mm long.
fn s355_member(sec: FrameSection, len: f64, stations: Vec<FrameStationResult>) -> SapModel {
    let mut model = one_member(Units::NMmC, EC3, sec, len, stations);
    let mat = model.materials.get_mut("A992Fy50").unwrap();
    mat.fy = 355.0;
    mat.e = 210_000.0;
    mat.g = 210_000.0 / 2.6;
    model
}

fn welded_i(tf: f64, tw: f64) -> FrameSection {
    make_i_section("WI", "A992Fy50", 400.0, 250.0, tf, tw, 250.0, tf)
}

fn i_welded(tf: f64, tw: f64) -> (f64, f64, f64, f64) {
    // area, I33, Wel, Wpl of a 400 x 250 welded I (hand): flanges 250 x tf, web 400 - 2 tf by tw
    let hw = 400.0 - 2.0 * tf;
    let a = 2.0 * 250.0 * tf + hw * tw;
    let i = (250.0 * 400.0_f64.powi(3) - (250.0 - tw) * hw.powi(3)) / 12.0;
    let wel = i / 200.0;
    let wpl = 2.0 * 250.0 * tf * (400.0 - tf) / 2.0 + tw * hw * hw / 4.0;
    (a, i, wel, wpl)
}

/// Welded I h 400, b 250, tf 11, tw 10, S355: eps = sqrt(235/355) = 0.813617;
/// flange c/tf = (250 - 10)/2/11 = 10.909: 10 eps = 8.136 < 10.909 <= 14 eps = 11.391: Class 3;
/// web in bending 378/10 = 37.8 <= 72 eps = 58.58: Class 1. I = 2.531303e8 mm^4,
/// Wel = I/200 = 1.265651e6, Wpl = 1.426960e6 mm^3. L = 500 mm (lambda_LT < 0.2, chi_LT = 1),
/// M3 = 400 kNm: ratio = M/(Wel fy) = 400e6/(1.265651e6 (355)) = 0.89026 (Class 3 takes Wel;
/// the old code used Wpl: 400e6/(1.426960e6 (355)) = 0.78962).
#[test]
fn d90_class_3_section_takes_the_elastic_modulus() {
    let (_, i, wel, wpl) = i_welded(11.0, 10.0);
    assert!((i - 2.531303e8).abs() < 2e2, "{i}");
    assert!((wel - 1.265651e6).abs() < 1.0);
    assert!((wpl - 1.426960e6).abs() < 1.0);
    let eps = (235.0_f64 / 355.0).sqrt();
    assert!((eps - 0.813617).abs() < 1e-6);
    assert!(10.0 * eps < 120.0 / 11.0 && 120.0 / 11.0 <= 14.0 * eps);
    let expected = 400.0e6 / (wel * 355.0);
    assert!((expected - 0.89026).abs() < 1e-5);

    let mut model = s355_member(welded_i(11.0, 10.0), 500.0, vec![st3(0.0, 400.0e6, 0.0)]);
    try_run_steel_design(&mut model).unwrap();
    let r = &model.steel_design_results["M1"];
    assert!(rel(r.total_ratio, expected) < 1e-6, "{} vs {expected}", r.total_ratio);
    // the old Wpl value, 0.78962, is not what comes out
    assert!((r.total_ratio - 400.0e6 / (wpl * 355.0)).abs() > 0.05);
}

/// Same section in pure compression: web 378/10 = 37.8 > 42 eps = 34.172: Class 4: refused by name.
#[test]
fn d90_class_4_web_in_compression_is_refused() {
    let mut model = s355_member(welded_i(11.0, 10.0), 3000.0, vec![st3(-500_000.0, 0.0, 0.0)]);
    let text = try_run_steel_design(&mut model).unwrap_err().to_string();
    assert!(
        text.contains("'M1'") && text.contains("'WI'") && text.contains("web") && text.contains("Class 4"),
        "{text}"
    );
    assert!(model.steel_design_results.is_empty());
}

/// tw = 12: web 378/12 = 31.5, 38 eps = 30.917 < 31.5 <= 42 eps = 34.172: Class 3, compression is
/// designed with the gross area: N_b,Rd = chi A fy with A = 2 (250)(11) + 378 (12) = 10036 mm^2,
/// minor axis (I22 = 2 (11)(250)^3/12 + 378 (12)^3/12 = 2.8700265e7) at L = 3000 mm:
/// Ncr = pi^2 E I22/L^2 = 6.6094e6 N, lambda = sqrt(A fy/Ncr) = 0.73422, curve b (alpha 0.34, the
/// existing Table 6.2 reading of the code for h/b = 1.6 > 1.2): phi = 0.86035, chi = 0.76404,
/// N_b,Rd = 2.7221e6 N (unchanged by D90).
#[test]
fn d90_class_3_web_in_compression_is_designed_with_gross_area() {
    let (a, ..) = i_welded(11.0, 12.0);
    assert!((a - 10036.0).abs() < 1e-9);
    let i22 = 2.0 * 11.0 * 250.0_f64.powi(3) / 12.0 + 378.0 * 12.0_f64.powi(3) / 12.0;
    let ncr = std::f64::consts::PI.powi(2) * 210_000.0 * i22 / 3000.0_f64.powi(2);
    let lambda = (a * 355.0 / ncr).sqrt();
    let phi = 0.5 * (1.0 + 0.34 * (lambda - 0.2) + lambda * lambda);
    let chi = 1.0 / (phi + (phi * phi - lambda * lambda).sqrt());
    let nb = chi * a * 355.0;
    assert!((nb - 2.7221e6).abs() < 2e2, "{nb}");
    let mut model = s355_member(welded_i(11.0, 12.0), 3000.0, vec![st3(-500_000.0, 0.0, 0.0)]);
    try_run_steel_design(&mut model).unwrap();
    let r = &model.steel_design_results["M1"];
    assert!(rel(r.phi_pnc, nb) < 1e-9, "{} vs {nb}", r.phi_pnc);
    assert!(rel(r.total_ratio, 500_000.0 / nb) < 1e-9);
}

/// tf = 8 (tw 10): flange c/tf = 120/8 = 15.0 > 14 eps = 11.391: Class 4 in bending: refused.
#[test]
fn d90_class_4_flange_in_bending_is_refused() {
    let mut model = s355_member(welded_i(8.0, 10.0), 500.0, vec![st3(0.0, 100.0e6, 0.0)]);
    let text = try_run_steel_design(&mut model).unwrap_err().to_string();
    assert!(text.contains("'M1'") && text.contains("flange") && text.contains("bending"), "{text}");
}

/// Pipe 323.9 x 6: d/t = 53.98: 70 eps^2 = 46.34 < 53.98 <= 90 eps^2 = 59.58: Class 3:
/// Wel = pi (D^4 - (D - 2t)^4)/(32 D) = 4.675806e5 mm^3 (not Wpl = 6.064345e5);
/// M = 100 kNm at L = 500 mm: ratio = 100e6/(4.675806e5 (355)) = 0.60244.
#[test]
fn d90_class_3_pipe_takes_the_elastic_modulus() {
    let d = 323.9_f64;
    let t = 6.0_f64;
    let di = d - 2.0 * t;
    let wel = std::f64::consts::PI * (d.powi(4) - di.powi(4)) / (32.0 * d);
    let wpl = (d.powi(3) - di.powi(3)) / 6.0;
    assert!((wel - 4.675806e5).abs() < 2.0, "{wel}");
    assert!((wpl - 6.064345e5).abs() < 2.0, "{wpl}");
    let eps2 = 235.0 / 355.0;
    assert!(70.0 * eps2 < d / t && d / t <= 90.0 * eps2);
    let pipe = make_pipe("CHS324", "A992Fy50", d, t);
    let mut model = s355_member(pipe, 500.0, vec![st3(0.0, 100.0e6, 0.0)]);
    try_run_steel_design(&mut model).unwrap();
    let r = &model.steel_design_results["M1"];
    let expected = 100.0e6 / (wel * 355.0);
    assert!(rel(r.total_ratio, expected) < 1e-6, "{} vs {expected}", r.total_ratio);
}

/// Negative: a Class 1 section keeps Wpl: IPE 300-like (flat widths, no root radius): flange
/// (150 - 7.1)/2/10.7 = 6.68 <= 9 eps = 7.32, web (300 - 21.4)/7.1 = 39.2 <= 72 eps = 58.6: Class 1.
/// Wpl = 2 (150)(10.7)(300 - 10.7)/2 + 7.1 (278.6)^2/4 = 4.6433e5 + 1.3777e5 = 6.0210e5 mm^3;
/// M = 100 kNm at L = 500 mm gives M/(Wpl fy) = 0.46785.
#[test]
fn d90_class_1_section_keeps_the_plastic_modulus() {
    let sec = make_i_section("IPE300", "A992Fy50", 300.0, 150.0, 10.7, 7.1, 150.0, 10.7);
    let hw = 300.0 - 2.0 * 10.7;
    let wpl = 2.0 * 150.0 * 10.7 * (300.0 - 10.7) / 2.0 + 7.1 * hw * hw / 4.0;
    let eps = (235.0_f64 / 355.0).sqrt();
    assert!(0.5 * (150.0 - 7.1) / 10.7 <= 9.0 * eps && hw / 7.1 <= 72.0 * eps);
    let mut model = s355_member(sec, 500.0, vec![st3(0.0, 100.0e6, 0.0)]);
    try_run_steel_design(&mut model).unwrap();
    let r = &model.steel_design_results["M1"];
    let expected = 100.0e6 / (wpl * 355.0);
    assert!(rel(r.total_ratio, expected) < 1e-6, "{} vs {expected}", r.total_ratio);
}

/// Tee stem in bending: Class 1 only up to 9 eps (the limits above need alpha and k_sigma, which the
/// manual does not give): stem c/t = (200 - 12)/8 = 23.5 > 9 eps = 7.32: refused as not classifiable;
/// the same Tee in pure tension (no bending, no compression) is designed.
#[test]
fn d90_tee_stem_in_bending_beyond_9_eps_is_refused() {
    let tee = FrameSection {
        name: "TEE".into(),
        material: "A992Fy50".into(),
        prop_type: FramePropType::T,
        t3: 200.0,
        t2: 150.0,
        tf: 12.0,
        tw: 8.0,
        t2b: 0.0,
        tfb: 0.0,
        area: 150.0 * 12.0 + 188.0 * 8.0,
        i33: 1.0e7,
        i22: 3.4e6,
        s33: 1.0e5,
        s22: 4.5e4,
        z33: 1.8e5,
        z22: 7.0e4,
        r33: 80.0,
        r22: 45.0,
        as2: 1504.0,
        as3: 1800.0,
        torsional_j: 8.0e4,
        ..FrameSection::default()
    };
    let mut model = s355_member(tee.clone(), 500.0, vec![st3(0.0, 10.0e6, 0.0)]);
    let text = try_run_steel_design(&mut model).unwrap_err().to_string();
    assert!(text.contains("stem") && text.contains("not classifiable"), "{text}");
    let mut model = s355_member(tee, 500.0, vec![st3(50_000.0, 0.0, 0.0)]);
    try_run_steel_design(&mut model).unwrap();
}

/// Angle rule of Table 4-2: Class 3 only if h/t <= 15 eps and (b + h)/(2t) <= 11.5 eps, else Class 4.
/// L100x100x10 in S355: eps = 0.81362, h/t = 10 <= 15 eps = 12.2 but (b+h)/2t = 10 > 11.5 eps = 9.356:
/// Class 4. L100x100x12: h/t = 8.33 <= 12.2 and (b+h)/2t = 8.33 <= 9.356: Class 3.
#[test]
fn d90_angle_follows_the_table_4_2_rule() {
    let eps = (235.0_f64 / 355.0).sqrt();
    assert!(10.0 > 11.5 * eps && 100.0 / 12.0 <= 11.5 * eps && 10.0 <= 15.0 * eps);
    let angle = |t: f64| FrameSection {
        prop_type: FramePropType::Angle,
        t3: 100.0,
        t2: 100.0,
        tf: t,
        tw: t,
        ..FrameSection::default()
    };
    let c = sap_design::classify_ec3_section(&angle(10.0), 355.0, false).unwrap();
    assert_eq!(c.class, 4);
    let c = sap_design::classify_ec3_section(&angle(12.0), 355.0, true).unwrap();
    assert_eq!(c.class, 3);
    // shapes outside the vendor tables are refused
    let cold = FrameSection {
        prop_type: FramePropType::ColdC,
        ..FrameSection::default()
    };
    assert!(sap_design::classify_ec3_section(&cold, 355.0, true).is_none());
}

// ---------------------------------------------------------------- D91

fn slender_flange_i(tw: f64) -> FrameSection {
    make_i_section("SLF", "A992Fy50", 12.0, 12.0, 0.2, tw, 12.0, 0.2)
}

/// User I t3 = 12, t2 = 12, tf = 0.2, tw = 0.5, Fy 50: bf/2tf = 30 > 1.0 sqrt(E/Fy) = 24.083 (slender
/// flange), h/tw = 11.6/0.5 = 23.2 < 3.76 r = 90.6 (compact web). I33 = (12 (12)^3 - 11.5 (11.6)^3)/12
/// = 232.141, S33 = I33/6 = 38.690. F3-2: kc = 4/sqrt(23.2) = 0.830 -> 0.76 (upper limit),
/// Mn = 0.9 E kc S/lambda^2 = 0.9 (29000)(0.76)(38.690)/900 = 852.73, phi Mn = 767.46 kip-in.
/// Lp = 1.76 ry sqrt(E/Fy) = 98.9 in, so a 60 in member has no LTB. M3 = 400 kip-in:
/// ratio 400/767.459 = 0.52121; the status was "SLENDER", now "OK".
#[test]
fn d91_slender_flange_alone_does_not_make_a_passing_i_section_slender() {
    let sec = slender_flange_i(0.5);
    let i33 = (12.0 * 12.0_f64.powi(3) - 11.5 * 11.6_f64.powi(3)) / 12.0;
    let s33 = i33 / 6.0;
    let mn = 0.9 * 29000.0 * 0.76 * s33 / 900.0;
    let phi_mn = 0.9 * mn;
    assert!((phi_mn - 767.459).abs() < 1e-2, "{phi_mn}");
    let mut model = one_member(Units::KipInF, "AISC 360-10", sec, 60.0, vec![st(0.0, 400.0, 0.0)]);
    try_run_steel_design(&mut model).unwrap();
    let r = &model.steel_design_results["M1"];
    assert!(rel(r.total_ratio, 400.0 / phi_mn) < 1e-4, "{}", r.total_ratio);
    assert!((r.total_ratio - 0.52121).abs() < 1e-4);
    assert_eq!(r.status, "OK");

    // and an overload of the same section is still OVERSTRESSED
    let mut model = one_member(
        Units::KipInF,
        "AISC 360-10",
        slender_flange_i(0.5),
        60.0,
        vec![st(0.0, 800.0, 0.0)],
    );
    try_run_steel_design(&mut model).unwrap();
    assert_eq!(model.steel_design_results["M1"].status, "OVERSTRESSED");
}

/// Negative: with tw = 0.08 the web h/tw = 11.6/0.08 = 145 > 5.70 r = 137.27 is slender too (F5 is
/// not built): the status stays "SLENDER" whatever the ratio.
#[test]
fn d91_slender_web_keeps_the_slender_status() {
    let mut model = one_member(
        Units::KipInF,
        "AISC 360-10",
        slender_flange_i(0.08),
        60.0,
        vec![st(0.0, 50.0, 0.0)],
    );
    try_run_steel_design(&mut model).unwrap();
    let r = &model.steel_design_results["M1"];
    assert!(r.total_ratio < 1.0);
    assert_eq!(r.status, "SLENDER");
}

/// Auto Select: the list [slender-flange I, heavier compact I] picks the lighter slender-flange I
/// (it passes: ratio 0.52 <= 0.95); with the slender-web variant the heavier is picked.
#[test]
fn d91_auto_select_takes_the_lighter_slender_flange_section() {
    for (tw, expected) in [(0.5, "SLF"), (0.08, "HEAVY")] {
        let mut m = SapModel::new(Units::KipInF);
        let light = slender_flange_i(tw);
        let heavy = make_i_section("HEAVY", "A992Fy50", 14.0, 8.0, 0.8, 0.5, 8.0, 0.8);
        assert!(heavy.area > light.area);
        m.frame_sections.insert(light.name.clone(), light);
        m.frame_sections.insert(heavy.name.clone(), heavy);
        m.define_auto_select_list("LIST", &["SLF".to_string(), "HEAVY".to_string()])
            .unwrap();
        m.add_point("A", 0.0, 0.0, 0.0);
        m.add_point("B", 60.0, 0.0, 0.0);
        m.add_frame("BM", "A", "B", "LIST");
        m.assign_auto_select_list("LIST", &["BM".to_string()]).unwrap();
        m.steel_design_code = "AISC 360-10".into();
        let mut cr = CaseResults {
            case_name: "DEAD".into(),
            ..Default::default()
        };
        cr.frame_forces.insert("BM".into(), vec![st(0.0, 400.0, 0.0)]);
        m.case_results.insert("DEAD".into(), cr);
        m.is_locked = true;
        try_run_steel_design(&mut m).unwrap();
        assert_eq!(m.steel_design_results["BM"].section_name, expected, "tw {tw}");
    }
}

// ---------------------------------------------------------------- D92

/// EC3 does not design torsion (manual 2.2.1, printed p. 2-4): T = 5 kNm at a station of an IPE 300-like
/// section with M3 = 100 kNm and no compression gives the same ratio as T = 0 (hand: Class 1,
/// M/(Wpl fy) = 100e6/(6.0210e5 (355)) = 0.46785, short member) and the result carries a note that
/// says so; T = 0 gives no note. 5 kNm is 5% of M3, far above the threshold 1e-6 M.
#[test]
fn d92_ec3_torsion_on_an_open_shape_is_noted_not_silently_dropped() {
    let sec = make_i_section("IPE300", "A992Fy50", 300.0, 150.0, 10.7, 7.1, 150.0, 10.7);
    let mut with_t = s355_member(sec.clone(), 500.0, vec![st(0.0, 100.0e6, 5.0e6)]);
    try_run_steel_design(&mut with_t).unwrap();
    let r = &with_t.steel_design_results["M1"];
    assert!((r.total_ratio - 0.46785).abs() < 1e-4, "{}", r.total_ratio);
    assert_eq!(r.notes.len(), 1);
    assert!(r.notes[0].contains("torsion") && r.notes[0].contains("not designed"), "{:?}", r.notes);

    let mut without_t = s355_member(sec, 500.0, vec![st(0.0, 100.0e6, 0.0)]);
    try_run_steel_design(&mut without_t).unwrap();
    let r0 = &without_t.steel_design_results["M1"];
    assert!(r0.notes.is_empty());
    assert_eq!(r0.total_ratio, r.total_ratio);
}

/// Negative: AISC open shapes get no note (the AISC manual ignores torsion there too, section 3.5.5)
/// and Box under EC3 keeps its refusal.
#[test]
fn d92_aisc_open_shape_has_no_note_and_ec3_box_torsion_stays_refused() {
    let sec = make_i_section("IPE300", "A992Fy50", 300.0, 150.0, 10.7, 7.1, 150.0, 10.7);
    let mut model = nmm_member("AISC 360-10", sec, vec![st(0.0, 100.0e6, 5.0e6)]);
    try_run_steel_design(&mut model).unwrap();
    assert!(model.steel_design_results["M1"].notes.is_empty());

    let bx = make_box_tube("BOX", "A992Fy50", 200.0, 100.0, 8.0, 8.0);
    let mut model = s355_member(bx, 500.0, vec![st(0.0, 10.0e6, 1.0e6)]);
    let text = try_run_steel_design(&mut model).unwrap_err().to_string();
    assert!(text.contains("torsion") && text.contains("'M1'"), "{text}");
}

// ---------------------------------------------------------------- D93

/// A cable object is not a frame member: steel design gives it no result and no error, with or
/// without forces (a cable without any force row used to stop the run with MissingDemand, one with
/// forces was designed as a beam-column). The ordinary frame of the same model is designed as
/// XX
#[test]
fn d93_cables_are_not_designed_and_frames_are_unchanged() {
    use sap_model::CableParams;
    let sec = make_i_section("W", "A992Fy50", 18.0, 7.5, 0.57, 0.355, 7.5, 0.57);
    let area = 2.0 * 7.5 * 0.57 + (18.0 - 1.14) * 0.355;
    let mut model = one_member(Units::KipInF, "AISC 360-10", sec, 120.0, vec![st(100.0, 0.0, 0.0)]);
    model.add_point("CI", 0.0, 100.0, 0.0);
    model.add_point("CJ", 0.0, 100.0, 120.0);
    model.add_frame("CABLE", "CI", "CJ", "W");
    model.frames.get_mut("CABLE").unwrap().cable_params = Some(CableParams {
        undeformed_length: None,
        target_tension_i: None,
        horizontal_tension: None,
        num_segments: 1,
    });
    // no force row for the cable at all
    try_run_steel_design(&mut model).unwrap();
    assert!(!model.steel_design_results.contains_key("CABLE"));
    let r = &model.steel_design_results["M1"];
    let expected = 0.5 * 100.0 / (0.9 * 50.0 * area); // H1-1b: Pr/(2 Pc) below Pr/Pc = 0.2
    assert!((r.total_ratio - expected).abs() < 1e-9, "{} vs {expected}", r.total_ratio);

    // with forces for the cable: still no result
    model
        .case_results
        .get_mut("LC")
        .unwrap()
        .frame_forces
        .insert("CABLE".into(), vec![st(100.0, 0.0, 0.0)]);
    try_run_steel_design(&mut model).unwrap();
    assert!(!model.steel_design_results.contains_key("CABLE"));
    assert!(model.steel_design_results.contains_key("M1"));

    // negative: the same frame without the cable mark is designed (and needs forces)
    model.frames.get_mut("CABLE").unwrap().cable_params = None;
    try_run_steel_design(&mut model).unwrap();
    assert!(model.steel_design_results.contains_key("CABLE"));
}

/// A compression of roundoff size (1e-6 N against N_b,Rd = 2.7e6 N, a ratio of 4e-13) in a beam whose
/// web is Class 4 in compression is not "compressed": the Class 4 gate and D89 use the same
/// dimensionless threshold (1e-9 of the resistance), and the beam in bending (Class 3) is designed.
#[test]
fn d90_roundoff_compression_does_not_trigger_the_class_4_gate() {
    let mut model = s355_member(welded_i(11.0, 10.0), 500.0, vec![st3(-1.0e-6, 400.0e6, 0.0)]);
    try_run_steel_design(&mut model).unwrap();
    let (_, _, wel, _) = i_welded(11.0, 10.0);
    let r = &model.steel_design_results["M1"];
    assert!(rel(r.total_ratio, 400.0e6 / (wel * 355.0)) < 1e-6, "{}", r.total_ratio);
}
