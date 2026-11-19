//! Batch S2 steel-design repairs: AISC 360-10 flexural-torsional buckling in compression (D128)
//! and flexure of box, pipe, channel, tee and double angle (D129), with the refusals G81, G82, G83.
//!
//! Every expected value is written out from the formulas of the vendor manual (printed pages are
//! quoted at each test) in kip and in; Fy = 50, E = 29000, G = E/2.6 (the A992Fy50 of
//! `SapModel::new`). The intermediates quoted in the comments come from an independent Python
//! hand calculation (a numerical sectorial integration for the section constants); the
//! assertions recompute them here and compare with those six-digit numbers at 1e-4.

use sap_design::steel::{
    channel_shear_centre_e0, channel_warping_constant, double_channel_warping_constant,
};
use sap_design::{try_run_steel_design, DesignError};
use sap_model::{
    CaseResults, FramePropType, FrameSection, FrameStationResult, SapModel, SteelDesignOverwrite,
    SteelDesignResult,
};
use sap_section::{make_box_tube, make_i_section, make_pipe};
use sap_units::Units;
use std::f64::consts::PI;

const E: f64 = 29000.0;
const FY: f64 = 50.0;

fn g() -> f64 {
    E / 2.6
}

fn rt() -> f64 {
    (E / FY).sqrt() // 24.083189
}

fn close(got: f64, want: f64, tol: f64, what: &str) {
    assert!(
        (got - want).abs() <= tol * want.abs(),
        "{what}: got {got}, expected {want}"
    );
}

/// A section from plate dimensions and the properties of the hand calculation.
#[allow(clippy::too_many_arguments)]
fn shape(
    name: &str,
    prop: FramePropType,
    dims: [f64; 5], // t3, t2, tf, tw, dis
    a: f64,
    i33: f64,
    i22: f64,
    j: f64,
    s33: f64,
    z33: f64,
    s22: f64,
    z22: f64,
) -> FrameSection {
    let channel = matches!(prop, FramePropType::Channel | FramePropType::DbChannel);
    FrameSection {
        name: name.into(),
        material: "A992Fy50".into(),
        prop_type: prop,
        t3: dims[0],
        t2: dims[1],
        tf: dims[2],
        tw: dims[3],
        dis: dims[4],
        // `FrameSection::default()` carries t2b 7.5 and tfb 0.57: a channel keeps tfb = tf (its
        // Q code reads it), the other shapes have no bottom flange.
        t2b: 0.0,
        tfb: if channel { dims[2] } else { 0.0 },
        area: a,
        i33,
        i22,
        r33: (i33 / a).sqrt(),
        r22: (i22 / a).sqrt(),
        torsional_j: j,
        s33,
        z33,
        s22,
        z22,
        as2: 0.5 * a,
        as3: 0.5 * a,
        ..FrameSection::default()
    }
}

fn station(p: f64, m3: f64, m2: f64) -> FrameStationResult {
    FrameStationResult {
        station: 0.0,
        rel_dist: 0.5,
        p,
        m3,
        m2,
        ..Default::default()
    }
}

fn member(sec: FrameSection, len: f64, stations: Vec<FrameStationResult>) -> SapModel {
    let mut model = SapModel::new(Units::KipInF);
    model.steel_design_code = "AISC 360-10".to_string();
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

fn with_overwrite(model: &mut SapModel, ow: SteelDesignOverwrite) {
    model.frames.get_mut("M1").unwrap().steel_overwrite = Some(ow);
}

fn design(model: &mut SapModel) -> SteelDesignResult {
    try_run_steel_design(model).unwrap();
    model.steel_design_results["M1"].clone()
}

fn try_design(
    sec: FrameSection,
    len: f64,
    stations: Vec<FrameStationResult>,
) -> Result<SteelDesignResult, DesignError> {
    let mut model = member(sec, len, stations);
    try_run_steel_design(&mut model)?;
    Ok(model.steel_design_results["M1"].clone())
}

fn phi_pnc(sec: FrameSection, len: f64, ow: Option<SteelDesignOverwrite>) -> f64 {
    let mut model = member(sec, len, vec![station(-1.0, 0.0, 0.0)]);
    if let Some(ow) = ow {
        with_overwrite(&mut model, ow);
    }
    design(&mut model).phi_pnc
}

fn phi_m3(sec: FrameSection, len: f64, m3: f64) -> f64 {
    design(&mut member(sec, len, vec![station(0.0, m3, 0.0)])).phi_mn33
}

fn phi_m2(sec: FrameSection, len: f64, m2: f64) -> f64 {
    design(&mut member(sec, len, vec![station(0.0, 0.0, m2)])).phi_mn22
}

// ------------------------------------------------------------------ hand formulas

/// AISC E3-2/E3-3 for a stress `fe`.
fn e3(fe: f64) -> f64 {
    if FY / fe <= 2.25 {
        0.658_f64.powf(FY / fe) * FY
    } else {
        0.877 * fe
    }
}

fn e4_5(fa: f64, fez: f64, h: f64) -> f64 {
    (fa + fez) / (2.0 * h) * (1.0 - (1.0 - 4.0 * fa * fez * h / ((fa + fez) * (fa + fez))).sqrt())
}

/// Centroid of a plate tee from the outer face of the flange: flange `bf tf`, stems `n (d - tf) tw`.
fn ybar(bf: f64, tf: f64, n: f64, d: f64, tw: f64) -> f64 {
    (bf * tf * 0.5 * tf + n * (d - tf) * tw * (tf + 0.5 * (d - tf))) / (bf * tf + n * (d - tf) * tw)
}

/// Compression of a nonslender tee or double angle, AISC E4-2, E4-3 (manual p. 3-26): returns
/// phi Pn = 0.9 A min(Fcr flexural (E3 with the larger KL/r), Fcr,ftb).
#[allow(clippy::too_many_arguments)]
fn phi_pn_e4_2(a: f64, i33: f64, i22: f64, j: f64, y0: f64, kl_r_max: f64, kl_r22: f64) -> f64 {
    let fe_flex = PI * PI * E / (kl_r_max * kl_r_max);
    let fe22 = PI * PI * E / (kl_r22 * kl_r22);
    let r0sq = y0 * y0 + (i22 + i33) / a;
    let h = 1.0 - y0 * y0 / r0sq;
    let fcrz = g() * j / (a * r0sq);
    let fcr = e3(fe_flex).min(e4_5(e3(fe22), fcrz, h));
    0.9 * a * fcr
}

// ------------------------------------------------------------------ section helpers (D128)

/// Channel shear centre from the web back, e0 = e - tw/2 with e = 3 b'^2 tf/(6 b' tf + h tw),
/// b' = bf - tw/2, h = d - tf. C10 x 2.6 x 0.436 x 0.24: b' = 2.48, h = 9.564,
/// e = 3 (6.1504)(0.436)/(6 (2.48)(0.436) + 9.564 (0.24)) = 8.04449/(6.48768 + 2.29536) = 0.915938,
/// e0 = 0.795938. The vendor's v6 library holds 0.796 for C10x15.3 (same dimensions).
/// Cw = tf b'^3 h^2/12 (3 b' tf + 2 h tw)/(6 b' tf + h tw) = 45.217773.
/// Double channel (t3 8, t2 8.375, tf 0.2, tw 0.3, dis 0.375): a = dis/2 + tw/2 = 0.3375, b' = 3.85, h = 7.8,
/// Cw = a^2 tw h^3/6 + h^2 tf [a^3 - (a - b')^3]/3 = 178.6295 (checked numerically).
#[test]
fn d128_section_helpers_match_the_hand_formulas() {
    let c10 = shape(
        "C10",
        FramePropType::Channel,
        [10.0, 2.6, 0.436, 0.24, 0.0],
        4.45792,
        67.092135,
        2.83905,
        0.185724,
        13.418427,
        15.840973,
        1.510231,
        2.5,
    );
    let (bp, h): (f64, f64) = (2.6 - 0.12, 10.0 - 0.436);
    let e = 3.0 * bp * bp * 0.436 / (6.0 * bp * 0.436 + h * 0.24);
    close(e - 0.12, 0.795938, 1e-5, "e0 by hand");
    close(channel_shear_centre_e0(&c10), e - 0.12, 1e-9, "e0");
    close(channel_shear_centre_e0(&c10), 0.7959, 1e-4, "e0 against the library 0.796");
    let cw = 0.436 * bp.powi(3) * h * h / 12.0 * (3.0 * bp * 0.436 + 2.0 * h * 0.24)
        / (6.0 * bp * 0.436 + h * 0.24);
    close(cw, 45.217773, 1e-6, "Cw by hand");
    close(channel_warping_constant(&c10), cw, 1e-9, "Cw");

    let d = shape(
        "2C8",
        FramePropType::DbChannel,
        [8.0, 8.375, 0.2, 0.3, 0.375],
        5.0,
        50.0,
        50.0,
        0.1,
        10.0,
        12.0,
        10.0,
        12.0,
    );
    let (a, bp, h): (f64, f64, f64) = (0.1875 + 0.15, 4.0 - 0.15, 7.8);
    let cw2 = a * a * 0.3 * h.powi(3) / 6.0 + h * h * 0.2 * (a.powi(3) - (a - bp).powi(3)) / 3.0;
    close(cw2, 178.6295, 1e-6, "double channel Cw by hand");
    close(double_channel_warping_constant(&d), cw2, 1e-9, "double channel Cw");
}

// ------------------------------------------------------------------ D128 compression

/// T6x8 (t3 6, t2 8, tf 0.5, tw 0.4; A 6.2, I33 18.40336, I22 21.362667, J 0.450667), L = 120 in,
/// K = 1. Stem d/t = 15 <= 0.75 r = 18.06 and flange 8 <= 0.56 r: Q = 1, so E4-2/E4-3 (p. 3-26).
/// ybar = (4 (0.25) + 2.2 (3.25))/6.2 = 1.314516, y0 = 1.064516, r0^2 = y0^2 + (I22 + I33)/A =
/// 7.547070, H = 0.849850, KL/r22 = 120/1.85623 = 64.6471, Fe22 = 68.4856, Fcr22 = 36.83496,
/// Fcrz = G J/(A r0^2) = 107.4262, Fcr,ftb = 34.401561 against the flexural 35.068738 (about 3).
/// phi Pn = 0.9 (34.401561)(6.2) = 191.960712 (flexural alone: 195.683557).
#[test]
fn d128_tee_nonslender_takes_the_e4_2_flexural_torsional_stress() {
    let sec = shape(
        "T6X8",
        FramePropType::T,
        [6.0, 8.0, 0.5, 0.4, 0.0],
        6.2,
        18.40336,
        21.362667,
        0.450667,
        3.92774,
        6.94875,
        5.340667,
        8.22,
    );
    let yb = ybar(8.0, 0.5, 1.0, 6.0, 0.4);
    close(yb, 1.314516, 1e-6, "ybar");
    let (r33, r22) = ((18.40336_f64 / 6.2).sqrt(), (21.362667_f64 / 6.2).sqrt());
    let want = phi_pn_e4_2(6.2, 18.40336, 21.362667, 0.450667, yb - 0.25, 120.0 / r22.min(r33), 120.0 / r22);
    close(want, 191.960712, 1e-6, "hand value");
    close(phi_pnc(sec, 120.0, None), want, 1e-6, "T6x8 phi Pn");
}

/// 2L4x4x1/2 (t3 4, t2 8.375, dis 0.375, tf = tw = 0.5; A 7.5, I33 11.122917, I22 25.216797,
/// J 0.625), L = 96 with the major-axis length ratio overwritten to 0.5 (L33 = 48): the flexural
/// Fcr is that of about 2 (KL/r22 = 52.35, Fe22 = 104.42, Fcr22 = 40.919485) and E4-2/E4-3 lowers
/// it: ybar = (4 (0.25) + 3.5 (2.25))/7.5 = 1.183333, y0 = 0.933333, r0^2 = 5.716406, H = 0.847612,
/// Fcrz = 162.599917, Fcr,ftb = 39.039792; phi Pn = 0.9 (39.039792)(7.5) = 263.518593 (today
/// 276.206526). Without the overwrite the flexural Fcr about 3 (31.742544) governs and the result is
/// the old 214.262170, unchanged: the negative case.
#[test]
fn d128_double_angle_nonslender_and_the_case_where_flexural_governs() {
    let sec = shape(
        "2L4",
        FramePropType::DblAngle,
        [4.0, 8.375, 0.5, 0.5, 0.375],
        7.5,
        11.122917,
        25.216797,
        0.625,
        4.0,
        7.0,
        6.0,
        9.0,
    );
    let yb = ybar(8.0, 0.5, 2.0, 4.0, 0.5);
    close(yb, 1.183333, 1e-6, "ybar");
    let (r33, r22) = ((11.122917_f64 / 7.5).sqrt(), (25.216797_f64 / 7.5).sqrt());
    let ow = SteelDesignOverwrite {
        l_ratio_major: Some(0.5),
        ..Default::default()
    };
    let want = phi_pn_e4_2(7.5, 11.122917, 25.216797, 0.625, yb - 0.25, 96.0 / r22, 96.0 / r22);
    close(want, 263.518593, 1e-6, "hand value, overwrite");
    close(phi_pnc(sec.clone(), 96.0, Some(ow)), want, 1e-6, "2L4 with L33 = 48");
    // no overwrite: KL/r33 = 96/r33 governs the flexural Fcr (31.742544), below Fcr,ftb
    let fcr_flex = e3(PI * PI * E / (96.0 / r33).powi(2));
    close(fcr_flex, 31.742544, 1e-6, "flexural Fcr about 3");
    close(phi_pnc(sec, 96.0, None), 0.9 * 7.5 * fcr_flex, 1e-9, "2L4 flexural governs");
    close(0.9 * 7.5 * fcr_flex, 214.262170, 1e-6, "the unchanged value");
}

/// C10x20 plates (t3 10, t2 2.739, tf 0.436, tw 0.379; A 5.84792, I33 78.675469, I22 3.501957,
/// J 0.316985), L = 240 with the minor ratio 0.2 (L22 = Lz = 48): E4-5 with the symmetry axis 3
/// (p. 3-27, 3-30). b' = 2.5495, h = 9.564, e = 0.825892, xbar = 0.671436, x0 = xbar + e - tw/2 =
/// 1.307827, Cw = 56.625913, r0^2 = 15.762832, H = 0.891491, Fe33 = 66.851811, Fez = 114.667832,
/// Fe,ftb = 59.785105 (flexural 66.851811, KL33/r33 = 65.43 > KL22/r22 = 62.03); Q = 1;
/// Fcr = 35.232795 (flexural 36.560909); phi Pn = 185.434711 (today 192.424745).
#[test]
fn d128_channel_takes_e4_5_with_its_own_shear_centre_and_warping_constant() {
    let sec = shape(
        "C10X20",
        FramePropType::Channel,
        [10.0, 2.739, 0.436, 0.379, 0.0],
        5.84792,
        78.675469,
        3.501957,
        0.316985,
        15.735094,
        19.315973,
        2.0,
        3.5,
    );
    let (bp, h): (f64, f64) = (2.739 - 0.1895, 10.0 - 0.436);
    let e = 3.0 * bp * bp * 0.436 / (6.0 * bp * 0.436 + h * 0.379);
    let fl: f64 = 2.739 - 0.379;
    let xbar = (10.0 * 0.379 * 0.1895 + 2.0 * fl * 0.436 * (0.379 + 0.5 * fl)) / (10.0 * 0.379 + 2.0 * fl * 0.436);
    let x0 = xbar + e - 0.1895;
    let cw = 0.436 * bp.powi(3) * h * h / 12.0 * (3.0 * bp * 0.436 + 2.0 * h * 0.379)
        / (6.0 * bp * 0.436 + h * 0.379);
    close(e, 0.825892, 1e-5, "e");
    close(xbar, 0.671436, 1e-5, "xbar (plates, G86)");
    close(cw, 56.625913, 1e-6, "Cw");
    let a = 5.84792;
    let r0sq = x0 * x0 + (3.501957 + 78.675469) / a;
    let hh = 1.0 - x0 * x0 / r0sq;
    let (r33, r22) = ((78.675469_f64 / a).sqrt(), (3.501957_f64 / a).sqrt());
    let kl33 = 240.0 / r33;
    let kl22 = 48.0 / r22;
    let fe33 = PI * PI * E / (kl33 * kl33);
    let fez = (PI * PI * E * cw / (48.0 * 48.0) + g() * 0.316985) / (a * r0sq);
    close(fez, 114.667832, 1e-5, "Fez");
    let fe = (PI * PI * E / kl33.max(kl22).powi(2)).min(e4_5(fe33, fez, hh));
    close(fe, 59.785105, 1e-5, "Fe,ftb");
    let want = 0.9 * a * e3(fe);
    close(want, 185.434711, 1e-6, "hand value");
    let ow = SteelDesignOverwrite {
        l_ratio_minor: Some(0.2),
        ..Default::default()
    };
    close(phi_pnc(sec, 240.0, Some(ow)), want, 1e-6, "C10x20 phi Pn");
}

/// 2C10x20 back to back (t2 5.853, dis 0.375; A 11.69584, I33 157.350937, I22 15.632761,
/// J 0.63397), L = 24: E4-4 with the double channel's own Cw (p. 3-26, 3-30): a = dis/2 + tw/2 =
/// 0.377, b' = 2.5495, h = 9.564, Cw = a^2 tw h^3/6 + h^2 tf [a^3 - (a - b')^3]/3 = 144.875028,
/// Fez = (pi^2 E Cw/L^2 + G J)/(I22 + I33) = 457.040988 < the flexural 664.170419 (r22 =
/// 1.156118); Fcr = 47.762168, phi Pn = 502.756805 (today 509.987622).
#[test]
fn d128_double_channel_takes_e4_4() {
    let sec = shape(
        "2C10X20",
        FramePropType::DbChannel,
        [10.0, 5.853, 0.436, 0.379, 0.375],
        11.69584,
        157.350937,
        15.632761,
        0.63397,
        31.470187,
        38.631946,
        10.0,
        12.0,
    );
    let (a_w, bp, h): (f64, f64, f64) = (0.1875 + 0.1895, (5.853 - 0.375) / 2.0 - 0.1895, 9.564);
    let cw = a_w * a_w * 0.379 * h.powi(3) / 6.0 + h * h * 0.436 * (a_w.powi(3) - (a_w - bp).powi(3)) / 3.0;
    close(cw, 144.875028, 1e-6, "Cw");
    let fez = (PI * PI * E * cw / (24.0 * 24.0) + g() * 0.63397) / (15.632761 + 157.350937);
    close(fez, 457.040988, 1e-5, "Fez");
    let r22 = (15.632761_f64 / 11.69584).sqrt();
    let fe_flex = PI * PI * E / (24.0 / r22).powi(2);
    close(fe_flex, 664.170419, 1e-5, "flexural Fe");
    let want = 0.9 * 11.69584 * e3(fe_flex.min(fez));
    close(want, 502.756805, 1e-6, "hand value");
    close(phi_pnc(sec, 24.0, None), want, 1e-6, "2C10x20 phi Pn");
}

/// Negative cases of D128 and G81: a single angle under compression is refused naming the member
/// and "single angle" (it was designed twice unconservatively: geometric r, no flexural-torsional
/// buckling); the same angle in tension alone is still designed (phi Pnt = 0.9 (50) A = 348.75);
/// a tee with J = 0 under compression is refused; a roundoff compression (1e-12 of Fy A) neither
/// refuses nor designs.
#[test]
fn d128_single_angle_and_tee_without_j_are_refused_by_name() {
    let angle = shape(
        "L8X8",
        FramePropType::Angle,
        [8.0, 8.0, 0.5, 0.5, 0.0],
        7.75,
        48.4375,
        48.4375,
        0.6,
        8.0,
        14.0,
        8.0,
        14.0,
    );
    let err = try_design(angle.clone(), 12.0, vec![station(-1.0, 0.0, 0.0)]).unwrap_err();
    let msg = err.to_string();
    assert!(matches!(err, DesignError::AiscLimitStateNotBuilt { .. }), "{msg}");
    assert!(msg.contains("'M1'") && msg.contains("single angle") && msg.contains("G81"), "{msg}");
    // tension only: designed
    let r = try_design(angle.clone(), 12.0, vec![station(10.0, 0.0, 0.0)]).unwrap();
    close(r.phi_pnt, 0.9 * 50.0 * 7.75, 1e-12, "angle phi Pnt");
    // bending refused too
    assert!(try_design(angle.clone(), 12.0, vec![station(0.0, 5.0, 0.0)]).is_err());
    assert!(try_design(angle.clone(), 12.0, vec![station(0.0, 0.0, 5.0)]).is_err());
    // roundoff compression: designed
    assert!(try_design(angle, 12.0, vec![station(-1e-12, 0.0, 0.0)]).is_ok());

    let tee = shape(
        "T6X8",
        FramePropType::T,
        [6.0, 8.0, 0.5, 0.4, 0.0],
        6.2,
        18.40336,
        21.362667,
        0.0,
        3.92774,
        6.94875,
        5.340667,
        8.22,
    );
    let err = try_design(tee.clone(), 120.0, vec![station(-1.0, 0.0, 0.0)]).unwrap_err();
    let msg = err.to_string();
    assert!(matches!(err, DesignError::AiscLimitStateNotBuilt { .. }));
    assert!(msg.contains("J = 0"), "{msg}");
    // tension with J = 0 is designed
    assert!(try_design(tee, 120.0, vec![station(5.0, 0.0, 0.0)]).is_ok());
}

// ------------------------------------------------------------------ D129 pipe and box

/// Pipe 20 x 0.25: D/t = 80 lies between 0.07 E/Fy = 40.6 and 0.31 E/Fy = 179.8 (noncompact, F8-2,
/// p. 3-58): Mn = (0.021 E/(D/t) + Fy) S = (7.6125 + 50) S, Mp = Fy Z = 4876.04 (Z 97.520833);
/// S = 75.643354, Mn = 4358.002722, phi Mn = 3922.202450 for M3 and M2 alike (today phi Mp = 4388.4375).
/// Pipe 30 x 0.12: D/t = 250 > 0.31 E/Fy (slender): Fcr = 0.33 E/(D/t) = 38.28, S = 83.810543,
/// phi Mn = 0.9 (38.28)(83.810543) = 2887.440843 (today 4821.22368); M3 = 2000 gives the ratio
/// 0.692655 and "OK" (today 0.414833 and "SLENDER"). The compact HSS10x0.5 keeps phi Mp = 2032.5.
#[test]
fn d129_pipe_noncompact_and_slender() {
    let p = make_pipe("P20X0.25", "A992Fy50", 20.0, 0.25);
    let lambda = 80.0;
    let mn = (0.021 * E / lambda + FY) * p.s33;
    close(p.s33, 75.643354, 1e-6, "S");
    close(0.9 * mn, 3922.202450, 1e-6, "hand value");
    close(phi_m3(p.clone(), 120.0, 1000.0), 0.9 * mn, 1e-9, "pipe M3");
    close(phi_m2(p, 120.0, 1000.0), 0.9 * mn, 1e-9, "pipe M2");

    let p = make_pipe("P30X0.12", "A992Fy50", 30.0, 0.12);
    let fcr = 0.33 * E / 250.0;
    close(fcr, 38.28, 1e-9, "Fcr");
    let want = 0.9 * fcr * p.s33;
    close(want, 2887.440843, 1e-6, "hand value");
    let r = design(&mut member(p.clone(), 120.0, vec![station(0.0, 2000.0, 0.0)]));
    close(r.phi_mn33, want, 1e-9, "slender pipe phi Mn");
    close(r.total_ratio, 2000.0 / want, 1e-9, "ratio");
    close(r.total_ratio, 0.692655, 1e-5, "ratio against the hand number");
    assert_eq!(r.status, "OK");
    close(phi_m2(p, 120.0, 1000.0), want, 1e-9, "slender pipe M2");

    // negative: the compact pipe is unchanged, phi Mp = 0.9 Fy Z
    let p = make_pipe("HSS10X0.5", "A992Fy50", 10.0, 0.5);
    let z = (1000.0 - 729.0) / 6.0;
    close(phi_m3(p, 120.0, 100.0), 0.9 * 50.0 * z, 1e-9, "compact pipe");
}

/// Box F7 (pp. 3-56 to 3-58). Box 8 x 10 x 5/16 about 3: flange b = t2 - 3tw = 9.0625, b/t = 29 is
/// noncompact (1.12 r = 26.97, 1.40 r = 33.72): Mn = Mp - (Mp - Fy S)(3.57 (b/t)/r - 4.0), factor
/// 0.298849, S33 28.320796, Mp = Fy Z = 1626.098633: Mn = 1563.322701, phi Mn = 1406.990430 (today
/// phi Mp = 1463.48877); the web h/t = 22.6 is compact. About 2 the flange is the 8 in wall
/// (22.6, compact): phi Mp22 = 1707.824707 unchanged. Box 10 x 8 about 2: the same element by symmetry.
#[test]
fn d129_box_flange_local_buckling_both_axes() {
    let b = make_box_tube("B8X10", "A992Fy50", 8.0, 10.0, 0.3125, 0.3125);
    let lam = (10.0 - 3.0 * 0.3125) / 0.3125;
    close(lam, 29.0, 1e-12, "b/t");
    let factor = 3.57 * lam / rt() - 4.0;
    close(factor, 0.298849, 1e-5, "factor");
    let mp = FY * b.z33;
    let mn = mp - (mp - FY * b.s33) * factor;
    close(mn, 1563.322701, 1e-6, "Mn");
    close(phi_m3(b.clone(), 120.0, 500.0), 0.9 * mn, 1e-9, "box FLB, M3");
    close(phi_m2(b.clone(), 120.0, 500.0), 0.9 * FY * b.z22, 1e-9, "box minor compact");
    close(0.9 * FY * b.z22, 1707.824707, 1e-6, "compact value");
    let b2 = make_box_tube("B10X8", "A992Fy50", 10.0, 8.0, 0.3125, 0.3125);
    close(phi_m2(b2, 120.0, 500.0), 0.9 * mn, 1e-9, "box minor, noncompact flange");
}

/// Box web: 20 x 6 x 1/4, h/t = (20 - 0.75)/0.25 = 77 > 2.42 r = 58.28 (noncompact, 5.70 r =
/// 137.3): Mn = Mp - (Mp - Fy S)(0.305 (h/t)/r - 0.738), factor 0.237162, Mn = 3656.1696,
/// phi Mn = 3290.55264 (today 3472.03125). Box 30 x 5 x 3/16: h/t = 157 > 5.70 r, slender web:
/// the vendor's note (pp. 3-57/58) gives the same formula, factor 1.250316, phi Mn = 3368.757339
/// (today 4960.26123) and the status is by ratio ("OK"), not "SLENDER".
#[test]
fn d129_box_web_local_buckling_including_the_vendor_slender_note() {
    let b = make_box_tube("B20X6", "A992Fy50", 20.0, 6.0, 0.25, 0.25);
    let factor = 0.305 * 77.0 / rt() - 0.738;
    close(factor, 0.237162, 1e-5, "factor");
    let mp = FY * b.z33;
    let mn = mp - (mp - FY * b.s33) * factor;
    close(mn, 3656.1696, 1e-6, "Mn");
    close(phi_m3(b, 120.0, 1000.0), 0.9 * mn, 1e-9, "web noncompact");

    let b = make_box_tube("B30X5", "A992Fy50", 30.0, 5.0, 0.1875, 0.1875);
    let factor = 0.305 * 157.0 / rt() - 0.738;
    close(factor, 1.250316, 1e-5, "factor");
    let mp = FY * b.z33;
    let mn = mp - (mp - FY * b.s33) * factor;
    let r = design(&mut member(b, 120.0, vec![station(0.0, 1000.0, 0.0)]));
    close(r.phi_mn33, 0.9 * mn, 1e-9, "web slender, vendor note");
    close(r.phi_mn33, 3368.757339, 1e-6, "hand value");
    assert_eq!(r.status, "OK");
}

/// G82: a slender box flange (12 x 12 x 3/16, b/t = 61 > 1.40 r = 33.72) with a moment is refused
/// by name; compression alone keeps its D86 Q (phi Pn = 261.290 as before).
#[test]
fn d129_box_slender_flange_is_refused_with_a_moment_only() {
    let b = make_box_tube("B12", "A992Fy50", 12.0, 12.0, 0.1875, 0.1875);
    for st in [station(0.0, 100.0, 0.0), station(0.0, 0.0, 100.0)] {
        let err = try_design(b.clone(), 120.0, vec![st]).unwrap_err();
        let msg = err.to_string();
        assert!(matches!(err, DesignError::AiscLimitStateNotBuilt { .. }));
        assert!(msg.contains("'M1'") && msg.contains("slender flange") && msg.contains("G82"), "{msg}");
    }
    let got = phi_pnc(b, 12.0, None);
    close(got, 261.290, 1e-4, "D86 value");
}

// ------------------------------------------------------------------ D129 tee and double angle

fn t8x4() -> FrameSection {
    shape(
        "T8X4",
        FramePropType::T,
        [8.0, 4.0, 0.5, 0.35, 0.0],
        4.625,
        30.508516,
        2.693464,
        0.273854,
        5.567522,
        10.064732,
        1.346732,
        2.1,
    )
}

/// F9-4/F9-5 (p. 3-59): Mcr = pi sqrt(E Iy G J)/Lb [B + sqrt(1 + B^2)], B = +-2.3 (d/Lb) sqrt(Iy/J),
/// plus for the stem in tension (M3 > 0), minus for the stem in compression. T8x4 (t3 8, t2 4,
/// tf 0.5, tw 0.35; A 4.625, I33 30.508516, I22 2.693464, J 0.273854, S33 5.567522, Z33 10.064732),
/// Lb = 240: B = 0.240438, Mcr+ = 256.569561, Mcr- (B = -0.240438) = 159.340154.
/// Stem in compression: My = Fy S33 = 278.376105 = Mp-; stem local buckling F9-9..F9-11 (v24 p. 3-60;
/// d/tw = 22.857 lies between 0.84 r = 20.23 and 1.03 r = 24.81): Fcr = (2.55 - 1.84 (d/tw)/r) Fy =
/// 40.183610, Mn = Fcr S33 = 223.723135; Mn- = 159.340154, phi Mn- = 143.406139 (today 400.861591).
/// Stem in tension: Mp+ = min(Fy Z33, 1.6 My) = 445.401767 (flange lambda = 4 compact), Mn+ = Mcr+,
/// phi Mn+ = 230.912605. The page does not say which moment decides the sign when the moment
/// changes sign inside the unbraced length: with the stem compressed at any station of the state,
/// every station takes the minus sign (the safe side). The positive station then has
/// min(Mp+, Mcr-) = 159.340154, phi Mn = 143.406139.
#[test]
fn d129_tee_ltb_takes_the_sign_of_the_moment() {
    let t = t8x4();
    let b = 2.3 * (8.0 / 240.0) * (2.693464_f64 / 0.273854).sqrt();
    close(b, 0.240438, 1e-5, "B");
    let k = PI * (E * 2.693464 * g() * 0.273854).sqrt() / 240.0;
    let mcr_pos = k * (b + (1.0 + b * b).sqrt());
    let mcr_neg = k * (-b + (1.0 + b * b).sqrt());
    close(mcr_pos, 256.569561, 1e-5, "Mcr+");
    close(mcr_neg, 159.340154, 1e-5, "Mcr-");
    let my = FY * 5.567522;
    let dt = 8.0 / 0.35;
    let fcr_stem = (2.55 - 1.84 * dt / rt()) * FY;
    close(fcr_stem, 40.183610, 1e-5, "stem Fcr");
    let mn_neg = my.min(mcr_neg).min(fcr_stem * 5.567522);
    let mn_pos = (FY * 10.064732).min(1.6 * my).min(mcr_pos);
    close(0.9 * mn_neg, 143.406139, 1e-5, "negative hand value");
    close(0.9 * mn_pos, 230.912605, 1e-5, "positive hand value");
    close(phi_m3(t.clone(), 240.0, -50.0), 0.9 * mn_neg, 1e-9, "T8x4 stem compressed");
    close(phi_m3(t.clone(), 240.0, 50.0), 0.9 * mn_pos, 1e-9, "T8x4 stem in tension");
    // mixed signs on one member: the negative station governs, with its own strength
    let r = design(&mut member(
        t,
        240.0,
        vec![station(0.0, 50.0, 0.0), station(0.0, -50.0, 0.0)],
    ));
    close(r.phi_mn33, 0.9 * mn_neg, 1e-9, "mixed signs, governing station");
    close(r.total_ratio, 50.0 / (0.9 * mn_neg), 1e-9, "mixed signs, ratio");
    // +50 and -10: the stem is compressed at one station, so the positive station takes the
    // lateral-torsional value with the minus sign: min(Mp+ 445.401767, Mcr- 159.340154) =
    // 159.340154, phi Mn = 143.406139 (230.912605 with its own sign, which the page does not
    // promise). 50/143.406139 = 0.34866 governs against 10/143.406139.
    let mn_pos_rev = (FY * 10.064732).min(1.6 * my).min(mcr_neg);
    let r = design(&mut member(
        t8x4(),
        240.0,
        vec![station(0.0, 50.0, 0.0), station(0.0, -10.0, 0.0)],
    ));
    close(r.phi_mn33, 0.9 * mn_pos_rev, 1e-9, "mixed signs, positive station");
    close(r.phi_mn33, 143.406139, 1e-5, "mixed signs, hand value");
    close(r.total_ratio, 50.0 / (0.9 * mn_pos_rev), 1e-9, "mixed signs, positive ratio");
    // a roundoff negative moment does not count as a compressed stem
    let r = design(&mut member(
        t8x4(),
        240.0,
        vec![station(0.0, 50.0, 0.0), station(0.0, -1e-12, 0.0)],
    ));
    close(r.phi_mn33, 0.9 * mn_pos, 1e-9, "roundoff negative moment");
}

/// A double angle whose thickness is set in only one of `tf` and `tw` is read as the existing angle
/// code reads it (the one that is set), not refused and not divided by zero: with tw = 0 the result
/// is the same as with tw = tf = 0.5.
#[test]
fn d128_double_angle_with_one_thickness_zero_is_read_like_the_angle() {
    let full = shape(
        "2L4",
        FramePropType::DblAngle,
        [4.0, 8.375, 0.5, 0.5, 0.375],
        7.5,
        11.122917,
        25.216797,
        0.625,
        4.0,
        7.0,
        6.0,
        9.0,
    );
    let mut half = full.clone();
    half.tw = 0.0;
    let a = phi_pnc(full.clone(), 96.0, None);
    let b = phi_pnc(half.clone(), 96.0, None);
    assert_eq!(a.to_bits(), b.to_bits());
    let m_full = phi_m3(full, 60.0, 20.0);
    let m_half = phi_m3(half, 60.0, 20.0);
    assert_eq!(m_full.to_bits(), m_half.to_bits());
}

/// Stem local buckling middle branch (T7x12: t3 7, t2 12, tf 0.5, tw 0.32; A 8.08, I33 26.369125,
/// I22 72.017749, J 0.570997, S33 4.508306, Z33 7.939867, S22 12.002958, Z22 18.1664), Lb = 60,
/// M3 < 0: d/tw = 21.875, Fcr = (2.55 - 1.84 (21.875)/24.0832) 50 = 43.935486, Mn = 198.074607 (My =
/// 225.415291, LTB 975.774257), phi Mn = 178.267146 (today 324.598019). The v19 page's middle branch
/// Fy[1.19 - 0.50 (b/t) sqrt(Fy/E)] would give a different, discontinuous value. M3 > 0: flange lambda
/// = 12 (noncompact, 0.38 r = 9.15, r = 24.08), FLB F9-6 gives Mn = 360.664466 (the 1.6 My cap),
/// phi Mn = 324.598019, unchanged. Minor axis F6: lambda 12, Mp22 = Fy Z22 = 908.32,
/// Mn22 = Mp22 - (Mp22 - 0.7 Fy S22)(12 - 9.1516)/(24.0832 - 9.1516) = 815.186505, phi = 733.667855
/// (today 817.488).
#[test]
fn d129_tee_stem_middle_branch_flange_and_minor_axis() {
    let t = shape(
        "T7X12",
        FramePropType::T,
        [7.0, 12.0, 0.5, 0.32, 0.0],
        8.08,
        26.369125,
        72.017749,
        0.570997,
        4.508306,
        7.939867,
        12.002958,
        18.1664,
    );
    let dt = 7.0 / 0.32;
    let fcr = (2.55 - 1.84 * dt / rt()) * FY;
    close(fcr, 43.935486, 1e-6, "Fcr");
    let mn = fcr * 4.508306;
    close(mn, 198.074607, 1e-6, "Mn");
    close(phi_m3(t.clone(), 60.0, -50.0), 0.9 * mn, 1e-9, "T7x12 negative");
    close(phi_m3(t.clone(), 60.0, 50.0), 0.9 * 1.6 * FY * 4.508306, 1e-9, "T7x12 positive");
    close(0.9 * 1.6 * FY * 4.508306, 324.598019, 1e-6, "unchanged");
    let (lp, lr) = (0.38 * rt(), rt());
    let mp22 = FY * 18.1664;
    let mn22 = mp22 - (mp22 - 0.7 * FY * 12.002958) * (12.0 - lp) / (lr - lp);
    close(mn22, 815.186505, 1e-6, "Mn22");
    close(phi_m2(t, 60.0, 50.0), 0.9 * mn22, 1e-9, "T7x12 minor");
}

/// Stem slender (T9x16: t3 9, t2 16, tf 0.3, tw 0.3; A 7.41, I33 50.735012, I22 102.419575, J 0.2223,
/// S33 6.983504, Z33 11.998561), Lb = 60, M3 < 0: d/tw = 30 > 1.03 r, Fcr = 0.69 E/(d/tw)^2 =
/// 22.233333, Mn = 155.266563, phi Mn = 139.739907 (today 502.812259). Flange slender (T6x25: t3 6,
/// t2 25, tf 0.25, tw 1.0; A 12, I33 42.828125, I22 326, J 2.046875, S33 9.651408, Z33 17.31), M3 > 0:
/// ybar = 1.5625, Sxc = I33/ybar = 27.41, lambda = 50 > r, F9-7 gives 0.7 E Sxc/lambda^2 = 222.5692,
/// phi Mn = 200.312280 (today 694.901408).
#[test]
fn d129_tee_stem_slender_and_flange_slender() {
    let t = shape(
        "T9X16",
        FramePropType::T,
        [9.0, 16.0, 0.3, 0.3, 0.0],
        7.41,
        50.735012,
        102.419575,
        0.2223,
        6.983504,
        11.998561,
        12.8,
        19.0,
    );
    let fcr = 0.69 * E / 900.0;
    close(fcr, 22.233333, 1e-6, "Fcr");
    close(phi_m3(t, 60.0, -30.0), 0.9 * fcr * 6.983504, 1e-9, "T9x16 stem slender");
    close(0.9 * fcr * 6.983504, 139.739907, 1e-6, "hand value");

    let t = shape(
        "T6X25",
        FramePropType::T,
        [6.0, 25.0, 0.25, 1.0, 0.0],
        12.0,
        42.828125,
        326.0,
        2.046875,
        9.651408,
        17.31,
        26.0,
        40.0,
    );
    let yb = ybar(25.0, 0.25, 1.0, 6.0, 1.0);
    close(yb, 1.5625, 1e-9, "ybar");
    let sxc = 42.828125 / yb;
    close(sxc, 27.41, 1e-5, "Sxc");
    let mn = 0.7 * E * sxc / 2500.0;
    close(mn, 222.5692, 1e-6, "Mn");
    close(phi_m3(t, 60.0, 30.0), 0.9 * mn, 1e-9, "T6x25 flange slender");
    close(0.9 * mn, 200.31228, 1e-6, "hand value");
}

/// Double angle (t3 4, t2 12.375, dis 0.375, tf 0.3125, tw 0.75; A 9.28125, I33 15.237581, I22 51.359985,
/// J 1.15918, S33 5.746061, Z33 10.255371), Lb = 60, M3 > 0: horizontal legs b1 = 6, lambda = 19.2
/// between 0.54 r = 13.005 and 0.91 r = 21.916 (Table 3-1 case 12); ybar = 1.348169, Sxc = I33/ybar =
/// 11.302425, Mp+ = min(Fy Z33, 1.6 My) = 459.684868, My = 287.303043: Mn = Mp+ - (Mp+ - 0.7 Fy Sxc)
/// (lambda - 0.54 r)/(0.37 r) = 415.120369 (<= 1.6 My), phi Mn = 373.608332 (today 413.716381).
#[test]
fn d129_double_angle_flange_local_buckling_noncompact() {
    let d = shape(
        "2L6X4",
        FramePropType::DblAngle,
        [4.0, 12.375, 0.3125, 0.75, 0.375],
        9.28125,
        15.237581,
        51.359985,
        1.15918,
        5.746061,
        10.255371,
        8.0,
        12.0,
    );
    let yb = ybar(12.0, 0.3125, 2.0, 4.0, 0.75);
    close(yb, 1.348169, 1e-6, "ybar");
    let sxc = 15.237581 / yb;
    let my = FY * 5.746061;
    let mp = (FY * 10.255371).min(1.6 * my);
    let (lp, lr) = (0.54 * rt(), 0.91 * rt());
    let mn = mp - (mp - 0.7 * FY * sxc) * (19.2 - lp) / (lr - lp);
    close(mn, 415.120369, 1e-6, "Mn");
    close(phi_m3(d, 60.0, 50.0), 0.9 * mn, 1e-9, "double angle FLB");
    close(0.9 * mn, 373.608332, 1e-6, "hand value");
}

// ------------------------------------------------------------------ D129 channel

fn c10() -> FrameSection {
    shape(
        "C10",
        FramePropType::Channel,
        [10.0, 2.6, 0.436, 0.24, 0.0],
        4.45792,
        67.092135,
        2.83905,
        0.185724,
        13.418427,
        15.840973,
        1.510231,
        2.5,
    )
}

/// Channel LTB with its own Cw and c (F2-8b: c = (h0/2) sqrt(Iy/Cw)). C10 plates (t3 10, t2 2.6,
/// tf 0.436, tw 0.24; A 4.45792, I33 67.092135, I22 2.83905, J 0.185724, S33 13.418427, Z33 15.840973),
/// Lb = 120, Cb = 1: h0 = 9.564, Cw = 45.217773, c = 1.198234, rts = 0.918903, Lp = 33.82572,
/// Lr = 108.283007; Lb > Lr: Mn = Fcr S33 with Fcr = pi^2 E/(Lb/rts)^2 sqrt(1 + 0.078 (J c/(S h0))(Lb/rts)^2)
/// = 409.516795 / S33; phi Mn = 368.565115 (today, with Cw = 0.25 Iy h0^2 and c = 1, 392.099125).
#[test]
fn d129_channel_ltb_uses_the_channels_own_warping_constant() {
    let sec = c10();
    let h0 = 9.564;
    // Cw written out: tf b'^3 h^2/12 (3 b' tf + 2 h tw)/(6 b' tf + h tw), b' = 2.6 - 0.12
    let bp: f64 = 2.6 - 0.12;
    let cw = 0.436 * bp.powi(3) * h0 * h0 / 12.0 * (3.0 * bp * 0.436 + 2.0 * h0 * 0.24)
        / (6.0 * bp * 0.436 + h0 * 0.24);
    close(cw, 45.217773, 1e-6, "Cw");
    let c = 0.5 * h0 * (2.83905 / cw).sqrt();
    close(c, 1.198234, 1e-5, "c");
    let rts = ((2.83905 * cw).sqrt() / 13.418427).sqrt();
    close(rts, 0.918903, 1e-5, "rts");
    let jc = 0.185724 * c / (13.418427 * h0);
    let lp = 1.76 * (2.83905_f64 / 4.45792).sqrt() * rt();
    let lr = 1.95 * rts * E / (0.7 * FY) * (jc + (jc * jc + 6.76 * (0.7 * FY / E).powi(2)).sqrt()).sqrt();
    close(lr, 108.283007, 1e-5, "Lr");
    assert!(120.0 > lr && lp < lr);
    let lb_rts = 120.0 / rts;
    let fcr = PI * PI * E / (lb_rts * lb_rts) * (1.0 + 0.078 * jc * lb_rts * lb_rts).sqrt();
    let mn = (fcr * 13.418427).min(FY * 15.840973);
    close(mn, 409.516795, 1e-6, "Mn");
    close(phi_m3(sec, 120.0, 100.0), 0.9 * mn, 1e-9, "C10 LTB");
    close(0.9 * mn, 368.565115, 1e-6, "hand value");
}

/// Channel flange local buckling and minor axis (the d86 C8: t3 8, t2 4, tf 0.2, tw 0.3; A 3.88,
/// I33 34.92, I22 6.0625, J 0.05, S33 8, Z33 9, S22 2, Z22 3), L = 12: lambda = bf/tf = 20 between 0.38 r =
/// 9.15 and r = 24.08, web h/tw = 25.3 compact: F3-1 Mn = Mp - (Mp - 0.7 Fy S33)(lambda - lp)/(lr - lp) =
/// 450 - 170 (10.8484/14.9316) = 326.4882, phi Mn = 293.83938 (today 405). F6 minor axis: Mn22 =
/// 150 - (150 - 70)(0.726...) = 91.8768, phi = 82.68912 (today 135). The short double channel and the
/// compact flange of C10 are unchanged.
#[test]
fn d129_channel_flange_local_buckling_and_minor_axis() {
    let sec = shape(
        "C8",
        FramePropType::Channel,
        [8.0, 4.0, 0.2, 0.3, 0.0],
        3.88,
        34.92,
        6.0625,
        0.05,
        8.0,
        9.0,
        2.0,
        3.0,
    );
    let (lp, lr) = (0.38 * rt(), rt());
    let mn = 450.0 - (450.0 - 0.7 * 50.0 * 8.0) * (20.0 - lp) / (lr - lp);
    close(mn, 326.4882, 1e-6, "Mn");
    close(phi_m3(sec.clone(), 12.0, 100.0), 0.9 * mn, 1e-9, "C8 FLB");
    let mn22 = 150.0 - (150.0 - 0.7 * 50.0 * 2.0) * (20.0 - lp) / (lr - lp);
    close(mn22, 91.8768, 1e-6, "Mn22");
    close(phi_m2(sec, 12.0, 20.0), 0.9 * mn22, 1e-9, "C8 minor");
    // compact flange: C10, short length: phi Mp
    close(phi_m3(c10(), 12.0, 100.0), 0.9 * FY * 15.840973, 1e-9, "C10 short");
}

/// Double channel LTB (2C10, t2 5.575, dis 0.375; A 8.91584, I33 134.18427, I22 13.022771, J 0.371447,
/// S33 26.836854, Z33 31.681946), Lb = 240, Cb = 1: Cw = 140.004062, c = 1 (F2-8a), rts = 1.261378,
/// Lp = 51.2268, Lr = 142.702064; Lb > Lr: Mn = 478.52798, phi Mn = 430.675182 (today, with the I-shape
/// Cw = 297.7985, 543.043339).
#[test]
fn d129_double_channel_ltb() {
    let sec = shape(
        "2C10",
        FramePropType::DbChannel,
        [10.0, 5.575, 0.436, 0.24, 0.375],
        8.91584,
        134.18427,
        13.022771,
        0.371447,
        26.836854,
        31.681946,
        8.0,
        10.0,
    );
    // Cw written out: a^2 tw h^3/6 + h^2 tf [a^3 - (a - b')^3]/3, a = dis/2 + tw/2, b' = (t2 - dis)/2 - tw/2
    let h0: f64 = 9.564;
    let (a_w, bp): (f64, f64) = (0.1875 + 0.12, 2.6 - 0.12);
    let cw = a_w * a_w * 0.24 * h0.powi(3) / 6.0
        + h0 * h0 * 0.436 * (a_w.powi(3) - (a_w - bp).powi(3)) / 3.0;
    close(cw, 140.004062, 1e-6, "Cw");
    let rts = ((13.022771 * cw).sqrt() / 26.836854).sqrt();
    close(rts, 1.261378, 1e-5, "rts");
    let jc = 0.371447 / (26.836854 * h0);
    let lr = 1.95 * rts * E / (0.7 * FY) * (jc + (jc * jc + 6.76 * (0.7 * FY / E).powi(2)).sqrt()).sqrt();
    close(lr, 142.702064, 1e-5, "Lr");
    let lb_rts = 240.0 / rts;
    let fcr = PI * PI * E / (lb_rts * lb_rts) * (1.0 + 0.078 * jc * lb_rts * lb_rts).sqrt();
    let mn = (fcr * 26.836854).min(FY * 31.681946);
    close(mn, 478.52798, 1e-6, "Mn");
    close(phi_m3(sec, 240.0, 100.0), 0.9 * mn, 1e-9, "2C10 LTB");
    close(0.9 * mn, 430.675182, 1e-6, "hand value");
}

/// G83: a channel with h/tw = 19/0.2 = 95 > 3.76 r = 90.55 (t3 20, t2 4, tf 0.5, tw 0.2) is refused
/// under M3 by name, and designed under compression alone; the minor axis does not need the web.
#[test]
fn d129_channel_with_a_noncompact_web_is_refused_in_major_bending_only() {
    let sec = shape(
        "C20",
        FramePropType::Channel,
        [20.0, 4.0, 0.5, 0.2, 0.0],
        7.8,
        494.5,
        5.0,
        0.37,
        49.45,
        58.0,
        3.0,
        5.0,
    );
    let err = try_design(sec.clone(), 60.0, vec![station(0.0, 100.0, 0.0)]).unwrap_err();
    let msg = err.to_string();
    assert!(matches!(err, DesignError::AiscLimitStateNotBuilt { .. }));
    assert!(msg.contains("'M1'") && msg.contains("web") && msg.contains("G83"), "{msg}");
    assert!(try_design(sec.clone(), 60.0, vec![station(-10.0, 0.0, 0.0)]).is_ok());
    assert!(try_design(sec, 60.0, vec![station(0.0, 0.0, 5.0)]).is_ok());
}

/// Dimensions that do not define the section are refused where a design needs them: a channel with
/// t3 <= 2 tf.
#[test]
fn d128_dimensions_that_do_not_define_the_section_are_refused() {
    let sec = shape(
        "CBAD",
        FramePropType::Channel,
        [0.8, 2.6, 0.436, 0.24, 0.0],
        1.0,
        1.0,
        1.0,
        0.1,
        1.0,
        1.0,
        1.0,
        1.0,
    );
    let err = try_design(sec.clone(), 60.0, vec![station(-1.0, 0.0, 0.0)]).unwrap_err();
    assert!(err.to_string().contains("dimensions do not define the section"));
    assert!(try_design(sec, 60.0, vec![station(1.0, 0.0, 0.0)]).is_ok());
}

// ------------------------------------------------------------------ status and Auto Select

/// An I with a slender web still reads "SLENDER" (limit state not built, F5), a pipe or tee that is
/// now designed reads by ratio, a built-up slender I keeps its status.
#[test]
fn d129_i_with_a_slender_web_keeps_the_slender_status() {
    let sec = make_i_section("SLW", "A992Fy50", 20.0, 6.0, 0.5, 0.1, 6.0, 0.5);
    let r = try_design(sec, 60.0, vec![station(0.0, 50.0, 0.0)]).unwrap();
    assert_eq!(r.status, "SLENDER");
}

/// Auto Select in the manner of `d91_auto_select_*`: a list [P30X0.12, the heavier P10X0.5] with
/// M3 = 1000 chooses P30X0.12 (phi Mn 2887.44 >= 1000, lighter area); it was skipped as "SLENDER".
#[test]
fn d129_auto_select_chooses_the_lighter_slender_pipe() {
    let mut m = SapModel::new(Units::KipInF);
    let light = make_pipe("P30X0.12", "A992Fy50", 30.0, 0.12);
    let heavy = make_pipe("P10X0.5", "A992Fy50", 10.0, 0.5);
    assert!(heavy.area > light.area);
    m.frame_sections.insert(light.name.clone(), light);
    m.frame_sections.insert(heavy.name.clone(), heavy);
    m.define_auto_select_list("LIST", &["P30X0.12".to_string(), "P10X0.5".to_string()])
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
    cr.frame_forces.insert("BM".into(), vec![station(0.0, 1000.0, 0.0)]);
    m.case_results.insert("DEAD".into(), cr);
    m.is_locked = true;
    try_run_steel_design(&mut m).unwrap();
    assert_eq!(m.steel_design_results["BM"].section_name, "P30X0.12");
}

// ------------------------------------------------------------------ EC3 stays as it was

/// EC3 results and refusals do not move (D133): the values below were taken from the code before
/// this batch (the same inputs run on the base tree: the ratios agree to the last bit). The
/// compact pipe P10x0.5 with M3 = 500 kip-in in a 120 in member, a C10 channel with M3 = 100 and a
/// double channel 2C10 with M3 = 100 (flexure through the EC3 path, whose Iw is untouched here)
/// keep their ratios bitwise; the slender pipe, the C10 in compression and the T6x8 stay refused by
/// the EC3 class rules with their own error.
#[test]
fn d129_ec3_results_and_refusals_are_bitwise_unchanged() {
    let ec3 = |mut m: SapModel| -> Result<SteelDesignResult, DesignError> {
        m.steel_design_code = "Eurocode 3-2005".to_string();
        try_run_steel_design(&mut m)?;
        Ok(m.steel_design_results["M1"].clone())
    };
    let p = make_pipe("P10X0.5", "A992Fy50", 10.0, 0.5);
    let r = ec3(member(p, 120.0, vec![station(0.0, 500.0, 0.0)])).unwrap();
    assert_eq!(r.total_ratio.to_bits(), 0x3fcc56e8623e58cd);
    assert_eq!(r.status, "OK");
    let r = ec3(member(c10(), 120.0, vec![station(0.0, 100.0, 0.0)])).unwrap();
    assert_eq!(r.total_ratio.to_bits(), 0x3fd3fd607935f717);
    let d = shape(
        "2C10",
        FramePropType::DbChannel,
        [10.0, 5.575, 0.436, 0.24, 0.375],
        8.91584,
        134.18427,
        13.022771,
        0.371447,
        26.836854,
        31.681946,
        8.0,
        10.0,
    );
    let r = ec3(member(d, 240.0, vec![station(0.0, 100.0, 0.0)])).unwrap();
    assert_eq!(r.total_ratio.to_bits(), 0x3fc8cba9082784d9);
    // refusals of the EC3 path keep their own variant
    let err = ec3(member(c10(), 120.0, vec![station(-10.0, 0.0, 0.0)])).unwrap_err();
    assert!(matches!(err, DesignError::Ec3SectionClass { .. }), "{err}");
    let slender = make_pipe("P30X0.12", "A992Fy50", 30.0, 0.12);
    let err = ec3(member(slender, 120.0, vec![station(0.0, 2000.0, 0.0)])).unwrap_err();
    assert!(matches!(err, DesignError::Ec3SectionClass { .. }), "{err}");
    let t = shape(
        "T6X8",
        FramePropType::T,
        [6.0, 8.0, 0.5, 0.4, 0.0],
        6.2,
        18.40336,
        21.362667,
        0.450667,
        3.92774,
        6.94875,
        5.340667,
        8.22,
    );
    let err = ec3(member(t, 120.0, vec![station(0.0, 20.0, 0.0)])).unwrap_err();
    assert!(matches!(err, DesignError::Ec3SectionClass { .. }), "{err}");
    // The EC3 status rule is the earlier one: a box with a slender flange (12 x 12 x 3/16, Class 3
    // for EC3 in bending) is designed and reads "SLENDER", while AISC refuses it (G82).
    let b = make_box_tube("B12", "A992Fy50", 12.0, 12.0, 0.1875, 0.1875);
    let r = ec3(member(b.clone(), 120.0, vec![station(0.0, 100.0, 0.0)])).unwrap();
    assert_eq!(r.status, "SLENDER");
    assert_eq!(r.total_ratio.to_bits(), 0x3faa173e54f791cd);
    assert!(try_design(b, 120.0, vec![station(0.0, 100.0, 0.0)]).is_err());
}
