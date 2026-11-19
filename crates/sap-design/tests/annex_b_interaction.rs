//! EN 1993-1-1 6.3.3(4) with the interaction factors of Annex B (Method 2), Tables B.1 to B.3
//! (D89, G71). N and mm, S355 (fy 355 MPa, E 210000 MPa). Every expected value is a hand
//! calculation: the helpers below re-derive the closed forms of the standard (the formulas are
//! written out in the comments) and do not call the crate; the literals are the reviewer's hand
//! numbers (`hand_annexb.py`) and, for the factors, the published check of ECCS publication
//! No. 119, Worked Example 5 (IPE 500).

use sap_design::steel::{
    ec3_annex_b_factors, ec3_equivalent_moment_factor, Ec3AnnexBShape, Ec3InteractionFactors,
};
use sap_design::try_run_steel_design;
use sap_model::{
    CaseResults, FramePropType, FrameSection, FrameStationResult, SapModel, SteelDesignOverwrite,
};
use sap_section::{make_box_tube, make_i_section, make_pipe};
use sap_units::Units;

const EC3: &str = "Eurocode 3-2005";
const FY: f64 = 355.0;
const E: f64 = 210_000.0;

fn rel_close(got: f64, want: f64, tol: f64) -> bool {
    (got - want).abs() <= tol * want.abs().max(1e-12)
}

fn assert_rel(got: f64, want: f64, tol: f64, what: &str) {
    assert!(rel_close(got, want, tol), "{what}: got {got}, expected {want}");
}

// ------------------------------------------------------------------ the model helpers

/// Stations of a member `len` long at 0, len/2, len with the given axial force and
/// (M3, M2) at the three stations.
fn stations(len: f64, p: f64, m3: [f64; 3], m2: [f64; 3]) -> Vec<FrameStationResult> {
    (0..3)
        .map(|i| FrameStationResult {
            station: 0.5 * len * i as f64,
            rel_dist: 0.5 * i as f64,
            p,
            m3: m3[i],
            m2: m2[i],
            ..Default::default()
        })
        .collect()
}

fn member(
    code: &str,
    sec: FrameSection,
    len: f64,
    st: Vec<FrameStationResult>,
    ow: Option<SteelDesignOverwrite>,
) -> SapModel {
    let mut model = SapModel::new(Units::NMmC);
    model.steel_design_code = code.to_string();
    let name = sec.name.clone();
    model.frame_sections.insert(name.clone(), sec);
    model.add_point("PI", 0.0, 0.0, 0.0);
    model.add_point("PJ", 0.0, 0.0, len);
    model.add_frame("M1", "PI", "PJ", &name);
    let mat = model.materials.get_mut("A992Fy50").unwrap();
    mat.fy = FY;
    mat.e = E;
    mat.g = E / 2.6;
    model.frames.get_mut("M1").unwrap().steel_overwrite = ow;
    let mut cr = CaseResults {
        case_name: "LC".into(),
        ..Default::default()
    };
    cr.frame_forces.insert("M1".into(), st);
    model.case_results.insert("LC".into(), cr);
    model
}

/// K = 1 and Cb = 1 (so that C1 = 1 in the elastic critical moment).
fn ow_k1() -> SteelDesignOverwrite {
    SteelDesignOverwrite {
        k_major: Some(1.0),
        k_minor: Some(1.0),
        cb: Some(1.0),
        ..Default::default()
    }
}

fn i300() -> FrameSection {
    make_i_section("I300", "A992Fy50", 300.0, 300.0, 19.0, 11.0, 300.0, 19.0)
}

fn design(model: &mut SapModel) -> sap_model::SteelDesignResult {
    try_run_steel_design(model).unwrap();
    model.steel_design_results["M1"].clone()
}

// ------------------------------------------------------------------ the hand calculation

/// EN 1993-1-1 6.3.1.2: chi = 1/(phi + sqrt(phi^2 - lambda^2)), phi = 0.5 (1 + alpha (lambda - 0.2) + lambda^2).
fn chi(lam: f64, alpha: f64) -> f64 {
    if lam <= 0.2 {
        return 1.0;
    }
    let phi = 0.5 * (1.0 + alpha * (lam - 0.2) + lam * lam);
    (1.0 / (phi + (phi * phi - lam * lam).sqrt())).min(1.0)
}

/// Doubly symmetric I without fillets: h, b, tf, tw.
struct Hand {
    a: f64,
    i33: f64,
    i22: f64,
    wpl_y: f64,
    wpl_z: f64,
    wel_z: f64,
    j: f64,
    h0: f64,
}

fn hand_i(h: f64, b: f64, tf: f64, tw: f64) -> Hand {
    let hw = h - 2.0 * tf;
    Hand {
        a: 2.0 * b * tf + hw * tw,
        i33: (b * h.powi(3) - (b - tw) * hw.powi(3)) / 12.0,
        i22: (2.0 * tf * b.powi(3) + hw * tw.powi(3)) / 12.0,
        wpl_y: b * tf * (h - tf) + tw * hw * hw / 4.0,
        wpl_z: 0.25 * (2.0 * tf * b * b + hw * tw * tw),
        wel_z: ((2.0 * tf * b.powi(3) + hw * tw.powi(3)) / 12.0) / (b / 2.0),
        j: (2.0 * b * tf.powi(3) + hw * tw.powi(3)) / 3.0,
        h0: h - tf,
    }
}

/// lambda = sqrt(A fy / Ncr), Ncr = pi^2 E I / L^2.
fn lambda(a: f64, i: f64, l: f64) -> f64 {
    (a * FY / (std::f64::consts::PI.powi(2) * E * i / (l * l))).sqrt()
}

/// Mcr = C1 pi^2 E Iz/L^2 sqrt(Iw/Iz + L^2 G It/(pi^2 E Iz)), C1 = 1, Iw = Iz h0^2/4, G = E/2.6.
fn mcr(s: &Hand, l: f64) -> f64 {
    let pi2 = std::f64::consts::PI.powi(2);
    let g = E / 2.6;
    let iw = 0.25 * s.i22 * s.h0 * s.h0;
    pi2 * E * s.i22 / (l * l) * (iw / s.i22 + l * l * g * s.j / (pi2 * E * s.i22)).sqrt()
}

// ------------------------------------------------------------------ factor formulas (unit level)

/// ECCS No. 119 Worked Example 5 (IPE 500, Class 1): lambda_y 0.195, lambda_z 0.927,
/// chi_y 1.00, chi_z 0.644, N_c,Rd 2715 kN, N_Ed 500 kN, Cmy = CmLT = 0.925, Cmz = 0.6.
/// n_y = 500/(1.00 x 2715) = 0.184162, n_z = 500/(0.644 x 2715) = 0.285965.
/// kyy = 0.925 (1 + (0.195 - 0.2) 0.184162) = 0.924148 (cap 0.925 (1 + 0.8 n_y) = 1.06)
/// kzz = 0.6 (1 + (2 x 0.927 - 0.6) 0.285965) = 0.815164 (cap 0.6 (1 + 1.4 n_z) = 0.84)
/// kyz = 0.6 kzz = 0.489098
/// kzy (lambda_z >= 0.4, Table B.2) = 1 - 0.1 x 0.927 x 0.285965/(0.925 - 0.25) = 0.960729
///   (the second form 1 - 0.1 n_z/0.675 = 0.957636 is smaller; the larger governs).
/// Published: 0.924, 0.489, 0.961, 0.815.
#[test]
fn factors_reproduce_eccs_worked_example_5() {
    let n_y = 500.0 / (1.00 * 2715.0);
    let n_z = 500.0 / (0.644 * 2715.0);
    let k = ec3_annex_b_factors(0.195, 0.927, n_y, n_z, 0.925, 0.6, 0.925, true, Ec3AnnexBShape::I);
    assert_rel(k.kyy, 0.92415, 1e-5, "kyy");
    assert_rel(k.kyz, 0.48910, 1e-5, "kyz");
    assert_rel(k.kzy, 0.96073, 1e-5, "kzy");
    assert_rel(k.kzz, 0.81516, 1e-5, "kzz");
    // against the published three digits
    for (got, want) in [(k.kyy, 0.924), (k.kyz, 0.489), (k.kzy, 0.961), (k.kzz, 0.815)] {
        assert!((got - want).abs() < 1e-3, "{got} against published {want}");
    }
}

/// Table B.2, plastic, lambda_z < 0.4: kzy = 0.6 + lambda_z <= 1 - 0.1 lambda_z n_z/(CmLT - 0.25).
/// lambda_z 0.3, n_z 0.5, CmLT 0.6: 0.6 + 0.3 = 0.9 against 1 - 0.03 x 0.5/0.35 = 0.957143: the
/// smaller, 0.9, governs (the lambda_z >= 0.4 branch would give 0.957143).
#[test]
fn factors_kzy_below_lambda_04_takes_0p6_plus_lambda() {
    let k = ec3_annex_b_factors(0.3, 0.3, 0.5, 0.5, 1.0, 1.0, 0.6, true, Ec3AnnexBShape::I);
    assert_rel(k.kzy, 0.9, 1e-12, "kzy");
    // negative: lambda_z 0.4 takes the other branch: 1 - 0.04 x 0.5/0.35 = 0.942857 (and
    // 1 - 0.1 x 0.5/0.35 = 0.857143 is smaller)
    let k = ec3_annex_b_factors(0.4, 0.4, 0.5, 0.5, 1.0, 1.0, 0.6, true, Ec3AnnexBShape::I);
    assert_rel(k.kzy, 1.0 - 0.1 * 0.4 * 0.5 / 0.35, 1e-12, "kzy at 0.4");
}

/// Table B.1 (RHS, not susceptible), lambda 0.8, n_y 0.3, n_z 0.4, Cmy 0.9, Cmz 0.6.
/// Plastic: kyy = 0.9 (1 + 0.6 x 0.3) = 1.062 (cap 0.9 (1 + 0.24) = 1.116),
/// kzz = 0.6 (1 + 0.6 x 0.4) = 0.744 (cap 0.6 (1 + 0.32) = 0.792), kyz = 0.6 kzz = 0.4464,
/// kzy = 0.6 kyy = 0.6372. Elastic: kyy = 0.9 (1 + 0.6 x 0.8 x 0.3) = 1.0296 (cap 1.062),
/// kzz = 0.6 (1 + 0.6 x 0.8 x 0.4) = 0.7152 (cap 0.744), kyz = kzz = 0.7152, kzy = 0.8 kyy = 0.82368.
#[test]
fn factors_table_b1_rhs_plastic_and_elastic() {
    let p = ec3_annex_b_factors(0.8, 0.8, 0.3, 0.4, 0.9, 0.6, 1.0, true, Ec3AnnexBShape::Rhs);
    assert_rel(p.kyy, 1.062, 1e-12, "plastic kyy");
    assert_rel(p.kyz, 0.4464, 1e-12, "plastic kyz");
    assert_rel(p.kzy, 0.6372, 1e-12, "plastic kzy");
    assert_rel(p.kzz, 0.744, 1e-12, "plastic kzz");
    let q = ec3_annex_b_factors(0.8, 0.8, 0.3, 0.4, 0.9, 0.6, 1.0, false, Ec3AnnexBShape::Rhs);
    assert_rel(q.kyy, 1.0296, 1e-12, "elastic kyy");
    assert_rel(q.kyz, 0.7152, 1e-12, "elastic kyz");
    assert_rel(q.kzy, 0.82368, 1e-12, "elastic kzy");
    assert_rel(q.kzz, 0.7152, 1e-12, "elastic kzz");
    // the caps: lambda 3 makes the first form larger than the cap, so the cap governs
    // kyy = 1.0 (1 + 0.8 x 0.3) = 1.24 and not 1 + 2.8 x 0.3 = 1.84
    let c = ec3_annex_b_factors(3.0, 3.0, 0.3, 0.3, 1.0, 1.0, 1.0, true, Ec3AnnexBShape::Rhs);
    assert_rel(c.kyy, 1.24, 1e-12, "cap kyy");
}

/// Table B.3, diagrams as (position, moment); the mid-length moment is the middle point.
/// Linear: 0.6 + 0.4 psi >= 0.4. Span-load rows take the larger of uniform and concentrated.
#[test]
fn moment_factor_table_b3_rows() {
    let cm = |a: f64, b: f64, m: f64| ec3_equivalent_moment_factor(&[(0.0, a), (0.5, m), (1.0, b)]);
    // linear: (1, psi, (1 + psi)/2)
    assert_rel(cm(1.0, 1.0, 1.0), 1.0, 1e-12, "psi 1");
    assert_rel(cm(1.0, 0.0, 0.5), 0.6, 1e-12, "psi 0");
    assert_rel(cm(1.0, -1.0, 0.0), 0.4, 1e-12, "psi -1");
    assert_rel(cm(1.0, 0.5, 0.75), 0.8, 1e-12, "psi 0.5");
    // simply supported, span load: Mh = 0 at both ends, Ms = 1, alpha_h = 0: 0.95 + 0 (uniform)
    assert_rel(cm(0.0, 0.0, 1.0), 0.95, 1e-12, "simply supported");
    // fixed-fixed, uniform load: Mh = -1/12, psi = 1, Ms = +1/24, alpha_s = -0.5:
    // uniform 0.1 + 0.8 x 0.5 = 0.5, concentrated 0.8 x 0.5 = 0.4: 0.5
    assert_rel(cm(-1.0 / 12.0, -1.0 / 12.0, 1.0 / 24.0), 0.5, 1e-12, "fixed-fixed");
    // propped, uniform: Mh = -1/8, psi = 0, Ms = +1/16, alpha_s = -0.5: 0.5
    assert_rel(cm(-1.0 / 8.0, 0.0, 1.0 / 16.0), 0.5, 1e-12, "propped");
    // alpha_s >= 0 (Mh 1, psi 0.5, Ms 0.9; the chord gives 0.75): 0.2 + 0.8 x 0.9 = 0.92
    assert_rel(cm(1.0, 0.5, 0.9), 0.92, 1e-12, "alpha_s >= 0");
    // alpha_s < 0, psi < 0 (Mh 1, psi -0.5, Ms -0.4, alpha_s -0.4): uniform
    // 0.1 (1 + 0.5) + 0.32 = 0.47, concentrated 0.2 x 0.5 + 0.32 = 0.42: 0.47
    assert_rel(cm(1.0, -0.5, -0.4), 0.47, 1e-12, "alpha_s < 0, psi < 0");
    // |Ms| > |Mh|, alpha_h = 0.5: 0.95 + 0.025 = 0.975
    assert_rel(cm(0.5, 0.5, 1.0), 0.975, 1e-12, "alpha_h >= 0");
    // alpha_h = -0.5 (Mh -0.5, other end +0.5, psi -1): 0.95 + 0.05 (-0.5)(1 - 2) = 0.975
    assert_rel(cm(-0.5, 0.5, 1.0), 0.975, 1e-12, "alpha_h < 0, psi < 0");
    // a diagram that is zero everywhere: 1.0
    assert_rel(cm(0.0, 0.0, 0.0), 1.0, 1e-12, "zero");
    // a linear diagram with five stations and a kink is not linear: ends 1 and 0, Ms 0.5 on the
    // chord but a station at 0.25 off it
    let kinked = [(0.0, 1.0), (0.25, 0.9), (0.5, 0.5), (1.0, 0.0)];
    // psi = 0, Mh = 1, alpha_s = 0.5: 0.2 + 0.4 = 0.6 (the linear value would be 0.6 as well):
    // use a case where they differ: Ms 0.9 above the chord
    assert_rel(ec3_equivalent_moment_factor(&kinked), 0.6, 1e-12, "kinked, same value");
    let bulged = [(0.0, 1.0), (0.5, 0.9), (1.0, 0.0)];
    assert_rel(ec3_equivalent_moment_factor(&bulged), 0.2 + 0.8 * 0.9, 1e-12, "bulged");
}

// ------------------------------------------------------------------ through the design entry

/// I 300 x 300, tf 19, tw 11, L = 6000, S355: A 14282, I33 2.41868e8, I22 8.55291e7,
/// Wpl,y 1.79047e6, Wpl,z 862926. Flange (300 - 11)/2/19 = 7.605 <= 9 eps = 7.323?  No: 7.605 is
/// Class 2 (<= 10 eps = 8.136); web 23.8 (Class 1). h/b = 1: curves b (major) and c (minor).
/// lambda_y 0.603408, lambda_z 1.014714, chi_y 0.835345, chi_z 0.531448, N_pl = 5070110 N.
/// N = 1500 kN: n_y = 0.354167, n_z = 0.556690.
struct Case1 {
    h: Hand,
    n_pl: f64,
    lam_y: f64,
    lam_z: f64,
    chi_y: f64,
    chi_z: f64,
}

fn case1(len_z: f64) -> Case1 {
    let h = hand_i(300.0, 300.0, 19.0, 11.0);
    let lam_y = lambda(h.a, h.i33, 6000.0);
    let lam_z = lambda(h.a, h.i22, len_z);
    Case1 {
        n_pl: h.a * FY,
        chi_y: chi(lam_y, 0.34),
        chi_z: chi(lam_z, 0.49),
        lam_y,
        lam_z,
        h,
    }
}

const N_ED: f64 = 1_500_000.0;

/// Test 1: M2 = 100 kNm at every station (Cmz = 1.0, linear psi = 1).
/// kzz = min(1 (1 + (2 x 1.014714 - 0.6) 0.556690) = 1.795..., 1 (1 + 1.4 x 0.556690) = 1.779365)
/// = 1.779365 (the cap governs); kyz = 0.6 kzz = 1.067619.
/// m_z = 100e6/(862926 x 355) = 0.326440.
/// eq 6.61 = 0.354167 + 1.067619 x 0.326440 = 0.702676;
/// eq 6.62 = 0.556690 + 1.779365 x 0.326440 = 1.137539 > 1 OVERSTRESSED.
/// Cross section 6.2.1(7): 1500e3/5070110 + 0.326440 = 0.622288 (the sum of the earlier code, with
/// N_b,Rd, gave 0.883126).
#[test]
fn test1_uniform_minor_moment_governed_by_eq_6_62() {
    let c = case1(6000.0);
    assert_rel(c.n_pl, 5_070_110.0, 1e-9, "N_pl");
    assert_rel(c.lam_y, 0.603408, 1e-6, "lambda_y");
    assert_rel(c.lam_z, 1.014714, 1e-6, "lambda_z");
    assert_rel(c.chi_y, 0.835345, 1e-6, "chi_y");
    assert_rel(c.chi_z, 0.531448, 1e-6, "chi_z");
    let n_y = N_ED / (c.chi_y * c.n_pl);
    let n_z = N_ED / (c.chi_z * c.n_pl);
    let kzz = (1.0 + (2.0 * c.lam_z - 0.6) * n_z).min(1.0 + 1.4 * n_z);
    assert_rel(kzz, 1.779365, 1e-6, "kzz");
    let kyz = 0.6 * kzz;
    let m_z = 100.0e6 / (c.h.wpl_z * FY);
    let eq61 = n_y + kyz * m_z;
    let eq62 = n_z + kzz * m_z;
    assert_rel(eq61, 0.702676, 1e-6, "hand eq 6.61");
    assert_rel(eq62, 1.137539, 1e-6, "hand eq 6.62");

    let mut model = member(
        EC3,
        i300(),
        6000.0,
        stations(6000.0, -N_ED, [0.0; 3], [100.0e6; 3]),
        Some(ow_k1()),
    );
    let r = design(&mut model);
    assert_eq!(r.eq_name, "EC3-6.3.3(4) eq 6.62");
    assert_rel(r.total_ratio, eq62, 1e-6, "total");
    assert_rel(r.pr_ratio, n_z, 1e-6, "pr = n_z");
    assert_rel(r.mr_minor_ratio, kzz * m_z, 1e-6, "mr minor = kzz m_z");
    assert!(r.mr_major_ratio.abs() < 1e-12);
    assert_eq!(r.status, "OVERSTRESSED");
    assert_eq!(r.notes.len(), 1, "{:?}", r.notes);
    let note = &r.notes[0];
    assert!(
        note.contains("Cmz = 1.0000") && note.contains("kzz = 1.7794") && note.contains("kyz = 1.0676"),
        "{note}"
    );
    // The cross-section check alone is 0.622288, below the member check.
    assert_rel(N_ED / c.n_pl + m_z, 0.622288, 1e-6, "section check, by hand");
}

/// Test 2: M2 = 100 kNm at I falling linearly to 0 at J: psi = 0, Cmz = 0.6 + 0 = 0.6.
/// kzz = 0.6 (1 + 1.4 x 0.556690) = 1.067619 (cap), eq 6.62 = 0.556690 + 1.067619 x 0.326440 = 0.905199.
/// eq 6.61 = 0.354167 + 0.6 x 1.067619 x 0.326440 = 0.563273.
#[test]
fn test2_linear_minor_moment() {
    let c = case1(6000.0);
    let n_y = N_ED / (c.chi_y * c.n_pl);
    let n_z = N_ED / (c.chi_z * c.n_pl);
    let cmz = 0.6;
    let kzz = (cmz * (1.0 + (2.0 * c.lam_z - 0.6) * n_z)).min(cmz * (1.0 + 1.4 * n_z));
    let m_z = 100.0e6 / (c.h.wpl_z * FY);
    let eq62 = n_z + kzz * m_z;
    assert_rel(kzz, 1.067619, 1e-6, "kzz");
    assert_rel(eq62, 0.905199, 1e-6, "hand eq 6.62");
    assert_rel(n_y + 0.6 * kzz * m_z, 0.563273, 1e-6, "hand eq 6.61");
    let mut model = member(
        EC3,
        i300(),
        6000.0,
        stations(6000.0, -N_ED, [0.0; 3], [100.0e6, 50.0e6, 0.0]),
        Some(ow_k1()),
    );
    let r = design(&mut model);
    assert_eq!(r.eq_name, "EC3-6.3.3(4) eq 6.62");
    assert_rel(r.total_ratio, eq62, 1e-6, "total");
    assert_eq!(r.status, "OK");
    // station of the largest moment: the I end
    assert_eq!(r.governing_station, 0.0);
}

/// Test 3: M3 = +300 kNm at I, -150 kNm at J (linear, psi = -0.5: Cmy = CmLT = 0.6 - 0.2 = 0.4),
/// Cb = 1. Mcr = pi^2 E Iz/L^2 sqrt(Iw/Iz + L^2 G It/(pi^2 E Iz)) = 1.03463e9 Nmm,
/// lambda_LT = sqrt(Wpl,y fy/Mcr) = 0.783799, chi_LT (alpha 0.21) = 0.804830, M_b,Rd = 5.11564e8.
/// kyy = 0.4 (1 + (0.603408 - 0.2) 0.354167) = 0.457150 (cap 0.4 (1 + 0.8 n_y) = 0.513333);
/// kzy = 1 - 0.1 x 1.014714 x 0.556690/(0.4 - 0.25) = 0.623... larger of the two forms
/// (second: 1 - 0.1 x 0.556690/0.15 = 0.628873): 0.628874; m_y = 300e6/5.11564e8 = 0.586437.
/// eq 6.61 = 0.354167 + 0.457150 x 0.586437 = 0.622256; eq 6.62 = 0.556690 + 0.628874 x 0.586437 = 0.925484.
/// Cross section at I: 0.29585 + 300e6/(1.79047e6 x 355) = 0.767834. 6.3.2 alone 0.586437.
#[test]
fn test3_linear_major_moment() {
    let c = case1(6000.0);
    let n_y = N_ED / (c.chi_y * c.n_pl);
    let n_z = N_ED / (c.chi_z * c.n_pl);
    let m_cr = mcr(&c.h, 6000.0);
    assert_rel(m_cr, 1.03463e9, 1e-5, "Mcr");
    let lam_lt = (c.h.wpl_y * FY / m_cr).sqrt();
    let chi_lt = chi(lam_lt, 0.21);
    let m_b = chi_lt * c.h.wpl_y * FY;
    assert_rel(lam_lt, 0.783799, 1e-6, "lambda_LT");
    assert_rel(chi_lt, 0.804830, 1e-6, "chi_LT");
    assert_rel(m_b, 5.11564e8, 1e-5, "M_b,Rd");
    let (cmy, cmz, cmlt) = (0.4, 1.0, 0.4);
    let kyy = (cmy * (1.0 + (c.lam_y - 0.2) * n_y)).min(cmy * (1.0 + 0.8 * n_y));
    let kzy = (1.0 - 0.1 * c.lam_z * n_z / (cmlt - 0.25)).max(1.0 - 0.1 * n_z / (cmlt - 0.25));
    let kzz = cmz * (1.0 + 1.4 * n_z).min(1.0 + (2.0 * c.lam_z - 0.6) * n_z);
    let _ = kzz;
    let m_y = 300.0e6 / m_b;
    let eq61 = n_y + kyy * m_y;
    let eq62 = n_z + kzy * m_y;
    assert_rel(kyy, 0.457150, 1e-6, "kyy");
    assert_rel(kzy, 0.628874, 1e-6, "kzy");
    assert_rel(eq61, 0.622256, 1e-6, "hand eq 6.61");
    assert_rel(eq62, 0.925484, 1e-6, "hand eq 6.62");
    assert_rel(
        N_ED / c.n_pl + 300.0e6 / (c.h.wpl_y * FY),
        0.767834,
        1e-6,
        "section check at I, by hand",
    );

    let mut model = member(
        EC3,
        i300(),
        6000.0,
        stations(6000.0, -N_ED, [300.0e6, 75.0e6, -150.0e6], [0.0; 3]),
        Some(ow_k1()),
    );
    let r = design(&mut model);
    assert_eq!(r.eq_name, "EC3-6.3.3(4) eq 6.62");
    assert_rel(r.total_ratio, eq62, 1e-6, "total");
    assert_rel(r.pr_ratio, n_z, 1e-6, "pr");
    assert_rel(r.mr_major_ratio, kzy * m_y, 1e-6, "mr major = kzy m_y");
    assert_eq!(r.governing_station, 0.0);
    assert!(r.notes[0].contains("Cmy = 0.4000") && r.notes[0].contains("CmLT = 0.4000"), "{:?}", r.notes);
    // 6.3.2 alone is below, 0.586437, so is the cross section, 0.767834
    assert!(r.total_ratio > 0.767834);
}

/// Test 4, Class 3: welded I 400 x 250, tf 11, tw 12, L = 3000, S355. Flange (250 - 12)/2/11 = 10.82
/// is above 10 eps = 8.136 and below 14 eps = 11.39: Class 3 in compression, so the elastic set
/// and W_el apply (A 10036, I33 2.62132e8, I22 2.87003e7, Wel,z 229602; curves a and b:
/// lambda_y 0.242938, lambda_z 0.734198, chi_y 0.990514, chi_z 0.764049), N = 500 kN,
/// M2 = 10 kNm at I falling to 0 at J (Cmz 0.6): n_y = 0.141684, n_z = 0.183679,
/// kzz = kyz = 0.6 (1 + 0.6 x 0.734198 x 0.183679) = 0.648548 (cap 0.6 (1 + 0.6 n_z) = 0.666124);
/// m_z = 10e6/(229602 x 355) = 0.122689; eq 6.61 = 0.221252, eq 6.62 = 0.183679 + 0.648548 x 0.122689
/// = 0.263247 (cross section 0.263026).
#[test]
fn test4_class_3_uses_the_elastic_set_and_modulus() {
    let h = hand_i(400.0, 250.0, 11.0, 12.0);
    let lam_y = lambda(h.a, h.i33, 3000.0);
    let lam_z = lambda(h.a, h.i22, 3000.0);
    let chi_y = chi(lam_y, 0.21);
    let chi_z = chi(lam_z, 0.34);
    let n_pl = h.a * FY;
    let n = 500.0e3;
    let (n_y, n_z) = (n / (chi_y * n_pl), n / (chi_z * n_pl));
    let cmz = 0.6;
    let kzz = (cmz * (1.0 + 0.6 * lam_z * n_z)).min(cmz * (1.0 + 0.6 * n_z));
    let m_z = 10.0e6 / (h.wel_z * FY);
    assert_rel(lam_z, 0.734198, 1e-6, "lambda_z");
    assert_rel(chi_z, 0.764049, 1e-6, "chi_z");
    assert_rel(kzz, 0.648548, 1e-6, "kzz");
    let eq61 = n_y + kzz * m_z;
    let eq62 = n_z + kzz * m_z;
    assert_rel(eq61, 0.221252, 1e-6, "hand eq 6.61");
    assert_rel(eq62, 0.263247, 1e-6, "hand eq 6.62");
    assert_rel(n / n_pl + m_z, 0.263026, 1e-6, "hand cross section");
    let sec = make_i_section("WI", "A992Fy50", 400.0, 250.0, 11.0, 12.0, 250.0, 11.0);
    let mut model = member(
        EC3,
        sec,
        3000.0,
        stations(3000.0, -n, [0.0; 3], [10.0e6, 5.0e6, 0.0]),
        Some(ow_k1()),
    );
    let r = design(&mut model);
    assert_eq!(r.eq_name, "EC3-6.3.3(4) eq 6.62");
    assert_rel(r.total_ratio, eq62, 1e-6, "total");
    // the plastic modulus would give 0.5 per cent less (kzz is plastic-set and Wpl,z is larger)
    assert!(h.wpl_z > h.wel_z);
}

/// Test 5, sway buckling mode: test 2 with `is_sway` and K = 1: Cmz = 0.9 (and Cmy = 0.9),
/// kzz = 0.9 (1 + 1.4 x 0.556690) = 1.601429, eq 6.62 = 0.556690 + 1.601429 x 100e6/(862926 x 355)
/// = 1.079454.
#[test]
fn test5_sway_mode_takes_cm_0p9() {
    let c = case1(6000.0);
    let n_z = N_ED / (c.chi_z * c.n_pl);
    let kzz = 0.9 * (1.0 + 1.4 * n_z);
    assert_rel(kzz, 1.601429, 1e-6, "kzz");
    let eq62 = n_z + kzz * 100.0e6 / (c.h.wpl_z * FY);
    assert_rel(eq62, 1.079454, 1e-6, "hand eq 6.62");
    let mut ow = ow_k1();
    ow.is_sway = true;
    let mut model = member(
        EC3,
        i300(),
        6000.0,
        stations(6000.0, -N_ED, [0.0; 3], [100.0e6, 50.0e6, 0.0]),
        Some(ow),
    );
    let r = design(&mut model);
    assert_rel(r.total_ratio, eq62, 1e-6, "total");
    assert!(r.notes[0].contains("Cmz = 0.9000") && r.notes[0].contains("Cmy = 0.9000"), "{:?}", r.notes);
    // negative: without the sway flag the same member gives the linear Cmz = 0.6 (test 2)
    let mut model = member(
        EC3,
        i300(),
        6000.0,
        stations(6000.0, -N_ED, [0.0; 3], [100.0e6, 50.0e6, 0.0]),
        Some(ow_k1()),
    );
    assert_rel(design(&mut model).total_ratio, 0.905199, 1e-6, "no sway");
}

/// Test 6, minor length ratio 0.5: the diagram is the one between braced points, which the model
/// does not hold, so Cmz = 1.0 (not 0.6). Minor buckling length 3000: lambda_z = sqrt(A fy/Ncr)
/// with Ncr = pi^2 E I22/3000^2 (hand below), chi_z of curve c; kzz = 1 (1 + (2 lambda_z - 0.6) n_z)
/// limited by 1 (1 + 1.4 n_z); eq 6.62 = n_z + kzz m_z with m_z = 0.326440.
#[test]
fn test6_unbraced_ratio_other_than_one_gives_cm_one() {
    let c = case1(3000.0);
    let n_y = N_ED / (c.chi_y * c.n_pl);
    let n_z = N_ED / (c.chi_z * c.n_pl);
    let kzz = (1.0 + (2.0 * c.lam_z - 0.6) * n_z).min(1.0 + 1.4 * n_z);
    let kyz = 0.6 * kzz;
    let m_z = 100.0e6 / (c.h.wpl_z * FY);
    let eq62 = n_z + kzz * m_z;
    let eq61 = n_y + kyz * m_z;
    let mut ow = ow_k1();
    ow.l_ratio_minor = Some(0.5);
    let mut model = member(
        EC3,
        i300(),
        6000.0,
        stations(6000.0, -N_ED, [0.0; 3], [100.0e6, 50.0e6, 0.0]),
        Some(ow),
    );
    let r = design(&mut model);
    let want = eq61.max(eq62);
    assert_rel(r.total_ratio, want, 1e-6, "total");
    assert!(r.notes[0].contains("Cmz = 1.0000"), "{:?}", r.notes);
    // with Cmz = 0.6 the same member would be lower
    let low = n_z + 0.6 * kzz * m_z;
    assert!(low < want - 0.05, "{low} against {want}");
}

/// Test 7a, Pipe (RHS column of Table B.1, SRSS of the two bending terms): 300 x 12 pipe, S355,
/// L = 4000, constant N = 1000 kN, M3 = 150 kNm, M2 = 100 kNm (psi = 1: Cm = 1.0 for all).
/// d/t = 25 is Class 2 (<= 70 eps^2 = 30.7). chi_y = chi_z (axisymmetric, curve a), M_b,Rd = M_c,Rd
/// for a pipe (no lateral-torsional buckling, taken from a run under M3 alone and checked below
/// against W_pl fy within chi_LT).
/// kyy = 1 + (lambda - 0.2) n <= 1 + 0.8 n, kzz equal, kyz = 0.6 kzz, kzy = 0.6 kyy;
/// eq 6.61 = n + sqrt((kyy m_y)^2 + (kyz m_z)^2), eq 6.62 = n + sqrt((kzy m_y)^2 + (kzz m_z)^2).
#[test]
fn test7_pipe_srss_and_box_table_b1() {
    // ----- pipe
    let sec = make_pipe("P300", "A992Fy50", 300.0, 12.0);
    let (a, i, wpl) = (sec.area, sec.i33, sec.z33);
    let n = 1000.0e3;
    let (m3, m2) = (150.0e6, 100.0e6);
    // M_b,Rd from the same member under M3 alone (not what is tested)
    let mut alone = member(
        EC3,
        sec.clone(),
        4000.0,
        stations(4000.0, 0.0, [m3; 3], [0.0; 3]),
        Some(ow_k1()),
    );
    let m_b = design(&mut alone).phi_mn33;
    assert!(m_b <= wpl * FY * (1.0 + 1e-12) && m_b > 0.9 * wpl * FY, "{m_b}");
    let lam = lambda(a, i, 4000.0);
    let chi_p = chi(lam, 0.21);
    let n_pl = a * FY;
    let n_v = n / (chi_p * n_pl);
    let k_yy = (1.0 + (lam - 0.2) * n_v).min(1.0 + 0.8 * n_v);
    let k_zz = k_yy;
    let (k_yz, k_zy) = (0.6 * k_zz, 0.6 * k_yy);
    let (m_y, m_z) = (m3 / m_b, m2 / (sec.z22 * FY));
    let eq61 = n_v + ((k_yy * m_y).powi(2) + (k_yz * m_z).powi(2)).sqrt();
    let eq62 = n_v + ((k_zy * m_y).powi(2) + (k_zz * m_z).powi(2)).sqrt();
    let mut model = member(
        EC3,
        sec,
        4000.0,
        stations(4000.0, -n, [m3; 3], [m2; 3]),
        Some(ow_k1()),
    );
    let r = design(&mut model);
    // station check 6.2.1(7) with the section moduli, SRSS of the bending terms
    let section = n / n_pl + ((m3 / (wpl * FY)).powi(2) + (m2 / (wpl * FY)).powi(2)).sqrt();
    let want = eq61.max(eq62).max(section);
    assert_rel(r.total_ratio, want, 1e-6, "pipe total");
    assert!(section < eq61.max(eq62), "member check governs the pipe");
    assert!(r.eq_name.starts_with("EC3-6.3.3(4)"), "{}", r.eq_name);
    // the SRSS form is smaller than the plain sum
    let sum61 = n_v + k_yy * m_y + k_yz * m_z;
    assert!(sum61 > eq61 + 0.01);

    // ----- box 250 x 200 x 10, Class 1, Table B.1 (not susceptible): kzy = 0.6 kyy, plastic set
    let sec = make_box_tube("B250", "A992Fy50", 250.0, 200.0, 10.0, 10.0);
    let (sec_z33, sec_z22) = (sec.z33, sec.z22);
    let n = 800.0e3;
    let (m3, m2) = (120.0e6, 60.0e6);
    let mut alone = member(
        EC3,
        sec.clone(),
        4000.0,
        stations(4000.0, 0.0, [m3; 3], [0.0; 3]),
        Some(ow_k1()),
    );
    let m_b = design(&mut alone).phi_mn33;
    let lam_y = lambda(sec.area, sec.i33, 4000.0);
    let lam_z = lambda(sec.area, sec.i22, 4000.0);
    let (chi_y, chi_z) = (chi(lam_y, 0.21), chi(lam_z, 0.21));
    let n_pl = sec.area * FY;
    let (n_y, n_z) = (n / (chi_y * n_pl), n / (chi_z * n_pl));
    let k_yy = (1.0 + (lam_y - 0.2) * n_y).min(1.0 + 0.8 * n_y);
    let k_zz = (1.0 + (lam_z - 0.2) * n_z).min(1.0 + 0.8 * n_z);
    let (k_yz, k_zy) = (0.6 * k_zz, 0.6 * k_yy);
    let (m_y, m_z) = (m3 / m_b, m2 / (sec.z22 * FY));
    let eq61 = n_y + k_yy * m_y + k_yz * m_z;
    let eq62 = n_z + k_zy * m_y + k_zz * m_z;
    let mut model = member(
        EC3,
        sec,
        4000.0,
        stations(4000.0, -n, [m3; 3], [m2; 3]),
        Some(ow_k1()),
    );
    let r = design(&mut model);
    // Here the station check of 6.2.1(7) governs: N/N_pl + M3/(Wpl,y fy) + M2/(Wpl,z fy) = 0.262037
    // + 0.454034 + 0.265328 = 0.981399, above eq 6.61 = 0.969568 and eq 6.62 = 0.903713.
    let section = n / n_pl + m3 / (sec_z33 * FY) + m2 / (sec_z22 * FY);
    assert_rel(section, 0.981399, 1e-6, "hand cross section");
    assert_rel(eq61, 0.969568, 1e-6, "hand eq 6.61");
    assert_rel(r.total_ratio, section.max(eq61).max(eq62), 1e-6, "box total");
    assert_eq!(r.eq_name, "EC3-6.2");
}

/// Box 250 x 200 x 10, L = 7000 (more slender: the member equations govern over 6.2.1(7)),
/// N = 700 kN, M3 = 40 kNm, M2 = 30 kNm constant. Table B.1: kzy = 0.6 kyy, plastic set;
/// the expected values are written out from lambda, chi (curve a, 0.21) and the factors; M_b,Rd is
/// taken from a run under M3 alone.
#[test]
fn test7b_box_member_equation_governs_and_is_named() {
    let sec = make_box_tube("B250", "A992Fy50", 250.0, 200.0, 10.0, 10.0);
    let len = 7000.0;
    let (n, m3, m2) = (700.0e3, 40.0e6, 30.0e6);
    let mut alone = member(
        EC3,
        sec.clone(),
        len,
        stations(len, 0.0, [m3; 3], [0.0; 3]),
        Some(ow_k1()),
    );
    let m_b = design(&mut alone).phi_mn33;
    let lam_y = lambda(sec.area, sec.i33, len);
    let lam_z = lambda(sec.area, sec.i22, len);
    let (chi_y, chi_z) = (chi(lam_y, 0.21), chi(lam_z, 0.21));
    let n_pl = sec.area * FY;
    let (n_y, n_z) = (n / (chi_y * n_pl), n / (chi_z * n_pl));
    let k_yy = (1.0 + (lam_y - 0.2) * n_y).min(1.0 + 0.8 * n_y);
    let k_zz = (1.0 + (lam_z - 0.2) * n_z).min(1.0 + 0.8 * n_z);
    let (k_yz, k_zy) = (0.6 * k_zz, 0.6 * k_yy);
    let (m_y, m_z) = (m3 / m_b, m2 / (sec.z22 * FY));
    let eq61 = n_y + k_yy * m_y + k_yz * m_z;
    let eq62 = n_z + k_zy * m_y + k_zz * m_z;
    let section = n / n_pl + m3 / (sec.z33 * FY) + m2 / (sec.z22 * FY);
    let (want, name) = if eq61 >= eq62 {
        (eq61, "EC3-6.3.3(4) eq 6.61")
    } else {
        (eq62, "EC3-6.3.3(4) eq 6.62")
    };
    assert!(want > section + 0.02, "{want} against section {section}");
    let mut model = member(
        EC3,
        sec,
        len,
        stations(len, -n, [m3; 3], [m2; 3]),
        Some(ow_k1()),
    );
    let r = design(&mut model);
    assert_rel(r.total_ratio, want, 1e-6, "box member total");
    assert_eq!(r.eq_name, name);
    assert_eq!(r.notes.len(), 1);
}

// ------------------------------------------------------------------ what stays refused / unchanged

/// A channel with compression and a moment is refused by name (Annex B has columns for I and
/// rectangular hollow sections only); the message names the member, the combination, the station
/// and the section.
#[test]
fn channel_with_compression_and_a_moment_is_refused_by_name() {
    let mut sec = i300();
    sec.prop_type = FramePropType::Channel;
    sec.name = "CH".into();
    sec.t2 = 150.0; // flange (150 - 11)/19 = 7.3: Class 1
    let mut model = member(
        EC3,
        sec.clone(),
        6000.0,
        stations(6000.0, -N_ED, [0.0; 3], [100.0e6; 3]),
        Some(ow_k1()),
    );
    let text = try_run_steel_design(&mut model).unwrap_err().to_string();
    assert!(
        text.contains("'M1'") && text.contains("combination 'LC'") && text.contains("Annex B")
            && text.contains("'CH'") && text.contains("Channel"),
        "{text}"
    );
    assert!(model.steel_design_results.is_empty());
    // the same channel without the moment is designed
    let mut model = member(
        EC3,
        sec,
        6000.0,
        stations(6000.0, -N_ED, [0.0; 3], [0.0; 3]),
        Some(ow_k1()),
    );
    try_run_steel_design(&mut model).unwrap();
}

/// Tension with bending, compression alone and bending alone: the results of the code before this
/// change, pinned bitwise (the values were printed by a run on the unchanged code):
/// I 300 x 300, S355, L = 6000, default overwrites, stations at 0, 3000, 6000.
#[test]
fn stations_without_compression_and_moment_keep_their_results_bitwise() {
    let sta = |p: f64, m3: [f64; 3], m2: [f64; 3]| stations(6000.0, p, m3, m2);
    // tension 1500 kN, M3 = 300, 100, -100 kNm, M2 = 50, 20, 10 kNm
    let mut m = member(
        EC3,
        i300(),
        6000.0,
        sta(1.5e6, [3e8, 1e8, -1e8], [5e7, 2e7, 1e7]),
        None,
    );
    let r = design(&mut m);
    assert_eq!(r.total_ratio, 0.9760852391471677);
    assert_eq!(r.pr_ratio, 0.29585156929534073);
    assert_eq!(r.mr_major_ratio, 0.5170155584128496);
    assert_eq!(r.mr_minor_ratio, 0.1632181114389773);
    assert_eq!(r.eq_name, "EC3-6.2");
    assert_eq!(r.phi_pnc, 2694499.9421515577);
    assert_eq!(r.phi_mn33, 580253331.1008073);
    assert!(r.notes.is_empty());
    // compression alone
    let mut m = member(EC3, i300(), 6000.0, sta(-1.5e6, [0.0; 3], [0.0; 3]), None);
    let r = design(&mut m);
    assert_eq!(r.total_ratio, 0.5566895647443401);
    assert_eq!(r.eq_name, "EC3-6.2");
    assert_eq!(r.phi_mn33, 511563692.92532533);
    // bending alone
    let mut m = member(
        EC3,
        i300(),
        6000.0,
        sta(0.0, [3e8, 1e8, -1e8], [5e7, 2e7, 1e7]),
        None,
    );
    let r = design(&mut m);
    assert_eq!(r.total_ratio, 0.680233669851827);
    assert_eq!(r.mr_major_ratio, 0.5170155584128496);
    assert_eq!(r.eq_name, "EC3-6.2");
    // AISC on the member of test 1 (compression 1500 kN, M2 = 100 kNm): H1-1a, unchanged
    let mut m = member(
        "AISC 360-10",
        i300(),
        6000.0,
        sta(-1.5e6, [0.0; 3], [1e8; 3]),
        None,
    );
    let r = design(&mut m);
    assert_eq!(r.total_ratio, 0.8282238282168681);
    assert_eq!(r.eq_name, "H1-1a");
    assert_eq!(r.phi_pnc, 2965495.3808277044);
}

/// A Class 4 member (slender web in compression) with a moment stays refused by the Class 4 gate.
#[test]
fn class_4_with_compression_and_a_moment_stays_refused() {
    // web 400 x 3: c/t = 133 >> 42 eps
    let sec = make_i_section("SL", "A992Fy50", 406.0, 200.0, 3.0, 3.0, 200.0, 3.0);
    let mut model = member(
        EC3,
        sec,
        3000.0,
        stations(3000.0, -50.0e3, [0.0; 3], [1.0e6; 3]),
        Some(ow_k1()),
    );
    let text = try_run_steel_design(&mut model).unwrap_err().to_string();
    assert!(text.contains("Class 4"), "{text}");
}

#[allow(dead_code)]
fn _unused(_: Ec3InteractionFactors) {}
