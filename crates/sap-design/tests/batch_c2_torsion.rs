//! Batch C2: torsion in concrete beam design (ledger D130, G85, D148).
//!
//! References: ACI 318-14 CSI manual pp. 3-42 to 3-46 (torsion of rectangular beams, T-beam properties on
//! p. 3-44) and Eurocode 2-2004 CSI manual pp. 3-32 to 3-37 (v24 manual pp. 3-36 and 3-41 for the
//! "or" of the strut clause). Every expected value is a hand calculation written out in the test
//! with its units; none is read from a function of the crate. No vendor example covers torsion.
use sap_design::concrete::{design_aci_beam_torsion, design_ec2_beam_torsion};
use sap_design::{try_run_concrete_design, DesignError};
use sap_model::{
    CaseResults, ComboType, ConcreteDesignResult, FramePropType, FrameStationResult, LoadCase,
    LoadCombo, LoadPattern, LoadPatternType, Material, SapModel,
};
use sap_section::make_rectangular;
use sap_units::Units;

fn close(a: f64, b: f64, rel: f64, what: &str) {
    assert!(
        (a - b).abs() <= rel * b.abs().max(1e-300),
        "{what}: got {a}, expected {b} (relative tolerance {rel})"
    );
}

/// Compares with a number the brief prints, at the brief's tolerance of 1e-6 relative (0 must
/// be 0). Four printed numbers cannot meet it, because the brief rounds or cuts them to fewer
/// digits (Av/s 0.005710 for 0.00570953, At/s 0.0322577 for 0.03225776, Asl 0.00071 for
/// 0.00071407); those are not asserted here, only the closed forms at 1e-9 (see the report).
fn printed(got: f64, brief: &str, what: &str) {
    let want: f64 = brief.parse().unwrap();
    if want == 0.0 {
        assert_eq!(got, 0.0, "{what}");
    } else {
        close(got, want, 1e-6, what);
    }
}

fn stn(rel: f64, p: f64, v2: f64, t: f64) -> FrameStationResult {
    FrameStationResult {
        station: rel * 120.0,
        rel_dist: rel,
        p,
        v2,
        t,
        ..Default::default()
    }
}

/// kip, in; f'c = 4 ksi, fy = fys = 60 ksi, default cover 2.5 in.
fn aci_model(
    section: sap_model::FrameSection,
    vertical: bool,
    stations: Vec<FrameStationResult>,
) -> SapModel {
    let mut m = SapModel::new(Units::KipInF);
    m.concrete_design_code = "ACI 318-14".into();
    let mut concrete = Material::default_concrete_kip_in();
    concrete.name = "C4".into();
    concrete.fc = 4.0;
    concrete.fy = 60.0;
    concrete.fys = 60.0;
    m.materials.insert("C4".into(), concrete);
    m.frame_sections.insert(section.name.clone(), section.clone());
    m.add_point("1", 0.0, 0.0, 0.0);
    if vertical {
        m.add_point("2", 0.0, 0.0, 120.0);
    } else {
        m.add_point("2", 120.0, 0.0, 0.0);
    }
    m.add_frame("B1", "1", "2", &section.name);
    let mut cr = CaseResults {
        case_name: "COMBO1".into(),
        ..Default::default()
    };
    cr.frame_forces.insert("B1".into(), stations);
    m.case_results.insert("COMBO1".into(), cr);
    m
}

fn aci_beam(tu: f64, pu: f64) -> Result<ConcreteDesignResult, DesignError> {
    // station p = -Pu: SAP2000 has compression negative
    let mut m = aci_model(
        make_rectangular("B", "C4", 24.0, 12.0),
        false,
        vec![stn(0.5, -pu, 30.0, tu)],
    );
    try_run_concrete_design(&mut m)?;
    Ok(m.concrete_design_results["B1"].clone())
}

/// N, mm; fck 30 MPa, fy = fys = 500 MPa, default cover 63.5 mm (d = 436.5 for t3 = 500).
fn ec2_model(
    section: sap_model::FrameSection,
    vertical: bool,
    stations: Vec<FrameStationResult>,
) -> SapModel {
    let mut m = SapModel::new(Units::NMmC);
    m.concrete_design_code = "Eurocode 2-2004".into();
    let mut concrete = Material::default_concrete_kip_in();
    concrete.name = "C30".into();
    concrete.fc = 30.0;
    concrete.fy = 500.0;
    concrete.fys = 500.0;
    concrete.e = 33_000.0;
    m.materials.insert("C30".into(), concrete);
    m.frame_sections.insert(section.name.clone(), section.clone());
    m.add_point("1", 0.0, 0.0, 0.0);
    if vertical {
        m.add_point("2", 0.0, 0.0, 3000.0);
    } else {
        m.add_point("2", 3000.0, 0.0, 0.0);
    }
    m.add_frame("B1", "1", "2", &section.name);
    let mut cr = CaseResults {
        case_name: "COMBO1".into(),
        ..Default::default()
    };
    cr.frame_forces.insert("B1".into(), stations);
    m.case_results.insert("COMBO1".into(), cr);
    m
}

fn ec2_beam(stations: Vec<FrameStationResult>) -> Result<ConcreteDesignResult, DesignError> {
    let mut m = ec2_model(make_rectangular("B", "C30", 500.0, 300.0), false, stations);
    try_run_concrete_design(&mut m)?;
    Ok(m.concrete_design_results["B1"].clone())
}

fn same_bits(a: &ConcreteDesignResult, b: &ConcreteDesignResult) {
    for (x, y, what) in [
        (a.as_top, b.as_top, "as_top"),
        (a.as_bot, b.as_bot, "as_bot"),
        (a.as_side, b.as_side, "as_side"),
        (a.av_over_s, b.av_over_s, "av_over_s"),
        (a.at_over_s, b.at_over_s, "at_over_s"),
        (a.al_torsion, b.al_torsion, "al_torsion"),
        (a.min_rebar_area, b.min_rebar_area, "min_rebar_area"),
    ] {
        assert_eq!(x.to_bits(), y.to_bits(), "{what}: {x} vs {y}");
    }
    assert_eq!(a.status, b.status);
    assert_eq!(a.notes, b.notes);
}

// ------------------------------------------------------------------ step 4: ACI 318-14

/// Hand calculation of the manual's rules (pp. 3-44 to 3-46) for b = 12, h = 24, d = 21.5 in,
/// f'c = 4 ksi, fy = fys = 60 ksi; returns (at/s, Al, Av/s, overstressed, ignored).
fn aci_hand(tu: f64, vu: f64, pu: f64) -> (f64, f64, f64, bool, bool) {
    let (b, h, d, phi) = (12.0_f64, 24.0_f64, 21.5_f64, 0.75_f64);
    let sq = 4000.0_f64.sqrt() / 1000.0; // sqrt(f'c) in ksi
    let c = 1.75;
    let acp = b * h;
    let pcp = 2.0 * (b + h);
    let aoh = (b - 2.0 * c) * (h - 2.0 * c);
    let ph = 2.0 * (b - 2.0 * c) + 2.0 * (h - 2.0 * c);
    let ao = 0.85 * aoh;
    let root = (1.0 + pu / (4.0 * acp * sq)).max(0.0).sqrt();
    let tth = sq * acp * acp / pcp * root;
    let tth0 = sq * acp * acp / pcp;
    let vc = 2.0 * sq * b * d;
    let strength = if vu > phi * vc { (vu - phi * vc) / (phi * 60.0 * d) } else { 0.0 };
    let tmin = (0.75 * sq * b / 60.0).max(0.05 * b / 60.0);
    if tu <= (phi * tth).max(1e-6 * phi * tth0) {
        let today = if vu <= 0.5 * phi * vc { 0.0 } else { strength.max(tmin) };
        return (0.0, 0.0, today, false, true);
    }
    let (at, al_req) = if tu <= 4.0 * phi * tth {
        (0.0, 0.0)
    } else {
        (tu / (phi * 2.0 * ao * 60.0), tu * ph / (phi * 2.0 * ao * 60.0))
    };
    let a = 5.0 * sq * acp / 60.0 - at * ph;
    let bb = 5.0 * sq * acp / 60.0 - (0.025 * b / 60.0) * ph;
    let al = al_req.max(a.min(bb).max(0.0));
    let av = strength.max(tmin - 2.0 * at).max(0.0);
    let lhs = ((vu / (b * d)).powi(2) + (tu * ph / (1.7 * aoh * aoh)).powi(2)).sqrt();
    let rhs = phi * (vc / (b * d) + 8.0 * sq);
    (at, al, av, lhs > rhs, false)
}

#[test]
fn aci_torsion_rows_match_the_hand_calculation_and_the_brief_numbers() {
    // sqrt(f'c) = 63.2456 psi; Acp = 288 in2, pcp = 72 in, Aoh = 174.25 in2, ph = 58 in,
    // Ao = 148.1125 in2; Tth = 0.0632456 x 288^2/72 = 72.858877 kip-in, phi Tth = 54.644158,
    // phi Tcr = 218.576632; Vc = 2 x 0.0632456 x 12 x 21.5 = 32.6347 kip, phi Vc = 24.476029
    // transverse minimum max(0.75 x 63.2456, 50) x 12/60000 = 0.010000 in2/in.
    // (Tu, at/s, Al, Av/s, status) from the brief's table (six digits)
    let rows = [
        (100.0, "0", "1.227893", Some("0.010000"), "OK"),
        (400.0, "0.0300072", "1.740419", None, "OK"),
        (430.0, "", "1.870950", None, "OVERSTRESSED"),
    ];
    for (tu, at, al, av, status) in rows {
        let r = aci_beam(tu, 0.0).unwrap();
        if !at.is_empty() {
            printed(r.at_over_s, at, "at/s digits");
        }
        printed(r.al_torsion, al, "Al digits");
        if let Some(av) = av {
            printed(r.av_over_s, av, "Av/s digits");
        }
        assert_eq!(r.status, status, "Tu = {tu}");
        // exactly the same closed form
        let (hat, hal, hav, hover, ignored) = aci_hand(tu, 30.0, 0.0);
        assert!(!ignored);
        assert_eq!(r.status == "OVERSTRESSED", hover);
        if hat == 0.0 {
            assert_eq!(r.at_over_s, 0.0);
        } else {
            close(r.at_over_s, hat, 1e-9, "at/s");
        }
        close(r.al_torsion, hal, 1e-9, "Al");
        close(r.av_over_s, hav, 1e-9, "Av/s");
    }
}

#[test]
fn aci_torsion_below_phi_tth_is_bitwise_as_without_torsion() {
    // phi Tth = 54.644158 kip-in > 50: ignored; Av/s = 0.01 (Vu = 30 > phi Vc, minimum governs)
    let with = aci_beam(50.0, 0.0).unwrap();
    let without = aci_beam(0.0, 0.0).unwrap();
    same_bits(&with, &without);
    assert_eq!(with.at_over_s, 0.0);
    assert_eq!(with.al_torsion, 0.0);
    close(with.av_over_s, 0.010000, 1e-9, "Av/s today");
}

#[test]
fn aci_axial_force_moves_the_threshold_and_tension_removes_it() {
    // Pu = 100 kip compression: 1 + 100/(4 x 288 x 0.0632456) = 2.372516, Tth = 112.224346,
    // phi Tth = 84.168259: Tu = 50 is still ignored, Tu = 100 is above it (minimum only).
    let r50 = aci_beam(50.0, 100.0).unwrap();
    assert_eq!((r50.at_over_s, r50.al_torsion), (0.0, 0.0));
    let r100 = aci_beam(100.0, 100.0).unwrap();
    printed(r100.al_torsion, "1.227893", "Al, compression");
    assert_eq!(r100.at_over_s, 0.0);
    close(r100.av_over_s, 0.010000, 1e-9, "Av/s, compression");
    // Pu = -100 (tension): 1 + (-100)/(4 x 288 x 0.0632456) = -0.372516 < 0, Tth = 0: the
    // strength regime applies: At/s = 100/(0.75 x 2 x 148.1125 x 60) = 0.0075018,
    // Al(a) = 1.517893 - 0.0075018 x 58 = 1.082789 governs the minimum, Al = max(0.435105, 1.082789)
    let rt = aci_beam(100.0, -100.0).unwrap();
    printed(rt.at_over_s, "0.0075018", "At/s, tension");
    printed(rt.al_torsion, "1.082789", "Al, tension");
    let (hat, hal, hav, _, _) = aci_hand(100.0, 30.0, -100.0);
    close(rt.at_over_s, hat, 1e-9, "At/s, tension (formula)");
    close(rt.al_torsion, hal, 1e-9, "Al, tension (formula)");
    close(rt.av_over_s, hav, 1e-9, "Av/s, tension (formula)");
    // the noise floor 1e-6 phi Tth0 = 5.46e-5 kip-in: a roundoff torsion with tension is ignored
    let tiny = aci_beam(1e-6, -100.0).unwrap();
    let none = aci_beam(0.0, -100.0).unwrap();
    same_bits(&tiny, &none);
}

#[test]
fn aci_torsion_function_regimes_and_the_section_limit() {
    let u = Units::KipInF;
    let ignored = design_aci_beam_torsion(50.0, 30.0, 0.0, 12.0, 24.0, 21.5, 4.0, 60.0, 60.0, u).unwrap();
    assert!(ignored.ignored);
    // Tu = 430: sqrt((30/258)^2 + (430 x 58/(1.7 x 174.25^2))^2) = 0.496967 > 0.75 (32.6347/258 + 8 x 0.0632456) = 0.474342
    let over = design_aci_beam_torsion(430.0, 30.0, 0.0, 12.0, 24.0, 21.5, 4.0, 60.0, 60.0, u).unwrap();
    assert!(over.section_overstressed);
    close(over.transverse_min, 0.010000, 1e-9, "transverse min");
    let ok = design_aci_beam_torsion(400.0, 30.0, 0.0, 12.0, 24.0, 21.5, 4.0, 60.0, 60.0, u).unwrap();
    assert!(!ok.section_overstressed);
    // a section too small for the closed stirrup at 1.75 in: b = 3 in <= 2c = 3.5 in
    let e = design_aci_beam_torsion(100.0, 30.0, 0.0, 3.0, 24.0, 21.5, 4.0, 60.0, 60.0, u).unwrap_err();
    assert!(e.contains("does not fit"), "{e}");
}

#[test]
fn aci_beam_that_is_too_small_for_the_stirrup_is_refused_naming_the_member() {
    let mut m = aci_model(
        make_rectangular("B", "C4", 24.0, 3.0),
        false,
        vec![stn(0.5, 0.0, 5.0, 40.0)],
    );
    let e = try_run_concrete_design(&mut m).unwrap_err();
    assert!(matches!(e, DesignError::ConcreteTorsionNotDesigned { .. }), "{e}");
    let text = e.to_string();
    assert!(text.contains("'B1'") && text.contains("'B'") && text.contains("COMBO1"), "{text}");
    assert!(text.contains("does not fit"), "{text}");
    // a refused member stops the run and leaves no results (D127)
    assert!(m.concrete_design_results.is_empty());
}

#[test]
fn nonrectangular_beams_are_refused_even_without_torsion() {
    // The previous checks accepted a T beam using its 36 in flange as the shear width,
    // although its web is only 12 in wide. The other unimplemented geometries also used
    // their bounding rectangle (or essentially zero width when dimensions were absent).
    for kind in [FramePropType::T, FramePropType::I, FramePropType::General, FramePropType::Variable] {
        for t in [0.0, 70.0] {
            let mut section = make_rectangular("B", "C4", 24.0, 36.0);
            section.prop_type = kind;
            section.tf = 5.0;
            section.tw = 12.0;
            let mut model = aci_model(section, false, vec![stn(0.5, 0.0, 130.0, t)]);
            let err = try_run_concrete_design(&mut model).unwrap_err();
            assert!(matches!(err, DesignError::UnsupportedConcreteBeamSection { .. }));
            let text = err.to_string();
            assert!(text.contains("'B1'") && text.contains("'B'") && text.contains(&format!("{kind:?}")), "{text}");
            assert!(model.concrete_design_results.is_empty());
        }
    }

    let mut section = make_rectangular("B", "C30", 500.0, 900.0);
    section.prop_type = FramePropType::T;
    section.tf = 120.0;
    section.tw = 300.0;
    let mut model = ec2_model(section, false, vec![stn(0.5, 0.0, 20e3, 0.0)]);
    assert!(matches!(
        try_run_concrete_design(&mut model),
        Err(DesignError::UnsupportedConcreteBeamSection { .. })
    ));
}

// ------------------------------------------------------------------ step 3: columns

#[test]
fn a_column_with_torsion_is_designed_as_before_and_gets_one_note() {
    let col = |t: f64| {
        let mut m = aci_model(
            make_rectangular("C", "C4", 24.0, 24.0),
            true,
            vec![stn(0.5, -300.0, 20.0, t)],
        );
        try_run_concrete_design(&mut m).unwrap();
        m.concrete_design_results["B1"].clone()
    };
    let with = col(80.0);
    let without = col(0.0);
    assert!(with.is_column);
    assert_eq!(with.notes.len(), 1, "{:?}", with.notes);
    assert!(with.notes[0].contains("torsion T is not designed for columns (ACI 318-14 manual"));
    assert!(without.notes.is_empty());
    let mut stripped = with.clone();
    stripped.notes.clear();
    same_bits(&stripped, &without);
    assert_eq!(with.at_over_s, 0.0);
    // EC2 column: the same with the other manual
    let col2 = |t: f64| {
        let mut m = ec2_model(
            make_rectangular("C", "C30", 400.0, 400.0),
            true,
            vec![stn(0.5, -500e3, 20e3, t)],
        );
        try_run_concrete_design(&mut m).unwrap();
        m.concrete_design_results["B1"].clone()
    };
    let w2 = col2(30e6);
    assert_eq!(w2.notes.len(), 1);
    assert!(w2.notes[0].contains("Eurocode 2-2004 manual"));
    assert!(col2(0.0).notes.is_empty());
}

// ------------------------------------------------------------------ step 5: Eurocode 2

/// Hand numbers of the 500 x 300 mm beam, fck 30 MPa, fyk = fywk 500 MPa, d = 436.5 mm, rho_l = 0.
struct Ec2Hand {
    vrdc: f64,
    trdc: f64,
    tef: f64,
    ak: f64,
    uk: f64,
    z: f64,
    fywd: f64,
    fcd: f64,
    nu: f64,
}

fn ec2_hand() -> Ec2Hand {
    let (b, h, d) = (300.0_f64, 500.0_f64, 436.5_f64);
    let fck = 30.0_f64;
    let tef = b * h / (2.0 * (b + h));
    let ak = (b - tef) * (h - tef);
    let uk = 2.0 * (b - tef) + 2.0 * (h - tef);
    let fctm = 0.30 * fck.powf(2.0 / 3.0);
    let fctd = 1.0 * 0.7 * fctm / 1.5;
    let k = (1.0 + (200.0_f64 / d).sqrt()).min(2.0);
    let vmin = 0.035 * k.powf(1.5) * fck.sqrt();
    Ec2Hand {
        vrdc: vmin * b * d,
        trdc: fctd * tef * 2.0 * ak,
        tef,
        ak,
        uk,
        z: 0.9 * d,
        fywd: 500.0 / 1.15,
        fcd: 0.85 * fck / 1.5,
        nu: 0.6 * (1.0 - fck / 250.0),
    }
}

/// (At/s, Asl, Asw/s at 45 degrees, Eq. 6.29 sum) for torsion that Eq. 6.31 does not ignore.
fn ec2_hand_station(v: f64, t: f64) -> (f64, f64, f64, f64) {
    let e = ec2_hand();
    let tcon = (1.0 - v / e.vrdc) * e.trdc;
    let vt = 2.0 * (500.0 - e.tef) * (t - tcon) / (2.0 * e.ak);
    let at = vt / (e.z * e.fywd);
    let asl = t * e.uk / (2.0 * e.ak * (500.0 / 1.15));
    let asw_min = 0.08 * 30.0_f64.sqrt() / 500.0 * 300.0;
    let asw = (v / (e.z * e.fywd)).max(asw_min);
    let trdmax = 2.0 * e.nu * e.fcd * e.ak * e.tef * 0.5;
    let vrdmax = 300.0 * e.z * e.nu * e.fcd / 2.0;
    (at, asl, asw, t / trdmax + v / vrdmax)
}

#[test]
fn ec2_hand_intermediates_match_the_brief() {
    let e = ec2_hand();
    close(e.tef, 93.75, 1e-12, "tef");
    close(e.ak, 83789.0625, 1e-12, "Ak");
    close(e.uk, 1225.0, 1e-12, "uk");
    close(e.trdc, 21235580.727, 1e-9, "TRd,c");
    close(e.vrdc, 54512.2275, 1e-8, "VRd,c");
}

#[test]
fn ec2_torsion_cases_a_b_e_f_g_follow_the_printed_rules() {
    let e = ec2_hand();
    // (case, V, T, Eq. 6.31 sum, At/s, Asl, Asw/s, overstressed)
    let cases = [
        ("A", 0.5 * e.vrdc, 15e6, 1.206362, "0.1243942", "252.19580", "0.262907", false),
        ("B", 1.5 * e.vrdc, 1e6, 1.547091, "0.3297848", "16.81305", "0.478725", false),
        ("E", 1.5 * e.vrdc, 2e-6 * e.trdc, 1.500002, "0.3013998", "", "0.478725", false),
        ("F", 0.5 * e.vrdc, 70e6, 3.796354, "1.6856347", "1176.91375", "0.262907", true),
        ("G", 4.0 * e.vrdc, 20e6, 4.941816, "2.3761156", "336.26107", "1.276600", false),
    ];
    for (case, v, t, sum, at, asl, asw, over) in cases {
        // the printed Eq. 6.31 sum
        close(t / e.trdc + v / e.vrdc, sum, 1e-6, case);
        let r = ec2_beam(vec![stn(0.5, 0.0, v, t)]).unwrap();
        printed(r.at_over_s, at, &format!("{case} At/s digits"));
        if !asl.is_empty() {
            printed(r.al_torsion, asl, &format!("{case} Asl digits"));
        }
        printed(r.av_over_s, asw, &format!("{case} Asw/s digits"));
        assert_eq!(r.status == "OVERSTRESSED", over, "{case}");
        // the closed form written in this file
        let (hat, hasl, hasw, s629) = ec2_hand_station(v, t);
        close(r.at_over_s, hat, 1e-9, &format!("{case} At/s"));
        close(r.al_torsion, hasl, 1e-9, &format!("{case} Asl"));
        close(r.av_over_s, hasw, 1e-9, &format!("{case} Asw/s"));
        assert_eq!(s629 > 1.0, over, "{case}: Eq. 6.29 sum {s629}");
    }
    // Case A: the 6.29 sum is 0.264271 (OK); case F: 1.044319; case B: 0.168774
    close(ec2_hand_station(0.5 * e.vrdc, 15e6).3, 0.264271, 1e-5, "A 6.29");
    close(ec2_hand_station(0.5 * e.vrdc, 70e6).3, 1.044319, 1e-5, "F 6.29");
    close(ec2_hand_station(1.5 * e.vrdc, 1e6).3, 0.168774, 1e-4, "B 6.29");
}

#[test]
fn ec2_torsion_that_eq_6_31_ignores_leaves_every_number_bitwise() {
    let e = ec2_hand();
    // C: V = 0.5 VRd,c, T = 1e6 (sum 0.547): ignored. D: V = 1.5 VRd,c, T = 0.5e-6 TRd,c: below the floor.
    let none = ec2_beam(vec![stn(0.5, 0.0, 0.5 * e.vrdc, 0.0)]).unwrap();
    same_bits(&ec2_beam(vec![stn(0.5, 0.0, 0.5 * e.vrdc, 1e6)]).unwrap(), &none);
    let none_d = ec2_beam(vec![stn(0.5, 0.0, 1.5 * e.vrdc, 0.0)]).unwrap();
    same_bits(
        &ec2_beam(vec![stn(0.5, 0.0, 1.5 * e.vrdc, 0.5e-6 * e.trdc)]).unwrap(),
        &none_d,
    );
    // today's number there: cot 2.5, V/(z fywd cot) = 0.191490 < the minimum 0.262907
    close(none_d.av_over_s, 0.262907, 1e-6, "Asw/s min today");
    // D148, case E: just above the floor the printed rule gives the 45 degree strut
    let above = ec2_beam(vec![stn(0.5, 0.0, 1.5 * e.vrdc, 2e-6 * e.trdc)]).unwrap();
    assert!(above.at_over_s > 0.3 && above.av_over_s > none_d.av_over_s);
}

#[test]
fn ec2_the_largest_values_over_the_stations_are_reported() {
    let e = ec2_hand();
    let a = ec2_beam(vec![stn(0.0, 0.0, 0.5 * e.vrdc, 15e6)]).unwrap();
    let ac = ec2_beam(vec![
        stn(0.0, 0.0, 0.5 * e.vrdc, 15e6),
        stn(1.0, 0.0, 0.5 * e.vrdc, 1e6),
    ])
    .unwrap();
    same_bits(&a, &ac);
    close(ac.at_over_s, 0.1243942, 1e-6, "At/s of case A");
}

#[test]
fn ec2_torsion_function_and_the_seismic_strut() {
    let e = ec2_hand();
    let t = design_ec2_beam_torsion(
        15e6, 0.5 * e.vrdc, 300.0, 500.0, 436.5, 0.0, 30.0, 500.0, 500.0, Units::NMmC,
    );
    assert!(!t.ignored);
    // Tcon = 0.5 x 21 235 580.727 = 10 617 790.364 N mm; Vt = 2 x (500 - 93.75) x (15e6 - Tcon)/(2 x 83789.0625)
    close(t.at_over_s, 0.1243942, 1e-6, "At/s");
    let ign = design_ec2_beam_torsion(
        1e6, 0.5 * e.vrdc, 300.0, 500.0, 436.5, 0.0, 30.0, 500.0, 500.0, Units::NMmC,
    );
    assert!(ign.ignored && ign.at_over_s == 0.0 && ign.asl == 0.0);
    // case B in a seismic combination: 45 degrees either way, so the same Asw/s
    let b_plain = ec2_beam(vec![stn(0.5, 0.0, 1.5 * e.vrdc, 1e6)]).unwrap();
    let mut m = ec2_model(
        make_rectangular("B", "C30", 500.0, 300.0),
        false,
        vec![stn(0.5, 0.0, 1.5 * e.vrdc, 1e6)],
    );
    let pat = |name: &str, t| LoadPattern { name: name.into(), pat_type: t, self_wt_multiplier: 0.0 };
    m.load_patterns.insert("QK".into(), pat("QK", LoadPatternType::Quake));
    m.load_cases.insert("CASE1".into(), LoadCase::linear_static("CASE1", "QK", 1.0));
    m.load_combos.insert(
        "COMBO1".into(),
        LoadCombo { name: "COMBO1".into(), combo_type: ComboType::LinearAdd, cases: vec![("CASE1".into(), 1.0)] },
    );
    try_run_concrete_design(&mut m).unwrap();
    let seis = &m.concrete_design_results["B1"];
    assert_eq!(seis.av_over_s.to_bits(), b_plain.av_over_s.to_bits());
    close(seis.av_over_s, 0.478725, 1e-6, "Asw/s 45 degrees");
}

// A model-level check of both synthesis and the stored maximum/minimum rows used after analysis.
#[test]
fn range_add_pairs_opposite_signed_shear_and_torsion_extrema() {
    let e = ec2_hand();
    let mut models = [
        (aci_model(make_rectangular("B", "C4", 24.0, 12.0), false, vec![]), 70.0, 400.0),
        (ec2_model(make_rectangular("B", "C30", 500.0, 300.0), false, vec![]), 4.0 * e.vrdc, 45e6),
    ];
    for (model, v, t) in &mut models {
        for sign in [1.0, -1.0] {
            for (name, station) in [
                ("V", stn(0.5, 0.0, *v, 0.0)),
                ("T", stn(0.5, 0.0, 0.0, sign * *t)),
            ] {
                let mut case = CaseResults { case_name: name.into(), ..Default::default() };
                case.frame_forces.insert("B1".into(), vec![station]);
                model.case_results.insert(name.into(), case);
            }
            model.case_results.shift_remove("COMBO1");
            model.load_combos.insert("BOTH".into(), LoadCombo {
                name: "BOTH".into(),
                combo_type: ComboType::LinearAdd,
                cases: vec![("V".into(), 1.0), ("T".into(), 1.0)],
            });
            try_run_concrete_design(model).unwrap();
            let linear = model.concrete_design_results["B1"].clone();
            assert_eq!(linear.status, "OVERSTRESSED");
            model.load_combos.get_mut("BOTH").unwrap().combo_type = ComboType::RangeAdd;
            try_run_concrete_design(model).unwrap();
            same_bits(&linear, &model.concrete_design_results["B1"]);

            let mut hi = CaseResults { case_name: "BOTH".into(), ..Default::default() };
            let mut lo = CaseResults { case_name: "BOTH".into(), ..Default::default() };
            hi.frame_forces.insert("B1".into(), vec![stn(0.5, 0.0, *v, (sign * *t).max(0.0))]);
            lo.frame_forces.insert("B1".into(), vec![stn(0.5, 0.0, 0.0, (sign * *t).min(0.0))]);
            hi.combo_min = Some(Box::new(lo));
            model.case_results.insert("BOTH".into(), hi);
            try_run_concrete_design(model).unwrap();
            same_bits(&linear, &model.concrete_design_results["B1"]);
            model.case_results.shift_remove("BOTH");
        }
    }
}

#[test]
fn ranged_torsion_keeps_the_axial_and_flexural_companions_that_activate_it() {
    // ACI: 50 kip-in is ignored at zero axial load, but tension eliminates phi Tth.
    let mut aci = aci_model(
        make_rectangular("B", "C4", 24.0, 12.0),
        false,
        vec![stn(0.5, 100.0, 0.0, 0.0)],
    );
    // EC2: the positive moment increases rho_l enough to make Eq. 6.31 pass at
    // 60 kN and 5 kN-m. The zero-M3 branch of the same range fails Eq. 6.31.
    let mut hi_ec2 = stn(0.5, 0.0, 60e3, 5e6);
    hi_ec2.m3 = 250e6;
    let mut ec2 = ec2_model(
        make_rectangular("B", "C30", 500.0, 300.0),
        false,
        vec![hi_ec2],
    );
    for (model, lo_station) in [
        (&mut aci, stn(0.5, 0.0, 0.0, -50.0)),
        (&mut ec2, stn(0.5, 0.0, 0.0, 0.0)),
    ] {
        try_run_concrete_design(model).unwrap();
        assert_eq!(model.concrete_design_results["B1"].at_over_s, 0.0);
        let mut lo = CaseResults::default();
        lo.frame_forces.insert("B1".into(), vec![lo_station]);
        model.case_results.get_mut("COMBO1").unwrap().combo_min = Some(Box::new(lo));
        model.load_combos.insert("COMBO1".into(), LoadCombo {
            name: "COMBO1".into(),
            combo_type: ComboType::RangeAdd,
            cases: Vec::new(),
        });
        try_run_concrete_design(model).unwrap();
        assert!(model.concrete_design_results["B1"].at_over_s > 0.0);
    }
}

#[test]
fn torsion_failure_governs_over_a_passing_flexure_combination() {
    let mut flexure = stn(0.5, 0.0, 0.0, 0.0);
    flexure.m3 = 4100.0;
    let mut model = aci_model(
        make_rectangular("B", "C4", 24.0, 12.0),
        false,
        vec![flexure],
    );
    try_run_concrete_design(&mut model).unwrap();
    assert_eq!(model.concrete_design_results["B1"].status, "OK");
    let mut torsion = CaseResults { case_name: "TORSION".into(), ..Default::default() };
    // The torsion-only case exceeds the section limit with an area metric of about
    // 4.08 in², below the passing flexure case's 4.11 in².
    torsion.frame_forces.insert("B1".into(), vec![stn(0.5, 0.0, 0.0, 425.0)]);
    model.case_results.insert("TORSION".into(), torsion);
    try_run_concrete_design(&mut model).unwrap();
    let result = &model.concrete_design_results["B1"];
    assert_eq!(result.status, "OVERSTRESSED");
    assert_eq!(result.governing_combo, "TORSION");
}
