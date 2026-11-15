//! Steel design workflow: default design combinations, Auto Select lists,
//! the design choice and the two verification commands.
//!
//! Sources: CSI Steel Frame Design Manual AISC 360-10 (`sfd_aisc_360_10.pdf`):
//! section 2.1 pp. 2-1/2-2 (Auto Select), section 2.4 p. 2-4 and section 3.2
//! pp. 3-6 to 3-8 (default combinations), Appendix E (analysis versus design
//! sections), table B-1 (D/C ratio limit 0.95). Units are kip, in, ksi.

use sap_design::{
    add_default_design_combos, try_run_steel_design, verify_all_members_passed,
    verify_analysis_vs_design, DefaultComboOptions, DesignMethod,
};
use sap_model::auto_select::{median_member, members_by_weight, resolve_analysis_sections};
use sap_model::{
    CaseResults, FramePropType, FrameSection, FrameStationResult, LoadCase, LoadCaseType,
    LoadPattern, LoadPatternType, MatType, SapModel, SteelDesignOverwrite,
};
use sap_units::Units;

// ---------------------------------------------------------------------------
// Default combinations
// ---------------------------------------------------------------------------

fn add_pattern(model: &mut SapModel, name: &str, kind: LoadPatternType) {
    model.load_patterns.insert(
        name.into(),
        LoadPattern {
            name: name.into(),
            pat_type: kind,
            self_wt_multiplier: 0.0,
        },
    );
    model
        .load_cases
        .insert(name.into(), LoadCase::linear_static(name, name, 1.0));
}

type Expected<'a> = Vec<Vec<(&'a str, f64)>>;

fn assert_combos(model: &SapModel, expected: &Expected, what: &str) {
    let names: Vec<&String> = model.load_combos.keys().collect();
    assert_eq!(
        names.len(),
        expected.len(),
        "{what}: combination count, got {names:?}"
    );
    for (i, (name, combo)) in model.load_combos.iter().enumerate() {
        assert_eq!(name, &format!("DSTL{}", i + 1), "{what}: name");
        let want = &expected[i];
        assert_eq!(
            combo.cases.len(),
            want.len(),
            "{what}: {name} has {:?}",
            combo.cases
        );
        for ((got_case, got_f), (want_case, want_f)) in combo.cases.iter().zip(want) {
            assert_eq!(got_case, want_case, "{what}: {name} case");
            assert!(
                (got_f - want_f).abs() < 1e-12,
                "{what}: {name} {got_case} factor {got_f} != {want_f}"
            );
        }
    }
}

/// Model with dead, super dead, live, static wind, static quake and a
/// response-spectrum case (and a modal case, which is skipped).
fn combo_model() -> SapModel {
    let mut m = SapModel::new(Units::KipInF);
    add_pattern(&mut m, "DEAD", LoadPatternType::Dead);
    add_pattern(&mut m, "SDEAD", LoadPatternType::SuperDead);
    add_pattern(&mut m, "LIVE", LoadPatternType::Live);
    add_pattern(&mut m, "WIND", LoadPatternType::Wind);
    add_pattern(&mut m, "EQ", LoadPatternType::Quake);
    let mut rs = LoadCase::linear_static("SPEC", "EQ", 1.0);
    rs.case_type = LoadCaseType::ResponseSpectrum;
    m.load_cases.insert("SPEC".into(), rs);
    m.load_cases
        .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 6));
    m
}

#[test]
fn lrfd_default_combinations_match_the_manual_rows() {
    let mut m = combo_model();
    let mut opts = DefaultComboOptions::new(DesignMethod::Lrfd);
    opts.rho = 1.3;
    opts.sds = 0.82;
    let summary = add_default_design_combos(&mut m, &opts).unwrap();
    assert_eq!(summary.skipped.len(), 1);
    assert_eq!(summary.skipped[0].0, "MODAL");

    // Factors typed from the manual (pp. 3-6 and 3-8) with rho = 1.3, SDS = 0.82.
    // Terms of absent classes (RL, SL) are left out, rows that become equal are
    // listed once, rows without their lateral case are not generated:
    //   1.4 DL                                      p. 3-6  (ASCE 2.3.2-1)
    //   1.2 DL + 1.6 LL (+0.5 RL / +0.5 SL rows)    p. 3-6  (ASCE 2.3.2-2)
    //   1.2 DL + 1.0 LL (from the 1.6 RL / 1.6 SL rows)     (ASCE 2.3.2-3)
    //   0.9 DL +- 1.0 WL                            p. 3-6  (ASCE 2.3.2-6)
    //   1.2 DL +- 0.5 WL (from the 1.6 RL / 1.6 SL +- 0.5 WL rows)
    //   1.2 DL + 1.0 LL +- 1.0 WL (from the 0.5 RL / 0.5 SL rows)
    //   (0.9 - 0.2 SDS) DL +- rho QE = 0.736 DL +- 1.3 QE          p. 3-8
    //   (1.2 + 0.2 SDS) DL +- rho QE = 1.364 DL +- 1.3 QE          p. 3-8
    //   (1.2 + 0.2 SDS) DL +- rho QE + 1.0 LL                      p. 3-8
    // Dead and super dead cases are additive (p. 2-4). The response-spectrum
    // case has a minus row too: it supplies the minima, which the manual
    // (p. 2-5) takes as the negative of the maxima.
    let expected: Expected = vec![
        vec![("DEAD", 1.4), ("SDEAD", 1.4)],
        vec![("DEAD", 1.2), ("SDEAD", 1.2), ("LIVE", 1.6)],
        vec![("DEAD", 1.2), ("SDEAD", 1.2), ("LIVE", 1.0)],
        vec![("DEAD", 0.9), ("SDEAD", 0.9), ("WIND", 1.0)],
        vec![("DEAD", 0.9), ("SDEAD", 0.9), ("WIND", -1.0)],
        vec![("DEAD", 1.2), ("SDEAD", 1.2), ("WIND", 0.5)],
        vec![("DEAD", 1.2), ("SDEAD", 1.2), ("WIND", -0.5)],
        vec![("DEAD", 1.2), ("SDEAD", 1.2), ("LIVE", 1.0), ("WIND", 1.0)],
        vec![("DEAD", 1.2), ("SDEAD", 1.2), ("LIVE", 1.0), ("WIND", -1.0)],
        vec![("DEAD", 0.736), ("SDEAD", 0.736), ("EQ", 1.3)],
        vec![("DEAD", 0.736), ("SDEAD", 0.736), ("EQ", -1.3)],
        vec![("DEAD", 0.736), ("SDEAD", 0.736), ("SPEC", 1.3)],
        vec![("DEAD", 0.736), ("SDEAD", 0.736), ("SPEC", -1.3)],
        vec![("DEAD", 1.364), ("SDEAD", 1.364), ("EQ", 1.3)],
        vec![("DEAD", 1.364), ("SDEAD", 1.364), ("EQ", -1.3)],
        vec![("DEAD", 1.364), ("SDEAD", 1.364), ("SPEC", 1.3)],
        vec![("DEAD", 1.364), ("SDEAD", 1.364), ("SPEC", -1.3)],
        vec![
            ("DEAD", 1.364),
            ("SDEAD", 1.364),
            ("EQ", 1.3),
            ("LIVE", 1.0),
        ],
        vec![
            ("DEAD", 1.364),
            ("SDEAD", 1.364),
            ("EQ", -1.3),
            ("LIVE", 1.0),
        ],
        vec![
            ("DEAD", 1.364),
            ("SDEAD", 1.364),
            ("SPEC", 1.3),
            ("LIVE", 1.0),
        ],
        vec![
            ("DEAD", 1.364),
            ("SDEAD", 1.364),
            ("SPEC", -1.3),
            ("LIVE", 1.0),
        ],
    ];
    assert_combos(&m, &expected, "LRFD");
    assert_eq!(summary.created.len(), 21);
}

#[test]
fn asd_default_combinations_match_the_manual_rows() {
    let mut m = combo_model();
    let mut opts = DefaultComboOptions::new(DesignMethod::Asd);
    opts.rho = 1.3;
    opts.sds = 0.82;
    add_default_design_combos(&mut m, &opts).unwrap();

    // Manual pp. 3-6, 3-7, 3-8, rho = 1.3, SDS = 0.82 (terms of absent RL / SL
    // left out, equal rows listed once):
    //   1.0 DL                                         (ASCE 2.4.1-1)
    //   1.0 DL + 1.0 LL                                (ASCE 2.4.1-2)
    //   1.0 DL + 0.75 LL (from the 0.75 RL / SL rows)
    //   1.0 DL +- 0.6 WL                               (ASCE 2.4.1-5)
    //   1.0 DL + 0.75 LL +- 0.75(0.6 WL) = 0.45 WL     (ASCE 2.4.1-6a)
    //   0.6 DL +- 0.6 WL                               (ASCE 2.4.1-7)
    //   (1.0 + 0.14 SDS) DL +- 0.7 rho QE = 1.1148 DL +- 0.91 QE   (2.4.1-5)
    //   (1.0 + 0.105 SDS) DL +- 0.75(0.7 rho) QE + 0.75 LL
    //        = 1.0861 DL +- 0.6825 QE + 0.75 LL                    (2.4.1-6b)
    //   (0.6 - 0.14 SDS) DL +- 0.7 rho QE = 0.4852 DL +- 0.91 QE   (2.4.1-8)
    let d = |f: f64| vec![("DEAD", f), ("SDEAD", f)];
    let with = |mut v: Vec<(&'static str, f64)>, extra: Vec<(&'static str, f64)>| {
        v.extend(extra);
        v
    };
    let expected: Expected = vec![
        d(1.0),
        with(d(1.0), vec![("LIVE", 1.0)]),
        with(d(1.0), vec![("LIVE", 0.75)]),
        with(d(1.0), vec![("WIND", 0.6)]),
        with(d(1.0), vec![("WIND", -0.6)]),
        with(d(1.0), vec![("LIVE", 0.75), ("WIND", 0.45)]),
        with(d(1.0), vec![("LIVE", 0.75), ("WIND", -0.45)]),
        with(d(0.6), vec![("WIND", 0.6)]),
        with(d(0.6), vec![("WIND", -0.6)]),
        with(d(1.1148), vec![("EQ", 0.91)]),
        with(d(1.1148), vec![("EQ", -0.91)]),
        with(d(1.1148), vec![("SPEC", 0.91)]),
        with(d(1.1148), vec![("SPEC", -0.91)]),
        with(d(1.0861), vec![("EQ", 0.6825), ("LIVE", 0.75)]),
        with(d(1.0861), vec![("EQ", -0.6825), ("LIVE", 0.75)]),
        with(d(1.0861), vec![("SPEC", 0.6825), ("LIVE", 0.75)]),
        with(d(1.0861), vec![("SPEC", -0.6825), ("LIVE", 0.75)]),
        with(d(0.4852), vec![("EQ", 0.91)]),
        with(d(0.4852), vec![("EQ", -0.91)]),
        with(d(0.4852), vec![("SPEC", 0.91)]),
        with(d(0.4852), vec![("SPEC", -0.91)]),
    ];
    assert_combos(&m, &expected, "ASD");
}

#[test]
fn roof_live_and_snow_rows_are_generated_when_those_cases_exist() {
    let mut m = SapModel::new(Units::KipInF);
    add_pattern(&mut m, "DEAD", LoadPatternType::Dead);
    add_pattern(&mut m, "LIVE", LoadPatternType::ReduceLive);
    add_pattern(&mut m, "ROOF", LoadPatternType::Rooflive);
    add_pattern(&mut m, "SNOW", LoadPatternType::Snow);
    add_pattern(&mut m, "WIND", LoadPatternType::Wind);

    add_default_design_combos(&mut m, &DefaultComboOptions::new(DesignMethod::Lrfd)).unwrap();
    // Manual p. 3-6, LRFD rows in the order printed.
    let expected: Expected = vec![
        vec![("DEAD", 1.4)],
        vec![("DEAD", 1.2), ("LIVE", 1.6), ("ROOF", 0.5)],
        vec![("DEAD", 1.2), ("LIVE", 1.0), ("ROOF", 1.6)],
        vec![("DEAD", 1.2), ("LIVE", 1.6), ("SNOW", 0.5)],
        vec![("DEAD", 1.2), ("LIVE", 1.0), ("SNOW", 1.6)],
        vec![("DEAD", 0.9), ("WIND", 1.0)],
        vec![("DEAD", 0.9), ("WIND", -1.0)],
        vec![("DEAD", 1.2), ("ROOF", 1.6), ("WIND", 0.5)],
        vec![("DEAD", 1.2), ("ROOF", 1.6), ("WIND", -0.5)],
        vec![("DEAD", 1.2), ("LIVE", 1.0), ("ROOF", 0.5), ("WIND", 1.0)],
        vec![("DEAD", 1.2), ("LIVE", 1.0), ("ROOF", 0.5), ("WIND", -1.0)],
        vec![("DEAD", 1.2), ("SNOW", 1.6), ("WIND", 0.5)],
        vec![("DEAD", 1.2), ("SNOW", 1.6), ("WIND", -0.5)],
        vec![("DEAD", 1.2), ("LIVE", 1.0), ("SNOW", 0.5), ("WIND", 1.0)],
        vec![("DEAD", 1.2), ("LIVE", 1.0), ("SNOW", 0.5), ("WIND", -1.0)],
    ];
    assert_combos(&m, &expected, "LRFD roof/snow");

    let mut opts = DefaultComboOptions::new(DesignMethod::Asd);
    opts.replace = true;
    add_default_design_combos(&mut m, &opts).unwrap();
    // Manual pp. 3-6 and 3-7, ASD rows in the order printed.
    let expected: Expected = vec![
        vec![("DEAD", 1.0)],
        vec![("DEAD", 1.0), ("LIVE", 1.0)],
        vec![("DEAD", 1.0), ("ROOF", 1.0)],
        vec![("DEAD", 1.0), ("LIVE", 0.75), ("ROOF", 0.75)],
        vec![("DEAD", 1.0), ("SNOW", 1.0)],
        vec![("DEAD", 1.0), ("LIVE", 0.75), ("SNOW", 0.75)],
        vec![("DEAD", 1.0), ("WIND", 0.6)],
        vec![("DEAD", 1.0), ("WIND", -0.6)],
        vec![
            ("DEAD", 1.0),
            ("LIVE", 0.75),
            ("ROOF", 0.75),
            ("WIND", 0.45),
        ],
        vec![
            ("DEAD", 1.0),
            ("LIVE", 0.75),
            ("ROOF", 0.75),
            ("WIND", -0.45),
        ],
        vec![
            ("DEAD", 1.0),
            ("LIVE", 0.75),
            ("SNOW", 0.75),
            ("WIND", 0.45),
        ],
        vec![
            ("DEAD", 1.0),
            ("LIVE", 0.75),
            ("SNOW", 0.75),
            ("WIND", -0.45),
        ],
        vec![("DEAD", 0.6), ("WIND", 0.6)],
        vec![("DEAD", 0.6), ("WIND", -0.6)],
    ];
    assert_combos(&m, &expected, "ASD roof/snow");
}

#[test]
fn dead_and_live_only_give_the_gravity_rows() {
    let mut m = SapModel::new(Units::KipInF);
    add_pattern(&mut m, "DEAD", LoadPatternType::Dead);
    add_pattern(&mut m, "LIVE", LoadPatternType::Live);
    add_default_design_combos(&mut m, &DefaultComboOptions::new(DesignMethod::Lrfd)).unwrap();
    // 1.4 DL; 1.2 DL + 1.6 LL (manual p. 3-6); 1.2 DL + 1.0 LL (the row printed
    // with 1.6 RL / 1.6 SL, whose roof live / snow term is left out).
    let expected: Expected = vec![
        vec![("DEAD", 1.4)],
        vec![("DEAD", 1.2), ("LIVE", 1.6)],
        vec![("DEAD", 1.2), ("LIVE", 1.0)],
    ];
    assert_combos(&m, &expected, "gravity only");
}

#[test]
fn default_combination_refusals() {
    let lrfd = |sds: f64| {
        let mut o = DefaultComboOptions::new(DesignMethod::Lrfd);
        o.sds = sds;
        o
    };
    let refused = |m: &mut SapModel, o: &DefaultComboOptions, needle: &str| {
        let before = m.load_combos.clone();
        let err = match add_default_design_combos(m, o) {
            Err(e) => e.to_string(),
            Ok(s) => panic!("expected a refusal mentioning '{needle}', got {s:?}"),
        };
        assert!(err.contains(needle), "'{err}' should mention '{needle}'");
        assert_eq!(m.load_combos, before, "a refusal must not change the model");
    };

    // A case mixing classes.
    let mut m = combo_model();
    m.load_cases
        .get_mut("DEAD")
        .unwrap()
        .loads
        .push(sap_model::LoadCaseAssignment {
            load_name: "LIVE".into(),
            scale_factor: 1.0,
        });
    refused(&mut m, &lrfd(0.8), "mixes load classes");

    // Notional load pattern.
    let mut m = combo_model();
    add_pattern(&mut m, "NOTL", LoadPatternType::Notional);
    refused(&mut m, &lrfd(0.8), "notional");

    // A pattern type the manual's rule does not cover.
    let mut m = combo_model();
    add_pattern(&mut m, "TEMP", LoadPatternType::Temperature);
    refused(&mut m, &lrfd(0.8), "TEMP");

    // A static case applying something that is not a pattern.
    let mut m = combo_model();
    m.load_cases.get_mut("LIVE").unwrap().loads[0].load_name = "Accel UX".into();
    refused(&mut m, &lrfd(0.8), "Accel UX");

    // A static case without loads.
    let mut m = combo_model();
    m.load_cases.get_mut("LIVE").unwrap().loads.clear();
    refused(&mut m, &lrfd(0.8), "LIVE");

    // SDS missing or not positive with a seismic case.
    let mut m = combo_model();
    refused(&mut m, &lrfd(0.0), "SDS");
    refused(&mut m, &lrfd(f64::NAN), "SDS");
    // ... but not needed without one.
    let mut m = SapModel::new(Units::KipInF);
    add_pattern(&mut m, "DEAD", LoadPatternType::Dead);
    assert!(add_default_design_combos(&mut m, &lrfd(0.0)).is_ok());

    // Bad rho.
    let mut m = combo_model();
    let mut o = lrfd(0.8);
    o.rho = 0.0;
    refused(&mut m, &o, "rho");

    // Existing DSTL combinations without replace; replace removes all of them.
    let mut m = combo_model();
    add_default_design_combos(&mut m, &lrfd(0.8)).unwrap();
    refused(&mut m, &lrfd(0.8), "DSTL1");
    let mut o = lrfd(0.8);
    o.replace = true;
    let summary = add_default_design_combos(&mut m, &o).unwrap();
    assert_eq!(summary.removed.len(), 21);
    assert_eq!(m.load_combos.len(), 21);

    // A case named like a generated combination.
    let mut m = combo_model();
    add_pattern(&mut m, "DSTL2", LoadPatternType::Dead);
    refused(&mut m, &lrfd(0.8), "DSTL2");

    // Nothing to combine.
    let mut m = SapModel::new(Units::KipInF);
    m.load_cases.clear();
    refused(&mut m, &lrfd(0.8), "no combination");

    // Locked model.
    let mut m = combo_model();
    m.is_locked = true;
    refused(&mut m, &lrfd(0.8), "locked");

    // Eurocode 3 is not an AISC method.
    assert!(DesignMethod::from_steel_code("Eurocode 3-2005").is_err());
    assert_eq!(
        DesignMethod::from_steel_code("AISC 360-10 ASD").unwrap(),
        DesignMethod::Asd
    );
}

// ---------------------------------------------------------------------------
// Auto Select
// ---------------------------------------------------------------------------

const FY: f64 = 50.0; // ksi, A992Fy50 of `SapModel::new`

/// Rolled W shapes with hand-typed properties (AISC shapes database values to
/// three digits): name, d, bf, tf, tw, A, Ix, Zx, Sx, Iy, Zy, Sy, ry, J.
#[allow(clippy::type_complexity)]
const SHAPES: [(
    &str,
    f64,
    f64,
    f64,
    f64,
    f64,
    f64,
    f64,
    f64,
    f64,
    f64,
    f64,
    f64,
    f64,
); 4] = [
    (
        "W14X22", 13.7, 5.00, 0.335, 0.230, 6.49, 199.0, 33.2, 29.0, 7.00, 3.87, 2.80, 1.04, 0.208,
    ),
    (
        "W16X26", 15.7, 5.50, 0.345, 0.250, 7.68, 301.0, 44.2, 38.4, 9.59, 5.48, 3.49, 1.12, 0.262,
    ),
    (
        "W16X31", 15.9, 5.53, 0.440, 0.275, 9.13, 375.0, 54.0, 47.2, 12.4, 7.03, 4.49, 1.17, 0.461,
    ),
    (
        "W18X35", 17.7, 6.00, 0.425, 0.300, 10.3, 510.0, 66.5, 57.6, 15.3, 8.06, 5.12, 1.22, 0.506,
    ),
];

fn w_section(i: usize) -> FrameSection {
    let (name, d, bf, tf, tw, a, ix, zx, sx, iy, zy, sy, ry, j) = SHAPES[i];
    FrameSection {
        name: name.into(),
        material: "A992Fy50".into(),
        prop_type: FramePropType::I,
        t3: d,
        t2: bf,
        tf,
        tw,
        t2b: bf,
        tfb: tf,
        area: a,
        i33: ix,
        i22: iy,
        z33: zx,
        s33: sx,
        z22: zy,
        s22: sy,
        r33: (ix / a).sqrt(),
        r22: ry,
        torsional_j: j,
        as2: 0.0,
        as3: 0.0,
        ..FrameSection::default()
    }
}

const SPAN: f64 = 240.0; // in (20 ft)

/// Simply supported beam along X with the Auto Select list "LIST" of all four
/// shapes (listed out of weight order) and the forces of a uniform factored
/// load `w` (kip/in) entered as station results (closed form).
fn beam_model(w: f64) -> SapModel {
    let mut m = SapModel::new(Units::KipInF);
    for i in [2, 0, 3, 1] {
        let s = w_section(i);
        m.frame_sections.insert(s.name.clone(), s);
    }
    let order: Vec<String> = [2, 0, 3, 1]
        .iter()
        .map(|&i| SHAPES[i].0.to_string())
        .collect();
    m.define_auto_select_list("LIST", &order).unwrap();
    m.add_point("A", 0.0, 0.0, 0.0);
    m.add_point("B", SPAN, 0.0, 0.0);
    m.add_frame("BM", "A", "B", "LIST");
    m.assign_auto_select_list("LIST", &["BM".to_string()])
        .unwrap();
    // Fully braced: unbraced length 1 % of the span, far below Lp.
    m.frames.get_mut("BM").unwrap().steel_overwrite = Some(SteelDesignOverwrite {
        l_ratio_major: Some(0.01),
        l_ratio_minor: Some(0.01),
        l_ratio_ltb: Some(0.01),
        k_major: Some(1.0),
        k_minor: Some(1.0),
        cb: Some(1.0),
        is_sway: false,
    });
    let mut cr = CaseResults {
        case_name: "DEAD".into(),
        ..Default::default()
    };
    let n = 8;
    let stations = (0..=n)
        .map(|k| {
            let x = SPAN * k as f64 / n as f64;
            FrameStationResult {
                station: x,
                rel_dist: k as f64 / n as f64,
                v2: w * (SPAN / 2.0 - x),
                m3: w * x * (SPAN - x) / 2.0,
                ..Default::default()
            }
        })
        .collect();
    cr.frame_forces.insert("BM".into(), stations);
    m.case_results.insert("DEAD".into(), cr);
    m.is_locked = true;
    m
}

fn phi_mn(i: usize) -> f64 {
    0.9 * FY * SHAPES[i].7 // 0.9 Fy Zx (kip-in)
}

#[test]
fn median_rule_for_odd_and_even_lists() {
    let mut m = SapModel::new(Units::KipInF);
    for i in 0..4 {
        let s = w_section(i);
        m.frame_sections.insert(s.name.clone(), s);
    }
    // Weights per length ascend W14X22 < W16X26 < W16X31 < W18X35 (areas 6.49 < 7.68 < 9.13 < 10.3).
    // Odd count (3, listed out of order): the middle one by weight.
    m.define_auto_select_list("ODD", &["W18X35".into(), "W14X22".into(), "W16X31".into()])
        .unwrap();
    assert_eq!(median_member(&m, "ODD").unwrap(), "W16X31");

    // Assignment starts analysis and design at the median (manual appendix E).
    m.add_point("A", 0.0, 0.0, 0.0);
    m.add_point("B", 100.0, 0.0, 0.0);
    m.add_frame("F", "A", "B", "ODD");
    m.assign_auto_select_list("ODD", &["F".into()]).unwrap();
    let state = m.frames["F"].auto_select.clone().unwrap();
    assert_eq!(state.design, "W16X31");
    assert_eq!(state.analysis.as_deref(), Some("W16X31"));
    // A working copy resolves to the design section.
    let mut working = m.clone();
    resolve_analysis_sections(&mut working).unwrap();
    assert_eq!(working.frames["F"].section, "W16X31");
    assert_eq!(m.frames["F"].section, "ODD");

    // Even count (4): the lower of the two middle members.
    m.define_auto_select_list(
        "EVEN",
        &[
            "W16X31".into(),
            "W18X35".into(),
            "W14X22".into(),
            "W16X26".into(),
        ],
    )
    .unwrap();
    assert_eq!(median_member(&m, "EVEN").unwrap(), "W16X26");
    // One member.
    m.define_auto_select_list("ONE", &["W18X35".into()])
        .unwrap();
    assert_eq!(median_member(&m, "ONE").unwrap(), "W18X35");
    // Ties keep list order: two sections of equal weight.
    let mut twin = w_section(1);
    twin.name = "TWIN".into();
    m.frame_sections.insert("TWIN".into(), twin);
    m.define_auto_select_list("TIE", &["TWIN".into(), "W16X26".into(), "W18X35".into()])
        .unwrap();
    let order: Vec<String> = members_by_weight(&m, "TIE")
        .unwrap()
        .iter()
        .map(|s| s.name.clone())
        .collect();
    assert_eq!(order, ["TWIN", "W16X26", "W18X35"]);
    // The weight is area times unit weight: reversing the material weight of
    // one section reorders the list.
    let mut heavy = m.materials["A992Fy50"].clone();
    heavy.name = "HEAVY".into();
    heavy.unit_weight *= 2.0;
    m.materials.insert("HEAVY".into(), heavy);
    m.frame_sections.get_mut("W14X22").unwrap().material = "HEAVY".into();
    // 6.49 * 2 = 12.98 > 10.3 now.
    m.define_auto_select_list("MAT", &["W14X22".into(), "W16X31".into(), "W18X35".into()])
        .unwrap();
    assert_eq!(median_member(&m, "MAT").unwrap(), "W18X35");
}

#[test]
fn design_chooses_the_lightest_section_with_ratio_within_the_limit() {
    // Mu = w L^2 / 8 = 1950 kip-in with w = 0.2708333 kip/in (factored).
    let mu = 1950.0;
    let w = 8.0 * mu / (SPAN * SPAN);
    let mut m = beam_model(w);
    assert!(m.steel_design_prefs.dc_ratio_limit == 0.95);
    try_run_steel_design(&mut m).unwrap();

    // Expected: the lightest of the four with Mu / (0.9 Fy Zx) <= 0.95,
    // evaluated here from the typed section properties.
    let mut order: Vec<usize> = (0..4).collect();
    order.sort_by(|&a, &b| SHAPES[a].5.total_cmp(&SHAPES[b].5)); // by area = weight
    let expected = order
        .iter()
        .copied()
        .find(|&i| mu / phi_mn(i) <= 0.95)
        .unwrap();
    assert_eq!(SHAPES[expected].0, "W16X31");
    // W16X26 has a ratio between 0.95 and 1.0: it would pass the status test
    // at 1.0 but not the D/C limit.
    let r26 = mu / phi_mn(1);
    assert!(r26 > 0.95 && r26 < 1.0, "{r26}");

    let result = &m.steel_design_results["BM"];
    assert_eq!(result.section_name, SHAPES[expected].0);
    assert_eq!(result.status, "OK");
    // Flexure governs: ratio = Mu / (0.9 Fy Zx) within 1e-9.
    let want = mu / phi_mn(expected);
    assert!(
        (result.total_ratio - want).abs() < 1e-9,
        "{} vs {}",
        result.total_ratio,
        want
    );
    assert!((result.mr_major_ratio - want).abs() < 1e-9);
    // Shear does not govern: even with phi_v = 0.9 the ratio Vu / (phi 0.6 Fy d tw) is far below.
    let (d, tw) = (SHAPES[expected].1, SHAPES[expected].4);
    let vu = w * SPAN / 2.0;
    let shear_bound = vu / (0.9 * 0.6 * FY * d * tw);
    assert!(shear_bound < 0.5 * want, "shear bound {shear_bound}");
    assert!(result.shear_major_ratio <= shear_bound + 1e-12);
    assert!((result.mr_major_ratio - result.total_ratio).abs() < 1e-12);
    // The chosen section is stored as the design section.
    let state = m.frames["BM"].auto_select.clone().unwrap();
    assert_eq!(state.design, "W16X31");
    // The capacity used was the plastic moment of the chosen section.
    assert!((result.phi_mn33 - phi_mn(expected)).abs() < 1e-9);
}

#[test]
fn a_lower_limit_selects_a_heavier_section() {
    let mu = 1950.0;
    let w = 8.0 * mu / (SPAN * SPAN);
    let mut m = beam_model(w);
    m.is_locked = false;
    // Limit 0.75: W16X31 (0.8025) fails, W18X35 (0.6516) passes.
    m.set_dc_ratio_limit(0.75).unwrap();
    m.is_locked = true;
    try_run_steel_design(&mut m).unwrap();
    assert_eq!(m.steel_design_results["BM"].section_name, "W18X35");
    assert!((m.steel_design_results["BM"].total_ratio - mu / phi_mn(3)).abs() < 1e-9);
}

#[test]
fn verify_commands_report_mismatch_and_failures() {
    let mu = 1950.0;
    let w = 8.0 * mu / (SPAN * SPAN);
    let mut m = beam_model(w);
    // Analysis used the median (W16X26); design is still the median: no mismatch yet.
    assert!(verify_analysis_vs_design(&m).unwrap().is_empty());
    try_run_steel_design(&mut m).unwrap();
    assert_eq!(
        m.frames["BM"].auto_select.as_ref().unwrap().design,
        "W16X31"
    );
    assert_eq!(
        m.frames["BM"]
            .auto_select
            .as_ref()
            .unwrap()
            .analysis
            .as_deref(),
        Some("W16X26")
    );
    assert_eq!(verify_analysis_vs_design(&m).unwrap(), ["BM"]);
    assert!(verify_all_members_passed(&m).unwrap().is_empty());

    // Tightening the limit after design: W16X31 now has ratio 0.80 > 0.75.
    let mut tight = m.clone();
    tight.is_locked = false;
    tight.set_dc_ratio_limit(0.75).unwrap();
    // Altering a preference deletes the design results (appendix E), so verification refuses.
    tight.is_locked = true;
    assert!(verify_all_members_passed(&tight).is_err());
}

#[test]
fn a_ratio_between_the_limit_and_one_is_reported_by_verify_although_status_is_ok() {
    // W16X26 assigned directly: ratio 1950 / 1989 = 0.9804 is above the 0.95
    // limit but not above 1.0, so the status string stays "OK" (threshold 1.0).
    let mu = 1950.0;
    let mut m = beam_model(8.0 * mu / (SPAN * SPAN));
    m.frames.get_mut("BM").unwrap().section = "W16X26".into();
    m.frames.get_mut("BM").unwrap().auto_select = None;
    try_run_steel_design(&mut m).unwrap();
    let r = &m.steel_design_results["BM"];
    assert_eq!(r.status, "OK");
    assert!((r.total_ratio - mu / phi_mn(1)).abs() < 1e-9);
    assert_eq!(verify_all_members_passed(&m).unwrap(), ["BM"]);
}

#[test]
fn no_section_passes_so_the_heaviest_is_chosen_and_flagged() {
    for mu in [3500.0, 2900.0] {
        // 3500 > phi Mn of every section (heaviest 2992.5): ratio > 1.
        // 2900: heaviest ratio 0.969 is within 1.0 but above the 0.95 limit.
        let w = 8.0 * mu / (SPAN * SPAN);
        let mut m = beam_model(w);
        try_run_steel_design(&mut m).unwrap();
        let r = &m.steel_design_results["BM"];
        assert_eq!(r.section_name, "W18X35", "Mu = {mu}");
        assert_eq!(r.status, "OVERSTRESSED", "Mu = {mu}");
        assert!((r.total_ratio - mu / phi_mn(3)).abs() < 1e-9);
        assert_eq!(
            m.frames["BM"].auto_select.as_ref().unwrap().design,
            "W18X35"
        );
        assert_eq!(verify_all_members_passed(&m).unwrap(), ["BM"]);
    }
}

#[test]
fn frames_without_auto_select_lists_are_unchanged() {
    // The same frame with an individually assigned section gives the result a
    // list of that one section gives.
    let mu = 1950.0;
    let w = 8.0 * mu / (SPAN * SPAN);
    let mut single = beam_model(w);
    single.frames.get_mut("BM").unwrap().section = "W16X31".into();
    single.frames.get_mut("BM").unwrap().auto_select = None;
    try_run_steel_design(&mut single).unwrap();
    let mut listed = beam_model(w);
    listed.is_locked = false;
    listed
        .define_auto_select_list("ONE", &["W16X31".into()])
        .unwrap();
    listed
        .assign_auto_select_list("ONE", &["BM".into()])
        .unwrap();
    listed.is_locked = true;
    try_run_steel_design(&mut listed).unwrap();
    assert_eq!(
        single.steel_design_results["BM"],
        listed.steel_design_results["BM"]
    );
}

#[test]
fn auto_select_refusals() {
    let base = || {
        let mut m = SapModel::new(Units::KipInF);
        for i in 0..3 {
            let s = w_section(i);
            m.frame_sections.insert(s.name.clone(), s);
        }
        m.add_point("A", 0.0, 0.0, 0.0);
        m.add_point("B", 100.0, 0.0, 0.0);
        m.add_frame("F", "A", "B", "W16X26");
        m
    };
    let err = |r: Result<(), String>, needle: &str| {
        let e = r.unwrap_err();
        assert!(e.contains(needle), "'{e}' should mention '{needle}'");
    };

    // Empty list.
    let mut m = base();
    err(m.define_auto_select_list("L", &[]), "empty");
    // Missing member.
    err(m.define_auto_select_list("L", &["W99X99".into()]), "W99X99");
    // Non-steel member.
    let conc = sap_section::make_rectangular("CONC", "4000Psi", 12.0, 12.0);
    m.frame_sections.insert("CONC".into(), conc);
    err(
        m.define_auto_select_list("L", &["W16X26".into(), "CONC".into()]),
        "non-steel",
    );
    // Nested list.
    m.define_auto_select_list("INNER", &["W16X26".into()])
        .unwrap();
    err(
        m.define_auto_select_list("OUTER", &["INNER".into()]),
        "nested",
    );
    // A list cannot take the name of an ordinary section.
    err(
        m.define_auto_select_list("W16X26", &["W14X22".into()]),
        "not an Auto Select",
    );
    // A failed definition leaves the model unchanged.
    assert!(!m.frame_sections.contains_key("L"));
    assert!(!m.frame_sections.contains_key("OUTER"));
    // Assignment: unknown frame, empty selection, unknown list.
    err(m.assign_auto_select_list("INNER", &["NOPE".into()]), "NOPE");
    err(m.assign_auto_select_list("INNER", &[]), "no frames");
    err(
        m.assign_auto_select_list("W16X26", &["F".into()]),
        "not an Auto Select",
    );
    assert_eq!(m.frames["F"].section, "W16X26");
    // Locked model.
    m.is_locked = true;
    err(
        m.define_auto_select_list("L2", &["W16X26".into()]),
        "locked",
    );
    err(m.assign_auto_select_list("INNER", &["F".into()]), "locked");
    m.is_locked = false;

    // Analysis-time resolution refuses an empty list, a missing member and a
    // design section that is not in the list.
    let mut m = base();
    m.define_auto_select_list("L", &["W16X26".into(), "W14X22".into()])
        .unwrap();
    m.assign_auto_select_list("L", &["F".into()]).unwrap();
    let mut w = m.clone();
    w.frame_sections
        .get_mut("L")
        .unwrap()
        .auto_select_list
        .clear();
    err(resolve_analysis_sections(&mut w), "empty");
    let mut w = m.clone();
    w.frame_sections
        .get_mut("L")
        .unwrap()
        .auto_select_list
        .push("GONE".into());
    err(resolve_analysis_sections(&mut w), "GONE");
    let mut w = m.clone();
    w.frames
        .get_mut("F")
        .unwrap()
        .auto_select
        .as_mut()
        .unwrap()
        .design = "W18X35".into();
    err(resolve_analysis_sections(&mut w), "W18X35");
    let mut w = m.clone();
    w.frame_sections.get_mut("W14X22").unwrap().material = "Missing".into();
    err(resolve_analysis_sections(&mut w), "material");
    let mut w = m.clone();
    let mut nested = w_section(0);
    nested.name = "NEST".into();
    nested.prop_type = FramePropType::Auto;
    w.frame_sections.insert("NEST".into(), nested);
    w.frame_sections
        .get_mut("L")
        .unwrap()
        .auto_select_list
        .push("NEST".into());
    err(resolve_analysis_sections(&mut w), "nested");
    // Non-steel material on a member.
    let mut w = m.clone();
    let mut mat = w.materials["A992Fy50"].clone();
    mat.name = "CONCM".into();
    mat.mat_type = MatType::Concrete;
    w.materials.insert("CONCM".into(), mat);
    w.frame_sections.get_mut("W14X22").unwrap().material = "CONCM".into();
    err(resolve_analysis_sections(&mut w), "non-steel");

    // D/C limit outside (0, 1].
    let mut m = base();
    for bad in [0.0, -0.5, 1.01, f64::NAN, f64::INFINITY] {
        assert!(m.set_dc_ratio_limit(bad).is_err(), "{bad}");
    }
    assert_eq!(m.steel_design_prefs.dc_ratio_limit, 0.95);
    m.set_dc_ratio_limit(1.0).unwrap();

    // Verification needs a locked model with results; "passed" needs design results.
    let mut m = base();
    assert!(verify_analysis_vs_design(&m).is_err());
    assert!(verify_all_members_passed(&m).is_err());
    m.is_locked = true;
    assert!(
        verify_analysis_vs_design(&m).is_err(),
        "no analysis results"
    );
    m.case_results.insert("X".into(), CaseResults::default());
    assert!(verify_analysis_vs_design(&m).unwrap().is_empty());
    assert!(verify_all_members_passed(&m).is_err(), "no design results");
}

#[test]
fn design_refuses_a_broken_list_and_leaves_no_results() {
    let mut m = beam_model(0.27);
    m.steel_design_results.insert(
        "STALE".into(),
        sap_model::SteelDesignResult {
            frame_name: "STALE".into(),
            section_name: "X".into(),
            design_code: String::new(),
            governing_combo: String::new(),
            governing_station: 0.0,
            total_ratio: 0.0,
            pr_ratio: 0.0,
            mr_major_ratio: 0.0,
            mr_minor_ratio: 0.0,
            shear_major_ratio: 0.0,
            shear_minor_ratio: 0.0,
            phi_pnc: 0.0,
            phi_pnt: 0.0,
            phi_mn33: 0.0,
            phi_mn22: 0.0,
            phi_vn2: 0.0,
            phi_vn3: 0.0,
            status: "OK".into(),
            eq_name: String::new(),
        },
    );
    m.frame_sections
        .get_mut("LIST")
        .unwrap()
        .auto_select_list
        .push("GONE".into());
    let err = try_run_steel_design(&mut m).unwrap_err();
    assert!(
        matches!(err, sap_design::DesignError::Workflow(_)),
        "{err:?}"
    );
    assert!(err.to_string().contains("GONE"), "{err}");
    assert!(m.steel_design_results.is_empty());
    assert_eq!(
        m.frames["BM"].auto_select.as_ref().unwrap().design,
        "W16X26"
    );
}
