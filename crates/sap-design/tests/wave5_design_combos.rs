//! Wave 5, step 3c: steel design takes a multi-valued combination as the manual's
//! pair (CSI Analysis Reference pp. 351-353): a response-spectrum contribution is
//! (+v, -v), so `1.0 D + 1.0 RS` with D = -10 kip and RS = 7 kip is checked at -3
//! kip (maximum) and at -17 kip (minimum). The member is a column in pure axial
//! compression (M = V = 0), so the ratio is |P| / phi Pn and phi Pn is the same
//! number whatever the combination is: the tests compare the reported ratio
//! times phi Pn with the governing |P| of the manual's rule.
use sap_design::try_run_steel_design;
use sap_model::{
    CaseResults, ComboType, FrameStationResult, LoadCase, LoadCaseType, LoadCombo, SapModel,
};
use sap_units::Units;

fn column(p_dead: f64, p_rs: f64) -> SapModel {
    let mut m = SapModel::new(Units::KipInF);
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 0.0, 0.0, 144.0);
    // Base: translations and torsion restrained; top: lateral translations restrained
    // (a pin-ended column, no rigid-body motion left in the sense of the design check).
    m.points["1"].restraint = [true, true, true, false, false, true];
    m.points["2"].restraint = [true, true, false, false, false, false];
    m.add_frame("B1", "1", "2", "FSEC1");
    let mut rs = LoadCase::linear_static("RS", "DEAD", 1.0);
    rs.case_type = LoadCaseType::ResponseSpectrum;
    m.load_cases.insert("RS".into(), rs);
    for (name, p) in [("DEAD", p_dead), ("RS", p_rs)] {
        let mut cr = CaseResults {
            case_name: name.into(),
            ..Default::default()
        };
        cr.frame_forces.insert(
            "B1".into(),
            vec![FrameStationResult {
                station: 72.0,
                rel_dist: 0.5,
                p,
                ..Default::default()
            }],
        );
        m.case_results.insert(name.into(), cr);
    }
    m
}

fn add_combo(m: &mut SapModel, name: &str, ty: ComboType, cases: &[(&str, f64)]) {
    m.load_combos.insert(
        name.into(),
        LoadCombo {
            name: name.into(),
            combo_type: ty,
            cases: cases.iter().map(|(c, f)| (c.to_string(), *f)).collect(),
        },
    );
}

/// The reported D/C ratio of the column.
fn ratio(m: &mut SapModel) -> f64 {
    m.steel_design_results.clear();
    try_run_steel_design(m).unwrap();
    m.steel_design_results["B1"].total_ratio
}

/// The D/C ratio of the same column under a plain static axial force `p` kip: the
/// reference for a demand of |p| (the ratio is a function of |P| / phi Pn alone, whatever
/// design branch the code takes for that magnitude).
fn ratio_of_static(p: f64) -> f64 {
    let mut m = column(p, 0.0);
    add_combo(&mut m, "Z", ComboType::LinearAdd, &[("DEAD", 1.0)]);
    ratio(&mut m)
}

/// Asserts that `got` equals the ratio of a static compression of `kip` kip.
fn demand(what: &str, got: f64, kip: f64) {
    let want = ratio_of_static(-kip);
    assert!(
        (got - want).abs() <= 1e-9 * want,
        "{what}: ratio {got}, a demand of {kip} kip gives {want}"
    );
}

#[test]
fn additive_with_a_spectrum_case_is_designed_at_minus_3_and_minus_17_kip() {
    // Manual's rule: max = -10 + 7 = -3, min = -10 - 7 = -17 kip. The old code added
    // the positive 7 only: -3 kip (ratio 3 / phi Pn).
    let mut m = column(-10.0, 7.0);
    add_combo(&mut m, "C", ComboType::LinearAdd, &[("DEAD", 1.0), ("RS", 1.0)]);
    let got = ratio(&mut m);
    demand("synthesized from the contributors", got, 17.0);
    // The old value (a demand of 3 kip) is a different, smaller ratio.
    assert!(got > 4.0 * ratio_of_static(-3.0));

    // The same combination with stored results (the solver's output): maximum row
    // -3, minimum row -17.
    let mut m = column(-10.0, 7.0);
    add_combo(&mut m, "C", ComboType::LinearAdd, &[("DEAD", 1.0), ("RS", 1.0)]);
    let station = |p: f64| FrameStationResult {
        station: 72.0,
        rel_dist: 0.5,
        p,
        ..Default::default()
    };
    let mut hi = CaseResults::default();
    hi.frame_forces.insert("B1".into(), vec![station(-3.0)]);
    let mut lo = CaseResults::default();
    lo.frame_forces.insert("B1".into(), vec![station(-17.0)]);
    hi.combo_min = Some(Box::new(lo));
    m.case_results.insert("C".into(), hi);
    let got = ratio(&mut m);
    demand("stored maximum and minimum rows", got, 17.0);

    // A static-only additive combination is unchanged: one state, P = -10 - 4.
    let mut m = column(-10.0, 7.0);
    m.case_results.get_mut("RS").unwrap().frame_forces.get_mut("B1").unwrap()[0].p = 4.0;
    m.load_cases.get_mut("RS").unwrap().case_type = LoadCaseType::LinearStatic;
    add_combo(&mut m, "C", ComboType::LinearAdd, &[("DEAD", 1.0), ("RS", -1.0)]);
    let got = ratio(&mut m);
    demand("single valued", got, 14.0);
}

#[test]
fn negative_factor_on_a_spectrum_case_swaps_nothing_in_the_pair() {
    // D + (-1) RS: pair (-10 + 7, -10 - 7) the same as with +1 (p. 351).
    let mut m = column(-10.0, 7.0);
    add_combo(&mut m, "C", ComboType::LinearAdd, &[("DEAD", 1.0), ("RS", -1.0)]);
    let got = ratio(&mut m);
    demand("D - RS", got, 17.0);
}

#[test]
fn envelope_and_range_follow_the_manual_pair() {
    // Envelope of D (-10) and RS (+-7): the states are -10, +7 and -7; the governing
    // compression is 10 kip (the old code had the states -10 and +7 only).
    let mut m = column(-10.0, 7.0);
    add_combo(&mut m, "E", ComboType::Envelope, &[("DEAD", 1.0), ("RS", 1.0)]);
    let got = ratio(&mut m);
    demand("envelope of D and RS", got, 10.0);
    // Envelope of RS alone: the sense -7 is checked (compression 7 kip).
    let mut m = column(-10.0, 7.0);
    add_combo(&mut m, "E", ComboType::Envelope, &[("RS", 1.0)]);
    let got = ratio(&mut m);
    demand("envelope of RS", got, 7.0);
    // Range of D (-10) and RS (+-7): maximum = 0 + 7 = 7, minimum = -10 - 7 = -17.
    let mut m = column(-10.0, 7.0);
    add_combo(&mut m, "R", ComboType::RangeAdd, &[("DEAD", 1.0), ("RS", 1.0)]);
    let got = ratio(&mut m);
    demand("range", got, 17.0);
}

#[test]
fn srss_and_absolute_keep_both_senses_of_the_axial_force() {
    // SRSS of D (-10) and RS (7): sqrt(100 + 49) = 12.2065556 (tension and compression).
    let mut m = column(-10.0, 7.0);
    add_combo(&mut m, "S", ComboType::Srss, &[("DEAD", 1.0), ("RS", 1.0)]);
    let got = ratio(&mut m);
    demand("srss", got, 149f64.sqrt());
    // Absolute: 10 + 7 = 17.
    let mut m = column(-10.0, 7.0);
    add_combo(&mut m, "A", ComboType::AbsAdd, &[("DEAD", 1.0), ("RS", 1.0)]);
    let got = ratio(&mut m);
    demand("absolute", got, 17.0);
}
