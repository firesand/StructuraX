//! Wave 5, D49 (absorbs D42): design of a combination with a maximum and a minimum.
use sap_design::{try_run_concrete_design, try_run_steel_design};
use sap_model::{
    CaseResults, ComboType, ConcreteRebarConfig, FrameStationResult, LoadCase, LoadCaseType,
    LoadCombo, MatType, Material, SapModel,
};
use sap_section::make_rectangular;
use sap_units::Units;

type Row = (f64, f64, f64, f64, f64, f64); // p, v2, v3, t, m2, m3

fn stations(len: f64, rows: &[Row]) -> Vec<FrameStationResult> {
    let n = rows.len();
    rows.iter()
        .enumerate()
        .map(|(i, r)| {
            let rel = if n == 1 { 0.5 } else { i as f64 / (n - 1) as f64 };
            FrameStationResult {
                station: rel * len,
                rel_dist: rel,
                p: r.0,
                v2: r.1,
                v3: r.2,
                t: r.3,
                m2: r.4,
                m3: r.5,
                ..Default::default()
            }
        })
        .collect()
}

fn put(m: &mut SapModel, name: &str, len: f64, rows: &[Row]) {
    let mut cr = CaseResults {
        case_name: name.into(),
        ..Default::default()
    };
    cr.frame_forces.insert("B1".into(), stations(len, rows));
    m.case_results.insert(name.into(), cr);
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

const STEEL_LEN: f64 = 144.0;
/// A long member, whose lateral-torsional buckling governs so that Cb changes the ratio.
const LONG: f64 = 480.0;

fn steel_model() -> SapModel {
    steel_model_len(STEEL_LEN)
}

fn steel_model_len(len: f64) -> SapModel {
    let mut m = SapModel::new(Units::KipInF);
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 0.0, 0.0, len);
    m.points["1"].restraint = [true, true, true, false, false, true];
    m.points["2"].restraint = [true, true, false, false, false, false];
    m.add_frame("B1", "1", "2", "FSEC1");
    m
}

fn steel_ratio(m: &mut SapModel) -> f64 {
    m.steel_design_results.clear();
    try_run_steel_design(m).unwrap();
    m.steel_design_results["B1"].total_ratio
}

fn concrete_model(column: bool) -> SapModel {
    let mut m = SapModel::new(Units::KipInF);
    m.concrete_design_code = "ACI 318-14".into();
    let mut concrete = Material::default_concrete_kip_in();
    concrete.fy = 60.0;
    concrete.fys = 60.0;
    m.materials.insert(concrete.name.clone(), concrete);
    let mut section = make_rectangular("CSEC", "4000Psi", 20.0, 20.0);
    let mut rebar = Material::default_steel_kip_in();
    rebar.name = "REBAR".into();
    rebar.mat_type = MatType::Rebar;
    rebar.fy = 60.0;
    rebar.fys = 60.0;
    m.materials.insert("REBAR".into(), rebar);
    section.concrete_rebar = Some(ConcreteRebarConfig {
        is_column: column,
        rebar_mat_long: "REBAR".into(),
        rebar_mat_confine: "REBAR".into(),
        cover_top: 2.5,
        cover_bot: 2.5,
        num_bars_3dir: 4,
        num_bars_2dir: 4,
        total_rebar_area: 8.0,
    });
    m.frame_sections.insert("CSEC".into(), section);
    m.add_point("1", 0.0, 0.0, 0.0);
    if column {
        m.add_point("2", 0.0, 0.0, 120.0);
    } else {
        m.add_point("2", 240.0, 0.0, 0.0);
    }
    m.add_frame("B1", "1", "2", "CSEC");
    m
}

fn concrete_numbers(m: &mut SapModel) -> [f64; 6] {
    m.concrete_design_results.clear();
    try_run_concrete_design(m).unwrap();
    let r = &m.concrete_design_results["B1"];
    [r.as_top, r.as_bot, r.as_side, r.av_over_s, r.pmm_ratio, r.min_rebar_area]
}

/// Models whose combinations are all single valued (and plain cases): the numbers must
/// not change with the repair.
fn single_valued_numbers() -> Vec<(&'static str, Vec<f64>)> {
    let d: [Row; 5] = [
        (-30.0, 2.0, 1.0, 0.0, 5.0, 0.0),
        (-30.0, 1.0, 0.5, 0.0, 8.0, 60.0),
        (-30.0, 0.0, 0.0, 0.0, 9.0, 80.0),
        (-30.0, -1.0, -0.5, 0.0, 8.0, 60.0),
        (-30.0, -2.0, -1.0, 0.0, 5.0, 0.0),
    ];
    let l: [Row; 5] = [
        (-8.0, 1.0, 0.5, 0.0, 2.0, 10.0),
        (-8.0, 0.5, 0.2, 0.0, 3.0, 30.0),
        (-8.0, 0.0, 0.0, 0.0, 3.5, 35.0),
        (-8.0, -0.5, -0.2, 0.0, 3.0, 30.0),
        (-8.0, -1.0, -0.5, 0.0, 2.0, 10.0),
    ];
    let mut out = Vec::new();
    // Steel: a combination of two static cases, and the fallback of plain cases.
    let mut m = steel_model();
    put(&mut m, "DEAD", STEEL_LEN, &d);
    put(&mut m, "LIVE", STEEL_LEN, &l);
    add_combo(&mut m, "C", ComboType::LinearAdd, &[("DEAD", 1.2), ("LIVE", 1.6)]);
    out.push(("steel combo", vec![steel_ratio(&mut m)]));
    let r = &m.steel_design_results["B1"];
    out.push((
        "steel combo detail",
        vec![
            r.governing_station,
            r.pr_ratio,
            r.mr_major_ratio,
            r.mr_minor_ratio,
            r.shear_major_ratio,
            r.shear_minor_ratio,
            r.phi_mn33,
        ],
    ));
    let mut m = steel_model();
    put(&mut m, "DEAD", STEEL_LEN, &d);
    put(&mut m, "LIVE", STEEL_LEN, &l);
    out.push(("steel plain cases", vec![steel_ratio(&mut m)]));
    // Concrete column and beam.
    for (name, plain, column) in [("column", "column plain", true), ("beam", "beam plain", false)] {
        let len = if column { 120.0 } else { 240.0 };
        let mut m = concrete_model(column);
        put(&mut m, "DEAD", len, &d);
        put(&mut m, "LIVE", len, &l);
        add_combo(&mut m, "C", ComboType::LinearAdd, &[("DEAD", 1.2), ("LIVE", 1.6)]);
        out.push((name, concrete_numbers(&mut m).to_vec()));
        let mut m = concrete_model(column);
        put(&mut m, "DEAD", len, &d);
        put(&mut m, "LIVE", len, &l);
        out.push((plain, concrete_numbers(&mut m).to_vec()));
    }
    out
}

fn spectrum(m: &mut SapModel, name: &str) {
    let mut rs = LoadCase::linear_static(name, "DEAD", 1.0);
    rs.case_type = LoadCaseType::ResponseSpectrum;
    m.load_cases.insert(name.into(), rs);
}

fn put_pair(m: &mut SapModel, name: &str, len: f64, hi: &[Row], lo: &[Row]) {
    put(m, name, len, hi);
    let mut l = CaseResults::default();
    l.frame_forces.insert("B1".into(), stations(len, lo));
    m.case_results.get_mut(name).unwrap().combo_min = Some(Box::new(l));
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-12 * a.abs().max(b.abs())
}

fn only_p_m3(p: f64, m3: f64) -> Row {
    (p, 0.0, 0.0, 0.0, 0.0, m3)
}

/// The D/C ratio of the steel column under the plain static force set `rows` (one
/// combination of one static case: no maximum and minimum, no permutation, the pre-wave
/// path). This is the reference of every permutation below: the design of a state with
/// known forces.
fn steel_static(rows: &[Row]) -> f64 {
    steel_static_len(STEEL_LEN, rows)
}

fn steel_static_len(len: f64, rows: &[Row]) -> f64 {
    let mut m = steel_model_len(len);
    put(&mut m, "S", len, rows);
    add_combo(&mut m, "Z", ComboType::LinearAdd, &[("S", 1.0)]);
    steel_ratio(&mut m)
}

fn largest(values: &[f64]) -> f64 {
    values.iter().copied().fold(f64::NEG_INFINITY, f64::max)
}

/// Review example R2 row 2: D (P -20, M3 100) and a spectrum case RS (P 6, M3 80), added.
/// Max row (-14, 180), Min row (-26, 20). The governing ratio is the largest over the
/// permutations of the maxima and minima of P and M3 (AISC 360-10 manual 2.4), each
/// designed as a static force set. Baseline: the combination gave 0.0588 (the rows
/// (-14, 180) and (-26, 20) only); the permutation (-26, 180) gives 0.0859.
#[test]
fn additive_with_spectrum_is_designed_at_every_permutation_of_p_and_m3() {
    let perms = [(-14.0, 180.0), (-26.0, 20.0), (-14.0, 20.0), (-26.0, 180.0)];
    let want = largest(&perms.map(|(p, m)| steel_static(&[only_p_m3(p, m)])));
    // The permutation of the review exceeds both rows (otherwise the test sees nothing).
    assert!(want > steel_static(&[only_p_m3(-14.0, 180.0)]) * 1.1);
    assert!(close(want, steel_static(&[only_p_m3(-26.0, 180.0)])));

    // Synthesized from the contributors.
    let mut m = steel_model();
    put(&mut m, "DEAD", STEEL_LEN, &[only_p_m3(-20.0, 100.0)]);
    put(&mut m, "RS", STEEL_LEN, &[only_p_m3(6.0, 80.0)]);
    spectrum(&mut m, "RS");
    add_combo(&mut m, "C", ComboType::LinearAdd, &[("DEAD", 1.0), ("RS", 1.0)]);
    let got = steel_ratio(&mut m);
    assert!(close(got, want), "synthesized {got}, permutations {want}");

    // Stored rows of the solver: Max row and Min row.
    let mut m = steel_model();
    put(&mut m, "DEAD", STEEL_LEN, &[only_p_m3(-20.0, 100.0)]);
    put(&mut m, "RS", STEEL_LEN, &[only_p_m3(6.0, 80.0)]);
    spectrum(&mut m, "RS");
    add_combo(&mut m, "C", ComboType::LinearAdd, &[("DEAD", 1.0), ("RS", 1.0)]);
    put_pair(&mut m, "C", STEEL_LEN, &[only_p_m3(-14.0, 180.0)], &[only_p_m3(-26.0, 20.0)]);
    let got = steel_ratio(&mut m);
    assert!(close(got, want), "stored {got}, permutations {want}");
}

/// Envelope of A (P -20, M3 40) and B (P -2, M3 100), both single valued: the combination's
/// maxima are (P -2, M3 100) and its minima (P -20, M3 40); correspondence is lost
/// (Analysis Reference p. 352: "the maximum of all of the maximum values ... the minimum of
/// all of the minimum values"), so (P -20, M3 100) is designed. Baseline: the two states
/// only.
#[test]
fn envelope_range_srss_and_absolute_are_permuted() {
    let a = only_p_m3(-20.0, 40.0);
    let b = only_p_m3(-2.0, 100.0);
    let build = |ty: ComboType| {
        let mut m = steel_model();
        put(&mut m, "A", STEEL_LEN, &[a]);
        put(&mut m, "B", STEEL_LEN, &[b]);
        add_combo(&mut m, "C", ty, &[("A", 1.0), ("B", 1.0)]);
        m
    };
    // Envelope: P in {-2, -20}, M3 in {100, 40}.
    let env = largest(&[(-2.0, 100.0), (-20.0, 100.0), (-2.0, 40.0), (-20.0, 40.0)].map(|(p, m)| steel_static(&[only_p_m3(p, m)])));
    assert!(env > 1.05 * steel_static(&[a]).max(steel_static(&[b])), "the permutation must govern");
    let got = steel_ratio(&mut build(ComboType::Envelope));
    assert!(close(got, env), "envelope {got}, permutations {env}");

    // Range: max = sum of the positive maxima = (0, 140), min = sum of the negative minima
    // = (-22, 0): P in {0, -22}, M3 in {140, 0}.
    let range = largest(&[(0.0, 140.0), (-22.0, 140.0), (0.0, 0.0), (-22.0, 0.0)].map(|(p, m)| steel_static(&[only_p_m3(p, m)])));
    let got = steel_ratio(&mut build(ComboType::RangeAdd));
    assert!(close(got, range), "range {got}, permutations {range}");
    assert!(range > steel_static(&[only_p_m3(0.0, 140.0)]) * 1.05);

    // SRSS: max = (sqrt(20^2+2^2), sqrt(40^2+100^2)), min = -max; the permutations are the
    // sign choices of P and M3 (the ratio uses |M3|).
    let (rp, rm) = ((20.0f64.powi(2) + 2.0f64.powi(2)).sqrt(), (40.0f64.powi(2) + 100.0f64.powi(2)).sqrt());
    let srss = largest(&[(rp, rm), (-rp, rm), (rp, -rm), (-rp, -rm)].map(|(p, m)| steel_static(&[only_p_m3(p, m)])));
    let got = steel_ratio(&mut build(ComboType::Srss));
    assert!(close(got, srss), "srss {got}, permutations {srss}");

    // Absolute: max = (22, 140), min = -max.
    let abs = largest(&[(22.0, 140.0), (-22.0, 140.0), (22.0, -140.0), (-22.0, -140.0)].map(|(p, m)| steel_static(&[only_p_m3(p, m)])));
    let got = steel_ratio(&mut build(ComboType::AbsAdd));
    assert!(close(got, abs), "absolute {got}, permutations {abs}");
}

/// Nested multi-valued combination: I = Range(A, B) with A (P -20, M3 40), B (P 6, M3 80);
/// O = I + C with C (P -3, M3 20) single valued. I has max (6, 120) and min (-20, 0); O has
/// max (3, 140) and min (-23, 20); the permutation (-23, 140) governs. Baseline: the two
/// rows only.
#[test]
fn a_nested_multi_valued_combination_is_permuted() {
    let mut m = steel_model();
    put(&mut m, "A", STEEL_LEN, &[only_p_m3(-20.0, 40.0)]);
    put(&mut m, "B", STEEL_LEN, &[only_p_m3(6.0, 80.0)]);
    put(&mut m, "C", STEEL_LEN, &[only_p_m3(-3.0, 20.0)]);
    add_combo(&mut m, "I", ComboType::RangeAdd, &[("A", 1.0), ("B", 1.0)]);
    add_combo(&mut m, "O", ComboType::LinearAdd, &[("I", 1.0), ("C", 1.0)]);
    let want = largest(&[(3.0, 140.0), (-23.0, 140.0), (3.0, 20.0), (-23.0, 20.0)].map(|(p, mm)| steel_static(&[only_p_m3(p, mm)])));
    assert!(want > 1.05 * steel_static(&[only_p_m3(3.0, 140.0)]));
    // The solver stores no result for a combination it did not run; the design synthesizes
    // both I and O. The design of O alone is checked by hiding I (the design also designs I
    // itself, whose ratio is smaller than that of O here).
    let got = steel_ratio(&mut m);
    assert!(close(got, want), "nested {got}, permutations {want}");
    assert_eq!(m.steel_design_results["B1"].governing_combo, "O");
}


fn diagram(p: f64, m3: [f64; 5]) -> Vec<Row> {
    m3.iter().map(|&m| only_p_m3(p, m)).collect()
}

/// AISC 360-10 F1-1 by hand: Cb = 12.5 Mmax / (2.5 Mmax + 3 MA + 4 MB + 3 MC) from the
/// absolute moments at the quarter points of five equally spaced stations.
fn cb_by_hand(m3: [f64; 5]) -> f64 {
    let a = m3.map(f64::abs);
    let mmax = a.iter().copied().fold(0.0, f64::max);
    12.5 * mmax / (2.5 * mmax + 3.0 * a[1] + 4.0 * a[2] + 3.0 * a[3])
}

/// A multi-valued combination whose Max row has a uniform moment and whose Min row has
/// the larger, V-shaped moment. The rows are two states, each with its own moment diagram
/// and its own Cb; the governing ratio is the largest over the four permutations of the
/// maxima and minima of P and M3, each designed as a static force set with its own diagram.
/// Baseline: one station list of ten stations, so Cb came from the first row's quarter
/// points with the largest moment of both rows (2.78 for every state, where the Min row's
/// own diagram gives 1.81).
#[test]
fn each_state_has_its_own_moment_diagram_and_cb() {
    use sap_design::steel::compute_cb_from_stations;
    let hi_m = [20.0; 5];
    let lo_m = [-100.0, -60.0, -20.0, -60.0, -100.0];
    // Hand values of the manual's equation for the two diagrams.
    assert!(close(cb_by_hand(hi_m), 1.0));
    assert!(close(cb_by_hand(lo_m), 1250.0 / 690.0));
    // The code's Cb of a state's own station list is that value (reference: the hand formula).
    assert!(close(
        compute_cb_from_stations(&stations(LONG, &diagram(-26.0, lo_m))),
        1250.0 / 690.0
    ));
    // The concatenation of both rows is what the baseline read: 12.5*100 / (250 + 3*20 + 4*20 + 3*20).
    let concatenated: Vec<_> = [hi_m, lo_m]
        .iter()
        .flat_map(|m| stations(LONG, &diagram(0.0, *m)))
        .collect();
    assert!(close(compute_cb_from_stations(&concatenated), 1250.0 / 450.0));

    let mut want = f64::NEG_INFINITY;
    for p in [-14.0, -26.0] {
        for m in [hi_m, lo_m] {
            want = want.max(steel_static_len(LONG, &diagram(p, m)));
        }
    }
    let mut m = steel_model_len(LONG);
    put(&mut m, "DEAD", LONG, &diagram(-20.0, [0.0; 5]));
    put(&mut m, "RS", LONG, &diagram(0.0, [0.0; 5]));
    spectrum(&mut m, "RS");
    add_combo(&mut m, "C", ComboType::LinearAdd, &[("DEAD", 1.0), ("RS", 1.0)]);
    put_pair(&mut m, "C", LONG, &diagram(-14.0, hi_m), &diagram(-26.0, lo_m));
    let got = steel_ratio(&mut m);
    assert!(close(got, want), "stored {got}, per-state reference {want}");
}

/// Max and Min rows with the same moment diagram (a parabola) and different P: Cb is that of
/// one row (hand value 1000/880), the ratio that of the static case with that diagram and
/// the worse P. (On the baseline this also holds: the concatenated list reads the first
/// row's quarter points.)
#[test]
fn equal_moments_in_both_rows_give_the_cb_of_one_row() {
    use sap_design::steel::compute_cb_from_stations;
    let parab = [0.0, 60.0, 80.0, 60.0, 0.0];
    assert!(close(cb_by_hand(parab), 1000.0 / 880.0));
    assert!(close(
        compute_cb_from_stations(&stations(LONG, &diagram(-5.0, parab))),
        1000.0 / 880.0
    ));
    let want = steel_static_len(LONG, &diagram(-5.0, parab)).max(steel_static_len(LONG, &diagram(-40.0, parab)));
    let mut m = steel_model_len(LONG);
    put(&mut m, "DEAD", LONG, &diagram(-20.0, parab));
    put(&mut m, "RS", LONG, &diagram(0.0, [0.0; 5]));
    spectrum(&mut m, "RS");
    add_combo(&mut m, "C", ComboType::LinearAdd, &[("DEAD", 1.0), ("RS", 1.0)]);
    put_pair(&mut m, "C", LONG, &diagram(-5.0, parab), &diagram(-40.0, parab));
    let got = steel_ratio(&mut m);
    assert!(close(got, want), "{got} against {want}");
}

fn concrete_static(column: bool, len: f64, rows: &[Row]) -> [f64; 6] {
    let mut m = concrete_model(column);
    put(&mut m, "S", len, rows);
    add_combo(&mut m, "Z", ComboType::LinearAdd, &[("S", 1.0)]);
    concrete_numbers(&mut m)
}

/// Concrete column P-M-M (ACI 318-14 manual 2.1: "multiple sub combinations using
/// maxima/minima permutations of interacting quantities"): D (P -400, M2 0, M3 1500) plus a
/// spectrum case RS (P 300, M2 300, M3 1000), added: Max row (-100, 300, 2500), Min row
/// (-700, -300, 500). The P-M-M ratio is the largest over the eight permutations of the
/// maxima and minima of P, M2 and M3 (|M2| is 300 for both), each designed as a static
/// force set. Baseline: the two rows only.
#[test]
fn column_pmm_is_designed_at_every_permutation() {
    let len = 120.0;
    let row = |p: f64, m2: f64, m3: f64| -> Row { (p, 0.0, 0.0, 0.0, m2, m3) };
    let mut want = f64::NEG_INFINITY;
    for p in [-100.0, -700.0] {
        for m2 in [300.0, -300.0] {
            for m3 in [2500.0, 500.0] {
                want = want.max(concrete_static(true, len, &[row(p, m2, m3)])[4]);
            }
        }
    }
    let rows_only = concrete_static(true, len, &[row(-100.0, 300.0, 2500.0)])[4]
        .max(concrete_static(true, len, &[row(-700.0, -300.0, 500.0)])[4]);
    assert!(want > 1.05 * rows_only, "the permutation must govern: {want} against {rows_only}");

    // Synthesized from the contributors.
    let mut m = concrete_model(true);
    put(&mut m, "DEAD", len, &[row(-400.0, 0.0, 1500.0)]);
    put(&mut m, "RS", len, &[row(300.0, 300.0, 1000.0)]);
    spectrum(&mut m, "RS");
    add_combo(&mut m, "C", ComboType::LinearAdd, &[("DEAD", 1.0), ("RS", 1.0)]);
    let got = concrete_numbers(&mut m)[4];
    assert!(close(got, want), "synthesized {got}, permutations {want}");
    // Stored Max and Min rows.
    put_pair(
        &mut m,
        "C",
        len,
        &[row(-100.0, 300.0, 2500.0)],
        &[row(-700.0, -300.0, 500.0)],
    );
    let got = concrete_numbers(&mut m)[4];
    assert!(close(got, want), "stored {got}, permutations {want}");
}

/// Concrete column under an Envelope of A and B: the maxima and minima of the envelope are
/// permuted (Analysis Reference p. 352 for the rule of the envelope, the design manual for
/// the permutation).
#[test]
fn column_pmm_envelope_is_permuted() {
    let len = 120.0;
    let row = |p: f64, m3: f64| -> Row { (p, 0.0, 0.0, 0.0, 0.0, m3) };
    // A (P -700, M3 500) and B (P -100, M3 2500).
    let mut want = f64::NEG_INFINITY;
    for p in [-100.0, -700.0] {
        for m3 in [2500.0, 500.0] {
            want = want.max(concrete_static(true, len, &[row(p, m3)])[4]);
        }
    }
    let states = concrete_static(true, len, &[row(-700.0, 500.0)])[4]
        .max(concrete_static(true, len, &[row(-100.0, 2500.0)])[4]);
    assert!(want > 1.05 * states);
    let mut m = concrete_model(true);
    put(&mut m, "A", len, &[row(-700.0, 500.0)]);
    put(&mut m, "B", len, &[row(-100.0, 2500.0)]);
    add_combo(&mut m, "E", ComboType::Envelope, &[("A", 1.0), ("B", 1.0)]);
    let got = concrete_numbers(&mut m)[4];
    assert!(close(got, want), "envelope {got}, permutations {want}");
}

/// Concrete beam: every check uses one force alone (M3 for the top and bottom steel, M2 for
/// the side steel, V2 and V3 for the shear steel), so a multi-valued combination is the
/// largest over its states, with the strength floors of the framing type taken from the
/// state's own end moments. D (M3 100 at the ends, 60 at mid, V2 3) plus RS (M3 150, 90, V2
/// 8), added: Max row (250, 150, V2 11), Min row (-50, -30, V2 -5). Reference: the static
/// design of each row; the steel areas are the larger of the two rows' areas. (The same
/// numbers come out on the baseline: nothing in a beam looks across the rows.)
#[test]
fn concrete_beam_is_the_largest_over_its_states() {
    let len = 240.0;
    let rows = |a: f64, b: f64, v: f64| -> Vec<Row> {
        vec![
            (0.0, v, 0.0, 0.0, 0.0, a),
            (0.0, v / 2.0, 0.0, 0.0, 0.0, b),
            (0.0, 0.0, 0.0, 0.0, 0.0, a),
        ]
    };
    let hi = rows(250.0, 150.0, 11.0);
    let lo = rows(-50.0, -30.0, -5.0);
    let a = concrete_static(false, len, &hi);
    let b = concrete_static(false, len, &lo);
    let mut m = concrete_model(false);
    m.frames.get_mut("B1").unwrap().concrete_framing = sap_model::ConcreteFramingType::SwaySpecial;
    put(&mut m, "DEAD", len, &rows(100.0, 60.0, 3.0));
    put(&mut m, "RS", len, &rows(150.0, 90.0, 8.0));
    spectrum(&mut m, "RS");
    add_combo(&mut m, "C", ComboType::LinearAdd, &[("DEAD", 1.0), ("RS", 1.0)]);
    put_pair(&mut m, "C", len, &hi, &lo);
    let got = concrete_numbers(&mut m);
    // Without a framing floor the areas are those of the two static runs; with the special
    // framing floors (positive strength at an end at least half the negative one, 0.25 of the
    // largest end strength everywhere) they are at least those.
    for k in 0..4 {
        assert!(got[k] >= a[k].max(b[k]) * (1.0 - 1e-12), "item {k}: {} against {}", got[k], a[k].max(b[k]));
    }
    // The same combination without the framing floor equals the static maxima exactly.
    let mut m2 = m.clone();
    m2.frames.get_mut("B1").unwrap().concrete_framing = Default::default();
    let got2 = concrete_numbers(&mut m2);
    for k in 0..4 {
        assert!(close(got2[k], a[k].max(b[k])), "item {k}: {} against {}", got2[k], a[k].max(b[k]));
    }
}

/// Eurocode 3 with the lateral-torsional buckling length set to a hundredth of the member, so
/// the moment resistance is the plastic one and the shear reduction of 6.2.8 can act.
fn ec3_unbraced(m: &mut SapModel) {
    m.steel_design_code = "Eurocode 3".into();
    m.frames.get_mut("B1").unwrap().steel_overwrite = Some(sap_model::SteelDesignOverwrite {
        l_ratio_ltb: Some(0.01),
        ..Default::default()
    });
}

fn ec3_static(rows: &[Row]) -> sap_model::SteelDesignResult {
    let mut m = steel_model();
    ec3_unbraced(&mut m);
    put(&mut m, "S", STEEL_LEN, rows);
    add_combo(&mut m, "Z", ComboType::LinearAdd, &[("S", 1.0)]);
    m.steel_design_results.clear();
    try_run_steel_design(&mut m).unwrap();
    m.steel_design_results["B1"].clone()
}

/// Eurocode 3 reduces the moment resistance where the shear exceeds half the shear
/// resistance (EN 1993-1-1 6.2.8), so V2 and M3 interact there and both are permuted. Envelope
/// of A (V2 0.9 Vc, M3 0.1 Mc) and B (V2 0, M3 0.95 Mc), Vc and Mc the resistances found
/// from a run without forces: the permutation (V2 0.9 Vc, M3 0.95 Mc) is designed. Baseline: the two
/// states only.
#[test]
fn eurocode_shear_and_moment_are_permuted_together() {
    let zero = ec3_static(&[only_p_m3(0.0, 0.0)]);
    let (vc, mc) = (zero.phi_vn2, zero.phi_mn33);
    let a: Row = (0.0, 0.9 * vc, 0.0, 0.0, 0.0, 0.1 * mc);
    let b: Row = (0.0, 0.0, 0.0, 0.0, 0.0, 0.95 * mc);
    let c: Row = (0.0, 0.9 * vc, 0.0, 0.0, 0.0, 0.95 * mc);
    let want = ec3_static(&[c]).total_ratio;
    let (ra, rb) = (ec3_static(&[a]).total_ratio, ec3_static(&[b]).total_ratio);
    assert!(want > 1.05 * ra.max(rb), "a {ra} b {rb} c {want}");
    let mut m = steel_model();
    ec3_unbraced(&mut m);
    put(&mut m, "A", STEEL_LEN, &[a]);
    put(&mut m, "B", STEEL_LEN, &[b]);
    add_combo(&mut m, "E", ComboType::Envelope, &[("A", 1.0), ("B", 1.0)]);
    let got = steel_ratio(&mut m);
    assert!(close(got, want), "envelope {got}, permutation {want}");
}

/// AISC checks the shear alone (G2-1 on V2) and the axial-flexure interaction on P, M2, M3
/// (H1-1): an envelope whose largest shear (A) and largest moment (B) occur in different
/// states is designed at each state, and the result is the larger of the two static designs;
/// V2 is not paired with the other state's moment. (Equal on the baseline.)
#[test]
fn aisc_shear_is_checked_alone() {
    let a: Row = (0.0, 30.0, 0.0, 0.0, 0.0, 10.0);
    let b: Row = (0.0, 2.0, 0.0, 0.0, 0.0, 90.0);
    let mut m = steel_model();
    put(&mut m, "A", STEEL_LEN, &[a]);
    put(&mut m, "B", STEEL_LEN, &[b]);
    add_combo(&mut m, "E", ComboType::Envelope, &[("A", 1.0), ("B", 1.0)]);
    let got = steel_ratio(&mut m);
    let want = steel_static(&[a]).max(steel_static(&[b]));
    assert!(close(got, want), "{got} against {want}");
}

/// Single-valued combinations and plain cases give bitwise the numbers they gave before the
/// repair. The pinned bits were printed by `single_valued_numbers` on the baseline
/// (W5_S2 plus nothing), before any change of `steel.rs` or `concrete.rs`. They are not a
/// reference of the design code, only a guard that one state is designed as it was; the
/// models have five stations with different moment diagrams, so Cb and the station choice are
/// in the numbers. (Passes on the baseline by construction.)
///
/// D86 changed three pinned values, all of the compression term of the default `FSEC1` (I 18 x 7.5,
/// tf 0.57, tw 0.355, KL/r = 87.3): its web h/tw = 47.5 is slender (> 1.49 sqrt(E/Fy) = 35.9),
/// and the effective width of the web now takes f = Fcr with Q = 1 (manual p. 3-33) instead of
/// Fy. Hand: Fe = pi^2 29000/(144/1.65)^2 = 37.58, f = 28.649, be = 16.7464, Q = 0.99726 (was
/// 0.92091), phi Pn = 378.566 kip (was 364.768), pr_ratio = 0.5 (48.8)/378.566 = 0.064454
/// (was 0.066892); the flexural and shear ratios did not change. Old bits of the steel combo:
/// 0x3fc042a3b9bb88a0 (0.12703), of its pr_ratio 0x3fb11fd24d2ffa52, of the plain cases
/// 0x3fb2a5fbbe0511eb.
#[test]
fn single_valued_combinations_and_plain_cases_are_bitwise_unchanged() {
    let pinned: [(&str, &[u64]); 7] = [
        ("steel combo", &[0x3fbfe57fb4999a0f]),
        (
            "steel combo detail",
            &[
                0x4052000000000000,
                0x3fb0800a8e528322,
                0x3fa38d4c09cbef36,
                0x3f967b3c85847d49,
                0x0000000000000000,
                0x0000000000000000,
                0x40af18be736f02e2,
            ],
        ),
        ("steel plain cases", &[0x3fb243c1ff3df8e9]),
        // Re-pinned for D102 (2026-11-19): the ACI minimum eccentricity (0.6 + 0.03 h in) now
        // raises the moment of a column station whose moment is below Pu e_min, so the P-M-M
        // ratio of this column went from 0x3faa816aa1659ff4 (0.051768619729784776) to the
        // value below (0.05183388005629001, +0.13 %). The hand-checked tests of that rule
        // are in `batch_c_repairs.rs`; this entry stays a guard of one state only.
        (
            "column",
            &[0x4010000000000000, 0x4010000000000000, 0, 0, 0x3faa89f867554656, 0x4010000000000000],
        ),
        (
            "column plain",
            &[0x4010000000000000, 0x4010000000000000, 0, 0, 0x3fa04b5cd487eaba, 0x4010000000000000],
        ),
        (
            "beam",
            &[0x3f771e8557c5b1e4, 0x3fcb90391e3b41bc, 0x3f97b47e1e66ec00, 0, 0, 0x3ff2aaaaaaaaaaab],
        ),
        (
            "beam plain",
            &[0x3f6ce5dadf7c6f1c, 0x3fbcf5377691bdf0, 0x3f8a033981a2dfaa, 0, 0, 0x3ff2aaaaaaaaaaab],
        ),
    ];
    let got = single_valued_numbers();
    assert_eq!(got.len(), pinned.len());
    for ((name, values), (pname, bits)) in got.iter().zip(pinned.iter()) {
        assert_eq!(name, pname);
        let now: Vec<u64> = values.iter().map(|v| v.to_bits()).collect();
        assert_eq!(&now[..], *bits, "{name}");
    }
}
