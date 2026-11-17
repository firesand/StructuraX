//! Wave 5 verification rows (package P3): the maximum and minimum of
//! combinations, CSI Analysis Reference p. 353.

use crate::benchmarks::BenchmarkVerificationRow as Row;
use sap_model::{
    CaseResults, ComboType, DynamicFunction, FrameStationResult, LoadCase, LoadCaseType,
    LoadCombo, LoadPattern, LoadPatternType, PointLoadForce, SapModel,
};

fn combo(m: &mut SapModel, name: &str, kind: ComboType, cases: &[(&str, f64)]) {
    m.load_combos.insert(
        name.into(),
        LoadCombo {
            name: name.into(),
            combo_type: kind,
            cases: cases.iter().map(|(c, f)| (c.to_string(), *f)).collect(),
        },
    );
}

/// The axial-force example of p. 353: GRAV 10, WINDX 5, WINDY 3 (static) and
/// EQ 7 (response spectrum); GRAVEQ = GRAV + EQ is 17 / 3 and SEVERE, the
/// envelope of GRAVEQ and GRAVWIN, is 17 / 3 as the manual prints.
fn manual_axial_rows(rows: &mut Vec<Row>) {
    let mut m = SapModel::default();
    m.load_cases.clear();
    for (name, ty, p) in [
        ("GRAV", LoadCaseType::LinearStatic, 10.0),
        ("WINDX", LoadCaseType::LinearStatic, 5.0),
        ("WINDY", LoadCaseType::LinearStatic, 3.0),
        ("EQ", LoadCaseType::ResponseSpectrum, 7.0),
    ] {
        let mut lc = LoadCase::linear_static(name, "DEAD", 1.0);
        lc.case_type = ty;
        m.load_cases.insert(name.into(), lc);
        let mut cr = CaseResults::default();
        cr.frame_forces.insert(
            "B".into(),
            vec![FrameStationResult {
                p,
                ..Default::default()
            }],
        );
        m.case_results.insert(name.into(), cr);
    }
    combo(&mut m, "WIND", ComboType::Srss, &[("WINDX", 1.0), ("WINDY", 1.0)]);
    combo(&mut m, "GRAVEQ", ComboType::LinearAdd, &[("GRAV", 1.0), ("EQ", 1.0)]);
    combo(&mut m, "GRAVWIN", ComboType::LinearAdd, &[("GRAV", 1.0), ("WIND", 1.0)]);
    combo(&mut m, "SEVERE", ComboType::Envelope, &[("GRAVEQ", 1.0), ("GRAVWIN", 1.0)]);
    crate::combos::evaluate_load_combos(&mut m);
    let pair = |name: &str| {
        let hi = &m.case_results[name];
        let lo = hi.combo_min.as_deref().unwrap_or(hi);
        (hi.frame_forces["B"][0].p, lo.frame_forces["B"][0].p)
    };
    let ex = "Combinations p. 353";
    for (name, max, min) in [("GRAVEQ", 17.0, 3.0), ("SEVERE", 17.0, 3.0)] {
        let (hi, lo) = pair(name);
        rows.push(Row::new(ex, name, "P max [kip]", max, hi, 1e-9));
        rows.push(Row::new(ex, name, "P min [kip]", min, lo, 1e-9));
    }
}

/// COMB1 and COMB2 of p. 353 with a real response-spectrum case: one mass 2 on
/// a spring 50 (w^2 = 25), a static load of 175 (LL = 3.5) and a constant
/// spectrum of 50 (QUAKE = 50 / 25 = 2.0). COMB1 (additive) is 5.5 / 1.5 and
/// COMB2 (envelope) is 3.5 / -2.0.
fn manual_solver_rows(rows: &mut Vec<Row>) {
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    m.add_point("C", 0.0, 0.0, 0.0);
    let c = &mut m.points["C"];
    c.mass = [2.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    c.restraint = [false, true, true, true, true, true];
    c.spring = [50.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    m.load_patterns.insert(
        "LL".into(),
        LoadPattern {
            name: "LL".into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
    m.points["C"].loads_force.push(PointLoadForce {
        load_pat: "LL".into(),
        csys: "GLOBAL".into(),
        force: [175.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    });
    m.load_cases
        .insert("LL".into(), LoadCase::linear_static("LL", "LL", 1.0));
    m.load_cases
        .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 1));
    m.functions.insert(
        "SPEC".into(),
        DynamicFunction {
            name: "SPEC".into(),
            is_time_history: false,
            damping_ratio: 0.05,
            points: vec![(0.0, 50.0), (10.0, 50.0)],
        },
    );
    let mut rs = LoadCase::linear_static("QUAKE", "LL", 1.0);
    rs.case_type = LoadCaseType::ResponseSpectrum;
    rs.loads.clear();
    rs.initial_case = Some("MODAL".into());
    rs.spectrum_loads = vec![("SPEC".into(), "U1".into(), 1.0)];
    m.load_cases.insert("QUAKE".into(), rs);
    combo(&mut m, "COMB1", ComboType::LinearAdd, &[("LL", 1.0), ("QUAKE", 1.0)]);
    combo(&mut m, "COMB2", ComboType::Envelope, &[("LL", 1.0), ("QUAKE", 1.0)]);
    // An analysis error must fail the suite, not drop the rows.
    crate::analyze_model(&mut m).expect("combination benchmark model of p. 353 must analyse");
    let ex = "Combinations p. 353";
    for (name, max, min) in [("COMB1", 5.5, 1.5), ("COMB2", 3.5, -2.0)] {
        let hi = &m.case_results[name];
        let lo = hi.combo_min.as_deref().unwrap_or(hi);
        rows.push(Row::new(ex, name, "U1 max [in]", max, hi.joint_displacements["C"][0], 1e-6));
        rows.push(Row::new(ex, name, "U1 min [in]", min, lo.joint_displacements["C"][0], 1e-6));
    }
}

pub(crate) fn rows() -> Vec<crate::benchmarks::BenchmarkVerificationRow> {
    let mut rows = Vec::new();
    manual_axial_rows(&mut rows);
    manual_solver_rows(&mut rows);
    rows
}
