//! D127: a member that the steel or concrete design refuses is recorded with its reason and the
//! run goes on with the other members (tolerant entry points); the strict entry points still stop
//! at the first refused member. SAP2000 reports design error messages per frame (v19 API help,
//! cDesignSteel.GetSummaryResults, ErrorSummary).
//!
//! No number is computed here: the independent reference is the same member designed alone in a
//! model of its own, compared field by field (the `Debug` text of a float is its shortest
//! round-trip form, so equal text is equal bits).

use sap_design::{
    run_concrete_design_per_member, run_steel_design_per_member,
    run_steel_design_per_member_with_overwrites, try_run_concrete_design, try_run_steel_design,
    verify_all_members_passed, verify_analysis_vs_design, DesignError, DesignRunReport,
};
use sap_model::{
    CaseResults, ComboType, FramePropType, FrameSection, FrameStationResult, LoadCombo, Material,
    SapModel,
};
use sap_section::{make_i_section, make_rectangular};
use sap_units::Units;
use std::collections::HashMap;

type Member = (&'static str, FrameSection, [f64; 3], [f64; 3], Vec<FrameStationResult>);

fn st(p: f64, v2: f64, m3: f64, t: f64) -> FrameStationResult {
    FrameStationResult {
        station: 60.0,
        rel_dist: 0.5,
        p,
        v2,
        m3,
        t,
        ..Default::default()
    }
}

/// A model of the members: each with its own two points, forces in the case "LC".
fn build(units: Units, steel_code: &str, concrete_code: &str, members: Vec<Member>) -> SapModel {
    let mut model = SapModel::new(units);
    model.steel_design_code = steel_code.to_string();
    model.concrete_design_code = concrete_code.to_string();
    let mut cr = CaseResults {
        case_name: "LC".into(),
        ..Default::default()
    };
    for (i, (name, sec, a, b, stations)) in members.into_iter().enumerate() {
        let (pi, pj) = (format!("PI{i}"), format!("PJ{i}"));
        model.frame_sections.insert(sec.name.clone(), sec.clone());
        model.add_point(&pi, a[0], a[1], a[2]);
        model.add_point(&pj, b[0], b[1], b[2]);
        model.add_frame(name, &pi, &pj, &sec.name);
        cr.frame_forces.insert(name.to_string(), stations);
    }
    model.case_results.insert("LC".into(), cr);
    model
}

fn same(a: &impl std::fmt::Debug, b: &impl std::fmt::Debug, what: &str) {
    assert_eq!(format!("{a:?}"), format!("{b:?}"), "{what}");
}

// ---------------------------------------------------------------------------
// steel
// ---------------------------------------------------------------------------

/// kip, in: a W-like I beam (designed) and a single angle in compression (refused, G81).
fn angle() -> FrameSection {
    FrameSection {
        name: "L8X8".into(),
        material: "A992Fy50".into(),
        prop_type: FramePropType::Angle,
        t3: 8.0,
        t2: 8.0,
        tf: 0.5,
        tw: 0.5,
        t2b: 0.0,
        tfb: 0.0,
        area: 7.75,
        i33: 48.4375,
        i22: 48.4375,
        r33: (48.4375_f64 / 7.75).sqrt(),
        r22: (48.4375_f64 / 7.75).sqrt(),
        torsional_j: 0.6,
        s33: 8.0,
        z33: 14.0,
        s22: 8.0,
        z22: 14.0,
        as2: 3.875,
        as3: 3.875,
        ..FrameSection::default()
    }
}

fn beam() -> FrameSection {
    make_i_section("IB", "A992Fy50", 16.0, 7.0, 0.5, 0.3, 7.0, 0.5)
}

fn aisc_members(angle_p: f64) -> Vec<Member> {
    vec![
        (
            "BEAM",
            beam(),
            [0.0, 0.0, 0.0],
            [120.0, 0.0, 0.0],
            vec![st(0.0, 5.0, 100.0, 0.0)],
        ),
        (
            "ANG",
            angle(),
            [0.0, 50.0, 0.0],
            [120.0, 50.0, 0.0],
            vec![st(angle_p, 0.0, 0.0, 0.0)],
        ),
    ]
}

fn aisc_model(angle_p: f64) -> SapModel {
    build(Units::KipInF, "AISC 360-10", "ACI 318-14", aisc_members(angle_p))
}

fn assert_angle_refused(report: &DesignRunReport, model: &SapModel) {
    assert_eq!(report.designed, 1);
    assert_eq!(report.not_designed.len(), 1);
    let (frame, error) = &report.not_designed[0];
    assert_eq!(frame, "ANG");
    assert!(matches!(error, DesignError::AiscLimitStateNotBuilt { .. }), "{error:?}");
    let text = &model.steel_design_errors["ANG"];
    assert_eq!(text, &error.to_string());
    assert!(text.contains("'ANG'") && text.contains("single angle"), "{text}");
    assert!(!model.steel_design_results.contains_key("ANG"));
}

#[test]
fn aisc_refused_angle_is_recorded_and_the_beam_is_designed() {
    let mut model = aisc_model(-1.0);
    let report = run_steel_design_per_member(&mut model).unwrap();
    assert_angle_refused(&report, &model);
    assert_eq!(model.steel_design_results.len(), 1);

    // the beam alone, in a model of its own
    let mut alone = build(Units::KipInF, "AISC 360-10", "ACI 318-14", vec![aisc_members(0.0).remove(0)]);
    try_run_steel_design(&mut alone).unwrap();
    same(&model.steel_design_results["BEAM"], &alone.steel_design_results["BEAM"], "beam result");
    assert!(alone.steel_design_errors.is_empty());
}

#[test]
fn aisc_strict_run_keeps_its_contract() {
    let mut model = aisc_model(-1.0);
    let err = try_run_steel_design(&mut model).unwrap_err();
    assert!(matches!(err, DesignError::AiscLimitStateNotBuilt { .. }), "{err:?}");
    assert!(model.steel_design_results.is_empty());
    assert!(model.steel_design_errors.is_empty());
}

fn s355(model: &mut SapModel) {
    let mat = model.materials.get_mut("A992Fy50").unwrap();
    mat.fy = 355.0;
    mat.e = 210_000.0;
    mat.g = 210_000.0 / 2.6;
}

/// N, mm, S355: a welded I 400 x 250 with tf 11 and tw 12 (web Class 3 in compression, designed)
/// and the same with tw 10 (web Class 4 in compression, refused: batch S, D90).
fn ec3_model() -> SapModel {
    let class3 = make_i_section("WI12", "A992Fy50", 400.0, 250.0, 11.0, 12.0, 250.0, 11.0);
    let class4 = make_i_section("WI10", "A992Fy50", 400.0, 250.0, 11.0, 10.0, 250.0, 11.0);
    let mut model = build(
        Units::NMmC,
        "Eurocode 3-2005",
        "Eurocode 2-2004",
        vec![
            ("OK3", class3, [0.0, 0.0, 0.0], [0.0, 0.0, 3000.0], vec![st(-500_000.0, 0.0, 0.0, 0.0)]),
            ("C4", class4, [1000.0, 0.0, 0.0], [1000.0, 0.0, 3000.0], vec![st(-500_000.0, 0.0, 0.0, 0.0)]),
        ],
    );
    s355(&mut model);
    model
}

#[test]
fn ec3_class_4_member_is_recorded_and_the_other_is_designed() {
    let mut model = ec3_model();
    let report = run_steel_design_per_member(&mut model).unwrap();
    assert_eq!(report.designed, 1);
    assert_eq!(report.not_designed.len(), 1);
    let (frame, error) = &report.not_designed[0];
    assert_eq!(frame, "C4");
    assert!(matches!(error, DesignError::Ec3SectionClass { .. }), "{error:?}");
    let text = &model.steel_design_errors["C4"];
    assert!(text.contains("'C4'") && text.contains("Class 4"), "{text}");
    assert!(!model.steel_design_results.contains_key("C4"));

    let mut alone = ec3_model();
    alone.frames.shift_remove("C4");
    alone.case_results.get_mut("LC").unwrap().frame_forces.shift_remove("C4");
    try_run_steel_design(&mut alone).unwrap();
    same(&model.steel_design_results["OK3"], &alone.steel_design_results["OK3"], "EC3 result");

    let mut strict = ec3_model();
    let err = try_run_steel_design(&mut strict).unwrap_err();
    assert!(matches!(err, DesignError::Ec3SectionClass { .. }), "{err:?}");
    assert!(strict.steel_design_results.is_empty() && strict.steel_design_errors.is_empty());
}

#[test]
fn a_rerun_and_every_invalidation_clear_the_error_map() {
    let mut model = aisc_model(-1.0);
    run_steel_design_per_member(&mut model).unwrap();
    assert_eq!(model.steel_design_errors.len(), 1);

    // the cause is removed (the angle is in tension now): no stale entry
    model
        .case_results
        .get_mut("LC")
        .unwrap()
        .frame_forces
        .insert("ANG".into(), vec![st(10.0, 0.0, 0.0, 0.0)]);
    let report = run_steel_design_per_member(&mut model).unwrap();
    assert_eq!(report.designed, 2);
    assert!(report.not_designed.is_empty());
    assert!(model.steel_design_errors.is_empty());
    assert_eq!(model.steel_design_results.len(), 2);

    for clear in 0..3 {
        let mut m = aisc_model(-1.0);
        run_steel_design_per_member(&mut m).unwrap();
        assert!(!m.steel_design_errors.is_empty());
        match clear {
            0 => m.invalidate_results(),
            1 => m.convert_units_in_place(Units::NMmC),
            _ => m.set_dc_ratio_limit(0.9).unwrap(),
        }
        assert!(m.steel_design_errors.is_empty(), "clear {clear}");
    }
}

#[test]
fn a_run_error_is_err_and_leaves_both_maps_empty() {
    let stale = |m: &mut SapModel| {
        m.steel_design_errors.insert("OLD".into(), "old".into());
        m.concrete_design_errors.insert("OLD".into(), "old".into());
    };
    // an unknown code name
    let mut m = aisc_model(-1.0);
    m.steel_design_code = "No such code".into();
    stale(&mut m);
    let err = run_steel_design_per_member(&mut m).unwrap_err();
    assert!(matches!(err, DesignError::UnsupportedSteelCode(_)), "{err:?}");
    assert!(m.steel_design_results.is_empty() && m.steel_design_errors.is_empty());

    // a combination without results
    let mut m = aisc_model(-1.0);
    m.load_combos.insert(
        "COMB".into(),
        LoadCombo {
            name: "COMB".into(),
            combo_type: ComboType::LinearAdd,
            cases: vec![("NOCASE".into(), 1.0)],
        },
    );
    stale(&mut m);
    let err = run_steel_design_per_member(&mut m).unwrap_err();
    assert!(matches!(err, DesignError::UnavailableCombination { .. }), "{err:?}");
    assert!(m.steel_design_results.is_empty() && m.steel_design_errors.is_empty());

    // a member without demand is a run error as well
    let mut m = aisc_model(-1.0);
    m.case_results.get_mut("LC").unwrap().frame_forces.shift_remove("BEAM");
    stale(&mut m);
    let err = run_steel_design_per_member(&mut m).unwrap_err();
    assert!(matches!(err, DesignError::MissingDemand { .. }), "{err:?}");
    assert!(m.steel_design_results.is_empty() && m.steel_design_errors.is_empty());

    // an invalid overwrite
    let mut m = aisc_model(-1.0);
    let mut ow = HashMap::new();
    ow.insert(
        "BEAM".to_string(),
        sap_design::SteelMemberOverwrite {
            k_major: Some(-1.0),
            ..Default::default()
        },
    );
    stale(&mut m);
    let err = run_steel_design_per_member_with_overwrites(&mut m, &ow).unwrap_err();
    assert!(matches!(err, DesignError::InvalidSteelOverwrite { .. }), "{err:?}");
    assert!(m.steel_design_results.is_empty() && m.steel_design_errors.is_empty());

    // concrete: an unknown code
    let mut m = aisc_model(-1.0);
    m.concrete_design_code = "No such code".into();
    stale(&mut m);
    let err = run_concrete_design_per_member(&mut m).unwrap_err();
    assert!(matches!(err, DesignError::UnsupportedConcreteCode(_)), "{err:?}");
    assert!(m.concrete_design_results.is_empty() && m.concrete_design_errors.is_empty());
}

// ---------------------------------------------------------------------------
// Auto Select
// ---------------------------------------------------------------------------

fn small_angle() -> FrameSection {
    let mut a = angle();
    a.name = "L6X6".into();
    a.area = 5.75;
    a.i33 = 19.9;
    a.i22 = 19.9;
    a.r33 = (19.9_f64 / 5.75).sqrt();
    a.r22 = a.r33;
    a.s33 = 4.6;
    a.s22 = 4.6;
    a.z33 = 8.3;
    a.z22 = 8.3;
    a.as2 = 2.875;
    a.as3 = 2.875;
    a
}

/// Frame "AS" has the list "LIST"; the other frame "BEAM" is an ordinary I beam.
fn auto_model(list: &[&str], as_p: f64) -> SapModel {
    let mut model = build(
        Units::KipInF,
        "AISC 360-10",
        "ACI 318-14",
        vec![
            ("BEAM", beam(), [0.0, 0.0, 0.0], [120.0, 0.0, 0.0], vec![st(0.0, 5.0, 100.0, 0.0)]),
            ("AS", beam(), [0.0, 50.0, 0.0], [120.0, 50.0, 0.0], vec![st(as_p, 0.0, 0.0, 0.0)]),
        ],
    );
    let mut heavy = beam();
    heavy.name = "IHEAVY".into();
    heavy.area *= 2.0;
    model.frame_sections.insert("IHEAVY".into(), heavy);
    model.frame_sections.insert("L8X8".into(), angle());
    model.frame_sections.insert("L6X6".into(), small_angle());
    let members: Vec<String> = list.iter().map(|s| s.to_string()).collect();
    model.define_auto_select_list("LIST", &members).unwrap();
    model.assign_auto_select_list("LIST", &["AS".to_string()]).unwrap();
    model
}

#[test]
fn auto_select_passes_over_a_refused_candidate() {
    // the lightest is the 6 x 6 angle (refused in compression), then the 8 x 8 angle (refused),
    // then the I section "IB"
    let mut model = auto_model(&["L6X6", "L8X8", "IB"], -1.0);
    let report = run_steel_design_per_member(&mut model).unwrap();
    assert_eq!(report.designed, 2);
    assert!(report.not_designed.is_empty());
    let r = &model.steel_design_results["AS"];
    assert_eq!(r.section_name, "IB");
    assert_eq!(model.frames["AS"].auto_select.as_ref().unwrap().design, "IB");
    let note = r
        .notes
        .iter()
        .find(|n| n.starts_with("Auto Select passed over 2 sections that are not designed: L6X6 ("))
        .unwrap_or_else(|| panic!("{:?}", r.notes));
    assert!(note.contains("single angle") && note.ends_with(", L8X8"), "{note}");

    // the same frame with a list of only the I section has no note and the same result
    let mut only = auto_model(&["IB"], -1.0);
    run_steel_design_per_member(&mut only).unwrap();
    let mut expected = only.steel_design_results["AS"].clone();
    expected.notes.push(note.clone());
    same(&model.steel_design_results["AS"], &expected, "chosen section result");

    // the strict form stops at the lightest candidate, as before
    let mut strict = auto_model(&["L6X6", "L8X8", "IB"], -1.0);
    let err = try_run_steel_design(&mut strict).unwrap_err();
    assert!(matches!(err, DesignError::AiscLimitStateNotBuilt { .. }), "{err:?}");
    assert!(strict.steel_design_results.is_empty() && strict.steel_design_errors.is_empty());
}

#[test]
fn auto_select_with_no_designable_section_refuses_the_frame_and_forgets_the_choice() {
    // tension: both angles are designed and the lighter passing one is chosen
    let mut model = auto_model(&["L6X6", "L8X8"], 10.0);
    let report = run_steel_design_per_member(&mut model).unwrap();
    assert_eq!(report.designed, 2);
    assert!(!model.frames["AS"].auto_select.as_ref().unwrap().design.is_empty());
    let designed_choice = model.frames["AS"].auto_select.as_ref().unwrap().design.clone();
    assert_eq!(designed_choice, "L6X6");

    // compression: every candidate is refused
    model
        .case_results
        .get_mut("LC")
        .unwrap()
        .frame_forces
        .insert("AS".into(), vec![st(-1.0, 0.0, 0.0, 0.0)]);
    let report = run_steel_design_per_member(&mut model).unwrap();
    assert_eq!(report.designed, 1);
    assert_eq!(report.not_designed.len(), 1);
    assert_eq!(report.not_designed[0].0, "AS");
    match &report.not_designed[0].1 {
        DesignError::AutoSelectNoSection { frame, list, first } => {
            assert_eq!((frame.as_str(), list.as_str()), ("AS", "LIST"));
            assert!(first.contains("single angle"), "{first}");
        }
        other => panic!("{other:?}"),
    }
    assert!(model.steel_design_errors["AS"].contains("no section of the Auto Select list 'LIST'"));
    assert!(!model.steel_design_results.contains_key("AS"));
    assert!(model.steel_design_results.contains_key("BEAM"));
    // no stale choice: the design section is empty, as for a frame that was never designed
    assert_eq!(model.frames["AS"].auto_select.as_ref().unwrap().design, "");

    // strict: the first candidate's own error, nothing stored
    let mut strict = model.clone();
    let err = try_run_steel_design(&mut strict).unwrap_err();
    assert!(matches!(err, DesignError::AiscLimitStateNotBuilt { .. }), "{err:?}");
    assert!(strict.steel_design_results.is_empty() && strict.steel_design_errors.is_empty());
}

// ---------------------------------------------------------------------------
// concrete
// ---------------------------------------------------------------------------

fn concrete_material(model: &mut SapModel, name: &str, fc: f64, fy: f64) {
    let mut c = Material::default_concrete_kip_in();
    c.name = name.into();
    c.fc = fc;
    c.fy = fy;
    c.fys = fy;
    if fc > 20.0 {
        c.e = 33_000.0;
    }
    model.materials.insert(name.into(), c);
}

/// A rectangular beam and a column (designed) and a beam of another shape with torsion (refused,
/// G85). `kip_in` selects ACI in kip and in, else EC2 in N and mm.
fn concrete_model(kip_in: bool) -> SapModel {
    let (units, mat, code, len, b, h) = if kip_in {
        (Units::KipInF, "C4", "ACI 318-14", 120.0, 12.0, 24.0)
    } else {
        (Units::NMmC, "C30", "Eurocode 2-2004", 3000.0, 300.0, 500.0)
    };
    let (pu, vu, mu, tu) = if kip_in {
        (-100.0, 30.0, 1500.0, 80.0)
    } else {
        (-500_000.0, 100_000.0, 150.0e6, 20.0e6)
    };
    let rect = make_rectangular("RB", mat, h, b);
    let col = make_rectangular("RC", mat, h, h);
    let mut other = make_rectangular("OB", mat, h, b);
    other.prop_type = FramePropType::I;
    let mut model = build(
        units,
        "AISC 360-10",
        code,
        vec![
            ("BM", rect, [0.0, 0.0, 0.0], [len, 0.0, 0.0], vec![st(0.0, vu, mu, 0.0)]),
            ("COL", col, [0.0, 500.0, 0.0], [0.0, 500.0, len], vec![st(pu, vu, mu, 0.0)]),
            ("TB", other, [0.0, 1000.0, 0.0], [len, 1000.0, 0.0], vec![st(0.0, vu, mu, tu)]),
        ],
    );
    if kip_in {
        concrete_material(&mut model, "C4", 4.0, 60.0);
    } else {
        concrete_material(&mut model, "C30", 30.0, 500.0);
    }
    model
}

fn without_torsion_beam(mut m: SapModel) -> SapModel {
    m.frames.shift_remove("TB");
    m.case_results.get_mut("LC").unwrap().frame_forces.shift_remove("TB");
    m
}

#[test]
fn concrete_refused_beam_is_recorded_under_aci_and_ec2() {
    for kip_in in [true, false] {
        let mut model = concrete_model(kip_in);
        let report = run_concrete_design_per_member(&mut model).unwrap();
        assert_eq!(report.designed, 2, "kip_in {kip_in}");
        assert_eq!(report.not_designed.len(), 1);
        let (frame, error) = &report.not_designed[0];
        assert_eq!(frame, "TB");
        assert!(matches!(error, DesignError::ConcreteTorsionNotDesigned { .. }), "{error:?}");
        let text = &model.concrete_design_errors["TB"];
        assert_eq!(text, &error.to_string());
        assert!(text.contains("'TB'") && text.contains("torsion"), "{text}");
        // the refused beam has no result at all
        assert!(!model.concrete_design_results.contains_key("TB"));
        assert_eq!(model.concrete_design_results.len(), 2);

        let mut alone = without_torsion_beam(concrete_model(kip_in));
        try_run_concrete_design(&mut alone).unwrap();
        for f in ["BM", "COL"] {
            same(&model.concrete_design_results[f], &alone.concrete_design_results[f], f);
        }

        let mut strict = concrete_model(kip_in);
        let err = try_run_concrete_design(&mut strict).unwrap_err();
        assert!(matches!(err, DesignError::ConcreteTorsionNotDesigned { .. }), "{err:?}");
        assert!(strict.concrete_design_results.is_empty() && strict.concrete_design_errors.is_empty());

        // rerun without the cause: no stale entry
        let mut again = model.clone();
        again.case_results.get_mut("LC").unwrap().frame_forces.insert(
            "TB".into(),
            vec![st(0.0, if kip_in { 30.0 } else { 100_000.0 }, if kip_in { 1500.0 } else { 150.0e6 }, 0.0)],
        );
        let report = run_concrete_design_per_member(&mut again).unwrap();
        assert!(report.not_designed.is_empty());
        assert!(again.concrete_design_errors.is_empty());
        assert_eq!(again.concrete_design_results.len(), 3);
    }
}

#[test]
fn concrete_member_without_rebar_yield_does_not_stop_the_good_one() {
    let mut model = without_torsion_beam(concrete_model(true));
    // the column's section names a concrete without any fy
    concrete_material(&mut model, "C4NOFY", 4.0, 0.0);
    model.frame_sections.get_mut("RC").unwrap().material = "C4NOFY".into();
    let report = run_concrete_design_per_member(&mut model).unwrap();
    assert_eq!(report.designed, 1);
    assert_eq!(report.not_designed.len(), 1);
    assert!(matches!(report.not_designed[0].1, DesignError::MissingRebarYield { .. }));
    assert!(model.concrete_design_errors["COL"].contains("'COL'"));
    assert!(!model.concrete_design_results.contains_key("COL"));

    let mut alone = without_torsion_beam(concrete_model(true));
    alone.frames.shift_remove("COL");
    alone.case_results.get_mut("LC").unwrap().frame_forces.shift_remove("COL");
    try_run_concrete_design(&mut alone).unwrap();
    same(&model.concrete_design_results["BM"], &alone.concrete_design_results["BM"], "beam");

    // strict: the error of the first refused member in the frame order, nothing stored
    let mut strict = without_torsion_beam(concrete_model(true));
    concrete_material(&mut strict, "C4NOFY", 4.0, 0.0);
    strict.frame_sections.get_mut("RC").unwrap().material = "C4NOFY".into();
    let err = try_run_concrete_design(&mut strict).unwrap_err();
    assert!(matches!(err, DesignError::MissingRebarYield { .. }), "{err:?}");
    assert!(strict.concrete_design_results.is_empty() && strict.concrete_design_errors.is_empty());
}

// ---------------------------------------------------------------------------
// classes of error, verification
// ---------------------------------------------------------------------------

#[test]
fn is_member_refusal_by_class() {
    let member = [
        DesignError::AiscLimitStateNotBuilt {
            frame: "F".into(),
            section: "S".into(),
            kind: "Angle".into(),
            what: "w".into(),
        },
        DesignError::MissingRebarYield {
            frame: "F".into(),
            section: "S".into(),
            material: "M".into(),
        },
        DesignError::UnsupportedClosedSectionTorsion { frame: "F".into() },
        DesignError::AutoSelectNoSection {
            frame: "F".into(),
            list: "L".into(),
            first: "x".into(),
        },
    ];
    for e in &member {
        assert!(e.is_member_refusal(), "{e:?}");
    }
    let run = [
        DesignError::UnsupportedSteelCode("x".into()),
        DesignError::UnsupportedConcreteCode("x".into()),
        DesignError::UnavailableCombination {
            discipline: "steel",
            combo: "C".into(),
            reason: "r".into(),
        },
        DesignError::MissingDemand {
            discipline: "steel",
            frame: "F".into(),
        },
        DesignError::Workflow("w".into()),
    ];
    for e in &run {
        assert!(!e.is_member_refusal(), "{e:?}");
    }
}

#[test]
fn verify_all_members_passed_counts_refused_members() {
    let mut model = aisc_model(-1.0);
    run_steel_design_per_member(&mut model).unwrap();
    model.is_locked = true;
    // the beam passes, the angle was not designed: it did not pass
    assert_eq!(verify_all_members_passed(&model).unwrap(), vec!["ANG".to_string()]);

    // every member refused: no results at all, the refused frames are still returned
    let mut all = build(
        Units::KipInF,
        "AISC 360-10",
        "ACI 318-14",
        vec![aisc_members(-1.0).remove(1)],
    );
    run_steel_design_per_member(&mut all).unwrap();
    assert!(all.steel_design_results.is_empty());
    all.is_locked = true;
    assert_eq!(verify_all_members_passed(&all).unwrap(), vec!["ANG".to_string()]);

    // nothing designed and nothing refused is still the error of today
    let mut none = aisc_model(-1.0);
    none.is_locked = true;
    assert!(verify_all_members_passed(&none).is_err());
}

#[test]
fn verify_analysis_vs_design_lists_a_refused_auto_select_frame() {
    let mut model = auto_model(&["L6X6", "L8X8"], -1.0);
    run_steel_design_per_member(&mut model).unwrap();
    model.is_locked = true;
    assert_eq!(model.frames["AS"].auto_select.as_ref().unwrap().design, "");
    assert_eq!(verify_analysis_vs_design(&model).unwrap(), vec!["AS".to_string()]);
}
