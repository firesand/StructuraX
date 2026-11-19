//! Persistence of frame hinge properties, hinge assignments and overwrites,
//! and the parameters of nonlinear static cases through S2K reconstruction
//! extension tables and columns. The genuine SAP2000 hinge table names are
//! evidenced but their columns are not, so the extension tables are
//! "RUST ..." tables; text with genuine hinge tables is kept verbatim and
//! marks the nonlinear cases as depending on unmodeled data.

use sap_format::{parse_s2k, parse_s2k_with_report, write_s2k, FormatError};
use sap_model::hinge::*;
use sap_model::*;

const EXTENSION_MARKERS: [&str; 12] = [
    "RUST HINGE PROPERTIES",
    "RUST FRAME HINGE ASSIGNMENTS",
    "RUST FRAME HINGE OVERWRITES",
    "RustResultsSaved",
    "RustMinSaved",
    "RustMaxSaved",
    "RustPositiveOnly",
    "RustMaxTotalSteps",
    "RustMaxNullSteps",
    "RustHingeUnloading",
    "LoadType=Mode",
    "LoadType=Accel",
];

fn base() -> SapModel {
    let mut m = SapModel::default();
    m.add_point("A", 0., 0., 0.);
    m.add_point("B", 12., 0., 0.);
    m.add_frame("F", "A", "B", "FSEC1");
    m
}

fn curve(values: [(f64, f64); 4]) -> [HingePoint; 4] {
    values.map(|(force, deformation)| HingePoint::new(force, deformation))
}

/// Non-round numbers on purpose: the text must carry every bit.
fn awkward() -> f64 {
    0.1 + 0.2
}

fn rich_model() -> SapModel {
    let mut m = base();
    m.add_frame("G", "A", "B", "FSEC1");
    m.add_frame("H", "A", "B", "FSEC1");

    // Symmetric, the vendor's example 1-026 moment hinge.
    let mut moment = HingeProp::symmetric(
        "Moment",
        HingeDof::M3,
        curve([(1440., 0.), (1920., 0.04), (480., 0.04), (480., 0.4)]),
    );
    moment.acceptance_positive = HingeAcceptance { io: 2., ls: 4., cp: 6. };
    moment.acceptance_negative = HingeAcceptance { io: -2., ls: -4., cp: -6. };
    moment.beyond_e = None;
    m.hinge_props.insert("Moment".into(), moment);

    // Not symmetric, scale factors, every non-default setting.
    let shear = HingeProp {
        name: "Shear 2".into(),
        dof: HingeDof::V2,
        behavior: HingeBehavior::ForceControlled,
        symmetric: false,
        positive: curve([(70., 0.), (80., awkward()), (90., 0.8), (100., 1.2)]),
        negative: curve([(-65., 0.), (-75., -0.3), (-85., -0.7), (-95., -1.1)]),
        force_sf: 1.5,
        deformation_sf: 0.001,
        force_sf_negative: 2.5,
        deformation_sf_negative: 0.002,
        acceptance_positive: HingeAcceptance { io: 0.1, ls: 0.2, cp: 0.3 },
        acceptance_negative: HingeAcceptance { io: -0.1, ls: -0.2, cp: -0.3 },
        beyond_e: Some(HingeBeyondE::Extrapolated),
    };
    m.hinge_props.insert("Shear 2".into(), shear);

    // Symmetric flag set but a negative branch that is not the mirror image:
    // it is stored, so it must come back.
    let mut odd = HingeProp::symmetric(
        "P-axial",
        HingeDof::P,
        curve([(10., 0.), (12., 1e-9), (3., 2e-9), (3., 1e-3)]),
    );
    odd.negative = curve([(-1., 0.), (-2., -0.5), (-3., -0.6), (-4., -0.7)]);
    m.hinge_props.insert("P-axial".into(), odd);

    m.frames["F"].hinges = vec![
        FrameHinge { prop: "Moment".into(), rel_dist: 0.0 },
        FrameHinge { prop: "Shear 2".into(), rel_dist: awkward() },
        FrameHinge { prop: "Moment".into(), rel_dist: 1.0 },
    ];
    m.frames["F"].hinge_overwrite = Some(FrameHingeOverwrite {
        auto_subdivide: false,
        relative_length: 0.05,
        do_not_drop_load: true,
    });
    m.frames["G"].hinges = vec![FrameHinge { prop: "P-axial".into(), rel_dist: 0.5 }];
    m.frames["G"].hinge_overwrite = Some(FrameHingeOverwrite::default());
    // H has neither.

    m.load_patterns.insert(
        "PUSH PAT".into(),
        LoadPattern {
            name: "PUSH PAT".into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
    let mut modal = LoadCase::modal_eigen("MODES", 6);
    modal.loads.clear();
    m.load_cases.insert("MODES".into(), modal);

    let mut push = LoadCase::linear_static("PUSH", "PUSH PAT", 1.0);
    push.case_type = LoadCaseType::NonlinearStatic;
    push.loads = vec![
        LoadCaseAssignment { load_name: "PUSH PAT".into(), scale_factor: awkward() },
        LoadCaseAssignment { load_name: "UX".into(), scale_factor: -2.5 },
        LoadCaseAssignment { load_name: "U3".into(), scale_factor: 1e-3 },
    ];
    push.nl_static = NonlinearStaticParams {
        results_saved: ResultsSaved::MultipleStates {
            min_saved: 6,
            max_saved: 10,
            positive_only: false,
        },
        max_total_steps: 500,
        max_null_steps: 7,
        hinge_unloading: HingeUnloading::RestartUsingSecantStiffness,
        mode_loads: Vec::new(),
    };
    m.load_cases.insert("PUSH".into(), push);

    let mut modal_push = LoadCase::linear_static("MODEPUSH", "PUSH PAT", 1.0);
    modal_push.case_type = LoadCaseType::NonlinearStatic;
    modal_push.loads.clear();
    modal_push.modal_case = Some("MODES".into());
    modal_push.initial_case = Some("PUSH".into());
    modal_push.nl_static.mode_loads = vec![
        ModeLoad { mode: 1, scale_factor: awkward() },
        ModeLoad { mode: 3, scale_factor: -1.0 },
    ];
    modal_push.nl_static.hinge_unloading = HingeUnloading::ApplyLocalRedistribution;
    m.load_cases.insert("MODEPUSH".into(), modal_push);
    m
}

#[test]
fn a_model_without_hinge_data_writes_none_of_the_extension_tables_or_columns() {
    let text = write_s2k(&base());
    for marker in EXTENSION_MARKERS {
        assert!(!text.contains(marker), "unexpected {marker}");
    }
    // A nonlinear static case with default parameters writes nothing either,
    // and keeps writing its load pattern as before.
    let mut m = base();
    let mut push = LoadCase::linear_static("PUSH", "DEAD", 1.0);
    push.case_type = LoadCaseType::NonlinearStatic;
    m.load_cases.insert("PUSH".into(), push);
    let text = write_s2k(&m);
    for marker in EXTENSION_MARKERS {
        assert!(!text.contains(marker), "unexpected {marker}");
    }
    let back = parse_s2k(&text).unwrap();
    assert!(back.load_cases["PUSH"].nl_static.is_default());
    assert!(back.hinge_props.is_empty());
    assert!(back.frames["F"].hinges.is_empty());
    assert!(back.frames["F"].hinge_overwrite.is_none());
}

#[test]
fn s2k_keeps_hinge_properties_assignments_overwrites_and_case_parameters_exactly() {
    let m = rich_model();
    let text = write_s2k(&m);
    let back = parse_s2k(&text).unwrap();

    assert_eq!(back.hinge_props, m.hinge_props);
    // The order of the properties is the order of definition.
    assert_eq!(
        back.hinge_props.keys().collect::<Vec<_>>(),
        m.hinge_props.keys().collect::<Vec<_>>()
    );
    assert_eq!(back.hinge_props["Moment"].beyond_e, None);
    assert_eq!(
        back.hinge_props["Shear 2"].beyond_e,
        Some(HingeBeyondE::Extrapolated)
    );
    for name in ["F", "G", "H"] {
        assert_eq!(back.frames[name].hinges, m.frames[name].hinges, "{name}");
        assert_eq!(
            back.frames[name].hinge_overwrite, m.frames[name].hinge_overwrite,
            "{name}"
        );
    }
    for name in ["PUSH", "MODEPUSH", "MODES"] {
        assert_eq!(back.load_cases[name].nl_static, m.load_cases[name].nl_static, "{name}");
        assert_eq!(back.load_cases[name].loads, m.load_cases[name].loads, "{name}");
        assert_eq!(back.load_cases[name].modal_case, m.load_cases[name].modal_case, "{name}");
        assert_eq!(back.load_cases[name].case_type, m.load_cases[name].case_type, "{name}");
        assert_eq!(
            back.load_cases[name].initial_case, m.load_cases[name].initial_case,
            "{name}"
        );
    }
    assert_eq!(back.unmodeled_source, m.unmodeled_source);
    // A second pass writes the same text: nothing is lost or invented.
    assert_eq!(write_s2k(&back), text);
}

#[test]
fn every_enumerated_value_survives() {
    for dof in HingeDof::ALL {
        for behavior in [HingeBehavior::DeformationControlled, HingeBehavior::ForceControlled] {
            for beyond_e in [
                None,
                Some(HingeBeyondE::DropsToZero),
                Some(HingeBeyondE::Extrapolated),
            ] {
                let mut m = base();
                let mut prop = HingeProp::symmetric(
                    "H1",
                    dof,
                    curve([(5., 0.), (6., 0.1), (2., 0.1), (2., 0.5)]),
                );
                prop.behavior = behavior;
                prop.beyond_e = beyond_e;
                m.hinge_props.insert("H1".into(), prop);
                m.frames["F"].hinges.push(FrameHinge { prop: "H1".into(), rel_dist: 0.25 });
                let back = parse_s2k(&write_s2k(&m)).unwrap();
                assert_eq!(back.hinge_props, m.hinge_props, "{dof:?} {behavior:?} {beyond_e:?}");
            }
        }
    }
    for unloading in [
        HingeUnloading::UnloadEntireStructure,
        HingeUnloading::ApplyLocalRedistribution,
        HingeUnloading::RestartUsingSecantStiffness,
    ] {
        for results_saved in [
            ResultsSaved::FinalState,
            ResultsSaved::multiple_default(),
            ResultsSaved::MultipleStates { min_saved: 1, max_saved: 1, positive_only: false },
        ] {
            let mut m = base();
            let mut push = LoadCase::linear_static("PUSH", "DEAD", 1.0);
            push.case_type = LoadCaseType::NonlinearStatic;
            push.nl_static.hinge_unloading = unloading;
            push.nl_static.results_saved = results_saved;
            m.load_cases.insert("PUSH".into(), push);
            let text = write_s2k(&m);
            let back = parse_s2k(&text).unwrap();
            assert_eq!(back.load_cases["PUSH"].nl_static, m.load_cases["PUSH"].nl_static);
            assert_eq!(write_s2k(&back), text);
        }
    }
}

#[test]
fn only_the_non_default_case_columns_are_written() {
    let mut m = base();
    let mut push = LoadCase::linear_static("PUSH", "DEAD", 1.0);
    push.case_type = LoadCaseType::NonlinearStatic;
    push.nl_static.max_total_steps = 300;
    m.load_cases.insert("PUSH".into(), push);
    let text = write_s2k(&m);
    assert!(text.contains("RustMaxTotalSteps=300"));
    for absent in [
        "RustResultsSaved",
        "RustMinSaved",
        "RustMaxNullSteps",
        "RustHingeUnloading",
        "RustPositiveOnly",
    ] {
        assert!(!text.contains(absent), "{absent}");
    }
}

#[test]
fn hand_written_text_is_read() {
    // The negative branch of a symmetric property is optional text; it takes
    // the mirror image. The case columns take the vendor defaults for what
    // they leave out.
    let mut text = write_s2k(&base());
    text = text.replace(
        "END TABLE DATA",
        r#"TABLE:  "RUST HINGE PROPERTIES"
   Name=MH   DOF=m3   Symmetric=Yes   DeformSF=0.5

TABLE:  "RUST HINGE PROPERTIES - CURVE"
   Name=MH   Point=B   Force=1440   Deform=0
   Name=MH   Point=C   Force=1920   Deform=0.04
   Name=MH   Point=D   Force=480   Deform=0.04
   Name=MH   Point=E   Force=480   Deform=0.4

TABLE:  "RUST FRAME HINGE ASSIGNMENTS"
   Frame=F   HingeNum=1   Prop=MH   RelDist=0
   Frame=F   HingeNum=2   Prop=MH   RelDist=1

TABLE:  "RUST FRAME HINGE OVERWRITES"
   Frame=F   AutoSubdivide=Yes

END TABLE DATA"#,
    );
    // Written before the case it belongs to, on purpose: table order in the
    // file does not matter.
    text = text.replace(
        "TABLE:  \"RUST HINGE PROPERTIES\"",
        "TABLE:  \"CASE - STATIC 1 - LOAD ASSIGNMENTS\"\n   Case=DEAD   LoadType=Accel   LoadName=U1   LoadSF=2\n\nTABLE:  \"RUST HINGE PROPERTIES\"",
    );
    let m = parse_s2k(&text).unwrap();
    let prop = &m.hinge_props["MH"];
    assert_eq!(prop.dof, HingeDof::M3);
    assert_eq!(prop.deformation_sf, 0.5);
    assert_eq!(prop.force_sf, 1.0);
    assert_eq!(prop.beyond_e, Some(HingeBeyondE::DropsToZero));
    assert_eq!(prop.positive[1], HingePoint::new(1920., 0.04));
    assert_eq!(prop.negative[1], HingePoint::new(-1920., -0.04));
    assert_eq!(m.frames["F"].hinges.len(), 2);
    assert_eq!(m.frames["F"].hinges[1].rel_dist, 1.0);
    assert_eq!(
        m.frames["F"].hinge_overwrite,
        Some(FrameHingeOverwrite { auto_subdivide: true, relative_length: 0.02, do_not_drop_load: false })
    );
    assert!(m.load_cases["DEAD"]
        .loads
        .iter()
        .any(|l| l.load_name == "U1" && l.scale_factor == 2.0));
    // The source text is not canonical, so it is kept; the model it gives
    // reopens to an equal model.
    let report = parse_s2k_with_report(&text).unwrap();
    assert!(report.model.s2k_preservation.is_some());
    let reopened = parse_s2k(&write_s2k(&report.model)).unwrap();
    assert_eq!(reopened.hinge_props, m.hinge_props);
    assert_eq!(reopened.frames["F"].hinges, m.frames["F"].hinges);
}

#[test]
fn genuine_vendor_hinge_tables_mark_nonlinear_cases_as_unmodeled() {
    for table in [
        "HINGES DEF 01 - OVERVIEW",
        "FRAME HINGE ASSIGNS 01 - OVERVIEW",
        "FRAME HINGE ASSIGNS 09 - HINGE OVERWRITES",
    ] {
        let text = write_s2k(&base()).replace(
            "END TABLE DATA",
            &format!("TABLE:  \"{table}\"\n   Frame=F   Anything=1\n\nEND TABLE DATA"),
        );
        let m = parse_s2k(&text).unwrap();
        assert_eq!(m.unmodeled_source.len(), 1, "{table}");
        assert_eq!(m.unmodeled_source[0].scope, UnmodeledScope::NonlinearCases);
        assert!(m.unmodeled_source[0].description.contains(table), "{table}");
        // Reopening what we write keeps the entry, and does not add a second.
        let again = parse_s2k(&write_s2k(&m)).unwrap();
        assert_eq!(again.unmodeled_source, m.unmodeled_source);
    }
    // An empty vendor table is not data.
    let text = write_s2k(&base()).replace(
        "END TABLE DATA",
        "TABLE:  \"HINGES DEF 01 - OVERVIEW\"\n\nEND TABLE DATA",
    );
    assert!(parse_s2k(&text).unwrap().unmodeled_source.is_empty());
}

fn good_text() -> String {
    let mut m = base();
    m.hinge_props.insert(
        "MH".into(),
        HingeProp::symmetric(
            "MH",
            HingeDof::M3,
            curve([(1440., 0.), (1920., 0.04), (480., 0.04), (480., 0.4)]),
        ),
    );
    m.frames["F"].hinges.push(FrameHinge { prop: "MH".into(), rel_dist: 0.5 });
    m.frames["F"].hinge_overwrite = Some(FrameHingeOverwrite::default());
    let mut push = LoadCase::linear_static("PUSH", "DEAD", 1.0);
    push.case_type = LoadCaseType::NonlinearStatic;
    push.nl_static.results_saved = ResultsSaved::multiple_default();
    push.nl_static.mode_loads = vec![ModeLoad { mode: 2, scale_factor: 1.0 }];
    push.loads.push(LoadCaseAssignment { load_name: "UY".into(), scale_factor: 1.0 });
    m.load_cases.insert("PUSH".into(), push);
    write_s2k(&m)
}

fn replaced(text: &str, from: &str, to: &str) -> String {
    assert_eq!(text.matches(from).count(), 1, "'{from}' must occur once in:\n{text}");
    text.replace(from, to)
}

fn assert_import_error(text: &str, needles: &[&str]) {
    let error = parse_s2k(text).unwrap_err();
    assert!(matches!(error, FormatError::ParseS2k(_)), "{error}");
    for needle in needles {
        assert!(error.to_string().contains(needle), "'{needle}' missing from: {error}");
    }
}

#[test]
fn the_good_text_reads() {
    let m = parse_s2k(&good_text()).unwrap();
    assert_eq!(m.hinge_props.len(), 1);
    assert_eq!(m.load_cases["PUSH"].nl_static.mode_loads.len(), 1);
}

#[test]
fn unreadable_or_out_of_range_values_are_import_errors_naming_table_and_value() {
    let good = good_text();
    let table = "RUST HINGE PROPERTIES";
    let curve_table = "RUST HINGE PROPERTIES - CURVE";
    let cases: Vec<(String, Vec<&str>)> = vec![
        // Hinge properties.
        (replaced(&good, "DOF=M3", "DOF=Q9"), vec![table, "Q9"]),
        (replaced(&good, "DOF=M3", "DOF=\"P-M3\""), vec![table, "P-M3"]),
        (
            replaced(&good, "Behavior=DeformationControlled", "Behavior=Brittle"),
            vec![table, "Brittle"],
        ),
        (replaced(&good, "Symmetric=Yes", "Symmetric=Maybe"), vec![table, "Maybe"]),
        (replaced(&good, "   ForceSF=1 ", "   ForceSF=abc "), vec![table, "abc"]),
        (replaced(&good, "   ForceSF=1 ", "   ForceSF=NaN "), vec![table, "NaN"]),
        (replaced(&good, "   DeformSF=1 ", "   DeformSF=inf "), vec![table, "inf"]),
        (replaced(&good, "   ForceSF=1 ", "   ForceSF=0 "), vec![table, "scale factors"]),
        (replaced(&good, "   ForceSF=1 ", "   ForceSF=-1 "), vec![table, "scale factors"]),
        (replaced(&good, "BeyondE=DropsToZero", "BeyondE=Forever"), vec![table, "Forever"]),
        (replaced(&good, "   IO=0 ", "   IO=1 "), vec![table, "acceptance"]),
        (replaced(&good, "   IO=0 ", "   IO=x1 "), vec![table, "x1"]),
        // Curve.
        (
            replaced(&good, "Name=MH   Point=B   Force=1440", "Name=MH   Point=B   Force=oops"),
            vec![curve_table, "oops"],
        ),
        (
            replaced(&good, "Point=C   Force=1920   Deform=0.04", "Point=C   Force=1920   Deform=1x"),
            vec![curve_table, "1x"],
        ),
        (
            replaced(&good, "Point=C   Force=1920", "Point=Z   Force=1920"),
            vec![curve_table, "Z"],
        ),
        (
            replaced(&good, "Name=MH   Point=D", "Name=NOPE   Point=D"),
            vec![curve_table, "NOPE"],
        ),
        (
            replaced(&good, "Point=C   Force=1920   Deform=0.04", "Point=C   Force=-5   Deform=0.04"),
            vec![table, "not be negative"],
        ),
        (
            replaced(&good, "Point=E   Force=480   Deform=0.4", "Point=E   Force=480   Deform=0.01"),
            vec![table, "deformations must not decrease"],
        ),
        (
            replaced(&good, "Name=MH   Point=B   Force=1440   Deform=0\n", ""),
            vec![curve_table, "no point B"],
        ),
        (
            replaced(
                &good,
                "Name=MH   Point=B   Force=1440   Deform=0\n",
                "Name=MH   Point=B   Force=1440   Deform=0\n   Name=MH   Point=B   Force=1440   Deform=0\n",
            ),
            vec![curve_table, "twice"],
        ),
        // Assignments.
        (replaced(&good, "RelDist=0.5", "RelDist=1.5"), vec!["RUST FRAME HINGE ASSIGNMENTS", "1.5"]),
        (replaced(&good, "RelDist=0.5", "RelDist=-0.1"), vec!["RUST FRAME HINGE ASSIGNMENTS", "-0.1"]),
        (replaced(&good, "RelDist=0.5", "RelDist=half"), vec!["RUST FRAME HINGE ASSIGNMENTS", "half"]),
        (replaced(&good, "HingeNum=1", "HingeNum=2"), vec!["RUST FRAME HINGE ASSIGNMENTS", "HingeNum 2"]),
        (replaced(&good, "HingeNum=1", "HingeNum=one"), vec!["RUST FRAME HINGE ASSIGNMENTS", "one"]),
        (replaced(&good, "Prop=MH", "Prop=GONE"), vec!["RUST FRAME HINGE ASSIGNMENTS", "GONE"]),
        (
            replaced(&good, "Frame=F   HingeNum=1", "Frame=NOFRAME   HingeNum=1"),
            vec!["RUST FRAME HINGE ASSIGNMENTS", "NOFRAME"],
        ),
        // Overwrites.
        (replaced(&good, "RelLength=0.02", "RelLength=0"), vec!["RUST FRAME HINGE OVERWRITES", "RelLength 0"]),
        (replaced(&good, "RelLength=0.02", "RelLength=1.5"), vec!["RUST FRAME HINGE OVERWRITES", "1.5"]),
        (replaced(&good, "RelLength=0.02", "RelLength=wide"), vec!["RUST FRAME HINGE OVERWRITES", "wide"]),
        (replaced(&good, "AutoSubdivide=Yes", "AutoSubdivide=Perhaps"), vec!["RUST FRAME HINGE OVERWRITES", "Perhaps"]),
        (replaced(&good, "DoNotDropLoad=No", "DoNotDropLoad=2"), vec!["RUST FRAME HINGE OVERWRITES", "'2'"]),
        (
            replaced(&good, "Frame=F   AutoSubdivide", "Frame=NOFRAME   AutoSubdivide"),
            vec!["RUST FRAME HINGE OVERWRITES", "NOFRAME"],
        ),
        // Case columns.
        (replaced(&good, "RustResultsSaved=Multiple", "RustResultsSaved=Some"), vec!["LOAD CASE DEFINITIONS", "Some"]),
        (replaced(&good, "RustMinSaved=10", "RustMinSaved=ten"), vec!["LOAD CASE DEFINITIONS", "ten"]),
        (replaced(&good, "RustMinSaved=10", "RustMinSaved=-3"), vec!["LOAD CASE DEFINITIONS", "-3"]),
        (replaced(&good, "RustMinSaved=10", "RustMinSaved=0"), vec!["LOAD CASE DEFINITIONS", "RustMinSaved 0"]),
        (replaced(&good, "RustMinSaved=10", "RustMinSaved=101"), vec!["LOAD CASE DEFINITIONS", "RustMaxSaved 100"]),
        (replaced(&good, "RustPositiveOnly=Yes", "RustPositiveOnly=Sometimes"), vec!["LOAD CASE DEFINITIONS", "Sometimes"]),
        (
            replaced(&good, "RustResultsSaved=Multiple   ", ""),
            vec!["LOAD CASE DEFINITIONS", "without RustResultsSaved=Multiple"],
        ),
        // The following columns are added to the row of case PUSH.
        (
            replaced(&good, "RustPositiveOnly=Yes", "RustPositiveOnly=Yes   RustMaxTotalSteps=0"),
            vec!["LOAD CASE DEFINITIONS", "RustMaxTotalSteps 0"],
        ),
        (
            replaced(&good, "RustPositiveOnly=Yes", "RustPositiveOnly=Yes   RustMaxTotalSteps=1.5"),
            vec!["LOAD CASE DEFINITIONS", "1.5"],
        ),
        (
            replaced(&good, "RustPositiveOnly=Yes", "RustPositiveOnly=Yes   RustMaxNullSteps=-1"),
            vec!["LOAD CASE DEFINITIONS", "-1"],
        ),
        (
            replaced(&good, "RustPositiveOnly=Yes", "RustPositiveOnly=Yes   RustHingeUnloading=Secant"),
            vec!["LOAD CASE DEFINITIONS", "Secant"],
        ),
        // Load types.
        (replaced(&good, "LoadType=Mode", "LoadType=Spectrum"), vec!["CASE - STATIC 1 - LOAD ASSIGNMENTS", "Spectrum"]),
        (replaced(&good, "LoadType=Accel", "LoadType=Gravity"), vec!["CASE - STATIC 1 - LOAD ASSIGNMENTS", "Gravity"]),
        (replaced(&good, "LoadName=UY", "LoadName=UW"), vec!["CASE - STATIC 1 - LOAD ASSIGNMENTS", "UW"]),
        (replaced(&good, "LoadName=UY", "LoadName=U4"), vec!["CASE - STATIC 1 - LOAD ASSIGNMENTS", "U4"]),
        (replaced(&good, "LoadName=2   LoadSF=1", "LoadName=0   LoadSF=1"), vec!["CASE - STATIC 1 - LOAD ASSIGNMENTS", "'0'"]),
        (replaced(&good, "LoadName=2   LoadSF=1", "LoadName=two   LoadSF=1"), vec!["CASE - STATIC 1 - LOAD ASSIGNMENTS", "two"]),
        (replaced(&good, "LoadName=2   LoadSF=1", "LoadName=1.5   LoadSF=1"), vec!["CASE - STATIC 1 - LOAD ASSIGNMENTS", "1.5"]),
        (replaced(&good, "LoadName=2   LoadSF=1", "LoadName=2   LoadSF=lots"), vec!["LoadSF", "lots"]),
    ];
    for (text, needles) in &cases {
        assert_import_error(text, needles);
    }
}

#[test]
fn a_property_defined_twice_and_a_second_overwrite_are_import_errors() {
    let good = good_text();
    let row = "   Name=MH   DOF=M3";
    let start = good.find(row).unwrap();
    let end = start + good[start..].find('\n').unwrap() + 1;
    let end = end + good[end..].find('\n').unwrap() + 1; // the continuation line
    let property = good[start..end].to_string();
    assert_import_error(&good.replacen(&property, &format!("{property}{property}"), 1), &["RUST HINGE PROPERTIES", "'MH'", "twice"]);
    let overwrite = "   Frame=F   AutoSubdivide=Yes   RelLength=0.02   DoNotDropLoad=No\n";
    assert_import_error(
        &replaced(&good, overwrite, &format!("{overwrite}{overwrite}")),
        &["RUST FRAME HINGE OVERWRITES", "two hinge overwrites"],
    );
}

#[test]
fn missing_required_columns_are_import_errors() {
    let good = good_text();
    for (from, to, needle) in [
        ("   Name=MH   DOF=M3", "   Name=MH", "needs DOF"),
        ("   Symmetric=Yes", "", "needs Symmetric"),
        ("   RelDist=0.5", "", "needs RelDist"),
        ("   HingeNum=1", "", "needs HingeNum"),
        ("   AutoSubdivide=Yes", "", "needs AutoSubdivide"),
        ("   Deform=0.4", "", "needs Deform"),
    ] {
        assert_import_error(&replaced(&good, from, to), &[needle]);
    }
}

#[test]
fn a_nonsymmetric_property_must_give_its_negative_points() {
    let good = good_text();
    let text = replaced(&good, "Symmetric=Yes", "Symmetric=No");
    // The writer gave all eight points, so this reads.
    parse_s2k(&text).unwrap();
    let start = text.find("   Name=MH   Point=\"-C\"").unwrap();
    let end = start + text[start..].find('\n').unwrap() + 1;
    let mut cut = text.clone();
    cut.replace_range(start..end, "");
    assert_import_error(&cut, &["RUST HINGE PROPERTIES - CURVE", "no point -C"]);
}

#[test]
fn the_hinge_tables_are_supported_so_preserved_edits_see_them() {
    // A model read from non-canonical text keeps its source; editing a hinge
    // is written back, not refused as an unsupported table.
    let mut text = write_s2k(&rich_model());
    text = text.replace("Name=Moment   DOF=M3", "Name=Moment    DOF=M3");
    let report = parse_s2k_with_report(&text).unwrap();
    assert!(!report
        .warnings
        .iter()
        .any(|w| w.contains("Unsupported S2K table") && w.contains("HINGE")));
    let mut model = report.model;
    model.frames["G"].hinges[0].rel_dist = 0.75;
    let edited = parse_s2k(&write_s2k(&model)).unwrap();
    assert_eq!(edited.frames["G"].hinges[0].rel_dist, 0.75);
    assert_eq!(edited.hinge_props, model.hinge_props);
}
