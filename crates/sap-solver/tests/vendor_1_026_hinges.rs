//! Examples 1-026 A and B of the vendor corpus, imported from the native .sdb
//! files with their frame hinges and nonlinear static parameters, and analysed.
//!
//! The expected hinge data is the in-code model of tests T1 and T2 of
//! `frame_hinges.rs`; the published values are the SAP2000 results printed in
//! the verification example (kip, in): Model A and B share points 1 and 2,
//! point 3 differs (Uz 1.7612, Ry -0.0567 for A; Uz 1.3701, Ry -0.0404 for B).
//! The printed points are found by load P = 60, 80 and 20. Tolerance: 5e-5
//! absolute, the printed precision.
//!
//! Exceptions to strict equality with the in-code model, by design of the
//! importer: the files store acceptance criteria 2, 4, 6 (the in-code model
//! has none) and `m_AfterLastPt` 0, which is imported as `beyond_e: None`
//! because its meaning is not established.
#![cfg(feature = "vendor-fixtures")]

use std::path::{Path, PathBuf};

use sap_format::{extract_sdb_to_model, inspect_native_sdb_file, load_sdb_file};
use sap_model::hinge::*;
use sap_model::{CaseResults, SapModel, UnmodeledScope};
use sap_solver::analyze_model;

fn vendor_dir() -> PathBuf {
    std::env::var_os("STRUCTURAX_VENDOR_FIXTURES")
        .map(PathBuf::from)
        .unwrap_or_else(|| Path::new(env!("CARGO_MANIFEST_DIR")).join("../../extracted/files"))
}

fn moment() -> HingeProp {
    let mut p = HingeProp::symmetric(
        "Moment",
        HingeDof::M3,
        [
            HingePoint::new(1440.0, 0.0),
            HingePoint::new(1920.0, 0.04),
            HingePoint::new(480.0, 0.04),
            HingePoint::new(480.0, 0.4),
        ],
    );
    // The files' own data beyond the in-code model.
    p.acceptance_positive = HingeAcceptance {
        io: 2.0,
        ls: 4.0,
        cp: 6.0,
    };
    p.acceptance_negative = HingeAcceptance {
        io: -2.0,
        ls: -4.0,
        cp: -6.0,
    };
    p.beyond_e = None;
    p
}

fn shear() -> HingeProp {
    let mut p = HingeProp::symmetric(
        "Shear",
        HingeDof::V2,
        [
            HingePoint::new(70.0, 0.0),
            HingePoint::new(80.0, 0.4),
            HingePoint::new(90.0, 0.8),
            HingePoint::new(100.0, 1.2),
        ],
    );
    p.acceptance_positive = HingeAcceptance {
        io: 2.0,
        ls: 4.0,
        cp: 6.0,
    };
    p.acceptance_negative = HingeAcceptance {
        io: -2.0,
        ls: -4.0,
        cp: -6.0,
    };
    p.beyond_e = None;
    p
}

fn load(file: &str) -> SapModel {
    load_sdb_file(vendor_dir().join(file)).expect(file)
}

fn close_abs(got: f64, want: f64, tol: f64, what: &str) {
    assert!((got - want).abs() <= tol, "{what}: {got} vs {want}");
}

/// The tip state (Uz, Ry) of the first saved step at or after `from` whose
/// load factor is `load`, with the step index.
fn first_at_load(case: &CaseResults, joint: &str, load: f64, from: usize) -> (usize, f64, f64) {
    let step = (from..case.static_steps.len())
        .find(|&s| (case.static_steps[s].load_factor - load).abs() < 1e-9)
        .unwrap_or_else(|| panic!("no saved step at P = {load} from step {from}"));
    let u = case.static_steps[step].joint_displacements[joint];
    (step, u[2], u[4])
}

fn check(file: &str, overwrite: Option<FrameHingeOverwrite>, point3: (f64, f64)) {
    let mut model = load(file);
    assert!(
        model.unmodeled_source.is_empty(),
        "{file}: {:?}",
        model.unmodeled_source
    );

    // Hinge data equals the in-code model (T1 for A, T2 for B).
    assert_eq!(
        model.hinge_props.len(),
        2,
        "{file}: only the defined properties"
    );
    assert_eq!(model.hinge_props["Moment"], moment(), "{file}");
    assert_eq!(model.hinge_props["Shear"], shear(), "{file}");
    assert_eq!(model.frames.len(), 1);
    let frame = &model.frames["1"];
    let hinge = |prop: &str| FrameHinge {
        prop: prop.into(),
        rel_dist: 0.0,
    };
    assert_eq!(
        frame.hinges,
        vec![hinge("Moment"), hinge("Shear")],
        "{file}"
    );
    assert_eq!(frame.hinge_overwrite, overwrite, "{file}");

    let case = &model.load_cases["NLSTAT1"];
    assert_eq!(
        case.nl_static,
        NonlinearStaticParams {
            results_saved: ResultsSaved::MultipleStates {
                min_saved: 6,
                max_saved: 10,
                positive_only: false,
            },
            max_total_steps: 200,
            max_null_steps: 50,
            hinge_unloading: HingeUnloading::UnloadEntireStructure,
            mode_loads: Vec::new(),
        },
        "{file}"
    );
    let control = case
        .displacement_control
        .as_ref()
        .expect("displacement control");
    assert_eq!(control.target, 2.0);
    let (joint, dof) = control.monitored.clone().expect("monitored joint");
    assert_eq!(dof, 2);

    analyze_model(&mut model).expect(file);
    let results = &model.case_results["NLSTAT1"];
    // Points 1 and 2 are common to both models; point 3 follows the model.
    let (s1, uz1, ry1) = first_at_load(results, &joint, 60.0, 0);
    close_abs(uz1, 0.0185, 5e-5, &format!("{file} Uz point 1"));
    close_abs(ry1, -0.0008, 5e-5, &format!("{file} Ry point 1"));
    let (s2, uz2, ry2) = first_at_load(results, &joint, 80.0, s1);
    close_abs(uz2, 1.3847, 5e-5, &format!("{file} Uz point 2"));
    close_abs(ry2, -0.0411, 5e-5, &format!("{file} Ry point 2"));
    let (_, uz3, ry3) = first_at_load(results, &joint, 20.0, s2);
    close_abs(uz3, point3.0, 5e-5, &format!("{file} Uz point 3"));
    close_abs(ry3, point3.1, 5e-5, &format!("{file} Ry point 3"));
}

#[test]
fn example_1_026_a_imports_its_hinges_and_reproduces_the_published_points() {
    check("example_1_026a.sdb", None, (1.7612, -0.0567));
}

#[test]
fn example_1_026_b_imports_its_overwrite_and_reproduces_the_published_snap_back() {
    let overwrite = Some(FrameHingeOverwrite {
        auto_subdivide: true,
        relative_length: 0.02,
        do_not_drop_load: false,
    });
    check("example_1_026b.sdb", overwrite, (1.3701, -0.0404));
}

/// A field the two files do not show is not established: the frame gets no
/// hinge, an unmodeled entry names it, and the analysis refuses.
#[test]
fn an_unestablished_hinge_value_leaves_an_unmodeled_entry_and_no_hinge() {
    let path = vendor_dir().join("example_1_026a.sdb");
    let nth = |document: &sap_format::NativeSdbDocument, class: &str, n: usize| {
        document
            .records()
            .iter()
            .enumerate()
            .filter(|(_, r)| r.class_id == class)
            .nth(n)
            .map(|(i, _)| i)
            .unwrap_or_else(|| panic!("no {class} record {n}"))
    };
    // (class, record number, field, patched value, expected reason): the
    // defined property's copy then differs from the object's own copy.
    for (class, n, field, value, reason) in [
        ("S_Hinge", 1, "m_drel", 0.5, "m_drel is 0.5"),
        (
            "S_HingeNLProp",
            2,
            "m_Yield",
            2.0,
            "differs from the defined property",
        ),
        (
            "S_HingeOverwrites",
            0,
            "m_AutoSubdivideRelLength",
            0.05,
            "m_AutoSubdivideRelLength is 0.05",
        ),
    ] {
        let mut document = inspect_native_sdb_file(&path).unwrap();
        let record = nth(&document, class, n);
        document.set_f64(record, field, value).unwrap();
        let mut model = extract_sdb_to_model(&document.to_sdb()).unwrap();
        assert!(model.frames["1"].hinges.is_empty(), "{field}");
        assert!(model.hinge_props.is_empty(), "{field}");
        assert!(model.frames["1"].hinge_overwrite.is_none(), "{field}");
        let entries: Vec<_> = model
            .unmodeled_source
            .iter()
            .filter(|e| e.scope == UnmodeledScope::NonlinearCases)
            .collect();
        assert_eq!(entries.len(), 1, "{field}: {:?}", model.unmodeled_source);
        assert!(
            entries[0].description.contains("frame '1'") && entries[0].description.contains(reason),
            "{field}: {}",
            entries[0].description
        );
        // Without a hinge the case keeps the default parameters.
        assert!(
            model.load_cases["NLSTAT1"].nl_static.is_default(),
            "{field}"
        );
        let error = analyze_model(&mut model).unwrap_err().to_string();
        assert!(error.contains("frame hinge assignment"), "{field}: {error}");
        assert!(model.case_results.is_empty());
    }
}

/// A load assignment of a nonlinear static case whose type is neither a load
/// pattern nor an acceleration is not dropped silently any more.
#[test]
fn a_nonlinear_static_load_of_an_unmodeled_type_leaves_an_unmodeled_entry() {
    let document = inspect_native_sdb_file(vendor_dir().join("example_1_026a.sdb")).unwrap();
    let la = document
        .records()
        .iter()
        .rfind(|r| r.class_id == "LAARRAY") // the case 1 record comes first, then NLSTAT1's
        .expect("load assignment record");
    let span = la
        .fields
        .iter()
        .find(|f| f.name.as_deref() == Some("m_ltype"))
        .expect("m_ltype");
    let mut sdb = document.to_sdb();
    sdb.decompressed_stream[span.offset..span.offset + 4].copy_from_slice(&4_i32.to_le_bytes());
    let model = extract_sdb_to_model(&sdb).unwrap();
    let entries: Vec<_> = model
        .unmodeled_source
        .iter()
        .filter(|e| e.scope == UnmodeledScope::Case("NLSTAT1".into()))
        .collect();
    assert_eq!(entries.len(), 1, "{:?}", model.unmodeled_source);
    assert!(
        entries[0].description.contains("type 4"),
        "{}",
        entries[0].description
    );
    // The hinges themselves are unaffected.
    assert_eq!(model.frames["1"].hinges.len(), 2);
}
