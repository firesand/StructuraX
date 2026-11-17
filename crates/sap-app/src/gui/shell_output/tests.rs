//! Tests of the shell output quantities. Every expected value is derived in
//! a comment from the formulas of the CSI Analysis Reference (Eqns. 1 and 2,
//! pp. 210-214), not from the code under test.
use super::*;
use sap_model::{
    AreaSection, CaseResults, LoadCase, LoadCombo, LoadPattern, LoadPatternType, Material,
    PointLoadForce, ShellType,
};
use sap_solver::analyze_model;
use sap_units::Units;

use Face::{AbsMaximum, Bottom, Maximum, Minimum, Top};
use ShellQuantity::{Resultant as Res, Stress as Str};
use ShellResultant as R;
use ShellStress as S;

fn close(what: &str, got: f64, want: f64, tol: f64) {
    assert!(
        (got - want).abs() <= tol * want.abs().max(1.0),
        "{what}: got {got:e}, want {want:e}"
    );
}

// ---------------------------------------------------------------------------
// Synthetic results: no analysis, the stored numbers are chosen by hand.

const TM: f64 = 0.2;
const TB: f64 = 0.1;

fn section(name: &str, shell_type: ShellType, prop: AreaPropType) -> AreaSection {
    AreaSection {
        name: name.into(),
        prop_type: prop,
        shell_type,
        thickness_membrane: TM,
        thickness_bending: TB,
        ..AreaSection::default()
    }
}

/// Areas `A` (shell), `AP` (plate), `AM` (membrane), `APL` (plane), `AAS`
/// (asolid), each with a section of its own, all carrying the stored result
/// `r` in the case `S`; the case `MOHR` has `mohr` in area `A`.
fn synthetic(r: &AreaResult, mohr: &AreaResult) -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    m.load_cases.clear();
    for (name, ty) in [
        ("S", LoadCaseType::LinearStatic),
        ("MOHR", LoadCaseType::LinearStatic),
        ("RS", LoadCaseType::ResponseSpectrum),
        ("MD", LoadCaseType::Modal),
        ("BK", LoadCaseType::Buckling),
    ] {
        let mut c = LoadCase::linear_static(name, "D", 1.0);
        c.case_type = ty;
        m.load_cases.insert(name.into(), c);
    }
    let secs = [
        ("A", "SH", ShellType::ShellThin, AreaPropType::Shell),
        ("AP", "PL", ShellType::PlateThin, AreaPropType::Shell),
        ("AM", "ME", ShellType::Membrane, AreaPropType::Shell),
        ("APL", "PS", ShellType::ShellThin, AreaPropType::Plane),
        ("AAS", "AS", ShellType::ShellThin, AreaPropType::ASolid),
    ];
    for (area, sec, st, prop) in secs {
        m.area_sections.insert(sec.into(), section(sec, st, prop));
        m.add_area(area, vec![], sec);
    }
    let mut s = CaseResults::default();
    for (area, _, _, _) in secs {
        s.area_results.insert(area.into(), r.clone());
    }
    m.case_results.insert("S".into(), s);
    let mut mo = CaseResults::default();
    mo.area_results.insert("A".into(), mohr.clone());
    m.case_results.insert("MOHR".into(), mo);
    // Combinations of every type over one item, with the results of S copied.
    let combos = [
        ("LIN", ComboType::LinearAdd, "S"),
        ("ENV", ComboType::Envelope, "S"),
        ("SRS", ComboType::Srss, "S"),
        ("ABS", ComboType::AbsAdd, "S"),
        ("RNG", ComboType::RangeAdd, "S"),
        ("LIN_ENV", ComboType::LinearAdd, "ENV"),
        ("LIN_LIN", ComboType::LinearAdd, "LIN"),
        ("LIN_RS", ComboType::LinearAdd, "RS"),
        ("LIN_NONE", ComboType::LinearAdd, "NOSUCH"),
    ];
    for (name, ty, item) in combos {
        m.load_combos.insert(
            name.into(),
            LoadCombo {
                name: name.into(),
                combo_type: ty,
                cases: vec![(item.into(), 1.0)],
            },
        );
        let src = m.case_results["S"].clone();
        m.case_results.insert(name.into(), src);
    }
    m
}

/// Stored result of the stress tests, tm = 0.2 and tb = 0.1 (kN, m):
/// F11 = 10, F12 = 0.4, M11 = 0.3, V13 = 0.5, V23 = 1.2, the rest zero.
///   top    s11 = 10/0.2 - 6*0.3/0.1^2 = 50 - 180 = -130,  s12 = 0.4/0.2 = 2
///   bottom s11 = 50 + 180 = 230,                           s12 = 2
fn stress_result() -> AreaResult {
    AreaResult {
        f11: 10.0,
        f12: 0.4,
        m11: 0.3,
        v13: 0.5,
        v23: 1.2,
        s11_top: -130.0,
        s22_top: 0.0,
        s12_top: 2.0,
        // sqrt(a^2 - ab + b^2 + 3c^2) = sqrt(130^2 + 3*4) = sqrt(16912)
        svm_top: 16912f64.sqrt(),
        ..AreaResult::default()
    }
}

/// Mohr example of the plan: (a, b, c) = (3, -1, 2) for F and for M.
///   centre 1, radius hypot(2, 2) = sqrt(8) = 2.828427124746190
///   max 3.828427124746190, min -1.828427124746190
///   von Mises sqrt(9 + 3 + 1 + 3*4) = sqrt(25) = 5
fn mohr_result() -> AreaResult {
    AreaResult {
        f11: 3.0,
        f22: -1.0,
        f12: 2.0,
        m11: 3.0,
        m22: -1.0,
        m12: 2.0,
        v13: 3.0,
        v23: 4.0,
        ..AreaResult::default()
    }
}

fn syn() -> SapModel {
    synthetic(&stress_result(), &mohr_result())
}

#[test]
fn principal_values_by_mohr() {
    let (hi, lo) = principal(3.0, -1.0, 2.0);
    close("max", hi, 1.0 + 8f64.sqrt(), 1e-14);
    close("min", lo, 1.0 - 8f64.sqrt(), 1e-14);
    // Uniaxial and pure shear: (a, 0, 0) -> (a, 0) for a > 0; (0, 0, c) -> (c, -c).
    assert_eq!(principal(5.0, 0.0, 0.0), (5.0, 0.0));
    assert_eq!(principal(0.0, 0.0, 2.0), (2.0, -2.0));
    // Equal biaxial: no shear radius.
    assert_eq!(principal(-4.0, -4.0, 0.0), (-4.0, -4.0));
}

#[test]
fn resultants_principal_von_mises_and_vmax() {
    let m = syn();
    let v = |q| area_value(&m, "MOHR", "A", q).unwrap();
    close("FMAX", v(Res(R::FMax)), 1.0 + 8f64.sqrt(), 1e-14);
    close("FMIN", v(Res(R::FMin)), 1.0 - 8f64.sqrt(), 1e-14);
    close("MMAX", v(Res(R::MMax)), 1.0 + 8f64.sqrt(), 1e-14);
    close("MMIN", v(Res(R::MMin)), 1.0 - 8f64.sqrt(), 1e-14);
    close("FVM", v(Res(R::Fvm)), 5.0, 1e-14);
    // VMAX = hypot(3, 4) = 5
    close("VMAX", v(Res(R::VMax)), 5.0, 1e-14);
    // Stored resultants are returned bit for bit.
    for (q, want) in [
        (R::F11, 3.0),
        (R::F22, -1.0),
        (R::F12, 2.0),
        (R::M11, 3.0),
        (R::M22, -1.0),
        (R::M12, 2.0),
        (R::V13, 3.0),
        (R::V23, 4.0),
    ] {
        assert_eq!(v(Res(q)), want, "{q:?}");
    }
}

#[test]
fn legacy_five_return_the_stored_fields() {
    let m = syn();
    let r = stress_result();
    let want = [r.f11, r.f22, r.m11, r.m22, r.svm_top];
    for (q, w) in ShellQuantity::LEGACY.into_iter().zip(want) {
        assert_eq!(area_value(&m, "S", "A", q).unwrap().to_bits(), w.to_bits());
        // and for a linear combination of the case
        assert_eq!(area_value(&m, "LIN", "A", q).unwrap().to_bits(), w.to_bits());
    }
}

#[test]
fn stresses_use_membrane_and_bending_thicknesses_separately() {
    let m = syn();
    let v = |q| area_value(&m, "S", "A", q).unwrap();
    // Top: stored. Bottom: F/tm + 6M/tb^2 = 50 + 180 = 230 (a wrong thickness
    // rule, tm for both, would give 50 + 6*0.3/0.04 = 95).
    close("S11 top", v(Str(S::S11, Top)), -130.0, 1e-14);
    close("S11 bottom", v(Str(S::S11, Bottom)), 230.0, 1e-14);
    close("S22 bottom", v(Str(S::S22, Bottom)), 0.0, 1e-14);
    close("S12 bottom", v(Str(S::S12, Bottom)), 2.0, 1e-14);
    // von Mises: bottom sqrt(230^2 + 3*4) = sqrt(52912); top sqrt(16912).
    close("SVM bottom", v(Str(S::Svm, Bottom)), 52912f64.sqrt(), 1e-14);
    close("SVM top", v(Str(S::Svm, Top)), 16912f64.sqrt(), 1e-14);
    // Principal bottom: centre 115, radius hypot(115, 2) = sqrt(13229). SMIN is a
    // difference of nearly equal numbers (0.0174), so its tolerance is 1e-12 absolute,
    // about 4e-15 of the stress scale 230.
    close("SMAX bottom", v(Str(S::SMax, Bottom)), 115.0 + 13229f64.sqrt(), 1e-12);
    close("SMIN bottom", v(Str(S::SMin, Bottom)), 115.0 - 13229f64.sqrt(), 1e-12);
    // Principal top: centre -65, radius hypot(65, 2) = sqrt(4229).
    close("SMAX top", v(Str(S::SMax, Top)), -65.0 + 4229f64.sqrt(), 1e-12);
    close("SMIN top", v(Str(S::SMin, Top)), -65.0 - 4229f64.sqrt(), 1e-12);
    // Shear stresses V/tb: 0.5/0.1 = 5, 1.2/0.1 = 12, SMAXV = hypot(5, 12) = 13,
    // the same on every face.
    for face in [Top, Bottom, Maximum, Minimum, AbsMaximum] {
        close("S13", v(Str(S::S13, face)), 5.0, 1e-14);
        close("S23", v(Str(S::S23, face)), 12.0, 1e-14);
        close("SMAXV", v(Str(S::SMaxV, face)), 13.0, 1e-14);
    }
    // Face extremes of S11 (-130 top, 230 bottom).
    close("S11 max", v(Str(S::S11, Maximum)), 230.0, 1e-14);
    close("S11 min", v(Str(S::S11, Minimum)), -130.0, 1e-14);
    close("S11 abs", v(Str(S::S11, AbsMaximum)), 230.0, 1e-14);
    // Extremes of derived values: SVM top sqrt(16912), bottom sqrt(52912).
    close("SVM max", v(Str(S::Svm, Maximum)), 52912f64.sqrt(), 1e-14);
    close("SVM min", v(Str(S::Svm, Minimum)), 16912f64.sqrt(), 1e-14);
}

#[test]
fn abs_maximum_keeps_the_sign_and_a_tie_goes_to_the_top() {
    let mut r = stress_result();
    // F11 = -10, M11 = -0.3: top = -50 + 180 = 130, bottom = -50 - 180 = -230.
    r.f11 = -10.0;
    r.m11 = -0.3;
    r.s11_top = 130.0;
    let m = synthetic(&r, &mohr_result());
    let v = |q| area_value(&m, "S", "A", q).unwrap();
    close("abs", v(Str(S::S11, AbsMaximum)), -230.0, 1e-14);
    close("max", v(Str(S::S11, Maximum)), 130.0, 1e-14);
    close("min", v(Str(S::S11, Minimum)), -230.0, 1e-14);
    // F11 = 0, M11 = 0.3: top = -180, bottom = 180, equal magnitudes -> the top value.
    r.f11 = 0.0;
    r.m11 = 0.3;
    r.s11_top = -180.0;
    let m = synthetic(&r, &mohr_result());
    let tie = area_value(&m, "S", "A", Str(S::S11, AbsMaximum)).unwrap();
    close("tie", tie, -180.0, 1e-14);
}

#[test]
fn multi_valued_results_refuse_derived_quantities_only() {
    let m = syn();
    let derived = [
        Res(R::FMax),
        Res(R::FMin),
        Res(R::Fvm),
        Res(R::MMax),
        Res(R::MMin),
        Res(R::VMax),
        Str(S::SMax, Top),
        Str(S::SMin, Top),
        Str(S::Svm, Top),
        Str(S::SMaxV, Top),
        Str(S::S11, Bottom),
        Str(S::S22, Maximum),
        Str(S::S12, Minimum),
        Str(S::S11, AbsMaximum),
    ];
    let allowed = [
        Res(R::F11),
        Res(R::F22),
        Res(R::F12),
        Res(R::M11),
        Res(R::M22),
        Res(R::M12),
        Res(R::V13),
        Res(R::V23),
        Str(S::S11, Top),
        Str(S::S22, Top),
        Str(S::S12, Top),
        Str(S::S13, Top),
        Str(S::S23, Bottom),
    ];
    for name in ["ENV", "SRS", "ABS", "RNG", "LIN_ENV", "LIN_RS", "LIN_NONE", "RS"] {
        assert!(!is_single_valued(&m, name), "{name}");
    }
    // Results exist for ENV, SRS, ABS, RNG and LIN_ENV (copies of S).
    for name in ["ENV", "SRS", "ABS", "RNG", "LIN_ENV"] {
        for q in derived {
            assert!(q.is_derived(), "{q:?}");
            let e = area_value(&m, name, "A", q).unwrap_err();
            assert!(e.contains(name) && e.contains("p. 214"), "{name} {q:?}: {e}");
        }
        for q in allowed {
            assert!(!q.is_derived(), "{q:?}");
            area_value(&m, name, "A", q).unwrap_or_else(|e| panic!("{name} {q:?}: {e}"));
        }
    }
    // Single-valued: cases, a linear combination, a nested one, a modal case.
    for name in ["S", "LIN", "LIN_LIN", "MD", "BK"] {
        assert!(is_single_valued(&m, name), "{name}");
    }
    for name in ["S", "LIN", "LIN_LIN"] {
        for q in derived {
            // LIN_LIN has no stored result in this model: use the ones that exist.
            if let Some(cr) = m.case_results.get(name) {
                assert!(cr.area_results.contains_key("A"));
                area_value(&m, name, "A", q).unwrap_or_else(|e| panic!("{name} {q:?}: {e}"));
            }
        }
    }
}

#[test]
fn range_valued_case_types_are_not_single_valued() {
    let mut m = syn();
    for ty in [
        LoadCaseType::MovingLoad,
        LoadCaseType::SteadyState,
        LoadCaseType::PowerSpectralDensity,
    ] {
        m.load_cases.get_mut("S").unwrap().case_type = ty;
        assert!(!is_single_valued(&m, "S"), "{ty:?}");
        assert!(!is_single_valued(&m, "LIN"), "{ty:?}");
    }
    m.load_cases.get_mut("S").unwrap().case_type = LoadCaseType::NonlinearStatic;
    assert!(is_single_valued(&m, "S") && is_single_valued(&m, "LIN"));
}

#[test]
fn plate_face_stress_of_a_multi_valued_result_is_refused() {
    // Plate top-face stress is -6M/tb^2. An SRSS of M is positive whatever the
    // sign of the contributing moments, so -6M/tb^2 of the combined M would be
    // a compressive -180 where the combination's own value is +180. Refused.
    let m = syn();
    for name in ["SRS", "ABS", "ENV", "RNG"] {
        for q in [Str(S::S11, Top), Str(S::S22, Top), Str(S::S12, Top)] {
            let e = area_value(&m, name, "AP", q).unwrap_err();
            assert!(e.contains("'AP'") && e.contains(name) && e.contains("plate"), "{e}");
        }
        // The moment itself and the shear stress follow the combination.
        close("plate M11", area_value(&m, name, "AP", Res(R::M11)).unwrap(), 0.3, 1e-14);
        close("plate S13", area_value(&m, name, "AP", Str(S::S13, Top)).unwrap(), 5.0, 1e-14);
        // A membrane section's F/tm keeps its sign through the combination.
        close("membrane S11", area_value(&m, name, "AM", Str(S::S11, Top)).unwrap(), 50.0, 1e-14);
    }
    // Single-valued results are not affected.
    close("plate S11 top", area_value(&m, "LIN", "AP", Str(S::S11, Top)).unwrap(), -180.0, 1e-14);
}

#[test]
fn a_combination_cycle_is_not_single_valued() {
    let mut m = syn();
    for (a, b) in [("C1", "C2"), ("C2", "C1")] {
        m.load_combos.insert(
            a.into(),
            LoadCombo {
                name: a.into(),
                combo_type: ComboType::LinearAdd,
                cases: vec![(b.into(), 1.0)],
            },
        );
    }
    assert!(!is_single_valued(&m, "C1"));
}

#[test]
fn plate_membrane_plane_and_asolid_sections() {
    // Plate: no membrane. The stored F is not a result (the solver stabilises the
    // in-plane degrees of freedom), so F quantities are refused and stresses come
    // from the moment alone: top = -6*0.3/0.1^2 = -180, bottom = 180.
    let m = syn();
    let v = |a, q| area_value(&m, "S", a, q);
    for q in [
        Res(R::F11),
        Res(R::F22),
        Res(R::F12),
        Res(R::FMax),
        Res(R::FMin),
        Res(R::Fvm),
    ] {
        let e = v("AP", q).unwrap_err();
        assert!(e.contains("'AP'") && e.contains("'PL'"), "{e}");
    }
    close("plate M11", v("AP", Res(R::M11)).unwrap(), 0.3, 1e-14);
    close("plate S11 top", v("AP", Str(S::S11, Top)).unwrap(), -180.0, 1e-14);
    close("plate S11 bottom", v("AP", Str(S::S11, Bottom)).unwrap(), 180.0, 1e-14);
    close("plate S13", v("AP", Str(S::S13, Top)).unwrap(), 5.0, 1e-14);
    // von Mises of (-180, 0, 0) is 180: the membrane part of the stored value is dropped.
    close("plate SVM top", v("AP", Str(S::Svm, Top)).unwrap(), 180.0, 1e-14);
    // Membrane: no moments or shears. Stress F/tm on both faces: 10/0.2 = 50, s12 = 2.
    for q in [
        Res(R::M11),
        Res(R::MMax),
        Res(R::V13),
        Res(R::VMax),
        Str(S::S13, Top),
        Str(S::SMaxV, Top),
    ] {
        let e = v("AM", q).unwrap_err();
        assert!(e.contains("'AM'") && e.contains("'ME'"), "{e}");
    }
    close("membrane F11", v("AM", Res(R::F11)).unwrap(), 10.0, 1e-14);
    close("membrane S11 top", v("AM", Str(S::S11, Top)).unwrap(), 50.0, 1e-14);
    close("membrane S11 bottom", v("AM", Str(S::S11, Bottom)).unwrap(), 50.0, 1e-14);
    close(
        "membrane SVM",
        v("AM", Str(S::Svm, Top)).unwrap(),
        (2500.0f64 + 12.0).sqrt(),
        1e-14,
    );
    // Plane stress: a membrane of the section's thickness. F11 = 10, stress
    // 10/0.2 = 50 on both faces; no moments or shears.
    close("plane-stress F11", v("APL", Res(R::F11)).unwrap(), 10.0, 1e-14);
    close("plane-stress S11 top", v("APL", Str(S::S11, Top)).unwrap(), 50.0, 1e-14);
    close("plane-stress S11 bottom", v("APL", Str(S::S11, Bottom)).unwrap(), 50.0, 1e-14);
    for q in [Res(R::M11), Res(R::V13), Str(S::S13, Top)] {
        let e = v("APL", q).unwrap_err();
        assert!(e.contains("'APL'") && e.contains("'PS'"), "{e}");
    }
    // Plane strain and asolid: nothing (the plane-strain forces are recovered
    // with the plane-stress relation, and an asolid is never analysed).
    let mut strain = syn();
    strain.area_sections.get_mut("PS").unwrap().plane_type = sap_model::PlaneType::PlaneStrain;
    for (model, a, sec, kind) in [
        (&strain, "APL", "PS", "plane-strain"),
        (&m, "AAS", "AS", "axisymmetric-solid"),
    ] {
        for q in [
            Res(R::F11),
            Res(R::M11),
            Str(S::S11, Top),
            Str(S::Svm, Top),
            Str(S::S13, Top),
        ] {
            let e = area_value(model, "S", a, q).unwrap_err();
            assert!(
                e.contains(&format!("'{a}'")) && e.contains(&format!("'{sec}'")) && e.contains(kind),
                "{e}"
            );
        }
    }
}

#[test]
fn missing_objects_are_named() {
    let mut m = syn();
    let q = Res(R::F11);
    assert!(area_value(&m, "NOPE", "A", q).unwrap_err().contains("'NOPE'"));
    assert!(area_value(&m, "S", "ZZ", q).unwrap_err().contains("'ZZ'"));
    // A result for an area that the model does not have.
    let r = m.case_results["S"].area_results["A"].clone();
    m.case_results["S"].area_results.insert("GHOST".into(), r);
    assert!(area_value(&m, "S", "GHOST", q).unwrap_err().contains("'GHOST'"));
    // A missing section.
    m.areas["A"].section = "NOSEC".into();
    let e = area_value(&m, "S", "A", q).unwrap_err();
    assert!(e.contains("'A'") && e.contains("'NOSEC'"), "{e}");
}

#[test]
fn missing_results_message_names_cases_and_types() {
    let m = syn();
    let k = ResultKind::Area;
    assert_eq!(missing_results_message(&m, "S", k), None);
    assert_eq!(missing_results_message(&m, "LIN", k), None);
    let msg = missing_results_message(&m, "MD", k).unwrap();
    assert!(
        msg.contains("'MD'") && msg.contains("Modal") && msg.contains("carries no shell results"),
        "{msg}"
    );
    let msg = missing_results_message(&m, "BK", k).unwrap();
    assert!(msg.contains("Buckling") && msg.contains("carries no shell results"), "{msg}");
    let msg = missing_results_message(&m, "RS", k).unwrap();
    assert!(msg.contains("ResponseSpectrum"), "{msg}");
    // A combination over a modal case and, through a nested combination, a spectrum case.
    let mut m = syn();
    m.load_combos.insert(
        "MIX".into(),
        LoadCombo {
            name: "MIX".into(),
            combo_type: ComboType::LinearAdd,
            cases: vec![("S".into(), 1.0), ("LIN_RS".into(), 1.0), ("MD".into(), 1.0)],
        },
    );
    let msg = missing_results_message(&m, "MIX", k).unwrap();
    assert!(msg.contains("'MIX'") && msg.contains("'RS'") && msg.contains("'MD'"), "{msg}");
    assert!(!msg.contains("'S'"), "{msg}");
    // A static case that has not been run.
    let mut m = syn();
    m.case_results.shift_remove("S");
    let msg = missing_results_message(&m, "S", k).unwrap();
    assert!(msg.contains("has no results"), "{msg}");
    // Another kind: the shell results of S are no solid results.
    let msg = missing_results_message(&syn(), "S", ResultKind::Solid).unwrap();
    assert!(msg.contains("solid"), "{msg}");
}

#[test]
fn visible_face_follows_local_3() {
    assert_eq!(visible_face([0.0, 0.0, 1.0], [0.0, 0.0, 1.0]), Top);
    assert_eq!(visible_face([0.0, 0.0, 1.0], [0.0, 0.0, -1.0]), Bottom);
    assert_eq!(visible_face([0.0, 0.0, 1.0], [1.0, 0.0, 0.0]), Top); // edge on: tie
    assert_eq!(visible_face([0.0, 1.0, 0.0], [0.3, -0.5, 0.2]), Bottom);
    assert_eq!(visible_face([0.0, 1.0, 0.0], [-0.3, 0.5, -0.2]), Top);
}

#[test]
fn labels_are_distinct_and_the_legacy_ones_are_fixed() {
    let l: Vec<String> = ShellQuantity::LEGACY.iter().map(|q| q.label()).collect();
    assert_eq!(
        l,
        [
            "Membrane F11",
            "Membrane F22",
            "Plate Moment M11",
            "Plate Moment M22",
            "Von Mises Stress SVM"
        ]
    );
    let all_r = [
        R::F11,
        R::F22,
        R::F12,
        R::FMax,
        R::FMin,
        R::Fvm,
        R::M11,
        R::M22,
        R::M12,
        R::MMax,
        R::MMin,
        R::V13,
        R::V23,
        R::VMax,
    ];
    let all_s = [S::S11, S::S22, S::S12, S::SMax, S::SMin, S::Svm, S::S13, S::S23, S::SMaxV];
    let mut labels: Vec<String> = all_r.iter().map(|&r| Res(r).label()).collect();
    for s in all_s {
        let faces: &[Face] = if matches!(s, S::S13 | S::S23 | S::SMaxV) {
            &[Top]
        } else {
            &[Top, Bottom, Maximum, Minimum, AbsMaximum]
        };
        for &f in faces {
            labels.push(Str(s, f).label());
        }
    }
    let n = labels.len();
    labels.sort();
    labels.dedup();
    assert_eq!(labels.len(), n, "labels must be distinct");
    assert_eq!(Res(R::FMax).label(), "Membrane FMAX");
    assert_eq!(Res(R::VMax).label(), "Plate Shear VMAX");
    assert_eq!(Str(S::Svm, Bottom).label(), "Von Mises Stress SVM (Bottom)");
    assert_eq!(Str(S::S11, AbsMaximum).label(), "Stress S11 (Abs Max)");
    assert_eq!(Str(S::SMaxV, Top).label(), "Stress SMAXV");
}

#[test]
fn dimensions_follow_the_quantity() {
    use super::super::display_units::Dimension as D;
    assert_eq!(Res(R::F12).dimension(), D::ForcePerLength);
    assert_eq!(Res(R::V13).dimension(), D::ForcePerLength);
    assert_eq!(Res(R::VMax).dimension(), D::ForcePerLength);
    assert_eq!(Res(R::M12).dimension(), D::MomentPerLength);
    assert_eq!(Res(R::MMin).dimension(), D::MomentPerLength);
    assert_eq!(Str(S::S13, Top).dimension(), D::Stress);
}

// ---------------------------------------------------------------------------
// Solved models.

const L: f64 = 4.0;
const T: f64 = 0.2;
const E: f64 = 2.5e7;

/// Cantilever plate strip of the third wave (kN, m): 4 x 1 elements, root
/// fixed, nu = 0; pattern `D`: FX = 25 and MY = +5 on each tip joint (F11 = 50,
/// M11 = -10), pattern `L`: MY = -5 on each tip joint.
fn strip() -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.e = E;
    mat.u = 0.0;
    mat.g = E / 2.0;
    mat.unit_weight = 0.0;
    mat.unit_mass = 0.0;
    m.materials.insert("MAT".into(), mat);
    let mut s = AreaSection::default();
    s.name = "PL".into();
    s.material = "MAT".into();
    s.thickness_membrane = T;
    s.thickness_bending = T;
    m.area_sections.insert("PL".into(), s);
    m.load_cases.clear();
    m.load_patterns.clear();
    for name in ["D", "L"] {
        m.load_patterns.insert(
            name.into(),
            LoadPattern {
                name: name.into(),
                pat_type: LoadPatternType::Dead,
                self_wt_multiplier: 0.0,
            },
        );
        m.load_cases
            .insert(name.into(), LoadCase::linear_static(name, name, 1.0));
    }
    for i in 0..=4 {
        for (j, y) in [0.0, 1.0].iter().enumerate() {
            let name = format!("J{i}{j}");
            m.add_point(name.clone(), i as f64 * L / 4.0, *y, 0.0);
            if i == 0 {
                m.points[&name].restraint = [true; 6];
            }
            if i == 4 {
                for (pat, f) in [
                    ("D", [25.0, 0., 0., 0., 5.0, 0.]),
                    ("L", [0., 0., 0., 0., -5.0, 0.]),
                ] {
                    m.points[&name].loads_force.push(PointLoadForce {
                        load_pat: pat.into(),
                        csys: "GLOBAL".into(),
                        force: f,
                    });
                }
            }
        }
    }
    for i in 0..4 {
        m.add_area(
            format!("A{i}"),
            vec![
                format!("J{i}0"),
                format!("J{}0", i + 1),
                format!("J{}1", i + 1),
                format!("J{i}1"),
            ],
            "PL",
        );
    }
    for (name, ty, items) in [
        ("NEG", ComboType::LinearAdd, vec![("D".to_string(), -1.0)]),
        (
            "MIXLIN",
            ComboType::LinearAdd,
            vec![("D".to_string(), 0.5), ("L".to_string(), 1.0)],
        ),
        (
            "ENV",
            ComboType::Envelope,
            vec![("D".to_string(), 1.0), ("L".to_string(), 1.0)],
        ),
    ] {
        m.load_combos.insert(
            name.into(),
            LoadCombo {
                name: name.into(),
                combo_type: ty,
                cases: items,
            },
        );
    }
    m
}

#[test]
fn strip_stresses_on_both_faces() {
    // Hand derivation (kN, m, tm = tb = 0.2): F11 = 50, M11 = -10 (hogging).
    //   top    s11 = 50/0.2 - 6*(-10)/0.04 = 250 + 1500 = 1750
    //   bottom s11 = 250 - 1500 = -1250
    // s22 and s12 are 0 (nu = 0, no shear). Von Mises bottom = 1250, principal
    // bottom (-1250, 0, 0): max 0, min -1250. Tolerance 1e-9 of the 1750 scale.
    let mut m = strip();
    analyze_model(&mut m).unwrap();
    let tol = 1e-9 * 1750.0;
    for a in ["A0", "A1", "A2", "A3"] {
        let chk = |what: &str, q, want: f64| {
            let got = area_value(&m, "D", a, q).unwrap();
            assert!((got - want).abs() <= tol, "{a} {what}: got {got:e}, want {want:e}");
        };
        chk("S11 top", Str(S::S11, Top), 1750.0);
        chk("S11 bottom", Str(S::S11, Bottom), -1250.0);
        chk("SVM bottom", Str(S::Svm, Bottom), 1250.0);
        chk("SVM top", Str(S::Svm, Top), 1750.0);
        chk("SMAX bottom", Str(S::SMax, Bottom), 0.0);
        chk("SMIN bottom", Str(S::SMin, Bottom), -1250.0);
        chk("S11 max", Str(S::S11, Maximum), 1750.0);
        chk("S11 min", Str(S::S11, Minimum), -1250.0);
        chk("S11 abs", Str(S::S11, AbsMaximum), 1750.0);
        chk("F11", Res(R::F11), 50.0);
        chk("M11", Res(R::M11), -10.0);
        chk("FMAX", Res(R::FMax), 50.0);
        chk("MMIN", Res(R::MMin), -10.0);
        // No transverse force acts on the strip.
        chk("S13", Str(S::S13, Top), 0.0);
    }
}

#[test]
fn derived_top_face_equals_the_stored_top_face() {
    // For a case and for linear combinations the top stress of the quantities is the
    // stored one: s11 bit for bit, the von Mises value to 1e-15 relative.
    let mut m = strip();
    analyze_model(&mut m).unwrap();
    for res in ["D", "L", "NEG", "MIXLIN"] {
        assert!(is_single_valued(&m, res));
        for a in ["A0", "A3"] {
            let ar = &m.case_results[res].area_results[a];
            let v = |q| area_value(&m, res, a, q).unwrap();
            assert_eq!(v(Str(S::S11, Top)).to_bits(), ar.s11_top.to_bits());
            assert_eq!(v(Str(S::S22, Top)).to_bits(), ar.s22_top.to_bits());
            assert_eq!(v(Str(S::S12, Top)).to_bits(), ar.s12_top.to_bits());
            close("svm", v(Str(S::Svm, Top)), ar.svm_top, 1e-15);
            // The stored value is the solver's von Mises function of the stored components.
            close(
                "svm recomputed",
                ar.svm_top,
                sap_solver::area::plane_von_mises(ar.s11_top, ar.s22_top, ar.s12_top),
                1e-15,
            );
        }
    }
}

#[test]
fn negated_combination_keeps_the_sign_of_the_larger_face() {
    // NEG = -D: F11 = -50, M11 = +10. top = -250 - 1500 = -1750,
    // bottom = -250 + 1500 = 1250. Maximum 1250, Minimum -1750, AbsMaximum -1750.
    let mut m = strip();
    analyze_model(&mut m).unwrap();
    let tol = 1e-9 * 1750.0;
    for (what, q, want) in [
        ("top", Str(S::S11, Top), -1750.0),
        ("bottom", Str(S::S11, Bottom), 1250.0),
        ("max", Str(S::S11, Maximum), 1250.0),
        ("min", Str(S::S11, Minimum), -1750.0),
        ("abs", Str(S::S11, AbsMaximum), -1750.0),
    ] {
        let got = area_value(&m, "NEG", "A1", q).unwrap();
        assert!((got - want).abs() <= tol, "{what}: got {got:e}, want {want:e}");
    }
    // MIXLIN = 0.5 D + L: F11 = 25, M11 = -5 + 10 = 5:
    // top = 125 - 750 = -625, bottom = 125 + 750 = 875.
    let g = area_value(&m, "MIXLIN", "A1", Str(S::S11, Bottom)).unwrap();
    assert!((g - 875.0).abs() <= tol, "{g}");
    let g = area_value(&m, "MIXLIN", "A1", Str(S::S11, Top)).unwrap();
    assert!((g + 625.0).abs() <= tol, "{g}");
}

#[test]
fn envelope_of_solved_cases_refuses_the_legacy_von_mises_contour() {
    let mut m = strip();
    analyze_model(&mut m).unwrap();
    let svm = ShellQuantity::LEGACY[4];
    let e = area_value(&m, "ENV", "A0", svm).unwrap_err();
    assert!(e.contains("'ENV'") && e.contains("p. 214"), "{e}");
    // The stored fields of the envelope remain available.
    area_value(&m, "ENV", "A0", ShellQuantity::LEGACY[0]).unwrap();
    area_value(&m, "ENV", "A0", Str(S::S11, Top)).unwrap();
}

// ---------------------------------------------------------------------------
// Example 2-010 (opt-in, STRUCTURAX_VENDOR_FIXTURES): cylinder under internal pressure.

#[test]
fn vendor_example_2_010_hoop_stress_matches_the_closed_form() {
    let Some(dir) = std::env::var_os("STRUCTURAX_VENDOR_FIXTURES") else {
        return;
    };
    let dir = std::path::PathBuf::from(dir);
    // R = 60 in, t = 1 in, p = 1 k/in2: sigma = p R cos(pi/N) / t with N = 16 (mesh a)
    // and N = 48 (mesh b) elements around; published 58.85 and 59.87 k/in2.
    let pi = std::f64::consts::PI;
    let cases = [
        ("example_2_010a_thin.sdb", 60.0 * (pi / 16.0).cos(), 58.85),
        ("example_2_010a_thick.sdb", 60.0 * (pi / 16.0).cos(), 58.85),
        ("example_2_010b_thin.sdb", 60.0 * (pi / 48.0).cos(), 59.87),
        ("example_2_010b_thick.sdb", 60.0 * (pi / 48.0).cos(), 59.87),
    ];
    for (file, closed, printed) in cases {
        let path = dir.join(file);
        if !path.exists() {
            eprintln!("skipped: {} not found", path.display());
            continue;
        }
        let mut m = sap_format::load_sdb_file(&path).unwrap_or_else(|e| panic!("{file}: {e}"));
        analyze_model(&mut m).unwrap_or_else(|e| panic!("{file}: {e}"));
        let case = m
            .case_results
            .iter()
            .find(|(_, cr)| !cr.area_results.is_empty())
            .map(|(n, _)| n.clone())
            .unwrap_or_else(|| panic!("{file}: no case with area results"));
        let mut checked = 0;
        for area in m.case_results[&case].area_results.keys() {
            let top = area_value(&m, &case, area, Str(S::S11, Top)).unwrap();
            let bot = area_value(&m, &case, area, Str(S::S11, Bottom)).unwrap();
            assert!(
                (top - bot).abs() <= 1e-6 * top.abs().max(1.0),
                "{file} {area}: top {top}, bottom {bot}"
            );
            assert!(
                (top - printed).abs() <= 0.005,
                "{file} {area}: S11 {top} against the printed {printed}"
            );
            assert!(
                (top - closed).abs() <= 1e-6 * closed,
                "{file} {area}: S11 {top} against the closed form {closed}"
            );
            checked += 1;
        }
        assert!(checked > 0, "{file}");
    }
}

#[test]
fn multi_valued_result_shows_the_value_of_larger_magnitude_with_its_sign() {
    // ENV: maximum row F11 = +3, minimum row F11 = -5 (an envelope of +3 and -5):
    // the larger magnitude is the minimum, so the form shows -5. M11: maximum
    // +0.3, minimum -0.1: the maximum, +0.3. SRS has min = -max (tie): the
    // maximum row, +4.
    let mut m = syn();
    let (hi, lo) = (
        AreaResult {
            f11: 3.0,
            m11: 0.3,
            s11_top: -165.0,
            ..stress_result()
        },
        AreaResult {
            f11: -5.0,
            m11: -0.1,
            s11_top: 35.0,
            ..stress_result()
        },
    );
    let mut env = m.case_results["ENV"].clone();
    env.area_results.insert("A".into(), hi);
    let mut min = CaseResults::default();
    min.area_results.insert("A".into(), lo);
    env.combo_min = Some(Box::new(min));
    m.case_results.insert("ENV".into(), env);
    let v = |m: &SapModel, name: &str, q| area_value(m, name, "A", q).unwrap();
    close("F11", v(&m, "ENV", Res(R::F11)), -5.0, 1e-14);
    close("M11", v(&m, "ENV", Res(R::M11)), 0.3, 1e-14);
    // Top-face S11 = F/tm - 6M/tb^2 on each row (tm = 0.2, tb = 0.1):
    //   maximum row 3/0.2 - 6 * 0.3 / 0.01 = 15 - 180 = -165,
    //   minimum row -5/0.2 + 6 * 0.1 / 0.01 = -25 + 60 = 35;
    //   -165 has the larger magnitude.
    close("S11 top", v(&m, "ENV", Str(S::S11, Top)), -165.0, 1e-12);
    let mut srs = m.case_results["SRS"].clone();
    srs.area_results.insert(
        "A".into(),
        AreaResult {
            f11: 4.0,
            ..stress_result()
        },
    );
    let mut neg = CaseResults::default();
    neg.area_results.insert(
        "A".into(),
        AreaResult {
            f11: -4.0,
            ..stress_result()
        },
    );
    srs.combo_min = Some(Box::new(neg));
    m.case_results.insert("SRS".into(), srs);
    close("tie", v(&m, "SRS", Res(R::F11)), 4.0, 1e-14);
    // Derived values stay refused for a multi-valued result.
    for q in [Res(R::FMax), Str(S::Svm, Top), Str(S::S11, Bottom)] {
        assert!(area_value(&m, "ENV", "A", q).is_err(), "{q:?}");
    }
    // A single-valued result is unaffected.
    close("S F11", v(&m, "S", Res(R::F11)), 10.0, 1e-14);
}
