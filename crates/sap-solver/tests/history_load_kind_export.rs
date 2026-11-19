//! D81: the kind of a history load through native export (reopened file, same
//! response) and through the dependency check on imported data that is not
//! modeled.
//!
//! Model (kN, m, s): joint J, spring k = 100 kN/m in U1, mass m = 1, so
//! w = sqrt(k/m) = 10 rad/s; one mode, no damping, a constant function of 1
//! scaled by SF = 2. Modal superposition of one mode is exact for this single
//! degree of freedom. A ground acceleration a = SF * 1 = 2 m/s2 gives the
//! relative displacement u(t) = -(a/w^2)(1 - cos wt) = -0.02 (1 - cos 10 t):
//! u(0.1) = -0.009193953882637, u(0.5) = -0.014326756290 m. The tolerance is
//! 1e-9 m, far above integration rounding (observed ~1e-17).

use sap_format::extract_sdb_to_model;
use sap_format::s2k::parse_s2k;
use sap_format::sdb::native::pack_model_to_native_sdb;
use sap_model::{HistoryLoadKind, SapModel};
use sap_solver::analyze_model;

const TOL: f64 = 1e-9;

fn closed_form(t: f64) -> f64 {
    -0.02 * (1.0 - (10.0 * t).cos())
}

/// `patterns`: load pattern names; `kind`: text of the `LoadType` column or "";
/// `unmodeled`: an extra `RUST UNMODELED SOURCE DATA` row target, or "".
fn s2k(patterns: &[&str], kind: &str, load: &str, unmodeled: &str) -> String {
    let pats: String = patterns
        .iter()
        .map(|p| format!(" LoadPat={p} DesignType=Other SelfWtMult=0\n"))
        .collect();
    let unmodeled = if unmodeled.is_empty() {
        String::new()
    } else {
        format!("TABLE: \"RUST UNMODELED SOURCE DATA\"\n Scope=Pattern Target={unmodeled} Description=\"unsupported load assigned only to load pattern {unmodeled}\"\n")
    };
    format!(
        r#"TABLE: "PROGRAM CONTROL"
 CurrUnits="kN, m, C"
TABLE: "JOINT COORDINATES"
 Joint=J XorR=10 Y=0 Z=0
TABLE: "JOINT RESTRAINT ASSIGNMENTS"
 Joint=J U2=Yes U3=Yes R1=Yes R2=Yes R3=Yes
TABLE: "JOINT SPRING ASSIGNMENTS 1 - UNCOUPLED"
 Joint=J U1=100
TABLE: "JOINT ADDED MASS ASSIGNMENTS"
 Joint=J Mass1=1
TABLE: "LOAD PATTERN DEFINITIONS"
{pats}TABLE: "FUNCTION - TIME HISTORY - USER"
 Name=CONST Time=0 Value=1
 Name=CONST Time=1 Value=1
TABLE: "LOAD CASE DEFINITIONS"
 Case=MODAL Type=LinModal MaxNumModes=1 MinNumModes=1
 Case=HIST Type=LinHist RunCase=Yes RustModalCase=MODAL NumSteps=5 StepSize=0.1 ConstDamp=0
TABLE: "CASE - TIME HISTORY 2 - LOAD ASSIGNMENTS"
 Case=HIST {kind} Function=CONST LoadName={load} SF=2
{unmodeled}END TABLE DATA
"#
    )
}

/// Joint J U1 displacement at each output step of HIST.
fn response(model: &SapModel) -> Vec<(f64, f64)> {
    let h = &model.case_results["HIST"];
    h.time_steps
        .iter()
        .enumerate()
        .map(|(k, t)| (*t, h.time_history_disp["J"][k][0]))
        .collect()
}

fn analyzed(text: &str) -> Result<SapModel, String> {
    let mut m = parse_s2k(text).map_err(|e| e.to_string())?;
    analyze_model(&mut m).map_err(|e| e.to_string())?;
    Ok(m)
}

fn assert_closed_form(model: &SapModel, what: &str) {
    let r = response(model);
    assert_eq!(r.len(), 6, "{what}");
    for (t, u) in r {
        assert!(
            (u - closed_form(t)).abs() < TOL,
            "{what}: t={t} u={u} exact={}",
            closed_form(t)
        );
    }
}

/// Exports the S2K model natively, reopens it, checks the stored entry and
/// returns the analyzed reopened model.
fn reopened(text: &str, kind: HistoryLoadKind) -> SapModel {
    let source = parse_s2k(text).unwrap();
    let sdb = pack_model_to_native_sdb(&source).unwrap_or_else(|e| panic!("{e}"));
    let mut back = extract_sdb_to_model(&sdb).unwrap();
    let lc = &back.load_cases["HIST"];
    assert_eq!(lc.history_loads.len(), 1);
    assert_eq!(lc.history_kind(0), kind);
    assert_eq!(lc.history_loads[0].1, "U1");
    analyze_model(&mut back).unwrap_or_else(|e| panic!("{e}"));
    back
}

#[test]
fn legacy_u1_explicit_accel_u1_and_x_export_and_reopen_with_the_same_response() {
    let accel = HistoryLoadKind::Acceleration;
    // Pattern X exists in all three: only the explicit forms may use X.
    for (kind, load, what) in [
        ("", "U1", "legacy U1"),
        ("LoadType=Accel", "U1", "Accel U1"),
        ("LoadType=Accel", "X", "Accel X"),
        ("LoadType=Accel", "UX", "Accel UX"),
    ] {
        let text = s2k(&["X"], kind, load, "");
        let direct = analyzed(&text).unwrap();
        assert_closed_form(&direct, &format!("S2K {what}"));
        let back = reopened(&text, accel);
        assert_closed_form(&back, &format!("reopened {what}"));
        for ((_, a), (_, b)) in response(&direct).into_iter().zip(response(&back)) {
            assert!((a - b).abs() < TOL, "{what}: {a} vs {b}");
        }
    }
}

#[test]
fn ambiguous_legacy_x_is_refused_at_export_and_at_analysis_with_the_same_words() {
    let text = s2k(&["X"], "", "X", "");
    let export = pack_model_to_native_sdb(&parse_s2k(&text).unwrap())
        .unwrap_err()
        .to_string();
    let analysis = analyzed(&text).unwrap_err();
    for needle in [
        "case 'HIST'",
        "'X'",
        "load pattern 'X'",
        "ground acceleration in direction U1",
    ] {
        assert!(export.contains(needle), "{needle}: {export}");
        assert!(analysis.contains(needle), "{needle}: {analysis}");
    }
}

#[test]
fn an_explicit_acceleration_u1_gives_one_response_whether_or_not_an_unmodeled_pattern_is_called_u1()
{
    // Matched name: pattern U1 is unsupported and the entry is Accel U1.
    // Matched control: pattern U1 exists, nothing unmodeled.
    // Different name: pattern UNUSED is the unsupported one.
    let matched_name = analyzed(&s2k(&["U1"], "LoadType=Accel", "U1", "U1")).unwrap();
    let matched_control = analyzed(&s2k(&["U1"], "LoadType=Accel", "U1", "")).unwrap();
    let different_name = analyzed(&s2k(&["UNUSED"], "LoadType=Accel", "U1", "UNUSED")).unwrap();
    for (m, what) in [
        (&matched_name, "unused-pattern-matched-name"),
        (&matched_control, "unused-pattern-matched-control"),
        (&different_name, "unused-pattern-different-name"),
    ] {
        assert_closed_form(m, what);
    }
    for ((_, a), (_, b)) in response(&matched_name)
        .into_iter()
        .zip(response(&different_name))
    {
        assert!((a - b).abs() < 1e-15, "{a} vs {b}");
    }
}

#[test]
fn a_history_that_uses_the_unsupported_pattern_is_still_refused_naming_it() {
    // Explicit pattern U1 (the entry applies the pattern called U1).
    let err = analyzed(&s2k(&["U1"], "LoadType=\"Load pattern\"", "U1", "U1")).unwrap_err();
    assert!(
        err.contains("case 'HIST' depends on imported source data")
            && err.contains("load pattern U1"),
        "{err}"
    );
    // Legacy U1 beside a pattern U1: it may be the pattern, so the dependency
    // stands (the ambiguity is refused as well when the entry is resolved).
    let err = analyzed(&s2k(&["U1"], "", "U1", "U1")).unwrap_err();
    assert!(err.contains("depends on imported source data"), "{err}");
    // An ordinary pattern name.
    let err = analyzed(&s2k(&["WIND"], "", "WIND", "WIND")).unwrap_err();
    assert!(err.contains("load pattern WIND"), "{err}");
    // A zero scale does not use the pattern (existing rule, kept).
    let text = s2k(&["WIND"], "", "WIND", "WIND").replace("SF=2", "SF=0");
    assert!(analyzed(&text).is_ok());
}

#[test]
fn a_legacy_acceleration_with_no_pattern_of_its_name_does_not_depend_on_the_unmodeled_u1() {
    // Legacy U1, pattern U1 absent from the model but named by the unmodeled
    // entry (the entry survives a deleted pattern): the entry is an acceleration.
    let m = analyzed(&s2k(&["WIND"], "", "U1", "U1")).unwrap();
    assert_closed_form(&m, "legacy U1, no pattern U1");
}
