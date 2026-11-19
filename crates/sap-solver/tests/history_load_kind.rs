//! D77: whether a time-history assignment is a load pattern or a ground
//! acceleration is stored (S2K `LoadType`, native `m_ltype`) and carried to the
//! solver; a name never decides it once the kind is known.
//!
//! Model (kN, m, s): joint J has a spring of k = 100 kN/m in U1 and a mass
//! m = 1 (so w = sqrt(k/m) = 10 rad/s), no damping, one mode, a constant time
//! function of 1. Modal superposition of a single mode is exact for this one
//! degree of freedom; the tolerance of 1e-9 m is far above the rounding of the
//! integration (observed difference 1e-17 m).
//!
//! References (closed forms):
//! * a joint force F = 10 kN applied suddenly and held: u(t) = (F/k)(1 - cos wt),
//!   u(0.1 s) = 0.1 (1 - cos 1) = 0.045969769413186 m;
//! * a ground acceleration a = 1 m/s2 (function value 1, scale 1):
//!   u(t) = -(a/w^2)(1 - cos wt), u(0.1 s) = -0.01 (1 - cos 1)
//!   = -0.0045969769413186 m (relative motion).

use sap_format::s2k::{parse_s2k, write_s2k};
use sap_model::{HistoryLoadKind, SapModel};
use sap_solver::analyze_model;

// 1 - cos 1 = 0.45969769413186023 (cos 1 = 0.5403023058681398).
const PATTERN_RESULT: f64 = 0.045_969_769_413_186_02;
const ACCEL_RESULT: f64 = -0.004_596_976_941_318_602;
const TOL: f64 = 1e-9;

/// The pack's model with a chosen pattern name, history load name and the text
/// of the `LoadType` column (`""` leaves the column out: a legacy input).
fn s2k(pattern: &str, load_name: &str, load_type: &str) -> String {
    format!(
        r#"TABLE: "PROGRAM CONTROL"
 CurrUnits="kN, m, C"
TABLE: "JOINT COORDINATES"
 Joint=I XorR=0 Y=0 Z=0
 Joint=K XorR=6 Y=0 Z=0
 Joint=J XorR=10 Y=0 Z=0
TABLE: "JOINT RESTRAINT ASSIGNMENTS"
 Joint=I U1=Yes U2=Yes U3=Yes R1=Yes R2=Yes R3=Yes
 Joint=K U1=Yes U2=Yes U3=Yes R1=Yes R2=Yes R3=Yes
 Joint=J U2=Yes U3=Yes R1=Yes R2=Yes R3=Yes
TABLE: "JOINT SPRING ASSIGNMENTS 1 - UNCOUPLED"
 Joint=J U1=100
TABLE: "JOINT ADDED MASS ASSIGNMENTS"
 Joint=J Mass1=1
TABLE: "CONNECTIVITY - FRAME"
 Frame=F JointI=I JointJ=K
TABLE: "FRAME SECTION ASSIGNMENTS"
 Frame=F AnalSect=FSEC1
TABLE: "LOAD PATTERN DEFINITIONS"
 LoadPat={pattern} DesignType=Other SelfWtMult=0
TABLE: "JOINT LOADS - FORCE"
 Joint=J LoadPat={pattern} F1=10
TABLE: "FUNCTION - TIME HISTORY - USER"
 Name=CONST Time=0 Value=1
 Name=CONST Time=1 Value=1
TABLE: "LOAD CASE DEFINITIONS"
 Case={pattern} Type=LinStatic RunCase=Yes
 Case=MODAL Type=Modal MaxNumModes=1 MinNumModes=1
 Case=HIST Type=LinHist RunCase=Yes RustModalCase=MODAL NumSteps=5 StepSize=0.1 ConstDamp=0
TABLE: "CASE - TIME HISTORY 2 - LOAD ASSIGNMENTS"
 Case=HIST {load_type} Function=CONST LoadName={load_name} SF=1
END TABLE DATA
"#
    )
}

fn u_at_0_1(model: &SapModel) -> f64 {
    let hist = &model.case_results["HIST"];
    let k = hist
        .time_steps
        .iter()
        .position(|t| (t - 0.1).abs() < 1e-9)
        .expect("0.1 s is an output step");
    hist.time_history_disp["J"][k][0]
}

fn run(text: &str) -> Result<SapModel, String> {
    let mut model = parse_s2k(text).map_err(|e| e.to_string())?;
    analyze_model(&mut model).map_err(|e| e.to_string())?;
    Ok(model)
}

const PATTERN_TYPE: &str = r#"LoadType="Load pattern""#;
const ACCEL_TYPE: &str = "LoadType=Accel";

#[test]
fn an_explicit_pattern_gives_the_force_result_under_every_name() {
    for name in ["P", "X", "U1"] {
        let model = run(&s2k(name, name, PATTERN_TYPE)).unwrap_or_else(|e| panic!("{name}: {e}"));
        assert_eq!(model.load_cases["HIST"].history_kind(0), HistoryLoadKind::Pattern);
        let u = u_at_0_1(&model);
        assert!((u - PATTERN_RESULT).abs() < TOL, "{name}: {u} vs {PATTERN_RESULT}");
    }
}

#[test]
fn an_explicit_acceleration_still_gives_the_ground_motion_result() {
    // A pattern P exists and is not used; U1 is an acceleration.
    let model = run(&s2k("P", "U1", ACCEL_TYPE)).unwrap();
    assert_eq!(model.load_cases["HIST"].history_kind(0), HistoryLoadKind::Acceleration);
    let u = u_at_0_1(&model);
    assert!((u - ACCEL_RESULT).abs() < TOL, "{u} vs {ACCEL_RESULT}");
    // The spellings UX and X are the same direction.
    for name in ["UX", "X"] {
        let u = u_at_0_1(&run(&s2k("P", name, ACCEL_TYPE)).unwrap());
        assert!((u - ACCEL_RESULT).abs() < TOL, "{name}: {u}");
    }
}

#[test]
fn a_pattern_named_u1_and_an_acceleration_u1_are_told_apart_by_the_stored_kind() {
    // The model has a pattern U1 (10 kN), and the same name is used once as
    // the pattern and once as the acceleration.
    let as_pattern = u_at_0_1(&run(&s2k("U1", "U1", PATTERN_TYPE)).unwrap());
    let as_accel = u_at_0_1(&run(&s2k("U1", "U1", ACCEL_TYPE)).unwrap());
    assert!((as_pattern - PATTERN_RESULT).abs() < TOL, "{as_pattern}");
    assert!((as_accel - ACCEL_RESULT).abs() < TOL, "{as_accel}");
}

#[test]
fn a_legacy_input_that_cannot_tell_is_refused_naming_the_ambiguity() {
    for name in ["X", "U1"] {
        let err = run(&s2k(name, name, "")).unwrap_err();
        for want in ["HIST", &format!("'{name}'"), "ambiguous", "load pattern", "ground acceleration"] {
            assert!(err.contains(want), "{name}: {want} in {err}");
        }
    }
}

#[test]
fn a_legacy_input_with_only_one_possible_reading_keeps_working() {
    // Pattern named P: not an acceleration name, so only one reading exists.
    let model = run(&s2k("P", "P", "")).unwrap();
    assert_eq!(model.load_cases["HIST"].history_kind(0), HistoryLoadKind::Unspecified);
    assert!((u_at_0_1(&model) - PATTERN_RESULT).abs() < TOL);
    // U1 with no pattern of that name: only the acceleration exists.
    let model = run(&s2k("P", "U1", "")).unwrap();
    assert!((u_at_0_1(&model) - ACCEL_RESULT).abs() < TOL);
}

#[test]
fn a_mismatched_explicit_kind_is_refused() {
    // Accel with a name that is not a direction: refused by the reader.
    let err = parse_s2k(&s2k("P", "P", ACCEL_TYPE)).unwrap_err().to_string();
    assert!(err.contains("Accel load 'P'") && err.contains("HIST"), "{err}");
    let err = parse_s2k(&s2k("P", "P", "LoadType=Bogus")).unwrap_err().to_string();
    assert!(err.contains("'Bogus'") && err.contains("HIST"), "{err}");
}

#[test]
fn the_written_s2k_keeps_the_kind_for_all_three_names() {
    for name in ["P", "X", "U1"] {
        for (type_text, kind, want) in [
            (PATTERN_TYPE, HistoryLoadKind::Pattern, PATTERN_RESULT),
            (ACCEL_TYPE, HistoryLoadKind::Acceleration, ACCEL_RESULT),
        ] {
            // An acceleration needs a direction name; P is only a pattern.
            if kind == HistoryLoadKind::Acceleration && name == "P" {
                continue;
            }
            let first = parse_s2k(&s2k(name, name, type_text)).unwrap();
            let written = write_s2k(&first);
            let mut again = parse_s2k(&written).unwrap();
            assert_eq!(again.load_cases["HIST"].history_kind(0), kind, "{name}\n{written}");
            analyze_model(&mut again).unwrap();
            let u = u_at_0_1(&again);
            assert!((u - want).abs() < TOL, "{name} {kind:?}: {u} vs {want}");
        }
    }
}

/// Direct integration (no modal case): Newmark average acceleration with
/// dt = 0.001 s. Its period error at w t = 1 is about (w dt)^2 / 12 ~ 8e-6 rad,
/// i.e. under 1e-6 m in u(0.1 s); the tolerance is 5e-6 m.
#[test]
fn direct_integration_follows_the_stored_kind_too() {
    let direct = |name: &str, type_text: &str| {
        let text = s2k(name, name, type_text).replace(
            "RustModalCase=MODAL NumSteps=5 StepSize=0.1",
            "NumSteps=100 StepSize=0.001",
        );
        run(&text)
    };
    for name in ["P", "X", "U1"] {
        let u = u_at_0_1(&direct(name, PATTERN_TYPE).unwrap());
        assert!((u - PATTERN_RESULT).abs() < 5e-6, "{name}: {u}");
    }
    let u = u_at_0_1(&direct("U1", ACCEL_TYPE).unwrap());
    assert!((u - ACCEL_RESULT).abs() < 5e-6, "{u}");
    let err = direct("U1", "").unwrap_err();
    assert!(err.contains("ambiguous") && err.contains("'U1'"), "{err}");
}
