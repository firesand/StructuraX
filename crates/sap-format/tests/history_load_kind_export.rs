//! D81: native export of a history load kind, and the reopened file.
//!
//! The round-trip guard compares what an entry MEANS (`resolve_history_load`),
//! not its stored tag or the alias of its direction:
//! * an entry without a kind that resolves without ambiguity exports and is
//!   read back with the explicit kind it resolves to;
//! * an explicit acceleration exports under any direction alias (`U1`, `UX`,
//!   `X`): the record holds the type `m_ltype = 2` and the 1-based direction in
//!   `m_load`, as the vendor's Example 1-022 does, and is read back as `U1`;
//! * an ambiguous entry (a direction name that a load pattern also has, no kind)
//!   is refused with the case, the load and both readings;
//! * a real change is still refused.

use sap_format::extract_sdb_to_model;
use sap_format::s2k::parse_s2k;
use sap_format::sdb::native::pack_model_to_native_sdb;
use sap_model::{HistoryLoadKind, SapModel};

/// The pack's native controls: a pattern called `pattern`, a modal case, one
/// history entry. `kind` is the text of the `LoadType` column or "".
fn s2k(pattern: &str, kind: &str, load: &str) -> String {
    format!(
        r#"TABLE: "JOINT COORDINATES"
 Joint=A XorR=0 Y=0 Z=0
 Joint=B XorR=120 Y=0 Z=0
TABLE: "CONNECTIVITY - FRAME"
 Frame=F JointI=A JointJ=B
TABLE: "LOAD PATTERN DEFINITIONS"
 LoadPat={pattern} DesignType=Other SelfWtMult=0
TABLE: "FUNCTION - TIME HISTORY - USER"
 Name=RAMP Time=0 Value=0 DampRatio=0
 Name=RAMP Time=1 Value=1 DampRatio=0
TABLE: "LOAD CASE DEFINITIONS"
 Case=MODES Type=LinModal NumSteps=200
 Case=MHIST Type=LinHist RustModalCase=MODES NumSteps=200 ConstDamp=0.05
TABLE: "CASE - TIME HISTORY 2 - LOAD ASSIGNMENTS"
 Case=MHIST {kind} Function=RAMP LoadName={load} SF=1
END TABLE DATA
"#
    )
}

fn export(text: &str) -> Result<SapModel, String> {
    let model = parse_s2k(text).map_err(|e| e.to_string())?;
    let sdb = pack_model_to_native_sdb(&model).map_err(|e| e.to_string())?;
    let back = extract_sdb_to_model(&sdb).map_err(|e| e.to_string())?;
    // A second round trip: the record is the whole truth, nothing in the name.
    let again = pack_model_to_native_sdb(&back).map_err(|e| e.to_string())?;
    let second = extract_sdb_to_model(&again).map_err(|e| e.to_string())?;
    assert_eq!(
        second.load_cases["MHIST"].history_loads,
        back.load_cases["MHIST"].history_loads
    );
    assert_eq!(
        second.load_cases["MHIST"].history_kind(0),
        back.load_cases["MHIST"].history_kind(0)
    );
    Ok(back)
}

fn entry(m: &SapModel) -> (HistoryLoadKind, String) {
    let lc = &m.load_cases["MHIST"];
    (lc.history_kind(0), lc.history_loads[0].1.clone())
}

#[test]
fn unambiguous_legacy_u1_exports_and_reopens_as_acceleration_u1() {
    // Pattern X exists, no pattern U1: legacy U1 can only be the acceleration.
    let back = export(&s2k("X", "", "U1")).unwrap();
    assert_eq!(entry(&back), (HistoryLoadKind::Acceleration, "U1".into()));
}

#[test]
fn explicit_accel_u1_exports_and_reopens_as_acceleration_u1() {
    let back = export(&s2k("X", "LoadType=Accel", "U1")).unwrap();
    assert_eq!(entry(&back), (HistoryLoadKind::Acceleration, "U1".into()));
}

#[test]
fn explicit_accel_x_exports_as_the_direction_not_the_pattern_called_x() {
    // The pattern X exists; the entry is the acceleration in direction 1.
    for alias in ["X", "UX", "U1", "x"] {
        let back = export(&s2k("X", "LoadType=Accel", alias)).unwrap();
        assert_eq!(
            entry(&back),
            (HistoryLoadKind::Acceleration, "U1".into()),
            "{alias}"
        );
    }
    // The other directions keep their index.
    let back = export(&s2k("X", "LoadType=Accel", "UZ")).unwrap();
    assert_eq!(entry(&back), (HistoryLoadKind::Acceleration, "U3".into()));
}

#[test]
fn an_explicit_pattern_named_like_a_direction_stays_a_pattern() {
    let back = export(&s2k("X", "LoadType=\"Load pattern\"", "X")).unwrap();
    assert_eq!(entry(&back), (HistoryLoadKind::Pattern, "X".into()));
}

#[test]
fn a_legacy_pattern_with_an_ordinary_name_exports_as_a_pattern() {
    let back = export(&s2k("WIND", "", "WIND")).unwrap();
    assert_eq!(entry(&back), (HistoryLoadKind::Pattern, "WIND".into()));
}

#[test]
fn ambiguous_legacy_x_with_a_pattern_x_is_refused_naming_case_load_and_both_readings() {
    let err = export(&s2k("X", "", "X")).unwrap_err();
    for needle in [
        "MHIST",
        "'X'",
        "load pattern 'X'",
        "ground acceleration in direction U1",
    ] {
        assert!(err.contains(needle), "{needle}: {err}");
    }
    // No pattern X: the same name is no longer ambiguous.
    let back = export(&s2k("WIND", "", "X")).unwrap();
    assert_eq!(entry(&back), (HistoryLoadKind::Acceleration, "U1".into()));
}

#[test]
fn what_the_format_cannot_hold_is_refused_not_substituted() {
    // An explicit pattern the model does not have.
    let err = export(&s2k("WIND", "LoadType=\"Load pattern\"", "X")).unwrap_err();
    assert!(err.contains("missing referenced name 'X'"), "{err}");
    // An acceleration entry whose name is no direction.
    let mut m = parse_s2k(&s2k("WIND", "LoadType=Accel", "U1")).unwrap();
    m.load_cases.get_mut("MHIST").unwrap().history_loads[0].1 = "FOO".into();
    let err = pack_model_to_native_sdb(&m).unwrap_err().to_string();
    assert!(err.contains("missing referenced name 'FOO'"), "{err}");
}

#[test]
fn the_guard_still_refuses_a_real_change_beside_a_resolvable_entry() {
    // The history entry is fine; a time-integration parameter is not carried.
    let mut m = parse_s2k(&s2k("X", "", "U1")).unwrap();
    assert!(pack_model_to_native_sdb(&m).is_ok());
    let case = m.load_cases.get_mut("MHIST").unwrap();
    case.hht_alpha = -0.123;
    case.wilson_theta = 1.7;
    let err = pack_model_to_native_sdb(&m).unwrap_err().to_string();
    assert!(err.contains("would change represented model data"), "{err}");
}
