//! D82: the definition check of a time-history case (`set_case_definition`,
//! the call behind the GUI's case form) accepts exactly the patterns the
//! solver accepts and recovers, and refuses the rest with the solver's reason.
//!
//! Model: a 6 m fixed-fixed beam F (I-K, along X, kN and m) with an
//! oscillator J that only carries the modal case. Pattern P has a uniform
//! 2 kN/m on F (global Z, direction 6) and a 10 kN joint force on J; pattern Q
//! has a 12 kN point load at mid-span of F.
//!
//! References (L = 6 m, fixed-fixed beam):
//! * uniform w = 2: end shear wL/2 = 6 kN (+6 at I, -6 at K in station
//!   coordinates), end moment wL^2/12 = 6 kN m at both ends;
//! * point load Q = 12 at mid-span: end shear Q/2 = 6 kN, end moment
//!   QL/8 = 9 kN m;
//! * support reactions: Rz = wL/2 and My = wL^2/12 (Q/2 and QL/8).
//!
//! The history cases hold the load for the whole record, so the response is
//! the static one (the beam joints are fixed and J is not coupled to the beam).

use sap_api::{cSapModel, set_case_definition, validate_case_definition, OAPI_OK};
use sap_model::*;
use sap_units::Units;

const L: f64 = 6.0;
const W: f64 = 2.0;
const Q: f64 = 12.0;
const TOL: f64 = 1e-9;

const MODEL: &str = r#"TABLE: "PROGRAM CONTROL"
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
 LoadPat=P DesignType=Other SelfWtMult=0
 LoadPat=Q DesignType=Other SelfWtMult=0
 LoadPat=T DesignType=Other SelfWtMult=0
 LoadPat=G DesignType=Other SelfWtMult=0
 LoadPat=S DesignType=Other SelfWtMult=1
 LoadPat=M DesignType=Other SelfWtMult=0
 LoadPat=A DesignType=Other SelfWtMult=0
 LoadPat=V DesignType=Other SelfWtMult=0
TABLE: "JOINT LOADS - FORCE"
 Joint=J LoadPat=P F1=10
TABLE: "FRAME LOADS - DISTRIBUTED"
 Frame=F LoadPat=P CoordSys=GLOBAL Dir=6 RelDistA=0 RelDistB=1 FOverLA=2 FOverLB=2
TABLE: "FRAME LOADS - POINT"
 Frame=F LoadPat=Q CoordSys=GLOBAL Dir=6 DistType=RelDist RelDist=0.5 Force=12
TABLE: "FUNCTION - TIME HISTORY - USER"
 Name=CONST Time=0 Value=1
 Name=CONST Time=1 Value=1
TABLE: "LOAD CASE DEFINITIONS"
 Case=P Type=LinStatic RunCase=Yes
 Case=Q Type=LinStatic RunCase=Yes
 Case=MODAL Type=Modal MaxNumModes=1 MinNumModes=1
END TABLE DATA
"#;

fn api() -> cSapModel {
    let mut a = cSapModel::new(Units::KnMC);
    a.inner = sap_format::parse_s2k(MODEL).expect("S2K parses");
    a
}

fn hist(name: &str, pattern: &str, modal: bool) -> LoadCase {
    let mut h = LoadCase::linear_static(name, pattern, 1.0);
    h.case_type = LoadCaseType::LinearHistory;
    h.modal_damping = 0.0;
    h.loads.clear();
    h.num_time_steps = 5;
    h.time_step_dt = 0.1;
    h.history_loads = vec![("CONST".into(), pattern.into(), 1.0)];
    if modal {
        h.modal_case = Some("MODAL".into());
    }
    h
}

fn close(actual: f64, expected: f64, what: &str) {
    assert!(
        (actual - expected).abs() <= TOL,
        "{what}: {actual} vs expected {expected} (tol {TOL})"
    );
}

/// Frame forces of F and reactions of I and K of `case`, read through the
/// result getters of the API.
fn read(a: &mut cSapModel, case: &str) -> (Vec<FrameStationResult>, [f64; 6], [f64; 6]) {
    a.results().setup_deselect_all_cases_and_combos_for_output();
    assert_eq!(a.results().setup_set_case_selected_for_output(case, true), OAPI_OK);
    let f = a.results().frame_force("F");
    assert_eq!(f.len(), 1, "one row set for {case}");
    let ri = a.results().joint_react("I");
    let rk = a.results().joint_react("K");
    (f[0].1.clone(), ri[0].1, rk[0].1)
}

/// The history case defined through `set_case_definition` for pattern
/// `pattern` (modal superposition or direct integration), the analysis through
/// the API, and the results compared with the static case `stat`.
fn history_through_the_api(pattern: &str, modal: bool) -> (cSapModel, String) {
    let mut a = api();
    let name = "HIST".to_string();
    set_case_definition(&mut a.inner, hist(&name, pattern, modal))
        .unwrap_or_else(|e| panic!("pattern {pattern} (modal {modal}): {e}"));
    assert_eq!(a.analyze().run_analysis(), OAPI_OK);
    (a, name)
}

fn assert_equal_to_static(a: &mut cSapModel, hist: &str, stat: &str) {
    let (hf, hi, hk) = read(a, hist);
    let (sf, si, sk) = read(a, stat);
    assert_eq!(hf.len(), sf.len());
    for (h, s) in hf.iter().zip(&sf) {
        for (x, y, what) in [
            (h.p, s.p, "P"),
            (h.v2, s.v2, "V2"),
            (h.v3, s.v3, "V3"),
            (h.t, s.t, "T"),
            (h.m2, s.m2, "M2"),
            (h.m3, s.m3, "M3"),
        ] {
            close(x, y, &format!("{what} at rel_dist {}", h.rel_dist));
        }
    }
    for d in 0..6 {
        close(hi[d], si[d], &format!("reaction I[{d}]"));
        close(hk[d], sk[d], &format!("reaction K[{d}]"));
    }
}

/// The control of the verification pack: the distributed load was refused
/// ("history patterns currently support joint forces only ...") although the
/// solver recovers it. Both integrators, defined and run through the API.
#[test]
fn uniform_load_history_is_defined_and_gives_the_closed_form_through_the_api() {
    for modal in [true, false] {
        let (mut a, h) = history_through_the_api("P", modal);
        let (f, ri, rk) = read(&mut a, &h);
        let (i, k) = (f.first().unwrap(), f.last().unwrap());
        // wL/2 = 2 * 6 / 2 = 6 kN: +6 at I, -6 at K.
        close(i.v2, W * L / 2.0, "V2 at I");
        close(k.v2, -W * L / 2.0, "V2 at K");
        // wL^2/12 = 2 * 36 / 12 = 6 kN m at both ends.
        close(i.m3, W * L * L / 12.0, "M3 at I");
        close(k.m3, W * L * L / 12.0, "M3 at K");
        // Support reactions: Rz wL/2 and My wL^2/12 in magnitude.
        close(ri[2].abs(), W * L / 2.0, "Rz at I");
        close(rk[2].abs(), W * L / 2.0, "Rz at K");
        close(ri[4].abs(), W * L * L / 12.0, "My at I");
        close(rk[4].abs(), W * L * L / 12.0, "My at K");
        assert_equal_to_static(&mut a, &h, "P");
    }
}

/// Point load Q = 12 kN at mid-span: Q/2 = 6 kN, QL/8 = 9 kN m.
#[test]
fn point_load_history_is_defined_and_gives_the_closed_form_through_the_api() {
    for modal in [true, false] {
        let (mut a, h) = history_through_the_api("Q", modal);
        let (f, ri, rk) = read(&mut a, &h);
        let (i, k) = (f.first().unwrap(), f.last().unwrap());
        close(i.v2.abs(), Q / 2.0, "|V2| at I");
        close(k.v2.abs(), Q / 2.0, "|V2| at K");
        close(i.m3.abs(), Q * L / 8.0, "|M3| at I");
        close(k.m3.abs(), Q * L / 8.0, "|M3| at K");
        close(ri[2].abs(), Q / 2.0, "Rz at I");
        close(rk[4].abs(), Q * L / 8.0, "My at K");
        assert_equal_to_static(&mut a, &h, "Q");
    }
}

/// The joint-only control of the pack keeps passing: a pattern with joint
/// forces only (the J force of P is part of P; use a pattern that has nothing
/// else, a new joint-force pattern 'J1').
#[test]
fn joint_only_pattern_is_still_accepted() {
    let mut a = api();
    a.inner.load_patterns.insert(
        "J1".into(),
        LoadPattern {
            name: "J1".into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
    a.inner.points.get_mut("J").unwrap().loads_force.push(PointLoadForce {
        load_pat: "J1".into(),
        csys: "GLOBAL".into(),
        force: [5.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    });
    set_case_definition(&mut a.inner, hist("HIST", "J1", true)).unwrap();
    assert_eq!(a.analyze().run_analysis(), OAPI_OK);
}

/// Adds to the model the one load type that pattern `pattern` is named for.
fn add_refused_load(m: &mut SapModel, pattern: &str) -> &'static str {
    match pattern {
        "T" => {
            m.frames.get_mut("F").unwrap().loads_temp.push(FrameLoadTemperature {
                load_pat: "T".into(),
                my_type: 1,
                val: 10.0,
                pattern_name: None,
            });
            "frame temperature loads"
        }
        "G" => {
            m.frames
                .get_mut("F")
                .unwrap()
                .loads_gravity
                .insert("G".into(), [0.0, 0.0, -1.0]);
            "frame gravity loads"
        }
        "S" => "self-weight",
        "M" => {
            m.points.get_mut("I").unwrap().loads_displ.push(PointLoadDispl {
                load_pat: "M".into(),
                csys: "GLOBAL".into(),
                displ: [0.01, 0.0, 0.0, 0.0, 0.0, 0.0],
            });
            "support displacement (motion) loads"
        }
        "A" => {
            let mut area = AreaObj::new("AR", vec!["I".into(), "K".into(), "J".into()], "NONE");
            area.loads_uniform.push(AreaLoadUniform {
                load_pat: "A".into(),
                csys: "GLOBAL".into(),
                dir: 6,
                val: 1.0,
            });
            area.loads_pressure.push(AreaLoadSurfacePressure {
                load_pat: "A".into(),
                face: -2,
                val: 1.0,
                pattern_name: None,
            });
            area.loads_temp.push(AreaLoadTemperature {
                load_pat: "A".into(),
                my_type: 1,
                val: 5.0,
                pattern_name: None,
            });
            m.areas.insert("AR".into(), area);
            "area uniform loads"
        }
        "V" => {
            let p = |n: &str| n.to_string();
            let solid = SolidObj {
                name: p("SO"),
                points: [p("I"), p("K"), p("J"), p("I"), p("K"), p("J"), p("I"), p("K")],
                section: p("NONE"),
                loads_surface: vec![SolidSurfaceLoad {
                    load_pat: p("V"),
                    face: 1,
                    val: 1.0,
                }],
                loads_temp: [(p("V"), 5.0)].into_iter().collect(),
                selected: false,
            };
            m.solids.insert(p("SO"), solid);
            "solid surface loads"
        }
        _ => unreachable!(),
    }
}

/// Negative controls: every load type that a history still refuses names the
/// pattern and the load type, in the definition check and in the solver, and
/// the refused definition is not stored.
#[test]
fn unsupported_history_load_types_are_refused_naming_pattern_and_type() {
    for pattern in ["T", "G", "S", "M", "A", "V"] {
        let mut a = api();
        let kind = add_refused_load(&mut a.inner, pattern);
        let case = hist("HIST", pattern, true);
        let err = set_case_definition(&mut a.inner, case.clone()).unwrap_err().to_string();
        assert!(
            err.contains(&format!("history pattern '{pattern}'")) && err.contains(kind),
            "pattern {pattern}: {err}"
        );
        assert!(!a.inner.load_cases.contains_key("HIST"));
        // The same text from the one shared predicate the solver validates with.
        let reason = sap_solver::history_pattern_refusal(&a.inner, pattern).unwrap();
        assert!(err.contains(&reason), "{err} / {reason}");
        // Direct integration is refused alike.
        let mut direct = case;
        direct.modal_case = None;
        assert!(validate_case_definition(&a.inner, &direct).is_err());
    }
    // A pattern that carries several refused types names each of them.
    let mut a = api();
    add_refused_load(&mut a.inner, "A");
    let reason = sap_solver::history_pattern_refusal(&a.inner, "A").unwrap();
    for t in ["area uniform loads", "area surface pressure loads", "area temperature loads"] {
        assert!(reason.contains(t), "{reason}");
    }
}

/// The solver refuses the same patterns, with the same reason, when a case
/// reaches it without the definition check (for example from a file).
#[test]
fn the_solver_refuses_the_same_patterns_with_the_same_reason() {
    for pattern in ["T", "G", "S", "M"] {
        let mut a = api();
        let kind = add_refused_load(&mut a.inner, pattern);
        a.inner
            .load_cases
            .insert("HIST".into(), hist("HIST", pattern, true));
        let err = sap_solver::analyze_model(&mut a.inner).unwrap_err().to_string();
        assert!(
            err.contains(&format!("history pattern '{pattern}'")) && err.contains(kind),
            "pattern {pattern}: {err}"
        );
    }
}

/// Other rules of `validate_case_definition` are unchanged: a missing pattern
/// and a pattern of an entry marked as an acceleration.
#[test]
fn missing_pattern_is_still_refused() {
    let a = api();
    let err = validate_case_definition(&a.inner, &hist("HIST", "NOPE", true))
        .unwrap_err()
        .to_string();
    assert!(err.contains("NOPE"), "{err}");
}

/// Solver-level negatives for an area and a solid (valid sections and
/// joints, so the refusal comes from the history validation): the solver now
/// refuses solid loads in a history too (D82; its right-hand side has no
/// solid part, so they were dropped silently before).
#[test]
fn the_solver_refuses_area_and_solid_loads_in_a_history_naming_the_type() {
    for (pattern, kind) in [("A", "area uniform loads"), ("V", "solid surface loads")] {
        let mut a = api();
        for (n, x, y, z) in [
            ("B1", 0.0, 2.0, 0.0),
            ("B2", 1.0, 2.0, 0.0),
            ("B3", 1.0, 3.0, 0.0),
            ("B4", 0.0, 3.0, 0.0),
            ("B5", 0.0, 2.0, 1.0),
            ("B6", 1.0, 2.0, 1.0),
            ("B7", 1.0, 3.0, 1.0),
            ("B8", 0.0, 3.0, 1.0),
        ] {
            a.inner.add_point(n, x, y, z);
        }
        if pattern == "A" {
            let section = a.inner.area_sections.keys().next().unwrap().clone();
            let mut area = AreaObj::new("AR", vec!["B1".into(), "B2".into(), "B3".into()], section);
            area.loads_uniform.push(AreaLoadUniform {
                load_pat: "A".into(),
                csys: "GLOBAL".into(),
                dir: 6,
                val: 1.0,
            });
            a.inner.areas.insert("AR".into(), area);
        } else {
            let mut sec = SolidSection::default();
            sec.material = a.inner.materials.keys().next().unwrap().clone();
            let section = sec.name.clone();
            a.inner.solid_sections.insert(section.clone(), sec);
            a.inner.solids.insert(
                "SO".into(),
                SolidObj {
                    name: "SO".into(),
                    points: ["B1", "B2", "B4", "B3", "B5", "B6", "B8", "B7"].map(String::from),
                    section,
                    loads_surface: vec![SolidSurfaceLoad {
                        load_pat: "V".into(),
                        face: 1,
                        val: 1.0,
                    }],
                    loads_temp: Default::default(),
                    selected: false,
                },
            );
        }
        a.inner
            .load_cases
            .insert("HIST".into(), hist("HIST", pattern, true));
        let err = sap_solver::analyze_model(&mut a.inner).unwrap_err().to_string();
        assert!(
            err.contains(&format!("history pattern '{pattern}'")) && err.contains(kind),
            "{err}"
        );
    }
}

/// The history case types that share the validation give the same closed
/// form through the API (direct integration, linear and nonlinear types).
#[test]
fn every_admitted_history_type_gives_the_closed_form_for_the_uniform_load() {
    for ty in [
        LoadCaseType::LinearDynamic,
        LoadCaseType::NonlinearHistory,
        LoadCaseType::NonlinearDynamic,
    ] {
        let mut a = api();
        let mut c = hist("HIST", "P", false);
        c.case_type = ty;
        if let Err(e) = set_case_definition(&mut a.inner, c) {
            panic!("{ty:?} refused: {e}");
        }
        assert_eq!(a.analyze().run_analysis(), OAPI_OK, "{ty:?}");
        let (f, _, _) = read(&mut a, "HIST");
        let (i, k) = (f.first().unwrap(), f.last().unwrap());
        close(i.v2, W * L / 2.0, &format!("{ty:?} V2 at I"));
        close(k.v2, -W * L / 2.0, &format!("{ty:?} V2 at K"));
        close(i.m3, W * L * L / 12.0, &format!("{ty:?} M3 at I"));
        assert_equal_to_static(&mut a, "HIST", "P");
    }
}
