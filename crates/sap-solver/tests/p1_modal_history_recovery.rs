//! D75 (frame forces of a history case include the span loads at the reported
//! time) and D76 (base reactions of a history case include the joints that
//! area meshing generates).
//!
//! Models are written as S2K text and go through `parse_s2k` and
//! `analyze_model`. Units are kN and m. The history cases are exercised both
//! as modal superposition (`RustModalCase`) and as direct integration (no
//! modal case); the two share the recovery.
//!
//! References, written next to each assertion:
//! * fixed-fixed beam, span L under a uniform load w: end shear wL/2, end
//!   moment wL^2/12, mid-span moment wL^2/24 (opposite sense to the ends);
//! * fixed-fixed beam with a point load Q at mid-span: end shear Q/2, end
//!   moment QL/8, mid-span moment QL/8 (opposite sense);
//! * restrained shell of mass M under a ground acceleration a: base shear
//!   M*a and, with the mass centroid at height y_c above the origin of the
//!   moments, base moment about Z of -y_c*M*a.

use sap_format::parse_s2k;
use sap_model::SapModel;
use sap_solver::analyze_model;

const L: f64 = 6.0;
const W: f64 = 2.0;

/// Fixed-fixed beam I-K (span 6 m along X) with a free oscillator J (spring
/// 100 kN/m, mass 1, free in U1). `loads` are extra frame-load rows,
/// `history` the history-case definition rows, `functions` the function rows.
fn beam_text(loads: &str, patterns: &str, functions: &str, history: &str, cases: &str) -> String {
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
 LoadPat=P DesignType=Other SelfWtMult=0
 LoadPat=Q DesignType=Other SelfWtMult=0
{patterns}
TABLE: "JOINT LOADS - FORCE"
 Joint=J LoadPat=P F1=10
TABLE: "FRAME LOADS - DISTRIBUTED"
 Frame=F LoadPat=P CoordSys=GLOBAL Dir=6 RelDistA=0 RelDistB=1 FOverLA=2 FOverLB=2
TABLE: "FRAME LOADS - POINT"
 Frame=F LoadPat=Q CoordSys=GLOBAL Dir=6 DistType=RelDist RelDist=0.5 Force=12
{loads}
TABLE: "FUNCTION - TIME HISTORY - USER"
{functions}
TABLE: "LOAD CASE DEFINITIONS"
 Case=P Type=LinStatic RunCase=Yes
 Case=Q Type=LinStatic RunCase=Yes
 Case=MODAL Type=Modal MaxNumModes=1 MinNumModes=1
{cases}
TABLE: "CASE - TIME HISTORY 2 - LOAD ASSIGNMENTS"
{history}
END TABLE DATA
"#
    )
}

const CONST_FN: &str = " Name=CONST Time=0 Value=1\n Name=CONST Time=1 Value=1";
const MODAL_HIST: &str = " Case=HIST Type=LinHist RunCase=Yes RustModalCase=MODAL NumSteps=5 StepSize=0.1 ConstDamp=0";
const DIRECT_HIST: &str =
    " Case=HIST Type=LinHist RunCase=Yes NumSteps=5 StepSize=0.1 ConstDamp=0";

fn run(text: &str) -> SapModel {
    let mut m = parse_s2k(text).expect("S2K parses");
    analyze_model(&mut m).expect("analysis runs");
    m
}

fn close(actual: f64, expected: f64, tol: f64, what: &str) {
    assert!(
        (actual - expected).abs() <= tol,
        "{what}: {actual} vs expected {expected} (tol {tol})"
    );
}

/// End forces of frame F in `case`: (V2 at I, V2 at J, M3 at I, M3 at J).
fn ends(m: &SapModel, case: &str) -> (f64, f64, f64, f64) {
    let st = &m.case_results[case].frame_forces["F"];
    let (a, b) = (st.first().unwrap(), st.last().unwrap());
    (a.v2, b.v2, a.m3, b.m3)
}

fn assert_hist_equals_static(m: &SapModel, stat: &str, factor: f64, tol: f64) {
    let hist = &m.case_results["HIST"].frame_forces["F"];
    let st = &m.case_results[stat].frame_forces["F"];
    assert_eq!(hist.len(), st.len());
    for (h, s) in hist.iter().zip(st) {
        for (hv, sv, what) in [
            (h.p, s.p, "P"),
            (h.v2, s.v2, "V2"),
            (h.v3, s.v3, "V3"),
            (h.t, s.t, "T"),
            (h.m2, s.m2, "M2"),
            (h.m3, s.m3, "M3"),
        ] {
            close(
                hv,
                factor * sv,
                tol,
                &format!("{what} at rel_dist {}", h.rel_dist),
            );
        }
    }
    for j in ["I", "K"] {
        let (rh, rs) = (
            m.case_results["HIST"].joint_reactions[j],
            m.case_results[stat].joint_reactions[j],
        );
        for d in 0..6 {
            close(rh[d], factor * rs[d], tol, &format!("reaction {j}[{d}]"));
        }
    }
}

fn uniform_case(history_cases: &str) -> SapModel {
    run(&beam_text(
        "",
        "",
        CONST_FN,
        " Case=HIST Function=CONST LoadName=P SF=1",
        history_cases,
    ))
}

/// D75, modal superposition: the review's model. Static P and history HIST
/// both give |V| = wL/2 = 6 kN and |M| = wL^2/12 = 6 kN m at both ends. The
/// beam's own modes are not in the single requested mode (the oscillator),
/// and its joints are fixed, so the history response of the beam is exactly
/// the static one.
#[test]
fn modal_history_uniform_load_end_shears_and_moments_equal_the_static_case() {
    let m = uniform_case(MODAL_HIST);
    let (v_i, v_j, m_i, m_j) = ends(&m, "HIST");
    close(v_i.abs(), W * L / 2.0, 1e-9, "|V2| at I");
    close(v_j.abs(), W * L / 2.0, 1e-9, "|V2| at K");
    close(m_i.abs(), W * L * L / 12.0, 1e-9, "|M3| at I");
    close(m_j.abs(), W * L * L / 12.0, 1e-9, "|M3| at K");
    // Station by station and in the support reactions, equal to the static case.
    assert_hist_equals_static(&m, "P", 1.0, 1e-9);
    // Mid-span moment wL^2/24, opposite in sense to the ends.
    let mid = m.case_results["HIST"].frame_forces["F"]
        .iter()
        .find(|s| (s.rel_dist - 0.5).abs() < 1e-9)
        .expect("mid-span station");
    close(mid.m3.abs(), W * L * L / 24.0, 1e-9, "|M3| at mid-span");
    assert!(mid.m3 * m_i < 0.0, "mid-span and end moments differ in sense");
    // Support reactions of both ends: vertical wL/2 and moment wL^2/12.
    let r = &m.case_results["HIST"].joint_reactions;
    close(r["I"][2].abs(), W * L / 2.0, 1e-9, "Rz at I");
    close(r["K"][2].abs(), W * L / 2.0, 1e-9, "Rz at K");
    close(r["I"][4].abs(), W * L * L / 12.0, 1e-9, "My at I");
    close(r["K"][4].abs(), W * L * L / 12.0, 1e-9, "My at K");
}

/// The same recovery serves direct integration (D75), where the loads were
/// missing from the frame forces in the same way.
#[test]
fn direct_history_uniform_load_end_shears_and_moments_equal_the_static_case() {
    let m = uniform_case(DIRECT_HIST);
    let (v_i, v_j, m_i, m_j) = ends(&m, "HIST");
    close(v_i.abs(), W * L / 2.0, 1e-9, "|V2| at I");
    close(v_j.abs(), W * L / 2.0, 1e-9, "|V2| at K");
    close(m_i.abs(), W * L * L / 12.0, 1e-9, "|M3| at I");
    close(m_j.abs(), W * L * L / 12.0, 1e-9, "|M3| at K");
    assert_hist_equals_static(&m, "P", 1.0, 1e-9);
}

/// Point load Q = 12 kN at mid-span of the fixed-fixed beam: end shear Q/2 =
/// 6 kN, end moment QL/8 = 9 kN m, mid-span moment QL/8 = 9 kN m of opposite
/// sense. Point loads take the same path as the span loads.
#[test]
fn history_point_load_forces_match_the_closed_form_in_both_integrators() {
    for cases in [MODAL_HIST, DIRECT_HIST] {
        let m = run(&beam_text(
            "",
            "",
            CONST_FN,
            " Case=HIST Function=CONST LoadName=Q SF=1",
            cases,
        ));
        let q = 12.0;
        let (v_i, v_j, m_i, m_j) = ends(&m, "HIST");
        close(v_i.abs(), q / 2.0, 1e-9, "|V2| at I");
        close(v_j.abs(), q / 2.0, 1e-9, "|V2| at K");
        close(m_i.abs(), q * L / 8.0, 1e-9, "|M3| at I");
        close(m_j.abs(), q * L / 8.0, 1e-9, "|M3| at K");
        assert_hist_equals_static(&m, "Q", 1.0, 1e-9);
        let r = &m.case_results["HIST"].joint_reactions;
        close(r["I"][2].abs(), q / 2.0, 1e-9, "Rz at I");
        close(r["K"][4].abs(), q * L / 8.0, 1e-9, "My at K");
    }
}

/// Trapezoidal span load, w = 1 kN/m at I rising to 3 kN/m at K (an
/// additional table in the same file). The default section is shear-flexible
/// in kN and m, so the Euler-Bernoulli split (4.8 / 7.2 kN) does not apply;
/// the references are the exact statics |V_I| + |V_K| = (w1 + w2) L / 2 =
/// 12 kN, the end-moment sum of 12 kN m (= (w1 + w2) L^2 / 6 minus the
/// unequal-shear moment, checked through the base moment 12 * 3.5 = 42 kN m
/// about Y from the centroid at L (w1 + 2 w2) / (3 (w1 + w2)) = 3.5 m) and
/// equality with the static case T station by station.
#[test]
fn history_trapezoidal_load_forces_match_the_closed_form_in_both_integrators() {
    for cases in [MODAL_HIST, DIRECT_HIST] {
        let m = run(&beam_text(
            "TABLE: \"FRAME LOADS - DISTRIBUTED\"\n Frame=F LoadPat=T CoordSys=GLOBAL Dir=6 RelDistA=0 RelDistB=1 FOverLA=1 FOverLB=3",
            " LoadPat=T DesignType=Other SelfWtMult=0",
            CONST_FN,
            " Case=HIST Function=CONST LoadName=T SF=1",
            &format!(" Case=T Type=LinStatic RunCase=Yes\n{cases}"),
        ));
        let (v_i, v_j, m_i, m_j) = ends(&m, "HIST");
        close(v_i.abs() + v_j.abs(), 12.0, 1e-9, "|V_I| + |V_K|");
        assert!(v_i.abs() < v_j.abs(), "heavier end carries more shear");
        close(m_i.abs() + m_j.abs(), 12.0, 0.1, "|M_I| + |M_K| (about 12)");
        let base = m.case_results["HIST"].base_reactions;
        close(base[4], 42.0, 1e-9, "base My = 12 kN * 3.5 m");
        assert_hist_equals_static(&m, "T", 1.0, 1e-9);
    }
}

/// Negative: self weight in a history pattern is still refused, in both
/// integrators, naming the pattern (the recovery of member loads does not
/// widen what the history accepts).
#[test]
fn history_with_a_self_weight_pattern_is_still_refused() {
    for cases in [MODAL_HIST, DIRECT_HIST] {
        let mut m = parse_s2k(&beam_text(
            "",
            " LoadPat=G DesignType=Dead SelfWtMult=1",
            CONST_FN,
            " Case=HIST Function=CONST LoadName=G SF=1",
            cases,
        ))
        .expect("parses");
        let error = analyze_model(&mut m).unwrap_err().to_string();
        assert!(error.contains("'G'") && error.contains("self-weight"), "{error}");
    }
}

/// Scale factor and time function: SF = 1.5 and a function rising linearly
/// from 0 at t = 0 to 1 at t = 2 s. The undamped oscillator driven by the 10
/// kN joint force through the same function moves monotonically, so the
/// stored step is the last, t = 1 s, where the function is 0.5 and the member
/// load is 1.5 * 0.5 = 0.75 of the pattern: V = 0.75 * 6 = 4.5 kN and
/// M = 0.75 * 6 = 4.5 kN m. A negative factor flips the signs.
#[test]
fn history_member_loads_follow_scale_factor_and_time_function_at_the_reported_time() {
    let ramp = " Name=RAMP Time=0 Value=0\n Name=RAMP Time=2 Value=1";
    for cases in [
        " Case=HIST Type=LinHist RunCase=Yes RustModalCase=MODAL NumSteps=5 StepSize=0.2 ConstDamp=0",
        " Case=HIST Type=LinHist RunCase=Yes NumSteps=5 StepSize=0.2 ConstDamp=0",
    ] {
        let mut signs = Vec::new();
        for sf in [1.5, -1.5] {
            let m = run(&beam_text(
                "",
                "",
                ramp,
                &format!(" Case=HIST Function=RAMP LoadName=P SF={sf}"),
                cases,
            ));
            let t = m.case_results["HIST"].time_steps.last().copied().unwrap();
            close(t, 1.0, 1e-12, "last output time");
            let factor = sf * 0.5; // SF * f(1 s)
            assert_hist_equals_static(&m, "P", factor, 1e-9);
            let (v_i, _, m_i, _) = ends(&m, "HIST");
            close(v_i.abs(), 0.75 * W * L / 2.0, 1e-9, "|V2| at I");
            close(m_i.abs(), 0.75 * W * L * L / 12.0, 1e-9, "|M3| at I");
            signs.push(v_i.signum());
        }
        assert_eq!(signs[0], -signs[1], "negative SF flips the member forces");
    }
}

/// Negative: a pattern whose function is zero, and one applied with scale
/// factor zero, carry no member force although the static cases P and Q have
/// them.
#[test]
fn history_without_the_pattern_or_with_zero_function_has_no_member_forces() {
    for cases in [MODAL_HIST, DIRECT_HIST] {
        let zero = " Name=ZERO Time=0 Value=0\n Name=ZERO Time=1 Value=0";
        let m = run(&beam_text(
            "",
            "",
            &format!("{CONST_FN}\n{zero}"),
            " Case=HIST Function=ZERO LoadName=P SF=1\n Case=HIST Function=CONST LoadName=Q SF=0",
            cases,
        ));
        let (v_i, v_j, m_i, m_j) = ends(&m, "HIST");
        for (v, what) in [(v_i, "V2 I"), (v_j, "V2 K"), (m_i, "M3 I"), (m_j, "M3 K")] {
            close(v, 0.0, 1e-12, what);
        }
    }
}

// ---------------------------------------------------------------------------
// D76
// ---------------------------------------------------------------------------

/// Fully restrained 2 m x 2 m shell in the XY plane, meshed `n` x `n`, mass
/// per area 1 kN s^2/m^3-consistent unit (the model's own number), driven by a
/// constant U1 ground acceleration of 1 m/s^2 (function value 1, SF 1). A
/// separate oscillator J (restrained in U1, free in U2) supplies the free mode.
fn shell_text(n: usize, cases: &str) -> String {
    format!(
        r#"TABLE: "PROGRAM CONTROL"
 CurrUnits="kN, m, C"
TABLE: "JOINT COORDINATES"
 Joint=A XorR=0 Y=0 Z=0
 Joint=B XorR=2 Y=0 Z=0
 Joint=C XorR=2 Y=2 Z=0
 Joint=D XorR=0 Y=2 Z=0
 Joint=J XorR=10 Y=0 Z=0
TABLE: "JOINT RESTRAINT ASSIGNMENTS"
 Joint=A U1=Yes U2=Yes U3=Yes R1=Yes R2=Yes R3=Yes
 Joint=B U1=Yes U2=Yes U3=Yes R1=Yes R2=Yes R3=Yes
 Joint=C U1=Yes U2=Yes U3=Yes R1=Yes R2=Yes R3=Yes
 Joint=D U1=Yes U2=Yes U3=Yes R1=Yes R2=Yes R3=Yes
 Joint=J U1=Yes U3=Yes R1=Yes R2=Yes R3=Yes
TABLE: "JOINT SPRING ASSIGNMENTS 1 - UNCOUPLED"
 Joint=J U2=100
TABLE: "JOINT ADDED MASS ASSIGNMENTS"
 Joint=J Mass2=1
TABLE: "CONNECTIVITY - AREA"
 Area=S Joint1=A Joint2=B Joint3=C Joint4=D MeshN1={n} MeshN2={n} MassPerArea=1
TABLE: "FUNCTION - TIME HISTORY - USER"
 Name=CONST Time=0 Value=1
 Name=CONST Time=1 Value=1
TABLE: "LOAD CASE DEFINITIONS"
 Case=MODAL Type=Modal MaxNumModes=1 MinNumModes=1
{cases}
TABLE: "CASE - TIME HISTORY 2 - LOAD ASSIGNMENTS"
 Case=HIST Function=CONST LoadName=U1 SF=1
END TABLE DATA
"#
    )
}

/// Mass of the shell as the model states it: (MassPerArea + t * rho) * A with
/// the section's membrane thickness and the material's unit mass.
fn shell_mass(m: &SapModel) -> f64 {
    let area = &m.areas["S"];
    let sec = &m.area_sections[&area.section];
    (area.mass_per_area + sec.thickness_membrane * m.materials[&sec.material].unit_mass) * 4.0
}

/// D76. Base shear = M a and base moment about Z = -y_c M a with y_c = 1 m,
/// for every mesh, in both integrators; generated joints are counted once
/// (a double count would exceed the closed form). Rows of the results still
/// name only the model's joints.
#[test]
fn modal_history_base_reactions_of_a_meshed_restrained_shell_are_independent_of_the_mesh() {
    meshed_shell_base_reactions(MODAL_HIST);
}

#[test]
fn direct_history_base_reactions_of_a_meshed_restrained_shell_are_independent_of_the_mesh() {
    meshed_shell_base_reactions(DIRECT_HIST);
}

fn meshed_shell_base_reactions(cases: &str) {
    {
        let mut shears = Vec::new();
        for n in [1usize, 2, 4] {
            let m = run(&shell_text(n, cases));
            let mass = shell_mass(&m);
            assert!(mass > 4.0 && mass < 4.001, "mass of the model: {mass}");
            let base = m.case_results["HIST"].base_reactions;
            let tol = 1e-9 * mass;
            close(base[0], mass * 1.0, tol, &format!("n={n} base Fx = M a"));
            close(base[1], 0.0, tol, "base Fy");
            close(base[2], 0.0, tol, "base Fz");
            close(base[3], 0.0, tol, "base Mx");
            close(base[4], 0.0, tol, "base My");
            let y_c = 1.0;
            close(base[5], -y_c * mass, tol, &format!("n={n} base Mz"));
            let rows: Vec<&String> = m.case_results["HIST"].joint_reactions.keys().collect();
            assert!(rows.iter().all(|k| !k.starts_with("__mesh")), "{rows:?}");
            shears.push(base[0]);
        }
        assert!(
            shears.windows(2).all(|w| (w[0] - w[1]).abs() < 1e-12),
            "meshes disagree: {shears:?}"
        );
    }
}

/// D75 and D77 together: a pattern named like a direction ("UZ") that the
/// history entry marks as a load pattern carries its member loads. Point
/// load Q = 12 kN at mid-span: end shear Q/2 = 6 kN, end moment QL/8 = 9 kN m,
/// in both integrators. Marked as an acceleration, the same name loads the
/// joints only: the fixed beam has no member forces. Unmarked, the entry is
/// ambiguous and the case is refused, naming the case and the load.
#[test]
fn a_pattern_named_like_a_direction_carries_its_member_loads_when_marked_as_a_pattern() {
    let text = |load_type: &str, cases: &str| {
        beam_text(
            " Frame=F LoadPat=UZ CoordSys=GLOBAL Dir=6 DistType=RelDist RelDist=0.5 Force=12",
            " LoadPat=UZ DesignType=Other SelfWtMult=0",
            CONST_FN,
            &format!(" Case=HIST Function=CONST LoadName=UZ{load_type} SF=1"),
            &format!(" Case=UZ Type=LinStatic RunCase=Yes\n{cases}"),
        )
    };
    for cases in [MODAL_HIST, DIRECT_HIST] {
        let m = run(&text(" LoadType=\"Load pattern\"", cases));
        let (v_i, v_j, m_i, m_j) = ends(&m, "HIST");
        close(v_i.abs(), 12.0 / 2.0, 1e-9, "|V2| at I");
        close(v_j.abs(), 12.0 / 2.0, 1e-9, "|V2| at K");
        close(m_i.abs(), 12.0 * L / 8.0, 1e-9, "|M3| at I");
        close(m_j.abs(), 12.0 * L / 8.0, 1e-9, "|M3| at K");
        assert_hist_equals_static(&m, "UZ", 1.0, 1e-9);

        let m = run(&text(" LoadType=Accel", cases));
        let (v_i, v_j, m_i, m_j) = ends(&m, "HIST");
        for (v, what) in [(v_i, "V2 at I"), (v_j, "V2 at K"), (m_i, "M3 at I"), (m_j, "M3 at K")] {
            close(v, 0.0, 1e-9, what);
        }

        let mut ambiguous = parse_s2k(&text("", cases)).expect("S2K parses");
        let e = format!("{:?}", analyze_model(&mut ambiguous).unwrap_err());
        assert!(e.contains("HIST") && e.contains("'UZ'") && e.contains("ambiguous"), "{e}");
        assert!(ambiguous.case_results.is_empty());
    }
}
