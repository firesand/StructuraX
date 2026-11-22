//! The convergence of the P-Delta axial forces on the hinge path, and the
//! direction of a displacement-controlled push that continues under P-Delta.
//! Found by an independent verification on 2026-11-18.
//!
//! The model is a cantilever strip of length L along X, fixed at I, bending
//! in the X-Y plane (flexural rigidity EI about local 2), with a rotational
//! hinge at the free end J that carries no moment and never yields. Units kN
//! with metres or millimetres; the same structure is written in both.
//!
//! * `PUSH`: a lateral force H0 at the tip, P-Delta, no axial force.
//! * `FOLLOW` continues from `PUSH` with, per unit load factor, a compression
//!   N1 and a further lateral force H0, and is pushed by displacement control
//!   until the tip has moved a further 4.5 mm.
//!
//! Reference (closed form of the element the program uses): with the cubic
//! geometric stiffness of a compression N on one element of length L,
//!   K_uu = 12 EI / L^3 - 1.2 N / L,
//!   K_ur = -6 EI / L^2 + 0.1 N,
//!   K_rr = 4 EI / L - 2 N L / 15,
//! the stiffness of the tip with its rotation free is
//!   k(N) = K_uu - K_ur^2 / K_rr,
//! and the tip deflection under a lateral force H is H / k(N). At load factor
//! f: H = H0 (1 + f), N = N1 f. The factor that brings the tip to the total
//! deflection u is the root of H0 (1 + f) / k(N1 f) = u, found by bisection.
use sap_format::parse_s2k;
use sap_model::*;
use sap_solver::analyze_model;

/// Length (m), flexural rigidity about local 2 (kN m^2), lateral force of
/// `PUSH` (kN), compression of `FOLLOW` per unit factor (kN), further tip
/// deflection asked of `FOLLOW` (m).
const L: f64 = 3.0;
const EI: f64 = 2.0e8 * 4.5e-9;
const H0: f64 = 0.000045;
const N1: f64 = 0.0045;
const FURTHER: f64 = 0.0045;

/// Tip stiffness of the one-element cantilever under a compression `n`.
fn tip_stiffness(n: f64) -> f64 {
    let kuu = 12.0 * EI / L.powi(3) - 1.2 * n / L;
    let kur = -6.0 * EI / (L * L) + 0.1 * n;
    let krr = 4.0 * EI / L - 2.0 * n * L / 15.0;
    kuu - kur * kur / krr
}

/// Tip deflection (m) at load factor `f` of `FOLLOW`.
fn tip_at(f: f64) -> f64 {
    H0 * (1.0 + f) / tip_stiffness(N1 * f)
}

/// The load factor at which the tip has moved `FURTHER` beyond `PUSH`.
fn closed_form_factor() -> f64 {
    let target = H0 / tip_stiffness(0.0) + FURTHER;
    let (mut lo, mut hi) = (0.0, 20.0);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if tip_at(mid) < target {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

/// What `FOLLOW` does: push to a further tip deflection, or apply a factor.
enum Follow {
    Monitored,
    Load(f64),
}

/// The cantilever in kN and metres (`mm = 1`) or kN and millimetres
/// (`mm = 1000`). `capacity` is the hinge moment in kN m; `tol` an optional
/// convergence tolerance for both cases.
fn model(mm: f64, capacity: f64, tol: Option<f64>, follow: Follow) -> SapModel {
    let units = if mm == 1.0 { "kN, m, C" } else { "kN, mm, C" };
    let tol = tol.map(|t| format!(" ConvTol={t:e}")).unwrap_or_default();
    let (control, factor) = match follow {
        Follow::Monitored => (
            format!(
                " RustDisplTarget={} RustDisplJoint=J RustDisplDOF=U2 RustDisplConjugate=No",
                FURTHER * mm
            ),
            1.0,
        ),
        Follow::Load(f) => (String::new(), f),
    };
    let text = format!(
        r#"TABLE: "PROGRAM CONTROL"
 CurrUnits="{units}"
TABLE: "MATERIAL PROPERTIES 02 - BASIC MECHANICAL PROPERTIES"
 Material=MAT E1={e} G12={g} U12=0.3 UnitWeight=0 UnitMass=0
TABLE: "JOINT COORDINATES"
 Joint=I XorR=0 Y=0 Z=0
 Joint=J XorR={l} Y=0 Z=0
TABLE: "JOINT RESTRAINT ASSIGNMENTS"
 Joint=I U1=Yes U2=Yes U3=Yes R1=Yes R2=Yes R3=Yes
 Joint=J U3=Yes R1=Yes R2=Yes
TABLE: "FRAME SECTION PROPERTIES 01 - GENERAL"
 SectionName=BAR Material=MAT Shape=General Area={area} I33={i33} I22={i22} TorsConst={tors}
TABLE: "CONNECTIVITY - FRAME"
 Frame=F JointI=I JointJ=J MeshSegments=1
TABLE: "FRAME SECTION ASSIGNMENTS"
 Frame=F AnalSect=BAR
TABLE: "LOAD PATTERN DEFINITIONS"
 LoadPat=P DesignType=Other SelfWtMult=0
 LoadPat=AX DesignType=Other SelfWtMult=0
TABLE: "JOINT LOADS - FORCE"
 Joint=J LoadPat=P F2={H0}
 Joint=J LoadPat=AX F1=-{N1} F2={H0}
TABLE: "RUST HINGE PROPERTIES"
 Name=H DOF=M3 Symmetric=Yes BeyondE=Extrapolated
TABLE: "RUST HINGE PROPERTIES - CURVE"
 Name=H Point=B Force={cap} Deform=0
 Name=H Point=C Force={cap} Deform=0.1
 Name=H Point=D Force={cap} Deform=0.2
 Name=H Point=E Force={cap} Deform=1
TABLE: "RUST FRAME HINGE ASSIGNMENTS"
 Frame=F HingeNum=1 Prop=H RelDist=1
TABLE: "LOAD CASE DEFINITIONS"
 Case=PUSH Type=NonStatic RunCase=Yes GeomNonLin=PDelta{tol}
 Case=FOLLOW Type=NonStatic GeomNonLin=PDelta{tol} InitialCond=PUSH{control}
TABLE: "CASE - STATIC 1 - LOAD ASSIGNMENTS"
 Case=FOLLOW LoadType="Load pattern" LoadName=AX LoadSF={factor:?}
 Case=PUSH LoadType="Load pattern" LoadName=P LoadSF=1
END TABLE DATA
"#,
        e = 2.0e8 / (mm * mm),
        g = 76_923_077.0 / (mm * mm),
        l = L * mm,
        area = 0.0015 * mm * mm,
        i33 = 7.8125e-6 * mm.powi(4),
        i22 = 4.5e-9 * mm.powi(4),
        tors = 1.77e-8 * mm.powi(4),
        cap = capacity * mm,
    );
    let mut m = parse_s2k(&text).expect("S2K parses");
    analyze_model(&mut m).expect("analysis runs");
    m
}

/// Load factor of `FOLLOW` and tip deflection in metres.
fn answer(m: &SapModel, mm: f64) -> (f64, f64) {
    let r = &m.case_results["FOLLOW"];
    (
        r.displacement_control_factor.unwrap_or(1.0),
        r.joint_displacements["J"][1] / mm,
    )
}

fn close(actual: f64, expected: f64, rel: f64, what: &str) {
    assert!(
        (actual - expected).abs() <= rel * expected.abs(),
        "{what}: {actual:e} vs expected {expected:e} (relative {:e}, allowed {rel:e})",
        (actual - expected) / expected
    );
}

/// The convergence test of the axial forces is in force units, so the same
/// structure in metres and in millimetres gives one load factor at the
/// default tolerance (1e-6). Before the repair the test was scaled by the
/// hinge's moment capacity (20 kN m, or 20000 kN mm): the factors were
/// 8.345645 and 8.018115.
#[test]
fn the_same_cantilever_in_metres_and_millimetres_gives_one_load_factor() {
    let expected = closed_form_factor();
    // The closed form: about 8.346152.
    close(expected, 8.346152, 1e-6, "closed-form factor");
    let (f_m, u_m) = answer(&model(1.0, 20.0, None, Follow::Monitored), 1.0);
    let (f_mm, u_mm) = answer(&model(1000.0, 20.0, None, Follow::Monitored), 1000.0);
    let total = H0 / tip_stiffness(0.0) + FURTHER;
    close(u_m, total, 1e-12, "tip deflection, metres");
    close(u_mm, total, 1e-12, "tip deflection, millimetres");
    // One structure, one factor: the units of length change nothing beyond
    // rounding.
    close(f_mm, f_m, 1e-9, "factor in millimetres against metres");
    // Within the tolerance of the case of the closed form.
    close(f_m, expected, 1e-6, "factor against the closed form, metres");
    close(f_mm, expected, 1e-6, "factor against the closed form, millimetres");
}

/// A factor reported under displacement control, applied under load control,
/// gives back the deflection that was asked for: the accepted state is in
/// equilibrium. Tolerance: the convergence tolerance of the case (1e-6
/// relative; the measured residual is about 1e-7). Before the repair the
/// millimetre model's factor gave 4.743 mm for the 4.95 mm asked.
#[test]
fn the_reported_factor_replayed_under_load_control_recovers_the_deflection() {
    let total = H0 / tip_stiffness(0.0) + FURTHER;
    for mm in [1.0, 1000.0] {
        let (factor, _) = answer(&model(mm, 20.0, None, Follow::Monitored), mm);
        let (_, replayed) = answer(&model(mm, 20.0, None, Follow::Load(factor)), mm);
        close(replayed, total, 1e-6, &format!("replayed tip deflection, mm = {mm}"));
    }
}

/// The hinge never yields, so its moment capacity is not part of the answer:
/// 20, 1e4 and 1e14 kN m give one factor. (With the old scale a larger
/// capacity loosened the test until no iteration was made at all.)
#[test]
fn the_moment_capacity_of_an_unyielded_hinge_does_not_change_the_factor() {
    let (reference, _) = answer(&model(1.0, 20.0, None, Follow::Monitored), 1.0);
    for capacity in [1.0e4, 1.0e14] {
        let (factor, _) = answer(&model(1.0, capacity, None, Follow::Monitored), 1.0);
        close(factor, reference, 1e-12, &format!("factor with capacity {capacity:e}"));
    }
}

/// With a tight tolerance both unit systems meet the closed form closely
/// (1e-9 relative), and the replay under load control is exact to 1e-9.
#[test]
fn a_tight_tolerance_meets_the_closed_form_in_both_unit_systems() {
    let expected = closed_form_factor();
    let total = H0 / tip_stiffness(0.0) + FURTHER;
    for mm in [1.0, 1000.0] {
        let (factor, _) = answer(&model(mm, 20.0, Some(1e-12), Follow::Monitored), mm);
        close(factor, expected, 1e-9, &format!("tight factor, mm = {mm}"));
        let (_, replayed) = answer(&model(mm, 20.0, Some(1e-12), Follow::Load(factor)), mm);
        close(replayed, total, 1e-9, &format!("tight replay, mm = {mm}"));
    }
}

// ---------------------------------------------------------------------------
// A push that moves the monitored joint only through the geometric stiffness
// ---------------------------------------------------------------------------
//
// A cantilever column (L = 3 m, EI = 2e4 kN m^2) gets a lateral preload of
// H kN at its tip (`PUSH`, P-Delta). `FOLLOW` continues with a compression of
// 100 kN per unit load factor and nothing else, under displacement control of
// the tip. The compression has no direct effect on the tip deflection; the tip
// moves because the geometric stiffness softens the column:
//   u(f) = H / k(100 f),  k(N) the tip stiffness written at the top of the file
// (with this file's `tip_stiffness` scaled to EI = 2e4). Before the repair the
// case was refused: "load pattern does not move the monitored displacement".

const EI2: f64 = 2.0e8 * 1.0e-4;

fn tip_stiffness2(n: f64) -> f64 {
    let kuu = 12.0 * EI2 / L.powi(3) - 1.2 * n / L;
    let kur = -6.0 * EI2 / (L * L) + 0.1 * n;
    let krr = 4.0 * EI2 / L - 2.0 * n * L / 15.0;
    kuu - kur * kur / krr
}

/// `follow`: `Ok(further)` pushes the tip by `further` m under displacement
/// control, `Err(factor)` applies the factor under load control.
fn column_text(lateral: f64, follow: Result<f64, f64>) -> String {
    let (control, factor) = match follow {
        Ok(further) => (
            format!(" RustDisplTarget={further:e} RustDisplJoint=J RustDisplDOF=U2 RustDisplConjugate=No"),
            1.0,
        ),
        Err(f) => (String::new(), f),
    };
    format!(
        r#"TABLE: "PROGRAM CONTROL"
 CurrUnits="kN, m, C"
TABLE: "MATERIAL PROPERTIES 02 - BASIC MECHANICAL PROPERTIES"
 Material=MAT E1=200000000 G12=76923077 U12=0.3 UnitWeight=0 UnitMass=0
TABLE: "JOINT COORDINATES"
 Joint=I XorR=0 Y=0 Z=0
 Joint=J XorR=3 Y=0 Z=0
TABLE: "JOINT RESTRAINT ASSIGNMENTS"
 Joint=I U1=Yes U2=Yes U3=Yes R1=Yes R2=Yes R3=Yes
 Joint=J U3=Yes R1=Yes R2=Yes
TABLE: "FRAME SECTION PROPERTIES 01 - GENERAL"
 SectionName=BAR Material=MAT Shape=General Area=0.01 I33=0.0001 I22=0.0001 TorsConst=0.0001
TABLE: "CONNECTIVITY - FRAME"
 Frame=F JointI=I JointJ=J MeshSegments=1
TABLE: "FRAME SECTION ASSIGNMENTS"
 Frame=F AnalSect=BAR
TABLE: "LOAD PATTERN DEFINITIONS"
 LoadPat=P DesignType=Other SelfWtMult=0
 LoadPat=AX DesignType=Other SelfWtMult=0
TABLE: "JOINT LOADS - FORCE"
 Joint=J LoadPat=P F2={lateral}
 Joint=J LoadPat=AX F1=-100
TABLE: "RUST HINGE PROPERTIES"
 Name=H DOF=P Symmetric=Yes BeyondE=Extrapolated
TABLE: "RUST HINGE PROPERTIES - CURVE"
 Name=H Point=B Force=10000 Deform=0
 Name=H Point=C Force=10000 Deform=0.1
 Name=H Point=D Force=10000 Deform=0.2
 Name=H Point=E Force=10000 Deform=1
TABLE: "RUST FRAME HINGE ASSIGNMENTS"
 Frame=F HingeNum=1 Prop=H RelDist=1
TABLE: "LOAD CASE DEFINITIONS"
 Case=PUSH Type=NonStatic RunCase=Yes GeomNonLin=PDelta
 Case=FOLLOW Type=NonStatic GeomNonLin=PDelta InitialCond=PUSH{control}
TABLE: "CASE - STATIC 1 - LOAD ASSIGNMENTS"
 Case=FOLLOW LoadType="Load pattern" LoadName=AX LoadSF={factor:?}
 Case=PUSH LoadType="Load pattern" LoadName=P LoadSF=1
END TABLE DATA
"#
    )
}

/// The factor at which the tip has moved `further` beyond the preload.
fn column_factor(lateral: f64, further: f64) -> f64 {
    let target = lateral / tip_stiffness2(0.0) + further;
    let (mut lo, mut hi) = (0.0, 5.0);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if lateral / tip_stiffness2(100.0 * mid) < target {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

/// The compression moves the tip only through the geometric stiffness, and
/// displacement control follows it: the factor meets the closed form within
/// the tolerance of the case, and the factor replayed under load control
/// gives the deflection asked for. Two increments: a very small one (1e-8 m,
/// 2e-5 of the preload deflection; the review's control) and one near a full
/// unit of the load (8e-6 m).
#[test]
fn a_compression_after_a_lateral_preload_is_followed_by_displacement_control() {
    let preload = 1.0 / tip_stiffness2(0.0);
    close(preload, 0.00045, 1e-12, "preload deflection H L^3 / 3 EI");
    for further in [1.0e-8, 8.0e-6] {
        let mut m = parse_s2k(&column_text(1.0, Ok(further))).expect("S2K parses");
        analyze_model(&mut m).expect("the case moves the tip and must run");
        let follow = &m.case_results["FOLLOW"];
        let factor = follow.displacement_control_factor.expect("a factor is reported");
        let tip = follow.joint_displacements["J"][1];
        // The increment asked for, to 1e-9 of itself or the rounding of the
        // absolute deflection it is part of.
        assert!(
            (tip - preload - further).abs() <= 1e-9 * further + 8.0 * f64::EPSILON * preload,
            "increment {further:e}: tip {tip:e}, preload {preload:e}"
        );
        close(factor, column_factor(1.0, further), 1e-6, &format!("factor for {further:e}"));
        // Replayed under load control.
        let mut replay = parse_s2k(&column_text(1.0, Err(factor))).expect("S2K parses");
        analyze_model(&mut replay).expect("load control runs");
        let replayed = replay.case_results["FOLLOW"].joint_displacements["J"][1];
        assert!(
            (replayed - tip).abs() <= 1e-6 * further + 8.0 * f64::EPSILON * preload,
            "increment {further:e}: replayed {replayed:e} against {tip:e}"
        );
    }
}

/// The negative control: without the lateral preload the column is straight,
/// the compression moves the tip nowhere, and displacement control of the tip
/// is refused, naming the case, with no results.
#[test]
fn a_compression_of_a_straight_column_does_not_move_the_tip_and_is_refused() {
    let mut m = parse_s2k(&column_text(0.0, Ok(1.0e-8))).expect("S2K parses");
    let e = format!("{:?}", analyze_model(&mut m).unwrap_err());
    assert!(e.contains("FOLLOW"), "{e}");
    assert!(e.contains("does not move the monitored displacement"), "{e}");
    assert!(m.case_results.is_empty(), "no results are kept");
}

#[test]
fn an_unrelated_preload_does_not_change_the_column_equilibrium() {
    let text = include_str!("fixtures/pdelta/disconnected-preload.s2k");
    let mut model = parse_s2k(text).unwrap();
    analyze_model(&mut model).unwrap();
    let (factor, tip) = answer(&model, 1.0);
    assert_eq!(model.case_results["FOLLOW"].status, CaseStatus::Finished);
    close(factor, closed_form_factor(), 1e-6, "column factor");
    close(tip, tip_at(factor), 1e-6, "equilibrium at the reported factor");
}

#[test]
fn scaling_the_axial_pattern_preserves_the_coupled_displacement_solution() {
    let further = 8e-6;
    for compression in [1e7, 100.0, 0.01, 1e-10] {
        let text = column_text(1.0, Ok(further)).replace("F1=-100", &format!("F1=-{compression:e}"));
        let mut model = parse_s2k(&text).unwrap();
        analyze_model(&mut model).unwrap();
        let (factor, tip) = answer(&model, 1.0);
        assert_eq!(model.case_results["FOLLOW"].status, CaseStatus::Finished);
        close(factor * compression, 100.0 * column_factor(1.0, further), 1e-6, "compression");
        close(tip - 1.0 / tip_stiffness2(0.0), further, 1e-9, "tip increment");
    }
}

#[test]
fn a_yield_event_halfway_to_the_conjugate_target_does_not_end_the_case() {
    let fixture = include_str!("fixtures/pdelta/conjugate-axial.s2k");
    for force in [1.0, 1e-14] {
        let text = fixture.replace("F1=1e-14", &format!("F1={force:e}"));
        let mut model = parse_s2k(&text).unwrap();
        analyze_model(&mut model).unwrap();
        let result = &model.case_results["FOLLOW"];
        assert_eq!(result.status, CaseStatus::Finished);
        close(result.joint_displacements["J"][0], 1.0, 1e-12, "final displacement");
        // Elastic bar and hardening hinge in series: u = F/0.001 + (F-0.0005)/0.001.
        close(result.displacement_control_factor.unwrap() * force, 0.00075, 1e-12, "final force");
    }
}

#[test]
fn an_independently_loaded_flexible_member_does_not_mask_the_monitored_rate() {
    let further = 8e-6;
    let fixture = include_str!("fixtures/pdelta/disconnected-follow.s2k");
    let bar_only = fixture.replace(" Frame=F HingeNum=1 Prop=H RelDist=1\n", "");
    for text in [fixture, &bar_only] {
        let mut model = parse_s2k(text).unwrap();
        analyze_model(&mut model).unwrap();
        let (factor, tip) = answer(&model, 1.0);
        assert_eq!(model.case_results["FOLLOW"].status, CaseStatus::Finished);
        close(factor * 0.01, 100.0 * column_factor(1.0, further), 1e-6, "compression");
        close(tip - 1.0 / tip_stiffness2(0.0), further, 1e-9, "tip increment");
    }
}

#[test]
fn a_stiff_or_fixed_shared_pier_does_not_mask_a_coupled_tip_rate() {
    let fixture = include_str!("fixtures/pdelta/shared-pier.s2k");
    let spring = fixture.replace("Joint=I U1=Yes U2=Yes U3=Yes R1=Yes R2=Yes R3=Yes", "Joint=I U3=Yes R1=Yes R2=Yes")
        .replace("END TABLE DATA", "TABLE: \"JOINT SPRING ASSIGNMENTS 1 - UNCOUPLED\"\n Joint=I U1=1e12 U2=1e12 R3=1e12\nEND TABLE DATA");
    for text in [fixture, &spring] {
        let mut model = parse_s2k(text).unwrap();
        analyze_model(&mut model).unwrap();
        let (factor, tip) = answer(&model, 1.0);
        assert_eq!(model.case_results["FOLLOW"].status, CaseStatus::Finished);
        close(factor * 0.01, 100.0 * column_factor(1.0, 8e-6), 1e-6, "compression");
        close(tip - model.case_results["PUSH"].joint_displacements["J"][1],
            8e-6, 1e-9, "tip increment");
    }
}
