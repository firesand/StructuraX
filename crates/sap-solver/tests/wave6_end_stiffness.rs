//! Wave 6, stage S3: the stiffness at the end of a nonlinear static case.
//!
//! A Modal, linear static or multi-step static case that continues from a
//! nonlinear static case takes a geometric stiffness from that case's axial
//! forces only if that case considered geometric nonlinearity (v19 Analysis
//! Reference p. 411). References are closed forms written out below. Units kN,
//! m, t, s. Tolerance 1e-9 relative unless noted.
//!
//! The column is a cantilever along global X (the member runs from the fixed
//! joint "1" at the origin to the free joint "2" at x = H) bending in the X-Z
//! plane (active degrees of freedom U1, U3, R2; local axis 3, so I33 is the
//! stiffness and M3 the hinge moment), with the translational mass at the free
//! end. The axial load of the gravity case is a joint force at the free end
//! toward the base: compression.
use sap_model::hinge::*;
use sap_model::*;
use sap_solver::{analyze_model, SolverError};

/// Column height (m), flexural rigidity EI (kN m^2), end mass (t), axial load (kN).
const H: f64 = 3.0;
const EI: f64 = 2.0e4;
const MASS: f64 = 10.0;
const P: f64 = 2000.0;

fn close(a: f64, b: f64, rel: f64, what: &str) {
    let scale = a.abs().max(b.abs()).max(1e-300);
    assert!(
        (a - b).abs() <= rel * scale,
        "{what}: got {a}, expected {b} (relative tolerance {rel})"
    );
}

fn pattern(m: &mut SapModel, name: &str) {
    m.load_patterns.insert(
        name.into(),
        LoadPattern {
            name: name.into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
}

fn joint_force(m: &mut SapModel, pattern: &str, force: [f64; 6]) {
    m.points["2"].loads_force.push(PointLoadForce {
        load_pat: pattern.into(),
        csys: "GLOBAL".into(),
        force,
    });
}

/// The column without hinges and without cases. E = 2e8, I = 1e-4 give EI = 2e4;
/// A = 1 makes it axially stiff (EA = 2e8); no self weight, no self mass.
fn column() -> SapModel {
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    m.active_dofs = [true, false, true, false, true, false];
    let mut material = Material::default_steel_kip_in();
    material.name = "MAT".into();
    material.e = 2.0e8;
    material.g = 7.7e7;
    material.u = 0.3;
    material.unit_weight = 0.0;
    material.unit_mass = 0.0;
    m.materials.insert("MAT".into(), material);
    m.frame_sections.insert(
        "SEC".into(),
        FrameSection {
            name: "SEC".into(),
            material: "MAT".into(),
            prop_type: FramePropType::General,
            area: 1.0,
            i33: EI / 2.0e8,
            i22: EI / 2.0e8,
            torsional_j: 2.0 * EI / 2.0e8,
            as2: 0.0,
            as3: 0.0,
            modifiers: [1.0; 8],
            ..FrameSection::default()
        },
    );
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", H, 0.0, 0.0);
    m.points["1"].restraint = [true; 6];
    // Translational mass in the two active translations; none in rotation.
    m.points["2"].mass = [MASS, 0.0, MASS, 0.0, 0.0, 0.0];
    m.add_frame("1", "1", "2", "SEC");
    pattern(&mut m, "GRAV");
    pattern(&mut m, "LAT");
    joint_force(&mut m, "GRAV", [-P, 0.0, 0.0, 0.0, 0.0, 0.0]);
    joint_force(&mut m, "LAT", [0.0, 0.0, 10.0, 0.0, 0.0, 0.0]);
    m
}

/// `GRAV`, a nonlinear static case under the 2000 kN axial load.
fn gravity(m: &mut SapModel, nl_geom: NLGeomType) {
    let mut c = LoadCase::linear_static("GRAV", "GRAV", 1.0);
    c.case_type = LoadCaseType::NonlinearStatic;
    c.nl_geom = nl_geom;
    m.load_cases.insert("GRAV".into(), c);
}

/// `MODAL` and `LAT` (10 kN across the free end) on the end stiffness of `GRAV`.
fn continuations(m: &mut SapModel) {
    let mut modal = LoadCase::modal_eigen("MODAL", 2);
    modal.initial_case = Some("GRAV".into());
    m.load_cases.insert("MODAL".into(), modal);
    let mut lat = LoadCase::linear_static("LAT", "LAT", 1.0);
    lat.initial_case = Some("GRAV".into());
    m.load_cases.insert("LAT".into(), lat);
}

/// Squared circular frequency of the first mode (rad^2/s^2) and the tip
/// deflection of `LAT` (m).
fn answers(m: &SapModel) -> (f64, f64) {
    let modes = &m.case_results["MODAL"].modes;
    (
        modes[0].eigenvalue,
        m.case_results["LAT"].joint_displacements["2"][2],
    )
}

/// Closed forms of the cantilever column with the translational mass at the
/// free end (one degree of freedom, w):
/// * no geometric stiffness: k = 3EI/H^3 = 3 * 2e4 / 27 = 2222.2222 kN/m,
///   omega^2 = k/m = 2222.2222 / 10 = 222.2222 rad^2/s^2, and a lateral 10 kN
///   deflects the tip by 10/k = 0.0045 m;
/// * with the cubic-shape geometric stiffness of an axial compression P (the
///   element stiffness 12EI/H^3 - 1.2P/H, -6EI/H^2 + 0.1P, 4EI/H - 2PH/15,
///   i.e. 8088.889, -13133.333, 25866.667 for P = 2000), condensed on the
///   free rotation: k = K77 - K7,11^2 / K11,11 = 1420.6758 kN/m,
///   omega^2 = k/m = 142.06758 rad^2/s^2 (the same value as a hinge-free
///   model analyzed with P-Delta).
fn k_plain() -> f64 {
    3.0 * EI / H.powi(3)
}

fn k_with_geometric_stiffness() -> f64 {
    let kww = 12.0 * EI / H.powi(3) - 1.2 * P / H;
    let kwt = -6.0 * EI / (H * H) + 0.1 * P;
    let ktt = 4.0 * EI / H - 2.0 * P * H / 15.0;
    kww - kwt * kwt / ktt
}

#[test]
fn a_continuation_of_a_case_without_geometric_nonlinearity_has_no_geometric_stiffness() {
    // Test 6 of the plan (defect D70) and the linear-static half of test 5, no hinges.
    let mut m = column();
    gravity(&mut m, NLGeomType::None);
    continuations(&mut m);
    analyze_model(&mut m).unwrap();
    // The case carries the axial load, so the old code added a geometric stiffness.
    let axial = m.case_results["GRAV"].frame_forces["1"][0].p;
    close(axial.abs(), P, 1e-9, "axial force of GRAV");
    let (omega2, tip) = answers(&m);
    close(omega2, 222.2222222222222, 1e-9, "omega^2 without geometric stiffness");
    close(omega2, k_plain() / MASS, 1e-12, "3EI/(H^3 m)");
    close(tip, 10.0 / k_plain(), 1e-9, "LAT tip deflection without it");
    close(tip, 0.0045, 1e-9, "LAT tip deflection, closed form");
}

#[test]
fn a_continuation_of_a_p_delta_case_keeps_the_geometric_stiffness() {
    let mut m = column();
    gravity(&mut m, NLGeomType::PDelta);
    continuations(&mut m);
    analyze_model(&mut m).unwrap();
    let (omega2, tip) = answers(&m);
    close(k_with_geometric_stiffness(), 1420.6758304696456, 1e-9, "k");
    close(omega2, 142.06758304696456, 1e-9, "omega^2 with P-Delta");
    close(tip, 10.0 / k_with_geometric_stiffness(), 1e-9, "LAT tip deflection with P-Delta");
}

#[test]
fn other_kinds_of_initial_case_keep_their_geometric_stiffness() {
    // A linear static initial case is left as it was (open question in the
    // report): its axial forces still give the cubic geometric stiffness.
    let mut m = column();
    m.load_cases.insert(
        "GRAV".into(),
        LoadCase::linear_static("GRAV", "GRAV", 1.0),
    );
    continuations(&mut m);
    analyze_model(&mut m).unwrap();
    let (omega2, _) = answers(&m);
    close(omega2, 142.06758304696456, 1e-9, "omega^2 after a linear static case");
}

// ---------------------------------------------------------------------------
// G48 (a): the stiffness at the end of a nonlinear static case of a model with
// hinges, when no hinge has yielded. Hinges are rigid (v19 help, "Hinge
// Properties": "For all other types of analysis, the hinges are rigid"), so
// the answers are those of the hinge-free column.
// ---------------------------------------------------------------------------

/// A symmetric moment hinge (M3) whose curve is B (mp, 0), C (1.2 mp, 0.04 rad),
/// D (0.3 mp, 0.04), E (0.3 mp, 0.4), at the fixed end of the column.
fn with_base_hinge(m: &mut SapModel, mp: f64) {
    let prop = HingeProp::symmetric(
        "HM",
        HingeDof::M3,
        [
            HingePoint::new(mp, 0.0),
            HingePoint::new(1.2 * mp, 0.04),
            HingePoint::new(0.3 * mp, 0.04),
            HingePoint::new(0.3 * mp, 0.4),
        ],
    );
    m.hinge_props.insert("HM".into(), prop);
    m.frames["1"].hinges = vec![FrameHinge {
        prop: "HM".into(),
        rel_dist: 0.0,
    }];
}

fn unsupported(e: &SolverError) -> &str {
    match e {
        SolverError::UnsupportedInput(msg) => msg,
        other => panic!("expected UnsupportedInput, got {other:?}"),
    }
}

#[test]
fn a_hinge_that_never_yields_leaves_the_elastic_stiffness_at_the_end_of_the_case() {
    // Test 1 of the plan and the first half of test 5. Mp = 1e6 kN m is never
    // reached: GRAV carries the axial load only (no moment at the base). Without
    // geometric nonlinearity the geometric stiffness is left out (D70), so the
    // frequencies are those of the cantilever: omega^2 = 3EI/(H^3 m) = 222.2222
    // rad^2/s^2, and the lateral 10 kN deflects the tip by 10/(3EI/H^3) = 0.0045 m.
    // Before wave 6 the model was refused ("tangent stiffness of the hinged structure").
    let mut m = column();
    with_base_hinge(&mut m, 1.0e6);
    gravity(&mut m, NLGeomType::None);
    continuations(&mut m);
    analyze_model(&mut m).unwrap();
    let hinges = &m.case_results["GRAV"].hinge_results;
    assert_eq!(hinges.len(), 1);
    assert_eq!(hinges[0].direction, 0);
    assert_eq!(hinges[0].plastic_deformation, 0.0);
    let (omega2, tip) = answers(&m);
    close(omega2, 222.2222222222222, 1e-9, "omega^2 with a hinge that never yields");
    close(tip, 10.0 / k_plain(), 1e-9, "LAT tip deflection with the hinge");
    // The same numbers as the model without the hinge.
    let mut plain = column();
    gravity(&mut plain, NLGeomType::None);
    continuations(&mut plain);
    analyze_model(&mut plain).unwrap();
    let (omega2_plain, tip_plain) = answers(&plain);
    close(omega2, omega2_plain, 1e-12, "omega^2 equals the hinge-free one");
    close(tip, tip_plain, 1e-12, "tip deflection equals the hinge-free one");
}

/// `GRAV` pushed to a tip deflection of `target` m by displacement control
/// under the lateral pattern (hinge yields at M = Mp: F = Mp/H, u = F H^3/3EI).
fn pushed_gravity(m: &mut SapModel, target: f64) {
    let mut c = LoadCase::linear_static("GRAV", "LAT", 1.0);
    c.case_type = LoadCaseType::NonlinearStatic;
    c.displacement_control = Some(DisplacementControl {
        target,
        monitored: Some(("2".into(), 2)),
        conjugate: false,
    });
    m.load_cases.insert("GRAV".into(), c);
}

#[test]
fn a_hinge_on_its_backbone_at_the_end_of_the_case_is_refused_naming_it() {
    // Mp = 100 kN m: the hinge yields at F = Mp/H = 33.33 kN, u = 0.015 m, and the
    // push to 0.05 m leaves it on its backbone (B to C).
    let mut m = column();
    with_base_hinge(&mut m, 100.0);
    pushed_gravity(&mut m, 0.05);
    continuations(&mut m);
    let e = analyze_model(&mut m).unwrap_err();
    let msg = unsupported(&e);
    assert!(msg.contains("hinge 1H1"), "{msg}");
    assert!(msg.contains("frame '1'"), "{msg}");
    assert!(msg.contains("case 'MODAL'") || msg.contains("case 'LAT'"), "{msg}");
    assert!(msg.contains("nonlinear static case 'GRAV'"), "{msg}");
    assert!(msg.contains("yielded and is on its backbone"), "{msg}");
    assert!(!msg.contains("unloaded"), "{msg}");
    assert!(m.case_results.is_empty(), "no partial results are kept");
}

#[test]
fn a_hinge_that_yielded_and_unloaded_is_refused_with_its_own_wording() {
    // L1 loads the lateral pattern (10 kN per unit) to 36 kN (M = 108 kN m, past Mp = 100, on B to C);
    // L2 takes 12 kN off (total 24 kN, M = 72 kN m): the hinge keeps its plastic
    // rotation and sits below its backbone.
    let mut m = column();
    with_base_hinge(&mut m, 100.0);
    for (name, sf, initial) in [("L1", 3.6, None), ("L2", -1.2, Some("L1"))] {
        let mut c = LoadCase::linear_static(name, "LAT", sf);
        c.case_type = LoadCaseType::NonlinearStatic;
        c.initial_case = initial.map(str::to_string);
        m.load_cases.insert(name.into(), c);
    }
    let mut modal = LoadCase::modal_eigen("MODAL", 2);
    modal.initial_case = Some("L2".into());
    m.load_cases.insert("MODAL".into(), modal);
    let e = analyze_model(&mut m).unwrap_err();
    let msg = unsupported(&e);
    assert!(msg.contains("hinge 1H1"), "{msg}");
    assert!(msg.contains("nonlinear static case 'L2'"), "{msg}");
    assert!(msg.contains("yielded and then unloaded"), "{msg}");
    assert!(!msg.contains("on its backbone"), "{msg}");
}

#[test]
fn buckling_on_a_hinged_nonlinear_case_stays_refused() {
    let mut m = column();
    with_base_hinge(&mut m, 1.0e6);
    gravity(&mut m, NLGeomType::None);
    let mut buckling = LoadCase::linear_static("BUCK", "GRAV", 1.0);
    buckling.case_type = LoadCaseType::Buckling;
    buckling.initial_case = Some("GRAV".into());
    m.load_cases.insert("BUCK".into(), buckling);
    let e = analyze_model(&mut m).unwrap_err();
    let msg = unsupported(&e);
    assert!(msg.contains("case 'BUCK'"), "{msg}");
    assert!(msg.contains("GRAV"), "{msg}");
    assert!(msg.contains("tangent stiffness of the hinged structure"), "{msg}");
}

// ---------------------------------------------------------------------------
// Review of 2026-11-18, finding P1-1: the guard after automatic subdivision
// ---------------------------------------------------------------------------
//
// The cases run on the meshed model, where the pieces of a subdivided frame
// carry no hinge assignment. The guard used to ask the model for hinge
// assignments and so let a case continue, with the elastic stiffness, from a
// yielded hinge of a frame cut into two or more elements. It now decides from
// the hinge state the initial case recorded.

/// One continuation of `initial` at a time, so that the refusal is the one of
/// that case type: "LAT" linear static, "MULTI" multi-step static, "MODAL".
fn one_continuation(m: &mut SapModel, kind: &str, initial: &str) {
    match kind {
        "MODAL" => {
            let mut modal = LoadCase::modal_eigen("MODAL", 2);
            modal.initial_case = Some(initial.into());
            m.load_cases.insert("MODAL".into(), modal);
        }
        _ => {
            let mut c = LoadCase::linear_static(kind, "LAT", 1.0);
            if kind == "MULTI" {
                c.case_type = LoadCaseType::LinearStaticMultiStep;
            }
            c.initial_case = Some(initial.into());
            m.load_cases.insert(kind.into(), c);
        }
    }
}

#[test]
fn a_yielded_hinge_is_refused_however_the_frame_is_subdivided() {
    // The model of `a_hinge_on_its_backbone_at_the_end_of_the_case_is_refused_naming_it`
    // (Mp = 100 kN m, pushed to 0.05 m: the hinge is on B to C), with the frame
    // as one element and cut automatically into 2 and 4. Every continuation is
    // refused and names the hinge by its object name, 1H1.
    for segments in [0usize, 1, 2, 4] {
        for kind in ["LAT", "MODAL", "MULTI"] {
            let mut m = column();
            with_base_hinge(&mut m, 100.0);
            m.frames["1"].auto_mesh_segments = segments;
            pushed_gravity(&mut m, 0.05);
            one_continuation(&mut m, kind, "GRAV");
            let e = analyze_model(&mut m).unwrap_err();
            let msg = unsupported(&e);
            let what = format!("{segments} segments, {kind}: {msg}");
            assert!(m.case_results.is_empty(), "{what}");
            if kind == "MULTI" && !msg.contains("hinge 1H1") {
                // A multi-step static case is not analyzed at all today; that
                // refusal comes first and is as closed as the guard.
                assert!(msg.contains("LinearStaticMultiStep"), "{what}");
                continue;
            }
            assert!(msg.contains("hinge 1H1"), "{what}");
            assert!(msg.contains("frame '1'"), "{what}");
            assert!(msg.contains(&format!("case '{kind}'")), "{what}");
            assert!(msg.contains("nonlinear static case 'GRAV'"), "{what}");
            assert!(msg.contains("yielded and is on its backbone"), "{what}");
        }
    }
}

#[test]
fn a_hinge_that_unloaded_keeps_its_own_wording_in_a_subdivided_frame() {
    // The two-case chain of `a_hinge_that_yielded_and_unloaded_is_refused_with_its_own_wording`
    // on a frame cut into 2 and 4 elements.
    for segments in [2usize, 4] {
        let mut m = column();
        with_base_hinge(&mut m, 100.0);
        m.frames["1"].auto_mesh_segments = segments;
        for (name, sf, initial) in [("L1", 3.6, None), ("L2", -1.2, Some("L1"))] {
            let mut c = LoadCase::linear_static(name, "LAT", sf);
            c.case_type = LoadCaseType::NonlinearStatic;
            c.initial_case = initial.map(str::to_string);
            m.load_cases.insert(name.into(), c);
        }
        one_continuation(&mut m, "LAT2", "L2");
        let e = analyze_model(&mut m).unwrap_err();
        let msg = unsupported(&e);
        assert!(msg.contains("hinge 1H1"), "{segments}: {msg}");
        assert!(msg.contains("yielded and then unloaded"), "{segments}: {msg}");
        assert!(!msg.contains("on its backbone"), "{segments}: {msg}");
    }
}

#[test]
fn a_subdivided_frame_whose_hinge_stays_elastic_may_be_continued() {
    // The control: Mp = 1e6 kN m is never reached, the frame is cut into 2 and
    // 4 elements. The cubic elements are exact for end loads, and the only
    // mass is at the free end, so the closed forms of the unsubdivided column
    // hold: omega^2 = 3EI/(H^3 m) = 222.2222 rad^2/s^2 and a lateral 10 kN
    // deflects the tip by 10/(3EI/H^3) = 0.0045 m.
    for segments in [2usize, 4] {
        let mut m = column();
        with_base_hinge(&mut m, 1.0e6);
        m.frames["1"].auto_mesh_segments = segments;
        gravity(&mut m, NLGeomType::None);
        continuations(&mut m);
        analyze_model(&mut m).unwrap();
        let hinges = &m.case_results["GRAV"].hinge_results;
        assert_eq!(hinges.len(), 1, "{segments}");
        assert_eq!((hinges[0].direction, hinges[0].plastic_deformation), (0, 0.0));
        assert_eq!(hinges[0].name, "1H1");
        let (omega2, tip) = answers(&m);
        close(omega2, 222.2222222222222, 1e-9, "omega^2, subdivided, elastic hinge");
        close(tip, 10.0 / k_plain(), 1e-9, "LAT tip deflection, subdivided, elastic hinge");
    }
}

#[test]
fn a_nonlinear_history_may_not_continue_from_a_yielded_hinge() {
    // Found next to review finding P1-1: a NonlinearHistory case continued
    // from the yielded hinge and integrated with the elastic stiffness, with
    // the frame as one element and as two. A history treats hinges as rigid
    // (`frame_hinges.rs`: `nonlinear_history_runs_and_ignores_hinges`), which
    // is the state at the end of the initial case only when no hinge yielded.
    let history = |m: &mut SapModel, initial: &str| {
        m.functions.insert(
            "RAMP".into(),
            DynamicFunction {
                name: "RAMP".into(),
                is_time_history: true,
                damping_ratio: 0.0,
                points: vec![(0.0, 0.0), (1.0, 1.0)],
            },
        );
        let mut c = LoadCase::linear_static("HIST", "LAT", 1.0);
        c.loads.clear();
        c.case_type = LoadCaseType::NonlinearHistory;
        c.modal_damping = 0.0;
        c.history_loads = vec![("RAMP".into(), "LAT".into(), 1.0)];
        c.time_step_dt = 0.01;
        c.num_time_steps = 5;
        c.convergence_tol = 1e-10;
        c.initial_case = Some(initial.into());
        m.load_cases.insert("HIST".into(), c);
    };
    for segments in [0usize, 2] {
        // Yielded (Mp = 100 kN m, pushed to 0.05 m): refused, naming the hinge.
        let mut m = column();
        with_base_hinge(&mut m, 100.0);
        m.frames["1"].auto_mesh_segments = segments;
        pushed_gravity(&mut m, 0.05);
        history(&mut m, "GRAV");
        let e = analyze_model(&mut m).unwrap_err();
        let msg = unsupported(&e);
        assert!(msg.contains("case 'HIST'"), "{segments}: {msg}");
        assert!(msg.contains("hinge 1H1"), "{segments}: {msg}");
        assert!(msg.contains("yielded and is on its backbone"), "{segments}: {msg}");
        assert!(m.case_results.is_empty(), "{segments}");
        // Control: the hinge stays elastic (Mp = 1e6), the history runs.
        let mut m = column();
        with_base_hinge(&mut m, 1.0e6);
        m.frames["1"].auto_mesh_segments = segments;
        pushed_gravity(&mut m, 0.001);
        history(&mut m, "GRAV");
        analyze_model(&mut m).unwrap();
        assert_eq!(m.case_results["GRAV"].hinge_results[0].direction, 0);
        assert!(m.case_results.contains_key("HIST"), "{segments}");
    }
}
