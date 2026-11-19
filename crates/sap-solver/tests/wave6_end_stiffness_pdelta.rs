//! Wave 6, stage S3b: the stiffness at the end of a P-Delta nonlinear static
//! case of a model with frame hinges (G48 (a) together with G14).
//!
//! Hinges that have not yielded are rigid in a Modal or linear static case
//! (v19 help, "Hinge Properties"), so the answers are those of the hinge-free
//! model, with the geometric stiffness of the P-Delta case's axial forces
//! (v19 Analysis Reference p. 411). Units kN, m, t, s unless a test says
//! kip, in, s. Tolerances are written with each reference.
use sap_model::hinge::*;
use sap_model::*;
use sap_solver::{analyze_model, SolverError};

fn close(a: f64, b: f64, rel: f64, what: &str) {
    let scale = a.abs().max(b.abs()).max(1e-300);
    assert!(
        (a - b).abs() <= rel * scale,
        "{what}: got {a}, expected {b} (relative tolerance {rel})"
    );
}

fn unsupported(e: &SolverError) -> &str {
    match e {
        SolverError::UnsupportedInput(msg) => msg,
        other => panic!("expected UnsupportedInput, got {other:?}"),
    }
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

fn joint_force(m: &mut SapModel, joint: &str, pattern: &str, force: [f64; 6]) {
    m.points[joint].loads_force.push(PointLoadForce {
        load_pat: pattern.into(),
        csys: "GLOBAL".into(),
        force,
    });
}

fn material(m: &mut SapModel, e: f64, unit_mass: f64) {
    let mut material = Material::default_steel_kip_in();
    material.name = "MAT".into();
    material.e = e;
    material.g = e / 2.6;
    material.u = 0.3;
    material.unit_weight = 0.0;
    material.unit_mass = unit_mass;
    m.materials.insert("MAT".into(), material);
}

/// A symmetric moment hinge (M3) whose curve is B (mp, 0), C (1.2 mp, 0.04 rad),
/// D (0.3 mp, 0.04), E (0.3 mp, 0.4).
fn m3_hinge(m: &mut SapModel, mp: f64) {
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
}

fn p_delta_case(m: &mut SapModel, name: &str, pattern: &str, initial: Option<&str>) {
    let mut c = LoadCase::linear_static(name, pattern, 1.0);
    c.case_type = LoadCaseType::NonlinearStatic;
    c.nl_geom = NLGeomType::PDelta;
    c.initial_case = initial.map(str::to_string);
    m.load_cases.insert(name.into(), c);
}

// ---------------------------------------------------------------------------
// Tests 1 and 2 (plan stage S3, reference tests 2 and 5): the column with a
// base hinge that never yields, P-Delta GRAV.
// ---------------------------------------------------------------------------

const H: f64 = 3.0;
const EI: f64 = 2.0e4;
const MASS: f64 = 10.0;
const P: f64 = 2000.0;

/// Cantilever along global X (X-Z bending, local axis 3, I33, M3), E = 2e8
/// kN/m2, I = 1e-4 m4 (EI = 2e4 kN m2), A = 1 m2, no shear deformation, mass
/// 10 t translational at the free end, no self mass. `GRAV`: 2000 kN
/// compression at the free end; `LAT`: 10 kN across it.
fn column() -> SapModel {
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    m.active_dofs = [true, false, true, false, true, false];
    material(&mut m, 2.0e8, 0.0);
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
    m.points["2"].mass = [MASS, 0.0, MASS, 0.0, 0.0, 0.0];
    m.add_frame("1", "1", "2", "SEC");
    pattern(&mut m, "GRAV");
    pattern(&mut m, "LAT");
    joint_force(&mut m, "2", "GRAV", [-P, 0.0, 0.0, 0.0, 0.0, 0.0]);
    joint_force(&mut m, "2", "LAT", [0.0, 0.0, 10.0, 0.0, 0.0, 0.0]);
    m
}

fn base_hinge(m: &mut SapModel, mp: f64) {
    m3_hinge(m, mp);
    m.frames["1"].hinges = vec![FrameHinge {
        prop: "HM".into(),
        rel_dist: 0.0,
    }];
}

/// `MODAL` and `LAT` (10 kN across the free end) on the end stiffness of `initial`.
fn continuations(m: &mut SapModel, initial: &str) {
    let mut modal = LoadCase::modal_eigen("MODAL", 2);
    modal.initial_case = Some(initial.into());
    m.load_cases.insert("MODAL".into(), modal);
    let mut lat = LoadCase::linear_static("LAT", "LAT", 1.0);
    lat.initial_case = Some(initial.into());
    m.load_cases.insert("LAT".into(), lat);
}

fn answers(m: &SapModel) -> (f64, f64) {
    (
        m.case_results["MODAL"].modes[0].eigenvalue,
        m.case_results["LAT"].joint_displacements["2"][2],
    )
}

/// Closed form (v19 cubic geometric stiffness of an element under axial
/// compression P, written out in `wave6_end_stiffness.rs` too):
/// K77 = 12EI/H^3 - 1.2P/H = 8088.889 kN/m, K7,11 = -6EI/H^2 + 0.1P =
/// -13133.333 kN, K11,11 = 4EI/H - 2PH/15 = 25866.667 kN m, condensed on the
/// free rotation k = K77 - K7,11^2/K11,11 = 1420.6758 kN/m; one mass degree of
/// freedom, omega^2 = k/m = 142.06758 rad^2/s^2.
fn k_with_geometric_stiffness() -> f64 {
    let kww = 12.0 * EI / H.powi(3) - 1.2 * P / H;
    let kwt = -6.0 * EI / (H * H) + 0.1 * P;
    let ktt = 4.0 * EI / H - 2.0 * P * H / 15.0;
    kww - kwt * kwt / ktt
}

#[test]
fn a_p_delta_case_with_a_hinge_that_never_yields_gives_the_cubic_geometric_stiffness() {
    // Plan test 2 and the P-Delta half of plan test 5. Mp = 1e6 kN m is never
    // reached (GRAV carries the axial load only). References: omega^2 =
    // k/m = 142.06758304696456 rad^2/s^2 (1e-9) and LAT tip deflection
    // u = 10/k = 10/1420.6758 = 7.0390e-3 m (1e-9); equal to the same model
    // without the hinge (1e-12): the hinge is rigid in these cases.
    let mut m = column();
    base_hinge(&mut m, 1.0e6);
    p_delta_case(&mut m, "GRAV", "GRAV", None);
    continuations(&mut m, "GRAV");
    analyze_model(&mut m).unwrap();
    let hinges = &m.case_results["GRAV"].hinge_results;
    assert_eq!(hinges.len(), 1);
    assert_eq!((hinges[0].direction, hinges[0].plastic_deformation), (0, 0.0));
    let axial = m.case_results["GRAV"].frame_forces["1"][0].p;
    close(axial.abs(), P, 1e-9, "axial force of GRAV");
    let k = k_with_geometric_stiffness();
    close(k, 1420.6758304696456, 1e-9, "k");
    let (omega2, tip) = answers(&m);
    close(omega2, k / MASS, 1e-9, "omega^2 = k/m");
    close(omega2, 142.06758304696456, 1e-9, "omega^2 against the digits of the probe");
    close(tip, 10.0 / k, 1e-9, "LAT tip deflection u = 10/k");

    let mut plain = column();
    p_delta_case(&mut plain, "GRAV", "GRAV", None);
    continuations(&mut plain, "GRAV");
    analyze_model(&mut plain).unwrap();
    let (omega2_plain, tip_plain) = answers(&plain);
    close(omega2, omega2_plain, 1e-12, "omega^2 equals the hinge-free model");
    close(tip, tip_plain, 1e-12, "tip deflection equals the hinge-free model");
}

// ---------------------------------------------------------------------------
// Test 3 (plan test 3): pin-ended column in 20 objects.
// ---------------------------------------------------------------------------

/// Pin-ended column along X, L = 6 m in `n` frame objects "F0".."F{n-1}",
/// E = 2e8, I = 1e-4 (EI = 2e4 kN m2), A = 1 m2, uniform mass 1 t/m (unit mass
/// 1 t/m3 on A = 1), no shear deformation. Left joint: U1, U3 fixed; right
/// joint: U3 fixed, axial compression `p` from `GRAV`. Rotations are free.
/// Optionally non-yielding M3 hinges at the outer ends of the first and last
/// object.
const L: f64 = 6.0;
const MBAR: f64 = 1.0;

fn pinned_column(n: usize, p: f64, hinges: bool) -> SapModel {
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    m.active_dofs = [true, false, true, false, true, false];
    material(&mut m, 2.0e8, MBAR);
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
    for k in 0..=n {
        m.add_point(format!("J{k}"), L * k as f64 / n as f64, 0.0, 0.0);
    }
    m.points["J0"].restraint = [true, true, true, true, false, true];
    m.points[format!("J{n}").as_str()].restraint = [false, true, true, true, false, true];
    for k in 0..n {
        m.add_frame(format!("F{k}"), format!("J{k}"), format!("J{}", k + 1), "SEC");
    }
    if hinges {
        m3_hinge(&mut m, 1.0e6);
        for (frame, rel) in [("F0".to_string(), 0.0), (format!("F{}", n - 1), 1.0)] {
            m.frames[frame.as_str()].hinges = vec![FrameHinge {
                prop: "HM".into(),
                rel_dist: rel,
            }];
        }
    }
    pattern(&mut m, "GRAV");
    joint_force(&mut m, &format!("J{n}"), "GRAV", [-p, 0.0, 0.0, 0.0, 0.0, 0.0]);
    p_delta_case(&mut m, "GRAV", "GRAV", None);
    let mut modal = LoadCase::modal_eigen("MODAL", 2);
    modal.initial_case = Some("GRAV".into());
    m.load_cases.insert("MODAL".into(), modal);
    // The same modes without any initial case: the stiffness without P.
    m.load_cases
        .insert("MODAL0".into(), LoadCase::modal_eigen("MODAL0", 2));
    m
}

#[test]
fn a_pin_ended_column_with_end_hinges_softens_as_the_beam_column_does() {
    // Reference (independent): the first frequency of a pin-ended beam-column
    // with uniform mass m = 1 t/m under axial compression P,
    //   omega^2 = (pi/L)^4 (EI/m)(1 - P/P_E),  P_E = pi^2 EI/L^2.
    // EI = 2e4 kN m2, L = 6 m: P_E = 5483.1 kN; P = 0.5 P_E = 2741.6 kN, so
    // omega(P)/omega(0) = sqrt(0.5) = 0.70710678 (tolerance 1e-4), and
    // omega(0) = (pi/L)^2 sqrt(EI/m) = 0.27416 * 141.421 = 38.77 rad/s
    // (tolerance 2e-3, the error of lumped masses on 20 elements).
    let pe = std::f64::consts::PI.powi(2) * EI / (L * L);
    let p = 0.5 * pe;
    let mut m = pinned_column(20, p, true);
    analyze_model(&mut m).unwrap();
    let hinges = &m.case_results["GRAV"].hinge_results;
    assert_eq!(hinges.len(), 2);
    assert!(hinges.iter().all(|h| h.direction == 0 && h.plastic_deformation == 0.0));
    let w_p = m.case_results["MODAL"].modes[0].eigenvalue.sqrt();
    let w_0 = m.case_results["MODAL0"].modes[0].eigenvalue.sqrt();
    let exact_0 = (std::f64::consts::PI / L).powi(2) * (EI / MBAR).sqrt();
    let exact_p = ((std::f64::consts::PI / L).powi(4) * (EI / MBAR) * (1.0 - p / pe)).sqrt();
    close(w_p / w_0, 0.5f64.sqrt(), 1e-4, "omega(P)/omega(0)");
    close(w_0, exact_0, 2e-3, "omega(0) against (pi/L)^2 sqrt(EI/m)");
    close(w_p, exact_p, 2e-3, "omega(P) against the beam-column formula");
    // Hinges are rigid: the same numbers as the hinge-free model.
    let mut plain = pinned_column(20, p, false);
    analyze_model(&mut plain).unwrap();
    close(
        w_p,
        plain.case_results["MODAL"].modes[0].eigenvalue.sqrt(),
        1e-12,
        "omega(P) equals the hinge-free model",
    );
}

// ---------------------------------------------------------------------------
// Test 4 (plan test 4): Example 1-017 of the v19 verification suite with
// non-yielding hinges at the two string ends.
// ---------------------------------------------------------------------------

/// Example 1-017 (CSI v19 Software Verification): 100 in wire, 1/16 in
/// diameter, E = 30000 ksi, mass 7.324e-7 kip-s2/in4, 0.5 kip tension from a
/// nonlinear static P-Delta case, shear area 0.9 A, `n` explicit frame
/// objects "1e0".."1e{n-1}" (kip, in, s); optional non-yielding M3 hinges at
/// rel 0 of the first and rel 1 of the last.
fn example_1_017(n: usize, hinges: bool) -> SapModel {
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    m.active_dofs = [true, false, true, false, true, false];
    material(&mut m, 30000.0, 7.324e-7);
    let area = 0.0030679615757712823;
    let inertia = 7.490140565847857e-7;
    m.frame_sections.insert(
        "WIRE".into(),
        FrameSection {
            name: "WIRE".into(),
            material: "MAT".into(),
            prop_type: FramePropType::General,
            area,
            i33: inertia,
            i22: inertia,
            torsional_j: 2.0 * inertia,
            as2: 0.0027611653450482503,
            as3: 0.0027611653450482503,
            modifiers: [1.0; 8],
            ..FrameSection::default()
        },
    );
    for k in 0..=n {
        m.add_point(format!("J{k}"), -50.0 + 100.0 * k as f64 / n as f64, 0.0, 0.0);
    }
    m.points["J0"].restraint = [true, true, true, true, false, true];
    m.points[format!("J{n}").as_str()].restraint = [false, false, true, false, false, false];
    for k in 0..n {
        m.add_frame(format!("1e{k}"), format!("J{k}"), format!("J{}", k + 1), "WIRE");
    }
    if hinges {
        m3_hinge(&mut m, 1.0e6);
        for (frame, rel) in [("1e0".to_string(), 0.0), (format!("1e{}", n - 1), 1.0)] {
            m.frames[frame.as_str()].hinges = vec![FrameHinge {
                prop: "HM".into(),
                rel_dist: rel,
            }];
        }
    }
    pattern(&mut m, "T");
    joint_force(&mut m, &format!("J{n}"), "T", [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]);
    p_delta_case(&mut m, "T", "T", None);
    let mut modal = LoadCase::modal_eigen("MODAL", 3);
    modal.initial_case = Some("T".into());
    m.load_cases.insert("MODAL".into(), modal);
    m
}

#[test]
fn example_1_017_with_hinges_at_the_string_ends_keeps_the_published_frequencies() {
    // Published SAP2000 frequencies (Hz), Example 1-017 of the v19 verification
    // suite, as in `frame_auto_mesh_regressions.rs`: 10 elements 74.579,
    // 148.93, 222.06; 100 elements 74.587, 149.18, 223.80. Tolerances of that
    // test: 6e-4 Hz below 100 Hz, 6e-3 Hz above. The hinges (M3, Mp = 1e6
    // kip in, at the pinned ends, so the moment there is zero) are rigid and
    // do not yield: the frequencies equal those of the hinge-free model (1e-12).
    for (n, published) in [(10, [74.579, 148.93, 222.06]), (100, [74.587, 149.18, 223.80])] {
        let mut m = example_1_017(n, true);
        analyze_model(&mut m).unwrap_or_else(|e| panic!("{n} objects: {e}"));
        let hinges = &m.case_results["T"].hinge_results;
        assert_eq!(hinges.len(), 2, "{n}");
        assert!(hinges.iter().all(|h| h.direction == 0 && h.plastic_deformation == 0.0));
        let mut plain = example_1_017(n, false);
        analyze_model(&mut plain).unwrap();
        let modes = &m.case_results["MODAL"].modes;
        let modes_plain = &plain.case_results["MODAL"].modes;
        assert_eq!(modes.len(), 3);
        for ((mode, mode_plain), expected) in modes.iter().zip(modes_plain).zip(published) {
            let tolerance = if expected < 100.0 { 6e-4 } else { 6e-3 };
            assert!(
                (mode.freq_hz - expected).abs() < tolerance,
                "{n} objects, mode {}: {} vs {expected}",
                mode.mode_num,
                mode.freq_hz
            );
            close(
                mode.freq_hz,
                mode_plain.freq_hz,
                1e-12,
                &format!("{n} objects, mode {} equals the hinge-free model", mode.mode_num),
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Tests 5 and 6: a yielded hinge in a P-Delta chain is refused; an elastic one
// is accepted; with and without automatic subdivision.
// ---------------------------------------------------------------------------

/// `GRAV` (P-Delta, 2000 kN compression) then `PUSH` (P-Delta, displacement
/// control to `target` m under the lateral pattern, continuing from `GRAV`).
fn p_delta_push(m: &mut SapModel, target: f64) {
    p_delta_case(m, "GRAV", "GRAV", None);
    p_delta_case(m, "PUSH", "LAT", Some("GRAV"));
    m.load_cases["PUSH"].displacement_control = Some(DisplacementControl {
        target,
        monitored: Some(("2".into(), 2)),
        conjugate: false,
    });
}

#[test]
fn a_hinge_yielded_in_a_p_delta_case_refuses_the_modal_case_naming_it() {
    // Mp = 100 kN m: under P = 2000 kN the hinge yields at a tip deflection
    // near 0.02 m (M = lambda H + P u = 100), the push goes to 0.05 m and
    // leaves it on its backbone. The Modal and the linear static case on
    // PUSH are refused (G48 (a): only a hinge that has not yielded is rigid).
    // Run with the frame as one element and automatically cut into 2.
    for segments in [0usize, 2] {
        for kind in ["MODAL", "LAT"] {
            let mut m = column();
            base_hinge(&mut m, 100.0);
            m.frames["1"].auto_mesh_segments = segments;
            p_delta_push(&mut m, 0.05);
            if kind == "MODAL" {
                let mut modal = LoadCase::modal_eigen("MODAL", 2);
                modal.initial_case = Some("PUSH".into());
                m.load_cases.insert("MODAL".into(), modal);
            } else {
                let mut lat = LoadCase::linear_static("LAT", "LAT", 1.0);
                lat.initial_case = Some("PUSH".into());
                m.load_cases.insert("LAT".into(), lat);
            }
            let e = analyze_model(&mut m).unwrap_err();
            let msg = unsupported(&e);
            let what = format!("{segments} segments, {kind}: {msg}");
            assert!(msg.contains("hinge 1H1"), "{what}");
            assert!(msg.contains("frame '1'"), "{what}");
            assert!(msg.contains(&format!("case '{kind}'")), "{what}");
            assert!(msg.contains("nonlinear static case 'PUSH'"), "{what}");
            assert!(msg.contains("yielded and is on its backbone"), "{what}");
            assert!(m.case_results.is_empty(), "{what}");
        }
    }
}

#[test]
fn a_subdivided_frame_with_an_elastic_end_hinge_in_a_p_delta_chain() {
    // The hinge sits at rel 0 of the object, which is the end I of its first
    // analysis element when the object is cut into 2. The hinge never yields
    // (Mp = 1e6). Results equal those of the hinge-free model cut the same
    // way (1e-12), and approach the exact beam-column: tip stiffness
    // k = P / (tan(mu H)/mu - H), mu = sqrt(P/EI) = 0.316228, so omega^2 =
    // k/m = 141.86503 rad^2/s^2 (mass only at the free end). The cubic
    // geometric stiffness is approximate: one element is 1.4e-3 above the
    // exact value (142.06758), two elements 1.05e-4 above (141.87996); the
    // bound asserted for 2 elements is 5e-4 (mesh error, not a tolerance of
    // the plan).
    let mut m = column();
    base_hinge(&mut m, 1.0e6);
    m.frames["1"].auto_mesh_segments = 2;
    p_delta_case(&mut m, "GRAV", "GRAV", None);
    continuations(&mut m, "GRAV");
    analyze_model(&mut m).unwrap();
    let hinges = &m.case_results["GRAV"].hinge_results;
    assert_eq!(hinges.len(), 1);
    assert_eq!((hinges[0].direction, hinges[0].plastic_deformation), (0, 0.0));
    let (omega2, tip) = answers(&m);

    let mut plain = column();
    plain.frames["1"].auto_mesh_segments = 2;
    p_delta_case(&mut plain, "GRAV", "GRAV", None);
    continuations(&mut plain, "GRAV");
    analyze_model(&mut plain).unwrap();
    let (omega2_plain, tip_plain) = answers(&plain);
    close(omega2, omega2_plain, 1e-12, "omega^2 equals the hinge-free subdivided model");
    close(tip, tip_plain, 1e-12, "tip deflection equals the hinge-free subdivided model");
    let mu = (P / EI).sqrt();
    let k_exact = P / ((mu * H).tan() / mu - H);
    close(omega2, k_exact / MASS, 5e-4, "omega^2 against the exact beam-column");
    close(tip, 10.0 / k_exact, 5e-4, "tip deflection against the exact beam-column");
}
