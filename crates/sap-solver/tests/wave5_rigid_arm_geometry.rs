//! Wave 5 P13, step b: geometric stiffness of the rigid arms of a frame.
//!
//! A rigid arm of length a at a joint turns with the joint. The axial force P
//! acts through the slope of the arm, which is the rotation r of the joint, over
//! the length a: the energy of the arm is P a r^2 / 2, the "P x d term on the
//! joint rotation". The flexible part carries its own consistent (cubic Hermite)
//! geometric stiffness, so the geometric stiffness of the whole member needs the
//! arm term in addition.
//!
//! Reference 1 (hand derivation). A column pinned at both joints (rotations free)
//! with rigid arms d at both ends (rigid-end factor 1) and the flexible span of
//! length lf = L - 2 d. In the symmetric mode the face displacements are
//! (u, r, u, r) = (d t, t, d t, -t) for a joint rotation t at I and -t at J, so
//! with the cubic Hermite matrices of the span
//!   q^T K q = 4 E I t^2 / lf               (rows: 2 lf^2 t and -2 lf^2 t),
//!   q^T Kg q = P (lf / 3) t^2              (2 (c22 - c24) t^2, c22 - c24 = P lf / 6),
//! and the two arms add 2 P d t^2. The member buckles when 4 E I / lf + P (lf / 3
//! + 2 d) = 0 with P = -Pcr, that is
//!   Pcr = 4 E I / (lf (lf / 3 + 2 d)).
//!
//! For d = 0 this is the 12 E I / L^2 of the single cubic element.
//!
//! Reference 2: the same column with the arms modelled as separate frame elements
//! of very large stiffness between the joint and the face. Their cubic geometric
//! stiffness is exact for a rigid-body rotation, so the two models must give the
//! same factor up to the finite stiffness of the arms.
//!
//! The continuum solution of the same column (the flexible span between rigid
//! arms, pinned at the joints) is the root of tan(k lf / 2) = 1 / (k d), Pcr = E I
//! k^2; the single cubic element overestimates it, so the factor must lie above.
//! Units: kip, in.

use sap_model::*;
use sap_solver::analyze_model;

const E: f64 = 29_000.0;
const I: f64 = 100.0;
const L: f64 = 120.0;

fn section(name: &str, area: f64, inertia: f64) -> FrameSection {
    FrameSection {
        name: name.into(),
        material: "MAT".into(),
        area,
        i22: inertia,
        i33: inertia,
        torsional_j: inertia,
        as2: 0.0,
        as3: 0.0,
        ..FrameSection::default()
    }
}

fn base() -> SapModel {
    let mut model = SapModel::default();
    model.load_cases.clear();
    model.load_patterns.clear();
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.e = E;
    mat.u = 0.3;
    mat.g = 11_000.0;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0;
    model.materials.insert("MAT".into(), mat);
    model.frame_sections.insert("S".into(), section("S", 10.0, I));
    model
        .frame_sections
        .insert("STIFF".into(), section("STIFF", 1.0e6, 1.0e9));
    model
}

/// Adds the compressive load of 1 at the last joint and the buckling case.
fn finish(model: &mut SapModel, last: &str) {
    model.load_patterns.insert(
        "A".into(),
        LoadPattern {
            name: "A".into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
    model.points.get_mut(last).unwrap().loads_force.push(PointLoadForce {
        load_pat: "A".into(),
        csys: "GLOBAL".into(),
        force: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    });
    let mut case = LoadCase::linear_static("BUCK", "A", 1.0);
    case.case_type = LoadCaseType::Buckling;
    case.num_buckling_modes = 1;
    model.load_cases.insert("BUCK".into(), case);
}

/// Pinned at both joints: U1 (first joint only), U2, U3 and R1 fixed, R2 and R3
/// free.
fn pin(model: &mut SapModel, name: &str, axial_fixed: bool) {
    model.points.get_mut(name).unwrap().restraint = [axial_fixed, true, true, true, false, false];
}

fn offsets_model(d: f64) -> SapModel {
    let mut model = base();
    model.add_point("J1", 0.0, 0.0, 0.0);
    model.add_point("J2", L, 0.0, 0.0);
    pin(&mut model, "J1", true);
    pin(&mut model, "J2", false);
    model.add_frame("F", "J1", "J2", "S");
    let frame = model.frames.get_mut("F").unwrap();
    frame.end_offset_i = d;
    frame.end_offset_j = d;
    frame.rigid_zone_factor = 1.0;
    finish(&mut model, "J2");
    model
}

fn arms_as_elements(d: f64) -> SapModel {
    let mut model = base();
    for (name, x) in [("J1", 0.0), ("F1", d), ("F2", L - d), ("J2", L)] {
        model.add_point(name, x, 0.0, 0.0);
    }
    pin(&mut model, "J1", true);
    pin(&mut model, "J2", false);
    model.add_frame("A1", "J1", "F1", "STIFF");
    model.add_frame("M", "F1", "F2", "S");
    model.add_frame("A2", "F2", "J2", "STIFF");
    finish(&mut model, "J2");
    model
}

fn factor(model: &mut SapModel) -> f64 {
    analyze_model(model).unwrap();
    model.case_results["BUCK"].buckling_factors[0]
}

/// The column with arms d = 10 at both ends: lf = 100.
#[test]
fn rigid_arms_add_p_times_arm_to_the_joint_rotation_stiffness() {
    let d = 10.0;
    let lf = L - 2.0 * d;
    let ei = E * I;
    let hermite = 4.0 * ei / (lf * (lf / 3.0 + 2.0 * d));
    let found = factor(&mut offsets_model(d));
    assert!(
        (found - hermite).abs() <= 1e-9 * hermite,
        "buckling factor {found} against the hand formula {hermite}"
    );

    // The arms as stiff elements: the same factor, up to the finite stiffness
    // (EI 1e9 against 2.9e6 of the span: a relative error of order 1e-5).
    let explicit = factor(&mut arms_as_elements(d));
    assert!(
        (explicit - hermite).abs() <= 1e-4 * hermite,
        "arms as elements {explicit} against {hermite}"
    );

    // The continuum value (root of tan(k lf / 2) = 1 / (k d), found by
    // bisection on x = k lf / 2 for x tan x = lf / (2 d)) is a lower bound of the
    // single cubic element: its stiffness and geometric matrices come from one
    // assumed displacement field, so the Rayleigh quotient cannot fall below the
    // exact critical load (the plain Euler column: 12 against pi^2).
    let target = lf / (2.0 * d);
    let (mut lo, mut hi) = (1.0e-6, std::f64::consts::FRAC_PI_2 - 1.0e-9);
    for _ in 0..200 {
        let x = 0.5 * (lo + hi);
        if x * x.tan() < target {
            lo = x;
        } else {
            hi = x;
        }
    }
    let k = 2.0 * lo / lf;
    let exact = ei * k * k;
    assert!(found > exact, "{found} against the continuum {exact}");
}

/// Without arms the formula is the single cubic element, 12 E I / L^2, and the
/// factor does not change (the arm term is zero).
#[test]
fn a_column_without_offsets_is_the_single_cubic_element() {
    let found = factor(&mut offsets_model(0.0));
    let hermite = 12.0 * E * I / (L * L);
    assert!((found - hermite).abs() <= 1e-9 * hermite, "{found} against {hermite}");
}
