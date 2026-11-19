//! Wave 5, ledger G24: the eccentric moment of an automatic seismic load on a
//! diaphragm (CSI Lateral Loads Manual, section 2.2.3, pp. 2-4 and 2-5).
//!
//! The manual: the moment equals "the specified ratio eccentricity times the
//! maximum width of the diaphragm perpendicular to the direction of the
//! seismic loading times the total lateral force applied to the diaphragm",
//! where the width comes from the extreme coordinates of the joints of the
//! diaphragm constraint; an overwrite distance replaces ratio times width;
//! without a diaphragm the eccentricity is ignored.
//!
//! The manual does NOT state the sign of a positive eccentricity. Project
//! choice (also in the code and the window): a positive eccentricity moves the
//! line of action of the force toward +Y for an X load and +X for a Y load, so
//! the couple about Z is -e F for X and +e F for Y. The tests check the
//! magnitude and both senses (positive and negative ratio) against statics:
//! the extra moment about Z of the applied loads is M, so the base reaction
//! moment about the origin changes by -M while the base shears do not change.
//!
//! Model: one storey, columns at (0,0), (10,0), (0,6), (10,6) (kN, m), rigid
//! Z diaphragm at z = 3, equal masses at the four top joints. F is evaluated
//! here from the manual's equations (Cs of Eqn. 12.8-2/3/5, one level so
//! V = Cs W) and compared with the reaction. Tolerance: 1e-7 relative to
//! F times the width for moments (the diaphragm is a penalty constraint, whose
//! conditioning limits the equilibrium residual; the measured error is far
//! smaller, 6e-11 for the moments and 1.1e-8 for the shear, printed with `--nocapture`).
use sap_model::seismic::{AutoSeismicAsce710, SeismicDirection, SeismicPeriod};
use sap_model::{
    ConstraintAxis, ConstraintDef, ConstraintType, FrameSection, LoadCase, LoadPattern,
    LoadPatternType, Material, SapModel,
};
use sap_solver::{analyze_model, SolverError};
use sap_units::Units;

const G: f64 = 9.806_65;
const MASS: f64 = 10.0; // kN s^2 / m at each top joint
const HEIGHT: f64 = 3.0;
const COLUMNS: [(&str, f64, f64); 4] = [
    ("1", 0.0, 0.0),
    ("2", 10.0, 0.0),
    ("3", 0.0, 6.0),
    ("4", 10.0, 6.0),
];

fn definition(direction: SeismicDirection, ratio: f64) -> AutoSeismicAsce710 {
    AutoSeismicAsce710 {
        direction,
        eccentricity_ratio: ratio,
        period: SeismicPeriod::User { t: 1.0 },
        r: 5.0,
        omega0: 3.0,
        cd: 4.5,
        ie: 1.0,
        ss: 1.5,
        s1: 0.5,
        tl: 8.0,
        fa: Some(1.0),
        fv: Some(1.0),
        z_min: None,
        z_max: None,
    }
}

/// Lateral force of the storey from the manual's equations (ASCE 7-10 12.8):
/// T = 1 s <= TL, one level so V = Cs W.
fn storey_force() -> f64 {
    let (ss, s1, fa, fv, r, ie, t): (f64, f64, f64, f64, f64, f64, f64) =
        (1.5, 0.5, 1.0, 1.0, 5.0, 1.0, 1.0);
    let sds = 2.0 / 3.0 * fa * ss;
    let sd1 = 2.0 / 3.0 * fv * s1;
    let ri = r / ie;
    let cs = (sds / ri)
        .min(sd1 / (t * ri)) // 12.8-3 (T <= TL)
        .max((0.044_f64 * sds * ie).max(0.01)); // 12.8-5
    cs * 4.0 * MASS * G
}

fn building(with_diaphragm: bool, direction: SeismicDirection, ratio: f64) -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    m.load_cases.clear();
    m.load_patterns.clear();
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.e = 2.0e8;
    mat.g = 2.0e8 / 2.6;
    mat.unit_weight = 0.0;
    mat.unit_mass = 0.0;
    m.materials.insert("MAT".into(), mat);
    let mut sec = FrameSection::default();
    sec.name = "SEC".into();
    sec.material = "MAT".into();
    sec.area = 0.05;
    sec.i33 = 5.0e-4;
    sec.i22 = 5.0e-4;
    sec.torsional_j = 1.0e-3;
    sec.as2 = 0.0;
    sec.as3 = 0.0;
    m.frame_sections.insert("SEC".into(), sec);
    m.load_patterns.insert(
        "EQ".into(),
        LoadPattern {
            name: "EQ".into(),
            pat_type: LoadPatternType::Quake,
            self_wt_multiplier: 0.0,
        },
    );
    for (name, x, y) in COLUMNS {
        let base = format!("B{name}");
        m.add_point(base.clone(), x, y, 0.0);
        m.points[&base].restraint = [true; 6];
        m.add_point(name, x, y, HEIGHT);
        m.points[name].mass = [MASS, MASS, MASS, 0.0, 0.0, 0.0];
        m.add_frame(format!("C{name}"), base, name.to_string(), "SEC");
    }
    if with_diaphragm {
        m.constraints.insert(
            "D1".into(),
            ConstraintDef {
                name: "D1".into(),
                constraint_type: ConstraintType::Diaphragm,
                axis: ConstraintAxis::Z,
                dofs: [true, true, false, false, false, true],
            },
        );
        for (name, _, _) in COLUMNS {
            m.points[name].constraints.push("D1".into());
        }
    }
    m.auto_seismic
        .insert("EQ".into(), definition(direction, ratio));
    m.load_cases
        .insert("CASE".into(), LoadCase::linear_static("CASE", "EQ", 1.0));
    m
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1e-300)
}

struct Outcome {
    reactions: [f64; 6],
    rz: f64,
    moments: usize,
    moment_z: f64,
    width: f64,
}

fn run(m: &mut SapModel) -> Outcome {
    analyze_model(m).unwrap();
    let case = &m.case_results["CASE"];
    let report = &case.auto_lateral[0];
    Outcome {
        reactions: case.base_reactions,
        rz: case.joint_displacements["1"][5],
        moments: report.diaphragm_moments.len(),
        moment_z: report.diaphragm_moments.first().map_or(0.0, |d| d.moment_z),
        width: report.diaphragm_moments.first().map_or(0.0, |d| d.width),
    }
}

/// Reaction moment change against the eccentric case for a load direction.
fn check(direction: SeismicDirection, ratio: f64, width: f64, sense: f64) {
    let f = storey_force();
    let base = run(&mut building(true, direction, 0.0));
    let ecc = run(&mut building(true, direction, ratio));
    let expected_m = sense * ratio * width * f;
    // The reported moment and its width.
    assert_eq!(ecc.moments, 1);
    assert!(rel(ecc.width, width) <= 1e-12, "width {}", ecc.width);
    assert!(
        (ecc.moment_z - expected_m).abs() <= 1e-9 * f * width,
        "moment {} vs {expected_m}",
        ecc.moment_z
    );
    // Statics: the reaction moment about Z changes by -M; the shears do not.
    let d_mz = ecc.reactions[5] - base.reactions[5];
    let err = (d_mz + expected_m).abs() / (f * width);
    println!(
        "{direction:?} ratio {ratio}: dMz {d_mz}, expected {}, error {err:e}",
        -expected_m
    );
    assert!(
        err <= 1e-7,
        "reaction moment changed by {d_mz}, expected {}",
        -expected_m
    );
    for i in [0, 1, 2, 3, 4] {
        assert!(
            (ecc.reactions[i] - base.reactions[i]).abs() <= 1e-7 * f * width,
            "reaction {i} changed"
        );
    }
    // The total base shear in the load direction is F (reaction opposes it); the
    // penalty diaphragm leaves a residual of 1e-8 relative (measured 1.1e-8).
    let axis = direction.index();
    assert!(
        rel(-ecc.reactions[axis], f) <= 1e-7,
        "base shear {}",
        ecc.reactions[axis]
    );
    // The diaphragm turns with the moment (no rotation without it, by symmetry).
    println!("rz without eccentricity {:e}, with {:e}", base.rz, ecc.rz);
    assert!(base.rz.abs() <= 1e-6 * ecc.rz.abs());
    assert!(
        ecc.rz * expected_m > 0.0,
        "rotation {} against moment {expected_m}",
        ecc.rz
    );
}

#[test]
fn x_load_moment_is_ratio_times_width_perpendicular_times_force() {
    // Width in Y is 6, M = 0.05 * 6 * F = 0.3 F; sense -1 for X (project choice).
    check(SeismicDirection::X, 0.05, 6.0, -1.0);
}

#[test]
fn y_load_uses_the_x_width() {
    // Width in X is 10, M = 0.05 * 10 * F = 0.5 F.
    check(SeismicDirection::Y, 0.05, 10.0, 1.0);
}

#[test]
fn a_negative_ratio_reverses_the_sense() {
    check(SeismicDirection::X, -0.05, 6.0, -1.0);
    check(SeismicDirection::Y, -0.05, 10.0, 1.0);
}

#[test]
fn an_overwrite_distance_replaces_ratio_times_width() {
    let f = storey_force();
    let mut with_ratio = building(true, SeismicDirection::Y, 0.05);
    with_ratio
        .auto_seismic_eccentricity
        .entry("EQ".into())
        .or_default()
        .insert("D1".into(), 0.4);
    let base = run(&mut building(true, SeismicDirection::Y, 0.0));
    let out = run(&mut with_ratio);
    // M = 0.4 F, not 0.5 F.
    assert!((out.moment_z - 0.4 * f).abs() <= 1e-9 * f);
    let err = (out.reactions[5] - base.reactions[5] + 0.4 * f).abs() / f;
    assert!(err <= 1e-7, "error {err:e}");
    // An overwrite acts although the ratio is zero.
    let mut zero_ratio = building(true, SeismicDirection::X, 0.0);
    zero_ratio
        .auto_seismic_eccentricity
        .entry("EQ".into())
        .or_default()
        .insert("D1".into(), 0.4);
    let out = run(&mut zero_ratio);
    assert!((out.moment_z + 0.4 * f).abs() <= 1e-9 * f);
}

#[test]
fn without_a_diaphragm_the_ratio_changes_nothing_bitwise() {
    let none = run(&mut building(false, SeismicDirection::X, 0.0));
    let with_ratio = run(&mut building(false, SeismicDirection::X, 0.05));
    assert_eq!(with_ratio.moments, 0);
    for i in 0..6 {
        assert_eq!(
            none.reactions[i].to_bits(),
            with_ratio.reactions[i].to_bits()
        );
    }
    assert_eq!(none.rz.to_bits(), with_ratio.rz.to_bits());
}

#[test]
fn a_zero_ratio_with_a_diaphragm_adds_nothing() {
    let out = run(&mut building(true, SeismicDirection::X, 0.0));
    assert_eq!(out.moments, 0);
}

fn refused(m: &mut SapModel, needle: &str) {
    let err = analyze_model(m).expect_err("must be refused");
    let text = err.to_string();
    assert!(
        matches!(
            err,
            SolverError::UnsupportedInput(_) | SolverError::InvalidInput(_)
        ),
        "wrong error {err:?}"
    );
    assert!(text.contains(needle), "message `{text}` lacks `{needle}`");
    assert!(!m.is_locked && m.case_results.is_empty());
}

#[test]
fn refusals_name_the_diaphragm() {
    // An overwrite on something that is not a Z diaphragm, or on nothing.
    let mut m = building(true, SeismicDirection::X, 0.05);
    m.auto_seismic_eccentricity
        .entry("EQ".into())
        .or_default()
        .insert("GHOST".into(), 0.4);
    refused(&mut m, "GHOST");
    let mut m = building(true, SeismicDirection::X, 0.05);
    m.constraints.get_mut("D1").unwrap().axis = ConstraintAxis::X;
    refused(&mut m, "D1");
    let mut m = building(true, SeismicDirection::X, 0.05);
    m.auto_seismic_eccentricity
        .entry("EQ".into())
        .or_default()
        .insert("D1".into(), f64::NAN);
    refused(&mut m, "D1");
    // A moment about Z needs RZ.
    let mut m = building(true, SeismicDirection::X, 0.05);
    m.active_dofs[5] = false;
    m.constraints.get_mut("D1").unwrap().dofs = [true, true, false, false, false, false];
    refused(&mut m, "rotation about Z");
}
