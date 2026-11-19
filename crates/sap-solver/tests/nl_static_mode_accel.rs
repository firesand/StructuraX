//! Accel and Mode loads of nonlinear static cases.
//!
//! References, all closed forms evaluated in the tests:
//! - CSI Analysis Reference, "Nonlinear Static Analysis": an Accel load is
//!   minus the joint masses times the unit acceleration (`f = -sf M r`); a
//!   Mode load is the mode shape times its squared circular frequency times
//!   the mass, with `phi^T M phi = 1`, so that a linear structure answers with
//!   `sf phi_n`.
//! - Two-degree-of-freedom chain (two bars of stiffness k, two masses m):
//!   `omega^2 = (3 -+ sqrt 5) k / (2 m)` (Chopra, Dynamics of Structures,
//!   ch. 10).
//! - Plastic collapse by virtual work: a cantilever with a base hinge of
//!   strength Mp collapses under `lambda sum(m_j z_j) = Mp`; a fixed portal
//!   with Mp at the four member ends under `lambda sum(m_j) h = 4 Mp`.
//!
//! Units are consistent (kN, m, s unless noted). Tolerances: 1e-9 relative for
//! the linear closed forms, 1e-7 for quantities that pass through the
//! eigen-solver, 1e-6 for load factors read from an event-driven pushover.
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

fn bare_model(active: [bool; 6]) -> SapModel {
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    m.active_dofs = active;
    m.load_patterns.insert(
        "P".into(),
        LoadPattern {
            name: "P".into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
    m
}

fn add_material(m: &mut SapModel, e: f64, unit_mass: f64) {
    let mut material = Material::default_steel_kip_in();
    material.name = "MAT".into();
    material.e = e;
    material.g = e / 2.6;
    material.u = 0.3;
    material.unit_weight = unit_mass * 9.80665;
    material.unit_mass = unit_mass;
    m.materials.insert("MAT".into(), material);
}

fn add_section(m: &mut SapModel, name: &str, a: f64, i: f64) {
    m.frame_sections.insert(
        name.into(),
        FrameSection {
            name: name.into(),
            material: "MAT".into(),
            prop_type: FramePropType::General,
            area: a,
            i33: i,
            i22: i,
            torsional_j: 2.0 * i,
            as2: 0.0,
            as3: 0.0,
            modifiers: [1.0; 8],
            ..FrameSection::default()
        },
    );
}

fn nl_case(name: &str, loads: &[(&str, f64)]) -> LoadCase {
    let mut c = LoadCase::linear_static(name, "P", 1.0);
    c.case_type = LoadCaseType::NonlinearStatic;
    c.loads.clear();
    for (n, sf) in loads {
        c.loads.push(LoadCaseAssignment {
            load_name: (*n).into(),
            scale_factor: *sf,
        });
    }
    c
}

fn mode_case(name: &str, modal: &str, mode: usize, sf: f64) -> LoadCase {
    let mut c = nl_case(name, &[]);
    c.modal_case = Some(modal.into());
    c.nl_static.mode_loads.push(ModeLoad {
        mode,
        scale_factor: sf,
    });
    c
}

fn refused(m: &mut SapModel) -> SolverError {
    analyze_model(m).expect_err("the analysis should be refused")
}

fn assert_kind(e: &SolverError, unsupported: bool, needles: &[&str]) {
    let msg = match (e, unsupported) {
        (SolverError::UnsupportedInput(s), true) | (SolverError::InvalidInput(s), false) => s,
        other => panic!("wrong error kind: {other:?}"),
    };
    for n in needles {
        assert!(msg.contains(n), "message '{msg}' lacks '{n}'");
    }
}

// ---------------------------------------------------------------------------
// Axial chain along X: fixed joint J0, free joints J1 (mass m1) and J2 (m2),
// bars of stiffness k1 = E A / L (J0-J1) and k2 (J1-J2). Only U1 is active.
// ---------------------------------------------------------------------------

fn chain(k1: f64, k2: f64, m1: f64, m2: f64) -> SapModel {
    let mut m = bare_model([true, false, false, false, false, false]);
    // E = 1 and L = 1: the bar stiffness is its area.
    add_material(&mut m, 1.0, 0.0);
    add_section(&mut m, "S1", k1, 1.0);
    add_section(&mut m, "S2", k2, 1.0);
    m.add_point("J0", 0.0, 0.0, 0.0);
    m.add_point("J1", 1.0, 0.0, 0.0);
    m.add_point("J2", 2.0, 0.0, 0.0);
    m.points["J0"].restraint = [true; 6];
    m.points["J1"].mass[0] = m1;
    m.points["J2"].mass[0] = m2;
    m.add_frame("B1", "J0", "J1", "S1");
    m.add_frame("B2", "J1", "J2", "S2");
    m
}

fn u(m: &SapModel, case: &str, joint: &str) -> f64 {
    m.case_results[case].joint_displacements[joint][0]
}

#[test]
fn accel_load_is_minus_mass_times_acceleration_and_the_base_shear_is_sf_times_the_mass() {
    // Joint forces -sf m1 and -sf m2 on a chain: the bar forces are
    // N2 = -sf m2 (J1-J2) and N1 = -sf (m1 + m2) (J0-J1), so
    // u1 = -sf (m1 + m2) / k1 and u2 = u1 - sf m2 / k2.
    let (k1, k2, m1, m2, sf) = (40.0, 15.0, 2.0, 3.0, 1.5);
    let mut m = chain(k1, k2, m1, m2);
    m.load_cases
        .insert("ACC".into(), nl_case("ACC", &[("UX", sf)]));
    analyze_model(&mut m).unwrap();
    let u1 = -sf * (m1 + m2) / k1;
    close(u(&m, "ACC", "J1"), u1, 1e-9, "J1 displacement");
    close(
        u(&m, "ACC", "J2"),
        u1 - sf * m2 / k2,
        1e-9,
        "J2 displacement",
    );
    // The support reaction balances the total force: sf (m1 + m2).
    close(
        m.case_results["ACC"].base_reactions[0],
        sf * (m1 + m2),
        1e-9,
        "base shear",
    );
}

#[test]
fn accel_load_uses_the_element_mass_and_a_chained_case_adds_to_the_response() {
    // Self mass of the bars (rho A L / 2 at each end, so the mass source of
    // the model decides what M is) plus a point mass. ACC1 is the Accel case,
    // ACC2 continues from it with a joint force in a pattern: linear
    // superposition.
    let (k, rho, m1, f) = (20.0, 0.25, 1.0, 4.0);
    let mut m = chain(k, k, m1, 0.0);
    m.materials["MAT"].unit_mass = rho;
    m.materials["MAT"].unit_weight = rho * 9.80665;
    // The self mass of one bar is rho * A * L = rho * k, half at each end.
    let bar = rho * k;
    let (mass1, mass2) = (m1 + bar, bar / 2.0);
    m.points["J2"].loads_force.push(PointLoadForce {
        load_pat: "P".into(),
        csys: "GLOBAL".into(),
        force: [f, 0.0, 0.0, 0.0, 0.0, 0.0],
    });
    m.load_cases
        .insert("ACC1".into(), nl_case("ACC1", &[("UX", -1.0)]));
    let mut c2 = nl_case("ACC2", &[("P", 1.0)]);
    c2.initial_case = Some("ACC1".into());
    m.load_cases.insert("ACC2".into(), c2);
    analyze_model(&mut m).unwrap();
    // ACC1: forces +mass1 at J1 and +mass2 at J2 (acceleration -1).
    let u1 = (mass1 + mass2) / k;
    close(u(&m, "ACC1", "J1"), u1, 1e-9, "ACC1 J1");
    close(u(&m, "ACC1", "J2"), u1 + mass2 / k, 1e-9, "ACC1 J2");
    // ACC2 adds f at J2: both bars carry f.
    close(u(&m, "ACC2", "J1"), u1 + f / k, 1e-9, "ACC2 J1");
    close(
        u(&m, "ACC2", "J2"),
        u1 + mass2 / k + 2.0 * f / k,
        1e-9,
        "ACC2 J2",
    );
}

#[test]
fn mode_load_of_the_two_dof_chain_produces_the_mode_shape() {
    // k1 = k2 = k, m1 = m2 = m. omega^2 = (3 -+ sqrt 5) k / (2 m); the shape
    // ratio phi2 / phi1 = 1 / (1 - m omega^2 / k); phi^T M phi = 1. The load
    // is sf omega^2 m phi, so the static response is sf phi.
    let (k, mass, sf) = (30.0, 2.0, 2.5);
    let s5 = 5.0f64.sqrt();
    for (mode, w2) in [
        (1, (3.0 - s5) * k / (2.0 * mass)),
        (2, (3.0 + s5) * k / (2.0 * mass)),
    ] {
        let mut m = chain(k, k, mass, mass);
        // PUSH is listed before its modal case: the case order follows the
        // modal dependency, not the file order.
        m.load_cases
            .insert("PUSH".into(), mode_case("PUSH", "MODAL", mode, sf));
        m.load_cases
            .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 2));
        analyze_model(&mut m).unwrap();
        let r = 1.0 / (1.0 - mass * w2 / k);
        let phi1 = 1.0 / (mass * (1.0 + r * r)).sqrt();
        let modal = &m.case_results["MODAL"].modes[mode - 1];
        close(modal.circ_freq_rad_s.powi(2), w2, 1e-8, "omega^2");
        // The stored shape may have either sign; the load follows it.
        let sign = modal.mode_shape["J1"][0].signum();
        close(
            u(&m, "PUSH", "J1"),
            sf * sign * phi1,
            1e-7,
            &format!("mode {mode} J1"),
        );
        close(
            u(&m, "PUSH", "J2"),
            sf * sign * phi1 * r,
            1e-7,
            &format!("mode {mode} J2"),
        );
        // Base shear: minus the sum of the joint forces sf w2 m phi.
        close(
            m.case_results["PUSH"].base_reactions[0],
            -sf * w2 * mass * sign * phi1 * (1.0 + r),
            1e-7,
            &format!("mode {mode} base shear"),
        );
    }
}

#[test]
fn mode_load_on_a_system_with_unequal_masses_keeps_the_generalized_mass_at_one() {
    // m1 = 1, m2 = 4, k1 = 10, k2 = 6: the response to the Mode load is
    // sf phi regardless of the normalization, and phi^T M phi = 1 gives
    // m1 u1^2 + m2 u2^2 = sf^2.
    let (k1, k2, m1, m2, sf) = (10.0, 6.0, 1.0, 4.0, 0.7);
    let mut m = chain(k1, k2, m1, m2);
    m.load_cases
        .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 2));
    m.load_cases
        .insert("PUSH".into(), mode_case("PUSH", "MODAL", 2, sf));
    analyze_model(&mut m).unwrap();
    let (u1, u2) = (u(&m, "PUSH", "J1"), u(&m, "PUSH", "J2"));
    close(m1 * u1 * u1 + m2 * u2 * u2, sf * sf, 1e-7, "phi^T M phi");
    // The response is the eigenvector: K phi = w^2 M phi.
    let w2 = m.case_results["MODAL"].modes[1].circ_freq_rad_s.powi(2);
    close(
        (k1 + k2) * u1 - k2 * u2,
        w2 * m1 * u1,
        1e-7,
        "row 1 of K phi = w^2 M phi",
    );
    close(
        -k2 * u1 + k2 * u2,
        w2 * m2 * u2,
        1e-7,
        "row 2 of K phi = w^2 M phi",
    );
}

// ---------------------------------------------------------------------------
// Collapse under Accel loads. Expected load factors by virtual work; the
// mechanism is the one the results report (the hinge that deforms on the
// plateau), and the load is checked against the uniqueness theorem: the
// external work of that mechanism equals the plastic work, and no hinge
// exceeds its strength.
// ---------------------------------------------------------------------------

const E: f64 = 2.0e8;
const IC: f64 = 1.0e-4;

fn epp_moment(name: &str, mp: f64) -> HingeProp {
    let mut p = HingeProp::symmetric(
        name,
        HingeDof::M3,
        [
            HingePoint::new(1.0, 0.0),
            HingePoint::new(1.0, 0.25),
            HingePoint::new(1.0, 0.35),
            HingePoint::new(1.0, 0.5),
        ],
    );
    p.force_sf = mp;
    p.force_sf_negative = mp;
    p
}

fn hinge_at(prop: &str, rel: f64) -> FrameHinge {
    FrameHinge {
        prop: prop.into(),
        rel_dist: rel,
    }
}

fn pushover(c: &mut LoadCase, joint: &str, target: f64) {
    c.displacement_control = Some(DisplacementControl {
        target,
        monitored: Some((joint.into(), 0)),
        conjugate: false,
    });
    c.nl_static.results_saved = ResultsSaved::MultipleStates {
        min_saved: 20,
        max_saved: 100,
        positive_only: true,
    };
}

/// Work of the load vector `f` (per unit load factor, on U1 of the joints) on
/// the last plateau interval against the plastic work of the hinges.
fn plateau_work(case: &CaseResults, joints: &[(&str, f64)]) -> (f64, f64) {
    let n = case.static_steps.len();
    let (a, b) = (&case.static_steps[n - 2], &case.static_steps[n - 1]);
    let lambda = 0.5 * (a.load_factor + b.load_factor);
    let external: f64 = joints
        .iter()
        .map(|(j, f)| lambda * f * (b.joint_displacements[*j][0] - a.joint_displacements[*j][0]))
        .sum();
    let plastic: f64 = a
        .hinges
        .iter()
        .zip(&b.hinges)
        .map(|(x, y)| 0.5 * (x.force + y.force) * (y.plastic_deformation - x.plastic_deformation))
        .sum();
    (external, plastic)
}

#[test]
fn cantilever_with_a_base_hinge_collapses_at_mp_over_the_sum_of_mass_times_height() {
    // Column along Z, masses m1 at z1 and m2 at z2, base hinge Mp. Accel UX
    // with sf = -2 loads the masses with +2 m_j in X. Mechanism: rotation of
    // the whole column about the base, so lambda * 2 (m1 z1 + m2 z2) = Mp.
    let (mp, z1, z2, m1, m2, sf) = (150.0, 2.0, 5.0, 3.0, 1.5, -2.0);
    let mut m = bare_model([true, false, true, false, true, false]);
    add_material(&mut m, E, 0.0);
    add_section(&mut m, "SEC", 1.0e2, IC);
    m.add_point("B", 0.0, 0.0, 0.0);
    m.add_point("J1", 0.0, 0.0, z1);
    m.add_point("J2", 0.0, 0.0, z2);
    m.points["B"].restraint = [true; 6];
    for (j, mass) in [("J1", m1), ("J2", m2)] {
        m.points[j].mass = [mass, mass, mass, 0.0, 0.0, 0.0];
    }
    m.add_frame("C1", "B", "J1", "SEC");
    m.add_frame("C2", "J1", "J2", "SEC");
    m.hinge_props.insert("H".into(), epp_moment("H", mp));
    m.frames["C1"].hinges = vec![hinge_at("H", 0.0)];
    let mut push = nl_case("PUSH", &[("UX", sf)]);
    pushover(&mut push, "J2", 0.5);
    m.load_cases.insert("PUSH".into(), push);
    analyze_model(&mut m).unwrap();
    let case = &m.case_results["PUSH"];

    let f1 = -sf * m1;
    let f2 = -sf * m2;
    let lambda_c = mp / (f1 * z1 + f2 * z2);
    // The first event: elastic top deflection of a cantilever under the two
    // forces at the collapse load.
    let first = case
        .static_steps
        .iter()
        .find(|s| s.hinges[0].force.abs() >= mp * (1.0 - 1e-9))
        .expect("the hinge yields");
    close(first.load_factor, lambda_c, 1e-6, "collapse load factor");
    let ei = E * IC;
    let delta2 =
        lambda_c * (f1 * z1 * z1 * (3.0 * z2 - z1) / (6.0 * ei) + f2 * z2.powi(3) / (3.0 * ei));
    close(
        first.joint_displacements["J2"][0],
        delta2,
        1e-6,
        "top deflection at the first event",
    );
    // Uniqueness theorem: the reported mechanism is the base hinge alone; the
    // work of the load on it equals the plastic work, and the yield bound
    // holds at every hinge.
    let last = case.static_steps.last().unwrap();
    close(last.load_factor, lambda_c, 1e-6, "plateau load factor");
    let (external, plastic) = plateau_work(case, &[("J1", f1), ("J2", f2)]);
    assert!(plastic > 0.0, "the base hinge must deform on the plateau");
    close(external, plastic, 1e-6, "virtual work of the mechanism");
    for h in &last.hinges {
        assert!(h.force.abs() <= mp * (1.0 + 1e-9), "{h:?}");
    }
    // Base shear equals the applied total.
    close(
        -last.base_reactions[0],
        last.load_factor * (f1 + f2),
        1e-9,
        "base shear",
    );
}

#[test]
fn portal_pushed_by_accel_collapses_at_four_mp_over_the_sum_of_mass_times_height() {
    // Fixed portal h = 3, b = 6, Mp = 100 on all member ends, beam strong.
    // Masses m1, m2 on the top joints: lambda (m1 + m2) h = 4 Mp.
    let (mp, h, b, m1, m2) = (100.0, 3.0, 6.0, 2.0, 3.0);
    let mut m = bare_model([true, false, true, false, true, false]);
    add_material(&mut m, E, 0.0);
    add_section(&mut m, "SEC", 1.0e2, IC);
    for (name, x, z) in [
        ("B1", 0.0, 0.0),
        ("B2", b, 0.0),
        ("T1", 0.0, h),
        ("T2", b, h),
    ] {
        m.add_point(name, x, 0.0, z);
    }
    m.points["B1"].restraint = [true; 6];
    m.points["B2"].restraint = [true; 6];
    m.points["T1"].mass = [m1, m1, m1, 0.0, 0.0, 0.0];
    m.points["T2"].mass = [m2, m2, m2, 0.0, 0.0, 0.0];
    m.add_frame("C1", "B1", "T1", "SEC");
    m.add_frame("C2", "B2", "T2", "SEC");
    m.add_frame("BM", "T1", "T2", "SEC");
    m.hinge_props.insert("COL".into(), epp_moment("COL", mp));
    m.hinge_props
        .insert("BEAM".into(), epp_moment("BEAM", 3.0 * mp));
    for c in ["C1", "C2"] {
        m.frames[c].hinges = vec![hinge_at("COL", 0.0), hinge_at("COL", 1.0)];
    }
    m.frames["BM"].hinges = vec![hinge_at("BEAM", 0.0), hinge_at("BEAM", 1.0)];
    let mut push = nl_case("PUSH", &[("UX", -1.0)]);
    pushover(&mut push, "T1", 0.2);
    m.load_cases.insert("PUSH".into(), push);
    analyze_model(&mut m).unwrap();
    let case = &m.case_results["PUSH"];
    let lambda_c = 4.0 * mp / (h * (m1 + m2));
    let last = case.static_steps.last().unwrap();
    close(last.load_factor, lambda_c, 1e-6, "collapse load factor");
    let (external, plastic) = plateau_work(case, &[("T1", m1), ("T2", m2)]);
    close(
        external,
        plastic,
        1e-6,
        "virtual work of the sway mechanism",
    );
    for (i, hinge) in last.hinges.iter().enumerate() {
        let strength = if i < 4 { mp } else { 3.0 * mp };
        assert!(hinge.force.abs() <= strength * (1.0 + 1e-9), "{hinge:?}");
    }
    // The four column ends carry the plastic moment in the mechanism.
    for hinge in &last.hinges[..4] {
        close(hinge.force.abs(), mp, 1e-6, "column end moment");
    }
}

// ---------------------------------------------------------------------------
// Negative tests
// ---------------------------------------------------------------------------

#[test]
fn refuses_rotational_acceleration_and_a_pattern_named_like_an_acceleration() {
    let mut m = chain(10.0, 10.0, 1.0, 1.0);
    m.load_cases
        .insert("PUSH".into(), nl_case("PUSH", &[("U4", 1.0)]));
    assert_kind(&refused(&mut m), true, &["PUSH", "rotational", "U4"]);

    let mut m = chain(10.0, 10.0, 1.0, 1.0);
    m.load_patterns.insert(
        "UX".into(),
        LoadPattern {
            name: "UX".into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
    m.load_cases
        .insert("PUSH".into(), nl_case("PUSH", &[("UX", 1.0)]));
    assert_kind(&refused(&mut m), true, &["PUSH", "'UX'", "load pattern"]);

    // Linear static cases still refuse acceleration loads.
    let mut m = chain(10.0, 10.0, 1.0, 1.0);
    let mut lin = LoadCase::linear_static("LIN", "P", 1.0);
    lin.loads[0].load_name = "UX".into();
    m.load_cases.insert("LIN".into(), lin);
    assert_kind(&refused(&mut m), true, &["LIN", "not implemented"]);
}

#[test]
fn refuses_a_mode_load_that_has_no_usable_modal_case() {
    // Mode number out of range, and mode 0.
    for mode in [0, 3] {
        let mut m = chain(10.0, 10.0, 1.0, 1.0);
        m.load_cases
            .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 2));
        m.load_cases
            .insert("PUSH".into(), mode_case("PUSH", "MODAL", mode, 1.0));
        assert_kind(&refused(&mut m), false, &["PUSH", "Mode load", "MODAL"]);
    }
    // Missing modal case.
    let mut m = chain(10.0, 10.0, 1.0, 1.0);
    m.load_cases
        .insert("PUSH".into(), mode_case("PUSH", "NOPE", 1, 1.0));
    assert_kind(&refused(&mut m), false, &["PUSH", "NOPE"]);
    // Not a Modal case.
    let mut m = chain(10.0, 10.0, 1.0, 1.0);
    m.load_cases
        .insert("MODAL".into(), LoadCase::linear_static("MODAL", "P", 1.0));
    m.load_cases
        .insert("PUSH".into(), mode_case("PUSH", "MODAL", 1, 1.0));
    assert_kind(
        &refused(&mut m),
        true,
        &["PUSH", "MODAL", "not a Modal case"],
    );
    // Modal case not flagged to run.
    let mut m = chain(10.0, 10.0, 1.0, 1.0);
    let mut modal = LoadCase::modal_eigen("MODAL", 2);
    modal.run_flag = false;
    m.load_cases.insert("MODAL".into(), modal);
    m.load_cases
        .insert("PUSH".into(), mode_case("PUSH", "MODAL", 1, 1.0));
    assert_kind(
        &refused(&mut m),
        false,
        &["PUSH", "MODAL", "not flagged to run"],
    );
    // Mode loads without a modal case, and a modal case without Mode loads.
    let mut m = chain(10.0, 10.0, 1.0, 1.0);
    let mut c = mode_case("PUSH", "MODAL", 1, 1.0);
    c.modal_case = None;
    m.load_cases.insert("PUSH".into(), c);
    assert_kind(&refused(&mut m), false, &["PUSH", "no modal case"]);
    let mut m = chain(10.0, 10.0, 1.0, 1.0);
    m.load_cases
        .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 2));
    let mut c = nl_case("PUSH", &[("UX", 1.0)]);
    c.modal_case = Some("MODAL".into());
    m.load_cases.insert("PUSH".into(), c);
    assert_kind(&refused(&mut m), true, &["PUSH", "no Mode load"]);
    // Mode loads on a linear static case.
    let mut m = chain(10.0, 10.0, 1.0, 1.0);
    m.load_cases
        .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 2));
    let mut c = mode_case("LIN", "MODAL", 1, 1.0);
    c.case_type = LoadCaseType::LinearStatic;
    m.load_cases.insert("LIN".into(), c);
    assert_kind(&refused(&mut m), true, &["LIN", "nonlinear static"]);
}

#[test]
fn refuses_mode_loads_mixed_with_other_loads() {
    for other in ["P", "UX"] {
        let mut m = chain(10.0, 10.0, 1.0, 1.0);
        m.load_cases
            .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 2));
        let mut c = mode_case("PUSH", "MODAL", 1, 1.0);
        c.loads.push(LoadCaseAssignment {
            load_name: other.into(),
            scale_factor: 1.0,
        });
        m.load_cases.insert("PUSH".into(), c);
        assert_kind(&refused(&mut m), true, &["PUSH", "mixes Mode loads"]);
    }
}

#[test]
fn refuses_accel_and_mode_loads_in_a_model_with_area_objects() {
    for mode in [false, true] {
        let mut m = chain(10.0, 10.0, 1.0, 1.0);
        for (n, x, y) in [("A3", 1.0, 1.0), ("A4", 2.0, 1.0)] {
            m.add_point(n, x, y, 0.0);
        }
        m.areas.insert(
            "A1".into(),
            AreaObj::new(
                "A1",
                vec!["J1".into(), "J2".into(), "A4".into(), "A3".into()],
                "ASEC",
            ),
        );
        m.area_sections.insert(
            "ASEC".into(),
            AreaSection {
                name: "ASEC".into(),
                material: "MAT".into(),
                ..AreaSection::default()
            },
        );
        if mode {
            m.load_cases
                .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 1));
            m.load_cases
                .insert("PUSH".into(), mode_case("PUSH", "MODAL", 1, 1.0));
        } else {
            m.load_cases
                .insert("PUSH".into(), nl_case("PUSH", &[("UX", 1.0)]));
        }
        let e = refused(&mut m);
        assert_kind(&e, true, &["area, solid or link"]);
    }
}
