//! Wave 6, D68: the "use conjugate displacement" option of displacement
//! control on the two nonlinear static paths.
//!
//! The hinge path (frame hinges) borders each step with the conjugate
//! displacement, the work of the case's own load pattern; the non-hinge path
//! (Wen link) scales the load until the monitored displacement equals the
//! target and ignores the option. These tests pin both behaviours.
//!
//! References: closed forms derived in the comments (kN, m); CSI Analysis
//! Reference v19 p. 430 (conjugate displacement decides the load direction,
//! the monitored displacement sets the target); CSI Analysis Reference v24
//! p. 432 (a forward increment has the sign of the displacement in the first
//! load step); the v19 manual is silent on that sign rule. Example 6-008
//! (Wen link, k = 100 kip/in, yield = 50 kip, ratio = 0.1, exp = 1) gives the
//! link model of the first test, which is built here independently.
//!
//! Not covered: that the two `Problem` rows of the hinge solver differ; that
//! needs crate-internal access and is left to the unit tests of `hinge.rs`.
use sap_model::hinge::*;
use sap_model::*;
use sap_solver::analyze_model;

fn close(actual: f64, expected: f64, tolerance: f64, what: &str) {
    assert!(
        (actual - expected).abs() <= tolerance * (1.0 + expected.abs()),
        "{what}: got {actual}, expected {expected} (tolerance {tolerance})"
    );
}

// ---------------------------------------------------------------------------
// 1. A model without hinges: the option changes nothing.
// ---------------------------------------------------------------------------

const K: f64 = 100.0;
const YIELD: f64 = 50.0;
const RATIO: f64 = 0.1;

/// One axial Wen link (exponent 1) from fixed joint I to joint J, loaded by a
/// unit force at J; nonlinear static case "S" with displacement control on
/// the U1 of J.
fn wen_model(conjugate: bool, scale: f64, target: f64) -> SapModel {
    let mut m = SapModel::default();
    m.load_patterns["DEAD"].self_wt_multiplier = 0.;
    m.load_cases.clear();
    m.active_dofs = [true, false, false, false, false, false];
    m.add_point("I", 0., 0., 0.);
    m.add_point("J", 3., 0., 0.);
    m.points["I"].restraint = [true; 6];
    m.points["J"].loads_force.push(PointLoadForce {
        load_pat: "DEAD".into(),
        csys: "GLOBAL".into(),
        force: [1., 0., 0., 0., 0., 0.],
    });
    let mut prop = LinkProp {
        name: "W".into(),
        link_type: LinkPropType::PlasticWen,
        ..LinkProp::default()
    };
    prop.dof_nonlinear[0] = true;
    prop.ke[0] = 13.;
    prop.k_nl[0] = K;
    prop.param1[0] = YIELD;
    prop.param2[0] = RATIO;
    prop.param3[0] = 1.;
    m.link_props.insert("W".into(), prop);
    m.links.insert(
        "L".into(),
        LinkObj {
            name: "L".into(),
            point_i: "I".into(),
            point_j: Some("J".into()),
            prop: "W".into(),
            angle_deg: 0.,
            selected: false,
        },
    );
    let mut case = LoadCase::linear_static("S", "DEAD", scale);
    case.case_type = LoadCaseType::NonlinearStatic;
    case.convergence_tol = 1e-12;
    case.max_iterations = 200;
    case.displacement_control = Some(DisplacementControl {
        target,
        monitored: Some(("J".into(), 0)),
        conjugate,
    });
    m.load_cases.insert("S".into(), case);
    m
}

#[test]
fn without_hinges_the_conjugate_option_changes_nothing() {
    // The non-hinge path scales the load until the monitored displacement
    // equals the target (`load_path_factor`), and never reads `conjugate`.
    // The final state is therefore identical, to rounding.
    // Closed form of the link (Example 6-008 model, units kip, in):
    // z = 1 - exp(-k d / yield), f = ratio k d + (1 - ratio) yield z, in the
    // direction of the load.
    for (scale, target) in [(1.0, 0.8), (1.0, 9.6), (-1.0, 0.8)] {
        let mut a = wen_model(false, scale, target);
        let mut b = wen_model(true, scale, target);
        analyze_model(&mut a).unwrap();
        analyze_model(&mut b).unwrap();
        let (ra, rb) = (&a.case_results["S"], &b.case_results["S"]);
        for dof in 0..6 {
            for joint in ["I", "J"] {
                close(
                    rb.joint_displacements[joint][dof],
                    ra.joint_displacements[joint][dof],
                    1e-12,
                    "joint displacement",
                );
            }
            close(
                rb.joint_reactions["I"][dof],
                ra.joint_reactions["I"][dof],
                1e-12,
                "joint reaction",
            );
        }
        close(rb.link_forces["L"][0], ra.link_forces["L"][0], 1e-12, "link force");
        close(
            rb.displacement_control_factor.unwrap(),
            ra.displacement_control_factor.unwrap(),
            1e-12,
            "load factor",
        );
        let d = scale.signum() * target;
        let z = 1.0 - (-K / YIELD * target).exp();
        let f = scale.signum() * (RATIO * K * target + (1. - RATIO) * YIELD * z);
        close(ra.joint_displacements["J"][0], d, 1e-10, "target reached");
        close(ra.link_forces["L"][0], f, 1e-10, "closed-form link force");
        close(rb.link_forces["L"][0], f, 1e-10, "closed-form link force, conjugate");
    }
}

// ---------------------------------------------------------------------------
// 2. and 3. Hinge path where conjugate and monitored displacements are not
//    proportional.
// ---------------------------------------------------------------------------
//
// Cantilever column along Z (kN, m): base joint fixed, joints at mid-height
// (z = h/2) and top (z = h), h = 3 m; EI = E I = 2e8 kN/m2 * 1e-4 m4 =
// 2e4 kN m2; no shear deformation (As = 0), axially stiff. The case load is
// one pattern with 1 kN in +X at the mid joint and 1 kN at the top, scaled
// by the case factor, so the load factor lambda multiplies both.
//
// Elastic tip deflection per unit lambda (beam theory, loads at a = h/2 and
// at h):
//   load at h:   h^3 / (3 EI)
//   load at a:   a^2 (3 h - a) / (6 EI) = (h^2/4)(5h/2)/(6 EI) = 5 h^3 / (48 EI)
//   sum:         c = 7 h^3 / (16 EI) = 7 * 27 / (16 * 2e4) = 5.90625e-4 m/kN
// Base moment per unit lambda: 1 * h/2 + 1 * h = 1.5 h = 4.5 kN m per unit.
// The elastic-perfectly-plastic M3 hinge at the base has Mp = 9 kN m, so it
// yields at lambda_y = Mp / (1.5 h) = 2.0, with tip deflection
// u_y = lambda_y c = 1.18125e-3 m.
// On the plateau the hinge carries Mp, so lambda stays at Mp/(1.5 h) = 2.0
// and the tip moves by rigid rotation about the base:
// u = lambda_y c + h theta_p, theta_p the plastic rotation (rad).
// The conjugate displacement of the case load is u_mid + u_top and the
// monitored one is u_top. Elastically u_mid = lambda (a^3/(3 EI) +
// a^2 (3h - a)/(6 EI)) = lambda 3.9375 / EI = u_top / 3 (u_top = 11.8125 /
// EI per unit lambda). On the plateau the rigid rotation adds (h/2) theta_p
// to u_mid and h theta_p to u_top, so u_mid / u_top climbs toward 1/2: the
// two measures are not proportional.
const H: f64 = 3.0;
const EP: f64 = 2.0e8;
const IC: f64 = 1.0e-4;
const MP: f64 = 9.0;
const TARGET: f64 = 0.01; // m, monitored top displacement (magnitude)

fn tip_flexibility() -> f64 {
    7.0 * H.powi(3) / (16.0 * EP * IC) // m per kN, tip, unit lambda
}

fn cantilever(conjugate: bool, scale: f64) -> SapModel {
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    m.active_dofs = [true, false, true, false, true, false];
    let mut material = Material::default_steel_kip_in();
    material.name = "STEEL".into();
    material.e = EP;
    material.g = EP / 2.6;
    material.u = 0.3;
    material.unit_weight = 0.0;
    material.unit_mass = 0.0;
    m.materials.insert("STEEL".into(), material);
    m.frame_sections.insert(
        "COL".into(),
        FrameSection {
            name: "COL".into(),
            material: "STEEL".into(),
            prop_type: FramePropType::General,
            area: 1.0e2,
            i33: IC,
            i22: IC,
            torsional_j: 2.0 * IC,
            as2: 0.0,
            as3: 0.0,
            modifiers: [1.0; 8],
            ..FrameSection::default()
        },
    );
    m.add_point("B", 0., 0., 0.);
    m.add_point("M", 0., 0., 0.5 * H);
    m.add_point("T", 0., 0., H);
    m.points["B"].restraint = [true; 6];
    m.add_frame("C1", "B", "M", "COL");
    m.add_frame("C2", "M", "T", "COL");
    let mut prop = HingeProp::symmetric(
        "BASE",
        HingeDof::M3,
        [
            HingePoint::new(1.0, 0.0),
            HingePoint::new(1.0, 0.25),
            HingePoint::new(1.0, 0.35),
            HingePoint::new(1.0, 0.5),
        ],
    );
    prop.force_sf = MP;
    prop.force_sf_negative = MP;
    m.hinge_props.insert("BASE".into(), prop);
    m.frames["C1"].hinges = vec![FrameHinge {
        prop: "BASE".into(),
        rel_dist: 0.0,
    }];
    m.load_patterns.insert(
        "LAT".into(),
        LoadPattern {
            name: "LAT".into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
    for joint in ["M", "T"] {
        m.points[joint].loads_force.push(PointLoadForce {
            load_pat: "LAT".into(),
            csys: "GLOBAL".into(),
            force: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        });
    }
    let mut push = LoadCase::linear_static("PUSH", "LAT", scale);
    push.case_type = LoadCaseType::NonlinearStatic;
    push.displacement_control = Some(DisplacementControl {
        target: TARGET,
        monitored: Some(("T".into(), 0)),
        conjugate,
    });
    push.nl_static.results_saved = ResultsSaved::MultipleStates {
        min_saved: 20,
        max_saved: 100,
        positive_only: false,
    };
    m.load_cases.insert("PUSH".into(), push);
    m
}

/// Checks every saved state of `case` against the closed form for the
/// direction `sign` (+1 or -1) and returns the last state.
fn check_states<'a>(case: &'a CaseResults, sign: f64, what: &str) -> &'a NonlinearStaticStep {
    let c = tip_flexibility();
    let lambda_y = MP / (1.5 * H); // 2.0
    let mut yielded = false;
    for (i, s) in case.static_steps.iter().enumerate() {
        // The load factor multiplies the loads of the case, which already
        // carry the case scale factor (sign); it stays positive.
        let lambda = s.load_factor;
        let u_top = sign * s.joint_displacements["T"][0];
        let hinge = &s.hinges[0];
        let theta_p = hinge.plastic_deformation.abs();
        assert!(
            lambda <= lambda_y * (1.0 + 1e-9),
            "{what} step {i}: lambda {lambda} above Mp/(1.5 h) = {lambda_y}"
        );
        // Tip deflection: elastic part plus rigid rotation at the plastic hinge.
        close(u_top, lambda * c + H * theta_p, 1e-9, "tip deflection");
        // Hinge moment is the base moment 1.5 h lambda.
        close(hinge.force.abs(), 1.5 * H * lambda, 1e-9, "hinge moment");
        if theta_p > 1e-12 || yielded {
            yielded = true;
            close(lambda, lambda_y, 1e-9, "load factor on the plateau");
        }
    }
    assert!(yielded, "{what}: the hinge never yielded");
    case.static_steps.last().unwrap()
}

#[test]
fn hinge_path_conjugate_and_monitored_control_reach_the_same_state() {
    let mut a = cantilever(false, 1.0);
    let mut b = cantilever(true, 1.0);
    analyze_model(&mut a).unwrap();
    analyze_model(&mut b).unwrap();
    let (ca, cb) = (&a.case_results["PUSH"], &b.case_results["PUSH"]);
    let (la, lb) = (
        check_states(ca, 1.0, "monitored"),
        check_states(cb, 1.0, "conjugate"),
    );
    // Final state: the target magnitude, lambda = Mp/(1.5 h) = 2 on the
    // plateau, plastic rotation (u - lambda_y c)/h.
    let lambda_y = MP / (1.5 * H);
    let theta = (TARGET - lambda_y * tip_flexibility()) / H;
    for s in [la, lb] {
        close(s.joint_displacements["T"][0], TARGET, 1e-9, "target reached");
        close(s.load_factor, lambda_y, 1e-9, "final load factor");
        close(s.hinges[0].plastic_deformation.abs(), theta, 1e-9, "plastic rotation");
    }
    // The two controls end in the same state.
    close(lb.load_factor, la.load_factor, 1e-12, "load factor");
    for dof in 0..6 {
        for joint in ["B", "M", "T"] {
            close(
                lb.joint_displacements[joint][dof],
                la.joint_displacements[joint][dof],
                1e-12,
                "joint displacement",
            );
        }
        close(
            lb.joint_reactions["B"][dof],
            la.joint_reactions["B"][dof],
            1e-12,
            "base reaction",
        );
    }
    close(
        lb.hinges[0].plastic_deformation,
        la.hinges[0].plastic_deformation,
        1e-12,
        "plastic rotation",
    );
}

#[test]
fn a_push_toward_the_negative_side_is_followed_to_the_target_magnitude() {
    // v24 Analysis Reference p. 432: a forward increment has the sign of the
    // monitored displacement in the first load step; the v19 manual is silent.
    // The case factor -1 pushes the top joint toward -X while the target is
    // the positive magnitude 0.01 m. The closed form of the cantilever above
    // is mirrored: the load factor on the case loads (scale -1) is +2 on the
    // plateau, so the applied load is -2 per unit pattern and u_top = -0.01 m.
    for conjugate in [false, true] {
        let mut m = cantilever(conjugate, -1.0);
        analyze_model(&mut m).unwrap();
        let case = &m.case_results["PUSH"];
        let last = check_states(case, -1.0, "negative push");
        let lambda_y = MP / (1.5 * H);
        close(last.joint_displacements["T"][0], -TARGET, 1e-9, "u runs to -target");
        close(last.load_factor, lambda_y, 1e-9, "load factor Mp/(1.5 h)");
        close(
            last.hinges[0].plastic_deformation.abs(),
            (TARGET - lambda_y * tip_flexibility()) / H,
            1e-9,
            "plastic rotation",
        );
        // No state overshoots the target magnitude.
        for s in &case.static_steps {
            assert!(s.joint_displacements["T"][0] >= -TARGET * (1.0 + 1e-9));
        }
    }
}
