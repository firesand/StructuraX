//! Frame plastic hinges in nonlinear static (pushover) cases.
//!
//! References: CSI Software Verification Example 1-026 (the published SAP2000
//! points), and closed forms evaluated inside the tests (plastic collapse
//! loads and elastic-plastic beam and frame solutions). Every model is built
//! in code. Tolerances: 1e-9 relative against closed forms unless noted;
//! +-5e-5 against printed SAP2000 values; a diaphragm or other penalty
//! constraint limits the accuracy to 1e-4 relative.
use sap_model::hinge::*;
use sap_model::*;
use sap_solver::{analyze_model, SolverError};
use sap_units::Units;

fn close(a: f64, b: f64, rel: f64, what: &str) {
    let scale = a.abs().max(b.abs()).max(1e-300);
    assert!(
        (a - b).abs() <= rel * scale,
        "{what}: got {a}, expected {b} (relative tolerance {rel})"
    );
}

fn close_abs(a: f64, b: f64, tol: f64, what: &str) {
    assert!(
        (a - b).abs() <= tol,
        "{what}: got {a}, expected {b} (tolerance {tol})"
    );
}

fn bare_model(active: [bool; 6]) -> SapModel {
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    m.active_dofs = active;
    m
}

fn add_pattern(m: &mut SapModel, name: &str, self_weight: f64) {
    m.load_patterns.insert(
        name.into(),
        LoadPattern {
            name: name.into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: self_weight,
        },
    );
}

fn add_material(m: &mut SapModel, name: &str, e: f64, g: f64, unit_weight: f64) {
    let mut material = Material::default_steel_kip_in();
    material.name = name.into();
    material.e = e;
    material.g = g;
    material.u = e / (2.0 * g) - 1.0;
    material.unit_weight = unit_weight;
    material.unit_mass = unit_weight / 9.80665;
    m.materials.insert(name.into(), material);
}

#[allow(clippy::too_many_arguments)]
fn add_section(m: &mut SapModel, name: &str, material: &str, a: f64, i33: f64, i22: f64, as2: f64) {
    m.frame_sections.insert(
        name.into(),
        FrameSection {
            name: name.into(),
            material: material.into(),
            prop_type: FramePropType::General,
            area: a,
            i33,
            i22,
            torsional_j: i33 + i22,
            as2,
            as3: 0.0,
            modifiers: [1.0; 8],
            ..FrameSection::default()
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

fn nl_case(name: &str, pattern: &str, sf: f64, initial: Option<&str>) -> LoadCase {
    let mut c = LoadCase::linear_static(name, pattern, sf);
    c.case_type = LoadCaseType::NonlinearStatic;
    c.initial_case = initial.map(str::to_string);
    c
}

fn push_control(c: &mut LoadCase, joint: &str, dof: usize, target: f64) {
    c.displacement_control = Some(DisplacementControl {
        target,
        monitored: Some((joint.into(), dof)),
        conjugate: false,
    });
}

fn hinge_at(prop: &str, rel: f64) -> FrameHinge {
    FrameHinge {
        prop: prop.into(),
        rel_dist: rel,
    }
}

/// An elastic-perfectly-plastic moment curve: B (1, 0), C (1, dc), D (1, dd), E (1, de).
fn epp_curve(dc: f64, dd: f64, de: f64) -> [HingePoint; 4] {
    [
        HingePoint::new(1.0, 0.0),
        HingePoint::new(1.0, dc),
        HingePoint::new(1.0, dd),
        HingePoint::new(1.0, de),
    ]
}

fn epp_moment(name: &str, mp: f64) -> HingeProp {
    let mut p = HingeProp::symmetric(name, HingeDof::M3, epp_curve(0.25, 0.35, 0.5));
    p.force_sf = mp;
    p.force_sf_negative = mp;
    p
}

fn step_u(case: &CaseResults, step: usize, joint: &str, dof: usize) -> f64 {
    case.static_steps[step].joint_displacements[joint][dof]
}

// ---------------------------------------------------------------------------
// Example 1-026: cantilever with a moment hinge and a shear hinge at the root
// (kip, in). E = 3600, G = 1500, A = 216, I33 = 5832, As2 = 180, L = 24.
// ---------------------------------------------------------------------------

const E: f64 = 3600.0;
const G: f64 = 1500.0;
const A: f64 = 216.0;
const I33: f64 = 5832.0;
const AS2: f64 = 180.0;
const L: f64 = 24.0;

fn moment_1_026() -> HingeProp {
    HingeProp::symmetric(
        "Moment",
        HingeDof::M3,
        [
            HingePoint::new(1440.0, 0.0),
            HingePoint::new(1920.0, 0.04),
            HingePoint::new(480.0, 0.04),
            HingePoint::new(480.0, 0.4),
        ],
    )
}

fn shear_1_026() -> HingeProp {
    HingeProp::symmetric(
        "Shear",
        HingeDof::V2,
        [
            HingePoint::new(70.0, 0.0),
            HingePoint::new(80.0, 0.4),
            HingePoint::new(90.0, 0.8),
            HingePoint::new(100.0, 1.2),
        ],
    )
}

/// `overwrite`: the frame hinge overwrite of the object (Model B subdivides).
fn example_1_026(overwrite: Option<FrameHingeOverwrite>, saved: ResultsSaved) -> SapModel {
    let mut m = bare_model([true, false, true, false, true, false]);
    add_material(&mut m, "CONC", E, G, 0.0);
    add_section(&mut m, "BEAM", "CONC", A, I33, 2592.0, AS2);
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", L, 0.0, 0.0);
    m.points["1"].restraint = [true; 6];
    m.add_frame("1", "1", "2", "BEAM");
    add_pattern(&mut m, "P", 0.0);
    joint_force(&mut m, "2", "P", [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);
    m.hinge_props.insert("Moment".into(), moment_1_026());
    m.hinge_props.insert("Shear".into(), shear_1_026());
    m.frames["1"].hinges = vec![hinge_at("Moment", 0.0), hinge_at("Shear", 0.0)];
    m.frames["1"].hinge_overwrite = overwrite;
    let mut push = nl_case("PUSH", "P", 1.0, None);
    push_control(&mut push, "2", 2, 2.0);
    push.nl_static.results_saved = saved;
    m.load_cases.insert("PUSH".into(), push);
    m
}

/// Elastic tip deflection per unit load, and tip rotation per unit load.
fn c_elastic() -> f64 {
    L.powi(3) / (3.0 * E * I33) + L / (G * AS2)
}

fn r_elastic() -> f64 {
    L * L / (2.0 * E * I33)
}

const MULTI_6_10: ResultsSaved = ResultsSaved::MultipleStates {
    min_saved: 6,
    max_saved: 10,
    positive_only: false,
};

fn model_b_overwrite() -> Option<FrameHingeOverwrite> {
    Some(FrameHingeOverwrite {
        auto_subdivide: true,
        relative_length: 0.02,
        do_not_drop_load: false,
    })
}

/// The tip state of a saved step: (P, Uz, Ry).
fn tip(case: &CaseResults, step: usize) -> (f64, f64, f64) {
    (
        case.static_steps[step].load_factor,
        step_u(case, step, "2", 2),
        step_u(case, step, "2", 4),
    )
}

/// Closed form of a state of Example 1-026 on the moment hinge's softening or
/// plateau branch, from the plastic rotation `qm` and shear jump `qs`.
fn tip_closed_form(p: f64, qm: f64, qs: f64) -> (f64, f64) {
    (p * c_elastic() + L * qm + qs, -(p * r_elastic() + qm))
}

#[test]
fn t1_example_1_026_a_follows_the_published_points() {
    let mut m = example_1_026(None, MULTI_6_10);
    analyze_model(&mut m).unwrap();
    let case = &m.case_results["PUSH"];
    // The schedule of saved states is a consequence of the saved-step rule
    // (min 6, max 10, target 2): grid points of 2/6, and events.
    let u: Vec<f64> = (0..case.static_steps.len())
        .map(|s| step_u(case, s, "2", 2))
        .collect();
    let want = [
        0.0, 0.0185021, 0.3518354, 0.5015857, 0.8349191, 1.1682524, 1.3846694, 1.7180027,
        1.7612291, 2.0,
    ];
    assert_eq!(u.len(), want.len(), "{u:?}");
    for (g, w) in u.iter().zip(want) {
        close_abs(*g, w, 5e-7, "saved Uz");
    }
    let k_lim = 0.1 * E * I33 / L;
    // Point 1: the moment hinge yields at 24 P = 1440.
    let (p1, uz1, ry1) = tip(case, 1);
    close(p1, 60.0, 1e-9, "P at point 1");
    let (cu, cr) = tip_closed_form(60.0, 0.0, 0.0);
    close(uz1, cu, 1e-9, "Uz at point 1");
    close(ry1, cr, 1e-9, "Ry at point 1");
    close_abs(uz1, 0.0185, 5e-5, "published Uz point 1");
    close_abs(ry1, -0.0008, 5e-5, "published Ry point 1");
    // Point 2: both hinges reach point C at P = 80 (moment 1920, shear 80).
    let (p2, uz2, ry2) = tip(case, 6);
    close(p2, 80.0, 1e-9, "P at point 2");
    let (cu, cr) = tip_closed_form(80.0, 0.04, 0.4);
    close(uz2, cu, 1e-9, "Uz at point 2");
    close(ry2, cr, 1e-9, "Ry at point 2");
    close_abs(uz2, 1.3847, 5e-5, "published Uz point 2");
    close_abs(ry2, -0.0411, 5e-5, "published Ry point 2");
    // Point 3: the moment hinge has dropped to 480 on the slope limit, P = 20.
    let qm3 = 0.04 + (1920.0 - 480.0) / k_lim;
    let (p3, uz3, ry3) = tip(case, 8);
    close(p3, 20.0, 1e-9, "P at point 3");
    let (cu, cr) = tip_closed_form(20.0, qm3, 0.4);
    close(uz3, cu, 1e-9, "Uz at point 3");
    close(ry3, cr, 1e-9, "Ry at point 3");
    close_abs(uz3, 1.7612, 5e-5, "published Uz point 3");
    close_abs(ry3, -0.0567, 5e-5, "published Ry point 3");
    // End: mechanism at P = 20 with the shear hinge unloaded at constant q.
    let (pf, uzf, ryf) = tip(case, 9);
    close(pf, 20.0, 1e-9, "P at the end");
    close(uzf, 2.0, 1e-12, "Uz at the end");
    let qm_end = (2.0 - 20.0 * c_elastic() - 0.4) / L;
    let (_, cr) = tip_closed_form(20.0, qm_end, 0.4);
    close(ryf, cr, 1e-9, "Ry at the end");
    let hinges = &case.hinge_results;
    close(
        hinges[0].plastic_deformation,
        qm_end,
        1e-9,
        "final q of the moment hinge",
    );
    close(
        hinges[1].plastic_deformation,
        0.4,
        1e-12,
        "final q of the shear hinge",
    );
    close(hinges[0].force, 480.0, 1e-9, "final moment");
    // The shear hinge unloaded to P = 20 below its backbone.
    close(hinges[1].force, 20.0, 1e-9, "final shear");
    assert_eq!(hinges[0].state, HingeState::DToE);
    assert_eq!(hinges[1].state, HingeState::CToD);
    close(
        case.displacement_control_factor.unwrap(),
        20.0,
        1e-12,
        "final load factor",
    );
    // Frame stations at the root equal the hinge forces at every saved state.
    for s in 0..case.static_steps.len() {
        let st = &case.static_steps[s];
        let root = &st.frame_forces["1"][0];
        close(
            root.m3,
            st.hinges[0].force,
            1e-9,
            "root M3 equals the moment hinge force",
        );
        close(
            root.v2,
            st.hinges[1].force,
            1e-9,
            "root V2 equals the shear hinge force",
        );
        // Equilibrium: base shear is the load.
        close(st.base_reactions[2], -st.load_factor, 1e-9, "base reaction");
    }
    // The final state fills the regular fields.
    close(case.joint_displacements["2"][2], 2.0, 1e-12, "final Uz");
}

#[test]
fn t2_example_1_026_b_snaps_back_to_the_published_point() {
    let mut m = example_1_026(model_b_overwrite(), MULTI_6_10);
    analyze_model(&mut m).unwrap();
    let case = &m.case_results["PUSH"];
    let u: Vec<f64> = (0..case.static_steps.len())
        .map(|s| step_u(case, s, "2", 2))
        .collect();
    let want = [
        0.0, 0.0185021, 0.3518354, 0.5015857, 0.8349191, 1.1682524, 1.3846694, 1.3701180,
        1.7034513, 2.0,
    ];
    assert_eq!(u.len(), want.len(), "{u:?}");
    for (g, w) in u.iter().zip(want) {
        close_abs(*g, w, 5e-7, "saved Uz");
    }
    // The slope limit uses the short element: 0.1 E I / (0.01 L), L_lim = 0.24 in.
    let k_lim = 0.1 * E * I33 / (0.01 * L);
    let qm3 = 0.04 + (1920.0 - 480.0) / k_lim;
    let (p3, uz3, ry3) = tip(case, 7);
    close(p3, 20.0, 1e-9, "P at point 3");
    let (cu, cr) = tip_closed_form(20.0, qm3, 0.4);
    close(uz3, cu, 1e-9, "Uz at point 3");
    close(ry3, cr, 1e-9, "Ry at point 3");
    close_abs(uz3, 1.3701, 5e-5, "published Uz point 3");
    close_abs(ry3, -0.0404, 5e-5, "published Ry point 3");
    assert!(uz3 < step_u(case, 6, "2", 2), "the displacement snaps back");
    // The created joint is not in any result.
    assert!(!case
        .joint_displacements
        .keys()
        .any(|k| k.starts_with("__fmesh")));
    for st in &case.static_steps {
        assert!(!st
            .joint_displacements
            .keys()
            .any(|k| k.starts_with("__fmesh")));
        assert!(!st.joint_reactions.keys().any(|k| k.starts_with("__fmesh")));
    }
    // Points 1 and 2 equal Model A (same elastic response through the short element).
    let mut a = example_1_026(None, MULTI_6_10);
    analyze_model(&mut a).unwrap();
    let ca = &a.case_results["PUSH"];
    for s in 0..=6 {
        // Points 1 and 2 (steps 1 and 6) agree to 1e-12. The grid states between
        // them differ by up to 1.1e-12 (measured): the 0.24 in element makes the
        // shear stiffness 1e6 times that of Model A, so roundoff reaches 1e-12.
        let tol = if matches!(s, 3..=5) { 1e-11 } else { 1e-12 };
        close(
            step_u(case, s, "2", 2),
            step_u(ca, s, "2", 2),
            tol,
            "B equals A before point 3",
        );
        close(
            step_u(case, s, "2", 4),
            step_u(ca, s, "2", 4),
            1e-9,
            "B equals A before point 3 (Ry)",
        );
    }
    // Positive increments only: the backward point is not saved.
    let mut m = example_1_026(
        model_b_overwrite(),
        ResultsSaved::MultipleStates {
            min_saved: 6,
            max_saved: 10,
            positive_only: true,
        },
    );
    analyze_model(&mut m).unwrap();
    let case = &m.case_results["PUSH"];
    let u: Vec<f64> = (0..case.static_steps.len())
        .map(|s| step_u(case, s, "2", 2))
        .collect();
    let want = [
        0.0, 0.0185021, 0.3518354, 0.5015857, 0.8349191, 1.1682524, 1.3846694, 1.7180027, 2.0,
    ];
    assert_eq!(u.len(), want.len(), "{u:?}");
    for (g, w) in u.iter().zip(want) {
        close_abs(*g, w, 5e-7, "saved Uz, positive increments only");
    }
}

#[test]
fn t3_conjugate_control_gives_the_same_states() {
    for overwrite in [None, model_b_overwrite()] {
        let mut a = example_1_026(overwrite.clone(), MULTI_6_10);
        analyze_model(&mut a).unwrap();
        let mut b = example_1_026(overwrite, MULTI_6_10);
        b.load_cases["PUSH"]
            .displacement_control
            .as_mut()
            .unwrap()
            .conjugate = true;
        analyze_model(&mut b).unwrap();
        let (ca, cb) = (&a.case_results["PUSH"], &b.case_results["PUSH"]);
        assert_eq!(ca.static_steps.len(), cb.static_steps.len());
        for s in 0..ca.static_steps.len() {
            close_abs(
                step_u(ca, s, "2", 2),
                step_u(cb, s, "2", 2),
                1e-10,
                "conjugate Uz",
            );
            close(
                ca.static_steps[s].load_factor,
                cb.static_steps[s].load_factor,
                1e-9,
                "conjugate P",
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Helpers for the closed-form models
// ---------------------------------------------------------------------------

/// The first saved step for which `pred` holds.
fn first_step(case: &CaseResults, pred: impl Fn(&NonlinearStaticStep) -> bool) -> usize {
    case.static_steps
        .iter()
        .position(pred)
        .expect("no saved state satisfies the condition")
}

fn h_force(step: &NonlinearStaticStep, hinge: usize) -> f64 {
    step.hinges[hinge].force
}

const MULTI_20_100: ResultsSaved = ResultsSaved::MultipleStates {
    min_saved: 20,
    max_saved: 100,
    positive_only: true,
};

fn refused(m: &mut SapModel) -> SolverError {
    analyze_model(m).expect_err("the analysis should be refused")
}

fn assert_unsupported(e: &SolverError, needle: &str) {
    match e {
        SolverError::UnsupportedInput(msg) => {
            assert!(msg.contains(needle), "message '{msg}' lacks '{needle}'")
        }
        other => panic!("expected UnsupportedInput containing '{needle}', got {other:?}"),
    }
}

/// kN, m portal: bases fixed, h = 3, b = 6, E = 2e8, I_c = 1e-4, I_b = 2e-4,
/// axially stiff. EPP moment hinges at both ends of every member: `mp_col` on
/// the columns, `mp_beam` on the beam. A pattern of 1 kN total (half on each
/// top joint) pushes in +X; displacement control on the top-left joint.
const H: f64 = 3.0;
const B: f64 = 6.0;
const EP: f64 = 2.0e8;
const IC: f64 = 1.0e-4;
const IB: f64 = 2.0e-4;

fn portal(mp_col: f64, mp_beam: f64) -> SapModel {
    portal_with_area(mp_col, mp_beam, 1.0e2)
}

fn portal_with_area(mp_col: f64, mp_beam: f64, axial: f64) -> SapModel {
    let mut m = bare_model([true, false, true, false, true, false]);
    add_material(&mut m, "STEEL", EP, EP / 2.6, 0.0);
    add_section(&mut m, "COLS", "STEEL", axial, IC, IC, 0.0);
    add_section(&mut m, "BEAMS", "STEEL", axial, IB, IB, 0.0);
    for (name, x, z) in [
        ("B1", 0.0, 0.0),
        ("B2", B, 0.0),
        ("T1", 0.0, H),
        ("T2", B, H),
    ] {
        m.add_point(name, x, 0.0, z);
    }
    m.points["B1"].restraint = [true; 6];
    m.points["B2"].restraint = [true; 6];
    m.add_frame("C1", "B1", "T1", "COLS");
    m.add_frame("C2", "B2", "T2", "COLS");
    m.add_frame("BM", "T1", "T2", "BEAMS");
    m.hinge_props
        .insert("COLM".into(), epp_moment("COLM", mp_col));
    m.hinge_props
        .insert("BEAMM".into(), epp_moment("BEAMM", mp_beam));
    for c in ["C1", "C2"] {
        m.frames[c].hinges = vec![hinge_at("COLM", 0.0), hinge_at("COLM", 1.0)];
    }
    m.frames["BM"].hinges = vec![hinge_at("BEAMM", 0.0), hinge_at("BEAMM", 1.0)];
    add_pattern(&mut m, "LAT", 0.0);
    for t in ["T1", "T2"] {
        joint_force(&mut m, t, "LAT", [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]);
    }
    let mut push = nl_case("PUSH", "LAT", 1.0, None);
    push_control(&mut push, "T1", 0, 0.2);
    push.nl_static.results_saved = MULTI_20_100;
    m.load_cases.insert("PUSH".into(), push);
    m
}

/// Plastic collapse of the portal, closed form (Mp on all four ends):
/// first the bases yield, then the tops, then the sway mechanism.
struct PortalClosedForm {
    h1: f64,
    d1: f64,
    h_u: f64,
    d2: f64,
}

fn portal_closed_form(mp: f64) -> PortalClosedForm {
    let (ic, ib) = (EP * IC / H, EP * IB / B);
    let k = ib / ic;
    let h1 = 2.0 * mp * (6.0 * k + 1.0) / (H * (3.0 * k + 1.0));
    let d1 = h1 * H * H * (2.0 + 3.0 * k) / (12.0 * ic * (1.0 + 6.0 * k));
    let h_u = 4.0 * mp / H;
    let d2 = d1 + (h_u - h1) * H * H * (ic + 2.0 * ib) / (12.0 * ib * ic);
    PortalClosedForm { h1, d1, h_u, d2 }
}

/// External work of the last two saved states against the plastic work of the
/// hinges (index list), on a plateau where forces are constant.
fn plateau_work(case: &CaseResults, joint: &str, lateral: f64) -> (f64, f64) {
    let n = case.static_steps.len();
    let (a, b) = (&case.static_steps[n - 2], &case.static_steps[n - 1]);
    let lambda = 0.5 * (a.load_factor + b.load_factor);
    let external =
        lambda * lateral * (b.joint_displacements[joint][0] - a.joint_displacements[joint][0]);
    let plastic: f64 = a
        .hinges
        .iter()
        .zip(&b.hinges)
        .map(|(x, y)| 0.5 * (x.force + y.force) * (y.plastic_deformation - x.plastic_deformation))
        .sum();
    (external, plastic)
}

// ---------------------------------------------------------------------------
// T4: fixed portal. Closed forms of the elastic-plastic sway: both bases yield
// together, then both tops, then the sway mechanism 4 Mp / h.
// ---------------------------------------------------------------------------

#[test]
fn t4_fixed_portal_follows_the_closed_form_collapse() {
    let mp = 100.0;
    let cf = portal_closed_form(mp);
    close(cf.h1, 116.666_666_666_666_67, 1e-12, "closed form H1");
    close(cf.d1, 0.009375, 1e-12, "closed form D1");
    close(cf.d2, 0.015, 1e-12, "closed form D2");
    let mut m = portal(mp, 300.0);
    analyze_model(&mut m).unwrap();
    let case = &m.case_results["PUSH"];
    // First event: both bases yield together.
    let s1 = first_step(case, |s| h_force(s, 0).abs() >= mp * (1.0 - 1e-9));
    let st = &case.static_steps[s1];
    close(st.load_factor, cf.h1, 1e-6, "H1");
    close(st.joint_displacements["T1"][0], cf.d1, 1e-6, "D1");
    close(
        h_force(st, 0).abs(),
        mp,
        1e-9,
        "base moment at the first event",
    );
    close(h_force(st, 2).abs(), mp, 1e-9, "both bases yield together");
    close(h_force(st, 1).abs(), 75.0, 1e-6, "top moment at H1");
    assert_eq!(st.hinges[0].plastic_deformation, 0.0);
    // Second event: both tops yield together; the collapse load is exact.
    let s2 = first_step(case, |s| h_force(s, 1).abs() >= mp * (1.0 - 1e-9));
    let st = &case.static_steps[s2];
    close(st.load_factor, cf.h_u, 1e-9, "collapse load 4 Mp / h");
    close(st.joint_displacements["T1"][0], cf.d2, 1e-6, "D2");
    close(h_force(st, 3).abs(), mp, 1e-9, "both tops yield together");
    // Plateau to the target: constant load, work equals plastic work.
    let last = case.static_steps.last().unwrap();
    close(last.load_factor, cf.h_u, 1e-9, "load on the plateau");
    close(last.joint_displacements["T1"][0], 0.2, 1e-9, "target");
    let (external, plastic) = plateau_work(case, "T1", 1.0);
    close(external, plastic, 1e-9, "H du = sum Mp |dq|");
    // The beam hinges never yield (Mp_b = 300).
    for s in &case.static_steps {
        assert!(h_force(s, 4).abs() < 300.0 && h_force(s, 5).abs() < 300.0);
        assert_eq!(s.hinges[4].plastic_deformation, 0.0);
        // Sum of the horizontal reactions equals the applied load.
        close(
            s.base_reactions[0],
            -s.load_factor,
            1e-9,
            "horizontal equilibrium",
        );
    }
}

#[test]
fn t4_diaphragm_variant_has_the_same_collapse_load() {
    // The collapse load 4 Mp / h does not depend on the elastic properties,
    // so the variant uses an ordinary axial area. A diaphragm is a penalty
    // constraint of 1e6 times the largest diagonal stiffness: with the nearly
    // rigid area of `portal` that is about 1e12 times the sway stiffness and
    // costs four digits of the load factor (4e-4 measured). With this area
    // the penalty is about 1e8 times the sway stiffness.
    let mut m = portal_with_area(100.0, 300.0, 1.0e-2);
    m.constraints.insert(
        "D1".into(),
        ConstraintDef {
            name: "D1".into(),
            constraint_type: ConstraintType::Diaphragm,
            axis: ConstraintAxis::Z,
            dofs: [true, true, false, false, false, true],
        },
    );
    for j in ["T1", "T2"] {
        m.points[j].constraints.push("D1".into());
    }
    analyze_model(&mut m).unwrap();
    let case = &m.case_results["PUSH"];
    let last = case.static_steps.last().unwrap();
    let h_u = 400.0 / 3.0;
    close(
        last.load_factor,
        h_u,
        1e-6,
        "collapse load factor with a diaphragm",
    );
    // The base shear is recovered from the column forces.
    close(
        -last.base_reactions[0],
        h_u,
        1e-6,
        "base shear through the penalty",
    );
    for hinge in &last.hinges {
        assert!(hinge.force.abs() <= 300.0 * (1.0 + 1e-9), "{hinge:?}");
    }
}

// ---------------------------------------------------------------------------
// T5: series tie. A column top and a beam end at the same joint have equal
// strength, so one of them deforms on the plateau: the lower index (project
// choice, documented; the path in load and displacement is unaffected).
// ---------------------------------------------------------------------------

#[test]
fn t5_series_tie_splits_deterministically_to_the_lower_index() {
    let mut m = portal(100.0, 100.0);
    analyze_model(&mut m).unwrap();
    let case = &m.case_results["PUSH"];
    let last = case.static_steps.last().unwrap();
    close(
        last.load_factor,
        4.0 * 100.0 / H,
        1e-9,
        "collapse load with tied hinges",
    );
    close(last.joint_displacements["T1"][0], 0.2, 1e-9, "target");
    // Indices: C1 (0 base, 1 top), C2 (2 base, 3 top), BM (4 left, 5 right).
    let q: Vec<f64> = last.hinges.iter().map(|h| h.plastic_deformation).collect();
    assert!(q[1] != 0.0 && q[3] != 0.0, "column tops deform: {q:?}");
    assert_eq!(q[4], 0.0, "the beam end at the left corner stays rigid");
    assert_eq!(q[5], 0.0, "the beam end at the right corner stays rigid");
    assert!(q[0] != 0.0 && q[2] != 0.0, "bases deform: {q:?}");
    // Both members' hinges carry the plastic moment at the corner.
    close(h_force(last, 4).abs(), 100.0, 1e-9, "beam end moment");
    close(h_force(last, 1).abs(), 100.0, 1e-9, "column top moment");
    let (external, plastic) = plateau_work(case, "T1", 1.0);
    close(external, plastic, 1e-9, "plastic work on the plateau");
}

// ---------------------------------------------------------------------------
// T6: fixed beam with a hinge at midspan that needs no joint; uniform load
// (kN, m; L = 6, EI = 2e4, Mp = 120).
// ---------------------------------------------------------------------------

const EI_BEAM: f64 = 2.0e8 * 1.0e-4;
const LB: f64 = 6.0;
const MP_BEAM: f64 = 120.0;

/// A fixed-fixed beam under a uniform downward load (1 kN/m in pattern W);
/// `split`: two objects with a joint at midspan, hinges at both ends of each,
/// else one object with hinges at 0, 0.5 and 1.
fn fixed_beam(split: bool) -> SapModel {
    let mut m = bare_model([true, false, true, false, true, false]);
    add_material(&mut m, "STEEL", 2.0e8, 2.0e8 / 2.6, 0.0);
    add_section(&mut m, "S", "STEEL", 1.0, 1.0e-4, 1.0e-4, 0.0);
    m.add_point("J1", 0.0, 0.0, 0.0);
    m.add_point("J2", LB, 0.0, 0.0);
    m.points["J1"].restraint = [true; 6];
    m.points["J2"].restraint = [true; 6];
    add_pattern(&mut m, "W", 0.0);
    m.hinge_props
        .insert("EPP".into(), epp_moment("EPP", MP_BEAM));
    let udl = |m: &mut SapModel, name: &str| {
        m.frames[name].loads_dist.push(FrameLoadDistributed {
            load_pat: "W".into(),
            my_type: 1,
            dir: 10,
            csys: "GLOBAL".into(),
            rel_dist1: 0.0,
            rel_dist2: 1.0,
            val1: 1.0,
            val2: 1.0,
        });
    };
    if split {
        m.add_point("M", LB / 2.0, 0.0, 0.0);
        m.add_frame("B1", "J1", "M", "S");
        m.add_frame("B2", "M", "J2", "S");
        for f in ["B1", "B2"] {
            m.frames[f].hinges = vec![hinge_at("EPP", 0.0), hinge_at("EPP", 1.0)];
            udl(&mut m, f);
        }
    } else {
        m.add_frame("B", "J1", "J2", "S");
        m.frames["B"].hinges = vec![
            hinge_at("EPP", 0.0),
            hinge_at("EPP", 0.5),
            hinge_at("EPP", 1.0),
        ];
        udl(&mut m, "B");
    }
    m
}

#[test]
fn t6_fixed_beam_with_an_interior_hinge_matches_the_closed_form() {
    let w1 = 12.0 * MP_BEAM / (LB * LB);
    close(w1, 40.0, 1e-12, "closed form w1");
    let mut m = fixed_beam(false);
    m.load_cases
        .insert("LOAD".into(), nl_case("LOAD", "W", 50.0, None));
    analyze_model(&mut m).unwrap();
    let case = &m.case_results["LOAD"];
    assert!(case.static_steps.is_empty(), "only the final state is kept");
    let h = &case.hinge_results;
    assert_eq!(h.len(), 3);
    // Ends: the plastic moment; plastic rotation (w - w1) L^3 / (24 EI); mid: w L^2/8 - Mp.
    let q_end = (50.0 - w1) * LB.powi(3) / (24.0 * EI_BEAM);
    close(q_end, 0.0045, 1e-12, "closed form end rotation");
    for i in [0, 2] {
        close(h[i].force.abs(), MP_BEAM, 1e-9, "end moment");
        close(
            h[i].plastic_deformation.abs(),
            q_end,
            1e-9,
            "end plastic rotation",
        );
    }
    close(
        h[1].force.abs(),
        50.0 * LB * LB / 8.0 - MP_BEAM,
        1e-9,
        "midspan moment",
    );
    close(h[1].force.abs(), 105.0, 1e-9, "midspan moment value");
    assert!(
        h[0].force * h[1].force < 0.0,
        "hogging at the ends, sagging at midspan"
    );
    assert_eq!(h[1].plastic_deformation, 0.0);
    close(
        case.base_reactions[2],
        50.0 * LB,
        1e-9,
        "vertical equilibrium",
    );
    // The station forces at the hinges are the hinge forces (interior station too).
    let stations = &case.frame_forces["B"];
    close(stations[0].m3, h[0].force, 1e-9, "M3 at the first hinge");
    close(
        stations.last().unwrap().m3,
        h[2].force,
        1e-9,
        "M3 at the last hinge",
    );
    let mid = stations
        .iter()
        .find(|s| (s.rel_dist - 0.5).abs() < 1e-9)
        .expect("a station at midspan");
    close(mid.m3, h[1].force, 1e-9, "M3 at the interior hinge");
}

#[test]
fn t6_two_object_beam_reaches_the_collapse_load_and_deflections() {
    let w_u = 16.0 * MP_BEAM / (LB * LB);
    let mut m = fixed_beam(true);
    let mut push = nl_case("PUSH", "W", 1.0, None);
    push_control(&mut push, "M", 2, 0.03);
    push.nl_static.results_saved = MULTI_20_100;
    m.load_cases.insert("PUSH".into(), push);
    analyze_model(&mut m).unwrap();
    let case = &m.case_results["PUSH"];
    // Indices: B1 (0 left end, 1 midspan side), B2 (2 midspan side, 3 right end).
    let s1 = first_step(case, |s| h_force(s, 0).abs() >= MP_BEAM * (1.0 - 1e-9));
    let st = &case.static_steps[s1];
    close(st.load_factor, 40.0, 1e-9, "w1");
    close(
        -st.joint_displacements["M"][2],
        0.00675,
        1e-9,
        "deflection at w1",
    );
    close(
        h_force(st, 3).abs(),
        MP_BEAM,
        1e-9,
        "both ends yield together",
    );
    let s2 = first_step(case, |s| h_force(s, 1).abs() >= MP_BEAM * (1.0 - 1e-9));
    let st = &case.static_steps[s2];
    close(st.load_factor, w_u, 1e-9, "w_u = 16 Mp / L^2");
    close(
        -st.joint_displacements["M"][2],
        0.018,
        1e-9,
        "deflection at w_u",
    );
    let last = case.static_steps.last().unwrap();
    close(last.load_factor, w_u, 1e-9, "plateau load");
    close(-last.joint_displacements["M"][2], 0.03, 1e-9, "target");
    // The two hinges at the midspan joint form a series tie: the lower index deforms.
    assert!(last.hinges[1].plastic_deformation != 0.0);
    assert_eq!(last.hinges[2].plastic_deformation, 0.0);
    close(
        last.base_reactions[2],
        w_u * LB,
        1e-9,
        "vertical equilibrium",
    );
}

// ---------------------------------------------------------------------------
// T7: continuation from a case that ended with yielded hinges.
// ---------------------------------------------------------------------------

#[test]
fn t7_continuation_equals_the_single_case_result() {
    let w_u = 16.0 * MP_BEAM / (LB * LB);
    // Single case: displacement control to 0.03 at midspan.
    let mut single = fixed_beam(true);
    let mut push = nl_case("PUSH", "W", 1.0, None);
    push_control(&mut push, "M", 2, 0.03);
    single.load_cases.insert("PUSH".into(), push);
    analyze_model(&mut single).unwrap();
    // Chain: load control to w = 50 (ends yielded), then displacement control.
    // At w = 50 the midspan deflection is 5 w L^4/(384 EI) - Mp L^2/(8 EI).
    let u50 = 5.0 * 50.0 * LB.powi(4) / (384.0 * EI_BEAM) - MP_BEAM * LB * LB / (8.0 * EI_BEAM);
    close(u50, 0.0151875, 1e-12, "closed form deflection at w = 50");
    let mut chain = fixed_beam(true);
    chain
        .load_cases
        .insert("L50".into(), nl_case("L50", "W", 50.0, None));
    let mut more = nl_case("MORE", "W", 10.0, Some("L50"));
    push_control(&mut more, "M", 2, 0.03 - u50);
    chain.load_cases.insert("MORE".into(), more);
    analyze_model(&mut chain).unwrap();
    let first = &chain.case_results["L50"];
    close(
        first.joint_displacements["M"][2],
        -u50,
        1e-9,
        "deflection after the first case",
    );
    let q_end = (50.0 - 40.0) * LB.powi(3) / (24.0 * EI_BEAM);
    close(
        first.hinge_results[0].plastic_deformation.abs(),
        q_end,
        1e-9,
        "carried end rotation",
    );
    let second = &chain.case_results["MORE"];
    // The case factor: the load on top of w = 50 divided by the case's own 10.
    close(
        second.displacement_control_factor.unwrap(),
        (w_u - 50.0) / 10.0,
        1e-9,
        "case factor",
    );
    let one = &single.case_results["PUSH"];
    close(
        second.joint_displacements["M"][2],
        one.joint_displacements["M"][2],
        1e-9,
        "final deflection",
    );
    for (a, b) in second.hinge_results.iter().zip(&one.hinge_results) {
        close_abs(a.force, b.force, 1e-9 * MP_BEAM, "final hinge force");
        close_abs(
            a.plastic_deformation,
            b.plastic_deformation,
            1e-9 * q_end,
            "final plastic deformation",
        );
    }
    close(
        second.base_reactions[2],
        one.base_reactions[2],
        1e-9,
        "final reaction",
    );
}

// ---------------------------------------------------------------------------
// T8: unloading parallel to A-B, rigid reload to the backbone.
// ---------------------------------------------------------------------------

/// The 1-026 cantilever with only the given hinge property at the root.
fn cantilever(prop: HingeProp) -> SapModel {
    let mut m = bare_model([true, false, true, false, true, false]);
    add_material(&mut m, "CONC", E, G, 0.0);
    add_section(&mut m, "BEAM", "CONC", A, I33, 2592.0, AS2);
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", L, 0.0, 0.0);
    m.points["1"].restraint = [true; 6];
    m.add_frame("1", "1", "2", "BEAM");
    add_pattern(&mut m, "P", 0.0);
    joint_force(&mut m, "2", "P", [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);
    let name = prop.name.clone();
    m.hinge_props.insert(name.clone(), prop);
    m.frames["1"].hinges = vec![hinge_at(&name, 0.0)];
    m
}

#[test]
fn t8_unloading_follows_a_b_and_reloads_to_the_backbone() {
    let mut m = cantilever(moment_1_026());
    m.load_cases
        .insert("L1".into(), nl_case("L1", "P", 70.0, None));
    m.load_cases
        .insert("L2".into(), nl_case("L2", "P", -40.0, Some("L1")));
    m.load_cases
        .insert("L3".into(), nl_case("L3", "P", 45.0, Some("L2")));
    analyze_model(&mut m).unwrap();
    let c = c_elastic();
    // Slope B-C of the curve is (1920 - 1440) / 0.04 = 12000.
    let q70 = (24.0 * 70.0 - 1440.0) / 12000.0;
    let r1 = &m.case_results["L1"];
    close(q70, 0.02, 1e-12, "closed form q at P = 70");
    close(
        r1.hinge_results[0].plastic_deformation,
        q70,
        1e-9,
        "q at P = 70",
    );
    close(
        r1.joint_displacements["2"][2],
        70.0 * c + L * q70,
        1e-9,
        "Uz at P = 70",
    );
    // Unload to P = 30: q stays, force 720, Uz = 30 c + 24 q = 0.48925103.
    let r2 = &m.case_results["L2"];
    close(
        r2.hinge_results[0].plastic_deformation,
        q70,
        1e-12,
        "q unchanged on unloading",
    );
    close(
        r2.hinge_results[0].force,
        720.0,
        1e-9,
        "force after unloading",
    );
    close(
        r2.joint_displacements["2"][2],
        30.0 * c + L * q70,
        1e-9,
        "Uz after unloading",
    );
    close_abs(
        r2.joint_displacements["2"][2],
        0.48925103,
        5e-9,
        "published-form Uz",
    );
    // Reload to P = 75: rigid up to 1680, then q = (1800 - 1440) / 12000 = 0.03.
    let r3 = &m.case_results["L3"];
    close(
        r3.hinge_results[0].plastic_deformation,
        0.03,
        1e-9,
        "q after reloading",
    );
    close(
        r3.hinge_results[0].force,
        24.0 * 75.0,
        1e-9,
        "force after reloading",
    );
    close(
        r3.joint_displacements["2"][2],
        75.0 * c + L * 0.03,
        1e-9,
        "Uz after reloading",
    );
    close_abs(
        r3.joint_displacements["2"][2],
        0.74312757,
        5e-9,
        "published-form Uz",
    );
}

// ---------------------------------------------------------------------------
// T9: asymmetric curve, loaded in the negative direction.
// ---------------------------------------------------------------------------

fn asymmetric_moment() -> HingeProp {
    let mut p = moment_1_026();
    p.name = "Asym".into();
    p.symmetric = false;
    p.negative = [
        HingePoint::new(-1000.0, 0.0),
        HingePoint::new(-1200.0, -0.05),
        HingePoint::new(-300.0, -0.05),
        HingePoint::new(-300.0, -0.5),
    ];
    p
}

/// Saved states of a downward push as (P, |Uz|), P the load factor of a unit load.
fn push_down(prop: HingeProp, target: f64) -> Vec<(f64, f64, f64)> {
    let mut m = cantilever(prop);
    let mut push = nl_case("PUSH", "P", -1.0, None);
    push_control(&mut push, "2", 2, target);
    push.nl_static.results_saved = ResultsSaved::MultipleStates {
        min_saved: 20,
        max_saved: 100,
        positive_only: false,
    };
    m.load_cases.insert("PUSH".into(), push);
    analyze_model(&mut m).unwrap();
    let case = &m.case_results["PUSH"];
    (0..case.static_steps.len())
        .map(|s| {
            let st = &case.static_steps[s];
            (
                st.load_factor,
                -st.joint_displacements["2"][2],
                st.hinges[0].plastic_deformation,
            )
        })
        .collect()
}

fn has_state(states: &[(f64, f64, f64)], p: f64, u: f64, what: &str) {
    assert!(
        states
            .iter()
            .any(|(sp, su, _)| (sp - p).abs() <= 1e-9 * p.abs().max(1.0)
                && (su - u).abs() <= 1e-9 * u.abs().max(1.0)),
        "{what}: no saved state with load factor {p} and |Uz| {u} in {states:?}"
    );
}

#[test]
fn t9_asymmetric_curve_yields_in_the_negative_direction_at_its_own_strength() {
    let c = c_elastic();
    let k_lim = 0.1 * E * I33 / L;
    let states = push_down(asymmetric_moment(), 3.0);
    // Yield at 24 P = 1000.
    has_state(&states, 1000.0 / 24.0, 1000.0 / 24.0 * c, "yield");
    // Point C: 1200 at q = 0.05 (slope 4000), P = 50.
    has_state(&states, 50.0, 50.0 * c + L * 0.05, "point C");
    // Drop to 300 on the slope limit, P = 12.5.
    let qd = 0.05 + (1200.0 - 300.0) / k_lim;
    has_state(
        &states,
        12.5,
        12.5 * c + L * qd,
        "point D after the slope limit",
    );
    // Plateau at P = 12.5 to the target.
    has_state(&states, 12.5, 3.0, "plateau at the target");
    // The plastic deformation is negative in this direction.
    assert!(states.last().unwrap().2 < 0.0);
}

#[test]
fn t9_scale_factor_of_the_negative_branch_alone_changes_the_negative_strength() {
    let c = c_elastic();
    let mut p = moment_1_026();
    p.name = "Half".into();
    p.symmetric = false;
    p.negative = p
        .positive
        .map(|pt| HingePoint::new(-pt.force, -pt.deformation));
    p.force_sf_negative = 0.5;
    let k_lim = 0.1 * E * I33 / L;
    let states = push_down(p, 2.0);
    has_state(
        &states,
        720.0 / 24.0,
        720.0 / 24.0 * c,
        "yield at half strength",
    );
    has_state(
        &states,
        40.0,
        40.0 * c + L * 0.04,
        "point C at half strength",
    );
    let qd = 0.04 + (960.0 - 240.0) / k_lim;
    has_state(&states, 10.0, 10.0 * c + L * qd, "point D at half strength");
}

// ---------------------------------------------------------------------------
// T10: the deformation entered at B is ignored; scale factors normalize tables.
// ---------------------------------------------------------------------------

fn tip_schedule(m: &mut SapModel) -> Vec<(f64, f64)> {
    analyze_model(m).unwrap();
    let case = &m.case_results["PUSH"];
    (0..case.static_steps.len())
        .map(|s| (case.static_steps[s].load_factor, step_u(case, s, "2", 2)))
        .collect()
}

#[test]
fn t10_point_b_deformation_is_ignored_and_scale_factors_normalize() {
    let reference = tip_schedule(&mut example_1_026(None, MULTI_6_10));
    // Point B at deformation 0.005; the other points shifted by the same amount.
    let mut shifted = moment_1_026();
    for (i, p) in shifted.positive.iter_mut().enumerate() {
        p.deformation += 0.005;
        shifted.negative[i].deformation = -p.deformation;
    }
    let mut m = example_1_026(None, MULTI_6_10);
    m.hinge_props.insert("Moment".into(), shifted);
    let got = tip_schedule(&mut m);
    assert_eq!(got.len(), reference.len());
    for (a, b) in got.iter().zip(&reference) {
        close(a.0, b.0, 1e-12, "P with B deformation");
        close_abs(
            a.1,
            b.1,
            1e-12 * b.1.abs().max(1.0),
            "Uz with B deformation",
        );
    }
    // A normalized table with scale factors 1440 and 0.01.
    let mut normalized = HingeProp::symmetric(
        "Moment",
        HingeDof::M3,
        [
            HingePoint::new(1.0, 0.0),
            HingePoint::new(4.0 / 3.0, 4.0),
            HingePoint::new(1.0 / 3.0, 4.0),
            HingePoint::new(1.0 / 3.0, 40.0),
        ],
    );
    normalized.force_sf = 1440.0;
    normalized.deformation_sf = 0.01;
    let mut m = example_1_026(None, MULTI_6_10);
    m.hinge_props.insert("Moment".into(), normalized);
    let got = tip_schedule(&mut m);
    assert_eq!(got.len(), reference.len());
    for (a, b) in got.iter().zip(&reference) {
        close(a.0, b.0, 1e-12, "P with scale factors");
        close_abs(
            a.1,
            b.1,
            1e-12 * b.1.abs().max(1.0),
            "Uz with scale factors",
        );
    }
}

// ---------------------------------------------------------------------------
// T11: beyond point E.
// ---------------------------------------------------------------------------

#[test]
fn t11_beyond_e_drops_to_zero_on_the_slope_limit_then_a_mechanism_at_zero_load() {
    let c = c_elastic();
    let k_lim = 0.1 * E * I33 / L;
    let mut prop = HingeProp::symmetric("Drop", HingeDof::M3, epp_curve(0.01, 0.02, 0.03));
    prop.force_sf = 1000.0;
    prop.force_sf_negative = 1000.0;
    let mut m = cantilever(prop);
    let mut push = nl_case("PUSH", "P", 1.0, None);
    push_control(&mut push, "2", 2, 2.0);
    push.nl_static.results_saved = ResultsSaved::MultipleStates {
        min_saved: 20,
        max_saved: 100,
        positive_only: false,
    };
    m.load_cases.insert("PUSH".into(), push);
    analyze_model(&mut m).unwrap();
    let case = &m.case_results["PUSH"];
    // At q_F = q_E + F_E / (0.1 EI / L) the hinge carries nothing: P = 0.
    let q_f = 0.03 + 1000.0 / k_lim;
    let at_f = case
        .static_steps
        .iter()
        .find(|s| s.load_factor.abs() < 1e-9 * 1000.0 / L && s.joint_displacements["2"][2] > 0.1)
        .expect("a saved state where the hinge has lost its strength");
    close(at_f.joint_displacements["2"][2], L * q_f, 1e-9, "Uz at q_F");
    close(at_f.hinges[0].plastic_deformation, q_f, 1e-9, "q_F");
    assert_eq!(at_f.hinges[0].state, HingeState::BeyondE);
    // Mechanism at P = 0 to the target: Uz = 24 q.
    let last = case.static_steps.last().unwrap();
    close_abs(
        last.load_factor,
        0.0,
        1e-9 * 1000.0 / L,
        "load on the zero-force plateau",
    );
    close(last.joint_displacements["2"][2], 2.0, 1e-9, "target");
    close(
        last.hinges[0].plastic_deformation,
        2.0 / L,
        1e-9,
        "q on the plateau",
    );
    let _ = c;
}

#[test]
fn t11_extrapolation_beyond_e_continues_the_slope() {
    let c = c_elastic();
    let mut prop = HingeProp::symmetric(
        "Hard",
        HingeDof::M3,
        [
            HingePoint::new(1000.0, 0.0),
            HingePoint::new(1100.0, 0.01),
            HingePoint::new(1200.0, 0.02),
            HingePoint::new(1300.0, 0.03),
        ],
    );
    prop.beyond_e = Some(HingeBeyondE::Extrapolated);
    let mut m = cantilever(prop);
    let mut push = nl_case("PUSH", "P", 1.0, None);
    push_control(&mut push, "2", 2, 1.0);
    m.load_cases.insert("PUSH".into(), push);
    analyze_model(&mut m).unwrap();
    let case = &m.case_results["PUSH"];
    // F = 1000 + 10000 q, 24 P = F, U = P c + 24 q.
    let q = (1.0 - 1000.0 * c / L) / (L + 10000.0 * c / L);
    close(
        case.hinge_results[0].plastic_deformation,
        q,
        1e-9,
        "q beyond E",
    );
    close(
        case.displacement_control_factor.unwrap(),
        (1000.0 + 10000.0 * q) / L,
        1e-9,
        "P beyond E",
    );
    assert_eq!(case.hinge_results[0].state, HingeState::BeyondE);
}

#[test]
fn t11_shear_hinge_reaching_e_with_a_drop_is_refused() {
    let mut m = cantilever(shear_1_026());
    let mut push = nl_case("PUSH", "P", 1.0, None);
    push_control(&mut push, "2", 2, 3.0);
    m.load_cases.insert("PUSH".into(), push);
    let e = refused(&mut m);
    assert_unsupported(&e, "point E");
    assert_unsupported(&e, "1H1");
}

// ---------------------------------------------------------------------------
// T12: a gravity chain that yields nothing equals the linear static solution.
// ---------------------------------------------------------------------------

fn max_abs_difference(a: &[f64], b: &[f64]) -> (f64, f64) {
    let diff = a
        .iter()
        .zip(b)
        .fold(0.0f64, |m, (x, y)| m.max((x - y).abs()));
    let scale = a.iter().chain(b).fold(0.0f64, |m, x| m.max(x.abs()));
    (diff, scale)
}

/// Joint displacements, reactions and frame forces agree to `rel` of the largest value.
fn same_results(a: &CaseResults, b: &CaseResults, rel: f64) {
    let flat = |r: &CaseResults| -> Vec<f64> {
        let mut v: Vec<f64> = r.joint_displacements.values().flatten().copied().collect();
        v.extend(r.joint_reactions.values().flatten().copied());
        for stations in r.frame_forces.values() {
            for s in stations {
                v.extend([s.p, s.v2, s.m3]);
            }
        }
        v
    };
    let (fa, fb) = (flat(a), flat(b));
    assert_eq!(fa.len(), fb.len());
    let (diff, scale) = max_abs_difference(&fa, &fb);
    assert!(
        diff <= rel * scale,
        "results differ by {diff} (scale {scale})"
    );
}

#[test]
fn t12_gravity_chain_without_yielding_equals_the_linear_solution() {
    let mut m = cantilever(moment_1_026());
    m.materials["CONC"].unit_weight = 0.0000868; // kip/in3, self weight along the span
    m.load_patterns["P"].self_wt_multiplier = 0.0;
    add_pattern(&mut m, "DEAD", 1.0);
    add_pattern(&mut m, "UDL", 0.0);
    m.frames["1"].loads_dist.push(FrameLoadDistributed {
        load_pat: "UDL".into(),
        my_type: 1,
        dir: 10,
        csys: "GLOBAL".into(),
        rel_dist1: 0.0,
        rel_dist2: 1.0,
        val1: 0.01,
        val2: 0.01,
    });
    m.load_cases
        .insert("A".into(), nl_case("A", "DEAD", 1.0, None));
    m.load_cases
        .insert("B".into(), nl_case("B", "UDL", 1.0, Some("A")));
    let mut lin = LoadCase::linear_static("LIN", "DEAD", 1.0);
    lin.loads.push(LoadCaseAssignment {
        load_name: "UDL".into(),
        scale_factor: 1.0,
    });
    m.load_cases.insert("LIN".into(), lin);
    analyze_model(&mut m).unwrap();
    same_results(&m.case_results["B"], &m.case_results["LIN"], 1e-9);
    let h = &m.case_results["B"].hinge_results[0];
    assert_eq!(h.state, HingeState::AToB);
    assert_eq!(h.direction, 0);
    assert!(h.force.abs() > 0.0 && h.force.abs() < 1440.0);
    // The hinge force is the root moment of the linear solution.
    close(
        h.force,
        m.case_results["LIN"].frame_forces["1"][0].m3,
        1e-9,
        "hinge force",
    );
}

// ---------------------------------------------------------------------------
// T13: four-storey, three-bay frame with 56 hinges (kN, m): the beam-sway
// mechanism load factor from virtual work.
// ---------------------------------------------------------------------------

const XS: [f64; 4] = [0.0, 4.0, 10.0, 14.0];
const FLOOR_H: f64 = 3.0;
const MP_BEAM_POS: f64 = 120.0;
const MP_BEAM_NEG: f64 = 160.0;
const MP_COL: f64 = 300.0;
const GAMMA: f64 = 23.5631;

fn joint_name(i: usize, k: usize) -> String {
    format!("J{i}{k}")
}

fn video_frame() -> SapModel {
    let mut m = bare_model([true, false, true, false, true, false]);
    let e = 24_855_578.0;
    add_material(&mut m, "CONC", e, e / 2.4, GAMMA);
    m.frame_sections.insert(
        "COL".into(),
        FrameSection {
            name: "COL".into(),
            material: "CONC".into(),
            prop_type: FramePropType::General,
            area: 0.09,
            i33: 0.3f64.powi(4) / 12.0,
            i22: 0.3f64.powi(4) / 12.0,
            torsional_j: 2.0 * 0.3f64.powi(4) / 12.0,
            as2: 0.09 * 5.0 / 6.0,
            as3: 0.09 * 5.0 / 6.0,
            modifiers: [1.0; 8],
            ..FrameSection::default()
        },
    );
    m.frame_sections.insert(
        "BEAM".into(),
        FrameSection {
            name: "BEAM".into(),
            material: "CONC".into(),
            prop_type: FramePropType::General,
            area: 0.12,
            i33: 0.3 * 0.4f64.powi(3) / 12.0,
            i22: 0.4 * 0.3f64.powi(3) / 12.0,
            torsional_j: 0.3 * 0.4f64.powi(3) / 12.0,
            as2: 0.12 * 5.0 / 6.0,
            as3: 0.12 * 5.0 / 6.0,
            modifiers: [1.0; 8],
            ..FrameSection::default()
        },
    );
    for (i, x) in XS.iter().enumerate() {
        for k in 0..=4 {
            let name = joint_name(i, k);
            m.add_point(name.clone(), *x, 0.0, FLOOR_H * k as f64);
            if k == 0 {
                m.points[&name].restraint = [true; 6];
            }
        }
    }
    add_pattern(&mut m, "DEAD", 1.0);
    add_pattern(&mut m, "SDL", 0.0);
    add_pattern(&mut m, "LIVE", 0.0);
    add_pattern(&mut m, "LAT", 0.0);
    let beam_prop = HingeProp {
        name: "BEAM-M3".into(),
        dof: HingeDof::M3,
        behavior: HingeBehavior::DeformationControlled,
        symmetric: false,
        positive: epp_curve(0.25, 0.35, 0.5),
        negative: epp_curve(0.25, 0.35, 0.5).map(|p| HingePoint::new(-p.force, -p.deformation)),
        force_sf: MP_BEAM_POS,
        deformation_sf: 1.0,
        force_sf_negative: MP_BEAM_NEG,
        deformation_sf_negative: 1.0,
        acceptance_positive: HingeAcceptance::default(),
        acceptance_negative: HingeAcceptance::default(),
        beyond_e: Some(HingeBeyondE::DropsToZero),
    };
    beam_prop.validate().unwrap();
    m.hinge_props.insert("BEAM-M3".into(), beam_prop);
    m.hinge_props
        .insert("COL-M3".into(), epp_moment("COL-M3", MP_COL));
    for i in 0..4 {
        for k in 0..4 {
            let name = format!("C{i}{k}");
            m.add_frame(name.clone(), joint_name(i, k), joint_name(i, k + 1), "COL");
            m.frames[&name].hinges = vec![hinge_at("COL-M3", 0.0), hinge_at("COL-M3", 1.0)];
        }
    }
    for k in 1..=4 {
        for i in 0..3 {
            let name = format!("B{i}{k}");
            m.add_frame(name.clone(), joint_name(i, k), joint_name(i + 1, k), "BEAM");
            m.frames[&name].hinges = vec![hinge_at("BEAM-M3", 0.0), hinge_at("BEAM-M3", 1.0)];
            for (pattern, value) in [("SDL", 15.0), ("LIVE", 10.0)] {
                m.frames[&name].loads_dist.push(FrameLoadDistributed {
                    load_pat: pattern.into(),
                    my_type: 1,
                    dir: 10,
                    csys: "GLOBAL".into(),
                    rel_dist1: 0.0,
                    rel_dist2: 1.0,
                    val1: value,
                    val2: value,
                });
            }
        }
        joint_force(
            &mut m,
            &joint_name(0, k),
            "LAT",
            [50.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        );
    }
    m.load_cases
        .insert("NL-DEAD".into(), nl_case("NL-DEAD", "DEAD", 1.0, None));
    m.load_cases.insert(
        "NL-SDL".into(),
        nl_case("NL-SDL", "SDL", 1.0, Some("NL-DEAD")),
    );
    m.load_cases.insert(
        "NL-LIVE".into(),
        nl_case("NL-LIVE", "LIVE", 1.0, Some("NL-SDL")),
    );
    let mut push = nl_case("PUSH", "LAT", 1.0, Some("NL-LIVE"));
    // The amendment asks for 0.3, which ends at load factor 2.73 on the way up; the
    // plateau needs about 0.8 (see the test).
    push_control(&mut push, &joint_name(0, 4), 0, 1.2);
    push.nl_static.results_saved = ResultsSaved::MultipleStates {
        min_saved: 10,
        max_saved: 100,
        positive_only: true,
    };
    push.nl_static.max_total_steps = 1000;
    m.load_cases.insert("PUSH".into(), push);
    let mut lin = LoadCase::linear_static("LIN", "DEAD", 1.0);
    for p in ["SDL", "LIVE"] {
        lin.loads.push(LoadCaseAssignment {
            load_name: p.into(),
            scale_factor: 1.0,
        });
    }
    m.load_cases.insert("LIN".into(), lin);
    // The lateral pattern alone, linear: the first yield of the push follows from it.
    m.load_cases.insert(
        "LATLIN".into(),
        LoadCase::linear_static("LATLIN", "LAT", 1.0),
    );
    m
}

fn yield_moment(frame: &str, force: f64) -> f64 {
    if frame.starts_with('C') {
        MP_COL
    } else if force >= 0.0 {
        MP_BEAM_POS
    } else {
        MP_BEAM_NEG
    }
}

/// Hinge force of the hinge `index` (0 or 1: relative distance 0 or 1) of a
/// frame in a linear static result: the section force at the end station.
fn linear_hinge_force(case: &CaseResults, frame: &str, hinge: usize) -> f64 {
    let stations = &case.frame_forces[frame];
    if hinge == 1 {
        stations[0].m3
    } else {
        stations.last().unwrap().m3
    }
}

#[test]
fn t13_four_storey_frame_with_many_hinges_pushed_to_collapse() {
    let mut m = video_frame();
    analyze_model(&mut m).unwrap();
    // (a) The gravity chain is the linear solution and nothing has yielded.
    let live = &m.case_results["NL-LIVE"];
    same_results(live, &m.case_results["LIN"], 1e-9);
    assert!(live
        .hinge_results
        .iter()
        .all(|h| h.state == HingeState::AToB && h.direction == 0));
    assert_eq!(live.hinge_results.len(), 56);
    let total_gravity = GAMMA * (0.09 * 48.0 + 0.12 * 56.0) + (15.0 + 10.0) * 56.0;
    close(
        live.base_reactions[2],
        total_gravity,
        1e-9,
        "vertical reactions of the chain",
    );

    let push = &m.case_results["PUSH"];
    let roof = joint_name(0, 4);
    // The first yield load from two linear static solutions: gravity and the lateral pattern.
    let (grav, lat) = (&m.case_results["LIN"], &m.case_results["LATLIN"]);
    let mut first_yield = f64::INFINITY;
    for h in &live.hinge_results {
        let (fg, fl) = (
            linear_hinge_force(grav, &h.frame, h.hinge),
            linear_hinge_force(lat, &h.frame, h.hinge),
        );
        let cap = if h.frame.starts_with('C') {
            MP_COL
        } else if fl > 0.0 {
            MP_BEAM_POS
        } else {
            MP_BEAM_NEG
        };
        let target = if fl > 0.0 { cap } else { -cap };
        let lambda = (target - fg) / fl;
        if lambda > 0.0 {
            first_yield = first_yield.min(lambda);
        }
    }
    let s1 = first_step(push, |s| {
        s.hinges
            .iter()
            .any(|h| h.force.abs() >= yield_moment(&h.frame, h.force) * (1.0 - 1e-9))
    });
    close(
        push.static_steps[s1].load_factor,
        first_yield,
        1e-9,
        "first yield load factor",
    );

    // (b) The push ends on a plateau: the last two saved states carry the same load.
    // The beam-sway mechanism of the plan gives 3.04 by virtual work, an upper bound,
    // but it is not the one that governs: strong columns (300 against 280 at interior
    // joints) are not strong enough. The governing mechanism, from the hinges the
    // solution ends in: column lines rotate rigidly about their bases up to floor 3,
    // floors 3 and 4 translate together. Hinges: the 4 bases, the 12 beam ends of floors
    // 1 and 2 (rotation theta each, 120 + 160 per beam) and the tops of the four
    // third-storey columns. Beams stay horizontal, so gravity does no work:
    // (4 * 300 + 6 * 280 + 4 * 300) / (50 * (3 + 6 + 9 + 9)).
    let sum_floor_heights: f64 = (1..=4).map(|k| FLOOR_H * k as f64).sum();
    let beam_sway =
        (12.0 * (MP_BEAM_POS + MP_BEAM_NEG) + 4.0 * MP_COL) / (50.0 * sum_floor_heights);
    close(beam_sway, 3.04, 1e-12, "closed form beam-sway load factor");
    let governing = (4.0 * MP_COL + 6.0 * (MP_BEAM_POS + MP_BEAM_NEG) + 4.0 * MP_COL)
        / (50.0 * (3.0 + 6.0 + 9.0 + 9.0));
    close(
        governing,
        4080.0 / 1350.0,
        1e-12,
        "closed form governing mechanism",
    );
    let n = push.static_steps.len();
    assert!(n >= 4, "{n} saved states");
    let (prev, last) = (&push.static_steps[n - 2], &push.static_steps[n - 1]);
    close(
        prev.load_factor,
        governing,
        1e-9,
        "load factor of the next to last state",
    );
    close(
        last.load_factor,
        governing,
        1e-9,
        "load factor of the last state",
    );
    assert!(
        governing < beam_sway,
        "the beam-sway mechanism is only an upper bound"
    );
    close(
        last.joint_displacements[&roof][0],
        live.joint_displacements[&roof][0] + 1.2,
        1e-9,
        "roof displacement moves by the target",
    );
    // (c) Reactions: base shear is the applied lateral load at every state.
    close(
        last.base_reactions[0],
        -last.load_factor * 200.0,
        1e-9,
        "final base shear",
    );
    for s in &push.static_steps {
        close_abs(
            s.base_reactions[0],
            -s.load_factor * 200.0,
            1e-8 * last.load_factor * 200.0,
            "horizontal equilibrium at a saved state",
        );
        close(
            s.base_reactions[2],
            total_gravity,
            1e-8,
            "vertical equilibrium at a saved state",
        );
        // (d) No hinge exceeds the yield moment of its direction.
        for h in &s.hinges {
            assert!(
                h.force.abs() <= yield_moment(&h.frame, h.force) * (1.0 + 1e-9),
                "hinge {} of {} carries {}",
                h.hinge,
                h.frame,
                h.force
            );
        }
    }
    // (e) On the plateau exactly the 20 hinges of that mechanism move, each by the
    // rotation theta = roof displacement increment / 9 m, carrying its yield moment in
    // the direction it moves; the external work equals the plastic work.
    let theta = (last.joint_displacements[&roof][0] - prev.joint_displacements[&roof][0]) / 9.0;
    let mut moving: Vec<(String, usize)> = Vec::new();
    let mut plastic_work = 0.0;
    for (a, b) in prev.hinges.iter().zip(&last.hinges) {
        let dq = b.plastic_deformation - a.plastic_deformation;
        let force = 0.5 * (a.force + b.force);
        if dq.abs() > 1e-6 * theta {
            moving.push((a.frame.clone(), a.hinge));
            assert!(force * dq > 0.0, "plastic work is positive");
            close(
                force.abs(),
                yield_moment(&a.frame, force),
                1e-9,
                "moving hinge at its yield moment",
            );
            close(
                dq.abs(),
                theta,
                1e-6,
                "rotation of a hinge of the mechanism",
            );
        }
        plastic_work += force * dq;
    }
    let mut expected: Vec<(String, usize)> = Vec::new();
    for i in 0..4 {
        expected.push((format!("C{i}0"), 1)); // bases
        expected.push((format!("C{i}2"), 2)); // tops of the third-storey columns
    }
    for i in 0..3 {
        for k in 1..=2 {
            expected.push((format!("B{i}{k}"), 1));
            expected.push((format!("B{i}{k}"), 2));
        }
    }
    moving.sort();
    expected.sort();
    assert_eq!(moving, expected, "hinges moving on the plateau");
    assert_eq!(moving.len(), 20);
    let external: f64 = (1..=4)
        .map(|k| {
            let j = joint_name(0, k);
            0.5 * (prev.load_factor + last.load_factor)
                * 50.0
                * (last.joint_displacements[&j][0] - prev.joint_displacements[&j][0])
        })
        .sum();
    close(
        external,
        plastic_work,
        1e-9,
        "external work equals the plastic work",
    );
    // (f) The load factor never decreases.
    for pair in push.static_steps.windows(2) {
        assert!(
            pair[1].load_factor >= pair[0].load_factor - 1e-9 * beam_sway,
            "load factor decreased from {} to {}",
            pair[0].load_factor,
            pair[1].load_factor
        );
    }
}

// ---------------------------------------------------------------------------
// Routing: which cases are solved in hinge space.
// ---------------------------------------------------------------------------

#[test]
fn routing_multiple_states_without_hinges_solves_the_linear_problem_in_steps() {
    // No hinges, no other nonlinearity: the case saves its states.
    let mut m = cantilever(moment_1_026());
    m.frames["1"].hinges.clear();
    let mut case = nl_case("PUSH", "P", 10.0, None);
    case.nl_static.results_saved = ResultsSaved::MultipleStates {
        min_saved: 5,
        max_saved: 10,
        positive_only: true,
    };
    m.load_cases.insert("PUSH".into(), case);
    m.load_cases
        .insert("LIN".into(), LoadCase::linear_static("LIN", "P", 10.0));
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["PUSH"];
    assert_eq!(r.static_steps.len(), 6);
    for (s, st) in r.static_steps.iter().enumerate() {
        close_abs(
            st.load_factor,
            0.2 * s as f64,
            1e-12,
            "load factor of the step",
        );
        assert!(st.hinges.is_empty());
    }
    same_results(r, &m.case_results["LIN"], 1e-12);
    // The final state is the linear solution.
    close(
        r.joint_displacements["2"][2],
        10.0 * c_elastic(),
        1e-9,
        "tip deflection",
    );
}

#[test]
fn routing_multiple_states_with_other_nonlinearity_and_no_hinges_keeps_the_final_state_only() {
    let mut m = cantilever(moment_1_026());
    m.frames["1"].hinges.clear();
    let mut case = nl_case("PUSH", "P", 10.0, None);
    case.nl_geom = NLGeomType::PDelta;
    case.nl_static.results_saved = ResultsSaved::MultipleStates {
        min_saved: 5,
        max_saved: 10,
        positive_only: true,
    };
    m.load_cases.insert("PUSH".into(), case);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["PUSH"];
    assert!(
        r.static_steps.is_empty(),
        "the existing path keeps the final state only"
    );
    assert!(r.joint_displacements["2"][2] > 0.0);
}

#[test]
fn routing_a_model_with_hinges_and_large_displacements_is_refused() {
    // Wave 6 (S2b): P-Delta with hinges is implemented (see
    // `wave6_pdelta_hinges.rs`; this test used to assert its refusal), large
    // displacements stay refused.
    let mut m = cantilever(moment_1_026());
    let mut case = nl_case("PUSH", "P", 10.0, None);
    case.nl_geom = NLGeomType::LargeDisplacements;
    m.load_cases.insert("PUSH".into(), case);
    let e = refused(&mut m);
    assert_unsupported(&e, "requests large displacements");
}

#[test]
fn routing_a_mode_load_takes_the_hinge_path_and_solves_it() {
    // Replaces the Stage A test that asserted the stub refusal. Hinge-free
    // model, final state only: a Mode load still routes to the hinge solver.
    // Cantilever with a tip mass m in Z: omega^2 = 1 / (m c), c the elastic
    // tip flexibility, phi = 1 / sqrt(m) at the tip; the Mode load
    // (sf omega^2 m phi) makes the tip move by sf phi.
    let mass = 2.0;
    let sf = 3.0;
    let mut m = cantilever(moment_1_026());
    m.frames["1"].hinges.clear();
    m.points["2"].mass[2] = mass;
    m.load_cases
        .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 1));
    let mut case = nl_case("PUSH", "P", 1.0, None);
    case.loads.clear();
    case.modal_case = Some("MODAL".into());
    case.nl_static.mode_loads.push(ModeLoad {
        mode: 1,
        scale_factor: sf,
    });
    m.load_cases.insert("PUSH".into(), case);
    analyze_model(&mut m).unwrap();
    close(
        m.case_results["MODAL"].modes[0].circ_freq_rad_s.powi(2),
        1.0 / (mass * c_elastic()),
        1e-8,
        "omega^2",
    );
    let sign = m.case_results["MODAL"].modes[0].mode_shape["2"][2].signum();
    close(
        m.case_results["PUSH"].joint_displacements["2"][2],
        sign * sf / mass.sqrt(),
        1e-8,
        "tip displacement sf phi",
    );
}

#[test]
fn a_mode_load_includes_the_mass_of_the_joints_a_subdivision_creates() {
    // A member with distributed mass that is subdivided, automatically or by
    // a hinge overwrite, has lumped mass at joints the model does not show.
    // The modal load `sf omega^2 M phi` must include them: then, and only
    // then, the elastic response to it is `sf phi` (K phi = omega^2 M phi).
    // Small enough that the hinge of the second model stays elastic.
    let sf = 0.002;
    let run = |subdivide: &dyn Fn(&mut SapModel)| {
        let mut m = cantilever(moment_1_026());
        m.materials["CONC"].unit_mass = 2.0e-4;
        subdivide(&mut m);
        m.load_cases
            .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 1));
        let mut case = nl_case("PUSH", "P", 1.0, None);
        case.loads.clear();
        case.modal_case = Some("MODAL".into());
        case.nl_static.mode_loads.push(ModeLoad {
            mode: 1,
            scale_factor: sf,
        });
        m.load_cases.insert("PUSH".into(), case);
        analyze_model(&mut m).unwrap();
        let shape = m.case_results["MODAL"].modes[0].mode_shape["2"];
        let tip = m.case_results["PUSH"].joint_displacements["2"];
        assert!(m.case_results["PUSH"].hinge_results.iter().all(|h| h.direction == 0));
        (shape, tip)
    };
    // Four elements, three created joints with three quarters of the mass.
    let (shape, tip) = run(&|m| {
        m.frames["1"].hinges.clear();
        m.frames["1"].auto_mesh_segments = 4;
    });
    close(tip[2], sf * shape[2], 1e-8, "tip Uz, automatic subdivision");
    close(tip[4], sf * shape[4], 1e-8, "tip Ry, automatic subdivision");
    // A hinge at midspan in an element of half the length: joints at the
    // quarter points. The hinge stays elastic under this load.
    let (shape, tip) = run(&|m| {
        m.frames["1"].hinges = vec![hinge_at("Moment", 0.5)];
        m.frames["1"].hinge_overwrite = Some(FrameHingeOverwrite {
            auto_subdivide: true,
            relative_length: 0.5,
            do_not_drop_load: false,
        });
    });
    close(tip[2], sf * shape[2], 1e-8, "tip Uz, hinge overwrite");
    close(tip[4], sf * shape[4], 1e-8, "tip Ry, hinge overwrite");
}

#[test]
fn accel_and_mode_pushes_are_the_same_structure_after_a_unit_change() {
    // The scale factor of an Accel load is an acceleration and that of a
    // Mode load carries length times sqrt(mass): after the model changes
    // units, a load-controlled push must describe the same physical load.
    let build = |mode: bool| {
        let mut m = cantilever(moment_1_026());
        m.present_units = Units::KipInF;
        m.frames["1"].hinges.clear();
        m.materials["CONC"].unit_mass = 2.0e-4;
        m.load_cases
            .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 1));
        let mut case = nl_case("PUSH", "P", 1.0, None);
        case.loads.clear();
        if mode {
            case.modal_case = Some("MODAL".into());
            case.nl_static.mode_loads.push(ModeLoad {
                mode: 1,
                scale_factor: 0.002,
            });
        } else {
            case.loads.push(LoadCaseAssignment {
                load_name: "UZ".into(),
                scale_factor: -50.0,
            });
        }
        m.load_cases.insert("PUSH".into(), case);
        m
    };
    for mode in [false, true] {
        let mut inches = build(mode);
        let mut metres = build(mode);
        metres.convert_units_in_place(Units::KnMC);
        analyze_model(&mut inches).unwrap();
        analyze_model(&mut metres).unwrap();
        let (a, b) = (
            inches.case_results["PUSH"].joint_displacements["2"],
            metres.case_results["PUSH"].joint_displacements["2"],
        );
        assert!(a[2].abs() > 1e-6, "the push must move the tip: {a:?}");
        // The sign of a mode shape is set by its largest mass component and
        // does not depend on the units.
        close(b[2], a[2] * 0.0254, 1e-9, "tip Uz in metres");
        close(b[4], a[4], 1e-9, "tip Ry");
    }
}

// ---------------------------------------------------------------------------
// Negative tests: every refusal asserts the error kind and the named object.
// ---------------------------------------------------------------------------

/// The model with a load-controlled case PUSH of 1 kip.
fn load_controlled(mut m: SapModel) -> SapModel {
    m.load_cases
        .insert("PUSH".into(), nl_case("PUSH", "P", 1.0, None));
    m
}

fn pushed(mut m: SapModel, target: f64) -> SapModel {
    let mut push = nl_case("PUSH", "P", 1.0, None);
    push_control(&mut push, "2", 2, target);
    m.load_cases.insert("PUSH".into(), push);
    m
}

#[test]
fn refuses_force_controlled_hinges() {
    let mut prop = moment_1_026();
    prop.behavior = HingeBehavior::ForceControlled;
    let mut m = pushed(cantilever(prop), 1.0);
    let e = refused(&mut m);
    assert_unsupported(&e, "force controlled");
    assert_unsupported(&e, "Moment");
}

#[test]
fn refuses_negative_slope_in_a_shear_hinge() {
    let mut prop = shear_1_026();
    prop.positive[2].force = 60.0;
    prop.positive[3].force = 60.0;
    prop.negative = prop
        .positive
        .map(|p| HingePoint::new(-p.force, -p.deformation));
    let mut m = pushed(cantilever(prop), 1.0);
    let e = refused(&mut m);
    assert_unsupported(&e, "bending");
    assert_unsupported(&e, "frame '1'");
}

#[test]
fn refuses_invalid_curves_and_unknown_properties_as_invalid_input() {
    let mut prop = moment_1_026();
    prop.positive[0].force = 0.0;
    let mut m = pushed(cantilever(prop), 1.0);
    match refused(&mut m) {
        SolverError::InvalidInput(msg) => assert!(msg.contains("Moment"), "{msg}"),
        other => panic!("expected InvalidInput, got {other:?}"),
    }
    let mut m = pushed(cantilever(moment_1_026()), 1.0);
    m.frames["1"].hinges[0].prop = "Ghost".into();
    match refused(&mut m) {
        SolverError::InvalidInput(msg) => {
            assert!(msg.contains("Ghost") && msg.contains("'1'"), "{msg}")
        }
        other => panic!("expected InvalidInput, got {other:?}"),
    }
}

#[test]
fn refuses_do_not_drop_load_overlapping_overwrites_and_auto_mesh_combinations() {
    // Do not allow hinges to drop loads.
    let mut m = pushed(cantilever(moment_1_026()), 1.0);
    m.frames["1"].hinge_overwrite = Some(FrameHingeOverwrite {
        auto_subdivide: true,
        relative_length: 0.02,
        do_not_drop_load: true,
    });
    assert_unsupported(&refused(&mut m), "drop");
    // Two hinges whose elements would overlap.
    let mut m = pushed(cantilever(moment_1_026()), 1.0);
    m.frames["1"].hinges = vec![hinge_at("Moment", 0.5), hinge_at("Moment", 0.51)];
    m.frames["1"].hinge_overwrite = model_b_overwrite();
    let e = refused(&mut m);
    assert_unsupported(&e, "overlap");
    assert_unsupported(&e, "frame '1'");
    // An overwrite together with automatic subdivision.
    let mut m = pushed(cantilever(moment_1_026()), 1.0);
    m.frames["1"].hinge_overwrite = model_b_overwrite();
    m.frames["1"].auto_mesh_segments = 4;
    assert_unsupported(&refused(&mut m), "hinge overwrite");
    // A hinge on the boundary of two automatic elements.
    let mut m = pushed(cantilever(moment_1_026()), 1.0);
    m.frames["1"].hinges = vec![hinge_at("Moment", 0.5)];
    m.frames["1"].auto_mesh_segments = 2;
    let e = refused(&mut m);
    assert_unsupported(&e, "boundary");
    assert_unsupported(&e, "frame '1'");
    // An overwrite on a frame without hinges is ignored.
    let mut m = pushed(cantilever(moment_1_026()), 1.0);
    m.frames["1"].hinges.clear();
    m.frames["1"].hinge_overwrite = Some(FrameHingeOverwrite {
        auto_subdivide: true,
        relative_length: 0.02,
        do_not_drop_load: true,
    });
    analyze_model(&mut m).unwrap();
}

#[test]
fn refuses_duplicate_hinges_positions_and_frame_features() {
    let mut m = pushed(cantilever(moment_1_026()), 1.0);
    m.frames["1"].hinges = vec![hinge_at("Moment", 0.0), hinge_at("Moment", 0.0)];
    let e = refused(&mut m);
    assert_unsupported(&e, "same degree of freedom");
    assert_unsupported(&e, "frame '1'");
    let mut m = pushed(cantilever(moment_1_026()), 1.0);
    m.frames["1"].hinges = vec![hinge_at("Moment", 1.5)];
    assert_unsupported(&refused(&mut m), "outside");
    let mut m = pushed(cantilever(moment_1_026()), 1.0);
    m.frames["1"].release_j[5] = true;
    let e = refused(&mut m);
    assert_unsupported(&e, "releases");
    assert_unsupported(&e, "frame '1'");
    let mut m = pushed(cantilever(moment_1_026()), 1.0);
    m.frames["1"].end_offset_i = 1.0;
    assert_unsupported(&refused(&mut m), "end length offsets");
    let mut m = pushed(cantilever(moment_1_026()), 1.0);
    m.frames["1"].loads_point.push(point_load_at(0.0));
    assert_unsupported(&refused(&mut m), "point load");
}

/// A unit point load at relative distance `rel` for the refusal test.
fn point_load_at(rel: f64) -> FrameLoadPoint {
    FrameLoadPoint {
        load_pat: "P".into(),
        my_type: 1,
        dir: 6,
        csys: "GLOBAL".into(),
        rel_dist: rel,
        val: 1.0,
    }
}

#[test]
fn refuses_hinges_with_other_nonlinearity_and_unsupported_objects() {
    // Tension/compression limit on a frame.
    let mut m = load_controlled(cantilever(moment_1_026()));
    m.add_point("3", 2.0 * L, 0.0, 0.0);
    m.add_frame("2", "2", "3", "BEAM");
    m.frames["2"].tc_limits = Some((true, 10.0, false, 0.0));
    assert_unsupported(&refused(&mut m), "only nonlinearity is frame hinges");
    // A nonlinear link.
    let mut m = load_controlled(cantilever(moment_1_026()));
    let mut prop = LinkProp {
        name: "G".into(),
        link_type: LinkPropType::Gap,
        ..LinkProp::default()
    };
    prop.dof_nonlinear[0] = true;
    prop.ke[0] = 1.0;
    prop.k_nl[0] = 1.0;
    m.link_props.insert("G".into(), prop);
    m.links.insert(
        "L".into(),
        LinkObj {
            name: "L".into(),
            point_i: "2".into(),
            point_j: None,
            prop: "G".into(),
            angle_deg: 0.0,
            selected: false,
        },
    );
    assert_unsupported(&refused(&mut m), "only nonlinearity is frame hinges");
    // An area object next to hinged frames.
    let mut m = load_controlled(cantilever(moment_1_026()));
    m.add_point("3", L + 10.0, 0.0, 0.0);
    m.add_point("4", L + 10.0, 10.0, 0.0);
    m.add_point("5", L, 10.0, 0.0);
    m.areas.insert(
        "A1".into(),
        AreaObj::new(
            "A1",
            vec!["2".into(), "3".into(), "4".into(), "5".into()],
            "ASEC",
        ),
    );
    m.area_sections.insert(
        "ASEC".into(),
        AreaSection {
            name: "ASEC".into(),
            material: "CONC".into(),
            ..AreaSection::default()
        },
    );
    assert_unsupported(&refused(&mut m), "area, solid or link");
}

#[test]
fn refuses_nonlinear_dynamic_and_continuation_of_linear_cases_and_unloading_options() {
    let mut m = pushed(cantilever(moment_1_026()), 1.0);
    let mut dynamic = LoadCase::linear_static("DYN", "P", 1.0);
    dynamic.case_type = LoadCaseType::NonlinearDynamic;
    m.load_cases.insert("DYN".into(), dynamic);
    assert_unsupported(&refused(&mut m), "direct integration");
    // A linear case continuing from a hinged nonlinear static case.
    let mut m = pushed(cantilever(moment_1_026()), 1.0);
    let mut lin = LoadCase::linear_static("LIN", "P", 1.0);
    lin.initial_case = Some("PUSH".into());
    m.load_cases.insert("LIN".into(), lin);
    // The push yields the hinge: the run-time rule of wave 6 names it and says it
    // is on its backbone (before: refused in validation, "tangent stiffness").
    let e = refused(&mut m);
    assert_unsupported(&e, "yielded and is on its backbone");
    assert_unsupported(&e, "1H1");
    assert_unsupported(&e, "PUSH");
    // Hinge unloading options other than "unload entire structure".
    for option in [
        HingeUnloading::ApplyLocalRedistribution,
        HingeUnloading::RestartUsingSecantStiffness,
    ] {
        let mut m = pushed(cantilever(moment_1_026()), 1.0);
        m.load_cases["PUSH"].nl_static.hinge_unloading = option;
        assert_unsupported(&refused(&mut m), "unload entire structure");
    }
    // A hinged model with only linear cases is analyzed; hinges are ignored.
    let mut m = cantilever(moment_1_026());
    m.load_cases
        .insert("LIN".into(), LoadCase::linear_static("LIN", "P", 10.0));
    analyze_model(&mut m).unwrap();
    close(
        m.case_results["LIN"].joint_displacements["2"][2],
        10.0 * c_elastic(),
        1e-9,
        "linear Uz",
    );
    assert!(m.case_results["LIN"].hinge_results.is_empty());
}

#[test]
fn refuses_reversal_after_yield_with_the_hinge_named() {
    let mut m = cantilever(moment_1_026());
    m.load_cases
        .insert("L1".into(), nl_case("L1", "P", 70.0, None));
    // Total P = -10: the hinge force goes from 1680 through zero.
    m.load_cases
        .insert("L2".into(), nl_case("L2", "P", -80.0, Some("L1")));
    let e = refused(&mut m);
    assert_unsupported(&e, "reversal");
    assert_unsupported(&e, "1H1");
    assert_unsupported(&e, "frame '1'");
    assert!(m.case_results.is_empty(), "no partial results are kept");
}

/// G16: the load-control cantilever of Example 1-026's geometry (EPP hinge of 1440 kip in,
/// L = 24 in) cannot pass its limit load P = 1440 / 24 = 60 kip. The case keeps what it
/// reached and is "not finished" (v19 help "Set Load Cases to Run": "unable to reach its
/// requested final state"); a linear case of the model keeps its results. Before this
/// stage the test asserted `NonConvergence` and "no partial results are kept".
#[test]
fn load_control_past_the_limit_load_is_not_finished_with_the_limit_named() {
    let mut prop = HingeProp::symmetric("Epp", HingeDof::M3, epp_curve(0.1, 0.2, 0.3));
    prop.force_sf = 1440.0;
    prop.force_sf_negative = 1440.0;
    let mut m = cantilever(prop);
    // The hinge yields at P = 60; load control cannot pass it (target P = 70).
    m.load_cases
        .insert("PUSH".into(), nl_case("PUSH", "P", 70.0, None));
    m.load_cases
        .insert("LIN".into(), LoadCase::linear_static("LIN", "P", 10.0));
    analyze_model(&mut m).expect("a case that is not finished is not an error");
    let push = &m.case_results["PUSH"];
    match &push.status {
        CaseStatus::NotFinished { reason, steps } => {
            assert!(reason.contains("limit"), "{reason}");
            assert_eq!(*steps, push.static_steps.len());
        }
        other => panic!("expected NotFinished, got {other:?}"),
    }
    // Last state: load P = 60 kip (the case scale is 70), hinge force 1440 kip in.
    let lambda = push.displacement_control_factor.expect("last state kept");
    close(lambda * 70.0, 1440.0 / 24.0, 1e-9, "load P at the stop");
    close(push.hinge_results[0].force, 1440.0, 1e-9, "hinge force at the stop");
    close(
        push.joint_displacements["2"][2],
        60.0 * c_elastic(),
        1e-9,
        "tip Uz at the stop",
    );
    // The linear case elsewhere in the model keeps its results.
    assert!(m.case_results["LIN"].status.is_finished());
    close(
        m.case_results["LIN"].joint_displacements["2"][2],
        10.0 * c_elastic(),
        1e-9,
        "linear case Uz",
    );
    assert_eq!(sap_model::results::case_status_code(&m, "PUSH"), 3);
    assert_eq!(sap_model::results::case_status_code(&m, "LIN"), 4);
    let unfinished = sap_solver::unfinished_cases(&m);
    assert_eq!(unfinished.len(), 1);
    assert_eq!(unfinished[0].0, "PUSH");
}

/// G16 (negative): a case that is refused (not "not finished") keeps nothing: zero total
/// steps and zero saved steps stay `InvalidInput`.
#[test]
fn refusals_keep_no_partial_results_and_stay_errors() {
    let mut m = example_1_026(None, MULTI_6_10);
    m.load_cases["PUSH"].nl_static.max_total_steps = 0;
    assert!(matches!(refused(&mut m), SolverError::InvalidInput(_)));
    assert!(m.case_results.is_empty());
    assert!(m.unavailable_results.is_empty());
    let mut m = example_1_026(
        None,
        ResultsSaved::MultipleStates {
            min_saved: 0,
            max_saved: 10,
            positive_only: true,
        },
    );
    assert!(matches!(refused(&mut m), SolverError::InvalidInput(_)));
    assert!(m.case_results.is_empty());
}

/// The total-step budget of Example 1-026 model A: `max_total_steps = 5` ends the case
/// early, with the states saved before the stop equal to the first saved states of the
/// unlimited run (1e-12; published first point P = 60 at Uz = 0.0185021 in, CSI v19
/// Example 1-026) and the last accepted state appended. Before this stage the test
/// `step_budgets_are_honoured_and_fail_closed` asserted `NonConvergence`, iterations = 5 and
/// no results.
#[test]
fn step_budgets_end_the_case_early_and_keep_the_states_reached() {
    let mut full = example_1_026(None, MULTI_6_10);
    analyze_model(&mut full).unwrap();
    let full_steps = &full.case_results["PUSH"].static_steps;
    assert_eq!(full.case_results["PUSH"].status, CaseStatus::Finished);

    let mut m = example_1_026(None, MULTI_6_10);
    m.load_cases["PUSH"].nl_static.max_total_steps = 5;
    analyze_model(&mut m).expect("not finished is not an error");
    let push = &m.case_results["PUSH"];
    let CaseStatus::NotFinished { reason, steps } = &push.status else {
        panic!("expected NotFinished, got {:?}", push.status);
    };
    assert!(reason.contains("total steps"), "{reason}");
    assert_eq!(*steps, push.static_steps.len());
    let n = push.static_steps.len();
    assert!(n >= 2 && n < full_steps.len(), "{n} saved states");
    // Every saved state but the appended last one is a saved state of the unlimited run.
    for i in 0..n - 1 {
        let (a, b) = (&push.static_steps[i], &full_steps[i]);
        close_abs(a.load_factor, b.load_factor, 1e-12, "load factor");
        for dof in [2, 4] {
            close_abs(
                a.joint_displacements["2"][dof],
                b.joint_displacements["2"][dof],
                1e-12,
                "tip displacement",
            );
        }
    }
    // The appended state is the final state of the case, and the first yield (P = 60 at
    // Uz = 0.0185021 in) was passed.
    let last = &push.static_steps[n - 1];
    close_abs(
        last.joint_displacements["2"][2],
        push.joint_displacements["2"][2],
        1e-12,
        "last saved state = final state",
    );
    assert!(push.joint_displacements["2"][2] > 0.0185021);
    // The first published point (v19 Example 1-026): P = 60 kip at Uz = 0.0185021 in.
    close_abs(push.static_steps[1].load_factor, 60.0, 1e-9, "P at point 1 (kip)");
    close_abs(step_u(push, 1, "2", 2), 0.0185021, 5e-7, "Uz at point 1 (in)");
    close_abs(
        push.displacement_control_factor.unwrap(),
        last.load_factor,
        1e-12,
        "factor",
    );
}

/// The last accepted state is appended to the saved ones when the case stops, and only
/// then. Example 1-026 model A with `min_saved = max_saved = 1` saves the start state only,
/// so after `max_total_steps` steps the case holds two states: the start and the last
/// accepted one. Step 1 is the first published point (P = 60 at Uz = 0.0185021 in), step 3
/// the point P = 80 at Uz = 1.3846694 in, step 4 the point P = 20 at Uz = 1.7612291 in (CSI
/// v19 Example 1-026, model A, the schedule also asserted by `hinge::tests`; +-5e-7).
#[test]
fn the_last_accepted_state_is_appended_once_to_the_saved_states() {
    let saved = ResultsSaved::MultipleStates {
        min_saved: 1,
        max_saved: 1,
        positive_only: false,
    };
    for (steps, p, uz) in [(1, 60.0, 0.0185021), (3, 80.0, 1.3846694), (4, 20.0, 1.7612291)] {
        let mut m = example_1_026(None, saved);
        m.load_cases["PUSH"].nl_static.max_total_steps = steps;
        analyze_model(&mut m).unwrap();
        let c = &m.case_results["PUSH"];
        assert!(matches!(c.status, CaseStatus::NotFinished { steps: 2, .. }), "{:?}", c.status);
        assert_eq!(c.static_steps.len(), 2, "start state and the appended last state");
        close_abs(c.static_steps[0].load_factor, 0.0, 1e-12, "start state");
        close_abs(c.static_steps[1].load_factor, p, 1e-9, "P after the last step (kip)");
        close_abs(step_u(c, 1, "2", 2), uz, 5e-7, "Uz after the last step (in)");
        close_abs(c.joint_displacements["2"][2], uz, 5e-7, "final state = appended state");
    }
    // With the full budget the case finishes with the start state and the final state.
    let mut m = example_1_026(None, saved);
    analyze_model(&mut m).unwrap();
    assert!(m.case_results["PUSH"].status.is_finished());
    assert_eq!(m.case_results["PUSH"].static_steps.len(), 2);
    // A stop on a state that was just saved does not save it twice: load control past the
    // limit load (P = 60 kip) with saved states keeps strictly distinct states.
    let mut prop = HingeProp::symmetric("Epp", HingeDof::M3, epp_curve(0.1, 0.2, 0.3));
    prop.force_sf = 1440.0;
    prop.force_sf_negative = 1440.0;
    let mut m = cantilever(prop);
    let mut push = nl_case("PUSH", "P", 70.0, None);
    push.nl_static.results_saved = MULTI_6_10;
    m.load_cases.insert("PUSH".into(), push);
    analyze_model(&mut m).unwrap();
    let c = &m.case_results["PUSH"];
    assert!(!c.status.is_finished());
    assert!(c.static_steps.len() >= 2);
    for pair in c.static_steps.windows(2) {
        let (a, b) = (&pair[0], &pair[1]);
        let same = a.load_factor == b.load_factor
            && a.joint_displacements == b.joint_displacements
            && a.hinges.iter().zip(&b.hinges).all(|(x, y)| x.force == y.force);
        assert!(!same, "steps {} and {} are the same state", a.step, b.step);
    }
    let last = c.static_steps.last().unwrap();
    close_abs(last.load_factor * 70.0, 60.0, 1e-9, "P of the last saved state (kip)");
}

/// A case that saves only its final state keeps that state when it stops (no saved steps).
#[test]
fn a_final_state_only_case_that_stops_keeps_its_last_state() {
    let mut m = example_1_026(None, ResultsSaved::FinalState);
    m.load_cases["PUSH"].nl_static.max_total_steps = 3;
    analyze_model(&mut m).unwrap();
    let push = &m.case_results["PUSH"];
    assert!(matches!(push.status, CaseStatus::NotFinished { steps: 0, .. }));
    assert!(push.static_steps.is_empty());
    assert!(push.displacement_control_factor.is_some());
    assert!(!push.hinge_results.is_empty());
}

/// G16: cases that depend on a case that is not finished could not start (v19 help "Could
/// Not Start: ... a prerequisite case was not finished"): a case that continues from it, a
/// modal case on its stiffness, a dependant of a dependant; a combination that contains it
/// is not evaluated. They have no results and are listed with the reason; the other cases
/// run.
#[test]
fn dependants_of_a_case_that_is_not_finished_could_not_start() {
    let mut m = example_1_026(None, MULTI_6_10);
    m.load_cases["PUSH"].nl_static.max_total_steps = 5;
    m.load_cases
        .insert("NEXT".into(), nl_case("NEXT", "P", 1.0, Some("PUSH")));
    let mut modal = LoadCase::modal_eigen("MODAL", 1);
    modal.initial_case = Some("PUSH".into());
    m.load_cases.insert("MODAL".into(), modal);
    m.load_cases
        .insert("LATER".into(), nl_case("LATER", "P", 1.0, Some("NEXT")));
    m.load_cases
        .insert("LIN".into(), LoadCase::linear_static("LIN", "P", 10.0));
    for (name, contributors) in [
        ("WITH_PUSH", vec![("PUSH", 1.0), ("LIN", 1.0)]),
        ("ONLY_LIN", vec![("LIN", 2.0)]),
        ("NESTED", vec![("WITH_PUSH", 1.0)]),
    ] {
        m.load_combos.insert(
            name.into(),
            LoadCombo {
                name: name.into(),
                combo_type: ComboType::LinearAdd,
                cases: contributors
                    .into_iter()
                    .map(|(c, f)| (c.to_string(), f))
                    .collect(),
            },
        );
    }
    analyze_model(&mut m).expect("only unfinished and unavailable results");
    assert!(!m.case_results["PUSH"].status.is_finished());
    for name in ["NEXT", "MODAL", "LATER"] {
        assert!(!m.case_results.contains_key(name), "{name} has no results");
        let reason = &m.unavailable_results[name];
        assert!(reason.starts_with("could not start:"), "{reason}");
        assert_eq!(sap_model::results::case_status_code(&m, name), 2, "{name}");
    }
    assert!(m.unavailable_results["NEXT"].contains("'PUSH'"));
    assert!(m.unavailable_results["MODAL"].contains("'PUSH'"));
    assert!(m.unavailable_results["LATER"].contains("'NEXT'"));
    let mut names: Vec<String> = sap_solver::could_not_start_cases(&m)
        .into_iter()
        .map(|(n, _)| n)
        .collect();
    names.sort();
    assert_eq!(names, ["LATER", "MODAL", "NEXT"]);
    // A combination that names the case that is not finished, directly or through another
    // combination, is not evaluated; one that does not is.
    for name in ["WITH_PUSH", "NESTED"] {
        assert!(!m.case_results.contains_key(name));
        assert!(
            m.unavailable_results[name].contains("'PUSH' is not finished"),
            "{name}: {}",
            m.unavailable_results[name]
        );
    }
    close(
        m.case_results["ONLY_LIN"].joint_displacements["2"][2],
        20.0 * c_elastic(),
        1e-9,
        "combination without the unfinished case",
    );
    close(
        m.case_results["LIN"].joint_displacements["2"][2],
        10.0 * c_elastic(),
        1e-9,
        "linear case",
    );
}

/// The v19 OAPI codes of `GetCaseStatus`: 1 not run, 2 could not start, 3 not finished,
/// 4 finished.
#[test]
fn case_status_codes_follow_the_oapi() {
    let mut m = example_1_026(None, MULTI_6_10);
    m.load_cases["PUSH"].nl_static.max_total_steps = 5;
    m.load_cases
        .insert("NEXT".into(), nl_case("NEXT", "P", 1.0, Some("PUSH")));
    let mut off = LoadCase::linear_static("OFF", "P", 1.0);
    off.run_flag = false;
    m.load_cases.insert("OFF".into(), off);
    m.load_cases
        .insert("LIN".into(), LoadCase::linear_static("LIN", "P", 1.0));
    // Before the analysis nothing was run.
    for name in ["PUSH", "NEXT", "OFF", "LIN"] {
        assert_eq!(sap_model::results::case_status_code(&m, name), 1, "{name}");
    }
    analyze_model(&mut m).unwrap();
    let codes: Vec<i32> = ["LIN", "PUSH", "NEXT", "OFF"]
        .iter()
        .map(|n| sap_model::results::case_status_code(&m, n))
        .collect();
    assert_eq!(codes, [4, 3, 2, 1]);
    // A re-analysis after the budget is raised finishes the case and the dependant runs.
    m.load_cases["PUSH"].nl_static.max_total_steps = 200;
    analyze_model(&mut m).unwrap();
    assert!(m.unavailable_results.is_empty());
    assert_eq!(sap_model::results::case_status_code(&m, "PUSH"), 4);
    // NEXT starts now (it has results, whatever it reaches: load control on a hinge that
    // has softened can be not finished too).
    // NEXT starts now (it has results and is no longer "could not start"). It is itself
    // not finished: load control cannot go on past the limit load of the hinge that
    // PUSH left on its plateau, so its code is 3 (observed, not a closed form).
    assert!(m.case_results.contains_key("NEXT"));
    assert_eq!(sap_model::results::case_status_code(&m, "NEXT"), 3);
    assert!(m.unavailable_results.is_empty());
}

#[test]
fn refuses_ground_motion_loads_and_a_missing_continuation_state() {
    let mut m = pushed(cantilever(moment_1_026()), 1.0);
    m.points["2"].loads_displ.push(PointLoadDispl {
        load_pat: "P".into(),
        csys: "GLOBAL".into(),
        displ: [0.0, 0.0, 0.1, 0.0, 0.0, 0.0],
    });
    assert_unsupported(&refused(&mut m), "displacement");
    // A hinge case may only continue from a nonlinear static case.
    let mut m = pushed(cantilever(moment_1_026()), 1.0);
    m.load_cases
        .insert("LIN".into(), LoadCase::linear_static("LIN", "P", 1.0));
    m.load_cases["PUSH"].initial_case = Some("LIN".into());
    assert_unsupported(&refused(&mut m), "not a nonlinear static case");
}

#[test]
fn nonlinear_history_runs_and_ignores_hinges() {
    // Hinges do not affect a history case (modal superposition in the manual).
    let build = |with_hinges: bool| {
        let mut m = cantilever(moment_1_026());
        if !with_hinges {
            m.frames["1"].hinges.clear();
        }
        m.materials["CONC"].unit_mass = 1.0e-4;
        m.functions.insert(
            "RAMP".into(),
            DynamicFunction {
                name: "RAMP".into(),
                is_time_history: true,
                damping_ratio: 0.0,
                points: vec![(0.0, 0.0), (1.0, 1.0)],
            },
        );
        let mut c = LoadCase::linear_static("HIST", "P", 1.0);
        c.loads.clear();
        c.case_type = LoadCaseType::NonlinearHistory;
        c.modal_damping = 0.0;
        c.history_loads = vec![("RAMP".into(), "P".into(), 1.0)];
        c.time_step_dt = 0.01;
        c.num_time_steps = 5;
        c.convergence_tol = 1e-10;
        m.load_cases.insert("HIST".into(), c);
        m
    };
    let (mut with, mut without) = (build(true), build(false));
    analyze_model(&mut with).unwrap();
    analyze_model(&mut without).unwrap();
    assert_eq!(
        with.case_results["HIST"].joint_displacements,
        without.case_results["HIST"].joint_displacements
    );
}

// ---------------------------------------------------------------------------
// Other degrees of freedom, subdivision, and routing of hinge-free models.
// ---------------------------------------------------------------------------

#[test]
fn minor_axis_hinges_equal_major_axis_hinges_of_the_rotated_member() {
    // The 1-026 cantilever turned by 90 degrees about its axis: the same bending
    // is now about local 2 (M2) with shear V3. The response must not change.
    let mut major = pushed(cantilever(moment_1_026()), 2.0);
    major.frames["1"].hinges.push(hinge_at("Shear", 0.0));
    major.hinge_props.insert("Shear".into(), shear_1_026());
    let mut minor = major.clone();
    minor.frame_sections["BEAM"].i22 = I33;
    minor.frame_sections["BEAM"].i33 = 2592.0;
    minor.frame_sections["BEAM"].as2 = 0.0;
    minor.frame_sections["BEAM"].as3 = AS2;
    minor.frames["1"].angle_deg = 90.0;
    minor.hinge_props["Moment"].dof = HingeDof::M2;
    minor.hinge_props["Shear"].dof = HingeDof::V3;
    for m in [&mut major, &mut minor] {
        m.load_cases["PUSH"].nl_static.results_saved = MULTI_6_10;
        analyze_model(m).unwrap();
    }
    let (a, b) = (&major.case_results["PUSH"], &minor.case_results["PUSH"]);
    assert_eq!(a.static_steps.len(), b.static_steps.len());
    for s in 0..a.static_steps.len() {
        close(
            a.static_steps[s].load_factor,
            b.static_steps[s].load_factor,
            1e-9,
            "load factor",
        );
        close_abs(
            step_u(a, s, "2", 2),
            step_u(b, s, "2", 2),
            1e-9,
            "tip deflection",
        );
        for h in 0..2 {
            let (x, y) = (&a.static_steps[s].hinges[h], &b.static_steps[s].hinges[h]);
            close_abs(
                x.force.abs(),
                y.force.abs(),
                1e-9 * 1920.0,
                "hinge force magnitude",
            );
            close_abs(
                x.plastic_deformation.abs(),
                y.plastic_deformation.abs(),
                1e-9,
                "plastic deformation magnitude",
            );
            assert_eq!(x.state, y.state);
        }
        // The reported forces and the plastic jumps have the sign of the station
        // moment, M2 included (the hinge variable is reported like the station force).
        let root = &b.static_steps[s].frame_forces["1"][0];
        close(
            root.m2,
            b.static_steps[s].hinges[0].force,
            1e-9,
            "root M2 equals the M2 hinge force",
        );
        close(
            root.v3,
            b.static_steps[s].hinges[1].force,
            1e-9,
            "root V3 equals the V3 hinge force",
        );
    }
}

#[test]
fn hinges_in_automatically_subdivided_members_use_the_element_length() {
    // Hinge at the root of a member cut into 4 elements: the slope limit is
    // 0.1 EI / 6 in (the element), and the results are reported on the object.
    let mut m = example_1_026(None, MULTI_6_10);
    m.frames["1"].auto_mesh_segments = 4;
    analyze_model(&mut m).unwrap();
    let mut plain = example_1_026(None, MULTI_6_10);
    analyze_model(&mut plain).unwrap();
    let (case, reference) = (&m.case_results["PUSH"], &plain.case_results["PUSH"]);
    for s in 0..=6 {
        close_abs(
            step_u(case, s, "2", 2),
            step_u(reference, s, "2", 2),
            1e-9,
            "Uz before point 3",
        );
    }
    let k_lim = 0.1 * E * I33 / (L / 4.0);
    let qm3 = 0.04 + (1920.0 - 480.0) / k_lim;
    let (p3, uz3, _) = tip(case, case.static_steps.len() - 3);
    close(p3, 20.0, 1e-9, "P at point 3");
    close(
        uz3,
        tip_closed_form(20.0, qm3, 0.4).0,
        1e-9,
        "Uz at point 3 with a 6 in element",
    );
    assert_eq!(
        case.frame_forces.len(),
        1,
        "results are folded onto the object"
    );
    assert!(!case
        .joint_displacements
        .keys()
        .any(|k| k.starts_with("__fmesh")));
    assert!(!case.static_steps[3]
        .joint_displacements
        .keys()
        .any(|k| k.starts_with("__fmesh")));
    assert_eq!(case.static_steps[3].frame_forces.len(), 1);
}

#[test]
fn an_overwrite_around_interior_and_end_hinges_changes_nothing_that_is_not_a_slope_limit() {
    let mut plain = fixed_beam(false);
    plain
        .load_cases
        .insert("LOAD".into(), nl_case("LOAD", "W", 50.0, None));
    let mut cut = plain.clone();
    cut.frames["B"].hinge_overwrite = model_b_overwrite();
    analyze_model(&mut plain).unwrap();
    analyze_model(&mut cut).unwrap();
    let (a, b) = (&plain.case_results["LOAD"], &cut.case_results["LOAD"]);
    for h in 0..3 {
        close(
            a.hinge_results[h].force,
            b.hinge_results[h].force,
            1e-9,
            "hinge force",
        );
        close(
            a.hinge_results[h].plastic_deformation,
            b.hinge_results[h].plastic_deformation,
            1e-9,
            "plastic deformation",
        );
    }
    assert!(!b
        .joint_displacements
        .keys()
        .any(|k| k.starts_with("__fmesh")));
    assert_eq!(b.frame_forces.len(), 1);
    // The cut object has more stations (end, hinge intervals) than the plain one.
    assert!(b.frame_forces["B"].len() > a.frame_forces["B"].len());
    close(b.base_reactions[2], 300.0, 1e-9, "vertical equilibrium");
}

#[test]
fn a_subdivided_object_with_final_state_only_still_takes_the_hinge_path() {
    // Regression: the elements of a subdivided object do not carry its hinges, so
    // routing must not look for them there (it once fell through to the path that
    // ignores hinges).
    let mut m = example_1_026(model_b_overwrite(), ResultsSaved::FinalState);
    analyze_model(&mut m).unwrap();
    let case = &m.case_results["PUSH"];
    assert!(case.static_steps.is_empty());
    assert_eq!(case.hinge_results.len(), 2);
    close(case.joint_displacements["2"][2], 2.0, 1e-12, "target");
    close(
        case.displacement_control_factor.unwrap(),
        20.0,
        1e-9,
        "load factor on the plateau",
    );
    let mut with_steps = example_1_026(model_b_overwrite(), MULTI_6_10);
    analyze_model(&mut with_steps).unwrap();
    assert_eq!(
        case.joint_displacements, with_steps.case_results["PUSH"].joint_displacements,
        "the final state does not depend on which states are saved"
    );
}

#[test]
fn the_existing_path_gives_the_same_final_state_with_or_without_multiple_states() {
    let run = |saved: ResultsSaved| {
        let mut m = cantilever(moment_1_026());
        m.frames["1"].hinges.clear();
        let mut case = nl_case("PUSH", "P", 10.0, None);
        case.nl_geom = NLGeomType::PDelta;
        case.nl_static.results_saved = saved;
        m.load_cases.insert("PUSH".into(), case);
        analyze_model(&mut m).unwrap();
        m.case_results["PUSH"].clone()
    };
    let (final_only, multiple) = (run(ResultsSaved::FinalState), run(MULTI_6_10));
    assert_eq!(final_only.joint_displacements, multiple.joint_displacements);
    assert_eq!(final_only.joint_reactions, multiple.joint_reactions);
    assert_eq!(final_only.frame_forces, multiple.frame_forces);
    assert!(multiple.static_steps.is_empty());
}

#[test]
fn routing_multiple_states_with_a_linear_link_and_no_hinges_keeps_the_final_state_only() {
    // A saved state holds no link results, so a model with a link object
    // keeps the existing path: the same final state, and no saved states.
    let run = |saved: ResultsSaved| {
        let mut m = cantilever(moment_1_026());
        m.frames["1"].hinges.clear();
        let mut prop = LinkProp {
            name: "K".into(),
            link_type: LinkPropType::Linear,
            ..LinkProp::default()
        };
        prop.dof_active[2] = true;
        prop.ke[2] = 5.0;
        m.link_props.insert("K".into(), prop);
        m.links.insert(
            "L".into(),
            LinkObj {
                name: "L".into(),
                point_i: "2".into(),
                point_j: None,
                prop: "K".into(),
                angle_deg: 0.0,
                selected: false,
            },
        );
        let mut case = nl_case("PUSH", "P", 10.0, None);
        case.nl_static.results_saved = saved;
        m.load_cases.insert("PUSH".into(), case);
        analyze_model(&mut m).unwrap();
        m.case_results["PUSH"].clone()
    };
    let (final_only, multiple) = (run(ResultsSaved::FinalState), run(MULTI_6_10));
    assert!(!final_only.link_forces.is_empty());
    assert_eq!(final_only.joint_displacements, multiple.joint_displacements);
    assert_eq!(final_only.link_forces, multiple.link_forces);
    assert!(multiple.static_steps.is_empty());
}

#[test]
fn refuses_remaining_frame_features_and_loads_of_the_hinge_path() {
    // A non-prismatic section.
    let mut m = load_controlled(cantilever(moment_1_026()));
    m.frame_sections.insert(
        "VAR".into(),
        FrameSection {
            name: "VAR".into(),
            prop_type: FramePropType::Variable,
            non_prismatic: vec![NonPrismaticSegment {
                start_sec: "BEAM".into(),
                end_sec: "BEAM".into(),
                length: 1.0,
                is_variable_length: true,
                ei33_variation: 1,
                ei22_variation: 1,
            }],
            ..m.frame_sections["BEAM"].clone()
        },
    );
    m.frames["1"].section = "VAR".into();
    let e = refused(&mut m);
    assert_unsupported(&e, "non-prismatic");
    assert_unsupported(&e, "frame '1'");
    // A cable.
    let mut m = load_controlled(cantilever(moment_1_026()));
    m.add_point("3", 2.0 * L, 0.0, 0.0);
    m.add_frame("2", "2", "3", "BEAM");
    m.frames["2"].cable_params = Some(CableParams {
        undeformed_length: None,
        target_tension_i: None,
        horizontal_tension: None,
        num_segments: 1,
    });
    // Refused before the hinge path is reached: cables cannot mix with ordinary frames.
    assert_unsupported(&refused(&mut m), "cable");
    // A temperature load in a pattern of the case.
    let mut m = load_controlled(cantilever(moment_1_026()));
    m.frames["1"].loads_temp.push(FrameLoadTemperature {
        load_pat: "P".into(),
        my_type: 1,
        val: 10.0,
        pattern_name: None,
    });
    let e = refused(&mut m);
    assert_unsupported(&e, "temperature");
    assert_unsupported(&e, "frame '1'");
    // An insertion point with stiffness transformation: refused for every nonlinear
    // case by input validation, hinges or not.
    let mut m = load_controlled(cantilever(moment_1_026()));
    m.frames["1"].cardinal_point = 1;
    m.frames["1"].stiff_transform = true;
    assert_unsupported(&refused(&mut m), "insertion point");
    // A solid object next to hinged frames.
    let mut m = load_controlled(cantilever(moment_1_026()));
    let corners = [
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        (0.0, 10.0, 0.0),
        (10.0, 10.0, 0.0),
        (0.0, 0.0, 10.0),
        (10.0, 0.0, 10.0),
        (0.0, 10.0, 10.0),
        (10.0, 10.0, 10.0),
    ];
    let names: Vec<String> = (0..8).map(|i| format!("S{i}")).collect();
    for (name, (x, y, z)) in names.iter().zip(corners) {
        m.add_point(name.clone(), 100.0 + x, y, z);
    }
    m.solid_sections.insert(
        "SOL".into(),
        SolidSection {
            name: "SOL".into(),
            material: "CONC".into(),
            ..SolidSection::default()
        },
    );
    m.solids.insert(
        "S".into(),
        SolidObj {
            name: "S".into(),
            points: std::array::from_fn(|i| names[i].clone()),
            section: "SOL".into(),
            loads_surface: Vec::new(),
            loads_temp: Default::default(),
            selected: false,
        },
    );
    assert_unsupported(&refused(&mut m), "area, solid or link");
}

#[test]
fn refuses_modal_and_buckling_cases_that_continue_from_a_hinged_case_and_bad_overwrites() {
    for case_type in [LoadCaseType::Modal, LoadCaseType::Buckling] {
        let mut m = pushed(cantilever(moment_1_026()), 1.0);
        let mut next = LoadCase::linear_static("NEXT", "P", 1.0);
        next.case_type = case_type;
        next.initial_case = Some("PUSH".into());
        m.load_cases.insert("NEXT".into(), next);
        let e = refused(&mut m);
        if case_type == LoadCaseType::Modal {
            // Wave 6: a Modal case is judged at run time by the hinge state at the
            // end of PUSH (yielded, on its backbone). Before: refused in validation.
            assert_unsupported(&e, "yielded and is on its backbone");
            assert_unsupported(&e, "1H1");
        } else {
            // Buckling with a hinged initial case stays refused in validation.
            assert_unsupported(&e, "tangent stiffness");
        }
    }
    // A hinge that may not reach point E when the property does not say what follows it.
    let mut prop = HingeProp::symmetric("Open", HingeDof::M3, epp_curve(0.001, 0.002, 0.003));
    prop.force_sf = 1000.0;
    prop.force_sf_negative = 1000.0;
    prop.beyond_e = None;
    let mut m = pushed(cantilever(prop), 2.0);
    let e = refused(&mut m);
    assert_unsupported(&e, "not established");
    assert_unsupported(&e, "1H1");
    // Invalid overwrite lengths.
    for length in [0.0, -0.1, 1.5, f64::NAN] {
        let mut m = pushed(cantilever(moment_1_026()), 1.0);
        m.frames["1"].hinge_overwrite = Some(FrameHingeOverwrite {
            auto_subdivide: true,
            relative_length: length,
            do_not_drop_load: false,
        });
        match refused(&mut m) {
            SolverError::InvalidInput(msg) => assert!(msg.contains("relative length"), "{msg}"),
            other => panic!("expected InvalidInput, got {other:?}"),
        }
    }
}

#[test]
fn routing_multiple_states_with_a_nonlinear_link_and_no_hinges_keeps_the_final_state_only() {
    let mut m = cantilever(moment_1_026());
    m.frames["1"].hinges.clear();
    let mut prop = LinkProp {
        name: "G".into(),
        link_type: LinkPropType::Gap,
        ..LinkProp::default()
    };
    prop.dof_nonlinear[0] = true;
    prop.ke[0] = 1.0;
    prop.k_nl[0] = 1.0;
    m.link_props.insert("G".into(), prop);
    m.links.insert(
        "L".into(),
        LinkObj {
            name: "L".into(),
            point_i: "2".into(),
            point_j: None,
            prop: "G".into(),
            angle_deg: 0.0,
            selected: false,
        },
    );
    let mut case = nl_case("PUSH", "P", 10.0, None);
    case.nl_static.results_saved = MULTI_6_10;
    m.load_cases.insert("PUSH".into(), case);
    analyze_model(&mut m).unwrap();
    assert!(m.case_results["PUSH"].static_steps.is_empty());
}
