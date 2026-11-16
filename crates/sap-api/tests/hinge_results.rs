//! Pushover curve and hinge assignment queries.
use sap_api::hinges::*;
use sap_model::hinge::*;
use sap_model::*;
use sap_solver::analyze_model;

fn bare_model(active: [bool; 6]) -> SapModel {
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    m.active_dofs = active;
    m
}
fn add_pattern(m: &mut SapModel, name: &str) {
    m.load_patterns.insert(
        name.into(),
        LoadPattern {
            name: name.into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
}
fn add_material(m: &mut SapModel, name: &str, e: f64, g: f64) {
    let mut material = Material::default_steel_kip_in();
    material.name = name.into();
    material.e = e;
    material.g = g;
    material.u = e / (2.0 * g) - 1.0;
    material.unit_weight = 0.0;
    material.unit_mass = 0.0;
    m.materials.insert(name.into(), material);
}
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
fn nl_case(name: &str, pattern: &str) -> LoadCase {
    let mut c = LoadCase::linear_static(name, pattern, 1.0);
    c.case_type = LoadCaseType::NonlinearStatic;
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
fn close(a: f64, b: f64, rel: f64, what: &str) {
    let scale = a.abs().max(b.abs()).max(1e-300);
    assert!(
        (a - b).abs() <= rel * scale,
        "{what}: got {a}, expected {b} (relative tolerance {rel})"
    );
}

const E: f64 = 3600.0;
const G: f64 = 1500.0;
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
fn example_1_026() -> SapModel {
    let mut m = bare_model([true, false, true, false, true, false]);
    add_material(&mut m, "CONC", E, G);
    add_section(&mut m, "BEAM", "CONC", 216.0, 5832.0, 2592.0, 180.0);
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", L, 0.0, 0.0);
    m.points["1"].restraint = [true; 6];
    m.add_frame("1", "1", "2", "BEAM");
    add_pattern(&mut m, "P");
    joint_force(&mut m, "2", "P", [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);
    m.hinge_props.insert("Moment".into(), moment_1_026());
    m.hinge_props.insert("Shear".into(), shear_1_026());
    m.frames["1"].hinges = vec![hinge_at("Moment", 0.0), hinge_at("Shear", 0.0)];
    let mut push = nl_case("PUSH", "P");
    push_control(&mut push, "2", 2, 2.0);
    push.nl_static.results_saved = ResultsSaved::MultipleStates {
        min_saved: 6,
        max_saved: 10,
        positive_only: false,
    };
    m.load_cases.insert("PUSH".into(), push);
    m
}

fn close_abs(a: f64, b: f64, tol: f64, what: &str) {
    assert!(
        (a - b).abs() <= tol,
        "{what}: got {a}, expected {b} (tolerance {tol})"
    );
}

fn counts(p: &PushoverPoint) -> [usize; 10] {
    [
        p.a_to_b,
        p.b_to_c,
        p.c_to_d,
        p.d_to_e,
        p.beyond_e,
        p.a_to_io,
        p.io_to_ls,
        p.ls_to_cp,
        p.beyond_cp,
        p.total,
    ]
}

fn analyzed(mut m: SapModel) -> SapModel {
    analyze_model(&mut m).unwrap();
    m
}

fn refusal(m: &SapModel, case: &str) -> String {
    pushover_curve(m, case).expect_err("the curve should be refused")
}

// ---------------------------------------------------------------------------
// Example 1-026 model A (kip, in): the cantilever. The monitored DOF is Uz of
// the tip and the load is a unit tip force in +Z, so the base shear is the
// load factor P and the displacement is Uz.
// ---------------------------------------------------------------------------

#[test]
fn example_1_026_curve_has_the_load_factor_as_base_shear_and_counts_by_state() {
    let m = analyzed(example_1_026());
    let steps = &m.case_results["PUSH"].static_steps;
    let curve = pushover_curve(&m, "PUSH").unwrap();
    assert_eq!(curve.len(), steps.len());
    assert_eq!(curve.len(), 10);
    // Published SAP2000 points (Uz 0.0185 / 1.3847 / 1.7612, P 60 / 80 / 20).
    close_abs(curve[1].base_shear, 60.0, 1e-9, "P at point 1");
    close_abs(curve[1].displacement, 0.0185, 5e-5, "Uz at point 1");
    close_abs(curve[6].base_shear, 80.0, 1e-9, "P at point 2");
    close_abs(curve[6].displacement, 1.3847, 5e-5, "Uz at point 2");
    close_abs(curve[8].base_shear, 20.0, 1e-9, "P at point 3");
    close_abs(curve[8].displacement, 1.7612, 5e-5, "Uz at point 3");
    for (p, st) in curve.iter().zip(steps) {
        assert_eq!(p.step, st.step);
        close(
            p.base_shear,
            st.load_factor,
            1e-9,
            "base shear equals the load",
        );
        close(
            p.displacement,
            st.joint_displacements["2"][2],
            1e-12,
            "displacement is the monitored Uz",
        );
    }
    // Hinge counts by state: A-B, B-C, C-D, D-E, >E, then by status (no
    // acceptance criteria are given, so B-C is B-IO and every hinge is in the
    // first band), then the total. The states follow from the closed-form
    // plastic rotation of the moment hinge q = -Ry - P r (0 at point 1, then
    // 0.0138, 0.0200, 0.0275, 0.0351, 0.04 = C at point 2, then on the drop
    // above 480 and at 480 from point 3 on) and of the shear hinge
    // (q = 0 up to P = 70, then 0.151, 0.302, 0.4 = C).
    let want: [[usize; 10]; 10] = [
        [2, 0, 0, 0, 0, 2, 0, 0, 0, 2],
        [2, 0, 0, 0, 0, 2, 0, 0, 0, 2],
        [1, 1, 0, 0, 0, 2, 0, 0, 0, 2],
        [1, 1, 0, 0, 0, 2, 0, 0, 0, 2],
        [0, 2, 0, 0, 0, 2, 0, 0, 0, 2],
        [0, 2, 0, 0, 0, 2, 0, 0, 0, 2],
        [0, 0, 2, 0, 0, 2, 0, 0, 0, 2],
        [0, 0, 2, 0, 0, 2, 0, 0, 0, 2],
        [0, 0, 1, 1, 0, 2, 0, 0, 0, 2],
        [0, 0, 1, 1, 0, 2, 0, 0, 0, 2],
    ];
    for (s, (p, w)) in curve.iter().zip(want).enumerate() {
        assert_eq!(counts(p), w, "hinge counts at saved state {s}");
    }
}

#[test]
fn acceptance_criteria_split_the_b_to_c_hinges_by_status() {
    // IO 0.015, LS 0.025, CP 0.033 rad of plastic rotation on the moment hinge.
    let mut model = example_1_026();
    let prop = model.hinge_props.get_mut("Moment").unwrap();
    prop.acceptance_positive = HingeAcceptance {
        io: 0.015,
        ls: 0.025,
        cp: 0.033,
    };
    prop.acceptance_negative = HingeAcceptance {
        io: -0.015,
        ls: -0.025,
        cp: -0.033,
    };
    let m = analyzed(model);
    let curve = pushover_curve(&m, "PUSH").unwrap();
    // Plastic rotations of the moment hinge at the saved states 2 to 5 follow
    // from the closed form q = -Ry - P r (r = L^2 / 2 E I): 0.0138, 0.0200,
    // 0.0275, 0.0351 rad. Each falls in a different performance band.
    for (s, band) in [(2, 0), (3, 1), (4, 2), (5, 3)] {
        // The shear hinge has no criteria and stays in the first band.
        let mut want = [0; 4];
        want[band] += 1;
        want[0] += 1;
        let c = counts(&curve[s]);
        assert_eq!(c[5..9], want, "status counts at state {s}");
        assert_eq!(c[9], 2);
    }
    // Counts by state do not depend on the criteria.
    assert_eq!(counts(&curve[5])[..5], [0, 2, 0, 0, 0]);
}

#[test]
fn a_push_toward_negative_gives_negative_shear_and_local_axes_are_followed() {
    // Pushing down: the tip moves in -Z and the base shear is negative.
    let mut down = example_1_026();
    down.points["2"].loads_force[0].force = [0.0, 0.0, -1.0, 0.0, 0.0, 0.0];
    let down = analyzed(down);
    let curve = pushover_curve(&down, "PUSH").unwrap();
    let last = curve.last().unwrap();
    assert!(
        last.displacement < -1.9 && last.base_shear < 0.0,
        "{last:?}"
    );
    close(last.base_shear, -20.0, 1e-9, "mirror of the +Z push");
    // The joint's local axis 3 points down (B = 180 deg about Y) and the load
    // pushes down: the monitored local displacement and the base shear are
    // positive, as for the original push.
    let mut local = example_1_026();
    local.points["2"].local_axes_deg = [0.0, 180.0, 0.0];
    local.points["2"].loads_force[0].force = [0.0, 0.0, -1.0, 0.0, 0.0, 0.0];
    let local = analyzed(local);
    let curve = pushover_curve(&local, "PUSH").unwrap();
    let last = curve.last().unwrap();
    close(last.displacement, 2.0, 1e-9, "local displacement");
    close(
        last.base_shear,
        20.0,
        1e-9,
        "base shear along the local axis",
    );
}

// ---------------------------------------------------------------------------
// Fixed portal (kN, m): h = 3, b = 6, E = 2e8, Ic = 1e-4, Ib = 2e-4, Mp = 100
// at the column ends, 300 at the beam ends. Closed form: H1 = 116.6667 kN at
// 0.009375 m, collapse 4 Mp / h = 133.3333 kN at 0.015 m.
// ---------------------------------------------------------------------------

fn portal() -> SapModel {
    const H: f64 = 3.0;
    const B: f64 = 6.0;
    const EP: f64 = 2.0e8;
    let mut m = bare_model([true, false, true, false, true, false]);
    add_material(&mut m, "STEEL", EP, EP / 2.6);
    add_section(&mut m, "COLS", "STEEL", 1.0e2, 1.0e-4, 1.0e-4, 0.0);
    add_section(&mut m, "BEAMS", "STEEL", 1.0e2, 2.0e-4, 2.0e-4, 0.0);
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
    for (name, mp) in [("COLM", 100.0), ("BEAMM", 300.0)] {
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
        m.hinge_props.insert(name.into(), p);
    }
    for c in ["C1", "C2"] {
        m.frames[c].hinges = vec![hinge_at("COLM", 0.0), hinge_at("COLM", 1.0)];
    }
    m.frames["BM"].hinges = vec![hinge_at("BEAMM", 0.0), hinge_at("BEAMM", 1.0)];
    add_pattern(&mut m, "LAT");
    for t in ["T1", "T2"] {
        joint_force(&mut m, t, "LAT", [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]);
    }
    let mut push = nl_case("PUSH", "LAT");
    push_control(&mut push, "T1", 0, 0.2);
    push.nl_static.results_saved = ResultsSaved::MultipleStates {
        min_saved: 20,
        max_saved: 100,
        positive_only: true,
    };
    m.load_cases.insert("PUSH".into(), push);
    m
}

#[test]
fn fixed_portal_curve_follows_the_closed_form_first_yield_and_collapse() {
    // Closed form, evaluated here from the frame stiffness ratio.
    let (h, b, ep, ic, ib, mp) = (3.0, 6.0, 2.0e8, 1.0e-4, 2.0e-4, 100.0);
    let (kc, kb) = (ep * ic / h, ep * ib / b);
    let k = kb / kc;
    let h1 = 2.0 * mp * (6.0 * k + 1.0) / (h * (3.0 * k + 1.0));
    let d1 = h1 * h * h * (2.0 + 3.0 * k) / (12.0 * kc * (1.0 + 6.0 * k));
    let h_u = 4.0 * mp / h;
    let d2 = d1 + (h_u - h1) * h * h * (kc + 2.0 * kb) / (12.0 * kb * kc);
    close(h1, 116.666_666_666_666_67, 1e-12, "closed form H1");
    close(d1, 0.009375, 1e-12, "closed form D1");
    close(d2, 0.015, 1e-12, "closed form D2");
    let _ = b;

    let m = analyzed(portal());
    let curve = pushover_curve(&m, "PUSH").unwrap();
    assert_eq!(curve.len(), m.case_results["PUSH"].static_steps.len());
    // The start of the case, then a monotone curve in displacement.
    close_abs(curve[0].displacement, 0.0, 1e-15, "start displacement");
    close_abs(curve[0].base_shear, 0.0, 1e-12, "start base shear");
    for w in curve.windows(2) {
        assert!(w[1].displacement >= w[0].displacement - 1e-15);
        assert!(w[1].base_shear >= w[0].base_shear - 1e-9, "no softening");
    }
    // First yield: both bases reach Mp together.
    let first = curve
        .iter()
        .find(|p| (p.base_shear - h1).abs() < 1e-6 * h1)
        .expect("a saved state at first yield");
    close(first.base_shear, h1, 1e-6, "first yield shear");
    close(first.displacement, d1, 1e-6, "first yield displacement");
    // At the yield event the hinges sit at B: nothing has deformed plastically.
    assert_eq!(counts(first), [6, 0, 0, 0, 0, 6, 0, 0, 0, 6]);
    // Collapse: the tops yield, the load stays at 4 Mp / h.
    let collapse = curve
        .iter()
        .find(|p| (p.base_shear - h_u).abs() < 1e-9 * h_u)
        .expect("a saved state at the collapse load");
    close(collapse.base_shear, h_u, 1e-9, "collapse shear");
    close(collapse.displacement, d2, 1e-6, "collapse displacement");
    // The two bases are past B (B-C), the other four hinges are at B.
    assert_eq!(counts(collapse), [4, 2, 0, 0, 0, 6, 0, 0, 0, 6]);
    // The plateau runs to the target with the tops and bases deformed.
    let last = curve.last().unwrap();
    close(last.base_shear, h_u, 1e-9, "plateau shear");
    close(last.displacement, 0.2, 1e-9, "target displacement");
    assert_eq!(counts(last), [2, 4, 0, 0, 0, 6, 0, 0, 0, 6]);
}

// ---------------------------------------------------------------------------
// Refusals
// ---------------------------------------------------------------------------

#[test]
fn curve_refusals_name_the_case_and_the_reason() {
    let m = analyzed(example_1_026());
    assert!(refusal(&m, "NOPE").contains("'NOPE'"));
    // Not analysed yet.
    let fresh = example_1_026();
    let e = refusal(&fresh, "PUSH");
    assert!(e.contains("'PUSH'") && e.contains("no saved states"), "{e}");
    // A linear case.
    let mut lin = example_1_026();
    lin.load_cases
        .insert("LIN".into(), LoadCase::linear_static("LIN", "P", 1.0));
    let e = refusal(&lin, "LIN");
    assert!(e.contains("not a nonlinear static"), "{e}");
    // Final state only.
    let mut single = example_1_026();
    single.load_cases["PUSH"].nl_static.results_saved = ResultsSaved::FinalState;
    let single = analyzed(single);
    let e = refusal(&single, "PUSH");
    assert!(e.contains("'PUSH'") && e.contains("no saved states"), "{e}");
    // A single saved state (for example a model edited by hand).
    let mut one = analyzed(example_1_026());
    one.case_results
        .get_mut("PUSH")
        .unwrap()
        .static_steps
        .truncate(1);
    let e = refusal(&one, "PUSH");
    assert!(e.contains("'PUSH'") && e.contains("single state"), "{e}");
    // Load control.
    let mut load = example_1_026();
    load.load_cases["PUSH"].displacement_control = None;
    let e = refusal(&load, "PUSH");
    assert!(e.contains("displacement control"), "{e}");
    // A rotational monitored DOF (Ry of the tip).
    let mut rot = example_1_026();
    push_control(rot.load_cases.get_mut("PUSH").unwrap(), "2", 4, 0.05);
    let e = refusal(&rot, "PUSH");
    assert!(e.contains("rotation") && e.contains("'2'"), "{e}");
    // A monitored quantity that is not a joint DOF.
    let mut gen = example_1_026();
    gen.load_cases.get_mut("PUSH").unwrap().displacement_control = Some(DisplacementControl {
        target: 1.0,
        monitored: None,
        conjugate: false,
    });
    let e = refusal(&gen, "PUSH");
    assert!(e.contains("not a joint DOF"), "{e}");
}

// ---------------------------------------------------------------------------
// GetHingeAssigns and hinge unloading
// ---------------------------------------------------------------------------

#[test]
fn hinge_assigns_have_the_shape_of_the_vendor_call() {
    let m = example_1_026();
    let a = get_hinge_assigns(&m, "1").unwrap();
    assert_eq!(a.number_hinges, 2);
    assert_eq!(a.hinge_num, vec![1, 2]);
    assert_eq!(a.prop, vec!["1H1".to_string(), "1H2".to_string()]);
    // OAPI codes: 6 = M3, 2 = V2; 2 = deformation controlled.
    assert_eq!(a.my_type, vec![6, 2]);
    assert_eq!(a.behavior, vec![2, 2]);
    assert_eq!(a.source, vec!["Moment".to_string(), "Shear".to_string()]);
    assert_eq!(a.rd, vec![0.0, 0.0]);
    // A frame without hinges: empty arrays.
    let mut m = example_1_026();
    m.frames["1"].hinges.clear();
    let a = get_hinge_assigns(&m, "1").unwrap();
    assert_eq!(a, HingeAssigns::default());
    // A force-controlled property and an interior hinge.
    let mut m = example_1_026();
    m.hinge_props.get_mut("Shear").unwrap().behavior = HingeBehavior::ForceControlled;
    m.frames["1"].hinges[1].rel_dist = 0.5;
    let a = get_hinge_assigns(&m, "1").unwrap();
    assert_eq!(a.behavior, vec![2, 1]);
    assert_eq!(a.rd, vec![0.0, 0.5]);
    // Refusals.
    assert!(get_hinge_assigns(&m, "9").unwrap_err().contains("'9'"));
    m.hinge_props.shift_remove("Shear");
    let e = get_hinge_assigns(&m, "1").unwrap_err();
    assert!(e.contains("'Shear'") && e.contains("'1'"), "{e}");
}

#[test]
fn only_hinge_unloading_option_one_is_accepted() {
    let mut m = example_1_026();
    assert_eq!(get_hinge_unloading(&m, "PUSH").unwrap(), 1);
    set_hinge_unloading(&mut m, "PUSH", 1).unwrap();
    for option in [2, 3] {
        let e = set_hinge_unloading(&mut m, "PUSH", option).unwrap_err();
        assert!(e.contains("not implemented"), "{e}");
    }
    assert!(set_hinge_unloading(&mut m, "PUSH", 4).is_err());
    assert!(set_hinge_unloading(&mut m, "NOPE", 1).is_err());
    // An imported option 2 is reported as 2 and can be set back to 1.
    m.load_cases
        .get_mut("PUSH")
        .unwrap()
        .nl_static
        .hinge_unloading = HingeUnloading::ApplyLocalRedistribution;
    assert_eq!(get_hinge_unloading(&m, "PUSH").unwrap(), 2);
    set_hinge_unloading(&mut m, "PUSH", 1).unwrap();
    assert_eq!(get_hinge_unloading(&m, "PUSH").unwrap(), 1);
    m.is_locked = true;
    assert!(set_hinge_unloading(&mut m, "PUSH", 1)
        .unwrap_err()
        .contains("locked"));
}
