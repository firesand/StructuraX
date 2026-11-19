//! Stage S1 of wave 6: a flexural hinge at end I of its analysis element under
//! a geometric stiffness (defect D69), and the overwrite "do not allow hinges
//! to drop loads" where it changes nothing (G56).
//!
//! Model (kN, m): a vertical column, h = 3, E = 2e8 kN/m2, I = 1e-4 m4 (EI = 2e4
//! kN m2) in both bending planes, A = 1e-2 m2, no shear deformation (As2 = As3 =
//! 0, so the elastic stiffness is Euler-Bernoulli), fixed base, elastic
//! perfectly plastic moment hinge at the base, Mp = 100 kN m. The directly
//! specified P-Delta force of each frame is -500 kN (P = 500 compression); a unit
//! lateral load acts at the top and displacement control on U1 at the top runs
//! to 0.3 m, saving 4 to 50 states.
//!
//! References, all closed forms evaluated in the tests:
//! - moment equilibrium of the deformed column at the base, M = lambda h + P u,
//!   which gives lambda = (Mp - P u) / h once the hinge has yielded;
//! - the cubic geometric stiffness of one element, for the first yield of the
//!   one-element model (a consistency check, since the solver uses the same
//!   matrix);
//! - the exact beam-column solution of a cantilever with compression P and tip
//!   load H, against a mesh of N frame objects (independent of the element).
use sap_model::hinge::*;
use sap_model::*;
use sap_solver::{analyze_model, SolverError};

const EI: f64 = 2.0e8 * 1.0e-4;
const H: f64 = 3.0;
const P: f64 = 500.0;
const MP: f64 = 100.0;

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

fn epp(name: &str, dof: HingeDof, mp: f64) -> HingeProp {
    let curve = [
        HingePoint::new(1.0, 0.0),
        HingePoint::new(1.0, 0.25),
        HingePoint::new(1.0, 0.35),
        HingePoint::new(1.0, 0.5),
    ];
    let mut p = HingeProp::symmetric(name, dof, curve);
    p.force_sf = mp;
    p.force_sf_negative = mp;
    p
}

const SAVED: ResultsSaved = ResultsSaved::MultipleStates {
    min_saved: 4,
    max_saved: 50,
    positive_only: false,
};

struct Column {
    /// Number of frame objects the column is divided into.
    n: usize,
    /// Frames run from the top to the base.
    reversed: bool,
    /// Local axis angle of every frame (degrees).
    angle: f64,
    /// The hinge property at the base end of the base frame.
    hinge: HingeProp,
    /// Directly specified P-Delta force of every frame (tension positive).
    pdelta: f64,
    target: f64,
    /// Load direction of the unit load and of the monitored displacement
    /// (0 = U1, 1 = U2).
    dir: usize,
}

impl Column {
    fn new(hinge: HingeProp) -> Self {
        Column {
            n: 1,
            reversed: false,
            angle: 0.0,
            hinge,
            pdelta: -P,
            target: 0.3,
            dir: 0,
        }
    }

    fn build(&self) -> SapModel {
        let mut m = SapModel::default();
        m.load_cases.clear();
        m.load_patterns.clear();
        m.active_dofs = [true; 6];
        let mut material = Material::default_steel_kip_in();
        material.name = "MAT".into();
        material.e = 2.0e8;
        material.g = 2.0e8 / 2.6;
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
                area: 1.0e-2,
                i33: 1.0e-4,
                i22: 1.0e-4,
                torsional_j: 2.0e-4,
                as2: 0.0,
                as3: 0.0,
                modifiers: [1.0; 8],
                ..FrameSection::default()
            },
        );
        for k in 0..=self.n {
            let z = H * k as f64 / self.n as f64;
            m.add_point(format!("P{k}"), 0.0, 0.0, z);
        }
        m.points["P0"].restraint = [true; 6];
        m.hinge_props
            .insert(self.hinge.name.clone(), self.hinge.clone());
        for k in 0..self.n {
            let name = format!("F{}", k + 1);
            let (a, b) = (format!("P{k}"), format!("P{}", k + 1));
            let (i, j) = if self.reversed { (b, a) } else { (a, b) };
            m.add_frame(&name, &i, &j, "SEC");
            let f = &mut m.frames[name.as_str()];
            f.angle_deg = self.angle;
            f.pdelta_force = self.pdelta;
            if k == 0 {
                f.hinges = vec![FrameHinge {
                    prop: self.hinge.name.clone(),
                    rel_dist: if self.reversed { 1.0 } else { 0.0 },
                }];
            }
        }
        m.load_patterns.insert(
            "P".into(),
            LoadPattern {
                name: "P".into(),
                pat_type: LoadPatternType::Other,
                self_wt_multiplier: 0.0,
            },
        );
        let top = format!("P{}", self.n);
        let mut force = [0.0; 6];
        force[self.dir] = 1.0;
        m.points[top.as_str()].loads_force.push(PointLoadForce {
            load_pat: "P".into(),
            csys: "GLOBAL".into(),
            force,
        });
        let mut c = LoadCase::linear_static("PUSH", "P", 1.0);
        c.case_type = LoadCaseType::NonlinearStatic;
        c.displacement_control = Some(DisplacementControl {
            target: self.target,
            monitored: Some((top, self.dir)),
            conjugate: false,
        });
        c.nl_static.results_saved = SAVED;
        m.load_cases.insert("PUSH".into(), c);
        m
    }

    fn top(&self) -> String {
        format!("P{}", self.n)
    }

    /// Runs the case: the saved (load factor, top displacement along `dir`).
    fn run(&self) -> Vec<(f64, f64)> {
        let mut m = self.build();
        analyze_model(&mut m).unwrap();
        let case = &m.case_results["PUSH"];
        case.static_steps
            .iter()
            .map(|s| (s.load_factor, s.joint_displacements[self.top().as_str()][self.dir]))
            .collect()
    }
}

fn moment_hinge() -> HingeProp {
    epp("M", HingeDof::M3, MP)
}

/// First saved state at which the hinge has reached its yield moment.
fn first_yield(states: &[(f64, f64)]) -> (f64, f64) {
    states[1]
}

/// Test 1: moment equilibrium of the deformed column at the base, both
/// orientations of the frame (hinge at element end I and at element end J).
#[test]
fn t1_the_base_moment_balances_the_deformed_column_in_both_orientations() {
    for reversed in [false, true] {
        let mut c = Column::new(moment_hinge());
        c.reversed = reversed;
        let states = c.run();
        assert!(states.len() >= 4, "{states:?}");
        let (l1, u1) = first_yield(&states);
        // Before the hinge yields the response is elastic.
        assert!(u1 > 0.0 && u1 < 0.05, "first yield at u = {u1}");
        let mut yielded = 0;
        for &(lambda, u) in &states[1..] {
            // lambda = (Mp - P u) / h, tolerance 1e-9 of Mp/h (kN).
            close_abs(
                lambda,
                (MP - P * u) / H,
                1e-9 * MP / H,
                &format!("lambda at u = {u} (reversed {reversed})"),
            );
            yielded += 1;
        }
        // Calibration of the reaction check used below: the moment reaction at
        // the base joint is lambda h + P u after the yield, to 1e-9 of Mp.
        let mut m = c.build();
        analyze_model(&mut m).unwrap();
        for st in &m.case_results["PUSH"].static_steps[1..] {
            let u = st.joint_displacements["P1"][0];
            close_abs(
                st.base_reactions[4].abs(),
                st.load_factor * H + P * u,
                1e-9 * MP,
                "base moment reaction (kN m)",
            );
        }
        assert!(yielded >= 3);
        let (lf, uf) = *states.last().unwrap();
        close_abs(uf, 0.3, 1e-12, "final u");
        close_abs(lf, -50.0 / 3.0, 1e-9 * MP / H, "final lambda = -16.666667 kN");
        assert!(l1 > 0.0);
    }
}

/// The first yield of the one-element model from its own cubic geometric
/// stiffness. NOT independent of the solver (it is the same matrix): a
/// consistency check of the assembly and of the hinge, kept next to the
/// independent mesh test below.
fn cubic_first_yield() -> (f64, f64) {
    let k77 = 12.0 * EI / H.powi(3) - 1.2 * P / H;
    let k7_11 = -6.0 * EI / (H * H) + 0.1 * P;
    let k11 = 4.0 * EI / H - 2.0 * P * H / 15.0;
    let k = k77 - k7_11 * k7_11 / k11;
    let lambda = MP / (H + P / k);
    (lambda, lambda / k)
}

/// Test 2: consistency check, one element, both orientations equal.
#[test]
fn t2_first_yield_of_the_one_element_model_from_its_cubic_stiffness() {
    let (lambda, u) = cubic_first_yield();
    close(lambda, 30.795153, 1e-7, "closed form lambda1 (kN) against the probe value");
    let mut got = Vec::new();
    for reversed in [false, true] {
        let mut c = Column::new(moment_hinge());
        c.reversed = reversed;
        let states = c.run();
        let (l1, u1) = first_yield(&states);
        close(l1, lambda, 1e-9, "lambda1 (kN)");
        close(u1, u, 1e-9, "u1 (m)");
        got.push((l1, u1));
    }
    close(got[0].0, got[1].0, 1e-12, "orientations: lambda1");
    close(got[0].1, got[1].1, 1e-12, "orientations: u1");
}

/// Test 2b, independent reference: the same column in N equal frame objects,
/// elastic up to the first yield, against the exact beam-column solution of a
/// cantilever with compression P and tip load H:
///   mu = sqrt(P / EI), u = (H / P) (tan(mu h) / mu - h), M_base = H tan(mu h) / mu,
/// so the first yield is at H1 = Mp mu / tan(mu h). The section has no shear
/// deformation (As2 = As3 = 0), as the closed form requires. The element is a
/// cubic with a consistent geometric stiffness, so the mesh converges towards
/// the exact value; the error falls from N = 8 to N = 16 and the tolerance
/// asserted is 1e-7 relative for N = 8 and 1e-8 for N = 16 (observed: H1 1.4e-9
/// and 9.1e-11, u1 1.7e-8 and 1.1e-9, a factor 16 per halving as for a cubic).
#[test]
fn t2b_a_mesh_of_n_frames_converges_to_the_exact_beam_column() {
    let mu = (P / EI).sqrt();
    let t = (mu * H).tan();
    let h1 = MP * mu / t;
    let u1 = (h1 / P) * (t / mu - H);
    let mut errors = Vec::new();
    for (n, tol) in [(8usize, 1e-7), (16, 1e-8)] {
        let mut c = Column::new(moment_hinge());
        c.n = n;
        let states = c.run();
        let (l1, uu1) = first_yield(&states);
        let (el, eu) = ((l1 - h1).abs() / h1, (uu1 - u1).abs() / u1);
        eprintln!("N = {n}: H1 = {l1} (exact {h1}), error {el:e}; u1 = {uu1} (exact {u1}), error {eu:e}");
        assert!(el <= tol, "N = {n}: H1 error {el}");
        assert!(eu <= tol, "N = {n}: u1 error {eu}");
        errors.push((el, eu));
    }
    // After the yield the mesh obeys the same moment equilibrium as test 1.
    let mut c = Column::new(moment_hinge());
    c.n = 8;
    let states = c.run();
    for &(lambda, u) in &states[1..] {
        close_abs(lambda, (MP - P * u) / H, 1e-9 * MP / H, "mesh of 8: lambda after yield");
    }
    assert!(states.len() >= 4);
    assert!(errors[1].0 < errors[0].0 / 8.0, "{errors:?}");
    assert!(errors[1].1 < errors[0].1 / 8.0, "{errors:?}");
}

/// Test 3: the column turned 90 degrees about its axis (the M2 hinge bends in
/// the loaded plane; the load and the monitored displacement are U1 again, so
/// the local 3 axis lies along X) gives the same states.
#[test]
fn t3_an_m2_hinge_gives_the_same_response() {
    let reference = Column::new(moment_hinge()).run();
    for reversed in [false, true] {
        let mut c = Column::new(epp("M2", HingeDof::M2, MP));
        c.angle = 90.0;
        c.reversed = reversed;
        let got = c.run();
        assert_eq!(got.len(), reference.len(), "reversed {reversed}");
        for (a, b) in got.iter().zip(&reference) {
            close(a.0, b.0, 1e-12, "lambda");
            close(a.1, b.1, 1e-12, "u");
        }
        let (lf, _) = *got.last().unwrap();
        close_abs(lf, -50.0 / 3.0, 1e-9 * MP / H, "final lambda");
    }
}

/// The bending plane of an M3 hinge at 90 degrees is the other one: with the
/// load along U1 the column does not yield (negative control).
#[test]
fn t3b_the_hinge_of_the_other_plane_does_not_yield() {
    let mut c = Column::new(moment_hinge());
    c.angle = 90.0;
    let states = c.run();
    let (lf, uf) = *states.last().unwrap();
    close_abs(uf, 0.3, 1e-12, "final u");
    // Elastic: lambda = k u with the reduced one-element stiffness.
    let k = states[1].0 / states[1].1;
    close(lf, k * uf, 1e-9, "elastic response");
    assert!(lf > MP / H);
}

/// Shear hinges V2 and V3 at the base of the column under P-Delta: the shear
/// jump is a relative transverse displacement of the two ends of the element,
/// which the geometric stiffness carries as it is (unit test
/// `the_geometric_stiffness_annihilates_the_rigid_part_of_p_v_and_t_jumps`), so
/// the response must not depend on which end of the element is the base.
#[test]
fn t3c_shear_hinges_do_not_depend_on_the_orientation_of_the_frame() {
    for (dof, angle) in [(HingeDof::V2, 0.0), (HingeDof::V3, 90.0)] {
        let vp = 20.0;
        let mut results = Vec::new();
        for reversed in [false, true] {
            let mut c = Column::new(epp("V", dof, vp));
            c.angle = angle;
            c.reversed = reversed;
            results.push(c.run());
        }
        assert_eq!(results[0].len(), results[1].len(), "{dof:?}");
        for (a, b) in results[0].iter().zip(&results[1]) {
            close(a.0, b.0, 1e-9, "lambda");
            close(a.1, b.1, 1e-9, "u");
        }
        // The end shear of the element equals the applied load (K_g annihilates
        // the translation of the whole element), so once the hinge has yielded
        // lambda stays at Vp.
        for &(lambda, _) in &results[0][1..] {
            close(lambda, vp, 1e-9, "lambda on the shear plateau (kN)");
        }
    }
}

/// Independent reference for the slip: a rigid lateral translation of the base
/// leaves the P-Delta of the column untouched, so after the shear hinge has
/// yielded the top displacement is the slip q plus the exact beam-column
/// deflection at the plateau load, u = q + (Vp / P)(tan(mu h) / mu - h).
/// Mesh of 16 frames, both orientations, tolerance 1e-7 relative to u.
#[test]
fn t3d_a_shear_slip_adds_to_the_exact_beam_column_deflection() {
    let mu = (P / EI).sqrt();
    let vp = 20.0;
    let u_el = (vp / P) * ((mu * H).tan() / mu - H);
    for reversed in [false, true] {
        let mut c = Column::new(epp("V", HingeDof::V2, vp));
        c.n = 16;
        c.reversed = reversed;
        let mut m = c.build();
        analyze_model(&mut m).unwrap();
        let case = &m.case_results["PUSH"];
        let q = case.hinge_results[0].plastic_deformation;
        let u = case.joint_displacements["P16"][0];
        assert!(q.abs() > 0.1, "slip {q}");
        // The reported deformation is the magnitude with the sign of the jump; here the load is
        // positive and the jump moves the top by |q|.
        close(u, q.abs() + u_el, 1e-7, &format!("u = q + u_el (reversed {reversed})"));
        // The moment of the section at the base of the body (frame F1 at its
        // base end): statics of the part above in the deformed position, with
        // the axial force acting at the lateral offset of the top from the body
        // end, which has slipped by |q|: M = lambda h + P (u - |q|). A lost
        // P-Delta of the slip would give a different value (tolerance 1e-6 relative).
        let lambda = case.displacement_control_factor.unwrap();
        close(lambda, 20.0, 1e-9, "plateau load (kN)");
        let station = if reversed {
            case.frame_forces["F1"].last().unwrap()
        } else {
            &case.frame_forces["F1"][0]
        };
        close(station.m3.abs(), lambda * H + P * (u - q.abs()), 1e-6, "section moment at the slip (kN m)");
    }
}

// ---------------------------------------------------------------------------
// Test 4: without a geometric stiffness the hinge results do not change.
// ---------------------------------------------------------------------------

/// Example 1-026, model A (CSI Software Verification, kip and inch): the
/// cantilever with a moment hinge and a shear hinge at the fixed end, tip
/// load; the values below were produced by the code before this change
/// (commit 8ca1d44) and printed with 17 digits.
fn example_1_026_a() -> SapModel {
    let (e, g, a, i33, as2, l) = (3600.0, 1500.0, 216.0, 5832.0, 180.0, 24.0);
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    m.active_dofs = [true, false, true, false, true, false];
    let mut material = Material::default_steel_kip_in();
    material.name = "CONC".into();
    material.e = e;
    material.g = g;
    material.u = e / (2.0 * g) - 1.0;
    material.unit_weight = 0.0;
    material.unit_mass = 0.0;
    m.materials.insert("CONC".into(), material);
    m.frame_sections.insert(
        "BEAM".into(),
        FrameSection {
            name: "BEAM".into(),
            material: "CONC".into(),
            prop_type: FramePropType::General,
            area: a,
            i33,
            i22: 2592.0,
            torsional_j: i33 + 2592.0,
            as2,
            as3: 0.0,
            modifiers: [1.0; 8],
            ..FrameSection::default()
        },
    );
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", l, 0.0, 0.0);
    m.points["1"].restraint = [true; 6];
    m.add_frame("1", "1", "2", "BEAM");
    m.load_patterns.insert(
        "P".into(),
        LoadPattern {
            name: "P".into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
    m.points["2"].loads_force.push(PointLoadForce {
        load_pat: "P".into(),
        csys: "GLOBAL".into(),
        force: [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    });
    let pts = |v: [(f64, f64); 4]| v.map(|(f, d)| HingePoint::new(f, d));
    m.hinge_props.insert(
        "Moment".into(),
        HingeProp::symmetric(
            "Moment",
            HingeDof::M3,
            pts([(1440.0, 0.0), (1920.0, 0.04), (480.0, 0.04), (480.0, 0.4)]),
        ),
    );
    m.hinge_props.insert(
        "Shear".into(),
        HingeProp::symmetric(
            "Shear",
            HingeDof::V2,
            pts([(70.0, 0.0), (80.0, 0.4), (90.0, 0.8), (100.0, 1.2)]),
        ),
    );
    m.frames["1"].hinges = vec![
        FrameHinge {
            prop: "Moment".into(),
            rel_dist: 0.0,
        },
        FrameHinge {
            prop: "Shear".into(),
            rel_dist: 0.0,
        },
    ];
    let mut c = LoadCase::linear_static("PUSH", "P", 1.0);
    c.case_type = LoadCaseType::NonlinearStatic;
    c.displacement_control = Some(DisplacementControl {
        target: 2.0,
        monitored: Some(("2".into(), 2)),
        conjugate: false,
    });
    c.nl_static.results_saved = ResultsSaved::MultipleStates {
        min_saved: 6,
        max_saved: 10,
        positive_only: false,
    };
    m.load_cases.insert("PUSH".into(), c);
    m
}

#[test]
fn t4_example_1_026_a_equals_the_values_before_the_change() {
    let mut m = example_1_026_a();
    analyze_model(&mut m).unwrap();
    let case = &m.case_results["PUSH"];
    let got: Vec<(f64, f64, f64)> = case
        .static_steps
        .iter()
        .map(|s| {
            (
                s.load_factor,
                s.joint_displacements["2"][2],
                s.joint_displacements["2"][4],
            )
        })
        .collect();
    let before: &[(f64, f64, f64)] = BEFORE_1_026_A;
    assert_eq!(got.len(), before.len(), "{got:?}");
    for (g, b) in got.iter().zip(before) {
        close_abs(g.0, b.0, 1e-12 * b.0.abs().max(1.0), "P (kip)");
        close_abs(g.1, b.1, 1e-12 * b.1.abs().max(1.0), "Uz (in)");
        close_abs(g.2, b.2, 1e-12 * b.2.abs().max(1.0), "Ry (rad)");
    }
    let q: Vec<f64> = case.hinge_results.iter().map(|h| h.plastic_deformation).collect();
    close(q[0], BEFORE_1_026_A_Q[0], 1e-12, "final q moment (rad)");
    close(q[1], BEFORE_1_026_A_Q[1], 1e-12, "final q shear (in)");
}

/// Saved states (P, Uz, Ry) of the model above before this change.
const BEFORE_1_026_A: &[(f64, f64, f64)] = &[
    (0e0, 0e0, 0e0),
    (6.000000000000001e1, 1.850205761316873e-2, -8.230452674897122e-4),
    (6.690011585379706e1, 3.51835390946502e-1, -1.4717928770060453e-2),
    (7.000000000000001e1, 5.015857338820304e-1, -2.0960219478738006e-2),
    (7.377465173789317e1, 8.349190672153637e-1, -2.856130144201256e-2),
    (7.754930347578632e1, 1.168252400548697e0, -3.6162383405287116e-2),
    (8e1, 1.3846694101508916e0, -4.1097393689986286e-2),
    (2.688756775660081e1, 1.718002743484225e0, -5.494014009087222e-2),
    (1.9999999999999993e1, 1.7612290809327846e0, -5.6735253772290814e-2),
    (1.9999999999999993e1, 2e0, -6.668404206675813e-2),
];
/// Final plastic deformations (moment hinge in rad, shear hinge in in) before this change.
const BEFORE_1_026_A_Q: [f64; 2] = [6.640969364426155e-2, 4e-1];

// ---------------------------------------------------------------------------
// G56: "do not allow hinges to drop loads"
// ---------------------------------------------------------------------------

fn with_do_not_drop(mut m: SapModel, on: bool) -> SapModel {
    m.frames["F1"].hinge_overwrite = Some(FrameHingeOverwrite {
        auto_subdivide: false,
        relative_length: 0.02,
        do_not_drop_load: on,
    });
    m
}

fn hardening() -> HingeProp {
    let mut p = HingeProp::symmetric(
        "Hard",
        HingeDof::M3,
        [
            HingePoint::new(1.0, 0.0),
            HingePoint::new(1.2, 0.01),
            HingePoint::new(1.3, 0.02),
            HingePoint::new(1.4, 0.03),
        ],
    );
    p.force_sf = MP;
    p.force_sf_negative = MP;
    p.beyond_e = Some(HingeBeyondE::Extrapolated);
    p
}

fn all_states(m: &SapModel) -> Vec<f64> {
    let case = &m.case_results["PUSH"];
    let mut v = Vec::new();
    for s in &case.static_steps {
        v.push(s.load_factor);
        v.extend(s.joint_displacements["P1"]);
        v.extend(s.hinges.iter().flat_map(|h| [h.force, h.plastic_deformation]));
    }
    v
}

#[test]
fn g56_do_not_drop_changes_nothing_for_a_curve_that_never_drops() {
    let mut c = Column::new(hardening());
    c.pdelta = 0.0;
    c.target = 0.1;
    let mut base = c.build();
    let mut on = with_do_not_drop(c.build(), true);
    analyze_model(&mut base).unwrap();
    analyze_model(&mut on).unwrap();
    let (a, b) = (all_states(&base), all_states(&on));
    assert!(a.len() > 20);
    assert_eq!(a, b, "bitwise equal");
    // The hinge did harden: the base moment passed Mp.
    let last = base.case_results["PUSH"].static_steps.last().unwrap();
    assert!(last.hinges[0].force > MP * 1.2, "{}", last.hinges[0].force);
}

fn refused(m: &mut SapModel) -> String {
    match analyze_model(m).expect_err("the analysis should be refused") {
        SolverError::UnsupportedInput(msg) => msg,
        other => panic!("expected UnsupportedInput, got {other:?}"),
    }
}

#[test]
fn g56_is_refused_for_a_curve_that_drops() {
    // A C-to-D drop.
    let mut p = hardening();
    p.positive[2] = HingePoint::new(0.5, 0.02);
    p.positive[3] = HingePoint::new(0.5, 0.03);
    p.negative = p.positive.map(|q| HingePoint::new(-q.force, -q.deformation));
    let mut c = Column::new(p);
    c.pdelta = 0.0;
    let msg = refused(&mut with_do_not_drop(c.build(), true));
    assert!(msg.contains("'F1'") && msg.contains("drops") && msg.contains("Hard"), "{msg}");
    // The same without the overwrite is analyzed (the slope limit applies).
    let mut m = c.build();
    analyze_model(&mut m).unwrap();
    // A drop to zero beyond E (the default for a table that says nothing else).
    let mut p = hardening();
    p.beyond_e = Some(HingeBeyondE::DropsToZero);
    let mut c = Column::new(p);
    c.pdelta = 0.0;
    let msg = refused(&mut with_do_not_drop(c.build(), true));
    assert!(msg.contains("'F1'") && msg.contains("beyond point E"), "{msg}");
    // Beyond E not stated.
    let mut p = hardening();
    p.beyond_e = None;
    let mut c = Column::new(p);
    c.pdelta = 0.0;
    refused(&mut with_do_not_drop(c.build(), true));
}

// ---------------------------------------------------------------------------
// Interior hinges under a geometric stiffness
// ---------------------------------------------------------------------------

#[test]
fn an_interior_hinge_with_a_geometric_stiffness_is_refused() {
    let c = Column::new(moment_hinge());
    let mut m = c.build();
    m.frames["F1"].hinges[0].rel_dist = 0.5;
    let msg = refused(&mut m);
    assert!(msg.contains("frame 'F1'") && msg.contains("0.5") && msg.contains("geometric"), "{msg}");
    // Without the geometric stiffness the same hinge is analyzed.
    let mut c = Column::new(moment_hinge());
    c.pdelta = 0.0;
    let mut m = c.build();
    m.frames["F1"].hinges[0].rel_dist = 0.5;
    analyze_model(&mut m).unwrap();
    // A hinge at the end J with the geometric stiffness is analyzed.
    let mut m = Column::new(moment_hinge()).build();
    m.frames["F1"].hinges[0].rel_dist = 1.0;
    analyze_model(&mut m).unwrap();
}
