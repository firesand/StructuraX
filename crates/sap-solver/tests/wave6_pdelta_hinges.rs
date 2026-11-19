//! Stage S2b of wave 6: nonlinear static cases with P-Delta and frame hinges
//! (ledger item G14, a named subset).
//!
//! Supported here: a case with `nl_geom = PDelta` on the hinge path (the model
//! has frame hinges, or the chain applies Accel or Mode loads), load or
//! displacement control, hinges at the ends of their analysis elements, every
//! case of the chain with the same setting, no directly specified P-Delta
//! force. The axial force of every frame element (the average of its two end
//! forces) is brought to a fixed point at every accepted state.
//!
//! Model (kN, m), as in `wave6_hinge_end_kink.rs`: a vertical column, h = 3,
//! E = 2e8 kN/m2, I = 1e-4 m4 (EI = 2e4 kN m2) in both bending planes, no shear
//! deformation (As2 = As3 = 0, so the elastic stiffness is Euler-Bernoulli),
//! fixed base, a moment hinge at the base, Mp = 100 kN m. `GRAV` is a nonlinear
//! static case, P-Delta, load control, 500 kN down at the top; `PUSH` continues
//! from `GRAV`, P-Delta, displacement control on U1 at the top, and applies a
//! unit lateral load at the top (and, in some tests, a downward load kappa that
//! grows with the push). Tests that compare with closed forms set
//! `convergence_tol = 1e-12`.
//!
//! References, closed forms evaluated in the tests:
//! - moment equilibrium of the deformed column at the base, M = lambda h + N u
//!   (N the axial compression), at every saved state, from the results;
//! - the exact beam-column solution of a cantilever with compression P and tip
//!   load H, u = (H / P)(tan(mu h) / mu - h), M_base = H tan(mu h) / mu,
//!   mu = sqrt(P / EI), against meshes of 8 and 16 frame objects;
//! - the one-element cubic geometric stiffness written out by hand
//!   (a consistency check only: the solver uses the same matrix).
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

/// An elastic-perfectly-plastic moment curve with yield moment `mp`.
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
    hinge: HingeProp,
    /// Gravity at the top, kN down.
    gravity: f64,
    /// Downward load at the top per unit load factor of `PUSH`.
    kappa: f64,
    /// Target of the displacement control (m).
    target: f64,
    tol: f64,
    max_iterations: usize,
    /// P-Delta in both cases.
    p_delta: bool,
    saved: ResultsSaved,
    /// Load control with this scale factor of the push instead of
    /// displacement control.
    load_scale: Option<f64>,
    /// Work-conjugate displacement control.
    conjugate: bool,
}

impl Column {
    fn new(hinge: HingeProp) -> Self {
        Column {
            n: 1,
            reversed: false,
            angle: 0.0,
            hinge,
            gravity: P,
            kappa: 0.0,
            target: 0.3,
            tol: 1e-12,
            max_iterations: 50,
            p_delta: true,
            saved: SAVED,
            load_scale: None,
            conjugate: false,
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
            if k == 0 {
                f.hinges = vec![FrameHinge {
                    prop: self.hinge.name.clone(),
                    rel_dist: if self.reversed { 1.0 } else { 0.0 },
                }];
            }
        }
        for name in ["G", "P"] {
            m.load_patterns.insert(
                name.into(),
                LoadPattern {
                    name: name.into(),
                    pat_type: LoadPatternType::Other,
                    self_wt_multiplier: 0.0,
                },
            );
        }
        let top = self.top();
        m.points[top.as_str()].loads_force.push(PointLoadForce {
            load_pat: "G".into(),
            csys: "GLOBAL".into(),
            force: [0.0, 0.0, -self.gravity, 0.0, 0.0, 0.0],
        });
        m.points[top.as_str()].loads_force.push(PointLoadForce {
            load_pat: "P".into(),
            csys: "GLOBAL".into(),
            force: [1.0, 0.0, -self.kappa, 0.0, 0.0, 0.0],
        });
        let geometry = if self.p_delta {
            NLGeomType::PDelta
        } else {
            NLGeomType::None
        };
        let mut g = LoadCase::linear_static("GRAV", "G", 1.0);
        g.case_type = LoadCaseType::NonlinearStatic;
        g.nl_geom = geometry;
        g.convergence_tol = self.tol;
        g.max_iterations = self.max_iterations;
        m.load_cases.insert("GRAV".into(), g);
        let mut c = LoadCase::linear_static("PUSH", "P", 1.0);
        c.case_type = LoadCaseType::NonlinearStatic;
        c.nl_geom = geometry;
        c.initial_case = Some("GRAV".into());
        c.convergence_tol = self.tol;
        c.max_iterations = self.max_iterations;
        match self.load_scale {
            Some(sf) => c.loads[0].scale_factor = sf,
            None => {
                c.displacement_control = Some(DisplacementControl {
                    target: self.target,
                    monitored: Some((top, 0)),
                    conjugate: self.conjugate,
                })
            }
        }
        c.nl_static.results_saved = self.saved;
        m.load_cases.insert("PUSH".into(), c);
        m
    }

    fn top(&self) -> String {
        format!("P{}", self.n)
    }

    /// Runs both cases and returns the model.
    fn run_model(&self) -> SapModel {
        let mut m = self.build();
        analyze_model(&mut m).unwrap();
        m
    }

    /// The saved (load factor, top displacement U1) of `PUSH`.
    fn run(&self) -> Vec<(f64, f64)> {
        let m = self.run_model();
        m.case_results["PUSH"]
            .static_steps
            .iter()
            .map(|s| (s.load_factor, s.joint_displacements[self.top().as_str()][0]))
            .collect()
    }
}

fn moment_hinge() -> HingeProp {
    epp("M", HingeDof::M3, MP)
}

fn refused(m: &mut SapModel) -> String {
    match analyze_model(m).expect_err("the analysis should be refused") {
        SolverError::UnsupportedInput(msg) => msg,
        other => panic!("expected UnsupportedInput, got {other:?}"),
    }
}

/// The first yield of the one-element model from its own cubic geometric
/// stiffness at the axial compression `n`, with the unit lateral load and the
/// downward load `kappa` per unit load factor: lambda1 = Mp / (h + N1 / k(N1)),
/// N1 = n + kappa lambda1, k(N) the tip stiffness after condensing the rotation
/// at the top (Euler-Bernoulli cubic element with the consistent geometric
/// stiffness). NOT independent of the solver (same matrix), a consistency
/// check of the assembly and of the hinge.
fn cubic_first_yield(n: f64, kappa: f64) -> (f64, f64) {
    let k_of = |axial: f64| {
        let k77 = 12.0 * EI / H.powi(3) - 1.2 * axial / H;
        let k7_11 = -6.0 * EI / (H * H) + 0.1 * axial;
        let k11 = 4.0 * EI / H - 2.0 * axial * H / 15.0;
        k77 - k7_11 * k7_11 / k11
    };
    // Bisection on f(lambda) = lambda (h + N / k) - Mp, increasing in lambda.
    let f = |lambda: f64| {
        let axial = n + kappa * lambda;
        lambda * (H + axial / k_of(axial)) - MP
    };
    let (mut lo, mut hi) = (0.0, 80.0);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if f(mid) < 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let lambda = 0.5 * (lo + hi);
    let axial = n + kappa * lambda;
    (lambda, lambda / k_of(axial))
}

/// The elastic response of the one-element model (cubic geometric stiffness,
/// the hinge not yielded) at the top displacement `u`, with the axial
/// compression N = n + kappa lambda: lambda = u k(N), by bisection.
fn cubic_elastic_lambda(n: f64, kappa: f64, u: f64) -> f64 {
    let f = |lambda: f64| lambda - u * cubic_tip_stiffness(n + kappa * lambda);
    let (mut lo, mut hi) = (-100.0, 100.0);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if f(mid) < 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

/// The tip stiffness of the one-element column (rotation at the top
/// condensed, base fixed) with the cubic geometric stiffness at the axial
/// compression `axial`.
fn cubic_tip_stiffness(axial: f64) -> f64 {
    let k77 = 12.0 * EI / H.powi(3) - 1.2 * axial / H;
    let k7_11 = -6.0 * EI / (H * H) + 0.1 * axial;
    let k11 = 4.0 * EI / H - 2.0 * axial * H / 15.0;
    k77 - k7_11 * k7_11 / k11
}

// ---------------------------------------------------------------------------
// Reference test 1: constant axial force
// ---------------------------------------------------------------------------

/// Both orientations (hinge at element end I and at end J): the first yield
/// is lambda1 = 30.795153 kN at u1 = 0.0152292 m (the cubic element's own
/// numbers, consistency), and on the plateau lambda = (Mp - P u) / h at every
/// saved state (moment equilibrium of the deformed column, 1e-9 of Mp / h).
#[test]
fn t1_constant_axial_force_first_yield_and_plateau() {
    let (l1, u1) = cubic_first_yield(P, 0.0);
    close(l1, 30.795153, 1e-7, "closed form against the probe value");
    for reversed in [false, true] {
        let mut c = Column::new(moment_hinge());
        c.reversed = reversed;
        let m = c.run_model();
        let case = &m.case_results["PUSH"];
        let states = &case.static_steps;
        assert!(states.len() >= 4, "{} states", states.len());
        let top = c.top();
        let got: Vec<(f64, f64)> = states
            .iter()
            .map(|s| (s.load_factor, s.joint_displacements[top.as_str()][0]))
            .collect();
        close(got[1].0, l1, 1e-9, "lambda1 (kN)");
        close(got[1].1, u1, 1e-9, "u1 (m)");
        for (s, &(lambda, u)) in states.iter().zip(&got).skip(1) {
            close_abs(
                lambda,
                (MP - P * u) / H,
                1e-9 * MP / H,
                &format!("lambda at u = {u} (reversed {reversed})"),
            );
            // The base moment of the deformed column: lambda h + P u.
            close_abs(
                s.base_reactions[4].abs(),
                lambda * H + P * u,
                1e-9 * MP,
                "base moment reaction (kN m)",
            );
        }
        let (lf, uf) = *got.last().unwrap();
        close_abs(uf, 0.3, 1e-12, "final u");
        close_abs(lf, (MP - P * 0.3) / H, 1e-9 * MP / H, "final lambda = -16.666667 kN");
        // The axial force reported by the station of the column is P.
        for s in states {
            let sta = &s.frame_forces["F1"];
            close(sta[0].p, -P, 1e-9, "axial force at the base (kN)");
        }
    }
}

/// `GRAV` alone: the same final state as the non-hinge P-Delta path of the
/// model without hinges (the hinge has not yielded: elastic).
#[test]
fn t1b_gravity_on_the_hinge_path_equals_the_non_hinge_p_delta_path() {
    let c = Column::new(moment_hinge());
    let hinged = c.run_model();
    let mut plain = c.build();
    plain.frames["F1"].hinges.clear();
    plain.load_cases.shift_remove("PUSH");
    plain.load_cases["GRAV"].nl_static.results_saved = ResultsSaved::MultipleStates {
        min_saved: 4,
        max_saved: 50,
        positive_only: false,
    };
    analyze_model(&mut plain).unwrap();
    let (a, b) = (&hinged.case_results["GRAV"], &plain.case_results["GRAV"]);
    assert!(b.static_steps.is_empty(), "the non-hinge path keeps the final state only");
    assert!(!a.hinge_results.is_empty());
    for (name, d) in &b.joint_displacements {
        for k in 0..6 {
            close_abs(a.joint_displacements[name][k], d[k], 1e-9 * d[k].abs().max(1e-6), "joint displacement");
        }
    }
    for (name, r) in &b.joint_reactions {
        for k in 0..6 {
            close_abs(a.joint_reactions[name][k], r[k], 1e-9 * r[k].abs().max(1.0), "reaction");
        }
    }
    for (name, st) in &b.frame_forces {
        for (x, y) in a.frame_forces[name].iter().zip(st) {
            close_abs(x.p, y.p, 1e-9 * y.p.abs().max(1.0), "axial force");
            close_abs(x.m3, y.m3, 1e-9 * y.m3.abs().max(1.0), "moment");
        }
    }
    close(b.joint_displacements["P1"][2], -P * H / (2.0e8 * 1.0e-2), 1e-9, "axial shortening");
}

// ---------------------------------------------------------------------------
// Reference test 2: an axial force that changes with the push
// ---------------------------------------------------------------------------

/// The push applies 1 kN horizontal and kappa = 2 kN downward per unit load
/// factor at the top, so the compression is N = 500 + 2 lambda. Moment
/// equilibrium at the base on the plateau: lambda h + (500 + 2 lambda) u = Mp,
/// lambda(u) = (Mp - 500 u) / (h + 2 u); at the target u = 0.15 that is
/// 25 / 3.3 = 7.575758 kN. A frozen axial force would give (Mp - 500 u) / h =
/// 8.333333 at 0.15, so this test discriminates. First yield: the cubic
/// element's own geometric stiffness (consistency, bisection in the helper).
#[test]
fn t2_an_axial_force_that_grows_with_the_push() {
    let kappa = 2.0;
    let (l1, u1) = cubic_first_yield(P, kappa);
    let mut errors = Vec::new();
    for tol in [1e-6, 1e-12] {
        let mut c = Column::new(moment_hinge());
        c.kappa = kappa;
        c.target = 0.15;
        c.tol = tol;
        let m = c.run_model();
        let case = &m.case_results["PUSH"];
        let states = &case.static_steps;
        assert!(states.len() >= 4);
        let top = c.top();
        let mut worst = 0.0f64;
        for (i, s) in states.iter().enumerate().skip(1) {
            let (lambda, u) = (s.load_factor, s.joint_displacements[top.as_str()][0]);
            if i == 1 {
                // first yield
                worst = worst.max((lambda - l1).abs() / l1).max((u - u1).abs() / u1);
                continue;
            }
            let want = (MP - P * u) / (H + kappa * u);
            worst = worst.max((lambda - want).abs() / want.abs());
            // The axial compression reported by the base station is the one of the case.
            close(
                -s.frame_forces["F1"][0].p,
                P + kappa * lambda,
                if tol > 1e-9 { 1e-5 } else { 1e-9 },
                "axial compression at the base (kN)",
            );
        }
        let last = states.last().unwrap();
        close_abs(last.joint_displacements[top.as_str()][0], 0.15, 1e-12, "final u");
        close(last.load_factor, 25.0 / 3.3, if tol > 1e-9 { 1e-5 } else { 1e-9 }, "lambda at 0.15 (kN)");
        // A frozen axial force would give 8.3333 here.
        assert!((last.load_factor - 25.0 / 3.0).abs() > 0.5);
        eprintln!("tol {tol:e}: worst relative error {worst:e}");
        errors.push(worst);
    }
    assert!(errors[1] <= errors[0], "{errors:?}");
    assert!(errors[1] < 1e-9, "{errors:?}");
}

/// G16: a budget of one iteration cannot settle the axial forces of this case (they
/// change with the push). The case is "not finished" (it was an error naming the P-Delta
/// iteration before stage S4): its reason names the iteration and the load factor of the
/// step that failed, and it keeps the states of the steps before it. With the plan's
/// tolerance the very first step fails and only the start state is kept; with a tolerance
/// of 2e-3 several states are accepted before a step fails, and each equals the state of
/// the run with 50 iterations (the iteration of those steps converged at its first pass,
/// so it is the same iteration; 1e-12). Which step fails is not pinned: until the rates of
/// a step included the coupling through the axial forces (review of 2026-11-18) it was the
/// step at load factor 17.27 after four states; with them the first pass converges for
/// more steps and the run gets further.
#[test]
fn t2b_one_iteration_is_not_enough_and_the_case_is_not_finished_naming_the_iteration() {
    let mut c = Column::new(moment_hinge());
    c.kappa = 2.0;
    c.target = 0.15;
    c.max_iterations = 1;
    let mut m = c.build();
    analyze_model(&mut m).expect("not finished is not an error");
    let push = &m.case_results["PUSH"];
    match &push.status {
        CaseStatus::NotFinished { reason, steps } => {
            assert!(reason.contains("P-Delta axial-force iteration"), "{reason}");
            assert!(reason.contains("at load factor"), "{reason}");
            assert!(!reason.contains("start state"), "{reason}");
            assert_eq!(*steps, 1, "only the start state was reached");
        }
        other => panic!("expected NotFinished, got {other:?}"),
    }
    assert_eq!(push.static_steps.len(), 1);
    close_abs(push.static_steps[0].load_factor, 0.0, 1e-12, "start state");
    assert!(m.case_results["GRAV"].status.is_finished());
    // The same budget is enough when the axial force does not change.
    let mut c = Column::new(moment_hinge());
    c.max_iterations = 1;
    let mut m = c.build();
    analyze_model(&mut m).unwrap();
    assert!(m.case_results["PUSH"].status.is_finished());
    assert!(m.unavailable_results.is_empty());

    // States before the failed step are kept.
    let mut c = Column::new(moment_hinge());
    c.kappa = 2.0;
    c.target = 0.15;
    c.tol = 2e-3;
    c.max_iterations = 1;
    let mut m = c.build();
    analyze_model(&mut m).unwrap();
    let push = &m.case_results["PUSH"];
    let CaseStatus::NotFinished { reason, steps } = &push.status else {
        panic!("expected NotFinished, got {:?}", push.status);
    };
    // The reason names the load factor the failed step started from, which is
    // the last state kept.
    let last = push.static_steps.last().expect("states are kept");
    assert!(
        reason.contains(&format!("load factor {}", last.load_factor)),
        "{reason} (last kept state at {})",
        last.load_factor
    );
    assert_eq!(*steps, push.static_steps.len());
    assert!(*steps >= 2, "more than the start state is kept: {steps}");
    let mut full = Column::new(moment_hinge());
    full.kappa = 2.0;
    full.target = 0.15;
    full.tol = 2e-3;
    full.max_iterations = 50;
    let mut mf = full.build();
    analyze_model(&mut mf).unwrap();
    let want = &mf.case_results["PUSH"].static_steps;
    assert_eq!(mf.case_results["PUSH"].status, CaseStatus::Finished);
    assert!(push.static_steps.len() < want.len(), "the short run stops earlier");
    for (i, got) in push.static_steps.iter().enumerate() {
        close_abs(got.load_factor, want[i].load_factor, 1e-12, "load factor");
        close_abs(
            got.joint_displacements[c.top().as_str()][0],
            want[i].joint_displacements[c.top().as_str()][0],
            1e-12,
            "top displacement",
        );
    }
    // Negative: a failure of the iteration of the start state of the case stays an error
    // with no results (nothing was reached). Gravity above the buckling load is the
    // refusal `gravity_above_the_buckling_load_is_refused_with_no_results`.
}

// ---------------------------------------------------------------------------
// The target and the saved states between the old and the corrected event
// ---------------------------------------------------------------------------

/// The first event of a step is found under the axial forces at its start; the
/// axial force changes along the step, so the corrected first yield is not
/// where the uncorrected one is. With kappa = +2 (N = 500 + 2 lambda) the
/// corrected yield is at a LARGER top displacement than the uncorrected one
/// (and at a smaller load): a target, or a saved state, between the two is
/// reached before the yield and the case must stop there, elastic, with
/// lambda = u k(N) of the cubic element (the correction has to see that the
/// target comes first). With kappa = -2 the corrected yield is at a SMALLER
/// displacement: a target between the two is reached on the plateau
/// lambda = (Mp - 500 u) / (h - 2 u) (the correction has to see that the
/// yield comes first). Under load control the order of the loads is the other
/// way round, and kappa = -2 gives the elastic stop.
#[test]
fn the_target_and_saved_states_between_the_uncorrected_and_the_corrected_yield() {
    let (l_old, u_old) = cubic_first_yield(P, 0.0);

    // Displacement control, kappa = +2: the yield comes later in u.
    let (l_new, u_new) = cubic_first_yield(P, 2.0);
    assert!(u_new > u_old && l_new < l_old, "{u_old} {u_new} {l_old} {l_new}");
    let mid = 0.5 * (u_old + u_new);
    let mut c = Column::new(moment_hinge());
    c.kappa = 2.0;
    c.target = mid;
    let m = c.run_model();
    let case = &m.case_results["PUSH"];
    let last = case.static_steps.last().unwrap();
    close(last.joint_displacements["P1"][0], mid, 1e-12, "the case stops at its target (m)");
    assert_eq!(case.hinge_results[0].plastic_deformation, 0.0, "elastic at the target");
    close(last.load_factor, cubic_elastic_lambda(P, 2.0, mid), 1e-9, "elastic load factor (kN)");
    assert!(last.hinges[0].force.abs() < MP);
    // A saved state in the gap: min_saved = 2 puts one at half the target.
    let mut c = Column::new(moment_hinge());
    c.kappa = 2.0;
    c.target = 2.0 * mid;
    c.saved = ResultsSaved::MultipleStates {
        min_saved: 2,
        max_saved: 50,
        positive_only: false,
    };
    let m = c.run_model();
    let steps = &m.case_results["PUSH"].static_steps;
    let at_mid = &steps[1];
    close(at_mid.joint_displacements["P1"][0], mid, 1e-12, "the saved state is where the grid puts it (m)");
    assert_eq!(at_mid.hinges[0].plastic_deformation, 0.0);
    assert!(at_mid.hinges[0].force.abs() < MP * (1.0 - 1e-6));
    close(at_mid.load_factor, cubic_elastic_lambda(P, 2.0, mid), 1e-9, "elastic load factor at the grid (kN)");
    // The next saved state is the corrected first yield.
    let yielded = steps
        .iter()
        .find(|s| s.hinges[0].force.abs() >= MP * (1.0 - 1e-9))
        .unwrap();
    close(yielded.joint_displacements["P1"][0], u_new, 1e-9, "corrected first yield (m)");
    close(yielded.load_factor, l_new, 1e-9, "corrected first yield (kN)");

    // Displacement control, kappa = -2: the yield comes earlier in u.
    let (l2, u2) = cubic_first_yield(P, -2.0);
    assert!(u2 < u_old && l2 > l_old, "{u2} {u_old} {l2} {l_old}");
    let mid = 0.5 * (u2 + u_old);
    let mut c = Column::new(moment_hinge());
    c.kappa = -2.0;
    c.target = mid;
    let m = c.run_model();
    let case = &m.case_results["PUSH"];
    let last = case.static_steps.last().unwrap();
    close(last.joint_displacements["P1"][0], mid, 1e-12, "target (m)");
    assert!(case.hinge_results[0].plastic_deformation != 0.0, "yielded before the target");
    close(last.load_factor, (MP - P * mid) / (H - 2.0 * mid), 1e-9, "plateau load factor (kN)");
    let first = case
        .static_steps
        .iter()
        .find(|s| s.hinges[0].force.abs() >= MP * (1.0 - 1e-9))
        .unwrap();
    close(first.load_factor, l2, 1e-9, "first yield (kN)");
    close(first.joint_displacements["P1"][0], u2, 1e-9, "first yield (m)");

    // Load control, kappa = -2: the corrected yield load is larger than the
    // uncorrected one, and a push between them stops elastic.
    let sf = 0.5 * (l_old + l2);
    let mut c = Column::new(moment_hinge());
    c.kappa = -2.0;
    c.load_scale = Some(sf);
    let m = c.run_model();
    let case = &m.case_results["PUSH"];
    assert_eq!(case.hinge_results[0].plastic_deformation, 0.0, "elastic under load control");
    close(case.displacement_control_factor.unwrap(), 1.0, 1e-12, "load factor of the case (the push is scaled by sf)");
    close(
        case.joint_displacements["P1"][0],
        sf / cubic_tip_stiffness(P - 2.0 * sf),
        1e-9,
        "top displacement at the load (m)",
    );
}

/// Work-conjugate displacement control with P-Delta: the load is the lateral
/// force at the top only, so the conjugate displacement is the monitored one
/// and the states equal those of the ordinary displacement control.
#[test]
fn conjugate_control_with_p_delta_gives_the_states_of_displacement_control() {
    let reference = Column::new(moment_hinge()).run();
    let mut c = Column::new(moment_hinge());
    c.conjugate = true;
    let got = c.run();
    assert_eq!(got.len(), reference.len());
    assert!(got.len() >= 4);
    for (a, b) in got.iter().zip(&reference) {
        close_abs(a.0, b.0, 1e-9 * MP / H, "lambda (kN)");
        close_abs(a.1, b.1, 1e-9 * 0.3, "u (m)");
    }
}

// ---------------------------------------------------------------------------
// Reference test 3: portal with overturning
// ---------------------------------------------------------------------------

const PB: f64 = 6.0;
const PIC: f64 = 1.0e-4;

/// Portal (kN, m): h = 3, b = 6, E = 2e8, columns I = 1e-4, beam I = 1e4 times
/// that (a rigid beam), area 1e2 for all members, As = 0; EPP moment hinges at
/// both ends of every member, Mp = 100 for the columns and 300 for the beam;
/// `GRAV`: 500 kN down at each top joint (P-Delta, load control); `PUSH`: a
/// unit lateral load at T1, P-Delta, displacement control on U1 of T1 to `target`.
fn portal(target: f64, tol: f64) -> SapModel {
    portal_with_gravity(target, tol, P)
}

fn portal_with_gravity(target: f64, tol: f64, gravity: f64) -> SapModel {
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    m.active_dofs = [true, false, true, false, true, false];
    let mut material = Material::default_steel_kip_in();
    material.name = "STEEL".into();
    material.e = 2.0e8;
    material.g = 2.0e8 / 2.6;
    material.u = 0.3;
    material.unit_weight = 0.0;
    material.unit_mass = 0.0;
    m.materials.insert("STEEL".into(), material);
    for (name, i) in [("COLS", PIC), ("BEAMS", 1.0e4 * PIC)] {
        m.frame_sections.insert(
            name.into(),
            FrameSection {
                name: name.into(),
                material: "STEEL".into(),
                prop_type: FramePropType::General,
                area: 1.0e2,
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
    for (name, x, z) in [("B1", 0.0, 0.0), ("B2", PB, 0.0), ("T1", 0.0, H), ("T2", PB, H)] {
        m.add_point(name, x, 0.0, z);
    }
    m.points["B1"].restraint = [true; 6];
    m.points["B2"].restraint = [true; 6];
    m.add_frame("C1", "B1", "T1", "COLS");
    m.add_frame("C2", "B2", "T2", "COLS");
    m.add_frame("BM", "T1", "T2", "BEAMS");
    m.hinge_props.insert("COLM".into(), epp("COLM", HingeDof::M3, MP));
    m.hinge_props.insert("BEAMM".into(), epp("BEAMM", HingeDof::M3, 3.0 * MP));
    for (f, prop) in [("C1", "COLM"), ("C2", "COLM"), ("BM", "BEAMM")] {
        m.frames[f].hinges = vec![
            FrameHinge { prop: prop.into(), rel_dist: 0.0 },
            FrameHinge { prop: prop.into(), rel_dist: 1.0 },
        ];
    }
    for name in ["G", "LAT"] {
        m.load_patterns.insert(
            name.into(),
            LoadPattern {
                name: name.into(),
                pat_type: LoadPatternType::Other,
                self_wt_multiplier: 0.0,
            },
        );
    }
    for t in ["T1", "T2"] {
        m.points[t].loads_force.push(PointLoadForce {
            load_pat: "G".into(),
            csys: "GLOBAL".into(),
            force: [0.0, 0.0, -gravity, 0.0, 0.0, 0.0],
        });
    }
    m.points["T1"].loads_force.push(PointLoadForce {
        load_pat: "LAT".into(),
        csys: "GLOBAL".into(),
        force: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    });
    let mut g = LoadCase::linear_static("GRAV", "G", 1.0);
    g.case_type = LoadCaseType::NonlinearStatic;
    g.nl_geom = NLGeomType::PDelta;
    g.convergence_tol = tol;
    m.load_cases.insert("GRAV".into(), g);
    let mut c = LoadCase::linear_static("PUSH", "LAT", 1.0);
    c.case_type = LoadCaseType::NonlinearStatic;
    c.nl_geom = NLGeomType::PDelta;
    c.initial_case = Some("GRAV".into());
    c.convergence_tol = tol;
    c.displacement_control = Some(DisplacementControl {
        target,
        monitored: Some(("T1".into(), 0)),
        conjugate: false,
    });
    c.nl_static.results_saved = ResultsSaved::MultipleStates {
        min_saved: 20,
        max_saved: 100,
        positive_only: false,
    };
    m.load_cases.insert("PUSH".into(), c);
    m
}

/// Every saved state: each column satisfies the equilibrium of the deformed
/// element from its own end forces, `(M_J - M_I) + h V = -N delta` in the sign
/// of the station results (internal sense; N the compression and delta the
/// relative end displacement along local 2), 1e-9 relative; the base shears
/// add up to the load and the vertical reactions to the gravity, 1e-9. On the
/// sway mechanism (four column hinges at Mp, the rigid-beam approximation,
/// 1e-6): lambda(u) = (4 Mp - 2 * 500 u) / h, 66.6667 kN at u = 0.2, and the
/// axial compressions are N1 = 500 - 2 Mp / b = 466.667 and N2 = 533.333 kN
/// (moment equilibrium of the frame about a base). The plateau alone cannot
/// tell a frozen axial force from an updated one (the sum of the column forces
/// is the total gravity either way); the changing axial force of test 2 does.
#[test]
fn t3_portal_overturning_equilibrium_and_the_sway_mechanism() {
    let mut m = portal(0.2, 1e-12);
    analyze_model(&mut m).unwrap();
    let case = &m.case_results["PUSH"];
    let states = &case.static_steps;
    assert!(states.len() >= 20, "{} states", states.len());
    for (k, s) in states.iter().enumerate() {
        close_abs(s.base_reactions[0], -s.load_factor, 1e-9 * s.load_factor.abs().max(1.0), "base shear");
        close_abs(s.base_reactions[2], 2.0 * P, 1e-9 * 2.0 * P, "vertical reactions");
        for (c, base, top) in [("C1", "B1", "T1"), ("C2", "B2", "T2")] {
            let sta = &s.frame_forces[c];
            let (a, b) = (&sta[0], sta.last().unwrap());
            let n = -0.5 * (a.p + b.p);
            let delta = s.joint_displacements[top][0] - s.joint_displacements[base][0];
            let lhs = (b.m3 - a.m3) + a.v2 * H;
            close_abs(
                lhs + n * delta,
                0.0,
                1e-9 * (n * delta).abs() + 1e-12,
                &format!("{c} equilibrium at state {k}"),
            );
        }
    }
    let last = states.last().unwrap();
    close_abs(last.joint_displacements["T1"][0], 0.2, 1e-12, "target");
    close(last.load_factor, (4.0 * MP - 2.0 * P * 0.2) / H, 1e-6, "sway load 66.6667 kN");
    let n1 = -0.5 * (last.frame_forces["C1"][0].p + last.frame_forces["C1"].last().unwrap().p);
    let n2 = -0.5 * (last.frame_forces["C2"][0].p + last.frame_forces["C2"].last().unwrap().p);
    close(n1, P - 2.0 * MP / PB, 1e-6, "N1 (kN)");
    close(n2, P + 2.0 * MP / PB, 1e-6, "N2 (kN)");
    close(n1 + n2, 2.0 * P, 1e-9, "sum of the column forces");
    // The four column hinges are at Mp and the beam hinges never yielded.
    for h in &last.hinges {
        if h.frame == "BM" {
            assert_eq!(h.plastic_deformation, 0.0);
            assert!(h.force.abs() < 3.0 * MP, "{}", h.force);
        } else {
            close(h.force.abs(), MP, 1e-9, "column hinge moment (kN m)");
            assert!(h.plastic_deformation != 0.0, "{} yielded", h.name);
        }
    }
}

// ---------------------------------------------------------------------------
// Curves that harden and lose strength (constant axial force)
// ---------------------------------------------------------------------------

/// B (1.0 Mp at 0), C (1.1 Mp at a plastic rotation of 0.02 rad), D (0.2 Mp at
/// 0.03), E (0.2 Mp at 0.08), "drops to zero" beyond E. The C-to-D drop falls
/// at 9000 kN m/rad, more steeply than the slope limit of strength loss,
/// 0.1 EI/L = 666.67 kN m/rad (project choice, see `build_backbone`), so the
/// path follows the limited slope: C at 0.02, D at 0.02 + 90 / 666.67 = 0.155,
/// E at 0.205, the force zero from 0.235 on.
fn hardening_and_drop() -> HingeProp {
    let mut p = HingeProp::symmetric(
        "HD",
        HingeDof::M3,
        [
            HingePoint::new(1.0, 0.0),
            HingePoint::new(1.1, 0.02),
            HingePoint::new(0.2, 0.03),
            HingePoint::new(0.2, 0.08),
        ],
    );
    p.force_sf = MP;
    p.force_sf_negative = MP;
    p.beyond_e = Some(HingeBeyondE::DropsToZero);
    p
}

/// The backbone the solver uses, written out from the rule above: the hinge
/// moment (kN m) at the plastic rotation `theta` (rad), L = h of the
/// one-element column.
fn hd_backbone(theta: f64) -> f64 {
    let limit = 0.1 * EI / H;
    let (qc, qd, qe) = (0.02, 0.02 + 90.0 / limit, 0.02 + 90.0 / limit + 0.05);
    let qz = qe + 20.0 / limit;
    if theta <= qc {
        MP + (10.0 / 0.02) * theta
    } else if theta <= qd {
        110.0 - limit * (theta - qc)
    } else if theta <= qe {
        20.0
    } else if theta <= qz {
        20.0 - limit * (theta - qe)
    } else {
        0.0
    }
}

/// At EVERY saved state of the column under constant gravity: the base moment
/// of the results is lambda h + P u (moment equilibrium of the deformed
/// column, 1e-9 of Mp), the hinge moment lies on the backbone at its plastic
/// rotation (1e-9 of Mp), and the hinge moment is the base moment. Both
/// orientations, and also with an axial compression that grows with the push
/// (kappa = 2 kN per unit load factor, so N = 500 + 2 lambda and the moment is
/// lambda h + N u). The path must pass the hardening peak 1.1 Mp, the drop, the
/// flat part and the fall to zero.
#[test]
fn a_hardening_and_dropping_curve_keeps_equilibrium_and_the_backbone() {
    for (reversed, kappa) in [(false, 0.0), (true, 0.0), (false, 2.0)] {
        let mut c = Column::new(hardening_and_drop());
        c.reversed = reversed;
        c.kappa = kappa;
        c.target = 1.2;
        // Once the hinge is a pin, lambda = -N u / h with N = P + kappa lambda,
        // so the fixed point on N contracts by kappa u / h = 0.6 per iteration
        // (a property of the model): 1e-12 needs about 60 iterations.
        c.max_iterations = 200;
        c.saved = ResultsSaved::MultipleStates {
            min_saved: 40,
            max_saved: 200,
            positive_only: false,
        };
        let m = c.run_model();
        let case = &m.case_results["PUSH"];
        let top = c.top();
        let mut peak = 0.0f64;
        let mut max_theta = 0.0f64;
        assert!(case.static_steps.len() >= 40, "{} states", case.static_steps.len());
        for (k, s) in case.static_steps.iter().enumerate() {
            let u = s.joint_displacements[top.as_str()][0];
            let hinge = &s.hinges[0];
            let theta = hinge.plastic_deformation.abs();
            // Base moment of the deformed column: the support reaction opposes
            // the moment of the load and of the gravity about the base.
            close_abs(
                s.base_reactions[4],
                -(s.load_factor * H + (P + kappa * s.load_factor) * u),
                1e-9 * MP,
                &format!("base moment at state {k} (reversed {reversed}, kappa {kappa})"),
            );
            close_abs(
                hinge.force.abs(),
                s.base_reactions[4].abs(),
                1e-9 * MP,
                &format!("hinge moment is the base moment at state {k}"),
            );
            if theta > 0.0 {
                close_abs(
                    hinge.force.abs(),
                    hd_backbone(theta),
                    1e-9 * MP,
                    &format!("hinge moment on the backbone at theta = {theta} (state {k})"),
                );
            }
            peak = peak.max(hinge.force.abs());
            max_theta = max_theta.max(theta);
        }
        close(peak, 1.1 * MP, 1e-9, "the hardening peak (kN m)");
        assert!(max_theta > 0.235, "the hinge reached the zero force: {max_theta}");
        let last = case.static_steps.last().unwrap();
        close_abs(last.hinges[0].force, 0.0, 1e-9 * MP, "pinned at the end (kN m)");
        // Pendulum: lambda h + (P + kappa lambda) u = 0.
        close_abs(
            last.load_factor,
            -P * 1.2 / (H + kappa * 1.2),
            1e-9 * MP / H,
            "pendulum: lambda = -P u / (h + kappa u) (kN)",
        );
    }
}

// ---------------------------------------------------------------------------
// Independent reference for the first yield: the exact beam-column
// ---------------------------------------------------------------------------

/// The column in N equal frame objects, `GRAV` (P-Delta, 500 kN down) then
/// `PUSH` (P-Delta, lateral, displacement control), against the exact solution
/// of a cantilever with compression P and tip load H (no shear deformation):
///   mu = sqrt(P / EI), u = (H / P)(tan(mu h) / mu - h), M_base = H tan(mu h) / mu,
/// so the first yield is H1 = Mp mu / tan(mu h). The element is a cubic with a
/// consistent geometric stiffness, so the mesh converges to the exact value
/// from above in H1; the error falls from N = 8 to N = 16 (observed with this
/// code: H1 1.4e-9 and 9.1e-11, u1 1.7e-8 and 1.1e-9, a factor of 16 per
/// halving, as for a cubic). Asserted: 1e-7 (N = 8) and 1e-8 (N = 16)
/// relative, and an error ratio above 8. After the yield the mesh of 8 obeys
/// lambda = (Mp - P u) / h.
#[test]
fn a_mesh_of_8_and_16_frames_converges_to_the_exact_beam_column() {
    let mu = (P / EI).sqrt();
    let t = (mu * H).tan();
    let h1 = MP * mu / t;
    let u1 = (h1 / P) * (t / mu - H);
    let mut errors = Vec::new();
    for (n, tol) in [(8usize, 1e-7), (16, 1e-8)] {
        let mut c = Column::new(moment_hinge());
        c.n = n;
        let states = c.run();
        assert!(states.len() >= 4);
        let (l1, uu1) = states[1];
        let (el, eu) = ((l1 - h1).abs() / h1, (uu1 - u1).abs() / u1);
        eprintln!("N = {n}: H1 = {l1} (exact {h1}), error {el:e}; u1 = {uu1} (exact {u1}), error {eu:e}");
        assert!(el <= tol, "N = {n}: H1 error {el}");
        assert!(eu <= tol, "N = {n}: u1 error {eu}");
        errors.push((el, eu));
        for &(lambda, u) in &states[1..] {
            close_abs(lambda, (MP - P * u) / H, 1e-9 * MP / H, &format!("N = {n}: lambda after yield"));
        }
    }
    assert!(errors[1].0 < errors[0].0 / 8.0, "{errors:?}");
    assert!(errors[1].1 < errors[0].1 / 8.0, "{errors:?}");
}

// ---------------------------------------------------------------------------
// Reference test 4: Example 1-026 with P-Delta and no axial force
// ---------------------------------------------------------------------------

/// Example 1-026, model A (CSI Software Verification for SAP2000 v19,
/// kip and inch): the horizontal cantilever with a moment hinge and a shear
/// hinge at the fixed end and a vertical tip load carries no axial force, so
/// P-Delta changes nothing: the saved states, joint displacements and hinge
/// deformations equal those of the case without P-Delta to 1e-12.
fn example_1_026_a(p_delta: bool) -> SapModel {
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
        FrameHinge { prop: "Moment".into(), rel_dist: 0.0 },
        FrameHinge { prop: "Shear".into(), rel_dist: 0.0 },
    ];
    let mut c = LoadCase::linear_static("PUSH", "P", 1.0);
    c.case_type = LoadCaseType::NonlinearStatic;
    if p_delta {
        c.nl_geom = NLGeomType::PDelta;
        c.convergence_tol = 1e-12;
    }
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
fn t4_example_1_026_a_with_p_delta_equals_the_case_without() {
    let mut plain = example_1_026_a(false);
    let mut pd = example_1_026_a(true);
    analyze_model(&mut plain).unwrap();
    analyze_model(&mut pd).unwrap();
    let (a, b) = (&plain.case_results["PUSH"], &pd.case_results["PUSH"]);
    assert_eq!(a.static_steps.len(), b.static_steps.len());
    assert!(a.static_steps.len() >= 8);
    for (x, y) in a.static_steps.iter().zip(&b.static_steps) {
        close_abs(x.load_factor, y.load_factor, 1e-12 * x.load_factor.abs().max(1.0), "P (kip)");
        for j in ["1", "2"] {
            for d in 0..6 {
                close_abs(
                    x.joint_displacements[j][d],
                    y.joint_displacements[j][d],
                    1e-12 * x.joint_displacements[j][d].abs().max(1.0),
                    "joint displacement",
                );
            }
        }
        for (hx, hy) in x.hinges.iter().zip(&y.hinges) {
            close_abs(hx.force, hy.force, 1e-12 * hx.force.abs().max(1.0), "hinge force");
            close_abs(
                hx.plastic_deformation,
                hy.plastic_deformation,
                1e-12 * hx.plastic_deformation.abs().max(1.0),
                "plastic deformation",
            );
            assert_eq!(hx.state, hy.state);
        }
    }
    // The case without P-Delta is the one `wave6_hinge_end_kink` pins to the
    // published points of the example and to the values of 8ca1d44.
    close_abs(
        a.hinge_results[0].plastic_deformation,
        b.hinge_results[0].plastic_deformation,
        1e-12,
        "final plastic rotation (rad)",
    );
}

// ---------------------------------------------------------------------------
// Accel and Mode loads with P-Delta (a model without hinges is on the hinge path)
// ---------------------------------------------------------------------------

/// The column without hinges, a point mass of 3 at the top, `GRAV` as above;
/// `PUSH` applies the loads of `kind` with load control and saves states.
fn special_load_model(kind: &str, sf: f64) -> SapModel {
    let mut m = Column::new(moment_hinge()).build();
    m.frames["F1"].hinges.clear();
    m.points["P1"].mass = [3.0, 3.0, 3.0, 0.0, 0.0, 0.0];
    if kind == "mode" {
        m.load_cases
            .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 1));
    }
    let push = &mut m.load_cases["PUSH"];
    push.displacement_control = None;
    push.loads.clear();
    match kind {
        "accel" => push.loads.push(LoadCaseAssignment {
            load_name: "UX".into(),
            scale_factor: sf,
        }),
        _ => {
            push.modal_case = Some("MODAL".into());
            push.nl_static.mode_loads.push(ModeLoad {
                mode: 1,
                scale_factor: sf,
            });
        }
    }
    m
}

/// The same column with the load given as a pattern force at the top (the
/// existing P-Delta path: no hinges, no Accel or Mode load, final state).
fn pattern_load_model(force: f64) -> SapModel {
    let mut c = Column::new(moment_hinge());
    c.saved = ResultsSaved::FinalState;
    let mut m = c.build();
    m.frames["F1"].hinges.clear();
    let push = &mut m.load_cases["PUSH"];
    push.displacement_control = None;
    push.loads[0].scale_factor = force;
    m
}

fn assert_same_final_state(a: &CaseResults, b: &CaseResults) {
    for k in 0..6 {
        close_abs(
            a.joint_displacements["P1"][k],
            b.joint_displacements["P1"][k],
            1e-9 * b.joint_displacements["P1"][0].abs().max(b.joint_displacements["P1"][2].abs()),
            "top displacement",
        );
        close_abs(a.joint_reactions["P0"][k], b.joint_reactions["P0"][k], 1e-9 * 1.0e3, "base reaction");
    }
    for (x, y) in a.frame_forces["F1"].iter().zip(&b.frame_forces["F1"]) {
        close_abs(x.p, y.p, 1e-9 * 500.0, "axial force");
        close_abs(x.m3, y.m3, 1e-9 * 100.0, "moment");
    }
}

/// An Accel UX load of scale -1 on a mass of 3 is a force of 3 kN at the top,
/// so the P-Delta response must equal that of the pattern force of 3 kN on the
/// existing P-Delta path (a separate code path), with 500 kN of gravity.
#[test]
fn an_accel_load_with_p_delta_equals_the_same_force_as_a_pattern() {
    let mut m = special_load_model("accel", -1.0);
    analyze_model(&mut m).unwrap();
    let mut reference = pattern_load_model(3.0);
    analyze_model(&mut reference).unwrap();
    let (a, b) = (&m.case_results["PUSH"], &reference.case_results["PUSH"]);
    assert_same_final_state(a, b);
    // The P-Delta effect is there: the elastic top deflection of 3 kN is
    // amplified (the exact beam-column amplification, cubic element: 1e-4).
    let mu = (P / EI).sqrt();
    let exact = (3.0 / P) * ((mu * H).tan() / mu - H);
    close(a.joint_displacements["P1"][0], exact, 1e-4, "exact beam-column deflection (m)");
    let linear = 3.0 * H.powi(3) / (3.0 * EI);
    assert!(a.joint_displacements["P1"][0] > 1.05 * linear);
    // The hinge path keeps the saved states with P-Delta (a hinge-free model
    // with an Accel load is on it).
    assert!(!a.static_steps.is_empty());
    // Load factor 1 at the end of the case (load control).
    close(a.displacement_control_factor.unwrap(), 1.0, 1e-12, "load factor");
}

/// A Mode load with P-Delta: the load is sf omega^2 M phi of the modal case
/// (omega^2 = 1 / (m c), phi = 1 / sqrt(m) at the top, c the elastic tip
/// flexibility), applied through the stiffness with the P-Delta of the chain;
/// the pattern force of the same value on the existing path must agree.
#[test]
fn a_mode_load_with_p_delta_equals_the_same_force_as_a_pattern() {
    let sf = 2.0;
    let mut m = special_load_model("mode", sf);
    analyze_model(&mut m).unwrap();
    let modal = &m.case_results["MODAL"];
    let omega2 = modal.modes[0].circ_freq_rad_s.powi(2);
    let phi = modal.modes[0].mode_shape["P1"][0];
    let force = sf * omega2 * 3.0 * phi;
    assert!(force.abs() > 1e-3, "{force}");
    let mut reference = pattern_load_model(force);
    analyze_model(&mut reference).unwrap();
    assert_same_final_state(&m.case_results["PUSH"], &reference.case_results["PUSH"]);
    assert!(!m.case_results["PUSH"].static_steps.is_empty());
}

// ---------------------------------------------------------------------------
// Negative tests: what stays refused
// ---------------------------------------------------------------------------

#[test]
fn large_displacements_with_hinges_stay_refused() {
    let mut m = Column::new(moment_hinge()).build();
    m.load_cases["PUSH"].nl_geom = NLGeomType::LargeDisplacements;
    m.load_cases["PUSH"].displacement_control = None;
    let msg = refused(&mut m);
    assert!(msg.contains("PUSH") && msg.contains("large displacements"), "{msg}");
}

#[test]
fn a_mixed_chain_is_refused_naming_both_cases() {
    // GRAV without P-Delta, PUSH with, and the reverse.
    for (grav, push) in [
        (NLGeomType::None, NLGeomType::PDelta),
        (NLGeomType::PDelta, NLGeomType::None),
    ] {
        let mut m = Column::new(moment_hinge()).build();
        m.load_cases["GRAV"].nl_geom = grav;
        m.load_cases["PUSH"].nl_geom = push;
        m.load_cases["PUSH"].displacement_control = Some(DisplacementControl {
            target: 0.1,
            monitored: Some(("P1".into(), 0)),
            conjugate: false,
        });
        // Displacement control with P-Delta is allowed on the hinge path; with
        // none the case is the plain one.
        let msg = refused(&mut m);
        assert!(
            msg.contains("'PUSH'") && msg.contains("'GRAV'") && msg.contains("same setting"),
            "{msg}"
        );
    }
}

#[test]
fn an_interior_hinge_is_refused_in_a_p_delta_case_naming_the_frame() {
    let mut m = Column::new(moment_hinge()).build();
    m.frames["F1"].hinges[0].rel_dist = 0.5;
    let msg = refused(&mut m);
    assert!(
        msg.contains("frame 'F1'") && msg.contains("0.5") && msg.contains("a P-Delta case"),
        "{msg}"
    );
    // The same hinge without P-Delta is analyzed.
    let mut c = Column::new(moment_hinge());
    c.p_delta = false;
    let mut m = c.build();
    m.frames["F1"].hinges[0].rel_dist = 0.5;
    analyze_model(&mut m).unwrap();
}

#[test]
fn a_directly_specified_p_delta_force_is_refused_in_a_p_delta_case() {
    let mut m = Column::new(moment_hinge()).build();
    m.frames["F1"].pdelta_force = -100.0;
    let msg = refused(&mut m);
    assert!(
        msg.contains("frame 'F1'") && msg.contains("directly specified P-Delta force"),
        "{msg}"
    );
    // Without P-Delta in the cases the directly specified force is as before.
    let mut c = Column::new(moment_hinge());
    c.p_delta = false;
    let mut m = c.build();
    m.frames["F1"].pdelta_force = -100.0;
    analyze_model(&mut m).unwrap();
}

/// The one-element cubic model has the critical load 5525 kN (the 2 x 2 top
/// stiffness 5.9259e7 - 11555.6 P + 0.15 P^2 vanishes; the exact value is
/// pi^2 EI / (4 h^2) = 5483 kN). Gravity of 7000 kN is beyond it: the
/// stiffness with the geometric stiffness of 7000 kN does not factor and the
/// case fails with the singular-matrix error that names the joint, with no
/// results.
#[test]
fn gravity_above_the_buckling_load_is_refused_with_no_results() {
    let mut c = Column::new(moment_hinge());
    c.gravity = 7000.0;
    let mut m = c.build();
    let e = analyze_model(&mut m).expect_err("beyond the buckling load");
    let text = format!("{e}");
    assert!(
        matches!(e, SolverError::Unstable { .. }) && text.contains("Singular matrix"),
        "{e:?}"
    );
    assert!(!m.case_results.contains_key("GRAV"));
    assert!(!m.case_results.contains_key("PUSH"));
    // Below it (5000 kN) the case runs.
    let mut c = Column::new(moment_hinge());
    c.gravity = 5000.0;
    c.target = 0.02;
    analyze_model(&mut c.build()).unwrap();
}

/// A model without hinges, Accel or Mode loads keeps the old P-Delta path: it
/// keeps the final state only and still refuses displacement control with
/// geometric nonlinearity.
#[test]
fn the_non_hinge_path_is_unchanged() {
    let mut c = Column::new(moment_hinge());
    c.saved = ResultsSaved::MultipleStates {
        min_saved: 4,
        max_saved: 50,
        positive_only: false,
    };
    let mut m = c.build();
    m.frames["F1"].hinges.clear();
    let msg = refused(&mut m);
    assert!(
        msg.contains("combines displacement control with geometric nonlinearity"),
        "{msg}"
    );
    let mut m = c.build();
    m.frames["F1"].hinges.clear();
    m.load_cases["PUSH"].displacement_control = None;
    analyze_model(&mut m).unwrap();
    assert!(m.case_results["PUSH"].static_steps.is_empty());
    assert!(m.case_results["GRAV"].static_steps.is_empty());
}

/// Saved states: a P-Delta case on the hinge path keeps them like any other
/// hinge-path case when it asks for multiple states, and keeps the final
/// state only when it does not.
#[test]
fn saved_states_of_p_delta_cases_on_the_hinge_path() {
    let mut c = Column::new(moment_hinge());
    c.saved = ResultsSaved::FinalState;
    let m = c.run_model();
    assert!(m.case_results["PUSH"].static_steps.is_empty());
    assert_eq!(m.case_results["PUSH"].hinge_results.len(), 1);
    let m = Column::new(moment_hinge()).run_model();
    let steps = &m.case_results["PUSH"].static_steps;
    assert!(steps.len() >= 4 && steps.len() <= 50, "{}", steps.len());
    // Step 0 is the start of the case (the gravity state), each step carries
    // the frame forces of its own axial force.
    close_abs(steps[0].load_factor, 0.0, 1e-15, "step 0");
    assert!(steps.iter().all(|s| !s.frame_forces.is_empty() && s.hinges.len() == 1));
}

/// The portal with a beam as weak as the columns (Mp = 100 everywhere) and a
/// heavy gravity (8000 and 12000 kN on each top joint, about 0.3 to 0.45 of the
/// sway buckling load of the columns). The column top and the beam end at a
/// joint yield together (their forces are equal and opposite, a series tie),
/// and the P-Delta correction has to settle the order of events: with 8000 kN
/// an active hinge turns round under the corrected stiffness (it is dropped
/// from the active set of the step), with 12000 kN the end-state equations are
/// singular and the hinge in series is dropped. Reference: the equilibrium of
/// every column at every saved state (1e-9 relative), the base shears and the
/// vertical reactions (1e-9), and the sway mechanism
/// lambda(u) = (4 Mp - 2 G u) / h (1e-6 relative, the beam is not exactly
/// rigid) with the column compressions G -/+ 2 Mp / b (1e-5).
#[test]
fn a_heavy_portal_with_a_weak_beam_settles_its_events() {
    for gravity in [8000.0, 12000.0] {
        let mut m = portal_with_gravity(0.3, 1e-9, gravity);
        m.hinge_props["BEAMM"] = epp("BEAMM", HingeDof::M3, MP);
        analyze_model(&mut m).unwrap();
        let case = &m.case_results["PUSH"];
        let states = &case.static_steps;
        assert!(states.len() >= 20, "{} states", states.len());
        for (k, s) in states.iter().enumerate() {
            close_abs(s.base_reactions[0], -s.load_factor, 1e-9 * s.load_factor.abs().max(1.0), "base shear");
            close_abs(s.base_reactions[2], 2.0 * gravity, 1e-9 * 2.0 * gravity, "vertical reactions");
            for (c, base, top) in [("C1", "B1", "T1"), ("C2", "B2", "T2")] {
                let sta = &s.frame_forces[c];
                let (a, b) = (&sta[0], sta.last().unwrap());
                let n = -0.5 * (a.p + b.p);
                let delta = s.joint_displacements[top][0] - s.joint_displacements[base][0];
                let lhs = (b.m3 - a.m3) + a.v2 * H;
                close_abs(
                    lhs + n * delta,
                    0.0,
                    1e-9 * (n * delta).abs() + 1e-9,
                    &format!("{c} equilibrium at state {k} (gravity {gravity})"),
                );
            }
        }
        let last = states.last().unwrap();
        close_abs(last.joint_displacements["T1"][0], 0.3, 1e-12, "target");
        close(last.load_factor, (4.0 * MP - 2.0 * gravity * 0.3) / H, 1e-6, "sway load (kN)");
        let n1 = -0.5 * (last.frame_forces["C1"][0].p + last.frame_forces["C1"].last().unwrap().p);
        let n2 = -0.5 * (last.frame_forces["C2"][0].p + last.frame_forces["C2"].last().unwrap().p);
        close(n1, gravity - 2.0 * MP / PB, 1e-5, "N1 (kN)");
        close(n2, gravity + 2.0 * MP / PB, 1e-5, "N2 (kN)");
        for h in last.hinges.iter().filter(|h| h.frame != "BM") {
            close(h.force.abs(), MP, 1e-9, "column hinge moment (kN m)");
        }
    }
}
