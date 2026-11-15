//! Ritz starting loads, generation cycles, dynamic-participation targets and
//! the static/dynamic load-participation tables.
//!
//! Source: CSI Analysis Reference Manual, "Ritz-Vector Analysis" (printed
//! pp. 372-375) and "Static and Dynamic Load Participation Ratios"
//! (pp. 378-381).
//!
//! Model: a vertical cantilever column along +Z, fixed at the base, flexural
//! only. Only UX and RY are active, the section has zero shear areas, and the
//! frame has no self mass, so the only mass is the lumped UX mass at each
//! free joint. Units: kip, in, s (mass in kip s^2/in). EI = E*1 in^4.
//! Expected values are closed forms evaluated in the tests.

use sap_model::ritz::RitzLoadControl;
use sap_model::*;
use sap_solver::{analyze_model, SolverError};

const EI: f64 = 1.0e6; // kip in^2
const L: f64 = 100.0; // in, spacing of the masses

fn cantilever(masses: &[f64]) -> SapModel {
    let mut m = SapModel::default();
    m.load_patterns["DEAD"].self_wt_multiplier = 0.0;
    m.load_cases.clear();
    m.active_dofs = [true, false, false, false, true, false];
    let mut mat = Material::default_steel_kip_in();
    mat.name = "M".into();
    mat.e = EI;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0;
    m.materials.insert("M".into(), mat);
    m.frame_sections.insert(
        "S".into(),
        FrameSection {
            name: "S".into(),
            material: "M".into(),
            area: 1.0e3,
            i33: 1.0,
            i22: 1.0,
            as2: 0.0,
            as3: 0.0,
            ..FrameSection::default()
        },
    );
    m.add_point("P0", 0.0, 0.0, 0.0);
    m.points["P0"].restraint = [true; 6];
    for (i, mass) in masses.iter().enumerate() {
        let name = format!("P{}", i + 1);
        m.add_point(name.clone(), 0.0, 0.0, L * (i as f64 + 1.0));
        m.points[&name].mass[0] = *mass;
        m.add_frame(format!("F{}", i + 1), format!("P{i}"), name, "S");
    }
    m
}

fn add_pattern(m: &mut SapModel, name: &str, joint_forces: &[(&str, [f64; 6])]) {
    m.load_patterns.insert(
        name.into(),
        LoadPattern {
            name: name.into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
    for (joint, force) in joint_forces {
        m.points[*joint].loads_force.push(PointLoadForce {
            load_pat: name.into(),
            csys: "GLOBAL".into(),
            force: *force,
        });
    }
}

fn ritz_case<'a>(m: &'a mut SapModel, loads: &[(&str, f64)], max_modes: usize) -> &'a mut LoadCase {
    let mut c = LoadCase::modal_eigen("R", max_modes);
    c.modal_sub_type = ModalSubType::Ritz;
    c.loads = loads
        .iter()
        .map(|(n, sf)| LoadCaseAssignment {
            load_name: (*n).into(),
            scale_factor: *sf,
        })
        .collect();
    m.load_cases.insert("R".into(), c);
    m.load_cases.get_mut("R").unwrap()
}

fn control(name: &str, cycles: usize, target: f64) -> RitzLoadControl {
    RitzLoadControl {
        load_name: name.into(),
        max_cycles: cycles,
        target_dynamic_participation: target,
    }
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(f64::MIN_POSITIVE)
}

/// Flexibility of the cantilever between loaded points at heights `a` and `b`
/// (Euler-Bernoulli, tip-load influence): `min^2 (3 max - min) / (6 EI)`.
fn flex(a: f64, b: f64) -> f64 {
    let (lo, hi) = if a <= b { (a, b) } else { (b, a) };
    lo * lo * (3.0 * hi - lo) / (6.0 * EI)
}

fn flex_matrix(n: usize) -> Vec<Vec<f64>> {
    (0..n)
        .map(|i| {
            (0..n)
                .map(|j| flex(L * (i as f64 + 1.0), L * (j as f64 + 1.0)))
                .collect()
        })
        .collect()
}

fn mdot(m: &[f64], a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).zip(m).map(|((x, y), w)| x * y * w).sum()
}

/// Cumulative dynamic participation of unit acceleration after `k` Ritz
/// vectors, evaluated independently of the solver on the condensed mass-DOF
/// problem: the M-orthogonal projection of `r` on span{(F M)^j r}, squared
/// M-norm, over `r^T M r`.
fn expected_cumulative_dynamic(masses: &[f64], k: usize) -> f64 {
    let n = masses.len();
    let f = flex_matrix(n);
    let r = vec![1.0; n];
    let mut basis: Vec<Vec<f64>> = Vec::new();
    let mut prev = r.clone();
    for _ in 0..k {
        // u = F M prev
        let mut u: Vec<f64> = (0..n)
            .map(|i| (0..n).map(|j| f[i][j] * masses[j] * prev[j]).sum())
            .collect();
        for b in &basis {
            let proj = mdot(masses, &u, b);
            for i in 0..n {
                u[i] -= proj * b[i];
            }
        }
        let norm = mdot(masses, &u, &u).sqrt();
        for v in &mut u {
            *v /= norm;
        }
        prev = u.clone();
        basis.push(u);
    }
    basis
        .iter()
        .map(|b| mdot(masses, b, &r).powi(2))
        .sum::<f64>()
        / mdot(masses, &r, &r)
}

/// Eigen-decomposition of the symmetric 2x2 matrix `[[a, b], [b, d]]`:
/// eigenvalues descending, each with a unit eigenvector.
fn eig2(a: f64, b: f64, d: f64) -> [(f64, [f64; 2]); 2] {
    let mean = 0.5 * (a + d);
    let rad = (0.25 * (a - d) * (a - d) + b * b).sqrt();
    let mut out = [(mean + rad, [0.0; 2]), (mean - rad, [0.0; 2])];
    for (lam, v) in &mut out {
        // (A - lam I) v = 0  ->  v ~ (b, lam - a)
        let (x, y) = (b, *lam - a);
        let n = x.hypot(y);
        *v = [x / n, y / n];
    }
    out
}

// 1. Two lumped masses, UX start, two modes: the exact eigenvalues.
#[test]
fn ritz_ux_two_masses_reproduces_the_two_by_two_eigenproblem() {
    let m = 0.01; // kip s^2/in
    let mut model = cantilever(&[m, m]);
    ritz_case(&mut model, &[("UX", 1.0)], 2);
    analyze_model(&mut model).unwrap();
    let modes = &model.case_results["R"].modes;
    assert_eq!(modes.len(), 2);
    // F = c [[2, 5], [5, 16]], c = L^3 / (6 EI); omega^2 = 1 / (m c lambda).
    let c = L.powi(3) / (6.0 * EI);
    // eigenvalues of [[2,5],[5,16]]: trace 18, det 7 -> 9 +- sqrt(74)
    let lam = [9.0 + 74f64.sqrt(), 9.0 - 74f64.sqrt()];
    for (mode, l) in modes.iter().zip(lam) {
        let expected = 1.0 / (m * c * l);
        assert!(
            rel(mode.eigenvalue, expected) < 1e-10,
            "omega^2 {} vs {expected} (1/s^2)",
            mode.eigenvalue
        );
    }
    assert!(modes[0].eigenvalue < modes[1].eigenvalue);
}

// 2. Dynamic-participation target: one vector exactly when the ratio of the
// first vector reaches the target.
fn first_vector_ratio(masses: &[f64]) -> (f64, f64) {
    let n = masses.len();
    let f = flex_matrix(n);
    let r = vec![1.0; n];
    let u: Vec<f64> = (0..n)
        .map(|i| (0..n).map(|j| f[i][j] * masses[j] * r[j]).sum())
        .collect();
    let q = mdot(masses, &u, &r).powi(2) / (mdot(masses, &u, &u) * mdot(masses, &r, &r));
    // Rayleigh quotient of the single vector: u^T K u / u^T M u = u^T M r / u^T M u
    let w2 = mdot(masses, &u, &r) / mdot(masses, &u, &u);
    (q, w2)
}

#[test]
fn target_is_met_by_one_vector_or_needs_two() {
    for masses in [[0.01, 0.01], [0.01, 0.04], [0.02, 0.01]] {
        let (q, w2_single) = first_vector_ratio(&masses);
        assert!(q > 0.05 && q < 0.995, "test threshold {q} must be interior");
        // Target below the first vector's ratio: one vector.
        let mut below = cantilever(&masses);
        ritz_case(&mut below, &[("UX", 1.0)], 2)
            .ritz_controls
            .push(control("UX", 0, q - 0.01));
        analyze_model(&mut below).unwrap();
        let modes = &below.case_results["R"].modes;
        assert_eq!(modes.len(), 1, "target {} < ratio {q}", q - 0.01);
        assert!(rel(modes[0].eigenvalue, w2_single) < 1e-10);
        // Target above it: the second vector is needed, and two exist.
        let mut above = cantilever(&masses);
        ritz_case(&mut above, &[("UX", 1.0)], 2)
            .ritz_controls
            .push(control("UX", 0, q + 0.01));
        analyze_model(&mut above).unwrap();
        assert_eq!(above.case_results["R"].modes.len(), 2);
    }
    // Equal masses: the ratio is (28)^2 / (490 * 2) = 0.8 (hand check of the helper).
    let (q, _) = first_vector_ratio(&[0.01, 0.01]);
    assert!(rel(q, 0.8) < 1e-12);
}

#[test]
fn target_stops_generation_at_the_expected_vector_count_with_three_masses() {
    let masses = [0.01, 0.02, 0.015];
    let r1 = expected_cumulative_dynamic(&masses, 1);
    let r2 = expected_cumulative_dynamic(&masses, 2);
    let r3 = expected_cumulative_dynamic(&masses, 3);
    assert!(r1 < r2 && r2 < r3 && rel(r3, 1.0) < 1e-12);
    for (target, count) in [
        (0.5 * (r1 + r2), 2usize),
        (r1 - 1e-3, 1),
        (0.5 * (r2 + r3), 3),
    ] {
        let mut model = cantilever(&masses);
        ritz_case(&mut model, &[("UX", 1.0)], 3)
            .ritz_controls
            .push(control("UX", 0, target));
        analyze_model(&mut model).unwrap();
        assert_eq!(
            model.case_results["R"].modes.len(),
            count,
            "target {target} with cumulative ratios {r1}, {r2}, {r3}"
        );
    }
}

// 3. Static participation of a load-pattern start sums to one (p. 380), and
// each mode's ratio matches the closed form for the exact 2x2 modes.
#[test]
fn static_participation_of_a_pattern_start_sums_to_one() {
    let m = 0.01;
    let (p1, p2) = (3.0, 10.0); // kip, global X at the two joints
    let mut model = cantilever(&[m, m]);
    add_pattern(
        &mut model,
        "LAT",
        &[
            ("P1", [p1, 0.0, 0.0, 0.0, 0.0, 0.0]),
            ("P2", [p2, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ],
    );
    ritz_case(&mut model, &[("LAT", 1.0)], 2);
    analyze_model(&mut model).unwrap();
    let res = &model.case_results["R"];
    assert_eq!(res.modes.len(), 2);
    let part = &res.load_participation[0];
    assert_eq!(part.load_name, "LAT");
    assert!(!part.is_acceleration);
    assert_eq!(part.vectors_generated, 2);
    assert!(rel(part.static_cumulative[1], 1.0) < 1e-10);
    // Exact modes: with A = [[2,5],[5,16]], mode n has psi_n, lambda_n and
    // static ratio lambda_n (psi_n.p)^2 / (p.A.p); dynamic ratio (psi_n.p)^2/(p.p).
    let eig = eig2(2.0, 5.0, 16.0);
    let pap = 2.0 * p1 * p1 + 10.0 * p1 * p2 + 16.0 * p2 * p2;
    let pp = p1 * p1 + p2 * p2;
    for (n, (lam, psi)) in eig.iter().enumerate() {
        let fp = psi[0] * p1 + psi[1] * p2;
        assert!(rel(part.static_ratio[n], lam * fp * fp / pap) < 1e-9);
        assert!(rel(part.dynamic_ratio[n], fp * fp / pp) < 1e-9);
    }
    assert!(rel(part.dynamic_cumulative[1], 1.0) < 1e-10);

    // Three masses, two vectors: the modes are not exact, yet the static sum is one.
    let mut three = cantilever(&[0.01, 0.02, 0.015]);
    add_pattern(&mut three, "TIP", &[("P3", [7.0, 0.0, 0.0, 0.0, 0.0, 0.0])]);
    ritz_case(&mut three, &[("TIP", 1.0)], 2);
    analyze_model(&mut three).unwrap();
    let part = &three.case_results["R"].load_participation[0];
    assert_eq!(part.static_ratio.len(), 2);
    assert!(rel(part.static_cumulative[1], 1.0) < 1e-10);
    assert!(part.dynamic_cumulative[1] < 1.0);
}

// 4. Acceleration start: the cumulative dynamic participation is the
// cumulative participating mass ratio.
#[test]
fn acceleration_dynamic_participation_is_the_mass_participation_ratio() {
    let masses = [0.01, 0.02, 0.015];
    let mut model = cantilever(&masses);
    ritz_case(&mut model, &[("UX", 1.0)], 3);
    analyze_model(&mut model).unwrap();
    let res = &model.case_results["R"];
    let part = &res.load_participation[0];
    assert_eq!(part.load_name, "UX");
    assert!(part.is_acceleration);
    // Ritz vectors from UX span a subspace of growing dimension; after k
    // modes the cumulative ratio equals the independent projection value.
    let mut cum_ref = 0.0;
    for (k, mode) in res.modes.iter().enumerate() {
        cum_ref += mode.modal_mass_ratios[0];
        assert!(
            rel(part.dynamic_cumulative[k], cum_ref) < 1e-10,
            "mode {k}: {} vs mass ratio sum {cum_ref}",
            part.dynamic_cumulative[k]
        );
    }
    // The independent closed form holds after all three modes: all mass.
    assert!(rel(part.dynamic_cumulative[2], 1.0) < 1e-10);
    assert!(rel(part.static_cumulative[2], 1.0) < 1e-10);
    // One-vector truncation: the same model with max_modes = 1.
    let mut one = cantilever(&masses);
    ritz_case(&mut one, &[("UX", 1.0)], 1);
    analyze_model(&mut one).unwrap();
    let p1 = &one.case_results["R"].load_participation[0];
    assert!(
        rel(
            p1.dynamic_cumulative[0],
            expected_cumulative_dynamic(&masses, 1)
        ) < 1e-10
    );
    assert!(
        rel(
            one.case_results["R"].modes[0].modal_mass_ratios[0],
            expected_cumulative_dynamic(&masses, 1)
        ) < 1e-10
    );
}

// Eigen cases report UX participation against the exact 2x2 modes.
#[test]
fn eigen_case_reports_acceleration_participation() {
    let m = 0.01;
    let mut model = cantilever(&[m, m]);
    model
        .load_cases
        .insert("E".into(), LoadCase::modal_eigen("E", 2));
    analyze_model(&mut model).unwrap();
    let res = &model.case_results["E"];
    // Only UX has mass: UY and UZ are not reported.
    assert_eq!(res.load_participation.len(), 1);
    let part = &res.load_participation[0];
    assert_eq!(part.load_name, "UX");
    assert_eq!(part.vectors_generated, 0);
    let eig = eig2(2.0, 5.0, 16.0);
    // phi = psi / sqrt(m); phi^T M 1 = sqrt(m) sum(psi); dynamic = (sum psi)^2 / 2.
    // Static with p = m (1, 1): lambda (psi.1)^2 / (1.A.1) with 1.A.1 = 2 + 10 + 16.
    for (n, (lam, psi)) in eig.iter().enumerate() {
        let s = psi[0] + psi[1];
        assert!(rel(part.dynamic_ratio[n], s * s / 2.0) < 1e-9);
        assert!(rel(part.static_ratio[n], lam * s * s / 28.0) < 1e-9);
    }
    assert!(rel(part.dynamic_cumulative[1], 1.0) < 1e-10);
    assert!(rel(part.static_cumulative[1], 1.0) < 1e-10);
}

// Cycles, order and redundancy.
fn generated(model: &SapModel) -> Vec<usize> {
    model.case_results["R"]
        .load_participation
        .iter()
        .map(|p| p.vectors_generated)
        .collect()
}

#[test]
fn cycles_follow_load_order_and_the_last_cycle_is_partial() {
    let masses = [0.01, 0.02, 0.015, 0.012];
    let tip = [("P4", [5.0, 0.0, 0.0, 0.0, 0.0, 0.0])];
    let mid = [("P2", [5.0, 0.0, 0.0, 0.0, 0.0, 0.0])];
    // Two unlimited loads, three modes: A gets cycles 1 and 2, B only cycle 1.
    let mut model = cantilever(&masses);
    add_pattern(&mut model, "TIP", &tip);
    add_pattern(&mut model, "MID", &mid);
    ritz_case(&mut model, &[("TIP", 1.0), ("MID", 1.0)], 3);
    analyze_model(&mut model).unwrap();
    assert_eq!(generated(&model), vec![2, 1]);
    assert_eq!(model.case_results["R"].modes.len(), 3);
    // Reverse the order: MID gets the extra vector.
    let mut model = cantilever(&masses);
    add_pattern(&mut model, "TIP", &tip);
    add_pattern(&mut model, "MID", &mid);
    ritz_case(&mut model, &[("MID", 1.0), ("TIP", 1.0)], 3);
    analyze_model(&mut model).unwrap();
    assert_eq!(generated(&model), vec![2, 1]);
    assert_eq!(
        model.case_results["R"].load_participation[0].load_name,
        "MID"
    );
    // max_cycles = 1 on the first load: it makes one vector, the other the rest.
    let mut model = cantilever(&masses);
    add_pattern(&mut model, "TIP", &tip);
    add_pattern(&mut model, "MID", &mid);
    ritz_case(&mut model, &[("TIP", 1.0), ("MID", 1.0)], 3)
        .ritz_controls
        .push(control("TIP", 1, 0.0));
    analyze_model(&mut model).unwrap();
    assert_eq!(generated(&model), vec![1, 2]);
}

#[test]
fn redundant_starting_load_is_discarded_and_dropped_from_later_cycles() {
    let masses = [0.01, 0.02, 0.015];
    let mut model = cantilever(&masses);
    // A pattern proportional to the masses is the UX start in disguise.
    add_pattern(
        &mut model,
        "SELF",
        &[
            ("P1", [0.01, 0.0, 0.0, 0.0, 0.0, 0.0]),
            ("P2", [0.02, 0.0, 0.0, 0.0, 0.0, 0.0]),
            ("P3", [0.015, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ],
    );
    ritz_case(&mut model, &[("UX", 1.0), ("SELF", 1.0)], 3);
    analyze_model(&mut model).unwrap();
    assert_eq!(generated(&model), vec![3, 0]);
    assert_eq!(model.case_results["R"].modes.len(), 3);
}

// The dynamic participation of a load counts every vector generated, whichever
// load made it (R^D sums over all modes, p. 380-381).
#[test]
fn target_counts_vectors_made_by_other_loads() {
    let masses = [0.01, 0.02, 0.015, 0.012];
    let n = masses.len();
    let f = flex_matrix(n);
    let p_tip = 5.0; // kip at the top joint
                     // Condensed mass-DOF parts of the first two Ritz vectors: UX start, then
                     // the TIP pattern start (cycle 1 order), M-orthonormalized.
    let u1: Vec<f64> = (0..n)
        .map(|i| (0..n).map(|j| f[i][j] * masses[j]).sum())
        .collect();
    let u2: Vec<f64> = (0..n).map(|i| f[i][n - 1] * p_tip).collect();
    let nrm1 = mdot(&masses, &u1, &u1).sqrt();
    let b1: Vec<f64> = u1.iter().map(|v| v / nrm1).collect();
    let proj = mdot(&masses, &u2, &b1);
    let w: Vec<f64> = u2.iter().zip(&b1).map(|(a, b)| a - proj * b).collect();
    let nrm2 = mdot(&masses, &w, &w).sqrt();
    let b2: Vec<f64> = w.iter().map(|v| v / nrm2).collect();
    let mut p = vec![0.0; n];
    p[n - 1] = p_tip;
    let a_p = p_tip * p_tip / masses[n - 1];
    let from_b1 = b1.iter().zip(&p).map(|(a, b)| a * b).sum::<f64>().powi(2) / a_p;
    let from_b2 = b2.iter().zip(&p).map(|(a, b)| a * b).sum::<f64>().powi(2) / a_p;
    let combined = from_b1 + from_b2;
    assert!(
        from_b1 > 0.02 && from_b2 + 0.02 < combined,
        "{from_b1} {from_b2}"
    );

    let build = |target: f64| {
        let mut model = cantilever(&masses);
        add_pattern(
            &mut model,
            "TIP",
            &[("P4", [p_tip, 0.0, 0.0, 0.0, 0.0, 0.0])],
        );
        ritz_case(&mut model, &[("UX", 1.0), ("TIP", 1.0)], 4)
            .ritz_controls
            .push(control("TIP", 0, target));
        analyze_model(&mut model).unwrap();
        model
    };
    // Without a target TIP generates in cycles 1 and 2: UX 2 + TIP 2 vectors.
    assert_eq!(generated(&build(0.0)), vec![2, 2]);
    // Target between TIP's own vector alone and both vectors: it needs UX's
    // vector counted, and then stops after its first vector.
    let target = from_b2 + 0.5 * from_b1;
    assert_eq!(generated(&build(target)), vec![3, 1]);
    // A target already met by UX's first vector alone still leaves TIP its
    // cycle-1 vector (its static solution), and no more.
    assert_eq!(generated(&build(0.5 * from_b1)), vec![3, 1]);
}

// Imported Ritz cases name accelerations U1, U2, U3.
#[test]
fn imported_acceleration_names_are_starting_loads() {
    let masses = [0.01, 0.02];
    let mut named = cantilever(&masses);
    ritz_case(&mut named, &[("UX", 1.0)], 2);
    analyze_model(&mut named).unwrap();
    let mut imported = cantilever(&masses);
    ritz_case(&mut imported, &[("U1", 1.0)], 2);
    analyze_model(&mut imported).unwrap();
    for (a, b) in named.case_results["R"]
        .modes
        .iter()
        .zip(&imported.case_results["R"].modes)
    {
        assert_eq!(a.eigenvalue.to_bits(), b.eigenvalue.to_bits());
    }
    // Rotational accelerations are refused by name.
    let mut rot = cantilever(&masses);
    ritz_case(&mut rot, &[("U4", 1.0)], 2);
    refused(rot, "rotational acceleration");
}

// 5. Legacy path: no starting loads keeps the old unit-acceleration start.
// The bit patterns were recorded by running this exact model through the
// code of commit 2238221 (before this change).
#[test]
fn legacy_ritz_without_starting_loads_is_bitwise_unchanged() {
    let mut model = cantilever(&[0.01, 0.02, 0.015]);
    let mut c = LoadCase::modal_eigen("R", 3);
    c.modal_sub_type = ModalSubType::Ritz;
    model.load_cases.insert("R".into(), c);
    analyze_model(&mut model).unwrap();
    let recorded: [[u64; 3]; 3] = [
        [0x4015663b9b0bb992, 0x401ba3363c872aea, 0x3fc819a9d087f2f8],
        [0x406d19fcfd5fd8e7, 0x4010cf831e512262, 0xbfb5ac6f8f53adae],
        [0x40a15ec0c3979046, 0x3ff1fe7a05640e5f, 0x3fa8fcb0404b9af0],
    ];
    let modes = &model.case_results["R"].modes;
    assert_eq!(modes.len(), 3);
    for (mode, want) in modes.iter().zip(recorded) {
        assert_eq!(mode.eigenvalue.to_bits(), want[0]);
        assert_eq!(mode.mode_shape["P3"][0].to_bits(), want[1]);
        assert_eq!(mode.participation_factors[0].to_bits(), want[2]);
    }
    // The table of the legacy path reports UX against these modes.
    let part = &model.case_results["R"].load_participation;
    assert_eq!(part.len(), 1);
    assert!(rel(part[0].dynamic_cumulative[2], 1.0) < 1e-10);
}

// 6. Refusals at analysis.
fn refused(mut model: SapModel, needle: &str) {
    let err = analyze_model(&mut model).expect_err("analysis must refuse");
    let text = err.to_string();
    assert!(
        matches!(
            err,
            SolverError::UnsupportedInput(_) | SolverError::InvalidInput(_)
        ),
        "unexpected error kind: {text}"
    );
    assert!(text.contains(needle), "'{text}' should mention '{needle}'");
}

#[test]
fn ritz_refusals_name_the_offending_item() {
    let base = || {
        let mut m = cantilever(&[0.01, 0.02]);
        add_pattern(&mut m, "LAT", &[("P2", [1.0, 0.0, 0.0, 0.0, 0.0, 0.0])]);
        m
    };
    // unknown load
    let mut m = base();
    ritz_case(&mut m, &[("NOPE", 1.0)], 2);
    refused(m, "NOPE");
    // a pattern literally named UX
    let mut m = base();
    add_pattern(&mut m, "UX", &[("P2", [1.0, 0.0, 0.0, 0.0, 0.0, 0.0])]);
    ritz_case(&mut m, &[("UX", 1.0)], 2);
    refused(m, "UX");
    // the same load twice
    let mut m = base();
    ritz_case(&mut m, &[("LAT", 1.0), ("LAT", 2.0)], 2);
    refused(m, "more than once");
    // control for a load that is not listed
    let mut m = base();
    ritz_case(&mut m, &[("LAT", 1.0)], 2)
        .ritz_controls
        .push(control("UX", 1, 0.0));
    refused(m, "not one of its starting loads");
    // targets outside [0, 1]
    for bad in [1.5, -0.1, f64::NAN] {
        let mut m = base();
        ritz_case(&mut m, &[("LAT", 1.0)], 2)
            .ritz_controls
            .push(control("LAT", 0, bad));
        refused(m, "must be in [0, 1]");
    }
    // two controls for one load
    let mut m = base();
    let c = ritz_case(&mut m, &[("LAT", 1.0)], 2);
    c.ritz_controls.push(control("LAT", 1, 0.0));
    c.ritz_controls.push(control("LAT", 2, 0.0));
    refused(m, "two generation controls");
    // a load on a joint without mass DOF components: a moment about RY, which has no inertia
    let mut m = base();
    add_pattern(&mut m, "MOM", &[("P2", [0.0, 0.0, 0.0, 0.0, 5.0, 0.0])]);
    ritz_case(&mut m, &[("MOM", 1.0)], 2);
    refused(m, "no component on a mass degree of freedom");
    // a load on the restrained base joint
    let mut m = base();
    add_pattern(&mut m, "BASE", &[("P0", [4.0, 0.0, 0.0, 0.0, 0.0, 0.0])]);
    ritz_case(&mut m, &[("BASE", 1.0)], 2);
    refused(m, "no component on a mass degree of freedom");
    // UY excites no mass: there is none in Y
    let mut m = base();
    ritz_case(&mut m, &[("UY", 1.0)], 2);
    refused(m, "no component on a mass degree of freedom");
    // zero scale factor
    let mut m = base();
    ritz_case(&mut m, &[("LAT", 0.0)], 2);
    refused(m, "no component on a mass degree of freedom");
    // Eigen cases take neither controls nor starting loads
    let mut m = base();
    let mut e = LoadCase::modal_eigen("R", 2);
    e.ritz_controls.push(control("UX", 1, 0.0));
    m.load_cases.insert("R".into(), e);
    refused(m, "Eigen");
    let mut m = base();
    let mut e = LoadCase::modal_eigen("R", 2);
    e.loads.push(LoadCaseAssignment {
        load_name: "LAT".into(),
        scale_factor: 1.0,
    });
    m.load_cases.insert("R".into(), e);
    refused(m, "Eigen");
    // a minimum mode count above one is not implemented for starting loads
    let mut m = base();
    ritz_case(&mut m, &[("LAT", 1.0)], 2).min_modes = 2;
    refused(m, "minimum");
}

// Rotated joint axes: the UX start is a global direction.
#[test]
fn acceleration_start_uses_global_directions_with_rotated_joint_axes() {
    let masses = [0.01, 0.02];
    let mut plain = cantilever(&masses);
    ritz_case(&mut plain, &[("UX", 1.0)], 2);
    analyze_model(&mut plain).unwrap();
    // All joints turned 180 degrees about Z: joint 1 now points along global
    // -X. The structure is unchanged, so UX still gives the same modes and
    // the same participation.
    let mut turned = cantilever(&masses);
    for point in turned.points.values_mut() {
        point.local_axes_deg = [180.0, 0.0, 0.0];
    }
    ritz_case(&mut turned, &[("UX", 1.0)], 2);
    analyze_model(&mut turned).unwrap();
    let (a, b) = (&plain.case_results["R"], &turned.case_results["R"]);
    for (x, y) in a.modes.iter().zip(&b.modes) {
        assert!(rel(x.eigenvalue, y.eigenvalue) < 1e-10);
    }
    for k in 0..2 {
        assert!(
            rel(
                a.load_participation[0].dynamic_ratio[k],
                b.load_participation[0].dynamic_ratio[k]
            ) < 1e-10
        );
    }
    // Joints turned 90 degrees: global X is local 2, which is inactive, so
    // UX has no component on a mass degree of freedom and is refused.
    let mut quarter = cantilever(&masses);
    for point in quarter.points.values_mut() {
        point.local_axes_deg = [90.0, 0.0, 0.0];
    }
    ritz_case(&mut quarter, &[("UX", 1.0)], 2);
    refused(quarter, "no component on a mass degree of freedom");
}
