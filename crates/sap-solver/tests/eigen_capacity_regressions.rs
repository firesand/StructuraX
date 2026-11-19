//! Regression tests for the profile LDL^T factorization and the subspace
//! eigen solver of `sap_solver::linalg` (work package P1).
//!
//! Expected values come from closed forms evaluated in the tests, from the
//! printed CSI Example 1-021 results (Peterson 1981), or from the verbatim
//! copy of the previous dense algorithm at the end of this file.

use sap_model::{LoadCase, Material, SapModel};
use sap_solver::analyze_model;
use sap_solver::linalg::{
    solve_modal_eigen, solve_modal_eigen_checked, solve_modal_eigen_subspace, LinalgError,
    SymmetricLdlt,
};

// ---------------------------------------------------------------------------
// Deterministic random numbers (simple LCG, no external crate).
// ---------------------------------------------------------------------------

struct Lcg(u64);

impl Lcg {
    fn next_u32(&mut self) -> u32 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 33) as u32
    }
    /// Uniform in [0, 1).
    fn uniform(&mut self) -> f64 {
        f64::from(self.next_u32()) / f64::from(1u32 << 31)
    }
    /// Uniform in [-1, 1).
    fn signed(&mut self) -> f64 {
        2.0 * self.uniform() - 1.0
    }
    fn below(&mut self, n: usize) -> usize {
        (self.next_u32() as usize) % n
    }
}

/// Symmetric, strictly diagonally dominant (hence SPD) matrix whose row `i`
/// couples to columns `i - band[i] ..= i - 1`. Entries inside the band are
/// zeroed at random (zero gaps inside the profile).
fn banded_spd(n: usize, max_band: usize, rng: &mut Lcg) -> Vec<f64> {
    let mut k = vec![0.0; n * n];
    for i in 0..n {
        let band = rng.below(max_band + 1).min(i);
        for j in (i - band)..i {
            if rng.below(4) == 0 {
                continue;
            }
            let v = rng.signed();
            k[i * n + j] = v;
            k[j * n + i] = v;
        }
    }
    for i in 0..n {
        let off: f64 = (0..n).filter(|&j| j != i).map(|j| k[i * n + j].abs()).sum();
        k[i * n + i] = off + 0.5 + rng.uniform();
    }
    k
}

/// Block-diagonal SPD matrix of dense blocks of random size, then symmetric
/// off-block entries of exactly `-0.0` to exercise signed zeros.
fn block_spd(n: usize, rng: &mut Lcg) -> Vec<f64> {
    let mut k = vec![0.0; n * n];
    let mut start = 0;
    while start < n {
        let size = (1 + rng.below(24)).min(n - start);
        for i in start..start + size {
            for j in start..i {
                let v = rng.signed();
                k[i * n + j] = v;
                k[j * n + i] = v;
            }
        }
        for i in start..start + size {
            let off: f64 = (start..start + size)
                .filter(|&j| j != i)
                .map(|j| k[i * n + j].abs())
                .sum();
            k[i * n + i] = off + 1.0 + rng.uniform();
        }
        start += size;
    }
    for _ in 0..n {
        let (i, j) = (rng.below(n), rng.below(n));
        if i != j && k[i * n + j] == 0.0 {
            k[i * n + j] = -0.0;
            k[j * n + i] = -0.0;
        }
    }
    k
}

fn zero_row_and_column(k: &mut [f64], n: usize, i: usize) {
    for j in 0..n {
        k[i * n + j] = 0.0;
        k[j * n + i] = 0.0;
    }
}

/// Multiplies the diagonal of a few DOFs by a penalty-size factor.
fn inflate_diagonals(k: &mut [f64], n: usize, rng: &mut Lcg) -> Vec<f64> {
    let mut reference: Vec<f64> = (0..n).map(|i| k[i * n + i]).collect();
    for _ in 0..(n / 8).max(1) {
        let i = rng.below(n);
        k[i * n + i] *= 1.0e12;
    }
    // A reference mixing structural diagonals and zeros.
    for (i, r) in reference.iter_mut().enumerate() {
        if i % 7 == 3 {
            *r = 0.0;
        }
    }
    reference
}

fn bits(v: &[f64]) -> Vec<u64> {
    v.iter().map(|x| x.to_bits()).collect()
}

fn error_shape(e: &LinalgError) -> String {
    match e {
        LinalgError::SingularMatrix(i) => format!("singular {i}"),
        LinalgError::NonFinite(w) => format!("nonfinite {w}"),
        LinalgError::DimensionMismatch { expected, actual } => format!("dim {expected} {actual}"),
        other => format!("other {other}"),
    }
}

fn oracle_error_shape(e: &oracle::LinalgError) -> String {
    match e {
        oracle::LinalgError::SingularMatrix(i) => format!("singular {i}"),
        oracle::LinalgError::NonFinite(w) => format!("nonfinite {w}"),
        oracle::LinalgError::DimensionMismatch { expected, actual } => {
            format!("dim {expected} {actual}")
        }
        other => format!("other {other}"),
    }
}

/// Factorizes with the new and the verbatim old code and compares everything
/// bit for bit, including the solutions for several right-hand sides.
/// Returns the shared error when both factorizations failed (and agreed), `None` when both succeeded.
fn assert_identical(
    k: &[f64],
    n: usize,
    reference: Option<&[f64]>,
    rng: &mut Lcg,
    what: &str,
) -> Option<String> {
    let old = oracle::SymmetricLdlt::factorize_with_reference(k, n, reference);
    let new = SymmetricLdlt::factorize_with_reference(k, n, reference);
    match (old, new) {
        (Ok(old), Ok(new)) => {
            assert_eq!(bits(&old.l), bits(&new.l), "{what}: L");
            assert_eq!(bits(&old.d), bits(&new.d), "{what}: D");
            assert_eq!(old.zero_dof, new.zero_dof, "{what}: zero_dof");
            assert_eq!(old.sturm_count, new.sturm_count, "{what}: sturm_count");
            let mut rhs_list: Vec<Vec<f64>> = Vec::new();
            for _ in 0..4 {
                let mut b: Vec<f64> = (0..n).map(|_| rng.signed()).collect();
                // The load must not sit on a zero-stiffness DOF for the first two.
                if rhs_list.len() < 2 {
                    for i in 0..n {
                        if old.zero_dof[i] {
                            b[i] = 0.0;
                        }
                    }
                }
                rhs_list.push(b);
            }
            // Sparse load: a single entry, and an all-zero load.
            let mut single = vec![0.0; n];
            single[rng.below(n)] = 3.5;
            rhs_list.push(single);
            rhs_list.push(vec![0.0; n]);
            for (index, b) in rhs_list.iter().enumerate() {
                match (old.solve(b), new.solve(b)) {
                    (Ok(xo), Ok(xn)) => {
                        assert_eq!(bits(&xo), bits(&xn), "{what}: solution {index}");
                    }
                    (Err(eo), Err(en)) => {
                        assert_eq!(
                            oracle_error_shape(&eo),
                            error_shape(&en),
                            "{what}: solve error {index}"
                        );
                    }
                    (o, nw) => panic!("{what}: solve {index} differs: {o:?} vs {nw:?}"),
                }
            }
            // A bad load on a zero DOF reports the same index.
            if let Some(z) = old.zero_dof.iter().position(|&z| z) {
                let mut b = vec![0.0; n];
                b[z] = 1.0;
                let eo = old.solve(&b).unwrap_err();
                let en = new.solve(&b).unwrap_err();
                assert_eq!(
                    oracle_error_shape(&eo),
                    error_shape(&en),
                    "{what}: zero dof"
                );
            }
        }
        (Err(eo), Err(en)) => {
            assert_eq!(oracle_error_shape(&eo), error_shape(&en), "{what}: error");
            return Some(error_shape(&en));
        }
        (o, nw) => panic!(
            "{what}: one factorization failed: old ok={} new ok={}",
            o.is_ok(),
            nw.is_ok()
        ),
    }
    None
}

#[test]
fn profile_ldlt_is_bitwise_identical_to_the_dense_algorithm() {
    let mut rng = Lcg(0x5eed_1234_abcd_0001);
    let mut cases = 0;
    for &n in &[50usize, 77, 120, 200, 331, 400] {
        for max_band in [0usize, 1, 3, 9, 30, n] {
            // Banded.
            let k = banded_spd(n, max_band, &mut rng);
            assert_identical(&k, n, None, &mut rng, &format!("band n={n} bw={max_band}"));
            // Banded with zero rows (unused DOFs).
            let mut kz = k.clone();
            for _ in 0..(1 + n / 40) {
                zero_row_and_column(&mut kz, n, rng.below(n));
            }
            assert_identical(
                &kz,
                n,
                None,
                &mut rng,
                &format!("zero rows n={n} bw={max_band}"),
            );
            // Penalty-size diagonals with a reference diagonal.
            let mut kp = k.clone();
            let reference = inflate_diagonals(&mut kp, n, &mut rng);
            assert_identical(
                &kp,
                n,
                Some(&reference),
                &mut rng,
                &format!("penalty n={n} bw={max_band}"),
            );
            assert_identical(&kp, n, None, &mut rng, &format!("penalty/no ref n={n}"));
            // Reference equal to the structural diagonal of the untouched matrix.
            let plain: Vec<f64> = (0..n).map(|i| k[i * n + i]).collect();
            assert_identical(&k, n, Some(&plain), &mut rng, &format!("reference n={n}"));
            cases += 5;
        }
        // Block matrices, plain and with zero rows.
        let kb = block_spd(n, &mut rng);
        assert_identical(&kb, n, None, &mut rng, &format!("blocks n={n}"));
        let mut kbz = kb.clone();
        zero_row_and_column(&mut kbz, n, rng.below(n));
        zero_row_and_column(&mut kbz, n, 0);
        zero_row_and_column(&mut kbz, n, n - 1);
        assert_identical(&kbz, n, None, &mut rng, &format!("blocks+zero n={n}"));
        cases += 2;
    }
    assert!(cases > 40);
}

#[test]
fn profile_ldlt_reports_the_same_error_indices_as_the_dense_algorithm() {
    let mut rng = Lcg(0x5eed_1234_abcd_0002);
    let mut errors: Vec<String> = Vec::new();
    for &n in &[60usize, 150, 400] {
        for trial in 0..6 {
            let mut k = banded_spd(n, 12, &mut rng);
            // Mechanism: make one DOF an exact copy of another coupled one by
            // removing its diagonal dominance.
            let victim = 1 + rng.below(n - 1);
            let off: f64 = (0..n)
                .filter(|&j| j != victim)
                .map(|j| k[victim * n + j].abs())
                .sum();
            k[victim * n + victim] = off * 0.1 + 1e-14 * f64::from(trial);
            if trial % 2 == 0 {
                k[victim * n + victim] = 0.0;
                // Keep the row nonzero so it is not a zero DOF.
                let j = if victim > 0 { victim - 1 } else { 1 };
                k[victim * n + j] = 1.0;
                k[j * n + victim] = 1.0;
            }
            errors.extend(assert_identical(
                &k,
                n,
                None,
                &mut rng,
                &format!("singular n={n} trial={trial}"),
            ));
            let plain: Vec<f64> = (0..n).map(|i| k[i * n + i]).collect();
            errors.extend(assert_identical(
                &k,
                n,
                Some(&plain),
                &mut rng,
                &format!("singular/ref n={n} trial={trial}"),
            ));
        }
    }
    // The comparison must really have compared failures: many distinct
    // `SingularMatrix(index)` outcomes, all identical in old and new code.
    let mut distinct = errors.clone();
    distinct.sort();
    distinct.dedup();
    assert!(
        errors.len() >= 20 && distinct.len() >= 8,
        "only {} errors / {} distinct: {distinct:?}",
        errors.len(),
        distinct.len()
    );
    assert!(
        errors.iter().all(|e| e.starts_with("singular ")),
        "{errors:?}"
    );
    // Dimension and finiteness errors are untouched.
    let k = vec![1.0, 0.0, 0.0, f64::NAN];
    assert!(matches!(
        SymmetricLdlt::factorize(&k, 2),
        Err(LinalgError::NonFinite(_))
    ));
    assert!(matches!(
        SymmetricLdlt::factorize(&k[..3], 2),
        Err(LinalgError::DimensionMismatch { .. })
    ));
    let good = SymmetricLdlt::factorize(&[2.0, -1.0, -1.0, 2.0], 2).unwrap();
    assert!(matches!(
        good.solve(&[1.0]),
        Err(LinalgError::DimensionMismatch { .. })
    ));
    assert!(matches!(
        good.solve(&[1.0, f64::INFINITY]),
        Err(LinalgError::NonFinite(_))
    ));
}

#[test]
fn subspace_solver_refuses_mismatched_dimensions() {
    let (n, spring, mass) = (6usize, 1.0, 1.0);
    let (k, m) = chain_matrices(n, spring, mass);
    let ldlt = SymmetricLdlt::factorize(&k, n).unwrap();
    assert!(matches!(
        solve_modal_eigen_subspace(&k[..n * n - 1], &ldlt, &m, 2),
        Err(LinalgError::DimensionMismatch { .. })
    ));
    assert!(matches!(
        solve_modal_eigen_subspace(&k, &ldlt, &m[..n - 1], 2),
        Err(LinalgError::DimensionMismatch { .. })
    ));
    // Nothing to compute is not an error.
    assert!(solve_modal_eigen_subspace(&k, &ldlt, &m, 0)
        .unwrap()
        .is_empty());
    assert!(solve_modal_eigen_subspace(&k, &ldlt, &vec![0.0; n], 3)
        .unwrap()
        .is_empty());
}

// ---------------------------------------------------------------------------
// CSI Example 1-021 (Bathe and Wilson frame).
// ---------------------------------------------------------------------------

const BAYS: usize = 10;
const STOREYS: usize = 9;
const BAY_WIDTH: f64 = 20.0; // ft
const STOREY_HEIGHT: f64 = 10.0; // ft
const E_KSF: f64 = 432_000.0; // k/ft^2
const AREA_FT2: f64 = 3.0; // ft^2
const INERTIA_FT4: f64 = 1.0; // ft^4
const MASS_PER_LENGTH: f64 = 3.0; // k s^2 / ft per ft of member

/// Peterson 1981 eigenvalues omega^2 [1/s^2] as printed in CSI Example 1-021
/// (problem_1_021.pdf, page 2, "Comparison with Peterson 1981").
const EXAMPLE_1_021: [f64; 3] = [0.589541, 5.52696, 16.5879];

fn bathe_wilson_model() -> SapModel {
    let mut model = SapModel::default();
    model.load_cases.clear();
    model.active_dofs = [true, false, true, false, true, false];
    let mut material = Material::default_steel_kip_in();
    material.name = "BW".into();
    material.e = E_KSF;
    // Weight is zero and the density gives 3 k s^2/ft per unit length for A = 3 ft^2.
    material.unit_weight = 0.0;
    material.unit_mass = MASS_PER_LENGTH / AREA_FT2;
    model.materials.insert("BW".into(), material);
    let mut section = sap_section::make_rectangular("BW", "BW", 1.0, 3.0);
    section.area = AREA_FT2;
    section.i33 = INERTIA_FT4;
    section.i22 = INERTIA_FT4;
    section.torsional_j = INERTIA_FT4;
    section.as2 = 0.0;
    section.as3 = 0.0;
    model.frame_sections.insert("BW".into(), section);
    for j in 0..=STOREYS {
        for i in 0..=BAYS {
            let name = format!("{i}_{j}");
            model.add_point(
                name.clone(),
                BAY_WIDTH * i as f64,
                0.0,
                STOREY_HEIGHT * j as f64,
            );
            if j == 0 {
                model.points[&name].restraint = [true; 6];
            }
        }
    }
    for j in 1..=STOREYS {
        for i in 0..BAYS {
            model.add_frame(
                format!("B{i}_{j}"),
                format!("{i}_{j}"),
                format!("{}_{j}", i + 1),
                "BW",
            );
        }
    }
    for j in 0..STOREYS {
        for i in 0..=BAYS {
            model.add_frame(
                format!("C{i}_{j}"),
                format!("{i}_{j}"),
                format!("{i}_{}", j + 1),
                "BW",
            );
        }
    }
    model
        .load_cases
        .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 3));
    model
}

/// Independent plane-frame assembly for the same structure: Euler-Bernoulli
/// members (axial and bending only), lumped translational mass (half of each
/// member's mass to each end), restrained base. Returns the dense matrix and
/// the diagonal mass, 3 DOFs (x, z, rotation) per joint.
fn bathe_wilson_matrices() -> (Vec<f64>, Vec<f64>, usize) {
    let nodes = (BAYS + 1) * (STOREYS + 1);
    let n = 3 * nodes;
    let id = |i: usize, j: usize| j * (BAYS + 1) + i;
    let mut k = vec![0.0; n * n];
    let mut m = vec![0.0; n];
    let mut member = |a: usize, b: usize, horizontal: bool, len: f64| {
        let ea = E_KSF * AREA_FT2 / len;
        let ei = E_KSF * INERTIA_FT4;
        // Local (axial, transverse, rotation) stiffness, then rotated to global x, z.
        let mut kl = [[0.0f64; 6]; 6];
        kl[0][0] = ea;
        kl[3][3] = ea;
        kl[0][3] = -ea;
        kl[3][0] = -ea;
        let c1 = 12.0 * ei / len.powi(3);
        let c2 = 6.0 * ei / len.powi(2);
        let c3 = 4.0 * ei / len;
        let c4 = 2.0 * ei / len;
        kl[1][1] = c1;
        kl[4][4] = c1;
        kl[1][4] = -c1;
        kl[4][1] = -c1;
        kl[1][2] = c2;
        kl[2][1] = c2;
        kl[1][5] = c2;
        kl[5][1] = c2;
        kl[4][2] = -c2;
        kl[2][4] = -c2;
        kl[4][5] = -c2;
        kl[5][4] = -c2;
        kl[2][2] = c3;
        kl[5][5] = c3;
        kl[2][5] = c4;
        kl[5][2] = c4;
        // Horizontal member: local axes = global (x, z). Vertical member
        // (pointing up, +z): axial = z, transverse = -x, rotation unchanged.
        let r: [[f64; 3]; 3] = if horizontal {
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        } else {
            // global (ux, uz, ry) -> local (axial = uz, transverse = -ux, rotation = ry)
            [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
        };
        let dofs = [3 * a, 3 * a + 1, 3 * a + 2, 3 * b, 3 * b + 1, 3 * b + 2];
        for p in 0..6 {
            for q in 0..6 {
                let mut sum = 0.0;
                for s in 0..6 {
                    for t in 0..6 {
                        // T has R in both diagonal blocks.
                        let tp = r[s % 3][p % 3] * f64::from(u8::from(s / 3 == p / 3));
                        let tq = r[t % 3][q % 3] * f64::from(u8::from(t / 3 == q / 3));
                        sum += tp * kl[s][t] * tq;
                    }
                }
                k[dofs[p] * n + dofs[q]] += sum;
            }
        }
        for node in [a, b] {
            m[3 * node] += 0.5 * MASS_PER_LENGTH * len;
            m[3 * node + 1] += 0.5 * MASS_PER_LENGTH * len;
        }
    };
    for j in 1..=STOREYS {
        for i in 0..BAYS {
            member(id(i, j), id(i + 1, j), true, BAY_WIDTH);
        }
    }
    for j in 0..STOREYS {
        for i in 0..=BAYS {
            member(id(i, j), id(i, j + 1), false, STOREY_HEIGHT);
        }
    }
    // Fixed base: row and column replaced by the identity, no mass.
    for i in 0..=BAYS {
        for d in 0..3 {
            let g = 3 * id(i, 0) + d;
            for x in 0..n {
                k[g * n + x] = 0.0;
                k[x * n + g] = 0.0;
            }
            k[g * n + g] = 1.0;
            m[g] = 0.0;
        }
    }
    (k, m, n)
}

#[test]
fn csi_example_1_021_eigenvalues_by_analysis_condensation_and_subspace() {
    // Inputs read from problem_1_021.pdf pp. 1-2: ten 20 ft bays, nine 10 ft
    // storeys, fixed base, E = 432000 k/ft^2, A = 3 ft^2, I = 1 ft^4,
    // mass 3 k s^2/ft per unit length, shear areas zero. Tolerance 1e-5 relative.
    let mut model = bathe_wilson_model();
    analyze_model(&mut model).expect("analysis");
    let modes = &model.case_results["MODAL"].modes;
    assert_eq!(modes.len(), 3);
    for (mode, expected) in modes.iter().zip(EXAMPLE_1_021) {
        let rel = (mode.eigenvalue - expected).abs() / expected;
        assert!(
            rel < 1e-5,
            "analyze_model: {} vs {expected}",
            mode.eigenvalue
        );
    }

    let (k, m, n) = bathe_wilson_matrices();
    let ldlt = SymmetricLdlt::factorize(&k, n).unwrap();
    for (label, result) in [
        ("condensation", solve_modal_eigen(&ldlt, &m, 3).unwrap()),
        (
            "subspace",
            solve_modal_eigen_subspace(&k, &ldlt, &m, 3).unwrap(),
        ),
    ] {
        assert_eq!(result.len(), 3, "{label}");
        for ((lambda, phi), expected) in result.iter().zip(EXAMPLE_1_021) {
            let rel = (lambda - expected).abs() / expected;
            assert!(rel < 1e-5, "{label}: {lambda} vs {expected}");
            let modal_mass: f64 = (0..n).map(|i| m[i] * phi[i] * phi[i]).sum();
            assert!(
                (modal_mass - 1.0).abs() < 1e-9,
                "{label}: phi^T M phi = {modal_mass}"
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Fixed-free chain.
// ---------------------------------------------------------------------------

fn chain_matrices(n: usize, spring: f64, mass: f64) -> (Vec<f64>, Vec<f64>) {
    let mut k = vec![0.0; n * n];
    for i in 0..n {
        k[i * n + i] = if i + 1 < n { 2.0 * spring } else { spring };
        if i + 1 < n {
            k[i * n + i + 1] = -spring;
            k[(i + 1) * n + i] = -spring;
        }
    }
    (k, vec![mass; n])
}

#[test]
fn subspace_solver_matches_the_fixed_free_chain_closed_form() {
    // n masses m [kg] joined by springs k [N/m], first spring to ground:
    // lambda_j = (4k/m) sin^2((2j-1) pi / (2(2n+1))) [1/s^2] (j = 1, 2, ...).
    let (n, spring, mass) = (2000usize, 1.0e3, 2.5);
    let (k, m) = chain_matrices(n, spring, mass);
    let ldlt = SymmetricLdlt::factorize(&k, n).unwrap();
    let wanted = 20;
    let modes = solve_modal_eigen_subspace(&k, &ldlt, &m, wanted).unwrap();
    assert_eq!(modes.len(), wanted);
    let routed = solve_modal_eigen_checked(&k, &ldlt, &m, wanted).unwrap();
    for (a, b) in modes.iter().zip(&routed) {
        assert_eq!(
            a.0.to_bits(),
            b.0.to_bits(),
            "checked entry point uses the subspace path"
        );
    }
    let pi = std::f64::consts::PI;
    for (idx, (lambda, phi)) in modes.iter().enumerate() {
        let j = (idx + 1) as f64;
        let angle = (2.0 * j - 1.0) * pi / (2.0 * (2.0 * n as f64 + 1.0));
        let expected = 4.0 * spring / mass * angle.sin().powi(2);
        let rel = (lambda - expected).abs() / expected;
        assert!(
            rel < 1e-10,
            "mode {}: {lambda} vs {expected} (rel {rel:e})",
            idx + 1
        );
        // Mode shape sin((2j-1) pi i / (2n+1)), mass-normalized, largest component positive.
        let shape: Vec<f64> = (1..=n)
            .map(|i| ((2.0 * j - 1.0) * pi * i as f64 / (2.0 * n as f64 + 1.0)).sin())
            .collect();
        let norm = shape.iter().map(|v| v * v * mass).sum::<f64>().sqrt();
        let largest = shape
            .iter()
            .copied()
            .fold(0.0f64, |a, v| if v.abs() > a.abs() { v } else { a });
        let sign = largest.signum();
        for i in 0..n {
            let expected_phi = sign * shape[i] / norm;
            assert!(
                (phi[i] - expected_phi).abs() < 1e-6 / norm,
                "mode {} component {i}: {} vs {expected_phi}",
                idx + 1,
                phi[i]
            );
        }
    }
}

#[test]
fn subspace_solver_returns_the_right_modes_when_the_cut_splits_a_repeated_eigenvalue() {
    // Two identical decoupled fixed-free chains (250 unit masses, unit springs,
    // nm = 500): every eigenvalue is double,
    // lambda_j = 4 sin^2((2j-1) pi / (2(2*250+1))). With p = 5 the cut falls
    // inside the pair lambda_3: expected [l1, l1, l2, l2, l3].
    let chain = 250usize;
    let n = 2 * chain;
    let mut k = vec![0.0; n * n];
    for block in 0..2 {
        let o = block * chain;
        for i in 0..chain {
            k[(o + i) * n + o + i] = if i + 1 < chain { 2.0 } else { 1.0 };
            if i + 1 < chain {
                k[(o + i) * n + o + i + 1] = -1.0;
                k[(o + i + 1) * n + o + i] = -1.0;
            }
        }
    }
    let m = vec![1.0; n];
    let ldlt = SymmetricLdlt::factorize(&k, n).unwrap();
    let pi = std::f64::consts::PI;
    let lambda = |j: f64| {
        4.0 * ((2.0 * j - 1.0) * pi / (2.0 * (2.0 * chain as f64 + 1.0)))
            .sin()
            .powi(2)
    };
    for (p, expected) in [
        (
            5usize,
            vec![
                lambda(1.0),
                lambda(1.0),
                lambda(2.0),
                lambda(2.0),
                lambda(3.0),
            ],
        ),
        (4, vec![lambda(1.0), lambda(1.0), lambda(2.0), lambda(2.0)]),
        (
            7,
            vec![
                lambda(1.0),
                lambda(1.0),
                lambda(2.0),
                lambda(2.0),
                lambda(3.0),
                lambda(3.0),
                lambda(4.0),
            ],
        ),
    ] {
        let modes = solve_modal_eigen_checked(&k, &ldlt, &m, p).unwrap();
        assert_eq!(modes.len(), p);
        for (i, ((got, _), want)) in modes.iter().zip(&expected).enumerate() {
            assert!(
                (got - want).abs() <= 1e-10 * want,
                "p = {p}, mode {}: {got} vs {want}",
                i + 1
            );
        }
    }
}

// ---------------------------------------------------------------------------
// 3D frame cross-check through the public analysis.
// ---------------------------------------------------------------------------

/// 5 x 5 plan, 6 storeys, all six DOFs, fixed base, joint masses in the three
/// translations only (150 free joints = 450 mass DOFs > 400).
fn frame_3d(max_modes: usize) -> (SapModel, Vec<(String, f64)>) {
    let (nx, ny, nz) = (5usize, 5usize, 6usize);
    let mut model = SapModel::default();
    model.load_cases.clear();
    let mut material = Material::default_steel_kip_in();
    material.name = "S".into();
    material.unit_mass = 0.0;
    material.unit_weight = 0.0;
    model.materials.insert("S".into(), material);
    let mut section = sap_section::make_rectangular("COL", "S", 12.0, 8.0);
    section.as2 = 0.0;
    section.as3 = 0.0;
    model.frame_sections.insert("COL".into(), section);
    let mut rng = Lcg(0x3d_f4a3_0001);
    let mut masses = Vec::new();
    for k in 0..=nz {
        for j in 0..ny {
            for i in 0..nx {
                let name = format!("J{i}_{j}_{k}");
                // Irregular spacing separates the modes.
                let x = 120.0 * i as f64 + 5.0 * (i * i) as f64;
                let y = 100.0 * j as f64 + 7.0 * (j * j) as f64;
                let z = 144.0 * k as f64;
                model.add_point(name.clone(), x, y, z);
                if k == 0 {
                    model.points[&name].restraint = [true; 6];
                } else {
                    let mass = 0.5 + rng.uniform();
                    model.points[&name].mass = [mass, mass, mass, 0.0, 0.0, 0.0];
                    masses.push((name, mass));
                }
            }
        }
    }
    for k in 0..nz {
        for j in 0..ny {
            for i in 0..nx {
                model.add_frame(
                    format!("C{i}_{j}_{k}"),
                    format!("J{i}_{j}_{k}"),
                    format!("J{i}_{j}_{}", k + 1),
                    "COL",
                );
            }
        }
    }
    for k in 1..=nz {
        for j in 0..ny {
            for i in 0..nx {
                if i + 1 < nx {
                    model.add_frame(
                        format!("BX{i}_{j}_{k}"),
                        format!("J{i}_{j}_{k}"),
                        format!("J{}_{j}_{k}", i + 1),
                        "COL",
                    );
                }
                if j + 1 < ny {
                    model.add_frame(
                        format!("BY{i}_{j}_{k}"),
                        format!("J{i}_{j}_{k}"),
                        format!("J{i}_{}_{k}", j + 1),
                        "COL",
                    );
                }
            }
        }
    }
    model
        .load_cases
        .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", max_modes));
    (model, masses)
}

#[test]
fn subspace_and_condensation_agree_on_a_3d_frame() {
    let wanted = 8;
    let (mut subspace, masses) = frame_3d(wanted);
    let nm = 3 * masses.len();
    assert!(nm > 400 && 4 * wanted < nm, "subspace path is selected");
    analyze_model(&mut subspace).expect("subspace analysis");
    // 4p >= nm selects the unchanged condensation; its lowest modes are the reference.
    let (mut reference, _) = frame_3d(nm / 4 + 1);
    analyze_model(&mut reference).expect("condensation analysis");

    let sub = &subspace.case_results["MODAL"].modes;
    let refm = &reference.case_results["MODAL"].modes;
    assert_eq!(sub.len(), wanted);
    assert!(refm.len() >= wanted);
    let mass_of: std::collections::HashMap<&str, f64> =
        masses.iter().map(|(n, m)| (n.as_str(), *m)).collect();
    let m_dot = |a: &sap_model::ModalModeResult, b: &sap_model::ModalModeResult| -> f64 {
        mass_of
            .iter()
            .map(|(name, m)| {
                let (ua, ub) = (a.mode_shape[*name], b.mode_shape[*name]);
                m * (ua[0] * ub[0] + ua[1] * ub[1] + ua[2] * ub[2])
            })
            .sum()
    };
    let mut separated = 0;
    for i in 0..wanted {
        let (ls, lr) = (sub[i].eigenvalue, refm[i].eigenvalue);
        assert!((ls - lr).abs() <= 1e-9 * lr, "mode {}: {ls} vs {lr}", i + 1);
        let gap_below = if i > 0 {
            (lr - refm[i - 1].eigenvalue) / lr
        } else {
            1.0
        };
        let gap_above = (refm[i + 1].eigenvalue - lr) / lr;
        if gap_below > 1e-3 && gap_above > 1e-3 {
            separated += 1;
            let overlap = m_dot(&sub[i], &refm[i]).abs();
            assert!(
                overlap >= 1.0 - 1e-8,
                "mode {}: |phi^T M phi_ref| = {overlap}",
                i + 1
            );
        }
        let self_mass = m_dot(&sub[i], &sub[i]);
        assert!(
            (self_mass - 1.0).abs() < 1e-8,
            "mode {} phi^T M phi = {self_mass}",
            i + 1
        );
    }
    assert!(
        separated >= 3,
        "too few well-separated modes ({separated}) to be a useful check"
    );
}

// ---------------------------------------------------------------------------
// A mode orthogonal to every starting vector.
// ---------------------------------------------------------------------------

#[test]
fn subspace_finds_or_reports_a_mode_orthogonal_to_all_starting_vectors() {
    // Block B: fixed-fixed chain of 450 unit masses and unit springs,
    // lambda_j = 4 sin^2(j pi / (2 (450 + 1))). Its even modes are
    // antisymmetric, hence M-orthogonal to M*1. Block A: 12 uncoupled
    // oscillators with m = 1000 and k = 1 + 0.01 i, lambda ~ 1e-3, whose
    // m_ii / k_ii (about 1e3) is far above the chain's 0.5, so every unit
    // starting vector of the first attempt (q = 11 needs 10) falls into A.
    // Mode B2 (lambda ~ 1.9e-4 < every A eigenvalue) is therefore not in the
    // starting subspace, yet it is one of the three lowest modes.
    let chain = 450usize;
    let a_count = 12usize;
    let n = chain + a_count;
    let mut k = vec![0.0; n * n];
    let mut m = vec![1.0; n];
    for i in 0..chain {
        k[i * n + i] = 2.0;
        if i + 1 < chain {
            k[i * n + i + 1] = -1.0;
            k[(i + 1) * n + i] = -1.0;
        }
    }
    for a in 0..a_count {
        let dof = chain + a;
        k[dof * n + dof] = 1.0 + 0.01 * a as f64;
        m[dof] = 1000.0;
    }
    let ldlt = SymmetricLdlt::factorize(&k, n).unwrap();
    let pi = std::f64::consts::PI;
    let lambda_chain = |j: f64| 4.0 * (j * pi / (2.0 * (chain as f64 + 1.0))).sin().powi(2);
    let expected = [lambda_chain(1.0), lambda_chain(2.0), lambda_chain(3.0)];
    // All of A lies above the third chain mode, so the exact lowest three are B1..B3.
    assert!(expected[2] < 1.0e-3);

    match solve_modal_eigen_subspace(&k, &ldlt, &m, 3) {
        Ok(modes) => {
            assert_eq!(modes.len(), 3);
            for ((lambda, _), want) in modes.iter().zip(expected) {
                assert!(
                    (lambda - want).abs() <= 1e-9 * want,
                    "{lambda} vs {want}: a mode was silently missed"
                );
            }
        }
        Err(error) => {
            // Reporting the problem is the other acceptable outcome.
            assert!(
                matches!(
                    error,
                    LinalgError::EigenSturmMismatch { .. } | LinalgError::EigenConvergence(_)
                ),
                "unexpected error {error}"
            );
        }
    }
    // The condensation reference agrees with the closed form.
    let reference = solve_modal_eigen(&ldlt, &m, 3).unwrap();
    for ((lambda, _), want) in reference.iter().zip(expected) {
        assert!((lambda - want).abs() <= 1e-9 * want);
    }
}

// ---------------------------------------------------------------------------
// Timing (ignored): old and new paths on a lattice of about 500 joints.
// ---------------------------------------------------------------------------

/// 8 x 8 x 8 grid of joints, 3 translational DOFs each. Springs (k = 1e4 [N/m])
/// along every grid edge and every face and body diagonal give a stable
/// space truss; the bottom layer is fixed; masses are 1 + 0.5 rand [kg].
fn lattice(side: usize) -> (Vec<f64>, Vec<f64>, usize) {
    let id = |i: usize, j: usize, k: usize| (k * side + j) * side + i;
    let joints = side * side * side;
    let n = 3 * joints;
    let mut stiffness = vec![0.0; n * n];
    let mut rng = Lcg(0x1a77_1ce5);
    let mut add_spring = |a: usize, b: usize, dir: [f64; 3], kk: f64| {
        let len = (dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]).sqrt();
        let c = [dir[0] / len, dir[1] / len, dir[2] / len];
        for r in 0..3 {
            for s in 0..3 {
                let v = kk * c[r] * c[s];
                stiffness[(3 * a + r) * n + 3 * a + s] += v;
                stiffness[(3 * b + r) * n + 3 * b + s] += v;
                stiffness[(3 * a + r) * n + 3 * b + s] -= v;
                stiffness[(3 * b + r) * n + 3 * a + s] -= v;
            }
        }
    };
    for k in 0..side {
        for j in 0..side {
            for i in 0..side {
                for (di, dj, dk) in [
                    (1i32, 0i32, 0i32),
                    (0, 1, 0),
                    (0, 0, 1),
                    (1, 1, 0),
                    (1, -1, 0),
                    (1, 0, 1),
                    (1, 0, -1),
                    (0, 1, 1),
                    (0, 1, -1),
                    (1, 1, 1),
                    (1, 1, -1),
                    (1, -1, 1),
                    (1, -1, -1),
                ] {
                    let (ti, tj, tk) = (i as i32 + di, j as i32 + dj, k as i32 + dk);
                    let side_i = side as i32;
                    if ti < 0 || tj < 0 || tk < 0 || ti >= side_i || tj >= side_i || tk >= side_i {
                        continue;
                    }
                    add_spring(
                        id(i, j, k),
                        id(ti as usize, tj as usize, tk as usize),
                        [f64::from(di), f64::from(dj), f64::from(dk)],
                        1.0e4,
                    );
                }
            }
        }
    }
    let mut mass = vec![0.0; n];
    for k in 0..side {
        for j in 0..side {
            for i in 0..side {
                let node = id(i, j, k);
                for d in 0..3 {
                    let g = 3 * node + d;
                    if k == 0 {
                        for x in 0..n {
                            stiffness[g * n + x] = 0.0;
                            stiffness[x * n + g] = 0.0;
                        }
                        stiffness[g * n + g] = 1.0;
                    } else {
                        mass[g] = 1.0 + 0.5 * rng.uniform();
                    }
                }
            }
        }
    }
    (stiffness, mass, n)
}

#[test]
fn subspace_solver_agrees_with_condensation_on_a_lattice() {
    // Smaller lattice (nm = 3 * 6 * 6 * 5 = 540 > 400) so it runs in debug builds.
    let (k, m, n) = lattice(6);
    let ldlt = SymmetricLdlt::factorize(&k, n).unwrap();
    let p = 10;
    let sub = solve_modal_eigen_subspace(&k, &ldlt, &m, p).unwrap();
    let full = solve_modal_eigen(&ldlt, &m, p).unwrap();
    assert_eq!(sub.len(), p);
    for (i, ((ls, _), (lf, _))) in sub.iter().zip(&full).enumerate() {
        assert!((ls - lf).abs() <= 1e-9 * lf, "mode {}: {ls} vs {lf}", i + 1);
    }
}

#[test]
#[ignore = "timing report: cargo test --release -- --ignored timing_old_and_new_paths --nocapture"]
fn timing_old_and_new_paths() {
    use std::time::Instant;
    let (k, m, n) = lattice(8);
    println!("lattice: {} joints, {n} DOFs", n / 3);

    let t = Instant::now();
    let old = oracle::SymmetricLdlt::factorize(&k, n).unwrap();
    let t_old_factor = t.elapsed();
    let t = Instant::now();
    let ldlt = SymmetricLdlt::factorize(&k, n).unwrap();
    let t_new_factor = t.elapsed();
    assert_eq!(bits(&old.l), bits(&ldlt.l));
    println!("factorization: old dense {t_old_factor:?}, new profile {t_new_factor:?}");

    let rhs: Vec<f64> = (0..n).map(|i| ((i % 17) as f64) - 8.0).collect();
    let t = Instant::now();
    for _ in 0..20 {
        std::hint::black_box(old.solve(&rhs).unwrap());
    }
    let t_old_solve = t.elapsed() / 20;
    let t = Instant::now();
    for _ in 0..20 {
        std::hint::black_box(ldlt.solve(&rhs).unwrap());
    }
    let t_new_solve = t.elapsed() / 20;
    println!("one solve: old {t_old_solve:?}, new {t_new_solve:?}");

    let p = 12;
    let t = Instant::now();
    let condensed = solve_modal_eigen(&ldlt, &m, p).unwrap();
    let t_condensation = t.elapsed();
    let t = Instant::now();
    let subspace = solve_modal_eigen_subspace(&k, &ldlt, &m, p).unwrap();
    let t_subspace = t.elapsed();
    println!(
        "modal solve for {p} modes (nm = {}): condensation {t_condensation:?}, subspace {t_subspace:?}",
        m.iter().filter(|&&v| v > 0.0).count()
    );
    for ((a, _), (b, _)) in condensed.iter().zip(&subspace) {
        assert!((a - b).abs() <= 1e-9 * b);
    }
}

// ---------------------------------------------------------------------------
// Verbatim copy of the dense algorithm of revision 2238221 (the oracle).
// ---------------------------------------------------------------------------

#[allow(dead_code, clippy::all)]
mod oracle {
    use thiserror::Error;

    #[derive(Debug, Error, Clone)]
    pub enum LinalgError {
        #[error("Matrix dimension mismatch: expected {expected}, got {actual}")]
        DimensionMismatch { expected: usize, actual: usize },
        #[error("Singular matrix encountered at pivot index {0}")]
        SingularMatrix(usize),
        #[error("Eigenvalue solver failed to converge within {0} sweeps")]
        EigenConvergence(usize),
        #[error("Nonfinite value encountered in {0}")]
        NonFinite(&'static str),
    }

    /// Active-column / dense $LDL^T$ factorization of a symmetric matrix $\mathbf{K} = \mathbf{L}\mathbf{D}\mathbf{L}^T$.
    ///
    /// Tracks the Sturm sequence negative pivot count (`sturm_count`) for eigenvalue bounds and
    /// stability diagnostics (`Csi_sFire_nSturm`), and automatically stabilizes zero-stiffness DOFs.
    #[derive(Debug, Clone)]
    pub struct SymmetricLdlt {
        pub n: usize,
        /// Unit lower-triangular factor $\mathbf{L}$ stored row-by-row (flattened $n \times n$).
        pub l: Vec<f64>,
        /// Diagonal pivot vector $\mathbf{D}$ ($n$).
        pub d: Vec<f64>,
        /// Boolean mask indicating whether DOF $i$ had identically zero stiffness and was stabilized.
        pub zero_dof: Vec<bool>,
        /// Number of strictly negative pivots $D_{ii} < 0$ (`Csi_sFire_nSturm`).
        pub sturm_count: usize,
    }

    impl SymmetricLdlt {
        /// Factorizes an $n \times n$ symmetric matrix `k` (stored as flattened row-major `n * n`).
        pub fn factorize(k: &[f64], n: usize) -> Result<Self, LinalgError> {
            Self::factorize_with_reference(k, n, None)
        }

        /// As [`Self::factorize`], with a reference diagonal for the singularity
        /// test. A pivot is normally compared with the original diagonal of its
        /// DOF. A penalty constraint inflates that diagonal by many orders of
        /// magnitude, and the DOF of a constrained pair that is eliminated second
        /// legitimately keeps only the structural stiffness, so the comparison
        /// would report a sound but flexible structure as singular. Where
        /// `reference[j]` is positive (the structural diagonal without penalties)
        /// the pivot is compared with the smaller of the two, and where it is zero
        /// with nothing; in both cases never with less than the roundoff of the
        /// arithmetic actually performed on the inflated diagonal, so a true
        /// mechanism at a constrained DOF is still detected.
        pub fn factorize_with_reference(
            k: &[f64],
            n: usize,
            reference: Option<&[f64]>,
        ) -> Result<Self, LinalgError> {
            if reference.is_some_and(|r| r.len() != n) {
                return Err(LinalgError::DimensionMismatch {
                    expected: n,
                    actual: reference.map_or(0, <[f64]>::len),
                });
            }
            if k.len() != n * n {
                return Err(LinalgError::DimensionMismatch {
                    expected: n * n,
                    actual: k.len(),
                });
            }
            if k.iter().any(|v| !v.is_finite()) {
                return Err(LinalgError::NonFinite("stiffness matrix"));
            }
            let mut l = vec![0.0; n * n];
            let mut d = vec![0.0; n];
            let mut zero_dof = vec![false; n];

            let mut orig_diag = vec![0.0f64; n];
            for i in 0..n {
                orig_diag[i] = k[i * n + i];
            }

            // Identify rows/cols that are completely zero prior to elimination (unused DOFs, e.g. 2D truss out-of-plane or released rotations)
            for i in 0..n {
                let mut row_norm = 0.0f64;
                for j in 0..n {
                    row_norm = row_norm.max(k[i * n + j].abs());
                }
                if row_norm == 0.0 {
                    zero_dof[i] = true;
                }
                l[i * n + i] = 1.0;
            }

            for j in 0..n {
                if zero_dof[j] {
                    d[j] = 1.0;
                    continue;
                }
                let mut dj = k[j * n + j];
                for p in 0..j {
                    let l_jp = l[j * n + p];
                    if l_jp != 0.0 {
                        dj -= l_jp * l_jp * d[p];
                    }
                }

                // If this DOF had positive initial stiffness,
                // losing stiffness during elimination (`dj <= 1e-11 * orig_diag[j]`) or going negative
                // indicates a kinematic mechanism or unstable structure!
                if !dj.is_finite() {
                    return Err(LinalgError::NonFinite("stiffness factorization"));
                }
                let scale = match reference.map(|r| r[j]) {
                    Some(structural) if structural > 0.0 => orig_diag[j].abs().min(structural),
                    // A DOF whose whole diagonal is penalty (for example the
                    // rotation of a solid joint tied into a body constraint) has no
                    // structural scale; only the roundoff floor applies to it.
                    Some(_) => 0.0,
                    None => orig_diag[j].abs(),
                };
                let roundoff_floor = 64.0 * f64::EPSILON * orig_diag[j].abs();
                let rel_pivot_tol = (scale * 1e-11).max(if reference.is_some() {
                    roundoff_floor
                } else {
                    0.0
                });
                if dj <= rel_pivot_tol {
                    return Err(LinalgError::SingularMatrix(j));
                }

                d[j] = dj;
                let inv_dj = 1.0 / dj;

                for i in (j + 1)..n {
                    if zero_dof[i] {
                        continue;
                    }
                    let mut sum = k[i * n + j];
                    for p in 0..j {
                        let l_ip = l[i * n + p];
                        let l_jp = l[j * n + p];
                        if l_ip != 0.0 && l_jp != 0.0 {
                            sum -= l_ip * l_jp * d[p];
                        }
                    }
                    l[i * n + j] = sum * inv_dj;
                }
            }

            Ok(Self {
                n,
                l,
                d,
                zero_dof,
                sturm_count: 0,
            })
        }

        /// Solves $\mathbf{K} \mathbf{x} = \mathbf{b}$ using forward, diagonal, and backward substitution.
        pub fn solve(&self, b: &[f64]) -> Result<Vec<f64>, LinalgError> {
            if b.len() != self.n {
                return Err(LinalgError::DimensionMismatch {
                    expected: self.n,
                    actual: b.len(),
                });
            }
            if b.iter().any(|v| !v.is_finite()) {
                return Err(LinalgError::NonFinite("load vector"));
            }
            let n = self.n;
            let mut y = vec![0.0; n];

            // Forward substitution: L * y = b
            for i in 0..n {
                if self.zero_dof[i] {
                    if b[i] != 0.0 {
                        // Applied load on an unrestrained zero-stiffness DOF -> mechanism!
                        return Err(LinalgError::SingularMatrix(i));
                    }
                    y[i] = 0.0;
                    continue;
                }
                let mut sum = b[i];
                let row_off = i * n;
                for j in 0..i {
                    let l_ij = self.l[row_off + j];
                    if l_ij != 0.0 {
                        sum -= l_ij * y[j];
                    }
                }
                y[i] = sum;
            }

            // Diagonal scaling: D * z = y
            for i in 0..n {
                if self.zero_dof[i] {
                    y[i] = 0.0;
                } else {
                    y[i] /= self.d[i];
                }
            }

            // Backward substitution: L^T * x = z
            let mut x = y;
            for i in (0..n).rev() {
                if self.zero_dof[i] {
                    x[i] = 0.0;
                    continue;
                }
                let mut sum = x[i];
                for j in (i + 1)..n {
                    let l_ji = self.l[j * n + i];
                    if l_ji != 0.0 {
                        sum -= l_ji * x[j];
                    }
                }
                x[i] = sum;
            }

            if x.iter().any(|v| !v.is_finite()) {
                return Err(LinalgError::NonFinite("solution vector"));
            }
            Ok(x)
        }
    }
}
