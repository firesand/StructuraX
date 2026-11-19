//! Load-dependent Ritz vectors and modal load participation.
//!
//! Source: CSI Analysis Reference Manual, "Ritz-Vector Analysis" (printed
//! pp. 372-375) and "Static and Dynamic Load Participation Ratios"
//! (pp. 378-381); Wilson (1985) for the algorithm.
//!
//! All vectors live in the global joint-DOF numbering of the solver
//! (`6 * joint + dof`, joint-local axes), as `K` and the diagonal `M` do.

use crate::linalg::{jacobi_symmetric_eigen, SymmetricLdlt};
use crate::SolverError;
use sap_model::ritz::{acceleration_direction, LoadParticipation};
use sap_model::{LoadCase, ModalSubType, SapModel};

/// Mass below this value does not make a degree of freedom a mass DOF.
const MASS_TOL: f64 = 1e-18;
/// A generated vector whose M-norm falls to this fraction of its norm before
/// orthogonalization is redundant.
const REDUNDANT: f64 = 1e-10;
/// The stiffness form `B^T K B` of the Ritz projection is used while its round-off
/// estimate stays below this fraction of its lowest eigenvalue.
const STIFFNESS_FORM_TOL: f64 = 1e-9;

/// One starting load vector of a Ritz case.
#[derive(Debug, Clone)]
pub(crate) struct StartingLoad {
    pub name: String,
    pub is_acceleration: bool,
    /// Spatial load vector with restrained and inactive DOFs zeroed.
    pub p: Vec<f64>,
    /// `0` is unlimited.
    pub max_cycles: usize,
    /// `0` is no target.
    pub target: f64,
}

/// Modes produced from a set of starting loads.
pub(crate) struct RitzModes {
    /// `(eigenvalue w^2, mode shape)`, ascending, M-orthonormal.
    pub modes: Vec<(f64, Vec<f64>)>,
    /// Ritz vectors generated per starting load, in the order of the loads.
    pub generated: Vec<usize>,
}

/// The unit acceleration load in global direction `dir` (0, 1, 2) with scale
/// factor `scale`: `-scale * M * r`, as the time-history code builds it.
pub(crate) fn acceleration_load(
    m: &[f64],
    joint_rot: &[[[f64; 3]; 3]],
    dir: usize,
    scale: f64,
) -> Vec<f64> {
    let mut f = vec![0.0; m.len()];
    for (idx, rot) in joint_rot.iter().enumerate() {
        for d in 0..3 {
            let g = 6 * idx + d;
            f[g] = -m[g] * rot[dir][d] * scale;
        }
    }
    f
}

/// Rigid-body acceleration shapes of the six global acceleration loads, in the
/// solver's joint-local DOF numbering (CSI Analysis Reference Manual, "Modal
/// Analysis Output", printed pp. 376-378, and "Acceleration Loads", p. 339).
///
/// `r[dir]` is the displacement of every DOF under a unit rigid-body motion:
/// `dir` 0..3 translates along global X, Y, Z (`R[dir][d]` on the translational
/// DOFs of a joint with local-to-global matrix `R`); `dir` 3..6 rotates about
/// the axis X, Y, Z through the centre of mass the reference defines: the
/// lever arms use the centroid of the mass acting in each translational
/// component (RX: y of UZ, z of UY; RY: z of UX, x of UZ; RZ: y of UX, x of UY).
/// The printed text of p. 339 gives "z centroid of the UY load" for RZ, which
/// cannot place a vertical axis; the x centroid of UY is used (project choice).
#[derive(Debug, Clone)]
pub(crate) struct RigidBodyShapes {
    r: [Vec<f64>; 6],
    /// `r^T M r` per direction: the unrestrained mass or rotational inertia.
    pub total: [f64; 6],
    /// True when every joint has exactly identity axes: translational factors
    /// then use the plain `i % 6` accumulation.
    identity_axes: bool,
}

impl RigidBodyShapes {
    pub(crate) fn new(m: &[f64], joint_rot: &[[[f64; 3]; 3]], coords: &[[f64; 3]]) -> Self {
        let n = m.len();
        let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let identity_axes = joint_rot.iter().all(|r| *r == identity);
        // Mass acting in global component c at joint i: sum_d m_d R[c][d]^2.
        let comp_mass = |i: usize, c: usize| -> f64 {
            (0..3)
                .map(|d| m[6 * i + d] * joint_rot[i][c][d] * joint_rot[i][c][d])
                .sum()
        };
        // Centroid (x, y, z) of the mass acting in component c.
        let mut centroid = [[0.0f64; 3]; 3];
        for (c, cen) in centroid.iter_mut().enumerate() {
            let mut w = 0.0;
            for (i, xyz) in coords.iter().enumerate() {
                let mi = comp_mass(i, c);
                w += mi;
                for k in 0..3 {
                    cen[k] += mi * xyz[k];
                }
            }
            if w > 0.0 {
                for v in cen.iter_mut() {
                    *v /= w;
                }
            }
        }
        let mut r: [Vec<f64>; 6] = std::array::from_fn(|_| vec![0.0; n]);
        for (i, xyz) in coords.iter().enumerate() {
            let rot = &joint_rot[i];
            for dir in 0..3 {
                for d in 0..3 {
                    r[dir][6 * i + d] = rot[dir][d];
                }
            }
            // Global translation of the point under a unit rotation about
            // each axis through its own centroids.
            let [x, y, z] = *xyz;
            let g: [[f64; 3]; 3] = [
                [0.0, -(z - centroid[1][2]), y - centroid[2][1]],
                [z - centroid[0][2], 0.0, -(x - centroid[2][0])],
                [-(y - centroid[0][1]), x - centroid[1][0], 0.0],
            ];
            for a in 0..3 {
                for d in 0..3 {
                    // local = R^T global
                    r[3 + a][6 * i + d] = (0..3).map(|k| rot[k][d] * g[a][k]).sum();
                    // unit rotation about the global axis, in local components
                    r[3 + a][6 * i + 3 + d] = rot[a][d];
                }
            }
        }
        let total = std::array::from_fn(|dir| {
            if dir < 3 && identity_axes {
                (0..n).filter(|i| i % 6 == dir).map(|i| m[i]).sum()
            } else {
                (0..n).map(|i| m[i] * r[dir][i] * r[dir][i]).sum()
            }
        });
        Self {
            r,
            total,
            identity_axes,
        }
    }

    /// The load vector `-M r` of unit acceleration `dir` (sign as the
    /// time-history code builds it).
    pub(crate) fn load(&self, m: &[f64], dir: usize) -> Vec<f64> {
        m.iter().zip(&self.r[dir]).map(|(mi, ri)| -mi * ri).collect()
    }

    /// Participation factors `phi^T M r` of one mode for the six directions.
    pub(crate) fn factors(&self, m: &[f64], phi: &[f64]) -> [f64; 6] {
        let mut pf = [0.0f64; 6];
        for (dir, out) in pf.iter_mut().enumerate() {
            if dir < 3 && self.identity_axes {
                for i in 0..m.len() {
                    if i % 6 == dir {
                        *out += m[i] * phi[i];
                    }
                }
            } else {
                for i in 0..m.len() {
                    *out += m[i] * self.r[dir][i] * phi[i];
                }
            }
        }
        pf
    }

    /// Participating mass ratios `f^2 / total` (zero where the total is zero).
    pub(crate) fn mass_ratios(&self, pf: &[f64; 6]) -> [f64; 6] {
        std::array::from_fn(|d| {
            if self.total[d] > MASS_TOL {
                pf[d] * pf[d] / self.total[d]
            } else {
                0.0
            }
        })
    }
}

fn has_mass(m: &[f64], is_fixed: &[bool], i: usize) -> bool {
    !is_fixed[i] && m[i] > MASS_TOL
}

/// `p^T M^-1 p` over the mass DOFs (`a.p` with `M a = p`).
fn a_dot_p(m: &[f64], is_fixed: &[bool], p: &[f64]) -> f64 {
    (0..p.len())
        .filter(|&i| has_mass(m, is_fixed, i))
        .map(|i| p[i] * p[i] / m[i])
        .sum()
}

/// Checks every Ritz control of `lc` and returns the starting loads, in the
/// order of `lc.loads`. `pattern_force(name, scale)` returns the joint
/// equivalent load of a pattern (`f_ext - f_fef`).
pub(crate) fn build_starting_loads(
    model: &SapModel,
    lc: &LoadCase,
    m: &[f64],
    is_fixed: &[bool],
    joint_rot: &[[[f64; 3]; 3]],
    pattern_force: &dyn Fn(&str, f64) -> Result<Vec<f64>, SolverError>,
) -> Result<Vec<StartingLoad>, SolverError> {
    let case = &lc.name;
    if lc.min_modes > 1 {
        return Err(SolverError::UnsupportedInput(format!(
            "Ritz case '{case}' asks for a minimum of {} modes; a minimum above 1 is not implemented for load-dependent Ritz vectors",
            lc.min_modes
        )));
    }
    for (i, load) in lc.loads.iter().enumerate() {
        if lc.loads[..i]
            .iter()
            .any(|other| other.load_name == load.load_name)
        {
            return Err(SolverError::UnsupportedInput(format!(
                "Ritz case '{case}' lists starting load '{}' more than once",
                load.load_name
            )));
        }
    }
    for (i, control) in lc.ritz_controls.iter().enumerate() {
        if !lc.loads.iter().any(|l| l.load_name == control.load_name) {
            return Err(SolverError::UnsupportedInput(format!(
                "Ritz case '{case}' has a generation control for '{}', which is not one of its starting loads",
                control.load_name
            )));
        }
        let t = control.target_dynamic_participation;
        if !t.is_finite() || !(0.0..=1.0).contains(&t) {
            return Err(SolverError::UnsupportedInput(format!(
                "Ritz case '{case}': dynamic participation target {t} for '{}' must be in [0, 1]",
                control.load_name
            )));
        }
        if lc.ritz_controls[..i]
            .iter()
            .any(|other| other.load_name == control.load_name)
        {
            return Err(SolverError::UnsupportedInput(format!(
                "Ritz case '{case}' has two generation controls for '{}'",
                control.load_name
            )));
        }
    }

    let mut loads = Vec::with_capacity(lc.loads.len());
    for assignment in &lc.loads {
        let name = &assignment.load_name;
        let accel = acceleration_direction(name);
        let is_pattern = model.load_patterns.contains_key(name);
        if accel.is_some() && is_pattern {
            return Err(SolverError::UnsupportedInput(format!(
                "Ritz case '{case}': '{name}' is both a load pattern and an acceleration load name; rename the pattern"
            )));
        }
        let mut p = match accel {
            Some(dir) => acceleration_load(m, joint_rot, dir, assignment.scale_factor),
            None if is_pattern => pattern_force(name, assignment.scale_factor)?,
            None => {
                let rotational = matches!(
                    name.trim().to_ascii_uppercase().as_str(),
                    "U4" | "U5" | "U6" | "R1" | "R2" | "R3" | "RX" | "RY" | "RZ"
                );
                return Err(SolverError::UnsupportedInput(if rotational {
                    format!("Ritz case '{case}': rotational acceleration load '{name}' is not supported as a starting load; use UX, UY, UZ or a load pattern")
                } else {
                    format!("Ritz case '{case}': starting load '{name}' is neither a load pattern nor an acceleration load (UX, UY, UZ)")
                }));
            }
        };
        if p.iter().any(|v| !v.is_finite()) {
            return Err(SolverError::InvalidInput(format!(
                "Ritz case '{case}': starting load '{name}' is not finite"
            )));
        }
        // Reference magnitude of the load: the largest mass times the scale
        // for an acceleration, the largest force of a pattern before restrained
        // DOFs are zeroed. A load that reaches the mass DOFs only at round-off
        // level (for example a global acceleration along a joint axis turned
        // 90 degrees) has no usable component there.
        let pmax = match accel {
            Some(_) => assignment.scale_factor.abs() * m.iter().fold(0.0f64, |a, v| a.max(v.abs())),
            None => p.iter().fold(0.0f64, |a, v| a.max(v.abs())),
        };
        for (i, v) in p.iter_mut().enumerate() {
            if is_fixed[i] {
                *v = 0.0;
            }
        }
        let pmass = (0..p.len())
            .filter(|&i| has_mass(m, is_fixed, i))
            .fold(0.0f64, |a, i| a.max(p[i].abs()));
        if pmass <= 0.0 || pmass < 1e-12 * pmax {
            return Err(SolverError::UnsupportedInput(format!(
                "Ritz case '{case}': starting load '{name}' has no component on a mass degree of freedom"
            )));
        }
        let control = lc.ritz_controls.iter().find(|c| &c.load_name == name);
        loads.push(StartingLoad {
            name: name.clone(),
            is_acceleration: accel.is_some(),
            p,
            max_cycles: control.map_or(0, |c| c.max_cycles),
            target: control.map_or(0.0, |c| c.target_dynamic_participation),
        });
    }
    Ok(loads)
}

fn m_norm(m: &[f64], v: &[f64]) -> f64 {
    v.iter()
        .zip(m)
        .map(|(x, mi)| mi * x * x)
        .sum::<f64>()
        .sqrt()
}

fn matvec(k: &[f64], v: &[f64]) -> Vec<f64> {
    let n = v.len();
    (0..n)
        .map(|i| {
            k[i * n..(i + 1) * n]
                .iter()
                .zip(v)
                .map(|(a, b)| a * b)
                .sum()
        })
        .collect()
}

struct LoadState {
    active: bool,
    created: usize,
    last: Vec<f64>,
    /// Index in the basis of `last`.
    last_idx: usize,
    dyn_sum: f64,
    a_p: f64,
}

/// Generates Ritz modes from `loads` (Wilson 1985; reference p. 372-375).
///
/// Each cycle takes the active loads in order; cycle 1 solves `K u = p`, later
/// cycles `K u = M b` with `b` the load's previous (M-orthonormal) vector. The
/// candidate is M-orthogonalized twice against all earlier vectors; a
/// candidate whose M-norm falls to `1e-10` of its norm before is discarded
/// and its load is dropped. A load stops at its cycle limit or when its
/// cumulative dynamic participation `sum (b.p)^2 / (a.p)`, summed over all
/// vectors generated so far whichever load made them, reaches its target.
/// Generation ends at `max_modes` vectors (the last cycle makes only as many
/// as are needed), at the number of mass degrees of freedom (the rank of M: more
/// M-orthonormal vectors do not exist) or when no load is active.
///
/// The Ritz values come from `B^T K B`, or from `B^T M K^-1 M B` where the first loses
/// its digits: see the comment at the projection.
pub(crate) fn generate_ritz_modes(
    k: &[f64],
    ldlt: &SymmetricLdlt,
    m: &[f64],
    is_fixed: &[bool],
    loads: &[StartingLoad],
    max_modes: usize,
) -> Result<RitzModes, SolverError> {
    let n = ldlt.n;
    // A set of M-orthonormal vectors has at most as many members as M has rank, and
    // the manual lets the program "compute fewer modes if there are fewer mass degrees
    // of freedom" (Analysis Reference, Ritz-Vector Analysis, Number of Modes, p. 373).
    let rank = (0..n).filter(|&i| has_mass(m, is_fixed, i)).count();
    let max_modes = max_modes.min(rank);
    let mut states: Vec<LoadState> = loads
        .iter()
        .map(|l| LoadState {
            active: true,
            created: 0,
            last: Vec::new(),
            last_idx: 0,
            dyn_sum: 0.0,
            a_p: a_dot_p(m, is_fixed, &l.p),
        })
        .collect();
    let mut basis: Vec<Vec<f64>> = Vec::new();
    // `ab[i] = K^-1 M basis[i]`, the next vector of the sequence before orthogonalization.
    let mut ab: Vec<Option<Vec<f64>>> = Vec::new();

    'cycles: loop {
        if !states.iter().any(|s| s.active) {
            break;
        }
        for (li, load) in loads.iter().enumerate() {
            if !states[li].active {
                continue;
            }
            if basis.len() >= max_modes {
                break 'cycles;
            }
            let rhs: Vec<f64> = if states[li].created == 0 {
                load.p.clone()
            } else {
                states[li]
                    .last
                    .iter()
                    .zip(m)
                    .map(|(b, mi)| b * mi)
                    .collect()
            };
            let mut cand = ldlt.solve(&rhs)?;
            if states[li].created > 0 {
                ab[states[li].last_idx] = Some(cand.clone());
            }
            let before = m_norm(m, &cand);
            for _pass in 0..2 {
                for b in &basis {
                    let proj: f64 = cand
                        .iter()
                        .zip(b)
                        .zip(m)
                        .map(|((c, bi), mi)| c * bi * mi)
                        .sum();
                    for i in 0..n {
                        cand[i] -= proj * b[i];
                    }
                }
            }
            let after = m_norm(m, &cand);
            if after.is_nan() || after <= REDUNDANT * before || !after.is_finite() {
                states[li].active = false;
                continue;
            }
            for c in &mut cand {
                *c /= after;
            }
            // The cumulative dynamic participation of EVERY load grows with
            // each new vector (R^D sums over all modes, reference p. 380-381),
            // not only with the vectors that load generated itself.
            for (lj, other) in loads.iter().enumerate() {
                let mut p_dot = 0.0;
                for i in 0..n {
                    if has_mass(m, is_fixed, i) {
                        p_dot += cand[i] * other.p[i];
                    }
                }
                states[lj].dyn_sum += p_dot * p_dot / states[lj].a_p;
            }
            states[li].created += 1;
            states[li].last = cand.clone();
            states[li].last_idx = basis.len();
            if load.max_cycles != 0 && states[li].created >= load.max_cycles {
                states[li].active = false;
            }
            // A load whose target is met stops generating. A load that has
            // not yet made its first vector still makes it (the static
            // solution, so that its static participation reaches 100%,
            // reference p. 380) even when other loads' vectors already
            // reached its target.
            for (lj, other) in loads.iter().enumerate() {
                if states[lj].created > 0
                    && other.target > 0.0
                    && states[lj].dyn_sum >= other.target
                {
                    states[lj].active = false;
                }
            }
            ab.push(None);
            basis.push(cand);
        }
    }

    let nb = basis.len();
    if nb == 0 {
        return Err(SolverError::UnsupportedInput(
            "no Ritz vector could be generated from the starting loads".into(),
        ));
    }
    // Rayleigh-Ritz on span(basis). The basis is M-orthonormal (B^T M B = I), so the
    // projected problem is the standard one, and by the min-max theorem no Ritz value lies
    // below the exact one of the same index.
    //
    // Stiffness form: K_r z = w^2 z with K_r = B^T K B, a Galerkin projection with the true K.
    // It is a sum of terms of size |K| |b|^2, so its error is about eps * max |b| |K b| per
    // entry; where that exceeds a small fraction of the lowest Ritz value (stiff models:
    // penalty constraints, axial stiffness modifiers, whose massless components are large)
    // K_r has lost its digits and may even be indefinite.
    //
    // Carrying K b along while the vectors are made (K x = rhs from the solve, the same
    // Gram-Schmidt coefficients applied to K b) was tried as a single form (wave 5, P15) and
    // rejected: the rounding of `cand -= proj * b` is not mirrored in the carried K b, and
    // it enters K_r multiplied by |K| / (M-norm after orthogonalization). On the stiff
    // models of `wave5_ritz_bound` that gave K_r asymmetric by up to 1e-7 of its norm and
    // violations of the Rayleigh-Ritz bound of 1e-3 up to indefinite matrices (soft model
    // 1.7e-7 against a tolerance of 2e-11), worse than either form below.
    //
    // Flexibility form (used then): F_r = B^T M K^-1 M B, F_r z = mu z, w^2 = 1 / mu, the
    // Rayleigh-Ritz of the operator K^-1 M in the M inner product. It meets only products
    // of B with M K^-1 M B, whose error is the one of the solve with K, the same as the
    // Eigen solver's.
    let kb: Vec<Vec<f64>> = basis.iter().map(|b| matvec(k, b)).collect();
    let norm2 = |v: &[f64]| v.iter().map(|x| x * x).sum::<f64>().sqrt();
    let scale = (0..nb)
        .map(|a| norm2(&basis[a]) * norm2(&kb[a]))
        .fold(0.0f64, f64::max);
    let mut kr = vec![0.0; nb * nb];
    for a in 0..nb {
        for b in a..nb {
            let dot_ab: f64 = basis[a].iter().zip(&kb[b]).map(|(x, y)| x * y).sum();
            let dot_ba: f64 = basis[b].iter().zip(&kb[a]).map(|(x, y)| x * y).sum();
            let v = 0.5 * (dot_ab + dot_ba);
            kr[a * nb + b] = v;
            kr[b * nb + a] = v;
        }
    }
    let (kvals, kvecs) = jacobi_symmetric_eigen(&kr, nb, 80)?;
    let lowest = kvals.iter().cloned().fold(f64::INFINITY, f64::min);
    let error = f64::EPSILON * nb as f64 * scale;
    // (w^2, column of `vecs`), ascending
    let (pairs, vecs): (Vec<(f64, usize)>, Vec<f64>) =
        if lowest > 0.0 && error <= STIFFNESS_FORM_TOL * lowest {
            let mut order: Vec<usize> = (0..nb).collect();
            order.sort_by(|&a, &b| kvals[a].total_cmp(&kvals[b]));
            (order.into_iter().map(|j| (kvals[j], j)).collect(), kvecs)
        } else {
            for (i, slot) in ab.iter_mut().enumerate() {
                if slot.is_none() {
                    let mb: Vec<f64> = basis[i].iter().zip(m).map(|(b, mi)| b * mi).collect();
                    *slot = Some(ldlt.solve(&mb)?);
                }
            }
            let mdot = |u: &[f64], v: &[f64]| -> f64 {
                u.iter().zip(v).zip(m).map(|((a, b), mi)| a * b * mi).sum()
            };
            let mut fr = vec![0.0; nb * nb];
            for a in 0..nb {
                for b in a..nb {
                    let ab_b = ab[b].as_ref().expect("filled above");
                    let ab_a = ab[a].as_ref().expect("filled above");
                    let v = 0.5 * (mdot(&basis[a], ab_b) + mdot(&basis[b], ab_a));
                    fr[a * nb + b] = v;
                    fr[b * nb + a] = v;
                }
            }
            let (mus, fvecs) = jacobi_symmetric_eigen(&fr, nb, 80)?;
            let mu_max = mus.iter().cloned().fold(0.0f64, f64::max);
            if mu_max.is_nan() || mu_max <= 0.0 || !mu_max.is_finite() {
                return Err(SolverError::InvalidInput(format!(
                    "Ritz projection has no positive flexibility eigenvalue (largest {mu_max:e}); the stiffness matrix is not positive definite"
                )));
            }
            // F_r is positive definite for independent vectors; an eigenvalue at the
            // round-off level of the largest one stands for a frequency that cannot be
            // resolved.
            let floor = f64::EPSILON * nb as f64 * mu_max;
            let mut order: Vec<usize> = (0..nb).filter(|&j| mus[j] > floor).collect();
            order.sort_by(|&a, &b| mus[b].total_cmp(&mus[a]));
            (order.into_iter().map(|j| (1.0 / mus[j], j)).collect(), fvecs)
        };
    let mut modes = Vec::with_capacity(pairs.len());
    for &(w2, j) in &pairs {
        let mut phi = vec![0.0; n];
        for (b, basis_vec) in basis.iter().enumerate() {
            let z = vecs[b * nb + j];
            for i in 0..n {
                phi[i] += z * basis_vec[i];
            }
        }
        let norm = m_norm(m, &phi);
        let mut max_abs = 0.0;
        let mut sign = 1.0;
        for i in 0..n {
            if has_mass(m, is_fixed, i) && phi[i].abs() > max_abs {
                max_abs = phi[i].abs();
                sign = if phi[i] >= 0.0 { 1.0 } else { -1.0 };
            }
        }
        let scale = sign / norm;
        for v in &mut phi {
            *v *= scale;
        }
        modes.push((w2, phi));
    }
    Ok(RitzModes {
        modes,
        generated: states.iter().map(|s| s.created).collect(),
    })
}

/// Static and dynamic participation of each `(name, is_acceleration, p)` in
/// `modes` (reference pp. 379-381). `generated[i]` is the Ritz vector count of
/// load `i`, or empty for an eigenvector case.
pub(crate) fn load_participation(
    ldlt: &SymmetricLdlt,
    m: &[f64],
    is_fixed: &[bool],
    modes: &[(f64, Vec<f64>)],
    loads: &[(String, bool, Vec<f64>)],
    generated: &[usize],
) -> Result<Vec<LoadParticipation>, SolverError> {
    let mut out = Vec::with_capacity(loads.len());
    for (li, (name, is_acceleration, p)) in loads.iter().enumerate() {
        let a_p = a_dot_p(m, is_fixed, p);
        if a_p.is_nan() || a_p <= 0.0 {
            continue;
        }
        let u = ldlt.solve(p)?;
        let u_p: f64 = u.iter().zip(p).map(|(a, b)| a * b).sum();
        let mut part = LoadParticipation {
            load_name: name.clone(),
            is_acceleration: *is_acceleration,
            vectors_generated: generated.get(li).copied().unwrap_or(0),
            ..Default::default()
        };
        let (mut cs, mut cd) = (0.0, 0.0);
        for (w2, phi) in modes {
            let f_mass: f64 = (0..p.len())
                .filter(|&i| has_mass(m, is_fixed, i))
                .map(|i| phi[i] * p[i])
                .sum();
            let f_all: f64 = phi.iter().zip(p).map(|(a, b)| a * b).sum();
            let rd = f_mass * f_mass / a_p;
            let rs = if *w2 > 0.0 && u_p > 0.0 {
                f_all * f_all / w2 / u_p
            } else {
                0.0
            };
            cs += rs;
            cd += rd;
            part.static_ratio.push(rs);
            part.static_cumulative.push(cs);
            part.dynamic_ratio.push(rd);
            part.dynamic_cumulative.push(cd);
        }
        out.push(part);
    }
    Ok(out)
}

/// True when this case uses load-dependent starting vectors: a Ritz case that
/// lists starting loads or controls. An empty Ritz case keeps the legacy
/// unit-acceleration start.
pub(crate) fn uses_starting_loads(lc: &LoadCase) -> bool {
    lc.modal_sub_type == ModalSubType::Ritz
        && (!lc.loads.is_empty() || !lc.ritz_controls.is_empty())
}

#[cfg(test)]
mod tests {
    use super::*;

    const I3: [[f64; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

    fn weighted_dot(m: &[f64], a: &[f64], b: &[f64]) -> f64 {
        (0..m.len()).map(|i| m[i] * a[i] * b[i]).sum()
    }

    /// Three joints at arbitrary positions with isotropic translational
    /// masses 2, 5, 3 and some rotational mass.
    fn scattered() -> (Vec<f64>, Vec<[f64; 3]>) {
        let coords = vec![[1.0, 2.0, 0.5], [4.0, -1.0, 3.0], [-2.0, 3.5, 1.5]];
        let masses = [2.0, 5.0, 3.0];
        let mut m = Vec::new();
        for (i, mass) in masses.iter().enumerate() {
            m.extend([*mass, *mass, *mass, 1.0 + i as f64, 2.0, 0.5]);
        }
        (m, coords)
    }

    /// R4: with isotropic masses the rotation about the centre of mass is
    /// M-orthogonal to every translation (this is why the manual reports the
    /// rotational factors about the centre of mass, p. 339):
    /// r_RZ^T M r_UX = -sum m (y - y_c) = 0, r_RZ^T M r_UY = sum m (x - x_c)
    /// = 0, and likewise for RX with UY, UZ and RY with UX, UZ. The rotational
    /// inertia is the sum of m (distance to the axis through the centroid)^2
    /// plus the rotational DOF mass.
    #[test]
    fn rotations_about_the_centre_of_mass_are_orthogonal_to_translations() {
        let (m, coords) = scattered();
        let rot = vec![I3; 3];
        let shapes = RigidBodyShapes::new(&m, &rot, &coords);
        for (rotation, translations) in [(3, [1, 2]), (4, [0, 2]), (5, [0, 1])] {
            for t in translations {
                let dot = weighted_dot(&m, &shapes.r[rotation], &shapes.r[t]);
                assert!(dot.abs() < 1e-12, "r{rotation}.M.r{t} = {dot}");
            }
        }
        assert_eq!(shapes.total[0], 10.0);
        let xc = (2.0 * 1.0 + 5.0 * 4.0 - 3.0 * 2.0) / 10.0;
        let yc = (2.0 * 2.0 - 5.0 + 3.0 * 3.5) / 10.0;
        let zc = (2.0 * 0.5 + 5.0 * 3.0 + 3.0 * 1.5) / 10.0;
        // Rotational DOF masses: Jx = 1 + 2 + 3, Jz = 0.5 * 3.
        let mut jz = 1.5;
        let mut jx = 6.0;
        for (i, c) in coords.iter().enumerate() {
            let mass = [2.0, 5.0, 3.0][i];
            jz += mass * ((c[0] - xc).powi(2) + (c[1] - yc).powi(2));
            jx += mass * ((c[1] - yc).powi(2) + (c[2] - zc).powi(2));
        }
        assert!((shapes.total[5] - jz).abs() < 1e-12, "{} {jz}", shapes.total[5]);
        assert!((shapes.total[3] - jx).abs() < 1e-12, "{} {jx}", shapes.total[3]);
    }

    /// The documented exception: a joint turned about Z with different masses
    /// along its local axes has a global mass matrix with an X-Y coupling
    /// term, so the translations are not orthogonal to each other any more
    /// and the centroid rule cannot make the rotation about Z independent.
    /// With R[0] = (c, -s), R[1] = (s, c) and local masses 1 and 4:
    /// r_UX^T M r_UY = 1 c s + 4 (-s) c = -3 c s.
    #[test]
    fn anisotropic_rotated_masses_leave_a_coupling_term() {
        let (c, s) = (30f64.to_radians().cos(), 30f64.to_radians().sin());
        let rot = vec![[[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]];
        let coords = vec![[2.0, 1.0, 0.0]];
        let m = vec![1.0, 4.0, 0.0, 0.0, 0.0, 0.0];
        let shapes = RigidBodyShapes::new(&m, &rot, &coords);
        let coupling = weighted_dot(&m, &shapes.r[0], &shapes.r[1]);
        assert!((coupling + 3.0 * c * s).abs() < 1e-12, "{coupling}");
        assert!(coupling.abs() > 1.0);
    }

    /// A model with a rotated joint does not take the identity fast path; one
    /// with exact identity axes does, and then the translational factor is
    /// the plain `i % 6` sum, bit for bit.
    #[test]
    fn identity_axes_take_the_plain_accumulation() {
        let (m, coords) = scattered();
        let shapes = RigidBodyShapes::new(&m, &vec![I3; 3], &coords);
        assert!(shapes.identity_axes);
        let phi: Vec<f64> = (0..m.len()).map(|i| 0.1 * i as f64 - 0.7).collect();
        let pf = shapes.factors(&m, &phi);
        let mut want = 0.0;
        for i in 0..m.len() {
            if i % 6 == 1 {
                want += m[i] * phi[i];
            }
        }
        assert_eq!(pf[1].to_bits(), want.to_bits());
        let mut rot = vec![I3; 3];
        rot[1] = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
        assert!(!RigidBodyShapes::new(&m, &rot, &coords).identity_axes);
    }
}
