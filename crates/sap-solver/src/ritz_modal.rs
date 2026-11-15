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
/// as are needed) or when no load is active.
pub(crate) fn generate_ritz_modes(
    k: &[f64],
    ldlt: &SymmetricLdlt,
    m: &[f64],
    is_fixed: &[bool],
    loads: &[StartingLoad],
    max_modes: usize,
) -> Result<RitzModes, SolverError> {
    let n = ldlt.n;
    let mut states: Vec<LoadState> = loads
        .iter()
        .map(|l| LoadState {
            active: true,
            created: 0,
            last: Vec::new(),
            dyn_sum: 0.0,
            a_p: a_dot_p(m, is_fixed, &l.p),
        })
        .collect();
    let mut basis: Vec<Vec<f64>> = Vec::new();
    let mut kb: Vec<Vec<f64>> = Vec::new();

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
            kb.push(matvec(k, &cand));
            basis.push(cand);
        }
    }

    let nb = basis.len();
    if nb == 0 {
        return Err(SolverError::UnsupportedInput(
            "no Ritz vector could be generated from the starting loads".into(),
        ));
    }
    // Rayleigh-Ritz on span(basis): K_r z = w^2 z, because the basis is
    // M-orthonormal (B^T M B = I).
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
    let (vals, vecs) = jacobi_symmetric_eigen(&kr, nb, 80)?;
    let mut order: Vec<usize> = (0..nb).collect();
    order.sort_by(|&a, &b| vals[a].total_cmp(&vals[b]));
    let mut modes = Vec::with_capacity(nb);
    for &j in &order {
        let w2 = vals[j];
        if w2.is_nan() || w2 <= 0.0 || !w2.is_finite() {
            return Err(SolverError::InvalidInput(format!(
                "Ritz projection produced a non-positive eigenvalue {w2:e}; the stiffness matrix is not positive definite"
            )));
        }
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
