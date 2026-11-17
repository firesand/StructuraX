//! Mode-superposition time-history analysis.
//!
//! A `LinearHistory` case with `modal_case: Some(m)` superposes the modes of
//! the Modal case `m` (CSI Analysis Reference, "Linear Time-History Analysis",
//! topics "Time Steps" and "Modal Time-History Analysis", printed pp. 402-405).
//!
//! Supported model
//! ---------------
//! * Loads `r(t) = Σᵢ fᵢ(t) pᵢ`: each history entry is a function, a spatial
//!   vector `pᵢ` (a load pattern of joint forces, or a global ground
//!   acceleration `UX`/`UY`/`UZ` giving `pᵢ = −M·rᵢ`) and a scale factor.
//! * Damping: one constant ratio `ξ = modal_damping`, `0 <= ξ < 1`, for every
//!   mode (uncoupled modal damping). Zero initial conditions (the reference
//!   states that linear transient analyses always start at rest).
//! * Time functions are piecewise linear between their points. Before the
//!   first point the value is zero and after the last point it is held
//!   constant at the last value (reference p. 401: "the value of the Function
//!   for all time t < t0 is taken as zero, and ... for all time t > tn is held
//!   constant at fn"). A function whose first value is not zero therefore
//!   jumps at `t0`. The built-in ramp, arrival time and time-scale factors of
//!   the reference are not representable in `history_loads` and are not
//!   implemented.
//!
//! Integration
//! -----------
//! The load is piecewise linear between breakpoints, the union of the output
//! times `k·dt` and every function abscissa inside the analysis span (the
//! reference's "load steps"). On each piece the modal equation
//! `q̈ + 2ξω q̇ + ω² q = p₀ + (p₁ − p₀)τ/h` is integrated exactly: the particular
//! solution is the line `A + Bτ` with `B = s/ω²`, `A = (p₀ − 2ξs/ω)/ω²`
//! (`s` the slope), and the homogeneous part is the damped free vibration from
//! the remaining initial state. This is the closed-form recurrence of Chopra,
//! *Dynamics of Structures*, §5.2 (Nigam & Jennings 1969); `ξ = 0` needs no
//! special case. Relative accelerations are taken from the equation of motion
//! itself, `q̈ = p − 2ξω q̇ − ω² q`.
//!
//! Outputs
//! -------
//! Displacements `u = Σ φₙqₙ` and relative accelerations `Σ φₙq̈ₙ` at the output
//! steps (`time_history_disp`, `time_history_accel`); joint displacements,
//! frame forces and reactions are those at the step of largest displacement
//! norm, recovered like the direct-integration path. Reactions there are
//! `K·u` plus the inertia of the mass at restrained degrees of freedom minus
//! the applied load at them; the force carried by modal damping has no
//! physical damping matrix and is not part of them. Reactions converge slowly
//! with the number of modes unless Ritz vectors are used (reference p. 404).
//! The case leaves no `transient_state`: a modal history cannot seed another
//! case.
//!
//! Not supported (refused with a reason): Rayleigh damping, an initial case,
//! link damping (the reference converts it to modal damping, which is not
//! implemented), a non-Modal, disabled or mode-less modal case, non-positive
//! frequencies.

use crate::{
    assemble_static_pattern_rhs, link, recover_results_from_displacement_vector, SolverError,
};
use indexmap::IndexMap;
use sap_model::{AreaObj, CaseResults, LoadCase, LoadCaseType, PointObj, SapModel};

/// True for the history load names that mean a global ground acceleration.
fn acceleration_direction(name: &str) -> Option<usize> {
    match name.trim().to_ascii_uppercase().as_str() {
        "U1" | "UX" | "X" => Some(0),
        "U2" | "UY" | "Y" => Some(1),
        "U3" | "UZ" | "Z" => Some(2),
        _ => None,
    }
}

/// Checks everything about a modal history case that can be decided before
/// the analysis runs. `name` is the case name.
pub(crate) fn validate_case(
    model: &SapModel,
    name: &str,
    lc: &LoadCase,
) -> Result<(), SolverError> {
    if lc.case_type == LoadCaseType::NonlinearStatic || !lc.nl_static.mode_loads.is_empty() {
        return validate_static_mode_case(model, name, lc);
    }
    let Some(modal_name) = &lc.modal_case else {
        return Ok(());
    };
    if lc.case_type != LoadCaseType::LinearHistory {
        return Err(SolverError::UnsupportedInput(format!(
            "case '{name}' of type {:?} names modal case '{modal_name}'; only a LinearHistory case can superpose the modes of a modal case",
            lc.case_type
        )));
    }
    if lc.rayleigh_damping != (0.0, 0.0) {
        return Err(SolverError::UnsupportedInput(format!(
            "case '{name}' superposes modes and has Rayleigh damping {:?}; modal damping uses one damping ratio, set the Rayleigh coefficients to zero",
            lc.rayleigh_damping
        )));
    }
    if let Some(initial) = &lc.initial_case {
        return Err(SolverError::UnsupportedInput(format!(
            "case '{name}' superposes modes and continues from case '{initial}'; linear modal history starts at rest"
        )));
    }
    if !lc.modal_damping.is_finite() || !(0.0..1.0).contains(&lc.modal_damping) {
        return Err(SolverError::InvalidInput(format!(
            "case '{name}' has modal damping ratio {}; it must satisfy 0 <= ratio < 1",
            lc.modal_damping
        )));
    }
    let modal = model.load_cases.get(modal_name).ok_or_else(|| {
        SolverError::InvalidInput(format!(
            "case '{name}' superposes the modes of missing case '{modal_name}'"
        ))
    })?;
    if modal.case_type != LoadCaseType::Modal {
        return Err(SolverError::UnsupportedInput(format!(
            "case '{name}' superposes the modes of '{modal_name}', which is a {:?} case, not a Modal case",
            modal.case_type
        )));
    }
    if !modal.run_flag {
        return Err(SolverError::InvalidInput(format!(
            "case '{name}' superposes the modes of '{modal_name}', which is not flagged to run"
        )));
    }
    if modal.max_modes == 0 {
        return Err(SolverError::InvalidInput(format!(
            "case '{name}' superposes the modes of '{modal_name}', which requests no modes"
        )));
    }
    for (link_name, link) in &model.links {
        let Some(prop) = model.link_props.get(&link.prop) else {
            continue;
        };
        if (0..6).any(|d| prop.dof_active[d] && !prop.dof_fixed[d] && prop.ce[d] != 0.0) {
            return Err(SolverError::UnsupportedInput(format!(
                "case '{name}' superposes modes and link '{link_name}' has viscous damping; its conversion to modal damping is not implemented"
            )));
        }
    }
    Ok(())
}

/// The modal case of a nonlinear static case: it is named if and only if the
/// case has Mode loads, must be a Modal case flagged to run, and must have
/// the requested modes; Mode loads are the only loads of their case.
fn validate_static_mode_case(
    model: &SapModel,
    name: &str,
    lc: &LoadCase,
) -> Result<(), SolverError> {
    let modes = &lc.nl_static.mode_loads;
    if lc.case_type != LoadCaseType::NonlinearStatic {
        if modes.is_empty() {
            return Ok(());
        }
        return Err(SolverError::UnsupportedInput(format!(
            "case '{name}' of type {:?} has Mode loads; Mode loads apply to nonlinear static cases only",
            lc.case_type
        )));
    }
    match (&lc.modal_case, modes.is_empty()) {
        (None, true) => return Ok(()),
        (None, false) => {
            return Err(SolverError::InvalidInput(format!(
                "case '{name}' has Mode loads but names no modal case"
            )))
        }
        (Some(modal_name), true) => {
            return Err(SolverError::UnsupportedInput(format!(
                "nonlinear static case '{name}' names modal case '{modal_name}' but has no Mode load"
            )))
        }
        (Some(_), false) => {}
    }
    let modal_name = lc.modal_case.as_ref().expect("checked above");
    if !lc.loads.is_empty() {
        return Err(SolverError::UnsupportedInput(format!(
            "case '{name}' mixes Mode loads with other loads; the scale of a modal load depends on the mode normalization, so a Mode load must be the only load of its case"
        )));
    }
    let modal = model.load_cases.get(modal_name).ok_or_else(|| {
        SolverError::InvalidInput(format!(
            "case '{name}' applies Mode loads of missing case '{modal_name}'"
        ))
    })?;
    if modal.case_type != LoadCaseType::Modal {
        return Err(SolverError::UnsupportedInput(format!(
            "case '{name}' applies Mode loads of '{modal_name}', which is a {:?} case, not a Modal case",
            modal.case_type
        )));
    }
    if !modal.run_flag {
        return Err(SolverError::InvalidInput(format!(
            "case '{name}' applies Mode loads of '{modal_name}', which is not flagged to run"
        )));
    }
    for load in modes {
        if load.mode == 0 || load.mode > modal.max_modes {
            return Err(SolverError::InvalidInput(format!(
                "case '{name}': Mode load {} is outside the modes 1..={} of '{modal_name}'",
                load.mode, modal.max_modes
            )));
        }
        if !load.scale_factor.is_finite() {
            return Err(SolverError::InvalidInput(format!(
                "case '{name}': Mode load {} has a nonfinite scale factor",
                load.mode
            )));
        }
    }
    Ok(())
}

/// Exact state after a step of length `h` under the load
/// `p0 + (p1 - p0) τ / h`, from displacement `q` and velocity `v`, for
/// `q̈ + 2ξω q̇ + ω² q = p`.
pub(crate) fn advance_mode(
    omega: f64,
    xi: f64,
    h: f64,
    p0: f64,
    p1: f64,
    q: f64,
    v: f64,
) -> (f64, f64) {
    let w2 = omega * omega;
    let slope = (p1 - p0) / h;
    let b = slope / w2;
    let a = (p0 - 2.0 * xi * slope / omega) / w2;
    let x0 = q - a;
    let y0 = v - b;
    let wd = omega * (1.0 - xi * xi).sqrt();
    let decay = xi * omega;
    let e = (-decay * h).exp();
    let (sn, cs) = (wd * h).sin_cos();
    let c = (y0 + decay * x0) / wd;
    let x1 = e * (x0 * cs + c * sn);
    let y1 = e * (y0 * cs - (x0 * wd + decay * c) * sn);
    (x1 + a + b * h, y1 + b)
}

/// Global ground acceleration `[ax, ay, az]` (model length/s²) at every output
/// step `k·dt`, `k = 0..=num_time_steps`, of the linear history case `case`,
/// which must name a modal case (mode superposition): the sum of
/// `scale factor · f(t)` over the case's acceleration entries (`UX`, `UY`,
/// `UZ`, and the spellings this module accepts for them), with this module's
/// function rule (`function_value` with a tolerance of `1e-9·dt`). Load
/// pattern entries add nothing.
pub fn ground_acceleration_trace(
    model: &SapModel,
    case: &str,
) -> Result<Vec<[f64; 3]>, SolverError> {
    let lc = model
        .load_cases
        .get(case)
        .ok_or_else(|| SolverError::InvalidInput(format!("case '{case}' does not exist")))?;
    if lc.case_type != LoadCaseType::LinearHistory {
        return Err(SolverError::UnsupportedInput(format!(
            "case '{case}' is of type {:?}; ground accelerations are traced for modal linear history cases",
            lc.case_type
        )));
    }
    if lc.modal_case.is_none() {
        return Err(SolverError::UnsupportedInput(format!(
            "case '{case}' is a direct-integration history case; only modal time-history cases record accelerations"
        )));
    }
    let dt = lc.time_step_dt;
    if !dt.is_finite() || dt <= 0.0 {
        return Err(SolverError::InvalidInput(format!(
            "case '{case}' has time step {dt}; it must be positive"
        )));
    }
    let mut entries: Vec<(usize, f64, &[(f64, f64)])> = Vec::new();
    for (function, target, sf) in &lc.history_loads {
        let points = model
            .functions
            .get(function)
            .map(|f| f.points.as_slice())
            .ok_or_else(|| {
                SolverError::InvalidInput(format!(
                    "case '{case}' references missing function '{function}'"
                ))
            })?;
        if let Some(dir) = acceleration_direction(target) {
            entries.push((dir, *sf, points));
        }
    }
    let tol = 1e-9 * dt;
    Ok((0..=lc.num_time_steps)
        .map(|k| {
            let t = k as f64 * dt;
            let mut a = [0.0; 3];
            for (dir, sf, points) in &entries {
                a[*dir] += sf * function_value(points, t, tol);
            }
            a
        })
        .collect())
}

/// Time function value, zero before the first point and constant after the
/// last. Right-continuous: at a jump the later value is returned. Times within
/// `tol` of a point count as that point.
pub fn function_value(points: &[(f64, f64)], t: f64, tol: f64) -> f64 {
    let Some(&(t0, _)) = points.first() else {
        return 0.0;
    };
    let (tn, yn) = points[points.len() - 1];
    if t < t0 - tol {
        return 0.0;
    }
    if t >= tn - tol {
        return yn;
    }
    for w in points.windows(2) {
        let ((x0, y0), (x1, y1)) = (w[0], w[1]);
        if t < x1 - tol {
            let s = t.clamp(x0, x1);
            return y0 + (y1 - y0) * (s - x0) / (x1 - x0);
        }
    }
    yn
}

/// Values of the function at the start (right limit) and the end (left limit)
/// of a piece `[ta, tb]` on which it is linear. The abscissae of the function
/// are breakpoints, so the piece lies inside one interval of the function or
/// outside all of them.
fn function_piece(points: &[(f64, f64)], ta: f64, tb: f64) -> (f64, f64) {
    let Some(&(t0, _)) = points.first() else {
        return (0.0, 0.0);
    };
    let (tn, yn) = points[points.len() - 1];
    let tm = 0.5 * (ta + tb);
    if tm < t0 {
        return (0.0, 0.0);
    }
    if tm >= tn {
        return (yn, yn);
    }
    for w in points.windows(2) {
        let ((x0, y0), (x1, y1)) = (w[0], w[1]);
        if tm < x1 {
            let line = |t: f64| y0 + (y1 - y0) * (t - x0) / (x1 - x0);
            return (line(ta), line(tb));
        }
    }
    (yn, yn)
}

/// Output steps and function abscissae merged into one ascending list of
/// `(time, output step index)`.
fn breakpoints(dt: f64, n_steps: usize, abscissae: &[f64]) -> Vec<(f64, Option<usize>)> {
    let t_end = n_steps as f64 * dt;
    let tol = 1e-9 * dt;
    let mut all: Vec<(f64, Option<usize>)> = (0..=n_steps)
        .map(|k| (k as f64 * dt, Some(k)))
        .chain(
            abscissae
                .iter()
                .copied()
                .filter(|x| *x > tol && *x < t_end - tol)
                .map(|x| (x, None)),
        )
        .collect();
    all.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut merged: Vec<(f64, Option<usize>)> = Vec::with_capacity(all.len());
    for (t, tag) in all {
        if let Some(last) = merged.last_mut() {
            if t - last.0 <= tol {
                if last.1.is_none() && tag.is_some() {
                    *last = (t, tag);
                }
                continue;
            }
        }
        merged.push((t, tag));
    }
    merged
}

/// Runs a linear modal time-history case.
#[allow(clippy::too_many_arguments)]
pub(crate) fn run_case(
    model: &SapModel,
    lc: &LoadCase,
    work_points: &IndexMap<String, PointObj>,
    work_areas: &IndexMap<String, AreaObj>,
    orig_area_map: &IndexMap<String, Vec<String>>,
    pt_index: &IndexMap<String, usize>,
    joint_rot: &[[[f64; 3]; 3]],
    k_struct: &[f64],
    k_base: &[f64],
    diag_mass: &[f64],
    ndof: usize,
) -> Result<CaseResults, SolverError> {
    validate_case(model, &lc.name, lc)?;
    let modal_name = lc.modal_case.as_ref().ok_or_else(|| {
        SolverError::InvalidInput(format!("case '{}' has no modal case", lc.name))
    })?;
    let modal = model.case_results.get(modal_name).ok_or_else(|| {
        SolverError::InvalidInput(format!(
            "case '{}' needs the modes of '{modal_name}', which has not been analysed",
            lc.name
        ))
    })?;
    if modal.modes.is_empty() {
        return Err(SolverError::InvalidInput(format!(
            "case '{}' superposes the modes of '{modal_name}', which has none",
            lc.name
        )));
    }

    // Mode shapes in the global degree-of-freedom layout, frequencies, and
    // generalized masses (1 for the normalization of the modal solver, but the
    // value is recomputed so another normalization stays correct).
    let xi = lc.modal_damping;
    let mut phi: Vec<Vec<f64>> = Vec::with_capacity(modal.modes.len());
    let mut omega: Vec<f64> = Vec::with_capacity(modal.modes.len());
    let mut gen_mass: Vec<f64> = Vec::with_capacity(modal.modes.len());
    for mode in &modal.modes {
        let w = mode.circ_freq_rad_s;
        if !w.is_finite() || w <= 0.0 {
            return Err(SolverError::UnsupportedInput(format!(
                "case '{}' superposes mode {} of '{modal_name}' whose circular frequency is {w}; modes must have a positive frequency",
                lc.name, mode.mode_num
            )));
        }
        let mut shape = vec![0.0; ndof];
        for (pname, disp) in &mode.mode_shape {
            if let Some(&idx) = pt_index.get(pname) {
                shape[6 * idx..6 * idx + 6].copy_from_slice(disp);
            }
        }
        let mu: f64 = shape.iter().zip(diag_mass).map(|(f, m)| m * f * f).sum();
        if !mu.is_finite() || mu <= 0.0 {
            return Err(SolverError::InvalidInput(format!(
                "case '{}': mode {} of '{modal_name}' has no mass",
                lc.name, mode.mode_num
            )));
        }
        phi.push(shape);
        omega.push(w);
        gen_mass.push(mu);
    }
    let n_modes = phi.len();

    // Spatial vectors, as run_time_history_case builds them.
    let mut spatial: Vec<Vec<f64>> = Vec::with_capacity(lc.history_loads.len());
    for (_, pat_or_acc, sf) in &lc.history_loads {
        let vector = if let Some(dir) = acceleration_direction(pat_or_acc) {
            let mut f = vec![0.0f64; ndof];
            for idx in 0..work_points.len() {
                for d in 0..3 {
                    let gdof = 6 * idx + d;
                    f[gdof] = -diag_mass[gdof] * joint_rot[idx][dir][d] * (*sf);
                }
            }
            f
        } else {
            assemble_static_pattern_rhs(
                model,
                &[(pat_or_acc.clone(), *sf)],
                work_points,
                work_areas,
                pt_index,
                joint_rot,
                ndof,
            )?
        };
        spatial.push(vector);
    }
    // Participation of every entry in every mode: φₙᵀpᵢ / φₙᵀMφₙ.
    let participation: Vec<Vec<f64>> = (0..n_modes)
        .map(|n| {
            spatial
                .iter()
                .map(|p| phi[n].iter().zip(p).map(|(f, x)| f * x).sum::<f64>() / gen_mass[n])
                .collect()
        })
        .collect();
    let functions: Vec<&[(f64, f64)]> = lc
        .history_loads
        .iter()
        .map(|(name, _, _)| {
            model
                .functions
                .get(name)
                .map(|f| f.points.as_slice())
                .ok_or_else(|| {
                    SolverError::InvalidInput(format!(
                        "case '{}' references missing function '{name}'",
                        lc.name
                    ))
                })
        })
        .collect::<Result<_, _>>()?;

    let dt = lc.time_step_dt;
    let n_steps = lc.num_time_steps;
    let abscissae: Vec<f64> = functions
        .iter()
        .flat_map(|points| points.iter().map(|(x, _)| *x))
        .collect();
    let marks = breakpoints(dt, n_steps, &abscissae);
    let tol = 1e-9 * dt;

    let mut q = vec![0.0f64; n_modes];
    let mut v = vec![0.0f64; n_modes];
    let mut time_steps = Vec::with_capacity(n_steps + 1);
    let mut time_history_disp: IndexMap<String, Vec<[f64; 6]>> = IndexMap::new();
    let mut time_history_accel: IndexMap<String, Vec<[f64; 6]>> = IndexMap::new();
    for pname in model.points.keys() {
        time_history_disp.insert(pname.clone(), Vec::with_capacity(n_steps + 1));
        time_history_accel.insert(pname.clone(), Vec::with_capacity(n_steps + 1));
    }
    let mut u_peak = vec![0.0f64; ndof];
    let mut v_peak = vec![0.0f64; ndof];
    let mut a_peak = vec![0.0f64; ndof];
    let mut t_peak = 0.0f64;
    let mut peak_norm = 0.0f64;

    let modal_load = |values: &[f64], n: usize| -> f64 {
        participation[n]
            .iter()
            .zip(values)
            .map(|(s, f)| s * f)
            .sum()
    };

    for (index, &(t, tag)) in marks.iter().enumerate() {
        if index > 0 {
            let t_prev = marks[index - 1].0;
            let h = t - t_prev;
            let pieces: Vec<(f64, f64)> = functions
                .iter()
                .map(|points| function_piece(points, t_prev, t))
                .collect();
            let start: Vec<f64> = pieces.iter().map(|p| p.0).collect();
            let end: Vec<f64> = pieces.iter().map(|p| p.1).collect();
            for n in 0..n_modes {
                let (q1, v1) = advance_mode(
                    omega[n],
                    xi,
                    h,
                    modal_load(&start, n),
                    modal_load(&end, n),
                    q[n],
                    v[n],
                );
                q[n] = q1;
                v[n] = v1;
            }
        }
        let Some(step) = tag else {
            continue;
        };
        debug_assert!(step <= n_steps);
        let values: Vec<f64> = functions
            .iter()
            .map(|points| function_value(points, t, tol))
            .collect();
        let mut u = vec![0.0f64; ndof];
        let mut vel = vec![0.0f64; ndof];
        let mut acc = vec![0.0f64; ndof];
        for n in 0..n_modes {
            let qdd =
                modal_load(&values, n) - 2.0 * xi * omega[n] * v[n] - omega[n] * omega[n] * q[n];
            for i in 0..ndof {
                let f = phi[n][i];
                if f != 0.0 {
                    u[i] += f * q[n];
                    vel[i] += f * v[n];
                    acc[i] += f * qdd;
                }
            }
        }
        time_steps.push(step as f64 * dt);
        let norm: f64 = u.iter().map(|x| x * x).sum();
        if norm >= peak_norm {
            peak_norm = norm;
            u_peak.clone_from(&u);
            v_peak.clone_from(&vel);
            a_peak.clone_from(&acc);
            t_peak = t;
        }
        for (pname, trace) in &mut time_history_disp {
            let idx = pt_index[pname];
            trace.push(std::array::from_fn(|d| u[6 * idx + d]));
        }
        for (pname, trace) in &mut time_history_accel {
            let idx = pt_index[pname];
            trace.push(std::array::from_fn(|d| acc[6 * idx + d]));
        }
    }

    let mut res = recover_results_from_displacement_vector(
        model,
        work_points,
        work_areas,
        orig_area_map,
        pt_index,
        joint_rot,
        k_struct,
        &u_peak,
        ndof,
    );

    // Reaction at a restrained degree of freedom: elastic force through the
    // stiffness with springs and constraints, plus the inertia of the mass
    // there, minus the load applied there at the step of the peak.
    let peak_values: Vec<f64> = functions
        .iter()
        .map(|points| function_value(points, t_peak, tol))
        .collect();
    let mut applied_peak = vec![0.0f64; ndof];
    for (vector, value) in spatial.iter().zip(&peak_values) {
        for i in 0..ndof {
            applied_peak[i] += value * vector[i];
        }
    }
    for (name, reaction) in &mut res.joint_reactions {
        let idx = pt_index[name];
        let point = &work_points[name];
        let mut delta = [0.0; 6];
        for d in 0..6 {
            let i = 6 * idx + d;
            if point.restraint[d] {
                let internal: f64 = (0..ndof).map(|j| k_base[i * ndof + j] * u_peak[j]).sum();
                let physical = internal + diag_mass[i] * a_peak[i] - applied_peak[i];
                delta[d] = physical - reaction[d];
            }
            reaction[d] += delta[d];
        }
        let r = joint_rot[idx];
        let global: [f64; 6] =
            std::array::from_fn(|i| (0..3).map(|j| r[i % 3][j] * delta[3 * (i / 3) + j]).sum());
        for d in 0..6 {
            res.base_reactions[d] += global[d];
        }
        res.base_reactions[3] += point.y * global[2] - point.z * global[1];
        res.base_reactions[4] += point.z * global[0] - point.x * global[2];
        res.base_reactions[5] += point.x * global[1] - point.y * global[0];
    }

    // Link forces and the reactions of grounded links at the peak.
    let mut grounded = IndexMap::<String, [f64; 6]>::new();
    for (name, object) in &model.links {
        let prop = &model.link_props[&object.prop];
        let op = link::operator(object, prop, work_points, pt_index, joint_rot);
        let raw = link::dynamic_spring_forces(prop, &op, &u_peak, &v_peak, &[None; 6], 0.0);
        res.link_forces
            .insert(name.clone(), link::section_at_j(prop, raw));
        if object.point_j.is_none() {
            let force = grounded.entry(object.point_i.clone()).or_insert([0.0; 6]);
            for (c, value) in force.iter_mut().enumerate() {
                *value -= (0..6).map(|d| op.b[d][c] * raw[d]).sum::<f64>();
            }
        }
    }
    crate::add_reaction_contributions(
        grounded,
        work_points,
        pt_index,
        joint_rot,
        &mut res.joint_reactions,
        &mut res.base_reactions,
    );

    res.case_name = lc.name.clone();
    res.time_steps = time_steps;
    res.time_history_disp = time_history_disp;
    res.time_history_accel = time_history_accel;
    Ok(res)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Independent integration of q'' + 2 xi w q' + w^2 q = p0 + s t with RK4.
    fn rk4(omega: f64, xi: f64, p0: f64, p1: f64, h: f64, q0: f64, v0: f64) -> (f64, f64) {
        let n = 20000;
        let dt = h / n as f64;
        let s = (p1 - p0) / h;
        let f = |t: f64, q: f64, v: f64| (v, p0 + s * t - 2.0 * xi * omega * v - omega * omega * q);
        let (mut q, mut v) = (q0, v0);
        for k in 0..n {
            let t = k as f64 * dt;
            let (k1q, k1v) = f(t, q, v);
            let (k2q, k2v) = f(t + dt / 2.0, q + dt / 2.0 * k1q, v + dt / 2.0 * k1v);
            let (k3q, k3v) = f(t + dt / 2.0, q + dt / 2.0 * k2q, v + dt / 2.0 * k2v);
            let (k4q, k4v) = f(t + dt, q + dt * k3q, v + dt * k3v);
            q += dt / 6.0 * (k1q + 2.0 * k2q + 2.0 * k3q + k4q);
            v += dt / 6.0 * (k1v + 2.0 * k2v + 2.0 * k3v + k4v);
        }
        (q, v)
    }

    #[test]
    fn recurrence_matches_numerical_integration_of_the_modal_equation() {
        for (omega, xi) in [(2.0, 0.0), (7.0, 0.05), (3.0, 0.6)] {
            let (q, v) = advance_mode(omega, xi, 0.4, 1.5, -2.0, 0.3, -0.7);
            let (qr, vr) = rk4(omega, xi, 1.5, -2.0, 0.4, 0.3, -0.7);
            assert!((q - qr).abs() < 1e-10, "{omega} {xi}: {q} {qr}");
            assert!((v - vr).abs() < 1e-10, "{omega} {xi}: {v} {vr}");
        }
    }

    /// A mode with a non-positive circular frequency cannot be superposed.
    /// The modal solver never produces one for a restrained structure, so the
    /// stored result is edited and the runner called directly.
    #[test]
    fn modes_with_non_positive_frequency_are_refused() {
        let mut model = SapModel::default();
        model.load_cases.clear();
        model.active_dofs = [true, false, false, false, false, false];
        model.add_point("J", 0.0, 0.0, 0.0);
        model.points["J"].mass[0] = 2.0;
        model.points["J"].spring[0] = 8.0;
        model
            .load_cases
            .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 1));
        crate::analyze_model(&mut model).unwrap();
        let mut hist = LoadCase::linear_static("HIST", "DEAD", 1.0);
        hist.case_type = LoadCaseType::LinearHistory;
        hist.modal_case = Some("MODAL".into());
        hist.modal_damping = 0.05;
        hist.num_time_steps = 2;
        hist.time_step_dt = 0.1;
        let points = model.points.clone();
        let pt_index: IndexMap<String, usize> = points
            .keys()
            .enumerate()
            .map(|(i, k)| (k.clone(), i))
            .collect();
        let rot = vec![[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]];
        let k = vec![0.0; 36];
        let mass = [2.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let areas = IndexMap::new();
        let area_map = IndexMap::new();
        let run = |model: &SapModel| {
            run_case(
                model, &hist, &points, &areas, &area_map, &pt_index, &rot, &k, &k, &mass, 6,
            )
        };
        for bad in [0.0, -3.0, f64::NAN] {
            model.case_results["MODAL"].modes[0].circ_freq_rad_s = bad;
            let error = run(&model).unwrap_err().to_string();
            assert!(error.contains("positive frequency"), "{bad}: {error}");
        }
        // With the true frequency the same call succeeds.
        model.case_results["MODAL"].modes[0].circ_freq_rad_s = 2.0;
        assert!(run(&model).is_ok());
    }

    #[test]
    fn breakpoints_merge_abscissae_and_output_times() {
        let marks = breakpoints(0.02, 5, &[0.0, 0.03, 0.06, 0.06000000000000001, 0.5]);
        let times: Vec<f64> = marks.iter().map(|m| m.0).collect();
        // 0.03 is new, 0.06 coincides with the output time 3*0.02, 0.5 is
        // outside the span 0.1 and 0.0 coincides with the first output.
        assert_eq!(times.len(), 7);
        assert!(marks.iter().filter(|m| m.1.is_some()).count() == 6);
        assert!(marks.windows(2).all(|w| w[0].0 < w[1].0));
    }

    #[test]
    fn function_is_zero_before_and_constant_after() {
        let f = [(1.0, 2.0), (3.0, 6.0)];
        assert_eq!(function_value(&f, 0.5, 1e-12), 0.0);
        assert_eq!(function_value(&f, 1.0, 1e-12), 2.0);
        assert_eq!(function_value(&f, 2.0, 1e-12), 4.0);
        assert_eq!(function_value(&f, 3.0, 1e-12), 6.0);
        assert_eq!(function_value(&f, 9.0, 1e-12), 6.0);
        assert_eq!(function_piece(&f, 0.0, 1.0), (0.0, 0.0));
        assert_eq!(function_piece(&f, 1.0, 3.0), (2.0, 6.0));
        assert_eq!(function_piece(&f, 3.0, 4.0), (6.0, 6.0));
    }
}
