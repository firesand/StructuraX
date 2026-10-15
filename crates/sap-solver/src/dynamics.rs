//! Linear direct integration with diagonal mass, Rayleigh and element damping.
//!
//! HHT uses the CSI convention -1/3 <= alpha <= 0 and weighted equilibrium,
//! with forces linearly interpolated between physical step endpoints. Wilson
//! solves at t + theta*dt with extrapolated force, then returns to t + dt.
//! References and applicability limits: docs/reconstruction/solver-wave1.md.

use crate::{linalg::SymmetricLdlt, SolverError};
use sap_model::{LoadCase, TimeIntegrationMethod};

pub(crate) fn validate_parameters(case: &LoadCase) -> Result<(), SolverError> {
    let invalid =
        |reason: &str| SolverError::InvalidInput(format!("case '{}': {reason}", case.name));
    if !case.time_step_dt.is_finite() || case.time_step_dt <= 0.0 || case.num_time_steps == 0 {
        return Err(invalid(
            "time step and step count must be positive and finite",
        ));
    }
    if [case.rayleigh_damping.0, case.rayleigh_damping.1]
        .iter()
        .any(|x| !x.is_finite() || *x < 0.0)
    {
        return Err(invalid(
            "Rayleigh damping coefficients must be finite and nonnegative",
        ));
    }
    if case.time_integration == TimeIntegrationMethod::HilberHughesTaylor
        && (!case.hht_alpha.is_finite() || !(-1.0 / 3.0..=0.0).contains(&case.hht_alpha))
    {
        return Err(invalid("HHT alpha must be in [-1/3, 0]"));
    }
    if case.time_integration == TimeIntegrationMethod::WilsonTheta
        && (!case.wilson_theta.is_finite() || case.wilson_theta < 1.0)
    {
        return Err(invalid("Wilson theta must be finite and at least 1"));
    }
    Ok(())
}

#[derive(Clone, Debug)]
pub(crate) struct DynamicState {
    pub u: Vec<f64>,
    pub v: Vec<f64>,
    pub a: Vec<f64>,
}

struct MasslessBlock {
    dofs: Vec<usize>,
    factor: SymmetricLdlt,
}

pub(crate) struct LinearStepper {
    n: usize,
    k: Vec<f64>,
    c: Vec<f64>,
    mass: Vec<f64>,
    fixed: Vec<bool>,
    factor: SymmetricLdlt,
    massless: Option<MasslessBlock>,
    stiffness_damping: f64,
    dt: f64,
    theta: f64,
    alpha: f64,
    gamma: f64,
    a0: f64,
    a1: f64,
    a2: f64,
    a3: f64,
    a4: f64,
    a5: f64,
    previous_force: Vec<f64>,
    pub state: DynamicState,
}

impl LinearStepper {
    #[cfg(test)]
    pub fn new(k: &[f64], mass: &[f64], fixed: &[bool], case: &LoadCase,
        force: &[f64], force_slope: &[f64]) -> Result<Self, SolverError> {
        Self::with_element_damping(k, mass, fixed, case, force, force_slope, &vec![0.; k.len()])
    }

    pub fn with_element_damping(
        k: &[f64],
        mass: &[f64],
        fixed: &[bool],
        case: &LoadCase,
        force: &[f64],
        force_slope: &[f64],
        element_damping: &[f64],
    ) -> Result<Self, SolverError> {
        validate_parameters(case)?;
        let n = mass.len();
        if k.len() != n * n || element_damping.len() != n*n || fixed.len() != n || force.len() != n || force_slope.len() != n {
            return Err(SolverError::InvalidInput(
                "dynamic system dimensions differ".into(),
            ));
        }
        if mass.iter().any(|m| !m.is_finite() || *m < 0.0)
            || k.iter()
                .chain(element_damping)
                .chain(force)
                .chain(force_slope)
                .any(|x| !x.is_finite())
        {
            return Err(SolverError::InvalidInput(
                "dynamic system needs finite data and nonnegative masses".into(),
            ));
        }
        // The proportional-damping massless initializer below relies on Czz=b*Kzz.
        // A general singular first-order/algebraic block requires a separate DAE
        // reduction. Reject it instead of silently projecting away dashpot forces.
        for i in 0..n {
            if !fixed[i] && mass[i] == 0.0 && (0..n).any(|j| !fixed[j] && element_damping[i*n+j] != 0.) {
                return Err(SolverError::UnsupportedInput("link damping on a free massless coordinate requires general DAE initialization".into()));
            }
        }
        let alpha = if case.time_integration == TimeIntegrationMethod::HilberHughesTaylor {
            case.hht_alpha
        } else {
            0.0
        };
        let theta = if case.time_integration == TimeIntegrationMethod::WilsonTheta {
            case.wilson_theta
        } else {
            1.0
        };
        let (gamma, beta) = match case.time_integration {
            TimeIntegrationMethod::NewmarkLinear | TimeIntegrationMethod::WilsonTheta => {
                (0.5, 1.0 / 6.0)
            }
            TimeIntegrationMethod::NewmarkAverage => (0.5, 0.25),
            TimeIntegrationMethod::HilberHughesTaylor => {
                (0.5 - alpha, 0.25 * (1.0 - alpha).powi(2))
            }
        };
        let h = theta * case.time_step_dt;
        let a0 = 1.0 / (beta * h * h);
        let a1 = gamma / (beta * h);
        let a2 = 1.0 / (beta * h);
        let a3 = 1.0 / (2.0 * beta) - 1.0;
        let a4 = gamma / beta - 1.0;
        let a5 = h * (gamma / (2.0 * beta) - 1.0);
        let (alpha_m, beta_k) = case.rayleigh_damping;
        let mut c: Vec<f64> = k.iter().zip(element_damping).map(|(kij, cij)| beta_k * kij + cij).collect();
        for i in 0..n {
            c[i * n + i] += alpha_m * mass[i];
        }
        let weight = 1.0 + alpha;
        let mut effective: Vec<f64> = k
            .iter()
            .zip(&c)
            .map(|(kij, cij)| weight * (kij + a1 * cij))
            .collect();
        for i in 0..n {
            effective[i * n + i] += a0 * mass[i];
        }
        for i in 0..n {
            if fixed[i] {
                for j in 0..n {
                    effective[i * n + j] = 0.0;
                    effective[j * n + i] = 0.0;
                }
                effective[i * n + i] = 1.0;
            }
        }
        let factor = SymmetricLdlt::factorize(&effective, n)?;
        let massless_dofs: Vec<usize> = (0..n).filter(|i| !fixed[*i] && mass[*i] == 0.0).collect();
        let massless = if massless_dofs.is_empty() {
            None
        } else {
            let block: Vec<f64> = massless_dofs
                .iter()
                .flat_map(|i| massless_dofs.iter().map(move |j| k[i * n + j]))
                .collect();
            Some(MasslessBlock {
                factor: SymmetricLdlt::factorize(&block, massless_dofs.len())?,
                dofs: massless_dofs,
            })
        };
        let mut integrator = Self {
            n,
            k: k.to_vec(),
            c,
            mass: mass.to_vec(),
            fixed: fixed.to_vec(),
            factor,
            massless,
            stiffness_damping: beta_k,
            dt: case.time_step_dt,
            theta,
            alpha,
            gamma,
            a0,
            a1,
            a2,
            a3,
            a4,
            a5,
            previous_force: force.to_vec(),
            state: DynamicState {
                u: vec![0.0; n],
                v: vec![0.0; n],
                a: vec![0.0; n],
            },
        };
        integrator.initialize(force, force_slope)?;
        Ok(integrator)
    }

    fn initialize(&mut self, force: &[f64], slope: &[f64]) -> Result<(), SolverError> {
        // Massive DOFs start from rest. Massless displacements must satisfy
        // Kzz*uz = Fz(0); forcing them to zero violates initial equilibrium.
        if let Some(block) = &self.massless {
            let rhs: Vec<f64> = block.dofs.iter().map(|i| force[*i]).collect();
            let uz = block.factor.solve(&rhs)?;
            for (&i, value) in block.dofs.iter().zip(uz) {
                self.state.u[i] = value;
            }
            if self.stiffness_damping == 0.0 {
                // Differentiated algebraic constraint, using the right-hand
                // derivative of the piecewise-linear applied load at t=0.
                let rhs: Vec<f64> = block.dofs.iter().map(|i| slope[*i]).collect();
                let vz = block.factor.solve(&rhs)?;
                for (&i, value) in block.dofs.iter().zip(vz) {
                    self.state.v[i] = value;
                }
            }
        }
        for i in 0..self.n {
            if !self.fixed[i] && self.mass[i] > 0.0 {
                let resistance: f64 = (0..self.n)
                    .map(|j| {
                        self.k[i * self.n + j] * self.state.u[j]
                            + self.c[i * self.n + j] * self.state.v[j]
                    })
                    .sum();
                self.state.a[i] = (force[i] - resistance) / self.mass[i];
            }
        }
        if let Some(block) = &self.massless {
            let rhs: Vec<f64> = block
                .dofs
                .iter()
                .map(|i| {
                    let coupling: f64 = (0..self.n)
                        .filter(|j| self.mass[*j] > 0.0)
                        .map(|j| self.k[i * self.n + j] * self.state.a[j])
                        .sum();
                    if self.stiffness_damping > 0.0 {
                        slope[*i] / self.stiffness_damping - coupling
                    } else {
                        -coupling
                    }
                })
                .collect();
            let az = block.factor.solve(&rhs)?;
            for (&i, value) in block.dofs.iter().zip(az) {
                self.state.a[i] = value;
            }
        }
        self.check_state()
    }

    pub fn step(&mut self, force: &[f64], force_slope: &[f64]) -> Result<(), SolverError> {
        if force.len() != self.n
            || force_slope.len() != self.n
            || force.iter().chain(force_slope).any(|x| !x.is_finite())
        {
            return Err(SolverError::InvalidInput(
                "nonfinite or incorrectly sized dynamic load".into(),
            ));
        }
        let weight = 1.0 + self.alpha;
        let mut rhs = vec![0.0; self.n];
        for i in 0..self.n {
            if self.fixed[i] {
                continue;
            }
            let f_stage = self.previous_force[i] + self.theta * (force[i] - self.previous_force[i]);
            rhs[i] = weight * f_stage - self.alpha * self.previous_force[i]
                + self.mass[i]
                    * (self.a0 * self.state.u[i]
                        + self.a2 * self.state.v[i]
                        + self.a3 * self.state.a[i]);
            for j in 0..self.n {
                let q = self.a1 * self.state.u[j]
                    + self.a4 * self.state.v[j]
                    + self.a5 * self.state.a[j];
                rhs[i] += self.c[i * self.n + j] * (weight * q + self.alpha * self.state.v[j])
                    + self.alpha * self.k[i * self.n + j] * self.state.u[j];
            }
        }
        let stage_u = self.factor.solve(&rhs)?;
        let mut next = DynamicState {
            u: stage_u.clone(),
            v: vec![0.0; self.n],
            a: vec![0.0; self.n],
        };
        for i in 0..self.n {
            if self.fixed[i] {
                continue;
            }
            let stage_a = self.a0 * (stage_u[i] - self.state.u[i])
                - self.a2 * self.state.v[i]
                - self.a3 * self.state.a[i];
            if self.theta != 1.0 {
                next.a[i] = self.state.a[i] + (stage_a - self.state.a[i]) / self.theta;
                next.v[i] = self.state.v[i] + 0.5 * self.dt * (self.state.a[i] + next.a[i]);
                next.u[i] = self.state.u[i]
                    + self.dt * self.state.v[i]
                    + self.dt.powi(2) / 6.0 * (2.0 * self.state.a[i] + next.a[i]);
            } else {
                next.a[i] = stage_a;
                next.v[i] = self.state.v[i]
                    + self.dt * ((1.0 - self.gamma) * self.state.a[i] + self.gamma * next.a[i]);
            }
        }
        self.state = next;
        self.previous_force.copy_from_slice(force);
        // With no stiffness damping, massless coordinates are algebraic. In
        // particular, Wilson interpolation must not leave them off equilibrium.
        if self.stiffness_damping == 0.0 {
            if let Some(block) = &self.massless {
                for component in 0..3 {
                    let values = match component {
                        0 => &self.state.u,
                        1 => &self.state.v,
                        _ => &self.state.a,
                    };
                    let rhs: Vec<f64> = block
                        .dofs
                        .iter()
                        .map(|i| {
                            let load = match component {
                                0 => force[*i],
                                1 => force_slope[*i],
                                _ => 0.0,
                            };
                            load - (0..self.n)
                                .filter(|j| self.mass[*j] > 0.0)
                                .map(|j| self.k[i * self.n + j] * values[j])
                                .sum::<f64>()
                        })
                        .collect();
                    let solved = block.factor.solve(&rhs)?;
                    let values = match component {
                        0 => &mut self.state.u,
                        1 => &mut self.state.v,
                        _ => &mut self.state.a,
                    };
                    for (&i, value) in block.dofs.iter().zip(solved) {
                        values[i] = value;
                    }
                }
            }
        }
        self.check_state()
    }

    fn check_state(&self) -> Result<(), SolverError> {
        if self
            .state
            .u
            .iter()
            .chain(&self.state.v)
            .chain(&self.state.a)
            .any(|x| !x.is_finite())
        {
            return Err(SolverError::InvalidInput(
                "nonfinite dynamic state; check time step and system conditioning".into(),
            ));
        }
        Ok(())
    }
}

#[derive(Clone)]
struct DynamicLinkSpring {
    link_name: String,
    dof_index: usize,
    dofs: Vec<usize>,
    b: Vec<f64>,
    kind: sap_model::LinkPropType,
    k_nl: f64,
    param1: f64,
    param2: f64,
    f_prev: f64,
}

impl DynamicLinkSpring {
    fn deformation(&self, u: &[f64]) -> f64 {
        self.dofs.iter().zip(&self.b).map(|(i, b)| u[*i] * b).sum()
    }

    fn eval(&self, d: f64, q_link: f64, h: f64, a1: f64) -> (f64, f64) {
        match self.kind {
            sap_model::LinkPropType::Gap | sap_model::LinkPropType::Hook => {
                crate::nonlinear::law(self.kind, self.k_nl, self.param1, d)
            }
            sap_model::LinkPropType::Damper => crate::nonlinear::damper_law(
                self.k_nl,
                self.param1,
                self.param2,
                h,
                a1,
                self.f_prev,
                a1 * d - q_link,
            ),
            _ => (0.0, 0.0),
        }
    }

    fn add_matrix(&self, matrix: &mut [f64], n: usize, stiffness: f64) {
        for (i, bi) in self.dofs.iter().zip(&self.b) {
            for (j, bj) in self.dofs.iter().zip(&self.b) {
                matrix[i * n + j] += stiffness * bi * bj;
            }
        }
    }

    fn add_force(&self, force: &mut [f64], scalar: f64) {
        for (i, b) in self.dofs.iter().zip(&self.b) {
            force[*i] += b * scalar;
        }
    }
}

pub(crate) struct NonlinearStepper {
    n: usize,
    k: Vec<f64>,
    c: Vec<f64>,
    effective_linear: Vec<f64>,
    reference_factor: SymmetricLdlt,
    mass: Vec<f64>,
    fixed: Vec<bool>,
    dt: f64,
    h: f64,
    theta: f64,
    alpha: f64,
    gamma: f64,
    a0: f64,
    a1: f64,
    a2: f64,
    a3: f64,
    a4: f64,
    a5: f64,
    previous_force: Vec<f64>,
    springs: Vec<DynamicLinkSpring>,
    pub state: DynamicState,
}

impl NonlinearStepper {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        model: &sap_model::SapModel,
        points: &indexmap::IndexMap<String, sap_model::PointObj>,
        indices: &indexmap::IndexMap<String, usize>,
        rotations: &[[[f64; 3]; 3]],
        k_linear: &[f64],
        mass: &[f64],
        fixed: &[bool],
        case: &LoadCase,
        force0: &[f64],
        element_damping: &[f64],
        u_init: Option<&[f64]>,
    ) -> Result<Self, SolverError> {
        validate_parameters(case)?;
        let n = mass.len();
        let alpha = if case.time_integration == TimeIntegrationMethod::HilberHughesTaylor {
            case.hht_alpha
        } else {
            0.0
        };
        let theta = if case.time_integration == TimeIntegrationMethod::WilsonTheta {
            case.wilson_theta
        } else {
            1.0
        };
        let (gamma, beta) = match case.time_integration {
            TimeIntegrationMethod::NewmarkLinear | TimeIntegrationMethod::WilsonTheta => {
                (0.5, 1.0 / 6.0)
            }
            TimeIntegrationMethod::NewmarkAverage => (0.5, 0.25),
            TimeIntegrationMethod::HilberHughesTaylor => {
                (0.5 - alpha, 0.25 * (1.0 - alpha).powi(2))
            }
        };
        let h = theta * case.time_step_dt;
        let a0 = 1.0 / (beta * h * h);
        let a1 = gamma / (beta * h);
        let a2 = 1.0 / (beta * h);
        let a3 = 1.0 / (2.0 * beta) - 1.0;
        let a4 = gamma / beta - 1.0;
        let a5 = h * (gamma / (2.0 * beta) - 1.0);
        let (alpha_m, beta_k) = case.rayleigh_damping;
        let mut c: Vec<f64> = k_linear
            .iter()
            .zip(element_damping)
            .map(|(kij, cij)| beta_k * kij + cij)
            .collect();
        for i in 0..n {
            c[i * n + i] += alpha_m * mass[i];
        }
        let weight = 1.0 + alpha;
        let mut effective_linear: Vec<f64> = k_linear
            .iter()
            .zip(&c)
            .map(|(kij, cij)| weight * (kij + a1 * cij))
            .collect();
        for i in 0..n {
            effective_linear[i * n + i] += a0 * mass[i];
        }

        let mut u0 = vec![0.0; n];
        if let Some(init) = u_init {
            for i in 0..n {
                if !fixed[i] {
                    u0[i] = init.get(i).copied().unwrap_or(0.0);
                }
            }
        }

        let mut springs = Vec::new();
        for object in model.links.values() {
            let prop = &model.link_props[&object.prop];
            let op = crate::link::operator(object, prop, points, indices, rotations);
            for d in 0..6 {
                if prop.dof_nonlinear[d] && prop.dof_active[d] && !prop.dof_fixed[d] {
                    let mut sp = DynamicLinkSpring {
                        link_name: object.name.clone(),
                        dof_index: d,
                        dofs: op.dofs.clone(),
                        b: op.b[d].clone(),
                        kind: prop.link_type,
                        k_nl: prop.k_nl[d],
                        param1: prop.param1[d],
                        param2: prop.param2[d],
                        f_prev: 0.0,
                    };
                    if matches!(
                        sp.kind,
                        sap_model::LinkPropType::Gap | sap_model::LinkPropType::Hook
                    ) {
                        let d0 = sp.deformation(&u0);
                        sp.f_prev = crate::nonlinear::law(sp.kind, sp.k_nl, sp.param1, d0).0;
                    }
                    springs.push(sp);
                }
            }
        }

        let mut reference = effective_linear.clone();
        for sp in &springs {
            let k_ref = match sp.kind {
                sap_model::LinkPropType::Gap | sap_model::LinkPropType::Hook => sp.k_nl,
                sap_model::LinkPropType::Damper => {
                    let kh = sp.k_nl * h;
                    if kh + sp.param1 > 0.0 {
                        a1 * (kh * sp.param1) / (kh + sp.param1)
                    } else {
                        sp.k_nl
                    }
                }
                _ => sp.k_nl,
            };
            sp.add_matrix(&mut reference, n, weight * k_ref.max(1e-6));
        }
        for i in 0..n {
            if fixed[i] {
                for j in 0..n {
                    reference[i * n + j] = 0.0;
                    reference[j * n + i] = 0.0;
                }
                reference[i * n + i] = 1.0;
            }
        }
        let reference_factor = SymmetricLdlt::factorize(&reference, n)?;

        // Compute initial acceleration at massive DOFs when starting from rest at u0
        let mut a_init = vec![0.0; n];
        if u_init.is_none() {
            let mut r_nl0 = vec![0.0; n];
            for sp in &springs {
                sp.add_force(&mut r_nl0, sp.f_prev);
            }
            for i in 0..n {
                if !fixed[i] && mass[i] > 0.0 {
                    let resistance: f64 =
                        (0..n).map(|j| k_linear[i * n + j] * u0[j]).sum::<f64>() + r_nl0[i];
                    a_init[i] = (force0[i] - resistance) / mass[i];
                }
            }
        }

        Ok(Self {
            n,
            k: k_linear.to_vec(),
            c,
            effective_linear,
            reference_factor,
            mass: mass.to_vec(),
            fixed: fixed.to_vec(),
            dt: case.time_step_dt,
            h,
            theta,
            alpha,
            gamma,
            a0,
            a1,
            a2,
            a3,
            a4,
            a5,
            previous_force: force0.to_vec(),
            springs,
            state: DynamicState {
                u: u0,
                v: vec![0.0; n],
                a: a_init,
            },
        })
    }

    pub fn current_nonlinear_link_forces(&self) -> indexmap::IndexMap<(String, usize), f64> {
        let mut map = indexmap::IndexMap::new();
        for sp in &self.springs {
            map.insert((sp.link_name.clone(), sp.dof_index), sp.f_prev);
        }
        map
    }

    pub fn step(&mut self, force: &[f64], case: &LoadCase) -> Result<(), SolverError> {
        let weight = 1.0 + self.alpha;
        let mut rhs_linear = vec![0.0; self.n];
        let mut q_vec = vec![0.0; self.n];
        for j in 0..self.n {
            q_vec[j] =
                self.a1 * self.state.u[j] + self.a4 * self.state.v[j] + self.a5 * self.state.a[j];
        }
        for i in 0..self.n {
            if self.fixed[i] {
                continue;
            }
            let f_stage = self.previous_force[i] + self.theta * (force[i] - self.previous_force[i]);
            rhs_linear[i] = weight * f_stage - self.alpha * self.previous_force[i]
                + self.mass[i]
                    * (self.a0 * self.state.u[i]
                        + self.a2 * self.state.v[i]
                        + self.a3 * self.state.a[i]);
            for j in 0..self.n {
                rhs_linear[i] +=
                    self.c[i * self.n + j] * (weight * q_vec[j] + self.alpha * self.state.v[j])
                        + self.alpha * self.k[i * self.n + j] * self.state.u[j];
            }
        }
        for sp in &self.springs {
            if self.alpha != 0.0 {
                sp.add_force(&mut rhs_linear, self.alpha * sp.f_prev);
            }
        }
        for i in 0..self.n {
            if self.fixed[i] {
                rhs_linear[i] = 0.0;
            }
        }

        let q_links: Vec<f64> = self
            .springs
            .iter()
            .map(|sp| sp.deformation(&q_vec))
            .collect();

        let constrain = |mut matrix: Vec<f64>| {
            for i in 0..self.n {
                if self.fixed[i] {
                    for j in 0..self.n {
                        matrix[i * self.n + j] = 0.0;
                        matrix[j * self.n + i] = 0.0;
                    }
                    matrix[i * self.n + i] = 1.0;
                }
            }
            matrix
        };

        let eval_res_tan = |u: &[f64]| -> (Vec<f64>, Vec<f64>) {
            let mut internal: Vec<f64> = (0..self.n)
                .map(|i| {
                    if self.fixed[i] {
                        0.0
                    } else {
                        (0..self.n)
                            .map(|j| self.effective_linear[i * self.n + j] * u[j])
                            .sum()
                    }
                })
                .collect();
            let mut tangent = self.effective_linear.clone();
            for (sp, &q_link) in self.springs.iter().zip(&q_links) {
                let d = sp.deformation(u);
                let (f, k_tan) = sp.eval(d, q_link, self.h, self.a1);
                sp.add_force(&mut internal, weight * f);
                sp.add_matrix(&mut tangent, self.n, weight * k_tan);
            }
            for i in 0..self.n {
                if self.fixed[i] {
                    internal[i] = 0.0;
                }
            }
            let residual = internal
                .iter()
                .zip(&rhs_linear)
                .map(|(a, b)| a - b)
                .collect();
            (residual, tangent)
        };

        let eval_dir_deriv = |u: &[f64], p: &[f64], step_alpha: f64| -> f64 {
            let mut val = 0.0;
            for i in 0..self.n {
                if self.fixed[i] {
                    continue;
                }
                let lin_i: f64 = (0..self.n)
                    .map(|j| self.effective_linear[i * self.n + j] * (u[j] + step_alpha * p[j]))
                    .sum();
                val += p[i] * (lin_i - rhs_linear[i]);
            }
            for (sp, &q_link) in self.springs.iter().zip(&q_links) {
                let dp = sp.deformation(p);
                let d = sp.deformation(u) + step_alpha * dp;
                let (f, _) = sp.eval(d, q_link, self.h, self.a1);
                val += dp * weight * f;
            }
            val
        };

        let mut stage_u = self.state.u.clone();
        for i in 0..self.n {
            if self.fixed[i] {
                stage_u[i] = 0.0;
            }
        }
        let max_iters = case.max_iterations.max(20);
        let tol = case.convergence_tol.clamp(1e-12, 1e-4);

        for iter in 0..=max_iters {
            let (residual, tangent) = eval_res_tan(&stage_u);
            let scale = (0..self.n)
                .filter(|&i| !self.fixed[i])
                .map(|i| rhs_linear[i].abs().max((residual[i] + rhs_linear[i]).abs()))
                .fold(1e-12, f64::max);
            let norm = (0..self.n)
                .filter(|&i| !self.fixed[i])
                .map(|i| residual[i].abs())
                .fold(0.0, f64::max);
            if norm <= tol * scale {
                break;
            }
            if iter == max_iters {
                return Err(SolverError::NonConvergence {
                    case: case.name.clone(),
                    iterations: iter,
                    residual: norm,
                    reason: "nonlinear dynamic substep iteration limit reached".into(),
                });
            }
            let rhs: Vec<f64> = (0..self.n)
                .map(|i| if self.fixed[i] { 0.0 } else { -residual[i] })
                .collect();
            let mut direction = SymmetricLdlt::factorize(&constrain(tangent), self.n)
                .and_then(|f| f.solve(&rhs))
                .or_else(|_| self.reference_factor.solve(&rhs))?;
            for i in 0..self.n {
                if self.fixed[i] {
                    direction[i] = 0.0;
                }
            }
            let mut slope: f64 = residual.iter().zip(&direction).map(|(r, p)| r * p).sum();
            if !slope.is_finite() || slope >= 0.0 {
                direction = self.reference_factor.solve(&rhs)?;
                for i in 0..self.n {
                    if self.fixed[i] {
                        direction[i] = 0.0;
                    }
                }
                slope = residual.iter().zip(&direction).map(|(r, p)| r * p).sum();
                if !slope.is_finite() || slope >= 0.0 {
                    break;
                }
            }
            let mut hi = 1.0;
            for _ in 0..40 {
                if eval_dir_deriv(&stage_u, &direction, hi) >= 0.0 {
                    break;
                }
                hi *= 2.0;
            }
            let mut lo = 0.0;
            for _ in 0..40 {
                let mid = 0.5 * (lo + hi);
                if eval_dir_deriv(&stage_u, &direction, mid) < 0.0 {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            let step_alpha = 0.5 * (lo + hi);
            for i in 0..self.n {
                stage_u[i] += step_alpha * direction[i];
            }
        }

        let mut next = DynamicState {
            u: stage_u.clone(),
            v: vec![0.0; self.n],
            a: vec![0.0; self.n],
        };
        for i in 0..self.n {
            if self.fixed[i] {
                continue;
            }
            let stage_a = self.a0 * (stage_u[i] - self.state.u[i])
                - self.a2 * self.state.v[i]
                - self.a3 * self.state.a[i];
            if self.theta != 1.0 {
                next.a[i] = self.state.a[i] + (stage_a - self.state.a[i]) / self.theta;
                next.v[i] = self.state.v[i] + 0.5 * self.dt * (self.state.a[i] + next.a[i]);
                next.u[i] = self.state.u[i]
                    + self.dt * self.state.v[i]
                    + self.dt.powi(2) / 6.0 * (2.0 * self.state.a[i] + next.a[i]);
            } else {
                next.a[i] = stage_a;
                next.v[i] = self.state.v[i]
                    + self.dt * ((1.0 - self.gamma) * self.state.a[i] + self.gamma * next.a[i]);
            }
        }

        for (sp, &q_link) in self.springs.iter_mut().zip(&q_links) {
            let d = sp.deformation(&next.u);
            let (f, _) = sp.eval(d, q_link, self.h, self.a1);
            sp.f_prev = f;
        }

        self.state = next;
        self.previous_force.copy_from_slice(force);
        if self
            .state
            .u
            .iter()
            .chain(&self.state.v)
            .chain(&self.state.a)
            .any(|x| !x.is_finite())
        {
            return Err(SolverError::InvalidInput(
                "nonfinite nonlinear dynamic state".into(),
            ));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn case(method: TimeIntegrationMethod, dt: f64) -> LoadCase {
        let mut case = LoadCase::linear_static("DYNAMIC", "P", 1.0);
        case.time_integration = method;
        case.time_step_dt = dt;
        case.hht_alpha = -0.2;
        case.wilson_theta = 1.4;
        case
    }

    fn methods() -> [TimeIntegrationMethod; 4] {
        [
            TimeIntegrationMethod::NewmarkAverage,
            TimeIntegrationMethod::NewmarkLinear,
            TimeIntegrationMethod::HilberHughesTaylor,
            TimeIntegrationMethod::WilsonTheta,
        ]
    }

    // Closed-form underdamped unit-step response, derived from m*u''+c*u'+k*u=F.
    fn step_exact(m: f64, k: f64, c: f64, force: f64, t: f64) -> (f64, f64) {
        let decay = c / (2.0 * m);
        let wd = (k / m - decay * decay).sqrt();
        let u =
            force / k * (1.0 - (-decay * t).exp() * ((wd * t).cos() + decay / wd * (wd * t).sin()));
        let v = force / (m * wd) * (-decay * t).exp() * (wd * t).sin();
        (u, v)
    }

    #[test]
    fn all_integrators_converge_to_damped_step_closed_form() {
        for method in methods() {
            let mut errors = Vec::new();
            for dt in [0.02, 0.01] {
                let mut settings = case(method, dt);
                settings.rayleigh_damping = (0.2, 0.05); // c=.2*2+.05*8=.8
                let mut integrator =
                    LinearStepper::new(&[8.0], &[2.0], &[false], &settings, &[3.0], &[0.0])
                        .unwrap();
                let mut max_error = 0.0_f64;
                for step in 1..=(2.0 / dt) as usize {
                    integrator.step(&[3.0], &[0.0]).unwrap();
                    let (u, _) = step_exact(2.0, 8.0, 0.8, 3.0, step as f64 * dt);
                    max_error = max_error.max((integrator.state.u[0] - u).abs());
                }
                errors.push(max_error);
            }
            assert!(errors[0] / errors[1] > 3.5, "{method:?}: errors {errors:?}");
            assert!(
                errors[1] < 0.00012,
                "{method:?}: fine-step error {}",
                errors[1]
            );
        }
    }

    #[test]
    fn limiting_parameters_reproduce_corresponding_newmark_schemes() {
        for (method, equivalent) in [
            (
                TimeIntegrationMethod::HilberHughesTaylor,
                TimeIntegrationMethod::NewmarkAverage,
            ),
            (
                TimeIntegrationMethod::WilsonTheta,
                TimeIntegrationMethod::NewmarkLinear,
            ),
        ] {
            let mut settings = case(method, 0.03);
            settings.hht_alpha = 0.0;
            settings.wilson_theta = 1.0;
            settings.rayleigh_damping = (0.15, 0.02);
            let mut first =
                LinearStepper::new(&[8.0], &[2.0], &[false], &settings, &[2.0], &[1.0]).unwrap();
            settings.time_integration = equivalent;
            let mut second =
                LinearStepper::new(&[8.0], &[2.0], &[false], &settings, &[2.0], &[1.0]).unwrap();
            for step in 1..=100 {
                let t = step as f64 * settings.time_step_dt;
                let force = [2.0 + t.sin()];
                first.step(&force, &[t.cos()]).unwrap();
                second.step(&force, &[t.cos()]).unwrap();
                for (a, b) in first
                    .state
                    .u
                    .iter()
                    .chain(&first.state.v)
                    .chain(&first.state.a)
                    .zip(
                        second
                            .state
                            .u
                            .iter()
                            .chain(&second.state.v)
                            .chain(&second.state.a),
                    )
                {
                    assert!((a - b).abs() < 1e-12, "{method:?}: {a} != {b}");
                }
            }
        }
    }

    #[test]
    fn hht_satisfies_weighted_equilibrium_with_damping_and_changing_force() {
        let mut settings = case(TimeIntegrationMethod::HilberHughesTaylor, 0.075);
        settings.rayleigh_damping = (0.2, 0.05);
        let mut integrator =
            LinearStepper::new(&[8.0], &[2.0], &[false], &settings, &[3.0], &[1.0]).unwrap();
        let mut old_force = 3.0;
        for step in 1..=80 {
            let previous = integrator.state.clone();
            let t = step as f64 * settings.time_step_dt;
            let force = 3.0 + t.sin();
            integrator.step(&[force], &[t.cos()]).unwrap();
            let state = &integrator.state;
            let weight = 1.0 + settings.hht_alpha;
            let residual = 2.0 * state.a[0] + weight * (0.8 * state.v[0] + 8.0 * state.u[0])
                - settings.hht_alpha * (0.8 * previous.v[0] + 8.0 * previous.u[0])
                - (weight * force - settings.hht_alpha * old_force);
            assert!(residual.abs() < 1e-10, "step={step}, residual={residual}");
            old_force = force;
        }
    }

    #[test]
    fn dissipative_integrators_attenuate_high_frequency_free_vibration() {
        let mut energies = Vec::new();
        for method in [
            TimeIntegrationMethod::NewmarkAverage,
            TimeIntegrationMethod::HilberHughesTaylor,
            TimeIntegrationMethod::WilsonTheta,
        ] {
            let settings = case(method, 0.1);
            let mut integrator =
                LinearStepper::new(&[2500.0], &[1.0], &[false], &settings, &[0.0], &[0.0]).unwrap();
            integrator.state.u[0] = 1.0;
            integrator.state.a[0] = -2500.0;
            for _ in 0..100 {
                integrator.step(&[0.0], &[0.0]).unwrap();
            }
            let energy =
                0.5 * integrator.state.v[0].powi(2) + 1250.0 * integrator.state.u[0].powi(2);
            energies.push(energy);
        }
        assert!((energies[0] - 1250.0).abs() < 1e-8);
        assert!(
            energies[1] < 0.01 && energies[2] < 0.01,
            "energies={energies:?}"
        );
    }

    #[test]
    fn massless_loaded_coordinate_matches_condensed_analytical_response() {
        // K=[4,-2;-2,2], M=diag(1,0), F=[0,1+t]. Eliminating z gives
        // u''+2u=1+t and z=u+(1+t)/2, including z(0)=z'(0)=1/2.
        for method in methods() {
            let settings = case(method, 0.002);
            let mut integrator = LinearStepper::new(
                &[4.0, -2.0, -2.0, 2.0],
                &[1.0, 0.0],
                &[false; 2],
                &settings,
                &[0.0, 1.0],
                &[0.0, 1.0],
            )
            .unwrap();
            assert_eq!(integrator.state.u, vec![0.0, 0.5]);
            assert_eq!(integrator.state.v, vec![0.0, 0.5]);
            assert_eq!(integrator.state.a, vec![1.0, 1.0]);
            let w = 2.0_f64.sqrt();
            for step in 1..=500 {
                let t = step as f64 * settings.time_step_dt;
                integrator.step(&[0.0, 1.0 + t], &[0.0, 1.0]).unwrap();
                let exact = (1.0 - (w * t).cos() + t - (w * t).sin() / w) / 2.0;
                assert!(
                    (integrator.state.u[0] - exact).abs() < 3e-6,
                    "{method:?}, t={t}"
                );
                assert!(
                    (integrator.state.u[1] - integrator.state.u[0] - (1.0 + t) / 2.0).abs() < 1e-12
                );
            }
        }
    }

    #[test]
    fn massless_initialization_and_stiffness_damping_preserve_constant_offset() {
        for method in methods() {
            let mut settings = case(method, 0.002);
            settings.rayleigh_damping = (0.1, 0.2);
            let mut integrator = LinearStepper::new(
                &[4.0, -2.0, -2.0, 2.0],
                &[1.0, 0.0],
                &[false; 2],
                &settings,
                &[0.0, 1.0],
                &[0.0, 0.0],
            )
            .unwrap();
            for step in 1..=500 {
                integrator.step(&[0.0, 1.0], &[0.0, 0.0]).unwrap();
                let t = step as f64 * settings.time_step_dt;
                let (exact, _) = step_exact(1.0, 2.0, 0.5, 1.0, t);
                assert!(
                    (integrator.state.u[0] - exact).abs() < 3e-6,
                    "{method:?}, t={t}"
                );
                assert!((integrator.state.u[1] - integrator.state.u[0] - 0.5).abs() < 1e-9);
            }
        }
    }

    #[test]
    fn massless_stiffness_damping_matches_first_order_analytical_constraint() {
        // With Fz=1+t and betaK=.2, q=z-u satisfies .2*q'+q=(1+t)/2.
        // The massive coordinate satisfies u''+.5*u'+2*u=1+t.
        for method in methods() {
            let mut settings = case(method, 0.001);
            settings.rayleigh_damping = (0.1, 0.2);
            let mut integrator = LinearStepper::new(
                &[4.0, -2.0, -2.0, 2.0],
                &[1.0, 0.0],
                &[false; 2],
                &settings,
                &[0.0, 1.0],
                &[0.0, 1.0],
            )
            .unwrap();
            let decay = 0.25;
            let wd = (2.0_f64 - decay * decay).sqrt();
            let aa = -(0.5 - 0.5 / 4.0);
            let bb = (decay * aa - 0.5) / wd;
            for step in 1..=1000 {
                let t = step as f64 * settings.time_step_dt;
                integrator.step(&[0.0, 1.0 + t], &[0.0, 1.0]).unwrap();
                let exact_u = 0.5 * t + 0.5 - 0.5 / 4.0
                    + (-decay * t).exp() * (aa * (wd * t).cos() + bb * (wd * t).sin());
                let exact_q = (1.0 + t - 0.2 + 0.2 * (-t / 0.2).exp()) / 2.0;
                assert!(
                    (integrator.state.u[0] - exact_u).abs() < 2e-6,
                    "{method:?}: u at {t}"
                );
                assert!(
                    (integrator.state.u[1] - exact_u - exact_q).abs() < 2e-6,
                    "{method:?}: z at {t}: {} versus {}",
                    integrator.state.u[1],
                    exact_u + exact_q
                );
            }
        }
    }

    #[test]
    fn invalid_time_integration_parameters_are_not_clamped() {
        for (method, value) in [
            (TimeIntegrationMethod::HilberHughesTaylor, -0.34),
            (TimeIntegrationMethod::HilberHughesTaylor, 0.1),
            (TimeIntegrationMethod::HilberHughesTaylor, f64::NAN),
            (TimeIntegrationMethod::WilsonTheta, 0.99),
            (TimeIntegrationMethod::WilsonTheta, f64::INFINITY),
        ] {
            let mut settings = case(method, 0.01);
            settings.hht_alpha = value;
            settings.wilson_theta = value;
            assert!(
                LinearStepper::new(&[1.0], &[1.0], &[false], &settings, &[0.0], &[0.0]).is_err()
            );
        }
    }
}
