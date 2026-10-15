//! Convex small-displacement Gap/Hook equilibrium. No effective-stiffness
//! approximation is used for a DOF explicitly marked nonlinear.
use crate::{linalg::SymmetricLdlt, link, SolverError};
use indexmap::IndexMap;
use sap_model::{LinkPropType, LoadCase, PointObj, SapModel};

/// CSI manual pp.286-287. The tangent at the opening is chosen as zero; the
/// reference-stiffness descent direction handles an initially open mechanism.
pub(crate) fn law(kind: LinkPropType, k: f64, opening: f64, displacement: f64) -> (f64, f64) {
    let elastic = match kind {
        LinkPropType::Gap => (displacement + opening).min(0.),
        LinkPropType::Hook => (displacement - opening).max(0.),
        LinkPropType::Damper => 0.,
        _ => unreachable!("validated nonlinear spring kind"),
    };
    (k * elastic, if elastic != 0. { k } else { 0. })
}

/// Exponential Maxwell damper constitutive update (CSI Reference Manual pp. 279-280):
/// series spring `k_nl` in series with dashpot `f = c_nl * |v_c|^c_exp * sgn(v_c)`.
/// Over time step `dt` with Newmark velocity-displacement derivative `a1 = dv/du`,
/// returns `(f_next, df_next_du)`.
pub(crate) fn damper_law(
    k_nl: f64,
    c_nl: f64,
    c_exp: f64,
    dt: f64,
    a1: f64,
    f_prev: f64,
    v_link: f64,
) -> (f64, f64) {
    if k_nl <= 0.0 || c_nl <= 0.0 || dt <= 0.0 {
        return (0.0, 0.0);
    }
    let kh = k_nl * dt;
    let rhs_signed = f_prev + kh * v_link;
    if rhs_signed.abs() <= 1e-30 {
        return (0.0, 0.0);
    }
    let e = if c_exp > 0.0 { c_exp } else { 1.0 };
    if (e - 1.0).abs() <= 1e-12 {
        let vc = rhs_signed / (c_nl + kh);
        let f_next = c_nl * vc;
        let k_tan = a1 * (kh * c_nl) / (c_nl + kh);
        return (f_next, k_tan);
    }
    let sgn = rhs_signed.signum();
    let r = rhs_signed.abs();
    let mut lo = 0.0;
    let mut hi = r / kh;
    let mut w = (r / c_nl).powf(1.0 / e).min(0.5 * hi);
    for _ in 0..32 {
        let pow_e = w.powf(e);
        let phi = c_nl * pow_e + kh * w - r;
        if phi.abs() <= 1e-14 * r {
            break;
        }
        if phi < 0.0 {
            lo = w;
        } else {
            hi = w;
        }
        let dphi = if w > 1e-30 {
            c_nl * e * w.powf(e - 1.0) + kh
        } else {
            kh
        };
        let next = w - phi / dphi;
        w = if next > lo && next < hi {
            next
        } else {
            0.5 * (lo + hi)
        };
    }
    let f_next = sgn * (r - kh * w);
    let k_tan = if w <= 1e-30 && e < 1.0 {
        a1 * kh
    } else {
        let ce_w = c_nl * e * w.powf(e - 1.0);
        a1 * (kh * ce_w) / (ce_w + kh)
    };
    (f_next, k_tan)
}

#[derive(Clone)]
enum SpringLaw {
    Contact {
        kind: LinkPropType,
        stiffness: f64,
        opening: f64,
    },
    FrameTc {
        ea_over_l: f64,
        p0_axial: f64,
        limits: crate::cable::FrameTcLimits,
    },
}

struct Spring {
    dofs: Vec<usize>,
    b: Vec<f64>,
    law: SpringLaw,
    stiffness: f64,
    effective: f64,
}

impl Spring {
    fn deformation(&self, u: &[f64]) -> f64 {
        self.dofs.iter().zip(&self.b).map(|(i, b)| u[*i] * b).sum()
    }
    fn evaluate(&self, d: f64) -> (f64, f64) {
        match &self.law {
            SpringLaw::Contact {
                kind,
                stiffness,
                opening,
            } => law(*kind, *stiffness, *opening, d),
            SpringLaw::FrameTc {
                ea_over_l,
                p0_axial,
                limits,
            } => {
                let (p_clamped, k_ax, _) =
                    crate::cable::eval_frame_tc_axial(*ea_over_l, d, *p0_axial, *limits, 0.0);
                // Subtract p0_axial because -f_fef already contributes p0_axial to the RHS
                (p_clamped - *p0_axial, k_ax)
            }
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

pub(crate) struct System {
    n: usize,
    linear: Vec<f64>,
    reference: Vec<f64>,
    springs: Vec<Spring>,
}

impl System {
    pub fn new_with_frame_p0(
        model: &SapModel,
        points: &IndexMap<String, PointObj>,
        indices: &IndexMap<String, usize>,
        rotations: &[[[f64; 3]; 3]],
        k: &[f64],
        frame_p0: &IndexMap<String, f64>,
        n: usize,
    ) -> Self {
        let mut linear = k.to_vec();
        let mut springs = Vec::new();
        for object in model.links.values() {
            let prop = &model.link_props[&object.prop];
            let op = link::operator(object, prop, points, indices, rotations);
            for d in 0..6 {
                if prop.dof_nonlinear[d] && prop.dof_active[d] && !prop.dof_fixed[d] {
                    let spring = Spring {
                        dofs: op.dofs.clone(),
                        b: op.b[d].clone(),
                        law: SpringLaw::Contact {
                            kind: prop.link_type,
                            stiffness: prop.k_nl[d],
                            opening: prop.param1[d],
                        },
                        stiffness: prop.k_nl[d],
                        effective: prop.ke[d],
                    };
                    spring.add_matrix(&mut linear, n, -spring.effective);
                    springs.push(spring);
                }
            }
        }
        for (fname, frame) in &model.frames {
            if let Some((has_comp, comp_lim, has_tens, tens_lim)) = frame.tc_limits {
                if has_comp || has_tens {
                    let idx_i = indices[&frame.point_i];
                    let idx_j = indices[&frame.point_j];
                    let pt_i = &points[idx_i];
                    let pt_j = &points[idx_j];
                    let sec = &model.frame_sections[&frame.section];
                    let mat = &model.materials[&sec.material];
                    let (length, e_axes) = crate::frame::frame_local_axes(
                        [pt_i.x, pt_i.y, pt_i.z],
                        [pt_j.x, pt_j.y, pt_j.z],
                        frame.angle_deg,
                    );
                    let rz = frame.rigid_zone_factor.clamp(0.0, 1.0);
                    let lf = (length - rz * (frame.end_offset_i + frame.end_offset_j))
                        .max(1e-6 * length);
                    let k_loc =
                        crate::frame::compute_flexible_local_stiffness(model, frame, sec, mat, lf);
                    let ea_over_l = k_loc[0][0];
                    let ti = crate::frame::mat3_mul(&e_axes, &rotations[idx_i]);
                    let tj = crate::frame::mat3_mul(&e_axes, &rotations[idx_j]);
                    let dofs = vec![
                        6 * idx_i,
                        6 * idx_i + 1,
                        6 * idx_i + 2,
                        6 * idx_j,
                        6 * idx_j + 1,
                        6 * idx_j + 2,
                    ];
                    let b = vec![
                        -ti[0][0], -ti[0][1], -ti[0][2], tj[0][0], tj[0][1], tj[0][2],
                    ];
                    let p0_axial = frame_p0.get(fname).copied().unwrap_or(0.0);
                    let spring = Spring {
                        dofs,
                        b,
                        law: SpringLaw::FrameTc {
                            ea_over_l,
                            p0_axial,
                            limits: crate::cable::FrameTcLimits {
                                tension_limit: has_tens.then_some(tens_lim),
                                compression_limit: has_comp.then_some(comp_lim),
                            },
                        },
                        stiffness: ea_over_l,
                        effective: ea_over_l,
                    };
                    spring.add_matrix(&mut linear, n, -spring.effective);
                    springs.push(spring);
                }
            }
        }
        let mut reference = linear.clone();
        for spring in &springs {
            spring.add_matrix(&mut reference, n, spring.stiffness);
        }
        Self {
            n,
            linear,
            reference,
            springs,
        }
    }

    pub fn correction(&self, u: &[f64]) -> Vec<f64> {
        let mut correction = vec![0.; self.n];
        for spring in &self.springs {
            let d = spring.deformation(u);
            let (force, _) = spring.evaluate(d);
            spring.add_force(&mut correction, force - spring.effective * d);
        }
        correction
    }

    fn residual_tangent(&self, u: &[f64], load: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let mut internal: Vec<f64> = (0..self.n)
            .map(|i| {
                (0..self.n)
                    .map(|j| self.linear[i * self.n + j] * u[j])
                    .sum()
            })
            .collect();
        let mut tangent = self.linear.clone();
        for spring in &self.springs {
            let (f, k) = spring.evaluate(spring.deformation(u));
            spring.add_force(&mut internal, f);
            spring.add_matrix(&mut tangent, self.n, k);
        }
        let residual = internal.iter().zip(load).map(|(a, b)| a - b).collect();
        (residual, tangent)
    }

    fn directional_derivative(&self, u: &[f64], p: &[f64], load: &[f64], alpha: f64) -> f64 {
        let mut value = 0.;
        for i in 0..self.n {
            value += p[i]
                * ((0..self.n)
                    .map(|j| self.linear[i * self.n + j] * (u[j] + alpha * p[j]))
                    .sum::<f64>()
                    - load[i]);
        }
        for spring in &self.springs {
            let dp = spring.deformation(p);
            let (f, _) = spring.evaluate(spring.deformation(u) + alpha * dp);
            value += dp * f;
        }
        value
    }

    pub fn solve_from(
        &self,
        load: &[f64],
        fixed: &[bool],
        prescribed: &[f64],
        u_init: Option<&[f64]>,
        case: &LoadCase,
    ) -> Result<Vec<f64>, SolverError> {
        if load.iter().chain(prescribed).any(|x| !x.is_finite()) {
            return Err(SolverError::InvalidInput(
                "nonlinear loads and prescribed displacements must be finite".into(),
            ));
        }
        let constrain = |mut matrix: Vec<f64>| {
            for i in 0..self.n {
                if fixed[i] {
                    for j in 0..self.n {
                        matrix[i * self.n + j] = 0.;
                        matrix[j * self.n + i] = 0.;
                    }
                    matrix[i * self.n + i] = 1.;
                }
            }
            matrix
        };
        let reference = SymmetricLdlt::factorize(&constrain(self.reference.clone()), self.n)?;
        let mut u: Vec<f64> = (0..self.n)
            .map(|i| {
                if fixed[i] {
                    prescribed[i]
                } else {
                    u_init.and_then(|init| init.get(i).copied()).unwrap_or(0.)
                }
            })
            .collect();
        let fail = |iterations, residual, reason: &str| SolverError::NonConvergence {
            case: case.name.clone(),
            iterations,
            residual,
            reason: reason.into(),
        };
        for iteration in 0..=case.max_iterations {
            let (residual, tangent) = self.residual_tangent(&u, load);
            // Restrained loads and reactions must not loosen free equilibrium.
            let scale = (0..self.n)
                .filter(|i| !fixed[*i])
                .map(|i| load[i].abs().max((residual[i] + load[i]).abs()))
                .fold(0., f64::max);
            let norm = (0..self.n)
                .filter(|i| !fixed[*i])
                .map(|i| residual[i].abs())
                .fold(0., f64::max);
            if !norm.is_finite()
                || residual.iter().any(|x| !x.is_finite())
                || u.iter().any(|x| !x.is_finite())
            {
                return Err(fail(iteration, norm, "nonfinite nonlinear state"));
            }
            if norm <= case.convergence_tol * scale {
                return Ok(u);
            }
            if iteration == case.max_iterations {
                return Err(fail(iteration, norm, "iteration limit reached"));
            }
            let rhs: Vec<f64> = (0..self.n)
                .map(|i| if fixed[i] { 0. } else { -residual[i] })
                .collect();
            let mut direction = SymmetricLdlt::factorize(&constrain(tangent), self.n)
                .and_then(|factor| factor.solve(&rhs))
                .or_else(|_| reference.solve(&rhs))?;
            let mut slope: f64 = residual.iter().zip(&direction).map(|(r, p)| r * p).sum();
            if !slope.is_finite() || slope >= 0. {
                direction = reference.solve(&rhs)?;
                slope = residual.iter().zip(&direction).map(|(r, p)| r * p).sum();
                if !slope.is_finite() || slope >= 0. {
                    return Err(fail(iteration, norm, "no descent direction"));
                }
            }
            // Exact one-dimensional convex minimization (to numerical precision)
            // crosses an initially open gap without fake stabilization forces.
            let mut hi = 1.;
            let mut bracketed = false;
            for _ in 0..80 {
                let derivative = self.directional_derivative(&u, &direction, load, hi);
                if !derivative.is_finite() {
                    return Err(fail(iteration, norm, "nonfinite line search"));
                }
                if derivative >= 0. {
                    bracketed = true;
                    break;
                }
                hi *= 2.;
            }
            if !bracketed {
                return Err(fail(
                    iteration,
                    norm,
                    "unbounded load direction or no reachable resisting contact",
                ));
            }
            let mut lo = 0.;
            for _ in 0..80 {
                let mid = 0.5 * (lo + hi);
                if self.directional_derivative(&u, &direction, load, mid) < 0. {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            let alpha = 0.5 * (lo + hi);
            for i in 0..self.n {
                u[i] += alpha * direction[i];
            }
        }
        unreachable!()
    }
}
