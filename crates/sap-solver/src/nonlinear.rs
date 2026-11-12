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
        _ => unreachable!("path-dependent and unsupported link laws never reach the elastic law"),
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

/// Virgin loading curve of the CSI Wen plasticity property (Analysis Reference
/// Manual, "Wen Plasticity Property"): `dW/dX = 1 - W^exp`, `W(0) = 0`, with
/// `X = k d / yield`. Exponents 1 and 2 use their closed forms; other exponents
/// use a tabulated RK4 solution with cubic Hermite interpolation. Advancing the
/// state is `W(W^-1(w0) + x)`, so splitting one monotonic deformation increment
/// into several gives the same state to interpolation precision.
#[derive(Clone, Debug)]
pub(crate) struct WenCurve {
    exponent: f64,
    table: Option<std::sync::Arc<Vec<f64>>>,
    x_sat: f64,
}

const WEN_TABLE_INTERVALS: usize = 20_000;

impl WenCurve {
    pub fn new(exponent: f64) -> Self {
        if exponent == 1.0 || exponent == 2.0 {
            Self {
                exponent,
                table: None,
                x_sat: f64::INFINITY,
            }
        } else {
            Self::tabulated(exponent)
        }
    }

    /// Always uses the numerical table, including for exponents with a closed form.
    pub fn tabulated(exponent: f64) -> Self {
        // Near the yield surface 1-W decays at rate `exponent`; 40 e-foldings
        // are below f64 resolution. The additive 3 covers the initial rise.
        let x_sat = 3.0 + 40.0 / exponent;
        let h = x_sat / WEN_TABLE_INTERVALS as f64;
        let rate = |w: f64| 1.0 - w.clamp(0.0, 1.0).powf(exponent);
        let mut table = Vec::with_capacity(WEN_TABLE_INTERVALS + 1);
        let mut w = 0.0f64;
        table.push(w);
        for _ in 0..WEN_TABLE_INTERVALS {
            for _ in 0..2 {
                let hh = 0.5 * h;
                let k1 = rate(w);
                let k2 = rate(w + 0.5 * hh * k1);
                let k3 = rate(w + 0.5 * hh * k2);
                let k4 = rate(w + hh * k3);
                w = (w + hh / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)).min(1.0);
            }
            table.push(w);
        }
        Self {
            exponent,
            table: Some(std::sync::Arc::new(table)),
            x_sat,
        }
    }

    fn hermite(&self, table: &[f64], i: usize, t: f64) -> f64 {
        let h = self.x_sat / WEN_TABLE_INTERVALS as f64;
        let (w0, w1) = (table[i], table[i + 1]);
        let (m0, m1) = (
            h * (1.0 - w0.powf(self.exponent)),
            h * (1.0 - w1.powf(self.exponent)),
        );
        let t2 = t * t;
        let t3 = t2 * t;
        ((2.0 * t3 - 3.0 * t2 + 1.0) * w0
            + (t3 - 2.0 * t2 + t) * m0
            + (-2.0 * t3 + 3.0 * t2) * w1
            + (t3 - t2) * m1)
            .clamp(w0, w1)
    }

    fn value(&self, x: f64) -> f64 {
        let Some(table) = &self.table else {
            unreachable!("closed-form Wen exponents do not use the table")
        };
        if x <= 0.0 {
            return 0.0;
        }
        if x >= self.x_sat {
            return 1.0;
        }
        let s = x / self.x_sat * WEN_TABLE_INTERVALS as f64;
        let i = (s.floor() as usize).min(WEN_TABLE_INTERVALS - 1);
        self.hermite(table, i, s - i as f64)
    }

    fn inverse(&self, w: f64) -> f64 {
        let Some(table) = &self.table else {
            unreachable!("closed-form Wen exponents do not use the table")
        };
        if w <= 0.0 {
            return 0.0;
        }
        if w >= table[WEN_TABLE_INTERVALS] {
            return self.x_sat;
        }
        let i = table.partition_point(|v| *v <= w).saturating_sub(1);
        let i = i.min(WEN_TABLE_INTERVALS - 1);
        let (mut lo, mut hi) = (0.0f64, 1.0f64);
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if self.hermite(table, i, mid) < w {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        (i as f64 + 0.5 * (lo + hi)) * self.x_sat / WEN_TABLE_INTERVALS as f64
    }

    /// State `w` after a normalized loading increment `x >= 0` from `w0` in `[0, 1]`.
    pub fn advance(&self, w0: f64, x: f64) -> f64 {
        let w0 = w0.clamp(0.0, 1.0);
        if x <= 0.0 {
            return w0;
        }
        let w = if self.table.is_none() && self.exponent == 1.0 {
            1.0 - (1.0 - w0) * (-x).exp()
        } else if self.table.is_none() {
            let t = x.tanh();
            (w0 + t) / (1.0 + w0 * t)
        } else {
            let x0 = self.inverse(w0);
            if x0 >= self.x_sat {
                1.0
            } else {
                self.value(x0 + x)
            }
        };
        w.clamp(w0, 1.0)
    }

    fn loading_rate(&self, w: f64) -> f64 {
        (1.0 - w.clamp(0.0, 1.0).powf(self.exponent)).max(0.0)
    }
}

/// Uniaxial Wen plasticity for one link deformation (CSI Analysis Reference
/// Manual, "Wen Plasticity Property"; Software Verification Example 6-008):
/// `f = ratio k d + (1 - ratio) yield z`, with
/// `dz/dd = (k/yield) (1 - |z|^exp)` while `dd z > 0` and `k/yield` otherwise.
#[derive(Clone, Debug)]
pub(crate) struct WenLaw {
    pub k: f64,
    pub yield_force: f64,
    pub ratio: f64,
    curve: WenCurve,
}

impl WenLaw {
    pub fn new(k: f64, yield_force: f64, ratio: f64, exponent: f64) -> Self {
        Self {
            k,
            yield_force,
            ratio,
            curve: WenCurve::new(exponent),
        }
    }

    /// `(z, dz/dd)` after a monotonic deformation change from `(d_ref, z_ref)` to `d`.
    pub fn advance(&self, d_ref: f64, z_ref: f64, d: f64) -> (f64, f64) {
        let c = self.k / self.yield_force;
        let delta = d - d_ref;
        if delta == 0.0 {
            return (z_ref, c);
        }
        let s = delta.signum();
        let mut w = s * z_ref;
        let mut x = c * delta.abs();
        if w < 0.0 {
            // Elastic unloading back to z = 0 before yielding in the new direction.
            if x <= -w {
                return (s * (w + x), c);
            }
            x += w;
            w = 0.0;
        }
        let w = self.curve.advance(w, x);
        (s * w, c * self.curve.loading_rate(w))
    }

    pub fn force(&self, d: f64, z: f64) -> f64 {
        self.ratio * self.k * d + (1.0 - self.ratio) * self.yield_force * z
    }

    /// `(force, tangent)` at `d` for a monotonic change from `(d_ref, z_ref)`.
    pub fn eval(&self, d_ref: f64, z_ref: f64, d: f64) -> (f64, f64) {
        let (z, dz) = self.advance(d_ref, z_ref, d);
        (
            self.force(d, z),
            self.ratio * self.k + (1.0 - self.ratio) * self.yield_force * dz,
        )
    }
}

/// Checked Wen parameters for one nonlinear link DOF.
pub(crate) fn wen_law_for(
    prop: &sap_model::LinkProp,
    d: usize,
    link: &str,
) -> Result<WenLaw, SolverError> {
    let (k, y, r, e) = (prop.k_nl[d], prop.param1[d], prop.param2[d], prop.param3[d]);
    if !(k.is_finite() && k > 0.0)
        || !(y.is_finite() && y > 0.0)
        || !(r.is_finite() && (0.0..=1.0).contains(&r))
        || !(e.is_finite() && (1.0..=20.0).contains(&e))
    {
        return Err(SolverError::InvalidInput(format!(
            "link '{link}' plastic Wen DOF {} needs stiffness > 0, yield > 0, post-yield ratio in [0, 1] and yield exponent in [1, 20]",
            d + 1
        )));
    }
    Ok(WenLaw::new(k, y, r, e))
}

#[derive(Clone)]
enum SpringLaw {
    Wen {
        law: WenLaw,
        d_ref: f64,
        z_ref: f64,
    },
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
    /// Link name and DOF for link springs; `None` for frame member limits.
    tag: Option<(String, usize)>,
}

impl Spring {
    fn deformation(&self, u: &[f64]) -> f64 {
        self.dofs.iter().zip(&self.b).map(|(i, b)| u[*i] * b).sum()
    }
    fn evaluate(&self, d: f64) -> (f64, f64) {
        match &self.law {
            SpringLaw::Wen { law, d_ref, z_ref } => law.eval(*d_ref, *z_ref, d),
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

/// Assembled loads of one nonlinear static case, split into the state inherited
/// from its initial-case chain and the increment the case itself applies.
pub(crate) struct StageLoads<'a> {
    pub rhs_prev: &'a [f64],
    pub rhs_own: &'a [f64],
    pub u_prev: &'a [f64],
    pub u_own: &'a [f64],
    pub fixed: &'a [bool],
}

pub(crate) struct System {
    n: usize,
    linear: Vec<f64>,
    reference: Vec<f64>,
    springs: Vec<Spring>,
}

impl System {
    #[allow(clippy::too_many_arguments)]
    pub fn new_with_frame_p0(
        model: &SapModel,
        points: &IndexMap<String, PointObj>,
        indices: &IndexMap<String, usize>,
        rotations: &[[[f64; 3]; 3]],
        k: &[f64],
        frame_p0: &IndexMap<String, f64>,
        hysteresis: &[sap_model::LinkHysteresisState],
        n: usize,
    ) -> Result<Self, SolverError> {
        let mut linear = k.to_vec();
        let mut springs = Vec::new();
        for object in model.links.values() {
            let prop = &model.link_props[&object.prop];
            let op = link::operator(object, prop, points, indices, rotations);
            for d in 0..6 {
                if prop.dof_nonlinear[d] && prop.dof_active[d] && !prop.dof_fixed[d] {
                    let law = if prop.link_type == LinkPropType::PlasticWen {
                        let (d_ref, z_ref) = hysteresis
                            .iter()
                            .find(|state| state.link == object.name && state.dof == d)
                            .map_or((0.0, 0.0), |state| (state.deformation, state.z));
                        if !d_ref.is_finite() || !z_ref.is_finite() || z_ref.abs() > 1.0 {
                            return Err(SolverError::InvalidInput(format!(
                                "link '{}' has an invalid persisted hysteretic state",
                                object.name
                            )));
                        }
                        SpringLaw::Wen {
                            law: wen_law_for(prop, d, &object.name)?,
                            d_ref,
                            z_ref,
                        }
                    } else {
                        SpringLaw::Contact {
                            kind: prop.link_type,
                            stiffness: prop.k_nl[d],
                            opening: prop.param1[d],
                        }
                    };
                    let spring = Spring {
                        dofs: op.dofs.clone(),
                        b: op.b[d].clone(),
                        law,
                        stiffness: prop.k_nl[d],
                        effective: prop.ke[d],
                        tag: Some((object.name.clone(), d)),
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
                        tag: None,
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
        Ok(Self {
            n,
            linear,
            reference,
            springs,
        })
    }

    fn hysteretic(&self) -> impl Iterator<Item = (&Spring, &WenLaw, f64, f64, &(String, usize))> {
        self.springs.iter().filter_map(|spring| match (&spring.law, &spring.tag) {
            (SpringLaw::Wen { law, d_ref, z_ref }, Some(tag)) => {
                Some((spring, law, *d_ref, *z_ref, tag))
            }
            _ => None,
        })
    }

    /// Path-dependent link forces at `u`, keyed by link name and DOF.
    pub fn hysteretic_forces(&self, u: &[f64]) -> IndexMap<(String, usize), f64> {
        self.hysteretic()
            .map(|(spring, law, d_ref, z_ref, tag)| {
                (tag.clone(), law.eval(d_ref, z_ref, spring.deformation(u)).0)
            })
            .collect()
    }

    /// Hysteretic state reached at `u`, for a following case.
    pub fn hysteretic_states(&self, u: &[f64]) -> Vec<sap_model::LinkHysteresisState> {
        self.hysteretic()
            .map(|(spring, law, d_ref, z_ref, tag)| {
                let deformation = spring.deformation(u);
                sap_model::LinkHysteresisState {
                    link: tag.0.clone(),
                    dof: tag.1,
                    deformation,
                    z: law.advance(d_ref, z_ref, deformation).0,
                }
            })
            .collect()
    }

    /// Follows the proportional load path `prev + factor * own` of one nonlinear
    /// static case and returns the final load factor: 1 under load control, or
    /// the factor at which the monitored displacement has moved by the target.
    ///
    /// The Wen state update assumes each hysteretic deformation changes
    /// monotonically inside a case. That is checked on `PATH_CHECKS` equal load
    /// increments; a reversal is rejected instead of being integrated with an
    /// unverified load-path resolution. Displacement control additionally
    /// requires the monitored displacement to increase with the load factor.
    pub fn load_path_factor(
        &self,
        stage: &StageLoads<'_>,
        u_init: Option<&[f64]>,
        case: &LoadCase,
        monitor: Option<(usize, f64)>,
    ) -> Result<f64, SolverError> {
        const PATH_CHECKS: usize = 8;
        let solve = |factor: f64, guess: &[f64]| -> Result<Vec<f64>, SolverError> {
            let load: Vec<f64> = (0..self.n)
                .map(|i| stage.rhs_prev[i] + factor * stage.rhs_own[i])
                .collect();
            let prescribed: Vec<f64> = (0..self.n)
                .map(|i| stage.u_prev[i] + factor * stage.u_own[i])
                .collect();
            self.solve_from(&load, stage.fixed, &prescribed, Some(guess), case)
        };
        let zero = vec![0.0; self.n];
        let start = solve(0.0, u_init.unwrap_or(&zero))?;
        let (factor, monitor_sign) = if let Some((dof, target)) = monitor {
            if !target.is_finite() || target <= 0.0 {
                return Err(SolverError::InvalidInput(format!(
                    "case '{}' displacement-control target must be finite and positive",
                    case.name
                )));
            }
            if stage.fixed[dof] {
                return Err(SolverError::InvalidInput(format!(
                    "case '{}' monitors a restrained or inactive displacement",
                    case.name
                )));
            }
            let unit = solve(1.0, &start)?;
            let first = unit[dof] - start[dof];
            let scale = unit.iter().fold(0.0f64, |m, x| m.max(x.abs()));
            if !first.is_finite() || first.abs() <= 1e-12 * scale || first == 0.0 {
                return Err(SolverError::InvalidInput(format!(
                    "case '{}' load pattern does not move the monitored displacement",
                    case.name
                )));
            }
            let sign = first.signum();
            let moved = |factor: f64, guess: &[f64]| -> Result<(f64, Vec<f64>), SolverError> {
                let u = solve(factor, guess)?;
                Ok((sign * (u[dof] - start[dof]) - target, u))
            };
            let unreachable = |iterations: usize, residual: f64| SolverError::NonConvergence {
                case: case.name.clone(),
                iterations,
                residual,
                reason: "displacement-control target is not reachable by scaling the load".into(),
            };
            let (mut lo, mut g_lo) = (0.0f64, -target);
            let (mut hi, mut g_hi, mut guess) = (1.0f64, sign * first - target, unit);
            // A factor with no equilibrium (for example above a perfectly plastic
            // capacity) bounds the search from above without being a solution.
            let mut no_equilibrium: Option<f64> = None;
            let mut searches = 0;
            while g_hi < 0.0 {
                searches += 1;
                if searches > 200 {
                    return Err(unreachable(searches, -g_hi));
                }
                (lo, g_lo) = (hi, g_hi);
                let trial = match no_equilibrium {
                    Some(limit) => {
                        if limit - lo <= 4.0 * f64::EPSILON * limit {
                            return Err(unreachable(searches, -g_lo));
                        }
                        0.5 * (lo + limit)
                    }
                    None => 2.0 * lo,
                };
                match moved(trial, &guess) {
                    Ok((g, u)) => {
                        (hi, g_hi, guess) = (trial, g, u);
                    }
                    Err(SolverError::NonConvergence { .. }) => {
                        no_equilibrium = Some(trial);
                        // `lo` stays the last solved factor; search below `trial`.
                        (hi, g_hi) = (lo, g_lo);
                    }
                    Err(other) => return Err(other),
                }
            }
            // Illinois regula falsi inside a maintained bracket.
            let mut side = 0i32;
            for _ in 0..200 {
                if g_hi.abs() <= 1e-13 * target || (hi - lo) <= 4.0 * f64::EPSILON * hi {
                    break;
                }
                let mut mid = hi - g_hi * (hi - lo) / (g_hi - g_lo);
                if !(mid > lo && mid < hi) {
                    mid = 0.5 * (lo + hi);
                }
                let (g_mid, u_mid) = moved(mid, &guess)?;
                if g_mid < 0.0 {
                    (lo, g_lo) = (mid, g_mid);
                    if side == -1 {
                        g_hi *= 0.5;
                    }
                    side = -1;
                } else {
                    (hi, g_hi) = (mid, g_mid);
                    guess = u_mid;
                    if side == 1 {
                        g_lo *= 0.5;
                    }
                    side = 1;
                }
            }
            // `hi` is the smallest bracketed factor that reaches the target.
            (hi, Some(sign))
        } else {
            (1.0, None)
        };

        let mut previous = start.clone();
        let mut directions: Vec<f64> = vec![0.0; self.springs.len()];
        let mut monitor_previous = 0.0f64;
        for step in 1..=PATH_CHECKS {
            let u = solve(factor * step as f64 / PATH_CHECKS as f64, &previous)?;
            for (index, spring) in self.springs.iter().enumerate() {
                let (SpringLaw::Wen { law, .. }, Some(tag)) = (&spring.law, &spring.tag) else {
                    continue;
                };
                let change = spring.deformation(&u) - spring.deformation(&previous);
                // Deformation noise from the equilibrium tolerance is not a reversal.
                let noise = 1e-9 * law.yield_force / law.k;
                if change.abs() <= noise {
                    continue;
                }
                if directions[index] != 0.0 && directions[index] != change.signum() {
                    return Err(SolverError::UnsupportedInput(format!(
                        "case '{}': plastic Wen deformation of link '{}' DOF {} reverses within the load case; split the case at the reversal",
                        case.name,
                        tag.0,
                        tag.1 + 1
                    )));
                }
                directions[index] = change.signum();
            }
            if let (Some((dof, target)), Some(sign)) = (monitor, monitor_sign) {
                let moved = sign * (u[dof] - start[dof]);
                if moved < monitor_previous - 1e-9 * target {
                    return Err(SolverError::UnsupportedInput(format!(
                        "case '{}': monitored displacement does not increase monotonically with the load; load reduction under displacement control is not implemented",
                        case.name
                    )));
                }
                monitor_previous = moved;
            }
            previous = u;
        }
        Ok(factor)
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

#[cfg(test)]
mod tests {
    use super::*;

    /// The tabulated curve is checked against the two exponents that have a
    /// closed form: `1 - exp(-x)` for exponent 1 and `tanh(x)` for exponent 2.
    #[test]
    fn tabulated_wen_curve_matches_closed_forms_and_composes() {
        for (exponent, exact) in [
            (1.0, (|x: f64| 1.0 - (-x).exp()) as fn(f64) -> f64),
            (2.0, f64::tanh),
        ] {
            let table = WenCurve::tabulated(exponent);
            let closed = WenCurve::new(exponent);
            for i in 0..=400 {
                let x = 0.05 * i as f64;
                assert!((table.advance(0.0, x) - exact(x)).abs() < 1e-11, "{exponent} {x}");
                assert!((closed.advance(0.0, x) - exact(x)).abs() < 1e-14);
                // Advancing in two parts equals advancing once.
                let split = table.advance(table.advance(0.0, 0.37 * x), 0.63 * x);
                assert!((split - table.advance(0.0, x)).abs() < 1e-11);
                let split = closed.advance(closed.advance(0.0, 0.37 * x), 0.63 * x);
                assert!((split - exact(x)).abs() < 1e-13);
            }
            assert_eq!(table.advance(0.3, 0.0), 0.3);
            assert_eq!(table.advance(1.0, 5.0), 1.0);
        }
    }

    /// Unloading is elastic until z returns to zero, then yields the other way.
    #[test]
    fn wen_state_unloads_elastically_then_yields_in_reverse() {
        let law = WenLaw::new(100.0, 50.0, 0.1, 1.0);
        let (z_up, _) = law.advance(0.0, 0.0, 10.0);
        // Example 6-008: 9.6 in on the way back is still elastic (z = 0.2).
        let (z, slope) = law.advance(10.0, z_up, 9.6);
        assert!((z - 0.2).abs() < 1e-8 && slope == 2.0);
        assert!((law.force(9.6, z) - 105.0).abs() < 1e-6);
        // 8.6 in: z crossed zero at 9.5 in and has yielded 0.9 in the other way.
        let (z, _) = law.advance(10.0, z_up, 8.6);
        assert!((z + (1.0 - (-1.8f64).exp())).abs() < 1e-8);
        assert!((law.force(8.6, z) - 48.438).abs() < 5e-4);
        assert_eq!(law.advance(3.0, 0.4, 3.0), (0.4, 2.0));
    }
}
