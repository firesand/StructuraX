//! Frame plastic hinges in nonlinear static analysis.
//!
//! Source: CSI Analysis Reference Manual, chapters "Frame Hinge Properties"
//! and "Nonlinear Static Analysis". The traversal of strength loss follows the
//! OAPI option "unload entire structure" (`SetHingeUnloading` 1); the manual does
//! not describe it, so it is a reconstruction checked against the published
//! points of Example 1-026 only. Choices the manual does not state are marked
//! "project choice".
//!
//! A hinge is a plastic jump `q` (rotation or displacement) inside an elastic
//! frame element. Every response is linear in the load factor `lambda` of the
//! case and in the jumps `q`, so the nonlinear problem is solved in the space
//! of the hinges only: `F = F0 + lambda g + Phi q`, `U = U0 + lambda c + w.q`.
//! The module has two parts: a stepper that knows nothing about the finite
//! element model, and the glue that builds `F0, g, Phi, c, w` from it and
//! recovers the results of every saved state.

use indexmap::IndexMap;
use sap_model::hinge::{
    generated_name, FrameHingeResult, HingeBehavior, HingeBeyondE, HingeDof, HingeProp, HingeState,
    HingeStatus, HingeUnloading, NonlinearStaticStep, ResultsSaved,
};
use sap_model::{AreaObj, CaseResults, LoadCase, LoadCaseType, NLGeomType, PointObj, SapModel};

use crate::frame::{frame_section_forces_at, mat3_mul};
use crate::linalg::SymmetricLdlt;
use crate::{frame_mesh, nl_static_loads, SolverError};

// ---------------------------------------------------------------------------
// Routing
// ---------------------------------------------------------------------------

/// True when the model has a frame hinge.
pub(crate) fn model_has_hinges(model: &SapModel) -> bool {
    model.frames.values().any(|f| !f.hinges.is_empty())
}

/// Nonlinearity other than frame hinges that a case carries.
pub(crate) fn other_nonlinearity(model: &SapModel, lc: &LoadCase) -> bool {
    model
        .frames
        .values()
        .any(|f| f.cable_params.is_some() || f.tc_limits.is_some_and(|(c, _, t, _)| c || t))
        || model.links.values().any(|object| {
            model.link_props.get(&object.prop).is_some_and(|prop| {
                (0..6).any(|d| prop.dof_nonlinear[d] && prop.dof_active[d] && !prop.dof_fixed[d])
            })
        })
        || lc.nl_geom != NLGeomType::None
}

/// The nonlinear static chain of `lc`, oldest case first, `lc` last.
fn static_chain<'a>(model: &'a SapModel, lc: &'a LoadCase) -> Vec<&'a LoadCase> {
    let mut chain = vec![lc];
    let mut cur = lc;
    while let Some(prev) = cur
        .initial_case
        .as_ref()
        .and_then(|name| model.load_cases.get(name))
        .filter(|prev| prev.case_type == LoadCaseType::NonlinearStatic)
    {
        if chain.iter().any(|c| c.name == prev.name) {
            break;
        }
        chain.push(prev);
        cur = prev;
    }
    chain.reverse();
    chain
}

/// Whether a nonlinear static case is solved in hinge space: the model has a
/// frame hinge (`has_hinges`, known from the hinge sites because the elements
/// of a subdivided frame object do not carry its hinges), or the chain of the
/// case applies an Accel or Mode load, or the case saves multiple states in a
/// model of frames only with no other nonlinearity. A case that saves
/// multiple states in a model without hinges that has other nonlinearity, or
/// area, solid or link objects, keeps the existing path and only its final
/// state: a saved state holds joint, reaction, frame and hinge results only.
pub(crate) fn applies(model: &SapModel, lc: &LoadCase, has_hinges: bool) -> bool {
    if has_hinges {
        return true;
    }
    if static_chain(model, lc)
        .iter()
        .any(|c| nl_static_loads::uses_special_loads(c))
    {
        return true;
    }
    matches!(
        lc.nl_static.results_saved,
        ResultsSaved::MultipleStates { .. }
    ) && !other_nonlinearity(model, lc)
        && !has_other_objects(model)
}

/// Area, solid or link objects, which the hinge path does not carry.
fn has_other_objects(model: &SapModel) -> bool {
    !model.areas.is_empty() || !model.solids.is_empty() || !model.links.is_empty()
}

/// Refusals that depend on the case types of the model, for models with
/// hinges: hinges are ignored by linear cases and by modal-superposition
/// history, but nothing here may continue from a hinged tangent or integrate
/// a hinged structure directly.
pub(crate) fn validate_case_types(model: &SapModel) -> Result<(), SolverError> {
    if !model_has_hinges(model) {
        return Ok(());
    }
    for (name, lc) in model.load_cases.iter().filter(|(_, lc)| lc.run_flag) {
        if lc.case_type == LoadCaseType::NonlinearDynamic {
            return Err(SolverError::UnsupportedInput(format!(
                "case '{name}' integrates a model with frame hinges directly (nonlinear direct integration); frame hinges are implemented for nonlinear static cases only"
            )));
        }
        if matches!(
            lc.case_type,
            LoadCaseType::LinearStatic
                | LoadCaseType::LinearStaticMultiStep
                | LoadCaseType::Modal
                | LoadCaseType::Buckling
        ) {
            if let Some(initial) = lc
                .initial_case
                .as_ref()
                .and_then(|n| model.load_cases.get(n))
                .filter(|c| c.case_type == LoadCaseType::NonlinearStatic)
            {
                return Err(SolverError::UnsupportedInput(format!(
                    "case '{name}' continues from nonlinear static case '{}' of a model with frame hinges; the tangent stiffness of the hinged structure is not implemented",
                    initial.name
                )));
            }
        }
    }
    Ok(())
}

/// Whether any case that runs is solved in hinge space.
pub(crate) fn any_hinge_case(model: &SapModel) -> bool {
    model_has_hinges(model)
        && model.load_cases.values().any(|lc| {
            lc.run_flag && lc.case_type == LoadCaseType::NonlinearStatic && applies(model, lc, true)
        })
}

// ---------------------------------------------------------------------------
// Backbone curves
// ---------------------------------------------------------------------------

/// One segment of a backbone, in the plastic deformation magnitude `m`.
#[derive(Debug, Clone)]
pub(crate) struct Seg {
    pub q0: f64,
    /// `INFINITY` for the last segment.
    pub q1: f64,
    pub f0: f64,
    pub slope: f64,
    pub state: HingeState,
    /// The segment ends at point E of the table.
    pub ends_at_e: bool,
}

/// The force-deformation curve of one hinge for one direction of loading,
/// force and plastic deformation as magnitudes. Point B is the first point
/// (`q = 0`).
#[derive(Debug, Clone)]
pub(crate) struct Backbone {
    pub segs: Vec<Seg>,
    /// Why a hinge may not reach point E, if it may not.
    pub refuse_at_e: Option<String>,
    scale: f64,
}

impl Backbone {
    pub(crate) fn force_b(&self) -> f64 {
        self.segs[0].f0
    }

    /// Largest force on the curve, the scale of the hinge.
    pub(crate) fn force_scale(&self) -> f64 {
        self.segs.iter().fold(0.0f64, |m, s| m.max(s.f0))
    }

    /// Index of the segment of plastic deformation `m`; a point shared by two
    /// segments belongs to the later one.
    pub(crate) fn seg_at(&self, m: f64) -> usize {
        let tol = 1e-12 * self.scale;
        let mut at = 0;
        for (i, s) in self.segs.iter().enumerate() {
            if m >= s.q0 - tol {
                at = i;
            }
        }
        at
    }

    pub(crate) fn force_at(&self, m: f64) -> f64 {
        let s = &self.segs[self.seg_at(m)];
        (s.f0 + s.slope * (m - s.q0)).max(0.0)
    }
}

/// Builds the backbone of `prop` for loading in direction `sign`.
///
/// Only the deformation beyond point B is used (manual, "Frame Hinge
/// Properties"). `limit` is the stiffness of the slope limit for strength
/// loss of a bending hinge, 0.1 times the elastic bending stiffness `EI/L` of
/// the element (the value that reproduces Example 1-026; project choice for
/// property modifiers). A segment that falls more steeply, or drops, gets
/// that slope and ends later; the later points are moved by the same amount
/// (project choice). Hinges of the other degrees of freedom do not accept
/// strength loss.
pub(crate) fn build_backbone(
    prop: &HingeProp,
    sign: i8,
    limit: Option<f64>,
) -> Result<Backbone, String> {
    let (points, force_sf, deformation_sf) = if sign > 0 || prop.symmetric {
        (&prop.positive, prop.force_sf, prop.deformation_sf)
    } else {
        (
            &prop.negative,
            prop.force_sf_negative,
            prop.deformation_sf_negative,
        )
    };
    let force = |i: usize| points[i].force.abs() * force_sf;
    let deformation = |i: usize| points[i].deformation.abs() * deformation_sf;
    let d_b = deformation(0);
    // Points B, C, D, E as (plastic deformation, force).
    let pts: Vec<(f64, f64)> = (0..4).map(|i| (deformation(i) - d_b, force(i))).collect();
    let bending = matches!(prop.dof, HingeDof::M2 | HingeDof::M3);
    let lim = limit.filter(|l| *l > 0.0 && bending);

    let mut segs: Vec<Seg> = Vec::new();
    let states = [HingeState::BToC, HingeState::CToD, HingeState::DToE];
    let mut last_slope = 0.0f64;
    let mut at = (0.0f64, pts[0].1);
    for i in 0..3 {
        let (qa, fa) = pts[i];
        let (qb, fb) = pts[i + 1];
        let dq = qb - qa;
        let df = fb - fa;
        if dq <= 0.0 && df == 0.0 {
            continue;
        }
        if dq <= 0.0 && df > 0.0 {
            return Err(format!(
                "hinge property '{}' rises in force with no deformation",
                prop.name
            ));
        }
        if df < 0.0 && lim.is_none() {
            return Err(format!(
                "hinge property '{}' loses strength between points B and E, which is implemented for bending (M2, M3) hinges only",
                prop.name
            ));
        }
        let slope = if dq > 0.0 { df / dq } else { f64::NEG_INFINITY };
        let (slope, length) = match lim {
            Some(l) if df < 0.0 && slope < -l => (-l, -df / l),
            _ => (slope, dq),
        };
        // Later points move by `length - dq` because `at` carries on from here.
        let q0 = at.0;
        let q1 = q0 + length;
        segs.push(Seg {
            q0,
            q1,
            f0: at.1,
            slope,
            state: states[i],
            ends_at_e: false,
        });
        if slope.is_finite() && length > 0.0 {
            last_slope = slope;
        }
        at = (q1, fb);
    }
    if let Some(last) = segs.last_mut() {
        last.ends_at_e = true;
    }
    let e_force = at.1;
    let mut refuse_at_e = None;
    match prop.beyond_e {
        None => {
            refuse_at_e = Some(format!(
                "hinge property '{}' does not say what happens beyond point E (the imported code is not established)",
                prop.name
            ));
            segs.push(plateau(at, e_force));
        }
        Some(HingeBeyondE::Extrapolated) => {
            if last_slope < 0.0 {
                return Err(format!(
                    "hinge property '{}' extrapolates beyond point E with a negative slope, which is not implemented",
                    prop.name
                ));
            }
            segs.push(Seg {
                q0: at.0,
                q1: f64::INFINITY,
                f0: e_force,
                slope: last_slope,
                state: HingeState::BeyondE,
                ends_at_e: false,
            });
        }
        Some(HingeBeyondE::DropsToZero) => {
            if e_force > 0.0 {
                match lim {
                    Some(l) => {
                        let drop = e_force / l;
                        segs.push(Seg {
                            q0: at.0,
                            q1: at.0 + drop,
                            f0: e_force,
                            slope: -l,
                            state: HingeState::BeyondE,
                            ends_at_e: false,
                        });
                        segs.push(plateau((at.0 + drop, 0.0), 0.0));
                    }
                    None => {
                        refuse_at_e = Some(format!(
                            "a hinge of property '{}' reaches point E, beyond which its capacity drops to zero; that is implemented for bending (M2, M3) hinges only",
                            prop.name
                        ));
                        segs.push(plateau(at, e_force));
                    }
                }
            } else {
                segs.push(plateau(at, 0.0));
            }
        }
    }
    let scale = segs
        .iter()
        .filter(|s| s.q0.is_finite())
        .fold(0.0f64, |m, s| m.max(s.q0));
    Ok(Backbone {
        segs,
        refuse_at_e,
        scale,
    })
}

fn plateau(at: (f64, f64), force: f64) -> Seg {
    Seg {
        q0: at.0,
        q1: f64::INFINITY,
        f0: force,
        slope: 0.0,
        state: HingeState::BeyondE,
        ends_at_e: false,
    }
}

/// Both directions of one hinge.
#[derive(Debug, Clone)]
pub(crate) struct Curve {
    pub pos: Backbone,
    pub neg: Backbone,
}

impl Curve {
    fn bb(&self, dir: i8) -> &Backbone {
        if dir >= 0 {
            &self.pos
        } else {
            &self.neg
        }
    }
}

// ---------------------------------------------------------------------------
// Saved states
// ---------------------------------------------------------------------------

/// Which states of a case with multiple saved states are kept (manual,
/// "Output Steps"). Progress is the load factor of a load-controlled case and
/// the monitored displacement moved in the direction of loading otherwise.
#[derive(Debug, Clone)]
pub(crate) struct SavePlan {
    delta: f64,
    target: f64,
    max_steps: usize,
    positive_only: bool,
    steps_saved: usize,
    last: f64,
}

impl SavePlan {
    pub(crate) fn new(
        min_saved: usize,
        max_saved: usize,
        positive_only: bool,
        target: f64,
    ) -> Self {
        Self {
            delta: target / min_saved.max(1) as f64,
            target,
            max_steps: max_saved,
            positive_only,
            steps_saved: 0,
            last: 0.0,
        }
    }

    /// Progress of the next regularly spaced state.
    pub(crate) fn next_grid(&self) -> f64 {
        self.last + self.delta
    }

    /// Whether an event at `progress` is saved. The manual: an event is saved
    /// only if the maximum increment can still be kept within the maximum
    /// number of saved steps.
    pub(crate) fn event_wanted(&self, progress: f64) -> bool {
        if self.positive_only && progress <= self.last + 1e-12 * self.target {
            return false;
        }
        let remaining = ((self.target - progress) / self.delta - 1e-9)
            .ceil()
            .max(0.0) as usize;
        self.steps_saved + 1 + remaining <= self.max_steps
    }

    pub(crate) fn record(&mut self, progress: f64) {
        self.steps_saved += 1;
        self.last = progress;
    }

    /// Whether a final state at the target must still be recorded (it is not
    /// when the state at the target was saved already).
    pub(crate) fn needs_final(&self) -> bool {
        self.last < self.target * (1.0 - 1e-12)
    }
}

// ---------------------------------------------------------------------------
// Stepper
// ---------------------------------------------------------------------------

/// Everything the stepper needs to know about the structure.
#[derive(Debug, Clone)]
pub(crate) struct Problem {
    pub curves: Vec<Curve>,
    /// `n x n`, row-major: force at hinge i per unit plastic jump at hinge j.
    pub phi: Vec<f64>,
    /// Hinge forces per unit load factor of the case.
    pub g: Vec<f64>,
    /// Hinge forces at load factor 0 and no plastic jumps.
    pub f0: Vec<f64>,
    /// Monitored displacement per unit load factor and per unit jump.
    pub c: f64,
    pub w: Vec<f64>,
    /// Work-conjugate displacement of the case load, same decomposition.
    pub c_conj: f64,
    pub w_conj: Vec<f64>,
}

#[derive(Debug, Clone)]
pub(crate) struct State {
    pub lambda: f64,
    pub q: Vec<f64>,
    /// 0 never yielded, else the sign of the direction of loading.
    pub dir: Vec<i8>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum Control {
    Load,
    Monitored { conjugate: bool },
}

#[derive(Debug, Clone)]
pub(crate) struct RunSpec {
    pub control: Control,
    /// 1 for load control, the displacement target otherwise.
    pub target: f64,
    pub save: Option<(usize, usize, bool)>,
    pub max_total_steps: usize,
    pub max_null_steps: usize,
}

#[derive(Debug, Clone)]
pub(crate) struct RunOutcome {
    /// Saved states, step 0 first; empty when only the final state is kept.
    pub saved: Vec<State>,
    pub last: State,
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) enum StepError {
    /// The monitored displacement does not move with the load.
    NoMotion,
    /// Hinge `h` would unload through zero force after yielding.
    Reversal(usize),
    /// Hinge `h` reached point E and may not.
    ReachedE(usize, String),
    /// No state of the structure with the required direction of motion exists.
    Stuck {
        reason: String,
        steps: usize,
        progress: f64,
    },
    /// Steps ran out.
    Budget {
        reason: String,
        steps: usize,
        progress: f64,
    },
    Internal(String),
}

const EVENT_TIE: f64 = 1e-9;

struct Sol {
    dlam: f64,
    dq: Vec<f64>,
    df: Vec<f64>,
    /// Rate of progress per unit of the stepping measure.
    dp: f64,
    /// Magnitude of the terms of each force rate, for tolerances.
    row_mag: Vec<f64>,
}

struct Stepper<'a> {
    p: &'a Problem,
    n: usize,
    control: Control,
    s_target: f64,
    f_ref: f64,
    q_ref: f64,
}

impl Problem {
    pub(crate) fn forces(&self, st: &State) -> Vec<f64> {
        let n = self.curves.len();
        (0..n)
            .map(|i| {
                let mut f = self.f0[i] + st.lambda * self.g[i];
                for j in 0..n {
                    f += self.phi[i * n + j] * st.q[j];
                }
                f
            })
            .collect()
    }

    /// Monitored displacement relative to its value with no load factor and
    /// no plastic jumps.
    pub(crate) fn displacement(&self, st: &State) -> f64 {
        self.c * st.lambda + self.w.iter().zip(&st.q).map(|(w, q)| w * q).sum::<f64>()
    }
}

/// Solves `a x = b` (row-major `dim x dim`) after scaling rows and columns to
/// unit maximum, so that the singularity test does not depend on units.
fn solve_scaled(mut a: Vec<f64>, mut b: Vec<f64>, dim: usize) -> Option<Vec<f64>> {
    let mut col_scale = vec![1.0f64; dim];
    for i in 0..dim {
        let m = (0..dim).fold(0.0f64, |m, j| m.max(a[i * dim + j].abs()));
        if m == 0.0 || !m.is_finite() {
            return None;
        }
        for j in 0..dim {
            a[i * dim + j] /= m;
        }
        b[i] /= m;
    }
    for j in 0..dim {
        let m = (0..dim).fold(0.0f64, |m, i| m.max(a[i * dim + j].abs()));
        if m == 0.0 || !m.is_finite() {
            return None;
        }
        col_scale[j] = 1.0 / m;
        for i in 0..dim {
            a[i * dim + j] /= m;
        }
    }
    for k in 0..dim {
        let mut piv = k;
        let mut best = a[k * dim + k].abs();
        for i in k + 1..dim {
            if a[i * dim + k].abs() > best {
                best = a[i * dim + k].abs();
                piv = i;
            }
        }
        if best <= 1e-12 {
            return None;
        }
        if piv != k {
            for j in 0..dim {
                a.swap(k * dim + j, piv * dim + j);
            }
            b.swap(k, piv);
        }
        for i in k + 1..dim {
            let f = a[i * dim + k] / a[k * dim + k];
            if f != 0.0 {
                for j in k..dim {
                    a[i * dim + j] -= f * a[k * dim + j];
                }
                b[i] -= f * b[k];
            }
        }
    }
    let mut x = vec![0.0; dim];
    for i in (0..dim).rev() {
        let mut s = b[i];
        for j in i + 1..dim {
            s -= a[i * dim + j] * x[j];
        }
        x[i] = s / a[i * dim + i];
    }
    if x.iter().any(|v| !v.is_finite()) {
        return None;
    }
    Some(x.iter().zip(&col_scale).map(|(v, c)| v * c).collect())
}

/// Per-hinge classification at the start of a step.
struct Classes {
    /// Plastic deformation magnitude in the direction of loading.
    m: Vec<f64>,
    /// Hinge with `dir != 0` whose force lies on its backbone.
    on_backbone: Vec<bool>,
}

impl<'a> Stepper<'a> {
    fn slope(&self, h: usize, st: &State, m: f64) -> f64 {
        let bb = self.p.curves[h].bb(st.dir[h]);
        bb.segs[bb.seg_at(m)].slope
    }

    /// Fixes up yield flags from the current forces and classifies hinges.
    fn classify(
        &self,
        st: &mut State,
        active: &mut [bool],
        f: &[f64],
    ) -> Result<Classes, StepError> {
        let mut m = vec![0.0; self.n];
        let mut on_backbone = vec![false; self.n];
        for h in 0..self.n {
            let curve = &self.p.curves[h];
            if st.dir[h] == 0 {
                let (fp, fneg) = (curve.pos.force_b(), curve.neg.force_b());
                if f[h] >= fp * (1.0 - 1e-9) {
                    if f[h] > fp * (1.0 + 1e-6) {
                        return Err(StepError::Internal(format!(
                            "hinge {} force exceeds its yield force",
                            h + 1
                        )));
                    }
                    st.dir[h] = 1;
                    active[h] = true;
                } else if f[h] <= -fneg * (1.0 - 1e-9) {
                    if -f[h] > fneg * (1.0 + 1e-6) {
                        return Err(StepError::Internal(format!(
                            "hinge {} force exceeds its yield force",
                            h + 1
                        )));
                    }
                    st.dir[h] = -1;
                    active[h] = true;
                }
            } else if st.q[h] == 0.0
                && f[h] * f64::from(st.dir[h]) < curve.bb(st.dir[h]).force_b() * (1.0 - 1e-9)
            {
                st.dir[h] = 0;
                active[h] = false;
            }
            if st.dir[h] != 0 {
                let d = f64::from(st.dir[h]);
                m[h] = d * st.q[h];
                let bb = curve.bb(st.dir[h]);
                let fb = bb.force_at(m[h]);
                let fref = bb.force_scale().max(1e-300);
                let x = d * f[h];
                if m[h] > 0.0 && x < -1e-9 * fref {
                    return Err(StepError::Reversal(h));
                }
                if x > fb + 1e-6 * fref {
                    return Err(StepError::Internal(format!(
                        "hinge {} force exceeds its backbone",
                        h + 1
                    )));
                }
                on_backbone[h] = (x - fb).abs() <= 1e-9 * fref;
                if !on_backbone[h] {
                    active[h] = false;
                }
            } else {
                active[h] = false;
            }
        }
        Ok(Classes { m, on_backbone })
    }

    /// Solves the bordered system of the active set `a` for a unit step of
    /// the stepping measure with sign `sigma`.
    fn solve(&self, st: &State, cls: &Classes, a: &[bool], sigma: f64) -> Option<Sol> {
        let p = self.p;
        let n = self.n;
        let idx: Vec<usize> = (0..n).filter(|&h| a[h]).collect();
        let m = idx.len();
        let dim = m + 1;
        let mut mat = vec![0.0; dim * dim];
        let mut rhs = vec![0.0; dim];
        for (ri, &i) in idx.iter().enumerate() {
            for (cj, &j) in idx.iter().enumerate() {
                mat[ri * dim + cj] = -p.phi[i * n + j];
            }
            mat[ri * dim + ri] += self.slope(i, st, cls.m[i]);
            mat[ri * dim + m] = -p.g[i];
        }
        let (l_lam, l_q): (f64, Vec<f64>) = match self.control {
            Control::Load => (1.0, vec![0.0; m]),
            Control::Monitored { conjugate: false } => (
                self.s_target * p.c,
                idx.iter().map(|&j| self.s_target * p.w[j]).collect(),
            ),
            Control::Monitored { conjugate: true } => {
                (p.c_conj, idx.iter().map(|&j| p.w_conj[j]).collect())
            }
        };
        for (cj, lq) in l_q.iter().enumerate() {
            mat[m * dim + cj] = *lq;
        }
        mat[m * dim + m] = l_lam;
        rhs[m] = sigma;
        let x = solve_scaled(mat, rhs, dim)?;
        let dlam = x[m];
        let mut dq = vec![0.0; n];
        for (ri, &i) in idx.iter().enumerate() {
            dq[i] = x[ri];
        }
        let mut df = vec![0.0; n];
        let mut row_mag = vec![0.0; n];
        for i in 0..n {
            let mut v = p.g[i] * dlam;
            let mut mag = v.abs();
            for &j in &idx {
                let t = p.phi[i * n + j] * dq[j];
                v += t;
                mag += t.abs();
            }
            df[i] = v;
            row_mag[i] = mag;
        }
        let dp = match self.control {
            Control::Load => dlam,
            Control::Monitored { .. } => {
                self.s_target * (p.c * dlam + (0..n).map(|j| p.w[j] * dq[j]).sum::<f64>())
            }
        };
        Some(Sol {
            dlam,
            dq,
            df,
            dp,
            row_mag,
        })
    }

    /// The lowest-index hinge that violates the complementarity conditions.
    fn violator(&self, st: &State, a: &[bool], cand: &[bool], sol: &Sol) -> Option<usize> {
        let scale_q = sol.dq.iter().fold(0.0f64, |m, v| m.max(v.abs()));
        let scale_f = sol.df.iter().fold(0.0f64, |m, v| m.max(v.abs()));
        let tol_q = 1e-9 * scale_q + 1e-12 * self.q_ref;
        for h in 0..self.n {
            let d = f64::from(st.dir[h]);
            if a[h] {
                if sol.dq[h] * d < -tol_q {
                    return Some(h);
                }
            } else if cand[h] {
                let tol_f = 1e-9 * scale_f.max(sol.row_mag[h]) + 1e-12 * self.f_ref;
                if sol.df[h] * d > tol_f {
                    return Some(h);
                }
            }
        }
        None
    }

    /// Whether the set drives strength loss: some active hinge of negative
    /// slope keeps deforming in its direction of loading.
    fn softening_drives(&self, st: &State, cls: &Classes, a: &[bool], sol: &Sol) -> bool {
        let scale_q = sol.dq.iter().fold(0.0f64, |m, v| m.max(v.abs()));
        (0..self.n).any(|h| {
            a[h] && self.slope(h, st, cls.m[h]) < 0.0
                && sol.dq[h] * f64::from(st.dir[h]) > 1e-9 * scale_q + 1e-12 * self.q_ref
        })
    }

    /// Finds an admissible active set for the step `sigma` (a linear
    /// complementarity problem): least-index pivoting from the previous set,
    /// then enumeration when there are at most 12 candidates.
    #[allow(clippy::too_many_arguments)]
    fn search(
        &self,
        st: &State,
        cls: &Classes,
        a0: &[bool],
        cand: &[bool],
        sigma: f64,
        require_softening: bool,
    ) -> Option<(Vec<bool>, Sol)> {
        let n = self.n;
        let cidx: Vec<usize> = (0..n).filter(|&h| cand[h]).collect();
        let accept =
            |a: &[bool], sol: &Sol| !require_softening || self.softening_drives(st, cls, a, sol);
        let mut a: Vec<bool> = (0..n).map(|h| a0[h] && cand[h]).collect();
        for _ in 0..(4 * cidx.len() + 8) {
            match self.solve(st, cls, &a, sigma) {
                None => {
                    // A series tie leaves a null vector: drop the highest-index
                    // active hinge that carries no strength (project choice).
                    let drop = (0..n)
                        .rev()
                        .find(|&h| a[h] && self.slope(h, st, cls.m[h]) <= 0.0);
                    match drop {
                        Some(h) => a[h] = false,
                        None => break,
                    }
                }
                Some(sol) => match self.violator(st, &a, cand, &sol) {
                    None => {
                        if accept(&a, &sol) {
                            return Some((a, sol));
                        }
                        break;
                    }
                    Some(h) => a[h] = !a[h],
                },
            }
        }
        if cidx.len() <= 12 {
            let k = cidx.len();
            let mut masks: Vec<u32> = (0..(1u32 << k)).collect();
            masks.sort_by(|x, y| y.count_ones().cmp(&x.count_ones()).then(x.cmp(y)));
            for mask in masks {
                let mut set = vec![false; n];
                for (b, &h) in cidx.iter().enumerate() {
                    set[h] = mask >> b & 1 == 1;
                }
                if let Some(sol) = self.solve(st, cls, &set, sigma) {
                    if self.violator(st, &set, cand, &sol).is_none() && accept(&set, &sol) {
                        return Some((set, sol));
                    }
                }
            }
        }
        None
    }
}

/// Advances the state of the structure along the load path of one case.
///
/// The path is followed event by event: between events the active hinges
/// have a constant slope and the response is linear. Events are a hinge
/// yielding, reaching the end of a segment of its curve, returning to its
/// backbone (reload), the next saved state and the target. Event lumping is
/// exact (the vendor lumps events within one percent; project choice).
pub(crate) fn run(
    problem: &Problem,
    start: &State,
    spec: &RunSpec,
) -> Result<RunOutcome, StepError> {
    let n = problem.curves.len();
    let f_ref = problem
        .curves
        .iter()
        .map(|c| c.pos.force_scale().max(c.neg.force_scale()))
        .fold(0.0f64, f64::max);
    let q_ref = problem
        .curves
        .iter()
        .flat_map(|c| [&c.pos, &c.neg])
        .map(|b| b.scale)
        .fold(0.0f64, f64::max)
        .max(1e-3);
    let mut stepper = Stepper {
        p: problem,
        n,
        control: spec.control,
        s_target: 1.0,
        f_ref,
        q_ref,
    };
    let mut st = start.clone();
    let lambda0 = st.lambda;
    let u0 = problem.displacement(&st);
    let mut active = vec![false; n];

    if matches!(spec.control, Control::Monitored { .. }) {
        // The direction of loading: the way the monitored displacement moves
        // as the load grows from the start of the case.
        let f = problem.forces(&st);
        let mut probe = st.clone();
        let cls = stepper.classify(&mut probe, &mut active, &f)?;
        let cand: Vec<bool> = cls.on_backbone.clone();
        let saved_control = stepper.control;
        stepper.control = Control::Load;
        let first = stepper.search(&probe, &cls, &active.clone(), &cand, 1.0, false);
        stepper.control = saved_control;
        let rate = match first {
            Some((_, sol)) => {
                problem.c * sol.dlam + (0..n).map(|j| problem.w[j] * sol.dq[j]).sum::<f64>()
            }
            None => problem.c,
        };
        let scale = problem.c.abs() + problem.w.iter().fold(0.0f64, |m, w| m.max(w.abs()));
        if !rate.is_finite() || rate.abs() <= 1e-12 * scale || rate == 0.0 {
            return Err(StepError::NoMotion);
        }
        stepper.s_target = rate.signum();
        active = vec![false; n];
    }
    let s_target = stepper.s_target;
    let progress = |st: &State| match spec.control {
        Control::Load => st.lambda - lambda0,
        Control::Monitored { .. } => s_target * (problem.displacement(st) - u0),
    };
    let target = spec.target;
    let mut plan = spec
        .save
        .map(|(min, max, positive_only)| SavePlan::new(min, max, positive_only, target));
    let mut saved: Vec<State> = Vec::new();
    if plan.is_some() {
        saved.push(st.clone());
    }
    let mut steps = 0usize;
    let mut nulls = 0usize;

    loop {
        let p_now = progress(&st);
        if p_now >= target * (1.0 - 1e-12) {
            break;
        }
        let f = problem.forces(&st);
        let cls = stepper.classify(&mut st, &mut active, &f)?;
        steps += 1;
        if steps > spec.max_total_steps {
            return Err(StepError::Budget {
                reason: format!(
                    "the maximum number of total steps ({}) was reached; raise it in the case's nonlinear parameters",
                    spec.max_total_steps
                ),
                steps: steps - 1,
                progress: p_now,
            });
        }
        let cand: Vec<bool> = (0..n).map(|h| cls.on_backbone[h]).collect();
        // Forward first; with displacement control a step that moves the
        // monitored displacement back is accepted when strength loss drives
        // it (project reconstruction of "unload entire structure").
        let mut found = stepper.search(&st, &cls, &active, &cand, 1.0, false);
        let mut sigma = 1.0;
        if found.is_none() && matches!(spec.control, Control::Monitored { .. }) {
            found = stepper.search(&st, &cls, &active, &cand, -1.0, true);
            sigma = -1.0;
        }
        let Some((a, sol)) = found else {
            let reason = match spec.control {
                Control::Load => format!(
                    "no admissible state with increasing load at load factor {}: limit load or strength loss; use displacement control",
                    st.lambda
                ),
                Control::Monitored { .. } => format!(
                    "no admissible state continues the displacement-controlled path at progress {p_now}"
                ),
            };
            return Err(StepError::Stuck {
                reason,
                steps,
                progress: p_now,
            });
        };

        // Candidate events, in units of the stepping measure.
        #[derive(Clone, Copy, PartialEq)]
        enum Ev {
            SegEnd(usize),
            Yield(usize, i8),
            Reload(usize),
            Reverse(usize),
            Target,
            Grid,
        }
        let mut evs: Vec<(f64, Ev)> = Vec::new();
        let tiny = 1e-300;
        for h in 0..n {
            let curve = &problem.curves[h];
            let d = f64::from(st.dir[h]);
            if a[h] {
                let rate = sol.dq[h] * d;
                if rate > tiny {
                    let bb = curve.bb(st.dir[h]);
                    let seg = &bb.segs[bb.seg_at(cls.m[h])];
                    if seg.q1.is_finite() {
                        evs.push((((seg.q1 - cls.m[h]) / rate).max(0.0), Ev::SegEnd(h)));
                    }
                }
                continue;
            }
            if cls.m[h] == 0.0 {
                // Virgin (or just yielded and not active): the yield forces.
                let (fp, fneg) = (curve.pos.force_b(), curve.neg.force_b());
                if sol.df[h] > tiny && f[h] < fp * (1.0 - 1e-9) {
                    evs.push((((fp - f[h]) / sol.df[h]).max(0.0), Ev::Yield(h, 1)));
                } else if sol.df[h] < -tiny && f[h] > -fneg * (1.0 - 1e-9) {
                    evs.push((((-fneg - f[h]) / sol.df[h]).max(0.0), Ev::Yield(h, -1)));
                }
                continue;
            }
            // Yielded and rigid: unloaded, or on its backbone.
            let bb = curve.bb(st.dir[h]);
            let x = d * f[h];
            let dx = d * sol.df[h];
            if !cls.on_backbone[h] && dx > tiny {
                evs.push((((bb.force_at(cls.m[h]) - x) / dx).max(0.0), Ev::Reload(h)));
            }
            if dx < -tiny {
                evs.push(((x / -dx).max(0.0), Ev::Reverse(h)));
            }
        }
        if sol.dp > tiny {
            evs.push((((target - p_now) / sol.dp).max(0.0), Ev::Target));
            if let Some(plan) = &plan {
                let grid = plan.next_grid();
                if grid < target * (1.0 - 1e-9) {
                    evs.push((((grid - p_now) / sol.dp).max(0.0), Ev::Grid));
                }
            }
        }
        let Some(tau) = evs.iter().map(|(t, _)| *t).reduce(f64::min) else {
            return Err(StepError::Stuck {
                reason: "no event or target is reached along the current path".into(),
                steps,
                progress: p_now,
            });
        };
        if tau <= 1e-12 * target {
            nulls += 1;
            if nulls > spec.max_null_steps {
                return Err(StepError::Budget {
                    reason: format!(
                        "the maximum number of null steps ({}) was reached; raise it in the case's nonlinear parameters",
                        spec.max_null_steps
                    ),
                    steps,
                    progress: p_now,
                });
            }
        }
        // Advance, then process every event tied with the earliest.
        st.lambda += tau * sol.dlam;
        for h in 0..n {
            st.q[h] += tau * sol.dq[h];
        }
        let limit = tau * (1.0 + EVENT_TIE) + 1e-13 * target;
        let mut next_active = a.clone();
        let mut hit_target = false;
        let mut hit_grid = false;
        let mut hit_event = false;
        for (t, ev) in &evs {
            if *t > limit {
                continue;
            }
            match *ev {
                Ev::SegEnd(h) => {
                    let d = f64::from(st.dir[h]);
                    let bb = problem.curves[h].bb(st.dir[h]);
                    let seg = &bb.segs[bb.seg_at(cls.m[h])];
                    st.q[h] = d * seg.q1;
                    if seg.ends_at_e {
                        if let Some(reason) = &bb.refuse_at_e {
                            return Err(StepError::ReachedE(h, reason.clone()));
                        }
                    }
                    hit_event = true;
                }
                Ev::Yield(h, s) => {
                    st.dir[h] = s;
                    st.q[h] = 0.0;
                    next_active[h] = true;
                    hit_event = true;
                }
                Ev::Reload(h) => {
                    next_active[h] = true;
                    hit_event = true;
                }
                Ev::Reverse(h) => return Err(StepError::Reversal(h)),
                Ev::Target => hit_target = true,
                Ev::Grid => hit_grid = true,
            }
        }
        active = next_active;
        let p_new = progress(&st);
        let _ = sigma;
        if let Some(plan) = plan.as_mut() {
            if hit_target || p_new >= target * (1.0 - 1e-12) {
                if plan.needs_final() {
                    plan.record(p_new);
                    saved.push(st.clone());
                }
            } else if hit_grid || (hit_event && plan.event_wanted(p_new)) {
                plan.record(p_new);
                saved.push(st.clone());
            }
        }
    }
    Ok(RunOutcome { saved, last: st })
}

// ---------------------------------------------------------------------------
// Finite element glue
// ---------------------------------------------------------------------------

type Mat12 = [[f64; 12]; 12];

/// An analysis element that carries hinges.
struct Elem {
    name: String,
    ii: usize,
    ij: usize,
    ti: [[f64; 3]; 3],
    tj: [[f64; 3]; 3],
    k: Mat12,
    e_axes: [[f64; 3]; 3],
    length: f64,
}

/// A hinge with its place in the model.
struct Site {
    frame: String,
    index: usize,
    rel_dist: f64,
    prop: String,
    dof: HingeDof,
    elem: usize,
    rel: f64,
    /// -1 for M2, whose reported moment has the sense opposite to the
    /// element's internal one; the hinge variable is reported like the
    /// station force.
    sign: f64,
    /// `K_el * J`: the element-local forces of the "dislocation" J, the
    /// end-J motion of the part of the element beyond the hinge per unit jump.
    kj: [f64; 12],
    /// `J . K_el J`, the scale of the hinge's influence on itself.
    self_stiffness: f64,
}

fn mat_vec(k: &Mat12, v: &[f64; 12]) -> [f64; 12] {
    std::array::from_fn(|r| (0..12).map(|c| k[r][c] * v[c]).sum())
}

/// Element-local end displacements from joint-local displacements.
fn gather(u: &[f64], e: &Elem) -> [f64; 12] {
    let mut out = [0.0; 12];
    for r in 0..3 {
        for c in 0..3 {
            out[r] += e.ti[r][c] * u[6 * e.ii + c];
            out[r + 3] += e.ti[r][c] * u[6 * e.ii + 3 + c];
            out[r + 6] += e.tj[r][c] * u[6 * e.ij + c];
            out[r + 9] += e.tj[r][c] * u[6 * e.ij + 3 + c];
        }
    }
    out
}

/// Adds an element-local end force vector to joint-local forces.
fn scatter(f: &[f64; 12], e: &Elem, out: &mut [f64]) {
    for c in 0..3 {
        let (mut fi_f, mut fi_m, mut fj_f, mut fj_m) = (0.0, 0.0, 0.0, 0.0);
        for r in 0..3 {
            fi_f += e.ti[r][c] * f[r];
            fi_m += e.ti[r][c] * f[r + 3];
            fj_f += e.tj[r][c] * f[r + 6];
            fj_m += e.tj[r][c] * f[r + 9];
        }
        out[6 * e.ii + c] += fi_f;
        out[6 * e.ii + 3 + c] += fi_m;
        out[6 * e.ij + c] += fj_f;
        out[6 * e.ij + 3 + c] += fj_m;
    }
}

/// The J-end motion of the part of an element beyond station `x` when the
/// hinge there jumps by one unit (CSI Analysis Reference: the hinge is an
/// inelastic jump in an otherwise elastic element). Internal sense.
fn dislocation(dof: HingeDof, length: f64, x: f64) -> [f64; 12] {
    let mut j = [0.0; 12];
    let a = length - x;
    match dof {
        HingeDof::P => j[6] = 1.0,
        HingeDof::V2 => j[7] = 1.0,
        HingeDof::V3 => j[8] = 1.0,
        HingeDof::T => j[9] = 1.0,
        HingeDof::M2 => {
            j[10] = 1.0;
            j[8] = -a;
        }
        HingeDof::M3 => {
            j[11] = 1.0;
            j[7] = a;
        }
    }
    j
}

fn unsupported(msg: String) -> SolverError {
    SolverError::UnsupportedInput(msg)
}

/// Refusals for an element that carries a hinge.
fn check_element(model: &SapModel, site: &frame_mesh::HingeSite) -> Result<(), SolverError> {
    let frame = model.frames.get(&site.element).ok_or_else(|| {
        SolverError::InvalidInput(format!(
            "frame '{}' has no analysis element for its hinge",
            site.frame
        ))
    })?;
    let object = &site.frame;
    let sec = model
        .frame_sections
        .get(&frame.section)
        .ok_or_else(|| SolverError::SectionNotFound(frame.section.clone()))?;
    let reason = if frame.release_i.iter().chain(&frame.release_j).any(|r| *r)
        || frame
            .start_spring
            .iter()
            .chain(&frame.end_spring)
            .any(|k| *k != 0.0)
    {
        Some("end releases or partial fixity")
    } else if frame.end_offset_i != 0.0 || frame.end_offset_j != 0.0 {
        Some("end length offsets")
    } else if sec.prop_type == sap_model::FramePropType::Variable && !sec.non_prismatic.is_empty() {
        Some("a non-prismatic section")
    } else if frame.cable_params.is_some() {
        Some("a cable")
    } else if frame.tc_limits.is_some_and(|(c, _, t, _)| c || t) {
        Some("a tension or compression limit")
    } else if crate::frame::insertion_offset_local(model, frame) != Ok([0.0; 3]) {
        Some("an insertion point with stiffness transformation")
    } else {
        None
    };
    if let Some(reason) = reason {
        return Err(unsupported(format!(
            "frame '{object}' has hinges and {reason}, a combination that is not implemented"
        )));
    }
    if frame
        .loads_point
        .iter()
        .any(|lp| (lp.rel_dist - site.rel).abs() <= 1e-9)
    {
        return Err(unsupported(format!(
            "frame '{object}' has a point load at the station of a hinge (relative distance {}), where the section force is not defined",
            site.rel_dist
        )));
    }
    Ok(())
}

/// Maps a stepper failure to the solver error of the case.
fn map_error(e: StepError, lc: &LoadCase, sites: &[Site], target: f64) -> SolverError {
    let label = |h: usize| {
        format!(
            "hinge '{}' of frame '{}'",
            generated_name(&sites[h].frame, sites[h].index + 1),
            sites[h].frame
        )
    };
    match e {
        StepError::NoMotion => SolverError::InvalidInput(format!(
            "case '{}' load pattern does not move the monitored displacement",
            lc.name
        )),
        StepError::Reversal(h) => unsupported(format!(
            "case '{}': load reversal after yielding in {} (hysteresis is not implemented)",
            lc.name,
            label(h)
        )),
        StepError::ReachedE(h, reason) => {
            unsupported(format!("case '{}': {}: {reason}", lc.name, label(h)))
        }
        StepError::Stuck {
            reason,
            steps,
            progress,
        }
        | StepError::Budget {
            reason,
            steps,
            progress,
        } => SolverError::NonConvergence {
            case: lc.name.clone(),
            iterations: steps,
            residual: (target - progress).max(0.0),
            reason,
        },
        StepError::Internal(msg) => SolverError::InvalidInput(format!("case '{}': {msg}", lc.name)),
    }
}

fn hinge_result(
    site: &Site,
    prop: &HingeProp,
    curve: &Curve,
    force: f64,
    q: f64,
    dir: i8,
) -> FrameHingeResult {
    let m = f64::from(dir) * q;
    let (state, status, direction) = if dir == 0 || m <= 0.0 {
        (HingeState::AToB, HingeStatus::AToB, 0)
    } else {
        let bb = curve.bb(dir);
        let state = bb.segs[bb.seg_at(m)].state;
        // Acceptance criteria are table deformations on the axis of the curve
        // (informational; they never change the response).
        let (acc, sf, d_b) = if dir > 0 || prop.symmetric {
            (
                prop.acceptance_positive,
                prop.deformation_sf,
                prop.positive[0].deformation.abs(),
            )
        } else {
            (
                prop.acceptance_negative,
                prop.deformation_sf_negative,
                prop.negative[0].deformation.abs(),
            )
        };
        let status = if !acc.is_given() {
            HingeStatus::BToIO
        } else {
            let x = d_b + m / sf;
            if x <= acc.io.abs() {
                HingeStatus::BToIO
            } else if x <= acc.ls.abs() {
                HingeStatus::IOToLS
            } else if x <= acc.cp.abs() {
                HingeStatus::LSToCP
            } else {
                HingeStatus::BeyondCP
            }
        };
        (state, status, dir)
    };
    FrameHingeResult {
        frame: site.frame.clone(),
        hinge: site.index + 1,
        name: generated_name(&site.frame, site.index + 1),
        prop: site.prop.clone(),
        dof: site.dof,
        rel_dist: site.rel_dist,
        force,
        plastic_deformation: q,
        direction,
        state,
        status,
    }
}

/// Runs a nonlinear static case in hinge space (see the module comment).
///
/// Supported: frame elements with uncoupled deformation-controlled hinges of
/// user-defined curves, joint constraints, grounded joint springs, property
/// modifiers, self weight, frame distributed and point loads and joint
/// forces. A case that cannot reach its target returns an error and keeps no
/// partial results.
///
/// Limitation: the `local_disp` of the stations of a frame object is
/// integrated from the elastic curvature between its end values and has no
/// jump at an interior hinge; joint displacements, reactions and station
/// forces are exact.
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
    mesh: &frame_mesh::FrameMesh,
    ndof: usize,
) -> Result<CaseResults, SolverError> {
    let chain = static_chain(model, lc);
    for c in &chain {
        if other_nonlinearity(model, c) {
            return Err(unsupported(format!(
                "case '{}': saving multiple states, Mode and Accel loads and frame hinges are implemented for cases whose only nonlinearity is frame hinges; the model or the case has a nonlinear link, a tension/compression limit, a cable or geometric nonlinearity",
                c.name
            )));
        }
        if c.nl_static.hinge_unloading != HingeUnloading::UnloadEntireStructure {
            return Err(unsupported(format!(
                "case '{}' asks for hinge unloading {:?}; only \"unload entire structure\" is implemented",
                c.name, c.nl_static.hinge_unloading
            )));
        }
        if let Some(initial) = &c.initial_case {
            if model
                .load_cases
                .get(initial)
                .is_none_or(|p| p.case_type != LoadCaseType::NonlinearStatic)
            {
                return Err(unsupported(format!(
                    "case '{}' continues from '{initial}', which is not a nonlinear static case; a case with hinges can only continue from a nonlinear static case",
                    c.name
                )));
            }
        }
    }
    if has_other_objects(model) {
        return Err(unsupported(format!(
            "case '{}': frame hinges and Accel or Mode loads of a nonlinear static case are implemented for models of frame objects only; the model has area, solid or link objects",
            lc.name
        )));
    }
    let monitor_spec = match &lc.displacement_control {
        Some(ctl) => {
            let (point, dof) = ctl.monitored.as_ref().ok_or_else(|| {
                unsupported(format!(
                    "case '{}' uses displacement control without a representable monitored joint displacement",
                    lc.name
                ))
            })?;
            let index = *pt_index
                .get(point)
                .ok_or_else(|| SolverError::PointNotFound(point.clone()))?;
            Some((6 * index + dof, ctl))
        }
        None => None,
    };

    // Loads: what the chain of earlier cases left on the structure, and the
    // case's own loads at unit load factor.
    let mut chain_loads: Vec<(String, f64)> = Vec::new();
    let mut chain_special: Option<Vec<f64>> = None;
    for c in &chain[..chain.len() - 1] {
        let factor = model
            .case_results
            .get(&c.name)
            .and_then(|r| r.displacement_control_factor)
            .unwrap_or(1.0);
        for (pattern, sf) in nl_static_loads::pattern_loads(c) {
            chain_loads.push((pattern, factor * sf));
        }
        if let Some(v) =
            nl_static_loads::special_loads(model, c, pt_index, joint_rot, diag_mass, ndof)?
        {
            let acc = chain_special.get_or_insert_with(|| vec![0.0; ndof]);
            for (a, b) in acc.iter_mut().zip(&v) {
                *a += factor * b;
            }
        }
    }
    let own_loads = nl_static_loads::pattern_loads(lc);
    let own_special =
        nl_static_loads::special_loads(model, lc, pt_index, joint_rot, diag_mass, ndof)?;

    let no_axial = IndexMap::new();
    let assemble = |loads: &[(String, f64)]| {
        crate::assemble_static_loads(
            model,
            loads,
            work_points,
            work_areas,
            pt_index,
            joint_rot,
            &no_axial,
            ndof,
        )
    };
    let base = assemble(&[])?;
    let chain_sl = assemble(&chain_loads)?;
    let own_sl = assemble(&own_loads)?;
    for sl in [&chain_sl, &own_sl] {
        if sl.prescribed_u.iter().any(Option::is_some)
            || sl.spring_ground_u.iter().any(|u| *u != 0.0)
        {
            return Err(unsupported(format!(
                "case '{}' uses joint displacement (ground motion) loads, which a nonlinear static case with hinges does not implement",
                lc.name
            )));
        }
    }
    let used: Vec<&str> = chain_loads
        .iter()
        .chain(&own_loads)
        .filter(|(_, sf)| *sf != 0.0)
        .map(|(p, _)| p.as_str())
        .collect();
    for frame in model.frames.values() {
        if frame
            .loads_temp
            .iter()
            .any(|t| used.contains(&t.load_pat.as_str()))
        {
            return Err(unsupported(format!(
                "frame '{}' has a temperature load in a pattern of case '{}'; temperature loads are not implemented in nonlinear static cases with hinges",
                frame.name, lc.name
            )));
        }
    }

    // Hinge sites and curves.
    let mut elems: Vec<Elem> = Vec::new();
    let mut elem_of: std::collections::HashMap<String, usize> = std::collections::HashMap::new();
    let mut sites: Vec<Site> = Vec::new();
    let mut curves: Vec<Curve> = Vec::new();
    let mut props: Vec<&HingeProp> = Vec::new();
    for hs in &mesh.hinge_sites {
        check_element(model, hs)?;
        let prop = model.hinge_props.get(&hs.prop).ok_or_else(|| {
            SolverError::InvalidInput(format!(
                "frame '{}' has a hinge of undefined property '{}'",
                hs.frame, hs.prop
            ))
        })?;
        if prop.behavior == HingeBehavior::ForceControlled {
            return Err(unsupported(format!(
                "hinge property '{}' (frame '{}') is force controlled, which is not implemented",
                prop.name, hs.frame
            )));
        }
        prop.validate().map_err(SolverError::InvalidInput)?;
        if sites.iter().any(|s| {
            s.frame == hs.frame && s.dof == prop.dof && (s.rel_dist - hs.rel_dist).abs() <= 1e-9
        }) {
            return Err(unsupported(format!(
                "frame '{}' has two hinges of the same degree of freedom ({}) at relative distance {}",
                hs.frame,
                prop.dof.label(),
                hs.rel_dist
            )));
        }
        let e = match elem_of.get(&hs.element) {
            Some(&e) => e,
            None => {
                let frame = &model.frames[&hs.element];
                let (_, k, e_axes, length) = &base.frame_fef_map[&hs.element];
                let (ii, ij) = (pt_index[&frame.point_i], pt_index[&frame.point_j]);
                elems.push(Elem {
                    name: hs.element.clone(),
                    ii,
                    ij,
                    ti: mat3_mul(e_axes, &joint_rot[ii]),
                    tj: mat3_mul(e_axes, &joint_rot[ij]),
                    k: *k,
                    e_axes: *e_axes,
                    length: *length,
                });
                elem_of.insert(hs.element.clone(), elems.len() - 1);
                elems.len() - 1
            }
        };
        let elem = &elems[e];
        let frame = &model.frames[&hs.element];
        let sec = &model.frame_sections[&frame.section];
        let mat = &model.materials[&sec.material];
        let x = hs.rel * elem.length;
        // Slope limit of strength loss: 0.1 EI/L of the element that holds the
        // hinge (see `build_backbone`).
        let section = crate::frame::eval_section_props_at(model, frame, sec, mat, 0.5, elem.length);
        let limit = match prop.dof {
            HingeDof::M3 => Some(0.1 * mat.e * section.i33 / elem.length),
            HingeDof::M2 => Some(0.1 * mat.e * section.i22 / elem.length),
            _ => None,
        };
        let build = |sign: i8| {
            build_backbone(prop, sign, limit)
                .map_err(|msg| unsupported(format!("frame '{}': {msg}", hs.frame)))
        };
        curves.push(Curve {
            pos: build(1)?,
            neg: build(-1)?,
        });
        props.push(prop);
        let sign = if prop.dof == HingeDof::M2 { -1.0 } else { 1.0 };
        let mut jv = dislocation(prop.dof, elem.length, x);
        for v in &mut jv {
            *v *= sign;
        }
        let kj = mat_vec(&elem.k, &jv);
        let self_stiffness = jv.iter().zip(&kj).map(|(a, b)| a * b).sum::<f64>().abs();
        sites.push(Site {
            frame: hs.frame.clone(),
            index: hs.index,
            rel_dist: hs.rel_dist,
            prop: hs.prop.clone(),
            dof: prop.dof,
            elem: e,
            rel: hs.rel,
            sign,
            kj,
            self_stiffness,
        });
    }
    let n = sites.len();

    // Factor the stiffness once; restrained and inactive DOFs carry zero.
    let is_fixed = &base.is_fixed;
    let mut k_mod = k_base.to_vec();
    for i in 0..ndof {
        if is_fixed[i] {
            for j in 0..ndof {
                k_mod[i * ndof + j] = 0.0;
                k_mod[j * ndof + i] = 0.0;
            }
            k_mod[i * ndof + i] = 1.0;
        }
    }
    let structural_diagonal: Vec<f64> = (0..ndof).map(|i| k_struct[i * ndof + i]).collect();
    let ldlt = SymmetricLdlt::factorize_with_reference(&k_mod, ndof, Some(&structural_diagonal))?;
    let solve = |rhs: &[f64]| -> Result<Vec<f64>, SolverError> {
        let mut r = rhs.to_vec();
        for i in 0..ndof {
            if is_fixed[i] {
                r[i] = 0.0;
            }
        }
        Ok(ldlt.solve(&r)?)
    };
    let with_special = |rhs: &[f64], special: Option<&Vec<f64>>| -> Vec<f64> {
        let mut r = rhs.to_vec();
        if let Some(s) = special {
            for (a, b) in r.iter_mut().zip(s) {
                *a += b;
            }
        }
        r
    };
    let rhs_chain = with_special(&chain_sl.rhs, chain_special.as_ref());
    let rhs_own = with_special(&own_sl.rhs, own_special.as_ref());
    let u_chain = solve(&rhs_chain)?;
    let u_own = solve(&rhs_own)?;

    // Hinge forces from an element-end force vector.
    let section_force = |site: &Site, f_end: &[f64; 12], loads: &[(String, f64)]| -> f64 {
        let e = &elems[site.elem];
        let frame = &model.frames[&e.name];
        let sec = &model.frame_sections[&frame.section];
        let mat = &model.materials[&sec.material];
        site.sign
            * frame_section_forces_at(
                model, frame, sec, mat, &e.e_axes, e.length, f_end, loads, site.rel,
            )[site.dof.local_index()]
    };
    let elastic_end_forces = |sl: &crate::StaticLoads, u: &[f64], e: &Elem| -> [f64; 12] {
        let local = gather(u, e);
        let kl = mat_vec(&e.k, &local);
        let fef = &sl.frame_fef_map[&e.name].0;
        std::array::from_fn(|r| fef[r] + kl[r])
    };
    let elem_end_forces = |sl: &crate::StaticLoads, u: &[f64]| -> Vec<[f64; 12]> {
        elems.iter().map(|e| elastic_end_forces(sl, u, e)).collect()
    };
    let f_end_chain = elem_end_forces(&chain_sl, &u_chain);
    let f_end_own = elem_end_forces(&own_sl, &u_own);
    let f0: Vec<f64> = sites
        .iter()
        .map(|s| section_force(s, &f_end_chain[s.elem], &chain_loads))
        .collect();
    let g: Vec<f64> = sites
        .iter()
        .map(|s| section_force(s, &f_end_own[s.elem], &own_loads))
        .collect();

    // Unit jumps: displacement fields, hinge-force influence matrix and the
    // monitored and conjugate displacements.
    let mut phi = vec![0.0; n * n];
    let mut u_hinge: Vec<Vec<f64>> = Vec::with_capacity(n);
    let monitor = monitor_spec.as_ref().map(|(i, _)| *i);
    if let Some(i) = monitor {
        if is_fixed[i] {
            return Err(SolverError::InvalidInput(format!(
                "case '{}' monitors a restrained or inactive displacement",
                lc.name
            )));
        }
    }
    let mut w = vec![0.0; n];
    let mut w_conj = vec![0.0; n];
    for (h, site) in sites.iter().enumerate() {
        // The jump relaxes the element: an eigen fixed-end force `-K J`.
        let mut nodal = vec![0.0; ndof];
        scatter(&site.kj, &elems[site.elem], &mut nodal);
        let uh = solve(&nodal)?;
        for (i, s) in sites.iter().enumerate() {
            let e = &elems[s.elem];
            let local = gather(&uh, e);
            let mut f = mat_vec(&e.k, &local);
            if s.elem == site.elem {
                for r in 0..12 {
                    f[r] -= site.kj[r];
                }
            }
            phi[i * n + h] = section_force(s, &f, &[]);
        }
        if let Some(m) = monitor {
            w[h] = uh[m];
        }
        w_conj[h] = rhs_own.iter().zip(&uh).map(|(a, b)| a * b).sum();
        u_hinge.push(uh);
    }
    // The influence matrix of unit jumps is symmetric: the work of a jump at
    // one hinge through the force of another is reciprocal.
    let phi_scale = phi
        .iter()
        .fold(0.0f64, |m, v| m.max(v.abs()))
        .max(sites.iter().fold(0.0f64, |m, s| m.max(s.self_stiffness)));
    for i in 0..n {
        for j in 0..i {
            debug_assert!(
                (phi[i * n + j] - phi[j * n + i]).abs() <= 1e-6 * phi_scale.max(1e-300),
                "hinge influence matrix is not symmetric"
            );
        }
    }
    let c = monitor.map_or(0.0, |m| u_own[m]);
    let c_conj: f64 = rhs_own.iter().zip(&u_own).map(|(a, b)| a * b).sum();
    let problem = Problem {
        curves,
        phi,
        g,
        f0,
        c,
        w,
        c_conj,
        w_conj,
    };

    // Starting state: the hinges as the initial case left them.
    let mut start = State {
        lambda: 0.0,
        q: vec![0.0; n],
        dir: vec![0; n],
    };
    let mut carried: Vec<Option<f64>> = vec![None; n];
    if let Some(initial) = &lc.initial_case {
        let previous = &model.case_results[initial];
        if n > 0 && previous.hinge_results.len() != n {
            return Err(SolverError::InvalidInput(format!(
                "case '{}' continues from '{initial}', which has no hinge state for its {n} hinges",
                lc.name
            )));
        }
        for (h, site) in sites.iter().enumerate() {
            let r = previous
                .hinge_results
                .iter()
                .find(|r| r.frame == site.frame && r.hinge == site.index + 1)
                .ok_or_else(|| {
                    SolverError::InvalidInput(format!(
                        "case '{initial}' has no state for hinge {} of frame '{}'",
                        site.index + 1,
                        site.frame
                    ))
                })?;
            start.q[h] = r.plastic_deformation;
            start.dir[h] = r.direction;
            if (start.q[h] != 0.0) != (r.direction != 0) {
                return Err(SolverError::InvalidInput(format!(
                    "case '{initial}' has an inconsistent state for hinge {} of frame '{}'",
                    site.index + 1,
                    site.frame
                )));
            }
            carried[h] = Some(r.force);
        }
        let f_now = problem.forces(&start);
        let scale = problem
            .curves
            .iter()
            .map(|c| c.pos.force_scale().max(c.neg.force_scale()))
            .fold(0.0f64, f64::max);
        for h in 0..n {
            if let Some(f) = carried[h] {
                if (f - f_now[h]).abs() > 1e-8 * scale.max(1e-300) {
                    return Err(SolverError::InvalidInput(format!(
                        "case '{}' does not continue from the state of '{initial}': hinge {} of frame '{}' carries {f} there but {} here",
                        lc.name,
                        sites[h].index + 1,
                        sites[h].frame,
                        f_now[h]
                    )));
                }
            }
        }
    }

    let (control, target) = match monitor_spec {
        Some((_, ctl)) => (
            Control::Monitored {
                conjugate: ctl.conjugate,
            },
            ctl.target,
        ),
        None => (Control::Load, 1.0),
    };
    let save = match lc.nl_static.results_saved {
        ResultsSaved::FinalState => None,
        ResultsSaved::MultipleStates {
            min_saved,
            max_saved,
            positive_only,
        } => {
            if min_saved == 0 || max_saved == 0 {
                return Err(SolverError::InvalidInput(format!(
                    "case '{}' saves multiple states with a minimum of {min_saved} and a maximum of {max_saved} steps; both must be at least 1",
                    lc.name
                )));
            }
            Some((min_saved, max_saved, positive_only))
        }
    };
    if lc.nl_static.max_total_steps == 0 {
        return Err(SolverError::InvalidInput(format!(
            "case '{}' has a maximum of zero total steps",
            lc.name
        )));
    }
    let spec = RunSpec {
        control,
        target,
        save,
        max_total_steps: lc.nl_static.max_total_steps,
        max_null_steps: lc.nl_static.max_null_steps,
    };
    let outcome = run(&problem, &start, &spec).map_err(|e| map_error(e, lc, &sites, target))?;

    // Results of the saved states and of the final state.
    let recover = |st: &State| -> Result<(CaseResults, Vec<FrameHingeResult>), SolverError> {
        let mut case_loads = chain_loads.clone();
        for (pattern, sf) in &own_loads {
            case_loads.push((pattern.clone(), st.lambda * sf));
        }
        let mut sl = assemble(&case_loads)?;
        for i in 0..ndof {
            if let Some(s) = &chain_special {
                sl.f_ext_joint[i] += s[i];
            }
            if let Some(s) = &own_special {
                sl.f_ext_joint[i] += st.lambda * s[i];
            }
        }
        let mut u: Vec<f64> = (0..ndof)
            .map(|i| u_chain[i] + st.lambda * u_own[i])
            .collect();
        for (h, site) in sites.iter().enumerate() {
            let q = st.q[h];
            if q == 0.0 {
                continue;
            }
            for (a, b) in u.iter_mut().zip(&u_hinge[h]) {
                *a += q * b;
            }
            let e = &elems[site.elem];
            let eigen: [f64; 12] = std::array::from_fn(|r| -q * site.kj[r]);
            let fef = &mut sl.frame_fef_map.get_mut(&e.name).unwrap().0;
            for r in 0..12 {
                fef[r] += eigen[r];
            }
            scatter(&eigen, e, &mut sl.f_fef_joint);
        }
        let results = crate::recover_static_results(
            model,
            &lc.name,
            &case_loads,
            work_points,
            work_areas,
            orig_area_map,
            pt_index,
            joint_rot,
            k_struct,
            &sl,
            &u,
            None,
            None,
        )?;
        let forces = problem.forces(st);
        let hinges = sites
            .iter()
            .enumerate()
            .map(|(h, site)| {
                hinge_result(
                    site,
                    props[h],
                    &problem.curves[h],
                    forces[h],
                    st.q[h],
                    st.dir[h],
                )
            })
            .collect();
        Ok((results, hinges))
    };
    let mut steps = Vec::with_capacity(outcome.saved.len());
    for (number, state) in outcome.saved.iter().enumerate() {
        let (r, hinges) = recover(state)?;
        let monitored = monitor_spec
            .as_ref()
            .and_then(|(_, ctl)| ctl.monitored.as_ref())
            .map(|(point, dof)| r.joint_displacements[point.as_str()][*dof]);
        steps.push(NonlinearStaticStep {
            step: number,
            load_factor: state.lambda,
            monitored_displacement: monitored,
            base_reactions: r.base_reactions,
            joint_displacements: r.joint_displacements,
            joint_reactions: r.joint_reactions,
            frame_forces: r.frame_forces,
            hinges,
        });
    }
    let (mut results, hinges) = recover(&outcome.last)?;
    results.displacement_control_factor = Some(outcome.last.lambda);
    results.hinge_results = hinges;
    results.static_steps = steps;
    Ok(results)
}

#[cfg(test)]
mod tests {
    use super::*;
    use sap_model::hinge::HingePoint;

    /// Example 1-026 (kip, in): cantilever, moment hinge and shear hinge at
    /// the fixed end, unit load at the tip. `limit` is the slope limit.
    fn example_1_026(limit: f64) -> Problem {
        let curve = |dof, pts: [(f64, f64); 4], lim| {
            let prop = HingeProp::symmetric("H", dof, pts.map(|(f, d)| HingePoint::new(f, d)));
            Curve {
                pos: build_backbone(&prop, 1, lim).unwrap(),
                neg: build_backbone(&prop, -1, lim).unwrap(),
            }
        };
        let moment = curve(
            HingeDof::M3,
            [(1440.0, 0.0), (1920.0, 0.04), (480.0, 0.04), (480.0, 0.4)],
            Some(limit),
        );
        let shear = curve(
            HingeDof::V2,
            [(70.0, 0.0), (80.0, 0.4), (90.0, 0.8), (100.0, 1.2)],
            None,
        );
        let c = 24.0f64.powi(3) / (3.0 * 3600.0 * 5832.0) + 24.0 / (1500.0 * 180.0);
        Problem {
            curves: vec![moment, shear],
            phi: vec![0.0; 4],
            g: vec![24.0, 1.0],
            f0: vec![0.0, 0.0],
            c,
            w: vec![24.0, 1.0],
            c_conj: c,
            w_conj: vec![24.0, 1.0],
        }
    }

    fn run_1_026(limit: f64, conjugate: bool, positive_only: bool) -> (Problem, RunOutcome) {
        let problem = example_1_026(limit);
        let start = State {
            lambda: 0.0,
            q: vec![0.0; 2],
            dir: vec![0; 2],
        };
        let spec = RunSpec {
            control: Control::Monitored { conjugate },
            target: 2.0,
            save: Some((6, 10, positive_only)),
            max_total_steps: 200,
            max_null_steps: 50,
        };
        let out = run(&problem, &start, &spec).expect("traverses");
        (problem, out)
    }

    fn schedule(problem: &Problem, out: &RunOutcome) -> Vec<f64> {
        out.saved.iter().map(|s| problem.displacement(s)).collect()
    }

    fn assert_schedule(got: &[f64], want: &[f64]) {
        assert_eq!(got.len(), want.len(), "{got:?}");
        for (g, w) in got.iter().zip(want) {
            assert!((g - w).abs() <= 5e-7, "{got:?} vs {want:?}");
        }
    }

    const A_LIMIT: f64 = 0.1 * 3600.0 * 5832.0 / 24.0;
    const B_LIMIT: f64 = 0.1 * 3600.0 * 5832.0 / 0.24;

    #[test]
    fn example_1_026_a_follows_the_published_points() {
        let (problem, out) = run_1_026(A_LIMIT, false, false);
        assert_schedule(
            &schedule(&problem, &out),
            &[
                0.0, 0.0185021, 0.3518354, 0.5015857, 0.8349191, 1.1682524, 1.3846694, 1.7180027,
                1.7612291, 2.0,
            ],
        );
        // Point 1, 2, 3 and the end: P = 60, 80, 20, 20.
        let lam: Vec<f64> = out.saved.iter().map(|s| s.lambda).collect();
        for (i, p) in [(1, 60.0), (6, 80.0), (8, 20.0), (9, 20.0)] {
            assert!((lam[i] - p).abs() <= 1e-9, "{lam:?}");
        }
        let last = &out.last;
        assert!((last.q[0] - 0.066409694).abs() <= 1e-8);
        assert!((last.q[1] - 0.4).abs() <= 1e-12);
    }

    #[test]
    fn example_1_026_b_snaps_back_to_the_published_point() {
        let (problem, out) = run_1_026(B_LIMIT, false, false);
        assert_schedule(
            &schedule(&problem, &out),
            &[
                0.0, 0.0185021, 0.3518354, 0.5015857, 0.8349191, 1.1682524, 1.3846694, 1.3701180,
                1.7034513, 2.0,
            ],
        );
        let lam: Vec<f64> = out.saved.iter().map(|s| s.lambda).collect();
        assert!((lam[7] - 20.0).abs() <= 1e-9, "{lam:?}");
        // Positive increments only drop the backward point.
        let (problem, out) = run_1_026(B_LIMIT, false, true);
        assert_schedule(
            &schedule(&problem, &out),
            &[
                0.0, 0.0185021, 0.3518354, 0.5015857, 0.8349191, 1.1682524, 1.3846694, 1.7180027,
                2.0,
            ],
        );
    }

    /// Walks the progress from 0 to `target` the way `run` does, with significant
    /// events at the given progress values, and returns the saved progress values.
    fn simulate(min: usize, max: usize, target: f64, events: &[f64]) -> Vec<f64> {
        let mut plan = SavePlan::new(min, max, true, target);
        let mut saved = Vec::new();
        let mut p = 0.0;
        let mut pending = events.iter().copied().peekable();
        loop {
            let grid = plan.next_grid().min(target);
            let event = pending.peek().copied().filter(|e| *e > p);
            match event {
                Some(e) if e < grid - 1e-12 => {
                    p = e;
                    pending.next();
                    if plan.event_wanted(p) {
                        plan.record(p);
                        saved.push(p);
                    }
                }
                _ => {
                    p = grid;
                    if p >= target - 1e-12 {
                        if plan.needs_final() {
                            plan.record(p);
                            saved.push(p);
                        }
                        break;
                    }
                    plan.record(p);
                    saved.push(p);
                }
            }
        }
        saved
    }

    fn assert_progress(got: &[f64], want: &[f64]) {
        assert_eq!(got.len(), want.len(), "{got:?} vs {want:?}");
        for (g, w) in got.iter().zip(want) {
            assert!((g - w).abs() < 1e-9, "{got:?} vs {want:?}");
        }
    }

    #[test]
    fn saved_steps_follow_the_two_examples_of_the_manual() {
        // Min 20, max 30, target 10: spacing 0.5; an event at 2.7 shifts the grid.
        let mut want: Vec<f64> = (1..=5).map(|k| 0.5 * k as f64).collect();
        want.push(2.7);
        want.extend((0..).map(|k| 3.2 + 0.5 * k as f64).take_while(|v| *v < 9.8));
        want.push(10.0);
        assert_progress(&simulate(20, 30, 10.0, &[2.7]), &want);
        // Max 21: 2.7 and then 3.2, 3.7 are saved, an event at 3.9 is skipped (it would
        // break the maximum increment within 21 steps) ...
        let mut want: Vec<f64> = (1..=5).map(|k| 0.5 * k as f64).collect();
        want.extend([2.7, 3.2, 3.7]);
        want.extend((0..).map(|k| 4.2 + 0.5 * k as f64).take_while(|v| *v < 9.8));
        want.push(10.0);
        assert_eq!(want.len(), 21);
        assert_progress(&simulate(20, 21, 10.0, &[2.7, 3.9]), &want);
        // ... but one at 4.1 is saved.
        let mut want: Vec<f64> = (1..=5).map(|k| 0.5 * k as f64).collect();
        want.extend([2.7, 3.2, 3.7, 4.1]);
        want.extend((0..).map(|k| 4.6 + 0.5 * k as f64).take_while(|v| *v < 9.7));
        want.push(10.0);
        assert_eq!(want.len(), 21);
        assert_progress(&simulate(20, 21, 10.0, &[2.7, 4.1]), &want);
    }

    #[test]
    fn saved_steps_never_repeat_the_target_or_save_backwards_when_positive_only() {
        let plan = SavePlan::new(4, 10, true, 1.0);
        assert!(!plan.event_wanted(0.0));
        let mut plan = SavePlan::new(4, 10, true, 1.0);
        plan.record(0.5);
        assert!(!plan.event_wanted(0.4), "positive increments only");
        assert!(plan.event_wanted(0.6));
        let mut open = SavePlan::new(4, 10, false, 1.0);
        open.record(0.5);
        assert!(
            open.event_wanted(0.4),
            "backward states are kept when asked for"
        );
        assert!(plan.needs_final());
        plan.record(1.0);
        assert!(!plan.needs_final());
        // An exact multiple of the spacing does not count an extra step.
        let exact = SavePlan::new(10, 11, true, 1.0);
        assert!(exact.event_wanted(0.1 + 1e-12));
    }

    #[test]
    fn backbone_applies_the_slope_limit_and_moves_later_points() {
        let prop = HingeProp::symmetric(
            "Moment",
            HingeDof::M3,
            [
                HingePoint::new(1440.0, 0.0),
                HingePoint::new(1920.0, 0.04),
                HingePoint::new(480.0, 0.04),
                HingePoint::new(480.0, 0.4),
            ],
        );
        let k = A_LIMIT;
        let bb = build_backbone(&prop, 1, Some(k)).unwrap();
        let delta = (1920.0 - 480.0) / k;
        assert_eq!(bb.segs.len(), 5);
        assert_eq!(
            (bb.segs[0].q0, bb.segs[0].q1, bb.segs[0].slope),
            (0.0, 0.04, 12000.0)
        );
        assert_eq!(bb.segs[0].state, HingeState::BToC);
        assert!((bb.segs[1].q1 - (0.04 + delta)).abs() < 1e-15);
        assert_eq!(
            (bb.segs[1].f0, bb.segs[1].slope, bb.segs[1].state),
            (1920.0, -k, HingeState::CToD)
        );
        // D and E move by the same amount: the D-E segment keeps its 0.36 length.
        assert!((bb.segs[2].q0 - (0.04 + delta)).abs() < 1e-15);
        assert!((bb.segs[2].q1 - (0.4 + delta)).abs() < 1e-15);
        assert!(bb.segs[2].ends_at_e && bb.refuse_at_e.is_none());
        // Beyond E the capacity drops to zero on the limit, then stays zero.
        assert!((bb.segs[3].q1 - (0.4 + delta + 480.0 / k)).abs() < 1e-14);
        assert_eq!((bb.segs[4].f0, bb.segs[4].slope), (0.0, 0.0));
        assert!(bb.segs[4].q1.is_infinite());
        // Force on the curve, with a shared point belonging to the later segment.
        assert_eq!(bb.force_at(0.0), 1440.0);
        assert!((bb.force_at(0.02) - 1680.0).abs() < 1e-9);
        assert_eq!(bb.segs[bb.seg_at(0.04)].state, HingeState::CToD);
        assert!((bb.force_at(0.04 + delta) - 480.0).abs() < 1e-9);
        assert_eq!(bb.force_at(10.0), 0.0);
        assert_eq!(bb.force_scale(), 1920.0);
        // A steep but not limited fall keeps its slope; a limit of zero stiffness means none.
        let mut gentle = prop.clone();
        gentle.positive[2] = HingePoint::new(1900.0, 0.4);
        gentle.positive[3] = HingePoint::new(1900.0, 0.5);
        let bb = build_backbone(&gentle, 1, Some(k)).unwrap();
        assert!((bb.segs[1].slope - (1900.0 - 1920.0) / 0.36).abs() < 1e-9);
        assert!((bb.segs[1].q1 - 0.4).abs() < 1e-15);
    }

    #[test]
    fn backbone_refuses_strength_loss_of_other_hinges_and_handles_beyond_e() {
        let mut shear = HingeProp::symmetric(
            "V",
            HingeDof::V2,
            [
                HingePoint::new(70.0, 0.0),
                HingePoint::new(80.0, 0.4),
                HingePoint::new(60.0, 0.8),
                HingePoint::new(60.0, 1.2),
            ],
        );
        assert!(build_backbone(&shear, 1, None)
            .unwrap_err()
            .contains("bending"));
        shear.positive[2].force = 90.0;
        shear.positive[3].force = 100.0;
        let bb = build_backbone(&shear, 1, None).unwrap();
        assert!(
            bb.refuse_at_e.is_some(),
            "a drop beyond E is refused for a shear hinge at run time"
        );
        shear.beyond_e = Some(HingeBeyondE::Extrapolated);
        assert!(build_backbone(&shear, 1, None)
            .unwrap()
            .refuse_at_e
            .is_none());
        shear.beyond_e = None;
        assert!(build_backbone(&shear, 1, None)
            .unwrap()
            .refuse_at_e
            .is_some());
        // Extrapolating a falling slope is not implemented.
        let mut bending = HingeProp::symmetric(
            "M",
            HingeDof::M3,
            [
                HingePoint::new(10.0, 0.0),
                HingePoint::new(12.0, 1.0),
                HingePoint::new(11.0, 2.0),
                HingePoint::new(10.5, 3.0),
            ],
        );
        bending.beyond_e = Some(HingeBeyondE::Extrapolated);
        assert!(build_backbone(&bending, 1, Some(100.0))
            .unwrap_err()
            .contains("negative slope"));
    }

    #[test]
    fn the_negative_branch_uses_its_own_points_and_scale_factors() {
        let mut p = HingeProp::symmetric(
            "A",
            HingeDof::M3,
            [
                HingePoint::new(10.0, 0.0),
                HingePoint::new(12.0, 1.0),
                HingePoint::new(12.0, 2.0),
                HingePoint::new(12.0, 3.0),
            ],
        );
        p.symmetric = false;
        p.negative = [
            HingePoint::new(-5.0, -0.5),
            HingePoint::new(-6.0, -1.5),
            HingePoint::new(-6.0, -2.5),
            HingePoint::new(-6.0, -3.5),
        ];
        p.force_sf = 2.0;
        p.force_sf_negative = 3.0;
        p.deformation_sf = 1.0;
        p.deformation_sf_negative = 10.0;
        let pos = build_backbone(&p, 1, Some(1.0)).unwrap();
        let neg = build_backbone(&p, -1, Some(1.0)).unwrap();
        assert_eq!((pos.force_b(), pos.segs[0].q1), (20.0, 1.0));
        // B deformation (0.5 in the table) is subtracted: C is 1.0 * 10 beyond B.
        assert_eq!((neg.force_b(), neg.segs[0].q1), (15.0, 10.0));
        assert_eq!(neg.force_at(10.0), 18.0);
    }

    #[test]
    fn routing_branches_of_the_hinge_path() {
        let mut model = SapModel::default();
        model.load_cases.clear();
        let mut lc = LoadCase::linear_static("PUSH", "DEAD", 1.0);
        lc.case_type = LoadCaseType::NonlinearStatic;
        model.load_cases.insert("PUSH".into(), lc.clone());
        // No hinges, final state only: the existing path.
        assert!(!applies(&model, &lc, false));
        // A model with a hinge.
        assert!(applies(&model, &lc, true));
        // Multiple states without other nonlinearity.
        let mut multi = lc.clone();
        multi.nl_static.results_saved = ResultsSaved::multiple_default();
        assert!(applies(&model, &multi, false));
        // Multiple states with geometric nonlinearity (or a link, limit, cable): existing path.
        let mut pdelta = multi.clone();
        pdelta.nl_geom = NLGeomType::PDelta;
        assert!(!applies(&model, &pdelta, false));
        // A Mode load in the chain, also of an earlier case.
        let mut modal = lc.clone();
        modal.nl_static.mode_loads.push(sap_model::hinge::ModeLoad {
            mode: 1,
            scale_factor: 1.0,
        });
        assert!(applies(&model, &modal, false));
        let mut accel = lc.clone();
        accel.loads[0].load_name = "UX".into();
        assert!(applies(&model, &accel, false));
        model.load_cases.insert("MODAL-LOAD".into(), modal);
        let mut next = lc.clone();
        next.name = "NEXT".into();
        next.initial_case = Some("MODAL-LOAD".into());
        assert!(applies(&model, &next, false));
    }

    #[test]
    fn the_scaled_solver_detects_singular_systems_whatever_the_units() {
        // Well conditioned in different units: rows and columns scaled by 1e9.
        let a = vec![2.0e9, 1.0e9, 1.0e12, 3.0e12];
        let x = solve_scaled(a, vec![4.0e9, 5.0e12], 2).unwrap();
        assert!(
            (x[0] - 1.4).abs() < 1e-9 && (x[1] - 1.2).abs() < 1e-9,
            "{x:?}"
        );
        // Singular whatever the scaling.
        assert!(solve_scaled(vec![1.0e12, 1.0e12, 1.0, 1.0], vec![1.0, 1.0], 2).is_none());
        assert!(solve_scaled(vec![0.0, 0.0, 1.0, 1.0], vec![1.0, 1.0], 2).is_none());
    }

    #[test]
    fn null_steps_are_counted_and_limited() {
        // Two parallel hardening hinges whose yield loads differ by 3e-9 relative: not
        // lumped (the tie is 1e-9), and the second step is shorter than 1e-12 of the target.
        let mut prop = HingeProp::symmetric(
            "H",
            HingeDof::M3,
            [
                HingePoint::new(1.0, 0.0),
                HingePoint::new(2.0, 1.0),
                HingePoint::new(3.0, 2.0),
                HingePoint::new(4.0, 3.0),
            ],
        );
        prop.beyond_e = Some(HingeBeyondE::Extrapolated);
        let curve = Curve {
            pos: build_backbone(&prop, 1, Some(1.0)).unwrap(),
            neg: build_backbone(&prop, -1, Some(1.0)).unwrap(),
        };
        let problem = Problem {
            curves: vec![curve.clone(), curve],
            phi: vec![0.0; 4],
            g: vec![1.0, 1.0 + 3.0e-9],
            f0: vec![0.0, 0.0],
            c: 1.0,
            w: vec![1.0, 1.0],
            c_conj: 1.0,
            w_conj: vec![1.0, 1.0],
        };
        let start = State {
            lambda: 0.0,
            q: vec![0.0; 2],
            dir: vec![0; 2],
        };
        let spec = |nulls| RunSpec {
            control: Control::Monitored { conjugate: false },
            target: 1.0e4,
            save: None,
            max_total_steps: 100,
            max_null_steps: nulls,
        };
        match run(&problem, &start, &spec(0)) {
            Err(StepError::Budget { reason, .. }) => {
                assert!(reason.contains("null steps"), "{reason}")
            }
            other => panic!("expected the null step budget to be exceeded, got {other:?}"),
        }
        run(&problem, &start, &spec(3)).expect("runs with a null step allowed");
        match run(
            &problem,
            &start,
            &RunSpec {
                max_total_steps: 1,
                ..spec(3)
            },
        ) {
            Err(StepError::Budget { reason, steps, .. }) => {
                assert!(reason.contains("total steps"), "{reason}");
                assert_eq!(steps, 1);
            }
            other => panic!("expected the total step budget to be exceeded, got {other:?}"),
        }
    }

    #[test]
    fn a_displacement_that_does_not_move_with_the_load_is_refused() {
        let mut problem = example_1_026(A_LIMIT);
        problem.c = 0.0;
        problem.w = vec![0.0, 0.0];
        let start = State {
            lambda: 0.0,
            q: vec![0.0; 2],
            dir: vec![0; 2],
        };
        let spec = RunSpec {
            control: Control::Monitored { conjugate: false },
            target: 1.0,
            save: None,
            max_total_steps: 10,
            max_null_steps: 10,
        };
        assert_eq!(
            run(&problem, &start, &spec).unwrap_err(),
            StepError::NoMotion
        );
    }

    #[test]
    fn conjugate_control_gives_the_same_states_when_the_measures_coincide() {
        for limit in [A_LIMIT, B_LIMIT] {
            let (pa, a) = run_1_026(limit, false, false);
            let (pb, b) = run_1_026(limit, true, false);
            let (sa, sb) = (schedule(&pa, &a), schedule(&pb, &b));
            assert_eq!(sa.len(), sb.len());
            for (x, y) in sa.iter().zip(&sb) {
                assert!((x - y).abs() <= 1e-12, "{sa:?} vs {sb:?}");
            }
        }
    }
}
