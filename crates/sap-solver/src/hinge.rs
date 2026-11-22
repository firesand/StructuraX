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
use std::rc::Rc;
use sap_model::hinge::{
    generated_name, FrameHingeResult, HingeBehavior, HingeBeyondE, HingeDof, HingeProp, HingeState,
    HingeStatus, HingeUnloading, NonlinearStaticStep, ResultsSaved,
};
use sap_model::{AreaObj, CaseResults, CaseStatus, LoadCase, LoadCaseType, NLGeomType, PointObj, SapModel};

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
    non_geometric_nonlinearity(model) || lc.nl_geom != NLGeomType::None
}

/// Cables, tension/compression limits and nonlinear links: the nonlinearity
/// other than frame hinges and geometry.
fn non_geometric_nonlinearity(model: &SapModel) -> bool {
    model
        .frames
        .values()
        .any(|f| f.cable_params.is_some() || f.tc_limits.is_some_and(|(c, _, t, _)| c || t))
        || model.links.values().any(|object| {
            model.link_props.get(&object.prop).is_some_and(|prop| {
                (0..6).any(|d| prop.dof_nonlinear[d] && prop.dof_active[d] && !prop.dof_fixed[d])
            })
        })
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
/// history, but nothing here may integrate a hinged structure directly, and a
/// buckling case may not continue from a hinged case. A Modal, linear static
/// or multi-step static case that continues from a nonlinear static case is
/// allowed here; whether the hinges of that case permit it is decided when the
/// case runs, because it depends on the state the hinges ended in
/// ([`check_end_state_without_yield`]).
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
        // Buckling takes the forces of its initial case as its load, which is
        // not the "Stiffness to Use" reading of the help; not probed.
        if lc.case_type == LoadCaseType::Buckling {
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

/// The run-time half of the rule for a Modal, linear static or multi-step
/// static case `case` that uses the stiffness at the end of nonlinear static
/// case `initial` of a model with frame hinges.
///
/// Allowed (a): no hinge has yielded at the end of `initial`. The hinges then
/// contribute nothing: they are rigid (v19 help, "Hinge Properties": "For all
/// other types of analysis, the hinges are rigid"), so the stiffness is the
/// elastic one. Refused, each with the hinge named by its generated name:
/// (b) a hinge on its backbone curve at the end, (c) a hinge that yielded and
/// unloaded. The manuals say nothing of what a yielded hinge contributes
/// (v19 Analysis Reference p. 133 only says the nonlinear property is used "as
/// it existed at the end of the nonlinear case"; for (c) the reading of
/// p. 151 and p. 402 gives rigid, but no statement covers hinges).
///
/// The decision rests on the hinge state that the initial case recorded, not
/// on the hinge assignments of `model`: the cases run on the meshed model,
/// where the pieces of an automatically subdivided frame carry no hinge
/// assignment (`frame_mesh::expand` moves the hinges into its sites), while
/// the recorded hinge results name every hinge of the objects. (Review of
/// 2026-11-18: with the assignments as the gate, a frame cut into two
/// elements let a case continue from a yielded hinge with the elastic
/// stiffness.)
///
/// Does nothing for an initial case that is not a nonlinear static one, that
/// has no results, or that recorded no hinge (a model without hinges).
pub(crate) fn check_end_state_without_yield(
    model: &SapModel,
    case: &str,
    initial: &str,
) -> Result<(), SolverError> {
    if model
        .load_cases
        .get(initial)
        .is_none_or(|c| c.case_type != LoadCaseType::NonlinearStatic)
    {
        return Ok(());
    }
    let Some(results) = model.case_results.get(initial) else {
        return Ok(());
    };
    for r in &results.hinge_results {
        if r.direction == 0 && r.plastic_deformation == 0.0 {
            continue;
        }
        let on_backbone = on_backbone(model, r);
        let tail = "; the stiffness of a yielded hinge is not implemented (a case may use the stiffness at the end of a hinged nonlinear static case only when no hinge has yielded)";
        return Err(SolverError::UnsupportedInput(if on_backbone {
            format!(
                "case '{case}' uses the stiffness at the end of nonlinear static case '{initial}', where hinge {} (property '{}', frame '{}') has yielded and is on its backbone curve (plastic deformation {:e}, force {:e}){tail}",
                r.name, r.prop, r.frame, r.plastic_deformation, r.force
            )
        } else {
            format!(
                "case '{case}' uses the stiffness at the end of nonlinear static case '{initial}', where hinge {} (property '{}', frame '{}') yielded and then unloaded, so it is below its backbone curve (plastic deformation {:e}, force {:e}){tail}",
                r.name, r.prop, r.frame, r.plastic_deformation, r.force
            )
        }));
    }
    Ok(())
}

/// Whether a yielded hinge ended on its backbone curve (force equal to the
/// backbone force at its plastic deformation) rather than below it after
/// unloading. The slope limit of strength loss of a bending hinge depends on
/// the stiffness of the element that holds the hinge, which is taken here
/// from the frame object and from every piece of it in the meshed model; a
/// hinge that matches the curve of any of them counts as on its backbone
/// (only the wording of the refusal depends on it). A hinge whose curve
/// cannot be built counts as on its backbone.
fn on_backbone(model: &SapModel, r: &FrameHingeResult) -> bool {
    let Some(prop) = model.hinge_props.get(&r.prop) else {
        return true;
    };
    let dir: i8 = if r.direction < 0 { -1 } else { 1 };
    let m = r.plastic_deformation.abs();
    // The slope limit of a bending hinge comes from the element that holds
    // it: the object itself, or, in the meshed model, one of its pieces
    // (named by `frame_mesh::expand`). Every one of them is a candidate.
    let piece_prefix = format!("{}__seg_", r.frame);
    let mut candidates: Vec<Option<f64>> = vec![None];
    for frame in model
        .frames
        .values()
        .filter(|f| f.name == r.frame || f.name.starts_with(&piece_prefix))
    {
        let (Some(pi), Some(pj), Some(sec)) = (
            model.points.get(&frame.point_i),
            model.points.get(&frame.point_j),
            model.frame_sections.get(&frame.section),
        ) else {
            continue;
        };
        let Some(mat) = model.materials.get(&sec.material) else {
            continue;
        };
        let length = ((pj.x - pi.x).powi(2) + (pj.y - pi.y).powi(2) + (pj.z - pi.z).powi(2)).sqrt();
        if length > 0.0 {
            let section = crate::frame::eval_section_props_at(model, frame, sec, mat, 0.5, length);
            candidates.push(match prop.dof {
                HingeDof::M3 => Some(0.1 * mat.e * section.i33 / length),
                HingeDof::M2 => Some(0.1 * mat.e * section.i22 / length),
                _ => None,
            });
        }
    }
    let mut built = false;
    for limit in candidates {
        if let Ok(bb) = build_backbone(prop, dir, limit) {
            built = true;
            if (r.force.abs() - bb.force_at(m)).abs() <= 1e-6 * bb.force_scale().max(1e-300) {
                return true;
            }
        }
    }
    !built
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

#[derive(Debug, Clone, PartialEq)]
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
    /// Why the run ended before its target (G16): `last` and `saved` are then the
    /// states reached. `None` when the target was reached.
    pub stop: Option<Stop>,
}

/// An early end of a run that keeps what it reached: the case is "not finished" (v19
/// "Set Load Cases to Run"). Steps that run out and a path with no admissible state are
/// such ends; a refusal of behaviour that is not implemented (reversal, point E, no
/// motion, an internal error) is a [`StepError`] and keeps nothing.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Stop {
    pub reason: String,
    /// Steps taken.
    pub steps: usize,
    /// Progress reached, in the units of the target.
    pub progress: f64,
}

impl Stop {
    /// The early ends among the stepper failures; the others are returned unchanged.
    fn of(e: StepError) -> Result<Stop, StepError> {
        match e {
            StepError::Stuck {
                reason,
                steps,
                progress,
            }
            | StepError::Budget {
                reason,
                steps,
                progress,
            } => Ok(Stop {
                reason,
                steps,
                progress,
            }),
            other => Err(other),
        }
    }
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

/// An event that can end a step. Times are in units of the stepping measure.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Ev {
    /// The active hinge reaches the end of its current segment.
    SegEnd(usize),
    /// The virgin hinge reaches its yield force of the given sign.
    Yield(usize, i8),
    /// The yielded, rigid hinge returns to its backbone.
    Reload(usize),
    /// The yielded, rigid hinge unloads through zero force.
    Reverse(usize),
    Target,
    Grid,
}

/// Counters of a run, shared by its steps.
struct Counters {
    steps: usize,
    nulls: usize,
}

/// What one step found: the candidate end state and the events that end it.
/// Nothing in it has been accepted yet (see [`Stepper::accept`]).
///
/// `sol`, `tau` and `event` are read by the P-Delta correction of the end
/// state; without P-Delta a step is accepted as it stands.
struct Step {
    /// The state at the end of the step, before the events are processed
    /// (the yield, segment-end and reload adjustments are not yet applied).
    cand: State,
    /// The active set during the step.
    active: Vec<bool>,
    /// The classification of the hinges at the start of the step.
    cls: Classes,
    /// The rates of the step.
    sol: Sol,
    /// Length of the step in units of the stepping measure.
    tau: f64,
    /// The terminating event: the first of the earliest ones.
    event: Ev,
    /// Every event whose time ties with the earliest (the terminating event
    /// included), in the order they were found.
    tied: Vec<Ev>,
}

/// What accepting a step as it stands leaves.
struct Accepted {
    state: State,
    next_active: Vec<bool>,
    hit_target: bool,
    hit_grid: bool,
    hit_event: bool,
}

impl Stepper<'_> {
    /// One step of the stepper from the accepted state `st`, which is
    /// classified in place (yield flags). Finds the active set, the earliest
    /// event and the candidate end state. Counts total and null steps and
    /// stops at their limits.
    fn step_once(
        &self,
        st: &mut State,
        active: &mut [bool],
        plan: Option<&SavePlan>,
        spec: &RunSpec,
        p_now: f64,
        counters: &mut Counters,
    ) -> Result<Step, StepError> {
        let problem = self.p;
        let n = self.n;
        let target = spec.target;
        let f = problem.forces(st);
        let cls = self.classify(st, active, &f)?;
        counters.steps += 1;
        if counters.steps > spec.max_total_steps {
            return Err(StepError::Budget {
                reason: format!(
                    "the maximum number of total steps ({}) was reached; raise it in the case's nonlinear parameters",
                    spec.max_total_steps
                ),
                steps: counters.steps - 1,
                progress: p_now,
            });
        }
        let steps = counters.steps;
        let cand: Vec<bool> = (0..n).map(|h| cls.on_backbone[h]).collect();
        // Forward first; with displacement control a step that moves the
        // monitored displacement back is accepted when strength loss drives
        // it (project reconstruction of "unload entire structure").
        let mut found = self.search(st, &cls, active, &cand, 1.0, false);
        if found.is_none() && matches!(spec.control, Control::Monitored { .. }) {
            found = self.search(st, &cls, active, &cand, -1.0, true);
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
            if let Some(plan) = plan {
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
        let progress_scale = if sol.dp != 0.0 { target / sol.dp.abs() } else { 0.0 };
        if tau <= 1e-12 * progress_scale {
            counters.nulls += 1;
            if counters.nulls > spec.max_null_steps {
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
        // Advance to the earliest event; the events tied with it are those
        // within the tie tolerance.
        let mut end = st.clone();
        end.lambda += tau * sol.dlam;
        for h in 0..n {
            end.q[h] += tau * sol.dq[h];
        }
        let limit = tau * (1.0 + EVENT_TIE) + 1e-13 * progress_scale;
        let event = evs
            .iter()
            .find(|(t, _)| *t == tau)
            .map_or(evs[0].1, |(_, e)| *e);
        let tied: Vec<Ev> = evs
            .iter()
            .filter(|(t, _)| *t <= limit)
            .map(|(_, e)| *e)
            .collect();
        Ok(Step {
            cand: end,
            active: a,
            cls,
            sol,
            tau,
            event,
            tied,
        })
    }

    /// Accepts a step: processes the events `tied` with the earliest on the
    /// end state `end` (the candidate end state `step.cand` and every tied
    /// event `step.tied` when the step is accepted as it stands; the state
    /// corrected for P-Delta and the tied events that still hold there
    /// otherwise), and returns the new state and active set with what the
    /// step reached.
    fn accept(&self, step: &Step, end: &State, tied: &[Ev]) -> Result<Accepted, StepError> {
        let problem = self.p;
        let mut st = end.clone();
        let mut next_active = step.active.clone();
        let mut hit_target = false;
        let mut hit_grid = false;
        let mut hit_event = false;
        for ev in tied {
            match *ev {
                Ev::SegEnd(h) => {
                    let d = f64::from(st.dir[h]);
                    let bb = problem.curves[h].bb(st.dir[h]);
                    let seg = &bb.segs[bb.seg_at(step.cls.m[h])];
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
        Ok(Accepted {
            state: st,
            next_active,
            hit_target,
            hit_grid,
            hit_event,
        })
    }
}


/// The force and deformation scales of a set of hinge curves: the largest
/// force and the largest plastic deformation at a breakpoint (at least 1e-3).
fn curve_scales(curves: &[Curve]) -> (f64, f64) {
    let f_ref = curves
        .iter()
        .map(|c| c.pos.force_scale().max(c.neg.force_scale()))
        .fold(0.0f64, f64::max);
    let q_ref = curves
        .iter()
        .flat_map(|c| [&c.pos, &c.neg])
        .map(|b| b.scale)
        .fold(0.0f64, f64::max)
        .max(1e-3);
    (f_ref, q_ref)
}

/// The direction of loading under displacement control: the sign of the rate
/// at which the monitored displacement moves as the load grows from `st`. The
/// stepper's own control is restored; `active` is left as classified.
fn loading_direction(
    stepper: &mut Stepper,
    problem: &Problem,
    st: &State,
    active: &mut [bool],
) -> Result<f64, StepError> {
    let n = problem.curves.len();
    let f = problem.forces(st);
    let mut probe = st.clone();
    let cls = stepper.classify(&mut probe, active, &f)?;
    let cand: Vec<bool> = cls.on_backbone.clone();
    let saved_control = stepper.control;
    stepper.control = Control::Load;
    let first = stepper.search(&probe, &cls, active, &cand, 1.0, false);
    stepper.control = saved_control;
    let (rate, scale) = match first {
        Some((_, sol)) => {
            let direct = problem.c * sol.dlam;
            (0..n).fold((direct, direct.abs()), |(sum, magnitude), j| {
                let contribution = problem.w[j] * sol.dq[j];
                (sum + contribution, magnitude + contribution.abs())
            })
        }
        None => (problem.c, problem.c.abs()),
    };
    if !rate.is_finite() || rate.abs() <= 1e-12 * scale || rate == 0.0 {
        return Err(StepError::NoMotion);
    }
    Ok(rate.signum())
}

/// Whether the state just reached (at progress `p_new`) is saved, and the
/// bookkeeping of the plan when it is.
fn save_decision(
    plan: &mut Option<SavePlan>,
    p_new: f64,
    target: f64,
    hit_target: bool,
    hit_grid: bool,
    hit_event: bool,
) -> bool {
    let Some(plan) = plan.as_mut() else {
        return false;
    };
    if hit_target || p_new >= target * (1.0 - 1e-12) {
        if plan.needs_final() {
            plan.record(p_new);
            return true;
        }
    } else if hit_grid || (hit_event && plan.event_wanted(p_new)) {
        plan.record(p_new);
        return true;
    }
    false
}

/// Advances the state of the structure along the load path of one case.
///
/// The path is followed event by event: between events the active hinges
/// have a constant slope and the response is linear. Events are a hinge
/// yielding, reaching the end of a segment of its curve, returning to its
/// backbone (reload), the next saved state and the target. Event lumping is
/// exact (the vendor lumps events within one percent; project choice).
///
/// The problem is the same at every state; a case with P-Delta, whose problem
/// changes with the axial forces, is followed by `CaseSetup::run_pdelta`.
pub(crate) fn run(
    problem: &Problem,
    start: &State,
    spec: &RunSpec,
) -> Result<RunOutcome, StepError> {
    let n = problem.curves.len();
    let (f_ref, q_ref) = curve_scales(&problem.curves);
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
        stepper.s_target = loading_direction(&mut stepper, problem, &st, &mut active)?;
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
    let mut counters = Counters { steps: 0, nulls: 0 };
    let mut stop = None;

    loop {
        let p_now = progress(&st);
        if p_now >= target * (1.0 - 1e-12) {
            break;
        }
        let step = match stepper.step_once(
            &mut st,
            &mut active,
            plan.as_ref(),
            spec,
            p_now,
            &mut counters,
        ) {
            Ok(step) => step,
            Err(e) => {
                stop = Some(Stop::of(e)?);
                break;
            }
        };
        let accepted = stepper.accept(&step, &step.cand, &step.tied)?;
        st = accepted.state;
        active = accepted.next_active;
        let p_new = progress(&st);
        if save_decision(
            &mut plan,
            p_new,
            target,
            accepted.hit_target,
            accepted.hit_grid,
            accepted.hit_event,
        ) {
            saved.push(st.clone());
        }
    }
    Ok(RunOutcome { saved, last: st, stop })
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

/// The rigid rotation of a whole element about end I in the plane of a
/// flexural hinge, by one unit: the rotation at both ends and the transverse
/// displacement `L * 1` it gives end J (local 2 for M3; local 3 for M2, with the
/// opposite sense about local 2). Other hinge degrees of freedom have no
/// rotation here: `None`.
fn rigid_rotation(dof: HingeDof, length: f64) -> Option<[f64; 12]> {
    let mut r = [0.0; 12];
    match dof {
        HingeDof::M3 => {
            r[5] = 1.0;
            r[11] = 1.0;
            r[7] = length;
        }
        HingeDof::M2 => {
            r[4] = 1.0;
            r[10] = 1.0;
            r[8] = -length;
        }
        _ => return None,
    }
    Some(r)
}

/// The jump of a flexural hinge at end I of an element that has a geometric
/// stiffness, `J - R` with `R` the rigid rotation of the whole element that has
/// the same end-J values as `J` (`dislocation` at `x = 0` is `R` plus a pure
/// kink of the end-I rotation). The elastic stiffness annihilates `R`, so
/// `K_e (J - R) = K_e J`; the geometric stiffness does not (`K_g R` is the P-Delta
/// shear pair of the tilt), and the part beyond the hinge is a body that tilts
/// with the hinge rotation, which is what `K_g` must not be asked to undo.
/// Internal sense.
fn kink_vector(dof: HingeDof, length: f64) -> [f64; 12] {
    let mut jk = dislocation(dof, length, 0.0);
    if let Some(r) = rigid_rotation(dof, length) {
        for (a, b) in jk.iter_mut().zip(&r) {
            *a -= b;
        }
    }
    jk
}

/// Adds to the station displacements of an element the deflection of a
/// flexural hinge of plastic rotation `q` at distance `x_h` from end I of an
/// element of length `length`.
///
/// The recovered stations hold the chord between the element-end displacements
/// plus the integral of the elastic curvature. A hinge adds a rotation `q`
/// (slope jump, `dislocation`: the part beyond the hinge moves by `q (L - x_h)`
/// at end J) that the elastic curvature does not contain; with the end values
/// kept, the missing deflection is `q (max(0, x - x_h) - (L - x_h) x / L)`,
/// in local 2 for M3 and in local 3 for M2 (whose dislocation carries the
/// opposite sense of rotation about local 2, see `Site::sign`).
fn add_hinge_kink(
    stations: &mut [sap_model::FrameStationResult],
    dof: HingeDof,
    q: f64,
    x_h: f64,
    length: f64,
) {
    let axis = match dof {
        HingeDof::M3 => 1,
        HingeDof::M2 => 2,
        _ => return,
    };
    if q == 0.0 || length <= 0.0 {
        return;
    }
    for s in stations {
        let x = s.station;
        s.local_disp[axis] += q * ((x - x_h).max(0.0) - (length - x_h) * x / length);
    }
}

/// A hinge within this relative distance of an element end is at the end.
const END_TOL: f64 = 1e-12;

/// The overwrite "do not allow hinges to drop loads" changes nothing for a
/// curve that never drops; for a curve that does, the vendor material does not
/// state what the curve after the drop is, so it is refused.
fn check_never_drops(prop: &HingeProp, curve: &Curve, frame: &str) -> Result<(), SolverError> {
    for backbone in [&curve.pos, &curve.neg] {
        let drops = backbone.refuse_at_e.is_some()
            || prop.beyond_e != Some(HingeBeyondE::Extrapolated)
            || backbone.segs.iter().any(|s| s.slope < 0.0);
        if drops {
            return Err(unsupported(format!(
                "frame '{frame}' has the hinge overwrite \"do not allow hinges to drop loads\" and its hinge property '{}' drops in force (a negative slope, or a capacity that falls to zero or is not stated beyond point E); the curve after a drop that the overwrite would give is not stated in the vendor material, so it is not implemented",
                prop.name
            )));
        }
    }
    Ok(())
}

fn unsupported(msg: String) -> SolverError {
    SolverError::UnsupportedInput(msg)
}

/// Refusals for an element that carries a hinge.
///
/// `p_delta_case` is true when the case has P-Delta: every element then has a
/// geometric stiffness, as if it carried a directly specified P-Delta force.
fn check_element(
    model: &SapModel,
    site: &frame_mesh::HingeSite,
    p_delta_case: bool,
) -> Result<(), SolverError> {
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
    if (frame.pdelta_force != 0.0 || p_delta_case)
        && site.rel > END_TOL
        && site.rel < 1.0 - END_TOL
    {
        return Err(unsupported(format!(
            "frame '{object}' has a hinge inside its analysis element at relative distance {} together with a geometric stiffness ({}); the P-Delta of a hinge inside an element is not implemented",
            site.rel_dist,
            if p_delta_case {
                "a P-Delta case"
            } else {
                "a P-Delta force"
            }
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

/// What a case needs that does not depend on the P-Delta axial forces: the
/// loads of the chain and of the case, the restraints, and the hinges with
/// their curves. Made once by [`CaseSetup::new`], which also holds every
/// refusal that does not depend on the axial forces.
struct CaseSetup<'a> {
    model: &'a SapModel,
    lc: &'a LoadCase,
    work_points: &'a IndexMap<String, PointObj>,
    work_areas: &'a IndexMap<String, AreaObj>,
    orig_area_map: &'a IndexMap<String, Vec<String>>,
    pt_index: &'a IndexMap<String, usize>,
    joint_rot: &'a [[[f64; 3]; 3]],
    k_struct: &'a [f64],
    k_base: &'a [f64],
    mesh: &'a frame_mesh::FrameMesh,
    ndof: usize,
    /// What the chain of earlier cases left on the structure.
    chain_loads: Vec<(String, f64)>,
    chain_special: Option<Vec<f64>>,
    /// The case's own loads at unit load factor.
    own_loads: Vec<(String, f64)>,
    own_special: Option<Vec<f64>>,
    /// Monitored degree of freedom and the control of the case.
    monitor_spec: Option<(usize, &'a sap_model::DisplacementControl)>,
    is_fixed: Vec<bool>,
    /// One per hinge site of the mesh, in the order of `mesh.hinge_sites`.
    curves: Vec<Curve>,
    props: Vec<&'a HingeProp>,
}

/// The problem of the hinges for one set of P-Delta axial forces, with what
/// the recovery of a state needs. See [`CaseSetup::build`].
struct Built {
    /// The axial forces (tension positive, by analysis frame element) the
    /// problem was built for; empty when none are given.
    axial: IndexMap<String, f64>,
    elems: Vec<Elem>,
    sites: Vec<Site>,
    problem: Problem,
    /// Displacement of the chain loads, `u_chain(N)`, and per unit load
    /// factor of the case.
    u_chain: Vec<f64>,
    u_own: Vec<f64>,
    /// Displacement field of a unit jump at each hinge.
    u_hinge: Vec<Vec<f64>>,
    /// Monitored component of `u_chain(N)` (`c0` of the P-Delta algorithm;
    /// the absolute monitored displacement is `c0 + c lambda + w.q`).
    /// Zero without displacement control.
    monitored_chain: f64,
    /// The structural stiffness with the geometric stiffness of `axial`, for
    /// the reactions; `None` when there are no axial forces (the case's own
    /// `k_struct` applies).
    k_struct_n: Option<Vec<f64>>,
}

impl<'a> CaseSetup<'a> {
    #[allow(clippy::too_many_arguments)]
    fn new(
        model: &'a SapModel,
        lc: &'a LoadCase,
        work_points: &'a IndexMap<String, PointObj>,
        work_areas: &'a IndexMap<String, AreaObj>,
        orig_area_map: &'a IndexMap<String, Vec<String>>,
        pt_index: &'a IndexMap<String, usize>,
        joint_rot: &'a [[[f64; 3]; 3]],
        k_struct: &'a [f64],
        k_base: &'a [f64],
        diag_mass: &[f64],
        mesh: &'a frame_mesh::FrameMesh,
        ndof: usize,
    ) -> Result<Self, SolverError> {
        let chain = static_chain(model, lc);
        let p_delta_case = lc.nl_geom == NLGeomType::PDelta;
        for c in &chain {
            if non_geometric_nonlinearity(model) || c.nl_geom == NLGeomType::LargeDisplacements {
                return Err(unsupported(if c.nl_geom == NLGeomType::None {
                    format!(
                        "case '{}': saving multiple states, Mode and Accel loads and frame hinges are implemented for cases whose only nonlinearity is frame hinges; the model or the case has a nonlinear link, a tension/compression limit, a cable or geometric nonlinearity",
                        c.name
                    )
                } else {
                    format!(
                        "case '{}': saving multiple states, Mode and Accel loads and frame hinges are implemented for cases whose only nonlinearity is frame hinges (P-Delta is the one geometric nonlinearity allowed with them); the model or the case has a nonlinear link, a tension/compression limit, a cable or large displacements",
                        c.name
                    )
                }));
            }
            if c.nl_geom != lc.nl_geom {
                return Err(unsupported(format!(
                    "case '{}' has geometric nonlinearity {:?} but the case '{}' of its chain has {:?}; every case of a chain of nonlinear static cases with frame hinges must have the same setting (the vendor recommends it, and the stiffness of the carried state would not be that of the case)",
                    lc.name, lc.nl_geom, c.name, c.nl_geom
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
        if p_delta_case {
            if let Some(frame) = model.frames.values().find(|f| f.pdelta_force != 0.0) {
                return Err(unsupported(format!(
                    "case '{}' has P-Delta and frame '{}' has a directly specified P-Delta force ({}); a directly specified force together with a P-Delta case is not implemented (the vendor adds it to the force of the case)",
                    lc.name, frame.name, frame.pdelta_force
                )));
            }
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

        // Refusals that look at the assembled loads (the loads themselves do
        // not depend on the axial forces).
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
        let mut curves: Vec<Curve> = Vec::new();
        let mut props: Vec<&HingeProp> = Vec::new();
        let mut seen: Vec<(&str, HingeDof, f64)> = Vec::new();
        for hs in &mesh.hinge_sites {
            check_element(model, hs, p_delta_case)?;
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
            if seen
                .iter()
                .any(|(f, d, r)| *f == hs.frame && *d == prop.dof && (r - hs.rel_dist).abs() <= 1e-9)
            {
                return Err(unsupported(format!(
                    "frame '{}' has two hinges of the same degree of freedom ({}) at relative distance {}",
                    hs.frame,
                    prop.dof.label(),
                    hs.rel_dist
                )));
            }
            let length = base.frame_fef_map[&hs.element].3;
            let frame = &model.frames[&hs.element];
            let sec = &model.frame_sections[&frame.section];
            let mat = &model.materials[&sec.material];
            // Slope limit of strength loss: 0.1 EI/L of the element that holds the
            // hinge (see `build_backbone`).
            let section = crate::frame::eval_section_props_at(model, frame, sec, mat, 0.5, length);
            let limit = match prop.dof {
                HingeDof::M3 => Some(0.1 * mat.e * section.i33 / length),
                HingeDof::M2 => Some(0.1 * mat.e * section.i22 / length),
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
            if hs.do_not_drop_load {
                check_never_drops(prop, &curves[curves.len() - 1], &hs.frame)?;
            }
            seen.push((hs.frame.as_str(), prop.dof, hs.rel_dist));
            props.push(prop);
        }
        Ok(CaseSetup {
            model,
            lc,
            work_points,
            work_areas,
            orig_area_map,
            pt_index,
            joint_rot,
            k_struct,
            k_base,
            mesh,
            ndof,
            chain_loads,
            chain_special,
            own_loads,
            own_special,
            monitor_spec,
            is_fixed: base.is_fixed.clone(),
            curves,
            props,
        })
    }

    /// Static loads of the case's model for `loads`, with the axial forces
    /// `axial` in the element stiffness and fixed-end terms.
    fn assemble(
        &self,
        loads: &[(String, f64)],
        axial: &IndexMap<String, f64>,
    ) -> Result<crate::StaticLoads, SolverError> {
        crate::assemble_static_loads(
            self.model,
            loads,
            self.work_points,
            self.work_areas,
            self.pt_index,
            self.joint_rot,
            axial,
            self.ndof,
        )
    }

    /// The global stiffness and the structural reference stiffness for the
    /// axial forces `axial`: `k_base + K_g(N)` and `k_struct + K_g(N)`, with
    /// the geometric stiffness of every frame element
    /// (`assemble_global_geometric_stiffness`; no frame has a directly
    /// specified force in a P-Delta case, so nothing is subtracted). With no
    /// axial forces they are the matrices of the case as given.
    fn stiffness_for<'s>(
        &'s self,
        axial: &IndexMap<String, f64>,
    ) -> Result<(std::borrow::Cow<'s, [f64]>, std::borrow::Cow<'s, [f64]>), SolverError> {
        if axial.is_empty() {
            return Ok((
                std::borrow::Cow::Borrowed(self.k_base),
                std::borrow::Cow::Borrowed(self.k_struct),
            ));
        }
        let kg = crate::assemble_global_geometric_stiffness(
            self.model,
            self.work_points,
            self.pt_index,
            self.joint_rot,
            axial,
            true,
            self.ndof,
        )?;
        let add = |k: &[f64]| -> Vec<f64> { k.iter().zip(&kg).map(|(a, b)| a + b).collect() };
        Ok((
            std::borrow::Cow::Owned(add(self.k_base)),
            std::borrow::Cow::Owned(add(self.k_struct)),
        ))
    }

    /// Builds the problem of the hinges for the P-Delta axial forces `axial`
    /// (tension positive, one per analysis frame element; the map is empty
    /// when the case has no P-Delta, which gives the problem of a case with
    /// the `pdelta_force` of its frames only).
    ///
    /// The three places where the axial forces enter are marked:
    /// 1. [`Self::stiffness_for`], the stiffness that is factored;
    /// 2. the element stiffness at the hinge sites (`Elem::k`), taken here
    ///    from the assembled loads (`StaticLoads::frame_fef_map`), whose
    ///    stiffness already holds `K_g` of `axial` (or of `pdelta_force` for
    ///    a frame not in the map);
    /// 3. the loads `assemble` assembles with `axial`, used for the end
    ///    forces of the chain's and the case's displacements and for recovery.
    fn build(&self, axial: &IndexMap<String, f64>) -> Result<Built, SolverError> {
        let model = self.model;
        let lc = self.lc;
        let ndof = self.ndof;
        let base = self.assemble(&[], axial)?;
        let chain_sl = self.assemble(&self.chain_loads, axial)?;
        let own_sl = self.assemble(&self.own_loads, axial)?;

        let mut elems: Vec<Elem> = Vec::new();
        let mut elem_of: std::collections::HashMap<String, usize> =
            std::collections::HashMap::new();
        let mut sites: Vec<Site> = Vec::new();
        for (hs, prop) in self.mesh.hinge_sites.iter().zip(&self.props) {
            let e = match elem_of.get(&hs.element) {
                Some(&e) => e,
                None => {
                    let frame = &model.frames[&hs.element];
                    let (_, k, e_axes, length) = &base.frame_fef_map[&hs.element];
                    let (ii, ij) = (
                        self.pt_index[&frame.point_i],
                        self.pt_index[&frame.point_j],
                    );
                    elems.push(Elem {
                        name: hs.element.clone(),
                        ii,
                        ij,
                        ti: mat3_mul(e_axes, &self.joint_rot[ii]),
                        tj: mat3_mul(e_axes, &self.joint_rot[ij]),
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
            let x = hs.rel * elem.length;
            let sign = if prop.dof == HingeDof::M2 { -1.0 } else { 1.0 };
            // Under a geometric stiffness a flexural hinge at end I carries the
            // P-Delta of its own plastic rotation only if the rigid tilt of the
            // part beyond it is left out of the jump (`kink_vector`). Without one
            // the two are the same jump, and `J` is kept, bit for bit. The
            // axial force of the element is the one of the map, else its own
            // `pdelta_force`, as the assembly of the stiffness has it.
            let element_axial = axial.get(&hs.element).copied().unwrap_or(frame.pdelta_force);
            let mut jv = if element_axial != 0.0
                && hs.rel <= END_TOL
                && matches!(prop.dof, HingeDof::M2 | HingeDof::M3)
            {
                kink_vector(prop.dof, elem.length)
            } else {
                dislocation(prop.dof, elem.length, x)
            };
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
        let is_fixed = &self.is_fixed;
        let (k_model, k_struct) = self.stiffness_for(axial)?;
        let mut k_mod = k_model.to_vec();
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
        let ldlt =
            SymmetricLdlt::factorize_with_reference(&k_mod, ndof, Some(&structural_diagonal))?;
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
        let rhs_chain = with_special(&chain_sl.rhs, self.chain_special.as_ref());
        let rhs_own = with_special(&own_sl.rhs, self.own_special.as_ref());
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
            .map(|s| section_force(s, &f_end_chain[s.elem], &self.chain_loads))
            .collect();
        let g: Vec<f64> = sites
            .iter()
            .map(|s| section_force(s, &f_end_own[s.elem], &self.own_loads))
            .collect();

        // Unit jumps: displacement fields, hinge-force influence matrix and the
        // monitored and conjugate displacements.
        let mut phi = vec![0.0; n * n];
        let mut u_hinge: Vec<Vec<f64>> = Vec::with_capacity(n);
        let monitor = self.monitor_spec.as_ref().map(|(i, _)| *i);
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
        let monitored_chain = monitor.map_or(0.0, |m| u_chain[m]);
        let c_conj: f64 = rhs_own.iter().zip(&u_own).map(|(a, b)| a * b).sum();
        let problem = Problem {
            curves: self.curves.clone(),
            phi,
            g,
            f0,
            c,
            w,
            c_conj,
            w_conj,
        };
        Ok(Built {
            axial: axial.clone(),
            elems,
            sites,
            problem,
            u_chain,
            u_own,
            u_hinge,
            monitored_chain,
            k_struct_n: if axial.is_empty() {
                None
            } else {
                Some(k_struct.to_vec())
            },
        })
    }

    /// The state the hinges start the case in: as the initial case left them.
    /// The hinge forces the initial case carried are checked against the
    /// problem `built` with a tolerance of 1e-8 of the force scale.
    fn start_state(&self, built: &Built) -> Result<State, SolverError> {
        let (start, carried) = self.carried_state(&built.sites)?;
        self.check_continuation(built, &start, &carried, 1e-8)?;
        Ok(start)
    }

    /// The plastic jumps and directions of the hinges as the initial case left
    /// them, with the hinge forces it reported.
    fn carried_state(&self, sites: &[Site]) -> Result<(State, Vec<Option<f64>>), SolverError> {
        let lc = self.lc;
        let model = self.model;
        let n = sites.len();
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
        }
        Ok((start, carried))
    }

    /// Checks that the hinge forces of `start` under the problem `built` are
    /// the ones the initial case carried, to `tol` of the force scale.
    fn check_continuation(
        &self,
        built: &Built,
        start: &State,
        carried: &[Option<f64>],
        tol: f64,
    ) -> Result<(), SolverError> {
        let lc = self.lc;
        let problem = &built.problem;
        let sites = &built.sites;
        let n = sites.len();
        let Some(initial) = &lc.initial_case else {
            return Ok(());
        };
        let f_now = problem.forces(start);
        let scale = problem
            .curves
            .iter()
            .map(|c| c.pos.force_scale().max(c.neg.force_scale()))
            .fold(0.0f64, f64::max);
        for h in 0..n {
            if let Some(f) = carried[h] {
                if (f - f_now[h]).abs() > tol * scale.max(1e-300) {
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
        Ok(())
    }

    /// The control, target and saving of the case.
    fn run_spec(&self) -> Result<RunSpec, SolverError> {
        let lc = self.lc;
        let (control, target) = match self.monitor_spec {
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
        Ok(RunSpec {
            control,
            target,
            save,
            max_total_steps: lc.nl_static.max_total_steps,
            max_null_steps: lc.nl_static.max_null_steps,
        })
    }

    /// Results of the saved states and of the final state of a run.
    fn results(&self, built: &Built, outcome: &RunOutcome) -> Result<CaseResults, SolverError> {
        let mut saved: Vec<(&State, &Built)> = outcome.saved.iter().map(|s| (s, built)).collect();
        if outcome.stop.is_some() && saved.last().is_some_and(|(s, _)| **s != outcome.last) {
            saved.push((&outcome.last, built));
        }
        self.results_of(&saved, (&outcome.last, built), outcome.stop.as_ref())
    }

    /// Results of states each with the problem it is consistent with: the
    /// saved states in order, then the final state.
    fn results_of(
        &self,
        saved: &[(&State, &Built)],
        last: (&State, &Built),
        stop: Option<&Stop>,
    ) -> Result<CaseResults, SolverError> {
        let mut steps = Vec::with_capacity(saved.len());
        for (number, (state, built)) in saved.iter().enumerate() {
            let (r, hinges) = self.recover(built, state)?;
            let monitored = self
                .monitor_spec
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
        let (mut results, hinges) = self.recover(last.1, last.0)?;
        results.displacement_control_factor = Some(last.0.lambda);
        results.hinge_results = hinges;
        results.static_steps = steps;
        if let Some(stop) = stop {
            results.status = CaseStatus::NotFinished {
                reason: stop.reason.clone(),
                steps: results.static_steps.len(),
            };
        }
        Ok(results)
    }

    /// Results of one state, with the problem `built` (and so its axial
    /// forces, `built.axial`, in the loads assembled here; extension point
    /// 3 of [`Self::build`], and the place where the axial forces a frame
    /// station reports are the ones used).
    fn recover(
        &self,
        built: &Built,
        st: &State,
    ) -> Result<(CaseResults, Vec<FrameHingeResult>), SolverError> {
        let ndof = self.ndof;
        let (elems, sites, problem) = (&built.elems, &built.sites, &built.problem);
        let mut case_loads = self.chain_loads.clone();
        for (pattern, sf) in &self.own_loads {
            case_loads.push((pattern.clone(), st.lambda * sf));
        }
        let mut sl = self.assemble(&case_loads, &built.axial)?;
        for i in 0..ndof {
            if let Some(s) = &self.chain_special {
                sl.f_ext_joint[i] += s[i];
            }
            if let Some(s) = &self.own_special {
                sl.f_ext_joint[i] += st.lambda * s[i];
            }
        }
        let mut u: Vec<f64> = (0..ndof)
            .map(|i| built.u_chain[i] + st.lambda * built.u_own[i])
            .collect();
        for (h, site) in sites.iter().enumerate() {
            let q = st.q[h];
            if q == 0.0 {
                continue;
            }
            for (a, b) in u.iter_mut().zip(&built.u_hinge[h]) {
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
        let mut results = crate::recover_static_results(
            self.model,
            &self.lc.name,
            &case_loads,
            self.work_points,
            self.work_areas,
            self.orig_area_map,
            self.pt_index,
            self.joint_rot,
            built.k_struct_n.as_deref().unwrap_or(self.k_struct),
            Some((self.k_struct, self.k_base)),
            &sl,
            &u,
            None,
            None,
        )?;
        // The station deflections of the recovery integrate the elastic
        // curvature only: add the kink of every flexural hinge.
        for (h, site) in sites.iter().enumerate() {
            let e = &elems[site.elem];
            if let Some(stations) = results.frame_forces.get_mut(&e.name) {
                add_hinge_kink(stations, site.dof, st.q[h], site.rel * e.length, e.length);
            }
        }
        let forces = problem.forces(st);
        let hinges = sites
            .iter()
            .enumerate()
            .map(|(h, site)| {
                hinge_result(
                    site,
                    self.props[h],
                    &problem.curves[h],
                    forces[h],
                    st.q[h],
                    st.dir[h],
                )
            })
            .collect();
        Ok((results, hinges))
    }
}

/// The P-Delta axial force of every analysis frame element of a recovered
/// state: the average of the element's two end axial forces, tension
/// positive (the axial force of the geometric stiffness; v19 Analysis
/// Reference, "Nonlinear Static Analysis", P-Delta). Elements without
/// stations are left out.
pub(crate) fn frame_axial_forces(results: &CaseResults) -> IndexMap<String, f64> {
    results
        .frame_forces
        .iter()
        .filter(|(_, stations)| !stations.is_empty())
        .map(|(name, stations)| {
            let p = 0.5 * (stations.first().unwrap().p + stations.last().unwrap().p);
            (name.clone(), p)
        })
        .collect()
}

// ---------------------------------------------------------------------------
// P-Delta
// ---------------------------------------------------------------------------
//
// With P-Delta the problem of the hinges depends on the axial force `N` of
// every frame element (the geometric stiffness `K_g(N)` is part of the
// element stiffness), and `N` depends on the state. The path is followed with
// the stepper of `run`, one step at a time, each under the problem of its
// start state. The end state of a step is then corrected: its axial forces
// are iterated to a fixed point, with the end-state equations solved again for
// every iterate (the rows of the active hinges on their segments and one row
// for the event that ends the step), and the corrected state is accepted when
// no other event has been crossed (otherwise that event ends the step).

/// The slope of the segment of the curve of hinge `h` that the step uses.
fn step_slope(cls: &Classes, curves: &[Curve], st: &State, h: usize) -> f64 {
    let bb = curves[h].bb(st.dir[h]);
    bb.segs[bb.seg_at(cls.m[h])].slope
}

/// What the corrected end state of a step breaks.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Violation {
    /// Another event comes before the end of the step: it ends the step.
    Event(Ev),
    /// The active hinge deforms backwards under the corrected stiffness (the
    /// small plastic rate of the step changes sign): the hinge is not active
    /// in this step, which is solved again without it.
    Unload(usize),
}

/// What the P-Delta run of a case keeps: every saved state with the problem it
/// is consistent with, and the final state.
struct PdRun {
    saved: Vec<(State, Rc<Built>)>,
    last: (State, Rc<Built>),
    /// Why the run ended before its target (see [`Stop`]).
    stop: Option<Stop>,
}

/// What the correction of a step needs to know about the run.
struct PdCtx {
    control: Control,
    s_target: f64,
    /// Load factor at the start of the case, and the absolute monitored
    /// displacement there (`c0 + c lambda + w.q` of the start problem).
    lambda0: f64,
    u0: f64,
    target: f64,
    /// Progress of the next regularly spaced saved state.
    grid: Option<f64>,
    q_ref: f64,
    /// Convergence of the axial forces and its iteration limit.
    tol: f64,
    max_iterations: usize,
}

impl PdCtx {
    /// Progress of state `st` under the problem `b`.
    fn progress(&self, st: &State, b: &Built) -> f64 {
        match self.control {
            Control::Load => st.lambda - self.lambda0,
            Control::Monitored { .. } => {
                self.s_target * (b.monitored_chain + b.problem.displacement(st) - self.u0)
            }
        }
    }

    /// Check a corrected target or grid row in the units it controls.
    fn progress_row_holds(&self, event: Ev, progress: f64) -> bool {
        let goal = match event {
            Ev::Target => self.target,
            Ev::Grid => self.grid.unwrap_or(self.target),
            _ => return true,
        };
        let origin = match self.control {
            Control::Load => self.lambda0,
            Control::Monitored { .. } => self.u0,
        };
        let rounding = 16.0 * f64::EPSILON * (origin.abs() + goal.abs());
        (progress - goal).abs() <= 1e-12 * goal.abs() + rounding
    }
}

/// The axial forces of one recovered state together with what their
/// convergence is judged against. Every quantity here is a force, so the test
/// does not depend on the units of length or on the size of a hinge's moment
/// capacity (review of 2026-11-18: the test used to be scaled by the largest
/// hinge force, a moment for a rotational hinge, and the same cantilever in
/// metres and in millimetres gave load factors of 8.3456 and 8.0181).
struct AxialState {
    /// P-Delta axial force per analysis element.
    n: IndexMap<String, f64>,
    /// Largest axial or shear force at each element's stations.
    force: IndexMap<String, f64>,
    /// Per element: relative displacement of its ends over its length (the
    /// chord rotation, with the small axial strain), which turns an error of
    /// the axial force into an unbalanced transverse force `dN * tilt`.
    tilt: IndexMap<String, f64>,
    /// Per element: what rounding alone can change the axial force by,
    /// `EA/L` times the rounding of the end displacements.
    rounding: IndexMap<String, f64>,
}

/// Whether the axial forces `new` (of the state solved with the forces `old`
/// in the geometric stiffness) have converged, and the largest change of an
/// axial force (for the message of a failure).
///
/// Two conditions, both in force units (v19 Analysis Reference p. 435: the
/// convergence tolerance compares "the magnitude of force error with the
/// magnitude of the force acting on the structure"):
/// * the axial change is relative to that element's axial force;
/// * the unbalanced transverse force is relative to its station forces.
///
/// A change below what rounding can produce in an element counts as none.
fn axial_converged(new: &AxialState, old: &IndexMap<String, f64>, tol: f64) -> (bool, f64) {
    let mut gap = 0.0f64;
    let mut converged = true;
    for (name, value) in &new.n {
        let previous = old.get(name).copied().unwrap_or(0.0);
        let change = (value - previous).abs();
        gap = gap.max(change);
        let real = (change - new.rounding.get(name).copied().unwrap_or(0.0)).max(0.0);
        let tilt = new.tilt.get(name).copied().unwrap_or(0.0);
        let force = new.force.get(name).copied().unwrap_or(0.0);
        converged &= real <= tol * value.abs().max(previous.abs()) && real * tilt <= tol * force;
    }
    for (name, value) in old {
        if !new.n.contains_key(name) {
            gap = gap.max(value.abs());
            converged &= *value == 0.0;
        }
    }
    (converged, gap)
}

/// The rates, per unit load factor at fixed plastic jumps, of the hinge
/// forces and of the monitored displacement at the start state of a step,
/// INCLUDING what comes through the change of the axial forces (and so of the
/// geometric stiffness) with the load. The problem of one set of axial forces
/// has only the direct part (`Problem::g`, `Problem::c`); under a load that
/// acts along the members the direct part can be zero while the structure
/// still moves (review of 2026-11-18: after a lateral preload, a compression
/// moved the tip and displacement control answered "does not move").
struct Tangent {
    g: Vec<f64>,
    c: f64,
}

/// The problem a step is predicted with when the coupling through the axial
/// forces matters: `p` with the rates of `t`, and with the forces at zero
/// load factor shifted so that the forces of the state at load factor
/// `lambda` are unchanged.
fn coupled_problem(p: &Problem, t: &Tangent, lambda: f64) -> Problem {
    let mut out = p.clone();
    for i in 0..out.g.len() {
        out.f0[i] += (p.g[i] - t.g[i]) * lambda;
        out.g[i] = t.g[i];
    }
    out.c = t.c;
    out
}

impl CaseSetup<'_> {
    fn p_delta(&self) -> bool {
        self.lc.nl_geom == NLGeomType::PDelta
    }

    /// The P-Delta axial force of every frame element of state `st` under the
    /// problem `built`.
    fn axial_of(&self, built: &Built, st: &State) -> Result<IndexMap<String, f64>, SolverError> {
        Ok(self.axial_state(built, st)?.n)
    }

    /// The axial forces of state `st` under the problem `built`, with the
    /// force scale, the chord rotations and the rounding floors that
    /// `axial_converged` needs.
    fn axial_state(&self, built: &Built, st: &State) -> Result<AxialState, SolverError> {
        let (results, _) = self.recover(built, st)?;
        let n = frame_axial_forces(&results);
        let force = results
            .frame_forces
            .iter()
            .map(|(name, stations)| {
                let peak = stations.iter().fold(0.0f64, |m, f| {
                    m.max(f.p.abs()).max(f.v2.abs()).max(f.v3.abs())
                });
                (name.clone(), peak)
            })
            .collect();
        let mut tilt = IndexMap::new();
        let mut rounding = IndexMap::new();
        for name in n.keys() {
            let Some(frame) = self.model.frames.get(name) else {
                continue;
            };
            let (Some(pi), Some(pj)) = (
                self.work_points.get(&frame.point_i),
                self.work_points.get(&frame.point_j),
            ) else {
                continue;
            };
            let length =
                ((pj.x - pi.x).powi(2) + (pj.y - pi.y).powi(2) + (pj.z - pi.z).powi(2)).sqrt();
            if length <= 0.0 {
                continue;
            }
            let zero = [0.0; 6];
            let ui = results.joint_displacements.get(&frame.point_i).unwrap_or(&zero);
            let uj = results.joint_displacements.get(&frame.point_j).unwrap_or(&zero);
            let relative = (0..3).map(|d| (uj[d] - ui[d]).powi(2)).sum::<f64>().sqrt();
            let largest = (0..3).fold(0.0f64, |m, d| m.max(ui[d].abs()).max(uj[d].abs()));
            tilt.insert(name.clone(), relative / length);
            let ea = self
                .model
                .frame_sections
                .get(&frame.section)
                .and_then(|sec| Some(sec.area * self.model.materials.get(&sec.material)?.e))
                .unwrap_or(0.0);
            rounding.insert(name.clone(), 256.0 * f64::EPSILON * ea / length * largest);
        }
        Ok(AxialState {
            n,
            force,
            tilt,
            rounding,
        })
    }

    fn not_converged(&self, iterations: usize, residual: f64, what: &str) -> SolverError {
        SolverError::NonConvergence {
            case: self.lc.name.clone(),
            iterations,
            residual,
            reason: format!("P-Delta axial-force iteration {what}"),
        }
    }

    /// The problem for the axial forces of state `st`: the fixed point
    /// `N = axial(st; problem(N))`, iterated from the axial forces the initial
    /// case ended with (none for a case that starts from rest).
    fn settle_axial(
        &self,
        st: &State,
        tol: f64,
        max_iterations: usize,
    ) -> Result<Rc<Built>, SolverError> {
        let mut n = self
            .lc
            .initial_case
            .as_ref()
            .and_then(|initial| self.model.case_results.get(initial))
            .map(frame_axial_forces)
            .unwrap_or_default();
        let mut gap_last = 0.0;
        for _ in 0..max_iterations {
            let built = self.build(&n)?;
            let next = self.axial_state(&built, st)?;
            let (converged, gap) = axial_converged(&next, &n, tol);
            if converged {
                return Ok(Rc::new(built));
            }
            gap_last = gap;
            n = next.n;
        }
        Err(self.not_converged(
            max_iterations,
            gap_last,
            "of the start state of the case did not converge",
        ))
    }

    /// The rates of the state `st` (whose problem is `built`) with the coupling
    /// through the axial forces, by a small trial increase of the load factor
    /// with the axial forces brought to their fixed point. `None` when the
    /// coupling is small: no rate differs from the direct one by more than a
    /// tenth, so the direct rates serve and nothing changes for such a case.
    /// Also `None` when the trial state cannot be settled.
    fn coupled_tangent(
        &self,
        built: &Built,
        st: &State,
        monitored: bool,
        tol: f64,
    ) -> Option<Tangent> {
        // The trial state is the solver's own device: its iteration has a
        // limit of its own, not the limit the case sets for a step, so that
        // the rates (and with them the path) do not depend on that limit.
        const TRIAL_ITERATIONS: usize = 50;
        // Choose the perturbation in physical response units. Rescaling a
        // pattern then inversely rescales its trial load factor.
        let mut response = 0.0f64;
        // All frame elements can affect a monitored displacement, including
        // elements without hinge sites. Compare translations in common axes.
        let u = &built.u_own;
        let translation = |joint: usize, axis: usize| -> f64 {
            (0..3).map(|d| self.joint_rot[joint][axis][d] * u[6 * joint + d]).sum()
        };
        for frame in self.model.frames.values() {
            let (i, j) = (self.pt_index[&frame.point_i], self.pt_index[&frame.point_j]);
            let (pi, pj) = (&self.work_points[&frame.point_i], &self.work_points[&frame.point_j]);
            let length = (pi.x - pj.x).hypot(pi.y - pj.y).hypot(pi.z - pj.z);
            if length == 0.0 {
                continue;
            }
            for d in 0..3 {
                response = response
                    .max((translation(j, d) - translation(i, d)).abs() / length)
                    .max(u[6 * i + 3 + d].abs())
                    .max(u[6 * j + 3 + d].abs());
            }
        }
        // Both samples must satisfy the same fixed-point tolerance. Otherwise
        // the correction of a previously accepted state enters the derivative
        // divided by the (possibly tiny) probe interval.
        let settle = |sample: &State, mut n: IndexMap<String, f64>| -> Option<Built> {
            for _ in 0..TRIAL_ITERATIONS {
                let b = self.build(&n).ok()?;
                let next = self.axial_state(&b, sample).ok()?;
                if axial_converged(&next, &n, tol.min(1e-10)).0 {
                    return Some(b);
                }
                n = next.n;
            }
            None
        };
        let base = settle(st, built.axial.clone())?;
        let observations = |b: &Built, state: &State| {
            let mut values = b.problem.forces(state);
            if monitored {
                values.push(b.monitored_chain + b.problem.displacement(state));
            }
            values
        };
        let at_base = observations(&base, st);
        let mut direct = built.problem.g.clone();
        if monitored {
            direct.push(built.problem.c);
        }
        let undeformed = st.lambda == 0.0 && st.q.iter().all(|&q| q == 0.0)
            && base.u_chain.iter().all(|&u| u == 0.0);
        let mut rates = direct.clone();
        let mut resolved = vec![false; rates.len()];
        let mut previous = vec![0.0; rates.len()];
        // If a large response elsewhere makes this first interval too small
        // to resolve a particular force or displacement, increase it only for
        // that observable. Two successive secants must agree before replacing
        // its direct rate. A coarse probe hitting instability is discarded.
        let mut delta = if response > 0.0 {
            (1e-8 / response).max(1e-8 * st.lambda.abs())
        } else {
            1e-3 * st.lambda.abs().max(1.0)
        };
        const GROWTH: f64 = 8.0;
        for attempt in 0..12 {
            let mut trial = st.clone();
            trial.lambda += delta;
            let Some(b) = settle(&trial, base.axial.clone()) else { break };
            let at_trial = observations(&b, &trial);
            for i in 0..rates.len() {
                if resolved[i] {
                    continue;
                }
                let diff = at_trial[i] - at_base[i];
                let magnitude = at_base[i].abs() + at_trial[i].abs();
                let total = diff / delta;
                let significant = diff.abs() > 1e-9 * magnitude;
                if significant {
                    let agreement = 0.05 * total.abs()
                        + 64.0 * f64::EPSILON * magnitude * GROWTH / delta;
                    if attempt == 0 || (total - previous[i]).abs() <= agreement {
                        if (total - direct[i]).abs() * delta > 64.0 * f64::EPSILON * magnitude {
                            rates[i] = total;
                        }
                        resolved[i] = true;
                    }
                } else if magnitude == 0.0 && undeformed {
                    // No preload or plastic jump can drive a geometric rate.
                    resolved[i] = true;
                }
                previous[i] = total;
            }
            if resolved.iter().all(|&done| done) {
                break;
            }
            delta *= GROWTH;
        }
        let coupled = rates.iter().zip(&direct).any(|(total, direct)| {
            (total - direct).abs() > 0.1 * total.abs().max(direct.abs())
        });
        let c = if monitored { rates.pop()? } else { built.problem.c };
        coupled.then_some(Tangent { g: rates, c })
    }

    /// The end state of `step` for the problem `b`: the load factor and the
    /// plastic jumps of the active hinges that put every active hinge on the
    /// segment of its curve the step used and satisfy the row of `event`
    /// (plan step 5). Hinges that are not active keep their jump. `None` when
    /// the equations are singular (hinges in series, as at a joint where a
    /// column and a beam have yielded, leave a null vector).
    ///
    /// With `coupled` (the rates of the step's start state with the coupling
    /// through the axial forces, and the load factor of the iterate the axial
    /// forces of `b` were taken from) the column of the load factor holds
    /// those rates, and the right-hand side is shifted so that the solution is
    /// one quasi-Newton step from that iterate. At the fixed point the same
    /// equations hold as without it. The flag returned with the state says
    /// whether they hold already: every row within the tolerance of the case
    /// (hinge rows against the hinge's force scale; a target row to 1e-12 of
    /// the target, so that the displacement asked for is met as closely as
    /// without the coupling).
    #[allow(clippy::too_many_arguments)]
    fn solve_end(
        &self,
        b: &Built,
        step: &Step,
        active: &[bool],
        st_k: &State,
        event: Ev,
        ctx: &PdCtx,
        coupled: Option<(&Tangent, f64)>,
    ) -> Result<Option<(State, bool)>, SolverError> {
        let direct = &b.problem;
        let shifted;
        let p = match coupled {
            Some((t, lambda_i)) => {
                shifted = coupled_problem(direct, t, lambda_i);
                &shifted
            }
            None => direct,
        };
        let n = p.curves.len();
        let idx: Vec<usize> = (0..n).filter(|&h| active[h]).collect();
        let m = idx.len();
        let dim = m + 1;
        // The part of a row that the jumps of the hinges that are not active
        // contribute: `sum_j coef(j) q_j` over those hinges.
        let fixed = |coef: &dyn Fn(usize) -> f64| -> f64 {
            (0..n)
                .filter(|&j| !active[j])
                .map(|j| coef(j) * st_k.q[j])
                .sum()
        };
        let mut mat = vec![0.0; dim * dim];
        let mut rhs = vec![0.0; dim];
        for (ri, &i) in idx.iter().enumerate() {
            for (cj, &j) in idx.iter().enumerate() {
                mat[ri * dim + cj] = p.phi[i * n + j];
            }
            let d = f64::from(st_k.dir[i]);
            let bb = p.curves[i].bb(st_k.dir[i]);
            let seg = &bb.segs[bb.seg_at(step.cls.m[i])];
            mat[ri * dim + ri] -= seg.slope;
            mat[ri * dim + m] = p.g[i];
            rhs[ri] = d * (seg.f0 - seg.slope * seg.q0) - p.f0[i] - fixed(&|j| p.phi[i * n + j]);
        }
        let force_row = |mat: &mut [f64], rhs: &mut [f64], h: usize, value: f64| {
            for (cj, &j) in idx.iter().enumerate() {
                mat[m * dim + cj] = p.phi[h * n + j];
            }
            mat[m * dim + m] = p.g[h];
            rhs[m] = value - p.f0[h] - fixed(&|j| p.phi[h * n + j]);
        };
        let progress_row = |mat: &mut [f64], rhs: &mut [f64], value: f64| match ctx.control {
            Control::Load => {
                mat[m * dim + m] = 1.0;
                rhs[m] = ctx.lambda0 + value;
            }
            Control::Monitored { .. } => {
                let s = ctx.s_target;
                for (cj, &j) in idx.iter().enumerate() {
                    mat[m * dim + cj] = s * p.w[j];
                }
                mat[m * dim + m] = s * p.c;
                // The row is `u0_b + c lambda + w.q`; with a coupled rate the
                // part of the displacement at the iterate that the direct rate
                // gave is kept: `(c_direct - c) lambda_i`.
                let shift = coupled.map_or(0.0, |(_, lambda_i)| (direct.c - p.c) * lambda_i);
                rhs[m] = value + s * (ctx.u0 - b.monitored_chain) - s * fixed(&|j| p.w[j]) - s * shift;
            }
        };
        match event {
            Ev::SegEnd(h) => {
                let bb = p.curves[h].bb(st_k.dir[h]);
                let seg = &bb.segs[bb.seg_at(step.cls.m[h])];
                let cj = idx.iter().position(|&k| k == h).ok_or_else(|| {
                    SolverError::InvalidInput(format!(
                        "case '{}': a segment end of an inactive hinge ended a P-Delta step",
                        self.lc.name
                    ))
                })?;
                mat[m * dim + cj] = 1.0;
                rhs[m] = f64::from(st_k.dir[h]) * seg.q1;
            }
            Ev::Yield(h, sign) => {
                let curve = &p.curves[h];
                let y = if sign > 0 {
                    curve.pos.force_b()
                } else {
                    -curve.neg.force_b()
                };
                force_row(&mut mat, &mut rhs, h, y);
            }
            Ev::Reload(h) => {
                let bb = p.curves[h].bb(st_k.dir[h]);
                let y = f64::from(st_k.dir[h]) * bb.force_at(step.cls.m[h]);
                force_row(&mut mat, &mut rhs, h, y);
            }
            Ev::Reverse(h) => force_row(&mut mat, &mut rhs, h, 0.0),
            Ev::Target => progress_row(&mut mat, &mut rhs, ctx.target),
            Ev::Grid => progress_row(&mut mat, &mut rhs, ctx.grid.unwrap_or(ctx.target)),
        }
        let Some(x) = solve_scaled(mat, rhs, dim) else {
            return Ok(None);
        };
        let mut end = st_k.clone();
        end.lambda = x[m];
        for (ri, &i) in idx.iter().enumerate() {
            end.q[i] = x[ri];
        }
        // How far the equations of `b` itself are from holding at `end`: the
        // difference of the two columns times the change of the load factor.
        let mut holds = true;
        if let Some((t, lambda_i)) = coupled {
            let change = (end.lambda - lambda_i).abs();
            let force_scale = |h: usize| {
                direct.curves[h]
                    .pos
                    .force_scale()
                    .max(direct.curves[h].neg.force_scale())
            };
            let force_ok =
                |h: usize| ((direct.g[h] - t.g[h]) * change).abs() <= ctx.tol * force_scale(h);
            holds = idx.iter().all(|&i| force_ok(i));
            holds &= match event {
                Ev::Yield(h, _) | Ev::Reload(h) | Ev::Reverse(h) => force_ok(h),
                Ev::SegEnd(_) => true,
                Ev::Target | Ev::Grid => match ctx.control {
                    Control::Load => true,
                    Control::Monitored { .. } => {
                        // To 1e-12 of the target, and not below what rounding
                        // leaves of the absolute displacement it is measured in.
                        let rounding = 16.0 * f64::EPSILON * (ctx.u0.abs() + ctx.target.abs());
                        ((direct.c - t.c) * change).abs() <= 1e-12 * ctx.target.abs() + rounding
                    }
                },
            };
        }
        holds &= ctx.progress_row_holds(event, ctx.progress(&end, b));
        Ok(Some((end, holds)))
    }

    /// What the corrected end state `end` breaks most under the problem `b`,
    /// if anything (plan step 6): another event that it has passed (the step
    /// was too long), or an active hinge that deforms backwards.
    fn violation(
        &self,
        b: &Built,
        step: &Step,
        active: &[bool],
        st_k: &State,
        end: &State,
        ctx: &PdCtx,
    ) -> Option<Violation> {
        let p = &b.problem;
        let f = p.forces(end);
        let scale_q = end
            .q
            .iter()
            .zip(&st_k.q)
            .fold(0.0f64, |m, (a, b)| m.max((a - b).abs()));
        let tol_q = 1e-9 * scale_q + 1e-12 * ctx.q_ref;
        let mut worst: Option<(f64, Violation)> = None;
        let mut note = |excess: f64, v: Violation| {
            if worst.is_none_or(|(w, _)| excess > w) {
                worst = Some((excess, v));
            }
        };
        for h in 0..p.curves.len() {
            let curve = &p.curves[h];
            let d = f64::from(st_k.dir[h]);
            let fref = curve
                .pos
                .force_scale()
                .max(curve.neg.force_scale())
                .max(1e-300);
            if active[h] {
                let bb = curve.bb(st_k.dir[h]);
                let seg = &bb.segs[bb.seg_at(step.cls.m[h])];
                let m_end = d * end.q[h];
                if seg.q1.is_finite() && m_end > seg.q1 + tol_q {
                    note((m_end - seg.q1) / ctx.q_ref, Violation::Event(Ev::SegEnd(h)));
                } else if m_end < step.cls.m[h] - tol_q {
                    note((step.cls.m[h] - m_end) / ctx.q_ref, Violation::Unload(h));
                }
                continue;
            }
            if step.cls.m[h] == 0.0 {
                let (fp, fneg) = (curve.pos.force_b(), curve.neg.force_b());
                if f[h] > fp * (1.0 + 1e-9) {
                    note((f[h] - fp) / fref, Violation::Event(Ev::Yield(h, 1)));
                } else if f[h] < -fneg * (1.0 + 1e-9) {
                    note((-fneg - f[h]) / fref, Violation::Event(Ev::Yield(h, -1)));
                }
                continue;
            }
            let x = d * f[h];
            let fb = curve.bb(st_k.dir[h]).force_at(step.cls.m[h]);
            if x > fb + 1e-9 * fref {
                note((x - fb) / fref, Violation::Event(Ev::Reload(h)));
            } else if x < -1e-9 * fref {
                note(-x / fref, Violation::Event(Ev::Reverse(h)));
            }
        }
        // The target, and the next saved state, may have been passed: under
        // the corrected stiffness the event that ended the step comes later
        // than they do. (Only when the step moves towards them, as `step_once`.)
        if step.sol.dp > 1e-300 {
            let p_end = ctx.progress(end, b);
            let tol_p = 1e-11 * ctx.target;
            if p_end > ctx.target + tol_p {
                note((p_end - ctx.target) / ctx.target, Violation::Event(Ev::Target));
            }
            if let Some(grid) = ctx.grid.filter(|g| *g < ctx.target * (1.0 - 1e-9)) {
                if p_end > grid + tol_p {
                    note((p_end - grid) / ctx.target, Violation::Event(Ev::Grid));
                }
            }
        }
        worst.map(|(_, v)| v)
    }

    /// The events processed at the corrected end state: the terminating one,
    /// and every event of the step tied with it that still holds there (its
    /// remaining time, along the rates of the step, within the tie tolerance
    /// of the step; plan step 7).
    fn tied_events(&self, b: &Built, step: &Step, end: &State, event: Ev, ctx: &PdCtx) -> Vec<Ev> {
        let p = &b.problem;
        let f = p.forces(end);
        let p_end = ctx.progress(end, b);
        let limit = step.tau * EVENT_TIE;
        let remaining = |ev: Ev| -> f64 {
            match ev {
                Ev::SegEnd(h) => {
                    let d = f64::from(end.dir[h]);
                    let bb = p.curves[h].bb(end.dir[h]);
                    let seg = &bb.segs[bb.seg_at(step.cls.m[h])];
                    (seg.q1 - d * end.q[h]) / (step.sol.dq[h] * d)
                }
                Ev::Yield(h, s) => {
                    let curve = &p.curves[h];
                    let y = if s > 0 {
                        curve.pos.force_b()
                    } else {
                        -curve.neg.force_b()
                    };
                    (y - f[h]) / step.sol.df[h]
                }
                Ev::Reload(h) => {
                    let d = f64::from(end.dir[h]);
                    let fb = p.curves[h].bb(end.dir[h]).force_at(step.cls.m[h]);
                    (fb - d * f[h]) / (d * step.sol.df[h])
                }
                Ev::Reverse(h) => {
                    let d = f64::from(end.dir[h]);
                    d * f[h] / -(d * step.sol.df[h])
                }
                Ev::Target => (ctx.target - p_end) / step.sol.dp,
                Ev::Grid => (ctx.grid.unwrap_or(ctx.target) - p_end) / step.sol.dp,
            }
        };
        let mut out = Vec::new();
        if !step.tied.contains(&event) {
            out.push(event);
        }
        for ev in &step.tied {
            let reached = match ev {
                Ev::Target | Ev::Grid => ctx.progress_row_holds(*ev, p_end),
                _ => remaining(*ev) <= limit,
            };
            if *ev == event || reached {
                out.push(*ev);
            }
        }
        out
    }

    /// The end state of `step` (taken from the state `st_k`, whose problem is
    /// `built_k`) with the axial forces in equilibrium (plan steps 5 and 6):
    /// the state, the problem it is consistent with, the events to process
    /// there, and the active set of the step (the one of `step`, less the
    /// hinges that turned out not to deform in it).
    #[allow(clippy::type_complexity)]
    fn correct_step(
        &self,
        step: &Step,
        st_k: &State,
        built_k: &Built,
        ctx: &PdCtx,
        tangent: Option<&Tangent>,
        counters: &mut Counters,
        max_null_steps: usize,
    ) -> Result<(State, Rc<Built>, Vec<Ev>, Vec<bool>), SolverError> {
        let mut event = step.event;
        let mut tried = vec![event];
        let mut active = step.active.clone();
        let mut guess = self.axial_of(built_k, &step.cand)?;
        loop {
            let mut n_prime = guess.clone();
            // The load factor of the iterate `n_prime` was taken from.
            let mut lambda_i = step.cand.lambda;
            let mut residual = 0.0;
            let mut converged = None;
            let mut singular = false;
            for _ in 0..ctx.max_iterations {
                let b = Rc::new(self.build(&n_prime)?);
                let coupled = tangent.map(|t| (t, lambda_i));
                let Some((end, holds)) =
                    self.solve_end(&b, step, &active, st_k, event, ctx, coupled)?
                else {
                    singular = true;
                    break;
                };
                let state = self.axial_state(&b, &end)?;
                let (done, gap) = axial_converged(&state, &n_prime, ctx.tol);
                let n_new = state.n;
                if done && holds {
                    converged = Some((b, end, n_new));
                    break;
                }
                residual = gap;
                lambda_i = end.lambda;
                n_prime = n_new;
            }
            if singular {
                // As the search of the active set does: drop the highest-index
                // active hinge that carries no strength (project choice).
                let event_hinge = match event {
                    Ev::SegEnd(h) => Some(h),
                    _ => None,
                };
                let drop = (0..active.len()).rev().find(|&h| {
                    active[h]
                        && Some(h) != event_hinge
                        && step_slope(&step.cls, &self.curves, st_k, h) <= 0.0
                });
                counters.nulls += 1;
                match drop {
                    Some(h) if counters.nulls <= max_null_steps => {
                        active[h] = false;
                        continue;
                    }
                    _ => {
                        return Err(SolverError::NonConvergence {
                            case: self.lc.name.clone(),
                            iterations: counters.steps,
                            residual: 0.0,
                            reason: format!(
                                "P-Delta axial-force iteration: the end-state equations of the step are singular at load factor {}",
                                st_k.lambda
                            ),
                        });
                    }
                }
            }
            let Some((b, end, n_new)) = converged else {
                return Err(self.not_converged(
                    ctx.max_iterations,
                    residual,
                    &format!(
                        "did not converge at load factor {} (raise the maximum number of iterations or the tolerance of the case)",
                        st_k.lambda
                    ),
                ));
            };
            let Some(violation) = self.violation(&b, step, &active, st_k, &end, ctx) else {
                let tied = self.tied_events(&b, step, &end, event, ctx);
                return Ok((end, b, tied, active));
            };
            // The step was too long (another event comes first), or a hinge is
            // not active in it. Both are retries, and count as null steps.
            counters.nulls += 1;
            if counters.nulls > max_null_steps {
                return Err(SolverError::NonConvergence {
                    case: self.lc.name.clone(),
                    iterations: counters.steps,
                    residual: (ctx.target - ctx.progress(st_k, built_k)).max(0.0),
                    reason: format!(
                        "the maximum number of null steps ({max_null_steps}) was reached in the P-Delta correction of the end of a step; raise it in the case's nonlinear parameters"
                    ),
                });
            }
            match violation {
                Violation::Event(violated) => {
                    if tried.contains(&violated) {
                        return Err(SolverError::NonConvergence {
                            case: self.lc.name.clone(),
                            iterations: counters.steps,
                            residual: 0.0,
                            reason: format!(
                                "the events of a step reorder under P-Delta and loop (event {violated:?} was already tried after {tried:?}) at load factor {}",
                                st_k.lambda
                            ),
                        });
                    }
                    tried.push(violated);
                    event = violated;
                }
                Violation::Unload(h) => active[h] = false,
            }
            guess = n_new;
        }
    }

    /// Follows the load path of a case with P-Delta (see the comment above).
    fn run_pdelta(&self, spec: &RunSpec) -> Result<PdRun, SolverError> {
        let lc = self.lc;
        let (f_ref, q_ref) = curve_scales(&self.curves);
        let tol = lc.convergence_tol;
        let max_iterations = lc.max_iterations.max(1);
        let probe = self.build(&IndexMap::new())?;
        let (mut st, carried) = self.carried_state(&probe.sites)?;
        let mut built = self.settle_axial(&st, tol, max_iterations)?;
        // The carried forces were reported under the axial forces of the
        // initial case, which agree with these to the tolerance of the case.
        self.check_continuation(&built, &st, &carried, (100.0 * tol).max(1e-8))?;
        let n = built.sites.len();
        let mut active = vec![false; n];
        let mut s_target = 1.0;
        let monitored = matches!(spec.control, Control::Monitored { .. });
        if monitored {
            // The direction the monitored displacement takes as the load
            // grows, with the part that comes through the axial forces.
            let tangent = self.coupled_tangent(&built, &st, true, tol);
            let coupled = tangent
                .as_ref()
                .map(|t| coupled_problem(&built.problem, t, st.lambda));
            let problem = coupled.as_ref().unwrap_or(&built.problem);
            let mut stepper = Stepper {
                p: problem,
                n,
                control: spec.control,
                s_target: 1.0,
                f_ref,
                q_ref,
            };
            s_target = loading_direction(&mut stepper, problem, &st, &mut active)
                .map_err(|e| map_error(e, lc, &built.sites, spec.target))?;
            active = vec![false; n];
        }
        let target = spec.target;
        let mut ctx = PdCtx {
            control: spec.control,
            s_target,
            lambda0: st.lambda,
            u0: built.monitored_chain + built.problem.displacement(&st),
            target,
            grid: None,
            q_ref,
            tol,
            max_iterations,
        };
        let mut plan = spec
            .save
            .map(|(min, max, positive_only)| SavePlan::new(min, max, positive_only, target));
        let mut saved: Vec<(State, Rc<Built>)> = Vec::new();
        if plan.is_some() {
            saved.push((st.clone(), Rc::clone(&built)));
        }
        let mut counters = Counters { steps: 0, nulls: 0 };
        let mut stop = None;
        loop {
            let p_now = ctx.progress(&st, &built);
            if p_now >= target * (1.0 - 1e-12) {
                break;
            }
            ctx.grid = plan.as_ref().map(SavePlan::next_grid);
            // The step is predicted, and its end state solved, with the rates
            // that include the coupling through the axial forces where that
            // coupling matters; otherwise with the direct rates, as before.
            let tangent = self.coupled_tangent(&built, &st, monitored, tol);
            let coupled = tangent
                .as_ref()
                .map(|t| coupled_problem(&built.problem, t, st.lambda));
            let stepper = Stepper {
                p: coupled.as_ref().unwrap_or(&built.problem),
                n,
                control: spec.control,
                s_target,
                f_ref,
                q_ref,
            };
            let step = match stepper.step_once(
                &mut st,
                &mut active,
                plan.as_ref(),
                spec,
                p_now,
                &mut counters,
            ) {
                Ok(step) => step,
                Err(e) => {
                    let stopped = Stop::of(e).map_err(|e| map_error(e, lc, &built.sites, spec.target));
                    stop = Some(stopped?);
                    break;
                }
            };
            // A failure of the axial-force iteration inside the step, or the null
            // steps its correction used up, ends the case early like the budgets do.
            let (end, built_next, tied, step_active) = match self.correct_step(
                &step,
                &st,
                &built,
                &ctx,
                tangent.as_ref(),
                &mut counters,
                spec.max_null_steps,
            ) {
                Ok(done) => done,
                Err(SolverError::NonConvergence { reason, .. }) => {
                    stop = Some(Stop {
                        reason,
                        steps: counters.steps,
                        progress: p_now,
                    });
                    break;
                }
                Err(e) => return Err(e),
            };
            let step = Step {
                active: step_active,
                ..step
            };
            let accepted = stepper
                .accept(&step, &end, &tied)
                .map_err(|e| map_error(e, lc, &built.sites, spec.target))?;
            st = accepted.state;
            active = accepted.next_active;
            built = built_next;
            let p_new = ctx.progress(&st, &built);
            if save_decision(
                &mut plan,
                p_new,
                target,
                accepted.hit_target,
                accepted.hit_grid,
                accepted.hit_event,
            ) {
                saved.push((st.clone(), Rc::clone(&built)));
            }
            // The target row was solved: the case is done. (The test at the
            // top of the loop compares to 1e-12 of the target, which rounding
            // of the absolute displacement can miss when the increment asked
            // for is a small part of it.)
            if accepted.hit_target {
                break;
            }
        }
        Ok(PdRun {
            saved,
            last: (st, built),
            stop,
        })
    }
}

/// Runs a nonlinear static case in hinge space (see the module comment).
///
/// Supported: frame elements with uncoupled deformation-controlled hinges of
/// user-defined curves, joint constraints, grounded joint springs, property
/// modifiers, self weight, frame distributed and point loads and joint
/// forces. A case that cannot reach its target (steps or null steps ran out, no
/// admissible state, the P-Delta axial-force iteration failed inside a step) returns
/// the states it reached with the status "not finished" (G16). The status words are
/// the vendor's (v19 help "Set Load Cases to Run"); KEEPING the states reached before
/// the stop, and appending the last accepted state to the saved ones, is a project
/// choice: v19 does not say which results such a case keeps. Refusals of behaviour that
/// is not implemented (reversal, point E, no motion) stay errors with no results.
///
/// The `local_disp` of the stations of an element with a flexural (M2 or M3)
/// hinge carries the slope jump of the hinge (see [`add_hinge_kink`]). Limit:
/// the jumps of axial, shear and torsion hinges are not added to the station
/// displacements (the joint displacements, reactions and station forces are
/// exact for all of them).
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
    let setup = CaseSetup::new(
        model,
        lc,
        work_points,
        work_areas,
        orig_area_map,
        pt_index,
        joint_rot,
        k_struct,
        k_base,
        diag_mass,
        mesh,
        ndof,
    )?;
    if setup.p_delta() {
        let spec = setup.run_spec()?;
        let run = setup.run_pdelta(&spec)?;
        let mut saved: Vec<(&State, &Built)> = run.saved.iter().map(|(s, b)| (s, &**b)).collect();
        if run.stop.is_some() && saved.last().is_some_and(|(s, _)| **s != run.last.0) {
            saved.push((&run.last.0, &*run.last.1));
        }
        return setup.results_of(&saved, (&run.last.0, &*run.last.1), run.stop.as_ref());
    }
    // The problem with no P-Delta axial forces.
    let built = setup.build(&IndexMap::new())?;
    let start = setup.start_state(&built)?;
    let spec = setup.run_spec()?;
    let outcome = run(&built.problem, &start, &spec)
        .map_err(|e| map_error(e, lc, &built.sites, spec.target))?;
    setup.results(&built, &outcome)
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
        // G16: the budgets end the run early and keep what it reached (a stop).
        let out = run(&problem, &start, &spec(0)).expect("keeps what it reached");
        let stop = out.stop.expect("the null step budget is exceeded");
        assert!(stop.reason.contains("null steps"), "{}", stop.reason);
        assert!(run(&problem, &start, &spec(3)).expect("runs with a null step allowed").stop.is_none());
        let out = run(
            &problem,
            &start,
            &RunSpec {
                max_total_steps: 1,
                ..spec(3)
            },
        )
        .expect("keeps what it reached");
        let stop = out.stop.expect("the total step budget is exceeded");
        assert!(stop.reason.contains("total steps"), "{}", stop.reason);
        assert_eq!(stop.steps, 1);
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

    /// Elastic stiffness of a prismatic 3D beam written from the textbook
    /// formulas, local DOF order u1 u2 u3 r1 r2 r3 at I then at J (r2 = -du3/dx).
    /// No shear deformation.
    fn textbook_k(e: f64, g: f64, a: f64, j: f64, i2: f64, i3: f64, l: f64) -> Mat12 {
        let mut k = [[0.0; 12]; 12];
        let mut set = |r: usize, c: usize, v: f64| {
            k[r][c] = v;
            k[c][r] = v;
        };
        set(0, 0, e * a / l);
        set(6, 6, e * a / l);
        set(0, 6, -e * a / l);
        set(3, 3, g * j / l);
        set(9, 9, g * j / l);
        set(3, 9, -g * j / l);
        for (dofs, ei, s) in [([1, 5, 7, 11], e * i3, 1.0), ([2, 4, 8, 10], e * i2, -1.0)] {
            let [va, ta, vb, tb] = dofs;
            set(va, va, 12.0 * ei / l.powi(3));
            set(vb, vb, 12.0 * ei / l.powi(3));
            set(va, vb, -12.0 * ei / l.powi(3));
            set(ta, ta, 4.0 * ei / l);
            set(tb, tb, 4.0 * ei / l);
            set(ta, tb, 2.0 * ei / l);
            set(va, ta, s * 6.0 * ei / l.powi(2));
            set(va, tb, s * 6.0 * ei / l.powi(2));
            set(vb, ta, -s * 6.0 * ei / l.powi(2));
            set(vb, tb, -s * 6.0 * ei / l.powi(2));
        }
        k
    }

    fn norm(v: &[f64; 12]) -> f64 {
        v.iter().map(|x| x * x).sum::<f64>().sqrt()
    }

    fn unit(i: usize) -> [f64; 12] {
        let mut v = [0.0; 12];
        v[i] = 1.0;
        v
    }

    fn sub(a: &[f64; 12], b: &[f64; 12]) -> [f64; 12] {
        std::array::from_fn(|i| a[i] - b[i])
    }

    const BEAM: (f64, f64, f64, f64, f64, f64, f64, f64) =
        (2.0e8, 8.0e7, 1.0e-2, 3.0e-4, 2.0e-4, 1.0e-4, 3.0, -500.0);

    #[test]
    fn the_kink_vector_is_the_jump_without_the_rigid_tilt() {
        let (e, g, a, j, i2, i3, l, p) = BEAM;
        let ke = textbook_k(e, g, a, j, i2, i3, l);
        let kg = crate::frame::compute_local_geometric_stiffness(p, l);
        for dof in [HingeDof::M2, HingeDof::M3] {
            let jj = dislocation(dof, l, 0.0);
            let jk = kink_vector(dof, l);
            // Derived: J = R + Jk, and the kink is the end-I rotation alone.
            let pure = if dof == HingeDof::M3 { unit(5) } else { unit(4) };
            let want: [f64; 12] = std::array::from_fn(|i| -pure[i]);
            assert_eq!(jk, want, "{dof:?}");
            // K_e annihilates the rigid rotation: K_e Jk = K_e J.
            let (kj, kk) = (mat_vec(&ke, &jj), mat_vec(&ke, &jk));
            assert!(
                norm(&sub(&kj, &kk)) <= 1e-12 * norm(&kj),
                "{dof:?}: K_e J vs K_e Jk"
            );
            // K_g does not: K_g R is the P-Delta shear pair of the tilt.
            let r = sub(&jj, &jk);
            let kgr = mat_vec(&kg, &r);
            let mut tilt = [0.0; 12];
            if dof == HingeDof::M3 {
                tilt[1] = -p;
                tilt[7] = p;
            } else {
                tilt[2] = p;
                tilt[8] = -p;
            }
            assert!(norm(&sub(&kgr, &tilt)) <= 1e-12 * p.abs(), "{dof:?}: K_g R");
        }
    }

    /// What the geometric stiffness does to the four other hinge vectors, at
    /// end I (x = 0) and at end J (x = L). The rigid part of the jump is a
    /// translation or a twist of the whole element; K_g has no axial or
    /// torsional terms and sees only the relative transverse displacement of
    /// the ends, so it annihilates that part and J and J - R give the same
    /// force. A transverse slip (V2, V3) is the relative displacement of the
    /// two ends and K_g carries it as it is; P and T give K_g nothing.
    #[test]
    fn the_geometric_stiffness_annihilates_the_rigid_part_of_p_v_and_t_jumps() {
        let (e, g, a, j, i2, i3, l, p) = BEAM;
        let ke = textbook_k(e, g, a, j, i2, i3, l);
        let kg = crate::frame::compute_local_geometric_stiffness(p, l);
        for (dof, ends) in [
            (HingeDof::P, [0usize, 6]),
            (HingeDof::V2, [1, 7]),
            (HingeDof::V3, [2, 8]),
            (HingeDof::T, [3, 9]),
        ] {
            // The rigid motion of the whole element: the same at both ends.
            let mut r = [0.0; 12];
            r[ends[0]] = 1.0;
            r[ends[1]] = 1.0;
            for (who, name) in [(&ke, "K_e"), (&kg, "K_g")] {
                let kr = mat_vec(who, &r);
                assert!(norm(&kr) <= 1e-9 * p.abs(), "{dof:?}: {name} R = {kr:?}");
            }
            for x in [0.0, l] {
                let jj = dislocation(dof, l, x);
                let transverse = matches!(dof, HingeDof::V2 | HingeDof::V3);
                assert_eq!(norm(&mat_vec(&kg, &jj)) > 0.0, transverse, "{dof:?} at {x}");
                // J minus the rigid motion that carries J's end-J values.
                let jk = sub(&jj, &r);
                assert!(
                    norm(&sub(&mat_vec(&kg, &jj), &mat_vec(&kg, &jk))) <= 1e-9 * p.abs(),
                    "{dof:?} at {x}"
                );
            }
        }
    }

    #[test]
    fn do_not_drop_is_a_no_op_only_for_a_curve_that_never_drops() {
        let sym = |pts: [(f64, f64); 4], beyond| {
            let mut p =
                HingeProp::symmetric("H", HingeDof::M3, pts.map(|(f, d)| HingePoint::new(f, d)));
            p.beyond_e = beyond;
            p
        };
        let ok = |p: &HingeProp| {
            let curve = Curve {
                pos: build_backbone(p, 1, Some(10.0)).unwrap(),
                neg: build_backbone(p, -1, Some(10.0)).unwrap(),
            };
            check_never_drops(p, &curve, "F").is_ok()
        };
        let hardening = [(100.0, 0.0), (110.0, 0.1), (120.0, 0.2), (130.0, 0.3)];
        let flat = [(100.0, 0.0), (100.0, 0.1), (100.0, 0.2), (100.0, 0.3)];
        let drops = [(100.0, 0.0), (120.0, 0.1), (50.0, 0.1), (50.0, 0.3)];
        assert!(ok(&sym(hardening, Some(HingeBeyondE::Extrapolated))));
        assert!(ok(&sym(flat, Some(HingeBeyondE::Extrapolated))));
        assert!(!ok(&sym(drops, Some(HingeBeyondE::Extrapolated))));
        // Drops to zero beyond E, or not stated: refused.
        assert!(!ok(&sym(hardening, Some(HingeBeyondE::DropsToZero))));
        assert!(!ok(&sym(hardening, None)));
    }

    /// Axial force of every analysis element of a recovered state. A vertical
    /// column of two equal elements (kN, m; A = 1e-2 m2, unit weight 10
    /// kN/m3, so 0.1 kN/m of self weight; L = 1.5 m each) carries 500 kN
    /// down at the top and 200 kN down at the middle joint. The axial force
    /// of a station is tension positive and varies linearly with the self
    /// weight, so the average of the two end forces is the force at the
    /// middle of the element:
    /// top element: -(500 + 0.1 * 0.75) = -500.075,
    /// base element: -(500 + 200 + 0.1 * 1.5 + 0.1 * 0.75) = -700.225.
    #[test]
    fn the_axial_force_of_every_element_is_the_average_of_its_end_forces() {
        let mut m = SapModel::default();
        m.load_cases.clear();
        m.load_patterns.clear();
        m.active_dofs = [true; 6];
        let mut material = sap_model::Material::default_steel_kip_in();
        material.name = "MAT".into();
        material.e = 2.0e8;
        material.g = 2.0e8 / 2.6;
        material.u = 0.3;
        material.unit_weight = 10.0;
        material.unit_mass = 0.0;
        m.materials.insert("MAT".into(), material);
        m.frame_sections.insert(
            "SEC".into(),
            sap_model::FrameSection {
                name: "SEC".into(),
                material: "MAT".into(),
                prop_type: sap_model::FramePropType::General,
                area: 1.0e-2,
                i33: 1.0e-4,
                i22: 1.0e-4,
                torsional_j: 2.0e-4,
                as2: 0.0,
                as3: 0.0,
                modifiers: [1.0; 8],
                ..sap_model::FrameSection::default()
            },
        );
        for k in 0..3 {
            m.add_point(format!("P{k}"), 0.0, 0.0, 1.5 * k as f64);
        }
        m.points["P0"].restraint = [true; 6];
        m.add_frame("F1", "P0", "P1", "SEC");
        m.add_frame("F2", "P1", "P2", "SEC");
        m.load_patterns.insert(
            "G".into(),
            sap_model::LoadPattern {
                name: "G".into(),
                pat_type: sap_model::LoadPatternType::Other,
                self_wt_multiplier: 1.0,
            },
        );
        for (point, fz) in [("P1", -200.0), ("P2", -500.0)] {
            m.points[point].loads_force.push(sap_model::PointLoadForce {
                load_pat: "G".into(),
                csys: "GLOBAL".into(),
                force: [0.0, 0.0, fz, 0.0, 0.0, 0.0],
            });
        }
        m.load_cases
            .insert("G".into(), LoadCase::linear_static("G", "G", 1.0));
        crate::analyze_model(&mut m).unwrap();
        let axial = frame_axial_forces(&m.case_results["G"]);
        assert_eq!(axial.len(), 2);
        for (name, want) in [("F1", -700.225), ("F2", -500.075)] {
            let got = axial[name];
            assert!(
                (got - want).abs() <= 1e-9 * want.abs(),
                "frame {name}: got {got} kN, expected {want} kN"
            );
        }
        // The two end forces really differ (the test is of the average).
        let stations = &m.case_results["G"].frame_forces["F1"];
        let (pi, pj) = (stations.first().unwrap().p, stations.last().unwrap().p);
        assert!((pi - pj).abs() > 0.1);
    }
}
