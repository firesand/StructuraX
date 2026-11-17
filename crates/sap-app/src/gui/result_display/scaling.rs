//! Automatic scale of the deformed shape (D32) and the title of what the
//! viewport shows (D33).
//!
//! Source: the help topic "Show Deformed Shape" of the vendor says "The Auto
//! option will automatically set the scale factor. The Scale Factor option can
//! be used to exaggerate the displacements relative to the geometry of the
//! structure. If Auto was selected previously, the Scale Factor edit box will
//! display the scale factor previously used." The help gives no number for the
//! automatic scale, so the fraction below is a PROJECT CHOICE: with Auto the
//! largest drawn translation is [`AUTO_FRACTION`] of the diagonal of the
//! bounding box of the model. The topic gives no wording for the view title
//! either; the wording of [`view_subtitle`] is the project's.

use sap_model::{CaseResults, SapModel};

use super::contours::{self, StepSelection};

/// Largest drawn translation as a fraction of the diagonal of the model's
/// bounding box (project choice, see the module comment).
pub(in crate::gui) const AUTO_FRACTION: f64 = 0.1;

/// Largest displacement, relative to the diagonal, that still counts as a
/// displacement: below it the result is treated as "all zero" (roundoff noise
/// would otherwise be magnified to the size of the model).
pub(in crate::gui) const NEGLIGIBLE_RELATIVE: f64 = 1e-12;

/// Scale factor at which a largest translation `max_translation` is drawn as
/// `fraction * diagonal` (all in the same length unit). Always finite and
/// positive: a result without a (non-negligible) displacement, or a model
/// without extent, gives 1.
pub(in crate::gui) fn auto_scale_factor(max_translation: f64, diagonal: f64, fraction: f64) -> f64 {
    let usable = max_translation.is_finite()
        && diagonal.is_finite()
        && fraction.is_finite()
        && fraction > 0.0
        && diagonal > 0.0
        && max_translation > NEGLIGIBLE_RELATIVE * diagonal;
    if usable {
        let f = fraction * diagonal / max_translation;
        if f.is_finite() && f > 0.0 {
            return f;
        }
    }
    1.0
}

/// Diagonal of the box with corners `min` and `max`.
pub(in crate::gui) fn bounding_diagonal(min: [f64; 3], max: [f64; 3]) -> f64 {
    let d = |i: usize| (max[i] - min[i]).abs();
    let v = d(0).hypot(d(1)).hypot(d(2));
    if v.is_finite() {
        v
    } else {
        0.0
    }
}

fn norm3(v: [f64; 3]) -> f64 {
    let n = v[0].hypot(v[1]).hypot(v[2]);
    if n.is_finite() {
        n
    } else {
        0.0
    }
}

/// Largest translation (length of `(UX, UY, UZ)`, global axes) of any joint in
/// the shown state of `cr`, and of any frame station that the deformed shape
/// draws from the station results. A non-finite component counts as zero, as
/// the drawing does.
pub(in crate::gui) fn max_translation(model: &SapModel, cr: &CaseResults, sel: StepSelection) -> f64 {
    let mut largest = 0.0_f64;
    for joint in model.points.keys() {
        if let Some(u) = contours::joint_displacement_global(model, cr, joint, sel) {
            let t = [u[0], u[1], u[2]].map(|c| if c.is_finite() { c } else { 0.0 });
            largest = largest.max(norm3(t));
        }
    }
    // The deformed frames use their stations when the result has no modes and
    // no time-history step is shown (see the viewport).
    let station_source = if sel.use_step && !cr.static_steps.is_empty() {
        &cr.static_steps[sel.step_idx.min(cr.static_steps.len() - 1)].frame_forces
    } else {
        &cr.frame_forces
    };
    if cr.modes.is_empty() && !(sel.use_step && !cr.time_steps.is_empty()) {
        for stations in station_source.values() {
            for st in stations {
                let t = st.local_disp.map(|c| if c.is_finite() { c } else { 0.0 });
                largest = largest.max(norm3(t));
            }
        }
    }
    largest
}

/// The scale factor the automatic option sets for `cr` in the shown state.
pub(in crate::gui) fn auto_scale_for(
    model: &SapModel,
    cr: &CaseResults,
    sel: StepSelection,
    min: [f64; 3],
    max: [f64; 3],
) -> f64 {
    auto_scale_factor(max_translation(model, cr, sel), bounding_diagonal(min, max), AUTO_FRACTION)
}

/// What the deformed shape shows, to follow the case in the view title: the
/// mode number with its period and frequency for a modal result, the time of
/// the shown step of a time history, and the step (with its load factor) of a
/// saved nonlinear static state. `None` for the final state of a case.
pub(in crate::gui) fn view_subtitle(cr: &CaseResults, sel: StepSelection) -> Option<String> {
    if !cr.modes.is_empty() {
        let m = &cr.modes[sel.mode_idx.min(cr.modes.len() - 1)];
        return Some(format!(
            "Mode {}, Period = {:.5} s, Frequency = {:.4} Hz",
            m.mode_num, m.period_sec, m.freq_hz
        ));
    }
    if sel.use_step && !cr.static_steps.is_empty() {
        let i = sel.step_idx.min(cr.static_steps.len() - 1);
        let step = &cr.static_steps[i];
        return Some(format!(
            "Step {} of {}, Load factor = {:.5}",
            step.step,
            cr.static_steps.last().map_or(step.step, |s| s.step),
            step.load_factor
        ));
    }
    if sel.use_step && !cr.time_steps.is_empty() {
        let i = sel.step_idx.min(cr.time_steps.len() - 1);
        return Some(format!("Step {i}, Time = {:.4} s", cr.time_steps[i]));
    }
    None
}

/// The title of a viewport: its name, the case, and what of the case is shown.
pub(in crate::gui) fn view_title(name: &str, case: &str, subtitle: Option<String>) -> String {
    match subtitle {
        Some(sub) => format!("{name}  [Result: {case}]  {sub}"),
        None => format!("{name}  [Result: {case}]"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sap_model::hinge::NonlinearStaticStep;
    use sap_model::{ModalModeResult, PointObj};
    use sap_units::Units;

    fn close(what: &str, got: f64, want: f64, rel: f64) {
        assert!(
            (got - want).abs() <= rel * want.abs().max(1e-300),
            "{what}: got {got:e}, want {want:e}"
        );
    }

    /// Closed form: the drawn largest translation is `scale * u_max`, which
    /// must be `fraction * diagonal`. A 3-4-12 box has diagonal 13.
    #[test]
    fn the_automatic_scale_draws_the_largest_translation_as_a_fraction_of_the_diagonal() {
        let diag = bounding_diagonal([0.0, 0.0, 0.0], [3.0, 4.0, 12.0]);
        close("diagonal", diag, 13.0, 1e-15);
        let u = 0.8; // mass-normalised amplitudes are large; any value will do
        let f = auto_scale_factor(u, diag, AUTO_FRACTION);
        close("scale", f, 0.1 * 13.0 / 0.8, 1e-15);
        close("drawn", f * u, AUTO_FRACTION * diag, 1e-15);
        // Twice the amplitude, half the factor.
        close("half", auto_scale_factor(2.0 * u, diag, AUTO_FRACTION), 0.5 * f, 1e-15);
    }

    #[test]
    fn a_result_without_displacement_or_a_model_without_extent_gives_a_finite_factor() {
        for (u, d) in [
            (0.0, 10.0),
            (f64::NAN, 10.0),
            (f64::INFINITY, 10.0),
            (1.0, 0.0),
            (1.0, f64::NAN),
            (1.0, -3.0),
            // below the negligible threshold: roundoff, not a displacement
            (1e-15, 10.0),
        ] {
            let f = auto_scale_factor(u, d, AUTO_FRACTION);
            assert!(f.is_finite() && f > 0.0, "{u} {d}: {f}");
            assert_eq!(f, 1.0, "{u} {d}");
        }
        assert_eq!(auto_scale_factor(1.0, 10.0, 0.0), 1.0);
        assert_eq!(bounding_diagonal([f64::NAN; 3], [1.0; 3]), 0.0);
    }

    fn two_joint_model() -> SapModel {
        let mut m = SapModel::new(Units::KnMC);
        m.points.insert("1".into(), PointObj::new("1", 0.0, 0.0, 0.0));
        m.points.insert("2".into(), PointObj::new("2", 3.0, 4.0, 0.0));
        m
    }

    fn mode(num: usize, shape: [f64; 6]) -> ModalModeResult {
        let mode_shape = [("2".to_string(), shape)].into_iter().collect();
        ModalModeResult {
            mode_num: num,
            period_sec: 0.5,
            freq_hz: 2.0,
            circ_freq_rad_s: 4.0 * std::f64::consts::PI,
            eigenvalue: 0.0,
            participation_factors: [0.0; 6],
            modal_mass_ratios: [0.0; 6],
            mode_shape,
        }
    }

    #[test]
    fn the_largest_translation_of_a_mode_is_the_longest_joint_vector() {
        let model = two_joint_model();
        let cr = CaseResults {
            modes: vec![mode(1, [3.0, 0.0, 4.0, 9.0, 9.0, 9.0]), mode(2, [0.0, 0.5, 0.0, 0.0, 0.0, 0.0])],
            ..CaseResults::default()
        };
        // |(3, 0, 4)| = 5 (rotations are not translations); mode 2: 0.5.
        close("mode 1", max_translation(&model, &cr, StepSelection::default()), 5.0, 1e-15);
        let sel = StepSelection { mode_idx: 1, ..Default::default() };
        close("mode 2", max_translation(&model, &cr, sel), 0.5, 1e-15);
        // 5 against the diagonal 5 of the model: factor 0.1 * 5 / 5.
        let f = auto_scale_for(&model, &cr, StepSelection::default(), [0.0; 3], [3.0, 4.0, 0.0]);
        close("factor", f, 0.1, 1e-15);
        // A result with no displacement at all.
        let zero = CaseResults::default();
        assert_eq!(max_translation(&model, &zero, StepSelection::default()), 0.0);
        assert_eq!(auto_scale_for(&model, &zero, StepSelection::default(), [0.0; 3], [3.0, 4.0, 0.0]), 1.0);
    }

    #[test]
    fn the_title_names_the_mode_the_time_step_or_the_saved_state() {
        let mut cr = CaseResults { modes: vec![mode(1, [0.0; 6]), mode(2, [0.0; 6])], ..CaseResults::default() };
        let sel = StepSelection { mode_idx: 1, ..Default::default() };
        assert_eq!(
            view_subtitle(&cr, sel).unwrap(),
            "Mode 2, Period = 0.50000 s, Frequency = 2.0000 Hz"
        );
        // An index past the end is clamped as the drawing clamps it.
        let past = StepSelection { mode_idx: 9, ..Default::default() };
        assert!(view_subtitle(&cr, past).unwrap().starts_with("Mode 2,"));

        cr.modes.clear();
        cr.time_steps = vec![0.0, 0.02, 0.04];
        let t = StepSelection { use_step: true, step_idx: 2, ..Default::default() };
        assert_eq!(view_subtitle(&cr, t).unwrap(), "Step 2, Time = 0.0400 s");
        // The final state is not a step.
        assert!(view_subtitle(&cr, StepSelection::default()).is_none());

        cr.time_steps.clear();
        let step = |n: usize, lf: f64| NonlinearStaticStep {
            step: n,
            load_factor: lf,
            monitored_displacement: None,
            base_reactions: [0.0; 6],
            joint_displacements: Default::default(),
            joint_reactions: Default::default(),
            frame_forces: Default::default(),
            hinges: Vec::new(),
        };
        cr.static_steps = vec![step(0, 0.0), step(1, 0.5), step(2, 1.0)];
        let s = StepSelection { use_step: true, step_idx: 1, ..Default::default() };
        assert_eq!(view_subtitle(&cr, s).unwrap(), "Step 1 of 2, Load factor = 0.50000");
        assert_eq!(view_title("3D", "MODAL", None), "3D  [Result: MODAL]");
        assert_eq!(
            view_title("3D", "MODAL", Some("Mode 1, Period = 0.5 s".into())),
            "3D  [Result: MODAL]  Mode 1, Period = 0.5 s"
        );
    }
}
