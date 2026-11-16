//! Functions from file and time-history plot functions.
//!
//! "Function from file" reads a time-history function from a text file the way
//! the SAP2000 form of that name does (see `sap_format::function_file`) and
//! stores it as a user-defined function. "Plot functions" plots the response
//! of a time-history case at a joint and prints its maximum absolute value.

use eframe::egui;
use sap_format::function_file::{read_function_file, FunctionFileOptions, ItemFormat, ValueLayout};
use super::display_units::{self, Dimension};
use sap_model::{CaseResults, DynamicFunction, SapModel};
use sap_units::Units;
use std::path::Path;

/// Window state. It is reset when another model is loaded.
pub(super) struct State {
    pub(super) open: bool,
    tab: usize,
    // Function from file.
    name: String,
    path: String,
    header_lines: usize,
    prefix_chars: usize,
    points_per_line: usize,
    fixed_format: bool,
    chars_per_item: usize,
    equal_interval: bool,
    interval: f64,
    /// Points read by the last "Read file", or why that failed.
    loaded: Option<Result<Vec<(f64, f64)>, String>>,
    // Plot functions.
    case: String,
    joint: String,
    component: usize,
    quantity: Quantity,
    /// Named plot functions, the plot setup and the named sets. Reset with
    /// the window state when another model is loaded (`gui.rs` replaces the
    /// whole `State` there), so nothing here outlives its model.
    plots: PlotWindow,
}

impl Default for State {
    fn default() -> Self {
        Self {
            open: false,
            tab: 0,
            name: "FILEFUNC1".into(),
            path: String::new(),
            header_lines: 0,
            prefix_chars: 0,
            points_per_line: 1,
            fixed_format: false,
            chars_per_item: 10,
            equal_interval: false,
            interval: 0.02,
            loaded: None,
            case: String::new(),
            joint: String::new(),
            component: 0,
            quantity: Quantity::Displacement,
            plots: PlotWindow::default(),
        }
    }
}

/// Response quantity of a time-history case, as the vendor's form lists them.
/// Velocity and Reaction are not recorded by any case and are refused.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Quantity {
    Displacement,
    Velocity,
    RelativeAcceleration,
    AbsoluteAcceleration,
    Reaction,
}

impl Quantity {
    const ALL: [Quantity; 5] = [
        Quantity::Displacement,
        Quantity::Velocity,
        Quantity::RelativeAcceleration,
        Quantity::AbsoluteAcceleration,
        Quantity::Reaction,
    ];

    fn label(self) -> &'static str {
        match self {
            Quantity::Displacement => "Displacement",
            Quantity::Velocity => "Velocity",
            Quantity::RelativeAcceleration => "Relative acceleration",
            Quantity::AbsoluteAcceleration => "Absolute acceleration",
            Quantity::Reaction => "Reaction",
        }
    }
}

const COMPONENTS: [&str; 6] = ["U1", "U2", "U3", "R1", "R2", "R3"];

/// The reading options of the state's form.
fn options(state: &State) -> FunctionFileOptions {
    FunctionFileOptions {
        header_lines: state.header_lines,
        prefix_chars: state.prefix_chars,
        points_per_line: state.points_per_line,
        format: if state.fixed_format {
            ItemFormat::Fixed {
                chars_per_item: state.chars_per_item,
            }
        } else {
            ItemFormat::Free
        },
        layout: if state.equal_interval {
            ValueLayout::EqualInterval { dt: state.interval }
        } else {
            ValueLayout::Pairs
        },
    }
}

/// Reads the function file named in `path`; the error carries the line number.
fn read_points(path: &str, options: &FunctionFileOptions) -> Result<Vec<(f64, f64)>, String> {
    let path = path.trim();
    if path.is_empty() {
        return Err("Enter the path of the function file.".into());
    }
    read_function_file(Path::new(path), options).map_err(|e| e.to_string())
}

/// Stores `points` as the user-defined time-history function `name`. Nothing
/// changes when the model is locked, the name is in use, or the points are not
/// a valid function (at least two samples, nonnegative strictly increasing
/// times).
fn add_function(model: &mut SapModel, name: &str, points: &[(f64, f64)]) -> Result<String, String> {
    if model.is_locked {
        return Err("Unlock the model before adding functions.".into());
    }
    let name = name.trim();
    if model.functions.contains_key(name) {
        return Err(format!(
            "Function '{name}' already exists; choose another name."
        ));
    }
    let function = DynamicFunction {
        name: name.to_string(),
        is_time_history: true,
        damping_ratio: 0.05,
        points: points.to_vec(),
    };
    sap_api::set_function_definition(model, function).map_err(|e| e.to_string())?;
    Ok(format!(
        "Added time-history function {name} with {} points.",
        points.len()
    ))
}

/// Cases with a stored time history, and the joints of the first of them.
fn history_cases(model: &SapModel) -> Vec<&str> {
    model
        .case_results
        .iter()
        .filter(|(_, r)| !r.time_steps.is_empty())
        .map(|(name, _)| name.as_str())
        .collect()
}

/// `(time, value)` of one response component of one joint.
fn history_series(
    model: &SapModel,
    case: &str,
    joint: &str,
    component: usize,
    quantity: Quantity,
) -> Result<Vec<(f64, f64)>, String> {
    let results: &CaseResults = model
        .case_results
        .get(case)
        .ok_or_else(|| format!("Case '{case}' has no results; run the analysis first."))?;
    if results.time_steps.is_empty() {
        return Err(format!("Case '{case}' is not a time-history case."));
    }
    if component >= 6 {
        return Err(format!("Component index {component} is not 0..5."));
    }
    let traces = match quantity {
        Quantity::Displacement => &results.time_history_disp,
        Quantity::RelativeAcceleration | Quantity::AbsoluteAcceleration => {
            &results.time_history_accel
        }
        Quantity::Velocity | Quantity::Reaction => {
            return Err(format!(
                "{} is not recorded by time-history cases; only displacement and, for modal cases, accelerations are stored.",
                quantity.label()
            ))
        }
    };
    if traces.is_empty() {
        return Err(if quantity == Quantity::AbsoluteAcceleration {
            format!(
                "Case '{case}' has no absolute accelerations: only modal time-history cases record accelerations."
            )
        } else {
            format!(
                "Case '{case}' has no relative accelerations: only modal (mode-superposition) time-history cases record them."
            )
        });
    }
    let trace = traces
        .get(joint)
        .ok_or_else(|| format!("Joint '{joint}' has no time history in case '{case}'."))?;
    if trace.len() != results.time_steps.len() {
        return Err(format!(
            "Case '{case}' stores {} steps but {} values for joint '{joint}'.",
            results.time_steps.len(),
            trace.len()
        ));
    }
    let series: Vec<(f64, f64)> = results
        .time_steps
        .iter()
        .zip(trace)
        .map(|(t, u)| (*t, u[component]))
        .collect();
    if quantity != Quantity::AbsoluteAcceleration {
        return Ok(series);
    }
    let point = model
        .points
        .get(joint)
        .ok_or_else(|| format!("Joint '{joint}' does not exist in the model."))?;
    let ground = sap_solver::ground_acceleration_trace(model, case).map_err(|e| e.to_string())?;
    let rotation = sap_solver::frame::joint_local_to_global_matrix(point);
    absolute_acceleration(&series, &ground, &rotation, component)
}

/// Absolute acceleration of one joint component from its relative series
/// (stored in the joint's local axes) and the global ground acceleration
/// `a_g` at the same steps: for a translation `rel + (Rᵀ·a_g)[component]`,
/// that is `Σ_g R[g][component]·a_g[g]` with `R` the joint-local-to-global
/// matrix (`v_global = R·v_local`); a rotation is not changed by a ground
/// translation and equals its relative value.
fn absolute_acceleration(
    relative: &[(f64, f64)],
    ground: &[[f64; 3]],
    rotation: &[[f64; 3]; 3],
    component: usize,
) -> Result<Vec<(f64, f64)>, String> {
    if component >= 6 {
        return Err(format!("Component index {component} is not 0..5."));
    }
    if relative.len() != ground.len() {
        return Err(format!(
            "The record has {} steps but the ground acceleration has {} values.",
            relative.len(),
            ground.len()
        ));
    }
    Ok(relative
        .iter()
        .zip(ground)
        .map(|(&(t, rel), a)| {
            if component < 3 {
                let local: f64 = (0..3).map(|g| rotation[g][component] * a[g]).sum();
                (t, rel + local)
            } else {
                (t, rel)
            }
        })
        .collect())
}

/// The point of largest absolute value, the first one on a tie.
fn peak_of(points: &[(f64, f64)]) -> Option<(f64, f64)> {
    points
        .iter()
        .copied()
        .fold(None, |best: Option<(f64, f64)>, p| match best {
            Some(b) if b.1.abs() >= p.1.abs() => Some(b),
            _ => Some(p),
        })
}

/// Minimum and maximum of each bucket, in time order, so that a long record
/// keeps its peaks when drawn on a few hundred pixels.
fn decimate(points: &[(f64, f64)], max_points: usize) -> Vec<(f64, f64)> {
    if points.len() <= max_points || max_points < 4 {
        return points.to_vec();
    }
    let buckets = max_points / 2;
    let size = points.len().div_ceil(buckets);
    let mut out = Vec::with_capacity(max_points + 2);
    for chunk in points.chunks(size) {
        let (mut lo, mut hi) = (0, 0);
        for (i, p) in chunk.iter().enumerate() {
            if p.1 < chunk[lo].1 {
                lo = i;
            }
            if p.1 > chunk[hi].1 {
                hi = i;
            }
        }
        let (a, b) = if lo <= hi { (lo, hi) } else { (hi, lo) };
        out.push(chunk[a]);
        if b != a {
            out.push(chunk[b]);
        }
    }
    out
}

// ---------------------------------------------------------------------------
// Named plot functions (pure code; the window below only calls it)
// ---------------------------------------------------------------------------

/// What a plot function reads.
#[derive(Clone, Debug, PartialEq)]
enum Source {
    /// One component of one joint's response.
    Response {
        joint: String,
        quantity: Quantity,
        component: usize,
    },
    /// `sf·f(t)` of entry `entry` of the case's history loads, at the output
    /// steps.
    Load { entry: usize },
}

/// A named plot function: a source in one time-history case.
#[derive(Clone, Debug, PartialEq)]
struct PlotFunction {
    name: String,
    case: String,
    source: Source,
}

/// What is drawn: vertical functions, a horizontal one (time when `None`),
/// a time range (`(from, to)` in seconds; the whole record when `None`) and
/// the display units (the model's when `None`).
#[derive(Clone, Debug, Default, PartialEq)]
struct PlotSetup {
    vertical: Vec<PlotFunction>,
    horizontal: Option<PlotFunction>,
    range: Option<(f64, f64)>,
    units: Option<Units>,
}

/// A named plot setup kept for the session.
#[derive(Clone, Debug, PartialEq)]
struct NamedSet {
    name: String,
    setup: PlotSetup,
}

/// Value of the maximum or minimum of a trace and the time it occurs at.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Extreme {
    time: f64,
    value: f64,
}

/// One drawn trace: points `(x, y)` in display units (`x` is time in seconds
/// for a plot against time) and the extremes of `y`, as the vendor's legend
/// prints them.
#[derive(Clone, Debug, PartialEq)]
struct Curve {
    name: String,
    x_label: String,
    y_label: String,
    unit: String,
    points: Vec<(f64, f64)>,
    max: Extreme,
    min: Extreme,
}

/// Plot-function window state: the functions, the selection, the sets.
#[derive(Default)]
struct PlotWindow {
    functions: Vec<PlotFunction>,
    vertical: Vec<String>,
    horizontal: Option<String>,
    range_on: bool,
    from: f64,
    to: f64,
    units: Option<Units>,
    sets: Vec<NamedSet>,
    function_name: String,
    set_name: String,
    load_entry: usize,
    message: Option<Result<String, String>>,
}

/// Load-history targets that mean a ground acceleration (the spellings of
/// `sap_solver`'s modal history).
fn is_acceleration_target(target: &str) -> bool {
    matches!(
        target.trim().to_ascii_uppercase().as_str(),
        "U1" | "UX" | "X" | "U2" | "UY" | "Y" | "U3" | "UZ" | "Z"
    )
}

/// Time function value as the solvers read it: zero before the first point,
/// held at the last value after the last point, linear between, the later
/// value at a jump; times within `tol` of a point count as that point.
fn load_function_value(points: &[(f64, f64)], t: f64, tol: f64) -> f64 {
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

/// `(time, value)` of a plot function in model units.
fn function_series(model: &SapModel, f: &PlotFunction) -> Result<Vec<(f64, f64)>, String> {
    match &f.source {
        Source::Response {
            joint,
            quantity,
            component,
        } => history_series(model, &f.case, joint, *component, *quantity),
        Source::Load { entry } => {
            let results = model
                .case_results
                .get(&f.case)
                .ok_or_else(|| format!("Case '{}' has no results; run the analysis first.", f.case))?;
            if results.time_steps.is_empty() {
                return Err(format!("Case '{}' is not a time-history case.", f.case));
            }
            let case = model
                .load_cases
                .get(&f.case)
                .ok_or_else(|| format!("Case '{}' does not exist.", f.case))?;
            let (function, _, sf) = case.history_loads.get(*entry).ok_or_else(|| {
                format!(
                    "Case '{}' has no load entry {} (it has {}).",
                    f.case,
                    entry + 1,
                    case.history_loads.len()
                )
            })?;
            let points = model
                .functions
                .get(function)
                .ok_or_else(|| format!("Function '{function}' does not exist."))?;
            let tol = 1e-9 * case.time_step_dt;
            Ok(results
                .time_steps
                .iter()
                .map(|&t| (t, sf * load_function_value(&points.points, t, tol)))
                .collect())
        }
    }
}

/// Factor from model units to `display` units and the unit label of a plot
/// function. Rotations are in radians; a load entry that is not a ground
/// acceleration is a scale on a load pattern and has no unit.
fn function_units(
    model: &SapModel,
    f: &PlotFunction,
    display: Units,
) -> Result<(f64, String), String> {
    let from = model.present_units;
    let dim = |d: Dimension| (display_units::factor(d, from, display), display_units::label(d, display));
    Ok(match &f.source {
        Source::Response {
            quantity, component, ..
        } => match quantity {
            Quantity::Displacement => dim(if *component < 3 {
                Dimension::Length
            } else {
                Dimension::Rotation
            }),
            Quantity::RelativeAcceleration | Quantity::AbsoluteAcceleration => {
                if *component < 3 {
                    dim(Dimension::Acceleration)
                } else {
                    (1.0, "rad/s2".to_string())
                }
            }
            Quantity::Velocity | Quantity::Reaction => {
                return Err(format!("{} is not recorded.", quantity.label()))
            }
        },
        Source::Load { entry } => {
            let case = model
                .load_cases
                .get(&f.case)
                .ok_or_else(|| format!("Case '{}' does not exist.", f.case))?;
            let (_, target, _) = case.history_loads.get(*entry).ok_or_else(|| {
                format!("Case '{}' has no load entry {}.", f.case, entry + 1)
            })?;
            if is_acceleration_target(target) {
                dim(Dimension::Acceleration)
            } else {
                (1.0, String::new())
            }
        }
    })
}

/// Maximum and minimum of `(time, value)` points, the first on a tie.
fn extremes(points: &[(f64, f64)]) -> Option<(Extreme, Extreme)> {
    let first = points.first()?;
    let mut max = Extreme {
        time: first.0,
        value: first.1,
    };
    let mut min = max;
    for &(time, value) in points {
        if value > max.value {
            max = Extreme { time, value };
        }
        if value < min.value {
            min = Extreme { time, value };
        }
    }
    Some((max, min))
}

/// Checks a time range against a record `[first, last]` and returns the
/// indices of the steps inside it (inclusive, with a tolerance of
/// `1e-9·(last - first)`).
fn range_indices(
    times: &[f64],
    range: Option<(f64, f64)>,
) -> Result<std::ops::Range<usize>, String> {
    let (Some(&first), Some(&last)) = (times.first(), times.last()) else {
        return Err("The record has no steps.".into());
    };
    let Some((from, to)) = range else {
        return Ok(0..times.len());
    };
    if !from.is_finite() || !to.is_finite() {
        return Err("The time range must be finite.".into());
    }
    if from >= to {
        return Err(format!(
            "The time range {from} to {to} s is empty: the start must be before the end."
        ));
    }
    let tol = 1e-9 * (last - first).abs().max(f64::MIN_POSITIVE);
    if from < first - tol || to > last + tol {
        return Err(format!(
            "The time range {from} to {to} s is outside the record {first} to {last} s."
        ));
    }
    let lo = times.iter().position(|&t| t >= from - tol);
    let hi = times.iter().rposition(|&t| t <= to + tol);
    match (lo, hi) {
        (Some(lo), Some(hi)) if lo <= hi => Ok(lo..hi + 1),
        _ => Err(format!(
            "The time range {from} to {to} s contains no output step."
        )),
    }
}

/// The traces of a plot setup in display units, with their extremes (value
/// and time of the step). A horizontal function must belong to the same case
/// as every vertical one; its values and the vertical values are paired at
/// equal steps. The time range selects the steps of the record.
fn build_plot(model: &SapModel, setup: &PlotSetup) -> Result<Vec<Curve>, String> {
    if setup.vertical.is_empty() {
        return Err("Select at least one function for the vertical axis.".into());
    }
    let display = setup.units.unwrap_or(model.present_units);
    let horizontal = match &setup.horizontal {
        None => None,
        Some(h) => {
            if let Some(other) = setup.vertical.iter().find(|v| v.case != h.case) {
                return Err(format!(
                    "Cannot plot '{}' (case {}) against '{}' (case {}): a horizontal function must belong to the same case.",
                    other.name, other.case, h.name, h.case
                ));
            }
            let (factor, unit) = function_units(model, h, display)?;
            let series = function_series(model, h)?;
            Some((h, factor, unit, series))
        }
    };
    let mut curves = Vec::with_capacity(setup.vertical.len());
    for f in &setup.vertical {
        let (factor, unit) = function_units(model, f, display)?;
        let series = function_series(model, f)?;
        let times: Vec<f64> = series.iter().map(|p| p.0).collect();
        let selected = range_indices(&times, setup.range)?;
        let y: Vec<(f64, f64)> = series[selected.clone()]
            .iter()
            .map(|&(t, v)| (t, v * factor))
            .collect();
        let (max, min) = extremes(&y).ok_or_else(|| format!("'{}' has no steps.", f.name))?;
        let (points, x_label) = match &horizontal {
            None => (y.clone(), "Time [s]".to_string()),
            Some((h, hfactor, hunit, hseries)) => {
                if hseries.len() != series.len()
                    || hseries.iter().zip(&series).any(|(a, b)| a.0 != b.0)
                {
                    return Err(format!(
                        "'{}' and '{}' are not recorded at the same steps.",
                        f.name, h.name
                    ));
                }
                let pairs = hseries[selected.clone()]
                    .iter()
                    .zip(&y)
                    .map(|(x, y)| (x.1 * hfactor, y.1))
                    .collect();
                (pairs, format!("{} [{hunit}]", h.name))
            }
        };
        curves.push(Curve {
            name: f.name.clone(),
            x_label,
            y_label: format!("{} [{unit}]", f.name),
            unit,
            points,
            max,
            min,
        });
    }
    Ok(curves)
}

/// The legend line of a curve.
fn legend_text(curve: &Curve) -> String {
    format!(
        "{}: max {} at t = {} s, min {} at t = {} s {}",
        curve.name,
        format_value(curve.max.value),
        format_value(curve.max.time),
        format_value(curve.min.value),
        format_value(curve.min.time),
        curve.unit
    )
}

/// A number with five decimals in scientific notation, `0` for zero.
fn format_value(v: f64) -> String {
    if v == 0.0 {
        "0".into()
    } else {
        format!("{v:.5e}")
    }
}

/// Bounds `[x0, x1, y0, y1]` of all curves, widened where a range is flat.
fn plot_bounds(curves: &[Curve]) -> Option<[f64; 4]> {
    let mut b: Option<[f64; 4]> = None;
    for &(x, y) in curves.iter().flat_map(|c| c.points.iter()) {
        b = Some(match b {
            None => [x, x, y, y],
            Some([x0, x1, y0, y1]) => [x0.min(x), x1.max(x), y0.min(y), y1.max(y)],
        });
    }
    let [mut x0, mut x1, mut y0, mut y1] = b?;
    if x1 - x0 <= 0.0 {
        x0 -= 0.5;
        x1 += 0.5;
    }
    if y1 - y0 <= 0.0 {
        y0 -= 0.5;
        y1 += 0.5;
    }
    Some([x0, x1, y0, y1])
}

/// Position of a point in the unit square of `bounds` (`y` up).
fn to_unit_square(bounds: [f64; 4], (x, y): (f64, f64)) -> (f64, f64) {
    (
        (x - bounds[0]) / (bounds[1] - bounds[0]),
        (y - bounds[2]) / (bounds[3] - bounds[2]),
    )
}

/// Adds a plot function; refuses an empty or used name (case-insensitive)
/// and a component outside 0..5, leaving the list unchanged.
fn add_plot_function(list: &mut Vec<PlotFunction>, f: PlotFunction) -> Result<String, String> {
    let name = f.name.trim().to_string();
    if name.is_empty() {
        return Err("Enter a name for the plot function.".into());
    }
    if list.iter().any(|g| g.name.eq_ignore_ascii_case(&name)) {
        return Err(format!(
            "A plot function named '{name}' already exists; choose another name."
        ));
    }
    if let Source::Response { component, .. } = &f.source {
        if *component >= 6 {
            return Err(format!("Component index {component} is not 0..5."));
        }
    }
    list.push(PlotFunction { name: name.clone(), ..f });
    Ok(format!("Added plot function {name}."))
}

/// Keeps `setup` under `name` for the session; refuses an empty or used name
/// (case-insensitive), leaving the sets unchanged.
fn save_set(sets: &mut Vec<NamedSet>, name: &str, setup: PlotSetup) -> Result<String, String> {
    let name = name.trim();
    if name.is_empty() {
        return Err("Enter a name for the set.".into());
    }
    if sets.iter().any(|s| s.name.eq_ignore_ascii_case(name)) {
        return Err(format!("A set named '{name}' already exists; choose another name."));
    }
    sets.push(NamedSet {
        name: name.to_string(),
        setup,
    });
    Ok(format!("Saved set {name}."))
}

/// Makes the setup of the set `name` current: its functions join the
/// function list (a function with the name of one that already exists must
/// be the same, otherwise nothing changes) and the selection, range and
/// units are returned for the window state.
fn recall_set(
    list: &mut Vec<PlotFunction>,
    sets: &[NamedSet],
    name: &str,
) -> Result<PlotSetup, String> {
    let set = sets
        .iter()
        .find(|s| s.name.eq_ignore_ascii_case(name.trim()))
        .ok_or_else(|| format!("There is no set named '{}'.", name.trim()))?;
    let members = set.setup.vertical.iter().chain(set.setup.horizontal.iter());
    for f in members.clone() {
        if let Some(existing) = list.iter().find(|g| g.name == f.name) {
            if existing != f {
                return Err(format!(
                    "Set '{}' uses plot function '{}', which is defined differently now.",
                    set.name, f.name
                ));
            }
        }
    }
    for f in members {
        if !list.iter().any(|g| g.name == f.name) {
            list.push(f.clone());
        }
    }
    Ok(set.setup.clone())
}

/// The setup the window state describes: the named selection looked up in
/// the function list.
fn current_setup(w: &PlotWindow) -> Result<PlotSetup, String> {
    let find = |name: &str| {
        w.functions
            .iter()
            .find(|f| f.name == name)
            .cloned()
            .ok_or_else(|| format!("There is no plot function '{name}'."))
    };
    Ok(PlotSetup {
        vertical: w
            .vertical
            .iter()
            .map(|n| find(n))
            .collect::<Result<_, _>>()?,
        horizontal: w.horizontal.as_deref().map(find).transpose()?,
        range: w.range_on.then_some((w.from, w.to)),
        units: w.units,
    })
}

fn draw_curves(ui: &mut egui::Ui, curves: &[Curve]) {
    const COLORS: [egui::Color32; 6] = [
        egui::Color32::from_rgb(110, 200, 255),
        egui::Color32::from_rgb(255, 170, 80),
        egui::Color32::from_rgb(140, 230, 120),
        egui::Color32::from_rgb(240, 120, 200),
        egui::Color32::from_rgb(240, 230, 100),
        egui::Color32::from_rgb(190, 150, 255),
    ];
    let size = egui::vec2(ui.available_width().clamp(240.0, 720.0), 220.0);
    let (rect, _) = ui.allocate_exact_size(size, egui::Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(16, 20, 28));
    let Some(bounds) = plot_bounds(curves) else {
        return;
    };
    let to_screen = |p: (f64, f64)| {
        let (u, v) = to_unit_square(bounds, p);
        egui::pos2(
            rect.left() + u as f32 * rect.width(),
            rect.bottom() - v as f32 * rect.height(),
        )
    };
    if bounds[2] <= 0.0 && bounds[3] >= 0.0 {
        let zero = to_screen((bounds[0], 0.0)).y;
        painter.line_segment(
            [egui::pos2(rect.left(), zero), egui::pos2(rect.right(), zero)],
            egui::Stroke::new(1.0_f32, egui::Color32::from_gray(90)),
        );
    }
    for (i, curve) in curves.iter().enumerate() {
        let line: Vec<egui::Pos2> = decimate(&curve.points, 2 * rect.width().max(2.0) as usize)
            .into_iter()
            .map(to_screen)
            .collect();
        painter.add(egui::Shape::line(
            line,
            egui::Stroke::new(1.5_f32, COLORS[i % COLORS.len()]),
        ));
    }
}

fn draw_curve(ui: &mut egui::Ui, points: &[(f64, f64)]) {
    let size = egui::vec2(ui.available_width().clamp(240.0, 720.0), 170.0);
    let (rect, _) = ui.allocate_exact_size(size, egui::Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(16, 20, 28));
    if points.len() < 2 {
        return;
    }
    let t0 = points[0].0;
    let t1 = points[points.len() - 1].0;
    let (mut lo, mut hi) = (0.0f64, 0.0f64);
    for p in points {
        lo = lo.min(p.1);
        hi = hi.max(p.1);
    }
    if hi - lo <= 0.0 || t1 - t0 <= 0.0 {
        return;
    }
    let to_screen = |(t, v): (f64, f64)| {
        egui::pos2(
            rect.left() + ((t - t0) / (t1 - t0)) as f32 * rect.width(),
            rect.bottom() - ((v - lo) / (hi - lo)) as f32 * rect.height(),
        )
    };
    let zero = to_screen((t0, 0.0)).y;
    painter.line_segment(
        [
            egui::pos2(rect.left(), zero),
            egui::pos2(rect.right(), zero),
        ],
        egui::Stroke::new(1.0_f32, egui::Color32::from_gray(90)),
    );
    let line: Vec<egui::Pos2> = decimate(points, 2 * rect.width().max(2.0) as usize)
        .into_iter()
        .map(to_screen)
        .collect();
    painter.add(egui::Shape::line(
        line,
        egui::Stroke::new(1.5_f32, egui::Color32::from_rgb(110, 200, 255)),
    ));
}

fn function_tab(
    ui: &mut egui::Ui,
    state: &mut State,
    model: &mut SapModel,
    files: &mut super::file_browser::State,
) -> Option<Result<String, String>> {
    let mut outcome = None;
    ui.label(
        "Reads time-history values from a text file: header lines are skipped, lines starting with $ are skipped, and reading stops at a blank line or the end of the file.",
    );
    ui.horizontal(|ui| {
        ui.label("File path:");
        ui.text_edit_singleline(&mut state.path);
        if ui.button("Browse...").clicked() {
            super::file_browser::open_for(
                files,
                super::file_browser::Purpose::FunctionFile,
                std::path::Path::new(state.path.trim()),
                &[],
                false,
            );
        }
    });
    if let Some(path) =
        super::file_browser::take_choice(files, super::file_browser::Purpose::FunctionFile)
    {
        state.path = path.display().to_string();
    }
    egui::Grid::new("function_file_options")
        .num_columns(2)
        .show(ui, |ui| {
            ui.label("Header lines to skip:");
            ui.add(egui::DragValue::new(&mut state.header_lines).range(0..=100_000));
            ui.end_row();
            ui.label("Prefix characters per line to skip:");
            ui.add(egui::DragValue::new(&mut state.prefix_chars).range(0..=1000));
            ui.end_row();
            ui.label("Number of points per line:");
            ui.add(egui::DragValue::new(&mut state.points_per_line).range(1..=1000));
            ui.end_row();
        });
    ui.horizontal(|ui| {
        ui.label("Format:");
        ui.selectable_value(&mut state.fixed_format, false, "Free");
        ui.selectable_value(&mut state.fixed_format, true, "Fixed");
        if state.fixed_format {
            ui.label("characters per item:");
            ui.add(egui::DragValue::new(&mut state.chars_per_item).range(1..=200));
        }
    });
    ui.horizontal(|ui| {
        ui.label("Values are:");
        ui.selectable_value(&mut state.equal_interval, false, "Time and value pairs");
        ui.selectable_value(
            &mut state.equal_interval,
            true,
            "Values at equal intervals of",
        );
        if state.equal_interval {
            ui.add(
                egui::DragValue::new(&mut state.interval)
                    .speed(0.001)
                    .max_decimals(8),
            );
            ui.label("s (the first value is at t = 0)");
        }
    });
    ui.horizontal(|ui| {
        if ui.button("Read file").clicked() {
            state.loaded = Some(read_points(&state.path, &options(state)));
        }
        ui.label("Function name:");
        ui.text_edit_singleline(&mut state.name);
    });
    match &state.loaded {
        None => {
            ui.label("Read the file to see the function.");
        }
        Some(Err(error)) => {
            ui.colored_label(egui::Color32::LIGHT_RED, error);
        }
        Some(Ok(points)) => {
            let duration = points.last().map_or(0.0, |p| p.0) - points.first().map_or(0.0, |p| p.0);
            ui.label(format!(
                "{} points, duration {duration} s, largest |value| {}",
                points.len(),
                peak_of(points).map_or(0.0, |p| p.1.abs())
            ));
            draw_curve(ui, points);
            if ui.button("Add as function").clicked() {
                outcome = Some(add_function(model, &state.name, points));
            }
            ui.label("The function is stored in the model as user-defined values; the file is not linked.");
        }
    }
    outcome
}

fn plot_tab(ui: &mut egui::Ui, state: &mut State, model: &SapModel) {
    let cases = history_cases(model);
    if cases.is_empty() {
        ui.label("Run an analysis with a time-history case first.");
        return;
    }
    if !cases.contains(&state.case.as_str()) {
        state.case = cases[0].to_string();
    }
    ui.label(format!(
        "Model units: {}; time in seconds; acceleration is length per second squared.",
        model.present_units.s2k_label()
    ));
    ui.horizontal(|ui| {
        ui.label("Case:");
        egui::ComboBox::from_id_salt("plot_function_case")
            .selected_text(state.case.clone())
            .show_ui(ui, |ui| {
                for name in &cases {
                    ui.selectable_value(&mut state.case, name.to_string(), *name);
                }
            });
        ui.label("Joint:");
        egui::ComboBox::from_id_salt("plot_function_joint")
            .selected_text(state.joint.clone())
            .show_ui(ui, |ui| {
                if let Some(result) = model.case_results.get(&state.case) {
                    for joint in result.time_history_disp.keys() {
                        ui.selectable_value(&mut state.joint, joint.clone(), joint);
                    }
                }
            });
        ui.label("Component:");
        egui::ComboBox::from_id_salt("plot_function_component")
            .selected_text(COMPONENTS[state.component.min(5)])
            .show_ui(ui, |ui| {
                for (i, name) in COMPONENTS.iter().enumerate() {
                    ui.selectable_value(&mut state.component, i, *name);
                }
            });
    });
    ui.horizontal(|ui| {
        ui.label("Quantity:");
        for quantity in Quantity::ALL {
            ui.selectable_value(&mut state.quantity, quantity, quantity.label());
        }
    });
    if state.joint.is_empty() {
        if let Some(first) = model
            .case_results
            .get(&state.case)
            .and_then(|r| r.time_history_disp.keys().next())
        {
            state.joint = first.clone();
        }
    }
    match history_series(
        model,
        &state.case,
        &state.joint,
        state.component,
        state.quantity,
    ) {
        Err(error) => {
            ui.colored_label(egui::Color32::LIGHT_RED, error);
        }
        Ok(series) => {
            if let Some((t, value)) = peak_of(&series) {
                ui.label(format!(
                    "Maximum absolute value {} at t = {t} s.",
                    value.abs()
                ));
            }
            draw_curve(ui, &series);
        }
    }
    ui.separator();
    plot_functions_section(ui, state, model);
}

/// Named plot functions, several traces, a horizontal function, a time
/// range, display units, the legend with extremes, and named sets. All
/// numbers come from [`build_plot`] and [`legend_text`].
fn plot_functions_section(ui: &mut egui::Ui, state: &mut State, model: &SapModel) {
    ui.heading("Named plot functions");
    ui.label("Add functions from the selection above (joint, quantity, component) or from a load of the case, tick the vertical ones, and optionally pick a horizontal function of the same case instead of time.");
    // Actions are collected and applied after the widgets are drawn.
    let mut add_response = false;
    let mut add_load = false;
    let mut remove: Option<usize> = None;
    let mut save = false;
    let mut recall: Option<String> = None;
    let mut forget: Option<String> = None;
    let case_name = state.case.clone();
    let entries: Vec<String> = model
        .load_cases
        .get(&case_name)
        .map(|c| {
            c.history_loads
                .iter()
                .enumerate()
                .map(|(i, (f, t, sf))| format!("{}: {f} -> {t} x {sf}", i + 1))
                .collect()
        })
        .unwrap_or_default();
    let w = &mut state.plots;
    ui.horizontal(|ui| {
        ui.label("Name:");
        ui.text_edit_singleline(&mut w.function_name);
        add_response = ui.button("Add response function").clicked();
    });
    ui.horizontal(|ui| {
        ui.label("Load entry:");
        let selected = entries
            .get(w.load_entry)
            .cloned()
            .unwrap_or_else(|| "none".into());
        egui::ComboBox::from_id_salt("plot_function_load_entry")
            .selected_text(selected)
            .show_ui(ui, |ui| {
                for (i, text) in entries.iter().enumerate() {
                    ui.selectable_value(&mut w.load_entry, i, text);
                }
            });
        add_load = ui.button("Add load function").clicked();
    });
    for (i, f) in w.functions.iter().enumerate() {
        ui.horizontal(|ui| {
            let mut on = w.vertical.contains(&f.name);
            if ui.checkbox(&mut on, "vertical").changed() {
                if on {
                    w.vertical.push(f.name.clone());
                } else {
                    w.vertical.retain(|n| *n != f.name);
                }
            }
            let mut horizontal = w.horizontal.as_deref() == Some(f.name.as_str());
            if ui.checkbox(&mut horizontal, "horizontal").changed() {
                w.horizontal = horizontal.then(|| f.name.clone());
            }
            ui.label(format!("{} ({})", f.name, describe(f)));
            if ui.button("Remove").clicked() {
                remove = Some(i);
            }
        });
    }
    ui.horizontal(|ui| {
        ui.checkbox(&mut w.range_on, "Time range:");
        if w.range_on {
            ui.add(egui::DragValue::new(&mut w.from).speed(0.01));
            ui.label("to");
            ui.add(egui::DragValue::new(&mut w.to).speed(0.01));
            ui.label("s");
        }
        ui.label("Units:");
        egui::ComboBox::from_id_salt("plot_function_units")
            .selected_text(
                w.units
                    .map_or("Model units".to_string(), |u| u.s2k_label().to_string()),
            )
            .show_ui(ui, |ui| {
                ui.selectable_value(&mut w.units, None, "Model units");
                for u in Units::ALL {
                    ui.selectable_value(&mut w.units, Some(u), u.s2k_label());
                }
            });
    });
    match current_setup(w).and_then(|setup| build_plot(model, &setup)) {
        Err(error) => {
            ui.colored_label(egui::Color32::LIGHT_RED, error);
        }
        Ok(curves) => {
            for curve in &curves {
                ui.label(legend_text(curve));
            }
            if let Some(first) = curves.first() {
                ui.label(format!("Horizontal: {}; vertical: {}", first.x_label, first.y_label));
            }
            draw_curves(ui, &curves);
        }
    }
    ui.horizontal(|ui| {
        ui.label("Set name:");
        ui.text_edit_singleline(&mut w.set_name);
        save = ui.button("Save set").clicked();
    });
    for set in &w.sets {
        ui.horizontal(|ui| {
            ui.label(&set.name);
            if ui.button("Use").clicked() {
                recall = Some(set.name.clone());
            }
            if ui.button("Delete").clicked() {
                forget = Some(set.name.clone());
            }
        });
    }
    if let Some(Err(error)) = &w.message {
        ui.colored_label(egui::Color32::LIGHT_RED, error);
    } else if let Some(Ok(text)) = &w.message {
        ui.label(text);
    }

    // Apply.
    if add_response {
        let f = PlotFunction {
            name: w.function_name.clone(),
            case: state.case.clone(),
            source: Source::Response {
                joint: state.joint.clone(),
                quantity: state.quantity,
                component: state.component,
            },
        };
        w.message = Some(add_plot_function(&mut w.functions, f));
    }
    if add_load {
        let f = PlotFunction {
            name: w.function_name.clone(),
            case: state.case.clone(),
            source: Source::Load { entry: w.load_entry },
        };
        w.message = Some(if w.load_entry >= entries.len() {
            Err(format!("Case '{}' has no load entries.", state.case))
        } else {
            add_plot_function(&mut w.functions, f)
        });
    }
    if let Some(i) = remove {
        let name = w.functions.remove(i).name;
        w.vertical.retain(|n| *n != name);
        if w.horizontal.as_deref() == Some(name.as_str()) {
            w.horizontal = None;
        }
    }
    if save {
        w.message = Some(
            current_setup(w).and_then(|setup| save_set(&mut w.sets, &w.set_name.clone(), setup)),
        );
    }
    if let Some(name) = recall {
        w.message = Some(match recall_set(&mut w.functions, &w.sets, &name) {
            Ok(setup) => {
                w.vertical = setup.vertical.iter().map(|f| f.name.clone()).collect();
                w.horizontal = setup.horizontal.map(|f| f.name);
                w.range_on = setup.range.is_some();
                let (from, to) = setup.range.unwrap_or((w.from, w.to));
                w.from = from;
                w.to = to;
                w.units = setup.units;
                Ok(format!("Using set {name}."))
            }
            Err(e) => Err(e),
        });
    }
    if let Some(name) = forget {
        w.sets.retain(|s| s.name != name);
    }
}

/// One-line description of a plot function for the list.
fn describe(f: &PlotFunction) -> String {
    match &f.source {
        Source::Response {
            joint,
            quantity,
            component,
        } => format!(
            "{}, joint {joint}, {}, {}",
            f.case,
            quantity.label(),
            COMPONENTS[(*component).min(5)]
        ),
        Source::Load { entry } => format!("{}, load entry {}", f.case, entry + 1),
    }
}

/// Draws the window when it is open. Returns the outcome of a command the
/// user ran in this frame: a status message, or why it was refused.
pub(super) fn render(
    ctx: &egui::Context,
    state: &mut State,
    model: &mut SapModel,
    files: &mut super::file_browser::State,
) -> Option<Result<String, String>> {
    if !state.open {
        return None;
    }
    let mut open = state.open;
    let mut outcome = None;
    egui::Window::new("Functions from File and Plot Functions")
        .open(&mut open)
        .default_width(620.0)
        .default_height(760.0)
        .vscroll(true)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.selectable_value(&mut state.tab, 0, "Function from file");
                ui.selectable_value(&mut state.tab, 1, "Plot functions");
            });
            ui.separator();
            if state.tab == 0 {
                outcome = function_tab(ui, state, model, files);
            } else {
                plot_tab(ui, state, model);
            }
        });
    state.open = open;
    outcome
}

#[cfg(test)]
mod tests {
    use super::*;
    use sap_units::Units;

    fn model_with_history(with_accel: bool) -> SapModel {
        let mut model = SapModel::new(Units::KipInF);
        let mut result = CaseResults::default();
        result.time_steps = vec![0.0, 0.1, 0.2, 0.3];
        let trace = |scale: f64| -> Vec<[f64; 6]> {
            [0.0, 1.0, -3.0, 2.0]
                .iter()
                .map(|v| [v * scale, 0.0, 0.0, 0.0, 0.0, 0.0])
                .collect()
        };
        result.time_history_disp.insert("J".into(), trace(1.0));
        if with_accel {
            result.time_history_accel.insert("J".into(), trace(10.0));
        }
        model.case_results.insert("H".into(), result);
        model
    }

    /// A unique file in the system temporary directory.
    fn temp_file(tag: &str, text: &str) -> std::path::PathBuf {
        let path = std::env::temp_dir().join(format!(
            "structurax_function_file_{}_{tag}.txt",
            std::process::id()
        ));
        std::fs::write(&path, text).unwrap();
        path
    }

    #[test]
    fn reading_uses_the_form_options_and_reports_the_line_of_an_error() {
        let path = temp_file("equal", "title\n3 POINTS AT .5 SEC\n1.5 2.5 -4\n");
        let mut state = State::default();
        state.path = path.display().to_string();
        state.header_lines = 2;
        state.points_per_line = 3;
        state.equal_interval = true;
        state.interval = 0.5;
        let points = read_points(&state.path, &options(&state)).unwrap();
        assert_eq!(points, vec![(0.0, 1.5), (0.5, 2.5), (1.0, -4.0)]);
        // The same file read as pairs fails on the odd item count of line 3.
        state.equal_interval = false;
        state.points_per_line = 2;
        let error = read_points(&state.path, &options(&state)).unwrap_err();
        assert!(error.contains("line 3"), "{error}");
        assert!(read_points("  ", &options(&state)).is_err());
        assert!(read_points("/nonexistent/structurax/none.th", &options(&state)).is_err());
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn adding_a_function_validates_and_refuses_without_changing_the_model() {
        let mut model = SapModel::new(Units::KipInF);
        model.case_results.insert("OLD".into(), Default::default());
        let points = [(0.0, 0.0), (0.5, 1.0), (1.0, 0.0)];
        // Invalid function: one sample only.
        let before = model.clone();
        assert!(add_function(&mut model, "ONE", &points[..1]).is_err());
        // Invalid name.
        assert!(add_function(&mut model, "  ", &points).is_err());
        assert_eq!(model, before);
        // Locked model.
        model.is_locked = true;
        let before = model.clone();
        assert!(add_function(&mut model, "F", &points)
            .unwrap_err()
            .contains("Unlock"));
        assert_eq!(model, before);
        model.is_locked = false;
        // Success stores the points as a time-history function and drops results.
        let message = add_function(&mut model, "F", &points).unwrap();
        assert!(message.contains("3 points"));
        assert_eq!(model.functions["F"].points, points.to_vec());
        assert!(model.functions["F"].is_time_history);
        assert!(model.case_results.is_empty());
        // The name is now taken.
        assert!(add_function(&mut model, "F", &points)
            .unwrap_err()
            .contains("already exists"));
    }

    #[test]
    fn series_and_peak_come_from_the_stored_traces() {
        let model = model_with_history(true);
        let u = history_series(&model, "H", "J", 0, Quantity::Displacement).unwrap();
        assert_eq!(u, vec![(0.0, 0.0), (0.1, 1.0), (0.2, -3.0), (0.3, 2.0)]);
        assert_eq!(peak_of(&u), Some((0.2, -3.0)));
        let a = history_series(&model, "H", "J", 0, Quantity::RelativeAcceleration).unwrap();
        assert_eq!(peak_of(&a), Some((0.2, -30.0)));
        // Other components are zero; the peak of an all-zero series is its first point.
        let z = history_series(&model, "H", "J", 2, Quantity::Displacement).unwrap();
        assert_eq!(peak_of(&z), Some((0.0, 0.0)));
        assert_eq!(peak_of(&[]), None);
        assert_eq!(history_cases(&model), vec!["H"]);
    }

    #[test]
    fn plotting_refuses_what_has_no_stored_history() {
        let model = model_with_history(false);
        let error =
            history_series(&model, "H", "J", 0, Quantity::RelativeAcceleration).unwrap_err();
        assert!(error.contains("modal"), "{error}");
        assert!(history_series(&model, "NOPE", "J", 0, Quantity::Displacement).is_err());
        assert!(history_series(&model, "H", "K", 0, Quantity::Displacement).is_err());
        assert!(history_series(&model, "H", "J", 6, Quantity::Displacement).is_err());
        let mut model = model;
        model.case_results["H"].time_steps.clear();
        assert!(history_series(&model, "H", "J", 0, Quantity::Displacement)
            .unwrap_err()
            .contains("not a time-history"));
        // A trace of another length than the steps is reported, not truncated.
        let mut model = model_with_history(false);
        model.case_results["H"].time_history_disp["J"].pop();
        assert!(history_series(&model, "H", "J", 0, Quantity::Displacement).is_err());
    }

    #[test]
    fn decimation_keeps_the_extremes_of_a_long_record() {
        let mut points: Vec<(f64, f64)> = (0..40_000)
            .map(|i| (i as f64 * 0.001, (i as f64 * 0.01).sin()))
            .collect();
        points[12_345].1 = 7.5;
        points[30_001].1 = -9.25;
        let reduced = decimate(&points, 1000);
        assert!(reduced.len() <= 1002, "{}", reduced.len());
        assert!(reduced.contains(&points[12_345]));
        assert!(reduced.contains(&points[30_001]));
        assert!(reduced.windows(2).all(|w| w[0].0 < w[1].0));
        // A short series is returned unchanged.
        assert_eq!(decimate(&points[..10], 1000), points[..10].to_vec());
    }

    fn frame(
        ctx: &egui::Context,
        time: f64,
        events: Vec<egui::Event>,
        state: &mut State,
        model: &mut SapModel,
    ) -> (egui::FullOutput, Option<Result<String, String>>) {
        let mut outcome = None;
        let out = ctx.run(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::Vec2::new(1400., 1200.),
                )),
                time: Some(time),
                events,
                ..Default::default()
            },
            |ctx| {
                if let Some(result) = render(ctx, state, model, &mut Default::default()) {
                    outcome = Some(result);
                }
            },
        );
        (out, outcome)
    }

    fn texts(out: &egui::FullOutput) -> Vec<String> {
        out.shapes
            .iter()
            .filter_map(|shape| match &shape.shape {
                egui::epaint::Shape::Text(text) => Some(text.galley.job.text.clone()),
                _ => None,
            })
            .collect()
    }

    fn center_of(out: &egui::FullOutput, label: &str) -> Option<egui::Pos2> {
        out.shapes.iter().find_map(|shape| match &shape.shape {
            egui::epaint::Shape::Text(text) if text.galley.job.text == label => {
                Some(text.pos + text.galley.size() * 0.5)
            }
            _ => None,
        })
    }

    fn click(
        ctx: &egui::Context,
        time: f64,
        pos: egui::Pos2,
        state: &mut State,
        model: &mut SapModel,
    ) -> Option<Result<String, String>> {
        let button = |pressed| egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed,
            modifiers: Default::default(),
        };
        let _ = frame(
            ctx,
            time,
            vec![egui::Event::PointerMoved(pos), button(true)],
            state,
            model,
        );
        frame(ctx, time + 0.1, vec![button(false)], state, model).1
    }

    #[test]
    fn window_renders_both_tabs_and_a_click_adds_the_function() {
        let ctx = egui::Context::default();
        let mut model = model_with_history(true);
        let mut state = State::default();
        // Closed: nothing is drawn and nothing is reported.
        let (out, outcome) = frame(&ctx, 0.5, vec![], &mut state, &mut model);
        assert!(out.shapes.is_empty() && outcome.is_none());

        // Function tab with a file read through the same function the button uses.
        let path = temp_file("window", "0 0\n0.5 2\n1 0\n");
        state.open = true;
        state.path = path.display().to_string();
        state.points_per_line = 1;
        state.loaded = Some(read_points(&state.path, &options(&state)));
        let mut time = 1.0;
        let mut out = None;
        for _ in 0..3 {
            time += 1.0;
            out = Some(frame(&ctx, time, vec![], &mut state, &mut model).0);
        }
        let out = out.unwrap();
        assert!(texts(&out)
            .iter()
            .any(|t| t.contains("3 points, duration 1 s")));
        let pos = center_of(&out, "Add as function").expect("button is drawn");
        let outcome = click(&ctx, time + 1.0, pos, &mut state, &mut model);
        assert!(outcome.expect("click reports").is_ok());
        assert_eq!(model.functions["FILEFUNC1"].points.len(), 3);
        std::fs::remove_file(path).unwrap();

        // Plot tab: the case, joint and maximum are shown.
        let mut model = model_with_history(true);
        state.tab = 1;
        let mut out = None;
        for _ in 0..3 {
            time += 2.0;
            out = Some(frame(&ctx, time, vec![], &mut state, &mut model).0);
        }
        let shown = texts(&out.unwrap());
        assert!(shown
            .iter()
            .any(|t| t.contains("Maximum absolute value 3 at t = 0.2 s")));
        assert_eq!(state.case, "H");
        assert_eq!(state.joint, "J");
    }

    // -----------------------------------------------------------------
    // Plot functions: absolute acceleration, named functions, sets
    // -----------------------------------------------------------------

    use sap_model::{LoadCase, LoadCaseType, LoadPattern, LoadPatternType};

    fn analysis_model(active: [bool; 6]) -> SapModel {
        let mut m = SapModel::default();
        m.load_cases.clear();
        m.load_patterns.clear();
        m.active_dofs = active;
        m.load_patterns.insert(
            "P".into(),
            LoadPattern {
                name: "P".into(),
                pat_type: LoadPatternType::Other,
                self_wt_multiplier: 0.0,
            },
        );
        m
    }

    fn add_function_points(m: &mut SapModel, name: &str, points: Vec<(f64, f64)>) {
        m.functions.insert(
            name.into(),
            DynamicFunction {
                name: name.into(),
                is_time_history: true,
                damping_ratio: 0.0,
                points,
            },
        );
    }

    /// Modal history case "HIST" of modal case "MODAL" and analysis.
    fn run_modal_history(
        m: &mut SapModel,
        modes: usize,
        xi: f64,
        dt: f64,
        steps: usize,
        loads: Vec<(&str, &str, f64)>,
    ) {
        m.load_cases
            .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", modes));
        let mut case = LoadCase::linear_static("HIST", "P", 1.0);
        case.case_type = LoadCaseType::LinearHistory;
        case.loads.clear();
        case.modal_case = Some("MODAL".into());
        case.modal_damping = xi;
        case.time_step_dt = dt;
        case.num_time_steps = steps;
        case.history_loads = loads
            .into_iter()
            .map(|(f, p, s)| (f.to_string(), p.to_string(), s))
            .collect();
        m.load_cases.insert("HIST".into(), case);
        sap_solver::analyze_model(m).unwrap();
    }

    /// m = 2, k = 50 (omega = 5), ground step of acceleration 1 in X.
    fn single_mass(xi: f64) -> SapModel {
        let mut m = analysis_model([true, false, false, false, false, false]);
        m.add_point("J", 0.0, 0.0, 0.0);
        m.points["J"].mass[0] = 2.0;
        m.points["J"].spring[0] = 50.0;
        add_function_points(&mut m, "STEP", vec![(0.0, 1.0), (10.0, 1.0)]);
        run_modal_history(&mut m, 1, xi, 0.02, 500, vec![("STEP", "UX", 1.0)]);
        m
    }

    fn response(joint: &str, quantity: Quantity, component: usize) -> PlotFunction {
        PlotFunction {
            name: format!("{joint}-{component}"),
            case: "HIST".into(),
            source: Source::Response {
                joint: joint.into(),
                quantity,
                component,
            },
        }
    }

    #[test]
    fn absolute_acceleration_adds_the_ground_in_the_joint_axes() {
        // R = Rz(30): columns (c, s, 0), (-s, c, 0), (0, 0, 1); a_g = (1, 0.5, 0).
        // (R^T a_g)_1 = c*1 + s*0.5, (R^T a_g)_2 = -s*1 + c*0.5, (R^T a_g)_3 = 0.
        let (c, s) = (30f64.to_radians().cos(), 30f64.to_radians().sin());
        let r = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]];
        let rel = [(0.0, 2.0), (0.1, -1.0)];
        let ground = [[1.0, 0.5, 0.0], [1.0, 0.5, 0.0]];
        let a = |k| absolute_acceleration(&rel, &ground, &r, k).unwrap();
        assert!((a(0)[0].1 - (2.0 + c + 0.5 * s)).abs() < 1e-15);
        assert!((a(0)[1].1 - (-1.0 + c + 0.5 * s)).abs() < 1e-15);
        assert!((a(1)[0].1 - (2.0 - s + 0.5 * c)).abs() < 1e-15);
        assert_eq!(a(2)[0].1, 2.0);
        // Rotations equal the relative values; times are kept.
        assert_eq!(a(4), rel.to_vec());
        assert!(absolute_acceleration(&rel, &ground, &r, 6).is_err());
        assert!(absolute_acceleration(&rel, &ground[..1], &r, 0).is_err());
    }

    #[test]
    fn single_mass_absolute_acceleration_matches_the_closed_form() {
        // omega = 5, xi = 0.05, a0 = 1: u = -(a0/w^2)[1 - e^{-xi w t}(cos wd t +
        // xi/s sin wd t)], du = -(a0/w^2)(w/s) e^{-xi w t} sin wd t, s = sqrt(1-xi^2),
        // a_abs = -w^2 u - 2 xi w du. Tolerance 1e-9 of the largest |a_abs|.
        let (w, xi) = (5.0f64, 0.05f64);
        let s = (1.0 - xi * xi).sqrt();
        let m = single_mass(xi);
        let series =
            history_series(&m, "HIST", "J", 0, Quantity::AbsoluteAcceleration).unwrap();
        assert_eq!(series.len(), 501);
        let expected: Vec<f64> = series
            .iter()
            .map(|&(t, _)| {
                let e = (-xi * w * t).exp();
                let wd = w * s;
                let u = -(1.0 - e * ((wd * t).cos() + xi / s * (wd * t).sin())) / (w * w);
                let du = -(w / s) * e * (wd * t).sin() / (w * w);
                -w * w * u - 2.0 * xi * w * du
            })
            .collect();
        let peak = expected.iter().fold(0.0f64, |a, b| a.max(b.abs()));
        for (k, (got, want)) in series.iter().zip(&expected).enumerate() {
            assert!((got.1 - want).abs() <= 1e-9 * peak, "step {k}: {} {want}", got.1);
        }
        // The relative acceleration is unchanged: at t = 0, -a0 (the mass is at rest).
        let rel = history_series(&m, "HIST", "J", 0, Quantity::RelativeAcceleration).unwrap();
        assert!((rel[0].1 + 1.0).abs() < 1e-12);
        assert!(series[0].1.abs() < 1e-12);
    }

    #[test]
    fn rotated_joint_absolute_acceleration_is_the_transpose_of_the_global_one() {
        // Joint axes turned 30 deg about Z, isotropic m = 2, k = 50 (omega = 5),
        // undamped, ground (1, 0.5, 0)*STEP in global axes. Global absolute
        // acceleration (1 - cos wt)(1, 0.5, 0); local = R^T of it:
        // (c + 0.5 s, -s + 0.5 c, 0)(1 - cos wt).
        let (c, s) = (30f64.to_radians().cos(), 30f64.to_radians().sin());
        let mut m = analysis_model([true, true, true, false, false, false]);
        m.add_point("J", 0.0, 0.0, 0.0);
        m.points["J"].mass = [2.0, 2.0, 2.0, 0.0, 0.0, 0.0];
        m.points["J"].spring = [50.0, 50.0, 50.0, 0.0, 0.0, 0.0];
        m.points["J"].local_axes_deg = [30.0, 0.0, 0.0];
        add_function_points(&mut m, "STEP", vec![(0.0, 1.0), (10.0, 1.0)]);
        run_modal_history(
            &mut m,
            3,
            0.0,
            0.02,
            200,
            vec![("STEP", "UX", 1.0), ("STEP", "UY", 0.5)],
        );
        let want = [c + 0.5 * s, -s + 0.5 * c, 0.0];
        for (comp, factor) in want.iter().enumerate() {
            let series =
                history_series(&m, "HIST", "J", comp, Quantity::AbsoluteAcceleration).unwrap();
            for &(t, got) in &series {
                let expected = factor * (1.0 - (5.0 * t).cos());
                assert!((got - expected).abs() < 1e-12, "comp {comp} t {t}: {got} {expected}");
            }
        }
        // The rotation components are the relative ones.
        let r = history_series(&m, "HIST", "J", 3, Quantity::AbsoluteAcceleration).unwrap();
        assert!(r.iter().all(|p| p.1 == 0.0));
    }

    #[test]
    fn restrained_joint_has_the_ground_acceleration() {
        let mut m = analysis_model([true, false, false, false, false, false]);
        m.add_point("J", 0.0, 0.0, 0.0);
        m.points["J"].mass[0] = 2.0;
        m.points["J"].spring[0] = 50.0;
        m.add_point("B", 5.0, 0.0, 0.0);
        m.points["B"].restraint[0] = true;
        m.points["B"].mass[0] = 1.0;
        add_function_points(&mut m, "STEP", vec![(0.0, 1.0), (10.0, 1.0)]);
        run_modal_history(&mut m, 1, 0.05, 0.02, 50, vec![("STEP", "UX", 1.5)]);
        let series = history_series(&m, "HIST", "B", 0, Quantity::AbsoluteAcceleration).unwrap();
        assert!(series.iter().all(|p| (p.1 - 1.5).abs() < 1e-15));
    }

    #[test]
    fn velocity_reaction_and_direct_integration_are_refused() {
        let model = model_with_history(true);
        for q in [Quantity::Velocity, Quantity::Reaction] {
            let e = history_series(&model, "H", "J", 0, q).unwrap_err();
            assert!(e.contains("not recorded"), "{e}");
        }
        // A case without stored accelerations (direct integration).
        let model = model_with_history(false);
        let e = history_series(&model, "H", "J", 0, Quantity::AbsoluteAcceleration).unwrap_err();
        assert!(e.contains("only modal time-history cases record accelerations"), "{e}");
        // Accelerations stored but no load case to read the ground from.
        let model = model_with_history(true);
        assert!(history_series(&model, "H", "J", 0, Quantity::AbsoluteAcceleration).is_err());
    }

    #[test]
    fn load_function_trace_equals_the_solver_ground_acceleration() {
        let mut m = single_mass(0.05);
        // The trace of entry 0 (STEP, UX, 1.0) is the solver's ground acceleration.
        let f = PlotFunction {
            name: "LOAD".into(),
            case: "HIST".into(),
            source: Source::Load { entry: 0 },
        };
        let load = function_series(&m, &f).unwrap();
        let ground = sap_solver::ground_acceleration_trace(&m, "HIST").unwrap();
        assert_eq!(load.len(), ground.len());
        for (k, (l, g)) in load.iter().zip(&ground).enumerate() {
            assert!((l.1 - g[0]).abs() < 1e-15, "{k}");
            assert!((l.0 - k as f64 * 0.02).abs() < 1e-12);
        }
        // A late-starting, scaled function that is not on the output grid:
        // zero before 0.55, then 3 + 2(t - 0.55) up to 1.55 and held at 5.
        add_function_points(&mut m, "LATE", vec![(0.55, 3.0), (1.55, 5.0)]);
        m.load_cases["HIST"]
            .history_loads
            .push(("LATE".into(), "UX".into(), 2.0));
        let late = PlotFunction {
            source: Source::Load { entry: 1 },
            ..f.clone()
        };
        let series = function_series(&m, &late).unwrap();
        for &(t, v) in &series {
            let f = if t < 0.55 - 1e-12 {
                0.0
            } else if t < 1.55 {
                3.0 + 2.0 * (t - 0.55)
            } else {
                5.0
            };
            assert!((v - 2.0 * f).abs() < 1e-9, "t {t}: {v}");
        }
        // Refusals.
        let bad = PlotFunction {
            source: Source::Load { entry: 5 },
            ..f.clone()
        };
        assert!(function_series(&m, &bad).unwrap_err().contains("no load entry 6"));
        let no_case = PlotFunction {
            case: "NOPE".into(),
            ..f
        };
        assert!(function_series(&m, &no_case).is_err());
    }

    #[test]
    fn display_units_scale_and_label_each_quantity() {
        // 1 in = 0.0254 m; rotations are radians; a load entry that is not a
        // ground acceleration is a scale without unit.
        let mut m = single_mass(0.05);
        m.load_cases["HIST"]
            .history_loads
            .push(("STEP".into(), "P".into(), 4.0));
        assert_eq!(m.present_units, Units::KipInF);
        let u = |f: &PlotFunction, to| function_units(&m, f, to).unwrap();
        let disp = response("J", Quantity::Displacement, 0);
        let (k, l) = u(&disp, Units::KnMC);
        assert!((k - 0.0254).abs() < 1e-15 && l == "m", "{k} {l}");
        let (k, l) = u(&response("J", Quantity::Displacement, 3), Units::KnMC);
        assert!(k == 1.0 && l == "rad");
        let (k, l) = u(&response("J", Quantity::AbsoluteAcceleration, 1), Units::KnMC);
        assert!((k - 0.0254).abs() < 1e-15 && l == "m/s2");
        let (k, l) = u(&response("J", Quantity::RelativeAcceleration, 5), Units::KnMC);
        assert!(k == 1.0 && l == "rad/s2");
        let (k, l) = u(&disp, Units::KipInF);
        assert!(k == 1.0 && l == "in");
        let load = |entry| PlotFunction {
            name: "L".into(),
            case: "HIST".into(),
            source: Source::Load { entry },
        };
        let (k, l) = u(&load(0), Units::KnMC);
        assert!((k - 0.0254).abs() < 1e-15 && l == "m/s2");
        let (k, l) = u(&load(1), Units::KnMC);
        assert!(k == 1.0 && l.is_empty());
        assert!(function_units(&m, &response("J", Quantity::Velocity, 0), Units::KnMC).is_err());
    }

    fn setup_of(vertical: Vec<PlotFunction>) -> PlotSetup {
        PlotSetup {
            vertical,
            ..Default::default()
        }
    }

    #[test]
    fn traces_have_extremes_with_times_and_ranges_select_steps() {
        // H: U = [0, 1, -3, 2] at t = 0, 0.1, 0.2, 0.3; accelerations 10 x U.
        let model = model_with_history(true);
        let mut f = response("J", Quantity::Displacement, 0);
        f.case = "H".into();
        let curves = build_plot(&model, &setup_of(vec![f.clone()])).unwrap();
        assert_eq!(curves[0].points, vec![(0.0, 0.0), (0.1, 1.0), (0.2, -3.0), (0.3, 2.0)]);
        assert_eq!(curves[0].max, Extreme { time: 0.3, value: 2.0 });
        assert_eq!(curves[0].min, Extreme { time: 0.2, value: -3.0 });
        assert_eq!(curves[0].x_label, "Time [s]");
        // A range keeps the steps inside it: 0.1 to 0.2 -> values 1, -3.
        let mut setup = setup_of(vec![f.clone()]);
        setup.range = Some((0.1, 0.2));
        let c = &build_plot(&model, &setup).unwrap()[0];
        assert_eq!(c.points, vec![(0.1, 1.0), (0.2, -3.0)]);
        assert_eq!(c.max, Extreme { time: 0.1, value: 1.0 });
        assert_eq!(c.min, Extreme { time: 0.2, value: -3.0 });
        // Display units: 1 kip-in model -> kN-m, lengths x 0.0254.
        let mut setup = setup_of(vec![f.clone()]);
        setup.units = Some(Units::KnMC);
        let c = &build_plot(&model, &setup).unwrap()[0];
        assert!((c.max.value - 2.0 * 0.0254).abs() < 1e-15);
        assert!((c.min.value + 3.0 * 0.0254).abs() < 1e-15);
        assert_eq!(c.unit, "m");
        // Refusals: from >= to, outside the record, no step inside, no function.
        for (range, text) in [
            ((0.2, 0.2), "empty"),
            ((0.3, 0.1), "empty"),
            ((-0.1, 0.2), "outside"),
            ((0.0, 0.31), "outside"),
            ((0.11, 0.12), "no output step"),
        ] {
            let mut setup = setup_of(vec![f.clone()]);
            setup.range = Some(range);
            let e = build_plot(&model, &setup).unwrap_err();
            assert!(e.contains(text), "{range:?}: {e}");
        }
        assert!(build_plot(&model, &PlotSetup::default()).is_err());
        let mut setup = setup_of(vec![f]);
        setup.range = Some((f64::NAN, 0.2));
        assert!(build_plot(&model, &setup).is_err());
    }

    #[test]
    fn cross_plot_pairs_functions_of_one_case_at_equal_steps() {
        let mut model = model_with_history(true);
        let mut disp = response("J", Quantity::Displacement, 0);
        disp.case = "H".into();
        let mut acc = response("J", Quantity::RelativeAcceleration, 0);
        acc.name = "ACC".into();
        acc.case = "H".into();
        // x = acceleration, y = displacement: (0,0), (10,1), (-30,-3), (20,2).
        let setup = PlotSetup {
            vertical: vec![disp.clone()],
            horizontal: Some(acc.clone()),
            range: Some((0.1, 0.3)),
            units: None,
        };
        let c = &build_plot(&model, &setup).unwrap()[0];
        assert_eq!(c.points, vec![(10.0, 1.0), (-30.0, -3.0), (20.0, 2.0)]);
        assert_eq!(c.max.time, 0.3);
        assert_eq!(c.min.time, 0.2);
        assert!(c.x_label.starts_with("ACC [") && c.x_label.ends_with("in/s2]"));
        // Another case is refused.
        let other = model.case_results["H"].clone();
        model.case_results.insert("H2".into(), other);
        let mut other_case = disp.clone();
        other_case.case = "H2".into();
        let setup = PlotSetup {
            vertical: vec![other_case],
            horizontal: Some(acc),
            ..Default::default()
        };
        let e = build_plot(&model, &setup).unwrap_err();
        assert!(e.contains("same case"), "{e}");
    }

    #[test]
    fn several_traces_legend_and_bounds() {
        let model = model_with_history(true);
        let mut a = response("J", Quantity::Displacement, 0);
        a.name = "A".into();
        a.case = "H".into();
        let mut b = response("J", Quantity::RelativeAcceleration, 0);
        b.name = "B".into();
        b.case = "H".into();
        let curves = build_plot(&model, &setup_of(vec![a, b])).unwrap();
        assert_eq!(curves.len(), 2);
        assert_eq!(
            legend_text(&curves[0]),
            "A: max 2.00000e0 at t = 3.00000e-1 s, min -3.00000e0 at t = 2.00000e-1 s in"
        );
        assert!(legend_text(&curves[1]).starts_with("B: max 2.00000e1 at t = 3.00000e-1 s"));
        // x in [0, 0.3], y in [-30, 20] over both traces.
        let bounds = plot_bounds(&curves).unwrap();
        assert_eq!(bounds, [0.0, 0.3, -30.0, 20.0]);
        let (u, v) = to_unit_square(bounds, (0.15, -5.0));
        assert!((u - 0.5).abs() < 1e-15 && (v - 0.5).abs() < 1e-15);
        // A flat record is widened to a unit span; no points, no bounds.
        let flat = Curve {
            name: "F".into(),
            x_label: String::new(),
            y_label: String::new(),
            unit: String::new(),
            points: vec![(1.0, 4.0), (1.0, 4.0)],
            max: Extreme { time: 0.0, value: 4.0 },
            min: Extreme { time: 0.0, value: 4.0 },
        };
        assert_eq!(plot_bounds(&[flat]).unwrap(), [0.5, 1.5, 3.5, 4.5]);
        assert_eq!(plot_bounds(&[]), None);
        assert_eq!(format_value(0.0), "0");
    }

    #[test]
    fn named_functions_and_sets_refuse_bad_names_and_stay_unchanged() {
        let mut list = Vec::new();
        let f = response("J", Quantity::Displacement, 0);
        let named = |n: &str| PlotFunction {
            name: n.into(),
            ..f.clone()
        };
        assert!(add_plot_function(&mut list, named("  ")).is_err());
        assert!(add_plot_function(&mut list, named("D")).is_ok());
        assert!(add_plot_function(&mut list, named("d")).is_err());
        let bad_component = response("J", Quantity::Displacement, 6);
        assert!(add_plot_function(&mut list, bad_component).is_err());
        assert_eq!(list.len(), 1);

        let mut sets = Vec::new();
        let setup = PlotSetup {
            vertical: list.clone(),
            range: Some((0.1, 0.2)),
            units: Some(Units::KnMC),
            ..Default::default()
        };
        assert!(save_set(&mut sets, "", setup.clone()).is_err());
        assert!(save_set(&mut sets, "S1", setup.clone()).is_ok());
        assert!(save_set(&mut sets, "s1", PlotSetup::default()).is_err());
        assert_eq!(sets.len(), 1);
        assert_eq!(sets[0].setup, setup);

        // Recalling puts the functions back; a clashing definition refuses.
        let mut empty = Vec::new();
        let back = recall_set(&mut empty, &sets, "S1").unwrap();
        assert_eq!(back, setup);
        assert_eq!(empty, list);
        let mut clash = vec![PlotFunction {
            case: "OTHER".into(),
            ..list[0].clone()
        }];
        let before = clash.clone();
        assert!(recall_set(&mut clash, &sets, "S1").is_err());
        assert_eq!(clash, before);
        assert!(recall_set(&mut empty, &sets, "NOPE").is_err());
    }

    #[test]
    fn sets_and_functions_are_reset_with_the_window_state() {
        // gui.rs replaces the whole State when another model is loaded
        // (`self.panel_time_history = time_history::State::default()`).
        let mut state = State::default();
        state.plots.functions.push(response("J", Quantity::Displacement, 0));
        state.plots.sets.push(NamedSet {
            name: "S".into(),
            setup: PlotSetup::default(),
        });
        state = State::default();
        assert!(state.plots.functions.is_empty() && state.plots.sets.is_empty());
    }

    #[test]
    fn window_shows_the_legend_and_a_click_saves_a_set_once() {
        let ctx = egui::Context::default();
        let mut model = model_with_history(true);
        let mut state = State::default();
        state.open = true;
        state.tab = 1;
        let mut f = response("J", Quantity::Displacement, 0);
        f.name = "UJ".into();
        f.case = "H".into();
        state.plots.functions.push(f);
        state.plots.vertical.push("UJ".into());
        state.plots.set_name = "SET1".into();
        let expected = legend_text(&build_plot(&model, &current_setup(&state.plots).unwrap()).unwrap()[0]);
        let mut time = 1.0;
        let mut out = None;
        for _ in 0..10 {
            time += 1.0;
            out = Some(frame(&ctx, time, vec![], &mut state, &mut model).0);
        }
        let out = out.unwrap();
        assert!(texts(&out).contains(&expected), "{expected}");
        let pos = center_of(&out, "Save set").expect("button is drawn");
        click(&ctx, time + 1.0, pos, &mut state, &mut model);
        assert_eq!(state.plots.sets.len(), 1);
        assert_eq!(state.plots.sets[0].name, "SET1");
        // The same name again is refused and the sets do not change.
        let mut out = None;
        for _ in 0..10 {
            time += 2.0;
            out = Some(frame(&ctx, time + 5.0, vec![], &mut state, &mut model).0);
        }
        let pos = center_of(&out.unwrap(), "Save set").expect("button is drawn");
        click(&ctx, time + 10.0, pos, &mut state, &mut model);
        assert_eq!(state.plots.sets.len(), 1);
        assert!(matches!(&state.plots.message, Some(Err(e)) if e.contains("already exists")));
    }
}
