//! Functions from file and time-history plot functions.
//!
//! "Function from file" reads a time-history function from a text file the way
//! the SAP2000 form of that name does (see `sap_format::function_file`) and
//! stores it as a user-defined function. "Plot functions" plots the response
//! of a time-history case at a joint and prints its maximum absolute value.

use eframe::egui;
use sap_format::function_file::{read_function_file, FunctionFileOptions, ItemFormat, ValueLayout};
use sap_model::{CaseResults, DynamicFunction, SapModel};
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
        }
    }
}

/// Response quantity of a time-history case.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Quantity {
    Displacement,
    RelativeAcceleration,
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
        Quantity::RelativeAcceleration => &results.time_history_accel,
    };
    if traces.is_empty() {
        return Err(format!(
            "Case '{case}' has no relative accelerations: only modal (mode-superposition) time-history cases record them."
        ));
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
    Ok(results
        .time_steps
        .iter()
        .zip(trace)
        .map(|(t, u)| (*t, u[component]))
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
) -> Option<Result<String, String>> {
    let mut outcome = None;
    ui.label(
        "Reads time-history values from a text file: header lines are skipped, lines starting with $ are skipped, and reading stops at a blank line or the end of the file.",
    );
    ui.horizontal(|ui| {
        ui.label("File path:");
        ui.text_edit_singleline(&mut state.path);
    });
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
        ui.selectable_value(&mut state.quantity, Quantity::Displacement, "Displacement");
        ui.selectable_value(
            &mut state.quantity,
            Quantity::RelativeAcceleration,
            "Relative acceleration",
        );
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
}

/// Draws the window when it is open. Returns the outcome of a command the
/// user ran in this frame: a status message, or why it was refused.
pub(super) fn render(
    ctx: &egui::Context,
    state: &mut State,
    model: &mut SapModel,
) -> Option<Result<String, String>> {
    if !state.open {
        return None;
    }
    let mut open = state.open;
    let mut outcome = None;
    egui::Window::new("Functions from File and Plot Functions")
        .open(&mut open)
        .default_width(620.0)
        .vscroll(true)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.selectable_value(&mut state.tab, 0, "Function from file");
                ui.selectable_value(&mut state.tab, 1, "Plot functions");
            });
            ui.separator();
            if state.tab == 0 {
                outcome = function_tab(ui, state, model);
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
                if let Some(result) = render(ctx, state, model) {
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
}
