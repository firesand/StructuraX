//! Part 3 of the video: a ground-acceleration record read from a text file
//! through the "Functions from File and Plot Functions" window, a modal
//! time-history case solved by the application's `execute_analysis`, and the
//! response read through the plot-function tab.
//!
//! The window is driven only by synthetic pointer and key input sent through
//! `render_ui`; the numbers checked are the texts it paints ("Maximum absolute
//! value ... at t = ... s.", the legend lines, the file summary) and the function
//! stored in the model by its "Add as function" button.
//!
//! Single-storey closed form (this file, `single_storey`): four columns of
//! 0.3 m x 0.3 m (EI = 2e8 x 6.75e-4 = 135000 kN-m2, no shear deformation),
//! h = 4 m, fixed at the base, the roof joints tied by a rigid diaphragm and
//! held against UZ, RX and RY (so each column is guided-fixed and has the sway
//! stiffness 12 EI / h^3; with free roof rotations the columns would be
//! cantilevers with 3 EI / h^3), roof mass M = 4 x 625 = 2500 t in X and Y:
//!   k = 4 x 12 EI / h^3 = 101250 kN/m, omega^2 = k / M = 40.5, omega = 6.3640 rad/s,
//!   T = 0.98730 s. Record: the constant 1.0 read from the file, scale factor
//!   0.01 (a0 = 0.01 m/s2), damping xi = 0.05:
//!   u(t)  = -(a0/w^2) [1 - e^{-xi w t} (cos wd t + xi/s sin wd t)], s = sqrt(1 - xi^2),
//!   du(t) = -(a0/w^2) (w/s) e^{-xi w t} sin wd t,
//!   absolute acceleration = -w^2 u - 2 xi w du (the damped single mass).

use super::harness::{close, Run};
use sap_model::{
    DynamicFunction, FramePropType, FrameSection, LoadCase, LoadCaseType, LoadPattern,
    LoadPatternType, Material, SapModel,
};
use sap_units::Units;
use std::path::PathBuf;

pub const DT: f64 = 0.02;

/// A function file in the format of the video: two header lines, 8 values per line.
pub fn write_record(tag: &str, values: &[f64]) -> PathBuf {
    // Unique per call: the tests of one process run in parallel threads.
    static NEXT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
    let n = NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let path =
        std::env::temp_dir().join(format!("structurax_p6_{}_{n}_{tag}.txt", std::process::id()));
    let mut text = String::from("Synthetic ground acceleration record\nunits g, 0.02 s step, 8 values per line\n");
    for line in values.chunks(8) {
        let items: Vec<String> = line.iter().map(|v| format!("{v:.6}")).collect();
        text.push_str(&items.join(" "));
        text.push('\n');
    }
    std::fs::write(&path, text).unwrap();
    path
}

/// The values as the file holds them (six decimals).
pub fn round6(v: f64) -> f64 {
    format!("{v:.6}").parse().unwrap()
}

/// Opens the window and reads `path` into the function `name`, the way the
/// form is used: path, header lines, values per line, equal intervals of 0.02 s,
/// name, "Read file", "Add as function". Returns the summary line the window
/// painted after "Read file".
pub fn add_function_from_file(run: &mut Run, path: &std::path::Path, name: &str) -> String {
    run.app.panel_time_history.open = true;
    run.settle();
    run.type_into("File path:", &path.display().to_string());
    run.set_number("Header lines to skip:", "2");
    run.set_number("Number of points per line:", "8");
    run.click_text("Values at equal intervals of");
    run.type_into("Function name:", name);
    run.click_text("Read file");
    let summary = run
        .text_starting("3000 points")
        .unwrap_or_else(|| panic!("no summary in {:?}", run.shown()));
    run.click_text("Add as function");
    summary
}

/// Like `write_record`, in a directory of its own (so that the file chooser lists
/// only this file).
pub fn write_record_in_own_dir(tag: &str, values: &[f64]) -> PathBuf {
    let file = write_record(tag, values);
    let dir = file.with_extension("dir");
    std::fs::create_dir_all(&dir).unwrap();
    let moved = dir.join(format!("{tag}.txt"));
    std::fs::rename(&file, &moved).unwrap();
    moved
}

/// Like `add_function_from_file`, but the file is chosen with the file
/// chooser: the directory is typed in "File path:", "Browse..." opens the chooser,
/// a click on the file name and "Choose" take the file.
pub fn add_function_via_browse(run: &mut Run, path: &std::path::Path, name: &str) -> String {
    run.app.panel_time_history.open = true;
    run.settle();
    run.type_into("File path:", &path.parent().unwrap().display().to_string());
    run.click_text("Browse...");
    run.click_text(&path.file_name().unwrap().to_string_lossy());
    run.click_text("Choose");
    run.settle();
    run.set_number("Header lines to skip:", "2");
    run.set_number("Number of points per line:", "8");
    run.click_text("Values at equal intervals of");
    run.type_into("Function name:", name);
    run.click_text("Read file");
    let summary = run
        .text_starting("3000 points")
        .unwrap_or_else(|| panic!("no summary in {:?}", run.shown()));
    run.click_text("Add as function");
    summary
}

/// Ticks "Time range:" if it is not on, and sets the range `from` .. `to` (seconds).
pub fn set_time_range(run: &mut Run, from: f64, to: f64) {
    if !run.has_text("to") {
        run.click_text("Time range:");
    }
    run.set_number("Time range:", &from.to_string());
    run.set_number("to", &to.to_string());
}

/// The legend line "name: max ... min ..." painted for the plot function `name`:
/// (max, time of max, min, time of min).
pub fn legend_of(run: &Run, name: &str) -> (f64, f64, f64, f64) {
    let text = run
        .text_starting(&format!("{name}: max "))
        .unwrap_or_else(|| panic!("no legend of {name} in {:?}", run.shown()));
    let rest = text.strip_prefix(&format!("{name}: max ")).unwrap();
    let (max, rest) = rest.split_once(" at t = ").unwrap();
    let (tmax, rest) = rest.split_once(" s, min ").unwrap();
    let (min, rest) = rest.split_once(" at t = ").unwrap();
    let (tmin, _) = rest.split_once(" s").unwrap();
    (max.parse().unwrap(), tmax.parse().unwrap(), min.parse().unwrap(), tmin.parse().unwrap())
}

/// The model of the single storey, without functions and cases.
pub fn single_storey() -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    m.load_cases.clear();
    m.load_patterns.clear();
    m.frame_sections.clear();
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.e = 2.0e8;
    mat.u = 0.3;
    mat.g = 2.0e8 / 2.6;
    mat.unit_weight = 0.0;
    mat.unit_mass = 0.0;
    m.materials.insert("MAT".into(), mat);
    let side: f64 = 0.3;
    m.frame_sections.insert(
        "COL".into(),
        FrameSection {
            name: "COL".into(),
            material: "MAT".into(),
            prop_type: FramePropType::General,
            area: side * side,
            i33: side.powi(4) / 12.0,
            i22: side.powi(4) / 12.0,
            torsional_j: 0.141 * side.powi(4),
            as2: 0.0,
            as3: 0.0,
            modifiers: [1.0; 8],
            ..FrameSection::default()
        },
    );
    let mut k = 0;
    for (x, y) in [(-3.0, -3.0), (3.0, -3.0), (3.0, 3.0), (-3.0, 3.0)] {
        k += 1;
        let top = m.add_point(format!("T{k}"), x, y, 4.0);
        let bottom = m.add_point(format!("B{k}"), x, y, 0.0);
        m.points[&bottom].restraint = [true; 6];
        // Held against UZ, RX and RY: the sway of every column is guided-fixed.
        m.points[&top].restraint = [false, false, true, true, true, false];
        m.points[&top].mass = [625.0, 625.0, 0.0, 0.0, 0.0, 0.0];
        m.points[&top].selected = true;
        m.add_frame(format!("C{k}"), bottom, top, "COL");
    }
    m.assign_diaphragm("DIAPH", true).unwrap();
    m.load_patterns.insert(
        "P".into(),
        LoadPattern { name: "P".into(), pat_type: LoadPatternType::Other, self_wt_multiplier: 0.0 },
    );
    m
}

/// The modal case MODAL and the modal history HIST with the entries `loads`
/// (function, direction, scale factor).
pub fn add_history(m: &mut SapModel, modes: usize, xi: f64, steps: usize, loads: &[(&str, &str, f64)]) {
    m.load_cases.insert("MODAL".into(), LoadCase::modal_eigen("MODAL", modes));
    let mut case = LoadCase::linear_static("HIST", "P", 1.0);
    case.case_type = LoadCaseType::LinearHistory;
    case.loads.clear();
    case.modal_case = Some("MODAL".into());
    case.modal_damping = xi;
    case.time_step_dt = DT;
    case.num_time_steps = steps;
    case.history_loads =
        loads.iter().map(|(f, d, s)| (f.to_string(), d.to_string(), *s)).collect();
    m.load_cases.insert("HIST".into(), case);
}

/// The closed form of the damped single mass under a ground acceleration step `a0`:
/// relative displacement and absolute acceleration at time `t`.
fn step_response(w: f64, xi: f64, a0: f64, t: f64) -> (f64, f64) {
    let s = (1.0 - xi * xi).sqrt();
    let wd = w * s;
    let e = (-xi * w * t).exp();
    let u = -(a0 / (w * w)) * (1.0 - e * ((wd * t).cos() + xi / s * (wd * t).sin()));
    let du = -(a0 / (w * w)) * (w / s) * e * (wd * t).sin();
    (u, -w * w * u - 2.0 * xi * w * du)
}

/// The value after "Maximum absolute value " and the time after "at t = ".
pub fn parse_peak(text: &str) -> (f64, f64) {
    let rest = text.strip_prefix("Maximum absolute value ").unwrap();
    let (value, rest) = rest.split_once(" at t = ").unwrap();
    let time = rest.strip_suffix(" s.").unwrap();
    (value.parse().unwrap(), time.parse().unwrap())
}

#[test]
fn a_record_read_through_the_window_is_stored_as_the_function_of_the_file() {
    // Independent side: the values written to the file by this test.
    let values: Vec<f64> =
        (0..3000).map(|k| round6((0.3 * k as f64 * DT).sin() + 0.001 * (k % 7) as f64)).collect();
    let path = write_record("function", &values);
    let mut run = Run::new(single_storey());
    let summary = add_function_from_file(&mut run, &path, "GM1");
    let _ = std::fs::remove_file(&path);
    // The summary the window painted: 3000 points over 2999 x 0.02 s, the largest |value|.
    let largest = values.iter().fold(0.0f64, |a, v| a.max(v.abs()));
    assert!(summary.starts_with("3000 points, duration 59.98"), "{summary}");
    assert!(summary.ends_with(&format!("largest |value| {largest}")), "{summary} against {largest}");
    // The function "Add as function" stored: pairs (k dt, value), the first at t = 0.
    let f: &DynamicFunction = &run.app.model.functions["GM1"];
    assert!(f.is_time_history);
    assert_eq!(f.points.len(), 3000);
    for (k, (t, v)) in f.points.iter().enumerate() {
        assert!((t - k as f64 * DT).abs() <= 1e-12, "time {k}: {t}");
        assert_eq!(*v, values[k], "value {k}");
    }
    // A name that is used is refused with a message and the model is unchanged.
    let before = run.app.model.functions.len();
    let path = write_record("function2", &values);
    let _ = add_function_from_file_again(&mut run, &path, "GM1");
    let _ = std::fs::remove_file(&path);
    assert_eq!(run.app.model.functions.len(), before);
}

/// The window is open and has read a file already: read another with the same name.
fn add_function_from_file_again(run: &mut Run, path: &std::path::Path, name: &str) -> String {
    run.type_into("File path:", &path.display().to_string());
    run.type_into("Function name:", name);
    run.click_text("Read file");
    run.click_text("Add as function");
    run.settle();
    run.text_starting("3000 points").unwrap_or_default()
}

#[test]
fn the_single_storey_displacement_and_absolute_acceleration_match_the_closed_form() {
    let record = vec![1.0; 3000];
    let path = write_record_in_own_dir("step", &record);
    let mut run = Run::new(single_storey());
    // The record is chosen with the file chooser (Browse...).
    add_function_via_browse(&mut run, &path, "STEP");
    let _ = std::fs::remove_dir_all(path.parent().unwrap());
    assert_eq!(run.app.model.functions["STEP"].points.len(), 3000);
    let steps = 500;
    add_history(&mut run.app.model, 3, 0.05, steps, &[("STEP", "UX", 0.01)]);
    run.app.active_case = "HIST".into();
    assert!(run.app.execute_analysis(), "{}", run.app.status_msg);
    run.settle();
    let m = &run.app.model;
    // The modal frequency, from the stiffness and the mass.
    let (ei, h, mass): (f64, f64, f64) = (2.0e8 * 0.3_f64.powi(4) / 12.0, 4.0, 2500.0);
    let k = 4.0 * 12.0 * ei / h.powi(3);
    let w = (k / mass).sqrt();
    close("omega^2", w * w, 40.5, 1e-12);
    let modes = &m.case_results["MODAL"].modes;
    close("period X", modes[0].period_sec, 2.0 * std::f64::consts::PI / w, 1e-5);
    close("period Y", modes[1].period_sec, 2.0 * std::f64::consts::PI / w, 1e-5);
    // The trace of the solver at all steps against the closed form, in the roof corner.
    let (xi, a0) = (0.05, 0.01);
    let cr = &m.case_results["HIST"];
    let trace = &cr.time_history_disp["T1"];
    assert_eq!(trace.len(), steps + 1);
    let peak_u = (0..=steps).map(|k| step_response(w, xi, a0, k as f64 * DT).0.abs()).fold(0.0, f64::max);
    let peak_a = (0..=steps).map(|k| step_response(w, xi, a0, k as f64 * DT).1.abs()).fold(0.0, f64::max);
    let mut worst = 0.0f64;
    for (k, u) in trace.iter().enumerate() {
        let (want, _) = step_response(w, xi, a0, k as f64 * DT);
        worst = worst.max((u[0] - want).abs() / peak_u);
    }
    eprintln!("single storey: worst |u - closed form| / peak |u| = {worst:e}");
    assert!(worst <= 1e-6, "worst relative deviation {worst:e}");
    // The absolute acceleration at all steps: the stored relative acceleration plus the
    // ground acceleration 0.01 m/s2 of the record, against -w^2 u - 2 xi w du.
    let mut worst_a = 0.0f64;
    for (k, a) in cr.time_history_accel["T1"].iter().enumerate() {
        let (_, want) = step_response(w, xi, a0, k as f64 * DT);
        worst_a = worst_a.max((a[0] + a0 - want).abs() / peak_a);
    }
    eprintln!("single storey: worst |a_abs - closed form| / peak |a_abs| = {worst_a:e}");
    assert!(worst_a <= 1e-6, "worst relative deviation {worst_a:e}");
    // The plot-function window: Plot functions tab, Displacement, U1 of the first
    // joint of the result, then Absolute acceleration.
    let first = cr.time_history_disp.keys().next().unwrap().clone();
    assert_eq!(first, "T1");
    run.click_text("Plot functions");
    run.click_text("Displacement");
    let (value, time) = parse_peak(&run.text_starting("Maximum absolute value ").expect("peak line"));
    let (want_k, want) = (0..=steps)
        .map(|k| (k, step_response(w, xi, a0, k as f64 * DT).0.abs()))
        .fold((0, 0.0), |a, b| if b.1 > a.1 { b } else { a });
    close("painted peak displacement", value, want, 1e-5);
    assert!((time - want_k as f64 * DT).abs() <= 1e-9, "time {time} against {}", want_k as f64 * DT);
    run.click_text("Absolute acceleration");
    let (value, time) = parse_peak(&run.text_starting("Maximum absolute value ").expect("peak line"));
    let (want_k, want) = (0..=steps)
        .map(|k| (k, step_response(w, xi, a0, k as f64 * DT).1.abs()))
        .fold((0, 0.0), |a, b| if b.1 > a.1 { b } else { a });
    assert!(want == peak_a);
    close("painted peak absolute acceleration", value, want, 1e-5);
    assert!((time - want_k as f64 * DT).abs() <= 1e-9, "time {time} against {}", want_k as f64 * DT);
    // Velocity and reaction are refused with a message, not drawn.
    run.click_text("Velocity");
    assert!(run.shown().iter().any(|t| t.contains("not recorded")), "{:?}", run.shown());
    // The legend of two named functions, read at every 25th step through a time range that
    // holds that step alone: the values the window shows at 21 of the 501 steps.
    run.click_text("Displacement");
    run.type_into("Name:", "DISP");
    run.click_text("Add response function");
    run.click_text("Absolute acceleration");
    run.type_into("Name:", "ACC");
    run.click_text("Add response function");
    let verticals = |run: &Run| {
        let mut v: Vec<_> = super::harness::texts(&run.last)
            .into_iter()
            .filter(|(t, _)| t == "vertical")
            .map(|(_, r)| r)
            .collect();
        v.sort_by(|a, b| a.center().y.total_cmp(&b.center().y));
        v
    };
    for n in 0..2 {
        let r = verticals(&run)[n];
        run.click(r.center());
    }
    for k in (0..=steps).step_by(25) {
        let t = k as f64 * DT;
        let (from, to) = if k == steps { (t - 0.001, t) } else { (t, t + 0.001) };
        set_time_range(&mut run, from, to);
        let (u, a) = {
            let (u, a) = step_response(w, xi, a0, t);
            (u, a)
        };
        let (dmax, _, dmin, _) = legend_of(&run, "DISP");
        assert!((dmax - u).abs() <= 1e-5 * peak_u && (dmin - u).abs() <= 1e-5 * peak_u, "step {k}: {dmax} {dmin} against {u}");
        let (amax, _, amin, _) = legend_of(&run, "ACC");
        assert!((amax - a).abs() <= 1e-5 * peak_a && (amin - a).abs() <= 1e-5 * peak_a, "step {k}: {amax} {amin} against {a}");
    }
}
