//! Part 3 of the video, the ten-storey frame: two records read from files
//! through the window, a modal history with 20 eigenvectors from subspace
//! iteration, and the plot-function window on the base and the roof joints
//! (absolute acceleration, maximum and minimum with times, a time range, a named
//! set) and the animation through the steps.
//!
//! Independent sides: the records written to the files by `records()` (the
//! base joint's absolute acceleration is the ground acceleration 0.01 f(t), a
//! kinematic identity), and the stored trace of the roof joint (the displayed
//! maximum equals the trace maximum).

use super::frame::{
    add_function_from_file, add_history, legend_of, parse_peak, round6, set_time_range, write_record, DT,
};
use super::harness::{close, flat, Run};
use crate::gui::DisplayMode;
use eframe::egui::{epaint::Shape, Color32, Pos2};
use sap_model::{
    FramePropType, FrameSection, LoadPattern, LoadPatternType, Material, SapModel,
};
use sap_units::Units;

/// Ten storeys of 3 m, five 6 m bays in X and Y, columns 0.6 m x 0.6 m, beams
/// 0.3 m x 0.6 m, concrete mass density 2.5 t/m3, fixed bases.
pub fn ten_storey() -> SapModel {
    use sap_model::generate::GridRegion;
    let mut m = SapModel::new(Units::KnMC);
    m.load_cases.clear();
    m.load_patterns.clear();
    m.frame_sections.clear();
    let mut mat = Material::default_steel_kip_in();
    mat.name = "CONC".into();
    mat.e = 3.0e7;
    mat.u = 0.2;
    mat.g = 3.0e7 / 2.4;
    mat.unit_weight = 0.0;
    mat.unit_mass = 2.5;
    m.materials.insert("CONC".into(), mat);
    let section = |name: &str, b: f64, d: f64| FrameSection {
        name: name.into(),
        material: "CONC".into(),
        prop_type: FramePropType::General,
        area: b * d,
        i33: b * d.powi(3) / 12.0,
        i22: d * b.powi(3) / 12.0,
        torsional_j: 0.141 * b.min(d).powi(3) * b.max(d),
        as2: 0.0,
        as3: 0.0,
        modifiers: [1.0; 8],
        ..FrameSection::default()
    };
    m.frame_sections.insert("COL".into(), section("COL", 0.6, 0.6));
    m.frame_sections.insert("BEAM".into(), section("BEAM", 0.3, 0.6));
    let (n, nz) = (6usize, 11usize);
    m.quick_grid([n, n, nz], [6.0, 6.0, 3.0], [0.0; 3]).unwrap();
    let all = GridRegion { x: (0, n - 1), y: (0, n - 1), z: (0, nz - 1) };
    m.quick_draw_columns(all, "COL").unwrap();
    m.quick_draw_beams(GridRegion { z: (1, nz - 1), ..all }, true, true, "BEAM").unwrap();
    for p in m.points.values_mut().filter(|p| p.z.abs() < 1e-9) {
        p.restraint = [true; 6];
    }
    m.load_patterns.insert(
        "P".into(),
        LoadPattern { name: "P".into(), pat_type: LoadPatternType::Other, self_wt_multiplier: 0.0 },
    );
    m
}

/// Record 1 (X) and record 2 (Y), 3000 values at 0.02 s, as the files hold them.
pub fn records() -> (Vec<f64>, Vec<f64>) {
    let t = |k: usize| k as f64 * DT;
    let pi2 = 2.0 * std::f64::consts::PI;
    let f1 = (0..3000)
        .map(|k| {
            round6((pi2 * 0.7 * t(k)).sin() * (-t(k) / 20.0).exp() + 0.3 * (pi2 * 2.3 * t(k)).sin())
        })
        .collect();
    let f2 = (0..3000).map(|k| round6((pi2 * 0.5 * t(k)).cos() * (-t(k) / 30.0).exp())).collect();
    (f1, f2)
}

fn peak_abs(values: &[f64]) -> (usize, f64) {
    values
        .iter()
        .enumerate()
        .fold((0, 0.0), |a, (k, v)| if v.abs() > a.1 { (k, v.abs()) } else { a })
}

/// Parses `name: max X at t = T s, min Y at t = U s unit` into (X, T, Y, U).
fn parse_legend(text: &str, name: &str) -> (f64, f64, f64, f64) {
    let rest = text.strip_prefix(&format!("{name}: max ")).unwrap_or_else(|| panic!("{text}"));
    let (max, rest) = rest.split_once(" at t = ").unwrap();
    let (tmax, rest) = rest.split_once(" s, min ").unwrap();
    let (min, rest) = rest.split_once(" at t = ").unwrap();
    let (tmin, _) = rest.split_once(" s").unwrap();
    (max.parse().unwrap(), tmax.parse().unwrap(), min.parse().unwrap(), tmin.parse().unwrap())
}

/// A copy of `model` whose first listed joint of the history traces is `joint`
/// (the plot window's joint selection starts at the first listed joint; the
/// values are untouched).
fn with_joint_first(model: &SapModel, joint: &str) -> SapModel {
    let mut m = model.clone();
    let cr = m.case_results.get_mut("HIST").unwrap();
    let mut entries: Vec<_> = cr.time_history_disp.drain(..).collect();
    let at = entries.iter().position(|(k, _)| k == joint).unwrap();
    let chosen = entries.remove(at);
    entries.insert(0, chosen);
    cr.time_history_disp.extend(entries);
    m
}

fn max_of(v: &[f64]) -> f64 {
    v.iter().cloned().fold(f64::MIN, f64::max)
}

fn min_of(v: &[f64]) -> f64 {
    v.iter().cloned().fold(f64::MAX, f64::min)
}

/// The analysed ten-storey frame with the two records loaded through the window.
fn analysed() -> (Run, Vec<f64>, Vec<f64>) {
    let (f1, f2) = records();
    let (p1, p2) = (write_record("rec1", &f1), write_record("rec2", &f2));
    let mut run = Run::new(ten_storey());
    add_function_from_file(&mut run, &p1, "REC1");
    add_function_from_file(&mut run, &p2, "REC2");
    let _ = (std::fs::remove_file(&p1), std::fs::remove_file(&p2));
    assert_eq!(run.app.model.functions["REC1"].points.len(), 3000);
    add_history(&mut run.app.model, 20, 0.05, 2999, &[("REC1", "U1", 0.01), ("REC2", "U2", 0.01)]);
    let started = std::time::Instant::now();
    assert!(run.app.execute_analysis(), "{}", run.app.status_msg);
    eprintln!("ten-storey analysis (20 modes, 2999 steps): {:?}", started.elapsed());
    run.settle();
    (run, f1, f2)
}

#[test]
fn ten_storey_frame_the_base_acceleration_is_the_record_and_the_plot_window_shows_the_traces() {
    let (mut run, f1, f2) = analysed();
    let steps = 2999;
    let m = &run.app.model;
    assert_eq!(m.case_results["MODAL"].modes.len(), 20);
    let cr = &m.case_results["HIST"];
    assert_eq!(cr.time_steps.len(), steps + 1);
    // The solver's ground acceleration trace is 0.01 f(t) in X and Y at every step.
    let ground = sap_solver::ground_acceleration_trace(m, "HIST").unwrap();
    assert_eq!(ground.len(), steps + 1);
    for k in 0..=steps {
        assert!((ground[k][0] - 0.01 * f1[k]).abs() <= 1e-15, "step {k}");
        assert!((ground[k][1] - 0.01 * f2[k]).abs() <= 1e-15, "step {k}");
        assert_eq!(ground[k][2], 0.0);
    }
    // A base joint does not move relative to the ground, so its absolute
    // acceleration is the ground acceleration.
    let base = cr.time_history_disp.keys().next().unwrap().clone();
    assert!(m.points[&base].z.abs() < 1e-9 && m.points[&base].restraint == [true; 6]);
    for a in &cr.time_history_accel[&base] {
        assert!(a.iter().all(|v| v.abs() <= 1e-12), "{a:?}");
    }
    // Through the window: the base joint, Absolute acceleration, U1 then U2.
    run.click_text("Plot functions");
    run.click_text("Absolute acceleration");
    let (value, time) = parse_peak(&run.text_starting("Maximum absolute value ").expect("peak line"));
    let (k1, peak1) = peak_abs(&f1);
    close("painted peak of the base U1", value, 0.01 * peak1, 1e-12);
    assert!((time - k1 as f64 * DT).abs() <= 1e-9, "{time} against {}", k1 as f64 * DT);
    run.choose("Component:", "U2");
    let (value, time) = parse_peak(&run.text_starting("Maximum absolute value ").expect("peak line"));
    let (k2, peak2) = peak_abs(&f2);
    close("painted peak of the base U2", value, 0.01 * peak2, 1e-12);
    assert!((time - k2 as f64 * DT).abs() <= 1e-9, "{time} against {}", k2 as f64 * DT);
    // Named functions BY (U2, current) and BX (U1) of the base joint, read at every
    // 600th step and the last through a time range holding that step alone: the
    // window shows 0.01 f(t) in both directions at 6 of the 3000 steps.
    run.type_into("Name:", "BY");
    run.click_text("Add response function");
    run.choose("Component:", "U1");
    run.type_into("Name:", "BX");
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
    let mut steps_read: Vec<usize> = (0..=steps).step_by(600).collect();
    steps_read.push(steps);
    for k in steps_read {
        let t = k as f64 * DT;
        let (from, to) = if k == steps { (t - 0.001, t) } else { (t, t + 0.001) };
        set_time_range(&mut run, from, to);
        for (name, want) in [("BX", 0.01 * f1[k]), ("BY", 0.01 * f2[k])] {
            let (max, _, min, _) = legend_of(&run, name);
            let tol = 1e-5 * 0.01 * peak1.max(peak2);
            assert!((max - want).abs() <= tol && (min - want).abs() <= tol, "{name} step {k}: {max} {min} against {want}");
        }
    }
}

#[test]
fn ten_storey_frame_roof_traces_range_named_set_and_animation() {
    let (run, _, _) = analysed();
    let m = &run.app.model;
    let roof = m
        .points
        .values()
        .find(|p| p.z > 29.9 && p.x.abs() < 1e-9 && p.y.abs() < 1e-9)
        .unwrap()
        .name
        .clone();
    let trace: Vec<f64> = m.case_results["HIST"].time_history_disp[&roof].iter().map(|u| u[0]).collect();
    // The plot window on a copy whose first listed joint is the roof corner.
    let mut run2 = Run::new(with_joint_first(m, &roof));
    run2.app.panel_time_history.open = true;
    run2.settle();
    run2.click_text("Plot functions");
    // Displayed maximum = trace maximum (value and time).
    let (value, time) = parse_peak(&run2.text_starting("Maximum absolute value ").expect("peak line"));
    let (kp, peak) = peak_abs(&trace);
    close("displayed maximum of the roof U1", value, peak, 1e-12);
    assert!((time - kp as f64 * DT).abs() <= 1e-9);
    // A named function, vertical: its legend line shows the maximum and the minimum.
    run2.type_into("Name:", "ROOFX");
    run2.click_text("Add response function");
    run2.click_text("vertical");
    let legend = run2.text_starting("ROOFX: max").unwrap_or_else(|| panic!("{:?}", run2.shown()));
    let (mx, tmx, mn, tmn) = parse_legend(&legend, "ROOFX");
    let (want_max, want_min) = (max_of(&trace), min_of(&trace));
    close("legend max", mx, want_max, 1e-5);
    close("legend min", mn, want_min, 1e-5);
    let at = |v: f64| trace.iter().position(|x| *x == v).unwrap() as f64 * DT;
    assert!((tmx - at(want_max)).abs() <= 1e-9 && (tmn - at(want_min)).abs() <= 1e-9);
    // Time range 10 to 20 s (inclusive): the extremes of the steps inside it.
    run2.click_text("Time range:");
    run2.set_number("Time range:", "10");
    run2.set_number("to", "20");
    let legend = run2.text_starting("ROOFX: max").unwrap_or_else(|| panic!("{:?}", run2.shown()));
    let (mx, _, mn, _) = parse_legend(&legend, "ROOFX");
    let inside: Vec<f64> = (500..=1000).map(|k| trace[k]).collect();
    close("range max", mx, max_of(&inside), 1e-5);
    close("range min", mn, min_of(&inside), 1e-5);
    // Named set: save it, drop the range (the legend returns to the whole record), use it.
    run2.type_into("Set name:", "SET10-20");
    run2.click_text("Save set");
    assert!(run2.has_text("Saved set SET10-20."), "{:?}", run2.shown());
    run2.click_text("Time range:");
    let legend = run2.text_starting("ROOFX: max").unwrap();
    close("whole record max", parse_legend(&legend, "ROOFX").0, want_max, 1e-5);
    run2.click_text("Use");
    assert!(run2.has_text("Using set SET10-20."), "{:?}", run2.shown());
    let legend = run2.text_starting("ROOFX: max").unwrap();
    close("restored range max", parse_legend(&legend, "ROOFX").0, max_of(&inside), 1e-5);
    // A reversed range is refused with a message.
    run2.set_number("Time range:", "30");
    assert!(run2.shown().iter().any(|t| t.contains("is empty")), "{:?}", run2.shown());

    // Animation through the steps (Display, Deformed, Animate): the step shown after
    // `elapsed` seconds is floor(elapsed / 0.05) mod 3000, and the roof corner is
    // drawn at its base position plus the scale times its stored displacement.
    let mut run3 = Run::new(m.clone());
    run3.app.active_case = "HIST".into();
    run3.app.display_mode = DisplayMode::Deformed;
    run3.app.deform_scale = 10.0;
    run3.settle();
    run3.app.animate_deformed = true;
    let t0 = 100.0;
    run3.frame_at(t0, vec![]);
    assert_eq!((run3.app.active_step_idx, run3.app.use_time_step), (0, true));
    let full = m.case_results["HIST"].time_history_disp[&roof].clone();
    let base_xy = (m.points[&roof].x, m.points[&roof].y);
    for (elapsed, step) in [(0.025, 0usize), (0.075, 1), (0.62, 12), (7.31, 146), (149.97, 2999), (150.02, 0), (150.07, 1)] {
        run3.frame_at(t0 + elapsed, vec![]);
        assert_eq!(run3.app.active_step_idx, step, "elapsed {elapsed}");
        let u = full[step];
        let want = run3.screen([base_xy.0 + 10.0 * u[0], base_xy.1 + 10.0 * u[1], 30.0 + 10.0 * u[2]]);
        let circles: Vec<Pos2> = flat(&run3.last)
            .into_iter()
            .filter_map(|s| match s {
                Shape::Circle(c) if c.fill == Color32::from_rgb(100, 255, 175) && (c.radius - 3.8).abs() < 1e-6 => Some(c.center),
                _ => None,
            })
            .collect();
        assert!(circles.iter().any(|c| c.distance(want) < 1e-2), "step {step}: roof joint not drawn at {want:?}");
    }
}
