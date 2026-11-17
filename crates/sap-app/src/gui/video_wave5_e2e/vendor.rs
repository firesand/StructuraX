//! Opt-in vendor tests (`STRUCTURAX_VENDOR_FIXTURES`, skipped silently without the corpus): the
//! files are opened with File > Open / Save, analysed with Analyze > Run Analysis and read through
//! the plot window, the joint read-out and the status line.
//!
//! Independent sides: the values printed in the problem statements (`problem_6_001.pdf`: 0.34718,
//! 2.0000, 0.69436, 0.74030 in; `problem_2_015.pdf`: -1.077 in), the closed form of Chopra 1995
//! (eqs. 4.5.2 and 4.5.4b) for the SDOF ramp, and Ugural's centre deflection of the orthotropic
//! plate as the document quotes it (w0 = q a^4 / (8 (3 Dx + 2 H + 3 Dy)) = 1.069195 in).

use super::ext::{analyze, menu};
use super::harness::{texts, Run};
use crate::gui::DisplayMode;
use sap_model::SapModel;
use std::f64::consts::PI;
use std::path::PathBuf;

fn vendor_dir() -> Option<PathBuf> {
    std::env::var_os("STRUCTURAX_VENDOR_FIXTURES").map(PathBuf::from)
}

/// A run of the application on `file` opened with the file window, or `None` without the corpus.
fn open(file: &str, edit: impl FnOnce(&mut SapModel)) -> Option<Run> {
    let path = vendor_dir()?.join(file);
    if !path.is_file() {
        eprintln!("skipped: {} not found", path.display());
        return None;
    }
    let mut run = Run::new(SapModel::new(sap_units::Units::KipInF));
    menu(&mut run, &["File", "Open / Save Model (.s2k / .sdb)... (Ctrl+O)"]);
    run.app.file_path_input = path.display().to_string();
    run.settle();
    run.click_text("📂 Open Model");
    assert!(run.app.status_msg.starts_with("Opened "), "{file}: {}", run.app.status_msg);
    edit(&mut run.app.model);
    run.settle();
    Some(run)
}

/// Displacement of the undamped SDOF of Example 6-001 (omega = 2 rad/s, p0/k = 2 in) at `t` under
/// a load ramped over `tr` and constant afterwards (Chopra 1995, eqs. 4.5.2 and 4.5.4b).
fn ramp(tr: f64, t: f64) -> f64 {
    let omega = 2.0;
    if t <= tr {
        2.0 * (t / tr - (omega * t).sin() / (omega * tr))
    } else {
        2.0 * (1.0 + ((omega * (t - tr)).sin() - (omega * t).sin()) / (omega * tr))
    }
}

/// The legend line "name: max ... min ..." painted for the plot function `name`:
/// (max, time of max, min, time of min).
fn legend_of(run: &Run, name: &str) -> (f64, f64, f64, f64) {
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

#[test]
fn vendor_example_6_001_modal_histories_through_the_plot_window_against_the_published_values() {
    let Some(mut run) = open("example_6_001.sdb", |m| {
        // The file links its ramp functions without samples: the problem statement's ramps over
        // pi s (RAMP1) and pi/2 s (RAMP2), constant afterwards, are attached (as the solver's own
        // test of this example does). Only the modal case and its histories are run.
        m.functions["RAMP1"].points = vec![(0.0, 0.0), (PI, 1.0), (20.0, 1.0)];
        m.functions["RAMP2"].points = vec![(0.0, 0.0), (PI / 2.0, 1.0), (20.0, 1.0)];
        for (name, lc) in m.load_cases.iter_mut() {
            lc.run_flag = matches!(name.as_str(), "MODAL" | "MHISTA" | "MHISTB");
        }
    }) else {
        return;
    };
    analyze(&mut run);
    assert!(run.app.model.is_locked, "{}", run.app.status_msg);
    // Display > Show Plot Functions...: the response of joint 2, U3, in the case MHISTA and MHISTB.
    menu(&mut run, &["Display", "Show Plot Functions..."]);
    run.click_text("Plot functions");
    let mut added = Vec::new();
    for (case, name) in [("MHISTA", "A"), ("MHISTB", "B")] {
        run.choose("Case:", case);
        run.choose("Joint:", "2");
        run.choose("Component:", "U3");
        run.click_text("Displacement");
        run.type_into("Name:", name);
        run.click_text("Add response function");
        added.push(name);
    }
    // Tick both functions as vertical.
    let verticals = |run: &Run| {
        let mut v: Vec<_> = texts(&run.last).into_iter().filter(|(t, _)| t == "vertical").map(|(_, r)| r).collect();
        v.sort_by(|a, b| a.center().y.total_cmp(&b.center().y));
        v
    };
    for n in 0..2 {
        let r = verticals(&run)[n];
        run.click(r.center());
    }
    // The values at t = 1 s and t = 4 s (output steps of 0.1 s): a range of 1 ms holds that step only.
    run.click_text("Time range:");
    for (t, a_pub, b_pub, a_tr, b_tr) in [(1.0, 0.34718, 0.69436, PI, PI / 2.0), (4.0, 2.0000, 0.74030, PI, PI / 2.0)] {
        run.set_number("Time range:", &t.to_string());
        run.set_number("to", &(t + 0.001).to_string());
        let (a, _, a_min, _) = legend_of(&run, "A");
        let (b, _, _, _) = legend_of(&run, "B");
        eprintln!("6-001 plot window at t = {t}: MHISTA {a}, MHISTB {b}");
        assert_eq!(a, a_min, "one step in the range");
        // Printed to five digits (the legend shows five decimals in scientific notation, the
        // statement five digits): half a unit of the last printed digit of the statement.
        assert!((a - a_pub).abs() <= 5.0e-6, "MHISTA at {t}: {a} against the published {a_pub}");
        assert!((b - b_pub).abs() <= 5.0e-6, "MHISTB at {t}: {b} against the published {b_pub}");
        // The closed form of the idealised problem (m = 1): the 1e-5 link mass of the file shifts omega
        // by 2.5e-6 relative, which moves u by at most 2e-5 relative.
        for (v, tr, what) in [(a, a_tr, "MHISTA"), (b, b_tr, "MHISTB")] {
            let ideal = ramp(tr, t);
            assert!((v - ideal).abs() <= 2.0e-5 * ideal.abs() + 6.0e-6, "{what} at {t}: {v} against the closed form {ideal}");
        }
    }
}

#[test]
fn vendor_example_2_015a_thin_deformed_shape_read_out_of_the_centre_joint() {
    let Some(mut run) = open("example_2_015a_thin.sdb", |_| {}) else { return };
    analyze(&mut run);
    assert!(run.app.model.is_locked, "{}", run.app.status_msg);
    let case = run.app.model.case_results.keys().next().unwrap().clone();
    run.app.active_case = case.clone();
    // The centre joint is the one at the origin of the plate; Display > Show Deformed Shape (F6)
    // and a right-click on it opens the joint read-out.
    let centre = run
        .app
        .model
        .points
        .values()
        .find(|p| p.x.abs() < 1e-9 && p.y.abs() < 1e-9)
        .unwrap_or_else(|| panic!("no joint at the centre"))
        .clone();
    menu(&mut run, &["Display", "Show Deformed Shape (F6)"]);
    assert_eq!(run.app.display_mode, DisplayMode::Deformed);
    run.right_click(run.screen([centre.x, centre.y, centre.z]));
    assert!(run.has_text("Joint Displacements"), "the read-out window: {:?}", run.shown());
    let header = run.text_starting("Joint ").expect("read-out header");
    assert!(header.contains(&centre.name), "{header}");
    let stored_u3 = run.app.model.case_results[&case].joint_displacements[&centre.name][2];
    // The painted U3: five significant digits. Published -1.077 in (thin mesh 8 x 16) to one
    // percent (the measured difference is 0.7 %: the vendor's value is one element formulation's
    // solution of the plate), and Ugural's closed form -1.069195 to 1.5 % (discretisation of the
    // 8 x 16 mesh), as in the solver's own test of this example.
    let u3_row = texts(&run.last)
        .into_iter()
        .find(|(t, _)| t.starts_with("U3"))
        .unwrap_or_else(|| panic!("no U3 row in {:?}", run.shown()));
    let value = texts(&run.last)
        .into_iter()
        .filter(|(t, r)| (r.center().y - u3_row.1.center().y).abs() < 3.0 && r.left() > u3_row.1.right() && t.parse::<f64>().is_ok())
        .map(|(t, _)| t.parse::<f64>().unwrap())
        .next()
        .unwrap_or_else(|| panic!("no value on the U3 row in {:?}", run.shown()));
    eprintln!("2-015a thin: painted centre U3 {value} (stored {stored_u3})");
    assert!((value + 1.077).abs() <= 0.01 * 1.077, "U3 {value} against the published -1.077");
    assert!((value + 1.069_195).abs() <= 0.015 * 1.069_195, "U3 {value} against the closed form -1.069195");
    // The read-out is the stored result.
    let stored = stored_u3;
    assert!((value - stored).abs() <= 1e-4 * stored.abs(), "painted {value} stored {stored}");
}

#[test]
fn vendor_example_6_010_refused_modal_case_is_named_on_the_status_line() {
    let Some(mut run) = open("example_6_010.sdb", |_| {}) else { return };
    analyze(&mut run);
    let status = run.app.status_msg.clone();
    // The Modal case of this example carries a Ritz starting-load record for link degrees of
    // freedom that the importer does not model (D36): the analysis is refused, naming the case.
    assert!(status.starts_with("Analysis Error: "), "{status}");
    let modal = run
        .app
        .model
        .load_cases
        .iter()
        .find(|(_, lc)| lc.case_type == sap_model::LoadCaseType::Modal)
        .map(|(n, _)| n.clone())
        .expect("a modal case");
    assert!(status.contains(&format!("case '{modal}' depends on imported source data that is not modeled")), "{status}");
    assert!(status.contains("Ritz starting load"), "the refusal must say what is not modeled: {status}");
    // The line the user reads: the painted text carries the case name, and the model's own
    // unmodeled-source record is the one that refused (not another refusal that mentions the case).
    let painted = run.texts().into_iter().find(|t| t.starts_with("Analysis Error: ")).unwrap_or_else(|| panic!("{:?}", run.shown()));
    assert!(painted.contains(&format!("'{modal}'")), "painted: {painted}");
    assert!(!run.app.model.is_locked && run.app.model.case_results.is_empty(), "no results of a refused analysis");
}
