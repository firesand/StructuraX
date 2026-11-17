//! Wave 5 P4 (D07, D26, D27): vendor modal-history cases (opt-in, `STRUCTURAX_VENDOR_FIXTURES`;
//! they skip silently when the corpus is absent).
#![cfg(feature = "vendor-fixtures")]

use std::f64::consts::PI;
use std::path::PathBuf;

use sap_model::{LoadCaseType, SapModel};
use sap_solver::analyze_model;

fn vendor_dir() -> Option<PathBuf> {
    let dir = PathBuf::from(std::env::var_os("STRUCTURAX_VENDOR_FIXTURES")?);
    dir.is_dir().then_some(dir)
}

fn load(name: &str) -> Option<SapModel> {
    let path = vendor_dir()?.join(name);
    if !path.is_file() {
        eprintln!("skipped: {name} not found");
        return None;
    }
    Some(sap_format::load_sdb_file(&path).unwrap_or_else(|e| panic!("import {name}: {e}")))
}

/// Keeps `keep` and what it depends on (initial and modal cases); the others are not run.
fn only_with_dependencies(m: &mut SapModel, keep: &str) {
    let mut wanted = vec![keep.to_string()];
    let mut i = 0;
    while i < wanted.len() {
        let lc = &m.load_cases[&wanted[i]];
        for dep in lc.initial_case.iter().chain(lc.modal_case.iter()) {
            if !wanted.contains(dep) {
                wanted.push(dep.clone());
            }
        }
        i += 1;
    }
    for (name, lc) in m.load_cases.iter_mut() {
        lc.run_flag = wanted.contains(name);
    }
}

/// Displacement of an undamped SDOF (p0/k = 2 in, circular frequency `omega`) at time `t` under a
/// load ramped over `tr` and constant afterwards (Chopra 1995, eqs. 4.5.2 and 4.5.4b).
fn ramp_closed_form(omega: f64, tr: f64, t: f64) -> f64 {
    let p0_over_k = 2.0;
    if t <= tr {
        p0_over_k * (t / tr - (omega * t).sin() / (omega * tr))
    } else {
        p0_over_k * (1.0 + ((omega * (t - tr)).sin() - (omega * t).sin()) / (omega * tr))
    }
}

/// Uz of joint 2 of Example 6-001 at output time `t` (an output step of `dt = 0.1`).
fn uz_at(m: &SapModel, case: &str, t: f64) -> f64 {
    let results = &m.case_results[case];
    let step = results
        .time_steps
        .iter()
        .position(|s| (s - t).abs() < 1e-9)
        .unwrap_or_else(|| panic!("{t} s is not an output step of {case}"));
    results.time_history_disp["2"][step][2]
}

/// Example 6-001 (SDOF, k = 4 kip/in, m = 1 kip-s²/in, so ω = 2 rad/s and T = π s; load
/// p0 = 8 kip ramped over tr then constant). Closed forms from Chopra 1995, eqs. 4.5.2 and
/// 4.5.4b (as the problem statement cites them), written out here:
///   t <= tr:  u = (p0/k) [t/tr - sin(ωt)/(ω tr)]
///   t >= tr:  u = (p0/k) [1 + (sin ω(t-tr) - sin ωt)/(ω tr)]
/// The file stores the ramp functions RAMP1 (tr = π) and RAMP2 (tr = π/2) without samples (they
/// are linked), so they are rebuilt from the problem statement. MHISTA/B/C superpose the modes
/// of MODAL; before wave 5 they were integrated directly over all degrees of freedom.
#[test]
fn example_6_001_modal_histories_against_the_published_values_and_the_closed_form() {
    let Some(mut m) = load("example_6_001.sdb") else { return };
    let closed = |tr: f64, t: f64| ramp_closed_form(2.0, tr, t);
    m.functions["RAMP1"].points = vec![(0.0, 0.0), (PI, 1.0), (20.0, 1.0)];
    m.functions["RAMP2"].points = vec![(0.0, 0.0), (PI / 2.0, 1.0), (20.0, 1.0)];
    for case in ["MHISTA", "MHISTB", "MHISTC"] {
        assert_eq!(m.load_cases[case].case_type, LoadCaseType::LinearHistory);
        assert_eq!(m.load_cases[case].modal_case.as_deref(), Some("MODAL"), "{case}");
    }
    for (name, lc) in m.load_cases.iter_mut() {
        lc.run_flag = matches!(name.as_str(), "MODAL" | "MHISTA" | "MHISTB" | "MHISTC");
    }
    analyze_model(&mut m).unwrap();

    // Published (problem_6_001.pdf, printed to five digits) and closed form.
    let a1 = uz_at(&m, "MHISTA", 1.0);
    let a4 = uz_at(&m, "MHISTA", 4.0);
    let b1 = uz_at(&m, "MHISTB", 1.0);
    let b4 = uz_at(&m, "MHISTB", 4.0);
    eprintln!("6-001 measured: A1 {a1:.10} A4 {a4:.10} B1 {b1:.10} B4 {b4:.10}");
    eprintln!(
        "6-001 closed form: A1 {:.10} A4 {:.10} B1 {:.10} B4 {:.10}",
        closed(PI, 1.0),
        closed(PI, 4.0),
        closed(PI / 2.0, 1.0),
        closed(PI / 2.0, 4.0)
    );
    for (value, published, what) in [
        (a1, 0.34718, "MHISTA t=1"),
        (a4, 2.0000, "MHISTA t=4"),
        (b1, 0.69436, "MHISTB t=1"),
        (b4, 0.74030, "MHISTB t=4"),
    ] {
        // Printed to five digits (0.34718, 2.0000, 0.69436, 0.74030): half a unit of the last
        // printed digit.
        assert!((value - published).abs() <= 5.0e-6, "{what}: {value} vs {published}");
    }
    // The file's link carries a mass of 1e-5 (a stored property of the example; half of it
    // sits on joint 2), so the moving mass is 1.000005 and not the 1 the statement idealises.
    assert_eq!(m.link_props.values().map(|p| p.mass).collect::<Vec<_>>(), [1.0e-5]);
    assert_eq!(m.points["2"].mass[2], 1.0);
    let omega_file = 2.0 / (1.0_f64 + 0.5e-5).sqrt();
    for (value, tr, t, what) in [
        (a1, PI, 1.0, "MHISTA t=1"),
        (a4, PI, 4.0, "MHISTA t=4"),
        (b1, PI / 2.0, 1.0, "MHISTB t=1"),
        (b4, PI / 2.0, 4.0, "MHISTB t=4"),
    ] {
        // Against the closed form of the idealised problem (m = 1): the 1e-5 link mass shifts
        // omega by 2.5e-6 relative, which moves u by at most omega*t*2.5e-6 ~ 2e-5 relative.
        let ideal = ramp_closed_form(2.0, tr, t);
        assert!(
            (value - ideal).abs() <= 2.0e-5 * ideal.abs(),
            "{what}: {value} vs closed form {ideal}"
        );
        // Against the closed form with the file's mass: the modal path integrates the
        // piecewise-linear load exactly between breakpoints (a breakpoint inside an output step
        // splits it), so only roundoff of the eigenproblem is left.
        let with_link_mass = ramp_closed_form(omega_file, tr, t);
        eprintln!("6-001 {what}: {value:.12} vs file-mass closed form {with_link_mass:.12}");
        assert!(
            (value - with_link_mass).abs() <= 1.0e-9 * with_link_mass.abs().max(1.0),
            "{what}: {value} vs closed form with the link mass {with_link_mass}"
        );
    }

    // Very large damping (xi = 0.999) analyses; the value is printed, not asserted (the
    // problem prints none for it).
    eprintln!(
        "6-001 MODAL period {:.12} (pi = {:.12}); link ke {:?} mass {:?}; point 2 mass {:?}",
        m.case_results["MODAL"].modes[0].period_sec,
        PI,
        m.link_props.values().map(|p| p.ke[0]).collect::<Vec<_>>(),
        m.link_props.values().map(|p| p.mass).collect::<Vec<_>>(),
        m.points["2"].mass
    );
    let c = &m.case_results["MHISTC"];
    eprintln!("6-001 MHISTC (xi 0.999) Uz at 1 s = {:.6}, at 4 s = {:.6}", uz_at(&m, "MHISTC", 1.0), uz_at(&m, "MHISTC", 4.0));
    assert_eq!(c.time_steps.len(), 101);
}

/// Example 1-015 MHIST1 is a periodic modal history (native motion type 2). As imported the
/// model is refused for its frame added mass (a different, older refusal, which names the
/// Modal case it depends on); with that entry removed, only the periodic history is left and
/// it is refused naming the case and the reason.
#[test]
fn a_periodic_history_is_refused_by_name() {
    let Some(mut m) = load("example_1_015.sdb") else { return };
    assert!(m
        .unmodeled_source
        .iter()
        .any(|u| u.description == "periodic modal history of case 'MHIST1' is not implemented"));
    assert_eq!(m.load_cases["MHIST1"].modal_case.as_deref(), Some("MODAL1"));
    only_with_dependencies(&mut m, "MHIST1");
    let error = analyze_model(&mut m).unwrap_err().to_string();
    assert!(error.contains("frame added-mass"), "{error}");

    m.unmodeled_source.retain(|u| !u.description.contains("added-mass"));
    let error = analyze_model(&mut m).unwrap_err().to_string();
    assert!(
        error.contains("'MHIST1'") && error.contains("periodic modal history"),
        "{error}"
    );
    assert!(m.case_results.is_empty());
}

/// Example 6-005a NLMHIST1 is a nonlinear modal (FNA) history: it used to be integrated
/// directly over all degrees of freedom; it is refused naming the case and the reason.
#[test]
fn an_fna_history_is_refused_by_name() {
    let Some(mut m) = load("example_6_005a.sdb") else { return };
    assert_eq!(m.load_cases["NLMHIST1"].case_type, LoadCaseType::NonlinearHistory);
    only_with_dependencies(&mut m, "NLMHIST1");
    let error = analyze_model(&mut m).unwrap_err().to_string();
    assert!(
        error.contains("'NLMHIST1'") && error.contains("nonlinear modal (FNA) time history"),
        "{error}"
    );
    assert!(m.case_results.is_empty());
}

/// Example 1-022: MHIST1 (type 5, modal case MODAL, constant ratio 0.05) imports with its modal
/// case and damping; MHIST2 (type 2, Rayleigh) names MODAL too and the modal path refuses
/// Rayleigh damping.
#[test]
fn example_1_022_histories_import_their_modal_case() {
    let Some(mut m) = load("example_1_022.sdb") else { return };
    assert_eq!(m.load_cases["MHIST1"].modal_case.as_deref(), Some("MODAL"));
    only_with_dependencies(&mut m, "MHIST2");
    let error = analyze_model(&mut m).unwrap_err().to_string();
    assert!(error.contains("'MHIST2'") && error.contains("Rayleigh"), "{error}");
    let Some(m) = load("example_1_022.sdb") else { return };
    assert_eq!(m.load_cases["MHIST1"].modal_damping, 0.05);
    assert_eq!(m.load_cases["MHIST2"].modal_case.as_deref(), Some("MODAL"));
    assert_ne!(m.load_cases["MHIST2"].rayleigh_damping, (0.0, 0.0));
    assert!(m.unmodeled_source.iter().all(|u| !u.description.contains("1-022")));
}

/// Listing of every modal-history case type (native 5 and 6) of the corpus: how it imports and
/// whether it runs. Run with `-- --ignored --nocapture`; the output is the before/after table
/// of step 4a. Nothing is asserted: this is a measurement. (Functions whose samples are linked
/// files are not attached, so some cases are refused for that reason before and after.)
#[test]
#[ignore]
fn list_modal_history_cases_of_the_corpus() {
    let Some(dir) = vendor_dir() else { return };
    let mut files: Vec<_> = std::fs::read_dir(&dir)
        .unwrap()
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "sdb"))
        .collect();
    files.sort();
    for path in files {
        let file = path.file_name().unwrap().to_string_lossy().to_string();
        let Ok(model) = sap_format::load_sdb_file(&path) else { continue };
        for (name, lc) in &model.load_cases {
            if !matches!(
                lc.case_type,
                LoadCaseType::LinearHistory | LoadCaseType::NonlinearHistory
            ) {
                continue;
            }
            let mut m = model.clone();
            only_with_dependencies(&mut m, name);
            let outcome = match analyze_model(&mut m) {
                Ok(()) => "RUNS".to_string(),
                Err(e) => {
                    let e = e.to_string();
                    format!("REFUSED {}", &e[..e.len().min(140)])
                }
            };
            println!(
                "{file} {name} {:?} modal_case={:?} xi={} rayleigh={:?}: {outcome}",
                lc.case_type, lc.modal_case, lc.modal_damping, lc.rayleigh_damping
            );
        }
    }
}
