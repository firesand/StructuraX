//! Main entry point for `structurax` (`sap-app`), supporting both interactive
//! Desktop CAD GUI mode (`structurax` / `structurax gui`) and headless CLI automation
//! (`structurax verify`, `structurax analyze`, `structurax inspect-pro`,
//! `structurax inspect-sdb`, `structurax copy-native-sdb`, `structurax export-native-sdb`).

mod gui;

use anyhow::Result;
use clap::{Parser, Subcommand};
use sap_design::{try_run_concrete_design, try_run_steel_design};
use sap_format::{
    inspect_native_sdb_file, load_model_with_report, save_native_sdb_file,
    save_s2k_file_with_report, save_sdb_file_with_report,
};
use sap_section::ProDatabase;
use sap_solver::{analyze_model, benchmarks::run_verification_suite};
use std::path::PathBuf;

#[derive(Parser, Debug)]
#[command(
    name = "structurax",
    version = "1.0.0",
    about = "StructuraX — 3D Finite Element Structural Analysis & Code-Based Member Design Suite"
)]
struct Cli {
    #[command(subcommand)]
    command: Option<Commands>,
}

#[derive(Subcommand, Debug)]
enum Commands {
    /// Launch the interactive 2D/3D StructuraX Desktop CAD GUI
    Gui {
        /// Optional .s2k / .$2k / .sdb structural model file to open on startup
        file: Option<PathBuf>,
    },
    /// Run the built-in structural mechanics verification benchmark suite
    Verify,
    /// Load, analyze, and optionally run steel/concrete member design on a .s2k or .sdb model
    Analyze {
        /// Input .s2k, .$2k, or .sdb structural model file
        input: PathBuf,
        /// Optional output file (.json, .s2k, or .sdb)
        #[arg(short, long)]
        output: Option<PathBuf>,
        /// Run code-based member design for all present steel and concrete frame members
        #[arg(long)]
        design: bool,
        /// Run steel frame member design explicitly
        #[arg(long)]
        design_steel: bool,
        /// Run concrete frame member design explicitly
        #[arg(long)]
        design_concrete: bool,
        /// Override steel design code (e.g. "AISC 360-10", "AISC 360-10 ASD", "Eurocode 3-2005")
        #[arg(long)]
        steel_code: Option<String>,
        /// Override concrete design code (e.g. "ACI 318-14", "Eurocode 2-2004")
        #[arg(long)]
        concrete_code: Option<String>,
    },
    /// Inspect a binary .pro structural section property database
    InspectPro {
        /// Path to the .pro file (e.g., sections/aisc14.pro)
        pro_file: PathBuf,
        /// Optional section name to inspect in detail (e.g., W14X90)
        section: Option<String>,
        /// List all sections instead of truncating at 50 entries
        #[arg(long)]
        all: bool,
        /// Case-insensitive substring filter for section names
        #[arg(long)]
        filter: Option<String>,
    },
    /// Inspect the record inventory of a binary .sdb structural model file
    InspectSdb {
        /// Path to the binary .sdb file
        input: PathBuf,
    },
    /// Copy a binary .sdb structural model file byte-for-byte without overwriting the destination
    CopyNativeSdb {
        /// Path to the input .sdb file
        input: PathBuf,
        /// New destination path; must not already exist
        output: PathBuf,
    },
    /// Export a .s2k or .sdb model to a binary stream .sdb file
    ExportNativeSdb {
        /// Input .s2k, .$2k, or .sdb model file
        input: PathBuf,
        /// New destination binary .sdb file path; must not already exist
        output: PathBuf,
    },
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    match cli.command {
        None => launch_gui(None),
        Some(Commands::Gui { file }) => launch_gui(file),
        Some(Commands::Verify) => run_cli_verify(),
        Some(Commands::Analyze {
            input,
            output,
            design,
            design_steel,
            design_concrete,
            steel_code,
            concrete_code,
        }) => run_cli_analyze(
            &input,
            output.as_deref(),
            AnalyzeDesignOptions {
                design,
                design_steel,
                design_concrete,
                steel_code: steel_code.as_deref(),
                concrete_code: concrete_code.as_deref(),
            },
        ),
        Some(Commands::InspectPro {
            pro_file,
            section,
            all,
            filter,
        }) => run_cli_inspect_pro(&pro_file, section.as_deref(), all, filter.as_deref()),
        Some(Commands::InspectSdb { input }) => run_cli_inspect_sdb(&input),
        Some(Commands::CopyNativeSdb { input, output }) => run_cli_copy_native_sdb(&input, &output),
        Some(Commands::ExportNativeSdb { input, output }) => {
            run_cli_export_native_sdb(&input, &output)
        }
    }
}

fn launch_gui(initial_file: Option<PathBuf>) -> Result<()> {
    let native_options = eframe::NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default()
            .with_inner_size([1440.0, 900.0])
            .with_title("StructuraX v1.0.0 — 3D Finite Element Structural Analysis & Code-Based Member Design Suite"),
        ..Default::default()
    };
    eframe::run_native(
        "StructuraX",
        native_options,
        Box::new(move |_cc| Ok(Box::new(gui::SapGuiApp::new(initial_file)))),
    )
    .map_err(|e| anyhow::anyhow!("GUI error: {e}"))
}

fn run_cli_verify() -> Result<()> {
    let rows = run_verification_suite();
    println!(
        "{:<28} | {:<24} | {:<18} | {:>12} | {:>12} | {:>8} | {:<6}",
        "Benchmark", "Load Case", "Parameter", "Target Ref", "StructuraX", "% Diff", "Status"
    );
    println!("{}", "-".repeat(122));
    let mut passed_count = 0usize;
    for r in &rows {
        if r.passed {
            passed_count += 1;
        }
        println!(
            "{:<28} | {:<24} | {:<18} | {:>12.5} | {:>12.5} | {:>7.4}% | {:<6}",
            r.example,
            r.load_case,
            r.parameter,
            r.sap2000_ref,
            r.computed,
            r.pct_diff,
            if r.passed { "PASS" } else { "FAIL" }
        );
    }
    println!("{}", "-".repeat(122));
    println!(
        "Summary: {passed_count} / {} implemented reference checks PASSED (selected synthetic examples, not the complete vendor suite).",
        rows.len()
    );
    if rows.is_empty() || passed_count != rows.len() {
        anyhow::bail!("{} verification checks failed", rows.len() - passed_count);
    }
    Ok(())
}

#[derive(Debug, Clone, Copy, Default)]
struct AnalyzeDesignOptions<'a> {
    design: bool,
    design_steel: bool,
    design_concrete: bool,
    steel_code: Option<&'a str>,
    concrete_code: Option<&'a str>,
}

fn paths_refer_to_same_file(a: &std::path::Path, b: &std::path::Path) -> bool {
    if a == b {
        return true;
    }
    match (std::fs::canonicalize(a), std::fs::canonicalize(b)) {
        (Ok(ca), Ok(cb)) => ca == cb,
        _ => false,
    }
}

fn run_cli_analyze(
    input: &std::path::Path,
    output: Option<&std::path::Path>,
    design_opts: AnalyzeDesignOptions<'_>,
) -> Result<()> {
    if let Some(out_path) = output {
        anyhow::ensure!(
            !paths_refer_to_same_file(input, out_path),
            "Refusing to overwrite input file '{}'; specify a distinct --output path",
            input.display()
        );
    }

    let report = load_model_with_report(input)?;
    for warning in &report.warnings {
        eprintln!("Import warning: {warning}");
    }
    let mut model = report.model;
    println!(
        "Loaded model '{}' (Units: {}): {} joints, {} frames, {} areas, {} solids, {} links, {} load cases, {} combos.",
        model.model_name,
        model.present_units.s2k_label(),
        model.points.len(),
        model.frames.len(),
        model.areas.len(),
        model.solids.len(),
        model.links.len(),
        model.load_cases.len(),
        model.load_combos.len()
    );
    analyze_model(&mut model)?;
    for (cname, cr) in &model.case_results {
        let mut details = vec![
            format!("{} joint disp", cr.joint_displacements.len()),
            format!("{} reactions", cr.joint_reactions.len()),
        ];
        if !cr.frame_forces.is_empty() {
            details.push(format!("{} frames", cr.frame_forces.len()));
        }
        if !cr.area_results.is_empty() {
            details.push(format!("{} areas", cr.area_results.len()));
        }
        if !cr.solid_results.is_empty() {
            details.push(format!("{} solids", cr.solid_results.len()));
        }
        if !cr.link_forces.is_empty() {
            details.push(format!("{} links", cr.link_forces.len()));
        }
        if !cr.modes.is_empty() {
            details.push(format!("{} modes", cr.modes.len()));
        }
        if !cr.buckling_factors.is_empty() {
            details.push(format!(
                "{} buckling modes (first factor={:.4})",
                cr.buckling_factors.len(),
                cr.buckling_factors[0]
            ));
        }
        if !cr.time_steps.is_empty() {
            details.push(format!("{} time steps", cr.time_steps.len()));
        }
        if cr.base_reactions.iter().any(|v| v.abs() > 1e-9) {
            details.push(format!(
                "base F=[{:.3}, {:.3}, {:.3}]",
                cr.base_reactions[0], cr.base_reactions[1], cr.base_reactions[2]
            ));
        }
        println!("  Result '{}': {}", cname, details.join(", "));
    }

    if let Some(code) = design_opts.steel_code {
        model.steel_design_code = code.to_string();
    }
    if let Some(code) = design_opts.concrete_code {
        model.concrete_design_code = code.to_string();
    }

    let has_steel_frames = model.frames.values().any(|f| {
        model
            .frame_sections
            .get(&f.section)
            .and_then(|s| model.materials.get(&s.material))
            .is_some_and(|m| m.mat_type == sap_model::MatType::Steel)
    });
    let has_concrete_frames = model.frames.values().any(|f| {
        model
            .frame_sections
            .get(&f.section)
            .and_then(|s| model.materials.get(&s.material))
            .is_some_and(|m| m.mat_type == sap_model::MatType::Concrete)
    });

    let run_steel = design_opts.design_steel
        || design_opts.steel_code.is_some()
        || (design_opts.design && has_steel_frames);
    let run_concrete = design_opts.design_concrete
        || design_opts.concrete_code.is_some()
        || (design_opts.design && has_concrete_frames);

    if design_opts.design && !run_steel && !run_concrete {
        println!("  Design skipped: model contains 0 steel or concrete frame members.");
    }
    if run_steel {
        try_run_steel_design(&mut model)?;
        let total = model.steel_design_results.len();
        let failed = model
            .steel_design_results
            .values()
            .filter(|r| r.status != "OK")
            .count();
        let max_dc = model
            .steel_design_results
            .values()
            .map(|r| r.total_ratio)
            .fold(0.0_f64, f64::max);
        println!(
            "  Steel design ({}): {} members evaluated ({} OK, {} failed, max D/C = {:.3}).",
            model.steel_design_code,
            total,
            total - failed,
            failed,
            max_dc
        );
    }
    if run_concrete {
        try_run_concrete_design(&mut model)?;
        let total = model.concrete_design_results.len();
        let failed = model
            .concrete_design_results
            .values()
            .filter(|r| r.status != "OK")
            .count();
        let max_pmm = model
            .concrete_design_results
            .values()
            .filter(|r| r.is_column)
            .map(|r| r.pmm_ratio)
            .fold(0.0_f64, f64::max);
        println!(
            "  Concrete design ({}): {} members evaluated ({} OK, {} failed, max column PMM = {:.3}).",
            model.concrete_design_code,
            total,
            total - failed,
            failed,
            max_pmm
        );
    }

    if let Some(out_path) = output {
        let ext = out_path
            .extension()
            .and_then(|e| e.to_str())
            .unwrap_or("")
            .to_ascii_lowercase();
        if ext == "json" {
            let json = serde_json::to_string_pretty(&model)?;
            std::fs::write(out_path, json)?;
        } else if ext == "sdb" {
            eprintln!("Output warning: .sdb output is a private StructuraX container; use export-native-sdb for binary stream export or .s2k for text interoperability.");
            for warning in save_sdb_file_with_report(&model, out_path)? {
                eprintln!("Output warning: {warning}");
            }
        } else {
            eprintln!("Output warning: .s2k output stores model definitions only; computed analysis and design results are not persisted (use .json or .sdb to save results).");
            for warning in save_s2k_file_with_report(&model, out_path)? {
                eprintln!("Output warning: {warning}");
            }
        }
        println!("Saved output to {}", out_path.display());
    }
    Ok(())
}

fn run_cli_inspect_pro(
    pro_file: &std::path::Path,
    section: Option<&str>,
    show_all: bool,
    filter: Option<&str>,
) -> Result<()> {
    let db = ProDatabase::from_file(pro_file)?;
    println!(
        "Loaded .PRO database '{}' (Length unit: {:?}, {} sections).",
        pro_file.display(),
        db.length_unit,
        db.sections.len()
    );
    if let Some(sec_name) = section {
        if let Some(rec) = db
            .sections
            .iter()
            .find(|s| s.name.eq_ignore_ascii_case(sec_name))
        {
            println!("{rec:#?}");
        } else {
            anyhow::bail!("Section '{sec_name}' not found in {}", pro_file.display());
        }
    } else {
        let needle = filter.map(|f| f.to_ascii_uppercase());
        let matching: Vec<&str> = db
            .sections
            .iter()
            .filter(|s| {
                needle
                    .as_ref()
                    .is_none_or(|n| s.name.to_ascii_uppercase().contains(n))
            })
            .map(|s| s.name.as_str())
            .collect();
        let limit = if show_all { matching.len() } else { 50 };
        for name in matching.iter().take(limit) {
            println!("  {name}");
        }
        if matching.len() > limit {
            println!(
                "  ... showing {} of {} matching sections (pass --all or --filter <SUBSTR> to inspect more).",
                limit,
                matching.len()
            );
        }
    }
    Ok(())
}

fn run_cli_inspect_sdb(input: &std::path::Path) -> Result<()> {
    let document = inspect_native_sdb_file(input)?;
    println!("{}", serde_json::to_string_pretty(document.inventory())?);
    Ok(())
}

fn run_cli_copy_native_sdb(input: &std::path::Path, output: &std::path::Path) -> Result<()> {
    // The document retains the original compressed container, including gzip metadata. Compare
    // it with the current input before creating the output to catch a source changed mid-read.
    let document = inspect_native_sdb_file(input)?;
    let original = std::fs::read(input)?;
    let bytes = document.to_bytes()?;
    anyhow::ensure!(
        bytes == original,
        "Input changed during inspection; refusing to copy an unverified snapshot"
    );

    use std::io::Write;
    let mut destination = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output)
        .map_err(|error| {
            anyhow::anyhow!(
                "Cannot create {} without overwriting: {error}",
                output.display()
            )
        })?;
    if let Err(error) = destination.write_all(&bytes) {
        drop(destination);
        let _ = std::fs::remove_file(output);
        return Err(error.into());
    }
    drop(destination);

    let copied = std::fs::read(output)?;
    if copied != original {
        let _ = std::fs::remove_file(output);
        anyhow::bail!("Copied file did not match the original input byte-for-byte");
    }
    println!(
        "Copied binary SDB to {} ({} bytes; verified exact original bytes).",
        output.display(),
        copied.len()
    );
    Ok(())
}

fn run_cli_export_native_sdb(input: &std::path::Path, output: &std::path::Path) -> Result<()> {
    let report = load_model_with_report(input)?;
    for warning in &report.warnings {
        eprintln!("Import warning: {warning}");
    }
    save_native_sdb_file(&report.model, output)?;
    let doc = inspect_native_sdb_file(output)?;
    let inv = doc.inventory();
    anyhow::ensure!(
        inv.parsed_bytes == inv.stream_bytes,
        "Exported native SDB stream was only partially parsed ({}/{} bytes)",
        inv.parsed_bytes,
        inv.stream_bytes
    );
    println!(
        "Exported experimental SDB stream to {} ({} bytes consumed by the internal schema parser; vendor application acceptance is unverified).",
        output.display(),
        inv.stream_bytes
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn native_sdb_cli_copy_is_exact_and_never_overwrites() {
        let unique = format!(
            "sap-app-native-copy-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        );
        let output = std::env::temp_dir().join(format!("{unique}.sdb"));
        let input = std::env::temp_dir().join(format!("{unique}-input.sdb"));
        let mut model = sap_model::SapModel::new(sap_units::Units::KipInF);
        // This test exercises the explicitly supported steel-only native subset.
        // The separate handwritten-S2K test also covers the default concrete metadata.
        model.materials.shift_remove("4000Psi");
        model.area_sections.clear();
        save_native_sdb_file(&model, &input).unwrap();
        let original = std::fs::read(&input).expect("native SDB fixture must be available");

        run_cli_copy_native_sdb(&input, &output).unwrap();
        assert_eq!(std::fs::read(&output).unwrap(), original);

        let error = run_cli_copy_native_sdb(&input, &output).unwrap_err();
        assert!(error.to_string().contains("without overwriting"));
        assert_eq!(std::fs::read(&output).unwrap(), original);
        std::fs::remove_file(output).unwrap();
        std::fs::remove_file(input).unwrap();
    }

    #[test]
    fn native_sdb_cli_export_roundtrips_and_passes_inspection() {
        let unique = format!(
            "sap-app-native-export-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        );
        let output = std::env::temp_dir().join(format!("{unique}.sdb"));
        let input = std::env::temp_dir().join(format!("{unique}-input.sdb"));
        let mut model = sap_model::SapModel::new(sap_units::Units::KipInF);
        // This test exercises the explicitly supported steel-only native subset.
        // The separate handwritten-S2K test also covers the default concrete metadata.
        model.materials.shift_remove("4000Psi");
        model.area_sections.clear();
        save_native_sdb_file(&model, &input).unwrap();

        run_cli_export_native_sdb(&input, &output).unwrap();
        let doc = inspect_native_sdb_file(&output).unwrap();
        assert_eq!(doc.inventory().parsed_bytes, doc.inventory().stream_bytes);
        let original = std::fs::read(&output).unwrap();
        let error = run_cli_export_native_sdb(&input, &output).unwrap_err();
        assert!(error.to_string().contains("without overwriting"));
        assert_eq!(std::fs::read(&output).unwrap(), original);
        std::fs::remove_file(output).unwrap();
        std::fs::remove_file(input).unwrap();
    }

    #[test]
    fn native_sdb_cli_exports_handwritten_s2k_without_discarding_opaque_fields() {
        let unique = format!(
            "structurax-handwritten-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        );
        let input = std::env::temp_dir().join(format!("{unique}.s2k"));
        let output = std::env::temp_dir().join(format!("{unique}.sdb"));
        let source = "TABLE: \"JOINT COORDINATES\"\n Joint=A XorR=0 Y=0 Z=0\n Joint=B XorR=120 Y=0 Z=0\nTABLE: \"CONNECTIVITY - FRAME\"\n Frame=F JointI=A JointJ=B\nEND TABLE DATA\n";
        std::fs::write(&input, source).unwrap();
        run_cli_export_native_sdb(&input, &output).unwrap();
        assert_eq!(std::fs::read_to_string(&input).unwrap(), source);
        let reopened = sap_format::load_sdb_file(&output).unwrap();
        assert_eq!(reopened.points["B"].x, 120.);
        assert_eq!(reopened.frames["F"].point_i, "A");
        assert_eq!(reopened.frames["F"].point_j, "B");
        let bytes = std::fs::read(&output).unwrap();
        assert!(run_cli_export_native_sdb(&input, &output)
            .unwrap_err()
            .to_string()
            .contains("without overwriting"));
        assert_eq!(std::fs::read(&output).unwrap(), bytes);
        std::fs::remove_file(&output).unwrap();
        std::fs::write(
            &input,
            source.replace("Joint=A XorR=0", "Joint=A VendorTag=retain XorR=0"),
        )
        .unwrap();
        let error = run_cli_export_native_sdb(&input, &output).unwrap_err();
        assert!(error.to_string().contains("VENDORTAG"), "{error}");
        assert!(!output.exists());
        std::fs::remove_file(input).unwrap();
    }

    #[test]
    fn cli_analyze_design_single_material_and_overwrite_guard() {
        let unique = format!(
            "structurax-cli-analyze-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        );
        let input = std::env::temp_dir().join(format!("{unique}.s2k"));
        let json_out = std::env::temp_dir().join(format!("{unique}.json"));

        let mut model = sap_model::SapModel::new(sap_units::Units::KipInF);
        // Simulate a steel-only model carrying an unimplemented concrete code string
        model.concrete_design_code = "LEGACY-CONCRETE-99".to_string();
        let sec =
            sap_section::make_i_section("W14_TEST", "A992Fy50", 14.0, 10.0, 0.5, 0.35, 10.0, 0.5);
        model.frame_sections.insert(sec.name.clone(), sec);
        model.add_point("1", 0.0, 0.0, 0.0);
        model.points.get_mut("1").unwrap().restraint = [true; 6];
        model.add_point("2", 120.0, 0.0, 0.0);
        model
            .points
            .get_mut("2")
            .unwrap()
            .loads_force
            .push(sap_model::PointLoadForce {
                load_pat: "DEAD".to_string(),
                csys: "GLOBAL".to_string(),
                force: [0.0, 0.0, -5.0, 0.0, 0.0, 0.0],
            });
        model.add_frame("1", "1", "2", "W14_TEST");
        save_s2k_file_with_report(&model, &input).unwrap();

        // Refuses overwriting input file
        let same_err =
            run_cli_analyze(&input, Some(&input), AnalyzeDesignOptions::default()).unwrap_err();
        assert!(same_err
            .to_string()
            .contains("Refusing to overwrite input file"));

        // `--design` evaluates steel and cleanly skips absent concrete discipline
        run_cli_analyze(
            &input,
            Some(&json_out),
            AnalyzeDesignOptions {
                design: true,
                steel_code: Some("AISC 360-10 ASD"),
                ..Default::default()
            },
        )
        .unwrap();
        let saved: sap_model::SapModel =
            serde_json::from_str(&std::fs::read_to_string(&json_out).unwrap()).unwrap();
        assert_eq!(saved.steel_design_code, "AISC 360-10 ASD");
        assert_eq!(saved.steel_design_results.len(), 1);
        assert!(saved.concrete_design_results.is_empty());

        // Explicit `--design-concrete` still fails closed on the unsupported concrete code
        let conc_err = run_cli_analyze(
            &input,
            None,
            AnalyzeDesignOptions {
                design_concrete: true,
                ..Default::default()
            },
        )
        .unwrap_err();
        assert!(
            conc_err.to_string().contains("LEGACY-CONCRETE-99"),
            "{conc_err}"
        );

        let _ = std::fs::remove_file(input);
        let _ = std::fs::remove_file(json_out);
    }

    #[test]
    fn cli_clap_parses_extended_analyze_and_inspect_pro_flags() {
        let cli = Cli::try_parse_from([
            "structurax",
            "analyze",
            "model.s2k",
            "--design-steel",
            "--steel-code",
            "Eurocode 3-2005",
            "-o",
            "out.json",
        ])
        .unwrap();
        match cli.command {
            Some(Commands::Analyze {
                design_steel,
                steel_code,
                ..
            }) => {
                assert!(design_steel);
                assert_eq!(steel_code.as_deref(), Some("Eurocode 3-2005"));
            }
            other => panic!("unexpected command: {other:?}"),
        }

        let cli_pro = Cli::try_parse_from([
            "structurax",
            "inspect-pro",
            "sections/aisc14.pro",
            "--filter",
            "W14",
            "--all",
        ])
        .unwrap();
        match cli_pro.command {
            Some(Commands::InspectPro { all, filter, .. }) => {
                assert!(all);
                assert_eq!(filter.as_deref(), Some("W14"));
            }
            other => panic!("unexpected command: {other:?}"),
        }
    }
}
