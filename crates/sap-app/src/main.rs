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
        /// Run AISC 360 / Eurocode 3 steel design and ACI 318 / Eurocode 2 concrete design
        #[arg(long)]
        design: bool,
    },
    /// Inspect a binary .pro structural section property database
    InspectPro {
        /// Path to the .pro file (e.g., sections/aisc14.pro)
        pro_file: PathBuf,
        /// Optional section name to inspect in detail (e.g., W14X90)
        section: Option<String>,
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
        /// Destination binary .sdb file path
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
        }) => run_cli_analyze(&input, output.as_deref(), design),
        Some(Commands::InspectPro { pro_file, section }) => {
            run_cli_inspect_pro(&pro_file, section.as_deref())
        }
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

fn run_cli_analyze(
    input: &std::path::Path,
    output: Option<&std::path::Path>,
    design: bool,
) -> Result<()> {
    let report = load_model_with_report(input)?;
    for warning in &report.warnings {
        eprintln!("Import warning: {warning}");
    }
    let mut model = report.model;
    println!(
        "Loaded model '{}' (Units: {}): {} joints, {} frames, {} areas, {} load cases.",
        model.model_name,
        model.present_units.s2k_label(),
        model.points.len(),
        model.frames.len(),
        model.areas.len(),
        model.load_cases.len()
    );
    analyze_model(&mut model)?;
    for (cname, cr) in &model.case_results {
        println!(
            "  Case '{}': {} joint displacements, {} support reactions, {} modes",
            cname,
            cr.joint_displacements.len(),
            cr.joint_reactions.len(),
            cr.modes.len()
        );
    }
    if design {
        try_run_steel_design(&mut model)?;
        try_run_concrete_design(&mut model)?;
        println!(
            "  Design complete: {} steel members, {} concrete members evaluated.",
            model.steel_design_results.len(),
            model.concrete_design_results.len()
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
            for warning in save_sdb_file_with_report(&model, out_path)? { eprintln!("Output warning: {warning}"); }
        } else {
            for warning in save_s2k_file_with_report(&model, out_path)? { eprintln!("Output warning: {warning}"); }
        }
        println!("Saved output to {}", out_path.display());
    }
    Ok(())
}

fn run_cli_inspect_pro(pro_file: &std::path::Path, section: Option<&str>) -> Result<()> {
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
        for s in db.sections.iter().take(50) {
            println!("  {}", s.name);
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
        "Exported binary stream SDB to {} ({} stream bytes, 100% schema verified).",
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
        let model = sap_model::SapModel::new(sap_units::Units::KipInF);
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
        let model = sap_model::SapModel::new(sap_units::Units::KipInF);
        save_native_sdb_file(&model, &input).unwrap();

        run_cli_export_native_sdb(&input, &output).unwrap();
        let doc = inspect_native_sdb_file(&output).unwrap();
        assert_eq!(doc.inventory().parsed_bytes, doc.inventory().stream_bytes);
        std::fs::remove_file(output).unwrap();
        std::fs::remove_file(input).unwrap();
    }
}
