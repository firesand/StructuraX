//! SAP2000 file format parser and writer (`.s2k` / `.$2k` text tables and `.sdb` binary model container).

pub mod s2k;
pub mod sdb;

use sap_model::SapModel;
use std::path::Path;
use thiserror::Error;

pub use s2k::{parse_s2k, parse_s2k_with_report, try_write_s2k_with_report, write_s2k};
pub use sdb::native::{
    inspect_native_sdb, inspect_native_sdb_bytes, inspect_native_sdb_file,
    pack_model_to_native_sdb, save_native_sdb_file, NativeFieldSpan, NativeRecord,
    NativeSdbDocument, NativeSdbInventory, NativeTopLevelField,
};
pub use sdb::{
    extract_sdb_to_model, pack_model_to_sdb, read_sdb_bytes, read_sdb_file, write_sdb_bytes,
    write_sdb_file, SdbFile,
};

/// Import result with compatibility diagnostics. A successful import does not imply that
/// every vendor table/column is represented by the reconstruction's model.
#[derive(Debug)]
pub struct ImportReport {
    pub model: SapModel,
    pub warnings: Vec<String>,
}

/// Checked text export plus preservation/compatibility diagnostics.
#[derive(Debug)]
pub struct ExportReport {
    pub text: String,
    pub warnings: Vec<String>,
}

#[derive(Debug, Error)]
pub enum FormatError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
    #[error("Unit conversion error: {0}")]
    Unit(#[from] sap_units::UnitError),
    #[error("S2K parse error: {0}")]
    ParseS2k(String),
    #[error("S2K preservation conflict: {0}")]
    PreservationConflict(String),
    #[error("Invalid SDB file: {0}")]
    InvalidSdb(String),
    #[error("Native case '{case_name}' cannot be imported safely: {reason}")]
    UnsupportedNativeCase { case_name: String, reason: String },
}

/// Load a `SapModel` from a `.s2k` or `.$2k` text file.
pub fn load_s2k_file(path: impl AsRef<Path>) -> Result<SapModel, FormatError> {
    let text = std::fs::read_to_string(path)?;
    parse_s2k(&text)
}

/// Save a `SapModel` to a `.s2k` or `.$2k` text file.
pub fn save_s2k_file(model: &SapModel, path: impl AsRef<Path>) -> Result<(), FormatError> {
    save_s2k_file_with_report(model, path).map(|_| ())
}

/// Save checked text and return diagnostics without computing the projection twice.
pub fn save_s2k_file_with_report(
    model: &SapModel,
    path: impl AsRef<Path>,
) -> Result<Vec<String>, FormatError> {
    let report = try_write_s2k_with_report(model)?;
    std::fs::write(path, report.text)?;
    Ok(report.warnings)
}

/// Load a `SapModel` from a SAP2000 `.sdb` binary file.
pub fn load_sdb_file(path: impl AsRef<Path>) -> Result<SapModel, FormatError> {
    let sdb = read_sdb_file(path)?;
    extract_sdb_to_model(&sdb)
}

/// Save a `SapModel` to a SAP2000 `.sdb` binary file.
pub fn save_sdb_file(model: &SapModel, path: impl AsRef<Path>) -> Result<(), FormatError> {
    save_sdb_file_with_report(model, path).map(|_| ())
}

/// Save a reconstruction private snapshot, returning text-projection warnings.
/// This is not native vendor SDB serialization.
pub fn save_sdb_file_with_report(
    model: &SapModel,
    path: impl AsRef<Path>,
) -> Result<Vec<String>, FormatError> {
    let report = try_write_s2k_with_report(model)?;
    let sdb = sdb::pack_projected_text(model, &report.text);
    write_sdb_file(path, &sdb)?;
    Ok(report.warnings)
}

/// Automatically detect whether `path` is a `.sdb` binary file or `.s2k`/`.$2k` text file and load it.
pub fn load_model_auto(path: impl AsRef<Path>) -> Result<SapModel, FormatError> {
    Ok(load_model_with_report(path)?.model)
}

/// Detect the input container and load its supported model data with explicit diagnostics.
pub fn load_model_with_report(path: impl AsRef<Path>) -> Result<ImportReport, FormatError> {
    let bytes = std::fs::read(path)?;
    if bytes.starts_with(b"SAP2000") {
        let sdb = read_sdb_bytes(&bytes)?;
        if let Some(text) = sdb
            .decompressed_stream
            .strip_prefix(sdb::RUST_SDB_S2K_MAGIC)
        {
            return parse_s2k_with_report(std::str::from_utf8(text).map_err(|e| {
                FormatError::InvalidSdb(format!("Invalid UTF-8 text model stream: {}", e))
            })?);
        }
        let model = extract_sdb_to_model(&sdb)?;
        let mut warnings = vec!["Native SDB import reads a supported subset. Grid definitions, project metadata, generated/external dynamic-function sources, nonlinear/coupled link parameters, patterned solid loads, and many advanced assignments are omitted; saving cannot preserve the full vendor model.".into()];
        for prop in model
            .link_props
            .values()
            .filter(|p| !p.nonlinear_data_complete)
        {
            warnings.push(format!("Native link '{}' has verified activation flags but unmapped constitutive/opening data; nonlinear analysis is blocked", prop.name));
        }
        return Ok(ImportReport { model, warnings });
    }
    let text = std::str::from_utf8(&bytes)
        .map_err(|e| FormatError::ParseS2k(format!("Text model is not valid UTF-8: {}", e)))?;
    parse_s2k_with_report(text)
}

#[cfg(test)]
mod tests {
    use super::*;
    use sap_model::*;
    use sap_units::Units;

    #[test]
    fn test_s2k_roundtrip() {
        let mut model = SapModel::new(Units::KipInF);
        model.active_dofs = [true, false, true, false, true, false];

        let mut r_sec = FrameSection::default();
        r_sec.name = "R12X24".into();
        r_sec.prop_type = FramePropType::Rectangular;
        r_sec.t3 = 24.0;
        r_sec.t2 = 12.0;
        r_sec.area = 288.0;
        r_sec.i33 = 12.0 * 24.0_f64.powi(3) / 12.0;
        r_sec.i22 = 24.0 * 12.0_f64.powi(3) / 12.0;
        model.frame_sections.insert("R12X24".into(), r_sec);

        let mut p1 = PointObj::new("1", 0.0, 0.0, 0.0);
        p1.restraint = [true; 6];
        let mut p2 = PointObj::new("2", 120.0, 0.0, 0.0);
        p2.restraint = [true, true, true, false, false, false];
        p2.spring = [100.0, 0.0, 200.0, 0.0, 500.0, 0.0];
        let mut p3 = PointObj::new("3", 120.0, 0.0, 144.0);
        p3.loads_force.push(PointLoadForce {
            load_pat: "LIVE".into(),
            csys: "GLOBAL".into(),
            force: [10.0, 0.0, -25.0, 0.0, 120.0, 0.0],
        });
        model.points.insert("1".into(), p1);
        model.points.insert("2".into(), p2);
        model.points.insert("3".into(), p3);

        let mut f1 = FrameObj::new("F1", "1", "2", "R12X24");
        f1.release_i = [false, false, false, false, false, true];
        f1.end_offset_i = 6.0;
        f1.end_offset_j = 6.0;
        f1.rigid_zone_factor = 0.5;
        f1.loads_dist.push(FrameLoadDistributed {
            load_pat: "DEAD".into(),
            my_type: 1,
            dir: 10,
            csys: "GLOBAL".into(),
            rel_dist1: 0.0,
            rel_dist2: 1.0,
            val1: 0.15,
            val2: 0.15,
        });
        f1.loads_point.push(FrameLoadPoint {
            load_pat: "LIVE".into(),
            my_type: 1,
            dir: 10,
            csys: "GLOBAL".into(),
            rel_dist: 0.5,
            val: 15.0,
        });
        model.frames.insert("F1".into(), f1);

        let a1 = AreaObj::new("A1", vec!["1".into(), "2".into(), "3".into()], "ASEC1");
        model.areas.insert("A1".into(), a1);

        model.load_patterns.insert(
            "LIVE".into(),
            LoadPattern {
                name: "LIVE".into(),
                pat_type: LoadPatternType::Live,
                self_wt_multiplier: 0.0,
            },
        );
        model
            .load_cases
            .insert("LIVE".into(), LoadCase::linear_static("LIVE", "LIVE", 1.0));
        model.load_combos.insert(
            "1.2D+1.6L".into(),
            LoadCombo {
                name: "1.2D+1.6L".into(),
                combo_type: ComboType::LinearAdd,
                cases: vec![("DEAD".into(), 1.2), ("LIVE".into(), 1.6)],
            },
        );

        let s2k_text = write_s2k(&model);
        assert!(s2k_text.contains("TABLE:  \"PROGRAM CONTROL\""));
        assert!(s2k_text.contains("TABLE:  \"JOINT COORDINATES\""));
        assert!(s2k_text.contains("END TABLE DATA"));

        let parsed = parse_s2k(&s2k_text).expect("Failed to parse generated S2K");
        assert_eq!(model, parsed);
    }

    #[test]
    fn test_parse_s2k_rejects_garbage() {
        assert!(parse_s2k("").is_err());
        assert!(parse_s2k("   \n  \t ").is_err());
        assert!(parse_s2k("not a valid s2k file at all").is_err());
        assert!(parse_s2k(
            "TABLE:  \"JOINT COORDINATES\"\n   garbage_without_equals\nEND TABLE DATA"
        )
        .is_err());
    }

    #[test]
    fn absolute_load_distances_and_assignments_are_independent_of_table_order() {
        let text = r#"TABLE: "FRAME LOADS - POINT"
 Frame=F LoadPat=DEAD DistType=AbsDist AbsDist=2 Force=7
TABLE: "FRAME LOADS - DISTRIBUTED"
 Frame=F LoadPat=DEAD DistType=AbsDist AbsDistA=2.5 AbsDistB=7.5 FOverLA=3 FOverLB=4
TABLE: "FRAME SECTION ASSIGNMENTS"
 Frame=F AnalSect=R
TABLE: "JOINT RESTRAINT ASSIGNMENTS"
 Joint=I U1=Yes U2=Yes U3=Yes
TABLE: "CONNECTIVITY - FRAME"
 Frame=F JointI=I JointJ=J
TABLE: "JOINT COORDINATES"
 Joint=I XorR=0 Y=0 Z=0
 Joint=J XorR=6 Y=8 Z=0
TABLE: "FRAME SECTION PROPERTIES 01 - GENERAL"
 SectionName=R Shape=Rectangular t3=2 t2=3
END TABLE DATA"#;
        let model = parse_s2k(text).unwrap();
        assert_eq!(model.frames["F"].section, "R");
        assert!(model.points["I"].restraint[0]);
        assert_eq!(model.frames["F"].loads_point[0].rel_dist, 0.2);
        assert_eq!(model.frames["F"].loads_dist[0].rel_dist1, 0.25);
        assert_eq!(model.frames["F"].loads_dist[0].rel_dist2, 0.75);
        assert_eq!(model, parse_s2k(&write_s2k(&model)).unwrap());
        assert!(parse_s2k(&text.replace("AbsDist=2", "AbsDist=12")).is_err());
        assert!(parse_s2k(&text.replace("AbsDist=2", "AbsDist=NaN")).is_err());
        assert!(parse_s2k(&text.replace("DistType=AbsDist", "DistType=Guess")).is_err());
    }

    #[test]
    fn quoted_identifiers_preserve_quotes_tabs_and_unicode() {
        let mut model = SapModel::new(Units::KnMC);
        model.model_name = "A \"quoted\" model".into();
        let name = "J\t\"α\"";
        model
            .points
            .insert(name.into(), PointObj::new(name, 1.0, 2.0, 3.0));
        assert_eq!(model, parse_s2k(&write_s2k(&model)).unwrap());
        for row in [
            "Joint=\"unterminated",
            "Joint=\"J\"trailing",
            "Joint=I Joint=J",
            "Joint=I broken",
        ] {
            assert!(
                parse_s2k(&format!("TABLE: \"JOINT COORDINATES\"\n{}", row)).is_err(),
                "{}",
                row
            );
        }
    }

    #[test]
    fn units_and_coordinate_formats_cannot_silently_change_geometry() {
        let input = "\u{feff}TaBlE: \"PROGRAM CONTROL\"\n CurrUnits=\"kN, m, C\"\nTaBlE: \"JOINT COORDINATES\"\n Joint=J XorR=2 Y=3 Z=4";
        assert_eq!(parse_s2k(input).unwrap().present_units, Units::KnMC);
        assert!(parse_s2k(&input.replace("kN, m, C", "bad-units")).is_err());
        assert!(parse_s2k(&input.replace("XorR=2", "CoordType=Cylindrical XorR=2")).is_err());
        assert!(parse_s2k(&input.replace("XorR=2", "CoordSys=ROTATED XorR=2")).is_err());
        let global = input.replace(
            "XorR=2",
            "CoordSys=ROTATED XorR=2 GlobalX=8 GlobalY=9 GlobalZ=10",
        );
        assert_eq!(parse_s2k(&global).unwrap().points["J"].x, 8.0);
        assert!(parse_s2k(&input.replace("XorR=2", "XorR=oops")).is_err());
        assert!(parse_s2k(&input.replace("XorR=2", "XorR=NaN")).is_err());
        assert!(parse_s2k("TABLE: \"JOINT RESTRAINT ASSIGNMENTS\"\n Joint=J U1=Maybe").is_err());
    }

    #[test]
    fn import_report_names_actual_unsupported_tables() {
        let report = parse_s2k_with_report("TABLE: \"UNSUPPORTED LOADS\"\n Joint=1 Load=20\n Joint=2 Load=30\nTABLE: \"PROGRAM CONTROL\"\n CurrUnits=\"Kip, in, F\"").unwrap();
        assert_eq!(
            report
                .warnings
                .iter()
                .filter(|w| w.contains("UNSUPPORTED LOADS"))
                .count(),
            1
        );
        assert!(report
            .warnings
            .iter()
            .any(|w| w.contains("preserved verbatim")));
    }

    #[test]
    fn wilson_theta_parameter_roundtrips_through_text_and_private_snapshot() {
        let mut model = SapModel::new(Units::KnMC);
        model.load_cases.get_mut("DEAD").unwrap().wilson_theta = 1.7;
        assert_eq!(model, parse_s2k(&write_s2k(&model)).unwrap());
        assert_eq!(
            model,
            extract_sdb_to_model(&pack_model_to_sdb(&model).unwrap()).unwrap()
        );
    }

    #[test]
    fn test_corrupt_sdb_bounds_and_depth_protection() {
        // Too short header
        assert!(read_sdb_bytes(b"SAP2000").is_err());

        // Huge PNG length should fail cleanly without OOM
        let mut fake = vec![0u8; 64];
        fake[..7].copy_from_slice(b"SAP2000");
        fake[0x30..0x34].copy_from_slice(&0x7FFF_FFFFu32.to_le_bytes());
        assert!(read_sdb_bytes(&fake).is_err());
    }

    #[test]
    fn test_sdb_real_example_1_001_and_1_002a() {
        let path_001 = Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../../extracted/files/example_1_001.sdb"));
        if path_001.exists() {
            let sdb = read_sdb_file(path_001).expect("read example_1_001.sdb");
            assert!(sdb.program_name.starts_with("SAP2000"));
            assert!(!sdb.thumbnail_png.is_empty());
            let model = extract_sdb_to_model(&sdb).expect("extract example_1_001.sdb to SapModel");
            assert_eq!(model.present_units, Units::KipInF);
            assert_eq!(model.points.len(), 4);
            assert_eq!(model.frames.len(), 3);
            let restrained_count = model
                .points
                .values()
                .filter(|p| p.restraint.iter().any(|&r| r))
                .count();
            assert_eq!(restrained_count, 2);
            assert!(model.frame_sections.contains_key("FSEC1"));
            let fsec1 = &model.frame_sections["FSEC1"];
            assert!((fsec1.area - 144.0).abs() < 1e-6);
            assert!((fsec1.i33 - 1728.0).abs() < 1e-6);
            assert_eq!(model.load_patterns.len(), 7);
            assert_eq!(model.load_cases.len(), 7);
            let total_jf: usize = model.points.values().map(|p| p.loads_force.len()).sum();
            let total_fd: usize = model.frames.values().map(|f| f.loads_dist.len()).sum();
            let total_fc: usize = model.frames.values().map(|f| f.loads_point.len()).sum();
            assert_eq!(total_jf, 2);
            assert_eq!(total_fd, 6);
            assert_eq!(total_fc, 1);

            // Verify lossless round-trip on real vendor model
            let s2k = write_s2k(&model);
            let reloaded = parse_s2k(&s2k).expect("parse_s2k on example_1_001");
            assert_eq!(model, reloaded);
        }

        let path_002a = Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../../extracted/files/example_1_002a.sdb"));
        if path_002a.exists() {
            let model = load_sdb_file(path_002a).expect("load example_1_002a.sdb");
            assert_eq!(model.points.len(), 2);
            assert_eq!(model.frames.len(), 1);
            assert!((model.points["2"].x - 10.0).abs() < 1e-9);
            let s2k = write_s2k(&model);
            let reloaded = parse_s2k(&s2k).expect("parse_s2k on example_1_002a");
            assert_eq!(model, reloaded);
        }
    }

    #[test]
    fn test_sdb_roundtrip_and_vendor_combos() {
        let mut model = SapModel::new(Units::KipInF);
        let mut p1 = PointObj::new("1", 0.0, 0.0, 0.0);
        p1.restraint = [true; 6];
        let p2 = PointObj::new("2", 144.0, 0.0, 0.0);
        model.points.insert("1".into(), p1);
        model.points.insert("2".into(), p2);
        model
            .frames
            .insert("1".into(), FrameObj::new("1", "1", "2", "FSEC1"));

        let sdb = pack_model_to_sdb(&model).expect("pack model to sdb");
        let bytes = write_sdb_bytes(&sdb).expect("write sdb bytes");
        assert!(bytes.starts_with(b"SAP2000"));

        let sdb_back = read_sdb_bytes(&bytes).expect("read sdb bytes back");
        let model_back = extract_sdb_to_model(&sdb_back).expect("extract model back");
        assert_eq!(model, model_back);

        // Check vendor SDB files with combinations and verify lossless round-tripping
        let dir = Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../../extracted/files"));
        if dir.exists() {
            let mut checked = 0;
            let mut rejected_unsupported_cases = 0;
            let mut non_empty_combos = 0;
            if let Ok(entries) = std::fs::read_dir(dir) {
                for entry in entries.flatten() {
                    let p = entry.path();
                    if p.extension().and_then(|s| s.to_str()) == Some("sdb") {
                        {
                            let m1 = match load_sdb_file(&p) {
                                Ok(model) => model,
                                Err(FormatError::UnsupportedNativeCase { .. }) => {
                                    rejected_unsupported_cases += 1;
                                    continue;
                                }
                                Err(e) => {
                                    panic!("vendor import failed for {:?}: {}", p.file_name(), e)
                                }
                            };
                            for cb in m1.load_combos.values() {
                                if !cb.cases.is_empty() {
                                    non_empty_combos += 1;
                                }
                            }
                            let s2k = write_s2k(&m1);
                            let m2 = parse_s2k(&s2k).unwrap_or_else(|e| {
                                panic!("parse_s2k failed for {:?}: {}", p.file_name(), e)
                            });
                            assert_eq!(m1, m2, "round-trip mismatch for {:?}", p.file_name());
                            checked += 1;
                        }
                    }
                }
            }
            assert!(
                checked >= 300,
                "Expected at least 300 vendor SDBs, got {}",
                checked
            );
            assert!(checked + rejected_unsupported_cases >= 336);
            eprintln!("Native model corpus: {} safe imports/roundtrips, {} explicit unsupported-case rejections", checked, rejected_unsupported_cases);
            assert!(
                non_empty_combos > 0,
                "Expected vendor SDBs to have populated load combinations"
            );
        }
    }
}
