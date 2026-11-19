//! Opt-in vendor check of the shell moment sign: CSI Software Verification Example 2-015
//! (clamped circular plate, uniform downward load, kip and in), model B on the thin plate
//! option (`example_2_015b_thin.sdb`). Published (SAP2000 and the independent Ugural
//! solution): Uz at the center is -1.077 in (independent -1.069), and the bending moments at
//! the clamped edge are NEGATIVE (M11 at joint A = -81.54 k-in/in, independent -80.71): a
//! downward-loaded clamped plate hogs at its edge, and the CSI sign makes hogging negative
//! (positive moments give positive stress at the bottom face). That fixes the sign of
//! `AreaResult::m11` independently of the plate strip test. The published values are at
//! joints and ours are element-center values, so only the sign, the deflection and a lower
//! bound on the magnitude are asserted.
#![cfg(feature = "vendor-fixtures")]

use std::path::{Path, PathBuf};

use sap_format::load_sdb_file;
use sap_solver::analyze_model;

#[test]
fn vendor_example_2_015_edge_moment_is_negative_and_top_face_is_in_tension() {
    let dir = std::env::var_os("STRUCTURAX_VENDOR_FIXTURES")
        .map(PathBuf::from)
        .unwrap_or_else(|| Path::new(env!("CARGO_MANIFEST_DIR")).join("../../extracted/files"));
    let path = dir.join("example_2_015b_thin.sdb");
    if !path.exists() {
        return;
    }
    let mut model = load_sdb_file(&path).expect("import");
    analyze_model(&mut model).expect("analysis");
    let result = model.case_results.values().next().expect("case results");
    let uz = result
        .joint_displacements
        .values()
        .map(|d| d[2])
        .fold(f64::MAX, f64::min);
    assert!((uz - (-1.077)).abs() <= 0.01 * 1.077, "center Uz {uz}");
    let (name, edge) = result
        .area_results
        .iter()
        .min_by(|a, b| a.1.m11.partial_cmp(&b.1.m11).unwrap())
        .unwrap();
    assert!(
        edge.m11 < -50.0 && edge.m11 > -81.54,
        "most negative M11 {} in {name}",
        edge.m11
    );
    // Hogging: the top face is in tension (section thickness from the model).
    let area = &model.areas[name];
    let t = model.area_sections[&area.section].thickness_bending;
    let top = edge.f11 / model.area_sections[&area.section].thickness_membrane
        - 6.0 * edge.m11 / (t * t);
    assert!((edge.s11_top - top).abs() <= 1e-9 * top.abs());
    assert!(edge.s11_top > 0.0, "top-face S11 {}", edge.s11_top);
}
