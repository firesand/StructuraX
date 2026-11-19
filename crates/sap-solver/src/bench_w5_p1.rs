//! Wave 5 verification rows (package P1).

use crate::benchmarks::BenchmarkVerificationRow;
use sap_model::{
    FrameSection, LoadCase, LoadPattern, LoadPatternType, Material, PointLoadForce, SapModel,
};
use sap_units::Units;

/// A bar of length 144 in between joints "1" (fixed) and "2" (free), E 4320 ksi,
/// G 1800 ksi, A 216 in2, J 1000 in4, with rigid zones of 6 in at both ends
/// and the rigid-zone factor 1, loaded at joint 2 by 10 kip along X (pattern
/// AX) and by 100 kip-in about X (pattern TQ).
fn rigid_zone_bar() -> SapModel {
    let mut model = SapModel::new(Units::KipInF);
    model.load_patterns.clear();
    model.load_cases.clear();
    let mut mat = Material::default_concrete_kip_in();
    mat.name = "CONC".to_string();
    mat.e = 4_320.0;
    mat.u = 0.2;
    mat.g = 1_800.0;
    model.materials.insert("CONC".to_string(), mat);
    let section = FrameSection {
        name: "BAR".to_string(),
        material: "CONC".to_string(),
        area: 216.0,
        i22: 1_000.0,
        i33: 1_000.0,
        torsional_j: 1_000.0,
        as2: 0.0,
        as3: 0.0,
        ..FrameSection::default()
    };
    model.frame_sections.insert("BAR".to_string(), section);
    model.add_point("1", 0.0, 0.0, 0.0);
    model.points.get_mut("1").unwrap().restraint = [true; 6];
    model.add_point("2", 144.0, 0.0, 0.0);
    model.add_frame("1", "1", "2", "BAR");
    let frame = model.frames.get_mut("1").unwrap();
    frame.end_offset_i = 6.0;
    frame.end_offset_j = 6.0;
    frame.rigid_zone_factor = 1.0;
    for (pattern, force) in [
        ("AX", [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ("TQ", [0.0, 0.0, 0.0, 100.0, 0.0, 0.0]),
    ] {
        model.load_patterns.insert(
            pattern.to_string(),
            LoadPattern {
                name: pattern.to_string(),
                pat_type: LoadPatternType::Other,
                self_wt_multiplier: 0.0,
            },
        );
        model
            .load_cases
            .insert(pattern.to_string(), LoadCase::linear_static(pattern, pattern, 1.0));
        model
            .points
            .get_mut("2")
            .unwrap()
            .loads_force
            .push(PointLoadForce {
                load_pat: pattern.to_string(),
                csys: "GLOBAL".to_string(),
                force,
            });
    }
    model
}

/// Axial and torsional stiffness of a frame with rigid end zones use the full
/// length (Analysis Reference p. 130): delta = P L / (E A), theta = T L / (G J).
fn verify_rigid_zone_axial_and_torsion() -> Vec<BenchmarkVerificationRow> {
    let mut model = rigid_zone_bar();
    // An analysis error must fail the suite, not drop the rows.
    crate::benchmarks::analyze_model(&mut model).expect("rigid-zone benchmark bar must analyse");
    let tip = |case: &str, dof: usize| model.case_results[case].joint_displacements["2"][dof];
    vec![
        BenchmarkVerificationRow::new(
            "Axial bar with rigid end zones (p. 130): delta = PL/EA",
            "AX",
            "U1 (Jt 2) [in]",
            10.0 * 144.0 / (4_320.0 * 216.0),
            tip("AX", 0),
            1e-8,
        ),
        BenchmarkVerificationRow::new(
            "Torsion with rigid end zones: theta = TL/GJ",
            "TQ",
            "R1 (Jt 2) [rad]",
            100.0 * 144.0 / (1_800.0 * 1_000.0),
            tip("TQ", 3),
            1e-8,
        ),
    ]
}

pub(crate) fn rows() -> Vec<BenchmarkVerificationRow> {
    let mut rows = Vec::new();
    rows.extend(verify_rigid_zone_axial_and_torsion());
    rows
}
