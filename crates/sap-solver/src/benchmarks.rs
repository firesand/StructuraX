//! Official SAP2000 v19.0.0 Software Verification Suite (`sap2000_verification.xls`).
//!
//! Provides both unit tests (`#[cfg(test)]`) and a public `run_verification_suite()` runner
//! used by the CLI (`sap2000-rs verify`) and GUI (`Help -> Verification Suite`) to compare
//! `sap2000-rs` against CSI SAP2000 v19 reference benchmarks.

use crate::analyze_model;
use sap_model::{
    FrameLoadDistributed, FrameLoadPoint, FrameLoadTemperature, FrameSection, LoadCase,
    LoadCaseType, LoadPattern, LoadPatternType, Material, NLGeomType, PointLoadForce, SapModel,
};
use sap_section::make_rectangular;
use sap_units::Units;
use serde::{Deserialize, Serialize};

/// A single verification comparison row between SAP2000 v19 reference and `sap2000-rs`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BenchmarkVerificationRow {
    pub example: String,
    pub load_case: String,
    pub parameter: String,
    pub sap2000_ref: f64,
    pub computed: f64,
    pub pct_diff: f64,
    pub passed: bool,
}

impl BenchmarkVerificationRow {
    pub fn new(
        example: &str,
        load_case: &str,
        parameter: &str,
        sap2000_ref: f64,
        computed: f64,
        tol_pct: f64,
    ) -> Self {
        let pct_diff = if sap2000_ref.abs() > 1e-9 {
            ((computed - sap2000_ref) / sap2000_ref).abs() * 100.0
        } else {
            (computed - sap2000_ref).abs() * 100.0
        };
        Self {
            example: example.to_string(),
            load_case: load_case.to_string(),
            parameter: parameter.to_string(),
            sap2000_ref,
            computed,
            pct_diff,
            passed: pct_diff <= tol_pct,
        }
    }
}

/// Runs the official SAP2000 verification suite and returns all comparison rows.
pub fn run_verification_suite() -> Vec<BenchmarkVerificationRow> {
    let mut rows = Vec::new();
    rows.extend(verify_example_1_001());
    rows.extend(verify_example_1_002());
    rows.extend(verify_example_1_003());
    rows.extend(verify_example_1_004());
    rows.extend(verify_example_1_007());
    rows.extend(verify_example_1_008());
    rows.extend(verify_example_1_010());
    rows.extend(verify_example_1_016());
    rows.extend(verify_frame_auto_subdivision());
    rows.extend(verify_example_1_011());
    rows.extend(verify_example_5_001());
    rows.extend(verify_example_6_003());
    rows.extend(verify_example_6_005());
    rows.extend(verify_example_6_008());
    rows.extend(verify_example_7_001());
    rows.extend(verify_example_7_002());
    rows.extend(verify_example_7_003());
    rows
}

/// Example 1-001: 2D Frame Under General Loading (All 7 Load Cases).
pub fn verify_example_1_001() -> Vec<BenchmarkVerificationRow> {
    let mut model = SapModel::new(Units::KipInF);
    model.load_patterns.clear();
    model.load_cases.clear();

    let mut mat = Material::default_concrete_kip_in();
    mat.name = "CONC".to_string();
    mat.e = 3600.0;
    mat.u = 0.2;
    mat.g = 3600.0 / 2.4;
    mat.unit_weight = 0.15 / 1728.0;
    model.materials.insert("CONC".to_string(), mat);

    let mut sec = make_rectangular("FSEC1", "CONC", 12.0, 12.0);
    sec.area = 144.0;
    sec.i33 = 1728.0;
    sec.as2 = 0.0;
    sec.as3 = 0.0;
    sec.modifiers[0] = 1000.0;
    model.frame_sections.insert("FSEC1".to_string(), sec);

    model.add_point("1", 0.0, 0.0, 0.0);
    model.points.get_mut("1").unwrap().restraint = [true, true, true, true, false, true];

    model.add_point("2", 0.0, 0.0, 120.0);
    model.points.get_mut("2").unwrap().restraint = [false, true, false, true, false, true];

    model.add_point("3", 96.0, 0.0, 192.0);
    model.points.get_mut("3").unwrap().restraint = [true, true, false, true, false, true];

    model.add_point("4", -48.0, 0.0, 120.0);
    model.points.get_mut("4").unwrap().restraint = [false, true, false, true, false, true];

    model.add_frame("1", "1", "2", "FSEC1");
    model.add_frame("2", "2", "3", "FSEC1");
    model.add_frame("3", "4", "2", "FSEC1");

    for i in 1..=7 {
        let lp_name = format!("LCASE{i}");
        model.load_patterns.insert(
            lp_name.clone(),
            LoadPattern {
                name: lp_name.clone(),
                pat_type: LoadPatternType::Other,
                self_wt_multiplier: if i == 1 { 1.0 } else { 0.0 },
            },
        );
        model.load_cases.insert(
            lp_name.clone(),
            LoadCase::linear_static(&lp_name, &lp_name, 1.0),
        );
    }

    // LC2: Global -Z uniform load of 1.8 kip/ft on Frame 3 + -10 kip FZ at Joint 4
    model
        .frames
        .get_mut("3")
        .unwrap()
        .loads_dist
        .push(FrameLoadDistributed {
            load_pat: "LCASE2".to_string(),
            my_type: 1,
            dir: 6,
            csys: "GLOBAL".to_string(),
            rel_dist1: 0.0,
            rel_dist2: 1.0,
            val1: -1.8 / 12.0,
            val2: -1.8 / 12.0,
        });
    model
        .points
        .get_mut("4")
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: "LCASE2".to_string(),
            csys: "GLOBAL".to_string(),
            force: [0.0, 0.0, -10.0, 0.0, 0.0, 0.0],
        });

    // LC3: Statically equivalent joint force at Joint 2
    model
        .points
        .get_mut("2")
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: "LCASE3".to_string(),
            csys: "GLOBAL".to_string(),
            force: [0.0, 0.0, -17.2, 0.0, -54.4 * 12.0, 0.0],
        });

    // LC4: Global -Z projected uniform load of 2.0 kip/ft on Frame 2
    model
        .frames
        .get_mut("2")
        .unwrap()
        .loads_dist
        .push(FrameLoadDistributed {
            load_pat: "LCASE4".to_string(),
            my_type: 1,
            dir: 9,
            csys: "GLOBAL".to_string(),
            rel_dist1: 0.0,
            rel_dist2: 1.0,
            val1: -2.0 / 12.0,
            val2: -2.0 / 12.0,
        });

    // LC5: Local 2 uniform load of 2.0 kip/ft on Frames 1 and 2
    model
        .frames
        .get_mut("1")
        .unwrap()
        .loads_dist
        .push(FrameLoadDistributed {
            load_pat: "LCASE5".to_string(),
            my_type: 1,
            dir: 2,
            csys: "LOCAL".to_string(),
            rel_dist1: 0.0,
            rel_dist2: 1.0,
            val1: 2.0 / 12.0,
            val2: 2.0 / 12.0,
        });
    model
        .frames
        .get_mut("2")
        .unwrap()
        .loads_dist
        .push(FrameLoadDistributed {
            load_pat: "LCASE5".to_string(),
            my_type: 1,
            dir: 2,
            csys: "LOCAL".to_string(),
            rel_dist1: 0.0,
            rel_dist2: 1.0,
            val1: -2.0 / 12.0,
            val2: -2.0 / 12.0,
        });

    // LC6: Trapezoidal load in Local 2 direction on Frames 1 and 2
    model
        .frames
        .get_mut("1")
        .unwrap()
        .loads_dist
        .push(FrameLoadDistributed {
            load_pat: "LCASE6".to_string(),
            my_type: 1,
            dir: 2,
            csys: "LOCAL".to_string(),
            rel_dist1: 0.0,
            rel_dist2: 1.0,
            val1: 0.9984 / 12.0,
            val2: 0.3744 / 12.0,
        });
    model
        .frames
        .get_mut("2")
        .unwrap()
        .loads_dist
        .push(FrameLoadDistributed {
            load_pat: "LCASE6".to_string(),
            my_type: 1,
            dir: 2,
            csys: "LOCAL".to_string(),
            rel_dist1: 0.0,
            rel_dist2: 1.0,
            val1: -0.3744 / 12.0,
            val2: 0.0,
        });

    // LC7: Concentrated midspan load of 15 kip in Local -2 direction on Frame 2
    model
        .frames
        .get_mut("2")
        .unwrap()
        .loads_point
        .push(FrameLoadPoint {
            load_pat: "LCASE7".to_string(),
            my_type: 1,
            dir: 2,
            csys: "LOCAL".to_string(),
            rel_dist: 0.5,
            val: -15.0,
        });

    analyze_model(&mut model).expect("Example 1-001 analysis failed");

    vec![
        BenchmarkVerificationRow::new(
            "Ex 1-001",
            "LCASE1 (Self-Weight)",
            "Uz (Jt 3) [in]",
            -0.02639,
            model.case_results["LCASE1"].joint_displacements["3"][2],
            0.2,
        ),
        BenchmarkVerificationRow::new(
            "Ex 1-001",
            "LCASE2 (Cantilever)",
            "Uz (Jt 3) [in]",
            0.06296,
            model.case_results["LCASE2"].joint_displacements["3"][2],
            0.2,
        ),
        BenchmarkVerificationRow::new(
            "Ex 1-001",
            "LCASE3 (Equiv Joint)",
            "Uz (Jt 3) [in]",
            0.06296,
            model.case_results["LCASE3"].joint_displacements["3"][2],
            0.2,
        ),
        BenchmarkVerificationRow::new(
            "Ex 1-001",
            "LCASE4 (Projected Z)",
            "Uz (Jt 3) [in]",
            -0.29630,
            model.case_results["LCASE4"].joint_displacements["3"][2],
            0.2,
        ),
        BenchmarkVerificationRow::new(
            "Ex 1-001",
            "LCASE5 (Local Unif)",
            "Ux (Jt 2) [in]",
            0.31250,
            model.case_results["LCASE5"].joint_displacements["2"][0],
            0.2,
        ),
        BenchmarkVerificationRow::new(
            "Ex 1-001",
            "LCASE6 (Trapezoidal)",
            "Ux (Jt 2) [in]",
            0.11556,
            model.case_results["LCASE6"].joint_displacements["2"][0],
            0.2,
        ),
        BenchmarkVerificationRow::new(
            "Ex 1-001",
            "LCASE7 (Point Load)",
            "Ux (Jt 2) [in]",
            0.00651,
            model.case_results["LCASE7"].joint_displacements["2"][0],
            0.2,
        ),
    ]
}

/// Example 1-002a & 1-002b: Frame Temperature Loading.
pub fn verify_example_1_002() -> Vec<BenchmarkVerificationRow> {
    let mut rows = Vec::new();
    for is_propped in [false, true] {
        let mut model = SapModel::new(Units::KipInF);
        model.load_patterns.clear();
        model.load_cases.clear();

        let mut mat = Material::default_steel_kip_in();
        mat.name = "STEEL".to_string();
        mat.e = 29_000.0;
        mat.a = 6.5e-6;
        model.materials.insert("STEEL".to_string(), mat);

        let mut sec = make_rectangular("RECT", "STEEL", 3.0, 2.0);
        sec.area = 6.0;
        sec.i33 = 4.5;
        sec.as2 = 0.0;
        sec.as3 = 0.0;
        model.frame_sections.insert("RECT".to_string(), sec);

        model.add_point("1", 0.0, 0.0, 0.0);
        model.points.get_mut("1").unwrap().restraint = [true; 6];

        model.add_point("2", 10.0, 0.0, 0.0);
        model.points.get_mut("2").unwrap().restraint = if is_propped {
            [true, true, true, true, false, true]
        } else {
            [false, true, false, true, false, true]
        };
        model
            .points
            .get_mut("1")
            .unwrap()
            .pattern_values
            .insert("LIN_T".to_string(), 0.0);
        model
            .points
            .get_mut("2")
            .unwrap()
            .pattern_values
            .insert("LIN_T".to_string(), 10.0);

        model.add_frame("1", "1", "2", "RECT");

        for i in 1..=3 {
            let name = format!("LC{i}");
            model.load_patterns.insert(
                name.clone(),
                LoadPattern {
                    name: name.clone(),
                    pat_type: LoadPatternType::Temperature,
                    self_wt_multiplier: 0.0,
                },
            );
            model
                .load_cases
                .insert(name.clone(), LoadCase::linear_static(&name, &name, 1.0));
        }

        model
            .frames
            .get_mut("1")
            .unwrap()
            .loads_temp
            .push(FrameLoadTemperature {
                load_pat: "LC1".to_string(),
                my_type: 1,
                val: 20.0,
                pattern_name: None,
            });
        model
            .frames
            .get_mut("1")
            .unwrap()
            .loads_temp
            .push(FrameLoadTemperature {
                load_pat: "LC2".to_string(),
                my_type: 1,
                val: 2.0,
                pattern_name: Some("LIN_T".to_string()),
            });
        model
            .frames
            .get_mut("1")
            .unwrap()
            .loads_temp
            .push(FrameLoadTemperature {
                load_pat: "LC3".to_string(),
                my_type: 2,
                val: 20.0,
                pattern_name: None,
            });

        analyze_model(&mut model).expect("Example 1-002 analysis failed");

        if !is_propped {
            rows.push(BenchmarkVerificationRow::new(
                "Ex 1-002a",
                "LC1 (Uniform +20F)",
                "Ux (Jt 2) [in]",
                0.00130,
                model.case_results["LC1"].joint_displacements["2"][0],
                0.1,
            ));
            rows.push(BenchmarkVerificationRow::new(
                "Ex 1-002a",
                "LC2 (Linear +2F/in)",
                "Ux (Jt 2) [in]",
                0.00065,
                model.case_results["LC2"].joint_displacements["2"][0],
                0.1,
            ));
            rows.push(BenchmarkVerificationRow::new(
                "Ex 1-002a",
                "LC3 (Gradient 20F/in)",
                "Uz (Jt 2) [in]",
                -0.00650,
                model.case_results["LC3"].joint_displacements["2"][2],
                0.1,
            ));
        } else {
            rows.push(BenchmarkVerificationRow::new(
                "Ex 1-002b",
                "LC1 (Uniform +20F)",
                "Fx (Jt 2) [kip]",
                -22.62,
                model.case_results["LC1"].joint_reactions["2"][0],
                0.1,
            ));
            rows.push(BenchmarkVerificationRow::new(
                "Ex 1-002b",
                "LC2 (Linear +2F/in)",
                "Fx (Jt 2) [kip]",
                -11.31,
                model.case_results["LC2"].joint_reactions["2"][0],
                0.1,
            ));
            rows.push(BenchmarkVerificationRow::new(
                "Ex 1-002b",
                "LC3 (Gradient 20F/in)",
                "Fz (Jt 2) [kip]",
                2.545,
                model.case_results["LC3"].joint_reactions["2"][2],
                0.2,
            ));
        }
    }
    rows
}

/// Example 1-003: Frame Distributed and Concentrated Torsional Moments.
pub fn verify_example_1_003() -> Vec<BenchmarkVerificationRow> {
    let mut model = SapModel::new(Units::KipInF);
    model.load_patterns.clear();
    model.load_cases.clear();

    let mut mat = Material::default_steel_kip_in();
    mat.name = "STEEL".to_string();
    mat.e = 28_990.0;
    mat.u = 0.3;
    mat.g = 11_150.0;
    model.materials.insert("STEEL".to_string(), mat);

    let mut sec = FrameSection::default();
    sec.name = "SHAFT".to_string();
    sec.material = "STEEL".to_string();
    sec.area = 7854.0 / 10_000.0;
    sec.torsional_j = 0.09817;
    sec.i22 = 0.04909;
    sec.i33 = 0.04909;
    model.frame_sections.insert("SHAFT".to_string(), sec);

    model.add_point("1", 0.0, 0.0, 0.0);
    model.points.get_mut("1").unwrap().restraint = [true; 6];
    for (id, x) in [("2", 5.0), ("3", 7.0), ("4", 8.0), ("5", 10.0)] {
        model.add_point(id, x, 0.0, 0.0);
        model.points.get_mut(id).unwrap().restraint = [true, true, true, false, true, true];
    }

    model.add_frame("1", "1", "2", "SHAFT");
    model.add_frame("2", "2", "3", "SHAFT");
    model.add_frame("3", "3", "4", "SHAFT");
    model.add_frame("4", "4", "5", "SHAFT");

    for name in ["LC1", "LC2"] {
        model.load_patterns.insert(
            name.to_string(),
            LoadPattern {
                name: name.to_string(),
                pat_type: LoadPatternType::Other,
                self_wt_multiplier: 0.0,
            },
        );
        model
            .load_cases
            .insert(name.to_string(), LoadCase::linear_static(name, name, 1.0));
    }

    // LC1: Uniform torque mx = 1.0 kip-in/in on Elem 2 + point torque T = 2.0 kip-in at Jt 4 end of Elem 3
    model
        .frames
        .get_mut("2")
        .unwrap()
        .loads_dist
        .push(FrameLoadDistributed {
            load_pat: "LC1".to_string(),
            my_type: 2,
            dir: 1,
            csys: "LOCAL".to_string(),
            rel_dist1: 0.0,
            rel_dist2: 1.0,
            val1: 1.0,
            val2: 1.0,
        });
    model
        .frames
        .get_mut("3")
        .unwrap()
        .loads_point
        .push(FrameLoadPoint {
            load_pat: "LC1".to_string(),
            my_type: 2,
            dir: 1,
            csys: "LOCAL".to_string(),
            rel_dist: 1.0,
            val: 2.0,
        });

    // LC2: Triangular torque 0 -> 1.5 kip-in/in on Elem 1 + point torque T = 2.0 kip-in at Jt 5 end of Elem 4
    model
        .frames
        .get_mut("1")
        .unwrap()
        .loads_dist
        .push(FrameLoadDistributed {
            load_pat: "LC2".to_string(),
            my_type: 2,
            dir: 1,
            csys: "LOCAL".to_string(),
            rel_dist1: 0.0,
            rel_dist2: 1.0,
            val1: 0.0,
            val2: 1.5,
        });
    model
        .frames
        .get_mut("4")
        .unwrap()
        .loads_point
        .push(FrameLoadPoint {
            load_pat: "LC2".to_string(),
            my_type: 2,
            dir: 1,
            csys: "LOCAL".to_string(),
            rel_dist: 1.0,
            val: 2.0,
        });

    analyze_model(&mut model).expect("Example 1-003 analysis failed");

    vec![
        BenchmarkVerificationRow::new(
            "Ex 1-003",
            "LC1 (Unif + Pt Torque)",
            "Mx (Jt 1) [kip-in]",
            -4.0,
            model.case_results["LC1"].joint_reactions["1"][3],
            0.2,
        ),
        BenchmarkVerificationRow::new(
            "Ex 1-003",
            "LC1 (Unif + Pt Torque)",
            "Rx (Jt 3) [rad]",
            0.02375,
            model.case_results["LC1"].joint_displacements["3"][3],
            0.2,
        ),
        BenchmarkVerificationRow::new(
            "Ex 1-003",
            "LC1 (Unif + Pt Torque)",
            "Rx (Jt 5) [rad]",
            0.02558,
            model.case_results["LC1"].joint_displacements["5"][3],
            0.2,
        ),
        BenchmarkVerificationRow::new(
            "Ex 1-003",
            "LC2 (Tri + Pt Torque)",
            "Mx (Jt 1) [kip-in]",
            -5.75,
            model.case_results["LC2"].joint_reactions["1"][3],
            0.2,
        ),
        BenchmarkVerificationRow::new(
            "Ex 1-003",
            "LC2 (Tri + Pt Torque)",
            "Rx (Jt 3) [rad]",
            0.02421,
            model.case_results["LC2"].joint_displacements["3"][3],
            0.2,
        ),
        BenchmarkVerificationRow::new(
            "Ex 1-003",
            "LC2 (Tri + Pt Torque)",
            "Rx (Jt 5) [rad]",
            0.02969,
            model.case_results["LC2"].joint_displacements["5"][3],
            0.2,
        ),
    ]
}

/// Example 1-004: Frame with Rotated Local Axes (beta = 30 deg).
pub fn verify_example_1_004() -> Vec<BenchmarkVerificationRow> {
    let mut model = SapModel::new(Units::KipInF);
    model.load_patterns.clear();
    model.load_cases.clear();

    let mut mat = Material::default_steel_kip_in();
    mat.name = "STEEL".to_string();
    mat.e = 29_000.0;
    model.materials.insert("STEEL".to_string(), mat);

    let mut sec = FrameSection::default();
    sec.name = "W12X106".to_string();
    sec.material = "STEEL".to_string();
    sec.area = 31.2;
    sec.i33 = 933.0;
    sec.i22 = 301.0;
    sec.as2 = 0.0;
    sec.as3 = 0.0;
    model.frame_sections.insert("W12X106".to_string(), sec);

    model.add_point("1", 0.0, 0.0, 0.0);
    model.points.get_mut("1").unwrap().restraint = [true; 6];
    model.add_point("2", 144.0, 0.0, 0.0);

    model.add_frame("1", "1", "2", "W12X106");
    model.frames.get_mut("1").unwrap().angle_deg = 30.0;

    for name in ["LC1", "LC2", "LC3"] {
        model.load_patterns.insert(
            name.to_string(),
            LoadPattern {
                name: name.to_string(),
                pat_type: LoadPatternType::Other,
                self_wt_multiplier: 0.0,
            },
        );
        model
            .load_cases
            .insert(name.to_string(), LoadCase::linear_static(name, name, 1.0));
    }

    // LC1: Uniform load wz = -0.01 kip/in in Global Z
    model
        .frames
        .get_mut("1")
        .unwrap()
        .loads_dist
        .push(FrameLoadDistributed {
            load_pat: "LC1".to_string(),
            my_type: 1,
            dir: 6,
            csys: "GLOBAL".to_string(),
            rel_dist1: 0.0,
            rel_dist2: 1.0,
            val1: -0.01,
            val2: -0.01,
        });

    // LC2: Point load Pz = -1.0 kip at Joint 2
    model
        .points
        .get_mut("2")
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: "LC2".to_string(),
            csys: "GLOBAL".to_string(),
            force: [0.0, 0.0, -1.0, 0.0, 0.0, 0.0],
        });

    // LC3: Point moment My = +240 kip-in at Joint 2 (bends cantilever downward in XZ plane)
    model
        .points
        .get_mut("2")
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: "LC3".to_string(),
            csys: "GLOBAL".to_string(),
            force: [0.0, 0.0, 0.0, 0.0, 240.0, 0.0],
        });

    analyze_model(&mut model).expect("Example 1-004 analysis failed");

    vec![
        BenchmarkVerificationRow::new(
            "Ex 1-004",
            "LC1 (Uniform Z, 30 deg)",
            "Uy (Jt 2) [in]",
            -0.01806,
            model.case_results["LC1"].joint_displacements["2"][1],
            0.2,
        ),
        BenchmarkVerificationRow::new(
            "Ex 1-004",
            "LC1 (Uniform Z, 30 deg)",
            "Uz (Jt 2) [in]",
            -0.03029,
            model.case_results["LC1"].joint_displacements["2"][2],
            0.2,
        ),
        BenchmarkVerificationRow::new(
            "Ex 1-004",
            "LC2 (Point Fz, 30 deg)",
            "Uy (Jt 2) [in]",
            -0.03345,
            model.case_results["LC2"].joint_displacements["2"][1],
            0.2,
        ),
        BenchmarkVerificationRow::new(
            "Ex 1-004",
            "LC2 (Point Fz, 30 deg)",
            "Uz (Jt 2) [in]",
            -0.05610,
            model.case_results["LC2"].joint_displacements["2"][2],
            0.2,
        ),
        BenchmarkVerificationRow::new(
            "Ex 1-004",
            "LC3 (Moment My, 30 deg)",
            "Uy (Jt 2) [in]",
            -0.08361,
            model.case_results["LC3"].joint_displacements["2"][1],
            0.2,
        ),
        BenchmarkVerificationRow::new(
            "Ex 1-004",
            "LC3 (Moment My, 30 deg)",
            "Uz (Jt 2) [in]",
            -0.14024,
            model.case_results["LC3"].joint_displacements["2"][2],
            0.2,
        ),
    ]
}

/// Example 1-007: Frame End Releases (Models A, B, C).
pub fn verify_example_1_007() -> Vec<BenchmarkVerificationRow> {
    let mut rows = Vec::new();
    for (label, rel_mode, target_fz, target_my) in [
        ("Ex 1-007a (Shear V2 Release)", 1, 0.0, 50.0),
        ("Ex 1-007b (Axial P Release)", 2, 0.0, 50.0),
        ("Ex 1-007c (Moment M3 Release)", 3, 5.0, 0.0),
    ] {
        let mut model = SapModel::new(Units::KipFtF);
        model.load_patterns.clear();
        model.load_cases.clear();

        let mut sec = FrameSection::default();
        sec.name = "SEC1".to_string();
        sec.area = 1.0;
        sec.i33 = 1.0;
        sec.as2 = 0.0;
        model.frame_sections.insert("SEC1".to_string(), sec);

        model.add_point("1", 0.0, 0.0, 0.0);
        model.points.get_mut("1").unwrap().restraint = [true; 6];
        model.add_point("2", 0.0, 0.0, 10.0);
        model.points.get_mut("2").unwrap().restraint = [false, true, false, true, false, true];
        model.add_point("3", 10.0, 0.0, 10.0);
        model.points.get_mut("3").unwrap().restraint = [false, true, true, true, false, true];

        model.add_frame("1", "1", "2", "SEC1");
        model.add_frame("2", "2", "3", "SEC1");

        match rel_mode {
            1 => {
                // Shear V2 release at Jt 2 end of Beam (Frame 2 I-end)
                model.frames.get_mut("2").unwrap().release_i[1] = true;
            }
            2 => {
                // Axial P release at Jt 2 end of Column (Frame 1 J-end)
                model.frames.get_mut("1").unwrap().release_j[0] = true;
            }
            3 => {
                // Moment M3 release at Jt 2 end of Beam (Frame 2 I-end)
                model.frames.get_mut("2").unwrap().release_i[5] = true;
            }
            _ => {}
        }

        model.load_patterns.insert(
            "L1".to_string(),
            LoadPattern {
                name: "L1".to_string(),
                pat_type: LoadPatternType::Other,
                self_wt_multiplier: 0.0,
            },
        );
        model
            .load_cases
            .insert("L1".to_string(), LoadCase::linear_static("L1", "L1", 1.0));
        model
            .frames
            .get_mut("2")
            .unwrap()
            .loads_point
            .push(FrameLoadPoint {
                load_pat: "L1".to_string(),
                my_type: 1,
                dir: 6,
                csys: "GLOBAL".to_string(),
                rel_dist: 0.5,
                val: -10.0,
            });

        analyze_model(&mut model).expect("Example 1-007 analysis failed");
        rows.push(BenchmarkVerificationRow::new(
            label,
            "Midspan -10k",
            "Fz (Jt 1) [kip]",
            target_fz,
            model.case_results["L1"].joint_reactions["1"][2],
            0.2,
        ));
        rows.push(BenchmarkVerificationRow::new(
            label,
            "Midspan -10k",
            "My (Jt 1) [kip-ft]",
            target_my,
            model.case_results["L1"].joint_reactions["1"][4],
            0.2,
        ));
    }
    rows
}

/// Example 1-008: Frame Partial Fixity End Releases + Timoshenko Shear.
pub fn verify_example_1_008() -> Vec<BenchmarkVerificationRow> {
    let mut model = SapModel::new(Units::KipInF);
    model.load_patterns.clear();
    model.load_cases.clear();

    let mut mat = Material::default_concrete_kip_in();
    mat.name = "CONC".to_string();
    mat.e = 4_320.0;
    mat.u = 0.2;
    mat.g = 1_800.0;
    mat.unit_weight = 0.15 / 1728.0;
    model.materials.insert("CONC".to_string(), mat);

    let mut sec = make_rectangular("RECT", "CONC", 30.0, 18.0);
    sec.area = 540.0;
    sec.i33 = 40_500.0;
    sec.as2 = 450.0;
    model.frame_sections.insert("RECT".to_string(), sec);

    model.add_point("1", 0.0, 0.0, 0.0);
    model.points.get_mut("1").unwrap().restraint = [true; 6];
    model.add_point("2", 288.0, 0.0, 0.0);
    model.points.get_mut("2").unwrap().restraint = [false, true, false, true, false, true];

    model.add_frame("1", "1", "2", "RECT");
    {
        let f = model.frames.get_mut("1").unwrap();
        f.release_i[1] = true; // Partial shear release
        f.start_spring[1] = 540.0;
        f.release_i[5] = true; // Partial moment M3 release
        f.start_spring[5] = 3_888_000.0;
    }

    model.load_patterns.insert(
        "DEAD".to_string(),
        LoadPattern {
            name: "DEAD".to_string(),
            pat_type: LoadPatternType::Dead,
            self_wt_multiplier: 2.0,
        },
    );
    model.load_cases.insert(
        "DEAD".to_string(),
        LoadCase::linear_static("DEAD", "DEAD", 1.0),
    );

    analyze_model(&mut model).expect("Example 1-008 analysis failed");
    vec![BenchmarkVerificationRow::new(
        "Ex 1-008",
        "Partial Fixity + Shear",
        "Uz (Jt 2) [in]",
        -0.8036,
        model.case_results["DEAD"].joint_displacements["2"][2],
        0.1,
    )]
}

/// Example 1-010: Frame End Offsets & Rigid Zone Factor (Models A, B, C, D).
pub fn verify_example_1_010() -> Vec<BenchmarkVerificationRow> {
    let mut rows = Vec::new();
    for (label, offset_i, rz_factor, target_uz) in [
        ("Ex 1-010a (a=0)", 0.0, 0.0, -0.39951),
        ("Ex 1-010b (a=6, r=0.0)", 6.0, 0.0, -0.39951),
        ("Ex 1-010c (a=6, r=0.5)", 6.0, 0.5, -0.37523),
        ("Ex 1-010d (a=6, r=1.0)", 6.0, 1.0, -0.35197),
    ] {
        let mut model = SapModel::new(Units::KipInF);
        model.load_patterns.clear();
        model.load_cases.clear();

        let mut mat = Material::default_concrete_kip_in();
        mat.name = "CONC".to_string();
        mat.e = 4_320.0;
        mat.u = 0.2;
        mat.g = 1_800.0;
        model.materials.insert("CONC".to_string(), mat);

        let mut sec = make_rectangular("RECT", "CONC", 18.0, 12.0);
        sec.area = 216.0;
        sec.i33 = 5_832.0;
        sec.as2 = 180.0;
        model.frame_sections.insert("RECT".to_string(), sec);

        model.add_point("1", 0.0, 0.0, 0.0);
        model.points.get_mut("1").unwrap().restraint = [true; 6];
        model.add_point("2", 144.0, 0.0, 0.0);
        model.points.get_mut("2").unwrap().restraint = [false, true, false, true, false, true];

        model.add_frame("1", "1", "2", "RECT");
        {
            let f = model.frames.get_mut("1").unwrap();
            f.end_offset_i = offset_i;
            f.rigid_zone_factor = rz_factor;
        }

        model.load_patterns.insert(
            "L1".to_string(),
            LoadPattern {
                name: "L1".to_string(),
                pat_type: LoadPatternType::Other,
                self_wt_multiplier: 0.0,
            },
        );
        model
            .load_cases
            .insert("L1".to_string(), LoadCase::linear_static("L1", "L1", 1.0));
        model
            .points
            .get_mut("2")
            .unwrap()
            .loads_force
            .push(PointLoadForce {
                load_pat: "L1".to_string(),
                csys: "GLOBAL".to_string(),
                force: [0.0, 0.0, -10.0, 0.0, 0.0, 0.0],
            });

        analyze_model(&mut model).expect("Example 1-010 analysis failed");
        rows.push(BenchmarkVerificationRow::new(
            label,
            "Tip -10k",
            "Uz (Jt 2) [in]",
            target_uz,
            model.case_results["L1"].joint_displacements["2"][2],
            0.1,
        ));
    }
    rows
}

/// Example 1-016: Tension Stiffening Using P-Delta Analysis.
pub fn verify_example_1_016() -> Vec<BenchmarkVerificationRow> {
    let mut rows = Vec::new();
    for with_pdelta in [false, true] {
        let mut model = SapModel::new(Units::KipInF);
        model.load_patterns.clear();
        model.load_cases.clear();

        let mut mat = Material::default_steel_kip_in();
        mat.name = "STEEL".to_string();
        mat.e = 30_000.0;
        model.materials.insert("STEEL".to_string(), mat);

        let mut sec = make_rectangular("ROD", "STEEL", 3.0, 3.0);
        sec.area = 9.0;
        sec.i33 = 6.75;
        sec.as2 = 0.0;
        model.frame_sections.insert("ROD".to_string(), sec);

        model.add_point("1", 0.0, 0.0, 0.0);
        model.points.get_mut("1").unwrap().restraint = [true, true, true, true, false, true];
        model.add_point("2", 150.0, 0.0, 0.0);
        model.points.get_mut("2").unwrap().restraint = [false, true, false, true, false, true];
        model.add_point("3", 300.0, 0.0, 0.0);
        model.points.get_mut("3").unwrap().restraint = [false, true, true, true, false, true];

        model.add_frame("1", "1", "2", "ROD");
        model.add_frame("2", "2", "3", "ROD");

        model.load_patterns.insert(
            "L1".to_string(),
            LoadPattern {
                name: "L1".to_string(),
                pat_type: LoadPatternType::Other,
                self_wt_multiplier: 0.0,
            },
        );
        let mut lc = LoadCase::linear_static("L1", "L1", 1.0);
        if with_pdelta {
            lc.case_type = LoadCaseType::NonlinearStatic;
            lc.nl_geom = NLGeomType::PDelta;
        }
        model.load_cases.insert("L1".to_string(), lc);

        for fid in ["1", "2"] {
            model
                .frames
                .get_mut(fid)
                .unwrap()
                .loads_dist
                .push(FrameLoadDistributed {
                    load_pat: "L1".to_string(),
                    my_type: 1,
                    dir: 6,
                    csys: "GLOBAL".to_string(),
                    rel_dist1: 0.0,
                    rel_dist2: 1.0,
                    val1: -0.002,
                    val2: -0.002,
                });
        }
        if with_pdelta {
            model
                .points
                .get_mut("3")
                .unwrap()
                .loads_force
                .push(PointLoadForce {
                    load_pat: "L1".to_string(),
                    csys: "GLOBAL".to_string(),
                    force: [20.25, 0.0, 0.0, 0.0, 0.0, 0.0],
                });
        }

        analyze_model(&mut model).expect("Example 1-016 analysis failed");
        let uz_mid = model.case_results["L1"].joint_displacements["2"][2];
        if !with_pdelta {
            rows.push(BenchmarkVerificationRow::new(
                "Ex 1-016a (Linear)",
                "No P-Delta",
                "Uz (Midspan) [in]",
                -1.04167,
                uz_mid,
                0.1,
            ));
        } else {
            rows.push(BenchmarkVerificationRow::new(
                "Ex 1-016b (P-Delta)",
                "Tension S=20.25k",
                "Uz (Midspan) [in]",
                -0.54555,
                uz_mid,
                0.1,
            ));
        }
    }
    rows
}

/// Automatic frame subdivision: the published SAP2000 value for each mesh of
/// Examples 1-014 (cantilever periods), 1-016 (tension stiffening), 1-017
/// (string frequencies) and 1-019 (frame buckling). Each frame object is one
/// object assigned a number of segments.
pub fn verify_frame_auto_subdivision() -> Vec<BenchmarkVerificationRow> {
    use sap_model::{FramePropType, FrameSection};

    let bare = |active_dofs: [bool; 6], e: f64, unit_mass: f64, area: f64, inertia: [f64; 2]| {
        let mut model = SapModel::new(Units::KipInF);
        model.load_cases.clear();
        model.load_patterns.clear();
        model.active_dofs = active_dofs;
        let mut material = Material::default_steel_kip_in();
        material.name = "MAT".into();
        material.e = e;
        material.g = e / 2.6;
        material.unit_mass = unit_mass;
        material.unit_weight = 0.0;
        model.materials.insert("MAT".into(), material);
        model.frame_sections.insert(
            "SEC".into(),
            FrameSection {
                name: "SEC".into(),
                material: "MAT".into(),
                prop_type: FramePropType::General,
                area,
                i33: inertia[0],
                i22: inertia[1],
                as2: 0.0,
                as3: 0.0,
                modifiers: [1.0; 8],
                ..FrameSection::default()
            },
        );
        model
    };
    let pattern = |model: &mut SapModel, name: &str| {
        model.load_patterns.insert(
            name.into(),
            LoadPattern {
                name: name.into(),
                pat_type: LoadPatternType::Other,
                self_wt_multiplier: 0.0,
            },
        );
    };
    let joint_force = |model: &mut SapModel, joint: &str, pat: &str, force: [f64; 6]| {
        model.points[joint].loads_force.push(PointLoadForce {
            load_pat: pat.into(),
            csys: "GLOBAL".into(),
            force,
        });
    };
    let mut rows = Vec::new();

    // Example 1-014: first period of a 96 in concrete cantilever.
    for (label, segments, period) in
        [("Ex 1-014b", 2, 0.042333), ("Ex 1-014f", 10, 0.038175), ("Ex 1-014g", 96, 0.038003)]
    {
        let mut model =
            bare([false, true, true, false, true, true], 3600.0, 2.3e-7, 216.0, [5832.0, 2592.0]);
        model.add_point("1", 0.0, 0.0, 0.0);
        model.add_point("2", 96.0, 0.0, 0.0);
        model.points["1"].restraint = [true; 6];
        model.add_frame("1", "1", "2", "SEC");
        model.frames["1"].auto_mesh_segments = segments;
        model
            .load_cases
            .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 5));
        analyze_model(&mut model).expect("Example 1-014 analysis failed");
        rows.push(BenchmarkVerificationRow::new(
            label,
            &format!("{segments} elements"),
            "T1 [s]",
            period,
            model.case_results["MODAL"].modes[0].period_sec,
            0.002,
        ));
    }

    // Example 1-016: midpoint deflection with an assigned P-Delta force.
    for (label, segments, uz) in [("Ex 1-016c", 2, -0.54343), ("Ex 1-016d", 16, -0.54330)] {
        let mut model = bare([true; 6], 30_000.0, 0.0, 9.0, [6.75, 6.75]);
        model.add_point("1", 0.0, 0.0, 0.0);
        model.add_point("2", 150.0, 0.0, 0.0);
        model.add_point("3", 300.0, 0.0, 0.0);
        model.points["1"].restraint = [true, true, true, true, false, true];
        model.points["3"].restraint = [false, true, true, true, false, true];
        pattern(&mut model, "Q");
        for (frame, i, j) in [("1", "1", "2"), ("2", "2", "3")] {
            model.add_frame(frame, i, j, "SEC");
            let object = &mut model.frames[frame];
            object.auto_mesh_segments = segments;
            object.pdelta_force = 20.25;
            object.loads_dist.push(FrameLoadDistributed {
                load_pat: "Q".into(),
                my_type: 1,
                dir: 10,
                csys: "GLOBAL".into(),
                rel_dist1: 0.0,
                rel_dist2: 1.0,
                val1: 0.002,
                val2: 0.002,
            });
        }
        model
            .load_cases
            .insert("Q".into(), LoadCase::linear_static("Q", "Q", 1.0));
        analyze_model(&mut model).expect("Example 1-016 analysis failed");
        rows.push(BenchmarkVerificationRow::new(
            label,
            &format!("P-Delta force, {segments} elements"),
            "Uz (Midspan) [in]",
            uz,
            model.case_results["Q"].joint_displacements["2"][2],
            0.002,
        ));
    }

    // Example 1-017: frequencies of a string tensioned by a P-Delta case.
    {
        let (area, inertia) = (0.0030679615757712823, 7.490140565847857e-7);
        let mut model = bare(
            [true, false, true, false, true, false],
            30_000.0,
            7.324e-7,
            area,
            [inertia, inertia],
        );
        model.frame_sections["SEC"].as2 = 0.9 * area;
        model.frame_sections["SEC"].as3 = 0.9 * area;
        model.add_point("1", -50.0, 0.0, 0.0);
        model.add_point("2", 50.0, 0.0, 0.0);
        model.points["1"].restraint = [true, true, true, true, false, true];
        model.points["2"].restraint = [false, false, true, false, false, false];
        model.add_frame("1", "1", "2", "SEC");
        model.frames["1"].auto_mesh_segments = 10;
        pattern(&mut model, "T");
        joint_force(&mut model, "2", "T", [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]);
        let mut tension = LoadCase::linear_static("T", "T", 1.0);
        tension.case_type = LoadCaseType::NonlinearStatic;
        tension.nl_geom = NLGeomType::PDelta;
        model.load_cases.insert("T".into(), tension);
        let mut modal = LoadCase::modal_eigen("MODAL", 3);
        modal.initial_case = Some("T".into());
        model.load_cases.insert("MODAL".into(), modal);
        analyze_model(&mut model).expect("Example 1-017 analysis failed");
        for (mode, frequency) in [(0, 74.579), (2, 222.06)] {
            rows.push(BenchmarkVerificationRow::new(
                "Ex 1-017a",
                "10 elements",
                &format!("f{} [Hz]", mode + 1),
                frequency,
                model.case_results["MODAL"].modes[mode].freq_hz,
                0.003,
            ));
        }
    }

    // Example 1-019: buckling load of a pinned-base portal frame. E is 29000
    // ksi as in the vendor model and its published numbers (the problem
    // statement prints 29,900).
    for (label, segments, load) in [("Ex 1-019b", 2, 280.24), ("Ex 1-019c", 4, 280.19)] {
        let mut model =
            bare([true, false, true, false, true, false], 29_000.0, 0.0, 9.12, [110.0, 37.1]);
        model.frame_sections["SEC"].modifiers[0] = 100_000.0;
        for (name, x, z) in
            [("1", 0.0, 0.0), ("2", 0.0, 144.0), ("5", 144.0, 144.0), ("6", 144.0, 0.0)]
        {
            model.add_point(name, x, 0.0, z);
        }
        for base in ["1", "6"] {
            model.points[base].restraint = [true, true, true, true, false, true];
        }
        for (frame, i, j) in [("1", "1", "2"), ("3", "2", "5"), ("5", "6", "5")] {
            model.add_frame(frame, i, j, "SEC");
            model.frames[frame].auto_mesh_segments = segments;
        }
        pattern(&mut model, "P");
        for top in ["2", "5"] {
            joint_force(&mut model, top, "P", [0.0, 0.0, -1.0, 0.0, 0.0, 0.0]);
        }
        let mut case = LoadCase::linear_static("BUCK", "P", 1.0);
        case.case_type = LoadCaseType::Buckling;
        case.num_buckling_modes = 1;
        model.load_cases.insert("BUCK".into(), case);
        analyze_model(&mut model).expect("Example 1-019 analysis failed");
        rows.push(BenchmarkVerificationRow::new(
            label,
            &format!("{segments} elements per object"),
            "Buckling load [kip]",
            load,
            model.case_results["BUCK"].buckling_factors[0],
            0.002,
        ));
    }
    rows
}

/// Example 1-011: Frame Insertion Point. A 12 x 18 in cantilever inserted at
/// its bottom left corner (cardinal point 1) with +12 in joint offsets along
/// local 2, under 10 kip of compression on the end joint (case 1) and on the
/// end of the frame object (case 2).
pub fn verify_example_1_011() -> Vec<BenchmarkVerificationRow> {
    let mut model = SapModel::new(Units::KipInF);
    model.load_cases.clear();
    model.load_patterns.clear();
    let mut material = Material::default_steel_kip_in();
    material.name = "CONC".into();
    material.e = 3600.0;
    material.g = 1500.0;
    model.materials.insert("CONC".into(), material);
    model
        .frame_sections
        .insert("FSEC1".into(), make_rectangular("FSEC1", "CONC", 18.0, 12.0));
    model.add_point("1", 0.0, 0.0, 0.0);
    model.add_point("2", 120.0, 0.0, 0.0);
    model.points["1"].restraint = [true; 6];
    model.add_frame("1", "1", "2", "FSEC1");
    let frame = &mut model.frames["1"];
    frame.cardinal_point = 1;
    frame.stiff_transform = true;
    frame.joint_offset_is_global = false;
    frame.joint_offset_i = [0.0, 12.0, 0.0];
    frame.joint_offset_j = [0.0, 12.0, 0.0];
    frame.loads_point.push(FrameLoadPoint {
        load_pat: "2".into(),
        my_type: 1,
        dir: 4,
        csys: "GLOBAL".into(),
        rel_dist: 1.0,
        val: -10.0,
    });
    for name in ["1", "2"] {
        model.load_patterns.insert(
            name.into(),
            LoadPattern {
                name: name.into(),
                pat_type: LoadPatternType::Other,
                self_wt_multiplier: 0.0,
            },
        );
        model
            .load_cases
            .insert(name.into(), LoadCase::linear_static(name, name, 1.0));
    }
    model.points["2"].loads_force.push(PointLoadForce {
        load_pat: "1".into(),
        csys: "GLOBAL".into(),
        force: [-10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    });
    analyze_model(&mut model).expect("Example 1-011 analysis failed");
    let beam = &model.case_results["1"].frame_forces["1"][4];
    let support = model.case_results["2"].joint_reactions["1"];
    vec![
        BenchmarkVerificationRow::new("Ex 1-011", "1 (load on joint)", "M3 (beam) [k-in]", -210.0, beam.m3, 0.001),
        BenchmarkVerificationRow::new("Ex 1-011", "1 (load on joint)", "M2 (beam) [k-in]", -60.0, beam.m2, 0.001),
        BenchmarkVerificationRow::new("Ex 1-011", "2 (load on frame)", "My (fixed end) [k-in]", 210.0, support[4], 0.001),
        BenchmarkVerificationRow::new("Ex 1-011", "2 (load on frame)", "Mz (fixed end) [k-in]", 60.0, support[5], 0.001),
    ]
}

/// Example 5-001: Solid Element Patch Test (MacNeal & Harder 1985, 7-Element Irregular Unit Cube).
pub fn verify_example_5_001() -> Vec<BenchmarkVerificationRow> {
    use sap_model::{PointLoadDispl, SolidObj, SolidSection};
    let mut rows = Vec::new();
    let joints: [[f64; 3]; 16] = [
        // Inner joints 1..8
        [0.249, 0.342, 0.192],
        [0.826, 0.288, 0.288],
        [0.850, 0.649, 0.263],
        [0.273, 0.750, 0.230],
        [0.320, 0.186, 0.643],
        [0.677, 0.305, 0.683],
        [0.788, 0.693, 0.644],
        [0.165, 0.745, 0.702],
        // Outer cube corners 9..16
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [1.0, 1.0, 1.0],
        [0.0, 1.0, 1.0],
    ];
    let elem_conn: [[usize; 8]; 7] = [
        [9, 1, 12, 4, 13, 5, 16, 8],
        [9, 10, 1, 2, 13, 14, 5, 6],
        [1, 2, 4, 3, 5, 6, 8, 7],
        [4, 3, 12, 11, 8, 7, 16, 15],
        [2, 10, 3, 11, 6, 14, 7, 15],
        [5, 6, 8, 7, 13, 14, 16, 15],
        [9, 10, 12, 11, 1, 2, 4, 3],
    ];
    for (incomp, label) in [
        (false, "Ex 5-001 (Solid Comp)"),
        (true, "Ex 5-001 (Solid Incomp)"),
    ] {
        let mut model = SapModel::new(Units::KipInF);
        model.load_patterns.clear();
        model.load_cases.clear();
        let mut mat = Material::default_steel_kip_in();
        mat.name = "M1".into();
        mat.e = 1.0e6;
        mat.u = 0.25;
        mat.g = 1.0e6 / 2.5;
        mat.unit_weight = 0.0;
        model.materials.insert("M1".into(), mat);
        model.solid_sections.insert(
            "SOL1".into(),
            SolidSection {
                name: "SOL1".into(),
                material: "M1".into(),
                mat_angle_deg: [0.0; 3],
                incompatible_modes: incomp,
                modifiers: [1.0; 8],
            },
        );
        model.load_patterns.insert(
            "PAT1".into(),
            LoadPattern {
                name: "PAT1".into(),
                pat_type: LoadPatternType::Other,
                self_wt_multiplier: 0.0,
            },
        );
        model
            .load_cases
            .insert("PAT1".into(), LoadCase::linear_static("PAT1", "PAT1", 1.0));
        for (i, &[x, y, z]) in joints.iter().enumerate() {
            let name = (i + 1).to_string();
            model.add_point(&name, x, y, z);
            if i >= 8 {
                let u = 1.0e-3 * (2.0 * x + y + z) / 2.0;
                let v = 1.0e-3 * (x + 2.0 * y + z) / 2.0;
                let w = 1.0e-3 * (x + y + 2.0 * z) / 2.0;
                let pt = model.points.get_mut(&name).unwrap();
                pt.restraint = [true, true, true, false, false, false];
                pt.loads_displ.push(PointLoadDispl {
                    load_pat: "PAT1".into(),
                    csys: "GLOBAL".into(),
                    displ: [u, v, w, 0.0, 0.0, 0.0],
                });
            }
        }
        for (ei, conn) in elem_conn.iter().enumerate() {
            let pts: [String; 8] = std::array::from_fn(|k| conn[k].to_string());
            let sname = (ei + 1).to_string();
            model.solids.insert(
                sname.clone(),
                SolidObj {
                    name: sname,
                    points: pts,
                    section: "SOL1".into(),
                    loads_surface: Vec::new(),
                    loads_temp: std::collections::BTreeMap::new(),
                    selected: false,
                },
            );
        }
        analyze_model(&mut model).expect("Ex 5-001 analysis failed");
        let res = &model.case_results["PAT1"];
        let u8 = res.joint_displacements["8"];
        let st = res.solid_results["3"].center_stress;
        rows.push(BenchmarkVerificationRow::new(
            label,
            "PAT1",
            "Ux (Jt 8) [in]",
            0.0008885,
            u8[0],
            0.1,
        ));
        rows.push(BenchmarkVerificationRow::new(
            label,
            "PAT1",
            "Uy (Jt 8) [in]",
            0.0011785,
            u8[1],
            0.1,
        ));
        rows.push(BenchmarkVerificationRow::new(
            label,
            "PAT1",
            "Uz (Jt 8) [in]",
            0.001157,
            u8[2],
            0.1,
        ));
        rows.push(BenchmarkVerificationRow::new(
            label,
            "PAT1",
            "S11 [psi]",
            2000.0,
            st.s11,
            0.1,
        ));
        rows.push(BenchmarkVerificationRow::new(
            label,
            "PAT1",
            "S12 [psi]",
            400.0,
            st.s12,
            0.1,
        ));
    }
    rows
}

/// Example 6-003: Portal Frame with Grounded Gap Link Under Static & Direct-Integration Time History.
pub fn verify_example_6_003() -> Vec<BenchmarkVerificationRow> {
    use sap_model::{DynamicFunction, LinkObj, LinkProp, LinkPropType};
    let mut model = SapModel::new(Units::KipInF);
    model.load_patterns.clear();
    model.load_cases.clear();

    let mut mat = Material::default_steel_kip_in();
    mat.name = "STEEL".to_string();
    mat.e = 29_900.0;
    mat.u = 0.3;
    mat.g = 11_500.0;
    mat.unit_weight = 0.0;
    mat.unit_mass = 0.0;
    model.materials.insert("STEEL".to_string(), mat);

    let mut sec = FrameSection::default();
    sec.name = "FSEC1".to_string();
    sec.material = "STEEL".to_string();
    sec.area = 10.0;
    sec.i33 = 100.0;
    sec.as2 = 2.0;
    model.frame_sections.insert("FSEC1".to_string(), sec);

    model.add_point("1", 0.0, 0.0, 0.0);
    model.points.get_mut("1").unwrap().restraint = [true; 6];
    model.add_point("2", 144.0, 0.0, 0.0);
    model.points.get_mut("2").unwrap().mass = [0.0, 0.0, 0.001, 0.0, 0.0, 0.0];
    model.add_point("3", 0.0, 0.0, 144.0);
    model.points.get_mut("3").unwrap().mass = [0.3, 0.0, 0.1, 0.0, 0.0, 0.0];
    model.add_point("4", 144.0, 0.0, 144.0);
    model.points.get_mut("4").unwrap().mass = [0.3, 0.0, 0.1, 0.0, 0.0, 0.0];

    model.add_frame("1", "1", "3", "FSEC1");
    model.add_frame("2", "2", "4", "FSEC1");
    model.add_frame("3", "3", "4", "FSEC1");

    let mut gap = LinkProp {
        name: "GAP1".to_string(),
        link_type: LinkPropType::Gap,
        ..LinkProp::default()
    };
    gap.dof_active[0] = true;
    gap.dof_nonlinear[0] = true;
    gap.nonlinear_data_complete = true;
    gap.ke[0] = 200_000.0;
    gap.k_nl[0] = 200_000.0;
    gap.param1[0] = 0.0;
    model.link_props.insert("GAP1".to_string(), gap);
    model.links.insert(
        "1".to_string(),
        LinkObj {
            name: "1".to_string(),
            point_i: "2".to_string(),
            point_j: None,
            prop: "GAP1".to_string(),
            angle_deg: 0.0,
            selected: false,
        },
    );

    for pat in ["P", "V"] {
        model.load_patterns.insert(
            pat.to_string(),
            LoadPattern {
                name: pat.to_string(),
                pat_type: LoadPatternType::Other,
                self_wt_multiplier: 0.0,
            },
        );
    }
    model
        .frames
        .get_mut("3")
        .unwrap()
        .loads_point
        .push(FrameLoadPoint {
            load_pat: "P".to_string(),
            my_type: 1,
            dir: 6,
            csys: "GLOBAL".to_string(),
            rel_dist: 0.5,
            val: -10.0,
        });
    model
        .points
        .get_mut("4")
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: "V".to_string(),
            csys: "GLOBAL".to_string(),
            force: [-20.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        });

    model.functions.insert(
        "RAMP20".to_string(),
        DynamicFunction {
            name: "RAMP20".to_string(),
            is_time_history: true,
            damping_ratio: 0.0,
            points: vec![(0.0, 0.0), (20.0, 1.0), (100.0, 1.0)],
        },
    );
    model.functions.insert(
        "RAMP40".to_string(),
        DynamicFunction {
            name: "RAMP40".to_string(),
            is_time_history: true,
            damping_ratio: 0.0,
            points: vec![(0.0, 0.0), (40.0, 1.0), (100.0, 1.0)],
        },
    );

    let mut nlstat1 = LoadCase::linear_static("NLSTAT1", "P", 1.0);
    nlstat1.case_type = LoadCaseType::NonlinearStatic;
    model.load_cases.insert("NLSTAT1".to_string(), nlstat1);

    let mut nlstat2 = LoadCase::linear_static("NLSTAT2", "V", 1.0);
    nlstat2.case_type = LoadCaseType::NonlinearStatic;
    nlstat2.initial_case = Some("NLSTAT1".to_string());
    model.load_cases.insert("NLSTAT2".to_string(), nlstat2);

    let w1 = 2.0 * std::f64::consts::PI / 1.0;
    let w2 = 2.0 * std::f64::consts::PI / 3.4;
    let zeta = 0.999;
    let am = 2.0 * zeta * w1 * w2 / (w1 + w2);
    let bk = 2.0 * zeta / (w1 + w2);

    let mut nldhist1 = LoadCase::linear_static("NLDHIST1", "P", 1.0);
    // The vendor case is a direct-integration history (NLDHIST).
    nldhist1.case_type = LoadCaseType::NonlinearDynamic;
    nldhist1.loads.clear();
    nldhist1.history_loads = vec![("RAMP20".to_string(), "P".to_string(), 1.0)];
    nldhist1.num_time_steps = 400;
    nldhist1.time_step_dt = 0.1;
    nldhist1.rayleigh_damping = (am, bk);
    nldhist1.max_iterations = 100;
    nldhist1.convergence_tol = 1e-6;
    model
        .load_cases
        .insert("NLDHIST1".to_string(), nldhist1.clone());

    let mut nlstat3 = LoadCase::linear_static("NLSTAT3", "V", 1.0);
    nlstat3.case_type = LoadCaseType::NonlinearStatic;
    nlstat3.initial_case = Some("NLDHIST1".to_string());
    model.load_cases.insert("NLSTAT3".to_string(), nlstat3);

    let mut nldhist2 = nldhist1.clone();
    nldhist2.name = "NLDHIST2".to_string();
    nldhist2.initial_case = Some("NLDHIST1".to_string());
    nldhist2.history_loads = vec![("RAMP40".to_string(), "V".to_string(), 1.0)];
    nldhist2.num_time_steps = 800;
    model.load_cases.insert("NLDHIST2".to_string(), nldhist2);

    analyze_model(&mut model).expect("Ex 6-003 analysis failed");
    let f_nlstat1 = model.case_results["NLSTAT1"].link_forces["1"][0];
    let f_nldhist1 = model.case_results["NLDHIST1"].link_forces["1"][0];
    let uz_nlstat2 = model.case_results["NLSTAT2"].joint_displacements["2"][2];
    let uz_nlstat3 = model.case_results["NLSTAT3"].joint_displacements["2"][2];
    let uz_nldhist2 = model.case_results["NLDHIST2"].time_history_disp["2"]
        .last()
        .expect("NLDHIST2 terminal step")[2];
    vec![
        BenchmarkVerificationRow::new(
            "Ex 6-003 (Gap Static)",
            "NLSTAT1",
            "P (Gap 1) [kips]",
            -4.53378,
            f_nlstat1,
            0.2,
        ),
        BenchmarkVerificationRow::new(
            "Ex 6-003 (Gap History)",
            "NLDHIST1",
            "P (Gap 1) [kips]",
            -4.53378,
            f_nldhist1,
            0.2,
        ),
        BenchmarkVerificationRow::new(
            "Ex 6-003 (Gap Uplift)",
            "NLSTAT2",
            "Uz (Jt 2) [in]",
            3.9169,
            uz_nlstat2,
            0.2,
        ),
        BenchmarkVerificationRow::new(
            "Ex 6-003 (Hist->Static)",
            "NLSTAT3",
            "Uz (Jt 2) [in]",
            3.9169,
            uz_nlstat3,
            0.2,
        ),
        BenchmarkVerificationRow::new(
            "Ex 6-003 (Hist->Hist)",
            "NLDHIST2",
            "Uz (Jt 2) [in]",
            3.9169,
            uz_nldhist2,
            0.2,
        ),
    ]
}

/// Example 6-008: Plastic Wen link pushed to +10 in and back to -10 in under
/// displacement-controlled nonlinear static analysis (kip, in).
///
/// The published model loads the link by its own weight as a link gravity
/// load and orients it with advanced link axes. Neither assignment is modeled,
/// so this is an equivalent single-joint model: the same link property on a
/// grounded link along global Z, pushed by a unit joint force that displacement
/// control scales. The compared quantity is the published link force at each
/// link deformation, which depends only on the link property.
pub fn verify_example_6_008() -> Vec<BenchmarkVerificationRow> {
    use sap_model::{DisplacementControl, LinkObj, LinkProp, LinkPropType};

    // (case label, push of NLSTAT1, push back of NLSTAT2, deformation, published force)
    let points: [(&str, f64, Option<f64>, f64, f64); 6] = [
        ("NLSTAT1", 0.2, None, 0.2, 16.836),
        ("NLSTAT1", 0.8, None, 0.8, 43.915),
        ("NLSTAT1", 9.6, None, 9.6, 141.000),
        ("NLSTAT2", 10.0, Some(0.4), 9.6, 105.000),
        ("NLSTAT2", 10.0, Some(1.4), 8.6, 48.438),
        ("NLSTAT2", 10.0, Some(19.6), -9.6, -141.000),
    ];
    let mut rows = Vec::new();
    for (label, push, back, deformation, published) in points {
        let mut model = SapModel::new(Units::KipInF);
        model.load_cases.clear();
        model.load_patterns.clear();
        model.load_patterns.insert(
            "PUSH".into(),
            LoadPattern {
                name: "PUSH".into(),
                pat_type: LoadPatternType::Other,
                self_wt_multiplier: 0.0,
            },
        );
        model.active_dofs = [false, false, true, false, false, false];
        model.add_point("1", 0.0, 0.0, 0.0);
        model.points["1"].loads_force.push(PointLoadForce {
            load_pat: "PUSH".into(),
            csys: "GLOBAL".into(),
            force: [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        });
        let mut prop = LinkProp {
            name: "PLAW1".into(),
            link_type: LinkPropType::PlasticWen,
            ..LinkProp::default()
        };
        prop.ke = [0.0; 6];
        prop.dof_nonlinear[0] = true;
        prop.k_nl[0] = 100.0;
        prop.param1[0] = 50.0;
        prop.param2[0] = 0.1;
        prop.param3[0] = 1.0;
        model.link_props.insert("PLAW1".into(), prop);
        model.links.insert(
            "1".into(),
            LinkObj {
                name: "1".into(),
                point_i: "1".into(),
                point_j: None,
                prop: "PLAW1".into(),
                angle_deg: 0.0,
                selected: false,
            },
        );
        let case = |name: &str, scale: f64, target: f64, initial: Option<&str>| {
            let mut lc = LoadCase::linear_static(name, "PUSH", scale);
            lc.case_type = LoadCaseType::NonlinearStatic;
            lc.max_iterations = 100;
            lc.convergence_tol = 1e-10;
            lc.initial_case = initial.map(str::to_owned);
            lc.displacement_control = Some(DisplacementControl {
                target,
                monitored: Some(("1".into(), 2)),
                conjugate: true,
            });
            lc
        };
        model
            .load_cases
            .insert("NLSTAT1".into(), case("NLSTAT1", 1.0, push, None));
        if let Some(back) = back {
            model.load_cases.insert(
                "NLSTAT2".into(),
                case("NLSTAT2", -1.0, back, Some("NLSTAT1")),
            );
        }
        let computed = match analyze_model(&mut model) {
            Ok(()) => model.case_results[label].link_forces["1"][0],
            Err(_) => f64::NAN,
        };
        rows.push(BenchmarkVerificationRow::new(
            "Ex 6-008 (Plastic Wen)",
            label,
            &format!("Link force at d = {deformation} in [kips]"),
            published,
            computed,
            0.01,
        ));
    }
    rows
}

/// Example 6-005: Maxwell Viscoelastic Damper Under Harmonic Sinusoidal Excitation.
pub fn verify_example_6_005() -> Vec<BenchmarkVerificationRow> {
    use sap_model::{DynamicFunction, LinkObj, LinkProp, LinkPropType};
    let mut model = SapModel::new(Units::KipInF);
    model.load_patterns.clear();
    model.load_cases.clear();

    model.add_point("1", 0.0, 0.0, 0.0);
    model.points.get_mut("1").unwrap().restraint = [true, true, false, true, true, true];
    model.points.get_mut("1").unwrap().spring[2] = 1.0;

    let mut lp = LinkProp {
        name: "DAMP1".into(),
        link_type: LinkPropType::Damper,
        dof_active: [true, false, false, false, false, false],
        dof_nonlinear: [true, false, false, false, false, false],
        nonlinear_data_complete: true,
        ..Default::default()
    };
    lp.ke[0] = 0.0;
    lp.k_nl[0] = 1.0;
    lp.param1[0] = 1.0; // c = 1.0 kip-s/in
    lp.param2[0] = 1.0; // exp = 1.0 (linear Maxwell damper)
    model.link_props.insert("DAMP1".into(), lp);
    model.links.insert(
        "1".into(),
        LinkObj {
            name: "1".into(),
            point_i: "1".into(),
            point_j: None,
            prop: "DAMP1".into(),
            angle_deg: 0.0,
            selected: false,
        },
    );

    model.load_patterns.insert(
        "P".into(),
        LoadPattern {
            name: "P".into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
    model
        .points
        .get_mut("1")
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: "P".into(),
            csys: "GLOBAL".into(),
            force: [0.0, 0.0, 2.0_f64.sqrt(), 0.0, 0.0, 0.0],
        });
    let pts: Vec<(f64, f64)> = (0..=1100)
        .map(|i| {
            let t = i as f64 * 0.01;
            (t, (-t).sin())
        })
        .collect();
    model.functions.insert(
        "SINE".into(),
        DynamicFunction {
            name: "SINE".into(),
            is_time_history: true,
            damping_ratio: 0.0,
            points: pts,
        },
    );
    let mut lc = LoadCase::linear_static("NLDHIST1", "P", 1.0);
    // The vendor case is a direct-integration history (NLDHIST).
    lc.case_type = LoadCaseType::NonlinearDynamic;
    lc.loads.clear();
    lc.history_loads = vec![("SINE".into(), "P".into(), 1.0)];
    lc.num_time_steps = 1100;
    lc.time_step_dt = 0.01;
    model.load_cases.insert("NLDHIST1".into(), lc);
    analyze_model(&mut model).expect("Ex 6-005 analysis failed");
    let trace = &model.case_results["NLDHIST1"].time_history_disp["1"];
    vec![
        BenchmarkVerificationRow::new(
            "Ex 6-005 (Maxwell Damper)",
            "NLDHIST1 (t=5.0s)",
            "Uz (Jt 1) [in]",
            0.87195,
            trace[500][2],
            0.1,
        ),
        BenchmarkVerificationRow::new(
            "Ex 6-005 (Maxwell Damper)",
            "NLDHIST1 (t=11.0s)",
            "Uz (Jt 1) [in]",
            0.84970,
            trace[1100][2],
            0.1,
        ),
    ]
}

/// Example 7-001: 3D Elastic Catenary Cables (Peyrot & Goulois 1979).
pub fn verify_example_7_001() -> Vec<BenchmarkVerificationRow> {
    use crate::cable::{evaluate_cable_element, CableElement};
    let c1_elem = CableElement::from_undeformed_length(
        [0.0, 0.0, 90.0],
        [100.0, 0.0, 30.0],
        1.0,
        30.0e6,
        6.5e-6,
        100.0,
        [0.0, 0.0, -1.0],
        100.0,
    )
    .expect("Ex 7-001 C1");
    let c1 = evaluate_cable_element(&c1_elem, [0.0; 3], [0.0; 3]).expect("Ex 7-001 C1 state");

    let c2_elem = CableElement::from_undeformed_length(
        [0.0, 0.0, 90.0],
        [80.0, 0.0, 30.0],
        1.0,
        30.0e6,
        6.5e-6,
        100.0,
        [0.0, 0.0, -1.0],
        100.0,
    )
    .expect("Ex 7-001 C2");
    let c2 = evaluate_cable_element(&c2_elem, [0.0; 3], [0.0; 3]).expect("Ex 7-001 C2 state");

    let c6_elem = CableElement::from_undeformed_length(
        [0.0, 0.0, 90.0],
        [0.0, 0.0, 30.0],
        1.0,
        30.0e6,
        6.5e-6,
        100.0,
        [0.0, 0.0, -1.0],
        100.0,
    )
    .expect("Ex 7-001 C6");
    let c6 = evaluate_cable_element(&c6_elem, [0.0; 3], [0.0; 3]).expect("Ex 7-001 C6 state");

    vec![
        BenchmarkVerificationRow::new(
            "Ex 7-001 (Cable C1)",
            "DEADTEMP",
            "Fx (Jt 2) [lb]",
            4_258_491.0,
            c1.f_j[0],
            0.1,
        ),
        BenchmarkVerificationRow::new(
            "Ex 7-001 (Cable C1)",
            "DEADTEMP",
            "Fz (Jt 2) [lb]",
            -2_555_045.0,
            c1.f_j[2],
            0.1,
        ),
        BenchmarkVerificationRow::new(
            "Ex 7-001 (Cable C2)",
            "DISP1",
            "Fx (Jt 2) [lb]",
            504.10,
            c2.f_j[0],
            0.1,
        ),
        BenchmarkVerificationRow::new(
            "Ex 7-001 (Cable C6)",
            "DISP5",
            "Fz (Jt 2) [lb]",
            20.02,
            c6.f_j[2],
            0.1,
        ),
    ]
}

/// Example 7-002: Two-Segment Catenary Cable Under Concentrated Point Load (Tibert 1999).
pub fn verify_example_7_002() -> Vec<BenchmarkVerificationRow> {
    use crate::cable::{solve_cable_network, CableElement};
    let x1 = [0.0, 0.0, 0.0];
    let x2_init = [400.0, 0.0, -150.0];
    let x3 = [1000.0, 0.0, 0.0];
    let area = 0.85;
    let e = 19_000.0;
    let w = 0.00316;
    let seg_12 = CableElement::from_undeformed_length(
        x1,
        x2_init,
        area,
        e,
        0.0,
        412.8837,
        [0.0, 0.0, -w],
        0.0,
    )
    .expect("seg_12");
    let seg_23 = CableElement::from_undeformed_length(
        x2_init,
        x3,
        area,
        e,
        0.0,
        613.0422,
        [0.0, 0.0, -w],
        0.0,
    )
    .expect("seg_23");
    let elements = vec![(0, 1, seg_12), (1, 2, seg_23)];
    let restraints = [[true, true, true], [false, true, false], [true, true, true]];
    let loads_sw = [[0.0; 3], [0.0, 0.0, 0.0], [0.0; 3]];
    let (u_sw, _) =
        solve_cable_network(3, &elements, &restraints, &loads_sw, None, 60, 1e-10).expect("sw");
    let loads_pt = [[0.0; 3], [0.0, 0.0, -8.0], [0.0; 3]];
    let (u_pt, _) =
        solve_cable_network(3, &elements, &restraints, &loads_pt, Some(&u_sw), 60, 1e-10)
            .expect("pt");
    let disp_ux = u_pt[1][0] - u_sw[1][0];
    let disp_uz = u_pt[1][2] - u_sw[1][2];
    vec![
        BenchmarkVerificationRow::new(
            "Ex 7-002 (2-Seg Cable)",
            "DEAD+PTLOAD",
            "Ux (Jt 2) [ft]",
            -2.819,
            disp_ux,
            0.1,
        ),
        BenchmarkVerificationRow::new(
            "Ex 7-002 (2-Seg Cable)",
            "DEAD+PTLOAD",
            "Uz (Jt 2) [ft]",
            -18.457,
            disp_uz,
            0.1,
        ),
    ]
}

/// Example 7-003: 3D Prestressed 3x3 Cable Net Initialized via Target I-End Tension (Tibert 1999).
pub fn verify_example_7_003() -> Vec<BenchmarkVerificationRow> {
    use sap_model::CableParams;
    let mut net = SapModel::new(Units::KipFtF);
    net.load_patterns.clear();
    net.load_cases.clear();
    net.load_patterns.insert(
        "SELF".into(),
        LoadPattern {
            name: "SELF".into(),
            pat_type: LoadPatternType::Dead,
            self_wt_multiplier: 1.0,
        },
    );
    net.load_patterns.insert(
        "POINT".into(),
        LoadPattern {
            name: "POINT".into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
    let mut mat = Material::default_steel_kip_in();
    mat.name = "CABLE_MAT".into();
    mat.e = 12_000.0 * 144.0;
    mat.a = 0.0;
    let area_ft2 = 0.227 / 144.0;
    mat.unit_weight = 0.0001 / area_ft2;
    mat.unit_mass = 0.0;
    net.materials.insert("CABLE_MAT".into(), mat);
    net.frame_sections.insert(
        "CAB_SEC".into(),
        FrameSection {
            name: "CAB_SEC".into(),
            material: "CABLE_MAT".into(),
            area: area_ft2,
            ..FrameSection::default()
        },
    );

    net.add_point("1", 100.0, 100.0, 10.0);
    net.add_point("2", 200.0, 100.0, 10.0);
    net.add_point("3", 100.0, 200.0, 10.0);
    net.add_point("4", 200.0, 200.0, 10.0);
    for (sname, x, y) in [
        ("5", 100.0, 0.0),
        ("6", 200.0, 0.0),
        ("7", 0.0, 100.0),
        ("8", 300.0, 100.0),
        ("9", 0.0, 200.0),
        ("10", 300.0, 200.0),
        ("11", 100.0, 300.0),
        ("12", 200.0, 300.0),
    ] {
        net.add_point(sname, x, y, 40.0);
        net.points.get_mut(sname).unwrap().restraint = [true; 6];
    }
    for j in ["1", "2", "3", "4"] {
        net.points
            .get_mut(j)
            .unwrap()
            .loads_force
            .push(PointLoadForce {
                load_pat: "POINT".into(),
                csys: "GLOBAL".into(),
                force: [0.0, 0.0, -8.0, 0.0, 0.0, 0.0],
            });
    }
    for (cname, pi, pj) in [
        ("1", "7", "1"),
        ("4", "9", "3"),
        ("9", "5", "1"),
        ("12", "6", "2"),
        ("3", "8", "2"),
        ("6", "10", "4"),
        ("10", "4", "12"),
        ("7", "3", "11"),
    ] {
        net.add_frame(cname, pi, pj, "CAB_SEC");
        let f = net.frames.get_mut(cname).unwrap();
        f.tc_limits = Some((true, 0.0, false, 0.0));
        f.cable_params = Some(CableParams {
            undeformed_length: None,
            target_tension_i: Some(5.325),
            horizontal_tension: None,
            num_segments: 1,
        });
    }
    for (cname, pi, pj) in [
        ("2", "1", "2"),
        ("5", "3", "4"),
        ("11", "2", "4"),
        ("8", "3", "1"),
    ] {
        net.add_frame(cname, pi, pj, "CAB_SEC");
        let f = net.frames.get_mut(cname).unwrap();
        f.tc_limits = Some((true, 0.0, false, 0.0));
        f.cable_params = Some(CableParams {
            undeformed_length: None,
            target_tension_i: Some(5.459),
            horizontal_tension: None,
            num_segments: 1,
        });
    }

    let mut lc_self = LoadCase::linear_static("SELF", "SELF", 1.0);
    lc_self.case_type = LoadCaseType::NonlinearStatic;
    net.load_cases.insert("SELF".into(), lc_self);

    let mut lc_pt = LoadCase::linear_static("POINT", "POINT", 1.0);
    lc_pt.case_type = LoadCaseType::NonlinearStatic;
    lc_pt.initial_case = Some("SELF".into());
    net.load_cases.insert("POINT".into(), lc_pt);

    analyze_model(&mut net).expect("Ex 7-003 analysis failed");
    let u1 = net.case_results["POINT"].joint_displacements["1"];
    vec![
        BenchmarkVerificationRow::new(
            "Ex 7-003 (3D Cable Net)",
            "POINT",
            "Ux (Jt 1) [ft]",
            -0.1328,
            u1[0],
            0.2,
        ),
        BenchmarkVerificationRow::new(
            "Ex 7-003 (3D Cable Net)",
            "POINT",
            "Uy (Jt 1) [ft]",
            -0.1328,
            u1[1],
            0.2,
        ),
        BenchmarkVerificationRow::new(
            "Ex 7-003 (3D Cable Net)",
            "POINT",
            "Uz (Jt 1) [ft]",
            -1.4745,
            u1[2],
            0.2,
        ),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use sap_model::{AreaLoadUniform, AreaSection, DynamicFunction, ModalComboMethod, ShellType};

    #[test]
    fn test_official_sap2000_verification_suite() {
        let rows = run_verification_suite();
        assert!(!rows.is_empty());
        let failures: Vec<String> = rows
            .iter()
            .filter(|r| !r.passed)
            .map(|row| {
                format!(
                    "Benchmark {} [{}] {} FAILED: computed = {}, expected = {} (diff = {:.4}%)",
                    row.example,
                    row.load_case,
                    row.parameter,
                    row.computed,
                    row.sap2000_ref,
                    row.pct_diff
                )
            })
            .collect();
        assert!(
            failures.is_empty(),
            "{} of {} benchmark rows failed:\n{}",
            failures.len(),
            rows.len(),
            failures.join("\n")
        );
    }

    #[test]
    fn test_vendor_sdb_verification_examples() {
        use std::path::Path;
        let dir = Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../extracted/files"
        ));
        if !dir.exists() {
            return;
        }
        let mut rows = Vec::new();

        // Example 1-001 from vendor .sdb
        let p_001 = dir.join("example_1_001.sdb");
        if p_001.exists() {
            let mut m = sap_format::load_sdb_file(&p_001).expect("load example_1_001.sdb");
            analyze_model(&mut m).expect("solve example_1_001.sdb");
            for (lc, alt, pt, dof, label, ref_val) in [
                ("LCASE1", "1", "3", 2, "Uz (Jt 3) [in]", -0.02639),
                ("LCASE2", "2", "3", 2, "Uz (Jt 3) [in]", 0.06296),
                ("LCASE3", "3", "3", 2, "Uz (Jt 3) [in]", 0.06296),
                ("LCASE4", "4", "3", 2, "Uz (Jt 3) [in]", -0.29630),
                ("LCASE5", "5", "2", 0, "Ux (Jt 2) [in]", 0.31250),
                ("LCASE6", "6", "2", 0, "Ux (Jt 2) [in]", 0.11556),
                ("LCASE7", "7", "2", 0, "Ux (Jt 2) [in]", 0.00651),
            ] {
                if let Some(res) = m.case_results.get(lc).or_else(|| m.case_results.get(alt)) {
                    rows.push(BenchmarkVerificationRow::new(
                        "Vendor SDB Ex 1-001",
                        lc,
                        label,
                        ref_val,
                        res.joint_displacements[pt][dof],
                        0.5,
                    ));
                }
            }
        }

        // Example 1-002a (unpropped cantilever temperature loads) from vendor .sdb
        let p_002a = dir.join("example_1_002a.sdb");
        if p_002a.exists() {
            let mut m = sap_format::load_sdb_file(&p_002a).expect("load example_1_002a.sdb");
            analyze_model(&mut m).expect("solve example_1_002a.sdb");
            for (lc, alt, dof, label, ref_val) in [
                ("LC1", "1", 0, "Ux (Jt 2) [in]", 0.00130),
                ("LC2", "2", 0, "Ux (Jt 2) [in]", 0.00065),
                ("LC3", "3", 2, "Uz (Jt 2) [in]", -0.00650),
            ] {
                if let Some(res) = m.case_results.get(lc).or_else(|| m.case_results.get(alt)) {
                    rows.push(BenchmarkVerificationRow::new(
                        "Vendor SDB Ex 1-002a",
                        lc,
                        label,
                        ref_val,
                        res.joint_displacements["2"][dof],
                        0.5,
                    ));
                }
            }
        }

        // Example 1-002b (propped cantilever temperature loads) from vendor .sdb
        let p_002b = dir.join("example_1_002b.sdb");
        if p_002b.exists() {
            let mut m = sap_format::load_sdb_file(&p_002b).expect("load example_1_002b.sdb");
            analyze_model(&mut m).expect("solve example_1_002b.sdb");
            for (lc, alt, dof, label, ref_val) in [
                ("LC1", "1", 0, "Fx (Jt 2) [kip]", -22.62),
                ("LC2", "2", 0, "Fx (Jt 2) [kip]", -11.31),
                ("LC3", "3", 2, "Fz (Jt 2) [kip]", 2.545),
            ] {
                if let Some(res) = m.case_results.get(lc).or_else(|| m.case_results.get(alt)) {
                    rows.push(BenchmarkVerificationRow::new(
                        "Vendor SDB Ex 1-002b",
                        lc,
                        label,
                        ref_val,
                        res.joint_reactions["2"][dof],
                        0.5,
                    ));
                }
            }
        }

        // Example 1-003 from vendor .sdb
        let p_003 = dir.join("example_1_003.sdb");
        if p_003.exists() {
            let mut m = sap_format::load_sdb_file(&p_003).expect("load example_1_003.sdb");
            analyze_model(&mut m).expect("solve example_1_003.sdb");
            if let Some(res) = m
                .case_results
                .get("LC1")
                .or_else(|| m.case_results.get("1"))
            {
                rows.push(BenchmarkVerificationRow::new(
                    "Vendor SDB Ex 1-003",
                    "LC1",
                    "Mx (Jt 1) [kip-in]",
                    -4.0,
                    res.joint_reactions["1"][3],
                    0.5,
                ));
                rows.push(BenchmarkVerificationRow::new(
                    "Vendor SDB Ex 1-003",
                    "LC1",
                    "Rx (Jt 3) [rad]",
                    0.02375,
                    res.joint_displacements["3"][3],
                    0.5,
                ));
                rows.push(BenchmarkVerificationRow::new(
                    "Vendor SDB Ex 1-003",
                    "LC1",
                    "Rx (Jt 5) [rad]",
                    0.02558,
                    res.joint_displacements["5"][3],
                    0.5,
                ));
            }
            if let Some(res) = m
                .case_results
                .get("LC2")
                .or_else(|| m.case_results.get("2"))
            {
                rows.push(BenchmarkVerificationRow::new(
                    "Vendor SDB Ex 1-003",
                    "LC2",
                    "Mx (Jt 1) [kip-in]",
                    -5.75,
                    res.joint_reactions["1"][3],
                    0.5,
                ));
                rows.push(BenchmarkVerificationRow::new(
                    "Vendor SDB Ex 1-003",
                    "LC2",
                    "Rx (Jt 5) [rad]",
                    0.02969,
                    res.joint_displacements["5"][3],
                    0.5,
                ));
            }
        }

        // Example 1-004 from vendor .sdb
        let p_004 = dir.join("example_1_004.sdb");
        if p_004.exists() {
            let mut m = sap_format::load_sdb_file(&p_004).expect("load example_1_004.sdb");
            analyze_model(&mut m).expect("solve example_1_004.sdb");
            for (lc, alt, ref_uy, ref_uz) in [
                ("LC1", "1", -0.01806, -0.03029),
                ("LC2", "2", -0.03345, -0.05610),
                ("LC3", "3", -0.08361, -0.14024),
            ] {
                if let Some(res) = m.case_results.get(lc).or_else(|| m.case_results.get(alt)) {
                    rows.push(BenchmarkVerificationRow::new(
                        "Vendor SDB Ex 1-004",
                        lc,
                        "Uy (Jt 2) [in]",
                        ref_uy,
                        res.joint_displacements["2"][1],
                        0.5,
                    ));
                    rows.push(BenchmarkVerificationRow::new(
                        "Vendor SDB Ex 1-004",
                        lc,
                        "Uz (Jt 2) [in]",
                        ref_uz,
                        res.joint_displacements["2"][2],
                        0.5,
                    ));
                }
            }
        }

        // Example 1-007a, 1-007b, 1-007c from vendor .sdb
        for (fname, label, ref_fz, ref_my) in [
            ("example_1_007a.sdb", "Vendor SDB Ex 1-007a", 0.0, 50.0),
            ("example_1_007b.sdb", "Vendor SDB Ex 1-007b", 0.0, 50.0),
            ("example_1_007c.sdb", "Vendor SDB Ex 1-007c", 5.0, 0.0),
        ] {
            let p = dir.join(fname);
            if p.exists() {
                let mut m = sap_format::load_sdb_file(&p).expect(fname);
                analyze_model(&mut m).expect(fname);
                if let Some(res) = m
                    .case_results
                    .get("L1")
                    .or_else(|| m.case_results.get("1"))
                    .or_else(|| m.case_results.values().next())
                {
                    rows.push(BenchmarkVerificationRow::new(
                        label,
                        "L1",
                        "Fz (Jt 1) [kips]",
                        ref_fz,
                        res.joint_reactions["1"][2],
                        0.5,
                    ));
                    rows.push(BenchmarkVerificationRow::new(
                        label,
                        "L1",
                        "My (Jt 1) [kip-ft]",
                        ref_my,
                        res.joint_reactions["1"][4],
                        0.5,
                    ));
                }
            }
        }

        assert!(
            rows.len() >= 25,
            "Expected at least 25 vendor SDB benchmark checks, got {}",
            rows.len()
        );
        let failures: Vec<String> = rows
            .iter()
            .filter(|r| !r.passed)
            .map(|row| {
                format!(
                    "{} [{}] {} FAILED: computed = {}, expected = {} (diff = {:.4}%)",
                    row.example,
                    row.load_case,
                    row.parameter,
                    row.computed,
                    row.sap2000_ref,
                    row.pct_diff
                )
            })
            .collect();
        assert!(
            failures.is_empty(),
            "{} of {} vendor SDB benchmark rows failed:\n{}",
            failures.len(),
            rows.len(),
            failures.join("\n")
        );
    }

    #[test]
    fn test_modal_response_spectrum_and_shell() {
        let mut model = SapModel::new(Units::KipInF);
        model.add_point("1", 0.0, 0.0, 0.0);
        model.points.get_mut("1").unwrap().restraint = [true; 6];
        model.add_point("2", 0.0, 0.0, 100.0);
        model.points.get_mut("2").unwrap().restraint = [false, true, true, true, false, true];
        model.points.get_mut("2").unwrap().mass = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0];

        let mut sec = FrameSection::default();
        sec.name = "COL".to_string();
        sec.material = "A992Fy50".to_string();
        sec.area = 20.0;
        sec.i33 = 1000.0;
        sec.as2 = 0.0;
        sec.modifiers[6] = 0.0;
        model.frame_sections.insert("COL".to_string(), sec);
        model.add_frame("1", "1", "2", "COL");

        model.functions.insert(
            "RS1".to_string(),
            DynamicFunction {
                name: "RS1".to_string(),
                is_time_history: false,
                damping_ratio: 0.05,
                points: vec![(0.0, 100.0), (10.0, 100.0)],
            },
        );
        let mut rs_case = LoadCase::modal_eigen("SPEC1", 1);
        rs_case.case_type = LoadCaseType::ResponseSpectrum;
        rs_case.spectrum_loads = vec![("RS1".to_string(), "U1".to_string(), 1.0)];
        rs_case.modal_combo = ModalComboMethod::Cqc;
        model.load_cases.insert("SPEC1".to_string(), rs_case);

        analyze_model(&mut model).expect("Modal + RS analysis failed");
        let modes = &model.case_results["MODAL"].modes;
        assert!(!modes.is_empty());
        let omega = modes[0].circ_freq_rad_s;
        assert!((omega - 870.0_f64.sqrt()).abs() < 1e-4, "omega = {omega}");
        let ux_spec = model.case_results["SPEC1"].joint_displacements["2"][0];
        assert!(
            (ux_spec - (100.0 / 870.0)).abs() < 1e-4,
            "ux_spec = {ux_spec}"
        );

        // Test 4-node Shell Plate under uniform load
        let mut shell_model = SapModel::new(Units::KipInF);
        let mut asec = AreaSection::default();
        asec.name = "SLAB".to_string();
        asec.shell_type = ShellType::ShellThick;
        asec.thickness_membrane = 6.0;
        asec.thickness_bending = 6.0;
        shell_model.area_sections.insert("SLAB".to_string(), asec);

        shell_model.generate_wall(2, 60.0, 2, 60.0, true, "SLAB");
        for area in shell_model.areas.values_mut() {
            area.loads_uniform.push(AreaLoadUniform {
                load_pat: "DEAD".to_string(),
                csys: "GLOBAL".to_string(),
                dir: 5,
                val: 0.01,
            });
        }
        analyze_model(&mut shell_model).expect("Shell analysis failed");
        assert!(!shell_model.case_results["DEAD"].area_results.is_empty());
    }
}
