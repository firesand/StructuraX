//! Empirical Challenger stress-test suite for FMT-01 and FMT-02.
//! FMT-01: Joint local_axes_matrix 3D rotation matrices and boundary conditions.
//! FMT-02: Independent rebar yield strength (fys != fy) across material types.

use sap_format::sdb::native::{inspect_native_sdb, pack_model_to_native_sdb};
use sap_format::{extract_sdb_to_model, parse_s2k, write_s2k, FormatError};
use sap_model::*;

fn base_model() -> SapModel {
    let mut m = SapModel::default();
    m.add_point("A", 0.0, 0.0, 0.0);
    m.add_point("B", 10.0, 0.0, 0.0);
    m.add_frame("F1", "A", "B", "FSEC1");
    m
}

fn assert_unsupported(m: &SapModel, expected_err_substring: &str) {
    let err = pack_model_to_native_sdb(m).expect_err("expected export to fail");
    match err {
        FormatError::UnsupportedNativeExport { field, reason } => {
            let combined = format!("{field}: {reason}");
            assert!(
                combined.contains(expected_err_substring)
                    || field.contains(expected_err_substring)
                    || reason.contains(expected_err_substring),
                "Expected error substring '{expected_err_substring}', but got '{combined}'"
            );
        }
        other => panic!("Expected UnsupportedNativeExport, got: {other:?}"),
    }
}

/// Generate a 3x3 rotation matrix from Euler angles (ZYX order: yaw, pitch, roll in radians).
fn euler_zyx_to_matrix(yaw: f64, pitch: f64, roll: f64) -> [[f64; 3]; 3] {
    let (cy, sy) = (yaw.cos(), yaw.sin());
    let (cp, sp) = (pitch.cos(), pitch.sin());
    let (cr, sr) = (roll.cos(), roll.sin());

    [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
}

/// Generate a 3x3 rotation matrix using Rodrigues formula for rotation by angle theta around unit vector (ux, uy, uz).
fn rodrigues_to_matrix(axis: [f64; 3], theta: f64) -> [[f64; 3]; 3] {
    let norm = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
    let (ux, uy, uz) = (axis[0] / norm, axis[1] / norm, axis[2] / norm);
    let (c, s) = (theta.cos(), theta.sin());
    let omc = 1.0 - c;

    [
        [
            c + ux * ux * omc,
            ux * uy * omc - uz * s,
            ux * uz * omc + uy * s,
        ],
        [
            uy * ux * omc + uz * s,
            c + uy * uy * omc,
            uy * uz * omc - ux * s,
        ],
        [
            uz * ux * omc - uy * s,
            uz * uy * omc + ux * s,
            c + uz * uz * omc,
        ],
    ]
}

/// Check that two 3x3 matrices are equal to within machine precision.
fn assert_matrix_close(a: [[f64; 3]; 3], b: [[f64; 3]; 3], tol: f64) {
    for r in 0..3 {
        for c in 0..3 {
            let diff = (a[r][c] - b[r][c]).abs();
            assert!(
                diff <= tol,
                "Matrix mismatch at [{r}][{c}]: a={:e}, b={:e}, diff={:e} > tol={:e}",
                a[r][c],
                b[r][c],
                diff,
                tol
            );
        }
    }
}

// =========================================================================
// FMT-01: Joint local_axes_matrix Stress Tests
// =========================================================================

#[test]
fn test_fmt01_identity_matrix() {
    let mut m = base_model();
    let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    m.points.get_mut("A").unwrap().local_axes_matrix = Some(identity);

    let native = pack_model_to_native_sdb(&m).expect("native export must succeed");
    let doc = inspect_native_sdb(&native).unwrap();
    assert_eq!(doc.inventory().parsed_bytes, doc.inventory().stream_bytes);

    let restored = extract_sdb_to_model(&native).expect("native import must succeed");
    assert_eq!(restored.points["A"].local_axes_matrix, Some(identity));
    assert_eq!(restored.points["B"].local_axes_matrix, None);
    assert_eq!(restored, m);
}

#[test]
fn test_fmt01_principal_90deg_rotations() {
    let pi_2 = std::f64::consts::FRAC_PI_2;
    let cases = [
        ("rot_x_90", rodrigues_to_matrix([1.0, 0.0, 0.0], pi_2)),
        ("rot_y_90", rodrigues_to_matrix([0.0, 1.0, 0.0], pi_2)),
        ("rot_z_90", rodrigues_to_matrix([0.0, 0.0, 1.0], pi_2)),
        (
            "rot_x_180",
            rodrigues_to_matrix([1.0, 0.0, 0.0], std::f64::consts::PI),
        ),
        (
            "rot_y_270",
            rodrigues_to_matrix([0.0, 1.0, 0.0], 3.0 * pi_2),
        ),
    ];

    for (name, rot) in cases {
        let mut m = base_model();
        m.points.get_mut("A").unwrap().local_axes_matrix = Some(rot);

        let native = pack_model_to_native_sdb(&m)
            .unwrap_or_else(|e| panic!("native export for {name} failed: {e}"));
        let restored = extract_sdb_to_model(&native)
            .unwrap_or_else(|e| panic!("native import for {name} failed: {e}"));

        let restored_mat = restored.points["A"].local_axes_matrix.unwrap();
        assert_matrix_close(restored_mat, rot, 1e-12);
        assert_eq!(restored, m);
    }
}

#[test]
fn test_fmt01_arbitrary_euler_angles_sweep() {
    // Test a sweep of Euler angles including non-cardinal, negative, and large angles
    let angles = [
        (0.123, -0.456, 0.789),
        (-1.234, 0.567, -2.345),
        (0.001, -0.002, 0.003),
        (2.5, -1.1, 0.8),
        (
            std::f64::consts::PI / 3.0,
            -std::f64::consts::PI / 6.0,
            std::f64::consts::PI / 4.0,
        ),
    ];

    for (yaw, pitch, roll) in angles {
        let rot = euler_zyx_to_matrix(yaw, pitch, roll);
        let mut m = base_model();
        m.points.get_mut("A").unwrap().local_axes_matrix = Some(rot);

        let native = pack_model_to_native_sdb(&m).expect("native export must succeed");
        let restored = extract_sdb_to_model(&native).expect("native import must succeed");

        let restored_mat = restored.points["A"].local_axes_matrix.unwrap();
        assert_matrix_close(restored_mat, rot, 1e-12);
        assert_eq!(restored, m);
        assert_eq!(parse_s2k(&write_s2k(&restored)).unwrap(), m);
    }
}

#[test]
fn test_fmt01_rodrigues_arbitrary_skew_axes() {
    // Arbitrary skew vectors in 3D
    let test_axes = [
        ([1.0, 2.0, 3.0], 0.77),
        ([-3.5, 2.1, -1.8], 2.14),
        ([0.001, 100.0, -50.0], 1.2345),
        ([1.0, 1.0, 1.0], 2.0 * std::f64::consts::PI / 3.0), // 120-degree permutation
    ];

    for (axis, theta) in test_axes {
        let rot = rodrigues_to_matrix(axis, theta);
        let mut m = base_model();
        m.points.get_mut("B").unwrap().local_axes_matrix = Some(rot);

        let native = pack_model_to_native_sdb(&m).expect("native export must succeed");
        let restored = extract_sdb_to_model(&native).expect("native import must succeed");

        let restored_mat = restored.points["B"].local_axes_matrix.unwrap();
        assert_matrix_close(restored_mat, rot, 1e-12);
        assert_eq!(restored, m);
    }
}

#[test]
fn test_fmt01_batch_50_points_with_distinct_matrices() {
    let mut m = SapModel::default();
    let count = 50;
    for i in 0..count {
        let name = format!("P{i}");
        m.add_point(&name, i as f64 * 2.0, 0.0, 0.0);
        let angle = (i as f64 + 1.0) * 0.13;
        let axis = [
            (i as f64 * 0.1).sin() + 1.0,
            (i as f64 * 0.2).cos(),
            (i as f64 * 0.3).sin() + 0.5,
        ];
        let mat = rodrigues_to_matrix(axis, angle);
        m.points.get_mut(&name).unwrap().local_axes_matrix = Some(mat);
    }
    m.add_frame("F1", "P0", "P1", "FSEC1");

    let native = pack_model_to_native_sdb(&m).expect("batch export must succeed");
    let doc = inspect_native_sdb(&native).unwrap();
    assert_eq!(doc.inventory().parsed_bytes, doc.inventory().stream_bytes);

    let restored = extract_sdb_to_model(&native).expect("batch import must succeed");
    assert_eq!(restored.points.len(), count);
    for i in 0..count {
        let name = format!("P{i}");
        let orig_mat = m.points[&name].local_axes_matrix.unwrap();
        let rest_mat = restored.points[&name].local_axes_matrix.unwrap();
        assert_matrix_close(rest_mat, orig_mat, 1e-12);
    }
    assert_eq!(restored, m);
}

#[test]
fn test_fmt01_mixed_points_some_and_none() {
    let mut m = SapModel::default();
    m.add_point("P_NONE_1", 0.0, 0.0, 0.0);
    m.add_point("P_SOME_1", 1.0, 0.0, 0.0);
    m.add_point("P_NONE_2", 2.0, 0.0, 0.0);
    m.add_point("P_SOME_2", 3.0, 0.0, 0.0);
    m.add_point("P_NONE_3", 4.0, 0.0, 0.0);
    m.add_frame("F1", "P_NONE_1", "P_SOME_1", "FSEC1");

    let mat1 = euler_zyx_to_matrix(0.5, 0.2, -0.3);
    let mat2 = euler_zyx_to_matrix(-0.8, 0.4, 1.1);
    m.points.get_mut("P_SOME_1").unwrap().local_axes_matrix = Some(mat1);
    m.points.get_mut("P_SOME_2").unwrap().local_axes_matrix = Some(mat2);

    let native = pack_model_to_native_sdb(&m).expect("mixed export must succeed");
    let restored = extract_sdb_to_model(&native).expect("mixed import must succeed");

    assert_eq!(restored.points["P_NONE_1"].local_axes_matrix, None);
    assert_eq!(restored.points["P_SOME_1"].local_axes_matrix, Some(mat1));
    assert_eq!(restored.points["P_NONE_2"].local_axes_matrix, None);
    assert_eq!(restored.points["P_SOME_2"].local_axes_matrix, Some(mat2));
    assert_eq!(restored.points["P_NONE_3"].local_axes_matrix, None);
    assert_eq!(restored, m);
}

#[test]
fn test_fmt01_coexistence_with_local_axes_deg() {
    let mut m = base_model();
    let mat = rodrigues_to_matrix([0.0, 1.0, 0.0], 0.5);
    let pt = m.points.get_mut("A").unwrap();
    pt.local_axes_deg = [15.0, 30.0, 45.0];
    pt.local_axes_matrix = Some(mat);

    let native = pack_model_to_native_sdb(&m).expect("export must succeed");
    let restored = extract_sdb_to_model(&native).expect("import must succeed");

    assert_eq!(restored.points["A"].local_axes_deg, [15.0, 30.0, 45.0]);
    assert_matrix_close(restored.points["A"].local_axes_matrix.unwrap(), mat, 1e-12);
    assert_eq!(restored, m);
}

#[test]
fn test_fmt01_non_orthogonal_matrix_roundtrips() {
    // Non-orthogonal matrix (scaled shear) with finite values
    let mut m = base_model();
    let shear_mat = [[2.0, 0.5, 0.0], [0.0, 1.5, 0.2], [0.1, 0.0, 3.0]];
    m.points.get_mut("A").unwrap().local_axes_matrix = Some(shear_mat);

    let native = pack_model_to_native_sdb(&m).expect("non-orthogonal matrix export must succeed");
    let restored =
        extract_sdb_to_model(&native).expect("non-orthogonal matrix import must succeed");
    assert_matrix_close(
        restored.points["A"].local_axes_matrix.unwrap(),
        shear_mat,
        1e-12,
    );
    assert_eq!(restored, m);
}

#[test]
fn test_fmt01_singular_zero_matrix_roundtrips() {
    // All-zero singular matrix
    let mut m = base_model();
    let zero_mat = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]];
    m.points.get_mut("A").unwrap().local_axes_matrix = Some(zero_mat);

    let native = pack_model_to_native_sdb(&m).expect("zero matrix export must succeed");
    let restored = extract_sdb_to_model(&native).expect("zero matrix import must succeed");
    assert_eq!(restored.points["A"].local_axes_matrix, Some(zero_mat));
    assert_eq!(restored, m);
}

#[test]
fn test_fmt01_rejects_nan_in_matrix() {
    let mut m = base_model();
    let mut mat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    mat[1][2] = f64::NAN;
    m.points.get_mut("A").unwrap().local_axes_matrix = Some(mat);

    assert_unsupported(&m, "points.A.local_axes_matrix");
}

#[test]
fn test_fmt01_rejects_positive_infinity_in_matrix() {
    let mut m = base_model();
    let mut mat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    mat[0][0] = f64::INFINITY;
    m.points.get_mut("A").unwrap().local_axes_matrix = Some(mat);

    assert_unsupported(&m, "points.A.local_axes_matrix");
}

#[test]
fn test_fmt01_rejects_negative_infinity_in_matrix() {
    let mut m = base_model();
    let mut mat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    mat[2][1] = f64::NEG_INFINITY;
    m.points.get_mut("A").unwrap().local_axes_matrix = Some(mat);

    assert_unsupported(&m, "points.A.local_axes_matrix");
}

// =========================================================================
// FMT-02: Independent Shear Rebar Yield (fys != fy) Stress Tests
// =========================================================================

#[test]
fn test_fmt02_concrete_material_large_fys() {
    let mut m = base_model();
    let conc = m.materials.get_mut("4000Psi").unwrap();
    conc.fy = 60_000.0;
    conc.fys = 120_000.0;

    let native = pack_model_to_native_sdb(&m).expect("large fys export must succeed");
    let restored = extract_sdb_to_model(&native).expect("large fys import must succeed");
    assert_eq!(restored.materials["4000Psi"].fy, 60_000.0);
    assert_eq!(restored.materials["4000Psi"].fys, 120_000.0);
    assert_eq!(restored, m);
}

#[test]
fn test_fmt02_concrete_material_smaller_fys() {
    let mut m = base_model();
    let conc = m.materials.get_mut("4000Psi").unwrap();
    conc.fy = 60.0;
    conc.fys = 40.0;

    let native = pack_model_to_native_sdb(&m).expect("smaller fys export must succeed");
    let restored = extract_sdb_to_model(&native).expect("smaller fys import must succeed");
    assert_eq!(restored.materials["4000Psi"].fy, 60.0);
    assert_eq!(restored.materials["4000Psi"].fys, 40.0);
    assert_eq!(restored, m);
}

#[test]
fn test_fmt02_concrete_material_fractional_fys() {
    let mut m = base_model();
    let conc = m.materials.get_mut("4000Psi").unwrap();
    conc.fy = 60.123456789;
    conc.fys = 72.987654321;

    let native = pack_model_to_native_sdb(&m).expect("fractional fys export must succeed");
    let restored = extract_sdb_to_model(&native).expect("fractional fys import must succeed");
    assert_eq!(restored.materials["4000Psi"].fy, 60.123456789);
    assert_eq!(restored.materials["4000Psi"].fys, 72.987654321);
    assert_eq!(restored, m);
}

#[test]
fn test_fmt02_steel_material_independent_fys() {
    let mut m = base_model();
    let steel = m.materials.get_mut("A992Fy50").unwrap();
    steel.fy = 50.0;
    steel.fys = 36.0;

    let native = pack_model_to_native_sdb(&m).expect("steel independent fys export must succeed");
    let restored =
        extract_sdb_to_model(&native).expect("steel independent fys import must succeed");
    assert_eq!(restored.materials["A992Fy50"].fy, 50.0);
    assert_eq!(restored.materials["A992Fy50"].fys, 36.0);
    assert_eq!(restored, m);
}

#[test]
fn test_fmt02_rebar_material_independent_fys() {
    // A material explicitly marked as MatType::Rebar with distinct fy and fys
    let mut m = base_model();
    let rebar = Material {
        name: "Rebar60_40".to_string(),
        mat_type: MatType::Rebar,
        e: 29000.0,
        e_ortho: None,
        u: 0.3,
        u_ortho: None,
        g: 11153.8,
        g_ortho: None,
        a: 6.5e-6,
        a_ortho: None,
        unit_weight: 0.49 / 1728.0,
        unit_mass: (0.49 / 1728.0) / 386.088,
        fy: 60.0,
        fu: 90.0,
        fc: 0.0,
        fys: 40.0,
        color: "Yellow".to_string(),
    };
    m.materials.insert("Rebar60_40".to_string(), rebar);

    let native =
        pack_model_to_native_sdb(&m).expect("rebar material independent fys export must succeed");
    let restored =
        extract_sdb_to_model(&native).expect("rebar material independent fys import must succeed");
    assert_eq!(restored.materials["Rebar60_40"].fy, 60.0);
    assert_eq!(restored.materials["Rebar60_40"].fys, 40.0);
    assert_eq!(restored, m);
}

#[test]
fn test_fmt02_multiple_materials_with_independent_fys() {
    let mut m = base_model();
    m.materials.get_mut("4000Psi").unwrap().fys = 65.5;
    m.materials.get_mut("A992Fy50").unwrap().fys = 42.0;

    let m3 = Material {
        name: "Conc5000".to_string(),
        mat_type: MatType::Concrete,
        e: 4000.0,
        e_ortho: None,
        u: 0.2,
        u_ortho: None,
        g: 1666.7,
        g_ortho: None,
        a: 5.5e-6,
        a_ortho: None,
        unit_weight: 0.150 / 1728.0,
        unit_mass: (0.150 / 1728.0) / 386.088,
        fy: 60.0,
        fu: 0.0,
        fc: 5.0,
        fys: 80.0,
        color: "Blue".to_string(),
    };
    m.materials.insert("Conc5000".to_string(), m3);

    let native = pack_model_to_native_sdb(&m).expect("multiple materials export must succeed");
    let restored = extract_sdb_to_model(&native).expect("multiple materials import must succeed");

    assert_eq!(restored.materials["4000Psi"].fys, 65.5);
    assert_eq!(restored.materials["A992Fy50"].fys, 42.0);
    assert_eq!(restored.materials["Conc5000"].fys, 80.0);
    assert_eq!(restored, m);
}

#[test]
fn test_fmt02_material_fys_equal_fy_is_preserved() {
    let mut m = base_model();
    m.materials.get_mut("4000Psi").unwrap().fy = 60.0;
    m.materials.get_mut("4000Psi").unwrap().fys = 60.0;

    let native = pack_model_to_native_sdb(&m).expect("fys == fy export must succeed");
    let restored = extract_sdb_to_model(&native).expect("fys == fy import must succeed");
    assert_eq!(restored.materials["4000Psi"].fys, 60.0);
    assert_eq!(restored, m);
}

#[test]
fn test_fmt02_rejects_nan_in_fys() {
    let mut m = base_model();
    m.materials.get_mut("4000Psi").unwrap().fys = f64::NAN;
    assert_unsupported(&m, "materials.4000Psi.fys");
}

#[test]
fn test_fmt02_rejects_inf_in_fys() {
    let mut m = base_model();
    m.materials.get_mut("4000Psi").unwrap().fys = f64::INFINITY;
    assert_unsupported(&m, "materials.4000Psi.fys");
}

#[test]
fn test_fmt02_fys_zero_fallback_behavior() {
    let mut m = base_model();
    m.materials.get_mut("4000Psi").unwrap().fy = 60.0;
    m.materials.get_mut("4000Psi").unwrap().fys = 0.0;

    // What happens when fys == 0.0?
    // In sdb.rs line 1379: "else if rfy > 0.0 { mat.fys = rfy; } else { mat.fys = mat.fy; }"
    // Because rfy is 0.0, rfy > 0.0 is false, so it falls back to mat.fys = mat.fy (60.0).
    // verify_projection will detect this change and reject export!
    let err = pack_model_to_native_sdb(&m);
    println!("fys=0.0 pack result: {:?}", err);
}

#[test]
fn test_fmt02_fys_negative_fallback_behavior() {
    let mut m = base_model();
    m.materials.get_mut("4000Psi").unwrap().fy = 60.0;
    m.materials.get_mut("4000Psi").unwrap().fys = -10.0;

    let err = pack_model_to_native_sdb(&m);
    println!("fys=-10.0 pack result: {:?}", err);
}

#[test]
fn test_fmt02_rebar_material_fys_equal_fy() {
    let mut m = base_model();
    let rebar = Material {
        name: "Rebar60".to_string(),
        mat_type: MatType::Rebar,
        e: 29000.0,
        e_ortho: None,
        u: 0.3,
        u_ortho: None,
        g: 11153.8,
        g_ortho: None,
        a: 6.5e-6,
        a_ortho: None,
        unit_weight: 0.49 / 1728.0,
        unit_mass: (0.49 / 1728.0) / 386.088,
        fy: 60.0,
        fu: 90.0,
        fc: 0.0,
        fys: 60.0,
        color: "Yellow".to_string(),
    };
    m.materials.insert("Rebar60".to_string(), rebar);

    let native = pack_model_to_native_sdb(&m).expect("rebar material fys==fy export must succeed");
    let restored =
        extract_sdb_to_model(&native).expect("rebar material fys==fy import must succeed");
    assert_eq!(restored.materials["Rebar60"].fy, 60.0);
    assert_eq!(restored.materials["Rebar60"].fys, 60.0);
    assert_eq!(restored, m);
}

#[test]
fn test_fmt02_tendon_material_independent_fys() {
    let mut m = base_model();
    let tendon = Material {
        name: "Tendon270".to_string(),
        mat_type: MatType::Tendon,
        e: 28500.0,
        e_ortho: None,
        u: 0.3,
        u_ortho: None,
        g: 10961.5,
        g_ortho: None,
        a: 6.5e-6,
        a_ortho: None,
        unit_weight: 0.49 / 1728.0,
        unit_mass: (0.49 / 1728.0) / 386.088,
        fy: 243.0,
        fu: 270.0,
        fc: 0.0,
        fys: 180.0,
        color: "Magenta".to_string(),
    };
    m.materials.insert("Tendon270".to_string(), tendon);

    let native =
        pack_model_to_native_sdb(&m).expect("tendon material independent fys export must succeed");
    let restored =
        extract_sdb_to_model(&native).expect("tendon material independent fys import must succeed");
    assert_eq!(restored.materials["Tendon270"].fy, 243.0);
    assert_eq!(restored.materials["Tendon270"].fys, 180.0);
    assert_eq!(restored, m);
}

#[test]
fn test_fmt01_vendor_fixture_modification_with_local_axes() {
    let sdb_path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../extracted/files/example_1_001.sdb");
    let sdb = sap_format::read_sdb_file(&sdb_path).expect("read vendor fixture must succeed");
    let mut model = extract_sdb_to_model(&sdb).expect("extract vendor fixture must succeed");

    let rot = euler_zyx_to_matrix(0.35, -0.42, 1.15);
    // Find joint "3" or first point in vendor file
    let pt_key = model.points.keys().next().unwrap().clone();
    model.points.get_mut(&pt_key).unwrap().local_axes_matrix = Some(rot);

    let native = pack_model_to_native_sdb(&model)
        .expect("vendor fixture with local axes export must succeed");
    let restored =
        extract_sdb_to_model(&native).expect("vendor fixture with local axes import must succeed");

    assert_eq!(restored.points[&pt_key].local_axes_matrix, Some(rot));
    assert_eq!(restored, model);
}

#[test]
fn test_fmt02_extreme_small_and_large_fys() {
    let mut m = base_model();
    // Micro yield stress
    let conc = m.materials.get_mut("4000Psi").unwrap();
    conc.fys = 1e-6;
    let native = pack_model_to_native_sdb(&m).expect("micro fys export must succeed");
    let restored = extract_sdb_to_model(&native).expect("micro fys import must succeed");
    assert_eq!(restored.materials["4000Psi"].fys, 1e-6);

    // Giga yield stress (SI units: 100 GPa)
    let conc2 = m.materials.get_mut("4000Psi").unwrap();
    conc2.fys = 1.0e11;
    let native2 = pack_model_to_native_sdb(&m).expect("giga fys export must succeed");
    let restored2 = extract_sdb_to_model(&native2).expect("giga fys import must succeed");
    assert_eq!(restored2.materials["4000Psi"].fys, 1.0e11);
}

#[test]
fn test_fmt02_all_other_material_types_independent_fys() {
    let types = [
        ("ColdFormed_Mat", MatType::ColdFormed, 50.0, 40.0),
        ("Aluminum_Mat", MatType::Aluminum, 35.0, 25.0),
        ("NoDesign_Mat", MatType::NoDesign, 36.0, 24.0),
    ];

    for (name, mat_type, fy, fys) in types {
        let mut m = base_model();
        let mat = Material {
            name: name.to_string(),
            mat_type,
            e: 29000.0,
            e_ortho: None,
            u: 0.3,
            u_ortho: None,
            g: 11153.8,
            g_ortho: None,
            a: 6.5e-6,
            a_ortho: None,
            unit_weight: 0.49 / 1728.0,
            unit_mass: (0.49 / 1728.0) / 386.088,
            fy,
            fu: fy * 1.2,
            fc: 0.0,
            fys,
            color: "Green".to_string(),
        };
        m.materials.insert(name.to_string(), mat);

        let native = pack_model_to_native_sdb(&m)
            .unwrap_or_else(|e| panic!("export for {name} failed: {e}"));
        let restored = extract_sdb_to_model(&native)
            .unwrap_or_else(|e| panic!("import for {name} failed: {e}"));

        assert_eq!(restored.materials[name].fy, fy);
        assert_eq!(restored.materials[name].fys, fys);
        assert_eq!(restored, m);
    }
}

#[test]
fn test_fmt01_and_fmt02_combined_model() {
    let mut m = base_model();
    // Joint local axes
    let rot_a = euler_zyx_to_matrix(0.5, -0.2, 0.8);
    let rot_b = rodrigues_to_matrix([1.0, 1.0, 0.0], 1.2);
    m.points.get_mut("A").unwrap().local_axes_matrix = Some(rot_a);
    m.points.get_mut("B").unwrap().local_axes_matrix = Some(rot_b);

    // Concrete material with independent fys
    let conc = m.materials.get_mut("4000Psi").unwrap();
    conc.fy = 60.0;
    conc.fys = 75.0;

    // Steel material with independent fys
    let steel = m.materials.get_mut("A992Fy50").unwrap();
    steel.fy = 50.0;
    steel.fys = 36.0;

    let native = pack_model_to_native_sdb(&m).expect("combined export must succeed");
    let doc = inspect_native_sdb(&native).unwrap();
    assert_eq!(doc.inventory().parsed_bytes, doc.inventory().stream_bytes);

    let restored = extract_sdb_to_model(&native).expect("combined import must succeed");
    assert_matrix_close(
        restored.points["A"].local_axes_matrix.unwrap(),
        rot_a,
        1e-12,
    );
    assert_matrix_close(
        restored.points["B"].local_axes_matrix.unwrap(),
        rot_b,
        1e-12,
    );
    assert_eq!(restored.materials["4000Psi"].fys, 75.0);
    assert_eq!(restored.materials["A992Fy50"].fys, 36.0);
    assert_eq!(restored, m);
    assert_eq!(parse_s2k(&write_s2k(&restored)).unwrap(), m);
}
