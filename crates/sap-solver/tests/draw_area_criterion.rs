//! `sap_model::draw::check_area_corners` (and so `SapModel::draw_areas`) must
//! apply the criterion the analysis applies to every area
//! (`validate_area_geometry` in this crate), because `sap-model` cannot call
//! the solver and repeats the test.
//!
//! Two checks:
//! - an area `draw_areas` accepts is analyzed (a corner loaded, the others
//!   fixed: the corner displaces in the load direction);
//! - the shapes `check_area_corners` refuses are refused by the analysis with
//!   the geometry error, including over a sweep of pseudo-random quadrilaterals
//!   and triangles (a linear congruential generator with a fixed seed).
//!
//! Units kip, in (the default model).

use sap_model::draw::check_area_corners;
use sap_model::{PointLoadForce, SapModel};
use sap_solver::analyze_model;

/// A model whose single area has the given corners. Joint 1 is free and loaded
/// with 1 kip in -Z; every other joint is fully fixed.
fn model_with_area(corners: &[[f64; 3]]) -> SapModel {
    let mut m = SapModel::default();
    let mut names = Vec::new();
    for (k, c) in corners.iter().enumerate() {
        let name = (k + 1).to_string();
        m.add_point(&name, c[0], c[1], c[2]);
        if k > 0 {
            m.points.get_mut(&name).unwrap().restraint = [true; 6];
        }
        names.push(name);
    }
    m.add_area("A", names, "ASEC1");
    m.load_patterns.get_mut("DEAD").unwrap().self_wt_multiplier = 0.0;
    m.points.get_mut("1").unwrap().loads_force.push(PointLoadForce {
        load_pat: "DEAD".into(),
        csys: "GLOBAL".into(),
        force: [0.0, 0.0, -1.0, 0.0, 0.0, 0.0],
    });
    m
}

fn is_geometry_refusal(e: &sap_solver::SolverError) -> bool {
    e.to_string().contains("folded geometry")
}

#[test]
fn an_area_draw_areas_accepts_is_analyzed() {
    let accepted: [&[[f64; 3]]; 5] = [
        &[[0.0, 0.0, 0.0], [40.0, 0.0, 0.0], [40.0, 50.0, 0.0], [0.0, 50.0, 0.0]],
        &[[0.0, 0.0, 0.0], [40.0, 0.0, 0.0], [0.0, 50.0, 0.0]],
        // Vertical, clockwise, and slightly warped quadrilaterals.
        &[[0.0, 0.0, 0.0], [40.0, 0.0, 0.0], [40.0, 0.0, 30.0], [0.0, 0.0, 30.0]],
        &[[0.0, 0.0, 0.0], [0.0, 50.0, 0.0], [40.0, 50.0, 0.0], [40.0, 0.0, 0.0]],
        &[[0.0, 0.0, 0.0], [40.0, 0.0, 1.0], [40.0, 50.0, 0.0], [0.0, 50.0, 1.0]],
    ];
    for corners in accepted {
        // The area drawn by the command itself is the one analyzed: joint 1 is
        // the first corner; the others are fixed and joint 1 carries the load.
        let mut m = SapModel::default();
        m.draw_areas(&[corners.to_vec()], "ASEC1")
            .unwrap_or_else(|e| panic!("{corners:?}: {e}"));
        assert_eq!(m.areas.len(), 1);
        for (name, p) in m.points.iter_mut() {
            p.restraint = [name != "1"; 6];
        }
        m.load_patterns.get_mut("DEAD").unwrap().self_wt_multiplier = 0.0;
        m.points.get_mut("1").unwrap().loads_force.push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: [0.0, 0.0, -1.0, 0.0, 0.0, 0.0],
        });
        analyze_model(&mut m).unwrap_or_else(|e| panic!("{corners:?}: {e}"));
        let u = m.case_results["DEAD"].joint_displacements["1"];
        assert!(u.iter().all(|v| v.is_finite()));
        assert!(u.iter().any(|v| v.abs() > 0.0), "{corners:?}: the corner must move");
    }
}

#[test]
fn shapes_check_area_corners_refuses_are_refused_by_the_analysis() {
    let refused: [&[[f64; 3]]; 6] = [
        // Bow-tie, dart (concave), repeated corner, collinear, two equal triangle corners.
        &[[0.0, 0.0, 0.0], [40.0, 50.0, 0.0], [40.0, 0.0, 0.0], [0.0, 50.0, 0.0]],
        &[[0.0, 0.0, 0.0], [40.0, 0.0, 0.0], [10.0, 10.0, 0.0], [0.0, 40.0, 0.0]],
        &[[0.0, 0.0, 0.0], [40.0, 0.0, 0.0], [40.0, 0.0, 0.0], [0.0, 50.0, 0.0]],
        &[[0.0, 0.0, 0.0], [40.0, 0.0, 0.0], [80.0, 0.0, 0.0], [120.0, 0.0, 0.0]],
        &[[0.0, 0.0, 0.0], [40.0, 0.0, 0.0], [40.0, 0.0, 0.0]],
        &[[0.0, 0.0, 0.0], [40.0, 0.0, 0.0], [80.0, 0.0, 0.0]],
    ];
    for corners in refused {
        assert!(check_area_corners(corners).is_err(), "{corners:?}");
        let mut m = model_with_area(corners);
        let e = analyze_model(&mut m).expect_err("the analysis must refuse the area");
        assert!(is_geometry_refusal(&e), "{corners:?}: {e}");
    }
}

/// Agreement over pseudo-random shapes: whenever `check_area_corners` refuses,
/// the analysis refuses with the geometry error; whenever it accepts, the
/// analysis gives no geometry error.
#[test]
fn the_two_criteria_agree_on_random_corner_sets() {
    let mut state = 0x2545_f491_4f6c_dd1d_u64;
    let mut next = || {
        state = state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        ((state >> 33) as f64) / f64::from(1u32 << 31)
    };
    let (mut accepted, mut refused) = (0, 0);
    for case in 0..120 {
        let n = 3 + case % 2;
        let planar = case % 3 != 0;
        let corners: Vec<[f64; 3]> = (0..n)
            .map(|_| {
                let z = if planar { 0.0 } else { 20.0 * next() };
                [100.0 * next(), 100.0 * next(), z]
            })
            .collect();
        let mut m = model_with_area(&corners);
        let result = analyze_model(&mut m);
        match check_area_corners(&corners) {
            Ok(()) => {
                accepted += 1;
                if let Err(e) = result {
                    assert!(!is_geometry_refusal(&e), "{corners:?}: {e}");
                }
            }
            Err(_) => {
                refused += 1;
                let e = result.expect_err("the analysis must refuse the area");
                assert!(is_geometry_refusal(&e), "{corners:?}: {e}");
            }
        }
    }
    assert!(accepted > 10 && refused > 10, "{accepted} accepted, {refused} refused");
}
