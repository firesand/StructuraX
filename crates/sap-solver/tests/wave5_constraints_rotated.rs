//! Wave 5 step 1d (D22): multipoint constraints are written in global
//! components, whatever the local axes of the constrained joints.
//!
//! Analysis Reference pp. 66-67: the displacement at a constrained degree of
//! freedom is a linear combination of the displacements of the master joint, and
//! "Joint local coordinate systems, springs, masses, and loads may all be applied
//! to constrained joints". A Diaphragm, Body or Equal constraint relates global
//! displacement components; the unknowns of a joint with local axes are its local
//! components, `u_global = R u_local`.
//!
//! Reference: turning the local axes of joint B about Z by an angle changes the
//! unknowns of B by the orthogonal change of variables `u_local = R^T u_global`
//! and nothing else. The columns are pipes (I22 = I33, the same J), so the
//! stiffness of B is invariant under that change: every diagonal entry is
//! unchanged, hence the penalty (a multiple of the largest diagonal) is the same
//! in both models, and the penalty quadratic form is the same up to the
//! orthogonal change of variables. The two models therefore agree to the
//! roundoff of the penalty solve, whose size is a small multiple of
//! eps * kappa with kappa = penalty / k_eff: the penalty is 1e6 times the largest
//! diagonal 4 E I / L = 4 * 29000 * 100 / 3 kip-in (L = 3 in, the stiffest term
//! of a column) and k_eff = 10 kip / (largest displacement) is the stiffness the
//! load sees. The constant 8 in the tolerance is that "small multiple" (the
//! number of unknowns that meet at a constrained joint is of that order); it is
//! a bound for the backward error, not a measured value. (The two measured
//! differences of the Diaphragm and Body models were 5e-10 and 2e-10 relative,
//! against a bound of about 1e-9.)

use sap_model::*;
use sap_solver::analyze_model;

fn model(local_axes_deg_at_b: [f64; 3], constraint: ConstraintType, dofs: [bool; 6]) -> SapModel {
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.e = 29_000.0;
    mat.u = 0.3;
    mat.g = 11_000.0;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0;
    m.materials.insert("MAT".into(), mat);
    m.frame_sections.insert(
        "PIPE".into(),
        FrameSection {
            name: "PIPE".into(),
            material: "MAT".into(),
            area: 10.0,
            i22: 100.0,
            i33: 100.0,
            torsional_j: 200.0,
            as2: 0.0,
            as3: 0.0,
            ..FrameSection::default()
        },
    );
    m.add_point("BASE_A", 0.0, 0.0, 0.0);
    m.add_point("BASE_B", 5.0, 0.0, 0.0);
    m.add_point("A", 0.0, 0.0, 3.0);
    m.add_point("B", 5.0, 0.0, 3.0);
    for base in ["BASE_A", "BASE_B"] {
        m.points.get_mut(base).unwrap().restraint = [true; 6];
    }
    m.add_frame("CA", "BASE_A", "A", "PIPE");
    m.add_frame("CB", "BASE_B", "B", "PIPE");
    m.constraints.insert(
        "C".into(),
        ConstraintDef {
            name: "C".into(),
            constraint_type: constraint,
            axis: ConstraintAxis::Z,
            dofs,
        },
    );
    for joint in ["A", "B"] {
        m.points.get_mut(joint).unwrap().constraints.push("C".into());
    }
    m.points.get_mut("B").unwrap().local_axes_deg = local_axes_deg_at_b;
    m.load_patterns.insert(
        "P".into(),
        LoadPattern {
            name: "P".into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
    m.load_cases.insert("P".into(), LoadCase::linear_static("P", "P", 1.0));
    m.points.get_mut("A").unwrap().loads_force.push(PointLoadForce {
        load_pat: "P".into(),
        csys: "GLOBAL".into(),
        force: [10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    });
    m
}

/// `R^T v` for the rotation about Z by `angle` degrees, on both triples of a
/// six-component vector.
fn to_local(angle: f64, v: [f64; 6]) -> [f64; 6] {
    let (s, c) = angle.to_radians().sin_cos();
    let rt = |a: f64, b: f64, z: f64| [c * a + s * b, -s * a + c * b, z];
    let t = rt(v[0], v[1], v[2]);
    let r = rt(v[3], v[4], v[5]);
    [t[0], t[1], t[2], r[0], r[1], r[2]]
}

fn assert_models_agree(constraint: ConstraintType, dofs: [bool; 6]) {
    let angle = 30.0;
    let mut plain = model([0.0; 3], constraint, dofs);
    let mut rotated = model([angle, 0.0, 0.0], constraint, dofs);
    analyze_model(&mut plain).unwrap();
    analyze_model(&mut rotated).unwrap();
    let p = &plain.case_results["P"];
    let r = &rotated.case_results["P"];
    let scale = p
        .joint_displacements
        .values()
        .flat_map(|d| d.iter().take(3))
        .fold(0.0f64, |m, v| m.max(v.abs()));
    assert!(scale > 1e-6, "the load must move the structure: {scale}");
    let penalty = 1.0e6 * 4.0 * 29_000.0 * 100.0 / 3.0;
    let tol = 8.0 * f64::EPSILON * penalty / (10.0 / scale);
    for (joint, u1) in &p.joint_displacements {
        let expected = if joint == "B" { to_local(angle, *u1) } else { *u1 };
        let actual = r.joint_displacements[joint];
        for d in 0..6 {
            // Rotations are scaled by the length of the structure (5 in) to
            // compare them with displacements.
            let unit = if d < 3 { 1.0 } else { 5.0 };
            assert!(
                (actual[d] - expected[d]).abs() * unit <= tol * scale,
                "{constraint:?} joint {joint} dof {d}: {} against {}",
                actual[d],
                expected[d]
            );
        }
    }
    // The base reactions are those of unrotated joints.
    for (joint, f1) in &p.joint_reactions {
        let f2 = r.joint_reactions[joint];
        let big = f1.iter().fold(0.0f64, |m, v| m.max(v.abs()));
        for d in 0..6 {
            assert!((f2[d] - f1[d]).abs() <= tol * big, "{constraint:?} reaction {joint} {d}");
        }
    }
}

/// R9: a Z diaphragm over A and B, 10 kip along X at A.
#[test]
fn r9_a_diaphragm_ignores_the_local_axes_of_a_constrained_joint() {
    assert_models_agree(ConstraintType::Diaphragm, [true; 6]);
}

/// R10: a Body constraint and an Equal constraint on UX.
#[test]
fn r10_body_and_equal_constraints_ignore_the_local_axes_too() {
    assert_models_agree(ConstraintType::Body, [true; 6]);
    assert_models_agree(ConstraintType::Equal, [true, false, false, false, false, false]);
}

/// The Local constraint ties the joint-local DOFs themselves: with B turned
/// 90 degrees about Z, local U1 of B is global Y, so tying local U1 of A and B
/// ties UX of A to UY of B. A 10 kip load along X at A then moves B in Y by
/// exactly the UX of A (the constraint is penalised; 1e-6 is its accuracy).
#[test]
fn a_local_constraint_still_ties_the_local_components() {
    let mut m = model([90.0, 0.0, 0.0], ConstraintType::Local, [true, false, false, false, false, false]);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["P"];
    let ux_a = r.joint_displacements["A"][0];
    let u1_b_local = r.joint_displacements["B"][0];
    assert!(ux_a.abs() > 1e-6);
    assert!((u1_b_local - ux_a).abs() <= 1e-6 * ux_a.abs(), "{u1_b_local} {ux_a}");
}
