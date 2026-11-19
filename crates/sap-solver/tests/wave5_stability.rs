//! Wave 5 step 1a (D03): a singular pivot is reported at a joint and a degree
//! of freedom, and a mechanism whose pivots pass the pivot test is refused.
//!
//! References (none of them comes from the solver):
//! - R1: one frame along X whose two ends are restrained in every DOF but the
//!   torsion R1. The 2x2 torsion block is G J / L [[1, -1], [-1, 1]], with the
//!   exact null vector (1, 1) (a rigid twist of the bar).
//! - R2, R3: a cylinder of 36 flat facets under internal pressure. A rigid
//!   rotation `u = w e_z x (x - c)` about the vertical line through the point
//!   c = (60, 60) is a null vector of the stiffness when the supports are UZ on
//!   the base ring, UY at theta = 0 and UX at theta = 90 degrees (their two
//!   restraint lines meet at c); its drilling energy is exactly zero because
//!   the rotation of the joints equals the in-plane rotation. Adding UY at
//!   theta = 180 degrees removes it. The hoop membrane force of the sound
//!   cylinder is p R cos(pi/36) per unit height (joint forces of the lumped
//!   pressure balanced by the facet forces).

use sap_model::*;
use sap_solver::{analyze_model, SolverError};

fn steel(model: &mut SapModel, e: f64, g: f64) {
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.e = e;
    mat.u = 0.3;
    mat.g = g;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0;
    model.materials.insert("MAT".into(), mat);
}

fn torsion_bar() -> SapModel {
    let mut model = SapModel::default();
    for (_, pattern) in model.load_patterns.iter_mut() {
        pattern.self_wt_multiplier = 0.0;
    }
    steel(&mut model, 1.0, 1.0);
    model.frame_sections.insert(
        "S".into(),
        FrameSection {
            name: "S".into(),
            material: "MAT".into(),
            area: 1.0,
            i22: 1.0,
            i33: 1.0,
            torsional_j: 1.0,
            ..FrameSection::default()
        },
    );
    model.add_point("J1", 0.0, 0.0, 0.0);
    model.add_point("J2", 1.0, 0.0, 0.0);
    for name in ["J1", "J2"] {
        model.points.get_mut(name).unwrap().restraint = [true, true, true, false, true, true];
    }
    model.add_frame("F", "J1", "J2", "S");
    model
}

#[test]
fn r1_a_singular_torsion_pair_is_named_at_its_joint_and_dof() {
    let mut model = torsion_bar();
    let error = analyze_model(&mut model).unwrap_err();
    assert!(matches!(error, SolverError::Unstable { .. }), "{error:?}");
    assert_eq!(
        error.to_string(),
        "Singular matrix: the structure is unstable at joint 'J2' R1 \
         (zero or negative pivot after restraints)"
    );
    // A refused analysis leaves no results behind.
    assert!(model.case_results.is_empty());
    assert!(!model.is_locked);
}

#[test]
fn r1b_the_same_pair_at_a_joint_with_local_axes_says_so() {
    let mut model = torsion_bar();
    // Rotating the axes of J2 about the bar axis X keeps its local R1 along
    // global X, and every other DOF is restrained, so the pair stays singular.
    model.points.get_mut("J2").unwrap().local_axes_deg = [0.0, 0.0, 30.0];
    let error = analyze_model(&mut model).unwrap_err().to_string();
    assert_eq!(
        error,
        "Singular matrix: the structure is unstable at joint 'J2' R1 (joint local axes) \
         (zero or negative pivot after restraints)"
    );
}
