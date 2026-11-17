//! Wave 5 P13: the message of an unstable structure never names a joint that an
//! automatic subdivision created.
//!
//! The model is the torsion bar of `wave5_stability.rs`: one frame along X whose
//! two ends are restrained in every degree of freedom but the torsion R1, so its
//! twist about X is a rigid-body motion (the exact null vector of the torsion
//! stiffness G J / L [[1, -1], [-1, 1]] is (1, 1)). With three automatic segments
//! the singular pivot can lie at a joint the subdivision created
//! (`__fmesh_F_1`, `__fmesh_F_2`): the message must name the frame object and the
//! joints at its ends instead.

use sap_model::*;
use sap_solver::{analyze_model, SolverError};

fn torsion_bar(segments: usize) -> SapModel {
    let mut model = SapModel::default();
    for (_, pattern) in model.load_patterns.iter_mut() {
        pattern.self_wt_multiplier = 0.0;
    }
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.e = 1.0;
    mat.u = 0.3;
    mat.g = 1.0;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0;
    model.materials.insert("MAT".into(), mat);
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
    model.add_point("A", 0.0, 0.0, 0.0);
    model.add_point("B", 3.0, 0.0, 0.0);
    for name in ["A", "B"] {
        model.points.get_mut(name).unwrap().restraint = [true, true, true, false, true, true];
    }
    model.add_frame("F3", "A", "B", "S");
    model.frames.get_mut("F3").unwrap().auto_mesh_segments = segments;
    model
}

/// Fails on the baseline: the message named the internal joint.
#[test]
fn an_unstable_subdivided_frame_is_named_by_its_frame_and_end_joints() {
    let mut model = torsion_bar(3);
    let error = analyze_model(&mut model).unwrap_err();
    assert!(!matches!(error, SolverError::UnsupportedInput(_)), "{error:?}");
    let text = error.to_string();
    assert!(!text.contains("__fmesh"), "{text}");
    assert!(
        text.contains("frame 'F3' between its joints 'A' and 'B'"),
        "{text}"
    );
    assert!(text.contains("R1"), "{text}");
    // A refused analysis leaves no results behind and no created joint in the model.
    assert!(model.case_results.is_empty());
    assert!(model.points.keys().all(|name| !name.starts_with("__fmesh")));
    assert_eq!(model.frames.len(), 1);
}

/// A joint of the model keeps its own name (no subdivision, then the same bar).
#[test]
fn an_unstable_joint_of_the_model_keeps_its_name() {
    let mut model = torsion_bar(1);
    let text = analyze_model(&mut model).unwrap_err().to_string();
    assert!(text.contains("unstable at joint 'B' R1"), "{text}");
}
