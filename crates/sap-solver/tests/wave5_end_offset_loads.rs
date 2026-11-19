//! Wave 5 P13, step a (D46): span loads on a frame with end offsets.
//!
//! Analysis Reference p. 135: the self-weight "is constant within the end
//! offsets", so every span load acts on the whole length between the joints and
//! the distances of span loads are measured along it. p. 130: the rigid-end
//! factor gives the fraction of each end offset that is rigid for bending and
//! shear; "If a moment or shear release is specified in either bending plane at
//! either end of the element, the end offset is assumed to be rigid for bending
//! and shear in that plane at that end (i.e., it acts as if rigid = 1)" (pp.
//! 130-131), and releases are at the support faces.
//!
//! The part of a load on a rigid arm goes to the joint of that end as a force and
//! the moment of that force about the joint; the flexible span carries the rest.
//! The references are statics and the closed forms of a fixed-fixed or
//! simply supported beam of the flexible length, written out in each test.
//! Units: kip, in.

use sap_model::*;
use sap_solver::analyze_model;

const E: f64 = 29_000.0;
const L: f64 = 144.0;
const OFFSET: f64 = 12.0;
const I33: f64 = 100.0;

fn assert_close(actual: f64, expected: f64, tol: f64, what: &str) {
    let scale = expected.abs().max(1.0);
    assert!(
        (actual - expected).abs() <= tol * scale,
        "{what}: {actual} against {expected} (tolerance {tol} relative)"
    );
}

/// A member of length 144 from joint 1 at the origin to joint 2 at `end`, both
/// joints fully fixed. No shear deformation (As2 = As3 = 0). The patterns W
/// (uniform load 1 per unit length along the member, downward), SW (self weight,
/// weight density 0.1 and area 10 so 1 per unit length) and PL (a point load) are
/// one linear static case each.
fn beam(end: [f64; 3], offset_i: f64, offset_j: f64, rz: f64) -> SapModel {
    let mut model = SapModel::default();
    model.load_cases.clear();
    model.load_patterns.clear();
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.e = E;
    mat.u = 0.3;
    mat.g = 11_000.0;
    mat.a = 1.0e-5;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.1;
    model.materials.insert("MAT".into(), mat);
    model.frame_sections.insert(
        "S".into(),
        FrameSection {
            name: "S".into(),
            material: "MAT".into(),
            area: 10.0,
            i22: I33,
            i33: I33,
            torsional_j: 10.0,
            as2: 0.0,
            as3: 0.0,
            ..FrameSection::default()
        },
    );
    model.add_point("1", 0.0, 0.0, 0.0);
    model.add_point("2", end[0], end[1], end[2]);
    for joint in ["1", "2"] {
        model.points.get_mut(joint).unwrap().restraint = [true; 6];
    }
    model.add_frame("F", "1", "2", "S");
    let frame = model.frames.get_mut("F").unwrap();
    frame.end_offset_i = offset_i;
    frame.end_offset_j = offset_j;
    frame.rigid_zone_factor = rz;
    frame.loads_dist.push(FrameLoadDistributed {
        load_pat: "W".into(),
        my_type: 1,
        dir: 10,
        csys: "GLOBAL".into(),
        rel_dist1: 0.0,
        rel_dist2: 1.0,
        val1: 1.0,
        val2: 1.0,
    });
    for (name, self_weight) in [("W", 0.0), ("SW", 1.0), ("PL", 0.0)] {
        model.load_patterns.insert(
            name.into(),
            LoadPattern {
                name: name.into(),
                pat_type: LoadPatternType::Other,
                self_wt_multiplier: self_weight,
            },
        );
        model
            .load_cases
            .insert(name.into(), LoadCase::linear_static(name, name, 1.0));
    }
    model
}

fn release_both_ends(model: &mut SapModel) {
    let frame = model.frames.get_mut("F").unwrap();
    frame.release_i[5] = true;
    frame.release_j[5] = true;
}

fn reaction(model: &SapModel, case: &str, joint: &str) -> [f64; 6] {
    model.case_results[case].joint_reactions[joint]
}

/// Sum of the joint reactions and of their moments about the origin, which must
/// balance a vertical load of `total` whose resultant acts at `at`.
fn assert_equilibrium(model: &SapModel, case: &str, total: f64, at: [f64; 3], what: &str) {
    let mut force = [0.0; 3];
    let mut moment = [0.0; 3];
    for (joint, r) in &model.case_results[case].joint_reactions {
        let p = &model.points[joint];
        let x = [p.x, p.y, p.z];
        for c in 0..3 {
            force[c] += r[c];
            moment[c] += r[3 + c];
        }
        moment[0] += x[1] * r[2] - x[2] * r[1];
        moment[1] += x[2] * r[0] - x[0] * r[2];
        moment[2] += x[0] * r[1] - x[1] * r[0];
    }
    // The load is -total along Z at `at`; the reactions balance it.
    assert_close(force[0], 0.0, 1e-9, &format!("{what}: sum Fx"));
    assert_close(force[1], 0.0, 1e-9, &format!("{what}: sum Fy"));
    assert_close(force[2], total, 1e-9, &format!("{what}: sum Fz"));
    let applied = [at[1] * -total, -at[0] * -total, 0.0];
    // moment of the load about the origin: r x (0, 0, -total)
    assert_close(moment[0] + applied[0], 0.0, 1e-9, &format!("{what}: sum Mx"));
    assert_close(moment[1] + applied[1], 0.0, 1e-9, &format!("{what}: sum My"));
    assert_close(moment[2], 0.0, 1e-9, &format!("{what}: sum Mz"));
}

/// Review numbers (rows 1 of the R2 review): a horizontal fixed-fixed beam,
/// offsets 12 and 12, uniform load 1 (total 144). Baseline: 120 with a release at
/// rigid-end factor 0, 132 with 0.5 and no release, 120 with 1.
#[test]
fn d46_sum_of_reactions_equals_the_load_for_every_rigid_end_factor_and_release() {
    for rz in [0.0, 0.5, 1.0] {
        for released in [false, true] {
            for case in ["W", "SW"] {
                let mut model = beam([L, 0.0, 0.0], OFFSET, OFFSET, rz);
                if released {
                    release_both_ends(&mut model);
                }
                analyze_model(&mut model).unwrap();
                assert_equilibrium(
                    &model,
                    case,
                    144.0,
                    [72.0, 0.0, 0.0],
                    &format!("horizontal rz {rz} released {released} {case}"),
                );
            }
        }
    }
}

/// The same on an inclined 3-4-5 member (86.4, 0, 115.2): the load is 1 per unit
/// length along the member, so the total is 144 and its resultant is at the
/// midpoint. Offsets 12 and 24 (unequal), the release at end I only for the
/// second pass.
#[test]
fn d46_equilibrium_on_an_inclined_member_with_unequal_offsets() {
    for rz in [0.0, 0.5, 1.0] {
        for released in [false, true] {
            for case in ["W", "SW"] {
                let mut model = beam([86.4, 0.0, 115.2], 12.0, 24.0, rz);
                if released {
                    model.frames.get_mut("F").unwrap().release_i[5] = true;
                }
                analyze_model(&mut model).unwrap();
                assert_equilibrium(
                    &model,
                    case,
                    144.0,
                    [43.2, 0.0, 57.6],
                    &format!("inclined rz {rz} released {released} {case}"),
                );
            }
        }
    }
}

/// Fixed-fixed beam under a uniform load w = 1 on the whole length, no release,
/// offsets 12 and 12. The rigid arm is rz x 12 and the flexible span
/// lf = 144 - 24 rz. Closed form of the beam of length lf: face shear w lf / 2 and
/// face moment w lf^2 / 12; the arm of length a carries w a at its middle. The
/// joint therefore receives the shear w (lf / 2 + a) = 72 and the moment
/// w lf^2 / 12 + (w lf / 2) a + w a^2 / 2.
#[test]
fn d46_joint_moment_of_a_uniform_load_on_arm_and_span() {
    for rz in [0.0, 0.5, 1.0] {
        let arm = rz * OFFSET;
        let lf = L - 2.0 * arm;
        let moment = lf * lf / 12.0 + 0.5 * lf * arm + 0.5 * arm * arm;
        for case in ["W", "SW"] {
            let mut model = beam([L, 0.0, 0.0], OFFSET, OFFSET, rz);
            analyze_model(&mut model).unwrap();
            let r1 = reaction(&model, case, "1");
            let r2 = reaction(&model, case, "2");
            assert_close(r1[2], 72.0, 1e-9, &format!("rz {rz} {case}: Fz at 1"));
            assert_close(r2[2], 72.0, 1e-9, &format!("rz {rz} {case}: Fz at 2"));
            assert_close(r1[4].abs(), moment, 1e-9, &format!("rz {rz} {case}: |My| at 1"));
            assert_close(r2[4].abs(), moment, 1e-9, &format!("rz {rz} {case}: |My| at 2"));
            assert!(r1[4] * r2[4] < 0.0, "rz {rz} {case}: the end moments are opposite");
        }
    }
}

/// With a moment release at both faces the span is simply supported between the
/// faces whatever the rigid-end factor (the offset acts as rigid, p. 131): face
/// shear w lf / 2 = 60 with lf = 120, no face moment. The joint receives
/// 60 + 12 = 72 and the moment 60 x 12 + 12^2 / 2 = 792.
#[test]
fn d46_released_ends_carry_the_load_on_the_arm_for_every_rigid_end_factor() {
    for rz in [0.0, 0.5, 1.0] {
        for case in ["W", "SW"] {
            let mut model = beam([L, 0.0, 0.0], OFFSET, OFFSET, rz);
            release_both_ends(&mut model);
            analyze_model(&mut model).unwrap();
            for joint in ["1", "2"] {
                let r = reaction(&model, case, joint);
                assert_close(r[2], 72.0, 1e-9, &format!("rz {rz} {case}: Fz at {joint}"));
                assert_close(r[4].abs(), 792.0, 1e-9, &format!("rz {rz} {case}: |My| at {joint}"));
            }
        }
    }
}

/// A point load of 10 (downward, global) at 6 in from joint 1, on the rigid arm
/// (offset 12, rz 1): the arm carries it to joint 1, which receives the force 10
/// and the moment 60; joint 2 receives nothing. Baseline: the load was placed
/// at 6 / 144 of the flexible length, in the span.
#[test]
fn d46_a_point_load_on_the_arm_goes_to_its_joint() {
    let mut model = beam([L, 0.0, 0.0], OFFSET, OFFSET, 1.0);
    model.frames.get_mut("F").unwrap().loads_point.push(FrameLoadPoint {
        load_pat: "PL".into(),
        my_type: 1,
        dir: 6,
        csys: "GLOBAL".into(),
        rel_dist: 6.0 / L,
        val: -10.0,
    });
    analyze_model(&mut model).unwrap();
    let r1 = reaction(&model, "PL", "1");
    let r2 = reaction(&model, "PL", "2");
    assert_close(r1[2], 10.0, 1e-9, "Fz at 1");
    assert_close(r1[4].abs(), 60.0, 1e-9, "|My| at 1");
    assert_close(r2[2], 0.0, 1e-9, "Fz at 2");
    assert_close(r2[4], 0.0, 1e-9, "My at 2");
}

/// A point load of 10 at 60 in from joint 1 on a fixed-fixed beam with rigid
/// offsets of 12 (rz 1): the span between the faces has length 120 and the load
/// is a = 48 from the near face and b = 72 from the far one. Fixed-fixed beam: V_I
/// = P b^2 (3a + b) / lf^3, M_I = P a b^2 / lf^2. The joint receives the same shear
/// and the moment M_I + 12 V_I (the arm is rigid).
#[test]
fn d46_a_point_load_in_the_span_is_placed_along_the_full_length() {
    let (p, a, b, lf) = (10.0f64, 48.0f64, 72.0f64, 120.0f64);
    let shear = p * b * b * (3.0 * a + b) / lf.powi(3);
    let face_moment = p * a * b * b / (lf * lf);
    let mut model = beam([L, 0.0, 0.0], OFFSET, OFFSET, 1.0);
    model.frames.get_mut("F").unwrap().loads_point.push(FrameLoadPoint {
        load_pat: "PL".into(),
        my_type: 1,
        dir: 6,
        csys: "GLOBAL".into(),
        rel_dist: 60.0 / L,
        val: -p,
    });
    analyze_model(&mut model).unwrap();
    let r1 = reaction(&model, "PL", "1");
    assert_close(r1[2], shear, 1e-9, "Fz at 1");
    assert_close(r1[4].abs(), face_moment + OFFSET * shear, 1e-9, "|My| at 1");
}

/// Review numbers: the inclined member of the review (3-4-5, offsets 12 and 12,
/// rz 0), self weight 14.4 (0.1 per unit length over 144: weight density 0.01, area 10) with a release at I:
/// baseline reaction (Fx, Fz) = (0.576, 13.968) against (0, 14.4) without the
/// release. The release only redistributes the load, so the sum of the reactions
/// at the two fixed joints is the weight whatever the release.
#[test]
fn d46_review_inclined_self_weight_with_a_release_keeps_the_resultant() {
    for released in [false, true] {
        let mut model = beam([86.4, 0.0, 115.2], 12.0, 12.0, 0.0);
        model.materials.get_mut("MAT").unwrap().unit_weight = 0.01;
        if released {
            model.frames.get_mut("F").unwrap().release_i[5] = true;
        }
        analyze_model(&mut model).unwrap();
        let (r1, r2) = (reaction(&model, "SW", "1"), reaction(&model, "SW", "2"));
        assert_close(r1[0] + r2[0], 0.0, 1e-9, &format!("released {released}: sum Fx"));
        assert_close(r1[2] + r2[2], 14.4, 1e-9, &format!("released {released}: sum Fz"));
    }
}

/// Without any offset nothing changes: the beam without offsets under the
/// uniform load has the textbook fixed-fixed reactions w L / 2 and w L^2 / 12.
#[test]
fn d46_member_without_offsets_keeps_the_textbook_reactions() {
    let mut model = beam([L, 0.0, 0.0], 0.0, 0.0, 1.0);
    analyze_model(&mut model).unwrap();
    let r1 = reaction(&model, "W", "1");
    assert_close(r1[2], 72.0, 1e-12, "Fz");
    assert_close(r1[4].abs(), L * L / 12.0, 1e-12, "|My|");
}
