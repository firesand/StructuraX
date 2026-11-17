//! Wave 5 P13, step c (D48): the output stations of a frame with end offsets.
//!
//! Analysis Reference p. 130 ("Effect upon Internal Force Output"): "All
//! internal forces and moments are output at the faces of the supports and at
//! other equally-spaced points within the clear length of the element. No output
//! is produced within the end offset, which includes the joint. This is not
//! affected by the value of the rigid-end factor, rigid." p. 144: "Output will
//! only be produced at joints I or j when the corresponding end offset is zero."
//! p. 130: "End releases are always assumed to be at the support faces", and
//! with a moment or shear release the end offset acts as rigid (rigid = 1).
//!
//! `station` is the distance from joint I (so the first station is at the end
//! offset at I) and `rel_dist` that distance over the length between the joints.
//! References: the closed forms of a fixed-fixed beam of the flexible length
//! lf = L - rz (ioff + joff) under a uniform load w (with x' measured from the
//! start of the flexible part: M3 = -w lf^2 / 12 + w lf x' / 2 - w x'^2 / 2, V2 =
//! -dM3/dx') and of a simply supported beam when both ends are released (M3 = w
//! lf x' / 2 - w x'^2 / 2, midspan deflection 5 w lf^4 / (384 E I)). The load acts
//! on the whole length between the joints (p. 135), so the arms carry load that
//! the sections inside the faces do not see. Sign convention of the forces: the
//! sagging moment of a gravity load is positive M3 and dM3/dx = -V2 (Analysis
//! Reference, "Internal Force Output"). Units: kip, in.

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
        "{what}: {actual} against {expected} (tolerance {tol} relative to max(1, |expected|))"
    );
}

/// Beam from the origin to (144, 0, 0), both joints fixed, uniform gravity load 1
/// per unit length (pattern W), five stations. No shear deformation.
fn beam(offset_i: f64, offset_j: f64, rz: f64) -> SapModel {
    let mut model = SapModel::default();
    model.load_cases.clear();
    model.load_patterns.clear();
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.e = E;
    mat.u = 0.3;
    mat.g = 11_000.0;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0;
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
    model.add_point("2", L, 0.0, 0.0);
    for joint in ["1", "2"] {
        model.points.get_mut(joint).unwrap().restraint = [true; 6];
    }
    model.add_frame("F", "1", "2", "S");
    let frame = model.frames.get_mut("F").unwrap();
    frame.end_offset_i = offset_i;
    frame.end_offset_j = offset_j;
    frame.rigid_zone_factor = rz;
    frame.min_stations = 5;
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
    model.load_patterns.insert(
        "W".into(),
        LoadPattern {
            name: "W".into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
    model
        .load_cases
        .insert("W".into(), LoadCase::linear_static("W", "W", 1.0));
    model
}

fn stations(model: &SapModel) -> &Vec<FrameStationResult> {
    &model.case_results["W"].frame_forces["F"]
}

/// Review row 5. Baseline: the stations ran from joint to joint and a released
/// end reported the joint moment (-720 for the review's beam); the first and last
/// station were at 0 and 144.
#[test]
fn d48_stations_run_from_support_face_to_support_face() {
    for rz in [0.0, 0.5, 1.0] {
        let mut model = beam(OFFSET, OFFSET, rz);
        analyze_model(&mut model).unwrap();
        let st = stations(&model);
        assert_eq!(st.len(), 5, "rz {rz}");
        for (k, s) in st.iter().enumerate() {
            let x = OFFSET + k as f64 * (L - 2.0 * OFFSET) / 4.0;
            assert_close(s.station, x, 1e-12, &format!("rz {rz}: station {k}"));
            assert_close(s.rel_dist, x / L, 1e-12, &format!("rz {rz}: rel_dist {k}"));
        }
    }
    // An end without an offset reports at its joint.
    let mut model = beam(OFFSET, 0.0, 0.5);
    analyze_model(&mut model).unwrap();
    let st = stations(&model);
    assert_close(st[0].station, OFFSET, 1e-12, "offset at I only: first");
    assert_close(st[4].station, L, 1e-12, "offset at I only: last");
    assert_eq!(st[4].rel_dist, 1.0);
    let mut model = beam(0.0, 0.0, 0.5);
    analyze_model(&mut model).unwrap();
    let st = stations(&model);
    assert_eq!((st[0].station, st[4].station), (0.0, L), "no offsets");
}

/// Both ends released in bending: the span between the faces is simply
/// supported whatever the rigid-end factor (lf = 120): zero moment at the faces
/// (baseline -720), face shear w lf / 2 = 60, midspan moment w lf^2 / 8 = 1800 and
/// midspan deflection 5 w lf^4 / (384 E I), down (local 2 is global Z).
#[test]
fn d48_a_released_end_reports_zero_moment_at_its_support_face() {
    let lf = L - 2.0 * OFFSET;
    for rz in [0.0, 0.5, 1.0] {
        let mut model = beam(OFFSET, OFFSET, rz);
        let frame = model.frames.get_mut("F").unwrap();
        frame.release_i[5] = true;
        frame.release_j[5] = true;
        analyze_model(&mut model).unwrap();
        let st = stations(&model);
        for (k, s) in st.iter().enumerate() {
            let xp = k as f64 * lf / 4.0;
            let what = format!("rz {rz} station {k}");
            assert_close(s.m3, 0.5 * lf * xp - 0.5 * xp * xp, 1e-9, &format!("{what}: M3"));
            assert_close(s.v2, -(0.5 * lf - xp), 1e-9, &format!("{what}: V2"));
        }
        assert!(st[0].m3.abs() < 1e-9 && st[4].m3.abs() < 1e-9, "rz {rz}");
        let mid = 5.0 * lf.powi(4) / (384.0 * E * I33);
        assert_close(st[2].local_disp[1], -mid, 1e-9, &format!("rz {rz}: midspan deflection"));
        // The joints are fixed: the faces do not move.
        assert!(st[0].local_disp[1].abs() < 1e-12 && st[4].local_disp[1].abs() < 1e-12);
    }
}

/// Fixed-fixed beam, no release: the closed forms with the flexible length
/// lf = 144 - 24 rz, x' = x - 12 rz from the start of the flexible part (for
/// rz < 1 the part of the offset that is not rigid is flexible, so the face at 12
/// is inside the flexible part). Midspan deflection w lf^4 / (384 E I).
#[test]
fn d48_forces_and_deflections_at_the_stations_of_a_fixed_beam_with_offsets() {
    for rz in [0.0, 0.5, 1.0] {
        let arm = rz * OFFSET;
        let lf = L - 2.0 * arm;
        let mut model = beam(OFFSET, OFFSET, rz);
        analyze_model(&mut model).unwrap();
        let st = stations(&model);
        for (k, s) in st.iter().enumerate() {
            let xp = s.station - arm;
            let what = format!("rz {rz} station {k}");
            let m3 = -lf * lf / 12.0 + 0.5 * lf * xp - 0.5 * xp * xp;
            assert_close(s.m3, m3, 1e-9, &format!("{what}: M3"));
            assert_close(s.v2, -(0.5 * lf - xp), 1e-9, &format!("{what}: V2"));
            // Deflection of the fixed-fixed beam, down: w x'^2 (lf - x')^2 / (24 E I).
            let w = xp * xp * (lf - xp) * (lf - xp) / (24.0 * E * I33);
            assert_close(s.local_disp[1], -w, 1e-9, &format!("{what}: u2"));
        }
    }
}

/// A cantilever fixed at joint 1 with a rigid offset of 12 at its free end J
/// (rigid-end factor one) and a load Q = 10 down at joint 2, the tip of the arm. The flexible part
/// runs from joint 1 to the face at 132 (lf); the load acts on the arm, so the
/// face carries the force Q and the moment 12 Q (hogging). Cantilever closed forms
/// with the tip load Q and the tip moment M = 12 Q at x = lf (no shear
/// deformation): deflection w(x) = Q x^2 (3 lf - x) / (6 E I) + M x^2 / (2 E I) and
/// slope theta(x) = Q x (2 lf - x) / (2 E I) + M x / (E I). The joint rotates by
/// theta(lf), so the joint deflection is w(lf) + 12 theta(lf) and the last station
/// (the face) reports w(lf), the difference being the rotation of the joint times
/// the rigid arm. M3(x) = -Q (144 - x), V2 = -Q (dM3/dx = -V2).
#[test]
fn d48_station_deflections_follow_the_rotation_of_a_joint_with_an_offset() {
    let (q, lf, arm) = (10.0f64, 132.0f64, 12.0f64);
    let m = q * arm;
    let w = |x: f64| q * x * x * (3.0 * lf - x) / (6.0 * E * I33) + m * x * x / (2.0 * E * I33);
    let theta = |x: f64| q * x * (2.0 * lf - x) / (2.0 * E * I33) + m * x / (E * I33);
    let mut model = beam(0.0, arm, 1.0);
    model.points.get_mut("2").unwrap().restraint = [false; 6];
    let frame = model.frames.get_mut("F").unwrap();
    frame.loads_dist.clear();
    model.load_patterns.insert(
        "Q".into(),
        LoadPattern {
            name: "Q".into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
    model
        .load_cases
        .insert("Q".into(), LoadCase::linear_static("Q", "Q", 1.0));
    model.points.get_mut("2").unwrap().loads_force.push(PointLoadForce {
        load_pat: "Q".into(),
        csys: "GLOBAL".into(),
        force: [0.0, 0.0, -q, 0.0, 0.0, 0.0],
    });
    analyze_model(&mut model).unwrap();
    let joint = model.case_results["Q"].joint_displacements["2"];
    assert_close(joint[2], -(w(lf) + arm * theta(lf)), 1e-9, "joint 2 deflection");
    let st = &model.case_results["Q"].frame_forces["F"];
    assert_eq!(st.len(), 5);
    for (k, s) in st.iter().enumerate() {
        let x = k as f64 * lf / 4.0;
        let what = format!("station {k} (x = {x})");
        assert_close(s.station, x, 1e-12, &format!("{what}: position"));
        assert_close(s.local_disp[1], -w(x), 1e-9, &format!("{what}: u2"));
        assert_close(s.m3, -q * (lf + arm - x), 1e-9, &format!("{what}: M3"));
        assert_close(s.v2, -q, 1e-9, &format!("{what}: V2"));
    }
    // The last station is the face: the joint minus the rotation times the arm.
    assert_close(
        st[4].local_disp[1],
        joint[2] + arm * theta(lf),
        1e-9,
        "face deflection = joint deflection + arm x rotation",
    );
}
