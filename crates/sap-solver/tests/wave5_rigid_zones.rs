//! Wave 5 steps 1b and 1c (D21, D28): rigid end zones.
//!
//! Step 1b. Analysis Reference p. 130: "The rigid-zone offsets never affect
//! axial and torsional deformation. The full element length is assumed to be
//! flexible for these deformations." The references are the closed forms
//! delta = P L / (E A) and theta = T L / (G J) of a bar of length L between its
//! joints, with the rigid-zone factor 1 and offsets at both ends. For a
//! non-prismatic section the properties are constant within the offset at each
//! end (p. 130, "Effect upon Non-prismatic Elements"), so the flexibility of the
//! bar is the sum of the flexibilities of its pieces, each exact for a section
//! constant over the piece.
//!
//! Step 1c. pp. 130-131: "If a moment or shear release is specified in either
//! bending plane at either end of the element, the end offset is assumed to be
//! rigid for bending and shear in that plane at that end (i.e., it acts as if
//! rigid = 1)."

use sap_model::*;
use sap_solver::analyze_model;

const E: f64 = 4320.0;
const G: f64 = 1800.0;

fn material(model: &mut SapModel) {
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.e = E;
    mat.u = 0.2;
    mat.g = G;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0;
    model.materials.insert("MAT".into(), mat);
}

fn section(name: &str, area: f64, j: f64) -> FrameSection {
    FrameSection {
        name: name.into(),
        material: "MAT".into(),
        area,
        i22: 1000.0,
        i33: 1000.0,
        torsional_j: j,
        as2: 0.0,
        as3: 0.0,
        ..FrameSection::default()
    }
}

/// A bar J1 (fixed) - J2 (free) along X, length 144, with the patterns AX
/// (10 kip along X at J2) and TQ (100 kip-in about X at J2), one linear static
/// case each.
fn bar(section_name: &str, offset_i: f64, offset_j: f64, rz: f64) -> SapModel {
    let mut model = SapModel::default();
    model.load_cases.clear();
    model.load_patterns.clear();
    material(&mut model);
    model
        .frame_sections
        .insert("PRISM".into(), section("PRISM", 216.0, 1000.0));
    model
        .frame_sections
        .insert("S1".into(), section("S1", 200.0, 2000.0));
    model
        .frame_sections
        .insert("S2".into(), section("S2", 100.0, 1000.0));
    let mut variable = section("VAR", 200.0, 2000.0);
    variable.prop_type = FramePropType::Variable;
    variable.non_prismatic = vec![
        NonPrismaticSegment {
            start_sec: "S1".into(),
            end_sec: "S1".into(),
            length: 1.0,
            is_variable_length: true,
            ei33_variation: 1,
            ei22_variation: 1,
        },
        NonPrismaticSegment {
            start_sec: "S2".into(),
            end_sec: "S2".into(),
            length: 1.0,
            is_variable_length: true,
            ei33_variation: 1,
            ei22_variation: 1,
        },
    ];
    model.frame_sections.insert("VAR".into(), variable);
    model.add_point("J1", 0.0, 0.0, 0.0);
    model.add_point("J2", 144.0, 0.0, 0.0);
    model.points.get_mut("J1").unwrap().restraint = [true; 6];
    model.add_frame("F", "J1", "J2", section_name);
    let frame = model.frames.get_mut("F").unwrap();
    frame.end_offset_i = offset_i;
    frame.end_offset_j = offset_j;
    frame.rigid_zone_factor = rz;
    for (pattern, force) in [
        ("AX", [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ("TQ", [0.0, 0.0, 0.0, 100.0, 0.0, 0.0]),
    ] {
        model.load_patterns.insert(
            pattern.into(),
            LoadPattern {
                name: pattern.into(),
                pat_type: LoadPatternType::Other,
                self_wt_multiplier: 0.0,
            },
        );
        model
            .load_cases
            .insert(pattern.into(), LoadCase::linear_static(pattern, pattern, 1.0));
        model.points.get_mut("J2").unwrap().loads_force.push(PointLoadForce {
            load_pat: pattern.into(),
            csys: "GLOBAL".into(),
            force,
        });
    }
    model
}

fn tip(model: &SapModel, case: &str) -> [f64; 6] {
    model.case_results[case].joint_displacements["J2"]
}

fn assert_rel(actual: f64, expected: f64, tol: f64, what: &str) {
    assert!(
        (actual - expected).abs() <= tol * expected.abs(),
        "{what}: {actual:.12e}, expected {expected:.12e}"
    );
}

/// R5 and R6: E 4320, A 216, G 1800, J 1000, L 144, offsets 6 and 6, rz 1.
/// delta = 10 * 144 / (4320 * 216) = 1.543209877e-3 in, and
/// theta = 100 * 144 / (1800 * 1000) = 8.0e-3 rad. The flexible length
/// (132 in) would give 1.414609053e-3 in and 7.333333333e-3 rad. The closed
/// forms are exact for this model; 1e-12 is the roundoff of the solution.
#[test]
fn r5_r6_axial_and_torsional_stiffness_use_the_full_length() {
    let mut model = bar("PRISM", 6.0, 6.0, 1.0);
    analyze_model(&mut model).unwrap();
    assert_rel(tip(&model, "AX")[0], 10.0 * 144.0 / (E * 216.0), 1e-12, "axial");
    assert_rel(tip(&model, "TQ")[3], 100.0 * 144.0 / (G * 1000.0), 1e-12, "torsion");
    // Without offsets the same bar gives the same values.
    let mut plain = bar("PRISM", 0.0, 0.0, 1.0);
    analyze_model(&mut plain).unwrap();
    assert_rel(tip(&plain, "AX")[0], 10.0 * 144.0 / (E * 216.0), 1e-12, "axial, no offsets");
    assert_rel(tip(&plain, "TQ")[3], 100.0 * 144.0 / (G * 1000.0), 1e-12, "torsion, no offsets");
}

/// The rigid-zone factor scales the rigid length for bending only; the axial
/// and torsional stiffness stay those of the full length for every factor.
#[test]
fn r5b_the_rigid_zone_factor_does_not_change_axial_or_torsion() {
    for rz in [0.0, 0.5, 1.0] {
        let mut model = bar("PRISM", 6.0, 6.0, rz);
        analyze_model(&mut model).unwrap();
        assert_rel(tip(&model, "AX")[0], 10.0 * 144.0 / (E * 216.0), 1e-12, "axial");
        assert_rel(tip(&model, "TQ")[3], 100.0 * 144.0 / (G * 1000.0), 1e-12, "torsion");
    }
}

/// Non-prismatic section of two segments of equal length over the flexible
/// length 132 in, A = 200 then 100 in2 (J = 2000 then 1000 in4), constant
/// within each segment and within the offsets at the ends (p. 130):
/// delta = 10 (6/200 + 66/200 + 66/100 + 6/100) / 4320 = 2.5e-3 in and
/// theta = 100 (6/2000 + 66/2000 + 66/1000 + 6/1000) / 1800 = 6.0e-3 rad.
#[test]
fn r5c_a_non_prismatic_bar_adds_the_end_zones_with_the_end_sections() {
    let mut model = bar("VAR", 6.0, 6.0, 1.0);
    analyze_model(&mut model).unwrap();
    let delta = 10.0 * (6.0 / 200.0 + 66.0 / 200.0 + 66.0 / 100.0 + 6.0 / 100.0) / E;
    let theta = 100.0 * (6.0 / 2000.0 + 66.0 / 2000.0 + 66.0 / 1000.0 + 6.0 / 1000.0) / G;
    assert_rel(delta, 2.5e-3, 1e-12, "the closed form itself");
    assert_rel(theta, 6.0e-3, 1e-12, "the closed form itself");
    assert_rel(tip(&model, "AX")[0], delta, 1e-12, "axial");
    assert_rel(tip(&model, "TQ")[3], theta, 1e-12, "torsion");
}

/// Examples 6-006 and 6-007 (vendor corpus, opt-in): the published periods of
/// the three-storey frame with rigid-zone factor 0.6 are 0.438, 0.135 and
/// 0.074 s (three printed digits, so at most 0.0005 s of rounding, that is
/// 0.11 %, 0.37 % and 0.68 %). With the axial stiffness of the columns and
/// braces on the flexible length the periods were 0.4309, 0.1303 and 0.0745 s
/// (-1.6 %, -3.5 %, +0.6 %); with it on the full length (p. 130) they were
/// 0.4391, 0.1348 and 0.0736 s, and with the Ritz projection repaired in the
/// same wave (D37) they are 0.4373, 0.1350 and 0.0737 s (-0.17 %, +0.02 %,
/// -0.39 %), equal to the Eigen solution of the same model. The second and
/// third are inside the printed rounding; the first is 0.0002 s below it, and
/// that remainder is not explained (the vendor's Ritz basis also has the link
/// starting vectors, see below). The tolerance of 1 % bounds the three printed
/// digits and that open remainder; it is not a claim of agreement closer than
/// that.
#[cfg(feature = "vendor-fixtures")]
#[test]
fn vendor_6_006_and_6_007_periods_follow_the_published_values_with_full_length_axial_stiffness() {
    use std::path::{Path, PathBuf};
    let dir = std::env::var_os("STRUCTURAX_VENDOR_FIXTURES")
        .map(PathBuf::from)
        .unwrap_or_else(|| Path::new(env!("CARGO_MANIFEST_DIR")).join("../../extracted/files"));
    for fname in ["example_6_006.sdb", "example_6_007.sdb"] {
        let path = dir.join(fname);
        if !path.exists() {
            continue;
        }
        let mut model = sap_format::load_sdb_file(&path).expect(fname);
        // The vendor's Ritz case also starts from the nonlinear degrees of freedom of the
        // links, which this solver does not have: the importer records that as unmodeled
        // source data and the Modal case is refused as imported (D36). This test measures
        // the effect of the axial length with the acceleration starting vector alone, so it
        // removes that entry explicitly.
        let before = model.unmodeled_source.len();
        model
            .unmodeled_source
            .retain(|u| !u.description.contains("Ritz starting load of type 4"));
        assert_eq!(before - model.unmodeled_source.len(), 1, "{fname}");
        for case in model.load_cases.values_mut() {
            case.run_flag = case.case_type == LoadCaseType::Modal;
        }
        analyze_model(&mut model).expect(fname);
        let modes = &model.case_results["MODAL"].modes;
        for (mode, published) in modes.iter().zip([0.438, 0.135, 0.074]) {
            assert!(
                (mode.period_sec - published).abs() <= 0.01 * published,
                "{fname}: mode {} period {}, published {published}",
                mode.mode_num,
                mode.period_sec
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Step 1c (D28): a released end is rigid in that plane (pp. 130-131).
// ---------------------------------------------------------------------------

/// Beam along X, L 120 in, E 29000 ksi, I33 100 in4, no shear deformation
/// (As2 = 0), offset 12 in at end I with rigid-zone factor 0 and the given
/// release at end I (indices of release_i: 1 = U2, 5 = R3). The local 2 axis of
/// a horizontal member is global Z and its local 3 axis is global -Y, so the
/// bending in the 1-2 plane is bending about global Y: joint 1 is fixed,
/// joint 2 is restrained in UZ only, and the moment 1000 kip-in about global Y
/// acts at joint 2 (the rotation about Y is read with its sign dropped).
fn propped_beam(release_at_i: &[usize]) -> SapModel {
    let mut model = SapModel::default();
    model.load_cases.clear();
    model.load_patterns.clear();
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.e = 29_000.0;
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
            i22: 100.0,
            i33: 100.0,
            torsional_j: 10.0,
            as2: 0.0,
            as3: 0.0,
            ..FrameSection::default()
        },
    );
    model.add_point("1", 0.0, 0.0, 0.0);
    model.add_point("2", 120.0, 0.0, 0.0);
    model.points.get_mut("1").unwrap().restraint = [true; 6];
    model.points.get_mut("2").unwrap().restraint = [false, false, true, false, false, false];
    model.add_frame("F", "1", "2", "S");
    let frame = model.frames.get_mut("F").unwrap();
    frame.end_offset_i = 12.0;
    frame.rigid_zone_factor = 0.0;
    for &dof in release_at_i {
        frame.release_i[dof] = true;
    }
    model.load_patterns.insert(
        "M".into(),
        LoadPattern {
            name: "M".into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
    model.load_cases.insert("M".into(), LoadCase::linear_static("M", "M", 1.0));
    model.points.get_mut("2").unwrap().loads_force.push(PointLoadForce {
        load_pat: "M".into(),
        csys: "GLOBAL".into(),
        force: [0.0, 0.0, 0.0, 0.0, 1000.0, 0.0],
    });
    model
}

/// R8: a moment release at I makes the 12 in offset rigid although rz = 0, so
/// the beam is pinned at 12 in and spans 108 in: theta_J = M (L - a) / (3 E I)
/// = 1000 * 108 / (3 * 29000 * 100) = 0.0124137931 rad (the flexible length
/// taken as 120 in would give 0.0137931034). The closed form is exact; 1e-12
/// is the roundoff of the solution.
#[test]
fn r8_a_moment_release_makes_the_offset_rigid_in_its_plane() {
    let mut model = propped_beam(&[5]);
    analyze_model(&mut model).unwrap();
    let theta = model.case_results["M"].joint_displacements["2"][4].abs();
    assert_rel(theta, 1000.0 * 108.0 / (3.0 * 29_000.0 * 100.0), 1e-12, "theta_J");
}

/// A shear release (U2) at I does the same: the end is guided (rotation fixed
/// by the joint, vertical slip free), so the moment is constant, the
/// curvature M/EI is constant over the 108 in and theta_J = M (L - a) / (E I)
/// = 1000 * 108 / (29000 * 100) = 0.0372413793 rad.
#[test]
fn r8b_a_shear_release_makes_the_offset_rigid_in_its_plane() {
    let mut model = propped_beam(&[1]);
    analyze_model(&mut model).unwrap();
    let theta = model.case_results["M"].joint_displacements["2"][4].abs();
    assert_rel(theta, 1000.0 * 108.0 / (29_000.0 * 100.0), 1e-12, "theta_J");
}

/// Without a release the offset with rz = 0 is flexible: the beam spans the
/// full 120 in, fixed at joint 1 and roller at joint 2, and the rotation of a
/// propped cantilever under a moment at its propped end is theta_J = M L / (4 E I).
#[test]
fn r8c_without_a_release_the_offset_stays_flexible() {
    let mut model = propped_beam(&[]);
    analyze_model(&mut model).unwrap();
    let theta = model.case_results["M"].joint_displacements["2"][4].abs();
    assert_rel(theta, 1000.0 * 120.0 / (4.0 * 29_000.0 * 100.0), 1e-12, "theta_J");
}

/// The rigid length per plane: full offset for a release of moment or shear
/// in that plane (pp. 130-131), the factor times the offset otherwise, and the
/// factor also for a partial-fixity spring (the manual does not say it counts;
/// a project choice).
#[test]
fn rigid_zones_per_plane_follow_the_releases() {
    use sap_solver::frame::rigid_zones;
    let mut frame = FrameObj::new("F", "1", "2", "S");
    frame.end_offset_i = 12.0;
    frame.end_offset_j = 8.0;
    frame.rigid_zone_factor = 0.5;
    let plain = rigid_zones(&frame, 120.0);
    assert_eq!((plain.di, plain.dj, plain.lf), (6.0, 4.0, 110.0));
    assert_eq!(plain.arm_i, [6.0, 6.0]);
    assert_eq!(plain.flex, [110.0, 110.0]);
    // R3 released at I: plane 1-2 only.
    frame.release_i[5] = true;
    let zones = rigid_zones(&frame, 120.0);
    assert_eq!(zones.arm_i, [12.0, 6.0]);
    assert_eq!(zones.arm_j, [4.0, 4.0]);
    assert_eq!(zones.flex, [120.0 - 12.0 - 4.0, 110.0]);
    // U3 released at J: plane 1-3 only.
    frame.release_i[5] = false;
    frame.release_j[2] = true;
    let zones = rigid_zones(&frame, 120.0);
    assert_eq!(zones.arm_j, [4.0, 8.0]);
    assert_eq!(zones.flex, [110.0, 120.0 - 6.0 - 8.0]);
    // A partial-fixity spring keeps the factor.
    frame.release_j[2] = false;
    frame.start_spring[5] = 1.0e4;
    frame.release_i[5] = true;
    let zones = rigid_zones(&frame, 120.0);
    assert_eq!(zones.arm_i, [6.0, 6.0]);
    // Axial and torsional lengths do not depend on any release.
    assert_eq!((zones.di, zones.dj, zones.lf), (6.0, 4.0, 110.0));
}

/// Negative: an unstable release set is refused as before. Shear (local 2 =
/// global Z) released at both ends leaves joint 2 without vertical stiffness, so
/// a vertical load on it is refused with the joint and the degree of freedom
/// named.
#[test]
fn an_unstable_release_set_is_still_refused() {
    let mut model = propped_beam(&[1]);
    model.frames.get_mut("F").unwrap().release_j[1] = true;
    model.points.get_mut("2").unwrap().restraint = [false; 6];
    model.points.get_mut("2").unwrap().loads_force[0].force = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0];
    let error = analyze_model(&mut model).unwrap_err().to_string();
    assert!(
        error.starts_with("Singular matrix: the structure is unstable at joint '2' U3"),
        "{error}"
    );
}
