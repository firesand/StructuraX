//! Frame insertion points (cardinal point and joint offsets) with stiffness
//! transformation: the element is set off from its joints on rigid arms.
//! Published values, closed forms, and the combinations that are refused.
//! Units are kip, in.
use sap_model::*;
use sap_section::make_rectangular;
use sap_solver::analyze_model;

const E: f64 = 3600.0;
const LENGTH: f64 = 120.0;

/// A cantilever along +X, fixed at joint 1, with a 12 x 18 in concrete section
/// (18 in deep along local 2, which is global Z for this member).
fn cantilever() -> SapModel {
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    let mut material = Material::default_steel_kip_in();
    material.name = "CONC".into();
    material.e = E;
    material.g = 1500.0;
    material.unit_weight = 0.0;
    material.unit_mass = 0.0;
    m.materials.insert("CONC".into(), material);
    m.frame_sections
        .insert("B12X18".into(), make_rectangular("B12X18", "CONC", 18.0, 12.0));
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", LENGTH, 0.0, 0.0);
    m.points["1"].restraint = [true; 6];
    m.add_frame("1", "1", "2", "B12X18");
    m.frames["1"].min_stations = 9;
    m
}

fn add_case(m: &mut SapModel, name: &str) {
    m.load_patterns.insert(
        name.into(),
        LoadPattern {
            name: name.into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
    m.load_cases
        .insert(name.into(), LoadCase::linear_static(name, name, 1.0));
}

fn joint_load(m: &mut SapModel, joint: &str, pattern: &str, force: [f64; 6]) {
    m.points[joint].loads_force.push(PointLoadForce {
        load_pat: pattern.into(),
        csys: "GLOBAL".into(),
        force,
    });
}

// ---------------------------------------------------------------------------
// CSI Software Verification Example 1-011: frame insertion point.
// ---------------------------------------------------------------------------

/// Cardinal point 1 (bottom left) and joint offsets of +12 in along global Z
/// at both ends. Case 1 applies 10 kip of compression to the joint at the
/// free end, case 2 to the free end of the frame object.
fn example_1_011() -> SapModel {
    let mut m = cantilever();
    let frame = &mut m.frames["1"];
    frame.cardinal_point = 1;
    frame.stiff_transform = true;
    frame.joint_offset_is_global = false;
    frame.joint_offset_i = [0.0, 12.0, 0.0];
    frame.joint_offset_j = [0.0, 12.0, 0.0];
    add_case(&mut m, "1");
    joint_load(&mut m, "2", "1", [-10.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    add_case(&mut m, "2");
    m.frames["1"].loads_point.push(FrameLoadPoint {
        load_pat: "2".into(),
        my_type: 1,
        dir: 4,
        csys: "GLOBAL".into(),
        rel_dist: 1.0,
        val: -10.0,
    });
    m
}

#[test]
fn example_1_011_moments_match_the_published_values() {
    let mut m = example_1_011();
    analyze_model(&mut m).unwrap();

    // Case 2 fixes the geometry by statics alone: the load acts at the
    // section centroid, 21 in above the joints (12 in offset plus half the
    // depth above the bottom) and 6 in to the right of the left edge, which
    // is the -Y side for this member. Published fixed-end moments: My = 210,
    // Mz = 60 kip-in; no moment in the beam.
    let case2 = &m.case_results["2"];
    let reaction = case2.joint_reactions["1"];
    assert!((reaction[0] - 10.0).abs() < 1e-9, "{reaction:?}");
    assert!((reaction[4] - 210.0).abs() < 1e-9, "{reaction:?}");
    assert!((reaction[5] - 60.0).abs() < 1e-9, "{reaction:?}");
    assert!(reaction[1].abs() < 1e-9 && reaction[2].abs() < 1e-9 && reaction[3].abs() < 1e-9);
    for station in &case2.frame_forces["1"] {
        // The station at the load itself reports the force beyond it.
        let axial = if station.rel_dist < 1.0 { -10.0 } else { 0.0 };
        assert!((station.p - axial).abs() < 1e-9, "{station:?}");
        assert!(station.m3.abs() < 1e-8 && station.m2.abs() < 1e-8, "{station:?}");
    }

    // Case 1: the load acts on the line between the joints, so the support
    // carries no moment, while the beam, off that line, carries M3 = -210
    // and M2 = -60 kip-in over its whole length (published as "beam max").
    let case1 = &m.case_results["1"];
    let reaction = case1.joint_reactions["1"];
    assert!((reaction[0] - 10.0).abs() < 1e-9, "{reaction:?}");
    assert!(reaction[1..].iter().all(|r| r.abs() < 1e-8), "{reaction:?}");
    let stations = &case1.frame_forces["1"];
    assert_eq!(stations.len(), 9);
    for station in stations {
        assert!((station.p + 10.0).abs() < 1e-9, "{station:?}");
        assert!((station.m3 + 210.0).abs() < 1e-8, "{station:?}");
        assert!((station.m2 + 60.0).abs() < 1e-8, "{station:?}");
        assert!(station.v2.abs() < 1e-9 && station.v3.abs() < 1e-9 && station.t.abs() < 1e-9);
    }
}

// ---------------------------------------------------------------------------
// Closed forms for a cantilever loaded at its end joint.
// ---------------------------------------------------------------------------

/// End-joint displacement of a cantilever whose centroid is `e2` along local 2
/// and `e3` along local 3 from the joints, under an axial force `p` at the
/// end joint (positive along local 1, so tension).
///
/// The force acts at (-e2, -e3) from the centroid, so the member carries the
/// constant moment vectors p e2 about local 3 and -p e3 about local 2. Its
/// end deflects by p e2 L^2 / (2 E I33) along local 2 and p e3 L^2 / (2 E I22)
/// along local 3 (a tension applied below the centroid bends the end down,
/// toward the load line), and stretches by p L / (E A). The joint follows the
/// rotated end section on its rigid arm, which adds p e^2 L / (E I) along the
/// axis for each plane and nothing across it.
///
/// Returns the joint displacement along local 1, 2 and 3.
fn eccentric_axial_tip(p: f64, e2: f64, e3: f64, area: f64, i33: f64, i22: f64) -> [f64; 3] {
    let l = LENGTH;
    [
        p * l / (E * area) + p * e2 * e2 * l / (E * i33) + p * e3 * e3 * l / (E * i22),
        p * e2 * l * l / (2.0 * E * i33),
        p * e3 * l * l / (2.0 * E * i22),
    ]
}

#[test]
fn an_eccentric_axial_load_gives_the_closed_form_joint_displacement() {
    let (area, i33, i22) = (216.0, 5832.0, 2592.0);
    // (cardinal point, local offset at both ends, centroid e2, e3)
    let cases = [
        (8, [0.0, 0.0, 0.0], -9.0, 0.0),
        (2, [0.0, 0.0, 0.0], 9.0, 0.0),
        (4, [0.0, 0.0, 0.0], 0.0, 6.0),
        (6, [0.0, 0.0, 0.0], 0.0, -6.0),
        (1, [0.0, 12.0, 0.0], 21.0, 6.0),
        (9, [0.0, -2.0, 5.0], -11.0, -1.0),
        (10, [0.0, 4.0, -3.0], 4.0, -3.0),
        // Middle center and shear center are the centroid of this section.
        (5, [0.0, 0.0, 0.0], 0.0, 0.0),
        (11, [0.0, 0.0, 0.0], 0.0, 0.0),
    ];
    for (cardinal, offset, e2, e3) in cases {
        let mut m = cantilever();
        let frame = &mut m.frames["1"];
        frame.cardinal_point = cardinal;
        frame.joint_offset_i = offset;
        frame.joint_offset_j = offset;
        frame.joint_offset_is_global = false;
        add_case(&mut m, "P");
        let p = -25.0;
        joint_load(&mut m, "2", "P", [p, 0.0, 0.0, 0.0, 0.0, 0.0]);
        analyze_model(&mut m).unwrap_or_else(|e| panic!("cardinal {cardinal}: {e}"));
        let tip = m.case_results["P"].joint_displacements["2"];
        // Local 1, 2, 3 of this member are global X, Z, -Y.
        let expected = eccentric_axial_tip(p, e2, e3, area, i33, i22);
        let got = [tip[0], tip[2], -tip[1]];
        for d in 0..3 {
            assert!(
                (got[d] - expected[d]).abs() < 1e-10 * expected[0].abs(),
                "cardinal {cardinal} local {}: {} vs {}",
                d + 1,
                got[d],
                expected[d]
            );
        }
    }
}

#[test]
fn the_offset_follows_the_member_axes_and_either_offset_system() {
    let (area, i33, i22) = (216.0, 5832.0, 2592.0);
    let p = -25.0;
    let expected = eccentric_axial_tip(p, 21.0, 6.0, area, i33, i22);

    // The same insertion point given by a global joint offset (+12 in Z).
    let mut m = cantilever();
    m.frames["1"].cardinal_point = 1;
    m.frames["1"].joint_offset_is_global = true;
    m.frames["1"].joint_offset_i = [0.0, 0.0, 12.0];
    m.frames["1"].joint_offset_j = [0.0, 0.0, 12.0];
    add_case(&mut m, "P");
    joint_load(&mut m, "2", "P", [p, 0.0, 0.0, 0.0, 0.0, 0.0]);
    analyze_model(&mut m).unwrap();
    let tip = m.case_results["P"].joint_displacements["2"];
    for (got, want) in [tip[0], tip[2], -tip[1]].into_iter().zip(expected) {
        assert!((got - want).abs() < 1e-10 * expected[0].abs(), "{got} vs {want}");
    }

    // Rolled 90 degrees about its axis: local 2 is now global -Y and local 3
    // global -Z, and the local offset turns with the section.
    let mut m = cantilever();
    m.frames["1"].angle_deg = 90.0;
    m.frames["1"].cardinal_point = 1;
    m.frames["1"].joint_offset_is_global = false;
    m.frames["1"].joint_offset_i = [0.0, 12.0, 0.0];
    m.frames["1"].joint_offset_j = [0.0, 12.0, 0.0];
    add_case(&mut m, "P");
    joint_load(&mut m, "2", "P", [p, 0.0, 0.0, 0.0, 0.0, 0.0]);
    analyze_model(&mut m).unwrap();
    let tip = m.case_results["P"].joint_displacements["2"];
    let along_local_2 = [tip[1], tip[2]];
    let rolled = [tip[0], expected[1], expected[2]];
    assert!((tip[0] - expected[0]).abs() < 1e-10 * expected[0].abs());
    // The two transverse components keep their magnitudes in the rolled axes.
    let magnitudes = |a: f64, b: f64| {
        let mut v = [a.abs(), b.abs()];
        v.sort_by(f64::total_cmp);
        v
    };
    let (got, want) = (
        magnitudes(along_local_2[0], along_local_2[1]),
        magnitudes(rolled[1], rolled[2]),
    );
    assert!((got[0] - want[0]).abs() < 1e-10 * expected[0].abs(), "{got:?} vs {want:?}");
    assert!((got[1] - want[1]).abs() < 1e-10 * expected[0].abs(), "{got:?} vs {want:?}");
    // The larger one is the bending about the strong axis, now sideways.
    assert!(tip[1].abs() > tip[2].abs());

    // A vertical member (a column fixed at its base) with its top-center
    // cardinal point: local 1 is global Z, and the eccentric axial load at
    // the top joint bends it by the same closed form.
    let mut m = cantilever();
    m.points["2"].x = 0.0;
    m.points["2"].z = LENGTH;
    m.frames["1"].cardinal_point = 8;
    add_case(&mut m, "P");
    joint_load(&mut m, "2", "P", [0.0, 0.0, p, 0.0, 0.0, 0.0]);
    analyze_model(&mut m).unwrap();
    let tip = m.case_results["P"].joint_displacements["2"];
    let expected = eccentric_axial_tip(p, -9.0, 0.0, area, i33, i22);
    assert!((tip[2] - expected[0]).abs() < 1e-10 * expected[0].abs(), "{tip:?}");
    let sideways = (tip[0] * tip[0] + tip[1] * tip[1]).sqrt();
    assert!((sideways - expected[1].abs()).abs() < 1e-10 * expected[0].abs(), "{tip:?}");
}

#[test]
fn span_loads_act_on_the_offset_element_and_subdivision_changes_nothing() {
    // A uniform load along the member axis (global X) on the frame object
    // acts at the centroid, 9 in below the joints for the top-center
    // cardinal point. Statics at the support: Fx = -q L and a moment
    // q L e about global Y.
    let (q, e2) = (0.05, -9.0);
    let build = |segments: usize| {
        let mut m = cantilever();
        m.frames["1"].cardinal_point = 8;
        m.frames["1"].auto_mesh_segments = segments;
        m.frames["1"].min_stations = 9;
        add_case(&mut m, "Q");
        m.frames["1"].loads_dist.push(FrameLoadDistributed {
            load_pat: "Q".into(),
            my_type: 1,
            dir: 4,
            csys: "GLOBAL".into(),
            rel_dist1: 0.0,
            rel_dist2: 1.0,
            val1: q,
            val2: q,
        });
        m.frames["1"].loads_point.push(FrameLoadPoint {
            load_pat: "Q".into(),
            my_type: 1,
            dir: 6,
            csys: "GLOBAL".into(),
            rel_dist: 0.4,
            val: -3.0,
        });
        analyze_model(&mut m).unwrap();
        m
    };
    let whole = build(0);
    let reference = &whole.case_results["Q"];
    let reaction = reference.joint_reactions["1"];
    assert!((reaction[0] + q * LENGTH).abs() < 1e-9, "{reaction:?}");
    assert!((reaction[2] - 3.0).abs() < 1e-9, "{reaction:?}");
    // About Y at the support: the axial load at height e2, and the point
    // load 48 in out.
    let expected = -(q * LENGTH * e2) - 3.0 * 0.4 * LENGTH;
    assert!((reaction[4] - expected).abs() < 1e-8, "{} vs {expected}", reaction[4]);

    for segments in [2, 4] {
        let meshed = build(segments);
        let results = &meshed.case_results["Q"];
        for d in 0..6 {
            let (x, y) = (reference.joint_displacements["2"][d], results.joint_displacements["2"][d]);
            assert!((x - y).abs() <= 1e-9 * x.abs().max(1e-6), "{segments}: u{d} {x} vs {y}");
            let (x, y) = (reference.joint_reactions["1"][d], results.joint_reactions["1"][d]);
            assert!((x - y).abs() <= 1e-9 * x.abs().max(1.0), "{segments}: r{d} {x} vs {y}");
        }
        // Shared station positions (every eighth of the length).
        for station in &reference.frame_forces["1"] {
            let twin = results.frame_forces["1"]
                .iter()
                .find(|s| (s.rel_dist - station.rel_dist).abs() < 1e-12)
                .unwrap();
            for (x, y) in [
                (station.p, twin.p),
                (station.v2, twin.v2),
                (station.m3, twin.m3),
                (station.local_disp[0], twin.local_disp[0]),
                (station.local_disp[1], twin.local_disp[1]),
            ] {
                assert!((x - y).abs() <= 1e-9 * x.abs().max(1.0), "{segments} at {}: {x} vs {y}", station.rel_dist);
            }
        }
    }
}

#[test]
fn releases_and_temperature_act_on_the_offset_element() {
    // Both joints fixed, top-center cardinal point (centroid 9 in below the
    // joints), no shear deformation.
    let build = || {
        let mut m = cantilever();
        m.frame_sections["B12X18"].as2 = 0.0;
        m.frame_sections["B12X18"].as3 = 0.0;
        m.points["2"].restraint = [true; 6];
        m.frames["1"].cardinal_point = 8;
        add_case(&mut m, "L");
        m
    };

    // A moment release at J makes a propped cantilever: the hinge is on the
    // member axis, so the offset changes nothing in the closed form. Under a
    // uniform load w the hinged end carries 3 w L / 8, the fixed end
    // 5 w L / 8 and w L^2 / 8, and the joint above the hinge no moment.
    let w = 0.02;
    let mut m = build();
    m.frames["1"].release_j[5] = true;
    m.frames["1"].loads_dist.push(FrameLoadDistributed {
        load_pat: "L".into(),
        my_type: 1,
        dir: 10,
        csys: "GLOBAL".into(),
        rel_dist1: 0.0,
        rel_dist2: 1.0,
        val1: w,
        val2: w,
    });
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["L"];
    assert!((r.joint_reactions["2"][2] - 3.0 * w * LENGTH / 8.0).abs() < 1e-10);
    assert!((r.joint_reactions["1"][2] - 5.0 * w * LENGTH / 8.0).abs() < 1e-10);
    assert!((r.joint_reactions["1"][4].abs() - w * LENGTH * LENGTH / 8.0).abs() < 1e-9);
    assert!(r.joint_reactions["2"][4].abs() < 1e-9);
    assert!(r.joint_reactions["1"][0].abs() < 1e-9 && r.joint_reactions["2"][0].abs() < 1e-9);
    let stations = &r.frame_forces["1"];
    assert!((stations[0].m3.abs() - w * LENGTH * LENGTH / 8.0).abs() < 1e-9);
    assert!(stations.last().unwrap().m3.abs() < 1e-9);

    // A uniform temperature rise in the member held between the two fixed
    // joints: axial force -E A alpha dT and no bending in the member, while
    // each joint, 9 in above the force, carries that force times 9 in.
    let (dt, area) = (50.0, 216.0);
    let mut m = build();
    let alpha = m.materials["CONC"].a;
    assert!(alpha > 0.0);
    m.frames["1"].loads_temp.push(FrameLoadTemperature {
        load_pat: "L".into(),
        my_type: 1,
        val: dt,
        pattern_name: None,
    });
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["L"];
    let force = E * area * alpha * dt;
    for station in &r.frame_forces["1"] {
        assert!((station.p + force).abs() < 1e-9 * force, "{station:?}");
        assert!(station.m3.abs() < 1e-7 && station.m2.abs() < 1e-7, "{station:?}");
    }
    let (left, right) = (r.joint_reactions["1"], r.joint_reactions["2"]);
    assert!((left[0] - force).abs() < 1e-9 * force && (right[0] + force).abs() < 1e-9 * force);
    assert!((left[4].abs() - 9.0 * force).abs() < 1e-8 * force, "{left:?}");
    assert!((left[4] + right[4]).abs() < 1e-8 * force, "{left:?} {right:?}");
}

#[test]
fn example_1_011_survives_a_unit_conversion() {
    // kip-in to kN-m: section dimensions and joint offsets convert with the
    // model, so the published moments convert too.
    let mut m = example_1_011();
    m.convert_units_in_place(sap_units::Units::KnMC);
    assert!((m.frames["1"].joint_offset_i[1] - 12.0 * 0.0254).abs() < 1e-12);
    analyze_model(&mut m).unwrap();
    let kn_m = 4.4482216152605 * 0.0254;
    for station in &m.case_results["1"].frame_forces["1"] {
        assert!((station.m3 + 210.0 * kn_m).abs() < 1e-8, "{station:?}");
        assert!((station.m2 + 60.0 * kn_m).abs() < 1e-8, "{station:?}");
    }
    let reaction = m.case_results["2"].joint_reactions["1"];
    assert!((reaction[4] - 210.0 * kn_m).abs() < 1e-8, "{reaction:?}");
    assert!((reaction[5] - 60.0 * kn_m).abs() < 1e-8, "{reaction:?}");
}

#[test]
fn without_the_transformation_the_frame_is_analyzed_at_its_centroid() {
    let build = || {
        let mut m = cantilever();
        add_case(&mut m, "P");
        joint_load(&mut m, "2", "P", [-10.0, 0.0, -2.0, 0.0, 0.0, 0.0]);
        m
    };
    let mut centroidal = build();
    analyze_model(&mut centroidal).unwrap();
    let reference = &centroidal.case_results["P"];
    let expected = -10.0 * LENGTH / (E * 216.0);
    assert!((reference.joint_displacements["2"][0] - expected).abs() < 1e-12 * expected.abs());

    // With the transformation off, the insertion point is not part of the
    // analysis model: the results are those of the centroidal frame, exactly.
    let mut m = build();
    let frame = &mut m.frames["1"];
    frame.cardinal_point = 1;
    frame.joint_offset_i = [0.0, 0.0, 12.0];
    frame.joint_offset_j = [3.0, 0.0, 7.0];
    frame.stiff_transform = false;
    analyze_model(&mut m).unwrap();
    assert_eq!(&m.case_results["P"], reference);

    // With it on, the same load gives a different answer.
    m.frames["1"].joint_offset_j = [0.0, 0.0, 12.0];
    m.frames["1"].joint_offset_is_global = true;
    m.frames["1"].stiff_transform = true;
    analyze_model(&mut m).unwrap();
    assert!(
        (m.case_results["P"].joint_displacements["2"][0] - expected).abs() > 0.1 * expected.abs()
    );
}

// ---------------------------------------------------------------------------
// Combinations that are refused instead of approximated.
// ---------------------------------------------------------------------------

#[test]
fn insertion_points_the_transformation_does_not_cover_are_refused() {
    type Change = fn(&mut SapModel);
    let cases: [(&str, Change); 13] = [
        ("it is a cable", |m| {
            m.frames["1"].cable_params = Some(CableParams {
                undeformed_length: Some(LENGTH),
                target_tension_i: None,
                horizontal_tension: None,
                num_segments: 1,
            });
        }),
        ("a non-prismatic section", |m| {
            // Centroid cardinal point, so only the joint offsets set it off.
            m.frames["1"].cardinal_point = 10;
            m.frames["1"].joint_offset_i = [0.0, 4.0, 0.0];
            m.frames["1"].joint_offset_j = [0.0, 4.0, 0.0];
            let mut variable = m.frame_sections["B12X18"].clone();
            variable.name = "VAR".into();
            variable.prop_type = FramePropType::Variable;
            variable.non_prismatic.push(NonPrismaticSegment {
                start_sec: "B12X18".into(),
                end_sec: "B12X18".into(),
                length: 1.0,
                is_variable_length: true,
                ei33_variation: 1,
                ei22_variation: 1,
            });
            m.frame_sections.insert("VAR".into(), variable);
            m.frames["1"].section = "VAR".into();
        }),
        ("joint offsets that differ between its ends", |m| {
            m.frames["1"].joint_offset_j = [0.0, 3.0, 0.0];
        }),
        ("a joint offset along its own axis", |m| {
            m.frames["1"].joint_offset_i = [2.0, 0.0, 0.0];
            m.frames["1"].joint_offset_j = [2.0, 0.0, 0.0];
        }),
        ("whose shape is not a doubly symmetric one", |m| {
            m.frame_sections["B12X18"].prop_type = FramePropType::T;
        }),
        ("whose shape is not a doubly symmetric one", |m| {
            m.frame_sections["B12X18"].prop_type = FramePropType::General;
        }),
        ("which has no outside dimensions", |m| {
            m.frame_sections["B12X18"].t2 = 0.0;
        }),
        ("cardinal point 12 is not one of 1 to 11", |m| {
            m.frames["1"].cardinal_point = 12;
        }),
        ("end length offsets", |m| {
            m.frames["1"].end_offset_i = 4.0;
        }),
        ("an assigned P-Delta force", |m| {
            m.frames["1"].pdelta_force = 5.0;
        }),
        ("automatic subdivision at joints", |m| {
            m.frames["1"].auto_mesh_at_joints = true;
        }),
        // Mass and geometric stiffness of the offset are not modeled.
        ("case 'MODAL' is not one", |m| {
            m.load_cases
                .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 2));
        }),
        ("case 'PD' is not one", |m| {
            let mut case = LoadCase::linear_static("PD", "P", 1.0);
            case.case_type = LoadCaseType::NonlinearStatic;
            case.nl_geom = NLGeomType::PDelta;
            m.load_cases.insert("PD".into(), case);
        }),
    ];
    for (expected, change) in cases {
        let mut m = cantilever();
        m.frames["1"].cardinal_point = 8;
        add_case(&mut m, "P");
        joint_load(&mut m, "2", "P", [-10.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        // The starting point analyzes.
        analyze_model(&mut m.clone()).unwrap();
        change(&mut m);
        let before = m.clone();
        let error = analyze_model(&mut m).unwrap_err().to_string();
        assert!(error.contains("Unsupported analysis input"), "{error}");
        assert!(error.contains("frame '1' has an insertion point"), "{error}");
        assert!(error.contains(expected), "{expected}: {error}");
        assert!(error.contains("FRAME INSERTION POINT ASSIGNMENTS"), "{error}");
        assert!(m.case_results.is_empty() && !m.is_locked);
        assert_eq!(m.frames, before.frames);
    }

    // A case that is switched off does not count.
    let mut m = cantilever();
    m.frames["1"].cardinal_point = 8;
    add_case(&mut m, "P");
    joint_load(&mut m, "2", "P", [-10.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    let mut modal = LoadCase::modal_eigen("MODAL", 2);
    modal.run_flag = false;
    m.load_cases.insert("MODAL".into(), modal);
    analyze_model(&mut m).unwrap();
}
