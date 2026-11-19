//! Wave 5 P13, step b (D47): non-prismatic members with end offsets.
//!
//! Analysis Reference p. 120: "The clear length of the element, Lc, is first
//! calculated as the total length minus the end offsets" and the segment lengths
//! are laid over it. p. 123 ("Effect upon End Offsets"): "Section properties
//! within end offset ioff are constant using the starting Section of the first
//! segment. Section properties within end offset joff are constant using the
//! ending Section of the last segment." p. 130: "Section properties vary only
//! along the clear length of the element between support faces. This is not
//! affected by the value of the rigid-end factor"; "The rigid-zone offsets never
//! affect axial and torsional deformation. The full element length is assumed to
//! be flexible for these deformations." p. 135: the self-weight "is constant
//! within the end offsets".
//!
//! The section VAR has two segments of equal variable length: S1 (A 200, I 2000,
//! J 2000) and S2 (A 100, I 1000, J 1000), each constant. Every reference is the
//! unit-load integral of a bar or beam with piecewise-constant properties,
//! written out in the test. Units: kip, in.

use sap_model::*;
use sap_solver::analyze_model;

const E: f64 = 4320.0;
const G: f64 = 1800.0;
const L: f64 = 144.0;
const ALPHA: f64 = 1.0e-5;
const GAMMA: f64 = 0.001;

fn assert_close(actual: f64, expected: f64, tol: f64, what: &str) {
    let scale = expected.abs().max(1e-12);
    assert!(
        (actual - expected).abs() <= tol * scale,
        "{what}: {actual} against {expected} (tolerance {tol} relative)"
    );
}

fn section(name: &str, area: f64, inertia: f64, j: f64) -> FrameSection {
    FrameSection {
        name: name.into(),
        material: "MAT".into(),
        area,
        i22: inertia,
        i33: inertia,
        torsional_j: j,
        as2: 0.0,
        as3: 0.0,
        ..FrameSection::default()
    }
}

/// A member J1 (origin) to J2 (144 along X) with the section VAR, the end
/// offsets and rigid-end factor given, J1 fixed, J2 fixed or free, and the
/// patterns AX (10 along X at J2), TQ (100 about X at J2), QZ (10 down at J2),
/// TH (temperature 100 over the member), SW (self weight) and PL (set by the
/// test) as one linear static case each.
fn member(
    section_name: &str,
    offset_i: f64,
    offset_j: f64,
    rz: f64,
    tip_fixed: bool,
) -> SapModel {
    let mut model = SapModel::default();
    model.load_cases.clear();
    model.load_patterns.clear();
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.e = E;
    mat.u = 0.2;
    mat.g = G;
    mat.a = ALPHA;
    mat.unit_mass = 0.0;
    mat.unit_weight = GAMMA;
    model.materials.insert("MAT".into(), mat);
    model
        .frame_sections
        .insert("PRISM".into(), section("PRISM", 216.0, 1000.0, 1000.0));
    model
        .frame_sections
        .insert("S1".into(), section("S1", 200.0, 2000.0, 2000.0));
    model
        .frame_sections
        .insert("S2".into(), section("S2", 100.0, 1000.0, 1000.0));
    let mut variable = section("VAR", 200.0, 2000.0, 2000.0);
    variable.prop_type = FramePropType::Variable;
    variable.non_prismatic = ["S1", "S2"]
        .into_iter()
        .map(|name| NonPrismaticSegment {
            start_sec: name.into(),
            end_sec: name.into(),
            length: 1.0,
            is_variable_length: true,
            ei33_variation: 1,
            ei22_variation: 1,
        })
        .collect();
    model.frame_sections.insert("VAR".into(), variable);
    model.add_point("J1", 0.0, 0.0, 0.0);
    model.add_point("J2", L, 0.0, 0.0);
    model.points.get_mut("J1").unwrap().restraint = [true; 6];
    if tip_fixed {
        model.points.get_mut("J2").unwrap().restraint = [true; 6];
    }
    model.add_frame("F", "J1", "J2", section_name);
    let frame = model.frames.get_mut("F").unwrap();
    frame.end_offset_i = offset_i;
    frame.end_offset_j = offset_j;
    frame.rigid_zone_factor = rz;
    frame.loads_temp.push(FrameLoadTemperature {
        load_pat: "TH".into(),
        my_type: 1,
        val: 100.0,
        pattern_name: None,
    });
    for (pattern, self_weight) in [
        ("AX", 0.0),
        ("TQ", 0.0),
        ("QZ", 0.0),
        ("TH", 0.0),
        ("SW", 1.0),
        ("PL", 0.0),
    ] {
        model.load_patterns.insert(
            pattern.into(),
            LoadPattern {
                name: pattern.into(),
                pat_type: LoadPatternType::Other,
                self_wt_multiplier: self_weight,
            },
        );
        model
            .load_cases
            .insert(pattern.into(), LoadCase::linear_static(pattern, pattern, 1.0));
    }
    if !tip_fixed {
        for (pattern, force) in [
            ("AX", [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            ("TQ", [0.0, 0.0, 0.0, 100.0, 0.0, 0.0]),
            ("QZ", [0.0, 0.0, -10.0, 0.0, 0.0, 0.0]),
        ] {
            model.points.get_mut("J2").unwrap().loads_force.push(PointLoadForce {
                load_pat: pattern.into(),
                csys: "GLOBAL".into(),
                force,
            });
        }
    }
    model
}

fn tip(model: &SapModel, case: &str) -> [f64; 6] {
    model.case_results[case].joint_displacements["J2"]
}

/// Review row 4: bar with a piecewise-constant section. The offsets 24 at I and
/// 12 at J leave the clear length 108, so the segments are 54 long: S1 from 24 to
/// 78, S2 from 78 to 132, and the end zone at J has S2. The whole length is
/// flexible for axial and torsional deformation whatever the rigid-end factor, so
/// delta = P (78 / (E A1) + 66 / (E A2)) and theta = T (78 / (G J1) + 66 / (G J2)).
/// Baseline: the segments were laid over the flexible length and the zones added
/// with the section of the end, giving 1.05 against 1.02 for the review's offset
/// 24 at I with rigid-end factor 0.5.
#[test]
fn d47_axial_and_torsional_flexibility_follow_the_clear_length_for_every_rigid_end_factor() {
    for rz in [0.0, 0.5, 1.0] {
        let mut model = member("VAR", 24.0, 12.0, rz, false);
        analyze_model(&mut model).unwrap();
        let delta = 10.0 * (78.0 / 200.0 + 66.0 / 100.0) / E;
        let theta = 100.0 * (78.0 / 2000.0 + 66.0 / 1000.0) / G;
        assert_close(tip(&model, "AX")[0], delta, 1e-12, &format!("rz {rz}: axial"));
        assert_close(tip(&model, "TQ")[3], theta, 1e-12, &format!("rz {rz}: torsion"));
    }
    // The review's own example: offset 24 at I only, rz 0.5: Lc = 120, S1 from 24
    // to 84 and S2 from 84 to 144; 84 / 200 + 60 / 100 = 1.02.
    let mut model = member("VAR", 24.0, 0.0, 0.5, false);
    analyze_model(&mut model).unwrap();
    assert_close(tip(&model, "AX")[0] * E / 10.0, 1.02, 1e-12, "review example");
}

/// Antiderivative of (a - x) (b - x): the unit-load integral of a cantilever
/// with a load at `a` and the deflection read at `b`, as M(x) m(x).
fn moment_area(a: f64, b: f64, lo: f64, hi: f64) -> f64 {
    let f = |x: f64| a * b * x - 0.5 * (a + b) * x * x + x * x * x / 3.0;
    f(hi) - f(lo)
}

/// Cantilever fixed at joint 1 with offset 24 at I: bending and shear use the
/// flexible part, which starts after the rigid arm rz x 24, and the section is S1
/// up to 84 (offset zone and first segment) and S2 beyond. With a tip load Q the
/// deflection is Q / E (int_x0^84 (144 - x)^2 dx / I1 + int_84^144 (144 - x)^2 dx /
/// I2) (no shear deformation; the fixed joint does not rotate, so the rigid arm
/// adds nothing).
#[test]
fn d47_bending_stiffness_uses_the_flexible_part_and_the_clear_length_sections() {
    for rz in [0.0, 0.5, 1.0] {
        let x0 = rz * 24.0;
        let mut model = member("VAR", 24.0, 0.0, rz, false);
        analyze_model(&mut model).unwrap();
        let delta = 10.0
            * (moment_area(144.0, 144.0, x0, 84.0) / 2000.0
                + moment_area(144.0, 144.0, 84.0, 144.0) / 1000.0)
            / E;
        assert_close(
            tip(&model, "QZ")[2],
            -delta,
            1e-10,
            &format!("rz {rz}: tip deflection"),
        );
    }
}

/// The same cantilever under a span point load P = 10 at 100 in from joint 1
/// (relative distance 100 / 144 of the full length): the tip deflection is
/// P / E (int_x0^84 (100 - x)(144 - x) dx / I1 + int_84^100 (100 - x)(144 - x) dx /
/// I2), the moment of the load at x times the moment of a unit tip load. This
/// checks the fixed-end forces of a non-prismatic member with the same zoned
/// model as the stiffness.
#[test]
fn d47_span_point_load_on_a_non_prismatic_cantilever_with_an_offset() {
    for rz in [0.0, 0.5, 1.0] {
        let x0 = rz * 24.0;
        let mut model = member("VAR", 24.0, 0.0, rz, false);
        model.frames.get_mut("F").unwrap().loads_point.push(FrameLoadPoint {
            load_pat: "PL".into(),
            my_type: 1,
            dir: 6,
            csys: "GLOBAL".into(),
            rel_dist: 100.0 / L,
            val: -10.0,
        });
        analyze_model(&mut model).unwrap();
        let delta = 10.0
            * (moment_area(100.0, 144.0, x0, 84.0) / 2000.0
                + moment_area(100.0, 144.0, 84.0, 100.0) / 1000.0)
            / E;
        assert_close(
            tip(&model, "PL")[2],
            -delta,
            1e-10,
            &format!("rz {rz}: tip deflection"),
        );
    }
}

/// Free thermal expansion of the unrestrained member is alpha dT L whatever the
/// section and the offsets (the axial stiffness of the element is the one of the
/// whole length, p. 130, and the fixed-end forces restrain with the same one).
/// Baseline (review row 3): 0.140 against 0.144 for VAR, offset 12 at I, rz 1.
#[test]
fn d47_free_thermal_expansion_is_alpha_dt_l_for_a_non_prismatic_member_with_offsets() {
    for (offset_i, offset_j, rz) in [
        (12.0, 0.0, 1.0),
        (24.0, 0.0, 0.5),
        (24.0, 12.0, 0.0),
        (6.0, 6.0, 1.0),
        (0.0, 0.0, 1.0),
    ] {
        let mut model = member("VAR", offset_i, offset_j, rz, false);
        analyze_model(&mut model).unwrap();
        assert_close(
            tip(&model, "TH")[0],
            ALPHA * 100.0 * L,
            1e-12,
            &format!("offsets {offset_i}/{offset_j} rz {rz}"),
        );
    }
}

/// Axial point load P = 10 at mid length (72) of a bar fixed at both joints, VAR
/// with offset 12 at I and rigid-end factor 1: Lc = 132, S1 from 12 to 78 and S2
/// from 78 to 144, the zone at I has S1. The flexibility from joint 1 to the load
/// is f_a = 72 / (E 200) and from the load to joint 2 f_b = (6 / 200 + 66 / 100) /
/// E, so the reaction at joint 1 is P f_b / (f_a + f_b) = 6.5714 (review row 3: the
/// baseline gave 6.667) and at joint 2 P f_a / (f_a + f_b) = 3.4286.
#[test]
fn d47_axial_span_load_reactions_follow_the_flexibilities_of_the_zoned_bar() {
    let mut model = member("VAR", 12.0, 0.0, 1.0, true);
    model.frames.get_mut("F").unwrap().loads_point.push(FrameLoadPoint {
        load_pat: "PL".into(),
        my_type: 1,
        dir: 1,
        csys: "Local".into(),
        rel_dist: 0.5,
        val: 10.0,
    });
    analyze_model(&mut model).unwrap();
    let (f_a, f_b) = (72.0 / 200.0, 6.0 / 200.0 + 66.0 / 100.0);
    let r1 = model.case_results["PL"].joint_reactions["J1"][0];
    let r2 = model.case_results["PL"].joint_reactions["J2"][0];
    // Reactions oppose the load: the support at I pulls back along -X.
    assert_close(r1, -10.0 * f_b / (f_a + f_b), 1e-12, "reaction at J1");
    assert_close(r2, -10.0 * f_a / (f_a + f_b), 1e-12, "reaction at J2");
}

/// Self weight of the member with offsets 24 at I and 12 at J: the weight per
/// unit length is gamma A with A = 200 from 0 to 78 and 100 from 78 to 144
/// (constant within the end offsets, p. 135). Fixed at both joints, the reactions
/// sum to W = gamma (200 x 78 + 100 x 66) and their moment about joint 1 balances
/// the weight at its centroid, for every rigid-end factor with and without a
/// moment release at both ends.
#[test]
fn d47_self_weight_of_a_non_prismatic_member_with_offsets_is_in_equilibrium() {
    let weight = GAMMA * (200.0 * 78.0 + 100.0 * 66.0);
    let centroid = GAMMA * (200.0 * 78.0 * 78.0 / 2.0 + 100.0 * 66.0 * (78.0 + 33.0)) / weight;
    for rz in [0.0, 0.5, 1.0] {
        for released in [false, true] {
            let mut model = member("VAR", 24.0, 12.0, rz, true);
            if released {
                let frame = model.frames.get_mut("F").unwrap();
                frame.release_i[5] = true;
                frame.release_j[5] = true;
            }
            analyze_model(&mut model).unwrap();
            let r1 = model.case_results["SW"].joint_reactions["J1"];
            let r2 = model.case_results["SW"].joint_reactions["J2"];
            let what = format!("rz {rz} released {released}");
            assert_close(r1[2] + r2[2], weight, 1e-10, &format!("{what}: sum Fz"));
            // Moment about the origin: M_y of both reactions plus x Fz ... = the
            // moment of the weight W at x = centroid about Y, which is
            // -x W for a load -W along Z: sum(My) - sum(x Fz) cancels it.
            let my = r1[4] + r2[4] - L * r2[2];
            assert_close(my, -centroid * weight, 1e-9, &format!("{what}: sum My"));
        }
    }
}
