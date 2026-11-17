//! Wave 5 P10 (D39): the seismic diaphragm eccentricity overwrite survives an S2K save and reopen
//! and gives the same analysis. Reference of the moment: CSI Lateral Loads Manual, section 2.2.3,
//! pp. 2-4 and 2-5: with an overwrite the moment on the diaphragm is the distance times the total
//! lateral force applied to it (the ratio times the width is replaced); F is the storey force from
//! ASCE 7-10 12.8 (Cs of Eqn. 12.8-2, 12.8-3, 12.8-5; one level, V = Cs W), as in
//! `wave5_eccentricity.rs` which uses the same model: one storey, columns at (0,0), (10,0), (0,6),
//! (10,6) in kN, m, rigid Z diaphragm at z = 3, mass 10 kN s^2/m at each top joint, bases fixed
//! (24 restrained degrees of freedom, no rigid-body motion left).
use sap_format::{parse_s2k, write_s2k};
use sap_model::seismic::{AutoSeismicAsce710, SeismicDirection, SeismicPeriod};
use sap_model::{
    ConstraintAxis, ConstraintDef, ConstraintType, FrameSection, LoadCase, LoadPattern,
    LoadPatternType, Material, SapModel,
};
use sap_solver::analyze_model;
use sap_units::Units;

const G: f64 = 9.806_65;
const MASS: f64 = 10.0;
const COLUMNS: [(&str, f64, f64); 4] = [
    ("1", 0.0, 0.0),
    ("2", 10.0, 0.0),
    ("3", 0.0, 6.0),
    ("4", 10.0, 6.0),
];

fn storey_force() -> f64 {
    let (ss, s1, fa, fv, r, ie, t): (f64, f64, f64, f64, f64, f64, f64) =
        (1.5, 0.5, 1.0, 1.0, 5.0, 1.0, 1.0);
    let sds = 2.0 / 3.0 * fa * ss;
    let sd1 = 2.0 / 3.0 * fv * s1;
    let ri = r / ie;
    let cs = (sds / ri)
        .min(sd1 / (t * ri))
        .max((0.044_f64 * sds * ie).max(0.01));
    cs * 4.0 * MASS * G
}

fn building() -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    m.load_cases.clear();
    m.load_patterns.clear();
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.e = 2.0e8;
    mat.g = 2.0e8 / 2.6;
    mat.unit_weight = 0.0;
    mat.unit_mass = 0.0;
    m.materials.insert("MAT".into(), mat);
    let mut sec = FrameSection::default();
    sec.name = "SEC".into();
    sec.material = "MAT".into();
    sec.area = 0.05;
    sec.i33 = 5.0e-4;
    sec.i22 = 5.0e-4;
    sec.torsional_j = 1.0e-3;
    sec.as2 = 0.0;
    sec.as3 = 0.0;
    m.frame_sections.insert("SEC".into(), sec);
    m.load_patterns.insert(
        "EQ".into(),
        LoadPattern {
            name: "EQ".into(),
            pat_type: LoadPatternType::Quake,
            self_wt_multiplier: 0.0,
        },
    );
    for (name, x, y) in COLUMNS {
        let base = format!("B{name}");
        m.add_point(base.clone(), x, y, 0.0);
        m.points[&base].restraint = [true; 6];
        m.add_point(name, x, y, 3.0);
        m.points[name].mass = [MASS, MASS, MASS, 0.0, 0.0, 0.0];
        m.add_frame(format!("C{name}"), base, name.to_string(), "SEC");
    }
    m.constraints.insert(
        "D1".into(),
        ConstraintDef {
            name: "D1".into(),
            constraint_type: ConstraintType::Diaphragm,
            axis: ConstraintAxis::Z,
            dofs: [true, true, false, false, false, true],
        },
    );
    for (name, _, _) in COLUMNS {
        m.points[name].constraints.push("D1".into());
    }
    m.auto_seismic.insert(
        "EQ".into(),
        AutoSeismicAsce710 {
            direction: SeismicDirection::Y,
            eccentricity_ratio: 0.05,
            period: SeismicPeriod::User { t: 1.0 },
            r: 5.0,
            omega0: 3.0,
            cd: 4.5,
            ie: 1.0,
            ss: 1.5,
            s1: 0.5,
            tl: 8.0,
            fa: Some(1.0),
            fv: Some(1.0),
            z_min: None,
            z_max: None,
        },
    );
    m.load_cases
        .insert("CASE".into(), LoadCase::linear_static("CASE", "EQ", 1.0));
    m
}

fn moment(m: &mut SapModel) -> (f64, [f64; 6], f64) {
    analyze_model(m).unwrap();
    let case = &m.case_results["CASE"];
    let moments = &case.auto_lateral[0].diaphragm_moments;
    assert_eq!(moments.len(), 1);
    (moments[0].moment_z, case.base_reactions, case.joint_displacements["1"][5])
}

#[test]
fn an_overwrite_gives_the_same_analysis_after_an_s2k_round_trip() {
    let f = storey_force();
    let mut original = building();
    original
        .auto_seismic_eccentricity
        .entry("EQ".into())
        .or_default()
        .insert("D1".into(), 0.4);
    let mut reopened = parse_s2k(&write_s2k(&original)).unwrap();
    assert_eq!(
        reopened.auto_seismic_eccentricity,
        original.auto_seismic_eccentricity
    );

    let (m0, r0, rz0) = moment(&mut original);
    let (m1, r1, rz1) = moment(&mut reopened);
    // Manual: M = distance x F, here 0.4 F (Y load: the couple is +e F by the project's sign
    // choice), where the ratio 0.05 x width 10 = 0.5 F would apply without the overwrite.
    assert!((m0 - 0.4 * f).abs() <= 1e-9 * f, "moment {m0} against {}", 0.4 * f);
    // The reopened model reproduces the whole result; the S2K text carries 17 digits, so the
    // two analyses run on equal numbers and agree to far better than 1e-9 relative to F.
    assert!((m1 - m0).abs() <= 1e-9 * f, "moment {m1} against {m0}");
    for i in 0..6 {
        assert!((r1[i] - r0[i]).abs() <= 1e-9 * f * 10.0, "reaction {i}: {} {}", r1[i], r0[i]);
    }
    assert!((rz1 - rz0).abs() <= 1e-9 * rz0.abs());

    // Without the table the moment is ratio x width x F = 0.5 F: the overwrite is what is read.
    let mut without = parse_s2k(&write_s2k(&building())).unwrap();
    let (m2, _, _) = moment(&mut without);
    assert!((m2 - 0.5 * f).abs() <= 1e-9 * f, "moment {m2} against {}", 0.5 * f);
}
