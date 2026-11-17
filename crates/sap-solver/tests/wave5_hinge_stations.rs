//! Wave 5, ledger D09: the station deflections of a frame with an interior
//! plastic hinge carry the slope jump of the hinge.
//!
//! Reference (closed form, written out here): a cantilever of length L = 4,
//! EI = 1000 (shear deformation off), tip load P at J, a rigid-perfectly-plastic
//! M3 hinge of yield moment My = 10 at distance x_h = 2 from the fixed end I,
//! tip displacement controlled to 0.2. The hinge yields at P_y = My / (L - x_h) = 5;
//! until then v(x) = P x^2 (3L - x) / (6 EI); the tip delta_y = P_y L^3 / (3 EI) =
//! 0.10666..; the remaining tip displacement is a rigid rotation of the part
//! beyond the hinge, theta_p = (0.2 - delta_y) / (L - x_h) = 0.046666..; so
//! v(x) = P_y x^2 (3L - x) / (6 EI) + theta_p max(0, x - x_h) at every x.
//! Tolerance 1e-9 (absolute, in length units; the tip value is 0.2): the case is
//! solved in hinge space by a direct method, so only roundoff is expected.
use sap_model::hinge::*;
use sap_model::*;
use sap_solver::analyze_model;

const L: f64 = 4.0;
const EI: f64 = 1000.0;
const MY: f64 = 10.0;
const XH: f64 = 2.0;
const TARGET: f64 = 0.2;

fn theta_p() -> f64 {
    let py = MY / (L - XH);
    (TARGET - py * L.powi(3) / (3.0 * EI)) / (L - XH)
}

fn reference(x: f64) -> f64 {
    let py = MY / (L - XH);
    py * x * x * (3.0 * L - x) / (6.0 * EI) + theta_p() * (x - XH).max(0.0)
}

fn model(split: bool) -> SapModel {
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    m.active_dofs = [true, false, true, false, true, false];
    m.load_patterns.insert(
        "P".into(),
        LoadPattern {
            name: "P".into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
    let mut material = Material::default_steel_kip_in();
    material.name = "M".into();
    material.e = EI;
    material.g = EI / 2.6;
    material.u = 0.3;
    material.unit_weight = 0.0;
    material.unit_mass = 0.0;
    m.materials.insert("M".into(), material);
    m.frame_sections.insert(
        "S".into(),
        FrameSection {
            name: "S".into(),
            material: "M".into(),
            prop_type: FramePropType::General,
            area: 1.0,
            i33: 1.0,
            i22: 1.0,
            torsional_j: 2.0,
            as2: 0.0,
            as3: 0.0,
            modifiers: [1.0; 8],
            ..FrameSection::default()
        },
    );
    m.add_point("I", 0.0, 0.0, 0.0);
    m.add_point("J", L, 0.0, 0.0);
    m.points["I"].restraint = [true; 6];
    let mut prop = HingeProp::symmetric(
        "EPP",
        HingeDof::M3,
        [
            HingePoint::new(1.0, 0.0),
            HingePoint::new(1.0, 0.25),
            HingePoint::new(1.0, 0.35),
            HingePoint::new(1.0, 0.5),
        ],
    );
    prop.force_sf = MY;
    prop.force_sf_negative = MY;
    m.hinge_props.insert("EPP".into(), prop);
    let hinge = |rel: f64| FrameHinge {
        prop: "EPP".into(),
        rel_dist: rel,
    };
    if split {
        m.add_point("M", XH, 0.0, 0.0);
        m.add_frame("A", "I", "M", "S");
        m.add_frame("B", "M", "J", "S");
        m.frames["A"].hinges = vec![hinge(1.0)];
    } else {
        m.add_frame("A", "I", "J", "S");
        m.frames["A"].hinges = vec![hinge(XH / L)];
    }
    m.points["J"].loads_force.push(PointLoadForce {
        load_pat: "P".into(),
        csys: "GLOBAL".into(),
        force: [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    });
    let mut c = LoadCase::linear_static("PUSH", "P", 1.0);
    c.case_type = LoadCaseType::NonlinearStatic;
    c.displacement_control = Some(DisplacementControl {
        target: TARGET,
        monitored: Some(("J".into(), 2)),
        conjugate: false,
    });
    m.load_cases.insert("PUSH".into(), c);
    m
}

fn close_abs(a: f64, b: f64, tol: f64, what: &str) {
    assert!(
        (a - b).abs() <= tol,
        "{what}: got {a}, expected {b} (tolerance {tol})"
    );
}

#[test]
fn r1_single_object_stations_have_the_hinge_kink() {
    let mut m = model(false);
    analyze_model(&mut m).unwrap();
    let case = &m.case_results["PUSH"];
    close_abs(case.joint_displacements["J"][2], TARGET, 1e-9, "tip");
    let h = &case.hinge_results[0];
    close_abs(h.force, MY, 1e-9, "hinge moment");
    close_abs(h.plastic_deformation, 0.0466666666667, 1e-9, "theta_p");
    close_abs(
        case.base_reactions[2],
        -MY / (L - XH),
        1e-9,
        "root reaction (opposes the +Z load)",
    );
    let stations = &case.frame_forces["A"];
    assert!(
        stations.iter().any(|s| s.station > XH + 0.5),
        "a station beyond the hinge is needed"
    );
    for s in stations {
        close_abs(s.local_disp[1], reference(s.station), 1e-9, "v at station");
    }
    // The values written out in the closed form above, where stations fall.
    for (x, v) in [
        (1.0, 0.0091666667),
        (2.0, 0.0333333333),
        (3.0, 0.1141666667),
        (4.0, 0.2),
    ] {
        if let Some(s) = stations.iter().find(|s| (s.station - x).abs() < 1e-9) {
            close_abs(s.local_disp[1], v, 1e-9, "printed reference");
        }
    }
}

#[test]
fn r2_two_objects_give_the_same_deflections() {
    let mut m = model(true);
    analyze_model(&mut m).unwrap();
    let case = &m.case_results["PUSH"];
    close_abs(case.joint_displacements["J"][2], TARGET, 1e-9, "tip");
    for (name, origin) in [("A", 0.0), ("B", XH)] {
        for s in &case.frame_forces[name] {
            close_abs(
                s.local_disp[1],
                reference(origin + s.station),
                1e-9,
                "v at station",
            );
        }
    }
}

#[test]
fn single_and_two_object_models_agree_at_common_stations() {
    let mut one = model(false);
    let mut two = model(true);
    analyze_model(&mut one).unwrap();
    analyze_model(&mut two).unwrap();
    let a = &one.case_results["PUSH"].frame_forces["A"];
    let b = &two.case_results["PUSH"].frame_forces["B"];
    let mut compared = 0;
    for s in b {
        if let Some(t) = a
            .iter()
            .find(|t| (t.station - (XH + s.station)).abs() < 1e-9)
        {
            close_abs(t.local_disp[1], s.local_disp[1], 1e-9, "common station");
            compared += 1;
        }
    }
    assert!(compared >= 2, "no common stations to compare");
}
