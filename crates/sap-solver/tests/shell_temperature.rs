//! Shell temperature loads: a uniform change and a gradient through the thickness.
//!
//! References.
//! * Closed forms for one square plate (kN, m), below.
//! * CSI Software Verification Example 2-013 (uniform +100 F on the MacNeal-Harder patch
//!   plate; free: Ux = 0.000132 in, Uy = 0.000066 in at joint 8 and no stress;
//!   restrained: -733.33 lb/in2).
//! * CSI Software Verification Example 2-014 (annular plate, clamped outside, bottom
//!   100 F warmer than top: Uz at the inner edge +0.01905 in (independent 0.01931), M11 at
//!   the outer edge -2.182 k-in/in (independent -2.181)).
//!
//! Conventions (CSI Analysis Reference, shell element): the gradient is dT/dx3, positive
//! when it is warmer toward +local 3; positive moments give positive stress at the bottom.
use sap_model::*;
use sap_solver::{analyze_model, SolverError};
use sap_units::Units;

const SIDE: f64 = 2.0;
const T: f64 = 0.1;
const E: f64 = 3.0e7;
const NU: f64 = 0.2;
const ALPHA: f64 = 1.0e-5;

fn close(what: &str, got: f64, want: f64, tol: f64) {
    assert!(
        (got - want).abs() <= tol * want.abs().max(1e-12),
        "{what}: got {got:e}, want {want:e}"
    );
}

/// One square plate of side 2 m in the X-Y plane, joints counter-clockwise from above
/// (local 3 = +Z, local 1 = +X), with one temperature load in pattern `T`.
fn plate(my_type: i32, val: f64, all_fixed: bool) -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.e = E;
    mat.u = NU;
    mat.g = E / (2.0 * (1.0 + NU));
    mat.a = ALPHA;
    mat.unit_weight = 0.0;
    mat.unit_mass = 0.0;
    m.materials.insert("MAT".into(), mat);
    let mut s = AreaSection::default();
    s.name = "PL".into();
    s.material = "MAT".into();
    s.thickness_membrane = T;
    s.thickness_bending = T;
    m.area_sections.insert("PL".into(), s);
    m.load_cases.clear();
    m.load_patterns.clear();
    m.load_patterns.insert(
        "T".into(),
        LoadPattern {
            name: "T".into(),
            pat_type: LoadPatternType::Temperature,
            self_wt_multiplier: 0.0,
        },
    );
    m.load_cases
        .insert("T".into(), LoadCase::linear_static("T", "T", 1.0));
    for (name, x, y) in [
        ("1", 0.0, 0.0),
        ("2", SIDE, 0.0),
        ("3", SIDE, SIDE),
        ("4", 0.0, SIDE),
    ] {
        m.add_point(name, x, y, 0.0);
        if all_fixed || name == "1" {
            m.points[name].restraint = [true; 6];
        }
    }
    m.add_area(
        "A",
        vec!["1".into(), "2".into(), "3".into(), "4".into()],
        "PL",
    );
    m.areas["A"].loads_temp.push(AreaLoadTemperature {
        load_pat: "T".into(),
        my_type,
        val,
        pattern_name: None,
    });
    m
}

#[test]
fn uniform_change_expands_a_free_plate_without_stress() {
    // Clamped at joint 1 only: the plate is free to grow by alpha dT = 4e-4 per length.
    let mut m = plate(1, 40.0, false);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["T"];
    let grow = ALPHA * 40.0 * SIDE;
    close("ux 2", r.joint_displacements["2"][0], grow, 1e-9);
    close("ux 3", r.joint_displacements["3"][0], grow, 1e-9);
    close("uy 3", r.joint_displacements["3"][1], grow, 1e-9);
    close("uy 4", r.joint_displacements["4"][1], grow, 1e-9);
    // Stress scale of the restrained plate, to judge "zero".
    let scale = E * ALPHA * 40.0 / (1.0 - NU);
    let a = &r.area_results["A"];
    for (what, v) in [
        ("F11", a.f11 / T),
        ("F22", a.f22 / T),
        ("F12", a.f12 / T),
        ("s11", a.s11_top),
        ("s22", a.s22_top),
        ("svm", a.svm_top),
    ] {
        assert!(v.abs() <= 1e-9 * scale, "{what} = {v:e} in a free plate");
    }
}

#[test]
fn uniform_change_in_a_restrained_plate_gives_the_biaxial_stress() {
    // All joints fixed: no strain, so s = -E alpha dT / (1 - nu) in both directions
    // = -3e7 * 1e-5 * 40 / 0.8 = -15000 kN/m2, and F = s * t = -1500 kN/m.
    let mut m = plate(1, 40.0, true);
    analyze_model(&mut m).unwrap();
    let a = &m.case_results["T"].area_results["A"];
    close("F11", a.f11, -1500.0, 1e-12);
    close("F22", a.f22, -1500.0, 1e-12);
    close("s11", a.s11_top, -15000.0, 1e-12);
    close("s22", a.s22_top, -15000.0, 1e-12);
    // Equal biaxial stress: von Mises equals its magnitude.
    close("svm", a.svm_top, 15000.0, 1e-12);
    assert_eq!(a.f12, 0.0);
    assert_eq!(a.m11, 0.0);
}

#[test]
fn gradient_bends_a_free_plate_to_a_cap_without_moments() {
    // Warmer on top by g = 50 per metre of thickness. The free strain alpha g x3 is the
    // bending strain -x3 w,xx of the surface w = -alpha g (x^2 + y^2) / 2: the plate
    // curls away from its warm face. With joint 1 clamped at the origin:
    //   joint 2 (2, 0): uz = -alpha g 2 = -1e-3, ry = -w,x = alpha g 2 = 1e-3, rx = 0;
    //   joint 3 (2, 2): uz = -2e-3, rx = w,y = -1e-3, ry = 1e-3.
    // The element represents that field exactly (quadratic w, linear rotations).
    let g = 50.0;
    let mut m = plate(2, g, false);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["T"];
    let k = ALPHA * g;
    let d2 = r.joint_displacements["2"];
    let d3 = r.joint_displacements["3"];
    let d4 = r.joint_displacements["4"];
    close("uz 2", d2[2], -k * SIDE * SIDE / 2.0, 1e-9);
    close("ry 2", d2[4], k * SIDE, 1e-9);
    assert!(d2[3].abs() <= 1e-12, "rx 2 = {:e}", d2[3]);
    close("uz 3", d3[2], -k * SIDE * SIDE, 1e-9);
    close("rx 3", d3[3], -k * SIDE, 1e-9);
    close("ry 3", d3[4], k * SIDE, 1e-9);
    close("uz 4", d4[2], -k * SIDE * SIDE / 2.0, 1e-9);
    close("rx 4", d4[3], -k * SIDE, 1e-9);
    // No restraint against the curvature, so no moment and no stress.
    let m_scale = E * T.powi(3) * ALPHA * g / (12.0 * (1.0 - NU));
    let a = &r.area_results["A"];
    for (what, v) in [("M11", a.m11), ("M22", a.m22), ("M12", a.m12)] {
        assert!(v.abs() <= 1e-9 * m_scale, "{what} = {v:e} in a free plate");
    }
    assert!(a.svm_top.abs() <= 1e-9 * 6.0 * m_scale / (T * T));
}

#[test]
fn gradient_in_a_restrained_plate_compresses_the_warm_face() {
    // All joints fixed: no curvature. The warm top face wants to grow by
    // alpha g t/2 and cannot: s_top = -E alpha g t / (2 (1 - nu))
    // = -3e7 * 1e-5 * 50 * 0.1 / 1.6 = -937.5 kN/m2. With s_top = -6 M / t^2:
    // M11 = M22 = E t^3 alpha g / (12 (1 - nu)) = 1.5625 kN m/m (positive: tension below).
    let mut m = plate(2, 50.0, true);
    analyze_model(&mut m).unwrap();
    let a = &m.case_results["T"].area_results["A"];
    close("M11", a.m11, 1.5625, 1e-12);
    close("M22", a.m22, 1.5625, 1e-12);
    close("s11", a.s11_top, -937.5, 1e-12);
    close("s22", a.s22_top, -937.5, 1e-12);
    assert_eq!(a.f11, 0.0);
    // Reversing the gradient reverses everything.
    let mut m = plate(2, -50.0, true);
    analyze_model(&mut m).unwrap();
    let a = &m.case_results["T"].area_results["A"];
    close("M11", a.m11, -1.5625, 1e-12);
    close("s11", a.s11_top, 937.5, 1e-12);
}

#[test]
fn a_combination_scales_the_thermal_part_of_the_stress() {
    let mut m = plate(1, 40.0, true);
    m.load_combos.insert(
        "C".into(),
        LoadCombo {
            name: "C".into(),
            combo_type: ComboType::LinearAdd,
            cases: vec![("T".into(), 1.5)],
        },
    );
    analyze_model(&mut m).unwrap();
    close("F11", m.case_results["C"].area_results["A"].f11, -2250.0, 1e-12);
    // A case scale factor scales the temperature too.
    let mut m = plate(1, 40.0, true);
    m.load_cases["T"].loads[0].scale_factor = -0.5;
    analyze_model(&mut m).unwrap();
    close("F11", m.case_results["T"].area_results["A"].f11, 750.0, 1e-12);
}

fn refused(m: &mut SapModel, needle: &str) {
    match analyze_model(m) {
        Err(SolverError::UnsupportedInput(msg)) | Err(SolverError::InvalidInput(msg)) => {
            assert!(msg.contains(needle) && msg.contains("'A'"), "{msg}")
        }
        other => panic!("expected a refusal containing '{needle}', got {other:?}"),
    }
    assert!(m.case_results.is_empty());
}

#[test]
fn temperature_loads_the_solver_cannot_apply_are_refused() {
    // An unknown load type.
    let mut m = plate(3, 10.0, false);
    refused(&mut m, "type 3");
    // A joint pattern.
    let mut m = plate(1, 10.0, false);
    m.areas["A"].loads_temp[0].pattern_name = Some("P".into());
    refused(&mut m, "joint pattern");
    // A three-joint area.
    let mut m = plate(1, 10.0, false);
    m.areas["A"].points.pop();
    refused(&mut m, "four-joint");
    // A nonfinite value.
    let mut m = plate(2, f64::NAN, false);
    refused(&mut m, "nonfinite");
    // A load in a pattern no case uses, and a zero load, are not in the way.
    let mut m = plate(3, 10.0, false);
    m.areas["A"].loads_temp[0].load_pat = "UNUSED".into();
    analyze_model(&mut m).unwrap();
    let mut m = plate(3, 0.0, false);
    analyze_model(&mut m).unwrap();
}

#[test]
fn a_section_without_the_behaviour_a_temperature_load_acts_on_is_refused() {
    // A plate has no membrane stiffness for a uniform change to act on.
    let mut m = plate(1, 40.0, true);
    m.area_sections["PL"].shell_type = ShellType::PlateThin;
    refused(&mut m, "cannot carry");
    // A membrane has no bending stiffness for a gradient.
    let mut m = plate(2, 50.0, true);
    m.area_sections["PL"].shell_type = ShellType::Membrane;
    refused(&mut m, "cannot carry");
    // Plane sections: the thermal stress of plane strain has another form.
    let mut m = plate(1, 40.0, true);
    m.area_sections["PL"].prop_type = AreaPropType::Plane;
    refused(&mut m, "cannot carry");
    // The combinations that are carried: a gradient on a plate, a uniform
    // change on a membrane, with the values of the shell.
    let mut m = plate(2, 50.0, true);
    m.area_sections["PL"].shell_type = ShellType::PlateThin;
    analyze_model(&mut m).unwrap();
    close("plate M11", m.case_results["T"].area_results["A"].m11, 1.5625, 1e-12);
    let mut m = plate(1, 40.0, true);
    m.area_sections["PL"].shell_type = ShellType::Membrane;
    analyze_model(&mut m).unwrap();
    close("membrane F11", m.case_results["T"].area_results["A"].f11, -1500.0, 1e-12);
}

#[test]
fn a_nonlinear_static_case_and_its_continuation_keep_the_thermal_stress() {
    // The same restrained plate in a nonlinear static case, and a second case that
    // continues from it and adds nothing: both report the stress of the linear case.
    let mut m = plate(1, 40.0, true);
    m.load_cases["T"].case_type = LoadCaseType::NonlinearStatic;
    m.load_patterns.insert(
        "NONE".into(),
        LoadPattern {
            name: "NONE".into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
    let mut next = LoadCase::linear_static("T2", "NONE", 1.0);
    next.case_type = LoadCaseType::NonlinearStatic;
    next.initial_case = Some("T".into());
    m.load_cases.insert("T2".into(), next);
    analyze_model(&mut m).unwrap();
    for case in ["T", "T2"] {
        let a = &m.case_results[case].area_results["A"];
        close(case, a.f11, -1500.0, 1e-12);
        close(case, a.s11_top, -15000.0, 1e-12);
    }
    // And for a free plate, the continuation keeps the expansion without stress.
    let mut m = plate(1, 40.0, false);
    m.load_cases["T"].case_type = LoadCaseType::NonlinearStatic;
    m.load_patterns.insert(
        "NONE".into(),
        LoadPattern {
            name: "NONE".into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
    let mut next = LoadCase::linear_static("T2", "NONE", 1.0);
    next.case_type = LoadCaseType::NonlinearStatic;
    next.initial_case = Some("T".into());
    m.load_cases.insert("T2".into(), next);
    analyze_model(&mut m).unwrap();
    for case in ["T", "T2"] {
        let r = &m.case_results[case];
        close(case, r.joint_displacements["2"][0], ALPHA * 40.0 * SIDE, 1e-9);
        assert!(r.area_results["A"].svm_top.abs() <= 1e-9 * 15000.0, "{case}");
    }
}

#[cfg(feature = "vendor-fixtures")]
mod vendor {
    use super::close;
    use sap_format::load_sdb_file;
    use sap_model::SapModel;
    use sap_solver::analyze_model;
    use std::path::{Path, PathBuf};

    fn load(file: &str) -> Option<SapModel> {
        let dir = std::env::var_os("STRUCTURAX_VENDOR_FIXTURES")
            .map(PathBuf::from)
            .unwrap_or_else(|| Path::new(env!("CARGO_MANIFEST_DIR")).join("../../extracted/files"));
        let path = dir.join(file);
        path.exists().then(|| {
            let mut model = load_sdb_file(&path).expect("import");
            analyze_model(&mut model).expect("analysis");
            model
        })
    }

    #[test]
    fn example_2_013_uniform_temperature_free_and_restrained() {
        for option in ["thin", "thick"] {
            // Model A, free expansion (lb, in): published Ux = 0.000132, Uy = 0.000066 at
            // joint 8 (0.24, 0.12), i.e. alpha dT = 5.5e-4, and no stress.
            let Some(a) = load(&format!("example_2_013a_{option}.sdb")) else {
                return;
            };
            let r = a.case_results.values().next().unwrap();
            let d = r.joint_displacements["8"];
            close("2-013 A Ux", d[0], 0.000132, 1e-6);
            close("2-013 A Uy", d[1], 0.000066, 1e-6);
            for (name, ar) in &r.area_results {
                assert!(
                    ar.svm_top.abs() <= 1e-6 * 733.33,
                    "2-013 A {option}: area {name} has stress {:e}",
                    ar.svm_top
                );
            }
            // Model B, corners pinned: published sxx = syy = -733.33 lb/in2 at joint 8
            // (hand value E alpha dT / (1 - nu) = 1e6 * 5.5e-4 / 0.75 = 733.33). The state
            // is uniform, so every element has it.
            let b = load(&format!("example_2_013b_{option}.sdb")).unwrap();
            let r = b.case_results.values().next().unwrap();
            assert_eq!(r.area_results.len(), 5);
            for ar in r.area_results.values() {
                close("2-013 B sxx", ar.s11_top, -733.33, 1e-5);
                close("2-013 B syy", ar.s22_top, -733.33, 1e-5);
            }
            assert!(r.joint_displacements["8"].iter().all(|v| v.abs() < 1e-12));
        }
    }

    #[test]
    fn example_2_014_temperature_gradient_on_an_annular_plate() {
        for (file, uz_tol) in [
            ("example_2_014a_thin.sdb", 0.04),
            ("example_2_014b_thin.sdb", 0.01),
        ] {
            let Some(model) = load(file) else {
                return;
            };
            let r = model.case_results.values().next().unwrap();
            let radius = |p: &str| model.points[p].x.hypot(model.points[p].y);
            // Free inner edge (r = 3 in): the plate lifts. Independent (Roark) 0.01931 in.
            let mut inner = 0;
            for (p, d) in &r.joint_displacements {
                if (radius(p) - 3.0).abs() < 1e-6 {
                    inner += 1;
                    close("2-014 Uz inner", d[2], 0.01931, uz_tol);
                }
            }
            assert!(inner >= 16);
            // Clamped outer edge: M11 (radial) = -2.181 k-in/in independent, -2.182 SAP2000.
            // Ours is the value at the center of the outermost ring of elements.
            let center_radius = |a: &str| {
                let pts = &model.areas[a].points;
                let (x, y) = pts.iter().fold((0.0, 0.0), |s, p| {
                    (s.0 + model.points[p].x, s.1 + model.points[p].y)
                });
                (x / pts.len() as f64).hypot(y / pts.len() as f64)
            };
            let outer = r
                .area_results
                .keys()
                .map(|a| center_radius(a))
                .fold(0.0, f64::max);
            for (a, ar) in &r.area_results {
                if (center_radius(a) - outer).abs() < 1e-6 {
                    close("2-014 M11 outer", ar.m11, -2.181, 0.005);
                    // Negative moment: tension on the (cooler) top face.
                    assert!(ar.s11_top > 0.0);
                }
            }
        }
    }
}
