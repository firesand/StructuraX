//! Area added mass, and the refusals that keep area and solid models from
//! being solved with a formulation they did not ask for.
use sap_model::*;
use sap_solver::analyze_model;

/// One 4-joint shell on four equal vertical springs, Uz only (kip, in, s).
fn plate_on_springs(spring: f64, mass_per_area: f64) -> SapModel {
    let mut m = SapModel::default();
    m.load_patterns["DEAD"].self_wt_multiplier = 0.;
    m.load_cases.clear();
    m.active_dofs = [false, false, true, false, false, false];
    for (name, x, y) in [("A", 0., 0.), ("B", 30., 0.), ("C", 30., 20.), ("D", 0., 20.)] {
        m.add_point(name, x, y, 0.);
        m.points[name].spring[2] = spring;
    }
    for material in m.materials.values_mut() {
        material.unit_mass = 0.;
    }
    let section = m.area_sections.keys().next().unwrap().clone();
    let mut area = AreaObj::new(
        "P",
        vec!["A".into(), "B".into(), "C".into(), "D".into()],
        section,
    );
    area.mass_per_area = mass_per_area;
    m.areas.insert("P".into(), area);
    m.load_cases
        .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 4));
    m
}

#[test]
fn area_added_mass_gives_the_closed_form_rigid_translation_period() {
    // Uniform translation does not strain the plate, so it is an exact mode:
    // omega^2 = 4 k / (mu A), with mu A / 4 lumped at each joint. It is the
    // lowest mode because every other one adds plate stiffness.
    let (spring, mu, area) = (50.0, 2.0e-4, 30.0 * 20.0);
    let mut m = plate_on_springs(spring, mu);
    analyze_model(&mut m).unwrap();
    let period = m.case_results["MODAL"].modes[0].period_sec;
    let expected = 2.0 * std::f64::consts::PI * (mu * area / (4.0 * spring)).sqrt();
    assert!((period - expected).abs() < 1e-10 * expected, "{period} vs {expected}");

    // The mass modifier scales the section's own mass, not the added mass.
    m.areas["P"].modifiers[8] = 0.0;
    analyze_model(&mut m).unwrap();
    let unscaled = m.case_results["MODAL"].modes[0].period_sec;
    assert!((unscaled - expected).abs() < 1e-10 * expected);

    // kip-in -> kN-m: mass per area converts with the model, the period stays.
    m.convert_units_in_place(sap_units::Units::KnMC);
    let converted = m.areas["P"].mass_per_area;
    // 1 kip-s2/in per in2 = 4.4482216152605 kN / 0.0254 m / 0.0254^2 m2.
    let factor = 4.4482216152605 / 0.0254 / (0.0254 * 0.0254);
    assert!((converted - mu * factor).abs() < 1e-9 * mu * factor);
    analyze_model(&mut m).unwrap();
    let period = m.case_results["MODAL"].modes[0].period_sec;
    assert!((period - expected).abs() < 1e-9 * expected);

    for bad in [-1.0, f64::NAN] {
        let mut m = plate_on_springs(spring, bad);
        let error = analyze_model(&mut m).unwrap_err().to_string();
        assert!(error.contains("added mass per area"), "{error}");
    }
}

#[test]
fn geometric_stiffness_cases_refuse_models_with_area_elements() {
    // Buckling and P-Delta assemble geometric stiffness from frame axial
    // forces only; a plate's membrane forces would be left out.
    let mut m = plate_on_springs(50.0, 0.0);
    m.load_cases.clear();
    let mut buckling = LoadCase::linear_static("BUCK", "DEAD", 1.);
    buckling.case_type = LoadCaseType::Buckling;
    m.load_cases.insert("BUCK".into(), buckling);
    let error = analyze_model(&mut m).unwrap_err().to_string();
    assert!(error.contains("geometric stiffness"), "{error}");
    assert!(m.case_results.is_empty());

    m.load_cases.clear();
    let mut pdelta = LoadCase::linear_static("PD", "DEAD", 1.);
    pdelta.case_type = LoadCaseType::NonlinearStatic;
    pdelta.nl_geom = NLGeomType::PDelta;
    m.load_cases.insert("PD".into(), pdelta);
    // A linear case that takes its stiffness from the P-Delta case.
    let mut after = LoadCase::linear_static("AFTER", "DEAD", 1.);
    after.initial_case = Some("PD".into());
    m.load_cases.insert("AFTER".into(), after);
    let error = analyze_model(&mut m).unwrap_err().to_string();
    assert!(error.contains("geometric stiffness"), "{error}");

    // A linear static or modal case with any initial case takes geometric
    // stiffness from that case's frame forces, P-Delta or not.
    for case_type in [LoadCaseType::LinearStatic, LoadCaseType::Modal] {
        m.load_cases.clear();
        m.load_cases
            .insert("FIRST".into(), LoadCase::linear_static("FIRST", "DEAD", 1.));
        let mut next = LoadCase::linear_static("NEXT", "DEAD", 1.);
        if case_type == LoadCaseType::Modal {
            next = LoadCase::modal_eigen("NEXT", 2);
        }
        next.initial_case = Some("FIRST".into());
        m.load_cases.insert("NEXT".into(), next);
        let error = analyze_model(&mut m).unwrap_err().to_string();
        assert!(
            error.contains("NEXT") && error.contains("geometric stiffness"),
            "{case_type:?}: {error}"
        );
    }

    // Without geometric stiffness the same model is fine.
    m.load_cases.clear();
    m.load_cases
        .insert("LIN".into(), LoadCase::linear_static("LIN", "DEAD", 1.));
    analyze_model(&mut m).unwrap();
}

#[test]
fn axisymmetric_and_incompatible_plane_sections_are_refused() {
    let section_name = |m: &SapModel| m.area_sections.keys().next().unwrap().clone();
    let linear = |m: &mut SapModel| {
        m.load_cases.clear();
        m.load_cases
            .insert("LIN".into(), LoadCase::linear_static("LIN", "DEAD", 1.));
    };

    let mut m = plate_on_springs(50.0, 0.0);
    linear(&mut m);
    let name = section_name(&m);
    m.area_sections[&name].prop_type = AreaPropType::ASolid;
    let error = analyze_model(&mut m).unwrap_err().to_string();
    assert!(error.contains("axisymmetric-solid section"), "{error}");
    assert!(m.case_results.is_empty());

    // A plane section: in-plane model, with and without incompatible modes.
    let mut m = plate_on_springs(0.0, 0.0);
    linear(&mut m);
    m.active_dofs = [true, true, false, false, false, false];
    for joint in ["A", "D"] {
        m.points[joint].restraint = [true, true, false, false, false, false];
    }
    for joint in ["B", "C"] {
        m.points[joint].loads_force.push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: [5., 0., 0., 0., 0., 0.],
        });
    }
    let name = section_name(&m);
    m.area_sections[&name].prop_type = AreaPropType::Plane;
    m.area_sections[&name].plane_type = PlaneType::PlaneStress;
    m.area_sections[&name].incompatible_modes = true;
    let error = analyze_model(&mut m).unwrap_err().to_string();
    assert!(error.contains("incompatible bending modes"), "{error}");
    m.area_sections[&name].incompatible_modes = false;
    analyze_model(&mut m).unwrap();
    // 10 kip on a 20 in edge: membrane force 0.5 kip/in along X.
    let forces = &m.case_results["LIN"].area_results["P"];
    assert!((forces.f11 - 0.5).abs() < 1e-6, "{}", forces.f11);
}
