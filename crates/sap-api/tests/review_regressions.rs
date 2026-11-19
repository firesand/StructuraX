use sap_api::{cSapModel, OAPI_ERR, OAPI_OK};
use sap_model::*;
use sap_units::Units;

fn cube() -> (cSapModel, [String; 8]) {
    let mut api = cSapModel::new(Units::KipInF);
    assert_eq!(
        api.prop_solid()
            .set_prop("S", "A992Fy50", 0., 0., 0., false),
        OAPI_OK
    );
    let names = std::array::from_fn(|i| format!("P{i}"));
    for (i, name) in names.iter().enumerate() {
        assert_eq!(
            api.point_obj()
                .add_cartesian(
                    (i & 1) as f64,
                    ((i >> 1) & 1) as f64,
                    ((i >> 2) & 1) as f64,
                    name
                )
                .0,
            OAPI_OK
        );
    }
    (api, names)
}

#[test]
fn malformed_solid_creation_is_atomic_and_valid_cube_is_accepted() {
    let (mut api, names) = cube();
    api.inner
        .case_results
        .insert("OLD".into(), CaseResults::default());
    let before = api.inner.clone();
    for points in [
        std::array::from_fn(|_| names[0].clone()),
        std::array::from_fn(|i| names[i ^ 1].clone()),
        std::array::from_fn(|i| names[i & 3].clone()),
    ] {
        assert_eq!(api.solid_obj().add_by_point(points, "S", "BAD").0, OAPI_ERR);
        assert_eq!(api.inner, before);
    }
    api.inner.points["P7"].x = f64::INFINITY;
    assert_eq!(
        api.solid_obj().add_by_point(names.clone(), "S", "BAD").0,
        OAPI_ERR
    );
    assert!(api.inner.solids.is_empty());
    assert!(api.inner.case_results.contains_key("OLD"));
    api.inner = before;
    assert_eq!(api.solid_obj().add_by_point(names, "S", "CUBE").0, OAPI_OK);
    assert!(api.inner.case_results.is_empty());
}

#[test]
fn additive_solid_temperature_overflow_preserves_assignment_and_results() {
    let (mut api, names) = cube();
    assert_eq!(api.solid_obj().add_by_point(names, "S", "CUBE").0, OAPI_OK);
    assert_eq!(
        api.solid_obj()
            .set_load_temperature("CUBE", "DEAD", f64::MAX, true),
        OAPI_OK
    );
    api.inner
        .case_results
        .insert("OLD".into(), CaseResults::default());
    let before = api.inner.clone();
    assert_eq!(
        api.solid_obj()
            .set_load_temperature("CUBE", "DEAD", f64::MAX, false),
        OAPI_ERR
    );
    assert_eq!(api.inner, before);
    assert_eq!(
        api.solid_obj()
            .set_load_temperature("CUBE", "DEAD", -f64::MAX, false),
        OAPI_OK
    );
    assert_eq!(api.inner.solids["CUBE"].loads_temp["DEAD"], 0.);
    assert!(api.inner.case_results.is_empty());
}

#[test]
fn nonlinear_history_definition_rejects_unsafe_options_atomically() {
    let (mut api, names) = cube();
    assert_eq!(api.solid_obj().add_by_point(names, "S", "CUBE").0, OAPI_OK);
    api.inner.load_patterns["DEAD"].self_wt_multiplier = 0.;
    assert_eq!(
        api.functions().set_definition(DynamicFunction {
            name: "STEP".into(),
            is_time_history: true,
            damping_ratio: 0.,
            points: vec![(0., 1.), (1., 1.)],
        }),
        OAPI_OK
    );
    let mut history = LoadCase::linear_static("NLH", "DEAD", 1.);
    history.case_type = LoadCaseType::NonlinearHistory;
    // Modal history types are integrated directly; they take no modal damping.
    history.modal_damping = 0.;
    history.loads.clear();
    history.history_loads = vec![("STEP".into(), "DEAD".into(), 1.)];
    history.time_step_dt = 0.01;
    history.num_time_steps = 10;
    assert_eq!(api.load_cases().set_definition(history.clone()), OAPI_OK);
    api.inner
        .case_results
        .insert("OLD".into(), CaseResults::default());
    let before = api.inner.clone();
    for change in 0..5 {
        let mut invalid = history.clone();
        match change {
            0 => invalid.initial_case = Some("DEAD".into()),
            1 => invalid.max_iterations = 0,
            2 => invalid.convergence_tol = f64::NAN,
            3 => invalid.nl_geom = NLGeomType::PDelta,
            _ => invalid.time_step_dt = 0.,
        }
        assert_eq!(api.load_cases().set_definition(invalid), OAPI_ERR);
        assert_eq!(api.inner, before);
    }
    assert_eq!(
        api.solid_obj()
            .set_load_temperature("CUBE", "DEAD", 1., true),
        OAPI_OK
    );
    api.inner
        .case_results
        .insert("OLD".into(), CaseResults::default());
    let before = api.inner.clone();
    assert_eq!(api.load_cases().set_definition(history), OAPI_ERR);
    assert_eq!(api.inner, before);
}

#[test]
fn api_nonlinear_hook_history_matches_closed_form_and_support_equilibrium() {
    for kind in [
        LoadCaseType::NonlinearHistory,
        LoadCaseType::NonlinearDynamic,
    ] {
        let mut api = cSapModel::new(Units::KipInF);
        api.inner.load_cases.clear();
        api.inner.load_patterns["DEAD"].self_wt_multiplier = 0.;
        api.inner.add_point("I", 0., 0., 0.);
        api.inner.add_point("J", 1., 0., 0.);
        api.inner.points["I"].restraint = [true; 6];
        api.inner.points["J"].restraint = [false, true, true, true, true, true];
        api.inner.points["J"].mass[0] = 1.;
        api.inner.points["J"].loads_force.push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: [10., 0., 0., 0., 0., 0.],
        });
        let mut prop = LinkProp {
            name: "HOOK".into(),
            link_type: LinkPropType::Hook,
            ..Default::default()
        };
        prop.dof_nonlinear[0] = true;
        prop.k_nl[0] = 100.;
        prop.ke[0] = 13.;
        assert_eq!(api.prop_link().set_definition(prop), OAPI_OK);
        assert_eq!(
            api.link_obj().add_by_point("I", Some("J"), "HOOK", "L").0,
            OAPI_OK
        );
        assert_eq!(
            api.functions().set_definition(DynamicFunction {
                name: "RAMP".into(),
                is_time_history: true,
                damping_ratio: 0.,
                points: vec![(0., 0.), (1., 1.)],
            }),
            OAPI_OK
        );
        let mut h = LoadCase::linear_static("H", "DEAD", 1.);
        h.case_type = kind;
        // Modal history types are integrated directly; they take no modal damping.
        h.modal_damping = 0.;
        h.loads.clear();
        h.history_loads = vec![("RAMP".into(), "DEAD".into(), 1.)];
        h.num_time_steps = 500;
        h.time_step_dt = 0.001;
        h.convergence_tol = 1e-10;
        assert_eq!(api.load_cases().set_definition(h), OAPI_OK);
        sap_solver::analyze_model(&mut api.inner).unwrap();
        let result = &api.inner.case_results["H"];
        for (t, u) in result.time_steps.iter().zip(&result.time_history_disp["J"]) {
            let exact = 0.1 * (t - (10. * t).sin() / 10.);
            assert!((u[0] - exact).abs() < 5e-7, "t={t}: {} vs {exact}", u[0]);
        }
        let u = result.joint_displacements["J"][0];
        assert!((result.link_forces["L"][0] - 100. * u).abs() < 1e-8);
        assert!((result.joint_reactions["I"][0] + 100. * u).abs() < 1e-8);
    }
}

#[test]
fn native_file_api_reports_preservation_conflict_without_overwriting_destination() {
    let mut api = cSapModel::new(Units::KipInF);
    api.inner = sap_format::parse_s2k(
        "TABLE: \"JOINT COORDINATES\"\n Joint=A XorR=0 Y=0 Z=0 VendorTag=keep\nTABLE: \"OPAQUE VENDOR\"\n Value=21\nEND TABLE DATA\n"
    ).unwrap();
    let path = std::env::temp_dir().join(format!(
        "structurax-api-native-guard-{}-{}.sdb",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    std::fs::write(&path, b"preserve original destination").unwrap();
    let before = api.inner.clone();
    assert_eq!(api.file().save_native_sdb(path.to_str().unwrap()), OAPI_ERR);
    assert_eq!(api.inner, before);
    assert_eq!(
        std::fs::read(&path).unwrap(),
        b"preserve original destination"
    );
    assert!(api.last_file_error.as_ref().is_some_and(|e| !e.is_empty()));
    assert!(api.last_file_path.is_none());
    std::fs::remove_file(path).unwrap();
}

#[test]
fn native_file_api_uses_the_same_no_overwrite_contract_as_cli_and_gui() {
    let mut api = cSapModel::new(Units::KipInF);
    api.inner.materials.shift_remove("4000Psi");
    api.inner.area_sections.clear();
    let path = std::env::temp_dir().join(format!(
        "structurax-api-native-existing-{}-{}.sdb",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    let before = api.inner.clone();
    assert_eq!(api.file().save_native_sdb(path.to_str().unwrap()), OAPI_OK);
    let bytes = std::fs::read(&path).unwrap();
    assert_eq!(api.inner, before);
    assert_eq!(api.file().save_native_sdb(path.to_str().unwrap()), OAPI_ERR);
    assert!(api
        .last_file_error
        .as_ref()
        .unwrap()
        .contains("without overwriting"));
    assert_eq!(std::fs::read(&path).unwrap(), bytes);
    assert_eq!(api.inner, before);
    assert_eq!(api.last_file_path.as_ref(), Some(&path));
    std::fs::remove_file(path).unwrap();
}
