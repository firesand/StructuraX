use sap_api::{cSapModel, OAPI_ERR, OAPI_OK};
use sap_model::*;
use sap_units::Units;
#[test]
fn gap_hook_configured_through_api_reach_exact_contact_equilibrium() {
    for (kind, sign) in [(LinkPropType::Gap, -1.), (LinkPropType::Hook, 1.)] {
        let mut api = cSapModel::new(Units::KipInF);
        api.inner.load_cases.clear();
        api.inner.load_patterns["DEAD"].self_wt_multiplier = 0.;
        assert_eq!(api.point_obj().add_cartesian(0., 0., 0., "J").0, OAPI_OK);
        assert_eq!(
            api.point_obj()
                .set_restraint("J", [true, true, false, true, true, true]),
            OAPI_OK
        );
        assert_eq!(
            api.point_obj()
                .set_load_force("J", "DEAD", [0., 0., sign * 30., 0., 0., 0.], true),
            OAPI_OK
        );
        let mut prop = LinkProp {
            name: "CONTACT".into(),
            link_type: kind,
            ..Default::default()
        };
        prop.dof_nonlinear[0] = true;
        prop.k_nl[0] = 100.;
        prop.param1[0] = 0.1;
        prop.ke[0] = 1e6;
        assert_eq!(api.prop_link().set_definition(prop), OAPI_OK);
        assert_eq!(
            api.link_obj().add_by_point("J", None, "CONTACT", "L").0,
            OAPI_OK
        );
        let mut case = LoadCase::linear_static("NL", "DEAD", 1.);
        case.case_type = LoadCaseType::NonlinearStatic;
        assert_eq!(api.load_cases().set_definition(case), OAPI_OK);
        let text = sap_format::try_write_s2k_with_report(&api.inner).unwrap();
        assert!(text
            .warnings
            .iter()
            .any(|w| w.contains("Rust") || w.contains("extension")));
        let mut restored = sap_format::parse_s2k(&text.text).unwrap();
        assert!(restored.link_props["CONTACT"].dof_nonlinear[0]);
        sap_solver::analyze_model(&mut restored).unwrap();
        let result = &restored.case_results["NL"];
        assert!((result.joint_displacements["J"][2] - sign * 0.4).abs() < 1e-9);
        assert!((result.link_forces["L"][0] - sign * 30.).abs() < 1e-8);
        assert!((result.base_reactions[2] + sign * 30.).abs() < 1e-8);
    }
}
#[test]
fn bad_link_edits_and_duplicate_names_preserve_model_and_results() {
    let mut api = cSapModel::new(Units::KipInF);
    api.inner.add_point("J", 0., 0., 0.);
    assert_eq!(api.prop_link().set_definition(LinkProp::default()), OAPI_OK);
    assert_eq!(
        api.link_obj().add_by_point("J", None, "LIN1", "L").0,
        OAPI_OK
    );
    api.inner
        .case_results
        .insert("OLD".into(), Default::default());
    let before = api.inner.clone();
    assert_eq!(
        api.link_obj().add_by_point("J", None, "LIN1", "L").0,
        OAPI_ERR
    );
    assert_eq!(
        api.link_obj()
            .add_by_point("MISSING", None, "LIN1", "BAD")
            .0,
        OAPI_ERR
    );
    assert_eq!(
        api.link_obj().add_by_point("J", Some("J"), "LIN1", "BAD").0,
        OAPI_ERR
    );
    let mut bad = LinkProp::default();
    bad.ke[0] = f64::NAN;
    assert_eq!(api.prop_link().set_definition(bad), OAPI_ERR);
    let mut bad = LinkProp::default();
    bad.dof_nonlinear[0] = true;
    assert_eq!(api.prop_link().set_definition(bad), OAPI_ERR);
    assert_eq!(api.inner, before);
    api.inner.is_locked = true;
    let before = api.inner.clone();
    assert_eq!(api.link_obj().delete("L"), OAPI_ERR);
    assert_eq!(api.inner, before);
}
