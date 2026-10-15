use sap_api::{cSapModel, OAPI_ERR, OAPI_OK};
use sap_model::{CaseResults, FrameLoadDistributed, FrameLoadPoint};
use sap_solver::analyze_model;
use sap_units::Units;

fn cantilever() -> cSapModel {
    let mut api = cSapModel::new(Units::KipInF);
    let m = &mut api.inner;
    m.load_cases.shift_remove("MODAL");
    m.load_patterns.get_mut("DEAD").unwrap().self_wt_multiplier = 0.0;
    // Euler-Bernoulli comparison avoids shear integration approximation changes.
    let section = m.frame_sections.get_mut("FSEC1").unwrap();
    section.as2 = 0.0;
    section.as3 = 0.0;
    m.add_point("base", 0.0, 0.0, 0.0);
    m.add_point("tip", 120.0, 0.0, 0.0);
    m.points.get_mut("base").unwrap().restraint = [true; 6];
    m.add_frame("beam", "base", "tip", "FSEC1");
    let f = m.frames.get_mut("beam").unwrap();
    f.selected = true;
    f.loads_dist.push(FrameLoadDistributed {
        load_pat: "DEAD".into(),
        my_type: 1,
        dir: 6,
        csys: "GLOBAL".into(),
        rel_dist1: 0.125,
        rel_dist2: 0.875,
        val1: -0.2,
        val2: -0.8,
    });
    f.loads_point.push(FrameLoadPoint {
        load_pat: "DEAD".into(),
        my_type: 1,
        dir: 6,
        csys: "GLOBAL".into(),
        rel_dist: 0.5,
        val: -3.0,
    });
    api
}

#[test]
fn divided_cantilever_preserves_tip_displacement_and_support_reaction() {
    let mut api = cantilever();
    analyze_model(&mut api.inner).unwrap();
    let reference = api.inner.case_results["DEAD"].clone();
    // Editing analysed/locked geometry must fail atomically.
    let before = api.inner.clone();
    assert!(api.edit().divide_selected_frames(4).is_err());
    assert_eq!(api.inner, before);
    assert_eq!(api.set_model_is_locked(false), OAPI_OK);
    api.edit().divide_selected_frames(4).unwrap();
    analyze_model(&mut api.inner).unwrap();
    let result = &api.inner.case_results["DEAD"];
    for k in 0..6 {
        let a = reference.joint_displacements["tip"][k];
        let b = result.joint_displacements["tip"][k];
        assert!(
            (a - b).abs() <= 1e-9 * a.abs().max(1.0),
            "tip DOF {k}: {a} vs {b}"
        );
        let a = reference.joint_reactions["base"][k];
        let b = result.joint_reactions["base"][k];
        assert!(
            (a - b).abs() <= 1e-9 * a.abs().max(1.0),
            "reaction DOF {k}: {a} vs {b}"
        );
    }
}

#[test]
fn new_assignment_setters_are_atomic_on_invalid_input_and_clear_results_on_success() {
    let mut api = cantilever();
    api.inner.add_point("p3", 120.0, 10.0, 0.0);
    api.inner.add_point("p4", 0.0, 10.0, 0.0);
    api.inner.add_area(
        "roof",
        vec!["base".into(), "tip".into(), "p3".into(), "p4".into()],
        "ASEC1",
    );
    api.inner
        .case_results
        .insert("OLD".into(), CaseResults::default());
    let before = api.inner.clone();
    assert_eq!(api.area_obj().set_modifiers("missing", [1.0; 10]), OAPI_ERR);
    assert_eq!(
        api.area_obj()
            .set_load_surface_pressure("roof", "MISSING", -2, 2.0, None, true),
        OAPI_ERR
    );
    assert_eq!(
        api.area_obj()
            .set_load_surface_pressure("roof", "DEAD", 5, 2.0, None, true),
        OAPI_ERR
    );
    assert_eq!(
        api.area_obj()
            .set_load_surface_pressure("roof", "DEAD", -2, f64::INFINITY, None, true),
        OAPI_ERR
    );
    assert_eq!(api.prop_area().set_modifiers("ASEC1", [-1.0; 10]), OAPI_ERR);
    assert_eq!(
        api.frame_obj()
            .set_releases("beam", [false; 6], [false; 6], [1.0; 6], [0.0; 6]),
        OAPI_ERR
    );
    assert_eq!(
        api.area_obj()
            .set_load_surface_pressure("roof", "DEAD", -2, 2.0, Some("PATTERN"), true),
        OAPI_ERR
    );
    assert_eq!(api.inner, before);
    let mut modifiers = [1.0; 10];
    modifiers[1] = 0.0;
    assert_eq!(api.area_obj().set_modifiers("roof", modifiers), OAPI_OK);
    assert!(api.inner.case_results.is_empty());
    assert_eq!(
        api.area_obj()
            .set_load_surface_pressure("roof", "DEAD", -2, 2.0, None, true),
        OAPI_OK
    );
    assert_eq!(
        api.area_obj()
            .set_load_surface_pressure("roof", "DEAD", -1, 3.0, None, true),
        OAPI_OK
    );
    assert_eq!(
        api.area_obj()
            .set_load_surface_pressure("roof", "DEAD", -2, 4.0, None, true),
        OAPI_OK
    );
    assert_eq!(api.inner.areas["roof"].loads_pressure.len(), 2);
    assert_eq!(
        api.inner.areas["roof"]
            .loads_pressure
            .iter()
            .find(|p| p.face == -1)
            .unwrap()
            .val,
        3.0
    );
    api.inner.is_locked = true;
    let before = api.inner.clone();
    assert_eq!(api.prop_area().set_modifiers("ASEC1", [0.0; 10]), OAPI_ERR);
    assert_eq!(
        api.area_obj()
            .set_load_surface_pressure("roof", "DEAD", -2, 2.0, None, true),
        OAPI_ERR
    );
    assert_eq!(api.inner, before);
}

#[test]
fn surface_pressure_api_produces_the_expected_vertical_resultant() {
    for (face, pressure, expected) in [(-2, 2.0, 48.0), (-1, 2.0, -48.0)] {
        let mut api = cSapModel::new(Units::KipInF);
        api.inner.load_cases.shift_remove("MODAL");
        api.inner
            .load_patterns
            .get_mut("DEAD")
            .unwrap()
            .self_wt_multiplier = 0.0;
        for (id, xyz) in [
            ("1", [0.0, 0.0, 0.0]),
            ("2", [6.0, 0.0, 0.0]),
            ("3", [6.0, 4.0, 0.0]),
            ("4", [0.0, 4.0, 0.0]),
        ] {
            api.inner.add_point(id, xyz[0], xyz[1], xyz[2]);
            api.inner.points.get_mut(id).unwrap().restraint = [true; 6];
        }
        api.inner.add_area(
            "panel",
            vec!["1".into(), "2".into(), "3".into(), "4".into()],
            "ASEC1",
        );
        assert_eq!(
            api.area_obj()
                .set_load_surface_pressure("panel", "DEAD", face, pressure, None, true),
            OAPI_OK
        );
        analyze_model(&mut api.inner).unwrap();
        let reaction: f64 = api.inner.case_results["DEAD"]
            .joint_reactions
            .values()
            .map(|r| r[2])
            .sum();
        assert!(
            (reaction - expected).abs() < 1e-9,
            "pressure face {face}: resultant {reaction}, expected {expected}"
        );
    }
}
