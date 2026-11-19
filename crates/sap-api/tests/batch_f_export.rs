//! Batch F: native export after an analysis (D120).

use sap_format::sdb::native::pack_model_to_native_sdb;
use sap_model::{ComboType, LoadCase, LoadCombo, MatType, SapModel};
use sap_units::Units;

fn cantilever() -> SapModel {
    let mut m = SapModel::new(Units::KipInF);
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 0.0, 0.0, 120.0);
    m.points.get_mut("1").unwrap().restraint = [true; 6];
    m.add_frame("F1", "1", "2", "FSEC1");
    // A case that is not run, and a combination that needs it: its result is
    // unavailable (D78), which `analyze_model` records in `unavailable_results`.
    let mut idle = LoadCase::linear_static("IDLE", "DEAD", 1.0);
    idle.run_flag = false;
    m.load_cases.insert("IDLE".into(), idle);
    m.load_combos.insert(
        "C1".into(),
        LoadCombo {
            name: "C1".into(),
            combo_type: ComboType::LinearAdd,
            cases: vec![("IDLE".into(), 1.0)],
        },
    );
    m
}

#[test]
fn d120_native_export_succeeds_after_a_combination_lost_a_contributor() {
    let mut m = cantilever();
    sap_solver::analyze_model(&mut m).expect("analysis");
    assert!(
        m.unavailable_results.contains_key("C1"),
        "precondition: C1 must be unavailable, got {:?}",
        m.unavailable_results
    );
    // Before the repair: "native serialization/projection would change represented model data".
    pack_model_to_native_sdb(&m).expect("export of an analysed model must succeed");
}

#[test]
fn d120_a_real_unsupported_difference_is_still_refused() {
    let mut m = cantilever();
    sap_solver::analyze_model(&mut m).expect("analysis");
    assert!(!m.unavailable_results.is_empty());
    // A Rebar material whose shear yield differs from its yield has no native form.
    let mut rebar = m.materials["A992Fy50"].clone();
    rebar.name = "Rb".into();
    rebar.mat_type = MatType::Rebar;
    rebar.fy = 60.0;
    rebar.fys = 40.0;
    m.materials.insert("Rb".into(), rebar);
    assert!(pack_model_to_native_sdb(&m).is_err());
}
