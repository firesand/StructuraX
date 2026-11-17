//! Wave 5, ledger D16: a concrete member with no rebar yield stress anywhere is
//! refused instead of being designed with an assumed 60 ksi.
//!
//! References: the refusal is the project rule (CONTRIBUTING.md: unsupported or
//! unspecified input is refused naming the object, never approximated). The
//! unchanged path is checked against itself with the yield stress given on the
//! concrete material (the pre-wave-5 route), bitwise, and the dependence on fy
//! against the ACI 318 flexural closed form As = Mu / (phi fy (d - a/2)), which
//! at a given Mu and d decreases with fy.
use sap_design::try_run_concrete_design;
use sap_model::{
    CaseResults, ConcreteRebarConfig, FrameStationResult, MatType, Material, SapModel,
};
use sap_section::make_rectangular;
use sap_units::Units;

fn model(concrete_fy: f64, rebar_fy: Option<f64>) -> SapModel {
    let mut m = SapModel::new(Units::KipInF);
    m.concrete_design_code = "ACI 318-14".into();
    let mut concrete = Material::default_concrete_kip_in();
    concrete.fy = concrete_fy;
    concrete.fys = concrete_fy;
    m.materials.insert(concrete.name.clone(), concrete);
    let mut section = make_rectangular("BEAM", "4000Psi", 24.0, 14.0);
    if let Some(fy) = rebar_fy {
        let mut rebar = Material::default_steel_kip_in();
        rebar.name = "REBAR".into();
        rebar.mat_type = MatType::Rebar;
        rebar.fy = fy;
        rebar.fys = fy;
        m.materials.insert("REBAR".into(), rebar);
        section.concrete_rebar = Some(ConcreteRebarConfig {
            is_column: false,
            rebar_mat_long: "REBAR".into(),
            rebar_mat_confine: "REBAR".into(),
            cover_top: 2.5,
            cover_bot: 2.5,
            num_bars_3dir: 3,
            num_bars_2dir: 3,
            total_rebar_area: 0.0,
        });
    }
    m.frame_sections.insert("BEAM".into(), section);
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 120.0, 0.0, 0.0);
    m.add_frame("B1", "1", "2", "BEAM");
    let mut cr = CaseResults {
        case_name: "COMBO1".into(),
        ..Default::default()
    };
    cr.frame_forces.insert(
        "B1".into(),
        vec![FrameStationResult {
            station: 60.0,
            rel_dist: 0.5,
            m3: 2000.0,
            ..Default::default()
        }],
    );
    m.case_results.insert("COMBO1".into(), cr);
    m
}

fn as_bot(m: &SapModel) -> f64 {
    m.concrete_design_results["B1"].as_bot
}

#[test]
fn a_library_concrete_without_rebar_yield_is_refused_naming_section_and_material() {
    let mut m = model(0.0, None);
    let error = try_run_concrete_design(&mut m).unwrap_err();
    assert_eq!(
        error,
        sap_design::DesignError::MissingRebarYield {
            frame: "B1".into(),
            section: "BEAM".into(),
            material: "4000Psi".into(),
        }
    );
    let err = error.to_string();
    assert!(err.contains("'BEAM'"), "section not named: {err}");
    assert!(err.contains("'4000Psi'"), "material not named: {err}");
    assert!(err.contains("'B1'"), "frame not named: {err}");
    assert!(err.contains("fy"), "remedy not named: {err}");
    assert!(!err.contains("steel design"), "wrong discipline: {err}");
    assert!(m.concrete_design_results.is_empty());
}

#[test]
fn a_rebar_material_without_fy_is_refused_too() {
    let mut m = model(0.0, Some(0.0));
    let error = try_run_concrete_design(&mut m).unwrap_err();
    assert!(
        matches!(&error, sap_design::DesignError::MissingRebarYield { material, .. } if material == "REBAR"),
        "{error:?}"
    );
    let err = error.to_string();
    assert!(err.contains("'REBAR'"), "rebar material not named: {err}");
    assert!(m.concrete_design_results.is_empty());
}

#[test]
fn a_named_rebar_material_designs_bitwise_like_the_concrete_fy_route() {
    let mut via_concrete = model(60.0, None);
    let mut via_rebar = model(0.0, Some(60.0));
    try_run_concrete_design(&mut via_concrete).unwrap();
    try_run_concrete_design(&mut via_rebar).unwrap();
    assert_eq!(
        as_bot(&via_concrete).to_bits(),
        as_bot(&via_rebar).to_bits()
    );
    assert_eq!(
        via_concrete.concrete_design_results["B1"].status,
        via_rebar.concrete_design_results["B1"].status
    );
    assert!(as_bot(&via_rebar) > 0.0);
}

#[test]
fn the_required_area_follows_the_given_yield_stress() {
    let mut a = model(0.0, Some(60.0));
    let mut b = model(0.0, Some(75.0));
    try_run_concrete_design(&mut a).unwrap();
    try_run_concrete_design(&mut b).unwrap();
    // As = Mu / (phi fy (d - a/2)) with a small: As(60) / As(75) is just under 75/60.
    let ratio = as_bot(&a) / as_bot(&b);
    assert!(ratio > 1.2 && ratio < 1.26, "As(60)/As(75) = {ratio}");
}
