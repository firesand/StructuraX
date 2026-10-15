use sap_api::{cSapModel, OAPI_ERR, OAPI_OK};
use sap_model::{CaseResults, PointLoadForce};
use sap_units::Units;

fn api() -> cSapModel {
    let mut api = cSapModel::new(Units::KipInF);
    api.inner.add_point("A", 0., 0., 0.);
    api.inner.add_point("B", 12., 0., 0.);
    api.inner.add_frame("F", "A", "B", "FSEC1");
    api.inner
        .case_results
        .insert("OLD".into(), CaseResults::default());
    api
}

#[test]
fn invalid_point_assignments_and_duplicate_names_preserve_state_and_results() {
    let mut a = api();
    let before = a.inner.clone();
    assert_eq!(a.point_obj().add_cartesian(99., 9., 9., "A").0, OAPI_ERR);
    assert_eq!(
        a.point_obj().add_cartesian(f64::NAN, 0., 0., "C").0,
        OAPI_ERR
    );
    assert_eq!(a.point_obj().set_restraint("MISSING", [true; 6]), OAPI_ERR);
    assert_eq!(a.point_obj().set_spring("A", [-1.; 6]), OAPI_ERR);
    assert_eq!(
        a.point_obj().set_load_force("A", "MISSING", [1.; 6], true),
        OAPI_ERR
    );
    assert_eq!(
        a.point_obj()
            .set_load_force("A", "DEAD", [f64::NAN; 6], true),
        OAPI_ERR
    );
    assert_eq!(
        a.point_obj()
            .set_load_displ("A", "DEAD", [f64::INFINITY; 6], true),
        OAPI_ERR
    );
    assert_eq!(a.point_obj().set_mass("A", [-1.; 6], false), OAPI_ERR);
    assert_eq!(a.inner, before);
    a.inner.points["A"].mass = [f64::MAX; 6];
    let before = a.inner.clone();
    assert_eq!(a.point_obj().set_mass("A", [f64::MAX; 6], false), OAPI_ERR);
    assert_eq!(a.inner, before);
    a.inner.is_locked = true;
    let before = a.inner.clone();
    assert_eq!(a.point_obj().set_restraint("A", [true; 6]), OAPI_ERR);
    assert_eq!(a.point_obj().add_cartesian(1., 2., 3., "C").0, OAPI_ERR);
    assert_eq!(a.inner, before);
}

#[test]
fn invalid_frame_creation_never_leaves_orphan_points_or_overwrites_members() {
    let mut a = api();
    let before = a.inner.clone();
    assert_eq!(
        a.frame_obj().add_by_point("A", "MISSING", "FSEC1", "G").0,
        OAPI_ERR
    );
    assert_eq!(
        a.frame_obj().add_by_point("A", "A", "FSEC1", "G").0,
        OAPI_ERR
    );
    assert_eq!(
        a.frame_obj().add_by_point("A", "B", "MISSING", "G").0,
        OAPI_ERR
    );
    assert_eq!(
        a.frame_obj().add_by_point("A", "B", "FSEC1", "F").0,
        OAPI_ERR
    );
    assert_eq!(
        a.frame_obj()
            .add_by_coord(1., 2., 3., 4., 5., 6., "MISSING", "G")
            .0,
        OAPI_ERR
    );
    assert_eq!(
        a.frame_obj()
            .add_by_coord(1., 2., 3., 4., 5., 6., "FSEC1", "F")
            .0,
        OAPI_ERR
    );
    assert_eq!(
        a.frame_obj()
            .add_by_coord(1., 2., 3., f64::NAN, 5., 6., "FSEC1", "G")
            .0,
        OAPI_ERR
    );
    assert_eq!(a.inner, before);
    // A fixed tolerance used to collapse these distinct endpoints into one joint.
    let (status, id) = a
        .frame_obj()
        .add_by_coord(0., 0., 0., 1e-8, 0., 0., "FSEC1", "G");
    assert_eq!(status, OAPI_OK);
    assert_ne!(a.inner.frames[&id].point_i, a.inner.frames[&id].point_j);
    assert_eq!(a.inner.frames[&id].point_i, "A");
    assert!(a.inner.case_results.is_empty());
}

#[test]
fn invalid_frame_assignments_preserve_results_and_other_pattern_loads() {
    let mut a = api();
    let before = a.inner.clone();
    assert_eq!(a.frame_obj().set_section("F", "MISSING"), OAPI_ERR);
    assert_eq!(a.frame_obj().set_section("MISSING", "FSEC1"), OAPI_ERR);
    assert_eq!(a.frame_obj().set_local_axes("F", f64::NAN), OAPI_ERR);
    assert_eq!(a.frame_obj().set_modifiers("F", [-1.; 8]), OAPI_ERR);
    assert_eq!(
        a.frame_obj()
            .set_load_distributed("F", "DEAD", 1, 6, 0.8, 0.2, 1., 2., "GLOBAL", true),
        OAPI_ERR
    );
    assert_eq!(
        a.frame_obj()
            .set_load_point("F", "DEAD", 1, 6, 1.1, 1., "GLOBAL", true),
        OAPI_ERR
    );
    assert_eq!(
        a.frame_obj()
            .set_load_point("F", "DEAD", 99, 6, 0.5, 1., "GLOBAL", true),
        OAPI_ERR
    );
    assert_eq!(
        a.frame_obj()
            .set_load_point("F", "DEAD", 1, 99, 0.5, 1., "GLOBAL", true),
        OAPI_ERR
    );
    assert_eq!(
        a.frame_obj()
            .set_load_point("F", "DEAD", 1, 6, 0.5, 1., "MISSING", true),
        OAPI_ERR
    );
    assert_eq!(
        a.frame_obj()
            .set_load_temperature("F", "DEAD", 1, 3., Some("MISSING".into()), true),
        OAPI_ERR
    );
    assert_eq!(
        a.frame_obj().set_end_length_offset("F", 6., 6., 0.5),
        OAPI_ERR
    );
    assert_eq!(a.inner, before);
    let other = PointLoadForce {
        load_pat: "OTHER".into(),
        csys: "GLOBAL".into(),
        force: [3.; 6],
    };
    a.inner.points["A"].loads_force.push(other.clone());
    assert_eq!(
        a.point_obj().set_load_force("A", "DEAD", [2.; 6], true),
        OAPI_OK
    );
    assert_eq!(a.inner.points["A"].loads_force[0], other);
    assert_eq!(a.inner.points["B"], before.points["B"]);
    assert!(a.inner.case_results.is_empty());
}

#[test]
fn material_fields_are_assigned_and_unsupported_strength_or_lightweight_data_rejected() {
    let mut a = api();
    let before = a.inner.clone();
    assert_eq!(
        a.prop_material().set_osteel("A992Fy50", 50., 65., 55., 70.),
        OAPI_ERR
    );
    assert_eq!(
        a.prop_material().set_osteel("MISSING", 50., 65., 50., 65.),
        OAPI_ERR
    );
    assert_eq!(
        a.prop_material().set_oconcrete("4000Psi", 5., true),
        OAPI_ERR
    );
    assert_eq!(
        a.prop_material().set_oconcrete("4000Psi", f64::NAN, false),
        OAPI_ERR
    );
    assert_eq!(a.inner, before);
    assert_eq!(
        a.prop_material().set_osteel("A992Fy50", 55., 70., 55., 70.),
        OAPI_OK
    );
    assert_eq!(
        (
            a.inner.materials["A992Fy50"].fy,
            a.inner.materials["A992Fy50"].fu
        ),
        (55., 70.)
    );
    assert_eq!(
        a.prop_material().set_oconcrete("4000Psi", 5., false),
        OAPI_OK
    );
    assert_eq!(a.inner.materials["4000Psi"].fc, 5.);
    assert!(a.inner.case_results.is_empty());
}

#[test]
fn materials_created_in_si_units_have_physical_si_defaults() {
    let mut a = cSapModel::new(Units::KnMC);
    assert_eq!(
        a.prop_material()
            .set_material("SIsteel", sap_model::MatType::Steel),
        OAPI_OK
    );
    assert_eq!(
        a.prop_material()
            .set_material("SIconcrete", sap_model::MatType::Concrete),
        OAPI_OK
    );
    let steel = &a.inner.materials["SIsteel"];
    let ksi_to_kn_m2 = 4.4482216152605 / 0.0254_f64.powi(2);
    let compare = |actual: f64, expected: f64| {
        assert!(
            (actual - expected).abs() <= 1e-10 * expected.abs().max(1.),
            "{actual} != {expected}"
        )
    };
    compare(steel.e, 29000. * ksi_to_kn_m2);
    compare(steel.fy, 50. * ksi_to_kn_m2);
    compare(
        steel.unit_weight,
        0.490 * 4.4482216152605 / 0.3048_f64.powi(3),
    );
    compare(steel.unit_mass, steel.unit_weight / 9.80665);
    compare(steel.a, 6.5e-6 * 1.8);
    compare(a.inner.materials["SIconcrete"].fc, 4. * ksi_to_kn_m2);
    a.inner
        .case_results
        .insert("OLD".into(), CaseResults::default());
    let before = a.inner.clone();
    for family in [
        sap_model::MatType::Aluminum,
        sap_model::MatType::ColdFormed,
        sap_model::MatType::Rebar,
        sap_model::MatType::Tendon,
    ] {
        assert_eq!(
            a.prop_material().set_material("unsupported", family),
            OAPI_ERR
        );
    }
    assert_eq!(
        a.prop_material()
            .set_material(" ", sap_model::MatType::Steel),
        OAPI_ERR
    );
    assert_eq!(a.inner, before);
}

#[test]
fn isotropic_and_density_setters_validate_physical_data_without_clearing_on_failure() {
    let mut a = api();
    let before = a.inner.clone();
    for (e, u, alpha, g) in [
        (0., 0.3, 1e-5, 0.),
        (-1., 0.3, 1e-5, 0.),
        (200., -1., 1e-5, 0.),
        (200., 0.5, 1e-5, 0.),
        (200., 0.3, f64::NAN, 0.),
        (200., 0.3, 1e-5, -1.),
        (f64::MAX, -0.9999999999999999, 1e-5, 0.),
    ] {
        assert_eq!(
            a.prop_material()
                .set_mp_isotropic("A992Fy50", e, u, alpha, g),
            OAPI_ERR
        );
    }
    assert_eq!(
        a.prop_material()
            .set_mp_isotropic("MISSING", 200., 0.3, 1e-5, 0.),
        OAPI_ERR
    );
    for (weight, mass) in [(-1., 2.), (1., -2.), (f64::NAN, 2.), (1., f64::INFINITY)] {
        assert_eq!(
            a.prop_material()
                .set_weight_and_mass("A992Fy50", weight, mass),
            OAPI_ERR
        );
    }
    assert_eq!(
        a.prop_material().set_weight_and_mass("MISSING", 1., 2.),
        OAPI_ERR
    );
    assert_eq!(a.inner, before);
    assert_eq!(
        a.prop_material()
            .set_mp_isotropic("A992Fy50", 200., 0.25, 1e-5, 0.),
        OAPI_OK
    );
    assert_eq!(a.inner.materials["A992Fy50"].g, 80.);
    assert_eq!(
        a.prop_material().set_weight_and_mass("A992Fy50", 0., 0.),
        OAPI_OK
    );
    assert_eq!(a.inner.materials["A992Fy50"].unit_weight, 0.);
    assert_eq!(a.inner.materials["A992Fy50"].unit_mass, 0.);
    assert!(a.inner.case_results.is_empty());
    a.inner.is_locked = true;
    let before = a.inner.clone();
    assert_eq!(
        a.prop_material()
            .set_material("new", sap_model::MatType::Steel),
        OAPI_ERR
    );
    assert_eq!(
        a.prop_material()
            .set_mp_isotropic("A992Fy50", 250., 0.3, 2e-5, 0.),
        OAPI_ERR
    );
    assert_eq!(
        a.prop_material().set_weight_and_mass("A992Fy50", 1., 2.),
        OAPI_ERR
    );
    assert_eq!(a.inner, before);
}

#[test]
fn ambiguous_coincident_joint_identity_rejects_coordinate_creation_atomically() {
    let mut a = api();
    a.inner.add_point("coincident_A", 0., 0., 0.);
    a.inner.points["coincident_A"].restraint = [true; 6];
    let before = a.inner.clone();
    assert_eq!(
        a.frame_obj()
            .add_by_coord(0., 0., 0., 25., 0., 0., "FSEC1", "ambiguous")
            .0,
        OAPI_ERR
    );
    assert_eq!(a.inner, before);
    // The caller may deliberately select either identity through the named route.
    assert_eq!(
        a.frame_obj()
            .add_by_point("coincident_A", "B", "FSEC1", "explicit")
            .0,
        OAPI_OK
    );
    assert_eq!(a.inner.frames["explicit"].point_i, "coincident_A");
}
