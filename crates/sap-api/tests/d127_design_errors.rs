//! D127 through the API: a design run that refuses a member completes (`OAPI_OK`), the refused
//! member has a message (`get_error_summary`, the vendor's ErrorSummary) and a row in the design
//! summary tables with Status "Not designed"; a run error is `OAPI_ERR`.

use sap_api::{cSapModel, OAPI_ERR, OAPI_OK};
use sap_model::{CaseResults, FramePropType, FrameStationResult, Material};
use sap_section::{make_i_section, make_rectangular};
use sap_units::Units;

fn st(p: f64, v2: f64, m3: f64, t: f64) -> FrameStationResult {
    FrameStationResult {
        station: 60.0,
        rel_dist: 0.5,
        p,
        v2,
        m3,
        t,
        ..Default::default()
    }
}

fn table(api: &mut cSapModel, key: &str) -> (Vec<String>, Vec<Vec<String>>) {
    let (mut version, mut included, mut n, mut data) = (0, Vec::new(), 0, Vec::new());
    assert_eq!(
        api.database_tables().get_table_for_display_array(
            key, &[], "All", &mut version, &mut included, &mut n, &mut data,
        ),
        OAPI_OK
    );
    let k = included.len();
    let rows = data.chunks(k).map(|c| c.to_vec()).collect();
    (included, rows)
}

fn cell<'a>(fields: &[String], row: &'a [String], field: &str) -> &'a str {
    &row[fields.iter().position(|f| f == field).unwrap()]
}

/// A designed I beam "BEAM" and a single angle "ANG" in compression (refused under AISC).
fn steel_api() -> cSapModel {
    let mut api = cSapModel::new(Units::KipInF);
    let beam = make_i_section("IB", "A992Fy50", 16.0, 7.0, 0.5, 0.3, 7.0, 0.5);
    let mut angle = make_rectangular("L8X8", "A992Fy50", 8.0, 8.0);
    angle.prop_type = FramePropType::Angle;
    angle.area = 7.75;
    angle.i33 = 48.4375;
    angle.i22 = 48.4375;
    angle.torsional_j = 0.6;
    angle.s33 = 8.0;
    angle.z33 = 14.0;
    angle.s22 = 8.0;
    angle.z22 = 14.0;
    api.inner.frame_sections.insert("IB".into(), beam);
    api.inner.frame_sections.insert("L8X8".into(), angle);
    api.inner.add_point("A", 0.0, 0.0, 0.0);
    api.inner.add_point("B", 120.0, 0.0, 0.0);
    api.inner.add_point("C", 0.0, 50.0, 0.0);
    api.inner.add_point("D", 120.0, 50.0, 0.0);
    api.inner.add_frame("BEAM", "A", "B", "IB");
    api.inner.add_frame("ANG", "C", "D", "L8X8");
    let mut cr = CaseResults {
        case_name: "LC".into(),
        ..Default::default()
    };
    cr.frame_forces.insert("BEAM".into(), vec![st(0.0, 5.0, 100.0, 0.0)]);
    cr.frame_forces.insert("ANG".into(), vec![st(-1.0, 0.0, 0.0, 0.0)]);
    api.inner.case_results.insert("LC".into(), cr);
    assert_eq!(api.design_steel().set_code("AISC 360-10"), OAPI_OK);
    api
}

#[test]
fn steel_run_with_a_refused_member_is_ok_and_has_an_error_summary() {
    let mut api = steel_api();
    assert_eq!(api.design_steel().start_design(), OAPI_OK);
    assert!(api.design_steel().get_summary_results("BEAM").is_some());
    assert!(api.design_steel().get_summary_results("ANG").is_none());
    let text = api.design_steel().get_error_summary("ANG").unwrap();
    assert!(text.contains("'ANG'") && text.contains("single angle"), "{text}");
    assert!(api.design_steel().get_error_summary("BEAM").is_none());

    // the summary table has the refused frame beside the designed one
    let (fields, rows) = table(&mut api, "Steel Design 1 - Summary Data");
    assert_eq!(fields.last().unwrap(), "ErrMsg");
    assert_eq!(rows.len(), 2);
    let ang = rows.iter().find(|r| cell(&fields, r, "Frame") == "ANG").unwrap();
    assert_eq!(cell(&fields, ang, "Status"), "Not designed");
    assert_eq!(cell(&fields, ang, "ErrMsg"), text);
    assert_eq!(cell(&fields, ang, "DesignSect"), "L8X8");
    assert_eq!(cell(&fields, ang, "Ratio"), "");
    let beam = rows.iter().find(|r| cell(&fields, r, "Frame") == "BEAM").unwrap();
    assert_eq!(cell(&fields, beam, "ErrMsg"), "");
    assert_ne!(cell(&fields, beam, "Ratio"), "");

    // a code change clears the message, like the results
    assert_eq!(api.design_steel().set_code("AISC 360-10 ASD"), OAPI_OK);
    assert!(api.design_steel().get_error_summary("ANG").is_none());
}

#[test]
fn a_model_whose_every_member_is_refused_still_has_the_summary_rows() {
    let mut api = steel_api();
    api.inner.frames.shift_remove("BEAM");
    api.inner.case_results.get_mut("LC").unwrap().frame_forces.shift_remove("BEAM");
    assert_eq!(api.design_steel().start_design(), OAPI_OK);
    assert!(api.inner.steel_design_results.is_empty());
    let (_, rows) = table(&mut api, "Steel Design 1 - Summary Data");
    assert_eq!(rows.len(), 1);
}

#[test]
fn steel_run_error_is_err() {
    let mut api = steel_api();
    api.inner.steel_design_code = "No such code".into();
    api.inner.steel_design_errors.insert("OLD".into(), "old".into());
    assert_eq!(api.design_steel().start_design(), OAPI_ERR);
    assert!(api.inner.steel_design_results.is_empty() && api.inner.steel_design_errors.is_empty());
}

fn concrete_api() -> cSapModel {
    let mut api = cSapModel::new(Units::KipInF);
    let mut c = Material::default_concrete_kip_in();
    c.name = "C4".into();
    c.fc = 4.0;
    c.fy = 60.0;
    c.fys = 60.0;
    api.inner.materials.insert("C4".into(), c);
    let rect = make_rectangular("RB", "C4", 24.0, 12.0);
    let mut other = make_rectangular("OB", "C4", 24.0, 12.0);
    other.prop_type = FramePropType::I;
    api.inner.frame_sections.insert("RB".into(), rect);
    api.inner.frame_sections.insert("OB".into(), other);
    api.inner.add_point("A", 0.0, 0.0, 0.0);
    api.inner.add_point("B", 120.0, 0.0, 0.0);
    api.inner.add_point("C", 0.0, 50.0, 0.0);
    api.inner.add_point("D", 120.0, 50.0, 0.0);
    api.inner.add_frame("BM", "A", "B", "RB");
    api.inner.add_frame("TB", "C", "D", "OB");
    let mut cr = CaseResults {
        case_name: "LC".into(),
        ..Default::default()
    };
    cr.frame_forces.insert("BM".into(), vec![st(0.0, 30.0, 1500.0, 0.0)]);
    cr.frame_forces.insert("TB".into(), vec![st(0.0, 30.0, 1500.0, 80.0)]);
    api.inner.case_results.insert("LC".into(), cr);
    api
}

#[test]
fn concrete_run_with_a_refused_beam_is_ok_and_has_an_error_summary() {
    let mut api = concrete_api();
    assert_eq!(api.design_concrete().start_design(), OAPI_OK);
    assert!(api.design_concrete().get_summary_results("BM").is_some());
    assert!(api.design_concrete().get_summary_results("TB").is_none());
    let text = api.design_concrete().get_error_summary("TB").unwrap();
    assert!(text.contains("'TB'") && text.contains("torsion"), "{text}");

    let (fields, rows) = table(&mut api, "Concrete Design 2 - Beam Summary Data");
    assert_eq!(rows.len(), 2);
    let tb = rows.iter().find(|r| cell(&fields, r, "Frame") == "TB").unwrap();
    assert_eq!(cell(&fields, tb, "Status"), "Not designed");
    assert_eq!(cell(&fields, tb, "ErrMsg"), text);
    assert_eq!(cell(&fields, tb, "TopAs"), "");
    // no column is in the model: the column table has no row
    let (_, columns) = table(&mut api, "Concrete Design 1 - Column Summary Data");
    assert!(columns.is_empty());

    assert_eq!(api.design_concrete().set_framing_type("BM", sap_model::ConcreteFramingType::SwaySpecial), OAPI_OK);
    assert!(api.design_concrete().get_error_summary("TB").is_none());
}

#[test]
fn concrete_run_error_is_err() {
    let mut api = concrete_api();
    api.inner.concrete_design_code = "No such code".into();
    assert_eq!(api.design_concrete().start_design(), OAPI_ERR);
}
