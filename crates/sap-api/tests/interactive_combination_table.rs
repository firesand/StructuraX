use sap_api::{cSapModel, OAPI_ERR, OAPI_OK};
use sap_model::ComboType;
use sap_units::Units;

fn apply(api: &mut cSapModel, fields: &[&str], cells: &[&str]) -> i32 {
    let mut db = api.database_tables();
    if db.set_table_for_editing_array_str("COMBINATION DEFINITIONS", fields, 1, cells) != OAPI_OK {
        return OAPI_ERR;
    }
    db.apply_edited_tables_simple(true)
}

#[test]
fn combination_table_round_trips_every_canonical_type() {
    for kind in [
        ComboType::LinearAdd,
        ComboType::Envelope,
        ComboType::AbsAdd,
        ComboType::Srss,
        ComboType::RangeAdd,
    ] {
        let mut api = cSapModel::new(Units::KipInF);
        assert_eq!(api.resp_combo().add("C", kind), OAPI_OK);
        assert_eq!(api.resp_combo().set_case_list("C", "DEAD", 2.0), OAPI_OK);
        let before = api.inner.load_combos["C"].clone();

        let (status, fields, records, cells) = api
            .database_tables()
            .get_table_for_editing_array_vec("COMBINATION DEFINITIONS", "ALL");
        assert_eq!(status, OAPI_OK);
        assert_eq!(records, 1);
        let fields: Vec<_> = fields.iter().map(String::as_str).collect();
        let cells: Vec<_> = cells.iter().map(String::as_str).collect();
        assert_eq!(apply(&mut api, &fields, &cells), OAPI_OK);
        assert_eq!(api.inner.load_combos["C"], before, "type {kind:?}");
    }
}

#[test]
fn combination_table_accepts_type_aliases_and_omitted_defaults() {
    let cases = [
        ("Linear Add", ComboType::LinearAdd),
        ("Linear_Addition", ComboType::LinearAdd),
        ("Envelope", ComboType::Envelope),
        ("Absolute Add", ComboType::AbsAdd),
        ("AbsAdd", ComboType::AbsAdd),
        ("SRSS", ComboType::Srss),
        ("Range Add", ComboType::RangeAdd),
    ];
    for (input, expected) in cases {
        let mut api = cSapModel::new(Units::KipInF);
        assert_eq!(
            apply(
                &mut api,
                &["ComboName", "ComboType", "CaseName"],
                &["C", input, "DEAD"]
            ),
            OAPI_OK,
            "{input}"
        );
        let combo = &api.inner.load_combos["C"];
        assert_eq!(combo.combo_type, expected, "{input}");
        assert_eq!(combo.cases, vec![("DEAD".to_string(), 1.0)]);
    }
}

#[test]
fn malformed_combo_cells_are_rejected_without_mutating_model() {
    let mut api = cSapModel::new(Units::KipInF);
    assert_eq!(api.resp_combo().add("KEEP", ComboType::RangeAdd), OAPI_OK);
    assert_eq!(api.resp_combo().set_case_list("KEEP", "DEAD", 2.0), OAPI_OK);
    let before = api.inner.clone();

    for (fields, cells) in [
        (
            &["ComboName", "ComboType", "CaseName", "ScaleFactor"][..],
            &["KEEP", "RangeAdd", "OTHER", "not-a-number"][..],
        ),
        (
            &["ComboName", "ComboType", "CaseName", "ScaleFactor"][..],
            &["KEEP", "VendorSpecial", "OTHER", "3.0"][..],
        ),
    ] {
        assert_eq!(apply(&mut api, fields, cells), OAPI_ERR);
        assert_eq!(api.inner, before);
    }
}

#[test]
fn repeated_contributors_must_agree_on_the_combination_type() {
    let mut api = cSapModel::new(Units::KipInF);
    let fields = ["ComboName", "ComboType", "CaseName"];
    let cells = ["C", "RangeAdd", "DEAD", "C", "LinearAdd", "LIVE"];
    let mut tables = api.database_tables();
    assert_eq!(
        tables.set_table_for_editing_array_str("COMBINATION DEFINITIONS", &fields, 2, &cells),
        OAPI_OK
    );
    assert_eq!(tables.apply_edited_tables_simple(true), OAPI_ERR);
    assert!(!api.inner.load_combos.contains_key("C"));
}
