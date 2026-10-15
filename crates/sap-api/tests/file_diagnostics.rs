use sap_api::{cSapModel, OAPI_ERR, OAPI_OK};
use sap_units::Units;
#[test]
fn checked_file_api_reports_preservation_limits_without_overwriting_on_conflict() {
    let mut api = cSapModel::new(Units::KipInF);
    api.inner=sap_format::parse_s2k("TABLE: \"PROGRAM CONTROL\"\n CurrUnits=\"Kip, in, F\"\nTABLE: \"JOINT COORDINATES\"\n Joint=A XorR=0 Y=0 Z=0 VendorLength=25\nTABLE: \"EXTERNAL DATA\"\n X=5\n").unwrap();
    let path = std::env::temp_dir().join(format!("rea-api-diagnostics-{}.s2k", std::process::id()));
    let path = path.to_str().unwrap();
    assert_eq!(api.file().save(path), OAPI_OK);
    assert!(!api.export_warnings.is_empty());
    assert!(api.last_file_error.is_none());
    let original = std::fs::read(path).unwrap();
    api.set_present_units(Units::KnMC);
    assert_eq!(api.file().save(path), OAPI_ERR);
    assert!(api
        .last_file_error
        .as_ref()
        .unwrap()
        .contains("unit changes"));
    assert_eq!(std::fs::read(path).unwrap(), original);
    assert!(api.export_warnings.is_empty());
    std::fs::remove_file(path).unwrap();
}
