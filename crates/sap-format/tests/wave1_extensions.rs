//! Persistence of the mass source, automatic seismic loads, Ritz generation
//! controls, mode superposition, Auto Select state and steel design
//! preferences: S2K keeps them through reconstruction extension tables and
//! columns, and the native writer refuses a model that carries them instead
//! of dropping them.

use sap_format::sdb::native::pack_model_to_native_sdb;
use sap_format::{parse_s2k, write_s2k, FormatError};
use sap_model::auto_select::AutoSelectState;
use sap_model::ritz::RitzLoadControl;
use sap_model::seismic::{AutoSeismicAsce710, SeismicDirection, SeismicPeriod};
use sap_model::*;

fn base() -> SapModel {
    let mut m = SapModel::default();
    m.add_point("A", 0., 0., 0.);
    m.add_point("B", 12., 0., 0.);
    m.add_frame("F", "A", "B", "FSEC1");
    m
}

fn quake(m: &mut SapModel, name: &str) {
    m.load_patterns.insert(
        name.into(),
        LoadPattern {
            name: name.into(),
            pat_type: LoadPatternType::Quake,
            self_wt_multiplier: 0.0,
        },
    );
}

#[test]
fn a_model_without_the_new_data_writes_none_of_the_extension_tables() {
    let text = write_s2k(&base());
    for marker in [
        "RUST CASE - MODAL RITZ CONTROLS",
        "RUST MASS SOURCE",
        "RUST AUTO SEISMIC",
        "RUST STEEL DESIGN PREFERENCES",
        "RustModalCase",
        "AutoSelDesign",
    ] {
        assert!(!text.contains(marker), "unexpected {marker}");
    }
}

#[test]
fn s2k_keeps_every_new_definition() {
    let mut m = base();
    m.mass_source.element_self_mass = false;
    m.mass_source.load_patterns = vec![("DEAD".into(), 1.0), ("LIVE LOAD".into(), 0.25)];
    m.load_patterns.insert(
        "LIVE LOAD".into(),
        LoadPattern {
            name: "LIVE LOAD".into(),
            pat_type: LoadPatternType::Live,
            self_wt_multiplier: 0.0,
        },
    );
    quake(&mut m, "EX");
    quake(&mut m, "EY");
    quake(&mut m, "EZ");
    m.auto_seismic.insert(
        "EX".into(),
        AutoSeismicAsce710 {
            direction: SeismicDirection::X,
            eccentricity_ratio: 0.0,
            period: SeismicPeriod::Approximate { ct: 0.028, x: 0.8 },
            r: 7.0,
            omega0: 2.5,
            cd: 5.5,
            ie: 1.0,
            ss: 1.23,
            s1: 0.82,
            tl: 8.0,
            fa: Some(1.008),
            fv: Some(1.5),
            z_min: Some(0.0),
            z_max: Some(230.4),
        },
    );
    m.auto_seismic.insert(
        "EY".into(),
        AutoSeismicAsce710 {
            direction: SeismicDirection::Y,
            period: SeismicPeriod::User { t: 0.73 },
            fa: None,
            fv: None,
            z_min: None,
            z_max: None,
            ..AutoSeismicAsce710::default()
        },
    );
    m.auto_seismic.insert(
        "EZ".into(),
        AutoSeismicAsce710 {
            period: SeismicPeriod::ProgramCalculated,
            ..AutoSeismicAsce710::default()
        },
    );
    m.steel_design_prefs.dc_ratio_limit = 0.9;

    let modal = m.load_cases.get_mut("MODAL").unwrap();
    modal.modal_sub_type = ModalSubType::Ritz;
    modal.loads = vec![
        LoadCaseAssignment { load_name: "UX".into(), scale_factor: 1.0 },
        LoadCaseAssignment { load_name: "DEAD".into(), scale_factor: 2.0 },
    ];
    modal.ritz_controls = vec![
        RitzLoadControl {
            load_name: "UX".into(),
            max_cycles: 0,
            target_dynamic_participation: 0.99,
        },
        RitzLoadControl {
            load_name: "DEAD".into(),
            max_cycles: 3,
            target_dynamic_participation: 0.0,
        },
    ];
    let mut history = LoadCase::linear_static("MHIST", "DEAD", 1.0);
    history.case_type = LoadCaseType::LinearHistory;
    history.loads.clear();
    history.modal_case = Some("MODAL".into());
    m.load_cases.insert("MHIST".into(), history);

    m.frames["F"].auto_select = Some(AutoSelectState {
        design: "W16X31".into(),
        analysis: Some("W16X26".into()),
    });
    m.add_frame("G", "A", "B", "FSEC1");
    m.frames["G"].auto_select = Some(AutoSelectState {
        design: String::new(),
        analysis: None,
    });

    let text = write_s2k(&m);
    let back = parse_s2k(&text).unwrap();
    assert_eq!(back.mass_source, m.mass_source);
    assert_eq!(back.auto_seismic, m.auto_seismic);
    assert_eq!(back.steel_design_prefs, m.steel_design_prefs);
    assert_eq!(back.load_cases["MODAL"].loads, m.load_cases["MODAL"].loads);
    assert_eq!(
        back.load_cases["MODAL"].ritz_controls,
        m.load_cases["MODAL"].ritz_controls
    );
    assert_eq!(back.load_cases["MHIST"].modal_case, Some("MODAL".into()));
    assert_eq!(back.load_cases["MODAL"].modal_case, None);
    assert_eq!(back.frames["F"].auto_select, m.frames["F"].auto_select);
    assert_eq!(back.frames["G"].auto_select, m.frames["G"].auto_select);
    // A second pass writes the same text: nothing is lost or invented.
    assert_eq!(write_s2k(&back), text);
}

fn with_table(table: &str, row: &str) -> String {
    let text = write_s2k(&base());
    text.replace(
        "END TABLE DATA",
        &format!("TABLE:  \"{table}\"\n   {row}\n\nEND TABLE DATA"),
    )
}

#[test]
fn unreadable_extension_rows_are_import_errors() {
    for (table, row, needle) in [
        ("RUST MASS SOURCE", "Item=Options   Elements=Yes", "Elements and Masses"),
        ("RUST MASS SOURCE", "Item=Pattern   LoadPat=DEAD", "LoadPat and Multiplier"),
        ("RUST MASS SOURCE", "Item=Other", "Options or Pattern"),
        (
            "RUST CASE - MODAL RITZ CONTROLS",
            "Case=NOPE   LoadName=UX   MaxCycles=0   TargetPar=0.9",
            "unknown case",
        ),
        (
            "RUST CASE - MODAL RITZ CONTROLS",
            "Case=MODAL   LoadName=UX   MaxCycles=-1   TargetPar=0.9",
            "invalid MaxCycles",
        ),
        ("RUST CASE - MODAL RITZ CONTROLS", "Case=MODAL", "Case and LoadName"),
        (
            "RUST AUTO SEISMIC - ASCE 7-10",
            "LoadPat=EX   Dir=Z   EccRatio=0   PeriodType=User   UserT=1   R=3   Omega=3   Cd=3   I=1   Ss=1   S1=0.4   TL=8",
            "Dir must be X or Y",
        ),
        (
            "RUST AUTO SEISMIC - ASCE 7-10",
            "LoadPat=EX   Dir=X   EccRatio=0   PeriodType=Approximate   Ct=0.02   R=3   Omega=3   Cd=3   I=1   Ss=1   S1=0.4   TL=8",
            "Exponent",
        ),
        (
            "RUST AUTO SEISMIC - ASCE 7-10",
            "LoadPat=EX   Dir=X   EccRatio=0   PeriodType=Guess   R=3   Omega=3   Cd=3   I=1   Ss=1   S1=0.4   TL=8",
            "PeriodType",
        ),
        (
            "RUST AUTO SEISMIC - ASCE 7-10",
            "LoadPat=EX   Dir=X   EccRatio=0   PeriodType=User   UserT=1   Omega=3   Cd=3   I=1   Ss=1   S1=0.4   TL=8",
            "R is missing",
        ),
        ("RUST STEEL DESIGN PREFERENCES", "Other=1", "DCLimit"),
    ] {
        let error = parse_s2k(&with_table(table, row)).unwrap_err();
        assert!(
            matches!(error, FormatError::ParseS2k(_)) && error.to_string().contains(needle),
            "{table} / {row}: {error}"
        );
    }
}

#[test]
fn native_export_refuses_what_it_cannot_write() {
    let rejected = |m: &SapModel, field: &str| {
        let error = pack_model_to_native_sdb(m).unwrap_err();
        assert!(
            matches!(&error, FormatError::UnsupportedNativeExport { .. })
                && error.to_string().contains(field),
            "{field}: {error}"
        );
    };
    // The unchanged model is exportable, so each refusal below is caused by
    // the one field it sets.
    pack_model_to_native_sdb(&base()).unwrap();

    let mut m = base();
    m.mass_source.load_patterns.push(("DEAD".into(), 1.0));
    rejected(&m, "mass_source");

    let mut m = base();
    m.mass_source.additional_mass = false;
    rejected(&m, "mass_source");

    let mut m = base();
    quake(&mut m, "EX");
    m.auto_seismic.insert("EX".into(), AutoSeismicAsce710::default());
    rejected(&m, "auto_seismic.EX");

    let mut m = base();
    m.steel_design_prefs.dc_ratio_limit = 1.0;
    rejected(&m, "steel_design_prefs");

    let mut m = base();
    m.load_cases["MODAL"].ritz_controls.push(RitzLoadControl {
        load_name: "UX".into(),
        max_cycles: 2,
        target_dynamic_participation: 0.0,
    });
    rejected(&m, "load_cases.MODAL.ritz_controls");

    let mut m = base();
    m.load_cases["DEAD"].modal_case = Some("MODAL".into());
    rejected(&m, "load_cases.DEAD.modal_case");

    let mut m = base();
    m.frames["F"].auto_select = Some(AutoSelectState::default());
    rejected(&m, "frames.F.auto_select");
}
