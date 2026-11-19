//! Wave 5, step 3b: the API and the database tables report a multi-valued
//! combination as a Max row followed by a Min row (Analysis Reference p. 351;
//! OAPI "Analysis Results Step Type": "For double-valued combinations, this is
//! either Max or Min"). The order Max, Min is a project choice.
use sap_api::{cSapModel, OAPI_ERR, OAPI_OK};
use sap_model::*;
use sap_units::Units;

fn pattern(m: &mut SapModel, name: &str) {
    m.load_patterns.insert(
        name.into(),
        LoadPattern {
            name: name.into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
}

fn combo(m: &mut SapModel, name: &str, kind: ComboType, cases: &[(&str, f64)]) {
    m.load_combos.insert(
        name.into(),
        LoadCombo {
            name: name.into(),
            combo_type: kind,
            cases: cases.iter().map(|(c, f)| (c.to_string(), *f)).collect(),
        },
    );
}

/// p. 353: one mass 2 on a spring 50 along U1 (w^2 = 25); a static load of 175
/// gives LL = 3.5, the constant spectrum 50 gives QUAKE = 50 / 25 = 2.0.
fn sdof_api() -> cSapModel {
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    m.add_point("C", 0.0, 0.0, 0.0);
    let c = &mut m.points["C"];
    c.mass = [2.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    c.restraint = [false, true, true, true, true, true];
    c.spring = [50.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    pattern(&mut m, "LL");
    m.points["C"].loads_force.push(PointLoadForce {
        load_pat: "LL".into(),
        csys: "GLOBAL".into(),
        force: [175.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    });
    m.load_cases.insert("LL".into(), LoadCase::linear_static("LL", "LL", 1.0));
    m.load_cases.insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 1));
    m.functions.insert(
        "SPEC".into(),
        DynamicFunction {
            name: "SPEC".into(),
            is_time_history: false,
            damping_ratio: 0.05,
            points: vec![(0.0, 50.0), (10.0, 50.0)],
        },
    );
    let mut rs = LoadCase::linear_static("QUAKE", "LL", 1.0);
    rs.case_type = LoadCaseType::ResponseSpectrum;
    rs.loads.clear();
    rs.initial_case = Some("MODAL".into());
    rs.spectrum_loads = vec![("SPEC".into(), "U1".into(), 1.0)];
    m.load_cases.insert("QUAKE".into(), rs);
    combo(&mut m, "COMB1", ComboType::LinearAdd, &[("LL", 1.0), ("QUAKE", 1.0)]);
    combo(&mut m, "COMB2", ComboType::Envelope, &[("LL", 1.0), ("QUAKE", 1.0)]);
    combo(&mut m, "TWICE", ComboType::LinearAdd, &[("LL", 2.0)]);
    let mut api = cSapModel::new(Units::KipInF);
    api.inner = m;
    assert_eq!(api.analyze().run_analysis(), OAPI_OK);
    api.results().setup_deselect_all_cases_and_combos_for_output();
    for case in ["LL", "QUAKE"] {
        api.results().setup_set_case_selected_for_output(case, true);
    }
    for name in ["COMB1", "COMB2", "TWICE"] {
        api.results().setup_set_combo_selected_for_output(name, true);
    }
    api
}

fn near(what: &str, got: f64, want: f64) {
    // 1e-9: the one-degree-of-freedom eigenvalue solution of the spectrum case.
    assert!((got - want).abs() <= 1e-9 * want.abs().max(1.0), "{what}: {got} vs {want}");
}

#[test]
fn joint_displ_returns_max_then_min_for_multi_valued_combinations() {
    let mut api = sdof_api();
    let rows = api.results().joint_displ_step_type("C");
    let got: Vec<(&str, &str)> = rows.iter().map(|(n, s, _)| (n.as_str(), *s)).collect();
    assert_eq!(
        got,
        [
            ("LL", ""),
            ("QUAKE", ""),
            ("COMB1", "Max"),
            ("COMB1", "Min"),
            ("COMB2", "Max"),
            ("COMB2", "Min"),
            ("TWICE", ""),
        ]
    );
    // COMB1 = LL + QUAKE: 5.5 / 1.5; COMB2 = envelope: 3.5 / -2.0 (p. 353).
    for (i, want) in [3.5, 2.0, 5.5, 1.5, 3.5, -2.0, 7.0].into_iter().enumerate() {
        near(&format!("row {i}"), rows[i].2[0], want);
    }
    // The plain query returns the same rows without the step type.
    let plain = api.results().joint_displ("C");
    assert_eq!(plain.len(), 7);
    assert_eq!(plain[2].0, "COMB1");
    assert_eq!(plain[3].0, "COMB1");
    assert_eq!(plain[3].1, rows[3].2);
}

#[test]
fn base_reaction_and_other_queries_follow_the_same_rows() {
    let mut api = sdof_api();
    let (code, rows) = api.results().base_react_step_type();
    assert_eq!(code, OAPI_OK);
    let steps: Vec<&str> = rows.iter().map(|r| r.1).collect();
    assert_eq!(steps, ["", "", "Max", "Min", "Max", "Min", ""]);
    // The Max row of the additive combination is the sum of the Max rows of its
    // contributors: base reaction of LL plus the positive value of QUAKE.
    let ll = rows[0].2[0];
    let quake = rows[1].2[0];
    near("COMB1 max FX", rows[2].2[0], ll + quake.abs());
    near("COMB1 min FX", rows[3].2[0], ll - quake.abs());
    // A joint without results gives no row; a single-valued result one row.
    assert!(api.results().joint_react_step_type("NOPE").is_empty());
}

#[test]
fn multi_valued_combo_option_one_is_the_only_one_implemented() {
    let mut api = sdof_api();
    assert_eq!(api.results().setup_set_option_multi_valued_combo(1), OAPI_OK);
    for option in [2, 3, 0, 4, -1] {
        assert_eq!(api.results().setup_set_option_multi_valued_combo(option), OAPI_ERR, "{option}");
    }
}

#[test]
fn joint_displacement_table_has_max_and_min_rows() {
    let mut api = sdof_api();
    let (code, fields, n, data) = api.database_tables().get_table_for_display_array_vec(
        "Joint Displacements",
        &["Joint", "OutputCase", "StepType", "U1"],
        "All",
    );
    assert_eq!(code, OAPI_OK);
    assert_eq!(fields.len(), 4);
    assert_eq!(n, 7);
    let row = |i: usize| (data[4 * i + 1].as_str(), data[4 * i + 2].as_str(), data[4 * i + 3].parse::<f64>().unwrap());
    assert_eq!(row(0).1, "Static");
    assert_eq!((row(2).0, row(2).1), ("COMB1", "Max"));
    assert_eq!((row(3).0, row(3).1), ("COMB1", "Min"));
    near("table COMB1 max", row(2).2, 5.5);
    near("table COMB1 min", row(3).2, 1.5);
    near("table COMB2 min", row(5).2, -2.0);
    assert_eq!(row(6).1, "Static");
}

/// A bar fixed at joint 1 with axial loads at joint 2: the axial force is the
/// load at every station (tension positive). Two static cases A (+10) and B
/// (-4) in an envelope: P = 10 (Max) and -4 (Min), a Max row and a Min row at
/// every station of the frame table.
#[test]
fn frame_table_has_a_max_and_a_min_row_at_every_station() {
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.unit_weight = 0.0;
    mat.unit_mass = 0.0;
    m.materials.insert("MAT".into(), mat);
    m.frame_sections.insert(
        "SEC".into(),
        FrameSection {
            name: "SEC".into(),
            material: "MAT".into(),
            prop_type: FramePropType::General,
            area: 10.0,
            i33: 100.0,
            i22: 100.0,
            torsional_j: 200.0,
            as2: 0.0,
            as3: 0.0,
            modifiers: [1.0; 8],
            ..FrameSection::default()
        },
    );
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 100.0, 0.0, 0.0);
    m.points["1"].restraint = [true; 6];
    m.add_frame("B", "1", "2", "SEC");
    for (name, f) in [("A", 10.0), ("B", -4.0)] {
        pattern(&mut m, name);
        m.points["2"].loads_force.push(PointLoadForce {
            load_pat: name.into(),
            csys: "GLOBAL".into(),
            force: [f, 0.0, 0.0, 0.0, 0.0, 0.0],
        });
        m.load_cases.insert(name.into(), LoadCase::linear_static(name, name, 1.0));
    }
    combo(&mut m, "ENV", ComboType::Envelope, &[("A", 1.0), ("B", 1.0)]);
    let mut api = cSapModel::new(Units::KipInF);
    api.inner = m;
    assert_eq!(api.analyze().run_analysis(), OAPI_OK);
    api.results().setup_deselect_all_cases_and_combos_for_output();
    api.results().setup_set_combo_selected_for_output("ENV", true);
    let (code, _, n, data) = api.database_tables().get_table_for_display_array_vec(
        "Element Forces - Frames",
        &["Frame", "Station", "OutputCase", "StepType", "P"],
        "All",
    );
    assert_eq!(code, OAPI_OK);
    let stations = api.inner.case_results["ENV"].frame_forces["B"].len();
    assert!(stations >= 2);
    assert_eq!(n as usize, 2 * stations);
    for i in 0..stations {
        let (max_row, min_row) = (&data[10 * i..10 * i + 5], &data[10 * i + 5..10 * i + 10]);
        assert_eq!(max_row[3], "Max");
        assert_eq!(min_row[3], "Min");
        assert_eq!(max_row[1], min_row[1], "the pair shares the station");
        near("P max", max_row[4].parse().unwrap(), 10.0);
        near("P min", min_row[4].parse().unwrap(), -4.0);
    }
    // The API query gives the Max list and then the Min list.
    let rows = api.results().frame_force_step_type("B");
    assert_eq!(rows.iter().map(|r| r.1).collect::<Vec<_>>(), ["Max", "Min"]);
    near("API P max", rows[0].2[0].p, 10.0);
    near("API P min", rows[1].2[0].p, -4.0);
}
