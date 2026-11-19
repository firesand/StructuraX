//! D78/D79 through the API: the output selection, the lock and the database tables.
//!
//! Model (kN, m): D = 10 kN and L = 30 kN on a 100 kN/m spring, L not run. ULS =
//! 1.2 D + 1.6 L is unavailable (L has no results); SERV = 1.0 D is valid: joint U1 =
//! 10 / 100 = 0.1 m and base reaction FX = -10 kN (tolerance 1e-9).
use sap_api::{cSapModel, OAPI_ERR, OAPI_OK};

const HEALTHY: &str = r#"TABLE: "PROGRAM CONTROL"
 CurrUnits="kN, m, C"
TABLE: "JOINT COORDINATES"
 Joint=J XorR=0 Y=0 Z=0
TABLE: "JOINT RESTRAINT ASSIGNMENTS"
 Joint=J U2=Yes U3=Yes R1=Yes R2=Yes R3=Yes
TABLE: "JOINT SPRING ASSIGNMENTS 1 - UNCOUPLED"
 Joint=J U1=100
TABLE: "LOAD PATTERN DEFINITIONS"
 LoadPat=D DesignType=Dead SelfWtMult=0
 LoadPat=L DesignType=Live SelfWtMult=0
TABLE: "JOINT LOADS - FORCE"
 Joint=J LoadPat=D F1=10
 Joint=J LoadPat=L F1=30
TABLE: "LOAD CASE DEFINITIONS"
 Case=D Type=LinStatic RunCase=Yes
 Case=L Type=LinStatic RunCase=No
TABLE: "COMBINATION DEFINITIONS"
 ComboName=ULS ComboType="Linear Add" CaseName=D ScaleFactor=1.2
 ComboName=ULS ComboType="Linear Add" CaseName=L ScaleFactor=1.6
 ComboName=SERV ComboType="Linear Add" CaseName=D ScaleFactor=1
 ComboName=OK ComboType="Linear Add" CaseName=SERV ScaleFactor=2
END TABLE DATA
"#;

fn open(tag: &str) -> cSapModel {
    let path = std::env::temp_dir().join(format!("mb_combo_{tag}_{}.s2k", std::process::id()));
    std::fs::write(&path, HEALTHY).unwrap();
    let mut model = cSapModel::default();
    assert_eq!(model.file().open_file(path.to_str().unwrap()), OAPI_OK);
    let _ = std::fs::remove_file(&path);
    model
}

fn select_uls_and_serv(model: &mut cSapModel) {
    model.results().setup_deselect_all_cases_and_combos_for_output();
    assert_eq!(model.results().setup_set_combo_selected_for_output("ULS", true), OAPI_OK);
    assert_eq!(model.results().setup_set_combo_selected_for_output("SERV", true), OAPI_OK);
}

fn base_react_table(model: &mut cSapModel) -> (i32, Vec<String>, i32, Vec<String>) {
    model
        .database_tables()
        .get_table_for_display_array_vec("BASE REACTIONS", &["OutputCase", "GlobalFX"], "ALL")
}

#[test]
fn selection_with_an_unavailable_combination_is_a_failure_in_every_path() {
    let mut model = open("sel");
    select_uls_and_serv(&mut model);
    assert_eq!(model.analyze().run_analysis(), OAPI_OK);
    assert_eq!(
        model.analyze().unavailable_results().iter().map(|(n, _)| n.as_str()).collect::<Vec<_>>(),
        vec!["ULS"]
    );
    // The names it lists: ULS, with its reason.
    let sel = model.results().selected_unavailable();
    assert_eq!(sel.len(), 1);
    assert_eq!(sel[0].0, "ULS");
    assert!(sel[0].1.contains("'L'"), "{}", sel[0].1);
    // Getters with an error code.
    assert_eq!(model.results().base_react().0, OAPI_ERR);
    assert!(model.results().base_react().1.is_empty());
    assert_eq!(model.results().time_history_joint_displ("J").0, OAPI_ERR);
    // Getters that return rows only: none, never the rows of SERV alone.
    assert!(model.results().joint_displ("J").is_empty());
    // The database tables: the error code, no rows.
    for table in ["BASE REACTIONS", "JOINT DISPLACEMENTS", "JOINT REACTIONS"] {
        let (code, _fields, rows, data) = model
            .database_tables()
            .get_table_for_display_array_vec(table, &["OutputCase"], "ALL");
        assert_eq!(code, OAPI_ERR, "{table}");
        assert_eq!((rows, data.len()), (0, 0), "{table}");
    }
}

#[test]
fn positive_control_the_valid_sibling_alone_gives_its_rows() {
    let mut model = open("pos");
    assert_eq!(model.analyze().run_analysis(), OAPI_OK);
    model.results().setup_deselect_all_cases_and_combos_for_output();
    assert_eq!(model.results().setup_set_combo_selected_for_output("SERV", true), OAPI_OK);
    let (code, rows) = model.results().base_react();
    assert_eq!(code, OAPI_OK);
    assert_eq!(rows.len(), 1);
    assert_eq!(rows[0].0, "SERV");
    assert!((rows[0].1[0] + 10.0).abs() < 1e-9, "{}", rows[0].1[0]);
    let u = model.results().joint_displ("J");
    assert_eq!(u.len(), 1);
    assert!((u[0].1[0] - 0.1).abs() < 1e-9);
    let (code, fields, n, data) = base_react_table(&mut model);
    assert_eq!((code, n), (OAPI_OK, 1));
    assert_eq!(fields, vec!["OutputCase".to_string(), "GlobalFX".to_string()]);
    assert_eq!(data[0], "SERV");
    assert!((data[1].parse::<f64>().unwrap() + 10.0).abs() < 1e-3, "{}", data[1]);
}

#[test]
fn a_refused_selection_leaves_the_previous_selection_as_it_was() {
    let mut model = open("retry");
    assert_eq!(model.analyze().run_analysis(), OAPI_OK);
    model.results().setup_deselect_all_cases_and_combos_for_output();
    assert_eq!(model.results().setup_set_combo_selected_for_output("SERV", true), OAPI_OK);
    // Refused: ULS is unavailable.
    assert_eq!(model.results().setup_set_combo_selected_for_output("ULS", true), OAPI_ERR);
    assert_eq!(model.selected_combos, vec!["SERV".to_string()]);
    // A combination selected before the run and now refused: the retry must not drop it
    // from the selection (that turned the failure into a success with SERV only).
    let mut model = open("retry2");
    select_uls_and_serv(&mut model);
    assert_eq!(model.analyze().run_analysis(), OAPI_OK);
    assert_eq!(model.results().setup_set_combo_selected_for_output("ULS", true), OAPI_ERR);
    assert_eq!(model.selected_combos, vec!["ULS".to_string(), "SERV".to_string()]);
    assert_eq!(model.results().base_react().0, OAPI_ERR);
    assert!(model.results().joint_displ("J").is_empty());
    // Deselecting is allowed, and then SERV alone is complete.
    assert_eq!(model.results().setup_set_combo_selected_for_output("ULS", false), OAPI_OK);
    assert_eq!(model.results().base_react().0, OAPI_OK);
}

#[test]
fn delete_results_and_relock_do_not_bring_an_old_reason_back() {
    let mut model = open("lock");
    assert_eq!(model.analyze().run_analysis(), OAPI_OK);
    assert_eq!(model.analyze().unavailable_results().len(), 1);
    assert_eq!(model.analyze().delete_results(), OAPI_OK);
    assert!(model.inner.case_results.is_empty());
    assert!(model.inner.unavailable_results.is_empty(), "reasons go with the results");
    assert_eq!(model.set_model_is_locked(true), OAPI_OK);
    assert!(model.inner.case_results.is_empty());
    assert!(model.analyze().unavailable_results().is_empty());
    // A selection made now is not refused by a reason of a run that no longer exists.
    assert_eq!(model.results().setup_set_combo_selected_for_output("ULS", true), OAPI_OK);
    // Negative control: running again gives the reason again, once.
    assert_eq!(model.set_model_is_locked(false), OAPI_OK);
    assert_eq!(model.analyze().run_analysis(), OAPI_OK);
    assert_eq!(model.analyze().unavailable_results().len(), 1);
}

// ---- database table editing that would create a case/combination name (D79) ----------

fn apply(model: &mut cSapModel) -> (i32, i32, String) {
    let (mut fatal, mut err, mut warn, mut info) = (0, 0, 0, 0);
    let mut log = String::new();
    let code = model
        .database_tables()
        .apply_edited_tables(true, &mut fatal, &mut err, &mut warn, &mut info, &mut log);
    (code, fatal, log)
}

#[test]
fn a_table_edit_that_names_a_combination_like_a_case_is_refused_and_not_applied() {
    let mut model = open("tbl1");
    let combos = model.inner.load_combos.clone();
    let cases = model.inner.load_cases.clone();
    assert_eq!(
        model.database_tables().set_table_for_editing_array_str(
            "COMBINATION DEFINITIONS",
            &["ComboName", "ComboType", "CaseName", "ScaleFactor"],
            1,
            &["D", "Linear Add", "L", "1"],
        ),
        OAPI_OK
    );
    let (code, fatal, log) = apply(&mut model);
    assert_eq!(code, OAPI_ERR);
    assert_eq!(fatal, 1);
    assert!(
        log.contains("'D'") && log.contains("load case") && log.contains("load combination"),
        "{log}"
    );
    assert_eq!(model.inner.load_combos, combos);
    assert_eq!(model.inner.load_cases, cases);
}

#[test]
fn a_table_edit_that_names_a_case_like_a_combination_is_refused_and_not_applied() {
    let mut model = open("tbl2");
    let combos = model.inner.load_combos.clone();
    let cases = model.inner.load_cases.clone();
    assert_eq!(
        model.database_tables().set_table_for_editing_array_str(
            "LOAD CASE DEFINITIONS",
            &["Case", "Type", "Pattern"],
            1,
            &["SERV", "LinearStatic", "D"],
        ),
        OAPI_OK
    );
    let (code, fatal, log) = apply(&mut model);
    assert_eq!(code, OAPI_ERR);
    assert_eq!(fatal, 1);
    assert!(log.contains("'SERV'"), "{log}");
    assert_eq!(model.inner.load_cases, cases);
    assert_eq!(model.inner.load_combos, combos);
}

#[test]
fn positive_control_a_table_edit_with_a_new_name_is_applied() {
    let mut model = open("tbl3");
    assert_eq!(
        model.database_tables().set_table_for_editing_array_str(
            "COMBINATION DEFINITIONS",
            &["ComboName", "ComboType", "CaseName", "ScaleFactor"],
            1,
            &["NEWC", "Linear Add", "D", "2"],
        ),
        OAPI_OK
    );
    let (code, fatal, _) = apply(&mut model);
    assert_eq!((code, fatal), (OAPI_OK, 0));
    assert!(model.inner.load_combos.contains_key("NEWC"));
}
