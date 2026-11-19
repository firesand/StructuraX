//! D77 on vendor native models (opt-in, `STRUCTURAX_VENDOR_FIXTURES`; skips
//! silently without the corpus). The kind of each history entry is the native
//! record's `m_ltype`, not the name. The published results of the two models
//! are asserted by `wave5_vendor_history` (6-001, a load pattern) and
//! `modal_history_regressions` (1-022, a ground acceleration).
#![cfg(feature = "vendor-fixtures")]

use std::path::PathBuf;

use sap_model::HistoryLoadKind;

fn load(name: &str) -> Option<sap_model::SapModel> {
    let dir = PathBuf::from(std::env::var_os("STRUCTURAX_VENDOR_FIXTURES")?);
    let path = dir.join(name);
    path.is_file()
        .then(|| sap_format::load_sdb_file(&path).unwrap_or_else(|e| panic!("{name}: {e}")))
}

#[test]
fn example_6_001_histories_apply_a_load_pattern() {
    let Some(m) = load("example_6_001.sdb") else { return };
    for case in ["MHISTA", "MHISTB", "MHISTC"] {
        let lc = &m.load_cases[case];
        assert!(!lc.history_loads.is_empty(), "{case}");
        for i in 0..lc.history_loads.len() {
            assert_eq!(lc.history_kind(i), HistoryLoadKind::Pattern, "{case} entry {i}");
        }
    }
}

#[test]
fn example_1_022_histories_apply_a_ground_acceleration() {
    let Some(m) = load("example_1_022.sdb") else { return };
    let mut seen = 0;
    for (name, lc) in &m.load_cases {
        for (i, (_, load, _)) in lc.history_loads.iter().enumerate() {
            assert_eq!(load, "U1", "{name} entry {i}");
            assert_eq!(lc.history_kind(i), HistoryLoadKind::Acceleration, "{name} entry {i}");
            seen += 1;
        }
    }
    assert_eq!(seen, 4, "MHIST1, MHIST2, DHIST1, DHIST2");
}
