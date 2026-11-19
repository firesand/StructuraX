//! The text of the result tables of the Database Tables window.
//!
//! Every cell the window shows is made here, as a string, by pure functions:
//! the window only lays the strings out. Two things this file adds to the
//! tables:
//!
//! * D15: an Area, Solid or Link table of a combination that has no such
//!   results because one of its cases has none says which case and of what type
//!   (`shell_output::missing_results_message`), not only "no results recorded".
//! * Max/Min display: a combination that is multi-valued (Analysis Reference
//!   pp. 213, 351-352) has `CaseResults::combo_min`; its ordinary fields hold the
//!   maximum. Each object then has a `Max` row and a `Min` row ("Step Type", as
//!   in the vendor's database tables), and a derived quantity that the solver
//!   does not combine (the shell von Mises stress, the solid principal and von
//!   Mises stresses) shows "n/a" in both.

use sap_model::{CaseResults, SapModel};

use super::shell_output::{self, ResultKind};
use super::TableTab;

/// Shown where a quantity does not exist for a multi-valued result.
pub(super) const NOT_AVAILABLE: &str = "n/a";

/// Column header of the row type of a multi-valued result.
pub(super) const STEP_TYPE: &str = "Step Type";

/// What a table window tab shows.
#[derive(Clone, Debug, PartialEq, Default)]
pub(super) struct TableText {
    /// Egui id of the grid.
    pub grid: &'static str,
    pub headers: Vec<String>,
    pub rows: Vec<Vec<String>>,
    /// Shown in place of the grid when there is nothing to tabulate.
    pub empty: Option<String>,
    /// The first column is a number (monospace) and not a name.
    pub first_is_number: bool,
}

/// A table of `cols` value columns after the object column, laid out with a
/// `Step Type` column when `multi`.
fn headers(first: &str, multi: bool, cols: &[&str]) -> Vec<String> {
    let mut h = vec![first.to_string()];
    if multi {
        h.push(STEP_TYPE.to_string());
    }
    h.extend(cols.iter().map(|c| c.to_string()));
    h
}

/// Rows of one object: `values(r)` for the result (the Max row when the
/// result is multi-valued) and, if it is, the same for the Min result (or all
/// "n/a" when the Min result lacks the object). `prefix` are the cells before
/// the value cells (the object name, the station).
fn object_rows<T>(
    out: &mut Vec<Vec<String>>,
    multi: bool,
    prefix: &[String],
    max: &T,
    min: Option<&T>,
    width: usize,
    cells: &dyn Fn(&T) -> Vec<String>,
) {
    let mut row = |tag: Option<&str>, values: Vec<String>| {
        let mut r = prefix.to_vec();
        if let Some(t) = tag {
            r.push(t.to_string());
        }
        r.extend(values);
        out.push(r);
    };
    if !multi {
        row(None, cells(max));
        return;
    }
    row(Some("Max"), cells(max));
    row(
        Some("Min"),
        min.map_or_else(|| vec![NOT_AVAILABLE.to_string(); width], cells),
    );
}

fn fixed(v: f64, decimals: usize) -> String {
    format!("{v:.decimals$}")
}

fn six(v: &[f64; 6], decimals: usize) -> Vec<String> {
    v.iter().map(|x| fixed(*x, decimals)).collect()
}

/// The empty-table message of an object kind: the one that names the cases
/// when a combination lost its results (D15), else the table's own text.
fn empty_message(model: &SapModel, case: &str, kind: ResultKind, plain: &str) -> String {
    shell_output::missing_results_message(model, case, kind).unwrap_or_else(|| plain.to_string())
}

/// The tab `tab` for the result `case`; `None` for a tab that is not a result
/// table (the design tabs) and for a case without results.
pub(super) fn table_text(model: &SapModel, case: &str, tab: TableTab) -> Option<TableText> {
    let cr = model.case_results.get(case)?;
    let min = cr.combo_min.as_deref();
    let multi = min.is_some();
    let mut t = TableText::default();
    match tab {
        TableTab::JointDisplacements => {
            t.grid = "disp_tbl";
            t.headers = headers("Joint", multi, &["U1", "U2", "U3", "R1 (rad)", "R2 (rad)", "R3 (rad)"]);
            for (jid, u) in &cr.joint_displacements {
                let m = min.and_then(|m| m.joint_displacements.get(jid));
                object_rows(&mut t.rows, multi, std::slice::from_ref(jid), u, m, 6, &|u| six(u, 5));
            }
        }
        TableTab::JointReactions => {
            t.grid = "react_tbl";
            t.headers = headers("Joint", multi, &["F1", "F2", "F3", "M1", "M2", "M3"]);
            for (jid, r) in &cr.joint_reactions {
                let m = min.and_then(|m| m.joint_reactions.get(jid));
                object_rows(&mut t.rows, multi, std::slice::from_ref(jid), r, m, 6, &|r| six(r, 4));
            }
        }
        TableTab::BaseReactions => {
            t.grid = "base_react_tbl";
            t.headers = headers(
                "Case/Combo",
                multi,
                &["Global FX", "Global FY", "Global FZ", "Global MX", "Global MY", "Global MZ"],
            );
            object_rows(
                &mut t.rows,
                multi,
                &[case.to_string()],
                &cr.base_reactions,
                min.map(|m| &m.base_reactions),
                6,
                &|r| six(r, 4),
            );
        }
        TableTab::FrameForces => {
            if cr.frame_forces.is_empty() {
                t.empty = Some("No frame forces recorded for this case.".into());
                return Some(t);
            }
            t.grid = "frame_force_tbl";
            // Frame, Station, then the Step Type of a multi-valued result.
            let mut h = vec!["Frame".to_string(), "Station".to_string()];
            if multi {
                h.push(STEP_TYPE.to_string());
            }
            h.extend(["P", "V2", "V3", "T", "M2", "M3"].map(String::from));
            t.headers = h;
            for (fid, stations) in &cr.frame_forces {
                for (k, st) in stations.iter().enumerate() {
                    let m = min
                        .and_then(|m| m.frame_forces.get(fid))
                        .and_then(|s| s.get(k));
                    let prefix = [fid.clone(), fixed(st.station, 2)];
                    let mut rows = Vec::new();
                    object_rows(&mut rows, multi, &prefix, st, m, 6, &|s| {
                        [s.p, s.v2, s.v3, s.t, s.m2, s.m3].iter().map(|v| fixed(*v, 3)).collect()
                    });
                    t.rows.extend(rows);
                }
            }
        }
        TableTab::AreaResults => {
            if cr.area_results.is_empty() {
                t.empty = Some(empty_message(
                    model,
                    case,
                    ResultKind::Area,
                    "No area/shell results recorded for this case.",
                ));
                return Some(t);
            }
            t.grid = "area_res_tbl";
            t.headers = headers(
                "Area",
                multi,
                &["F11", "F22", "F12", "M11", "M22", "M12", "V13", "V23", "SVM Top"],
            );
            for (aid, ar) in &cr.area_results {
                let m = min.and_then(|m| m.area_results.get(aid));
                object_rows(&mut t.rows, multi, std::slice::from_ref(aid), ar, m, 9, &|a| {
                    let mut v: Vec<String> = [a.f11, a.f22, a.f12, a.m11, a.m22, a.m12, a.v13, a.v23]
                        .iter()
                        .map(|x| fixed(*x, 4))
                        .collect();
                    // The von Mises stress is not combined for a multi-valued result.
                    v.push(if multi { NOT_AVAILABLE.to_string() } else { fixed(a.svm_top, 4) });
                    v
                });
            }
        }
        TableTab::SolidStresses => {
            if cr.solid_results.is_empty() {
                t.empty = Some(empty_message(
                    model,
                    case,
                    ResultKind::Solid,
                    "No 3D solid stresses recorded for this case.",
                ));
                return Some(t);
            }
            t.grid = "solid_res_tbl";
            t.headers = headers("Solid", multi, &["S11", "S22", "S33", "S12", "S13", "S23", "Von Mises"]);
            for (sid, sr) in &cr.solid_results {
                let m = min.and_then(|m| m.solid_results.get(sid));
                object_rows(&mut t.rows, multi, std::slice::from_ref(sid), sr, m, 7, &|r| {
                    let s = &r.center_stress;
                    let mut v: Vec<String> = [s.s11, s.s22, s.s33, s.s12, s.s13, s.s23]
                        .iter()
                        .map(|x| fixed(*x, 4))
                        .collect();
                    v.push(if multi { NOT_AVAILABLE.to_string() } else { fixed(s.von_mises, 4) });
                    v
                });
            }
        }
        TableTab::LinkForces => {
            if cr.link_forces.is_empty() {
                t.empty = Some(empty_message(
                    model,
                    case,
                    ResultKind::Link,
                    "No link forces recorded for this case.",
                ));
                return Some(t);
            }
            t.grid = "link_force_tbl";
            t.headers = headers("Link", multi, &["P", "V2", "V3", "T", "M2", "M3"]);
            for (lid, lf) in &cr.link_forces {
                let m = min.and_then(|m| m.link_forces.get(lid));
                object_rows(&mut t.rows, multi, std::slice::from_ref(lid), lf, m, 6, &|f| six(f, 4));
            }
        }
        TableTab::ModalPeriods => {
            if cr.modes.is_empty() {
                t.empty = Some("Active case is not a Modal case (or has 0 solved modes).".into());
                return Some(t);
            }
            t.grid = "modal_tbl";
            t.first_is_number = true;
            t.headers = [
                "Mode",
                "Period (s)",
                "Freq (Hz)",
                "Eigenvalue",
                "Mass UX",
                "Mass UY",
                "Mass UZ",
            ]
            .map(String::from)
            .to_vec();
            for m in &cr.modes {
                t.rows.push(vec![
                    format!("{}", m.mode_num),
                    fixed(m.period_sec, 5),
                    fixed(m.freq_hz, 4),
                    fixed(m.eigenvalue, 4),
                    fixed(m.modal_mass_ratios[0], 4),
                    fixed(m.modal_mass_ratios[1], 4),
                    fixed(m.modal_mass_ratios[2], 4),
                ]);
            }
        }
        TableTab::BucklingFactors => {
            if cr.buckling_factors.is_empty() {
                t.empty = Some("Active case is not a Buckling case (or has 0 buckling modes).".into());
                return Some(t);
            }
            t.grid = "buckling_tbl";
            t.first_is_number = true;
            t.headers = vec!["Buckling Mode".into(), "Critical Factor λ".into()];
            for (idx, factor) in cr.buckling_factors.iter().enumerate() {
                t.rows.push(vec![format!("{}", idx + 1), fixed(*factor, 6)]);
            }
        }
        TableTab::SteelDesign | TableTab::ConcreteDesign => return None,
    }
    Some(t)
}

/// The two results of a diagram, Max first: the result itself and, for a
/// multi-valued combination, its minimum.
pub(super) fn max_min_results(cr: &CaseResults) -> Vec<(&'static str, &CaseResults)> {
    match cr.combo_min.as_deref() {
        Some(min) => vec![("Max", cr), ("Min", min)],
        None => vec![("", cr)],
    }
}
