//! Auto Select lists, design preferences, default combinations and design verification.
//!
//! The commands are plain functions that take the model and typed arguments
//! and return a status message or the reason for a refusal. Everything is
//! validated before the model changes; commands that change the model
//! definition refuse while the model is locked.

use eframe::egui;
use sap_design::{DefaultComboOptions, DesignMethod};
use sap_model::SapModel;
use std::collections::BTreeSet;

/// Window state. It is reset when another model is loaded.
pub(super) struct State {
    pub(super) open: bool,
    tab: usize,
    list_name: String,
    members: BTreeSet<String>,
    assign_list: String,
    dc_limit: f64,
    method: DesignMethod,
    rho: f64,
    sds: f64,
    replace: bool,
}

impl Default for State {
    fn default() -> Self {
        Self {
            open: false,
            tab: 0,
            list_name: "AUTO1".into(),
            members: BTreeSet::new(),
            assign_list: String::new(),
            dc_limit: 0.95,
            method: DesignMethod::Lrfd,
            rho: 1.0,
            sds: 0.0,
            replace: false,
        }
    }
}

fn unlocked(model: &SapModel) -> Result<(), String> {
    if model.is_locked {
        Err("The model is locked. Use Edit > Unlock Model before changing the definition.".into())
    } else {
        Ok(())
    }
}

/// Defines an Auto Select list from existing steel sections.
pub(super) fn define_list(
    model: &mut SapModel,
    name: &str,
    members: &BTreeSet<String>,
) -> Result<String, String> {
    unlocked(model)?;
    let members: Vec<String> = members.iter().cloned().collect();
    model.define_auto_select_list(name.trim(), &members)?;
    Ok(format!(
        "Defined Auto Select list {} with {} sections.",
        name.trim(),
        members.len()
    ))
}

/// Assigns the list to the selected frames.
pub(super) fn assign_list(model: &mut SapModel, list: &str) -> Result<String, String> {
    unlocked(model)?;
    let frames: Vec<String> = model
        .frames
        .values()
        .filter(|f| f.selected)
        .map(|f| f.name.clone())
        .collect();
    if frames.is_empty() {
        return Err("No frames are selected.".into());
    }
    model.assign_auto_select_list(list, &frames)?;
    Ok(format!(
        "Assigned Auto Select list {list} to {} frames.",
        frames.len()
    ))
}

/// Sets the D/C ratio limit (steel design manual table B-1: at most 1.0).
pub(super) fn set_limit(model: &mut SapModel, limit: f64) -> Result<String, String> {
    unlocked(model)?;
    model.set_dc_ratio_limit(limit)?;
    Ok(format!("D/C ratio limit set to {limit}."))
}

/// Adds the default steel design combinations.
pub(super) fn add_combos(
    model: &mut SapModel,
    method: DesignMethod,
    rho: f64,
    sds: f64,
    replace: bool,
) -> Result<String, String> {
    let options = DefaultComboOptions {
        method,
        rho,
        sds,
        replace,
    };
    let summary =
        sap_design::add_default_design_combos(model, &options).map_err(|e| e.to_string())?;
    let mut message = format!("Added {} design combinations.", summary.created.len());
    if !summary.removed.is_empty() {
        message.push_str(&format!(" Replaced {}.", summary.removed.len()));
    }
    if !summary.skipped.is_empty() {
        let names: Vec<&str> = summary.skipped.iter().map(|(n, _)| n.as_str()).collect();
        message.push_str(&format!(" Skipped cases: {}.", names.join(", ")));
    }
    Ok(message)
}

/// Runs the steel design on the analysis results.
pub(super) fn start_design(model: &mut SapModel) -> Result<String, String> {
    if !model.is_locked || model.case_results.is_empty() {
        return Err("Run the analysis first; the design uses its forces.".into());
    }
    sap_design::run_steel_design_per_member(model).map_err(|e| e.to_string())?;
    let changed = model
        .frames
        .values()
        .filter(|f| {
            f.auto_select
                .as_ref()
                .is_some_and(|s| s.analysis.as_deref().is_some_and(|a| a != s.design))
        })
        .count();
    Ok(format!(
        "Designed {} frames; {changed} Auto Select frames now differ from their analysis section{}.",
        model.steel_design_results.len(),
        super::not_designed_note(model.steel_design_errors.values())
    ))
}

fn select_only(model: &mut SapModel, frames: &[String]) {
    for frame in model.frames.values_mut() {
        frame.selected = frames.contains(&frame.name);
    }
}

/// Selects the frames whose analysis and design sections differ.
pub(super) fn verify_analysis_vs_design(model: &mut SapModel) -> Result<String, String> {
    let frames = sap_design::verify_analysis_vs_design(model).map_err(|e| e.to_string())?;
    select_only(model, &frames);
    Ok(format!(
        "{} frames have an analysis section different from the design section (selected).",
        frames.len()
    ))
}

/// Selects the frames that did not pass the design.
pub(super) fn verify_all_passed(model: &mut SapModel) -> Result<String, String> {
    let frames = sap_design::verify_all_members_passed(model).map_err(|e| e.to_string())?;
    select_only(model, &frames);
    Ok(format!(
        "{} frames did not pass (status not OK or ratio above the limit; selected).",
        frames.len()
    ))
}

/// Draws the window when it is open. Returns the outcome of a command the
/// user ran in this frame: a status message, or why it was refused.
pub(super) fn render(
    ctx: &egui::Context,
    state: &mut State,
    model: &mut SapModel,
) -> Option<Result<String, String>> {
    if !state.open {
        return None;
    }
    let mut open = true;
    let mut outcome = None;
    egui::Window::new("Steel Design Workflow")
        .open(&mut open)
        .default_width(520.0)
        .vscroll(true)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                for (i, label) in [
                    "Auto Select",
                    "Preferences",
                    "Combinations",
                    "Design and Verify",
                ]
                .iter()
                .enumerate()
                {
                    ui.selectable_value(&mut state.tab, i, *label);
                }
            });
            ui.separator();
            match state.tab {
                0 => {
                    let steel_sections: Vec<String> = model
                        .frame_sections
                        .values()
                        .filter(|s| !sap_model::auto_select::is_auto_list(s))
                        .filter(|s| {
                            model
                                .materials
                                .get(&s.material)
                                .is_some_and(|m| m.mat_type == sap_model::MatType::Steel)
                        })
                        .map(|s| s.name.clone())
                        .collect();
                    ui.label("Define an Auto Select list from existing steel sections:");
                    ui.horizontal(|ui| {
                        ui.label("List name");
                        ui.text_edit_singleline(&mut state.list_name);
                    });
                    for name in &steel_sections {
                        let mut on = state.members.contains(name);
                        if ui.checkbox(&mut on, name).changed() {
                            if on {
                                state.members.insert(name.clone());
                            } else {
                                state.members.remove(name);
                            }
                        }
                    }
                    if ui.button("Define Auto Select List").clicked() {
                        outcome = Some(define_list(model, &state.list_name, &state.members));
                    }
                    ui.separator();
                    ui.label("Assign a list to the selected frames:");
                    let lists: Vec<String> = model
                        .frame_sections
                        .values()
                        .filter(|s| sap_model::auto_select::is_auto_list(s))
                        .map(|s| s.name.clone())
                        .collect();
                    egui::ComboBox::from_id_salt("steel_auto_list")
                        .selected_text(state.assign_list.as_str())
                        .show_ui(ui, |ui| {
                            for l in &lists {
                                ui.selectable_value(&mut state.assign_list, l.clone(), l);
                            }
                        });
                    if ui.button("Assign to Selected Frames").clicked() {
                        outcome = Some(assign_list(model, &state.assign_list));
                    }
                }
                1 => {
                    ui.label("D/C ratio limit (dimensionless, at most 1.0; default 0.95):");
                    ui.add(
                        egui::DragValue::new(&mut state.dc_limit)
                            .speed(0.01)
                            .range(0.01..=1.0),
                    );
                    if ui.button("Set D/C Ratio Limit").clicked() {
                        outcome = Some(set_limit(model, state.dc_limit));
                    }
                }
                2 => {
                    ui.horizontal(|ui| {
                        ui.selectable_value(&mut state.method, DesignMethod::Lrfd, "LRFD");
                        ui.selectable_value(&mut state.method, DesignMethod::Asd, "ASD");
                    });
                    ui.horizontal(|ui| {
                        ui.label("Redundancy factor rho");
                        ui.add(egui::DragValue::new(&mut state.rho).speed(0.05));
                    });
                    ui.horizontal(|ui| {
                        ui.label("SDS (g, needed with a seismic case)");
                        ui.add(egui::DragValue::new(&mut state.sds).speed(0.05));
                    });
                    ui.checkbox(&mut state.replace, "Replace existing DSTL combinations");
                    if ui.button("Add Default Design Combinations").clicked() {
                        outcome = Some(add_combos(
                            model,
                            state.method,
                            state.rho,
                            state.sds,
                            state.replace,
                        ));
                    }
                }
                _ => {
                    ui.label(format!(
                        "Design code: {}. The design uses the current analysis results.",
                        model.steel_design_code
                    ));
                    if ui.button("Start Design").clicked() {
                        outcome = Some(start_design(model));
                    }
                    if ui.button("Verify Analysis vs Design Section").clicked() {
                        outcome = Some(verify_analysis_vs_design(model));
                    }
                    if ui.button("Verify All Members Passed").clicked() {
                        outcome = Some(verify_all_passed(model));
                    }
                }
            }
        });
    state.open = open;
    outcome
}

#[cfg(test)]
mod tests {
    use super::*;
    use eframe::egui::{Pos2, Rect, Vec2};
    use sap_model::{
        FrameSection, FrameStationResult, LoadCase, LoadPattern, LoadPatternType,
        SteelDesignOverwrite,
    };
    use sap_units::Units;

    fn w_section(name: &str, a: f64, ix: f64, zx: f64) -> FrameSection {
        FrameSection {
            name: name.into(),
            area: a,
            i33: ix,
            z33: zx,
            s33: zx / 1.12,
            ..FrameSection::default()
        }
    }

    fn model() -> SapModel {
        let mut m = SapModel::new(Units::KipInF);
        m.frame_sections.clear();
        for (n, a, ix, zx) in [
            ("S1", 6.0, 200.0, 30.0),
            ("S2", 8.0, 300.0, 44.0),
            ("S3", 10.0, 500.0, 66.0),
        ] {
            m.frame_sections.insert(n.into(), w_section(n, a, ix, zx));
        }
        m.add_point("A", 0.0, 0.0, 0.0);
        m.add_point("B", 200.0, 0.0, 0.0);
        m.add_frame("F1", "A", "B", "S1");
        m
    }

    fn set(names: &[&str]) -> BTreeSet<String> {
        names.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn define_assign_and_limit_commands_validate_before_changing() {
        let mut m = model();
        // Definition refusals leave the model unchanged.
        assert!(define_list(&mut m, "L", &set(&[])).is_err());
        assert!(define_list(&mut m, "L", &set(&["NOPE"])).is_err());
        assert!(!m.frame_sections.contains_key("L"));
        define_list(&mut m, "L", &set(&["S1", "S3", "S2"])).unwrap();
        assert_eq!(m.frame_sections["L"].auto_select_list, ["S1", "S2", "S3"]);

        // Nothing selected: refused.
        assert!(assign_list(&mut m, "L").unwrap_err().contains("No frames"));
        m.frames.get_mut("F1").unwrap().selected = true;
        assign_list(&mut m, "L").unwrap();
        assert_eq!(m.frames["F1"].section, "L");
        // Median by weight is S2.
        assert_eq!(m.frames["F1"].auto_select.as_ref().unwrap().design, "S2");
        assert!(assign_list(&mut m, "S1")
            .unwrap_err()
            .contains("not an Auto Select"));
        assert_eq!(m.frames["F1"].section, "L");

        assert!(set_limit(&mut m, 1.5).is_err());
        assert_eq!(m.steel_design_prefs.dc_ratio_limit, 0.95);
        set_limit(&mut m, 0.9).unwrap();
        assert_eq!(m.steel_design_prefs.dc_ratio_limit, 0.9);

        // A locked model refuses every definition command.
        m.is_locked = true;
        assert!(define_list(&mut m, "L2", &set(&["S1"]))
            .unwrap_err()
            .contains("locked"));
        assert!(assign_list(&mut m, "L").unwrap_err().contains("locked"));
        assert!(set_limit(&mut m, 0.8).unwrap_err().contains("locked"));
        assert_eq!(m.steel_design_prefs.dc_ratio_limit, 0.9);
        assert!(!m.frame_sections.contains_key("L2"));
    }

    #[test]
    fn combination_command_reports_and_refuses() {
        let mut m = model();
        m.load_patterns.insert(
            "DEAD".into(),
            LoadPattern {
                name: "DEAD".into(),
                pat_type: LoadPatternType::Dead,
                self_wt_multiplier: 0.0,
            },
        );
        m.load_cases.clear();
        m.load_cases
            .insert("DEAD".into(), LoadCase::linear_static("DEAD", "DEAD", 1.0));
        let msg = add_combos(&mut m, DesignMethod::Lrfd, 1.0, 0.0, false).unwrap();
        assert!(msg.contains("Added 2"), "{msg}");
        assert!(add_combos(&mut m, DesignMethod::Lrfd, 1.0, 0.0, false).is_err());
        assert!(add_combos(&mut m, DesignMethod::Lrfd, 1.0, 0.0, true).is_ok());
        m.is_locked = true;
        assert!(add_combos(&mut m, DesignMethod::Asd, 1.0, 0.0, true).is_err());
    }

    /// A locked model with forces for frame F1 (list of S1, S2, S3) so the
    /// design command and both verify commands can run.
    fn designed_model(moment: f64) -> SapModel {
        let mut m = model();
        define_list(&mut m, "L", &set(&["S1", "S2", "S3"])).unwrap();
        m.frames.get_mut("F1").unwrap().selected = true;
        assign_list(&mut m, "L").unwrap();
        m.frames.get_mut("F1").unwrap().steel_overwrite = Some(SteelDesignOverwrite {
            l_ratio_major: Some(0.01),
            l_ratio_minor: Some(0.01),
            l_ratio_ltb: Some(0.01),
            k_major: Some(1.0),
            k_minor: Some(1.0),
            cb: Some(1.0),
            is_sway: false,
        });
        let mut cr = sap_model::CaseResults {
            case_name: "DEAD".into(),
            ..Default::default()
        };
        cr.frame_forces.insert(
            "F1".into(),
            vec![FrameStationResult {
                station: 100.0,
                rel_dist: 0.5,
                m3: moment,
                ..Default::default()
            }],
        );
        m.case_results.insert("DEAD".into(), cr);
        m.is_locked = true;
        m
    }

    #[test]
    fn design_and_verify_commands_select_the_listed_frames() {
        // Phi Mn = 0.9 * 50 * Z: S1 1350, S2 1980, S3 2970 kip-in. Mu = 1500
        // gives S1 1.11, S2 0.76: S2 is the lightest within 0.95 and is the
        // median, so analysis and design agree.
        let mut m = designed_model(1500.0);
        assert!(start_design(&mut m).unwrap().contains("Designed 1"));
        assert_eq!(m.steel_design_results["F1"].section_name, "S2");
        assert!(verify_analysis_vs_design(&mut m)
            .unwrap()
            .starts_with("0 frames"));
        assert!(!m.frames["F1"].selected);
        assert!(verify_all_passed(&mut m).unwrap().starts_with("0 frames"));

        // Mu = 600: S1 passes (0.44); design S1, the analysis section is S2.
        let mut m = designed_model(600.0);
        start_design(&mut m).unwrap();
        assert_eq!(m.steel_design_results["F1"].section_name, "S1");
        assert!(verify_analysis_vs_design(&mut m)
            .unwrap()
            .starts_with("1 frames"));
        assert!(m.frames["F1"].selected);

        // Mu = 5000: nothing passes; the heaviest is flagged.
        let mut m = designed_model(5000.0);
        start_design(&mut m).unwrap();
        assert_eq!(m.steel_design_results["F1"].section_name, "S3");
        assert!(verify_all_passed(&mut m).unwrap().starts_with("1 frames"));
        assert!(m.frames["F1"].selected);

        // Without analysis results or while unlocked the commands refuse.
        let mut m = model();
        assert!(start_design(&mut m).is_err());
        assert!(verify_analysis_vs_design(&mut m).is_err());
        assert!(verify_all_passed(&mut m).is_err());
    }

    #[test]
    fn window_renders_every_tab_and_a_button_click_runs_a_command() {
        let mut state = State {
            open: true,
            ..State::default()
        };
        let mut m = designed_model(600.0);
        m.is_locked = false;
        let ctx = egui::Context::default();
        let input = |time: f64| egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1000., 900.))),
            time: Some(time),
            ..Default::default()
        };
        let mut time = 0.0;
        for tab in 0..4 {
            state.tab = tab;
            time += 1.0;
            let out = ctx.run(input(time), |ctx| {
                let _ = render(ctx, &mut state, &mut m);
            });
            assert!(!out.shapes.is_empty(), "tab {tab} drew nothing");
        }
        // Click "Set D/C Ratio Limit" on the Preferences tab.
        state.tab = 1;
        state.dc_limit = 0.9;
        time += 1.0;
        let _ = ctx.run(input(time), |ctx| {
            let _ = render(ctx, &mut state, &mut m);
        });
        time += 1.0;
        let out = ctx.run(input(time), |ctx| {
            let _ = render(ctx, &mut state, &mut m);
        });
        let pos = out
            .shapes
            .iter()
            .find_map(|shape| match &shape.shape {
                egui::epaint::Shape::Text(text)
                    if text.galley.job.text == "Set D/C Ratio Limit" =>
                {
                    Some(text.pos + text.galley.size() * 0.5)
                }
                _ => None,
            })
            .expect("button visible");
        let mut down = input(time + 1.0);
        down.events = vec![
            egui::Event::PointerMoved(pos),
            egui::Event::PointerButton {
                pos,
                button: egui::PointerButton::Primary,
                pressed: true,
                modifiers: Default::default(),
            },
        ];
        let _ = ctx.run(down, |ctx| {
            let _ = render(ctx, &mut state, &mut m);
        });
        let mut up = input(time + 1.1);
        up.events = vec![egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed: false,
            modifiers: Default::default(),
        }];
        let mut outcome = None;
        let _ = ctx.run(up, |ctx| {
            outcome = render(ctx, &mut state, &mut m);
        });
        assert!(matches!(outcome, Some(Ok(_))), "{outcome:?}");
        assert_eq!(m.steel_design_prefs.dc_ratio_limit, 0.9);
    }

    mod solver_workflow {
        //! Steel design workflow through the real solver: a simply supported beam
        //! assigned an Auto Select list is analyzed with the median section, designed
        //! (the lightest section within the D/C limit is chosen), verified and
        //! analyzed again with the design section.
        //!
        //! Sources: CSI Steel Frame Design Manual AISC 360-10 section 2.1 pp. 2-1/2-2
        //! (Auto Select), Appendix E (analysis versus design sections), table B-1
        //! (D/C ratio limit 0.95). Closed forms: Mu = w L^2 / 8, phi Mn = 0.9 Fy Zx for
        //! a compact, fully braced W shape, midspan deflection 5 w L^4 / (384 E I).
        //! Units kip, in, ksi. The test lives here because it needs both the solver
        //! and the design crate.
        use sap_design::{
            try_run_steel_design, verify_all_members_passed, verify_analysis_vs_design,
        };
        #[allow(unused_imports)]
        use sap_model::*;
        use sap_solver::analyze_model;
        use sap_units::Units;

        const FY: f64 = 50.0;
        const E: f64 = 29_000.0;
        const SPAN: f64 = 240.0;

        // name, d, bf, tf, tw, A, Ix, Zx, Sx, Iy, Zy, Sy, ry, J (AISC shapes values, 3 digits)
        #[allow(clippy::type_complexity)]
        const SHAPES: [(
            &str,
            f64,
            f64,
            f64,
            f64,
            f64,
            f64,
            f64,
            f64,
            f64,
            f64,
            f64,
            f64,
            f64,
        ); 4] = [
            (
                "W14X22", 13.7, 5.00, 0.335, 0.230, 6.49, 199.0, 33.2, 29.0, 7.00, 3.87, 2.80,
                1.04, 0.208,
            ),
            (
                "W16X26", 15.7, 5.50, 0.345, 0.250, 7.68, 301.0, 44.2, 38.4, 9.59, 5.48, 3.49,
                1.12, 0.262,
            ),
            (
                "W16X31", 15.9, 5.53, 0.440, 0.275, 9.13, 375.0, 54.0, 47.2, 12.4, 7.03, 4.49,
                1.17, 0.461,
            ),
            (
                "W18X35", 17.7, 6.00, 0.425, 0.300, 10.3, 510.0, 66.5, 57.6, 15.3, 8.06, 5.12,
                1.22, 0.506,
            ),
        ];

        fn w_section(i: usize) -> FrameSection {
            let (name, d, bf, tf, tw, a, ix, zx, sx, iy, zy, sy, ry, j) = SHAPES[i];
            FrameSection {
                name: name.into(),
                material: "A992Fy50".into(),
                prop_type: FramePropType::I,
                t3: d,
                t2: bf,
                tf,
                tw,
                t2b: bf,
                tfb: tf,
                area: a,
                i33: ix,
                i22: iy,
                z33: zx,
                s33: sx,
                z22: zy,
                s22: sy,
                r33: (ix / a).sqrt(),
                r22: ry,
                torsional_j: j,
                as2: 0.0, // no shear deformation, so the deflection is the flexural closed form
                as3: 0.0,
                ..FrameSection::default()
            }
        }

        fn model(w: f64) -> SapModel {
            let mut m = SapModel::new(Units::KipInF);
            m.load_cases.clear();
            m.load_patterns.clear();
            m.load_patterns.insert(
                "DEAD".into(),
                LoadPattern {
                    name: "DEAD".into(),
                    pat_type: LoadPatternType::Dead,
                    self_wt_multiplier: 0.0,
                },
            );
            m.load_cases
                .insert("DEAD".into(), LoadCase::linear_static("DEAD", "DEAD", 1.0));
            let names: Vec<String> = [2, 0, 3, 1]
                .iter()
                .map(|&i| {
                    let s = w_section(i);
                    let n = s.name.clone();
                    m.frame_sections.insert(n.clone(), s);
                    n
                })
                .collect();
            m.define_auto_select_list("LIST", &names).unwrap();
            m.add_point("A", 0.0, 0.0, 0.0);
            m.add_point("B", SPAN, 0.0, 0.0);
            m.points.get_mut("A").unwrap().restraint = [true, true, true, true, false, false];
            m.points.get_mut("B").unwrap().restraint = [false, true, true, false, false, false];
            m.add_frame("BM", "A", "B", "LIST");
            m.assign_auto_select_list("LIST", &["BM".to_string()])
                .unwrap();
            let f = m.frames.get_mut("BM").unwrap();
            f.min_stations = 9;
            f.loads_dist.push(FrameLoadDistributed {
                load_pat: "DEAD".into(),
                my_type: 1,
                dir: 10,
                csys: "GLOBAL".into(),
                rel_dist1: 0.0,
                rel_dist2: 1.0,
                val1: w,
                val2: w,
            });
            // Fully braced: unbraced lengths 1 % of the span, far below Lp.
            f.steel_overwrite = Some(SteelDesignOverwrite {
                l_ratio_major: Some(0.01),
                l_ratio_minor: Some(0.01),
                l_ratio_ltb: Some(0.01),
                k_major: Some(1.0),
                k_minor: Some(1.0),
                cb: Some(1.0),
                is_sway: false,
            });
            m
        }

        fn midspan(m: &SapModel) -> FrameStationResult {
            m.case_results["DEAD"].frame_forces["BM"]
                .iter()
                .find(|s| (s.rel_dist - 0.5).abs() < 1e-9)
                .expect("a station at midspan")
                .clone()
        }

        fn deflection(w: f64, i: usize) -> f64 {
            5.0 * w * SPAN.powi(4) / (384.0 * E * SHAPES[i].6)
        }

        fn relative(got: f64, want: f64) -> f64 {
            (got - want).abs() / want.abs()
        }

        #[test]
        fn auto_select_analyze_design_verify_reanalyze() {
            // Mu = w L^2 / 8 = 1950 kip-in.
            let mu = 1950.0;
            let w = 8.0 * mu / (SPAN * SPAN);
            let mut m = model(w);

            // Without the list substitution the placeholder properties would be used;
            // the first analysis uses the median W16X26 (manual p. 2-2).
            analyze_model(&mut m).unwrap();
            let state = m.frames["BM"].auto_select.clone().unwrap();
            assert_eq!(state.analysis.as_deref(), Some("W16X26"));
            assert_eq!(
                m.frames["BM"].section, "LIST",
                "the model keeps the list assignment"
            );
            let mid = midspan(&m);
            assert!(relative(mid.m3.abs(), mu) < 1e-9, "Mu {} vs {mu}", mid.m3);
            let d_median = mid.local_disp[1].abs().max(mid.local_disp[2].abs());
            assert!(
                relative(d_median, deflection(w, 1)) < 1e-6,
                "{d_median} vs {}",
                deflection(w, 1)
            );

            // Design: lightest section with Mu / (0.9 Fy Zx) <= 0.95 from the typed list.
            try_run_steel_design(&mut m).unwrap();
            let mut order: Vec<usize> = (0..4).collect();
            order.sort_by(|&a, &b| SHAPES[a].5.total_cmp(&SHAPES[b].5));
            let expected = order
                .iter()
                .copied()
                .find(|&i| mu / (0.9 * FY * SHAPES[i].7) <= 0.95)
                .unwrap();
            assert_eq!(SHAPES[expected].0, "W16X31");
            let r = &m.steel_design_results["BM"];
            assert_eq!(r.section_name, "W16X31");
            assert!(relative(r.total_ratio, mu / (0.9 * FY * SHAPES[expected].7)) < 1e-9);
            // Shear does not govern (even at phi_v = 0.9).
            let vu = w * SPAN / 2.0;
            assert!(
                vu / (0.9 * 0.6 * FY * SHAPES[expected].1 * SHAPES[expected].4)
                    < 0.5 * r.total_ratio
            );

            // Analysis section W16X26, design section W16X31: verification reports the frame.
            assert!(m.is_locked);
            assert_eq!(verify_analysis_vs_design(&m).unwrap(), ["BM"]);
            assert!(verify_all_members_passed(&m).unwrap().is_empty());

            // Unlock, analyze again: the analysis uses the design section.
            m.is_locked = false;
            analyze_model(&mut m).unwrap();
            assert_eq!(
                m.frames["BM"]
                    .auto_select
                    .as_ref()
                    .unwrap()
                    .analysis
                    .as_deref(),
                Some("W16X31")
            );
            assert!(verify_analysis_vs_design(&m).unwrap().is_empty());
            let mid = midspan(&m);
            let d_design = mid.local_disp[1].abs().max(mid.local_disp[2].abs());
            assert!(
                relative(d_design, deflection(w, expected)) < 1e-6,
                "{d_design} vs {}",
                deflection(w, expected)
            );
            // The heavier section is stiffer.
            assert!(d_design < d_median);

            // Design again with the new forces: the choice is stable.
            try_run_steel_design(&mut m).unwrap();
            assert_eq!(m.steel_design_results["BM"].section_name, "W16X31");
            assert!(verify_analysis_vs_design(&m).unwrap().is_empty());
            assert!(verify_all_members_passed(&m).unwrap().is_empty());
        }

        #[test]
        fn analysis_refuses_an_unusable_list_instead_of_using_placeholder_properties() {
            let mut m = model(0.27);
            m.frame_sections
                .get_mut("LIST")
                .unwrap()
                .auto_select_list
                .clear();
            let err = analyze_model(&mut m).unwrap_err().to_string();
            assert!(err.contains("empty"), "{err}");
            assert!(m.case_results.is_empty());

            let mut m = model(0.27);
            m.frame_sections
                .get_mut("LIST")
                .unwrap()
                .auto_select_list
                .push("GONE".into());
            let err = analyze_model(&mut m).unwrap_err().to_string();
            assert!(err.contains("GONE"), "{err}");
        }
    }
}
