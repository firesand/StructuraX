//! Static pushover curve window, and the hinge markers and legend of the
//! deformed shape.
//!
//! Source: CSI Analysis Reference, "Static Pushover Analysis" and "Frame
//! Hinge Properties - Analysis Results". The window only reads results. The
//! manual lists the hinge display classes in the order of
//! [`HingeDisplayClass::ALL`] but does not give their colours; the colours
//! below are our own choice.

use eframe::egui;
use sap_api::hinges::{pushover_curve, PushoverPoint};
use sap_model::hinge::HingeDisplayClass;
use sap_model::{LoadCaseType, SapModel};

/// Window state. It is reset when another model is loaded.
#[derive(Default)]
pub(super) struct State {
    pub(super) open: bool,
    /// Case shown; empty or unknown selects the first nonlinear static case
    /// with saved states.
    pub(super) case: String,
}

/// Our own colour of a hinge display class (the manual does not give one).
pub(super) fn class_color(class: HingeDisplayClass) -> egui::Color32 {
    match class {
        HingeDisplayClass::BToIO => egui::Color32::from_rgb(70, 140, 255),
        HingeDisplayClass::IOToLS => egui::Color32::from_rgb(40, 210, 220),
        HingeDisplayClass::LSToCP => egui::Color32::from_rgb(250, 225, 60),
        HingeDisplayClass::CPToC => egui::Color32::from_rgb(255, 150, 40),
        HingeDisplayClass::CToD => egui::Color32::from_rgb(240, 60, 60),
        HingeDisplayClass::DToE => egui::Color32::from_rgb(210, 70, 230),
        HingeDisplayClass::BeyondE => egui::Color32::from_rgb(245, 245, 245),
    }
}

/// The yielded hinges of a case at a saved state (`None`: the final state) as
/// (frame, relative distance from End I, display class). Hinges that have not
/// yielded are left out; an unknown case or a case without hinge results gives
/// an empty list.
pub(super) fn hinge_markers(
    model: &SapModel,
    case: &str,
    step: Option<usize>,
) -> Vec<(String, f64, HingeDisplayClass)> {
    let Some(results) = model.case_results.get(case) else {
        return Vec::new();
    };
    let hinges = match step {
        Some(i) if !results.static_steps.is_empty() => {
            &results.static_steps[i.min(results.static_steps.len() - 1)].hinges
        }
        _ => &results.hinge_results,
    };
    hinges
        .iter()
        .filter_map(|h| Some((h.frame.clone(), h.rel_dist, h.display_class()?)))
        .collect()
}

/// Draws the seven-entry legend at the lower left of `rect`.
pub(super) fn draw_legend(painter: &egui::Painter, rect: egui::Rect) {
    let row = 15.0;
    let n = HingeDisplayClass::ALL.len() as f32;
    let height = 20.0 + row * n;
    let top = (rect.bottom() - height - 8.0).max(rect.top());
    let box_rect =
        egui::Rect::from_min_size(egui::pos2(rect.left() + 8.0, top), egui::vec2(96.0, height));
    painter.rect_filled(
        box_rect,
        4.0,
        egui::Color32::from_rgba_unmultiplied(12, 16, 24, 210),
    );
    painter.text(
        box_rect.left_top() + egui::vec2(6.0, 4.0),
        egui::Align2::LEFT_TOP,
        "Hinge state",
        egui::FontId::proportional(10.0),
        egui::Color32::from_rgb(210, 225, 245),
    );
    for (i, class) in HingeDisplayClass::ALL.iter().enumerate() {
        let y = box_rect.top() + 24.0 + row * i as f32;
        painter.circle_filled(
            egui::pos2(box_rect.left() + 12.0, y),
            4.0,
            class_color(*class),
        );
        painter.text(
            egui::pos2(box_rect.left() + 22.0, y),
            egui::Align2::LEFT_CENTER,
            class.label(),
            egui::FontId::proportional(10.5),
            egui::Color32::from_rgb(200, 215, 235),
        );
    }
}

/// Nonlinear static cases with at least one saved state, in model order.
fn cases_with_states(model: &SapModel) -> Vec<String> {
    model
        .load_cases
        .iter()
        .filter(|(name, lc)| {
            lc.case_type == LoadCaseType::NonlinearStatic
                && model
                    .case_results
                    .get(*name)
                    .is_some_and(|r| !r.static_steps.is_empty())
        })
        .map(|(name, _)| name.clone())
        .collect()
}

/// The (displacement, base shear) points of a curve.
pub(super) fn curve_xy(points: &[PushoverPoint]) -> Vec<(f64, f64)> {
    points
        .iter()
        .map(|p| (p.displacement, p.base_shear))
        .collect()
}

/// Tick values of an axis from `lo` to `hi`: multiples of 1, 2 or 5 times a
/// power of ten, about `target` of them, all inside the range. Each is
/// `k * step` for an integer `k`, so that no sum of steps carries roundoff.
pub(super) fn nice_ticks(lo: f64, hi: f64, target: usize) -> Vec<f64> {
    if !(lo.is_finite() && hi.is_finite() && hi > lo && target > 0) {
        return Vec::new();
    }
    let raw = (hi - lo) / target as f64;
    let power = 10f64.powf(raw.log10().floor());
    let step = [1.0, 2.0, 5.0, 10.0]
        .iter()
        .map(|m| m * power)
        .find(|s| *s >= raw)
        .unwrap_or(10.0 * power);
    let first = (lo / step).ceil() as i64;
    let last = (hi / step).floor() as i64;
    (first..=last)
        .map(|k| k as f64 * step)
        .map(|v| if v == 0.0 { 0.0 } else { v })
        .collect()
}

/// A tick value as the axis prints it: four significant digits.
pub(super) fn tick_text(v: f64) -> String {
    super::view::significant_text(v, 4)
}

/// The plot without a title.
#[cfg(test)]
pub(super) fn draw_plot_untitled(ui: &mut egui::Ui, xy: &[(f64, f64)]) {
    draw_plot(ui, xy, None);
}

/// The plot with a title at its top right (G16: "Not finished" for a case that stopped
/// before its target).
pub(super) fn draw_plot(ui: &mut egui::Ui, xy: &[(f64, f64)], title: Option<&str>) {
    let size = egui::vec2(ui.available_width().clamp(300.0, 720.0), 220.0);
    let (rect, _) = ui.allocate_exact_size(size, egui::Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(16, 20, 28));
    if let Some(title) = title {
        painter.text(
            egui::pos2(rect.right() - 6.0, rect.top() + 4.0),
            egui::Align2::RIGHT_TOP,
            title,
            egui::FontId::monospace(11.0),
            egui::Color32::from_rgb(255, 170, 90),
        );
    }
    let finite: Vec<(f64, f64)> = xy
        .iter()
        .copied()
        .filter(|(x, y)| x.is_finite() && y.is_finite())
        .collect();
    if finite.len() < 2 {
        return;
    }
    let (mut x0, mut x1, mut y0, mut y1) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
    for (x, y) in &finite {
        x0 = x0.min(*x);
        x1 = x1.max(*x);
        y0 = y0.min(*y);
        y1 = y1.max(*y);
    }
    if x1 - x0 <= 0.0 || y1 - y0 <= 0.0 {
        return;
    }
    // Room for the tick labels on the left and at the bottom, and for the
    // axis names at the top and the bottom.
    let inner = egui::Rect::from_min_max(
        rect.min + egui::vec2(66.0, 24.0),
        rect.max - egui::vec2(18.0, 38.0),
    );
    let to_screen = |(x, y): (f64, f64)| {
        egui::pos2(
            inner.left() + ((x - x0) / (x1 - x0)) as f32 * inner.width(),
            inner.bottom() - ((y - y0) / (y1 - y0)) as f32 * inner.height(),
        )
    };
    let text_color = egui::Color32::from_gray(200);
    let font = egui::FontId::monospace(11.0);
    let grid = egui::Stroke::new(1.0_f32, egui::Color32::from_gray(40));
    for v in nice_ticks(x0, x1, 5) {
        let x = to_screen((v, 0.0)).x;
        painter.line_segment([egui::pos2(x, inner.top()), egui::pos2(x, inner.bottom())], grid);
        painter.text(
            egui::pos2(x, inner.bottom() + 3.0),
            egui::Align2::CENTER_TOP,
            tick_text(v),
            font.clone(),
            text_color,
        );
    }
    for v in nice_ticks(y0, y1, 5) {
        let y = to_screen((0.0, v)).y;
        painter.line_segment([egui::pos2(inner.left(), y), egui::pos2(inner.right(), y)], grid);
        painter.text(
            egui::pos2(inner.left() - 4.0, y),
            egui::Align2::RIGHT_CENTER,
            tick_text(v),
            font.clone(),
            text_color,
        );
    }
    // The axes through zero.
    let axis = egui::Stroke::new(1.0_f32, egui::Color32::from_gray(110));
    let origin = to_screen((0.0, 0.0));
    painter.line_segment(
        [egui::pos2(inner.left(), origin.y), egui::pos2(inner.right(), origin.y)],
        axis,
    );
    painter.line_segment(
        [egui::pos2(origin.x, inner.top()), egui::pos2(origin.x, inner.bottom())],
        axis,
    );
    let line: Vec<egui::Pos2> = finite.iter().copied().map(to_screen).collect();
    painter.add(egui::Shape::line(
        line.clone(),
        egui::Stroke::new(1.5_f32, egui::Color32::from_rgb(110, 200, 255)),
    ));
    for p in line {
        painter.circle_filled(p, 2.5, egui::Color32::from_rgb(255, 230, 120));
    }
    // The axis names, inside the plot rectangle whatever the signs of the data.
    painter.text(
        egui::pos2(inner.center().x, rect.bottom() - 4.0),
        egui::Align2::CENTER_BOTTOM,
        "monitored displacement (model length units)",
        font.clone(),
        text_color,
    );
    painter.text(
        egui::pos2(rect.left() + 6.0, rect.top() + 4.0),
        egui::Align2::LEFT_TOP,
        "base shear (model force units)",
        font,
        text_color,
    );
}

pub(super) fn render(
    ctx: &egui::Context,
    state: &mut State,
    model: &mut SapModel,
) -> Option<Result<String, String>> {
    if !state.open {
        return None;
    }
    let mut open = true;
    egui::Window::new("Static Pushover Curve")
        .open(&mut open)
        .default_width(620.0)
        .vscroll(true)
        .show(ctx, |ui| {
            let cases = cases_with_states(model);
            if !cases.contains(&state.case) {
                state.case = cases.first().cloned().unwrap_or_default();
            }
            ui.horizontal(|ui| {
                ui.label("Case:");
                egui::ComboBox::from_id_salt("pushover_case")
                    .selected_text(state.case.as_str())
                    .show_ui(ui, |ui| {
                        for name in &cases {
                            ui.selectable_value(&mut state.case, name.clone(), name);
                        }
                    });
            });
            if cases.is_empty() {
                ui.label("No nonlinear static results. Run the analysis with a nonlinear static case that saves multiple states first.");
                // G16: a case that could not start has no results; say why.
                for (name, reason) in sap_model::results::could_not_start_cases(model) {
                    ui.colored_label(
                        egui::Color32::from_rgb(255, 170, 90),
                        format!("Case '{name}': {reason}"),
                    );
                }
                return;
            }
            // G16: the curve of a case that stopped before its target is the part it reached;
            // the reason comes first, also when that part is too short to draw a curve.
            let unfinished = sap_api::hinges::pushover_not_finished(model, &state.case);
            if let Some((reason, steps)) = &unfinished {
                ui.colored_label(
                    egui::Color32::from_rgb(255, 170, 90),
                    format!("Not finished: {reason} ({steps} saved steps kept)"),
                );
            }
            let points = match pushover_curve(model, &state.case) {
                Ok(points) => points,
                Err(error) => {
                    ui.colored_label(egui::Color32::from_rgb(255, 120, 120), error);
                    return;
                }
            };
            ui.label("Base shear against monitored displacement (model units). Hinge counts by state and by performance status.");
            draw_plot(
                ui,
                &curve_xy(&points),
                unfinished.as_ref().map(|_| "NOT FINISHED"),
            );
            egui::Grid::new("pushover_grid").striped(true).show(ui, |ui| {
                for h in [
                    "Step", "Displacement", "Base shear", "A-B", "B-C", "C-D", "D-E", ">E",
                    "A-IO", "IO-LS", "LS-CP", ">CP", "Total",
                ] {
                    ui.strong(h);
                }
                ui.end_row();
                // The base shear of the first state is roundoff (5e-15 of
                // 600): a value below a billionth of the largest is printed 0.
                let peak = points.iter().map(|p| p.base_shear.abs()).fold(0.0_f64, f64::max);
                for p in &points {
                    ui.label(p.step.to_string());
                    ui.label(super::view::quantity_text(p.displacement));
                    ui.label(super::view::quantity_text(super::view::without_roundoff(
                        p.base_shear,
                        peak,
                    )));
                    for n in [
                        p.a_to_b,
                        p.b_to_c,
                        p.c_to_d,
                        p.d_to_e,
                        p.beyond_e,
                        p.a_to_io,
                        p.io_to_ls,
                        p.ls_to_cp,
                        p.beyond_cp,
                        p.total,
                    ] {
                        ui.label(n.to_string());
                    }
                    ui.end_row();
                }
            });
        });
    state.open = open;
    None
}

/// Example 1-026 model A (kip, in), built in code, for the tests of the
/// window and of the viewport: a cantilever with a moment and a shear hinge
/// at its root, pushed by a unit tip load in +Z to Uz = 2 in.
#[cfg(test)]
pub(super) fn example_1_026_model() -> SapModel {
    use sap_model::hinge::{FrameHinge, HingeDof, HingePoint, HingeProp, ResultsSaved};
    use sap_model::{
        DisplacementControl, FramePropType, FrameSection, LoadPattern, LoadPatternType, Material,
        PointLoadForce,
    };
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    m.active_dofs = [true, false, true, false, true, false];
    let mut material = Material::default_steel_kip_in();
    material.name = "CONC".into();
    material.e = 3600.0;
    material.g = 1500.0;
    material.u = 3600.0 / 3000.0 - 1.0;
    material.unit_weight = 0.0;
    material.unit_mass = 0.0;
    m.materials.insert("CONC".into(), material);
    m.frame_sections.insert(
        "BEAM".into(),
        FrameSection {
            name: "BEAM".into(),
            material: "CONC".into(),
            prop_type: FramePropType::General,
            area: 216.0,
            i33: 5832.0,
            i22: 2592.0,
            torsional_j: 8424.0,
            as2: 180.0,
            as3: 0.0,
            modifiers: [1.0; 8],
            ..FrameSection::default()
        },
    );
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 24.0, 0.0, 0.0);
    m.points["1"].restraint = [true; 6];
    m.add_frame("1", "1", "2", "BEAM");
    m.load_patterns.insert(
        "P".into(),
        LoadPattern {
            name: "P".into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
    m.points["2"].loads_force.push(PointLoadForce {
        load_pat: "P".into(),
        csys: "GLOBAL".into(),
        force: [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    });
    m.hinge_props.insert(
        "Moment".into(),
        HingeProp::symmetric(
            "Moment",
            HingeDof::M3,
            [
                HingePoint::new(1440.0, 0.0),
                HingePoint::new(1920.0, 0.04),
                HingePoint::new(480.0, 0.04),
                HingePoint::new(480.0, 0.4),
            ],
        ),
    );
    m.hinge_props.insert(
        "Shear".into(),
        HingeProp::symmetric(
            "Shear",
            HingeDof::V2,
            [
                HingePoint::new(70.0, 0.0),
                HingePoint::new(80.0, 0.4),
                HingePoint::new(90.0, 0.8),
                HingePoint::new(100.0, 1.2),
            ],
        ),
    );
    m.frames["1"].hinges = ["Moment", "Shear"]
        .iter()
        .map(|p| FrameHinge {
            prop: (*p).into(),
            rel_dist: 0.0,
        })
        .collect();
    let mut push = sap_model::LoadCase::linear_static("PUSH", "P", 1.0);
    push.case_type = LoadCaseType::NonlinearStatic;
    push.displacement_control = Some(DisplacementControl {
        target: 2.0,
        monitored: Some(("2".into(), 2)),
        conjugate: false,
    });
    push.nl_static.results_saved = ResultsSaved::MultipleStates {
        min_saved: 6,
        max_saved: 10,
        positive_only: false,
    };
    m.load_cases.insert("PUSH".into(), push);
    m
}

#[cfg(test)]
mod tests {
    use super::*;
    use sap_model::hinge::HingeDisplayClass as C;

    fn analyzed() -> SapModel {
        let mut m = example_1_026_model();
        sap_solver::analyze_model(&mut m).unwrap();
        m
    }

    fn classes(m: &SapModel, step: Option<usize>) -> Vec<(String, f64, C)> {
        hinge_markers(m, "PUSH", step)
    }

    #[test]
    fn markers_follow_the_hinge_states_of_example_1_026_a_per_saved_state() {
        let m = analyzed();
        let at = |class| ("1".to_string(), 0.0, class);
        // States of the moment hinge (plastic rotation 0, 0.0138, 0.0200,
        // 0.0275, 0.0351 rad, then C, the drop above 480, and 480) and of
        // the shear hinge (0 up to P = 70, then 0.151, 0.302, 0.4 = C):
        // no markers while the hinges are at or below B, B-C is drawn as
        // B to IO (no acceptance criteria), C-D and D-E as such.
        let want: [Vec<(String, f64, C)>; 10] = [
            vec![],
            vec![],
            vec![at(C::BToIO)],
            vec![at(C::BToIO)],
            vec![at(C::BToIO), at(C::BToIO)],
            vec![at(C::BToIO), at(C::BToIO)],
            vec![at(C::CToD), at(C::CToD)],
            vec![at(C::CToD), at(C::CToD)],
            vec![at(C::DToE), at(C::CToD)],
            vec![at(C::DToE), at(C::CToD)],
        ];
        for (s, w) in want.iter().enumerate() {
            assert_eq!(&classes(&m, Some(s)), w, "markers at saved state {s}");
        }
        // No step selects the final state; a step past the end the last one.
        assert_eq!(classes(&m, None), want[9]);
        assert_eq!(classes(&m, Some(99)), want[9]);
        // Unknown cases and cases without results draw nothing.
        assert!(hinge_markers(&m, "NOPE", None).is_empty());
        assert!(hinge_markers(&example_1_026_model(), "PUSH", Some(3)).is_empty());
    }

    #[test]
    fn the_seven_classes_have_distinct_colours_in_the_order_of_the_legend() {
        let mut seen = std::collections::HashSet::new();
        for class in C::ALL {
            assert!(
                seen.insert(class_color(class)),
                "{class:?} repeats a colour"
            );
        }
        let labels: Vec<&str> = C::ALL.iter().map(|c| c.label()).collect();
        assert_eq!(
            labels,
            ["B to IO", "IO to LS", "LS to CP", "CP to C", "C to D", "D to E", ">E"]
        );
    }

    fn texts(out: &egui::FullOutput) -> Vec<String> {
        out.shapes
            .iter()
            .filter_map(|shape| match &shape.shape {
                egui::epaint::Shape::Text(text) => Some(text.galley.job.text.clone()),
                _ => None,
            })
            .collect()
    }

    fn input(time: f64) -> egui::RawInput {
        egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(1200.0, 1000.0),
            )),
            time: Some(time),
            ..Default::default()
        }
    }

    /// Three frames: a window is laid out invisibly on its first frame.
    fn frames(ctx: &egui::Context, state: &mut State, model: &mut SapModel) -> egui::FullOutput {
        let mut out = ctx.run(input(0.0), |ctx| {
            render(ctx, state, model);
        });
        for t in [1.0, 2.0] {
            out = ctx.run(input(t), |ctx| {
                render(ctx, state, model);
            });
        }
        out
    }

    #[test]
    fn the_window_shows_the_curve_as_a_table_and_only_reads() {
        let mut model = analyzed();
        let before = model.clone();
        let mut state = State {
            open: true,
            case: String::new(),
        };
        let ctx = egui::Context::default();
        let t = texts(&frames(&ctx, &mut state, &mut model));
        for header in [
            "Step",
            "Displacement",
            "Base shear",
            "A-B",
            ">E",
            "A-IO",
            ">CP",
            "Total",
        ] {
            assert!(
                t.iter().any(|s| s == header),
                "missing column {header}: {t:?}"
            );
        }
        // Rows of the first saved states (the window scrolls the rest): point 1, 2.
        // The cells print the value with at least five significant digits
        // (they were rounded to 5 and 4 decimals before): compare as numbers,
        // with the resolution of those decimals.
        let has = |want: f64, tol: f64| {
            t.iter().any(|s| s.parse::<f64>().is_ok_and(|v| (v - want).abs() <= tol))
        };
        assert!(has(0.0185, 5e-6), "{t:?}");
        assert!(has(60.0, 5e-5) && has(80.0, 5e-5), "{t:?}");
        assert!(!t.iter().any(|s| s.starts_with("-0") && s.parse::<f64>() == Ok(0.0)));
        assert_eq!(state.case, "PUSH");
        assert_eq!(model, before, "the window must not edit the model");
    }

    #[test]
    fn the_window_explains_a_model_without_results_and_a_refused_case() {
        let mut model = example_1_026_model();
        let mut state = State {
            open: true,
            case: String::new(),
        };
        let ctx = egui::Context::default();
        let t = texts(&frames(&ctx, &mut state, &mut model));
        assert!(
            t.iter().any(|s| s.contains("No nonlinear static results")),
            "{t:?}"
        );
        // A case with results that cannot give a curve shows the refusal.
        let mut model = analyzed();
        model
            .load_cases
            .get_mut("PUSH")
            .unwrap()
            .displacement_control = None;
        let t = texts(&frames(&ctx, &mut state, &mut model));
        assert!(
            t.iter().any(|s| s.contains("displacement control")),
            "{t:?}"
        );
        // Closed window: nothing is drawn.
        state.open = false;
        assert!(texts(&frames(&ctx, &mut state, &mut model)).is_empty());
    }

    /// G16: a case that stopped before its target shows the reason above its table and
    /// "NOT FINISHED" in the plot title; a finished case shows neither.
    #[test]
    fn a_case_that_is_not_finished_says_so_above_the_table_and_in_the_plot() {
        let ctx = egui::Context::default();
        let mut state = State {
            open: true,
            case: String::new(),
        };
        let mut finished = analyzed();
        let t = texts(&frames(&ctx, &mut state, &mut finished));
        assert!(!t.iter().any(|s| s.contains("Not finished") || s == "NOT FINISHED"), "{t:?}");
        let mut model = example_1_026_model();
        model.load_cases.get_mut("PUSH").unwrap().nl_static.max_total_steps = 5;
        sap_solver::analyze_model(&mut model).unwrap();
        let reason = match &model.case_results["PUSH"].status {
            sap_model::CaseStatus::NotFinished { reason, .. } => reason.clone(),
            other => panic!("expected NotFinished, got {other:?}"),
        };
        let t = texts(&frames(&ctx, &mut state, &mut model));
        assert!(
            t.iter()
                .any(|s| s.starts_with("Not finished: ") && s.contains(&reason)),
            "{t:?}"
        );
        assert!(t.iter().any(|s| s == "NOT FINISHED"), "{t:?}");
        // The table still lists the states the case reached.
        assert!(t.iter().any(|s| s == "Step"), "{t:?}");
        // A case that stopped at its first step keeps one state only: no curve can be
        // drawn, and the window still says that the case is not finished.
        model.case_results.get_mut("PUSH").unwrap().static_steps.truncate(1);
        let t = texts(&frames(&ctx, &mut state, &mut model));
        assert!(t.iter().any(|s| s.starts_with("Not finished: ")), "{t:?}");
        assert!(t.iter().any(|s| s.contains("saved a single state")), "{t:?}");
    }

    #[test]
    fn the_plot_handles_degenerate_curves() {
        let points = [(0.0, 0.0), (1.0, 5.0), (2.0, 3.0)];
        let ctx = egui::Context::default();
        let _ = ctx.run(input(0.0), |ctx| {
            egui::CentralPanel::default().show(ctx, |ui| {
                draw_plot_untitled(ui, &points);
                draw_plot_untitled(ui, &points[..1]);
                draw_plot_untitled(ui, &[(0.0, 0.0), (0.0, 0.0)]);
                draw_plot_untitled(ui, &[(0.0, f64::NAN), (1.0, 1.0)]);
            });
        });
        assert_eq!(curve_xy(&[]).len(), 0);
    }
}
