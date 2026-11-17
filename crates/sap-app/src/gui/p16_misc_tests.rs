//! P16, D55 and D45: small repairs of the windows (each test names its
//! reference).

use super::wave5_windows_tests::{close, modal_model, texts, Run};
use super::*;
use eframe::egui::{Pos2, Rect};

#[test]
fn the_duration_of_a_function_read_from_a_file_has_no_float_noise() {
    // The hand check read 2000 points at 0.02 s: the last time is 1999 * 0.02,
    // which a float prints as 39.980000000000004; the duration is 39.98 s.
    let last = 1999.0 * 0.02;
    assert_ne!(last.to_string(), "39.98", "the noise this test is about");
    let text = time_history::loaded_summary(&[(0.0, 0.0), (last, -3.5)]);
    assert_eq!(text, "2 points, duration 39.98 s, largest |value| 3.5");
    // The existing wording for a round value is unchanged.
    assert_eq!(
        time_history::loaded_summary(&[(0.0, 0.0), (0.5, 2.0), (1.0, 0.0)]),
        "3 points, duration 1 s, largest |value| 2"
    );
}

fn window_rect(run: &Run, title: &str) -> Rect {
    run.ctx.memory(|m| m.area_rect(egui::Id::new(title))).unwrap_or_else(|| panic!("no window {title}"))
}

#[test]
fn big_windows_open_inside_a_1440_by_900_screen_and_scroll() {
    let mut run = Run::new(SapModel::new(Units::KnMC));
    run.screen = Vec2::new(1440.0, 900.0);
    let screen = Rect::from_min_size(Pos2::ZERO, run.screen);
    for tab in 0..7 {
        run.app.open_workflow(tab);
        run.settle();
        let r = window_rect(&run, "Model Workflows");
        assert!(r.height() <= 900.0 - 40.0, "workflows tab {tab}: {r:?}");
        assert!(screen.contains_rect(r), "workflows tab {tab}: {r:?} in {screen:?}");
    }
    run.app.workflows.open = false;
    run.app.panel_seismic.open = true;
    run.settle();
    let r = window_rect(&run, "Mass Source and Auto Seismic Loads");
    assert!(r.height() <= 900.0 - 40.0 && screen.contains_rect(r), "{r:?}");

    // Every control is reachable: the action button of the Auto Seismic tab is
    // below the fold of the window at first and a wheel turn brings it in.
    run.click_text("Auto seismic (ASCE 7-10)");
    let window = window_rect(&run, "Mass Source and Auto Seismic Loads");
    let button = "Set automatic lateral load";
    let visible = |run: &Run| {
        texts(&run.last)
            .into_iter()
            .find(|(t, _)| t == button)
            .is_some_and(|(_, r)| window.contains_rect(r))
    };
    for _ in 0..40 {
        if visible(&run) {
            break;
        }
        run.frame(vec![egui::Event::MouseWheel {
            unit: egui::MouseWheelUnit::Point,
            delta: Vec2::new(0.0, -120.0),
            modifiers: Default::default(),
        }]);
        run.frame(vec![egui::Event::PointerMoved(window.center())]);
    }
    assert!(visible(&run), "{:?}", run.all_texts());
}

#[test]
fn the_video_magnification_is_the_automatic_factor_until_the_user_edits_it() {
    // modal_model: the largest translation of mode 1 is |(30, 0, 40)| = 50 m
    // on a model of diagonal 5 m, so the automatic factor is 0.1 * 5 / 50 =
    // 0.01 and that of mode 2 (a tenth of the amplitude) is 0.1.
    let mut run = Run::new(modal_model());
    run.click_text("File");
    run.click_text("Create Video");
    run.click_text("Cyclic Animation Video...");
    assert!(run.has_text("Create Cyclic Animation Video"), "{:?}", run.all_texts());
    close("mode 1", run.app.video_form.magnification_value(), 0.01, 1e-9);
    assert!(run.has_text("0.01"), "the box shows the factor: {:?}", run.all_texts());
    run.app.video_form.set_mode(2);
    run.settle();
    close("mode 2", run.app.video_form.magnification_value(), 0.1, 1e-9);
    // An edit is the user's: a later change of mode leaves it alone.
    run.app.video_form.set_magnification(7.0);
    run.app.video_form.set_mode(1);
    run.settle();
    assert_eq!(run.app.video_form.magnification_value(), 7.0);
    // Reopening the form starts from the automatic factor again.
    run.app.video_form.open(&run.app.model, true);
    run.settle();
    close("reopened", run.app.video_form.magnification_value(), 0.01, 1e-9);
}

#[test]
fn the_title_of_a_narrow_pane_wraps_inside_the_pane() {
    // Mode 1 of the modal model: "3D Perspective View  [Result: M]  Mode 1,
    // Period = 0.50000 s, Frequency = 2.0000 Hz" is about 480 px wide at 12.5
    // pt; the pane of a 700 px screen next to the 275 px explorer is ~400 px.
    let mut run = Run::new(modal_model());
    run.app.left_proj = ViewProjection::Perspective3D;
    run.app.active_case = "M".into();
    run.app.display_mode = DisplayMode::Deformed;
    run.screen = Vec2::new(700.0, 500.0);
    run.settle();
    let (title, rect) = texts(&run.last)
        .into_iter()
        .find(|(t, _)| t.starts_with("3D Perspective View"))
        .expect("the pane title");
    assert!(title.contains("Frequency = 2.0000 Hz"), "{title}");
    assert!(rect.right() <= run.screen.x, "the title ends at {} on a {} px screen", rect.right(), run.screen.x);
    assert!(rect.height() > 20.0, "wrapped onto more than one line: {rect:?}");
}

#[test]
fn the_pushover_plot_has_readable_ticks_and_every_label_inside_the_plot() {
    // Closed forms: the ticks of [-0.03, 0] are multiples of 0.01 (the step
    // 1, 2 or 5 times a power of ten that is at least a fifth of the range),
    // those of [-284.5, 0] multiples of 100.
    let near = |a: &[f64], b: &[f64]| a.len() == b.len() && a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-12);
    assert!(near(&pushover::nice_ticks(-0.03, 0.0, 5), &[-0.03, -0.02, -0.01, 0.0]));
    assert!(near(&pushover::nice_ticks(-284.5, 0.0, 5), &[-200.0, -100.0, 0.0]));
    assert!(near(&pushover::nice_ticks(0.0, 1.0, 5), &[0.0, 0.2, 0.4, 0.6, 0.8, 1.0]));
    assert!(pushover::nice_ticks(1.0, 1.0, 5).is_empty());
    assert_eq!(pushover::tick_text(-0.03), "-0.03");
    assert_eq!(pushover::tick_text(3.0_f64 * 0.1), "0.3");

    // The curve of the hand check: displacement and base shear both negative.
    // The origin is then at the right edge of the plot, where the old label
    // of the shear axis ran off the picture.
    let xy = [(0.0, 0.0), (-0.01, -150.0), (-0.03, -284.0)];
    let ctx = egui::Context::default();
    let screen = Rect::from_min_size(Pos2::ZERO, Vec2::new(400.0, 400.0));
    let out = ctx.run(
        egui::RawInput { screen_rect: Some(screen), ..Default::default() },
        |ctx| {
            egui::CentralPanel::default().show(ctx, |ui| pushover::draw_plot(ui, &xy));
        },
    );
    let all = texts(&out);
    for want in ["-0.03", "-0.02", "-0.01", "-200", "-100"] {
        assert!(all.iter().any(|(t, _)| t == want), "tick {want} in {:?}", all.iter().map(|t| &t.0).collect::<Vec<_>>());
    }
    for (t, r) in &all {
        assert!(screen.contains_rect(*r) && r.right() <= 400.0 - 8.0, "'{t}' at {r:?} leaves the picture");
    }
}
