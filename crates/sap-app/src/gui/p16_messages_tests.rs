//! P16, D54: a status message longer than the status line can be read in full
//! on hover and in the Messages window.

use super::wave5_windows_tests::{flat, texts, Run};
use super::*;
use eframe::egui::{epaint::Shape, Pos2};
use sap_model::{UnmodeledScope, UnmodeledSourceData};

/// Glyphs of the first painted text equal to `full` that are on screen (a
/// truncated label paints fewer than the characters of its text).
fn painted_glyphs(run: &Run, full: &str) -> Vec<usize> {
    flat(&run.last)
        .into_iter()
        .filter_map(|s| match s {
            Shape::Text(t) if t.galley.job.text == full => {
                Some(t.galley.rows.iter().map(|r| r.glyphs.len()).sum())
            }
            _ => None,
        })
        .collect()
}

/// A refused analysis: the model holds source data that is not modeled, the
/// description of which makes the solver's message far longer than the line.
fn refused_analysis() -> (Run, String) {
    let mut app = SapGuiApp::new(None);
    app.model.unmodeled_source.push(UnmodeledSourceData {
        scope: UnmodeledScope::AllCases,
        description: format!("link parameters of the isolators {} END-OF-REASON", "ISO ".repeat(80)),
    });
    assert!(!app.execute_analysis());
    let full = app.status_msg.clone();
    assert!(full.starts_with("Analysis Error:") && full.len() > 300, "{full}");
    let mut run = Run::new(SapModel::new(Units::KnMC));
    run.app = app;
    run.settle();
    (run, full)
}

#[test]
fn the_whole_reason_of_a_refused_analysis_is_readable_on_hover_and_in_the_messages_window() {
    let (mut run, full) = refused_analysis();
    // The status line is one cut line: fewer glyphs than the message has.
    let glyphs = painted_glyphs(&run, &full);
    assert_eq!(glyphs.len(), 1, "one painted status line");
    assert!(glyphs[0] < full.chars().count(), "the line shows {} of {}", glyphs[0], full.len());
    assert!(!run.has_text_containing("END-OF-REASON") || glyphs[0] < full.len());

    // Hover: a tooltip that holds the whole text.
    let status = texts(&run.last)
        .into_iter()
        .find(|(t, _)| *t == full)
        .expect("status text")
        .1;
    run.frame(vec![egui::Event::PointerMoved(status.left_center() + Vec2::new(20.0, 0.0))]);
    run.frame(vec![]);
    run.frame(vec![]);
    let hovered = painted_glyphs(&run, &full);
    assert_eq!(hovered.len(), 2, "status line and one tooltip: {:?}", run.all_texts());
    assert_eq!(hovered.iter().max().copied(), Some(full.chars().count()), "the tooltip is whole");

    // The Messages window (Help menu): the entry is marked Error and whole.
    run.frame(vec![egui::Event::PointerMoved(Pos2::new(900.0, 700.0))]);
    run.click_text("Help");
    run.click_text("Messages...");
    assert!(run.app.messages.open);
    assert!(run.has_text("Messages"));
    assert!(run.has_text("Error"), "{:?}", run.all_texts());
    let in_window = painted_glyphs(&run, &full);
    assert_eq!(in_window.len(), 2, "status line and the window: {:?}", run.all_texts());
    assert_eq!(in_window.iter().max().copied(), Some(full.chars().count()));

    // Copy puts the whole message on the clipboard.
    let copy = texts(&run.last)
        .into_iter()
        .filter(|(t, _)| t == "Copy")
        .map(|(_, r)| r)
        .next()
        .expect("a Copy button");
    let pos = copy.center();
    let ev = |pressed| egui::Event::PointerButton {
        pos,
        button: egui::PointerButton::Primary,
        pressed,
        modifiers: Default::default(),
    };
    run.frame(vec![egui::Event::PointerMoved(pos), ev(true)]);
    run.frame(vec![ev(false)]);
    // The clipboard text is part of the output of the frame that saw the click.
    assert_eq!(run.last.platform_output.copied_text, full);
}

#[test]
fn the_messages_are_newest_first_bounded_and_the_errors_are_marked() {
    let mut run = Run::new(SapModel::new(Units::KnMC));
    for i in 1..=(messages::MAX_MESSAGES + 30) {
        run.app.status_msg = if i % 50 == 0 {
            format!("Analysis Error: refused {i}")
        } else {
            format!("Saved item {i}")
        };
        run.frame(vec![]);
    }
    assert_eq!(run.app.messages.len(), messages::MAX_MESSAGES);
    let newest = run.app.messages.newest_first().next().unwrap().clone();
    assert_eq!(newest.text, format!("Saved item {}", messages::MAX_MESSAGES + 30));
    // "Analysis Error: refused 150" is an error, "Saved item 149" is not.
    assert!(messages::is_error("Analysis Error: refused 150"));
    assert!(!messages::is_error("Saved item 149"));
    // A done command with warnings in its text is not an error.
    assert!(!messages::is_error("Opened f.sdb - Import warnings: saving cannot preserve the model"));
    assert!(messages::is_error("Model is locked. Click 'Edit > Unlock Model' before modifying the model."));
    assert!(messages::is_error("Not done: the grid must be increasing"));
    assert!(!messages::is_error("Analysis Complete: 8 joints, 12 frames solved across 3 results."));
    run.app.messages.open = true;
    run.settle();
    let at = |label: &str| {
        // The newest message is also the status line, at the bottom: the
        // listing in the window is the entry nearest the top.
        texts(&run.last)
            .into_iter()
            .filter(|(t, _)| t == label)
            .map(|(_, r)| r.top())
            .min_by(f32::total_cmp)
    };
    let newer = at(&format!("Saved item {}", messages::MAX_MESSAGES + 30)).expect("newest listed");
    let older = at(&format!("Saved item {}", messages::MAX_MESSAGES + 28)).expect("older listed");
    assert!(newer < older, "newest first: {newer} vs {older}");
    // The first message, 200 + 30 back, was dropped.
    assert!(at("Saved item 1").is_none());
}
