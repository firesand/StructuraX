//! Helpers on top of the shared headless driver: menu paths, the analysis
//! command, and clicks on a row of a form.

use super::harness::{texts, Run};

/// Clicks the menu entries in order (a menu, its sub-menu, the command).
pub fn menu(run: &mut Run, path: &[&str]) {
    for item in path {
        run.click_text(item);
    }
}

/// Analyze > Run Analysis, as a user picks it.
pub fn analyze(run: &mut Run) {
    menu(run, &["Analyze", "▶ Run Analysis (F5)"]);
}

/// Edit > Unlock Model.
pub fn unlock(run: &mut Run) {
    menu(run, &["Edit", "Unlock Model"]);
}

/// Clicks the text `item` that is painted on the row of the label `row_label`.
pub fn click_on_row(run: &mut Run, row_label: &str, item: &str) {
    let all = texts(&run.last);
    let l = all
        .iter()
        .rev()
        .find(|(t, _)| t == row_label)
        .unwrap_or_else(|| panic!("no text '{row_label}' in {:?}", run.shown()))
        .1;
    let r = all
        .iter()
        .find(|(t, r)| t == item && (r.center().y - l.center().y).abs() < 4.0 && r.left() > l.right() - 0.5)
        .unwrap_or_else(|| panic!("no '{item}' right of '{row_label}' in {:?}", run.shown()))
        .1;
    run.click(r.center());
}

/// The first painted text that contains `part`.
pub fn text_containing(run: &Run, part: &str) -> Option<String> {
    run.texts().into_iter().find(|t| t.contains(part))
}

/// Replaces the text of the text field whose label is `label` and whose editor starts `dx` points
/// right of the label's left edge (a grid column is as wide as its widest label).
pub fn type_at(run: &mut Run, label: &str, dx: f32, text: &str) {
    use eframe::egui;
    let r = run.find(label);
    run.click(egui::Pos2::new(r.left() + dx, r.center().y));
    let ctrl = egui::Modifiers { ctrl: true, command: true, ..Default::default() };
    let key = |key, pressed, modifiers| egui::Event::Key { key, physical_key: None, pressed, repeat: false, modifiers };
    run.frame(vec![key(egui::Key::A, true, ctrl), key(egui::Key::A, false, ctrl)]);
    run.frame(vec![egui::Event::Text(text.to_string())]);
    run.settle();
}

/// Scrolls the window with the text `anchor` by `points` downwards (negative: up) with the
/// mouse wheel over its body, as a user does when a window is taller than the screen.
pub fn scroll_window(run: &mut Run, anchor: &str, points: f32) {
    use eframe::egui;
    let r = run.find(anchor);
    let pos = egui::Pos2::new(r.center().x, r.center().y + 40.0);
    run.frame(vec![egui::Event::PointerMoved(pos)]);
    run.frame(vec![egui::Event::MouseWheel {
        unit: egui::MouseWheelUnit::Point,
        delta: egui::Vec2::new(0.0, -points),
        modifiers: Default::default(),
    }]);
    run.settle();
}
