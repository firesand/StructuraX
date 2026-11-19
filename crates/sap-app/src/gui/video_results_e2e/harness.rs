//! Headless driver shared by the end-to-end tests of the result displays:
//! the application is run frame by frame through `render_ui` with synthetic
//! pointer and keyboard input, and what it painted is read back as texts,
//! polygon fills and meshes.

use super::super::view::ViewTransform;
use super::super::{SapGuiApp, ViewProjection};
use eframe::egui::{self, epaint::Shape, Color32, Pos2, Rect, Vec2};
use sap_model::SapModel;
use std::path::PathBuf;

pub const SCREEN: Vec2 = Vec2::new(1800.0, 1400.0);

pub fn close(what: &str, got: f64, want: f64, rel: f64) {
    assert!(
        (got - want).abs() <= rel * want.abs().max(1e-300),
        "{what}: got {got:e}, want {want:e} (relative difference {:e}, allowed {rel:e})",
        (got - want).abs() / want.abs().max(1e-300)
    );
}

pub fn flat(out: &egui::FullOutput) -> Vec<Shape> {
    fn walk(s: &Shape, into: &mut Vec<Shape>) {
        match s {
            Shape::Vec(v) => v.iter().for_each(|x| walk(x, into)),
            other => into.push(other.clone()),
        }
    }
    let mut all = Vec::new();
    for c in &out.shapes {
        walk(&c.shape, &mut all);
    }
    all
}

pub fn texts(out: &egui::FullOutput) -> Vec<(String, Rect)> {
    flat(out)
        .into_iter()
        .filter_map(|s| match s {
            Shape::Text(t) => {
                Some((t.galley.job.text.clone(), Rect::from_min_size(t.pos, t.galley.size())))
            }
            _ => None,
        })
        .collect()
}

pub fn meshes(out: &egui::FullOutput) -> Vec<Vec<egui::epaint::Vertex>> {
    flat(out)
        .into_iter()
        .filter_map(|s| match s {
            Shape::Mesh(m) => Some(m.vertices.clone()),
            _ => None,
        })
        .collect()
}

/// A run of the application on a model.
pub struct Run {
    pub app: SapGuiApp,
    pub ctx: egui::Context,
    pub time: f64,
    pub last: egui::FullOutput,
}

impl Run {
    /// The application with `model`, one pane in plan view.
    pub fn new(model: SapModel) -> Self {
        let mut app = SapGuiApp::new(Some(PathBuf::from("/nonexistent/video-results.s2k")));
        app.model = model;
        app.dual_pane = false;
        app.left_proj = ViewProjection::PlanXY;
        app.left_plane = None;
        let mut run =
            Self { app, ctx: egui::Context::default(), time: 0.0, last: Default::default() };
        for _ in 0..3 {
            run.frame(vec![]);
        }
        run
    }

    pub fn frame_at(&mut self, time: f64, events: Vec<egui::Event>) {
        self.time = time;
        let input = egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, SCREEN)),
            time: Some(time),
            events,
            ..Default::default()
        };
        self.last = self.ctx.run(input, |ctx| self.app.render_ui(ctx));
    }

    pub fn frame(&mut self, events: Vec<egui::Event>) {
        self.frame_at(self.time + 0.05, events);
    }

    pub fn settle(&mut self) {
        for _ in 0..3 {
            self.frame(vec![]);
        }
    }

    fn button(pos: Pos2, b: egui::PointerButton, pressed: bool) -> egui::Event {
        egui::Event::PointerButton { pos, button: b, pressed, modifiers: Default::default() }
    }

    fn click_at(&mut self, pos: Pos2, b: egui::PointerButton) {
        self.frame(vec![egui::Event::PointerMoved(pos), Self::button(pos, b, true)]);
        self.frame(vec![Self::button(pos, b, false)]);
        self.time += 1.0; // past the double-click interval
        self.settle();
    }

    pub fn click(&mut self, pos: Pos2) {
        self.click_at(pos, egui::PointerButton::Primary);
    }

    pub fn right_click(&mut self, pos: Pos2) {
        self.click_at(pos, egui::PointerButton::Secondary);
    }

    pub fn texts(&self) -> Vec<String> {
        texts(&self.last).into_iter().map(|t| t.0).collect()
    }

    /// The painted texts without the model explorer's rows (joint numbers and
    /// object lines), for failure messages.
    pub fn shown(&self) -> Vec<String> {
        self.texts()
            .into_iter()
            .filter(|t| !t.starts_with("Fr ") && !t.starts_with("Ar ") && !t.chars().all(|c| c.is_ascii_digit()))
            .collect()
    }

    /// Rectangle of the first text with exactly `label` painted in the last frame.
    pub fn find(&self, label: &str) -> Rect {
        texts(&self.last)
            .into_iter()
            .find(|(t, _)| t == label)
            .unwrap_or_else(|| panic!("no text '{label}' in {:?}", self.shown()))
            .1
    }

    pub fn has_text(&self, label: &str) -> bool {
        texts(&self.last).iter().any(|(t, _)| t == label)
    }

    /// The first painted text that starts with `prefix`.
    pub fn text_starting(&self, prefix: &str) -> Option<String> {
        self.texts().into_iter().find(|t| t.starts_with(prefix))
    }

    pub fn click_text(&mut self, label: &str) {
        let r = self.find(label);
        self.click(r.center());
    }

    /// Types `text` into the field right of the label `label_before` (the last
    /// such label painted: the Shells form has radio buttons "Min" and "Max"
    /// above its range fields).
    pub fn type_into(&mut self, label_before: &str, text: &str) {
        let r = texts(&self.last)
            .into_iter()
            .rev()
            .find(|(t, _)| t == label_before)
            .unwrap_or_else(|| panic!("no text '{label_before}' in {:?}", self.shown()))
            .1;
        self.click(Pos2::new(r.right() + 30.0, r.center().y));
        // Select all and delete what the field holds, then type.
        let ctrl = egui::Modifiers { ctrl: true, command: true, ..Default::default() };
        let key = |key, pressed, modifiers| egui::Event::Key {
            key,
            physical_key: None,
            pressed,
            repeat: false,
            modifiers,
        };
        self.frame(vec![key(egui::Key::A, true, ctrl), key(egui::Key::A, false, ctrl)]);
        self.frame(vec![
            key(egui::Key::Backspace, true, Default::default()),
            key(egui::Key::Backspace, false, Default::default()),
        ]);
        if !text.is_empty() {
            self.frame(vec![egui::Event::Text(text.to_string())]);
        }
        self.settle();
    }

    /// The nearest text on the row of the label `label` (the last such label
    /// painted), to the right of it: the value of a drag value or the selected
    /// text of a combo box.
    fn value_right_of(&self, label: &str) -> Rect {
        let all = texts(&self.last);
        let l = all
            .iter()
            .rev()
            .find(|(t, _)| t == label)
            .unwrap_or_else(|| panic!("no text '{label}' in {:?}", self.shown()))
            .1;
        all.iter()
            .filter(|(_, r)| (r.center().y - l.center().y).abs() < 4.0 && r.left() >= l.right() - 0.5)
            .min_by(|a, b| a.1.left().total_cmp(&b.1.left()))
            .unwrap_or_else(|| panic!("nothing right of '{label}'"))
            .1
    }

    /// Sets the drag value on the row of `label` by clicking it (which starts
    /// the text edit), selecting all, typing `text` and pressing Enter.
    pub fn set_number(&mut self, label: &str, text: &str) {
        let r = self.value_right_of(label);
        self.click(r.center());
        let ctrl = egui::Modifiers { ctrl: true, command: true, ..Default::default() };
        let key = |key, pressed, modifiers| egui::Event::Key {
            key,
            physical_key: None,
            pressed,
            repeat: false,
            modifiers,
        };
        self.frame(vec![key(egui::Key::A, true, ctrl), key(egui::Key::A, false, ctrl)]);
        self.frame(vec![egui::Event::Text(text.to_string())]);
        self.frame(vec![
            key(egui::Key::Enter, true, Default::default()),
            key(egui::Key::Enter, false, Default::default()),
        ]);
        self.settle();
    }

    /// Sets the `index`-th number (counting from zero) on the row of the label
    /// `label` (the last such label painted), as `set_number` does. A row of the
    /// Build window reads `Start X [0] Y [0] Z [0]`; the numbers are the drag values.
    pub fn set_number_at(&mut self, label: &str, index: usize, text: &str) {
        let all = texts(&self.last);
        let l = all
            .iter()
            .rev()
            .find(|(t, _)| t == label)
            .unwrap_or_else(|| panic!("no text '{label}' in {:?}", self.shown()))
            .1;
        let mut numbers: Vec<Rect> = all
            .iter()
            .filter(|(t, r)| {
                (r.center().y - l.center().y).abs() < 4.0
                    && r.left() >= l.right() - 0.5
                    && t.parse::<f64>().is_ok()
            })
            .map(|(_, r)| *r)
            .collect();
        numbers.sort_by(|a, b| a.left().total_cmp(&b.left()));
        let r = *numbers
            .get(index)
            .unwrap_or_else(|| panic!("row of '{label}' has {} numbers", numbers.len()));
        self.click(r.center());
        let ctrl = egui::Modifiers { ctrl: true, command: true, ..Default::default() };
        let key = |key, pressed, modifiers| egui::Event::Key {
            key,
            physical_key: None,
            pressed,
            repeat: false,
            modifiers,
        };
        self.frame(vec![key(egui::Key::A, true, ctrl), key(egui::Key::A, false, ctrl)]);
        self.frame(vec![egui::Event::Text(text.to_string())]);
        self.frame(vec![
            key(egui::Key::Enter, true, Default::default()),
            key(egui::Key::Enter, false, Default::default()),
        ]);
        self.settle();
    }

    /// Opens the combo box on the row of `label` and picks `item` (the last
    /// painted text of that name, which is the popup entry).
    pub fn choose(&mut self, label: &str, item: &str) {
        let r = self.value_right_of(label);
        self.click(r.center());
        let entry = texts(&self.last)
            .into_iter()
            .rev()
            .find(|(t, _)| t == item)
            .unwrap_or_else(|| panic!("no entry '{item}' in {:?}", self.shown()))
            .1;
        self.click(entry.center());
    }

    pub fn view(&self) -> ViewTransform {
        self.app.last_view[0].expect("pane drawn").0
    }

    pub fn screen(&self, p: [f64; 3]) -> Pos2 {
        self.view().project(p)
    }
}

/// Opens the Build window ("Grid, Quick Draw, Radial and Selection") on the tab
/// with the label `tab`, by a pointer click on the tab.
pub fn open_build_tab(run: &mut Run, tab: &str) {
    run.app.panel_build.open = true;
    run.settle();
    run.click_text(tab);
}

fn key3(m: &SapModel, id: &str) -> [i64; 3] {
    let p = &m.points[id];
    [p.x, p.y, p.z].map(|v| (v * 1e6).round() as i64)
}

/// Joints, frames (section and end pair) and areas (section and corners) of a
/// model as sorted coordinate lists in micrometres, so that two models built
/// by different routes compare equal whatever their joint names.
pub type Geometry = (Vec<[i64; 3]>, Vec<(String, [i64; 3], [i64; 3])>, Vec<(String, Vec<[i64; 3]>)>);

pub fn geometry(m: &SapModel) -> Geometry {
    let mut joints: Vec<[i64; 3]> = m.points.keys().map(|id| key3(m, id)).collect();
    joints.sort();
    let mut frames: Vec<_> = m
        .frames
        .values()
        .map(|f| {
            let (a, b) = (key3(m, &f.point_i), key3(m, &f.point_j));
            (f.section.clone(), a.min(b), a.max(b))
        })
        .collect();
    frames.sort();
    let mut areas: Vec<_> = m
        .areas
        .values()
        .map(|a| {
            let mut corners: Vec<[i64; 3]> = a.points.iter().map(|id| key3(m, id)).collect();
            // Start at the smallest corner, keeping the order around the area.
            let start = (0..corners.len()).min_by_key(|&i| corners[i]).unwrap();
            corners.rotate_left(start);
            (a.section.clone(), corners)
        })
        .collect();
    areas.sort();
    (joints, frames, areas)
}

/// Fills of the closed polygons that are not transparent (`view::outlined_polygon`
/// draws a fill as a one-colour fan; other closed paths keep their fill).
pub fn polygon_fills(out: &egui::FullOutput) -> Vec<Color32> {
    flat(out)
        .into_iter()
        .filter_map(|s| match s {
            Shape::Path(p) if p.closed && p.fill != Color32::TRANSPARENT => Some(p.fill),
            other => super::super::view::polygon_fill_of(&other),
        })
        .collect()
}
