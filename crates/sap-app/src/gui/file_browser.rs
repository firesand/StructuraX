//! An in-application file chooser built on `std::fs`. The program has no
//! dependency on an operating-system dialog.

use eframe::egui;
use std::path::{Path, PathBuf};

/// What a chosen path is for; the window that asked takes it back by purpose.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Purpose {
    ModelFile,
    SectionFile,
    FunctionFile,
    MaterialLibraryDir,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct Entry {
    pub name: String,
    pub is_dir: bool,
}

#[derive(Debug, Clone, Default)]
pub(super) struct State {
    open: bool,
    purpose: Option<Purpose>,
    pick_dir: bool,
    extensions: Vec<String>,
    /// The directory shown.
    dir: PathBuf,
    /// The typed path field.
    typed: String,
    entries: Vec<Entry>,
    /// Why the directory could not be listed, or the last refused input.
    error: Option<String>,
    /// The highlighted file name (file mode only).
    selected: Option<String>,
    /// A finished choice waiting for its window to take it.
    choice: Option<(Purpose, PathBuf)>,
}

impl State {
    fn show_dir(&mut self, dir: PathBuf) {
        let dir = std::fs::canonicalize(&dir).unwrap_or(dir);
        match list_dir(&dir, &self.extensions_ref(), self.pick_dir) {
            Ok(entries) => {
                self.entries = entries;
                self.error = None;
            }
            Err(error) => {
                self.entries.clear();
                self.error = Some(error);
            }
        }
        self.typed = dir.display().to_string();
        self.dir = dir;
        self.selected = None;
    }

    fn extensions_ref(&self) -> Vec<&str> {
        self.extensions.iter().map(String::as_str).collect()
    }

    /// One level up; the root stays where it is.
    fn go_up(&mut self) {
        if let Some(parent) = self.dir.parent() {
            let parent = parent.to_path_buf();
            self.show_dir(parent);
        }
    }

    /// Enter a listed subdirectory.
    fn enter(&mut self, name: &str) {
        let dir = self.dir.join(name);
        self.show_dir(dir);
    }

    /// Apply the typed path: a directory is shown; an existing file is
    /// highlighted in its directory (file mode); anything else is an error.
    fn go_typed(&mut self) {
        let text = self.typed.trim().to_string();
        let path = PathBuf::from(&text);
        if text.is_empty() {
            self.error = Some("type a path".into());
        } else if path.is_dir() {
            self.show_dir(path);
        } else if !self.pick_dir && path.is_file() {
            let name = path
                .file_name()
                .map(|n| n.to_string_lossy().into_owned())
                .unwrap_or_default();
            if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
                self.show_dir(parent.to_path_buf());
            }
            self.selected = Some(name);
        } else {
            self.error = Some(format!("'{text}' is not a directory"));
        }
    }

    /// Finish with the highlighted file, the typed path, or (directory mode)
    /// the directory shown. Returns false and sets an error when nothing can
    /// be chosen.
    fn choose(&mut self) -> bool {
        let Some(purpose) = self.purpose else {
            return false;
        };
        let text = self.typed.trim().to_string();
        let shown = self.dir.display().to_string();
        let path = if self.pick_dir {
            if text != shown && !text.is_empty() {
                let typed = PathBuf::from(&text);
                if !typed.is_dir() {
                    self.error = Some(format!("'{text}' is not a directory"));
                    return false;
                }
                typed
            } else {
                self.dir.clone()
            }
        } else if let Some(name) = &self.selected {
            self.dir.join(name)
        } else if text != shown && !text.is_empty() {
            let typed = PathBuf::from(&text);
            if typed.is_dir() {
                self.show_dir(typed);
                return false;
            }
            let parent_exists = typed
                .parent()
                .map(|p| p.as_os_str().is_empty() || p.is_dir())
                .unwrap_or(false);
            if !parent_exists {
                self.error = Some(format!("the directory of '{text}' does not exist"));
                return false;
            }
            typed
        } else {
            self.error = Some("select a file or type a path".into());
            return false;
        };
        self.choice = Some((purpose, path));
        self.open = false;
        true
    }

    fn cancel(&mut self) {
        self.open = false;
    }
}

/// Open the chooser for `purpose` at `start` (a directory, or a file whose
/// directory is used). `extensions` filters files, without the dot and
/// without regard to case; empty shows every file. `pick_dir` chooses a
/// directory instead of a file.
pub(super) fn open_for(
    state: &mut State,
    purpose: Purpose,
    start: &Path,
    extensions: &[&str],
    pick_dir: bool,
) {
    let dir = if start.is_dir() {
        start.to_path_buf()
    } else if start.is_file() {
        match start.parent() {
            Some(parent) if !parent.as_os_str().is_empty() => parent.to_path_buf(),
            _ => current_dir(),
        }
    } else {
        current_dir()
    };
    state.open = true;
    state.purpose = Some(purpose);
    state.pick_dir = pick_dir;
    state.extensions = extensions
        .iter()
        .map(|e| e.trim_start_matches('.').to_string())
        .collect();
    state.choice = None;
    state.show_dir(dir);
}

fn current_dir() -> PathBuf {
    std::env::current_dir().unwrap_or_else(|_| PathBuf::from("."))
}

/// The path chosen for `purpose` since the last call, if any.
pub(super) fn take_choice(state: &mut State, purpose: Purpose) -> Option<PathBuf> {
    match &state.choice {
        Some((p, _)) if *p == purpose => state.choice.take().map(|(_, path)| path),
        _ => None,
    }
}

/// Entries of `dir`: directories first, then files, each by name without
/// regard to case (equal names in different case by exact name).
pub(super) fn list_dir(
    dir: &Path,
    extensions: &[&str],
    dirs_only: bool,
) -> Result<Vec<Entry>, String> {
    let read = std::fs::read_dir(dir)
        .map_err(|e| format!("cannot read directory '{}': {e}", dir.display()))?;
    let wanted: Vec<String> = extensions
        .iter()
        .map(|e| e.trim_start_matches('.').to_lowercase())
        .collect();
    let mut entries = Vec::new();
    for item in read {
        let item = item.map_err(|e| format!("cannot read directory '{}': {e}", dir.display()))?;
        let name = item.file_name().to_string_lossy().into_owned();
        let is_dir = item.path().is_dir();
        if is_dir {
            entries.push(Entry { name, is_dir });
        } else if !dirs_only {
            let extension = Path::new(&name)
                .extension()
                .map(|e| e.to_string_lossy().to_lowercase())
                .unwrap_or_default();
            if wanted.is_empty() || wanted.contains(&extension) {
                entries.push(Entry { name, is_dir });
            }
        }
    }
    entries.sort_by(|a, b| {
        b.is_dir
            .cmp(&a.is_dir)
            .then_with(|| a.name.to_lowercase().cmp(&b.name.to_lowercase()))
            .then_with(|| a.name.cmp(&b.name))
    });
    Ok(entries)
}

/// Draws the chooser window while it is open.
pub(super) fn render(ctx: &egui::Context, state: &mut State) {
    if !state.open {
        return;
    }
    let title = if state.pick_dir {
        "Choose Folder"
    } else {
        "Choose File"
    };
    let mut open = true;
    egui::Window::new(title)
        .open(&mut open)
        .default_width(460.0)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                if ui.button("Up").clicked() {
                    state.go_up();
                }
                let response = ui
                    .add(egui::TextEdit::singleline(&mut state.typed).desired_width(f32::INFINITY));
                if response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                    state.go_typed();
                }
            });
            if !state.pick_dir && !state.extensions.is_empty() {
                ui.label(format!("Files: *.{}", state.extensions.join(", *.")));
            }
            if let Some(error) = &state.error {
                ui.colored_label(egui::Color32::LIGHT_RED, error);
            }
            let mut entered = None;
            let mut picked = None;
            let mut chosen_file = false;
            egui::ScrollArea::vertical()
                .max_height(280.0)
                .auto_shrink([false, true])
                .show(ui, |ui| {
                    for entry in &state.entries {
                        if entry.is_dir {
                            if ui
                                .selectable_label(false, format!("{}/", entry.name))
                                .clicked()
                            {
                                entered = Some(entry.name.clone());
                            }
                        } else {
                            let on = state.selected.as_deref() == Some(entry.name.as_str());
                            let response = ui.selectable_label(on, &entry.name);
                            if response.clicked() {
                                picked = Some(entry.name.clone());
                            }
                            if response.double_clicked() {
                                chosen_file = true;
                            }
                        }
                    }
                });
            if let Some(name) = entered {
                state.enter(&name);
            }
            if let Some(name) = picked {
                state.typed = state.dir.join(&name).display().to_string();
                state.selected = Some(name);
                state.error = None;
            }
            ui.separator();
            ui.horizontal(|ui| {
                if ui.button("Choose").clicked() || chosen_file {
                    state.choose();
                }
                if ui.button("Cancel").clicked() {
                    state.cancel();
                }
            });
        });
    if !open {
        state.open = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use eframe::egui::{Pos2, Rect, Vec2};

    /// A uniquely named directory that is removed when the guard drops.
    struct TempDir(PathBuf);

    impl TempDir {
        fn new(tag: &str) -> Self {
            let path = std::env::temp_dir().join(format!(
                "structurax_file_browser_{}_{tag}",
                std::process::id()
            ));
            let _ = std::fs::remove_dir_all(&path);
            std::fs::create_dir_all(&path).unwrap();
            Self(path)
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    fn entry(name: &str, is_dir: bool) -> Entry {
        Entry {
            name: name.into(),
            is_dir,
        }
    }

    fn fixture(tag: &str) -> TempDir {
        let dir = TempDir::new(tag);
        std::fs::write(dir.0.join("a.pro"), b"x").unwrap();
        std::fs::write(dir.0.join("b.PRO"), b"x").unwrap();
        std::fs::write(dir.0.join("c.txt"), b"x").unwrap();
        std::fs::create_dir(dir.0.join("sub")).unwrap();
        dir
    }

    fn canon(path: &Path) -> PathBuf {
        std::fs::canonicalize(path).unwrap()
    }

    #[test]
    fn listing_puts_directories_first_and_filters_extensions_without_regard_to_case() {
        let dir = fixture("list");
        assert_eq!(
            list_dir(&dir.0, &["pro"], false).unwrap(),
            vec![
                entry("sub", true),
                entry("a.pro", false),
                entry("b.PRO", false)
            ]
        );
        assert_eq!(
            list_dir(&dir.0, &["PRO", ".txt"], false).unwrap(),
            vec![
                entry("sub", true),
                entry("a.pro", false),
                entry("b.PRO", false),
                entry("c.txt", false)
            ]
        );
        // An empty filter shows every file; the listing is repeatable.
        let all = list_dir(&dir.0, &[], false).unwrap();
        assert_eq!(all.len(), 4);
        assert_eq!(all, list_dir(&dir.0, &[], false).unwrap());
        assert_eq!(
            list_dir(&dir.0, &["pro"], true).unwrap(),
            vec![entry("sub", true)]
        );
    }

    #[test]
    fn names_sort_without_regard_to_case_and_deterministically() {
        let dir = TempDir::new("order");
        for name in ["b.txt", "B.txt", "a.txt", "Z.txt", "m.txt"] {
            std::fs::write(dir.0.join(name), b"x").unwrap();
        }
        std::fs::create_dir(dir.0.join("Zdir")).unwrap();
        std::fs::create_dir(dir.0.join("adir")).unwrap();
        let names: Vec<String> = list_dir(&dir.0, &[], false)
            .unwrap()
            .into_iter()
            .map(|e| e.name)
            .collect();
        assert_eq!(
            names,
            ["adir", "Zdir", "a.txt", "B.txt", "b.txt", "m.txt", "Z.txt"]
        );
    }

    #[test]
    fn a_missing_directory_gives_an_error_naming_it_and_the_state_does_not_panic() {
        let dir = TempDir::new("missing");
        let gone = dir.0.join("nope");
        let error = list_dir(&gone, &[], false).unwrap_err();
        assert!(error.contains("nope"), "{error}");
        // A file is not a directory either.
        std::fs::write(dir.0.join("f.txt"), b"x").unwrap();
        assert!(list_dir(&dir.0.join("f.txt"), &[], false).is_err());
        let mut state = State::default();
        state.extensions = vec![];
        state.show_dir(gone.clone());
        assert!(state.error.as_deref().unwrap().contains("nope"));
        assert!(state.entries.is_empty());
    }

    #[test]
    fn open_for_starts_at_a_directory_the_parent_of_a_file_or_the_current_directory() {
        let dir = fixture("start");
        let mut state = State::default();
        open_for(&mut state, Purpose::SectionFile, &dir.0, &["pro"], false);
        assert!(state.open);
        assert_eq!(state.dir, canon(&dir.0));
        assert_eq!(state.entries.len(), 3);
        open_for(
            &mut state,
            Purpose::SectionFile,
            &dir.0.join("a.pro"),
            &["pro"],
            false,
        );
        assert_eq!(state.dir, canon(&dir.0));
        open_for(
            &mut state,
            Purpose::SectionFile,
            &dir.0.join("does_not_exist.pro"),
            &["pro"],
            false,
        );
        assert_eq!(state.dir, canon(&std::env::current_dir().unwrap()));
        // Directory mode lists directories only.
        open_for(&mut state, Purpose::MaterialLibraryDir, &dir.0, &[], true);
        assert_eq!(state.entries, vec![entry("sub", true)]);
    }

    #[test]
    fn choosing_returns_once_and_only_for_the_purpose_it_was_opened_for() {
        let dir = fixture("choose");
        let mut state = State::default();
        open_for(&mut state, Purpose::SectionFile, &dir.0, &["pro"], false);
        // Nothing highlighted, nothing typed beyond the directory: refused.
        assert!(!state.choose());
        assert!(state.error.is_some());
        state.selected = Some("b.PRO".into());
        assert!(state.choose());
        assert!(!state.open);
        assert_eq!(take_choice(&mut state, Purpose::FunctionFile), None);
        assert_eq!(
            take_choice(&mut state, Purpose::SectionFile),
            Some(canon(&dir.0).join("b.PRO"))
        );
        assert_eq!(take_choice(&mut state, Purpose::SectionFile), None);
        // Cancel closes without a choice.
        open_for(&mut state, Purpose::FunctionFile, &dir.0, &[], false);
        state.cancel();
        assert!(!state.open);
        assert_eq!(take_choice(&mut state, Purpose::FunctionFile), None);
    }

    #[test]
    fn up_enter_typed_paths_and_directory_mode() {
        let dir = fixture("nav");
        let mut state = State::default();
        open_for(&mut state, Purpose::ModelFile, &dir.0, &["pro"], false);
        state.enter("sub");
        assert_eq!(state.dir, canon(&dir.0).join("sub"));
        assert!(state.entries.is_empty());
        state.go_up();
        assert_eq!(state.dir, canon(&dir.0));
        // A typed file highlights it; a typed missing path is an error.
        state.typed = dir.0.join("a.pro").display().to_string();
        state.go_typed();
        assert_eq!(state.selected.as_deref(), Some("a.pro"));
        state.typed = dir.0.join("zzz").display().to_string();
        state.go_typed();
        assert!(state.error.as_deref().unwrap().contains("zzz"));
        // A new file name in an existing directory can be chosen (saving).
        state.selected = None;
        state.typed = dir.0.join("new.s2k").display().to_string();
        assert!(state.choose());
        assert_eq!(
            take_choice(&mut state, Purpose::ModelFile),
            Some(dir.0.join("new.s2k"))
        );
        // Directory mode returns the directory shown.
        open_for(&mut state, Purpose::MaterialLibraryDir, &dir.0, &[], true);
        state.enter("sub");
        assert!(state.choose());
        assert_eq!(
            take_choice(&mut state, Purpose::MaterialLibraryDir),
            Some(canon(&dir.0).join("sub"))
        );
        // The root has no parent: going up there changes nothing.
        let mut root = State::default();
        open_for(&mut root, Purpose::FunctionFile, Path::new("/"), &[], false);
        root.go_up();
        assert_eq!(root.dir, PathBuf::from("/"));
    }

    fn input(time: f64) -> egui::RawInput {
        egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(1000., 800.))),
            time: Some(time),
            ..Default::default()
        }
    }

    fn text_center(out: &egui::FullOutput, label: &str) -> Option<Pos2> {
        out.shapes.iter().find_map(|shape| {
            if let egui::epaint::Shape::Text(text) = &shape.shape {
                (text.galley.job.text == label).then(|| text.pos + text.galley.size() * 0.5)
            } else {
                None
            }
        })
    }

    fn click(ctx: &egui::Context, state: &mut State, pos: Pos2, time: &mut f64) {
        for pressed in [true, false] {
            *time += 0.3;
            let mut raw = input(*time);
            raw.events = vec![
                egui::Event::PointerMoved(pos),
                egui::Event::PointerButton {
                    pos,
                    button: egui::PointerButton::Primary,
                    pressed,
                    modifiers: Default::default(),
                },
            ];
            let _ = ctx.run(raw, |ctx| render(ctx, state));
        }
    }

    fn frame(ctx: &egui::Context, state: &mut State, time: &mut f64) -> egui::FullOutput {
        *time += 0.3;
        ctx.run(input(*time), |ctx| render(ctx, state))
    }

    #[test]
    fn window_lists_the_filtered_entries_and_clicks_navigate_and_choose() {
        let dir = fixture("window");
        let mut state = State::default();
        let ctx = egui::Context::default();
        let mut time = 0.0;
        // Closed: draws nothing.
        assert!(frame(&ctx, &mut state, &mut time).shapes.is_empty());
        open_for(&mut state, Purpose::SectionFile, &dir.0, &["pro"], false);
        let mut out = frame(&ctx, &mut state, &mut time);
        for _ in 0..2 {
            out = frame(&ctx, &mut state, &mut time);
        }
        assert!(text_center(&out, "sub/").is_some());
        assert!(text_center(&out, "a.pro").is_some());
        assert!(text_center(&out, "b.PRO").is_some());
        assert!(text_center(&out, "c.txt").is_none());
        // Click a file, then Choose.
        let pos = text_center(&out, "b.PRO").unwrap();
        click(&ctx, &mut state, pos, &mut time);
        assert_eq!(state.selected.as_deref(), Some("b.PRO"));
        let out = frame(&ctx, &mut state, &mut time);
        let choose = text_center(&out, "Choose").expect("Choose button");
        click(&ctx, &mut state, choose, &mut time);
        assert_eq!(
            take_choice(&mut state, Purpose::SectionFile),
            Some(canon(&dir.0).join("b.PRO"))
        );
        let out = frame(&ctx, &mut state, &mut time);
        assert!(out.shapes.is_empty(), "the window closed");
        // Click a directory: the listing moves into it; Cancel closes.
        open_for(&mut state, Purpose::SectionFile, &dir.0, &["pro"], false);
        let mut out = frame(&ctx, &mut state, &mut time);
        for _ in 0..2 {
            out = frame(&ctx, &mut state, &mut time);
        }
        let pos = text_center(&out, "sub/").unwrap();
        click(&ctx, &mut state, pos, &mut time);
        assert_eq!(state.dir, canon(&dir.0).join("sub"));
        let out = frame(&ctx, &mut state, &mut time);
        assert!(text_center(&out, "a.pro").is_none());
        let cancel = text_center(&out, "Cancel").expect("Cancel button");
        click(&ctx, &mut state, cancel, &mut time);
        assert!(!state.open);
        assert_eq!(take_choice(&mut state, Purpose::SectionFile), None);
    }

    #[test]
    fn window_shows_an_error_line_for_an_unreadable_directory() {
        let mut state = State::default();
        let ctx = egui::Context::default();
        let mut time = 0.0;
        let gone = std::env::temp_dir().join(format!(
            "structurax_file_browser_{}_never_created",
            std::process::id()
        ));
        state.open = true;
        state.purpose = Some(Purpose::FunctionFile);
        state.show_dir(gone);
        let mut out = frame(&ctx, &mut state, &mut time);
        for _ in 0..2 {
            out = frame(&ctx, &mut state, &mut time);
        }
        let shown = out.shapes.iter().any(|s| {
            matches!(&s.shape, egui::epaint::Shape::Text(t)
                if t.galley.job.text.contains("cannot read directory"))
        });
        assert!(shown);
    }
}
