//! The "Messages" window (D54): the status messages of the session, newest
//! first, so that a message longer than the status line (the reason a pushover
//! was refused, the list of diaphragms a command assigned) can be read and
//! copied. The status line shows one message cut to its width; its tooltip has
//! the whole text.

use eframe::egui::{self, Color32, RichText};
use std::collections::VecDeque;

/// Messages kept; the oldest is dropped when a new one arrives.
pub(super) const MAX_MESSAGES: usize = 200;

#[derive(Debug, Clone, PartialEq)]
pub(super) struct Entry {
    /// Counts up from 1 over the session (it keeps counting past the limit).
    pub number: u64,
    pub text: String,
    pub error: bool,
}

#[derive(Debug, Clone, Default)]
pub(super) struct State {
    pub open: bool,
    entries: VecDeque<Entry>,
    last: String,
    count: u64,
}

/// Words that mark the message of a refused or failed command. The commands
/// build their messages as plain text ("Analysis Error: ...", "Not done: ...",
/// "Model is locked ..."), so the mark is read from the text.
const ERROR_WORDS: [&str; 13] = [
    "error", "cannot", "can't", "refus", "failed", "not done", "unable", "invalid", "is locked",
    "not found", "no results", "must be", "unknown",
];

/// Openings of the message of a command that was done. Such a message may
/// carry warnings that use the words above ("Opened f.sdb - Import warnings:
/// ... saving cannot preserve ...") and is not an error.
const DONE_OPENINGS: [&str; 8] =
    ["opened ", "saved ", "loaded ", "added ", "assigned ", "created ", "generated ", "video written"];

/// Whether `text` reads as the message of a command that was refused or failed.
pub(super) fn is_error(text: &str) -> bool {
    let lower = text.to_lowercase();
    if DONE_OPENINGS.iter().any(|w| lower.starts_with(w)) {
        return false;
    }
    ERROR_WORDS.iter().any(|w| lower.contains(w))
}

impl State {
    /// Records the status message of this frame when it differs from the last
    /// one recorded (an empty message is not recorded).
    pub(super) fn observe(&mut self, status: &str) {
        if status.is_empty() || status == self.last {
            return;
        }
        self.last = status.to_string();
        self.count += 1;
        self.entries.push_back(Entry {
            number: self.count,
            text: status.to_string(),
            error: is_error(status),
        });
        while self.entries.len() > MAX_MESSAGES {
            self.entries.pop_front();
        }
    }

    /// The messages, newest first.
    pub(super) fn newest_first(&self) -> impl Iterator<Item = &Entry> {
        self.entries.iter().rev()
    }

    pub(super) fn len(&self) -> usize {
        self.entries.len()
    }
}

/// The window. Returns the text of the message the user asked to copy; the
/// text goes to the clipboard.
pub(super) fn window(ctx: &egui::Context, state: &mut State) {
    if !state.open {
        return;
    }
    let mut open = true;
    let mut copy: Option<String> = None;
    egui::Window::new("Messages")
        .open(&mut open)
        .default_size([640.0, 360.0])
        .vscroll(false)
        .show(ctx, |ui| {
            ui.label(format!(
                "The last {} status messages of this session, newest first.",
                MAX_MESSAGES
            ));
            ui.separator();
            if state.len() == 0 {
                ui.label("No messages yet.");
            }
            egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
                for entry in state.newest_first() {
                    ui.horizontal_top(|ui| {
                        ui.label(RichText::new(format!("#{}", entry.number)).weak());
                        if entry.error {
                            ui.label(RichText::new("Error").strong().color(Color32::LIGHT_RED));
                        } else {
                            ui.label("Info");
                        }
                        if ui.small_button("Copy").clicked() {
                            copy = Some(entry.text.clone());
                        }
                        ui.add(egui::Label::new(entry.text.as_str()).wrap());
                    });
                    ui.separator();
                }
            });
        });
    state.open = open;
    if let Some(text) = copy {
        ctx.copy_text(text);
    }
}
