//! File > Create Video: the application's side of the video export. The form,
//! the frame plan, the rasteriser and the AVI writer are in `video_export`;
//! this file refuses what must not be recorded, sets the display to the pose
//! of each frame, paints the central viewport (no menus) into the context the
//! recorder gives, writes the file and reports it on the status line.

use eframe::egui;
use sap_model::SapModel;

use super::video_export::{self, FramePose, VideoKind, VideoSettings};
use super::{DisplayMode, SapGuiApp};

/// The display settings of one frame.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct PoseDisplay {
    pub case: String,
    /// Zero-based.
    pub mode_idx: usize,
    pub use_step: bool,
    pub step_idx: usize,
    /// The cyclic animation: the shape is the mode shape times `sin(phase)`.
    pub animate: bool,
    pub phase: f32,
    /// User-defined scale factor (the form's magnification).
    pub scale: f32,
}

/// What a frame at `pose` of a video with `s` shows. The mode number of a
/// cyclic video is 1-based, the display's index zero-based; a multi-step
/// frame shows saved step `pose.step_idx`.
pub(super) fn pose_display(s: &VideoSettings, pose: &FramePose) -> PoseDisplay {
    let scale = s.magnification as f32;
    match &s.kind {
        VideoKind::MultiStep { case, .. } => PoseDisplay {
            case: case.clone(),
            mode_idx: 0,
            use_step: true,
            step_idx: pose.step_idx.unwrap_or(0),
            animate: false,
            phase: 0.0,
            scale,
        },
        VideoKind::Cyclic { case, mode, .. } => PoseDisplay {
            case: case.clone(),
            mode_idx: mode.saturating_sub(1),
            use_step: false,
            step_idx: 0,
            animate: true,
            phase: pose.phase as f32,
            scale,
        },
    }
}

/// Refuses, before anything is recorded, a video of a model that has no
/// results (an unlocked model has none), or of a case without results.
pub(super) fn precheck(model: &SapModel, s: &VideoSettings) -> Result<(), String> {
    let case = match &s.kind {
        VideoKind::MultiStep { case, .. } | VideoKind::Cyclic { case, .. } => case,
    };
    if !model.is_locked {
        return Err("Create Video needs the analysis results: the model is not locked; run the analysis first.".into());
    }
    if !model.case_results.contains_key(case) {
        return Err(format!("Create Video: '{case}' has no results; run the analysis first."));
    }
    if !(s.magnification.is_finite() && s.magnification > 0.0) {
        return Err("Create Video: the magnification must be a positive number.".into());
    }
    Ok(())
}

impl SapGuiApp {
    /// Draws the form while it is open and records the video when OK is pressed.
    pub(super) fn render_video_form(&mut self, ctx: &egui::Context) {
        if let Some(settings) = video_export::window(ctx, &mut self.video_form, &self.model) {
            self.status_msg = self.create_video(&settings);
        }
    }

    /// Records `s` and returns the status line: the file and the frame count,
    /// or why nothing was written.
    pub(super) fn create_video(&mut self, s: &VideoSettings) -> String {
        if let Err(e) = precheck(&self.model, s) {
            return e;
        }
        let poses = match video_export::frame_plan(&self.model, s) {
            Ok(p) => p,
            Err(e) => return format!("Create Video: {e}"),
        };
        // The display the user had comes back after the recording.
        let saved = (
            self.active_case.clone(),
            self.display_mode,
            self.active_mode_idx,
            self.use_time_step,
            self.active_step_idx,
            self.animate_deformed,
            self.anim_phase,
            self.deform_scale,
            self.deform_auto,
            self.deform_auto_written,
            self.results.stepping,
        );
        let recorded = {
            let mut paint = |ctx: &egui::Context, pose: &FramePose| {
                let d = pose_display(s, pose);
                self.active_case = d.case;
                self.active_mode_idx = d.mode_idx;
                self.use_time_step = d.use_step;
                self.active_step_idx = d.step_idx;
                self.display_mode = DisplayMode::Deformed;
                self.animate_deformed = d.animate;
                self.anim_phase = d.phase;
                self.results.stepping = false;
                self.deform_auto = false;
                self.deform_scale = d.scale;
                egui::CentralPanel::default()
                    .frame(egui::Frame::none())
                    .show(ctx, |ui| self.render_viewport(ui, 0, "3D Perspective View"));
            };
            video_export::record(s, &poses, &mut paint)
        };
        (
            self.active_case,
            self.display_mode,
            self.active_mode_idx,
            self.use_time_step,
            self.active_step_idx,
            self.animate_deformed,
            self.anim_phase,
            self.deform_scale,
            self.deform_auto,
            self.deform_auto_written,
            self.results.stepping,
        ) = saved;
        let bytes = match recorded {
            Ok(b) => b,
            Err(e) => return format!("Create Video: {e}"),
        };
        match video_export::save(s, &bytes) {
            Ok(()) => format!("Video written ({} frames): {}", poses.len(), s.path.trim()),
            Err(e) => format!("Create Video: {e}"),
        }
    }
}
