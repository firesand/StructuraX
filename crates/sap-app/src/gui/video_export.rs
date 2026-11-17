//! Video export (File > Create Video): software rasteriser, AVI/BMP writers and
//! the Create Video form.
//!
//! The vendor's File > Create Video writes an `.avi` of an animated mode shape (Cyclic
//! Animation) or of the steps of a time history (Multi Step Animation). Here the frames are
//! produced by painting the view into a private `egui::Context`, tessellating it and
//! rasterising the triangles in software ([`raster`]); the AVI ([`avi`]) is the vendor's
//! default "Full Frames (Uncompressed)", written by hand (no encoder, no image crate).
//!
//! Limits: the video matches the screen in geometry and colour values, not pixel for pixel
//! (no GPU dithering, no display colour management, no compressor choice).

mod avi;
mod raster;
#[cfg(test)]
mod tests;

use super::contours::StepSelection;
use super::result_display::scaling;
use eframe::egui;
use sap_model::SapModel;

pub(super) use raster::FontAtlas;

/// A row-major RGB image, top row first.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct RgbImage {
    pub width: usize,
    pub height: usize,
    pub pixels: Vec<[u8; 3]>,
}

/// Rasterises tessellated egui primitives (in points; `pixels_per_point` scales them) over
/// `background`. Refuses a mesh that uses a texture other than the font texture, a paint
/// callback, or malformed indices.
pub(super) fn rasterize(
    prims: &[egui::ClippedPrimitive],
    atlas: &FontAtlas,
    width: usize,
    height: usize,
    pixels_per_point: f32,
    background: egui::Color32,
) -> Result<RgbImage, String> {
    raster::rasterize(prims, atlas, width, height, pixels_per_point, background)
}

/// An uncompressed AVI of `frames` at `fps` (2..4096 pixels per side, 1..120 fps, at least
/// one frame, all frames of one size, at most 1 GiB).
#[allow(dead_code)] // Used by the tests and kept as the plain entry point; the menu goes through `record` and `save`.
pub(super) fn avi_bytes(frames: &[RgbImage], fps: u32) -> Result<Vec<u8>, String> {
    avi::avi_bytes(frames, fps)
}

/// A 24 bit bitmap file of one frame.
#[allow(dead_code)] // Used by the tests and kept as the plain entry point; the menu goes through `record` and `save`.
pub(super) fn bmp_bytes(image: &RgbImage) -> Vec<u8> {
    avi::bmp_bytes(image)
}

#[derive(Clone, Debug, PartialEq)]
pub(super) enum VideoKind {
    /// Steps of a time history (or the saved states of a multi-step static case) from
    /// `start` to `end` at `increment`; the values are times (history cases) or load
    /// factors (nonlinear static cases).
    MultiStep {
        case: String,
        start: f64,
        end: f64,
        increment: f64,
    },
    /// An animated mode shape: `mode` is the mode NUMBER (1-based, as in the modal tables).
    Cyclic {
        case: String,
        mode: usize,
        cycles: u32,
        frames_per_cycle: u32,
    },
}

#[derive(Clone, Debug, PartialEq)]
pub(super) struct VideoSettings {
    pub kind: VideoKind,
    pub width: usize,
    pub height: usize,
    pub fps: u32,
    pub magnification: f64,
    pub path: String,
}

/// What one frame shows: `step_idx` (zero-based index into the case's saved steps) for a
/// multi-step video, else `None`; `phase` is the cyclic phase in radians, `2 pi k / frames
/// per cycle` (the displacement is the mode shape times `sin(phase)`), and 0 for a
/// multi-step video.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct FramePose {
    pub step_idx: Option<usize>,
    pub phase: f64,
}

/// More frames than this are refused (a video of this length is not an animation).
pub(super) const MAX_FRAMES: usize = 100_000;

/// The times (or load factors) of the saved steps of `case`, or why it has none.
fn saved_steps(model: &SapModel, case: &str) -> Result<Vec<f64>, String> {
    let cr = model
        .case_results
        .get(case)
        .ok_or_else(|| format!("case '{case}' has no results; run the analysis first"))?;
    if !cr.time_steps.is_empty() {
        Ok(cr.time_steps.clone())
    } else if !cr.static_steps.is_empty() {
        Ok(cr.static_steps.iter().map(|s| s.load_factor).collect())
    } else {
        Err(format!(
            "case '{case}' has no saved steps; a multi-step video needs a time history or a \
             multi-step static case"
        ))
    }
}

/// Number of frames of a multi-step video: `floor((end - start) / increment + 1e-9) + 1`
/// (frame `k` is at `t = start + k increment`, the last `t <= end`). The caller has checked
/// `start < end` and `increment > 0`.
pub(super) fn multistep_frame_count(start: f64, end: f64, increment: f64) -> f64 {
    ((end - start) / increment + 1e-9).floor() + 1.0
}

/// Checks the settings that are independent of the kind.
fn check_common(s: &VideoSettings) -> Result<(), String> {
    if !(avi::MIN_SIDE..=avi::MAX_SIDE).contains(&s.width)
        || !(avi::MIN_SIDE..=avi::MAX_SIDE).contains(&s.height)
    {
        return Err(format!(
            "frame size {} x {} pixels is outside {}..{} per side",
            s.width,
            s.height,
            avi::MIN_SIDE,
            avi::MAX_SIDE
        ));
    }
    if !(avi::MIN_FPS..=avi::MAX_FPS).contains(&s.fps) {
        return Err(format!(
            "{} frames per second is outside {}..{}",
            s.fps,
            avi::MIN_FPS,
            avi::MAX_FPS
        ));
    }
    if !(s.magnification.is_finite() && s.magnification > 0.0) {
        return Err(format!(
            "magnification {} must be a positive number",
            s.magnification
        ));
    }
    if s.path.trim().is_empty() {
        return Err("the AVI file name is empty".into());
    }
    Ok(())
}

/// The poses of the video, or the reason it cannot be recorded.
///
/// Multi-step: `n = floor((end - start) / increment + 1e-9) + 1` frames, frame `k` at
/// `t_k = start + k increment` shows the saved step whose time (load factor) is nearest to
/// `t_k`. Cyclic: `n = cycles * frames_per_cycle` frames, frame `k` has phase
/// `2 pi (k mod frames_per_cycle) / frames_per_cycle`. The video length is `n / fps` seconds.
pub(super) fn frame_plan(model: &SapModel, s: &VideoSettings) -> Result<Vec<FramePose>, String> {
    check_common(s)?;
    let poses: Vec<FramePose> = match &s.kind {
        VideoKind::MultiStep {
            case,
            start,
            end,
            increment,
        } => {
            let times = saved_steps(model, case)?;
            if !(start.is_finite() && end.is_finite() && increment.is_finite()) {
                return Err("start, end and increment must be finite numbers".into());
            }
            if end <= start {
                return Err(format!(
                    "the time range is empty or reversed: start {start} is not below end {end}"
                ));
            }
            if *increment <= 0.0 {
                return Err(format!("the increment {increment} must be positive"));
            }
            let (first, last) = (times[0], times[times.len() - 1]);
            let tol = 1e-9 * (last - first).abs().max(1e-300);
            if *start < first - tol || *end > last + tol {
                return Err(format!(
                    "the range {start} to {end} is outside the saved steps of case '{case}' \
                     ({first} to {last})"
                ));
            }
            let n = multistep_frame_count(*start, *end, *increment);
            if n > MAX_FRAMES as f64 {
                return Err(format!("{n} frames exceed the limit of {MAX_FRAMES}"));
            }
            (0..n as usize)
                .map(|k| {
                    let t = start + k as f64 * increment;
                    let mut best = 0;
                    for (i, ti) in times.iter().enumerate() {
                        if (ti - t).abs() < (times[best] - t).abs() {
                            best = i;
                        }
                    }
                    FramePose {
                        step_idx: Some(best),
                        phase: 0.0,
                    }
                })
                .collect()
        }
        VideoKind::Cyclic {
            case,
            mode,
            cycles,
            frames_per_cycle,
        } => {
            let cr = model
                .case_results
                .get(case)
                .ok_or_else(|| format!("case '{case}' has no results; run the analysis first"))?;
            if cr.modes.is_empty() {
                return Err(format!(
                    "case '{case}' has no mode shapes; a cyclic video animates a mode of a \
                     modal case"
                ));
            }
            if !cr.modes.iter().any(|m| m.mode_num == *mode) {
                return Err(format!(
                    "mode {mode} is not a mode of case '{case}' (modes 1 to {})",
                    cr.modes.len()
                ));
            }
            if *cycles == 0 {
                return Err("the number of cycles must be at least 1".into());
            }
            if *frames_per_cycle < 2 {
                return Err("frames per cycle must be at least 2".into());
            }
            let n = *cycles as u64 * *frames_per_cycle as u64;
            if n > MAX_FRAMES as u64 {
                return Err(format!("{n} frames exceed the limit of {MAX_FRAMES}"));
            }
            let f = *frames_per_cycle as usize;
            (0..n as usize)
                .map(|k| FramePose {
                    step_idx: None,
                    phase: 2.0 * std::f64::consts::PI * (k % f) as f64 / f as f64,
                })
                .collect()
        }
    };
    let bytes = avi::file_bytes(s.width, s.height, poses.len());
    if bytes > avi::MAX_FILE_BYTES {
        return Err(format!(
            "the video would be {:.2} GiB; files above 1 GiB are refused (no OpenDML index); \
             use a smaller frame size or fewer frames",
            bytes as f64 / (1u64 << 30) as f64
        ));
    }
    Ok(poses)
}

/// Length of the video in seconds: frames over frames per second.
pub(super) fn duration_seconds(frames: usize, fps: u32) -> f64 {
    frames as f64 / fps as f64
}

/// Paints each pose with `paint` into a private `egui::Context` of `s.width x s.height`
/// points (pixels_per_point 1), rasterises it and returns the AVI bytes.
///
/// The first frame is painted twice (the first pass is discarded) so that egui's sizing pass
/// of new windows and its font atlas are settled; the background of every frame is the
/// context's `panel_fill`, which is what eframe clears the screen with.
pub(super) fn record(
    s: &VideoSettings,
    poses: &[FramePose],
    paint: &mut dyn FnMut(&egui::Context, &FramePose),
) -> Result<Vec<u8>, String> {
    check_common(s)?;
    if poses.is_empty() {
        return Err("a video needs at least one frame".into());
    }
    let mut builder = avi::AviBuilder::new(s.width, s.height, s.fps)?;
    let ctx = egui::Context::default();
    let mut atlas = FontAtlas::default();
    let screen = egui::Rect::from_min_size(
        egui::Pos2::ZERO,
        egui::vec2(s.width as f32, s.height as f32),
    );
    let input = |k: usize| egui::RawInput {
        screen_rect: Some(screen),
        time: Some(k as f64 / s.fps as f64),
        ..Default::default()
    };
    // warm-up pass
    let warm = ctx.run(input(0), |c| paint(c, &poses[0]));
    atlas.apply(&warm.textures_delta)?;
    for (k, pose) in poses.iter().enumerate() {
        let out = ctx.run(input(k), |c| paint(c, pose));
        atlas.apply(&out.textures_delta)?;
        let ppp = out.pixels_per_point;
        let prims = ctx.tessellate(out.shapes, ppp);
        let bg = ctx.style().visuals.panel_fill;
        let image = rasterize(&prims, &atlas, s.width, s.height, ppp, bg)
            .map_err(|e| format!("frame {}: {e}", k + 1))?;
        builder.push(&image)?;
    }
    builder.finish()
}

/// Writes the finished video to `s.path`; the file is created or replaced only when the
/// bytes are complete, so a failed recording leaves no half-written file.
pub(super) fn save(s: &VideoSettings, bytes: &[u8]) -> Result<(), String> {
    let path = std::path::Path::new(s.path.trim());
    let tmp = path.with_extension("avi.part");
    std::fs::write(&tmp, bytes).map_err(|e| format!("cannot write '{}': {e}", tmp.display()))?;
    std::fs::rename(&tmp, path).map_err(|e| {
        let _ = std::fs::remove_file(&tmp);
        format!("cannot write '{}': {e}", path.display())
    })
}

// ---------------------------------------------------------------------------------------
// The form

/// State of the Create Video form.
pub(super) struct VideoForm {
    open: bool,
    cyclic: bool,
    case: String,
    start: f64,
    end: f64,
    increment: f64,
    mode: usize,
    cycles: u32,
    frames_per_cycle: u32,
    magnification: f64,
    /// The user typed a magnification: the automatic one no longer follows
    /// the case (D45).
    magnification_edited: bool,
    /// Case, mode and kind the automatic magnification was computed for.
    auto_key: Option<(String, bool, usize)>,
    fps: u32,
    width: usize,
    height: usize,
    path: String,
    error: Option<String>,
}

impl Default for VideoForm {
    fn default() -> Self {
        Self {
            open: false,
            cyclic: false,
            case: String::new(),
            start: 0.0,
            end: 0.0,
            increment: 0.0,
            mode: 1,
            cycles: 3,
            frames_per_cycle: 20,
            magnification: 10.0,
            magnification_edited: false,
            auto_key: None,
            fps: 10,
            width: 640,
            height: 480,
            path: "video.avi".into(),
            error: None,
        }
    }
}

/// The automatic deformed-shape factor (`result_display::scaling::auto_scale_for`) of
/// `case`: of mode number `mode` for a cyclic video, of the saved step with the
/// largest translation for a multi-step one. `None` for a case without results.
pub(super) fn auto_magnification(
    model: &SapModel,
    case: &str,
    cyclic: bool,
    mode: usize,
) -> Option<f64> {
    let cr = model.case_results.get(case)?;
    let (min, max) = super::view::model_bounds(model);
    let sel = if cyclic {
        let mode_idx = cr.modes.iter().position(|m| m.mode_num == mode)?;
        StepSelection { mode_idx, use_step: false, step_idx: 0 }
    } else {
        let steps = cr.time_steps.len().max(cr.static_steps.len());
        let sel_of = |i| StepSelection { mode_idx: 0, use_step: true, step_idx: i };
        let step_idx = (0..steps)
            .map(|i| (i, scaling::max_translation(model, cr, sel_of(i))))
            .max_by(|a, b| a.1.total_cmp(&b.1))?
            .0;
        sel_of(step_idx)
    };
    Some(scaling::auto_scale_for(model, cr, sel, min, max))
}

/// Cases that can be recorded: with saved steps (multi-step) or with mode shapes (cyclic).
pub(super) fn eligible_cases(model: &SapModel, cyclic: bool) -> Vec<String> {
    model
        .case_results
        .iter()
        .filter(|(_, cr)| {
            if cyclic {
                !cr.modes.is_empty()
            } else {
                !cr.time_steps.is_empty() || !cr.static_steps.is_empty()
            }
        })
        .map(|(n, _)| n.clone())
        .collect()
}

impl VideoForm {
    /// Opens the form for a multi-step (`cyclic == false`) or cyclic video, choosing the
    /// first recordable case and filling its range.
    pub(super) fn open(&mut self, model: &SapModel, cyclic: bool) {
        self.open = true;
        self.cyclic = cyclic;
        self.error = None;
        let cases = eligible_cases(model, cyclic);
        self.case = cases.first().cloned().unwrap_or_default();
        self.reset_case(model);
        self.magnification_edited = false;
        self.auto_key = None;
        self.refresh_auto_magnification(model);
    }

    /// The user's magnification (the box of the form calls this when the value
    /// is edited): from now on it is the user's, not the automatic one.
    pub(super) fn set_magnification(&mut self, value: f64) {
        self.magnification = value;
        self.magnification_edited = true;
    }

    #[cfg(test)]
    pub(super) fn magnification_value(&self) -> f64 {
        self.magnification
    }

    #[cfg(test)]
    pub(super) fn set_mode(&mut self, mode: usize) {
        self.mode = mode;
    }

    /// While the user has not edited the magnification, it is the automatic
    /// deformed-shape factor of the chosen case and of its mode (cyclic) or of
    /// the saved step with the largest displacement (multi-step), so that no
    /// frame of the video leaves the picture. Recomputed when the case, the
    /// kind or the mode changes.
    pub(super) fn refresh_auto_magnification(&mut self, model: &SapModel) {
        if self.magnification_edited {
            return;
        }
        let key = (self.case.clone(), self.cyclic, if self.cyclic { self.mode } else { 0 });
        if self.auto_key.as_ref() == Some(&key) {
            return;
        }
        self.auto_key = Some(key);
        if let Some(f) = auto_magnification(model, &self.case, self.cyclic, self.mode) {
            self.magnification = f;
        }
    }

    #[allow(dead_code)] // Used by the tests and kept as the plain entry point; the menu goes through `record` and `save`.
    pub(super) fn is_open(&self) -> bool {
        self.open
    }

    #[allow(dead_code)] // Used by the tests and kept as the plain entry point; the menu goes through `record` and `save`.
    pub(super) fn close(&mut self) {
        self.open = false;
    }

    fn reset_case(&mut self, model: &SapModel) {
        if self.cyclic {
            self.mode = model
                .case_results
                .get(&self.case)
                .and_then(|c| c.modes.first())
                .map_or(1, |m| m.mode_num);
        } else if let Ok(t) = saved_steps(model, &self.case) {
            self.start = t[0];
            self.end = t[t.len() - 1];
            self.increment = if t.len() > 1 { t[1] - t[0] } else { 1.0 };
        }
    }

    fn kind(&self) -> VideoKind {
        if self.cyclic {
            VideoKind::Cyclic {
                case: self.case.clone(),
                mode: self.mode,
                cycles: self.cycles,
                frames_per_cycle: self.frames_per_cycle,
            }
        } else {
            VideoKind::MultiStep {
                case: self.case.clone(),
                start: self.start,
                end: self.end,
                increment: self.increment,
            }
        }
    }

    fn settings(&self) -> VideoSettings {
        VideoSettings {
            kind: self.kind(),
            width: self.width,
            height: self.height,
            fps: self.fps,
            magnification: self.magnification,
            path: self.path.clone(),
        }
    }
}

/// The settings of the form when they can be recorded, else the reason.
pub(super) fn form_settings(form: &VideoForm, model: &SapModel) -> Result<VideoSettings, String> {
    let s = form.settings();
    frame_plan(model, &s)?;
    Ok(s)
}

/// The line under the fields: frames, length and size of the file, or the reason the
/// settings cannot be recorded.
pub(super) fn summary(form: &VideoForm, model: &SapModel) -> String {
    let s = form.settings();
    match frame_plan(model, &s) {
        Ok(p) => format!(
            "{} frames, {:.2} s at {} frames per second, about {:.1} MB",
            p.len(),
            duration_seconds(p.len(), s.fps),
            s.fps,
            avi::file_bytes(s.width, s.height, p.len()) as f64 / 1.0e6
        ),
        Err(e) => format!("Cannot record: {e}"),
    }
}

/// What the form says about the fidelity of the video.
pub(super) const FIDELITY_NOTE: &str = "The video is drawn by StructuraX's own software \
    renderer from the same shapes as the screen: geometry and colour values match the screen, \
    not pixel for pixel (no GPU dithering or display colour management). It is written as an \
    uncompressed AVI (Full Frames); no compressor can be chosen.";

/// The Create Video form; returns the settings when the user presses OK.
///
/// Fields offered: the case; multi-step: start, end and increment (times, or load factors
/// of a multi-step static case); cyclic: mode number, cycles and frames per cycle;
/// magnification, frames per second, frame size in pixels and the file name.
pub(super) fn window(
    ctx: &egui::Context,
    form: &mut VideoForm,
    model: &SapModel,
) -> Option<VideoSettings> {
    if !form.open {
        return None;
    }
    let mut result = None;
    let mut close = false;
    let title = if form.cyclic {
        "Create Cyclic Animation Video"
    } else {
        "Create Multi-Step Animation Video"
    };
    egui::Window::new(title)
        .collapsible(false)
        .resizable(false)
        .show(ctx, |ui| {
            let cases = eligible_cases(model, form.cyclic);
            if cases.is_empty() {
                ui.label(if form.cyclic {
                    "No case with mode shapes: run a modal analysis first."
                } else {
                    "No case with saved steps: run a time history or multi-step static case first."
                });
            }
            egui::Grid::new("video_form_grid")
                .num_columns(2)
                .show(ui, |ui| {
                    ui.label("Load case");
                    let before = form.case.clone();
                    egui::ComboBox::from_id_salt("video_case")
                        .selected_text(form.case.clone())
                        .show_ui(ui, |ui| {
                            for c in &cases {
                                ui.selectable_value(&mut form.case, c.clone(), c);
                            }
                        });
                    if form.case != before {
                        form.reset_case(model);
                    }
                    form.refresh_auto_magnification(model);
                    ui.end_row();
                    if form.cyclic {
                        ui.label("Mode number");
                        ui.add(egui::DragValue::new(&mut form.mode).range(1..=100_000));
                        ui.end_row();
                        ui.label("Number of cycles");
                        ui.add(egui::DragValue::new(&mut form.cycles).range(1..=10_000));
                        ui.end_row();
                        ui.label("Frames per cycle");
                        ui.add(egui::DragValue::new(&mut form.frames_per_cycle).range(2..=10_000));
                        ui.end_row();
                    } else {
                        ui.label("Start (time)");
                        ui.add(egui::DragValue::new(&mut form.start).speed(0.01));
                        ui.end_row();
                        ui.label("End (time)");
                        ui.add(egui::DragValue::new(&mut form.end).speed(0.01));
                        ui.end_row();
                        ui.label("Time increment");
                        ui.add(egui::DragValue::new(&mut form.increment).speed(0.001));
                        ui.end_row();
                    }
                    ui.label("Magnification");
                    let mut value = form.magnification;
                    let response = ui
                        .add(egui::DragValue::new(&mut value).speed(0.5))
                        .on_hover_text(
                            "Automatic until you edit it: the largest displacement of the \
                             recorded shapes is drawn as 10 % of the model diagonal.",
                        );
                    if response.changed() {
                        form.set_magnification(value);
                    }
                    ui.end_row();
                    ui.label("Frames per second");
                    ui.add(egui::DragValue::new(&mut form.fps).range(avi::MIN_FPS..=avi::MAX_FPS));
                    ui.end_row();
                    ui.label("Frame width (pixels)");
                    ui.add(
                        egui::DragValue::new(&mut form.width).range(avi::MIN_SIDE..=avi::MAX_SIDE),
                    );
                    ui.end_row();
                    ui.label("Frame height (pixels)");
                    ui.add(
                        egui::DragValue::new(&mut form.height).range(avi::MIN_SIDE..=avi::MAX_SIDE),
                    );
                    ui.end_row();
                    ui.label("AVI file name");
                    ui.text_edit_singleline(&mut form.path);
                    ui.end_row();
                });
            ui.separator();
            ui.label(summary(form, model));
            if let Some(e) = &form.error {
                ui.colored_label(ui.visuals().error_fg_color, e);
            }
            ui.add_sized(
                [420.0, 0.0],
                egui::Label::new(egui::RichText::new(FIDELITY_NOTE).small()).wrap(),
            );
            ui.horizontal(|ui| {
                if ui.button("OK").clicked() {
                    match form_settings(form, model) {
                        Ok(s) => {
                            result = Some(s);
                            close = true;
                        }
                        Err(e) => form.error = Some(e),
                    }
                }
                if ui.button("Cancel").clicked() {
                    close = true;
                }
            });
        });
    if close {
        form.open = false;
        form.error = None;
    }
    result
}
