//! Row 3.8: File > Create Video of a small modal history (multi-step) and of a mode
//! (cyclic). The file is read back from its bytes: the chunk count, the frame size, and the
//! position of a joint marker in every frame; `ffprobe` and `ffmpeg` read the same file as an
//! independent decoder when they are installed (skipped silently otherwise).
//!
//! Model (this file, `storey`): the single storey of Part 3 (kN, m): four columns of
//! 0.3 m x 0.3 m (EI = 2e8 x 6.75e-4), h = 4 m, fixed at the base, the roof joints tied by a
//! rigid diaphragm and held against UZ, RX and RY, roof mass 4 x 625 t in X and Y:
//!   k = 4 x 12 EI / h^3 = 101250 kN/m, omega^2 = k / M = 40.5.
//! Ground acceleration: a step of a0 = 2 m/s2 (the constant record 1.0 with scale factor 2),
//! damping xi = 0.05: u(t) = -(a0/w^2) [1 - e^{-xi w t} (cos wd t + xi/s sin wd t)], s = sqrt(1 - xi^2).

use super::ext::{analyze, menu};
use super::harness::Run;
use crate::gui::view::ViewTransform;
use sap_model::{
    DynamicFunction, FramePropType, FrameSection, LoadCase, LoadCaseType, LoadPattern,
    LoadPatternType, Material, SapModel,
};
use sap_units::Units;
use std::path::{Path, PathBuf};

const DT: f64 = 0.02;
const W: usize = 160;
const H: usize = 120;
const FRAME_BYTES: usize = W * 3 * H; // rows of 160 x 3 = 480 bytes, a multiple of 4: no padding

pub(super) fn storey() -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    m.load_cases.clear();
    m.load_patterns.clear();
    m.frame_sections.clear();
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.e = 2.0e8;
    mat.u = 0.3;
    mat.g = 2.0e8 / 2.6;
    mat.unit_weight = 0.0;
    mat.unit_mass = 0.0;
    m.materials.insert("MAT".into(), mat);
    let side: f64 = 0.3;
    m.frame_sections.insert(
        "COL".into(),
        FrameSection {
            name: "COL".into(),
            material: "MAT".into(),
            prop_type: FramePropType::General,
            area: side * side,
            i33: side.powi(4) / 12.0,
            // 1.5 times stiffer for sway in the other horizontal direction: no degenerate pair of
            // modes (the direction of a mode of a repeated eigenvalue is not defined).
            i22: 1.5 * side.powi(4) / 12.0,
            torsional_j: 0.141 * side.powi(4),
            as2: 0.0,
            as3: 0.0,
            modifiers: [1.0; 8],
            ..FrameSection::default()
        },
    );
    for (k, (x, y)) in [(-3.0, -3.0), (3.0, -3.0), (3.0, 3.0), (-3.0, 3.0)].into_iter().enumerate() {
        let (top, bottom) = (format!("T{}", k + 1), format!("B{}", k + 1));
        m.add_point(top.clone(), x, y, 4.0);
        m.add_point(bottom.clone(), x, y, 0.0);
        m.points[&bottom].restraint = [true; 6];
        m.points[&top].restraint = [false, false, true, true, true, false];
        m.points[&top].mass = [625.0, 625.0, 0.0, 0.0, 0.0, 0.0];
        m.points[&top].selected = true;
        m.add_frame(format!("C{}", k + 1), bottom, top, "COL");
    }
    m.assign_diaphragm("DIAPH", true).unwrap();
    for p in m.points.values_mut() {
        p.selected = false;
    }
    m.load_patterns.insert(
        "P".into(),
        LoadPattern { name: "P".into(), pat_type: LoadPatternType::Other, self_wt_multiplier: 0.0 },
    );
    m.functions.insert(
        "STEP".into(),
        DynamicFunction {
            name: "STEP".into(),
            is_time_history: true,
            damping_ratio: 0.0,
            points: (0..3000).map(|k| (k as f64 * DT, 1.0)).collect(),
        },
    );
    m.load_cases.insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 3));
    let mut case = LoadCase::linear_static("HIST", "P", 1.0);
    case.case_type = LoadCaseType::LinearHistory;
    case.loads.clear();
    case.modal_case = Some("MODAL".into());
    case.modal_damping = 0.05;
    case.time_step_dt = DT;
    case.num_time_steps = 500;
    case.history_loads = vec![("STEP".into(), "UX".into(), 2.0)];
    m.load_cases.insert("HIST".into(), case);
    m
}

/// The roof's relative displacement under the step (closed form above).
pub(super) fn roof_u(t: f64) -> f64 {
    let (w, xi, a0): (f64, f64, f64) = (40.5_f64.sqrt(), 0.05, 2.0);
    let s = (1.0 - xi * xi).sqrt();
    let wd = w * s;
    -(a0 / (w * w)) * (1.0 - (-xi * w * t).exp() * ((wd * t).cos() + xi / s * (wd * t).sin()))
}

fn temp_avi(tag: &str) -> PathBuf {
    std::env::temp_dir().join(format!("structurax_w5e2e_{}_{tag}.avi", std::process::id()))
}

/// The frames of an AVI file as the writer lays them out (chunks `00db` of one frame's size).
fn frames_of(bytes: &[u8]) -> Vec<&[u8]> {
    let mut out = Vec::new();
    let mut i = 0;
    while i + 8 <= bytes.len() {
        if &bytes[i..i + 4] == b"00db"
            && u32::from_le_bytes(bytes[i + 4..i + 8].try_into().unwrap()) as usize == FRAME_BYTES
            && i + 8 + FRAME_BYTES <= bytes.len()
        {
            out.push(&bytes[i + 8..i + 8 + FRAME_BYTES]);
            i += 8 + FRAME_BYTES;
        } else {
            i += 1;
        }
    }
    out
}

/// Colour (r, g, b) of the pixel (x, y), y from the top, of a bottom-up BGR frame.
fn pixel(frame: &[u8], x: usize, y: usize) -> [u8; 3] {
    let o = (H - 1 - y) * W * 3 + 3 * x;
    [frame[o + 2], frame[o + 1], frame[o]]
}

/// Centroid, weighted by greenness, of the joint marker (100, 255, 175) near `at`: the
/// pixels within 5.5 px whose green exceeds red and blue clearly. The marker is a disc of
/// radius 3.8 px; the weight of a pixel is its green above the dark background.
fn marker_near(frame: &[u8], at: (f64, f64)) -> Option<(f64, f64)> {
    let (mut sw, mut sx, mut sy) = (0.0, 0.0, 0.0);
    for y in 0..H {
        for x in 0..W {
            let (dx, dy) = (x as f64 + 0.5 - at.0, y as f64 + 0.5 - at.1);
            if dx * dx + dy * dy > 30.25 {
                continue;
            }
            let [r, g, b] = pixel(frame, x, y);
            if g as i32 - r as i32 > 60 && g as i32 - b as i32 > 30 {
                let w = (g as f64 - 60.0).max(0.0);
                sw += w;
                sx += w * (x as f64 + 0.5);
                sy += w * (y as f64 + 0.5);
            }
        }
    }
    (sw > 0.0).then(|| (sx / sw, sy / sw))
}

/// Removes the file, unless the variable `W5_KEEP_VIDEO` names a directory to move it to (to look
/// at a frame by hand).
fn keep_or_remove(path: &Path) {
    match std::env::var_os("W5_KEEP_VIDEO") {
        Some(dir) => {
            let _ = std::fs::rename(path, Path::new(&dir).join(path.file_name().unwrap()));
        }
        None => {
            let _ = std::fs::remove_file(path);
        }
    }
}

fn have(tool: &str) -> bool {
    std::process::Command::new(tool).arg("-version").output().map(|o| o.status.success()).unwrap_or(false)
}

/// `ffprobe` says: (nb_read_frames, width, height, r_frame_rate, codec).
fn probe(path: &Path) -> Option<String> {
    if !have("ffprobe") {
        return None;
    }
    let out = std::process::Command::new("ffprobe")
        .args(["-v", "error", "-count_frames", "-select_streams", "v:0", "-show_entries"])
        .arg("stream=width,height,nb_read_frames,r_frame_rate,codec_name")
        .args(["-of", "default=nw=1"])
        .arg(path)
        .output()
        .ok()?;
    assert!(out.status.success(), "ffprobe: {}", String::from_utf8_lossy(&out.stderr));
    Some(String::from_utf8_lossy(&out.stdout).to_string())
}

/// `ffmpeg` decodes frame `k` to rgb24 (top row first).
fn decode(path: &Path, k: usize) -> Option<Vec<u8>> {
    if !have("ffmpeg") {
        return None;
    }
    let out = std::process::Command::new("ffmpeg")
        .args(["-v", "error", "-i"])
        .arg(path)
        .args(["-vf", &format!("select=eq(n\\,{k})"), "-frames:v", "1", "-f", "rawvideo", "-pix_fmt", "rgb24", "-"])
        .output()
        .ok()?;
    assert!(out.status.success(), "ffmpeg: {}", String::from_utf8_lossy(&out.stderr));
    Some(out.stdout)
}

/// The view of the last frame the recorder painted (the same transform for every frame:
/// it depends on the model's bounds, the zoom and the angles, not on the pose).
fn video_view(run: &Run) -> ViewTransform {
    run.app.last_view[0].expect("the recorder painted the viewport").0
}

/// Opens the form from File > Create Video and fills the fields the video needs.
fn open_form(run: &mut Run, entry: &str) {
    menu(run, &["File", "Create Video", entry]);
}

fn fill_common(run: &mut Run, path: &Path, magnification: &str) {
    run.set_number("Magnification", magnification);
    run.set_number("Frames per second", "10");
    run.set_number("Frame width (pixels)", &W.to_string());
    run.set_number("Frame height (pixels)", &H.to_string());
    super::ext::type_at(run, "AVI file name", 150.0, &path.display().to_string());
}

/// Presses OK of the form with the pointer and returns the view of the viewport the recorder
/// painted last (read in the frame of the release, before the application paints its own).
fn press_ok(run: &mut Run) -> ViewTransform {
    use eframe::egui;
    let pos = run.find("OK").center();
    let ev = |pressed| egui::Event::PointerButton { pos, button: egui::PointerButton::Primary, pressed, modifiers: Default::default() };
    run.frame(vec![egui::Event::PointerMoved(pos), ev(true)]);
    run.frame(vec![ev(false)]);
    let view = video_view(run);
    run.settle();
    view
}

fn analysed() -> Run {
    let mut run = Run::new(storey());
    // The harness shows a plan view; the application's own start is the 3D perspective view.
    run.app.left_proj = crate::gui::ViewProjection::Perspective3D;
    run.settle();
    analyze(&mut run);
    assert!(run.app.model.is_locked, "{}", run.app.status_msg);
    // The case the video needs holds 501 saved steps (0 .. 10 s at 0.02 s).
    assert_eq!(run.app.model.case_results["HIST"].time_steps.len(), 501);
    // View > Show Support Restraints off: the support symbols are filled shapes that would hide
    // the lower half of the joint markers in the small frames.
    menu(&mut run, &["View", "Show Support Restraints"]);
    assert!(!run.app.show_restraints);
    // The menu stays open after a check box: Escape closes it.
    let esc = |pressed| eframe::egui::Event::Key { key: eframe::egui::Key::Escape, physical_key: None, pressed, repeat: false, modifiers: Default::default() };
    run.frame(vec![esc(true)]);
    run.frame(vec![esc(false)]);
    run.settle();
    run
}

#[test]
fn a_multi_step_video_of_the_modal_history_has_the_frames_the_form_states_and_the_joint_where_the_pose_puts_it() {
    let mut run = analysed();
    let path = temp_avi("multi");
    let _ = std::fs::remove_file(&path);
    open_form(&mut run, "Multi-Step Animation Video...");
    assert!(run.has_text("Create Multi-Step Animation Video"), "{:?}", run.shown());
    // The form opens on the case with steps, HIST, with its whole range (0 to 10 s, 0.02 s).
    run.set_number("Start (time)", "0");
    run.set_number("End (time)", "2");
    run.set_number("Time increment", "0.2");
    fill_common(&mut run, &path, "10");
    // The count the form states: (2 - 0) / 0.2 + 1 = 11 frames, 11 / 10 fps = 1.10 s.
    assert!(run.text_starting("11 frames, 1.10 s at 10 frames per second").is_some(), "{:?}", run.shown());
    let view = press_ok(&mut run);
    assert_eq!(run.app.status_msg, format!("Video written (11 frames): {}", path.display()));
    let bytes = std::fs::read(&path).unwrap();
    let frames = frames_of(&bytes);
    assert_eq!(frames.len(), 11);
    // The container: a RIFF file of type AVI whose size field is the file length minus 8, with
    // eleven frames of 57600 bytes (160 x 3 bytes per row, 120 rows).
    assert_eq!(&bytes[0..4], b"RIFF");
    assert_eq!(&bytes[8..12], b"AVI ");
    assert_eq!(u32::from_le_bytes(bytes[4..8].try_into().unwrap()) as usize, bytes.len() - 8);
    assert!(bytes.len() > 11 * FRAME_BYTES);
    assert!(frames.windows(2).all(|w| w[0] != w[1]), "every frame differs from the next: the roof moves");
    // The tolerance of 1 px is the plan's: the marker is a disc of radius 3.8 px whose centroid is
    // read from antialiased pixels next to the frame lines (the worst distance is printed).
    // The joint: frame k shows the saved step nearest t_k = 0.2 k, drawn at base + 10 u(t_k)
    // (magnification 10 is a user factor). u from the closed form, in X only.
    let mut worst: f64 = 0.0;
    for (k, frame) in frames.iter().enumerate() {
        let t = 0.2 * k as f64;
        let u = roof_u(t);
        for (joint, base) in [("T1", [-3.0, -3.0, 4.0]), ("T3", [3.0, 3.0, 4.0]), ("B1", [-3.0, -3.0, 0.0])] {
            let uu = if joint.starts_with('B') { 0.0 } else { u };
            let want = view.project([base[0] + 10.0 * uu, base[1], base[2]]);
            let got = marker_near(frame, (want.x as f64, want.y as f64))
                .unwrap_or_else(|| panic!("frame {k}: no marker of {joint} near ({:.1}, {:.1})", want.x, want.y));
            let d = ((got.0 - want.x as f64).powi(2) + (got.1 - want.y as f64).powi(2)).sqrt();
            worst = worst.max(d);
            assert!(d <= 1.0, "frame {k} {joint}: marker at ({:.2}, {:.2}), pose puts it at ({:.2}, {:.2})", got.0, got.1, want.x, want.y);
        }
    }
    eprintln!("multi-step video: worst distance between a marker and the pose's position: {worst:.3} px");
    // The roof joint moved by hand-computed amounts: frame 2 (t = 0.4 s, the first peak) against frame 0.
    let (p0, p5) = (view.project([-3.0, -3.0, 4.0]), view.project([-3.0 + 10.0 * roof_u(0.4), -3.0, 4.0]));
    assert!(((p5.x - p0.x).powi(2) + (p5.y - p0.y).powi(2)).sqrt() > 3.0, "the displacement is visible");
    // The independent decoder.
    if let Some(text) = probe(&path) {
        for want in ["width=160", "height=120", "nb_read_frames=11", "r_frame_rate=10/1", "codec_name=rawvideo"] {
            assert!(text.lines().any(|l| l == want), "ffprobe lacks {want}: {text}");
        }
        // ffmpeg's decoding of frame 5 agrees with the bytes (top row first, RGB).
        let rgb = decode(&path, 5).expect("ffmpeg");
        let mut mine = Vec::new();
        for y in 0..H {
            for x in 0..W {
                mine.extend_from_slice(&pixel(frames[5], x, y));
            }
        }
        assert_eq!(rgb, mine, "ffmpeg decodes frame 5 differently from the bytes");
    }
    keep_or_remove(&path);
}

#[test]
fn a_cyclic_video_of_a_mode_has_cycles_times_frames_and_repeats_each_cycle() {
    let mut run = analysed();
    let path = temp_avi("cyclic");
    let _ = std::fs::remove_file(&path);
    open_form(&mut run, "Cyclic Animation Video...");
    assert!(run.has_text("Create Cyclic Animation Video"), "{:?}", run.shown());
    run.set_number("Mode number", "1");
    run.set_number("Number of cycles", "2");
    run.set_number("Frames per cycle", "12");
    fill_common(&mut run, &path, "100");
    // 2 cycles x 12 frames = 24 frames, 2.40 s at 10 fps.
    assert!(run.text_starting("24 frames, 2.40 s at 10 frames per second").is_some(), "{:?}", run.shown());
    let view = press_ok(&mut run);
    assert_eq!(run.app.status_msg, format!("Video written (24 frames): {}", path.display()));
    let bytes = std::fs::read(&path).unwrap();
    let frames = frames_of(&bytes);
    assert_eq!(frames.len(), 24);
    // Frame k has the phase 2 pi (k mod 12) / 12: frames k and k + 12 are the same picture.
    for k in 0..12 {
        assert_eq!(frames[k], frames[k + 12], "frame {k} and {} differ", k + 12);
    }
    // Frame k of a cycle shows the roof displaced by magnification x phi x sin(phase): at the rest
    // position at phase 0 and pi, to the two sides at the quarter phases.
    let shape = &run.app.model.case_results["MODAL"].modes[0].mode_shape["T1"];
    // Mode 1 is the sway in X (the other direction is 1.5 times stiffer). Mass-normalised
    // (phi' M phi = 1, Analysis Reference, eigenvector normalisation) a rigid-roof sway has
    // phi = 1/sqrt(total mass) = 1/sqrt(4 x 625) = 0.02 at every roof joint; only its sign is
    // arbitrary and is read from the stored shape. The amplitude is the hand value.
    assert!((shape[0].abs() - 0.02).abs() <= 1e-6 * 0.02 && shape[1].abs() <= 1e-6 * 0.02, "{shape:?}");
    let (amp_x, amp_y) = (0.02 * shape[0].signum(), 0.0);
    let base = [-3.0, -3.0, 4.0];
    let mut worst: f64 = 0.0;
    for (k, frame) in frames.iter().take(12).enumerate() {
        let phase = 2.0 * std::f64::consts::PI * k as f64 / 12.0;
        let want = view.project([base[0] + 100.0 * amp_x * phase.sin(), base[1] + 100.0 * amp_y * phase.sin(), base[2]]);
        let got = marker_near(frame, (want.x as f64, want.y as f64)).unwrap_or_else(|| panic!("frame {k}: no marker near ({:.1}, {:.1})", want.x, want.y));
        let d = ((got.0 - want.x as f64).powi(2) + (got.1 - want.y as f64).powi(2)).sqrt();
        worst = worst.max(d);
        assert!(d <= 1.0, "frame {k}: marker at ({:.2}, {:.2}), pose puts it at ({:.2}, {:.2})", got.0, got.1, want.x, want.y);
    }
    eprintln!("cyclic video: worst distance between a marker and the pose's position: {worst:.3} px");
    // Frames 0 and 6 (phase 0 and pi) have the joint at rest: the same marker position.
    let rest = view.project(base);
    let m0 = marker_near(frames[0], (rest.x as f64, rest.y as f64)).unwrap();
    let m6 = marker_near(frames[6], (rest.x as f64, rest.y as f64)).unwrap();
    assert!((m0.0 - m6.0).abs() <= 0.3 && (m0.1 - m6.1).abs() <= 0.3, "{m0:?} {m6:?}");
    if let Some(text) = probe(&path) {
        for want in ["width=160", "height=120", "nb_read_frames=24", "r_frame_rate=10/1"] {
            assert!(text.lines().any(|l| l == want), "ffprobe lacks {want}: {text}");
        }
    }
    keep_or_remove(&path);
}
