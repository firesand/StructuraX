//! Tests of the result forms: the pure functions first, then headless runs of
//! the application through `render_ui` with synthetic pointer input.
//!
//! The model of most tests is two quadrilateral areas side by side in the
//! X-Y plane at z = 0 (joints 1..6 at x = 0, 1, 1, 0, 2, 2 and y = 0, 0, 1, 1,
//! 0, 1); `A1` runs counter-clockwise (local 3 = +Z), `A2` clockwise (local 3
//! = -Z). Joint displacements are chosen by hand: Ux = 0.001 x, Uy = 0.002 y,
//! Uz = 0.01 x + 0.02 y (metres). Both areas carry the stored result
//! F11 = 10, M11 = 0.3 with tm = 0.2 and tb = 0.1 (kN, m): top S11 = 10/0.2 -
//! 6 * 0.3/0.1^2 = -130, bottom S11 = 50 + 180 = 230 (Analysis Reference,
//! Eqns. 2).
use super::super::joint_disp_3d;
use super::super::view::ViewTransform;
use super::*;
use eframe::egui::{epaint::Shape, Rect};
use sap_model::{
    AreaResult, AreaSection, ComboType, LoadCase, LoadCaseType, LoadCombo, ModalModeResult,
};
use std::path::PathBuf;

const GREEN: Color32 = Color32::from_rgb(100, 255, 175);

fn close(what: &str, got: f64, want: f64, tol: f64) {
    assert!((got - want).abs() <= tol, "{what}: got {got:e}, want {want:e}");
}

fn plate() -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    m.load_cases.clear();
    m.load_cases.insert("S".into(), LoadCase::linear_static("S", "D", 1.0));
    m.area_sections.insert(
        "SH".into(),
        AreaSection {
            name: "SH".into(),
            thickness_membrane: 0.2,
            thickness_bending: 0.1,
            ..AreaSection::default()
        },
    );
    let xy = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (2.0, 0.0), (2.0, 1.0)];
    for (i, (x, y)) in xy.iter().enumerate() {
        m.add_point(format!("{}", i + 1), *x, *y, 0.0);
    }
    let names = |ids: [usize; 4]| ids.iter().map(|i| i.to_string()).collect::<Vec<_>>();
    m.add_area("A1", names([1, 2, 3, 4]), "SH");
    m.add_area("A2", names([2, 3, 6, 5]), "SH");
    let mut cr = CaseResults { case_name: "S".into(), ..Default::default() };
    for (i, (x, y)) in xy.iter().enumerate() {
        cr.joint_displacements
            .insert(format!("{}", i + 1), [0.001 * x, 0.002 * y, 0.01 * x + 0.02 * y, 0.0, 0.0, 0.0]);
    }
    let res = AreaResult {
        f11: 10.0,
        m11: 0.3,
        s11_top: -130.0,
        svm_top: 130.0,
        ..AreaResult::default()
    };
    for a in ["A1", "A2"] {
        cr.area_results.insert(a.into(), res.clone());
    }
    m.case_results.insert("S".into(), cr);
    m
}

fn with_srss(mut m: SapModel) -> SapModel {
    m.load_combos.insert(
        "SRSS1".into(),
        LoadCombo { name: "SRSS1".into(), combo_type: ComboType::Srss, cases: vec![("S".into(), 1.0)] },
    );
    let src = m.case_results["S"].clone();
    m.case_results.insert("SRSS1".into(), src);
    m
}

const SEL: StepSelection = StepSelection { mode_idx: 0, use_step: false, step_idx: 0 };
const UP: [f64; 3] = [0.0, 0.0, 1.0];
const DOWN: [f64; 3] = [0.0, 0.0, -1.0];

fn s11(face: Face) -> ShellQuantity {
    ShellQuantity::Stress(ShellStress::S11, face)
}

fn ok_value(t: &Table, aid: &str) -> f64 {
    t.entries[aid].as_ref().unwrap_or_else(|e| panic!("{e}"))[0]
}

// ------------------------------------------------------------- pure functions

#[test]
fn the_range_fields_are_parsed_and_checked() {
    assert_eq!(parse_range("", "  ").unwrap(), (0.0, 0.0));
    assert_eq!(parse_range("-0.5", "2e-3").unwrap(), (-0.5, 0.002));
    assert_eq!(parse_range("0", "1").unwrap(), (0.0, 1.0));
    let e = parse_range("abc", "1").unwrap_err();
    assert!(e.contains("minimum") && e.contains("'abc'"), "{e}");
    assert!(parse_range("1", "nan").unwrap_err().contains("maximum"));
    assert!(parse_range("1", "inf").is_err());
    // min must be below max, also when only one is typed.
    assert!(parse_range("2", "1").unwrap_err().contains("below"));
    assert!(parse_range("1", "").unwrap_err().contains("below"));
}

fn view_of(proj: ViewProjection, yaw: f32, pitch: f32) -> ViewTransform {
    ViewTransform {
        proj,
        center: [0.3, -0.2, 0.9],
        scale: 40.0,
        screen_center: Pos2::new(500.0, 400.0),
        yaw,
        pitch,
    }
}

/// The eye direction as `right x up`, derived from `project` alone.
fn toward_from_project(v: &ViewTransform) -> [f64; 3] {
    let o = v.project(v.center);
    let axis = |i: usize| {
        let mut p = v.center;
        p[i] += 1.0;
        let s = v.project(p);
        // (screen right, screen up) of one model unit along axis i, in 1/scale.
        ((s.x - o.x) as f64 / v.scale, -(s.y - o.y) as f64 / v.scale)
    };
    let (r, u) = ([axis(0).0, axis(1).0, axis(2).0], [axis(0).1, axis(1).1, axis(2).1]);
    [
        r[1] * u[2] - r[2] * u[1],
        r[2] * u[0] - r[0] * u[2],
        r[0] * u[1] - r[1] * u[0],
    ]
}

#[test]
fn the_eye_direction_follows_the_projection_code() {
    let mut views = vec![
        view_of(ViewProjection::PlanXY, 0.0, 0.0),
        view_of(ViewProjection::ElevationXZ, 0.0, 0.0),
        view_of(ViewProjection::ElevationYZ, 0.0, 0.0),
    ];
    for (yaw, pitch) in [(0.0, 0.0), (-0.65, 0.42), (2.0, -0.7), (3.0, 1.2), (-1.0, -1.4)] {
        views.push(view_of(ViewProjection::Perspective3D, yaw, pitch));
    }
    for v in &views {
        let want = toward_from_project(v);
        let got = toward_viewer(v);
        for i in 0..3 {
            close(&format!("{:?} yaw {} pitch {} axis {i}", v.proj, v.yaw, v.pitch), got[i], want[i], 2e-5);
        }
    }
    // The named directions of the plans.
    assert_eq!(toward_viewer(&views[0]), [0.0, 0.0, 1.0]);
    assert_eq!(toward_viewer(&views[1]), [0.0, -1.0, 0.0]);
    assert_eq!(toward_viewer(&views[2]), [1.0, 0.0, 0.0]);
    // The default 3D camera (yaw -0.65, pitch +0.42) looks from above: the eye
    // lies at positive Z (sin 0.42 = 0.4078), so a slab with local 3 = +Z shows
    // its top face.
    let default = toward_viewer(&view_of(ViewProjection::Perspective3D, -0.65, 0.42));
    assert!(default[2] > 0.4, "{default:?}");
    let level = toward_viewer(&view_of(ViewProjection::Perspective3D, 0.0, 0.0));
    assert_eq!(level, [0.0, -1.0, 0.0]);
}

#[test]
fn the_visible_face_of_an_area_follows_its_local_3_in_plan_elevation_and_3d() {
    let m = plate();
    let q = s11(Face::Top);
    // A1 has local 3 = +Z, A2 = -Z (checked first).
    assert_eq!(contours::area_local_axes(&m, "A1").unwrap()[2], [0.0, 0.0, 1.0]);
    assert_eq!(contours::area_local_axes(&m, "A2").unwrap()[2], [0.0, 0.0, -1.0]);
    // Plan view, eye at +Z: A1 shows its top face (-130), A2 its bottom (+230).
    let t = shell_table(&m, "S", q, true, UP, (0.0, 0.0));
    close("A1 plan", ok_value(&t, "A1"), -130.0, 1e-9);
    close("A2 plan", ok_value(&t, "A2"), 230.0, 1e-9);
    // Eye at -Z (a camera below the model): swapped.
    let t = shell_table(&m, "S", q, true, DOWN, (0.0, 0.0));
    close("A1 below", ok_value(&t, "A1"), 230.0, 1e-9);
    close("A2 below", ok_value(&t, "A2"), -130.0, 1e-9);
    // 3D at the default camera (from above) and after turning it to look from below.
    let above = toward_viewer(&view_of(ViewProjection::Perspective3D, -0.65, 0.42));
    let below = toward_viewer(&view_of(ViewProjection::Perspective3D, -0.65, -0.42));
    close("A1 3D default", ok_value(&shell_table(&m, "S", q, true, above, (0.0, 0.0)), "A1"), -130.0, 1e-9);
    close("A1 3D turned", ok_value(&shell_table(&m, "S", q, true, below, (0.0, 0.0)), "A1"), 230.0, 1e-9);
    // Elevations see the plan edge-on: the tie goes to the top face of both.
    for toward in [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0]] {
        let t = shell_table(&m, "S", q, true, toward, (0.0, 0.0));
        close("A1 edge-on", ok_value(&t, "A1"), -130.0, 1e-9);
        close("A2 edge-on", ok_value(&t, "A2"), -130.0, 1e-9);
    }
    // Without "visible" the face is the quantity's own, whatever the view.
    let t = shell_table(&m, "S", s11(Face::Bottom), false, UP, (0.0, 0.0));
    close("fixed bottom", ok_value(&t, "A2"), 230.0, 1e-9);
    // Resultants and shear stresses have no face: the flag changes nothing.
    let f = ShellQuantity::Resultant(ShellResultant::F11);
    let a = shell_table(&m, "S", f, true, UP, (0.0, 0.0));
    let b = shell_table(&m, "S", f, true, DOWN, (0.0, 0.0));
    assert_eq!(ok_value(&a, "A2"), 10.0);
    assert_eq!(ok_value(&b, "A2"), 10.0);
}

#[test]
fn the_shell_table_has_one_value_per_area_and_an_automatic_or_given_range() {
    let m = plate();
    let t = shell_table(&m, "S", s11(Face::Top), true, UP, (0.0, 0.0));
    assert_eq!(t.entries.len(), 2);
    assert!(t.messages.is_empty(), "{:?}", t.messages);
    let r = t.range.unwrap();
    close("lo", r.lo, -130.0, 1e-9);
    close("hi", r.hi, 230.0, 1e-9);
    let t = shell_table(&m, "S", s11(Face::Top), true, UP, (-50.0, 60.0));
    assert_eq!(t.range, Some(ContourRange { lo: -50.0, hi: 60.0 }));
    let t = shell_table(&m, "S", s11(Face::Top), true, UP, (5.0, 1.0));
    assert!(t.range.is_none() && t.messages.iter().any(|x| x.contains("below")), "{:?}", t.messages);
    let t = shell_table(&m, "NOPE", s11(Face::Top), true, UP, (0.0, 0.0));
    assert!(t.entries.is_empty() && t.messages[0].contains("'NOPE' has no results"), "{:?}", t.messages);
}

#[test]
fn a_refused_value_is_counted_with_its_reason_and_never_drawn_as_zero() {
    // SMAX of an SRSS combination is refused for every area.
    let m = with_srss(plate());
    let smax = ShellQuantity::Stress(ShellStress::SMax, Face::Top);
    let t = shell_table(&m, "SRSS1", smax, false, UP, (0.0, 0.0));
    assert!(t.entries.values().all(|e| e.is_err()));
    assert!(t.range.is_none());
    let note = t.messages.iter().find(|x| x.starts_with("2 area(s)")).expect("count");
    assert!(note.contains("SMAX") && note.contains("SRSS1") && note.contains("multi-valued"), "{note}");
    let fc = legend_of(&m, "SRSS1", Target::Shell { quantity: smax, visible: false });
    assert!(fc.legend.messages.iter().any(|x| x.contains("multi-valued")));
    // One refused area among valid ones: counted, outside the range, no ramp colour.
    let mut m = plate();
    m.areas.get_mut("A2").unwrap().section = "GONE".into();
    let t = shell_table(&m, "S", ShellQuantity::Resultant(ShellResultant::F11), false, UP, (0.0, 0.0));
    assert_eq!(t.range, Some(ContourRange { lo: 10.0, hi: 10.0 }));
    assert!(t.messages[0].starts_with("1 area(s)") && t.messages[0].contains("'GONE'"), "{:?}", t.messages);
    let fc = FrameContour { legend: Legend::default(), table: t };
    let poly = [Pos2::ZERO, Pos2::new(1.0, 0.0), Pos2::new(1.0, 1.0)];
    match fc.fill("A2", &poly) {
        AreaFill::Flat(c) => {
            assert_eq!(c, no_contour_fill());
            let zero = contours::value_color(0.0, 10.0, 10.0, CONTOUR_ALPHA);
            assert_ne!(c, zero);
        }
        other => panic!("{other:?}"),
    }
    assert!(matches!(fc.fill("A1", &poly), AreaFill::Flat(c) if c == contours::value_color(10.0, 10.0, 10.0, CONTOUR_ALPHA)));
}

fn legend_of(m: &SapModel, result: &str, target: Target) -> FrameContour {
    let spec = ContourSpec { target, range: (0.0, 0.0), units: m.present_units };
    frame_contour(m, result, spec, SEL, UP)
}

#[test]
fn missing_and_dropped_results_are_explained_in_the_legend() {
    let mut m = plate();
    m.load_cases.insert("MD".into(), {
        let mut c = LoadCase::linear_static("MD", "D", 1.0);
        c.case_type = LoadCaseType::Modal;
        c
    });
    m.case_results.insert("MD".into(), CaseResults::default());
    let q = ShellQuantity::Resultant(ShellResultant::F11);
    let fc = legend_of(&m, "MD", Target::Shell { quantity: q, visible: false });
    let msg = shell_output::missing_results_message(&m, "MD", ResultKind::Area).unwrap();
    assert_eq!(fc.legend.messages, vec![msg.clone()]);
    assert!(msg.contains("'MD'") && msg.contains("Modal"), "{msg}");
    assert!(fc.legend.range.is_none() && fc.legend.ticks.is_empty());
    // A result that does not exist at all.
    let fc = legend_of(&m, "NOPE", Target::Shell { quantity: q, visible: false });
    assert!(fc.legend.messages[0].contains("'NOPE' has no results"));
    // A combination of S and the modal case: it has shell results, the message
    // about the dropped case stays next to the scale.
    m.load_combos.insert(
        "C1".into(),
        LoadCombo { name: "C1".into(), combo_type: ComboType::LinearAdd, cases: vec![("S".into(), 1.0), ("MD".into(), 1.0)] },
    );
    let src = m.case_results["S"].clone();
    m.case_results.insert("C1".into(), src);
    let fc = legend_of(&m, "C1", Target::Shell { quantity: q, visible: false });
    assert!(fc.legend.range.is_some());
    assert!(fc.legend.messages.iter().any(|x| x.contains("'MD'") && x.contains("carries no shell results")), "{:?}", fc.legend.messages);
}

#[test]
fn displacement_values_follow_the_component_and_the_corners() {
    let m = plate();
    let ux = displacement_table(&m, "S", DisplacementComponent::Ux, SEL, (0.0, 0.0));
    // A1 corners 1, 2, 3, 4 at x = 0, 1, 1, 0.
    assert_eq!(ux.entries["A1"].as_ref().unwrap(), &vec![0.0, 0.001, 0.001, 0.0]);
    // A2 is clockwise through joints 2, 3, 6, 5 at x = 1, 1, 2, 2.
    assert_eq!(ux.entries["A2"].as_ref().unwrap(), &vec![0.001, 0.001, 0.002, 0.002]);
    assert_eq!(ux.range, Some(ContourRange { lo: 0.0, hi: 0.002 }));
    // Area 3 axis of A2 is -Z: -Uz.
    let a3 = displacement_table(&m, "S", DisplacementComponent::Area3, SEL, (0.0, 0.0));
    let uz2 = [0.01, 0.03, 0.04, 0.02]; // Uz = 0.01 x + 0.02 y at joints 2, 3, 6, 5
    for (got, want) in a3.entries["A2"].as_ref().unwrap().iter().zip(uz2) {
        close("area 3", *got, -want, 1e-15);
    }
}

#[test]
fn an_area_with_a_corner_without_result_or_with_five_joints_has_no_contour() {
    let mut m = plate();
    m.case_results.get_mut("S").unwrap().joint_displacements.shift_remove("5");
    let t = displacement_table(&m, "S", DisplacementComponent::Ux, SEL, (0.0, 0.0));
    assert!(t.entries["A1"].is_ok());
    let err = t.entries["A2"].as_ref().unwrap_err();
    assert!(err.contains("'5'"), "{err}");
    assert!(t.messages[0].starts_with("1 area(s)") && t.messages[0].contains("'5'"), "{:?}", t.messages);
    // The range comes from A1 only: nothing of A2 counts as zero.
    assert_eq!(t.range, Some(ContourRange { lo: 0.0, hi: 0.001 }));
    // Five joints: refused, not truncated.
    m.add_point("7", 3.0, 0.0, 0.0);
    m.add_area("A5", ["1", "2", "3", "4", "7"].map(String::from).to_vec(), "SH");
    let t = displacement_table(&m, "S", DisplacementComponent::Ux, SEL, (0.0, 0.0));
    let err = t.entries["A5"].as_ref().unwrap_err();
    assert!(err.contains("has 5 joints") && err.contains("three or four"), "{err}");
}

#[test]
fn tick_labels_and_numbers_have_a_fixed_rule() {
    let ticks = |lo: f64, hi: f64| {
        contours::legend_ticks(ContourRange { lo, hi }, LEGEND_INTERVALS)
    };
    assert_eq!(
        tick_labels(&ticks(0.0, 2.0)),
        ["0.000", "0.200", "0.400", "0.600", "0.800", "1.000", "1.200", "1.400", "1.600", "1.800", "2.000"]
    );
    // 360 wide: one decimal gives four significant digits of the span.
    assert_eq!(tick_labels(&ticks(-130.0, 230.0))[0], "-130.0");
    assert_eq!(tick_labels(&ticks(-130.0, 230.0))[10], "230.0");
    assert_eq!(tick_labels(&ticks(0.0, 36000.0))[1], "3600");
    assert_eq!(tick_labels(&ticks(0.0, 0.5))[1], "0.0500");
    // Tiny and huge values use scientific notation.
    assert_eq!(tick_labels(&ticks(0.0, 2e-4))[10], "2.000e-4");
    assert_eq!(tick_labels(&ticks(0.0, 3e6))[10], "3.000e6");
    // A constant field: one value, three decimals.
    assert_eq!(tick_labels(&[10.0, 10.0])[0], "10.000");
    assert!(tick_labels(&[]).is_empty());
    assert_eq!(number_text(0.0), "0.000000");
    assert_eq!(number_text(500.0), "500.000000");
    assert_eq!(number_text(1.5e-5), "1.50000e-5");
    assert_eq!(number_text(f64::NAN), "-");
}

#[test]
fn the_legend_has_ten_bands_ticks_in_display_units_and_a_title_with_the_unit() {
    let m = plate();
    // Stress S11 on both faces, model kN/m2, shown in N/mm2: x 1e-3.
    let spec = ContourSpec {
        target: Target::Shell { quantity: s11(Face::Top), visible: true },
        range: (0.0, 0.0),
        units: Units::NMmC,
    };
    let fc = frame_contour(&m, "S", spec, SEL, UP);
    let l = &fc.legend;
    assert_eq!(l.title, "Stress S11 (Visible Face) [N/mm2]");
    assert_eq!(l.bands.len(), LEGEND_INTERVALS);
    assert_eq!(l.ticks.len(), LEGEND_INTERVALS + 1);
    // Range -130..230 kN/m2 = -0.13..0.23 N/mm2, span 0.36, 4 decimals.
    assert_eq!(l.ticks[0], "-0.1300");
    assert_eq!(l.ticks[10], "0.2300");
    assert_eq!(l.ticks[5], "0.0500");
    close("band 0 middle", l.bands[0], -130.0 + 36.0 / 2.0, 1e-9);
    close("band 9 middle", l.bands[9], 230.0 - 36.0 / 2.0, 1e-9);
    // A user range is typed in display units: -0.1..0.2 N/mm2 = -100..200 kN/m2.
    let spec = ContourSpec { range: (-0.1, 0.2), ..spec };
    let fc = frame_contour(&m, "S", spec, SEL, UP);
    let r = fc.legend.range.unwrap();
    close("user lo", r.lo, -100.0, 1e-9);
    close("user hi", r.hi, 200.0, 1e-9);
    assert_eq!(fc.legend.ticks[0], "-0.1000");
    assert_eq!(fc.legend.ticks[10], "0.2000");
    // Moment per length: kN-m/m to N-mm/mm is x 1000.
    let spec = ContourSpec {
        target: Target::Shell { quantity: ShellQuantity::Resultant(ShellResultant::M11), visible: false },
        range: (0.0, 0.0),
        units: Units::NMmC,
    };
    let fc = frame_contour(&m, "S", spec, SEL, UP);
    assert_eq!(fc.legend.title, "Plate Moment M11 [N-mm/mm]");
    assert_eq!(fc.legend.ticks[0], "300.000");
    // Displacement in mm.
    let spec = ContourSpec {
        target: Target::Displacement(DisplacementComponent::Ux),
        range: (0.0, 0.0),
        units: Units::NMmC,
    };
    let fc = frame_contour(&m, "S", spec, SEL, UP);
    assert_eq!(fc.legend.title, "Displacement Ux [mm]");
    assert_eq!(fc.legend.ticks[0], "0.000");
    assert_eq!(fc.legend.ticks[10], "2.000");
}

#[test]
fn shell_titles_name_the_face() {
    use ShellQuantity as Q;
    assert_eq!(shell_title(s11(Face::Top), true), "Stress S11 (Visible Face)");
    assert_eq!(shell_title(Q::Stress(ShellStress::Svm, Face::Top), true), "Von Mises Stress SVM (Visible Face)");
    assert_eq!(shell_title(s11(Face::AbsMaximum), false), "Stress S11 (Abs Max)");
    assert_eq!(shell_title(Q::Stress(ShellStress::S13, Face::Top), true), "Stress S13");
    assert_eq!(shell_title(Q::Resultant(ShellResultant::FMax), true), "Membrane FMAX");
}

#[test]
fn the_contour_follows_the_display_mode_and_the_applied_form() {
    let u = Units::KnMC;
    let q = ShellQuantity::LEGACY[0];
    let disp = Contour::Displacement { component: DisplacementComponent::Uy, range: (1.0, 2.0), units: Units::NMmC };
    // Displacement contour only on the deformed shape.
    assert_eq!(
        contour_spec(DisplayMode::Deformed, disp, u),
        Some(ContourSpec { target: Target::Displacement(DisplacementComponent::Uy), range: (1.0, 2.0), units: Units::NMmC })
    );
    assert!(contour_spec(DisplayMode::Undeformed, disp, u).is_none());
    assert!(contour_spec(DisplayMode::Deformed, Contour::Off, u).is_none());
    assert!(contour_spec(DisplayMode::ShellContour(q), disp, u).map(|s| s.range) == Some((0.0, 0.0)));
    // Legacy shell contour: model units, automatic, own face.
    assert_eq!(
        contour_spec(DisplayMode::ShellContour(q), Contour::Off, u),
        Some(ContourSpec { target: Target::Shell { quantity: q, visible: false }, range: (0.0, 0.0), units: u })
    );
    let form = Contour::Shell { visible: true, range: (0.0, 5.0), units: Units::KipInF };
    assert_eq!(
        contour_spec(DisplayMode::ShellContour(q), form, u).unwrap().range,
        (0.0, 5.0)
    );
    assert!(contour_spec(DisplayMode::Deformed, form, u).is_none());
}

#[test]
fn the_deformed_form_is_validated_without_touching_the_model() {
    let mut m = plate();
    let mut f = DeformedForm::default();
    f.open_with("S", 10.0, Units::NMmC);
    f.contour = true;
    f.component = DisplacementComponent::Area2;
    f.min = "-1".into();
    f.max = "3".into();
    let before = m.clone();
    let a = f.apply(&m).unwrap();
    assert_eq!(
        a,
        DeformedApply {
            case: "S".into(),
            mode_idx: 0,
            use_step: false,
            step_idx: 0,
            scale: 10.0,
            contour: Contour::Displacement { component: DisplacementComponent::Area2, range: (-1.0, 3.0), units: Units::NMmC },
            units: Units::NMmC,
        }
    );
    f.contour = false;
    assert_eq!(f.apply(&m).unwrap().contour, Contour::Off);
    let mut bad = f.clone();
    bad.case = "NOPE".into();
    assert!(bad.apply(&m).unwrap_err().contains("'NOPE' has no results"));
    for scale in [0.0, -1.0, f32::NAN] {
        let mut bad = f.clone();
        bad.scale = scale;
        assert!(bad.apply(&m).unwrap_err().contains("scale"), "{scale}");
    }
    let mut bad = f.clone();
    bad.min = "x".into();
    assert!(bad.apply(&m).unwrap_err().contains("'x'"));
    // Modes: one-based, bounded.
    m.case_results.get_mut("S").unwrap().modes = vec![ModalModeResult::default(); 3];
    f.mode = 3;
    assert_eq!(f.apply(&m).unwrap().mode_idx, 2);
    f.mode = 4;
    assert!(f.apply(&m).unwrap_err().contains("mode 4 does not exist"));
    f.mode = 0;
    assert!(f.apply(&m).is_err());
    // Steps: only for results with at least two.
    let cr = m.case_results.get_mut("S").unwrap();
    cr.modes.clear();
    cr.time_steps = vec![0.0, 0.1, 0.2];
    cr.time_history_disp.insert("1".into(), vec![[0.0; 6]; 3]);
    f.step = 2;
    let a = f.apply(&m).unwrap();
    assert!(a.use_step && a.step_idx == 2);
    f.step = 3;
    assert!(f.apply(&m).unwrap_err().contains("steps 0 to 2"));
    // Nothing was changed by any of it.
    let mut m2 = before.clone();
    m2.case_results.get_mut("S").unwrap().time_steps.clear();
    assert_eq!(before.case_results["S"].joint_displacements, m2.case_results["S"].joint_displacements);
}

#[test]
fn the_shell_form_names_its_quantity_and_validates() {
    let m = with_srss(plate());
    let mut f = ShellForm::default();
    f.open_with("S", Units::NMmC);
    // Resultants ignore the face.
    f.resultant = ShellResultant::FMax;
    let a = f.apply(&m).unwrap();
    assert_eq!(a.quantity, ShellQuantity::Resultant(ShellResultant::FMax));
    assert_eq!(a.contour, Contour::Shell { visible: false, range: (0.0, 0.0), units: Units::NMmC });
    f.kind = ShellKind::Stress;
    f.stress = ShellStress::SMax;
    f.face = FaceChoice::Visible;
    let a = f.apply(&m).unwrap();
    assert_eq!(a.quantity, ShellQuantity::Stress(ShellStress::SMax, Face::Top));
    assert!(matches!(a.contour, Contour::Shell { visible: true, .. }));
    f.face = FaceChoice::Fixed(Face::AbsMaximum);
    let a = f.apply(&m).unwrap();
    assert_eq!(a.quantity, ShellQuantity::Stress(ShellStress::SMax, Face::AbsMaximum));
    assert!(matches!(a.contour, Contour::Shell { visible: false, .. }));
    // SMAX of the SRSS combination is accepted by the form (the legend says why
    // it cannot be drawn); a result without results is not.
    f.case = "SRSS1".into();
    assert!(f.apply(&m).is_ok());
    f.case = "NOPE".into();
    assert!(f.apply(&m).is_err());
    f.case = "S".into();
    f.max = "-1".into();
    assert!(f.apply(&m).unwrap_err().contains("below"));
}

#[test]
fn all_component_buttons_name_distinct_quantities() {
    let mut labels = std::collections::BTreeSet::new();
    for (r, name) in RESULTANT_ITEMS {
        assert!(ShellQuantity::Resultant(r).label().ends_with(name), "{name}");
        labels.insert(name);
    }
    for (s, name) in STRESS_ITEMS {
        assert!(ShellQuantity::Stress(s, Face::Top).label().contains(name), "{name}");
        labels.insert(name);
    }
    assert_eq!(labels.len(), 23);
    assert_eq!(FACE_CHOICES.len(), 6);
    assert_eq!(DISPLACEMENT_COMPONENTS.len(), 6);
}

#[test]
fn the_deformed_shape_uses_global_components_and_matches_the_old_expression_without_joint_axes() {
    let m = plate();
    let cr = &m.case_results["S"];
    // Every joint, scale and step selection of a model without joint local
    // axes: bit for bit the old `base + scale * u[0..3]`.
    for scale in [1.0, 10.0, -3.5, 0.0, 250.0] {
        for (id, p) in &m.points {
            let base = [p.x, p.y, p.z];
            let old = joint_disp_3d(cr, id, scale, 0, false, 0);
            let want = [base[0] + old[0], base[1] + old[1], base[2] + old[2]];
            let got = displaced_point(&m, Some(cr), id, base, scale, SEL);
            for i in 0..3 {
                assert_eq!(got[i].to_bits(), want[i].to_bits(), "joint {id} scale {scale} axis {i}");
            }
        }
    }
    // No result for the case: the undeformed point.
    assert_eq!(displaced_point(&m, None, "1", [1.0, 2.0, 3.0], 5.0, SEL), [1.0, 2.0, 3.0]);
    assert_eq!(displaced_point(&m, Some(cr), "99", [1.0, 2.0, 3.0], 5.0, SEL), [1.0, 2.0, 3.0]);
}

#[test]
fn a_rotated_joint_is_drawn_along_its_own_axes() {
    // Joint 2 turned 90 degrees about Z: local 1 is global Y (R columns are the
    // joint's axes, u_global = R u_local, Analysis Reference p. 24).
    let mut m = plate();
    m.points.get_mut("2").unwrap().local_axes_deg = [90.0, 0.0, 0.0];
    let cr = m.case_results.get_mut("S").unwrap();
    cr.joint_displacements.insert("2".into(), [0.5, 0.0, 0.25, 0.0, 0.0, 0.0]);
    let cr = &m.case_results["S"];
    let base = [1.0, 0.0, 0.0];
    let old = joint_disp_3d(cr, "2", 2.0, 0, false, 0);
    let old_pos = [base[0] + old[0], base[1] + old[1], base[2] + old[2]];
    assert_eq!(old_pos, [2.0, 0.0, 0.5]);
    let new_pos = displaced_point(&m, Some(cr), "2", base, 2.0, SEL);
    // base + 2 * R * (0.5, 0, 0.25) = (1, 0, 0) + 2 * (0, 0.5, 0.25).
    close("x", new_pos[0], 1.0, 1e-12);
    close("y", new_pos[1], 1.0, 1e-12);
    close("z", new_pos[2], 0.5, 1e-12);
    // A non-finite component counts as zero (a NaN in one local component also
    // zeroes the global components that depend on it, which the old drawing did
    // not do for an unrotated joint's other components).
    let mut m2 = plate();
    m2.case_results.get_mut("S").unwrap().joint_displacements.insert("2".into(), [f64::NAN, 0.0, 1.0, 0.0, 0.0, 0.0]);
    let p = displaced_point(&m2, Some(&m2.case_results["S"]), "2", base, 1.0, SEL);
    assert!(p.iter().all(|v| v.is_finite()));
}

#[test]
fn animation_steps_follow_the_time_at_mid_step_instants() {
    let mut cr = CaseResults::default();
    assert!(step_animation(Some(&cr), 1.0, 2.0).is_none());
    assert!(step_animation(None, 1.0, 2.0).is_none());
    cr.time_steps = vec![0.0, 0.1, 0.2];
    cr.time_history_disp.insert("1".into(), vec![[0.0; 6]; 3]);
    let at = |dt: f64| step_animation(Some(&cr), 10.0, 10.0 + dt);
    // 0.05 s per step, three steps, looping; mid-step instants avoid rounding.
    assert_eq!(at(0.0), Some(0));
    assert_eq!(at(0.025), Some(0));
    assert_eq!(at(0.075), Some(1));
    assert_eq!(at(0.125), Some(2));
    assert_eq!(at(0.175), Some(0));
    assert_eq!(at(0.325), Some(0)); // 6 steps: two loops
    assert_eq!(at(-1.0), Some(0)); // a clock that went back
    // A single saved state is not animated; a modal result keeps its sine.
    cr.time_steps = vec![0.0];
    assert!(step_animation(Some(&cr), 0.0, 1.0).is_none());
    let mut modal = CaseResults::default();
    modal.modes = vec![ModalModeResult::default()];
    assert!(step_animation(Some(&modal), 0.0, 1.0).is_none());
}

#[test]
fn the_joint_pick_takes_the_closest_joint_within_ten_pixels() {
    let pts = vec![("a".to_string(), Pos2::new(100.0, 100.0)), ("b".to_string(), Pos2::new(106.0, 100.0))];
    assert_eq!(closest_within(&pts, Pos2::new(103.5, 100.0), JOINT_PICK_RADIUS).as_deref(), Some("b"));
    assert_eq!(closest_within(&pts, Pos2::new(101.0, 100.0), JOINT_PICK_RADIUS).as_deref(), Some("a"));
    assert_eq!(closest_within(&pts, Pos2::new(120.0, 100.0), JOINT_PICK_RADIUS), None);
    assert_eq!(closest_within(&pts, Pos2::new(109.9, 100.0), JOINT_PICK_RADIUS).as_deref(), Some("b"));
    assert_eq!(closest_within(&pts, Pos2::new(116.5, 100.0), JOINT_PICK_RADIUS), None);
    assert_eq!(closest_within(&[], Pos2::ZERO, JOINT_PICK_RADIUS), None);
}

#[test]
fn the_readout_lists_joint_local_components_in_display_units() {
    let mut m = plate();
    m.case_results.get_mut("S").unwrap().joint_displacements.insert("3".into(), [0.5, -0.002, 0.0, 0.0, 1.0e-5, 0.0]);
    let (head, lines) = readout_lines(&m, "S", "3", SEL, Units::NMmC);
    assert_eq!(head, "Joint 3 (result S), local axes");
    let want = [
        ("U1 [mm]", "500.000000"),
        ("U2 [mm]", "-2.000000"),
        ("U3 [mm]", "0.000000"),
        ("R1 [rad]", "0.000000"),
        ("R2 [rad]", "1.00000e-5"),
        ("R3 [rad]", "0.000000"),
    ];
    let got: Vec<(&str, &str)> = lines.iter().map(|(a, b)| (a.as_str(), b.as_str())).collect();
    assert_eq!(got, want);
    // Errors are one line with the reason.
    let (_, lines) = readout_lines(&m, "S", "99", SEL, Units::NMmC);
    assert_eq!(lines.len(), 1);
    assert!(lines[0].1.contains("'99'"));
    let (_, lines) = readout_lines(&m, "NOPE", "3", SEL, Units::NMmC);
    assert!(lines[0].1.contains("NOPE"));
}

#[test]
fn a_readout_is_refused_while_animating() {
    let mut st = State::default();
    assert_eq!(st.open_readout("3", true).unwrap_err(), "Joint displacements can not be displayed during animation.");
    assert!(st.readout_joint.is_none());
    st.open_readout("3", false).unwrap();
    assert_eq!(st.readout_joint.as_deref(), Some("3"));
    assert!(st.open_readout("4", true).is_err());
    assert!(st.readout_joint.is_none());
}

// ----------------------------------------------------------- headless application

const SCREEN: Vec2 = Vec2::new(1800.0, 1400.0);

struct Run {
    app: SapGuiApp,
    ctx: egui::Context,
    time: f64,
    last: egui::FullOutput,
}

fn flat(out: &egui::FullOutput) -> Vec<Shape> {
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

fn texts(out: &egui::FullOutput) -> Vec<(String, Rect)> {
    flat(out)
        .into_iter()
        .filter_map(|s| match s {
            Shape::Text(t) => Some((t.galley.job.text.clone(), Rect::from_min_size(t.pos, t.galley.size()))),
            _ => None,
        })
        .collect()
}

fn meshes(out: &egui::FullOutput) -> Vec<Vec<egui::epaint::Vertex>> {
    flat(out)
        .into_iter()
        .filter_map(|s| match s {
            Shape::Mesh(m) => Some(m.vertices.clone()),
            _ => None,
        })
        .collect()
}

impl Run {
    fn new(model: SapModel) -> Self {
        let mut app = SapGuiApp::new(Some(PathBuf::from("/nonexistent/result-display.s2k")));
        app.model = model;
        app.active_case = "S".into();
        app.dual_pane = false;
        app.left_proj = ViewProjection::PlanXY;
        app.left_plane = None;
        let mut run = Self { app, ctx: egui::Context::default(), time: 0.0, last: Default::default() };
        for _ in 0..3 {
            run.frame(vec![]);
        }
        run
    }

    fn frame_at(&mut self, time: f64, events: Vec<egui::Event>) {
        self.time = time;
        let input = egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, SCREEN)),
            time: Some(time),
            events,
            ..Default::default()
        };
        self.last = self.ctx.run(input, |ctx| self.app.render_ui(ctx));
    }

    fn frame(&mut self, events: Vec<egui::Event>) {
        self.frame_at(self.time + 0.05, events);
    }

    fn settle(&mut self) {
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

    fn click(&mut self, pos: Pos2) {
        self.click_at(pos, egui::PointerButton::Primary);
    }

    fn right_click(&mut self, pos: Pos2) {
        self.click_at(pos, egui::PointerButton::Secondary);
    }

    /// Rectangle of the first text with exactly `label` painted in the last frame.
    fn find(&self, label: &str) -> Rect {
        texts(&self.last)
            .into_iter()
            .find(|(t, _)| t == label)
            .unwrap_or_else(|| panic!("no text '{label}' in {:?}", texts(&self.last).iter().map(|t| t.0.clone()).collect::<Vec<_>>()))
            .1
    }

    fn has_text(&self, label: &str) -> bool {
        texts(&self.last).iter().any(|(t, _)| t == label)
    }

    fn click_text(&mut self, label: &str) {
        let r = self.find(label);
        self.click(r.center());
    }

    fn type_into(&mut self, label_before: &str, text: &str) {
        // The field right of its label.
        let r = self.find(label_before);
        self.click(Pos2::new(r.right() + 30.0, r.center().y));
        self.frame(vec![egui::Event::Text(text.to_string())]);
        self.settle();
    }

    fn view(&self) -> ViewTransform {
        self.app.last_view[0].expect("pane drawn").0
    }

    fn screen(&self, p: [f64; 3]) -> Pos2 {
        self.view().project(p)
    }

    fn open_deformed_form_from_the_menu(&mut self) {
        self.click_text("Display");
        self.click_text("Show Deformed Shape...");
        assert!(self.app.results.deformed.open, "the menu entry opens the form");
        assert!(self.has_text("Joint displacements are drawn in global axes."));
    }
}

fn displaced_joint_circles(out: &egui::FullOutput) -> Vec<Pos2> {
    flat(out)
        .into_iter()
        .filter_map(|s| match s {
            Shape::Circle(c) if c.fill == GREEN && (c.radius - 3.8).abs() < 1e-6 => Some(c.center),
            _ => None,
        })
        .collect()
}

#[test]
fn the_deformed_shape_form_draws_a_displacement_contour_and_its_legend() {
    let mut run = Run::new(plate());
    run.open_deformed_form_from_the_menu();
    run.click_text("Draw Displacement Contours on Area Objects");
    run.click_text("Ux");
    run.type_into("Max", "1.5");
    run.settle();
    // Display units: pick N, mm, C in the combo box.
    run.click_text("kN, m, C");
    run.click_text("N, mm, C");
    run.app.results.deformed.scale = 100.0;
    assert_eq!(run.app.results.deformed.max, "1.5");
    run.click_text("OK");
    let st = &run.app;
    assert_eq!(st.display_mode, DisplayMode::Deformed);
    assert_eq!(st.active_case, "S");
    assert_eq!(st.deform_scale, 100.0);
    assert!(!st.results.deformed.open);
    assert_eq!(
        st.results.contour,
        Contour::Displacement { component: DisplacementComponent::Ux, range: (0.0, 1.5), units: Units::NMmC }
    );
    // The model is untouched (units included).
    assert_eq!(st.model.present_units, Units::KnMC);
    // Range 0..1.5 mm = 0..0.0015 m; 0 / 1.5 is not "automatic": it is min 0, max 1.5.
    run.settle();
    let out = run.last.clone();
    let mesh_list = meshes(&out);
    // Two areas: two meshes of (4+1)^2 vertices each.
    let area_meshes: Vec<_> = mesh_list.iter().filter(|v| v.len() == 25).collect();
    assert_eq!(area_meshes.len(), 2, "{:?}", mesh_list.iter().map(|v| v.len()).collect::<Vec<_>>());
    // At each corner of A1 the vertex colour is the ramp colour of the joint's Ux
    // on the deformed shape (scale 100).
    let m = &run.app.model;
    let ux = |j: &str| m.case_results["S"].joint_displacements[j][0];
    let lo = 0.0;
    let hi = 0.0015;
    for (j, p) in [("1", [0.0, 0.0]), ("2", [1.0, 0.0]), ("3", [1.0, 1.0]), ("4", [0.0, 1.0])] {
        let uxj = ux(j);
        let uyj = m.case_results["S"].joint_displacements[j][1];
        let uzj = m.case_results["S"].joint_displacements[j][2];
        let pos = run.screen([p[0] + 100.0 * uxj, p[1] + 100.0 * uyj, 100.0 * uzj]);
        let want = contours::value_color(uxj, lo, hi, CONTOUR_ALPHA);
        let v = area_meshes[0]
            .iter()
            .find(|v| v.pos.distance(pos) < 1e-3)
            .unwrap_or_else(|| panic!("no mesh vertex at joint {j}"));
        assert_eq!(v.color, want, "joint {j}");
    }
    // Legend: title with the unit, 11 tick labels in mm, from 0 to 1.5.
    assert!(run.has_text("Displacement Ux [mm]"));
    let ticks: Vec<String> = (0..=10).map(|k| format!("{:.3}", 0.15 * k as f64)).collect();
    // 1.5 mm span: log10(1.5) = 0.18, three decimals.
    for t in &ticks {
        assert!(run.has_text(t), "tick {t}");
    }
    // The legend also shows no message: every area has all corner values.
    assert!(!texts(&run.last).iter().any(|(t, _)| t.contains("without a contour colour")));
}

#[test]
fn the_deformed_form_refuses_a_bad_range_and_keeps_the_application_unchanged() {
    let mut run = Run::new(plate());
    run.open_deformed_form_from_the_menu();
    run.click_text("Draw Displacement Contours on Area Objects");
    run.type_into("Min", "5");
    run.type_into("Max", "1");
    let (mode, case, contour) = (run.app.display_mode, run.app.active_case.clone(), run.app.results.contour);
    run.click_text("OK");
    assert!(run.app.results.deformed.open);
    assert!(run.app.results.deformed.error.contains("below"), "{}", run.app.results.deformed.error);
    run.settle();
    assert!(run.has_text(&run.app.results.deformed.error.clone()));
    assert_eq!((run.app.display_mode, run.app.active_case.clone(), run.app.results.contour), (mode, case, contour));
    run.click_text("Cancel");
    assert!(!run.app.results.deformed.open);
}

#[test]
fn a_corner_without_result_leaves_its_area_uncoloured_and_the_legend_says_so() {
    let mut m = plate();
    m.case_results.get_mut("S").unwrap().joint_displacements.shift_remove("5");
    let mut run = Run::new(m);
    run.app.display_mode = DisplayMode::Deformed;
    run.app.results.contour = Contour::Displacement { component: DisplacementComponent::Uz, range: (0.0, 0.0), units: Units::KnMC };
    run.settle();
    let meshes = meshes(&run.last);
    assert_eq!(meshes.len(), 1, "only A1 is a mesh");
    let note = texts(&run.last).into_iter().map(|t| t.0).find(|t| t.contains("without a contour colour")).expect("legend note");
    assert!(note.starts_with("1 area(s)") && note.contains("'5'"), "{note}");
    // A2 is a polygon with the no-contour fill, not a ramp colour.
    let fills: Vec<Color32> = flat(&run.last)
        .into_iter()
        .filter_map(|s| match s {
            Shape::Path(p) if p.closed => Some(p.fill),
            _ => None,
        })
        .collect();
    assert!(fills.contains(&no_contour_fill()), "{fills:?}");
}

#[test]
fn the_shell_form_draws_the_visible_face_in_plan_and_the_legend_shows_a_refusal() {
    let mut run = Run::new(with_srss(plate()));
    run.click_text("Display");
    run.click_text("Show Forces/Stresses");
    run.click_text("Shells...");
    assert!(run.app.results.shells.open);
    assert!(run.has_text("Element-centre values, no stress averaging."));
    run.click_text("Shell Stresses");
    run.click_text("S11");
    run.click_text("kN, m, C");
    run.click_text("N, mm, C");
    run.click_text("OK");
    assert_eq!(run.app.display_mode, DisplayMode::ShellContour(s11(Face::Top)));
    assert_eq!(
        run.app.results.contour,
        Contour::Shell { visible: true, range: (0.0, 0.0), units: Units::NMmC }
    );
    run.settle();
    // Plan view: A1 shows its top face (-130 kN/m2), A2 its bottom (230), two
    // flat polygons coloured with the ends of the range.
    let fills: Vec<Color32> = flat(&run.last)
        .into_iter()
        .filter_map(|s| match s {
            Shape::Path(p) if p.closed && p.fill != Color32::TRANSPARENT => Some(p.fill),
            _ => None,
        })
        .collect();
    let lo = contours::value_color(-130.0, -130.0, 230.0, CONTOUR_ALPHA);
    let hi = contours::value_color(230.0, -130.0, 230.0, CONTOUR_ALPHA);
    assert!(fills.contains(&lo) && fills.contains(&hi), "{fills:?}");
    assert!(run.has_text("Stress S11 (Visible Face) [N/mm2]"));
    assert!(run.has_text("-0.1300") && run.has_text("0.2300"));
    // In the 3D view the default camera looks from above, like the plan; turned to
    // look from below the faces swap: the colours stay the ends of the same range
    // but now on the other areas.
    let colour_of = |run: &Run, p: [f64; 3]| -> Color32 {
        // Fill of the polygon that contains the screen point of `p`.
        let at = run.screen(p);
        flat(&run.last)
            .into_iter()
            .filter_map(|s| match s {
                Shape::Path(pa) if pa.closed && pa.fill != Color32::TRANSPARENT => {
                    let r = Rect::from_points(&pa.points);
                    r.contains(at).then_some(pa.fill)
                }
                _ => None,
            })
            .next()
            .expect("polygon")
    };
    assert_eq!(colour_of(&run, [0.5, 0.5, 0.0]), lo);
    assert_eq!(colour_of(&run, [1.5, 0.5, 0.0]), hi);
    run.app.left_proj = ViewProjection::Perspective3D;
    run.app.yaw_rad = -0.65;
    run.app.pitch_rad = 0.42;
    run.settle();
    assert_eq!(colour_of(&run, [0.5, 0.5, 0.0]), lo, "from above A1 shows its top face");
    assert_eq!(colour_of(&run, [1.5, 0.5, 0.0]), hi);
    run.app.pitch_rad = -0.42;
    run.settle();
    assert_eq!(colour_of(&run, [0.5, 0.5, 0.0]), hi, "from below A1 shows its bottom face");
    assert_eq!(colour_of(&run, [1.5, 0.5, 0.0]), lo);
    run.app.left_proj = ViewProjection::PlanXY;
    // SMAX of the SRSS combination: refused, the reason is in the legend area.
    run.app.results.shells.open_with("SRSS1", Units::KnMC);
    run.app.results.shells.kind = ShellKind::Stress;
    run.app.results.shells.stress = ShellStress::SMax;
    run.app.results.shells.face = FaceChoice::Fixed(Face::Top);
    run.settle();
    run.click_text("OK");
    assert_eq!(run.app.active_case, "SRSS1");
    run.settle();
    let note = texts(&run.last).into_iter().map(|t| t.0).find(|t| t.starts_with("2 area(s)")).expect("refusal in the legend");
    assert!(note.contains("SMAX") && note.contains("multi-valued"), "{note}");
    assert!(run.has_text("Stress SMAX (Top) [kN/m2]"));
    assert!(meshes(&run.last).iter().all(|m| m.len() != 25));
}

#[test]
fn legacy_shell_shortcuts_still_draw_the_single_valued_contour() {
    let mut run = Run::new(plate());
    run.app.results.contour = Contour::Shell { visible: true, range: (1.0, 2.0), units: Units::NMmC };
    // The legacy menu entry resets any applied form contour.
    run.click_text("Display");
    run.click_text("Shell Contour: Membrane F11");
    assert_eq!(run.app.display_mode, DisplayMode::ShellContour(ShellQuantity::LEGACY[0]));
    assert_eq!(run.app.results.contour, Contour::Off);
    run.settle();
    // F11 = 10 everywhere: a constant field; the legend has the model unit.
    assert!(run.has_text("Membrane F11 [kN/m]"));
    // F6 returns to the plain deformed shape, without contour.
    run.app.results.contour = Contour::Displacement { component: DisplacementComponent::Ux, range: (0.0, 0.0), units: Units::KnMC };
    run.frame(vec![egui::Event::Key { key: egui::Key::F6, physical_key: None, pressed: true, repeat: false, modifiers: Default::default() }]);
    assert_eq!(run.app.display_mode, DisplayMode::Deformed);
    assert_eq!(run.app.results.contour, Contour::Off);
}

fn frame_model() -> SapModel {
    let mut m = plate();
    m.add_frame("F1", "1", "2", "FSEC1");
    m
}

#[test]
fn a_right_click_on_a_joint_of_the_deformed_shape_opens_the_read_out_and_a_frame_still_opens_its_inspector() {
    let mut run = Run::new(frame_model());
    run.app.display_mode = DisplayMode::Deformed;
    run.app.deform_scale = 1.0;
    run.app.results.readout_units = Some(Units::NMmC);
    run.settle();
    // Joint 3 at (1, 1): U = (0.001, 0.002, 0.03) m, local = global; shown in mm.
    let joint = run.screen([1.0 + 0.001, 1.0 + 0.002, 0.03]);
    run.right_click(joint);
    assert_eq!(run.app.results.readout_joint.as_deref(), Some("3"));
    assert!(run.app.inspected_frame.is_none());
    assert!(run.has_text("Joint Displacements"));
    assert!(run.has_text("Joint 3 (result S), local axes"));
    assert!(run.has_text("U1 [mm]") && run.has_text("1.000000"));
    assert!(run.has_text("U2 [mm]") && run.has_text("2.000000"));
    assert!(run.has_text("U3 [mm]") && run.has_text("30.000000"));
    // A left click elsewhere closes the window.
    run.click(Pos2::new(1200.0, 1000.0));
    assert!(run.app.results.readout_joint.is_none());
    run.settle();
    assert!(!run.has_text("Joint Displacements"));
    // Frame F1 from joint 1 (0,0) to joint 2 (1,0): its middle is 25 px or more
    // from either joint (the scale is about 400 px per m in this pane); a
    // right-click there opens the frame inspector, as before.
    let mid = run.screen([0.5, 0.0, 0.0]);
    assert!(mid.distance(run.screen([0.0, 0.0, 0.0])) > 30.0);
    run.right_click(mid);
    assert_eq!(run.app.inspected_frame.as_deref(), Some("F1"));
    assert!(run.app.results.readout_joint.is_none());
    run.app.inspected_frame = None;
    // Within 10 px of a joint the joint wins over the frame that ends there.
    let near_joint1 = run.screen([0.0, 0.0, 0.0]) + Vec2::new(6.0, 0.0);
    run.right_click(near_joint1);
    assert_eq!(run.app.results.readout_joint.as_deref(), Some("1"));
    assert!(run.app.inspected_frame.is_none());
    // Outside the deformed display a right-click near a joint is the frame's again.
    run.app.results.readout_joint = None;
    run.app.display_mode = DisplayMode::Undeformed;
    run.settle();
    run.right_click(run.screen([0.0, 0.0, 0.0]) + Vec2::new(6.0, 0.0));
    assert!(run.app.results.readout_joint.is_none());
    assert_eq!(run.app.inspected_frame.as_deref(), Some("F1"));
}

#[test]
fn the_read_out_is_refused_while_animating_and_without_results() {
    let mut run = Run::new(plate());
    run.app.display_mode = DisplayMode::Deformed;
    run.app.animate_deformed = true;
    run.settle();
    let joint = run.screen([1.0, 1.0, 0.0]);
    run.right_click(joint);
    assert!(run.app.results.readout_joint.is_none());
    assert_eq!(run.app.status_msg, "Joint displacements can not be displayed during animation.");
    assert!(!run.has_text("Joint Displacements"));
    // An open read-out disappears when the animation starts.
    run.app.animate_deformed = false;
    run.app.results.readout_joint = Some("3".into());
    run.settle();
    assert!(run.has_text("Joint Displacements"));
    run.app.animate_deformed = true;
    run.settle();
    assert!(!run.has_text("Joint Displacements"));
    assert!(run.app.results.readout_joint.is_none());
}

fn stepped() -> SapModel {
    let mut m = plate();
    let cr = m.case_results.get_mut("S").unwrap();
    cr.time_steps = vec![0.0, 0.1, 0.2];
    // Joint 2 moves 0 / 0.1 / 0.2 m along its local 1 over the three steps.
    for id in cr.joint_displacements.keys().cloned().collect::<Vec<_>>() {
        let steps = (0..3).map(|k| if id == "2" { [0.1 * k as f64, 0.0, 0.0, 0.0, 0.0, 0.0] } else { [0.0; 6] }).collect();
        cr.time_history_disp.insert(id, steps);
    }
    m
}

#[test]
fn the_animation_walks_through_the_steps_with_the_input_time_and_without_the_sine() {
    let mut run = Run::new(stepped());
    run.app.display_mode = DisplayMode::Deformed;
    run.app.deform_scale = 10.0;
    let t0 = 100.0;
    run.app.animate_deformed = true;
    run.frame_at(t0, vec![]);
    assert_eq!((run.app.active_step_idx, run.app.use_time_step), (0, true));
    assert!(run.app.results.stepping);
    let phase = run.app.anim_phase;
    for (dt, step) in [(0.025, 0), (0.075, 1), (0.125, 2), (0.175, 0), (0.225, 1)] {
        run.frame_at(t0 + dt, vec![]);
        assert_eq!(run.app.active_step_idx, step, "dt {dt}");
        // Joint 2 is drawn at x = 1 + 10 * 0.1 * step: the full scale, no sine.
        let want = run.screen([1.0 + 10.0 * 0.1 * step as f64, 0.0, 0.0]);
        let circles = displaced_joint_circles(&run.last);
        assert!(circles.iter().any(|c| c.distance(want) < 1e-3), "step {step}: {circles:?} vs {want:?}");
    }
    assert_eq!(run.app.anim_phase, phase, "the sine phase does not advance while stepping");
    // Stopping the animation keeps the last step and clears the clock.
    run.app.animate_deformed = false;
    run.frame_at(t0 + 5.0, vec![]);
    assert!(run.app.results.anim_start.is_none() && !run.app.results.stepping);
    assert_eq!(run.app.active_step_idx, 1);
}

#[test]
fn a_modal_result_keeps_the_sine_animation() {
    let mut m = plate();
    let cr = m.case_results.get_mut("S").unwrap();
    cr.modes = vec![ModalModeResult {
        mode_shape: cr.joint_displacements.clone(),
        ..Default::default()
    }];
    let mut run = Run::new(m);
    run.app.display_mode = DisplayMode::Deformed;
    run.app.animate_deformed = true;
    let p0 = run.app.anim_phase;
    run.frame_at(50.0, vec![]);
    run.frame_at(50.05, vec![]);
    assert!(!run.app.results.stepping);
    assert!(run.app.anim_phase > p0 + 0.15, "{} -> {}", p0, run.app.anim_phase);
    assert_eq!(run.app.active_step_idx, 0);
}

#[test]
fn a_rotated_joint_is_drawn_at_the_projection_of_base_plus_scale_times_r_u() {
    let mut m = frame_model();
    m.points.get_mut("2").unwrap().local_axes_deg = [90.0, 0.0, 0.0];
    m.case_results.get_mut("S").unwrap().joint_displacements.insert("2".into(), [0.4, 0.0, 0.0, 0.0, 0.0, 0.0]);
    let mut run = Run::new(m);
    run.app.display_mode = DisplayMode::Deformed;
    run.app.deform_scale = 1.5;
    run.settle();
    // New: (1, 0, 0) + 1.5 * R * (0.4, 0, 0) = (1, 0.6, 0). Old: (1.6, 0, 0).
    let new_pos = run.screen([1.0, 0.6, 0.0]);
    let old_pos = run.screen([1.6, 0.0, 0.0]);
    let circles = displaced_joint_circles(&run.last);
    assert!(circles.iter().any(|c| c.distance(new_pos) < 1e-3), "{circles:?} vs {new_pos:?}");
    assert!(!circles.iter().any(|c| c.distance(old_pos) < 1.0), "the old position is not drawn");
    // The deformed frame follows the joint: a green line ends at the new place.
    let ends: Vec<Pos2> = flat(&run.last)
        .into_iter()
        .filter_map(|s| match s {
            Shape::LineSegment { points, stroke } if stroke.color == egui::epaint::ColorMode::Solid(Color32::from_rgb(80, 255, 160)) => Some(points[1]),
            _ => None,
        })
        .collect();
    assert!(ends.iter().any(|p| p.distance(new_pos) < 1e-3), "{ends:?}");
}

#[test]
fn the_forms_reset_with_the_model() {
    let mut run = Run::new(plate());
    run.app.results.contour = Contour::Displacement { component: DisplacementComponent::Ux, range: (0.0, 0.0), units: Units::KnMC };
    run.app.results.readout_joint = Some("3".into());
    run.app.results.deformed.open = true;
    run.app.replace_model(SapModel::new(Units::KnMC), "new".into(), vec![]);
    assert_eq!(run.app.results.contour, Contour::Off);
    assert!(run.app.results.readout_joint.is_none() && !run.app.results.deformed.open);
    // A joint that is no longer in the model drops the read-out.
    run.app.model = plate();
    run.app.results.readout_joint = Some("3".into());
    run.app.model.points.shift_remove("3");
    run.app.sync_active_case();
    assert!(run.app.results.readout_joint.is_none());
}
