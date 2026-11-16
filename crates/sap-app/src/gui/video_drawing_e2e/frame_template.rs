//! Test B: the part 2 frame from the custom 2D template, through the fields
//! of the New Model dialog, against the frame of the wave-2 test.

use super::models::*;
use crate::gui::{SapGuiApp, TemplateChoice};
use sap_model::{FrameType2D, SapModel};
use sap_solver::analyze_model;
use sap_units::Units;
use std::collections::BTreeSet;
use std::path::PathBuf;

fn mm3(c: [f64; 3]) -> [i64; 3] {
    c.map(|v| (v * 1000.0).round() as i64)
}

/// What the "Generate" button of the dialog does with the custom grid: a new
/// model in the present units, the template function on the two text fields,
/// then `replace_model`. (The button's own code is inline in the dialog, so
/// the test repeats its two calls; the analysis the button starts is not run
/// here.)
fn generate_from_dialog() -> SapGuiApp {
    let mut app = SapGuiApp::new(Some(PathBuf::from("/nonexistent/video-template.s2k")));
    app.model = SapModel::new(Units::KnMC);
    app.tpl_choice = TemplateChoice::Frame2D;
    app.tpl_frame2d_type = FrameType2D::PortalFrame;
    app.tpl_custom_grid = true;
    app.tpl_bays_text = "4, 6, 4".into();
    app.tpl_storeys_text = "3, 3, 3, 3".into();
    app.tpl_restraint = true;
    let mut m = SapModel::new(app.model.present_units);
    app.generate_custom_2d_frame(&mut m).unwrap();
    app.replace_model(m, "Generated Frame2D parametric model.".into(), Vec::new());
    app
}

/// The load factor at the end of the Accel push.
fn collapse_factor(mut m: SapModel) -> f64 {
    analyze_model(&mut m).unwrap();
    let steps = &m.case_results["PUSH-ACC"].static_steps;
    assert!(steps.len() >= 4, "{} saved states", steps.len());
    steps.last().unwrap().load_factor
}

#[test]
fn the_custom_template_frame_has_the_wave_2_geometry_and_collapses_at_the_same_factor() {
    let app = generate_from_dialog();
    let template = &app.model;

    // --- geometry: 20 joints, 16 columns, 12 beams, 4 bases ------------------
    assert_eq!(template.points.len(), 20);
    assert_eq!(template.frames.len(), 28);
    let (mut columns, mut beams) = (0, 0);
    for f in template.frames.values() {
        let (i, j) = (&template.points[&f.point_i], &template.points[&f.point_j]);
        if (i.z - j.z).abs() > 1e-9 {
            assert!((i.x - j.x).abs() < 1e-9 && (i.y - j.y).abs() < 1e-9);
            columns += 1;
        } else {
            assert!((i.y - j.y).abs() < 1e-9);
            beams += 1;
        }
        // The template carries no loads of its own.
        assert!(f.loads_dist.is_empty() && f.loads_point.is_empty());
    }
    assert_eq!((columns, beams), (16, 12));
    let based: Vec<_> = template.points.values().filter(|p| p.restraint == [true; 6]).collect();
    assert_eq!(based.len(), 4);
    assert!(based.iter().all(|p| p.z.abs() < 1e-9));
    assert_eq!(template.points.values().filter(|p| p.restraint != [false; 6]).count(), 4);
    let coords = |axis: usize| -> Vec<i64> {
        let set: BTreeSet<i64> = template
            .points
            .values()
            .map(|p| mm3([p.x, p.y, p.z])[axis])
            .collect();
        set.into_iter().collect()
    };
    assert_eq!(coords(0), [0, 4000, 10_000, 14_000]);
    assert_eq!(coords(2), [0, 3000, 6000, 9000, 12_000]);
    assert_eq!(coords(1), [0]);

    // The same joints and members as the wave-2 geometry.
    let reference = wave2_geometry();
    let joints = |m: &SapModel| -> BTreeSet<([i64; 3], [bool; 6])> {
        m.points.values().map(|p| (mm3([p.x, p.y, p.z]), p.restraint)).collect()
    };
    let members = |m: &SapModel| -> BTreeSet<([i64; 3], [i64; 3])> {
        m.frames
            .values()
            .map(|f| {
                let (i, j) = (&m.points[&f.point_i], &m.points[&f.point_j]);
                let (a, b) = (mm3([i.x, i.y, i.z]), mm3([j.x, j.y, j.z]));
                // Direction of drawing is not part of the geometry here.
                if a <= b { (a, b) } else { (b, a) }
            })
            .collect()
    };
    assert_eq!(joints(template), joints(&reference));
    assert_eq!(members(template), members(&reference));
    assert_eq!(members(template).len(), 28);

    // --- the wave-2 data on both geometries; the Accel push collapse factor --
    let mut from_template = template.clone();
    let mut from_wave2 = reference;
    wave2_data(&mut from_template);
    wave2_data(&mut from_wave2);
    let (a, b) = (collapse_factor(from_template), collapse_factor(from_wave2));
    eprintln!("Accel UX collapse factor: template {a}, wave-2 {b}");
    assert!(a.is_finite() && a > 0.1, "collapse factor {a}");
    assert!(
        (a - b).abs() <= 1e-9 * b.abs(),
        "template frame {a} against the wave-2 frame {b}"
    );
}
