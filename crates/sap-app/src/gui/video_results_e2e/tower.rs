//! Part 4 of the video: the shell of revolution of a cooling tower made with
//! the model commands the Build window calls (curved frame, radial extrusion,
//! radial replication), then weight, modes, the modal tables window, the
//! mode-1 displacement contour, the joint read-out and the DEAD stresses.
//!
//! Geometry (m, kN): parabolic meridian from (58.5, 0, 0) through (36, 0, 110) to
//! (34, 0, 162) in `segments` straight frames, extruded in 36 steps of 10
//! degrees (shell WALL, t = 0.8), V columns (circular, D = 0.8) from base joints
//! at z = -12 up to the ring joints at z = 0, replicated every 10 degrees, base
//! joints fixed. Concrete E = 3e7 kN/m2, nu = 0.2, gamma = 25 kN/m3.
//!
//! The independent side of each check is named where it is made: the documented
//! curve of the command, statics (the weight of the planar facets and of the
//! columns, summed from the joint coordinates, against the vertical reaction),
//! the 36-fold symmetry (paired periods, equal UX and UY sums), the identities of
//! the manual (a Ritz starting load has 100 % static participation; the dynamic
//! participation of the unit acceleration UX is the sum of the UX mass ratios)
//! and the unit conversion kN/m2 -> N/mm2 = 1e-3 for the stresses.

use super::super::contours::{self, DisplacementComponent, StepSelection};
use super::super::display_units::{self, Dimension};
use super::super::modal_tables;
use super::super::result_display::{self, Contour, CONTOUR_ALPHA};
use super::super::shell_output::{self, Face, ShellQuantity, ShellStress};
use super::super::{DisplayMode, ViewProjection};
use super::harness::{close, geometry, meshes, open_build_tab, Run};
use eframe::egui::{Color32, Pos2};
use sap_model::radial::CurveKind;
use sap_model::{
    AreaSection, LoadCase, LoadCaseAssignment, LoadPattern, LoadPatternType, Material,
    ModalSubType, SapModel, ShellType,
};
use sap_units::Units;
use std::collections::BTreeSet;

const GAMMA: f64 = 25.0;
const T_WALL: f64 = 0.8;
const D_COL: f64 = 0.8;

/// The tower's materials and sections, no geometry: concrete CONC, the circular
/// sections COL (the meridian, removed by the extrusion) and VCOL (the V columns)
/// and the shell WALL.
fn base_model() -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    m.load_cases.clear();
    m.load_patterns.clear();
    m.frame_sections.clear();
    m.area_sections.clear();
    let mut conc = Material::default_steel_kip_in();
    conc.name = "CONC".into();
    conc.e = 3.0e7;
    conc.u = 0.2;
    conc.g = 3.0e7 / 2.4;
    conc.unit_weight = GAMMA;
    conc.unit_mass = GAMMA / 9.80665;
    m.materials.insert("CONC".into(), conc);
    m.frame_sections.insert("COL".into(), sap_section::make_circle("COL", "CONC", D_COL));
    m.frame_sections.insert("VCOL".into(), sap_section::make_circle("VCOL", "CONC", D_COL));
    m.area_sections.insert(
        "WALL".into(),
        AreaSection {
            name: "WALL".into(),
            material: "CONC".into(),
            shell_type: ShellType::ShellThin,
            thickness_membrane: T_WALL,
            thickness_bending: T_WALL,
            ..AreaSection::default()
        },
    );
    m
}

/// One V drawn into a model whose shell exists: a base joint at 5 degrees,
/// 62 m from the axis, z = -12, and two columns (section VCOL) to the ring
/// joints at 0 and 10 degrees.
fn draw_v_columns(m: &mut SapModel) {
    let ring = |m: &SapModel, deg: f64| -> String {
        let (s, c) = deg.to_radians().sin_cos();
        m.points
            .values()
            .find(|p| {
                p.z.abs() < 1e-9 && (p.x - 58.5 * c).abs() < 1e-6 && (p.y - 58.5 * s).abs() < 1e-6
            })
            .unwrap_or_else(|| panic!("no ring joint at {deg} degrees"))
            .name
            .clone()
    };
    let (p0, p1) = (ring(m, 0.0), ring(m, 10.0));
    let (s5, c5) = 5.0_f64.to_radians().sin_cos();
    let base = m.add_point("BASE", 62.0 * c5, 62.0 * s5, -12.0);
    m.add_frame("V1", base.clone(), p0, "VCOL");
    m.add_frame("V2", base, p1, "VCOL");
}

/// Fixed bases, the DEAD pattern and case and the Ritz case MODAL of at most
/// `modes` vectors started from UX and UY with a dynamic participation target of
/// `target_percent` each (0: no target).
fn add_supports_and_cases(m: &mut SapModel, modes: usize, target_percent: f64) {
    for p in m.points.values_mut() {
        if (p.z + 12.0).abs() < 1e-9 {
            p.restraint = [true; 6];
        }
    }
    assert_eq!(m.points.values().filter(|p| p.restraint[0]).count(), 36);
    m.load_patterns.insert(
        "DEAD".into(),
        LoadPattern { name: "DEAD".into(), pat_type: LoadPatternType::Dead, self_wt_multiplier: 1.0 },
    );
    m.load_cases.insert("DEAD".into(), LoadCase::linear_static("DEAD", "DEAD", 1.0));
    let mut modal = LoadCase::modal_eigen("MODAL", modes);
    modal.modal_sub_type = ModalSubType::Ritz;
    for load in ["UX", "UY"] {
        modal.loads.push(LoadCaseAssignment { load_name: load.into(), scale_factor: 1.0 });
        // The Ritz target control of the case editor.
        modal_tables::set_control(&mut modal, load, 0, target_percent).unwrap();
    }
    m.load_cases.insert("MODAL".into(), modal);
}

/// The reference tower, made by the `sap-model` commands the Build window calls
/// (curved frame, radial extrusion, radial replication) without the window.
pub fn tower(segments: usize, modes: usize, target_percent: f64) -> SapModel {
    let mut m = base_model();
    let meridian = m
        .curved_frame(
            [58.5, 0.0, 0.0],
            [34.0, 0.0, 162.0],
            [36.0, 0.0, 110.0],
            CurveKind::Parabolic,
            segments,
            "COL",
        )
        .unwrap();
    assert_eq!(meridian.frames_created.len(), segments);
    m.select_frames_by_section("COL", true).unwrap();
    let shell = m.extrude_lines_to_areas_radial([0.0, 0.0], 10.0, 36, 0.0, "WALL", true).unwrap();
    assert_eq!(shell.areas_created.len(), segments * 36);
    assert!(m.frames.is_empty());
    draw_v_columns(&mut m);
    m.select_frames_by_section("VCOL", true).unwrap();
    let rep = m.replicate_radial([0.0, 0.0], 10.0, 35).unwrap();
    assert_eq!(rep.frames_created.len(), 70);
    add_supports_and_cases(&mut m, modes, target_percent);
    m
}

/// The tower made in the Build window by pointer clicks and typed numbers: the
/// Curved Frame tab (Parabolic, start, end and third point, segments, "Draw
/// curved frame"), the Select tab ("Select frames"), the Radial tab (angle step 10,
/// 36 steps, "Delete source frames", "Extrude to areas"), then, after the V
/// columns are drawn into the model, the Select tab on the section VCOL and the
/// Radial tab ("Copies" 35, "Replicate radially").
pub fn tower_in_the_window(segments: usize, modes: usize, target_percent: f64) -> Run {
    let mut run = Run::new(base_model());
    open_build_tab(&mut run, "Curved Frame");
    run.click_text("Parabolic");
    run.set_number_at("Start X", 0, "58.5");
    run.set_number_at("End X", 0, "34");
    run.set_number_at("End X", 2, "162");
    run.set_number_at("Third point X", 0, "36");
    run.set_number_at("Third point X", 2, "110");
    run.set_number("Segments", &segments.to_string());
    run.click_text("Draw curved frame");
    assert!(run.app.status_msg.starts_with("Curved frame: created"), "{}", run.app.status_msg);
    run.click_text("Select / Diaphragm");
    run.click_text("Select frames");
    assert!(
        run.app.status_msg.starts_with(&format!("Selected {segments} frames")),
        "{}",
        run.app.status_msg
    );
    run.click_text("Radial / Extrude");
    run.set_number("Angle step (deg)", "10");
    run.set_number("Steps", "36");
    run.click_text("Delete source frames");
    run.click_text("Extrude to areas");
    assert!(run.app.status_msg.starts_with("Radial extrusion: created"), "{}", run.app.status_msg);
    assert_eq!(run.app.model.areas.len(), segments * 36);
    draw_v_columns(&mut run.app.model);
    run.click_text("Select / Diaphragm");
    run.choose("Frame section", "VCOL");
    run.click_text("Select frames");
    assert!(
        run.app.status_msg.starts_with("Selected 2 frames with section VCOL"),
        "{}",
        run.app.status_msg
    );
    run.click_text("Radial / Extrude");
    run.set_number("Copies", "35");
    run.click_text("Replicate radially");
    assert!(run.app.status_msg.starts_with("Radial replicate: created"), "{}", run.app.status_msg);
    add_supports_and_cases(&mut run.app.model, modes, target_percent);
    run
}

/// Area of a planar quadrilateral from its joints: half the cross product of the diagonals.
fn quad_area(c: &[[f64; 3]]) -> f64 {
    let d1 = [c[2][0] - c[0][0], c[2][1] - c[0][1], c[2][2] - c[0][2]];
    let d2 = [c[3][0] - c[1][0], c[3][1] - c[1][1], c[3][2] - c[1][2]];
    let cross = [
        d1[1] * d2[2] - d1[2] * d2[1],
        d1[2] * d2[0] - d1[0] * d2[2],
        d1[0] * d2[1] - d1[1] * d2[0],
    ];
    0.5 * (cross[0].powi(2) + cross[1].powi(2) + cross[2].powi(2)).sqrt()
}

/// Weight of the model from its joint coordinates: gamma t A over the facets
/// (planar trapezoids) and gamma A_col L over the columns.
pub fn expected_weight(m: &SapModel) -> f64 {
    let at = |id: &String| {
        let p = &m.points[id];
        [p.x, p.y, p.z]
    };
    let mut w = 0.0;
    for a in m.areas.values() {
        let c: Vec<[f64; 3]> = a.points.iter().map(at).collect();
        assert_eq!(c.len(), 4);
        w += GAMMA * T_WALL * quad_area(&c);
    }
    let area_col = std::f64::consts::PI * D_COL * D_COL / 4.0;
    for f in m.frames.values() {
        let (i, j) = (at(&f.point_i), at(&f.point_j));
        let len = ((i[0] - j[0]).powi(2) + (i[1] - j[1]).powi(2) + (i[2] - j[2]).powi(2)).sqrt();
        w += GAMMA * area_col * len;
    }
    w
}

fn analysed(segments: usize, modes: usize, target: f64) -> Run {
    let mut run = tower_in_the_window(segments, modes, target);
    run.app.active_case = "DEAD".into();
    assert!(run.app.execute_analysis(), "{}", run.app.status_msg);
    run.settle();
    run
}

/// Joints that belong to at least one area (the joints a contour is drawn on).
fn area_joints(m: &SapModel) -> BTreeSet<String> {
    m.areas.values().flat_map(|a| a.points.iter().cloned()).collect()
}

#[test]
fn the_meridian_follows_the_documented_parabola_through_the_three_points() {
    // The curve of the command, from its documentation (CSI "Curved Frame
    // Geometry"): with u the chord direction, L the chord length and t the chord
    // coordinate of the third point, point k is
    // start + s u + w s (L - s) / (t (L - t)), s = k L / n, w the part of
    // (third - start) perpendicular to the chord. The plan's reading (the radius a
    // quadratic of z) is a different parabola, with an axis parallel to Z.
    let n = 8;
    let m = tower(n, 10, 0.0);
    let (s, e, third): ([f64; 3], [f64; 3], [f64; 3]) = ([58.5, 0.0, 0.0], [34.0, 0.0, 162.0], [36.0, 0.0, 110.0]);
    let c = [e[0] - s[0], e[1] - s[1], e[2] - s[2]];
    let l = (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).sqrt();
    let u = [c[0] / l, c[1] / l, c[2] / l];
    let v = [third[0] - s[0], third[1] - s[1], third[2] - s[2]];
    let t = v[0] * u[0] + v[1] * u[1] + v[2] * u[2];
    let w = [v[0] - t * u[0], v[1] - t * u[1], v[2] - t * u[2]];
    // The curve passes through the third point (at s = t the factor is 1).
    for i in 0..3 {
        close_abs("third point", s[i] + t * u[i] + w[i], third[i], 1e-12);
    }
    let rings: Vec<(f64, f64)> = {
        let mut r: Vec<(f64, f64)> = m
            .points
            .values()
            .filter(|p| p.z > -1.0 && p.y.abs() < 1e-9 && p.x > 0.0)
            .map(|p| (p.z, p.x))
            .collect();
        r.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        r
    };
    assert_eq!(rings.len(), n + 1, "{rings:?}");
    for (k, (z, x)) in rings.iter().enumerate() {
        let sk = k as f64 * l / n as f64;
        let f = sk * (l - sk) / (t * (l - t));
        close_abs("ring z", *z, s[2] + sk * u[2] + f * w[2], 1e-9);
        close_abs("ring radius", *x, s[0] + sk * u[0] + f * w[0], 1e-9);
    }
    // Every joint of the shell is on the surface of revolution of that meridian.
    for p in m.points.values().filter(|p| p.z > -1.0) {
        let r = p.x.hypot(p.y);
        assert!(rings.iter().any(|(z, x)| (z - p.z).abs() < 1e-9 && (x - r).abs() < 1e-9), "{p:?}");
    }
}

fn close_abs(what: &str, got: f64, want: f64, tol: f64) {
    assert!((got - want).abs() <= tol, "{what}: got {got:e}, want {want:e}");
}

#[test]
fn the_dead_reaction_is_the_weight_of_the_facets_and_the_columns() {
    let run = analysed(8, 10, 0.0);
    let m = &run.app.model;
    let w = expected_weight(m);
    // Statics: the sum of the vertical reactions is the weight (up). The planar
    // facets and the straight columns are integrated exactly by the 2 x 2 Gauss
    // loading of the elements, so only roundoff remains.
    let cr = &m.case_results["DEAD"];
    let fz: f64 = cr.joint_reactions.values().map(|r| r[2]).sum();
    close("weight", fz, w, 1e-9);
    let (fx, fy): (f64, f64) =
        cr.joint_reactions.values().fold((0.0, 0.0), |a, r| (a.0 + r[0], a.1 + r[1]));
    assert!(fx.abs() <= 1e-9 * w && fy.abs() <= 1e-9 * w, "horizontal reaction {fx} {fy}");
    assert!(w > 1.0e5, "{w}");
}

#[test]
fn ten_ritz_vectors_come_in_pairs_and_the_modal_tables_window_shows_the_participation() {
    let mut run = analysed(8, 10, 0.0);
    let m = &run.app.model;
    let cr = &m.case_results["MODAL"];
    assert_eq!(cr.modes.len(), 10);
    // 36-fold symmetry, X and Y equivalent: the Ritz vectors from UX and UY
    // come in pairs of equal period. The first two pairs agree to 1e-12; the
    // later ones do not (see `all_ten_ritz_vectors_come_in_pairs_of_equal_period`).
    for pair in cr.modes.chunks(2).take(2) {
        close("paired periods", pair[0].period_sec, pair[1].period_sec, 1e-9);
    }
    let rows = modal_tables::mass_ratio_rows("MODAL", cr);
    let last = rows.last().unwrap();
    // Equal by the 90-degree symmetry of the tower and of the two starting
    // loads, up to roundoff: measured 9e-10 (translations) and 3e-8 (rotations).
    close("SumUX = SumUY", last.cumulative[0], last.cumulative[1], 1e-6);
    close("SumRX = SumRY", last.cumulative[3], last.cumulative[4], 1e-5);
    assert!(last.cumulative[0] > 0.99 && last.cumulative[0] <= 1.0 + 1e-9, "{}", last.cumulative[0]);
    assert!(last.cumulative[2] < 1e-9, "no vertical participation from lateral starting loads");
    // The identities of the manual: a starting load has 100 % static participation
    // (p. 380), and the dynamic participation of the unit acceleration UX is the
    // sum of its mass ratios.
    let items = modal_tables::item_rows("MODAL", cr);
    let ux = items.iter().find(|r| r.item == "UX").unwrap();
    close("UX static %", ux.static_percent, 100.0, 1e-8);
    close("UX dynamic %", ux.dynamic_percent, 100.0 * last.cumulative[0], 1e-10);
    let uy = items.iter().find(|r| r.item == "UY").unwrap();
    close("UY static %", uy.static_percent, 100.0, 1e-8);
    // The window, opened as the Analyze menu opens it, and what it paints.
    run.app.panel_modal_tables.open = true;
    run.settle();
    assert!(run.has_text("Modal Results Tables"));
    assert!(run.has_text("SumUX") && run.has_text("SumRZ"), "{:?}", run.shown());
    assert!(run.has_text(&modal_tables::mass_ratio_totals(&rows)), "{:?}", run.shown());
    for row in &rows {
        let cells = modal_tables::mass_ratio_cells(row, 5);
        assert!(run.has_text(&cells[3]), "period cell {} of mode {}", cells[3], row.mode);
    }
    let totals = format!("UX {:.2} %", 100.0 * last.cumulative[0]);
    assert!(run.texts().iter().any(|t| t.contains(&totals)), "{totals}");
    // Second tab: one row per item; UX static 100.
    run.click_text("Modal Load Participation Ratios");
    let cells = modal_tables::item_cells(ux, 3);
    assert_eq!(cells[0..3], ["MODAL", "Acceleration", "UX"]);
    assert_eq!(cells[3], "100.000");
    assert!(run.has_text(&cells[3]) && run.has_text(&cells[4]), "{:?}", run.shown());
}

#[test]
fn a_99_percent_target_stops_the_ritz_vectors_when_it_is_reached() {
    let mut run = analysed(8, 10, 99.0);
    let m = &run.app.model;
    let cr = &m.case_results["MODAL"];
    // Two vectors from each starting load reach 99.81 % (the sum of the mass
    // ratios, independent of the participation table), so four vectors are made
    // although ten were allowed.
    let rows = modal_tables::mass_ratio_rows("MODAL", cr);
    assert_eq!(cr.modes.len(), 4);
    let last = rows.last().unwrap();
    assert!(last.cumulative[0] >= 0.99 && last.cumulative[1] >= 0.99);
    let first_two = rows[1].cumulative[0] + rows[1].cumulative[1];
    assert!(first_two < 1.98 && rows[1].cumulative[0].min(rows[1].cumulative[1]) < 0.99,
        "two vectors alone do not reach 99 %: {first_two}");
    run.app.panel_modal_tables.open = true;
    run.settle();
    run.click_text("Modal Load Participation Ratios");
    let line = format!(
        "UX: target 99.0 %, reached {:.1} %: target reached (4 vectors in all, 2 from this load)",
        100.0 * last.cumulative[0]
    );
    assert!(run.has_text(&line), "{line} in {:?}", run.shown());
}

#[test]
fn twenty_vectors_collect_more_mass_than_ten() {
    let ten = analysed(8, 10, 0.0);
    let twenty = analysed(8, 20, 0.0);
    let sum = |r: &Run, d: usize| {
        r.app.model.case_results["MODAL"].modes.iter().map(|x| x.modal_mass_ratios[d]).sum::<f64>()
    };
    assert_eq!(twenty.app.model.case_results["MODAL"].modes.len(), 20);
    assert!(sum(&twenty, 0) > sum(&ten, 0), "{} {}", sum(&twenty, 0), sum(&ten, 0));
    assert!(sum(&twenty, 0) <= 1.0 + 1e-9);
    // The X and Y sums of this run agree to 3e-9 (translations) and 6e-8
    // (rotations); that is roundoff of a longer Ritz sequence and is not asserted.
}

// Only the first two pairs of periods are asserted equal (above). Ritz vectors
// are not converged modes: measured on this tower, the pairs started from UX
// and UY agree to 2e-13 for the first two pairs and differ by 1.3e-6 to 1.6e-5
// (relative) for pairs 3 to 5 of a 10-vector run, and by up to 3e-3 in a
// 20-vector run. That run also holds three pairs with periods above the
// first loaded mode and UX/UY mass ratios of about 1e-26: other circumferential
// harmonics that roundoff brings into the sequence. They are modes of the
// structure, with no participation in the starting loads.

#[test]
fn the_mode_1_contour_colours_the_vertices_with_the_global_ux_of_the_mode_shape() {
    let mut run = analysed(8, 10, 0.0);
    run.app.active_case = "MODAL".into();
    run.settle();
    // The mode shape as stored by the solver (joint axes are global here).
    let shape = run.app.model.case_results["MODAL"].modes[0].mode_shape.clone();
    let drawn = area_joints(&run.app.model);
    let (lo, hi) = drawn
        .iter()
        .map(|j| shape[j][0])
        .fold((f64::MAX, f64::MIN), |a, v| (a.0.min(v), a.1.max(v)));
    assert!(hi > lo);
    // Show Deformed Shape, mode 1, contour of Ux, display units N, mm.
    run.click_text("Display");
    run.click_text("Show Deformed Shape...");
    run.click_text("Draw Displacement Contours on Area Objects");
    run.click_text("Ux");
    run.click_text("kN, m, C");
    run.click_text("N, mm, C");
    run.click_text("OK");
    assert_eq!(run.app.display_mode, DisplayMode::Deformed);
    assert_eq!(run.app.active_case, "MODAL");
    assert_eq!(
        run.app.results.contour,
        Contour::Displacement {
            component: DisplacementComponent::Ux,
            range: (0.0, 0.0),
            units: Units::NMmC
        }
    );
    run.app.left_proj = ViewProjection::Perspective3D;
    run.settle();
    assert!(run.has_text("Displacement Ux [mm]"), "{:?}", run.shown());
    // The range of the legend is the minimum and maximum of Ux over the joints
    // of the drawn areas, and the painted end ticks are those in mm.
    let spec = result_display::contour_spec(
        run.app.display_mode,
        run.app.results.contour,
        run.app.model.present_units,
    )
    .unwrap();
    let sel = StepSelection { mode_idx: 0, use_step: false, step_idx: 0 };
    let fc = result_display::frame_contour(&run.app.model, "MODAL", spec, sel, [0.0, 0.0, 1.0]);
    let r = fc.table.range.unwrap();
    close_abs("range lo", r.lo, lo, 1e-15);
    close_abs("range hi", r.hi, hi, 1e-15);
    let ticks = &fc.legend.ticks;
    assert_eq!(ticks.len(), 11);
    for t in ticks {
        assert!(run.has_text(t), "tick {t} not painted in {:?}", run.shown());
    }
    let (first, last): (f64, f64) = (ticks[0].parse().unwrap(), ticks[10].parse().unwrap());
    let span = (hi - lo) * 1000.0;
    assert!((first - lo * 1000.0).abs() <= 1e-3 * span, "{first} against {}", lo * 1000.0);
    assert!((last - hi * 1000.0).abs() <= 1e-3 * span, "{last} against {}", hi * 1000.0);
    // Painted mesh vertices: at each joint's drawn position the colour is the ramp
    // colour of that joint's Ux on the range [lo, hi] (joints that project to the
    // same screen point may differ, so the colour must be one of theirs).
    let scale = f64::from(run.app.deform_scale);
    let mut at_screen: Vec<(Pos2, Color32)> = Vec::new();
    for j in &drawn {
        let u = shape[j];
        let p = &run.app.model.points[j];
        let pos = run.screen([p.x + scale * u[0], p.y + scale * u[1], p.z + scale * u[2]]);
        at_screen.push((pos, contours::value_color(u[0], lo, hi, CONTOUR_ALPHA)));
    }
    let all = meshes(&run.last);
    let area_meshes: Vec<_> = all.iter().filter(|v| v.len() == 25).collect();
    assert_eq!(area_meshes.len(), 8 * 36);
    for mesh in &area_meshes {
        for corner in [0, 4, 20, 24] {
            let v = &mesh[corner];
            let colours: Vec<Color32> = at_screen
                .iter()
                .filter(|(p, _)| p.distance(v.pos) < 1e-2)
                .map(|(_, c)| *c)
                .collect();
            assert!(!colours.is_empty(), "corner vertex at {:?} is at no joint", v.pos);
            assert!(colours.contains(&v.color), "vertex colour {:?} not in {colours:?}", v.color);
        }
    }
    // Read-out of a roof joint (the last ring, theta = 0): U1..U3 of the mode shape in mm.
    let roof = run
        .app
        .model
        .points
        .values()
        .find(|p| p.z > 161.0 && p.y.abs() < 1e-6 && p.x > 0.0)
        .unwrap()
        .name
        .clone();
    let (header, lines) =
        result_display::readout_lines(&run.app.model, "MODAL", &roof, sel, Units::NMmC);
    assert!(header.contains(&roof) && header.contains("MODAL"));
    for (k, name) in ["U1 [mm]", "U2 [mm]", "U3 [mm]"].iter().enumerate() {
        let shown: f64 = lines.iter().find(|(l, _)| l == name).unwrap().1.parse().unwrap();
        let want = shape[&roof][k] * 1000.0;
        assert!((shown - want).abs() <= 5.0e-7, "{name}: {shown} against {want}");
    }
}

#[test]
fn the_dead_stresses_are_shown_in_mpa_through_the_shells_form() {
    let mut run = analysed(8, 10, 0.0);
    run.click_text("Display");
    run.click_text("Show Forces/Stresses");
    run.click_text("Shells...");
    run.click_text("Shell Stresses");
    run.click_text("S11");
    run.click_text("Top Face");
    run.click_text("kN, m, C");
    run.click_text("N, mm, C");
    run.click_text("OK");
    assert_eq!(
        run.app.display_mode,
        DisplayMode::ShellContour(ShellQuantity::Stress(ShellStress::S11, Face::Top))
    );
    run.settle();
    // The values behind the legend: stored top-face S11 (kN/m2) times 1e-3 = N/mm2.
    let m = &run.app.model;
    let q = ShellQuantity::Stress(ShellStress::S11, Face::Top);
    let vals: Vec<f64> =
        m.areas.keys().map(|a| shell_output::area_value(m, "DEAD", a, q).unwrap()).collect();
    assert_eq!(vals.len(), 8 * 36);
    let (lo, hi) = vals.iter().fold((f64::MAX, f64::MIN), |a, v| (a.0.min(*v), a.1.max(*v)));
    assert!(hi > lo);
    let stored_lo =
        m.case_results["DEAD"].area_results.values().map(|r| r.s11_top).fold(f64::MAX, f64::min);
    close("lowest stored S11 top", lo, stored_lo, 1e-15);
    let spec = result_display::contour_spec(run.app.display_mode, run.app.results.contour, m.present_units)
        .unwrap();
    let fc = result_display::frame_contour(m, "DEAD", spec, Default::default(), [0.0, 0.0, 1.0]);
    let ticks: Vec<f64> = fc.legend.ticks.iter().map(|t| t.parse().unwrap()).collect();
    // Model units kN/m2 times the display factor, which is the constant 1e-3.
    let factor = display_units::factor(Dimension::Stress, Units::KnMC, Units::NMmC);
    close("display factor kN/m2 -> N/mm2", factor, 1.0e-3, 1e-12);
    let span = (hi - lo) * 1e-3;
    assert!((ticks[0] - lo * 1e-3).abs() <= 1e-3 * span, "{} against {}", ticks[0], lo * 1e-3);
    assert!((ticks[10] - hi * 1e-3).abs() <= 1e-3 * span, "{} against {}", ticks[10], hi * 1e-3);
    let r = fc.table.range.unwrap();
    close("range lo in N/mm2", r.lo * factor, lo * 1e-3, 1e-12);
    close("range hi in N/mm2", r.hi * factor, hi * 1e-3, 1e-12);
    assert!(run.has_text("Stress S11 (Top) [N/mm2]"), "{:?}", run.shown());
    for t in &fc.legend.ticks {
        assert!(run.has_text(t), "tick {t}");
    }
}

#[test]
#[ignore = "larger variant of the plan, 16 meridian segments (17 rings, 612 shell joints): 6 s in a debug build"]
fn the_larger_tower_has_the_same_weight_identity_and_paired_modes() {
    let run = analysed(16, 10, 0.0);
    let m = &run.app.model;
    assert_eq!(m.points.values().filter(|p| p.z > -1.0).count(), 17 * 36);
    let fz: f64 = m.case_results["DEAD"].joint_reactions.values().map(|r| r[2]).sum();
    close("weight", fz, expected_weight(m), 1e-9);
    for pair in m.case_results["MODAL"].modes.chunks(2).take(2) {
        close("paired periods", pair[0].period_sec, pair[1].period_sec, 1e-9);
    }
}

#[test]
fn the_build_window_makes_the_same_tower_as_the_commands() {
    // Through the application: the Curved Frame, Select and Radial tabs of the Build
    // window. Reference: the same commands called directly.
    let run = tower_in_the_window(8, 10, 0.0);
    let reference = tower(8, 10, 0.0);
    assert_eq!(run.app.model.areas.len(), 8 * 36);
    assert_eq!(run.app.model.frames.len(), 72);
    assert_eq!(geometry(&run.app.model), geometry(&reference));
    assert!(run.app.model.frames.values().all(|f| f.section == "VCOL"), "the meridian was deleted");
}

#[test]
fn smax_and_smin_of_dead_are_shown_in_mpa_and_obey_mohrs_circle() {
    let mut run = analysed(8, 10, 0.0);
    // Mohr's circle (independent of the form): the principal stresses are the extremes of
    // the plane stress, so SMAX >= max(S11, S22) >= min(S11, S22) >= SMIN, and their sum is the
    // trace S11 + S22, on every area and on both faces.
    let m = &run.app.model;
    for a in m.areas.keys() {
        for face in [Face::Top, Face::Bottom] {
            let v = |s| shell_output::area_value(m, "DEAD", a, ShellQuantity::Stress(s, face)).unwrap();
            let (s11, s22) = (v(ShellStress::S11), v(ShellStress::S22));
            let (smax, smin) = (v(ShellStress::SMax), v(ShellStress::SMin));
            let tol = 1e-9 * (s11.abs() + s22.abs() + smax.abs() + smin.abs() + 1.0);
            assert!(smax >= s11.max(s22) - tol && smin <= s11.min(s22) + tol, "{a} {face:?}");
            assert!((smax + smin - (s11 + s22)).abs() <= tol, "{a} {face:?}: trace");
        }
    }
    // Through the form: SMAX, top face, N, mm; the ticks are the extremes of the stored
    // values (kN/m2) times the display factor 1e-3.
    run.click_text("Display");
    run.click_text("Show Forces/Stresses");
    run.click_text("Shells...");
    run.click_text("Shell Stresses");
    run.click_text("SMAX");
    run.click_text("Top Face");
    run.click_text("kN, m, C");
    run.click_text("N, mm, C");
    run.click_text("OK");
    run.settle();
    assert!(run.has_text("Stress SMAX (Top) [N/mm2]"), "{:?}", run.shown());
    let m = &run.app.model;
    let q = ShellQuantity::Stress(ShellStress::SMax, Face::Top);
    let vals: Vec<f64> =
        m.areas.keys().map(|a| shell_output::area_value(m, "DEAD", a, q).unwrap()).collect();
    let (lo, hi) = vals.iter().fold((f64::MAX, f64::MIN), |a, v| (a.0.min(*v), a.1.max(*v)));
    assert!(hi > lo);
    let spec = result_display::contour_spec(run.app.display_mode, run.app.results.contour, m.present_units)
        .unwrap();
    let fc = result_display::frame_contour(m, "DEAD", spec, Default::default(), [0.0, 0.0, 1.0]);
    let ticks: Vec<f64> = fc.legend.ticks.iter().map(|t| t.parse().unwrap()).collect();
    let span = (hi - lo) * 1e-3;
    assert!((ticks[0] - lo * 1e-3).abs() <= 1e-3 * span, "{} against {}", ticks[0], lo * 1e-3);
    assert!((ticks[10] - hi * 1e-3).abs() <= 1e-3 * span, "{} against {}", ticks[10], hi * 1e-3);
    for t in &fc.legend.ticks {
        assert!(run.has_text(t), "tick {t}");
    }
}
