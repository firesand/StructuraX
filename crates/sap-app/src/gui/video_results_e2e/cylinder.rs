//! Part 4 shell display against a closed form: a cylinder of 36 flat facets
//! under internal pressure (the model of the vendor's Example 2-010, without
//! its joint local axes), extruded in the Build window's radial-extrusion tab
//! (pointer clicks and typed numbers through `render_ui`), analysed by the
//! application's `execute_analysis` and read through the Shells form, the joint
//! read-out and the displacement contour.
//!
//! Independent side: the closed form of a polygonal ring of N = 36 flat facets
//! whose pressure is lumped to the joints (the same closed form reproduces the
//! four printed values of Example 2-010, see `vendor.rs`), and a reference model
//! built by the `sap-model` command without the window.
//!
//! Derivation (kip, in; R = 60, t = 1, E = 29000, nu = 0.3, p = 1 kip/in2, N = 36):
//!   a joint of the ring receives the half facets on both sides, each carrying
//!   a normal force p (c/2) h, with c = 2 R sin(pi/N) the chord and h the
//!   height; both normals make the angle pi/N with the radial direction, so the
//!   radial joint force is P = p c h cos(pi/N), and it is balanced by the two
//!   facet forces F h along the facets: 2 F h sin(pi/N) = P, so
//!   F = p R cos(pi/N) = 59.77168 kip/in (hoop membrane force per unit height);
//!   the stress is F / t = 59.77168 kip/in2 = 412.1112 N/mm2 (x 6.894757293168361);
//!   the elongation of a facet is F c / (E t), the radius grows by
//!   dR = F c / (E t 2 sin(pi/N)) = F R / (E t) = 0.12366555 in. The vertical
//!   force is zero (no axial load); the vertical strain is the Poisson
//!   contraction -nu F / (E t), so uz = -nu F z / (E t) with z from the base.
//!
//! Supports: UZ on the whole bottom ring, UY at theta = 0 and theta = 180
//! degrees, UX at theta = 90 degrees. (The plan's UY at 0 and UX at 90 degrees
//! alone leave a rigid rotation about the vertical axis through (60, 60): the
//! restraint lines x = 60 and y = 60 meet there; see `the_plans_two_horizontal_restraints_...`.)

use super::super::contours::{self, DisplacementComponent};
use super::super::display_units::{self, Dimension};
use super::super::result_display::{self, Contour};
use super::super::shell_output::{self, Face, ShellQuantity, ShellResultant, ShellStress};
use super::super::{DisplayMode, ViewProjection};
use super::harness::{close, geometry, meshes, open_build_tab, Run};
use sap_model::{
    AreaLoadUniform, AreaSection, LoadCase, LoadPattern, LoadPatternType, Material, PointObj,
    SapModel, ShellType,
};
use sap_units::Units;

pub const RADIUS: f64 = 60.0;
pub const HEIGHT: f64 = 200.0;
pub const THICKNESS: f64 = 1.0;
pub const YOUNG: f64 = 29_000.0;
pub const PRESSURE: f64 = 1.0;
pub const STEPS: usize = 36;
/// kip/in2 in N/mm2: 4448.2216152605 N / (25.4 mm)^2.
pub const KSI_TO_MPA: f64 = 6.894757293168361;

/// The model with the meridian only: `rings` straight frames of the section
/// COL from (R, 0, 0) to (R, 0, H), the area section SH and the material.
pub fn meridian(rings: usize) -> SapModel {
    let mut m = SapModel::new(Units::KipInF);
    m.load_cases.clear();
    m.load_patterns.clear();
    m.frame_sections.clear();
    m.area_sections.clear();
    let mut steel = Material::default_steel_kip_in();
    steel.name = "STEEL".into();
    steel.e = YOUNG;
    steel.u = 0.3;
    steel.g = YOUNG / 2.6;
    steel.unit_weight = 0.0;
    steel.unit_mass = 0.0;
    m.materials.insert("STEEL".into(), steel);
    m.frame_sections.insert("COL".into(), sap_section::make_circle("COL", "STEEL", 1.0));
    m.area_sections.insert(
        "SH".into(),
        AreaSection {
            name: "SH".into(),
            material: "STEEL".into(),
            shell_type: ShellType::ShellThin,
            thickness_membrane: THICKNESS,
            thickness_bending: THICKNESS,
            ..AreaSection::default()
        },
    );
    for k in 0..=rings {
        m.add_point(format!("M{k}"), RADIUS, 0.0, HEIGHT * k as f64 / rings as f64);
    }
    for k in 0..rings {
        m.add_frame(format!("F{k}"), format!("M{k}"), format!("M{}", k + 1), "COL");
    }
    m
}

/// Supports and the pressure of a model whose areas exist.
pub fn finish(m: &mut SapModel, plans_restraints_only: bool) {
    // Supports: UZ on the bottom ring; UY at theta = 0 and 180; UX at theta = 90.
    let ids: Vec<String> = m.points.keys().cloned().collect();
    for id in &ids {
        let (px, py, pz) = (m.points[id].x, m.points[id].y, m.points[id].z);
        if pz.abs() < 1e-9 {
            m.points[id].restraint[2] = true;
            let on_x_axis = py.abs() < 1e-6;
            if on_x_axis && (px - RADIUS).abs() < 1e-6 {
                m.points[id].restraint[1] = true;
            }
            if on_x_axis && (px + RADIUS).abs() < 1e-6 && !plans_restraints_only {
                m.points[id].restraint[1] = true;
            }
            if px.abs() < 1e-6 && (py - RADIUS).abs() < 1e-6 {
                m.points[id].restraint[0] = true;
            }
        }
    }
    // Pressure on the inside face: along local 3 when it points outward,
    // against it when it points inward, so the load pushes outward either way.
    m.load_patterns.insert(
        "P".into(),
        LoadPattern { name: "P".into(), pat_type: LoadPatternType::Other, self_wt_multiplier: 0.0 },
    );
    m.load_cases.insert("P".into(), LoadCase::linear_static("P", "P", 1.0));
    let names: Vec<String> = m.areas.keys().cloned().collect();
    for a in &names {
        let e3 = contours::area_local_axes(m, a).unwrap()[2];
        let c = area_centre(m, a);
        let outward = e3[0] * c[0] + e3[1] * c[1] > 0.0;
        m.areas[a].loads_uniform.push(AreaLoadUniform {
            load_pat: "P".into(),
            csys: "Local".into(),
            dir: 3,
            val: if outward { PRESSURE } else { -PRESSURE },
        });
    }
}

/// The reference cylinder: the `sap-model` command the Build window's
/// "Extrude to areas" calls, without the window.
pub fn cylinder(rings: usize) -> SapModel {
    cylinder_with(rings, false)
}

/// `plans_restraints_only`: without UY at theta = 180 degrees.
pub fn cylinder_with(rings: usize, plans_restraints_only: bool) -> SapModel {
    let mut m = meridian(rings);
    m.select_frames_by_section("COL", true).unwrap();
    let summary = m
        .extrude_lines_to_areas_radial([0.0, 0.0], 360.0 / STEPS as f64, STEPS, 0.0, "SH", true)
        .unwrap();
    assert_eq!(summary.areas_created.len(), rings * STEPS);
    assert!(m.frames.is_empty());
    finish(&mut m, plans_restraints_only);
    m
}

/// The application on the meridian, with the cylinder made in the Build window:
/// the Select tab ("Select frames" by section), then the Radial tab (angle step
/// 10, 36 steps, "Delete source frames", "Extrude to areas"), all by pointer
/// clicks and typed numbers.
pub fn cylinder_in_the_window(rings: usize) -> Run {
    let mut run = Run::new(meridian(rings));
    open_build_tab(&mut run, "Select / Diaphragm");
    run.click_text("Select frames");
    assert!(run.app.status_msg.starts_with(&format!("Selected {rings} frames")), "{}", run.app.status_msg);
    run.click_text("Radial / Extrude");
    run.set_number("Angle step (deg)", "10");
    run.set_number("Steps", "36");
    run.click_text("Delete source frames");
    run.click_text("Extrude to areas");
    assert!(run.app.status_msg.starts_with("Radial extrusion: created"), "{}", run.app.status_msg);
    assert_eq!(run.app.model.areas.len(), rings * STEPS);
    finish(&mut run.app.model, false);
    run
}

pub fn area_centre(m: &SapModel, a: &str) -> [f64; 3] {
    let pts = &m.areas[a].points;
    let mut c = [0.0; 3];
    for id in pts {
        let p = &m.points[id];
        c[0] += p.x / pts.len() as f64;
        c[1] += p.y / pts.len() as f64;
        c[2] += p.z / pts.len() as f64;
    }
    c
}

fn analysed() -> Run {
    let mut run = cylinder_in_the_window(8);
    run.app.active_case = "P".into();
    assert!(run.app.execute_analysis(), "{}", run.app.status_msg);
    run.settle();
    run
}

pub fn hoop_force() -> f64 {
    PRESSURE * RADIUS * (std::f64::consts::PI / STEPS as f64).cos()
}

/// Radial displacement R F / (E t) of the closed form.
pub fn radial_displacement() -> f64 {
    hoop_force() * RADIUS / (YOUNG * THICKNESS)
}

#[test]
fn the_build_window_extrudes_the_same_cylinder_as_the_command() {
    // Through the application: the Build window (Select tab, Radial tab). Reference:
    // `extrude_lines_to_areas_radial` called directly.
    let run = cylinder_in_the_window(8);
    let reference = cylinder(8);
    assert_eq!(run.app.model.areas.len(), 8 * 36);
    assert_eq!(run.app.model.points.len(), 9 * 36);
    assert!(run.app.model.frames.is_empty(), "'Delete source frames' removed the meridian");
    assert_eq!(geometry(&run.app.model), geometry(&reference));
    // The geometry itself: every facet has the chord and the height of the closed form.
    let m = &run.app.model;
    let chord = 2.0 * RADIUS * (5.0_f64.to_radians()).sin();
    for (name, a) in &m.areas {
        assert_eq!(a.points.len(), 4, "{name}");
        let p: Vec<_> = a.points.iter().map(|id| &m.points[id]).collect();
        for q in &p {
            close("radius", q.x.hypot(q.y), RADIUS, 1e-9);
        }
        let side = |i: usize, j: usize| {
            ((p[i].x - p[j].x).powi(2) + (p[i].y - p[j].y).powi(2) + (p[i].z - p[j].z).powi(2))
                .sqrt()
        };
        let mut sides = [side(0, 1), side(1, 2), side(2, 3), side(3, 0)];
        sides.sort_by(|a, b| a.partial_cmp(b).unwrap());
        close("chord", sides[0], chord, 1e-9);
        close("chord", sides[1], chord, 1e-9);
        close("height", sides[2], 25.0, 1e-9);
        close("height", sides[3], 25.0, 1e-9);
    }
    close("hoop force of the closed form", hoop_force(), 59.771_681_885_5, 1e-11);
    close("radial displacement of the closed form", radial_displacement(), 0.123_665_549, 1e-8);
}

#[test]
fn the_shells_form_shows_the_hoop_stress_in_mpa_and_the_axial_one_zero() {
    let mut run = analysed();
    assert_eq!(run.app.model.case_results["P"].area_results.len(), 8 * 36);
    // Which of S11 and S22 is the hoop component follows the area's own local
    // axes: the one whose local axis is horizontal.
    let a0 = run.app.model.areas.keys().next().unwrap().clone();
    let axes = contours::area_local_axes(&run.app.model, &a0).unwrap();
    let hoop_is_1 = axes[0][2].abs() < 1e-9;
    assert!(hoop_is_1 || axes[1][2].abs() < 1e-9, "{axes:?}");
    let (hoop, axial, fh, fa) = if hoop_is_1 {
        (ShellStress::S11, ShellStress::S22, ShellResultant::F11, ShellResultant::F22)
    } else {
        (ShellStress::S22, ShellStress::S11, ShellResultant::F22, ShellResultant::F11)
    };
    let f = hoop_force();
    // Pure function `shell_output::area_value`, every area: the membrane force
    // and the stress on both faces against the closed form (no bending: the
    // membrane state of a pressurised cylinder is free of moments).
    let m = &run.app.model;
    for a in m.areas.keys() {
        let v = |q| shell_output::area_value(m, "P", a, q).unwrap();
        close("hoop force", v(ShellQuantity::Resultant(fh)), f, 1e-6);
        for face in [Face::Top, Face::Bottom] {
            close("hoop stress", v(ShellQuantity::Stress(hoop, face)), f / THICKNESS, 1e-6);
            let s = v(ShellQuantity::Stress(axial, face));
            assert!(s.abs() <= 1e-6 * f, "axial stress {s} in {a}");
        }
        let axial_f = v(ShellQuantity::Resultant(fa));
        assert!(axial_f.abs() <= 1e-6 * f, "axial force {axial_f} in {a}");
    }
    // The form: Display > Show Forces/Stresses > Shells..., through pointer clicks.
    run.click_text("Display");
    run.click_text("Show Forces/Stresses");
    run.click_text("Shells...");
    assert!(run.app.results.shells.open);
    run.click_text("Shell Stresses");
    run.click_text(if hoop_is_1 { "S11" } else { "S22" });
    run.click_text("Kip, in, F");
    run.click_text("N, mm, C");
    run.click_text("OK");
    let q = ShellQuantity::Stress(hoop, Face::Top);
    assert_eq!(run.app.display_mode, DisplayMode::ShellContour(q));
    assert_eq!(
        run.app.results.contour,
        Contour::Shell { visible: true, range: (0.0, 0.0), units: Units::NMmC }
    );
    run.settle();
    // Painted text: the legend title with the form's unit, and the end ticks of
    // the automatic range, which is the single value of the uniform field,
    // 59.77168 kip/in2 x 6.894757293168361 = 412.1112 N/mm2 (`{:.3e}` is the
    // legend's notation for a range that narrow). The expected number is the
    // closed form times the physical constant, not the application's factor.
    let name = if hoop_is_1 { "S11" } else { "S22" };
    assert!(run.has_text(&format!("Stress {name} (Visible Face) [N/mm2]")), "{:?}", run.shown());
    let mpa = f / THICKNESS * KSI_TO_MPA;
    close("412.1112 MPa", mpa, 412.1112, 1e-6);
    let tick = format!("{mpa:.3e}");
    assert!(run.has_text(&tick), "tick {tick} in {:?}", run.shown());
    // The range of the legend (model units) times the application's own display
    // factor equals the closed form in N/mm2; the factor is the physical constant.
    let factor = display_units::factor(Dimension::Stress, Units::KipInF, Units::NMmC);
    close("display factor kip/in2 -> N/mm2", factor, KSI_TO_MPA, 1e-12);
    let spec = result_display::contour_spec(
        run.app.display_mode,
        run.app.results.contour,
        run.app.model.present_units,
    )
    .unwrap();
    let fc = result_display::frame_contour(
        &run.app.model,
        "P",
        spec,
        Default::default(),
        [0.0, 0.0, 1.0],
    );
    let r = fc.table.range.unwrap();
    close("range lo", r.lo * factor, mpa, 1e-6);
    close("range hi", r.hi * factor, mpa, 1e-6);
    // The model keeps its own units (the form has display units of its own).
    assert_eq!(run.app.model.present_units, Units::KipInF);
}

#[test]
fn the_radial_displacement_is_read_through_the_joint_read_out() {
    let mut run = analysed();
    let dr = radial_displacement();
    let mid = |p: &&PointObj| (p.z - 100.0).abs() < 1e-9;
    let at0 = run
        .app
        .model
        .points
        .values()
        .filter(mid)
        .find(|p| (p.x - RADIUS).abs() < 1e-6 && p.y.abs() < 1e-6)
        .unwrap()
        .name
        .clone();
    // Read-out of `result_display::readout_lines` (what the floating window
    // prints), in the model's units and in N, mm. Joint axes are global, so U1 at
    // theta = 0 is the radial displacement; six decimals are shown.
    let sel = Default::default();
    let model = &run.app.model;
    let (header, lines) = result_display::readout_lines(model, "P", &at0, sel, Units::KipInF);
    assert!(header.contains(&at0) && header.contains("(result P)"), "{header}");
    let u1: f64 = lines.iter().find(|(l, _)| l == "U1 [in]").unwrap().1.parse().unwrap();
    assert!((u1 - dr).abs() <= 5.0e-7, "read-out {u1} against {dr}");
    let (_, mm_lines) = result_display::readout_lines(model, "P", &at0, sel, Units::NMmC);
    let u1_mm: f64 = mm_lines.iter().find(|(l, _)| l == "U1 [mm]").unwrap().1.parse().unwrap();
    assert!((u1_mm - dr * 25.4).abs() <= 5.0e-7, "read-out {u1_mm} against {}", dr * 25.4);
    // Poisson contraction: uz = -nu F z / (E t) at mid height.
    let u3: f64 = lines.iter().find(|(l, _)| l == "U3 [in]").unwrap().1.parse().unwrap();
    let uz = -0.3 * hoop_force() * 100.0 / (YOUNG * THICKNESS);
    assert!((u3 - uz).abs() <= 5.0e-7, "read-out {u3} against {uz}");
    // The same through the right-click: the elevation view draws the joint at
    // theta = 0 on its deformed position; a right-click there opens the window.
    run.app.display_mode = DisplayMode::Deformed;
    run.app.deform_scale = 1.0;
    run.app.left_proj = ViewProjection::ElevationXZ;
    run.settle();
    let pos = run.screen([RADIUS + dr, 0.0, 100.0 + uz]);
    run.right_click(pos);
    assert_eq!(run.app.results.readout_joint.as_deref(), Some(at0.as_str()), "{}", run.app.status_msg);
    assert!(run.has_text("Joint Displacements"));
    // The painted value is the six-decimal text of the pure function.
    assert!(run.has_text(&format!("{u1:.6}")), "{:?}", run.shown());
    // Every mid-height joint moves radially by the same amount (global components).
    let cr = run.app.model.case_results["P"].clone();
    for p in run.app.model.points.values().filter(mid) {
        let u = contours::joint_displacement_global(&run.app.model, &cr, &p.name, sel).unwrap();
        let radial = (u[0] * p.x + u[1] * p.y) / p.x.hypot(p.y);
        close("radial displacement", radial, dr, 1e-6);
    }
}

#[test]
fn the_displacement_contour_along_area_3_is_the_radial_displacement_times_cos_5_degrees() {
    // Area 3 axis = the facet's normal, 5 degrees off the radial direction of
    // each of its joints, so every corner of every facet has |u.e3| = dr cos(5 deg)
    // (the vertical component is perpendicular to e3). The sign follows the
    // direction of local 3 (inward or outward), so the range is chosen from it.
    let mut run = analysed();
    let want = radial_displacement() * (5.0_f64.to_radians()).cos();
    let table = result_display::displacement_table(
        &run.app.model,
        "P",
        DisplacementComponent::Area3,
        Default::default(),
        (0.0, 0.0),
    );
    assert_eq!(table.entries.len(), 8 * 36);
    let sign = table.entries.values().next().unwrap().as_ref().unwrap()[0].signum();
    for (a, e) in &table.entries {
        for v in e.as_ref().unwrap_or_else(|e| panic!("{a}: {e}")) {
            close("u.e3", *v, sign * want, 1e-6);
        }
    }
    // Through the form: Show Deformed Shape, contour of Area 3 Axis, a range
    // around the closed-form value (0.1 to 0.14 in, or its negative); the painted mesh
    // vertices carry the colour of the closed-form value on that range, which lies mid-ramp.
    let (lo, hi) = if sign > 0.0 { (0.1, 0.14) } else { (-0.14, -0.1) };
    run.click_text("Display");
    run.click_text("Show Deformed Shape...");
    run.click_text("Draw Displacement Contours on Area Objects");
    run.click_text("Uz");
    run.click_text("Area 3 Axis");
    run.type_into("Min", &lo.to_string());
    run.type_into("Max", &hi.to_string());
    run.click_text("OK");
    assert_eq!(
        run.app.results.contour,
        Contour::Displacement {
            component: DisplacementComponent::Area3,
            range: (lo, hi),
            units: Units::KipInF
        }
    );
    run.app.left_proj = ViewProjection::Perspective3D;
    run.settle();
    assert!(run.has_text("Displacement Area 3 Axis [in]"), "{:?}", run.shown());
    let colour = contours::value_color(sign * want, lo, hi, result_display::CONTOUR_ALPHA);
    let t = (sign * want - lo) / (hi - lo);
    assert!(t > 0.3 && t < 0.7, "the closed-form value is mid-ramp: {t}");
    assert_ne!(colour, contours::value_color(lo, lo, hi, result_display::CONTOUR_ALPHA));
    assert_ne!(colour, contours::value_color(hi, lo, hi, result_display::CONTOUR_ALPHA));
    let all = meshes(&run.last);
    let area_meshes: Vec<_> = all.iter().filter(|v| v.len() == 25).collect();
    assert!(!area_meshes.is_empty(), "no area mesh painted");
    for v in area_meshes.iter().flat_map(|m| m.iter()) {
        assert_eq!(v.color, colour);
    }
}

#[test]
fn three_horizontal_restraints_hold_the_cylinder() {
    // The plan's supports, UY at theta = 0 and UX at theta = 90 degrees, leave a
    // rigid rotation: the lines x = 60 (restrained direction y through (60, 0))
    // and y = 60 (restrained direction x through (0, 60)) meet at (60, 60), the
    // centre of a rotation u = w x (r - c) that both restraints allow. A third
    // restraint, UY at theta = 180 degrees (the supports of `finish`), removes it.
    // Whether the solver reports the plan's mechanism as singular depends on
    // roundoff (it does at this wall thickness and not at a bending thickness of
    // 0.5 or 0.8), so nothing is asserted about that model.
    //
    // What is asserted is the closed form. A rigid motion of the horizontal plane
    // is (a, b, w): u = (a - w y, b + w x). A restrained direction d at (x, y)
    // gives the row [dx, dy, -dx y + dy x] of a 3 x 3 system. The vertical
    // restraints of the bottom ring leave no other rigid motion.
    let rows = |m: &SapModel| -> Vec<[f64; 3]> {
        let mut rows = Vec::new();
        for p in m.points.values() {
            for (dof, d) in [(0, [1.0, 0.0]), (1, [0.0, 1.0])] {
                if p.restraint[dof] {
                    rows.push([d[0], d[1], -d[0] * p.y + d[1] * p.x]);
                }
            }
        }
        rows
    };
    let det = |r: &[[f64; 3]]| {
        r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
            - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
            + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0])
    };
    let mut m = cylinder(8);
    let three = rows(&m);
    assert_eq!(three.len(), 3, "the three horizontal restraints of the plan");
    // UY at (R, 0), UX at (0, R), UY at (-R, 0): the determinant is 2 R.
    close("determinant of the rigid-motion system", det(&three), 2.0 * RADIUS, 1e-9);
    sap_solver::analyze_model(&mut m).expect("three horizontal restraints solve");
    // Without the third (the plan's two) the system has two rows for three
    // unknowns: the rotation about (R, R) is free.
    let two = rows(&cylinder_with(8, true));
    assert_eq!(two.len(), 2);
}
