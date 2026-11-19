//! Wave 5 step 1a (D03), the wave-4 mechanism: a cylinder of 36 flat facets
//! under internal pressure whose supports leave a rigid rotation.
//!
//! References (none of them comes from the solver):
//! - A rigid rotation `u = w e_z x (x - c)` about the vertical line through
//!   c = (60, 60) is an exact null vector of the stiffness when the supports are
//!   UZ on the base ring, UY at theta = 0 and UX at theta = 90 degrees (their
//!   two restraint lines meet at c). The drilling penalty of the shell element
//!   acts on theta3 - (du2/dx - du1/dy)/2, which is zero for a rigid rotation.
//!   Adding UY at theta = 180 degrees removes the rotation.
//! - Sound cylinder: the lumped pressure p c h cos(pi/N) at a joint (c the
//!   chord, h the height) is balanced by the two facet forces F h along the
//!   facets, 2 F h sin(pi/N) = P, so F = p R cos(pi/N) = 59.77168 kip/in
//!   (R = 60 in, p = 1 kip/in2, N = 36).

use sap_model::*;
use sap_solver::{analyze_model, SolverError};

const RADIUS: f64 = 60.0;
const HEIGHT: f64 = 200.0;
const RINGS: usize = 8;
const FACETS: usize = 36;

pub fn cylinder(bending_thickness: f64, with_third_support: bool) -> SapModel {
    let mut m = SapModel::new(sap_units::Units::KipInF);
    m.load_cases.clear();
    m.load_patterns.clear();
    m.frame_sections.clear();
    m.area_sections.clear();
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.e = 29_000.0;
    mat.u = 0.3;
    mat.g = 29_000.0 / 2.6;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0;
    m.materials.insert("MAT".into(), mat);
    m.frame_sections
        .insert("COL".into(), sap_section::make_circle("COL", "MAT", 1.0));
    m.area_sections.insert(
        "SH".into(),
        AreaSection {
            name: "SH".into(),
            material: "MAT".into(),
            shell_type: ShellType::ShellThin,
            thickness_membrane: 1.0,
            thickness_bending: bending_thickness,
            ..AreaSection::default()
        },
    );
    for k in 0..=RINGS {
        m.add_point(format!("M{k}"), RADIUS, 0.0, HEIGHT * k as f64 / RINGS as f64);
    }
    for k in 0..RINGS {
        m.add_frame(format!("F{k}"), format!("M{k}"), format!("M{}", k + 1), "COL");
    }
    m.select_frames_by_section("COL", true).unwrap();
    m.extrude_lines_to_areas_radial([0.0, 0.0], 360.0 / FACETS as f64, FACETS, 0.0, "SH", true)
        .unwrap();
    assert_eq!(m.areas.len(), RINGS * FACETS);
    let ids: Vec<String> = m.points.keys().cloned().collect();
    for id in &ids {
        let (px, py, pz) = (m.points[id].x, m.points[id].y, m.points[id].z);
        if pz.abs() < 1e-9 {
            m.points[id].restraint[2] = true;
            let on_x_axis = py.abs() < 1e-6;
            if on_x_axis && (px - RADIUS).abs() < 1e-6 {
                m.points[id].restraint[1] = true;
            }
            if on_x_axis && (px + RADIUS).abs() < 1e-6 && with_third_support {
                m.points[id].restraint[1] = true;
            }
            if px.abs() < 1e-6 && (py - RADIUS).abs() < 1e-6 {
                m.points[id].restraint[0] = true;
            }
        }
    }
    m.load_patterns.insert(
        "P".into(),
        LoadPattern {
            name: "P".into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
    m.load_cases.insert("P".into(), LoadCase::linear_static("P", "P", 1.0));
    let names: Vec<String> = m.areas.keys().cloned().collect();
    for a in &names {
        let pts: Vec<[f64; 3]> = m.areas[a]
            .points
            .iter()
            .map(|id| [m.points[id].x, m.points[id].y, m.points[id].z])
            .collect();
        // Local 3 axis of the element: (p2 - p0) x (p3 - p1).
        let d1 = [pts[2][0] - pts[0][0], pts[2][1] - pts[0][1], pts[2][2] - pts[0][2]];
        let d2 = [pts[3][0] - pts[1][0], pts[3][1] - pts[1][1], pts[3][2] - pts[1][2]];
        let e3 = [
            d1[1] * d2[2] - d1[2] * d2[1],
            d1[2] * d2[0] - d1[0] * d2[2],
            d1[0] * d2[1] - d1[1] * d2[0],
        ];
        let c = [
            pts.iter().map(|p| p[0]).sum::<f64>() / 4.0,
            pts.iter().map(|p| p[1]).sum::<f64>() / 4.0,
        ];
        let outward = e3[0] * c[0] + e3[1] * c[1] > 0.0;
        m.areas[a].loads_uniform.push(AreaLoadUniform {
            load_pat: "P".into(),
            csys: "Local".into(),
            dir: 3,
            val: if outward { 1.0 } else { -1.0 },
        });
    }
    m
}

/// The rigid rotation is a null vector with components only U1, U2 and R3, and
/// R3 = w at every joint. Elimination runs in increasing DOF order, so the pivot
/// that vanishes is the last DOF with a nonzero null-vector component: R3 of
/// the last joint. (At the bending thickness 1.0 in the pivot test refuses it;
/// at 0.5 and 0.8 in it does not, see the report of step 1a.)
#[test]
fn r2_the_rigid_rotation_of_the_cylinder_is_named_at_its_last_joint() {
    let mut m = cylinder(1.0, false);
    let last = m.points.keys().last().unwrap().clone();
    let error = analyze_model(&mut m).unwrap_err();
    match &error {
        SolverError::Unstable { joint, dof, .. } => {
            assert_eq!(joint, &last);
            assert_eq!(dof, "R3");
        }
        other => panic!("expected an unstable-structure error, got {other:?}"),
    }
    assert!(error.to_string().starts_with("Singular matrix: the structure is unstable at joint"));
}

/// Hoop membrane force p R cos(pi/36) = 59.77168 kip/in at every area.
#[test]
fn r3_the_sound_cylinder_analyses_at_every_thickness() {
    let hoop = 1.0 * RADIUS * (std::f64::consts::PI / FACETS as f64).cos();
    assert!((hoop - 59.77168).abs() < 1e-5, "{hoop}");
    for t in [0.5, 0.8, 1.0] {
        let mut m = cylinder(t, true);
        analyze_model(&mut m).unwrap_or_else(|e| panic!("t = {t}: {e}"));
        let results = &m.case_results["P"];
        assert_eq!(results.area_results.len(), RINGS * FACETS);
        for (name, area) in &results.area_results {
            // Local 1 is horizontal for these extruded areas or vertical;
            // the hoop force is the larger of the two normal membrane forces.
            let (hoop_force, axial_force) = if area.f11.abs() > area.f22.abs() {
                (area.f11, area.f22)
            } else {
                (area.f22, area.f11)
            };
            assert!(
                (hoop_force - hoop).abs() <= 1e-6 * hoop,
                "t = {t}, area {name}: hoop force {hoop_force}"
            );
            assert!(axial_force.abs() <= 1e-6 * hoop, "t = {t}, area {name}: {axial_force}");
        }
    }
}
