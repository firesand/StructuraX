//! Repairs of the solver seen through the application: the unstable joint named on the status
//! line, rigid end zones, and the area constitutive laws (plane strain, an orthotropic material
//! with a material angle) on a patch of one element.

use super::ext::{analyze, menu};
use super::harness::{close, Run};
use crate::gui::TableTab;
use sap_model::{
    AreaPropType, AreaSection, FramePropType, FrameSection, LoadCase, LoadPattern, LoadPatternType,
    Material, PlaneType, PointLoadForce, SapModel,
};
use sap_units::Units;

fn pattern(m: &mut SapModel, name: &str) {
    m.load_patterns.insert(
        name.into(),
        LoadPattern { name: name.into(), pat_type: LoadPatternType::Other, self_wt_multiplier: 0.0 },
    );
    m.load_cases.insert(name.into(), LoadCase::linear_static(name, name, 1.0));
}

fn force_x(m: &mut SapModel, joint: &str, pat: &str, fx: f64) {
    m.points.get_mut(joint).unwrap().loads_force.push(PointLoadForce {
        load_pat: pat.into(),
        csys: "GLOBAL".into(),
        force: [fx, 0.0, 0.0, 0.0, 0.0, 0.0],
    });
}

fn bare(units: Units) -> SapModel {
    let mut m = SapModel::new(units);
    m.load_cases.clear();
    m.load_patterns.clear();
    m.frame_sections.clear();
    m.area_sections.clear();
    m
}

fn steel(name: &str, e: f64, g: f64) -> Material {
    let mut mat = Material::default_steel_kip_in();
    mat.name = name.into();
    mat.e = e;
    mat.u = 0.2;
    mat.g = g;
    mat.unit_weight = 0.0;
    mat.unit_mass = 0.0;
    mat
}

fn section(area: f64) -> FrameSection {
    FrameSection {
        name: "SEC".into(),
        material: "MAT".into(),
        prop_type: FramePropType::General,
        area,
        i33: 1.0e-4,
        i22: 1.0e-4,
        torsional_j: 1.0e-4,
        as2: 0.0,
        as3: 0.0,
        ..FrameSection::default()
    }
}

// ------------------------------------------------------------------ the unstable joint (D03)

/// A bar along X in 3D, 4 m (kN, m): joint 1 held in U1, U2, U3 and joint 2 in U2, U3 only. The
/// rotation about the bar's own axis (R1) is free at both ends: a mechanism no load excites. The
/// third restraint, R1 at joint 1, removes it.
fn twisting_bar() -> SapModel {
    let mut m = bare(Units::KnMC);
    m.materials.insert("MAT".into(), steel("MAT", 2.0e8, 2.0e8 / 2.6));
    m.frame_sections.insert("SEC".into(), section(0.01));
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 4.0, 0.0, 0.0);
    m.points.get_mut("1").unwrap().restraint = [true, true, true, false, false, false];
    m.points.get_mut("2").unwrap().restraint = [false, true, true, false, false, false];
    m.add_frame("F1", "1", "2", "SEC");
    pattern(&mut m, "P");
    force_x(&mut m, "2", "P", 5.0);
    m
}

#[test]
fn an_unstable_model_names_the_joint_and_the_direction_on_the_status_line_and_the_third_restraint_cures_it() {
    let mut run = Run::new(twisting_bar());
    analyze(&mut run);
    let status = run.app.status_msg.clone();
    // The mechanism is the twist about the bar axis, R1 of one of its two joints; which of the
    // two the factorisation meets first is not specified, so either is accepted. The pivot is
    // R1 of the two joints is coupled by the torsion stiffness k = GJ/L alone, k [[1, -1], [-1, 1]],
    // which is singular in exact arithmetic (the twist is a rigid-body motion, not a near-zero
    // stiffness); the solver reports it through its pivot test.
    assert!(status.starts_with("Analysis Error: "), "{status}");
    assert!(status.contains("unstable at joint"), "{status}");
    assert!(
        status.contains("joint '1' R1") || status.contains("joint '2' R1"),
        "the message must name a joint and the direction R1: {status}"
    );
    assert!(!run.app.model.is_locked && run.app.model.case_results.is_empty(), "no results of a refused analysis");
    // The status line paints the message, with the joint and the direction in it.
    let painted = run.texts().into_iter().find(|t| t.starts_with("Analysis Error: Singular matrix")).unwrap_or_else(|| panic!("{:?}", run.shown()));
    assert!(painted.contains("joint '1' R1") || painted.contains("joint '2' R1"), "painted: {painted}");
    // The third restraint: R1 at joint 1. The 5 kN at joint 2 moves it by
    // P L / (E A) = 5 x 4 / (2e8 x 0.01) = 1e-5 m (closed form, bar held in X at joint 1).
    run.app.model.points.get_mut("1").unwrap().restraint[3] = true;
    analyze(&mut run);
    assert!(run.app.model.is_locked, "{}", run.app.status_msg);
    let u = run.app.model.case_results["P"].joint_displacements["2"][0];
    close("axial displacement", u, 1.0e-5, 1e-9);
}

// ------------------------------------------------------------------ rigid end zones (step 1b)

#[test]
fn rigid_end_zones_leave_the_full_length_flexible_for_axial_deformation() {
    // Analysis Reference p. 130: "The rigid-zone offsets never affect axial and torsional
    // deformation. The full element length is assumed to be flexible for these deformations."
    // A bar of 144 in (kip, in): E = 4320 ksi, A = 216 in2, P = 1000 kip at the free end:
    // delta = P L / (E A) = 1000 x 144 / (4320 x 216) = 0.154321 in, with offsets of 30 in at both
    // ends and the rigid-zone factor 1 (an element that took the offsets as rigid would give
    // 1000 x 84 / (4320 x 216) = 0.0900 in).
    let mut m = bare(Units::KipInF);
    m.materials.insert("MAT".into(), steel("MAT", 4320.0, 1800.0));
    m.frame_sections.insert("SEC".into(), FrameSection { i33: 1000.0, i22: 1000.0, torsional_j: 1000.0, ..section(216.0) });
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 144.0, 0.0, 0.0);
    m.points.get_mut("1").unwrap().restraint = [true; 6];
    m.add_frame("F1", "1", "2", "SEC");
    {
        let f = m.frames.get_mut("F1").unwrap();
        f.end_offset_i = 30.0;
        f.end_offset_j = 30.0;
        f.rigid_zone_factor = 1.0;
    }
    pattern(&mut m, "P");
    force_x(&mut m, "2", "P", 1000.0);
    let mut run = Run::new(m);
    analyze(&mut run);
    assert!(run.app.model.is_locked, "{}", run.app.status_msg);
    // Through the window: the joint displacement table paints U1 of joint 2 with 5 decimals.
    run.app.active_case = "P".into();
    run.app.tables_tab = TableTab::JointDisplacements;
    menu(&mut run, &["Display", "Show Database Tables... (Ctrl+T)"]);
    let want = 1000.0 * 144.0 / (4320.0 * 216.0);
    assert!(run.has_text(&format!("{want:.5}")), "{want:.5} in {:?}", run.shown());
    assert!(!run.has_text("0.09000"));
    let u = run.app.model.case_results["P"].joint_displacements["2"][0];
    assert!((u - want).abs() <= 5.0e-6, "{u} against {want}");
}

// ------------------------------------------------------------------ area patch

/// A rectangle of 2 m x 1 m, thickness 0.1 m, one area A of the section `AS` (kN, m), loaded by
/// the uniform traction p = 1000 kN/m2 in X on its two short edges (opposite senses: 50 kN at each
/// of the four joints, 1 + 4 at x = 0 pulling toward -X): a self-equilibrated nodal load that is the
/// consistent load of that traction on a bilinear element. Supports: joint 1 held in U1 and U2,
/// joint 2 in U2, and every joint in U3, R1, R2, R3 (the plane element has no stiffness there).
/// Three rigid-body motions are removed; the loads at joint 1 act on a held DOF, where the
/// reaction is the difference, so the uniform state is the solution (its rigid motion is chosen to
/// make U1 and U2 of joint 1 and U2 of joint 2 zero).
fn patch(units: Units, mat: Material, sec: AreaSection) -> SapModel {
    let mut m = bare(units);
    m.materials.insert(mat.name.clone(), mat);
    m.area_sections.insert("AS".into(), sec);
    for (id, x, y) in [("1", 0.0, 0.0), ("2", 2.0, 0.0), ("3", 2.0, 1.0), ("4", 0.0, 1.0)] {
        m.add_point(id, x, y, 0.0);
        m.points.get_mut(id).unwrap().restraint = [false, false, true, true, true, true];
    }
    m.points.get_mut("1").unwrap().restraint[0] = true;
    m.points.get_mut("1").unwrap().restraint[1] = true;
    m.points.get_mut("2").unwrap().restraint[1] = true;
    m.add_area("A", ["1", "2", "3", "4"].map(String::from).to_vec(), "AS");
    pattern(&mut m, "P");
    for (j, f) in [("1", -50.0), ("4", -50.0), ("2", 50.0), ("3", 50.0)] {
        force_x(&mut m, j, "P", f);
    }
    m
}

fn area_section(prop: AreaPropType, plane: PlaneType, material: &str, angle: f64) -> AreaSection {
    AreaSection {
        name: "AS".into(),
        material: material.into(),
        prop_type: prop,
        plane_type: plane,
        mat_angle_deg: angle,
        incompatible_modes: false, // the element refuses incompatible modes (checked below)
        thickness_membrane: 0.1,
        thickness_bending: 0.1,
        ..AreaSection::default()
    }
}

fn disp(run: &Run, joint: &str) -> [f64; 6] {
    run.app.model.case_results["P"].joint_displacements[joint]
}

#[test]
fn a_plane_strain_patch_gives_the_thickness_normal_stress_and_the_plane_strain_strains() {
    // Plane strain (Analysis Reference p. 218: e33 = 0): E = 3e7 kN/m2, nu = 0.3, uniaxial
    // traction p = 1000 kN/m2 (50 + 50 kN over the 1 m x 0.1 m edge: p = 100 / 0.1):
    //   e11 = (1 - nu^2) p / E = 0.91 x 1000 / 3e7 = 3.03333e-5, so U1 of joint 2 = 2 e11 = 6.06667e-5;
    //   e22 = -nu (1 + nu) p / E = -0.39 x 1000 / 3e7 = -1.3e-5, so U2 of joint 4 = e22 = -1.3e-5;
    //   s33 = nu (s11 + s22) = 300 kN/m2.
    let (e, nu, p) = (3.0e7, 0.3, 1000.0);
    let mut mat = steel("MAT", e, e / (2.0 * (1.0 + nu)));
    mat.u = nu;
    let m = patch(Units::KnMC, mat, area_section(AreaPropType::Plane, PlaneType::PlaneStrain, "MAT", 0.0));
    let mut run = Run::new(m);
    analyze(&mut run);
    assert!(run.app.model.is_locked, "{}", run.app.status_msg);
    close("U1 joint 2", disp(&run, "2")[0], 2.0 * (1.0 - nu * nu) * p / e, 1e-9);
    close("U2 joint 4", disp(&run, "4")[1], -nu * (1.0 + nu) * p / e, 1e-9);
    let r = &run.app.model.case_results["P"].area_results["A"];
    close("F11 = p t", r.f11, p * 0.1, 1e-9);
    // The stored S33 of a plane-strain area. No reader of it exists in the application or in the
    // API's tables (area S33 is in the stored `AreaResult` only; G43 is for wave 7), so it is read
    // from the stored result that the analysis of the application made.
    close("S33", r.s33, nu * p, 1e-9);
}

#[test]
fn an_orthotropic_area_with_a_material_angle_deforms_by_the_rotated_compliance_and_stores_element_axis_forces() {
    // The orthotropic material of Example 2-015, scaled to kN/m2: E1 = 3e7, E2 = 1.5e7,
    // nu_y = 0.2 (the document's, so nu_x = nu_y E1/E2 = 0.4), G12 = 6.6e6. Compliances (Jones,
    // Mechanics of Composite Materials, section 2.6):
    //   S11 = 1/E1, S22 = 1/E2, S12 = -nu_x/E1, S66 = 1/G12;
    //   S11' (rotated by theta) = S11 c^4 + (2 S12 + S66) s^2 c^2 + S22 s^4, etc.
    // Under the uniform stress (p, 0, 0) of the patch, in the element axes, the strains are
    // e11 = S'11 p, e22 = S'12 p, g12 = S'16 p with
    //   S'12 = (S11 + S22 - S66) s^2 c^2 + S12 (s^4 + c^4),
    //   S'16 = (2 S11 - 2 S12 - S66) s c^3 - (2 S22 - 2 S12 - S66) s^3 c,
    // and the joints move by U1(2) = 2 e11, U2(4) = e22, U1(4) = g12 (the rigid rotation chosen
    // by the supports). The stored resultants are those of the element axes: F11 = p t, F22 = 0,
    // F12 = 0, whatever the angle (the material axes would give F11 = p t c^2 ...).
    let (e1, e2, nu_y, g12, p, t) = (3.0e7, 1.5e7, 0.2, 6.6e6, 1000.0, 0.1);
    let nu_x = nu_y * e1 / e2;
    let (s11, s22, s12, s66) = (1.0 / e1, 1.0 / e2, -nu_x / e1, 1.0 / g12);
    let mut results = Vec::new();
    for angle in [0.0_f64, 30.0] {
        let (s, c) = angle.to_radians().sin_cos();
        let e11 = (s11 * c.powi(4) + (2.0 * s12 + s66) * s * s * c * c + s22 * s.powi(4)) * p;
        let e22 = ((s11 + s22 - s66) * s * s * c * c + s12 * (s.powi(4) + c.powi(4))) * p;
        let g = ((2.0 * s11 - 2.0 * s12 - s66) * s * c.powi(3) - (2.0 * s22 - 2.0 * s12 - s66) * s.powi(3) * c) * p;
        let mut mat = steel("ORTHO", e1, g12);
        mat.e_ortho = Some([e1, e2, 1.0]);
        mat.u_ortho = Some([nu_y, 0.0, 0.0]);
        mat.g_ortho = Some([g12, 1.1693e7, 1.1693e7]);
        mat.a_ortho = Some([0.0; 3]);
        let m = patch(Units::KnMC, mat, area_section(AreaPropType::Plane, PlaneType::PlaneStress, "ORTHO", angle));
        let mut run = Run::new(m);
        analyze(&mut run);
        assert!(run.app.model.is_locked, "{angle}: {}", run.app.status_msg);
        close(&format!("U1 joint 2 at {angle}"), disp(&run, "2")[0], 2.0 * e11, 1e-9);
        close(&format!("U2 joint 4 at {angle}"), disp(&run, "4")[1], e22, 1e-9);
        if g.abs() > 1e-12 {
            close(&format!("U1 joint 4 at {angle}"), disp(&run, "4")[0], g, 1e-9);
        }
        let r = &run.app.model.case_results["P"].area_results["A"];
        close("F11", r.f11, p * t, 1e-9);
        assert!(r.f22.abs() <= 1e-9 * p * t && r.f12.abs() <= 1e-9 * p * t, "F22 {} F12 {} at {angle}", r.f22, r.f12);
        results.push(e11);
    }
    // The angle matters: the rotated compliance differs from the unrotated by 39 %.
    assert!((results[1] / results[0] - 1.0).abs() > 0.3, "{results:?}");
}
