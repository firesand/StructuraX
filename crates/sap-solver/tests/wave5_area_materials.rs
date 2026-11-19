//! Wave 5, package P2: constitutive laws of area elements (ledger D01, D20, D02, D31).
//!
//! Every expected number below is a closed form written out in the comment of its test, from
//! CSI Analysis Reference (Shell and Plane chapters, pp. 75, 193, 217-219) or from
//! Ugural 1981 as quoted in CSI Software Verification Example 2-015; none is read from the
//! element code.
use sap_model::*;
use sap_solver::analyze_model;
use sap_solver::area::{area_local_system, compute_area_local_stiffness, recover_area_result};
use sap_units::Units;

fn close(what: &str, got: f64, want: f64, tol: f64) {
    assert!(
        (got - want).abs() <= tol * want.abs().max(1e-12),
        "{what}: got {got:.12e}, want {want:.12e}"
    );
}

fn isotropic(e: f64, nu: f64) -> Material {
    let mut m = Material::default_steel_kip_in();
    m.name = "MAT".into();
    m.e = e;
    m.u = nu;
    m.g = e / (2.0 * (1.0 + nu));
    m.unit_weight = 0.0;
    m.unit_mass = 0.0;
    m
}

fn section(prop: AreaPropType, plane: PlaneType, t: f64) -> AreaSection {
    let mut s = AreaSection::default();
    s.name = "SEC".into();
    s.material = "MAT".into();
    s.prop_type = prop;
    s.plane_type = plane;
    s.thickness_membrane = t;
    s.thickness_bending = t;
    s
}

/// The distorted quadrilateral of the patch tests, in the global X-Y plane, counter-clockwise.
const QUAD: [[f64; 3]; 4] = [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [2.2, 1.5, 0.0], [-0.1, 1.2, 0.0]];

/// Local displacement vector (24) of the corners for a displacement field given in the
/// element axes (local 1 = X, local 2 = Y, local 3 = Z for a counter-clockwise quad in X-Y).
fn corner_displacements(field: impl Fn(f64, f64) -> [f64; 3]) -> Vec<f64> {
    let mut u = vec![0.0; 24];
    for (i, c) in QUAD.iter().enumerate() {
        let d = field(c[0], c[1]);
        u[6 * i] = d[0];
        u[6 * i + 1] = d[1];
        u[6 * i + 2] = d[2];
    }
    u
}

fn recover(sec: &AreaSection, mat: &Material, area: &AreaObj, u: &[f64]) -> AreaResult {
    let (_, _, xy) = area_local_system(&QUAD, 0.0);
    recover_area_result(area, sec, mat, &xy, u, Vec::new())
}

fn area_obj() -> AreaObj {
    AreaObj::new("A", vec!["1".into(), "2".into(), "3".into(), "4".into()], "SEC")
}

// ---------------------------------------------------------------- step 2a (D01) ----------

/// R1. Plane strain (Analysis Reference p. 218: e33 = 0, s33 from Poisson effects), E = 1000,
/// nu = 0.3, t = 1, the linear field u1 = 1e-3 x + 0.1e-3 y, u2 = 0.1e-3 x + 0.5e-3 y gives
/// e11 = 1e-3, e22 = 0.5e-3, g12 = 0.2e-3. With c = E / ((1 + nu)(1 - 2 nu)) = 1923.0769...:
///   s11 = c ((1 - nu) e11 + nu e22) = c 0.85e-3 = 1.634615384615
///   s22 = c (nu e11 + (1 - nu) e22) = c 0.65e-3 = 1.25
///   s33 = nu (s11 + s22)           = 0.865384615385
///   s12 = G g12 = 384.615384615 * 0.2e-3 = 0.076923076923.
/// A bilinear quadrilateral reproduces a linear field exactly, so only roundoff remains.
#[test]
fn plane_strain_patch_recovers_s11_s22_s33_s12() {
    let mat = isotropic(1000.0, 0.3);
    let sec = section(AreaPropType::Plane, PlaneType::PlaneStrain, 1.0);
    let u = corner_displacements(|x, y| [1e-3 * x + 0.1e-3 * y, 0.1e-3 * x + 0.5e-3 * y, 0.0]);
    let r = recover(&sec, &mat, &area_obj(), &u);
    close("F11", r.f11, 1.634_615_384_615, 1e-11);
    close("F22", r.f22, 1.25, 1e-11);
    close("S33", r.s33, 0.865_384_615_385, 1e-11);
    close("F12", r.f12, 0.076_923_076_923, 1e-11);
    close("S11", r.s11_top, 1.634_615_384_615, 1e-11);
    close("S22", r.s22_top, 1.25, 1e-11);
}

/// R3. The same field in a plane-STRESS section: s11 = E/(1-nu^2) (e11 + nu e22) =
/// 1098.901098901 * 1.15e-3 = 1.263736263736 and s22 = E/(1-nu^2) (e22 + nu e11) =
/// 1098.901098901 * 0.8e-3 = 0.879120879121, and s33 = 0.
#[test]
fn plane_stress_patch_keeps_its_values_and_has_no_s33() {
    let mat = isotropic(1000.0, 0.3);
    let sec = section(AreaPropType::Plane, PlaneType::PlaneStress, 1.0);
    let u = corner_displacements(|x, y| [1e-3 * x + 0.1e-3 * y, 0.1e-3 * x + 0.5e-3 * y, 0.0]);
    let r = recover(&sec, &mat, &area_obj(), &u);
    close("F11", r.f11, 1.263_736_263_736, 1e-11);
    close("F22", r.f22, 0.879_120_879_121, 1e-11);
    assert_eq!(r.s33, 0.0);
    // A shell section keeps s33 = 0 as well.
    let shell = section(AreaPropType::Shell, PlaneType::PlaneStress, 1.0);
    assert_eq!(recover(&shell, &mat, &area_obj(), &u).s33, 0.0);
}

/// R2. Anti-plane shear (p. 217): u3 = 1e-3 x on the distorted quad gives g13 = 1e-3,
/// g23 = 0, so s13 = G g13 = 384.615384615 * 1e-3 = 0.384615384615 (stored as the resultant
/// v13 = s13 t with t = 1) and s23 = 0. The consistent nodal forces of the stiffness are, by
/// the divergence theorem for a linear u3 (the integrand dN_i/dx detJ is bilinear, so 2x2
/// Gauss is exact), F3_i = G t 1e-3 * (y_{i+1} - y_{i-1}) / 2, summing to zero.
#[test]
fn plane_strain_has_anti_plane_shear_stiffness_and_recovery() {
    let mat = isotropic(1000.0, 0.3);
    let sec = section(AreaPropType::Plane, PlaneType::PlaneStrain, 1.0);
    let u = corner_displacements(|x, _| [0.0, 0.0, 1e-3 * x]);
    let r = recover(&sec, &mat, &area_obj(), &u);
    close("V13", r.v13, 0.384_615_384_615, 1e-11);
    assert!(r.v23.abs() <= 1e-14, "V23 {}", r.v23);

    let (_, _, xy) = area_local_system(&QUAD, 0.0);
    let (k, _) = compute_area_local_stiffness(&area_obj(), &sec, &mat, &xy);
    let g = 1000.0 / 2.6;
    let mut sum = 0.0;
    for i in 0..4 {
        let f: f64 = (0..24).map(|c| k[(6 * i + 2) * 24 + c] * u[c]).sum();
        let want = g * 1e-3 * 0.5 * (QUAD[(i + 1) % 4][1] - QUAD[(i + 3) % 4][1]);
        assert!((f - want).abs() <= 1e-12, "F3 at corner {i}: {f:e} vs {want:e}");
        sum += f;
    }
    assert!(sum.abs() <= 1e-12, "sum of F3 {sum:e}");
}

fn unit_square_model(plane: PlaneType) -> SapModel {
    let mut m = SapModel::new(Units::KipInF);
    m.materials.insert("MAT".into(), isotropic(1000.0, 0.3));
    let mut sec = section(AreaPropType::Plane, plane, 1.0);
    // The element has no incompatible bending modes; a section asking for them is refused.
    sec.incompatible_modes = false;
    m.area_sections.insert("SEC".into(), sec);
    m.load_cases.clear();
    m.load_patterns.clear();
    m.load_patterns.insert(
        "P".into(),
        LoadPattern {
            name: "P".into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
    m.load_cases
        .insert("P".into(), LoadCase::linear_static("P", "P", 1.0));
    for (name, x, y) in [("1", 0.0, 0.0), ("2", 1.0, 0.0), ("3", 1.0, 1.0), ("4", 0.0, 1.0)] {
        m.add_point(name, x, y, 0.0);
    }
    m.add_area("A", vec!["1".into(), "2".into(), "3".into(), "4".into()], "SEC");
    m
}

fn point_force(m: &mut SapModel, joint: &str, f: [f64; 3]) {
    m.points
        .get_mut(joint)
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: "P".into(),
            csys: "GLOBAL".into(),
            force: [f[0], f[1], f[2], 0.0, 0.0, 0.0],
        });
}

/// End to end, plane strain, unit square, t = 1, E = 1000, nu = 0.3. Uniaxial tension
/// s11 = 1 on the right edge (two nodal forces of 0.5), the lower-left joint pinned, the
/// upper-left joint free in Y: s22 = 0, e33 = 0 so s33 = nu s11 = 0.3, and
/// e11 = (1 - nu^2) s11 / E = 0.91e-3 (plane strain), e22 = -nu (1 + nu) s11 / E.
#[test]
fn plane_strain_uniaxial_tension_gives_s33_equal_nu_s11() {
    let mut m = unit_square_model(PlaneType::PlaneStrain);
    for (j, p) in m.points.iter_mut() {
        p.restraint = [false, false, true, true, true, true];
        if j == "1" {
            p.restraint = [true; 6];
        }
        if j == "4" {
            p.restraint = [true, false, true, true, true, true];
        }
    }
    point_force(&mut m, "2", [0.5, 0.0, 0.0]);
    point_force(&mut m, "3", [0.5, 0.0, 0.0]);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["P"];
    let a = &r.area_results["A"];
    close("F11", a.f11, 1.0, 1e-9);
    assert!(a.f22.abs() <= 1e-9, "F22 {}", a.f22);
    close("S33", a.s33, 0.3, 1e-9);
    close("u1 at x = 1", r.joint_displacements["2"][0], 0.91e-3, 1e-9);
    close("u2 at y = 1", r.joint_displacements["4"][1], -0.3 * 1.3 / 1000.0, 1e-9);
}

/// End to end anti-plane shear: the same square, plane strain, left edge fixed, forces
/// F3 = 0.5 on both right joints, so s13 = 1 and u3 = s13 L / G = 2.6 / 1000 at x = 1;
/// the resultant v13 = s13 t = 1.
#[test]
fn plane_strain_anti_plane_shear_end_to_end() {
    let mut m = unit_square_model(PlaneType::PlaneStrain);
    for (j, p) in m.points.iter_mut() {
        p.restraint = [false, false, false, true, true, true];
        if j == "1" || j == "4" {
            p.restraint = [true; 6];
        }
        if j == "2" || j == "3" {
            p.restraint = [true, true, false, true, true, true];
        }
    }
    point_force(&mut m, "2", [0.0, 0.0, 0.5]);
    point_force(&mut m, "3", [0.0, 0.0, 0.5]);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["P"];
    close("u3 at x = 1", r.joint_displacements["2"][2], 2.6e-3, 1e-9);
    close("V13", r.area_results["A"].v13, 1.0, 1e-9);
}

/// Symmetry of the elastic law (the strain-energy density has one second derivative, so
/// D12 = D21): in an isotropic section with unequal modifiers (f11 = 1, f22 = 0.5, m11 = 1,
/// m22 = 0.5) the force F11 caused by a unit e22 equals the force F22 caused by a unit e11,
/// and the moment M11 caused by a unit k22 equals the moment M22 caused by a unit k11. The
/// recovery must use the stiffness' coupling term sqrt(m_i m_j) for this to hold.
#[test]
fn recovered_resultants_come_from_a_symmetric_law() {
    let mat = isotropic(1000.0, 0.3);
    let mut sec = section(AreaPropType::Shell, PlaneType::PlaneStress, 1.0);
    sec.modifiers = [1.0, 0.5, 1.0, 1.0, 0.5, 1.0, 1.0, 1.0, 1.0, 1.0];
    let ux = corner_displacements(|x, _| [1e-4 * x, 0.0, 0.0]);
    let uy = corner_displacements(|_, y| [0.0, 1e-4 * y, 0.0]);
    let (rx, ry) = (recover(&sec, &mat, &area_obj(), &ux), recover(&sec, &mat, &area_obj(), &uy));
    close("F11(e22) = F22(e11)", ry.f11, rx.f22, 1e-12);
    // curvature k11 = 1e-4: w = x^2 1e-4 / 2, theta2 = -1e-4 x; k22 = 1e-4: w = y^2 1e-4 / 2, theta1 = +1e-4 y
    let mut kx = vec![0.0; 24];
    let mut ky = vec![0.0; 24];
    for (i, c) in QUAD.iter().enumerate() {
        kx[6 * i + 2] = 0.5e-4 * c[0] * c[0];
        kx[6 * i + 4] = -1e-4 * c[0];
        ky[6 * i + 2] = 0.5e-4 * c[1] * c[1];
        ky[6 * i + 3] = 1e-4 * c[1];
    }
    let (bx, by) = (recover(&sec, &mat, &area_obj(), &kx), recover(&sec, &mat, &area_obj(), &ky));
    close("M11(k22) = M22(k11)", by.m11, bx.m22, 1e-12);
}

// ---------------------------------------------------------------- step 2b (D20) ----------

/// Orthotropic material of Example 2-015, model A (Ugural 1981 eq. 6.1 as printed there):
/// E1 = 30000, E2 = 15000, nu12 = 0.2 (the document's nu_y), G12 = 6600 (k, in).
fn ortho_2_015() -> Material {
    let mut m = isotropic(30000.0, 0.2);
    m.e_ortho = Some([30000.0, 15000.0, 1.0]);
    m.u_ortho = Some([0.2, 0.0, 0.0]);
    m.g = 6600.0;
    m.g_ortho = Some([6600.0, 11693.0, 11693.0]);
    m.a_ortho = Some([0.0; 3]);
    m
}

/// R6a. Plane stress, orthotropic, t = 2, e11 = 1e-4 only. With nu_x = nu_y E1/E2 = 0.4 as in
/// the document, 1 - nu_x nu_y = 1 - 0.2^2 E1/E2 = 0.92:
///   Q11 = E1 / 0.92 = 32608.6956522, Q12 = nu_y E1 / 0.92 = 6521.73913043, Q22 = E2/0.92.
///   F = t Q e = (6.52173913043, 1.304347826087, 0).
/// Bending (kappa11 = 1e-4, t^3/12 = 2/3): M = (2.17391304348, 0.434782608696, 0).
#[test]
fn orthotropic_membrane_and_bending_at_zero_angle() {
    let mat = ortho_2_015();
    let sec = section(AreaPropType::Shell, PlaneType::PlaneStress, 2.0);
    let u = corner_displacements(|x, _| [1e-4 * x, 0.0, 0.0]);
    let r = recover(&sec, &mat, &area_obj(), &u);
    close("F11", r.f11, 6.521_739_130_43, 1e-9);
    close("F22", r.f22, 1.304_347_826_087, 1e-9);
    assert!(r.f12.abs() <= 1e-12, "F12 {}", r.f12);
    // w = +x^2/2 * 1e-4 has curvature w,xx = kappa11 = 1e-4, with theta2 = -dw/dx = -1e-4 x.
    let mut ub = vec![0.0; 24];
    for (i, c) in QUAD.iter().enumerate() {
        ub[6 * i + 2] = 0.5e-4 * c[0] * c[0];
        ub[6 * i + 4] = -1e-4 * c[0];
    }
    let rb = recover(&sec, &mat, &area_obj(), &ub);
    close("M11", rb.m11, 2.173_913_043_48, 1e-9);
    close("M22", rb.m22, 0.434_782_608_696, 1e-9);
    assert!(rb.m12.abs() <= 1e-12, "M12 {}", rb.m12);
}

/// Model B of Example 2-015 (isotropic E = 30000, nu = sqrt(nu_x nu_y) = sqrt(0.08) with the
/// stiffness modifiers m22 = 0.5 and m12 = 0.5645 stored in the vendor file; the test also sets
/// f22 = 0.5 so that the membrane matches) must give the stiffness of model A: D11 = E/(1-nu^2) = 32608.696 = Q11, D22 = 0.5 Q11 = 16304.348 = Q22 and
/// D12 = nu sqrt(0.5) E/(1-nu^2) = 0.2 * 32608.696 = 6521.739 = Q12 (algebra, no code
/// output). So the recovered resultants of a strain field agree between A and B. This needs
/// the recovery to use the stiffness' matrix, coupling term included.
#[test]
fn modified_isotropic_model_b_equals_orthotropic_model_a() {
    let a = ortho_2_015();
    let mut b = isotropic(30000.0, 0.08_f64.sqrt());
    b.g = 11693.171188;
    let mut sec_b = section(AreaPropType::Shell, PlaneType::PlaneStress, 2.0);
    sec_b.modifiers = [1.0, 0.5, 1.0, 1.0, 0.5, 1.0, 1.0, 1.0, 1.0, 1.0];
    let sec_a = section(AreaPropType::Shell, PlaneType::PlaneStress, 2.0);
    let mut u = corner_displacements(|x, y| [1e-4 * x + 0.3e-4 * y, 0.2e-4 * x + 0.5e-4 * y, 0.0]);
    for (i, c) in QUAD.iter().enumerate() {
        u[6 * i + 2] = -0.5e-4 * c[0] * c[0] - 0.25e-4 * c[1] * c[1];
        u[6 * i + 4] = 1e-4 * c[0];
        u[6 * i + 3] = -0.5e-4 * c[1];
    }
    let ra = recover(&sec_a, &a, &area_obj(), &u);
    let rb = recover(&sec_b, &b, &area_obj(), &u);
    for (what, x, y) in [
        ("F11", ra.f11, rb.f11),
        ("F22", ra.f22, rb.f22),
        ("M11", ra.m11, rb.m11),
        ("M22", ra.m22, rb.m22),
    ] {
        close(what, y, x, 1e-9);
    }
}

fn fixed_plate(mat: Material, thermal: (i32, f64), all_fixed: bool) -> SapModel {
    let mut m = SapModel::new(Units::KipInF);
    m.materials.insert("MAT".into(), mat);
    m.area_sections.insert(
        "SEC".into(),
        section(AreaPropType::Shell, PlaneType::PlaneStress, 2.0),
    );
    m.load_cases.clear();
    m.load_patterns.clear();
    m.load_patterns.insert(
        "T".into(),
        LoadPattern {
            name: "T".into(),
            pat_type: LoadPatternType::Temperature,
            self_wt_multiplier: 0.0,
        },
    );
    m.load_cases
        .insert("T".into(), LoadCase::linear_static("T", "T", 1.0));
    for (name, x, y) in [("1", 0.0, 0.0), ("2", 100.0, 0.0), ("3", 100.0, 100.0), ("4", 0.0, 100.0)] {
        m.add_point(name, x, y, 0.0);
        if all_fixed || name == "1" {
            m.points.get_mut(name).unwrap().restraint = [true; 6];
        }
    }
    m.add_area("A", vec!["1".into(), "2".into(), "3".into(), "4".into()], "SEC");
    m.areas.get_mut("A").unwrap().loads_temp.push(AreaLoadTemperature {
        load_pat: "T".into(),
        my_type: thermal.0,
        val: thermal.1,
        pattern_name: None,
    });
    m
}

/// Orthotropic thermal stress (a1 = a2 = 1e-5, dT = 40, so the free strain is 4e-4): a fully
/// restrained plate has s = -Q a dT (1, 1, 0), so with t = 2
///   F11 = -t (Q11 + Q12) a dT = -2 * 39130.4347826 * 4e-4 = -31.3043478261
///   F22 = -t (Q12 + Q22) a dT = -2 * 22826.0869565 * 4e-4 = -18.2608695652.
/// A plate clamped at one joint expands freely by a dT L = 0.04 in over L = 100 whatever the
/// moduli, with no stress.
#[test]
fn orthotropic_uniform_temperature_restrained_and_free() {
    let mut mat = ortho_2_015();
    mat.a = 1e-5;
    mat.a_ortho = Some([1e-5; 3]);
    let mut m = fixed_plate(mat.clone(), (1, 40.0), true);
    analyze_model(&mut m).unwrap();
    let a = &m.case_results["T"].area_results["A"];
    close("F11", a.f11, -31.304_347_826_1, 1e-9);
    close("F22", a.f22, -18.260_869_565_2, 1e-9);
    assert!(a.f12.abs() <= 1e-9);
    let mut m = fixed_plate(mat, (1, 40.0), false);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["T"];
    close("ux 2", r.joint_displacements["2"][0], 0.04, 1e-9);
    close("uy 4", r.joint_displacements["4"][1], 0.04, 1e-9);
    let a = &r.area_results["A"];
    assert!(a.f11.abs() <= 1e-8 && a.f22.abs() <= 1e-8, "F {} {}", a.f11, a.f22);
}

/// Orthotropic gradient through the thickness, free plate clamped at joint 1: the free
/// curvature is -a g in both directions whatever the stiffness, w = -a g (x^2 + y^2) / 2 with
/// a g = 1e-5 * 0.5 = 5e-6 per in: joint 2 (100, 0): uz = -0.025, joint 3: -0.05, no moments.
#[test]
fn orthotropic_gradient_curls_a_free_plate_without_moments() {
    let mut mat = ortho_2_015();
    mat.a = 1e-5;
    mat.a_ortho = Some([1e-5; 3]);
    let mut m = fixed_plate(mat, (2, 0.5), false);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["T"];
    close("uz 2", r.joint_displacements["2"][2], -0.025, 1e-9);
    close("uz 3", r.joint_displacements["3"][2], -0.05, 1e-9);
    let a = &r.area_results["A"];
    assert!(a.m11.abs() <= 1e-8 && a.m22.abs() <= 1e-8 && a.m12.abs() <= 1e-8);
}

/// Negative tests of the material check functions (they are not yet called by
/// `validate_analysis_inputs`, see the hand-back report; the caller names the area object): a zero modulus,
/// a Poisson's ratio that makes the stress-strain matrix indefinite (nu12 = 1.5 with
/// E1 = E2: the 2x2 minor is 1/E^2 (1 - 2.25) < 0), and unequal in-plane expansion with a
/// temperature load.
#[test]
fn material_check_functions_reject_invalid_orthotropic_constants_by_name() {
    use sap_solver::area::{validate_area_material, validate_area_thermal_material};
    assert!(validate_area_material("A", &ortho_2_015()).is_ok());
    assert!(validate_area_material("A", &isotropic(1000.0, 0.3)).is_ok());
    let mut m = ortho_2_015();
    m.e_ortho = Some([30000.0, 0.0, 1.0]);
    let e = validate_area_material("A7", &m).unwrap_err();
    assert!(e.contains("A7") && e.contains("positive"), "{e}");
    let mut m = ortho_2_015();
    m.e_ortho = Some([1000.0, 1000.0, 1000.0]);
    m.u_ortho = Some([1.5, 0.0, 0.0]);
    let e = validate_area_material("A8", &m).unwrap_err();
    assert!(e.contains("A8") && e.contains("positive definite"), "{e}");
    let mut m = ortho_2_015();
    m.g_ortho = Some([6600.0, -1.0, 1.0]);
    assert!(validate_area_material("A9", &m).is_err());
    let mut m = ortho_2_015();
    m.a_ortho = Some([1e-5, 2e-5, 1e-5]);
    let e = validate_area_thermal_material("A5", &m).unwrap_err();
    assert!(e.contains("A5") && e.contains("thermal"), "{e}");
    m.a_ortho = Some([1e-5; 3]);
    assert!(validate_area_thermal_material("A5", &m).is_ok());
}

/// The thin-plate formulation does not use g13 and g23 (Analysis Reference p. 193: they serve
/// the thick plate); the thin plate of Example 2-015 gives the same answer whatever they are.
#[test]
fn thin_shell_does_not_depend_on_g13_and_g23() {
    let mut a = ortho_2_015();
    a.g_ortho = Some([6600.0, 11693.0, 11693.0]);
    let mut b = ortho_2_015();
    b.g_ortho = Some([6600.0, 1.0, 1.0]);
    let mut sec = section(AreaPropType::Shell, PlaneType::PlaneStress, 2.0);
    sec.shell_type = ShellType::ShellThin;
    let (_, _, xy) = area_local_system(&QUAD, 0.0);
    let (ka, _) = compute_area_local_stiffness(&area_obj(), &sec, &a, &xy);
    let (kb, _) = compute_area_local_stiffness(&area_obj(), &sec, &b, &xy);
    assert_eq!(ka, kb);
    // the thick plate does use them
    sec.shell_type = ShellType::ShellThick;
    let (ta, _) = compute_area_local_stiffness(&area_obj(), &sec, &a, &xy);
    let (tb, _) = compute_area_local_stiffness(&area_obj(), &sec, &b, &xy);
    assert_ne!(ta, tb);
}

// ---------------------------------------------------------------- step 2c (D02) ----------

/// Recovery with the element axes the solver uses (`area_axes_angle`), the displacement field
/// being given in GLOBAL components of the corners and brought into those axes.
fn recover_in_solver_axes(
    sec: &AreaSection,
    mat: &Material,
    area: &AreaObj,
    field: impl Fn(f64, f64) -> [f64; 3],
) -> (AreaResult, Vec<f64>, Vec<[f64; 2]>) {
    let (axes, _, xy) = area_local_system(&QUAD, sap_solver::area::area_axes_angle(area, sec));
    let mut u = vec![0.0; 24];
    for (i, c) in QUAD.iter().enumerate() {
        let d = field(c[0], c[1]);
        for r in 0..3 {
            u[6 * i + r] = axes[r][0] * d[0] + axes[r][1] * d[1] + axes[r][2] * d[2];
        }
    }
    let r = recover_area_result(area, sec, mat, &xy, &u, Vec::new());
    (r, u, xy)
}

/// R5. An isotropic material has no preferred direction, so the material angle of its section
/// changes nothing (Analysis Reference p. 193: "This angle has no effect for isotropic material
/// properties") and the element axes are the area's own. Plane stress shell, E = 1000,
/// nu = 0.3, t = 1, the field e11 = 1e-3 in element axes (area angle 0):
///   F11 = E/(1-nu^2) e11 = 1.098901098901, F22 = nu F11 = 0.329670329670, F12 = 0,
/// for the material angles 0 and 37 degrees.
#[test]
fn isotropic_material_is_invariant_under_the_material_angle() {
    let mat = isotropic(1000.0, 0.3);
    for angle in [0.0, 37.0] {
        let mut sec = section(AreaPropType::Shell, PlaneType::PlaneStress, 1.0);
        sec.mat_angle_deg = angle;
        let (r, _, _) = recover_in_solver_axes(&sec, &mat, &area_obj(), |x, _| [1e-3 * x, 0.0, 0.0]);
        close(&format!("F11 at {angle}"), r.f11, 1.098_901_098_901_1, 1e-12);
        close(&format!("F22 at {angle}"), r.f22, 0.329_670_329_670_33, 1e-12);
        assert!(r.f12.abs() <= 1e-12, "F12 {} at {angle}", r.f12);
    }
}

/// Plane-stress orthotropic matrix of Example 2-015 rotated by `a` (Jones, Mechanics of
/// Composite Materials, the transformed reduced stiffness; written out here independently of
/// the element code): returns `(Q11, Q12, Q16)` of the rotated matrix.
fn rotated_q(a_deg: f64) -> (f64, f64, f64) {
    let den = 1.0 - 0.2 * 0.2 * 30000.0 / 15000.0; // 1 - nu_x nu_y = 0.92
    let (q11, q22, q12, q66) = (30000.0 / den, 15000.0 / den, 0.2 * 30000.0 / den, 6600.0);
    let (s, c) = a_deg.to_radians().sin_cos();
    let qb11 = q11 * c.powi(4) + 2.0 * (q12 + 2.0 * q66) * s * s * c * c + q22 * s.powi(4);
    let qb12 = (q11 + q22 - 4.0 * q66) * s * s * c * c + q12 * (s.powi(4) + c.powi(4));
    let qb16 = (q11 - q12 - 2.0 * q66) * c.powi(3) * s - (q22 - q12 - 2.0 * q66) * c * s.powi(3);
    (qb11, qb12, qb16)
}

/// R6. The material angle turns the orthotropic material of Example 2-015 inside the element:
/// e11 = 1e-4 in element axes, t = 2, material 1 axis 30 degrees from the element 1 axis,
///   F = t Q_bar e = t 1e-4 (Q_bar11, Q_bar12, Q_bar16) = (5.351413043, 1.659456522, 0.911021072)
/// with Q_bar11 = Q11 c^4 + 2 (Q12 + 2 Q66) s^2 c^2 + Q22 s^4 = 26757.065. At 0 degrees it is
/// (6.52173913, 1.304347826, 0). The recovery and the nodal forces of the stiffness (for a
/// constant stress field F, the consistent force at corner i is
///   f_i = 1/2 [F11 (y_{i+1} - y_{i-1}) - F12 (x_{i+1} - x_{i-1}),
///              F22 (x_{i-1} - x_{i+1}) + F12 (y_{i+1} - y_{i-1})]
/// from the traction on the two edges of the corner) must both give it.
#[test]
fn orthotropic_material_angle_rotates_the_material() {
    let mat = ortho_2_015();
    for (angle, want) in [
        (0.0, [6.521_739_130_43, 1.304_347_826_087, 0.0]),
        (30.0, [5.351_413_043, 1.659_456_522, 0.911_021_072]),
    ] {
        // the closed form of the test itself agrees with the printed numbers
        let (q11, q12, q16) = rotated_q(angle);
        for (k, q) in [q11, q12, q16].into_iter().enumerate() {
            close("closed form", 2.0e-4 * q, want[k], 1e-9);
        }
        let mut sec = section(AreaPropType::Shell, PlaneType::PlaneStress, 2.0);
        sec.mat_angle_deg = angle;
        let (r, u, xy) = recover_in_solver_axes(&sec, &mat, &area_obj(), |x, _| [1e-4 * x, 0.0, 0.0]);
        let tol = 1e-9;
        if want[2] == 0.0 {
            assert!(r.f12.abs() <= 1e-9, "F12 {} at {angle}", r.f12);
        } else {
            close(&format!("F12 at {angle}"), r.f12, want[2], tol);
        }
        close(&format!("F11 at {angle}"), r.f11, want[0], tol);
        close(&format!("F22 at {angle}"), r.f22, want[1], tol);
        // the stiffness gives the same stresses: nodal forces of the constant field
        let (k, _) = compute_area_local_stiffness(&area_obj(), &sec, &mat, &xy);
        for i in 0..4 {
            let (n, p) = ((i + 1) % 4, (i + 3) % 4);
            let (dx, dy) = (xy[n][0] - xy[p][0], xy[n][1] - xy[p][1]);
            let fx = 0.5 * (r.f11 * dy - r.f12 * dx);
            let fy = 0.5 * (r.f12 * dy - r.f22 * dx);
            let kx: f64 = (0..24).map(|c| k[(6 * i) * 24 + c] * u[c]).sum();
            let ky: f64 = (0..24).map(|c| k[(6 * i + 1) * 24 + c] * u[c]).sum();
            assert!((kx - fx).abs() <= 1e-8 && (ky - fy).abs() <= 1e-8, "corner {i} at {angle}: ({kx}, {ky}) vs ({fx}, {fy})");
        }
    }
}

/// The material angle turns the plate-bending and transverse-shear matrices the same way: a
/// curvature k11 = 1e-4 gives M = (2/3) 1e-4 (Q_bar11, Q_bar12, Q_bar16) (t = 2), and the shear
/// stiffness diag(g13, g23) = diag(6000, 12000) at 30 degrees is
/// [[g13 c^2 + g23 s^2, (g13 - g23) s c], [(g13 - g23) s c, g13 s^2 + g23 c^2]]
/// = [[7500, -2598.076], [-2598.076, 10500]], so that gamma13 = 1e-3, gamma23 = 0 gives
/// (V13, V23) = (5/6) t (7.5, -2.598076) = (12.5, -4.330127) (thick plate, t = 2).
#[test]
fn material_angle_rotates_bending_and_shear() {
    let mut mat = ortho_2_015();
    mat.g_ortho = Some([6600.0, 6000.0, 12000.0]);
    let mut sec = section(AreaPropType::Shell, PlaneType::PlaneStress, 2.0);
    sec.shell_type = ShellType::ShellThick;
    sec.mat_angle_deg = 30.0;
    let mut ub = vec![0.0; 24];
    for (i, c) in QUAD.iter().enumerate() {
        ub[6 * i + 2] = 0.5e-4 * c[0] * c[0];
        ub[6 * i + 4] = -1e-4 * c[0];
    }
    let (q11, q12, q16) = rotated_q(30.0);
    let rb = recover(&sec, &mat, &area_obj(), &ub);
    close("M11", rb.m11, 2.0 / 3.0 * 1e-4 * q11, 1e-9);
    close("M22", rb.m22, 2.0 / 3.0 * 1e-4 * q12, 1e-9);
    close("M12", rb.m12, 2.0 / 3.0 * 1e-4 * q16, 1e-9);
    // w = 1e-3 x, theta2 = -1e-3 gives gamma13 = dw/dx + theta2... a pure shear: w = 1e-3 x with
    // zero rotations has gamma13 = 1e-3 and gamma23 = 0.
    let mut us = vec![0.0; 24];
    for (i, c) in QUAD.iter().enumerate() {
        us[6 * i + 2] = 1e-3 * c[0];
    }
    let rs = recover(&sec, &mat, &area_obj(), &us);
    close("V13", rs.v13, 12.5, 1e-9);
    close("V23", rs.v23, -4.330_127_019, 1e-9);
}

/// Plane strain and the material angle: the thickness-normal stress of an orthotropic
/// plane-strain section is u13 s'11 + u23 s'22 in material axes (p. 75, e33 = 0). With
/// E1 = 3000, E2 = 1500, E3 = 2000, u12 = 0.2, u13 = 0.3, u23 = 0.1, G12 = 600, t = 1, the
/// uniform strain e11 = 1e-3 in element axes at a material angle of 0 gives
/// S'11 = 1/E1 - u13^2/E3 = 3.3333e-4 - 4.5e-5 = 2.88333e-4,
/// S'22 = 1/E2 - u23^2/E3 = 6.66667e-4 - 5e-6 = 6.61667e-4,
/// S'12 = -u12/E2 - u13 u23 / E3 = -1.33333e-4 - 1.5e-5 = -1.48333e-4; det = 1.4997e-7 ...
/// computed in the test from these closed forms: s' = Q e with Q = S'^-1, s33 = u13 s11 + u23 s22.
#[test]
fn orthotropic_plane_strain_s33_and_stiffness() {
    let mut mat = isotropic(3000.0, 0.2);
    mat.e_ortho = Some([3000.0, 1500.0, 2000.0]);
    mat.u_ortho = Some([0.2, 0.3, 0.1]);
    mat.g = 600.0;
    mat.g_ortho = Some([600.0, 500.0, 500.0]);
    let mut sec = section(AreaPropType::Plane, PlaneType::PlaneStrain, 1.0);
    let (s11, s22, s12) = (
        1.0 / 3000.0 - 0.3 * 0.3 / 2000.0,
        1.0 / 1500.0 - 0.1 * 0.1 / 2000.0,
        -0.2 / 1500.0 - 0.3 * 0.1 / 2000.0,
    );
    let det = s11 * s22 - s12 * s12;
    let (q11, q12) = (s22 / det, -s12 / det);
    let (r, _, _) = recover_in_solver_axes(&sec, &mat, &area_obj(), |x, _| [1e-3 * x, 0.0, 0.0]);
    close("F11", r.f11, q11 * 1e-3, 1e-10);
    close("F22", r.f22, q12 * 1e-3, 1e-10);
    close("S33", r.s33, 0.3 * q11 * 1e-3 + 0.1 * q12 * 1e-3, 1e-10);
    // At 90 degrees the material axes swap: the element strain e11 is the material strain
    // e'22, so s'11 = Q12 e, s'22 = Q22 e with Q22 = S'11 / det, and s33 = u13 s'11 + u23 s'22.
    sec.mat_angle_deg = 90.0;
    let q22 = s11 / det;
    let (r90, _, _) = recover_in_solver_axes(&sec, &mat, &area_obj(), |x, _| [1e-3 * x, 0.0, 0.0]);
    close("F11 at 90", r90.f11, q22 * 1e-3, 1e-10);
    close("S33 at 90", r90.s33, 0.3 * q12 * 1e-3 + 0.1 * q22 * 1e-3, 1e-10);
}

// ---------------------------------------------------------------- step 2d (D31) ----------

/// Nodal forces `K u` (24) of the element for the displacement vector `u`.
fn element_forces(sec: &AreaSection, mat: &Material, u: &[f64]) -> Vec<f64> {
    let (_, _, xy) = area_local_system(&QUAD, 0.0);
    let (k, _) = compute_area_local_stiffness(&area_obj(), sec, mat, &xy);
    (0..24)
        .map(|r| (0..24).map(|c| k[r * 24 + c] * u[c]).sum())
        .collect()
}

/// D31. The plane element has stiffness for the three translations only (Analysis Reference
/// p. 217), so a rigid rotation of the element about its normal is a motion without strain
/// energy: with u = -w y, v = w x at the corners and the rotation DOF R3 = 0 (the joint's
/// rotation about local 3 means nothing to a plane element), the nodal forces vanish, whatever
/// the section's `drilling_dof` flag says (native files store false for plane sections, the S2K
/// reader and `AreaSection::default()` give true). A shell section with the drilling term, in
/// contrast, is rigid when R3 = w as well (so R3 = 0 is no rigid motion for it).
#[test]
fn plane_section_has_no_drilling_penalty_and_shell_keeps_it() {
    let mat = isotropic(1000.0, 0.3);
    let w = 1e-3;
    let mut rigid = vec![0.0; 24];
    for (i, c) in QUAD.iter().enumerate() {
        rigid[6 * i] = -w * c[1];
        rigid[6 * i + 1] = w * c[0];
    }
    // scale of the in-plane forces: E t w L ~ 1000 * 1e-3 * 2 = 2
    for plane in [PlaneType::PlaneStress, PlaneType::PlaneStrain] {
        for drilling in [true, false] {
            let mut sec = section(AreaPropType::Plane, plane, 1.0);
            sec.drilling_dof = drilling;
            let f = element_forces(&sec, &mat, &rigid);
            let worst = f.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
            assert!(
                worst <= 1e-10,
                "{plane:?} drilling_dof {drilling}: rigid rotation gives force {worst:e}"
            );
        }
    }
    // Shell with the drilling term: R3 = w makes the same motion rigid ...
    let shell = section(AreaPropType::Shell, PlaneType::PlaneStress, 1.0);
    let mut rigid_shell = rigid.clone();
    for i in 0..4 {
        rigid_shell[6 * i + 5] = w;
    }
    let f = element_forces(&shell, &mat, &rigid_shell);
    assert!(f.iter().fold(0.0_f64, |m, v| m.max(v.abs())) <= 1e-9);
    // ... and R3 = 0 is not: the drilling penalty 1e-3 G t resists the relative rotation.
    let f = element_forces(&shell, &mat, &rigid);
    assert!(f.iter().fold(0.0_f64, |m, v| m.max(v.abs())) > 1e-5);
}

// ---------------------------------------------------- refusals through the analysis ------

/// The analysis refuses an orthotropic material whose constants are not valid (a negative
/// modulus here) before any element is formed, naming the area and the material; the
/// isotropic material of the same model is analysed.
#[test]
fn analysis_refuses_an_orthotropic_material_with_a_negative_modulus() {
    let mut mat = ortho_2_015();
    mat.a = 1e-5;
    mat.a_ortho = Some([1e-5; 3]);
    let mut sound = fixed_plate(mat.clone(), (1, 40.0), true);
    analyze_model(&mut sound).unwrap();

    mat.e_ortho = Some([30000.0, -15000.0, 1.0]);
    let mut m = fixed_plate(mat, (1, 40.0), true);
    let error = analyze_model(&mut m).unwrap_err().to_string();
    assert!(error.contains("area 'A'"), "{error}");
    assert!(error.contains("orthotropic constants are not valid"), "{error}");
    assert!(m.case_results.is_empty(), "a refused analysis leaves no results");
}

/// An orthotropic material with different in-plane expansion coefficients is refused for a
/// temperature load on an area (only a1 = a2 is implemented), naming the area and both
/// coefficients; without the temperature load the same material is analysed.
#[test]
fn analysis_refuses_a_temperature_load_on_an_area_with_unequal_expansion_coefficients() {
    let mut mat = ortho_2_015();
    mat.a = 1e-5;
    mat.a_ortho = Some([1e-5, 2e-5, 1e-5]);
    let mut m = fixed_plate(mat.clone(), (1, 40.0), true);
    let error = analyze_model(&mut m).unwrap_err().to_string();
    assert!(error.contains("area 'A'"), "{error}");
    assert!(error.contains("only a1 = a2 is implemented"), "{error}");

    let mut unloaded = fixed_plate(mat, (1, 40.0), true);
    unloaded.areas.get_mut("A").unwrap().loads_temp.clear();
    analyze_model(&mut unloaded).unwrap();
}

/// An automatically meshed plane-strain area reports the mean S33 of its elements. The loads
/// of `plane_strain_uniaxial_tension_gives_s33_equal_nu_s11` on a 2 x 2 mesh of the same
/// square: the two corner forces do not load the mid-side joint, so the stress is not
/// uniform, but its mean over the body is fixed by the boundary forces alone (mean stress
/// theorem: V <s_ij> = sum of f_i x_j; the finite elements satisfy it because a linear
/// displacement field is in their space, and the centre value of a rectangular bilinear
/// element is its mean). So <s11> = 1, <s22> = 0 and <s33> = nu (<s11> + <s22>) = 0.3.
#[test]
fn an_auto_meshed_plane_strain_area_keeps_s33() {
    let mut m = unit_square_model(PlaneType::PlaneStrain);
    for (j, p) in m.points.iter_mut() {
        p.restraint = [false, false, true, true, true, true];
        if j == "1" {
            p.restraint = [true; 6];
        }
        if j == "4" {
            p.restraint = [true, false, true, true, true, true];
        }
    }
    point_force(&mut m, "2", [0.5, 0.0, 0.0]);
    point_force(&mut m, "3", [0.5, 0.0, 0.0]);
    m.areas.get_mut("A").unwrap().auto_mesh = (2, 2);
    analyze_model(&mut m).unwrap();
    let a = &m.case_results["P"].area_results["A"];
    close("F11", a.f11, 1.0, 1e-9);
    close("S33", a.s33, 0.3, 1e-9);
}
