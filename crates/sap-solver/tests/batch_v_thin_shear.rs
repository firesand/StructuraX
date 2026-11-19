//! Batch V, D105: the transverse shear V13/V23 of a thin shell or thin plate.
//!
//! The element stiffness multiplies the transverse shear law by `thin_boost =
//! clamp(1e4 (tb/h)^2, 1, 1e6)` (h = diagonal of the element); the recovery used to report
//! the unboosted `ds * gamma`, which is `Q / boost`. Every expected value below is statics of a
//! strip written out in the test (kip, in), not read from the element code.
use sap_model::*;
use sap_solver::analyze_model;
use sap_units::Units;

fn strip(shell_type: ShellType, tb: f64, n: usize) -> SapModel {
    let mut m = SapModel::new(Units::KipInF);
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.e = 3600.0;
    mat.u = 0.0;
    mat.g = 1800.0;
    mat.unit_weight = 0.0;
    mat.unit_mass = 0.0;
    m.materials.insert("MAT".into(), mat);
    let mut sec = AreaSection::default();
    sec.name = "SEC".into();
    sec.material = "MAT".into();
    sec.prop_type = AreaPropType::Shell;
    sec.shell_type = shell_type;
    sec.thickness_membrane = tb;
    sec.thickness_bending = tb;
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
    for i in 0..=n {
        for (k, y) in [0.0, 24.0].into_iter().enumerate() {
            m.add_point(format!("{i}_{k}"), 24.0 * i as f64, y, 0.0);
        }
    }
    for i in 0..n {
        m.add_area(
            format!("A{i}"),
            vec![
                format!("{i}_0"),
                format!("{}_0", i + 1),
                format!("{}_1", i + 1),
                format!("{i}_1"),
            ],
            "SEC",
        );
    }
    m
}

fn cantilever(shell_type: ShellType, tb: f64) -> SapModel {
    let mut m = strip(shell_type, tb, 5);
    for k in 0..2 {
        m.points.get_mut(&format!("0_{k}")).unwrap().restraint = [true; 6];
        m.points
            .get_mut(&format!("5_{k}"))
            .unwrap()
            .loads_force
            .push(PointLoadForce {
                load_pat: "P".into(),
                csys: "GLOBAL".into(),
                force: [0.0, 0.0, -5.0, 0.0, 0.0, 0.0],
            });
    }
    m
}

fn simply_supported(shell_type: ShellType, tb: f64) -> SapModel {
    let mut m = strip(shell_type, tb, 10);
    for p in m.points.values_mut() {
        // in-plane DOFs and the drilling rotation held: they carry no plate action
        p.restraint = [true, true, false, false, false, true];
    }
    for k in 0..2 {
        for end in ["0", "10"] {
            m.points.get_mut(&format!("{end}_{k}")).unwrap().restraint =
                [true, true, true, false, false, true];
        }
    }
    for i in 0..10 {
        m.areas
            .get_mut(&format!("A{i}"))
            .unwrap()
            .loads_uniform
            .push(AreaLoadUniform {
                load_pat: "P".into(),
                csys: "GLOBAL".into(),
                dir: 6,
                val: -0.01,
            });
    }
    m
}

fn centroid_x(i: usize) -> f64 {
    24.0 * i as f64 + 12.0
}

/// Cantilever strip, b = 24 in, L = 120 in, tip load P = 10 kip. Statics: |V13| = P/b =
/// 0.416667 kip/in and |M11| = P (L - xc)/b = 45, 35, 25, 15, 5 kip-in/in at xc = 12..108.
/// Boost = 1e4 (6/33.941)^2 = 312.5; the unboosted recovery gave 0.0013333.
fn check_cantilever(shell_type: ShellType, tol_v: f64) {
    let mut m = cantilever(shell_type, 6.0);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["P"];
    for i in 0..5 {
        let a = &r.area_results[&format!("A{i}")];
        let want_m = 10.0 * (120.0 - centroid_x(i)) / 24.0;
        assert!(
            (a.m11.abs() - want_m).abs() <= 1e-3 * want_m,
            "{shell_type:?} A{i} M11 {} want {want_m}",
            a.m11
        );
        assert!(
            (a.v13.abs() - 10.0 / 24.0).abs() <= tol_v * (10.0 / 24.0),
            "{shell_type:?} A{i} V13 {} want {}",
            a.v13,
            10.0 / 24.0
        );
        assert!(a.v23.abs() <= 1e-6, "V23 {}", a.v23);
    }
}

#[test]
fn thin_shell_cantilever_strip_reports_the_statical_shear() {
    check_cantilever(ShellType::ShellThin, 1e-4);
}

#[test]
fn thin_plate_cantilever_strip_reports_the_statical_shear() {
    check_cantilever(ShellType::PlateThin, 1e-4);
}

/// Simply supported strip, L = 240 in, q = 0.01 ksi on b = 24 in. Statics
/// V13 per unit width = q (L/2 - x), at xc = 12, 36, ... : 1.08, 0.84, 0.60, 0.36, 0.12
/// then the mirror. Tolerance 1 %.
#[test]
fn thin_shell_simply_supported_strip_reports_the_varying_shear() {
    for shell_type in [ShellType::ShellThin, ShellType::PlateThin] {
        let mut m = simply_supported(shell_type, 6.0);
        analyze_model(&mut m).unwrap();
        let r = &m.case_results["P"];
        for i in 0..10 {
            let a = &r.area_results[&format!("A{i}")];
            let want = (0.01 * (120.0 - centroid_x(i))).abs();
            assert!(
                (a.v13.abs() - want).abs() <= 0.01 * want,
                "{shell_type:?} A{i} V13 {} want {want}",
                a.v13
            );
        }
    }
}

/// Negative: a thick section has boost 1 and its shear law is the plain one. The thick
/// cantilever strip carries the same statical shear P/b (2 % allows for the shear-flexible
/// element); nothing is multiplied.
#[test]
fn thick_strip_shear_is_unchanged_by_the_boost() {
    for shell_type in [ShellType::ShellThick, ShellType::PlateThick] {
        check_cantilever_thick(shell_type);
    }
}

fn check_cantilever_thick(shell_type: ShellType) {
    let mut m = cantilever(shell_type, 6.0);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["P"];
    for i in 0..5 {
        let a = &r.area_results[&format!("A{i}")];
        assert!(
            (a.v13.abs() - 10.0 / 24.0).abs() <= 0.02 * (10.0 / 24.0),
            "{shell_type:?} A{i} V13 {}",
            a.v13
        );
    }
}

/// Negative: a Plane (plane stress) section has no plate bending, so its recovered transverse
/// shear keeps the plain law even though its (unused) shell type is the default ShellThin.
/// u3 = 1e-3 x on the 2.2 x 1.5 quadrilateral gives gamma13 = 1e-3; for E = 1000, nu = 0.3,
/// t = 1: G = 384.615, V13 = G (5/6) t gamma13 = 0.3205128 (a boost of 1e4 (1/2.663)^2 = 1410
/// would give 452).
#[test]
fn a_plane_section_keeps_the_plain_shear_law() {
    use sap_solver::area::{area_local_system, recover_area_result};
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.e = 1000.0;
    mat.u = 0.3;
    mat.g = 1000.0 / 2.6;
    let mut sec = AreaSection::default();
    sec.name = "SEC".into();
    sec.material = "MAT".into();
    sec.prop_type = AreaPropType::Plane;
    sec.plane_type = PlaneType::PlaneStress;
    sec.shell_type = ShellType::ShellThin;
    sec.thickness_membrane = 1.0;
    sec.thickness_bending = 1.0;
    let quad = [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [2.2, 1.5, 0.0], [-0.1, 1.2, 0.0]];
    let (_, _, xy) = area_local_system(&quad, 0.0);
    let mut u = vec![0.0; 24];
    for (i, c) in quad.iter().enumerate() {
        u[6 * i + 2] = 1e-3 * c[0];
    }
    let area = AreaObj::new("A", vec!["1".into(), "2".into(), "3".into(), "4".into()], "SEC");
    let r = recover_area_result(&area, &sec, &mat, &xy, &u, Vec::new());
    let want = 1000.0 / 2.6 * (5.0 / 6.0) * 1e-3;
    assert!((r.v13 - want).abs() <= 1e-9 * want, "V13 {} want {want}", r.v13);
}
