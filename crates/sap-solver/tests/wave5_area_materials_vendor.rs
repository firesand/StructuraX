//! Wave 5, package P2: opt-in vendor-corpus checks of the area constitutive laws.
//! They skip silently when the corpus is absent.
#![cfg(feature = "vendor-fixtures")]

use std::path::{Path, PathBuf};

use sap_format::load_sdb_file;
use sap_model::SapModel;
use sap_solver::analyze_model;

fn load(file: &str) -> Option<SapModel> {
    let dir = std::env::var_os("STRUCTURAX_VENDOR_FIXTURES")
        .map(PathBuf::from)
        .unwrap_or_else(|| Path::new(env!("CARGO_MANIFEST_DIR")).join("../../extracted/files"));
    let path = dir.join(file);
    if !path.exists() {
        return None;
    }
    Some(load_sdb_file(&path).expect(file))
}

/// R4, measured and not asserted. Example 3-004, thick cylinder a = 3, b = 9, p = 1 ksi,
/// E = 1000 (10 degree wedge), plane strain. Lame (plane strain):
/// ur(a) = (1 + nu) a^2 p / (E (b^2 - a^2)) ((1 - 2 nu) a + b^2 / a) = 0.004583 (nu 0.3),
/// 0.005040 (nu 0.49); sz = nu (sr + sth) = 0.0750 / 0.1225 at the inner face. The
/// published SAP2000 values are 0.004539, 0.005036, 0.005040 (three files). The section's
/// incompatible-modes flag is switched off in the test: the element has none. The files load
/// the inner face as an edge pressure, which the solver refuses (only top and bottom pressure
/// is implemented); the test replaces it by the equivalent nodal forces p t L / 2 on the two
/// joints of each inner chord, normal to the chord and pointing into the material.
#[test]
fn example_3_004_plane_strain_cylinder_measured() {
    for file in [
        "example_3_004a_strain.sdb",
        "example_3_004e_strain.sdb",
        "example_3_004f_strain.sdb",
    ] {
        let Some(mut m) = load(file) else { return };
        let mut pattern = None;
        let mut pressure = 1.0;
        for a in m.areas.values_mut() {
            for l in a.loads_pressure.drain(..) {
                pattern = Some(l.load_pat.clone());
                pressure = l.val.abs();
            }
        }
        let Some(pattern) = pattern else {
            println!("R4 {file}: no pressure load found");
            continue;
        };
        for s in m.area_sections.values_mut() {
            s.incompatible_modes = false;
        }
        let mut forces: Vec<(String, [f64; 2])> = Vec::new();
        for a in m.areas.values() {
            let t = m.area_sections[&a.section].thickness_membrane;
            let inner: Vec<&String> = a
                .points
                .iter()
                .filter(|p| {
                    let q = &m.points[*p];
                    ((q.x * q.x + q.y * q.y).sqrt() - 3.0).abs() < 2e-3
                })
                .collect();
            if inner.len() == 2 {
                let (p, q) = (&m.points[inner[0]], &m.points[inner[1]]);
                let len = ((p.x - q.x).powi(2) + (p.y - q.y).powi(2)).sqrt();
                let (mx, my) = (0.5 * (p.x + q.x), 0.5 * (p.y + q.y));
                let mn = (mx * mx + my * my).sqrt();
                let f = pressure * t * len / 2.0;
                for j in inner {
                    forces.push((j.clone(), [f * mx / mn, f * my / mn]));
                }
            }
        }
        for (j, f) in forces {
            m.points
                .get_mut(&j)
                .unwrap()
                .loads_force
                .push(sap_model::PointLoadForce {
                    load_pat: pattern.clone(),
                    csys: "GLOBAL".into(),
                    force: [f[0], f[1], 0.0, 0.0, 0.0, 0.0],
                });
        }
        if let Err(e) = analyze_model(&mut m) {
            println!("R4 {file}: refused: {e}");
            continue;
        }
        let res = &m.case_results[&pattern];
        let mut ur_inner = f64::NAN;
        for (name, p) in &m.points {
            let r = (p.x * p.x + p.y * p.y).sqrt();
            if (r - 3.0).abs() < 2e-3 {
                let d = res.joint_displacements[name];
                ur_inner = (d[0] * p.x + d[1] * p.y) / r;
            }
        }
        let nus: Vec<f64> = m.materials.values().map(|x| x.u).collect();
        let (an, s33) = res
            .area_results
            .iter()
            .next()
            .map_or(("-", f64::NAN), |(n, a)| (n.as_str(), a.s33));
        println!(
            "R4 {file} (material nu {nus:?}): ur(a) = {ur_inner:.6}, S33 of area {an} = {s33:.4}"
        );
    }
}

/// Prints, for each Example 2-015 file, the material and modifiers of its area section, the
/// centre deflection and the centre-of-element moments next to joint A (on the +X axis, r = 48)
/// and joint B (on the +Y axis). Measured and not asserted (joint values need G01).
/// Published (thin, 8x16): Uz -1.077 (A and B); M11 / M22 at A -81.54 / -16.20 (A), -81.54 /
/// -16.19 (B); at B -15.94 / -39.70 (both models).
#[test]
fn example_2_015_models_side_by_side_measured() {
    for file in [
        "example_2_015a_thin.sdb",
        "example_2_015b_thin.sdb",
        "example_2_015a_thick.sdb",
        "example_2_015b_thick.sdb",
        "example_2_015c_thick.sdb",
        "example_2_015d_thick.sdb",
    ] {
        let Some(mut m) = load(file) else { return };
        let sec = m.area_sections.values().next().unwrap().clone();
        let mat = m.materials[&sec.material].clone();
        println!(
            "2-015 {file}: section {} type {:?}/{:?} tm {} tb {} angle {} modifiers {:?}",
            sec.name, sec.prop_type, sec.shell_type, sec.thickness_membrane, sec.thickness_bending,
            sec.mat_angle_deg, sec.modifiers
        );
        println!(
            "      material {}: e {} e_ortho {:?} u {} u_ortho {:?} g {} g_ortho {:?} a_ortho {:?}",
            mat.name, mat.e, mat.e_ortho, mat.u, mat.u_ortho, mat.g, mat.g_ortho, mat.a_ortho
        );
        for (n, sc) in &m.area_sections {
            let used = m.areas.values().filter(|a| &a.section == n).count();
            println!("      [all sections] {n}: material {} used by {used} areas, modifiers {:?}", sc.material, sc.modifiers);
        }
        for (n, mt) in &m.materials {
            println!("      [all materials] {n}: type {:?} e {} u {} g {} e_ortho {:?}", mt.mat_type, mt.e, mt.u, mt.g, mt.e_ortho);
        }
        let area_mods: std::collections::BTreeSet<String> =
            m.areas.values().map(|a| format!("{:?}", a.modifiers)).collect();
        println!("      area-object modifiers: {area_mods:?}");
        if let Err(e) = analyze_model(&mut m) {
            println!("      refused: {e}");
            continue;
        }
        let res = m.case_results.values().next().unwrap();
        let uz = res.joint_displacements.values().map(|d| d[2]).fold(f64::MAX, f64::min);
        println!("      centre Uz {uz:.4}");
        for (label, on_x) in [("A (+X axis)", true), ("B (+Y axis)", false)] {
            let mut n = 0.0;
            let (mut m11, mut m22) = (0.0, 0.0);
            for (name, a) in &m.areas {
                let touches = a.points.iter().any(|p| {
                    let q = &m.points[p];
                    if on_x {
                        (q.x - 48.0).abs() < 1e-6 && q.y.abs() < 1e-6
                    } else {
                        (q.y - 48.0).abs() < 1e-6 && q.x.abs() < 1e-6
                    }
                });
                if touches {
                    let r = &res.area_results[name];
                    m11 += r.m11;
                    m22 += r.m22;
                    n += 1.0;
                }
            }
            println!(
                "      joint {label}: {n} elements, mean element-centre M11 {:.3} M22 {:.3}",
                m11 / n,
                m22 / n
            );
        }
    }
}

/// Centre deflection (most negative Uz) and the mean element-centre moments at the elements
/// touching joint A (on +X) and joint B (on +Y) of a solved 2-015 model.
fn centre_uz_and_moments(file: &str) -> Option<(f64, [f64; 4])> {
    let mut m = load(file)?;
    analyze_model(&mut m).expect(file);
    let res = m.case_results.values().next().unwrap();
    let uz = res.joint_displacements.values().map(|d| d[2]).fold(f64::MAX, f64::min);
    let mut out = [0.0; 4];
    for (k, on_x) in [(0, true), (2, false)] {
        let (mut n, mut m11, mut m22) = (0.0, 0.0, 0.0);
        for (name, a) in &m.areas {
            let touches = a.points.iter().any(|p| {
                let q = &m.points[p];
                if on_x {
                    (q.x - 48.0).abs() < 1e-6 && q.y.abs() < 1e-6
                } else {
                    (q.y - 48.0).abs() < 1e-6 && q.x.abs() < 1e-6
                }
            });
            if touches {
                n += 1.0;
                m11 += res.area_results[name].m11;
                m22 += res.area_results[name].m22;
            }
        }
        out[k] = m11 / n;
        out[k + 1] = m22 / n;
    }
    Some((uz, out))
}

/// R7 / R8. Example 2-015 uses the orthotropic constants of model A (E1 30000, E2 15000,
/// nu12 0.2, G12 6600). Centre deflection: published SAP2000 -1.077 (thin, 8x16), -1.074 (thick,
/// 8x16), -1.070 (thick, 16x32), each to one percent (the tolerance of the existing 2-015b test
/// on the same mesh); closed form (Ugural 1981 as in the document)
/// w0 = q a^4 / (8 (3 Dx + 2 H + 3 Dy)) = 1.069195 in, with Dx = Q11 t^3/12 = 21739.13,
/// Dy = 10869.57, H = Q12 t^3/12 + 2 G t^3/12 = 13147.83 (q = 0.2, a = 48): to 1.5 percent for
/// the 8x16 meshes. Model B (isotropic with the modifiers stored in the file) has, by algebra,
/// the stiffness of model A except the twisting term (m12 = 0.5645 gives 4400.7 against
/// 6600 t^3/12 = 4400.0, 1.6e-4), so A and B agree to 1e-3 in the centre deflection and in the
/// element-centre moments next to joints A and B.
#[test]
fn example_2_015_orthotropic_model_matches_published_deflection_and_model_b() {
    for (a, b, published) in [
        ("example_2_015a_thin.sdb", "example_2_015b_thin.sdb", -1.077),
        ("example_2_015a_thick.sdb", "example_2_015b_thick.sdb", -1.074),
        ("example_2_015c_thick.sdb", "example_2_015d_thick.sdb", -1.070),
    ] {
        let (Some((ua, ma)), Some((ub, mb))) =
            (centre_uz_and_moments(a), centre_uz_and_moments(b))
        else {
            return;
        };
        assert!((ua - published).abs() <= 0.01 * published.abs(), "{a}: Uz {ua} vs {published}");
        if !a.contains("2_015c") {
            assert!((ua - (-1.069_195)).abs() <= 0.015 * 1.069_195, "{a}: Uz {ua} vs closed form");
        }
        assert!((ua - ub).abs() <= 1e-3 * ua.abs(), "{a} vs {b}: Uz {ua} {ub}");
        for k in 0..4 {
            assert!(
                (ma[k] - mb[k]).abs() <= 1e-3 * ma[k].abs(),
                "{a} vs {b}: moment {k}: {} {}",
                ma[k],
                mb[k]
            );
        }
    }
}

/// Average of displacement component `dof` over the two tip joints 7 and 14 in `case`
/// (absolute values when `abs`).
fn tip(m: &SapModel, case: &str, dof: usize, abs: bool) -> f64 {
    let r = &m.case_results[case];
    let v = |j: &str| {
        let d = r.joint_displacements[j][dof];
        if abs {
            d.abs()
        } else {
            d
        }
    };
    0.5 * (v("7") + v("14"))
}

/// R9. Example 3-002 without incompatible modes (the "comp" files, section flags as stored:
/// drilling off), plane-stress straight cantilever, 6 x 1 meshes (a rectangles, b trapezoids,
/// c parallelograms) and 240 x 4 (f). Published SAP2000 values (the vendor's plain isoparametric
/// element): load 1 tip Ux 3.000e-5 in, which is also the exact axial extension
/// F L / (E A) = 1 * 6 / (1e7 * 0.02) = 3e-5; load 2 tip Uz 0.0101, 0.0030, 0.0038 and (f)
/// 0.1068; load 3 average |Ux| 0.840e-4, 0.206e-4, 0.282e-4 and (f) 8.958e-4. Each is asserted
/// within half a unit of the last printed digit: 5e-5 for the Uz values (four decimals, 0.0101 etc.),
/// 5e-8 for the |Ux| values (0.840e-4, 0.206e-4, 0.282e-4 and 8.958e-4 all end at 1e-7), and 1e-9
/// relative for the axial one. R9b: setting
/// `drilling_dof = true` on the section changes nothing once the plane element has no drilling
/// term (before step 2d the penalty stiffened bending).
#[test]
fn example_3_002_comp_plane_beam_matches_published_values() {
    let published: [(&str, f64, f64); 4] = [
        ("example_3_002a_comp.sdb", 0.0101, 0.840e-4),
        ("example_3_002b_comp.sdb", 0.0030, 0.206e-4),
        ("example_3_002c_comp.sdb", 0.0038, 0.282e-4),
        ("example_3_002f_comp.sdb", 0.1068, 8.958e-4),
    ];
    for (file, uz, ux3) in published {
        let Some(mut m) = load(file) else { return };
        for s in m.area_sections.values_mut() {
            s.incompatible_modes = false;
        }
        let mut with_drilling = m.clone();
        analyze_model(&mut m).expect(file);
        let cases: Vec<String> = m.case_results.keys().cloned().collect();
        let (c1, c2, c3) = (&cases[0], &cases[1], &cases[2]);
        let u1 = tip(&m, c1, 0, false);
        let u2 = tip(&m, c2, 2, false);
        let u3 = tip(&m, c3, 0, true);
        println!(
            "R9 {file}: Ux1 {u1:.4e} (3.000e-5), Uz2 {u2:.5} ({uz}), |Ux|3 {u3:.4e} ({ux3:.3e})"
        );
        assert!((u1 - 3.0e-5).abs() <= 1e-9 * 3.0e-5, "{file}: load 1 {u1:e}");
        assert!((u2 - uz).abs() <= 5e-5, "{file}: load 2 {u2} vs {uz}");
        assert!((u3 - ux3).abs() <= 5e-8, "{file}: load 3 {u3:e} vs {ux3:e}");
        // R9b
        for s in with_drilling.area_sections.values_mut() {
            s.drilling_dof = true;
        }
        analyze_model(&mut with_drilling).expect(file);
        println!(
            "R9b {file}: with drilling_dof = true: Uz2 {:.5}, |Ux|3 {:.4e}",
            tip(&with_drilling, c2, 2, false),
            tip(&with_drilling, c3, 0, true)
        );
        for (c, dof, abs) in [(c1, 0, false), (c2, 2, false), (c3, 0, true)] {
            assert_eq!(
                tip(&with_drilling, c, dof, abs),
                tip(&m, c, dof, abs),
                "{file}: drilling_dof must not matter for a plane section"
            );
        }
    }
}

/// Scan of the whole corpus, measured and not asserted: which models carry an area section that
/// the package changes (an orthotropic material on an area section, a nonzero material angle, a
/// plane-strain section, a plane or axisymmetric section with the drilling flag on) and whether
/// the orthotropic constants pass `validate_area_material`.
#[test]
fn corpus_scan_of_area_sections_touched_by_the_package() {
    let dir = std::env::var_os("STRUCTURAX_VENDOR_FIXTURES")
        .map(PathBuf::from)
        .unwrap_or_else(|| Path::new(env!("CARGO_MANIFEST_DIR")).join("../../extracted/files"));
    let Ok(entries) = std::fs::read_dir(&dir) else { return };
    let mut files: Vec<PathBuf> = entries
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x == "sdb"))
        .collect();
    files.sort();
    let (mut n_files, mut n_hits) = (0, 0);
    for path in files {
        let Ok(m) = load_sdb_file(&path) else { continue };
        n_files += 1;
        let name = path.file_name().unwrap().to_string_lossy().to_string();
        for (sn, sec) in &m.area_sections {
            let used = m.areas.values().filter(|a| &a.section == sn).count();
            if used == 0 {
                continue;
            }
            let mat = &m.materials[&sec.material];
            let ortho = sap_solver::area::area_material_is_orthotropic(mat);
            let plane = sec.prop_type != sap_model::AreaPropType::Shell;
            let strain = sec.plane_type == sap_model::PlaneType::PlaneStrain
                && sec.prop_type == sap_model::AreaPropType::Plane;
            // unequal membrane or bending modifiers: the recovered resultants now use the
            // stiffness' coupling term sqrt(m_i m_j) (section or any area of the section)
            let unequal = |f: &[f64; 10]| f[0] != f[1] || f[3] != f[4];
            let mods_differ = unequal(&sec.modifiers)
                || m.areas.values().any(|a| &a.section == sn && unequal(&a.modifiers));
            if ortho
                || sec.mat_angle_deg != 0.0
                || strain
                || (plane && sec.drilling_dof)
                || mods_differ
            {
                n_hits += 1;
                println!(
                    "SCAN {name}: section {sn} ({used} areas) {:?}/{:?} material {} ortho {ortho} (valid: {:?}) angle {} drilling {} modifiers {:?}",
                    sec.prop_type,
                    sec.plane_type,
                    mat.name,
                    sap_solver::area::validate_area_material(sn, mat).is_ok(),
                    sec.mat_angle_deg,
                    sec.drilling_dof,
                    sec.modifiers
                );
            }
        }
    }
    println!("SCAN {n_files} files read, {n_hits} sections listed");
}
