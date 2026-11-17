//! Wave 5 P4 step 4b (D06): measurement of the residual in the periods of Examples 6-006 and
//! 6-007 against the published 0.438, 0.135 and 0.074 s. Opt-in (`STRUCTURAX_VENDOR_FIXTURES`)
//! and `#[ignore]`d: it prints, it asserts nothing about the residual. Run with
//! `-- --ignored --nocapture`.
#![cfg(feature = "vendor-fixtures")]

use std::path::PathBuf;

use sap_model::{LoadCaseType, ModalSubType, SapModel};
use sap_solver::analyze_model;

const PUBLISHED: [f64; 3] = [0.438, 0.135, 0.074];

fn load(name: &str) -> Option<SapModel> {
    let dir = PathBuf::from(std::env::var_os("STRUCTURAX_VENDOR_FIXTURES")?);
    let path = dir.join(name);
    path.is_file().then(|| {
        let mut m = sap_format::load_sdb_file(&path).expect(name);
        // The importer records the Ritz starting load of type 4 (all link element nonlinear
        // degrees of freedom) as unmodeled source data and the Modal case is refused (D36). The
        // probe measures what the other starting vectors give, so it removes that entry
        // explicitly: the periods printed are those of the reduced Ritz subspace, not a result.
        let before = m.unmodeled_source.len();
        m.unmodeled_source
            .retain(|u| !u.description.contains("Ritz starting load of type 4"));
        println!("{name}: {} Ritz starting-load entries removed", before - m.unmodeled_source.len());
        m
    })
}

/// First periods of the (only) Modal case, with only the Modal cases run.
fn periods(m: &SapModel) -> Result<Vec<f64>, String> {
    let mut m = m.clone();
    for lc in m.load_cases.values_mut() {
        lc.run_flag = lc.case_type == LoadCaseType::Modal;
    }
    analyze_model(&mut m).map_err(|e| e.to_string())?;
    let modal = m
        .load_cases
        .values()
        .find(|c| c.case_type == LoadCaseType::Modal)
        .map(|c| c.name.clone())
        .ok_or("no modal case")?;
    let all: Vec<f64> = m.case_results[&modal].modes.iter().map(|x| x.period_sec).collect();
    println!("  ({} modes; periods {:?})", all.len(), all.iter().take(6).map(|t| (t * 1e4).round() / 1e4).collect::<Vec<_>>());
    Ok(all.into_iter().take(3).collect())
}

fn row(file: &str, label: &str, m: &SapModel) {
    match periods(m) {
        Ok(p) => {
            let inside = p.len() == 3
                && (0.4375..=0.4385).contains(&p[0])
                && (0.1345..=0.1355).contains(&p[1])
                && (0.0735..=0.0745).contains(&p[2]);
            let err: Vec<String> = p
                .iter()
                .zip(PUBLISHED)
                .map(|(t, e)| format!("{:+.2}%", 100.0 * (t - e) / e))
                .collect();
            println!(
                "{file} {label:<44} T = {:?}  err {}  inside printed rounding: {inside}",
                p.iter().map(|t| (t * 1e4).round() / 1e4).collect::<Vec<_>>(),
                err.join(" ")
            );
        }
        Err(e) => println!("{file} {label:<44} ERROR {}", &e[..e.len().min(120)]),
    }
}

fn dump(m: &SapModel) {
    println!("units {:?}  active dofs {:?}", m.present_units, m.active_dofs);
    for p in m.points.values() {
        println!(
            "joint {} ({:.4}, {:.4}, {:.4}) restr {:?} mass {:?} constraints {:?}",
            p.name, p.x, p.y, p.z, p.restraint.map(u8::from), p.mass, p.constraints
        );
    }
    for c in m.constraints.values() {
        println!("constraint {} {:?} axis {:?} dofs {:?}", c.name, c.constraint_type, c.axis, c.dofs);
    }
    for f in m.frames.values() {
        println!(
            "frame {} {}-{} sec {} off_i {:.4} off_j {:.4} rz {} releases {:?}/{:?} cardinal {} joff {:?}/{:?} mods {:?}",
            f.name, f.point_i, f.point_j, f.section, f.end_offset_i, f.end_offset_j,
            f.rigid_zone_factor, f.release_i.map(u8::from), f.release_j.map(u8::from),
            f.cardinal_point, f.joint_offset_i, f.joint_offset_j, f.modifiers
        );
    }
    for s in m.frame_sections.values() {
        println!(
            "section {} mat {} A {} I33 {} I22 {} J {} As2 {} As3 {}",
            s.name, s.material, s.area, s.i33, s.i22, s.torsional_j, s.as2, s.as3
        );
    }
    for mat in m.materials.values() {
        println!(
            "material {} E {} G {} nu {} unit mass {} unit weight {}",
            mat.name, mat.e, mat.g, mat.u, mat.unit_mass, mat.unit_weight
        );
    }
    for l in m.links.values() {
        let p = &m.link_props[&l.prop];
        println!(
            "link {} {}-{:?} prop {} type {:?} ke {:?} ce {:?} mass {} weight {} nonlinear {:?}",
            l.name, l.point_i, l.point_j, l.prop, p.link_type, p.ke, p.ce, p.mass, p.weight,
            p.dof_nonlinear.map(u8::from)
        );
    }
    println!("mass source {:?}", m.mass_source);
    for lc in m.load_cases.values().filter(|c| c.case_type == LoadCaseType::Modal) {
        println!(
            "modal case {} sub {:?} max {} min {} loads {:?} ritz {:?} shift {}",
            lc.name, lc.modal_sub_type, lc.max_modes, lc.min_modes, lc.loads, lc.ritz_controls,
            lc.shift_freq_hz
        );
    }
}

#[test]
#[ignore]
fn d06_print_the_imported_model() {
    let Some(m) = load("example_6_006.sdb") else { return };
    dump(&m);
}

#[test]
#[ignore]
fn d06_variants() {
    for file in ["example_6_006.sdb", "example_6_007.sdb"] {
        let Some(base) = load(file) else { return };
        row(file, "V0 as imported", &base);

        let mut m = base.clone();
        for lc in m.load_cases.values_mut().filter(|c| c.case_type == LoadCaseType::Modal) {
            lc.modal_sub_type = ModalSubType::Eigen;
            lc.loads.clear();
            lc.ritz_controls.clear();
            lc.max_modes = 12;
            lc.min_modes = 1;
        }
        row(file, "V1 Eigen (12 modes) instead of Ritz", &m);

        let mut m = base.clone();
        for p in m.link_props.values_mut() {
            p.mass = 0.0;
        }
        row(file, "V2 link mass omitted", &m);
        for lc in m.load_cases.values_mut().filter(|c| c.case_type == LoadCaseType::Modal) {
            lc.modal_sub_type = ModalSubType::Eigen;
            lc.loads.clear();
            lc.ritz_controls.clear();
            lc.max_modes = 12;
        }
        row(file, "V2b link mass omitted, Eigen", &m);

        let mut m = base.clone();
        for mat in m.materials.values_mut() {
            mat.unit_mass = 0.0;
            mat.unit_weight = 0.0;
        }
        row(file, "V3 frame self mass zero", &m);

        let mut m = base.clone();
        for f in m.frames.values_mut() {
            f.rigid_zone_factor = 0.0;
        }
        row(file, "V4 rz = 0", &m);

        let mut m = base.clone();
        for f in m.frames.values_mut() {
            f.rigid_zone_factor = 1.0;
        }
        row(file, "V5 rz = 1", &m);

        let mut m = base.clone();
        for s in m.frame_sections.values_mut() {
            s.as2 = 0.0;
            s.as3 = 0.0;
        }
        row(file, "V6 shear deformation off", &m);

        // V7 is measured on a patched solver (not in this patch): see the report.

        // V10: the starting vectors the problem statement lists besides Ux acceleration, "all
        // link element nonlinear degrees of freedom" (native: a second load assignment of the
        // Modal case, type 4, index -1, which the importer drops). Emulated by one load pattern
        // per damper: equal and opposite unit forces on its end joints along its axis (the
        // static load vector of a unit deformation of the link's axial degree of freedom).
        let mut m = base.clone();
        let links: Vec<_> = m.links.values().cloned().collect();
        for (n, link) in links.iter().enumerate() {
            let (pi, pj) = (&m.points[&link.point_i], &m.points[link.point_j.as_ref().unwrap()]);
            let d = [pj.x - pi.x, pj.y - pi.y, pj.z - pi.z];
            let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
            let u = [d[0] / len, d[1] / len, d[2] / len];
            let pat = format!("LINKDOF{n}");
            m.load_patterns.insert(
                pat.clone(),
                sap_model::LoadPattern {
                    name: pat.clone(),
                    pat_type: sap_model::LoadPatternType::Other,
                    self_wt_multiplier: 0.0,
                },
            );
            for (point, sign) in [(link.point_i.clone(), -1.0), (link.point_j.clone().unwrap(), 1.0)] {
                m.points[&point].loads_force.push(sap_model::PointLoadForce {
                    load_pat: pat.clone(),
                    csys: "GLOBAL".into(),
                    force: [sign * u[0], sign * u[1], sign * u[2], 0.0, 0.0, 0.0],
                });
            }
            for lc in m.load_cases.values_mut().filter(|c| c.case_type == LoadCaseType::Modal) {
                lc.loads.push(sap_model::LoadCaseAssignment { load_name: pat.clone(), scale_factor: 1.0 });
            }
        }
        row(file, "V10 + link-DOF starting vectors (as patterns)", &m);
        for lc in m.load_cases.values_mut().filter(|c| c.case_type == LoadCaseType::Modal) {
            lc.max_modes = 10;
        }
        row(file, "V10b same, 10 modes", &m);

        let mut m = base.clone();
        m.constraints.clear();
        for p in m.points.values_mut() {
            p.constraints.clear();
        }
        let horizontal: Vec<String> = m
            .frames
            .values()
            .filter(|f| (m.points[&f.point_i].z - m.points[&f.point_j].z).abs() < 1e-9)
            .map(|f| f.name.clone())
            .collect();
        for name in horizontal {
            m.frames[&name].modifiers[0] = 1.0e6;
        }
        row(file, "V8 diaphragms -> axial modifier 1e6 on beams", &m);
        for lc in m.load_cases.values_mut().filter(|c| c.case_type == LoadCaseType::Modal) {
            lc.modal_sub_type = ModalSubType::Eigen;
            lc.loads.clear();
            lc.ritz_controls.clear();
            lc.max_modes = 12;
        }
        row(file, "V8b same, Eigen", &m);

        let mut m = base.clone();
        m.links.clear();
        let used: std::collections::HashSet<String> = m
            .frames
            .values()
            .flat_map(|f| [f.point_i.clone(), f.point_j.clone()])
            .collect();
        m.points.retain(|name, _| used.contains(name));
        row(file, "V9 dampers and unused joints removed", &m);
    }
}
