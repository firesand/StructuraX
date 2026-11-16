//! Modal participation factors, mass ratios and load-participation items in
//! the global directions, including joints with local axes and the rotational
//! accelerations about the centre of mass.
//!
//! Source: CSI Analysis Reference Manual, "Acceleration Loads" (printed p. 339)
//! and "Modal Analysis Output" (pp. 376-378): translational factors are taken
//! along global X, Y, Z; RX, RY, RZ are reported about the centre of mass,
//! with the lever arms measured from the centroids of the masses acting in
//! each translational component (RX: y of UZ and z of UY; RY: z of UX and
//! x of UZ; RZ: y of UX and x of UY; the printed "z centroid of the UY load"
//! for RZ is read as the x centroid, the only one that can locate a vertical
//! axis). Closed forms are derived in each test. Units: kip, in, s.

use sap_model::*;
use sap_solver::frame::joint_local_to_global_matrix;
use sap_solver::{analyze_model, SolverError};

fn close(got: f64, want: f64, tol: f64, what: &str) {
    assert!(
        (got - want).abs() <= tol,
        "{what}: got {got}, want {want}, tol {tol}"
    );
}

fn base_model() -> SapModel {
    let mut m = SapModel::default();
    m.load_patterns["DEAD"].self_wt_multiplier = 0.0;
    m.load_cases.clear();
    m
}

fn add_modal(m: &mut SapModel, name: &str, n: usize) {
    m.load_cases
        .insert(name.into(), LoadCase::modal_eigen(name, n));
}

fn ritz_loads(m: &mut SapModel, name: &str, loads: &[&str], n: usize) {
    let mut c = LoadCase::modal_eigen(name, n);
    c.modal_sub_type = ModalSubType::Ritz;
    c.loads = loads
        .iter()
        .map(|l| LoadCaseAssignment {
            load_name: (*l).into(),
            scale_factor: 1.0,
        })
        .collect();
    m.load_cases.insert(name.into(), c);
}

// ---------------------------------------------------------------------------
// R1: a rotated joint. Joint C, mass 2 in every translation, local U3 and all
// rotations restrained, global springs kx = 50 and ky = 200 (spring_is_local =
// false). In the global frame the system is two uncoupled oscillators, w^2 =
// kx/m = 25 and ky/m = 100, whatever the joint axes are.
// ---------------------------------------------------------------------------
fn single_joint(angle: f64) -> SapModel {
    let mut m = base_model();
    m.add_point("C", 0.0, 0.0, 0.0);
    let c = &mut m.points["C"];
    c.mass = [2.0, 2.0, 2.0, 0.0, 0.0, 0.0];
    c.restraint = [false, false, true, true, true, true];
    c.spring = [50.0, 200.0, 0.0, 0.0, 0.0, 0.0];
    c.spring_is_local = false;
    c.local_axes_deg = [angle, 0.0, 0.0];
    add_modal(&mut m, "M", 2);
    m
}

#[test]
fn r1_rotated_joint_reports_global_factors_and_ratios() {
    for angle in [0.0, 30.0, -30.0, 90.0, 137.0] {
        let mut m = single_joint(angle);
        analyze_model(&mut m).unwrap();
        let modes = &m.case_results["M"].modes;
        assert_eq!(modes.len(), 2);
        close(modes[0].eigenvalue, 25.0, 1e-9, "w^2 of mode 1");
        close(modes[1].eigenvalue, 100.0, 1e-9, "w^2 of mode 2");
        // Mode 1 moves along global X only: (UX, UY) = (1, 0); mode 2 (0, 1).
        // Factor: |phi^T M e_x| = m |phi_x| with phi_x = 1/sqrt(m): sqrt(2).
        close(modes[0].modal_mass_ratios[0], 1.0, 1e-12, "mode 1 UX");
        close(modes[0].modal_mass_ratios[1], 0.0, 1e-12, "mode 1 UY");
        close(modes[1].modal_mass_ratios[0], 0.0, 1e-12, "mode 2 UX");
        close(modes[1].modal_mass_ratios[1], 1.0, 1e-12, "mode 2 UY");
        close(
            modes[0].participation_factors[0].abs(),
            2.0f64.sqrt(),
            1e-12,
            "factor",
        );
        // UZ is restrained (no free mass): ratio 0 with no division by zero.
        assert_eq!(modes[0].modal_mass_ratios[2], 0.0);
        // Rotations: no free rotational DOF and the lever arms vanish.
        for d in 3..6 {
            assert_eq!(modes[0].modal_mass_ratios[d], 0.0);
            assert_eq!(modes[1].modal_mass_ratios[d], 0.0);
        }
    }
}

#[test]
fn r1_response_spectrum_uses_the_global_factor() {
    // Constant Sa = 1, direction U1, SRSS. Mode 1 only has a nonzero global
    // UX factor: q1 = f1 Sa / w1^2 = sqrt(2) / 25, and the global displacement
    // is q1 phi_x = sqrt(2)/25 / sqrt(2) = 0.04 along X, 0 along Y. The result
    // is stored in the joint's local axes as a magnitude; for A = -30 degrees
    // both local components of mode 1 (R^T e_x / sqrt 2) are positive, so
    // R * local is the global vector. (The old i % 6 sum gave a factor 0.75
    // of this at 30 degrees.)
    let mut m = single_joint(-30.0);
    m.functions.insert(
        "SPEC".into(),
        DynamicFunction {
            name: "SPEC".into(),
            is_time_history: false,
            damping_ratio: 0.05,
            points: vec![(0.0, 1.0), (10.0, 1.0)],
        },
    );
    let mut rs = LoadCase::linear_static("RS", "DEAD", 1.0);
    rs.case_type = LoadCaseType::ResponseSpectrum;
    rs.loads.clear();
    rs.initial_case = Some("M".into());
    rs.modal_combo = ModalComboMethod::Srss;
    rs.spectrum_loads = vec![("SPEC".into(), "U1".into(), 1.0)];
    m.load_cases.insert("RS".into(), rs);
    analyze_model(&mut m).unwrap();
    let local = m.case_results["RS"].joint_displacements["C"];
    let r = joint_local_to_global_matrix(&m.points["C"]);
    let global: Vec<f64> = (0..3)
        .map(|k| (0..3).map(|d| r[k][d] * local[d]).sum())
        .collect();
    close(global[0], 0.04, 1e-12, "global UX");
    close(global[1], 0.0, 1e-12, "global UY");
}

// ---------------------------------------------------------------------------
// A space frame for completeness and bitwise checks: four free joints on top
// of four columns joined by a square of beams, all six DOFs of every free
// joint carry (different) mass, no element mass.
// ---------------------------------------------------------------------------
fn space_frame() -> SapModel {
    let mut m = base_model();
    let mut mat = Material::default_steel_kip_in();
    mat.name = "M".into();
    mat.e = 1000.0;
    mat.g = 400.0;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0;
    m.materials.insert("M".into(), mat);
    m.frame_sections.insert(
        "S".into(),
        FrameSection {
            name: "S".into(),
            material: "M".into(),
            area: 10.0,
            torsional_j: 40.0,
            i33: 100.0,
            i22: 60.0,
            as2: 0.0,
            as3: 0.0,
            ..FrameSection::default()
        },
    );
    let corners = [(0.0, 0.0), (120.0, 0.0), (120.0, 90.0), (0.0, 90.0)];
    for (i, (x, y)) in corners.iter().enumerate() {
        let b = format!("B{i}");
        let t = format!("T{i}");
        m.add_point(b.clone(), *x, *y, 0.0);
        m.points[&b].restraint = [true; 6];
        m.add_point(t.clone(), *x, *y, 100.0);
        let f = 1.0 + 0.3 * i as f64;
        m.points[&t].mass = [0.02 * f, 0.025 * f, 0.03 * f, 2.0 * f, 3.0 * f, 4.0 * f];
        m.add_frame(format!("C{i}"), b, t, "S");
    }
    for i in 0..4 {
        m.add_frame(
            format!("G{i}"),
            format!("T{i}"),
            format!("T{}", (i + 1) % 4),
            "S",
        );
    }
    add_modal(&mut m, "M", 24);
    m
}

/// R3: all 24 modes of a model whose every free DOF carries mass: the sum of
/// the ratios is exactly the complete mass (1) in all six directions, because
/// sum_n (phi_n^T M r)^2 = r^T M r for an M-orthonormal basis.
#[test]
fn r3_all_modes_sum_to_one_in_six_directions() {
    let mut m = space_frame();
    analyze_model(&mut m).unwrap();
    let modes = &m.case_results["M"].modes;
    assert_eq!(modes.len(), 24);
    for d in 0..6 {
        let sum: f64 = modes.iter().map(|mode| mode.modal_mass_ratios[d]).sum();
        close(sum, 1.0, 1e-10, &format!("sum of ratios, direction {d}"));
    }
}

/// R1b: without joint local axes the translational factors and ratios are
/// bitwise those of the old accumulation (`pf[i % 6] += m phi`,
/// `total[i % 6] += m`, ratio `pf^2 / total`), copied here.
#[test]
fn r1b_identity_axes_keep_the_old_translational_bits() {
    let mut m = space_frame();
    analyze_model(&mut m).unwrap();
    let modes = &m.case_results["M"].modes;
    // Mass vector in solver order (point insertion order), restrained = 0.
    let mut mass = Vec::new();
    for p in m.points.values() {
        for d in 0..6 {
            mass.push(if p.restraint[d] { 0.0 } else { p.mass[d] });
        }
    }
    let names: Vec<&String> = m.points.keys().collect();
    let mut total = [0.0f64; 6];
    for (i, mi) in mass.iter().enumerate() {
        total[i % 6] += mi;
    }
    for mode in modes {
        let phi: Vec<f64> = names.iter().flat_map(|n| mode.mode_shape[*n]).collect();
        let mut pf = [0.0f64; 6];
        for i in 0..phi.len() {
            pf[i % 6] += mass[i] * phi[i];
        }
        for d in 0..3 {
            assert_eq!(
                mode.participation_factors[d].to_bits(),
                pf[d].to_bits(),
                "pf {d}"
            );
            let ratio = if total[d] > 1e-18 {
                pf[d] * pf[d] / total[d]
            } else {
                0.0
            };
            assert_eq!(
                mode.modal_mass_ratios[d].to_bits(),
                ratio.to_bits(),
                "ratio {d}"
            );
        }
    }
}

// ---------------------------------------------------------------------------
// R2: two offset masses. A at x = -2 (m = 1) and B at x = +2 (m = 3), only UY
// free, springs ky = 1 and 12: w^2 = 1 (A) and 4 (B). The UY mass centroid is
// x_c = (-2 + 6) / 4 = 1, so the rotation about the vertical axis through it
// has r = x - 1 = -3 at A and +1 at B and M_rz = 1*9 + 3*1 = 12.
// Mode 1 (A only, phi_A = 1): f_rz = 1*1*(-3) = -3, ratio 9/12 = 0.75; f_y = 1,
// ratio 1/4 = 0.25. Mode 2 (B only, phi_B = 1/sqrt 3): f_rz = 3 phi_B * 1 =
// sqrt 3, ratio 3/12 = 0.25; f_y = sqrt 3, ratio 3/4. The old code, which sums
// the rotational-DOF mass (zero here), reported RZ = 0.
// ---------------------------------------------------------------------------
fn offset_masses() -> SapModel {
    let mut m = base_model();
    for (name, x, mass, k) in [("A", -2.0, 1.0, 1.0), ("B", 2.0, 3.0, 12.0)] {
        m.add_point(name, x, 0.0, 0.0);
        let p = &mut m.points[name];
        p.restraint = [true, false, true, true, true, true];
        p.mass = [mass, mass, mass, 0.0, 0.0, 0.0];
        p.spring = [0.0, k, 0.0, 0.0, 0.0, 0.0];
    }
    add_modal(&mut m, "M", 2);
    m
}

#[test]
fn r2_two_offset_masses_rz_about_the_centre_of_mass() {
    let mut m = offset_masses();
    analyze_model(&mut m).unwrap();
    let modes = &m.case_results["M"].modes;
    close(modes[0].eigenvalue, 1.0, 1e-12, "w^2 1");
    close(modes[1].eigenvalue, 4.0, 1e-12, "w^2 2");
    close(modes[0].modal_mass_ratios[5], 0.75, 1e-12, "RZ 1");
    close(modes[1].modal_mass_ratios[5], 0.25, 1e-12, "RZ 2");
    close(modes[0].modal_mass_ratios[1], 0.25, 1e-12, "UY 1");
    close(modes[1].modal_mass_ratios[1], 0.75, 1e-12, "UY 2");
    close(modes[0].participation_factors[5].abs(), 3.0, 1e-12, "f_rz 1");
    close(
        modes[1].participation_factors[5].abs(),
        3.0f64.sqrt(),
        1e-12,
        "f_rz 2",
    );
    for mode in modes {
        for d in [0, 2, 3, 4] {
            assert_eq!(mode.modal_mass_ratios[d], 0.0, "direction {d}");
        }
    }
    // Only the items that act on mass are listed: UY and RZ.
    let names: Vec<&str> = m.case_results["M"]
        .load_participation
        .iter()
        .map(|p| p.load_name.as_str())
        .collect();
    assert_eq!(names, vec!["UY", "RZ"]);
}

// ---------------------------------------------------------------------------
// R5: one joint, translational masses m = 2 and rotational inertias J =
// (3, 3, 3), uncoupled springs (10, 20, 30, 6, 9, 12): w^2 = k/m or k/J =
// 5, 10, 15 (translations) and 2, 3, 4 (rotations). The mass sits at the
// centre, so every lever arm vanishes and each rotational mode has f = sqrt J,
// ratio J / J = 1 in its own direction and 0 in the others.
// ---------------------------------------------------------------------------
#[test]
fn r5_single_joint_each_mode_is_one_direction() {
    let mut m = base_model();
    m.add_point("C", 1.0, 2.0, 3.0);
    let c = &mut m.points["C"];
    c.mass = [2.0, 2.0, 2.0, 3.0, 3.0, 3.0];
    c.spring = [10.0, 20.0, 30.0, 6.0, 9.0, 12.0];
    add_modal(&mut m, "M", 6);
    analyze_model(&mut m).unwrap();
    let modes = &m.case_results["M"].modes;
    // Ascending w^2: RX 2, RY 3, RZ 4, UX 5, UY 10, UZ 15.
    let dirs = [3, 4, 5, 0, 1, 2];
    let w2 = [2.0, 3.0, 4.0, 5.0, 10.0, 15.0];
    for (k, mode) in modes.iter().enumerate() {
        close(mode.eigenvalue, w2[k], 1e-9, "w^2");
        for d in 0..6 {
            let want = if d == dirs[k] { 1.0 } else { 0.0 };
            close(
                mode.modal_mass_ratios[d],
                want,
                1e-12,
                &format!("mode {k} dir {d}"),
            );
        }
    }
    close(
        modes[2].participation_factors[5].abs(),
        3.0f64.sqrt(),
        1e-12,
        "f_rz",
    );
}

// ---------------------------------------------------------------------------
// R6, R7 and the item list.
// ---------------------------------------------------------------------------
fn names(m: &SapModel, case: &str) -> Vec<String> {
    m.case_results[case]
        .load_participation
        .iter()
        .map(|p| p.load_name.clone())
        .collect()
}

/// R6: for an acceleration item the dynamic participation is the mass ratio of
/// its direction, mode by mode, so the cumulative values agree too.
#[test]
fn r6_acceleration_items_equal_the_mass_ratios() {
    let rotated = {
        let mut m = space_frame();
        for (i, p) in m.points.values_mut().enumerate() {
            if i % 2 == 1 {
                p.local_axes_deg = [20.0 + 5.0 * i as f64, 0.0, 0.0];
            }
        }
        m
    };
    for mut m in [space_frame(), rotated] {
        analyze_model(&mut m).unwrap();
        let cr = &m.case_results["M"];
        assert_eq!(names(&m, "M"), ["UX", "UY", "UZ", "RX", "RY", "RZ"]);
        for (d, item) in cr.load_participation.iter().enumerate() {
            let mut cum = 0.0;
            for (n, mode) in cr.modes.iter().enumerate() {
                cum += mode.modal_mass_ratios[d];
                close(
                    item.dynamic_ratio[n],
                    mode.modal_mass_ratios[d],
                    1e-12,
                    "dynamic ratio",
                );
                close(item.dynamic_cumulative[n], cum, 1e-12, "cumulative");
            }
        }
    }
}

/// R7 and the item order: the starting loads come first in case order (U3 is
/// the imported spelling of UZ and is not listed twice), then every one of
/// UX..RZ that is not yet there. The static participation of an acceleration
/// that starts the Ritz generation is 1 (reference p. 380).
#[test]
fn r7_ritz_items_start_with_the_starting_loads_and_static_ratio_is_one() {
    let mut m = space_frame();
    ritz_loads(&mut m, "R", &["U3", "UX"], 6);
    analyze_model(&mut m).unwrap();
    assert_eq!(names(&m, "R"), ["U3", "UX", "UY", "RX", "RY", "RZ"]);
    let parts = &m.case_results["R"].load_participation;
    for item in &parts[..2] {
        close(*item.static_cumulative.last().unwrap(), 1.0, 1e-10, "static");
        assert!(item.vectors_generated > 0);
    }
    for item in &parts[2..] {
        assert_eq!(item.vectors_generated, 0);
    }
    // The other items are still the mass ratios of the generated modes.
    let cr = &m.case_results["R"];
    for (d, name) in [(1usize, "UY"), (3, "RX")] {
        let item = parts.iter().find(|p| p.load_name == name).unwrap();
        for (n, mode) in cr.modes.iter().enumerate() {
            close(item.dynamic_ratio[n], mode.modal_mass_ratios[d], 1e-12, name);
        }
    }
}

#[test]
fn rotational_start_load_is_still_refused() {
    let mut m = space_frame();
    ritz_loads(&mut m, "R", &["UX", "RZ"], 4);
    let err = analyze_model(&mut m).expect_err("must refuse");
    assert!(matches!(err, SolverError::UnsupportedInput(_)));
    let text = err.to_string();
    assert!(
        text.contains("rotational") && text.contains("'RZ'") && text.contains("'R'"),
        "{text}"
    );
}
