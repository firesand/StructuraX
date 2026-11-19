//! Divide with a length ratio and break at intersections, checked on the
//! analysis: closed-form beam deflections.
//!
//! Units: arbitrary consistent ones. Every beam has E I = 1e4 (E = 1e4,
//! I = 1) and no shear area (as2 = as3 = 0), so the nodal values of the
//! element model are exactly the Euler-Bernoulli closed forms. Tolerance 1e-9
//! relative to the closed form.

use sap_model::*;
use sap_solver::analyze_model;

const EI: f64 = 1e4;

fn model() -> SapModel {
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    let mut material = Material::default_steel_kip_in();
    material.name = "MAT".into();
    material.e = EI;
    material.u = 0.3;
    material.g = EI / 2.6;
    material.unit_weight = 0.0;
    material.unit_mass = 0.0;
    m.materials.insert("MAT".into(), material);
    m.frame_sections.insert(
        "S".into(),
        FrameSection {
            name: "S".into(),
            material: "MAT".into(),
            prop_type: FramePropType::General,
            area: 20.0,
            i33: 1.0,
            i22: 1.0,
            torsional_j: 1.0,
            as2: 0.0,
            as3: 0.0,
            modifiers: [1.0; 8],
            ..FrameSection::default()
        },
    );
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
    m
}

fn close(actual: f64, expected: f64, what: &str) {
    assert!(
        (actual - expected).abs() <= 1e-9 * expected.abs(),
        "{what}: got {actual}, expected {expected}"
    );
}

fn uz(m: &SapModel, joint: &str) -> f64 {
    m.case_results["P"].joint_displacements[joint][2]
}

#[test]
fn ratio_division_keeps_the_closed_form_deflection_of_a_simply_supported_beam() {
    // L = 10, n = 4, r = 8: joints at 2/3, 2 and 14/3. Uniform w = 3 downward:
    // y(x) = w x (L^3 - 2 L x^2 + x^3) / (24 E I); at x = 2 that is
    // w * 2 * (1000 - 80 + 8) / (24 E I) = 1856 w / (24 E I) = 77.333 w / E I.
    let (length, w) = (10.0, 3.0);
    let mut m = model();
    m.add_point("A", 0.0, 0.0, 0.0);
    m.add_point("B", length, 0.0, 0.0);
    m.points["A"].restraint = [true, true, true, true, false, false];
    m.points["B"].restraint = [false, true, true, false, false, false];
    m.add_frame("F", "A", "B", "S");
    m.frames["F"].loads_dist.push(FrameLoadDistributed {
        load_pat: "P".into(),
        my_type: 1,
        dir: 6,
        csys: "GLOBAL".into(),
        rel_dist1: 0.0,
        rel_dist2: 1.0,
        val1: -w,
        val2: -w,
    });
    m.frames["F"].selected = true;
    m.divide_selected_frames_ratio(4, 8.0).unwrap();
    assert_eq!(m.frames.len(), 4);
    analyze_model(&mut m).unwrap();
    let deflection = |x: f64| w * x * (length.powi(3) - 2.0 * length * x * x + x.powi(3)) / (24.0 * EI);
    let mut checked = 0;
    for (name, p) in &m.points {
        if name == "A" || name == "B" {
            continue;
        }
        close(-uz(&m, name), deflection(p.x), &format!("joint at x = {}", p.x));
        checked += 1;
    }
    assert_eq!(checked, 3);
    // The joint at x = 2 follows the stated number 77.333 w / (E I).
    let (_, p2) = m.points.iter().find(|(_, p)| (p.x - 2.0).abs() < 1e-9).unwrap();
    close(deflection(p2.x), 1856.0 * w / (24.0 * EI), "closed form at x = 2");
    // The support reactions are w L / 2 each.
    let reactions = &m.case_results["P"].joint_reactions;
    close(reactions["A"][2], w * length / 2.0, "reaction at A");
    close(reactions["B"][2], w * length / 2.0, "reaction at B");
}

/// Two beams that cross at their middle without a joint: beam "1" along X
/// from (0, 0, 0) to (10, 0, 0), beam "2" along Y from (5, -5, 0) to
/// (5, 5, 0). UX, UY, UZ are held at all four ends, twist at one end of each
/// beam; a downward P = 10 acts at the middle of beam 1.
fn crossing_beams(p: f64) -> SapModel {
    let mut m = model();
    for (n, c) in [
        ("1", [0.0, 0.0, 0.0]),
        ("2", [10.0, 0.0, 0.0]),
        ("3", [5.0, -5.0, 0.0]),
        ("4", [5.0, 5.0, 0.0]),
    ] {
        m.add_point(n, c[0], c[1], c[2]);
        m.points[n].restraint = [true, true, true, false, false, false];
    }
    m.points["1"].restraint[3] = true; // RX: twist of beam 1
    m.points["3"].restraint[4] = true; // RY: twist of beam 2
    m.add_frame("1", "1", "2", "S");
    m.add_frame("2", "3", "4", "S");
    m.frames["1"].loads_point.push(FrameLoadPoint {
        load_pat: "P".into(),
        my_type: 1,
        dir: 6,
        csys: "GLOBAL".into(),
        rel_dist: 0.5,
        val: -p,
    });
    m.frames.values_mut().for_each(|f| f.selected = true);
    m
}

#[test]
fn breaking_crossing_beams_connects_them_and_halves_the_deflection() {
    let (p, length) = (10.0f64, 10.0f64);
    // Before the break the beams are independent: delta = P L^3 / (48 E I).
    // (Beam 1 gets a joint at its middle to read the deflection; beam 2 does
    // not share it.)
    let mut separate = crossing_beams(p);
    separate.frames.get_mut("2").unwrap().selected = false;
    let middle = separate.divide_selected_frames(2).unwrap().points_created[0].clone();
    analyze_model(&mut separate).unwrap();
    close(-uz(&separate, &middle), p * length.powi(3) / (48.0 * EI), "deflection before the break");

    // After the break the load falls on the right piece at the shared joint;
    // both beams carry half: delta = P L^3 / (96 E I), reactions P / 4.
    let mut m = crossing_beams(p);
    let summary = m.break_selected_frames_at_intersections().unwrap();
    assert_eq!(summary.points_created.len(), 1);
    let center = summary.points_created[0].clone();
    assert_eq!((m.points[&center].x, m.points[&center].y, m.points[&center].z), (5.0, 0.0, 0.0));
    analyze_model(&mut m).unwrap();
    close(-uz(&m, &center), p * length.powi(3) / (96.0 * EI), "deflection after the break");
    let reactions = &m.case_results["P"].joint_reactions;
    for end in ["1", "2", "3", "4"] {
        close(reactions[end][2], p / 4.0, &format!("reaction at {end}"));
    }
    // By symmetry no torsion is coupled in.
    for f in m.frames.values() {
        for s in &m.case_results["P"].frame_forces[&f.name] {
            assert!(s.t.abs() < 1e-9, "torsion {} in {}", s.t, f.name);
        }
    }
}

#[test]
fn breaking_a_beam_at_a_selected_span_joint_keeps_the_response() {
    // Simply supported, L = 10, uniform w = 2, and a joint at x = 4 that the
    // beam does not use yet. The break adds the joint to the beam; the
    // response at every joint stays the closed form.
    let (length, w, x0) = (10.0, 2.0, 4.0);
    let mut m = model();
    m.add_point("A", 0.0, 0.0, 0.0);
    m.add_point("B", length, 0.0, 0.0);
    m.add_point("C", x0, 0.0, 0.0);
    m.points["A"].restraint = [true, true, true, true, false, false];
    m.points["B"].restraint = [false, true, true, false, false, false];
    m.add_frame("F", "A", "B", "S");
    m.frames["F"].loads_dist.push(FrameLoadDistributed {
        load_pat: "P".into(),
        my_type: 1,
        dir: 6,
        csys: "GLOBAL".into(),
        rel_dist1: 0.0,
        rel_dist2: 1.0,
        val1: -w,
        val2: -w,
    });
    m.frames["F"].selected = true;
    m.points["C"].selected = true;
    m.break_selected_frames_at_intersections().unwrap();
    assert_eq!(m.frames.len(), 2);
    analyze_model(&mut m).unwrap();
    let want = w * x0 * (length.powi(3) - 2.0 * length * x0 * x0 + x0.powi(3)) / (24.0 * EI);
    close(-uz(&m, "C"), want, "deflection at the broken joint");
    close(m.case_results["P"].joint_reactions["A"][2], w * length / 2.0, "reaction");
}

#[test]
fn a_t_junction_break_connects_the_stem_to_the_beam() {
    // Beam "1" (0..10 along X) is pinned at both ends and twist-held at one.
    // A column "2" runs from (5, 0, 0), a joint that exists but is not part of
    // the beam, down to (5, 0, -5) where it is pinned. Before the break the
    // beam alone carries P at its middle: delta = P L^3 / (48 E I). After it
    // only the beam is split, joint 3 is shared, and the column is an axial
    // spring k_s = E A / L_s under the load:
    // delta = P / (48 E I / L^3 + E A / L_s).
    let (p, length, area, stem) = (10.0f64, 10.0f64, 20.0f64, 5.0f64);
    let build = || {
        let mut m = model();
        for (n, c) in [
            ("1", [0.0, 0.0, 0.0]),
            ("2", [10.0, 0.0, 0.0]),
            ("3", [5.0, 0.0, 0.0]),
            ("4", [5.0, 0.0, -stem]),
        ] {
            m.add_point(n, c[0], c[1], c[2]);
        }
        m.points["1"].restraint = [true, true, true, true, false, false];
        m.points["2"].restraint = [true, true, true, false, false, false];
        m.points["4"].restraint = [true, true, true, false, false, false];
        m.add_frame("1", "1", "2", "S");
        m.add_frame("2", "4", "3", "S");
        m.frames["1"].loads_point.push(FrameLoadPoint {
            load_pat: "P".into(),
            my_type: 1,
            dir: 6,
            csys: "GLOBAL".into(),
            rel_dist: 0.5,
            val: -p,
        });
        m.frames.values_mut().for_each(|f| f.selected = true);
        m
    };
    let mut m = build();
    let summary = m.break_selected_frames_at_intersections().unwrap();
    assert!(summary.points_created.is_empty());
    assert_eq!(m.frames.len(), 3, "only the beam is split");
    assert_eq!((m.frames["2"].point_i.as_str(), m.frames["2"].point_j.as_str()), ("4", "3"));
    analyze_model(&mut m).unwrap();
    let k_beam = 48.0 * EI / length.powi(3);
    let k_stem = EI * area / stem; // E = 1e4 = EI numerically, I = 1
    close(-uz(&m, "3"), p / (k_beam + k_stem), "deflection of the joined beam and stem");
    close(m.case_results["P"].joint_reactions["4"][2], p * k_stem / (k_beam + k_stem), "force in the stem");
    let total: f64 = m.case_results["P"].joint_reactions.values().map(|r| r[2]).sum();
    close(total, p, "sum of vertical reactions");
}
