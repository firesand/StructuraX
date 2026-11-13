//! Worked examples from SAP2000 training material, rebuilt through the model
//! API and compared with the values the documents print and with closed forms.
//!
//! Sources (Indonesian-language training documents, not part of the
//! repository): "Modul SAP2000 v11" sections 3.1 to 4.1, a SAP2000 v14 worked
//! example of a continuous concrete beam, a SAP2000 v14 tutorial of a 2D
//! concrete portal, and the "Program SAP2000 v20" ebook chapters 3 to 6.
//!
//! The documents use ACI 318-99/-05/-11 and AISC 360-05; this project has ACI
//! 318-14 and AISC 360-10. For the simple members here the editions give the
//! same required steel. Where a document's number rests on a SAP2000 default
//! this project does not have, the test says so and checks the closed form.
use std::collections::HashMap;

use sap_design::steel::{try_run_steel_design_with_overwrites, SteelMemberOverwrite};
use sap_design::{try_run_concrete_design, try_run_steel_design};
use sap_model::*;
use sap_section::{make_i_section, make_rectangular};
use sap_solver::analyze_model;
use sap_units::Units;

const PIN: [bool; 6] = [true, true, true, false, false, false];
const ROLLER: [bool; 6] = [false, true, true, false, false, false];
/// kgf/m2 per MPa.
const MPA_KGF_M2: f64 = 1.0e6 / 9.80665;

fn close(what: &str, got: f64, expected: f64, tolerance: f64) {
    assert!(
        (got - expected).abs() <= tolerance,
        "{what}: {got} vs {expected} (tolerance {tolerance})"
    );
}

fn base(units: Units, plane_frame: bool) -> SapModel {
    let mut m = SapModel::new(units);
    m.load_cases.clear();
    m.load_patterns.clear();
    if plane_frame {
        m.active_dofs = [true, false, true, false, true, false];
    }
    m
}

fn pattern(m: &mut SapModel, name: &str, self_weight: f64) {
    m.load_patterns.insert(
        name.into(),
        LoadPattern {
            name: name.into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: self_weight,
        },
    );
    m.load_cases
        .insert(name.into(), LoadCase::linear_static(name, name, 1.0));
}

fn combo(m: &mut SapModel, name: &str, terms: &[(&str, f64)]) {
    m.load_combos.insert(
        name.into(),
        LoadCombo {
            name: name.into(),
            combo_type: ComboType::LinearAdd,
            cases: terms.iter().map(|(c, f)| (c.to_string(), *f)).collect(),
        },
    );
}

fn material(m: &mut SapModel, from: &str, name: &str, edit: impl FnOnce(&mut Material)) {
    let mut mat = m.materials[from].clone();
    mat.name = name.into();
    edit(&mut mat);
    m.materials.insert(name.into(), mat);
}

/// Downward (gravity) distributed load between relative positions `a` and `b`.
fn gravity_udl(m: &mut SapModel, frame: &str, pat: &str, a: f64, b: f64, value: f64) {
    m.frames[frame].loads_dist.push(FrameLoadDistributed {
        load_pat: pat.into(),
        my_type: 1,
        dir: 10,
        csys: "GLOBAL".into(),
        rel_dist1: a,
        rel_dist2: b,
        val1: value,
        val2: value,
    });
}

fn point_load(m: &mut SapModel, frame: &str, pat: &str, dir: i32, rel: f64, value: f64) {
    m.frames[frame].loads_point.push(FrameLoadPoint {
        load_pat: pat.into(),
        my_type: 1,
        dir,
        csys: "GLOBAL".into(),
        rel_dist: rel,
        val: value,
    });
}

fn joint_force(m: &mut SapModel, joint: &str, pat: &str, force: [f64; 6]) {
    m.points[joint].loads_force.push(PointLoadForce {
        load_pat: pat.into(),
        csys: "GLOBAL".into(),
        force,
    });
}

/// Joints J1, J2, ... along X at `xs` with one frame F1, F2, ... per span.
fn beam_line(m: &mut SapModel, xs: &[f64], section: &str, stations: usize) {
    for (i, x) in xs.iter().enumerate() {
        m.add_point(format!("J{}", i + 1), *x, 0.0, 0.0);
    }
    for i in 1..xs.len() {
        let name = format!("F{i}");
        m.add_frame(name.clone(), format!("J{i}"), format!("J{}", i + 1), section);
        m.frames[name.as_str()].min_stations = stations;
    }
}

fn max_abs_m3(m: &SapModel, case: &str) -> f64 {
    m.case_results[case]
        .frame_forces
        .values()
        .flatten()
        .map(|s| s.m3.abs())
        .fold(0.0, f64::max)
}

fn beam_rebar(material: &str, cover: f64) -> Option<ConcreteRebarConfig> {
    Some(ConcreteRebarConfig {
        is_column: false,
        rebar_mat_long: material.into(),
        rebar_mat_confine: material.into(),
        cover_top: cover,
        cover_bot: cover,
        num_bars_3dir: 3,
        num_bars_2dir: 3,
        total_rebar_area: 0.0,
    })
}

/// Required tension steel of a singly reinforced rectangular section, ACI
/// rectangular stress block: `As = 0.85 fc b / fy (d - sqrt(d^2 - 2 Mu / (0.85 fc phi b)))`.
fn aci_flexural_steel(mu: f64, fc: f64, fy: f64, b: f64, d: f64) -> f64 {
    0.85 * fc * b / fy * (d - (d * d - 2.0 * mu / (0.85 * fc * 0.9 * b)).sqrt())
}

// ---------------------------------------------------------------------------
// Modul SAP2000 v11, sections 3.1 to 3.3: an 8 m beam on a pin and a roller,
// no self weight, kgf and m.
// ---------------------------------------------------------------------------

fn v11_beam() -> SapModel {
    let mut m = base(Units::KgfMC, true);
    let section = m.frame_sections.keys().next().unwrap().clone();
    // 33 stations put one every 0.25 m.
    beam_line(&mut m, &[0.0, 8.0], &section, 33);
    m.points["J1"].restraint = PIN;
    m.points["J2"].restraint = ROLLER;
    pattern(&mut m, "DEAD", 0.0);
    m
}

#[test]
fn v11_module_beams_match_the_printed_reactions_and_moments() {
    // 3.1: 4 kgf at mid-span and 2 kgf at 6 m. Printed: reactions 2.50 and
    // 3.50 kgf, maximum moment 10.00 kgf-m.
    let mut m = v11_beam();
    point_load(&mut m, "F1", "DEAD", 10, 0.5, 4.0);
    point_load(&mut m, "F1", "DEAD", 10, 0.75, 2.0);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["DEAD"];
    close("3.1 reaction A", r.joint_reactions["J1"][2], 2.5, 1e-9);
    close("3.1 reaction B", r.joint_reactions["J2"][2], 3.5, 1e-9);
    close("3.1 maximum moment", max_abs_m3(&m, "DEAD"), 10.0, 1e-9);

    // 3.2: 8 kgf/m over the first 6 m. Printed: reactions 30.00 and 18.00
    // kgf, moment label about 56. Closed form: zero shear at 3.75 m, where
    // M = 30^2 / (2 x 8) = 56.25 kgf-m.
    let mut m = v11_beam();
    gravity_udl(&mut m, "F1", "DEAD", 0.0, 0.75, 8.0);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["DEAD"];
    close("3.2 reaction A", r.joint_reactions["J1"][2], 30.0, 1e-9);
    close("3.2 reaction B", r.joint_reactions["J2"][2], 18.0, 1e-9);
    close("3.2 maximum moment", max_abs_m3(&m, "DEAD"), 56.25, 1e-9);

    // 3.3: 1.5 kgf/m over the first 3.5 m, 2 kgf at 5.5 m, and a 1.5 kgf load
    // at 30 degrees at 6.5 m that the document splits into 0.75 kgf downward
    // and 1.3 kgf along the beam. Printed: A 4.87 vertical and 1.30
    // horizontal, B 3.13. Closed form: B = (5.25 x 1.75 + 2 x 5.5 + 0.75 x
    // 6.5) / 8 = 3.1328, A = 8 - 3.1328 = 4.8672.
    let mut m = v11_beam();
    gravity_udl(&mut m, "F1", "DEAD", 0.0, 3.5 / 8.0, 1.5);
    point_load(&mut m, "F1", "DEAD", 10, 5.5 / 8.0, 2.0);
    point_load(&mut m, "F1", "DEAD", 10, 6.5 / 8.0, 0.75);
    point_load(&mut m, "F1", "DEAD", 4, 6.5 / 8.0, -1.3);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["DEAD"];
    let b = (5.25 * 1.75 + 2.0 * 5.5 + 0.75 * 6.5) / 8.0;
    close("3.3 reaction B", r.joint_reactions["J2"][2], b, 1e-9);
    close("3.3 reaction A", r.joint_reactions["J1"][2], 8.0 - b, 1e-9);
    close("3.3 reaction A horizontal", r.joint_reactions["J1"][0], 1.3, 1e-9);
    // Against the two printed decimals.
    close("3.3 printed A", r.joint_reactions["J1"][2], 4.87, 5e-3);
    close("3.3 printed B", r.joint_reactions["J2"][2], 3.13, 5e-3);
}

// ---------------------------------------------------------------------------
// Modul SAP2000 v11, section 4.1: concrete beam 30 x 40 cm on three supports,
// spans 4 m and 3 m, kgf and m.
// ---------------------------------------------------------------------------

#[test]
fn v11_two_span_concrete_beam_matches_the_printed_reactions() {
    let mut m = base(Units::KgfMC, true);
    // K-250: fc 21 MPa, E = 4700 sqrt(fc) = 21538.106 MPa, 2400 kgf/m3.
    material(&mut m, "4000Psi", "Beton", |c| {
        c.e = 21538.106 * MPA_KGF_M2;
        c.u = 0.2;
        c.g = c.e / 2.4;
        c.unit_weight = 2400.0;
        c.fc = 21.0 * MPA_KGF_M2;
    });
    m.frame_sections.insert(
        "BALOK30X40".into(),
        make_rectangular("BALOK30X40", "Beton", 0.4, 0.3),
    );
    beam_line(&mut m, &[0.0, 4.0, 7.0], "BALOK30X40", 9);
    m.points["J1"].restraint = PIN;
    m.points["J2"].restraint = ROLLER;
    m.points["J3"].restraint = ROLLER;
    pattern(&mut m, "DEAD", 1.0);
    pattern(&mut m, "LIVE", 0.0);
    for frame in ["F1", "F2"] {
        gravity_udl(&mut m, frame, "DEAD", 0.0, 1.0, 25.0);
        gravity_udl(&mut m, frame, "LIVE", 0.0, 1.0, 15.0);
    }
    combo(&mut m, "COMB1", &[("DEAD", 1.2), ("LIVE", 1.6)]);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["COMB1"];
    // Printed for COMB1 = 1.2 D + 1.6 L, to 0.01 kgf. The beam is deep enough
    // for shear deformation to move the middle reaction by about 1 %, so
    // these values also check the shear-flexible element.
    close("reaction A", r.joint_reactions["J1"][2], 638.41, 0.01);
    close("reaction at the middle support", r.joint_reactions["J2"][2], 1773.79, 0.01);
    close("reaction B", r.joint_reactions["J3"][2], 385.01, 0.01);
    // Statics: total load (1.2 x (0.12 x 2400 + 25) + 1.6 x 15) x 7 m.
    let total: f64 = r.joint_reactions.values().map(|f| f[2]).sum();
    close("total reaction", total, (1.2 * 313.0 + 24.0) * 7.0, 1e-8);
}

// ---------------------------------------------------------------------------
// SAP2000 v14 worked example: continuous concrete beam 20 x 30 cm, two spans
// of 4 m, fixed at A and pinned at B and C, kgf and m.
// ---------------------------------------------------------------------------

#[test]
fn v14_continuous_concrete_beam_matches_printed_forces_and_top_steel() {
    let mut m = base(Units::KgfMC, true);
    let (fc, fy) = (17.5 * MPA_KGF_M2, 413.68544 * MPA_KGF_M2);
    // The document changes only fc on the default concrete and leaves the
    // default A615 Grade 60 rebar.
    material(&mut m, "4000Psi", "Beton", |c| {
        c.e = 24855.578 * MPA_KGF_M2;
        c.u = 0.2;
        c.g = c.e / 2.4;
        c.fc = fc;
    });
    material(&mut m, "A992Fy50", "A615Gr60", |s| s.fy = fy);
    let mut section = make_rectangular("Balok 20x30", "Beton", 0.3, 0.2);
    section.concrete_rebar = beam_rebar("A615Gr60", 0.03);
    m.frame_sections.insert("Balok 20x30".into(), section);
    beam_line(&mut m, &[0.0, 4.0, 8.0], "Balok 20x30", 81);
    m.points["J1"].restraint = [true; 6];
    m.points["J2"].restraint = PIN;
    m.points["J3"].restraint = PIN;
    pattern(&mut m, "DEAD", 1.0);
    pattern(&mut m, "LIVE", 0.0);
    for frame in ["F1", "F2"] {
        gravity_udl(&mut m, frame, "DEAD", 0.0, 1.0, 200.0);
        gravity_udl(&mut m, frame, "LIVE", 0.0, 1.0, 100.0);
    }
    combo(&mut m, "COMB1", &[("DEAD", 1.2), ("LIVE", 1.6)]);
    analyze_model(&mut m).unwrap();

    // Printed for span A-B under 1.2 D + 1.6 L (573.00 kgf/m with self weight).
    let stations = m.case_results["COMB1"].frame_forces["F1"].clone();
    let (at_a, at_b) = (&stations[0], stations.last().unwrap());
    close("shear at A", at_a.v2.abs(), 1065.73, 0.01);
    close("moment at A", at_a.m3, -657.85, 0.01);
    close("moment at B", at_b.m3, -978.90, 0.01);
    close("shear at B (the design Vu)", at_b.v2.abs(), 1226.263, 1e-3);

    try_run_concrete_design(&mut m).unwrap();
    let design = &m.concrete_design_results["F1"];
    let (b, d) = (0.2, 0.3 - 0.03);
    // Top steel. Printed 1.306E-04 m2: the flexural requirement for the
    // support moment, increased by one third because it is below the minimum.
    let top = 4.0 / 3.0 * aci_flexural_steel(at_b.m3.abs(), fc, fy, b, d);
    close("top steel against the closed form", design.as_top, top, 1e-9);
    close("top steel against the document [mm2]", design.as_top * 1e6, 130.6, 0.05);

    // Bottom steel. The document prints 6.447E-05 m2, which is the steel for
    // a design moment of 489.452 kgf-m, half the support moment: the member
    // has SAP2000's default "Sway Special" framing type, whose positive
    // strength at a joint is at least half the negative strength there
    // (ACI 318-14 18.6.3.2). That is the default framing type here as well.
    assert_eq!(m.frames["F1"].concrete_framing, ConcreteFramingType::SwaySpecial);
    let special = 4.0 / 3.0 * aci_flexural_steel(0.5 * at_b.m3.abs(), fc, fy, b, d);
    close("bottom steel against the closed form", design.as_bot, special, 1e-9);
    close("bottom steel against the document [mm2]", design.as_bot * 1e6, 64.47, 0.01);

    // Sway Intermediate asks for one third of the end strength (18.4.2.2),
    // which here is below the maximum positive moment in the span, so both
    // it and Sway Ordinary design the bottom steel for that moment. Closed
    // form for the span: M(x) = -657.85 + 1065.73 x - 573 x^2 / 2.
    let positive = stations.iter().map(|s| s.m3).fold(0.0, f64::max);
    close("maximum positive moment", positive, -657.8467 + 1065.7343f64.powi(2) / (2.0 * 573.0), 0.05);
    assert!(positive > at_b.m3.abs() / 3.0 && positive < 0.5 * at_b.m3.abs());
    let ordinary = 4.0 / 3.0 * aci_flexural_steel(positive, fc, fy, b, d);
    for framing in [ConcreteFramingType::SwayIntermediate, ConcreteFramingType::SwayOrdinary, ConcreteFramingType::NonSway] {
        m.frames["F1"].concrete_framing = framing;
        try_run_concrete_design(&mut m).unwrap();
        let design = &m.concrete_design_results["F1"];
        close("bottom steel without the one-half rule", design.as_bot, ordinary, 1e-9);
        close("top steel is the same for every framing type", design.as_top, top, 1e-9);
    }
    assert!(ordinary < special);
}

/// The moment-strength floors of the framing types on a beam whose end moments
/// differ: closed forms from the ACI clauses, kN and m.
#[test]
fn concrete_framing_types_apply_the_aci_moment_strength_floors() {
    let (fc, fy) = (30.0e3, 400.0e3);
    let build = |framing: ConcreteFramingType| {
        let mut m = base(Units::KnMC, true);
        material(&mut m, "4000Psi", "C30", |c| c.fc = fc);
        material(&mut m, "A992Fy50", "BJTD40", |s| s.fy = fy);
        let mut section = make_rectangular("B30x50", "C30", 0.5, 0.3);
        section.concrete_rebar = beam_rebar("BJTD40", 0.05);
        m.frame_sections.insert("B30x50".into(), section);
        beam_line(&mut m, &[0.0, 6.0], "B30x50", 13);
        m.frames["F1"].concrete_framing = framing;
        // Results by hand: hogging 240 kN-m at I and 90 kN-m at J, sagging
        // 20 kN-m at mid-span, linear in between.
        let mut results = CaseResults { case_name: "U".into(), ..Default::default() };
        let moment = |s: f64| if s <= 0.5 { -240.0 + 520.0 * s } else { 20.0 - 220.0 * (s - 0.5) };
        results.frame_forces.insert(
            "F1".into(),
            (0..=12)
                .map(|k| {
                    let s = k as f64 / 12.0;
                    FrameStationResult { station: 6.0 * s, rel_dist: s, m3: moment(s), ..Default::default() }
                })
                .collect(),
        );
        m.case_results.insert("U".into(), results);
        try_run_concrete_design(&mut m).unwrap();
        m.concrete_design_results["F1"].clone()
    };
    let (b, d) = (0.3, 0.45);
    let steel = |moment: f64| {
        let required = aci_flexural_steel(moment, fc, fy, b, d);
        // Minimum 1.4 b d / fy (MPa), waived when a third more than required is provided.
        required.max((1.4e3 * b * d / fy).min(4.0 / 3.0 * required))
    };
    // Ordinary: the largest hogging moment on top, the sagging moment below.
    let ordinary = build(ConcreteFramingType::SwayOrdinary);
    close("ordinary top", ordinary.as_top, steel(240.0), 1e-9);
    close("ordinary bottom", ordinary.as_bot, steel(20.0), 1e-9);
    // Special: bottom steel for half the largest end hogging moment (120),
    // which also exceeds a quarter of it; the top is unchanged because 240
    // governs over the quarter rule.
    let special = build(ConcreteFramingType::SwaySpecial);
    close("special top", special.as_top, steel(240.0), 1e-9);
    close("special bottom", special.as_bot, steel(120.0), 1e-9);
    // Intermediate: one third at the end (80).
    let intermediate = build(ConcreteFramingType::SwayIntermediate);
    close("intermediate top", intermediate.as_top, steel(240.0), 1e-9);
    close("intermediate bottom", intermediate.as_bot, steel(80.0), 1e-9);
    assert!(ordinary.as_bot < intermediate.as_bot && intermediate.as_bot < special.as_bot);
}

// ---------------------------------------------------------------------------
// SAP2000 v14 tutorial: 2D concrete portal, bays 6 m and 2 m, storeys 5, 4 and
// 4 m, no self weight, kN and m.
// ---------------------------------------------------------------------------

#[test]
fn v14_tutorial_portal_matches_the_printed_column_forces() {
    let mut m = base(Units::KnMC, true);
    material(&mut m, "4000Psi", "Beton 22,5 Mpa", |c| {
        c.e = 2.2294057e7;
        c.u = 0.17;
        c.g = c.e / 2.34;
        c.fc = 22500.0;
    });
    for (name, depth, width) in [
        ("K40/40", 0.4, 0.4),
        ("K30/30", 0.3, 0.3),
        ("B30/50", 0.5, 0.3),
        ("B20/30", 0.3, 0.2),
    ] {
        m.frame_sections
            .insert(name.into(), make_rectangular(name, "Beton 22,5 Mpa", depth, width));
    }
    // Joints 1-4 on line A, 5-8 on line B, 9-12 on line C, numbered upward.
    let (xs, zs) = ([0.0, 6.0, 8.0], [0.0, 5.0, 9.0, 13.0]);
    for (line, x) in xs.iter().enumerate() {
        for (level, z) in zs.iter().enumerate() {
            m.add_point((line * 4 + level + 1).to_string(), *x, 0.0, *z);
        }
    }
    // Frames 1-9 are the columns line by line, 10-12 the 6 m beams, 13-15 the 2 m beams.
    let mut frame = 0;
    for line in 0..3 {
        for level in 0..3 {
            frame += 1;
            let i = line * 4 + level + 1;
            let section = if line == 2 { "K30/30" } else { "K40/40" };
            m.add_frame(frame.to_string(), i.to_string(), (i + 1).to_string(), section);
        }
    }
    for bay in 0..2 {
        for level in 1..4 {
            frame += 1;
            let i = bay * 4 + level + 1;
            let section = if bay == 0 { "B30/50" } else { "B20/30" };
            m.add_frame(frame.to_string(), i.to_string(), (i + 4).to_string(), section);
        }
    }
    m.points["1"].restraint = [true; 6];
    m.points["5"].restraint = [true; 6];
    m.points["9"].restraint = PIN;
    for name in ["MATI", "HIDUP", "GEMPA"] {
        pattern(&mut m, name, 0.0);
    }
    for frame in ["10", "11", "12"] {
        gravity_udl(&mut m, frame, "MATI", 0.0, 1.0, 20.0);
        gravity_udl(&mut m, frame, "HIDUP", 0.0, 1.0, 8.0);
        point_load(&mut m, frame, "MATI", 10, 0.5, 40.0);
        point_load(&mut m, frame, "HIDUP", 10, 0.5, 25.0);
    }
    for frame in ["13", "14", "15"] {
        gravity_udl(&mut m, frame, "MATI", 0.0, 1.0, 15.0);
        gravity_udl(&mut m, frame, "HIDUP", 0.0, 1.0, 6.0);
    }
    for (joint, dead, live) in [("4", 30.0, 20.0), ("8", 50.0, 30.0), ("12", 40.0, 15.0)] {
        joint_force(&mut m, joint, "MATI", [0.0, 0.0, -dead, 0.0, 0.0, 0.0]);
        joint_force(&mut m, joint, "HIDUP", [0.0, 0.0, -live, 0.0, 0.0, 0.0]);
    }
    for (joint, quake) in [("2", 60.0), ("3", 70.0), ("4", 50.0)] {
        joint_force(&mut m, joint, "GEMPA", [quake, 0.0, 0.0, 0.0, 0.0, 0.0]);
    }
    combo(&mut m, "KOMBINASI 2", &[("MATI", 1.2), ("HIDUP", 1.0), ("GEMPA", 1.0)]);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["KOMBINASI 2"];

    // Statics of the whole frame: dead 690 kN, live 320 kN, quake 180 kN.
    let vertical: f64 = r.joint_reactions.values().map(|f| f[2]).sum();
    let horizontal: f64 = r.joint_reactions.values().map(|f| f[0]).sum();
    close("total vertical reaction", vertical, 1.2 * 690.0 + 320.0, 1e-8);
    close("total horizontal reaction", horizontal, -180.0, 1e-8);

    // The document's design summary for frame 4 (the lowest column on line
    // B) under KOMBINASI 2 shows about 582.0 kN axial and about -240.2 kN-m;
    // both are read off a screenshot.
    let column = &r.frame_forces["4"];
    close("column 4 axial force", column[0].p, -582.0, 0.005 * 582.0);
    close("column 4 moment at the top", column.last().unwrap().m3, -240.2, 0.2);
}

// ---------------------------------------------------------------------------
// Program SAP2000 v20 ebook, chapter 3: simply supported concrete beam
// 150 x 300 mm, 4 m, fc 25 MPa, fy 240 MPa, live load 10 kN/m, kN and m.
// ---------------------------------------------------------------------------

#[test]
fn v20_ebook_concrete_beam_design_matches_the_printed_steel() {
    let mut m = base(Units::KnMC, true);
    let (fc, fy) = (25.0e3, 240.0e3);
    material(&mut m, "4000Psi", "beton", |c| {
        c.e = 23500.0e3;
        c.u = 0.2;
        c.g = c.e / 2.4;
        c.unit_weight = 24.0;
        c.fc = fc;
    });
    material(&mut m, "A992Fy50", "Tulangan", |s| {
        s.e = 200000.0e3;
        s.fy = fy;
        s.fu = 370.0e3;
    });
    let mut section = make_rectangular("Balok", "beton", 0.30, 0.15);
    section.concrete_rebar = beam_rebar("Tulangan", 0.055);
    m.frame_sections.insert("Balok".into(), section);
    beam_line(&mut m, &[0.0, 4.0], "Balok", 81);
    m.points["J1"].restraint = PIN;
    m.points["J2"].restraint = ROLLER;
    pattern(&mut m, "DEAD", 1.0);
    pattern(&mut m, "LIVE", 0.0);
    gravity_udl(&mut m, "F1", "LIVE", 0.0, 1.0, 10.0);
    combo(&mut m, "1.2D + 1.6L", &[("DEAD", 1.2), ("LIVE", 1.6)]);
    analyze_model(&mut m).unwrap();
    // wu = 1.2 x 0.15 x 0.30 x 24 + 1.6 x 10 = 17.296 kN/m; Mu = wu L^2 / 8.
    let wu = 1.2 * 0.15 * 0.30 * 24.0 + 16.0;
    close("maximum moment", max_abs_m3(&m, "1.2D + 1.6L"), wu * 16.0 / 8.0, 1e-9);

    try_run_concrete_design(&mut m).unwrap();
    let design = &m.concrete_design_results["F1"];
    let (b, d) = (0.15, 0.30 - 0.055);
    // Printed: 737 mm2 at mid-span (the document's hand value is 737.168).
    close("bottom steel against the closed form", design.as_bot, aci_flexural_steel(wu * 2.0, fc, fy, b, d), 1e-9);
    close("bottom steel against the document [mm2]", design.as_bot * 1e6, 737.0, 0.5);
    // Printed: Av/s = 0.265 mm2/mm at the supports (hand value 0.264), from
    // Vs = (Vu - phi Vc) / phi with Vc = sqrt(fc) b d / 6 in MPa and mm.
    let vu = wu * 2.0;
    let phi_vc = 0.75 * 25.0f64.sqrt() / 6.0 * 1.0e3 * b * d;
    let av_over_s = (vu - phi_vc) / 0.75 / (fy * d);
    close("stirrups against the closed form", design.av_over_s, av_over_s, 2e-6);
    close("stirrups against the document [mm2/mm]", design.av_over_s * 1e3, 0.265, 1e-3);
}

// ---------------------------------------------------------------------------
// Program SAP2000 v20 ebook, chapter 4: simply supported steel beam W30X99,
// 10 m, Fy 240 MPa, live load 10 kN/m, kN and m.
// ---------------------------------------------------------------------------

#[test]
fn v20_ebook_steel_beam_matches_the_printed_ratio_when_braced() {
    let mut m = base(Units::KnMC, true);
    material(&mut m, "A992Fy50", "Baja", |s| {
        s.e = 200000.0e3;
        s.u = 0.3;
        s.g = 76923.08e3;
        s.unit_weight = 78.5;
        s.fy = 240.0e3;
        s.fu = 370.0e3;
    });
    // W30X99 with the AISC table properties, as the document imports it from
    // the section database (no section database ships with this project).
    let inch = 0.0254;
    let mut section = make_i_section(
        "W30X99",
        "Baja",
        29.7 * inch,
        10.5 * inch,
        0.670 * inch,
        0.520 * inch,
        10.5 * inch,
        0.670 * inch,
    );
    section.area = 29.1 * inch.powi(2);
    section.i33 = 3990.0 * inch.powi(4);
    section.s33 = 269.0 * inch.powi(3);
    section.z33 = 312.0 * inch.powi(3);
    section.r33 = 11.7 * inch;
    section.i22 = 128.0 * inch.powi(4);
    section.s22 = 24.5 * inch.powi(3);
    section.z22 = 38.6 * inch.powi(3);
    section.r22 = 2.10 * inch;
    section.torsional_j = 3.77 * inch.powi(4);
    let plastic_capacity = 0.9 * section.z33 * 240.0e3;
    m.frame_sections.insert("W30X99".into(), section);
    beam_line(&mut m, &[0.0, 10.0], "W30X99", 41);
    m.points["J1"].restraint = PIN;
    m.points["J2"].restraint = ROLLER;
    pattern(&mut m, "DEAD", 1.0);
    pattern(&mut m, "LIVE", 0.0);
    gravity_udl(&mut m, "F1", "LIVE", 0.0, 1.0, 10.0);
    combo(&mut m, "COMB1", &[("DEAD", 1.2), ("LIVE", 1.6)]);
    analyze_model(&mut m).unwrap();
    let mu = max_abs_m3(&m, "COMB1");

    // The document prints a P-M ratio of 0.201, which is Mu / (0.9 Z Fy):
    // the full plastic moment, with no lateral-torsional buckling reduction.
    // That is the result for a beam whose compression flange is braced,
    // which is set here as the member's stored steel design overwrite (the
    // setting the GUI and the API write).
    m.frames["F1"].steel_overwrite = Some(SteelDesignOverwrite {
        l_ratio_ltb: Some(0.01),
        ..Default::default()
    });
    try_run_steel_design(&mut m).unwrap();
    let ratio = m.steel_design_results["F1"].total_ratio;
    close("braced ratio against the closed form", ratio, mu / plastic_capacity, 1e-9);
    close("braced ratio against the document", ratio, 0.201, 5e-4);

    // With no overwrite the beam is unbraced over its 10 m span, beyond Lr,
    // and elastic lateral-torsional buckling governs, as the vendor's steel
    // design manual specifies for the default (the unsupported length; see
    // the AISC 360-05 Example 001 test below, which this project matches).
    // Hand check (AISC 360 F2-3 and F2-4, Cb = 1.136, rts 66.5 mm, J 1.569e6
    // mm4, Sx 4.408e6 mm3, ho 737 mm): Fcr = 135.5 MPa, phi Mn = 537.6 kN-m,
    // ratio 0.413. The document's 0.201 is not what SAP2000's documented
    // default gives for this beam.
    m.frames["F1"].steel_overwrite = None;
    try_run_steel_design(&mut m).unwrap();
    let unbraced = &m.steel_design_results["F1"];
    close("unbraced ratio against the hand check", unbraced.total_ratio, 0.413, 0.02 * 0.413);
    assert!(unbraced.phi_mn33 < 0.5 * plastic_capacity);
}

// ---------------------------------------------------------------------------
// Program SAP2000 v20 ebook, chapter 5: square slab 4 x 4 m, 12 cm thin
// shell, all edges simply supported, kN and m.
// ---------------------------------------------------------------------------

/// Centre moment per unit width and total vertical reaction for an `n x n` mesh.
fn ebook_slab(n: usize) -> (f64, f64) {
    let mut m = base(Units::KnMC, false);
    material(&mut m, "4000Psi", "beton", |c| {
        c.e = 25742960.0;
        c.u = 0.2;
        c.g = c.e / 2.4;
        c.unit_weight = 23.536;
        c.fc = 30000.0;
    });
    let mut section = m.area_sections.values().next().unwrap().clone();
    section.name = "PLAT".into();
    section.material = "beton".into();
    section.prop_type = AreaPropType::Shell;
    section.shell_type = ShellType::ShellThin;
    section.thickness_membrane = 0.12;
    section.thickness_bending = 0.12;
    m.area_sections.insert("PLAT".into(), section);
    let h = 4.0 / n as f64;
    for j in 0..=n {
        for i in 0..=n {
            let name = format!("P{i}_{j}");
            m.add_point(name.clone(), i as f64 * h, j as f64 * h, 0.0);
            if i == 0 || j == 0 || i == n || j == n {
                m.points[name.as_str()].restraint = PIN;
            }
        }
    }
    pattern(&mut m, "DEAD", 1.0);
    pattern(&mut m, "LIVE", 0.0);
    for j in 0..n {
        for i in 0..n {
            let name = format!("A{i}_{j}");
            let corners = vec![
                format!("P{i}_{j}"),
                format!("P{}_{j}", i + 1),
                format!("P{}_{}", i + 1, j + 1),
                format!("P{i}_{}", j + 1),
            ];
            let mut area = AreaObj::new(name.clone(), corners, "PLAT");
            for (pat, value) in [("DEAD", 1.0), ("LIVE", 5.0)] {
                area.loads_uniform.push(AreaLoadUniform {
                    load_pat: pat.into(),
                    csys: "GLOBAL".into(),
                    dir: 10,
                    val: value,
                });
            }
            m.areas.insert(name, area);
        }
    }
    combo(&mut m, "1.2D+1.6L", &[("DEAD", 1.2), ("LIVE", 1.6)]);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["1.2D+1.6L"];
    let c = n / 2;
    let centre: f64 = [(c - 1, c - 1), (c, c - 1), (c - 1, c), (c, c)]
        .iter()
        .map(|(i, j)| {
            let a = &r.area_results[&format!("A{i}_{j}")];
            0.5 * (a.m11.abs() + a.m22.abs())
        })
        .sum::<f64>()
        / 4.0;
    (centre, r.joint_reactions.values().map(|f| f[2]).sum())
}

#[test]
fn v20_ebook_slab_converges_to_the_plate_solution() {
    // qu = 1.2 (1 + 0.12 x 23.536) + 1.6 x 5 = 12.589 kN/m2.
    let qu = 1.2 * (1.0 + 0.12 * 23.536) + 1.6 * 5.0;
    // Simply supported square plate under uniform load (Navier series,
    // Timoshenko and Woinowsky-Krieger): M = 0.0368 (1 + nu) q a^2 at the
    // centre, 8.895 kN-m/m here. The document's hand table (PBI 1971) gives
    // 8.91, and its SAP2000 run on the coarse template mesh shows 9.20.
    let plate = 0.0368 * 1.2 * qu * 16.0;
    let (coarse, total) = ebook_slab(6);
    close("total reaction", total, qu * 16.0, 1e-6);
    let (fine, total) = ebook_slab(12);
    close("total reaction", total, qu * 16.0, 1e-6);
    // The centre moment rises toward the plate solution as the mesh is refined.
    assert!(coarse < fine && fine < plate, "{coarse} {fine} {plate}");
    close("12 x 12 mesh against the plate solution", fine, plate, 0.02 * plate);
    close("12 x 12 mesh against the document's hand table", fine, 8.91, 0.02 * 8.91);
    // Second-order convergence: one Richardson step lands on the solution.
    let extrapolated = fine + (fine - coarse) / 3.0;
    close("extrapolated against the plate solution", extrapolated, plate, 0.005 * plate);
}

// ---------------------------------------------------------------------------
// Program SAP2000 v20 ebook, chapter 6: 5-storey concrete building, 3 x 3
// bays of 5 m, 4 m storeys, slabs as thin shells, one rigid diaphragm per
// level, mass from dead + superimposed dead + 0.3 live, kN and m.
// ---------------------------------------------------------------------------

/// The building with its slabs meshed `n x n` per bay and the frames split at
/// the mesh joints. Returns the model and its seismic weight by hand.
fn ebook_building(n: usize) -> (SapModel, f64) {
    let g = 9.80665;
    let mut m = base(Units::KnMC, false);
    material(&mut m, "4000Psi", "30 Mpa", |c| {
        c.e = 25742960.2;
        c.u = 0.2;
        c.g = c.e / 2.4;
        c.unit_weight = 24.0;
        c.unit_mass = 24.0 / g;
        c.fc = 30000.0;
    });
    for (name, depth, width) in [
        ("B1 20/45", 0.45, 0.20),
        ("B2 20/35", 0.35, 0.20),
        ("K1 45/45", 0.45, 0.45),
        ("K2 40/40", 0.40, 0.40),
        ("K3 35/35", 0.35, 0.35),
    ] {
        m.frame_sections
            .insert(name.into(), make_rectangular(name, "30 Mpa", depth, width));
    }
    for (name, thickness) in [("Pelat Lantai", 0.12), ("Pelat Atap", 0.10)] {
        let mut section = m.area_sections.values().next().unwrap().clone();
        section.name = name.into();
        section.material = "30 Mpa".into();
        section.prop_type = AreaPropType::Shell;
        section.shell_type = ShellType::ShellThin;
        section.thickness_membrane = thickness;
        section.thickness_bending = thickness;
        m.area_sections.insert(name.into(), section);
    }
    let per = 3 * n;
    let h = 15.0 / per as f64;
    let joint = |i: usize, j: usize, k: usize| format!("J{i}_{j}_{k}");
    for j in (0..=per).step_by(n) {
        for i in (0..=per).step_by(n) {
            let name = joint(i, j, 0);
            m.add_point(name.clone(), i as f64 * h, j as f64 * h, 0.0);
            m.points[name.as_str()].restraint = [true; 6];
        }
    }
    let mut weight = 0.0;
    for k in 1..=5usize {
        let roof = k == 5;
        let diaphragm = format!("DIAPH{k}");
        m.constraints.insert(
            diaphragm.clone(),
            ConstraintDef {
                name: diaphragm.clone(),
                constraint_type: ConstraintType::Diaphragm,
                axis: ConstraintAxis::Z,
                dofs: [true; 6],
            },
        );
        for j in 0..=per {
            for i in 0..=per {
                let name = joint(i, j, k);
                m.add_point(name.clone(), i as f64 * h, j as f64 * h, 4.0 * k as f64);
                m.points[name.as_str()].constraints.push(diaphragm.clone());
            }
        }
        // Slabs: superimposed dead load and 0.3 of the live load as added mass.
        let (superimposed, live, thickness, section) = if roof {
            (0.73, 1.0, 0.10, "Pelat Atap")
        } else {
            (1.49, 2.5, 0.12, "Pelat Lantai")
        };
        for j in 0..per {
            for i in 0..per {
                let corners = vec![joint(i, j, k), joint(i + 1, j, k), joint(i + 1, j + 1, k), joint(i, j + 1, k)];
                let mut area = AreaObj::new(format!("S{i}_{j}_{k}"), corners, section);
                area.mass_per_area = (superimposed + 0.3 * live) / g;
                m.areas.insert(area.name.clone(), area);
            }
        }
        weight += 225.0 * (superimposed + 0.3 * live + thickness * 24.0);
        // Beams along the grid lines; 8.875 kN/m of wall on the perimeter
        // beams of every floor below the roof, as joint mass.
        let (beam, beam_weight) = if roof { ("B2 20/35", 0.2 * 0.35 * 24.0) } else { ("B1 20/45", 0.2 * 0.45 * 24.0) };
        let mut segments: Vec<(String, String, bool)> = Vec::new();
        for line in (0..=per).step_by(n) {
            for s in 0..per {
                let perimeter = line == 0 || line == per;
                segments.push((joint(s, line, k), joint(s + 1, line, k), perimeter));
                segments.push((joint(line, s, k), joint(line, s + 1, k), perimeter));
            }
        }
        for (a, b, perimeter) in segments {
            m.add_frame(format!("B_{a}_{b}"), a.clone(), b.clone(), beam);
            if perimeter && !roof {
                for end in [a, b] {
                    for d in 0..3 {
                        m.points[end.as_str()].mass[d] += 8.875 * h / 2.0 / g;
                    }
                }
            }
        }
        weight += 120.0 * beam_weight + if roof { 0.0 } else { 60.0 * 8.875 };
        // Columns: K3 at the corners, K2 on the edges, K1 inside. Half of the
        // lowest storey's columns goes to the supports, not to a level.
        for j in (0..=per).step_by(n) {
            for i in (0..=per).step_by(n) {
                let edges = usize::from(i == 0 || i == per) + usize::from(j == 0 || j == per);
                let (section, side) = match edges {
                    2 => ("K3 35/35", 0.35),
                    1 => ("K2 40/40", 0.40),
                    _ => ("K1 45/45", 0.45),
                };
                m.add_frame(format!("C{i}_{j}_{k}"), joint(i, j, k - 1), joint(i, j, k), section);
                weight += side * side * 24.0 * 4.0 * if k == 1 { 0.5 } else { 1.0 };
            }
        }
    }
    m.load_cases
        .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 12));
    (m, weight)
}

#[test]
fn v20_ebook_building_weight_matches_the_printed_base_shear() {
    // The document prints a static base shear of 1182.785 kN from SAP2000's
    // IBC 2009 pattern with SDS = 0.9594, R = 8 and I = 1, so its seismic
    // weight is 1182.785 x 8 / 0.9594 = 9862.7 kN. The load take-down of this
    // model (with 1 kN/m2 of roof live load, which the document does not
    // show) gives the same weight. Automatic seismic loads themselves are
    // not implemented.
    let (model, weight) = ebook_building(1);
    close("seismic weight", weight, 1182.785 * 8.0 / 0.9594, 0.1);
    assert_eq!((model.points.len(), model.frames.len(), model.areas.len()), (96, 200, 45));
}

/// Slow: about 12 s in a release build, minutes in a debug build. The
/// document's own mesh (6 x 6 per bay, 1821 joints) is beyond the dense solver.
#[test]
#[ignore = "slow dense modal analysis of a 261-joint building"]
fn v20_ebook_building_periods_approach_the_printed_values_on_a_coarser_mesh() {
    // Printed by SAP2000 on a 6 x 6 mesh: T1 = T2 = 1.165482 s (the two
    // translations), T3 = 1.062094 s (torsion), and 99.4 % of the mass in 12
    // modes. A 2 x 2 mesh is stiffer: measured here, T1 is about 7 % short
    // (1 x 1 is 35 % short and 3 x 3 is 3.8 % short).
    let (mut model, _) = ebook_building(2);
    analyze_model(&mut model).unwrap();
    let modes = &model.case_results["MODAL"].modes;
    assert_eq!(modes.len(), 12);
    let (t1, t2, t3) = (modes[0].period_sec, modes[1].period_sec, modes[2].period_sec);
    close("the two translational periods are equal", t1, t2, 1e-3 * t1);
    assert!(t1 < 1.165482 && t1 > 0.90 * 1.165482, "T1 = {t1}");
    assert!(t3 < t1 && t3 > 0.90 * 1.062094, "T3 = {t3}");
    for direction in 0..2 {
        let mass: f64 = modes.iter().map(|mode| mode.modal_mass_ratios[direction]).sum();
        close("modal mass in 12 modes", mass, 0.99405, 0.005);
    }
}

// ---------------------------------------------------------------------------
// CSI Software Verification, AISC 360-05 Example 001: W18X50 beam, 35 ft span,
// 0.45 klf dead and 0.75 klf live, flexural strength for three unbraced
// lengths. Kip and inch.
// ---------------------------------------------------------------------------

fn aisc_360_05_example_001() -> SapModel {
    let mut m = base(Units::KipInF, true);
    material(&mut m, "A992Fy50", "A572Gr50", |s| {
        s.e = 29000.0;
        s.fy = 50.0;
        s.fu = 65.0;
    });
    // Section properties as the example lists them.
    let mut section = make_i_section("W18X50", "A572Gr50", 18.0, 7.5, 0.57, 0.355, 7.5, 0.57);
    section.area = 14.7;
    section.s33 = 88.9;
    section.z33 = 101.0;
    section.i33 = 800.0;
    section.i22 = 40.1;
    section.r22 = 1.652;
    section.torsional_j = 1.240;
    m.frame_sections.insert("W18X50".into(), section);
    beam_line(&mut m, &[0.0, 420.0], "W18X50", 41);
    m.points["J1"].restraint = PIN;
    m.points["J2"].restraint = ROLLER;
    pattern(&mut m, "D", 0.0);
    pattern(&mut m, "L", 0.0);
    gravity_udl(&mut m, "F1", "D", 0.0, 1.0, 0.45 / 12.0);
    gravity_udl(&mut m, "F1", "L", 0.0, 1.0, 0.75 / 12.0);
    combo(&mut m, "1.2D+1.6L", &[("D", 1.2), ("L", 1.6)]);
    analyze_model(&mut m).unwrap();
    m
}

#[test]
fn vendor_aisc_360_05_example_001_flexural_strengths_match() {
    let mut m = aisc_360_05_example_001();
    // Mu = 1.74 x 35^2 / 8 = 266.4375 kip-ft.
    close("factored moment [kip-ft]", max_abs_m3(&m, "1.2D+1.6L") / 12.0, 266.4375, 1e-9);
    // Published SAP2000 values: unbraced length [ft], Cb, phi Mn [kip-ft].
    for (unbraced, cb, strength) in [(5.0, 1.004, 378.750), (11.667, 1.015, 307.124), (35.0, 1.138, 94.377)] {
        let overwrite = SteelMemberOverwrite {
            l_ratio_ltb: Some(unbraced / 35.0),
            cb: Some(cb),
            ..Default::default()
        };
        try_run_steel_design_with_overwrites(&mut m, &HashMap::from([("F1".to_string(), overwrite)])).unwrap();
        let got = m.steel_design_results["F1"].phi_mn33 / 12.0;
        close(&format!("phi Mn at Lb = {unbraced} ft"), got, strength, 0.002 * strength);
    }
    // With Cb left to the program it is computed within each unbraced
    // segment and the smallest governs. The example's independent values
    // (AISC formula) are Cb = 1.002, 1.014 and 1.136, giving 378.750,
    // 306.657 and 94.218 kip-ft.
    for (unbraced, strength) in [(5.0, 378.750), (11.667, 306.657), (35.0, 94.218)] {
        m.frames["F1"].steel_overwrite = Some(SteelDesignOverwrite {
            l_ratio_ltb: Some(unbraced / 35.0),
            ..Default::default()
        });
        try_run_steel_design(&mut m).unwrap();
        let got = m.steel_design_results["F1"].phi_mn33 / 12.0;
        close(&format!("phi Mn at Lb = {unbraced} ft, automatic Cb"), got, strength, 0.002 * strength);
    }
    // With no overwrite at all the beam is unbraced over its whole span.
    m.frames["F1"].steel_overwrite = None;
    try_run_steel_design(&mut m).unwrap();
    close("phi Mn with program defaults", m.steel_design_results["F1"].phi_mn33 / 12.0, 94.218, 0.002 * 94.218);

    // An unbraced length set longer than the member takes Cb = 1.0, as the
    // vendor's steel design manual states for SAP2000.
    let strength = |m: &mut SapModel, overwrite: SteelDesignOverwrite| {
        m.frames["F1"].steel_overwrite = Some(overwrite);
        try_run_steel_design(m).unwrap();
        m.steel_design_results["F1"].phi_mn33
    };
    let longer = SteelDesignOverwrite { l_ratio_ltb: Some(1.5), ..Default::default() };
    let automatic = strength(&mut m, longer.clone());
    let unity = strength(&mut m, SteelDesignOverwrite { cb: Some(1.0), ..longer.clone() });
    let gradient = strength(&mut m, SteelDesignOverwrite { cb: Some(1.136), ..longer });
    close("Cb defaults to 1.0 beyond the member length", automatic, unity, 1e-9);
    assert!(gradient > 1.1 * unity);

    // A stored overwrite that is not a positive number stops the design
    // instead of reaching the capacity formulas.
    for bad in [
        SteelDesignOverwrite { l_ratio_ltb: Some(-1.0), ..Default::default() },
        SteelDesignOverwrite { k_minor: Some(0.0), ..Default::default() },
        SteelDesignOverwrite { cb: Some(f64::NAN), ..Default::default() },
    ] {
        m.frames["F1"].steel_overwrite = Some(bad);
        let error = try_run_steel_design(&mut m).unwrap_err().to_string();
        assert!(error.contains("member 'F1'") && error.contains("finite and positive"), "{error}");
        assert!(m.steel_design_results.is_empty());
    }
    // A bracing spacing far below the member length is continuous bracing.
    let continuous = strength(&mut m, SteelDesignOverwrite { l_ratio_ltb: Some(1e-9), ..Default::default() });
    close("continuously braced strength [kip-ft]", continuous / 12.0, 378.750, 1e-6 * 378.750);
}

/// Stored effective-length overwrites change the compression strength by the
/// Euler closed form. A slender pinned column, W18X50 about its minor axis:
/// in the elastic range phi Pn = 0.9 x 0.877 x pi^2 E A / (K L / r)^2.
#[test]
fn stored_steel_length_and_k_overwrites_follow_the_euler_closed_form() {
    let mut m = aisc_360_05_example_001();
    // Add an axial load so the member is a beam-column with a compression check.
    pattern(&mut m, "P", 0.0);
    joint_force(&mut m, "J2", "P", [-5.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    analyze_model(&mut m).unwrap();
    let (e, area, r22, length) = (29000.0, 14.7, 1.652, 420.0);
    let euler = |k: f64, ratio: f64| {
        let slenderness: f64 = k * ratio * length / r22;
        // Elastic buckling governs beyond 4.71 sqrt(E / Fy) = 113.4.
        assert!(slenderness > 113.5, "{slenderness}");
        0.9 * 0.877 * std::f64::consts::PI.powi(2) * e * area / slenderness.powi(2)
    };
    for (overwrite, expected) in [
        (SteelDesignOverwrite::default(), euler(1.0, 1.0)),
        (SteelDesignOverwrite { k_minor: Some(2.0), ..Default::default() }, euler(2.0, 1.0)),
        (SteelDesignOverwrite { l_ratio_minor: Some(0.5), ..Default::default() }, euler(1.0, 0.5)),
        (SteelDesignOverwrite { k_minor: Some(1.2), l_ratio_minor: Some(0.75), ..Default::default() }, euler(1.2, 0.75)),
    ] {
        m.frames["F1"].steel_overwrite = Some(overwrite.clone());
        try_run_steel_design(&mut m).unwrap();
        let got = m.steel_design_results["F1"].phi_pnc;
        close(&format!("phi Pn for {overwrite:?}"), got, expected, 1e-6 * expected);
    }
    // Eurocode 3 reads the same stored lengths: halving the minor length
    // raises the buckling resistance.
    m.steel_design_code = "Eurocode 3-2005".into();
    m.frames["F1"].steel_overwrite = None;
    try_run_steel_design(&mut m).unwrap();
    let full = m.steel_design_results["F1"].phi_pnc;
    m.frames["F1"].steel_overwrite = Some(SteelDesignOverwrite { l_ratio_minor: Some(0.5), ..Default::default() });
    try_run_steel_design(&mut m).unwrap();
    assert!(m.steel_design_results["F1"].phi_pnc > 2.0 * full);
}

// ---------------------------------------------------------------------------
// CSI Software Verification, ACI 318-14 Example 001: simply supported beam
// 10 x 16 in (d = 13.5 in), 10 ft span, ultimate load 9.736 kip/ft, fc 4 ksi,
// fy 60 ksi. Kip and inch.
// ---------------------------------------------------------------------------

#[test]
fn vendor_aci_318_14_example_001_flexural_steel_matches_for_every_framing_type() {
    for framing in [ConcreteFramingType::SwaySpecial, ConcreteFramingType::SwayOrdinary] {
        let mut m = base(Units::KipInF, true);
        material(&mut m, "4000Psi", "Conc", |c| {
            c.e = 3600.0;
            c.u = 0.2;
            c.g = 1500.0;
            c.fc = 4.0;
        });
        material(&mut m, "A992Fy50", "Rebar", |s| s.fy = 60.0);
        let mut section = make_rectangular("Conc Bm 10x16", "Conc", 16.0, 10.0);
        section.concrete_rebar = beam_rebar("Rebar", 2.5);
        m.frame_sections.insert("Conc Bm 10x16".into(), section);
        beam_line(&mut m, &[0.0, 120.0], "Conc Bm 10x16", 41);
        m.points["J1"].restraint = PIN;
        m.points["J2"].restraint = ROLLER;
        m.frames["F1"].concrete_framing = framing;
        pattern(&mut m, "Ultimate", 0.0);
        gravity_udl(&mut m, "F1", "Ultimate", 0.0, 1.0, 9.736 / 12.0);
        analyze_model(&mut m).unwrap();
        // Published: Mu = 1460.4 kip-in and As = 2.37 in2. The beam has no end
        // moments, so the seismic framing types change nothing.
        close("design moment", max_abs_m3(&m, "Ultimate"), 1460.4, 0.05);
        try_run_concrete_design(&mut m).unwrap();
        let design = &m.concrete_design_results["F1"];
        close("tension steel [in2]", design.as_bot, 2.37, 0.005);
        close("tension steel against the closed form", design.as_bot, aci_flexural_steel(1460.4, 4.0, 60.0, 10.0, 13.5), 1e-6);
        assert_eq!(design.as_top, 0.0);
        // The published Av/s = 0.041 in2/in is for the shear at d from the
        // support, 37.73 kip; this project reports the largest value over the
        // stations, which is at the support: Vu = 48.68 kip,
        // Av/s = (Vu - phi Vc) / (phi fy d) with phi Vc = 0.75 x 2 sqrt(4000) psi x b d.
        let phi_vc = 0.75 * 2.0 * 4000.0f64.sqrt() / 1000.0 * 10.0 * 13.5;
        close("stirrups at the support", design.av_over_s, (48.68 - phi_vc) / (0.75 * 60.0 * 13.5), 1e-4);
        close("stirrups at d from the support, by the same formula", (37.73 - phi_vc) / (0.75 * 60.0 * 13.5), 0.041, 5e-4);
    }
}
