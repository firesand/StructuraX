//! Wave 5 P13: a member without end offsets is bitwise unchanged by the repairs
//! of D46, D47 and D48.
//!
//! The expected values are fingerprints (FNV-1a over the bit patterns) of the results of the code before the
//! repairs (wave 5 baseline `W5_S2`, recorded with the `#[ignore]`d test
//! `print_expected` below), for a prismatic and a non-prismatic member without
//! offsets, with span loads, self weight, a temperature gradient and releases,
//! horizontal and inclined, including the station forces and station deflections.

use sap_model::*;
use sap_solver::analyze_model;

fn section(name: &str, area: f64, inertia: f64) -> FrameSection {
    FrameSection {
        name: name.into(),
        material: "MAT".into(),
        area,
        i22: inertia,
        i33: inertia * 0.8,
        torsional_j: inertia * 0.5,
        as2: area * 0.4,
        as3: area * 0.4,
        ..FrameSection::default()
    }
}

fn model(section_name: &str, end: [f64; 3], released: bool, tip_free: bool) -> SapModel {
    let mut model = SapModel::default();
    model.load_cases.clear();
    model.load_patterns.clear();
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.e = 29_000.0;
    mat.u = 0.3;
    mat.g = 11_000.0;
    mat.a = 1.0e-5;
    mat.unit_mass = 0.0;
    mat.unit_weight = 0.0003;
    model.materials.insert("MAT".into(), mat);
    model.frame_sections.insert("S".into(), section("S", 20.0, 300.0));
    model.frame_sections.insert("S1".into(), section("S1", 30.0, 500.0));
    model.frame_sections.insert("S2".into(), section("S2", 12.0, 150.0));
    let mut variable = section("VAR", 30.0, 500.0);
    variable.prop_type = FramePropType::Variable;
    variable.non_prismatic = vec![
        NonPrismaticSegment {
            start_sec: "S1".into(),
            end_sec: "S2".into(),
            length: 1.0,
            is_variable_length: true,
            ei33_variation: 2,
            ei22_variation: 1,
        },
        NonPrismaticSegment {
            start_sec: "S2".into(),
            end_sec: "S2".into(),
            length: 40.0,
            is_variable_length: false,
            ei33_variation: 1,
            ei22_variation: 1,
        },
    ];
    model.frame_sections.insert("VAR".into(), variable);
    model.add_point("1", 0.0, 0.0, 0.0);
    model.add_point("2", end[0], end[1], end[2]);
    model.points.get_mut("1").unwrap().restraint = [true; 6];
    model.points.get_mut("2").unwrap().restraint = [!tip_free; 6];
    model.add_frame("F", "1", "2", section_name);
    let frame = model.frames.get_mut("F").unwrap();
    frame.min_stations = 7;
    if released {
        // (A released end needs a fixed joint 2: with it free the member
        // would be a mechanism.)
        assert!(!tip_free);
        frame.release_i[5] = true;
        frame.release_j[4] = true;
    }
    frame.loads_dist.push(FrameLoadDistributed {
        load_pat: "W".into(),
        my_type: 1,
        dir: 10,
        csys: "GLOBAL".into(),
        rel_dist1: 0.1,
        rel_dist2: 0.8,
        val1: 0.5,
        val2: 1.5,
    });
    frame.loads_dist.push(FrameLoadDistributed {
        load_pat: "W".into(),
        my_type: 1,
        dir: 2,
        csys: "Local".into(),
        rel_dist1: 0.0,
        rel_dist2: 1.0,
        val1: 0.2,
        val2: 0.2,
    });
    frame.loads_point.push(FrameLoadPoint {
        load_pat: "W".into(),
        my_type: 1,
        dir: 3,
        csys: "Local".into(),
        rel_dist: 0.35,
        val: 4.0,
    });
    frame.loads_point.push(FrameLoadPoint {
        load_pat: "W".into(),
        my_type: 2,
        dir: 3,
        csys: "Local".into(),
        rel_dist: 0.6,
        val: 50.0,
    });
    frame.loads_point.push(FrameLoadPoint {
        load_pat: "W".into(),
        my_type: 1,
        dir: 1,
        csys: "Local".into(),
        rel_dist: 0.45,
        val: 7.0,
    });
    for my_type in [1, 2, 3] {
        frame.loads_temp.push(FrameLoadTemperature {
            load_pat: "T".into(),
            my_type,
            val: if my_type == 1 { 40.0 } else { 0.3 },
            pattern_name: None,
        });
    }
    for (name, self_weight) in [("W", 0.0), ("SW", 1.0), ("T", 0.0)] {
        model.load_patterns.insert(
            name.into(),
            LoadPattern {
                name: name.into(),
                pat_type: LoadPatternType::Other,
                self_wt_multiplier: self_weight,
            },
        );
        model
            .load_cases
            .insert(name.into(), LoadCase::linear_static(name, name, 1.0));
    }
    model
}

/// FNV-1a over the bit patterns of the values.
fn fingerprint(values: &[f64]) -> u64 {
    let mut hash = 0xcbf2_9ce4_8422_2325u64;
    for value in values {
        for byte in value.to_bits().to_le_bytes() {
            hash ^= u64::from(byte);
            hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    hash
}

/// One fingerprint per member and load case: the joint reactions and
/// displacements of both joints and, for every station, its position, forces and
/// local deflection.
fn results() -> Vec<(String, u64)> {
    let mut out = Vec::new();
    for section_name in ["S", "VAR"] {
        for (shape, end) in [("horizontal", [144.0, 0.0, 0.0]), ("inclined", [86.4, 20.0, 115.2])] {
            for (support, released, tip_free) in [
                ("fixed", false, false),
                ("fixed-released", true, false),
                ("free", false, true),
            ] {
                let mut m = model(section_name, end, released, tip_free);
                analyze_model(&mut m).unwrap();
                for case in ["W", "SW", "T"] {
                    let r = &m.case_results[case];
                    let mut values = Vec::new();
                    for joint in ["1", "2"] {
                        values.extend(r.joint_reactions.get(joint).copied().unwrap_or([0.0; 6]));
                        values.extend(r.joint_displacements[joint]);
                    }
                    for s in &r.frame_forces["F"] {
                        values.extend([s.station, s.rel_dist, s.p, s.v2, s.v3, s.t, s.m2, s.m3]);
                        values.extend(s.local_disp);
                    }
                    out.push((
                        format!("{section_name} {shape} {support} {case}"),
                        fingerprint(&values),
                    ));
                }
            }
        }
    }
    out
}

#[test]
#[ignore]
fn print_expected() {
    let values = results();
    println!("static EXPECTED: [(&str, u64); {}] = [", values.len());
    for (label, hash) in values {
        println!("    (\"{label}\", 0x{hash:016x}),");
    }
    println!("];");
}

#[test]
fn members_without_offsets_are_bitwise_unchanged() {
    let values = results();
    assert_eq!(values.len(), EXPECTED.len());
    for ((label, hash), (expected_label, expected)) in values.iter().zip(EXPECTED.iter()) {
        assert_eq!(label, expected_label);
        assert_eq!(hash, expected, "{label}: the bit patterns of the results changed");
    }
}

static EXPECTED: [(&str, u64); 36] = [
    ("S horizontal fixed W", 0x7d036ba17bfb783f),
    ("S horizontal fixed SW", 0xc4c5c62c6ba7fb3a),
    ("S horizontal fixed T", 0x004dee4cc2fdd7cf),
    ("S horizontal fixed-released W", 0xca8cac8471e08a91),
    ("S horizontal fixed-released SW", 0xbe7bf32e4f988ded),
    ("S horizontal fixed-released T", 0x73783b49e9fb60f4),
    ("S horizontal free W", 0xa72ca10ad8f8cb28),
    ("S horizontal free SW", 0x973861d8f5aa9970),
    ("S horizontal free T", 0xd7932bde7176284b),
    ("S inclined fixed W", 0xb170f3296a65f52a),
    ("S inclined fixed SW", 0xd71ed7bf7b3e9ad7),
    ("S inclined fixed T", 0x6131c22ff2e9357e),
    ("S inclined fixed-released W", 0x543a067bb51043ac),
    ("S inclined fixed-released SW", 0xafe17cb5a83492c1),
    ("S inclined fixed-released T", 0x6975946fb76e29ac),
    ("S inclined free W", 0x2f12d5b79dec1da5),
    ("S inclined free SW", 0xbacf562a7dc40e19),
    ("S inclined free T", 0xf1b7216a1ae6e756),
    ("VAR horizontal fixed W", 0xf54b7442b15a7420),
    ("VAR horizontal fixed SW", 0x011d1674ae83f433),
    ("VAR horizontal fixed T", 0x7bbaa6777266f1b6),
    ("VAR horizontal fixed-released W", 0x51d28f219db462b8),
    ("VAR horizontal fixed-released SW", 0x3a83ef86545dd0de),
    ("VAR horizontal fixed-released T", 0xc0bc311bff6682db),
    ("VAR horizontal free W", 0x593033f903d2d67c),
    ("VAR horizontal free SW", 0x3f6a9e8fa8c2045a),
    ("VAR horizontal free T", 0x89dc9322aef402ec),
    ("VAR inclined fixed W", 0x94cfb43299ea5b8f),
    ("VAR inclined fixed SW", 0xf37a41aa944c787f),
    ("VAR inclined fixed T", 0x2334b2312fbcb6b8),
    ("VAR inclined fixed-released W", 0x33626ca9ace63fe4),
    ("VAR inclined fixed-released SW", 0x48c49465a6142ce1),
    ("VAR inclined fixed-released T", 0xd63b0d0043b852bc),
    ("VAR inclined free W", 0x7a8c8c9c291a861f),
    ("VAR inclined free SW", 0x0dceb3c295933cbc),
    ("VAR inclined free T", 0xf4d668a63f7b5e3a),
];
