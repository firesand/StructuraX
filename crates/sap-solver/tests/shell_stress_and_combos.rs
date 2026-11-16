//! Shell top-face stress sign, combination of derived stress values, combinations
//! with missing results, and response-spectrum direction validation.
//!
//! Reference: CSI Analysis Reference, shell stress Eqns. 2 (p. 212):
//! `s11 = F11/th - 12 M11 x3 / thb^3`, positive moments give positive stress at the
//! bottom face, so at the top face `s11 = F11/th - 6 M11/thb^2`.
use sap_model::*;
use sap_solver::analyze_model;

fn close(what: &str, got: f64, want: f64, tol: f64) {
    assert!(
        (got - want).abs() <= tol * want.abs().max(1.0),
        "{what}: got {got:e}, want {want:e}"
    );
}
use sap_units::Units;

const L: f64 = 4.0;
const B: f64 = 1.0;
const T: f64 = 0.2;
const E: f64 = 2.5e7;

/// Cantilever plate strip (kN, m): 4 x 1 elements, root fixed, nu = 0. Load pattern
/// `D` puts FX = 25 and MY = +5 on each tip joint (N = 50, M0 = 10 about +Y);
/// `L` puts MY = -5 on each tip joint. Cases `D` and `L` are linear static.
fn strip() -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.e = E;
    mat.u = 0.0;
    mat.g = E / 2.0;
    mat.unit_weight = 0.0;
    mat.unit_mass = 0.0;
    m.materials.insert("MAT".into(), mat);
    let mut s = AreaSection::default();
    s.name = "PL".into();
    s.material = "MAT".into();
    s.thickness_membrane = T;
    s.thickness_bending = T;
    m.area_sections.insert("PL".into(), s);
    m.load_cases.clear();
    m.load_patterns.clear();
    for name in ["D", "L"] {
        m.load_patterns.insert(
            name.into(),
            LoadPattern {
                name: name.into(),
                pat_type: LoadPatternType::Dead,
                self_wt_multiplier: 0.0,
            },
        );
        m.load_cases
            .insert(name.into(), LoadCase::linear_static(name, name, 1.0));
    }
    for i in 0..=4 {
        for (j, y) in [0.0, B].iter().enumerate() {
            let name = format!("J{i}{j}");
            m.add_point(name.clone(), i as f64, *y, 0.0);
            if i == 0 {
                m.points[&name].restraint = [true; 6];
            }
            if i == 4 {
                for (pat, f) in [
                    ("D", [25.0, 0., 0., 0., 5.0, 0.]),
                    ("L", [0., 0., 0., 0., -5.0, 0.]),
                ] {
                    m.points[&name].loads_force.push(PointLoadForce {
                        load_pat: pat.into(),
                        csys: "GLOBAL".into(),
                        force: f,
                    });
                }
            }
        }
    }
    for i in 0..4 {
        m.add_area(
            format!("A{i}"),
            vec![
                format!("J{i}0"),
                format!("J{}0", i + 1),
                format!("J{}1", i + 1),
                format!("J{i}1"),
            ],
            "PL",
        );
    }
    m
}

#[test]
fn cantilever_strip_top_face_stress_and_csi_moment_sign() {
    // Hand derivation (kN, m). N = 2 * 25 = 50 over b = 1: F11 = 50. M0 = 2 * 5 = 10
    // about +Y. Rotation about +Y moves +X points down, so theta_y(L) = M0 L / EI > 0
    // and the tip deflects down: w'' < 0, hogging. Hogging is tension on the top face.
    // CSI (Analysis Reference Eqns. 2): s11 = F11/th - 12 M11 x3 / th^3, positive moment
    // gives positive stress at the bottom, so hogging is M11 = -M0/b = -10 and at the top
    // (x3 = +t/2) s11 = 50/0.2 + 6 * 10 / 0.2^2 = 250 + 1500 = 1750 kN/m^2.
    let mut m = strip();
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["D"];
    let (ei, ea) = (E * B * T.powi(3) / 12.0, E * B * T);
    for name in ["A0", "A1", "A2", "A3"] {
        let a = &r.area_results[name];
        close("F11", a.f11, 50.0, 1e-9);
        // Independent assertion 1: the CSI sign of the moment.
        close("M11", a.m11, -10.0, 1e-9);
        // Independent assertion 2: the physical stress at the top face.
        close("s11_top", a.s11_top, 1750.0, 1e-9);
        close("svm_top", a.svm_top, 1750.0, 1e-9);
        // Zero targets: absolute tolerance of 1e-9 of the 1750 kN/m^2 stress scale.
        assert!(a.s22_top.abs() < 1.75e-6, "s22_top {}", a.s22_top);
        assert!(a.s12_top.abs() < 1.75e-6, "s12_top {}", a.s12_top);
    }
    let tip = r.joint_displacements["J40"];
    close("tip w", tip[2], -10.0 * L * L / (2.0 * ei), 1e-9);
    close("tip u", tip[0], 50.0 * L / ea, 1e-9);
    close("tip theta_y", tip[4], 10.0 * L / ei, 1e-9);
}

#[test]
fn meshed_area_takes_von_mises_from_averaged_components() {
    // One auto-meshed area (2 x 2) of the same strip. The tip load acts at the two
    // corner joints only, so the sub-elements differ and the average of their von Mises
    // values is not the von Mises value of the averaged components. The reported value
    // must be the latter (a von Mises value is not linear in the components).
    let mut m = strip();
    for i in 1..4 {
        m.areas.shift_remove(&format!("A{i}"));
    }
    m.areas.shift_remove("A0");
    for j in ["J10", "J11", "J20", "J21", "J30", "J31"] {
        m.points.shift_remove(j);
    }
    m.add_area(
        "M",
        vec!["J00".into(), "J40".into(), "J41".into(), "J01".into()],
        "PL",
    );
    m.areas["M"].auto_mesh = (2, 2);
    analyze_model(&mut m).unwrap();
    let a = &m.case_results["D"].area_results["M"];
    let vm = (a.s11_top.powi(2) - a.s11_top * a.s22_top + a.s22_top.powi(2)
        + 3.0 * a.s12_top.powi(2))
    .sqrt();
    close("svm_top", a.svm_top, vm, 1e-12);
    // The top face is in tension at the root of a hogging strip, as for the 4 x 1 mesh.
    assert!(a.s11_top > 0.0 && a.m11 < 0.0, "{a:?}");
    // 1.0 D is D.
    m.load_combos.insert(
        "C".into(),
        LoadCombo {
            name: "C".into(),
            combo_type: ComboType::LinearAdd,
            cases: vec![("D".into(), 1.0)],
        },
    );
    analyze_model(&mut m).unwrap();
    let c = &m.case_results["C"].area_results["M"];
    let d = &m.case_results["D"].area_results["M"];
    for (x, y) in [(c.f11, d.f11), (c.m11, d.m11), (c.s11_top, d.s11_top), (c.svm_top, d.svm_top)] {
        assert!((x - y).abs() <= 1e-12 * y.abs().max(1.0), "{x} vs {y}");
    }
}

fn combo(m: &mut SapModel, name: &str, kind: ComboType, cases: &[(&str, f64)]) {
    m.load_combos.insert(
        name.into(),
        LoadCombo {
            name: name.into(),
            combo_type: kind,
            cases: cases.iter().map(|(c, f)| (c.to_string(), *f)).collect(),
        },
    );
}

#[test]
fn linear_combinations_recompute_von_mises_from_combined_components() {
    // D: top s11 = +1750 (F11 = 50, M11 = -10). L: MY = -5 per tip joint, M11 = +10,
    // F11 = 0, top s11 = -6 * 10 / 0.04 = -1500. All other components are zero.
    let mut m = strip();
    combo(&mut m, "ULT", ComboType::LinearAdd, &[("D", 1.2), ("L", 1.6)]);
    combo(&mut m, "SUM", ComboType::LinearAdd, &[("D", 1.0), ("L", 1.0)]);
    combo(&mut m, "RNG", ComboType::RangeAdd, &[("D", 1.2), ("L", 1.6)]);
    combo(&mut m, "ENV", ComboType::Envelope, &[("D", 1.0), ("L", 1.0)]);
    analyze_model(&mut m).unwrap();
    for name in ["A0", "A3"] {
        // 1.2 * 1750 - 1.6 * 1500 = -300; the old linear sum of von Mises values
        // would give 1.2 * 1750 + 1.6 * 1500 = 4500.
        for c in ["ULT", "RNG"] {
            let a = &m.case_results[c].area_results[name];
            close("s11_top", a.s11_top, -300.0, 1e-8);
            close("svm_top", a.svm_top, 300.0, 1e-8);
            close("F11", a.f11, 60.0, 1e-8);
            close("M11", a.m11, -12.0 + 16.0, 1e-8);
        }
        // D + L: s11 = 250, so von Mises = 250 (old value 3250).
        let a = &m.case_results["SUM"].area_results[name];
        close("svm_top", a.svm_top, 250.0, 1e-8);
        // The envelope keeps each field's largest-magnitude value (unchanged rule).
        let a = &m.case_results["ENV"].area_results[name];
        close("env s11_top", a.s11_top, 1750.0, 1e-8);
        close("env svm_top", a.svm_top, 1750.0, 1e-8);
    }
}

fn brick() -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.e = E;
    mat.u = 0.2;
    mat.g = E / 2.4;
    mat.unit_weight = 0.0;
    mat.unit_mass = 0.0;
    m.materials.insert("MAT".into(), mat);
    let mut s = SolidSection::default();
    s.name = "SOL".into();
    s.material = "MAT".into();
    m.solid_sections.insert("SOL".into(), s);
    m.load_cases.clear();
    m.load_patterns.clear();
    for name in ["D", "L"] {
        m.load_patterns.insert(
            name.into(),
            LoadPattern {
                name: name.into(),
                pat_type: LoadPatternType::Dead,
                self_wt_multiplier: 0.0,
            },
        );
        m.load_cases
            .insert(name.into(), LoadCase::linear_static(name, name, 1.0));
    }
    for i in 0..8usize {
        let name = format!("N{i}");
        let (x, y, z) = ((i & 1) as f64, ((i >> 1) & 1) as f64, ((i >> 2) & 1) as f64);
        m.add_point(name.clone(), x, y, z);
        if z == 0.0 {
            m.points[&name].restraint = [true; 6];
        } else {
            for (pat, f) in [
                ("D", [0., 0., 10., 0., 0., 0.]),
                ("L", [10., 0., 0., 0., 0., 0.]),
            ] {
                m.points[&name].loads_force.push(PointLoadForce {
                    load_pat: pat.into(),
                    csys: "GLOBAL".into(),
                    force: f,
                });
            }
        }
    }
    m.solids.insert(
        "S".into(),
        SolidObj {
            name: "S".into(),
            points: std::array::from_fn(|i| format!("N{i}")),
            section: "SOL".into(),
            loads_surface: vec![],
            loads_temp: Default::default(),
            selected: false,
        },
    );
    m
}

#[test]
fn solid_combinations_recompute_principal_and_von_mises_values() {
    use sap_solver::solid::solid_stress_from_components;
    let mut m = brick();
    combo(&mut m, "SUM", ComboType::LinearAdd, &[("D", 1.2), ("L", 1.6)]);
    combo(&mut m, "RNG", ComboType::RangeAdd, &[("D", 1.2), ("L", 1.6)]);
    analyze_model(&mut m).unwrap();
    let d = &m.case_results["D"].solid_results["S"];
    let l = &m.case_results["L"].solid_results["S"];
    assert!(d.center_stress.von_mises > 1.0 && l.center_stress.von_mises > 1.0);
    let pick = |s: &SolidStress, t: &SolidStress| {
        let comb = |f: fn(&SolidStress) -> f64| 1.2 * f(s) + 1.6 * f(t);
        [
            comb(|x| x.s11),
            comb(|x| x.s22),
            comb(|x| x.s33),
            comb(|x| x.s12),
            comb(|x| x.s13),
            comb(|x| x.s23),
        ]
    };
    for c in ["SUM", "RNG"] {
        let r = &m.case_results[c].solid_results["S"];
        let want = solid_stress_from_components(pick(&d.center_stress, &l.center_stress));
        let got = &r.center_stress;
        close("s11", got.s11, want.s11, 1e-9);
        close("s_max", got.s_max, want.s_max, 1e-9);
        close("s_mid", got.s_mid, want.s_mid, 1e-9);
        close("s_min", got.s_min, want.s_min, 1e-9);
        close("von_mises", got.von_mises, want.von_mises, 1e-9);
        // The linear sum of the von Mises values is a different (wrong) number.
        let linear_vm = 1.2 * d.center_stress.von_mises + 1.6 * l.center_stress.von_mises;
        assert!((got.von_mises - linear_vm).abs() > 1e-3 * linear_vm, "{c}");
        for g in 0..8 {
            let want =
                solid_stress_from_components(pick(&d.gauss_stresses[g], &l.gauss_stresses[g]));
            close("gauss vm", r.gauss_stresses[g].von_mises, want.von_mises, 1e-9);
            let want =
                solid_stress_from_components(pick(&d.nodal_stresses[g], &l.nodal_stresses[g]));
            close("nodal vm", r.nodal_stresses[g].von_mises, want.von_mises, 1e-9);
        }
    }
}

/// The strip with mass, a modal case and a response-spectrum case `RS`.
fn strip_with_spectrum() -> SapModel {
    let mut m = strip();
    m.materials["MAT"].unit_mass = 2.5;
    m.load_cases
        .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 6));
    m.functions.insert(
        "SPEC".into(),
        DynamicFunction {
            name: "SPEC".into(),
            is_time_history: false,
            damping_ratio: 0.05,
            points: vec![(0.0, 1.0), (10.0, 1.0)],
        },
    );
    let mut rs = LoadCase::linear_static("RS", "D", 1.0);
    rs.case_type = LoadCaseType::ResponseSpectrum;
    rs.loads.clear();
    rs.initial_case = Some("MODAL".into());
    rs.spectrum_loads = vec![("SPEC".into(), "U1".into(), 1.0)];
    m.load_cases.insert("RS".into(), rs);
    m
}

#[test]
fn combination_reports_area_results_only_when_every_case_does() {
    let mut m = strip_with_spectrum();
    combo(&mut m, "D+RS", ComboType::LinearAdd, &[("D", 1.0), ("RS", 1.0)]);
    combo(&mut m, "RS+D", ComboType::Envelope, &[("RS", 1.0), ("D", 1.0)]);
    combo(&mut m, "D+L", ComboType::LinearAdd, &[("D", 1.0), ("L", 1.0)]);
    analyze_model(&mut m).unwrap();
    // The spectrum case carries joint results but no area results.
    assert!(m.case_results["RS"].area_results.is_empty());
    assert!(!m.case_results["RS"].joint_displacements.is_empty());
    assert!(!m.case_results["D"].area_results.is_empty());
    // D + RS (either order, any type) reports none rather than D's alone.
    for c in ["D+RS", "RS+D"] {
        assert!(m.case_results[c].area_results.is_empty(), "{c}");
        assert!(m.case_results[c].solid_results.is_empty(), "{c}");
        assert!(!m.case_results[c].joint_displacements.is_empty(), "{c}");
    }
    assert_eq!(m.case_results["D+L"].area_results.len(), 4);
}

#[test]
fn response_spectrum_direction_is_validated() {
    let set = |dir: &str| {
        let mut m = strip_with_spectrum();
        m.load_cases["RS"].spectrum_loads[0].1 = dir.into();
        analyze_model(&mut m).map(|_| m)
    };
    for dir in ["Q9", "", "U4", "up"] {
        let error = set(dir).err().unwrap().to_string();
        assert!(
            error.contains("'RS'") && error.contains(&format!("'{dir}'")),
            "{dir}: {error}"
        );
    }
    for dir in ["R1", "r2", "RZ"] {
        let error = set(dir).err().unwrap().to_string();
        assert!(
            error.contains("'RS'")
                && error.contains("rotational spectrum loads are not implemented"),
            "{dir}: {error}"
        );
    }
    // A refused analysis leaves no usable results.
    let mut m = strip_with_spectrum();
    m.load_cases["RS"].spectrum_loads[0].1 = "Q9".into();
    assert!(analyze_model(&mut m).is_err());
    assert!(m.case_results.is_empty());
    // Every accepted spelling still runs; U1, UX, x and 1 are the same direction.
    let base = set("U1").unwrap().case_results["RS"].joint_displacements["J40"];
    for dir in ["UX", "x", "1"] {
        assert_eq!(
            set(dir).unwrap().case_results["RS"].joint_displacements["J40"],
            base
        );
    }
    for dir in ["U2", "UY", "u3", "Z"] {
        set(dir).unwrap();
    }
}

#[test]
fn s2k_spectrum_direction_missing_means_u1_and_unknown_is_refused() {
    let m = strip_with_spectrum();
    let text = sap_format::s2k::write_s2k(&m);
    assert!(text.contains("CoordDir=U1"), "the writer emits CoordDir");
    let parsed = sap_format::s2k::parse_s2k(&text).unwrap();
    assert_eq!(parsed.load_cases["RS"].spectrum_loads[0].1, "U1");

    // Missing column: U1.
    let without = text.replace("   CoordDir=U1", "");
    assert!(!without.contains("CoordDir"));
    let parsed = sap_format::s2k::parse_s2k(&without).unwrap();
    assert_eq!(parsed.load_cases["RS"].spectrum_loads[0].1, "U1");

    // Present but unknown: refused, naming the case and the value.
    let bad = text.replace("CoordDir=U1", "CoordDir=Q9");
    let error = sap_format::s2k::parse_s2k(&bad).err().unwrap().to_string();
    assert!(error.contains("RS") && error.contains("Q9"), "{error}");
    // A known direction other than U1 is kept.
    let u2 = text.replace("CoordDir=U1", "CoordDir=U2");
    let parsed = sap_format::s2k::parse_s2k(&u2).unwrap();
    assert_eq!(parsed.load_cases["RS"].spectrum_loads[0].1, "U2");
}

#[test]
fn base_reactions_include_the_supports_that_meshing_creates() {
    // A 6 m x 5 m panel with its four corners fixed and a 2 x 2 automatic mesh. The
    // joints the mesh creates on an edge between two fixed corners, and inside an area
    // whose four corners are fixed, inherit the restraints, so part of the load goes
    // to supports that are not joints of the model. Statics: the base reaction is the
    // whole load, 0.8 kN/m2 * 30 m2 = 24 kN, whatever the mesh.
    for mesh in [(1, 1), (2, 2), (3, 2)] {
        let mut m = SapModel::new(Units::KnMC);
        m.load_cases.clear();
        m.load_patterns.clear();
        m.load_patterns.insert(
            "D".into(),
            LoadPattern {
                name: "D".into(),
                pat_type: LoadPatternType::Dead,
                self_wt_multiplier: 0.0,
            },
        );
        m.load_cases
            .insert("D".into(), LoadCase::linear_static("D", "D", 1.0));
        for (name, x, y) in [("1", 0., 0.), ("2", 6., 0.), ("3", 6., 5.), ("4", 0., 5.)] {
            m.add_point(name, x, y, 0.0);
            m.points[name].restraint = [true; 6];
        }
        let section = m.area_sections.keys().next().cloned().unwrap();
        m.add_area("A", vec!["1".into(), "2".into(), "3".into(), "4".into()], section);
        m.areas["A"].auto_mesh = mesh;
        m.areas["A"].loads_uniform.push(AreaLoadUniform {
            load_pat: "D".into(),
            val: 0.8,
            dir: 10,
            csys: "GLOBAL".into(),
        });
        analyze_model(&mut m).unwrap();
        let r = &m.case_results["D"];
        close("base Fz", r.base_reactions[2], 24.0, 1e-9);
        assert!(r.base_reactions[0].abs() < 1e-9 && r.base_reactions[1].abs() < 1e-9);
        // Moments of the reaction about the origin balance the load at the center
        // (3, 2.5): Mx = -(y * Fz) reversed = +2.5 * 24, My = -3 * 24.
        close("base Mx", r.base_reactions[3], 2.5 * 24.0, 1e-9);
        close("base My", r.base_reactions[4], -3.0 * 24.0, 1e-9);
        // Only joints of the model have a reaction row.
        assert_eq!(r.joint_reactions.len(), 4);
    }
}
