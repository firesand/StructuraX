//! Automatic frame subdivision for analysis: published per-mesh SAP2000 values,
//! equivalence with a member split by hand, and the combinations that are
//! refused. Units are kip, in, s throughout.
use sap_model::*;
use sap_solver::analyze_model;

/// How a member between two joints is divided into frame elements.
#[derive(Clone, Copy, Debug)]
enum Mesh {
    /// `n` separate frame objects joined at real joints.
    Explicit(usize),
    /// One frame object with an automatic subdivision into `n` elements.
    Auto(usize),
}

fn bare_model(active_dofs: [bool; 6]) -> SapModel {
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    m.active_dofs = active_dofs;
    m
}

fn add_pattern(m: &mut SapModel, name: &str) {
    m.load_patterns.insert(
        name.into(),
        LoadPattern {
            name: name.into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
}

fn add_material(m: &mut SapModel, name: &str, e: f64, unit_mass: f64) {
    let mut material = Material::default_steel_kip_in();
    material.name = name.into();
    material.e = e;
    material.u = 0.3;
    material.g = e / 2.6;
    material.unit_mass = unit_mass;
    material.unit_weight = 0.0;
    m.materials.insert(name.into(), material);
}

/// A general section with the given properties and no shear deformation.
fn add_section(m: &mut SapModel, name: &str, material: &str, area: f64, i33: f64, i22: f64) {
    let section = FrameSection {
        name: name.into(),
        material: material.into(),
        prop_type: FramePropType::General,
        area,
        i33,
        i22,
        torsional_j: i33 + i22,
        as2: 0.0,
        as3: 0.0,
        modifiers: [1.0; 8],
        ..FrameSection::default()
    };
    m.frame_sections.insert(name.into(), section);
}

/// Adds the member `name` from joint `i` to joint `j` and returns the names of
/// its frame objects in order from `i`.
fn add_member(m: &mut SapModel, name: &str, i: &str, j: &str, section: &str, mesh: Mesh) -> Vec<String> {
    match mesh {
        Mesh::Auto(n) => {
            m.add_frame(name, i, j, section);
            m.frames[name].auto_mesh_segments = n;
            vec![name.to_string()]
        }
        Mesh::Explicit(n) => {
            let (a, b) = (m.points[i].clone(), m.points[j].clone());
            let mut joints = vec![i.to_string()];
            for k in 1..n {
                let t = k as f64 / n as f64;
                let joint = format!("{name}j{k}");
                m.add_point(
                    joint.clone(),
                    a.x + t * (b.x - a.x),
                    a.y + t * (b.y - a.y),
                    a.z + t * (b.z - a.z),
                );
                joints.push(joint);
            }
            joints.push(j.to_string());
            (0..n)
                .map(|k| {
                    let frame = if n == 1 { name.to_string() } else { format!("{name}e{k}") };
                    m.add_frame(frame.clone(), joints[k].clone(), joints[k + 1].clone(), section);
                    frame
                })
                .collect()
        }
    }
}

fn explicit_or_auto(n: usize) -> Vec<Mesh> {
    if n == 1 {
        vec![Mesh::Explicit(1)]
    } else {
        vec![Mesh::Explicit(n), Mesh::Auto(n)]
    }
}

// ---------------------------------------------------------------------------
// CSI Software Verification Example 1-014: eigenvalues of a cantilever beam.
// ---------------------------------------------------------------------------

/// 96 in concrete cantilever, 12 x 18 in, E = 3600 ksi, mass 2.3e-7
/// kip-s2/in4, no shear deformation, Ux and Rx excluded.
fn example_1_014(mesh: Mesh) -> SapModel {
    let mut m = bare_model([false, true, true, false, true, true]);
    add_material(&mut m, "CONC", 3600.0, 2.3e-7);
    add_section(&mut m, "BEAM", "CONC", 216.0, 5832.0, 2592.0);
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 96.0, 0.0, 0.0);
    m.points["1"].restraint = [true; 6];
    add_member(&mut m, "1", "1", "2", "BEAM", mesh);
    m.load_cases
        .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 5));
    m
}

/// Published SAP2000 periods (s) per number of elements; modes that the
/// lumped-mass model does not have are "N.A." in the publication.
const EXAMPLE_1_014: &[(usize, &[f64])] = &[
    (1, &[0.054547, 0.036364]),
    (2, &[0.042333, 0.028222, 0.008218, 0.005479]),
    (4, &[0.039090, 0.026060, 0.006651, 0.004434, 0.002511]),
    (6, &[0.038485, 0.025657, 0.006330, 0.004220, 0.002321]),
    (8, &[0.038273, 0.025516, 0.006214, 0.004143, 0.002254]),
    (10, &[0.038175, 0.025450, 0.006160, 0.004107, 0.002222]),
    (96, &[0.038003, 0.025335, 0.006065, 0.004043, 0.002166]),
];

#[test]
fn example_1_014_periods_match_the_published_value_for_every_mesh() {
    for &(n, published) in EXAMPLE_1_014 {
        for mesh in explicit_or_auto(n) {
            let mut m = example_1_014(mesh);
            analyze_model(&mut m).unwrap_or_else(|e| panic!("{mesh:?}: {e}"));
            let modes = &m.case_results["MODAL"].modes;
            // Two translational masses per free joint and no rotational mass:
            // a mesh of n elements has 2 n modes.
            assert_eq!(modes.len(), published.len().min(2 * n), "{mesh:?}");
            for (mode, &expected) in modes.iter().zip(published) {
                // Published to six decimals.
                assert!(
                    (mode.period_sec - expected).abs() < 6e-7,
                    "{mesh:?} mode {}: {} vs {expected}",
                    mode.mode_num,
                    mode.period_sec
                );
            }
        }
    }
}

// ---------------------------------------------------------------------------
// CSI Software Verification Example 1-019: buckling of a rigid frame.
// ---------------------------------------------------------------------------

/// 144 x 144 in portal frame, pinned bases, W8X31 (I = 110 in4), axial area
/// x 100000 and no shear deformation, unit load on each column.
///
/// E is 29000 ksi. The problem statement prints 29,900, but the vendor model
/// file stores 29000 and so do the published numbers: the independent value
/// 280.19 kip is Timoshenko and Gere's `kL tan kL = 6` with E = 29000
/// (29,900 would give 288.9 kip).
fn example_1_019(mesh: Mesh) -> SapModel {
    let mut m = bare_model([true, false, true, false, true, false]);
    add_material(&mut m, "STEEL", 29000.0, 0.0);
    add_section(&mut m, "W8X31", "STEEL", 9.12, 110.0, 37.1);
    m.frame_sections["W8X31"].modifiers[0] = 100_000.0;
    for (name, x, z) in [("1", 0.0, 0.0), ("2", 0.0, 144.0), ("5", 144.0, 144.0), ("6", 144.0, 0.0)] {
        m.add_point(name, x, 0.0, z);
    }
    for base in ["1", "6"] {
        m.points[base].restraint = [true, true, true, true, false, true];
    }
    add_member(&mut m, "1", "1", "2", "W8X31", mesh);
    add_member(&mut m, "3", "2", "5", "W8X31", mesh);
    add_member(&mut m, "5", "6", "5", "W8X31", mesh);
    add_pattern(&mut m, "P");
    for top in ["2", "5"] {
        m.points[top].loads_force.push(PointLoadForce {
            load_pat: "P".into(),
            csys: "GLOBAL".into(),
            force: [0.0, 0.0, -1.0, 0.0, 0.0, 0.0],
        });
    }
    let mut case = LoadCase::linear_static("BUCK", "P", 1.0);
    case.case_type = LoadCaseType::Buckling;
    case.num_buckling_modes = 1;
    m.load_cases.insert("BUCK".into(), case);
    m
}

#[test]
fn example_1_019_buckling_load_matches_the_published_value_for_every_mesh() {
    // Published SAP2000 buckling loads (kip); Timoshenko and Gere give 280.19.
    for (n, published) in [(1, 280.98), (2, 280.24), (4, 280.19)] {
        for mesh in explicit_or_auto(n) {
            let mut m = example_1_019(mesh);
            analyze_model(&mut m).unwrap_or_else(|e| panic!("{mesh:?}: {e}"));
            let factor = m.case_results["BUCK"].buckling_factors[0];
            assert!((factor - published).abs() < 6e-3, "{mesh:?}: {factor} vs {published}");
        }
    }
}

// ---------------------------------------------------------------------------
// CSI Software Verification Example 1-016: tension stiffening by P-Delta.
// ---------------------------------------------------------------------------

/// 300 in simply supported 3 x 3 in steel rod in two objects, 0.002 kip/in
/// downward, 20.25 kip tension applied either as an assigned P-Delta force or
/// by a nonlinear static P-Delta case whose stiffness the load case uses.
fn example_1_016(mesh: Mesh, tension_by_case: bool) -> SapModel {
    let mut m = bare_model([true; 6]);
    add_material(&mut m, "STEEL", 30000.0, 0.0);
    add_section(&mut m, "ROD", "STEEL", 9.0, 6.75, 6.75);
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 150.0, 0.0, 0.0);
    m.add_point("3", 300.0, 0.0, 0.0);
    m.points["1"].restraint = [true, true, true, true, false, true];
    m.points["3"].restraint = [false, true, true, true, false, true];
    add_pattern(&mut m, "Q");
    let mut frames = add_member(&mut m, "1", "1", "2", "ROD", mesh);
    frames.extend(add_member(&mut m, "2", "2", "3", "ROD", mesh));
    for frame in &frames {
        m.frames[frame.as_str()].loads_dist.push(FrameLoadDistributed {
            load_pat: "Q".into(),
            my_type: 1,
            dir: 10,
            csys: "GLOBAL".into(),
            rel_dist1: 0.0,
            rel_dist2: 1.0,
            val1: 0.002,
            val2: 0.002,
        });
        if !tension_by_case {
            m.frames[frame.as_str()].pdelta_force = 20.25;
        }
    }
    let mut case = LoadCase::linear_static("Q", "Q", 1.0);
    if tension_by_case {
        add_pattern(&mut m, "T");
        m.points["3"].loads_force.push(PointLoadForce {
            load_pat: "T".into(),
            csys: "GLOBAL".into(),
            force: [20.25, 0.0, 0.0, 0.0, 0.0, 0.0],
        });
        let mut tension = LoadCase::linear_static("T", "T", 1.0);
        tension.case_type = LoadCaseType::NonlinearStatic;
        tension.nl_geom = NLGeomType::PDelta;
        m.load_cases.insert("T".into(), tension);
        case.initial_case = Some("T".into());
    }
    m.load_cases.insert("Q".into(), case);
    m
}

#[test]
fn example_1_016_tension_stiffening_matches_the_published_value_for_every_mesh() {
    // Published SAP2000 midpoint Uz (in) and My (kip-in) per elements per object.
    for (n, uz, my) in [(1, -0.54555, 11.453), (2, -0.54343, 11.495), (16, -0.54330, 11.498)] {
        for tension_by_case in [false, true] {
            for mesh in explicit_or_auto(n) {
                let mut m = example_1_016(mesh, tension_by_case);
                analyze_model(&mut m).unwrap_or_else(|e| panic!("{mesh:?}: {e}"));
                let results = &m.case_results["Q"];
                let got_uz = results.joint_displacements["2"][2];
                assert!(
                    (got_uz - uz).abs() < 6e-6,
                    "{mesh:?} by case {tension_by_case}: Uz {got_uz} vs {uz}"
                );
                // Moment at the J end of the object that ends at the midpoint.
                let last_frame = match mesh {
                    Mesh::Auto(_) => "1".to_string(),
                    Mesh::Explicit(1) => "1".to_string(),
                    Mesh::Explicit(k) => format!("1e{}", k - 1),
                };
                let station = results.frame_forces[&last_frame].last().unwrap();
                assert!((station.rel_dist - 1.0).abs() < 1e-12);
                assert!(
                    (station.m3.abs() - my).abs() < 6e-4,
                    "{mesh:?} by case {tension_by_case}: My {} vs {my}",
                    station.m3
                );
            }
        }
    }
}

// ---------------------------------------------------------------------------
// CSI Software Verification Example 1-017: vibration of a string under tension.
// ---------------------------------------------------------------------------

/// 100 in wire, 1/16 in diameter, E = 30000 ksi, mass 7.324e-7 kip-s2/in4,
/// 0.5 kip tension from a nonlinear static P-Delta case; modes use its stiffness.
fn example_1_017(mesh: Mesh) -> SapModel {
    let mut m = bare_model([true, false, true, false, true, false]);
    add_material(&mut m, "STEEL", 30000.0, 7.324e-7);
    let area = 0.0030679615757712823;
    let inertia = 7.490140565847857e-7;
    add_section(&mut m, "WIRE", "STEEL", area, inertia, inertia);
    // The vendor model keeps shear deformation (shear area 0.9 A).
    m.frame_sections["WIRE"].as2 = 0.0027611653450482503;
    m.frame_sections["WIRE"].as3 = 0.0027611653450482503;
    m.add_point("1", -50.0, 0.0, 0.0);
    m.add_point("2", 50.0, 0.0, 0.0);
    m.points["1"].restraint = [true, true, true, true, false, true];
    m.points["2"].restraint = [false, false, true, false, false, false];
    add_member(&mut m, "1", "1", "2", "WIRE", mesh);
    add_pattern(&mut m, "T");
    m.points["2"].loads_force.push(PointLoadForce {
        load_pat: "T".into(),
        csys: "GLOBAL".into(),
        force: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
    });
    let mut tension = LoadCase::linear_static("T", "T", 1.0);
    tension.case_type = LoadCaseType::NonlinearStatic;
    tension.nl_geom = NLGeomType::PDelta;
    m.load_cases.insert("T".into(), tension);
    let mut modal = LoadCase::modal_eigen("MODAL", 3);
    modal.initial_case = Some("T".into());
    m.load_cases.insert("MODAL".into(), modal);
    m
}

#[test]
fn example_1_017_string_frequencies_match_the_published_value_for_every_mesh() {
    // Published SAP2000 frequencies (Hz), five significant figures.
    for (n, published) in [(10, [74.579, 148.93, 222.06]), (100, [74.587, 149.18, 223.80])] {
        for mesh in explicit_or_auto(n) {
            let mut m = example_1_017(mesh);
            analyze_model(&mut m).unwrap_or_else(|e| panic!("{mesh:?}: {e}"));
            let modes = &m.case_results["MODAL"].modes;
            for (mode, expected) in modes.iter().zip(published) {
                let tolerance = if expected < 100.0 { 6e-4 } else { 6e-3 };
                assert!(
                    (mode.freq_hz - expected).abs() < tolerance,
                    "{mesh:?} mode {}: {} vs {expected}",
                    mode.mode_num,
                    mode.freq_hz
                );
            }
        }
    }
}

// ---------------------------------------------------------------------------
// CSI Software Verification Example 1-006: non-prismatic cantilever, with and
// without automatic subdivision.
// ---------------------------------------------------------------------------

/// 144 in concrete cantilever of four 36 in non-prismatic segments (linear,
/// constant, parabolic and cubic inertia variation), E = 3600 ksi, weight
/// 0.15 kip/ft3, no shear deformation, seven load cases at the free end.
fn example_1_006(segments: usize) -> SapModel {
    let mut m = bare_model([true; 6]);
    add_material(&mut m, "CONC", 3600.0, 0.0);
    m.materials["CONC"].g = 1500.0;
    m.materials["CONC"].u = 0.2;
    m.materials["CONC"].unit_weight = 0.15 / 1728.0;
    for (name, depth, j) in [
        ("FSEC1", 12.0f64, 2920.3201375007625),
        ("FSEC2", 18.0, 6085.120136260985),
        ("FSEC3", 24.0, 9492.12013605237),
        ("FSEC4", 30.0, 12934.72986399536),
    ] {
        add_section(&mut m, name, "CONC", 12.0 * depth, depth.powi(3), 144.0 * depth);
        m.frame_sections[name].torsional_j = j;
    }
    add_section(&mut m, "VAR1", "CONC", 0.0, 0.0, 0.0);
    let variable = &mut m.frame_sections["VAR1"];
    variable.prop_type = FramePropType::Variable;
    variable.modifiers[1] = 0.0;
    variable.modifiers[2] = 0.0;
    for (start, end, variation) in [
        ("FSEC4", "FSEC3", 1),
        ("FSEC2", "FSEC2", 1),
        ("FSEC3", "FSEC1", 2),
        ("FSEC1", "FSEC2", 3),
    ] {
        variable.non_prismatic.push(NonPrismaticSegment {
            start_sec: start.into(),
            end_sec: end.into(),
            length: 36.0,
            is_variable_length: false,
            ei33_variation: variation,
            ei22_variation: variation,
        });
    }
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 144.0, 0.0, 0.0);
    m.points["1"].restraint = [true; 6];
    m.add_frame("1", "1", "2", "VAR1");
    m.frames["1"].auto_mesh_segments = segments;
    // 70 intervals, so that the stations of one, seven and ten elements
    // (uniform within each element) include the same positions.
    m.frames["1"].min_stations = 71;
    let tip_loads = [
        [500.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 5.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, -5.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 5000.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 5000.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 5000.0],
    ];
    add_pattern(&mut m, "1");
    m.load_patterns["1"].self_wt_multiplier = 1.0;
    m.load_cases.insert("1".into(), LoadCase::linear_static("1", "1", 1.0));
    for (index, force) in tip_loads.into_iter().enumerate() {
        let name = (index + 2).to_string();
        add_pattern(&mut m, &name);
        m.points["2"].loads_force.push(PointLoadForce {
            load_pat: name.clone(),
            csys: "GLOBAL".into(),
            force,
        });
        m.load_cases
            .insert(name.clone(), LoadCase::linear_static(name.clone(), name, 1.0));
    }
    m
}

#[test]
fn example_1_006_non_prismatic_results_match_published_values_with_and_without_subdivision() {
    // Published for Model A (one element) and Model B (ten elements) alike:
    // fixed-end My under self weight (kip-in), then the free-end displacement
    // component in the direction of each tip load (in, rad).
    let published = [0.09087, 0.43120, -0.14335, 0.07987, 0.03742, 0.07634];
    let mut reference: Option<SapModel> = None;
    // Seven elements cut every vendor segment away from its ends.
    for segments in [0, 10, 7] {
        let mut m = example_1_006(segments);
        analyze_model(&mut m).unwrap_or_else(|e| panic!("{segments} segments: {e}"));
        let fixed_end = &m.case_results["1"].frame_forces["1"][0];
        assert_eq!(fixed_end.rel_dist, 0.0);
        assert!(
            (fixed_end.m3 + 184.950).abs() < 6e-4,
            "{segments} segments: My {}",
            fixed_end.m3
        );
        for (index, expected) in published.into_iter().enumerate() {
            let got = m.case_results[&(index + 2).to_string()].joint_displacements["2"][index];
            assert!(
                (got - expected).abs() < 6e-6,
                "{segments} segments, case {}: {got} vs {expected}",
                index + 2
            );
        }
        // Free-end deflection under self weight, which the publication does
        // not list: unit-load integral of M m / (E I) over the four segments
        // with the varying weight, evaluated independently of the solver
        // (midpoint rule, 400 000 intervals): -0.0218825865 in.
        let tip = m.case_results["1"].joint_displacements["2"][2];
        assert!(
            (tip + 0.0218825865).abs() < 1e-7 * 0.0218825865,
            "{segments} segments: self-weight tip Uz {tip}"
        );
        // Each element carries its part of the variation: the subdivided
        // member is the same member, not an approximation of it. The
        // difference left is quadrature: five Gauss points per vendor
        // segment for one element, and per shorter piece when subdivided.
        if let Some(reference) = &reference {
            for case in m.case_results.keys() {
                let (a, b) = (&reference.case_results[case], &m.case_results[case]);
                for d in 0..6 {
                    let (x, y) = (a.joint_displacements["2"][d], b.joint_displacements["2"][d]);
                    assert!((x - y).abs() <= 1e-5 * x.abs().max(1e-6), "{case} dof {d}: {x} vs {y}");
                }
                assert_same_station_forces(&a.frame_forces["1"], &b.frame_forces["1"], 1e-5);
            }
        } else {
            reference = Some(m);
        }
    }
}

/// Every station of `whole` must appear in `meshed` at the same position with
/// the same forces and deflections. `meshed` may report a position twice where
/// the forces jump.
fn assert_same_station_forces(whole: &[FrameStationResult], meshed: &[FrameStationResult], tolerance: f64) {
    let values = |s: &FrameStationResult| {
        let mut v = vec![s.p, s.v2, s.v3, s.t, s.m2, s.m3];
        v.extend(s.local_disp);
        v
    };
    let scale = whole
        .iter()
        .flat_map(|s| [s.p, s.v2, s.v3, s.t, s.m2, s.m3])
        .fold(0.0f64, |m, v| m.max(v.abs()))
        .max(1e-12);
    let disp_scale = whole
        .iter()
        .flat_map(|s| s.local_disp)
        .fold(0.0f64, |m, v| m.max(v.abs()))
        .max(1e-12);
    assert!((meshed.last().unwrap().rel_dist - 1.0).abs() < 1e-12);
    assert!(meshed.windows(2).all(|w| w[0].rel_dist <= w[1].rel_dist));
    for station in whole {
        let matched = meshed
            .iter()
            .filter(|s| (s.rel_dist - station.rel_dist).abs() < 1e-9)
            .any(|s| {
                (s.station - station.station).abs() < 1e-9 * station.station.max(1.0)
                    && values(s).iter().zip(values(station)).enumerate().all(|(i, (a, b))| {
                        (a - b).abs() <= tolerance * if i < 6 { scale } else { disp_scale }
                    })
            });
        assert!(matched, "no matching station at {}: {station:?} in {meshed:?}", station.rel_dist);
    }
}

// ---------------------------------------------------------------------------
// A subdivided member is the same member in a linear static analysis.
// ---------------------------------------------------------------------------

/// A skew member with shear deformation, a partially fixed end release, self
/// weight and every kind of span load, fixed at one end and pinned at the other.
fn loaded_skew_member(segments: usize) -> SapModel {
    let mut m = bare_model([true; 6]);
    add_material(&mut m, "STEEL", 29000.0, 7.3e-7);
    m.materials["STEEL"].unit_weight = 2.83e-4;
    m.materials["STEEL"].a = 6.5e-6;
    add_section(&mut m, "S", "STEEL", 10.0, 300.0, 80.0);
    m.frame_sections["S"].as2 = 6.0;
    m.frame_sections["S"].as3 = 7.0;
    m.frame_sections["S"].torsional_j = 12.0;
    m.add_point("A", 0.0, 0.0, 0.0);
    m.add_point("B", 120.0, 50.0, 30.0);
    m.points["A"].restraint = [true; 6];
    m.points["B"].restraint = [true, true, true, false, false, false];
    m.points["A"].pattern_values.insert("TP".into(), 2.0);
    m.points["B"].pattern_values.insert("TP".into(), 5.0);
    m.add_frame("M", "A", "B", "S");
    add_pattern(&mut m, "L");
    m.load_patterns["L"].self_wt_multiplier = 1.3;
    let frame = &mut m.frames["M"];
    frame.angle_deg = 20.0;
    frame.min_stations = 29;
    frame.auto_mesh_segments = segments;
    frame.release_j[5] = true;
    frame.end_spring[5] = 4000.0;
    frame.release_i[3] = false;
    let dist = |my_type, dir, rel_dist1, rel_dist2, val1, val2| FrameLoadDistributed {
        load_pat: "L".into(),
        my_type,
        dir,
        csys: "GLOBAL".into(),
        rel_dist1,
        rel_dist2,
        val1,
        val2,
    };
    frame.loads_dist = vec![
        dist(1, 2, 0.15, 0.85, 0.1, 0.3),
        dist(1, 9, 0.3, 0.6, -0.2, -0.05),
        dist(1, 1, 0.0, 1.0, 0.04, 0.04),
        dist(2, 1, 0.1, 0.9, 0.5, 1.5),
        dist(1, 10, 0.0, 0.4, 0.02, 0.07),
    ];
    let point = |my_type, dir, rel_dist, val| FrameLoadPoint {
        load_pat: "L".into(),
        my_type,
        dir,
        csys: "GLOBAL".into(),
        rel_dist,
        val,
    };
    frame.loads_point = vec![
        point(1, 6, 0.37, -3.0),
        point(2, 5, 0.61, 40.0),
        // On an element boundary for four segments.
        point(1, 3, 0.5, 2.0),
        point(1, 4, 0.0, 9.0),
        point(1, 2, 1.0, 1.0),
    ];
    frame.loads_temp = vec![
        FrameLoadTemperature {
            load_pat: "L".into(),
            my_type: 1,
            val: 30.0,
            pattern_name: Some("TP".into()),
        },
        FrameLoadTemperature {
            load_pat: "L".into(),
            my_type: 2,
            val: 4.0,
            pattern_name: None,
        },
    ];
    frame.loads_gravity.insert("L".into(), [0.2, -0.1, 0.0]);
    m.load_cases.insert("L".into(), LoadCase::linear_static("L", "L", 1.0));
    m
}

#[test]
fn a_subdivided_member_gives_the_unsubdivided_linear_static_solution() {
    let mut whole = loaded_skew_member(0);
    analyze_model(&mut whole).unwrap();
    let reference = &whole.case_results["L"];
    // The loads must be doing something in every direction.
    assert!(reference.joint_displacements["B"][3..].iter().all(|r| r.abs() > 1e-6));
    for segments in [2, 4, 7] {
        let mut m = loaded_skew_member(segments);
        analyze_model(&mut m).unwrap();
        let results = &m.case_results["L"];
        // Only the model's own joints and frames are reported.
        assert_eq!(results.joint_displacements.keys().collect::<Vec<_>>(), ["A", "B"]);
        assert_eq!(results.frame_forces.keys().collect::<Vec<_>>(), ["M"]);
        for joint in ["A", "B"] {
            for d in 0..6 {
                let (x, y) = (reference.joint_displacements[joint][d], results.joint_displacements[joint][d]);
                assert!((x - y).abs() <= 1e-9 * x.abs().max(1e-6), "{segments}: {joint} u{d} {x} vs {y}");
                let (x, y) = (reference.joint_reactions[joint][d], results.joint_reactions[joint][d]);
                assert!((x - y).abs() <= 1e-9 * x.abs().max(1.0), "{segments}: {joint} r{d} {x} vs {y}");
            }
        }
        for d in 0..6 {
            let (x, y) = (reference.base_reactions[d], results.base_reactions[d]);
            assert!((x - y).abs() <= 1e-9 * x.abs().max(1.0), "{segments}: base {d} {x} vs {y}");
        }
        assert_same_station_forces(&reference.frame_forces["M"], &results.frame_forces["M"], 1e-9);
    }
    // (The point load at mid-length sits on an element boundary for two and
    // four segments; the reactions above show it is applied exactly once.)
}

// ---------------------------------------------------------------------------
// Subdivision at the joints that lie on a frame.
// ---------------------------------------------------------------------------

/// 200 in beam on two supports under 0.01 kip/in, with a third support joint
/// at mid-length that is not an end of any frame object.
fn beam_with_joint_on_span(at_joints: bool) -> SapModel {
    let mut m = bare_model([true, false, true, false, true, false]);
    add_material(&mut m, "STEEL", 29000.0, 0.0);
    add_section(&mut m, "S", "STEEL", 10.0, 100.0, 100.0);
    m.add_point("1", 0.0, 0.0, 0.0);
    m.add_point("2", 200.0, 0.0, 0.0);
    m.add_point("MID", 100.0, 0.0, 0.0);
    m.points["1"].restraint = [true, true, true, true, false, true];
    m.points["2"].restraint = [false, true, true, true, false, true];
    m.points["MID"].restraint = [true; 6];
    m.add_frame("B", "1", "2", "S");
    m.frames["B"].auto_mesh_at_joints = at_joints;
    add_pattern(&mut m, "W");
    m.frames["B"].loads_dist.push(FrameLoadDistributed {
        load_pat: "W".into(),
        my_type: 1,
        dir: 10,
        csys: "GLOBAL".into(),
        rel_dist1: 0.0,
        rel_dist2: 1.0,
        val1: 0.01,
        val2: 0.01,
    });
    m.load_cases.insert("W".into(), LoadCase::linear_static("W", "W", 1.0));
    m
}

#[test]
fn a_joint_on_the_span_connects_only_when_the_frame_is_subdivided_at_joints() {
    let (w, l) = (0.01, 200.0);
    // Not subdivided: the joint is not part of the beam, which spans 200 in.
    let mut m = beam_with_joint_on_span(false);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["W"];
    assert!((r.joint_reactions["1"][2] - w * l / 2.0).abs() < 1e-9);
    assert!(r.joint_reactions.get("MID").is_none_or(|f| f[2].abs() < 1e-12));

    // Subdivided at joints: a two-span continuous beam. Closed form: the
    // interior support takes 5 w L / 8, each end 3 w L / 16, and the moment
    // over the support is w (L/2)^2 / 8.
    let mut m = beam_with_joint_on_span(true);
    analyze_model(&mut m).unwrap();
    let r = &m.case_results["W"];
    assert!((r.joint_reactions["MID"][2] - 5.0 * w * l / 8.0).abs() < 1e-9);
    assert!((r.joint_reactions["1"][2] - 3.0 * w * l / 16.0).abs() < 1e-9);
    assert!((r.joint_reactions["2"][2] - 3.0 * w * l / 16.0).abs() < 1e-9);
    // The support joint is reported from both sides: the moment is the same
    // and the shear jumps by the reaction.
    let stations = &r.frame_forces["B"];
    let over_support: Vec<_> = stations.iter().filter(|s| (s.rel_dist - 0.5).abs() < 1e-12).collect();
    assert_eq!(over_support.len(), 2);
    for station in &over_support {
        assert!((station.station - 100.0).abs() < 1e-9);
        assert!((station.m3.abs() - w * (l / 2.0).powi(2) / 8.0).abs() < 1e-9);
    }
    let jump = (over_support[0].v2 - over_support[1].v2).abs();
    assert!((jump - 5.0 * w * l / 8.0).abs() < 1e-9, "{jump}");
    assert_eq!(r.frame_forces.keys().collect::<Vec<_>>(), ["B"]);

    // A joint off the axis by more than a millionth of the length is not on it.
    let mut m = beam_with_joint_on_span(true);
    m.points["MID"].z = 0.001;
    analyze_model(&mut m).unwrap();
    assert!((m.case_results["W"].joint_reactions["1"][2] - w * l / 2.0).abs() < 1e-9);
}

#[test]
fn every_joint_on_the_span_splits_the_frame_in_order_along_it() {
    // Three spring supports at unequal spacing, defined out of order, against
    // the same beam built from four frame objects.
    let build = |explicit: bool| {
        let mut m = bare_model([true, false, true, false, true, false]);
        add_material(&mut m, "STEEL", 29000.0, 0.0);
        add_section(&mut m, "S", "STEEL", 10.0, 100.0, 100.0);
        m.add_point("1", 10.0, 0.0, 5.0);
        m.add_point("2", 250.0, 0.0, 77.0);
        m.points["1"].restraint = [true, true, true, true, false, true];
        m.points["2"].restraint = [false, true, true, true, false, true];
        for (name, t, spring) in [("P7", 0.7, 3.0), ("P2", 0.2, 8.0), ("P45", 0.45, 1.5)] {
            m.add_point(name, 10.0 + 240.0 * t, 0.0, 5.0 + 72.0 * t);
            m.points[name].spring[2] = spring;
        }
        add_pattern(&mut m, "W");
        let load = |a: f64, b: f64| FrameLoadDistributed {
            load_pat: "W".into(),
            my_type: 1,
            dir: 10,
            csys: "GLOBAL".into(),
            rel_dist1: a,
            rel_dist2: b,
            val1: 0.01,
            val2: 0.01,
        };
        if explicit {
            for (name, i, j) in [("a", "1", "P2"), ("b", "P2", "P45"), ("c", "P45", "P7"), ("d", "P7", "2")] {
                m.add_frame(name, i, j, "S");
                m.frames[name].loads_dist.push(load(0.0, 1.0));
            }
        } else {
            m.add_frame("B", "1", "2", "S");
            m.frames["B"].auto_mesh_at_joints = true;
            m.frames["B"].loads_dist.push(load(0.0, 1.0));
        }
        m.load_cases.insert("W".into(), LoadCase::linear_static("W", "W", 1.0));
        analyze_model(&mut m).unwrap();
        m
    };
    let (split, meshed) = (build(true), build(false));
    let (a, b) = (&split.case_results["W"], &meshed.case_results["W"]);
    for joint in ["1", "2", "P2", "P45", "P7"] {
        for d in 0..6 {
            let (x, y) = (a.joint_displacements[joint][d], b.joint_displacements[joint][d]);
            assert!((x - y).abs() <= 1e-9 * x.abs().max(1e-9), "{joint} u{d}: {x} vs {y}");
            let (x, y) = (a.joint_reactions[joint][d], b.joint_reactions[joint][d]);
            assert!((x - y).abs() <= 1e-9 * x.abs().max(1e-9), "{joint} r{d}: {x} vs {y}");
        }
    }
    // The spring supports carry load, so the split matters.
    assert!(b.joint_reactions["P45"][2].abs() > 0.1);
}

// ---------------------------------------------------------------------------
// Only the model's own joints and frames appear in results.
// ---------------------------------------------------------------------------

#[test]
fn results_of_every_case_type_name_only_model_joints_and_frames() {
    let mut m = example_1_019(Mesh::Auto(4));
    for material in m.materials.values_mut() {
        material.unit_mass = 7.3e-7;
    }
    m.load_cases.insert("STATIC".into(), LoadCase::linear_static("STATIC", "P", 1.0));
    m.load_cases.insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 4));
    add_pattern(&mut m, "H");
    m.points["2"].loads_force.push(PointLoadForce {
        load_pat: "H".into(),
        csys: "GLOBAL".into(),
        force: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    });
    m.functions.insert(
        "STEP".into(),
        DynamicFunction {
            name: "STEP".into(),
            is_time_history: true,
            damping_ratio: 0.0,
            points: vec![(0.0, 1.0), (10.0, 1.0)],
        },
    );
    m.functions.insert(
        "SPEC".into(),
        DynamicFunction {
            name: "SPEC".into(),
            is_time_history: false,
            damping_ratio: 0.05,
            points: vec![(0.0, 1.0), (10.0, 1.0)],
        },
    );
    let mut history = LoadCase::linear_static("HIST", "H", 1.0);
    history.case_type = LoadCaseType::LinearDynamic;
    history.time_step_dt = 0.001;
    history.num_time_steps = 20;
    history.history_loads = vec![("STEP".into(), "H".into(), 1.0)];
    m.load_cases.insert("HIST".into(), history.clone());
    history.name = "HIST2".into();
    history.initial_case = Some("HIST".into());
    m.load_cases.insert("HIST2".into(), history);
    let mut spectrum = LoadCase::linear_static("SPEC", "H", 1.0);
    spectrum.case_type = LoadCaseType::ResponseSpectrum;
    spectrum.loads.clear();
    spectrum.initial_case = Some("MODAL".into());
    spectrum.spectrum_loads = vec![("SPEC".into(), "U1".into(), 1.0)];
    m.load_cases.insert("SPEC".into(), spectrum);
    let mut pdelta = LoadCase::linear_static("PDELTA", "P", 50.0);
    pdelta.case_type = LoadCaseType::NonlinearStatic;
    pdelta.nl_geom = NLGeomType::PDelta;
    m.load_cases.insert("PDELTA".into(), pdelta);
    m.load_combos.insert(
        "COMBO".into(),
        LoadCombo {
            name: "COMBO".into(),
            combo_type: ComboType::LinearAdd,
            cases: vec![("STATIC".into(), 1.2), ("HIST".into(), 0.5)],
        },
    );
    analyze_model(&mut m).unwrap();

    let joints: Vec<&String> = m.points.keys().collect();
    let frames: Vec<&String> = m.frames.keys().collect();
    assert_eq!(m.case_results.len(), 8);
    for (name, r) in &m.case_results {
        let only_joints = |keys: Vec<&String>, what: &str| {
            assert!(keys.iter().all(|k| joints.contains(k)), "{name} {what}: {keys:?}");
        };
        only_joints(r.joint_displacements.keys().collect(), "displacements");
        only_joints(r.joint_reactions.keys().collect(), "reactions");
        only_joints(r.time_history_disp.keys().collect(), "history");
        for mode in &r.modes {
            assert_eq!(mode.mode_shape.keys().collect::<Vec<_>>(), joints, "{name} mode shape");
        }
        if !r.frame_forces.is_empty() {
            assert_eq!(r.frame_forces.keys().collect::<Vec<_>>(), frames, "{name} frames");
            for stations in r.frame_forces.values() {
                assert_eq!(stations[0].rel_dist, 0.0);
                assert!((stations.last().unwrap().rel_dist - 1.0).abs() < 1e-12);
                assert!(stations.windows(2).all(|w| w[0].station <= w[1].station));
                assert!((stations.last().unwrap().station - 144.0).abs() < 1e-9);
            }
        }
        if let Some(state) = &r.transient_state {
            assert_eq!(state.point_order.iter().collect::<Vec<_>>(), joints, "{name} state");
            for vector in [&state.u, &state.v, &state.a] {
                assert_eq!(vector.len(), 6 * joints.len(), "{name} state vector");
            }
        }
    }
    // Every case and combination reports a frame at the same stations.
    let positions = |name: &str| -> Vec<Vec<f64>> {
        m.case_results[name]
            .frame_forces
            .values()
            .map(|stations| stations.iter().map(|s| s.station).collect())
            .collect()
    };
    for name in ["PDELTA", "HIST", "HIST2", "COMBO", "SPEC"] {
        if !m.case_results[name].frame_forces.is_empty() {
            assert_eq!(positions(name), positions("STATIC"), "{name}");
        }
    }
    for name in ["STATIC", "PDELTA", "HIST", "COMBO"] {
        assert_eq!(m.case_results[name].joint_displacements.len(), joints.len(), "{name}");
        assert!(!m.case_results[name].frame_forces.is_empty(), "{name}");
    }
    assert_eq!(m.case_results["MODAL"].modes.len(), 4);
    assert!(m.case_results["HIST2"].transient_state.is_some());
    // The model itself is untouched by the subdivision.
    assert_eq!(m.points.len(), 4);
    assert_eq!(m.frames.len(), 3);
    assert!(m.frames.values().all(|f| f.auto_mesh_segments == 4));
}

// ---------------------------------------------------------------------------
// Combinations that are refused instead of approximated.
// ---------------------------------------------------------------------------

#[test]
fn unsupported_subdivision_combinations_are_refused_and_leave_no_results() {
    type Change = fn(&mut SapModel);
    let cases: [(&str, Change); 11] = [
        ("is a cable", |m| {
            m.frames["B"].auto_mesh_segments = 2;
            m.frames["B"].cable_params = Some(CableParams {
                undeformed_length: Some(200.0),
                target_tension_i: None,
                horizontal_tension: None,
                num_segments: 1,
            });
            // Cables are analyzed in nonlinear static cases only.
            m.load_cases["W"].case_type = LoadCaseType::NonlinearStatic;
            m.active_dofs = [true; 6];
            m.points["2"].restraint = [true; 6];
        }),
        ("one of its new joints falls on joint 'MID'", |m| {
            // Not subdivided at joints, so the joint at mid-length would stay
            // unconnected next to the new one.
            m.frames["B"].auto_mesh_segments = 2;
        }),
        ("one of its new joints falls on a joint created by automatic area meshing", |m| {
            // Eight segments along the lower edge of a 2 x 2 meshed panel:
            // the new joint at x = 50 is where the panel's edge joint will be.
            m.active_dofs = [true; 6];
            m.points["MID"].restraint = [false; 6];
            m.points["MID"].x = 150.0;
            m.points["MID"].z = 40.0;
            m.add_point("C3", 100.0, 0.0, 0.0);
            m.add_point("C4", 0.0, 0.0, 40.0);
            m.add_point("C5", 100.0, 0.0, 40.0);
            let section = m.area_sections.keys().next().unwrap().clone();
            let mut area = AreaObj::new("A", vec!["1".into(), "C3".into(), "C5".into(), "C4".into()], section);
            area.auto_mesh = (2, 2);
            m.areas.insert("A".into(), area);
            m.frames["B"].auto_mesh_segments = 8;
        }),
        ("one of its new joints falls on a joint created for the subdivision of frame", |m| {
            // Two subdivided frames crossing at their midpoints.
            m.points["MID"].y = 30.0;
            m.active_dofs = [true; 6];
            m.add_point("X1", 100.0, 0.0, -50.0);
            m.add_point("X2", 100.0, 0.0, 50.0);
            m.points["X1"].restraint = [true; 6];
            m.add_frame("X", "X1", "X2", "S");
            m.frames["X"].auto_mesh_segments = 2;
            m.frames["B"].auto_mesh_segments = 4;
        }),
        ("has a tension or compression limit", |m| {
            m.frames["B"].auto_mesh_segments = 2;
            m.frames["B"].tc_limits = Some((true, 0.0, false, 0.0));
        }),
        ("has end length offsets", |m| {
            m.frames["B"].auto_mesh_segments = 2;
            m.frames["B"].end_offset_i = 5.0;
        }),
        ("has both a number of segments and joints lying on it", |m| {
            m.frames["B"].auto_mesh_at_joints = true;
            m.frames["B"].auto_mesh_segments = 2;
        }),
        ("passes through joints created by automatic area meshing", |m| {
            // A 2 x 2 meshed panel whose lower edge runs along half the beam.
            m.active_dofs = [true; 6];
            m.points["MID"].restraint = [false; 6];
            m.points["MID"].x = 150.0;
            m.points["MID"].z = 40.0;
            m.add_point("C3", 100.0, 0.0, 0.0);
            m.add_point("C4", 0.0, 0.0, 40.0);
            m.add_point("C5", 100.0, 0.0, 40.0);
            let section = m.area_sections.keys().next().unwrap().clone();
            let mut area = AreaObj::new("A", vec!["1".into(), "C3".into(), "C5".into(), "C4".into()], section);
            area.auto_mesh = (2, 2);
            m.areas.insert("A".into(), area);
            m.frames["B"].auto_mesh_at_joints = true;
        }),
        ("is reserved for the automatic subdivision", |m| {
            m.frames["B"].auto_mesh_segments = 3;
            m.add_point("__fmesh_B_2", 0.0, 50.0, 0.0);
            m.points["__fmesh_B_2"].restraint = [true; 6];
        }),
        ("is reserved for the automatic subdivision", |m| {
            m.frames["B"].auto_mesh_segments = 3;
            m.add_frame("B__seg_1", "1", "MID", "S");
        }),
        ("at most 10000 are accepted", |m| {
            m.frames["B"].auto_mesh_segments = 10_001;
        }),
    ];
    for (expected, change) in cases {
        let mut m = beam_with_joint_on_span(false);
        change(&mut m);
        let before = m.clone();
        let error = analyze_model(&mut m).unwrap_err().to_string();
        assert!(error.contains(expected), "{expected}: {error}");
        assert!(error.contains("'B'") || error.contains("B_") || error.contains("'X'"), "{error}");
        assert!(m.case_results.is_empty() && !m.is_locked);
        assert_eq!(m.frames, before.frames);
        assert_eq!(m.points, before.points);
    }
    // The same assignments without subdivision are not refused by this rule.
    let mut m = beam_with_joint_on_span(false);
    m.frames["B"].end_offset_i = 5.0;
    analyze_model(&mut m).unwrap();
}
