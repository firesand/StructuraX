//! Frame span moment loads, and the reference state of a buckling case.
//! Closed forms; units are kip, in.
use sap_model::*;
use sap_solver::analyze_model;

fn planar_model() -> SapModel {
    let mut m = SapModel::default();
    m.load_cases.clear();
    m.load_patterns.clear();
    m.active_dofs = [true, false, true, false, true, false];
    let mut material = Material::default_steel_kip_in();
    material.name = "MAT".into();
    material.e = 10_000.0;
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
            i33: 50.0,
            i22: 50.0,
            as2: 0.0,
            as3: 0.0,
            modifiers: [1.0; 8],
            ..FrameSection::default()
        },
    );
    m
}

fn pattern(m: &mut SapModel, name: &str) {
    m.load_patterns.insert(
        name.into(),
        LoadPattern {
            name: name.into(),
            pat_type: LoadPatternType::Other,
            self_wt_multiplier: 0.0,
        },
    );
}

#[test]
fn span_moment_loads_give_the_closed_form_moment_diagram_and_deflection() {
    // Cantilever along X, fixed at 0, length 100, E I = 5e5.
    let (length, ei) = (100.0, 10_000.0 * 50.0);
    let build = || {
        let mut m = planar_model();
        m.add_point("A", 0.0, 0.0, 0.0);
        m.add_point("B", length, 0.0, 0.0);
        m.points["A"].restraint = [true; 6];
        m.add_frame("F", "A", "B", "S");
        m.frames["F"].min_stations = 11;
        pattern(&mut m, "M");
        m.load_cases.insert("M".into(), LoadCase::linear_static("M", "M", 1.0));
        m
    };

    // A concentrated moment M0 about global Y at a = 40: the member carries
    // M0 from the support to the load and nothing beyond it. The free end
    // rotates by M0 a / (E I) and moves by M0 a (L - a / 2) / (E I).
    let (m0, a) = (30.0, 40.0);
    let mut m = build();
    m.frames["F"].loads_point.push(FrameLoadPoint {
        load_pat: "M".into(),
        my_type: 2,
        dir: 5,
        csys: "GLOBAL".into(),
        rel_dist: a / length,
        val: m0,
    });
    analyze_model(&mut m).unwrap();
    let results = &m.case_results["M"];
    let tip = results.joint_displacements["B"];
    assert!((tip[4].abs() - m0 * a / ei).abs() < 1e-12, "{tip:?}");
    assert!((tip[2].abs() - m0 * a * (length - a / 2.0) / ei).abs() < 1e-10, "{tip:?}");
    // A positive rotation about Y lowers the free end.
    assert!(tip[4] > 0.0 && tip[2] < 0.0);
    for station in &results.frame_forces["F"] {
        let expected = if station.station < a - 1e-9 { m0 } else { 0.0 };
        assert!(
            (station.m3.abs() - expected).abs() < 1e-9,
            "at {}: {}",
            station.station,
            station.m3
        );
        assert!(station.v2.abs() < 1e-9 && station.p.abs() < 1e-9);
        // The elastic curve: no curvature beyond the load.
        let x = station.station;
        let deflection = if x <= a { m0 * x * x / (2.0 * ei) } else { m0 * a * (x - a / 2.0) / ei };
        assert!(
            (station.local_disp[1].abs() - deflection).abs() < 1e-10,
            "at {x}: {:?} vs {deflection}",
            station.local_disp
        );
    }

    // A uniform moment per length over the whole member: M(x) = m (L - x),
    // free-end rotation m L^2 / (2 E I).
    let per_length = 0.4;
    let mut m = build();
    m.frames["F"].loads_dist.push(FrameLoadDistributed {
        load_pat: "M".into(),
        my_type: 2,
        dir: 5,
        csys: "GLOBAL".into(),
        rel_dist1: 0.0,
        rel_dist2: 1.0,
        val1: per_length,
        val2: per_length,
    });
    analyze_model(&mut m).unwrap();
    let results = &m.case_results["M"];
    let rotation = results.joint_displacements["B"][4];
    assert!((rotation.abs() - per_length * length * length / (2.0 * ei)).abs() < 1e-12);
    for station in &results.frame_forces["F"] {
        let expected = per_length * (length - station.station);
        assert!(
            (station.m3.abs() - expected).abs() < 1e-9,
            "at {}: {} vs {expected}",
            station.station,
            station.m3
        );
    }
}

#[test]
fn a_buckling_case_buckles_under_its_own_scaled_load_patterns() {
    // Pinned-pinned column of eight elements, L = 120: Euler load
    // pi^2 E I / L^2. The case applies 3 x 1 kip + 0.5 x 2 kip = 4 kip, and
    // no load case shares a name with either pattern.
    let (length, ei, elements) = (120.0, 10_000.0 * 50.0, 8);
    let mut m = planar_model();
    for k in 0..=elements {
        m.add_point(format!("J{k}"), 0.0, 0.0, length * k as f64 / elements as f64);
    }
    for k in 0..elements {
        m.add_frame(format!("F{k}"), format!("J{k}"), format!("J{}", k + 1), "S");
    }
    m.points["J0"].restraint = [true, true, true, true, false, true];
    m.points[format!("J{elements}").as_str()].restraint = [true, true, false, true, false, true];
    for (name, force) in [("A", -1.0), ("B", -2.0)] {
        pattern(&mut m, name);
        m.points[format!("J{elements}").as_str()].loads_force.push(PointLoadForce {
            load_pat: name.into(),
            csys: "GLOBAL".into(),
            force: [0.0, 0.0, force, 0.0, 0.0, 0.0],
        });
    }
    let mut case = LoadCase::linear_static("BUCK", "A", 3.0);
    case.case_type = LoadCaseType::Buckling;
    case.loads.push(LoadCaseAssignment {
        load_name: "B".into(),
        scale_factor: 0.5,
    });
    case.num_buckling_modes = 2;
    m.load_cases.insert("BUCK".into(), case);
    analyze_model(&mut m).unwrap();
    let factors = &m.case_results["BUCK"].buckling_factors;
    let euler = std::f64::consts::PI.powi(2) * ei / (length * length);
    // Cubic elements overestimate; eight of them by a few parts in 1e5.
    assert!(factors[0] >= euler / 4.0 && factors[0] < euler / 4.0 * (1.0 + 1e-4), "{factors:?}");
    // Second mode: four times the first, within the coarser-mode error.
    assert!((factors[1] / factors[0] - 4.0).abs() < 4e-3, "{factors:?}");
}

#[test]
fn bending_moments_follow_the_sap2000_sign_convention_about_both_axes() {
    // CSI Analysis Reference, "Internal Force Output": a positive bending
    // moment compresses the positive 2 face (M3) or the positive 3 face (M2).
    // A cantilever pushed at its tip toward +local 2 is compressed on its
    // positive 2 face at the support, and likewise for local 3, so both root
    // moments are +F (L - x); and dM3/dx = -V2, dM2/dx = -V3.
    let (length, force) = (100.0, 3.0);
    for axis in [2usize, 3] {
        let mut m = planar_model();
        m.active_dofs = [true; 6];
        m.add_point("A", 0.0, 0.0, 0.0);
        m.add_point("B", length, 0.0, 0.0);
        m.points["A"].restraint = [true; 6];
        m.add_frame("F", "A", "B", "S");
        m.frames["F"].min_stations = 11;
        pattern(&mut m, "P");
        m.frames["F"].loads_point.push(FrameLoadPoint {
            load_pat: "P".into(),
            my_type: 1,
            dir: axis as i32,
            csys: "Local".into(),
            rel_dist: 1.0,
            val: force,
        });
        m.load_cases.insert("P".into(), LoadCase::linear_static("P", "P", 1.0));
        analyze_model(&mut m).unwrap();
        let stations = &m.case_results["P"].frame_forces["F"];
        for pair in stations.windows(2) {
            let (a, b) = (&pair[0], &pair[1]);
            let (moment_a, moment_b, shear) = if axis == 2 {
                (a.m3, b.m3, a.v2)
            } else {
                (a.m2, b.m2, a.v3)
            };
            let expected = force * (length - a.station);
            assert!((moment_a - expected).abs() < 1e-9, "axis {axis} at {}: {moment_a}", a.station);
            // The other plane carries nothing.
            let other = if axis == 2 { a.m2 } else { a.m3 };
            assert!(other.abs() < 1e-9);
            let slope = (moment_b - moment_a) / (b.station - a.station);
            assert!((slope + shear).abs() < 1e-9, "axis {axis}: dM/dx {slope} vs V {shear}");
            assert!((shear - force).abs() < 1e-9);
        }
        // The tip moves the way it is pushed.
        let tip = stations.last().unwrap().local_disp[axis - 1];
        assert!(tip > 0.0);
    }
}
