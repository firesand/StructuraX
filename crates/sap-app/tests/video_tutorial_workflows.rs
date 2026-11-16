//! The three models of the "SAP2000 Complete Mastery" tutorial video that the
//! program can build, driven end to end through the same commands the GUI
//! panels call: quick grid and quick draw, mass source, ASCE 7-10 automatic
//! seismic load, diaphragms, Ritz vectors with participation targets, default
//! design combinations, Auto Select design, the eigen solver at ten storeys,
//! modal time history, curved frames and radial extrusion.
//!
//! The video states almost no result that can serve as a reference (its
//! numbers are read off the screen for a model whose section sizes are not
//! given), so no value here is taken from it. Every check is statics, a
//! printed code equation evaluated in the test, a closed form, or a symmetry
//! the structure must have. Dimensions are those spoken in the video; the
//! section properties are standard European table values typed below.

use sap_design::auto_select::{verify_all_members_passed, verify_analysis_vs_design};
use sap_design::default_combos::{add_default_design_combos, DefaultComboOptions, DesignMethod};
use sap_design::try_run_steel_design;
use sap_model::generate::GridRegion;
use sap_model::radial::CurveKind;
use sap_model::ritz::RitzLoadControl;
use sap_model::seismic::{AutoSeismicAsce710, SeismicDirection, SeismicPeriod};
use sap_model::*;
use sap_solver::analyze_model;
use sap_units::Units;

/// name, h, b, tw, tf (mm), A (cm2), Iy (cm4), Wel,y, Wpl,y (cm3), Iz (cm4),
/// Wel,z, Wpl,z (cm3), It (cm4): European section tables.
const SHAPES: [(&str, [f64; 12]); 4] = [
    ("IPE270", [270., 135., 6.6, 10.2, 45.9, 5790., 429., 484., 420., 62.2, 97.0, 15.9]),
    ("IPE330", [330., 160., 7.5, 11.5, 62.6, 11770., 713., 804., 788., 98.5, 154., 28.1]),
    ("IPE400", [400., 180., 8.6, 13.5, 84.5, 23130., 1160., 1307., 1320., 146., 229., 51.1]),
    ("HE300B", [300., 300., 11., 19., 149., 25170., 1680., 1869., 8560., 571., 870., 185.]),
];

/// Section in kN-m units.
fn shape(name: &str) -> FrameSection {
    let v = SHAPES.iter().find(|s| s.0 == name).unwrap().1;
    let (mm, cm2, cm3, cm4) = (1e-3, 1e-4, 1e-6, 1e-8);
    let (area, i33, i22) = (v[4] * cm2, v[5] * cm4, v[8] * cm4);
    FrameSection {
        name: name.into(),
        material: "A992Fy50".into(),
        prop_type: FramePropType::I,
        t3: v[0] * mm,
        t2: v[1] * mm,
        tw: v[2] * mm,
        tf: v[3] * mm,
        t2b: v[1] * mm,
        tfb: v[3] * mm,
        area,
        i33,
        i22,
        s33: v[6] * cm3,
        z33: v[7] * cm3,
        s22: v[9] * cm3,
        z22: v[10] * cm3,
        r33: (i33 / area).sqrt(),
        r22: (i22 / area).sqrt(),
        torsional_j: v[11] * cm4,
        // Shear deformation off: the checks below are statics and symmetry.
        as2: 0.0,
        as3: 0.0,
        ..FrameSection::default()
    }
}

fn pattern(m: &mut SapModel, name: &str, kind: LoadPatternType, self_weight: f64) {
    m.load_patterns.insert(
        name.into(),
        LoadPattern { name: name.into(), pat_type: kind, self_wt_multiplier: self_weight },
    );
    m.load_cases.insert(name.into(), LoadCase::linear_static(name, name, 1.0));
}

fn empty(units: Units) -> SapModel {
    let mut m = SapModel::new(units);
    m.load_cases.clear();
    m.load_patterns.clear();
    m.frame_sections.clear();
    for (name, _) in SHAPES {
        m.frame_sections.insert(name.into(), shape(name));
    }
    m
}

fn fix_base(m: &mut SapModel) {
    for p in m.points.values_mut().filter(|p| p.z.abs() < 1e-9) {
        p.restraint = [true; 6];
    }
}

fn close(actual: f64, expected: f64, tol: f64, what: &str) {
    assert!(
        (actual - expected).abs() <= tol * expected.abs().max(1e-300),
        "{what}: {actual} vs {expected}"
    );
}

/// Part 1 of the video: six storeys of 3.2 m, five 6 m bays in X and three
/// 5 m bays in Y, 15 cm slab, superimposed dead 0.8 kN/m2, live 2.1 kN/m2
/// (roof 1.0), Ss = 1.23 g, S1 = 0.82 g, R = 7, Omega = 2.5, Cd = 5.5, I = 1.
#[test]
fn six_storey_steel_building_from_grid_to_auto_select_design() {
    const NX: usize = 6;
    const NY: usize = 4;
    const NZ: usize = 7;
    let (dx, dy, dz) = (6.0, 5.0, 3.2);
    let mut m = empty(Units::KnMC);
    m.quick_grid([NX, NY, NZ], [dx, dy, dz], [0.0; 3]).unwrap();

    // Beams are an Auto Select list of three IPE shapes; columns one shape.
    m.define_auto_select_list(
        "BEAMS",
        &["IPE270".to_string(), "IPE330".to_string(), "IPE400".to_string()],
    )
    .unwrap();
    let mut slab = AreaSection { name: "SLAB".into(), ..AreaSection::default() };
    slab.thickness_membrane = 0.15;
    slab.thickness_bending = 0.15;
    m.area_sections.insert("SLAB".into(), slab);

    let all = GridRegion { x: (0, NX - 1), y: (0, NY - 1), z: (0, NZ - 1) };
    let floors = GridRegion { z: (1, NZ - 1), ..all };
    let columns = m.quick_draw_columns(all, "HE300B").unwrap();
    assert_eq!(columns.frames_created.len(), NX * NY * (NZ - 1));
    let beams = m.quick_draw_beams(floors, true, true, "IPE330").unwrap();
    let per_level = NY * (NX - 1) + NX * (NY - 1);
    assert_eq!(beams.frames_created.len(), per_level * (NZ - 1));
    let slabs = m.quick_draw_areas(floors, "SLAB").unwrap();
    assert_eq!(slabs.areas_created.len(), (NX - 1) * (NY - 1) * (NZ - 1));
    assert_eq!(m.points.len(), NX * NY * NZ);
    m.assign_auto_select_list("BEAMS", &beams.frames_created).unwrap();
    fix_base(&mut m);

    pattern(&mut m, "DEAD", LoadPatternType::Dead, 1.0);
    pattern(&mut m, "SDEAD", LoadPatternType::SuperDead, 0.0);
    pattern(&mut m, "LIVE", LoadPatternType::Live, 0.0);
    pattern(&mut m, "EX", LoadPatternType::Quake, 0.0);
    let roof = dz * (NZ - 1) as f64;
    let (sdl, live, roof_live) = (0.8, 2.1, 1.0);
    for area in m.areas.values_mut() {
        area.loads_uniform.push(AreaLoadUniform {
            load_pat: "SDEAD".into(),
            csys: "GLOBAL".into(),
            dir: 10,
            val: sdl,
        });
    }
    let names: Vec<String> = m.areas.keys().cloned().collect();
    for name in names {
        let z = m.points[&m.areas[&name].points[0]].z;
        let val = if (z - roof).abs() < 1e-9 { roof_live } else { live };
        m.areas[&name].loads_uniform.push(AreaLoadUniform {
            load_pat: "LIVE".into(),
            csys: "GLOBAL".into(),
            dir: 10,
            val,
        });
    }

    // Mass from the load patterns only: dead and superimposed dead in full,
    // a quarter of the live load.
    m.mass_source.element_self_mass = false;
    m.mass_source.additional_mass = false;
    m.mass_source.load_patterns =
        vec![("DEAD".into(), 1.0), ("SDEAD".into(), 1.0), ("LIVE".into(), 0.25)];

    // ASCE 7-10 load in X. Fa and Fv are entered (site class D values for
    // these accelerations are about 1.0 and 1.5); steel moment frame period
    // coefficients Ct = 0.028, x = 0.8.
    let (ss, s1, fa, fv, r, ie, tl) = (1.23, 0.82, 1.0, 1.5, 7.0, 1.0, 8.0);
    let (ct, x) = (0.028, 0.8);
    m.auto_seismic.insert(
        "EX".into(),
        AutoSeismicAsce710 {
            direction: SeismicDirection::X,
            eccentricity_ratio: 0.0,
            period: SeismicPeriod::Approximate { ct, x },
            r,
            omega0: 2.5,
            cd: 5.5,
            ie,
            ss,
            s1,
            tl,
            fa: Some(fa),
            fv: Some(fv),
            z_min: None,
            z_max: None,
        },
    );

    // One rigid diaphragm per floor.
    for p in m.points.values_mut() {
        p.selected = p.z > 1e-9;
    }
    let diaphragms = m.assign_diaphragm("DIAPH", true).unwrap();
    assert_eq!(diaphragms.len(), NZ - 1);
    for p in m.points.values_mut() {
        p.selected = false;
    }

    // Ritz vectors from the X and Y accelerations with 99 % targets.
    let mut modal = LoadCase::modal_eigen("MODAL", 12);
    modal.modal_sub_type = ModalSubType::Ritz;
    for load in ["UX", "UY"] {
        modal.loads.push(LoadCaseAssignment { load_name: load.into(), scale_factor: 1.0 });
        modal.ritz_controls.push(RitzLoadControl {
            load_name: load.into(),
            max_cycles: 0,
            target_dynamic_participation: 0.99,
        });
    }
    m.load_cases.insert("MODAL".into(), modal);

    let sds = 2.0 / 3.0 * fa * ss;
    let mut options = DefaultComboOptions::new(DesignMethod::Lrfd);
    options.sds = sds;
    let combos = add_default_design_combos(&mut m, &options).unwrap();
    assert!(!combos.created.is_empty());

    analyze_model(&mut m).unwrap();

    // --- Weights by hand, kN ------------------------------------------------
    let gamma_s = m.materials["A992Fy50"].unit_weight;
    let gamma_c = m.materials["4000Psi"].unit_weight;
    let a_col = m.frame_sections["HE300B"].area;
    // Beams are analysed with the median of the list by weight, IPE330.
    let a_beam = m.frame_sections["IPE330"].area;
    let storeys = (NZ - 1) as f64;
    let column_length = (NX * NY) as f64 * storeys * dz;
    let beam_length =
        ((NY * (NX - 1)) as f64 * dx + (NX * (NY - 1)) as f64 * dy) * storeys;
    let plan = (NX - 1) as f64 * dx * (NY - 1) as f64 * dy;
    let dead = gamma_s * (a_col * column_length + a_beam * beam_length)
        + gamma_c * 0.15 * plan * storeys;
    let sdead = sdl * plan * storeys;
    let live_total = live * plan * (storeys - 1.0) + roof_live * plan;
    for (case, weight) in [("DEAD", dead), ("SDEAD", sdead), ("LIVE", live_total)] {
        close(m.case_results[case].base_reactions[2], weight, 1e-9, case);
    }

    // --- Automatic seismic load against the printed equations ---------------
    // Half of each first-storey column hangs on a base joint, which is not
    // above the base and so is not part of W.
    let base_part = gamma_s * a_col * (NX * NY) as f64 * dz / 2.0;
    let weight = dead + sdead + 0.25 * live_total - base_part;
    let hn_ft = roof / 0.3048;
    let ta = ct * hn_ft.powf(x);
    let sd1 = 2.0 / 3.0 * fv * s1;
    assert!(ta <= tl);
    let cs_upper: f64 = sd1 / (ta * (r / ie)); // 12.8-3
    let cs_floor = (0.044 * sds * ie).max(0.01).max(0.5 * s1 / (r / ie)); // 12.8-5, 12.8-6
    let cs = (sds / (r / ie)).min(cs_upper).max(cs_floor);
    let k = 1.0 + (ta - 0.5) / 2.0;
    assert!(ta > 0.5 && ta < 2.5);
    let report = &m.case_results["EX"].auto_lateral[0];
    close(report.period, ta, 1e-12, "Ta");
    close(report.sds, sds, 1e-12, "SDS");
    close(report.sd1, sd1, 1e-12, "SD1");
    close(report.k, k, 1e-12, "k");
    close(report.cs, cs, 1e-12, "Cs");
    close(report.weight, weight, 1e-9, "W");
    close(report.base_shear, cs * weight, 1e-9, "V");
    // The diaphragms are penalty constraints, which hold a floor together to
    // about one part in 1e5 of the storey force.
    close(m.case_results["EX"].base_reactions[0], -cs * weight, 1e-4, "sum Rx");
    // Storey forces follow w h^k: every level carries the same weight except
    // the roof, so the ratio of two typical floors is (h2 / h1)^k.
    let level_force = |level: usize| -> f64 {
        report
            .joint_forces
            .iter()
            .filter(|j| (j.z - dz * level as f64).abs() < 1e-9)
            .map(|j| j.force)
            .sum()
    };
    close(level_force(4) / level_force(2), 2f64.powf(k), 1e-9, "F4 / F2");
    assert!(m.case_results["DEAD"].auto_lateral.is_empty());

    // --- Diaphragms: a floor moves as one body. The building and the load
    // are symmetric about the X axis through mid-plan, so it does not twist.
    let ex = &m.case_results["EX"].joint_displacements;
    for level in 1..NZ {
        let ux: Vec<f64> = m
            .points
            .values()
            .filter(|p| (p.z - dz * level as f64).abs() < 1e-9)
            .map(|p| ex[&p.name][0])
            .collect();
        let mean = ux.iter().sum::<f64>() / ux.len() as f64;
        assert!(mean > 0.0);
        for u in ux {
            close(u, mean, 1e-4, "floor UX");
        }
    }

    // --- Ritz vectors reached both targets, or used all twelve vectors -------
    let modal = &m.case_results["MODAL"];
    assert!(!modal.modes.is_empty() && modal.modes.len() <= 12);
    assert!(modal.modes.windows(2).all(|w| w[0].period_sec >= w[1].period_sec));
    // Only the starting loads carry a target; the table also lists the other
    // accelerations (UX..RZ), whose participation the Ritz vectors do not aim at.
    let starting = m.load_cases["MODAL"].loads.len();
    for part in modal.load_participation.iter().take(starting) {
        let reached = *part.dynamic_cumulative.last().unwrap();
        assert!(reached <= 1.0 + 1e-9);
        assert!(reached >= 0.99 || modal.modes.len() == 12, "{}: {reached}", part.load_name);
    }

    // --- Auto Select design, iterated until analysis and design agree --------
    assert!(m.case_results.contains_key("DSTL1"));
    let list = ["IPE270", "IPE330", "IPE400"];
    let mut rounds = 0;
    loop {
        try_run_steel_design(&mut m).unwrap();
        let differing = verify_analysis_vs_design(&m).unwrap();
        if differing.is_empty() {
            break;
        }
        rounds += 1;
        assert!(rounds <= 8, "analysis and design sections did not converge");
        analyze_model(&mut m).unwrap();
    }
    let limit = m.steel_design_prefs.dc_ratio_limit;
    let failed = verify_all_members_passed(&m).unwrap();
    for name in &beams.frames_created {
        let state = m.frames[name].auto_select.as_ref().unwrap();
        assert!(list.contains(&state.design.as_str()));
        assert_eq!(state.analysis.as_deref(), Some(state.design.as_str()));
        let result = &m.steel_design_results[name];
        assert_eq!(result.section_name, state.design);
        if failed.contains(name) {
            // Nothing in the list passes: the heaviest member is kept.
            assert_eq!(state.design, "IPE400");
        } else {
            assert!(result.total_ratio <= limit);
        }
    }

    // --- Save as S2K and reopen: the same model, the same analysis ----------
    let mut reopened = sap_format::parse_s2k(&sap_format::write_s2k(&m)).unwrap();
    assert_eq!(reopened.mass_source, m.mass_source);
    assert_eq!(reopened.auto_seismic, m.auto_seismic);
    assert_eq!(reopened.load_cases["MODAL"].ritz_controls, m.load_cases["MODAL"].ritz_controls);
    for name in &beams.frames_created {
        assert_eq!(reopened.frames[name].auto_select, m.frames[name].auto_select);
    }
    analyze_model(&mut reopened).unwrap();
    for case in ["DEAD", "EX", "DSTL1"] {
        for component in 0..3 {
            let (a, b) = (
                reopened.case_results[case].base_reactions[component],
                m.case_results[case].base_reactions[component],
            );
            assert!((a - b).abs() <= 1e-9 * dead, "{case} reaction {component}: {a} vs {b}");
        }
    }
    let (a, b) = (
        &reopened.case_results["EX"].auto_lateral[0],
        &m.case_results["EX"].auto_lateral[0],
    );
    close(a.base_shear, b.base_shear, 1e-12, "base shear after reopening");
    close(
        reopened.case_results["MODAL"].modes[0].period_sec,
        m.case_results["MODAL"].modes[0].period_sec,
        1e-9,
        "first period after reopening",
    );
    try_run_steel_design(&mut reopened).unwrap();
    assert!(verify_analysis_vs_design(&reopened).unwrap().is_empty());
}

/// Part 3 of the video: ten storeys of 3 m, five 6 m bays each way, fixed
/// bases, a ground acceleration history solved by mode superposition. With
/// 1080 mass degrees of freedom the 20 eigenvectors come from subspace
/// iteration.
#[test]
fn ten_storey_frame_modal_time_history_settles_on_the_static_response() {
    let (n, nz, bay, storey) = (6usize, 11usize, 6.0, 3.0);
    let mut m = empty(Units::KnMC);
    m.quick_grid([n, n, nz], [bay, bay, storey], [0.0; 3]).unwrap();
    let all = GridRegion { x: (0, n - 1), y: (0, n - 1), z: (0, nz - 1) };
    m.quick_draw_columns(all, "HE300B").unwrap();
    m.quick_draw_beams(GridRegion { z: (1, nz - 1), ..all }, true, true, "IPE400").unwrap();
    fix_base(&mut m);
    assert_eq!(m.points.len(), n * n * nz);

    // A constant ground acceleration a0 in X, applied as a step. Its static
    // equivalent is the force -m a0 on every joint, with the joint mass
    // lumped here by hand: half of each connected member.
    let a0 = 1.0; // m/s2
    pattern(&mut m, "PUSH", LoadPatternType::Other, 0.0);
    let density = m.materials["A992Fy50"].unit_mass;
    let mut joint_mass = std::collections::BTreeMap::<String, f64>::new();
    for f in m.frames.values() {
        let (i, j) = (&m.points[&f.point_i], &m.points[&f.point_j]);
        let length = ((j.x - i.x).powi(2) + (j.y - i.y).powi(2) + (j.z - i.z).powi(2)).sqrt();
        let half = 0.5 * density * m.frame_sections[&f.section].area * length;
        *joint_mass.entry(f.point_i.clone()).or_default() += half;
        *joint_mass.entry(f.point_j.clone()).or_default() += half;
    }
    for (joint, mass) in &joint_mass {
        m.points[joint].loads_force.push(PointLoadForce {
            load_pat: "PUSH".into(),
            csys: "GLOBAL".into(),
            force: [-mass * a0, 0.0, 0.0, 0.0, 0.0, 0.0],
        });
    }
    m.functions.insert(
        "STEP".into(),
        DynamicFunction {
            name: "STEP".into(),
            is_time_history: true,
            damping_ratio: 0.0,
            points: vec![(0.0, 1.0), (1.0, 1.0)],
        },
    );
    m.load_cases.insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 20));
    let mut ritz = LoadCase::modal_eigen("RITZ", 20);
    ritz.modal_sub_type = ModalSubType::Ritz;
    for load in ["UX", "UY"] {
        ritz.loads.push(LoadCaseAssignment { load_name: load.into(), scale_factor: 1.0 });
    }
    m.load_cases.insert("RITZ".into(), ritz);
    for (name, modal) in [("HIST_RITZ", "RITZ"), ("HIST_EIGEN", "MODAL")] {
        let mut history = LoadCase::linear_static(name, "PUSH", 1.0);
        history.case_type = LoadCaseType::LinearHistory;
        history.loads.clear();
        history.modal_case = Some(modal.into());
        history.modal_damping = 0.6;
        history.rayleigh_damping = (0.0, 0.0);
        history.history_loads = vec![("STEP".into(), "UX".into(), a0)];
        history.time_step_dt = 0.05;
        history.num_time_steps = 1200;
        m.load_cases.insert(name.into(), history);
    }

    analyze_model(&mut m).unwrap();

    let eigen = &m.case_results["MODAL"].modes;
    let ritz = &m.case_results["RITZ"].modes;
    assert_eq!((eigen.len(), ritz.len()), (20, 20));
    assert!(eigen.windows(2).all(|w| w[0].period_sec >= w[1].period_sec * (1.0 - 1e-12)));
    // Rayleigh-Ritz: a Ritz frequency is never below the exact one, and the
    // fundamental one, started from the acceleration loads, is close to it.
    assert!(ritz[0].period_sec <= eigen[0].period_sec * (1.0 + 1e-9));
    close(ritz[0].period_sec, eigen[0].period_sec, 1e-3, "fundamental period");

    let roof = roof_corner(&m, (nz - 1) as f64 * storey);
    let static_ux = m.case_results["PUSH"].joint_displacements[&roof][0];
    assert!(static_ux < 0.0);

    // The first Ritz vector of UX is the static response to this very load,
    // so the heavily damped history settles on the static displacement.
    let trace = &m.case_results["HIST_RITZ"].time_history_disp[&roof];
    assert_eq!(trace.len(), 1201);
    close(trace.last().unwrap()[0], static_ux, 1e-6, "settled roof UX, Ritz vectors");
    // On the way it overshoots less than an undamped system would (factor 2).
    let extreme = trace.iter().map(|u| u[0]).fold(0.0f64, f64::min);
    assert!(extreme >= 2.0 * static_ux && extreme <= static_ux * (1.0 - 1e-9));
    // Relative acceleration vanishes once the motion has settled.
    let accel = m.case_results["HIST_RITZ"].time_history_accel[&roof].last().unwrap()[0];
    assert!(accel.abs() < 1e-6 * a0, "{accel}");

    // Twenty eigenvectors of this frame are mostly the first sway and twist
    // shapes of its six parallel frames, so they carry the static sway only
    // approximately: the first sway shape alone overshoots a uniform load by
    // a few percent (32 / pi^3 = 1.032 for a uniform shear beam).
    let truncated = m.case_results["HIST_EIGEN"].time_history_disp[&roof].last().unwrap()[0];
    close(truncated, static_ux, 0.05, "settled roof UX, 20 eigenvectors");
}

fn roof_corner(m: &SapModel, z: f64) -> String {
    m.points
        .values()
        .find(|p| (p.z - z).abs() < 1e-9 && p.x.abs() < 1e-9 && p.y.abs() < 1e-9)
        .map(|p| p.name.clone())
        .unwrap()
}

/// Part 4 of the video: a shell of revolution made by drawing a curved
/// meridian and extruding it radially in 36 steps of 10 degrees.
#[test]
fn cooling_tower_shell_of_revolution_has_paired_modes() {
    let mut m = empty(Units::KnMC);
    let mut wall = AreaSection { name: "WALL".into(), ..AreaSection::default() };
    wall.thickness_membrane = 0.8;
    wall.thickness_bending = 0.8;
    m.area_sections.insert("WALL".into(), wall);

    // Meridian in the XZ plane: base radius 58.5 m, top radius 34 m at 162 m,
    // through a waist point; eight straight frames.
    let segments = 8;
    let meridian = m
        .curved_frame(
            [58.5, 0.0, 0.0],
            [34.0, 0.0, 162.0],
            [36.0, 0.0, 110.0],
            CurveKind::Parabolic,
            segments,
            "IPE270",
        )
        .unwrap();
    assert_eq!(meridian.frames_created.len(), segments);
    for f in m.frames.values_mut() {
        f.selected = true;
    }
    let steps = 36;
    let shell = m
        .extrude_lines_to_areas_radial([0.0, 0.0], 10.0, steps, 0.0, "WALL", true)
        .unwrap();
    assert_eq!(shell.areas_created.len(), segments * steps);
    assert!(m.frames.is_empty());
    // The 36th step closes on the meridian: one joint per ring and step.
    assert_eq!(m.points.len(), (segments + 1) * steps);
    for p in m.points.values() {
        // Every joint is on the surface of revolution of the meridian.
        let radius = p.x.hypot(p.y);
        assert!((34.0 - 1e-9..=58.5 + 1e-9).contains(&radius), "{radius}");
    }
    fix_base(&mut m);
    assert_eq!(m.points.values().filter(|p| p.restraint[0]).count(), steps);

    let mut modal = LoadCase::modal_eigen("MODAL", 10);
    modal.modal_sub_type = ModalSubType::Ritz;
    for load in ["UX", "UY"] {
        modal.loads.push(LoadCaseAssignment { load_name: load.into(), scale_factor: 1.0 });
    }
    m.load_cases.insert("MODAL".into(), modal);
    analyze_model(&mut m).unwrap();

    // The shell has 36-fold symmetry and X and Y are equivalent directions
    // (a quarter turn maps the mesh onto itself), so the Ritz vectors started
    // from UX and UY come in pairs of equal period, and each direction
    // collects the same participating mass.
    let modes = &m.case_results["MODAL"].modes;
    assert_eq!(modes.len(), 10);
    for pair in modes.chunks(2) {
        close(pair[0].period_sec, pair[1].period_sec, 1e-6, "paired periods");
    }
    let sum = |d: usize| modes.iter().map(|mode| mode.modal_mass_ratios[d]).sum::<f64>();
    close(sum(0), sum(1), 1e-6, "UX and UY participating mass");
    assert!(sum(0) > 0.5 && sum(0) <= 1.0 + 1e-9, "{}", sum(0));
    assert!(sum(2) < 1e-9, "no vertical participation from lateral starting loads");
}
