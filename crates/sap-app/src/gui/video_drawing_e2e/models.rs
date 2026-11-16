//! Models of the video, retyped from `tests/video_tutorial_workflows.rs` and
//! `tests/video_pushover_workflow.rs` (integration tests cannot be imported).

use sap_model::hinge::*;
use sap_model::ritz::RitzLoadControl;
use sap_model::seismic::{AutoSeismicAsce710, SeismicDirection, SeismicPeriod};
use sap_model::*;
use sap_units::Units;

/// name, h, b, tw, tf (mm), A (cm2), Iy (cm4), Wel,y, Wpl,y (cm3), Iz (cm4),
/// Wel,z, Wpl,z (cm3), It (cm4): European section tables.
const SHAPES: [(&str, [f64; 12]); 3] = [
    ("IPE270", [270., 135., 6.6, 10.2, 45.9, 5790., 429., 484., 420., 62.2, 97.0, 15.9]),
    ("IPE330", [330., 160., 7.5, 11.5, 62.6, 11770., 713., 804., 788., 98.5, 154., 28.1]),
    ("HE300B", [300., 300., 11., 19., 149., 25170., 1680., 1869., 8560., 571., 870., 185.]),
];

/// Section in kN-m units, shear deformation off (as the wave-1 test).
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
        as2: 0.0,
        as3: 0.0,
        ..FrameSection::default()
    }
}

pub const NX: usize = 6;
pub const NY: usize = 4;
pub const NZ: usize = 7;
pub const DX: f64 = 6.0;
pub const DY: f64 = 5.0;
pub const DZ: f64 = 3.2;

/// The grid, the three shapes and the slab section; no objects yet.
pub fn building_base() -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    m.load_cases.clear();
    m.load_patterns.clear();
    m.frame_sections.clear();
    for (name, _) in SHAPES {
        m.frame_sections.insert(name.into(), shape(name));
    }
    let mut slab = AreaSection { name: "SLAB".into(), ..AreaSection::default() };
    slab.thickness_membrane = 0.15;
    slab.thickness_bending = 0.15;
    m.area_sections.insert("SLAB".into(), slab);
    m.quick_grid([NX, NY, NZ], [DX, DY, DZ], [0.0; 3]).unwrap();
    m
}

fn pattern(m: &mut SapModel, name: &str, kind: LoadPatternType, self_weight: f64) {
    m.load_patterns.insert(
        name.into(),
        LoadPattern { name: name.into(), pat_type: kind, self_wt_multiplier: self_weight },
    );
    m.load_cases.insert(name.into(), LoadCase::linear_static(name, name, 1.0));
}

/// The loads, mass source and cases of the wave-1 building (no diaphragms:
/// they are penalty constraints and would cost digits in the comparison of
/// two joint orders), plus an eigen case.
pub fn building_loads_and_cases(m: &mut SapModel) {
    pattern(m, "DEAD", LoadPatternType::Dead, 1.0);
    pattern(m, "SDEAD", LoadPatternType::SuperDead, 0.0);
    pattern(m, "LIVE", LoadPatternType::Live, 0.0);
    pattern(m, "EX", LoadPatternType::Quake, 0.0);
    let roof = DZ * (NZ - 1) as f64;
    let (sdl, live, roof_live) = (0.8, 2.1, 1.0);
    let names: Vec<String> = m.areas.keys().cloned().collect();
    for name in names {
        let z = m.points[&m.areas[&name].points[0]].z;
        let area = &mut m.areas[&name];
        area.loads_uniform.push(AreaLoadUniform {
            load_pat: "SDEAD".into(),
            csys: "GLOBAL".into(),
            dir: 10,
            val: sdl,
        });
        let val = if (z - roof).abs() < 1e-9 { roof_live } else { live };
        area.loads_uniform.push(AreaLoadUniform {
            load_pat: "LIVE".into(),
            csys: "GLOBAL".into(),
            dir: 10,
            val,
        });
    }
    m.mass_source.element_self_mass = false;
    m.mass_source.additional_mass = false;
    m.mass_source.load_patterns =
        vec![("DEAD".into(), 1.0), ("SDEAD".into(), 1.0), ("LIVE".into(), 0.25)];
    m.auto_seismic.insert(
        "EX".into(),
        AutoSeismicAsce710 {
            direction: SeismicDirection::X,
            eccentricity_ratio: 0.0,
            period: SeismicPeriod::Approximate { ct: 0.028, x: 0.8 },
            r: 7.0,
            omega0: 2.5,
            cd: 5.5,
            ie: 1.0,
            ss: 1.23,
            s1: 0.82,
            tl: 8.0,
            fa: Some(1.0),
            fv: Some(1.5),
            z_min: None,
            z_max: None,
        },
    );
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
    m.load_cases.insert("EIGEN".into(), LoadCase::modal_eigen("EIGEN", 12));
}

// ---- part 2: the pushover frame of the wave-2 test --------------------------

pub const XS: [f64; 4] = [0.0, 4.0, 10.0, 14.0];
pub const STOREY_H: f64 = 3.0;
pub const STOREYS: usize = 4;
const E_MOD: f64 = 24_855_578.0;
const GAMMA: f64 = 23.5631;
const G_ACC: f64 = 9.80665;
const SDL: f64 = 15.0;
const LIVE: f64 = 10.0;
const COL_AREA: f64 = 0.09;
const BEAM_AREA: f64 = 0.12;
const MP_POS: f64 = 120.0;
const MP_NEG: f64 = 160.0;
const MP_COL: f64 = 300.0;
const TARGET: f64 = 1.2;

/// The geometry the wave-2 test builds: joints `J{i}{k}`, columns `C{i}{k}`,
/// beams `B{i}{k}`, fixed bases, no section data yet.
pub fn wave2_geometry() -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
    for (i, x) in XS.iter().enumerate() {
        for k in 0..=STOREYS {
            let name = m.add_point(format!("J{i}{k}"), *x, 0.0, STOREY_H * k as f64);
            if k == 0 {
                m.points[&name].restraint = [true; 6];
            }
        }
    }
    for i in 0..XS.len() {
        for k in 0..STOREYS {
            m.add_frame(format!("C{i}{k}"), format!("J{i}{k}"), format!("J{i}{}", k + 1), "COL");
        }
    }
    for k in 1..=STOREYS {
        for i in 0..XS.len() - 1 {
            m.add_frame(format!("B{i}{k}"), format!("J{i}{k}"), format!("J{}{k}", i + 1), "BEAM");
        }
    }
    m
}

fn epp_curve() -> [HingePoint; 4] {
    [
        HingePoint::new(1.0, 0.0),
        HingePoint::new(1.0, 0.25),
        HingePoint::new(1.0, 0.35),
        HingePoint::new(1.0, 0.5),
    ]
}

fn beam_hinge_prop() -> HingeProp {
    let prop = HingeProp {
        name: "BEAM-M3".into(),
        dof: HingeDof::M3,
        behavior: HingeBehavior::DeformationControlled,
        symmetric: false,
        positive: epp_curve(),
        negative: epp_curve().map(|p| HingePoint::new(-p.force, -p.deformation)),
        force_sf: MP_POS,
        deformation_sf: 1.0,
        force_sf_negative: MP_NEG,
        deformation_sf_negative: 1.0,
        acceptance_positive: HingeAcceptance::default(),
        acceptance_negative: HingeAcceptance::default(),
        beyond_e: Some(HingeBeyondE::DropsToZero),
    };
    prop.validate().unwrap();
    prop
}

fn column_hinge_prop() -> HingeProp {
    let mut prop = HingeProp::symmetric("COL-M3", HingeDof::M3, epp_curve());
    prop.force_sf = MP_COL;
    prop.force_sf_negative = MP_COL;
    prop.validate().unwrap();
    prop
}

fn rectangle(name: &str, b: f64, d: f64, area: f64) -> FrameSection {
    FrameSection {
        name: name.into(),
        material: "CONC".into(),
        prop_type: FramePropType::General,
        area,
        i33: b * d.powi(3) / 12.0,
        i22: d * b.powi(3) / 12.0,
        torsional_j: b * d.powi(3) / 12.0 + d * b.powi(3) / 12.0,
        as2: area * 5.0 / 6.0,
        as3: area * 5.0 / 6.0,
        modifiers: [1.0; 8],
        ..FrameSection::default()
    }
}

fn wave2_pattern(m: &mut SapModel, name: &str, kind: LoadPatternType, self_weight: f64) {
    m.load_patterns.insert(
        name.into(),
        LoadPattern { name: name.into(), pat_type: kind, self_wt_multiplier: self_weight },
    );
}

fn nl_case(name: &str, loads: &str, sf: f64, initial: Option<&str>) -> LoadCase {
    let mut c = LoadCase::linear_static(name, loads, sf);
    c.case_type = LoadCaseType::NonlinearStatic;
    c.initial_case = initial.map(str::to_string);
    c
}

/// Gives a frame geometry (columns and beams recognized by their end
/// coordinates, whatever the names) the data of the wave-2 model: the
/// concrete, the two sections, the hinges at both ends, SDL and LIVE on the
/// beams, LAT on the left column line, the mass source, the Ritz case, the
/// NL-DEAD, NL-SDL, NL-LIVE chain and the Accel push `PUSH-ACC`. Returns the
/// name of the roof joint on the left column line (the monitored one).
pub fn wave2_data(m: &mut SapModel) -> String {
    m.load_cases.clear();
    m.load_patterns.clear();
    m.frame_sections.clear();
    m.materials.clear();
    m.active_dofs = [true, false, true, false, true, false];
    let mut conc = Material::default_concrete_kip_in();
    conc.name = "CONC".into();
    conc.e = E_MOD;
    conc.u = 0.2;
    conc.g = E_MOD / 2.4;
    conc.unit_weight = GAMMA;
    conc.unit_mass = GAMMA / G_ACC;
    m.materials.insert("CONC".into(), conc);
    m.frame_sections.insert("COL".into(), rectangle("COL", 0.3, 0.3, COL_AREA));
    m.frame_sections.insert("BEAM".into(), rectangle("BEAM", 0.3, 0.4, BEAM_AREA));
    wave2_pattern(m, "DEAD", LoadPatternType::Dead, 1.0);
    wave2_pattern(m, "SDL", LoadPatternType::SuperDead, 0.0);
    wave2_pattern(m, "LIVE", LoadPatternType::Live, 0.0);
    wave2_pattern(m, "LAT", LoadPatternType::Quake, 0.0);
    m.hinge_props.insert("BEAM-M3".into(), beam_hinge_prop());
    m.hinge_props.insert("COL-M3".into(), column_hinge_prop());
    let hinge = |prop: &str, rel: f64| FrameHinge { prop: prop.into(), rel_dist: rel };
    let names: Vec<String> = m.frames.keys().cloned().collect();
    for name in names {
        let (zi, zj) = {
            let f = &m.frames[&name];
            (m.points[&f.point_i].z, m.points[&f.point_j].z)
        };
        let f = &mut m.frames[&name];
        if (zi - zj).abs() > 1e-9 {
            f.section = "COL".into();
            f.hinges = vec![hinge("COL-M3", 0.0), hinge("COL-M3", 1.0)];
        } else {
            f.section = "BEAM".into();
            f.hinges = vec![hinge("BEAM-M3", 0.0), hinge("BEAM-M3", 1.0)];
            for (pat, value) in [("SDL", SDL), ("LIVE", LIVE)] {
                f.loads_dist.push(FrameLoadDistributed {
                    load_pat: pat.into(),
                    my_type: 1,
                    dir: 10,
                    csys: "GLOBAL".into(),
                    rel_dist1: 0.0,
                    rel_dist2: 1.0,
                    val1: value,
                    val2: value,
                });
            }
        }
    }
    let mut roof = String::new();
    for p in m.points.values_mut() {
        if p.x.abs() < 1e-9 && p.z > 1e-9 {
            p.loads_force.push(PointLoadForce {
                load_pat: "LAT".into(),
                csys: "GLOBAL".into(),
                force: [50.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            });
            if (p.z - STOREY_H * STOREYS as f64).abs() < 1e-9 {
                roof = p.name.clone();
            }
        }
    }
    assert!(!roof.is_empty());
    m.mass_source.element_self_mass = false;
    m.mass_source.additional_mass = false;
    m.mass_source.load_patterns =
        vec![("DEAD".into(), 1.0), ("SDL".into(), 1.0), ("LIVE".into(), 0.25)];
    let mut modal = LoadCase::modal_eigen("MODAL", 8);
    modal.modal_sub_type = ModalSubType::Ritz;
    for load in ["UX", "UZ"] {
        modal.loads.push(LoadCaseAssignment { load_name: load.into(), scale_factor: 1.0 });
        modal.ritz_controls.push(RitzLoadControl {
            load_name: load.into(),
            max_cycles: 0,
            target_dynamic_participation: 0.95,
        });
    }
    let mut push = nl_case("PUSH-ACC", "UX", -1.0, Some("NL-LIVE"));
    push.displacement_control = Some(DisplacementControl {
        target: TARGET,
        monitored: Some((roof.clone(), 0)),
        conjugate: false,
    });
    push.nl_static.results_saved =
        ResultsSaved::MultipleStates { min_saved: 10, max_saved: 100, positive_only: true };
    push.nl_static.max_total_steps = 1000;
    for case in [
        modal,
        nl_case("NL-DEAD", "DEAD", 1.0, None),
        nl_case("NL-SDL", "SDL", 1.0, Some("NL-DEAD")),
        nl_case("NL-LIVE", "LIVE", 1.0, Some("NL-SDL")),
        push,
    ] {
        let name = case.name.clone();
        sap_api::validate_case_definition(m, &case)
            .unwrap_or_else(|e| panic!("case '{name}' fails validation: {e}"));
        sap_api::set_case_definition(m, case)
            .unwrap_or_else(|e| panic!("case '{name}' is refused: {e}"));
    }
    roof
}
