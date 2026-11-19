//! Mass source (CSI Analysis Reference, "Mass Source", pp. 334-337) and the
//! 2012 IBC / ASCE 7-10 automatic seismic load (CSI Lateral Loads Manual,
//! section 2.14). Expected values are closed forms evaluated in the tests.

use sap_model::seismic::{
    AutoSeismicAsce710, MassSource, SeismicDirection, SeismicPeriod,
};
use sap_model::{
    AreaLoadUniform, AreaObj, AreaSection, ConstraintAxis, ConstraintDef, ConstraintType,
    FrameLoadDistributed, FrameLoadTemperature, FrameSection, LoadCase, LoadPattern,
    LoadPatternType, Material, PointLoadForce, SapModel,
};
use sap_solver::{analyze_model, SolverError};
use sap_units::Units;

/// Standard gravity in m/s^2, as the unit system defines it (SI units: kN, m).
const G: f64 = 9.806_65;

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1e-300)
}

fn assert_close(what: &str, got: f64, want: f64, tol: f64) {
    assert!(
        rel(got, want) <= tol,
        "{what}: got {got:e}, want {want:e}, relative error {:e} > {tol:e}",
        rel(got, want)
    );
}

/// kN, m model with an elastic material of zero self weight and zero density,
/// and one section `SEC`. No cases are defined.
fn base_model(e: f64, area: f64, inertia: f64) -> SapModel {
    let mut model = SapModel::new(Units::KnMC);
    let mut mat = Material::default_steel_kip_in();
    mat.name = "MAT".into();
    mat.e = e;
    mat.g = e / 2.6;
    mat.unit_weight = 0.0;
    mat.unit_mass = 0.0;
    model.materials.insert("MAT".into(), mat);
    let mut sec = FrameSection::default();
    sec.name = "SEC".into();
    sec.material = "MAT".into();
    sec.area = area;
    sec.i33 = inertia;
    sec.i22 = inertia;
    sec.torsional_j = 2.0 * inertia;
    sec.as2 = 0.0;
    sec.as3 = 0.0;
    model.frame_sections.insert("SEC".into(), sec);
    model.load_cases.clear();
    model.load_patterns.clear();
    model
}

fn pattern(model: &mut SapModel, name: &str, pat_type: LoadPatternType, self_wt: f64) {
    model.load_patterns.insert(
        name.into(),
        LoadPattern {
            name: name.into(),
            pat_type,
            self_wt_multiplier: self_wt,
        },
    );
}

fn joint_force(model: &mut SapModel, joint: &str, pat: &str, force: [f64; 6]) {
    model
        .points
        .get_mut(joint)
        .unwrap()
        .loads_force
        .push(PointLoadForce {
            load_pat: pat.into(),
            csys: "GLOBAL".into(),
            force,
        });
}

// ---------------------------------------------------------------------------
// Mass source
// ---------------------------------------------------------------------------

/// Cantilever column along Z, rigid base, weight P at the tip in pattern LIVE.
/// Axial and shear flexibility do not enter the lateral mode: the shear areas
/// are zero (shear deformation off) and A is large. With the mass source
/// {self off, additional off, LIVE x s}, m = s P / g and
/// T = 2 pi sqrt(m L^3 / (3 E I)).
fn cantilever(source: MassSource) -> SapModel {
    let (e, area, inertia, length, load) = (2.0e8, 1.0, 1.0e-4, 4.0, 100.0);
    let mut model = base_model(e, area, inertia);
    pattern(&mut model, "LIVE", LoadPatternType::Live, 0.0);
    model.add_point("B", 0.0, 0.0, 0.0);
    model.add_point("T", 0.0, 0.0, length);
    model.points.get_mut("B").unwrap().restraint = [true; 6];
    model.add_frame("C", "B", "T", "SEC");
    joint_force(&mut model, "T", "LIVE", [0.0, 0.0, -load, 0.0, 0.0, 0.0]);
    model.mass_source = source;
    model
        .load_cases
        .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 2));
    model
}

#[test]
fn cantilever_period_from_load_pattern_mass() {
    let (e, inertia, length, load) = (2.0e8_f64, 1.0e-4_f64, 4.0_f64, 100.0_f64);
    for multiplier in [1.0, 0.25] {
        let mut model = cantilever(MassSource {
            element_self_mass: false,
            additional_mass: false,
            load_patterns: vec![("LIVE".into(), multiplier)],
        });
        analyze_model(&mut model).unwrap();
        let modes = &model.case_results["MODAL"].modes;
        let mass = multiplier * load / G;
        let period = 2.0 * std::f64::consts::PI * (mass * length.powi(3) / (3.0 * e * inertia)).sqrt();
        // The two lateral modes are degenerate (I22 = I33).
        for mode in &modes[..2] {
            assert_close("period", mode.period_sec, period, 1e-9);
        }
    }
}

#[test]
fn cantilever_mass_source_toggles() {
    // Self mass and joint mass are the only terms when the pattern is absent.
    let (e, inertia, length) = (2.0e8_f64, 1.0e-4_f64, 4.0_f64);
    // Additional mass only: joint mass 7 t at the tip, nothing else.
    let mut model = cantilever(MassSource {
        element_self_mass: false,
        additional_mass: true,
        load_patterns: vec![],
    });
    model.points.get_mut("T").unwrap().mass = [7.0, 7.0, 7.0, 0.0, 0.0, 0.0];
    analyze_model(&mut model).unwrap();
    let want = 2.0 * std::f64::consts::PI * (7.0 * length.powi(3) / (3.0 * e * inertia)).sqrt();
    assert_close("additional mass only", model.case_results["MODAL"].modes[0].period_sec, want, 1e-9);

    // Additional mass off: the same joint mass must not act. Add the pattern
    // so that the model still has mass: m = P/g only.
    let mut model = cantilever(MassSource {
        element_self_mass: false,
        additional_mass: false,
        load_patterns: vec![("LIVE".into(), 1.0)],
    });
    model.points.get_mut("T").unwrap().mass = [7.0, 7.0, 7.0, 0.0, 0.0, 0.0];
    analyze_model(&mut model).unwrap();
    let want = 2.0 * std::f64::consts::PI * ((100.0 / G) * length.powi(3) / (3.0 * e * inertia)).sqrt();
    assert_close("joint mass ignored", model.case_results["MODAL"].modes[0].period_sec, want, 1e-9);

    // Both the additional mass and the pattern: masses add (7 + P/g).
    let mut model = cantilever(MassSource {
        element_self_mass: false,
        additional_mass: true,
        load_patterns: vec![("LIVE".into(), 1.0)],
    });
    model.points.get_mut("T").unwrap().mass = [7.0, 7.0, 7.0, 0.0, 0.0, 0.0];
    analyze_model(&mut model).unwrap();
    let want = 2.0
        * std::f64::consts::PI
        * ((7.0 + 100.0 / G) * length.powi(3) / (3.0 * e * inertia)).sqrt();
    assert_close("sum of contributions", model.case_results["MODAL"].modes[0].period_sec, want, 1e-9);
}

/// Four columns on grounded springs, a perimeter of four beams and one shell
/// over the rectangle. All joints are free, so the whole mass is active.
/// Densities: frame 2.0 t/m^3, shell 2.0 t/m^3; weights 20 kN/m^3.
struct Table {
    model: SapModel,
    /// Total frame length (columns + beams), m.
    frame_length: f64,
    beam_length: f64,
    plan_area: f64,
    area_area: f64,
}

const DENSITY: f64 = 2.0;
const UNIT_WEIGHT: f64 = 20.0;
const BEAM_W: f64 = 3.0; // kN/m
const SHELL_Q: f64 = 1.5; // kPa
const SHELL_T: f64 = 0.2; // m
const AREA_MASS: f64 = 0.05; // t/m^2

fn table() -> Table {
    let (lx, ly, h) = (6.0, 4.0, 3.0);
    let mut model = base_model(2.0e8, 0.02, 4.0e-4);
    {
        let mat = model.materials.get_mut("MAT").unwrap();
        mat.unit_mass = DENSITY;
        mat.unit_weight = UNIT_WEIGHT;
    }
    let mut asec = AreaSection::default();
    asec.name = "ASEC".into();
    asec.material = "MAT".into();
    asec.thickness_membrane = SHELL_T;
    asec.thickness_bending = SHELL_T;
    model.area_sections.insert("ASEC".into(), asec);
    pattern(&mut model, "DEAD", LoadPatternType::Dead, 1.0);
    pattern(&mut model, "LIVE", LoadPatternType::Live, 0.0);
    let corners = [(0.0, 0.0), (lx, 0.0), (lx, ly), (0.0, ly)];
    for (i, (x, y)) in corners.iter().enumerate() {
        model.add_point(format!("B{i}"), *x, *y, 0.0);
        model.add_point(format!("T{i}"), *x, *y, h);
        let base = model.points.get_mut(&format!("B{i}")).unwrap();
        base.spring = [1.0e6, 1.0e6, 1.0e6, 1.0e3, 1.0e3, 1.0e3];
        model.add_frame(format!("C{i}"), format!("B{i}"), format!("T{i}"), "SEC");
    }
    for i in 0..4 {
        let name = format!("G{i}");
        model.add_frame(name.clone(), format!("T{i}"), format!("T{}", (i + 1) % 4), "SEC");
        let beam = model.frames.get_mut(&name).unwrap();
        for pat in ["DEAD", "LIVE"] {
            beam.loads_dist.push(FrameLoadDistributed {
                load_pat: pat.into(),
                my_type: 1,
                dir: 10,
                csys: "GLOBAL".into(),
                rel_dist1: 0.0,
                rel_dist2: 1.0,
                val1: BEAM_W,
                val2: BEAM_W,
            });
        }
    }
    model.add_area(
        "S",
        (0..4).map(|i| format!("T{i}")).collect(),
        "ASEC",
    );
    let shell: &mut AreaObj = model.areas.get_mut("S").unwrap();
    shell.mass_per_area = AREA_MASS;
    for pat in ["DEAD", "LIVE"] {
        shell.loads_uniform.push(AreaLoadUniform {
            load_pat: pat.into(),
            csys: "GLOBAL".into(),
            dir: 10,
            val: SHELL_Q,
        });
    }
    // Joint mass of 1.5 t on two top joints.
    for j in ["T0", "T2"] {
        model.points.get_mut(j).unwrap().mass = [1.5, 1.5, 1.5, 0.0, 0.0, 0.0];
    }
    model
        .load_cases
        .insert("DEAD".into(), LoadCase::linear_static("DEAD", "DEAD", 1.0));
    model
        .load_cases
        .insert("LIVE".into(), LoadCase::linear_static("LIVE", "LIVE", 1.0));
    model
        .load_cases
        .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 12));
    Table {
        model,
        frame_length: 4.0 * h + 2.0 * (lx + ly),
        beam_length: 2.0 * (lx + ly),
        plan_area: lx * ly,
        area_area: lx * ly,
    }
}

/// Total translational mass in global X, Y and Z, recovered from the modal
/// output: for any mode the participation factor satisfies
/// `pf^2 / mass_ratio = total mass`, using the mode that excites each
/// direction most.
fn total_mass(model: &SapModel) -> [f64; 3] {
    let mut model = model.clone();
    analyze_model(&mut model).unwrap();
    let modes = &model.case_results["MODAL"].modes;
    std::array::from_fn(|d| {
        let best = modes
            .iter()
            .max_by(|a, b| a.modal_mass_ratios[d].total_cmp(&b.modal_mass_ratios[d]))
            .unwrap();
        assert!(best.modal_mass_ratios[d] > 1e-6, "direction {d} not excited");
        best.participation_factors[d].powi(2) / best.modal_mass_ratios[d]
    })
}

#[test]
fn dead_load_mass_times_gravity_equals_total_vertical_reaction() {
    let mut t = table();
    t.model.mass_source = MassSource {
        element_self_mass: false,
        additional_mass: false,
        load_patterns: vec![("DEAD".into(), 1.0)],
    };
    let mass = total_mass(&t.model);
    let mut model = t.model.clone();
    analyze_model(&mut model).unwrap();
    let reaction_z = model.case_results["DEAD"].base_reactions[2];

    // The mass acts equally in the three translations.
    assert_close("Mx = My", mass[0], mass[1], 1e-12);
    assert_close("Mx = Mz", mass[0], mass[2], 1e-12);
    // g * M equals the total vertical reaction of the DEAD static case.
    assert_close("g M = reaction", G * mass[0], reaction_z, 1e-10);

    // Closed form of the DEAD load: frame self weight, beam load, shell self
    // weight and shell uniform load.
    let weight = UNIT_WEIGHT * 0.02 * t.frame_length
        + BEAM_W * t.beam_length
        + (UNIT_WEIGHT * SHELL_T + SHELL_Q) * t.area_area;
    assert_close("closed form weight", reaction_z, weight, 1e-10);
    assert_close("closed form mass", mass[0], weight / G, 1e-10);
}

#[test]
fn mass_source_is_linear_in_patterns_and_each_toggle_works() {
    let mut t = table();
    let source = |self_mass: bool, added: bool, patterns: &[(&str, f64)]| MassSource {
        element_self_mass: self_mass,
        additional_mass: added,
        load_patterns: patterns.iter().map(|(p, f)| (p.to_string(), *f)).collect(),
    };
    let mut mass_of = |s: MassSource| {
        t.model.mass_source = s;
        total_mass(&t.model)[2]
    };
    let dead = mass_of(source(false, false, &[("DEAD", 1.0)]));
    let live = mass_of(source(false, false, &[("LIVE", 1.0)]));
    let combined = mass_of(source(false, false, &[("DEAD", 1.0), ("LIVE", 0.25)]));
    assert_close("D + 0.25 L", combined, dead + 0.25 * live, 1e-12);
    // LIVE carries only the beam and shell loads.
    let live_weight = BEAM_W * t.beam_length + SHELL_Q * t.area_area;
    assert_close("live mass", live, live_weight / G, 1e-10);

    // Self mass: frames and the shell from the material density.
    let frame_mass = DENSITY * 0.02 * t.frame_length;
    let shell_mass = DENSITY * SHELL_T * t.area_area;
    let self_only = mass_of(source(true, false, &[]));
    assert_close("self mass", self_only, frame_mass + shell_mass, 1e-10);
    // Additional mass: two joint masses and the shell mass per area.
    let added_only = mass_of(source(false, true, &[]));
    assert_close("additional mass", added_only, 3.0 + AREA_MASS * t.plan_area, 1e-10);
    // The default source is the sum of both.
    let default = mass_of(MassSource::default());
    assert_close("default", default, self_only + added_only, 1e-12);
    // All three contributions add.
    let all = mass_of(source(true, true, &[("LIVE", 0.5)]));
    assert_close("all", all, default + 0.5 * live, 1e-12);
}

#[test]
fn default_mass_source_is_the_legacy_sum_and_stable() {
    // The default source is element self mass plus additional mass, with no
    // pattern, and is the value a model has when the field was never set.
    let reference = table().model;
    assert!(reference.mass_source.is_default());
    assert_eq!(reference.mass_source, MassSource::default());
    // Writing the default out explicitly changes no bit of the modal results.
    let mut explicit = reference.clone();
    explicit.mass_source = MassSource {
        element_self_mass: true,
        additional_mass: true,
        load_patterns: Vec::new(),
    };
    let mut plain = reference.clone();
    analyze_model(&mut plain).unwrap();
    analyze_model(&mut explicit).unwrap();
    let (a, b) = (&plain.case_results["MODAL"], &explicit.case_results["MODAL"]);
    assert_eq!(a.modes.len(), b.modes.len());
    for (x, y) in a.modes.iter().zip(&b.modes) {
        assert_eq!(x.eigenvalue.to_bits(), y.eigenvalue.to_bits());
        for (name, shape) in &x.mode_shape {
            for d in 0..6 {
                assert_eq!(shape[d].to_bits(), y.mode_shape[name][d].to_bits());
            }
        }
    }
    // The legacy total: frame mass rho A L, shell rho t A, joint masses and
    // added area mass.
    let t = table();
    let want = DENSITY * 0.02 * t.frame_length
        + DENSITY * SHELL_T * t.area_area
        + 3.0
        + AREA_MASS * t.plan_area;
    assert_close("legacy total mass", total_mass(&reference)[0], want, 1e-10);
}

// ---------------------------------------------------------------------------
// Refusals of the mass source
// ---------------------------------------------------------------------------

fn refused(model: &mut SapModel, needle: &str) {
    let err = analyze_model(model).expect_err("must be refused");
    let text = err.to_string();
    assert!(
        matches!(
            err,
            SolverError::UnsupportedInput(_) | SolverError::InvalidInput(_)
        ),
        "wrong error {err:?}"
    );
    assert!(text.contains(needle), "message `{text}` lacks `{needle}`");
    assert!(!model.is_locked && model.case_results.is_empty());
}

#[test]
fn mass_source_refusals() {
    let source = |p: &str, f: f64| MassSource {
        element_self_mass: true,
        additional_mass: true,
        load_patterns: vec![(p.to_string(), f)],
    };
    // Unknown pattern.
    let mut m = table().model;
    m.mass_source = source("NOPE", 1.0);
    refused(&mut m, "NOPE");
    // Non-finite factors.
    for f in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let mut m = table().model;
        m.mass_source = source("DEAD", f);
        refused(&mut m, "not finite");
    }
    // An automatic lateral pattern.
    let mut m = table().model;
    pattern(&mut m, "EQX", LoadPatternType::Quake, 0.0);
    m.auto_seismic.insert("EQX".into(), complete_definition());
    m.mass_source = source("EQX", 1.0);
    refused(&mut m, "automatic lateral");
    // Net negative translational mass at a free DOF: an upward load as the
    // only source.
    let mut m = table().model;
    m.mass_source = MassSource {
        element_self_mass: false,
        additional_mass: false,
        load_patterns: vec![("LIVE".into(), -1.0)],
    };
    refused(&mut m, "negative translational mass");
    // Temperature loads are not gravity loads.
    let mut m = table().model;
    m.frames
        .get_mut("G0")
        .unwrap()
        .loads_temp
        .push(FrameLoadTemperature {
            load_pat: "LIVE".into(),
            my_type: 1,
            val: 10.0,
            pattern_name: None,
        });
    m.mass_source = source("LIVE", 1.0);
    refused(&mut m, "temperature");
    // A joint load in a named system of a pattern that no case uses.
    let mut m = table().model;
    pattern(&mut m, "OTHER", LoadPatternType::Other, 0.0);
    joint_force(&mut m, "T0", "OTHER", [0.0, 0.0, -1.0, 0.0, 0.0, 0.0]);
    m.points.get_mut("T0").unwrap().loads_force[0].csys = "ROTATED".into();
    m.mass_source = source("OTHER", 1.0);
    refused(&mut m, "named coordinate system");
}

// ---------------------------------------------------------------------------
// Automatic seismic load
// ---------------------------------------------------------------------------

/// Three-storey stick in metric units: base at z = 0, masses 20, 25 and
/// 30 t at z = 3, 6 and 9 m. All frame mass is off (density 0).
const STORY_MASS: [f64; 3] = [20.0, 25.0, 30.0];
const STORY_Z: [f64; 3] = [3.0, 6.0, 9.0];

fn stick() -> SapModel {
    let mut model = base_model(2.0e8, 0.05, 5.0e-4);
    pattern(&mut model, "EQX", LoadPatternType::Quake, 0.0);
    model.add_point("0", 0.0, 0.0, 0.0);
    model.points.get_mut("0").unwrap().restraint = [true; 6];
    for (i, (z, m)) in STORY_Z.iter().zip(STORY_MASS).enumerate() {
        let name = (i + 1).to_string();
        model.add_point(name.clone(), 0.0, 0.0, *z);
        model.points.get_mut(&name).unwrap().mass = [m, m, m, 0.0, 0.0, 0.0];
        model.add_frame(format!("C{name}"), i.to_string(), name, "SEC");
    }
    model
        .load_cases
        .insert("EQ".into(), LoadCase::linear_static("EQ", "EQX", 1.0));
    model
}

fn complete_definition() -> AutoSeismicAsce710 {
    AutoSeismicAsce710 {
        direction: SeismicDirection::X,
        eccentricity_ratio: 0.05,
        period: SeismicPeriod::User { t: 1.0 },
        r: 5.0,
        omega0: 3.0,
        cd: 4.5,
        ie: 1.0,
        ss: 1.5,
        s1: 0.5,
        tl: 8.0,
        fa: Some(1.0),
        fv: Some(1.0),
        z_min: None,
        z_max: None,
    }
}

/// The equations of the manual, evaluated independently of the solver.
/// Returns `(Cs, k)`.
fn manual_cs(d: &AutoSeismicAsce710, t: f64) -> (f64, f64) {
    let sds = 2.0 / 3.0 * d.fa.unwrap() * d.ss;
    let sd1 = 2.0 / 3.0 * d.fv.unwrap() * d.s1;
    let ri = d.r / d.ie;
    let mut cs = sds / ri; // 12.8-2
    let upper = if t <= d.tl {
        sd1 / (t * ri) // 12.8-3
    } else {
        sd1 * d.tl / (t * t * ri) // 12.8-4
    };
    cs = cs.min(upper);
    cs = cs.max((0.044 * sds * d.ie).max(0.01)); // 12.8-5
    if d.s1 >= 0.6 {
        cs = cs.max(0.5 * d.s1 / ri); // 12.8-6
    }
    let k = if t <= 0.5 {
        1.0
    } else if t >= 2.5 {
        2.0
    } else {
        1.0 + (t - 0.5) / 2.0
    };
    (cs, k)
}

fn run_stick(def: AutoSeismicAsce710) -> SapModel {
    let mut model = stick();
    model.auto_seismic.insert("EQX".into(), def);
    analyze_model(&mut model).unwrap();
    model
}

/// Checks one definition against the printed equations and the statics.
fn check_stick(label: &str, def: AutoSeismicAsce710, t: f64, equation: &str) {
    let model = run_stick(def.clone());
    let res = &model.case_results["EQ"];
    assert_eq!(res.auto_lateral.len(), 1, "{label}");
    let report = &res.auto_lateral[0];
    let (cs, k) = manual_cs(&def, t);
    assert_eq!(report.governing_equation, equation, "{label}");
    assert_eq!(report.pattern, "EQX");
    assert_close(&format!("{label} T"), report.period, t, 1e-12);
    assert_close(&format!("{label} Cs"), report.cs, cs, 1e-12);
    assert_close(&format!("{label} k"), report.k, k, 1e-12);
    assert_close(
        &format!("{label} SDS"),
        report.sds,
        2.0 / 3.0 * def.fa.unwrap() * def.ss,
        1e-12,
    );
    assert_close(
        &format!("{label} SD1"),
        report.sd1,
        2.0 / 3.0 * def.fv.unwrap() * def.s1,
        1e-12,
    );
    // Weight: sum of m g over the three levels (the base joint has no mass).
    let weight: f64 = STORY_MASS.iter().map(|m| m * G).sum();
    assert_close(&format!("{label} W"), report.weight, weight, 1e-12);
    let v = cs * weight;
    assert_close(&format!("{label} V"), report.base_shear, v, 1e-12);
    let denom: f64 = STORY_MASS
        .iter()
        .zip(STORY_Z)
        .map(|(m, z)| m * G * z.powf(k))
        .sum();
    assert_eq!(report.joint_forces.len(), 3);
    for (i, f) in report.joint_forces.iter().enumerate() {
        let w = STORY_MASS[i] * G;
        let want = v * w * STORY_Z[i].powf(k) / denom;
        assert_eq!(f.joint, (i + 1).to_string());
        assert_close(&format!("{label} F{}", i + 1), f.force, want, 1e-12);
        assert_close(&format!("{label} z{}", i + 1), f.z, STORY_Z[i], 1e-12);
    }
    // Equilibrium of the applied load: the base reaction is -V in X.
    assert_close(&format!("{label} base shear"), -res.base_reactions[0], v, 1e-9);
    assert!(res.base_reactions[1].abs() < 1e-9 * v);
}

#[test]
fn every_branch_of_the_base_shear_coefficient() {
    let base = complete_definition(); // SDS = 1.0, SD1 = 1/3, R = 5, Ie = 1, TL = 8
    // 12.8-2 governs: T = 0.3, upper limit 0.333 / (0.3 * 5) = 0.222 > 0.2.
    let mut d = base.clone();
    d.period = SeismicPeriod::User { t: 0.3 };
    check_stick("12.8-2, k = 1", d, 0.3, "12.8-2");
    // 12.8-3 governs: T = 1.0 <= TL, cs = 0.333/5 = 0.0667 (> 0.044).
    let mut d = base.clone();
    d.period = SeismicPeriod::User { t: 1.0 };
    check_stick("12.8-3, k interpolated 1.25", d, 1.0, "12.8-3");
    // k = 1.5 at T = 1.5.
    let mut d = base.clone();
    d.period = SeismicPeriod::User { t: 1.5 };
    check_stick("12.8-3, k = 1.5", d, 1.5, "12.8-3");
    // 12.8-4 governs: T = 10 > TL, small SDS so that 12.8-5 does not.
    let mut d = base.clone();
    d.period = SeismicPeriod::User { t: 10.0 };
    d.ss = 0.3;
    d.r = 1.5;
    d.s1 = 0.5;
    d.fv = Some(1.5);
    check_stick("12.8-4, k = 2", d, 10.0, "12.8-4");
    // 12.8-5 (0.01 floor): the same with T = 40.
    let mut d = base.clone();
    d.period = SeismicPeriod::User { t: 40.0 };
    d.ss = 0.3;
    d.r = 1.5;
    d.s1 = 0.5;
    d.fv = Some(1.5);
    check_stick("12.8-5 floor 0.01", d, 40.0, "12.8-5");
    // 12.8-5 (0.044 SDS Ie): large R and Ie = 1.25.
    let mut d = base.clone();
    d.period = SeismicPeriod::User { t: 20.0 };
    d.r = 8.0;
    d.ie = 1.25;
    check_stick("12.8-5 floor 0.044 SDS Ie", d, 20.0, "12.8-5");
    // 12.8-6: S1 >= 0.6 raises the minimum.
    let mut d = base.clone();
    d.period = SeismicPeriod::User { t: 20.0 };
    d.r = 8.0;
    d.s1 = 0.8;
    d.fv = Some(1.0);
    check_stick("12.8-6", d, 20.0, "12.8-6");
}

#[test]
fn approximate_period_uses_feet() {
    // hn = 9 m = 9 / 0.3048 ft; Ta = Ct hn^x with Ct, x as the code prints.
    let (ct, x) = (0.028, 0.8);
    let mut d = complete_definition();
    d.period = SeismicPeriod::Approximate { ct, x };
    let ta = ct * (9.0f64 / 0.3048).powf(x);
    // Ta = 0.41 s: the cap SD1/(Ta R/Ie) = 0.163 is below SDS/(R/Ie) = 0.2.
    check_stick("approximate period", d.clone(), ta, "12.8-3");
    let model = run_stick(d);
    let rep = &model.case_results["EQ"].auto_lateral[0];
    assert_close("Ta stored", rep.approximate_period.unwrap(), ta, 1e-12);
    // Same Ta in a foot model of the same building: hn = 29.5276 ft.
    let mut foot = stick();
    foot.auto_seismic.insert("EQX".into(), {
        let mut d = complete_definition();
        d.period = SeismicPeriod::Approximate { ct, x };
        d
    });
    foot.convert_units_in_place(Units::KipFtF);
    assert!(foot.auto_seismic["EQX"].z_max.is_none());
    analyze_model(&mut foot).unwrap();
    let rep = &foot.case_results["EQ"].auto_lateral[0];
    assert_close("Ta in feet", rep.approximate_period.unwrap(), ta, 1e-9);
}

#[test]
fn level_range_excludes_base_mass_and_joints_above_the_top() {
    // Mass at the base joint (restrained, z = 0) is not weight.
    let mut model = stick();
    model.points.get_mut("0").unwrap().mass = [100.0, 100.0, 100.0, 0.0, 0.0, 0.0];
    // Top elevation 6 m: the joint at 9 m gets nothing and is not counted.
    let mut def = complete_definition();
    def.z_min = Some(0.0);
    def.z_max = Some(6.0);
    model.auto_seismic.insert("EQX".into(), def.clone());
    analyze_model(&mut model).unwrap();
    let rep = &model.case_results["EQ"].auto_lateral[0];
    let names: Vec<&str> = rep.joint_forces.iter().map(|f| f.joint.as_str()).collect();
    assert_eq!(names, ["1", "2"]);
    let weight = (STORY_MASS[0] + STORY_MASS[1]) * G;
    assert_close("W", rep.weight, weight, 1e-12);
    let t = 1.0;
    let (cs, k) = manual_cs(&def, t);
    let v = cs * weight;
    let denom = STORY_MASS[0] * G * 3.0f64.powf(k) + STORY_MASS[1] * G * 6.0f64.powf(k);
    assert_close("F1", rep.joint_forces[0].force, v * STORY_MASS[0] * G * 3.0f64.powf(k) / denom, 1e-12);
    // hn is the loaded range: 6 m for the approximate period.
    let mut def2 = def.clone();
    def2.period = SeismicPeriod::Approximate { ct: 0.02, x: 0.75 };
    model.auto_seismic.insert("EQX".into(), def2);
    analyze_model(&mut model).unwrap();
    let rep = &model.case_results["EQ"].auto_lateral[0];
    assert_close(
        "Ta over the loaded range",
        rep.period,
        0.02 * (6.0f64 / 0.3048).powf(0.75),
        1e-12,
    );
    // The reaction at the base carries exactly V; the mass at the base level
    // is not in it.
    assert_close("ΣRx", -model.case_results["EQ"].base_reactions[0], rep.base_shear, 1e-9);

    // A bottom elevation of 3 m excludes the joint at 3 m (z_min < z).
    let mut def3 = def;
    def3.z_min = Some(3.0);
    def3.z_max = Some(9.0);
    model.auto_seismic.insert("EQX".into(), def3);
    analyze_model(&mut model).unwrap();
    let rep = &model.case_results["EQ"].auto_lateral[0];
    let names: Vec<&str> = rep.joint_forces.iter().map(|f| f.joint.as_str()).collect();
    assert_eq!(names, ["2", "3"]);
    assert_close("h of joint 2", rep.joint_forces[0].z - 3.0, 3.0, 1e-12);
}

#[test]
fn report_is_stored_only_in_cases_that_apply_the_pattern() {
    let mut model = stick();
    pattern(&mut model, "OTHER", LoadPatternType::Other, 0.0);
    joint_force(&mut model, "3", "OTHER", [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    model
        .load_cases
        .insert("OTHER".into(), LoadCase::linear_static("OTHER", "OTHER", 1.0));
    let mut both = LoadCase::linear_static("BOTH", "EQX", 2.0);
    both.loads.push(sap_model::LoadCaseAssignment {
        load_name: "OTHER".into(),
        scale_factor: 1.0,
    });
    model.load_cases.insert("BOTH".into(), both);
    model
        .load_cases
        .insert("MODAL".into(), LoadCase::modal_eigen("MODAL", 1));
    model.auto_seismic.insert("EQX".into(), complete_definition());
    analyze_model(&mut model).unwrap();
    assert_eq!(model.case_results["EQ"].auto_lateral.len(), 1);
    assert_eq!(model.case_results["BOTH"].auto_lateral.len(), 1);
    assert!(model.case_results["OTHER"].auto_lateral.is_empty());
    assert!(model.case_results["MODAL"].auto_lateral.is_empty());
    // The load is applied at the case scale factor (2) on top of the user load.
    let v = model.case_results["EQ"].auto_lateral[0].base_shear;
    assert_close("scaled case", -model.case_results["BOTH"].base_reactions[0], 2.0 * v + 1.0, 1e-9);
}

#[test]
fn unit_change_scales_the_elevation_range_only() {
    let mut model = stick();
    let mut def = complete_definition();
    def.z_min = Some(3.0);
    def.z_max = Some(9.0);
    def.period = SeismicPeriod::Approximate { ct: 0.02, x: 0.75 };
    model.auto_seismic.insert("EQX".into(), def.clone());
    model.convert_units_in_place(Units::KnCmC);
    let d = &model.auto_seismic["EQX"];
    assert_close("z_min cm", d.z_min.unwrap(), 300.0, 1e-12);
    assert_close("z_max cm", d.z_max.unwrap(), 900.0, 1e-12);
    // Ct, x, TL, Ss, S1 are not lengths.
    assert_eq!(d.period, def.period);
    assert_eq!((d.tl, d.ss, d.s1, d.r), (def.tl, def.ss, def.s1, def.r));
}

#[test]
fn eccentricity_is_ignored_without_diaphragm() {
    let mut a = complete_definition();
    a.eccentricity_ratio = 0.0;
    let mut b = complete_definition();
    b.eccentricity_ratio = 0.05;
    let fa = run_stick(a).case_results["EQ"].auto_lateral[0].joint_forces.clone();
    let fb = run_stick(b).case_results["EQ"].auto_lateral[0].joint_forces.clone();
    assert_eq!(fa, fb);
}

// ---------------------------------------------------------------------------
// Refusals of the automatic load
// ---------------------------------------------------------------------------

#[test]
fn auto_seismic_refusals() {
    let with = |edit: &dyn Fn(&mut AutoSeismicAsce710)| {
        let mut m = stick();
        let mut d = complete_definition();
        edit(&mut d);
        m.auto_seismic.insert("EQX".into(), d);
        m
    };
    // Parameters that must be positive.
    refused(&mut with(&|d| d.r = 0.0), "R must");
    refused(&mut with(&|d| d.r = -1.0), "R must");
    refused(&mut with(&|d| d.ie = 0.0), "Ie must");
    refused(&mut with(&|d| d.tl = 0.0), "TL must");
    refused(&mut with(&|d| d.tl = f64::NAN), "TL must");
    refused(
        &mut with(&|d| d.period = SeismicPeriod::Approximate { ct: 0.0, x: 0.75 }),
        "Ct must",
    );
    refused(
        &mut with(&|d| d.period = SeismicPeriod::Approximate { ct: 0.02, x: -0.1 }),
        "x must",
    );
    refused(&mut with(&|d| d.period = SeismicPeriod::User { t: 0.0 }), "period T");
    refused(&mut with(&|d| d.period = SeismicPeriod::User { t: -1.0 }), "period T");
    // Program-calculated period.
    refused(
        &mut with(&|d| d.period = SeismicPeriod::ProgramCalculated),
        "program-calculated",
    );
    // Site coefficients from the site class need tables that are not available.
    refused(&mut with(&|d| d.fa = None), "Fa from the site class");
    refused(&mut with(&|d| d.fv = None), "Fv from the site class");
    // Elevation range.
    refused(
        &mut with(&|d| {
            d.z_min = Some(9.0);
            d.z_max = Some(3.0)
        }),
        "z_max must be above z_min",
    );
    // Eccentricity of a joint that is on a diaphragm.
    let mut m = with(&|_| {});
    m.constraints.insert(
        "D1".into(),
        ConstraintDef {
            name: "D1".into(),
            constraint_type: ConstraintType::Diaphragm,
            axis: ConstraintAxis::Z,
            dofs: [true, true, false, false, false, true],
        },
    );
    for j in ["2", "3"] {
        m.points.get_mut(j).unwrap().constraints.push("D1".into());
    }
    // Wave 5 (G24): a diaphragm with eccentricity is analysed; on this stick
    // every joint is on the Z axis, so the width, and with it the moment, is 0
    // (the magnitude and sense are tested in `wave5_eccentricity.rs`).
    analyze_model(&mut m).unwrap();
    assert!(m.case_results["EQ"].auto_lateral[0]
        .diaphragm_moments
        .is_empty());
    // A pattern that is not Quake.
    let mut m = with(&|_| {});
    m.load_patterns.get_mut("EQX").unwrap().pat_type = LoadPatternType::Wind;
    refused(&mut m, "not Quake");
    // A definition without a pattern.
    let mut m = with(&|_| {});
    m.auto_seismic.insert("GHOST".into(), complete_definition());
    refused(&mut m, "GHOST");
    // Inactive load direction.
    let mut m = with(&|_| {});
    m.active_dofs[0] = false;
    refused(&mut m, "active degree of freedom");
    // Rotated local axes with unequal masses.
    let mut m = with(&|_| {});
    {
        let p = m.points.get_mut("2").unwrap();
        p.mass = [10.0, 20.0, 30.0, 0.0, 0.0, 0.0];
        p.local_axes_deg = [30.0, 0.0, 0.0];
    }
    refused(&mut m, "rotated local axes");
    // No joint with mass above the base.
    let mut m = with(&|_| {});
    for p in m.points.values_mut() {
        p.mass = [0.0; 6];
    }
    refused(&mut m, "no joint with mass");
    // Mass only at the base level.
    let mut m = with(&|_| {});
    for p in m.points.values_mut() {
        p.mass = [0.0; 6];
    }
    m.points.get_mut("0").unwrap().mass = [5.0, 5.0, 5.0, 0.0, 0.0, 0.0];
    refused(&mut m, "no joint with mass");
}

// ---------------------------------------------------------------------------
// More refusals: one test line per refusal
// ---------------------------------------------------------------------------

#[test]
fn mass_pattern_load_refusals() {
    use sap_model::{AreaLoadSurfacePressure, AreaLoadTemperature, FrameLoadPoint};
    let source = MassSource {
        element_self_mass: true,
        additional_mass: true,
        load_patterns: vec![("OTHER".into(), 1.0)],
    };
    let with = |edit: &dyn Fn(&mut SapModel)| {
        let mut m = table().model;
        pattern(&mut m, "OTHER", LoadPatternType::Other, 0.0);
        m.mass_source = source.clone();
        edit(&mut m);
        m
    };
    // A nonfinite joint force of the pattern.
    refused(
        &mut with(&|m| joint_force(m, "T1", "OTHER", [f64::NAN, 0.0, 0.0, 0.0, 0.0, 0.0])),
        "nonfinite force",
    );
    // A frame load in a named coordinate system, distributed and point.
    refused(
        &mut with(&|m| {
            m.frames
                .get_mut("G0")
                .unwrap()
                .loads_dist
                .push(FrameLoadDistributed {
                    load_pat: "OTHER".into(),
                    my_type: 1,
                    dir: 6,
                    csys: "ROTATED".into(),
                    rel_dist1: 0.0,
                    rel_dist2: 1.0,
                    val1: 1.0,
                    val2: 1.0,
                })
        }),
        "named coordinate system",
    );
    refused(
        &mut with(&|m| {
            m.frames
                .get_mut("G0")
                .unwrap()
                .loads_point
                .push(FrameLoadPoint {
                    load_pat: "OTHER".into(),
                    my_type: 1,
                    dir: 6,
                    csys: "ROTATED".into(),
                    rel_dist: 0.5,
                    val: 1.0,
                })
        }),
        "named coordinate system",
    );
    // An area uniform load in a named system, and a projected one.
    for (csys, dir) in [("ROTATED", 6), ("GLOBAL", 9)] {
        refused(
            &mut with(&|m| {
                m.areas
                    .get_mut("S")
                    .unwrap()
                    .loads_uniform
                    .push(AreaLoadUniform {
                        load_pat: "OTHER".into(),
                        csys: csys.into(),
                        dir,
                        val: 1.0,
                    })
            }),
            "named system or projected",
        );
    }
    // An edge pressure and a patterned pressure.
    for (face, name) in [(1, None), (-2, Some("JP".to_string()))] {
        refused(
            &mut with(&|m| {
                m.areas
                    .get_mut("S")
                    .unwrap()
                    .loads_pressure
                    .push(AreaLoadSurfacePressure {
                        load_pat: "OTHER".into(),
                        face,
                        val: 1.0,
                        pattern_name: name.clone(),
                    })
            }),
            "edge or patterned pressure",
        );
    }
    // An area temperature load.
    refused(
        &mut with(&|m| {
            m.areas
                .get_mut("S")
                .unwrap()
                .loads_temp
                .push(AreaLoadTemperature {
                    load_pat: "OTHER".into(),
                    my_type: 1,
                    val: 5.0,
                    pattern_name: None,
                })
        }),
        "temperature",
    );
}

#[test]
fn auto_seismic_value_refusals() {
    let with = |edit: &dyn Fn(&mut AutoSeismicAsce710)| {
        let mut m = stick();
        let mut d = complete_definition();
        edit(&mut d);
        m.auto_seismic.insert("EQX".into(), d);
        m
    };
    refused(&mut with(&|d| d.omega0 = 0.0), "Omega0 must");
    refused(&mut with(&|d| d.cd = -2.0), "Cd must");
    refused(&mut with(&|d| d.ss = -0.1), "Ss must");
    refused(&mut with(&|d| d.ss = f64::NAN), "Ss must");
    refused(&mut with(&|d| d.s1 = -0.1), "S1 must");
    refused(&mut with(&|d| d.s1 = f64::INFINITY), "S1 must");
    refused(
        &mut with(&|d| d.eccentricity_ratio = f64::NAN),
        "eccentricity ratio",
    );
    refused(&mut with(&|d| d.z_min = Some(f64::NAN)), "z_min is not finite");
    refused(
        &mut with(&|d| d.z_max = Some(f64::INFINITY)),
        "z_max is not finite",
    );
    refused(&mut with(&|d| d.fa = Some(0.0)), "Fa must");
    refused(&mut with(&|d| d.fv = Some(-1.0)), "Fv must");
    refused(&mut with(&|d| d.r = f64::INFINITY), "R must");
    // Only a bottom elevation, above every joint: the range is empty.
    refused(&mut with(&|d| d.z_min = Some(20.0)), "elevation range is empty");
    // Only a top elevation below every joint.
    refused(&mut with(&|d| d.z_max = Some(-5.0)), "elevation range is empty");
}

// ---------------------------------------------------------------------------
// Automatically meshed frames and areas
// ---------------------------------------------------------------------------

#[test]
fn meshed_frames_and_areas_keep_the_mass_and_load_identities() {
    let mut t = table();
    for i in 0..4 {
        t.model
            .frames
            .get_mut(&format!("G{i}"))
            .unwrap()
            .auto_mesh_segments = 3;
    }
    t.model.areas.get_mut("S").unwrap().auto_mesh = (2, 2);
    pattern(&mut t.model, "EQX", LoadPatternType::Quake, 0.0);
    t.model
        .load_cases
        .insert("EQ".into(), LoadCase::linear_static("EQ", "EQX", 1.0));

    // (a) The DEAD pattern as the only mass: g M equals the vertical reaction.
    let mut a = t.model.clone();
    a.mass_source = MassSource {
        element_self_mass: false,
        additional_mass: false,
        load_patterns: vec![("DEAD".into(), 1.0)],
    };
    let mass = total_mass(&a);
    analyze_model(&mut a).unwrap();
    assert_close(
        "meshed g M",
        G * mass[2],
        a.case_results["DEAD"].base_reactions[2],
        1e-9,
    );
    let weight = UNIT_WEIGHT * 0.02 * t.frame_length
        + BEAM_W * t.beam_length
        + (UNIT_WEIGHT * SHELL_T + SHELL_Q) * t.area_area;
    assert_close("meshed closed form", mass[2], weight / G, 1e-9);

    // (b) Automatic load on the default mass: W is the mass above the base.
    // The four column bases carry half of each column mass.
    let mut b = t.model.clone();
    let def = complete_definition();
    b.auto_seismic.insert("EQX".into(), def.clone());
    let total = DENSITY * 0.02 * t.frame_length
        + DENSITY * SHELL_T * t.area_area
        + 3.0
        + AREA_MASS * t.plan_area;
    let above_base = total - 4.0 * 0.5 * DENSITY * 0.02 * 3.0;
    analyze_model(&mut b).unwrap();
    let rep = &b.case_results["EQ"].auto_lateral[0];
    assert_close("meshed W", rep.weight, above_base * G, 1e-9);
    let (cs, _) = manual_cs(&def, 1.0);
    assert_close("meshed V", rep.base_shear, cs * above_base * G, 1e-9);
    assert_close(
        "meshed ΣRx",
        -b.case_results["EQ"].base_reactions[0],
        rep.base_shear,
        1e-9,
    );
    // Generated joints take part: the report has more joints than the model.
    assert!(rep.joint_forces.len() > 8);
    let sum: f64 = rep.joint_forces.iter().map(|f| f.force).sum();
    assert_close("sum of forces", sum, rep.base_shear, 1e-12);
}
