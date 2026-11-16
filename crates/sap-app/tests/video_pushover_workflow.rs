//! Part 2 of the "SAP2000 Complete Mastery" tutorial video: the static
//! pushover of a four-storey plane frame, built and analyzed with the frame
//! hinges, chained nonlinear static cases, Accel and Mode pushover loads,
//! saved states, the pushover curve and S2K persistence.
//!
//! Model (kN, m): XZ frame, column lines x = 0, 4, 10, 14, floors z = 3, 6,
//! 9, 12, fixed bases, active UX, UZ, RY. Columns 0.30 x 0.30, beams
//! 0.30 x 0.40, E = 24 855 578 kN/m2, nu = 0.2, 23.5631 kN/m3. DEAD (self
//! weight), SDL 15 kN/m and LIVE 10 kN/m on every beam, LAT 50 kN per floor
//! (defined as in the video, not used by the push). Mass source: DEAD 1, SDL
//! 1, LIVE 0.25 and nothing else. Ritz modal case, 8 modes from UX and UZ.
//!
//! The video uses interacting P-M2-M3 hinges in the columns. This program
//! refuses coupled hinges, so uncoupled M3 hinges stand in for them: every
//! beam end carries "BEAM-M3" (elastic-perfectly-plastic, shape B (1, 0),
//! C (1, 0.25), D (1, 0.35), E (1, 0.5); 120 kN m in the positive and 160 kN m
//! in the negative direction through the force scale factors) and every
//! column end the symmetric "COL-M3" (300 kN m).
//!
//! The video's on-screen base shear of 400-450 kN is read off a screen for a
//! model whose data are not all stated; it is NOT a reference here. Every
//! reference is independent of the solver: statics, the uniqueness theorem
//! of plastic collapse (a load factor that is both statically admissible,
//! every hinge within its strength, and kinematically admissible, the virtual
//! work of a mechanism) and closed forms evaluated in the test.
//!
//! The collapse mechanism is never assumed. The test computes the
//! virtual-work load factor of a family of candidate mechanisms from its own
//! load vector (joint masses computed here from the mass source definition;
//! `sf omega^2 M phi` from the modal results) and requires the solver's
//! plateau factor to equal the smallest, with exactly that mechanism's
//! hinges moving.
//!
//! Tolerances: 1e-9 relative for every quantity that follows from linear
//! algebra on a handful of unknowns (load factors, plastic work, first yield),
//! the accuracy of the event-driven stepper at its tolerance; 1e-8 relative
//! to the largest load for the equilibrium of reactions at saved states
//! (sums of 16 reactions of a 150-unknown system); 1e-12 for a model that
//! has been written to S2K and read back (same input, same arithmetic). A
//! diaphragm is a penalty constraint of 1e6 times the largest diagonal
//! stiffness (`let penalty = max_diag * 1e6`), so tests that use one state
//! their measured tolerance next to the check.

use sap_model::hinge::*;
use sap_model::*;
use sap_solver::analyze_model;
use sap_units::Units;
use std::collections::{BTreeMap, BTreeSet};
use std::sync::OnceLock;

const XS: [f64; 4] = [0.0, 4.0, 10.0, 14.0];
const STOREY_H: f64 = 3.0;
const STOREYS: usize = 4;
const E_MOD: f64 = 24_855_578.0;
const GAMMA: f64 = 23.5631;
/// Standard gravity in kN, m, s units; the mass source divides weights by it.
const G_ACC: f64 = 9.80665;
const SDL: f64 = 15.0;
const LIVE: f64 = 10.0;
const LIVE_MASS_FACTOR: f64 = 0.25;
const COL_AREA: f64 = 0.09;
const BEAM_AREA: f64 = 0.12;
const MP_POS: f64 = 120.0;
const MP_NEG: f64 = 160.0;
const MP_COL: f64 = 300.0;
const TARGET: f64 = 1.2;

fn joint(i: usize, k: usize) -> String {
    format!("J{i}{k}")
}

/// Storey (floor) number of a joint name `J{i}{k}`.
fn floor_of(name: &str) -> usize {
    name[2..].parse().unwrap()
}

fn close(actual: f64, expected: f64, tol: f64, what: &str) {
    let scale = actual.abs().max(expected.abs()).max(1e-300);
    assert!(
        (actual - expected).abs() <= tol * scale,
        "{what}: got {actual}, expected {expected} (relative tolerance {tol})"
    );
}

fn close_to_scale(actual: f64, expected: f64, scale: f64, tol: f64, what: &str) {
    assert!(
        (actual - expected).abs() <= tol * scale,
        "{what}: got {actual}, expected {expected} (tolerance {tol} of {scale})"
    );
}

// ---------------------------------------------------------------------------
// The model
// ---------------------------------------------------------------------------

#[derive(Clone, Copy)]
struct Variant {
    /// Relative distances of the two beam hinges.
    beam_rel: [f64; 2],
    /// One diaphragm constraint per floor.
    diaphragms: bool,
}

const VIDEO: Variant = Variant {
    beam_rel: [0.0, 1.0],
    diaphragms: false,
};

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

fn pattern(m: &mut SapModel, name: &str, kind: LoadPatternType, self_weight: f64) {
    m.load_patterns.insert(
        name.into(),
        LoadPattern {
            name: name.into(),
            pat_type: kind,
            self_wt_multiplier: self_weight,
        },
    );
}

fn nl_case(name: &str, loads: &str, sf: f64, initial: Option<&str>) -> LoadCase {
    let mut c = LoadCase::linear_static(name, loads, sf);
    c.case_type = LoadCaseType::NonlinearStatic;
    c.initial_case = initial.map(str::to_string);
    c
}

fn push_case(name: &str) -> LoadCase {
    let mut c = nl_case(name, "UX", -1.0, Some("NL-LIVE"));
    c.displacement_control = Some(DisplacementControl {
        target: TARGET,
        monitored: Some((joint(0, STOREYS), 0)),
        conjugate: false,
    });
    c.nl_static.results_saved = ResultsSaved::MultipleStates {
        min_saved: 10,
        max_saved: 100,
        positive_only: true,
    };
    c.nl_static.max_total_steps = 1000;
    c
}

fn build(variant: Variant) -> SapModel {
    let mut m = SapModel::new(Units::KnMC);
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
    m.frame_sections
        .insert("COL".into(), rectangle("COL", 0.3, 0.3, COL_AREA));
    m.frame_sections
        .insert("BEAM".into(), rectangle("BEAM", 0.3, 0.4, BEAM_AREA));
    for (i, x) in XS.iter().enumerate() {
        for k in 0..=STOREYS {
            let name = m.add_point(joint(i, k), *x, 0.0, STOREY_H * k as f64);
            if k == 0 {
                m.points[&name].restraint = [true; 6];
            }
        }
    }
    pattern(&mut m, "DEAD", LoadPatternType::Dead, 1.0);
    pattern(&mut m, "SDL", LoadPatternType::SuperDead, 0.0);
    pattern(&mut m, "LIVE", LoadPatternType::Live, 0.0);
    pattern(&mut m, "LAT", LoadPatternType::Quake, 0.0);
    m.hinge_props.insert("BEAM-M3".into(), beam_hinge_prop());
    m.hinge_props.insert("COL-M3".into(), column_hinge_prop());
    let hinge = |prop: &str, rel: f64| FrameHinge {
        prop: prop.into(),
        rel_dist: rel,
    };
    for i in 0..XS.len() {
        for k in 0..STOREYS {
            let name = format!("C{i}{k}");
            m.add_frame(name.clone(), joint(i, k), joint(i, k + 1), "COL");
            m.frames[&name].hinges = vec![hinge("COL-M3", 0.0), hinge("COL-M3", 1.0)];
        }
    }
    for k in 1..=STOREYS {
        for i in 0..XS.len() - 1 {
            let name = format!("B{i}{k}");
            m.add_frame(name.clone(), joint(i, k), joint(i + 1, k), "BEAM");
            m.frames[&name].hinges = vec![
                hinge("BEAM-M3", variant.beam_rel[0]),
                hinge("BEAM-M3", variant.beam_rel[1]),
            ];
            for (pat, value) in [("SDL", SDL), ("LIVE", LIVE)] {
                m.frames[&name].loads_dist.push(FrameLoadDistributed {
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
        m.points[&joint(0, k)].loads_force.push(PointLoadForce {
            load_pat: "LAT".into(),
            csys: "GLOBAL".into(),
            force: [50.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        });
    }
    m.mass_source.element_self_mass = false;
    m.mass_source.additional_mass = false;
    m.mass_source.load_patterns = vec![
        ("DEAD".into(), 1.0),
        ("SDL".into(), 1.0),
        ("LIVE".into(), LIVE_MASS_FACTOR),
    ];
    if variant.diaphragms {
        for p in m.points.values_mut() {
            p.selected = p.z > 1e-9;
        }
        let made = m.assign_diaphragm("DIAPH", true).unwrap();
        assert_eq!(made.len(), STOREYS);
        for p in m.points.values_mut() {
            p.selected = false;
        }
    }

    let mut modal = LoadCase::modal_eigen("MODAL", 8);
    modal.modal_sub_type = ModalSubType::Ritz;
    for load in ["UX", "UZ"] {
        modal.loads.push(LoadCaseAssignment {
            load_name: load.into(),
            scale_factor: 1.0,
        });
        modal.ritz_controls.push(sap_model::ritz::RitzLoadControl {
            load_name: load.into(),
            max_cycles: 0,
            target_dynamic_participation: 0.95,
        });
    }
    let mut mode_push = nl_case("PUSH-MODE", "DEAD", 1.0, Some("NL-LIVE"));
    mode_push.loads.clear();
    mode_push.modal_case = Some("MODAL".into());
    mode_push.nl_static.mode_loads.push(ModeLoad {
        mode: 1,
        scale_factor: 1.0,
    });
    let push_acc = push_case("PUSH-ACC");
    mode_push.displacement_control = push_acc.displacement_control.clone();
    mode_push.nl_static.results_saved = push_acc.nl_static.results_saved;
    mode_push.nl_static.max_total_steps = 1000;
    // Every definition goes through the validation a user's definition gets.
    for case in [
        modal,
        nl_case("NL-DEAD", "DEAD", 1.0, None),
        nl_case("NL-SDL", "SDL", 1.0, Some("NL-DEAD")),
        nl_case("NL-LIVE", "LIVE", 1.0, Some("NL-SDL")),
        push_acc,
        mode_push,
    ] {
        let name = case.name.clone();
        sap_api::validate_case_definition(&m, &case)
            .unwrap_or_else(|e| panic!("case '{name}' fails validation: {e}"));
        sap_api::set_case_definition(&mut m, case)
            .unwrap_or_else(|e| panic!("case '{name}' is refused: {e}"));
    }
    m
}

fn analysed(variant: Variant) -> SapModel {
    let mut m = build(variant);
    analyze_model(&mut m).unwrap();
    m
}

fn video_model() -> &'static SapModel {
    static MODEL: OnceLock<SapModel> = OnceLock::new();
    MODEL.get_or_init(|| analysed(VIDEO))
}

// ---------------------------------------------------------------------------
// Statics computed in the test, from the geometry and the definitions
// ---------------------------------------------------------------------------

/// Members as (name, i joint, j joint, is column).
fn members() -> Vec<(String, String, String, bool)> {
    let mut v = Vec::new();
    for i in 0..XS.len() {
        for k in 0..STOREYS {
            v.push((format!("C{i}{k}"), joint(i, k), joint(i, k + 1), true));
        }
    }
    for k in 1..=STOREYS {
        for i in 0..XS.len() - 1 {
            v.push((format!("B{i}{k}"), joint(i, k), joint(i + 1, k), false));
        }
    }
    v
}

fn x_of(name: &str) -> f64 {
    XS[name[1..2].parse::<usize>().unwrap()]
}

fn z_of(name: &str) -> f64 {
    STOREY_H * floor_of(name) as f64
}

fn length(i: &str, j: &str) -> f64 {
    ((x_of(i) - x_of(j)).powi(2) + (z_of(i) - z_of(j)).powi(2)).sqrt()
}

/// Weight per unit length of a member under the full gravity loading.
fn gravity_per_length(column: bool) -> f64 {
    if column {
        GAMMA * COL_AREA
    } else {
        GAMMA * BEAM_AREA + SDL + LIVE
    }
}

/// Total weight and its moment about the global Y axis through the origin
/// (a downward force w at x gives +w x).
fn total_weight() -> (f64, f64) {
    members()
        .iter()
        .fold((0.0, 0.0), |(w, my), (_, i, j, col)| {
            let weight = gravity_per_length(*col) * length(i, j);
            (w + weight, my + weight * 0.5 * (x_of(i) + x_of(j)))
        })
}

/// Joint masses from the mass source (DEAD 1, SDL 1, LIVE 0.25, no element
/// mass on top): half the weight of each member, plus half of
/// (15 + 0.25 * 10) L of each beam, at each end, divided by g.
fn joint_masses() -> BTreeMap<String, f64> {
    let mut masses: BTreeMap<String, f64> = BTreeMap::new();
    for (_, i, j, col) in members() {
        let l = length(&i, &j);
        let w = if col {
            GAMMA * COL_AREA
        } else {
            GAMMA * BEAM_AREA + SDL + LIVE_MASS_FACTOR * LIVE
        } * l;
        for end in [i, j] {
            *masses.entry(end).or_default() += 0.5 * w / G_ACC;
        }
    }
    masses
}

/// Joint force (horizontal, vertical) of a push at unit load factor.
type PushLoads = BTreeMap<String, (f64, f64)>;

/// The applied load vector of a nonlinear static push, computed from its
/// definition: Accel is -sf M r, Mode is sf omega^2 M phi with phi^T M phi = 1.
fn push_loads(m: &SapModel, case: &str) -> PushLoads {
    let masses = joint_masses();
    let lc = &m.load_cases[case];
    let mut loads = PushLoads::new();
    if let Some(mode_load) = lc.nl_static.mode_loads.first() {
        let modal = &m.case_results[lc.modal_case.as_ref().unwrap()];
        let mode = &modal.modes[mode_load.mode - 1];
        let mu: f64 = masses
            .iter()
            .map(|(j, mass)| {
                let phi = mode.mode_shape[j.as_str()];
                mass * (phi[0] * phi[0] + phi[2] * phi[2])
            })
            .sum();
        close(mu, 1.0, 1e-8, "the stored mode shape has phi^T M phi = 1");
        let scale = mode_load.scale_factor * mode.circ_freq_rad_s.powi(2) / mu.sqrt();
        for (j, mass) in &masses {
            let phi = mode.mode_shape[j.as_str()];
            loads.insert(j.clone(), (scale * mass * phi[0], scale * mass * phi[2]));
        }
    } else {
        assert_eq!(lc.loads.len(), 1);
        assert_eq!(lc.loads[0].load_name, "UX");
        for (j, mass) in &masses {
            loads.insert(j.clone(), (-lc.loads[0].scale_factor * mass, 0.0));
        }
    }
    loads
}

fn strength(frame: &str, force: f64) -> f64 {
    if frame.starts_with('C') {
        MP_COL
    } else if force >= 0.0 {
        MP_POS
    } else {
        MP_NEG
    }
}

/// Force of hinge `hinge` (1: end I side, 2: end J side) of a member from a
/// linear static result, for hinges at the member ends only.
fn linear_end_force(case: &CaseResults, frame: &str, hinge: usize) -> f64 {
    let stations = &case.frame_forces[frame];
    if hinge == 1 {
        stations[0].m3
    } else {
        stations.last().unwrap().m3
    }
}

// ---------------------------------------------------------------------------
// Candidate mechanisms and their virtual work
// ---------------------------------------------------------------------------

type HingeKey = (String, usize);

/// A mechanism of rigid parts with the hinges that rotate, by unit of the
/// column rotation theta. Beams stay horizontal, so no joint moves
/// vertically and the vertical loads do no work.
struct Mechanism {
    name: String,
    hinges: BTreeSet<HingeKey>,
    /// Internal work per unit theta: the strength times the hinge rotation
    /// summed over the hinges.
    internal: f64,
    /// Horizontal displacement of floor k per unit theta.
    drift: [f64; STOREYS + 1],
}

impl Mechanism {
    fn external(&self, loads: &PushLoads) -> f64 {
        loads
            .iter()
            .map(|(j, (fx, _))| fx * self.drift[floor_of(j)])
            .sum()
    }

    /// Virtual-work load factor. Reversing the sense of the mechanism turns
    /// the external work negative and leaves the internal work unchanged (the
    /// beam end pair 120 + 160 and the symmetric columns do not care).
    fn factor(&self, loads: &PushLoads) -> f64 {
        let ext = self.external(loads).abs();
        if ext < 1e-300 {
            f64::INFINITY
        } else {
            self.internal / ext
        }
    }
}

/// `beam_factor` is the rotation of a beam hinge per unit column rotation:
/// 1 for hinges at the beam ends and 1 / (1 - 2 s) for hinges at s and 1 - s
/// from the ends (the rigid beam ends rotate with the joints by theta and the
/// middle segment turns by theta 2 s / (1 - 2 s) the other way).
fn mechanisms(beam_factor: f64) -> Vec<Mechanism> {
    let beam_pair = MP_POS + MP_NEG;
    let bases = |set: &mut BTreeSet<HingeKey>| {
        for i in 0..XS.len() {
            set.insert((format!("C{i}0"), 1));
        }
    };
    let beams_of_floor = |set: &mut BTreeSet<HingeKey>, k: usize| {
        for i in 0..XS.len() - 1 {
            set.insert((format!("B{i}{k}"), 1));
            set.insert((format!("B{i}{k}"), 2));
        }
    };
    let bays = (XS.len() - 1) as f64;
    let columns = XS.len() as f64;
    let mut out = Vec::new();
    // Beam sway: the column lines rotate rigidly about their bases up to the
    // roof; the beams of all four floors hinge at both ends.
    {
        let mut hinges = BTreeSet::new();
        bases(&mut hinges);
        for k in 1..=STOREYS {
            beams_of_floor(&mut hinges, k);
        }
        out.push(Mechanism {
            name: "beam sway of all four floors".into(),
            hinges,
            internal: columns * MP_COL + STOREYS as f64 * bays * beam_pair * beam_factor,
            drift: std::array::from_fn(|k| STOREY_H * k as f64),
        });
    }
    // Lower n storeys sway (n = 2..4; n = 1 is the first single-storey
    // mechanism): columns rotate about their bases up to floor n, the beams
    // of floors 1..n-1 hinge, the columns hinge at the top of storey n, and
    // everything above floor n translates with it.
    for n in 2..=STOREYS {
        let mut hinges = BTreeSet::new();
        bases(&mut hinges);
        for k in 1..n {
            beams_of_floor(&mut hinges, k);
        }
        for i in 0..XS.len() {
            hinges.insert((format!("C{i}{}", n - 1), 2));
        }
        out.push(Mechanism {
            name: format!("lower {n} storeys sway, column hinges at the top of storey {n}"),
            hinges,
            internal: 2.0 * columns * MP_COL + (n - 1) as f64 * bays * beam_pair * beam_factor,
            drift: std::array::from_fn(|k| STOREY_H * k.min(n) as f64),
        });
    }
    // Single-storey mechanisms: both ends of the columns of storey s.
    for s in 1..=STOREYS {
        let mut hinges = BTreeSet::new();
        for i in 0..XS.len() {
            hinges.insert((format!("C{i}{}", s - 1), 1));
            hinges.insert((format!("C{i}{}", s - 1), 2));
        }
        out.push(Mechanism {
            name: format!("single-storey mechanism of storey {s}"),
            hinges,
            internal: 2.0 * columns * MP_COL,
            drift: std::array::from_fn(|k| if k >= s { STOREY_H } else { 0.0 }),
        });
    }
    out
}

// ---------------------------------------------------------------------------
// The checks of one push
// ---------------------------------------------------------------------------

struct Tolerances {
    /// Equal load factors on the plateau, mechanism factor, moving set.
    plateau: f64,
    /// Equilibrium of reactions at every saved state, relative to the
    /// largest applied load.
    equilibrium: f64,
    /// External work against plastic work on the plateau.
    work: f64,
    /// Largest plastic increment of a hinge that is not part of the
    /// mechanism, relative to the largest increment. A hinge of the
    /// mechanism must exceed 1000 times this.
    spurious: f64,
}

const EXACT: Tolerances = Tolerances {
    plateau: 1e-9,
    equilibrium: 1e-8,
    work: 1e-9,
    spurious: 1e-9,
};

struct PushFacts {
    plateau: f64,
    mechanism: String,
    base_shear: f64,
    loads: PushLoads,
    sigma: f64,
    steps: usize,
}

fn roof(step: &NonlinearStaticStep) -> f64 {
    step.joint_displacements[joint(0, STOREYS).as_str()][0]
}

/// References 2 (a) to (e): the collapse load of a push by the uniqueness
/// theorem. `beam_rel` are the relative distances of the beam hinges.
fn verify_push(
    m: &SapModel,
    case: &str,
    beam_rel: [f64; 2],
    tol: &Tolerances,
    check_mechanism: bool,
) -> PushFacts {
    let results = &m.case_results[case];
    let steps = &results.static_steps;
    let n = steps.len();
    assert!(n >= 4, "{case}: {n} saved states");
    assert_eq!(steps[0].load_factor, 0.0);
    let loads = push_loads(m, case);
    let sum_fx: f64 = loads.values().map(|l| l.0).sum();
    let sum_fz: f64 = loads.values().map(|l| l.1).sum();
    assert!(sum_fx.abs() > 1.0, "{case}: no horizontal load");

    // The monitored displacement moves by the target in the direction the
    // load moves it, which is the direction of the horizontal load.
    let sigma = sum_fx.signum();
    let (first, last) = (&steps[0], &steps[n - 1]);
    close(
        (roof(last) - roof(first)) * sigma,
        TARGET,
        1e-9,
        "the roof moves by the target in the direction of the push",
    );
    let live = &m.case_results["NL-LIVE"];
    close_to_scale(
        roof(first),
        live.joint_displacements[joint(0, STOREYS).as_str()][0],
        TARGET,
        1e-9,
        "state 0 is the end of NL-LIVE",
    );

    let (weight, weight_moment) = total_weight();
    let lambda_last = last.load_factor;
    let largest = lambda_last * sum_fx.abs();

    for s in steps {
        let lam = s.load_factor;
        // (c) reactions balance the applied loads, in both force directions
        // and in the moment about the origin.
        close_to_scale(
            s.base_reactions[0],
            -lam * sum_fx,
            largest,
            tol.equilibrium,
            "horizontal equilibrium at a saved state",
        );
        close_to_scale(
            s.base_reactions[2],
            weight - lam * sum_fz,
            weight,
            tol.equilibrium,
            "vertical equilibrium at a saved state",
        );
        let applied_moment: f64 = weight_moment
            + loads
                .iter()
                .map(|(j, (fx, fz))| lam * (z_of(j) * fx - x_of(j) * fz))
                .sum::<f64>();
        let reaction_moment: f64 = (0..XS.len())
            .map(|i| {
                let r = s.joint_reactions[joint(i, 0).as_str()];
                r[4] - XS[i] * r[2]
            })
            .sum();
        close_to_scale(
            reaction_moment,
            -applied_moment,
            weight * XS[3],
            tol.equilibrium,
            "moment equilibrium at a saved state",
        );
        // (b) every hinge within the strength of its direction.
        for h in &s.hinges {
            assert!(
                h.force.abs() <= strength(&h.frame, h.force) * (1.0 + 1e-9),
                "{case} state {}: hinge {} of {} carries {}",
                s.step,
                h.hinge,
                h.frame,
                h.force
            );
        }
    }
    // The load factor never decreases.
    for pair in steps.windows(2) {
        assert!(
            pair[1].load_factor >= pair[0].load_factor - 1e-9 * lambda_last,
            "{case}: load factor decreased from {} to {}",
            pair[0].load_factor,
            pair[1].load_factor
        );
    }

    // (a) The push ends on a plateau.
    let (prev, last) = (&steps[n - 2], &steps[n - 1]);
    close(
        prev.load_factor,
        last.load_factor,
        tol.plateau,
        "the last two saved states carry the same load factor",
    );

    // (d) Work on the plateau. The elastic increments vanish there (the hinge
    // forces and the load do not change), so the structure moves as rigid
    // parts and every integral below is exact.
    let du = |j: &str, d: usize| last.joint_displacements[j][d] - prev.joint_displacements[j][d];
    let lam_mid = 0.5 * (prev.load_factor + last.load_factor);
    let mut external: f64 = loads
        .iter()
        .map(|(j, (fx, fz))| lam_mid * (fx * du(j, 0) + fz * du(j, 2)))
        .sum();
    for (_, i, j, col) in members() {
        let l = length(&i, &j);
        let w = gravity_per_length(col);
        let (vi, vj) = (du(&i, 2), du(&j, 2));
        let integral = if col {
            // A vertical rigid piece moves vertically as one body.
            l * 0.5 * (vi + vj)
        } else {
            // Rigid end pieces turn with the joints (a rotation RY of theta
            // lowers a point at distance x by theta x), the middle segment is
            // straight between the hinges.
            let (r0, r1) = (beam_rel[0], beam_rel[1]);
            let va = vi - du(&i, 4) * r0 * l;
            let vb = vj + du(&j, 4) * (1.0 - r1) * l;
            r0 * l * 0.5 * (vi + va)
                + (r1 - r0) * l * 0.5 * (va + vb)
                + (1.0 - r1) * l * 0.5 * (vb + vj)
        };
        external -= w * integral;
    }
    let mut plastic = 0.0;
    let mut plastic_scale = 0.0;
    let mut dqs: Vec<(HingeKey, f64)> = Vec::new();
    for (a, b) in prev.hinges.iter().zip(&last.hinges) {
        assert_eq!((&a.frame, a.hinge), (&b.frame, b.hinge));
        let dq = b.plastic_deformation - a.plastic_deformation;
        let force = 0.5 * (a.force + b.force);
        plastic += force * dq;
        plastic_scale += (force * dq).abs();
        dqs.push(((a.frame.clone(), a.hinge), dq));
    }
    close_to_scale(
        external,
        plastic,
        plastic_scale,
        tol.work,
        "external work equals the plastic work on the plateau",
    );
    let dq_max = dqs.iter().fold(0.0f64, |a, (_, dq)| a.max(dq.abs()));
    let moving: BTreeSet<HingeKey> = dqs
        .iter()
        .filter(|(_, dq)| dq.abs() > 1e3 * tol.spurious * dq_max)
        .map(|(k, _)| k.clone())
        .collect();
    // No hinge moves by a rounding-level amount: a clear gap separates them.
    let mut ratios: Vec<f64> = dqs.iter().map(|(_, dq)| dq.abs() / dq_max).collect();
    ratios.sort_by(f64::total_cmp);
    println!(
        "{case}: plastic increments relative to the largest: {:?}",
        ratios
            .iter()
            .map(|r| format!("{r:.1e}"))
            .collect::<Vec<_>>()
    );
    assert!(
        dqs.iter()
            .all(|(_, dq)| dq.abs() > 1e3 * tol.spurious * dq_max
                || dq.abs() < tol.spurious * dq_max),
        "{case}: increments of the hinges are not clearly separated"
    );

    // (e) The smallest virtual-work load factor of the candidates.
    let beam_factor = 1.0 / (1.0 - 2.0 * beam_rel[0].max(1.0 - beam_rel[1]));
    let candidates = mechanisms(beam_factor);
    let mut best: Option<(f64, &Mechanism)> = None;
    for c in &candidates {
        let lam = c.factor(&loads);
        println!("{case}: candidate '{}' -> load factor {lam}", c.name);
        // Upper-bound theorem: no candidate can lie below the solver's factor.
        assert!(
            lam >= last.load_factor * (1.0 - tol.plateau),
            "{case}: mechanism '{}' has load factor {lam} below the solver's {}",
            c.name,
            last.load_factor
        );
        if best.is_none_or(|(b, _)| lam < b) {
            best = Some((lam, c));
        }
    }
    let (best_lambda, mechanism) = best.unwrap();
    if check_mechanism {
        close(
            last.load_factor,
            best_lambda,
            tol.plateau,
            "plateau load factor equals the smallest virtual-work factor",
        );
        assert_eq!(
            moving, mechanism.hinges,
            "{case}: hinges moving on the plateau against those of '{}'",
            mechanism.name
        );
    }
    let base_shear = -last.base_reactions[0];
    println!(
        "{case}: mechanism '{}', {} hinges move, load factor {}, base shear {} kN, {n} saved states",
        mechanism.name,
        moving.len(),
        last.load_factor,
        base_shear
    );
    PushFacts {
        plateau: last.load_factor,
        mechanism: mechanism.name.clone(),
        base_shear,
        loads,
        sigma,
        steps: n,
    }
}

// ---------------------------------------------------------------------------
// Reference 3: first yield from two linear solves
// ---------------------------------------------------------------------------

/// The load factor at which the first hinge reaches its strength: gravity
/// (DEAD + SDL + LIVE) plus lambda times the push loads, both from linear
/// static solves of a copy of the model with the test's own load vectors.
fn first_yield_factor(m: &SapModel, loads: &PushLoads) -> f64 {
    let mut lin = m.clone();
    lin.load_cases.clear();
    lin.invalidate_results();
    pattern(&mut lin, "PUSHVEC", LoadPatternType::Other, 0.0);
    for (j, (fx, fz)) in loads {
        if !lin.points[j.as_str()].restraint[0] {
            lin.points[j.as_str()].loads_force.push(PointLoadForce {
                load_pat: "PUSHVEC".into(),
                csys: "GLOBAL".into(),
                force: [*fx, 0.0, *fz, 0.0, 0.0, 0.0],
            });
        }
    }
    let mut gravity = LoadCase::linear_static("GRAV", "DEAD", 1.0);
    for p in ["SDL", "LIVE"] {
        gravity.loads.push(LoadCaseAssignment {
            load_name: p.into(),
            scale_factor: 1.0,
        });
    }
    lin.load_cases.insert("GRAV".into(), gravity);
    lin.load_cases.insert(
        "UNITPUSH".into(),
        LoadCase::linear_static("UNITPUSH", "PUSHVEC", 1.0),
    );
    analyze_model(&mut lin).unwrap();
    let (grav, unit) = (&lin.case_results["GRAV"], &lin.case_results["UNITPUSH"]);
    let mut first = f64::INFINITY;
    for h in &m.case_results["NL-LIVE"].hinge_results {
        let fg = linear_end_force(grav, &h.frame, h.hinge);
        let g = linear_end_force(unit, &h.frame, h.hinge);
        if g.abs() < 1e-9 {
            continue;
        }
        let cap = strength(&h.frame, g);
        let target = if g > 0.0 { cap } else { -cap };
        first = first.min((target - fg) / g);
    }
    first
}

fn at_strength(h: &FrameHingeResult) -> bool {
    h.force.abs() >= strength(&h.frame, h.force) * (1.0 - 1e-9)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

/// Reference 1.
#[test]
fn gravity_chain_equals_the_linear_solution_and_carries_the_weight() {
    let mut m = build(VIDEO);
    // The linear case of the same loads.
    let mut lin = LoadCase::linear_static("LIN", "DEAD", 1.0);
    for p in ["SDL", "LIVE"] {
        lin.loads.push(LoadCaseAssignment {
            load_name: p.into(),
            scale_factor: 1.0,
        });
    }
    sap_api::set_case_definition(&mut m, lin).unwrap();
    analyze_model(&mut m).unwrap();
    let (live, linear) = (&m.case_results["NL-LIVE"], &m.case_results["LIN"]);
    // Both solve the same linear system; the chained case accumulates three
    // solutions, so allow 1e-9 of the largest value.
    let flat = |r: &CaseResults| -> Vec<f64> {
        let mut v: Vec<f64> = r.joint_displacements.values().flatten().copied().collect();
        v.extend(r.joint_reactions.values().flatten().copied());
        for stations in r.frame_forces.values() {
            for s in stations {
                v.extend([s.p, s.v2, s.m3]);
            }
        }
        v
    };
    let (a, b) = (flat(live), flat(linear));
    assert_eq!(a.len(), b.len());
    let scale = a.iter().fold(0.0f64, |m, x| m.max(x.abs()));
    let diff = a
        .iter()
        .zip(&b)
        .fold(0.0f64, |m, (x, y)| m.max((x - y).abs()));
    assert!(
        diff <= 1e-9 * scale,
        "chain differs from LIN by {diff} (scale {scale})"
    );

    let (weight, _) = total_weight();
    // 16 columns of 3 m and 12 beams of 14 m / 3 bays: the weight is also
    // the closed form of the video's section sizes.
    close(
        weight,
        GAMMA * (COL_AREA * 48.0 + BEAM_AREA * 56.0) + (SDL + LIVE) * 56.0,
        1e-12,
        "weight from the geometry",
    );
    close(
        live.base_reactions[2],
        weight,
        1e-9,
        "sum of vertical reactions",
    );
    close_to_scale(
        live.base_reactions[0],
        0.0,
        weight,
        1e-9,
        "no horizontal reaction",
    );
    let reactions: f64 = (0..XS.len())
        .map(|i| live.joint_reactions[joint(i, 0).as_str()][2])
        .sum();
    close(reactions, weight, 1e-9, "sum over the base joints");
    assert_eq!(live.hinge_results.len(), 56);
    assert!(live
        .hinge_results
        .iter()
        .all(|h| h.state == HingeState::AToB && h.direction == 0 && h.plastic_deformation == 0.0));
    // The earlier links of the chain carry their own weight only.
    close(
        m.case_results["NL-DEAD"].base_reactions[2],
        GAMMA * (COL_AREA * 48.0 + BEAM_AREA * 56.0),
        1e-9,
        "dead load reaction",
    );
    close(
        m.case_results["NL-SDL"].base_reactions[2],
        weight - LIVE * 56.0,
        1e-9,
        "dead plus superimposed dead reaction",
    );
    // The Ritz case reached its targets, or used all eight vectors.
    let modal = &m.case_results["MODAL"];
    assert!(modal.modes.len() <= 8 && !modal.modes.is_empty());
    // Only the starting loads carry a target; the table also lists the other
    // accelerations (UX..RZ), whose participation the Ritz vectors do not aim at.
    let starting = m.load_cases["MODAL"].loads.len();
    for part in modal.load_participation.iter().take(starting) {
        let reached = *part.dynamic_cumulative.last().unwrap();
        assert!(
            reached >= 0.95 || modal.modes.len() == 8,
            "{}: {reached}",
            part.load_name
        );
    }
    assert!(modal
        .modes
        .windows(2)
        .all(|w| w[0].period_sec >= w[1].period_sec));
}

/// References 2, 3 and 4 for the Accel push.
#[test]
fn push_with_accel_load_collapses_in_the_mechanism_the_virtual_work_selects() {
    let m = video_model();
    let facts = verify_push(m, "PUSH-ACC", VIDEO.beam_rel, &EXACT, true);
    assert_eq!(facts.sigma, 1.0);
    // The Accel load is the joint masses: the base shear is lambda times the
    // total mass (base joints included) times |sf| = 1.
    let total_mass: f64 = joint_masses().values().sum();
    close(
        facts.base_shear,
        facts.plateau * total_mass,
        1e-9,
        "base shear on the plateau is lambda times the total mass",
    );
    first_yield_and_curve(m, "PUSH-ACC", &facts);
}

/// References 2, 3 and 4 for the Mode push.
#[test]
fn push_with_mode_load_collapses_in_the_mechanism_the_virtual_work_selects() {
    let m = video_model();
    let facts = verify_push(m, "PUSH-MODE", VIDEO.beam_rel, &EXACT, true);
    println!(
        "PUSH-MODE pushes toward {}X",
        if facts.sigma > 0.0 { "+" } else { "-" }
    );
    first_yield_and_curve(m, "PUSH-MODE", &facts);
}

fn first_yield_and_curve(m: &SapModel, case: &str, facts: &PushFacts) {
    let steps = &m.case_results[case].static_steps;
    // Reference 3. The first saved state at which a hinge reaches its
    // strength is the event itself (events are saved states).
    let expected = first_yield_factor(m, &facts.loads);
    let at = steps
        .iter()
        .position(|s| s.hinges.iter().any(at_strength))
        .expect("a hinge yields");
    close(
        steps[at].load_factor,
        expected,
        1e-9,
        "first yield load factor from two linear solutions",
    );
    // Hinges leave A-B only after the first yield, at a load factor that is
    // not below it.
    let left = steps
        .iter()
        .position(|s| s.hinges.iter().any(|h| h.state != HingeState::AToB))
        .expect("a hinge leaves A-B");
    assert!(left >= at);
    assert!(steps[left].load_factor >= expected * (1.0 - 1e-9));
    for s in &steps[..at] {
        assert!(s
            .hinges
            .iter()
            .all(|h| h.state == HingeState::AToB && h.plastic_deformation == 0.0));
    }
    println!(
        "{case}: first yield at load factor {expected}, base shear {} kN; first hinge off A-B in state {left} at load factor {}",
        expected * facts.loads.values().map(|l| l.0).sum::<f64>(),
        steps[left].load_factor
    );

    // Reference 4: the pushover curve.
    let curve = sap_api::pushover_curve(m, case).unwrap();
    assert_eq!(curve.len(), steps.len());
    let sum_fx: f64 = facts.loads.values().map(|l| l.0).sum();
    let largest = steps.last().unwrap().load_factor * sum_fx.abs();
    for (row, s) in curve.iter().zip(steps) {
        assert_eq!(row.step, s.step);
        close_to_scale(
            row.displacement,
            s.monitored_displacement.unwrap(),
            TARGET,
            1e-8,
            "curve displacement",
        );
        close_to_scale(
            row.base_shear,
            s.load_factor * sum_fx,
            largest,
            1e-8,
            "curve base shear is the sum of the push forces times the load factor",
        );
        assert_eq!(row.total, 56);
        assert_eq!(
            row.a_to_b + row.b_to_c + row.c_to_d + row.d_to_e + row.beyond_e,
            56
        );
        assert_eq!(
            row.a_to_io + row.io_to_ls + row.ls_to_cp + row.beyond_cp,
            56
        );
        let off_a = s
            .hinges
            .iter()
            .filter(|h| h.state != HingeState::AToB)
            .count();
        assert_eq!(56 - row.a_to_b, off_a);
    }
    for pair in curve.windows(2) {
        assert!(
            facts.sigma * (pair[1].base_shear - pair[0].base_shear) >= -1e-9 * largest,
            "base shear decreases between states {} and {}",
            pair[0].step,
            pair[1].step
        );
    }
}

/// Reference 5.
#[test]
fn s2k_round_trip_keeps_the_definitions_and_every_saved_state() {
    let m = video_model();
    let text = sap_format::write_s2k(m);
    let mut reopened = sap_format::parse_s2k(&text).unwrap();
    assert_eq!(reopened.hinge_props, m.hinge_props);
    assert_eq!(reopened.mass_source, m.mass_source);
    for (name, frame) in &m.frames {
        assert_eq!(
            reopened.frames[name].hinges, frame.hinges,
            "hinges of {name}"
        );
        assert_eq!(reopened.frames[name].hinge_overwrite, frame.hinge_overwrite);
    }
    for name in [
        "MODAL",
        "NL-DEAD",
        "NL-SDL",
        "NL-LIVE",
        "PUSH-ACC",
        "PUSH-MODE",
    ] {
        assert_eq!(reopened.load_cases[name], m.load_cases[name], "case {name}");
    }
    analyze_model(&mut reopened).unwrap();
    for case in ["PUSH-ACC", "PUSH-MODE"] {
        let (a, b) = (
            &reopened.case_results[case].static_steps,
            &m.case_results[case].static_steps,
        );
        assert_eq!(a.len(), b.len(), "{case}: number of saved states");
        for (x, y) in a.iter().zip(b) {
            close_to_scale(
                x.load_factor,
                y.load_factor,
                b.last().unwrap().load_factor,
                1e-12,
                "load factor after reopening",
            );
            let scale = y
                .joint_displacements
                .values()
                .flatten()
                .fold(0.0f64, |s, v| s.max(v.abs()));
            for (name, disp) in &y.joint_displacements {
                for d in 0..6 {
                    close_to_scale(
                        x.joint_displacements[name][d],
                        disp[d],
                        scale,
                        1e-12,
                        "displacement after reopening",
                    );
                }
            }
            assert_eq!(x.hinges.len(), y.hinges.len());
            for (h, k) in x.hinges.iter().zip(&y.hinges) {
                assert_eq!((h.state, h.direction), (k.state, k.direction));
            }
        }
    }
    close(
        reopened.case_results["MODAL"].modes[0].period_sec,
        m.case_results["MODAL"].modes[0].period_sec,
        1e-12,
        "first period after reopening",
    );
}

/// Reference 6.
#[test]
fn diaphragm_variant_has_the_same_plateau_factor() {
    let plain = video_model();
    let m = analysed(Variant {
        diaphragms: true,
        ..VIDEO
    });
    // The Accel load does not depend on the stiffness, so the constrained
    // frame must reach the same plateau factor, but the diaphragm is a
    // penalty constraint (1e6 times the largest diagonal stiffness), so the
    // floors are rigid only to the accuracy of that penalty. Measured on
    // this frame (realistic areas, see `rectangle`): the plateau factor of
    // PUSH-ACC differs by 1.4e-6 relative, a reaction balance at a saved state
    // by 1.1e-6 of the largest load, and the hinges outside the mechanism
    // move by up to 8e-7 of the largest plastic increment. Tolerances: 1e-5
    // (about seven times the largest measured difference) instead of 1e-9,
    // and 1e-5 for the spurious increments.
    let tol = Tolerances {
        plateau: 1e-5,
        equilibrium: 1e-5,
        work: 1e-5,
        spurious: 1e-5,
    };
    let with = verify_push(&m, "PUSH-ACC", VIDEO.beam_rel, &tol, true);
    let without = plain.case_results["PUSH-ACC"]
        .static_steps
        .last()
        .unwrap()
        .load_factor;
    let measured = (with.plateau - without).abs() / without;
    println!(
        "diaphragm, PUSH-ACC: plateau {} against {without}, relative difference {measured:e}",
        with.plateau
    );
    assert!(measured <= 1e-5, "relative difference {measured:e}");
    // The Mode load depends on the mode shape, which the diaphragms change.
    // Its collapse load is checked against the variant's own virtual-work
    // factor (reference 2 with the variant's load vector).
    let mode = verify_push(&m, "PUSH-MODE", VIDEO.beam_rel, &tol, true);
    println!("diaphragm, PUSH-MODE: plateau {}", mode.plateau);
}

/// Reference 7.
#[test]
fn beam_hinges_at_a_tenth_and_nine_tenths_collapse_with_the_stub_factor() {
    let m = analysed(Variant {
        beam_rel: [0.1, 0.9],
        diaphragms: false,
    });
    for case in ["PUSH-ACC", "PUSH-MODE"] {
        // The closed form: a mechanism rotates its beam hinges by
        // 1 / (1 - 2 s) = 1.25 times the column rotation, because the stubs
        // turn with the joints and the middle segment the other way by a
        // quarter of it; the stubs and the middle segment move
        // antisymmetrically about the beam midpoint, so the uniform gravity
        // load does no work. `verify_push` with the stub factor checks
        // 2 (a) to (e), the work balance including the gravity work from the
        // rigid-segment kinematics.
        let facts = verify_push(&m, case, [0.1, 0.9], &EXACT, true);
        let plain = video_model().case_results[case]
            .static_steps
            .last()
            .unwrap()
            .load_factor;
        println!(
            "stub hinges, {case}: {} (video model: {plain})",
            facts.plateau
        );
        assert!(
            facts.plateau >= plain * (1.0 - 1e-9),
            "{case}: stubs cannot lower the collapse load"
        );
        // The candidate that governs, evaluated with the stub factor.
        let mechanisms = mechanisms(1.25);
        let lam = mechanisms
            .iter()
            .find(|c| c.name == facts.mechanism)
            .unwrap()
            .factor(&facts.loads);
        close(
            lam,
            facts.plateau,
            1e-9,
            "closed form of the reported mechanism",
        );
        assert!(facts.steps >= 4);
    }
}
