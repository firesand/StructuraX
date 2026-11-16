//! Frame hinge properties, assignments and the results of nonlinear static
//! cases that save more than their final state.
//!
//! Source: CSI Analysis Reference Manual, chapters "Frame Hinge Properties"
//! and "Nonlinear Static Analysis". Only user-defined, uncoupled,
//! deformation-controlled hinges are analyzed; the other kinds the vendor
//! program offers (coupled P-M2-M3 and its subsets, fiber, parametric,
//! automatic, force controlled) have no verification problem in the vendor
//! material and are refused.

use crate::{FrameStationResult, SapModel, Units};
use indexmap::IndexMap;
use serde::{Deserialize, Serialize};

/// The frame-local degree of freedom an uncoupled hinge acts in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
pub enum HingeDof {
    P,
    V2,
    V3,
    T,
    M2,
    #[default]
    M3,
}

impl HingeDof {
    pub const ALL: [HingeDof; 6] = [
        HingeDof::P,
        HingeDof::V2,
        HingeDof::V3,
        HingeDof::T,
        HingeDof::M2,
        HingeDof::M3,
    ];

    /// Index into a frame-local `[P, V2, V3, T, M2, M3]` array.
    pub fn local_index(self) -> usize {
        match self {
            HingeDof::P => 0,
            HingeDof::V2 => 1,
            HingeDof::V3 => 2,
            HingeDof::T => 3,
            HingeDof::M2 => 4,
            HingeDof::M3 => 5,
        }
    }

    /// The vendor OAPI hinge type code of `cFrameObj.GetHingeAssigns`:
    /// 1 = P, 2 = V2, 3 = V3, 4 = T, 5 = M2, 6 = M3.
    pub fn oapi_code(self) -> i32 {
        self.local_index() as i32 + 1
    }

    /// `None` for the codes of coupled and fiber hinges (7 to 11) and for
    /// anything else.
    pub fn from_oapi_code(code: i32) -> Option<Self> {
        Self::ALL.into_iter().find(|dof| dof.oapi_code() == code)
    }

    /// True when the hinge deformation is a rotation (radians) and its force
    /// a moment; false for a displacement and a force.
    pub fn is_rotation(self) -> bool {
        matches!(self, HingeDof::T | HingeDof::M2 | HingeDof::M3)
    }

    pub fn label(self) -> &'static str {
        match self {
            HingeDof::P => "P",
            HingeDof::V2 => "V2",
            HingeDof::V3 => "V3",
            HingeDof::T => "T",
            HingeDof::M2 => "M2",
            HingeDof::M3 => "M3",
        }
    }

    pub fn from_label(label: &str) -> Option<Self> {
        Self::ALL
            .into_iter()
            .find(|dof| dof.label().eq_ignore_ascii_case(label.trim()))
    }
}

/// Deformation controlled (ductile) or force controlled (brittle). A force
/// controlled hinge can be stored but is not analyzed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum HingeBehavior {
    #[default]
    DeformationControlled,
    ForceControlled,
}

/// Load carrying capacity beyond point E of the curve.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum HingeBeyondE {
    #[default]
    DropsToZero,
    Extrapolated,
}

/// How a nonlinear static case sheds load when a hinge loses strength
/// (vendor OAPI `SetHingeUnloading`). Only the first method is analyzed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum HingeUnloading {
    #[default]
    UnloadEntireStructure,
    ApplyLocalRedistribution,
    RestartUsingSecantStiffness,
}

/// One point of a hinge curve, in table values: multiply by the scale
/// factors of the property to get a force (moment) and a deformation.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct HingePoint {
    pub force: f64,
    pub deformation: f64,
}

impl HingePoint {
    pub fn new(force: f64, deformation: f64) -> Self {
        Self { force, deformation }
    }
}

/// Acceptance criteria as table deformations (multiply by the deformation
/// scale factor). All zero means none were given. They are informational and
/// never change the behaviour of the structure.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct HingeAcceptance {
    pub io: f64,
    pub ls: f64,
    pub cp: f64,
}

impl HingeAcceptance {
    pub fn is_given(&self) -> bool {
        self.io != 0.0 || self.ls != 0.0 || self.cp != 0.0
    }
}

fn default_beyond_e() -> Option<HingeBeyondE> {
    Some(HingeBeyondE::DropsToZero)
}

/// A user-defined uncoupled hinge property: the A-B-C-D-E curve of one
/// degree of freedom. Point A is the origin. The hinge is rigid up to point
/// B; the deformation entered at B is not used and is subtracted from the
/// deformations of C, D and E.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HingeProp {
    pub name: String,
    pub dof: HingeDof,
    #[serde(default)]
    pub behavior: HingeBehavior,
    /// When true the negative direction mirrors `positive` (with the positive
    /// scale factors and acceptance criteria) and `negative` is not used.
    pub symmetric: bool,
    /// Points B, C, D and E for positive force.
    pub positive: [HingePoint; 4],
    /// Points -B, -C, -D and -E, entered with their signs (values <= 0).
    pub negative: [HingePoint; 4],
    pub force_sf: f64,
    pub deformation_sf: f64,
    pub force_sf_negative: f64,
    pub deformation_sf_negative: f64,
    pub acceptance_positive: HingeAcceptance,
    /// Entered with their signs (values <= 0).
    pub acceptance_negative: HingeAcceptance,
    /// `None` records an imported setting whose meaning is not established;
    /// the analysis then refuses to carry a hinge past point E.
    #[serde(default = "default_beyond_e")]
    pub beyond_e: Option<HingeBeyondE>,
}

impl HingeProp {
    /// A symmetric deformation-controlled property with unit scale factors
    /// and no acceptance criteria.
    pub fn symmetric(name: impl Into<String>, dof: HingeDof, curve: [HingePoint; 4]) -> Self {
        let mirrored = curve.map(|p| HingePoint::new(-p.force, -p.deformation));
        Self {
            name: name.into(),
            dof,
            behavior: HingeBehavior::DeformationControlled,
            symmetric: true,
            positive: curve,
            negative: mirrored,
            force_sf: 1.0,
            deformation_sf: 1.0,
            force_sf_negative: 1.0,
            deformation_sf_negative: 1.0,
            acceptance_positive: HingeAcceptance::default(),
            acceptance_negative: HingeAcceptance::default(),
            beyond_e: default_beyond_e(),
        }
    }
}

impl HingeProp {
    /// Checks the curve and the scale factors. Both branches are checked
    /// unless the property is symmetric, in which case the negative branch is
    /// not used. The curve is B, C, D, E as entered; only deformations beyond
    /// B are used, so they must not decrease; the force at B is positive and
    /// the forces at C, D and E are not negative; a point may not rise in
    /// force over no deformation; scale factors are positive; acceptance
    /// criteria are all zero (none given) or `0 < IO <= LS <= CP`.
    pub fn validate(&self) -> Result<(), String> {
        let name = &self.name;
        let branches: &[(&str, &[HingePoint; 4], f64, f64, &HingeAcceptance, f64)] = &[
            (
                "positive",
                &self.positive,
                self.force_sf,
                self.deformation_sf,
                &self.acceptance_positive,
                1.0,
            ),
            (
                "negative",
                &self.negative,
                self.force_sf_negative,
                self.deformation_sf_negative,
                &self.acceptance_negative,
                -1.0,
            ),
        ];
        for (label, points, force_sf, deformation_sf, acceptance, sign) in
            branches.iter().take(if self.symmetric { 1 } else { 2 })
        {
            let bad = |what: &str| format!("hinge property '{name}', {label} branch: {what}");
            if points
                .iter()
                .any(|p| !p.force.is_finite() || !p.deformation.is_finite())
                || !force_sf.is_finite()
                || !deformation_sf.is_finite()
                || [acceptance.io, acceptance.ls, acceptance.cp]
                    .iter()
                    .any(|v| !v.is_finite())
            {
                return Err(bad("a value is not finite"));
            }
            if *force_sf <= 0.0 || *deformation_sf <= 0.0 {
                return Err(bad("scale factors must be positive"));
            }
            // Magnitudes; the negative branch is entered with its signs.
            if points
                .iter()
                .any(|p| p.force * sign < 0.0 || p.deformation * sign < 0.0)
            {
                return Err(bad(if *sign > 0.0 {
                    "forces and deformations must not be negative"
                } else {
                    "forces and deformations must not be positive"
                }));
            }
            let mag: Vec<(f64, f64)> = points
                .iter()
                .map(|p| (p.deformation.abs(), p.force.abs()))
                .collect();
            if mag[0].1 <= 0.0 {
                return Err(bad("the force at point B must be positive"));
            }
            for pair in mag.windows(2) {
                if pair[1].0 < pair[0].0 {
                    return Err(bad("deformations must not decrease from B to E"));
                }
                if pair[1].0 == pair[0].0 && pair[1].1 > pair[0].1 {
                    return Err(bad("force rises with no deformation"));
                }
            }
            let (io, ls, cp) = (
                acceptance.io.abs(),
                acceptance.ls.abs(),
                acceptance.cp.abs(),
            );
            if (io, ls, cp) != (0.0, 0.0, 0.0) && !(0.0 < io && io <= ls && ls <= cp)
                || [acceptance.io, acceptance.ls, acceptance.cp]
                    .iter()
                    .any(|v| v * sign < 0.0)
            {
                return Err(bad(
                    "acceptance criteria must be all zero or 0 < IO <= LS <= CP",
                ));
            }
        }
        Ok(())
    }
}

/// A hinge assigned to a frame object.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FrameHinge {
    /// Name of a property in `SapModel::hinge_props`.
    pub prop: String,
    /// Relative distance along the object, measured from end I, in `[0, 1]`.
    pub rel_dist: f64,
}

/// The frame hinge overwrite: subdivide the object around its hinges so that
/// each hinge sits in a short element (vendor "Auto Subdivide Line Objects
/// At Hinges").
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FrameHingeOverwrite {
    pub auto_subdivide: bool,
    /// Length of the element containing a hinge, relative to the object.
    pub relative_length: f64,
    /// Vendor "Do Not Allow Hinges To Drop Loads". Stored, not analyzed.
    pub do_not_drop_load: bool,
}

impl Default for FrameHingeOverwrite {
    fn default() -> Self {
        Self {
            auto_subdivide: true,
            relative_length: 0.02,
            do_not_drop_load: false,
        }
    }
}

/// Which states of a nonlinear static case are kept.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum ResultsSaved {
    #[default]
    FinalState,
    MultipleStates {
        min_saved: usize,
        max_saved: usize,
        positive_only: bool,
    },
}

impl ResultsSaved {
    /// The vendor defaults when multiple states are first requested.
    pub fn multiple_default() -> Self {
        ResultsSaved::MultipleStates {
            min_saved: 10,
            max_saved: 100,
            positive_only: true,
        }
    }
}

/// A modal load of a nonlinear static case: joint forces proportional to the
/// mode shape times its circular frequency squared times the joint mass.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ModeLoad {
    /// Mode number, starting at 1, of the case named by `LoadCase::modal_case`.
    pub mode: usize,
    pub scale_factor: f64,
}

/// Parameters of a nonlinear static case beyond its load list and load
/// application control.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NonlinearStaticParams {
    pub results_saved: ResultsSaved,
    pub max_total_steps: usize,
    pub max_null_steps: usize,
    pub hinge_unloading: HingeUnloading,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub mode_loads: Vec<ModeLoad>,
}

impl Default for NonlinearStaticParams {
    fn default() -> Self {
        Self {
            results_saved: ResultsSaved::FinalState,
            max_total_steps: 200,
            max_null_steps: 50,
            hinge_unloading: HingeUnloading::UnloadEntireStructure,
            mode_loads: Vec::new(),
        }
    }
}

impl NonlinearStaticParams {
    pub fn is_default(&self) -> bool {
        *self == Self::default()
    }
}

/// The most extreme part of the curve a hinge has reached.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
pub enum HingeState {
    #[default]
    AToB,
    BToC,
    CToD,
    DToE,
    BeyondE,
}

/// The most extreme performance status a hinge has reached.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
pub enum HingeStatus {
    #[default]
    AToB,
    BToIO,
    IOToLS,
    LSToCP,
    BeyondCP,
}

/// The class a hinge is drawn in on the deformed shape, in the order of the
/// vendor legend. A hinge without plastic deformation is not drawn.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum HingeDisplayClass {
    BToIO,
    IOToLS,
    LSToCP,
    CPToC,
    CToD,
    DToE,
    BeyondE,
}

impl HingeDisplayClass {
    pub const ALL: [HingeDisplayClass; 7] = [
        HingeDisplayClass::BToIO,
        HingeDisplayClass::IOToLS,
        HingeDisplayClass::LSToCP,
        HingeDisplayClass::CPToC,
        HingeDisplayClass::CToD,
        HingeDisplayClass::DToE,
        HingeDisplayClass::BeyondE,
    ];

    pub fn label(self) -> &'static str {
        match self {
            HingeDisplayClass::BToIO => "B to IO",
            HingeDisplayClass::IOToLS => "IO to LS",
            HingeDisplayClass::LSToCP => "LS to CP",
            HingeDisplayClass::CPToC => "CP to C",
            HingeDisplayClass::CToD => "C to D",
            HingeDisplayClass::DToE => "D to E",
            HingeDisplayClass::BeyondE => ">E",
        }
    }
}

/// The result of one hinge at one state of a nonlinear static case.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct FrameHingeResult {
    pub frame: String,
    /// Number of the hinge on its frame, starting at 1.
    pub hinge: usize,
    /// Generated name, `{frame}H{hinge}`.
    pub name: String,
    pub prop: String,
    pub dof: HingeDof,
    pub rel_dist: f64,
    /// Force or moment carried by the hinge, in the frame sign convention.
    pub force: f64,
    /// Plastic displacement or rotation.
    pub plastic_deformation: f64,
    /// 0 while the hinge has never yielded, else the sign of the direction
    /// it yielded in.
    pub direction: i8,
    pub state: HingeState,
    pub status: HingeStatus,
}

impl FrameHingeResult {
    /// `None` for a hinge that has not yielded.
    pub fn display_class(&self) -> Option<HingeDisplayClass> {
        Some(match self.state {
            HingeState::AToB => return None,
            HingeState::BToC => match self.status {
                HingeStatus::AToB | HingeStatus::BToIO => HingeDisplayClass::BToIO,
                HingeStatus::IOToLS => HingeDisplayClass::IOToLS,
                HingeStatus::LSToCP => HingeDisplayClass::LSToCP,
                HingeStatus::BeyondCP => HingeDisplayClass::CPToC,
            },
            HingeState::CToD => HingeDisplayClass::CToD,
            HingeState::DToE => HingeDisplayClass::DToE,
            HingeState::BeyondE => HingeDisplayClass::BeyondE,
        })
    }
}

/// One saved state of a nonlinear static case. Step 0 is the state at the
/// start of the case.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct NonlinearStaticStep {
    pub step: usize,
    /// Factor on the loads of the case itself.
    pub load_factor: f64,
    /// Total value of the monitored displacement, in the joint-local system,
    /// for a displacement-controlled case.
    pub monitored_displacement: Option<f64>,
    pub base_reactions: [f64; 6],
    pub joint_displacements: IndexMap<String, [f64; 6]>,
    pub joint_reactions: IndexMap<String, [f64; 6]>,
    pub frame_forces: IndexMap<String, Vec<FrameStationResult>>,
    pub hinges: Vec<FrameHingeResult>,
}

/// The generated name of the `hinge`-th hinge (starting at 1) of a frame.
pub fn generated_name(frame: &str, hinge: usize) -> String {
    format!("{frame}H{hinge}")
}

/// Converts the dimensional values of hinge properties and of the
/// acceleration and modal loads of nonlinear static cases when the model
/// changes units. Curve points and acceptance criteria are table values
/// without a unit of their own.
///
/// A modal load is `sf w^2 M phi` with `phi' M phi = 1`, so the mode shape
/// has the unit 1/sqrt(mass) and the response to the load is `sf phi`: the
/// scale factor carries length times sqrt(mass).
pub fn scale_units(model: &mut SapModel, from: Units, to: Units) {
    let fl = from.length_factor_to(to);
    let mode_sf = fl * from.mass_factor_to(to).sqrt();
    let ff = from.force_factor_to(to);
    let fm = from.moment_factor_to(to);
    for prop in model.hinge_props.values_mut() {
        let (force, deformation) = if prop.dof.is_rotation() {
            (fm, 1.0)
        } else {
            (ff, fl)
        };
        prop.force_sf *= force;
        prop.force_sf_negative *= force;
        prop.deformation_sf *= deformation;
        prop.deformation_sf_negative *= deformation;
    }
    for case in model.load_cases.values_mut() {
        if case.case_type != crate::LoadCaseType::NonlinearStatic {
            continue;
        }
        for load in &mut case.loads {
            if crate::ritz::acceleration_direction(&load.load_name).is_some() {
                load.scale_factor *= fl;
            }
        }
        for load in &mut case.nl_static.mode_loads {
            load.scale_factor *= mode_sf;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn oapi_codes_cover_the_six_uncoupled_kinds_only() {
        for (code, dof) in (1..=6).zip(HingeDof::ALL) {
            assert_eq!(HingeDof::from_oapi_code(code), Some(dof));
            assert_eq!(dof.oapi_code(), code);
            assert_eq!(HingeDof::from_label(dof.label()), Some(dof));
        }
        for code in [0, 7, 8, 9, 10, 11, -1] {
            assert_eq!(HingeDof::from_oapi_code(code), None);
        }
        assert_eq!(HingeDof::from_label("PMM"), None);
    }

    #[test]
    fn display_class_follows_status_before_c_and_state_after() {
        let hinge = |state, status| FrameHingeResult {
            state,
            status,
            ..FrameHingeResult::default()
        };
        assert_eq!(hinge(HingeState::AToB, HingeStatus::AToB).display_class(), None);
        for (status, class) in [
            (HingeStatus::AToB, HingeDisplayClass::BToIO),
            (HingeStatus::BToIO, HingeDisplayClass::BToIO),
            (HingeStatus::IOToLS, HingeDisplayClass::IOToLS),
            (HingeStatus::LSToCP, HingeDisplayClass::LSToCP),
            (HingeStatus::BeyondCP, HingeDisplayClass::CPToC),
        ] {
            assert_eq!(hinge(HingeState::BToC, status).display_class(), Some(class));
        }
        for (state, class) in [
            (HingeState::CToD, HingeDisplayClass::CToD),
            (HingeState::DToE, HingeDisplayClass::DToE),
            (HingeState::BeyondE, HingeDisplayClass::BeyondE),
        ] {
            assert_eq!(
                hinge(state, HingeStatus::BeyondCP).display_class(),
                Some(class)
            );
        }
    }

    #[test]
    fn unit_change_scales_factors_and_the_loads_of_nonlinear_static_cases() {
        let mut model = SapModel::new(Units::KnMC);
        let curve = [
            HingePoint::new(1.0, 0.0),
            HingePoint::new(1.2, 4.0),
            HingePoint::new(0.2, 4.0),
            HingePoint::new(0.2, 8.0),
        ];
        let mut moment = HingeProp::symmetric("M", HingeDof::M3, curve);
        moment.force_sf = 100.0;
        moment.deformation_sf = 0.01;
        let mut shear = HingeProp::symmetric("V", HingeDof::V2, curve);
        shear.force_sf = 50.0;
        shear.deformation_sf = 0.002;
        model.hinge_props.insert("M".into(), moment.clone());
        model.hinge_props.insert("V".into(), shear.clone());
        let mut push = crate::LoadCase::linear_static("PUSH", "UX", 2.0);
        push.case_type = crate::LoadCaseType::NonlinearStatic;
        push.nl_static.mode_loads.push(ModeLoad {
            mode: 1,
            scale_factor: 3.0,
        });
        model.load_cases.insert("PUSH".into(), push);

        model.convert_units_in_place(Units::NMmC);
        let (m, v) = (&model.hinge_props["M"], &model.hinge_props["V"]);
        // kN m -> N mm is 1e6; kN -> N is 1e3; m -> mm is 1e3; radians stay.
        assert!((m.force_sf - 1.0e8).abs() < 1e-3);
        assert_eq!(m.deformation_sf, 0.01);
        assert!((v.force_sf - 5.0e4).abs() < 1e-6);
        assert!((v.deformation_sf - 2.0).abs() < 1e-12);
        assert_eq!(m.positive, moment.positive);
        assert_eq!(v.negative, shear.negative);
        let push = &model.load_cases["PUSH"];
        assert!((push.loads[0].scale_factor - 2000.0).abs() < 1e-9);
        // Length times sqrt(mass); kN s2/m and N s2/mm are the same mass.
        assert!((push.nl_static.mode_loads[0].scale_factor - 3000.0).abs() < 1e-9);
        // A linear case with the same load name is left alone.
        assert_eq!(model.load_cases["DEAD"].loads[0].scale_factor, 1.0);
    }

    fn valid() -> HingeProp {
        HingeProp::symmetric(
            "OK",
            HingeDof::M3,
            [
                HingePoint::new(1.0, 0.0),
                HingePoint::new(1.2, 4.0),
                HingePoint::new(0.2, 4.0),
                HingePoint::new(0.2, 8.0),
            ],
        )
    }

    #[test]
    fn validate_accepts_a_good_curve_including_drops_and_a_b_deformation() {
        valid().validate().unwrap();
        let mut p = valid();
        // Deformation at B is ignored by the analysis but may be nonzero.
        p.positive[0].deformation = 0.5;
        for k in 1..4 {
            p.positive[k].deformation += 0.5;
        }
        p.validate().unwrap();
        let mut p = valid();
        p.acceptance_positive = HingeAcceptance {
            io: 2.0,
            ls: 4.0,
            cp: 6.0,
        };
        p.validate().unwrap();
    }

    #[test]
    fn validate_rejects_each_broken_rule() {
        let mutate = |f: &dyn Fn(&mut HingeProp)| {
            let mut p = valid();
            f(&mut p);
            p.validate().unwrap_err()
        };
        assert!(mutate(&|p| p.positive[1].force = f64::NAN).contains("not finite"));
        assert!(mutate(&|p| p.positive[0].force = 0.0).contains("point B"));
        assert!(mutate(&|p| p.positive[2].force = -0.1).contains("negative"));
        assert!(mutate(&|p| p.positive[2].deformation = 3.0).contains("decrease"));
        // Rising over no deformation: C above B at the same deformation.
        assert!(mutate(&|p| p.positive[1].deformation = 0.0).contains("rises"));
        assert!(mutate(&|p| p.force_sf = 0.0).contains("scale factors"));
        assert!(mutate(&|p| p.deformation_sf = -1.0).contains("scale factors"));
        assert!(mutate(&|p| {
            p.acceptance_positive = HingeAcceptance {
                io: 4.0,
                ls: 2.0,
                cp: 6.0,
            }
        })
        .contains("acceptance"));
        assert!(mutate(&|p| {
            p.acceptance_positive = HingeAcceptance {
                io: 2.0,
                ls: 0.0,
                cp: 0.0,
            }
        })
        .contains("acceptance"));
    }

    #[test]
    fn validate_checks_the_negative_branch_only_when_not_symmetric() {
        let mut p = valid();
        p.negative[0].force = 5.0; // wrong sign, but unused
        p.validate().unwrap();
        p.symmetric = false;
        assert!(p.validate().unwrap_err().contains("negative branch"));
        p.negative = valid().negative;
        p.force_sf_negative = 0.0;
        assert!(p.validate().unwrap_err().contains("scale factors"));
        p.force_sf_negative = 160.0;
        p.validate().unwrap();
    }
}
