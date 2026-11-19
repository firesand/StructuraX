//! Mass source and automatic seismic load definitions.

use crate::SapModel;
use sap_units::Units;
use serde::{Deserialize, Serialize};

/// Where the mass of the model comes from (CSI Analysis Reference, "Mass
/// Source"). The default is element self mass plus additional mass, with no
/// load pattern.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MassSource {
    /// Mass of frame, area, solid and link elements from their materials and
    /// properties.
    pub element_self_mass: bool,
    /// Mass assigned to joints and areas.
    pub additional_mass: bool,
    /// `(load pattern, multiplier)`: the downward load of each pattern divided
    /// by the gravitational acceleration is added as mass.
    pub load_patterns: Vec<(String, f64)>,
}

impl Default for MassSource {
    fn default() -> Self {
        Self {
            element_self_mass: true,
            additional_mass: true,
            load_patterns: Vec::new(),
        }
    }
}

impl MassSource {
    pub fn is_default(&self) -> bool {
        *self == Self::default()
    }
}

/// Global direction of an automatic lateral load.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum SeismicDirection {
    #[default]
    X,
    Y,
}

impl SeismicDirection {
    /// Index of the direction among the global translations `[X, Y, Z]`.
    pub fn index(self) -> usize {
        match self {
            Self::X => 0,
            Self::Y => 1,
        }
    }
}

/// Building period option of an automatic lateral load (Lateral Loads Manual,
/// section 2.14.1).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum SeismicPeriod {
    /// `T = Ct * hn^x` (ASCE 7-10 Eqn. 12.8-7), `hn` in feet, `Ct` always
    /// entered in English units as the code prints it.
    Approximate { ct: f64, x: f64 },
    /// The period entered by the user, in seconds.
    User { t: f64 },
    /// The period of the mode with the largest participation, limited by
    /// `Cu * Ta`. It can be stored but is refused at analysis.
    ProgramCalculated,
}

/// ASCE 7-10 automatic seismic load of one load pattern (Lateral Loads Manual,
/// section 2.14). Dimensional values (`z_min`, `z_max`) are in model length
/// units; `ct` is always in English units, `tl` and `t` in seconds, and
/// `ss`, `s1` in units of g.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct AutoSeismicAsce710 {
    /// Global direction of the load.
    pub direction: SeismicDirection,
    /// Diaphragm eccentricity ratio. The vendor ignores it where the model has
    /// no diaphragm. Where a Z diaphragm carries load, a moment of ratio times
    /// width times force is applied (see `AutoSeismicDiaphragmMoment`).
    pub eccentricity_ratio: f64,
    pub period: SeismicPeriod,
    /// Response modification factor R (Table 12.2-1).
    pub r: f64,
    /// System overstrength factor (Table 12.2-1). Stored; not used by the load.
    pub omega0: f64,
    /// Deflection amplification factor (Table 12.2-1). Stored; not used by the load.
    pub cd: f64,
    /// Occupancy importance factor Ie (Table 1.5-2), entered directly.
    pub ie: f64,
    /// Mapped short-period spectral acceleration, g.
    pub ss: f64,
    /// Mapped one-second spectral acceleration, g.
    pub s1: f64,
    /// Long-period transition period, seconds.
    pub tl: f64,
    /// Site coefficient Fa. `None` (site-class lookup) is refused at analysis
    /// because Table 11.4-1 is not implemented.
    pub fa: Option<f64>,
    /// Site coefficient Fv. `None` is refused (Table 11.4-2).
    pub fv: Option<f64>,
    /// Bottom elevation of the loaded range; `None` is the lowest joint.
    pub z_min: Option<f64>,
    /// Top elevation of the loaded range; `None` is the highest joint.
    pub z_max: Option<f64>,
}

impl Default for AutoSeismicAsce710 {
    fn default() -> Self {
        Self {
            direction: SeismicDirection::X,
            eccentricity_ratio: 0.05,
            period: SeismicPeriod::Approximate { ct: 0.02, x: 0.75 },
            r: 3.0,
            omega0: 3.0,
            cd: 3.0,
            ie: 1.0,
            ss: 1.0,
            s1: 0.4,
            tl: 8.0,
            fa: None,
            fv: None,
            z_min: None,
            z_max: None,
        }
    }
}

/// The force of an automatic lateral load at one joint.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct AutoSeismicJointForce {
    pub joint: String,
    /// Elevation of the joint, model length units.
    pub z: f64,
    /// Weight `m * g` of the joint in the load direction, force units.
    pub weight: f64,
    /// Applied global force in the load direction, force units.
    pub force: f64,
}

/// What an automatic seismic load computed in one analysis case. Values are
/// for the pattern at scale factor 1.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct AutoSeismicReport {
    pub pattern: String,
    pub direction: SeismicDirection,
    /// Period used, seconds.
    pub period: f64,
    /// Approximate period `Ct * hn^x`, when the option computes it.
    pub approximate_period: Option<f64>,
    /// Exponent of the vertical distribution.
    pub k: f64,
    pub sds: f64,
    pub sd1: f64,
    pub cs: f64,
    /// Equation that set `Cs`: "12.8-2", "12.8-3", "12.8-4", "12.8-5" or "12.8-6".
    pub governing_equation: String,
    /// Weight of the joints above the base, force units.
    pub weight: f64,
    /// Base shear `Cs * W`, force units.
    pub base_shear: f64,
    pub joint_forces: Vec<AutoSeismicJointForce>,
    /// Eccentric moments applied at diaphragms (empty without diaphragms or
    /// with a zero eccentricity).
    #[serde(default)]
    pub diaphragm_moments: Vec<AutoSeismicDiaphragmMoment>,
}

/// The eccentric moment of an automatic seismic load at one diaphragm
/// (Lateral Loads Manual, section 2.2.3). Project sign convention: a positive
/// eccentricity puts the line of action of the diaphragm's lateral force on the
/// positive side of the center of mass along the perpendicular global axis (+Y
/// for an X load, +X for a Y load), so the couple about global Z is
/// `-e F` for an X load and `+e F` for a Y load. The sign is not stated by the
/// manual.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct AutoSeismicDiaphragmMoment {
    /// The diaphragm constraint.
    pub diaphragm: String,
    /// Total lateral force of the pattern on the diaphragm, force units.
    pub force: f64,
    /// Width of the diaphragm perpendicular to the load, length units.
    pub width: f64,
    /// Eccentricity distance used (ratio times width, or the overwrite).
    pub eccentricity: f64,
    /// Whether `eccentricity` is an overwrite.
    pub overwritten: bool,
    /// Moment about global Z (force times length units), as applied.
    pub moment_z: f64,
    /// The joint the moment is applied at.
    pub joint: String,
}

/// Converts the dimensional values of the definitions in this module when the
/// model changes units. Only the elevation range carries a length; the other
/// inputs are dimensionless, in g, in seconds, or (Ct) always English.
pub fn scale_units(model: &mut SapModel, from: Units, to: Units) {
    let fl = from.length_factor_to(to);
    for def in model.auto_seismic.values_mut() {
        for z in [&mut def.z_min, &mut def.z_max].into_iter().flatten() {
            *z *= fl;
        }
    }
    for table in model.auto_seismic_eccentricity.values_mut() {
        for distance in table.values_mut() {
            *distance *= fl;
        }
    }
}
