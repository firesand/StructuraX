//! Display units of result forms.
//!
//! The units selector of the application converts the model in place and
//! needs an unlocked model, which clears the results. A result form therefore
//! shows its values in display units of its own: the stored value times
//! [`factor`], labelled with [`label`]. Nothing is stored.
use sap_units::Units;

/// Physical dimension of a displayed quantity.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[allow(dead_code)] // Force and Moment: no result form shows a joint force or moment yet.
pub(super) enum Dimension {
    Length,
    Rotation,
    Force,
    /// Shell membrane and shear resultants (force per length).
    ForcePerLength,
    Moment,
    /// Shell moment resultants (moment per length, a force).
    MomentPerLength,
    Stress,
    Acceleration,
}

/// Multiplier that takes a value of dimension `dim` from `from` to `to`.
pub(super) fn factor(dim: Dimension, from: Units, to: Units) -> f64 {
    match dim {
        Dimension::Length | Dimension::Acceleration => from.length_factor_to(to),
        Dimension::Rotation => 1.0,
        Dimension::Force | Dimension::MomentPerLength => from.force_factor_to(to),
        Dimension::ForcePerLength => from.line_load_factor_to(to),
        Dimension::Moment => from.moment_factor_to(to),
        Dimension::Stress => from.stress_factor_to(to),
    }
}

/// Unit label of dimension `dim` in `units`, for example `kN/m2`.
pub(super) fn label(dim: Dimension, units: Units) -> String {
    let f = units.force_unit().symbol();
    let l = units.length_unit().symbol();
    match dim {
        Dimension::Length => l.to_string(),
        Dimension::Rotation => "rad".to_string(),
        Dimension::Force => f.to_string(),
        Dimension::ForcePerLength => format!("{f}/{l}"),
        Dimension::Moment => format!("{f}-{l}"),
        Dimension::MomentPerLength => format!("{f}-{l}/{l}"),
        Dimension::Stress => format!("{f}/{l}2"),
        Dimension::Acceleration => format!("{l}/s2"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn factors_follow_the_dimension() {
        let (from, to) = (Units::KnMC, Units::NMmC);
        assert_eq!(factor(Dimension::Length, from, to), 1000.0);
        assert_eq!(factor(Dimension::Acceleration, from, to), 1000.0);
        assert_eq!(factor(Dimension::Rotation, from, to), 1.0);
        assert_eq!(factor(Dimension::Force, from, to), 1000.0);
        assert_eq!(factor(Dimension::MomentPerLength, from, to), 1000.0);
        assert_eq!(factor(Dimension::ForcePerLength, from, to), 1.0);
        assert_eq!(factor(Dimension::Moment, from, to), 1.0e6);
        // 1 kN/m2 = 0.001 N/mm2 (MPa).
        assert!((factor(Dimension::Stress, from, to) - 1.0e-3).abs() <= 1e-18);
    }

    #[test]
    fn labels_name_force_and_length() {
        assert_eq!(label(Dimension::Stress, Units::NMmC), "N/mm2");
        assert_eq!(label(Dimension::MomentPerLength, Units::KnMC), "kN-m/m");
        assert_eq!(label(Dimension::ForcePerLength, Units::KipInF), "kip/in");
        assert_eq!(label(Dimension::Acceleration, Units::KnMC), "m/s2");
        assert_eq!(label(Dimension::Rotation, Units::KnMC), "rad");
    }

    #[test]
    fn stress_values_convert_between_unit_systems() {
        // 1750 kN/m2 = 1.75 N/mm2 (1 kN/m2 = 1e3 N / 1e6 mm2 = 1e-3 N/mm2).
        let v = 1750.0 * factor(Dimension::Stress, Units::KnMC, Units::NMmC);
        assert!((v - 1.75).abs() <= 1e-12, "{v}");
        // 1 ksi = 4.4482216152605 kN / (25.4 mm)^2 * 1e3 N/kN = 6.894757293168361 N/mm2.
        let f = factor(Dimension::Stress, Units::KipInF, Units::NMmC);
        assert!((f - 6.894_757_293_168_361).abs() <= 1e-12 * 6.9, "{f}");
        // Same units: identity.
        assert_eq!(factor(Dimension::Stress, Units::KnMC, Units::KnMC), 1.0);
        // A moment per length is a force: kip-in/in to kN-m/m = 4.4482216152605 kN / 1 = per inch
        // of length, so 1 kip-in/in = 4.4482216152605 kN-m/m.
        let m = factor(Dimension::MomentPerLength, Units::KipInF, Units::KnMC);
        assert!((m - 4.448_221_615_260_5).abs() <= 1e-12 * 4.45, "{m}");
        // A force per length: 1 kip/in = 4.4482216152605 kN / 0.0254 m = 175.12685 kN/m.
        let l = factor(Dimension::ForcePerLength, Units::KipInF, Units::KnMC);
        assert!((l - 4.448_221_615_260_5 / 0.0254).abs() <= 1e-12 * 175.2, "{l}");
    }
}
