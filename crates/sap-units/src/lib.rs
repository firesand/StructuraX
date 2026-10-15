use serde::{Deserialize, Serialize};
use std::fmt;
use std::str::FromStr;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum UnitError {
    #[error("Unknown SAP2000 unit code: {0}")]
    InvalidCode(i32),
    #[error("Unknown SAP2000 unit string: '{0}'")]
    InvalidString(String),
}

/// Exact match for `SAP2000v19.eUnits` (1..=16) and `S_ProjectStartUp.m_Units`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
#[repr(i32)]
pub enum Units {
    LbInF = 1,
    LbFtF = 2,
    #[default]
    KipInF = 3,
    KipFtF = 4,
    KnMmC = 5,
    KnMC = 6,
    KgfMmC = 7,
    KgfMC = 8,
    NMmC = 9,
    NMC = 10,
    TonMmC = 11,
    TonMC = 12,
    KnCmC = 13,
    KgfCmC = 14,
    NCmC = 15,
    TonCmC = 16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ForceUnit {
    Lb,
    Kip,
    N,
    Kn,
    Kgf,
    Tonf,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum LengthUnit {
    In,
    Ft,
    Mm,
    Cm,
    M,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TempUnit {
    F,
    C,
}

impl ForceUnit {
    /// Multiplier to convert 1 unit of `self` into Newtons (N).
    /// Matches `QuickAnalysis.dll` (`_FactorForceToKN * 1000.0`) and `sapgraph.dll` (`cUnits`).
    #[inline]
    pub const fn to_newtons(self) -> f64 {
        match self {
            ForceUnit::Lb => 4.448_221_615_260_5,
            ForceUnit::Kip => 4_448.221_615_260_5,
            ForceUnit::N => 1.0,
            ForceUnit::Kn => 1_000.0,
            ForceUnit::Kgf => 9.806_65,
            ForceUnit::Tonf => 9_806.65,
        }
    }

    pub const fn symbol(self) -> &'static str {
        match self {
            ForceUnit::Lb => "lb",
            ForceUnit::Kip => "kip",
            ForceUnit::N => "N",
            ForceUnit::Kn => "kN",
            ForceUnit::Kgf => "kgf",
            ForceUnit::Tonf => "Tonf",
        }
    }

    pub const fn s2k_token(self) -> &'static str {
        match self {
            ForceUnit::Lb => "LB",
            ForceUnit::Kip => "KIP",
            ForceUnit::N => "N",
            ForceUnit::Kn => "KN",
            ForceUnit::Kgf => "KGF",
            ForceUnit::Tonf => "TON",
        }
    }
}

impl LengthUnit {
    /// Multiplier to convert 1 unit of `self` into meters (m).
    #[inline]
    pub const fn to_meters(self) -> f64 {
        match self {
            LengthUnit::In => 0.0254,
            LengthUnit::Ft => 0.3048,
            LengthUnit::Mm => 0.001,
            LengthUnit::Cm => 0.01,
            LengthUnit::M => 1.0,
        }
    }

    /// Parses `.pro` header unit float (`1.0`=in, `2.0`=ft, `3.0`=mm, `4.0`=cm, `5.0`=m).
    pub fn from_pro_code(code: f32) -> Option<Self> {
        match code.round() as i32 {
            1 => Some(LengthUnit::In),
            2 => Some(LengthUnit::Ft),
            3 => Some(LengthUnit::Mm),
            4 => Some(LengthUnit::Cm),
            5 => Some(LengthUnit::M),
            _ => None,
        }
    }

    pub const fn symbol(self) -> &'static str {
        match self {
            LengthUnit::In => "in",
            LengthUnit::Ft => "ft",
            LengthUnit::Mm => "mm",
            LengthUnit::Cm => "cm",
            LengthUnit::M => "m",
        }
    }

    pub const fn s2k_token(self) -> &'static str {
        match self {
            LengthUnit::In => "IN",
            LengthUnit::Ft => "FT",
            LengthUnit::Mm => "MM",
            LengthUnit::Cm => "CM",
            LengthUnit::M => "M",
        }
    }
}

impl TempUnit {
    /// Multiplier to convert a temperature difference $\Delta T$ into Kelvin/Celsius ($\Delta^\circ\text{C}$).
    #[inline]
    pub const fn delta_to_celsius(self) -> f64 {
        match self {
            TempUnit::F => 5.0 / 9.0,
            TempUnit::C => 1.0,
        }
    }

    pub const fn symbol(self) -> &'static str {
        match self {
            TempUnit::F => "F",
            TempUnit::C => "C",
        }
    }
}

impl Units {
    pub const ALL: [Units; 16] = [
        Units::LbInF,
        Units::LbFtF,
        Units::KipInF,
        Units::KipFtF,
        Units::KnMmC,
        Units::KnMC,
        Units::KgfMmC,
        Units::KgfMC,
        Units::NMmC,
        Units::NMC,
        Units::TonMmC,
        Units::TonMC,
        Units::KnCmC,
        Units::KgfCmC,
        Units::NCmC,
        Units::TonCmC,
    ];

    pub const fn from_code(code: i32) -> Result<Self, UnitError> {
        match code {
            1 => Ok(Units::LbInF),
            2 => Ok(Units::LbFtF),
            3 => Ok(Units::KipInF),
            4 => Ok(Units::KipFtF),
            5 => Ok(Units::KnMmC),
            6 => Ok(Units::KnMC),
            7 => Ok(Units::KgfMmC),
            8 => Ok(Units::KgfMC),
            9 => Ok(Units::NMmC),
            10 => Ok(Units::NMC),
            11 => Ok(Units::TonMmC),
            12 => Ok(Units::TonMC),
            13 => Ok(Units::KnCmC),
            14 => Ok(Units::KgfCmC),
            15 => Ok(Units::NCmC),
            16 => Ok(Units::TonCmC),
            _ => Err(UnitError::InvalidCode(code)),
        }
    }

    pub const fn code(self) -> i32 {
        self as i32
    }

    pub const fn components(self) -> (ForceUnit, LengthUnit, TempUnit) {
        match self {
            Units::LbInF => (ForceUnit::Lb, LengthUnit::In, TempUnit::F),
            Units::LbFtF => (ForceUnit::Lb, LengthUnit::Ft, TempUnit::F),
            Units::KipInF => (ForceUnit::Kip, LengthUnit::In, TempUnit::F),
            Units::KipFtF => (ForceUnit::Kip, LengthUnit::Ft, TempUnit::F),
            Units::KnMmC => (ForceUnit::Kn, LengthUnit::Mm, TempUnit::C),
            Units::KnMC => (ForceUnit::Kn, LengthUnit::M, TempUnit::C),
            Units::KgfMmC => (ForceUnit::Kgf, LengthUnit::Mm, TempUnit::C),
            Units::KgfMC => (ForceUnit::Kgf, LengthUnit::M, TempUnit::C),
            Units::NMmC => (ForceUnit::N, LengthUnit::Mm, TempUnit::C),
            Units::NMC => (ForceUnit::N, LengthUnit::M, TempUnit::C),
            Units::TonMmC => (ForceUnit::Tonf, LengthUnit::Mm, TempUnit::C),
            Units::TonMC => (ForceUnit::Tonf, LengthUnit::M, TempUnit::C),
            Units::KnCmC => (ForceUnit::Kn, LengthUnit::Cm, TempUnit::C),
            Units::KgfCmC => (ForceUnit::Kgf, LengthUnit::Cm, TempUnit::C),
            Units::NCmC => (ForceUnit::N, LengthUnit::Cm, TempUnit::C),
            Units::TonCmC => (ForceUnit::Tonf, LengthUnit::Cm, TempUnit::C),
        }
    }

    pub const fn force_unit(self) -> ForceUnit {
        self.components().0
    }

    pub const fn length_unit(self) -> LengthUnit {
        self.components().1
    }

    pub const fn temp_unit(self) -> TempUnit {
        self.components().2
    }

    /// Standard SAP2000 `.s2k` `CurrUnits` representation, e.g., `"Kip, in, F"`.
    pub const fn s2k_label(self) -> &'static str {
        match self {
            Units::LbInF => "lb, in, F",
            Units::LbFtF => "lb, ft, F",
            Units::KipInF => "Kip, in, F",
            Units::KipFtF => "Kip, ft, F",
            Units::KnMmC => "kN, mm, C",
            Units::KnMC => "kN, m, C",
            Units::KgfMmC => "kgf, mm, C",
            Units::KgfMC => "kgf, m, C",
            Units::NMmC => "N, mm, C",
            Units::NMC => "N, m, C",
            Units::TonMmC => "Tonf, mm, C",
            Units::TonMC => "Tonf, m, C",
            Units::KnCmC => "kN, cm, C",
            Units::KgfCmC => "kgf, cm, C",
            Units::NCmC => "N, cm, C",
            Units::TonCmC => "Tonf, cm, C",
        }
    }

    /// Multiplier to convert a length from `self` to `target`.
    #[inline]
    pub fn length_factor_to(self, target: Units) -> f64 {
        self.length_unit().to_meters() / target.length_unit().to_meters()
    }

    /// Multiplier to convert a force from `self` to `target`.
    #[inline]
    pub fn force_factor_to(self, target: Units) -> f64 {
        self.force_unit().to_newtons() / target.force_unit().to_newtons()
    }

    /// Multiplier to convert a temperature change $\Delta T$ from `self` to `target`.
    #[inline]
    pub fn temp_delta_factor_to(self, target: Units) -> f64 {
        self.temp_unit().delta_to_celsius() / target.temp_unit().delta_to_celsius()
    }

    /// Multiplier to convert stress / pressure / Elastic Modulus $E, G, F_y$ ($F / L^2$) from `self` to `target`.
    #[inline]
    pub fn stress_factor_to(self, target: Units) -> f64 {
        let f = self.force_factor_to(target);
        let l = self.length_factor_to(target);
        f / (l * l)
    }

    /// Multiplier to convert bending/torsional moment ($F \cdot L$) from `self` to `target`.
    #[inline]
    pub fn moment_factor_to(self, target: Units) -> f64 {
        self.force_factor_to(target) * self.length_factor_to(target)
    }

    /// Multiplier to convert distributed line load ($F / L$) from `self` to `target`.
    #[inline]
    pub fn line_load_factor_to(self, target: Units) -> f64 {
        self.force_factor_to(target) / self.length_factor_to(target)
    }

    /// Multiplier to convert unit weight $\gamma$ ($F / L^3$) from `self` to `target`.
    #[inline]
    pub fn unit_weight_factor_to(self, target: Units) -> f64 {
        let l = self.length_factor_to(target);
        self.force_factor_to(target) / (l * l * l)
    }

    /// Multiplier to convert translational mass ($F \cdot s^2 / L$) from `self` to `target`.
    #[inline]
    pub fn mass_factor_to(self, target: Units) -> f64 {
        self.force_factor_to(target) / self.length_factor_to(target)
    }

    /// Multiplier to convert mass density $\rho$ ($F \cdot s^2 / L^4$) from `self` to `target`.
    #[inline]
    pub fn mass_density_factor_to(self, target: Units) -> f64 {
        let l = self.length_factor_to(target);
        self.force_factor_to(target) / (l * l * l * l)
    }

    /// Multiplier to convert coefficient of thermal expansion $\alpha$ ($1 / \Delta T$) from `self` to `target`.
    #[inline]
    pub fn thermal_expansion_factor_to(self, target: Units) -> f64 {
        1.0 / self.temp_delta_factor_to(target)
    }

    /// Standard gravitational acceleration $g$ expressed in `self`'s length/$s^2$.
    #[inline]
    pub fn gravity_acceleration(self) -> f64 {
        9.806_65 / self.length_unit().to_meters()
    }
}

impl fmt::Display for Units {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.s2k_label())
    }
}

impl FromStr for Units {
    type Err = UnitError;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let cleaned = s
            .trim()
            .trim_matches('"')
            .to_ascii_uppercase()
            .replace([' ', ',', '-', '_'], "");
        match cleaned.as_str() {
            "LBINF" | "LBIN" => Ok(Units::LbInF),
            "LBFTF" | "LBFT" => Ok(Units::LbFtF),
            "KIPINF" | "KIPIN" => Ok(Units::KipInF),
            "KIPFTF" | "KIPFT" => Ok(Units::KipFtF),
            "KNMMC" | "KNMM" => Ok(Units::KnMmC),
            "KNMC" | "KNM" => Ok(Units::KnMC),
            "KGFMMC" | "KGFMM" => Ok(Units::KgfMmC),
            "KGFMC" | "KGFM" => Ok(Units::KgfMC),
            "NMMC" | "NMM" => Ok(Units::NMmC),
            "NMC" | "NM" => Ok(Units::NMC),
            "TONFMMC" | "TONMMC" | "TONMM" => Ok(Units::TonMmC),
            "TONFMC" | "TONMC" | "TONM" => Ok(Units::TonMC),
            "KNCMC" | "KNCM" => Ok(Units::KnCmC),
            "KGFCMC" | "KGFCM" => Ok(Units::KgfCmC),
            "NCMC" | "NCM" => Ok(Units::NCmC),
            "TONFCMC" | "TONCMC" | "TONCM" => Ok(Units::TonCmC),
            _ => Err(UnitError::InvalidString(s.to_string())),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_unit_conversions() {
        let kip_ft = Units::KipFtF;
        let kip_in = Units::KipInF;
        assert!((kip_ft.length_factor_to(kip_in) - 12.0).abs() < 1e-12);
        assert!((kip_ft.moment_factor_to(kip_in) - 12.0).abs() < 1e-12);
        assert!((kip_ft.line_load_factor_to(kip_in) - (1.0 / 12.0)).abs() < 1e-12);
        assert!((kip_in.stress_factor_to(kip_ft) - 144.0).abs() < 1e-10);
        assert!((kip_ft.gravity_acceleration() - 32.17404855643044).abs() < 1e-9);

        let parsed: Units = "Kip, ft, F".parse().unwrap();
        assert_eq!(parsed, Units::KipFtF);
    }
}

