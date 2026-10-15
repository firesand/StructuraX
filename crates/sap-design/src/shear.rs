//! AISC 360-10 minor-direction shear, following the supplied CSI design manual,
//! section 3.5.4.2 (printed pages 3-70 and 3-71). Dimensions are design dimensions;
//! the model currently has no ERW/SAW manufacturing or thickness-overwrite metadata.
use crate::DesignError;
use sap_model::{FramePropType, FrameSection, Material};

/// EC3 6.2.6 local-3 plastic shear resistance for a solid rectangular section.
/// This deliberately supports only rectangles: the design model's `as3` is an
/// analysis shear-deformation area, not the EC3 shear area `Av` for arbitrary
/// shapes. For a solid rectangle, EC3 6.2.6(3) gives `Av = A`.
pub fn compute_ec3_rectangular_minor_shear(
    sec: &FrameSection,
    mat: &Material,
) -> Result<f64, DesignError> {
    if sec.prop_type != FramePropType::Rectangular {
        return Err(DesignError::UnsupportedEc3MinorShearSection {
            section: sec.name.clone(),
            kind: format!("{:?}", sec.prop_type),
        });
    }
    let area = positive(sec, "area", sec.area)?;
    let fy = positive(sec, "Fy", mat.fy)?;
    // The existing EC3 implementation uses gamma_M0 = 1.00 (no NA setting).
    positive(
        sec,
        "EC3 minor shear resistance",
        area * fy / 3.0_f64.sqrt(),
    )
}

/// EC3 6.2.6(3) local-3 plastic shear resistance $V_{\text{pl},3,\text{Rd}} = A_{v3} (f_y / \sqrt{3}) / \gamma_{M0}$
/// for `Rectangular`, `I`, `Channel`, `DbChannel`, `T`, `Box`, `Pipe`, and `Circle` sections.
pub fn compute_ec3_minor_shear(sec: &FrameSection, mat: &Material) -> Result<f64, DesignError> {
    if !sec.as3.is_finite() || sec.as3 < 0.0 {
        return Err(DesignError::InvalidShearProperty {
            section: sec.name.clone(),
            property: "As3",
            value: sec.as3.to_string(),
        });
    }
    let fy = positive(sec, "Fy", mat.fy)?;
    let av3 = match sec.prop_type {
        FramePropType::Rectangular => positive(sec, "area", sec.area)?,
        FramePropType::I => {
            if sec.as3 > 0.0 {
                sec.as3
            } else {
                let b_top = positive(sec, "flange width", sec.t2)?;
                let tf_top = positive(sec, "flange thickness", sec.tf)?;
                let b_bot = positive(
                    sec,
                    "bottom flange width",
                    if sec.t2b > 0.0 { sec.t2b } else { sec.t2 },
                )?;
                let tf_bot = positive(
                    sec,
                    "bottom flange thickness",
                    if sec.tfb > 0.0 { sec.tfb } else { sec.tf },
                )?;
                b_top * tf_top + b_bot * tf_bot
            }
        }
        FramePropType::Channel => {
            if sec.as3 > 0.0 {
                sec.as3
            } else {
                let b = positive(sec, "flange width", sec.t2)?;
                let tf = positive(sec, "flange thickness", sec.tf)?;
                2.0 * b * tf
            }
        }
        FramePropType::DbChannel => {
            if sec.as3 > 0.0 {
                sec.as3
            } else {
                let b = positive(sec, "flange width", sec.t2)?;
                let tf = positive(sec, "flange thickness", sec.tf)?;
                4.0 * b * tf
            }
        }
        FramePropType::T => {
            if sec.as3 > 0.0 {
                sec.as3
            } else {
                let b = positive(sec, "flange width", sec.t2)?;
                let tf = positive(sec, "flange thickness", sec.tf)?;
                b * tf
            }
        }
        FramePropType::Box => {
            if sec.as3 > 0.0 {
                sec.as3
            } else {
                let b = positive(sec, "flange width", sec.t2)?;
                let tf = positive(sec, "flange thickness", sec.tf)?;
                2.0 * b * tf
            }
        }
        FramePropType::Pipe | FramePropType::Circle => {
            if sec.as3 > 0.0 {
                sec.as3
            } else {
                let area = positive(sec, "area", sec.area)?;
                (2.0 / std::f64::consts::PI) * area
            }
        }
        _ => {
            return Err(DesignError::UnsupportedEc3MinorShearSection {
                section: sec.name.clone(),
                kind: format!("{:?}", sec.prop_type),
            });
        }
    };
    positive(
        sec,
        "EC3 minor shear resistance",
        av3 * fy / 3.0_f64.sqrt(),
    )
}

fn positive(sec: &FrameSection, name: &'static str, value: f64) -> Result<f64, DesignError> {
    if value.is_finite() && value > 0.0 {
        Ok(value)
    } else {
        Err(DesignError::InvalidShearProperty {
            section: sec.name.clone(),
            property: name,
            value: value.to_string(),
        })
    }
}

fn cv(lambda: f64, kv: f64, e: f64, fy: f64) -> f64 {
    let root = (kv * e / fy).sqrt();
    if lambda <= 1.10 * root {
        1.0
    } else if lambda <= 1.37 * root {
        1.10 * root / lambda
    } else {
        1.51 * kv * e / (lambda * lambda * fy)
    }
}

/// Available local-3 shear strength (LRFD phi Vn or ASD Vn/Omega).
/// Unknown/ambiguous shapes and invalid properties return errors rather than an
/// invented effective area.
pub fn compute_aisc_minor_shear(
    sec: &FrameSection,
    mat: &Material,
    is_asd: bool,
) -> Result<f64, DesignError> {
    let e = positive(sec, "E", mat.e)?;
    let fy = positive(sec, "Fy", mat.fy)?;
    let flange = |width, thickness, half_width| -> Result<f64, DesignError> {
        let b = positive(sec, "flange width", width)?;
        let t = positive(sec, "flange thickness", thickness)?;
        Ok(b * t * cv(b / (if half_width { 2.0 } else { 1.0 } * t), 1.2, e, fy))
    };
    let nominal = match sec.prop_type {
        FramePropType::I => {
            0.6 * fy * (flange(sec.t2, sec.tf, true)? + flange(sec.t2b, sec.tfb, true)?)
        }
        FramePropType::Channel => 0.6 * fy * 2.0 * flange(sec.t2, sec.tf, false)?,
        FramePropType::DbChannel => 0.6 * fy * 4.0 * flange(sec.t2, sec.tf, false)?,
        FramePropType::T => 0.6 * fy * flange(sec.t2, sec.tf, true)?,
        FramePropType::Angle | FramePropType::DblAngle => {
            let b = positive(sec, "leg width", sec.t2)?;
            let t_raw = if sec.tf > 0.0 { sec.tf } else { sec.tw };
            let t = positive(sec, "leg thickness", t_raw)?;
            let mult = if sec.prop_type == FramePropType::DblAngle {
                2.0
            } else {
                1.0
            };
            0.6 * fy * (mult * b * t) * cv(b / t, 1.2, e, fy)
        }
        FramePropType::Box => {
            let t = positive(sec, "tf", sec.tf)?;
            let tw = positive(sec, "tw", sec.tw)?;
            let h = positive(sec, "minor clear wall width", sec.t2 - 3.0 * tw)?;
            0.6 * fy * 2.0 * h * t * cv(h / t, 5.0, e, fy)
        }
        FramePropType::Pipe => {
            let diameter = positive(sec, "diameter", sec.t3)?;
            let t = positive(sec, "wall thickness", sec.tw)?;
            positive(sec, "inner diameter", diameter - 2.0 * t)?;
            let area = positive(sec, "area", sec.area)?;
            (0.78 * e / (diameter / t).powf(1.5)).min(0.6 * fy) * area / 2.0
        }
        FramePropType::Rectangular
        | FramePropType::Circle
        | FramePropType::General
        | FramePropType::SD => 0.6 * fy * positive(sec, "As3", sec.as3)?,
        _ => {
            return Err(DesignError::UnsupportedShearSection {
                section: sec.name.clone(),
                kind: format!("{:?}", sec.prop_type),
            })
        }
    };
    positive(
        sec,
        "minor shear strength",
        if is_asd {
            nominal / 1.67
        } else {
            0.9 * nominal
        },
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use sap_model::SapModel;
    use sap_section::make_i_section;
    use sap_units::Units;

    #[test]
    fn flange_geometry_and_design_factors() {
        let model = SapModel::new(Units::KipInF);
        let mat = &model.materials["A992Fy50"];
        let mut sec = make_i_section("W", "A992Fy50", 14.0, 10.0, 0.5, 0.3, 8.0, 0.4);
        // Both flange slenderness ratios are 10: Cv=1, Aw=5+3.2 in^2.
        assert!((compute_aisc_minor_shear(&sec, mat, false).unwrap() - 221.4).abs() < 1e-10);
        assert!((compute_aisc_minor_shear(&sec, mat, true).unwrap() - 246.0 / 1.67).abs() < 1e-10);
        sec.prop_type = FramePropType::Channel;
        assert!((compute_aisc_minor_shear(&sec, mat, false).unwrap() - 270.0).abs() < 1e-10);
        sec.prop_type = FramePropType::DbChannel;
        assert!((compute_aisc_minor_shear(&sec, mat, false).unwrap() - 540.0).abs() < 1e-10);
        sec.prop_type = FramePropType::T;
        assert!((compute_aisc_minor_shear(&sec, mat, false).unwrap() - 135.0).abs() < 1e-10);
    }

    #[test]
    fn ec3_rectangular_area_and_yield_resistance_are_hand_substituted() {
        let model = SapModel::new(Units::KipInF);
        let mat = &model.materials["A992Fy50"];
        let sec = sap_section::make_rectangular("R", "A992Fy50", 8.0, 4.0);
        // EC3 6.2.6(3): Av=A=8*4=32 in2; Vpl,Rd=Av*Fy/sqrt(3), gamma_M0=1.
        let expected = 32.0 * 50.0 / 3.0_f64.sqrt();
        assert!((compute_ec3_rectangular_minor_shear(&sec, mat).unwrap() - expected).abs() < 1e-10);
    }

    #[test]
    fn ec3_minor_shear_rejects_non_rectangular_and_invalid_inputs() {
        let model = SapModel::new(Units::KipInF);
        let mat = &model.materials["A992Fy50"];
        let mut sec = sap_section::make_rectangular("R", "A992Fy50", 8.0, 4.0);
        sec.prop_type = FramePropType::I;
        assert!(matches!(
            compute_ec3_rectangular_minor_shear(&sec, mat),
            Err(DesignError::UnsupportedEc3MinorShearSection { .. })
        ));
        sec.prop_type = FramePropType::Rectangular;
        sec.area = f64::NAN;
        assert!(compute_ec3_rectangular_minor_shear(&sec, mat).is_err());
    }

    #[test]
    fn buckling_branches_are_not_plastic_strength() {
        // Independent dimensionless G2-3/4/5 substitutions at E/Fy=1000, kv=1.2.
        assert_eq!(cv(20.0, 1.2, 50000.0, 50.0), 1.0);
        assert!((cv(40.0, 1.2, 50000.0, 50.0) - 0.9526279441628825).abs() < 1e-12);
        assert!((cv(80.0, 1.2, 50000.0, 50.0) - 0.283125).abs() < 1e-12);
    }

    #[test]
    fn closed_sections_and_effective_area_strengths() {
        let model = SapModel::new(Units::KipInF);
        let mat = &model.materials["A992Fy50"];
        let mut sec = make_i_section("BOX", "A992Fy50", 12.0, 8.0, 0.5, 0.5, 8.0, 0.5);
        sec.prop_type = FramePropType::Box;
        // Minor wall clear width 8 - 3(0.5) = 6.5; two resisting walls, Cv=1.
        assert!((compute_aisc_minor_shear(&sec, mat, false).unwrap() - 175.5).abs() < 1e-10);
        sec.prop_type = FramePropType::Pipe;
        sec.area = 18.0;
        // D/t=24: yielding governs Fcr=0.6 Fy=30, Ag/2=9.
        assert!((compute_aisc_minor_shear(&sec, mat, false).unwrap() - 243.0).abs() < 1e-10);
        sec.t3 = 1000.0;
        sec.tw = 1.0;
        // D/t=1000: shear buckling governs; independent decimal substitution.
        assert!(
            (compute_aisc_minor_shear(&sec, mat, false).unwrap() - 5.793988374513708).abs() < 1e-10
        );
        sec.prop_type = FramePropType::General;
        sec.as3 = 4.0;
        assert!((compute_aisc_minor_shear(&sec, mat, false).unwrap() - 108.0).abs() < 1e-10);
    }

    #[test]
    fn invalid_properties_and_unknown_shapes_fail() {
        let model = SapModel::new(Units::KipInF);
        let mat = &model.materials["A992Fy50"];
        let mut sec = make_i_section("W", "A992Fy50", 14.0, 10.0, 0.5, 0.3, 10.0, 0.5);
        sec.tfb = f64::NAN;
        assert!(compute_aisc_minor_shear(&sec, mat, false).is_err());
        sec.prop_type = FramePropType::Auto;
        assert!(matches!(
            compute_aisc_minor_shear(&sec, mat, false),
            Err(DesignError::UnsupportedShearSection { .. })
        ));
        sec.prop_type = FramePropType::General;
        sec.as3 = 0.0;
        assert!(compute_aisc_minor_shear(&sec, mat, false).is_err());
    }
}
