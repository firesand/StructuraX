use crate::{
    compute_aisc_minor_shear, compute_ec3_minor_shear, compute_ec3_rectangular_minor_shear,
    error::validate_demands, DesignError,
};
use sap_model::{
    ComboType, FrameObj, FramePropType, FrameSection, FrameStationResult, LoadCombo, MatType,
    Material, SapModel, SteelDesignResult,
};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

/// Section compactness classification per AISC 360-10 Table B4.1a / B4.1b
/// and Eurocode 3 (`EN 1993-1-1` Table 5.2).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CompactnessClass {
    Compact,
    Noncompact,
    Slender,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct SectionCompactness {
    pub flange_compression: CompactnessClass,
    pub web_compression: CompactnessClass,
    pub flange_flexure: CompactnessClass,
    pub web_flexure: CompactnessClass,
}

impl SectionCompactness {
    pub fn is_slender(&self) -> bool {
        self.flange_flexure == CompactnessClass::Slender
            || self.web_flexure == CompactnessClass::Slender
    }
}

/// Design code variant for steel frame design (`sapdesign.dll` `apo` and `ard`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SteelCode {
    Aisc360Lrfd,
    Aisc360Asd,
    Eurocode3,
}

impl SteelCode {
    /// Parses a supported steel code name without silently selecting a fallback code.
    pub fn try_from_code_str(code: &str) -> Result<Self, DesignError> {
        let normalized: String = code
            .chars()
            .filter(|c| c.is_ascii_alphanumeric())
            .flat_map(char::to_uppercase)
            .collect();
        match normalized.as_str() {
            "AISC36010" | "AISC36010LRFD" | "AISC360" | "AISC360LRFD" | "LRFD" => {
                Ok(SteelCode::Aisc360Lrfd)
            }
            "AISC36010ASD" | "AISCASD" | "ASD" => Ok(SteelCode::Aisc360Asd),
            "EUROCODE3" | "EUROCODE32005" | "EC3" | "EN199311" | "EN1993112005" => {
                Ok(SteelCode::Eurocode3)
            }
            _ => Err(DesignError::UnsupportedSteelCode(code.to_string())),
        }
    }

    /// Legacy permissive parser. New design runs should use [`Self::try_from_code_str`].
    pub fn from_code_str(code: &str) -> Self {
        Self::try_from_code_str(code).unwrap_or(SteelCode::Aisc360Lrfd)
    }
}

/// Optional per-member steel design overwrites matching `cDesignSteel.GetOverwrite` / `SetOverwrite`.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SteelMemberOverwrite {
    /// Unbraced length ratio for major-axis bending ($L_{33} / L$)
    pub l_ratio_major: Option<f64>,
    /// Unbraced length ratio for minor-axis bending ($L_{22} / L$)
    pub l_ratio_minor: Option<f64>,
    /// Unbraced length ratio for lateral-torsional buckling ($L_{ltb} / L$)
    pub l_ratio_ltb: Option<f64>,
    /// Effective length factor $K_{33}$ (major axis)
    pub k_major: Option<f64>,
    /// Effective length factor $K_{22}$ (minor axis)
    pub k_minor: Option<f64>,
    /// Moment gradient factor $C_b$ (`None` = compute automatically from station moments)
    pub cb: Option<f64>,
    /// Sway frame flag for automatic alignment-chart $K$-factor calculation
    pub is_sway: bool,
}

/// Evaluates flange and web compactness for axial compression and flexure per AISC 360-10 Table B4.1a & B4.1b.
pub fn check_compactness_aisc(sec: &FrameSection, e: f64, fy: f64) -> SectionCompactness {
    let root_efy = (e / fy.max(1e-12)).sqrt();
    match sec.prop_type {
        FramePropType::I
        | FramePropType::Channel
        | FramePropType::T
        | FramePropType::DbChannel
        | FramePropType::BuiltUpICoverplate
        | FramePropType::BuiltUpIHybrid => {
            let bf = sec.t2.max(sec.t2b);
            let tf = sec.tf.max(1e-9);
            let tw = sec.tw.max(1e-9);
            let h = (sec.t3 - sec.tf - sec.tfb).max(0.0);

            let lambda_f = if sec.prop_type == FramePropType::Channel
                || sec.prop_type == FramePropType::DbChannel
            {
                bf / tf
            } else {
                bf / (2.0 * tf)
            };
            let lambda_w = h / tw;

            // Compression limits (Table B4.1a)
            let lambda_rf_comp = 0.56 * root_efy;
            let lambda_rw_comp = 1.49 * root_efy;
            let flange_compression = if lambda_f <= lambda_rf_comp {
                CompactnessClass::Compact
            } else {
                CompactnessClass::Slender
            };
            let web_compression = if lambda_w <= lambda_rw_comp {
                CompactnessClass::Compact
            } else {
                CompactnessClass::Slender
            };

            // Flexure limits (Table B4.1b)
            let lambda_pf = 0.38 * root_efy;
            let lambda_rf = 1.00 * root_efy;
            let flange_flexure = if lambda_f <= lambda_pf {
                CompactnessClass::Compact
            } else if lambda_f <= lambda_rf {
                CompactnessClass::Noncompact
            } else {
                CompactnessClass::Slender
            };

            let lambda_pw = 3.76 * root_efy;
            let lambda_rw = 5.70 * root_efy;
            let web_flexure = if lambda_w <= lambda_pw {
                CompactnessClass::Compact
            } else if lambda_w <= lambda_rw {
                CompactnessClass::Noncompact
            } else {
                CompactnessClass::Slender
            };

            SectionCompactness {
                flange_compression,
                web_compression,
                flange_flexure,
                web_flexure,
            }
        }
        FramePropType::Box | FramePropType::ColdBox => {
            let tf = sec.tf.max(1e-9);
            let tw = sec.tw.max(1e-9);
            let b_clear = (sec.t2 - 3.0 * tw).max(0.0);
            let h_clear = (sec.t3 - 3.0 * tf).max(0.0);
            let lambda_f = b_clear / tf;
            let lambda_w = h_clear / tw;

            let lambda_r_comp = 1.40 * root_efy;
            let flange_compression = if lambda_f <= lambda_r_comp {
                CompactnessClass::Compact
            } else {
                CompactnessClass::Slender
            };
            let web_compression = if lambda_w <= lambda_r_comp {
                CompactnessClass::Compact
            } else {
                CompactnessClass::Slender
            };

            let lambda_pf = 1.12 * root_efy;
            let lambda_rf = 1.40 * root_efy;
            let flange_flexure = if lambda_f <= lambda_pf {
                CompactnessClass::Compact
            } else if lambda_f <= lambda_rf {
                CompactnessClass::Noncompact
            } else {
                CompactnessClass::Slender
            };

            let lambda_pw = 2.42 * root_efy;
            let lambda_rw = 5.70 * root_efy;
            let web_flexure = if lambda_w <= lambda_pw {
                CompactnessClass::Compact
            } else if lambda_w <= lambda_rw {
                CompactnessClass::Noncompact
            } else {
                CompactnessClass::Slender
            };

            SectionCompactness {
                flange_compression,
                web_compression,
                flange_flexure,
                web_flexure,
            }
        }
        FramePropType::Pipe | FramePropType::ColdPipe => {
            let d = sec.t3;
            let t = sec.tw.max(sec.tf).max(1e-9);
            let lambda = d / t;
            let efy = e / fy.max(1e-12);
            let flange_compression = if lambda <= 0.11 * efy {
                CompactnessClass::Compact
            } else {
                CompactnessClass::Slender
            };
            let flange_flexure = if lambda <= 0.07 * efy {
                CompactnessClass::Compact
            } else if lambda <= 0.31 * efy {
                CompactnessClass::Noncompact
            } else {
                CompactnessClass::Slender
            };
            SectionCompactness {
                flange_compression,
                web_compression: CompactnessClass::Compact,
                flange_flexure,
                web_flexure: CompactnessClass::Compact,
            }
        }
        FramePropType::Angle | FramePropType::DblAngle => {
            let b = sec.t3.max(sec.t2);
            let t = sec.tf.max(sec.tw).max(1e-9);
            let lambda = b / t;
            let flange_compression = if lambda <= 0.45 * root_efy {
                CompactnessClass::Compact
            } else {
                CompactnessClass::Slender
            };
            let flange_flexure = if lambda <= 0.54 * root_efy {
                CompactnessClass::Compact
            } else if lambda <= 0.91 * root_efy {
                CompactnessClass::Noncompact
            } else {
                CompactnessClass::Slender
            };
            SectionCompactness {
                flange_compression,
                web_compression: CompactnessClass::Compact,
                flange_flexure,
                web_flexure: CompactnessClass::Compact,
            }
        }
        _ => SectionCompactness {
            flange_compression: CompactnessClass::Compact,
            web_compression: CompactnessClass::Compact,
            flange_flexure: CompactnessClass::Compact,
            web_flexure: CompactnessClass::Compact,
        },
    }
}

/// Computes the slender element reduction factor $Q = Q_s Q_a$ per AISC 360-10 Section E7.
pub fn compute_slender_q_factor(sec: &FrameSection, e: f64, fy: f64) -> f64 {
    let root_efy = (e / fy.max(1e-12)).sqrt();
    match sec.prop_type {
        FramePropType::I | FramePropType::Channel | FramePropType::T | FramePropType::DbChannel => {
            let bf = sec.t2.max(sec.t2b);
            let tf = sec.tf.max(1e-9);
            let tw = sec.tw.max(1e-9);
            let h = (sec.t3 - sec.tf - sec.tfb).max(0.0);
            let lambda_f = bf / (2.0 * tf);
            let qs = if lambda_f <= 0.56 * root_efy {
                1.0
            } else if lambda_f < 1.03 * root_efy {
                1.415 - 0.74 * lambda_f / root_efy
            } else {
                0.69 * e / (fy * lambda_f * lambda_f)
            };

            let lambda_w = h / tw;
            let qa = if lambda_w <= 1.49 * root_efy {
                1.0
            } else {
                let b_eff = (1.92 * tw * root_efy * (1.0 - (0.34 / lambda_w) * root_efy)).min(h);
                let a_eff = (sec.area - (h - b_eff) * tw).max(0.1 * sec.area);
                (a_eff / sec.area).clamp(0.1, 1.0)
            };
            (qs * qa).clamp(0.1, 1.0)
        }
        FramePropType::Pipe | FramePropType::ColdPipe => {
            let d = sec.t3;
            let t = sec.tw.max(sec.tf).max(1e-9);
            let lambda = d / t;
            let efy = e / fy.max(1e-12);
            if lambda <= 0.11 * efy {
                1.0
            } else if lambda <= 0.45 * efy {
                (0.038 * efy / lambda + 2.0 / 3.0).clamp(0.1, 1.0)
            } else {
                0.33
            }
        }
        _ => 1.0,
    }
}

/// Solves the Julian-Lawrence alignment chart equation for a **braced (non-sway)** frame
/// ($0.5 \le K \le 1.0$) given rotational flexibility ratios $G_A, G_B \ge 0$.
pub fn compute_k_braced(ga: f64, gb: f64) -> f64 {
    let ga = ga.max(0.0);
    let gb = gb.max(0.0);
    if ga < 1e-9 && gb < 1e-9 {
        return 0.5;
    }
    if ga > 1e6 && gb > 1e6 {
        return 1.0;
    }
    // Closed-form French-curve / Dumonteil accurate initial guess, refined by bisection
    let f = |k: f64| -> f64 {
        let pik = PI / k;
        let tan_pik = pik.tan();
        let tan_half = (0.5 * pik).tan();
        0.25 * ga * gb * pik * pik + 0.5 * (ga + gb) * (1.0 - pik / tan_pik) + 2.0 * tan_half / pik
            - 1.0
    };
    let mut lo = 0.500_001;
    let mut hi = 0.999_999;
    for _ in 0..50 {
        let mid = 0.5 * (lo + hi);
        if f(mid) > 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

/// Solves the Julian-Lawrence alignment chart equation for an **unbraced (sway)** frame
/// ($K \ge 1.0$) given rotational flexibility ratios $G_A, G_B \ge 0$.
pub fn compute_k_sway(ga: f64, gb: f64) -> f64 {
    let ga = ga.clamp(0.0, 1e6);
    let gb = gb.clamp(0.0, 1e6);
    if ga < 1e-9 && gb < 1e-9 {
        return 1.0;
    }
    let f = |k: f64| -> f64 {
        let pik = PI / k;
        (ga * gb * pik * pik - 36.0) / (6.0 * (ga + gb).max(1e-12)) - pik * pik.cos() / pik.sin()
    };
    let mut lo = 1.000_001;
    let mut hi = 20.0;
    for _ in 0..60 {
        let mid = 0.5 * (lo + hi);
        if f(mid) > 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

/// Automatically computes $G_A, G_B$ at the ends of a frame element when connected to a multi-member frame.
/// If the member is a standalone member or direct analysis ($K = 1.0$) applies, returns `(1.0, 1.0)`.
pub fn compute_member_k_factors(
    model: &SapModel,
    frame: &FrameObj,
    overwrite: Option<&SteelMemberOverwrite>,
) -> (f64, f64) {
    let k33_ow = overwrite.and_then(|o| o.k_major);
    let k22_ow = overwrite.and_then(|o| o.k_minor);
    if let (Some(k33), Some(k22)) = (k33_ow, k22_ow) {
        return (k33, k22);
    }

    let is_sway = overwrite.map(|o| o.is_sway).unwrap_or(false);
    if !is_sway {
        // Under AISC 360-10 Direct Analysis Method and standard non-sway design, default K = 1.0
        // unless explicit overwrites or sway alignment chart is requested.
        return (k33_ow.unwrap_or(1.0), k22_ow.unwrap_or(1.0));
    }

    let calc_g_at_joint = |joint_name: &str| -> f64 {
        let Some(pt) = model.points.get(joint_name) else {
            return 10.0;
        };
        if pt.restraint[3] || pt.restraint[4] || pt.restraint[5] {
            return 1.0; // AISC Commentary practical fixed base G = 1.0
        }
        if pt.restraint[0] || pt.restraint[1] || pt.restraint[2] {
            // Pinned support
            return 10.0;
        }
        let mut sum_col = 0.0;
        let mut sum_beam = 0.0;
        for other in model.frames.values() {
            if other.point_i != joint_name && other.point_j != joint_name {
                continue;
            }
            let len = frame_clear_length(model, other);
            if len <= 1e-9 {
                continue;
            }
            let Some(sec) = model.frame_sections.get(&other.section) else {
                continue;
            };
            let e = model
                .materials
                .get(&sec.material)
                .map(|m| m.e)
                .unwrap_or(29000.0);
            let ei_over_l = e * sec.i33 / len;
            let (Some(pi), Some(pj)) = (
                model.points.get(&other.point_i),
                model.points.get(&other.point_j),
            ) else {
                continue;
            };
            let dz = (pj.z - pi.z).abs();
            if dz / len > 0.7 {
                sum_col += ei_over_l;
            } else {
                sum_beam += ei_over_l;
            }
        }
        if sum_beam > 1e-12 && sum_col > 1e-12 {
            (sum_col / sum_beam).clamp(0.0, 100.0)
        } else {
            10.0
        }
    };

    let ga = calc_g_at_joint(&frame.point_i);
    let gb = calc_g_at_joint(&frame.point_j);
    let k_auto = compute_k_sway(ga, gb);
    (k33_ow.unwrap_or(k_auto), k22_ow.unwrap_or(1.0))
}

/// Computes the clear geometric length $L$ of a frame element (accounting for rigid end offsets).
pub fn frame_clear_length(model: &SapModel, frame: &FrameObj) -> f64 {
    let (Some(pi), Some(pj)) = (
        model.points.get(&frame.point_i),
        model.points.get(&frame.point_j),
    ) else {
        return 0.0;
    };
    let dx = pj.x - pi.x;
    let dy = pj.y - pi.y;
    let dz = pj.z - pi.z;
    let total_len = (dx * dx + dy * dy + dz * dz).sqrt();
    let clear = total_len - (frame.end_offset_i + frame.end_offset_j);
    if clear > 1e-6 {
        clear
    } else {
        total_len
    }
}

/// Computes the AISC Chapter F lateral-torsional buckling moment gradient factor:
/// $$C_b = \frac{12.5 M_{\max}}{2.5 M_{\max} + 3 M_A + 4 M_B + 3 M_C}$$
/// from a sequence of station results along the member.
pub fn compute_cb_from_stations(stations: &[FrameStationResult]) -> f64 {
    if stations.len() < 2 {
        return 1.0;
    }
    let m_max = stations.iter().map(|s| s.m3.abs()).fold(0.0_f64, f64::max);
    if m_max <= 1e-12 {
        return 1.0;
    }

    let sample_abs_m3 = |target_rel: f64| -> f64 {
        // Find surrounding stations by rel_dist
        for w in stations.windows(2) {
            let s0 = w[0].rel_dist;
            let s1 = w[1].rel_dist;
            if target_rel >= s0 - 1e-9 && target_rel <= s1 + 1e-9 {
                let ds = (s1 - s0).max(1e-12);
                let t = ((target_rel - s0) / ds).clamp(0.0, 1.0);
                let m = w[0].m3 + t * (w[1].m3 - w[0].m3);
                return m.abs();
            }
        }
        // Fallback if stations aren't sorted or don't have rel_dist populated
        let l_min = stations.first().map(|s| s.station).unwrap_or(0.0);
        let l_max = stations.last().map(|s| s.station).unwrap_or(1.0);
        let span = (l_max - l_min).max(1e-12);
        let target_x = l_min + target_rel * span;
        for w in stations.windows(2) {
            if target_x >= w[0].station - 1e-9 && target_x <= w[1].station + 1e-9 {
                let dx = (w[1].station - w[0].station).max(1e-12);
                let t = ((target_x - w[0].station) / dx).clamp(0.0, 1.0);
                let m = w[0].m3 + t * (w[1].m3 - w[0].m3);
                return m.abs();
            }
        }
        stations[0].m3.abs()
    };

    let ma = sample_abs_m3(0.25);
    let mb = sample_abs_m3(0.50);
    let mc = sample_abs_m3(0.75);
    let denom = 2.5 * m_max + 3.0 * ma + 4.0 * mb + 3.0 * mc;
    if denom <= 1e-12 {
        1.0
    } else {
        (12.5 * m_max / denom).clamp(1.0, 3.0)
    }
}

/// Computes factored capacities $(\phi_c P_{nc}, \phi_t P_{nt}, \phi_b M_{n33}, \phi_b M_{n22}, \phi_v V_{n2})$
/// per AISC 360-10 / 360-16 (LRFD or ASD).
pub fn compute_aisc_360_capacities(
    sec: &FrameSection,
    mat: &Material,
    l33: f64,
    l22: f64,
    l_ltb: f64,
    k33: f64,
    k22: f64,
    cb: f64,
    is_asd: bool,
) -> (f64, f64, f64, f64, f64, SectionCompactness) {
    let e = mat.e.max(1.0);
    let g = if mat.g > 0.0 {
        mat.g
    } else {
        e / (2.0 * (1.0 + mat.u.max(0.3)))
    };
    let fy = mat.fy.max(1e-6);
    let ag = sec.area.max(1e-9);
    let r33 = if sec.r33 > 1e-9 {
        sec.r33
    } else {
        (sec.i33 / ag).sqrt().max(1e-6)
    };
    let r22 = if sec.r22 > 1e-9 {
        sec.r22
    } else {
        (sec.i22 / ag).sqrt().max(1e-6)
    };

    let compactness = check_compactness_aisc(sec, e, fy);
    let q = compute_slender_q_factor(sec, e, fy);

    // 1. Tensile capacity (AISC D2: Yielding in gross section)
    let p_nt = fy * ag;
    let phi_pnt = if is_asd { p_nt / 1.67 } else { 0.90 * p_nt };

    // 2. Compressive capacity (AISC E3, E4, E7)
    let kl_r33 = (k33 * l33) / r33;
    let kl_r22 = (k22 * l22) / r22;
    let kl_r = kl_r33.max(kl_r22).max(1e-6);
    let fe_flex = (PI * PI * e) / (kl_r * kl_r);

    // Check torsional buckling elastic stress Fez for doubly-symmetric I-sections (AISC Eq. E4-4)
    let fe = if sec.prop_type == FramePropType::I && sec.torsional_j > 0.0 && sec.t3 > sec.tf {
        let h0 = (sec.t3 - 0.5 * (sec.tf + sec.tfb)).max(1e-6);
        let cw = 0.25 * sec.i22 * h0 * h0;
        let r0_sq = r33 * r33 + r22 * r22;
        let kz_lz = (k22 * l22).max(1e-6);
        let fez = ((PI * PI * e * cw) / (kz_lz * kz_lz) + g * sec.torsional_j) / (ag * r0_sq);
        fe_flex.min(fez)
    } else {
        fe_flex
    };

    let slenderness_limit = 4.71 * (e / (q * fy)).sqrt();
    let fcr = if kl_r <= slenderness_limit || (q * fy / fe) <= 2.25 {
        q * 0.658_f64.powf((q * fy) / fe) * fy
    } else {
        0.877 * fe
    };
    let p_nc = fcr * ag;
    let phi_pnc = if is_asd { p_nc / 1.67 } else { 0.90 * p_nc };

    // 3. Major-axis Flexural capacity Mn33 (AISC Chapter F)
    let s33 = sec.s33.max(1e-9);
    let z33 = if sec.z33 > 1e-9 { sec.z33 } else { 1.12 * s33 };
    let mp33 = (fy * z33).min(1.6 * fy * s33);

    let mn_ltb = match sec.prop_type {
        FramePropType::I
        | FramePropType::Channel
        | FramePropType::DbChannel
        | FramePropType::BuiltUpICoverplate
        | FramePropType::BuiltUpIHybrid => {
            let h0 = (sec.t3 - 0.5 * (sec.tf + sec.tfb)).max(1e-6);
            let cw = 0.25 * sec.i22 * h0 * h0;
            let rts_sq = (sec.i22 * cw).sqrt() / s33;
            let rts = if rts_sq > 1e-12 {
                rts_sq.sqrt()
            } else {
                r22 * 1.15
            };
            let c_factor = 1.0;
            let jc_sh0 = (sec.torsional_j * c_factor) / (s33 * h0);
            let lp = 1.76 * r22 * (e / fy).sqrt();
            let term = (jc_sh0 * jc_sh0 + 6.76 * (0.7 * fy / e).powi(2)).sqrt();
            let lr = 1.95 * rts * (e / (0.7 * fy)) * (jc_sh0 + term).sqrt();

            if l_ltb <= lp {
                mp33
            } else if l_ltb <= lr {
                let m = cb * (mp33 - (mp33 - 0.7 * fy * s33) * ((l_ltb - lp) / (lr - lp)));
                m.min(mp33)
            } else {
                let lb_rts = l_ltb / rts;
                let fcr_ltb = ((cb * PI * PI * e) / (lb_rts * lb_rts))
                    * (1.0 + 0.078 * jc_sh0 * lb_rts * lb_rts).sqrt();
                (fcr_ltb * s33).min(mp33)
            }
        }
        _ => mp33,
    };

    // Compression Flange Local Buckling (FLB) for major-axis bending (AISC F3)
    let mn_flb = if sec.prop_type == FramePropType::I && sec.tf > 0.0 {
        let bf = sec.t2.max(sec.t2b);
        let lambda_f = bf / (2.0 * sec.tf);
        let root_efy = (e / fy).sqrt();
        let lambda_pf = 0.38 * root_efy;
        let lambda_rf = 1.00 * root_efy;
        if lambda_f <= lambda_pf {
            mp33
        } else if lambda_f <= lambda_rf {
            mp33 - (mp33 - 0.7 * fy * s33) * ((lambda_f - lambda_pf) / (lambda_rf - lambda_pf))
        } else {
            let h = (sec.t3 - sec.tf - sec.tfb).max(1e-6);
            let kc = (4.0 / (h / sec.tw.max(1e-6)).sqrt()).clamp(0.35, 0.76);
            (0.90 * e * kc * s33) / (lambda_f * lambda_f)
        }
    } else {
        mp33
    };

    let mn33 = mp33.min(mn_ltb).min(mn_flb);
    let phi_mn33 = if is_asd { mn33 / 1.67 } else { 0.90 * mn33 };

    // 4. Minor-axis Flexural capacity Mn22 (AISC F6)
    let s22 = sec.s22.max(1e-9);
    let z22 = if sec.z22 > 1e-9 { sec.z22 } else { 1.5 * s22 };
    let mp22 = (fy * z22).min(1.6 * fy * s22);
    let mn22 = if sec.prop_type == FramePropType::I && sec.tf > 0.0 {
        let bf = sec.t2.max(sec.t2b);
        let lambda_f = bf / (2.0 * sec.tf);
        let root_efy = (e / fy).sqrt();
        let lambda_pf = 0.38 * root_efy;
        let lambda_rf = 1.00 * root_efy;
        if lambda_f <= lambda_pf {
            mp22
        } else if lambda_f <= lambda_rf {
            mp22 - (mp22 - 0.7 * fy * s22) * ((lambda_f - lambda_pf) / (lambda_rf - lambda_pf))
        } else {
            (0.69 * e * s22) / (lambda_f * lambda_f)
        }
    } else {
        mp22
    };
    let phi_mn22 = if is_asd { mn22 / 1.67 } else { 0.90 * mn22 };

    // 5. Shear capacity Vn2 (AISC Chapter G)
    let aw = if sec.t3 > 0.0 && sec.tw > 0.0 {
        match sec.prop_type {
            FramePropType::Box | FramePropType::ColdBox => {
                2.0 * (sec.t3 - 2.0 * sec.tf).max(0.0) * sec.tw
            }
            FramePropType::Pipe | FramePropType::ColdPipe => 0.5 * ag,
            _ => sec.t3 * sec.tw,
        }
    } else if sec.as2 > 0.0 {
        sec.as2
    } else {
        0.5 * ag
    };

    let h_tw = if sec.tw > 1e-9 {
        (sec.t3 - sec.tf - sec.tfb).max(0.0) / sec.tw
    } else {
        10.0
    };
    let root_efy = (e / fy).sqrt();
    let (cv, phi_v_lrfd, omega_v_asd) =
        if sec.prop_type == FramePropType::I && h_tw <= 2.24 * root_efy {
            // AISC G2.1(a) rolled I-shaped member
            (1.0, 1.00, 1.50)
        } else {
            let kv = 5.0_f64;
            let lim1 = 1.10 * (kv * e / fy).sqrt();
            let lim2 = 1.37 * (kv * e / fy).sqrt();
            let cv_val = if h_tw <= lim1 {
                1.0
            } else if h_tw <= lim2 {
                lim1 / h_tw
            } else {
                (1.51 * kv * e) / (h_tw * h_tw * fy)
            };
            (cv_val, 0.90, 1.67)
        };
    let vn2 = 0.60 * fy * aw * cv;
    let phi_vn2 = if is_asd {
        vn2 / omega_v_asd
    } else {
        phi_v_lrfd * vn2
    };

    (phi_pnc, phi_pnt, phi_mn33, phi_mn22, phi_vn2, compactness)
}

/// Computes Eurocode 3-2005 (`EN 1993-1-1` Section 6.2 & 6.3) design resistances
/// $(N_{b,Rd}, N_{t,Rd}, M_{b,Rd,y}, M_{c,Rd,z}, V_{c,Rd})$.
pub fn compute_ec3_capacities(
    sec: &FrameSection,
    mat: &Material,
    l33: f64,
    l22: f64,
    l_ltb: f64,
    k33: f64,
    k22: f64,
    cb: f64,
) -> (f64, f64, f64, f64, f64, SectionCompactness) {
    let e = mat.e.max(1.0);
    let g = if mat.g > 0.0 { mat.g } else { e / 2.6 };
    let fy = mat.fy.max(1e-6);
    let ag = sec.area.max(1e-9);

    let gamma_m0 = 1.00;
    let gamma_m1 = 1.00;
    let compactness = check_compactness_aisc(sec, e, fy);

    // 1. Tension resistance Nt,Rd = A * fy / gamma_M0
    let n_trd = (ag * fy) / gamma_m0;

    // 2. Buckling resistance Nb,Rd per EN 1993-1-1 6.3.1
    let n_cr33 = (PI * PI * e * sec.i33) / (k33 * l33).max(1e-6).powi(2);
    let n_cr22 = (PI * PI * e * sec.i22) / (k22 * l22).max(1e-6).powi(2);
    let lambda_bar_33 = (ag * fy / n_cr33.max(1e-9)).sqrt();
    let lambda_bar_22 = (ag * fy / n_cr22.max(1e-9)).sqrt();

    // Select imperfection factor alpha per Table 6.1 & 6.2 (rolled I-sections: h/b > 1.2 -> curve a (0.21) major, b (0.34) minor; h/b <= 1.2 -> curve b (0.34) major, c (0.49) minor)
    let h_over_b = sec.t3 / sec.t2.max(1e-6);
    let (alpha_33, alpha_22) = match sec.prop_type {
        FramePropType::I => {
            if h_over_b > 1.2 {
                (0.21, 0.34)
            } else {
                (0.34, 0.49)
            }
        }
        FramePropType::Pipe | FramePropType::Box => (0.21, 0.21),
        _ => (0.49, 0.49),
    };

    let calc_chi = |lambda_bar: f64, alpha: f64| -> f64 {
        if lambda_bar <= 0.2 {
            1.0
        } else {
            let phi = 0.5 * (1.0 + alpha * (lambda_bar - 0.2) + lambda_bar * lambda_bar);
            let disc = (phi * phi - lambda_bar * lambda_bar).max(0.0).sqrt();
            (1.0 / (phi + disc)).clamp(0.0, 1.0)
        }
    };

    let chi_33 = calc_chi(lambda_bar_33, alpha_33);
    let chi_22 = calc_chi(lambda_bar_22, alpha_22);
    let chi = chi_33.min(chi_22);
    let n_brd = (chi * ag * fy) / gamma_m1;

    // 3. Lateral-Torsional Buckling resistance Mb,Rd per EN 1993-1-1 6.3.2
    let wy = if sec.z33 > 1e-9 && !compactness.is_slender() {
        sec.z33
    } else {
        sec.s33.max(1e-9)
    };
    let h0 = (sec.t3 - 0.5 * (sec.tf + sec.tfb)).max(1e-6);
    let iw = 0.25 * sec.i22 * h0 * h0;
    let l_lt = l_ltb.max(1e-6);
    let m_cr = cb
        * (PI * PI * e * sec.i22 / (l_lt * l_lt))
        * (iw / sec.i22.max(1e-12)
            + (l_lt * l_lt * g * sec.torsional_j) / (PI * PI * e * sec.i22.max(1e-12)))
        .sqrt();
    let lambda_lt = (wy * fy / m_cr.max(1e-9)).sqrt();
    let alpha_lt = if h_over_b > 2.0 { 0.34 } else { 0.21 };
    let chi_lt = calc_chi(lambda_lt, alpha_lt);
    let m_brd_33 = (chi_lt * wy * fy) / gamma_m1;

    // 4. Minor-axis bending resistance Mc,Rd,z
    let wz = if sec.z22 > 1e-9 && !compactness.is_slender() {
        sec.z22
    } else {
        sec.s22.max(1e-9)
    };
    let m_crd_22 = (wz * fy) / gamma_m0;

    // 5. Shear resistance Vpl,Rd = Av * (fy / sqrt(3)) / gamma_M0
    let av = if sec.as2 > 0.0 {
        sec.as2
    } else if sec.t3 > 0.0 && sec.tw > 0.0 {
        sec.t3 * sec.tw
    } else {
        0.5 * ag
    };
    let v_crd = (av * fy) / (3.0_f64.sqrt() * gamma_m0);

    (n_brd, n_trd, m_brd_33, m_crd_22, v_crd, compactness)
}

/// Evaluates a single station under AISC 360-10 / 360-16 Chapter H (Equations H1-1a & H1-1b)
/// or Eurocode 3 (`EN 1993-1-1` Section 6.2 / 6.3).
pub fn evaluate_station_interaction(
    code: SteelCode,
    p: f64,
    v2: f64,
    m3: f64,
    m2: f64,
    phi_pnc: f64,
    phi_pnt: f64,
    phi_mn33: f64,
    phi_mn22: f64,
    phi_vn2: f64,
) -> (f64, f64, f64, f64, f64, &'static str) {
    let pr = p.abs();
    let pc = if p >= 0.0 {
        phi_pnt.max(1e-12)
    } else {
        phi_pnc.max(1e-12)
    };
    let mr3 = m3.abs();
    let mr2 = m2.abs();
    let mc3 = phi_mn33.max(1e-12);
    let mc2 = phi_mn22.max(1e-12);
    let shear_ratio = v2.abs() / phi_vn2.max(1e-12);

    match code {
        SteelCode::Aisc360Lrfd | SteelCode::Aisc360Asd => {
            let p_over_pc = pr / pc;
            if p_over_pc >= 0.2 {
                let pr_part = p_over_pc;
                let mr3_part = (8.0 / 9.0) * (mr3 / mc3);
                let mr2_part = (8.0 / 9.0) * (mr2 / mc2);
                let total = pr_part + mr3_part + mr2_part;
                (total, pr_part, mr3_part, mr2_part, shear_ratio, "H1-1a")
            } else {
                let pr_part = 0.5 * p_over_pc;
                let mr3_part = mr3 / mc3;
                let mr2_part = mr2 / mc2;
                let total = pr_part + mr3_part + mr2_part;
                (total, pr_part, mr3_part, mr2_part, shear_ratio, "H1-1b")
            }
        }
        SteelCode::Eurocode3 => {
            let pr_part = pr / pc;
            let mr3_part = mr3 / mc3;
            let mr2_part = mr2 / mc2;
            let total = pr_part + mr3_part + mr2_part;
            (total, pr_part, mr3_part, mr2_part, shear_ratio, "EC3-6.2")
        }
    }
}

/// Resolves all design load combinations (or analysis cases if no combos are defined)
/// into a list of `(combo_or_case_name, IndexMap<frame_name, Vec<FrameStationResult>>)`.
pub(crate) fn collect_design_combo_forces(
    model: &SapModel,
) -> Vec<(String, Vec<(String, Vec<FrameStationResult>)>)> {
    let mut out = Vec::new();

    if !model.load_combos.is_empty() {
        for (combo_name, combo) in &model.load_combos {
            // If case_results already has pre-computed forces for this combo, use them directly
            if let Some(cr) = model.case_results.get(combo_name) {
                if !cr.frame_forces.is_empty() {
                    let frames_vec = cr
                        .frame_forces
                        .iter()
                        .map(|(k, v)| (k.clone(), v.clone()))
                        .collect();
                    out.push((combo_name.clone(), frames_vec));
                    continue;
                }
            }
            // Otherwise synthesize from constituent cases
            let mut frames_vec = Vec::new();
            for frame_name in model.frames.keys() {
                if let Some(stations) = synthesize_combo_frame_forces(model, combo, frame_name, 0) {
                    if !stations.is_empty() {
                        frames_vec.push((frame_name.clone(), stations));
                    }
                }
            }
            if !frames_vec.is_empty() {
                out.push((combo_name.clone(), frames_vec));
            }
        }
    }

    // If no combo forces were found, fall back to all case_results that have frame_forces
    if out.is_empty() {
        for (case_name, cr) in &model.case_results {
            if !cr.frame_forces.is_empty() {
                let frames_vec = cr
                    .frame_forces
                    .iter()
                    .map(|(k, v)| (k.clone(), v.clone()))
                    .collect();
                out.push((case_name.clone(), frames_vec));
            }
        }
    }

    out
}

fn synthesize_combo_frame_forces(
    model: &SapModel,
    combo: &LoadCombo,
    frame_name: &str,
    depth: usize,
) -> Option<Vec<FrameStationResult>> {
    if depth > 8 {
        return None;
    }
    let mut constituent_lists: Vec<(f64, Vec<FrameStationResult>)> = Vec::new();
    for (ref_name, sf) in &combo.cases {
        if let Some(cr) = model.case_results.get(ref_name) {
            if let Some(st) = cr.frame_forces.get(frame_name) {
                if !st.is_empty() {
                    constituent_lists.push((*sf, st.clone()));
                    continue;
                }
            }
        }
        if let Some(sub_combo) = model.load_combos.get(ref_name) {
            if let Some(st) = synthesize_combo_frame_forces(model, sub_combo, frame_name, depth + 1)
            {
                if !st.is_empty() {
                    constituent_lists.push((*sf, st));
                }
            }
        }
    }

    if constituent_lists.is_empty() {
        return None;
    }

    let num_stations = constituent_lists[0].1.len();

    if combo.combo_type == ComboType::Envelope {
        // An Envelope combination evaluates each constituent case/combo independently (not summed!)
        let mut env_stations = Vec::with_capacity(num_stations * constituent_lists.len());
        for (sf, list) in &constituent_lists {
            for item in list {
                env_stations.push(FrameStationResult {
                    station: item.station,
                    rel_dist: item.rel_dist,
                    p: sf * item.p,
                    v2: sf * item.v2,
                    v3: sf * item.v3,
                    t: sf * item.t,
                    m2: sf * item.m2,
                    m3: sf * item.m3,
                    local_disp: [
                        sf * item.local_disp[0],
                        sf * item.local_disp[1],
                        sf * item.local_disp[2],
                    ],
                });
            }
        }
        return Some(env_stations);
    }

    let is_unsigned = matches!(combo.combo_type, ComboType::AbsAdd | ComboType::Srss);
    let mut combined = Vec::with_capacity(if is_unsigned {
        num_stations * 2
    } else {
        num_stations
    });
    for idx in 0..num_stations {
        let base = &constituent_lists[0].1[idx];
        let mut res = FrameStationResult {
            station: base.station,
            rel_dist: base.rel_dist,
            ..Default::default()
        };
        for (sf, list) in &constituent_lists {
            let item = list.get(idx).unwrap_or_else(|| list.last().unwrap());
            match combo.combo_type {
                ComboType::LinearAdd | ComboType::RangeAdd | ComboType::Envelope => {
                    res.p += sf * item.p;
                    res.v2 += sf * item.v2;
                    res.v3 += sf * item.v3;
                    res.t += sf * item.t;
                    res.m2 += sf * item.m2;
                    res.m3 += sf * item.m3;
                    for d in 0..3 {
                        res.local_disp[d] += sf * item.local_disp[d];
                    }
                }
                ComboType::AbsAdd => {
                    let s = sf.abs();
                    res.p += s * item.p.abs();
                    res.v2 += s * item.v2.abs();
                    res.v3 += s * item.v3.abs();
                    res.t += s * item.t.abs();
                    res.m2 += s * item.m2.abs();
                    res.m3 += s * item.m3.abs();
                }
                ComboType::Srss => {
                    res.p = (res.p * res.p + (sf * item.p).powi(2)).sqrt();
                    res.v2 = (res.v2 * res.v2 + (sf * item.v2).powi(2)).sqrt();
                    res.v3 = (res.v3 * res.v3 + (sf * item.v3).powi(2)).sqrt();
                    res.t = (res.t * res.t + (sf * item.t).powi(2)).sqrt();
                    res.m2 = (res.m2 * res.m2 + (sf * item.m2).powi(2)).sqrt();
                    res.m3 = (res.m3 * res.m3 + (sf * item.m3).powi(2)).sqrt();
                }
            }
        }
        if is_unsigned {
            // For SRSS and AbsAdd, axial force magnitude can act in compression (-P) as well as tension (+P)
            let mut comp_res = res.clone();
            comp_res.p = -res.p.abs();
            combined.push(comp_res);
        }
        combined.push(res);
    }

    Some(combined)
}

/// Runs steel frame design for all steel frame members in `model` and populates `model.steel_design_results`.
pub fn run_steel_design(model: &mut SapModel) {
    let _ = try_run_steel_design(model);
}

/// Checked steel design entry point. Unsupported code names or demands return an error and leave
/// `steel_design_results` empty, so stale or incomplete ratios cannot be mistaken for a pass.
pub fn try_run_steel_design(model: &mut SapModel) -> Result<(), DesignError> {
    try_run_steel_design_with_overwrites(model, &std::collections::HashMap::new())
}

/// Runs steel frame design with optional per-member overwrites.
pub fn run_steel_design_with_overwrites(
    model: &mut SapModel,
    overwrites: &std::collections::HashMap<String, SteelMemberOverwrite>,
) {
    let _ = try_run_steel_design_with_overwrites(model, overwrites);
}

/// Checked steel design entry point with optional per-member overwrites.
pub fn try_run_steel_design_with_overwrites(
    model: &mut SapModel,
    overwrites: &std::collections::HashMap<String, SteelMemberOverwrite>,
) -> Result<(), DesignError> {
    model.steel_design_results.clear();
    let code = SteelCode::try_from_code_str(&model.steel_design_code)?;
    let combo_forces = collect_design_combo_forces(model);
    validate_demands(model, &combo_forces, "steel", true, |model, frame_name| {
        let Some(frame) = model.frames.get(frame_name) else {
            return false;
        };
        let Some(section) = model.frame_sections.get(&frame.section) else {
            return false;
        };
        model
            .materials
            .get(&section.material)
            .is_some_and(|material| {
                matches!(
                    material.mat_type,
                    MatType::Steel | MatType::ColdFormed | MatType::Aluminum
                )
            })
    })?;
    run_steel_design_validated(model, overwrites, code, combo_forces)
}

/// Computes available torsional strength $T_c$ ($\phi_T T_n$ for LRFD, $T_n / \Omega_T$ for ASD)
/// for closed sections (`Box` and `Pipe`) per AISC 360-10 Section H3.1.
pub fn compute_aisc_torsional_capacity(
    sec: &FrameSection,
    mat: &Material,
    l_ltb: f64,
    is_asd: bool,
) -> Result<f64, DesignError> {
    let e = mat.e;
    let fy = mat.fy;
    if !e.is_finite() || e <= 0.0 || !fy.is_finite() || fy <= 0.0 {
        return Err(DesignError::UnsupportedClosedSectionTorsion {
            frame: sec.name.clone(),
        });
    }
    let tn = match sec.prop_type {
        FramePropType::Pipe | FramePropType::ColdPipe => {
            let d = sec.t3;
            let t = if sec.tw > 0.0 { sec.tw } else { sec.tf };
            if !d.is_finite() || d <= 0.0 || !t.is_finite() || t <= 0.0 || d <= 2.0 * t {
                return Err(DesignError::UnsupportedClosedSectionTorsion {
                    frame: sec.name.clone(),
                });
            }
            let c = 0.5 * PI * (d - t).powi(2) * t;
            let d_over_t = d / t;
            let l = l_ltb.max(1e-6);
            let fcr1 = (1.23 * e) / ((l / d).sqrt() * d_over_t.powf(1.25));
            let fcr2 = (0.60 * e) / d_over_t.powf(1.5);
            let fcr = fcr1.max(fcr2).min(0.60 * fy);
            fcr * c
        }
        FramePropType::Box | FramePropType::ColdBox => {
            let h_outer = sec.t3;
            let b_outer = sec.t2;
            if !h_outer.is_finite()
                || h_outer <= 0.0
                || !b_outer.is_finite()
                || b_outer <= 0.0
                || !sec.tw.is_finite()
                || sec.tw <= 0.0
                || !sec.tf.is_finite()
                || sec.tf <= 0.0
            {
                return Err(DesignError::UnsupportedClosedSectionTorsion {
                    frame: sec.name.clone(),
                });
            }
            let t = sec.tw.min(sec.tf);
            if b_outer <= 3.0 * t || h_outer <= 3.0 * t {
                return Err(DesignError::UnsupportedClosedSectionTorsion {
                    frame: sec.name.clone(),
                });
            }
            let c = 2.0 * (b_outer - t) * (h_outer - t) * t - 4.5 * (4.0 - PI) * t.powi(3);
            if c <= 0.0 {
                return Err(DesignError::UnsupportedClosedSectionTorsion {
                    frame: sec.name.clone(),
                });
            }
            let h_over_t = (h_outer.max(b_outer) - 3.0 * t) / t;
            if h_over_t > 260.0 {
                return Err(DesignError::UnsupportedClosedSectionTorsion {
                    frame: sec.name.clone(),
                });
            }
            let root_efy = (e / fy).sqrt();
            let fcr = if h_over_t <= 2.45 * root_efy {
                0.60 * fy
            } else if h_over_t <= 3.07 * root_efy {
                0.60 * fy * (2.45 * root_efy) / h_over_t
            } else {
                (0.458 * PI * PI * e) / (h_over_t * h_over_t)
            };
            fcr * c
        }
        _ => {
            return Err(DesignError::UnsupportedClosedSectionTorsion {
                frame: sec.name.clone(),
            });
        }
    };
    let tc = if is_asd { tn / 1.67 } else { 0.90 * tn };
    if tc.is_finite() && tc > 0.0 {
        Ok(tc)
    } else {
        Err(DesignError::UnsupportedClosedSectionTorsion {
            frame: sec.name.clone(),
        })
    }
}

fn run_steel_design_validated(
    model: &mut SapModel,
    overwrites: &std::collections::HashMap<String, SteelMemberOverwrite>,
    code: SteelCode,
    combo_forces: Vec<(String, Vec<(String, Vec<FrameStationResult>)>)>,
) -> Result<(), DesignError> {
    let is_asd = code == SteelCode::Aisc360Asd;
    let _ = compute_ec3_rectangular_minor_shear;

    let mut results = Vec::new();

    for (frame_name, frame) in &model.frames {
        let Some(sec) = model.frame_sections.get(&frame.section) else {
            continue;
        };
        let Some(mat) = model.materials.get(&sec.material) else {
            continue;
        };
        if mat.mat_type != MatType::Steel
            && mat.mat_type != MatType::ColdFormed
            && mat.mat_type != MatType::Aluminum
        {
            continue;
        }

        if mat.mat_type != MatType::Steel {
            return Err(DesignError::UnsupportedSteelMaterial {
                frame: frame_name.clone(),
                material: mat.name.clone(),
            });
        }
        let has_ec3_v3 = code == SteelCode::Eurocode3
            && combo_forces.iter().any(|(_, frames)| {
                frames.iter().any(|(name, stations)| {
                    name == frame_name && stations.iter().any(|st| st.v3.abs() > 1e-9)
                })
            });
        if has_ec3_v3 {
            for (_, frames) in &combo_forces {
                for (_, stations) in frames.iter().filter(|(name, _)| name == frame_name) {
                    for st in stations.iter().filter(|st| st.v3.abs() > 1e-9) {
                        let tol = 1e-9 * st.v3.abs().max(1.0);
                        if st.p.abs() > tol
                            || st.v2.abs() > tol
                            || st.m3.abs() > tol
                            || st.t.abs() > tol
                        {
                            return Err(DesignError::UnsupportedEc3ShearInteraction {
                                frame: frame_name.clone(),
                                p: st.p.to_string(),
                                v2: st.v2.to_string(),
                                m3: st.m3.to_string(),
                                t: st.t.to_string(),
                            });
                        }
                    }
                }
            }
        }
        let ow = overwrites.get(frame_name);
        let l_clear = frame_clear_length(model, frame).max(1e-6);
        let l33 = ow.and_then(|o| o.l_ratio_major).unwrap_or(1.0) * l_clear;
        let l22 = ow.and_then(|o| o.l_ratio_minor).unwrap_or(1.0) * l_clear;
        let l_ltb = ow.and_then(|o| o.l_ratio_ltb).unwrap_or(1.0) * l_clear;
        let (k33, k22) = compute_member_k_factors(model, frame, ow);

        // CSI AISC 360-10 section 3.5.5 ignores torsion for open shapes, and
        // evaluates H3.1/H3.2 interaction for closed sections (Box and Pipe).
        let has_closed_torsion = matches!(
            sec.prop_type,
            FramePropType::Box
                | FramePropType::ColdBox
                | FramePropType::Pipe
                | FramePropType::ColdPipe
        ) && combo_forces.iter().any(|(_, frames)| {
            frames.iter().any(|(name, stations)| {
                name == frame_name && stations.iter().any(|station| station.t.abs() > 1e-9)
            })
        });
        let phi_tn = if has_closed_torsion {
            if code == SteelCode::Eurocode3 {
                return Err(DesignError::UnsupportedClosedSectionTorsion {
                    frame: frame_name.clone(),
                });
            }
            Some(
                compute_aisc_torsional_capacity(sec, mat, l_ltb, is_asd).map_err(|_| {
                    DesignError::UnsupportedClosedSectionTorsion {
                        frame: frame_name.clone(),
                    }
                })?,
            )
        } else {
            None
        };

        let phi_vn3 = match code {
            SteelCode::Eurocode3 if has_ec3_v3 => compute_ec3_minor_shear(sec, mat)?,
            SteelCode::Eurocode3 => 0.0,
            _ => compute_aisc_minor_shear(sec, mat, is_asd)?,
        };

        let mut best_result: Option<SteelDesignResult> = None;

        for (combo_name, frame_map) in &combo_forces {
            let Some((_, stations)) = frame_map.iter().find(|(fn_k, _)| fn_k == frame_name) else {
                continue;
            };
            if stations.is_empty() {
                continue;
            }

            let cb = ow
                .and_then(|o| o.cb)
                .unwrap_or_else(|| compute_cb_from_stations(stations));

            let (phi_pnc, phi_pnt, phi_mn33, phi_mn22, phi_vn2, compactness) = match code {
                SteelCode::Aisc360Lrfd | SteelCode::Aisc360Asd => {
                    compute_aisc_360_capacities(sec, mat, l33, l22, l_ltb, k33, k22, cb, is_asd)
                }
                SteelCode::Eurocode3 => {
                    compute_ec3_capacities(sec, mat, l33, l22, l_ltb, k33, k22, cb)
                }
            };

            for st in stations {
                // For EC3 high local-3 shear, the reported minor-axis capacity
                // must match the reduced capacity used by the ratio below.
                // All other code/station paths retain the section capacity.
                let mut reported_phi_mn22 = phi_mn22;
                let (
                    mut total_ratio,
                    mut pr_ratio,
                    mut mr_major_ratio,
                    mut mr_minor_ratio,
                    shear_major_ratio,
                    mut eq_name,
                ) = if code == SteelCode::Eurocode3 && has_ec3_v3 && st.v3.abs() > 1e-9 {
                    let shear_ratio = st.v3.abs() / phi_vn3.max(1e-12);
                    let rho = if shear_ratio > 0.5 {
                        (2.0 * shear_ratio - 1.0).powi(2)
                    } else {
                        0.0
                    };
                    let reduced_mc = ((1.0 - rho) * phi_mn22).max(1e-12);
                    reported_phi_mn22 = reduced_mc;
                    let minor_ratio = st.m2.abs() / reduced_mc;
                    (
                        shear_ratio.max(minor_ratio),
                        0.0,
                        0.0,
                        minor_ratio,
                        0.0,
                        "EC3-6.2.6+6.2.8(3)",
                    )
                } else {
                    evaluate_station_interaction(
                        code, st.p, st.v2, st.m3, st.m2, phi_pnc, phi_pnt, phi_mn33, phi_mn22,
                        phi_vn2,
                    )
                };

                let shear_minor_ratio = if phi_vn3 > 0.0 {
                    st.v3.abs() / phi_vn3
                } else {
                    0.0
                };

                // AISC H3.2 Eq. H3-6: When |Tr| > 0.20 * Tc, check combined torsion + flexure + shear + axial
                if let Some(tc) = phi_tn {
                    let tr = st.t.abs();
                    if tr > 0.20 * tc {
                        let pc = if st.p >= 0.0 {
                            phi_pnt.max(1e-12)
                        } else {
                            phi_pnc.max(1e-12)
                        };
                        let pr_over_pc = st.p.abs() / pc;
                        let mr3_over_mc3 = st.m3.abs() / phi_mn33.max(1e-12);
                        let mr2_over_mc2 = st.m2.abs() / phi_mn22.max(1e-12);
                        let vr_over_vc = shear_major_ratio.max(shear_minor_ratio);
                        let tr_over_tc = tr / tc.max(1e-12);
                        let ratio_h3 = (pr_over_pc + mr3_over_mc3 + mr2_over_mc2)
                            + (vr_over_vc + tr_over_tc).powi(2);
                        if ratio_h3 > total_ratio {
                            total_ratio = ratio_h3;
                            pr_ratio = pr_over_pc;
                            mr_major_ratio = mr3_over_mc3;
                            mr_minor_ratio = mr2_over_mc2;
                            eq_name = "H3-6";
                        }
                    }
                }

                let gov_ratio = total_ratio.max(shear_major_ratio).max(shear_minor_ratio);
                let gov_eq = if shear_minor_ratio > total_ratio.max(shear_major_ratio) {
                    "G7/G2-1 (V3)"
                } else if shear_major_ratio > total_ratio {
                    "G2-1"
                } else {
                    eq_name
                };

                let status = if gov_ratio > 1.0 {
                    "OVERSTRESSED".to_string()
                } else if compactness.is_slender() {
                    "SLENDER".to_string()
                } else {
                    "OK".to_string()
                };

                let is_more_critical = match &best_result {
                    None => true,
                    Some(prev) => gov_ratio > prev.total_ratio.max(prev.shear_major_ratio) + 1e-12,
                };

                if is_more_critical {
                    best_result = Some(SteelDesignResult {
                        frame_name: frame_name.clone(),
                        section_name: sec.name.clone(),
                        design_code: model.steel_design_code.clone(),
                        governing_combo: combo_name.clone(),
                        governing_station: st.station,
                        total_ratio: gov_ratio,
                        pr_ratio,
                        mr_major_ratio,
                        mr_minor_ratio,
                        shear_major_ratio,
                        shear_minor_ratio,
                        phi_pnc,
                        phi_pnt,
                        phi_mn33,
                        phi_mn22: reported_phi_mn22,
                        phi_vn2,
                        phi_vn3,
                        status,
                        eq_name: gov_eq.to_string(),
                    });
                }
            }
        }

        if let Some(res) = best_result {
            results.push((frame_name.clone(), res));
        }
    }

    model.steel_design_results.clear();
    for (k, v) in results {
        model.steel_design_results.insert(k, v);
    }
    Ok(())
}
