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
/// The model stores one per frame (`FrameObj::steel_overwrite`); an entry
/// passed to [`try_run_steel_design_with_overwrites`] takes precedence.
pub use sap_model::SteelDesignOverwrite as SteelMemberOverwrite;

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
            // D87: for a double channel `t2` is the total width; the flange of one channel is
            // (t2 - dis)/2.
            let bf = if sec.prop_type == FramePropType::DbChannel {
                0.5 * (sec.t2 - sec.dis)
            } else {
                sec.t2.max(sec.t2b)
            };
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
            // D87: for a double angle `t2` is the total width; one leg is (t2 - dis)/2.
            let leg2 = if sec.prop_type == FramePropType::DblAngle {
                0.5 * (sec.t2 - sec.dis)
            } else {
                sec.t2
            };
            let b = sec.t3.max(leg2);
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

/// Unstiffened-element factor $Q_s$ of three branches (AISC E7-4..6, E7-10..12, E7-13..15):
/// 1 up to `lo` $\sqrt{E/F_y}$, `a - b x / sqrt(E/Fy)` up to `hi` $\sqrt{E/F_y}$, and
/// `c E / (Fy x^2)` beyond, with `x` the width-thickness ratio.
#[allow(clippy::too_many_arguments)]
fn qs_three_branch(x: f64, e: f64, fy: f64, lo: f64, hi: f64, a: f64, b: f64, c: f64) -> f64 {
    let r = (e / fy).sqrt();
    if x <= lo * r {
        1.0
    } else if x <= hi * r {
        a - b * x / r
    } else {
        c * e / (fy * x * x)
    }
}

/// Effective width of a slender stiffened element (AISC E7-17 for webs of I shapes, channels
/// and double channels with `k = 0.34` and `lim = 1.49`, E7-18 for box walls with `k = 0.38`
/// and `lim = 1.40`): `be = 1.92 t sqrt(E/f) [1 - k / (b/t) sqrt(E/f)] <= b` when
/// `b/t >= lim sqrt(E/f)`, else `b`.
fn effective_width(b: f64, t: f64, e: f64, f: f64, k: f64, lim: f64) -> f64 {
    let r = (e / f.max(1e-12)).sqrt();
    let x = b / t.max(1e-12);
    if x >= lim * r {
        (1.92 * t * r * (1.0 - k * r / x)).min(b)
    } else {
        b
    }
}

/// Computes the slender element reduction factor $Q = Q_s Q_a$ per AISC 360-10 Section E7
/// (vendor manual 3.5.2.2.3, printed pp. 3-31 to 3-34).
///
/// `fcr_q1` is the stress $f$ of the effective width of the stiffened webs of I shapes,
/// channels and double channels: $F_{cr}$ computed with $Q = 1$ (manual p. 3-33, AISC 7.2a);
/// the walls of a box use $f = F_y$ as the manual prints it. For double angles and double
/// channels `t2` is the total width and one member's width is `(t2 - dis)/2`.
/// A Pipe with $D/t > 0.45 E/F_y$ is "too slender and not designed" (p. 3-34) and is refused
/// ([`DesignError::SlenderPipeNotDesigned`], with an empty frame name that the caller fills).
/// This function gives $Q$ only: the flexural-torsional buckling of the tee, double angle,
/// channel and double channel (E4-2 to E4-5) is applied by `compute_aisc_360_strengths` (D128); a
/// single angle is refused under compression by design (E4-5/E4-6 on its principal axes, G81).
pub fn compute_slender_q_factor(
    sec: &FrameSection,
    e: f64,
    fy: f64,
    fcr_q1: f64,
) -> Result<f64, DesignError> {
    match sec.prop_type {
        FramePropType::I | FramePropType::Channel | FramePropType::DbChannel => {
            let tf = sec.tf.max(1e-9);
            let tw = sec.tw.max(1e-9);
            let h = (sec.t3 - sec.tf - sec.tfb).max(0.0);
            // b/t of the flange: bf/2tf for I, bf/tf for a channel and a double channel,
            // whose bf is one channel's flange (p. 3-32, E7.1).
            let x_f = match sec.prop_type {
                FramePropType::I => sec.t2.max(sec.t2b) / (2.0 * tf),
                FramePropType::Channel => sec.t2 / tf,
                _ => 0.5 * (sec.t2 - sec.dis) / tf,
            };
            let qs = qs_three_branch(x_f, e, fy, 0.56, 1.03, 1.415, 0.74, 0.69);
            let be = effective_width(h, tw, e, fcr_q1, 0.34, 1.49);
            let a_eff = sec.area - (h - be) * tw;
            Ok(qs * (a_eff / sec.area.max(1e-12)).min(1.0))
        }
        FramePropType::T => {
            // The flange (bf/2tf) and the stem (d/tw, E7-13..15) are unstiffened elements: the
            // smaller Qs is used and Qa = 1.
            let tf = sec.tf.max(1e-9);
            let tw = sec.tw.max(1e-9);
            let qs_flange =
                qs_three_branch(sec.t2 / (2.0 * tf), e, fy, 0.56, 1.03, 1.415, 0.74, 0.69);
            // The vendor page prints the middle branch as 1.908 - 1.22 sqrt(Fy/E) without the
            // ratio d/t, a misprint: AISC E7-14 has (d/t), and only then do the branches meet
            // at 0.75 and 1.03 sqrt(E/Fy).
            let qs_stem = qs_three_branch(sec.t3 / tw, e, fy, 0.75, 1.03, 1.908, 1.22, 0.69);
            Ok(qs_flange.min(qs_stem))
        }
        FramePropType::Angle | FramePropType::DblAngle => {
            let t = if sec.tf > 0.0 { sec.tf } else { sec.tw }.max(1e-9);
            let leg2 = if sec.prop_type == FramePropType::DblAngle {
                0.5 * (sec.t2 - sec.dis)
            } else {
                sec.t2
            };
            // Qs of the leg that gives the largest b/t (E7-10..12).
            let x = sec.t3.max(leg2) / t;
            Ok(qs_three_branch(x, e, fy, 0.45, 0.91, 1.34, 0.76, 0.53))
        }
        FramePropType::Box | FramePropType::ColdBox => {
            // E7-18 with f = Fy; four walls: two flanges (b = t2 - 3 tw, t = tf) and two
            // webs (b = t3 - 3 tf, t = tw). Qs = 1.
            let tf = sec.tf.max(1e-9);
            let tw = sec.tw.max(1e-9);
            let b_f = (sec.t2 - 3.0 * tw).max(0.0);
            let b_w = (sec.t3 - 3.0 * tf).max(0.0);
            let lost = 2.0 * (b_f - effective_width(b_f, tf, e, fy, 0.38, 1.40)) * tf
                + 2.0 * (b_w - effective_width(b_w, tw, e, fy, 0.38, 1.40)) * tw;
            Ok(((sec.area - lost) / sec.area.max(1e-12)).min(1.0))
        }
        FramePropType::Pipe | FramePropType::ColdPipe => {
            let t = sec.tw.max(sec.tf).max(1e-9);
            let lambda = sec.t3 / t;
            let efy = e / fy.max(1e-12);
            if lambda <= 0.11 * efy {
                Ok(1.0)
            } else if lambda <= 0.45 * efy {
                Ok(0.038 * efy / lambda + 2.0 / 3.0)
            } else {
                Err(DesignError::SlenderPipeNotDesigned {
                    frame: String::new(),
                    section: sec.name.clone(),
                    d_over_t: format!("{lambda}"),
                    limit: format!("{}", 0.45 * efy),
                })
            }
        }
        _ => Ok(1.0),
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

/// Coordinate along the output stations of one member: the relative position `rel_dist`
/// (`station / L`), or the distance `station` when `rel_dist` is not populated.
///
/// The stations of a member with end offsets run from the face of the support at the I end
/// to the face at the J end (CSI Analysis Reference, "Effect upon Internal Force Output"),
/// so the first has `rel_dist > 0` and the last `rel_dist < 1`. The unbraced length of the
/// design is the length between those faces (`frame_clear_length`), and the quarter points
/// of $C_b$ (AISC F1-1: $M_A$, $M_B$, $M_C$ "at quarter point, centerline and three-quarter
/// point of the unbraced segment") refer to that length: `at(u)` maps the fraction `u` of the
/// span between the first and the last station to a coordinate that lies inside the stations.
/// Without offsets the first station is at 0 and the last at 1 and `at(u)` is `u` itself.
struct StationSpan {
    coordinate: Vec<f64>,
    first: f64,
    last: f64,
}

impl StationSpan {
    fn new(stations: &[FrameStationResult]) -> Self {
        let rel: Vec<f64> = stations.iter().map(|s| s.rel_dist).collect();
        let spread = |v: &[f64]| v.iter().cloned().fold(f64::NEG_INFINITY, f64::max)
            - v.iter().cloned().fold(f64::INFINITY, f64::min);
        let coordinate = if spread(&rel) > 1e-12 {
            rel
        } else {
            stations.iter().map(|s| s.station).collect()
        };
        let first = coordinate.iter().cloned().fold(f64::INFINITY, f64::min);
        let last = coordinate.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        Self { coordinate, first, last }
    }

    /// Coordinate of the fraction `u` of the span between the first and the last station.
    /// `u` is held inside [0, 1]: a moment is never read outside the stations.
    fn at(&self, u: f64) -> f64 {
        self.first + u.clamp(0.0, 1.0) * (self.last - self.first)
    }

    /// The stations sorted by coordinate as (coordinate, M3) pairs.
    fn moments(&self, stations: &[FrameStationResult]) -> Vec<(f64, f64)> {
        let mut v: Vec<(f64, f64)> = self
            .coordinate
            .iter()
            .zip(stations)
            .map(|(&x, s)| (x, s.m3))
            .collect();
        v.sort_by(|a, b| a.0.total_cmp(&b.0));
        v
    }
}

/// Linear interpolation of M3 at coordinate `x` between the two stations around it; the
/// stations must be sorted and `x` inside their range (guaranteed by `StationSpan::at`).
fn interpolate_m3(sorted: &[(f64, f64)], x: f64, tol: f64) -> f64 {
    for w in sorted.windows(2) {
        let (s0, s1) = (w[0].0, w[1].0);
        if x >= s0 - tol && x <= s1 + tol {
            let t = ((x - s0) / (s1 - s0).max(1e-12)).clamp(0.0, 1.0);
            return w[0].1 + t * (w[1].1 - w[0].1);
        }
    }
    // Only reachable by roundoff at the ends of the range: the nearest station.
    if x <= sorted[0].0 {
        sorted[0].1
    } else {
        sorted[sorted.len() - 1].1
    }
}

/// Moment gradient factor for a member braced at equal spacings of `ratio`
/// times its (clear) length: $C_b$ is computed within each unbraced segment and the
/// smallest value is used for the member. With `ratio >= 1` the member is one
/// segment. (SAP2000 likewise bases $C_b$ on the moments of each unbraced
/// segment, not of the whole member.) The segments and their quarter points are
/// measured between the first and the last output station, which for a member with end
/// offsets are the faces of the supports (see `StationSpan`).
pub fn compute_cb_for_unbraced_ratio(stations: &[FrameStationResult], ratio: f64) -> f64 {
    if !(ratio > 0.0 && ratio < 1.0) || stations.len() < 2 {
        return compute_cb_from_stations(stations);
    }
    let span = StationSpan::new(stations);
    let sorted = span.moments(stations);
    let moment_at = |u: f64| -> f64 { interpolate_m3(&sorted, span.at(u), 1e-12) };
    // Bracing closer than a thousandth of the length is continuous bracing;
    // the moment gradient within such a segment is flat.
    if ratio < 1e-3 {
        return 1.0;
    }
    let segments = (1.0 / ratio - 1e-9).ceil().max(1.0) as usize;
    let mut smallest = f64::INFINITY;
    for k in 0..segments {
        let (a, b) = (k as f64 * ratio, ((k + 1) as f64 * ratio).min(1.0));
        if b <= a {
            continue;
        }
        let (xa, xb) = (span.at(a), span.at(b));
        let mut m_max = moment_at(a).abs().max(moment_at(b).abs());
        for &(_, m3) in sorted.iter().filter(|(x, _)| *x > xa && *x < xb) {
            m_max = m_max.max(m3.abs());
        }
        if m_max <= 1e-12 {
            continue;
        }
        let quarter = |t: f64| moment_at(a + t * (b - a)).abs();
        let cb = 12.5 * m_max / (2.5 * m_max + 3.0 * quarter(0.25) + 4.0 * quarter(0.5) + 3.0 * quarter(0.75));
        smallest = smallest.min(cb.clamp(1.0, 3.0));
    }
    if smallest.is_finite() {
        smallest
    } else {
        1.0
    }
}

/// Computes the AISC Chapter F lateral-torsional buckling moment gradient factor:
/// $$C_b = \frac{12.5 M_{\max}}{2.5 M_{\max} + 3 M_A + 4 M_B + 3 M_C}$$
/// from a sequence of station results along the member. The quarter points are those of the
/// span between the first and the last station (the faces of the supports for a member with
/// end offsets; see `StationSpan`).
pub fn compute_cb_from_stations(stations: &[FrameStationResult]) -> f64 {
    if stations.len() < 2 {
        return 1.0;
    }
    let m_max = stations.iter().map(|s| s.m3.abs()).fold(0.0_f64, f64::max);
    if m_max <= 1e-12 {
        return 1.0;
    }

    let span = StationSpan::new(stations);
    let sorted = span.moments(stations);
    let sample_abs_m3 = |u: f64| -> f64 { interpolate_m3(&sorted, span.at(u), 1e-9).abs() };

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

/// Section properties that the model does not hold and the AISC 360-10 limit states of the open
/// shapes need (D128, D129; G86): the model keeps no centroid and no shear centre, so they are
/// computed from the plate dimensions, without fillets (documented in `design-fixes.md`).
///
/// - `x0`, `y0`: coordinates of the shear centre with respect to the centroid (manual p. 3-27;
///   `x0` along the 3 axis for a channel, `y0` along the symmetry axis 2 for a tee and a double
///   angle, the other one 0).
/// - `cw`: warping constant. 0 for the tee and the double angle (a lower bound; the vendor gives
///   no Cw for them and its nonslender form E4-3 has no Cw term).
/// - `ybar`: distance from the outer face of the flange (the +2 face) to the centroid, tee and
///   double angle only.
#[derive(Debug, Clone, Copy, PartialEq)]
struct OpenShapeGeometry {
    x0: f64,
    y0: f64,
    cw: f64,
    ybar: f64,
}

/// Shear centre of a channel measured from the back of the web, `e0 = e - tw/2`, with `e` the
/// thin-walled distance from the web centreline `e = 3 b'^2 tf / (6 b' tf + h tw)`,
/// `b' = bf - tw/2`, `h = d - tf`. The vendor's v6 section libraries hold the same quantity
/// (C10x15.3: 0.796 there, 0.7959 here). Channel dimensions: `t3 = d`, `t2 = bf` (never
/// `t2b`/`tfb`, which a channel built from a default section carries unrelated values in).
pub fn channel_shear_centre_e0(sec: &FrameSection) -> f64 {
    let (tf, tw) = (sec.tf, sec.tw);
    let bp = sec.t2 - 0.5 * tw;
    let h = sec.t3 - tf;
    3.0 * bp * bp * tf / (6.0 * bp * tf + h * tw) - 0.5 * tw
}

/// Warping constant of a channel (AISC Design Guide 9; the manual names Cw without a formula):
/// `Cw = tf b'^3 h^2 / 12 * (3 b' tf + 2 h tw) / (6 b' tf + h tw)`.
pub fn channel_warping_constant(sec: &FrameSection) -> f64 {
    let (tf, tw) = (sec.tf, sec.tw);
    let bp = sec.t2 - 0.5 * tw;
    let h = sec.t3 - tf;
    tf * bp.powi(3) * h * h / 12.0 * (3.0 * bp * tf + 2.0 * h * tw) / (6.0 * bp * tf + h * tw)
}

/// Warping constant of two channels back to back (`t2` the total width, `dis` the gap between
/// the webs): the thin-walled sectorial integral with the pole at the centroid, with
/// `a = dis/2 + tw/2`, `b' = (t2 - dis)/2 - tw/2`, `h = t3 - tf`:
/// `Cw = a^2 tw h^3 / 6 + h^2 tf [a^3 - (a - b')^3] / 3`.
pub fn double_channel_warping_constant(sec: &FrameSection) -> f64 {
    let (tf, tw) = (sec.tf, sec.tw);
    let a = 0.5 * sec.dis + 0.5 * tw;
    let bp = 0.5 * (sec.t2 - sec.dis) - 0.5 * tw;
    let h = sec.t3 - tf;
    a * a * tw * h.powi(3) / 6.0 + h * h * tf * (a.powi(3) - (a - bp).powi(3)) / 3.0
}

/// The existing angle code takes the leg thickness from `tf` or, when that is 0, from `tw`
/// (`compute_slender_q_factor`); a double angle with only one of them set is read the same way
/// here, so it is neither refused nor divided by zero.
fn angle_leg_thickness(sec: &FrameSection) -> std::borrow::Cow<'_, FrameSection> {
    if sec.prop_type == FramePropType::DblAngle && ((sec.tf > 0.0) != (sec.tw > 0.0)) {
        let mut s = sec.clone();
        let t = s.tf.max(s.tw);
        s.tf = t;
        s.tw = t;
        std::borrow::Cow::Owned(s)
    } else {
        std::borrow::Cow::Borrowed(sec)
    }
}

/// Whether the dimensions of a T, double angle, channel or double channel define the section
/// (used by the refusals and by [`open_shape_geometry`]).
fn open_shape_dimensions_ok(sec: &FrameSection) -> bool {
    let (tf, tw) = (sec.tf, sec.tw);
    if !(tf > 0.0 && tw > 0.0 && sec.t3 > 0.0 && sec.t2 > 0.0) {
        return false;
    }
    match sec.prop_type {
        FramePropType::Channel => sec.t3 > 2.0 * tf && sec.t2 > tw,
        FramePropType::DbChannel => sec.t3 > 2.0 * tf && 0.5 * (sec.t2 - sec.dis) > tw,
        FramePropType::T => sec.t3 > tf,
        FramePropType::DblAngle => sec.t3 > tf && sec.t2 > sec.dis,
        _ => true,
    }
}

/// See [`OpenShapeGeometry`]; `None` for other shapes or when the dimensions do not define the
/// section.
fn open_shape_geometry(sec: &FrameSection) -> Option<OpenShapeGeometry> {
    if !open_shape_dimensions_ok(sec) {
        return None;
    }
    let (tf, tw) = (sec.tf, sec.tw);
    match sec.prop_type {
        FramePropType::Channel => {
            // Centroid from the back of the web, plates: the web d tw at tw/2 and the two
            // flanges (bf - tw) tf at tw + (bf - tw)/2 (G86: the library centroid is dropped).
            let fl = sec.t2 - tw;
            let a_tot = sec.t3 * tw + 2.0 * fl * tf;
            let xbar = (sec.t3 * tw * 0.5 * tw + 2.0 * fl * tf * (tw + 0.5 * fl)) / a_tot;
            let e0 = channel_shear_centre_e0(sec);
            Some(OpenShapeGeometry {
                // shear centre to centroid along the 3 axis: x0 = xbar + e - tw/2 = xbar + e0
                x0: xbar + e0,
                y0: 0.0,
                cw: channel_warping_constant(sec),
                ybar: 0.0,
            })
        }
        FramePropType::DbChannel => Some(OpenShapeGeometry {
            x0: 0.0,
            y0: 0.0,
            cw: double_channel_warping_constant(sec),
            ybar: 0.0,
        }),
        FramePropType::T | FramePropType::DblAngle => {
            // Flange (the horizontal legs of a double angle) on the +2 face; ybar from its outer
            // face, plates. The shear centre is at the meeting of the two centrelines.
            let (flange_w, stems) = if sec.prop_type == FramePropType::T {
                (sec.t2, 1.0)
            } else {
                (2.0 * 0.5 * (sec.t2 - sec.dis), 2.0)
            };
            let a_f = flange_w * tf;
            let a_s = stems * (sec.t3 - tf) * tw;
            let ybar = (a_f * 0.5 * tf + a_s * (tf + 0.5 * (sec.t3 - tf))) / (a_f + a_s);
            Some(OpenShapeGeometry {
                x0: 0.0,
                y0: ybar - 0.5 * tf,
                cw: 0.0,
                ybar,
            })
        }
        _ => None,
    }
}

/// AISC E4-5 (the form of E4-2 and E4-3 for the stress `Fcr` is the same): the flexural-torsional
/// buckling value of two elastic values `fa` (about the axis of no twist) and `fez`, with the
/// shear-centre factor `h` (E4-10).
fn aisc_e4_5(fa: f64, fez: f64, h: f64) -> f64 {
    let sum = fa + fez;
    sum / (2.0 * h) * (1.0 - (1.0 - 4.0 * fa * fez * h / (sum * sum)).max(0.0).sqrt())
}

/// F6 flange local buckling of the minor axis (also F9's FLB of the tee with `0.7 Fy S`): `mp`
/// at or below `lp`, the linear form up to `lr`, `slender` beyond.
fn flb_linear(mp: f64, fy: f64, s: f64, lambda: f64, lp: f64, lr: f64, slender: f64) -> f64 {
    if lambda <= lp {
        mp
    } else if lambda <= lr {
        mp - (mp - 0.7 * fy * s) * ((lambda - lp) / (lr - lp))
    } else {
        slender
    }
}

/// The factored strengths of [`compute_aisc_360_strengths`]. `phi_mn33_pos` is for M3 >= 0, which
/// compresses the +2 face (Analysis Reference p. 143): the flange of a tee, whose stem is then in
/// tension. `phi_mn33_neg` is for M3 < 0. `phi_mn33_pos_stem_compressed` is for a station with
/// M3 >= 0 of a state whose moment is negative at another station: the lateral-torsional value
/// then takes the minus sign of B (see the F9 comment). They differ only for T and DblAngle.
#[derive(Debug, Clone, Copy)]
pub struct AiscStrengths {
    pub phi_pnc: f64,
    pub phi_pnt: f64,
    pub phi_mn33_pos: f64,
    pub phi_mn33_pos_stem_compressed: f64,
    pub phi_mn33_neg: f64,
    pub phi_mn22: f64,
    pub phi_vn2: f64,
    pub compactness: SectionCompactness,
}

/// Computes factored capacities $(\phi_c P_{nc}, \phi_t P_{nt}, \phi_b M_{n33}, \phi_b M_{n22}, \phi_v V_{n2})$
/// per AISC 360-10 / 360-16 (LRFD or ASD); `phi_mn33` is the lower of the two signs of M3, see
/// [`compute_aisc_360_strengths`].
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
) -> Result<(f64, f64, f64, f64, f64, SectionCompactness), DesignError> {
    let s = compute_aisc_360_strengths(sec, mat, l33, l22, l_ltb, k33, k22, cb, is_asd)?;
    Ok((
        s.phi_pnc,
        s.phi_pnt,
        s.phi_mn33_pos.min(s.phi_mn33_neg),
        s.phi_mn22,
        s.phi_vn2,
        s.compactness,
    ))
}

/// The AISC 360-10 strengths of one section (vendor manual 3.5, printed pp. 3-24 to 3-63):
///
/// - D128, compression: `Pn = A min(flexural, flexural-torsional)`: E4-2/E4-3 for a nonslender
///   tee and double angle, E4-5 with E7 for a slender one, a channel (E4-5, axis 3 of symmetry)
///   and E4-4 for a double channel (pp. 3-26 to 3-30). I is unchanged; closed shapes ignore E4.
/// - D129, flexure: Pipe F8, Box F7 (both axes), channel and double channel F2 with the shape's
///   own Cw plus F3 flange buckling (web compact) and F6, tee and double angle F9 and F6, in
///   `phi_mn33_pos` and `phi_mn33_neg` by the sign of M3 (and `phi_mn33_pos_stem_compressed`).
///
/// What the manual does not give, or the model cannot supply, is refused by
/// `design_frame_with_section` before this function is used (G81 single angle, G82 slender box
/// flange, G83 channel with a noncompact or slender web, J = 0, dimensions that do not define
/// the section); the values this function returns for those cases (the flexural compression of a
/// single angle, the yield moment for a slender box flange, F2/F3 for a channel web beyond 3.76
/// sqrt(E/Fy)) are not used by design and are kept as before.
pub fn compute_aisc_360_strengths(
    sec_in: &FrameSection,
    mat: &Material,
    l33: f64,
    l22: f64,
    l_ltb: f64,
    k33: f64,
    k22: f64,
    cb: f64,
    is_asd: bool,
) -> Result<AiscStrengths, DesignError> {
    let sec_leg = angle_leg_thickness(sec_in);
    let sec: &FrameSection = &sec_leg;
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

    // 1. Tensile capacity (AISC D2: Yielding in gross section)
    let p_nt = fy * ag;
    let phi_pnt = if is_asd { p_nt / 1.67 } else { 0.90 * p_nt };

    // 2. Compressive capacity (AISC E3, E4, E7)
    let kl_r33 = (k33 * l33) / r33;
    let kl_r22 = (k22 * l22) / r22;
    let kl_r = kl_r33.max(kl_r22).max(1e-6);
    let fe_flex = (PI * PI * e) / (kl_r * kl_r);

    // Torsional buckling elastic stress Fez for the doubly-symmetric I-sections (AISC Eq. E4-4);
    // D128: the double channel is grouped with the I-shape there (E4-4, p. 3-26/3-30) with its
    // own Cw, and the channel, tee and double angle take the flexural-torsional E4-5 (p. 3-27,
    // 3-30) or, for a nonslender tee and double angle, E4-2/E4-3 (p. 3-26) on Fcr below.
    let mut fe = if sec.prop_type == FramePropType::I && sec.torsional_j > 0.0 && sec.t3 > sec.tf {
        let h0 = (sec.t3 - 0.5 * (sec.tf + sec.tfb)).max(1e-6);
        let cw = 0.25 * sec.i22 * h0 * h0;
        let r0_sq = r33 * r33 + r22 * r22;
        let kz_lz = (k22 * l22).max(1e-6);
        let fez = ((PI * PI * e * cw) / (kz_lz * kz_lz) + g * sec.torsional_j) / (ag * r0_sq);
        fe_flex.min(fez)
    } else {
        fe_flex
    };
    // Kz Lz = K22 L22 (p. 3-28). A shape with J = 0 or with dimensions that do not define it
    // keeps the flexural value here; design refuses such a member before this function.
    let ftb_geo = match sec.prop_type {
        FramePropType::T
        | FramePropType::DblAngle
        | FramePropType::Channel
        | FramePropType::DbChannel
            if sec.torsional_j > 0.0 =>
        {
            open_shape_geometry(sec)
        }
        _ => None,
    };
    // Nonslender tee and double angle: a cap on Fcr (E4-2, E4-3).
    let mut fcr_ftb_cap: Option<f64> = None;
    if let Some(geo) = ftb_geo {
        let kz_lz = (k22 * l22).max(1e-6);
        let r0_sq = geo.x0 * geo.x0 + geo.y0 * geo.y0 + (sec.i22 + sec.i33) / ag;
        let h_fac = 1.0 - (geo.x0 * geo.x0 + geo.y0 * geo.y0) / r0_sq;
        let fez_open = (PI * PI * e * geo.cw / (kz_lz * kz_lz) + g * sec.torsional_j) / (ag * r0_sq);
        match sec.prop_type {
            FramePropType::T | FramePropType::DblAngle => {
                let fe22 = (PI * PI * e) / (kl_r22.max(1e-6) * kl_r22.max(1e-6));
                // Q of these two shapes does not depend on the stress argument.
                let q_ts = compute_slender_q_factor(sec, e, fy, 0.0)?;
                if q_ts == 1.0 {
                    let fcr22 = if kl_r22 <= 4.71 * (e / fy).sqrt() || fy / fe22 <= 2.25 {
                        0.658_f64.powf(fy / fe22) * fy
                    } else {
                        0.877 * fe22
                    };
                    // Fcrz = GJ/(A r0^2): E4-3 with Cw = 0, the vendor's nonslender form.
                    let fcrz = g * sec.torsional_j / (ag * r0_sq);
                    fcr_ftb_cap = Some(aisc_e4_5(fcr22, fcrz, h_fac));
                } else {
                    // E4-5 with Fe22, Fez (Cw = 0). The page prints (Fe33 + Fez)^2 in the
                    // denominator for these shapes, a misprint: the nonslender form on p. 3-26
                    // and AISC E4-5 have (Fe22 + Fez)^2, and only then does H = 1 give
                    // min(Fe22, Fez).
                    fe = fe.min(aisc_e4_5(fe22, fez_open, h_fac));
                }
            }
            FramePropType::Channel => {
                let fe33 = (PI * PI * e) / (kl_r33.max(1e-6) * kl_r33.max(1e-6));
                fe = fe.min(aisc_e4_5(fe33, fez_open, h_fac));
            }
            _ => {
                // Double channel (E4-4): Fez = (pi^2 E Cw/(Kz Lz)^2 + G J)/(I22 + I33).
                let fez = (PI * PI * e * geo.cw / (kz_lz * kz_lz) + g * sec.torsional_j)
                    / (sec.i22 + sec.i33);
                fe = fe.min(fez);
            }
        }
    }

    // E7: the effective width of a slender web uses f = Fcr computed with Q = 1 (manual
    // p. 3-33), so Q is evaluated here, after Fe.
    let fcr_q1 = if kl_r <= 4.71 * (e / fy).sqrt() || fy / fe <= 2.25 {
        0.658_f64.powf(fy / fe) * fy
    } else {
        0.877 * fe
    };
    let q = compute_slender_q_factor(sec, e, fy, fcr_q1)?;
    let slenderness_limit = 4.71 * (e / (q * fy)).sqrt();
    let fcr = if kl_r <= slenderness_limit || (q * fy / fe) <= 2.25 {
        q * 0.658_f64.powf((q * fy) / fe) * fy
    } else {
        0.877 * fe
    };
    let fcr = fcr_ftb_cap.map_or(fcr, |cap| fcr.min(cap));
    let p_nc = fcr * ag;
    let phi_pnc = if is_asd { p_nc / 1.67 } else { 0.90 * p_nc };

    // 3. Major-axis Flexural capacity Mn33 (AISC Chapter F)
    let s33 = sec.s33.max(1e-9);
    let z33 = if sec.z33 > 1e-9 { sec.z33 } else { 1.12 * s33 };
    let mp33 = (fy * z33).min(1.6 * fy * s33);

    let root_efy = (e / fy).sqrt();
    let channel_like = matches!(
        sec.prop_type,
        FramePropType::Channel | FramePropType::DbChannel
    );
    let geo = open_shape_geometry(sec);
    // The shape's own warping constant and the F2-8a/F2-8b factor c for a channel and a double
    // channel (D129; they took the I-shape Cw = 0.25 Iy h0^2 and c = 1 before, which is
    // unconservative for them). h0 = d - tf for both.
    let own_cw = if channel_like {
        geo.map(|g| {
            let h0 = sec.t3 - sec.tf;
            let c = if sec.prop_type == FramePropType::Channel {
                0.5 * h0 * (sec.i22 / g.cw).sqrt() // F2-8b
            } else {
                1.0 // F2-8a
            };
            (g.cw, h0, c)
        })
    } else {
        None
    };

    let mn_ltb = match sec.prop_type {
        FramePropType::I
        | FramePropType::Channel
        | FramePropType::DbChannel
        | FramePropType::BuiltUpICoverplate
        | FramePropType::BuiltUpIHybrid => {
            let (cw, h0, c_factor) = own_cw.unwrap_or_else(|| {
                let h0 = (sec.t3 - 0.5 * (sec.tf + sec.tfb)).max(1e-6);
                (0.25 * sec.i22 * h0 * h0, h0, 1.0)
            });
            let rts_sq = (sec.i22 * cw).sqrt() / s33;
            let rts = if rts_sq > 1e-12 {
                rts_sq.sqrt()
            } else {
                r22 * 1.15
            };
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
                // F2-3 for l_ltb > Lr (p. 3-53 prints "Lp > Lr" for this branch, a misprint).
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
    } else if channel_like && geo.is_some() {
        // F3 with a compact web (h/tw <= 3.76 sqrt(E/Fy); the other webs are refused by design,
        // G83). b is the full flange of the channel (Table 3-1, p. 3-13: bf = t2, and one
        // channel's width (t2 - dis)/2 for the double channel); pp. 3-55/56 print bf/2tf, a
        // misprint against Table 3-1 and the manual's own definition of b.
        let bf = if sec.prop_type == FramePropType::DbChannel {
            0.5 * (sec.t2 - sec.dis)
        } else {
            sec.t2
        };
        let lambda_f = bf / sec.tf;
        let h = sec.t3 - 2.0 * sec.tf;
        // p. 3-55 prints "35 <= kc", a misprint for 0.35.
        let kc = (4.0 / (h / sec.tw).sqrt()).clamp(0.35, 0.76);
        flb_linear(
            mp33,
            fy,
            s33,
            lambda_f,
            0.38 * root_efy,
            1.00 * root_efy,
            (0.90 * e * kc * s33) / (lambda_f * lambda_f),
        )
    } else {
        mp33
    };

    let mut mn33_pos = mp33.min(mn_ltb).min(mn_flb);
    let mut mn33_neg = mn33_pos;
    // Tee and double angle only: the strength for M3 >= 0 with the minus sign of B.
    let mut mn33_pos_rev: Option<f64> = None;

    // 4. Minor-axis Flexural capacity Mn22 (AISC F6)
    let s22 = sec.s22.max(1e-9);
    let z22 = if sec.z22 > 1e-9 { sec.z22 } else { 1.5 * s22 };
    let mp22 = (fy * z22).min(1.6 * fy * s22);
    let mut mn22 = if sec.prop_type == FramePropType::I && sec.tf > 0.0 {
        let bf = sec.t2.max(sec.t2b);
        let lambda_f = bf / (2.0 * sec.tf);
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

    match sec.prop_type {
        // D129: Pipe, F8 (p. 3-58), both axes with the modulus of that axis. The page prints
        // Table 3-1's lambda_p as 0.07 sqrt(E/Fy), a misprint: the same row's lambda_r and
        // lambda_s, AISC Table B4.1b case 20 and `check_compactness_aisc` use E/Fy.
        FramePropType::Pipe | FramePropType::ColdPipe => {
            let t = if sec.tw > 0.0 { sec.tw } else { sec.tf }.max(1e-9);
            let lambda = sec.t3 / t;
            let efy = e / fy;
            let pipe = |mp: f64, s: f64| -> f64 {
                let mn = if lambda <= 0.07 * efy {
                    mp
                } else if lambda <= 0.31 * efy {
                    (0.021 * e / lambda + fy) * s
                } else {
                    0.33 * e / lambda * s
                };
                mn.min(mp)
            };
            mn33_pos = pipe(mp33, s33);
            mn33_neg = mn33_pos;
            mn22 = pipe(mp22, s22);
        }
        // D129: Box, F7 (pp. 3-56 to 3-58), the same formulas for both axes with the elements
        // swapped. Flange: lambda_p = 1.12 r, lambda_r = 1.40 r (a slender flange is refused by
        // design, G82; Mp is returned here); web: 2.42 r and 5.70 r, and a slender web uses the
        // noncompact formula as the vendor's note says (pp. 3-57/58) although AISC does not cover
        // slender box webs.
        FramePropType::Box | FramePropType::ColdBox => {
            let box_mn = |mp: f64, s: f64, b: f64, tb: f64, h: f64, th: f64| -> f64 {
                let lam_f = b / tb.max(1e-9);
                let lam_w = h / th.max(1e-9);
                let flb = if lam_f <= 1.12 * root_efy || lam_f > 1.40 * root_efy {
                    mp
                } else {
                    (mp - (mp - fy * s) * (3.57 * lam_f / root_efy - 4.0)).min(mp)
                };
                let wlb = if lam_w <= 2.42 * root_efy {
                    mp
                } else {
                    (mp - (mp - fy * s) * (0.305 * lam_w / root_efy - 0.738)).min(mp)
                };
                mp.min(flb).min(wlb)
            };
            let tf = sec.tf;
            let tw = sec.tw;
            mn33_pos = box_mn(
                mp33,
                s33,
                (sec.t2 - 3.0 * tw).max(0.0),
                tf,
                (sec.t3 - 3.0 * tf).max(0.0),
                tw,
            );
            mn33_neg = mn33_pos;
            mn22 = box_mn(
                mp22,
                s22,
                (sec.t3 - 3.0 * tf).max(0.0),
                tw,
                (sec.t2 - 3.0 * tw).max(0.0),
                tf,
            );
        }
        // D129: minor axis of a channel and a double channel, F6 (p. 3-56): the same lambda,
        // 0.38 r and 1.0 r as F3, `0.69 E S22 / lambda^2` beyond.
        FramePropType::Channel | FramePropType::DbChannel if geo.is_some() => {
            let bf = if sec.prop_type == FramePropType::DbChannel {
                0.5 * (sec.t2 - sec.dis)
            } else {
                sec.t2
            };
            let lambda = bf / sec.tf;
            mn22 = flb_linear(
                mp22,
                fy,
                s22,
                lambda,
                0.38 * root_efy,
                1.00 * root_efy,
                0.69 * e * s22 / (lambda * lambda),
            );
        }
        // D129: tee and double angle, F9 (pp. 3-58 to 3-60) and F6 (p. 3-61).
        FramePropType::T | FramePropType::DblAngle if geo.is_some() => {
            let ybar = geo.map_or(1.0, |g| g.ybar);
            let is_tee = sec.prop_type == FramePropType::T;
            let b1 = 0.5 * (sec.t2 - sec.dis);
            // lambda of the flange and its limits (Table 3-1; double angle: case 12, "any type
            // of compression in leg").
            let (lambda, lp, lr) = if is_tee {
                (sec.t2 / (2.0 * sec.tf), 0.38 * root_efy, 1.00 * root_efy)
            } else {
                (b1 / sec.tf, 0.54 * root_efy, 0.91 * root_efy)
            };
            // Yielding: My = Fy S33 with the model's S33 (the stem-tip modulus, AISC Sx).
            let my = fy * s33;
            let mp_pos = (fy * z33).min(1.6 * my);
            let mp_neg = (fy * z33).min(my);
            // LTB (F9-4, F9-5): B = +2.3 (d/Lb) sqrt(Iy/J) with the stem in tension (M > 0),
            // -B with the stem in compression (M < 0), as v19 and v24 p. 3-59 print it. The
            // page does not say which moment decides when the moment changes sign inside the
            // unbraced length. On the safe side the minus sign is taken for every station of a
            // state whose stem is compressed at any station (`mn33_pos_rev`); the buckling is
            // of the unbraced length, not of a station. No Cb is printed for F9.
            let ltb = |sign: f64| -> f64 {
                if sec.torsional_j <= 0.0 {
                    return f64::INFINITY;
                }
                let lb = l_ltb.max(1e-9);
                let b = sign * 2.3 * (sec.t3 / lb) * (sec.i22 / sec.torsional_j).sqrt();
                let root = (1.0 + b * b).sqrt();
                // b + root without cancellation for b < 0
                let factor = if b < 0.0 { 1.0 / (root - b) } else { b + root };
                PI * (e * sec.i22 * g * sec.torsional_j).sqrt() / lb * factor
            };
            // FLB, positive moment only (F9-6, F9-7): Sxc = I33/ybar is the modulus of the
            // compression flange. The v19 page prints S33 in the noncompact form and S22 in the
            // slender one (misprints); the v24 manual (p. 3-59) prints S33c in both, which is the
            // source here. The double-angle limits make the printed rule jump up at lambda_r
            // (0.7 E Sxc/(0.91 r)^2 = 0.845 Fy Sxc against 0.7 Fy Sxc): built as printed.
            let sxc = sec.i33 / ybar;
            let flb = if lambda <= lp {
                mp_pos
            } else if lambda <= lr {
                (mp_pos - (mp_pos - 0.7 * fy * sxc) * ((lambda - lp) / (lr - lp))).min(1.6 * my)
            } else {
                0.7 * e * sxc / (lambda * lambda)
            };
            // Stem local buckling, negative moment only (p. 3-60, F9-8 to F9-11), with d = t3:
            // the v19 page prints the middle branch as Fy[1.19 - 0.50 (b/t) sqrt(Fy/E)] (the AISC
            // 360-05 tee-flange formula, copied under the stem limits; it meets neither
            // neighbour, 0.77 at 0.84 r and 0.675 at 1.03 r against 1.0 and 0.6504) and says
            // "positive". The v24 manual (p. 3-60) prints Fy[2.55 - 1.84 (d/tw) sqrt(Fy/E)], S33
            // "the elastic section modulus" and "when the factored moment Mr is negative"; it is
            // the source for the branch, the modulus and the sign. The line through the two
            // printed ends is 2.5456 - 1.8400 (d/tw)/r.
            let dt = sec.t3 / sec.tw;
            let fcr_stem = if dt <= 0.84 * root_efy {
                fy
            } else if dt <= 1.03 * root_efy {
                (2.55 - 1.84 * dt / root_efy) * fy
            } else {
                0.69 * e / (dt * dt)
            };
            mn33_pos = mp_pos.min(ltb(1.0)).min(flb);
            mn33_pos_rev = Some(mp_pos.min(ltb(-1.0)).min(flb));
            mn33_neg = mp_neg.min(ltb(-1.0)).min(fcr_stem * s33);
            // F6: minor axis.
            mn22 = flb_linear(
                mp22,
                fy,
                s22,
                lambda,
                lp,
                lr,
                0.69 * e * s22 / (lambda * lambda),
            );
        }
        _ => {}
    }

    let phi = |mn: f64| if is_asd { mn / 1.67 } else { 0.90 * mn };
    let phi_mn33_pos = phi(mn33_pos);
    let phi_mn33_pos_stem_compressed = phi(mn33_pos_rev.unwrap_or(mn33_pos));
    let phi_mn33_neg = phi(mn33_neg);
    let phi_mn22 = phi(mn22);

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

    Ok(AiscStrengths {
        phi_pnc,
        phi_pnt,
        phi_mn33_pos,
        phi_mn33_pos_stem_compressed,
        phi_mn33_neg,
        phi_mn22,
        phi_vn2,
        compactness,
    })
}

/// EC3 section class of a cross section by the vendor's Tables 4-1 (bending only) and 4-2
/// (compression only), printed pp. 4-6 and 4-7 of the EC3 manual (D90). `class` is 1 to 4;
/// 5 means "not classifiable from the manual" (Tee stem in bending beyond 9 epsilon).
#[derive(Debug, Clone, PartialEq)]
pub struct Ec3Class {
    pub class: u8,
    /// The element that gives the class (the worst one), its width-to-thickness ratio and the
    /// limit of the class above its own (the Class 3 limit for Class 4).
    pub element: &'static str,
    pub ratio: f64,
    pub limit: f64,
}

impl Ec3Class {
    fn new(class: u8, element: &'static str, ratio: f64, limit: f64) -> Self {
        Self {
            class,
            element,
            ratio,
            limit,
        }
    }
}

/// Classifies one element: the limits are the Class 1, 2 and 3 values already multiplied by
/// epsilon (or epsilon squared); above the Class 3 limit the element is Class 4.
fn ec3_element(element: &'static str, ratio: f64, limits: [f64; 3]) -> Ec3Class {
    let class = if ratio <= limits[0] {
        1
    } else if ratio <= limits[1] {
        2
    } else if ratio <= limits[2] {
        3
    } else {
        4
    };
    Ec3Class::new(class, element, ratio, limits[2])
}

fn ec3_worst(parts: &[Ec3Class]) -> Ec3Class {
    let mut worst = parts[0].clone();
    for part in &parts[1..] {
        if part.class > worst.class {
            worst = part.clone();
        }
    }
    worst
}

/// Section class under EC3 for `sec` with `fy` in MPa (epsilon = sqrt(235/fy)): `bending`
/// selects Table 4-1, otherwise Table 4-2. The widths are the flat widths of the model (no
/// root radius; conservative against the vendor examples, which subtract r), for Box
/// `c = b - 3t` as in EN 1993-1-1 Table 5.2 for hollow sections. `None` when the shape is
/// not in the vendor tables. `ColdBox` and `ColdPipe` are read as Box and Pipe (their shear and
/// the rest of the design already treat them alike; the manual has no table of their own). Tees in bending: the stem limits need the stress ratio alpha and
/// k_sigma, which the manual does not give, so only Class 1 (c/t <= 9 epsilon) is classified
/// and anything above is class 5 (G72); the axial force is not used, so under tension this is
/// conservative against Table 4-3.
pub fn classify_ec3_section(sec: &FrameSection, fy_mpa: f64, bending: bool) -> Option<Ec3Class> {
    let eps = (235.0 / fy_mpa).sqrt();
    let flange = [9.0 * eps, 10.0 * eps, 14.0 * eps];
    let web_bend = [72.0 * eps, 83.0 * eps, 124.0 * eps];
    let web_comp = [33.0 * eps, 38.0 * eps, 42.0 * eps];
    let tube = [50.0 * eps * eps, 70.0 * eps * eps, 90.0 * eps * eps];
    let web = if bending { web_bend } else { web_comp };
    let tf = sec.tf.max(1e-9);
    let tfb = if sec.tfb > 0.0 { sec.tfb } else { tf };
    let tw = sec.tw.max(1e-9);
    match sec.prop_type {
        FramePropType::I => {
            let t2b = if sec.t2b > 0.0 { sec.t2b } else { sec.t2 };
            Some(ec3_worst(&[
                ec3_element("web", (sec.t3 - sec.tf - tfb).max(0.0) / tw, web),
                ec3_element("top flange", 0.5 * (sec.t2 - tw) / tf, flange),
                ec3_element("bottom flange", 0.5 * (t2b - tw) / tfb, flange),
            ]))
        }
        FramePropType::Channel | FramePropType::DbChannel => {
            let bf = if sec.prop_type == FramePropType::DbChannel {
                0.5 * (sec.t2 - sec.dis)
            } else {
                sec.t2
            };
            Some(ec3_worst(&[
                ec3_element("web", (sec.t3 - 2.0 * tf).max(0.0) / tw, web),
                ec3_element("flange", (bf - tw).max(0.0) / tf, flange),
            ]))
        }
        FramePropType::T => {
            let flange_part = ec3_element("flange", 0.5 * (sec.t2 - tw) / tf, flange);
            let stem_ratio = (sec.t3 - sec.tf).max(0.0) / tw;
            let stem = if bending {
                if stem_ratio <= flange[0] {
                    ec3_element("stem", stem_ratio, flange)
                } else {
                    Ec3Class::new(5, "stem", stem_ratio, flange[0])
                }
            } else {
                ec3_element("stem", stem_ratio, flange)
            };
            Some(ec3_worst(&[flange_part, stem]))
        }
        FramePropType::Angle | FramePropType::DblAngle => {
            // Table 4-2 for both loadings: Class 3 when h/t <= 15 eps and (b + h)/(2t) <= 11.5 eps,
            // else Class 4 (no Class 1 or 2).
            let t = if sec.tf > 0.0 { sec.tf } else { sec.tw }.max(1e-9);
            let b = if sec.prop_type == FramePropType::DblAngle {
                0.5 * (sec.t2 - sec.dis)
            } else {
                sec.t2
            };
            let h = sec.t3;
            let leg = ec3_element("leg h/t", h / t, [0.0, 0.0, 15.0 * eps]);
            let sum = ec3_element("legs (b+h)/2t", (b + h) / (2.0 * t), [0.0, 0.0, 11.5 * eps]);
            Some(ec3_worst(&[leg, sum]))
        }
        FramePropType::Box | FramePropType::ColdBox => Some(ec3_worst(&[
            ec3_element("flange", (sec.t2 - 3.0 * tf).max(0.0) / tf, web),
            ec3_element("web", (sec.t3 - 3.0 * tw).max(0.0) / tw, web),
        ])),
        FramePropType::Pipe | FramePropType::ColdPipe => {
            let t = if sec.tw > 0.0 { sec.tw } else { sec.tf }.max(1e-9);
            Some(ec3_element("wall d/t", sec.t3 / t, tube))
        }
        FramePropType::Rectangular | FramePropType::Circle => {
            Some(Ec3Class::new(2, "solid section", 0.0, f64::INFINITY))
        }
        FramePropType::General | FramePropType::SD => {
            Some(Ec3Class::new(3, "general section", 0.0, f64::INFINITY))
        }
        _ => None,
    }
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
    bending_class: u8,
) -> (f64, f64, f64, f64, f64, SectionCompactness) {
    let r = compute_ec3_member_resistances(sec, mat, l33, l22, l_ltb, k33, k22, cb, bending_class);
    (r.n_b_rd, r.n_t_rd, r.m_b_y_rd, r.m_c_z_rd, r.v_c_rd, r.compactness)
}

/// The EC3 member resistances of `compute_ec3_capacities`, with the per-axis values that the
/// 6.3.3(4) interaction needs (D89, G71): `chi_y`, `chi_z` and the slendernesses of each axis
/// (6.3.1, not only their minimum), the plastic resistance `N_pl,Rd = A fy`, and the section
/// moment resistance `M_c,y,Rd = W_y fy` next to the lateral-torsional `M_b,y,Rd = chi_LT W_y fy`.
/// `bending_class` selects the moduli (Class 1 and 2 plastic, else elastic). Not a different
/// calculation: `compute_ec3_capacities` returns these values.
#[derive(Debug, Clone)]
pub struct Ec3MemberResistances {
    pub n_pl_rd: f64,
    pub n_b_rd: f64,
    pub n_t_rd: f64,
    pub chi_y: f64,
    pub chi_z: f64,
    pub lambda_y: f64,
    pub lambda_z: f64,
    pub chi_lt: f64,
    pub m_c_y_rd: f64,
    pub m_b_y_rd: f64,
    pub m_c_z_rd: f64,
    pub v_c_rd: f64,
    pub compactness: SectionCompactness,
}

/// See [`Ec3MemberResistances`].
pub fn compute_ec3_member_resistances(
    sec: &FrameSection,
    mat: &Material,
    l33: f64,
    l22: f64,
    l_ltb: f64,
    k33: f64,
    k22: f64,
    cb: f64,
    bending_class: u8,
) -> Ec3MemberResistances {
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
    // D90: Class 1 and 2 use the plastic modulus, Class 3 the elastic modulus (manual p. 6-2).
    // `bending_class` 4 or 5 never reaches a moment check: the stations that carry a moment
    // are refused by the caller.
    let plastic = bending_class <= 2;
    let wy = if sec.z33 > 1e-9 && plastic {
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
    let wz = if sec.z22 > 1e-9 && plastic {
        sec.z22
    } else {
        sec.s22.max(1e-9)
    };
    let m_crd_22 = (wz * fy) / gamma_m0;

    // 5. Shear resistance Vpl,Rd = Av * (fy / sqrt(3)) / gamma_M0
    let v_crd = crate::shear::compute_ec3_major_shear(sec, mat).unwrap_or_else(|_| {
        let av = if sec.t3 > 0.0 && sec.tw > 0.0 {
            sec.t3 * sec.tw
        } else {
            0.5 * ag
        };
        (av * fy) / (3.0_f64.sqrt() * gamma_m0)
    });

    Ec3MemberResistances {
        n_pl_rd: n_trd,
        n_b_rd: n_brd,
        n_t_rd: n_trd,
        chi_y: chi_33,
        chi_z: chi_22,
        lambda_y: lambda_bar_33,
        lambda_z: lambda_bar_22,
        chi_lt,
        m_c_y_rd: (wy * fy) / gamma_m0,
        m_b_y_rd: m_brd_33,
        m_c_z_rd: m_crd_22,
        v_c_rd: v_crd,
        compactness,
    }
}

/// Shapes for which EN 1993-1-1 Annex B (Method 2) has interaction-factor formulas: I sections
/// (Table B.2, susceptible to torsional deformations, because the model holds no torsional
/// restraint) and rectangular hollow sections (Table B.1). Circular hollow sections use the
/// RHS column: the Annex has none of their own.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ec3AnnexBShape {
    I,
    Rhs,
}

/// The four interaction factors of EN 1993-1-1 6.3.3(4).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ec3InteractionFactors {
    pub kyy: f64,
    pub kyz: f64,
    pub kzy: f64,
    pub kzz: f64,
}

/// Interaction factors kyy, kyz, kzy, kzz of EN 1993-1-1 Annex B (Method 2), Tables B.1 and
/// B.2 (D89, G71). `lambda_y`, `lambda_z` are the relative slendernesses, `n_y = N_Ed/(chi_y N_Rk)`
/// and `n_z = N_Ed/(chi_z N_Rk)` (gamma_M1 = 1), `cmy`, `cmz`, `cmlt` the equivalent uniform
/// moment factors, `plastic` is true for Class 1 and 2 (plastic set), false for Class 3 (elastic
/// set). The note under Table B.1 (kzy may be 0 for uniaxial bending) is an allowance and is not used.
#[allow(clippy::too_many_arguments)]
pub fn ec3_annex_b_factors(
    lambda_y: f64,
    lambda_z: f64,
    n_y: f64,
    n_z: f64,
    cmy: f64,
    cmz: f64,
    cmlt: f64,
    plastic: bool,
    shape: Ec3AnnexBShape,
) -> Ec3InteractionFactors {
    let susceptible = shape == Ec3AnnexBShape::I;
    // CmLT is at least 0.4, so the denominator below is at least 0.15.
    let den = (cmlt - 0.25).max(1e-3);
    if plastic {
        let kyy = (cmy * (1.0 + (lambda_y - 0.2) * n_y)).min(cmy * (1.0 + 0.8 * n_y));
        let kzz = match shape {
            Ec3AnnexBShape::I => {
                (cmz * (1.0 + (2.0 * lambda_z - 0.6) * n_z)).min(cmz * (1.0 + 1.4 * n_z))
            }
            Ec3AnnexBShape::Rhs => {
                (cmz * (1.0 + (lambda_z - 0.2) * n_z)).min(cmz * (1.0 + 0.8 * n_z))
            }
        };
        let kyz = 0.6 * kzz;
        let kzy = if !susceptible {
            0.6 * kyy
        } else if lambda_z >= 0.4 {
            (1.0 - 0.1 * lambda_z * n_z / den).max(1.0 - 0.1 * n_z / den)
        } else {
            (0.6 + lambda_z).min(1.0 - 0.1 * lambda_z * n_z / den)
        };
        Ec3InteractionFactors { kyy, kyz, kzy, kzz }
    } else {
        let kyy = (cmy * (1.0 + 0.6 * lambda_y * n_y)).min(cmy * (1.0 + 0.6 * n_y));
        let kzz = (cmz * (1.0 + 0.6 * lambda_z * n_z)).min(cmz * (1.0 + 0.6 * n_z));
        let kyz = kzz;
        let kzy = if susceptible {
            (1.0 - 0.05 * lambda_z * n_z / den).max(1.0 - 0.05 * n_z / den)
        } else {
            0.8 * kyy
        };
        Ec3InteractionFactors { kyy, kyz, kzy, kzz }
    }
}

/// Equivalent uniform moment factor Cm of EN 1993-1-1 Table B.3 (Cmy, Cmz and CmLT, D89) from
/// one moment diagram given as `(position, moment)` points of the member (any positions, the
/// extreme two are the ends; the moment at mid-length is interpolated). A diagram on the chord
/// between its end moments (within 1e-9 of the largest moment) is linear: 0.6 + 0.4 psi >= 0.4.
/// Otherwise the rows of the table apply with M_h the end moment of larger magnitude and psi the
/// ratio of the other end to it. The span load is unknown at design: the larger of the uniform and
/// concentrated columns is taken (the safe side). A diagram that is zero everywhere gives 1.0.
/// The braced-point and sway conditions of the table notes are applied by the caller.
pub fn ec3_equivalent_moment_factor(points: &[(f64, f64)]) -> f64 {
    let mut pts: Vec<(f64, f64)> = points.to_vec();
    pts.sort_by(|a, b| a.0.total_cmp(&b.0));
    let m_max = pts.iter().fold(0.0_f64, |m, p| m.max(p.1.abs()));
    if pts.len() < 2 || m_max <= 0.0 {
        return 1.0;
    }
    let (x0, mi) = pts[0];
    let (x1, mj) = pts[pts.len() - 1];
    let span = x1 - x0;
    if span <= 0.0 {
        return 1.0;
    }
    let chord = |x: f64| mi + (mj - mi) * (x - x0) / span;
    if pts.iter().all(|p| (p.1 - chord(p.0)).abs() <= 1e-9 * m_max) {
        let psi = if mi.abs() >= mj.abs() { mj / mi } else { mi / mj };
        return (0.6 + 0.4 * psi).max(0.4);
    }
    // moment at mid-length
    let xm = x0 + 0.5 * span;
    let mut ms = pts[0].1;
    for w in pts.windows(2) {
        if w[0].0 <= xm && xm <= w[1].0 {
            let dx = w[1].0 - w[0].0;
            ms = if dx > 0.0 {
                w[0].1 + (w[1].1 - w[0].1) * (xm - w[0].0) / dx
            } else {
                w[0].1
            };
            break;
        }
    }
    let (mh, other) = if mi.abs() >= mj.abs() { (mi, mj) } else { (mj, mi) };
    let psi = if mh != 0.0 { other / mh } else { 0.0 };
    if mh.abs() >= ms.abs() {
        let a_s = ms / mh;
        if a_s >= 0.0 {
            (0.2 + 0.8 * a_s).max(0.4)
        } else if psi >= 0.0 {
            (0.1 - 0.8 * a_s).max(0.4).max((-0.8 * a_s).max(0.4))
        } else {
            (0.1 * (1.0 - psi) - 0.8 * a_s)
                .max(0.4)
                .max((0.2 * (-psi) - 0.8 * a_s).max(0.4))
        }
    } else {
        let a_h = mh / ms;
        if a_h >= 0.0 || psi >= 0.0 {
            (0.95 + 0.05 * a_h).max(0.90 + 0.10 * a_h)
        } else {
            (0.95 + 0.05 * a_h * (1.0 + 2.0 * psi)).max(0.90 + 0.10 * a_h * (1.0 + 2.0 * psi))
        }
    }
}

/// What the caller reads from the design overwrite for the moment factors (Table B.3 notes).
#[derive(Debug, Clone, Copy)]
struct Ec3CmContext {
    /// Unbraced length ratios of the major, minor and lateral-torsional direction equal 1.
    major_full: bool,
    minor_full: bool,
    ltb_full: bool,
    sway: bool,
}

/// One EN 1993-1-1 6.3.3(4) member check of a design state (equations 6.61 and 6.62).
struct Ec3MemberCheck {
    ratio: f64,
    eq_name: &'static str,
    pr: f64,
    mr_major: f64,
    mr_minor: f64,
    /// Index of the station with the largest moment term of the governing equation.
    station_index: usize,
    k: Ec3InteractionFactors,
    cm: (f64, f64, f64),
}

/// EN 1993-1-1 6.3.3(4) with the Annex B factors for one state (D89, G71). `class` is the worse
/// of the compression and bending classes (1 to 3); `res` holds the resistances for that class.
fn ec3_member_check(
    stations: &[FrameStationResult],
    res: &Ec3MemberResistances,
    class: u8,
    shape: Ec3AnnexBShape,
    pipe: bool,
    ctx: Ec3CmContext,
) -> Ec3MemberCheck {
    let n_ed = stations.iter().fold(0.0_f64, |n, s| n.max(-s.p));
    let my_ed = stations.iter().fold(0.0_f64, |m, s| m.max(s.m3.abs()));
    let mz_ed = stations.iter().fold(0.0_f64, |m, s| m.max(s.m2.abs()));
    let n_y = n_ed / (res.chi_y * res.n_pl_rd).max(1e-12);
    let n_z = n_ed / (res.chi_z * res.n_pl_rd).max(1e-12);
    // moment diagrams of the state, by distance along the member
    let diagram = |f: &dyn Fn(&FrameStationResult) -> f64| -> Vec<(f64, f64)> {
        stations.iter().map(|s| (s.station, f(s))).collect()
    };
    let mut cmy = ec3_equivalent_moment_factor(&diagram(&|s| s.m3));
    let mut cmz = ec3_equivalent_moment_factor(&diagram(&|s| s.m2));
    let mut cmlt = ec3_equivalent_moment_factor(&diagram(&|s| s.m3));
    // Table B.3 notes: the table wants the diagram between braced points; the model holds length
    // ratios, not brace positions, so an unbraced length other than the member length gives the
    // largest value of the table, 1.0. Sway buckling mode: 0.9 for Cmy and Cmz.
    if !ctx.major_full {
        cmy = 1.0;
    } else if ctx.sway {
        cmy = 0.9;
    }
    if !ctx.minor_full {
        cmz = 1.0;
    } else if ctx.sway {
        cmz = 0.9;
    }
    if !ctx.ltb_full {
        cmlt = 1.0;
    }
    let plastic = class <= 2;
    let k = ec3_annex_b_factors(
        res.lambda_y,
        res.lambda_z,
        n_y,
        n_z,
        cmy,
        cmz,
        cmlt,
        plastic,
        shape,
    );
    let m_y_res = res.m_b_y_rd.max(1e-12);
    let m_z_res = res.m_c_z_rd.max(1e-12);
    let terms = |ky: f64, kz: f64, m3: f64, m2: f64| -> (f64, f64) {
        (ky * m3.abs() / m_y_res, kz * m2.abs() / m_z_res)
    };
    let combine = |a: f64, b: f64| if pipe { (a * a + b * b).sqrt() } else { a + b };
    let (a61, b61) = terms(k.kyy, k.kyz, my_ed, mz_ed);
    let (a62, b62) = terms(k.kzy, k.kzz, my_ed, mz_ed);
    let eq61 = n_y + combine(a61, b61);
    let eq62 = n_z + combine(a62, b62);
    let first = eq61 >= eq62;
    let (ky, kz) = if first { (k.kyy, k.kyz) } else { (k.kzy, k.kzz) };
    let mut station_index = 0;
    let mut best = -1.0;
    for (i, s) in stations.iter().enumerate() {
        let (a, b) = terms(ky, kz, s.m3, s.m2);
        let t = combine(a, b);
        if t > best + 1e-15 {
            best = t;
            station_index = i;
        }
    }
    if first {
        Ec3MemberCheck {
            ratio: eq61,
            eq_name: "EC3-6.3.3(4) eq 6.61",
            pr: n_y,
            mr_major: a61,
            mr_minor: b61,
            station_index,
            k,
            cm: (cmy, cmz, cmlt),
        }
    } else {
        Ec3MemberCheck {
            ratio: eq62,
            eq_name: "EC3-6.3.3(4) eq 6.62",
            pr: n_z,
            mr_major: a62,
            mr_minor: b62,
            station_index,
            k,
            cm: (cmy, cmz, cmlt),
        }
    }
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

/// The design states of one combination at one frame: each state is a complete list of
/// station forces, a state of its own with its own station list (so its own moment diagram).
/// A single-valued combination or case has one state.
pub(crate) type FrameStates = Vec<Vec<FrameStationResult>>;

/// All design combinations (or analysis cases if no combos are defined) as
/// `(combo_or_case_name, [(frame_name, states)])`.
pub(crate) type ComboForces = Vec<(String, Vec<(String, FrameStates)>)>;

/// Refuses a design run when a combination of the model has no complete results (D78): a
/// combination whose contributor has no results is neither designed from the contributors
/// that exist nor skipped, because the envelope of the remaining combinations would then
/// claim a check that was not made. Names the combination and the reason (its case).
pub(crate) fn refuse_unavailable_combos(
    model: &SapModel,
    discipline: &'static str,
) -> Result<(), crate::DesignError> {
    // D79: a name that is both a case and a combination would let the result of the case
    // pass for that of the combination (the check below skips a combination that has a
    // stored result), so design refuses it by name before it looks at any result.
    if let Some(name) = sap_model::results::case_combo_name_collisions(model).first() {
        return Err(crate::DesignError::UnavailableCombination {
            discipline,
            combo: name.clone(),
            reason: sap_model::results::case_combo_collision_message(model).unwrap_or_default(),
        });
    }
    if model.case_results.is_empty() && model.unavailable_results.is_empty() {
        return Ok(());
    }
    // G16: design never uses the states of a case that is not finished as if they were its
    // response, and never skips a case that could not start. Without design combinations
    // design takes every case with frame forces (below), so those are refused by name;
    // with combinations, a contributor that is not finished makes the combination
    // unavailable (`combo_gap`) and is refused through it.
    if model.load_combos.is_empty() {
        for (name, case) in &model.case_results {
            if let sap_model::CaseStatus::NotFinished { reason, .. } = &case.status {
                return Err(crate::DesignError::UnfinishedCase {
                    discipline,
                    case: name.clone(),
                    reason: reason.clone(),
                });
            }
        }
        if let Some((name, reason)) = sap_model::results::could_not_start_cases(model).first() {
            return Err(crate::DesignError::UnfinishedCase {
                discipline,
                case: name.clone(),
                reason: reason.clone(),
            });
        }
    }
    for (name, combo) in &model.load_combos {
        if model.case_results.contains_key(name) {
            continue;
        }
        let reason = sap_model::results::unavailable_reason(model, name)
            .map(str::to_string)
            .or_else(|| sap_model::results::combo_gap(model, combo));
        if let Some(reason) = reason {
            return Err(crate::DesignError::UnavailableCombination {
                discipline,
                combo: name.clone(),
                reason,
            });
        }
    }
    Ok(())
}

/// Resolves all design load combinations (or analysis cases if no combos are defined) into
/// the states of every frame. The rows of a multi-valued combination are NOT stations of
/// one member: its maximum row and its minimum row are two states (CSI Analysis Reference
/// pp. 351-352), each a design entry with its own station list.
pub(crate) fn collect_design_combo_forces(model: &SapModel) -> ComboForces {
    let mut out = Vec::new();

    if !model.load_combos.is_empty() {
        for (combo_name, combo) in &model.load_combos {
            // If case_results already has pre-computed forces for this combo, use them directly
            if let Some(cr) = model.case_results.get(combo_name) {
                if !cr.frame_forces.is_empty() && cr.combo_min.is_some() {
                    // A multi-valued combination (Analysis Reference p. 351): the design sees
                    // both senses, the maximum row and the minimum row, as two states of
                    // every frame. An envelope is rebuilt from its contributors instead:
                    // every contributing state stays a state of its own with the forces of
                    // that state together (the permutations of the maxima and minima are
                    // formed from them by `design_entries`).
                    let lo = cr.combo_min.as_deref();
                    let frames_vec = cr
                        .frame_forces
                        .iter()
                        .map(|(k, v)| {
                            if combo.combo_type == ComboType::Envelope {
                                if let Some(st) = synthesize_combo_states(model, combo, k, 0)
                                    .filter(|st| !st.is_empty())
                                {
                                    return (k.clone(), st);
                                }
                            }
                            let mut states = vec![v.clone()];
                            if let Some(l) = lo.and_then(|l| l.frame_forces.get(k)) {
                                states.push(l.clone());
                            }
                            (k.clone(), states)
                        })
                        .collect();
                    out.push((combo_name.clone(), frames_vec));
                    continue;
                }
                if !cr.frame_forces.is_empty() {
                    let frames_vec = cr
                        .frame_forces
                        .iter()
                        .map(|(k, v)| (k.clone(), vec![v.clone()]))
                        .collect();
                    out.push((combo_name.clone(), frames_vec));
                    continue;
                }
            }
            // Otherwise synthesize from constituent cases
            let mut frames_vec = Vec::new();
            for frame_name in model.frames.keys() {
                if let Some(states) = synthesize_combo_states(model, combo, frame_name, 0) {
                    if states.iter().any(|s| !s.is_empty()) {
                        frames_vec.push((frame_name.clone(), states));
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
            // G16: the states of a case that is not finished are never designed (a design
            // run with no combinations refuses such a case by name before it gets here).
            if !cr.status.is_finished() {
                continue;
            }
            if !cr.frame_forces.is_empty() {
                // A response-spectrum, moving-load, steady-state or PSD case is a pair
                // (+v, -v) (p. 351): both senses are two states.
                let both_senses = sap_model::results::is_range_case(model, case_name);
                let frames_vec = cr
                    .frame_forces
                    .iter()
                    .map(|(k, v)| {
                        if both_senses {
                            let states = [1.0, -1.0]
                                .map(|sign| {
                                    v.iter()
                                        .map(|s| {
                                            station_from_values(
                                                s,
                                                station_values(s).map(|x| sign * x.abs()),
                                            )
                                        })
                                        .collect()
                                })
                                .to_vec();
                            (k.clone(), states)
                        } else {
                            (k.clone(), vec![v.clone()])
                        }
                    })
                    .collect();
                out.push((case_name.clone(), frames_vec));
            }
        }
    }

    out
}

/// Bits of the forces that a design check combines, for [`design_entries`].
pub(crate) mod interacting {
    pub const P: u8 = 1;
    pub const V2: u8 = 2;
    pub const V3: u8 = 4;
    pub const T: u8 = 8;
    pub const M2: u8 = 16;
    pub const M3: u8 = 32;
}

fn field_of(s: &FrameStationResult, bit: u8) -> f64 {
    match bit {
        interacting::P => s.p,
        interacting::V2 => s.v2,
        interacting::V3 => s.v3,
        interacting::T => s.t,
        interacting::M2 => s.m2,
        _ => s.m3,
    }
}

fn set_field(s: &mut FrameStationResult, bit: u8, v: f64) {
    match bit {
        interacting::P => s.p = v,
        interacting::V2 => s.v2 = v,
        interacting::V3 => s.v3 = v,
        interacting::T => s.t = v,
        interacting::M2 => s.m2 = v,
        _ => s.m3 = v,
    }
}

/// The design entries (each its own station list, so its own moment diagram) of one frame
/// of one combination, given its states.
///
/// A single state is its own entry, unchanged (bitwise the numbers of a single-valued
/// combination or a plain case). A multi-valued combination (two or more states) gives
/// first its states as they are, then the sub-combinations of the maxima and minima of the
/// interacting forces.
///
/// Rule (CSI Steel Frame Design AISC 360-10, section 2.4: "For multi-valued design
/// combinations, such as those involving response spectrum, time history, moving loads and
/// envelopes, where any correspondence between forces is lost, the program automatically
/// produces sub-combinations using the maxima/minima values of the interacting forces."
/// Concrete Frame Design ACI 318-14, section 2.1: "...multiple sub combinations using
/// maxima/minima permutations of interacting quantities"; both say that the minima of a
/// response spectrum case are the negatives of the maxima, which the pair of states
/// (+v, -v) of the combination already carries, Analysis Reference p. 351): at every
/// station each force in `mask` is taken at its maximum or at its minimum over the states
/// at that station, every choice of the 2^k, one design entry per choice.
///
/// `mask` names the forces the check combines (see `interacting`). A force that is not in
/// the mask (a shear that is checked alone, a torsion the check ignores) is not permuted:
/// it is taken from the state that supplies the chosen M3, so each entry stays a
/// consistent state in all that the permutation does not touch, and the largest value of
/// such a force over the states is reached by the state entries themselves.
///
/// Choice of the project (the manuals say nothing on it): an entry is one choice at ALL
/// stations, so its moment diagram, and with it $C_b$ and the largest moment of the
/// unbraced length, is the diagram of the maximum row (M3 at its maximum) or of the minimum
/// row (M3 at its minimum), never a mixture of the two. The state entries keep the diagrams
/// of the states themselves. Entries equal to an earlier entry are dropped. When the states
/// do not have the same stations, only the states are designed.
pub(crate) fn design_entries(
    states: &[Vec<FrameStationResult>],
    mask: u8,
) -> Vec<std::borrow::Cow<'_, [FrameStationResult]>> {
    use std::borrow::Cow;
    let mut entries: Vec<Cow<'_, [FrameStationResult]>> =
        states.iter().map(|s| Cow::Borrowed(s.as_slice())).collect();
    let n = states.first().map_or(0, Vec::len);
    if states.len() < 2 || n == 0 || states.iter().any(|s| s.len() != n) {
        return entries;
    }
    let bits: Vec<u8> = [
        interacting::P,
        interacting::M2,
        interacting::M3,
        interacting::V2,
        interacting::V3,
        interacting::T,
    ]
    .into_iter()
    .filter(|b| mask & b != 0 || *b == interacting::M3)
    .collect();
    let m3_slot = bits.iter().position(|b| *b == interacting::M3).unwrap_or(0);
    // Per station and force: (index of the state with the maximum, with the minimum).
    let mut ext: Vec<Vec<(usize, usize)>> = Vec::with_capacity(n);
    for i in 0..n {
        let mut row = Vec::with_capacity(bits.len());
        for &b in &bits {
            let (mut hi, mut lo) = (0, 0);
            for (k, st) in states.iter().enumerate() {
                let v = field_of(&st[i], b);
                if v > field_of(&states[hi][i], b) {
                    hi = k;
                }
                if v < field_of(&states[lo][i], b) {
                    lo = k;
                }
            }
            row.push((hi, lo));
        }
        ext.push(row);
    }
    for choice in 0..(1usize << bits.len()) {
        let pick = |slot: usize, i: usize| {
            let (hi, lo) = ext[i][slot];
            if (choice >> slot) & 1 == 0 {
                hi
            } else {
                lo
            }
        };
        let list: Vec<FrameStationResult> = (0..n)
            .map(|i| {
                let mut st = states[pick(m3_slot, i)][i].clone();
                for (slot, &b) in bits.iter().enumerate() {
                    set_field(&mut st, b, field_of(&states[pick(slot, i)][i], b));
                }
                st
            })
            .collect();
        if !entries.iter().any(|e| **e == list[..]) {
            entries.push(Cow::Owned(list));
        }
    }
    entries
}

/// The design states of one combination for one frame: each state is a complete list of
/// station forces. A single-valued combination has one state; a multi-valued one has the
/// maximum and the minimum sense of the manual (Analysis Reference pp. 351-352), and an
/// envelope keeps every contributing state of every contributor.
type DesignStates = Vec<Vec<FrameStationResult>>;

fn station_values(s: &FrameStationResult) -> [f64; 9] {
    [
        s.p,
        s.v2,
        s.v3,
        s.t,
        s.m2,
        s.m3,
        s.local_disp[0],
        s.local_disp[1],
        s.local_disp[2],
    ]
}

fn station_from_values(base: &FrameStationResult, v: [f64; 9]) -> FrameStationResult {
    FrameStationResult {
        station: base.station,
        rel_dist: base.rel_dist,
        p: v[0],
        v2: v[1],
        v3: v[2],
        t: v[3],
        m2: v[4],
        m3: v[5],
        local_disp: [v[6], v[7], v[8]],
    }
}

/// The states a case or combination contributes at `frame_name`: a stored result gives its
/// row, or its maximum and minimum rows when it is a multi-valued combination, or `(+v, -v)`
/// for a response-spectrum, moving-load, steady-state or PSD case (p. 351); a combination
/// without stored results is synthesized from its own contributors.
fn constituent_states(
    model: &SapModel,
    name: &str,
    frame_name: &str,
    depth: usize,
) -> Option<DesignStates> {
    if let Some(cr) = model.case_results.get(name) {
        if let Some(st) = cr.frame_forces.get(frame_name).filter(|st| !st.is_empty()) {
            if let Some(lo) = cr
                .combo_min
                .as_deref()
                .and_then(|m| m.frame_forces.get(frame_name))
                .filter(|l| l.len() == st.len())
            {
                return Some(vec![st.clone(), lo.clone()]);
            }
            if sap_model::results::is_range_case(model, name) {
                let signed = |sign: f64| -> Vec<FrameStationResult> {
                    st.iter()
                        .map(|s| station_from_values(s, station_values(s).map(|v| sign * v.abs())))
                        .collect()
                };
                return Some(vec![signed(1.0), signed(-1.0)]);
            }
            return Some(vec![st.clone()]);
        }
    }
    let sub = model.load_combos.get(name)?;
    synthesize_combo_states(model, sub, frame_name, depth + 1)
}

fn synthesize_combo_states(
    model: &SapModel,
    combo: &LoadCombo,
    frame_name: &str,
    depth: usize,
) -> Option<DesignStates> {
    if depth > 8 {
        return None;
    }
    let mut constituents: Vec<(f64, DesignStates)> = Vec::new();
    for (ref_name, sf) in &combo.cases {
        if let Some(states) = constituent_states(model, ref_name, frame_name, depth) {
            if !states.is_empty() {
                constituents.push((*sf, states));
            }
        }
    }
    if constituents.is_empty() {
        return None;
    }

    if combo.combo_type == ComboType::Envelope {
        // An envelope evaluates each constituent state independently (not summed), which
        // keeps the correlation between the forces of one state; the rows of a
        // multi-valued constituent are all candidates.
        let mut env = DesignStates::new();
        for (sf, states) in &constituents {
            for state in states {
                env.push(
                    state
                        .iter()
                        .map(|s| station_from_values(s, station_values(s).map(|v| sf * v)))
                        .collect(),
                );
            }
        }
        return Some(env);
    }

    let num_stations = constituents[0].1[0].len();
    // Every contributor is a pair (maximum, minimum) per quantity: the largest and the
    // smallest value over its states (a single state gives the pair (v, v)).
    let pair_at = |states: &DesignStates, idx: usize, k: usize| -> (f64, f64) {
        let mut hi = f64::NEG_INFINITY;
        let mut lo = f64::INFINITY;
        for state in states {
            let item = state.get(idx).unwrap_or_else(|| state.last().unwrap());
            let v = station_values(item)[k];
            hi = hi.max(v);
            lo = lo.min(v);
        }
        (hi, lo)
    };
    let all_single = constituents.iter().all(|(_, states)| states.len() == 1);
    let single_valued = all_single && combo.combo_type == ComboType::LinearAdd;
    let senses: &[bool] = if single_valued { &[true] } else { &[true, false] };
    let mut out = DesignStates::new();
    for &maximum in senses {
        let mut list = Vec::with_capacity(num_stations);
        for idx in 0..num_stations {
            let base = &constituents[0].1[0][idx];
            let mut values = [0.0; 9];
            for (k, slot) in values.iter_mut().enumerate() {
                let mut acc = 0.0;
                for (n, (sf, states)) in constituents.iter().enumerate() {
                    // A pair read off a state list built from a response-spectrum case
                    // is already (+v, -v), so `range` is false here.
                    acc = sap_model::results::combo_step(
                        combo.combo_type,
                        maximum,
                        acc,
                        pair_at(states, idx, k),
                        false,
                        *sf,
                        n == 0,
                    );
                }
                *slot = if combo.combo_type == ComboType::Srss {
                    let r = acc.max(0.0).sqrt();
                    if maximum {
                        r
                    } else {
                        -r
                    }
                } else {
                    acc
                };
            }
            list.push(station_from_values(base, values));
        }
        out.push(list);
    }
    Some(out)
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
    refuse_unavailable_combos(model, "steel")?;
    let combo_forces = collect_design_combo_forces(model);
    validate_demands(model, &combo_forces, "steel", true, |model, frame_name| {
        let Some(frame) = model.frames.get(frame_name) else {
            return false;
        };
        // D93: cable objects are not frame members; the vendor design manuals design frame
        // objects only.
        if frame.cable_params.is_some() {
            return false;
        }
        let Some(section) = model.frame_sections.get(&frame.section) else {
            return false;
        };
        if crate::auto_select::is_auto_list_section(section) {
            return true;
        }
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
            if b_outer <= 3.0 * sec.tw || h_outer <= 3.0 * sec.tf {
                return Err(DesignError::UnsupportedClosedSectionTorsion {
                    frame: sec.name.clone(),
                });
            }
            let c =
                2.0 * (b_outer - sec.tw) * (h_outer - sec.tf) * t - 4.5 * (4.0 - PI) * t.powi(3);
            if c <= 0.0 {
                return Err(DesignError::UnsupportedClosedSectionTorsion {
                    frame: sec.name.clone(),
                });
            }
            // Flat width is bounded by webs and is a flange element (tf); flat
            // depth is bounded by flanges and is a web element (tw).
            let h_over_t =
                ((b_outer - 3.0 * sec.tw) / sec.tf).max((h_outer - 3.0 * sec.tf) / sec.tw);
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
    combo_forces: ComboForces,
) -> Result<(), DesignError> {
    let _ = compute_ec3_rectangular_minor_shear;

    // Frames assigned an Auto Select list are designed from the list; the
    // neighbours' sections for the effective length factors are then the
    // sections the analysis used. Without lists the model itself is used.
    let has_auto_list = model.frames.values().any(|f| {
        model
            .frame_sections
            .get(&f.section)
            .is_some_and(crate::auto_select::is_auto_list_section)
    });
    let k_view;
    let k_model: &SapModel = if has_auto_list {
        k_view = crate::auto_select::analysis_view(model)?;
        &k_view
    } else {
        model
    };
    let dc_limit = model.steel_design_prefs.dc_ratio_limit;

    let mut results = Vec::new();
    let mut auto_choices: Vec<(String, String)> = Vec::new();

    for (frame_name, frame) in &model.frames {
        // D93: cables are not designed (see `try_run_steel_design_with_overwrites`).
        if frame.cable_params.is_some() {
            continue;
        }
        let Some(sec) = model.frame_sections.get(&frame.section) else {
            continue;
        };
        if crate::auto_select::is_auto_list_section(sec) {
            if let Some((res, chosen)) = crate::auto_select::design_frame_from_list(
                model,
                k_model,
                frame_name,
                frame,
                &sec.name,
                overwrites,
                code,
                &combo_forces,
                dc_limit,
            )? {
                results.push((frame_name.clone(), res));
                auto_choices.push((frame_name.clone(), chosen));
            }
            continue;
        }
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
        if let Some(res) = design_frame_with_section(
            model,
            k_model,
            frame_name,
            frame,
            sec,
            mat,
            overwrites,
            code,
            &combo_forces,
        )? {
            results.push((frame_name.clone(), res));
        }
    }

    model.steel_design_results.clear();
    for (k, v) in results {
        model.steel_design_results.insert(k, v);
    }
    for (frame_name, chosen) in auto_choices {
        if let Some(frame) = model.frames.get_mut(&frame_name) {
            let state = frame.auto_select.get_or_insert_with(Default::default);
            state.design = chosen;
        }
    }
    Ok(())
}

/// D128, D129: the AISC 360-10 limit states that are refused instead of designed (all
/// `AiscLimitStateNotBuilt`, AISC only). One refused member stops the whole run and leaves no
/// steel results, and Auto Select stops at a refused candidate (D127): a model with a single
/// angle in compression is no longer designed at all. Demand gates are dimensionless, so a
/// roundoff value neither refuses nor designs: compression `-P > 1e-9 Fy A`, `|M3| > 1e-9 Fy S33`,
/// `|M2| > 1e-9 Fy S22`; every station of every state of the frame is scanned (as D88 does).
fn refuse_aisc_limit_states_not_built(
    frame_name: &str,
    sec_in: &FrameSection,
    mat: &Material,
    combo_forces: &[(String, Vec<(String, FrameStates)>)],
) -> Result<(), DesignError> {
    use FramePropType as P;
    let sec_leg = angle_leg_thickness(sec_in);
    let sec: &FrameSection = &sec_leg;
    if !matches!(
        sec.prop_type,
        P::Angle | P::Box | P::ColdBox | P::T | P::DblAngle | P::Channel | P::DbChannel
    ) {
        return Ok(());
    }
    let fy = mat.fy.max(1e-6);
    let e = mat.e.max(1.0);
    let (mut compressed, mut m3_on, mut m2_on) = (false, false, false);
    for (_, frames) in combo_forces {
        for (name, states) in frames {
            if name != frame_name {
                continue;
            }
            for st in states.iter().flatten() {
                compressed |= st.p < 0.0 && -st.p > 1e-9 * fy * sec.area.max(1e-9);
                m3_on |= st.m3.abs() > 1e-9 * fy * sec.s33.max(1e-9);
                m2_on |= st.m2.abs() > 1e-9 * fy * sec.s22.max(1e-9);
            }
        }
    }
    let refuse = |what: &str| DesignError::AiscLimitStateNotBuilt {
        frame: frame_name.to_string(),
        section: sec.name.clone(),
        kind: format!("{:?}", sec.prop_type),
        what: what.to_string(),
    };
    let r = (e / fy).sqrt();
    match sec.prop_type {
        P::Angle if compressed || m3_on || m2_on => Err(refuse(
            "single angle under compression or bending: the vendor designs it about its principal \
             axes (manual 3.5, p. 3-22; r_z p. 3-24; E4-5/E4-6 p. 3-27; F10 pp. 3-61 to 3-63; H2 \
             p. 3-79); principal properties, shear centre and that path are not implemented (G81)",
        )),
        P::Box | P::ColdBox => {
            let slender_major = (sec.t2 - 3.0 * sec.tw) / sec.tf.max(1e-9) > 1.40 * r;
            let slender_minor = (sec.t3 - 3.0 * sec.tf) / sec.tw.max(1e-9) > 1.40 * r;
            if (m3_on && slender_major) || (m2_on && slender_minor) {
                Err(refuse(
                    "F7 slender flange: the manual (p. 3-57) names the effective section modulus \
                     Seff but not how it is computed (G82)",
                ))
            } else {
                Ok(())
            }
        }
        P::T | P::DblAngle | P::Channel | P::DbChannel => {
            if !(compressed || m3_on || m2_on) {
                return Ok(());
            }
            if !open_shape_dimensions_ok(sec) {
                return Err(refuse("dimensions do not define the section"));
            }
            if (compressed || m3_on) && sec.torsional_j <= 0.0 {
                return Err(refuse("torsional constant J = 0: E4 and F9 need it"));
            }
            if m3_on
                && matches!(sec.prop_type, P::Channel | P::DbChannel)
                && (sec.t3 - 2.0 * sec.tf) / sec.tw > 3.76 * r
            {
                return Err(refuse(
                    "major-axis flexure with a noncompact or slender web: the F4/F5 forms of the \
                     manual are not built (G83)",
                ));
            }
            Ok(())
        }
        _ => Ok(()),
    }
}

/// Design of one frame with an explicitly given section: the demand/capacity
/// evaluation of every design combination at every station. Returns the
/// governing result, or `None` when no combination has forces for the frame.
/// `k_model` supplies the neighbouring sections for the effective length
/// factors; it is `model` itself unless Auto Select lists are in play.
#[allow(clippy::too_many_arguments)]
pub(crate) fn design_frame_with_section(
    model: &SapModel,
    k_model: &SapModel,
    frame_name: &str,
    frame: &FrameObj,
    sec: &FrameSection,
    mat: &Material,
    overwrites: &std::collections::HashMap<String, SteelMemberOverwrite>,
    code: SteelCode,
    combo_forces: &[(String, Vec<(String, FrameStates)>)],
) -> Result<Option<SteelDesignResult>, DesignError> {
    let is_asd = code == SteelCode::Aisc360Asd;
    let has_ec3_v3 = code == SteelCode::Eurocode3
        && combo_forces.iter().any(|(_, frames)| {
            frames.iter().any(|(name, states)| {
                name == frame_name
                    && states.iter().flatten().any(|st| st.v3.abs() > 1e-9)
            })
        });
    let ow = overwrites.get(frame_name).or(frame.steel_overwrite.as_ref());
    // A stored overwrite can come from a file; a zero or negative length
    // ratio or factor must not reach the capacity formulas.
    if let Some(o) = ow {
        for (item, value) in [
            ("L33 ratio", o.l_ratio_major),
            ("L22 ratio", o.l_ratio_minor),
            ("LTB ratio", o.l_ratio_ltb),
            ("K33", o.k_major),
            ("K22", o.k_minor),
            ("Cb", o.cb),
        ] {
            if let Some(value) = value.filter(|v| !v.is_finite() || *v <= 0.0) {
                return Err(DesignError::InvalidSteelOverwrite {
                    frame: frame_name.to_string(),
                    item,
                    value: value.to_string(),
                });
            }
        }
    }
    // D88: I sections with unequal flanges are refused. The code takes the warping constant
    // Cw = 0.25 Iy h0^2 of a doubly symmetric section (AISC E4-4, F2-7, EC3 Iw) and pairs the
    // wider flange with the top thickness; F4 (singly symmetric I, G70) is not built. A zero
    // bottom flange (t2b or tfb = 0) means "same as the top", as the rest of the code reads it.
    if sec.prop_type == FramePropType::I {
        let differs =
            |top: f64, bottom: f64| bottom > 0.0 && (bottom - top).abs() > 1e-9 * top.abs();
        if differs(sec.t2, sec.t2b) || differs(sec.tf, sec.tfb) {
            let has_m3 = combo_forces.iter().any(|(_, frames)| {
                frames.iter().any(|(name, states)| {
                    name == frame_name && states.iter().flatten().any(|st| st.m3.abs() > 1e-9)
                })
            });
            if code != SteelCode::Eurocode3 {
                return Err(DesignError::UnequalFlangeISection {
                    frame: frame_name.to_string(),
                    section: sec.name.clone(),
                    what: "F4 / Cw for unequal flanges",
                });
            }
            if has_m3 {
                return Err(DesignError::UnequalFlangeISection {
                    frame: frame_name.to_string(),
                    section: sec.name.clone(),
                    what: "Iw for unequal flanges",
                });
            }
        }
    }
    // D128, D129 (G81, G82, G83): the AISC limit states of the open shapes and of the box that
    // are not built are refused by name where the demand needs them.
    if code != SteelCode::Eurocode3 {
        refuse_aisc_limit_states_not_built(frame_name, sec, mat, combo_forces)?;
    }
    let l_clear = frame_clear_length(model, frame).max(1e-6);
    let l33 = ow.and_then(|o| o.l_ratio_major).unwrap_or(1.0) * l_clear;
    let l22 = ow.and_then(|o| o.l_ratio_minor).unwrap_or(1.0) * l_clear;
    let l_ltb = ow.and_then(|o| o.l_ratio_ltb).unwrap_or(1.0) * l_clear;
    let (k33, k22) = compute_member_k_factors(k_model, frame, ow);

    // CSI AISC 360-10 section 3.5.5 ignores torsion for open shapes, and
    // evaluates H3.1/H3.2 interaction for closed sections (Box and Pipe).
    let has_closed_torsion = matches!(
        sec.prop_type,
        FramePropType::Box
            | FramePropType::ColdBox
            | FramePropType::Pipe
            | FramePropType::ColdPipe
    ) && combo_forces.iter().any(|(_, frames)| {
        frames.iter().any(|(name, states)| {
            name == frame_name && states.iter().flatten().any(|station| station.t.abs() > 1e-9)
        })
    });
    let phi_tn = if has_closed_torsion {
        if code == SteelCode::Eurocode3 {
            return Err(DesignError::UnsupportedClosedSectionTorsion {
                frame: frame_name.to_string(),
            });
        }
        Some(
            compute_aisc_torsional_capacity(sec, mat, l_ltb, is_asd).map_err(|_| {
                DesignError::UnsupportedClosedSectionTorsion {
                    frame: frame_name.to_string(),
                }
            })?,
        )
    } else {
        None
    };

    let phi_vn3 = match code {
        SteelCode::Eurocode3 if has_ec3_v3 => compute_ec3_minor_shear(sec, mat)?,
        SteelCode::Eurocode3 => compute_ec3_minor_shear(sec, mat).unwrap_or(0.0),
        _ => compute_aisc_minor_shear(sec, mat, is_asd)?,
    };

    // The forces this check combines, whose maxima and minima are permuted for a
    // multi-valued combination (CSI Steel Frame Design AISC 360-10, section 2.4, quoted at
    // `design_entries`). Axial-flexure interaction (AISC H1-1a/b, EC3 6.2.1): P, M2, M3.
    // The shear checks (AISC G, EC3 6.2.6) use one shear alone: the state entries reach its
    // largest absolute value, so it is not permuted. The Eurocode shear-moment reduction
    // (6.2.8) combines V2 with M3 and V3 with M2, and the torsion-shear-flexure-axial check
    // of closed sections (AISC H3-6) combines T, V2, V3, P, M2 and M3: those forces are
    // permuted where the check is active.
    let mut interacting_forces = interacting::P | interacting::M2 | interacting::M3;
    if code == SteelCode::Eurocode3 {
        interacting_forces |= interacting::V2 | interacting::V3;
    }
    if phi_tn.is_some() {
        interacting_forces |= interacting::V2 | interacting::V3 | interacting::T;
    }
    // D90: EC3 section class by the vendor's Tables 4-1 (bending) and 4-2 (compression).
    let ec3_classes = if code == SteelCode::Eurocode3 {
        let fy_mpa = mat.fy * model.present_units.stress_factor_to(sap_units::Units::NMmC);
        let classify = |bending: bool| {
            classify_ec3_section(sec, fy_mpa, bending).ok_or_else(|| {
                DesignError::UnsupportedEc3Classification {
                    frame: frame_name.to_string(),
                    section: sec.name.clone(),
                    kind: format!("{:?}", sec.prop_type),
                }
            })
        };
        Some((classify(false)?, classify(true)?))
    } else {
        None
    };
    // EC3 6.2.8(5) as the manual prints it (p. 6-2): the reduced plastic resistance moment
    // (Wpl - n rho Aw^2/(4 tw)) fy, limited by the resistance M_c,Rd of the section's class. For a
    // Class 3 section M_c,Rd is the elastic one, so the reduction only acts where the reduced
    // plastic value falls below it; the manual prints no separate Class 3 reduction (G72).
    let w_major_red = if sec.z33 > 1e-9 {
        sec.z33
    } else {
        sec.s33.max(1e-9)
    };
    let w_minor_red = if sec.z22 > 1e-9 {
        sec.z22
    } else {
        sec.s22.max(1e-9)
    };
    // D92: EC3 does not design torsion on open shapes (manual 2.2.1: "The effects of torsion are
    // not considered in the design (EC3 6.2.7)"); Box and Pipe are refused above. Say so in the
    // result when the member has a torsion that is not zero against its moments.
    let mut notes: Vec<String> = Vec::new();
    if code == SteelCode::Eurocode3 {
        let mut t_max = 0.0_f64;
        let mut m_max = 0.0_f64;
        for (_, frames) in combo_forces {
            for (name, states) in frames {
                if name != frame_name {
                    continue;
                }
                for st in states.iter().flatten() {
                    t_max = t_max.max(st.t.abs());
                    m_max = m_max.max(st.m2.abs()).max(st.m3.abs());
                }
            }
        }
        if t_max > 1e-6 * m_max && t_max > 0.0 {
            notes.push(
                "torsion T is not designed (EC3 manual 2.2.1: effects of torsion are not considered)"
                    .to_string(),
            );
        }
    }
    let mut best_result: Option<SteelDesignResult> = None;

    for (combo_name, frame_map) in combo_forces {
        let Some((_, states)) = frame_map.iter().find(|(fn_k, _)| fn_k == frame_name) else {
            continue;
        };
        // Every state is a design entry of its own, with its own station list (so its own
        // moment diagram and Cb); a multi-valued combination adds the sub-combinations of
        // the maxima and minima of the forces this check combines (see `design_entries`).
        for stations in design_entries(states, interacting_forces) {
            let stations: &[FrameStationResult] = &stations;
            if stations.is_empty() {
                continue;
            }

            let cb = ow
                .and_then(|o| o.cb)
                .unwrap_or_else(|| {
                    let ratio = ow.and_then(|o| o.l_ratio_ltb).unwrap_or(1.0);
                    let minor = ow.and_then(|o| o.l_ratio_minor).unwrap_or(1.0);
                    // As in SAP2000, Cb is 1.0 when the unbraced length is
                    // set longer than the member.
                    if ratio > 1.0 || minor > 1.0 {
                        1.0
                    } else {
                        compute_cb_for_unbraced_ratio(stations, ratio)
                    }
                });

            let mut ec3_res: Option<Ec3MemberResistances> = None;
            // `phi_mn33_neg` is the strength for M3 < 0 (the stem of a tee in compression); it
            // equals `phi_mn33` (M3 >= 0) for every other shape and under EC3 (D129).
            let mut phi_mn33_neg = 0.0;
            let (mut phi_pnc, mut phi_pnt, mut phi_mn33, mut phi_mn22, mut phi_vn2, compactness) = match code {
                SteelCode::Aisc360Lrfd | SteelCode::Aisc360Asd => {
                    let s = compute_aisc_360_strengths(
                        sec, mat, l33, l22, l_ltb, k33, k22, cb, is_asd,
                    )
                    .map_err(|err| match err {
                        DesignError::SlenderPipeNotDesigned {
                            section,
                            d_over_t,
                            limit,
                            ..
                        } => DesignError::SlenderPipeNotDesigned {
                            frame: frame_name.to_string(),
                            section,
                            d_over_t,
                            limit,
                        },
                        other => other,
                    })?;
                    phi_mn33_neg = s.phi_mn33_neg;
                    // D129, tee and double angle: when the stem is compressed at any station of
                    // this state (M3 < 0 beyond roundoff), the stations with M3 >= 0 take the
                    // lateral-torsional value with the minus sign of B too.
                    let m3_gate = 1e-9 * mat.fy.max(1e-6) * sec.s33.max(1e-9);
                    let stem_compressed = stations.iter().any(|st| st.m3 < -m3_gate);
                    (
                        s.phi_pnc,
                        s.phi_pnt,
                        if stem_compressed {
                            s.phi_mn33_pos_stem_compressed
                        } else {
                            s.phi_mn33_pos
                        },
                        s.phi_mn22,
                        s.phi_vn2,
                        s.compactness,
                    )
                }
                SteelCode::Eurocode3 => {
                    let r = compute_ec3_member_resistances(
                        sec,
                        mat,
                        l33,
                        l22,
                        l_ltb,
                        k33,
                        k22,
                        cb,
                        ec3_classes.as_ref().map_or(1, |(_, bend)| bend.class),
                    );
                    let t = (
                        r.n_b_rd,
                        r.n_t_rd,
                        r.m_b_y_rd,
                        r.m_c_z_rd,
                        r.v_c_rd,
                        r.compactness,
                    );
                    ec3_res = Some(r);
                    t
                }
            };
            if code == SteelCode::Eurocode3 {
                phi_mn33_neg = phi_mn33;
            }

            // D89 (G71): EN 1993-1-1 6.3.3(4) with the Annex B (Method 2) factors for a state that
            // has a station with compression and a moment (the same roundoff-proof dimensionless
            // tests as the D90 gate below). Other shapes than the ones the Annex has columns for
            // stay refused by name.
            let is_cb_station = |st: &FrameStationResult, phi_pnc: f64, phi_mn33: f64, phi_mn22: f64| {
                st.p < 0.0
                    && st.p.abs() / phi_pnc.max(1e-12) > 1e-9
                    && st.m3.abs() / phi_mn33.max(1e-12) + st.m2.abs() / phi_mn22.max(1e-12) > 1e-9
            };
            let mut member_check: Option<Ec3MemberCheck> = None;
            let mut member_mode = false;
            if let (SteelCode::Eurocode3, Some((comp, bend))) = (code, &ec3_classes) {
                if let Some(first) = stations
                    .iter()
                    .find(|st| is_cb_station(st, phi_pnc, phi_mn33, phi_mn22))
                {
                    let unequal_i = sec.prop_type == FramePropType::I && {
                        let differs = |top: f64, bottom: f64| {
                            bottom > 0.0 && (bottom - top).abs() > 1e-9 * top.abs()
                        };
                        differs(sec.t2, sec.t2b) || differs(sec.tf, sec.tfb)
                    };
                    let shape = match sec.prop_type {
                        FramePropType::I if !unequal_i => Some((Ec3AnnexBShape::I, false)),
                        FramePropType::Box | FramePropType::ColdBox => {
                            Some((Ec3AnnexBShape::Rhs, false))
                        }
                        FramePropType::Pipe | FramePropType::ColdPipe => {
                            Some((Ec3AnnexBShape::Rhs, true))
                        }
                        _ => None,
                    };
                    let Some((shape, pipe)) = shape else {
                        return Err(DesignError::Ec3CompressionBendingUnsupported {
                            frame: frame_name.to_string(),
                            combo: combo_name.clone(),
                            station: first.station.to_string(),
                            section: format!(
                                "'{}' ({})",
                                sec.name,
                                if unequal_i {
                                    "I with unequal flanges".to_string()
                                } else {
                                    format!("{:?}", sec.prop_type)
                                }
                            ),
                        });
                    };
                    member_mode = true;
                    // The class of a member in compression and bending is the worse of the two
                    // (conservative against the vendor's Table 4-3, which is not built, G72);
                    // Class 4 stays refused by the D90 gate below.
                    let worst = comp.class.max(bend.class);
                    if worst <= 3 {
                        let r = compute_ec3_member_resistances(
                            sec,
                            mat,
                            l33,
                            l22,
                            l_ltb,
                            k33,
                            k22,
                            cb,
                            worst,
                        );
                        phi_pnc = r.n_b_rd;
                        phi_pnt = r.n_t_rd;
                        phi_mn33 = r.m_b_y_rd;
                        phi_mn33_neg = phi_mn33;
                        phi_mn22 = r.m_c_z_rd;
                        phi_vn2 = r.v_c_rd;
                        let full = |ratio: Option<f64>| ratio.is_none_or(|r| (r - 1.0).abs() < 1e-9);
                        let ctx = Ec3CmContext {
                            major_full: full(ow.and_then(|o| o.l_ratio_major)),
                            minor_full: full(ow.and_then(|o| o.l_ratio_minor)),
                            ltb_full: full(ow.and_then(|o| o.l_ratio_ltb)),
                            sway: ow.is_some_and(|o| o.is_sway),
                        };
                        member_check = Some(ec3_member_check(stations, &r, worst, shape, pipe, ctx));
                        ec3_res = Some(r);
                    }
                }
            }

            for (station_index, st) in stations.iter().enumerate() {
                // D129: the strength of the sign of this station's M3 (M3 >= 0 compresses the +2
                // face: the flange of a tee, whose stem is then in tension).
                let phi_mn33 = if st.m3 < 0.0 { phi_mn33_neg } else { phi_mn33 };
                // A station with compression and a moment of a member-mode state: the section
                // check of 6.2.1(7) with the section resistances, and the buckling checks 6.3.1
                // and 6.3.2 (the member equations 6.61 and 6.62 are applied below).
                let station_mode =
                    member_mode && is_cb_station(st, phi_pnc, phi_mn33, phi_mn22);
                // D90: a Class 4 (or unclassifiable) element needs effective properties that
                // are not built (G72): refuse the station that would use them.
                if let Some((comp, bend)) = &ec3_classes {
                    // The same roundoff-proof tests as D89 (dimensionless against the resistances),
                    // so that a compression of 1e-12 of N_b,Rd in a beam does not stop the run.
                    let compressed = st.p < 0.0 && st.p.abs() / phi_pnc.max(1e-12) > 1e-9;
                    let bent = st.m3.abs() / phi_mn33.max(1e-12)
                        + st.m2.abs() / phi_mn22.max(1e-12)
                        > 1e-9;
                    for (needed, class, load) in [
                        (compressed, comp, "compression"),
                        (bent, bend, "bending"),
                    ] {
                        if needed && class.class >= 4 {
                            return Err(DesignError::Ec3SectionClass {
                                frame: frame_name.to_string(),
                                section: sec.name.clone(),
                                detail: format!(
                                    "{load} has element '{}' with c/t = {} above the limit {} ({})",
                                    class.element,
                                    class.ratio,
                                    class.limit,
                                    if class.class == 4 {
                                        "Class 4, the limit is that of Class 3"
                                    } else {
                                        "not classifiable from the manual, the limit is that of Class 1"
                                    }
                                ),
                            });
                        }
                    }
                }
                let mut reported_phi_mn33 = phi_mn33;
                let mut reported_phi_mn22 = phi_mn22;
                let (
                    mut total_ratio,
                    mut pr_ratio,
                    mut mr_major_ratio,
                    mut mr_minor_ratio,
                    shear_major_ratio,
                    mut eq_name,
                ) = if code == SteelCode::Eurocode3 {
                    let r_v2 = st.v2.abs() / phi_vn2.max(1e-12);
                    let r_v3 = if phi_vn3 > 0.0 {
                        st.v3.abs() / phi_vn3
                    } else {
                        0.0
                    };

                    let rho2 = if r_v2 > 0.5 {
                        (2.0 * r_v2 - 1.0).powi(2)
                    } else {
                        0.0
                    };
                    let rho3 = if r_v3 > 0.5 {
                        (2.0 * r_v3 - 1.0).powi(2)
                    } else {
                        0.0
                    };

                    // The shear reduction (6.2.8) acts on the moment resistance of the check at hand:
                    // M_b,Rd for the plain station sum of today, the section resistance M_c,y,Rd
                    // for the 6.2.1(7) check of a station under compression and a moment.
                    let m3_cap = match (&ec3_res, station_mode) {
                        (Some(r), true) => r.m_c_y_rd,
                        _ => phi_mn33,
                    };
                    let reduced_m3 = if rho2 > 0.0 {
                        let m_red = match sec.prop_type {
                            FramePropType::I | FramePropType::Channel => {
                                let n = 1.0;
                                let hw = (sec.t3 - 2.0 * sec.tf).max(1e-6);
                                let aw = hw * sec.tw;
                                let wpl = w_major_red;
                                (wpl - n * rho2 * aw * aw / (4.0 * sec.tw.max(1e-6))) * mat.fy
                            }
                            FramePropType::T => {
                                let n = 1.0;
                                let hw = (sec.t3 - sec.tf).max(1e-6);
                                let aw = hw * sec.tw;
                                let wpl = w_major_red;
                                (wpl - n * rho2 * aw * aw / (4.0 * sec.tw.max(1e-6))) * mat.fy
                            }
                            FramePropType::Box
                            | FramePropType::ColdBox
                            | FramePropType::DbChannel
                            | FramePropType::DblAngle => {
                                let n = 2.0;
                                let hw = (sec.t3 - 2.0 * sec.tf).max(1e-6);
                                let aw = hw * sec.tw;
                                let wpl = w_major_red;
                                (wpl - n * rho2 * aw * aw / (4.0 * sec.tw.max(1e-6))) * mat.fy
                            }
                            _ => (1.0 - rho2) * m3_cap,
                        };
                        m_red.min(m3_cap).max(1e-12)
                    } else {
                        m3_cap
                    };
                    reported_phi_mn33 = reduced_m3;

                    let reduced_m2 = if rho3 > 0.0 {
                        let m_red = match sec.prop_type {
                            FramePropType::T => {
                                let n = 1.0;
                                let af = sec.t2 * sec.tf;
                                let wpl = w_minor_red;
                                (wpl - n * rho3 * af * af / (4.0 * sec.tf.max(1e-6))) * mat.fy
                            }
                            FramePropType::I
                            | FramePropType::Channel
                            | FramePropType::Box
                            | FramePropType::ColdBox => {
                                let n = 2.0;
                                let af = sec.t2 * sec.tf;
                                let wpl = w_minor_red;
                                (wpl - n * rho3 * af * af / (4.0 * sec.tf.max(1e-6))) * mat.fy
                            }
                            // D87: `t2` is the total width of a double section; af is one
                            // member's flange, ((t2 - dis)/2) tf.
                            FramePropType::DblAngle => {
                                let n = 2.0;
                                let af = 0.5 * (sec.t2 - sec.dis) * sec.tf;
                                let wpl = w_minor_red;
                                (wpl - n * rho3 * af * af / (4.0 * sec.tf.max(1e-6))) * mat.fy
                            }
                            FramePropType::DbChannel => {
                                let n = 4.0;
                                let af = 0.5 * (sec.t2 - sec.dis) * sec.tf;
                                let wpl = w_minor_red;
                                (wpl - n * rho3 * af * af / (4.0 * sec.tf.max(1e-6))) * mat.fy
                            }
                            _ => (1.0 - rho3) * phi_mn22,
                        };
                        m_red.min(phi_mn22).max(1e-12)
                    } else {
                        phi_mn22
                    };
                    reported_phi_mn22 = reduced_m2;

                    // 6.2.1(7) uses N_pl,Rd = A fy, the member checks use the buckling resistance.
                    let pc = match (&ec3_res, station_mode) {
                        (Some(r), true) => r.n_pl_rd.max(1e-12),
                        _ if st.p >= 0.0 => phi_pnt.max(1e-12),
                        _ => phi_pnc.max(1e-12),
                    };
                    let pr_part = st.p.abs() / pc;
                    let mr3_part = st.m3.abs() / reduced_m3;
                    let mr2_part = st.m2.abs() / reduced_m2;

                    let tot = if matches!(
                        sec.prop_type,
                        FramePropType::Pipe | FramePropType::ColdPipe | FramePropType::Circle
                    ) {
                        pr_part + (mr3_part * mr3_part + mr2_part * mr2_part).sqrt()
                    } else {
                        pr_part + mr3_part + mr2_part
                    };

                    let eq = if rho2 > 0.0 || rho3 > 0.0 {
                        "EC3-6.2.6+6.2.8(3)"
                    } else {
                        "EC3-6.2"
                    };

                    if station_mode {
                        // 6.3.1: N/(min(chi_y, chi_z) N_pl) = N/N_b,Rd; 6.3.2: M3/M_b,Rd.
                        let buck = st.p.abs() / phi_pnc.max(1e-12);
                        let ltb = st.m3.abs() / phi_mn33.max(1e-12);
                        if buck > tot && buck >= ltb {
                            (buck, buck, 0.0, 0.0, r_v2, "EC3-6.3.1")
                        } else if ltb > tot {
                            (ltb, 0.0, ltb, 0.0, r_v2, "EC3-6.3.2")
                        } else {
                            (tot, pr_part, mr3_part, mr2_part, r_v2, eq)
                        }
                    } else {
                        (tot, pr_part, mr3_part, mr2_part, r_v2, eq)
                    }
                } else {
                    evaluate_station_interaction(
                        code, st.p, st.v2, st.m3, st.m2, phi_pnc, phi_pnt, phi_mn33, phi_mn22,
                        phi_vn2,
                    )
                };

                // The member equations 6.61 / 6.62 of the state, at the station of its largest
                // moment term: the ratio of the state is the largest of them and the station checks.
                let mut member_note: Option<String> = None;
                if let Some(mc) = &member_check {
                    if station_index == mc.station_index && mc.ratio > total_ratio {
                        total_ratio = mc.ratio;
                        pr_ratio = mc.pr;
                        mr_major_ratio = mc.mr_major;
                        mr_minor_ratio = mc.mr_minor;
                        eq_name = mc.eq_name;
                        member_note = Some(format!(
                            "EC3 6.3.3(4) with the Annex B (Method 2) factors: Cmy = {:.4}, Cmz = {:.4}, CmLT = {:.4}, kyy = {:.4}, kyz = {:.4}, kzy = {:.4}, kzz = {:.4}",
                            mc.cm.0, mc.cm.1, mc.cm.2, mc.k.kyy, mc.k.kyz, mc.k.kzy, mc.k.kzz
                        ));
                    }
                }

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
                        let shear_sum = shear_major_ratio + shear_minor_ratio;
                        let tr_over_tc = tr / tc.max(1e-12);
                        let shear_torsion_ratio = (shear_sum + tr_over_tc).powi(2);
                        let ratio_h3 = if matches!(
                            sec.prop_type,
                            FramePropType::Pipe | FramePropType::ColdPipe
                        ) {
                            // D85: the Pipe form of H3-6 (manual p. 3-81, rendered page) is
                            // Pr/Pc + sqrt[(Mr33/Mc33)^2 + (Mr22/Mc22)^2] + (...)^2; an
                            // earlier version squared each ratio without the radical.
                            pr_over_pc
                                + (mr3_over_mc3.powi(2) + mr2_over_mc2.powi(2)).sqrt()
                                + shear_torsion_ratio
                        } else {
                            pr_over_pc + mr3_over_mc3 + mr2_over_mc2 + shear_torsion_ratio
                        };
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
                let gov_eq = if code == SteelCode::Eurocode3 {
                    if shear_minor_ratio > total_ratio.max(shear_major_ratio) {
                        "EC3-6.2.6 (V3)"
                    } else if shear_major_ratio > total_ratio {
                        "EC3-6.2.6 (V2)"
                    } else {
                        eq_name
                    }
                } else if shear_minor_ratio > total_ratio.max(shear_major_ratio) {
                    "G7/G2-1 (V3)"
                } else if shear_major_ratio > total_ratio {
                    "G2-1"
                } else {
                    eq_name
                };

                // D91: for an I section the flexural limit state of a slender flange is built
                // (F3-2, `mn_flb`), so a slender flange alone does not make the result
                // "SLENDER"; a slender web (F5) still does. D129: under AISC the result reads
                // "SLENDER" only where the shape's slender limit state is still not built: the
                // I with a slender web and the built-up I types. The box, pipe, channel, double
                // channel, tee and double angle are designed (a slender box flange and the
                // channel webs beyond the compact limit are refused instead, G82, G83) and the
                // single angle is refused. Under EC3 the rule is the earlier one (D133).
                let slender_flange_only_of_i = sec.prop_type == FramePropType::I
                    && compactness.flange_flexure == CompactnessClass::Slender
                    && compactness.web_flexure != CompactnessClass::Slender;
                let slender_status = if code == SteelCode::Eurocode3 {
                    compactness.is_slender() && !slender_flange_only_of_i
                } else {
                    match sec.prop_type {
                        FramePropType::T
                        | FramePropType::DblAngle
                        | FramePropType::Channel
                        | FramePropType::DbChannel
                        | FramePropType::Box
                        | FramePropType::ColdBox
                        | FramePropType::Pipe
                        | FramePropType::ColdPipe
                        | FramePropType::Angle => false,
                        _ => compactness.is_slender() && !slender_flange_only_of_i,
                    }
                };
                let status = if gov_ratio > 1.0 {
                    "OVERSTRESSED".to_string()
                } else if slender_status {
                    "SLENDER".to_string()
                } else {
                    "OK".to_string()
                };

                let is_more_critical = match &best_result {
                    None => true,
                    Some(prev) => gov_ratio > prev.total_ratio.max(prev.shear_major_ratio) + 1e-12,
                };

                if is_more_critical {
                    let mut result_notes = notes.clone();
                    // only when the member check is what governs (not a shear check above it)
                    if gov_eq == eq_name && eq_name.starts_with("EC3-6.3.3(4)") {
                        result_notes.extend(member_note);
                    }
                    best_result = Some(SteelDesignResult {
                        frame_name: frame_name.to_string(),
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
                        phi_mn33: reported_phi_mn33,
                        phi_mn22: reported_phi_mn22,
                        phi_vn2,
                        phi_vn3,
                        status,
                        eq_name: gov_eq.to_string(),
                        notes: result_notes,
                    });
                }
            }
        }
    }

    Ok(best_result)
}
