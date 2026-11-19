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
/// This function gives $Q$ only: for T, single angle, double angle and channel the
/// compression strength still uses flexural buckling alone (E4-5 and E4-6 are not built).
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
) -> Result<(f64, f64, f64, f64, f64, SectionCompactness), DesignError> {
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

    Ok((phi_pnc, phi_pnt, phi_mn33, phi_mn22, phi_vn2, compactness))
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

            let (phi_pnc, phi_pnt, phi_mn33, phi_mn22, phi_vn2, compactness) = match code {
                SteelCode::Aisc360Lrfd | SteelCode::Aisc360Asd => {
                    compute_aisc_360_capacities(sec, mat, l33, l22, l_ltb, k33, k22, cb, is_asd)
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
                        })?
                }
                SteelCode::Eurocode3 => {
                    compute_ec3_capacities(
                        sec,
                        mat,
                        l33,
                        l22,
                        l_ltb,
                        k33,
                        k22,
                        cb,
                        ec3_classes.as_ref().map_or(1, |(_, bend)| bend.class),
                    )
                }
            };

            for st in stations {
                // D89 (G71): EC3 6.3.3(4) under compression and bending needs the interaction
                // factors of Annex A or B, which neither vendor manual prints.
                if code == SteelCode::Eurocode3
                    && st.p < 0.0
                    && st.p.abs() / phi_pnc.max(1e-12) > 1e-9
                    && st.m3.abs() / phi_mn33.max(1e-12) + st.m2.abs() / phi_mn22.max(1e-12) > 1e-9
                {
                    return Err(DesignError::Ec3CompressionBendingUnsupported {
                        frame: frame_name.to_string(),
                        combo: combo_name.clone(),
                        station: st.station.to_string(),
                    });
                }
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
                            _ => (1.0 - rho2) * phi_mn33,
                        };
                        m_red.min(phi_mn33).max(1e-12)
                    } else {
                        phi_mn33
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

                    let pc = if st.p >= 0.0 {
                        phi_pnt.max(1e-12)
                    } else {
                        phi_pnc.max(1e-12)
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

                    (tot, pr_part, mr3_part, mr2_part, r_v2, eq)
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
                // "SLENDER"; a slender web (F5), and every other shape whose noncompact or slender
                // flexure is taken as Mp, still do.
                let slender_flange_only_of_i = sec.prop_type == FramePropType::I
                    && compactness.flange_flexure == CompactnessClass::Slender
                    && compactness.web_flexure != CompactnessClass::Slender;
                let status = if gov_ratio > 1.0 {
                    "OVERSTRESSED".to_string()
                } else if compactness.is_slender() && !slender_flange_only_of_i {
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
                        notes: notes.clone(),
                    });
                }
            }
        }
    }

    Ok(best_result)
}
