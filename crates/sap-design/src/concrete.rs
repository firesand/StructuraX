use crate::{
    error::validate_demands,
    steel::{collect_design_combo_forces, frame_clear_length},
    DesignError,
};
use sap_model::{ConcreteDesignResult, FrameObj, FramePropType, FrameSection, MatType, SapModel};
use sap_units::Units;
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

/// Concrete design code variant (`sapdesign.dll` `apm` for ACI 318-14 and `ap7` for Eurocode 2-2004).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ConcreteCode {
    Aci318,
    Eurocode2,
}

impl ConcreteCode {
    /// Parses a supported concrete code name without silently selecting a fallback code.
    pub fn try_from_code_str(code: &str) -> Result<Self, DesignError> {
        let normalized: String = code
            .chars()
            .filter(|c| c.is_ascii_alphanumeric())
            .flat_map(char::to_uppercase)
            .collect();
        match normalized.as_str() {
            "ACI318" | "ACI31814" | "ACI" => Ok(ConcreteCode::Aci318),
            "EUROCODE2" | "EUROCODE22004" | "EC2" | "EN199211" | "EN1992112004" => {
                Ok(ConcreteCode::Eurocode2)
            }
            _ => Err(DesignError::UnsupportedConcreteCode(code.to_string())),
        }
    }

    /// Legacy permissive parser. New design runs should use [`Self::try_from_code_str`].
    pub fn from_code_str(code: &str) -> Self {
        Self::try_from_code_str(code).unwrap_or(ConcreteCode::Aci318)
    }
}

/// Computes the ACI 318-14 Section 22.2.2.4.3 Whitney rectangular stress block factor $\beta_1$:
/// $$\beta_1 = \text{clamp}\left(0.85 - 0.05 \frac{f'_{c,\text{psi}} - 4000}{1000},\; 0.65,\; 0.85\right)$$
pub fn compute_whitney_beta1(fc_psi: f64) -> f64 {
    if fc_psi <= 4000.0 {
        0.85
    } else {
        (0.85 - 0.05 * ((fc_psi - 4000.0) / 1000.0)).clamp(0.65, 0.85)
    }
}

/// Computes required flexural reinforcement area $A_s$ and minimum reinforcement $A_{s,\min}$
/// for a rectangular concrete beam section ($b \times h$, effective depth $d$, compression cover $d'$)
/// under factored moment $M_u \ge 0$ per ACI 318-14.
pub fn design_aci_beam_flexure(
    mu: f64,
    b: f64,
    d: f64,
    d_prime: f64,
    fc: f64,
    fy: f64,
    es: f64,
    units: Units,
) -> (f64, f64, bool) {
    let to_psi = units.stress_factor_to(Units::LbInF);
    let fc_psi = (fc * to_psi).max(2000.0);
    let fy_psi = (fy * to_psi).max(20000.0);
    let beta1 = compute_whitney_beta1(fc_psi);

    // Minimum flexural reinforcement per ACI 318-14 Section 9.6.1.2
    let rho_min = (3.0 * fc_psi.sqrt() / fy_psi).max(200.0 / fy_psi);
    let as_min = rho_min * b * d;

    if mu <= 1e-12 {
        return (0.0, as_min, false);
    }

    let phi_b = 0.90;
    let ecu = 0.003;
    // Maximum tension-controlled c/d ratio (et = 0.005 -> c/d = 0.375)
    let c_max = 0.375 * d;
    let a_max = beta1 * c_max;
    let mn_max_single = 0.85 * fc * a_max * b * (d - 0.5 * a_max);
    let phi_mn_max_single = phi_b * mn_max_single;

    if mu <= phi_mn_max_single {
        // Singly reinforced Whitney stress block exact quadratic solution:
        // Mu = phi_b * 0.85 * fc * b * a * (d - a/2)
        let rn = mu / (phi_b * b * d * d).max(1e-12);
        let term = (1.0 - (2.0 * rn) / (0.85 * fc).max(1e-12)).max(0.0);
        let a = d * (1.0 - term.sqrt());
        let as_req = (0.85 * fc * a * b) / fy.max(1e-12);
        // ACI 318-14 Section 9.6.1.3: enforce As_min unless As_req * 4/3 < As_min
        let as_govern = as_req.max(as_min.min((4.0 / 3.0) * as_req));
        (as_govern, as_min, false)
    } else {
        // Doubly reinforced beam section
        let as1 = (0.85 * fc * a_max * b) / fy.max(1e-12);
        let mu2 = mu - phi_mn_max_single;
        let lever = (d - d_prime).max(0.5 * d);
        let as2 = mu2 / (phi_b * fy * lever).max(1e-12);
        let eps_s_prime = ecu * ((c_max - d_prime) / c_max).max(0.0);
        let fs_prime = (eps_s_prime * es).min(fy);
        let overstressed = fs_prime <= 0.0 || mu2 > 1.5 * phi_mn_max_single;
        (as1 + as2, as_min, overstressed)
    }
}

/// Computes required shear stirrup area per unit spacing $A_v / s$ per ACI 318-14 Section 22.5.
pub fn design_aci_beam_shear(
    vu: f64,
    bw: f64,
    d: f64,
    fc: f64,
    fyt: f64,
    units: Units,
) -> (f64, bool) {
    let to_psi = units.stress_factor_to(Units::LbInF);
    let fc_psi = (fc * to_psi).max(2000.0);
    let fyt_psi = (fyt * to_psi).max(20000.0);

    let phi_v = 0.75;
    // vc = 2 * sqrt(fc_psi) in psi -> convert back to model stress units
    let vc_stress = (2.0 * fc_psi.sqrt()) / to_psi;
    let vc = vc_stress * bw * d;
    let phi_vc = phi_v * vc;

    let vs_max_stress = (8.0 * fc_psi.sqrt()) / to_psi;
    let phi_vn_max = phi_vc + phi_v * vs_max_stress * bw * d;
    let overstressed = vu > phi_vn_max;

    // Minimum shear reinforcement Av,min / s when Vu > 0.5 * phi * Vc (ACI 318-14 Table 9.6.3.3)
    let av_s_min = ((0.75 * fc_psi.sqrt()) / fyt_psi).max(50.0 / fyt_psi) * bw;

    if vu <= 0.5 * phi_vc {
        (0.0, false)
    } else if vu <= phi_vc {
        (av_s_min, false)
    } else {
        let vs_req = (vu - phi_vc) / phi_v;
        let av_s_req = vs_req / (fyt * d).max(1e-12);
        (av_s_req.max(av_s_min), overstressed)
    }
}

/// Computes required flexural and shear reinforcement for a beam under Eurocode 2-2004 (`EN 1992-1-1`).
pub fn design_ec2_beam(
    mu: f64,
    vu: f64,
    b: f64,
    d: f64,
    d_prime: f64,
    fck: f64,
    fyk: f64,
    fytk: f64,
    units: Units,
) -> (f64, f64, f64, bool) {
    let to_mpa = units.stress_factor_to(Units::NMmC);
    let fck_mpa = (fck * to_mpa).max(12.0);
    let fyk_mpa = (fyk * to_mpa).max(200.0);

    let gamma_c = 1.50;
    let gamma_s = 1.15;
    let fcd = 0.85 * fck / gamma_c;
    let fyd = fyk / gamma_s;
    let fywd = fytk / gamma_s;

    // Mean tensile strength fctm = 0.30 * fck^(2/3) MPa (EN 1992-1-1 Table 3.1)
    let fctm_mpa = 0.30 * fck_mpa.powf(2.0 / 3.0);
    let rho_min = (0.26 * fctm_mpa / fyk_mpa).max(0.0013);
    let as_min = rho_min * b * d;

    // Flexural design (EN 1992-1-1 6.1, K_bal = 0.167 for x/d <= 0.45)
    let k = mu / (b * d * d * fck).max(1e-12);
    let k_bal = 0.167;
    let (as_req, over_flex) = if mu <= 1e-12 {
        (0.0, false)
    } else if k <= k_bal {
        let z = d * (0.5 + (0.25 - k / 1.134).max(0.0).sqrt()).min(0.95);
        let as_val = (mu / (fyd * z).max(1e-12)).max(as_min);
        (as_val, false)
    } else {
        let z_bal = d * (0.5 + (0.25 - k_bal / 1.134).sqrt());
        let m_bal = k_bal * b * d * d * fck;
        let as1 = m_bal / (fyd * z_bal);
        let as2 = (mu - m_bal) / (fyd * (d - d_prime).max(0.5 * d));
        (as1 + as2, k > 2.0 * k_bal)
    };

    // Shear design (EN 1992-1-1 6.2.3 variable strut inclination cot(theta) = 2.5 clamped)
    let z = 0.9 * d;
    let cot_theta = 2.5;
    let rho_w_min = (0.08 * fck_mpa.sqrt()) / fyk_mpa;
    let asw_s_min = rho_w_min * b;
    let v_rd_c_stress =
        ((0.12 * (100.0 * rho_min * fck_mpa).cbrt()).max(0.035 * fck_mpa.sqrt())) / to_mpa;
    let v_rd_c = v_rd_c_stress * b * d;
    let nu1 = 0.6 * (1.0 - fck_mpa / 250.0).max(0.3);
    let v_rd_max = (b * z * nu1 * fcd) / (cot_theta + 1.0 / cot_theta);

    let av_s = if vu <= v_rd_c {
        asw_s_min
    } else {
        let req = vu / (z * fywd * cot_theta).max(1e-12);
        req.max(asw_s_min)
    };
    let overstressed = over_flex || vu > v_rd_max;

    (as_req, as_min, av_s, overstressed)
}

/// Point on a uniaxial factored P-M interaction curve $(\phi P_n, \phi M_n)$.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PmCurvePoint {
    pub phi_pn: f64,
    pub phi_mn: f64,
}

/// Generates a 1D rebar layer distribution `(depth_from_top, bar_area)` for a concrete column
/// bent about either the major (3-axis, depth $h = t_3$, width $b = t_2$) or minor (2-axis, depth $b = t_2$, width $h = t_3$).
fn build_column_rebar_layers(
    sec: &FrameSection,
    major_axis: bool,
    default_cover: f64,
) -> (f64, f64, f64, Vec<(f64, f64)>) {
    let ag = sec.area.max(1e-9);
    let (h, b) = if major_axis {
        (sec.t3.max(1e-6), sec.t2.max(1e-6))
    } else {
        (sec.t2.max(1e-6), sec.t3.max(1e-6))
    };

    let (cover, n_depth, n_width, ast) = if let Some(cfg) = &sec.concrete_rebar {
        let c = cfg.cover_top.max(cfg.cover_bot).max(default_cover);
        let ast_val = if cfg.total_rebar_area > 1e-9 {
            cfg.total_rebar_area
        } else {
            0.02 * ag
        };
        let (nd, nw) = if major_axis {
            (cfg.num_bars_3dir.max(2), cfg.num_bars_2dir.max(2))
        } else {
            (cfg.num_bars_2dir.max(2), cfg.num_bars_3dir.max(2))
        };
        (c, nd, nw, ast_val)
    } else {
        (default_cover, 3, 3, 0.02 * ag)
    };

    if sec.prop_type == FramePropType::Circle {
        let num_bars = (2 * (n_depth + n_width) - 4).max(6);
        let a_bar = ast / (num_bars as f64);
        let r_rebar = (0.5 * h - cover).max(0.1 * h);
        let mut layers = Vec::with_capacity(num_bars);
        for i in 0..num_bars {
            let theta = 2.0 * PI * (i as f64) / (num_bars as f64);
            let di = 0.5 * h - r_rebar * theta.cos();
            layers.push((di, a_bar));
        }
        return (h, b, ast, layers);
    }

    // Rectangular perimeter bar layout:
    // Top face has `n_width` bars, bottom face has `n_width` bars, intermediate layers have 2 bars each
    let total_bars = (2 * n_width + 2 * (n_depth.saturating_sub(2))).max(4);
    let a_bar = ast / (total_bars as f64);
    let d_min = cover.min(0.4 * h);
    let d_max = (h - d_min).max(d_min + 1e-6);

    let mut layers = Vec::with_capacity(n_depth);
    if n_depth <= 2 {
        layers.push((d_min, (n_width as f64) * a_bar));
        layers.push((d_max, (n_width as f64) * a_bar));
    } else {
        for idx in 0..n_depth {
            let frac = (idx as f64) / ((n_depth - 1) as f64);
            let di = d_min + frac * (d_max - d_min);
            let count = if idx == 0 || idx + 1 == n_depth {
                n_width
            } else {
                2
            };
            layers.push((di, (count as f64) * a_bar));
        }
    }

    (h, b, ast, layers)
}

/// Integrates the uniaxial factored P-M interaction curve $(\phi P_n, \phi M_n)$ for a column
/// using strain compatibility ($\epsilon_{cu} = 0.003$) and the Whitney stress block per ACI 318-14
/// (or Eurocode 2 when `code == ConcreteCode::Eurocode2`).
pub fn generate_column_pm_curve(
    sec: &FrameSection,
    fc: f64,
    fy: f64,
    es: f64,
    units: Units,
    code: ConcreteCode,
    major_axis: bool,
) -> (Vec<PmCurvePoint>, f64, f64, f64) {
    let default_cover = Units::KipInF.length_factor_to(units) * 2.5;
    let (h, b, ast, layers) = build_column_rebar_layers(sec, major_axis, default_cover);
    let is_spiral = sec.prop_type == FramePropType::Circle;
    let ag = if sec.area > 0.0 {
        sec.area
    } else if is_spiral {
        0.25 * PI * b * h
    } else {
        b * h
    };
    let to_psi = units.stress_factor_to(Units::LbInF);
    let fc_psi = (fc * to_psi).max(2000.0);

    let (alpha_cc, beta1, phi_comp_min, phi_tens_max, alpha_cap) = match code {
        ConcreteCode::Aci318 => {
            let b1 = compute_whitney_beta1(fc_psi);
            let (phi_c, cap) = if is_spiral {
                (0.75, 0.85)
            } else {
                (0.65, 0.80)
            };
            (0.85, b1, phi_c, 0.90, cap)
        }
        ConcreteCode::Eurocode2 => {
            // EN 1992-1-1: fcd = 0.85 * fck / 1.5, fyd = fyk / 1.15, lambda = 0.80
            (0.85 / 1.50, 0.80, 1.0, 1.0, 1.0)
        }
    };

    let fy_eff = match code {
        ConcreteCode::Aci318 => fy,
        ConcreteCode::Eurocode2 => fy / 1.15,
    };

    let ecu = 0.003;
    let ety = (fy_eff / es.max(1.0)).max(0.002);
    let d_extreme = layers.iter().map(|(d, _)| *d).fold(0.5 * h, f64::max);

    let p0 = alpha_cc * fc * (ag - ast) + fy_eff * ast;
    let phi_p0 = phi_comp_min * p0;
    let phi_pn_max = alpha_cap * phi_p0;
    let phi_pnt = -phi_tens_max * fy_eff * ast;

    let mut curve = Vec::with_capacity(64);
    // Point 0: Pure compression cap
    curve.push(PmCurvePoint {
        phi_pn: phi_pn_max,
        phi_mn: 0.0,
    });

    // Sweep neutral axis depth c from 2.5 * h down to 0.04 * h
    let num_steps = 60;
    for step in 0..=num_steps {
        let t = (step as f64) / (num_steps as f64);
        // Logarithmic/power spacing to cluster around balanced neutral axis
        let c = h * (2.2 * (1.0 - t).powi(2) + 0.04);
        let a = (beta1 * c).min(h);

        // Concrete compression resultant Cc and its lever arm about plastic centroid (h/2)
        let (cc, mc) = if is_spiral {
            let r = 0.5 * h;
            let cos_arg = ((r - a) / r.max(1e-12)).clamp(-1.0, 1.0);
            let alpha = cos_arg.acos();
            let a_seg = r * r * (alpha - alpha.sin() * alpha.cos());
            let q_seg = (2.0 / 3.0) * r * r * r * alpha.sin().powi(3);
            (alpha_cc * fc * a_seg, alpha_cc * fc * q_seg)
        } else {
            let cc_rect = alpha_cc * fc * a * b;
            (cc_rect, cc_rect * (0.5 * h - 0.5 * a))
        };

        let mut fs_sum = 0.0;
        let mut ms_sum = 0.0;
        for &(di, asi) in &layers {
            let eps_i = ecu * (c - di) / c;
            let sigma_s = (eps_i * es).clamp(-fy_eff, fy_eff);
            // Subtract displaced concrete stress if bar lies inside Whitney block
            let net_sigma = if di <= a && sigma_s > 0.0 {
                sigma_s - alpha_cc * fc
            } else {
                sigma_s
            };
            let fi = net_sigma * asi;
            fs_sum += fi;
            ms_sum += fi * (0.5 * h - di);
        }

        let pn = cc + fs_sum;
        let mn = (mc + ms_sum).max(0.0);

        // Tensile strain in extreme tension steel (positive in tension)
        let eps_t = ecu * (d_extreme - c) / c;
        let phi = if code == ConcreteCode::Eurocode2 {
            1.0
        } else if eps_t <= ety {
            phi_comp_min
        } else if eps_t >= 0.005 {
            phi_tens_max
        } else {
            phi_comp_min + (phi_tens_max - phi_comp_min) * ((eps_t - ety) / (0.005 - ety).max(1e-9))
        };

        let phi_pn = (phi * pn).min(phi_pn_max);
        let phi_mn = phi * mn;
        curve.push(PmCurvePoint { phi_pn, phi_mn });
    }

    // Final point: Pure tension
    curve.push(PmCurvePoint {
        phi_pn: phi_pnt,
        phi_mn: 0.0,
    });

    (curve, phi_p0, phi_pn_max, ast)
}

/// Interpolates the uniaxial flexural capacity $\phi M_{n0}(P_u)$ from a P-M curve at axial load $P_u$
/// (where $P_u > 0$ is compression, $P_u < 0$ is tension).
pub fn interpolate_phi_mn_at_pu(curve: &[PmCurvePoint], pu: f64) -> f64 {
    if curve.is_empty() {
        return 0.0;
    }
    let p_max = curve.first().unwrap().phi_pn;
    let p_min = curve.last().unwrap().phi_pn;
    if pu > p_max || pu < p_min {
        return 0.0;
    }

    let mut max_m = 0.0_f64;
    for w in curve.windows(2) {
        let p0 = w[0].phi_pn;
        let p1 = w[1].phi_pn;
        let hi = p0.max(p1);
        let lo = p0.min(p1);
        if pu <= hi + 1e-9 && pu >= lo - 1e-9 {
            let dp = p0 - p1;
            let m = if dp.abs() < 1e-12 {
                w[0].phi_mn.max(w[1].phi_mn)
            } else {
                let t = ((p0 - pu) / dp).clamp(0.0, 1.0);
                w[0].phi_mn + t * (w[1].phi_mn - w[0].phi_mn)
            };
            max_m = max_m.max(m);
        }
    }
    max_m
}

/// Evaluates 3D Biaxial P-M2-M3 interaction demand/capacity ratio (`pmm_ratio`) using both
/// the **Bresler Load Contour Method**:
/// $$\left(\frac{M_{u3}}{\phi M_{n30}(P_u)}\right)^\alpha + \left(\frac{M_{u2}}{\phi M_{n20}(P_u)}\right)^\alpha \le 1.0$$
/// and axial capacity check $P_u / \phi P_{n,\max}$.
pub fn evaluate_column_biaxial_pmm_ratio(
    pu_comp_positive: f64,
    mu3: f64,
    mu2: f64,
    curve_major: &[PmCurvePoint],
    curve_minor: &[PmCurvePoint],
    phi_pn_max: f64,
    phi_pnt: f64,
    h3: f64,
    h2: f64,
) -> f64 {
    let mu3 = mu3.abs();
    let mu2 = mu2.abs();

    // Minimum eccentricity per ACI 318-14 (0.6 + 0.03*h in inches) when bending moments are tiny
    let axial_ratio = if pu_comp_positive >= 0.0 {
        pu_comp_positive / phi_pn_max.max(1e-12)
    } else {
        pu_comp_positive.abs() / phi_pnt.abs().max(1e-12)
    };

    if axial_ratio > 1.0 && mu3 < 1e-9 && mu2 < 1e-9 {
        return axial_ratio;
    }

    // Clamp Pu inside curve domain to evaluate moment contour, then combine with axial ratio
    let pu_clamped = pu_comp_positive.clamp(phi_pnt * 0.999, phi_pn_max * 0.999);
    let phi_mn30 = interpolate_phi_mn_at_pu(curve_major, pu_clamped).max(1e-9);
    let phi_mn20 = interpolate_phi_mn_at_pu(curve_minor, pu_clamped).max(1e-9);

    // Bresler load contour exponent alpha = 1.15 (or 1.24 for standard rectangular columns)
    let alpha = 1.20_f64;
    let r3 = mu3 / phi_mn30;
    let r2 = mu2 / phi_mn20;
    let contour_ratio = (r3.powf(alpha) + r2.powf(alpha)).powf(1.0 / alpha);

    // Also account for minimum eccentricity moment ratio if pure axial compression
    let e_min_ratio = if pu_comp_positive > 0.0 && mu3 < 1e-9 && mu2 < 1e-9 {
        let m_min3 = pu_comp_positive * 0.05 * h3;
        let m_min2 = pu_comp_positive * 0.05 * h2;
        (m_min3 / phi_mn30).max(m_min2 / phi_mn20)
    } else {
        0.0
    };

    axial_ratio.max(contour_ratio).max(e_min_ratio)
}

/// Determines whether a concrete frame element is designed as a column (`true`) or a beam (`false`).
pub fn is_concrete_column(model: &SapModel, frame: &FrameObj, sec: &FrameSection) -> bool {
    if let Some(cfg) = &sec.concrete_rebar {
        return cfg.is_column;
    }
    if sec.prop_type == FramePropType::Circle {
        return true;
    }
    if let (Some(pi), Some(pj)) = (
        model.points.get(&frame.point_i),
        model.points.get(&frame.point_j),
    ) {
        let dx = pj.x - pi.x;
        let dy = pj.y - pi.y;
        let dz = pj.z - pi.z;
        let len = (dx * dx + dy * dy + dz * dz).sqrt();
        if len > 1e-9 && (dz.abs() / len) > 0.707 {
            return true;
        }
    }
    false
}

/// Runs concrete frame design for all concrete frame members in `model` and populates `model.concrete_design_results`.
pub fn run_concrete_design(model: &mut SapModel) {
    let _ = try_run_concrete_design(model);
}

/// Checked concrete design entry point. Unsupported code names or demands return an error and
/// leave `concrete_design_results` empty, preventing stale/incomplete ratios from appearing valid.
pub fn try_run_concrete_design(model: &mut SapModel) -> Result<(), DesignError> {
    model.concrete_design_results.clear();
    let code = ConcreteCode::try_from_code_str(&model.concrete_design_code)?;
    let combo_forces = collect_design_combo_forces(model);
    validate_demands(
        model,
        &combo_forces,
        "concrete",
        true,
        |model, frame_name| {
            let Some(frame) = model.frames.get(frame_name) else {
                return false;
            };
            let Some(section) = model.frame_sections.get(&frame.section) else {
                return false;
            };
            model
                .materials
                .get(&section.material)
                .is_some_and(|material| material.mat_type == MatType::Concrete)
        },
    )?;
    run_concrete_design_validated(model, code, combo_forces);
    Ok(())
}

fn run_concrete_design_validated(
    model: &mut SapModel,
    code: ConcreteCode,
    combo_forces: Vec<(String, Vec<(String, Vec<sap_model::FrameStationResult>)>)>,
) {
    let units = model.present_units;

    let mut results = Vec::new();

    // Standard steel modulus Es = 29,000 ksi converted to present units
    let es_default = 29_000.0 * Units::KipInF.stress_factor_to(units);
    let default_cover = 2.5 * Units::KipInF.length_factor_to(units);

    for (frame_name, frame) in &model.frames {
        let Some(sec) = model.frame_sections.get(&frame.section) else {
            continue;
        };
        let Some(mat) = model.materials.get(&sec.material) else {
            continue;
        };
        if mat.mat_type != MatType::Concrete {
            continue;
        }

        // Resolve longitudinal and confinement rebar yield stresses fy, fyt
        let mut fy = if mat.fy > 0.0 {
            mat.fy
        } else {
            60.0 * Units::KipInF.stress_factor_to(units)
        };
        let mut fyt = if mat.fys > 0.0 { mat.fys } else { fy };
        let mut es = es_default;

        if let Some(cfg) = &sec.concrete_rebar {
            if let Some(rmat) = model.materials.get(&cfg.rebar_mat_long) {
                if rmat.fy > 0.0 {
                    fy = rmat.fy;
                }
                if (rmat.mat_type == MatType::Rebar || rmat.mat_type == MatType::Steel)
                    && rmat.e > 0.0
                {
                    es = rmat.e;
                }
            }
            if let Some(cmat) = model.materials.get(&cfg.rebar_mat_confine) {
                if cmat.fy > 0.0 {
                    fyt = cmat.fy;
                } else if cmat.fys > 0.0 {
                    fyt = cmat.fys;
                }
            }
        }

        let fc = if mat.fc > 0.0 {
            mat.fc
        } else {
            4.0 * Units::KipInF.stress_factor_to(units)
        };

        let is_col = is_concrete_column(model, frame, sec);
        let h = sec.t3.max(1e-6);
        let b = sec.t2.max(1e-6);
        let (cover_top, cover_bot) = if let Some(cfg) = &sec.concrete_rebar {
            (
                if cfg.cover_top > 0.0 {
                    cfg.cover_top
                } else {
                    default_cover
                },
                if cfg.cover_bot > 0.0 {
                    cfg.cover_bot
                } else {
                    default_cover
                },
            )
        } else {
            let c = default_cover.min(0.15 * h);
            (c, c)
        };

        let d_bot = (h - cover_bot).max(0.5 * h);
        let d_top = (h - cover_top).max(0.5 * h);
        let is_circle = sec.prop_type == FramePropType::Circle;
        let (bw2, d2_shear) = if is_circle { (h, 0.8 * h) } else { (b, d_bot) };
        let d3_minor = (b - cover_bot).max(0.5 * b);
        let cover_minor = cover_bot.min(0.4 * b);
        let (bw3, d3_shear) = if is_circle {
            (h, 0.8 * h)
        } else {
            (h, d3_minor)
        };

        if is_col {
            let (curve_major, _phi_p0, phi_pn_max, ast) =
                generate_column_pm_curve(sec, fc, fy, es, units, code, true);
            let (curve_minor, _, _, _) =
                generate_column_pm_curve(sec, fc, fy, es, units, code, false);
            let phi_pnt = curve_major
                .last()
                .map(|p| p.phi_pn)
                .unwrap_or(-0.9 * fy * ast);
            let min_rebar_area = 0.01 * sec.area;
            let l_clear = frame_clear_length(model, frame);

            let mut max_pmm_ratio = 0.0_f64;
            let mut max_av_s = 0.0_f64;
            let mut any_shear_overstressed = false;
            let mut gov_combo = "NONE".to_string();

            // Euler critical buckling load Pc for non-sway moment magnification (ACI 318-14 6.6.4.3 & 6.6.4.5)
            let r_gyr = sec.r33.min(sec.r22).max(0.3 * h.min(b));
            let kl_over_r = l_clear / r_gyr.max(1e-6);
            let ei_eff = 0.40 * mat.e * sec.i33 / 1.4;
            let pc_euler = if l_clear > 1e-6 {
                (PI * PI * ei_eff) / (l_clear * l_clear)
            } else {
                1e18
            };

            let mut buckled = false;
            for (combo_name, frame_map) in &combo_forces {
                let Some((_, stations)) = frame_map.iter().find(|(fn_k, _)| fn_k == frame_name)
                else {
                    continue;
                };
                for st in stations {
                    // SAP2000 sign convention: st.p < 0 is compression, st.p > 0 is tension
                    let pu_comp = -st.p;
                    if kl_over_r > 34.0 && pu_comp > 0.0 && pu_comp >= 0.75 * pc_euler {
                        buckled = true;
                        let buck_ratio = (pu_comp / (0.75 * pc_euler.max(1e-12))).max(999.0);
                        if buck_ratio > max_pmm_ratio {
                            max_pmm_ratio = buck_ratio;
                            gov_combo = combo_name.clone();
                        }
                        continue;
                    }
                    let delta_ns = if kl_over_r > 34.0 && pu_comp > 0.0 {
                        (1.0 / (1.0 - pu_comp / (0.75 * pc_euler))).max(1.0)
                    } else {
                        1.0
                    };
                    let mu3 = st.m3.abs() * delta_ns;
                    let mu2 = st.m2.abs() * delta_ns;

                    let pmm = evaluate_column_biaxial_pmm_ratio(
                        pu_comp,
                        mu3,
                        mu2,
                        &curve_major,
                        &curve_minor,
                        phi_pn_max,
                        phi_pnt,
                        h,
                        b,
                    );
                    let (av_s_2, over_v2) = match code {
                        ConcreteCode::Aci318 => {
                            design_aci_beam_shear(st.v2.abs(), bw2, d2_shear, fc, fyt, units)
                        }
                        ConcreteCode::Eurocode2 => {
                            let (_, _, avs, ov) = design_ec2_beam(
                                0.0,
                                st.v2.abs(),
                                bw2,
                                d2_shear,
                                cover_top,
                                fc,
                                fy,
                                fyt,
                                units,
                            );
                            (avs, ov)
                        }
                    };
                    let (av_s_3, over_v3) = if st.v3.abs() > 1e-9 {
                        match code {
                            ConcreteCode::Aci318 => {
                                design_aci_beam_shear(st.v3.abs(), bw3, d3_shear, fc, fyt, units)
                            }
                            ConcreteCode::Eurocode2 => {
                                let (_, _, avs, ov) = design_ec2_beam(
                                    0.0,
                                    st.v3.abs(),
                                    bw3,
                                    d3_shear,
                                    cover_minor,
                                    fc,
                                    fy,
                                    fyt,
                                    units,
                                );
                                (avs, ov)
                            }
                        }
                    } else {
                        (0.0, false)
                    };
                    let av_s = av_s_2.max(av_s_3);
                    max_av_s = max_av_s.max(av_s);
                    any_shear_overstressed = any_shear_overstressed || over_v2 || over_v3;

                    if pmm > max_pmm_ratio || gov_combo == "NONE" {
                        max_pmm_ratio = pmm;
                        gov_combo = combo_name.clone();
                    }
                }
            }

            let status = if buckled {
                "BUCKLING_OVERSTRESSED".to_string()
            } else if max_pmm_ratio > 1.0 || any_shear_overstressed {
                "OVERSTRESSED".to_string()
            } else {
                "OK".to_string()
            };

            results.push((
                frame_name.clone(),
                ConcreteDesignResult {
                    frame_name: frame_name.clone(),
                    section_name: sec.name.clone(),
                    design_code: model.concrete_design_code.clone(),
                    governing_combo: gov_combo,
                    is_column: true,
                    as_top: 0.5 * ast,
                    as_bot: 0.5 * ast,
                    as_side: 0.0,
                    av_over_s: max_av_s,
                    pmm_ratio: max_pmm_ratio,
                    min_rebar_area,
                    status,
                },
            ));
        } else {
            // Concrete Beam Design
            let mut max_as_top = 0.0_f64;
            let mut max_as_bot = 0.0_f64;
            let mut max_as_side = 0.0_f64;
            let mut max_av_s = 0.0_f64;
            let mut any_overstressed = false;
            let mut gov_combo = "NONE".to_string();
            let mut max_demand_metric = -1.0_f64;

            // Compute baseline min_rebar even if no forces exist
            let mut min_rebar = match code {
                ConcreteCode::Aci318 => {
                    design_aci_beam_flexure(0.0, b, d_bot, cover_top, fc, fy, es, units).1
                }
                ConcreteCode::Eurocode2 => {
                    design_ec2_beam(0.0, 0.0, b, d_bot, cover_top, fc, fy, fyt, units).1
                }
            };

            for (combo_name, frame_map) in &combo_forces {
                let Some((_, stations)) = frame_map.iter().find(|(fn_k, _)| fn_k == frame_name)
                else {
                    continue;
                };
                for st in stations {
                    // Positive M3 -> bottom tension; Negative M3 -> top tension
                    let m_pos = st.m3.max(0.0);
                    let m_neg = (-st.m3).max(0.0);
                    let vu2 = st.v2.abs();
                    let vu3 = st.v3.abs();

                    let (as_bot, as_min_b, over_b, av_s_2, over_v2) = match code {
                        ConcreteCode::Aci318 => {
                            let (ab, amin, ob) = design_aci_beam_flexure(
                                m_pos, b, d_bot, cover_top, fc, fy, es, units,
                            );
                            let (avs, ov) =
                                design_aci_beam_shear(vu2, bw2, d2_shear, fc, fyt, units);
                            (ab, amin, ob, avs, ov)
                        }
                        ConcreteCode::Eurocode2 => {
                            let (ab, amin, avs, ov) = design_ec2_beam(
                                m_pos, vu2, b, d_bot, cover_top, fc, fy, fyt, units,
                            );
                            (ab, amin, ov, avs, ov)
                        }
                    };

                    let (as_top, as_min_t, over_t) = match code {
                        ConcreteCode::Aci318 => {
                            design_aci_beam_flexure(m_neg, b, d_top, cover_bot, fc, fy, es, units)
                        }
                        ConcreteCode::Eurocode2 => {
                            let (at, amin, _, ov) = design_ec2_beam(
                                m_neg, 0.0, b, d_top, cover_bot, fc, fy, fyt, units,
                            );
                            (at, amin, ov)
                        }
                    };

                    // Minor-axis flexure M2 produces tension on side faces across width b with depth b and effective width h
                    let (as_side, as_min_s, over_s) = if st.m2.abs() > 1e-12 {
                        match code {
                            ConcreteCode::Aci318 => design_aci_beam_flexure(
                                st.m2.abs(),
                                h,
                                d3_minor,
                                cover_minor,
                                fc,
                                fy,
                                es,
                                units,
                            ),
                            ConcreteCode::Eurocode2 => {
                                let (as_val, amin, _, ov) = design_ec2_beam(
                                    st.m2.abs(),
                                    0.0,
                                    h,
                                    d3_minor,
                                    cover_minor,
                                    fc,
                                    fy,
                                    fyt,
                                    units,
                                );
                                (as_val, amin, ov)
                            }
                        }
                    } else {
                        (0.0, 0.0, false)
                    };

                    let (av_s_3, over_v3) = if vu3 > 1e-9 {
                        match code {
                            ConcreteCode::Aci318 => {
                                design_aci_beam_shear(vu3, bw3, d3_shear, fc, fyt, units)
                            }
                            ConcreteCode::Eurocode2 => {
                                let (_, _, avs3, ov3) = design_ec2_beam(
                                    0.0,
                                    vu3,
                                    bw3,
                                    d3_shear,
                                    cover_minor,
                                    fc,
                                    fy,
                                    fyt,
                                    units,
                                );
                                (avs3, ov3)
                            }
                        }
                    } else {
                        (0.0, false)
                    };

                    let av_s = av_s_2.max(av_s_3);
                    let over_v = over_v2 || over_v3;

                    max_as_bot = max_as_bot.max(as_bot);
                    max_as_top = max_as_top.max(as_top);
                    max_as_side = max_as_side.max(as_side);
                    max_av_s = max_av_s.max(av_s);
                    min_rebar = min_rebar.max(as_min_b).max(as_min_t).max(as_min_s);
                    any_overstressed = any_overstressed || over_b || over_t || over_s || over_v;

                    let metric = as_bot.max(as_top).max(as_side) + av_s * d_bot;
                    if metric > max_demand_metric {
                        max_demand_metric = metric;
                        gov_combo = combo_name.clone();
                    }
                }
            }

            let status = if any_overstressed {
                "OVERSTRESSED".to_string()
            } else {
                "OK".to_string()
            };

            results.push((
                frame_name.clone(),
                ConcreteDesignResult {
                    frame_name: frame_name.clone(),
                    section_name: sec.name.clone(),
                    design_code: model.concrete_design_code.clone(),
                    governing_combo: gov_combo,
                    is_column: false,
                    as_top: max_as_top,
                    as_bot: max_as_bot,
                    as_side: max_as_side,
                    av_over_s: max_av_s,
                    pmm_ratio: 0.0,
                    min_rebar_area: min_rebar,
                    status,
                },
            ));
        }
    }

    model.concrete_design_results.clear();
    for (k, v) in results {
        model.concrete_design_results.insert(k, v);
    }
}
