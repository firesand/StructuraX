use crate::{
    error::validate_demands,
    steel::{collect_design_combo_forces, design_entries, frame_clear_length, interacting},
    DesignError,
};
use sap_model::{
    ConcreteDesignResult, FrameObj, FramePropType, FrameSection, MatType, Material, SapModel,
};
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
/// under factored moment $M_u \ge 0$ per ACI 318-14. The tension steel only: see
/// [`design_aci_beam_flexure_doubly`] for the compression steel of a doubly reinforced section.
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
    let (as_t, _as_c, as_min, over) =
        design_aci_beam_flexure_doubly(mu, b, d, d_prime, fc, fy, es, units);
    (as_t, as_min, over)
}

/// Flexural design of a rectangular beam per ACI 318-14 (CSI manual pp. 3-31 and 3-32) that also
/// returns the compression steel: `(A_s tension, A's compression, A_s,min, overstressed)`.
///
/// When $\phi M_n$ of the singly reinforced section at $a_{max}$ ($c_{max} = 0.375d$, tension
/// strain 0.005) is exceeded: $M_{uc} = \phi C (d - a_{max}/2)$, $C = 0.85 f'_c b a_{max}$,
/// $M_{us} = M_u - M_{uc}$, $A'_s = M_{us}/((f'_s - 0.85 f'_c)(d - d')\phi)$ with
/// $f'_s = E_s \varepsilon_c (c_{max} - d')/c_{max} \le f_y$, $A_{s1} = M_{uc}/(f_y(d - a_{max}/2)\phi)$,
/// $A_{s2} = M_{us}/(f_y(d - d')\phi)$, $A_s = A_{s1} + A_{s2}$. $A_s$ goes on the tension face,
/// $A'_s$ on the compression face. When $f'_s \le 0.85 f'_c$ no compression steel can
/// balance $M_{us}$: the section is overstressed and `A's` is returned as 0.
pub fn design_aci_beam_flexure_doubly(
    mu: f64,
    b: f64,
    d: f64,
    d_prime: f64,
    fc: f64,
    fy: f64,
    es: f64,
    units: Units,
) -> (f64, f64, f64, bool) {
    let to_psi = units.stress_factor_to(Units::LbInF);
    let fc_psi = (fc * to_psi).max(2000.0);
    let fy_psi = (fy * to_psi).max(20000.0);
    let beta1 = compute_whitney_beta1(fc_psi);

    // Minimum flexural reinforcement per ACI 318-14 Section 9.6.1.2
    let rho_min = (3.0 * fc_psi.sqrt() / fy_psi).max(200.0 / fy_psi);
    let as_min = rho_min * b * d;

    if mu <= 1e-12 {
        return (0.0, 0.0, as_min, false);
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
        (as_govern, 0.0, as_min, false)
    } else {
        // Doubly reinforced beam section
        let as1 = (0.85 * fc * a_max * b) / fy.max(1e-12);
        let mu2 = mu - phi_mn_max_single;
        let lever = (d - d_prime).max(0.5 * d);
        let as2 = mu2 / (phi_b * fy * lever).max(1e-12);
        let eps_s_prime = ecu * ((c_max - d_prime) / c_max).max(0.0);
        let fs_prime = (eps_s_prime * es).min(fy);
        let overstressed = fs_prime <= 0.0 || mu2 > 1.5 * phi_mn_max_single;
        let denom = (fs_prime - 0.85 * fc) * lever * phi_b;
        let as_comp = if denom > 0.0 { mu2 / denom } else { 0.0 };
        (as1 + as2, as_comp, as_min, overstressed || denom <= 0.0)
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
    // vc = 2 * sqrt(fc_psi) in psi -> convert back to model stress units
    let vc_stress = (2.0 * fc_psi.sqrt()) / to_psi;
    aci_shear_from_vc(vu, vc_stress * bw * d, bw, d, fc, fyt, units)
}

/// Stirrup demand $A_v/s$ and the overstress flag from a given concrete shear strength `vc`
/// (ACI 318-14 22.5.1, 22.5.10.5.3 and Table 9.6.3.3), shared by the beam and column rules.
fn aci_shear_from_vc(
    vu: f64,
    vc: f64,
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

/// Concrete shear strength $V_c$ of a column section under the axial force `pu_comp`
/// (compression positive), ACI 318-14 22.5.6.1 and 22.5.7.1 (CSI ACI 318-14 manual p. 3-25).
/// With $P_u/A_g$ in psi: compression $V_c = 2\lambda\sqrt{f'_c}(1 + P_u/2000A_g)A_{cv}$, at most
/// $3.5\lambda\sqrt{f'_c}\sqrt{1 + P_u/500A_g}\,A_{cv}$; tension
/// $V_c = 2\lambda\sqrt{f'_c}(1 + P_u/500A_g)A_{cv} \ge 0$. $\lambda = 1$: the model has no
/// lightweight-concrete factor. `bw * d` is $A_{cv}$.
pub fn aci_column_vc(pu_comp: f64, ag: f64, bw: f64, d: f64, fc: f64, units: Units) -> f64 {
    let to_psi = units.stress_factor_to(Units::LbInF);
    let fc_psi = (fc * to_psi).max(2000.0);
    let n_psi = pu_comp * to_psi / ag.max(1e-12);
    let base = 2.0 * fc_psi.sqrt();
    let stress_psi = if n_psi >= 0.0 {
        let cap = 3.5 * fc_psi.sqrt() * (1.0 + n_psi / 500.0).sqrt();
        (base * (1.0 + n_psi / 2000.0)).min(cap)
    } else {
        (base * (1.0 + n_psi / 500.0)).max(0.0)
    };
    stress_psi / to_psi * bw * d
}

/// Required stirrup area per unit spacing and overstress flag for a column in one shear
/// direction (ACI 318-14), with the axial force in $V_c$ (see [`aci_column_vc`]).
pub fn design_aci_column_shear(
    vu: f64,
    pu_comp: f64,
    ag: f64,
    bw: f64,
    d: f64,
    fc: f64,
    fyt: f64,
    units: Units,
) -> (f64, bool) {
    let vc = aci_column_vc(pu_comp, ag, bw, d, fc, units);
    aci_shear_from_vc(vu, vc, bw, d, fc, fyt, units)
}

/// Torsion strength reduction factor $\phi$ (ACI 318-14 Table 21.2.1, CSI manual p. 3-45).
const ACI_PHI_TORSION: f64 = 0.75;

/// Threshold torsion $T_{th}$ of ACI 318-14 Table 22.7.4.1a (CSI manual p. 3-44), with the gross
/// area `acp` and outer perimeter `pcp` of the section, $A_g = A_{cp}$ and $\lambda = 1$:
/// $T_{th} = \lambda\sqrt{f'_c}\,(A_{cp}^2/p_{cp})\sqrt{1 + P_u/(4A_g\lambda\sqrt{f'_c})}$, $f'_c$ in
/// psi, $P_u$ compression positive; the root is taken as 0 when its argument is negative (tension).
/// Returns `(T_th, T_th with P_u = 0)`.
fn aci_torsion_threshold(acp: f64, pcp: f64, pu_comp: f64, fc: f64, units: Units) -> (f64, f64) {
    let to_psi = units.stress_factor_to(Units::LbInF);
    let fc_psi = (fc * to_psi).max(2000.0);
    let sqrt_fc = fc_psi.sqrt() / to_psi;
    let base = sqrt_fc * (acp * acp / pcp.max(1e-12));
    let arg = 1.0 + pu_comp / (4.0 * acp * sqrt_fc).max(1e-12);
    (base * arg.max(0.0).sqrt(), base)
}

/// $\phi T_{th}$ of the rectangle `b` x `h` at $P_u = 0$ (ACI 318-14 Table 22.7.4.1a). With a section
/// that is not rectangular this is the bounding rectangle and serves only as the noise floor (D130).
fn aci_phi_tth0_rect(b: f64, h: f64, fc: f64, units: Units) -> f64 {
    ACI_PHI_TORSION * aci_torsion_threshold(b * h, 2.0 * (b + h), 0.0, fc, units).1
}

/// Result of [`design_aci_beam_torsion`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AciTorsion {
    /// Torsion is ignored: $T_u \le \max(\phi T_{th}, 10^{-6}\phi T_{th,0})$.
    pub ignored: bool,
    /// $A_t/s$ of ACI 318-14 22.7.6.1: one leg of the closed stirrup (manual p. 3-46).
    pub at_over_s: f64,
    /// Longitudinal torsion steel $A_l$: the larger of 22.7.6.1 and the minimum 9.6.4.3 (pp. 3-45, 3-46).
    pub al: f64,
    /// Minimum of $A_v/s + 2A_t/s$ of ACI 9.6.4.2 (p. 3-46); 0 when torsion is ignored.
    pub transverse_min: f64,
    /// The limit on $V_u$ and $T_u$ together is exceeded (p. 3-46).
    pub section_overstressed: bool,
}

impl AciTorsion {
    const IGNORED: AciTorsion = AciTorsion {
        ignored: true,
        at_over_s: 0.0,
        al: 0.0,
        transverse_min: 0.0,
        section_overstressed: false,
    };
}

/// Torsion design of a rectangular beam `b` x `h` (effective depth `d`, $b_w = b$) under ACI 318-14 as
/// the CSI manual prints it (pp. 3-42 to 3-46): `tu` and `vu` are the absolute torsion and V2 shear,
/// `pu_comp` the axial force with compression positive, `fy` and `fys` the longitudinal and stirrup
/// yield stresses. $\phi = 0.75$, $\lambda = 1$, $\theta = 45^\circ$.
///
/// Properties (p. 3-44) with the stirrup centreline $c = 1.75$ in from the surface:
/// $A_{oh} = (b - 2c)(h - 2c)$, $p_h = 2(b - 2c) + 2(h - 2c)$, $A_o = 0.85A_{oh}$; if the stirrup does
/// not fit the call returns `Err`. Regimes: $T_u \le \phi T_{th}$ (or the noise floor
/// $10^{-6}\phi T_{th,0}$, this program's): ignored; $\phi T_{th} < T_u \le \phi T_{cr}$,
/// $T_{cr} = 4T_{th}$: only the minimum reinforcement ($A_t/s = 0$, $A_l = A_{l,min}$); above
/// $\phi T_{cr}$: $A_t/s = T_u/(2\phi A_o f_{ys})$, $A_l = T_up_h/(2\phi A_of_y)$ and the minimums.
/// Following the manual, a torsion between $\phi T_{th}$ and $\phi T_{cr}$ gets minimum
/// reinforcement only, which is less than ACI 22.7.6 asks: this is the vendor's printed choice (D130).
/// $A_{l,min}$ is the lesser of 9.6.4.3a and 9.6.4.3b, at least 0. The transverse minimum of 9.6.4.2
/// is returned in `transverse_min`; the page prints its second term with $f_y$, this uses $f_{ys}$ as
/// ACI 9.6.4.2 and the vendor's shear minimum (p. 3-41) do. The section limit of p. 3-46 is
/// $\sqrt{(V_u/(bd))^2 + (T_up_h/(1.7A_{oh}^2))^2} \le \phi(V_c/(bd) + 8\sqrt{f'_c})$, $V_c = 2\sqrt{f'_c}bd$.
pub fn design_aci_beam_torsion(
    tu: f64,
    vu: f64,
    pu_comp: f64,
    b: f64,
    h: f64,
    d: f64,
    fc: f64,
    fy: f64,
    fys: f64,
    units: Units,
) -> Result<AciTorsion, &'static str> {
    let (tth, tth0) = aci_torsion_threshold(b * h, 2.0 * (b + h), pu_comp, fc, units);
    let phi = ACI_PHI_TORSION;
    if tu <= (phi * tth).max(1e-6 * phi * tth0) {
        return Ok(AciTorsion::IGNORED);
    }
    let c = 1.75 * Units::KipInF.length_factor_to(units);
    if b <= 2.0 * c || h <= 2.0 * c {
        return Err("the closed stirrup the manual assumes at 1.75 in from the surface does not fit");
    }
    let to_psi = units.stress_factor_to(Units::LbInF);
    let fc_psi = (fc * to_psi).max(2000.0);
    let sqrt_fc = fc_psi.sqrt() / to_psi;
    let fys_psi = (fys * to_psi).max(20000.0);
    let acp = b * h;
    let aoh = (b - 2.0 * c) * (h - 2.0 * c);
    let ph = 2.0 * (b - 2.0 * c) + 2.0 * (h - 2.0 * c);
    let ao = 0.85 * aoh;

    let (at, al_req) = if tu <= phi * 4.0 * tth {
        (0.0, 0.0)
    } else {
        (tu / (phi * 2.0 * ao * fys), tu * ph / (phi * 2.0 * ao * fy))
    };
    let al_a = 5.0 * sqrt_fc * acp / fy - at * ph * fys / fy;
    let al_b = 5.0 * sqrt_fc * acp / fy - (25.0 / fys_psi * b) * ph * fys / fy;
    let al_min = al_a.min(al_b).max(0.0);
    let transverse_min = ((0.75 * fc_psi.sqrt()).max(50.0) / fys_psi) * b;

    let vc = 2.0 * sqrt_fc * b * d;
    let lhs = ((vu / (b * d)).powi(2) + (tu * ph / (1.7 * aoh * aoh)).powi(2)).sqrt();
    let rhs = phi * (vc / (b * d) + 8.0 * sqrt_fc);
    Ok(AciTorsion {
        ignored: false,
        at_over_s: at,
        al: al_req.max(al_min),
        transverse_min,
        section_overstressed: lhs > rhs,
    })
}

/// Stirrup demand $A_v/s$ of a beam from strength alone, $(V_u - \phi V_c)/(\phi f_{ys}d)$ when
/// $V_u > \phi V_c$ and 0 otherwise: the shear rule of [`design_aci_beam_shear`] without its
/// minimum, for the combination with torsion (ACI 9.6.4.2 replaces 9.6.3.3; manual pp. 3-42, 3-46).
fn aci_beam_av_strength(vu: f64, bw: f64, d: f64, fc: f64, fyt: f64, units: Units) -> f64 {
    let to_psi = units.stress_factor_to(Units::LbInF);
    let fc_psi = (fc * to_psi).max(2000.0);
    let vc = (2.0 * fc_psi.sqrt()) / to_psi * bw * d;
    let phi_v = 0.75;
    let phi_vc = phi_v * vc;
    if vu <= phi_vc {
        0.0
    } else {
        ((vu - phi_vc) / phi_v) / (fyt * d).max(1e-12)
    }
}

/// Strut inclination $\cot\theta$ of a beam under EN 1992-1-1 6.2.3 as the CSI manual states it
/// (printed pp. 3-32 and 3-33). With seismic in the design combination $\theta = 45^\circ$
/// ($\cot\theta = 1$). Otherwise $\theta$ is optimised from
/// $\cot\theta + \tan\theta = 0.9\,\alpha_{cw}\nu_1 f_{cd}/v_{Ed}$, $v_{Ed} = V_{Ed}/(b_w d)$,
/// $\alpha_{cw} = 1$, $21.8^\circ \le \theta \le 45^\circ$ (the 0.9 is $z/d$: the equation is
/// $V_{Ed} = V_{Rd,max}(\theta)$). With $X = 0.9\nu_1 f_{cd}/v_{Ed}$ the root is
/// $\cot\theta = (X + \sqrt{X^2 - 4})/2$ clamped to $[1, 2.5]$; for $X < 2$ no angle gives
/// $V_{Rd,max} \ge V_{Ed}$ and $\cot\theta = 1$ (the station is then overstressed). The manual's
/// torsion clause ($T_{Ed} > T_{cr}$ gives 45 degrees) is passed in through `seismic`: the caller
/// sets the flag when Eq. 6.31 fails above the noise floor (see [`design_ec2_beam_torsion`]). The v24
/// manual (p. 3-36) prints "or" between that clause and the seismic one; v19 joins them unclearly (D130).
pub fn ec2_beam_strut_cot(vu: f64, b: f64, d: f64, fck: f64, units: Units, seismic: bool) -> f64 {
    if seismic {
        return 1.0;
    }
    let to_mpa = units.stress_factor_to(Units::NMmC);
    let fck_mpa = (fck * to_mpa).max(12.0);
    let fcd_mpa = 0.85 * fck_mpa / 1.5;
    let nu1 = 0.6 * (1.0 - fck_mpa / 250.0).max(0.3);
    let v_ed = vu.abs() * to_mpa / (b * d).max(1e-12);
    if v_ed <= 1e-12 {
        return 2.5;
    }
    let x = 0.9 * nu1 * fcd_mpa / v_ed;
    if x < 2.0 {
        1.0
    } else {
        (0.5 * (x + (x * x - 4.0).sqrt())).clamp(1.0, 2.5)
    }
}

/// $V_{Rd,c}$ of EN 1992-1-1 6.2.2(1) as [`ec2_shear_core`] uses it (the formula is in its doc
/// comment), shared with the torsion rule of [`design_ec2_beam_torsion`] (Eq. 6.31).
fn ec2_v_rd_c(b: f64, d: f64, rho_l: f64, fck: f64, units: Units) -> f64 {
    let to_mpa = units.stress_factor_to(Units::NMmC);
    let to_mm = units.length_factor_to(Units::NMmC);
    let fck_mpa = (fck * to_mpa).max(12.0);
    let gamma_c = 1.50;
    let k = (1.0 + (200.0 / (d * to_mm).max(1e-9)).sqrt()).min(2.0);
    let rho_l = rho_l.clamp(0.0, 0.02);
    let v_min = 0.035 * k.powf(1.5) * fck_mpa.sqrt();
    let v_rd_c_stress = ((0.18 / gamma_c) * k * (100.0 * rho_l * fck_mpa).cbrt()).max(v_min) / to_mpa;
    v_rd_c_stress * b * d
}

/// Shear reinforcement $A_{sw}/s$ and the $V_{Rd,max}$ flag of EN 1992-1-1 6.2.3 for the strut
/// inclination `cot_theta`: $A_{sw}/s = V_{Ed}/(z f_{ywd} \cot\theta)$, $z = 0.9d$,
/// $V_{Rd,max} = b z \nu_1 f_{cd}/(\cot\theta + \tan\theta)$. The strength without shear
/// reinforcement is (6.2.2(1), manual pp. 3-31 and 3-32) $V_{Rd,c} = C_{Rd,c} k (100\rho_l f_{ck})^{1/3} b d$
/// at least $v_{min} b d$, with $C_{Rd,c} = 0.18/1.5$, $k = 1 + \sqrt{200/d} \le 2$ ($d$ in mm),
/// $v_{min} = 0.035 k^{3/2} f_{ck}^{1/2}$ and $\rho_l$ = `rho_l`, at most 0.02 ($\sigma_{cp} = 0$). The
/// minimum ratio is $\rho_{w,\min} = 0.08\sqrt{f_{ck}}/f_{yk}$ with the yield stress `fytk` of the shear
/// reinforcement (EC2 9.5N). Shared by beams and columns.
fn ec2_shear_core(
    vu: f64,
    b: f64,
    d: f64,
    rho_l: f64,
    fck: f64,
    fytk: f64,
    cot_theta: f64,
    units: Units,
) -> (f64, bool) {
    let to_mpa = units.stress_factor_to(Units::NMmC);
    let fck_mpa = (fck * to_mpa).max(12.0);
    let fytk_mpa = (fytk * to_mpa).max(200.0);
    let gamma_c = 1.50;
    let gamma_s = 1.15;
    let fcd = 0.85 * fck / gamma_c;
    let fywd = fytk / gamma_s;

    let z = 0.9 * d;
    let rho_w_min = (0.08 * fck_mpa.sqrt()) / fytk_mpa;
    let asw_s_min = rho_w_min * b;
    let v_rd_c = ec2_v_rd_c(b, d, rho_l, fck, units);
    let nu1 = 0.6 * (1.0 - fck_mpa / 250.0).max(0.3);
    let v_rd_max = (b * z * nu1 * fcd) / (cot_theta + 1.0 / cot_theta);

    let av_s = if vu <= v_rd_c {
        asw_s_min
    } else {
        let req = vu / (z * fywd * cot_theta).max(1e-12);
        req.max(asw_s_min)
    };
    // V_Ed = V_Rd,max holds at the optimised angle: allow for the rounding of that equality
    (av_s, vu > v_rd_max * (1.0 + 1e-9))
}

/// Torsion of a beam under Eurocode 2-2004 as the CSI manual prints it (manual pp. 3-32 to 3-37,
/// v24 pp. 3-36 and 3-41), result of [`design_ec2_beam_torsion`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ec2Torsion {
    /// Torsion is ignored: below the noise floor, or Eq. 6.31 holds (p. 3-36).
    pub ignored: bool,
    /// $A_t/s = V_t/(z f_{ywd}\cot\theta)$, $\theta = 45^\circ$ (p. 3-36).
    pub at_over_s: f64,
    /// $A_{sl} = T_{Ed} u_k \cot\theta/(2A_k f_{yd})$ (Eq. 6.28, p. 3-36).
    pub asl: f64,
    /// Eq. 6.29 is exceeded: $T_{Ed}/T_{Rd,max} + V_{Ed}/V_{Rd,max} > 1$ (p. 3-37).
    pub overstressed: bool,
}

impl Ec2Torsion {
    const IGNORED: Ec2Torsion = Ec2Torsion {
        ignored: true,
        at_over_s: 0.0,
        asl: 0.0,
        overstressed: false,
    };
}

/// Properties of a section for the torsion rules of EN 1992-1-1 6.3.2 (manual pp. 3-34, 3-35):
/// `(t_ef, A_k, u_k, T_Rd,c)` from the gross area `a`, the outer circumference `u`, the width `bk`
/// and depth `hk` the core area is taken from, and the concrete strength `fck`:
/// $t_{ef} = A/u$, $A_k = (b_k - t_{ef})(h_k - t_{ef})$, $u_k = 2(b_k - t_{ef}) + 2(h_k - t_{ef})$,
/// $T_{Rd,c} = f_{ctd} t_{ef} 2A_k$, $f_{ctd} = \alpha_{ct} f_{ctk0.05}/\gamma_c$, $\alpha_{ct} = 1$,
/// $f_{ctk0.05} = 0.7 f_{ctm}$, $f_{ctm} = 0.30 f_{ck}^{2/3}$ (MPa), $\gamma_c = 1.5$ (manual p. 3-36).
fn ec2_torsion_section(
    a: f64,
    u: f64,
    bk: f64,
    hk: f64,
    fck: f64,
    units: Units,
) -> (f64, f64, f64, f64) {
    let to_mpa = units.stress_factor_to(Units::NMmC);
    let fck_mpa = (fck * to_mpa).max(12.0);
    let fctm_mpa = 0.30 * fck_mpa.powf(2.0 / 3.0);
    let fctd_mpa = 1.0 * (0.7 * fctm_mpa) / 1.5;
    let fctd = fctd_mpa / to_mpa;
    let tef = a / u.max(1e-12);
    let ak = (bk - tef) * (hk - tef);
    let uk = 2.0 * (bk - tef) + 2.0 * (hk - tef);
    (tef, ak, uk, fctd * tef * 2.0 * ak)
}

/// $T_{Rd,c}$ of the rectangle `b` x `h` (EN 1992-1-1 6.3.2, manual p. 3-36). With a section that
/// is not rectangular this is the bounding rectangle and serves only as the noise floor (D130).
fn ec2_trd_c_rect(b: f64, h: f64, fck: f64, units: Units) -> f64 {
    ec2_torsion_section(b * h, 2.0 * (b + h), b, h, fck, units).3
}

/// Noise floor of torsion in the EC2 rules: $T_{Ed} \le 10^{-6} T_{Rd,c}$ is a roundoff torsion
/// (D130; this program's, not the manual's). Relative to the section's own strength, so it does not
/// depend on the units.
const TORSION_FLOOR: f64 = 1e-6;

/// Torsion design of a rectangular beam `b` x `h` with effective depth `d` under Eurocode 2-2004 as
/// the CSI manual prints it (manual pp. 3-32 to 3-37; v24 manual pp. 3-36 and 3-41 for the "or" of the
/// strut clause). `t_ed` and `v_ed` are the absolute torsion and V2 shear, `rho_l` the flexural steel
/// ratio the shear design of the same station uses for $V_{Rd,c}$.
///
/// Torsion is ignored when $T_{Ed} \le 10^{-6} T_{Rd,c}$ (the noise floor, this program's) or
/// Eq. 6.31 holds, $T_{Ed}/T_{Rd,c} + V_{Ed}/V_{Rd,c} \le 1$. Otherwise
/// $T_{con} = (1 - V_{Ed}/V_{Rd,c})T_{Rd,c}$, $V_t = 2(h - t_{ef})(T_{Ed} - T_{con})/(2A_k)$,
/// $A_t/s = V_t/(z f_{ywd}\cot\theta)$, $A_{sl} = T_{Ed}u_k\cot\theta/(2A_kf_{yd})$ with $\theta = 45^\circ$,
/// $z = 0.9d$, and Eq. 6.29 with $T_{Rd,max} = 2\nu\alpha_{cw}f_{cd}A_kt_{ef}\sin\theta\cos\theta$
/// ($\alpha_{cw} = 1$) and $V_{Rd,max}$ at 45 degrees. No minimums (p. 3-37).
///
/// D148: with $V_{Ed} > V_{Rd,c}$ the printed $T_{con}$ is negative, so $A_t/s$ does not vanish with
/// the torsion; this is built as printed and has not been checked in the vendor program.
pub fn design_ec2_beam_torsion(
    t_ed: f64,
    v_ed: f64,
    b: f64,
    h: f64,
    d: f64,
    rho_l: f64,
    fck: f64,
    fyk: f64,
    fytk: f64,
    units: Units,
) -> Ec2Torsion {
    let (tef, ak, uk, t_rd_c) = ec2_torsion_section(b * h, 2.0 * (b + h), b, h, fck, units);
    let v_rd_c = ec2_v_rd_c(b, d, rho_l, fck, units);
    if t_ed <= TORSION_FLOOR * t_rd_c || t_ed / t_rd_c + v_ed / v_rd_c <= 1.0 {
        return Ec2Torsion::IGNORED;
    }
    let to_mpa = units.stress_factor_to(Units::NMmC);
    let fck_mpa = (fck * to_mpa).max(12.0);
    let fcd = 0.85 * fck / 1.50;
    let nu1 = 0.6 * (1.0 - fck_mpa / 250.0).max(0.3);
    let z = 0.9 * d;
    let fywd = fytk / 1.15;
    let fyd = fyk / 1.15;
    let t_con = (1.0 - v_ed / v_rd_c) * t_rd_c;
    let v_t = 2.0 * (h - tef) * ((t_ed - t_con) / (2.0 * ak));
    let cot = 1.0;
    let at_over_s = v_t / (z * fywd * cot).max(1e-12);
    let asl = t_ed / (2.0 * ak) * cot * uk / fyd.max(1e-12);
    let v_rd_max = (b * z * nu1 * fcd) / (cot + 1.0 / cot);
    let t_rd_max = 2.0 * nu1 * 1.0 * fcd * ak * tef * 0.5;
    Ec2Torsion {
        ignored: false,
        at_over_s,
        asl,
        overstressed: t_ed / t_rd_max + v_ed / v_rd_max > 1.0 + 1e-9,
    }
}

/// The section of a beam whose torsion the inner step of [`design_ec2_beam_seismic`] weighs: a
/// rectangle is designed, a T beam is only compared with the printed threshold (G85, D144).
#[derive(Debug, Clone, Copy)]
enum Ec2TorsionShape {
    Rect,
    /// Flange width, flange thickness, web width of a T beam.
    T { bf: f64, ds: f64, bw: f64 },
}

/// Shear reinforcement of a column in one direction, EN 1992-1-1 as the CSI manual states it for
/// columns (printed p. 3-21): the strut angle is taken as 45 degrees ("The program assumes the
/// conservative value of 45 degrees"), so $\cot\theta = 1$ for both $V_{Rd,max}$ and
/// $A_{sw}/s$. $\sigma_{cp}$ in $V_{Rd,c}$ is not included (open) and the call has no moment, so
/// $\rho_l = 0$.
pub fn design_ec2_column_shear(
    vu: f64,
    b: f64,
    d: f64,
    fck: f64,
    fytk: f64,
    units: Units,
) -> (f64, bool) {
    ec2_shear_core(vu, b, d, 0.0, fck, fytk, 1.0, units)
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
    let r = design_ec2_beam_doubly(mu, vu, b, d, d_prime, fck, fyk, fytk, units);
    (r.as_tension, r.as_min, r.av_s, r.overstressed)
}

/// Result of [`design_ec2_beam_doubly`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ec2BeamDesign {
    /// Tension steel $A_s$ (on the tension face).
    pub as_tension: f64,
    /// Compression steel $A'_s$ (on the compression face; 0 for a singly reinforced section).
    pub as_comp: f64,
    /// Minimum tension steel $A_{s,\min}$.
    pub as_min: f64,
    /// Shear reinforcement $A_{sw}/s$.
    pub av_s: f64,
    pub overstressed: bool,
}

/// Flexural part of [`design_ec2_beam_doubly`]: `(A_s, A's, overstressed)` for the moment `mu`.
fn ec2_flexure(
    mu: f64,
    b: f64,
    d: f64,
    d_prime: f64,
    fck: f64,
    fyk: f64,
    as_min: f64,
    units: Units,
) -> (f64, f64, bool) {
    let to_mpa = units.stress_factor_to(Units::NMmC);
    let fck_mpa = (fck * to_mpa).max(12.0);
    let gamma_s = 1.15;
    let fyd = fyk / gamma_s;

    // Flexural design (EN 1992-1-1 6.1, K_bal = 0.167 for x/d <= 0.45)
    let k = mu / (b * d * d * fck).max(1e-12);
    let k_bal = 0.167;
    if mu <= 1e-12 {
        (0.0, 0.0, false)
    } else if k <= k_bal {
        let z = d * (0.5 + (0.25 - k / 1.134).max(0.0).sqrt()).min(0.95);
        let as_val = (mu / (fyd * z).max(1e-12)).max(as_min);
        (as_val, 0.0, false)
    } else if fck_mpa <= 50.0 {
        // Doubly reinforced, CSI manual pp. 3-24 to 3-26 (fck <= 50 MPa: lambda = 0.8, eta = 1):
        // m = M/(b d^2 eta fcd), m_lim = lambda (x/d)lim (1 - lambda/2 (x/d)lim),
        // omega' = (m - m_lim)/(1 - d'/d), omega = lambda (x/d)lim + omega',
        // A's = omega' eta fcd b d / f's with f's = Es eps_cu2 (1 - d'/x_lim) <= fyd,
        // As = omega eta fcd b d / fyd. The design strength eta fcd is the code's 0.85 fck/1.5.
        let fcd = 0.85 * fck / 1.5;
        let lambda = 0.8;
        let ecu2 = 0.0035;
        let es = 200_000.0 / to_mpa;
        let x_lim_d = (1.0 - 0.44) / (1.25 * (0.6 + 0.0014 / ecu2));
        let m_lim = lambda * x_lim_d * (1.0 - 0.5 * lambda * x_lim_d);
        let omega_lim = lambda * x_lim_d;
        let m = mu / (b * d * d * fcd).max(1e-12);
        let omega_p = (m - m_lim) / (1.0 - d_prime / d).max(1e-9);
        let omega = omega_lim + omega_p;
        let x_lim = x_lim_d * d;
        let fs_prime = (es * ecu2 * (1.0 - d_prime / x_lim)).min(fyd);
        let as_t = omega * fcd * b * d / fyd;
        // f's <= 0: the compression steel lies at or beyond the neutral axis, no area balances it
        let as_c = if fs_prime > 0.0 {
            omega_p * fcd * b * d / fs_prime
        } else {
            0.0
        };
        (as_t, as_c, k > 2.0 * k_bal || fs_prime <= 0.0)
    } else {
        // fck > 50 MPa: the manual's high-strength parameters are not implemented here; the
        // earlier balanced-moment form is kept for the tension steel and the compression steel
        // balances the excess moment with both steels at fyd (A's = As2).
        let z_bal = d * (0.5 + (0.25 - k_bal / 1.134).sqrt());
        let m_bal = k_bal * b * d * d * fck;
        let as1 = m_bal / (fyd * z_bal);
        let as2 = (mu - m_bal) / (fyd * (d - d_prime).max(0.5 * d));
        (as1 + as2, as2, k > 2.0 * k_bal)
    }
}

/// Flexural and shear design of a rectangular beam under Eurocode 2-2004 (`EN 1992-1-1`), with the
/// compression steel of a doubly reinforced section; the combination has no seismic case (see
/// [`design_ec2_beam_seismic`]).
pub fn design_ec2_beam_doubly(
    mu: f64,
    vu: f64,
    b: f64,
    d: f64,
    d_prime: f64,
    fck: f64,
    fyk: f64,
    fytk: f64,
    units: Units,
) -> Ec2BeamDesign {
    design_ec2_beam_seismic(mu, vu, b, d, d_prime, fck, fyk, fytk, units, false)
}

/// [`design_ec2_beam_doubly`] with the seismic flag of the design combination: seismic fixes
/// $\theta = 45^\circ$, otherwise the strut angle is optimised ([`ec2_beam_strut_cot`]).
pub fn design_ec2_beam_seismic(
    mu: f64,
    vu: f64,
    b: f64,
    d: f64,
    d_prime: f64,
    fck: f64,
    fyk: f64,
    fytk: f64,
    units: Units,
    seismic: bool,
) -> Ec2BeamDesign {
    match ec2_beam_with_torsion(mu, vu, b, d, d_prime, fck, fyk, fytk, units, seismic, None) {
        Ok((design, _)) => design,
        Err(reason) => unreachable!("no torsion given, no refusal: {reason}"),
    }
}

/// [`design_ec2_beam_seismic`] with the torsion `t_ed` of the station and the depth `h` and shape of
/// the section (D130). With torsion that Eq. 6.31 does not let the design ignore, the strut of the
/// shear design is at 45 degrees ([`ec2_beam_strut_cot`]) and the torsion result is returned beside
/// the design; a T beam with such torsion is refused with the reason as text (G85).
fn ec2_beam_with_torsion(
    mu: f64,
    vu: f64,
    b: f64,
    d: f64,
    d_prime: f64,
    fck: f64,
    fyk: f64,
    fytk: f64,
    units: Units,
    seismic: bool,
    torsion: Option<(f64, f64, Ec2TorsionShape)>,
) -> Result<(Ec2BeamDesign, Ec2Torsion), String> {
    let to_mpa = units.stress_factor_to(Units::NMmC);
    let fck_mpa = (fck * to_mpa).max(12.0);
    let fyk_mpa = (fyk * to_mpa).max(200.0);

    // Mean tensile strength fctm = 0.30 * fck^(2/3) MPa (EN 1992-1-1 Table 3.1)
    let fctm_mpa = 0.30 * fck_mpa.powf(2.0 / 3.0);
    let rho_min = (0.26 * fctm_mpa / fyk_mpa).max(0.0013);
    let as_min = rho_min * b * d;

    let (as_req, as_comp, over_flex) = ec2_flexure(mu, b, d, d_prime, fck, fyk, as_min, units);

    // Shear design (EN 1992-1-1 6.2.3): rho_l is the tension steel this call designs (0 without
    // a moment), the strut angle follows the manual.
    let rho_l = as_req / (b * d).max(1e-12);
    let tors = match torsion {
        None => Ec2Torsion::IGNORED,
        Some((t_ed, h, Ec2TorsionShape::Rect)) => {
            design_ec2_beam_torsion(t_ed, vu, b, h, d, rho_l, fck, fyk, fytk, units)
        }
        Some((t_ed, h, Ec2TorsionShape::T { bf, ds, bw })) => {
            // Printed T-beam properties (manual p. 3-35); VRd,c is that of the rectangle the
            // shear design uses (D144).
            let (_, _, _, t_rd_c) = ec2_torsion_section(
                bw * h + (bf - bw) * ds,
                2.0 * bf + 2.0 * h,
                bf,
                h,
                fck,
                units,
            );
            let v_rd_c = ec2_v_rd_c(b, d, rho_l, fck, units);
            let sum = t_ed / t_rd_c + vu / v_rd_c;
            // the noise floor is that of the bounding rectangle b x h (D130)
            if t_ed <= TORSION_FLOOR * ec2_trd_c_rect(b, h, fck, units) || sum <= 1.0 {
                Ec2Torsion::IGNORED
            } else {
                return Err(format!(
                    "TEd = {t_ed} is above what Eq. 6.31 lets the design ignore \
                     (TEd/TRd,c + VEd/VRd,c = {sum} with TRd,c = {t_rd_c} and VRd,c = {v_rd_c}) \
                     of the T beam; torsion design is implemented for rectangular beams only \
                     (section type T); G85"
                ));
            }
        }
    };
    // Torsion that is not ignored fixes the strut at 45 degrees, as a seismic state does (the v24
    // manual p. 3-36 prints "or" between the two clauses).
    let cot_theta = ec2_beam_strut_cot(vu, b, d, fck, units, seismic || !tors.ignored);
    let (av_s, over_shear) = ec2_shear_core(vu, b, d, rho_l, fck, fytk, cot_theta, units);
    Ok((
        Ec2BeamDesign {
            as_tension: as_req,
            as_comp,
            as_min,
            av_s,
            overstressed: over_flex || over_shear || tors.overstressed,
        },
        tors,
    ))
}

/// Point on a uniaxial factored P-M interaction curve $(\phi P_n, \phi M_n)$.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PmCurvePoint {
    pub phi_pn: f64,
    pub phi_mn: f64,
}

/// Generates a 1D rebar layer distribution `(depth_from_top, bar_area)` for a concrete column
/// bent about either the major (3-axis, depth $h = t_3$, width $b = t_2$) or minor (2-axis, depth $b = t_2$, width $h = t_3$).
/// The first and the last output station of a member's state, its ends for design; none for
/// a list of fewer than two stations (a single station is not the end of a member).
fn state_ends(
    state: &[sap_model::FrameStationResult],
) -> (
    Option<&sap_model::FrameStationResult>,
    Option<&sap_model::FrameStationResult>,
) {
    if state.len() < 2 {
        return (None, None);
    }
    (state.first(), state.last())
}

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
    evaluate_pmm_ratio(
        pu_comp_positive,
        mu3,
        mu2,
        curve_major,
        curve_minor,
        phi_pn_max,
        phi_pnt,
        h3,
        h2,
        true,
    )
}

/// Minimum eccentricity of an ACI 318-14 column in the direction of the dimension `h`:
/// $e_{min} = 0.6 + 0.03h$ inches, $h$ in inches (ACI 318-14 6.6.4.5.4; CSI manual p. 3-13),
/// returned in model length units.
pub fn aci_min_eccentricity(h: f64, units: Units) -> f64 {
    let h_in = h * units.length_factor_to(Units::KipInF);
    (0.6 + 0.03 * h_in) * Units::KipInF.length_factor_to(units)
}

/// The P-M-M contour ratio of [`evaluate_column_biaxial_pmm_ratio`]; `old_min_ecc` keeps the
/// earlier 0.05 h minimum-eccentricity rule for pure axial load (used for Eurocode 2). ACI 318-14
/// applies its minimum eccentricity to the moments before the magnifiers instead.
fn evaluate_pmm_ratio(
    pu_comp_positive: f64,
    mu3: f64,
    mu2: f64,
    curve_major: &[PmCurvePoint],
    curve_minor: &[PmCurvePoint],
    phi_pn_max: f64,
    phi_pnt: f64,
    h3: f64,
    h2: f64,
    old_min_ecc: bool,
) -> f64 {
    let mu3 = mu3.abs();
    let mu2 = mu2.abs();

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

    // Earlier minimum-eccentricity rule (0.05 h, pure axial compression only); see `old_min_ecc`
    let e_min_ratio = if old_min_ecc && pu_comp_positive > 0.0 && mu3 < 1e-9 && mu2 < 1e-9 {
        let m_min3 = pu_comp_positive * 0.05 * h3;
        let m_min2 = pu_comp_positive * 0.05 * h2;
        (m_min3 / phi_mn30).max(m_min2 / phi_mn20)
    } else {
        0.0
    };

    axial_ratio.max(contour_ratio).max(e_min_ratio)
}

/// Euler buckling load $P_c = \pi^2 EI_{eff}/l^2$ of a column about one axis with
/// $EI_{eff} = 0.4 E_c I_g/(1 + \beta_{dns})$, $\beta_{dns} = 0.4$ (the code's fixed value),
/// $k = 1$ (ACI 318-14 6.6.4.4.4 and 6.6.4.4.2; CSI manual p. 3-15). `i` is the moment of
/// inertia for that axis: $I_{33}$ for bending about the 3-axis, $I_{22}$ for the 2-axis.
pub fn aci_column_pc(e: f64, i: f64, l_clear: f64) -> f64 {
    if l_clear > 1e-6 {
        let ei_eff = 0.40 * e * i / 1.4;
        (PI * PI * ei_eff) / (l_clear * l_clear)
    } else {
        1e18
    }
}

/// Non-sway moment magnifier $\delta_{ns} = 1/(1 - P_u/(0.75 P_c)) \ge 1$ for one axis
/// (ACI 318-14 6.6.4.5.2 with $C_m = 1$). Valid for $P_u < 0.75 P_c$; the caller declares the
/// failure condition of the manual ($P_u \ge 0.75 P_c$) before.
pub fn aci_moment_magnifier(pu_comp: f64, pc: f64) -> f64 {
    (1.0 / (1.0 - pu_comp / (0.75 * pc))).max(1.0)
}

/// Whether the design combination (or case, when the design runs on cases) includes seismic,
/// through any nesting of combinations: a response-spectrum case, or a case that loads a pattern of
/// type Quake (the classification of `default_combos.rs`). Time-history and other cases do not count.
pub fn combo_has_seismic(model: &SapModel, name: &str) -> bool {
    fn walk(model: &SapModel, name: &str, depth: usize) -> bool {
        if depth > 32 {
            return false;
        }
        if let Some(combo) = model.load_combos.get(name) {
            return combo
                .cases
                .iter()
                .any(|(member, _)| walk(model, member, depth + 1));
        }
        let Some(case) = model.load_cases.get(name) else {
            return false;
        };
        if case.case_type == sap_model::LoadCaseType::ResponseSpectrum {
            return true;
        }
        case.loads.iter().any(|assignment| {
            model
                .load_patterns
                .get(&assignment.load_name)
                .is_some_and(|p| p.pat_type == sap_model::LoadPatternType::Quake)
        })
    }
    walk(model, name, 0)
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
/// Strict: the first refused member returns its typed error and leaves both maps empty.
pub fn try_run_concrete_design(model: &mut SapModel) -> Result<(), DesignError> {
    run_concrete_design_core(model, true).map(|_| ())
}

/// D127: the concrete design that the applications call. A member the design refuses (a missing
/// rebar yield stress, a torsion that is not designed, ...) is recorded (typed in the report, as
/// text in `model.concrete_design_errors`) and the run goes on with the other members; a run
/// error returns `Err` and leaves both maps empty. Use `try_run_concrete_design` for the strict
/// form that stops at the first refused member.
pub fn run_concrete_design_per_member(
    model: &mut SapModel,
) -> Result<crate::DesignRunReport, DesignError> {
    run_concrete_design_core(model, false)
}

fn run_concrete_design_core(
    model: &mut SapModel,
    strict: bool,
) -> Result<crate::DesignRunReport, DesignError> {
    model.concrete_design_results.clear();
    model.concrete_design_errors.clear();
    let code = ConcreteCode::try_from_code_str(&model.concrete_design_code)?;
    crate::steel::refuse_unavailable_combos(model, "concrete")?;
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
            // A cable is not a design member (D103)
            if frame.cable_params.is_some() {
                return false;
            }
            let Some(section) = model.frame_sections.get(&frame.section) else {
                return false;
            };
            model
                .materials
                .get(&section.material)
                .is_some_and(|material| material.mat_type == MatType::Concrete)
        },
    )?;
    if strict {
        check_rebar_yield(model)?;
    }
    run_concrete_design_validated(model, code, combo_forces, strict)
}

/// The longitudinal rebar yield stress of a concrete member: that of the
/// rebar material the section names when it is positive, else the concrete
/// material's own `fy`. `None` when neither is positive (no value is assumed).
fn resolve_rebar_fy(model: &SapModel, sec: &FrameSection, mat: &Material) -> Option<f64> {
    let named = sec
        .concrete_rebar
        .as_ref()
        .and_then(|cfg| model.materials.get(&cfg.rebar_mat_long))
        .map(|rmat| rmat.fy)
        .filter(|fy| *fy > 0.0);
    named.or((mat.fy > 0.0).then_some(mat.fy))
}

/// Refuses a concrete member whose rebar yield stress is not given anywhere
/// (a library concrete carries no `fy`; the section must name a rebar
/// material with a positive yield stress). Earlier versions assumed 60 ksi.
fn check_rebar_yield(model: &SapModel) -> Result<(), DesignError> {
    for (frame_name, frame) in &model.frames {
        if frame.cable_params.is_some() {
            continue;
        }
        let Some(sec) = model.frame_sections.get(&frame.section) else {
            continue;
        };
        let Some(mat) = model.materials.get(&sec.material) else {
            continue;
        };
        if mat.mat_type != MatType::Concrete || resolve_rebar_fy(model, sec, mat).is_some() {
            continue;
        }
        return Err(rebar_yield_error(frame_name, sec, mat));
    }
    Ok(())
}

/// The `MissingRebarYield` error of a concrete member: the material that lacks the yield stress
/// is the named rebar material, else the concrete.
fn rebar_yield_error(
    frame_name: &str,
    sec: &FrameSection,
    mat: &Material,
) -> DesignError {
    let material = sec
        .concrete_rebar
        .as_ref()
        .map_or_else(|| mat.name.clone(), |cfg| cfg.rebar_mat_long.clone());
    DesignError::MissingRebarYield {
        frame: frame_name.to_string(),
        section: sec.name.clone(),
        material,
    }
}

/// A refused beam torsion (D130, G85) or a member without a rebar yield stress is recorded with
/// its reason and the run goes on with the other members (D127); the refused member has no result
/// at all. With `strict` the first refused member returns its typed error instead and nothing is
/// stored (`check_rebar_yield` has then already run over all members, as before).
fn run_concrete_design_validated(
    model: &mut SapModel,
    code: ConcreteCode,
    combo_forces: crate::steel::ComboForces,
    strict: bool,
) -> Result<crate::DesignRunReport, DesignError> {
    let units = model.present_units;

    let mut results = Vec::new();
    let mut not_designed: Vec<(String, DesignError)> = Vec::new();

    // Standard steel modulus Es = 29,000 ksi converted to present units
    let es_default = 29_000.0 * Units::KipInF.stress_factor_to(units);
    let default_cover = 2.5 * Units::KipInF.length_factor_to(units);

    // The design of one member: its result, or the error that refuses it (a refused beam leaves
    // no half-accumulated result); `None` for a frame that is not a concrete design member.
    let design_member = |frame_name: &String,
                         frame: &FrameObj|
     -> Result<Option<ConcreteDesignResult>, DesignError> {
        // A cable is not designed (D103): SAP2000 designs no cable members.
        if frame.cable_params.is_some() {
            return Ok(None);
        }
        let Some(sec) = model.frame_sections.get(&frame.section) else {
            return Ok(None);
        };
        let Some(mat) = model.materials.get(&sec.material) else {
            return Ok(None);
        };
        if mat.mat_type != MatType::Concrete {
            return Ok(None);
        }

        // Resolve longitudinal and confinement rebar yield stresses fy, fyt.
        // A member without a yield stress is refused (D127), so no default value is ever
        // assumed here.
        let Some(fy) = resolve_rebar_fy(model, sec, mat) else {
            return Err(rebar_yield_error(frame_name, sec, mat));
        };
        let mut fyt = if mat.fys > 0.0 { mat.fys } else { fy };
        let mut es = es_default;

        if let Some(cfg) = &sec.concrete_rebar {
            if let Some(rmat) = model.materials.get(&cfg.rebar_mat_long) {
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
            // Pc and delta_ns are evaluated per axis (manual p. 3-15): I33 for bending about 3,
            // I22 for bending about 2. The same unbraced length and k = 1 serve both axes.
            let pc33 = aci_column_pc(mat.e, sec.i33, l_clear);
            let pc22 = aci_column_pc(mat.e, sec.i22, l_clear);
            let pc_min = pc33.min(pc22);

            let mut buckled = false;
            // Every state of a combination is a design entry of its own; a multi-valued
            // combination adds the sub-combinations of the maxima and minima of P, M2 and M3,
            // the forces the P-M-M check combines (ACI 318-14 manual, section 2.1: "...multiple
            // sub combinations using maxima/minima permutations of interacting quantities").
            // The shears are checked alone (the state entries reach their largest absolute
            // value), so they are not permuted. See `design_entries`.
            let column_mask = interacting::P | interacting::M2 | interacting::M3;
            for (combo_name, stations) in combo_forces
                .iter()
                .filter_map(|(c, fm)| {
                    fm.iter()
                        .find(|(fn_k, _)| fn_k == frame_name)
                        .map(|(_, s)| (c, s))
                })
                .flat_map(|(c, s)| {
                    design_entries(s, column_mask)
                        .into_iter()
                        .map(move |e| (c, e))
                })
            {
                for st in stations.iter() {
                    // SAP2000 sign convention: st.p < 0 is compression, st.p > 0 is tension
                    let pu_comp = -st.p;
                    if kl_over_r > 34.0 && pu_comp > 0.0 && pu_comp >= 0.75 * pc_min {
                        buckled = true;
                        let buck_ratio = (pu_comp / (0.75 * pc_min.max(1e-12))).max(999.0);
                        if buck_ratio > max_pmm_ratio {
                            max_pmm_ratio = buck_ratio;
                            gov_combo = combo_name.clone();
                        }
                        continue;
                    }
                    let (delta33, delta22) = if kl_over_r > 34.0 && pu_comp > 0.0 {
                        (
                            aci_moment_magnifier(pu_comp, pc33),
                            aci_moment_magnifier(pu_comp, pc22),
                        )
                    } else {
                        (1.0, 1.0)
                    };
                    let pmm = match code {
                        ConcreteCode::Aci318 => {
                            // Minimum eccentricity (0.6 + 0.03h) in, applied in one direction
                            // at a time and then amplified (manual p. 3-13): two moment pairs.
                            let (m3_abs, m2_abs) = (st.m3.abs(), st.m2.abs());
                            let (m3_min, m2_min) = if pu_comp > 0.0 {
                                (
                                    pu_comp * aci_min_eccentricity(h, units),
                                    pu_comp * aci_min_eccentricity(b, units),
                                )
                            } else {
                                (0.0, 0.0)
                            };
                            [
                                (m3_abs.max(m3_min), m2_abs),
                                (m3_abs, m2_abs.max(m2_min)),
                            ]
                            .into_iter()
                            .map(|(m3p, m2p)| {
                                evaluate_pmm_ratio(
                                    pu_comp,
                                    m3p * delta33,
                                    m2p * delta22,
                                    &curve_major,
                                    &curve_minor,
                                    phi_pn_max,
                                    phi_pnt,
                                    h,
                                    b,
                                    false,
                                )
                            })
                            .fold(0.0_f64, f64::max)
                        }
                        ConcreteCode::Eurocode2 => evaluate_pmm_ratio(
                            pu_comp,
                            st.m3.abs() * delta33,
                            st.m2.abs() * delta22,
                            &curve_major,
                            &curve_minor,
                            phi_pn_max,
                            phi_pnt,
                            h,
                            b,
                            true,
                        ),
                    };
                    let (av_s_2, over_v2) = match code {
                        ConcreteCode::Aci318 => {
                            design_aci_column_shear(
                                st.v2.abs(),
                                pu_comp,
                                sec.area,
                                bw2,
                                d2_shear,
                                fc,
                                fyt,
                                units,
                            )
                        }
                        ConcreteCode::Eurocode2 => {
                            design_ec2_column_shear(
                                st.v2.abs(),
                                bw2,
                                d2_shear,
                                fc,
                                fyt,
                                units,
                            )
                        }
                    };
                    let (av_s_3, over_v3) = if st.v3.abs() > 1e-9 {
                        match code {
                            ConcreteCode::Aci318 => {
                                design_aci_column_shear(
                                    st.v3.abs(),
                                    pu_comp,
                                    sec.area,
                                    bw3,
                                    d3_shear,
                                    fc,
                                    fyt,
                                    units,
                                )
                            }
                            ConcreteCode::Eurocode2 => {
                                design_ec2_column_shear(
                                    st.v3.abs(),
                                    bw3,
                                    d3_shear,
                                    fc,
                                    fyt,
                                    units,
                                )
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

            // The vendor designs torsion for beams only (ACI 318-14 manual p. 3-28, EC2 manual
            // the same split): a column with torsion above the noise floor gets a note (D130).
            let t_max = combo_forces
                .iter()
                .filter_map(|(_, fm)| fm.iter().find(|(fn_k, _)| fn_k == frame_name))
                .flat_map(|(_, states)| states.iter().flatten())
                .map(|st| st.t.abs())
                .fold(0.0_f64, f64::max);
            let (t_floor, manual) = match code {
                ConcreteCode::Aci318 => (
                    1e-6 * aci_phi_tth0_rect(b, h, fc, units),
                    "ACI 318-14 manual",
                ),
                ConcreteCode::Eurocode2 => (
                    TORSION_FLOOR * ec2_trd_c_rect(b, h, fc, units),
                    "Eurocode 2-2004 manual",
                ),
            };
            let col_notes = if t_max > t_floor {
                vec![format!(
                    "torsion T is not designed for columns ({manual}: columns are designed for P-M-M and shear, torsion for beams only, 3.5)"
                )]
            } else {
                Vec::new()
            };

            Ok(Some(
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
                    at_over_s: 0.0,
                    al_torsion: 0.0,
                    notes: col_notes,
                    pmm_ratio: max_pmm_ratio,
                    min_rebar_area,
                    status,
                },
            ))
        } else {
            // Concrete Beam Design
            let mut max_as_top = 0.0_f64;
            let mut max_as_bot = 0.0_f64;
            let mut max_as_side = 0.0_f64;
            let mut max_av_s = 0.0_f64;
            let mut max_at_s = 0.0_f64;
            let mut max_al_t = 0.0_f64;
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

            // The largest moment of this beam over its stations, states and combinations: a
            // minor-axis moment below 1e-9 of it is numerical noise, not a demand (the threshold
            // is relative, so it does not depend on the force and length units).
            let moment_scale = combo_forces
                .iter()
                .filter_map(|(_, fm)| fm.iter().find(|(fn_k, _)| fn_k == frame_name))
                .flat_map(|(_, states)| states.iter().flatten())
                .map(|st| st.m2.abs().max(st.m3.abs()))
                .fold(0.0_f64, f64::max);
            let m2_threshold = 1e-9 * moment_scale;

            // Beam checks: flexure about 3 (top and bottom steel) from M3 alone, side flexure
            // from M2 alone, shear from V2 and V3 alone; no check combines two forces, so no
            // maxima/minima permutation is formed (ACI 318-14 manual section 2.1 permutes the
            // "interacting quantities"; P is not used in beam design) and each state is
            // designed as its own entry, where its extreme M3, M2, V2 and V3 occur.
            for (combo_name, state, st) in combo_forces
                .iter()
                .filter_map(|(c, fm)| {
                    fm.iter()
                        .find(|(fn_k, _)| fn_k == frame_name)
                        .map(|(_, s)| (c, s))
                })
                .flat_map(|(c, s)| {
                    s.iter()
                        .flat_map(move |state| state.iter().map(move |st| (c, state, st)))
                })
            {
                // Moment-strength floors of the framing type (ACI only): the
                // positive strength at each end is at least a fraction of
                // the negative strength there, and both strengths at every
                // section are at least a fraction of the largest end
                // strength (ACI 318-14 18.6.3.2 special, 18.4.2.2
                // intermediate). They act on the design moments, as in
                // SAP2000, so the minimum-steel rules apply afterwards.
                let (end_fraction, span_fraction) = match (code, frame.concrete_framing) {
                    (ConcreteCode::Aci318, sap_model::ConcreteFramingType::SwaySpecial) => (0.5, 0.25),
                    (ConcreteCode::Aci318, sap_model::ConcreteFramingType::SwayIntermediate) => {
                        (1.0 / 3.0, 0.2)
                    }
                    _ => (0.0, 0.0),
                };
                let end_positive = |st: &sap_model::FrameStationResult| {
                    st.m3.max(0.0).max(end_fraction * (-st.m3).max(0.0))
                };
                // The ends of the member for design are its first and its last output station.
                // Without end offsets these are at the joints (relative position 0 and 1); with
                // end offsets they are the faces of the supports, at `ioff` and `L - joff`, and
                // no station lies at 0 or 1. The manual takes the end moments "at any end
                // (support) of the beam" and from "any of the beam end (support) stations"
                // (CSI Concrete Frame Design ACI 318-14, section 3.5.1, special and
                // intermediate moment frames, ACI 18.6.3.2, 18.4.2.2).
                let (first_end, last_end) = state_ends(state);
                let is_end = |st: &sap_model::FrameStationResult| {
                    st.rel_dist <= 1e-6
                        || st.rel_dist >= 1.0 - 1e-6
                        || first_end.is_some_and(|e| std::ptr::eq(st, e))
                        || last_end.is_some_and(|e| std::ptr::eq(st, e))
                };
                // The strength floors read the end moments of the state itself (each state is a
                // design entry of its own; a floor from the end of another state would pair
                // the forces of two states, and cannot change the largest steel areas).
                let largest_end = state
                    .iter()
                    .filter(|st| is_end(st))
                    .map(|st| end_positive(st).max((-st.m3).max(0.0)))
                    .fold(0.0_f64, f64::max);
                {
                    // The seismic state of the combination fixes the EC2 strut angle at 45 degrees.
                    let seismic = combo_has_seismic(model, combo_name);
                    // Positive M3 -> bottom tension; Negative M3 -> top tension
                    let at_end = is_end(st);
                    let m_pos = if at_end { end_positive(st) } else { st.m3.max(0.0) }
                        .max(span_fraction * largest_end);
                    let m_neg = (-st.m3).max(0.0).max(span_fraction * largest_end);
                    let vu2 = st.v2.abs();
                    let vu3 = st.v3.abs();

                    // Torsion (D130). Only a rectangular beam is designed for it; a T beam is
                    // designed when the torsion is below the printed "may ignore" threshold and
                    // refused above it, any other shape when it is above the noise floor (G85).
                    let tu = st.t.abs();
                    let pu_comp = -st.p;
                    let refuse = |reason: String| DesignError::ConcreteTorsionNotDesigned {
                        frame: frame_name.clone(),
                        combo: combo_name.clone(),
                        section: sec.name.clone(),
                        reason,
                    };
                    let is_rect = sec.prop_type == FramePropType::Rectangular;
                    let is_tee = sec.prop_type == FramePropType::T;
                    if tu > 0.0 && !is_rect && !is_tee {
                        let floor = match code {
                            ConcreteCode::Aci318 => 1e-6 * aci_phi_tth0_rect(b, h, fc, units),
                            ConcreteCode::Eurocode2 => {
                                TORSION_FLOOR * ec2_trd_c_rect(b, h, fc, units)
                            }
                        };
                        if tu > floor {
                            return Err(refuse(format!(
                                "T = {tu} is above the noise floor {floor} of the bounding rectangle; torsion design is implemented for rectangular beams only (section type {:?}); G85",
                                sec.prop_type
                            )));
                        }
                    }
                    if tu > 0.0 && is_tee && code == ConcreteCode::Aci318 {
                        // Printed T-beam properties (manual p. 3-44), Ag = Acp
                        let (bf, ds, bw_t) = (sec.t2, sec.tf, sec.tw);
                        let acp_t = bw_t * h + (bf - bw_t) * ds;
                        let (tth_t, _) =
                            aci_torsion_threshold(acp_t, 2.0 * bf + 2.0 * h, pu_comp, fc, units);
                        // the noise floor is that of the bounding rectangle t2 x t3 (D130)
                        let limit = (ACI_PHI_TORSION * tth_t)
                            .max(1e-6 * aci_phi_tth0_rect(b, h, fc, units));
                        if tu > limit {
                            return Err(refuse(format!(
                                "T = {tu} is above phi Tth = {limit} of the T beam; torsion design is implemented for rectangular beams only (section type T); G85"
                            )));
                        }
                    }
                    let ec2_torsion_shape = if is_rect {
                        Some(Ec2TorsionShape::Rect)
                    } else if is_tee {
                        Some(Ec2TorsionShape::T { bf: sec.t2, ds: sec.tf, bw: sec.tw })
                    } else {
                        None
                    };
                    let mut st_at = 0.0_f64;
                    let mut st_al = 0.0_f64;

                    // A doubly reinforced section also needs compression steel: A's of the
                    // positive moment goes to the top face, A's of the negative moment to the
                    // bottom face (ACI 318-14 manual p. 3-32; EC2 manual p. 3-26).
                    let (as_bot_t, as_top_c, as_min_b, over_b, av_s_2, over_v2) = match code {
                        ConcreteCode::Aci318 => {
                            let (ab, ac, amin, ob) = design_aci_beam_flexure_doubly(
                                m_pos, b, d_bot, cover_top, fc, fy, es, units,
                            );
                            let (mut avs, mut ov) =
                                design_aci_beam_shear(vu2, bw2, d2_shear, fc, fyt, units);
                            if is_rect && tu > 0.0 {
                                let tor = design_aci_beam_torsion(
                                    tu, vu2, pu_comp, b, h, d2_shear, fc, fy, fyt, units,
                                )
                                .map_err(|reason| refuse(reason.to_string()))?;
                                if !tor.ignored {
                                    // ACI 9.6.4.2 replaces the shear minimum 9.6.3.3 (manual p. 3-46)
                                    let strength =
                                        aci_beam_av_strength(vu2, bw2, d2_shear, fc, fyt, units);
                                    avs = strength
                                        .max(tor.transverse_min - 2.0 * tor.at_over_s)
                                        .max(0.0);
                                    ov = ov || tor.section_overstressed;
                                    st_at = tor.at_over_s;
                                    st_al = tor.al;
                                }
                            }
                            (ab, ac, amin, ob, avs, ov)
                        }
                        ConcreteCode::Eurocode2 => {
                            let torsion = match (ec2_torsion_shape, tu > 0.0) {
                                (Some(shape), true) => Some((tu, h, shape)),
                                _ => None,
                            };
                            let (r, tor) = ec2_beam_with_torsion(
                                m_pos, vu2, b, d_bot, cover_top, fc, fy, fyt, units, seismic,
                                torsion,
                            )
                            .map_err(refuse)?;
                            st_at = tor.at_over_s;
                            st_al = tor.asl;
                            (
                                r.as_tension,
                                r.as_comp,
                                r.as_min,
                                r.overstressed,
                                r.av_s,
                                r.overstressed,
                            )
                        }
                    };

                    let (as_top_t, as_bot_c, as_min_t, over_t) = match code {
                        ConcreteCode::Aci318 => design_aci_beam_flexure_doubly(
                            m_neg, b, d_top, cover_bot, fc, fy, es, units,
                        ),
                        ConcreteCode::Eurocode2 => {
                            let r = design_ec2_beam_doubly(
                                m_neg, 0.0, b, d_top, cover_bot, fc, fy, fyt, units,
                            );
                            (r.as_tension, r.as_comp, r.as_min, r.overstressed)
                        }
                    };
                    let as_bot = as_bot_t.max(as_bot_c);
                    let as_top = as_top_t.max(as_top_c);

                    // Minor-axis flexure M2 produces tension on side faces across width b with depth b and effective width h
                    let (as_side, as_min_s, over_s) = if st.m2.abs() > m2_threshold.max(1e-300) {
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
                                let r = design_ec2_beam_seismic(
                                    0.0,
                                    vu3,
                                    bw3,
                                    d3_shear,
                                    cover_minor,
                                    fc,
                                    fy,
                                    fyt,
                                    units,
                                    seismic,
                                );
                                (r.av_s, r.overstressed)
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
                    max_at_s = max_at_s.max(st_at);
                    max_al_t = max_al_t.max(st_al);
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

            Ok(Some(
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
                    at_over_s: max_at_s,
                    al_torsion: max_al_t,
                    notes: Vec::new(),
                    pmm_ratio: 0.0,
                    min_rebar_area: min_rebar,
                    status,
                },
            ))
        }
    };

    for (frame_name, frame) in &model.frames {
        match design_member(frame_name, frame) {
            Ok(Some(result)) => results.push((frame_name.clone(), result)),
            Ok(None) => {}
            Err(e) if !strict && e.is_member_refusal() => {
                not_designed.push((frame_name.clone(), e));
            }
            Err(e) => return Err(e),
        }
    }

    model.concrete_design_results.clear();
    model.concrete_design_errors.clear();
    let designed = results.len();
    for (k, v) in results {
        model.concrete_design_results.insert(k, v);
    }
    for (frame_name, error) in &not_designed {
        model
            .concrete_design_errors
            .insert(frame_name.clone(), error.to_string());
    }
    Ok(crate::DesignRunReport {
        designed,
        not_designed,
    })
}
