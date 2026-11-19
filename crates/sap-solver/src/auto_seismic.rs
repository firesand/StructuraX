//! Automatic seismic lateral loads, 2012 IBC / ASCE 7-10.
//!
//! Source: CSI Lateral Loads Manual, section 2.14 (pp. 2-58 to 2-63) with the
//! common rules of sections 2.2.2 to 2.2.4:
//!
//! ```text
//! SDS = 2/3 Fa Ss,  SD1 = 2/3 Fv S1                      (11.4-1 to 11.4-4)
//! Ta  = Ct hn^x                                          (12.8-7)
//! Cs  = SDS / (R/Ie)                                     (12.8-2)
//!   not above SD1 / (T R/Ie)            for T <= TL      (12.8-3)
//!   not above SD1 TL / (T^2 R/Ie)       for T >  TL      (12.8-4)
//!   not below max(0.044 SDS Ie, 0.01)                    (12.8-5)
//!   not below 0.5 S1 / (R/Ie)           when S1 >= 0.6   (12.8-6)
//! V   = Cs W                                             (12.8-1)
//! Fx  = V w h^k / sum(w h^k)                             (12.8-11, 12.8-12)
//! k   = 1 for T <= 0.5, 2 for T >= 2.5, linear between
//! ```
//!
//! The manual distributes the base shear to story levels and then to the
//! joints of a level in proportion to their mass (section 2.2.2). That is the
//! same as the joint formula above, because joints of one level share `h`.
//!
//! What the manual does not print, and what is decided here:
//!
//! * `W` is "the weight of the building (based on specified mass)". The exact
//!   set of joints is not given. `W` here is the sum of `m * g` over the joints
//!   with `z_min < z <= z_max` (within a relative tolerance of 1e-9), where
//!   `m` is the assembled translational mass in the load direction (mass
//!   source included, restraints not applied). Mass at or below the base level
//!   is not counted and receives no force; mass above `z_max` receives no
//!   force and is not counted either.
//! * `hn` is `z_max - z_min` in feet, and `h` of a joint is `z - z_min`.
//! * The occupancy factor `Ie`, and `Fa`, `Fv` are entered directly; the
//!   vendor derives them from the occupancy category and site class
//!   (Tables 1.5-2, 11.4-1, 11.4-2), which are not implemented.
//! * Eccentricity (section 2.2.3): the vendor ignores it where there is no
//!   diaphragm. For each diaphragm constraint (type Diaphragm, axis Z) that
//!   carries load, the width is the maximum minus the minimum coordinate
//!   perpendicular to the load over all its joints, and a moment of
//!   `ratio * width * F` is applied, `F` being the total lateral force of the
//!   pattern on the diaphragm's joints; an overwrite distance replaces
//!   `ratio * width`. The manual applies it "about the diaphragm center of
//!   mass": a couple, so it is added as a global moment about Z at the first
//!   joint of the diaphragm whose rotation about Z is free (the diaphragm ties
//!   the rotations). **The sign of a positive eccentricity is not stated in
//!   the manual** (it prints both "+5%" and "-5%" patterns). Project choice:
//!   a positive eccentricity offsets the line of action of the force toward
//!   the positive perpendicular axis (+Y for an X load, +X for a Y load), so
//!   the couple about Z is `-e F` for an X load and `+e F` for a Y load; a
//!   negative eccentricity reverses it.
//!
//! The forces are added as GLOBAL joint loads to the pattern, after the mass
//! is final, so every case that applies the pattern sees them.

use crate::SolverError;
use indexmap::IndexMap;
use sap_model::seismic::{
    AutoSeismicAsce710, AutoSeismicDiaphragmMoment, AutoSeismicJointForce, AutoSeismicReport,
    SeismicPeriod,
};
use sap_model::{
    ConstraintAxis, ConstraintType, LoadPatternType, PointLoadForce, PointObj, SapModel,
};

const METERS_PER_FOOT: f64 = 0.3048;

fn bad(pattern: &str, what: &str) -> SolverError {
    SolverError::InvalidInput(format!("automatic seismic load '{pattern}': {what}"))
}

fn unsupported(pattern: &str, what: &str) -> SolverError {
    SolverError::UnsupportedInput(format!("automatic seismic load '{pattern}': {what}"))
}

fn positive(pattern: &str, name: &str, value: f64) -> Result<(), SolverError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(bad(pattern, &format!("{name} must be finite and positive, got {value}")))
    }
}

/// Refuses a definition the analysis cannot honor.
pub(crate) fn validate(model: &SapModel) -> Result<(), SolverError> {
    for (pattern, def) in &model.auto_seismic {
        let Some(pat) = model.load_patterns.get(pattern) else {
            return Err(bad(pattern, "the load pattern does not exist"));
        };
        if pat.pat_type != LoadPatternType::Quake {
            return Err(bad(
                pattern,
                &format!("the pattern type is {:?}, not Quake", pat.pat_type),
            ));
        }
        if !model.active_dofs[def.direction.index()] {
            return Err(unsupported(
                pattern,
                "the load direction is not an active degree of freedom",
            ));
        }
        match def.period {
            SeismicPeriod::Approximate { ct, x } => {
                positive(pattern, "Ct", ct)?;
                positive(pattern, "x", x)?;
            }
            SeismicPeriod::User { t } => positive(pattern, "period T", t)?,
            SeismicPeriod::ProgramCalculated => {
                return Err(unsupported(
                    pattern,
                    "the program-calculated period needs the modal period of a modal case; use an approximate or user period",
                ))
            }
        }
        positive(pattern, "R", def.r)?;
        positive(pattern, "Omega0", def.omega0)?;
        positive(pattern, "Cd", def.cd)?;
        positive(pattern, "Ie", def.ie)?;
        positive(pattern, "TL", def.tl)?;
        for (name, v) in [("Ss", def.ss), ("S1", def.s1)] {
            if !v.is_finite() || v < 0.0 {
                return Err(bad(pattern, &format!("{name} must be finite and not negative")));
            }
        }
        if !def.eccentricity_ratio.is_finite() {
            return Err(bad(pattern, "the eccentricity ratio is not finite"));
        }
        if let Some(table) = model.auto_seismic_eccentricity.get(pattern) {
            for (diaphragm, distance) in table {
                let ok = model.constraints.get(diaphragm).is_some_and(|c| {
                    c.constraint_type == ConstraintType::Diaphragm && c.axis == ConstraintAxis::Z
                });
                if !ok {
                    return Err(bad(
                        pattern,
                        &format!("the eccentricity overwrite names '{diaphragm}', which is not a diaphragm constraint about the Z axis"),
                    ));
                }
                if !distance.is_finite() {
                    return Err(bad(
                        pattern,
                        &format!("the eccentricity overwrite of diaphragm '{diaphragm}' is not finite"),
                    ));
                }
            }
        }
        for (name, v) in [("Fa", def.fa), ("Fv", def.fv)] {
            match v {
                None => {
                    return Err(unsupported(
                        pattern,
                        &format!("{name} from the site class is not implemented (the code tables are not available); enter {name} directly"),
                    ))
                }
                Some(v) => positive(pattern, name, v)?,
            }
        }
        for (name, v) in [("z_min", def.z_min), ("z_max", def.z_max)] {
            if v.is_some_and(|v| !v.is_finite()) {
                return Err(bad(pattern, &format!("{name} is not finite")));
            }
        }
        if let (Some(lo), Some(hi)) = (def.z_min, def.z_max) {
            if hi <= lo {
                return Err(bad(pattern, "z_max must be above z_min"));
            }
        }
    }
    Ok(())
}

/// Base shear coefficient and the equation that set it.
pub(crate) fn seismic_coefficient(
    def: &AutoSeismicAsce710,
    t: f64,
    sds: f64,
    sd1: f64,
) -> (f64, &'static str) {
    let r_over_ie = def.r / def.ie;
    let mut cs = sds / r_over_ie;
    let mut eq = "12.8-2";
    let cap = if t <= def.tl {
        (sd1 / (t * r_over_ie), "12.8-3")
    } else {
        (sd1 * def.tl / (t * t * r_over_ie), "12.8-4")
    };
    if cs > cap.0 {
        cs = cap.0;
        eq = cap.1;
    }
    let floor5 = (0.044 * sds * def.ie).max(0.01);
    if cs < floor5 {
        cs = floor5;
        eq = "12.8-5";
    }
    if def.s1 >= 0.6 {
        let floor6 = 0.5 * def.s1 / r_over_ie;
        if cs < floor6 {
            cs = floor6;
            eq = "12.8-6";
        }
    }
    (cs, eq)
}

/// Exponent of the vertical distribution.
pub(crate) fn height_exponent(t: f64) -> f64 {
    if t <= 0.5 {
        1.0
    } else if t >= 2.5 {
        2.0
    } else {
        1.0 + (t - 0.5) / 2.0
    }
}

fn is_identity(r: &[[f64; 3]; 3]) -> bool {
    (0..3).all(|i| (0..3).all(|j| (r[i][j] - if i == j { 1.0 } else { 0.0 }).abs() <= 1e-12))
}

/// The eccentric moments of one pattern: one couple about global Z per diaphragm
/// that carries load, added to `work_points` (see the module comment).
/// `applied` holds the joint index and force of every loaded joint.
fn eccentric_moments(
    model: &SapModel,
    pattern: &str,
    def: &AutoSeismicAsce710,
    work_points: &mut IndexMap<String, PointObj>,
    applied: &[(usize, f64)],
) -> Result<Vec<AutoSeismicDiaphragmMoment>, SolverError> {
    let overwrites = model.auto_seismic_eccentricity.get(pattern);
    let mut moments = Vec::new();
    if def.eccentricity_ratio == 0.0 && overwrites.is_none_or(|t| t.is_empty()) {
        return Ok(moments);
    }
    let dir = def.direction.index();
    // Sense of the couple about +Z per unit of eccentricity: see the module comment.
    let sense = if dir == 0 { -1.0 } else { 1.0 };
    for (name, constraint) in &model.constraints {
        if constraint.constraint_type != ConstraintType::Diaphragm {
            continue;
        }
        let members: Vec<usize> = work_points
            .values()
            .enumerate()
            .filter(|(_, p)| p.constraints.iter().any(|c| c == name))
            .map(|(i, _)| i)
            .collect();
        let force: f64 = applied
            .iter()
            .filter(|(i, _)| members.contains(i))
            .map(|(_, f)| f)
            .sum();
        if members.is_empty() || !applied.iter().any(|(i, _)| members.contains(i)) {
            continue;
        }
        let overwrite = overwrites.and_then(|t| t.get(name)).copied();
        if constraint.axis != ConstraintAxis::Z {
            return Err(unsupported(
                pattern,
                &format!("diaphragm '{name}' is not about the Z axis, so its eccentric moment is not implemented; set the eccentricity ratio to 0"),
            ));
        }
        let coordinate = |i: usize| {
            let p = &work_points[i];
            if dir == 0 {
                p.y
            } else {
                p.x
            }
        };
        let lo = members.iter().map(|&i| coordinate(i)).fold(f64::INFINITY, f64::min);
        let hi = members.iter().map(|&i| coordinate(i)).fold(f64::NEG_INFINITY, f64::max);
        let width = hi - lo;
        let eccentricity = overwrite.unwrap_or(def.eccentricity_ratio * width);
        let moment = sense * eccentricity * force;
        if moment == 0.0 {
            continue;
        }
        if !model.active_dofs[5] {
            return Err(unsupported(
                pattern,
                &format!("diaphragm '{name}' needs an eccentric moment about Z but the rotation about Z is not an active degree of freedom"),
            ));
        }
        let at = members
            .iter()
            .copied()
            .find(|&i| !work_points[i].restraint[5])
            .unwrap_or(members[0]);
        let mut f = [0.0; 6];
        f[5] = moment;
        work_points[at].loads_force.push(PointLoadForce {
            load_pat: pattern.to_string(),
            csys: "GLOBAL".into(),
            force: f,
        });
        moments.push(AutoSeismicDiaphragmMoment {
            diaphragm: name.clone(),
            force,
            width,
            eccentricity,
            overwritten: overwrite.is_some(),
            moment_z: moment,
            joint: work_points[at].name.clone(),
        });
    }
    Ok(moments)
}

/// Computes every automatic lateral load and adds its forces, as GLOBAL joint
/// loads of the pattern, to `work_points`. Returns one report per pattern.
pub(crate) fn apply(
    model: &SapModel,
    work_points: &mut IndexMap<String, PointObj>,
    joint_rot: &[[[f64; 3]; 3]],
    diag_mass: &[f64],
) -> Result<Vec<AutoSeismicReport>, SolverError> {
    let mut reports = Vec::new();
    if model.auto_seismic.is_empty() {
        return Ok(reports);
    }
    let g = model.present_units.gravity_acceleration();
    let to_feet = model.present_units.length_unit().to_meters() / METERS_PER_FOOT;
    let z_all_min = work_points.values().map(|p| p.z).fold(f64::INFINITY, f64::min);
    let z_all_max = work_points.values().map(|p| p.z).fold(f64::NEG_INFINITY, f64::max);
    let z_scale = work_points.values().fold(1.0f64, |a, p| a.max(p.z.abs()));
    let tol = 1e-9 * z_scale;

    for (pattern, def) in &model.auto_seismic {
        let z_min = def.z_min.unwrap_or(z_all_min);
        let z_max = def.z_max.unwrap_or(z_all_max);
        if z_max <= z_min {
            return Err(bad(pattern, "the elevation range is empty"));
        }
        let hn_ft = (z_max - z_min) * to_feet;
        let (t, approximate) = match def.period {
            SeismicPeriod::Approximate { ct, x } => {
                let ta = ct * hn_ft.powf(x);
                (ta, Some(ta))
            }
            SeismicPeriod::User { t } => (t, None),
            SeismicPeriod::ProgramCalculated => unreachable!("refused by validate"),
        };
        let sds = 2.0 / 3.0 * def.fa.expect("validated") * def.ss;
        let sd1 = 2.0 / 3.0 * def.fv.expect("validated") * def.s1;
        let (cs, equation) = seismic_coefficient(def, t, sds, sd1);
        let k = height_exponent(t);
        let dir = def.direction.index();

        let mut joints: Vec<(String, f64, f64)> = Vec::new(); // name, z, weight
        let mut order: Vec<usize> = Vec::new();
        for (idx, pt) in work_points.values().enumerate() {
            if !(pt.z > z_min + tol && pt.z <= z_max + tol) {
                continue;
            }
            let m = &diag_mass[6 * idx..6 * idx + 3];
            let rot = &joint_rot[idx];
            let top = m.iter().fold(0.0f64, |a, v| a.max(v.abs()));
            let equal = m.iter().all(|v| (v - m[0]).abs() <= 1e-12 * top);
            let m_dir = if equal {
                m[0]
            } else if is_identity(rot) {
                m[dir]
            } else {
                return Err(unsupported(
                    pattern,
                    &format!(
                        "joint '{}' has rotated local axes and unequal translational masses",
                        pt.name
                    ),
                ));
            };
            if m_dir <= 1e-18 {
                continue;
            }
            joints.push((pt.name.clone(), pt.z, m_dir * g));
            order.push(idx);
        }
        if joints.is_empty() {
            return Err(unsupported(
                pattern,
                "no joint with mass lies above the base elevation and within the top elevation",
            ));
        }
        let weight: f64 = joints.iter().map(|j| j.2).sum();
        let base_shear = cs * weight;
        let denom: f64 = joints.iter().map(|j| j.2 * (j.1 - z_min).powf(k)).sum();
        let mut joint_forces = Vec::with_capacity(joints.len());
        let mut applied: Vec<(usize, f64)> = Vec::with_capacity(joints.len());
        for ((name, z, w), idx) in joints.into_iter().zip(order) {
            let force = base_shear * w * (z - z_min).powf(k) / denom;
            let mut f = [0.0; 6];
            f[dir] = force;
            work_points[idx].loads_force.push(PointLoadForce {
                load_pat: pattern.clone(),
                csys: "GLOBAL".into(),
                force: f,
            });
            applied.push((idx, force));
            joint_forces.push(AutoSeismicJointForce {
                joint: name,
                z,
                weight: w,
                force,
            });
        }
        let diaphragm_moments = eccentric_moments(model, pattern, def, work_points, &applied)?;
        reports.push(AutoSeismicReport {
            pattern: pattern.clone(),
            direction: def.direction,
            period: t,
            approximate_period: approximate,
            k,
            sds,
            sd1,
            cs,
            governing_equation: equation.into(),
            weight,
            base_shear,
            joint_forces,
            diaphragm_moments,
        });
    }
    Ok(reports)
}

/// Stores each report in every case that applies its pattern.
pub(crate) fn attach(model: &mut SapModel, reports: &[AutoSeismicReport]) {
    for report in reports {
        for (name, case) in &model.load_cases {
            let applies = case
                .loads
                .iter()
                .any(|l| l.load_name == report.pattern && l.scale_factor != 0.0)
                || case
                    .history_loads
                    .iter()
                    .enumerate()
                    .any(|(i, (_, p, s))| {
                        *p == report.pattern
                            && *s != 0.0
                            && case.history_kind(i) != sap_model::HistoryLoadKind::Acceleration
                    });
            if applies {
                if let Some(results) = model.case_results.get_mut(name) {
                    results.auto_lateral.push(report.clone());
                }
            }
        }
    }
}
