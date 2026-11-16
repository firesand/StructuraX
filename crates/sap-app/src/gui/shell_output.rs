//! Output quantities of shell (area) results, as the form
//! "Display > Show Forces/Stresses > Shells" of the vendor lists them.
//!
//! Everything here is derived at display time from the stored resultants and
//! top-face stresses of an area result and the thicknesses of its section;
//! nothing new is stored. Values are element-centre values: the solver does
//! not extrapolate to the joints and nothing is averaged.
//!
//! Sign convention (CSI Analysis Reference, Eqns. 1 and 2): a positive moment
//! gives positive stress at the bottom face, so `s = F/tm -/+ 6 M/tb^2` at the
//! top/bottom face. Principal and von Mises values need a single-valued result
//! (p. 214); see [`is_single_valued`].
//!
//! Which quantities a section has follows the solver's `section_behaviours`:
//! a plate has no membrane forces, a membrane or a plane-stress section no
//! moments or shears, and plane-strain and axisymmetric-solid sections have no
//! shell output at all.

use super::display_units::Dimension;
use sap_model::{AreaPropType, AreaResult, ComboType, LoadCaseType, PlaneType, SapModel};
use sap_solver::area::{plane_von_mises, section_behaviours};

/// Resultant forces per unit length of the midsurface.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShellResultant {
    F11,
    F22,
    F12,
    FMax,
    FMin,
    Fvm,
    M11,
    M22,
    M12,
    MMax,
    MMin,
    V13,
    V23,
    VMax,
}

/// Shell stresses.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShellStress {
    S11,
    S22,
    S12,
    SMax,
    SMin,
    Svm,
    S13,
    S23,
    SMaxV,
}

/// The face a stress is reported on. `Top` is the face at positive local 3.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Face {
    Top,
    Bottom,
    /// The larger of the top and bottom values.
    Maximum,
    /// The smaller of the top and bottom values.
    Minimum,
    /// The value of larger magnitude, with its sign.
    AbsMaximum,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShellQuantity {
    Resultant(ShellResultant),
    Stress(ShellStress, Face),
}

/// The kind of object result a case or combination may lack.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[allow(dead_code)] // Solid and Link: the result forms of this package show shells only.
pub(super) enum ResultKind {
    Area,
    Solid,
    Link,
}

/// Where the manual limits principal and von Mises values.
const SINGLE_VALUED_RULE: &str = "CSI Analysis Reference Manual, p. 214";

fn resultant_name(r: ShellResultant) -> &'static str {
    match r {
        ShellResultant::F11 => "F11",
        ShellResultant::F22 => "F22",
        ShellResultant::F12 => "F12",
        ShellResultant::FMax => "FMAX",
        ShellResultant::FMin => "FMIN",
        ShellResultant::Fvm => "FVM",
        ShellResultant::M11 => "M11",
        ShellResultant::M22 => "M22",
        ShellResultant::M12 => "M12",
        ShellResultant::MMax => "MMAX",
        ShellResultant::MMin => "MMIN",
        ShellResultant::V13 => "V13",
        ShellResultant::V23 => "V23",
        ShellResultant::VMax => "VMAX",
    }
}

fn stress_name(s: ShellStress) -> &'static str {
    match s {
        ShellStress::S11 => "S11",
        ShellStress::S22 => "S22",
        ShellStress::S12 => "S12",
        ShellStress::SMax => "SMAX",
        ShellStress::SMin => "SMIN",
        ShellStress::Svm => "SVM",
        ShellStress::S13 => "S13",
        ShellStress::S23 => "S23",
        ShellStress::SMaxV => "SMAXV",
    }
}

impl ShellStress {
    /// True for the transverse-shear stresses, which have one value through
    /// the thickness here (`V/tb`) and so no face.
    fn is_shear(self) -> bool {
        matches!(self, Self::S13 | Self::S23 | Self::SMaxV)
    }
}

impl ShellQuantity {
    /// The five quantities the Display menu has always offered.
    pub(super) const LEGACY: [ShellQuantity; 5] = [
        ShellQuantity::Resultant(ShellResultant::F11),
        ShellQuantity::Resultant(ShellResultant::F22),
        ShellQuantity::Resultant(ShellResultant::M11),
        ShellQuantity::Resultant(ShellResultant::M22),
        ShellQuantity::Stress(ShellStress::Svm, Face::Top),
    ];

    /// Label for menus and legends. The five legacy labels are fixed; the
    /// others use the vendor's spelling (FMAX, SVM, ...). A stress label ends
    /// with its face, except for the transverse shear stresses, which have none.
    pub(super) fn label(self) -> String {
        match self {
            Self::Resultant(ShellResultant::F11) => "Membrane F11".to_string(),
            Self::Resultant(ShellResultant::F22) => "Membrane F22".to_string(),
            Self::Resultant(ShellResultant::M11) => "Plate Moment M11".to_string(),
            Self::Resultant(ShellResultant::M22) => "Plate Moment M22".to_string(),
            Self::Stress(ShellStress::Svm, Face::Top) => "Von Mises Stress SVM".to_string(),
            Self::Resultant(r) => {
                let kind = match r {
                    ShellResultant::M11
                    | ShellResultant::M22
                    | ShellResultant::M12
                    | ShellResultant::MMax
                    | ShellResultant::MMin => "Plate Moment",
                    ShellResultant::V13 | ShellResultant::V23 | ShellResultant::VMax => {
                        "Plate Shear"
                    }
                    _ => "Membrane",
                };
                format!("{kind} {}", resultant_name(r))
            }
            Self::Stress(s, face) => {
                let kind = if s == ShellStress::Svm {
                    "Von Mises Stress"
                } else {
                    "Stress"
                };
                if s.is_shear() {
                    return format!("{kind} {}", stress_name(s));
                }
                let face = match face {
                    Face::Top => "Top",
                    Face::Bottom => "Bottom",
                    Face::Maximum => "Max",
                    Face::Minimum => "Min",
                    Face::AbsMaximum => "Abs Max",
                };
                format!("{kind} {} ({face})", stress_name(s))
            }
        }
    }

    pub(super) fn dimension(self) -> Dimension {
        match self {
            Self::Resultant(
                ShellResultant::M11
                | ShellResultant::M22
                | ShellResultant::M12
                | ShellResultant::MMax
                | ShellResultant::MMin,
            ) => Dimension::MomentPerLength,
            Self::Resultant(_) => Dimension::ForcePerLength,
            Self::Stress(..) => Dimension::Stress,
        }
    }

    /// True for principal and von Mises values, bottom-face stresses and face
    /// extremes: the quantities a multi-valued result cannot give. The stored
    /// resultants, the top-face S11, S22, S12 and the transverse shear stresses
    /// S13 and S23 are not derived: `V/t` and the top-face components follow
    /// every combination rule, and p. 214 limits only the nonlinear derivations.
    pub(super) fn is_derived(self) -> bool {
        match self {
            Self::Resultant(r) => matches!(
                r,
                ShellResultant::FMax
                    | ShellResultant::FMin
                    | ShellResultant::Fvm
                    | ShellResultant::MMax
                    | ShellResultant::MMin
                    | ShellResultant::VMax
            ),
            Self::Stress(s, face) => match s {
                ShellStress::S13 | ShellStress::S23 => false,
                ShellStress::SMaxV | ShellStress::SMax | ShellStress::SMin | ShellStress::Svm => {
                    true
                }
                ShellStress::S11 | ShellStress::S22 | ShellStress::S12 => face != Face::Top,
            },
        }
    }

    /// What the section must have for the quantity to exist.
    fn needs(self) -> Needs {
        match self {
            Self::Resultant(
                ShellResultant::F11
                | ShellResultant::F22
                | ShellResultant::F12
                | ShellResultant::FMax
                | ShellResultant::FMin
                | ShellResultant::Fvm,
            ) => Needs::Membrane,
            Self::Resultant(_) => Needs::Bending,
            Self::Stress(s, _) if s.is_shear() => Needs::Bending,
            Self::Stress(..) => Needs::Either,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Needs {
    Membrane,
    Bending,
    /// In-plane stress needs the membrane or the bending behaviour.
    Either,
}

/// Behaviours and thicknesses of the section of an area, taken the way
/// `recover_area_result` takes them (`max(1e-12)`).
struct AreaBasis {
    membrane: bool,
    bending: bool,
    tm: f64,
    tb: f64,
}

fn area_basis(
    model: &SapModel,
    result: &str,
    area: &str,
    q: ShellQuantity,
) -> Result<AreaBasis, String> {
    let obj = model
        .areas
        .get(area)
        .ok_or_else(|| format!("area '{area}' does not exist (result '{result}')"))?;
    let sec = model.area_sections.get(&obj.section).ok_or_else(|| {
        format!(
            "area '{area}' has section '{}', which does not exist",
            obj.section
        )
    })?;
    // A plane-stress section is a membrane of the section's thickness and its
    // stored forces are results. A plane-strain section is not reported: the
    // solver assembles it with the plane-strain relation but recovers the
    // forces with the plane-stress one, and the out-of-plane stress that its
    // von Mises value needs is not stored. An axisymmetric-solid section is
    // never analysed.
    let plane_strain =
        sec.prop_type == AreaPropType::Plane && sec.plane_type == PlaneType::PlaneStrain;
    if sec.prop_type == AreaPropType::ASolid || plane_strain {
        let kind = if plane_strain {
            "a plane-strain"
        } else {
            "an axisymmetric-solid"
        };
        return Err(format!(
            "area '{area}' has section '{}', {kind} section: shell resultants and stresses are not reported for it",
            sec.name
        ));
    }
    let (membrane, bending) = section_behaviours(sec);
    let ok = match q.needs() {
        Needs::Membrane => membrane,
        Needs::Bending => bending,
        Needs::Either => membrane || bending,
    };
    if !ok {
        let what = if membrane {
            "has membrane behaviour only, so it has no moments, shears or transverse shear stresses"
        } else {
            "has plate behaviour only, so it has no membrane forces"
        };
        return Err(format!(
            "{} is not available for area '{area}': its section '{}' {what}",
            q.label(),
            sec.name
        ));
    }
    Ok(AreaBasis {
        membrane,
        bending,
        tm: sec.thickness_membrane.max(1e-12),
        tb: sec.thickness_bending.max(1e-12),
    })
}

/// Stress components `[s11, s22, s12]` on the top (`bottom == false`) or
/// bottom face. A full shell takes the stored top-face values; a section
/// without one of the behaviours drops that part of the resultants (the
/// solver stabilises the unused degrees of freedom, so its stored value there
/// is not a result).
fn face_components(ar: &AreaResult, b: &AreaBasis, bottom: bool) -> [f64; 3] {
    if !bottom && b.membrane && b.bending {
        return [ar.s11_top, ar.s22_top, ar.s12_top];
    }
    let f = if b.membrane {
        [ar.f11, ar.f22, ar.f12]
    } else {
        [0.0; 3]
    };
    let m = if b.bending {
        [ar.m11, ar.m22, ar.m12]
    } else {
        [0.0; 3]
    };
    let sign = if bottom { 1.0 } else { -1.0 };
    std::array::from_fn(|i| f[i] / b.tm + sign * 6.0 * m[i] / (b.tb * b.tb))
}

/// Value of an in-plane stress component or derived stress on one face.
fn stress_on_face(ar: &AreaResult, b: &AreaBasis, s: ShellStress, bottom: bool) -> f64 {
    let c = face_components(ar, b, bottom);
    match s {
        ShellStress::S11 => c[0],
        ShellStress::S22 => c[1],
        ShellStress::S12 => c[2],
        ShellStress::SMax => principal(c[0], c[1], c[2]).0,
        ShellStress::SMin => principal(c[0], c[1], c[2]).1,
        ShellStress::Svm => {
            if !bottom && b.membrane && b.bending {
                ar.svm_top
            } else {
                plane_von_mises(c[0], c[1], c[2])
            }
        }
        // The shear stresses have no face; `area_value` handles them first.
        ShellStress::S13 | ShellStress::S23 | ShellStress::SMaxV => f64::NAN,
    }
}

/// Value of `q` for area `area` in the result `result` (a load case or
/// combination), in model units. `Err` names what is missing or why the
/// quantity is not available for this result.
///
/// Stored resultants are returned as they are; stresses on the bottom face
/// are `F/tm + 6M/tb^2` with the thicknesses the solver recovers with; the
/// derived values (principal, von Mises, face extremes) need a single-valued
/// result.
pub(super) fn area_value(
    model: &SapModel,
    result: &str,
    area: &str,
    q: ShellQuantity,
) -> Result<f64, String> {
    let cr = model
        .case_results
        .get(result)
        .ok_or_else(|| format!("'{result}' has no results"))?;
    let ar = cr
        .area_results
        .get(area)
        .ok_or_else(|| format!("'{result}' has no result for area '{area}'"))?;
    let basis = area_basis(model, result, area, q)?;
    if q.is_derived() && !is_single_valued(model, result) {
        return Err(format!(
            "{} is not available for '{result}': it is a multi-valued combination or case, and principal, von Mises, bottom-face and face-extreme values need a single-valued result ({SINGLE_VALUED_RULE})",
            q.label()
        ));
    }
    // A plate section's in-plane stress is -6M/tb^2 on the top face. The sign
    // does not survive an SRSS, absolute or envelope combination of M, so the
    // value cannot be rebuilt from the combined moment (a membrane section's
    // F/tm can: the factor is positive).
    if let ShellQuantity::Stress(s, _) = q {
        if !s.is_shear() && !basis.membrane && !is_single_valued(model, result) {
            return Err(format!(
                "{} is not available for area '{area}' in '{result}': the section has plate behaviour only, its face stresses come from the moments with a sign, and '{result}' is multi-valued ({SINGLE_VALUED_RULE})",
                q.label()
            ));
        }
    }
    Ok(match q {
        ShellQuantity::Resultant(r) => match r {
            ShellResultant::F11 => ar.f11,
            ShellResultant::F22 => ar.f22,
            ShellResultant::F12 => ar.f12,
            ShellResultant::FMax => principal(ar.f11, ar.f22, ar.f12).0,
            ShellResultant::FMin => principal(ar.f11, ar.f22, ar.f12).1,
            ShellResultant::Fvm => plane_von_mises(ar.f11, ar.f22, ar.f12),
            ShellResultant::M11 => ar.m11,
            ShellResultant::M22 => ar.m22,
            ShellResultant::M12 => ar.m12,
            ShellResultant::MMax => principal(ar.m11, ar.m22, ar.m12).0,
            ShellResultant::MMin => principal(ar.m11, ar.m22, ar.m12).1,
            ShellResultant::V13 => ar.v13,
            ShellResultant::V23 => ar.v23,
            ShellResultant::VMax => ar.v13.hypot(ar.v23),
        },
        ShellQuantity::Stress(ShellStress::S13, _) => ar.v13 / basis.tb,
        ShellQuantity::Stress(ShellStress::S23, _) => ar.v23 / basis.tb,
        ShellQuantity::Stress(ShellStress::SMaxV, _) => ar.v13.hypot(ar.v23) / basis.tb,
        ShellQuantity::Stress(s, face) => {
            let top = || stress_on_face(ar, &basis, s, false);
            let bottom = || stress_on_face(ar, &basis, s, true);
            match face {
                Face::Top => top(),
                Face::Bottom => bottom(),
                Face::Maximum => top().max(bottom()),
                Face::Minimum => top().min(bottom()),
                Face::AbsMaximum => {
                    let (t, b) = (top(), bottom());
                    // A tie keeps the top face.
                    if b.abs() > t.abs() {
                        b
                    } else {
                        t
                    }
                }
            }
        }
    })
}

/// True when `result` has one value per quantity: a case other than a
/// response-spectrum, moving-load, steady-state or power-spectral-density
/// case, or a linear combination of such results (CSI Analysis Reference
/// Manual, pp. 214 and 352). Envelope, SRSS, absolute-sum and range-sum
/// combinations are multi-valued (the solver adds a range-sum linearly, but
/// the vendor reports its range), and so is a name that is neither a case nor
/// a combination.
pub(super) fn is_single_valued(model: &SapModel, result: &str) -> bool {
    single_valued_inner(model, result, 0)
}

fn single_valued_inner(model: &SapModel, name: &str, depth: usize) -> bool {
    if depth > model.load_combos.len() {
        return false; // a cycle
    }
    if let Some(combo) = model.load_combos.get(name) {
        return combo.combo_type == ComboType::LinearAdd
            && combo
                .cases
                .iter()
                .all(|(item, _)| single_valued_inner(model, item, depth + 1));
    }
    match model.load_cases.get(name) {
        // Response-spectrum, moving-load, steady-state and power-spectral-density
        // results are ranges, not single states.
        Some(case) => !matches!(
            case.case_type,
            LoadCaseType::ResponseSpectrum
                | LoadCaseType::MovingLoad
                | LoadCaseType::SteadyState
                | LoadCaseType::PowerSpectralDensity
        ),
        None => false,
    }
}

/// When `result` has no results of `kind` because of the cases behind it, the
/// message that names those cases and their types (a modal, buckling or
/// response-spectrum case carries no shell results); otherwise `None`.
/// A combination is searched through its nested combinations.
pub(super) fn missing_results_message(
    model: &SapModel,
    result: &str,
    kind: ResultKind,
) -> Option<String> {
    let noun = match kind {
        ResultKind::Area => "shell",
        ResultKind::Solid => "solid",
        ResultKind::Link => "link",
    };
    let mut leaves = Vec::new();
    collect_cases(model, result, &mut leaves, 0);
    let mut lacking = Vec::new();
    for name in leaves {
        let Some(case) = model.load_cases.get(&name) else {
            lacking.push(format!("'{name}' is neither a load case nor a combination"));
            continue;
        };
        let has = model.case_results.get(&name).is_some_and(|cr| match kind {
            ResultKind::Area => !cr.area_results.is_empty(),
            ResultKind::Solid => !cr.solid_results.is_empty(),
            ResultKind::Link => !cr.link_forces.is_empty(),
        });
        if has {
            continue;
        }
        let none_by_type = matches!(
            case.case_type,
            LoadCaseType::Modal | LoadCaseType::Buckling | LoadCaseType::ResponseSpectrum
        );
        lacking.push(if none_by_type {
            format!(
                "case '{name}' ({:?}) carries no {noun} results",
                case.case_type
            )
        } else if model.case_results.contains_key(&name) {
            format!("case '{name}' ({:?}) has no {noun} results", case.case_type)
        } else {
            format!("case '{name}' ({:?}) has no results", case.case_type)
        });
    }
    if lacking.is_empty() {
        return None;
    }
    Some(format!(
        "'{result}' has no {noun} results: {}",
        lacking.join("; ")
    ))
}

/// The names behind `name`, once each, in order of appearance: the cases of a
/// combination (through nested combinations), or `name` itself.
fn collect_cases(model: &SapModel, name: &str, out: &mut Vec<String>, depth: usize) {
    if depth > model.load_combos.len() {
        return;
    }
    if let Some(combo) = model.load_combos.get(name) {
        for (item, _) in &combo.cases {
            collect_cases(model, item, out, depth + 1);
        }
    } else if !out.iter().any(|n| n == name) {
        out.push(name.to_string());
    }
}

/// The face of an area with local 3 axis `local3` that a viewer in direction
/// `toward_viewer` (from the model toward the eye) sees. The top face is the
/// one at positive local 3; a tie (edge-on view) goes to the top.
pub(super) fn visible_face(local3: [f64; 3], toward_viewer: [f64; 3]) -> Face {
    let dot: f64 = local3.iter().zip(&toward_viewer).map(|(a, b)| a * b).sum();
    if dot >= 0.0 {
        Face::Top
    } else {
        Face::Bottom
    }
}

/// Principal values `(max, min)` of the plane tensor `[[a, c], [c, b]]`
/// (Mohr's circle: centre `(a+b)/2`, radius `hypot((a-b)/2, c)`).
pub(super) fn principal(a: f64, b: f64, c: f64) -> (f64, f64) {
    let centre = 0.5 * (a + b);
    let radius = (0.5 * (a - b)).hypot(c);
    (centre + radius, centre - radius)
}

#[cfg(test)]
mod tests;
