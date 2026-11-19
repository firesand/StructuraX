//! Default steel design load combinations.
//!
//! Source: CSI Steel Frame Design Manual AISC 360-10 (`sfd_aisc_360_10.pdf`),
//! section 2.4 (p. 2-4: dead and live cases are additive, each wind, quake or
//! response-spectrum case is non-additive and produces its own combinations,
//! static wind and quake also with the sense reversed) and section 3.2
//! (pp. 3-6 to 3-8: the combinations themselves, from ASCE 7-10).
//!
//! Differences and choices, all deliberate:
//!
//! * The manual prints two forms of the earthquake rows. The plain form on
//!   p. 3-6 (`0.9 DL +- 1.0 EL`, `1.2 DL + 1.0 LL + 0.2 SL +- 1.0 EL`) is
//!   followed by the "effectively" form with the redundancy factor rho and the
//!   vertical component 0.2 SDS (p. 3-8). The brief asks for the latter, which
//!   needs rho and SDS; it has no snow term. The overstrength rows (Omega0,
//!   pp. 3-7 and 3-8) are internal to the program in the manual and are not
//!   created.
//! * Absent classes: a term whose class has no case is left out of its row;
//!   a row whose lateral case does not exist is not generated; a row with no
//!   term left is not generated; rows that become identical are listed once.
//!   So a model with only dead and live cases gets `1.4 DL`,
//!   `1.2 DL + 1.6 LL` and `1.2 DL + 1.0 LL` (the last from the row printed
//!   with `1.6 RL`).
//! * Notional load patterns are refused: the manual adds them to the gravity
//!   combinations with a direction and sense that depend on the user's
//!   definition of the notional load, which is not recorded in the model.
//! * Order of the generated list: the order of the manual's rows; inside a
//!   lateral row, each lateral case in model order, `+` before `-`.
//! * A response-spectrum case gets a `+` and a `-` row like a static quake
//!   case. The manual (p. 2-5) creates one combination and has the program
//!   take the minima as the negative of the maxima. The design engine here
//!   adds the stored maxima of a case linearly, so one row could never
//!   reverse the sense of the spectrum result; the `-` row is what supplies
//!   the minima. All force components of a row still reverse together: the
//!   mixed-sign permutations of axial force and moments are not designed.

use crate::DesignError;
use sap_model::{ComboType, LoadCaseType, LoadCombo, LoadPatternType, SapModel};

/// LRFD (ASCE 7-10 2.3.2) or ASD (ASCE 7-10 2.4.1) set of combinations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DesignMethod {
    Lrfd,
    Asd,
}

impl DesignMethod {
    /// The method of an AISC 360-10 steel design code name; other codes are refused.
    pub fn from_steel_code(code: &str) -> Result<Self, DesignError> {
        match crate::SteelCode::try_from_code_str(code)? {
            crate::SteelCode::Aisc360Lrfd => Ok(Self::Lrfd),
            crate::SteelCode::Aisc360Asd => Ok(Self::Asd),
            crate::SteelCode::Eurocode3 => Err(refusal(format!(
                "default design combinations are defined for AISC 360-10 only, not '{code}'"
            ))),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct DefaultComboOptions {
    pub method: DesignMethod,
    /// Redundancy factor rho (ASCE 7-10 12.3.4), 1.0 by default.
    pub rho: f64,
    /// Design spectral acceleration SDS; required to be positive when a
    /// seismic case exists, otherwise unused.
    pub sds: f64,
    /// Replace existing `DSTLn` combinations instead of refusing.
    pub replace: bool,
}

impl DefaultComboOptions {
    pub fn new(method: DesignMethod) -> Self {
        Self {
            method,
            rho: 1.0,
            sds: 0.0,
            replace: false,
        }
    }
}

/// What the generation did.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct DefaultComboSummary {
    pub created: Vec<String>,
    /// Existing `DSTLn` combinations that were replaced.
    pub removed: Vec<String>,
    /// Cases that are not used, with the reason.
    pub skipped: Vec<(String, String)>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Class {
    Dl,
    Ll,
    Rl,
    Sl,
    Wl,
    Qe,
}

/// Factor of a term: a constant, `a + b*SDS`, or a multiple of rho.
#[derive(Debug, Clone, Copy)]
enum Factor {
    Const(f64),
    /// `a + b * SDS`
    Sds(f64, f64),
    /// `c * rho` evaluated as the manual groups it: `outer * (inner * rho)`
    Rho(f64, f64),
    /// `outer * inner` for a lateral factor such as 0.75(0.6 WL)
    Product(f64, f64),
}

impl Factor {
    fn value(self, rho: f64, sds: f64) -> f64 {
        match self {
            Factor::Const(c) => c,
            Factor::Sds(a, b) => a + b * sds,
            Factor::Rho(outer, inner) => outer * (inner * rho),
            Factor::Product(a, b) => a * b,
        }
    }
}

struct Row {
    /// Terms in the order the manual prints them (lateral term last).
    terms: &'static [(Class, Factor)],
    /// The lateral class of the row, which gets a `+` and a `-` variant when
    /// the case is static.
    lateral: Option<Class>,
}

use Class::*;
use Factor::*;

/// LRFD rows: manual p. 3-6 (gravity and wind rows) and p. 3-8 (earthquake
/// rows with rho and SDS).
const LRFD: &[Row] = &[
    // 1.4 DL (ASCE 2.3.2-1), p. 3-6
    Row {
        terms: &[(Dl, Const(1.4))],
        lateral: None,
    },
    // 1.2 DL + 1.6 LL + 0.5 RL (ASCE 2.3.2-2), p. 3-6
    Row {
        terms: &[(Dl, Const(1.2)), (Ll, Const(1.6)), (Rl, Const(0.5))],
        lateral: None,
    },
    // 1.2 DL + 1.0 LL + 1.6 RL (ASCE 2.3.2-3), p. 3-6
    Row {
        terms: &[(Dl, Const(1.2)), (Ll, Const(1.0)), (Rl, Const(1.6))],
        lateral: None,
    },
    // 1.2 DL + 1.6 LL + 0.5 SL (ASCE 2.3.2-2), p. 3-6
    Row {
        terms: &[(Dl, Const(1.2)), (Ll, Const(1.6)), (Sl, Const(0.5))],
        lateral: None,
    },
    // 1.2 DL + 1.0 LL + 1.6 SL (ASCE 2.3.2-3), p. 3-6
    Row {
        terms: &[(Dl, Const(1.2)), (Ll, Const(1.0)), (Sl, Const(1.6))],
        lateral: None,
    },
    // 0.9 DL +- 1.0 WL (ASCE 2.3.2-6), p. 3-6
    Row {
        terms: &[(Dl, Const(0.9)), (Wl, Const(1.0))],
        lateral: Some(Wl),
    },
    // 1.2 DL + 1.6 RL +- 0.5 WL (ASCE 2.3.2-3), p. 3-6
    Row {
        terms: &[(Dl, Const(1.2)), (Rl, Const(1.6)), (Wl, Const(0.5))],
        lateral: Some(Wl),
    },
    // 1.2 DL + 1.0 LL + 0.5 RL +- 1.0 WL (ASCE 2.3.2-4), p. 3-6
    Row {
        terms: &[
            (Dl, Const(1.2)),
            (Ll, Const(1.0)),
            (Rl, Const(0.5)),
            (Wl, Const(1.0)),
        ],
        lateral: Some(Wl),
    },
    // 1.2 DL + 1.6 SL +- 0.5 WL (ASCE 2.3.2-3), p. 3-6
    Row {
        terms: &[(Dl, Const(1.2)), (Sl, Const(1.6)), (Wl, Const(0.5))],
        lateral: Some(Wl),
    },
    // 1.2 DL + 1.0 LL + 0.5 SL +- 1.0 WL (ASCE 2.3.2-4), p. 3-6
    Row {
        terms: &[
            (Dl, Const(1.2)),
            (Ll, Const(1.0)),
            (Sl, Const(0.5)),
            (Wl, Const(1.0)),
        ],
        lateral: Some(Wl),
    },
    // (0.9 - 0.2 SDS) DL +- rho QE (ASCE 2.3.2-7, 12.4.2.3), p. 3-8
    Row {
        terms: &[(Dl, Sds(0.9, -0.2)), (Qe, Rho(1.0, 1.0))],
        lateral: Some(Qe),
    },
    // (1.2 + 0.2 SDS) DL +- rho QE (ASCE 2.3.2-5, 12.4.2.3), p. 3-8
    Row {
        terms: &[(Dl, Sds(1.2, 0.2)), (Qe, Rho(1.0, 1.0))],
        lateral: Some(Qe),
    },
    // (1.2 + 0.2 SDS) DL +- rho QE + 1.0 LL (ASCE 2.3.2-5, 12.4.2.3), p. 3-8
    Row {
        terms: &[(Dl, Sds(1.2, 0.2)), (Qe, Rho(1.0, 1.0)), (Ll, Const(1.0))],
        lateral: Some(Qe),
    },
];

/// ASD rows: manual pp. 3-6 and 3-7 (gravity, wind) and p. 3-8 (earthquake
/// rows with rho and SDS).
const ASD: &[Row] = &[
    // 1.0 DL (ASCE 2.4.1-1), p. 3-6
    Row {
        terms: &[(Dl, Const(1.0))],
        lateral: None,
    },
    // 1.0 DL + 1.0 LL (ASCE 2.4.1-2), p. 3-6
    Row {
        terms: &[(Dl, Const(1.0)), (Ll, Const(1.0))],
        lateral: None,
    },
    // 1.0 DL + 1.0 RL (ASCE 2.4.1-3), p. 3-6
    Row {
        terms: &[(Dl, Const(1.0)), (Rl, Const(1.0))],
        lateral: None,
    },
    // 1.0 DL + 0.75 LL + 0.75 RL (ASCE 2.3.2-4 as printed), p. 3-6
    Row {
        terms: &[(Dl, Const(1.0)), (Ll, Const(0.75)), (Rl, Const(0.75))],
        lateral: None,
    },
    // 1.0 DL + 1.0 SL (ASCE 2.4.1-3), p. 3-6
    Row {
        terms: &[(Dl, Const(1.0)), (Sl, Const(1.0))],
        lateral: None,
    },
    // 1.0 DL + 0.75 LL + 0.75 SL (ASCE 2.3.2-4 as printed), p. 3-6
    Row {
        terms: &[(Dl, Const(1.0)), (Ll, Const(0.75)), (Sl, Const(0.75))],
        lateral: None,
    },
    // 1.0 DL +- 0.6 WL (ASCE 2.4.1-5), p. 3-7
    Row {
        terms: &[(Dl, Const(1.0)), (Wl, Const(0.6))],
        lateral: Some(Wl),
    },
    // 1.0 DL + 0.75 LL + 0.75 RL +- 0.75 (0.6 WL) (ASCE 2.4.1-6a), p. 3-7
    Row {
        terms: &[
            (Dl, Const(1.0)),
            (Ll, Const(0.75)),
            (Rl, Const(0.75)),
            (Wl, Product(0.75, 0.6)),
        ],
        lateral: Some(Wl),
    },
    // 1.0 DL + 0.75 LL + 0.75 SL +- 0.75 (0.6 WL) (ASCE 2.4.1-6a), p. 3-7
    Row {
        terms: &[
            (Dl, Const(1.0)),
            (Ll, Const(0.75)),
            (Sl, Const(0.75)),
            (Wl, Product(0.75, 0.6)),
        ],
        lateral: Some(Wl),
    },
    // 0.6 DL +- 0.6 WL (ASCE 2.4.1-7), p. 3-7
    Row {
        terms: &[(Dl, Const(0.6)), (Wl, Const(0.6))],
        lateral: Some(Wl),
    },
    // (1.0 + 0.14 SDS) DL +- 0.7 rho QE (ASCE 2.4.1-5, 12.4.2.3), p. 3-8
    Row {
        terms: &[(Dl, Sds(1.0, 0.14)), (Qe, Rho(1.0, 0.7))],
        lateral: Some(Qe),
    },
    // (1.0 + 0.105 SDS) DL +- 0.75 (0.7 rho) QE + 0.75 LL (ASCE 2.4.1-6b, 12.4.2.3), p. 3-8
    Row {
        terms: &[
            (Dl, Sds(1.0, 0.105)),
            (Qe, Rho(0.75, 0.7)),
            (Ll, Const(0.75)),
        ],
        lateral: Some(Qe),
    },
    // (0.6 - 0.14 SDS) DL +- 0.7 rho QE (ASCE 2.4.1-8, 12.4.2.3), p. 3-8
    Row {
        terms: &[(Dl, Sds(0.6, -0.14)), (Qe, Rho(1.0, 0.7))],
        lateral: Some(Qe),
    },
];

fn refusal(message: impl Into<String>) -> DesignError {
    DesignError::Workflow(message.into())
}

/// A case with the classes it was sorted into; `signed` is true for a static
/// wind or quake case.
struct ClassifiedCase {
    name: String,
    class: Class,
    signed: bool,
}

fn pattern_class(kind: LoadPatternType) -> Option<Class> {
    match kind {
        LoadPatternType::Dead | LoadPatternType::SuperDead => Some(Dl),
        LoadPatternType::Live | LoadPatternType::ReduceLive => Some(Ll),
        LoadPatternType::Rooflive => Some(Rl),
        LoadPatternType::Snow => Some(Sl),
        LoadPatternType::Wind => Some(Wl),
        LoadPatternType::Quake => Some(Qe),
        _ => None,
    }
}

fn classify(
    model: &SapModel,
    summary: &mut DefaultComboSummary,
) -> Result<Vec<ClassifiedCase>, DesignError> {
    let mut cases = Vec::new();
    for (name, case) in &model.load_cases {
        match case.case_type {
            LoadCaseType::ResponseSpectrum => {
                if !case.run_flag {
                    summary
                        .skipped
                        .push((name.clone(), "the case is not flagged to run".into()));
                    continue;
                }
                cases.push(ClassifiedCase {
                    name: name.clone(),
                    class: Qe,
                    signed: true,
                });
            }
            LoadCaseType::LinearStatic => {
                if !case.run_flag {
                    summary
                        .skipped
                        .push((name.clone(), "the case is not flagged to run".into()));
                    continue;
                }
                if case.loads.is_empty() {
                    return Err(refusal(format!(
                        "static case '{name}' has no loads to classify"
                    )));
                }
                let mut class: Option<Class> = None;
                for assignment in &case.loads {
                    let pattern =
                        model
                            .load_patterns
                            .get(&assignment.load_name)
                            .ok_or_else(|| {
                                refusal(format!(
                            "static case '{name}' applies '{}', which is not a load pattern",
                            assignment.load_name
                        ))
                            })?;
                    if pattern.pat_type == LoadPatternType::Notional {
                        return Err(refusal(format!(
                            "case '{name}' uses the notional load pattern '{}'; notional loads are not supported by the default combinations",
                            pattern.name
                        )));
                    }
                    let this = pattern_class(pattern.pat_type).ok_or_else(|| {
                        refusal(format!(
                            "case '{name}' uses pattern '{}' of type {:?}, which the default design combinations do not cover",
                            pattern.name, pattern.pat_type
                        ))
                    })?;
                    match class {
                        None => class = Some(this),
                        Some(c) if c == this => {}
                        Some(_) => {
                            return Err(refusal(format!(
                                "case '{name}' mixes load classes (pattern '{}' differs from the others); split it into one case per class",
                                pattern.name
                            )))
                        }
                    }
                }
                let class = class.expect("non-empty loads give a class");
                cases.push(ClassifiedCase {
                    name: name.clone(),
                    class,
                    signed: matches!(class, Wl | Qe),
                });
            }
            other => summary.skipped.push((
                name.clone(),
                format!("case type {other:?} is not covered by the default combinations"),
            )),
        }
    }
    Ok(cases)
}

fn is_default_name(name: &str) -> bool {
    name.strip_prefix("DSTL")
        .is_some_and(|rest| !rest.is_empty() && rest.bytes().all(|b| b.is_ascii_digit()))
}

/// Builds the combinations without touching a model. Returns term lists in
/// generation order with duplicates removed.
fn build(
    cases: &[ClassifiedCase],
    method: DesignMethod,
    rho: f64,
    sds: f64,
) -> Vec<Vec<(String, f64)>> {
    let rows = match method {
        DesignMethod::Lrfd => LRFD,
        DesignMethod::Asd => ASD,
    };
    let mut out: Vec<Vec<(String, f64)>> = Vec::new();
    for row in rows {
        let lateral_cases: Vec<Option<&ClassifiedCase>> = match row.lateral {
            None => vec![None],
            Some(class) => cases
                .iter()
                .filter(|k| k.class == class)
                .map(Some)
                .collect(),
        };
        for lateral in lateral_cases {
            let senses: &[f64] = match lateral {
                Some(k) if k.signed => &[1.0, -1.0],
                _ => &[1.0],
            };
            for &sense in senses {
                let mut terms: Vec<(String, f64)> = Vec::new();
                for (class, factor) in row.terms {
                    let value = factor.value(rho, sds);
                    if Some(*class) == row.lateral {
                        let k = lateral.expect("lateral row has a lateral case");
                        terms.push((k.name.clone(), sense * value));
                    } else {
                        for k in cases.iter().filter(|k| k.class == *class) {
                            terms.push((k.name.clone(), value));
                        }
                    }
                }
                if terms.is_empty() || out.contains(&terms) {
                    continue;
                }
                out.push(terms);
            }
        }
    }
    out
}

/// Adds the default steel design combinations `DSTL1`, `DSTL2`, ... to the model.
///
/// Everything is validated before the model changes. The model must not be
/// locked. Altering design combinations deletes the steel design results
/// (design manual appendix E).
pub fn add_default_design_combos(
    model: &mut SapModel,
    options: &DefaultComboOptions,
) -> Result<DefaultComboSummary, DesignError> {
    if model.is_locked {
        return Err(refusal(
            "the model is locked; unlock it before adding design combinations",
        ));
    }
    if !options.rho.is_finite() || options.rho <= 0.0 {
        return Err(refusal(format!(
            "redundancy factor rho {} must be positive",
            options.rho
        )));
    }
    let mut summary = DefaultComboSummary::default();
    let cases = classify(model, &mut summary)?;
    if cases.iter().any(|c| c.class == Qe) && !(options.sds.is_finite() && options.sds > 0.0) {
        return Err(refusal(format!(
            "SDS must be positive when a seismic case exists (got {})",
            options.sds
        )));
    }
    let combos = build(&cases, options.method, options.rho, options.sds);
    if combos.is_empty() {
        return Err(refusal("no combination can be generated: the model has no usable dead, live, wind or seismic cases"));
    }

    let existing: Vec<String> = model
        .load_combos
        .keys()
        .filter(|n| is_default_name(n))
        .cloned()
        .collect();
    if !existing.is_empty() && !options.replace {
        return Err(refusal(format!(
            "design combination '{}' already exists; choose replace to overwrite the DSTL combinations",
            existing[0]
        )));
    }
    for i in 1..=combos.len() {
        let name = format!("DSTL{i}");
        if model.load_cases.contains_key(&name) {
            return Err(refusal(format!("'{name}' is the name of an analysis case")));
        }
        if !existing.contains(&name) && model.load_combos.contains_key(&name) {
            return Err(refusal(format!("combination '{name}' already exists")));
        }
    }

    for name in &existing {
        model.load_combos.shift_remove(name);
    }
    summary.removed = existing;
    for (i, terms) in combos.into_iter().enumerate() {
        let name = format!("DSTL{}", i + 1);
        model.load_combos.insert(
            name.clone(),
            LoadCombo {
                name: name.clone(),
                combo_type: ComboType::LinearAdd,
                cases: terms,
            },
        );
        summary.created.push(name);
    }
    model.steel_design_results.clear();
    model.steel_design_errors.clear();
    Ok(summary)
}
