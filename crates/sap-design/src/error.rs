use sap_model::{SapModel};
use thiserror::Error;

/// Failures that prevent a design run from producing an adequacy result.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum DesignError {
    #[error("unsupported steel design code '{0}'; supported codes are AISC 360-10 LRFD, AISC 360-10 ASD, and Eurocode 3")]
    UnsupportedSteelCode(String),
    #[error(
        "unsupported concrete design code '{0}'; supported codes are ACI 318-14 and Eurocode 2"
    )]
    UnsupportedConcreteCode(String),
    #[error("invalid shear property '{property}' for section '{section}': {value}")]
    InvalidShearProperty {
        section: String,
        property: &'static str,
        value: String,
    },
    #[error("AISC shear calculation is not implemented for section '{section}' of type {kind}")]
    UnsupportedShearSection { section: String, kind: String },
    #[error("steel code design is not applicable to material '{material}' on member '{frame}'")]
    UnsupportedSteelMaterial { frame: String, material: String },
    #[error("steel design does not yet implement the closed-section torsion interaction for member '{frame}'")]
    UnsupportedClosedSectionTorsion { frame: String },
    /// AISC E7.2b (manual p. 3-34): a round HSS with D/t above 0.45 E/Fy is too slender and is not
    /// designed.
    #[error("steel design of member '{frame}': Pipe section '{section}' has D/t = {d_over_t}, above the limit 0.45 E/Fy = {limit}; AISC 360-10 considers it too slender and does not design it")]
    SlenderPipeNotDesigned {
        frame: String,
        section: String,
        d_over_t: String,
        limit: String,
    },
    /// An I section whose flanges differ (D88, G70): the design formulas of the code take the
    /// warping constant and the flange pairing of an equal-flange section.
    #[error("steel design of member '{frame}': section '{section}' is a singly symmetric I (unequal flanges); {what} not implemented")]
    UnequalFlangeISection {
        frame: String,
        section: String,
        what: &'static str,
    },
    /// EC3 6.3.3(4) member check under compression and bending for a shape that EN 1993-1-1
    /// Annex B (Method 2) has no factors for (D89, G71). The Annex has columns for I sections
    /// (equal flanges) and rectangular hollow sections; the design reads Box and Pipe as the
    /// latter. Every other shape with compression and a moment at one station is refused.
    #[error("steel design of member '{frame}', combination '{combo}', station {station}: EC3 6.3.3(4) member check under compression and bending is built from the interaction factors of EN 1993-1-1 Annex B (Method 2) for I (equal flanges), Box and Pipe sections only; section {section} has none")]
    Ec3CompressionBendingUnsupported {
        frame: String,
        combo: String,
        station: String,
        /// `'<name>' (<shape>)`, kept in one string so that the error type stays small.
        section: String,
    },
    /// EC3 section whose class needs effective properties (Class 4) or cannot be read from the
    /// vendor tables (D90, G72): not designed.
    #[error("steel design of member '{frame}': section '{section}' under {detail}; EC3 Class 4 effective properties are not implemented")]
    Ec3SectionClass {
        frame: String,
        section: String,
        /// `"<load> has element '<element>' with c/t = <ratio> above the limit <limit> (<class>)"`, kept in one string so that the error type stays small.
        detail: String,
    },
    #[error("steel design of member '{frame}': Eurocode 3 section classification is not defined for section '{section}' of type {kind} (not in the vendor Tables 4-1 and 4-2)")]
    UnsupportedEc3Classification {
        frame: String,
        section: String,
        kind: String,
    },
    #[error("steel design overwrite '{item}' of member '{frame}' is {value}; it must be finite and positive")]
    InvalidSteelOverwrite {
        frame: String,
        item: &'static str,
        /// The offending value, as text (the error type is `Eq`).
        value: String,
    },
    /// A design combination that has no complete results (D78): design never
    /// uses the contributors that exist in its place.
    #[error("{discipline} design cannot use combination '{combo}': it is unavailable because {reason}")]
    UnavailableCombination {
        discipline: &'static str,
        combo: String,
        reason: String,
    },
    /// A case that is not finished (G16) has no complete response to design for, and a case
    /// that could not start has none at all.
    #[error("{discipline} design cannot use case '{case}': it did not finish ({reason})")]
    UnfinishedCase {
        discipline: &'static str,
        case: String,
        reason: String,
    },
    #[error("{discipline} design has no frame force demand for member '{frame}'")]
    MissingDemand {
        discipline: &'static str,
        frame: String,
    },
    #[error("{discipline} design does not yet support local-3 shear demand on member '{frame}' (V3={v3})")]
    UnsupportedLocal3Shear {
        discipline: &'static str,
        frame: String,
        v3: String,
    },
    #[error("concrete beam minor-axis flexure M2 is unsupported for member '{frame}' because the result schema cannot report side-face reinforcement (M2={m2})")]
    UnsupportedBeamMinorAxisFlexure { frame: String, m2: String },
    #[error("EC3 local-3 shear is not implemented for section '{section}' of type {kind}")]
    UnsupportedEc3MinorShearSection { section: String, kind: String },
    #[error("EC3 local-3 shear interaction on member '{frame}' is supported only for V3 plus local-2 bending (P=V2=M3=T=0); found P={p}, V2={v2}, M3={m3}, T={t}")]
    UnsupportedEc3ShearInteraction {
        frame: String,
        p: String,
        v2: String,
        m3: String,
        t: String,
    },
    /// A refusal of the steel design workflow (Auto Select lists, default
    /// design combinations, verification); the text names the offending item.
    #[error("steel design workflow: {0}")]
    Workflow(String),
    /// A concrete member whose rebar yield stress is given nowhere. `material` is the material
    /// that should carry `fy`: the rebar material the section names, or the concrete itself when
    /// the section names none.
    #[error("concrete design of frame '{frame}': section '{section}' has no longitudinal rebar yield stress, because material '{material}' has no positive fy; give that material a positive fy, or name a rebar material with a positive fy in the section's reinforcement")]
    MissingRebarYield {
        frame: String,
        section: String,
        material: String,
    },
    #[error("{discipline} design found nonfinite {component} demand/station data on member '{frame}' ({value})")]
    NonFiniteDemand {
        discipline: &'static str,
        frame: String,
        component: &'static str,
        value: String,
    },
}

pub(crate) fn validate_demands<F>(
    model: &SapModel,
    combo_forces: &[(String, Vec<(String, crate::steel::FrameStates)>)],
    discipline: &'static str,
    allow_minor_shear: bool,
    mut is_designable: F,
) -> Result<(), DesignError>
where
    F: FnMut(&SapModel, &str) -> bool,
{
    for frame_name in model
        .frames
        .keys()
        .filter(|name| is_designable(model, name))
    {
        let mut has_demand = false;
        for (_, frames) in combo_forces {
            for (name, stations) in frames
                .iter()
                .flat_map(|(n, states)| states.iter().map(move |s| (n, s)))
            {
                if name != frame_name || stations.is_empty() {
                    continue;
                }
                has_demand = true;
                for station in stations {
                    for (component, value) in [
                        ("station", station.station),
                        ("relative station", station.rel_dist),
                        ("P", station.p),
                        ("V2", station.v2),
                        ("V3", station.v3),
                        ("T", station.t),
                        ("M2", station.m2),
                        ("M3", station.m3),
                    ] {
                        if !value.is_finite() {
                            return Err(DesignError::NonFiniteDemand {
                                discipline,
                                frame: frame_name.clone(),
                                component,
                                value: value.to_string(),
                            });
                        }
                    }
                    // Both transverse shears have force units. Ignore only roundoff-scale noise.
                    let tol = 1e-9 * station.v2.abs().max(1.0);
                    if !allow_minor_shear && station.v3.abs() > tol {
                        return Err(DesignError::UnsupportedLocal3Shear {
                            discipline,
                            frame: frame_name.clone(),
                            v3: station.v3.to_string(),
                        });
                    }
                }
            }
        }
        if !has_demand {
            return Err(DesignError::MissingDemand {
                discipline,
                frame: frame_name.clone(),
            });
        }
    }

    Ok(())
}
