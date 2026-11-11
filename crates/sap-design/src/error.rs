use sap_model::{FrameStationResult, SapModel};
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
    combo_forces: &[(String, Vec<(String, Vec<FrameStationResult>)>)],
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
            for (name, stations) in frames {
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
