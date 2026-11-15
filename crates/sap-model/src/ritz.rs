//! Ritz-vector load controls and modal load-participation results.
//!
//! Source: CSI Analysis Reference Manual, "Ritz-Vector Analysis" (printed
//! pp. 372-375) and "Static and Dynamic Load Participation Ratios"
//! (pp. 378-381).

use serde::{Deserialize, Serialize};

/// Generation control of one Ritz starting load.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct RitzLoadControl {
    /// A load pattern, or `UX`, `UY`, `UZ` for a unit ground acceleration.
    pub load_name: String,
    /// Largest number of generation cycles; `0` is unlimited.
    pub max_cycles: usize,
    /// Target dynamic participation ratio in `(0, 1]`; `0` is no target.
    pub target_dynamic_participation: f64,
}

/// The names that select a unit ground acceleration instead of a load
/// pattern, with the global direction index each one stands for.
pub const ACCELERATION_LOADS: [(&str, usize); 3] = [("UX", 0), ("UY", 1), ("UZ", 2)];

/// Direction index (0, 1, 2) when `name` is `UX`, `UY` or `UZ` (any case) or
/// the imported spelling `U1`, `U2`, `U3` (the SDB importer stores a Ritz
/// case's acceleration starting loads under those names); otherwise `None`.
/// Rotational accelerations (`U4`..`U6`) are not starting loads.
pub fn acceleration_direction(name: &str) -> Option<usize> {
    let name = name.trim();
    ACCELERATION_LOADS
        .iter()
        .zip(["U1", "U2", "U3"])
        .find(|((label, _), alias)| {
            label.eq_ignore_ascii_case(name) || alias.eq_ignore_ascii_case(name)
        })
        .map(|((_, dir), _)| *dir)
}

/// Static and dynamic participation of one starting load in a modal case.
///
/// Every vector holds one value per mode, in mode order. For a spatial load
/// `p` and mode `n` with shape `phi_n` and circular frequency `w_n`:
/// the static ratio is `(phi_n.p / w_n)^2 / (u.p)` with `K u = p`; the
/// dynamic ratio is `(phi_n.p)^2 / (a.p)` with `M a = p`, both over the mass
/// degrees of freedom as the reference describes.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct LoadParticipation {
    /// Load pattern name, or `UX`, `UY`, `UZ`.
    pub load_name: String,
    /// True for a unit ground acceleration, false for a load pattern.
    pub is_acceleration: bool,
    /// Ritz vectors generated from this load; `0` for eigenvector cases.
    pub vectors_generated: usize,
    pub static_ratio: Vec<f64>,
    pub static_cumulative: Vec<f64>,
    pub dynamic_ratio: Vec<f64>,
    pub dynamic_cumulative: Vec<f64>,
}
