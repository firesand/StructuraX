# Introductory beam workflow regressions

These are independent analytical acceptance fixtures for the three workflow topics in
`supplementary-playlist.json`: simply supported, cantilever, and two-span continuous
beams. The playlist entries are metadata only; these models do not claim to reproduce
video loads, dimensions, screenshots, or results.

## Common setup

The integration tests in `crates/sap-solver/tests/beam_tutorial_regressions.rs` use kip,
inch, and kip/in units, with `E = 30,000 kip/in^2`, `I22 = I33 = 120 in^4`, span
`L = 120 in`, and `w = 0.02 kip/in`. Transverse load and displacement are along local
3 (global Y for these horizontal X-axis members); bending moment is local M2. Shear
areas are zero, so the analytical reference is Euler-Bernoulli bending with no shear
deflection. Self-weight is disabled. Reactions, moments, and joint displacements must
be finite and match the closed-form values within the stated absolute/relative test
tolerances.

## Checks

- Simply supported, pin/roller, uniform load: each reaction is `wL/2`, midspan moment
  is `wL^2/8`, and midspan deflection magnitude is `5wL^4/(384EI)`. A midspan joint is
  explicit so deflection is checked from the solved DOF, not from station interpolation.
  The same model is exported to S2K, parsed again, and reanalyzed.
- Cantilever, fixed base, tip force `P = wL`: base shear is `P`, base moment is `PL`,
  tip deflection is `PL^3/(3EI)`, and tip section moment is zero.
- Equal two-span continuous beam, uniform load on both spans: outer reactions are
  `3wL/8`, interior reaction is `5wL/4`, interior support moment magnitude is
  `wL^2/8`, each midspan moment magnitude is `wL^2/16`, and each midspan deflection
  magnitude is `wL^4/(192EI)`. Each span has an explicit midpoint joint for the
  displacement check.

## Evidence boundary

These checks exercise the Rust model and solver directly; only the simply supported
case also exercises the S2K writer/parser path. They do not exercise the GUI, command
line model editing, vendor SAP2000, design-code workflows, or the supplementary videos.
The first three checks use midpoint joint DOFs. The fourth explicitly tests one-element station output and exposed the previous omission of the distributed-load particular deflection between element ends. The dedicated
regression reproduced the defect: for the common values above, a one-element simply
supported beam reported `0.012 in` at midspan versus `0.015 in` exact (20% low), and a
one-element-per-span continuous beam reported `0.003 in` versus `0.006 in` exact (50%
low). Solver station recovery now integrates curvature and shear strain, splitting at
load discontinuities. The regression passes at both exact values. Thermal recovery is covered separately by `thermal_regressions.rs`; end-offset station recovery still needs a rigid-zone formulation. Both remain outside this introductory beam check.
