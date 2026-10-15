# Solver reconstruction wave 1

Checkpoint: 2026-10-15. This is a verified linear-dynamics implementation slice, not SAP2000 v19 parity or certification for engineering use. No original SAP executable was run. The supplied installer/extracted files remain evidence inputs.

## Implemented in this wave

- Added `crates/sap-solver/src/dynamics.rs`, a shared dense linear stepper used by the existing direct-integration history entry point.
- Implemented HHT with the CSI parameter convention `-1/3 <= alpha <= 0`, `gamma = 1/2 - alpha`, and `beta = (1-alpha)^2/4`. Both damping and stiffness enter weighted equilibrium; this is not the former approximation that only changed Newmark coefficients.
- Implemented Wilson-theta using an extended step `h = theta*dt`, extrapolated endpoint load, a linear-acceleration solve at the extended time, and interpolation of acceleration followed by integration back to the physical output time. `theta=1` reduces to Newmark linear acceleration. The model now supplies `wilson_theta`, default `1.4` with backward-compatible deserialization, implemented by the root agent.
- Retained Newmark average and linear acceleration through the same implementation. HHT `alpha=0` reproduces average acceleration.
- Added consistent initialization of massless, unrestrained coordinates. Massive coordinates begin with zero displacement and velocity. Massless displacements solve their static equilibrium equations; initial velocities/accelerations satisfy the right-hand differentiated equations for the piecewise-linear load. This removes the previous rejection of nonzero initial force on a massless coordinate.
- Retained the existing Rayleigh matrix `C = alpha_mass*M + beta_stiffness*K`. Massless coordinates with stiffness damping retain their first-order dynamic equation; they are not incorrectly condensed as purely algebraic coordinates. Without stiffness damping, algebraic displacements, velocities and accelerations are recovered after each physical step.
- Reject invalid HHT/Wilson parameters, nonfinite dynamic data, negative mass, and negative/nonfinite Rayleigh damping instead of clamping input. Wilson theta is accepted for values at least one; conditional/unconditional stability is a property of the chosen value and time step, not an accuracy guarantee.
- Preserved prior result invalidation, dependency ordering, geometry checks, named-coordinate rejection, and unsupported analysis/load guards.

## Equations and source decisions

HHT uses the endpoint-interpolated load convention:

```text
M*a[n+1] + (1+alpha)*(C*v[n+1] + K*u[n+1])
           - alpha*(C*v[n] + K*u[n])
  = (1+alpha)*F[n+1] - alpha*F[n].
```

The effective matrix is `(1+alpha)*K + (1+alpha)*gamma/(beta*dt)*C + 1/(beta*dt^2)*M`. This is the usual weighted equilibrium form with linear interpolation of applied force across a solver interval. If an interval crosses a time-function knot, the endpoint interpolation spans that knot. No automatic substepping at knots is implemented; align output steps with knots or refine the time step.

Wilson uses `F_stage = F[n] + theta*(F[n+1]-F[n])`, not a direct sample at the extrapolated time. The stage solve uses `K + 3/h*C + 6/h^2*M`. The stage acceleration is interpolated to the physical time before calculating output velocity/displacement. The returned time grid is always `n*dt`.

For a massless coordinate block `z`, initialization first solves `Kzz*uz=Fz(0)` with massive displacement zero. With no stiffness damping, `Kzz*vz=Fz'(0)`, and initial acceleration follows the differentiated algebraic constraint. With stiffness damping `b>0`, initial velocity is zero and `Kzz*az=Fz'(0)/b-Kzm*am`. These conditions follow directly by differentiating the massless rows of `M*a+C*v+K*u=F` and using the supplied proportional damping matrix. Singular loaded massless blocks still fail through the linear algebra solver.

Evidence inspected:

- Supplied `extracted/files/csirefer.pdf`, printed pages 405-406, PDF page 428 for the parameter/damping discussion. Page 428 was rendered and visually inspected. It establishes CSI's HHT sign/range convention, equivalence to average acceleration at zero, and proportional damping definition. It does not specify all implementation details.
- [DIANA 10.0, HHT, equations 48.47-48.52](https://manuals.dianafea.com/d100/Analys/node558.html): HHT kinematics, weighted equilibrium and parameter range. DIANA writes the external force at the intermediate time; this implementation uses linear interpolation of the physical endpoint samples.
- [DIANA 10.0, Wilson, equations 48.53-48.56](https://manuals.dianafea.com/d100/Analys/node559.html): extended-step kinematics and force extrapolation. Interpolation back to physical time follows the assumed linear acceleration.
- [OpenSees HHT documentation](https://opensees.github.io/OpenSeesDocumentation/user/manual/analysis/integrator/HHT.html): independent check of the weighted tangent and Newmark kinematics. Its parameter uses a shifted convention; it was not copied into the CSI input field. Its displayed force/time notation is not used as evidence of exact CSI load sampling.

## Verification completed

`cargo test -p sap-solver` passes 28 tests: 15 unit tests and 13 integration regressions. The prior vendor-fixture/reference tests remain green; they are not HHT/Wilson vendor-result comparisons.

New independent or cross-method checks:

1. All four methods converge toward the closed-form damped SDOF step response (`m=2`, `k=8`, `c=0.8`, `F=3`). Halving `dt` from 0.02 to 0.01 reduces maximum displacement error by more than 3.5, with fine-grid error below 0.00012.
2. HHT satisfies weighted equilibrium under varying force and physical damping to residual below `1e-10` in the tested SDOF problem.
3. HHT at zero equals Newmark average; Wilson at one equals Newmark linear, including velocity and acceleration histories.
4. For a high-frequency unforced oscillator, average acceleration conserves energy in the tested interval; negative-alpha HHT and Wilson attenuate it. This distinguishes the added numerical dissipation from merely changing the method label.
5. A two-coordinate system with one massless coordinate matches the independently condensed analytical ramp response. Its algebraic displacement constraint remains satisfied to `1e-12` in the test.
6. Massless initialization with Rayleigh stiffness damping matches both constant-offset and time-varying first-order analytical constraints. This checks that stiffness damping on zero-mass coordinates is retained.
7. Full `SapModel` HHT/Wilson cases produce the physical output time grid and agree with the undamped SDOF closed form.
8. A full frame model with a tip moment, tip translation mass and massless tip rotation starts with the required finite rotation. All four methods match the condensed cantilever response. This verifies the new initialization through assembly, coordinates and model-result storage, not only through the mathematical stepper.
9. Out-of-range/nonfinite integration parameters are rejected rather than silently adjusted.

No numerical output from a licensed running SAP2000 v19 installation was available for direct comparison. The tests establish the implemented linear formulations and bounded model paths; they do not establish identical solver ordering, rounding, defaults, result envelopes or all vendor semantics.

## Remaining boundaries

- Nonlinear direct integration remains rejected; there is no constitutive iteration or nonlinear state update in this stepper.
- Initial-case continuation, prescribed history support motion, and history element/self-weight loads remain rejected. Supported external history loads are joint forces/moments and uniform acceleration.
- Named load coordinate systems remain explicitly rejected on active affected paths. They were assessed but deliberately not implemented in this checkpoint.
- Time-history model output retains displacement traces only. Velocity and acceleration are internal to the stepper and verified by unit tests. The summary result is the simultaneous displacement vector with largest Euclidean norm, not a componentwise history envelope. Per-step dynamic reactions/member forces and inertial support-reaction recovery require separate work.
- Damping supplied through link properties is not assembled into this stepper; only case Rayleigh damping reaches it. This is a pre-existing model capability gap, not covered by the new damping verification.
- Dense matrices and factorization remain; there is no production-scale sparse solver claim. Mass is diagonal/lumped. No automatic time-step refinement, load-knot substepping, frequency-shift tuning or integration error estimator is implemented.
- General load-case/load-combination dependency and result semantics still need broader verification. Existing safeguards cover only the documented bounded paths.

## Next-work handoff: precise guard/implementation audit

These observations are read-only findings for the next phase, not fixes made in this wave:

1. `assemble_elastic_stiffness_and_mass` and link-force recovery read effective link stiffness, active/fixed DOFs and mass. They do not implement `LinkProp.link_type`, nonlinear parameters, viscous coefficients, local orientation, shear offsets or weight. A missing referenced link property currently causes the link to be skipped; this should fail as a missing-reference error.
2. Nonlinear-static cases containing nonlinear links need constitutive iteration or an explicit unsupported-input error. Do not indiscriminately reject a nonlinear link property in a *linear* case: effective linear stiffness may be the requested vendor behavior. Verify that distinction against the relevant vendor manual before choosing guards.
3. Active nonzero `LinkProp.ce` in direct integration requires damping assembly or explicit rejection. Nonzero link `angle_deg`, `dj2`/`dj3`, non-global link orientation, and rotated participating joint axes require proper local/global transforms and coupling. Current matching-component assembly is insufficient. Link self-weight also needs assembly before claiming gravity support.
4. `FrameObj.cable_params` is not read by the solver. Catenary cable definitions must not be presented as implemented beam behavior. `FrameObj.tc_limits` is likewise unread; enabled limits in nonlinear static require iteration or rejection. Frame insertion-point/joint-offset behavior should be audited separately.
5. Named-coordinate load support should start with a validated coordinate transform convention and rotated-model equivalence tests for all five guarded assignment paths. Removing the current guards alone is not an implementation.
6. For HHT/Wilson vendor equivalence, collect a small licensed-reference output set with constant/ramp forcing, nonzero damping, massless rotations and multiple parameter choices; compare full time traces. Preserve source cases, outputs, units and parameter metadata together.

Resume commands:

```sh
cargo test -p sap-solver
cargo test -p sap-solver dynamics::tests -- --nocapture
cargo test -p sap-solver tip_moment_history_initializes_massless_frame_rotation_consistently
```
