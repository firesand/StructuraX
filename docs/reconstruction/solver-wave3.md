# Solver reconstruction wave 3 — nonlinear Gap/Hook equilibrium

Checkpoint: 2026-10-15. This implements a bounded nonlinear static formulation, not complete SAP2000 v19 nonlinear-analysis parity. No licensed vendor executable or nonlinear vendor-output dataset was available for differential comparison.

## Constitutive scope and source

The supplied July 2016 `extracted/files/csirefer.pdf`, printed pp.276-278 and 286-287, establishes these uncoupled laws for each explicitly nonlinear internal link deformation `d`:

```text
Gap:  f = k * min(d + opening, 0)
Hook: f = k * max(d - opening, 0)
```

`k` and `opening` must be finite and nonnegative. The active branch tangent is `k`; the open branch tangent is zero. The tangent at the branch boundary is chosen as zero. The laws are conservative, path-independent and have no hysteretic memory. A zero nonlinear stiffness produces zero constitutive force; it must not be mistaken for an instruction to use effective linear stiffness.

The manual explicitly distinguishes degrees of freedom with nonlinear properties from those that remain linear. The root agent therefore added `LinkProp.dof_nonlinear: [bool; 6]`. Only active, nonfixed DOFs with this flag use the nonlinear law. Other DOFs retain `ke`, even in a Gap/Hook property. Unstressed linear cases retain `ke` for all active, nonfixed DOFs. Fixed link DOFs still use the existing `1e10` penalty stiffness; exact fixed-link constraints have not been implemented.

The format agent added `LinkProp.nonlinear_data_complete` because native nonlinear-parameter layouts are not completely mapped. Nonlinear cases reject incomplete imported constitutive data **before** considering flags: an unknown source flag vector must not silently become linear behavior, and an unknown opening must not silently become zero. The format agent owns source-field verification and import/export warnings. This solver does not infer flags from `k_nl`.

## Numerical implementation

`src/nonlinear.rs` uses the oriented link deformation operator from wave 2:

```text
d = B*u
R(u) = K_linear*u + sum(B^T*f(d)) - F
K_tangent = K_linear + sum(B^T*diag(df/dd)*B)
```

The linear part is reassembled with the nonlinear DOFs' effective stiffness set to zero in an analysis-only copy. This avoids subtracting a potentially enormous `ke` from an already accumulated matrix and losing small true linear terms to cancellation. Original model properties remain unchanged. Tests vary `ke` from zero to `1e20` with identical nonlinear displacements and forces.

The solver starts with prescribed constrained displacements and zero free displacement. Each iteration tries a Newton direction from the actual branch tangent. If an initially open contact makes that tangent singular, it uses an all-closed reference stiffness to obtain a descent direction. That matrix is only a search preconditioner; no reference spring force enters the residual or reported results.

A convex one-dimensional line search brackets the zero directional derivative by expansion and then bisects it. This can traverse a finite opening even when there is no parallel supporting stiffness at the starting state. A load pushing an unsupported hook/gap in the nonresisting direction fails explicitly when no resisting contact is reachable. This is a convex small-displacement formulation; it is not a general nonconvex, path-following or snap-through algorithm.

Convergence requires the infinity norm of the **free-coordinate** force residual to satisfy the requested relative tolerance. The scale uses free external/internal nodal forces; unrelated very large loads on restrained nodes do not loosen the equilibrium criterion. Nonfinite residuals, loads and prescribed motions fail. The requested iteration budget is honored, and exhaustion returns `SolverError::NonConvergence` with case, iteration count, residual and reason. Failure clears partial/stale results and leaves the model unlocked. There is no load stepping or constitutive history continuation in this slice.

The nonlinear nodal-force correction is included in restrained and prescribed-motion reactions. Link output uses the actual constitutive spring force and the existing J-end shear-offset section-moment conversion. Link rotations, differing joint axes, shear distances, grounded links and linear frame/area loading pass through the same assembly and recovery paths used by linear cases.

## Related correctness fixes

- **Grounded link reactions.** One-joint links now contribute their `-B^T*f` support force to the joint reaction sum and global base resultant, even when the modeled joint itself has no restraint. Direct-history snapshots also include grounded-link viscous and stiffness-proportional support forces. These are combined support reactions at a modeled joint; separate actuator-versus-grounded-element reaction channels are not represented.
- **Prescribed motion on an otherwise free joint.** The static solver already treated a nonzero prescribed displacement as a constraint but omitted its actuator reaction from output. Independent reviewer `beam_workflows` reproduced missing +10 balancing a two-joint hook's -10 fixed support reaction. Prescribed coordinates are now included in support detection and reaction recovery. A regression checks actuator/contact/global equilibrium.
- **P-Delta convergence settings.** The older geometric iteration silently capped `max_iterations` at 20, floored tolerance at `1e-9`, and returned its final unconverged iterate. It now honors the requested positive count/tolerance, checks relative axial-force convergence, and returns an explicit error if the budget is exhausted. A solve counts as one iteration on this path. The nonlinear contact path counts completed displacement updates, with an initial residual check before any update. This improves termination reporting; it does not replace the existing P-Delta formulation with general large-displacement mechanics.
- **Nonfinite inputs.** Active joint force/displacement assignments and load-case scale factors are validated before they can be silently skipped by comparisons against NaN.

## Tests and review

`tests/nonlinear_link_regressions.rs` contains fourteen independent regression groups:

1. Gap and Hook open/closed branches under positive, negative and zero loads with a parallel linear spring; force and reaction equilibrium.
2. Initially open contacts with no parallel stiffness, openings up to 100, and `ke` values 0, 13 and `1e20`; exact post-closure displacement and force.
3. Explicit nonlinear flags, valid zero nonlinear stiffness, and linear-case effective stiffness.
4. Contact activation involving rotation, nonzero shear distance and rotated joint axes; J-end moment and global equilibrium.
5. Coupled nonparallel contacts against independently constructed vector equilibrium; deliberate iteration-limit failure with result invalidation.
6. Nonresisting load direction, negative/nonfinite constitutive parameters and invalid iteration controls.
7. Grounded contact, parallel spring and global reaction moment about the origin.
8. Free-residual scaling in the presence of a `1e20` unrelated restrained load, plus simultaneous stiffness/load rescaling by `1e-9` and `1e9`.
9. P-Delta iteration budget 1 fails cleanly; budget 2 converges on an independently loaded axial/bending frame at tolerance `1e-12`.
10. Explicit unsupported errors for combined geometric/contact analysis and unstored nonlinear-tangent continuation.
11. Incomplete imported nonlinear data rejects with either false or true flags; its effective linear analysis remains available.
12. Uniformly loaded cantilever closing a grounded gap: independent Euler-Bernoulli compatibility, contact force, global force/moment equilibrium and every recovered displacement station.
13. Prescribed unrestrained contact DOF includes its actuator reaction.
14. Nonfinite applied forces and prescribed displacements do not falsely converge.

The cantilever check uses `L=10`, `EI=2000`, downward uniform load 1, gap opening 0.1 and contact stiffness 100. The free tip deflection is 0.625 and beam tip stiffness is 6, giving the contacted magnitude `(6*0.625 + 100*0.1)/(6+100)`. The full station displacement equals the sum of the uniform-load beam solution and the opposing contact-force solution.

One additional test in `coordinate_link_regressions.rs` checks a grounded viscous-link ramp history with no joint restraint, including Rayleigh stiffness damping and the global reaction moment. Earlier unsupported-type regression now uses PlasticWen rather than treating newly implemented Gap behavior as unsupported.

Read-only review by `beam_workflows` independently checked active/nonlinear/fixed selection, zero nonlinear stiffness, open/closed contact branches and scaling. Its concrete prescribed-motion reaction finding was repaired and tested; final review confirmation is reported by the parent.

Validation: `cargo test -p sap-solver` passed all 67 tests (15 unit, 13 analysis, 4 beam tutorial, 14 coordinate/link, 14 nonlinear link, and 7 thermal). `cargo clippy -p sap-solver --all-targets -- -D warnings` passed after the shared format initializer correction. Parent performs the final workspace-wide checkpoint after other owned changes settle.

## Explicit remaining boundaries

- Combined P-Delta/large-displacement/contact analysis remains rejected. The line-search proof assumes a convex small-displacement elastic base.
- A linear/modal/history case cannot use an unstored end-state nonlinear link tangent from an initial nonlinear case; that continuation remains rejected.
- Nonlinear dynamics, unloading/history-dependent plasticity, other nonlinear link types, FNA, cable/catenary elements and frame tension/compression limits remain unsupported.
- Frame tension/compression caps were not implemented in this slice: correct recovery must account for member axial span loads, thermal strains, releases, offsets and continuation state, rather than replacing a beam's axial stiffness with an unexplained scalar switch.
- General massless damping initialization, exact fixed-link constraints, production-scale sparse matrices and full solver outcome traces remain future work.
- Named-coordinate vendor angle ordering, native constitutive parameter mapping, all nonlinear load combinations, Windows runtime and vendor differential equivalence remain unverified.
