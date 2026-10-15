# Solver review repairs

Checkpoint: 2026-10-15. Scope is the concrete findings in `docs/review/structurax-latest/solver-review.md`. These repairs supersede the broader wave-4 capability claims where they conflict. They do not establish full SAP2000 equivalence.

## Corrected results

Nonlinear direct-history reactions now use the same linear stiffness base, nonlinear internal spring forces, viscous damping, Rayleigh damping and simultaneous load/acceleration snapshot used by integration. Effective `ke` is excluded from explicitly nonlinear DOFs during both integration and recovery. Recovery starts from the linear constitutive matrix to avoid catastrophic cancellation when effective `ke` is very large. Grounded Maxwell reactions use the saved force state, rather than the zero static damper law. Shear-offset section moments are converted once from internal spring forces.

The independent two-joint Hook regression varies `ke` from zero through 13 to `1e12`, includes viscous damping, and checks unchanged displacement and exact support/link balance. Separate tests check grounded Maxwell support balance and nonlinear shear-offset moments. Parent API tests additionally compare the full Hook history against the closed-form oscillator response.

History integration beginning from a supported nonlinear static state now computes its initial acceleration using the new initial force and the carried static displacement. The regression checks the independent step-load solution. History-origin continuation is explicitly rejected: terminal velocity, acceleration and Maxwell internal state are not persisted, so arbitrary transient continuation cannot be reconstructed correctly. This also rejects static or other cases requesting a history as their initial state. Static-to-static and the guarded static-to-history subset remain available.

Solid zero mass and weight modifiers are honored exactly. Negative/nonfinite modifiers reject; zero stiffness modifiers reject because that singular constitutive subset has not been implemented. A unit-cube test checks exactly zero mass and gravity loads, plus negative/NaN/infinite/zero-stiffness errors.

Cable prescribed motion at an otherwise unrestrained node now includes its actuator reaction and global force/moment resultant. Its regression checks equal/opposite axial reactions under an imposed elongation.

## Explicit supported boundaries

`src/supported_subset.rs` rejects assignments that the specialized wave-4 paths cannot represent:

- Cables are limited to cable-only nonlinear static models. Ordinary frames, links, areas, solids, joint springs, coupled springs, assigned multipoint constraints and rotated joint axes cannot be mixed into this path. Nonlinear cable results are not silently used for ordinary frames, and linear/dynamic/modal cable analysis is not silently substituted.
- Cable loading supports full-span uniform force intensity and uniform temperature. Partial-span/trapezoidal force loads, frame point loads, moment loading, patterned/gradient temperature, end offsets/releases, member caps, nonprismatic sections, joint moments and prescribed rotations reject. Nodal concentrated forces remain supported by the cable-network formulation.
- Target-end-tension initialization rejects; it is not inferred from or replaced by chord length. Specified undeformed length and the existing horizontal-tension initializer remain available. Horizontal initialization now checks its final compatibility residual before returning success.
- Frame tension/compression caps retain the bounded small-displacement nonlinear static subset with nodal axial loads and ordinary prismatic, unreleased, unoffset members. Axial span loads, axial selfweight/gravity, releases, offsets and nonprismatic properties reject. Geometric coupling and dynamic member caps reject. The regression checks a tension-only bar plus a parallel spring under both load signs, then checks explicit rejection of the reviewed axial distributed-load case.
- Nonlinear history with geometric nonlinearity rejects. Maxwell internal-state integration with Wilson-theta rejects because its extrapolated physical-step state was not correctly updated by the theta-stage formula. Newmark/HHT and Gap/Hook histories retain their existing scope.
- Solid surface/temperature history loads and prescribed support motion carried into history reject instead of disappearing from the spatial load vector.

These are explicit errors with cleared results, not zero responses or approximate substitutes. Other pre-existing unsupported-analysis guards remain in force.

## Convergence contract

Nonlinear history and cable-network solvers honor the requested iteration count and tolerance without minimum-iteration or tolerance clamps. Failed/nonfinite descent, unbracketed line searches and exhausted budgets return errors rather than publishing a state. Cable-network force tolerance is relative to the forces being balanced at free coordinates, including individual element forces before cancellation; this correctly scales selfweight networks whose external nodal force vector is zero. The former `1000*tolerance` acceptance on exhaustion was removed.

The catenary element now enforces its internal compatibility tolerance after the final iteration. High pretension exposed cancellation in `T_J-T_I` and sums of nearly opposite inverse hyperbolic sines. Algebraically equivalent rational differences and `ln_1p` expressions avoid this cancellation, allowing the existing C1 high-tension and all other cable benchmarks to pass without weakening the compatibility criterion. Singular flexibility is rejected instead of clamping its determinant. Cable constructors/evaluation reject nonfinite data and nonpositive thermal stretch.

Deliberately exhausted one-iteration cable and nonlinear-history regressions verify the error contract; adequate budgets converge and satisfy independently checked equilibrium.

## Verification and corpus limits

`tests/review_fixes.rs` contains 13 regression groups covering the corrected results, supported positive cases, rejected negative cases, old-result invalidation and iteration limits. Existing analytical frame, link, thermal, area, modal, solid and cable checks remain in place.

The optional native fixture tests distinguish success from intentional rejection:

- Native 7-001 and 7-002 contain target-end-tension metadata; complete imports now reject explicitly. Earlier results silently ignored that metadata. Their independent hand-built catenary/network benchmarks still verify the equations and references, but are not proof of full native case import equivalence.
- In the hand-built 6-003 regression, NLSTAT3 and NLDHIST2 request history-origin continuation and must reject. NLSTAT1/2, zero-state NLDHIST1 and static-initialized NLDHIST3 retain numerical checks. The unsupported NLDHIST2 benchmark row was removed rather than marked passing.
- Native 5-001, 6-004, 6-005 and native export/reimport numerical checks remain. Native 5-002/5-005 tests are import/solve smoke tests; they do not assert reference tip-deflection agreement.

Final validation commands:

```text
cargo test -p sap-solver --features vendor-fixtures
cargo clippy -p sap-solver --all-targets --features vendor-fixtures -- -D warnings
cargo test -p sap-api --test review_regressions
```

The optional fixture suite requires locally supplied vendor fixtures (`STRUCTURAX_VENDOR_FIXTURES`, otherwise `extracted/files`). Solver checkpoint: all 93 tests passed with `vendor-fixtures` enabled (86 portable tests plus 7 optional fixture tests); strict all-target Clippy passed. The parent API review suite passed all 5 tests after the reaction repair. Root records the final workspace-wide result after concurrent format/API changes settle.
