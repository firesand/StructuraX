# Solver reconstruction wave 2

Checkpoint: 2026-10-15. These changes implement additional mathematical paths in the experimental reconstruction. They do not establish SAP2000 v19 parity. No licensed vendor executable was run or compared.

## Implemented

### Named Cartesian load coordinates

`coordinates.rs` normalizes active named-coordinate loads in an analysis-only model copy. Original assignments and coordinate-system names survive analysis unchanged. Joint force/moment and prescribed translation/rotation vectors are rotated into global components. Frame distributed forces/moments, frame point forces/moments, and uniform area forces are expanded into equivalent global component assignments. Existing element and joint transformations then act normally in assembly and recovery.

The model's `angles_deg = [RZ, RY, RX]` is interpreted as intrinsic, right-handed `Rz * Ry * Rx`; this differs deliberately from the joint-local tilt convention implemented elsewhere. Origins do not translate free vectors or create additional moments at their assigned application locations. A regression exercises all three axes with a nonzero origin. The 3-angle named-grid ordering is an explicit reconstruction input convention; vendor differential validation of imported 3-angle systems remains required.

Direction codes 1-3 remain element-local; 4-6 use the named axes; 7-9 use the named axes and projected line-length factor (sine for forces, cosine magnitude for moments). The projection is computed against the original named direction **before** splitting globally, avoiding the incorrect sum of separately projected global components. Gravity direction codes remain global -Z. Missing active named systems and nonfinite coordinate definitions fail explicitly; unused assignments retain prior inactive behavior. Projected area loading remains rejected because surface projection is not implemented.

### Oriented finite-length and grounded links

`link.rs` implements the CSI six internal-deformation operator `d = B*u`, with element-local axes transformed through each participating joint's axes. Assembly uses `K = B^T*diag(ke)*B`, including shear-offset translation/rotation coupling and finite-length rigid arms. The local-2 bending sign is handled using `dr2 = r2i-r2j`. Rigid-body rotation produces zero internal deformation, including nonzero shear distances.

Finite links use the I-to-J axis. Zero-length and one-joint links default to local 1 along global +Z and support roll angle. A one-joint object's point is treated as the active J end with I grounded. The implementation currently uses a fixed `1e-12` geometric zero-length tolerance; the vendor's configurable Auto Merge Tolerance is not represented. Advanced link orientation vectors are not represented by the current model.

Recovered `CaseResults.link_forces` contain local section resultants **at J**, including `M2 = fr2-dj3*fu3` and `M3 = fr3-dj2*fu2`. There is still only one link section-result vector, not a station diagram. Existing fixed-link-DOF behavior retains its `1e10` penalty stiffness; this is a finite penalty approximation, not an exact constraint implementation.

Missing link properties and invalid/nonfinite link geometry, stiffness, damping, or mass now fail rather than disappearing. Translational link mass is halved at each connected end; for a grounded link, half is grounded. Link self-weight is likewise halved and applied downward in each active self-weight pattern. Rotational link inertia is not represented in the model.

### Effective viscous link damping

Direct integration now assembles `C_link = B^T*diag(ce)*B` and adds it to case Rayleigh damping. Newmark average/linear acceleration, HHT, and Wilson-theta use this complete matrix in their effective equations. This supports rotated and nonproportional link damping with positive participating masses. The simultaneous displacement-norm peak summary now includes link section resultants from both elastic strain and viscous strain rate at that same sample.

General element damping touching a free massless coordinate is **explicitly rejected**: the existing initializer handles proportional massless blocks, not arbitrary mixed first-order/algebraic damping constraints. Merely adding dashpots to the matrix and retaining algebraic projection would silently erase their forces; that is not done. General DAE reduction/initialization is a required follow-up. Per-step velocity/acceleration and link-force histories and spectrum/modal damping contributions remain outside this slice. The follow-up below corrects restrained-joint reaction snapshots using damping, inertia and applied support loads; comprehensive support-force histories are not provided.

### Span-load frame deflection recovery

The independent beam-tutorial agent exposed an existing defect: cubic interpolation of end DOFs omitted load-induced curvature between the nodes. A one-element simply-supported uniform-load beam returned 0.012 in instead of 0.015 in at midspan; a two-span beam with one element per span returned 0.003 instead of 0.006 in.

`recover_frame_stations` now integrates equilibrium section forces to recover axial strain, bending curvature, and transverse shear strain, enforcing the solved end translations. Quadrature intervals split at concentrated loads and distributed-load boundaries. This recovers the span-load particular solution and Timoshenko shear contribution without requiring artificial output nodes. For prismatic, piecewise-polynomial loading the five-point integration is exact to the polynomial order involved. Nonprismatic integration remains numerical; there is no new claim of exact variable-section or geometric-nonlinear displacement recovery.

Thermal eigenstrains are included by the follow-up below. Members with end length offsets retain the previous interpolation pending consistent flexible-face/rigid-zone kinematics and load-location semantics. End offsets are not presented as covered by the new regression.

### Explicit applicability checks

Active nonzero edge or patterned area pressure is rejected; uniform top/bottom pressure remains supported. Catenary cable parameters are rejected rather than analyzed as beam parameters. Nonlinear-static cases with nonlinear link property types or enabled frame tension/compression limits reject the unimplemented constitutive iteration. Linear cases may still use a nonlinear link's effective linear stiffness, as the CSI manual specifies. These checks expose pre-existing gaps; they do not implement nonlinear material behavior.

## Evidence and verification

Primary analytical reference: supplied `extracted/files/csirefer.pdf`, July 2016, extracted text in `target/reconstruction/csirefer.txt`:

- Printed pp. 254-256: link longitudinal axis, zero-length default +Z, default transverse axes and roll.
- Printed pp. 261-263: internal two-joint and grounded link deformation equations and sign conventions.
- Printed pp. 264-267: linear/nonlinear property behavior, internal springs, section resultants and shear moment arms.
- Printed pp. 271-272: translational mass and weight split between ends, including the grounded half.

Public CSI reference cross-check: https://docs.csiamerica.com/manuals/sap2000/CSiRefer.pdf . The supplied manual was the equation source. The vendor's grid help confirms named coordinate/grid systems but does not establish this reconstruction's 3-angle ordering; no stronger vendor parity claim is made.

`crates/sap-solver/tests/coordinate_link_regressions.rs` adds thirteen independent checks:

1. Named joint force/moment and prescribed displacement/rotation equivalence to explicitly transformed global assignments, including rotated joint axes and preservation of original names.
2. Named projected trapezoidal frame loading and point moments, including total reaction equilibrium.
3. Named uniform area load global equivalence and total force.
4. Local link rotations, unequal stiffnesses, nonzero shear offsets, joint-axis rotations, displacement recovery, J-end forces and reaction moments against a prescribed independent strain calculation.
5. Grounded-link axes, half self-weight and local force sign convention.
6. All four direct integrators against the damped SDOF closed form, with named loading and effective mass explicitly checking the grounded half-mass convention.
7. Missing link property and massless dashpot rejection with no stale results.
8. Three-angle right-handed named basis and nonfinite data rejection.
9. Finite-link rigid-body rotation with nonzero offsets produces zero strain and force.
10. Rotated nonproportional two-coordinate viscous damping against two independent closed-form modal responses.
11. Edge/patterned pressure and nonlinear link applicability boundaries; linear effective stiffness remains accepted.
12. Ramp history reactions with nonzero viscous force, Rayleigh stiffness damping, direct fixed-joint loads, and independent closed-form inertia equilibrium.
13. Projected moment cosine versus force sine under named/global directions and reversed member connectivity.

The beam-tutorial agent owns `tests/beam_tutorial_regressions.rs`, including the independent single-element station tests that exposed and verified the deflection correction. Existing missing-coordinate regression was renamed to clarify it rejects **missing** definitions rather than all named systems.

Validation commands: `cargo test -p sap-solver` and `cargo clippy -p sap-solver --all-targets -- -D warnings`. Both pass at this checkpoint. Exact integrated counts may increase as the independent beam agent adds further checks; parent integration owns final workspace totals.

## Remaining work

True gap/hook and other constitutive iteration; cable/catenary behavior; nonlinear dynamics; general massless damping DAE initialization; exact link fixed constraints; model-driven merge tolerance; rotational link inertia; advanced link axes; projected area loads; end-offset station recovery; comprehensive dynamic support and force histories; vendor differential cases and native-file semantic validation. The model-copy normalization incurs additional memory proportional to model size; production-scale memory/sparse matrix work is not included.

## Reviewed follow-up: thermal fields, reaction snapshots and tapered gravity

The parent requested additional thermal recovery and independent review. Seven tests in `thermal_regressions.rs` and two additional coordinate/link regressions validate the following corrections.

- **Thermal fixed-end loads and station fields.** The former endpoint formulas gave the wrong shear sign for longitudinally varying gradients and were incompatible with finite shear stiffness. Thermal fixed-end force now equals `-K_flexible * u_free`, where the stress-free J-end motion is integrated from the actual linear thermal strain/curvature profile with I fixed. This uses the same flexible stiffness as assembly, including section modifiers and nonprismatic flexibility. Station recovery adds axial `alpha*T` and transverse `-alpha*gradient` eigenstrains to elastic strain/curvature. It now recovers nonlinear internal axial displacement under linearly varying temperature, including restrained members with zero end displacement. The shear modulus fallback matches stiffness assembly. The supplied manual printed p.140 specifies positive gradient and linear joint-pattern interpolation; p.141 describes stress-free strain-load motion. Tests check free and fixed members, both bending planes, constant and varying profiles, finite/shear-suppressed stiffness, and a stepped-section series-flexibility solution. All four original thermal tests failed before these fixes and pass afterward.
- **History reaction snapshot.** Independent reviewer `editing_wave2` reproduced a ramp-loaded link with reported axial force `1.8725225141009403` but support reaction `-1.4951148335682718`; the missing `0.3774076805326685` was dashpot force. The reaction is now evaluated from `K_base*u + (C_link + beta*K_base)*v + alpha*M*v + M*a - F` at restrained coordinates at the same displacement-norm peak sample. Applied fixed-coordinate loads are retained in the sampler while the integrator still enforces zero constrained motion. Base reactions receive the transformed force and moment corrections. The reviewer reran the reproducer and confirmed exact reaction/link balance. A ramp regression checks nonzero velocity, direct support loading, stiffness-proportional damping and closed-form inertia balance. This is a simultaneous physical snapshot, not a support-reaction envelope or a complete reaction time history. For HHT/Wilson, physical endpoint residuals retain their method-specific discretization behavior; the reported reaction is the physical force at the returned state.
- **Projected distributed moments.** Manual printed p.140 explicitly distinguishes sine scaling for projected forces from cosine scaling for projected moments. Both the old global resolver and the new named normalization previously used sine for both. Distributed moment projection now uses the cosine magnitude; global and named cases are tested at 30 degrees, with reversed connectivity as well as normal connectivity. Concentrated loads are not length-projected.
- **Tapered gravity equilibrium.** The reviewer found `recover_frame_stations` used integrated varying weight for partial-span forces but midpoint weight for the full-span moment correction. A taper from area 1 to 3 over length 12 returned J-end moment 24 when the supplied J moment was zero. Partial and full-span gravity resultants now share the same quadrature, split at nonprismatic segment boundaries. The public reproducer and a full cantilever model both pass; every station matches exact variable-weight shear/moment and base reactions match total force and first moment. The deflection/thermal quadrature also splits at section boundaries, avoiding integration across stepped stiffness jumps.

Final solver checkpoint after these changes: **52 tests pass** (15 unit, 13 prior analysis, 4 independent beam, 13 coordinate/link, 7 thermal/tapered-section); strict all-target solver Clippy passes. The parent owns the final workspace-wide check. End-offset station/assembly semantics were reviewed but not changed: altering the output curve alone would not resolve load positions on rigid offsets, so this remains a stated boundary.
