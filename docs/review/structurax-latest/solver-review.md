StructuraX / Antigravity Gemini wave 4 solver review
Read-only source review, 2026-10-15. No repository source was changed.
Scope: cable.rs, solid.rs, nonlinear.rs, dynamics.rs, lib.rs integration, benchmarks.rs and wave4_vendor_regressions.rs against the prior wave3 boundaries. This is a bounded review, not physics certification.

Validation performed
- cargo test -p sap-solver --test wave4_vendor_regressions -- --nocapture: all 7 pass.
- Independent public-API reproducer: /tmp/structurax-solver-repro/src/main.rs
  Run: cargo run --quiet --manifest-path /tmp/structurax-solver-repro/Cargo.toml
  All numerical reproductions below ran successfully and returned ostensibly successful analysis results.

[P1] Adding any cable silently changes the physics of the rest of the model
Location: crates/sap-solver/src/lib.rs:1824-1832 and 1634-1707 (especially 1702-1705).
The presence of one frame with cable_params routes the complete nonlinear-static case into a standalone 3-translation cable solver. That routine converts EVERY ordinary frame to a tension-only cable with L0 equal to its chord and never assembles joint springs, links, areas, solids, bending/rotational stiffness, or constraints from the normal model path. This is not restricted to cable-only models by validation.
Repro MIXED: ordinary axial bar E=1000,A=1,L=10, J axial spring k=100, I fixed, J load +10. Normal nonlinear-static solution is uJ=0.05. Add a completely disconnected, fully restrained cable between two new nodes: the original bar's result changes to uJ=0.10000000000582077. Expected unchanged 0.05; the joint spring was silently discarded.
Fix: assemble cables into the common nonlinear equilibrium system, or explicitly reject mixed elements/supports/unsupported cable constraints before analysis until that integration is implemented. Do not silently repurpose ordinary frames.

[P1] Nonlinear-history fixed-support reactions use effective linear stiffness, not converged constitutive forces
Location: crates/sap-solver/src/lib.rs:3344-3355, contrasted with nonlinear assembly at 3231-3268 and link output replacement at 3380-3392.
Integration zeros nonlinear ke and uses constitutive link force, but recovery still computes fixed reactions from the original k_base*u. The nonlinear nodal-force correction is never added. Thus reported force and support reaction contradict equilibrium; Rayleigh terms also use a different matrix than the integrator.
Repro REACTION: two-joint axial Hook, fixed I, J mass=1, knl=100, opening=0, ke=13, load FJ=10*t, dt=.01, 50 steps, zero damping. At the reported displacement snapshot uJ=.05960096128252512, link force=5.960096128252513 but reaction I=-.7748124966728266; sum=5.185283631579686 instead of zero. Changing effective ke changes the erroneous reaction without changing constitutive response.
Fix: recover reactions from the same linear base, nonlinear B^T*f state, damping matrix, acceleration and simultaneous external load used during integration. Grounded Maxwell links also need the saved dynamic spring force rather than static damper law (=0).

[P1] History continuation resets velocity, acceleration, and Maxwell constitutive force
Location: crates/sap-solver/src/dynamics.rs:514-545,579-619; crates/sap-solver/src/lib.rs:1531-1553 and 3258-3274.
initial_displacement_vector restores only u. NonlinearStepper initializes v=0, initializes a=0 whenever any initial displacement is supplied, and initializes every Maxwell f_prev=0. Only Gap/Hook force is reconstructed from displacement. The documentation's claim that link state propagates is false for Maxwell links, and even a wholly elastic oscillator loses momentum at the case boundary.
Repro CONTINUATION: J mass=1, grounded spring=100, constant force=1 from t=0, dt=.01. One 20-step NonlinearHistory case gives u(.2)=.01414633034461877. Split into 10 steps then a 10-step case with initial_case=H1, no added loads (the preceding constant load is retained): u(.2)=.006853534160215677. Boundary u(.1)=.004589977053972456 is transferred, but its velocity/acceleration are lost. A true continuation must agree with the uninterrupted discretized history.
Fix: persist and restore terminal u/v/a and each internal link state, with consistent end-load semantics. Reject unsupported history continuation until the full state is available. Add split-versus-uninterrupted regressions for elastic, Gap/Hook and Maxwell response.

[P1] Tension/compression limit equilibrium and recovery use incompatible axial-force definitions under member span loads
Location: crates/sap-solver/src/lib.rs:2136,2398-2410; crates/sap-solver/src/nonlinear.rs:217-232; crates/sap-solver/src/cable.rs:736-742.
The nonlinear constitutive system uses the mean axial fixed-end force (fJ-fI)/2, while recovery clamps fJ alone and replaces both end forces with equal/opposite values. A member with axial distributed loading has nonzero fI+fJ, so this replacement destroys end-force/load equilibrium and can report forces beyond the requested limit.
Repro TC SPAN: fixed-fixed bar E=1000,A=1,L=10, uniform local-1 load +1 over the full span, compression limit=0 (has_comp=true), nonlinear-static. Returned support reactions are (-5,-5), but recovered axial force is 0 at I and -10 at J. The latter violates the zero compression limit and disagrees with the end forces implied by the reactions. The analysis reports success.
Fix: formulate member limits with consistent axial strain/load integration and end-force equilibrium; until then reject capped members with axial distributed/point loads (and other unvalidated releases/offset interactions). Add endpoint-force versus reaction and section-limit regressions, not just the scalar constitutive-helper test.

[P1] Cable partial/trapezoidal loads are silently converted to full-span uniform loading
Location: crates/sap-solver/src/lib.rs:1670-1679.
The cable path averages val1/val2 and applies that constant over the entire cable, never reading rel_dist1/rel_dist2. Concentrated frame loads are also absent from the cable assembly. This changes external loading rather than approximating an explicitly chosen cable discretization.
Repro CABLE SPAN: fixed endpoints, chord10,L0=12,E=1000,A=1, no selfweight, uniform global-Z -1. Changing the distributed load span from [0,1] to [0,.5] leaves total vertical support reaction exactly 12 in both analyses. The solver entirely ignores the half-span assignment.
Fix: split/integrate cable loading correctly or reject nonuniform/partial-span and point loads explicitly; retain only a clearly guarded full-span uniform subset until implemented.

[P2] target_tension_i is validated but ignored during cable initialization
Location: crates/sap-solver/src/lib.rs:1688-1700; docs/reconstruction/solver-wave4.md:21.
The initialization branch handles undeformed_length and horizontal_tension only. A target_tension_i-only assignment falls through to L0=chord, silently discarding the requested pretension, despite the docs claiming support.
Repro CABLE TARGET: fixed endpoints, chord10,E=1000,A=1, zero distributed load, only target_tension_i set. Requested 5 -> reported axial force 0; requested 50 -> force 0. Expected nonzero specified pretension, or an explicit unsupported-input error.
Fix: solve the end-tension compatibility initialization or reject this parameter and correct the capability claim.

[P2] Zero solid mass/weight/stiffness modifiers silently restore nominal properties
Location: crates/sap-solver/src/solid.rs:331-334,643-648,701-706.
All nonpositive modifiers are replaced by 1.0. A zero mass modifier therefore cannot remove a solid's mass; zero weight still adds gravity loading. Negative/NaN inputs are silently normalized too. Default modifiers already equal one, so this is not necessary for missing-data defaults.
Repro SOLID: unit cube, default steel material, section.modifiers[6]=0. compute_solid_stiffness_and_mass returns total translational-axis mass=7.344553233057461e-7, equal to the original material density, instead of zero.
Fix: validate finite/nonnegative modifier input; honor zero mass/weight exactly. For zero stiffness that the formulation cannot handle, return an explicit error rather than silently changing it to one. Add zero/negative/NaN modifier checks.

[P2] Wave4 reinstates silent convergence-control changes and success without the requested residual check
Location: crates/sap-solver/src/lib.rs:1717-1718; crates/sap-solver/src/cable.rs:668-671; crates/sap-solver/src/dynamics.rs:744-745,788-789.
Read-only code evidence: cable calls force at least80 iterations and tolerance<=1e-9, and accept residual up to1000*tol at exhaustion. Nonlinear dynamics forces at least20 iterations, clamps requested tolerance to [1e-12,1e-4], and a second failed descent-direction check breaks out into successful state publication instead of returning NonConvergence. This regresses the explicit settings/error contract repaired in wave3. The failed-descent path was not numerically triggered during this bounded review.
Fix: honor the case settings or reject unsupported values, and gate every success exit on a finite residual satisfying the requested criterion. Return a clear error for failed line search/descent. Add deliberate budget-exhaustion and tolerance-scaling tests.

Validation/capability claims to narrow before publishing
- The 7 native regressions pass but do not cover the failures above.
- wave4_vendor_regressions.rs:45-58 checks only nonempty results for native 5-002 and 5-005, not reference tip deflections. docs/solver-wave4.md:14 claims native reference agreement. benchmarks.rs contains no 5-005 verification entry. Add numerical comparisons or describe these as import/solve smoke tests.
- The source-distributed vendor test directory is hard-coded to /home/edo/REA/extracted/files (wave4_vendor_regressions.rs:9-10); it will fail on a normal GitHub checkout unless fixture handling is changed. Do not assume these bundled proprietary binaries can be redistributed.
- The existing hand-built 6-003 case uses heavy damping and checks final/quasistatic states; that does not validate momentum/internal-state continuity between arbitrary transient cases.
- New solids are assembled for dynamics, but assemble_static_pattern_rhs (lib.rs:3002-3110) handles joint and frame loads only, not solid surface/thermal/selfweight loads. This is a further inspected unsupported integration boundary, not separately reproduced here. Guard or extend it before claiming arbitrary solid history loading.

Recommended publishing posture: retain the wave4 work as experimental implementation, fix or fail-closed the reproduced P1 cases before presenting these paths as supported production analysis, and preserve analytical regressions outside the vendor corpus.
