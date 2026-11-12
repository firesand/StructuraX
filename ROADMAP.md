# StructuraX roadmap

The priority is reliable, explicitly scoped behavior before additional feature claims.

1. Extend the bounded [review repairs](docs/review/structurax-fixes/REVIEW.md): target I-end
   cable tension (`target_tension_i`) is implemented and verified; implement remaining rejected
   cable/member-load combinations (e.g., `tc_limits` with span axial loads) with independent regressions.
2. Complete native representation of remaining guarded assignments (`local_axes_matrix`) and
   independent shear-rebar yield data (`fys != fy`), preserving strict data-loss and
   referential-integrity checks (`TYSpring` 21-term coupled/local springs, orthotropic `S_Material`,
   and `S_CableData` `m_CableType` `3/5/8` now roundtrip).
3. Validate native export against a compatible licensed SAP2000 runtime, including
   unsupported-field diagnostics and critical assignment comparisons.
4. Complete mixed-element nonlinear equilibrium and modal-history continuation.
   Reactions and direct-integration `TransientTerminalState` continuation (`LinearDynamic`,
   `NonlinearHistory`, `NonlinearDynamic` -> static/direct-history) are verified; modal
   `LinearHistory` continuation still rejects explicitly.
   Plastic Wen links and displacement-controlled nonlinear static cases are implemented within
   the bounds in [solver-wen.md](docs/reconstruction/solver-wen.md). Still open: the other link
   families, Wen reversals inside one case, a real modal-superposition history (the modal
   history types integrate directly, refuse modal damping and have no mode truncation), and
   frame hinges. Data the native importer does not interpret (hinges, tendons, springs, added
   masses, auto-mesh, several load types) is recorded on import and refused
   ([unmodeled-source.md](docs/reconstruction/unmodeled-source.md)). The N x N area auto-mesh,
   area added mass and object-level area modifiers are imported and checked against Example
   2-008 ([area-import.md](docs/reconstruction/area-import.md)). Automatic frame subdivision
   is analyzed and checked against the published per-mesh values of Examples 1-006, 1-014,
   1-016, 1-017 and 1-019 ([frame-auto-mesh.md](docs/reconstruction/frame-auto-mesh.md)).
   A frame insertion point with stiffness transformation is modeled for parallel offsets in
   linear static cases and checked against Example 1-011
   ([frame-insertion-point.md](docs/reconstruction/frame-insertion-point.md)). Next there:
   mass and geometric stiffness of the offset, unsymmetric shapes, unstable models the vendor
   solver tolerates (the concrete design examples), the mesh restraint options, plane
   incompatible modes, geometric stiffness for area elements, and the penalty-aware
   singularity test in the modal and nonlinear paths.
5. Expand verified design-code interactions, element families, and GUI/API workflows.
6. Exercise clean-build Linux, Windows, and macOS packages on their actual runtimes.

The detailed implementation history and remaining limits are in
[STATE.md](docs/reconstruction/STATE.md). No completion percentage is assigned.
