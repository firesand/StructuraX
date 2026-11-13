# StructuraX roadmap

The priority is reliable, explicitly scoped behavior before additional feature claims.

1. Extend the bounded [review repairs](docs/review/structurax-fixes/REVIEW.md): target I-end
   cable tension (`target_tension_i`) and frame tension/compression limits (`tc_limits`) under
   distributed span axial loads and self-weight are implemented and verified; implement remaining
   rejected mixed cable/member-load combinations (e.g., partial/nonuniform cable loads and mixed
   cable-frame/shell/solid assemblies) with independent regressions.
2. Extend native `.sdb` schema coverage for remaining unmodeled tables while preserving strict
   data-loss and referential-integrity checks (`TYSpring` 21-term coupled/local springs,
   orthotropic `S_Material`, `S_CableData` `m_CableType` `3/5/8`, `S_POINTOBJECTS` 3x3
   `local_axes_matrix`, independent `S_MaterialPropertiesRebar` `fys != fy`, `S_ProjectInfo` /
   `S_ProjectInfoByUser`, and `COORDSYSTEMARRAY` `S_GUIDE` grids now roundtrip).
3. Validate native export against a compatible licensed SAP2000 runtime, including
   unsupported-field diagnostics and critical assignment comparisons.
4. Complete mixed-element nonlinear equilibrium and remaining dynamic case families.
   Reactions and `TransientTerminalState` continuation (`LinearHistory` modal generalized
   coordinates + `LinearDynamic`, `NonlinearHistory`, `NonlinearDynamic` -> static/modal/direct-history)
   are verified; `SteadyState`, `Psd`, `MovingLoad`, `Hyperstatic`, and nonlinear Wilson-theta
   Maxwell integration remain explicitly guarded.
   Plastic Wen links and displacement-controlled nonlinear static cases are implemented within
   the bounds in [solver-wen.md](docs/reconstruction/solver-wen.md). Still open: the other link
   families, Wen reversals inside one case, and frame hinges. Data the native importer does not
   interpret (hinges, tendons, springs, added masses, several load types) is recorded on import
   and refused ([unmodeled-source.md](docs/reconstruction/unmodeled-source.md)). The N x N area
   auto-mesh, area added mass and object-level area modifiers are imported and checked against
   Example 2-008 ([area-import.md](docs/reconstruction/area-import.md)). Automatic frame
   subdivision is analyzed and checked against the published per-mesh values of Examples 1-006,
   1-014, 1-016, 1-017 and 1-019 ([frame-auto-mesh.md](docs/reconstruction/frame-auto-mesh.md)).
   A frame insertion point with stiffness transformation is modeled for parallel offsets in
   linear static cases and checked against Example 1-011
   ([frame-insertion-point.md](docs/reconstruction/frame-insertion-point.md)). Next there:
   mass and geometric stiffness of the offset, unsymmetric shapes, unstable models the vendor
   solver tolerates (the concrete design examples), the mesh restraint options, plane
   incompatible modes, geometric stiffness for area elements, and the penalty-aware
   singularity test in the modal and nonlinear paths.
5. Expand verified design-code interactions, element families, and GUI/API workflows
   (concrete beam minor-axis `M2` with `as_side`, EN 1993-1-1 10-shape `V3` shear and multi-axis
   interaction, `cCableObj`, and 19-table `cDatabaseTables` are now verified).
6. Exercise clean-build Linux, Windows, and macOS packages on their actual runtimes.

The detailed implementation history and remaining limits are in
[STATE.md](docs/reconstruction/STATE.md). No completion percentage is assigned.
