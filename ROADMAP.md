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
5. Expand verified design-code interactions, element families, and GUI/API workflows.
6. Exercise clean-build Linux, Windows, and macOS packages on their actual runtimes.

The detailed implementation history and remaining limits are in
[STATE.md](docs/reconstruction/STATE.md). No completion percentage is assigned.
