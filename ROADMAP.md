# StructuraX roadmap

The priority is reliable, explicitly scoped behavior before additional feature claims.

1. Extend the bounded [review repairs](docs/review/structurax-fixes/REVIEW.md): implement
   currently rejected cable/member-load combinations with independent regressions.
2. Complete native representation of guarded assignments and default concrete metadata,
   preserving strict data-loss and referential-integrity checks.
3. Validate native export against a compatible licensed SAP2000 runtime, including
   unsupported-field diagnostics and critical assignment comparisons.
4. Complete mixed-element nonlinear equilibrium and persisted transient state.
   Reactions are repaired for the supported subset; history-origin continuation rejects.
5. Expand verified design-code interactions, element families, and GUI/API workflows.
6. Exercise clean-build Linux, Windows, and macOS packages on their actual runtimes.

The detailed implementation history and remaining limits are in
[STATE.md](docs/reconstruction/STATE.md). No completion percentage is assigned.
