# StructuraX roadmap

The priority is reliable, explicitly scoped behavior before additional feature claims.

1. Resolve the concrete solver, design, and native-export findings in
   [the latest review](docs/review/structurax-latest/REVIEW.md), with independent regressions.
2. Make local fixture discovery portable and expose which tests require external inputs.
3. Validate native export against a compatible licensed SAP2000 runtime, including
   unsupported-field diagnostics and critical assignment comparisons.
4. Complete mixed-element nonlinear equilibrium, dynamic reaction recovery and
   state continuation before claiming general nonlinear dynamics.
5. Expand verified design-code interactions, element families, and GUI/API workflows.
6. Exercise clean-build Linux, Windows, and macOS packages on their actual runtimes.

The detailed implementation history and remaining limits are in
[STATE.md](docs/reconstruction/STATE.md). No completion percentage is assigned.
