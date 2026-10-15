# StructuraX post-review repairs

**Historical first repair checkpoint.** The [PR follow-up](../structurax-followup/REVIEW.md) supersedes the S2K guard, cable initialization precision and native-file I/O contracts below.

Checkpoint: 2026-10-15. Baseline commit: `39089be716985de035906458f9f3133ab044ab2b`.
This follow-up addresses the confirmed findings in the [baseline review](../structurax-latest/REVIEW.md). It distinguishes repaired supported behavior from features disabled until they can be implemented correctly. StructuraX remains experimental; full reconstruction and engineering/vendor acceptance are not established.

## Corrected behavior

- Nonlinear history recovers reactions from the integrated constitutive forces, damping and inertial state, including grounded Maxwell links and shear offsets. An API-created ramp-loaded Hook oscillator matches an independent closed-form displacement history and support/link balance.
- Supported static-to-history initial acceleration uses the new initial load and carried displacement. Solid zero mass/weight modifiers stay zero. Cable and history solvers honor iteration/tolerance controls and report failed convergence.
- AISC Box/Pipe H3 interactions and unequal-wall HSS torsion geometry are corrected. Supported EC3 local-3 shear areas come from code formulas, independently of analysis `As3`.
- Solid API creation validates geometry before mutation. Temperature accumulation rejects overflow while preserving inputs and prior results.
- Native export validates names/references, finite values, incomplete laws and unsupported assignments, then checks represented model equality after projection before writing. The 48 emitted class-version tags use observed corpus evidence instead of a blanket version 1.
- Typed nonlinear history from rest and its GUI controls are available. API and GUI native-export warnings persist. CLI native export validates in memory and exclusively creates a new destination, preserving existing files.

## Findings contained by explicit rejection

These paths are not fully implemented by this repair:

- Mixed cable/ordinary-element models, cable joint springs/constraints, target end tension and unsupported member loads/offsets/releases.
- Capped-member axial span loads and unsupported geometry/release combinations.
- History-origin continuation: final velocity, acceleration and Maxwell internal state are not yet persisted. Nonlinear Wilson-theta Maxwell integration also rejects.
- Unsupported EC3 local-3 shape/orientation combinations, unequal-wall EC3 boxes and concrete beam M2 flexure.
- Native source-preservation data, unrepresented assignments and incomplete imported nonlinear laws. Even default concrete metadata can prevent synthesis; no silent stripping is performed.

The original findings therefore have bounded repairs or fail-closed guards, not blanket feature-completion status. The per-component notes contain exact restrictions: [solver](../../reconstruction/solver-fixes.md), [format](../../reconstruction/format-fixes.md), [design](../../reconstruction/design-fixes.md), [application/API](../../reconstruction/application-fixes.md).

## Verification

**214 fixture-enabled tests, 207 portable source-copy tests, strict workspace Clippy, native build and 59/59 selected reference checks passed.** [validation.json](validation.json) records commands, counts, source and log fingerprints. There are 27 additional test functions: 13 solver regressions, 9 native-export guard tests and 5 API regressions; existing design/application tests were strengthened. The selected reference suite now has 59 rows: the prior history-origin continuation row was removed because its state contract is unsupported. A rejection is not counted as a numerical reference match.

The optional local native corpus has 336 inventory/exact-copy checks, 334 strict S2K projection roundtrips, 329 strict native projection roundtrips, five explicit incomplete-law export refusals and two existing unsupported imports. Imported cable 7-001/7-002 fixtures now verify target-tension rejection; separate hand-built cable/network references still check numerical equations. Native 5-002/5-005 solver checks are smoke tests, not reference-deflection acceptance.

No vendor artifacts are added to Git. Tests without those assets remain independently runnable, although legacy optional tests return early when fixtures are absent. The seven explicitly gated solver fixture tests are excluded from the portable count.

## Remaining acceptance

Native SAP2000 file opening and differential execution, full design-code coverage, general nonlinear continuation/mixed-element physics, complete desktop tutorial workflows and Windows/macOS runtime tests remain outstanding. Internal parser success, self-roundtrips and selected analytical checks cannot establish those outcomes. API/GUI saves validate before overwriting but do not promise crash-safe atomic I/O.
