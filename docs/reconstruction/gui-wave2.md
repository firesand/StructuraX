# Native GUI workflows — wave 2

Checkpoint: 2026-10-15. This extends the existing Rust/egui desktop application. It is not full SAP2000 GUI or OAPI parity.

The Define, Edit and Assign menus now open a Model Workflows editor with Geometry, Loads, Areas, Releases, Combinations and Materials tabs. Geometry uses the shared transactional model operations for move, linear replication and equal frame division, and assigns existing frame sections. Materials supports isotropic steel, concrete and no-design properties, derives G from E and Poisson ratio, validates strengths/densities, and preserves unedited fields. Initial values and type switches use current-unit defaults, including when the model has no materials; changing type explicitly reloads defaults. The editor reloads the matching model material when units change. Orthotropic properties and other material families are explicitly refused by this editor.

Loads supports named patterns and matching linear static cases, all six global joint force/moment components, uniform global-Z frame loading, and uniform top/bottom area pressure. Positive pressure pushes into the selected face. Pattern creation refuses names belonging to incompatible analysis cases. Replacement is scoped to the selected load pattern and assignment type; pressure replacement also matches face.

Areas supports shell type, material, two thicknesses, all ten section modifiers and structured quad subdivision. Releases supports six full-release flags at each frame end, with an explicit statement that applying clears partial-fixity springs. Combinations supports the five represented combination types and nested case/combo factors, rejecting missing references, nonfinite factors and dependency cycles.

New operations validate before mutation, require explicit unlock, preserve inputs/results on rejection, and invalidate analysis and design results after success. Section and material edits affect all objects referencing the property. Move explicitly discloses that shared joints also move attached unselected elements. Replication discloses assignment-preserving coincident-joint compatibility checks.

## Verification

`cargo test -p sap-app` covers:

- Material properties, loads, cases, combinations, sections, mesh assignments, and releases through S2K save/reopen.
- Empty metric-model material initialization, type changes and conversion back to kip/in/F against independent physical defaults.
- Missing inputs, nonfinite values, incompatible case names, locked models, material validity and combination cycles without partial mutation or result loss.
- Pattern-scoped load replacement and successful edit invalidation.
- Rendering all six tabs through a headless egui context and actual pointer press/release events on the save-pattern button, verifying WIND and its analysis case are created.

These are application render/input tests, not a desktop operating-system interaction pass. The previous native GUI smoke test remains separate in wave-1 validation. No full tutorial, Windows, hardware renderer or human interaction acceptance is claimed.

## Boundaries

Drawing still selects the first existing frame/area property; the new assignment controls allow it to be changed afterward. Automatic code combinations, auto-select section optimization, design preferences, advanced mesh controls and complete material/section/load dialogs remain incomplete. Existing legacy GUI actions outside the new workflow editor have their previous behavior and are not covered by the new transactional claims. API signatures remain a native Rust facade; version-matched managed metadata evidence is in `rea-api-evidence-wave2.json`.
