# StructuraX capability scope

**Snapshot: 2026-10-15, post-review repairs.** This is a scope map, not product parity or a safety certification. Historical wave notes record implementation work; the latest repair notes supersede their broader claims. The machine-readable inventory is [capabilities.json](capabilities.json).

The strongest path remains selected linear structural analyses with analytical checks. The model contains more element, case and design families than every solver/API path can execute. Imported data and enum presence are not evidence of supported physics.

## Solver

Frame, area/shell and eight-node solid formulations are present. Solid mass/weight modifiers now honor zero; invalid values reject. Nonlinear static Gap/Hook and selected nonlinear histories recover reactions from actual constitutive forces. General large-displacement behavior, complete damping/constraint treatment and other nonlinear laws remain incomplete.

Cable analysis is a guarded cable-only nonlinear-static subset. Mixed element models, springs/constraints, target end tension and unsupported partial/nonuniform/point loads reject instead of being silently changed. Capped frames likewise reject axial span loads and unverified releases/offsets/geometric combinations. History-origin continuation rejects until full terminal velocity, acceleration and internal state can be stored; nonlinear Wilson-theta Maxwell integration is also unavailable. Some supplied native examples now explicitly reject unsupported assignments that earlier runs ignored. See [solver-fixes.md](solver-fixes.md).

## Design

AISC closed-section torsion uses distinct Box/Pipe interaction equations and the correct wall dimensions. EC3 local-3 design uses independently defined code shear areas for solid Rectangular, Pipe and equal-wall Box sections; other shape/orientation combinations reject. This is not comprehensive EC3 interaction or buckling support. Concrete beam M2 rejects because scalar top/bottom reinforcement cannot represent its side-face requirements. Existing column PMM and supported shear checks remain partial. See [design-fixes.md](design-fixes.md).

## Interchange

S2K semantic support remains table-selected with source/opaque preservation and conservative edit reconciliation. Native source-byte copy and inventory are distinct from synthesized model export. Native export now validates references, nonlinear completeness, source-preservation boundaries and represented-data roundtrips before writing. This catches unimplemented assignments and even unsupported default concrete metadata. Class-version tags come from the local corpus; internal parsing and self-roundtrips do not establish SAP2000 runtime acceptance. See [format-fixes.md](format-fixes.md).

## Application and API

The GUI exposes selected geometry, load, material, section, case/function and result workflows, including nonlinear history from rest. The typed Rust facade validates solid geometry and temperature accumulation before mutation. It is not a COM server, and the editor does not provide general state continuation. Full tutorial interaction and native Windows/macOS acceptance remain pending. See [application-fixes.md](application-fixes.md).

## Acceptance still required

- Independent numerical tests for each newly supported combination, with input provenance, units and tolerances.
- Licensed vendor-runtime comparisons, including exported-file acceptance and critical assignment comparisons.
- Wider structural/design code coverage and complete result/state contracts.
- Full desktop workflows and target-platform installation/runtime checks.

The latest integrated test counts and exact source fingerprint belong in the dated validation record, not in the capability status labels. Passing the selected reference suite does not establish correctness outside its checked cases.
