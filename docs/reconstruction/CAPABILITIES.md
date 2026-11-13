# StructuraX capability scope

**Snapshot: 2026-11-13, post-parity & hardening milestones.** This is a scope map, not product parity or a safety certification. Historical wave notes record implementation work; the latest repair and milestone notes supersede earlier unsupported-feature bounds where explicitly tested. The machine-readable inventory is [capabilities.json](capabilities.json).

The strongest path remains selected linear and bounded nonlinear structural analyses with analytical checks. The model contains more element, case and design families than every solver/API path can execute. Imported data and enum presence are not evidence of supported physics.

## Solver

Frame, area/shell and eight-node solid formulations are present. Solid mass/weight modifiers honor zero; invalid values reject. Nonlinear static Gap/Hook and selected nonlinear histories recover reactions from actual constitutive forces. General large-displacement behavior, complete damping/constraint treatment and other nonlinear laws remain incomplete.

Cable analysis is a guarded cable-only nonlinear-static subset supporting undeformed length, horizontal tension, and target I-end tension (`target_tension_i`, verified on Example 7-003). Mixed cable-frame/shell/solid element models, cable springs/constraints, and unsupported partial/nonuniform cable loads reject instead of being silently changed. Capped frames (`tc_limits`) enforce tension/compression cutoffs under distributed span axial loads and self-weight while preserving exact static reaction equilibrium (`< 1e-10`), and reject unverified releases/offsets/geometric combinations. `TransientTerminalState` stores physical (`u`, `v`, `a`, `previous_force`), nonlinear link, and mass-orthonormal modal generalized (`modal_q`, `modal_v`, `modal_a`) states for chained direct-integration and modal `LinearHistory` restarts; nonlinear Wilson-theta Maxwell integration remains unavailable. See [solver-fixes.md](solver-fixes.md).

## Design

AISC closed-section torsion uses distinct Box/Pipe interaction equations and unequal-wall HSS dimensions. EC3 local-3 design implements EN 1993-1-1 §6.2.6(3) plastic shear areas across 10 standard section profiles (`Rectangular`, `I`, `Channel`, `DbChannel`, `T`, `Angle`, `DblAngle`, `Box`, `Pipe`, `Circle`) and multi-axis ($P, V_2, V_3, M_2, M_3, T$) interaction checks; comprehensive lateral-torsional/local buckling and unmodeled shapes still reject. Concrete beam minor-axis flexure (`M2`) under ACI 318-14 and Eurocode 2 reports side-face longitudinal reinforcement via `ConcreteDesignResult.as_side`. Existing column PMM and supported shear checks remain partial, and concrete torsion remains unsupported. See [design-fixes.md](design-fixes.md).

## Interchange

S2K semantic support remains table-selected with source/opaque preservation and conservative edit reconciliation. Native source-byte copy and inventory are distinct from synthesized model export. Native export validates references, nonlinear completeness, source-preservation boundaries and represented-data roundtrips before writing. Recognized handwritten S2K formatting does not block export; unknown tables/columns and unrepresented assignments still do. Material colors, 21-term `TYSpring` coupled/local springs, orthotropic `S_Material`, `S_CableData` (`3/5/8`), 3x3 joint `local_axes_matrix` (`S_POINTOBJECTS`), independent shear-rebar yield `fys != fy` (`S_MaterialPropertiesRebar`), `project_info` (`S_ProjectInfo`), and coordinate system grids (`S_GUIDE`) all roundtrip losslessly. All native saves use exclusive creation and verified disk readback. Class-version tags come from the local corpus; internal parsing and self-roundtrips do not establish SAP2000 runtime acceptance. See [format-fixes.md](format-fixes.md).

## Application and API

The GUI exposes selected geometry, load, material, section, link/cable, case/function, database-table and result workflows, including `ConcreteDesignResult.as_side` reporting. The typed Rust OAPI surface validates solid geometry and temperature accumulation before mutation and includes a 21-method `cCableObj` facade (with strict namespace isolation from `cFrameObj`) and a 19-table transactional `cDatabaseTables` facade (`GetTableForDisplayArray`, `SetTableForEditingArray`, `ApplyEditedTables`). It is not a Windows COM server. Full tutorial interaction and native Windows/macOS acceptance remain pending. See [application-fixes.md](application-fixes.md).

## Acceptance still required

- Independent numerical tests for each newly supported combination, with input provenance, units and tolerances.
- Licensed vendor-runtime comparisons, including exported-file acceptance and critical assignment comparisons.
- Wider structural/design code coverage and complete result/state contracts.
- Full desktop workflows and target-platform installation/runtime checks.

The latest integrated test counts and exact source fingerprint belong in the dated validation record, not in the capability status labels. Passing the selected reference suite does not establish correctness outside its checked cases.
