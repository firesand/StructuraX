# StructuraX capability scope

**Snapshot: 2026-10-15, post-review repairs; solver section amended 2026-11-12 for plastic Wen links, displacement control and unmodeled source data.** This is a scope map, not product parity or a safety certification. Historical wave notes record implementation work; the latest repair notes supersede their broader claims. The machine-readable inventory is [capabilities.json](capabilities.json).

The strongest path remains selected linear structural analyses with analytical checks. The model contains more element, case and design families than every solver/API path can execute. Imported data and enum presence are not evidence of supported physics.

## Solver

Frame, area/shell and eight-node solid formulations are present. Solid mass/weight modifiers now honor zero; invalid values reject. Nonlinear static Gap/Hook and selected nonlinear histories recover reactions from actual constitutive forces. General large-displacement behavior, complete damping/constraint treatment and other nonlinear laws remain incomplete.

Cable analysis is a guarded cable-only nonlinear-static subset. Mixed element models, springs/constraints, target end tension and unsupported partial/nonuniform/point loads reject instead of being silently changed. Capped frames likewise reject axial span loads and unverified releases/offsets/geometric combinations. History-origin continuation rejects until full terminal velocity, acceleration and internal state can be stored; nonlinear Wilson-theta Maxwell integration is also unavailable. Some supplied native examples now explicitly reject unsupported assignments that earlier runs ignored. See [solver-fixes.md](solver-fixes.md).

Plastic Wen link DOFs are supported in small-displacement nonlinear static and direct-integration history cases, and nonlinear static cases can be displacement-controlled on one monitored joint displacement. A Wen deformation that reverses inside one load case, load reduction under displacement control, generalized monitored displacements, and combinations with geometric nonlinearity, cables, member limits or Wilson-theta integration reject. Other link families still reject. See [solver-wen.md](solver-wen.md).

Modal-superposition history types (`LinearHistory`, `NonlinearHistory`) are integrated directly over all degrees of freedom with Rayleigh damping. A nonzero modal damping ratio on them is refused; there is no mode truncation. The native N x N area auto-mesh (when its options match the solver's meshing), area added mass and object-level area modifiers are imported; see [area-import.md](area-import.md), which also records the corrected native shell-type, element-type and incompatible-modes codes and the published checks behind them. Axisymmetric-solid sections, plane sections with incompatible modes, and buckling or P-Delta with area or solid elements are refused. Frame hinges, tendons, line/area/solid springs, frame added mass, frame auto-subdivision, area auto-mesh options the model cannot express, frame joint offsets, link gravity loads, advanced link local axes and several load types are not modeled. Native import records them as unmodeled source data (83 of the 336 corpus files) and analysis refuses the affected cases until an entry is explicitly discarded. What that pass does not cover is listed in [unmodeled-source.md](unmodeled-source.md).

Automatic frame subdivision for analysis (a number of segments, or at the joints lying on a frame) is analyzed, with results reported per frame object and only for the model's joints. It is refused with cables, member tension/compression limits, end length offsets, a segment count together with a joint on the span, and area-mesh joints on the frame; subdivision at crossings and by maximum segment length is not modeled. A buckling case with no initial case buckles under its own scaled load patterns. Span moment loads and span loads on non-prismatic members were corrected at the same time. See [frame-auto-mesh.md](frame-auto-mesh.md). A frame insertion point (cardinal point other than the centroid, or joint offsets) with stiffness transformation is modeled as rigid arms for equal offsets across the member, doubly symmetric shapes and linear static cases, and refused otherwise; see [frame-insertion-point.md](frame-insertion-point.md). Frame M2 is reported in the SAP2000 sign convention (it had the opposite sign).

## Design

AISC closed-section torsion uses distinct Box/Pipe interaction equations and the correct wall dimensions. EC3 local-3 design uses independently defined code shear areas for solid Rectangular, Pipe and equal-wall Box sections; other shape/orientation combinations reject. This is not comprehensive EC3 interaction or buckling support. Concrete beam M2 rejects because scalar top/bottom reinforcement cannot represent its side-face requirements. Existing column PMM and supported shear checks remain partial. See [design-fixes.md](design-fixes.md).

## Interchange

S2K semantic support remains table-selected with source/opaque preservation and conservative edit reconciliation. Native source-byte copy and inventory are distinct from synthesized model export. Native export now validates references, nonlinear completeness, source-preservation boundaries and represented-data roundtrips before writing. Recognized handwritten S2K formatting does not block export; unknown tables/columns and unrepresented assignments still do. Default material colors now roundtrip, while independent Fys values differing from Fy remain unsupported by native synthesis. All native saves use exclusive creation and verified disk readback. Class-version tags come from the local corpus; internal parsing and self-roundtrips do not establish SAP2000 runtime acceptance. See [format-fixes.md](format-fixes.md).

Native synthesis refuses a model that carries unmodeled source data, so 47 corpus files that were previously written without that data are now refused (282 accepted native projections, down from 329). S2K keeps the entries in an extension table.

## Application and API

The GUI exposes selected geometry, load, material, section, case/function and result workflows, including nonlinear history from rest. The typed Rust facade validates solid geometry and temperature accumulation before mutation. It is not a COM server, and the editor does not provide general state continuation. Full tutorial interaction and native Windows/macOS acceptance remain pending. See [application-fixes.md](application-fixes.md).

## Acceptance still required

- Independent numerical tests for each newly supported combination, with input provenance, units and tolerances.
- Licensed vendor-runtime comparisons, including exported-file acceptance and critical assignment comparisons.
- Wider structural/design code coverage and complete result/state contracts.
- Full desktop workflows and target-platform installation/runtime checks.

The latest integrated test counts and exact source fingerprint belong in the dated validation record, not in the capability status labels. Passing the selected reference suite does not establish correctness outside its checked cases.
