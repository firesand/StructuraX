# Frame insertion points

Date: 2026-11-12. Scope note for one bounded addition. It does not change the
project status: vendor-runtime acceptance and broad parity remain unestablished.

## What is supported

A frame insertion point is a cardinal point (1 to 11; 10 is the centroid) plus
joint offsets at the two ends. With stiffness transformation on, the section
centroid is set off from the line between the joints and the source program
connects the two rigidly (CSI Analysis Reference, "Insertion Points": "a fully
rigid constraint that connects the neutral axis to the joints").

The frame element now does the same for a **parallel offset in linear static
cases**:

- The centroid is at the joint, plus the joint offset, minus the position of
  the cardinal point relative to the centroid. Bottom/top are the negative/
  positive local 2 side and left/right the negative/positive local 3 side, at
  half the outside depth and width.
- Stiffness and fixed-end forces are carried from the element ends to the
  joints through exact rigid arms (`k = T^T k T`, `f = T^T f`), not penalty
  constraints. Station forces and deflections are those of the element at its
  centroid.
- Loads on the frame object act on the element, so at the centroid; loads on a
  joint act at the joint. The offsets carry no load and no mass.
- Joint offsets may be given in the element local system or the global one.

With the transformation off the frame is analyzed between its joints, at its
centroid. That rests on the meaning of the API argument only: no corpus line
combines a non-centroid cardinal point with the flag off.

## Bounds that reject instead of approximating

A frame with stiffness transformation on and a centroid off the joint line is
refused when

- any run-flagged case is not a linear static case with no initial case. Mass
  is lumped on the diagonal at the joints, geometric stiffness lacks the rigid
  arm's own term, and the nonlinear and history paths build the element
  separately;
- the joint offsets differ between the ends, or have a component along the
  member. Both change the element's axes or length;
- the cardinal point is not 10 and the section is not a doubly symmetric shape
  with known outside dimensions (rectangle, box, pipe, circle, I with equal
  flanges), or is non-prismatic. Cardinal points 5 and 11 are the centroid for
  those shapes only;
- the frame also has end length offsets, an assigned P-Delta force, or
  automatic subdivision at joints (which line the vendor tests for joints is
  not established), or is a cable.

A number of automatic segments is allowed; each element keeps the offset.

## Evidence

| Check | Reference | Tolerance |
| --- | --- | --- |
| Example 1-011, case 2 (10 kip on the end of the frame object): fixed-end My, Mz; beam moments | CSI Software Verification Example 1-011: 210, 60, 0, 0 kip-in | 1e-8 |
| Example 1-011, case 1 (10 kip on the end joint): fixed-end moments; beam M3, M2 at every station | Published 0, 0, -210, -60 kip-in | 1e-8 |
| End-joint displacement of a cantilever under an eccentric axial load, cardinal points 1, 2, 4, 5, 6, 8, 9, 10, 11 with and without joint offsets | Closed form: P L / (E A) + P e^2 L / (E I) along the axis, P e L^2 / (2 E I) across it | 1e-10 |
| The same insertion point from a global and from a local joint offset; a member rolled 90 degrees; a vertical member | The same closed form in the member's axes | 1e-10 |
| Axial span load and a point load on an offset frame | Statics at the support | 1e-8 |
| An offset frame in 2 and 4 automatic segments | The unsubdivided frame: joint results, reactions, station forces and deflections | 1e-9 |
| Offset frame between two fixed joints with a moment release at one end, uniform load | Propped cantilever: 3wL/8, 5wL/8, wL^2/8, no moment at the joint above the hinge | 1e-9 |
| Offset frame between two fixed joints, uniform temperature rise | Axial force -E A alpha dT, no member bending, joint moments of that force times the offset | 1e-8 relative |
| Example 1-011 converted from kip-in to kN-m | The published moments in kN-m | 1e-8 |
| `FOffset` record written from the imported Example 1-011, and `FFixity` record from Example 1-008 | The vendor record, byte for byte | equal |
| Transformation off, any cardinal point and offsets | The centroidal frame, exactly | equal |

Case 2 fixes the geometry by statics alone (which side "left" is). Case 1 then
checks the rigid arms: the support carries no moment while the beam carries
210 kip-in.

Tests: `crates/sap-solver/tests/frame_insertion_point_regressions.rs`
(thirteen refusals, each leaving no results),
`crates/sap-format/tests/native_export_guards.rs`, unit tests in
`crates/sap-format/src/sdb.rs` (offset record forms, mirror flags),
`crates/sap-api/tests/legacy_validation.rs`, and the opt-in vendor tests
`vendor_sdb_example_1_011_insertion_point_matches_published_values`, which
reads the vendor file, and
`vendor_sdb_per_end_assignment_records_are_written_byte_for_byte`. The four
published values are rows of `structurax verify`.

The evidence is one published example plus closed forms. Nothing published
covers unsymmetric shapes, mirrored insertion points, or any case type other
than linear static.

## File formats

- **Native import** reads joint offsets from the `FOffset` section. The one
  corpus record (Example 1-011) stores a 3 x 2 array `[0, 0, 12, 12, 0, 0]`
  with coordinate system code -1 for +12 in along local 2 at both ends. That
  fixes the layout as [component][end] and -1 as the local system. Unequal
  ends (the end order is not shown by the fixture) and other codes are
  recorded as unmodeled source data instead of read.
- Native `m_StiffTransform = true` is read as "transform": Example 1-011
  carries it and its published analysis model has the rigid links.
- A mirrored insertion point (`m_Mirror2`, `m_mirror3`) is recorded as
  unmodeled; no corpus line sets either flag.
- **Native export** writes local, equal-ended offsets to `FOffset` and refuses
  global or unequal ones. Two-dimensional arrays holding per-end values
  (`FOffset` 3 x 2, and partial fixity in `FFixity` 6 x 2) are now written
  with the shapes the corpus has; they were written as a single column.
- **S2K** already carried the whole assignment in
  `FRAME INSERTION POINT ASSIGNMENTS`.
- **API**: `cFrameObj::set_insertion_point`.

## Corpus

Fifteen files have a non-centroid cardinal point, all with the transformation
on. Example 1-011 now analyzes and matches its published values. The fourteen
concrete design examples stay refused: thirteen combine the insertion point
with automatic subdivision at joints and one (AS 3600) has a T section. The
ACI 318-08 example, taken further, also runs a modal case and is singular once
reduced to its static case, with the transformation on or off.

Native projection: 334 S2K roundtrips, **283** native roundtrips, 5
incomplete-link-law refusals, **46 unmodeled-source refusals**, 2 unsupported
imports. Fifty files carry an unmodeled-source entry.

## Found and fixed on the way: the sign of M2

Frame `M2` was reported with the opposite sign to SAP2000's. The manual
("Internal Force Output") defines a positive bending moment as one that
compresses the positive 2 face (M3) or the positive 3 face (M2). The solver
reported M3 that way and M2 as the moment vector about local 2, which is the
opposite sense. Evidence: Example 1-011 publishes M2 = -60 where the solver
gave +60 with M3, the axial force and all reactions already matching; a
cantilever with a tip load along +local 3 has its positive 3 face in
compression at the support, where the solver reported a negative M2; and an
existing thermal test negated `m2` to meet its closed form. The convention now
has its own test (`frame_span_load_regressions.rs`): a cantilever pushed along
+local 2 and along +local 3 has root moments +F L about both axes, with
dM3/dx = -V2 and dM2/dx = -V3.

`FrameStationResult::m2` now follows the SAP2000 convention in every case type.
Magnitudes, M3, shears, reactions and displacements are unchanged. Design
checks use the magnitude of M2. Results saved before this change keep the old
sign.

## Not done

- Unequal end offsets and offsets along the member (rotated element axes).
- Unsymmetric shapes, which need the centroid position in the section.
- Mass and geometric stiffness of the offset, so modal, history, P-Delta and
  buckling cases.
- Automatic subdivision at joints together with an insertion point.
- The GUI has no control for the assignment.
