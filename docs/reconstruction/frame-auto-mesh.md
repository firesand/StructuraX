# Automatic frame subdivision for analysis

Date: 2026-11-12. Scope note for one bounded addition. It does not change the
project status: vendor-runtime acceptance and broad parity remain unestablished.

## What is supported

A frame object can be analyzed as several frame elements in a row, the way the
vendor's "automatic frame mesh" assignment does:

- `FrameObj::auto_mesh_segments = n` divides the object into `n` equal-length
  elements (`0` or `1` means one element).
- `FrameObj::auto_mesh_at_joints` divides it at every joint of the model that
  lies on it between its ends. A joint is on the object when it is within one
  millionth of the object's length of its axis and that far inside both ends.

The model is not changed. The solver expands a working copy before assembly and
folds every result back afterwards, so results are reported for the object and
only for joints the model defines: joint displacements, reactions, mode shapes,
history traces, the stored transient state and frame forces of every case and
combination. This follows the vendor's statement in Example 1-006 that results
"are combined and reported for the entire frame object, not the subdivided
elements".

How the object's assignments are carried to its elements:

| Assignment | Elements |
| --- | --- |
| End releases and partial-fixity springs | The object's two ends only |
| Section, angle, modifiers, P-Delta force, gravity multipliers, temperature loads | Every element |
| Distributed span loads | Clipped to each element, with the linear variation evaluated at the cut points |
| Point span loads | The element containing them; a load on a boundary goes to the element it ends, once |
| Temperature loads scaled by a joint pattern | New joints get the linear interpolation of the end-joint pattern values |
| Non-prismatic section | Each element gets its part of the variation (below) |
| Mass | Lumped per element, so it moves to the new joints |

**Non-prismatic sections.** The area, shear areas and torsion constant vary
linearly along a segment and each moment of inertia as a power of a linear
function, so the part of a segment between two cut points is a segment of the
same kind between the properties at those points. Each element gets a derived
section built that way. The CSI Analysis Reference states the intent ("each
element will represent a portion of the full non-prismatic length").

**Output stations.** Stations are uniform within each element, at least three
per element, and enough that the object has at least its requested number. A
station two elements share is reported once at a joint created for the
subdivision, and twice (before and after) at a joint of the model, where a load
or support can make the forces jump. The rule does not depend on the forces, so
a frame has the same stations in every case and combination.

## Bounds that reject instead of approximating

A frame that would be subdivided is refused when it

- is a cable object;
- has a tension or compression limit;
- has end length offsets;
- has both a number of segments and a joint lying on it (how the vendor
  combines the two is not established);
- is subdivided at joints and passes through a joint that automatic area
  meshing will create (the frame would have to connect to it);
- would get a new joint at the location of a joint of the model, of a joint
  area meshing will create, or of a new joint of another frame. The new joint
  would stay unconnected to the other one, and whether the vendor merges them
  is not established. This refuses, for example, a beam with N segments along
  the edge of an N x N meshed slab, and two subdivided frames that cross at a
  common division point;
- asks for more than 10 000 segments;
- would need a joint or frame name the model already uses
  (`__fmesh_<frame>_<k>`, `<frame>__seg_<k>`).

Not modeled, and recorded on native import when present:

- subdivision where the object crosses another line object away from a joint
  (`m_AutoMeshAtLines`); one corpus line sets the flag and crosses nothing;
- a maximum segment length (`m_AutoMeshMaxLength`); zero in every corpus file;
- any subdivision of a cable object.

Insertion points were added after this change
([frame-insertion-point.md](frame-insertion-point.md)). A frame with an offset
insertion point may have a number of segments, not subdivision at joints.

## Evidence

| Check | Reference | Tolerance |
| --- | --- | --- |
| Cantilever periods, 1, 2, 4, 6, 8, 10 and 96 elements, up to five modes each, including which modes do not exist | CSI Software Verification Example 1-014, published SAP2000 value per mesh | 6e-7 s (published to six decimals) |
| Buckling load of a portal frame, 1, 2 and 4 elements per object | Example 1-019: 280.98, 280.24, 280.19 kip | 6e-3 kip |
| Tie rod midpoint deflection and moment, 1, 2 and 16 elements per object, tension by assigned P-Delta force and by a P-Delta case | Example 1-016: -0.54555/11.453, -0.54343/11.495, -0.54330/11.498 | 6e-6 in, 6e-4 kip-in |
| String frequencies, 10 and 100 elements, modes after a P-Delta case | Example 1-017: 74.579, 148.93, 222.06 and 74.587, 149.18, 223.80 Hz | five significant figures |
| Non-prismatic cantilever, 1, 7 and 10 elements, seven load cases | Example 1-006: same published values with and without subdivision | published precision |
| Same member, self-weight tip deflection | Unit-load integral evaluated independently of the solver: -0.0218825865 in | 1e-7 relative |
| One element against 7 and 10 on that member, all cases, joint results and station forces | Each other | 1e-5 (quadrature of the one-element stiffness) |
| Skew member with shear deformation, a partially fixed release, self weight, five distributed loads, five point loads (one on an element boundary, one at each end), two temperature loads (one with a joint pattern), 2, 4 and 7 elements | The unsubdivided member: joint displacements, reactions, base reactions, station forces and deflections | 1e-9 |
| Beam with a support joint on its span | Closed form for a two-span continuous beam: 5wL/8, 3wL/16, wL^2/32 | 1e-9 |
| Three spring-supported joints on a span, defined out of order | The same beam built from four frame objects | 1e-9 |
| Result containers of static, P-Delta, modal, buckling, history, chained history, spectrum and combination results | Only the model's joint and frame names; same stations in every case; model unchanged | exact |

The automatic and the hand-split models are both compared with the published
values, so the first four rows also show that subdividing an object equals
building it from separate objects.

Each refusal above has a negative test that also checks no results are left
and the model is unchanged.

Tests: `crates/sap-solver/tests/frame_auto_mesh_regressions.rs` (portable),
`crates/sap-format/tests/native_export_guards.rs`, unit tests in
`crates/sap-format/src/sdb.rs` for the import rules (flag handling, joints on a
chord, crossings against T-junctions, parallel, collinear and skew lines),
`crates/sap-api/tests/legacy_validation.rs`, and the opt-in vendor test
`vendor_sdb_frame_auto_subdivision_examples_match_published_values`, which
reads the 21 vendor files of Examples 1-006, 1-014, 1-016, 1-017 and 1-019 and
checks every published value above from the file. Nine of the published values
are also rows of `structurax verify`.

Example 1-019 prints E = 29,900 ksi in its problem statement. The vendor model
file stores 29,000, and the published numbers are those of 29,000: the
independent value 280.19 kip is Timoshenko and Gere's `kL tan kL = 6` for that
modulus (29,900 gives 288.9 kip). The tests use 29,000.

## File formats

- **Native import** reads `m_AutoMesh`, `m_AutoMeshAtPoints` and `m_automeshn`
  on the line object. Corpus: 793 lines have no subdivision, 186 frame lines
  subdivide at joints only, 36 have a segment count (2 to 120), and 15 cable
  lines carry the at-joints flag. Before this change the settings were
  recorded and the affected cases refused.
- **Native export** writes the same three fields and the second enabling flag
  (`m_automesh2`, equal to the first in every corpus line).
- **S2K** uses reconstruction extension columns on the `CONNECTIVITY - FRAME`
  row, written only where assigned: `MeshSegments` and `MeshAtJoints`, like
  `MeshN1`/`MeshN2` on areas. The vendor's own frame auto-mesh table is not
  parsed; no vendor S2K sample is available to verify its layout.
- **API**: `cFrameObj::set_auto_mesh` / `get_auto_mesh`, after
  `FrameObj.SetAutoMesh`. Subdivision at intersections and a maximum segment
  length are refused there instead of stored and ignored.

A model built through the API, a template or a hand-written S2K file has no
subdivision unless it asks for one, so existing models solve as before. This
differs from the vendor's user interface, whose new frames default to
subdivision at joints.

## Corpus

Thirty-eight files carried a frame-subdivision record. None does now.

| Files | Outcome |
| --- | --- |
| 16 with a segment count: Examples 1-006b, 1-014b to g, 1-016c, d, f, g, 1-017a, b, 1-019b, c, EN 3-2005 example 3 | Analyze; all but the last are checked against published values |
| 1 with a joint on the span: KBC 2009 example 1 | Analyzes |
| 12 with a joint on the span: the other concrete and steel design examples | Not analyzed. Eleven combine the subdivision with an insertion point (top center with stiffness transformation), which is refused. All twelve are also singular at the centroid, with or without the subdivision: the beam is free to spin about its axis and to move sideways, which the vendor solver tolerates and this one does not |
| 6: Examples 1-009a to c, 1-013b, c, 1-015 | Still refused for tendons, line springs or frame added mass |
| 3: Examples 1-029a to c | Refused: displacement control with large displacements |

The 13 files with a joint on the span have no published value that depends on
the split; the evidence for that mode is the closed-form and hand-split checks.

A one-off sweep analyzed 304 of the 334 importable files twice, as imported
and with both settings cleared, before the insertion-point refusal existed. No
file changed between analyzable and not; the only difference was which pivot
the twelve singular design examples stopped at. The other 30 files were too large for the dense solver to sweep (more than
700 joints or 500 mesh elements); 27 of them have no frame objects and the
other 3 have no subdivided frame. The sweep is not part of the test suite.

What the three fixes below touch in the corpus: buckling cases on frame models
are in Examples 1-019a to c and IS 800:2007 example 1 (nine more files have
buckling cases on shell or solid models, which are refused); a span moment
load is in Example 1-003 only; a non-prismatic member with span load or self
weight is in Examples 1-006a and b only.

Native projection of the 336 files is now 334 S2K roundtrips, **282** native
roundtrips, 5 incomplete-link-law refusals, **47 unmodeled-source refusals**
and 2 unsupported imports (250 / 5 / 79 before). Fifty-one files carry an
unmodeled-source entry.

A model imported earlier and saved as S2K keeps its `RUST UNMODELED SOURCE
DATA` rows for frame subdivision and does not gain the new columns; re-import
the native file to analyze it.

## Three defects found and fixed on the way

None of them is specific to subdivision; the equivalence and published-value
checks exposed them.

1. **Span moment loads had the wrong sign in the moment diagram.** A
   concentrated or distributed moment about local 2 or 3 was added to the
   section moment beyond it instead of subtracted. A correction term that
   forces the diagram to close at the far end hid the error there and smeared
   it linearly along the member, so end forces, reactions and joint
   displacements were right and interior moments and the deflected shape were
   wrong. Now checked against closed forms (cantilever under a concentrated and
   under a uniform moment).
2. **Span loads on a non-prismatic member used prismatic fixed-end forces.**
   The load resultant was right and the end moments were not: the self-weight
   tip deflection of the Example 1-006 cantilever as one element was -0.023799
   in against -0.021883 in by independent integration (8.8 % off). The
   published check for that case is the fixed-end moment, which is statics and
   was not affected. Fixed-end forces of a varying section now come from the
   member released at one end: statics, the unit-load integrals of the varying
   section, and the member stiffness.
3. **A buckling case took its loads from another case.** The reference axial
   forces came from a previously analyzed case whose name equals the first
   load pattern's name, ignoring the scale factor and any further pattern; with
   no such case the result was an empty list of factors, which is what the
   vendor Example 1-019 files produced. A buckling case with no initial case
   now solves its own load patterns at their scale factors as the reference
   state. With an initial case the previous behaviour is unchanged. There was
   no test of a buckling factor before; there is now a closed-form one (Euler
   column, two scaled patterns) besides Example 1-019.

Tests for these: `crates/sap-solver/tests/frame_span_load_regressions.rs` and
the Example 1-006 test.

## Not done

- The GUI has no control for the two settings; they come from native files,
  S2K, or the API.
- The twelve design examples that are singular at the centroid need a decision
  about unstable models (the vendor reports and solves them). Eleven of them
  also combine subdivision at joints with an insertion point, which is refused
  ([frame-insertion-point.md](frame-insertion-point.md)).
- Stations are uniform per element, not the object's own uniform stations plus
  the element ends as the vendor reports them.
- The one-element non-prismatic stiffness uses five Gauss points per vendor
  segment; it differs from the subdivided member by up to 2e-6.
