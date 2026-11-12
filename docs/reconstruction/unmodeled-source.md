# Imported data that is not modeled: recorded and refused

Date: 2026-11-12. Scope note. Native import now records the analysis-relevant
data it does not interpret, for the sections listed below, and one solver rule
refuses modal damping on modal history cases. It still does not prove that
native import fails closed for every possible file; the last section says what
is not covered.

## Mechanism

`SapModel::unmodeled_source` is a list of entries, each a description and a
scope. An entry means: the importer saw data that changes analysis results and
that the model cannot represent.

| Scope | Cases refused |
| --- | --- |
| `AllCases` | every case |
| `NonlinearCases` | nonlinear static and nonlinear history cases, and cases that continue from one |
| `DynamicCases` | modal, response-spectrum and history cases, and cases that continue from one |
| `AllButLinearStatic` | every case except a linear static case with no initial case |
| `LoadPattern(p)` | cases that apply `p` with a nonzero scale (static or history), and cases that continue from one |
| `Case(c)` | case `c` and cases that continue from it |

- **Analysis** checks the list before anything else and names the entries in
  the error. Only run-flagged cases are checked, so disabling an affected case
  lets the others run.
- **Native synthesis** refuses a model with entries: the written file would
  silently lack the data.
- **S2K** keeps the entries in an extension table, `RUST UNMODELED SOURCE DATA`
  (`Item`, `Scope`, `Target`, `Description`), so a save and reopen does not lose
  the refusal. A row with an unknown scope, a missing target or an empty
  description is a parse error.
- **Import report** carries one warning per entry.
- Nothing removes an entry implicitly while the imported model is edited.
  Removing it is the explicit decision to analyze without that data:
  `sap_api::discard_unmodeled_source`, the "Discard and analyze without it"
  button in the GUI case editor, or deleting the S2K row. Deleting the object an
  entry names does not remove the entry. Entries match load patterns and cases
  by name; there is no rename operation for either in the API or GUI.
  Generating a new model from a template clears the list, like other
  file-derived state.

## What native import records

Fifty of the 336 corpus files carry at least one entry (83 before frame
auto-subdivision was modeled, 51 before joint offsets were read; see
[frame-auto-mesh.md](frame-auto-mesh.md) and
[frame-insertion-point.md](frame-insertion-point.md)).

| Source data | Detection | Scope | Corpus files |
| --- | --- | --- | --- |
| Frame auto-subdivision where the frame crosses another line object away from a joint | `m_AutoMeshAtLines` plus geometry | `AllCases` | 0 (one line sets the flag and crosses nothing) |
| Frame auto-subdivision to a maximum segment length | `m_AutoMeshMaxLength != 0` | `AllButLinearStatic` | 0 |
| Auto-subdivision of a cable object into segments, or at a joint lying on its chord | `m_automeshn > 1`, or `m_AutoMeshAtPoints` plus geometry, on a cable line | `AllButLinearStatic`, `AllCases` | 0 |
| Area auto-mesh with options the model cannot express | `ShMesh` record outside the import rule in `area-import.md` | `AllCases` | 12 |
| Tendon objects | line object type 3 | `AllCases` | 9 |
| Tendon prestress loads (`PTLoad`) | live records, by pattern | `LoadPattern` | 9 (the same files) |
| Per-mode or interpolated modal damping | `m_numberMDs > 0` or `m_numberFDs > 0` on case types 5 and 6 | `Case` | 5 (6 cases) |
| Area springs (`ShSpring`) | live records | `AllCases` | 4 |
| Solid surface springs (`SSpring`) | live records | `AllCases` | 4 |
| Frame line springs (`FSpring`) | live records | `AllCases` | 3 |
| Frame hinge assignments | count of `S_Hinge` records | `NonlinearCases` | 2 |
| Link gravity loads (`NLGLoad`) | live records with a nonzero multiplier | `LoadPattern` | 2 |
| Frame added mass (`FMass`) | live records | `DynamicCases` | 2 |
| Area gravity loads (`ShGLoad`) | live records, by pattern | `LoadPattern` | 2 |
| Area pore-pressure loads (`ShPPLoad`) | live records, by pattern | `LoadPattern` | 2 |
| Advanced link local axes | `S_NLLINKOBJECTS.m_UserAxes` | `AllCases` | 1 |
| Frame joint offsets (`FOffset`) not in the element local system, or unequal at the two ends | `m_csys != -1`, or the two columns of `m_P` differ | `AllCases` | 0 (the one corpus record is read) |
| Mirrored frame insertion point | `m_Mirror2` or `m_mirror3` on the line object | `AllCases` | 0 |
| Solid gravity loads (`SoGLoad`) | live records, by pattern | `LoadPattern` | 1 |
| Solid pore-pressure loads (`SoPPLoad`) | live records, by pattern | `LoadPattern` | 1 |
| Axisymmetric-solid rotate loads (`AsoRtLoad`) | live records, by pattern | `LoadPattern` | 1 |

Three of these were worse than a dropped assignment:

- **Area auto-mesh.** Nineteen files assign N x N meshes from 10 x 10 to
  64 x 64 (or 1 x 4 to 1 x 64) in `ShMesh`. The importer read mesh counts from
  fields of the area object that are zero in every corpus file, so each plate
  was solved as a single element. Seven of the nineteen are now imported; the
  other twelve use options the model cannot express and stay refused
  (`area-import.md`).
- **Tendon objects.** The importer read a tendon line object as a frame member
  and resolved its property index against the frame sections; in Example 1-009
  the tendon became a `W18X35` beam. Tendon objects (and line objects of any
  unrecognized type) are now left out of the model and recorded, so discarding
  the entry means analyzing without the tendon, not with an invented beam.
- **Frame auto-subdivision** into a number of segments, or at the joints
  lying on a frame, was recorded here (38 files) and is now modeled by the
  solver; see [frame-auto-mesh.md](frame-auto-mesh.md). Only the variants in
  the table above are still recorded.

Area added mass and per-object area modifiers are no longer recorded: they are
imported (`area-import.md`).

How the section records are read: every record of these sections has an
`m_alive` flag (deleted records are skipped), and load records carry a
load-pattern index in `m_lc` or `m_LC`. The sections are non-empty in the corpus
only in the files counted above, so a record means an assignment, not a default.
Section-level entries give a count, not object names. A load whose pattern index
cannot be resolved is recorded for all cases. If the stream cannot be
inventoried at all, one all-cases entry is recorded.

The same pass also covers sections that are empty in all 336 corpus files, by
section name in the vendor schema and with no fixture: `PanelZone`, `MatTemp`,
`FFireProofing` (all cases); `Seismic`, `Wind`, `FSLoad`, `FDefLoad`, `FTFLoad`,
`ShUFLoad`, `ShSLoad`, `ShWPLoad`, `SoSLoad`, `NLDefLoad`, `NLTFLoad` (by
pattern when the record has `m_lc`/`m_LC`, otherwise all cases). A mass source
other than the corpus default (one source: element masses plus additional
masses, nothing from loads) is recorded for dynamic cases.

Hinge entries give a count, not frame names: the hinge section lies beyond the
sections the model importer reads, and the frame it belongs to is not resolved.

Per the CSI Analysis Reference Manual, frame nonlinear properties "only affect
nonlinear analyses"; a linear case from zero behaves as if they were absent.
That is why hinges use the nonlinear scope and a linear case of Example 1-026
still runs. Added mass uses the dynamic scope: Example 1-015's modal case is
refused while a static case of the same model runs.

## Modal damping on modal history cases

`LinearHistory` and `NonlinearHistory` are the vendor's modal-superposition
history types (native case types 5 and 6). They are integrated here directly
over all degrees of freedom with Rayleigh and link damping. Two consequences:

- **A nonzero `modal_damping` on these two types is refused** by the solver and
  by the typed case validator. It was previously read and ignored. With zero
  modal damping the result is identical, step for step, to the corresponding
  direct-integration type.
- **Zero modal damping does not make them modal.** There is still no mode
  truncation: a vendor modal history that uses a subset of modes or Ritz vectors
  is not reproduced, only its untruncated limit is approximated.

How the native damping type is read (18 type-5/6 cases in the corpus):

| `m_DampType` | Cases | Meaning used | Evidence |
| --- | --- | --- | --- |
| 4 | 15 | constant ratio in `m_Damp` | default value on every case |
| 2 | 2 | mass/stiffness proportional, given by two frequencies | `example_6_011a`: ratio 0.0059 at 1 and 10 Hz, and `m_dd` holds exactly the Rayleigh coefficients those give (checked to 1e-12) |
| 1 | 1 | mass/stiffness proportional, given directly | `example_6_011b`: nonzero `m_dd` coefficients |

For types 1 and 2 the coefficients are imported as Rayleigh damping, which the
direct integration applies, and the stale `m_Damp` scalar is not kept. These
meanings are inferred from the corpus values, not from vendor documentation.
Native export writes type 1 for a modal history case that has Rayleigh
coefficients and type 4 otherwise, so the coefficients are not written next to
a constant-ratio type; a two-frequency specification (type 2) is written back
as the equivalent direct coefficients. A modal history case with both a
constant ratio and coefficients has no single type and is refused on export.
Four cases have zero constant damping and run; eleven have a nonzero constant
ratio and are refused; three are proportional.

The general `LoadCase` default of 0.05 is a response-spectrum setting. The GUI
now creates modal history cases with 0 and shows the ratio for those types; S2K
rows of those types without a `ConstDamp` column parse as 0.

Cases that were standing in for vendor direct-integration histories
(Example 6-003 and 6-005 `NLDHIST1` in the built-in suite and the vendor
regression) were typed `NonlinearHistory`; they are now `NonlinearDynamic`, and
their reference checks still pass.

## Frame insertion points

A frame insertion point with stiffness transformation used to be solved as a
centroidal frame without notice. It is now modeled for a parallel offset in
linear static cases and refused otherwise, on the model's own fields rather
than as a recorded entry; see [frame-insertion-point.md](frame-insertion-point.md).
Joint offsets in the native `FOffset` section are imported in the one form the
corpus shows; other forms, and mirrored insertion points, are recorded.

## Two related refusals

- **Load names in static cases.** A static, nonlinear static or buckling case
  applies load patterns by name, and a name that was not a load pattern
  contributed nothing. The native importer turns an acceleration load on such a
  case into the name `U1`..`U6`, so it would have been dropped the same way.
  The solver now refuses both: an acceleration-load name as not implemented,
  any other unknown name as a missing load pattern. No corpus file has a static
  acceleration load, so this has a portable test but no vendor fixture.
- **Native export of auto-meshed areas.** The writer used to store a model's
  N x N mesh in area-object fields that are zero in every corpus file. It now
  writes the `ShMesh` record the corpus uses (`area-import.md`); only unequal
  mesh counts are refused.

## Effect on the corpus

Native projection of the 336 files is now: 334 S2K roundtrips, **283** native
roundtrips, 5 incomplete-link-law refusals, **46 unmodeled-source refusals**,
2 unsupported imports. It was 329 / 5 / 0 before any of these records existed,
and 250 / 5 / 79 before frame auto-subdivision was modeled. The 46 were
previously written without the recorded data. Four more files carry entries
but are counted under their incomplete link law.

No test that existed before these records analyzes any of the affected files or
checks their values (the whole-corpus roundtrip tests load them), so no earlier
passing check depended on the dropped data.

## Evidence

- `crates/sap-solver/tests/unmodeled_source_regressions.rs`: every scope against
  linear, nonlinear, chained and history cases on a one-joint model with hand
  results; discard; persistence through S2K; the modal damping rule and its
  equality with the direct-integration type.
- `crates/sap-format/tests/native_export_guards.rs`: S2K roundtrip of the
  scopes with quoted text, malformed rows, native synthesis refusal, S2K default.
- `crates/sap-api/tests/analysis_definitions.rs`: validator and discard,
  including lock and bad index.
- `crates/sap-format/src/sdb/skipped.rs` unit tests: synthetic record streams
  for live and deleted records, pattern grouping with both field spellings,
  unresolved patterns, the mass-source rule, hinge counting and nested records.
- Opt-in vendor tests in `wave4_vendor_regressions.rs`: Example 6-008 entries
  and refusal text; Example 1-026 nonlinear refusal with a linear case still
  running; Example 6-007 per-mode damping; Example 6-011a proportional damping;
  Example 2-006 (only the case using the gravity pattern is refused, the other
  two analyze), 1-015 (modal refused, static runs), 1-009a (tendon), 2-016a
  (auto-mesh options), 1-013a (line springs).
- The corpus roundtrip test asserts the split (283 / 5 / 46) and one refused
  file per kind of recorded data.

The GUI list and discard button have no automated test.

## What is not covered

- **Sections left out on purpose.** `NLLinkFDProp` (frequency-dependent link
  properties act only in steady-state and PSD cases, which are refused as case
  types); `GenDispl`/`GD` (generalized displacements are output quantities; a
  displacement-controlled case that monitors one is refused separately);
  `SDSection`, `FLateralBrace`, `SteelData`, `ConcreteData` (section-designer
  and design data, not analysis input); named property-set definitions.
- **Sections not examined.** `Schedule`, `MultiStepMove`, `WindOverwriteLine`,
  the sea-state and wave sections, and every bridge, lane and vehicle section.
  All are empty in the corpus (apart from a default `WaveCharacteristic` record
  in every file), so their record layout and defaults are unknown.
- **Sections with no fixture.** The second group in the table above is recorded
  by section name only; no corpus file exercises it.
- **Advanced-axes flags on frames, areas, joints and solids.** No corpus file
  sets them. Only the link flag is recorded.
- **Fields inside records that are read.** This pass works per section. The
  ignored fields found so far (area mesh counts, frame auto-subdivision, the
  shell-type and incompatible-modes codes, the element type of area sections)
  were found by checking examples against published results, not by this pass;
  others may remain.
- **Vendor S2K.** The vendor's own S2K tables for all of this remain opaque
  tables with the existing unknown-table diagnostics.
