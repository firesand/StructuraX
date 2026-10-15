# Checked experimental native export fixes

The native synthesis route remains experimental. It produces a Rust-inspectable
v19-style stream without a SAP2000/Wine/.NET runtime, but no vendor application
has opened the generated files in this work. Internal roundtrips and matching
class tags do not establish vendor compatibility or arbitrary SDB preservation.

`pack_model_to_native_sdb` and `save_native_sdb_file` now share a checked boundary:
validation runs before packing, and the packed stream is projected back and
compared with the input model before success. Errors use
`FormatError::UnsupportedNativeExport { field, reason }` and identify the first
unsupported or inconsistent field. Saving performs these checks before touching
the destination; this is a preflight guarantee, not a crash-safe atomic-I/O claim.
Neither function mutates the input model.

## Guarded losses

- Opaque S2K preservation metadata blocks native synthesis. Keep the original
  text, use checked S2K writeback or a private snapshot. Explicitly clearing this
  metadata requests a supported-only projection and relinquishes opaque text.
- Nonempty project metadata, coupled joint springs, explicit joint-axis matrices,
  local spring coordinates, orthotropic material arrays and coordinate-system
  grids are rejected because their native serialization has not been verified.
- Any link property with `nonlinear_data_complete=false` is rejected, including
  properties with no currently activated nonlinear DOF. Synthetic parameter
  arrays cannot upgrade incomplete imported constitutive information.
- Missing object, section, material, pattern, function, case, constraint and
  coordinate-system references are rejected before index maps are used. Record
  keys must equal their nonempty names; case/combo names must be unambiguous.
- Nonfinite modeled numbers are rejected. Optional cable quantities are checked
  directly before JSON inspection, so `Some(NaN)` and `Some(Infinity)` cannot be
  mistaken for `None`; specified cable targets must be positive. Segment and
  native-index counts must fit their native integer range.
- A final exact comparison rejects any other represented field that the current
  writer/importer cannot preserve, rather than dropping it. This includes
  unsupported presentation state such as a custom material color, and some
  default concrete-material metadata. The exporter does not silently normalize
  these values to make a comparison pass.

The comparison intentionally excludes analysis results, design results and the
locked state: synthesis produces an unanalysed model. It does not preserve
unmodeled native source records. For that purpose, the native document's lossless
source-copy path remains separate from synthesis.

## Supported boundary

An export is accepted only when its represented persistent model survives the
checked projection exactly, aside from the explicitly excluded result/lock state.
Acceptance is evidence about StructuraX's projection, not vendor interpretation.
The portable success regression creates a default model, explicitly removes the
unused `4000Psi` material and dependent area section, then creates a two-joint
steel frame. It independently checks connectivity, restraints, joint forces and
frame temperature loading after export. Additional existing regressions exercise
cable, link and case records within the represented subset.

No blanket guarantee is made for all assignments of an object type. Unsupported
values in otherwise supported objects are rejected. Use the private snapshot for
full represented-model persistence when the native subset cannot carry a field.

## Class-version evidence

The earlier unconditional `ClassVersion=1` is removed. Each emitted class uses
its single unambiguous version observed in the 336 supplied v19 fixtures. All 48
emitted class IDs had observed tags, with no conflicting versions. The inventory
is recorded in `native-class-versions-observed.json`; generated tests check raw
tag bytes independently, and the optional corpus check verifies all 48 tag
constants against source records. An unobserved emitted class is rejected.
This establishes tags, not vendor reader behavior or version-dependent layout
compatibility; licensed vendor runtime validation remains outstanding.

## Validation

Portable regressions cover each of the four P1 review findings, missing
references, exact-comparison fallback, nonfinite numbers and integer overflow.
Both early failures and late projection failures preserve an existing destination
byte for byte. The optional corpus test keeps strict S2K equality and strict
native equality for accepted models; incomplete source laws are counted as
explicit refusals, never reclassified as successful roundtrips.

Executed on 2026-10-15 with the supplied optional fixture directory present:

- `cargo test -p sap-format`: 40 tests passed (20 library, 9 native guard,
  11 S2K preservation); no failures.
- `cargo clippy -p sap-format --all-targets --no-deps -- -D warnings`: passed.
- Native inventory/source-copy and class-tag checks inspected all 336 fixtures.
- Two source cases remain explicitly unsupported at import. The 334 successfully
  projected models all passed strict S2K roundtrip comparison.
- Of those models, 329 passed checked native synthesis and exact model comparison.
  Five were correctly rejected because their imported nonlinear law data are
  incomplete: `example_6_002.sdb`, `example_6_009.sdb`, `example_6_010.sdb`,
  `example_6_011a.sdb`, `example_6_011b.sdb`. The test independently checks each
  rejection's field against an actually incomplete property; other export errors
  fail the test. These five refusals are not successful native exports.

The fixture tests return early when the optional corpus is absent; the nine native
export guard regressions and eleven preservation regressions remain portable.
No additional vendor runtime dependency was introduced.
