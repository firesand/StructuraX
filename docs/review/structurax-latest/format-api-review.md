StructuraX wave4 native format/API review — 2026-10-15
Read-only review. No repository source modified. Scratch executables created under /tmp only.

Repro project: /tmp/structurax-format-api-repro/
Run: CARGO_TARGET_DIR=/home/edo/REA/target cargo run --quiet --manifest-path /tmp/structurax-format-api-repro/Cargo.toml --bin structurax-format-api-repro
Class-version evidence: same command with --bin class_versions

P1 — Native export silently drops represented physical assignments.
Files: crates/sap-format/src/sdb/native.rs:937-940 (orthotropic arrays replaced with repeated scalar values), :1194-1207 (point explicit local_axes_matrix omitted), :1444-1449 (only uncoupled six-spring vector emitted), :863 (project_info discarded via empty struct).
Repro: build an otherwise valid two-point frame; set B.spring_coupled=Some([2.;21]), B.local_axes_matrix=Some(90-degree-Z matrix), material.e_ortho=Some([29000,22000,13000]), project_info[Owner]="Do not lose". pack_model_to_native_sdb and extract_sdb_to_model both return Ok, but output is:
  LOSS project_info={} coupled=None localaxes=None ortho=None
Impact: a successful native save can change stiffness/restraint bases and remove coupled support stiffness. This is data already represented by SapModel, beyond the documentation's stated omission of fields outside its projection. Current corpus equality tests cannot detect native importer omissions shared by the exporter.
Action: serialize these fields correctly, or reject nondefault unsupported represented state before export. Do not return a clean success for this subset.

P1 — Native export bypasses the opaque S2K preservation safety boundary.
Files: crates/sap-format/src/sdb/native.rs:849-863 (starts a fresh stream with no preservation check); crates/sap-api/src/lib.rs:342-348 (clears export_warnings, calls unreported export and returns OAPI_OK).
Repro: parse_s2k a JOINT COORDINATES row with VendorTag="keep" and an OPAQUE VENDOR table containing Value=21. The input model has s2k_preservation=Some. Native pack/reimport succeeds with s2k_preservation=None; both opaque table and column are gone. Output: OPAQUE_LOSS source_metadata=true exported_metadata=false.
Impact: this route evades the checked wave3 semantics for unknown columns/tables, ambiguous edits and unit conversions. API users receive no data-loss warning because save_native_sdb explicitly clears warnings.
Action: reject preservation-bearing models until the caller explicitly selects a supported-only native projection; alternatively provide a checked loss report with an explicit opt-in. Keep private snapshot and source-copy routes separate.

P1 — Native save turns incomplete nonlinear data into supposedly complete law data.
Files: crates/sap-format/src/sdb/native.rs:1144-1151; crates/sap-format/src/sdb.rs:1445-1458.
Repro: Gap property G with nonlinear_data_complete=false, dof_nonlinear[0]=true, k_nl[0]=100, param1[0]=1. Export writes a synthetic 30-value m_param because one parameter is nonzero. Reimport uses array length >=12 to set nonlinear_data_complete=true. Output: COMPLETENESS before=false after=true.
Impact: the roundtrip removes the source-completeness guard that protects unsupported imported constitutive data, potentially enabling an analysis which was correctly blocked before save.
Action: refuse native export of incomplete activated nonlinear laws (or preserve a trustworthy completeness distinction); do not promote partially known data merely because a synthetic fixed-length parameter vector was emitted.

P1 — Missing native object/property references are silently substituted with index 1.
Files: crates/sap-format/src/sdb/native.rs:1215-1216, :1227; analogous fallback references at :975, :1064, :1090, :1285-1287, :1308-1310, :1323-1329 and load/function mappings.
Repro: valid A/B frame F; change F.point_j and F.section to "MISSING" through the shared model. Native pack and reimport succeed, yielding point_j=A and section=FSEC1. Output: INVALID_REF export OK point_j=A section=FSEC1.
Impact: corrupted or incompletely imported topology is rewritten into a different structural model rather than rejected. In the reproducer the frame collapses onto its I joint.
Action: perform referential-integrity validation before serialization and use errors for absent references. No arbitrary first-object/material/property substitution.

P2 — New additive solid-temperature API accepts arithmetic overflow and invalidates results.
File: crates/sap-api/src/lib.rs:1037-1044.
Repro: create a valid eight-corner unit cube and defined DEAD pattern; set_load_temperature(CUBE, DEAD, f64::MAX, true), restore an OLD result entry, then call the same magnitude with replace=false. The second call returns OAPI_OK, stores infinity and clears OLD: TEMP_OVERFLOW status=0 stored=inf results=0.
Impact: individually finite input values produce a nonfinite model state, reported as a successful edit.
Action: compute the prospective sum, require finiteness, then commit/invalidate. Reject without changing assignments or results on overflow.

P2 — Solid object creation reports success for zero-volume repeated-corner topology.
File: crates/sap-api/src/lib.rs:965-989.
Repro: define SSEC1, add point A, then solid_obj().add_by_point([A;8], SSEC1, ZERO). Output: DEGENERATE_SOLID status=0 id=ZERO.
Impact: a malformed solid is inserted and results invalidated even though the geometry cannot represent a Hex8 element. No distinct-corner, finite-geometry or Jacobian validation occurs.
Action: validate distinct corners and a finite, nonfolded positive-Jacobian element before committing, or explicitly document this as an unvalidated raw-record API rather than claiming validated creation.

P2 — Claimed nonlinear history/continuation API work is still rejected by the actual facade.
Files: crates/sap-api/src/definitions.rs:87-103; docs/reconstruction/application-wave4.md:17-18.
Repro: load_cases().set_definition(case_type=NonlinearHistory) returns OAPI_ERR; NonlinearStatic with initial_case=Some(existing DEAD) also returns OAPI_ERR. Output: NONLINEAR_HISTORY_API status=1; CONTINUATION_API status=1. definitions.rs only permits LinearStatic, NonlinearStatic, Modal, Buckling, ResponseSpectrum, LinearHistory, LinearDynamic, and rejects non-spectrum initial_case.
Action: implement validated state dependencies/history types if supported by the solver, or correct the claimed implementation status.

P2 compatibility evidence — Every generated record is tagged ClassVersion=1 despite differing supplied v19 record versions.
Files: crates/sap-format/src/sdb/native.rs:652-657; source fixture extracted/files/example_1_001.sdb.
Repro from class_versions binary: vendor RUNARRAY=2, S_MaterialPropertiesSteel=2, S_MaterialPropertiesConcrete=5, LOADPATTERNARRAY=3, SOLUTIONCONTROLARRAY=3, ANALYSISCASEARRAY=3; generated corresponding records all =1.
This is a verified wire-contract mismatch, not a claim that vendor GUI rejection was reproduced. The static inspector ignores version-specific reader behavior, so schema EOF/self-roundtrip success does not validate these tags or old-version field layouts. The documented lack of vendor runtime validation remains material.
Action: establish and emit version-correct class tags/layouts from source evidence, then test a licensed vendor runtime before calling synthesized files vendor-compatible. Keep the current output explicitly experimental meanwhile.

Additional documentation accuracy:
- application-wave4.md:8-16 lists set_prop notes/guid, cPropSolid get_prop/get_name_list, cSolidObj add_by_coord, get_tc_limits/get_cable_data, set_load_surface and set_load_temp. These methods/parameters do not exist under those public signatures. Actual methods are set_prop(name, mat_prop, a,b,c,incompatible), add_by_point, set_load_surface_pressure and set_load_temperature; only count is supplied for solid properties. solid_stress has one name argument, not item_type_elm (:19).
- formats-wave4.md:18 says project metadata is emitted, but native.rs:863 writes an empty ProjectInformation struct; project_info demonstrably disappears.

Packaging/runtime observations:
- The native writer itself is pure Rust over compile-time embedded schemas; the scratch native-export runs required no installed SAP2000, Wine, .NET or executable helper. This does not establish vendor file acceptance.
- Format corpus tests in native.rs still use repo-relative fixture_directory() and return early if absent, rather than unconditionally asserting external files (live-verified). Root is separately gating vendor solver fixtures for portable CI. A portable green run must distinguish executed vendor comparisons from absent optional fixtures.
- Resolved during root packaging work: the earlier hardcoded /home/edo/REA/extracted/files PRO path was replaced with STRUCTURAX_SECTION_DIR or relative sections (live-verified at lib.rs:121-123). Required section databases remain external optional data; the original path defect is no longer an open finding.

Review limit: no vendor application runtime, exhaustive format fuzzing, or full solver physics review was performed here. Findings above are based on current source and the listed independent runtime repros; no repository code was repaired during this read-only assignment.
