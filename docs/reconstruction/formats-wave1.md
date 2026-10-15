# Native format reconstruction, wave 1

Status on 2026-10-15: native v19 document preservation and a larger model import projection are implemented. A general native exporter from an arbitrary `SapModel` is still incomplete. Native SAP2000 runtime acceptance has not been tested; no vendor executable was run.

## Completed

- `sap_format::inspect_native_sdb_file`, `inspect_native_sdb_bytes`, and `inspect_native_sdb` traverse the complete supplied native stream, including design preferences/results, bridge data, window states, options, API extended entity data, model log, and final timestamp. `NativeSdbInventory` exposes a serializable top-level byte-span inventory and class counts; `NativeRecord` exposes nested field spans separately.
- `NativeSdbDocument::to_bytes()` returns the **exact original container bytes** for documents opened from a file or bytes until an explicit patch is applied. It preserves original compression metadata, thumbnail, header, and every model-unsupported record. `to_sdb()` retains the entire native decompressed payload; regenerating gzip preserves that payload byte for byte.
- `NativeSdbDocument::set_f64(record_index, field_name, value)` encodes a finite scalar only into a verified `r8` field span. A regression checks that all bytes outside the selected eight-byte field remain unchanged and that the resulting document traverses to EOF and imports the changed coordinate. This is a low-level encoding operation; it does **not** recompute caches, invalidate results, or validate engineering relationships. It is not an accepted native model-editing workflow.
- Native model import now includes solid connectivity/material properties, link connectivity/basic active/fixed DOFs and stiffness/damping/mass properties, sampled response-spectrum/time-history functions, area surface pressure and temperature, uniform unpatterned solid surface pressure and temperature, and supported spectrum/history assignments.
- Corrected internal native analysis-case codes. They differ from `eLoadCaseType` in OAPI: native 2 is nonlinear static, 3 modal, 4 response spectrum, 5 modal history, 6 nonlinear modal history, 7 direct history, 8 nonlinear direct history, 10 buckling, and 11 steady state. Native case 1 is linear static; `m_ntype != 0` does not make it nonlinear static. Independent record inspection found **436 incorrectly classified cases among 767 live cases in 202 of 336 files** under the previous mapping. Specific fixture assertions cover the principal changed families. Mapping branches for other native codes remain less directly corroborated by this corpus.
- Unsupported time-scaled, delayed, phase-shifted or rotated history assignments and non-global/rotated spectrum assignments produce `FormatError::UnsupportedNativeCase` naming the case and reason. They are not silently removed from an otherwise executable case. Original native inventory/copy remains available for these documents.
- Added the reconstruction's `WilsonTheta` S2K read/write field and non-default text/private-snapshot roundtrip coverage.
- Prior format fixes remain: absolute-distance frame loads, dependency ordering of text tables, strict unit/critical numeric/record checks, quoted identifiers, native coordinate names/reference slots, PRO centroid correction and header/record-count checks.

## Evidence and verification

The current native traversal is grounded in the existing full-stream decoder and struct `WriteSelf` schema. The old specification's section 2.3 is not a trustworthy exact top-level sequence for these supplied files: the actual `FileVersionInFile` is a **string**, and the actual sequence starts with ProjectStartUp/ProjectInformation/Standards/AnalysisModelDateTimeStamp/RunParameters. The implemented sequence follows the decoder and independently passes EOF checks for every supplied native fixture.

- Source schema: `crates/sap-format/src/sdb_schemas.txt`, SHA-256 `06592df236c1bc54291ac6e51f56d817a9d78986dec1a0807ed0ecf7074ae925`.
- Original full-stream decoder: `/home/edo/.gemini/antigravity/brain/93a71218-fc85-41f1-84cc-d6032276e1c4/scratch/parse_sdb.py`, SHA-256 `9d4a6ac003da2cfda1112c20953d134de6db138c3a82c6eaf1413db762acec13`.
- Static managed-artifact evidence: `docs/reconstruction/native-object-members-evidence.json`, REA evidence ID `ev_a0e448d71aaf14821822bd8e6f3d32f4435a1b171fbdc12f25f5e04f88cd00e6`, selected exact types `CSIMDIForm.SAP2000Form.S_SOLIDOBJECTS` and `CSIMDIForm.SAP2000Form.S_NLLINKOBJECTS`; 2 types, 144 methods, no unmatched selection. This corroborates managed member identities/CIL anchors; it does not by itself establish native runtime behavior.
- Selected vendor assembly: `extracted/files/sapsharedstructures.dll`, SHA-256 `83d2b0d34d9bf9d07754a4e4545318cdee22e2659031a3ddc254e99bb85b1c7b`.

`cargo test -p sap-format -p sap-section --offline -- --nocapture` passed **22 tests** (18 format, 4 section). Corpus results:

- **336/336** native SDB documents fully inventoried to EOF and copied with exact original compressed-container bytes; regenerated containers preserve their complete native payloads.
- **334** safe model imports and supported-projection S2K roundtrips; **2** explicit unsupported dynamic-case rejections. These results supersede the earlier 336-model-import claim, which did not guard scaled history assignments. Other compatibility warnings still apply to the imported projection.
- All **28** standard PRO fixtures parse, including legacy libraries with trailing complete records beyond the declared count. `joists.pro` uses a different layout and remains unsupported.
- Fixture examples verify 16 solids/16 face loads in `example_5_005a.sdb`, 100 uniform-temperature solid loads in `example_5_011.sdb`, link endpoints/property in `example_6_001.sdb`, 128 area pressure loads in `example_2_010a_thick.sdb`, 5 area temperature loads in `example_2_013a_thick.sdb`, and 33 sampled spectrum points plus case assignments/settings in `example_1_022.sdb`.

Strict format/section Clippy is part of the final checks. No vendor artifact was modified.

## Remaining reconstruction work

1. A from-scratch **native** `SapModel` writer needs verified initialization/defaults, full field serialization, native counts/reference indices, units, geometry/cache recalculation, result invalidation and cross-structure consistency. The current `pack_model_to_sdb` remains a private `SAP2000_S2K_STREAM` snapshot payload, expressly rejected by the native inventory API. Preservation/copy of a supplied native document does not complete this exporter.
2. Model projection still omits grid definitions/project metadata and many advanced records; the native document API preserves these bytes. Truly unknown class IDs cannot be given invented record boundaries: structured inventory fails explicitly, while the already-read `SdbFile` still retains the untouched opaque stream. Later-version/unknown schemas remain unverified.
3. Nonlinear/coupled link matrices, hysteresis/multilinear parameters, property multipliers, staged construction, moving/bridge/offshore loads, advanced local axes, patterned/gradient solid loads, and full dynamic case controls are not fully mapped into `SapModel`.
4. Generated and external-file dynamic function definitions are not regenerated/resolved. Some imported functions therefore have no sampled points; this is not a reconstructed excitation signal. Scaled/delayed/rotated excitation needs richer model fields before those cases can be imported safely.
5. S2K import reports populated unsupported tables; unsupported columns in otherwise supported tables remain omitted with an explicit compatibility note. Saving the model projection cannot preserve the full original text export.
6. Native SAP2000 reading an unchanged preserved file, accepting an edited document, or accepting a newly initialized reconstructed model has not been observed. Numerical parity and engineering acceptance are separate from byte-level preservation and import tests.
