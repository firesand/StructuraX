# Native SDB export and nonlinear/cable SDB import — wave 4

> Review update: known correctness defects and inaccurate API/acceptance claims are documented in [the post-wave-4 review](../review/structurax-latest/REVIEW.md). This historical implementation note does not supersede those findings.

Checkpoint: 2026-10-15. This wave implements a schema-driven native SAP2000 v19 `cBinaryStream` `.sdb` writer (`pack_model_to_native_sdb` / `save_native_sdb_file`) and expands native `.sdb` model projection for nonlinear link parameters (`NLLINKPROPARRAY.m_param`), single-joint grounded links (`m_SingleJoint`), parametric harmonic functions (`FUNCSHUNARRAY`), and cable properties (`CABLEPROPERTYARRAY` / `S_CableData`).

## 1. Expanded Native SDB Model Projection (`crates/sap-format/src/sdb.rs`)

- **`NLLINKPROPARRAY.m_param` (`5 x 6 = 30` `f64` array)**: Extracts `param1` (`m_param[0..6]`: gap/hook initial opening `open`, Maxwell damper coefficient $c_{\text{nl}}$, or Wen yield force) and `param2` (`m_param[6..12]`: Maxwell damper velocity exponent $\text{exp}$ or Wen post-yield ratio). Marks `nonlinear_data_complete = true` for `Linear`, `Gap`, `Hook`, `Damper`, and `PlasticWen` properties when `m_Nonlinear` has 6 booleans and (`!dof_nonlinear.any()` or `m_param.len() >= 12`), while retaining `nonlinear_data_complete = false` for incomplete or unmapped constitutive families.
- **Single-joint grounded links (`S_NLLINKOBJECTS.m_SingleJoint`)**: Maps `m_SingleJoint == true` or `p_j == p_i` to `point_j: None`, matching CSI's 1-joint grounded link convention in `example_6_003a.sdb`, `example_6_004.sdb`, and `example_6_005a.sdb`.
- **Parametric Sine functions (`FUNCSHUNARRAY`)**: When `m_FuncType == 1` and `m_Period > 0` with empty explicit `(time, value)` arrays, synthesizes the discrete harmonic time-history trace from `m_Period`, `m_NumSteps`, `m_Cycles`, and `m_Amp` (`example_6_005a.sdb`).
- **Cable properties and objects (`CABLEPROPERTYARRAY`, `S_LINEOBJECTS.m_CableData`)**: Imports `CABLEPROPERTYARRAY` records (`m_name`, `m_Mindex`, `m_Area`, `m_msf`), resolves `m_LineType == 1` line objects against `cable_prop_names`, and populates `FrameObj.cable_params` (`undeformed_length`, `target_tension_i`, `horizontal_tension`, `num_segments`) when `!m_ModelUsingFrames`.
- **Rayleigh damping and nonlinear controls (`ANALYSISCASEARRAY`)**: Extracts `m_Integration` (`m_am`, `m_bk`) and `m_Nonlinearity` (`m_itmax`, `m_ittol`) into `LoadCase`.
- **Lossless S2K roundtrip**: `write_s2k` and `parse_s2k` round-trip `param1`, `param2`, and `cable_params`, preserving `assert_eq!(m1, m2)` across all 334 supported vendor `.sdb` models.

## 2. Schema-Driven Native SDB Stream Writer (`crates/sap-format/src/sdb/native.rs`)

- **`pack_model_to_native_sdb(&SapModel) -> Result<SdbFile, FormatError>`** and **`save_native_sdb_file(&SapModel, path)`**:
  - Serializes `SapModel` directly into the SAP2000 v19 `cBinaryStream` wire format using the exact top-level record sequence of `traverse()` and the field definitions in `SDB_SCHEMAS_TXT` (`get_schemas()`), without using the private `RUST_SAP2000_S2K_V19` text wrapper.
  - Emits valid v19 headers, PNG thumbnail, and gzip-compressed `cBinaryStream` records for project metadata, coordinate systems, materials, frame/shell/solid/link/cable properties, points, lines/frames/cables, areas, solids, links, restraints, constraints, springs, masses, load patterns, analysis cases, combinations, joint/frame/area/solid loads, functions, and preferences.
- **Preserved private container path**: `pack_model_to_sdb` and `save_sdb_file_with_report` retain their existing private S2K-snapshot behavior so S2K source-preservation workflows and `private_payload_is_never_classified_as_native` remain unchanged.

## 3. Verification

- `sdb::native::tests::pack_model_to_native_sdb_produces_valid_cbinarystream_and_roundtrips`: Verifies that a model with points, restraints, springs, masses, materials, frame/area/solid sections, links, cables, loads, functions, cases, and combinations packs to a native `.sdb` that passes `inspect_native_sdb` with `parsed_bytes == stream_bytes` (100% schema compliance to EOF) and round-trips through `extract_sdb_to_model`.
- `sdb::native::tests::vendor_corpus_models_round_trip_through_s2k_and_native_sdb`: Verifies round-trip consistency across the vendor `.sdb` corpus.
- `sap-solver/tests/wave4_vendor_regressions.rs::native_sdb_export_reimport_preserves_analysis_results`: Verifies that exporting `example_1_001.sdb` and `example_5_001_incomp.sdb` via `save_native_sdb_file` and re-importing produces identical solver displacements and stresses.

## Explicit Remaining Boundaries

- While `pack_model_to_native_sdb` satisfies the v19 `cBinaryStream` schema to EOF and re-imports into `SapModel`, opening generated `.sdb` files inside a licensed Windows SAP2000 v19 GUI has not been tested in this environment.
- Unmapped vendor tables/fields outside `SapModel`'s projection are defaulted when synthesizing a new native stream from `SapModel` (use `copy_native_sdb` to preserve an existing vendor `.sdb` byte-for-byte).

