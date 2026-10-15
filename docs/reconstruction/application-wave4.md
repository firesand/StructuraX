# OAPI solid/cable/native-SDB facades and CLI export — wave 4

> Review update: known correctness defects and inaccurate API/acceptance claims are documented in [the post-wave-4 review](../review/structurax-latest/REVIEW.md). This historical implementation note does not supersede those findings.

Checkpoint: 2026-10-15. This wave exposes the newly implemented 3D solid elements, catenary cable / member T-C limit assignments, and schema-driven native `.sdb` export through `sap-api` and `sap-app`.

## 1. OAPI Facades (`crates/sap-api/src/lib.rs`)

- **`cPropSolid` (`SapModelApi::prop_solid()`)**:
  - `set_prop(name, mat_prop, a, b, c, incompatible, notes, guid)`: Validates material existence and finiteness of Euler angles `(a, b, c)`, creating or updating a `SolidSection` with `incompatible_modes: incompatible`.
  - `get_prop`, `get_name_list`, `count`: Queries solid property definitions.
- **`cSolidObj` (`SapModelApi::solid_obj()`)**:
  - `add_by_coord` and `add_by_point`: Validates the 8 corner joints, creates the `SolidObj`, and invalidates stale results.
  - `set_load_surface(name, load_pat, face, val, replace)`: Validates `face` in `1..=6`, finite pressure `val`, and load pattern existence.
  - `set_load_temp(name, load_pat, val, replace)`: Assigns uniform temperature change $\Delta T$ on a solid element.
- **`cFrameObj` Cable & T/C Limit Extensions**:
  - `set_tc_limits(name, limit_compression_exists, limit_compression, limit_tension_exists, limit_tension)` and `get_tc_limits`: Validates and assigns member tension/compression limits (`tc_limits`).
  - `set_cable_data(name, undeformed_length, tension_i, horiz_tension, num_segments)` and `get_cable_data`: Validates and assigns `CableParams` (`undeformed_length`, `target_tension_i`, `horizontal_tension`, `num_segments`).
- **`cLoadCases` Nonlinear History / Continuation Support**:
  - `set_definition` now accepts `LoadCaseType::NonlinearHistory` and `LoadCaseType::NonlinearDynamic` as well as `initial_case` state continuation across nonlinear static and direct-integration time-history cases.
- **`cAnalysisResults::solid_stress(name, item_type_elm)`**:
  - Recovers 8-point nodal stresses `(s11, s22, s33, s12, s13, s23, s_max, s_min, s_vm)` across selected load cases from `CaseResults::solid_results`.
- **`cFile::save_native_sdb(path)`**:
  - Exports the active `SapModel` using `sap_format::sdb::native::save_native_sdb_file`, recording any error in `last_file_error`.

## 2. CLI `export-native-sdb` Subcommand (`crates/sap-app/src/main.rs`)

- Added `sap2000-rs export-native-sdb <INPUT> --output <OUTPUT>`:
  - Loads any `.s2k` or `.sdb` model, writes a schema-compliant SAP2000 v19 `cBinaryStream` `.sdb` file via `save_native_sdb_file`, and refuses to overwrite an existing output path.
  - Verified by `tests::native_sdb_cli_export_roundtrips_and_passes_inspection`, which exports a model via the CLI, verifies 100% `inspect_native_sdb` stream parsing (`parsed_bytes == stream_bytes`), and confirms round-trip model extraction.

