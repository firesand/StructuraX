# S2K source preservation and nonlinear import safety — wave 3

Implemented 2026-10-15. This slice preserves text interchange evidence while editing supported model data. It does not implement unknown table physics, establish complete S2K coverage, or provide arbitrary native vendor SDB export. A reconstruction private snapshot remains a private format. No licensed vendor import/differential runtime was available for acceptance.

## Source retention and checked exports

`SapModel.s2k_preservation` is optional, serde-defaulted metadata containing the original `source_text` and the supported `canonical_baseline`. It contains no recursive model snapshot. The metadata survives model cloning and JSON serialization; private snapshots carry the preserved text. The format crate enables serde_json's `float_roundtrip` feature so JSON reopening does not cause one-ULP material/default changes that would falsely count as a structural edit.

Unchanged source exports retain every original UTF-8 byte, including CRLF/LF choice, headers, whitespace, continuation lines, escaped quoted strings, Unicode, comments, duplicate table occurrences and trailing data. Unsupported tables are treated as opaque blocks rather than forcing their rows through the supported Key=Value grammar. Their duplicate fields and unrecognized row syntax therefore survive. Duplicate fields inside interpreted records still cause a parse error; the importer does not guess which value represents the modeled quantity.

After supported edits, unchanged table blocks remain verbatim. Changed supported tables are regenerated from the current model and original opaque column tokens are attached only to a unique owner identity. Parser column reads are captured so that an ignored source field remains opaque even when the old writer happened to emit a fabricated/default value for that column (for example steel EffFy/EffFu). Replayed tokens retain original key spelling, quoting and number syntax. Any colliding regenerated field is removed first, avoiding duplicate keys/stale-value precedence.

Recognized aliases such as joint GlobalX/GlobalY/GlobalZ, absolute frame-load distances and function Accel/Value/Period/Time are replaced by the current canonical representation when that table changes. This prevents an old GlobalX from overriding a newly edited XorR or an old absolute distance from competing with a new relative distance.

Owner identities use object/property names and appropriate assignment keys. New rows receive no unknown fields from another record. Deleting an owner also removes that row's opaque columns, with a diagnostic. Opaque tables retain their original object references and are not retargeted on rename/delete. Changed exports warn that those references and unsupported assignments still need review. A name reused after deletion is the same interchange identity; this format does not have independent persistent object GUIDs for disambiguating that operation.

Unknown fields on duplicate/ambiguous owner records cause an actionable `PreservationConflict` instead of arbitrary association. Duplicate records/tables are still byte-preserved when their supported projection is unchanged. Unprojected source rows are diagnosed even when every column name is known, and regenerating their table is rejected if it would erase them. Previously unprojected source records cannot silently compete with a newly generated table of the same name; that collision is rejected. Edits to complex repeated rows can therefore be refused when a unique source identity cannot be established. This is a deliberate limit, not an automatic lossy fallback.

Unit changes are rejected while opaque source columns/tables remain, because their dimensions are unknown and raw numeric values cannot safely be scaled. A caller may explicitly clear `s2k_preservation` to request a supported-only export after retaining the original source. Metadata is not silently dropped by conversion, edits or export.

Public interfaces:

- `try_write_s2k_with_report(&SapModel) -> Result<ExportReport, FormatError>`, with public `ExportReport { text: String, warnings: Vec<String> }`;
- `save_s2k_file_with_report(&SapModel, path) -> Result<Vec<String>, FormatError>`;
- `save_sdb_file_with_report(&SapModel, path) -> Result<Vec<String>, FormatError>` for a reconstruction private snapshot.

The checked save paths compute one projection before writing, so a preservation conflict leaves an existing destination untouched. Legacy save wrappers discard the returned warnings for signature compatibility. Legacy `write_s2k -> String` is a convenience wrapper that panics on a preservation conflict; user-controlled exports should use the checked interface. GUI/CLI/API warning handling is integrated by the application owner.

## Nonlinear source-data boundary

The supplied native schema `NLLINKPROPARRAY` explicitly contains `m_Nonlinear:bool[]` (`sdb_schemas.txt`, schema entry 773). Native projection now imports those exact per-DOF flags. It does not infer nonlinear activation from stiffness magnitude, so zero nonlinear stiffness remains distinct from an inactive law.

The native opening/constitutive `m_param` layout remains unmapped. `LinkProp.nonlinear_data_complete` defaults to true for explicitly constructed Rust models but is false for imported activated nonlinear laws whose constitutive data is unavailable, and for nonlinear native properties with missing activation vectors. This retains valid effective-linear analyses and lets the solver reject unsupported nonlinear use. Import reports name incomplete properties. Native byte inventory and exact-copy preservation remain separate from semantic projection.

No supplied vendor text fixture established a S2K nonlinear-flag column/table contract. The reconstruction writes explicitly labeled `RustNonlinear` and `RustNonlinearDataComplete` extensions for its own text/private-snapshot roundtrip, with diagnostics and no vendor serialization claim. Existing opaque vendor nonlinear tables remain intact. A nonlinear family without explicit verified reconstruction activation/data is marked incomplete rather than silently treating missing source flags as usable all-false law data. The solver owner added the corresponding nonlinear-case guard.

## Verification

The targeted preservation suite has eleven passing regressions:

- unchanged byte-exact source through JSON and private snapshot reopening, including duplicate opaque tables/fields, Unicode, CRLF, continuations and comments;
- edited joint coordinate with opaque token retention and removal of stale global-coordinate aliases;
- added/deleted rows with no cross-owner opaque-field transfer;
- ambiguous duplicate owners rejected before overwriting an existing file;
- opaque numeric data blocking unit conversion until explicit supported-only export;
- missing source nonlinear flags not inferred from nonzero nonlinear stiffness;
- explicitly activated zero-stiffness nonlinear flags and incomplete-data status surviving reconstruction roundtrip;
- uninterpreted expected steel strengths retained without duplicate keys after a nominal-strength edit;
- absolute frame-load source distances replaced by one edited relative interval;
- known-column orphan source rows diagnosed and protected from loss during a load edit;
- function ordinate/sample aliases normalized so old Accel/Value and Period/Time values cannot override edited canonical data.

Final `cargo test -p sap-format -p sap-model` passed all 41 tests: 18 existing format tests, 11 preservation regressions and 12 model tests, including native corpus inventory/copy and model roundtrip checks. The format library suite completed in 14.37 seconds after indexing orphan/owner lookups; noncanonical source diagnostics remain enabled. `cargo clippy -p sap-format --all-targets --no-deps -- -D warnings` passed. The integration owner runs final whole-workspace checks after ownership freezes. These automated checks do not prove vendor acceptance, unknown-record semantics, arbitrary native SDB editing or full product parity.
