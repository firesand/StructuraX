# Contributing

Keep changes scoped and preserve unrelated work. Record the supported model and
load combinations for each solver or design feature. Add an independent closed-form
or attributed reference test, including units and tolerances, and negative tests
for unsupported inputs. A self-roundtrip alone is not external format acceptance.

Run `cargo test --locked --workspace`,
`cargo clippy --locked --workspace --all-targets -- -D warnings`, and
`cargo run --locked --bin structurax -- verify` before proposing a change.
Vendor fixtures are optional, local-only inputs; say when those tests were skipped.
Enable the new solver corpus with `cargo test -p sap-solver --features vendor-fixtures --test wave4_vendor_regressions`; `STRUCTURAX_VENDOR_FIXTURES` sets its directory.

Do not commit extracted vendor artifacts, section databases, installers, raw
binary/decompilation dumps, credentials, generated packages, or build outputs.
Build release binaries against the supported platform baseline; never lower ELF
symbol-version tags to claim compatibility. Report actual tested platforms.

Report solver bugs with a minimal shareable model, expected and actual quantities,
units, case settings, and the exact revision. Review findings and incomplete
validation remain visible until a regression demonstrates the repair.
