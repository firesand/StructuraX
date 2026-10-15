# Review reproductions

These diagnostic programs demonstrate current bugs through the public Rust APIs.
A successful program exit is not engineering acceptance. Run from the repository:

```sh
cargo run --manifest-path docs/review/structurax-latest/reproductions/solver/Cargo.toml
cargo run --manifest-path docs/review/structurax-latest/reproductions/format-api/Cargo.toml --bin structurax-format-api-repro
```

These programs are independent of the passing workspace tests. The optional
`class_versions` binary also needs the locally supplied vendor fixture named in
its source; that input is not distributed. The adjacent review reports record
observed values and the original local review commands.
