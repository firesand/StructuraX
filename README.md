# StructuraX

StructuraX is an experimental native structural-analysis and member-design application written in Rust. It has an egui desktop interface, a command-line interface, and typed Rust APIs. The internal crates retain their `sap-*` names from the reconstruction workspace.

**Development status:** selected analytical checks pass; full SAP2000 compatibility and engineering acceptance have not been established. The `1.0.0` package version is inherited metadata, not a certification or declaration of production readiness. See [review repairs](docs/review/structurax-fixes/REVIEW.md) before relying on the newer solver and export paths.

## Implemented areas

- Frame, area/shell, and solid element analysis, plus selected cable and nonlinear link formulations.
- Static, modal, buckling, response-spectrum, and selected direct-integration history cases.
- Selected AISC 360-10, EC3, ACI 318-14, and EC2 member checks.
- Geometry/load/material editing, case/function definitions, result displays, and Rust API access.
- Selected S2K text interchange with source preservation; native SDB inspection, import, exact copying, and experimental export.

These are scoped implementations. Unsupported combinations and known defects are documented in the review and [capability inventory](docs/reconstruction/CAPABILITIES.md). Native export passing this project's parser does not prove that SAP2000 will accept the file. Layered sections, general large-displacement behavior, full design-code coverage, complete history continuation, and broad GUI/API parity must not be inferred from the feature list.

## Build and run

Install a current stable Rust toolchain. On Debian/Ubuntu, the desktop build also needs:

```sh
sudo apt-get install pkg-config libx11-dev libxcb-render0-dev libxcb-shape0-dev libxcb-xfixes0-dev libxkbcommon-dev libgl1-mesa-dev
cargo build --locked --release --bin structurax
./target/release/structurax
```

The Linux desktop uses X11/OpenGL; a Wayland session needs XWayland. Native Windows and macOS packaging scripts exist, but local Linux checks do not establish those platforms' runtime behavior.

```sh
# Selected built-in reference checks
cargo run --locked --bin structurax -- verify

# Analyze a model, optionally including supported member design
cargo run --locked --bin structurax -- analyze model.s2k --design --output results.json

# Inspect a locally supplied section library
cargo run --locked --bin structurax -- inspect-pro /path/to/aisc14.pro W14X90

# Experimental native export of a supported projection; output must not exist
cargo run --locked --bin structurax -- export-native-sdb supported-model.sdb output.sdb
```

Native synthesis rejects unrepresented data, including retained opaque S2K source and some default concrete metadata. It does not automatically discard that data to make export succeed. Use S2K/private snapshots for supported preservation workflows and native byte-copy for an unchanged original.

## Validation

```sh
cargo test --locked --workspace
cargo clippy --locked --workspace --all-targets -- -D warnings
cargo run --locked --bin structurax -- verify
```

The built-in comparisons are selected synthetic/reference examples, not the complete vendor verification suite. Several legacy tests use locally supplied vendor fixtures and skip when absent. The seven newer solver fixture tests require explicit opt-in: `cargo test -p sap-solver --features vendor-fixtures --test wave4_vendor_regressions`. Set `STRUCTURAX_VENDOR_FIXTURES` to the fixture directory, or use the local `extracted/files/` directory. CI runs without vendor assets. Historical reports under `docs/reconstruction/` describe their own checkpoints and may refer to local-only evidence under `target/`.

## Packages

The GitHub workflow builds development artifacts. It does not publish a production release. Existing local `dist/` packages are excluded from Git.

- **Linux x86_64:** build on Ubuntu 22.04 or an equivalent baseline sysroot, then run `python3 scripts/build_dist_packages.py`. Packaging checks ELF GLIBC requirements and refuses requirements above 2.35. It never rewrites ABI version tags. A successful check is not a substitute for target-distribution GUI testing.
- **Windows x86_64:** run `pwsh -File scripts/package-windows.ps1` on Windows. NSIS is optional for the installer; the ZIP is always produced after a successful build.
- **macOS ARM64:** run `bash scripts/package-macos-arm64.sh` on macOS. Bundles use ad-hoc signing when available; they are not Developer ID signed or notarized.

No installer, vendor binary, vendor manual, vendor model, or vendor `.pro` section database is included in this repository or generated packages. Supply your own permitted section libraries via the application's file selection or `sections/` search path.

## Project and provenance

See [CONTRIBUTING.md](CONTRIBUTING.md) for validation expectations and [ROADMAP.md](ROADMAP.md) for priorities. Reconstruction notes retain the history and limitations of the implementation; they are not a claim of equivalent behavior.

StructuraX is independent of Computers and Structures, Inc. (CSI), and is not affiliated with or endorsed by CSI. SAP2000 is referenced as an interoperability and comparison target. Original project code uses the MIT license; third-party dependencies retain their respective licenses. See [LICENSE](LICENSE).
