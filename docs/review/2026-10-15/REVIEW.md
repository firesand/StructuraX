# SAP2000 reconstruction reliability review — 2026-10-15

## Scope and outcome

The user selected **make the existing reconstruction build and work reliably**.
This pass repairs demonstrated defects in `/home/edo/REA`, adds regression tests,
and exposes unsupported behavior instead of implying complete SAP2000 support.
It does not certify the reconstruction as a complete structural engineering
product or a binary-compatible replacement for SAP2000.

Work was split between GPT-6 Astra (solver and independent integration review),
GPT-6.1 Sol (formats/sections), GPT-6 Luna (capability and design review), and the
primary agent (API, GUI, documentation, integrated validation). Existing source
and extracted vendor artifacts were preserved; no original vendor executable
was launched, no native disassembler was installed, and no source repository
was initialized or reset.

## Reproduced defects and changes

### Numerical analysis

- Newmark initialized acceleration to zero even with nonzero force at time zero.
  A unit-mass oscillator with stiffness 4 and time step 0.1 produced first-step
  displacement 0.0024752475 instead of 0.0049504950. Initialization now satisfies
  equilibrium; tests include the discrete first step and the closed-form SDOF
  response over one second. Formula reference:
  [OpenSees Newmark documentation](https://opensees.github.io/OpenSeesDocumentation/user/manual/analysis/integrator/Newmark.html).
- History forces and acceleration directions now respect joint-local axes.
  Named coordinate systems on active joint force/displacement, frame point/
  distributed and area uniform loads now fail explicitly; their transformations
  were previously ignored. Unused load patterns do not block analysis.
- Analysis clears old design/results before running and removes partial results
  on failure; a failed run leaves the model unlocked.
- Initial-case dependencies are ordered and invalid/missing/cyclic dependencies
  rejected. Nonfinite algebra inputs, zero-length frames, invalid dynamic
  functions/time steps and loaded zero-stiffness DOFs fail explicitly.
- Unsupported analysis types and incomplete HHT/Wilson/nonlinear/large-displacement
  paths no longer fall through to different linear/static algorithms.
- A collinear, loaded quadrilateral previously returned success and zero
  reactions. Area geometry validation now rejects degenerate, repeated-corner,
  crossed or concave mappings before meshing, while accepting valid skewed quads
  in either orientation.

These are bounded guards, not exhaustive validation of every modeled feature.

### File and section interoperability

- S2K load positions specified as absolute distances are converted correctly;
  relevant assignment parsing no longer depends on table order.
- Quoted/tab/Unicode identifiers survive serialization. Invalid units, malformed
  records, critical invalid numbers and ambiguous coordinate imports are rejected.
- Native SDB coordinate-system names and array-reference indices are retained.
- PRO channel/angle minor-axis section modulus uses the horizontal centroid.
  Header validation and declared-record handling prevent corrupt interpretations.
- `ImportReport`, `load_model_with_report`, and `parse_s2k_with_report` expose
  omissions. CLI prints these warnings; the GUI retains them in the explorer;
  the Rust API exposes `import_warnings`.
- **Generated SDB files use a private sap2000-rs payload. They are not native
  SAP2000 serialization.** CLI/GUI and README now disclose this. S2K remains the
  supported text-interoperability route for the implemented subset.

### Design checks

- Checked entry points return errors for unsupported code names, absent member
  demand, nonfinite force/station data and unimplemented local-3 shear demand.
  They clear previous outputs rather than leaving stale success results.
- API, CLI and GUI use these checked entry points. The old void functions remain
  source-compatible but cannot return diagnostic errors; new callers should use
  the `try_run_*` APIs.
- EC2 beam minimum-rebar baseline now uses the EC2 calculation.
- This pass does not add a verified local-3 shear formula or validate full code
  compliance. The default 3D demo analyzes and renders, but steel design now
  reports its unsupported V3 demand rather than presenting an incomplete check
  as successful. The tested planar steel API workflow still passes.

### Application and API

- Save-on-exit uses the current `cFile` destination, including Save As. An unsaved
  model returns an error instead of silently skipping a requested save.
- New blank/reset operations clear the previous destination and output selection.
- API edits reject locked models. Unlock/delete-results clears analysis and both
  design result maps. Direct public mutation of `inner` still bypasses safeguards.
- GUI load failures remain visible. Design reruns current analysis and stops on
  failure; it cannot overwrite an analysis error with a success message.
- Editing supports, loads, sections, active DOFs and geometry clears old results.
  Changing units invalidates outputs instead of relabeling old-unit values.
- Deleting a joint also deletes attached elements and clears transient drawing
  state. Generated numeric IDs use a free name instead of overwriting surviving
  frames/areas after deletions. Template regeneration drops obsolete links and
  design outputs from the previous geometry.
- Verification displays the actual pass fraction and labels tolerance-based
  comparisons as PASS rather than EXACT MATCH. README/CLI distinguish the
  eight reconstructed example families from the full vendor verification suite.
  Documented commands now match the actual CLI.

## Validation evidence

Final results: **56 tests pass** (23 at baseline), strict workspace Clippy passes
with warnings denied, and the native build passes. The CLI passes **38 reference
comparisons** across eight reconstructed example families.

Command outcomes are in `validation-summary.json` beside this report. Full test/build/Clippy/CLI/runtime logs are under
`target/review/2026-10-15/` (generated artifacts, removed by `cargo clean`).

Checks include the workspace tests, strict Clippy, native build, CLI help,
reference verification, vendor model analysis, exact solved-result equality
through an S2K save/reload, and PRO lookup. Fixture tests read all 336 local native
SDB examples and compare the supported model projection through S2K; they also
parse 28 standard PRO databases. Those import tests are not 336 validated
structural analyses. `joists.pro` uses an unsupported format.

The native GUI was launched under Xvfb with software OpenGL, stayed running,
and rendered the default 3D/2D frame diagrams. A real click opened Verify Suite
and displayed 38/38 passing comparisons. Both screenshots were visually inspected:
[startup](gui-startup.png), [verification window](gui-verification.png).
This is a startup/render and one-action check, not a complete mouse/keyboard workflow
or hardware-GPU test. Test-owned GUI/Xvfb processes were terminated afterward.

## REA evidence and its limits

REA's installed CLI inspected the supplied `extracted/files/sap2000.exe` without
executing it. Active-session MCP tools were unavailable, so no MCP attachment
claim is made. Artifact SHA-256:
`e32030006c5f7ad9cd66feceb9ce8579a6b91dd399d00b5cd2744b2826ab55c5`.
Evidence ID:
`ev_7a4c25dbfbba16b340883c4ef5aa0736d0c1239bcbf3f53b90d4fa12b148fdce`.

The evidence identifies a managed x64 PE/.NET Framework CIL assembly with
2,721 type definitions and 71,198 method definitions. It did not execute,
resolve CLR dependencies, or decompile C#. A compact identity record is in
`sap2000-managed-identity.json`; the complete evidence is retained under the
corresponding `target/review/2026-10-15/` path. Existing `docs/re/raw_solver/`
evidence was available for orientation, but old narrative claims were not
accepted as current runtime proof. Source-visible defects were corrected from
source and regression evidence; deeper decompilation was not needed for these
repairs.

## Remaining work before broader reliability claims

- Native SDB import is partial; solids/links, functions, many area loads and
  advanced assignments may be omitted. Unknown S2K tables/columns are not
  round-trip-preserved. Warnings must be reviewed before trusting a model.
- Full nonlinear material/link behavior, convergence reporting, native SDB
  writing, broad OAPI/COM parity and many vendor analysis modes remain incomplete.
- Numerical/code-design coverage is a small subset of the vendor product;
  the rest of the vendor verification corpus needs explicit reference comparisons.
- Large-model performance, all templates, arbitrary malformed input, complete GUI
  interaction, original SAP2000 interoperability, other operating systems and
  the manifest's minimum Rust 1.85 toolchain remain unverified.
- Local fixture paths are currently host-specific. Portable fixture discovery
  and explicit missing-fixture reporting remain useful follow-up work.

The reliable claim is that the recorded supported workflows and regressions pass
on this machine, with specific unsupported paths now failing visibly. There is
no evidence for “100% SAP2000 compatibility.”
