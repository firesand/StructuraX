# SAP2000 v19 reconstruction — working state

Updated 2026-10-15.

**Latest checkpoint:** [PR review follow-up](../review/structurax-followup/REVIEW.md) correct supported-subset numerical/design defects and reject combinations previously processed incorrectly. [The baseline review](../review/structurax-latest/REVIEW.md) and wave notes remain historical evidence. Native runtime acceptance is still unverified.
 The requested destination is a fully implemented, reliable reconstruction of the supplied SAP2000 v19 product. **That destination has not been reached.** The source is an experimental native Rust application with selected working analysis, design, and interchange paths. Enum coverage, a successful build, and synthetic benchmark agreement do not establish product parity.

## Reference and environment

- Workspace: `/home/edo/REA`; preserve existing source and extracted artifacts. Git repository initialized for the private StructuraX development project.
- Supplied installer: `/mnt/Extern2/Torrents/CSI SAP2000 Version 19.0.0/SAP2000v1900Setup64.exe`, 522271856 bytes.
- Installer SHA-256: `866c8598f2a7c89ae42ebd5f1badbd4e2afced686c97f05d99b227896810b5bf`.
- Extracted manuals, managed binaries, 336 SDB examples, and 28 PRO libraries are local reference inputs. The installer has not been launched. A working licensed SAP2000 comparison installation has not been established; the installer path alone does not establish one.
- The user's supplied `CSiRefer.pdf` exactly matches the extracted July 2016 analysis manual. It is an analytical reference, not an OAPI specification. The supplied `SapTutor.pdf` has a January 2022 title but a Version 16 introduction. The extracted October 2016 tutorial explicitly identifies Version 19 and is the primary tutorial reference. Both originals are preserved; hashes/metadata are in `manual-provenance.json` and workflow checks in `tutorial-acceptance.md`.
- REA 6.5.0 is installed and now verified live through MCP (207 tools, 6 prompts). Execution-free metadata inspection of `sap2000v19.dll` confirms assembly 19.0.0.0; selected editing/material interface signatures and hashes are saved in `rea-api-evidence-wave2.json`, with connection evidence in `rea-connection-wave2.json`. No native decompiler engine, Wine, or .NET comparison runtime is configured.
- Earlier reliability evidence is in `docs/review/2026-10-15/REVIEW.md`. That dated report describes its own checkpoint; the following work supersedes specific unsupported-feature statements there.
- User-supplied CSI video catalog: `video-references.md` maps selected topics to acceptance work; `video-catalog.json` records 46 SAP2000 entries. Catalog metadata was inspected, not video playback. Explicit v23/v25 enhancement videos are outside the v19 baseline; other video versions still require checking.
- Additional user-supplied YouTube playlist: `supplementary-playlist.json` records three beam tutorials (simply supported, cantilever and continuous), with no overlap against the CSI catalog. Their proposed baseline checks are in `video-references.md`; video content, version and numerical results remain unverified.

## Implementation sequence and acceptance

1. Establish a traceable capability/fixture inventory, fix incorrect native semantics, and preserve unknown records. Every fixture needs source identity, input settings, expected quantities, units, and tolerances. See `CAPABILITIES.md` and `capabilities.json`.
2. Complete linear frame/area physics and dynamics against analytical and vendor fixtures. Add missing coordinate transforms, link damping/orientation, element load recovery, and accurate modal/mass treatment. Unsupported inputs must be reported explicitly.
3. Implement nonlinear constitutive response, cable/tension-only behavior, nonlinear dynamics, solids, and remaining analysis cases in individually validated slices. Existing approximations are not accepted substitutes for these capabilities.
4. Complete material- and shape-specific design rules, biaxial/shear/torsion interactions, code editions, overwrites, and detailed reports. Validate both successful and failing member examples.
5. Extend native SDB semantic import/export and S2K preservation. An unchanged native copy is a preservation feature; scalar byte patching is an experimental library primitive, not a validated native model editor. Generated private-payload SDB files are not vendor serialization.
6. Complete API contracts and native GUI workflows over verified primitives. Validate create/edit/save/reopen/analyze/design/report workflows and expose omissions consistently.
7. Run differential tests against a compatible SAP2000 v19 installation and the vendor verification suite; document every mismatch and untested platform. Full parity requires this evidence, including native file acceptance, not just self-roundtrips.

## Wave 1 implementation checkpoint

Parallel owners: GPT-6 Astra (dynamics), GPT-6.1 Sol (native format), GPT-6 Luna (inventory, CLI, independent design review), primary agent (design, integration, verification).

- HHT and Wilson-theta integration and consistent massless initial conditions: `solver-wave1.md`.
- Full native record inventory/preservation, experimental scalar patching, expanded model imports, corrected internal analysis-case mapping (436 formerly misclassified cases), and S2K Wilson theta: `formats-wave1.md`.
- AISC local-3 shear and material/closed-torsion guards: `design-wave1.md`.
- `inspect-sdb` and `copy-native-sdb` expose native inventory and exact source preservation without overwriting an existing output.
- Final integrated checks pass: **79 tests**, strict workspace Clippy, native build, and **38 selected synthetic reference comparisons**. Full inventory/exact copying passed on all **336** native fixtures. Supported model import/roundtrip passed on **334**; **2** are explicitly rejected for unsupported history settings. The native GUI completes steel design for the default **26-member** model and displays ratios. Commands, scope and results for wave 1 are recorded in `validation-wave1.json` and `target/reconstruction/` logs. These results do not establish complete product parity.

## Wave 2 implementation checkpoint

Parallel owners: GPT-6 Astra (solver), GPT-6.1 Sol (editing/API and independent solver review), GPT-6 Luna (independent beam fixtures and API/GUI review), primary agent (GUI, REA evidence, integration and verification).

- Transactional selected-object move, linear replication and equal frame division, preserving connectivity/assignments and clipped loads. Failed operations preserve inputs and results; unsupported offset/cable/nonprismatic division rejects. Native cEdit facade and area pressure/modifier setters: `editing-wave2.md`.
- Native GUI workflow tabs for geometry, materials, loads, area sections/mesh/modifiers, releases and combinations. Validated edits, dependency checks, S2K save/reopen and headless egui render/pointer tests: `gui-wave2.md`.
- Named Cartesian load transforms, finite/grounded link orientation and shear offsets, mass/self-weight, viscous damping in all four integrators, and physical dynamic reaction snapshots: `solver-wave2.md`.
- Corrected span-load station deflections, thermal eigenstrain/fixed-end forces, projected moment scaling and tapered self-weight equilibrium. Independent simply supported/cantilever/two-span checks: `beam-workflows.md`; thermal and coordinate/link regressions cover further closed forms.
- Bounded API validation repairs prevent invalid point/frame/material calls from clearing results or silently accepting unsupported values. Material defaults now use model units; unrepresented material families/expected strengths/lightweight behavior reject explicitly. Full facade/COM compatibility remains incomplete.
- Final integrated checks pass: **127 tests**, strict workspace Clippy, native build, and **38/38** selected synthetic reference checks. The imported native sample runs all seven cases. Workspace formatting remains mixed. Exact scope is in `validation-wave2.json`; logs and the source manifest are under `target/reconstruction/wave2/`. See the updated tutorial acceptance map for implemented operations versus pending full-workflow acceptance.

## Wave 3 implementation checkpoint

Parallel owners: GPT-6 Astra (nonlinear solver and format review), GPT-6.1 Sol (source preservation and native nonlinear-data import), GPT-6 Luna (EC3 shear and independent solver/API review), primary agent (case/function GUI, link/API validation, integration and verification).

- Small-displacement nonlinear static Gap/Hook equilibrium with explicit per-DOF activation, exact opening laws, physical reactions, convergence failures and analytical fixtures. P-Delta now honors requested iteration/tolerance and rejects nonconvergence: `solver-wave3.md`.
- S2K source retention and conservative edited-table reconciliation preserve opaque records and columns. Ambiguous ownership, orphan-row loss and unsafe unit changes fail before writing. Alias regressions prevent stale fields overriding edited values. Native nonlinear flags import with incomplete constitutive-data guards; Rust extensions are explicitly labeled: `formats-wave3.md`.
- EC3 local-3 shear with optional local-2 bending for solid rectangular sections, including high-shear bending reduction and consistent capacity reporting. Unsupported concurrent demands and other shapes still reject: `design-wave3.md`.
- Native GUI case/function editors and run selection, typed case/function/link APIs, transactional validation and persistent export diagnostics. API-created oscillator, modal/spectrum and Gap/Hook models have independent numerical checks; GUI tests exercise headless input/rendering: `application-wave3.md`.
- Final integrated checks pass: **167 tests in 26 suites**, strict workspace Clippy, native build and **38/38** selected synthetic reference checks. All **336** native fixtures pass inventory/copy checks; **334** pass supported projection roundtrips and **2** reject unsupported history settings. The native sample runs seven cases. Exact source/binary hashes, commands and scope are in `validation-wave3.json`; logs are under `target/reconstruction/wave3/`.
- This checkpoint does not establish full reconstruction, vendor-runtime parity or a new desktop acceptance pass. Wave 2 evidence is retained in `validation-wave2.json`.

## Wave 4 implementation checkpoint

Parallel owners: Solver Solid & Cable Engine (3D solid brick + Wilson-Taylor incompatible modes + Peyrot-Goulois 3D catenary cable + frame T-C limits), Native SDB Export & Import Engineer (schema-driven native `cBinaryStream` export + nonlinear link `m_param` + cable SDB import), Steel & Concrete Design Engineer (AISC Angle/DblAngle `V3`, EC3 `V3` across 8 shapes, AISC H3.1/H3-6 closed-section torsion, ACI/EC2 concrete minor-axis shear `V3` and flexure `M2`), primary agent (nonlinear direct-integration time-history & state continuation, grounded links & Sine functions in SDB, API/CLI integration, vendor `.sdb` regressions, and verification).

- 8-node 3D isoparametric solid elements (`solid-elements`) with Taylor-Beresford-Wilson (1976) 9 incompatible bending modes, degenerate wedges/tetrahedra, self-weight, `Face 1..6` surface pressure, uniform temperature loads, and `SolidResult` recovery; exact 3D elastic catenary cables (`cables-and-member-limits`, Peyrot & Goulois 1979) and frame tension/compression limits (`tc_limits`); and nonlinear direct-integration time-history (`NonlinearHistory` / `NonlinearDynamic`) for `Gap`, `Hook`, and Maxwell-series `Damper` links with `initial_case` state continuation: `solver-wave4.md`.
- Schema-driven native `cBinaryStream` serializer (`pack_model_to_native_sdb` / `save_native_sdb_file`) writing all 64 top-level SAP2000 v19 sections and passing `inspect_native_sdb` 100% to EOF, plus native SDB extraction of `NLLINKPROPARRAY.m_param` (`5 x 6 = 30` `f64`), single-joint grounded links (`m_SingleJoint`), parametric Sine functions (`FUNCSHUNARRAY`), and `CABLEPROPERTYARRAY` / `S_CableData`: `formats-wave4.md`.
- AISC 360-10 `Angle` and `DblAngle` local-3 shear (`V3`), EC3 local-3 shear across 8 section families (`Rectangular`, `I`, `Channel`, `DbChannel`, `T`, `Box`, `Pipe`, `Circle`), AISC 360-10 closed-section (`Box`, `Pipe`) H3.1 torsional strength and Eq. H3-6 combined interaction, and ACI 318-14 / Eurocode 2 concrete minor-axis shear (`V3`) and beam minor-axis flexure (`M2`): `design-wave4.md`.
- `cPropSolid`, `cSolidObj`, `cFrameObj` cable/T-C limits, `cAnalysisResults::solid_stress`, `cFile::save_native_sdb` in `sap-api`, and `export-native-sdb` in `sap-app`: `application-wave4.md`.
- Final integrated checks pass: **187 tests in 27 suites**, strict workspace Clippy (`0` warnings), native build, and **60/60** reference checks in `sap2000-rs verify` (adding `Example 5-001` compatible/incompatible solids, `Example 6-003` gap static/history, `Example 6-005` Maxwell damper history, `Example 7-001` catenary cables `C1/C2/C6`, and `Example 7-002` two-segment point-loaded cable, alongside end-to-end vendor `.sdb` solver regressions on `example_5_001_comp.sdb`, `example_5_001_incomp.sdb`, `example_5_002a1_incomp.sdb`, `example_5_005a.sdb`, `example_6_004.sdb`, `example_6_005a.sdb`, `example_7_001.sdb`, and `example_7_002.sdb`). All **336** native fixtures pass inventory/copy checks; **334** pass supported projection roundtrips and **2** reject unsupported history settings. Exact source/binary hashes, commands, and scope are in `validation.json`; logs are under `target/reconstruction/wave4/`. Wave 3 evidence is retained in `validation-wave3.json`.

## Post-review repair checkpoint

- Corrected nonlinear-history constitutive reactions, grounded Maxwell recovery, static-to-history initial acceleration, solid zero modifiers and requested convergence controls. Cables, capped members and continuation now reject unsupported combinations explicitly; `solver-fixes.md` records exact bounds.
- Corrected AISC Box/Pipe interactions and unequal-wall HSS dimensions. EC3 local-3 uses verified code shear areas for Rectangular/Pipe/equal-wall Box. Other shapes and concrete beam M2 reject; `design-fixes.md` records the scope.
- Native synthesis validates references, incomplete nonlinear data, source preservation and semantic roundtrip loss before writing. Class versions derive from the observed corpus. `format-fixes.md` distinguishes 329 accepted native projections, 5 incomplete-law export refusals and 2 unsupported imports in the 336-file corpus.
- Solid API invalid geometry and temperature overflow reject atomically. Typed/editor nonlinear history from rest, export warnings and CLI no-overwrite are covered in `application-fixes.md`.
- Integrated commands, counts and fingerprints are in `../review/structurax-fixes/validation.json`. Historical wave-4 counts include a continuation benchmark now removed because it exercised an unsupported path.

## PR review follow-up

Handwritten supported S2K can export without clearing preservation; actual opaque source data still reject. Native material colors roundtrip; independent Fys defaults to Fy on native import and unequal values reject synthesis. All native save entry points now refuse overwrites and verify synchronized disk readback. Cable horizontal-tension initialization is checked against compatibility-solver precision, with a documented stiff/taut sweep. Concrete beam M2 rejection is relative to moment demand and unit-independent. Explicit iteration budgets remain in force; additional continuation and slack-network tests distinguish rejection from missing coverage. Exact tests/source evidence are in `../review/structurax-followup/validation.json`.

## Remaining high-priority limits

Nonlinear link constitutive families beyond `Gap`, `Hook`, and `Damper` (`PlasticWen`, `Isolator1`, `Isolator2`), large-displacement corotational frame/shell kinematics, and `MovingLoad` / `SteadyState` / `Psd` / `Hyperstatic` case types remain unsupported. General viscous damping on free massless coordinates outside Maxwell damper elements requires DAE treatment; exact link fixed constraints, end-offset station kinematics, and complete dynamic step histories remain partial. Named-coordinate three-angle ordering needs vendor comparison. Native SDB export (`pack_model_to_native_sdb`) rejects represented-data losses and incomplete source laws. Unmodeled native tables still have no general export implementation, and generated `.sdb` files have not been opened inside a running Windows SAP2000 v19 executable. Code design remains partial for EC3 shape/orientation coverage and concurrent multi-axis demands beyond `V3 + M2`, concrete beam M2 and torsion, ERW/SAW thickness overwrites, auto-select sections, and generated code combinations. Broader GUI/API coverage, full tutorial desktop interaction, and licensed vendor-runtime differential execution remain unvalidated.

Resume from the first unverified acceptance item in this state file and the capability/tutorial inventories. Preserve completed wave evidence and recheck source before using historical line references. Do not repeat setup or relabel this checkpoint as a completed reconstruction.
