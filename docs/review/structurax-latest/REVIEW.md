# StructuraX: review before GitHub publication

**Historical baseline.** Subsequent repairs and explicit unsupported-feature guards are recorded in [the follow-up](../structurax-fixes/REVIEW.md). Findings below describe the pre-repair source.

Date: 2026-10-15. Scope: the current Antigravity/Gemini-attributed workspace changes after the wave-3 manifest, including wave-4 solver/design/native-export code and StructuraX packaging. There was no prior Git history to establish individual edit authorship. Review compared current source, saved manifests, implementation notes, tests and independent reproductions.

## Verdict

Keep this as an **experimental development repository**. The existing regression suite passes, but independent inputs reproduce serious incorrect results and silent native-export data loss. Do not represent the new feature list, version `1.0.0`, or successful schema parsing as production or SAP2000 parity acceptance. No release was tagged or published during this review.

## Confirmed high-priority findings still open

1. **Mixed cable models discard other physics.** A disconnected restrained cable changes an unrelated spring-supported bar displacement from 0.05 to 0.10 because the cable branch replaces ordinary frame behavior and omits supports/elements.
2. **Nonlinear history reactions are inconsistent.** A Hook reports force +5.9601 while its fixed-end reaction is only -0.7748; recovery uses effective linear stiffness instead of the integrated constitutive force.
3. **History continuation loses state.** Splitting an elastic transient into consecutive cases produces displacement 0.00685 versus 0.01415 for the uninterrupted run. Velocity/acceleration and Maxwell internal force are not restored.
4. **Member-limit and cable loads are not consistently represented.** Axial span-load recovery violates a zero compression cap; partial cable distributed loads give the same result as full-span loads. Target end tension is ignored.
5. **Native export silently changes data.** Coupled springs, explicit joint axes, orthotropic vectors, project metadata and opaque S2K records can disappear. Missing references become index 1. Incomplete nonlinear data can become marked complete after export/reimport.
6. **New steel design formulas need correction.** H3 interaction combines directional shears incorrectly and does not distinguish the Pipe flexural expression. Unequal-wall HSS torsion dimensions are wrong. EC3 code shear area is conflated with the analysis shear-deformation area.

Additional findings include zero solid modifiers restoring nominal values, altered convergence controls, solid API overflow/degenerate geometry acceptance, unimplemented API methods claimed in the wave-4 notes, and insufficient concrete minor-axis reinforcement evidence.

Detailed evidence, reproduction values, suggested fixes and reviewed line references:

- [Solver review](solver-review.md)
- [Native format and API review](format-api-review.md)
- [Design review](design-review.md)

Those reports describe the reviewed source. Line numbers may shift with subsequent fixes. Independent Rust reproducer source is retained under `reproductions/`; the temporary Cargo projects mentioned in the reports were local review harnesses, not committed build dependencies. Design arithmetic is recorded directly in its report. Proprietary manuals/fixtures used locally are excluded.

## Repairs made for repository readiness

- Replaced ELF GLIBC symbol-version rewriting with a read-only requirements check. Linux packaging refuses a binary needing GLIBC above 2.35; rebuild on the baseline distribution instead. The current Gentoo release binary needs GLIBC 2.43, so it is deliberately rejected. Existing local packages were not republished.
- Removed automatic `.pro` bundling from Linux, Windows and macOS packaging. Excluded extracted inputs, raw binary-analysis dumps, build outputs and generated packages from Git.
- Added the MIT license file matching existing Cargo metadata, a scoped README, contribution guidance, roadmap and CI workflow. Corrected invented homepage/support links and unsupported platform/signing claims.
- Added explicit Windows native-command exit checks and clean staging to prevent failed builds packaging stale executables.
- Made the seven new vendor solver tests opt-in with `vendor-fixtures`, with `STRUCTURAX_VENDOR_FIXTURES` discovery. Converted remaining developer-specific fixture paths to repository-relative paths. Replaced the unconditional API database dependency with a synthetic section in its end-to-end test, and gave CLI copy/export tests generated native inputs instead of requiring vendor files.
- GUI startup uses the existing section-library search routine. API defaults use `STRUCTURAX_SECTION_DIR` or `sections/`, rather than a developer's home directory.

These repairs address publication/build portability. They do **not** repair the numerical and native-export defects listed above. The roadmap keeps that distinction explicit.

## Verification boundaries

See [validation.json](validation.json) for the independently rerun checks and source fingerprint. Local fixture-enabled tests and a clean source copy without vendor inputs were checked separately. The packaging smoke uses a small ELF fixture to inspect package contents; it is not a StructuraX GUI or target-distribution acceptance test.

Windows/macOS runtime, full GUI workflows, licensed SAP2000 file acceptance, comprehensive design-code verification, and a full vendor differential suite remain untested. Historical `docs/reconstruction/validation.json` is a wave-4 implementation report; this review supersedes its broader acceptance implications.
