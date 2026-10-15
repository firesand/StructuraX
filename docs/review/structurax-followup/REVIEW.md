# PR 1 review follow-up

Baseline: `5824cceb409343b362dcbc7f2bdd05534d7971db`. Date: 2026-10-15.
The first repair's passing tests missed real S2K export and cable-initialization regressions. This follow-up adds cases that exercise those paths and repairs native file readback. PR 1 remains a draft; no merge or release is implied.

## 1. Supported S2K source can export

Retaining source text no longer automatically blocks native synthesis. The guard checks the source against its parsed baseline and supported table/column ownership. Formatting, comments and recognized canonicalization are accepted. Unknown tables/columns, omitted/merged rows, conflicting aliases, unrepresented values and meaningful text outside tables still reject with diagnostics. The original model/source text remains unchanged; there is no implicit clearing of preservation metadata or lossy CLI switch.

A handwritten two-joint `.s2k` now runs through the actual CLI export route and reopens with the expected coordinates/connectivity. Adding an opaque `VendorTag` rejects and creates no output. Default material metadata also required repair: native COLORREF values are written/read, and native import explicitly uses `Fys=Fy` when independent shear-rebar yield has no represented field. Import diagnostics disclose this assumption. This is a reconstruction fallback, not evidence that the vendor stores independent Fys that way. An unequal independent Fys still rejects export; no unrelated native field was used as a hidden carrier.

The precise format boundary and local corpus evidence are in [format-fixes.md](../../reconstruction/format-fixes.md). This does not make every vendor S2K table exportable.

## 2. Cable initialization uses compatible precision measures

The initializer previously demanded `1e-12` relative horizontal-force accuracy while its inner solver controlled length error. The corrected final criterion propagates that length uncertainty through the catenary tangent: `1e-12*H + hypot(Kxx,Kxz)*tau_L`. Internal compatibility accuracy and Newton refinement remain unchanged.

An independently chosen 488-case level/inclined grid includes the reported 10 ft / 10 in² / 1 kip case. All cases pass a separate physical horizontal-tension check at `1e-9` relative; the maximum measured error is `2.83914e-10`. Ninety-two cases exceed the old cutoff. Twelve consistent unit rescalings also pass. This is not claimed to reproduce the reviewer's unavailable 480-case grid. The grid, exclusions and error rationale are recorded in [solver-fixes.md](../../reconstruction/solver-fixes.md).

## 3. Native file I/O has one contract

The library helper used by the CLI, API and GUI exclusively creates a new file. It synchronizes the output, reads its actual bytes back, compares them with the serialized bytes and inspects the readback stream before returning success. The CLI additionally reopens the file for its inspection report. Existing files and symlink destinations are refused consistently; no pre-existing destination is truncated. Failed writes or verification remove the new incomplete output.

Regressions inject silent truncation, equal-size corruption and a write error; check malformed serialized bytes; refuse existing files/symlinks without changing their targets; and verify repeated API/CLI exports preserve the prior bytes. This changes the former API/GUI native-overwrite behavior deliberately. It does not change ordinary S2K/private-save policies or promise crash-atomic publication.

## Secondary findings

- Concrete beam M2 rejection now uses a dimensionless `1e-9` fraction of the same beam's maximum M2/M3 demand. Kip-in and kN-mm tests make the same accept/reject decision. A nonzero M2-only demand still rejects at any scale; unsupported flexure is not silently treated as zero.
- Explicit imported iteration budgets remain unchanged. Raising a requested 10 iterations to 80 would violate that contract; exhaustion remains an error rather than a fabricated result. Relative nonlinear tolerances must be finite and inside `(0,1)`.
- Linear history borrows the stiffness matrices instead of cloning both. The redundant acceleration-assembly block was removed.
- A completely slack unloaded network has zero residual and zero force scale, so it satisfies equilibrium exactly. A regression confirms this; zero displacement is an equilibrium representative, not proof of uniqueness or stability.
- The 6-003 `NLSTAT3` and `NLDHIST2` continuation refusals are checked individually. Portable guard tests also cover static/dynamic consumers of a history initial state. The removed numerical reference was not replaced by a passing rejection count.
- No new temperature-load restriction was added to member limits; the dropped claim did not warrant one.

## Validation and remaining limits

**230 fixture-enabled tests, 223 portable tests, strict Clippy, native build and 59/59 selected reference checks passed.** The integrated commands, counts, source hashes and local log fingerprints are in [validation.json](validation.json). Component regressions supplement the existing suite rather than using total passing counts as a completeness claim.

Native SAP2000 runtime acceptance, comprehensive structural/design-code correctness, general nonlinear continuation and full desktop/platform validation remain outstanding. This change repairs reviewed regressions within the documented subset.
