# Plastic Wen links and displacement-controlled nonlinear static cases

Date: 2026-11-12. Scope note for one bounded addition. It does not change the
project status: vendor-runtime acceptance and broad parity remain unestablished.

## What is supported

**Plastic Wen link DOFs** (`LinkPropType::PlasticWen`), independently per
deformation DOF, in small-displacement `NonlinearStatic`, `NonlinearHistory` and
`NonlinearDynamic` cases. The law is the one in the CSI Analysis Reference
Manual (July 2016), "Wen Plasticity Property":

```
f = ratio k d + (1 - ratio) yield z
dz/dd = (k / yield) (1 - |z|^exp)   while dd z > 0
dz/dd =  k / yield                  otherwise
```

Parameters: `k_nl` = k, `param1` = yield, `param2` = ratio, `param3` = exp.
Accepted ranges are k > 0, yield > 0, 0 <= ratio <= 1 and 1 <= exp <= 20; anything
else is an input error. Exponents 1 and 2 use closed forms. Other exponents use a
tabulated RK4 solution of the loading curve with cubic Hermite interpolation,
checked against the closed forms to `1e-11` and against an independent
integration to `2e-8`. Linear cases keep using the effective stiffness `ke`.

The hysteretic state `(deformation, z)` of every Wen DOF is stored in
`CaseResults::link_hysteresis` and inherited by a case that names the solved case
as its `initial_case`, in any order of static and direct-integration cases.

**Displacement-controlled nonlinear static cases**
(`LoadCase::displacement_control`), per the manual's "Load Application Control":
the case load pattern is scaled until one monitored joint displacement has moved
by the target magnitude from its value at the start of the case, in the direction
the load moves it. The reached load factor is stored in
`CaseResults::displacement_control_factor`, and a following case inherits the
loads at that factor. Displacement control works for linear, Gap/Hook and Wen
models.

## Bounds that reject instead of approximating

- A Wen deformation that reverses direction inside one load case. The state
  update is exact for a monotonic deformation change; the path is sampled at 8
  equal load increments and a reversal seen between those samples is refused
  with advice to split the case. A reversal that begins and ends between two
  consecutive samples is not detected.
- A monitored displacement that does not increase monotonically with the load
  (load reduction, snap-back), a monitored DOF that is restrained, a load that
  does not move it, or a target that cannot be reached by scaling the load.
- Displacement control with geometric nonlinearity, cables or frame
  tension/compression limits; a monitored generalized displacement; several
  monitored joints.
- Wen links together with frame tension/compression limits, and Wen links in
  Wilson-theta history integration.
- Every other nonlinear link family (multi-linear elastic/plastic, rubber,
  friction and triple-pendulum isolators, bilinear and friction-spring dampers).

The vendor "use conjugate displacement" option is stored and written back but
does not change the result here: it selects the vendor's internal stepping
measure, while the monitored displacement still sets the target. Which native
`m_DispType` value means "conjugate" is inferred, not documented: 1 is the value
every default case in the corpus carries and conjugate is the vendor default, so
1 is read as conjugate and 2 as monitored. Both values roundtrip unchanged.

A load factor with no equilibrium (for example above the capacity of a link with
ratio 0) bounds the displacement-control search from above; the target is then
approached from below.

## Evidence

| Check | Reference | Tolerance |
| --- | --- | --- |
| Link force at 0.2, 0.8, 9.6 in (push) and 9.6, 8.6, -9.6 in (push back) | CSI Software Verification Example 6-008, published 16.836, 43.915, 141.000, 105.000, 48.438, -141.000 kip | 5e-4 kip (published precision) |
| Same points | Closed form for exp = 1 | 1e-10 relative |
| Load control inverts the law; elastic unloading | Closed forms for exp = 1, 2 | 1e-9 relative |
| Exponents 1, 1.5, 2, 3, 7.25, 20 over push, partial unload and full reversal | Independent RK4 of the state equation, 200 000 steps per segment | 2e-8 relative |
| One push split into three cases | Same push in one case | 1e-9 relative |
| Wen oscillator under a load that yields both ways (kip, in, s); Newmark average, Newmark linear and HHT (alpha = -0.05); `NonlinearHistory` and `NonlinearDynamic` | Independent RK4 of `m u'' + f = p(t)`, 400 000 steps | 2e-5 at dt = 0.5 ms; error falls at least 8x when dt is cut 4x |
| Displacement control toward a perfectly plastic capacity (ratio 0, 2 in) | Closed form 50 (1 - exp(-4)) kip | 1e-9 relative |
| Static to history to static continuation | Equilibrium must hold with no motion; elastic unload by 30 kip | 1e-8 to 1e-9 relative |
| kip-in model converted to kN-m | Same response in converted units | 1e-6 relative |

Tests: `crates/sap-solver/tests/plastic_wen_regressions.rs`, unit tests in
`crates/sap-solver/src/nonlinear.rs`, `crates/sap-api/tests/link_definitions.rs`,
`crates/sap-format/tests/native_export_guards.rs`, and the opt-in vendor test
`vendor_sdb_example_6_008_wen_parameters_and_displacement_control`. The six
published values are also rows of `structurax verify`.

The Example 6-008 models are independent single-joint models with the published
link property. The vendor file itself is **not** reproduced end to end: its
loading is a link gravity load and its link orientation is an advanced axis
assignment, and neither is modeled. Import records both and analysis refuses
the file naming them; see [unmodeled-source.md](unmodeled-source.md).

## File formats

- Native import reads the third `NLLINKPROPARRAY.m_param` row as the Wen
  exponent (Example 6-008 stores yield 50, ratio 0.1, exponent 1 for U2), and
  reads `m_SolControl.m_control = 2`, `m_TargetDisp`, `m_DispType` and the single
  `TYNLCDisp` record in `m_djoints` as displacement control. Twenty nonlinear
  static cases in 17 of the 336 corpus files carry this setting. Before this
  change it was dropped and those cases were treated as full-load cases.
- Native export writes the same fields back. The `TYNLCDisp` class tag (version
  1) was observed 767 times in the corpus with no other value. These fields did
  not change the corpus split; the later unmodeled-source guards did (see
  [unmodeled-source.md](unmodeled-source.md) for the current numbers).
- S2K uses reconstruction extension columns, like the existing `Knl`/`Param1`/
  `Param2`: `Param3` on link DOF rows and `RustDisplTarget`, `RustDisplJoint`,
  `RustDisplDOF`, `RustDisplConjugate` on the case row. The vendor's own S2K
  tables for these settings are not parsed; no vendor S2K sample is available to
  verify their layout, so they stay opaque with the existing import diagnostics.

## Found while doing this

Three silent approximations turned up during this work: modal damping ignored
on the modal history types, frame hinges not modeled, and link gravity loads and
advanced link axes not imported. They are now recorded and refused; see
[unmodeled-source.md](unmodeled-source.md), which also lists the native sections
that are still dropped without a record.
