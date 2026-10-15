# Analysis definitions, link API and export diagnostics — wave 3

The native GUI now has Define > Analysis Cases and Define > Dynamic Functions. Case controls cover linear static, nonlinear static, modal Eigen/Ritz, buckling, response spectrum and linear history. The existing analysis action runs the enabled definitions. Saving a definition validates it before mutation, requires an unlocked model, and clears stale analysis/design results only after success.

Static cases take named patterns and factors. Nonlinear cases expose small-displacement or P-Delta geometry, iterations and tolerance; solver applicability checks still determine whether the selected element laws can be combined. Modal cases expose maximum modes and Eigen/Ritz starting patterns. Spectrum cases select a real modal dependency, function/direction/scale tuples, modal combination and damping. History cases select time-step/count, all four implemented integrators, Rayleigh damping and function/excitation tuples. The list of run flags can disable unsupported imported cases without rewriting their definitions.

The function editor accepts two-column whitespace/comma-separated samples and comments. It validates finite, increasing nonnegative time/period coordinates, at least two samples, and nonnegative spectrum ordinates. Changing the kind of a function used by an incompatible case is rejected. Sample ordinates and excitation scales remain explicit inputs; the editor does not generate seismic code spectra or automatic accelerograms.

`sap_api::set_case_definition` and `set_function_definition` expose typed errors. `cLoadCases::set_definition`, `get_definition`, `set_run_flag`, and `cFunctions` provide native facade access. Existing static/modal/P-Delta/buckling setters use the same validation. These additions are not Windows COM signature parity.

## Deliberate boundaries

- Eigen cases cannot contain ignored starting loads. Minimum mode count is fixed at one; nonzero frequency shifts are rejected because these controls are not implemented by the solver.
- Spectrum damping is limited to 0 through 0.99, matching the solver's usable range. Spectrum sources must be modal cases. Initial-state continuation is not exposed.
- History definitions allow up to one million steps. Wilson theta has an editor stability floor of 1.37; the lower-level solver still supports its separately tested values starting at one. HHT alpha is restricted to [-1/3,0]. Newmark linear acceleration remains conditionally stable; choosing an integration scheme does not establish a suitable time step.
- History pattern excitation currently supports joint forces only. Self-weight, element loads and support motion are rejected. Global base-acceleration directions are also supported. No nonlinear dynamic solver is claimed.
- The application still shows existing simultaneous result summaries; complete velocity/acceleration/force history viewers are not included in this editor slice.

## Link API

`cPropLink` and `cLinkObj` now expose typed property/object definitions and checked creation, queries and deletion. The property editor supports Linear, Gap and Hook, including per-DOF nonlinear flags, effective stiffness/damping, nonlinear stiffness/opening, shear distances, mass and weight. It refuses unsupported law families, inconsistent active/fixed/nonlinear flags, incomplete imported constitutive data and invalid numerical input. Two distinct coincident joints are allowed for a zero-length link; repeating the same joint identity is rejected. A missing J denotes a grounded link.

The text representation uses explicitly identified reconstruction extensions for nonlinear flags/completeness. Export warnings identify this boundary. Successful self-roundtrip is not proof that SAP2000 accepts those extension fields. The solver's nonlinear-link restrictions and fixed-DOF penalty limitation remain in force.

## Checked saves

CLI and GUI saves use the new checked export-report APIs. Warnings are printed by the CLI and retained in the GUI Model Explorer after saving. Ambiguous preservation and unknown dimensional data after unit changes cause errors before the destination is overwritten. The Rust file facade exposes `export_warnings` and `last_file_error`, keeping the current model and prior destination intact on such failures. Private SDB output remains explicitly labeled as a reconstruction container.

## Verification

- API-configured history is saved/reopened and compared at 201 samples against the undamped SDOF closed form `u=0.5*(1-cos(2t))`.
- API-configured modal/spectrum analysis matches period pi seconds and displacement 0.5 for an independent one-DOF oscillator.
- Invalid definitions, referenced-function type changes, locked models and unsupported pattern sources preserve input/results.
- API-configured Gap and Hook properties survive checked text roundtrip and reach the analytical displacement +/-0.4 and force +/-30, independent of their effective linear stiffness.
- A file-API regression preserves an existing destination when opaque dimensional source data prevents unit-converted export, and exposes the actual error.
- Headless egui tests render all case controls and perform pointer press/release on Save Dynamic Function. These are application tests, not a native desktop, Windows or full tutorial acceptance pass.
