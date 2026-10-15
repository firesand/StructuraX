# Application and API review repairs

This follow-up repairs confirmed wave-4 review defects and narrows the public
contracts to implemented behavior. The historical review remains available under
`docs/review/structurax-latest/`.

Solid creation now uses the solver's geometry validator before inserting an
object or invalidating results. Invalid/nonfinite, inverted and degenerate
center/integration-point Jacobians reject. Collapsed mappings remain subject to
the same geometry contract as imported solver elements; this is not a claim of
exhaustive mesh-quality validation. Temperature accumulation checks the prospective
sum for finiteness before mutation, preserving the assignment and all results on
arithmetic overflow.

The typed case API and native analysis editor now accept nonlinear history from
rest, with validated iteration controls, sample functions and time-step settings.
Both NonlinearHistory and NonlinearDynamic are accepted by the typed API; the GUI
adds the nonlinear-history choice and per-step iteration/tolerance controls.
Element loads, self-weight, prescribed support motion, geometric nonlinearity and
initial-state continuation remain unavailable through this editor. The validator
now includes solid surface/temperature loads in its unsupported-history-pattern
check. Solver applicability rules impose further element/integrator restrictions.

An API-created two-node Hook oscillator is checked at 501 ramp-load samples
against `u(t)=0.1*(t-sin(10t)/10)`, and its fixed-support reaction must balance the
constitutive link force independently of effective `ke`. Negative tests establish
atomic rejection of invalid definitions and solid changes.

Native-export APIs continue using the format crate's checked preflight. Errors
remain available in `last_file_error`; successful saves also retain an experimental
export warning. The GUI retains the same warning. All native-export entry points
now share exclusive destination creation: existing files (including symlinks)
are refused. The writer synchronizes the file, reads it back, compares exact
serialized bytes and inspects the actual stream before success; an I/O or
verification failure removes the new incomplete output. The CLI also reopens
and inspects the destination for its report. No existing file is truncated.
Tests inject truncated/corrupt writes and I/O failures, check repeat saves through
the API and CLI, and verify handwritten S2K conversion with unknown-column refusal.
This policy applies to native exports; ordinary S2K/private saves are unchanged.
Vendor application acceptance remains unverified.

The Rust facade is not a COM implementation. Existing actual solid/cable methods
are `cPropSolid::set_prop(name, material, a, b, c, incompatible)`, property `count`,
`cSolidObj::add_by_point`, `set_load_surface_pressure`, `set_load_temperature`,
object `count`, frame `set_tc_limits`, `set_cable_data`, and
`cAnalysisResults::solid_stress(name)`. Wave-4 notes previously listed several
nonexistent getters, extra parameters, and coordinate-creation methods; those
claims have been corrected. General continuation remains deliberately unavailable
through the typed editor until the result/state contract can preserve it.
