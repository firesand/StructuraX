# Native transactional editing — wave 2

Implemented 2026-10-15 in `sap-model/src/editing.rs`, with a Rust `cEdit` facade and selected assignment setters in `sap-api`. This is a native Rust API slice, not a Windows COM implementation or evidence of SAP2000 v19 signature/behavior parity. The complete tutorial and a vendor differential run remain unaccepted.

## Shared model operations

- `SapModel::move_selected(delta: [f64; 3]) -> Result<EditSummary, EditError>` moves selected points and endpoint/vertex points referenced by selected frames/areas once. Shared joints retain identity, so connected unselected members follow those joints; their assignments are unchanged. Distinct joints may not be moved onto one another. Move does not merge joints.
- `SapModel::replicate_selected(delta: [f64; 3], copies: usize) -> Result<EditSummary, EditError>` creates copies at delta through copies times delta. Each copy has one shared point mapping for selected frames/areas. All source object and point assignments, including restraint, mass, loads, releases, area mesh settings and modifiers, are cloned. Source objects/selection survive and newly created objects are selected. Exact coincident coordinates reuse an existing point only when all point assignments agree; incompatible points and duplicate frames/areas cause atomic rejection. No fuzzy coordinate tolerance or load-adding merge is imposed. Named constraint assignments are preserved verbatim; copied points therefore remain in the named constraint shared by the originals, rather than creating new constraint definitions.
- `SapModel::divide_selected_frames(segments: usize) -> Result<EditSummary, EditError>` divides selected prismatic frame geometry into equal segments. It retains the source frame's name for the first segment and produces collision-free numeric names for the others. Original end joints retain all assignments. New intermediate joints have no physical assignments, and exact existing intermediate joints are reused with their original assignments. Outer end releases and partial-fixity springs survive only at the original ends; internal ends are continuous. The first child retains the original identity, so `frames_removed` is empty for this operation.

Every operation stages a clone and commits only after complete validation. Locked models, empty supported selection, nonfinite values, arithmetic overflow, collapsed segments and invalid counts are rejected without changing model data or existing results. On successful edits, analysis and steel/concrete design results are cleared. Move/replicate reject selected links and solids. Copy counts are bounded at 10,000; generated object counts at one million. Selected source objects with missing point references or invalid frame/area topology are rejected. Native `cEdit` forwards typed `Result` values so GUI callers can display the actual reason.

## Frame load division and explicit limits

Distributed force or moment intensities are clipped to each child and interpolated at the clipped boundaries. Child distances are renormalized to its length; intensity values are not scaled. A frame point load appears exactly once, on the right child at an exact interior split and on the final child at the original J endpoint. Uniform unpatterned temperature and gravity assignments are carried to every child. Property modifiers, local angle and prescribed axial force remain assigned to each segment.

Division rejects nonprismatic sections, cables, tension/compression limits, nonzero end length offsets, nonzero joint offsets, noncentroid cardinal points and patterned temperature. Copying end length offsets to every child or cloning endpoint releases at interior joints would alter the structural model; this slice reports those unsupported cases instead. Offset division still needs a validated flexible-span and load mapping formulation. Point pattern values and constraints at newly inserted joints are not interpolated or synthesized.

## Assignment API additions

- `cAreaObj::set_modifiers` and `cPropArea::set_modifiers` validate ten finite nonnegative values, permit zero stiffness modifiers, check the target/lock, then invalidate results after success.
- `cAreaObj::set_load_surface_pressure(name, load_pat, face, val, pattern_name, replace)` accepts uniform top/bottom pressure (`-2`/`-1`) for a defined load pattern. Replacement affects only the same pattern and face. Edge faces and patterned pressure are rejected because their solver formulation is not implemented. Positive pressure is inward: top pressure acts along negative local 3, bottom pressure along positive local 3.
- Existing `cFrameObj::set_releases` now validates the target before mutation/result clearing, checks finite nonnegative spring stiffness, and rejects partial-fixity spring values on unreleased DOFs. Locked/invalid calls leave results and assignments unchanged.

The subsequent bounded legacy pass covers all `cPointObj` and `cFrameObj` mutations: target/lock validation precedes mutation; incoming loads must be finite and use defined patterns/sections/coordinate systems, valid load type/direction and relative distances. Spring, mass and modifiers must be nonnegative; added joint mass is checked for overflow. Explicit duplicate names, zero-length/nonfinite frames, invalid offset lengths and invalid frame geometry references are rejected. `add_by_coord` validates both ends before creating any point and reuses exact coordinate matches, avoiding a fixed unit-independent epsilon that previously collapsed short members. Multiple exact coincident point identities are ambiguous and rejected before creating any object; `add_by_point` remains the explicit identity-selection route. Successful edits alone invalidate results. Existing public signatures remain unchanged.

`cPropMaterial::set_osteel` now assigns represented nominal Fy/Fu after validating positive ordered strengths and the steel material type. Separately supplied expected strengths are not represented in `Material`; values that differ from nominal and are nonzero are rejected instead of ignored. Zero or equal nominal expected strengths carry no extra data. `set_oconcrete` validates positive Fc and the concrete material type, and rejects lightweight concrete because no lightweight formulation/parameter is represented. These rejected calls preserve material state and results. The real managed v19 metadata signatures are broader than these simplified facade methods (see `rea-api-evidence-wave2.json`); no COM signature parity is claimed.

`cPropMaterial::set_material` now obtains default physical values in the current model units from unit-scaled model initialization. It accepts steel, normal-weight concrete and no-design isotropic materials, and explicitly rejects aluminum, cold-formed steel, rebar and tendon creation because dedicated defaults/constitutive setters are missing. `set_mp_isotropic` and `set_weight_and_mass` now validate finite physical input, target and lock before mutation; require positive E/G, Poisson ratio strictly between -1 and 0.5, and nonnegative densities. Zero supplied G requests the finite computed isotropic value. Failed calls preserve outputs; accepted calls invalidate them.

The atomic setter improvements do not establish an audit of every API facade. Property creation, area creation/uniform-load and other older facade methods still require separate contract/validation review.

## Validation

`cargo test -p sap-model -p sap-api` passes 27 tests at this checkpoint: 12 model tests, five existing API unit tests and ten new API integration tests. Eighteen tests were added for this editing slice. Meaningful checks include:

- shared joint translation, unchanged unrelated points/member assignments and successful result invalidation;
- frame/area replication with shared endpoint mapping and copied restraint/mass/load/releases/pressure/mesh/modifiers;
- reuse of equal coincident joints, incompatible assignment rejection and unchanged state/results on failed edits;
- partial trapezoid integrated force and first moment conservation, one-time boundary point loads, correct outer releases/springs and intermediate joint reuse;
- atomic multi-frame division when a later member has unsupported offsets, and replication overflow after a valid staged copy;
- all six cantilever tip displacements and base reactions before/after four-way division with a partial trapezoid and midpoint point load, within relative tolerance `1e-9` (Euler–Bernoulli setup);
- API pressure sign and area integration: a 6-by-4 panel under uniform magnitude 2 has vertical reaction resultant +48 for top face and -48 for bottom face;
- invalid/locked assignment setters preserve model/results; successful setters clear results and replacement preserves the other pressure face;
- legacy point/frame invalid-input atomicity, duplicate-name protection, no orphan points after failed frame creation, short-member distinct endpoint preservation, joint-mass overflow rejection, explicit material expected-strength/lightweight rejection, physical SI material defaults, atomic isotropic/density validation, unsupported material-family rejection and coincident-joint ambiguity rejection.

`cargo clippy -p sap-model --all-targets -- -D warnings` passes. `cargo clippy -p sap-api --all-targets --no-deps -- -D warnings` also passes. Initial ordinary package Clippy was blocked by formatting in the concurrently edited solver `link.rs`, not an editing test failure.

These are automated native checks, not human GUI acceptance, vendor tutorial numerical acceptance, nonlinear equivalence, offset equivalence or Windows runtime evidence.

## Independent solver review follow-up

A read-only follow-up review reproduced two solver defects and sent them to the solver owner, who fixed them in solver-owned files. A ramp-driven two-joint axial viscous link (I fixed, J mass 2, stiffness 8, damping 0.8, applied force 3t, t=1) originally reported support reaction -1.4951148335682718 while its simultaneous link force was +1.8725225141009403. The omitted support damper contribution was 0.3774076805326685. Rerunning the independent executable after the fix returned reaction -1.8725225141009403 and link force +1.8725225141009403, balancing exactly.

Tapered self-weight station recovery used midpoint weight for the full-span first moment while using quadrature for partial moments. For area varying from 1 to 3, unit weight 1 and length 12, direct public recovery with specified zero end resultants returned J bending moment 24 rather than the specified 0. The independent rerun after the unified gravity-integral correction returned exactly 0. Solver-owned regressions additionally use a physical tapered cantilever and check exact station shear/moment, integrated weight and first moment.

The thermal and projected moment regressions were inspected for independent expectations: they use closed-form eigenstrain/curvature and restraint equations in both bending planes, plus explicit global load vectors/resultants and reversed endpoint checks. No further concrete defect was found in that focused review. This does not cover every nonlinear or response-spectrum path.
