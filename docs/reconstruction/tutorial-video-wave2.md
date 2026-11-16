# Features of a SAP2000 tutorial video, second wave: frame hinges and static pushover

Date: 2026-11-16. Scope note for one bounded set of additions. It does not
change the project status: vendor-runtime acceptance and broad parity remain
unestablished.

The first wave ([tutorial-video-wave1.md](tutorial-video-wave1.md)) left the
second model of the video out: the static pushover of a four-storey concrete
plane frame. This wave adds what that model needs. As before, the video only
lists which commands and options a user meets; no number in it is a reference.

What the vendor material supports was checked first, and it decides the scope:

- The corpus has **one** hinge verification problem, Example 1-026 (a
  cantilever with a moment hinge and a shear hinge, two models). It covers
  user-defined **uncoupled** hinges only.
- There is no verification problem for coupled P-M2-M3 hinges or their
  subsets, for fiber hinges or for automatic hinges, and the FEMA 356 /
  ASCE 41 tables that automatic hinges are generated from are not in the
  corpus. These kinds are **refused**, not approximated. The video's column
  hinge is an interacting P-M2-M3 hinge, so the video's frame can be built
  here only with user-defined M3 hinges in its place.
- The v19 Analysis Reference does not say how a hinge that loses strength is
  traversed. The vendor API names three methods (`SetHingeUnloading` 1 to 3).
  What is implemented is a reconstruction of the first, "unload entire
  structure", checked against the published points of Example 1-026 only.

## What is supported now

### Hinge properties, assignments and overwrites

- A hinge property is the A-B-C-D-E curve of one frame degree of freedom
  (P, V2, V3, T, M2 or M3), symmetric or with its own negative branch, with
  force and deformation scale factors for each direction, IO / LS / CP
  acceptance values and the choice of what happens beyond point E (drops to
  zero, or is extrapolated). The hinge is rigid up to B; the deformation
  entered at B is subtracted from C, D and E (Analysis Reference, "Plastic
  Deformation Curve").
- Hinges are assigned to frame objects at relative distances measured from
  end I. A hinge is a jump inside the elastic element: an interior hinge needs
  no joint, so the lumped mass and the elastic deformation of the member are
  unchanged.
- The frame hinge overwrite subdivides the object so that each hinge sits at
  the middle of an element of the given relative length (default 0.02; at an
  object end half of it), as in the vendor form.

### The nonlinear static solution with hinges

- Every response is linear in the load factor of the case and in the plastic
  jumps, so the case is solved in the space of the hinges and followed event
  by event: a hinge yields, reaches the end of a segment of its curve, or
  returns to its curve after unloading. Between events nothing is iterated.
  Events are exact; the vendor lumps events within one percent (project
  choice).
- **Strength loss.** A segment that drops more steeply than 10 % of the
  elastic stiffness `EI/L` of the element containing the hinge is given that
  slope (Analysis Reference, "Strength Loss"). Example 1-026 pins the measure
  as `EI/L` of the element: its models A and B differ only by the hinge
  overwrite and give 1.7612 in and 1.3701 in. Later points of the curve are
  moved by the extra deformation of the limited segment (project choice; the
  example never reaches them). The rule is evidenced for bending only: a
  dropping segment of a P, V2, V3 or T hinge is refused.
- **Snap-back.** Under displacement control a step that moves the monitored
  displacement backward is accepted when a hinge that is losing strength
  drives it. Model B of Example 1-026 does this (1.3847 in to 1.3701 in).
- Hinges unload parallel to A-B and reload to their curve. A hinge that has
  yielded and then unloads through zero force is **refused** (no hysteresis).
- A case continues from the hinge state of a previous nonlinear static case.
- Two hinges in series at one joint with equal strength: the one assigned
  first deforms (project choice; the load-displacement path does not depend
  on it).
- A case that cannot reach its target returns an error and keeps **no partial
  results**, unlike SAP2000. The step limits of the case (maximum total steps
  200, maximum null steps 50 by default) are honoured.

### Saved states

- A nonlinear static case keeps its final state, or multiple states with a
  minimum and maximum number of saved steps and the "positive increments
  only" option, selected as the manual describes ("Saving Multiple Steps";
  both of its worked examples are unit tests).
- Each saved state holds joint displacements, reactions, frame forces and,
  for every hinge, its force, plastic deformation, state (A-B, B-C, C-D, D-E,
  beyond E) and status (to IO, to LS, to CP, beyond CP).
- Multiple states are kept on the hinge path only: a model of frames with no
  other nonlinearity. A case that asks for them in a model with nonlinear
  links, cables, tension/compression limits, P-Delta, or area, solid or link
  objects is solved as before and keeps its final state.

### Accel and Mode loads

- A nonlinear static case may apply an **acceleration load** UX, UY or UZ:
  joint forces `−sf·M·r`, with `M` the lumped joint masses of the analysis
  (mass source included). A scale factor of −1 on UX pushes toward +X.
- It may apply a **modal load**: joint forces `sf·ω²·M·φ` of a mode of the
  Modal case it names (Analysis Reference, "Nonlinear Static Analysis —
  Loading"), with `φᵀMφ = 1`. The Modal case is run first. The push has the
  sign of the stored mode shape, which may be the mirror of the vendor's.
- Refused: rotational acceleration, a load pattern named like an acceleration
  load, a modal load mixed with other loads in one case, and either load in a
  model with area, solid or link objects. Linear static and buckling cases
  still refuse acceleration loads.

### Pushover curve and display

- `sap_api::hinges::pushover_curve` returns, per saved state, the monitored
  displacement, the base shear and the hinge counts by state and by status
  (the columns of the vendor table). The base shear is minus the component of
  the total support reaction along the monitored direction, so it includes
  the reaction to the loads of earlier cases of the chain.
- `get_hinge_assigns` has the shape of the vendor call; `set_hinge_unloading`
  accepts option 1 only.
- GUI: Define > Hinge Properties, Assign > Frame Hinges and Frame Hinge
  Overwrites, the nonlinear static case editor (initial case, load
  application, results saved, step limits, load rows of type pattern, Accel
  or Mode), Display > Show Static Pushover Curve (table and plot), a slider
  through the saved states in the 3D view, and a dot per yielded hinge on the
  deformed shape with the legend of the manual (B to IO, IO to LS, LS to CP,
  CP to C, C to D, D to E, >E). The manual does not give the colours; they
  are ours. All of this is tested through command functions and headless
  renders, **not by hand**.

### Native import of Example 1-026

- The hinge records of `example_1_026a.sdb` and `example_1_026b.sdb` are
  decoded, and both models analyze and reproduce the published values.
- The importer accepts only what those two files show: M3 and V2
  deformation-controlled hinges at relative distance 0, symmetric curves,
  unit scale factors, the default overwrite. Any other value keeps the
  frame's hinges out and records an unmodeled entry, so its nonlinear cases
  stay refused. The setting for behaviour beyond point E has one observed
  code whose meaning is not established; an imported hinge that reaches E is
  refused.
- Saved-state options and step limits are read for models with imported
  hinges only. Other vendor models that request multiple states are solved
  as before and keep their final state.
- A nonlinear static load assignment of a type the importer does not know is
  now recorded as unmodeled instead of being dropped.

### File formats

- S2K keeps hinge properties, assignments, overwrites, the nonlinear static
  parameters and the Accel and Mode loads in reconstruction extension tables
  and columns (`RUST HINGE PROPERTIES`, `... - CURVE`, `RUST FRAME HINGE
  ASSIGNMENTS`, `RUST FRAME HINGE OVERWRITES`, `Rust...` columns of the case
  table, `LoadType` of the static load assignments), written only when
  present. The genuine SAP2000 table names are known from the vendor report
  schema but their columns are not; a file that carries genuine hinge tables
  keeps them verbatim and its nonlinear cases are refused. The `Accel` and
  `Mode` spellings of `LoadType` are unverified.
- Native `.sdb` export refuses hinge properties, hinge assignments and
  overwrites, non-default nonlinear static parameters and modal loads. An
  acceleration load of a nonlinear static case spelled U1, U2 or U3 is
  written as before (the writer already had that load type); spelled UX, UY
  or UZ it is refused.

## What is refused

Each of these has a negative test.

- Hinge kinds: coupled (P-M2-M3, P-M2, P-M3, M2-M3), fiber, parametric,
  automatic, force controlled; hinge unloading methods 2 and 3; the "do not
  allow hinges to drop loads" overwrite.
- A model with hinges that also has area, solid or link objects, cables,
  tension/compression limits or geometric nonlinearity in the case.
- A frame with hinges that also has end releases or partial fixity, end
  length offsets, an insertion point with stiffness transformation, a
  non-prismatic section, or a point load at a hinge station. The GUI assigns
  such a hinge with a note; the analysis refuses it.
- Hinges in a nonlinear direct-integration history; a linear, modal or
  buckling case that continues from a nonlinear static case of a hinged
  model. Linear cases from zero and modal-superposition history ignore
  hinges, as the vendor does.
- Temperature and ground-displacement loads in a case on the hinge path.
- Two hinges of the same degree of freedom at one station; a hinge exactly on
  a boundary of an automatically subdivided frame; overlapping overwrite
  elements; a hinge overwrite together with automatic subdivision.
- Dividing a frame object that carries hinges.

## Differences from SAP2000 to keep in mind

- The traversal of strength loss and the saved-step selection are
  reconstructions checked on Example 1-026 and on the manual's two worked
  examples. With several hinges losing strength at once the path may differ
  from the vendor's.
- Events are not lumped, so saved states can fall at slightly different
  displacements than the vendor's.
- A hinge exactly at a point shared by two segments reports the later
  segment (a hinge exactly at C reports "C to D").
- Without acceptance criteria a yielded hinge has status "B to IO".
- The frame station deflections `local_disp` have no kink at an interior
  hinge; joint displacements, reactions and station forces are exact.
- A diaphragm is a penalty constraint of 10⁶ times the largest diagonal
  stiffness. With ordinary sections a collapse load through a diaphragm
  agrees to about 10⁻⁶; with a nearly rigid axial area it loses several
  digits.

## Evidence

| Check | Reference | Tolerance |
| --- | --- | --- |
| Example 1-026 model A, built in code: P, Uz, Ry at the three published points and the end | hand calculation of the problem (unit load method) and its printed SAP2000 values 1.7612 in, −0.0567 rad | 1e-9 closed form; 5e-5 printed; two rows of `structurax verify` |
| Example 1-026 model B (hinge overwrite): snap-back to the published point | printed SAP2000 values 1.3701 in, −0.0404 rad | 5e-5; two rows of `structurax verify` |
| Example 1-026 a and b imported from the vendor files (opt-in) | the same published values, found by P = 60, 80, 20 kips | 5e-5 |
| Fixed portal, sway: first yield, second yield, collapse | `2Mp(6k+1)/(h(3k+1))`, `4Mp/h` and the deflections at both | 1e-6; 1e-9 on the collapse load |
| Fixed beam with end and interior hinges, uniform load | `12Mp/L²`, `16Mp/L²`, end rotation, midspan deflection | 1e-9 |
| Continuation, unloading and reload, asymmetric curve, scale factors, point B deformation, beyond E | closed forms of the cantilever | 1e-9 |
| Four-storey frame, 56 hinges, joint loads, after three gravity cases | collapse load by the uniqueness theorem: 4080/1350 for the mechanism the results report (bases, beam ends of two floors, column tops of the third storey), below the 3.04 of the beam-sway mechanism; yield bound at every hinge; work balance | 1e-9 |
| Minor-axis hinges | equal the major-axis hinges of the member turned 90° | 1e-9 |
| Saved-step selection | the two worked examples of the manual | exact |
| Acceleration load on a two-mass chain; with self mass; continued | `u = −sf·Σm/k`; base shear `sf·Σm` | 1e-9 |
| Modal load on a two-degree-of-freedom chain | `ω² = (3 ∓ √5)k/2m`; response equals `sf·φ` | 1e-7 |
| Portal and cantilever pushed by an acceleration load | `4Mp/(h·Σm)`; work balance and yield bound | 1e-6 |
| Pushover curve of the portal and of Example 1-026; hinge counts; status bands | the closed forms above; plastic rotations against IO, LS, CP | 1e-6; 5e-5 |
| Hinge markers per saved state | classes derived from the closed-form plastic rotations | exact |
| S2K round trip of every new field; unreadable rows; native refusals | equality; errors | exact |

The collapse loads above rest on the uniqueness theorem of plastic analysis:
a state in equilibrium that nowhere exceeds the yield strength and forms a
mechanism is the collapse state. The tests therefore check the yield bound at
every hinge and the virtual-work load of the mechanism the results report.
An assumed mechanism is only an upper bound: the first reference written for
the four-storey frame (the beam-sway mechanism, 3.04) was wrong, and the
solver's lower value was confirmed by hand.

## End-to-end: the video's frame

`crates/sap-app/tests/video_pushover_workflow.rs` builds the four-storey
frame of the video (column lines at 0, 4, 10, 14 m; four storeys of 3 m;
columns 0.30 × 0.30, beams 0.30 × 0.40; dead, superimposed dead and live load;
mass source dead 1, superimposed dead 1, live 0.25; Ritz modal case), with
user-defined M3 hinges at every member end in place of the video's P-M2-M3
column hinges and its unstated hinge data (beams 120 / 160 kN m, columns
300 kN m). Three gravity cases are chained, then one push by acceleration UX
and one by mode 1, both to 1.2 m at the roof.

Checked there, none of it against the video's on-screen base shear:

- the last gravity case equals the linear solution and carries the weight
  computed from the geometry (1e-9);
- for each push, the collapse load by the uniqueness theorem: the yield bound
  at every hinge of every saved state, equilibrium of the reactions with the
  load vector the test computes itself (joint masses from the mass source;
  `ω²Mφ` from the modal results), the work balance on the plateau, and
  equality with the smallest virtual-work load of a family of candidate
  mechanisms with exactly that mechanism's hinges moving (1e-9);
- the first yield from two linear solves (1e-9);
- the pushover curve rows against the saved states;
- the model saved as S2K and reopened: equal definitions and equal saved
  states (1e-12);
- a variant with one diaphragm per floor (1e-5, the accuracy of the penalty
  constraint) and one with the beam hinges at 0.1 and 0.9 of the span.

| Push | Mechanism the results report | Load factor | Base shear |
| --- | --- | --- | --- |
| Acceleration UX | lower three storeys sway, column hinges at the top of the third storey (20 hinges) | 4.846072 | 612.83 kN |
| Mode 1 | beam sway of all four floors (28 hinges) | 0.743255 | 519.63 kN |

These numbers belong to the hinge strengths chosen for the test. They say
nothing about the video's result, whose hinge data are not stated.

## Not done

- Coupled P-M2-M3, fiber and automatic hinges; the reinforcement layout
  "to be checked" that automatic concrete hinges need.
- Hinge unloading by local redistribution or secant restart; hysteresis.
- Hinges together with P-Delta, links, shells or cables; hinges in nonlinear
  time history.
- Capacity-spectrum (ATC-40 / FEMA 440) forms of the pushover curve.
- Partial results of a case that stops early.
- Typed API calls for defining hinge properties and assigning hinges (the
  commands exist in the GUI; the API has the case definitions and the
  results).
- Native export of hinge data.
- Everything listed as not done in the first wave other than pushover.
