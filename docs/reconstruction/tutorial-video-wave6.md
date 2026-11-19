# Features of SAP2000 tutorial videos, sixth wave: nonlinear static

Date: 2026-11-18. Scope note for one bounded set of additions and repairs. It
does not change the project status: vendor-runtime acceptance and broad parity
remain unestablished, and no percentage of compatibility is claimed.

The fifth wave ([tutorial-video-wave5.md](tutorial-video-wave5.md)) closed the
defects of the first four. This wave serves a second tutorial video, a
pushover of a reinforced-concrete frame in the shape of its first two modes
(read on 2026-11-17; see "The second video" in
[tutorial-video-progress.md](tutorial-video-progress.md), whose identifiers
D.. and G.. are used below). As shown, that video's model did not run here
for five reasons. This wave removes two of them and a part of a third, and
re-checks the other two, which stay blocked. The video's model still does
not run as shown. It also repairs five defects that an outside review
found on 2026-11-18, three of them in code older than the video work.

No number shown in any video is a reference. Every new test has a closed
form or an attributed vendor verification value.

## What the vendor material supports

The wave was planned against the v19 material (the baseline) and, where v19
is silent, the v24 material that the user added on 2026-11-18. Every citation
below names its version.

- **P-Delta in a nonlinear static case** (v19 Analysis Reference pp. 411,
  415-416, 434): the transverse shape of an element is cubic, the P-Delta
  axial force is the average of the element's end forces, iteration brings
  each increment to equilibrium, and chained cases are "strongly recommended"
  to use one geometric setting. Nothing is said about a hinge inside an
  element. Hence the subset below: hinges at element ends, one setting per
  chain.
- **The stiffness at the end of a nonlinear case** (v19 Analysis Reference
  pp. 133, 402, 411; help topics "Stiffness to Use" and "Hinge Properties"):
  only the geometric nonlinearity that the case considered affects a later
  linear case, and "for all other types of analysis, the hinges are rigid".
  Nothing says what a yielded hinge contributes. Hence: allowed when no hinge
  has yielded, refused otherwise.
- **A case that stops early** (v19 help "Set Load Cases to Run", API
  `GetCaseStatus`): the status words "Not Finished" and "Could Not Start" and
  their codes are the vendor's. Neither v19 nor v24 says which results a case
  that is not finished keeps. Keeping the states it reached is a **project
  choice**.
- **Interacting P-M3 hinges and "Use Yield Moment"** (G12, G45): re-checked
  in v19 and v24, and still without a rule that could be implemented. Flow
  normal to the interaction surface is stated (v19 Analysis Reference p. 155),
  but not the surface between entered points, how it grows with plastic work,
  or the energy-equivalent adjustment of the curve; the default concrete
  surface needs a bar layout and a point rule that the manuals do not give;
  no formula for a computed yield moment is printed for steel or concrete.
  Both stay blocked. The parametric P-M2-M3 hinge of the v24 technical note
  is a different hinge.
- **Hinge verification**: still only Example 1-026, in v19 and in v24.
- The vendor's own pushover video (v15) uses no geometric nonlinearity and
  the monitored displacement. P-Delta with hinges is the second video's
  choice.

## What is supported now

### P-Delta together with frame hinges (G14, a named subset)

- A nonlinear static case with P-Delta runs on the hinge path, event by
  event. After each step the axial forces of all frame elements are brought
  into equilibrium with the state (a fixed point on the axial forces, to the
  case's convergence tolerance and within its iteration limit), and the end
  state of the step is solved again with those forces.
- Multiple saved states are kept, as for any case on the hinge path. A P-Delta
  case of a model without hinges, Accel or Mode loads keeps the old path and
  only its final state.
- The subset, each limit with a refusal that names the object:
  hinges at element ends only (a hinge at an end of the object, with or
  without the hinge overwrite); every case of a chain has the same geometric
  setting; no directly specified P-Delta force on a frame in a P-Delta case;
  large displacements stay refused.
- Curves that harden and lose strength are covered, not only flat ones.
- The case form and the command layer accept displacement control with
  P-Delta; the solver refuses the combinations it does not carry.

### A moment hinge at the start end of an element (D69, repaired)

A moment hinge at end I of its element lost the P-Delta of its own plastic
rotation when the element had a geometric stiffness: a column with 500 kN of
compression kept its load of 30.795 kN out to 0.3 m, where the same column
with the hinge at end J fell as (Mp − P u)/h. The input was accepted. The
jump of such a hinge now leaves out the rigid tilt of the part beyond it. A
hinge inside an element that has a geometric stiffness is refused.

### The stiffness at the end of a nonlinear static case (G48, D70)

- **D70, for all models:** a Modal or linear static case on the stiffness at
  the end of a nonlinear static case got a geometric stiffness from that
  case's axial forces even when the case had no geometric nonlinearity
  (142.07 rad²/s² where the elastic column gives 222.22). It now gets one
  only if the case considered P-Delta. A scan of 338 vendor models found four
  with such a continuation, all with P-Delta, so no vendor example moves; two
  models of that scan failed to import and were not examined.
- **G48:** in a model with hinges such a case is allowed when no hinge has
  yielded at the end of the initial case; the hinges then contribute nothing.
  A hinge on its backbone, and a hinge that yielded and unloaded, are refused
  with different wording, naming the hinge. Buckling after a hinged case
  stays refused.
- A model with areas or solids may now have a Modal case on a nonlinear case
  without P-Delta, because no geometric stiffness is needed for it; with
  P-Delta it is refused as before.
- The case form has a "Stiffness to use" choice for Modal and linear static
  cases (v19 help topic "Stiffness to Use"), and the command layer accepts
  such a definition; before, it could only arrive from a file.

### A case that stops early (G16)

- A case on the hinge path that runs out of total steps or null steps, finds
  no admissible state, or fails the axial-force iteration of a step keeps the
  states it reached and has the status "not finished", with the reason and
  the number of saved states. The last accepted state is added to the saved
  ones.
- A case that needs it (it continues from it, uses its stiffness or its
  modes) has no results and is listed as "could not start", naming the case.
  A combination with such a contributor is unavailable. Other cases keep
  their results, and the analysis as a whole succeeds.
- Steel and concrete design refuse a run that would use such a case. The
  `verify` command counts it as a failure. The API reports the vendor's
  status codes 1 to 4.
- The command line prints one line per case that is not finished and writes
  the status into its output file; its exit status stays 0. The pushover
  window, the hinge results window, the view titles and the case list show
  the status.
- Refusals of what is not implemented (reversal after yield, a curve that is
  refused beyond point E, a load that does not move the monitored
  displacement) stay errors with no results. So does a failure of the
  axial-force iteration at the start of a case, where nothing was reached.

### Windows

- **Generated hinge properties (G47):** the hinge window lists one generated
  property per assigned hinge, named by frame label and hinge number (v19
  Analysis Reference p. 158), with "Show Generated Props" and "Show Hinge
  Details"; a generated row is read-only. *View > Show Hinge Labels* writes
  `name(property)` at each hinge: above a hinge between the ends, and along
  its own member for a hinge at an end, so that hinges of two members at one
  joint stay apart. Nothing is stored.
  The label text, the renumbering when a hinge is removed and the refusal of
  a user property named like a generated one are project choices. The window
  also gained a "Copy Property" button.
- **Display > Show Hinge Results (G57):** a read-only window for one hinge of
  a case with saved states: force and plastic deformation at a step, the
  extremes up to it, state and status, and the path drawn over the entered
  backbone (v19 help "Show Hinge Results"). Where strength is lost the path
  leaves the drawn curve, because the solver limits the negative slope (v19
  Analysis Reference p. 152); the window says so.
- **"Do Not Allow Hinges To Drop Loads" (G56):** accepted for curves that
  never drop, where it changes nothing; refused otherwise, because neither
  help says what the curve is after a drop. The hinge window stores the
  option and says when a case will be refused.

### The conjugate displacement (D68, closed as a difference)

On the hinge path the conjugate displacement measures the step and the
monitored displacement sets the target, as the manual says (v19 Analysis
Reference p. 430); the push follows the direction the monitored displacement
takes in the first step (stated in the v24 Analysis Reference, p. 432; v19 is
silent). A model without hinges ignores the option: its search scales the
load until the monitored displacement reaches the target, which gives the
same states on a path that rises monotonically. Three tests pin this down.
Not tested: a path that does not rise monotonically without hinges.

## Review findings repaired

An outside review of 2026-11-18 gave five findings with models to reproduce
them. All five reproduced and are repaired; the ledger has them as D74 to D78.

| Finding | Was | Is |
| --- | --- | --- |
| D74: a case continued from a yielded hinge when the hinged frame was subdivided | the follow-up case ran with the elastic stiffness | refused, naming the hinge; an elastic subdivided control continues. Found beside it: a nonlinear history continuing from a yielded hinge ran the same way, with one element or two; it is refused too |
| D75: frame forces of a time-history case | zero, while the reactions held the loads | fixed-end forces and span loads at the stored time, in both history paths |
| D76: base reactions of a time-history case after area meshing | fell with the mesh (4.00, 1.00, 0.25 kN for one shell) | the same for every mesh; the base moment too |
| D77: a history load whose pattern is named like a direction | taken for a ground acceleration | the kind is stored and carried through both file formats, the solver, the API and the form; an input that does not state the kind and could be either is refused, naming both readings |
| D78: a combination with a contributor that has no results | published from the others under the same name | no result under any name, listed with its reason; nested combinations, design, the API and the window lists refuse it by name |

A nonlinear history with no initial case still treats hinges as rigid, as
before; that is the manual's statement for analyses other than nonlinear
static and nonlinear direct integration, and an existing test documents it.

## A second review: six more findings repaired

The reviewer rebuilt the repaired tree, confirmed the five findings above on
their eleven models, and held the merge for six further findings, found with
controls of their own. All six reproduced; the ledger has them as D79 to D84.

| Finding | Was | Is |
| --- | --- | --- |
| D83: a P-Delta case with hinges gave results that depend on the units | one cantilever pushed to 4.95 mm: load factor 8.345645 in kN m, 8.018115 in kN mm; the second replayed under load control gives 4.7432 mm | 8.346151615488 in both (closed form 8.346152); replayed under load control: 4.9499995 mm. The iteration on the axial forces is judged against forces only, and the accepted state against equilibrium; the default tolerance is unchanged |
| D84: displacement control refused a motion that comes from the geometric stiffness | "load pattern does not move the monitored displacement" for a preloaded column under compression | the increment of 1e-8 m is reached; the factor replayed under load control gives the same deflection; a straight column is still refused |
| D79: one name for a load case and a load combination | a parent combination published at -20 kN from the case's result; the API's result tables returned the rows of the valid selection only | the name is refused at import (S2K, native), at table editing, before analysis and in design; with an unavailable name selected the API returns the error code or no rows |
| D80: a combination with a time-history contributor | the one stored step as maximum and as minimum | withheld, with a reason naming the history case. The extrema over time are not computed (G69) |
| D81: native export and the dependency check of history loads | legacy U1 and acceleration X refused at export; an acceleration U1 blocked by an unused unsupported pattern named U1 | both export and reopen as an acceleration with the same response; the unused pattern does not block; a history that uses it stays refused |
| D82: the check of a history-case definition was narrower than the solver | the API and the case form refused frame distributed and point loads | one rule for both; solid loads, which the solver dropped, are refused by both |

How D83 was repaired. The fixed point on the element axial forces stopped
when the largest change of an axial force was within the tolerance of a
scale, and the scale held the capacities of the hinges. A moment capacity
is not a force, and in kN mm it is a thousand times its value in kN m, so
the same model stopped after fewer iterations there. The test is now, both
in force units (v19 Analysis Reference p. 435, the tolerance compares the
force error with the force acting on the structure): the largest change of
an axial force against the largest axial force, and the force that the
change leaves unbalanced on an element, the change times the tilt of the
element, against the largest force of the state. A change below the
rounding of an element (256 machine epsilons of EA/L times its
displacement) counts as none.

How D84 was repaired. The stepper needs the rate of each hinge force and of
the monitored displacement with the load factor. With P-Delta that rate has
a part that comes from the change of the axial forces, and it was left out:
the axial forces were held during a step and corrected after it. Before the
first step and before each later one, the rates are now taken from a trial
state a small load step away, with its axial forces converged, when they
differ from the direct ones by more than a tenth; the step is then
corrected until the target (or the grid point of the load factor) holds
within rounding. A case in which no rate moves the monitored displacement
is refused as before.

D80 is a withdrawal, not a computation: a combination of a time-history
case existed before and no longer does, in the solver, in design and in the
API. Combinations of response-spectrum cases are unchanged.

## Behaviour that changed

- A hinged model with P-Delta in a nonlinear static case was refused; within
  the subset it now runs.
- A hinge at end I of an element with a directly specified P-Delta force gives
  different (corrected) results; a hinge inside such an element is refused.
- A Modal or linear case after a nonlinear static case without P-Delta has
  no geometric stiffness any more (D70). Frequencies and deflections of such
  cases change wherever the members carry axial force.
- A hinge case that stopped early returned an error and no results; it now
  returns its states with a status, and the run succeeds.
- A combination with a contributor that was not run existed with partial
  content; it no longer exists. Two existing tests asserted the old
  behaviour on purpose and were changed (`combos.rs`, `review_fixes.rs`).
- A history load named X, U1 and the like, in an input that does not state
  its kind and has a pattern of that name, ran as a ground acceleration; it
  is refused.
- A model with the "do not drop loads" overwrite and only linear cases was
  refused outright; it is analyzed, because the check moved to where hinge
  cases run.
- Since the second review: a model in which a load case and a load
  combination share a name is refused at import; a combination with a
  time-history contributor has no result; a time-history pattern with solid
  loads is refused by the solver (they were dropped); a history load U4 to
  U6 without a stated kind and without a pattern is refused at native
  export; while a selected case or combination is unavailable, every result
  getter of the API returns the error code or no rows, the modal ones
  included; results of P-Delta cases with hinges change in the digits the
  old stopping rule left open (the kN mm cantilever by 4 percent). Three
  existing tests were changed for these: the one-iteration case of
  `wave6_pdelta_hinges.rs`, the static-plus-history combination of
  `frame_auto_mesh_regressions.rs`, and the message of the self-weight
  refusal in `analysis_regressions.rs`.

## Project choices the sources do not settle

- The fixed point on the axial forces and its order (after every accepted
  step); a hinge that deforms backwards after that correction is taken out of
  the active set and counted as a null step.
- Keeping the states of a case that is not finished, and adding the last
  accepted state to them.
- A zero scale factor does not excuse a contributor without results.
- The generated-property rules named above.

## Evidence

Tests added by this wave, by file; units and derivations are in each test.

- `sap-solver/tests/wave6_hinge_end_kink.rs`: the column with a specified
  P-Delta force in both orientations against (Mp − P u)/h; the column in 8
  and 16 objects against the exact beam-column (first-yield load within
  1.4e-9 and 9.1e-11); Example 1-026 unchanged.
- `sap-solver/tests/wave6_pdelta_hinges.rs`: the same column in a P-Delta
  chain; a push whose load changes the axial force, against
  (Mp − 500 u)/(h + 2u), which a frozen axial force would miss; a portal with
  overturning against element equilibrium in the deformed position and the
  sway mechanism; a hardening and dropping curve; the mesh against the exact
  beam-column; Example 1-026 with P-Delta and no axial force unchanged.
- `sap-solver/tests/wave6_end_stiffness.rs` and
  `wave6_end_stiffness_pdelta.rs`: 222.2222 and 142.0676 rad²/s² in closed
  form; a pin-ended column against the beam-column frequency; the vendor's
  Example 1-017 with hinges added at the string ends, equal to the published
  values and to the hinge-free model; the refusals; the subdivided variants.
- `sap-solver/tests/wave6_conjugate_control.rs`, `frame_hinges.rs` (status of
  a case that stops), `sap-app/tests/s4_not_finished.rs`,
  `sap-app/src/gui/hinge_results.rs`, `sap-app/src/gui/hinges.rs`.
- For the review findings: `sap-solver/tests/p1_modal_history_recovery.rs`,
  `history_load_kind.rs`, `history_load_kind_vendor.rs` (opt-in),
  `sap-format/tests/history_load_kind_native.rs`,
  `sap-app/tests/p1_combo_unavailable.rs`.
- For the second review: `sap-solver/tests/pdelta_axial_convergence.rs`
  (D83, D84), `sap-api/tests/mb_combo_selection.rs` and
  `sap-app/tests/mb_combo_names_history.rs` (D79, D80),
  `sap-format/tests/history_load_kind_export.rs` and
  `sap-solver/tests/history_load_kind_export.rs` (D81),
  `sap-api/tests/history_case_validation.rs` (D82).

The stepper was first refactored with no change of results: a dump of 241
solver runs was byte-identical before and after, and identical again for
every run without geometric nonlinearity after P-Delta was added.

## End-to-end: the second video's frame

`sap-app/tests/video2_pushover_workflow.rs` (18 tests) builds the frame from
the program's command functions and runs it: two bays of 8 m, five storeys of
3 m, fixed bases, 15 columns and 10 beams, a moment hinge at both ends of
every member (50 hinges), DEAD and LL as chained nonlinear static cases, a
Ritz modal case on the stiffness at the end of LL, a push by acceleration
and pushes in modes 1 and 2 with displacement control and saved states.
Section sizes, hinge strengths and targets are the test's own. Variant A has
no geometric nonlinearity; variant B has P-Delta in every case of the chain.

What it asserts, in both variants unless said:

1. Gravity: no hinge yields; the reactions sum to the weight. In B the
   result equals the P-Delta path of the same model without hinges.
2. The modal case: in A its periods equal those of a modal case from zero;
   in B the first period is longer and all periods equal those of the
   hinge-free P-Delta model.
3. Every saved state of every push: the base shear equals the load factor
   times the applied pattern, every hinge force is within its curve, and in
   B every column is in equilibrium in its deformed position.
4. The last state of every push: the virtual-work equation of the mechanism
   that has formed, with the mechanism's own motion solved from the hinges
   that rotate, closes to about 1e-12. In A the load factor also equals the
   smallest collapse load of the mechanisms with exactly those hinges.
5. The acceleration push moves the roof toward −X and reaches its target;
   so do the two mode pushes.
6. The rows of the pushover curve equal the saved states, with the hinge
   counts by state and by status.
7. A copy of the push with a limit of five steps is "not finished" and keeps
   the first states of the full run; the cases that need it "could not
   start"; the other cases keep their results.
8. The 50 generated hinge properties and what each was generated from.
9. Saved as S2K and read back, the model is equal and gives equal states.

Two checks hold less tightly than planned and are asserted as measured: the
storey form of the virtual work in B closes to 5e-6 of the plastic work, not
1e-6 (it neglects beam axial terms and column shortening); and the increment
between the last two saved states is not the mechanism's motion in B, where
it still holds elastic changes, so the mechanism's own motion is used.

The test also found two refusals in the command layer that the solver no
longer shared; both were repaired in this wave: displacement control together
with P-Delta, and a Modal or linear static case that names a nonlinear static
case for its stiffness. The case form has the matching choice ("Stiffness to
use") for those two case types.

Substitutions that remain between the video's model and this one: moment
hinges of entered strength in the columns in place of interacting P-M3 hinges
with the default surface (G12); entered yield moments in place of "Use Yield
Moment" (G45); hinges at relative distances 0 and 1 in place of the v26
assignment form with a hinge length (G46); P-Delta also in DEAD and LL,
because a chain has one setting here, whereas the video's gravity cases
have no geometric nonlinearity and only its pushes have P-Delta (read off
its case forms after the wave was built; such a mixed chain is refused);
flat-topped hinge curves in place of the
video's hardening and dropping curve (such curves are tested on a column,
not on this frame); no hysteresis type (G13); exact events; the test's own
targets; no reinforcement data; no table export (G49). The frame was not
built by hand in the windows.

Each variant B test takes about 5 s in a debug build, the whole file 10 s.

## Not done

- Interacting P-M3 hinges and the default interaction surface (G12), "Use
  Yield Moment" (G45), the v25 hinge assignment form (G46): blocked for
  evidence. The second video's model therefore still does not run as shown;
  it runs with moment hinges of entered strength in the columns.
- Hinge reversal after yield (G13): a later wave, with hinges in nonlinear
  time history.
- Of G14: chains with mixed geometric settings, which is what the second
  video uses (gravity without P-Delta, the push with it): the manual does
  not say how a case that switches P-Delta on takes over the earlier state.
  Also hinges with links, shells, solids, cables, end releases, end offsets,
  insertion points and non-prismatic sections; a hinge inside an element
  under P-Delta; large displacements. Not tested: P, V2, V3 and T hinges in
  a P-Delta case. In the step correction of a P-Delta case, the branch that
  drops events tied with the terminating one never ran in any test.
- Of G48: a yielded hinge at the end of the initial case, on its backbone or
  unloaded; buckling after a hinged case.
- Of G16: partial results of the nonlinear paths without hinges (Wen links,
  cables, P-Delta without hinges).
- Solution control (G50): nothing implemented. Events are exact; the vendor's
  event lumping, its two iteration limits and its line search have no
  counterpart, and the step counts differ. The v24 "solution scheme" is
  recorded in the tracker.
- D71 and D72 (how a specified P-Delta force and a buckling case with an
  initial case are treated): read in the code, not run, not changed.
- Of D77: static and Ritz case loads have no stored kind; a pattern named
  like a direction is refused there.
- Of D78: places that clear results directly, other than the API's unlock
  and edit paths, leave the list of reasons behind.
- Of D80: the maximum and minimum over time of the quantities of a
  time-history case (G69); until then its combinations are withheld.
- Of D79: the native reader's refusal of a shared name has no test. Of D81:
  three readers of the history load kind outside export and the dependency
  check still go by the stored tag. Of D84: no test has a case in which the
  coupled rates change the order of hinge events. The windows were not
  opened for D79, D80 and D82.
- Review coverage: the reviewer read the source changes of the first solver
  stage (D69, G56) and of the pieces written in review (the guard of D74 and
  its history sibling, the join of D75 with D77, the command-layer and form
  changes, the hinge labels). The stepper refactoring was accepted on its
  byte-identical dumps; P-Delta with hinges, the status of a case that
  stops, and the repairs of D77 and D78 were accepted on their tests and the
  gates, without a read of their diffs.
- Windows: the changes for D77 and D78 were tested headless only. The
  "Stiffness to use" row was looked at on the private display for a linear
  static case (the list with the nonlinear case, the choice and its note);
  saving that case, the same row of a modal case and displacement control
  with P-Delta in the case form were not. On the
  private display the agents looked at the hinge list, the hinge results
  window, and the run message, pushover window and hinge results window of a
  case that is not finished; the "could not start" tag was not seen there.
  The hinge labels were looked at three times: labels of two members at one
  joint covered each other, then two labels on one short member met in the
  middle, and after the second repair all five labels of the test model are
  apart in the plan view. In a foreshortened 3-D view two labels of one
  member can still touch.
- At a convergence tolerance of 1e-12 the axial-force iteration stops at a
  rounding floor on three heavy portal variants (relative gaps of 4e-12 to
  1.5e-11); with the default tolerance they converge.
