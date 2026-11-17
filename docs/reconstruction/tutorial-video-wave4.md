# Features of a SAP2000 tutorial video, fourth wave: result display for shells, displacements, time histories and modes

Date: 2026-11-17. Scope note for one bounded set of additions. It does not
change the project status: vendor-runtime acceptance and broad parity remain
unestablished.

The first three waves ([tutorial-video-wave1.md](tutorial-video-wave1.md),
[tutorial-video-wave2.md](tutorial-video-wave2.md),
[tutorial-video-wave3.md](tutorial-video-wave3.md)) added the analysis
features of the video's four models and the drawing commands that build them.
What a user following the video still could not do was look at the results the
way the presenter does in its third and fourth parts: choose a shell stress
component and face and read it in MPa, colour the deformed shape by a
displacement component, read a joint's displacement with a right-click, plot
several response functions of a time history including the absolute
acceleration, and read the vendor's two modal participation tables. This wave
adds that, plus the repair of defects found in the same code. As before, the
video only lists which commands a user meets; no number in it is a reference.

What was checked first:

- Every row of the video's feature list was compared again with the code as it
  stood after the third wave. The rows still open or partial that have a
  reference are the ones of this wave.
- The corpus was searched by content (every PDF, both help files, and the
  strings of 159 program and data files) for the 2016 editions the video uses.
  It holds **nothing for AISC 360-16 or ASCE 7-16**, so the video's steel
  design code and seismic code stay at the 2010 editions.
- Command semantics are those of the vendor help topics (Show Deformed Shape,
  Show Forces/Stresses Shells, Von Mises Stress, Plot Functions, Joint Plot
  Functions, Time History Load Functions) and of the CSI Analysis Reference
  Manual (shell stresses pp. 210-214, joint output axes p. 24, acceleration
  loads p. 339, combinations p. 352, modal output pp. 376-381). Where a source
  is silent the choice made here is listed under "Project choices".

## What is supported now

### Shell forces and stresses (*Display > Show Forces/Stresses > Shells...*)

- **Resultants** F11, F22, F12, FMAX, FMIN, FVM, M11, M22, M12, MMAX, MMIN,
  V13, V23, VMAX and **stresses** S11, S22, S12, SMAX, SMIN, SVM, S13, S23,
  SMAXV, the list of the vendor form.
- **Output face** for the in-plane stresses: Top, Bottom, Maximum, Minimum,
  Absolute Maximum (the value of larger magnitude, with its sign), or the
  Visible Face, resolved per area from its local 3 axis and the direction the
  pane looks from.
- Everything is derived when it is displayed from the stored resultants and
  top-face stresses and the section's thicknesses; nothing new is stored.
  Bottom face `F/tm + 6M/tb²` (Analysis Reference Eqns. 2), principal values
  by Mohr's circle, von Mises as the help topic gives it, `S13 = V13/tb`,
  `S23 = V23/tb`.
- **Display units** of the form, independent of the model's units: the model
  is not converted and stays locked with its results. Stresses in N/mm² are
  the video's MPa. The contour range is typed in display units; 0 and 0 mean
  automatic.
- A **legend** with ten bands, eleven tick values in display units and a title
  with the component, face and unit.
- The five contours the *Display* menu has always offered remain as shortcuts.

### Deformed shape (*Display > Show Deformed Shape...*)

- Case or combination, mode or step, scale, and **displacement contours on
  area objects** by Ux, Uy, Uz (global) or along the area's local 1, 2 or 3
  axis, with a range and display units. The colour varies over each area
  between its corner joints.
- **Right-click on a joint** of the deformed shape opens a floating *Joint
  Displacements* read-out: U1, U2, U3, R1, R2, R3 in the joint's local axes
  (Analysis Reference p. 24), translations in the form's display units.
- **Animation through the steps** of a time history or of the saved states of
  a nonlinear static case. Mode shapes keep their oscillating animation.

### Plot functions (time-history window)

- Quantities: displacement, relative acceleration, and **absolute
  acceleration** for modal time-history cases. Velocity and reaction are
  listed as in the vendor form and refused, because they are not recorded.
- **Named plot functions** (joint, quantity, component) and **load-function
  traces** (`scale factor × f(t)` of a load entry of the case); several
  vertical functions in one plot; the horizontal axis is time or another
  function of the same case; a time range; per trace the maximum and minimum
  with their times; display units.
- **Named sets** of functions with their range and units, kept for the session
  and reset when another model is loaded.
- `sap_solver::ground_acceleration_trace(model, case)` returns the global
  ground acceleration at every output step of a modal time-history case.

### Modal tables

- *Modal Participating Mass Ratios* with the sixteen fields of the vendor
  table: OutputCase, StepType, StepNum, Period, UX, UY, UZ, SumUX, SumUY,
  SumUZ, RX, RY, RZ, SumRX, SumRY, SumRZ.
- *Modal Load Participation Ratios*: one row per item with its static and
  dynamic percentage. Items are the case's starting loads, then each of the
  six unit accelerations UX to RZ that acts on mass (Analysis Reference
  p. 378). For a Ritz case one line per starting load says whether its target
  was reached and with how many vectors.
- The earlier table of participation mode by mode remains as a third tab.

## Defects repaired, and reported values that changed

Results from before this wave differ in the cases below.

| What changed | Before | Now | Reference |
| --- | --- | --- | --- |
| UX, UY, UZ participation factors and mass ratios of a model with joint local axes | joint-local components were summed as if global: a mass on a joint turned 30° reported UX 0.75 | global directions: 1.0 | Analysis Reference pp. 376-378; closed form at five angles |
| Response-spectrum results of such a model | built on those factors | follow the corrected factors | single mass, `Sa/ω²` |
| RX, RY, RZ factors and ratios, **for every model** | only the rotational inertia entered at joints was counted | about the centre of mass, with the lever arms of the translational masses | Analysis Reference pp. 339, 376-378; two offset masses give 0.75 and 0.25 |
| Items of the load participation list | the starting loads, or UX, UY, UZ for a case without any | the starting loads, then each of UX to RZ that acts on mass | Analysis Reference p. 378 |
| Deformed shape of a joint with local axes | drawn along the global axes | drawn along the joint's axes | Analysis Reference p. 24 |
| The 3D view | mirrored: mathematically a view from below at the default camera | right-handed, from above at a positive pitch; looking straight down it equals the plan view | handedness of screen right × screen up |
| Animation of a result with steps | one state, oscillated | steps through the states | help topic "Show Deformed Shape" |
| Contour legend | 16 colour steps, minimum and maximum without a unit | ten bands, eleven tick values, unit | help topic |

A model without joint local axes has bitwise the UX, UY, UZ factors and ratios
it had (tested against a copy of the earlier loop), and its deformed shape is
drawn at the same positions.

No vendor file in the corpus is affected by the first two rows: of the native
files, thirty have joints with local axes and none of those has a modal,
response-spectrum or time-history case.

## What is refused

Each of these has a negative test.

- **Derived shell values of a multi-valued result.** Principal and von Mises
  values, bottom-face stresses and face extremes are given only for a
  single-valued result: a case other than a response spectrum (or a
  moving-load, steady-state or power-spectral-density case), or a linear
  combination of such results. Envelope, SRSS, absolute and range
  combinations are multi-valued (Analysis Reference pp. 214 and 352). Their
  resultants, top-face S11, S22, S12 and the transverse shear stresses are
  still shown. **This removes a value that was shown before:** the "Von Mises
  Stress SVM" contour of an envelope, SRSS, absolute or range combination,
  which was a field-by-field combination and not a von Mises stress.
- **Quantities a section does not have.** Membrane forces of a plate section;
  moments, shears and transverse shear stresses of a membrane or plane-stress
  section; every shell quantity of a plane-strain or axisymmetric-solid
  section (see "Found and not changed" below); the face stresses
  of a plate section in a multi-valued result (they come from the moments with
  a sign that an SRSS or envelope loses). **This also removes values shown
  before:** the legacy contours of such areas.
- A contour of an area whose value cannot be obtained: the area is drawn
  without a contour colour and the legend gives the count and the first
  reason. It is never drawn as zero. A displacement contour needs an area of
  three or four joints and a result at each corner joint.
- A contour minimum that is not below the maximum.
- Absolute acceleration of a direct-integration or nonlinear history ("only
  modal time-history cases record accelerations"); velocity and reaction;
  a cross-plot of functions of different cases; a time range that is empty,
  reversed or outside the record; a component outside the six; an empty or
  repeated name of a plot function or set.
- The joint read-out while the animation runs (as the help topic states).
- A rotational acceleration as a Ritz starting load (unchanged).

## Project choices the sources do not settle

- **Element-centre shell values.** The vendor evaluates stresses at the 2 × 2
  Gauss points, extrapolates them to the joints and can average them at
  joints (Analysis Reference pp. 212-213). This solver recovers one value at
  the element centre, so a shell contour is one flat colour per area and the
  form offers no averaging. The form says so.
- **Range combinations** count as multi-valued, as the manual defines them,
  although the solver still adds them linearly.
- **Visible face.** A tie (an area seen edge-on) goes to the top face. In the
  3D view the face follows the camera and switches as it passes edge-on.
- **The centroid for RZ.** The manual prints "the z centroid of the UY load
  and the y centroid of the UX load" for RZ (p. 339). A z coordinate cannot
  place a vertical axis, and the sentences for RX and RY follow the pattern
  that gives "x centroid of UY" here; that is what is used.
- **Masses that differ by direction on a rotated joint** leave a coupling
  between the global X and Y masses whose vendor treatment is unknown. The
  rotational shapes are orthogonal to the translational ones only for masses
  equal in the three directions.
- **Absolute acceleration** is the stored relative acceleration plus the
  ground acceleration `Σ scale factor × f(t)` of the case's acceleration
  entries, rotated into the joint's axes. The help names the quantity and
  gives no formula. Rotations equal their relative values.
- **Area axes for the displacement contour** are the solver's own, which add
  the section's material angle to the area's local-axis angle.
- **Read-out and picking.** A right-click picks a joint within 10 px of its
  drawn position; this applies only while the deformed shape of a result is
  shown, where the joint wins over a frame ending at it. Everywhere else the
  right-click opens the frame diagram as before.
- **Animation speed** 0.05 s per step, looping; stopping keeps the last step.
- **Colours and bands.** The existing blue-to-red ramp, ten bands, four
  subdivisions per area edge for the displacement contour. The vendor's
  defaults are not stated.
- **Tables.** The header strings of the load participation table (OutputCase,
  ItemType, Item, Static %, Dynamic %) follow the vendor's API topic; the
  database-table field names are not in the corpus. Names of plot functions
  and sets are compared without regard to case.
- **Display units** replace the vendor's unit switch, which converts a locked
  model in place; here a units change still needs an unlocked model.

## Evidence

| Check | Reference | Tolerance |
| --- | --- | --- |
| Rotated joint, 0°, ±30°, 90°, 137°: UX and UY ratios of the two modes | `ω² = k/m`; ratios 1 and 0 | 1e-12 |
| The same without joint axes | a copy of the earlier loop | bitwise |
| Response spectrum on the rotated joint | `Sa/ω² = 0.04` | 1e-12 |
| Two offset masses (1 at x = −2, 3 at x = +2): RZ ratios 0.75 and 0.25, UY 0.25 and 0.75 | centre of mass x = 1, inertia 12 | 1e-12; two rows of `structurax verify` |
| Space frame, all 24 modes: sums of the six ratios | 1 | 1e-10 |
| Dynamic participation of an acceleration item | cumulative mass ratio of its direction | 1e-12 |
| Static participation of a Ritz starting acceleration | 1 (p. 380) | 1e-10 |
| Shell strip, F11 = 50 kN/m, M11 = −10 kN-m/m, t = 0.2 m: S11 top 1750, bottom −1250, SVM bottom 1250, SMAX bottom 0, SMIN bottom −1250, face extremes | `F/t ∓ 6M/t²` | 1e-9 |
| Mohr (3, −1, 2); VMAX of (3, 4) | `1 ± √8`, von Mises 5; 5 | 1e-12 |
| Top face derived from the resultants | the stored top-face values | bitwise; 1e-15 |
| Unit factors: kN/m² to N/mm², ksi to N/mm², m to mm | 1e-3, 6.894757293168361, 1000 | 1e-12; exact |
| Example 2-010 (opt-in), four files, every area: hoop stress S11, top = bottom | printed 58.85 and 59.87 k/in²; closed form `pR·cos(π/N)/t` = 58.8471 and 59.8715 | 0.005; 1e-6 |
| Example 2-010 (opt-in), every joint: read-out U1; global Ux | printed 0.12175 and 0.12387 in; `Rσ/E`; `U1·cos θ` | 5e-6; 1e-6; 1e-9 |
| Joint results to global axes: 30° joint, explicit axis matrix | `(cos 30°, sin 30°, 0)`; matrix product | 1e-15 |
| Area local components: horizontal, 90° angle, X-Z area, tilted with a material angle | hand-derived axes; the solver's own system | 1e-12 |
| Contour mesh: counts, colours, interpolated values | `(s+1)²` vertices and `2s²` triangles for a quad; `(s+1)(s+2)/2` and `s²` for a triangle | exact |
| Colour ramp | a copy of the earlier function, 603 inputs | bitwise |
| Ground acceleration trace of a triangle function | `scale factor × f(k·dt)` | 1e-15 |
| Damped single mass (m = 2, k = 50, ξ = 0.05) under a step: displacement and absolute acceleration at 500 steps | `ü_abs = −ω²u − 2ξω·u̇` with the closed-form u | 1e-9 of the peak |
| Two-mass chain, all modes | `M·ü_abs + K·u = 0` | 1e-9 |
| Absolute acceleration at a 30° joint; at a restrained joint | `Rᵀ·a` of the global value; the ground acceleration | 1e-12; 1e-15 |
| 3D view: straight down with zero yaw; eye direction at four cameras | the plan view; `screen right × screen up` | 1e-3 px; 1e-5 |
| Visible face in plan, both elevations and 3D, for local 3 toward and away | top or bottom by the rule | exact |
| Deformed shape of a joint turned 90° | drawn at `base + scale·R·u`, not at `base + scale·u` | painted position |
| Refusals above | error text | exact |

## End-to-end

`crates/sap-app/src/gui/video_results_e2e.rs` and the files beside it drive
the application itself: the Build window, the Display menu, the two result
forms, the function and plot window, the modal tables window and the file
chooser are operated with pointer and key input through the application's
frame function, and the checked values are the texts and colours painted in
that frame or the values of the pure display functions. Restraints, loads,
analysis cases and the tower's columns are set with model commands.

- **Cylinder under internal pressure** (the fourth part's shell display against
  a closed form). A vertical line is extruded radially in the Build window
  (36 × 10°, R = 60 in, t = 1 in, p = 1 k/in²). Hoop membrane force
  `pR·cos(π/36)` = 59.77168 kip/in (measured difference 1.4e-12), shown through
  the Shells form as 412.1112 N/mm²; radial displacement `R·F/(E·t)` =
  0.1236655 in through the joint read-out (2e-9) and the Area 3 displacement
  contour. Tolerance 1e-6.
- **Strip.** S11 top 1.75 and bottom −1.25 N/mm², SMAX and SMIN of the bottom
  face, through the form with a typed range; a combination with a
  response-spectrum case paints the message that names the case without shell
  results; SMAX of an SRSS combination paints the refusal and the areas are
  not coloured; top-face S11 of the SRSS combination is
  `√(1750² + 1500²)` kN/m².
- **The video's tower.** A meridian through three points, extruded radially
  (36 × 10°), on V columns replicated radially, fixed bases. The dead-load
  reaction equals the weight summed from the facets and columns (1e-9). With
  ten Ritz vectors from UX and UY: equal sums of the UX and UY ratios and of
  the RX and RY ratios, static participation of UX 100 %, dynamic
  participation equal to the cumulative UX ratio, all read from the painted
  modal tables. Targets of 99 % stop at four vectors, and the painted target
  line says so. The mode-1 Ux contour has the range of the drawn joints and
  vertex colours of the stored mode shape; the read-out of a roof joint equals
  the mode shape; DEAD S11 and SMAX are shown in N/mm².
- **The third part's frames.** A record file in the video's layout (two header
  lines, eight values per line, 0.02 s) is read through the window, one of
  them picked with the chooser. A single-storey frame with a rigid roof
  reproduces the damped single-mass closed form at all 501 steps (measured
  1.7e-8 for the displacement and 1.4e-8 for the absolute acceleration,
  asserted 1e-6), and the plot window paints the maxima with their times. On
  the ten-storey frame (20 modes, 2999 steps) the absolute acceleration of a
  base joint is the scaled record in X and Y at every step (1e-15), the
  displayed maximum of a roof joint equals the trace maximum, a time range
  and a named set work, and the animation shows step `⌊t/0.05⌋`.
- **Vendor files (opt-in).** Example 2-010, four files: S11 and the read-out
  through the forms against the printed values and the closed form. Example
  1-022 through the modal tables window: UX ratios 0.800 to 0.003 as printed
  (6e-4), their sum 1, dynamic participation 100 %; the model is prepared as
  in the solver's own test of that example.

What the end-to-end work corrected in the plan, and what it turned up:

- The plan's supports for the cylinder (UZ on the base ring, UY at 0°, UX at
  90°) leave a rigid rotation about a vertical axis through (60, 60); a third
  horizontal restraint is needed. The solver reports that model as singular
  at this wall thickness and **accepts the same mechanism at others** (bending
  thickness 0.5 or 0.8 in), where roundoff keeps the pivot above its
  tolerance. The singular-matrix error also names a pivot index, not a joint.
  Both are existing behaviour and are not changed here.
- Ritz vectors of the symmetric tower come in pairs of equal period only for
  the first two pairs (2e-13); later pairs differ by 1e-6 to 2e-5 in a
  ten-vector run and up to 3e-3 in a twenty-vector run, which also contains
  modes of other circumferential harmonics with no participation. Ritz vectors
  are not converged modes; nothing is asserted about the later pairs.
- The plan's single-storey stiffness `4 × 12EI/h³` needs the roof rotations
  held; the curved-frame command's "parabolic" curve is parametrised along the
  chord, not by height.

## Review coverage

Read: the shell quantities and their rules, the contour and read-out
functions, the ground acceleration and absolute acceleration code, the
rigid-body participation shapes and the solver change, the modal table rows,
and the value tables, legend, fills and viewport changes of the result forms.
Not read line by line: the window code of the plot functions, of the modal
tables and of the two forms, and the tests of the packages and of the
end-to-end work, which were integrated on their results, the agents' reports
and spot checks.

Found in review and corrected:

- The 3D projection was mirrored, which made "Visible Face" show the bottom
  face of a slab in the default view. Corrected in the projection itself.
- The top-face stress of a plate section in an SRSS, absolute or envelope
  combination would have been rebuilt from the combined moment with the wrong
  sign. Now refused.
- Moving-load, steady-state and power-spectral-density cases counted as
  single-valued. They are classed with the response spectrum.
- An area whose value is refused kept the ordinary fill when no area at all
  had a value. It is now marked like any other refused area.
- Two end-to-end tests asserted that all Ritz vector pairs of the tower have
  equal periods to 1e-6, an expectation of the plan without a reference; they
  were removed and the measurements recorded above. Two more assertions
  depended on roundoff (that the solver reports the plan's mechanism as
  singular; equal X and Y sums in a twenty-vector run) and were dropped.
- Plane-stress sections were refused together with plane-strain ones. Their
  stored forces are results, so they are reported as membranes.

The windows were exercised through command functions and headless renders with
synthetic pointer and key input. **Nobody has used them by hand.**

## Not done

Video export, the load read-out, the plot window's speed, the stored von
Mises of multi-valued combinations, the material angle, the plane-strain
recovery and the naming of the unstable joint are since done (see
[tutorial-video-wave5.md](tutorial-video-wave5.md)); the current list is
[tutorial-video-progress.md](tutorial-video-progress.md).

- **2016 code editions** (AISC 360-16, ASCE 7-16): absent from the corpus.
  Seismic design category, framing type and second-order method preferences of
  steel design: described in the vendor manual, exercised by no verification
  problem.
- **Stress averaging and joint values of shells** (Gauss-point extrapolation,
  averaging at joints, smooth shell contours); shell results per mode.
- **Velocity, reaction, base and energy plot functions** (not recorded); plot
  sets saved in the model; absolute acceleration of direct-integration and
  nonlinear histories.
- **Video and image export** of an animation: no encoder may be added, and
  none was written.
- **A unit switch on a locked model**; a read-out of the loads on an object.
- **Speed of the plot window.** It rebuilds its traces on every frame. With a
  3000-step record the end-to-end test needed about 4 s per legend read in a
  debug build; a release build was not timed.
- **Area section "None"** for openings (the sources give no semantics) and
  areas with more than four joints (no element).
- **Reinforcement "to be checked"** with a bar layout: the corpus has ACI
  318-14 Example 002 as a reference; it belongs to a design wave.
- Von Mises and principal values of SRSS, absolute, envelope and range
  combinations are now refused in the display; the stored top-face von Mises
  field of such a combination is still combined field by field.
- **Found and not changed.** The solver takes the section's material angle
  into the area's output axes, where the vendor rotates only the material. It
  assembles a plane-strain section with the plane-strain relation but recovers
  its forces with the plane-stress one, and stores no out-of-plane stress, so
  the stored results of plane-strain areas are wrong; the display refuses
  them. The singular-matrix check lets a rigid rotation of a shell model
  through at some thicknesses and names a pivot index instead of a joint.
- Coupled and automatic hinges, export to other programs, `cSapModel` facades
  for the commands of the four waves, and everything listed as not done in the
  earlier waves and not named above.
