# Features of a SAP2000 tutorial video, fifth wave: repairs

Date: 2026-11-17. Scope note for one bounded set of repairs and additions. It
does not change the project status: vendor-runtime acceptance and broad parity
remain unestablished.

The first four waves ([tutorial-video-wave1.md](tutorial-video-wave1.md),
[tutorial-video-wave2.md](tutorial-video-wave2.md),
[tutorial-video-wave3.md](tutorial-video-wave3.md),
[tutorial-video-wave4.md](tutorial-video-wave4.md)) added features and each
left a list of defects it had found and not repaired. This wave repairs those
first and adds only the features that need no new solver data: video export,
a read-out of the loads on an object, the eccentric moment of the automatic
seismic load, the "Grid Only" start and an I section with unequal flanges. The
ledger of the whole line of work is
[tutorial-video-progress.md](tutorial-video-progress.md); the identifiers
below (D01, G07, ...) are its rows. As before, the video only lists which
commands a user meets; no number in it is a reference.

How the wave was planned:

- **Every open item of the four earlier waves was checked against the vendor
  material** before any code was written: the pages of the CSI Analysis
  Reference Manual, the lateral loads and design manuals, the topics of both
  help files, the verification problems, and a scan of the 277 native example
  files for the fields in question.
- That check **reclassified** some items. To "blocked: evidence": the hinge
  status rules (D13), the capacity-spectrum forms (G15), the site and
  occupancy tables (G22), the program period (G23), a brace preview (G29),
  hinge unloading methods 2 and 3 (G13b) and the default materials by region
  (G36b). To "open": video export (G08, the vendor's default is an
  uncompressed AVI, which needs no encoder), the steel second-order methods
  (G26, the formulas are in the design manual) and the operating system's
  file dialog (G30). Automatic wind (G25) and export to other programs (G35)
  have evidence for a part only. Typed hinge calls (G17) are done for the
  calls the vendor's API has.
- It **found twelve more defects** by reading the code against the manual
  (D20 to D31): nine are repaired here (two of them, D26 and D27, by a
  refusal), D25 and D29 belong to the time-history wave, and D30 has no
  evidence.
- While the wave ran, looking at the real windows added D32 and D33, a
  review that read the window code of waves 3 and 4 added D34 and D35, and
  the packages found D36 to D45.
- A second review read the wave's own solver and design code once it was
  integrated. It confirmed six defects (D46, two that became D47, D48, D49,
  which absorbed D42, and D51) and suspected a seventh (D50). The six were
  repaired by two more packages and the reviewer; D50 was measured by a
  third package and left open.
- The real program was then gone through by hand on a private display (see
  "Hand check"). That found D52 to D55; the repairs that followed added D56
  and found D57 to D59. One more package repaired the design code that reads
  the stations D48 had moved.

The wave ran in this order: a scaffold, seven packages in parallel (frames
and solver checks, area materials, combinations, native import, hinge
stations and eccentricity, windows, video export) and the review of the
earlier window code; then window repairs, format repairs and the Ritz
repair; then the end-to-end tests, the second review and the hand check;
then the repairs those two asked for. The sections below describe the final
state. Where a later package changed what an earlier one had done, the text
says so.

## Defects repaired, and reported values that changed

Results from before this wave differ in the cases below. "Published" is the
value printed in the vendor's verification problem.

| What changed | Before | Now | Reference |
| --- | --- | --- | --- |
| Axial and torsional stiffness of a frame with end offsets and a rigid-zone factor (D21) | all twelve stiffness terms were built on the flexible length `L − rz·(oi + oj)` | axial and torsional terms use the full length; bending and shear keep the flexible length. A bar of 144 in with 6 in offsets and rz = 1: axial 1.543209877e-3 in (was 1.414609e-3), twist 8.0e-3 rad (was 7.33e-3). For a non-prismatic section see the row of D47 below, which replaced this package's first rule | Analysis Reference p. 130; `PL/EA`, `TL/GJ` |
| Periods of Examples 6-006 and 6-007 (D21, D37, D36) | 0.4309, 0.1303, 0.0745 s as imported: −1.6 %, −3.5 %, +0.6 % against the published 0.438, 0.135, 0.074 s | **as imported the Modal case is refused** (the row of D36 below). With that entry removed explicitly, after all solver repairs of the wave (a package's print; the test asserts 1 %): 0.437263, 0.135023, 0.073709 s (−0.17 %, +0.02 %, −0.39 %), equal to the Eigen solution of the same model. With the rigid-zone repair alone, before the Ritz repair, the values were 0.439136, 0.134836, 0.0736459 s. The later repairs of the end zones (D46 to D48) did not move them | published periods; see "Found and not changed" for the remainder |
| A frame end with a moment or shear release and an end offset (D28) | the offset kept its rigid-zone factor | the offset is fully rigid for bending and shear in the plane of the release. Propped beam with an R3 release at the offset end: end rotation 0.0124137931 rad (was 0.0137931034) | Analysis Reference pp. 130-131 |
| Constraints at joints with local axes (D22) | the constraint equations tied joint-local components: a diaphragm tied one joint's local U1 to the other's global UX (1.10e-5 against 0.956e-5 in the test model) | Equal, Diaphragm, Body, Plate, Weld and the Rod, Beam and Line constraints act in global directions. The Local constraint type still ties joint-local components. A model without joint local axes is unchanged bitwise | invariance of the result under a rotation of one joint's axes; Analysis Reference p. 66 |
| Time functions in direct-integration histories (D23) | the first value was held before the first point; a missing function was evaluated as 1.0 | zero before the first point and constant after the last, the rule the modal path already had; a missing function is an error naming the case and the function. Response spectra keep their own rule (first ordinate below the first period) | a load that starts at t = 1 s: displacement exactly 0 for the steps before it, first loaded step `1/(k + 4m/dt²)` |
| The message for an unstable structure (D03, first part) | "Linear algebra error: Singular matrix encountered at pivot index N" | "Singular matrix: the structure is unstable at joint 'J2' R1 (zero or negative pivot after restraints)", with "(joint local axes)" after the direction where the joint has them. Where the joint is an internal joint of a frame's automatic subdivision the message names the frame, the joints at its ends and the nearer one instead (before the second review it named the internal joint, `'__fmesh_F3_2'`). **Which models are refused did not change** | exact null vector of a two-joint torsion pair; the same pair as one subdivided frame |
| Area elements with an orthotropic material (D20) | analysed as isotropic with E1 and u12 | the orthotropic constants are used: plane stress for shells and plates, the reduced compliance for plane strain. Example 2-015a centre deflection: thin −0.7939 → −1.0691 in (published −1.077), thick −0.7939 → −1.0690 (published −1.074); 2-015c −0.7957 → −1.0692 (published −1.070). The element-centre moments of these files change with it | Analysis Reference p. 75, Eqn. 3; closed form `q·a⁴/(8(3Dx + 2H + 3Dy))` = 1.069195 in |
| Recovered shell resultants of a section with unequal stiffness modifiers | the coupling term of the recovery differed from the stiffness': `F11` from `ε22` was twice `F22` from `ε11` in the test | one constitutive function serves stiffness and recovery (coupling `√(mi·mj)`). Example 2-015b, mean of the two elements at joint A: M22 −8.252 → −12.036, M11 −64.101 → −64.247; at joint B: M11 −16.095 → −10.882. Deflections unchanged. Every area result moves at the 1e-16 level because the recovery is now the same matrix product | symmetry of the constitutive matrix; model B equals model A algebraically |
| Plane-strain areas (D01) | forces recovered with the plane-stress relation; no S33; no stiffness normal to the plane | plane-strain relation in the recovery; `S33 = ν(S11 + S22)` stored (orthotropic: from u13 and u23 in material axes); anti-plane shear stiffness on U3 (`g13·t`, `g23·t`) with the shear resultants in V13 and V23; moments zero. **U3 of a plane-strain area now has real stiffness and no stabiliser**, so the model must support it | Analysis Reference pp. 217-219; uniform-strain patch 1.634615, 1.25, 0.865385, 0.076923 |
| The material angle of an area section (D02) | turned the element's output axes, its local-direction loads and the Area 1 and 2 displacement contours | turns only the material inside the element; output axes and local loads follow the area's own angle. **This reverses a project choice of the fourth wave** ("area axes ... add the section's material angle"). No corpus area section has a nonzero angle, so no vendor number moved | Analysis Reference p. 193; isotropic invariance; orthotropic membrane at 30°, F11 = 5.351413 |
| Drilling stiffness on Plane and axisymmetric-solid sections (D31) | sections made in the program or read from S2K carried the shell's drilling penalty (native files switch it off). Example 3-002 with the flag on gave tip Uz 0.00875 for the published 0.0101, and 7.46e-5 for 8.40e-5 | no drilling term on these sections whatever the flag says; the flag changes nothing | Analysis Reference p. 217; Example 3-002, files without incompatible modes |
| Combinations (D05) | one value per quantity: an additive combination with a response spectrum stored `D + RS` only | every multi-valued combination stores a maximum and a minimum. The manual's worked numbers: COMB1 5.5 / 1.5, COMB2 3.5 / −2.0, WIND ±5.8309519, GRAVEQ 17 / 3, GRAVWIN 15.8309519 / 4.1690481, SEVERE 17 / 3. An additive combination of single-valued cases is unchanged bitwise | Analysis Reference pp. 351-353 |
| Envelope combinations | the value of larger magnitude with its sign | the ordinary fields hold the maxima, the second row the minima. Contributions −0.10 and +0.06: was −0.10; now Max +0.06, Min −0.10 | p. 352 |
| Range combinations | added linearly | maximum = sum of the positive maxima, minimum = sum of the negative minima. Values 3, −2, 1.5 give 4.5 / −2.0 | p. 352 |
| Derived fields of a multi-valued combination: top-face von Mises of areas; principal and von Mises stresses of solids (D04) | combined field by field | 0.0 in both rows; tables print "n/a"; the display refuses them as before | p. 213 |
| API results and database tables of a multi-valued combination | one row | a Max row, then a Min row, per joint, per station and per element; the row count doubles | p. 351; row order is a project choice |
| Design with a multi-valued contributor (D24) | the positive sense only: a column with D = −10 kip and a spectrum 7 kip was designed for 3 kip; Range was linear; a stored Envelope was designed for its row of larger magnitude | both senses: that column is designed for 17 kip too. Range, SRSS and Absolute give two states; an Envelope is designed for each contributing state, so its ratio can go down. The next row adds what this first repair lacked | pp. 351-353; ratio equal to that of the same column under a static 17 kip |
| Design of a multi-valued combination: which forces act together (D49, which absorbed D42) | after the repair above the Max row and the Min row reached the design as one station list. The largest moment was never paired with the smallest axial force, and Cb was computed across the two rows as if they were stations of one member. The review's example, a column with D (P = −20, M3 = 100) and a spectrum case (P = 6, M3 = 80) added: ratio 0.0588 | each state (the Max row, the Min row, each contributing state of an Envelope) is a design entry with its own moment diagram and its own Cb. For a multi-valued combination the design adds, at every station, the permutations of maximum and minimum of the forces a check combines: P, M2 and M3 for the axial-flexure interaction and the column P-M-M check; V2 and V3 as well for Eurocode 3 section 6.2.8; T, V2 and V3 as well for the closed-section torsion interaction. The largest ratio governs. The example: 0.0859, the ratio of (P = −26, M3 = 180). A member whose Min row has a V-shaped diagram up to 100 and whose Max row is uniform at 20: 0.70051 → 0.72075, with Cb = 1250/690 for the Min row instead of 12.5·100/450 for all. **Single-valued combinations and plain cases are unchanged bitwise.** No corpus number moved | steel design manual (AISC 360-10) section 2.4 and concrete design manual (ACI 318-14) section 2.1, which ask for sub-combinations of the maxima and minima of the interacting forces; each expected ratio is that of a static case with the hand-enumerated force set |
| Span loads and self weight on a frame with end offsets (D46) | the part of a load that lies on the rigid part of the member was dropped, and with a released end this wave's first repair (D28) had widened that. Fixed-fixed beam of 144 in with 12 in offsets and a uniform load 1: the reactions summed to 120 with releases (and for rz = 1 without) and to 132 for rz = 0.5. An inclined member with a release and self weight 14.4 had a base reaction 0.576 along the horizontal | loads are placed along the full length; what lies on a rigid arm goes to its joint as a force and its moment about the joint; the flexible part carries the rest. The reactions sum to 144 for rz = 0, 0.5 and 1, with and without releases; the horizontal reaction is 0. A member without offsets is unchanged bitwise | Analysis Reference p. 135 (self weight is constant within the end offsets); statics; fixed-fixed closed forms for the joint moment (`w·lf²/12 + w·lf·a/2 + w·a²/2`) |
| Non-prismatic frames with end offsets (D47) | the section variation was laid over the flexible length `L − rz·(oi + oj)`, and the fixed-end forces of thermal and axial span loads used the stiffness without the end zones. Axial tip displacement × E / P of the review's bar (areas 200 then 100, offset 24 at I, rz = 0.5): 1.05. Free thermal expansion of a cantilever with a 12 in offset: 0.140 | the variation runs over the clear length `L − oi − oj` whatever the rigid-end factor; the end zones have the end sections; the part `(1 − rz)·offset` is flexible in bending and shear with the end section. The bar: 84/200 + 60/100 = 1.02. Thermal expansion: `α·ΔT·L` = 0.144. Axial span load at mid length, fixed-fixed: reactions 6.5714 and 3.4286 (were 6.667 and 3.333) | Analysis Reference pp. 120, 123 and 130; unit-load integrals written in the test |
| Geometric stiffness of rigid end zones | a rigid arm contributed nothing to the geometric stiffness (older than this wave; the second review suspected it and the package confirmed it) | each rigid arm adds `P × arm` to the rotation stiffness of its joint. **This changes P-Delta and buckling results of every member with rigid arms.** Test column pinned at its joints, arms 10 in at both ends, flexible length 100 in: critical load 3480 → 2175 kip | hand derivation `Pcr = 4EI/(lf·(lf/3 + 2d))` for the single cubic element; the same column with the arms as stiff elements, equal to 1e-4; above the continuum root of `tan(k·lf/2) = 1/(k·d)` |
| Output stations of a frame with end offsets (D48) | the stations ran from joint to joint. With the released-end rule (D28) a released end then reported the moment at the joint, −720 for the beam of D46, where the support face has zero | the first and last stations are at the support faces and the others are equally spaced between them; none lies inside an offset. `station` is the distance from joint I and `rel_dist` is `station / L`, so a member with offsets has no station at 0 or 1. Forces come from statics with all loads; a released end has M3 = 0 at its face. Deflections at the faces are the joint translation plus arm × joint rotation. Members without offsets are unchanged bitwise | Analysis Reference pp. 130 and 144; fixed-fixed and simply supported closed forms |
| Design of a frame with end offsets (a consequence of D48, found by reading by the package that moved the stations, and repaired by a later one) | once the stations ran between the support faces, concrete beam design, which looked for the ends of a member at relative distance 0 and 1, found none: the end and span minimums of special and intermediate frames stopped applying (unconservative; in the test beam a bottom steel area of 0.715 where the rule gives 1.656, by the package's run). The steel Cb sampled the moment at relative positions from 0 to 1 and took the moment of the last station for a position before the first one (Cb 1.3473 for 500/440) | the ends of a member for the concrete minimums are its first and last output stations, the support faces. Cb quarter points and the segments of an unbraced-length ratio are measured over the span from the first to the last station, which equals the clear length that the unbraced length already used. A moment is never read outside the stations. Members without end offsets are unchanged bitwise; no corpus number moved (the corpus members with offsets are not designed) | concrete design manual (ACI 318-14) section 3.5.1; AISC F1-1 by hand; the design of a plain station list without offsets that carries the same moments |
| Examples 1-010b, c and d (offset 6 in at joint I; rz = 0, 0.5, 1) through D48 | nine stations at 0, 18, ..., 144 in; M3 = −1440 at the first | 6, 23.25, 40.5, ..., 144 in; M3 = −1380 at the support face. Deflection at the first station: 0 at the joint before; now −1.1997e-3, −3.4265e-4 and 0 in at the face. The last station is unchanged and equals the published tip deflection (−0.39951, −0.37523, −0.35197). No joint displacement and no reaction moved (the reaction moment stays −1440). **These are the package's before-and-after prints, not asserted by a test**; the existing test asserts the tip deflections | statics: `−10 × (144 − 6)`; published tip deflections |
| Station deflections of a frame with a flexural hinge inside it (D09) | no kink: 0.0140625 where the closed form gives 0.0023958 at the first inner station of the test cantilever | the slope jump of each M2 or M3 hinge is added: 0.0091666667, 0.0333333333, 0.1141666667, 0.2 at the test stations. Joint displacements, reactions, hinge results and station forces are unchanged | `v(x) = P·x²(3L − x)/(6EI) + θp·max(0, x − xh)` |
| Native import of modal time histories (D07, D26, D27) | the case's modal case and motion type were ignored; a history with zero damping was integrated directly | the modal case is imported and the history is superposed on its modes. A periodic history and a nonlinear modal (FNA) history are refused by name. Example 6-001 with its ramp samples attached: 0.3471800 and 1.9999993 (MHISTA at 1 s and 4 s), 0.6943601 and 0.7403038 (MHISTB) for the published 0.34718, 2.0000, 0.69436, 0.74030. **Example 6-005a NLMHIST1 no longer runs** (FNA); its direct-integration case is unchanged | native case record; published values; closed form of the ramp response |
| Ritz modes of stiff models (D37; see D50 under "Found and not changed") | the projected stiffness lost its digits where the basis vectors had large massless components (penalty diaphragms, very stiff members): more modes than mass degrees of freedom, frequencies below the exact ones, or a failure "non-positive eigenvalue" | the projection uses the stiffness form where its roundoff estimate is small and the flexibility form otherwise; at most as many modes as mass degrees of freedom. Measured by the package on its own baseline, which had neither the rigid-zone repair nor the starting-load entry (next row) that makes these cases refused as imported: Examples 6-006 and 6-007 give 10 modes (were 11); Example 6-010 T1 to T3 2.299119, 2.271675, 1.818099 → 2.304759, 2.280756, 1.801268 s; Examples 6-011a and 6-011b solve (64 modes) where they failed with eigenvalues −6.3e34 and −3.3e33. The new Ritz values equal the Eigen values of the same models to nine digits | frame stiffness written in the test, static condensation and a Jacobi solve; Analysis Reference p. 373 for the mode count |
| The Ritz starting-load record of a native Modal case (D36) | a record other than a load pattern or an acceleration was dropped silently. The vendor's "all link element nonlinear degrees of freedom" is such a record | recorded as unmodeled source data, so the Modal cases of Examples 6-006, 6-007, 6-010, 6-011a and 6-011b are **refused as imported**. Their periods are no longer reported | project rule: refuse, do not approximate; [unmodeled-source.md](unmodeled-source.md) |
| Concrete design of a section without a rebar yield stress (D16, D38) | designed with 60 ksi | refused with `DesignError::MissingRebarYield`, naming the frame, the section and the material | project rule; `As = Mu/(φ·fy·(d − a/2))` for the stress that is given |
| Automatic seismic load on a model with diaphragms and a nonzero eccentricity ratio (G24) | refused; the default ratio is 0.05 | analysed, with the eccentric moment (see below) | lateral loads manual, section 2.2.3 |

What did not move:

- No vendor number changed through the combination repair: the corpus has
  121 combinations and all are additive combinations of single-valued cases.
- The rigid-zone repair (D21, D28) moved Examples 6-006 and 6-007 only. The
  joint displacements of Examples 1-010b, c and d (offsets, transverse load)
  are unchanged to twelve digits in the packages' before-and-after prints,
  and their existing test against the published tip deflection still passes.
  Their **station** positions and station moments did move, through D48 (the
  row above).
- The repairs of loads on rigid arms (D46), of non-prismatic end zones (D47)
  and the arm geometric stiffness moved no corpus number, by the package's
  before-and-after print of Examples 1-010b to d, 6-006 and 6-007 and by the
  corpus tests: no corpus model has a load on a rigid arm or a non-prismatic
  member with offsets.
- The design repair (D49) moved no corpus number, for the same reason as the
  combination repair.
- The Ritz repair leaves every model on the stiffness form where that was
  sound; the tower of the fourth wave is one of them.

## What is supported now

### Video export (*File > Create Video*)

- **Multi-Step Animation** of a case with saved steps (a time history, or the
  saved states of a multi-step static case, which then uses the load factor
  where a history has the time): start, end and increment. The frame count is
  `⌊(end − start)/increment + 1e-9⌋ + 1`; frame k shows the saved step
  nearest to `start + k·increment`.
- **Cyclic Animation** of a mode of a modal case: cycles and frames per
  cycle; frame k shows the mode shape times `sin(2π·k/frames per cycle)`.
- Magnification, frames per second, frame size and file name. Defaults:
  640 × 480, 10 frames per second, 3 cycles of 20 frames. The magnification
  is the automatic scale of the view until the box is edited: that of the
  chosen mode for a cyclic video, that of the step with the largest
  translation for a multi-step one. (As first built the default was a fixed
  10; D45.)
- The file is an **uncompressed AVI** (24 bit, the vendor's default "Full
  Frames"), written by the project's own writer. The frames are drawn by a
  software rasteriser from the same shapes the screen is drawn from; no
  encoder or image library was added. The form says that geometry and colour
  values match the screen but not pixel for pixel.
- The status line reports "Video written (N frames): path". The file is
  written under a temporary name and renamed, so a failed write leaves none.
- The title of a view wraps to the width of its pane, so it is whole in a
  narrow video frame.

### Read-out of the loads on an object

In the Undeformed display a right-click on a joint, frame or area opens a
floating *Object Information* window with a Loads tab: every stored load
record of the object with its pattern, direction and value, in display units
chosen in the window. A joint is picked before a frame, a frame before an
area.

### Eccentricity of the automatic seismic load

- For each diaphragm about Z that carries load of the pattern a moment about
  Z is added: `ratio × width × F`, or `overwrite distance × F`. The width is
  the largest minus the smallest coordinate perpendicular to the load over
  the diaphragm's joints; F is the pattern's total force on them.
- Overwrites are stored per pattern and diaphragm and set in the seismic
  window, which also lists the computed moments after an analysis.
- Without a diaphragm, or with a zero ratio or a zero width and no
  overwrite, nothing is added and the results are equal bitwise.
- The S2K file keeps the overwrites in the extension table
  `RUST AUTO SEISMIC ECCENTRICITY` (`LoadPat`, `Diaphragm`, `Distance`, the
  distance in the file's length unit), written only when there are any.

### Smaller additions

- **Grid Only** in the template window: a blank model in the current units
  and the Quick Grid Lines form.
- **I section with unequal flanges**: the custom I-section dialog has top and
  bottom flange width and thickness and a "Same as the top flange" box that
  is ticked by default.
- **Automatic scale of the deformed shape.** The largest drawn translation is
  10 % of the diagonal of the model's bounding box. It is the default; a
  typed factor or the slider (logarithmic, 1e-3 to 1e6) switches it off.
- **View titles** of the Deformed display carry what is shown: "Mode 1,
  Period = 0.00944 s, Frequency = 105.9783 Hz", "Step 12, Time = 0.2400 s",
  "Step 3 of 10, Load factor = 0.50000".
- **Max and Min rows.** The API returns the Max row and then the Min row
  under the same output-case name; `*_step_type` variants of the result
  functions carry "Max", "Min" or "". The database tables and the result
  tables of the window have a Step Type column. The frame diagram draws both
  curves on one scale. The Shells form shows, of the two rows, the value of
  larger magnitude with its sign.
- **Rebar material in the section dialog.** A concrete section made in the
  dialog names a rebar material chosen from the model's materials that have a
  yield stress.
- **Table messages.** The Area, Solid and Link tabs of the result tables name
  the case of a combination that has no such results (D15).
- **Native files.** A linear modal history keeps its modal case through
  native import and export.
- The plot window builds its traces once per result and setup instead of on
  every frame (D10), and uses the solver's time-function rule (D11).
- **Messages** (*Help > Messages...*): the status messages of the session,
  newest first, at most 200, each marked as an error or as information and
  with a Copy button. The status line cuts a long message; the whole text is
  in its tooltip and in this window (D54).
- **Frames with end offsets in the drawing.** The deformed shape draws the
  rigid arm from the displaced joint to the first station and from the last
  station to the other joint; the moment diagram starts at the support face
  (D56).

### Frames with end offsets

The rules after this wave, in one place (the rows of D21, D28, D46, D47 and
D48 above give the numbers):

- Axial and torsional deformation use the full length; bending and shear use
  the flexible length `L − rz·(oi + oj)`; a full moment or shear release
  makes the offset at that end rigid in the plane of the release.
- Span loads, point loads and self weight act over the full length. The part
  on a rigid arm goes to the joint with its moment.
- A non-prismatic section varies over the clear length `L − oi − oj`; the
  end zones have the end sections.
- A rigid arm adds `P × arm` to the geometric stiffness of its joint's
  rotation, in P-Delta and in buckling.
- Output stations run from support face to support face.

Unchanged limits: automatic subdivision and frame hinges are refused on a
member with end offsets.

## Window defects found by reading and by looking

### The review of the window code of waves 3 and 4 (D19)

The window code and the tests of the third and fourth waves had been
integrated on the agents' reports. They were now read. The review confirmed
each finding with a scratch test before reporting it. It found no panic and
no wrong result number.

Coverage, as the review's report states it. Read completely: the result
forms, the draw tools, the plane views, the display units, the non-test
parts of the plot window, the modal tables, the material library, the
section import and the file chooser, and the third wave's changes to the
Build window. Read as the two waves' diffs plus the reset, units, click and
panel code, not top to bottom: `gui.rs`. Sampled: the selection rules, the
contour and shell-output functions the windows call, and the model commands
behind the draw tools and the grid editor. The test files were read by two
delegated audits; the reviewer re-read the items of the confirmed table.
Not reached: the end-to-end harness, the explorer-panel code outside the
diffs, and any run of the real program (the review was headless).

Nine defects (D34), all repaired. The repairing package reports that it ran
each new or rewritten test once with the behaviour broken by a local edit,
and that each failed:

1. **File chooser.** After a click on a file and a retyped path, Choose
   returned the clicked file, so a save overwrote it. The typed path now
   wins.
2. **Show Deformed Shape form** opened on mode 1 or step 0, so OK on an
   untouched form jumped there. It opens on the mode and step on screen; the
   final state of a result with steps opens as its last step.
3. **Units selector.** The plane views kept their coordinate (a plan at
   Z = 6.4 m became an empty pane in N-mm). The planes are converted, and
   points already picked by a draw tool are dropped.
4. **Shells form** reopened with the model's units and the old range text.
   It reopens with the units and range of the applied contour, or with an
   empty range. The deformed-shape form had the same fault after a shell
   contour.
5. **Enter and Escape** went to the pane under the pointer, which discarded a
   polygon picked in the other pane. They go to the pane the points were
   picked in.
6. **Grid editor.** "Read current lines" wrote a text its own parser refused
   for an id with a blank or comma. The last token is now the coordinate and
   the rest the id.
7. **Custom-grid template texts** were not converted with the units (240 m
   bays in a kN-m model).
8. **Status line of the Shells form** named the top face for "Visible Face".
9. **Plane titles** printed roundoff ("Z=9.600000000000001"); coordinates
   are printed with at most ten significant digits.

Two decisions the review asked for were taken: the deformed-shape form shows
the final state as the last step, and *Edit Grid Lines* opens with the
current lines of the axis and reloads them when the axis changes (it opened
empty, and "Apply" on the empty box removed the lines).

Eight tests could not fail or did not check their claim (D35). Seven were
rewritten: the validation
test of the deformed form (it compared a copy with itself), the routing of a
draw-tool window (its model had no grid), Escape and Enter while a draw tool
is active, the Set 2D View dialog (no valid coordinate was ever applied), the
refusal of a second record file under an existing function name, the reset of
plot functions and sets when another model is loaded, and the Ritz controls
of the case editor. The eighth, the silent skip of the opt-in vendor tests
when the corpus is absent, is the project's rule and stays. Three tests the
review only suspected were weak as well and were strengthened: the cylinder's
three horizontal restraints (now checked by the determinant 2R of the
rigid-motion rows), the ten-storey animation (at the old scale no joint moved
by more than 0.04 px, so the undeformed picture passed), and the Ritz
control's set and remove.

### Looking at the real program

The application is an X11 client. Since this wave it is started on a private
virtual display, driven with synthetic clicks, keys and drags, and looked at
through screenshots. Nothing is drawn on the user's own display. This is how
the following were found:

- **The deformed shape had no automatic scale (D32).** The first mode of the
  shell dome template was drawn far off the screen at the old default factor
  25; the automatic factor for it is 7.49.
- **The view title did not say what was shown (D33).**
- **Thin polygons drew spikes (D41).** The dome's first mode, and the
  undeformed dome, showed long straight lines from the apex across the whole
  view; they were in the recorded video too. The cause was the drawing, not
  the result: the toolkit's convex-polygon shape strokes with mitred corners
  and feathers the fill, and for a sliver (an area seen edge-on) that grows
  without limit. The repairing package reports a 70 × 22 px quad that
  tessellated to 524 × 161 px. Areas, solid faces and extruded frame sections are now drawn
  as a fill fan without fringe plus one segment per edge.
- A long status message overlapped the scale controls; the status line is
  truncated (the whole text is readable since D54, below).
- A new or loaded model kept the old zoom and pan; they are reset.

What follows is what the tracker and the packages' reports say was looked
at; no screenshot is kept in the repository. When the display was first set
up, before the packages started: the deformed-shape form, the joint
read-out, the modal tables, and the Shells form with its contour and legend.
By the packages: the
seismic window with an overwrite and its moment line after an analysis; the
dome's first mode before and after the drawing repair; the Object
Information window on a vendor frame example; the Grid Only button; the
I-section dialog; the Create Video cyclic form and one recording of the dome
(640 × 480, 60 frames), of which one frame was extracted and read; the grid
editor on both axes; the section dialog with reinforcement; the
deformed-shape form; Set 2D View. Not looked at by those packages after
their changes: the Shells form, the file chooser, the Max and Min tables and
diagram, the form that Grid Only opens, and the plot window with a case.
The hand check that followed went through all of these except the Max and
Min diagram and the chooser behind *Browse...*; it is under "Hand check".

### What the hand check found (D52 to D56)

The hand check drove the four parts of the video through the windows and
reported what was wrong on the screen. The repairing package verified each
report in the code first. Its new tests were not run against the tree
before the repair, except where a test cannot exist there; it says of one
(the explorer test) that it fails before the repair by its expected text.

- **Numbers printed with a fixed number of decimals (D52).** The explorer
  printed a section as "A=0.01, I33=0.0" in kN, m, where I33 is 3.3e-4 m⁴;
  the hand check took the section for one without bending stiffness.
  Quantities are now printed with at least four decimals and at least five
  significant digits, without trailing zeros: the explorer (joint
  coordinates, section area and inertia), the station table of the frame
  inspector, reaction labels, the concrete reinforcement labels and table,
  the peak labels of diagrams, the seismic moment lines and joint-force
  table, the pushover table, Object Information and the summary of a
  function read from a file ("39.98 s" for "39.980000000000004 s"). A
  reaction label is shown when the component is above 1e-6 of the largest
  one (before: above 1e-4 in absolute terms). A base shear below 1e-9 of the
  peak prints 0 in the pushover table.
- **Opening a file kept the display of the old model (D53).** A shell-stress
  display stayed on and said "no results", and the 2D pane lost its plane
  and drew overlapping labels. Opening or creating a model now sets the
  Undeformed display, the automatic scale, no animation and the first pane,
  and a model without results is unlocked. A 2D pane keeps its plane when
  the new model has a grid line there (or a joint coordinate, on an axis
  without grid lines); otherwise it shows the whole model. If every joint
  lies in one global plane, each 2D pane opens on that plane, so the 2D
  frame template and a planar file open as an elevation, not as a plan at
  Z = 0 that shows a line of joints. The reported "Locked" after opening an
  S2K file could not be reproduced: an S2K file parses as unlocked.
- **A long message could be read nowhere (D54).** The reason of a refused
  pushover run was cut at the edge of the status line. See *Messages* above.
- **Smaller items (D55).** The seismic window and Model Workflows open at
  560 × 600 and 680 × 560, never taller than the screen less 60 px, and
  scroll. The pushover curve window has tick marks at multiples of 1, 2 or 5,
  tick labels and axis names inside the plot; with negative data a label ran
  off the right edge. The duration of a function from a file and the title
  of a video frame are named above.
- **Rigid arms were not drawn (D56).** With the stations at the support
  faces (D48) the deformed line of a frame with offsets stopped at the faces
  and left a gap to the displaced joints.

Not changed, by decision: the result tables keep a fixed number of decimals
(D58), and the quick-draw forms stay forms (the mouse tools are in the Draw
menu on a plane view).

### Behaviour of existing windows that changed

- Show Deformed Shape defaults to the automatic scale; the factor 25 applies
  only after choosing "Scale Factor".
- In the Undeformed display a right-click opens Object Information instead
  of the frame diagram inspector. Every other display keeps the inspector,
  and the joint read-out on the deformed shape is unchanged.
- The file chooser, the two result forms, the units selector, Enter and
  Escape, the grid editor and the plane titles behave as described above.
- Area and solid polygons have no anti-aliased fringe; this is visible in
  videos too.
- Numbers in the explorer, the station table, the reaction labels, the
  seismic and pushover tables and Object Information print with other
  digits than before: "100.0000" is "100", "0.0" is "0", and a small value
  keeps five significant digits.
- Opening or creating a model resets the display, the scale, the animation
  and, without results, the lock; the 2D panes keep or change their plane as
  described above.
- The seismic window and Model Workflows have a default size and scroll.
- The video magnification starts at the automatic factor, not at 10.
- The pushover curve window has a new layout.
- A cut status line shows its whole text as a tooltip; *Help >
  Messages...* is new.
- Station positions of a frame with end offsets, in the inspector, the
  result tables and the database tables, start at the offset of joint I, not
  at 0.

## What is refused

Each of these names its reason in the error and has a negative test (file
and test in brackets; tests marked "opt-in" need the vendor corpus).

- **An unstable structure**, naming the joint and the direction, or the
  frame and its end joints where the joint belongs to a frame's automatic
  subdivision
  [`wave5_stability.rs`: `r1_a_singular_torsion_pair_is_named_at_its_joint_and_dof`,
  `r1b_the_same_pair_at_a_joint_with_local_axes_says_so`;
  `wave5_rigid_zones.rs`: `an_unstable_release_set_is_still_refused`;
  `wave5_unstable_frame_name.rs`:
  `an_unstable_subdivided_frame_is_named_by_its_frame_and_end_joints`,
  `an_unstable_joint_of_the_model_keeps_its_name`].
- **A time-history case naming a function that does not exist**
  [`wave5_direct_functions.rs`: `a_missing_function_is_an_error_naming_it`].
- **An orthotropic area material with constants that give no positive
  definite law** (a modulus or shear modulus not positive, or the normal
  compliance indefinite), naming the area
  [`wave5_area_materials.rs`:
  `analysis_refuses_an_orthotropic_material_with_a_negative_modulus`,
  `material_check_functions_reject_invalid_orthotropic_constants_by_name`].
- **A temperature load on an area whose material has a1 ≠ a2**: the thermal
  load is implemented for equal in-plane coefficients only
  [`analysis_refuses_a_temperature_load_on_an_area_with_unequal_expansion_coefficients`].
- **Reverse local 3 of an area whose section has an orthotropic material and
  a nonzero material angle**: the command preserves the element axes and
  cannot preserve that material orientation; the model is left unchanged
  [`sap-model/tests/generate_radial.rs`:
  `reverse_refuses_an_orthotropic_material_with_a_material_angle`].
- **The multi-valued combination option of the API other than 1**
  (envelopes): the other options are not implemented
  [`sap-api/tests/wave5_combo_rows.rs`:
  `multi_valued_combo_option_one_is_the_only_one_implemented`].
- **A periodic modal history** of a native file ("periodic modal history of
  case 'X' is not implemented"), **a nonlinear modal (FNA) history**, a
  motion type other than transient or periodic, and a modal-case index that
  does not name a Modal case. Each is recorded as unmodeled source data of
  that case [`sap-format/src/sdb.rs`: `native_modal_history_import_branches`;
  opt-in `wave5_vendor_history.rs`: `a_periodic_history_is_refused_by_name`,
  `an_fna_history_is_refused_by_name`].
- **A Ritz starting-load record of a native Modal case that is neither a
  load pattern nor an acceleration**: the solver has no such starting vector
  [`sap-format/tests/wave5_p10_formats.rs`:
  `the_link_nonlinear_starting_vectors_are_recorded_naming_the_case`,
  `any_other_starting_load_type_of_a_modal_case_is_recorded_naming_the_type`,
  `the_entry_survives_s2k_and_stops_the_native_writer`; opt-in
  `wave4_vendor_regressions.rs`:
  `vendor_sdb_example_6_006_modal_case_is_refused_for_its_link_starting_vectors`].
- **Native export of a modal-case reference** on anything but a linear
  history naming an existing Modal case; **native export of seismic
  eccentricity overwrites** ("use S2K"): the native writer has no record for
  them [`sap-format/tests/wave5_native_history.rs`:
  `modal_cases_the_writer_does_not_carry_are_refused_by_name`;
  `export_guard.rs`:
  `every_other_use_of_modal_case_is_refused_naming_the_case`,
  `seismic_eccentricity_overwrites_are_refused_naming_the_pattern`;
  `wave5_p10_formats.rs`:
  `the_native_writer_refuses_overwrites_naming_the_pattern`].
- **A row of the S2K eccentricity table** with an unknown pattern or
  constraint, a missing or non-finite distance, a missing key or a repeated
  pair; the message quotes the row
  [`a_row_naming_an_unknown_pattern_or_constraint_is_refused_with_the_line`,
  `malformed_rows_are_refused_naming_the_table`].
- **Concrete design without a rebar yield stress**: neither the section's
  rebar material nor the concrete gives one
  [`sap-design/tests/wave5_concrete_fy.rs`:
  `a_library_concrete_without_rebar_yield_is_refused_naming_section_and_material`,
  `a_rebar_material_without_fy_is_refused_too`]. In the section dialog a
  reinforced section cannot be added while the model has no material with a
  yield stress [`gui.rs`:
  `the_dialog_says_when_the_model_has_no_rebar_material_and_refuses_the_section`].
- **Seismic eccentricity**: an overwrite naming a constraint that is not a
  diaphragm about Z, a non-finite overwrite, a loaded diaphragm that is not
  about Z, and a diaphragm with no free rotation about Z. Each names the
  diaphragm [`wave5_eccentricity.rs`: `refusals_name_the_diaphragm`;
  `gui/seismic.rs`: `eccentricity_overwrite_commands_validate_and_clean_up`].
- **An I-section dimension that is not positive or does not fit**, naming
  the field [`gui/i_section.rs`: `a_wrong_dimension_is_refused_with_its_name`].
- **Create Video** before anything is written: an unlocked model, a case
  without results, a case without saved steps or without mode shapes, a mode
  number outside the case's modes, an empty or reversed range or one outside
  the record, an increment that is not positive, no cycles, fewer than two
  frames per cycle, a frame size outside 2 to 4096, a rate outside 1 to 120
  frames per second, a magnification that is not positive, an empty file
  name, more than 100000 frames, and a file above 1 GiB (the writer has no
  large-file index) [`gui/video_export/tests.rs`:
  `frame_plan_refusals_name_the_reason`, `avi_refusals`,
  `avi_one_gib_limit_is_arithmetic_on_the_layout`;
  `gui/wave5_windows_tests.rs`:
  `nothing_is_recorded_for_a_locked_out_or_result_less_state`,
  `an_error_of_the_recorder_reaches_the_status_line_and_no_file_is_left`].
- **A frame the rasteriser cannot draw**: a paint callback, a mesh with a
  texture other than the font atlas, bad indices, a font texture that was
  not delivered [`rasterize_refusals`].
- **A units change on a locked model** stays refused; the plane views are
  then left as they were [`gui.rs`:
  `a_plane_view_and_picked_points_survive_a_unit_change`].
- **A file that cannot be read** through *Open*: the model, the display and
  the panes stay as they were [`gui/p16_open_tests.rs`:
  `opening_a_file_that_cannot_be_read_changes_nothing`].

Still refused, unchanged: derived shell values of a multi-valued result; a
plane section that asks for incompatible bending modes, which keeps the six
plane-strain files of Example 3-004 refused as imported; temperature loads on
plane-strain sections; a modal history with Rayleigh damping; automatic
subdivision and hinges on a frame with end offsets.

One refusal the hand check met is a gap, not a rule: the Shells form
refuses every quantity of a combination with a response-spectrum case,
because the solver stores no area results for a response-spectrum case
(D57, under "Found and not changed").

## Project choices the sources do not settle

Frames and constraints:

- **Partial fixity is not a release.** A full release makes the offset rigid
  in its plane (the manual's rule); a partial-fixity spring keeps the
  rigid-zone factor. The manual does not say.
- **End zones of a non-prismatic frame.** The manual gives the variation
  over the clear length and constant end zones. The part `(1 − rz)·offset`
  that stays flexible in bending and shear takes the end section too; that
  is this project's reading.
- **Station coordinates with end offsets.** The manual places the stations
  (support faces, equal spacing between them). That `station` is the
  distance from joint I and `rel_dist` is `station / L`, the same scale as
  loads and hinges, is a project choice; a consumer that looks for an end
  at `rel_dist` 0 or 1 finds none. Design takes the first and the last
  station as the ends and measures Cb and unbraced segments between them.
  The manuals do not state this for end offsets; it follows the clear
  length of the unbraced length and the support-face rule of p. 130.
- **Geometric stiffness of a rigid arm** is `P × arm` on the joint rotation,
  from the energy of a rigid bar that turns with the joint. The manual has
  no sentence on it. The flexible part keeps its consistent geometric
  stiffness on the flexible length.
- **The Local constraint type** keeps tying joint-local components.
- **Response spectra keep their own function rule** (the first ordinate
  below the first period, the last above the last): a spectrum is read at
  periods, not at times.

Areas:

- **Stiffness modifiers** scale the diagonal constitutive terms by `mi` and
  the coupling terms by `√(mi·mj)`. Whether the vendor does the same is not
  established (see Example 2-015 model B under "Found and not changed").
- **Anti-plane shear of plane strain** is stored in the V13 and V23 fields
  as `S13·t` and `S23·t`, and the v13 and v23 modifiers scale its stiffness.
  The manual's sentence on these stresses names "s12, s13"; it is read as
  s13 and s23.
- **Thin shells** take their transverse-shear penalty from g12; only thick
  shells use g13 and g23.
- The orthotropic compliance follows the manual's Eqn. 3 as printed.

Combinations:

- **Row order** Max, then Min. The API topic says only that a row is "either
  Max or Min". Functions without a step-type field repeat the case name.
- **Step type of a single-valued result**: "" in the API, "Static" in the
  database tables, as before.
- **Shell display** of a multi-valued result: each quantity is evaluated on
  both rows and the value of larger magnitude is shown with its sign; a tie
  goes to the Max row.
- A stored spectrum-type value is read as its absolute value.
- **Derived fields** (von Mises, principal values) are stored as 0 in both
  rows of a multi-valued result. The manual says only that they exist for
  single-valued results.
- **The API's multi-valued option** accepts 1 (envelopes) only.
- **The minimum state of an SRSS or Absolute combination in design** is the
  negation of every field of the maximum state (before, only the axial force
  was negated; the checks use the magnitudes of moment and shear).
- **Design of an Envelope** uses each contributing state as a design entry,
  and then the permutations of maximum and minimum over those states like
  any other multi-valued combination. (Before the second review the
  permutations were left out on the argument that they pair forces of
  different states; the vendor's design manuals ask for exactly that
  pairing.)
- **One diagram per design entry.** A permutation takes the same choice at
  every station: its moment diagram is the Max-row diagram or the Min-row
  diagram, never a mixture, and Cb is computed from it. The manual says only
  that Cb is taken per unbraced segment.
- The fields a permutation does not vary (shears, torsion, deflections) come
  from the state that supplies the chosen M3. States whose station lists
  differ in length are designed one by one and not permuted.
- **Concrete beams** have no permutation, because no beam check combines two
  forces. The strength floors of special and intermediate frames read the
  end moments of the state itself.
- **Shear checks** (AISC chapter G, Eurocode 3 section 6.2.6) take the shear
  alone, from the states.
- A contributor that has results but no load-case entry counts as
  single-valued.

Native files:

- **Modal-case index 0** means a direct-integration history. No vendor file
  has it.
- The index is taken as 1-based over all case records, deleted ones
  included. That holds in the five files scanned and is not documented.
- A starting-load record with load index 0 is an empty row.

Seismic eccentricity:

- **Sign.** The manual prints "+5%" and "−5%" patterns and no sense. Here a
  positive eccentricity moves the line of action toward +Y for an X load and
  toward +X for a Y load: the couple about Z is `−e·F` for X and `+e·F` for
  Y.
- **Where the moment is applied.** The manual says about the diaphragm's
  centre of mass. It is added at the first joint of the diaphragm whose
  rotation about Z is free; the diaphragm ties the rotations.
- Overwrites live in the model per pattern and diaphragm, not in the
  pattern's definition. The S2K reader checks only that the constraint
  exists; the solver checks its type.

Ritz:

- **The switch between the two projections** is a roundoff estimate of the
  projected stiffness of 1e-9 of its lowest eigenvalue. The manual has no
  such number. The cap at the number of mass degrees of freedom is the
  manual's (p. 373). The two forms are kept after an attempt to replace
  them by one (D50, under "Found and not changed").

Windows and video:

- **Automatic scale**: 10 % of the bounding-box diagonal; a largest
  translation below 1e-12 of the diagonal gives factor 1. The help gives no
  number.
- **Title wording** and the number formats in it.
- **Object Information** opens only in the Undeformed display.
- **Grid Only** uses the units selector; the window has no units picker.
- **Final state** in the deformed-shape form is the last step.
- **Ten significant digits** in plane titles and the Set 2D View list.
- **Rebar material list**: the materials with a yield stress, a rebar-type
  one first; one choice serves longitudinal and confinement bars. The
  refusal names the rebar material when the section has one, otherwise the
  concrete.
- **Video.** The user gives cycles, frames per cycle and frames per second
  (the vendor's form asks for a duration and cycles). The mode number is
  1-based. A multi-step static case uses its load factors as the time axis.
  Blending follows the screen's (premultiplied, in gamma space). The
  default magnification of a multi-step video is the automatic scale of the
  step with the largest translation. The display options of the vendor's
  form and its compressor choice are not offered.
- **Number printing**: at least four decimals and five significant digits,
  no trailing zeros, scientific notation below 1e-6 and from 1e15.
- **Messages**: 200 kept; a message is marked as an error by keywords, and
  one that starts with "Opened", "Saved", "Loaded" and the like counts as
  information even with warnings in its text. A message replaced within one
  frame, or repeated unchanged, is listed once.
- **Plane of a planar model**: X-Z at the common Y is tried first, then X-Y,
  then Y-Z.
- **Pushover curve signs**: the horizontal axis is the monitored joint
  displacement and the vertical axis is minus the sum of the base reactions
  in the monitored direction, so both are negative for a push in a negative
  direction (the hand check saw −0.03 m and −284 kN). The help states no
  sign rule. No number was changed.

## Evidence

Test files are under `crates/<crate>/tests/` unless a source file is named.
"Opt-in" rows need the vendor corpus and skip without it. Twelve rows were
added to `structurax verify`: two for the rigid zones, two for the area
materials and eight for the combination pairs. 

Integrated checks of the whole wave: 1348 portable tests (4 slow ones ignored), 1383 with the local vendor corpus (8 ignored), 107 suites, strict workspace Clippy clean, 106/106 selected reference checks (`structurax verify`).

| Check | Reference | Tolerance |
| --- | --- | --- |
| Bar with rigid end zones: axial 1.543209877e-3 in, twist 8.0e-3 rad; rz = 0, 0.5, 1; non-prismatic 2.5e-3 in and 6.0e-3 rad | `PL/EA`, `TL/GJ` over the full length (p. 130) | 1e-12; two rows of `structurax verify` |
| Examples 6-006 and 6-007 (opt-in, Ritz entry removed): first three periods | published 0.438, 0.135, 0.074 s | 1 %: the three printed digits and the open remainder of T1, not a claim of closer agreement |
| Propped beam with a moment or a shear release at the offset end; no release | 0.0124137931 rad; 0.0372413793; rz kept (pp. 130-131) | 1e-12 |
| Loads on a frame with end offsets (`wave5_end_offset_loads.rs`): fixed-fixed beam of 144 with offsets 12, uniform load 1, rz = 0, 0.5, 1, with and without releases; an inclined member with unequal offsets; point loads on the arm and in the span; inclined self weight with a release | sum of the reactions 144; joint shear 72; joint moment `w·lf²/12 + w·lf·a/2 + w·a²/2`, and 792 with released ends; a point load on the arm gives the force and force × distance at the joint; `P·b²(3a + b)/lf³` for the span load | 1e-9 |
| Non-prismatic frame with end offsets (`wave5_nonprismatic_offsets.rs`): axial and torsional flexibility for rz = 0, 0.5, 1; the review's bar; tip deflections under a tip load and a span load; free thermal expansion; axial span load; self weight | 84/200 + 60/100 = 1.02; unit-load integrals written in the test; `α·ΔT·L` = 0.144; reactions 6.5714 and 3.4286; equilibrium with the centroid of the varying weight | 1e-12 for the flexibilities and the review's bar; 1e-9 to 1e-12 otherwise |
| Rigid-arm geometric stiffness (`wave5_rigid_arm_geometry.rs`): buckling of a pinned column with rigid arms; the same column without offsets | `4EI/(lf·(lf/3 + 2d))` = 2175; the arms as stiff elements; the continuum root as a lower bound; `12EI/L²` without arms | 1e-9 against the hand value; 1e-4 against the stiff elements |
| Stations of a frame with end offsets (`wave5_end_offset_stations.rs`): positions; a released end; a fixed beam; an offset at a free end whose joint rotates | first station at the offset, last at `L` minus the offset; M3 = 0 at the released face, face shear 60, midspan moment 1800; fixed-fixed closed forms; face deflection = joint deflection − arm × rotation | 1e-9; 1e-12 for the positions |
| Frames without end offsets after the three repairs (`wave5_p13_unchanged.rs`): twelve models (prismatic and non-prismatic, horizontal and inclined, fixed, released, free tip), three cases each: reactions, displacements, every station force and deflection | fingerprints of the bit patterns, recorded from the code before the repairs | bitwise |
| A subdivided frame that is a mechanism (`wave5_unstable_frame_name.rs`) | the message names the frame, its end joints and the nearer joint; a joint of the model keeps its name | exact text |
| Diaphragm, Body and Equal constraints with one joint's axes rotated | the same model without the rotation | `8·ε·penalty/k` ≈ 1e-9 (measured 5e-10 and 2e-10; the 8 is a chosen multiplier) |
| Direct history, load starting at 1 s | exactly 0 before; `1/(k + 4m/dt²)` at the first loaded step | exact; closed form |
| Singular torsion pair | null vector (1, 1); exact message | exact text |
| Sound cylinder at bending thickness 0.5, 0.8, 1.0 | hoop force `pR·cos(π/36)` = 59.77168 | 1e-6 |
| Scaled stiffness minimum of `[[2, −1], [−1, 2]]` (function not used by the solver) | 1/2 | closed form |
| Plane-strain patch: S11, S22, S33, S12; the plane-stress patch keeps its values and has no S33 | 1.634615, 1.25, 0.865385, 0.076923 by hand | 1e-11; one row of `structurax verify` (S33) |
| Plane-strain anti-plane shear: element forces; a model end to end | nodal forces by the divergence theorem; `u3 = 2.6e-3`, `S33 = ν·S11 = 0.3` | 1e-12; 1e-9 |
| Recovery law with unequal modifiers | the constitutive matrix is symmetric: F11 from ε22 equals F22 from ε11, and the same for the moments | 1e-12 |
| Orthotropic membrane and bending at zero angle; model B with modifiers equals model A | hand matrices; algebra (`ν = √0.08`, m22 = 0.5) | 1e-9 |
| Orthotropic material at 30°: recovery and stiffness nodal forces; bending and shear | rotated stiffness of the laminate texts, F11 = 5.351413 | 1e-9; 1e-8 for the nodal forces; one row of `structurax verify` |
| Isotropic material under any material angle | invariance (p. 193) | 1e-12 |
| Plane section under a rigid rotation | zero force (was 2.9e-4 with the drilling term) | exact |
| Example 2-015 (opt-in): centre deflection, models A thin, thick and 16 × 32 | published −1.077, −1.074, −1.070; closed form −1.069195 | 1 % of the published value; 1.5 % of the closed form for the thin model; A against B 1e-3 (measured 1.6e-4) |
| Example 3-002 without incompatible modes (opt-in), four meshes, three loads | published Ux 3.000e-5; Uz 0.0101, 0.0030, 0.0038, 0.1068; Ux 0.840e-4, 0.206e-4, 0.282e-4, 8.958e-4 | 1e-9 relative for the axial value; half a unit of the last printed digit for the others |
| Combination pairs: COMB1, COMB2, WIND, GRAVEQ, GRAVWIN, SEVERE; Range; Absolute; a negative factor on a spectrum case | the manual's worked numbers (p. 353) | exact arithmetic; 1e-9 through the eigen solve; eight rows of `structurax verify` |
| Additive combination of single-valued cases | the loop before the wave | bitwise |
| Envelope of a frame with automatic subdivision | Max and Min rows carry the object's stations | exact |
| API and tables: Max then Min for joints, base reactions and frame stations (10 / −4 at every station of a bar) | the pair rule | exact |
| Shells form on a multi-valued result | +3 / −5 shows −5; a tie shows the Max row | exact |
| Design: column D = −10 kip, spectrum 7 kip; Envelope; Range; SRSS; Absolute | ratio of the same column under static 17, 10, 17, √149 and 17 kip | equal ratios |
| Design by states and permutations (`sap-design/tests/wave5_design_states.rs`, eleven tests): the review's column, with synthesized and with stored rows; Envelope, Range, SRSS, Absolute; a Range inside an additive combination; Eurocode 3 with shear and moment; a concrete column with the eight permutations of P, M2, M3, and as an Envelope | the largest ratio over the static cases with the hand-enumerated force sets, for the review's column those of (−14, 180), (−26, 20), (−14, 20), (−26, 180) | 1e-12 relative. Seven of the eleven fail on the tree before the repair, by the package's run |
| Cb per state: a Min row with a V-shaped diagram, a Max row with a uniform one; equal moments in both rows | AISC F1-1 by hand: 1250/690 for the Min row (the two rows read as one list give 1250/450); 1000/880 | 1e-12 relative |
| Design with stations between the support faces (`sap-design/tests/wave5_p17_offset_stations.rs`, eight tests): Cb of a member of 120 with offsets 10 and 20; Cb between stations; unbraced ratio 0.5; a steel member governed by lateral-torsional buckling; concrete beams of special and intermediate frames; members without offsets; the design length | AISC F1-1 by hand: 500/440, 500/380, 500/300 per segment; the ratio of the plain list of the clear length and of a run with Cb overwritten to the hand value; floors `0.5 × 3000` and `3000/3` at the last station, equal to an ordinary-frame design carrying those moments; bit patterns printed before the change; `L − oi − oj` | 1e-12 relative; bitwise. Six of the eight fail on the tree before the repair, by the package's run |
| What is not permuted: AISC shear; a concrete beam under special-frame floors; single-valued combinations and plain cases | the largest over the states; bit patterns printed before the change | equal; bitwise |
| Hinge kink: one object and two objects | 0.0091666667, 0.0333333333, 0.1141666667, 0.2 | 1e-9 absolute |
| Concrete design: rebar-material route against the concrete-fy route; required area at 60 and 75 ksi | equal; ratio 1.2 to 1.26 from `As = Mu/(φ·fy·(d − a/2))` | bitwise; the stated range |
| Seismic eccentricity: X load, width 6, M = 0.3 F; Y load, width 10, M = 0.5 F; ratio −0.05; overwrite 0.4 gives 0.4 F; reaction Mz; no diaphragm | manual formula with F from ASCE 7-10 12.8 written out (F = 26.1511); statics | 1e-7 of F × width (measured 5.6e-11); base shear 1e-7; bitwise without a diaphragm |
| The same model through an S2K save and reopen | the model before the save | 1e-9 F |
| S2K overwrite table: round trip, order, absent when empty, distance 400 after a change to mm | the written rows | exact |
| Ritz vectors of a 12-sided cylinder, 10 and 20 vectors (D12) | `ΦᵀMΦ = I` (p. 372) | 1e-10 (measured 8.9e-16 and 1.2e-15); nothing else is asserted |
| Ritz against an independent solution: space frame with stiff floor loops, grid with stiff beams, stack, portal, shear building; below, at and above the number of mass degrees of freedom | element stiffness written in the test, static condensation, Jacobi; shear chain in closed form | no Ritz ω² below the exact one, complete basis equal to the exact spectrum, both to `100·ε·κ`; at most min(n, rank) modes |
| Native modal history: index 0, out of range, negative, deleted, not Modal, periodic, unknown motion, FNA | the rule stated above | exact |
| Native round trip of the modal-case index | the index in the stream bytes | exact |
| Example 6-001 (opt-in, ramp samples rebuilt from the problem statement): MHISTA and MHISTB at 1 s and 4 s | published 0.34718, 2.0000, 0.69436, 0.74030; closed form of the ramp response with the file's link mass (period 3.141600508, not π) | 5e-6; 1e-9 against the closed form with the link mass, 2e-5 against the idealised one |
| Example 1-022 (opt-in): MHIST1 imports its modal case and damping 0.05 | native record | exact |
| I section with unequal flanges: area and inertias, through the function and through the dialog | closed form of three rectangles | 1e-9 |
| Automatic scale: drawn largest translation | 10 % of the diagonal | 1e-15 |
| Load lines of Object Information | the stored records; N, mm and ksi conversions | exact factors |
| AVI of 3 × 2 px, 2 frames; BMP | a file built byte by byte from the format description (328 bytes) | exact |
| AVI sizes: 8 × 6 × 3 frames = 736 bytes, 7 × 5 × 2 = 520; every chunk size against its payload | layout arithmetic | exact |
| AVI read back by `ffprobe` and `ffmpeg` (skipped when they are absent) | 9 × 7, 4 frames, rate 12, pixels of frames 0 and 3 | exact |
| Rasteriser: two triangles of a square cover 45 and 55 pixels without overlap; opaque and premultiplied colours; blend (128, 0, 0, 128) over (0, 0, 255) = (128, 0, 127); vertex colours; clip; font coverage 0.5 gives alpha 174 | top-left rule; premultiplied blending; barycentric weights; `0.5^0.55 × 255` | exact |
| Recorded frames: a square driven by `sin(phase)` | x = 12, 20, 4 in frames 0, 1, 3, read from the file | exact |
| A video through the menu path: 5 frames of a history | file length of the layout; frames differ | exact |
| Dome template, undeformed and first mode, at 1440 × 900 | every tessellated vertex of a non-text shape inside the joints' box + 40 px (the old drawing call put a vertex at x = 1150 against a limit of 1026) | the stated margin |
| The nine window repairs and the rewritten tests | the behaviour stated above | exact; the package reports that each test fails when its behaviour is broken by a local edit |
| Explorer line of a section in kN, m and in kip, in; a joint coordinate 0.05 (`wave5_hand_check_tests.rs`) | `I33 = b·h³/12`, converted with 0.0254⁴; five significant digits in any unit | exact text |
| Open: an S2K saved and reopened through the command; a plane that the new grid has and one it has not; an unreadable file; a planar model; the 2D frame template against a 3D template (`p16_open_tests.rs`) | results, lock and display reset, "Unlocked" painted; the plane kept or dropped; nothing changed; the pane on the model's plane | exact |
| Messages (`p16_messages_tests.rs`): a refused analysis with a long reason; 200 and more messages | the status line is cut, the tooltip and the window entry are whole, the entry is marked as an error, Copy gives the text; newest first, at most 200 | exact |
| Windows at 1440 × 900: all seven tabs of Model Workflows and the seismic window; the pushover plot with a negative curve; the title of a 700 px pane; the duration of a function from a file; the video magnification (`p16_misc_tests.rs`) | inside the screen and scrollable; ticks at multiples of 1, 2, 5 and every label inside the plot; the title wraps inside the pane; "39.98 s"; the automatic factor until the box is edited | exact; geometry of the painted shapes |
| A 4 m cantilever with a 0.5 m rigid offset at the support, drawn (`p16_offset_tests.rs`) | the arm runs from the joint to the face, the curve ends at the displaced tip, the moment diagram starts at the face | positions of the painted shapes |
| Refusals above | error text | exact |

## Found and not changed

Every item of this section has a ledger id in [tutorial-video-progress.md](tutorial-video-progress.md); those that had none when the packages reported them are D60 to D67 there.

- **Mechanisms that pass the pivot test are still accepted (D03b).** The
  wave was to refuse them by the smallest eigenvalue of the stiffness scaled
  by its structural diagonal. That quantity was measured on 968 static, modal
  and buckling cases of the portable and corpus runs before enabling
  anything. Smallest value of a sound model: 1.66e-12 (the solid beams of
  Examples 5-002 and 5-005), then 1.82e-12 and 2.2e-11. Mechanisms without
  penalty constraints: at most 1.7e-16 in magnitude (the fourth wave's
  cylinder at 0.5 and 0.8 gives −1.94e-17 and −1.08e-17), a gap of about four
  orders. Mechanisms held by a diaphragm penalty: −1.06e-8, −1.16e-9,
  −1.57e-8 and +4.53e-7, **above** the smallest sound model. No threshold
  separates the two, so the detection was not enabled. The function
  (`linalg::scaled_stiffness_minimum`) is written, has a closed-form test and
  is called nowhere. Detection waits for exact constraints (wave 6).
- **Examples 6-006 and 6-007 (D06).** With the rigid-zone and Ritz repairs
  and the starting-load entry removed, T2 and T3 are inside the rounding of
  the published values and T1 = 0.4373 s is 0.0002 s below it. That
  remainder is not explained. The open candidate is the vendor's starting
  vectors for the link degrees of freedom (D36), which this solver does not
  have. Other variants were measured and none adopted: rigid-zone factor 0
  or 1 (+25 %, −14 %), shear deformation off, link mass omitted, diaphragms
  replaced by stiff beams.
- **Axial, shear and torsion hinges (D09b)**: their jumps are not added to
  the station deflections.
- **Frames with end offsets, not done** (reported by the package that
  repaired D46 to D48; no ledger identifier yet): the manual's rule that reduces the offsets so that
  the clear length is at least 1 % of the length (p. 129; the code only
  clamps at 1e-6 of the length); the mass of non-prismatic members and of
  the end zones (p. 134; the mass still uses the section at mid length); the
  geometric stiffness of the flexible part still uses the flexible length
  for bending only.
- **Static participation of a Ritz starting load on a stiff model (D50).**
  In the flexibility form of the projection (D37) the frequency is `1/μ`,
  not `φᵀKφ`, so the static participation of a starting load is not exactly
  100 % in that branch. The second review saw 1.0029 and 1.0099 with the
  branch forced on; under the program's own switch it found at most 3e-8.
  The repair it proposed, one form that carries `K·b` through the
  orthogonalisation, was built and measured and **rejected**: the rounding
  of the orthogonalisation is not mirrored in the carried product, and the
  bound tests of D37 failed again (the projected stiffness went indefinite
  on the stiff models, and even a soft model missed the exact spectrum by
  1.7e-7 at a complete basis). The two forms stay. Measured with them, not
  asserted: exactly 100 % on a soft model; off by at most 1.6e-8 at an axial
  modifier of 1e6 and by at most 7.2e-6 at 1e9. The switch between the
  forms remains a discontinuity. The flexibility form has the accuracy of
  the Eigen solver (about 1e-3 relative for a stiffness ratio of 1e9 in the
  test models), not better.
- **Design, untested or not done.** The permutation of T, V2 and V3 for the
  closed-section torsion interaction has no test; it uses the mechanism the
  Eurocode test covers. The manuals' sub-combinations per time step of a
  history and the "corresponding values" option of moving loads are not
  implemented. No ledger identifier yet.
- **API readers other than the result tables** (the cable force and
  similar) return only the Max row of a multi-valued combination.
- **Response-spectrum cases have no area results (D57).** The solver stores
  none, so a combination with such a case has none either, and the Shells
  form refuses it with "no shell results" before the rule for multi-valued
  results is reached. Seen in the hand check.
- **Result-table cells have a fixed number of decimals (D58):** a
  displacement of 1e-5 m reads "-0.00001". A scientific fallback would also
  print roundoff such as 1e-17, so the repairing package left it.
- **An open Shells form keeps its old overlay after Unlock (D59).**
- **Orthotropic solids (D43).** The solid element builds its compliance with
  `−u12/e1`, `−u13/e1`, `−u23/e2`; the manual's equation as printed, which
  the area element now follows and Example 2-015 supports, has `−u12/e2`,
  `−u13/e3`, `−u23/e3`. It matters for solids with unequal moduli.
- **Von Mises of a plane-strain area (D44)** ignores S33. The display of
  plane sections is not done (G43), so it is not shown.
- **Example 2-015, model B.** The file's isotropic section stores the
  modifiers m11 = 1.0, m22 = 0.5, m12 = 0.5645; the problem text says
  m11 = 0.5, m22 = 1.0. With the modifiers as stored, model B has the
  stiffness of model A (they agree to 1.6e-4) and the test of 2-015b passes
  unchanged. Whether the importer reads the modifier slots in the right order
  is not verified.
- **Plain plane elements against Example 3-004.** With the edge pressure
  replaced by nodal forces and incompatible modes off, the radial
  displacement is 2 to 3 % below the published value (0.004450 for 0.004539;
  0.004895 and 0.004944 for 0.005036 and 0.005040). Measured, not asserted;
  the vendor's element has incompatible modes (G37). As imported the files
  stay refused.
- **Fit of the view.** The fit takes 0.62 of the pane width for the 3D
  diagonal, so the dome fills about 45 % of a pane.
- **Start display.** The program starts with its built-in model analysed
  and locked, in a frame-diagram display, so a right-click on a frame opens
  the diagram inspector until the Undeformed display is chosen. A new or
  opened model starts in the Undeformed display (D53).
- **Seen in the hand check and left as they are:** the quick-draw commands
  of the Build window take index ranges (the mouse tools are in the Draw
  menu on a plane view); a second quick draw over existing members reports
  counts without saying which already existed; a Ritz case with 99 %
  targets stopped at 4 vectors where the default Eigen case has 12 modes, so
  the two cases offer different mode counts; *Define > Load Patterns* opens
  the Model Workflows window; the Display menu has "Show Deformed Shape
  (F6)" and "Show Deformed Shape..."; the automatic seismic form lists only
  existing Quake patterns; dialogs open over the 3D pane.
- **Library concretes in the windows.** They carry no yield stress, so
  concrete design is refused until the section names a rebar material.
- **Cyclic video of a plain deformed shape** (the vendor offers it) is not
  offered; only modes of a modal case.
- **Native persistence** of the eccentricity overwrites (G19).
- A Ritz case with no starting loads uses an older routine that was not
  changed.
- **Ritz vectors of an axisymmetric model (D12)** are closed as expected
  behaviour by measurement: the vectors are orthonormal to 1e-15; the
  content of other circumferential harmonics grows by a factor of 1e2 to
  1e3 per generation cycle (97 to 1354 measured, over eleven orders in seven
  cycles); the softer harmonics n ≥ 2 explain the direction of the growth.
  The factor is not constant, and only the orthonormality is asserted.
- Not tested: an error raised by the recorder itself after the plan was
  accepted (the tested errors come from the plan and from the file write);
  the reversal of local 3 through an analysis; S33 in a combination; the
  font-atlas protocol with a real application's incremental glyph patches;
  the station values of Examples 1-010b to d (only their tip deflection);
  the clipboard behind the Copy button of the Messages window on a real
  display. S33 of an area is stored and has no reader: no table, no API
  call and no display shows it (G43).
- Unverified: the meaning of native motion types other than 1 and 2 (only
  those occur). Example 6-001 MHISTC (damping 0.999) runs and gives 0.344809
  and 1.968550; the problem prints no value for it, so nothing is asserted.

## Review coverage

Two independent reviews were made by a separate reviewer model that had not
written the code. Each confirmed a finding with a scratch test before
reporting it, and each states what it read.

**The window code of waves 3 and 4** (D19), at the commit of the fourth
wave. Its coverage and findings are under "Window defects found by reading
and by looking" above.

**This wave's result-changing code**, on the integrated tree before the
last repairs. Coverage, as its report states it:

- Read completely, as the wave's diff: `frame.rs` (the whole file), the
  changed parts of the solver's `lib.rs`, `area.rs`, `combos.rs`, the
  model's `results.rs`, `ritz_modal.rs`, `linalg.rs`, `hinge.rs`,
  `auto_seismic.rs`, the design crate's `steel.rs` and `concrete.rs`, the
  API's `lib.rs` and `database_tables.rs`, `shell_output.rs`, the native
  reader, writer and export guard, `s2k.rs`, `radial.rs`, the seismic
  window, and the small diffs of `dynamics.rs`, `nonlinear.rs` and
  `frame_mesh.rs`.
- Sampled: six of the wave's test files and the three files of
  verification rows. The tests were not audited one by one. The result
  tables and `gui.rs` were searched for the Max and Min handling only.
- Not reached: the video rasteriser and AVI writer, Object Information, the
  scale and title code, the rest of the windows, and the packages' reports.

What it found:

- Six confirmed defects: loads on rigid arms (D46), two on non-prismatic
  members with end offsets (D47), the stations of a released end (D48),
  design of the two rows as one station list (D49, with D42 as a part), and
  two files of verification rows that returned no rows when their analysis
  failed, so that they could not fail by an error (D51).
- Four suspicions. Three were confirmed by the repairing packages and
  repaired: concrete design read the same rows as steel design; rigid arms
  had no geometric stiffness; the unstable-joint message could name an
  internal joint of a subdivision. The fourth is the Ritz static
  participation (D50), which stays open.
- No defect, by reading and in part by scratch runs: the element without
  offsets (bitwise unchanged), the area law (rotation direction, shear
  factors, plane-strain reduction, S33, thermal load; material and area
  angles that must cancel do, to 1e-9), the Ritz modes (orthonormal to 9e-16
  in both forms; with the stiffness form forced the wave's bound test
  fails, so that test does exercise the repair), the constraint rotation,
  the time-function rule, the combination and result code, the hinge kink
  sign, the eccentric moment, the rebar-yield refusal, and the native and
  S2K changes.

**What the integrating reviewer read.** The combination rule
(`sap_model::results::combo_step`, against the manual's five rules) and the
change of the Ritz projection; and the reviewer checked that the scaffold's
move of the combination code was verbatim. Everything else was integrated
on tests, the packages' reports, the gates and the two reviews.

**Not reviewed again line by line:** the repairs that followed the second
review (loads, non-prismatic members, arm geometric stiffness and stations
of frames with end offsets; design by states and permutations) and the
repairs that followed the hand check (the window package, and the design
code that reads the moved stations; that last package also audited every
other use of station positions in the design crate and reports no further
one that assumes stations at the joints). They rest on their tests, their
reports and the gates.

The repair of D51 has no test of its own: both files now stop with an error
where they returned no rows, as the 31 older benchmarks do.

## End-to-end

`crates/sap-app/src/gui/video_wave5_e2e.rs` and the files under
`video_wave5_e2e/`: seventeen tests, fourteen portable and three opt-in.
They drive the application as the earlier end-to-end tests do: headless,
with synthetic pointer and key input, and they read what is painted. They
were written on the tree before the repairs of the second review and of the
hand check, and found no product defect. The package reports that it broke
the product locally in twelve ways (a wrong pose scale, the width taken
from the wrong axis, the bottom flange ignored, the minimum computed as the
maximum, the material angle forced to zero, and seven more) and that each
of the fourteen portable tests failed under the edit aimed at it; the three
vendor tests were not broken, by reasoning.

**Rows of the video.**

| Row | What is done through the windows | Reference | Tolerance |
| --- | --- | --- | --- |
| 1.1 Grid Only | the units selector, then the template entry and the Quick Grid Lines form by typing; a spacing of −6 | four lines per axis at multiples of 6, 6 and 3.5; no objects; the refusal names the spacing and the grid is unchanged | exact |
| 1.16 eccentricity | two storeys with diaphragms; ratios 0.05 and −0.05 for an X load, 0.05 for a Y load, in the seismic window | storey forces from ASCE 7-10 equations 12.8-2, 12.8-3, 12.8-5, 12.8-11 and 12.8-12 written out in the test; moment = ratio × perpendicular width (6 and 10) × force; the base moment about Z changes by minus the sum of the moments and the other reactions do not change | 5e-5 on the painted lines (four decimals); 1e-7 of force × width for the statics |
| 1.17 load read-out | a right-click on a joint, a beam and a slab | the assigned values in kN, m; in N, mm; in kip, ft (1 kip = 4.4482216152605 kN, 1 ft = 0.3048 m) | exact factors; 5e-7 for the six printed decimals |
| 3.2 unequal flanges | a section made in the dialog (top 0.10 × 0.010, bottom 0.06 × 0.005, web 0.004, depth 0.20 m) on a cantilever | area 2.04e-3 m² and I33 by hand; tip deflection `P·L³/(3EI) + P·L/(G·As2)` | 1e-9 |
| 3.8 multi-step video | a 501-step modal history, 0 to 2 s every 0.2 s, 160 × 120, 10 frames per second | the form states "11 frames, 1.10 s"; eleven frame chunks of 57600 bytes; the size field equals the file length less 8; `ffprobe` and `ffmpeg` read the same size, count, rate and pixels; in every frame the marker of the joint is where the view projects `base + magnification × u(t)`, with u(t) the closed form of the damped single mass | exact for the layout; 1 px for the marker (worst 0.534 px) |
| 3.8 cyclic video | mode 1, two cycles of twelve frames | 24 frames; frame k and frame k + 12 are identical byte for byte; the marker at `base + magnification × 0.02 × sin(phase)`, 0.02 being the mass-normalised amplitude by hand | exact; 1 px (worst 0.503 px) |

**Repairs seen through the application.**

| What | Reference | Tolerance |
| --- | --- | --- |
| A combination with a response-spectrum case in the Database Tables window: static 3.5, spectrum 2.0 (flat spectrum, `Sa/ω²`) | Max 5.5 and Min 1.5 for the additive combination; Max 3.5 and Min −2.0 for the envelope (Analysis Reference p. 353) | 5e-6 (five printed decimals) |
| A bar free to twist, through *Analyze > Run Analysis* | the status line names the joint and "R1"; no results; with the restraint added, `P·L/(E·A)` = 1e-5 | exact text; 1e-9 |
| A plane-strain patch of one element | `2(1 − ν²)p/E`, `−ν(1 + ν)p/E`, `S33 = ν·p` | 1e-9 |
| An orthotropic area at material angles 0° and 30° | displacements from the rotated compliance; stored resultants (6.52174, 1.30435, 0) and (5.35141, 1.65946, 0.91102); the legend of the Shells form shows 6.522 and 5.351 | 1e-9; 1e-8 |
| A bar with 30 in offsets at both ends, rz = 1, 1000 kip, in the Joint Displacements table | 0.15432 = `P·L/(E·A)` over the full length (0.09000 over the flexible length) | the five printed decimals |
| Concrete design of a section made in the dialog | with a rebar material the design completes and the top steel is at least 5.29e-4 m², a lower bound from `Mu ≥ 90 kN·m`, φ = 0.9, fy = 420 MPa, d ≤ 0.45 m; without one the error names the frame, the section, the concrete and the yield stress | the bound; exact text |
| An eccentricity overwrite of 0.4 saved as S2K through the file window, cleared and reopened | the overwrite is back; after an analysis the first diaphragm has `−0.4 F` and the second `−0.3 F` | 5e-5 |

**Vendor files** (opt-in; opened through the file window).

| File | What is read | Reference | Tolerance |
| --- | --- | --- | --- |
| Example 6-001, ramp samples attached as data | the plot window, joint 2, U3 | published 0.34718 and 2.0000 (MHISTA at 1 s and 4 s), 0.69436 and 0.74030 (MHISTB); the closed form of the ramp response | 5e-6; 2e-5 relative |
| Example 2-015a, thin | the read-out of the centre joint on the deformed shape: −1.069063 | published −1.077; closed form −1.069195 | 1 % (measured 0.74 %); 1.5 % |
| Example 6-010 | the status line after *Run Analysis* | the refusal names the case 'RITZ' and the Ritz starting load of type 4; no results | exact wording |

Not covered end to end, by the package's own list: the table that names the
case behind an empty Area tab, and the Shells form on a multi-valued
combination (both are covered by the window tests only); the periodic
refusal of Example 1-015 and the other four refused Modal examples; loads
made in the Assign window for the read-out (they are set as model data). Set
as state, not by clicks: the fields of the section dialog, the active case
and table tab, the typed path of the file window, the 3D projection used
for the video, the ramp samples.

The package also looked at one decoded frame of the multi-step video (the
joint labels overlap and the view title was cut at the frame edge; the
title now wraps) and at the seismic window of the real program (its action
button was below the fold; D55).

## Hand check

"By hand" here means: an agent started the real program (a debug build) on a
private virtual display of 1440 × 900, sent it synthetic clicks, keys and
drags, and read screenshots. No person used the program, and nothing was
drawn on the user's display. No screenshot is kept in the repository. What
follows is what the two reports say was seen.

**What was gone through.** The four parts of the video, on the tree before
the repairs of the second review, in two passes. The report states a limit
of its own evidence: screenshots of the first pass were lost from the
agent's context, so its first-pass items rest on notes written at the time.
That concerns the second and third parts and the pushover and video items.

- *Part 1, the steel building*, went furthest. Grid Only, units, a quick
  grid of 6 × 4 × 7 lines, a plan view chosen from the level list, beams,
  columns, braces and areas by index range (168 joints, 382 frames, 90
  areas), selection by section, a diaphragm per level, restraints, the mass
  source, the automatic seismic form, an area uniform load, a Ritz case
  with targets, the run, the first mode with its title, the modal tables,
  the import of 166 shapes from a section file and their assignment, the
  default design combinations, steel design of the 382 members, Object
  Information on an area, and the joint read-out.
- *Part 2, the pushover*: the 2D frame template on a custom grid, a section
  with reinforcement, a hinge property and its assignment, chained nonlinear
  cases with displacement control, the hinge states by step and the
  pushover curve. A target of 0.03 m ran; a target of 0.3 m was refused.
- *Part 3, the time history*: the 3D template, a function from a file of
  2000 points, a modal history, the plot window with a named set and its
  extremes, and a multi-step video (read back with `ffprobe`: 320 × 240, 10
  frames per second, 11 frames that differ).
- *Part 4, the tower*: joints by coordinates, a curved frame, the radial
  extrusion to 288 areas, the reversal of local 3, a material from the
  library, Ritz modes, the modal tables, the shell stress S11 in N/mm² with
  its legend of eleven ticks, and the joint read-out.
- Save as S2K and reopen.

No crash and no hang in either pass. Every window that was opened responded
and closed.

**What it found.** The four parts can be driven through the windows. The
problems are D52 to D55 (above, under "What the hand check found"), and the
items listed under "Found and not changed" as seen and left. Two of its
reports were wrong or could not be confirmed: the section it took for one
without bending stiffness had a stiffness that was printed as "0.0" (D52),
and "Locked" after opening a file could not be reproduced (D53). It
reported the pushover curve with negative values; that is the sign of the
push (see the project choices).

**What the repairing package did and saw.** It repaired D45 and D52 to D56
and then ran the program again on a display of its own. Seen after the
repairs: the explorer line of a section with its inertia ("I33=0.000675");
a reopened building, unlocked and undeformed; the Messages window; the
seismic window and Model Workflows inside the screen; a vendor file with a
planar model opening as "X-Z Plane @ Y=0"; the pushover plot with ticks (on
a model with positive values); the video form with an automatic
magnification of 1.587 on a shell model and a cyclic video of 60 frames
with a visible mode shape; Example 1-010b with the deformed line starting
at the joint, the moment diagram starting at the face and 1380 at the root.

It also went through the checklist items the first agent had not reached,
and saw them working: the Max and Min rows of a combination with a
response-spectrum case in the tables; the cyclic video; the I-section
dialog with unequal flanges (the values it shows are rounded and were not
compared with the closed form on the display); Plane Up and Plane Down;
drawing with the mouse on a plan view (a frame by click, two frames by
window, an area by click); two vendor files opened through *Open* with a
typed path, with their import warnings listed. On the way it found D57,
D58 and D59.

**Never looked at**, by either agent:

- the arrow of an area's local 3 before and after the reversal;
- resizing the window and the panes (the display has no window manager);
- the file chooser behind *Browse...* (paths were typed);
- the face choice of the Shells form (Bottom, and Visible Face following
  the camera);
- the redraw of the plot window after a change of the time range, and the
  stepping of the animation through a history, by eye;
- the computed moment lines of the seismic window and the Z moment of the
  base reaction after a run of the first part's model (the end-to-end test
  checks both; an earlier package saw the moment line on its own model);
- the refusal of a derived shell quantity for a multi-valued combination:
  the form refuses such a combination earlier, for D57;
- the Max and Min curves of the frame diagram;
- radial replication of joints, the displacement-contour option of the
  deformed-shape form, the run of the verification suite from the Help
  menu, and the Copy button's clipboard.

Within the hand check, the mouse tools, the cyclic video and the I-section
dialog were seen by the second agent only. (Packages earlier in the wave
had looked at the cyclic form, one recorded frame and the I-section dialog
on models of their own; see "Looking at the real program".)

## Not done

The current list of everything open, blocked or partial is the ledger in
[tutorial-video-progress.md](tutorial-video-progress.md). Of this wave's own
items it holds: mechanism detection (D03b), the remainder of Examples 6-006
and 6-007 (D06), the jumps of axial, shear and torsion hinges (D09b), what
the hand check did not look at (D17), the orthotropic solid (D43), the von
Mises of plane-strain areas (D44), the Ritz static participation on stiff
models (D50), the area results of response-spectrum cases (D57), the digits
of the result tables (D58) and the stale Shells form after Unlock (D59).
Without an identifier, under "Found and not changed": the three leftovers of
the end-offset repairs, the untested torsion permutation and the two
sub-combination options of the design manuals, and the API readers that
return one row. The 2016 code editions remain absent from the corpus.
