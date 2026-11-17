# Features of a SAP2000 tutorial video, third wave: drawing and editing on plane views and the grid

Date: 2026-11-16. Scope note for one bounded set of additions. It does not
change the project status: vendor-runtime acceptance and broad parity remain
unestablished.

The first two waves ([tutorial-video-wave1.md](tutorial-video-wave1.md),
[tutorial-video-wave2.md](tutorial-video-wave2.md)) added the analysis
features of the video's four models. A user following the video still could
not build its models the way the presenter does: by picking a plan or an
elevation, drawing on the grid with the mouse, windowing objects, choosing a
material by region and grade and importing a family of shapes at once. This
wave adds that, plus the repair of defects in existing output that were found
while planning and integrating it. As before, the video only lists which
commands a user meets; no number in it is a reference.

What the vendor material supports was checked first:

- The command semantics are those of the vendor help topics (Set 2-D View,
  Quick Draw Frame/Area/Braces/Secondary Beams, Draw Poly and Rectangular
  Area, Snap To, Pointer/Window, Get Previous Selection, Divide Frames, Grid
  System Data, 2D Frames template, Define Materials, Import Frame Section
  Property). Where a topic is silent the choice made here is listed under
  "Project choices".
- The corpus holds **eight** material library files. Two were overlooked at
  first because the extraction truncated their names and dropped the
  extension: the United States and the New Zealand libraries. The video adds
  ASTM A36 from the United States library, so that case is covered.
- Coupled P-M2-M3 hinges were looked for again and stay out: no verification
  problem, and the reference manual leaves the interpolation of the curves,
  the energy adjustment and the growth of the surface open (pp. 154-155).

## What is supported now

### Plane views

- *View > Set 2D View*: an X-Y plan at a Z coordinate, or an X-Z or Y-Z
  elevation at a Y or X coordinate, in either pane; *Plane Up* and *Plane
  Down* step through the grid coordinates of that axis (the distinct joint
  coordinates when the axis has no grid line); *Whole Model* returns to the
  projection of everything. The pane header names the plane and the cursor's
  in-plane coordinates are shown.
- A plane view shows and picks only what lies in the plane: joints within the
  joint-merge tolerance (`1e-9 * max(1, largest |coordinate|)`), frames and
  links with every end in it, areas with every corner in it. Solid faces in
  the plane are drawn as outlines and cannot be picked. Only the front face of
  a plane is shown.
- The grid lines of both in-plane axes are drawn with their bubbles, and the
  view bounds include the grid, so a model that has only a grid is usable.

### Selection

- **Window selection.** Dragged left to right the window takes the objects
  fully inside it; dragged right to left (drawn dashed) also those it crosses
  (help topic "Pointer/Window"). Windows add to the selection, and only
  displayed objects are selectable.
- **Get Previous Selection**, in a new *Select* menu together with Select All
  and Clear Selection (moved from *Edit*).
- **Behaviour change.** In a 2D pane a primary drag now draws a window; it
  used to pan. Shift, Alt, middle or right drag pans. In the 3D pane a primary
  drag still rotates, Shift pans, and Ctrl+drag draws a window.

### Drawing with the mouse (*Draw* menu)

| Tool | Where | What a click or a window does |
| --- | --- | --- |
| Quick Draw Frame | plane view on a grid plane | click: the nearest grid segment (within 8 px); window: every grid segment with both ends inside |
| Quick Draw Area | plane view on a grid plane | click: the grid cell under the pointer; window: every cell fully inside |
| Quick Draw Braces | elevation | click or window grid cells; X, V or inverted V, pinned |
| Quick Draw Secondary Beams | plan | click or window grid cells; N beams per bay, pinned |
| Draw Frame through Points | any view | each click after the first adds a frame; Enter, a double click or a click on the same point ends the sequence |
| Draw Rectangular Area | plane view | two opposite corners |
| Draw Poly Area | any view | three or four points; Enter or a double click closes it |

- A click snaps to a joint of the plane within 10 px, else to a grid
  intersection within 10 px, else (plane views only) it is a free point of the
  plane. **A snapped point takes the stored coordinate** of the joint or grid
  line; screen positions are single precision and are never written into the
  model for a snapped point. In the 3D view and in whole-model 2D views the
  point tools pick joints only and the quick and rectangle tools are refused.
- Areas drawn in a plane view have local axis 3 toward the viewer: +Z in a
  plan, −Y in an X-Z elevation, +X in a Y-Z elevation.
- The tools run the model commands `draw_frames` and `draw_areas` (new) and
  the first wave's `quick_draw_*` commands. They are staged: a refused command
  leaves the model unchanged, a locked model is refused, and an accepted one
  clears stale results. An area must pass the solver's own geometry criterion
  (three or four distinct corners, convex, not folded); a test holds the two
  implementations to the same outcome.
- A floating properties window holds the section, pinned ends, brace kind and
  the number and direction of secondary beams. Escape leaves the tool.

### Grid, template, divide and break

- **Grid line editor** (*Edit > Edit Grid Lines*): the lines of one axis as
  "id coordinate" rows or as spacings from a first coordinate. Refused:
  repeated or blank ids, coordinates within tolerance of each other, more than
  1000 lines.
- **2D frame template on a custom grid** (New Model dialog, "Use custom grid
  spacing"): bay widths and storey heights as lists. With equal spacings it
  gives the joints and frames of the existing template bit for bit. The origin
  is at the first grid line. Grid ids continue AA, AB after Z (the equal-bay
  template repeats A; that is an existing defect of it and is left alone).
- **Divide with a length ratio**: pieces in a geometric series with a given
  last/first ratio (help topic "Divide Frames"); joints at
  `(q^k − 1)/(q^n − 1)` with `q = r^(1/(n−1))`. The equal division is
  unchanged bit for bit.
- **Break at intersections** of the selected frames with selected joints and
  with each other, including T-junctions (only the crossed frame breaks).
- **Area uniform load by direction** in the Loads tab: Gravity, Global X, Y,
  Z, Local 1 to 3. The projected directions are not offered; the solver
  refuses them.

### Material library

- *Define > Materials from Library*: region, type, standard and grade from
  the vendor's `CSiMaterialLibrary*.xml` files, read at run time from a folder
  that is searched like the section files or chosen with Browse. Nothing from
  those files ships with the program.
- The eight corpus files hold 287 materials (China 76, Europe 18, India 16,
  Italy 21, New Zealand 33, Russia 74, Spain 13, United States 36); 278
  convert. E, Poisson's ratio, thermal expansion, weight density and mass
  density are taken from the file each with its own unit factor (weight and
  mass are never derived from one another: the files' ratio is not always g);
  `G = E/(2(1+ν))`.
- The reader is a small XML subset reader written for these files. A file it
  cannot read is an error naming the file and the line, not a file silently
  skipped. Later files replace earlier entries with the same region, standard,
  type and grade, as the help topic says.
- Besides `CSiMaterialLibrary*.xml` the folder reader accepts a file whose
  name starts with `csimateriallibrary` and has no extension, because that is
  what extracting the installer leaves of the two long names.

### Sections and files

- *Define > Import Sections from .PRO (multiple)*: a list with click,
  Ctrl+click, Shift+click and drag selection, a shape-type filter and a name
  filter; the import is all or nothing; one button makes an Auto Select list
  of what was imported.
- **A file chooser inside the program** (no operating-system dialog and no new
  dependency) behind Browse buttons for the model file, section file, function
  file and material library folder.

### Defects of existing output that were repaired

These change numbers that earlier versions reported.

| Defect | Before | Now | Reference |
| --- | --- | --- | --- |
| Shell "top" stresses (`s11_top`, `s22_top`, `s12_top`, `svm_top`; the GUI's "SVM Top" contour; API `area_force_shell`) | `F/t + 6M/t²`, which is the bottom face | `F/t − 6M/t²` | Analysis Reference Eqns. 2; cantilever strip: top stress +1750 kN/m² with M11 = −10; Example 2-015 |
| Shell stress resultants under a temperature load | the free thermal strain was counted as stress: a freely expanding plate showed stress, a restrained one showed none | the thermal part is removed | Example 2-013: 0 and −733.33 lb/in² |
| Shell temperature gradient: sign of the load | a plate curled toward its warm face | away from it | closed form of a free plate; Example 2-014: Uz +0.019 in |
| Shell temperature gradient in native files (load type 3) | imported, then ignored by the analysis without a message: all results zero | imported as the gradient | Example 2-014: M11 −2.182 k-in/in |
| Temperature load on a three-joint area, with a joint pattern, of an unknown type, or on a section without the stiffness it acts on (a uniform change on a plate, a gradient on a membrane, either on a plane or axisymmetric-solid section) | ignored without a message, or applied to degrees of freedom without stiffness | refused | negative tests |
| Von Mises and principal stresses of a linear combination | combined as if they were linear | recomputed from the combined components | 1.2D + 1.6L of the strip |
| Von Mises of an automatically meshed area | average of the sub-elements' values | from the averaged components | identity |
| Combination with a case that has no shell, solid or link results (response spectrum, modal, buckling) | reported the other cases' values as the combination | reports none for that object | identity |
| Base reaction of a model whose area mesh creates restrained joints | left out the reactions at those joints (6 of 24 kN in the test) | the whole reaction | statics |
| Response-spectrum direction that is not U1, U2 or U3 | analyzed as U1 | refused; rotational directions refused by name | negative tests |

The moments themselves already had the vendor's sign (positive: tension at the
bottom) and are unchanged, except under temperature loads as listed.

## What is refused

Each of these has a negative test.

- Quick tools outside a plane view or on a plane that is not a grid plane;
  braces in a plan; secondary beams in an elevation; a rectangle in the 3D
  view.
- An area with more than four corners (solver limit), with a repeated corner,
  folded or degenerate, or on the joints of an existing area; a frame of zero
  length; a missing section; any drawing or editing command on a locked model.
- Breaking with selected areas, solids or links; frames that overlap along a
  common line; a frame the divide command refuses (hinges, end offsets,
  cardinal point, non-prismatic section, cable, tension/compression limits).
- Library materials: lightweight concrete and a shear strength reduction
  factor other than 1 (the concrete design would ignore them), aluminum and
  cold-formed steel, an entry without E, ν, α, a density or its strength, a
  unit pair with no unit system here. Nine of the 287 corpus entries.
- Shell temperature loads as listed above; unknown or rotational response
  spectrum directions; an S2K response-spectrum row with an unknown
  `CoordDir` (a missing one still means U1).

## Project choices the sources do not settle

- A Quick Draw Frame window takes the segments with both ends inside, in
  either drag direction (the help documents windows for areas only). In an
  elevation that includes the segments along the lowest Z line.
- Snap radius 10 px; segment pick radius 8 px; ties go to the horizontal axis,
  then the lower grid index.
- Quick Draw Area skips cells that already have an area.
- The previous selection is the last non-empty selection that was cleared or
  replaced by one with no object in common; restoring it makes the selection
  it replaced the previous one.
- Plane Up from no plane goes to the first level, Plane Down to the last.
  Changing a pane's projection clears its plane.
- The 2D template on a custom grid behaves like the other templates where the
  help is silent: it replaces the model even when that is locked, and a
  section name that is not defined falls back to the first defined section.
- Library steel is coloured blue and concrete green. A library concrete has no
  rebar yield stress: concrete design takes it from the section's rebar
  material, and where the section names none it uses the 60 ksi it has always
  assumed for such a concrete (an existing default, not introduced here).
- A combination that loses its shell, solid or link results gives no message;
  the result tables show them as empty.

## Evidence

| Check | Reference | Tolerance |
| --- | --- | --- |
| Plane members of the six-storey grid: 38 frames and 15 areas at a level, 66 frames in an elevation | counts from the grid: 5·4 + 3·6, 5·3, 6·6 + 5·6 | exact |
| Window selection of a two-bay portal, both directions | sets from the help topic's rule | exact |
| Quick tools with a fixed view: click, window, elevation, rectangle, free point | screen positions computed from the projection; objects written out by hand | exact; normal to 1e-12 |
| Quick tools over a whole plane | the model the first wave's index commands build | model equality |
| Pointer drags, clicks, Enter and Escape through the application (`video_drawing_e2e.rs`) | objects written out by hand | exact |
| Portal drawn through points | reactions of the code-built portal | 1e-12 |
| Area criterion | the solver's own validation, including 120 pseudo-random shapes | same outcome |
| Custom 2D frame with equal spacings | the existing template | bitwise |
| Ratio division, L = 10, n = 4, r = 8 | joints at 2/3, 2, 14/3; deflection `w·x(L³ − 2Lx² + x³)/(24EI)` at x = 2 | 1e-9 |
| Crossing beams before and after the break | `PL³/(48EI)`, `PL³/(96EI)`, four reactions of P/4 | 1e-9 |
| Area load by direction on a 6 × 5 m panel | resultants 24 kN and 15 kN | 1e-9 |
| Material unit factors; the duplicate rule; parser errors | exact powers of ten; equality; line numbers | 1e-12 |
| Vendor libraries (opt-in): totals; A36 29000/36/58 ksi; A992 Fy50; 4000 psi concrete; S355; C30/37 | the files' own values | 1e-9 |
| N shapes imported at once | N single imports | bitwise |
| `euro.pro` (opt-in): 58 IPE shapes; IPE300 | the file's values | 1e-12 |
| Shell strip: F11 = 50, M11 = −10, top stress 1750 kN/m², tip displacements | beam closed forms with ν = 0 | 1e-9; two rows of `structurax verify` |
| Example 2-015 (opt-in): hogging edge moment negative, top face in tension | published −81.54 k-in/in and −1.077 in | sign; 1 % on Uz |
| Free and restrained square plate under a uniform change and under a gradient; the same in a nonlinear static case and its continuation | `αΔT·L`; `−EαΔT/(1−ν)`; `w = −αg(x² + y²)/2`; `M = Et³αg/(12(1−ν))` | 1e-9 to 1e-12 |
| Example 2-013 (opt-in), thin and thick, free and restrained | published 0.000132 in, 0.000066 in, 0, −733.33 lb/in² | 1e-6; 1e-5 |
| Example 2-014 (opt-in), thin plate option, 9 × 16 and finer mesh | independent (Roark) Uz 0.01931 in, M11 −2.181 k-in/in | Uz 4 % and 1 %; M11 0.5 % |
| Base reaction of a meshed panel with fixed corners | the whole load and its moments | 1e-9 |
| Combination of the strip's cases; combination with a spectrum case | hand values; absence | 1e-9; exact |
| Refusals above | error text, model unchanged | exact |

In Example 2-014 the published SAP2000 displacement of the coarse model is
0.01905 in; this element gives 0.01859 in there and 0.01921 in on the finer
vendor mesh, against the independent 0.01931 in. The test bounds the
difference; it does not claim the vendor's element. Only the thin plate files
are tested: the thick plate option gives the same numbers here, while the
vendor publishes different ones for it (0.01778 in), so the thick formulation
is not reproduced for this problem.

## End-to-end

`crates/sap-app/src/gui/video_drawing_e2e.rs` and the files beside it drive
the application itself (pointer drags, clicks and keys through its frame
function) and compare with models built independently.

- **The video's first building, drawn with the mouse.** Six plan windows for
  the beams, twenty-four narrow elevation windows for the columns (one per
  column line: a window over a whole elevation would also take the grid
  segments along the base), six brace clicks, six slab windows, and a selection
  window at the base followed by the Fixed restraint command. The result has
  the same joints with restraints, frames (end order, section, releases) and
  areas (corner order) as the model the first wave's index commands build.
  With the first wave's loads, mass source and cases on both, base reactions
  agree to 1e-9 and the first three periods to 1e-8 (the joint order differs).
  Diaphragms are left out of both models: they are penalty constraints and
  would cost digits between two joint orders.
- **The video's second frame from the template.** Bay widths "4, 6, 4" and
  storey heights "3, 3, 3, 3" give the second wave's frame; with that wave's
  hinges, loads and cases the Accel pushover collapse factor is the same as
  for the frame built in code (1e-9). The dialog's Generate button is inline
  code; the test calls the function it calls.
- **Pieces of the fourth model.** A 10 m line drawn through two points and
  broken at nine selected joints equals the equal division into ten (1e-12).
  A concrete from a synthetic library file reaches the analysis: a cantilever's
  tip deflection is `PL³/(3EI)`, with the shear term `PL/(G·As)` when shear
  areas are present (1e-9). Opt-in: ASTM A36 from the United States file has
  Fy = 36 ksi in the model's units, and the names `C30/37` and a Cyrillic
  grade survive both S2K and the native writer.
- **Chooser to window.** Browse, pick a section file in the chooser, select
  two shapes with click and Ctrl+click, import, and draw a frame that gets one
  of them.
- **File formats.** The drawn building round-trips through S2K. The native
  writer writes the drawn objects and they read back equal; once the first
  wave's Ritz controls, mass source and automatic seismic load are on the
  model it refuses, as it has since that wave.

One thing the end-to-end work turned up: a library steel came out with a zero
shear-reinforcement yield stress, which the native writer refuses to write. It
now follows the yield stress, the model's convention for a steel without a
separate grade.

## Review coverage

The shell stress, temperature, combination and reaction changes, the model
commands for drawing and breaking, the material conversion and the file
chooser were read. The viewport changes (about 1100 lines) and the windows
were integrated on their tests, the agents' reports and spot checks, plus the
application-level tests of the join between the viewport and the tools. The
windows were exercised through command functions and headless renders with
synthetic pointer and key input. **Nobody has used them by hand.**

## Not done

The table message for dropped results, the stored von Mises of multi-valued
combinations, animation export and eccentric diaphragm moments are since
done (see [tutorial-video-wave5.md](tutorial-video-wave5.md)); the current
list is [tutorial-video-progress.md](tutorial-video-progress.md).

- Drawing aids: snapping to midpoints, intersections and perpendiculars, the
  length and angle constraints, back-face views, cylindrical grids, the 3D
  template with custom spacing, areas with more than four joints, an area
  section "None" for openings, breaking at area or solid edges.
- A preview of a brace or secondary-beam window; an operating-system file
  dialog.
- Material values the model has no field for (expected strengths, strains);
  lightweight concrete; aluminum and cold-formed materials.
- A message when a combination drops shell, solid or link results (since
  shown in the result display); von Mises and principal values of SRSS and
  absolute combinations (the stored value is still combined field by field;
  the display refuses it since the fourth wave).
- Temperature loads on three-joint areas, with joint patterns, and on plane
  and axisymmetric-solid sections.
- Result display: displacement contours, the choice of shell stress component
  and face, absolute-acceleration plot functions (since done; see
  [tutorial-video-wave4.md](tutorial-video-wave4.md)), animation export.
- Reinforcement "to be checked" with a bar layout (the corpus has ACI 318-14
  Example 002 as a reference; it belongs to a design wave).
- Coupled and automatic hinges, capacity spectrum, partial pushover results,
  typed API calls for hinge definitions.
- Eccentric diaphragm moments and wind loads of the lateral loads manual;
  program-calculated period.
- Sparse storage; `cSapModel` facades for the commands of the three waves.
- Everything listed as not done in the first two waves and not named above.
