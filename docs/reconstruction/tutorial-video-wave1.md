# Features of a SAP2000 tutorial video, first wave

Date: 2026-11-15. Scope note for one bounded set of additions. It does not
change the project status: vendor-runtime acceptance and broad parity remain
unestablished.

The source of the feature list is a two-hour tutorial video ("SAP2000 Complete
Mastery 2024", KSSE Structural Engineers) with four models: a six-storey steel
building with seismic load and steel design, the pushover of a concrete plane
frame, the time history of a ten-storey frame, and a hyperbolic cooling tower.
The video was used only to list which commands and options a user meets. No
number in it is a reference: its results are read off the screen for models
whose section sizes are not stated. Every reference below is a vendor manual or
verification problem in the local corpus, a closed form, or a symmetry.

What a feature needs was checked against the vendor material first. Three
things the video uses are **not** in that material and were therefore not
implemented from memory: AISC 360-16, ASCE 7-16, and the ASCE 7 site-coefficient
and importance-factor tables.

## What is supported now

### Solver capacity

- The LDLᵀ factorization and its solves work on each row's profile. Factors and
  solutions are bitwise the ones of the previous dense loops.
- An Eigen modal case with more than 400 mass degrees of freedom and fewer than
  a quarter of them requested uses subspace iteration (Bathe 1996, §11.6). A
  Sturm sequence count of `K − σM` must equal the number of modes found below
  `σ`; on a mismatch the iteration is retried with a doubled subspace, and if
  it still cannot prove its set complete the exact condensation of the previous
  version decides. Smaller problems use that condensation unchanged. The
  shift `σ` is placed in the gap above the wanted eigenvalues (half the gap to
  the next Ritz value, at most one percent): a shift only a millionth above the
  last eigenvalue gave a wrong pivot count on a stiff 1821-joint shell model,
  and the fallback is then far too slow for a model of that size (5415 mass
  degrees of freedom).
- A modal case factorizes the penalty-constrained stiffness against the
  structural diagonal, as the static path already did. Before this a model with
  diaphragm constraints could be reported singular in a modal case (vendor
  Example 1-022 as imported was).
- The fallback is used up to 1500 mass degrees of freedom. Above that it
  would run for hours, so the eigen error is returned instead.
- Storage is still dense: memory is `8 n²` bytes per matrix copy, so the
  practical ceiling is a few thousand joints.

Measured, release build, one run each:

| Model | Before | Now |
| --- | --- | --- |
| 8 × 8 × 8 lattice, 1344 mass degrees of freedom, 12 modes | 67.5 s (condensation) | 0.09 s |
| The same lattice, one factorization | 357 ms | 25 ms |
| Training-ebook building, 3 × 3 slab mesh, 516 joints | 3.6 min | 0.3 s |
| The same, 4 × 4 mesh, 861 joints | not finished in 21 min | 1.1 s |
| The same, the document's 6 × 6 mesh, 1821 joints | not attempted | 5.9 s |

The "before" times of the building are from 2026-11-13 test runs. On the
document's mesh the periods are 1.1452, 1.1442 and 1.0391 s against the printed
1.165482 (twice) and 1.062094 s, 1.7 to 2.2 % short, with 99.42 % of the mass
in 12 modes against 99.405 %; the mass of that model is entered by hand, so
this is a bound and not an identity (an ignored test). The three models of the
end-to-end test below (168, 396 and 324 joints) analyze in 7 s together in a
debug build.

The profile is that of the joint order as given. Joints created by area
meshing come last, which widens it; there is no bandwidth-reducing
reordering.

### Native import of end length offsets (defect found on the way)

The native line record stores the end length offsets as fractions of the
object length. The importer put those fractions into the model's offset
lengths, so an imported frame with a 20 cm offset had a 0.2 cm one and was too
flexible; the native writer made the mirror error. Both now convert. Seven
corpus files carry offsets (Examples 1-010b to d, 6-006, 6-007, 6-011a and b).

- Example 1-010b to d, read from the vendor files, now give the published tip
  deflections −0.39951, −0.37523 and −0.35197 in to all five digits.
- Example 6-006 and 6-007, whose modal case could not be factorized before the
  repair above: periods 0.4309, 0.1303 and 0.0745 s against the published
  0.438, 0.135 and 0.074 s (−1.6 %, −3.5 %, +0.6 %). With the old offsets they
  would have been 0.554, 0.169 and 0.098 s. **The residual is not explained**;
  the opt-in test bounds it at 4 % and does not claim agreement.
- Models built through the API, the GUI or S2K were not affected: there the
  offsets were always lengths.

### Mass source

`SapModel::mass_source`: element self mass, additional mass, and load patterns
with multipliers (CSI Analysis Reference, "Mass Source"). For each pattern the
downward joint-equivalent load divided by g is added to the three translations
of the joint. The default source gives bitwise the previous mass.

Refused: a net negative mass at a free translation (the vendor sets it to zero
with a warning), an automatic lateral pattern used as mass, temperature loads,
loads in a named coordinate system and projected area loads in a mass pattern.
One source per model; no per-case source.

### Automatic seismic load, ASCE 7-10

`SapModel::auto_seismic`, one definition per Quake pattern (CSI Lateral Loads
Manual §2.14 and §2.2.2 to 2.2.4): direction X or Y, approximate period
`Ct·hn^x` or a user period, R, Ie, Ss, S1, TL, Fa, Fv, optional elevation
range. `SDS`, `SD1`, `Cs` (Eqns. 12.8-2 to 12.8-6), `k`, `V = Cs·W` and the
joint forces `V·w·h^k/Σw·h^k` are computed and reported per case. The report
lists a force for every joint that has mass, including joints created by frame
subdivision and area meshing: they carry real force, so the report is the one
result that names joints the model does not define.

Differences from the vendor, all deliberate:

| Item | Vendor | Here |
| --- | --- | --- |
| Fa, Fv | from site class (Tables 11.4-1, 11.4-2) | entered; the tables are not in the corpus, a missing value is refused |
| Ie | from occupancy category (Table 1.5-2) | entered |
| Program-calculated period | modal period limited by `Cu·Ta` | refused |
| Eccentricity | moment about the diaphragm center of mass | a nonzero ratio on a loaded diaphragm joint is refused; default ratio 0.05 as in the vendor, so a model with diaphragms must set it to 0 |
| Weight `W` | "based on specified mass"; the joint set is not printed | mass-source weight of the joints above the base elevation up to the top elevation |
| Editions | ASCE 7-02, 7-05, 7-10 and many other codes | ASCE 7-10 only |

Ω0 and Cd are stored and not used. Wind loads are not implemented.

### Ritz vectors and participation

A Ritz case takes its starting vectors from its load list: load patterns and
the accelerations `UX`, `UY`, `UZ` (the names `U1` to `U3` of imported native
files are accepted). Per load: a number of generation cycles and a target
dynamic participation ratio (CSI Analysis Reference, "Ritz-Vector Analysis").
Every modal case reports, per load, the static and dynamic participation
ratios per mode and cumulatively.

- **Behaviour change.** Before this a Ritz case ignored its load list and
  always started from unit `UX`, `UY`, `UZ`. A Ritz case with an empty list
  still does, bitwise. A case with a list now uses exactly that list, so a
  saved model with such a case can give different modes than before. Five
  corpus files have such a case (Examples 6-006, 6-007, 6-010, 6-011a and
  6-011b, starting from `U1`, `U1`+`U2` or `U1`+`U3`). Before the modal
  factorization repair below all five stopped at a singular pivot in the modal
  case; now four analyze and 6-011a is refused (its Ritz projection is not
  positive definite). The files also list the nonlinear link degrees of freedom
  as starting vectors, which the import does not carry.
- Refused: rotational accelerations, a load with no component on a mass degree
  of freedom (the vendor only warns), an Eigen case with a load list or with
  controls, `min_modes` above 1 with starting loads.

### Modal time history

A `LinearHistory` case with `modal_case` set superposes the modes of that modal
case with one constant damping ratio (CSI Analysis Reference, "Linear
Time-History Analysis"). The load is piecewise linear between the output times
and the function abscissae, and each mode is integrated exactly on each piece.
Results: displacement and relative-acceleration traces, and the usual results
at the step of largest displacement.

- Without `modal_case` nothing changes: such a case is still integrated
  directly, and a nonzero modal damping ratio on it is still refused.
- A time function is zero before its first point and constant after its last
  one in the modal path (reference p. 401). The direct path keeps its previous
  rule, which holds the first value before the first point.
- Refused with `modal_case`: Rayleigh damping, an initial case, link viscous
  damping, a nonlinear history type.
- Not implemented: proportional or interpolated modal damping, damping
  overwrites, the built-in ramp, arrival time and time scale, periodic cases.
  Reactions leave out the force carried by modal damping.

### Functions from file

`sap_format::function_file` reads a text file of values at an equal interval
or of time and value pairs, free or fixed format, with header lines to skip,
prefix characters, points per line, `$` comment lines and a stop at the first
blank line, as the vendor help topic describes. The first value of an
equal-interval file is taken at `t = 0`; the topic does not say. The result is
stored as an ordinary user function, so a model does not depend on the file.

### Steel design workflow

- **Auto Select lists.** A frame assigned a list is analyzed with one member of
  it: the design section if a design has chosen one, otherwise the median of
  the list by weight (the lower median of an even count; the manual is silent
  on that). Design takes the lightest member whose ratio is within the limit
  and whose status is OK, or the heaviest, flagged, if none is (Steel Frame
  Design Manual AISC 360-10 §2.1). **Before this such a frame was analyzed with
  placeholder section properties and no warning.**
- **Verify Analysis vs Design Section** and **Verify All Members Passed**.
- **Demand/capacity ratio limit**, default 0.95 (Table B-1). It decides the
  Auto Select choice and the second verify command. The status text of a
  result still changes at 1.0.
- **Default design combinations** `DSTL1`, …, LRFD or ASD, from §3.2 of the
  manual, with the redundancy factor and `SDS` forms of p. 3-8.

Differences from the vendor: a term whose load type has no case is left out of
its row and rows that become equal are listed once, so a model without roof
live or snow load gets a few rows the vendor may not create (they are never
less safe than leaving them out). A response-spectrum case gets a `+` and a
`−` row; all components reverse together, the mixed-sign permutations are not
designed. Notional loads are refused. The overstrength (Ω0) combinations are
not created. Seismic design category, framing type (SCBF and the like) and
second-order method preferences are not implemented; design remains AISC
360-10 and Eurocode 3.

### Geometry commands

- **Quick grid**: numbers of lines and spacings per direction.
- **Quick draw on the grid**, by index region: beams, columns, braces (X, V,
  inverted V), secondary beams (N per bay), one area per cell. Braces and
  secondary beams are pinned in bending at both ends. The beam carrying a brace
  apex or a secondary beam is subdivided at joints for analysis.
- **Select by section**; **diaphragm constraints** for the selected joints, one
  constraint or one per elevation.
- **Radial replicate** and **radial extrusion of lines to areas** about a
  vertical axis. Joint local axes, vertical frames and horizontal areas are
  turned with the copy; assignments that would not be the rotated original
  (global X or Y loads, unequal X and Y restraints or springs on joints with
  global axes, links, solids) are refused instead of copied wrongly.
- **Reverse area local 3**, keeping the loads in place globally or locally.
- **Curved frame** as a number of straight frames on a circular arc or a
  parabola through three points.

Not implemented: cylindrical grids, extrusion and replication about other
axes, keeping a curved frame as one object, edge pressure under reversal
(refused). Drawing with the mouse on a plan or elevation and plane views at a
level were added later; see [tutorial-video-wave3.md](tutorial-video-wave3.md).

### File formats and GUI

- **S2K** keeps all of the above through reconstruction extension tables
  (`RUST MASS SOURCE`, `RUST AUTO SEISMIC - ASCE 7-10`, `RUST CASE - MODAL RITZ
  CONTROLS`, `RUST STEEL DESIGN PREFERENCES`) and extension columns
  (`RustModalCase` on the case row; `AutoSelDesign`, `AutoSelAnalysis` on the
  frame row), written only where the data is not the default.
- **Native export** refuses a model that carries any of it.
- **Native import** does not read the vendor's own mass source, automatic
  lateral load or Ritz target records; a non-default mass source and an
  automatic lateral load are still recorded as unmodeled source data.
- **GUI**: five windows — *Edit > Grid, Quick Draw, Radial, Extrude, Select by
  Property*; *Define > Mass Source and Auto Seismic Loads*; *Define > Functions
  from File and Plot Functions*; *Display > Modal Periods and Participation
  Tables*; *Design > Steel Auto Select, Preferences, Design Combos, Verify* —
  and new controls in the case editor (Ritz loads with cycles and targets; modal
  case and damping of a history case). Paths were typed; a file chooser inside
  the program came with the third wave. The windows were exercised through their command functions and
  headless render tests, not by hand.
- **API**: the Rust model and design functions only. No `cSapModel` facade was
  added for these features.

## Evidence

| Check | Reference | Tolerance |
| --- | --- | --- |
| Profile LDLᵀ: factors, solutions, zero rows, error indices | the previous implementation, copied into the test | bitwise |
| Eigenvalues of the Bathe–Wilson frame, by condensation, by subspace iteration and through a model | CSI Example 1-021, independent values 0.589541, 5.52696, 16.5879 (Peterson 1981) | 1e-5; also three rows of `structurax verify` |
| 2000-mass chain, lowest 20 eigenvalues, subspace iteration | `(4k/m)·sin²((2j−1)π/(2(2n+1)))` | 1e-10 |
| A mode orthogonal to every starting vector | found after the Sturm mismatch, or an error | exact |
| Sturm count with massless, restrained and unused degrees of freedom | 2 × 2 closed form | exact |
| Period of a column whose mass comes from a load pattern | `2π√(mL³/3EI)`, `m = P/g` | 1e-9 |
| Mass from dead load of frames, a shell and uniform loads | weight by hand and the vertical reaction | 1e-10 |
| `Ta`, `SDS`, `SD1`, `Cs` (each of Eqns. 12.8-2 to 12.8-6), `k`, joint forces | the printed equations evaluated in the test; base reaction `= −V` | 1e-9 |
| Ritz frequencies of a two-mass cantilever | eigenvalues `9 ± √74` of the flexibility matrix | 1e-10 |
| Number of vectors generated for a target | first-vector ratio from the flexibility matrix, one model on each side of the threshold | exact |
| Static participation of a pattern start; dynamic participation of an acceleration | sums to 1; equals the cumulative mass ratio | 1e-10 |
| Modal history: damped ramp, step off the output grid, two-mass step, ground acceleration | closed forms | 1e-9 of peak |
| Vendor Example 1-022 (opt-in): periods, static displacement, modal history peak 5.4862 in, as imported with its diaphragms | published 5.486 in (SAP2000), 5.46 in (independent) | 1 % |
| Function files (opt-in): 42 vendor `.th` files and the El Centro records | point count and interval stated in each file's header | exact |
| End offsets from vendor files (opt-in): Example 1-010b to d tip deflection | published −0.39951, −0.37523, −0.35197 in | 6e-6 in |
| End offsets written natively and read back | the model's lengths; the stored fractions | bitwise; 1e-15 |
| Modal cases as the case editor creates them: Eigen, Ritz without a load (refused at save), Ritz from `UX` | `2π√(m/k)` | 1e-12 |
| The designed building saved as S2K and reopened | same reactions, base shear, first period and analysis sections | 1e-9 |
| Default combinations, LRFD and ASD, ρ = 1.3, SDS = 0.82 | factors typed from the manual, page per row | 1e-12 |
| Auto Select choice for a braced beam | `Mu = wL²/8`, `φMn = 0.9·Fy·Zx` of four table sections | 1e-9 |
| Deflection after re-analysis with the designed section | `5wL⁴/(384EI)` | 1e-6 |
| Radial extrusion 36 × 10°: closure, coordinates, normals | sin and cos | 1e-12 |
| Copies of joints, frames and areas with local axes | response equals the rotated original; a wrong sign does not | 1e-9 |
| Reversed area | same response with loads kept global; different with loads kept local | 1e-9 |
| Braced bays, secondary beams, diaphragms | reactions balance the load; a floor moves as one body | 1e-9, 1e-4 |
| S2K round trip of every new field; unreadable rows; native refusals | equality; errors | exact |

End-to-end (`crates/sap-app/tests/video_tutorial_workflows.rs`): the video's
steel building, ten-storey frame and tower built through these commands.
Checked there: the hand load take-down against the reactions (1e-9); the
seismic weight, coefficient and base shear against the printed equations
(1e-9; 1e-4 through the penalty diaphragms); that Auto Select analysis and
design sections converge; that a heavily damped modal history on Ritz vectors
settles on the static response to the same load (1e-6); and that the modes of
the shell of revolution come in equal pairs (1e-6).

Each refusal named above has a negative test.

## Not done (later waves)

- **Pushover** (the video's second model; since done in part, see
  [tutorial-video-wave2.md](tutorial-video-wave2.md)): frame hinges, hinge states and the
  pushover curve, pushover loads from a mode or an acceleration, results saved
  at several steps. The corpus has the material for moment hinges (Analysis
  Reference Ch. VIII, Example 1-026); it needs a new nonlinear path.
  P-M2-M3 and automatic FEMA/ASCE 41 hinges have no tables in the corpus.
- Concrete reinforcement "to be checked".
- AISC 360-16, ASCE 7-16, AISC 341 seismic framing types, wind loads.
- Site-class and occupancy lookups, program-calculated period, eccentric
  diaphragm moments.
- A material library and multi-selection from a section file (since done; see
  [tutorial-video-wave3.md](tutorial-video-wave3.md)).
- Displacement contours, shell stress component choice, video export, export to
  other programs.
- Sparse storage for models beyond a few thousand joints.
- `cSapModel` facades and native persistence for the new data.
- Native import does not turn the vendor's modal time-history cases into
  `modal_case` references; they stay direct-integration cases, refused where
  they carry modal damping.
- The residual of Examples 6-006 and 6-007 above.
- A response-spectrum case mapped an unknown direction string to U1 without
  an error (found in review, not changed here; refused since the third wave).
