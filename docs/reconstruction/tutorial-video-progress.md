# Tutorial-video work: goal, timeline and ledger

This file tracks one line of work across its waves. It is the place to look for
what is done, what is open and why. The wave documents
([first](tutorial-video-wave1.md), [second](tutorial-video-wave2.md),
[third](tutorial-video-wave3.md), [fourth](tutorial-video-wave4.md),
[fifth](tutorial-video-wave5.md)) hold the scope, evidence and refusals of
each wave; [STATE.md](STATE.md) holds the history of the whole project. Where
this file and a wave document disagree about an open item, this file is the
newer one.

## Goal

1. Every feature that the tutorial video "SAP2000 Complete Mastery 2024" uses
   (63 rows, listed below) works in StructuraX with behaviour that has a
   reference in the vendor material or a closed form, or is refused with a
   stated reason.
2. Every defect found on the way is repaired, and every item a wave left
   undone is finished, or is recorded here with the reason it cannot be.
3. The same for a second video, "SAP2000 V26 Nonlinear Pushover Analysis of
   Multistory RC Structures Considering Higher Modes" (21 rows, added on
   2026-11-17 after the fifth wave; see "The second video" below). It shows
   SAP2000 v26, while the vendor material of this project is v19: a form
   that exists only in v26 has no reference here.
4. The five "CSI Spotlight" videos of the vendor's own video page, read on
   2026-11-18 (see "The CSI Spotlight videos" below). Two of them show
   versions 23 and 25 and are outside the v19 baseline. Two more videos of
   that page, "Static Pushover Analysis" and "P-Delta Analysis", were read
   the same day (see "The vendor's pushover and P-Delta videos").
5. On 2026-11-18 the user supplied vendor material of SAP2000 v24 (see "The
   v24 material" below). It is a second source of evidence, kept apart from
   the v19 corpus. v19 stays the baseline.
6. The other 39 videos of the vendor's page, read on 2026-11-18 by reading
   agents from transcripts and frames (see "The other 39 vendor videos"
   below and [vendor-video-gaps.md](vendor-video-gaps.md)). They are a list
   of what is missing, not rows that this file counts as done.

Progress is counted against the 63 rows, the second video's 21 rows (kept
apart) and the ledger. **No
percentage of SAP2000 compatibility is claimed**; the README's status ("not
established") stands. No number shown in the video is used as a reference.

## Status words

| Word | Meaning |
| --- | --- |
| done | works, with a test that has an independent reference |
| partial | part works; the missing part is a ledger item |
| open | can be done with the material at hand; not done yet |
| blocked: evidence | the vendor material has no formula, table or example for it, so the project rule "refuse, do not approximate" keeps it out |
| blocked: rule | a project rule forbids the means (for example a new dependency) |
| needs the user | cannot be done from this side |
| not classified | found in a reading; the vendor material has not yet been searched through for it, so it is neither "open" nor "blocked". The planning of the next wave classifies it |

## Timeline

| Wave | Date | Commit | Theme | Portable tests | With vendor corpus | Reference checks |
| --- | --- | --- | --- | --- | --- | --- |
| before | 2026-11-13 | `2238221` | state before the video work | — | — | — |
| 1 | 2026-11-15 | `c817b9b` | seismic load, mass source, Ritz, modal time history, Auto Select design, geometry commands, solver capacity | 656 | 674 | 86/86 |
| 2 | 2026-11-16 | `6b33019` | frame hinges and static pushover | 803 | 825 | 90/90 |
| 3 | 2026-11-16 | `7ee7d44` | drawing and editing on plane views and the grid; material library; shell output repairs | 965 | 990 | 92/92 |
| 4 | 2026-11-17 | `0dfc0ba` | result display: shell stresses, displacement contours, plot functions, modal tables; participation and 3D view repairs | 1102 | 1127 | 94/94 |
| 5 | 2026-11-17 | `1a7d2e8` | repairs: rigid zones, released ends, loads, non-prismatic sections, geometric stiffness and output stations of frames with end offsets, constraints at rotated joints, the unstable joint named, orthotropic and plane-strain areas, material angle, combinations as maximum and minimum, design by states and permutations, modal histories from native files, the Ritz projection, window defects found by review, by looking and by a hand check on a private display; video export, load read-out, seismic eccentricity, Grid Only, unequal flanges | 1348 | 1383 | 106/106 |
| 6 | 2026-11-18 | not committed | nonlinear static for the second video: P-Delta with hinges at member ends, the stiffness after a nonlinear case, a case that stops early, generated hinge properties, the hinge results window; repair of D69 and D70, of five findings of an outside review (D74 to D78) and of six findings of its second pass (D79 to D84) | 1539 | 1576 | 106/106 |

## The video's 63 feature rows

Status after the fifth wave (`1a7d2e8`). "Ledger" names the item that
holds the missing part. A row the fifth wave moved names the tests that
prove it; the tests are in `crates/sap-app/src/gui/` unless a crate is
named. The second test group of such a row, in `video_wave5_e2e/`, is the
wave's end-to-end test of the row through the windows. The end-to-end tests
and the hand check moved no row: the counts are those of the first half of
the wave.

| Rows | Status | Ledger |
| --- | --- | --- |
| **Part 1, steel building (28 rows)** | 23 done, 4 partial, 1 open | |
| 1.1 new model, units, grid-only start | done; the "Grid Only" template entry since wave 5 (`wave5_windows_tests.rs`: `grid_only_gives_a_blank_model_and_the_quick_grid_lines_form`; `video_wave5_e2e/rows.rs`: `grid_only_in_kn_m_makes_a_3_by_3_by_3_grid_and_refuses_a_negative_spacing`). No default materials by region | G36b |
| 1.2 quick grid; 1.3 plane views; 1.4 material by grade; 1.5 section file import; 1.6 Auto Select list; 1.7 shell section | done | |
| 1.8 to 1.11 quick draw of beams, secondary beams, columns, braces | done | |
| 1.12 rectangle, quick and poly areas | partial: more than four corners refused, no section "None" | G09, G10 |
| 1.13 select by property, previous selection; 1.14 replicate | done | |
| 1.15 automatic lateral load "ASCE 7" | partial: ASCE 7-10 | G21 |
| 1.16 seismic parameters | partial: Fa, Fv, Ie entered by hand; no program period. The eccentricity on diaphragms is done since wave 5 (`sap-solver/tests/wave5_eccentricity.rs`: `x_load_moment_is_ratio_times_width_perpendicular_times_force`, `an_overwrite_distance_replaces_ratio_times_width`; `video_wave5_e2e/rows.rs`: `the_seismic_window_eccentricity_gives_ratio_times_width_times_force_and_the_statics_of_it`) | G22, G23 |
| 1.17 area uniform loads by direction | done since wave 5: the read-out of the loads on an object (`wave5_windows_tests.rs`: `the_load_lines_show_the_stored_values_in_display_units`, `a_right_click_in_the_undeformed_display_opens_the_object_window_with_the_loads`; `video_wave5_e2e/rows.rs`: `the_object_window_paints_the_assigned_loads_in_the_display_units_asked_for`) | |
| 1.18 diaphragm; 1.19 restraints; 1.20 Ritz with targets; 1.21 mass source; 1.22 run and unlock; 1.23 diagrams and tables; 1.24 default design combinations | done | |
| 1.25 steel design preferences | partial: AISC 360-10 and EC3; no seismic category, framing type or second-order method | G20, G26 |
| 1.26 design with Auto Select and the verify commands; 1.27 extruded view | done | |
| 1.28 export to other programs (mentioned in the video, not shown) | open | G35 |
| **Part 2, pushover (13 rows)** | 11 done, 2 partial | |
| 2.1 2D frame template with custom grid | done | |
| 2.2 reinforcement "to be checked" with a bar layout | partial: total area and cover, fixed 3 x 3 bars | G11 |
| 2.3 frame loads | done | |
| 2.4 hinge properties | partial: uncoupled M3 and the other single-component hinges; P-M2-M3 refused | G12 |
| 2.5 to 2.13 hinge assignment, mass source, diaphragms, Ritz, chained nonlinear cases, Mode and Accel loads, displacement control, hinge states, pushover curve | done | |
| **Part 3, time history (8 rows)** | 8 done | |
| 3.1 3D template; 3.2 box and I sections; 3.3 Eigen 20 modes; 3.4 function from file; 3.5 modal time history; 3.6 deformed shape at a time and animation; 3.7 plot functions | done; 3.2 has separate bottom-flange fields since wave 5 (`i_section.rs`: `an_i_section_with_unequal_flanges_has_the_closed_form_properties`; `wave5_windows_tests.rs`: `the_i_section_dialog_makes_the_unequal_flange_section_and_refuses_a_wrong_dimension`; `video_wave5_e2e/rows.rs`: `a_section_with_unequal_flanges_made_in_the_dialog_gives_the_hand_deflection`) | |
| 3.8 create video | done since wave 5: multi-step and cyclic videos as uncompressed AVI (`video_export/tests.rs`: `avi_tiny_video_equals_the_hand_written_layout`, `record_paints_each_pose_and_writes_a_readable_avi`; `wave5_windows_tests.rs`: `a_video_of_a_history_through_the_menu_path_has_the_layout_length_and_moving_frames`; `video_wave5_e2e/video.rs`: `a_multi_step_video_of_the_modal_history_has_the_frames_the_form_states_and_the_joint_where_the_pose_puts_it`, `a_cyclic_video_of_a_mode_has_cycles_times_frames_and_repeats_each_cycle`). No compressor choice; no cyclic video of a plain deformed shape | |
| **Part 4, cooling tower (14 rows)** | 13 done, 1 partial | |
| 4.1 joints by coordinates; 4.2 curved frame; 4.3 divide and break; 4.4 radial extrusion; 4.5 radial replicate; 4.6 reverse local 3; 4.7 display options | done | |
| 4.8 sections and library concrete | partial: lightweight concrete refused | G32 |
| 4.9 select by section; 4.10 mass source; 4.11 Ritz 10 and 20 modes; 4.12 modal tables; 4.13 animation, displacement contours, read-out; 4.14 shell forces and stresses in MPa | done (shell contours are one value per element) | G01 |
| **All 63** | **55 done, 7 partial, 1 open** | |

## The second video

"SAP2000 V26 Nonlinear Pushover Analysis of Multistory RC Structures
Considering Higher Modes" (KSSE Structural Engineers, 37 min), read on
2026-11-17 from its transcript and from 26 frames of its screen. A
reinforced-concrete portal frame of two bays and five storeys gets moment
hinges in its beams and interacting P-M3 hinges in its columns, gravity as
two chained nonlinear cases, a Ritz modal case on the stiffness after
gravity, and three pushovers with P-Delta: one by a uniform acceleration,
one in the shape of mode 1 and one in the shape of mode 2. The curves are
exported to Excel and compared there.

**The model as shown does not run in StructuraX.** Five things stopped it
when it was read. The sixth wave removed the fourth and the fifth and a
part of the third; the first two remain for lack of a rule in the vendor
material:

1. *Remains.* The beam hinge has "Use Yield Moment" ticked, so SAP2000
   takes its strength from the section and its reinforcement; the column
   hinge takes its strength from the default interaction surface. StructuraX
   has neither; the strength must be entered as a scale factor (G45, G12).
2. *Remains.* The column hinges are interacting P-M3 hinges, which are
   refused (G12).
3. *Removed in part.* The pushover cases use P-Delta together with hinges
   (G14): supported since wave 6 for hinges at member ends when every case
   of the chain has P-Delta. The video's chain is mixed: its gravity cases
   DEAD and LL have no geometric nonlinearity and its push, which continues
   from them, has P-Delta (seen on the case forms on 2026-11-18, after the
   wave was built). A mixed chain is refused (G14b): the manual only
   "strongly recommends" one setting (v19 Analysis Reference pp. 427, 431)
   and does not say how a case that switches P-Delta on takes over the
   state of one that had it off.
4. *Removed.* The Modal case uses the stiffness at the end of a nonlinear
   case of a model with hinges (G48): supported since wave 6 while no hinge
   has yielded there. Both mode-shape pushovers name that Modal case.
5. *Removed, and its premise was not confirmed.* The curve and the table on
   screen end near 0.135 m after nine steps, against a target of 0.6 m or
   more. A StructuraX case that cannot reach its target used to return an
   error and keep nothing; since wave 6 it keeps its states and is marked
   "not finished" (G16). Whether the vendor run really stopped short is a
   reading: the saved-step rule and conjugate control cannot explain the
   curve (v19 Analysis Reference pp. 430, 432), and nothing in the manual
   confirms a stop.

With moment hinges of entered strength in the columns, and with one
geometric setting in every case of the chain, the frame is built and run
end to end, without P-Delta and with it
(`sap-app/tests/video2_pushover_workflow.rs`, 18 tests; see the wave
document for what it asserts and for the substitutions that remain).

Two points that the first reading left unchecked were looked at on
2026-11-17. Grade C40 converts: it is a normal-weight concrete of the
vendor's China file, and the corpus test of the material library places all
nine refused materials in the United States file. A load that moves the monitored joint toward the
negative side, as the video's acceleration load with scale 1 does, is
followed in that direction: the hinge solution measures the target along the
way the monitored displacement moves when the load grows (read in the code
on 2026-11-17, not run on this frame).

A status below that names a row of the first video or a ledger item is the
status of that row or item, with its caveats; a row the sixth wave moved
names its tests. The end-to-end test builds the frame from the program's
command functions; the model was not built by hand in the windows. No
number shown in the video is used as a reference.

| Row | Status | Covered by |
| --- | --- | --- |
| V2.1 new model in kN, m, C; 2D Frames, Portal, 5 storeys, 2 bays, storey height 3, bay width 8 (the numbers as spoken) | done | row 2.1 |
| V2.2 concrete from the material library: region China, grade C40 | done, when the vendor's library files are present. The grade is in the vendor's China file and converts (the nine materials that do not are all in the United States file) | row 1.4 |
| V2.3 rectangular concrete sections; the second section made as a copy of the first | partial: sections are made one by one; there is no "Add Copy of Property" | G44 |
| V2.4 reinforcement data: rebar materials for longitudinal and confinement bars, design type column or beam, bars along each face, bar size, confinement bars, "to be designed" | partial: total area and cover, fixed 3 x 3 bars | row 2.2, G11 |
| V2.5 fixed supports; patterns DEAD and LL; distributed loads on the beams; Previous Selection | done | rows 1.13, 1.19, 2.3 |
| V2.6 hinge property Moment M3: deformation controlled, symmetric five-point moment-rotation curve, drops to zero beyond E, acceptance criteria | done for this single-component hinge | row 2.4 |
| V2.7 "Use Yield Moment": the program computes the strength that scales the curve | blocked: evidence. Neither the v19 nor the v24 material prints a formula for the computed yield value, for steel or for concrete; the strength is entered | G45 |
| V2.8 hysteresis type of a hinge (Isotropic, Kinematic, Takeda, Pivot, Concrete with an energy degradation factor) | open: a hinge has no hysteresis model; a hinge that reverses after yield is refused | G13 |
| V2.9 column hinge of type Interacting P-M3: one axial force, one angle, the default interaction surface | blocked: evidence, also after the re-check in v19 and v24 when the sixth wave was planned (see G12). This is the item the video depends on most; a moment hinge of entered strength stands in for it | row 2.4, G12 |
| V2.10 hinge assignment through the v26 form: distribution type "Nonlinear Beam Column", expected hinge length 0.3 m, a hinge at each end | partial: hinges are assigned by relative distance, the v19 form. The v26 form and where it puts a hinge of a given length are not in the vendor material | row 2.5, G46 |
| V2.11 generated hinge properties named by frame and number (16H1, 16H2), "Show Hinge Details", "Show Generated Props", hinge labels in the view | done since wave 6 (`gui/hinges.rs`: `the_list_has_generated_rows_only_when_asked_and_gives_the_columns`, `the_view_labels_hinges_with_name_and_property_when_the_option_is_on`; `sap-app/tests/video2_pushover_workflow.rs`: the 50 generated properties of the frame). The label text is a project choice | G47 |
| V2.12 gravity as nonlinear static cases, LL continuing from DEAD | done | rows 2.5 to 2.13 |
| V2.13 Modal case: Ritz vectors, 12 modes, mass source; stiffness at the end of the nonlinear case LL | done since wave 6 while no hinge has yielded at the end of that case, which holds under gravity (`wave6_end_stiffness.rs`, `wave6_end_stiffness_pdelta.rs`; `video2_pushover_workflow.rs`: the periods of both variants). A yielded hinge is refused | rows 1.20, 1.21; G48, G48b |
| V2.14 pushover case: acceleration UX, continuing from gravity, P-Delta, displacement control with the conjugate displacement, monitored U1 at the top joint, multiple saved states (10 to 100) | partial: since wave 6 P-Delta runs with hinges at member ends when every case of the chain has it (`wave6_pdelta_hinges.rs`; `video2_pushover_workflow.rs`, variant B). The video's gravity cases have no geometric nonlinearity, and a chain with mixed settings is refused. A case that stops before its target keeps its states and is marked "not finished" | rows 2.5 to 2.13; G14, G14b, G16, D68 |
| V2.15 nonlinear parameters: total and null steps, event lumping tolerance, iteration limits and tolerance; in v26 also a solution scheme and a number of events per step | partial: the step limits are honoured, and in a P-Delta case one iteration limit and the tolerance; events are exact, not lumped; the other values have no effect. In v19 the events per step apply to time histories only; the "solution scheme" is in the v24 help | D14, G50 |
| V2.16 pushover in a mode shape: load type Mode, mode 1, and a second case with mode 2 | done for the load; the push may have the mirror sign of the vendor's. In the video it rests on the Modal case of V2.13 | rows 2.5 to 2.13 |
| V2.17 mass source from load patterns: 1.0 DEAD and 0.25 LL | done | row 1.21 |
| V2.18 mode shapes with their periods; deformed shape step by step with coloured hinges | done. The video's colour bar is lettered A to E (hinge state); the legend here has the seven classes of the v19 manual (B to IO up to beyond E) | rows 2.5 to 2.13 |
| V2.19 Show Static Pushover Curve: base shear against monitored displacement | done; since wave 6 also for a case that stops early, marked "not finished" | rows 2.5 to 2.13; G16 |
| V2.20 the pushover capacity curve as a table with hinge counts; export of the table to Excel | partial: the table is in the pushover window; it cannot be copied or exported | rows 2.5 to 2.13; G49 |
| V2.21 a scale factor of -1 to push the other way | done | rows 2.5 to 2.13 |
| **All 21** | **12 done, 6 partial, 1 open, 2 blocked** (before the sixth wave: 10 done, 7 partial, 2 open, 1 not classified, 1 blocked) | |

Mentioned in the video and not used: fiber hinges and distributed
plasticity, force-controlled hinges, moment-curvature hinges, P-M2-M3,
additional controlled displacements, P-Delta with large displacements.

## The CSI Spotlight videos

The vendor's video page for SAP2000 lists 46 videos (catalogued on
2026-10-15 in `video-catalog.json`, none of them watched then). Its first
group, "CSI Spotlight", has five. They were read on 2026-11-18 from their
transcripts only; no frame was looked at, so values in the forms that are
not spoken are not covered. The other 41 videos were read afterwards: two
in the next section and 39 in "The other 39 vendor videos".

| Video | What it shows | Here |
| --- | --- | --- |
| Initialization of New Models (3 min 55 s) | a new model takes its units and definitions from the program defaults, from an existing model file, or from a file of saved settings (SDBI); the default display units and decimal places are set under Options | the defaults with a choice of units are done (row 1.1). From an existing file: absent (G51; v19 help topic "New Model"). The settings file is in neither the v19 nor the v24 help, so it has no reference. The form of default display units: absent (G52) |
| SAP2000 Extrusion Features (4 min 20 s) | linear replication of a truss; points extruded to frames, lines to areas, linearly and radially, with a property for the new objects; areas to solids is named | linear and radial replication are done; radial extrusion of lines to areas is done (row 4.4). Linear extrusion, extrusion of points to frames and of areas to solids, extrusion along a path and the conversion commands of the same menu: absent (G53; v19 help topic "Extrude") |
| How to use the Copy/Paste Assigns Tool (2 min 53 s) | the assignments of one object (section, springs, loads) are copied and pasted to objects of the same type, with a choice of the kinds of assignment and of the load patterns; pasted assignments replace, they do not add | absent (G54; v19 help topic "Copy Assigns") |
| SAP2000 v23.0.0 Enhancements (9 min 42 s) | cloud licensing; cold-formed steel design to Eurocode 3; concrete design to ACI 318-19; database tables through the API; seismic loads and spectra of the Vietnamese and Korean standards; wind of NBCC 2015 | outside the v19 baseline (G55). The v24 material has a design manual for ACI 318-19 |
| SAP2000 v25.0.0 Enhancements (10 min 8 s) | a steel grade in the Chinese library; presets for hinge assignment (nonlinear beam-column with an expected hinge length, distributed plasticity, equal spacing, continuous spring support); ASCE 7-22 and NBCC 2020 loads; AS 4100-2020; a model explorer pane | outside the v19 baseline (G55). The hinge presets are the form the second video uses (G46): the video says what each preset is for and not where a hinge of a given length is placed, and the v24 help does not have the form either |

## The vendor's pushover and P-Delta videos

Two videos of the vendor's page, both from 2011, read on 2026-11-18 from
their transcripts and from frames of the screen (ten and nine frames). They
show SAP2000 v15, older than the v19 baseline; every form seen in them is
also in the v19 help. As before, a status is that of the row or ledger item
that covers the feature, and no number in a video is a reference. **Neither
model was built or run here.**

"Static Pushover Analysis" (10 min 39 s): a one-storey steel frame, sized
by steel design, gets automatic hinges and is pushed by an acceleration.

| Step | Status | Covered by |
| --- | --- | --- |
| P.1 sections from Auto Select lists, sized by a linear analysis and steel design | done | rows 1.6, 1.26 |
| P.2 automatic hinges "From Tables In FEMA 356": Table 5-6 for steel beams and for steel columns, component type, degree of freedom (M3 here; the column form also offers M2-M3, P-M2, P-M3 and P-M2-M3), the axial force of a column taken from a case | blocked: evidence. The tables are not in the vendor material of either version; the v24 help names the ASCE 41-13 and 41-17 tables and does not print them. This is the first thing that stops the video's model | G12 |
| P.3 the generated properties, reviewed under Define and labelled in the view ("6H1(Auto M3)"): the program has filled in the moment and rotation scale factors | done since wave 6 for the generated properties and labels of user hinges (G47); the automatic ones fall with P.2, and a computed yield value is blocked (G45) | G47, G45 |
| P.4 hinge overwrites: subdivide at hinges with relative length 0.02 | done | rows 2.5 to 2.13 |
| P.5 the overwrite "Do Not Allow Hinges To Drop Load" (not ticked in the video) | done since wave 6 for a curve that never drops, where it changes nothing; a curve that drops is refused with it | G56 |
| P.6 DEAD as a nonlinear case and the pushover continuing from it; **no geometric nonlinearity** | done. The vendor's own pushover video does not use P-Delta | rows 2.5 to 2.13 |
| P.7 acceleration UX with scale -1, "which will result in positive X displacement" | done; the same sign as here (the load is minus mass times acceleration) | rows 2.5 to 2.13 |
| P.8 displacement control with the **monitored** displacement (not the conjugate one), U1 at a joint, a magnitude | done | rows 2.5 to 2.13 |
| P.9 multiple states, minimum 10 and maximum 100, positive increments only | done | rows 2.5 to 2.13 |
| P.10 deformed shape step by step with coloured hinges; the legend is lettered B, IO, LS, CP, C, D, E | done; the same seven classes | rows 2.5 to 2.13 |
| P.11 pushover curve, base shear against monitored displacement: it drops sharply and runs on to the target | done for the curve. Where several hinges lose strength the path may differ (D14) | rows 2.5 to 2.13 |
| P.12 ATC-40 capacity spectrum: its parameter form, the performance point and the demand-capacity table | blocked: evidence | G15 |
| P.13 Display > Show Hinge Results: the path of one hinge against its backbone, step by step, with state, status and the largest plastic deformation | done since wave 6 (`gui/hinge_results.rs`); without the File and Select Hinge menus of the vendor form | G57 |
| G58 | Select menu: by polygon, intersecting polygon and line; specified coordinate range and 3D box; click a joint in a plane; lines parallel to a line or axis; by material, cable, tendon, solid and link property; by joint supports and constraints; using tables; invert; deselect; by groups and labels (the model has no group object) | 39 videos | open: 12 features, section 2.1 of `vendor-video-gaps.md` | not assigned |
| G59 | Drawing, editing and display aids: drawing in a 3-D view from a free point; editing joint coordinates in the information form; Align Points; Reshape mode; Trim/Extend Frames; show and merge duplicates; Change Labels; two windows with their own cases; Clear Display of Assigns; the Set Display Options form (fill, shrink, colours, local axes, hide links); the taper of a non-prismatic member in the extruded view; secondary beams normal to an edge | 39 videos | open: 14 features, section 2.2 (four more rows there are G10, G27, G29, G53) | not assigned |
| G60 | Area meshing: the other six modes of Automatic Area Mesh (maximum size, points on edges, cookie cuts by lines and points, general divide, meshing group, sub-mesh) and Edit > Divide Areas; generation of edge constraints (D73) | 39 videos | open: 4 features, section 2.3 | 8 |
| G61 | Sections and materials: the Section Designer (shapes, reinforcement, properties, interaction surface, moment-curvature); a form for non-prismatic sections; reinforcement bar sizes; nonlinear material data; layered shell sections; other items of section 2.4 | 39 videos | open: 17 features, section 2.4 | not assigned |
| G62 | Frame assignment forms: Insertion Point (the solver and the API carry it, there is no form; mirror about local 2 is refused), End Length Offsets with "automatic from connectivity", Automatic Frame Mesh, panel zones, others of section 2.5 | 39 videos | open: 7 features, section 2.5 | not assigned |
| G63 | Special objects: cable geometry (shape finder) and cable loads with target force; tendon sections, objects and loads; drawn links; triple-pendulum isolator and damper forms; foundation properties and assemblies (v24 help only) | 39 videos | open: 18 features, section 2.6; models that mix cables and frames are refused | not assigned |
| G64 | Load patterns and automatic loads: wave loads; joint patterns (names, assignment by multipliers or by a zero-pressure level); ground displacement loads; time-history function generators (sine, ramp and others); code response-spectrum functions; several named mass sources; the items of automatic wind that are not G25 | 39 videos | open: 19 features, section 2.7 (nine more rows there carry G07, G21, G23, G25, G33, G42, G55) | not assigned |
| G65 | Case types and analysis options: nonlinear staged construction (with groups); steady-state; power spectral density; the load optimizer; Model Alive; merging of results and running cases in parallel; frequency shift and cutoff of the modal case; directional combination and damping types of the response-spectrum case; time integration methods and damping by period; buckling-mode member forces | 39 videos | open: 26 features, section 2.8 (five more rows there carry D05, D26, D72, G39 to G41, G44, G48). Steady-state and power-spectral-density cases are refused today | not assigned |
| G66 | Results display: generalized displacements and section cuts; shell layer stresses; response-spectrum curves; assembled joint masses; time-history envelopes; the graph, wire frame, absolute displacements and compression of the multi-step video; load assignment display | 39 videos | open: 8 features, section 2.9 (eight more rows there carry D25, D45, G01 to G05, G08) | not assigned |
| G67 | Tables, reports and exchange: the interactive database editing window (the API has 19 editable tables, no joint-load table); the table format, filter and sort window with saved formats; Excel and Access export and import; the report writer; DXF import with layers; the API as a program another application can drive | 39 videos | open: 29 features, section 2.10 (six more rows there carry G35, G49, G55) | not assigned |
| G68 | Design: concrete shell reinforcement design (parameters, display components); the design items of section 2.11 that are not G20 or G26 | 39 videos | open: 2 features, section 2.11 | not assigned |
| G69 | Combinations of time-history cases: the maximum and minimum over time of every quantity (frame forces, reactions, area and solid results), which the vendor's combinations of history cases use; only joint displacements are stored over time today | review of 2026-11-18 (D80) | open: until then such a combination is withheld with a reason | 9 |
| P.14 "the program has automatically softened the drop": the hinge path leaves the backbone where strength is lost | done as the slope limit of the Analysis Reference ("Strength Loss"), checked on Example 1-026 only | rows 2.5 to 2.13 |
| P.15 moment diagrams step by step | not checked here | G57 |

With user-defined M3 hinges of entered strength in place of the automatic
ones, every other step of this video except P.12 has a counterpart.

"P-Delta Analysis" (10 min 17 s): one fixed column with a vertical and a
lateral load at its top, in a model without hinges.

| Step | Status | Covered by |
| --- | --- | --- |
| D.1 a nonlinear static case with P-Delta that applies both patterns | done | Example 1-016 |
| D.2 joint read-out and the frame diagram form, two windows side by side | done; windows side by side were not looked for | row 1.23 |
| D.3 an "initial P-Delta" case with the dead load only, then a linear static case on the stiffness at the end of it | done | Example 1-016 and the cases that name an initial case |
| D.4 a modal case on that stiffness; the period lengthens | done | Example 1-017 |

The column has a closed form (a cantilever beam-column). Since wave 6 a
compressed cantilever is checked against it, with a directly specified
P-Delta force and in a P-Delta case (`wave6_hinge_end_kink.rs`,
`wave6_pdelta_hinges.rs`: the column in 8 and 16 objects against the exact
solution); those columns carry a hinge that has not yet yielded. The
video's own column, its linear case on the P-Delta stiffness and its modal
case were not built.

What these two videos change for the plan: the vendor's pushover does not
need P-Delta with hinges (G14) or the conjugate displacement, and the
second video's use of both is that author's choice; the stiffness after a
nonlinear case (G48) is the vendor's recommended way to bring P-Delta into
modal cases; and a window for the results of one hinge was missing (G57,
added in wave 6).

## The other 39 vendor videos

With these, all 46 videos of the vendor's page have been read. The reading
(two passes by reading agents, transcripts and about 290 frames), the table
of the 39 videos, the 191 features found absent or partial and what could
not be settled are in [vendor-video-gaps.md](vendor-video-gaps.md). The
statuses there are the readers' findings with their evidence; the reviewer
checked a sample only. In short:

- **What limits each video most:** a missing feature in 29 videos, a partly
  present one in 6, a refused case type in 3 (tension-only bracing with
  subdivision, steady-state, power spectral density); licensing (1 video) is
  not a goal.
- **Whole videos without a counterpart:** staged construction; steady-state
  and power-spectral-density analysis; wave loads and automatic wind loads;
  tendons and the cable shape finder; layered nonlinear shells with
  generalized displacements and section cuts; the Section Designer; the load
  optimizer; concrete shell design; the report writer; interactive database
  editing and the table format window; DXF import; Model Alive; merging of
  results.
- **Partly present:** insertion points and non-prismatic sections (solver
  and files, no form); automatic area meshing (one mode of seven); the
  multi-step video (no graph, wire frame or absolute displacements); the API
  (a Rust facade, not a program that Excel can drive).
- **Versions:** 21 videos show v15, 6 show v16, 1 shows only an installer of
  v17, 4 show v19; 7 could not be read and look newer; none shows a version
  above v19.2.1. So most forms
  were read from an older program than the v19 baseline; each feature was
  looked up in the v19 help, and the gap list says where the help has it.
- **Accepted and not used:** the area edge constraint (D73). Five type names
  are stored with nothing behind them (wave, wind and prestress patterns,
  Section Designer sections, layered shells); whether a message appears for
  each is to be checked when its area is planned.

Of the 191 features, 35 were already in the ledger and 156 were not. The
156 are grouped by area as G58 to G68 below; the gap list has every row.
None of them has been planned: for each the vendor material still has to
be searched for a rule and a reference, as for every item of this file.

## The v24 material

The user supplied an installer of SAP2000 v24.0.0. It was unpacked without
being run (the container and the installer database were read as data); the
programs and a licence patch that came with the package were left out.
Outside the repository, and never to be copied into it, there are now 983
files: 308 PDF manuals and examples, three help files and 407 example
models.

What it adds to the v19 corpus, as far as looked at on 2026-11-18:

- the Analysis Reference in a later revision (Rev. 18, November 2017, 569
  pages; the v19 one is Rev. 15, July 2016, 556 pages);
- a steel design manual for AISC 360-16 with two verification examples
  (G20), and the Lateral Loads Manual with ASCE 7-16 seismic and wind
  sections (G21). The site-coefficient and importance tables are still
  named and not printed in the passages looked at (G22);
- a concrete design manual for ACI 318-19 and other later codes;
- the help of v24, which defines a "solution scheme" for nonlinear static
  cases (iterative, iterative event-to-event, event-to-event only) and what
  the step limits mean under each (G50, G16), and which has the forms of a
  parametric P-M2-M3 hinge with a technical note on its theory (G12);
- the verification suite of v24. Problems 1-027 to 1-030 are not hinge
  problems.

It does not have the hinge assignment form of the second video (G46).

**Rule of use.** Every citation names its version. Where the v19 material
is silent and the v24 material states a rule, the rule may be used and the
feature is marked as resting on v24. Where the two differ, v19 is kept and
the difference is recorded. Statuses in the ledger were set against v19; a
row that is "blocked: evidence" is re-checked against v24 when its wave is
planned, and keeps its status until then.

## Ledger

Every problem found and every item left undone, with one stable identifier
each. "Wave" is the wave that closed it or is planned to. The evidence for
each status was checked against the vendor material when the fifth wave was
planned (manual pages, help topics and a scan of the 277 native example
files). A row closed by the fifth wave names the test that proves it, as
file and test; test files are under `crates/sap-solver/tests/` and window
tests under `crates/sap-app/src/gui/` unless a crate is named, and "opt-in"
tests need the vendor corpus. An identifier with a letter (D03b) is the part
of an item that its wave left open.

Counts after the fifth wave, by counting the rows below. Defects and
differences, 69 rows: 43 done, 4 partial (D03, D06, D09, D17), 19 open, 3
blocked for evidence. Features and options, 45 rows: 5 done, 27 open, 11
blocked for evidence, and 2 with evidence for a part only (G25, G35), which
are counted apart. "Done" for D19 and D51 rests on a reading, not on a
test, and for D12 on a measurement; their rows say so.

Added on 2026-11-17 from the second video and not in the counts above: one
difference (D68, open) and seven features (G44, G47 and G49 open; G48 open
for a part and not classified for the rest; G45 and G50 not classified; G46
blocked for evidence). With them the tables below have 70 rows of defects
and differences and 52 of features and options.

Added on 2026-11-18 from the CSI Spotlight videos, also not in the counts
above: five features (G51 to G54 open; G55 outside the v19 baseline). The
feature table then has 57 rows. From the vendor's pushover video, the same
day: G56 (not classified) and G57 (open), 59 rows. From the other 39 vendor
videos: G58 to G68, eleven rows that group 156 features of the gap list; 70
rows. The sixth wave split three items (G14b, G16b, G48b): 73 rows. Defects found on 2026-11-18 and not in the counts above: D69 to D78
(84 rows of defects and differences). A second outside review, of the
repaired tree, gave six more on 2026-11-18, repaired on 2026-11-19: D79 to
D84 (90 rows), and one feature that the repair of D80 leaves open, G69 (74
rows).

### Defects and differences

| Id | Item | Found in | Status | Wave |
| --- | --- | --- | --- | --- |
| D01 | Plane-strain areas: forces are recovered with the plane-stress relation, no out-of-plane stress S33 is stored, and there is no anti-plane shear stiffness (Analysis Reference pp. 217-218) | 4 | done: plane-strain recovery, S33 and anti-plane shear stiffness (`wave5_area_materials.rs`: `plane_strain_patch_recovers_s11_s22_s33_s12`, `plane_strain_has_anti_plane_shear_stiffness_and_recovery`). The stored von Mises is D44; the display of plane sections is G43 | 5 |
| D02 | The section's material angle rotates an area's output axes; the vendor rotates only the material. No corpus area section has a nonzero angle (285 records scanned), so the references are closed forms | 4 | done: the angle turns only the material (`wave5_area_materials.rs`: `orthotropic_material_angle_rotates_the_material`, `isotropic_material_is_invariant_under_the_material_angle`) | 5 |
| D03 | The singular-matrix check tests each pivot against that degree of freedom's own diagonal, which lets a rigid rotation of a shell model through at some thicknesses; its message names a pivot index instead of a joint and direction | 4 | partial: the message names the joint and the direction (`wave5_stability.rs`: `r1_a_singular_torsion_pair_is_named_at_its_joint_and_dof`), or the frame and its end joints for an internal joint of an automatic subdivision (`wave5_unstable_frame_name.rs`: `an_unstable_subdivided_frame_is_named_by_its_frame_and_end_joints`). Which models are refused did not change; that part is D03b | 5 |
| D03b | Mechanisms that pass the pivot test are accepted (a rigid rotation of a shell model at some thicknesses). Measured in wave 5: the smallest eigenvalue of the stiffness scaled by its structural diagonal separates sound models (smallest 1.66e-12) from mechanisms by about four orders without penalty constraints (at most 1.7e-16), and not at all with a diaphragm penalty (up to 4.5e-7), so no threshold was enabled. `linalg::scaled_stiffness_minimum` is written, tested and called nowhere | 4 (as part of D03), 5 (measured) | open: needs the exact constraints of D08 | 7 |
| D04 | The stored von Mises field of multi-valued combinations is combined field by field (a symptom of D05) | 3 | done: derived fields are 0 in both rows of a multi-valued result and "n/a" in the tables (`sap-solver/src/combos.rs`: `derived_fields_are_zero_for_multi_valued_results`) | 5 |
| D05 | Combinations store one value. The manual defines every combination as a maximum and a minimum (pp. 351-353): today Range is added linearly, Envelope keeps the value of larger magnitude, and no minimum is stored for any type | 4 | done: every multi-valued combination stores a maximum and a minimum (`sap-solver/src/combos.rs`: `manual_p353_comb1_and_comb2`, `manual_p353_axial_forces`, `range_combination`; `wave5_combinations.rs`: `solver_spectrum_case_gives_the_manual_pairs_of_comb1_and_comb2`) | 5 |
| D06 | Examples 6-006 and 6-007 from the vendor files: periods 0.6 to 3.5 % from the published ones, unexplained | 1 | partial: the rigid-zone repair (D21) and the Ritz repair (D37) explain most of it. With the Ritz starting-load entry (D36) removed the periods are 0.437263, 0.135023, 0.073709 s after all solver repairs of the wave for the published 0.438, 0.135, 0.074 (−0.17 %, +0.02 %, −0.39 %): T2 and T3 inside the printed rounding, T1 0.0002 s below it, not explained (opt-in `wave5_rigid_zones.rs`: `vendor_6_006_and_6_007_periods_follow_the_published_values_with_full_length_axial_stiffness`, bound 1 %). As imported the Modal case is now refused (D36). Open candidate for the remainder: the vendor's link starting vectors, part of G41 | 5; the remainder with G41 (9) |
| D07 | Native import ignores the case record's modal case and motion type, so a vendor modal history with zero damping (6-001, 1-015) is integrated directly | 1 | done: the modal case is imported and written (`sap-format/src/sdb.rs`: `native_modal_history_import_branches`; `sap-format/tests/wave5_native_history.rs`: `a_linear_history_keeps_its_modal_case_through_the_native_writer`; opt-in `wave5_vendor_history.rs`: `example_6_001_modal_histories_against_the_published_values_and_the_closed_form`) | 5 |
| D08 | A diaphragm is a penalty constraint; with a nearly rigid axial area a collapse load through it loses several digits. The manual describes internal master joints (pp. 66-67) | 2 | open | 7 |
| D09 | Frame station deflections have no kink at an interior hinge | 2 | partial: the kink of M2 and M3 hinges is added (`wave5_hinge_stations.rs`: `r1_single_object_stations_have_the_hinge_kink`, `r2_two_objects_give_the_same_deflections`). The missing part is D09b | 5 |
| D09b | Station deflections have no jump at an axial, shear or torsion hinge | 5 | open | not assigned |
| D10 | The plot window rebuilds its traces on every frame | 4 | done: traces are cached by result revision and setup (`time_history.rs`: `the_plot_traces_are_cached_by_the_results_revision_and_the_setup`, which counts builds; there is no reference value) | 5 |
| D11 | The plot window has its own copy of the solver's time-function rule | 4 | done: the copy is deleted and the window calls `sap_solver::time_function_value`; covered by the existing `time_history.rs`: `load_function_trace_equals_the_solver_ground_acceleration` | 5 |
| D12 | Ritz vectors of an axisymmetric model: pairs after the first two differ by 1e-6 to 3e-3, and a 20-vector run contains harmonics the loads do not excite. The planner's reading: expected from the recurrence (a power iteration amplifies roundoff in other harmonics), to be confirmed by measurement | 4 | done: closed as expected behaviour by measurement, not by a reference value. The vectors are orthonormal to 1e-15, the content of other harmonics grows by 1e2 to 1e3 per generation cycle, and the harmonics n ≥ 2 of the test shell are softer than n = 1 (`wave5_ritz_symmetry.rs`: `ritz_modes_are_m_orthonormal_and_the_harmonic_content_is_measured`, which asserts the orthonormality only) | 5 |
| D13 | A yielded hinge without acceptance criteria has status "B to IO"; a hinge exactly on a point shared by two segments reports the later segment | 2 | blocked: evidence (pp. 163-164 give no rule; project choice) | — |
| D14 | With several hinges losing strength at once the path may differ from the vendor's; events are not lumped | 2 | blocked: evidence (one verification example, 1-026) | — |
| D15 | A combination that loses its shell, solid or link results says so only in the result display, not in the tables | 3 | done: the Area, Solid and Link tabs name the case (`wave5_windows_tests.rs`: `an_empty_area_table_of_a_combination_names_the_case_that_dropped_it`) | 5 |
| D16 | A library concrete without rebar yield stress falls back to 60 ksi in concrete design | 3 | done: refused (`sap-design/tests/wave5_concrete_fy.rs`: `a_library_concrete_without_rebar_yield_is_refused_naming_section_and_material`, `a_rebar_material_without_fy_is_refused_too`) | 5 |
| D17 | No window of any wave had been used by hand; the windows are tested with synthetic input only | 1 to 4 | partial: since the fifth wave the real program is run on a private display by an agent, with synthetic clicks, keys and drags, and looked at through screenshots. No person has used it, and no screenshot is kept. That found D32, D33, D41, D52 to D55 and D57 to D59. Gone through: the four parts of the video from the templates to the results (grid, quick draw, section import, loads, diaphragms, mass source, seismic form, Ritz case, run, deformed shape with its title, joint read-out, modal tables, design combinations and steel design, Object Information; 2D frame template, reinforced section, hinges, chained nonlinear cases, hinge states, pushover curve; function from file, modal history, plot window, multi-step video; curved frame, radial extrusion, reversal of local 3, library material, shell stresses with legend), Save as S2K and Open. Seen earlier in the wave by the packages, on models of their own: the deformed-shape form, the joint read-out, the modal tables, the shell form with contour and legend, the seismic window with an overwrite and its moment line, the first mode of the shell dome, Object Information on a vendor example, the Grid Only button, the I-section dialog, the cyclic form of Create Video and one recorded frame, the grid editor, the section dialog with reinforcement, Set 2D View. Seen by the repairing agent after the hand check: Max and Min rows of a combination with a response spectrum in the tables, the cyclic video, the I-section dialog with unequal flanges, Plane Up and Down, mouse drawing of frames and areas on a plan view, vendor files through Open, the Messages window, the rigid arms and the face diagram of Example 1-010b. **Not looked at:** the arrow of local 3; resizing of the window and the panes (the display has no window manager); the file chooser behind Browse; the Bottom and Visible Face choice of the shell form; the plot redraw on a change of the time range and the stepping of an animation, by eye; the moment lines of the seismic window and the Z moment of the base reaction on the first part's model; the refusal of a derived shell quantity of a multi-valued combination (the form refuses earlier, D57); the Max and Min curves of the frame diagram; radial replication of joints; the displacement-contour option; the verification run from the Help menu; the clipboard of the Copy button. The items of the hand check's first pass (parts 2 and 3) rest on the agent's notes, not on kept screenshots | 5; the rest not assigned |
| D18 | The I-section dialog has no separate bottom-flange fields | 4 | done (`i_section.rs`: `an_i_section_with_unequal_flanges_has_the_closed_form_properties`; `wave5_windows_tests.rs`: `the_i_section_dialog_makes_the_unequal_flange_section_and_refuses_a_wrong_dimension`) | 5 |
| D19 | Window code and agents' tests of waves 3 and 4 were not read line by line in review | 3, 4 | done as a review, not by a test: the window code was read in the fifth wave, with the coverage and its limits stated in the wave document (`gui.rs` as diffs, some files sampled, tests by delegated audits). Its findings are D34 and D35 | 5 |
| D20 | Area elements ignore orthotropic constants: Example 2-015a is analysed as isotropic with E1 | 5 (planning) | done (`wave5_area_materials.rs`: `orthotropic_membrane_and_bending_at_zero_angle`; opt-in `wave5_area_materials_vendor.rs`: `example_2_015_orthotropic_model_matches_published_deflection_and_model_b`) | 5 |
| D21 | The rigid-zone factor shortens the axial and torsional length too; the manual says it never affects axial and torsional deformation (p. 130) | 5 (planning) | done (`wave5_rigid_zones.rs`: `r5_r6_axial_and_torsional_stiffness_use_the_full_length`, `r5c_a_non_prismatic_bar_adds_the_end_zones_with_the_end_sections`). The rule for non-prismatic sections was replaced by that of D47 | 5 |
| D22 | Constraint equations at joints with local axes are written in joint-local directions | 5 (planning) | done: constraints act in global directions; the Local type stays joint-local (`wave5_constraints_rotated.rs`: `r9_a_diaphragm_ignores_the_local_axes_of_a_constrained_joint`, `r10_body_and_equal_constraints_ignore_the_local_axes_too`) | 5 |
| D23 | Direct integration uses a different time-function rule (first value held before the first point) and evaluates a missing function as 1.0 | 1 (documented), 5 (id) | done: one rule, zero before the first point; a missing function is an error (`wave5_direct_functions.rs`: `r11_a_function_that_starts_at_one_second_loads_nothing_before_it`). Spectra keep their own rule | 5 |
| D24 | Design takes a multi-valued contributor of a design combination with its positive sense only | 5 (planning) | done: both senses; an Envelope by its contributing states (`sap-design/tests/wave5_design_combos.rs`: `additive_with_a_spectrum_case_is_designed_at_minus_3_and_minus_17_kip`, `envelope_and_range_follow_the_manual_pair`). Which forces of the two rows act together, and Cb, are D49 | 5 |
| D25 | History cases report the state at the step of largest displacement instead of envelopes (p. 351) | 5 (planning) | open | 9 |
| D26 | Native nonlinear modal (FNA) cases run as direct nonlinear integration (6-005a) | 5 (planning) | done: refused by name (`sap-format/src/sdb.rs`: `native_modal_history_import_branches`; opt-in `wave5_vendor_history.rs`: `an_fna_history_is_refused_by_name`). The analysis itself is G41 | 5 |
| D27 | A native periodic modal history runs as a transient (1-015) | 5 (planning) | done: refused by name (`sap-format/src/sdb.rs`: `native_modal_history_import_branches`; opt-in `wave5_vendor_history.rs`: `a_periodic_history_is_refused_by_name`). The analysis itself is G40 | 5 |
| D28 | A released frame end keeps its rigid-zone factor instead of acting as fully rigid (pp. 130-131) | 5 (planning) | done; a partial-fixity spring keeps the factor, a project choice (`wave5_rigid_zones.rs`: `r8_a_moment_release_makes_the_offset_rigid_in_its_plane`, `r8b_a_shear_release_makes_the_offset_rigid_in_its_plane`) | 5 |
| D29 | Modal-history reactions leave out the modal damping force | 1 (documented) | open | 9 |
| D30 | Participation at rotated joints with masses that differ by direction: vendor treatment unknown | 4 (documented) | blocked: evidence | — |
| D31 | Plane sections made in the program or from S2K get the shell drilling penalty (native files switch it off); Example 3-002 publishes the plain results | 5 (planning) | done (`wave5_area_materials.rs`: `plane_section_has_no_drilling_penalty_and_shell_keeps_it`; opt-in `wave5_area_materials_vendor.rs`: `example_3_002_comp_plane_beam_matches_published_values`) | 5 |
| D32 | The deformed shape has no automatic scale: the first mode of the shell dome template is drawn with joints far off the screen at the default factor 25 (seen on the private display) | 5 | done: automatic by default, the largest drawn translation is 10 % of the bounding-box diagonal, a project choice (`result_display/scaling.rs`: `the_automatic_scale_draws_the_largest_translation_as_a_fraction_of_the_diagonal`; `wave5_windows_tests.rs`: `the_automatic_scale_keeps_a_large_mode_on_the_screen_and_a_typed_factor_wins`) | 5 |
| D33 | The view title names the case but not the mode number, period or step that is shown | 5 | done (`result_display/scaling.rs`: `the_title_names_the_mode_the_time_step_or_the_saved_state`; `wave5_windows_tests.rs`: `the_view_title_names_the_mode_with_its_period_and_frequency`) | 5 |
| D34 | Nine defects in the windows of waves 3 and 4, found by the review of D19: the file chooser returned the clicked file after the path was retyped; the deformed-shape form opened on mode 1 or step 0; plane views and custom-grid texts were not converted with the units; the shell form reopened with other units and the old range; Enter and Escape went to the pane under the pointer; the grid editor could not read back an id with a blank; the status line named the wrong face; plane titles printed roundoff | 5 (review) | done (`file_browser.rs`: `a_path_typed_after_a_highlighted_file_is_the_choice`; `result_display/tests.rs`: `ok_in_the_deformed_form_keeps_the_mode_shown`, `ok_in_the_deformed_form_keeps_the_step_shown_and_the_final_state_is_the_last_step`, `reopening_the_shell_form_keeps_range_and_units_together_and_the_status_names_the_face`; `gui.rs`: `a_plane_view_and_picked_points_survive_a_unit_change`, `enter_closes_the_polygon_in_the_pane_it_was_picked_in_wherever_the_pointer_is`, `the_custom_grid_text_follows_the_units`, `the_plane_title_of_the_fourth_level_is_not_printed_with_roundoff`; `build.rs`: `read_current_lines_then_apply_round_trips_ids_with_blanks_and_commas`) | 5 |
| D35 | Eight tests of waves 3 and 4 could not fail or did not check their claim | 5 (review) | done: seven rewritten; the repairing package reports that each fails when its behaviour is broken by a local edit (`result_display/tests.rs`: `the_deformed_form_is_validated_and_the_model_is_only_read`; `gui.rs`: `a_draw_tool_window_is_routed_and_the_old_draw_tools_ignore_windows`, `escape_returns_to_select_and_enter_is_routed_while_a_draw_tool_is_active`, `the_set_2d_view_dialog_renders_and_applies_a_typed_coordinate`; `video_results_e2e/frame.rs`: `a_record_read_through_the_window_is_stored_as_the_function_of_the_file`; `time_history.rs`: `sets_and_functions_are_reset_when_another_model_is_loaded`; `modal_tables.rs`: `ritz_controls_render_in_the_case_editor`). The eighth, the silent skip of opt-in vendor tests without the corpus, is the project's rule and stays | 5 |
| D36 | Native import dropped a Ritz starting-load record that is neither a load pattern nor an acceleration: the vendor's "all link element nonlinear degrees of freedom" in Examples 6-006, 6-007, 6-010, 6-011a and 6-011b | 5 | done: recorded as unmodeled source data, so the Modal case is refused as imported and the approximate periods of 6-006 and 6-007 are no longer reported (`sap-format/tests/wave5_p10_formats.rs`: `the_link_nonlinear_starting_vectors_are_recorded_naming_the_case`; opt-in `wave4_vendor_regressions.rs`: `vendor_sdb_example_6_006_modal_case_is_refused_for_its_link_starting_vectors`). The starting vectors themselves are part of G41 | 5 |
| D37 | The Ritz projection lost its digits on stiff models: more modes than mass degrees of freedom (11 for 10 in Example 6-006), a Ritz period above the exact one, or a failure with a negative eigenvalue | 5 | done: stiffness or flexibility form by a roundoff estimate, and at most as many modes as mass degrees of freedom (`wave5_ritz_bound.rs`: `space_frame_with_stiff_floor_loops`, `grid_with_stiff_beams_below_at_and_above_the_rank`). The static participation in the flexibility form is D50 | 5 |
| D38 | The refusal of concrete design without a rebar yield stress was carried by an error of the steel design workflow | 5 | done: `DesignError::MissingRebarYield` names the frame, section and material (`sap-design/tests/wave5_concrete_fy.rs`: `a_library_concrete_without_rebar_yield_is_refused_naming_section_and_material`) | 5 |
| D39 | The seismic eccentricity overwrites were dropped by S2K and by the native container | 5 | done: S2K table `RUST AUTO SEISMIC ECCENTRICITY`; the native writer refuses them (`sap-format/tests/wave5_p10_formats.rs`: `the_overwrite_table_round_trips_exactly_and_is_written_only_when_present`, `the_native_writer_refuses_overwrites_naming_the_pattern`; `wave5_p10_eccentricity_s2k.rs`: `an_overwrite_gives_the_same_analysis_after_an_s2k_round_trip`). Native persistence is G19 | 5 |
| D40 | The section dialog named the concrete itself as rebar material, so with D16 repaired a library concrete could not be designed from the windows | 5 | done: a rebar material choice in the dialog (`gui.rs`: `a_section_made_in_the_dialog_with_a_rebar_material_is_designed_and_one_without_is_refused`, `the_dialog_says_when_the_model_has_no_rebar_material_and_refuses_the_section`) | 5 |
| D41 | Thin polygons (an area seen edge-on) were drawn with spikes across the whole view, in the undeformed and the deformed shell dome and in recorded videos (seen on the private display). The cause was the drawing of mitred strokes and feathered fills, not the result | 5 | done: areas, solid faces and extruded sections are drawn as a fill fan and one segment per edge (`gui.rs`: `the_shell_dome_draws_no_spikes_in_the_undeformed_and_the_modal_views`; `view.rs`: `an_outlined_polygon_stays_inside_its_corners_even_when_an_edge_turns_back`) | 5 |
| D42 | Steel design computes Cb of a multi-valued combination from the stations of all its states together (the first state's moment shape, the largest moment of any state) instead of per state | 5 | done: absorbed by D49; each state has its own diagram and Cb (`sap-design/tests/wave5_design_states.rs`: `each_state_has_its_own_moment_diagram_and_cb`, `equal_moments_in_both_rows_give_the_cb_of_one_row`) | 5 |
| D43 | The solid element builds the orthotropic compliance with `−u12/e1`, `−u13/e1`, `−u23/e2`; the manual's Eqn. 3 as printed, which the area element follows and Example 2-015 supports, has `−u12/e2`, `−u13/e3`, `−u23/e3` | 5 | open | 8 |
| D44 | The stored von Mises of a plane-strain area ignores S33 | 5 | open (with G43) | 8 |
| D45 | The Create Video form's default magnification was a fixed 10, not the automatic scale of the view | 5 | done: automatic until the box is edited (`p16_misc_tests.rs`: `the_video_magnification_is_the_automatic_factor_until_the_user_edits_it`) | 5 |
| D46 | Span loads and self weight on the rigid part of a frame with end offsets were dropped, and the released-end rule of D28 widened that (fixed-fixed beam of 144 with offsets 12, uniform load 1: reactions summed to 120) | 5 (second review) | done: loads act over the full length; the part on a rigid arm goes to its joint with its moment (`wave5_end_offset_loads.rs`: `d46_sum_of_reactions_equals_the_load_for_every_rigid_end_factor_and_release`, `d46_joint_moment_of_a_uniform_load_on_arm_and_span`, `d46_a_point_load_on_the_arm_goes_to_its_joint`; `wave5_p13_unchanged.rs`: `members_without_offsets_are_bitwise_unchanged`) | 5 |
| D47 | Non-prismatic frames with end offsets: the variation was laid over the flexible length instead of the clear length, and thermal and axial fixed-end forces used the stiffness without the end zones (Analysis Reference pp. 120, 130). Found with it: rigid arms had no geometric stiffness | 5 (second review) | done: variation over the clear length, end zones with the end sections (`wave5_nonprismatic_offsets.rs`: `d47_axial_and_torsional_flexibility_follow_the_clear_length_for_every_rigid_end_factor`, `d47_free_thermal_expansion_is_alpha_dt_l_for_a_non_prismatic_member_with_offsets`); `P × arm` geometric stiffness of each rigid arm, which changes P-Delta and buckling of members with rigid arms (`wave5_rigid_arm_geometry.rs`: `rigid_arms_add_p_times_arm_to_the_joint_rotation_stiffness`). Not done, no identifier yet: the 1 % clear-length rule (p. 129), the mass of non-prismatic members and end zones (p. 134), the geometric stiffness of the flexible part on the flexible length | 5 |
| D48 | Output stations of a frame with end offsets ran joint to joint, so a released end reported the joint moment instead of zero at the support face (Analysis Reference p. 130) | 5 (second review) | done: the stations run from support face to support face (`wave5_end_offset_stations.rs`: `d48_stations_run_from_support_face_to_support_face`, `d48_a_released_end_reports_zero_moment_at_its_support_face`); Examples 1-010b to d moved in station position and M3 (first station 0 → 6 in, −1440 → −1380), by the package's print, not by a test. Two consequences, found by reading and repaired in the same wave: the drawing (D56), and the design code, where concrete beam design found no end station (the minimums of special and intermediate frames stopped applying, unconservative) and the steel Cb sampled outside the stations; design now takes the first and last station as the ends and measures Cb between them (`sap-design/tests/wave5_p17_offset_stations.rs`: `special_moment_frame_floors_act_at_the_support_faces`, `cb_quarter_points_lie_between_the_support_faces`, `members_without_offsets_are_bitwise_unchanged`) | 5 |
| D49 | Design read the Max and Min rows of a multi-valued combination as one station list: no permutation of maxima and minima of the interacting forces (the largest moment was never paired with the smallest axial force), and Cb across the rows (D42) | 5 (second review) | done: each state is a design entry; permutations of P, M2, M3 (with V2, V3 for Eurocode 3; with T, V2, V3 for closed sections under torsion, untested) (`sap-design/tests/wave5_design_states.rs`: `additive_with_spectrum_is_designed_at_every_permutation_of_p_and_m3`, `envelope_range_srss_and_absolute_are_permuted`, `column_pmm_is_designed_at_every_permutation`, `single_valued_combinations_and_plain_cases_are_bitwise_unchanged`) | 5 |
| D50 | The flexibility form of the Ritz projection (D37) gives a frequency that is not `φᵀKφ`, so the static participation of a starting load is not exactly 100 % on stiff models (measured: at most 1.6e-8 at an axial modifier of 1e6, 7.2e-6 at 1e9; 1.0029 and 1.0099 only with the branch forced). The single form the review proposed was built, measured and rejected (the bound tests of D37 fail with it) | 5 (second review) | open: the two forms and the switch between them stay | 7 |
| D51 | Two files of verification rows returned no rows when their analysis failed, so the rows could not fail by an error | 5 (second review) | done by reading, without a test: both now stop with an error, as the older verification files do (`sap-solver/src/bench_w5_p1.rs`, `bench_w5_p3.rs`) | 5 |
| D52 | The explorer printed quantities with a fixed number of decimals: a section read "I33=0.0" in kN, m (seen in the hand check, which took it for a section without stiffness) | 5 (hand check) | done: at least four decimals and five significant digits in the explorer, labels and the small tables (`wave5_hand_check_tests.rs`: `the_explorer_prints_section_properties_with_significant_digits`, `quantity_text_keeps_five_significant_digits_in_any_unit`). The result tables are D58 | 5 |
| D53 | Opening a file kept the display state of the old model: a shell-stress display with "no results", a 2D pane without its plane (seen in the hand check). The reported "Locked" status could not be reproduced | 5 (hand check) | done: a new or opened model starts undeformed, with the automatic scale, unlocked when it has no results; the panes keep a plane the new model has (`p16_open_tests.rs`: `opening_an_s2k_resets_results_lock_display_and_keeps_a_plane_of_the_new_grid`, `opening_a_file_that_cannot_be_read_changes_nothing`) | 5 |
| D54 | A status message longer than the line was cut and could be read nowhere (the reason of a refused pushover run) | 5 (hand check) | done: the whole text is the tooltip of the cut line, and Help > Messages lists the session's messages (`p16_messages_tests.rs`: `the_whole_reason_of_a_refused_analysis_is_readable_on_hover_and_in_the_messages_window`, `the_messages_are_newest_first_bounded_and_the_errors_are_marked`) | 5 |
| D55 | Smaller window items of the hand check: float noise in the duration of a function from a file; the seismic window and Model Workflows taller than the screen; the 2D frame template opening its 2D pane as a plan at Z = 0; the pushover curve window without readable ticks and with a cut label; the title of a video frame cut | 5 (hand check) | done (`p16_misc_tests.rs`: `the_duration_of_a_function_read_from_a_file_has_no_float_noise`, `big_windows_open_inside_a_1440_by_900_screen_and_scroll`, `the_pushover_plot_has_readable_ticks_and_every_label_inside_the_plot`, `the_title_of_a_narrow_pane_wraps_inside_the_pane`; `p16_open_tests.rs`: `the_2d_frame_template_opens_its_2d_pane_on_the_frame_and_a_3d_template_does_not`, `a_planar_model_opens_the_2d_pane_on_its_plane`) | 5 |
| D56 | With the stations at the support faces (D48) the deformed line of a frame with end offsets stopped at the faces and left a gap to the displaced joints | 5 (consequence of D48) | done: the rigid arms are drawn (`p16_offset_tests.rs`: `the_deformed_shape_runs_from_the_joint_through_the_face_to_the_tip`, `the_moment_diagram_starts_at_the_face_with_nothing_inside_the_offset`) | 5 |
| D57 | A response-spectrum case stores no area results, so the shell form refuses a combination with one for "no shell results" (seen in the hand check) | 5 (hand check) | open | 8 |
| D58 | Result-table cells have a fixed number of decimals: a displacement of 1e-5 m reads "-0.00001" | 5 (hand check) | open | 11 |
| D59 | After Unlock an open shell form still shows its old overlay | 5 (hand check) | open | 11 |
| D60 | End offsets are not reduced when the clear length would fall below 1 % of the member length (Analysis Reference p. 129); only a clamp at 1e-6 of the length exists | 5 (frame repairs) | open | 8 |
| D61 | The mass of a non-prismatic member, and of a member with end offsets, is taken from the section at mid length, half to each joint, not as p. 134 describes | 5 (frame repairs) | open | 8 |
| D62 | The geometric stiffness of a member with end offsets uses the flexible length for its bending part; no reference was established for it (the P x arm term of the rigid arms was added in wave 5) | 5 (frame repairs) | open | 8 |
| D63 | The permutation of T, V2 and V3 for closed sections under torsion (AISC H3-6) in the design of a maximum/minimum combination has no test | 5 (design repair) | open | 10 |
| D64 | Some API result readers return only the Max row of a maximum/minimum combination (the seven extraction functions with a step type return both) | 5 (second review) | open | 12 |
| D65 | The S33 of a plane-strain area is stored but no table, API call or display reads it | 5 (end-to-end tests) | open | 8 |
| D66 | Example 2-015 model B stores the bending modifiers as m11 = 1.0, m22 = 0.5 where the problem text says m11 = 0.5, m22 = 1.0; the order in which the importer reads the modifiers is not verified against a second source | 5 (area materials) | open (investigation) | 12 |
| D67 | A fitted model uses about 0.62 of its pane's width, and the program starts in the M3 diagram display, where a right-click on a frame opens the inspector instead of Object Information | 5 (hand check) | open | 11 |
| D68 | In a nonlinear static case of a model **without** hinges (Wen links, tension or compression limits) the "use conjugate displacement" option of displacement control is stored and written back but is not used: the monitored displacement alone measures the step. A model with hinges uses it since wave 2 (the conjugate displacement measures the step, the monitored one sets the target). The Analysis Reference (p. 430) defines the conjugate displacement as the average of all displacements weighted by the load on each degree of freedom | second video (recorded before in `solver-wen.md`) | done as a documented difference: on a path that rises monotonically the search of a model without hinges reaches the same states, and three tests pin both paths down (`wave6_conjugate_control.rs`). Not tested: a path that does not rise monotonically without hinges | 6 |
| D69 | A moment hinge at end I of its analysis element loses the P-Delta of its own plastic rotation when the element has a geometric stiffness (today: a directly specified P-Delta force). Found by a probe when the sixth wave was planned: a column with 500 kN of compression keeps its load of 30.795 kN out to 0.3 m with the hinge at end I, and falls as (Mp − P u)/h with the hinge at end J. The input was accepted | 2 (found in planning wave 6) | done: the jump of such a hinge leaves out the rigid tilt of the part beyond it; a hinge inside an element with a geometric stiffness is refused (`wave6_hinge_end_kink.rs`: both orientations against (Mp − P u)/h, and the column in 8 and 16 objects against the exact beam-column) | 6 |
| D70 | A linear or modal case on the stiffness at the end of a nonlinear static case **without** geometric nonlinearity still gets a geometric stiffness from that case's axial forces. The Analysis Reference (v19, p. 411) lets only the geometric nonlinearity "considered in the nonlinear analysis" affect the linear results. Probe: a column with 2000 kN of compression gives 142.07 rad²/s² instead of the elastic 222.22 | before the video work (found in planning wave 6) | done: the geometric stiffness is taken only from a case that considered P-Delta (`wave6_end_stiffness.rs`: 222.2222 and 142.0676 rad²/s² in closed form; `area_assignment_regressions.rs` for a model with areas). A scan of 338 vendor models found four such continuations, all with P-Delta; two models failed to import and were not examined. Unchanged and open to question: a linear static or history initial case still gives a geometric stiffness | 6 |
| D71 | In a P-Delta case of a model without hinges, a directly specified P-Delta force is replaced by the computed axial force; the Analysis Reference (v19, p. 417) says the specified forces are in addition. Read in the code, not run | before the video work (found in planning wave 6) | open | 14 |
| D72 | A buckling case with an initial case takes that case's forces as its load; the help topic "Stiffness to Use" makes the initial case the stiffness and the buckling case's own loads the load. Read in the code, not run | before the video work (found in planning wave 6) | open | 14 |
| D73 | The area edge constraint is read from an S2K file and ignored without a message: two meshes that do not share joints along an edge run as two unconnected parts. This is against the rule to refuse what is not analyzed. Found by a reader of the vendor's video "Area Edge Constraints"; seen in the code (`edge_constraint` is stored in `sap-model` and read in `s2k.rs`, and nothing else uses it) | before the video work (found on 2026-11-18) | open | 8 |
| D74 | A linear, modal or multi-step case continued, with the elastic stiffness, from a yielded hinge when the hinged frame was subdivided automatically: the guard asked the meshed model for hinge assignments, and the pieces of a subdivided frame carry none. Found by an outside review on 2026-11-18 (finding P1-1) in the wave 6 work | 6 (review) | done: the guard decides from the hinge state the earlier case recorded (`wave6_end_stiffness.rs`: `a_yielded_hinge_is_refused_however_the_frame_is_subdivided`, `a_hinge_that_unloaded_keeps_its_own_wording_in_a_subdivided_frame`, `a_subdivided_frame_whose_hinge_stays_elastic_may_be_continued`) | 6 |
| D75 | Frame forces of a time-history case, modal or by direct integration, left out the member loads: end forces came from stiffness times displacement alone, while the reactions held the loads (review finding P1-2) | before the video work (review of 2026-11-18) | done: fixed-end forces and span loads at the stored time, with scale factor and function value (`p1_modal_history_recovery.rs`, ten tests; closed forms wL/2, wL²/12, Q/2, QL/8). Forces are stored at the step of largest displacement only; neither history path produces area forces | 6 |
| D76 | Base reactions of a time-history case left out the restrained joints that area meshing generates, so the base shear of one restrained shell fell with the mesh (review finding P1-3) | before the video work (review of 2026-11-18) | done: generated restrained joints are included, each once (`p1_modal_history_recovery.rs`: the two `..._base_reactions_of_a_meshed_restrained_shell_are_independent_of_the_mesh` tests; reference mass times acceleration, and its moment) | 6 |
| D77 | A load of a time-history case was taken for a ground acceleration when its pattern was named X, U1 and the like; a dozen places decided by the name (review finding P1-4) | before the video work (review of 2026-11-18) | done for history loads: the kind is stored and carried through S2K, the native format, the solver, the API and the form; an input that does not state the kind and could be either is refused, naming both readings (`history_load_kind.rs`, `history_load_kind_native.rs`, opt-in `history_load_kind_vendor.rs`). Open: static and Ritz case loads have no stored kind; a pattern named like a direction is refused there | 6 |
| D78 | A combination was published from the contributors that had results when one had none (a case switched off): 1.2 D under the name of 1.2 D + 1.6 L (review finding P1-5) | before the video work (review of 2026-11-18) | done: such a combination has no result under any name and is listed with its reason; nested combinations, envelopes, steel and concrete design, the API and the lists of the windows refuse it by name; other results stay usable (`sap-app/tests/p1_combo_unavailable.rs`, 18 tests). Since 2026-11-19 a time-history contributor makes the combination unavailable (D80), and unlocking or editing through the API clears the reasons with the results (D79). Open: other places that clear results directly leave the list of reasons behind (a stale reason can only refuse) | 6 |
| D79 | A load case and a load combination could have one name, through S2K import and through the database tables. Results are kept by name, so the result of the case stood for the combination: in the reviewer's control a parent combination TOP was published at -20 kN while the run listed its contributor D as unavailable. Beside it, in the API: with an unavailable combination and a valid one selected, the result tables returned success with the rows of the valid one only; a refused selection call removed the name from the selection; unlocking and locking again brought an old reason back | review of the repaired tree, 2026-11-18 | done: one check (`sap_model::results::case_combo_collision_message`) refuses the shared name at S2K import, native import, when database-table edits are applied (the whole edit is refused, the model is unchanged), before any case is analyzed, and in steel and concrete design; the message names the name and both objects. API: while the selection holds an unavailable name, getters with a return code and the four result tables (joint displacements, joint reactions, base reactions, frame forces) return the error code and no rows, getters that return rows only return none, and `selected_unavailable()` gives the reasons; a refused selection call leaves the selection as it was; unlocking and editing clear the reasons with the results (`sap-api/tests/mb_combo_selection.rs`, 7 tests; `sap-app/tests/mb_combo_names_history.rs`, 9 tests with D80). Not tested: the native reader's refusal (no native file with a shared name exists to read). The windows were not opened | 6 |
| D80 | A combination with a time-history contributor took the one stored state of the history (the step of largest displacement) as both its maximum and its minimum: for a history whose 101 samples run from -0.1409388942 to +0.1415610469 m, the envelope gave +0.1415610469 m twice, and a sum with a static case one value where the bounds are -0.0409388942 and +0.2415610469 m | review of the repaired tree, 2026-11-18 (the defect is older than the video work) | done by withholding: such a combination has no result and is listed with a reason that names the history case; nested combinations, design, the API and the window lists refuse it through the mechanism of D78. The extrema themselves are not computed: G69. Combinations of response-spectrum cases are unchanged. One existing test combined a static case with a history and was changed to combine with a P-Delta case (`frame_auto_mesh_regressions.rs`) | 6 |
| D81 | Native export refused a history load without a stated kind that can only be a ground acceleration (U1 with no pattern U1), because the reread file states the kind and the comparison of the two models saw a change; it refused an acceleration stated under the alias X ("missing referenced name"); and the check for imported data that is not modeled took an acceleration U1 for a use of an unsupported pattern named U1 | review of the repaired tree, 2026-11-18; the first two came in with the repair of D77 | done: export and the dependency check go by what the entry means (`resolve_history_load`), not by its stored tag or the spelling of its direction. Legacy U1 and acceleration X export; reopened, the entry is an acceleration named U1 and the response of a restrained single-mass model equals that of the S2K model and the closed form u(t) = -0.02 (1 - cos 10 t) to 1e-9 m. An entry that could be either (X beside a pattern X) is refused at export with the words of the analysis. A history that really uses the unsupported pattern stays refused (`sap-format/tests/history_load_kind_export.rs`, 8 tests; `sap-solver/tests/history_load_kind_export.rs`, 5 tests). Tightened: an entry U4 to U6 without a kind and without a pattern is refused at export. Three other readers of the kind (`coordinates.rs`, `auto_seismic.rs`, the check of active loads) still go by the stored tag, which treats an unresolved direction name as a pattern | 6 |
| D82 | The check of a case definition (used by the API and by the case form) refused a time-history case whose pattern has frame distributed or point loads, which the solver runs and recovers since D75; and the solver accepted solid surface and temperature loads in a history pattern and dropped them | review of the repaired tree, 2026-11-18 | done: one rule, `sap_solver::history_pattern_refusal`, used by the solver and by the definition check; it names the pattern, each refused load type and one object. Accepted: joint forces, frame distributed and point loads. Refused: self-weight, frame temperature and gravity loads, area loads, solid loads (new in the solver), support motion. Through the API the fixed 6 m beam under 2 kN/m gives end shears +6 and -6 kN, end moments 6 kN m and matching reactions, equal to the static case to 1e-9, for modal and direct integration; a 12 kN point load gives 6 kN and 9 kN m (`sap-api/tests/history_case_validation.rs`, 8 tests). Two notes of the case form were corrected; the form was not opened | 6 |
| D83 | The result of a P-Delta case with hinges depended on the units of the model: the iteration on the axial forces compared the change of an axial force with a scale that held the capacities of the hinges, and a moment capacity is a thousand times larger in kN mm. One cantilever pushed to 4.95 mm gave load factor 8.345645 in metres and 8.018115 in millimetres; the second, applied under load control, gave 4.7432 mm | review of the repaired tree, 2026-11-18; in the tree since P-Delta with hinges (sixth wave) | done: the change of the axial forces is compared with the largest axial force, and the force it leaves unbalanced on an element (change times the tilt of the element) with the largest force of the state; a change below the rounding of an element counts as none; the default tolerance is unchanged. Both unit systems give 8.346151615488 (closed form 8.346152; with tolerance 1e-12 both give 8.3461524005); replayed under load control the factor gives the deflection to 1.0e-7 of itself; the capacity of an unyielded hinge (20, 1e4, 1e14) does not change the factor (`sap-solver/tests/pdelta_axial_convergence.rs`, 6 tests with D84) | 6 |
| D84 | A displacement-controlled P-Delta case was refused ("load pattern does not move the monitored displacement") when the load moves the monitored joint only by changing the geometric stiffness: a column with a lateral preload and then pure compression, whose tip does move (0.450000 to 0.458249 mm at 100 kN). The rate of the monitored displacement was taken with the axial forces held | review of the repaired tree, 2026-11-18; in the tree since P-Delta with hinges (sixth wave) | done: before the first step and before each later one the rates of the hinge forces and of the monitored displacement are taken from a trial state in which the axial forces follow the load, and the step is corrected until the target row holds. The reviewer's increment of 1e-8 m is reached (factor 0.00123454032, closed form to 1e-6) and the factor replayed under load control gives the same deflection (residual 9e-19 m); a straight column under the same compression is still refused. One existing test pinned a load factor of the old path and now asserts what it meant (`wave6_pdelta_hinges.rs`, the one-iteration case). Not covered by a test: a case where the coupled rate changes the order of hinge events | 6 |

### Features and options not done

| Id | Item | Video row | Status | Wave |
| --- | --- | --- | --- | --- |
| G01 | Shell stresses at the Gauss points, extrapolated to the joints, averaged at joints; smooth shell contours | 4.14 | open (Analysis Reference pp. 212-213) | 8 |
| G02 | Shell results per mode | 4.13 | open | 8 |
| G03 | Velocity, reaction, base-reaction and energy plot functions | 3.7 | open | 9 |
| G04 | Absolute acceleration of direct-integration and nonlinear histories | 3.7 | open | 9 |
| G05 | Plot function sets saved in the model file | 3.7 | open | 9 |
| G06 | A unit switch on a locked model | 4.14 | open | 11 |
| G07 | Read-out of the loads assigned to an object | 1.17 | done: the Object Information window on a right-click in the Undeformed display (`wave5_windows_tests.rs`: `the_load_lines_show_the_stored_values_in_display_units`, `a_right_click_in_the_undeformed_display_opens_the_object_window_with_the_loads`) | 5 |
| G08 | Video export of an animation (uncompressed AVI, the vendor's default, written without an encoder library) | 3.8 | done: multi-step and cyclic videos (`video_export/tests.rs`: `avi_tiny_video_equals_the_hand_written_layout`, `ffmpeg_reads_back_count_size_rate_and_pixels`, `record_paints_each_pose_and_writes_a_readable_avi`; `wave5_windows_tests.rs`: `a_video_of_a_history_through_the_menu_path_has_the_layout_length_and_moving_frames`). The default magnification is automatic (D45). Not offered: a compressor, a cyclic video of a plain deformed shape, files above 1 GiB | 5 |
| G09 | Area section "None" for openings | 1.12 | blocked: evidence (no semantics in the help or manuals) | — |
| G10 | Areas with more than four joints | 1.12 | blocked: evidence (no element) | — |
| G11 | Reinforcement "to be checked" with a bar layout | 2.2 | open (ACI 318-14 Example 002) | 10 |
| G12 | Coupled P-M2-M3 hinges and their subsets; fiber, automatic and force-controlled hinges | 2.4 | blocked: evidence. Re-checked in v19 and v24 when the sixth wave was planned: flow normal to the interaction surface is stated (v19 Analysis Reference p. 155), but not the surface between entered points, how it grows with plastic work, or the energy-equivalent adjustment of the curve; the default concrete surface needs a bar layout and a point rule the manuals do not give. Automatic hinges need the FEMA 356 or ASCE 41 tables, which neither version prints. The parametric P-M2-M3 hinge of the v24 technical note is a different hinge | — |
| G13 | Hinge reversal after yield (hysteresis models, pp. 85-99) | 2.11 | open. The minimum with a foundation in the manual is the isotropic and kinematic models for single-component hinges (v19 Analysis Reference pp. 88, 99, 157-158); a reference can be a cantilever cycled under displacement control, and Example 6-009 mapped onto a frame with a rigid-plastic hinge | 14 |
| G13b | Hinge unloading by local redistribution or secant restart (methods 2 and 3) | 2.11 | blocked: evidence (no text anywhere) | — |
| G14 | Hinges together with P-Delta, links, shells, cables, end releases or end offsets; hinges in nonlinear time history | 2.9 | done for P-Delta with hinges at element ends, one geometric setting per chain and no directly specified P-Delta force (`tutorial-video-wave6.md`; `wave6_pdelta_hinges.rs`, 20 tests, among them a push that changes the axial force and a curve that hardens and drops). The rest is G14b | 6 |
| G14b | Hinges together with links, shells, solids, cables, end releases, end offsets, insertion points or non-prismatic sections; a hinge inside an element under P-Delta; chains with mixed geometric settings; large displacements; hinges in nonlinear time history. Not tested: P, V2, V3 and T hinges in a P-Delta case | 2.9 | open | 14 |
| G15 | Capacity-spectrum forms of the pushover curve (ATC-40, FEMA 440) | 2.13 | blocked: evidence (the formulas are referenced, not given) | — |
| G16 | Partial results of a nonlinear case that stops early | 2.11 | done for cases on the hinge path: status "not finished" with the states reached, "could not start" for the cases that need it, design and combinations refuse it (`tutorial-video-wave6.md`; `frame_hinges.rs`, `sap-app/tests/s4_not_finished.rs`). Keeping the states is a project choice; the status words are the vendor's. The rest is G16b | 6 |
| G16b | Partial results of the nonlinear paths without hinges (Wen links, cables, P-Delta without hinges): a case that does not converge is an error with no results | 2.11 | open | 14 |
| G17 | Typed API calls for hinges | 2.4, 2.5 | done for what the vendor API has: the v19 API offers only `GetHingeAssigns` and `Get/SetHingeUnloading`, and both exist | — |
| G18 | `cSapModel` facades for the commands of the waves | all | open | 12 |
| G19 | Native file persistence and export of the data the waves added | all | open. Since wave 5 the modal case of a linear history is written; the seismic eccentricity overwrites are refused like the other data | 12 |
| G20 | AISC 360-16 | 1.25 | blocked: evidence (absent from the vendor material) | — |
| G21 | ASCE 7-16 | 1.15 | blocked: evidence (absent from the vendor material) | — |
| G22 | Site-class and occupancy lookups (Fa, Fv, Ie from tables) | 1.16 | blocked: evidence (Tables 11.4-1, 11.4-2 and 1.5-2 are named, not printed) | — |
| G23 | Program-calculated period for the seismic load | 1.16 | blocked: evidence (the Cu table 12.8-1 is not printed) | — |
| G24 | Eccentricity of the seismic load on diaphragms | 1.16 | done: ratio × width × force or an overwrite distance, on diaphragms about Z; the sign of a positive eccentricity is a project choice (`wave5_eccentricity.rs`: `x_load_moment_is_ratio_times_width_perpendicular_times_force`, `y_load_uses_the_x_width`, `an_overwrite_distance_replaces_ratio_times_width`) | 5 |
| G25 | Automatic wind loads (the video neglects wind) | — | partial evidence: ASCE 7-02 with a user exposure is fully specified (open); the ASCE 7-10 tables are missing (blocked) | 13 |
| G26 | Steel design preferences: seismic design category, framing type, second-order method | 1.25 | open (formulas in the steel design manual, §1.3 and 4.4-4.9) | 10 |
| G27 | Drawing aids: snaps to midpoints, intersections and perpendiculars; length and angle constraints | 1.8 | open | 11 |
| G28 | Cylindrical grids, the 3D template with custom spacing, back-face views | 1.2, 3.1 | open | 11 |
| G29 | A preview of a brace or secondary-beam window | 1.9, 1.11 | blocked: evidence (the help describes no preview) | — |
| G30 | The operating system's file dialog (through an external dialog program; no new dependency) | 1.5 | open | 11 |
| G31 | Breaking frames at area or solid edges | 4.3 | open | 11 |
| G32 | Lightweight concrete; aluminum and cold-formed materials; material values without a field | 4.8 | open | 10 |
| G33 | Temperature loads on three-joint areas, with joint patterns, and on plane and axisymmetric-solid sections | — | open | 8 |
| G34 | Sparse storage with a bandwidth-reducing order for models beyond a few thousand joints | 4.11 | open | 7 |
| G35 | Export to other programs | 1.28 | partial evidence: DXF is mapped in the help (open); IFC and Revit are blocked | 12 |
| G36 | A "Grid Only" template entry | 1.1 | done (`wave5_windows_tests.rs`: `grid_only_gives_a_blank_model_and_the_quick_grid_lines_form`) | 5 |
| G36b | Default materials by region | 1.1 | blocked: evidence (not in the v19 help) | — |
| G37 | Incompatible bending modes of plane sections (Examples 3-001 to 3-004) | — | open | 8 |
| G38 | Axisymmetric solid element (Examples 4-001 to 4-004) | — | open | 8 |
| G39 | Modal damping by mode, proportional and interpolated, for modal histories | 3.5 | open | 9 |
| G40 | Periodic modal time history (Example 1-015) | — | open; refused by name on native import since wave 5 (D27) | 9 |
| G41 | Nonlinear modal time history (FNA) (Examples 6-005 to 6-011), with the link degrees of freedom as Ritz starting vectors | — | open; refused by name on native import since wave 5 (D26, D36) | 9 |
| G42 | Area pressure by joint pattern and edge pressures (Example 2-011) | — | open | 8 |
| G43 | Display of plane sections (S11, S22, S33, S12, SMAX, SMIN, SVM) | — | open; S33 is stored since wave 5 (D01) | 8 |
| G44 | "Add Copy of Property" for sections, materials, hinge properties and load cases | V2.3 | open (the button is on the v19 forms) | 11 |
| G45 | "Use Yield Moment" and "Use Yield Rotation": the program computes the yield value that scales a hinge curve | V2.7 | blocked: evidence. Searched in v19 and v24 when the sixth wave was planned: the help says the program can compute yield values and that for automatic hinges they come from the frame section; no formula for a steel or a concrete section is printed in any file | — |
| G46 | The v26 hinge assignment form (distribution types, expected hinge length) | V2.10 | blocked: evidence (not in the v19 help; the v19 form assigns by relative distance) | — |
| G47 | Generated hinge properties: one per assigned hinge, named by frame label and hinge number; "Show Hinge Details" and "Show Generated Props" in the property list; hinge labels in the view | V2.11 | done (`tutorial-video-wave6.md`; `sap-model/src/hinge.rs`: `generated_properties_are_named_label_h_n_in_hinge_order`; `gui/hinges.rs`: the list, the read-only rows, the labels). The label text, the renumbering after a removal and the refusal of a user property named like a generated one are project choices | 6 |
| G48 | A linear, modal or buckling case on the stiffness at the end of a nonlinear static case of a model with hinges | V2.13 | done where no hinge has yielded at the end of that case, for Modal and linear static cases, through files, the API and the case form (`tutorial-video-wave6.md`; `wave6_end_stiffness.rs`, `wave6_end_stiffness_pdelta.rs` with the vendor's Example 1-017 and hinges added). The rest is G48b | 6 |
| G48b | The stiffness at the end of a nonlinear case in which a hinge has yielded (on its backbone, or unloaded: for the second a reading of the manual gives rigid, with no statement for hinges); buckling after a hinged case | V2.13 | blocked: evidence for a hinge on its backbone; open for the rest | 14 |
| G49 | Copy and export of a displayed table, the pushover capacity curve among them (the vendor exports to Excel and Access) | V2.20 | open (v19 help topic "Database Table Form"); the file format is to be chosen in planning, without a new dependency | 11 |
| G50 | Solution control of a nonlinear static case beyond the step limits: event-to-event stepping as an option, the event lumping tolerance, the events per step, the iteration limits and tolerance | V2.15 | documented as differences, nothing implemented: events are exact; the vendor's event lumping, its two iteration limits and its line search have no counterpart, and step counts differ. In v19 the events per step apply to time histories only. v24 adds a "solution scheme"; the exact event path corresponds to its "Event-to-Event Only" with no unbalance (`tutorial-video-wave6.md`) | 6 (documented) |
| G51 | New Model: "Initialize Model from an Existing File" (units and all definitions of another model, without its grids, objects and assignments) | Spotlight | open (v19 help topic "New Model"). The saved-settings file of later versions (SDBI) is blocked: evidence, it is in neither help | 11 |
| G52 | Options: program default display units, decimal places and tolerances for new models | Spotlight | open (v19 help topic "Set Program Default Display Units"; not read in full) | 11 |
| G53 | Edit > Extrude beyond radial lines to areas: linear extrusion; points to frames or cables; areas to solids; extrusion along a defined path; "Convert Lines to Areas", "Convert Lines to Solids", "Convert Areas to Solids"; "Add Areas between Lines", "Add Solids Between Areas"; the Delete Source option | Spotlight | open (v19 help topic "Extrude") | 11 |
| G54 | Assign > Copy Assigns and Paste Assigns for joints, frames, cables, tendons, areas, solids and links, with the paste options per kind of assignment and the choice of load patterns; pasting replaces | Spotlight | open (v19 help topic "Copy Assigns") | 11 |
| G55 | What the v23 and v25 enhancement videos show: cloud licensing, Eurocode 3 cold-formed design, ACI 318-19, database tables through the API, TCVN and KDS seismic loads, NBCC wind, ASCE 7-22, AS 4100-2020, a model explorer pane, the hinge assignment presets (G46) | Spotlight | outside the v19 baseline; blocked: evidence, except where the v24 material has a manual (ACI 318-19), to be looked at when design is planned. Licensing is not a goal | — |
| G56 | The hinge overwrite "Do Not Allow Hinges To Drop Loads" | P.5 | done for a curve that never drops, where the option changes nothing; refused for a curve that drops, because neither help says what the curve is after the drop (`wave6_hinge_end_kink.rs`: the two `g56_...` tests) | 6 |
| G57 | Display > Show Hinge Results (one hinge's path against its backbone per step, with state, status and extreme plastic deformation); the display commands "Step-by-Step Hinge Formation" and "Step-by-Step Pushover Forces" were not compared | P.13, P.15 | done for the window (`gui/hinge_results.rs`, nine tests on Example 1-026). Not in it: the File menu, Display Tables and the Select Hinge menu of the vendor form. The state and status shown are the most extreme reached, not a read-out at the pointer. "Step-by-Step Hinge Formation" is the deformed shape with hinge dots; the forces step by step were read in the code, not run | 6 |

## Plan

Each wave stays in one area of the code. A wave that changes the global
system comes before the ones that add output, so that reference tests move
once. The row counts are the plan's expectation until a wave is committed;
the timeline above holds the measured ones. D09b and the remainder of D17
have no wave yet.

| Wave | Theme | Ledger items | Video rows done / partial / open afterwards |
| --- | --- | --- | --- |
| 4 (done) | result display | | 53 / 8 / 2 |
| 5 (`1a7d2e8`) | repairs of the defects with evidence in hand, of what two reviews found and of what the hand check found; video export, load read-out, eccentricity, Grid Only | done: D01, D02, D04, D05, D07, D10 to D12, D15, D16, D18 to D24, D26 to D28, D31 to D42, D45 to D49, D51 to D56; G07, G08, G24, G36. Partial: D03 (rest is D03b), D06, D09 (rest is D09b), D17. Found and left open: D43, D44, D50, D57, D58, D59 | 55 / 7 / 1 |
| 6 (done, not committed) | nonlinear static, for the second video: P-Delta with hinges at member ends, the stiffness after a nonlinear case, partial results of a case that stops, generated hinge properties, the hinge results window; repair of two defects found in planning and of five found by an outside review | done: D68, D69, D70, D74 to D78, G47, G56, G57. Done for a part: G14 (rest G14b), G16 (rest G16b), G48 (rest G48b). Documented: G50. Re-checked and still blocked: G12, G45 | 55 / 7 / 1 |
| 7 | global system: exact constraints by internal master joints, sparse storage; then the detection of mechanisms and the Ritz projection on stiff models | D03b, D08, D50, G34 | 55 / 7 / 1 |
| 8 | area and solid elements and their output: joint stresses and averaging, shell results per mode and of response-spectrum cases, incompatible modes, axisymmetric solid, temperature and pressure variants, plane display, the orthotropic solid; the open points of frames with end offsets | D43, D44, D57, D60, D61, D62, D65, D73, G01, G02, G33, G37, G38, G42, G43, G60 | 55 / 7 / 1 |
| 9 | time histories: envelopes, velocity and reaction plots, plot sets in the model, modal damping types, periodic and nonlinear modal histories | D25, D29, G03, G04, G05, G39, G40, G41, G69 (and with it the remainder of D06) | 55 / 7 / 1 |
| 10 | design: bar layouts to be checked, lightweight concrete and other materials, steel second-order methods and framing types | D63, G11, G26, G32 | 57 / 5 / 1 |
| 11 | windows: snaps, cylindrical grids, back-face views, file dialog, breaking at area edges, unit switch on a locked model, digits of the result tables, the shell form after Unlock | D58, D59, D67, G06, G27, G28, G30, G31, G44, G49, G51, G52, G53, G54 | 57 / 5 / 1 |
| 12 | formats and API: native export of the waves' data, `cSapModel` facades, DXF export | D64, D66, G18, G19, G35 | 57 / 6 / 0 |
| 13 | automatic wind load (ASCE 7-02) | G25 | 57 / 6 / 0 |
| 14 | nonlinear static, second part: hinge reversal (isotropic and kinematic), hinges with other nonlinearity and in time history, partial results of the paths without hinges, the two readings of how P-Delta forces and buckling cases are treated | D71, D72, G13, G14b, G16b, G48b | 55 / 7 / 1 |
| not assigned | what the other 39 vendor videos show and StructuraX lacks, by area; each needs its own planning against the vendor material | G58, G59, G61 to G68, D09b | |

The row counts in this table are those of the first video; the sixth wave
moved none of them. Of the second video's 21 rows it moved V2.11, V2.13 and
V2.14 to done and removed the caveat of V2.19 and V2.20; V2.7 and V2.9 are
blocked for evidence. The numbers of the waves not yet carried out were
shifted by one when the nonlinear static wave was done sixth instead of
ninth; the last column of the ledger uses the numbers of this table.

**Where the goal ends with the material at hand:** 57 rows done and 6 partial.
The six are 1.12 (G09, G10), 1.15 (G21), 1.16 (G22, G23), 1.25 (G20), 2.4
(G12) and 1.28 (IFC and Revit of G35): every missing part is blocked because
the vendor material has no formula, table or example for it. They move only
if such material is added to the corpus (for example the AISC 360-16 and ASCE
7-16 design manuals of a later SAP2000 version).

Such material arrived on 2026-11-18 (see "The v24 material"): it has the
AISC 360-16 steel design manual and ASCE 7-16 sections in the Lateral Loads
Manual. Rows 1.25 (G20) and 1.15 (G21) may therefore move; that is decided
when their waves are planned, not here.

## How this file is kept

- A wave closes an item only with a test that has an independent reference;
  the row then names the wave and the item moves to "done" with that test.
- A new finding gets the next free identifier in the wave that finds it.
  Identifiers are never reused.
- The timeline gets one row per wave, with the counts of the integrated
  checks at its commit.
