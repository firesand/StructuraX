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

Progress is counted against the 63 rows and against the ledger. **No
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

## Timeline

| Wave | Date | Commit | Theme | Portable tests | With vendor corpus | Reference checks |
| --- | --- | --- | --- | --- | --- | --- |
| before | 2026-11-13 | `2238221` | state before the video work | — | — | — |
| 1 | 2026-11-15 | `c817b9b` | seismic load, mass source, Ritz, modal time history, Auto Select design, geometry commands, solver capacity | 656 | 674 | 86/86 |
| 2 | 2026-11-16 | `6b33019` | frame hinges and static pushover | 803 | 825 | 90/90 |
| 3 | 2026-11-16 | `7ee7d44` | drawing and editing on plane views and the grid; material library; shell output repairs | 965 | 990 | 92/92 |
| 4 | 2026-11-17 | `0dfc0ba` | result display: shell stresses, displacement contours, plot functions, modal tables; participation and 3D view repairs | 1102 | 1127 | 94/94 |
| 5 | 2026-11-17 | not committed | repairs: rigid zones, released ends, loads, non-prismatic sections, geometric stiffness and output stations of frames with end offsets, constraints at rotated joints, the unstable joint named, orthotropic and plane-strain areas, material angle, combinations as maximum and minimum, design by states and permutations, modal histories from native files, the Ritz projection, window defects found by review, by looking and by a hand check on a private display; video export, load read-out, seismic eccentricity, Grid Only, unequal flanges | 1348 | 1383 | 106/106 |

## The video's 63 feature rows

Status after the fifth wave (not committed). "Ledger" names the item that
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

### Defects and differences

| Id | Item | Found in | Status | Wave |
| --- | --- | --- | --- | --- |
| D01 | Plane-strain areas: forces are recovered with the plane-stress relation, no out-of-plane stress S33 is stored, and there is no anti-plane shear stiffness (Analysis Reference pp. 217-218) | 4 | done: plane-strain recovery, S33 and anti-plane shear stiffness (`wave5_area_materials.rs`: `plane_strain_patch_recovers_s11_s22_s33_s12`, `plane_strain_has_anti_plane_shear_stiffness_and_recovery`). The stored von Mises is D44; the display of plane sections is G43 | 5 |
| D02 | The section's material angle rotates an area's output axes; the vendor rotates only the material. No corpus area section has a nonzero angle (285 records scanned), so the references are closed forms | 4 | done: the angle turns only the material (`wave5_area_materials.rs`: `orthotropic_material_angle_rotates_the_material`, `isotropic_material_is_invariant_under_the_material_angle`) | 5 |
| D03 | The singular-matrix check tests each pivot against that degree of freedom's own diagonal, which lets a rigid rotation of a shell model through at some thicknesses; its message names a pivot index instead of a joint and direction | 4 | partial: the message names the joint and the direction (`wave5_stability.rs`: `r1_a_singular_torsion_pair_is_named_at_its_joint_and_dof`), or the frame and its end joints for an internal joint of an automatic subdivision (`wave5_unstable_frame_name.rs`: `an_unstable_subdivided_frame_is_named_by_its_frame_and_end_joints`). Which models are refused did not change; that part is D03b | 5 |
| D03b | Mechanisms that pass the pivot test are accepted (a rigid rotation of a shell model at some thicknesses). Measured in wave 5: the smallest eigenvalue of the stiffness scaled by its structural diagonal separates sound models (smallest 1.66e-12) from mechanisms by about four orders without penalty constraints (at most 1.7e-16), and not at all with a diaphragm penalty (up to 4.5e-7), so no threshold was enabled. `linalg::scaled_stiffness_minimum` is written, tested and called nowhere | 4 (as part of D03), 5 (measured) | open: needs the exact constraints of D08 | 6 |
| D04 | The stored von Mises field of multi-valued combinations is combined field by field (a symptom of D05) | 3 | done: derived fields are 0 in both rows of a multi-valued result and "n/a" in the tables (`sap-solver/src/combos.rs`: `derived_fields_are_zero_for_multi_valued_results`) | 5 |
| D05 | Combinations store one value. The manual defines every combination as a maximum and a minimum (pp. 351-353): today Range is added linearly, Envelope keeps the value of larger magnitude, and no minimum is stored for any type | 4 | done: every multi-valued combination stores a maximum and a minimum (`sap-solver/src/combos.rs`: `manual_p353_comb1_and_comb2`, `manual_p353_axial_forces`, `range_combination`; `wave5_combinations.rs`: `solver_spectrum_case_gives_the_manual_pairs_of_comb1_and_comb2`) | 5 |
| D06 | Examples 6-006 and 6-007 from the vendor files: periods 0.6 to 3.5 % from the published ones, unexplained | 1 | partial: the rigid-zone repair (D21) and the Ritz repair (D37) explain most of it. With the Ritz starting-load entry (D36) removed the periods are 0.437263, 0.135023, 0.073709 s after all solver repairs of the wave for the published 0.438, 0.135, 0.074 (−0.17 %, +0.02 %, −0.39 %): T2 and T3 inside the printed rounding, T1 0.0002 s below it, not explained (opt-in `wave5_rigid_zones.rs`: `vendor_6_006_and_6_007_periods_follow_the_published_values_with_full_length_axial_stiffness`, bound 1 %). As imported the Modal case is now refused (D36). Open candidate for the remainder: the vendor's link starting vectors, part of G41 | 5; the remainder with G41 (8) |
| D07 | Native import ignores the case record's modal case and motion type, so a vendor modal history with zero damping (6-001, 1-015) is integrated directly | 1 | done: the modal case is imported and written (`sap-format/src/sdb.rs`: `native_modal_history_import_branches`; `sap-format/tests/wave5_native_history.rs`: `a_linear_history_keeps_its_modal_case_through_the_native_writer`; opt-in `wave5_vendor_history.rs`: `example_6_001_modal_histories_against_the_published_values_and_the_closed_form`) | 5 |
| D08 | A diaphragm is a penalty constraint; with a nearly rigid axial area a collapse load through it loses several digits. The manual describes internal master joints (pp. 66-67) | 2 | open | 6 |
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
| D25 | History cases report the state at the step of largest displacement instead of envelopes (p. 351) | 5 (planning) | open | 8 |
| D26 | Native nonlinear modal (FNA) cases run as direct nonlinear integration (6-005a) | 5 (planning) | done: refused by name (`sap-format/src/sdb.rs`: `native_modal_history_import_branches`; opt-in `wave5_vendor_history.rs`: `an_fna_history_is_refused_by_name`). The analysis itself is G41 | 5 |
| D27 | A native periodic modal history runs as a transient (1-015) | 5 (planning) | done: refused by name (`sap-format/src/sdb.rs`: `native_modal_history_import_branches`; opt-in `wave5_vendor_history.rs`: `a_periodic_history_is_refused_by_name`). The analysis itself is G40 | 5 |
| D28 | A released frame end keeps its rigid-zone factor instead of acting as fully rigid (pp. 130-131) | 5 (planning) | done; a partial-fixity spring keeps the factor, a project choice (`wave5_rigid_zones.rs`: `r8_a_moment_release_makes_the_offset_rigid_in_its_plane`, `r8b_a_shear_release_makes_the_offset_rigid_in_its_plane`) | 5 |
| D29 | Modal-history reactions leave out the modal damping force | 1 (documented) | open | 8 |
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
| D43 | The solid element builds the orthotropic compliance with `−u12/e1`, `−u13/e1`, `−u23/e2`; the manual's Eqn. 3 as printed, which the area element follows and Example 2-015 supports, has `−u12/e2`, `−u13/e3`, `−u23/e3` | 5 | open | 7 |
| D44 | The stored von Mises of a plane-strain area ignores S33 | 5 | open (with G43) | 7 |
| D45 | The Create Video form's default magnification was a fixed 10, not the automatic scale of the view | 5 | done: automatic until the box is edited (`p16_misc_tests.rs`: `the_video_magnification_is_the_automatic_factor_until_the_user_edits_it`) | 5 |
| D46 | Span loads and self weight on the rigid part of a frame with end offsets were dropped, and the released-end rule of D28 widened that (fixed-fixed beam of 144 with offsets 12, uniform load 1: reactions summed to 120) | 5 (second review) | done: loads act over the full length; the part on a rigid arm goes to its joint with its moment (`wave5_end_offset_loads.rs`: `d46_sum_of_reactions_equals_the_load_for_every_rigid_end_factor_and_release`, `d46_joint_moment_of_a_uniform_load_on_arm_and_span`, `d46_a_point_load_on_the_arm_goes_to_its_joint`; `wave5_p13_unchanged.rs`: `members_without_offsets_are_bitwise_unchanged`) | 5 |
| D47 | Non-prismatic frames with end offsets: the variation was laid over the flexible length instead of the clear length, and thermal and axial fixed-end forces used the stiffness without the end zones (Analysis Reference pp. 120, 130). Found with it: rigid arms had no geometric stiffness | 5 (second review) | done: variation over the clear length, end zones with the end sections (`wave5_nonprismatic_offsets.rs`: `d47_axial_and_torsional_flexibility_follow_the_clear_length_for_every_rigid_end_factor`, `d47_free_thermal_expansion_is_alpha_dt_l_for_a_non_prismatic_member_with_offsets`); `P × arm` geometric stiffness of each rigid arm, which changes P-Delta and buckling of members with rigid arms (`wave5_rigid_arm_geometry.rs`: `rigid_arms_add_p_times_arm_to_the_joint_rotation_stiffness`). Not done, no identifier yet: the 1 % clear-length rule (p. 129), the mass of non-prismatic members and end zones (p. 134), the geometric stiffness of the flexible part on the flexible length | 5 |
| D48 | Output stations of a frame with end offsets ran joint to joint, so a released end reported the joint moment instead of zero at the support face (Analysis Reference p. 130) | 5 (second review) | done: the stations run from support face to support face (`wave5_end_offset_stations.rs`: `d48_stations_run_from_support_face_to_support_face`, `d48_a_released_end_reports_zero_moment_at_its_support_face`); Examples 1-010b to d moved in station position and M3 (first station 0 → 6 in, −1440 → −1380), by the package's print, not by a test. Two consequences, found by reading and repaired in the same wave: the drawing (D56), and the design code, where concrete beam design found no end station (the minimums of special and intermediate frames stopped applying, unconservative) and the steel Cb sampled outside the stations; design now takes the first and last station as the ends and measures Cb between them (`sap-design/tests/wave5_p17_offset_stations.rs`: `special_moment_frame_floors_act_at_the_support_faces`, `cb_quarter_points_lie_between_the_support_faces`, `members_without_offsets_are_bitwise_unchanged`) | 5 |
| D49 | Design read the Max and Min rows of a multi-valued combination as one station list: no permutation of maxima and minima of the interacting forces (the largest moment was never paired with the smallest axial force), and Cb across the rows (D42) | 5 (second review) | done: each state is a design entry; permutations of P, M2, M3 (with V2, V3 for Eurocode 3; with T, V2, V3 for closed sections under torsion, untested) (`sap-design/tests/wave5_design_states.rs`: `additive_with_spectrum_is_designed_at_every_permutation_of_p_and_m3`, `envelope_range_srss_and_absolute_are_permuted`, `column_pmm_is_designed_at_every_permutation`, `single_valued_combinations_and_plain_cases_are_bitwise_unchanged`) | 5 |
| D50 | The flexibility form of the Ritz projection (D37) gives a frequency that is not `φᵀKφ`, so the static participation of a starting load is not exactly 100 % on stiff models (measured: at most 1.6e-8 at an axial modifier of 1e6, 7.2e-6 at 1e9; 1.0029 and 1.0099 only with the branch forced). The single form the review proposed was built, measured and rejected (the bound tests of D37 fail with it) | 5 (second review) | open: the two forms and the switch between them stay | 6 |
| D51 | Two files of verification rows returned no rows when their analysis failed, so the rows could not fail by an error | 5 (second review) | done by reading, without a test: both now stop with an error, as the older verification files do (`sap-solver/src/bench_w5_p1.rs`, `bench_w5_p3.rs`) | 5 |
| D52 | The explorer printed quantities with a fixed number of decimals: a section read "I33=0.0" in kN, m (seen in the hand check, which took it for a section without stiffness) | 5 (hand check) | done: at least four decimals and five significant digits in the explorer, labels and the small tables (`wave5_hand_check_tests.rs`: `the_explorer_prints_section_properties_with_significant_digits`, `quantity_text_keeps_five_significant_digits_in_any_unit`). The result tables are D58 | 5 |
| D53 | Opening a file kept the display state of the old model: a shell-stress display with "no results", a 2D pane without its plane (seen in the hand check). The reported "Locked" status could not be reproduced | 5 (hand check) | done: a new or opened model starts undeformed, with the automatic scale, unlocked when it has no results; the panes keep a plane the new model has (`p16_open_tests.rs`: `opening_an_s2k_resets_results_lock_display_and_keeps_a_plane_of_the_new_grid`, `opening_a_file_that_cannot_be_read_changes_nothing`) | 5 |
| D54 | A status message longer than the line was cut and could be read nowhere (the reason of a refused pushover run) | 5 (hand check) | done: the whole text is the tooltip of the cut line, and Help > Messages lists the session's messages (`p16_messages_tests.rs`: `the_whole_reason_of_a_refused_analysis_is_readable_on_hover_and_in_the_messages_window`, `the_messages_are_newest_first_bounded_and_the_errors_are_marked`) | 5 |
| D55 | Smaller window items of the hand check: float noise in the duration of a function from a file; the seismic window and Model Workflows taller than the screen; the 2D frame template opening its 2D pane as a plan at Z = 0; the pushover curve window without readable ticks and with a cut label; the title of a video frame cut | 5 (hand check) | done (`p16_misc_tests.rs`: `the_duration_of_a_function_read_from_a_file_has_no_float_noise`, `big_windows_open_inside_a_1440_by_900_screen_and_scroll`, `the_pushover_plot_has_readable_ticks_and_every_label_inside_the_plot`, `the_title_of_a_narrow_pane_wraps_inside_the_pane`; `p16_open_tests.rs`: `the_2d_frame_template_opens_its_2d_pane_on_the_frame_and_a_3d_template_does_not`, `a_planar_model_opens_the_2d_pane_on_its_plane`) | 5 |
| D56 | With the stations at the support faces (D48) the deformed line of a frame with end offsets stopped at the faces and left a gap to the displaced joints | 5 (consequence of D48) | done: the rigid arms are drawn (`p16_offset_tests.rs`: `the_deformed_shape_runs_from_the_joint_through_the_face_to_the_tip`, `the_moment_diagram_starts_at_the_face_with_nothing_inside_the_offset`) | 5 |
| D57 | A response-spectrum case stores no area results, so the shell form refuses a combination with one for "no shell results" (seen in the hand check) | 5 (hand check) | open | 7 |
| D58 | Result-table cells have a fixed number of decimals: a displacement of 1e-5 m reads "-0.00001" | 5 (hand check) | open | 11 |
| D59 | After Unlock an open shell form still shows its old overlay | 5 (hand check) | open | 11 |
| D60 | End offsets are not reduced when the clear length would fall below 1 % of the member length (Analysis Reference p. 129); only a clamp at 1e-6 of the length exists | 5 (frame repairs) | open | 7 |
| D61 | The mass of a non-prismatic member, and of a member with end offsets, is taken from the section at mid length, half to each joint, not as p. 134 describes | 5 (frame repairs) | open | 7 |
| D62 | The geometric stiffness of a member with end offsets uses the flexible length for its bending part; no reference was established for it (the P x arm term of the rigid arms was added in wave 5) | 5 (frame repairs) | open | 7 |
| D63 | The permutation of T, V2 and V3 for closed sections under torsion (AISC H3-6) in the design of a maximum/minimum combination has no test | 5 (design repair) | open | 10 |
| D64 | Some API result readers return only the Max row of a maximum/minimum combination (the seven extraction functions with a step type return both) | 5 (second review) | open | 12 |
| D65 | The S33 of a plane-strain area is stored but no table, API call or display reads it | 5 (end-to-end tests) | open | 7 |
| D66 | Example 2-015 model B stores the bending modifiers as m11 = 1.0, m22 = 0.5 where the problem text says m11 = 0.5, m22 = 1.0; the order in which the importer reads the modifiers is not verified against a second source | 5 (area materials) | open (investigation) | 12 |
| D67 | A fitted model uses about 0.62 of its pane's width, and the program starts in the M3 diagram display, where a right-click on a frame opens the inspector instead of Object Information | 5 (hand check) | open | 11 |

### Features and options not done

| Id | Item | Video row | Status | Wave |
| --- | --- | --- | --- | --- |
| G01 | Shell stresses at the Gauss points, extrapolated to the joints, averaged at joints; smooth shell contours | 4.14 | open (Analysis Reference pp. 212-213) | 7 |
| G02 | Shell results per mode | 4.13 | open | 7 |
| G03 | Velocity, reaction, base-reaction and energy plot functions | 3.7 | open | 8 |
| G04 | Absolute acceleration of direct-integration and nonlinear histories | 3.7 | open | 8 |
| G05 | Plot function sets saved in the model file | 3.7 | open | 8 |
| G06 | A unit switch on a locked model | 4.14 | open | 11 |
| G07 | Read-out of the loads assigned to an object | 1.17 | done: the Object Information window on a right-click in the Undeformed display (`wave5_windows_tests.rs`: `the_load_lines_show_the_stored_values_in_display_units`, `a_right_click_in_the_undeformed_display_opens_the_object_window_with_the_loads`) | 5 |
| G08 | Video export of an animation (uncompressed AVI, the vendor's default, written without an encoder library) | 3.8 | done: multi-step and cyclic videos (`video_export/tests.rs`: `avi_tiny_video_equals_the_hand_written_layout`, `ffmpeg_reads_back_count_size_rate_and_pixels`, `record_paints_each_pose_and_writes_a_readable_avi`; `wave5_windows_tests.rs`: `a_video_of_a_history_through_the_menu_path_has_the_layout_length_and_moving_frames`). The default magnification is automatic (D45). Not offered: a compressor, a cyclic video of a plain deformed shape, files above 1 GiB | 5 |
| G09 | Area section "None" for openings | 1.12 | blocked: evidence (no semantics in the help or manuals) | — |
| G10 | Areas with more than four joints | 1.12 | blocked: evidence (no element) | — |
| G11 | Reinforcement "to be checked" with a bar layout | 2.2 | open (ACI 318-14 Example 002) | 10 |
| G12 | Coupled P-M2-M3 hinges and their subsets; fiber, automatic and force-controlled hinges | 2.4 | blocked: evidence (no verification problem or code table) | — |
| G13 | Hinge reversal after yield (hysteresis models, pp. 85-99) | 2.11 | open | 9 |
| G13b | Hinge unloading by local redistribution or secant restart (methods 2 and 3) | 2.11 | blocked: evidence (no text anywhere) | — |
| G14 | Hinges together with P-Delta, links, shells, cables, end releases or end offsets; hinges in nonlinear time history | 2.9 | open | 9 |
| G15 | Capacity-spectrum forms of the pushover curve (ATC-40, FEMA 440) | 2.13 | blocked: evidence (the formulas are referenced, not given) | — |
| G16 | Partial results of a nonlinear case that stops early | 2.11 | open | 9 |
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
| G33 | Temperature loads on three-joint areas, with joint patterns, and on plane and axisymmetric-solid sections | — | open | 7 |
| G34 | Sparse storage with a bandwidth-reducing order for models beyond a few thousand joints | 4.11 | open | 6 |
| G35 | Export to other programs | 1.28 | partial evidence: DXF is mapped in the help (open); IFC and Revit are blocked | 12 |
| G36 | A "Grid Only" template entry | 1.1 | done (`wave5_windows_tests.rs`: `grid_only_gives_a_blank_model_and_the_quick_grid_lines_form`) | 5 |
| G36b | Default materials by region | 1.1 | blocked: evidence (not in the v19 help) | — |
| G37 | Incompatible bending modes of plane sections (Examples 3-001 to 3-004) | — | open | 7 |
| G38 | Axisymmetric solid element (Examples 4-001 to 4-004) | — | open | 7 |
| G39 | Modal damping by mode, proportional and interpolated, for modal histories | 3.5 | open | 8 |
| G40 | Periodic modal time history (Example 1-015) | — | open; refused by name on native import since wave 5 (D27) | 8 |
| G41 | Nonlinear modal time history (FNA) (Examples 6-005 to 6-011), with the link degrees of freedom as Ritz starting vectors | — | open; refused by name on native import since wave 5 (D26, D36) | 8 |
| G42 | Area pressure by joint pattern and edge pressures (Example 2-011) | — | open | 7 |
| G43 | Display of plane sections (S11, S22, S33, S12, SMAX, SMIN, SVM) | — | open; S33 is stored since wave 5 (D01) | 7 |

## Plan

Each wave stays in one area of the code. A wave that changes the global
system comes before the ones that add output, so that reference tests move
once. The row counts are the plan's expectation until a wave is committed;
the timeline above holds the measured ones. D09b and the remainder of D17
have no wave yet.

| Wave | Theme | Ledger items | Video rows done / partial / open afterwards |
| --- | --- | --- | --- |
| 4 (done) | result display | | 53 / 8 / 2 |
| 5 (not committed) | repairs of the defects with evidence in hand, of what two reviews found and of what the hand check found; video export, load read-out, eccentricity, Grid Only | done: D01, D02, D04, D05, D07, D10 to D12, D15, D16, D18 to D24, D26 to D28, D31 to D42, D45 to D49, D51 to D56; G07, G08, G24, G36. Partial: D03 (rest is D03b), D06, D09 (rest is D09b), D17. Found and left open: D43, D44, D50, D57, D58, D59 | 55 / 7 / 1 |
| 6 | global system: exact constraints by internal master joints, sparse storage; then the detection of mechanisms and the Ritz projection on stiff models | D03b, D08, D50, G34 | 55 / 7 / 1 |
| 7 | area and solid elements and their output: joint stresses and averaging, shell results per mode and of response-spectrum cases, incompatible modes, axisymmetric solid, temperature and pressure variants, plane display, the orthotropic solid; the open points of frames with end offsets | D43, D44, D57, D60, D61, D62, D65, G01, G02, G33, G37, G38, G42, G43 | 55 / 7 / 1 |
| 8 | time histories: envelopes, velocity and reaction plots, plot sets in the model, modal damping types, periodic and nonlinear modal histories | D25, D29, G03, G04, G05, G39, G40, G41 (and with it the remainder of D06) | 55 / 7 / 1 |
| 9 | nonlinear static: hinge reversal, hinges with other nonlinearity, partial results | G13, G14, G16 | 55 / 7 / 1 |
| 10 | design: bar layouts to be checked, lightweight concrete and other materials, steel second-order methods and framing types | D63, G11, G26, G32 | 57 / 5 / 1 |
| 11 | windows: snaps, cylindrical grids, back-face views, file dialog, breaking at area edges, unit switch on a locked model, digits of the result tables, the shell form after Unlock | D58, D59, D67, G06, G27, G28, G30, G31 | 57 / 5 / 1 |
| 12 | formats and API: native export of the waves' data, `cSapModel` facades, DXF export | D64, D66, G18, G19, G35 | 57 / 6 / 0 |
| 13 | automatic wind load (ASCE 7-02) | G25 | 57 / 6 / 0 |

**Where the goal ends with the material at hand:** 57 rows done and 6 partial.
The six are 1.12 (G09, G10), 1.15 (G21), 1.16 (G22, G23), 1.25 (G20), 2.4
(G12) and 1.28 (IFC and Revit of G35): every missing part is blocked because
the vendor material has no formula, table or example for it. They move only
if such material is added to the corpus (for example the AISC 360-16 and ASCE
7-16 design manuals of a later SAP2000 version).

## How this file is kept

- A wave closes an item only with a test that has an independent reference;
  the row then names the wave and the item moves to "done" with that test.
- A new finding gets the next free identifier in the wave that finds it.
  Identifiers are never reused.
- The timeline gets one row per wave, with the counts of the integrated
  checks at its commit.
