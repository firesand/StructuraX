# CSI video references for SAP2000 reconstruction

Checked 2026-10-15. User supplied [CSI SAP2000 videos](https://www.csiamerica.com/products/sap2000/videos). The public catalog currently returns 46 entries for SAP2000 (product item 4617). Titles, categories, durations and links are recorded in `video-catalog.json`; the fetched response is retained in `target/reconstruction/csi-video-catalog.json`.

## Evidence boundary

The catalog metadata was inspected; the videos have not been watched or transcribed in this pass. The page requires JavaScript and no connected browser was available, so its public catalog endpoint was read directly. No demonstrated behavior, timestamps, formulas or expected numerical outputs are inferred from titles.

The catalog mixes versions: two entries explicitly describe v23 and v25 enhancements. Those are excluded from the v19 baseline. The remaining entries do not establish a version merely by being listed here; confirm UI/version/code settings against the extracted v19 tutorial, July 2016 analysis manual and shipped v19 artifacts.

## Proposed workflow checks

These are reconstruction acceptance tasks selected by catalog topic, not claims about content already observed.

- [Introductory Tutorial](https://youtu.be/2povDdw2pTQ) (30m17s): Model creation, assignment, solve and result review; pair with the thirteen-step tutorial checklist.

- [Draw Commands](https://youtu.be/aLCMg0P6pmw) (9m29s): Frame/area drawing and selection feedback; check units, snapping and connectivity.

- [TECH TIPS: Auto vs. Manual Meshing- SAP2000](https://youtu.be/MYEcqV96T_8) (3m45s): Meshing workflow; verify shared nodes, edge compatibility and object-versus-analysis-element distinctions.

- [Tension-only Bracing](https://youtu.be/JkhYzTtwsKE) (4m21s): Tension-only constitutive response and inactive-member equilibrium; currently incomplete.

- [Gap Elements](https://youtu.be/2xh4gRX9_A0) (17m46s): Gap opening/closing and load-path changes; effective linear stiffness alone does not establish this behavior.

- [Mass and Modal Analysis](https://youtu.be/3ZbKg-oVC5I) (5m46s): Mass source, mode shapes and participating mass; compare numerical quantities with version-matched fixtures.

- [Modal Time History Analysis](https://youtu.be/VjkqA4_hlMk) (14m4s): Modal history formulation and excitation controls; direct integration tests do not establish modal-history parity.

- [Open Application Programming Interface](https://youtu.be/QRdXGi0g0BE) (13m54s): API workflow context; validate exact v19 signatures against managed metadata/type-library/API help.

- [Interactive Database Editing](https://youtu.be/YIrv_W0xlT0) (7m59s): Table edits, transactional validation and result invalidation; preserve original model on invalid edits.

- [Creating Reports](https://youtu.be/SFNS22hUObk) (11m17s): Report creation and contents; validate displayed units, selected cases and traceability to actual results.

## Supplementary beam playlist

The user also supplied [SAP2000 Full Course – Structural Modeling from Beginner to Pro](https://www.youtube.com/playlist?list=PLF8qIet2NX2F23FtLNF7clwGJKvkCb0by), published by the channel **SAP2000 Tutorial Series 結構工程教學系列**. On 2026-10-15 its public playlist metadata showed three videos, with no video-ID overlap against the 46-entry CSI catalog. This is a supplementary source; CSI affiliation and demonstrated SAP2000 version have not been established. Metadata is saved in `supplementary-playlist.json`. Video playback, transcripts and numerical results have not been reviewed.

- [Simply supported beam](https://www.youtube.com/watch?v=ZZCFGD_1SM8): candidate test for restraint definitions, force equilibrium, end moments and displacement results.
- [Cantilever beam](https://www.youtube.com/watch?v=HlEwet5BFXY): candidate test for fixed-support reactions, tip displacement, local axes and force-diagram signs.
- [Continuous beam](https://www.youtube.com/watch?v=9O48Bm2-ZEw): candidate test for shared joints, interior-support reactions, end releases and moment continuity.

These proposed checks follow the lesson topics; they do not assert the videos use any particular loading, geometry or solver settings. The playlist's course title is not evidence of complete SAP2000 feature coverage. Before creating matching fixtures, capture the actual inputs and compare expected results independently against analytical solutions and version-matched CSI references.

## Before implementing from a video

Record the demonstrated version, exact timestamps, model geometry, units, load/case settings and visible result quantities. Separate visible UI behavior from inferred solver behavior. Cross-check equations/defaults with the v19 manual and fixtures; add a reproducible test with expected values and tolerance. Mark incompatible or newer-version behavior separately. Catalog presence never counts as implementation or validation.
