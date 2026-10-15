# Design reconstruction wave 1

## Implemented behavior

AISC 360-10 LRFD/ASD steel checks now calculate local-3 shear resistance and include its ratio when selecting the governing station and combination. `SteelDesignResult` exposes `phi_vn3` and `shear_minor_ratio` with backward-compatible deserialization defaults. A result's `total_ratio` is the maximum governing interaction/shear ratio. Missing demands still fail before design; failures leave the result map empty.

The new public `compute_aisc_minor_shear` implements individual flange contributions for symmetric/asymmetric I shapes, channels, double channels and T shapes; wall checks for boxes; pipe yielding/buckling; and supplied effective shear area for rectangular, circular, general and SD sections. Positive finite properties are required. Ambiguous angle orientation and other unsupported section types return an error instead of using an invented area.

Aluminum and cold-formed materials are rejected by the steel-code workflow. Boxes/pipes with nonzero torsion are rejected until their required H3 interaction is implemented. CSI's AISC implementation explicitly ignores torsion for open section shapes; this behavior is retained. EC3 and concrete local-3 shear remain guarded rather than borrowing AISC equations.

## Formula evidence and assumptions

Primary reference: supplied `extracted/files/sfd_aisc_360_10.pdf`, section 3.5.4.2, printed pages 3-70/3-71 (PDF pages 105/106). Both pages were rendered and inspected; extracted text and rendered pages are in `target/reconstruction/`.

The minor-direction equations use `Vn = 0.6 Fy Aw Cv`, flange `kv = 1.2`, the three G2 coefficient branches, LRFD factor 0.90 and ASD divisor 1.67. Asymmetric I flange contributions use their own slenderness independently. Box minor-wall clear width uses the corresponding major-wall convention, `b - 3 tw`, with `kv = 5`. Pipe resistance uses the manual's conservative single buckling expression, limited by `0.6 Fy`, times `Ag/2`. Section 3.5.5 documents the closed-section torsion distinction.

Dimensions are assumed to be **design dimensions**. The model does not store ERW/SAW classification or the vendor's thickness-reduction overwrite. This change does not implement those defaults, overhaul the previous major-shear code, or certify the other axial/flexural capacity branches. EC3 interaction and shape classification remain incomplete.

## Validation

Tests cover independently substituted flange strengths/design factors, all three buckling coefficient branches, box/pipe/effective-area strengths, invalid/missing dimensions and unsupported shapes, actual V3-governed design output, retained EC3 rejection, material mismatch, closed torsion rejection, and clearing stale results. Existing steel/concrete reference tests remain part of the workspace suite. See `validation.json` for final integrated execution results.

Next: reconcile the existing major-shear implementation with each shape-specific vendor equation; add H3 closed-section torsion/shear/bending interactions; represent thickness/manufacturing overwrites; validate complete member reports against vendor output. Formula-unit tests alone are not a full code-compliance or vendor-parity result.
