# Design review fixes

Checkpoint: 2026-10-15. Scope is the wave4 defects in the latest StructuraX design review. The fixes use the supplied CSI AISC 360-10 and Eurocode 3-2005 design manuals in `target/reconstruction/`; they do not claim whole-code or vendor parity.

## AISC closed-section torsion and H3-6

The HSS torsional constant uses the matching wall deduction for each outside dimension: `2 (B - tw) (H - tf) min(tw, tf) - 4.5 (4 - pi) min(tw, tf)^3`. Its slenderness check takes the maximum of the corresponding flat-width ratios `(B - 3 tw)/tf` for a flange and `(H - 3 tf)/tw` for a web. This follows the H3.1 expression and the instruction to use the maximum width/thickness or depth/thickness ratio in the supplied manual, `target/reconstruction/aisc36010.txt:5188-5215` (printed pp. 3-72–3-73). The unequal-wall capacity regression uses B=6 in, H=10 in, tw=.25 in, tf=.50 in; a second unequal-wall regression exercises the inelastic slenderness branch with B=20 in, H=10 in, tw=.10 in, tf=1 in.

H3-6 now has the manual's separate equations (`aisc36010.txt:5583-5607`, printed p. 3-80). For Box, the axial and flexural ratios add linearly; for Pipe, the flexural term is the square root of the sum of the squares of the two flexural ratios (rendered vendor page, printed p. 3-81; the earlier text of this paragraph, which squared each ratio, came from text extraction that dropped the radical). Both sum V2/Vc2, V3/Vc3, and T/Tc before squaring. The regressions exercise both section types with nonzero demands and independently reconstruct total utilization from the reported ratios and capacities. This also verifies that the reported P/M/V capacities and ratios correspond to the governing station's H3-6 calculation.

## EC3 local-3 shear area

`FrameSection.as3` is explicitly the analysis shear area used for shear deformation, with zero disabling that behavior (`crates/sap-model/src/lib.rs:368-372`). The EC3 design routine does not read or validate `as3`. For local-3, it supports solid rectangles (`Av=A`), equal-wall rectangular `Box`/RHS with load parallel to width (`Av=A b/(b+h)`), and circular hollow `Pipe` (`Av=2A/pi`) under EN 1993-1-1 6.2.6(3); the supplied Eurocode manual confirms its area basis and resistance expression (`target/reconstruction/ec3.txt:2086-2110`, printed p. 7-1). Unsupported I/channel/tee/circle local-3 directions and unequal-wall boxes now return `UnsupportedEc3MinorShearSection` instead of applying analysis As3 or a guessed area. Tests provide NaN or deliberately wrong positive `as3` and independently substitute the supported formulas.

## Concrete beam M2 boundary

Beam local-2 flexure (M2) is designed as side-face steel `as_side` for both ACI 318-14 and EC2. This is a StructuraX extension (ledger DSGN-01) beyond SAP2000: the vendor designs beams for major-direction flexure, shear and torsion only and leaves axial force and minor-direction bending to the user (ACI 318-14 manual p. 3-28, EC2 manual p. 3-22), so SAP2000 reports neither an M2 steel area nor an M2 ratio for a beam. `as_side` comes from M2 alone, as its own rectangular section of width `h` and depth `b` minus cover, and no check combines M2 with M3. `ConcreteDesignResult` keeps `as_top` and `as_bot` for the major axis. The M2 threshold is dimensionless: `|M2|` is designed only when it exceeds `1e-9` times the largest absolute M2 or M3 of that beam over its stations and combinations, so a rounding-level M2 gives no `as_side` and conversion between force and length units preserves the decision. Concrete columns continue to use their biaxial PMM interaction result.

## Validation

`cargo test -p sap-design` and `cargo clippy -p sap-design --all-targets -- -D warnings` are the required checks for this slice. These formula and rejection regressions establish local behavior only; they are not an end-to-end SAP2000 comparison.
