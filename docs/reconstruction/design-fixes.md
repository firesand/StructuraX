# Design review fixes

Checkpoint: 2026-10-15. Scope is the wave4 defects in the latest StructuraX design review. The fixes use the supplied CSI AISC 360-10 and Eurocode 3-2005 design manuals in `target/reconstruction/`; they do not claim whole-code or vendor parity.

## AISC closed-section torsion and H3-6

The HSS torsional constant uses the matching wall deduction for each outside dimension: `2 (B - tw) (H - tf) min(tw, tf) - 4.5 (4 - pi) min(tw, tf)^3`. Its slenderness check takes the maximum of the corresponding flat-width ratios `(B - 3 tw)/tf` for a flange and `(H - 3 tf)/tw` for a web. This follows the H3.1 expression and the instruction to use the maximum width/thickness or depth/thickness ratio in the supplied manual, `target/reconstruction/aisc36010.txt:5188-5215` (printed pp. 3-72–3-73). The unequal-wall capacity regression uses B=6 in, H=10 in, tw=.25 in, tf=.50 in; a second unequal-wall regression exercises the inelastic slenderness branch with B=20 in, H=10 in, tw=.10 in, tf=1 in.

H3-6 now has the manual's separate equations (`aisc36010.txt:5583-5607`, printed p. 3-80). For Box, the axial and flexural ratios add linearly; for Pipe, the individual flexural ratios are squared. Both sum V2/Vc2, V3/Vc3, and T/Tc before squaring. The regressions exercise both section types with nonzero demands and independently reconstruct total utilization from the reported ratios and capacities. This also verifies that the reported P/M/V capacities and ratios correspond to the governing station's H3-6 calculation.

## EC3 local-3 shear area

`FrameSection.as3` is explicitly the analysis shear area used for shear deformation, with zero disabling that behavior (`crates/sap-model/src/lib.rs:368-372`). The EC3 design routine does not read or validate `as3`. For local-3, it supports solid rectangles (`Av=A`), equal-wall rectangular `Box`/RHS with load parallel to width (`Av=A b/(b+h)`), and circular hollow `Pipe` (`Av=2A/pi`) under EN 1993-1-1 6.2.6(3); the supplied Eurocode manual confirms its area basis and resistance expression (`target/reconstruction/ec3.txt:2086-2110`, printed p. 7-1). Unsupported I/channel/tee/circle local-3 directions and unequal-wall boxes now return `UnsupportedEc3MinorShearSection` instead of applying analysis As3 or a guessed area. Tests provide NaN or deliberately wrong positive `as3` and independently substitute the supported formulas.

## Concrete beam M2 boundary

Beam local-2 flexure is now explicitly rejected for both ACI 318-14 and EC2. `ConcreteDesignResult` contains only `as_top` and `as_bot`; using their max to report local-2 side-face reinforcement conflates two different faces and cannot produce a reliable detailing requirement. No shared model/API schema was changed. ACI and EC2 tests assert that nonzero beam M2 returns `UnsupportedBeamMinorAxisFlexure` and clears results instead of presenting a plausible-looking top/bottom design. Concrete columns continue to use their existing biaxial PMM interaction result. `docs/reconstruction/design-wave4.md` was corrected to describe this support boundary.

## Validation

`cargo test -p sap-design` and `cargo clippy -p sap-design --all-targets -- -D warnings` are the required checks for this slice. These formula and rejection regressions establish local behavior only; they are not an end-to-end SAP2000 comparison.
