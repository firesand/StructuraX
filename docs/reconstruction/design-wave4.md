# Design reconstruction wave 4 — AISC Angle/DblAngle V3 shear, H3 closed-section torsion, EC3 V3 shapes, and concrete minor-axis V3/M2

> Review update: known correctness defects and inaccurate API/acceptance claims are documented in [the post-wave-4 review](../review/structurax-latest/REVIEW.md). This historical implementation note does not supersede those findings.

Checkpoint: 2026-10-15. This wave expands `sap-design` across three previously rejected demand/shape branches while retaining explicit errors for unverified combinations.

## 1. AISC 360-10 Angle & Double-Angle Local-3 Shear (`V3`) and EC3 Local-3 Shear Shapes (`crates/sap-design/src/shear.rs`)

- **AISC 360-10 Section G4 (`Angle` and `DblAngle`)**:
  - Implements minor-axis shear strength $V_{n3} = 0.60 F_y A_{v3} C_v$ with $A_{v3} = b t$ for single angles (`FramePropType::Angle`) and $A_{v3} = 2 b t$ for double angles (`FramePropType::DblAngle`), leg slenderness $\lambda = b / t$, $k_v = 1.2$, $\phi_v = 0.90$ (LRFD), and $\Omega_v = 1.67$ (ASD), matching CSI *Steel Frame Design Manual AISC 360-10* Section 3.5.4.2.
- **Eurocode 3 (`compute_ec3_minor_shear`)**:
  - Extends EC3 6.2.6(3) local-3 plastic shear resistance $V_{\text{pl},3,\text{Rd}} = A_{v3} (f_y / \sqrt{3}) / \gamma_{M0}$ beyond `Rectangular` to `I`, `Channel`, `DbChannel`, `T`, `Box`, `Pipe`, and `Circle` sections (with high-shear minor-bending reduction $\rho = (2 |V_3| / V_{\text{pl},3,\text{Rd}} - 1)^2$ when $|V_3| / V_{\text{pl},3,\text{Rd}} > 0.5$).
  - Concurrent axial/major-bending/major-shear/torsion demands with nonzero `V3` under EC3 remain explicitly rejected (`DesignError::UnsupportedEc3ShearInteraction`).

## 2. AISC 360-10 Closed-Section (`Box` and `Pipe`) Torsion & H3-6 Interaction (`crates/sap-design/src/steel.rs`)

- **AISC 360-10 Section H3.1 (`compute_aisc_torsional_capacity`)**:
  - **Round HSS (`Pipe` / `ColdPipe`)**: Torsional constant $C = \frac{\pi (D - t)^2 t}{2}$, critical stress $F_{cr} = \min\left(\max(F_{cr1}, F_{cr2}), 0.60 F_y\right)$ with $F_{cr1} = \frac{1.23 E}{\sqrt{L/D} (D/t)^{5/4}}$ and $F_{cr2} = \frac{0.60 E}{(D/t)^{3/2}}$, $T_n = F_{cr} C$, $\phi_T = 0.90$ (LRFD), $\Omega_T = 1.67$ (ASD).
  - **Rectangular HSS (`Box` / `ColdBox`)**: Torsional constant $C = 2(B - t)(H - t)t - 4.5(4 - \pi)t^3$, flat wall slenderness $h/t = (\max(H, B) - 3t)/t$ across compact yielding ($h/t \le 2.45\sqrt{E/F_y}$), inelastic buckling ($h/t \le 3.07\sqrt{E/F_y}$), and elastic buckling ($h/t \le 260$).
- **AISC 360-10 Section H3.2 Interaction (Eq. H3-6)**:
  - When $|T_r| > 0.20 T_c$ (or when $|T_r| / T_c$ governs), evaluates:
    $$\left(\frac{|P_r|}{P_c} + \frac{|M_{r3}|}{M_{c3}} + \frac{|M_{r2}|}{M_{c2}}\right) + \left(\max\left(\frac{|V_{r2}|}{V_{c2}}, \frac{|V_{r3}|}{V_{c3}}\right) + \frac{|T_r|}{T_c}\right)^2$$
    and reports `eq_name = "H3-6"` when it governs over H1 PMM interaction.
  - Closed sections missing wall thickness ($t \le 0$) or analyzed under Eurocode 3 with nonzero torsion still return `DesignError::UnsupportedClosedSectionTorsion`.

## 3. Concrete Minor-Axis Shear (`V3`) & Beam Minor-Axis Flexure (`M2`) (`crates/sap-design/src/concrete.rs`)

- Implements minor-axis shear reinforcement sizing $(A_v / s)_3$ and web-crushing overstress checks ($V_{u3} > \phi V_{n3,\max}$) for both `ACI 318-14` and `Eurocode 2-2004` beams and columns using the 3-direction effective web width and depth ($b_w = h$, $d_3 = b - \text{cover}$ for rectangular sections; $D$ and $0.8 D$ for circular columns), reporting `av_over_s = av_over_s_2.max(av_over_s_3)`.
- Implements concrete beam minor-axis flexural reinforcement sizing for nonzero `M2` under `ACI 318-14` and `Eurocode 2-2004`.
- Nonzero concrete torsion (`|T| > 1e-9`) remains explicitly rejected (`DesignError::UnsupportedDemand`).

