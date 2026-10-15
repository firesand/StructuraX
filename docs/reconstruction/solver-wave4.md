# Solver reconstruction wave 4 — 3D solids, catenary cables, member limits, and nonlinear dynamics

> Review update: known correctness defects and inaccurate API/acceptance claims are documented in [the post-wave-4 review](../review/structurax-latest/REVIEW.md). This historical implementation note does not supersede those findings.

Checkpoint: 2026-10-15. This wave adds 8-node 3D isoparametric solid elements (with optional Taylor-Beresford-Wilson incompatible bending modes), 3D elastic catenary cable elements (Peyrot & Goulois, 1979), frame tension/compression limits (`tc_limits`), and nonlinear direct-integration time-history analysis (`NonlinearHistory` / `NonlinearDynamic`) with state continuation (`initial_case`) for `Gap`, `Hook`, and Maxwell `Damper` links. Full SAP2000 v19 solver parity remains incomplete.

## 1. 8-Node 3D Solid Element (`crates/sap-solver/src/solid.rs`)

Following CSI *Analysis Reference Manual* (July 2016) Chapter XIII ("The Solid Element"):

- **Node ordering and degenerate elements**: Uses the CSI binary node numbering `j1..j8` on $[-1, 1]^3$ (`j1 = (-1,-1,-1)`, `j2 = (+1,-1,-1)`, `j3 = (-1,+1,-1)`, `j4 = (+1,+1,-1)`, `j5 = (-1,-1,+1)`, `j6 = (+1,-1,+1)`, `j7 = (-1,+1,+1)`, `j8 = (+1,+1,+1)`). Supports degenerate 6-node wedges and collapsed hexahedra with repeated joint IDs, validating positive volume and non-inverted Gauss-point Jacobians.
- **Taylor-Beresford-Wilson (1976) incompatible bending modes**: When `SolidSection.incompatible_modes` is `true` on an 8-distinct-node brick, 9 internal bubble modes $P_m(\boldsymbol{\xi}) = 1 - \xi_m^2$ ($m = 1, 2, 3$) are evaluated with the modified center-Jacobian gradient $\frac{\det \mathbf{J}_0}{\det \mathbf{J}(\boldsymbol{\xi}_g)} \mathbf{J}_0^{-T} \nabla_{\boldsymbol{\xi}} P_m(\boldsymbol{\xi}_g)$ and statically condensed into the $24 \times 24$ stiffness matrix $\mathbf{K}_{\text{cond}} = \mathbf{K}_{uu} - \mathbf{K}_{uq} \mathbf{K}_{qq}^{-1} \mathbf{K}_{uq}^T$. This formulation satisfies the constant-strain 3D Patch Test identically on arbitrary skewed hexahedra while eliminating parasitic shear locking in bending.
- **Constitutive matrix, loads, and stress recovery**: Supports isotropic and orthotropic materials rotated by `mat_angle_deg`, property modifiers `[E1, E2, E3, G12, G13, G23, Mass, Weight]`, lumped translational mass, gravity self-weight, inward-normal surface pressure on `Face 1..6` via $2 \times 2$ surface Gauss quadrature, and uniform thermal strain $\boldsymbol{\epsilon}_0 = [\alpha_1 \Delta T, \alpha_2 \Delta T, \alpha_3 \Delta T, 0, 0, 0]^T$. Recovers `SolidResult` (`center_stress`, `gauss_stresses`, extrapolated `nodal_stresses`, principal stresses, von Mises stress, and global `nodal_forces` for support reaction equilibrium).
- **Verification**:
  - `Example 5-001` (`example_5_001_comp.sdb` and `example_5_001_incomp.sdb`): MacNeal & Harder (1985) 7-element irregular unit cube Patch Test reproduces $\sigma_{11}=\sigma_{22}=\sigma_{33}=2000\text{ psi}$, $\tau_{12}=\tau_{13}=\tau_{23}=400\text{ psi}$, and Joint 8 displacement $(0.0008885, 0.0011785, 0.001157)\text{ in}$ to machine precision ($<10^{-11}$).
  - `Example 5-002` (`example_5_002a1_incomp.sdb`) and `Example 5-005` (`example_5_005a.sdb`): Crooked solid beam and pre-twisted beam under tip shear, bending, and extension match SAP2000 reference tip deflections.

## 2. 3D Elastic Catenary Cables & Frame Tension/Compression Limits (`crates/sap-solver/src/cable.rs`)

Following CSI *Analysis Reference Manual* Chapter IX ("The Cable Element") and Peyrot & Goulois (1979):

- **Exact 3D thermal-elastic catenary**: Integrates the extensible catenary equations in the gravity/load plane $(\mathbf{e}_x, \mathbf{e}_z)$ with analytical $2 \times 2$ flexibility matrix $\mathbf{F}_{\text{flex}}(H, F_4)$, exact in-plane tangent stiffness $\mathbf{K}_{2\text{D}} = \mathbf{F}_{\text{flex}}^{-1}$, and out-of-plane pendulum geometric stiffness $K_{yy} = H / l_x$. Analytically handles the vertical cable limit $l_x \to 0$ across both hanging-loop and taut-vertical regimes (`Cable C6` in `Example 7-001`) as well as unloaded tension-only straight cables ($|\mathbf{w}| \to 0$).
- **Multi-element cable network equilibrium (`solve_cable_network`)**: Solves coupled 3D catenary networks via Newton-Raphson with backtracking line search and supports shape initialization from undeformed length $L_0$, horizontal tension $H$, or target end tension $T_I$, plus state continuation (`initial_case`).
- **Frame tension/compression limits (`tc_limits`)**: Implements axial tension/compression cutoff (`FrameObj.tc_limits`) in `nonlinear.rs` with positive-definite tangent regularization and accounting for initial member fixed-end axial forces.
- **Verification**:
  - `Example 7-001` (`example_7_001.sdb`): Reproduces support reactions $(F_x, F_z)$ across all 6 cable configurations `C1..C6` (`DEADTEMP` and `DISP1..DISP5`).
  - `Example 7-002` (`example_7_002.sdb`): Two-segment suspended cable under concentrated load (Tibert, 1999) reproduces Point 2 incremental displacements $U_x = -2.819\text{ ft}$ and $U_z = -18.457\text{ ft}$ across `DEAD`, `PTLOAD`, and `COMB1`.

## 3. Nonlinear Direct-Integration Dynamics & State Continuation (`crates/sap-solver/src/dynamics.rs`, `nonlinear.rs`)

- **Nonlinear direct-integration time history (`NonlinearStepper`)**: Supports `LoadCaseType::NonlinearHistory` and `LoadCaseType::NonlinearDynamic` using Newmark average/linear acceleration, Hilber-Hughes-Taylor (`HHT-alpha`), and `Wilson-theta` with Newton-Raphson + line-search equilibrium iterations at every time step for `Gap`, `Hook`, and Maxwell-series `Damper` ($f_d = c |\dot{d}_c|^{\text{exp}} \operatorname{sgn}(\dot{d}_c)$ in series with spring $k_d$) links.
- **Initial-state continuation (`initial_case`)**: Propagates converged displacement and link state across chained `NonlinearStatic` and `NonlinearHistory`/`NonlinearDynamic` cases.
- **Verification**:
  - `Example 6-003` (Portal Frame with Grounded Gap Link, Model A & B): Verifies `NLSTAT1..3` and `NLDHIST1..3` (`P = -4.5338\text{ kips}`, uplift $U_z = 3.9169\text{ in}$).
  - `Example 6-004` (`example_6_004.sdb`): Hook link under thermal contraction ($U_z = 0.049177\text{ in}$, $P = 45.886\text{ kips}$).
  - `Example 6-005` (`example_6_005a.sdb`): Maxwell viscoelastic damper under sinusoidal excitation matches SAP2000 reference displacements at $t = 0.50\text{ s}$, $5.00\text{ s}$, and $11.00\text{ s}$.

## Explicit Remaining Boundaries

- Hysteretic Bouc-Wen (`PlasticWen`), friction isolator, and multi-linear plastic links in dynamic history, Fast Nonlinear Analysis (`FNA`), and coupled contact + large-displacement/P-Delta remain outside the verified subset.
- General viscous damping on free massless coordinates without elastic/link support, exact constraint equations for fixed link DOFs, and vendor-runtime differential testing across the full 336-file corpus remain pending.

