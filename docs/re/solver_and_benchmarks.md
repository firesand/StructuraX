# CSI SAPFire® & CsiGo2 FEM Solver Architecture and Verification Benchmarks Specification

## 1. Executive Summary & Binary Inventory

SAP2000 v19.0.0 delegates all finite element formulation, equation numbering, matrix assembly, linear/nonlinear equation solving, eigenvalue extraction, and element state recovery to the **CSI SAPFire® Analysis Engine** (`CSI.SAPFire.*` and `CsiGo2*.dll`).

### 1.1 Analyzed Solver Artifacts (`/home/edo/REA/extracted/files/`)

| Binary | Size (Bytes) | SHA-256 Digest | Runtime / Format | Architectural Role |
| :--- | ---: | :--- | :--- | :--- |
| `quickanalysis.dll` | 17,840 | `971fb6093d08c03289ae5db812ef54ee0746885676e025ba9e65a31267a54c3f` | .NET v4.0 (`il-only`, `x86_64`) | High-level managed orchestrator (`CSI.QuickAnalysis.cQuickAnalysis`) bridging `SAP2000.exe` to `CSI.SAPFire.dll`. |
| `csi.sapfire.common.dll` | 29,104 | `20d42d371915bd7db5a310a5e9bd43918b556e8355b8a4ca529954c772e90cf8` | .NET v4.0 (`il-only`, `x86_64`) | Shared interfaces (`ICallback`, `ICallbackResponse`), system memory inspection (`cSystemInfo` via `GlobalMemoryStatusEx`), and `cNonblockingWait` (`MsgWaitForMultipleObjects`). |
| `csi.sapfire.driver.exe` | 41,904 | `13503542d4ca0da5e1275126f41e9478791df2737041c456ce6101cd7c667bd2` | Mixed-Mode C++/CLI (`x86_64`) | Out-of-process analysis driver executable (`eTypeProcess.Analysis`) hosting `CSI.SAPFire.Driver.cCallback` via .NET Remoting / IPC. |
| `csi.sapfire.go.dll` | 60,336 | `f02e5d9028c1ef56c78eb20124658c8bd14275b5c0c9801d63218975743c0bd8` | .NET v4.0 (`il-only`, `x86_64`) | Managed P/Invoke wrapper (`CSI.SAPFire.CsiGo`, `CSI.SAPFire.HistSpec`) declaring 340 P/Invoke imports into `CsiGo2.dll`. |
| `csi.sapfire.dll` | 3,735,984 | `c3142300a763bf0dfc07e96b591fd152717d4dbd799512c4aeb88b55affda5c6` | Mixed-Mode C++/CLI (`x86_64`) | Object-oriented analysis model (`cAnalysisModel`, `cServer`, `cJob`, 732 managed types) **and** native C++ object store exporting 3,101 `Go*` C-callable accessors consumed by `csigo2.dll`. |
| `csigo2.dll` | 31,708,080 | *(Native PE32+ x86_64)* | Intel Fortran + C++ (`libifcoremd.dll`, OpenMP) | Core FEM analysis engine (64 Fortran modules, 1,368 internal `*_SBR`/`*_FCN` routines, 463 exports: 416 `Go_*` + 47 `CsiGoCB_*`). |
| `csigo2_f.dll` | 362,416 | *(Native PE32+ x86_64)* | Native C++ (`MSVCP110.dll`, OpenMP) | Catenary/cable element and element-state object engine exporting 123 `CsiGoAPI_*`, `Element*`, `Property*`, `LoadElement*`, and `StateElement*` functions. |
| `csigo2_a.dll` | 366,336 | *(Native PE32+ x86_64)* | Native C/C++ | Sparse matrix assembly, METIS ordering (`ONMETIS` / `MMDOrder`), Sturm sequence counting, and solver dispatch exporting 59 `Csi_sFire_*` functions. |
| `csigo2_d.dll` | 14,592 | *(Native PE32+ x86_64)* | Native C/C++ | Thin bridge exporting `SFire2`, linking `csigo2_a.dll` to Intel MKL `PARDISO` in `csigo2_n.dll`. |
| `csigo2_n.dll` | 129,003,440 | *(Native PE32+ x86_64)* | Intel MKL Custom Build (`libiomp5md.dll`) | Multi-threaded numerical kernel exporting 40 symbols: `PARDISO`, BLAS/LAPACK (`dgemm`, `dgemv`, `dgetrf`, `dpotrf`, `dsyrk`, `dtrsm`, etc.), and 1D FFT (`Dfti*` / `dfti_*`). |
| `csi.sapfire.solver.exe` | 104,192 | *(Native PE32+ x86_64)* | Native C++ (`CsiGo2_a.dll`) | Separate-process sparse matrix factorization and block vector solver (`Csi_sFire_SolveVectorsAsSeparateProcess`). |

---

## 2. Layered Solver Architecture & Native P/Invoke Boundaries

```
+-----------------------------------------------------------------------------------+
| SAP2000.exe (VB.NET / C# Host)                                                    |
|   |                                                                               |
|   v                                                                               |
| QuickAnalysis.dll (CSI.QuickAnalysis.cQuickAnalysis)                              |
|   - OpenQuickAnalysis(path), RunQuickAnalysis(model), CloseQuickAnalysis()        |
|   - Unit conversion factors: _FactorForceToKN, _FactorLengthToM, _FactorTempToC   |
+-----------------------------------------------------------------------------------+
    | (In-Process GUI or Out-of-Process via CSI.SAPFire.Driver.exe)
    v
+-----------------------------------------------------------------------------------+
| CSI.SAPFire.dll (Mixed-Mode C++/CLI: 732 Managed Types + 3,101 Native Exports)    |
|   - Managed API: cServer, cAnalysisModel, cJob, cNode, cJoint, cFrame, cShell,    |
|     cPlane, cAsolid, cSolid, cCable, cLink, cHinge*, cConstraint*, cProp*,        |
|     cLoadPattern*, cAnalysisCase (cLinStatic, cNonlinStatic, cLinModal, etc.)     |
|   - Native C++ Backing Store: Exports 3,101 Go<Class>_<Method> functions          |
|     (e.g., GoAnalysisModel_*, GoFrame_*, GoShell_*, GoSolid_*, GoJoint_*)         |
+-----------------------------------------------------------------------------------+
    |                                                   ^
    | Calls CSI.SAPFire.Go.dll                          | Callback reads model data
    | (340 P/Invoke imports into CsiGo2.dll)            | via 3,101 Go* C exports
    v                                                   |
+-----------------------------------------------------------------------------------+
| CsiGo2.dll (Intel Fortran 90/2003 Core FEM Engine - 31.7 MB)                      |
|   - 64 Fortran Modules (SapFire, Stiff, Eigen, Ritz, BuckLin, StatNon, HisMode,   |
|     HisDLin, HisDNon,HisDLinFreq, Respec, Bridge, Frame, Pipe, ShellElem,         |
|     Plasol, Solid, Cable, Nllink, Pf3DPMM, Fiber, Facet, Plastic, Constr, ...)    |
+-----------------------------------------------------------------------------------+
    |                               |
    | Element/Cable State           | Sparse Matrix Assembly, Ordering & Factorization
    v                               v
+-----------------------+     +-----------------------------------------------------+
| CsiGo2_f.dll          |     | CsiGo2_a.dll (59 Csi_sFire_* Exports)               |
| - ElementCable_*      |     | - CCS / CRS Sparse Matrix Storage                   |
| - StateElementCable_* |     | - METIS (ONMETIS Nested Dissection + MMDOrder)      |
| - LoadElement*        |     | - Standard Skyline/Block Active Column Solver       |
| - PropertyCable_*     |     | - Advanced Solver (SFire2) & Multi-Threaded Solver  |
+-----------------------+     +-----------------------------------------------------+
                                    |                   |                   |
              +---------------------+                   |                   +-------------------+
              v                                         v                                       v
+-------------------------------+     +-----------------------------------+     +-------------------------------+
| csi.sapfire.solver.exe        |     | CsiGo2_d.dll -> SFire2            |     | CsiGo2_n.dll (129 MB MKL)     |
| - Out-of-process matrix       |     | - Direct sparse solver bridge     |     | - PARDISO (Parallel Direct)   |
|   factorization & block       |     |   calling PARDISO & MKL           |     | - BLAS3/LAPACK: dgemm, dgetrf,|
|   vector solve via files      |     +-----------------------------------+     |   dpotrf, dsyrk, dtrsm        |
+-------------------------------+                                               | - FFT: DftiComputeForward/Bwd |
                                                                                +-------------------------------+
```

### 2.1 Managed-to-Native P/Invoke Boundary (`CSI.SAPFire.Go.dll` -> `CsiGo2.dll`)

`CSI.SAPFire.Go.dll` contains two static P/Invoke classes (`CSI.SAPFire.CsiGo` with 330 P/Invoke methods and `CSI.SAPFire.HistSpec` with 10 P/Invoke methods) targeting `CsiGo2.dll` (`CallingConvention.Winapi`, Fortran pass-by-reference `i4&`, `r8&`):

1. **Element & Material Constant Accessors (`Go_k*FCN`)**:
   - Element type IDs: `Go_kElemJointFCN`, `Go_kElemReactFCN`, `Go_kElemSpringFCN`, `Go_kElemFrameFCN`, `Go_kElemPipeFCN`, `Go_kElemAreaFCN`, `Go_kElemShellFCN`, `Go_kElemPlaneFCN`, `Go_kElemAsolidFCN`, `Go_kElemSolidFCN`, `Go_kElemNllinkFCN`.
   - Material directional symmetry & yield criteria: `Go_kTypeMatDirSymIsotropicFCN`, `Go_kTypeMatDirSymOrthotropicFCN`, `Go_kTypeMatDirSymAnisotropicFCN`, `Go_kTypeMatDirSymUniaxialFCN`, `Go_kTypeMatYieldNoneFCN`, `Go_kTypeMatYieldDruckerPragerFCN`, `Go_kTypeMatYieldFiberFrictionFCN`, `Go_kTypeMatYieldDarwinPecknoldFCN`.
   - Hysteresis & time-dependent concrete models: `Go_kTypeMatMultiElasticFCN`, `Go_kTypeMatMultiPlastKinemFCN`, `Go_kTypeMatMultiPlastTakedaFCN`, `Go_kTypeMatMultiPlastPivotFCN`, `Go_kTypeMatMultiPlastConcreteFCN`, `Go_kTypeMatMultiPlastBRBHardeningFCN`, `Go_kTypeMatTimeDepAci209r92FCN`, `Go_kTypeMatTimeDepCEBFIPFCN`, `Go_kTypeMatTimeDepCEBFIP10FCN`.
   - Nonlinear geometric options: `Go_kNLGeomCaseNoneFCN`, `Go_kNLGeomCasePdeltaFCN`, `Go_kNLGeomCaseLargeDispFCN`.
   - Direct time-integration schemes: `Go_kTypeIntegrateNewmarkFCN`, `Go_kTypeIntegrateWilsonFCN`, `Go_kTypeIntegrateCollocFCN`, `Go_kTypeIntegrateHHTAlphaFCN`, `Go_kTypeIntegrateChungHulFCN`, `Go_kTypeIntegrateModakFCN`.
   - Modal & directional combination rules: `Go_kCombModeSpectrumCqcFCN`, `Go_kCombModeSpectrumSrssFCN`, `Go_kCombModeSpectrumAbsFCN`, `Go_kCombModeSpectrumGmcFCN`, `Go_kCombModeSpectrumNrc10FCN`, `Go_kCombModeSpectrumNrcDSFCN`, `Go_kCombDirSpectrumSrssFCN`, `Go_kCombDirSpectrumAbsFCN`, `Go_kCombDirSpectrumCqc3FCN`.

2. **Coordinate Transformation & Utility Routines**:
   - `Go_LocalCoordPoint`, `Go_LocalCoordLine`, `Go_LocalCoordArea`, `Go_LocalCoordSolid`, `Go_PrincipalAxesOfPoints`.
   - Time-dependent concrete curves: `Go_rStrengthMeanConcrete*`, `Go_rModulusConcrete*`, `Go_rStrainShrinkageConcrete*`, `Go_rCoeffCreepConcrete*`, `Go_rCoeffRelaxSteelPrestressCebFip*`.
   - P-M-M interaction surface validation: `Go_CheckFacetPmmSurf`.
   - Time-history response spectrum generation (`CSI.SAPFire.HistSpec`): `Go_HistSpecInitrs`, `Go_HistSpecEqual`, `Go_HistSpecFrelst`, `Go_HistSpecSpecfr`, `Go_HistSpecPeaks`, `Go_HistSpecSpack`, `Go_HistSpecSpread`, `Go_HistSpecSpscale`, `Go_HistSpecGetaxs`, `Go_HistSpecCqc`, and `Go_TimeDomainMatch`.

3. **Job Execution & Case Configuration (`Go_Job*`)**:
   - Case setup: `Go_JobCaseInit`, `Go_JobCaseLoadSet`, `Go_JobCaseStaticLin`, `Go_JobCaseStaticHyper`, `Go_JobCaseCenterOfRigidity`, `Go_JobCaseStaticLinMulti`, `Go_JobCaseStaticNonlin01`, `Go_JobCaseModal`, `Go_JobCaseSpectrum`, `Go_JobCaseHistModalLin`, `Go_JobCaseHistModalNonlin`, `Go_JobCaseHistDirectLin`, `Go_JobCaseHistDirectLinFreq`, `Go_JobCaseHistDirectNonlin`, `Go_JobCaseBuckling`, `Go_JobCaseUDSS` (Steady-State), `Go_JobCasePsdDirect` (Power Spectral Density), `Go_JobCaseBridgeInfline`, `Go_JobCaseMovingLoad`.
   - Nonlinear solver controls: `Go_JobCaseIterationMaxConstantStiffSet`, `Go_JobCaseIterationMaxNewtonRaphsonSet`, `Go_JobCaseStepMaxLineSearchSet`, `Go_JobCaseTolRelLineSearchSet`, `Go_JobCaseFactorStepLineSearchSet`, `Go_JobCaseLoadApplicationControlSet`, `Go_JobCaseIterationMaxTargetForceSet`, `Go_JobCaseTolRelConvergeForceTargetForceSet`, `Go_JobCaseStageAction01`.
   - Response extraction: `Go_JobJointResponse`, `Go_JobFrameResponse`, `Go_JobPipeResponse`, `Go_JobShellResponse`, `Go_JobShellLayerResponse`, `Go_JobPlaneResponse`, `Go_JobSolidResponse`, `Go_JobNllinkResponse`, `Go_JobFrameHingeResponse`, `Go_JobModalPeriod`, `Go_JobModalStiffnessMass`, `Go_JobModalParticipation`, `Go_JobBucklingFactor`, `Go_JobEnergyResponse`, `Go_JobBaseResponse`, `Go_JobElementJointForce`, `Go_JobElementVirtualForce`, `Go_JobGroupForceSum`.

### 2.2 Reverse Native Callback Interface (`CsiGo2.dll` -> `CSI.SAPFire.dll`)

Rather than serializing the entire structural model across P/Invoke arrays, `CSI.SAPFire.dll` holds native C++ objects wrapped by C++/CLI classes (`cWrapper` / `cObject` / `cTaggedObject`) and exports **3,101 C functions** (`Go<ClassName>_<Property/Action>`) that `CsiGo2.dll` imports directly to pull model geometry, properties, loads, and push analysis results:

| Export Prefix in `CSI.SAPFire.dll` | Count | Underlying C++/CLI Class | Description |
| :--- | ---: | :--- | :--- |
| `GoAnalysisModel_*` | 303 | `cAnalysisModel` | Global model settings, solver options, DOF flags, lists of nodes/elements/cases |
| `GoTypeClass_*` | 168 | `cTaggedObject` | Runtime type identification tags (`CON` constants) for all model objects |
| `GoPipe_*` / `GoPropPipe*` | 128 | `cPipe`, `cPropPipe*` | Pipe elements (with ASME flexibility & pressure factors), prismatic/variable properties |
| `GoFrame_*` / `GoPropFrame*` | 129 | `cFrame`, `cPropFrame*` | 3D frame/truss elements, releases, end offsets, non-prismatic segments (`GoRecPropFrameVar`) |
| `GoShell_*` / `GoPropShell*` / `GoLayerShell*` | 132 | `cShell`, `cPropShell*` | Homogeneous (thin/thick/membrane/plate) and multi-layered nonlinear shell elements |
| `GoPropLink*` / `GoLink_*` | 328 | `cLink`, `cPropLink*` | Linear, multilinear elastic/plastic, Wen, gap, hook, bilinear/exponential/ring-spring dampers, single/double/triple friction pendulum & hysteretic isolators |
| `GoPropHinge*` / `GoHinge*` | 130 | `cHinge*`, `cPropHinge*` | Lumped P-M2-M3 plastic hinges (`Pf3DPMM`), fiber hinges (`GoFiber`), and facet hinges |
| `GoVehicle*` / `GoLane*` / `GoBridgeResponse_*` | 195 | `cVehicle*`, `cLane*` | Bridge influence lines, moving loads, lanes, floating axle loads |
| `GoConstraint*` / `GoRecConstraintGeneral_*` | 86 | `cConstraint*` | Equal, Rigid (Body/Diaphragm/Plate/Rod/Beam), Line, Local, Weld, and General linear constraints |
| `GoElemload*` | 196 | `cElemload*` | Gravity, Uniform, SpanConc, SpanDist, Temp, Strain, Deform, TargetForce, SurfacePress, PorePress, Rotate |

---

## 3. Coordinate Systems, Element Formulations & DOFs

### 3.1 Coordinate Systems & Local Axes (`cCoordSys*`, `cLocalAxes*`, `Coor_SetupSBR`)
- **Global / Fixed Coordinate Systems (`cCoordSys`, `cCoordSysAngle`, `cCoordSysCos`)**:
  - Defined by origin $(X_0, Y_0, Z_0)$ and either 3 Euler angles (`cCoordSysAngle`) or a $3 \times 3$ direction cosine matrix (`cCoordSysCos`).
  - Supports Cartesian (`Xpos`, `Ypos`, `Zpos`), Cylindrical (`CRpos`, `CApos`, `CZpos`), and Spherical (`SRpos`, `SApos`, `SBpos`) coordinate directions (`eDirCoordSys`).
- **Element Local Axes $(1, 2, 3)$ (`Go_LocalCoordPoint`, `Go_LocalCoordLine`, `Go_LocalCoordArea`, `Go_LocalCoordSolid`)**:
  - **Line Elements (Frame, Pipe, Cable, 2-Node Link)**: Local axis `1` runs along the element axis from Node $I$ to Node $J$: $\mathbf{e}_1 = (\mathbf{x}_J - \mathbf{x}_I)/L$. Default orientation places the `1-2` plane vertical (parallel to $+Z$); if the element is vertical ($\parallel Z$), local axis `2` aligns with $+X$. Rotation angle $\beta$ (`AngleLocalAxes`) rotates axes `(2, 3)` about local axis `1` according to the right-hand rule.
  - **Area Elements (Shell, Plane, Asolid)**: Local axis `3` is normal to the element mid-surface ($\mathbf{e}_3 = \mathbf{e}_1 \times \mathbf{e}_2$). Local axis `2` projects toward $+Z$ (or $+Y$ if horizontal), and local axis `1` is horizontal ($\mathbf{e}_1 = \mathbf{e}_2 \times \mathbf{e}_3$), further rotated in-plane by `AngleLocalAxes`.

### 3.2 Finite Element Catalog (`csigo2.dll` & `csigo2_f.dll`)

| Element Class | Native Module(s) | Nodes | Active DOFs / Node | Formulation & Key Capabilities |
| :--- | :--- | :---: | :---: | :--- |
| **Joint / Node** (`cJoint`, `cNode`) | `Joint`, `Restraint` | 1 | 6 ($U_1, U_2, U_3, R_1, R_2, R_3$) | Supports skewed local axes (`cLocalAxes3D`), rigid restraints, coupled $6 \times 6$ linear springs (`cNSJointSpring`), translational/rotational masses (`cElemMass`), and ground displacement loads (`cElemloadDisp`). |
| **3D Frame / Beam-Column** (`cFrame`) | `Frame`, `FrameResp`, `PushFrame` | 2 | 6 ($U_x, U_y, U_z, R_x, R_y, R_z$) | 3D Timoshenko beam-column with axial ($EA$), biaxial bending ($EI_{33}, EI_{22}$), coupled shear deformation ($GA_{v2}, GA_{v3}$), and St. Venant torsion ($GJ$). Supports:<ul><li>**Property Modifiers**: `[A, Av2, Av3, J, I22, I33, Mass, Weight]`. Setting $A_v = 0$ disables shear deformation (recovering exact Euler-Bernoulli beam).</li><li>**End Offsets**: Lengths $a_I, a_J$ (`OffsetI`, `OffsetJ`) with rigid-zone factor $r \in [0, 1]$ (`FactorRigid`), where flexible length is $L_f = L - r(a_I + a_J)$ and rigid offset length is $L_r = r a$.</li><li>**End Releases & Partial Fixity**: Full releases (`eTypeRelease`: `None`, `I`, `J`, `Both`) or partial fixity springs ($k_P, k_{V2}, k_{V3}, k_T, k_{M2}, k_{M3}$) via static condensation (`cNSFrameReleaseStiff`).</li><li>**Non-Prismatic Variation** (`cRecPropFrameVar`): Segmental variation where $A, J$ vary linearly and $I(x) = \left[ I_1^{1/n}\left(1 - \frac{x}{L}\right) + I_2^{1/n}\left(\frac{x}{L}\right) \right]^n$ with $n=1$ (Linear), $n=2$ (Parabolic), or $n=3$ (Cubic).</li><li>**Geometric Stiffness ($K_G$)**: Exact consistent cubic Hermite P-Delta stiffness matrix (`GoPDeltaForce` or nonlinear P-Delta state).</li></ul> |
| **3D Pipe** (`cPipe`) | `Pipe`, `PipeResp` | 2 | 6 | Specialized tubular beam element with ASME B31 flexibility factors (`UseASMEFlexibilityFactor`) and internal pressure stiffening (`UseASMEPressureFactor`). |
| **Catenary Cable** (`cCable`) | `Cable` (`csigo2.dll`), `ElementCable_*` (`csigo2_f.dll`) | 2 | 3 ($U_x, U_y, U_z$) | 3D nonlinear elastic catenary / truss cable element supporting undeformed length specification (`ElementCable_SetLengthUndeformed`), target force iteration (`ElementCable_HasTargetForce`), thermal and gravity loads. |
| **Homogeneous & Layered Shell** (`cShell`) | `ShellElem`, `ShellProp`, `ShellPropLayer` | 3 or 4 | 6 | 3-node triangle or 4-node quadrilateral combining:<ul><li>**Membrane**: Isoparametric membrane with incompatible modes and optional Allman-type drilling rotation stiffness (`UseDrilling` about local `3`).</li><li>**Plate Bending**: Selectable `eTypeThickness`:<ul><li>`Thin`: Kirchhoff thin-plate formulation (neglects transverse shear $\gamma_{13}, \gamma_{23}$).</li><li>`Thick`: Mindlin-Reissner thick-plate formulation including transverse shear deformations ($\gamma_{13}, \gamma_{23}$) with selective reduced integration to prevent shear locking.</li></ul></li><li>**Behavior Modes** (`eTypeBehavior`): Full `Shell` (Membrane + Plate), pure `Membrane`, or pure `Plate`.</li><li>**Multi-Layered Shell** (`cPropShellLayered`): Through-thickness Gauss or User quadrature (`eTypeIntegThick`) over directional (`cLayerShellDirectional`) or coupled (`cLayerShellCoupled` Darwin-Pecknold / Drucker-Prager) nonlinear material layers.</li></ul> |
| **2D Plane Stress / Strain** (`cPlane`) | `Plasol` | 3 to 9 | 2 ($U_x, U_z$) | 2D continuum element (`PlaneStress` or `PlaneStrain`) in the $XZ$ plane with optional Wilson incompatible bending modes (`UseIncompatible`). |
| **Axisymmetric Solid** (`cAsolid`) | `Plasol` (`GoAsolid_*`) | 3 to 9 | 2 ($U_r, U_z$) | 2D axisymmetric solid element defined by a radian subtended angle (`Thickness`) and optional incompatible modes (`UseIncompatible`). |
| **3D Solid (Brick/Wedge/Tet)** (`cSolid`) | `Solid` | 8 (4–20) | 3 ($U_x, U_y, U_z$) | 3D 8-node isoparametric hexahedral solid element (degenerates to 6-node wedge / 4-node tetrahedron) with optional incompatible bending modes (`UseIncompatible`) to eliminate parasitic shear in bending. |
| **Nonlinear Link / Isolator** (`cLink`) | `Nllink`, `PushNllink` | 1 or 2 | 6 | 6-DOF spring/link element with shear distance ratios (`DistanceAbsU2`, `DistanceAbsU3`) and P-Delta moment distribution factors (`PDeltaM3I`, `PDeltaM3J`, `PDeltaV2`, `PDeltaV3`). Supports Linear, Multi-Linear Elastic, Multi-Linear Plastic (Kinematic, Takeda, Pivot), Gap (compression-only), Hook (tension-only), Plastic Wen, Viscous Damper (Linear/Nonlinear velocity exponent, Bilinear, Ring-Spring), Hysteretic Rubber Isolator, Single/Double/Triple Friction Pendulum Isolators, and Frequency-Dependent impedance links. |

---

## 4. Solver Algorithms & Numerical Kernels

### 4.1 Sparse Matrix Storage & Equation Ordering (`csigo2_a.dll`, `csigo2_d.dll`, `csigo2_n.dll`)
`CSI.SAPFire.cAnalysisModel` exposes three solver types via `eTypeSolver` (confirmed by log strings in `csigo2.dll`):

1. **Standard Solver (`eTypeSolver.StandardOnly`)**:
   - Log banner: `"USING THE STANDARD SOLVER (PROVIDES COMPLETE INSTABILITY INFORMATION)"`.
   - Implemented in `csigo2_a.dll` (`Csi_sFire_*` routines). Uses compressed column/row storage (`Csi_sFire_CreateMatrix`, `Csi_sFire_WriteCCSMatrix`, `Csi_sFire_PrintCRSMatrix`) with **METIS** multilevel nested dissection (`ONMETIS` / `MlevelNestedDissectionCC`) and Multiple Minimum Degree (`MMDOrder`) equation reordering (`SapFire_OrderEquationsSBR`).
   - Performs $LDL^T$ factorization (`Csi_sFire_SolveFactorInline` or out-of-process via `csi.sapfire.solver.exe` when `Csi_sFire_kFactorSeparateProcessAlways`/`Optional` is active) with full diagonal pivot tracking (`Csi_sFire_kTypeStiffInfoZeroDiag`, `SmallDiag`, `NegativeDiag`, `Roundoff`) and Sturm sequence negative pivot counting (`Csi_sFire_nSturm`).

2. **Advanced Solver (`eTypeSolver.AdvancedOnly`)**:
   - Log banner: `"USING THE ADVANCED SOLVER (PROVIDES LIMITED INSTABILITY INFORMATION)"`.
   - Invokes `SapFire_GetReorderStiffSfire2SBR` and `SapFire_SolveForwBackSfire2SBR`, which call `Csi_sFire_InitializezObjectForSFire2` (`csigo2_a.dll`) $\rightarrow$ `SFire2` (`csigo2_d.dll`) $\rightarrow$ `PARDISO` (`csigo2_n.dll`) in single-threaded / deterministic mode with supernodal Level-3 BLAS (`dgemm`, `dpotrf`, `dtrsm`).

3. **Multi-Threaded Solver (`eTypeSolver.ParallelOnly`)**:
   - Log banner: `"USING THE MULTI-THREADED SOLVER (PROVIDES NO INSTABILITY INFORMATION)"`.
   - Uses Intel MKL `PARDISO` (`csigo2_n.dll`) with OpenMP parallelism (`libiomp5md.dll`, controlled by environment variables `SAPFIRE_NUM_THREADS`, `SAPFIRE_THREADS_MIN`, and `MKL_Set_Num_Threads`).

### 4.2 Modal, Buckling, Nonlinear & Dynamic Solution Algorithms (`csigo2.dll`)
- **Eigenvalue Modal Analysis (`Module Eigen`: `Eigen_SpacemSBR`, `Eigen_SpacemShiftedSBR`)**:
  - Solves $(K - \Omega^2 M)\Phi = 0$ using **Accelerated Subspace Iteration / Block Lanczos with frequency shifting** (`Go_JobCaseModalAutoShiftSet`, `eTypeAutoShiftEigen`) and Sturm sequence checks (`SapFire_nSturmFCN`) to guarantee no missed modes in the requested frequency range $[f_{\min}, f_{\max}]$.
- **Load-Dependent Ritz Vectors (`Module Ritz`: `Ritz_GenerateSBR`, `Ritz_JacobiSolveSBR`)**:
  - Generates Krylov-subspace starting load vectors (`Ritz_StartingLoadVectorsSBR`), performs $M$- and $K$-orthogonalization (`Ritz_OrthogMassSBR`, `Ritz_OrthogStiffSBR`), and solves the reduced tridiagonal/dense eigenvalue problem via Jacobi iteration (`Ritz_JacobiFormSBR`, `Ritz_JacobiSolveSBR`).
- **Linear Buckling Analysis (`Module BuckLin`: `BuckLin_SpacemSBR`)**:
  - Forms the elastic stiffness $K$ and geometric (stress) stiffness $K_G$ from the prerequisite load case and solves $(K - \lambda K_G)\Phi = 0$ via subspace iteration (`BuckLin_SpacemSBR`, `BuckLin_OrthoNormalizeSBR`) to extract critical buckling load factors $\lambda_{cr}$ (`Go_JobBucklingFactor`).
- **Nonlinear Static & Staged Construction (`Module StatNon`: `StatNon_CaseDriverSBR`)**:
  - Supports P-Delta (`Go_kNLGeomCasePdeltaFCN`) and Large Displacement (`Go_kNLGeomCaseLargeDispFCN`) geometric nonlinearity, material nonlinearity (hinges, layered shells, links), and staged construction (`Go_kActionStageAddFCN`, `Rem`, `RemAdd`, time-dependent creep/shrinkage `Module StrainAge`).
  - Uses combined **Constant-Stiffness** (`IterationMaxConstantStiff`) and **Full Newton-Raphson** (`IterationMaxNewtonRaphson`) iteration with **Line Search** (`StepMaxLineSearch`, `TolRelLineSearch`, `FactorStepLineSearch`), **Event-to-Event stepping**, and Force/Displacement/Conjugate-Displacement load control (`Go_kLoadControlCaseForceFCN`, `Monitor`, `Conjug`).
- **Time-History & Frequency-Domain Dynamics**:
  - **Modal Time-History (`Module HisMode`)**: Closed-form piecewise-exact integration of modal coordinates with FNA (Fast Nonlinear Analysis) iterative force correction for `Nllink` elements.
  - **Direct-Integration Time-History (`Module HisDLin`, `Module HisDNon`)**: Supports **Newmark** ($\gamma, \beta$), **Wilson-$\theta$**, **Collocation**, **Hilber-Hughes-Taylor (HHT-$\alpha$)**, and **Chung-Hulbert** (`Go_kTypeIntegrate*FCN`).
  - **Frequency-Domain / Steady-State / PSD (`Module HisDLinFreq`, `Module CycFLin`, `Go_JobCasePsdDirect`)**: Uses Intel MKL 1D FFT (`dfti_compute_forward_dz`, `dfti_compute_backward_zd` in `csigo2_n.dll`) and complex frequency-swept impedance matrices.

---

## 5. SAP2000 Verification Benchmark Suite

All 86 official SAP2000 v19.0.0 verification problems were extracted from `sap2000_verification.xls`, `sap2000_verification.sbf`, and `problem_1_001.pdf` through `problem_7_003.pdf`. Below are the **exact input parameters, geometry, boundary conditions, loading, hand-calculation formulas, and target reference outputs** for **15 core verification benchmarks** (Examples `1-001` through `1-010` plus `1-014`, `1-016`, `1-019`, `2-001`, and `2-005`) ready for automated unit/integration testing of the Rust FEM engine.

---

### Benchmark 1: Example 1-001 — 2D Frame Under General Loading (`example_1_001.sdb`)
- **Features Tested**: Self-weight, global/projected/local uniform & trapezoidal distributed frame loads, concentrated span loads, joint forces/moments, Euler-Bernoulli bending (`Av = 0`, `Area modifier = 1000`).
- **Geometry ($XZ$ Plane, units: $\text{kip}, \text{ft}, \text{in}$)**:
  - Joint 1: $(0, 0, 0)\text{ ft}$ — Pinned support ($U_x = U_y = U_z = R_x = R_z = 0$, $R_y$ free).
  - Joint 2: $(0, 0, 10)\text{ ft}$ — Free in $XZ$ plane.
  - Joint 3: $(8, 0, 16)\text{ ft}$ — Horizontal roller & rotation fixed ($U_x = 0, R_y = 0$, $U_z$ free).
  - Joint 4: $(-4, 0, 10)\text{ ft}$ — Free cantilever tip.
  - Element 1 (Column): Joint 1 $\rightarrow$ Joint 2 ($L = 10\text{ ft} = 120\text{ in}$).
  - Element 2 (Inclined Girder): Joint 2 $\rightarrow$ Joint 3 ($\Delta X = 8\text{ ft}, \Delta Z = 6\text{ ft}, L = 10\text{ ft} = 120\text{ in}$).
  - Element 3 (Cantilever Overhang): Joint 4 $\rightarrow$ Joint 2 ($L = 4\text{ ft} = 48\text{ in}$).
- **Material & Section Properties**:
  - $E = 3,600\text{ kip/in}^2$, unit weight $\gamma = 0.15\text{ kip/ft}^3$.
  - $A = 144\text{ in}^2$ (Modifier $= 1,000 \Rightarrow A_{\text{eff}} = 144,000\text{ in}^2$), $I_{33} = 1,728\text{ in}^4$, $A_v = 0$ (shear deformations ignored).
- **Load Cases (7 Independent Linear Static Cases)**:
  1. **LC 1**: Self-weight ($0.15\text{ kip/ft}$ downward $-Z$ on all 3 elements).
  2. **LC 2**: Global $-Z$ uniform load of $1.8\text{ kip/ft}$ on Element 3, plus $-10\text{ kip}$ ($-Z$) point load at Joint 4.
  3. **LC 3**: Global joint force $F_z = -17.2\text{ kip}$ and moment $M_y = -54.4\text{ kip-ft}$ at Joint 2 (statically equivalent to LC 2).
  4. **LC 4**: Global $-Z$ projected uniform load of $2.0\text{ kip/ft}$ on Element 2 (total vertical load $= 2.0 \times 8\text{ ft} = 16\text{ kip}$).
  5. **LC 5**: Uniform distributed load of $2.0\text{ kip/ft}$ in local $-2$ direction on Elements 1 and 2.
  6. **LC 6**: Trapezoidal distributed load in local $-2$ direction: Element 1 varies from $0.9984\text{ kip/ft}$ at Joint 1 to $0.3744\text{ kip/ft}$ at Joint 2; Element 2 varies from $0.3744\text{ kip/ft}$ at Joint 2 to $0\text{ kip/ft}$ at Joint 3.
  7. **LC 7**: Concentrated midspan load of $15\text{ kip}$ in local $-2$ direction on Element 2 at $x_h = 4\text{ ft}$ ($s = 0.5 L$).
- **Expected Reference Outputs**:

| Load Case | Output Parameter | Analytical Formula ($\Delta = \int \frac{M m}{EI} dx$) | Target Value (`SAP2000` & `Independent`) |
| :---: | :--- | :--- | ---: |
| **1** | $U_z(\text{Joint 3})$ ($\text{in}$) | $-95\text{ kip}\cdot\text{ft}^3 / EI$ | **`-0.02639`** |
| **2** | $U_z(\text{Joint 3})$ ($\text{in}$) | $+680 / (3 EI)\text{ kip}\cdot\text{ft}^3$ | **`0.06296`** |
| **3** | $U_z(\text{Joint 3})$ ($\text{in}$) | $+680 / (3 EI)\text{ kip}\cdot\text{ft}^3$ | **`0.06296`** |
| **4** | $U_z(\text{Joint 3})$ ($\text{in}$) | $-3200 / (3 EI)\text{ kip}\cdot\text{ft}^3$ | **`-0.29630`** |
| **5** | $U_x(\text{Joint 2})$ ($\text{in}$) | $+1125 / EI\text{ kip}\cdot\text{ft}^3$ | **`0.31250`** |
| **6** | $U_x(\text{Joint 2})$ ($\text{in}$) | $+416 / EI\text{ kip}\cdot\text{ft}^3$ | **`0.11556`** |
| **7** | $U_x(\text{Joint 2})$ ($\text{in}$) | $+23.4375 / EI\text{ kip}\cdot\text{ft}^3$ | **`0.00651`** |

---

### Benchmark 2: Example 1-002 — Frame Temperature Loading (`example_1_002a.sdb`, `example_1_002b.sdb`)
- **Features Tested**: Uniform temperature change $\Delta T$, linear temperature variation along local 1-axis (via Joint Pattern), and transverse thermal gradient $\Delta T_2$ along local 2-axis.
- **Geometry & Properties (units: $\text{kip}, \text{in}, ^\circ\text{F}$)**:
  - Horizontal beam along $X$-axis from Joint 1 $(0,0,0)$ to Joint 2 $(10\text{ in}, 0, 0)$, length $L = 10\text{ in}$.
  - Rectangular section: width $b = 2\text{ in}$, depth $d = 3\text{ in}$ ($A = 6\text{ in}^2$, $I_{33} = 4.5\text{ in}^4$, $A_v = 0$).
  - Material: $E = 29,000\text{ kip/in}^2$, thermal expansion coefficient $\alpha = 6.5 \times 10^{-6}\text{ /^\circ F}$.
  - **Model A (Free Cantilever)**: Joint 1 fixed ($U_x = U_z = R_y = 0$), Joint 2 free.
  - **Model B (Propped Cantilever)**: Joint 1 fixed ($U_x = U_z = R_y = 0$), Joint 2 restrained in $X$ and $Z$ ($U_x = U_z = 0, R_y$ free).
- **Load Cases**:
  - **LC 1**: Uniform temperature increase $\Delta T = +20^\circ\text{F}$.
  - **LC 2**: Axial temperature variation along $X$ of $2^\circ\text{F/in}$ from $0^\circ\text{F}$ at $x=0$ to $+20^\circ\text{F}$ at $x=10\text{ in}$.
  - **LC 3**: Transverse temperature gradient along local 2-axis of $+20^\circ\text{F/in}$ ($60^\circ\text{F}$ over $h = 3\text{ in}$ depth).
- **Expected Reference Outputs**:

| Model | Load Case | Output Parameter | Analytical Formula | Target Value |
| :---: | :---: | :--- | :--- | ---: |
| **Model A** | **1** | $U_x(\text{free end})$ ($\text{in}$) | $\alpha \Delta T L = 6.5\times 10^{-6} \times 20 \times 10$ | **`0.00130`** |
| **Model A** | **2** | $U_x(\text{free end})$ ($\text{in}$) | $\int_0^{10} 2\alpha x\,dx = 100\alpha$ | **`0.00065`** |
| **Model A** | **3** | $U_z(\text{free end})$ ($\text{in}$) | $-\frac{\alpha L^2 (T_2 - T_1)}{2 h} = -\frac{6.5\times 10^{-6} \times 10^2 \times 60}{2 \times 3}$ | **`-0.00650`** |
| **Model B** | **1** | $F_x(\text{at prop})$ ($\text{kip}$) | $-EA \Delta_{\text{A1}} / L$ | **`-22.62`** |
| **Model B** | **2** | $F_x(\text{at prop})$ ($\text{kip}$) | $-EA \Delta_{\text{A2}} / L$ | **`-11.31`** |
| **Model B** | **3** | $F_z(\text{at prop})$ ($\text{kip}$) | $+\frac{3 EI \alpha (T_2 - T_1)}{2 h L}$ | **`2.545`** |

---

### Benchmark 3: Example 1-003 — Frame Distributed and Concentrated Torsional Moments (`example_1_003.sdb`)
- **Features Tested**: Uniform and triangular distributed torsional moments and point torsional moments on frame elements.
- **Geometry & Properties (units: $\text{kip}, \text{in}, \text{rad}$)**:
  - 4 collinear frame elements along $X$-axis (total length $10\text{ in}$):
    - Elem 1: Jt 1 $(0)$ to Jt 2 $(5\text{ in})$, $L_1 = 5\text{ in}$.
    - Elem 2: Jt 2 $(5\text{ in})$ to Jt 3 $(7\text{ in})$, $L_2 = 2\text{ in}$.
    - Elem 3: Jt 3 $(7\text{ in})$ to Jt 4 $(8\text{ in})$, $L_3 = 1\text{ in}$.
    - Elem 4: Jt 4 $(8\text{ in})$ to Jt 5 $(10\text{ in})$, $L_4 = 2\text{ in}$.
  - Joint 1 fixed ($R_x = 0$), Joints 2–5 free to rotate about $X$.
  - Circular shaft ($d = 1\text{ in}$): $E = 28,990\text{ kip/in}^2, \nu = 0.3, G = 11,150\text{ kip/in}^2, J = 0.09817\text{ in}^4$ ($GJ = 1,094.5955\text{ kip}\cdot\text{in}^2$).
- **Load Cases**:
  - **LC 1**: Uniform distributed torsion $m_x = 1.0\text{ kip-in/in}$ on Elem 2 + concentrated torque $T = 2.0\text{ kip-in}$ at Joint 4 end of Elem 3.
  - **LC 2**: Triangular distributed torsion on Elem 1 increasing linearly from $0$ at Jt 1 to $1.5\text{ kip-in/in}$ at Jt 2 + concentrated torque $T = 2.0\text{ kip-in}$ at Joint 5 end of Elem 4.
- **Expected Reference Outputs**:

| Load Case | Output Parameter | Analytical Formula ($\theta = \int \frac{T(x)}{GJ} dx$) | Target Value |
| :---: | :--- | :--- | ---: |
| **1** | $M_x(\text{Joint 1})$ ($\text{kip-in}$) | $-(1.0 \times 2 + 2.0)$ | **`-4.0`** |
| **1** | $R_x(\text{Joint 3})$ ($\text{rad}$) | $26 / GJ$ | **`0.02375`** |
| **1** | $R_x(\text{Joint 5})$ ($\text{rad}$) | $28 / GJ$ | **`0.02558`** |
| **2** | $M_x(\text{Joint 1})$ ($\text{kip-in}$) | $-(\frac{1}{2} \times 5 \times 1.5 + 2.0)$ | **`-5.75`** |
| **2** | $R_x(\text{Joint 3})$ ($\text{rad}$) | $26.5 / GJ$ | **`0.02421`** |
| **2** | $R_x(\text{Joint 5})$ ($\text{rad}$) | $32.5 / GJ$ | **`0.02969`** |

---

### Benchmark 4: Example 1-004 — Frame with Rotated Local Axes (`example_1_004.sdb`)
- **Features Tested**: 3D transformation of frame stiffness matrix when local axes `(2, 3)` are rotated by $\beta = 30^\circ$ from global axes.
- **Geometry & Properties (units: $\text{kip}, \text{in}$)**:
  - Cantilever along $X$-axis of length $L = 144\text{ in}$ (Joint 1 fixed at $x=0$, Joint 2 free at $x=144\text{ in}$).
  - Section `W12X106`: $I_{33} = 933\text{ in}^4$, $I_{22} = 301\text{ in}^4$, $A_v = 0$ (shear deformations ignored).
  - Material: $E = 29,000\text{ kip/in}^2$. Local axis `2` rotated $\beta = 30^\circ$ from $+Z$ toward $-Y$.
- **Load Cases**:
  - **LC 1**: Uniform distributed load $w_z = -0.01\text{ kip/in}$ in global $Z$ direction.
  - **LC 2**: Concentrated load $P_z = -1.0\text{ kip}$ at free end in global $Z$ direction.
  - **LC 3**: Concentrated moment $M_z = +240\text{ kip-in}$ at free end about global $Z$-axis.
- **Expected Reference Outputs**:

| Load Case | Output Parameter | Analytical Decomposition ($\Delta_2, \Delta_3 \rightarrow \Delta_y, \Delta_z$) | Target Value |
| :---: | :--- | :--- | ---: |
| **1** | $U_y(\text{free end})$ ($\text{in}$) | $-\Delta_2 \sin 30^\circ - \Delta_3 \cos 30^\circ$ | **`-0.01806`** |
| **1** | $U_z(\text{free end})$ ($\text{in}$) | $-\Delta_2 \cos 30^\circ - \Delta_3 \sin 30^\circ$ | **`-0.03029`** |
| **2** | $U_y(\text{free end})$ ($\text{in}$) | $\Delta_2 = -0.031858\text{ in}, \Delta_3 = +0.057013\text{ in}$ | **`-0.03345`** |
| **2** | $U_z(\text{free end})$ ($\text{in}$) | $\Delta_z = \Delta_2 \cos 30^\circ - \Delta_3 \sin 30^\circ$ | **`-0.05610`** |
| **3** | $U_y(\text{free end})$ ($\text{in}$) | $\Delta_2 = -0.079645\text{ in}, \Delta_3 = -0.142532\text{ in}$ | **`-0.08361`** |
| **3** | $U_z(\text{free end})$ ($\text{in}$) | $\Delta_z = \Delta_2 \cos 30^\circ + \Delta_3 \sin 30^\circ$ | **`-0.14024`** |

---

### Benchmark 5: Example 1-005 — Portal Frame with Support Settlement, Rotation, Springs & Skewed Supports (`example_1_005a..f.sdb`)
- **Features Tested**: Prescribed support settlements and rotations, grounded spring settlements/rotations, skewed joint local axes (`cos θ = 0.8, sin θ = 0.6`).
- **Geometry & Properties (units: $\text{kip}, \text{in}$)**:
  - Portal frame in $XZ$ plane ($L = 144\text{ in}$ wide, $H = 144\text{ in}$ high):
    - Joint 1 $(0,0,0)$ Fixed; Joint 2 $(0,0,144)$; Joint 3 $(144,0,144)$; Joint 4 $(144,0,0)$ variable support.
    - Elem 1: Jt 1 $\rightarrow$ Jt 2; Elem 2: Jt 2 $\rightarrow$ Jt 3; Elem 3: Jt 4 $\rightarrow$ Jt 3.
  - $E = 29,000\text{ kip/in}^2$, $b=12\text{ in}, d=12\text{ in}$, $A = 144\text{ in}^2$ (Modifier $= 100,000$), $I_{33} = 1,728\text{ in}^4$, $A_v = 0$.
- **Models & Expected Reference Outputs**:

| Model | Joint 4 Support & Loading | Output Parameter | Target Value |
| :---: | :--- | :--- | ---: |
| **A** | Vertical Roller ($U_z=0$); Load: $U_z(\text{Jt 4}) = -0.5\text{ in}$ | $F_z(\text{Jt 1})$ ($\text{kip}$) / $M_y(\text{Jt 1})$ ($\text{kip-in}$) | **`6.293`** / **`-906.250`** |
| **B** | Vertical Spring $k_z = 10\text{ kip/in}$; Load: Ground $U_z(\text{Jt 4}) = -0.2\text{ in}$ | $F_z(\text{Jt 1})$ ($\text{kip}$) / $M_y(\text{Jt 1})$ ($\text{kip-in}$) | **`1.115`** / **`-160.492`** |
| **C** | Vertical Roller ($U_z=0$); Load: $R_y(\text{Jt 1}) = +0.01\text{ rad}$ | $F_z(\text{Jt 1})$ ($\text{kip}$) / $M_y(\text{Jt 1})$ ($\text{kip-in}$) | **`-18.125`** / **`2610.000`** |
| **D** | Fixed $U_x=U_z=0$ + Rotational Spring $k_{\theta y} = 80,000\text{ kip-in/rad}$; Load: Ground $R_y(\text{Jt 4}) = +0.01\text{ rad}$ | $M_y(\text{Jt 1})$ ($\text{kip-in}$) / $R_y(\text{Jt 4})$ ($\text{rad}$) | **`-473.469`** / **`0.00408`** |
| **E** | Skewed Roller ($U_3=0$, $\cos\theta=0.8, \sin\theta=0.6$); Load: $-10\text{ kip}$ ($Z$) at midspan of Elem 2 | $F_z(\text{Jt 1})$ ($\text{kip}$) / $F_3(\text{Jt 4})$ ($\text{kip}$) | **`5.811`** / **`5.236`** |
| **F** | Skewed Roller ($U_3=0$, $\cos\theta=0.8, \sin\theta=0.6$); Load: $U_3(\text{Jt 4}) = -1.0\text{ in}$ | $F_z(\text{Jt 1})$ ($\text{kip}$) / $M_y(\text{Jt 1})$ ($\text{kip-in}$) | **`27.215`** / **`-3918.919`** |

---

### Benchmark 6: Example 1-006 — Non-Prismatic Frame Sections and Automatic Subdivision (`example_1_006a..b.sdb`)
- **Features Tested**: Multi-segment non-prismatic frame property (`cPropFrameVar`) with linear $A(x), J(x)$ and linear, parabolic ($n=2$), and cubic ($n=3$) $I_{33}(x), I_{22}(x)$ interpolation.
- **Geometry & Properties (units: $\text{kip}, \text{in}$)**:
  - Cantilever along $X$-axis of total length $L = 144\text{ in}$ (Joint 1 fixed at $x=0$, Joint 2 free at $x=144\text{ in}$), $A_v = 0$.
  - Material: $E = 3,600\text{ kip/in}^2, \nu = 0.2, G = 1,500\text{ kip/in}^2, \gamma = 0.15\text{ kip/ft}^3$.
  - Base Sections (`FSec1`..`FSec4`, width $b=12\text{ in}$):
    - `FSec1` ($12\times 12$): $A=144, I_{33}=1728, I_{22}=1728, J=2920.32$.
    - `FSec2` ($12\times 18$): $A=216, I_{33}=5832, I_{22}=2592, J=6085.12$.
    - `FSec3` ($12\times 24$): $A=288, I_{33}=13824, I_{22}=3456, J=9492.12$.
    - `FSec4` ($12\times 30$): $A=360, I_{33}=27000, I_{22}=4320, J=12934.73$.
  - 4 Segments (each $36\text{ in}$ long):
    - Seg 1 ($0..36\text{ in}$): `FSec4` $\rightarrow$ `FSec3` ($A, I_{33}, I_{22}, J$ all Linear).
    - Seg 2 ($36..72\text{ in}$): `FSec2` $\rightarrow$ `FSec2` (Constant).
    - Seg 3 ($72..108\text{ in}$): `FSec3` $\rightarrow$ `FSec1` ($A, J$ Linear; $I_{33}, I_{22}$ Parabolic $n=2$).
    - Seg 4 ($108..144\text{ in}$): `FSec1` $\rightarrow$ `FSec2` ($A, J$ Linear; $I_{33}, I_{22}$ Cubic $n=3$).
- **Expected Reference Outputs (Identical for Model A [1 elem] and Model B [10 subdivided elems])**:

| Load Case | Applied Load at Free End | Output Parameter | Target Value |
| :---: | :--- | :--- | ---: |
| **1** | Self-weight ($-Z$) | $M_y(\text{fixed end})$ ($\text{kip-in}$) | **`-184.950`** |
| **2** | $F_x = +500\text{ kip}$ | $U_x(\text{free end})$ ($\text{in}$) | **`0.09087`** |
| **3** | $F_y = +5\text{ kip}$ | $U_y(\text{free end})$ ($\text{in}$) | **`0.43120`** |
| **4** | $F_z = -5\text{ kip}$ | $U_z(\text{free end})$ ($\text{in}$) | **`-0.14335`** |
| **5** | $M_x = +5,000\text{ kip-in}$ | $R_x(\text{free end})$ ($\text{rad}$) | **`0.07987`** |
| **6** | $M_y = +5,000\text{ kip-in}$ | $R_y(\text{free end})$ ($\text{rad}$) | **`0.03742`** |
| **7** | $M_z = +5,000\text{ kip-in}$ | $R_z(\text{free end})$ ($\text{rad}$) | **`0.07634`** |

---

### Benchmark 7: Example 1-007 — Frame End Releases (`example_1_007a..c.sdb`)
- **Features Tested**: Static condensation of axial ($P$), shear ($V_2$), and bending ($M_3$) end releases.
- **Geometry & Loading (units: $\text{kip}, \text{ft}$)**:
  - 2-element L-frame in $XZ$ plane: Column from Jt 1 $(0,0,0)$ [Fixed] to Jt 2 $(0,0,10\text{ ft})$; Beam from Jt 2 $(0,0,10\text{ ft})$ to Jt 3 $(10\text{ ft},0,10\text{ ft})$ [Pinned roller $U_x=U_z=0$].
  - Vertical point load $P_z = -10\text{ kip}$ at midpoint of Beam ($x = 5\text{ ft}$).
- **Expected Reference Outputs**:
  - **Model A** (Shear $V_2$ release at Jt 2 end of Beam): $F_z(\text{Jt 1}) = \mathbf{0\text{ kip}}$, $M_y(\text{Jt 1}) = \mathbf{50\text{ kip-ft}}$.
  - **Model B** (Axial $P$ release at Jt 2 end of Column): $F_z(\text{Jt 1}) = \mathbf{0\text{ kip}}$, $M_y(\text{Jt 1}) = \mathbf{50\text{ kip-ft}}$.
  - **Model C** (Moment $M_3$ release at Jt 2): $F_z(\text{Jt 1}) = \mathbf{5\text{ kip}}$, $M_y(\text{Jt 1}) = \mathbf{0\text{ kip-ft}}$.

---

### Benchmark 8: Example 1-008 — Frame Partial Fixity End Releases (`example_1_008.sdb`)
- **Features Tested**: Partial fixity end release springs ($k_M, k_V$) combined with Timoshenko beam bending + shear deformation.
- **Geometry & Properties (units: $\text{kip}, \text{in}$)**:
  - Cantilever of length $L = 24\text{ ft} = 288\text{ in}$.
  - Material: $E = 4,320\text{ kip/in}^2, \nu = 0.2, G = 1,800\text{ kip/in}^2, \gamma = 0.15\text{ kip/ft}^3$.
  - Section ($18\text{ in} \times 30\text{ in}$): $A = 540\text{ in}^2, I_{33} = 40,500\text{ in}^4, A_{v2} = 450\text{ in}^2$.
  - Partial fixity at fixed support: Moment spring $k_M = 3,888,000\text{ kip-in/rad}$, Shear spring $k_V = 540\text{ kip/in}$.
  - Loading: Gravity load $= 2 \times \text{self-weight} = 0.09375\text{ kip/in}$ ($-Z$).
- **Expected Reference Output**:
  - Tip vertical displacement: $\Delta_{\text{total}} = -(\Delta_{\text{flex}} + \Delta_{\text{shear,beam}} + \Delta_{\text{spring},V} + \Delta_{\text{spring},M}) = -(0.4608 + 0.0048 + 0.0500 + 0.2880) = \mathbf{-0.8036\text{ in}}$ (`0%` diff).

---

### Benchmark 9: Example 1-009 — Prestress Tendon Applied to Frame (`example_1_009a..c.sdb`)
- **Features Tested**: Parabolic prestressing tendon profile with asymmetric end eccentricities ($d_i = +10\text{ in}, d_c = -10\text{ in}, d_j = +3\text{ in}$), curvature friction ($\mu = 0.15$), wobble friction ($k = 0.0001\text{ /in}$), and elastic shortening losses, modeled both as equivalent loads (Model A) and internal tendon elements (Models B & C).
- **Geometry & Properties (units: $\text{kip}, \text{in}$)**:
  - Simply supported concrete beam of span $L = 360\text{ in}$ ($18\text{ in} \times 30\text{ in}$, $E = 3,600\text{ ksi}, G = 1,500\text{ ksi}, A = 540\text{ in}^2, I = 40,500\text{ in}^4, A_v = 450\text{ in}^2$).
  - Tendon: $T_0 = 220\text{ kip}$ (jacked at left end), $A_{ps} = 1.5\text{ in}^2, E_{ps} = 29,000\text{ ksi}$.
- **Expected Reference Outputs**:
  - Midspan upward camber $U_z(\text{center})$: **`0.17028 in`** (Model A - Loads), **`0.16041 in`** (Model B - 60" elems), **`0.16484 in`** (Model C - 12" elems), vs. Hand calc **`0.16564 in`**.
  - Midspan bending moment $M_y(\text{center})$: **`-2061.5 kip-in`** (Model A), **`-2013.9 kip-in`** (Model B), **`-2001.7 kip-in`** (Model C), vs. Hand calc **`-2004.3 kip-in`**.

---

### Benchmark 10: Example 1-010 — Frame End Offsets & Rigid Zone Factor (`example_1_010a..d.sdb`)
- **Features Tested**: Frame end offset $a = 6\text{ in}$ at the support with varying rigid-zone factor $r \in \{0.0, 0.5, 1.0\}$.
- **Geometry & Properties (units: $\text{kip}, \text{in}$)**:
  - Cantilever of total length $L = 144\text{ in}$ with end load $P = -10\text{ kip}$ ($-Z$).
  - Material: $E = 4,320\text{ ksi}, G = 1,800\text{ ksi}$. Section ($12\text{ in} \times 18\text{ in}$): $A = 216\text{ in}^2, I = 5,832\text{ in}^4, A_v = 180\text{ in}^2$.
  - Flexible length $L_f = L - r a = 144 - 6r\text{ in}$; tip deflection formula: $\Delta_z = -\left(\frac{P L_f^3}{3 EI} + \frac{P L_f}{G A_v}\right)$.
- **Expected Reference Outputs**:

| Model | Offset $a$ ($\text{in}$) | Rigid Zone Factor $r$ | Flexible Length $L_f$ ($\text{in}$) | Target $U_z(\text{free end})$ ($\text{in}$) |
| :---: | :---: | :---: | :---: | ---: |
| **A** | $0$ | N.A. | $144$ | **`-0.39951`** |
| **B** | $6$ | $0.0$ | $144$ | **`-0.39951`** |
| **C** | $6$ | $0.5$ | $141$ | **`-0.37523`** |
| **D** | $6$ | $1.0$ | $138$ | **`-0.35197`** |

---

### Benchmark 11: Example 1-014 — Cantilever Beam Eigenvalue Modal Analysis (`example_1_014a..g.sdb`)
- **Features Tested**: Eigenvalue solver (`Eigen_SpacemSBR`), lumped mass matrix generation, biaxial flexural vibration modes ($I_{22} \neq I_{33}$).
- **Geometry & Properties (units: $\text{kip}, \text{in}, \text{sec}$)**:
  - Cantilever beam of length $L = 96\text{ in}$, $E = 3,600\text{ ksi}$, mass density $\rho = 2.3 \times 10^{-7}\text{ kip}\cdot\text{sec}^2/\text{in}^4$.
  - Section ($12\text{ in} \times 18\text{ in}$): $A = 216\text{ in}^2, I_Y = 5,832\text{ in}^4, I_Z = 2,592\text{ in}^4, A_v = 0$.
- **Expected Reference Outputs (First 5 Modes)**:

| Mode | Bending Axis | Analytical Period $T = \frac{2\pi}{\lambda_n^2}\sqrt{\frac{\rho A L^4}{EI}}$ | SAP2000 (4 Elems) | SAP2000 (10 Elems) | SAP2000 (96 Elems) |
| :---: | :---: | ---: | ---: | ---: | ---: |
| **1** | 1st Mode about $Z$ ($\lambda_1 = 1.875$) | **`0.038005 sec`** | `0.039090` | `0.038175` | **`0.038003`** |
| **2** | 1st Mode about $Y$ ($\lambda_1 = 1.875$) | **`0.025337 sec`** | `0.026060` | `0.025450` | **`0.025335`** |
| **3** | 2nd Mode about $Z$ ($\lambda_2 = 4.694$) | **`0.006064 sec`** | `0.006651` | `0.006160` | **`0.006065`** |
| **4** | 2nd Mode about $Y$ ($\lambda_2 = 4.694$) | **`0.004043 sec`** | `0.004434` | `0.004107` | **`0.004043`** |
| **5** | 3rd Mode about $Z$ ($\lambda_3 = 7.855$) | **`0.002165 sec`** | `0.002511` | `0.002222` | **`0.002166`** |

---

### Benchmark 12: Example 1-016 — Tension Stiffening Using P-Delta Analysis (`example_1_016a..g.sdb`)
- **Features Tested**: Frame geometric stiffness matrix $K_G(P)$ under axial tension ($P = +20.25\text{ kip}$), tested both via direct P-Delta force assignment and via Nonlinear Static P-Delta preload followed by Linear Static transverse loading.
- **Geometry & Properties (units: $\text{kip}, \text{in}$)**:
  - Simply supported tie-rod of total span $L = 300\text{ in}$ (modeled with two $150\text{ in}$ frame objects meeting at Joint 2).
  - $E = 30,000\text{ ksi}$, section $3\text{ in} \times 3\text{ in}$ ($I = 6.75\text{ in}^4, A_v = 0$).
  - Transverse uniform load $w = -0.002\text{ kip/in}$ ($-Z$); axial tension $S = 20.25\text{ kip}$ ($u = \sqrt{S L^2 / (4 EI)} = 1.5$).
- **Expected Reference Outputs**:
  - **Without P-Delta (Model A)**: $U_z(\text{mid}) = -\frac{5 w L^4}{384 EI} = \mathbf{-1.04167\text{ in}}$, $M_y(\text{mid}) = \frac{w L^2}{8} = \mathbf{22.500\text{ kip-in}}$.
  - **With P-Delta Tension Stiffening**:
    - 1 Elem/half-span (Models B, E): $U_z(\text{mid}) = \mathbf{-0.54555\text{ in}}$, $M_y(\text{mid}) = \mathbf{11.453\text{ kip-in}}$.
    - 2 Elems/half-span (Models C, F): $U_z(\text{mid}) = \mathbf{-0.54343\text{ in}}$, $M_y(\text{mid}) = \mathbf{11.495\text{ kip-in}}$.
    - 16 Elems/half-span (Models D, G): $U_z(\text{mid}) = \mathbf{-0.54330\text{ in}}$, $M_y(\text{mid}) = \mathbf{11.498\text{ kip-in}}$ (Exact match to Timoshenko analytical solution).

---

### Benchmark 13: Example 1-019 — Linear Buckling of a Rigid Portal Frame (`example_1_019a..c.sdb`)
- **Features Tested**: Eigenvalue buckling solver (`BuckLin_SpacemSBR`) on a 2D portal frame in the $XZ$ plane.
- **Geometry & Properties (units: $\text{kip}, \text{in}$)**:
  - One-story, one-bay portal frame ($L = 144\text{ in}$, $h = 144\text{ in}$), pinned at base of both columns, vertical unit loads $P = -1\text{ kip}$ at top of each column.
  - $E = 29,900\text{ ksi}$, section `W8X31` ($A = 9.12\text{ in}^2$ with modifier $100,000$; $I = 110\text{ in}^4$; $A_v = 0$).
  - Transcendental sidesway buckling equation: $kl \tan(kl) = 6 \Rightarrow kl = 1.34955282 \Rightarrow P_{cr} = (kl)^2 EI / h^2 = \mathbf{280.19\text{ kips}}$.
- **Expected Reference Outputs**:
  - Model A (1 elem/member): **`280.98 kips`** (`+0.28%`).
  - Model B (2 elems/member): **`280.24 kips`** (`+0.02%`).
  - Model C (4 elems/member): **`280.19 kips`** (`0.00%`).

---

### Benchmark 14: Example 2-001 — MacNeal-Harder Shell Patch Test (`example_2_001-thin.sdb`, `example_2_001-thick.sdb`)
- **Features Tested**: Constant-strain membrane patch test and constant-curvature plate bending patch test on 5 skewed quadrilateral shell elements (both `Thin` Kirchhoff and `Thick` Mindlin-Reissner formulations).
- **Geometry & Properties (units: $\text{lb}, \text{in}$)**:
  - Rectangular domain $0.24\text{ in} \times 0.12\text{ in}$ in $XY$ plane ($Z=0$), thickness $t = 0.001\text{ in}$, $E = 10^6\text{ psi}, \nu = 0.25$.
  - Exterior nodes: Jt 1 $(0,0)$, Jt 2 $(0, 0.12)$, Jt 7 $(0.24, 0)$, Jt 8 $(0.24, 0.12)$.
  - Interior skewed nodes: Jt 3 $(0.04, 0.02)$, Jt 4 $(0.08, 0.08)$, Jt 5 $(0.18, 0.03)$, Jt 6 $(0.16, 0.08)$.
- **Prescribed Edge Displacements (at Jts 1, 2, 7, 8)**:
  - **Load Case `Membrane`**: $U_x = 10^{-3}(x + y/2)$, $U_y = 10^{-3}(y + x/2)$.
  - **Load Case `PlBend`**: $U_z = 10^{-3}(x^2 + xy + y^2)/2$, $R_x = \partial U_z / \partial y = 10^{-3}(y + x/2)$, $R_y = -\partial U_z / \partial x = 10^{-3}(-x - y/2)$.
- **Expected Reference Outputs (Exact `0%` error across all 5 elements for both `Thin` and `Thick` shells)**:
  - **Membrane Case**: $\sigma_{xx} = \mathbf{1333.33\text{ psi}}$, $\sigma_{yy} = \mathbf{1333.33\text{ psi}}$, $\sigma_{xy} = \mathbf{400.00\text{ psi}}$.
  - **Plate Bending Case**: $M_{xx} = \mathbf{1.111 \times 10^{-7}\text{ lb-in/in}}$, $M_{yy} = \mathbf{1.111 \times 10^{-7}\text{ lb-in/in}}$, $M_{xy} = \mathbf{3.333 \times 10^{-8}\text{ lb-in/in}}$.

---

### Benchmark 15: Example 2-005 — Rectangular Plate Bending with Static Loads (`example_2_005a..i.sdb`)
- **Features Tested**: Thin (Kirchhoff) and Thick (Mindlin-Reissner) shell plate bending across aspect ratios ($1:1$ and $5:1$), boundary conditions (Simply Supported vs. Clamped/Fixed), and loading (Uniform Pressure vs. Center Point Load).
- **Geometry & Properties (units: $\text{lb}, \text{in}$)**:
  - Quarter-symmetry model of $2\text{ in} \times 2\text{ in}$ ($1:1$) or $2\text{ in} \times 10\text{ in}$ ($5:1$) plate, meshed $4 \times 4$ and $16 \times 16$.
  - $E = 17,472,000\text{ psi}, \nu = 0.3, G = 6,720,000\text{ psi}, t = 0.0001\text{ in} \Rightarrow D = \frac{E t^3}{12(1-\nu^2)} = 1.6 \times 10^{-6}\text{ lb-in}$.
  - Loads: Uniform pressure $q = 0.0001\text{ psi}$, or Center point load $P = 0.0004\text{ lb}$ ($0.0001\text{ lb}$ on quarter-plate center node).
- **Expected Reference Center Deflections $U_z$ ($\text{in}$) ($16 \times 16$ Mesh)**:

| Plate Aspect | Edge Support | Load Type | Analytical Target ($\text{in}$) | SAP2000 Thin ($16\times 16$) | SAP2000 Thick ($16\times 16$) |
| :---: | :---: | :--- | ---: | ---: | ---: |
| **$2"\times 2"$ ($1:1$)** | Simple | Uniform ($q$) | **`4.06`** | `4.06` (`0%`) | `4.06` (`0%`) |
| **$2"\times 2"$ ($1:1$)** | Fixed | Uniform ($q$) | **`1.26`** | `1.27` (`+0.8%`) | `1.27` (`+0.8%`) |
| **$2"\times 10"$ ($5:1$)** | Simple | Uniform ($q$) | **`12.97`** | `12.96` (`-0.1%`) | `12.97` (`0%`) |
| **$2"\times 10"$ ($5:1$)** | Fixed | Uniform ($q$) | **`2.60`** | `2.60` (`0%`) | `2.60` (`0%`) |
| **$2"\times 2"$ ($1:1$)** | Simple | Point ($P$) | **`11.60`** | `11.63` (`+0.3%`) | `11.61` (`+0.1%`) |
| **$2"\times 2"$ ($1:1$)** | Fixed | Point ($P$) | **`5.60`** | `5.64` (`+0.7%`) | `5.61` (`+0.2%`) |
| **$2"\times 10"$ ($5:1$)** | Simple | Point ($P$) | **`16.95`** | `17.15` (`+1.2%`) | `16.92` (`-0.2%`) |
| **$2"\times 10"$ ($5:1$)** | Fixed | Point ($P$) | **`7.25`** | `7.41` (`+2.2%`) | `7.11` (`-1.9%`) |

---

### Summary Catalog of All 86 SAP2000 Verification Benchmarks (`sap2000_verification.xls`)

For complete regression coverage, the extracted BIFF8 spreadsheet dump (`/home/edo/REA/docs/re/raw_solver/xls_dump.txt`) contains every reference value for all 7 verification classes:
- **Class 1 — Frame (29 Problems, `1-001` to `1-029`)**: General loading (`1-001`), Thermal (`1-002`), Torsion (`1-003`), Rotated axes (`1-004`), Support settlement/skew/springs (`1-005`), Non-prismatic (`1-006`), End releases (`1-007`), Partial fixity (`1-008`), Prestress (`1-009`), End offsets (`1-010`), Insertion point (`1-011`), Tension/compression limits (`1-012`), Winkler foundation (`1-013`), Cantilever eigenvalues (`1-014`), Steady-state harmonics (`1-015`), P-Delta tension stiffening (`1-016`), Vibrating string (`1-017`), Portal frame deformations (`1-018`), Frame buckling (`1-019`), 2D/3D Response spectrum (`1-020`, `1-024`, `1-025`), Bathe-Wilson & ASME eigenvalues (`1-021`, `1-023`), Dynamic time-history (`1-022`), Plastic hinges (`1-026`), Staged construction (`1-027`), Large displacements / elastica (`1-028`, `1-029`).
- **Class 2 — Shell (20 Problems, `2-001` to `2-020`)**: Patch test (`2-001`), Straight/curved/twisted MacNeal-Harder beams (`2-002`..`2-004`), Rectangular plates (`2-005`), Scordelis-Lo barrel vault roof (`2-006`), Hemispherical shell (`2-007`), Cantilever plate modes (`2-008`), Plate on elastic foundation (`2-009`), Pressurized cylinder (`2-010`), ASME cooling tower (`2-011`), Mindlin thick-plate shear (`2-012`), Shell thermal membrane & gradient (`2-013`, `2-014`), Orthotropic plate (`2-015`), Shell buckling (`2-016`, `2-017`), Large displacements (`2-018`, `2-019`), Prestressed shell (`2-020`).
- **Class 3 — Plane (5 Problems, `3-001` to `3-005`)**: Patch test (`3-001`), Straight/curved beams (`3-002`, `3-003`), Thick-walled Lamé cylinder (`3-004`), Pore pressure (`3-005`).
- **Class 4 — Asolid (4 Problems, `4-001` to `4-004`)**: Circular footing on half-space (`4-001`), Thick-walled cylinder (`4-002`), Rotating disk (`4-003`), Pore pressure (`4-004`).
- **Class 5 — Solid (13 Problems, `5-001` to `5-013`)**: 3D Patch test (`5-001`), MacNeal-Harder beams/plates/shells with 8-node bricks & incompatible modes (`5-002`..`5-007`), Thick-walled cylinder near incompressibility $\nu = 0.4999$ (`5-008`), Prestressed solid (`5-009`), Solid buckling (`5-010`), Thermal (`5-011`), Elastic foundation (`5-012`), Pore pressure (`5-013`).
- **Class 6 — Link (12 Problems, `6-001` to `6-012`)**: Linear ramp (`6-001`), Multi-linear elastic (`6-002`), Gap (`6-003`), Hook (`6-004`), Linear/Nonlinear velocity dampers (`6-005`..`6-007`), Wen plasticity (`6-008`), Kinematic plasticity (`6-009`), Base-isolated 8-story rubber & 7-story friction-pendulum buildings (`6-010`, `6-011`), Frequency-dependent links (`6-012`).
- **Class 7 — Cable (3 Problems, `7-001` to `7-003`)**: Suspended catenary cable under uniform/thermal/concentrated loads (`7-001`, `7-002`) and prestressed 3D cable net (`7-003`).
