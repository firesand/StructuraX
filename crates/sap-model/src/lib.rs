pub mod templates;
pub mod editing;
pub use editing::{EditError, EditSummary};

use indexmap::IndexMap;
use sap_units::Units;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// Select a free numeric object name, including after objects have been deleted.
pub fn next_numeric_name<'a>(names: impl IntoIterator<Item = &'a String>) -> String {
    let occupied: std::collections::BTreeSet<_> = names.into_iter().map(String::as_str).collect();
    (1usize..)
        .map(|n| n.to_string())
        .find(|name| !occupied.contains(name.as_str()))
        .expect("a finite model has an unused numeric name")
}

/// Exact match for `SAP2000v19.eMatType` (1..=7).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[repr(i32)]
pub enum MatType {
    #[default]
    Steel = 1,
    Concrete = 2,
    NoDesign = 3,
    Aluminum = 4,
    ColdFormed = 5,
    Rebar = 6,
    Tendon = 7,
}

/// Exact match for `SAP2000v19.eFramePropType` (1..=34).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[repr(i32)]
pub enum FramePropType {
    #[default]
    I = 1,
    Channel = 2,
    T = 3,
    Angle = 4,
    DblAngle = 5,
    Box = 6,
    Pipe = 7,
    Rectangular = 8,
    Circle = 9,
    General = 10,
    DbChannel = 11,
    Auto = 12,
    SD = 13,
    Variable = 14,
    Joist = 15,
    Bridge = 16,
    ColdC = 17,
    Cold2C = 18,
    ColdZ = 19,
    ColdL = 20,
    Cold2L = 21,
    ColdHat = 22,
    BuiltUpICoverplate = 23,
    PCCGirderI = 24,
    PCCGirderU = 25,
    BuiltUpIHybrid = 26,
    BuiltUpUHybrid = 27,
    ConcreteL = 28,
    ColdBox = 29,
    ColdI = 30,
    ColdPipe = 31,
    ColdT = 32,
    Trapezoidal = 33,
    PCCGirderSuperT = 34,
}

/// Exact match for `SAP2000v19.eAreaPropType` (1..=3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[repr(i32)]
pub enum AreaPropType {
    #[default]
    Shell = 1,
    Plane = 2,
    ASolid = 3,
}

/// Exact match for `SAP2000v19.eShellType` (1..=6).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[repr(i32)]
pub enum ShellType {
    #[default]
    ShellThin = 1,
    ShellThick = 2,
    PlateThin = 3,
    PlateThick = 4,
    Membrane = 5,
    ShellLayered = 6,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum PlaneType {
    #[default]
    PlaneStress,
    PlaneStrain,
}

/// Exact match for `SAP2000v19.eLinkPropType` (1..=13).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[repr(i32)]
pub enum LinkPropType {
    #[default]
    Linear = 1,
    Damper = 2,
    Gap = 3,
    Hook = 4,
    PlasticWen = 5,
    RubberIsolator = 6,
    FrictionIsolator = 7,
    MultiLinearElastic = 8,
    MultiLinearPlastic = 9,
    TCFrictionIsolator = 10,
    DamperBilinear = 11,
    DamperFrictionSpring = 12,
    TriplePendulumIsolator = 13,
}

/// Exact match for `SAP2000v19.eLoadPatternType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[repr(i32)]
pub enum LoadPatternType {
    #[default]
    Dead = 1,
    SuperDead = 2,
    Live = 3,
    ReduceLive = 4,
    Quake = 5,
    Wind = 6,
    Snow = 7,
    Other = 8,
    Move = 9,
    Temperature = 10,
    Rooflive = 11,
    Notional = 12,
    PatternLive = 13,
    Wave = 14,
    Prestress = 34,
}

/// Exact match for `SAP2000v19.eLoadCaseType` (1..=15).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[repr(i32)]
pub enum LoadCaseType {
    #[default]
    LinearStatic = 1,
    NonlinearStatic = 2,
    Modal = 3,
    ResponseSpectrum = 4,
    LinearHistory = 5,
    NonlinearHistory = 6,
    LinearDynamic = 7,
    NonlinearDynamic = 8,
    MovingLoad = 9,
    Buckling = 10,
    SteadyState = 11,
    PowerSpectralDensity = 12,
    LinearStaticMultiStep = 13,
    HyperStatic = 14,
}

/// Exact match for `SAP2000v19.eConstraintType` (1..=9).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[repr(i32)]
pub enum ConstraintType {
    #[default]
    Body = 1,
    Diaphragm = 2,
    Plate = 3,
    Rod = 4,
    Beam = 5,
    Equal = 6,
    Local = 7,
    Weld = 8,
    Line = 9,
}

/// Exact match for `SAP2000v19.eConstraintAxis` (1..=4).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[repr(i32)]
pub enum ConstraintAxis {
    X = 1,
    Y = 2,
    #[default]
    Z = 3,
    AutoAxis = 4,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum FrameType2D {
    #[default]
    PortalFrame = 0,
    ConcentricBraced = 1,
    EccentricBraced = 2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum FrameType3D {
    #[default]
    OpenFrame = 0,
    PerimeterFrame = 1,
    BeamSlab = 2,
    FlatPlate = 3,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GridLine {
    pub axis: String, // "X", "Y", "Z"
    pub id: String,
    pub coordinate: f64,
    pub visible: bool,
    pub bubble_loc: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CoordSystem {
    pub name: String,
    pub origin: [f64; 3],
    pub angles_deg: [f64; 3], // RZ, RY, RX
    pub grids: Vec<GridLine>,
}

impl Default for CoordSystem {
    fn default() -> Self {
        Self {
            name: "GLOBAL".to_string(),
            origin: [0.0, 0.0, 0.0],
            angles_deg: [0.0, 0.0, 0.0],
            grids: Vec::new(),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Material {
    pub name: String,
    pub mat_type: MatType,
    /// Young's modulus $E_1$ (and $[E_1, E_2, E_3]$ for orthotropic)
    pub e: f64,
    pub e_ortho: Option<[f64; 3]>,
    /// Poisson's ratio $\nu_{12}$ (and $[\nu_{12}, \nu_{13}, \nu_{23}]$ for orthotropic)
    pub u: f64,
    pub u_ortho: Option<[f64; 3]>,
    /// Shear modulus $G_{12}$ (and $[G_{12}, G_{13}, G_{23}]$ for orthotropic)
    pub g: f64,
    pub g_ortho: Option<[f64; 3]>,
    /// Coefficient of thermal expansion $\alpha_1$ (and $[\alpha_1, \alpha_2, \alpha_3]$)
    pub a: f64,
    pub a_ortho: Option<[f64; 3]>,
    /// Weight per unit volume $\gamma$
    pub unit_weight: f64,
    /// Mass per unit volume $\rho$
    pub unit_mass: f64,
    /// Steel/Rebar minimum yield stress $F_y$
    pub fy: f64,
    /// Steel/Rebar minimum tensile strength $F_u$
    pub fu: f64,
    /// Concrete specified compressive strength $f'_c$
    pub fc: f64,
    /// Shear rebar yield stress $F_{ys}$
    pub fys: f64,
    pub color: String,
}

impl Material {
    pub fn default_steel_kip_in() -> Self {
        let e = 29_000.0;
        let u = 0.3;
        let g = e / (2.0 * (1.0 + u));
        let unit_weight = 0.490 / 1728.0; // kip/in^3
        let unit_mass = unit_weight / 386.08858267716536;
        Self {
            name: "A992Fy50".to_string(),
            mat_type: MatType::Steel,
            e,
            e_ortho: None,
            u,
            u_ortho: None,
            g,
            g_ortho: None,
            a: 6.5e-6,
            a_ortho: None,
            unit_weight,
            unit_mass,
            fy: 50.0,
            fu: 65.0,
            fc: 4.0,
            fys: 50.0,
            color: "Blue".to_string(),
        }
    }

    pub fn default_concrete_kip_in() -> Self {
        let e = 3_600.0;
        let u = 0.2;
        let g = e / (2.0 * (1.0 + u));
        let unit_weight = 0.150 / 1728.0; // kip/in^3
        let unit_mass = unit_weight / 386.08858267716536;
        Self {
            name: "4000Psi".to_string(),
            mat_type: MatType::Concrete,
            e,
            e_ortho: None,
            u,
            u_ortho: None,
            g,
            g_ortho: None,
            a: 5.5e-6,
            a_ortho: None,
            unit_weight,
            unit_mass,
            fy: 60.0,
            fu: 90.0,
            fc: 4.0,
            fys: 60.0,
            color: "Green".to_string(),
        }
    }
}

/// Non-prismatic segment specification (`cRecPropFrameVar`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NonPrismaticSegment {
    pub start_sec: String,
    pub end_sec: String,
    /// Segment length (either absolute or relative depending on `is_variable_length`)
    pub length: f64,
    pub is_variable_length: bool,
    /// EI33 variation exponent: 1=Linear, 2=Parabolic, 3=Cubic
    pub ei33_variation: i32,
    /// EI22 variation exponent: 1=Linear, 2=Parabolic, 3=Cubic
    pub ei22_variation: i32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FrameSection {
    pub name: String,
    pub material: String,
    pub prop_type: FramePropType,
    /// Depth $t_3$ ($d$) along local 2-axis
    pub t3: f64,
    /// Flange width $t_2$ ($b_f$) along local 3-axis
    pub t2: f64,
    /// Top flange thickness $t_f$
    pub tf: f64,
    /// Web thickness $t_w$
    pub tw: f64,
    /// Bottom flange width $t_{2b}$ (for asymmetric I or T)
    pub t2b: f64,
    /// Bottom flange thickness $t_{fb}$
    pub tfb: f64,
    /// Back-to-back distance `dis` for Double Angle / Double Channel
    pub dis: f64,
    /// Cross-sectional area $A$
    pub area: f64,
    /// Torsional constant $J$
    pub torsional_j: f64,
    /// Moment of inertia $I_{33}$ about local 3-axis (major bending in 1-2 plane)
    pub i33: f64,
    /// Moment of inertia $I_{22}$ about local 2-axis (minor bending in 1-3 plane)
    pub i22: f64,
    /// Product of inertia $I_{23}$
    pub i23: f64,
    /// Shear area $A_{s2}$ along local 2-axis (`0.0` disables shear deformation in 1-2 plane)
    pub as2: f64,
    /// Shear area $A_{s3}$ along local 3-axis (`0.0` disables shear deformation in 1-3 plane)
    pub as3: f64,
    /// Elastic section modulus $S_{33}$
    pub s33: f64,
    /// Elastic section modulus $S_{22}$
    pub s22: f64,
    /// Plastic section modulus $Z_{33}$
    pub z33: f64,
    /// Plastic section modulus $Z_{22}$
    pub z22: f64,
    /// Radius of gyration $r_{33}$
    pub r33: f64,
    /// Radius of gyration $r_{22}$
    pub r22: f64,
    /// Property modifiers `[Area, As2, As3, J, I22, I33, Mass, Weight]`
    pub modifiers: [f64; 8],
    /// Non-prismatic segments when `prop_type == FramePropType::Variable`
    pub non_prismatic: Vec<NonPrismaticSegment>,
    /// Auto-select section list when `prop_type == FramePropType::Auto`
    pub auto_select_list: Vec<String>,
    /// Concrete reinforcement details (cover, rebar area) if concrete section
    pub concrete_rebar: Option<ConcreteRebarConfig>,
    /// Source `.pro` file and name in file if imported
    pub file_name: Option<String>,
    pub name_in_file: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ConcreteRebarConfig {
    pub is_column: bool,
    pub rebar_mat_long: String,
    pub rebar_mat_confine: String,
    pub cover_top: f64,
    pub cover_bot: f64,
    pub num_bars_3dir: usize,
    pub num_bars_2dir: usize,
    pub total_rebar_area: f64,
}

impl Default for FrameSection {
    fn default() -> Self {
        Self {
            name: "FSEC1".to_string(),
            material: "A992Fy50".to_string(),
            prop_type: FramePropType::I,
            t3: 18.0,
            t2: 7.5,
            tf: 0.57,
            tw: 0.355,
            t2b: 7.5,
            tfb: 0.57,
            dis: 0.0,
            area: 14.7,
            torsional_j: 1.24,
            i33: 800.0,
            i22: 40.1,
            i23: 0.0,
            as2: 6.39,
            as3: 7.125,
            s33: 88.9,
            s22: 10.7,
            z33: 101.0,
            z22: 16.6,
            r33: 7.38,
            r22: 1.65,
            modifiers: [1.0; 8],
            non_prismatic: Vec::new(),
            auto_select_list: Vec::new(),
            concrete_rebar: None,
            file_name: None,
            name_in_file: None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AreaSection {
    pub name: String,
    pub material: String,
    pub prop_type: AreaPropType,
    pub shell_type: ShellType,
    pub plane_type: PlaneType,
    pub mat_angle_deg: f64,
    /// Membrane thickness $t_m$ (or subtended angle in radians for ASolid)
    pub thickness_membrane: f64,
    /// Bending thickness $t_b$
    pub thickness_bending: f64,
    /// Include incompatible bending modes (for Plane / ASolid / Membrane)
    pub incompatible_modes: bool,
    /// Include drilling DOF stiffness (for Shell)
    pub drilling_dof: bool,
    /// Modifiers `[f11, f22, f12, m11, m22, m12, v13, v23, Mass, Weight]`
    pub modifiers: [f64; 10],
}

impl Default for AreaSection {
    fn default() -> Self {
        Self {
            name: "ASEC1".to_string(),
            material: "4000Psi".to_string(),
            prop_type: AreaPropType::Shell,
            shell_type: ShellType::ShellThin,
            plane_type: PlaneType::PlaneStress,
            mat_angle_deg: 0.0,
            thickness_membrane: 6.0,
            thickness_bending: 6.0,
            incompatible_modes: true,
            drilling_dof: true,
            modifiers: [1.0; 10],
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SolidSection {
    pub name: String,
    pub material: String,
    pub mat_angle_deg: [f64; 3],
    pub incompatible_modes: bool,
    pub modifiers: [f64; 8],
}

impl Default for SolidSection {
    fn default() -> Self {
        Self {
            name: "SSEC1".to_string(),
            material: "4000Psi".to_string(),
            mat_angle_deg: [0.0; 3],
            incompatible_modes: true,
            modifiers: [1.0; 8],
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LinkProp {
    pub name: String,
    pub link_type: LinkPropType,
    /// Active DOFs `[U1, U2, U3, R1, R2, R3]`
    pub dof_active: [bool; 6],
    /// Fixed DOFs `[U1, U2, U3, R1, R2, R3]`
    pub dof_fixed: [bool; 6],
    /// Per-DOF nonlinear law activation; false uses effective linear stiffness.
    #[serde(default)]
    pub dof_nonlinear: [bool; 6],
    /// False when imported nonlinear activation/constitutive data is unmapped.
    #[serde(default = "default_true")]
    pub nonlinear_data_complete: bool,
    /// Effective linear stiffness `[k_u1, k_u2, k_u3, k_r1, k_r2, k_r3]`
    pub ke: [f64; 6],
    /// Effective viscous damping `[c_u1, c_u2, c_u3, c_r1, c_r2, c_r3]`
    pub ce: [f64; 6],
    /// Nonlinear stiffness `[k_nl1..6]` (for Gap/Hook/PlasticWen)
    pub k_nl: [f64; 6],
    /// Yield force `[fy1..6]` (for PlasticWen/Isolator) or initial opening `[open1..6]` (for Gap/Hook)
    pub param1: [f64; 6],
    /// Post-yield stiffness ratio `[r1..6]`
    pub param2: [f64; 6],
    /// Shear distance from J-end `[dj2, dj3]`
    pub dj2: f64,
    pub dj3: f64,
    pub mass: f64,
    pub weight: f64,
}

fn default_true() -> bool { true }

impl Default for LinkProp {
    fn default() -> Self {
        Self {
            name: "LIN1".to_string(),
            link_type: LinkPropType::Linear,
            dof_active: [true, false, false, false, false, false],
            dof_fixed: [false; 6],
            dof_nonlinear: [false; 6],
            nonlinear_data_complete: true,
            ke: [1000.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ce: [0.0; 6],
            k_nl: [0.0; 6],
            param1: [0.0; 6],
            param2: [0.0; 6],
            dj2: 0.0,
            dj3: 0.0,
            mass: 0.0,
            weight: 0.0,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PointLoadForce {
    pub load_pat: String,
    pub csys: String, // "GLOBAL" or "LOCAL"
    /// `[F1, F2, F3, M1, M2, M3]`
    pub force: [f64; 6],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PointLoadDispl {
    pub load_pat: String,
    pub csys: String,
    /// `[U1, U2, U3, R1, R2, R3]`
    pub displ: [f64; 6],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PointObj {
    pub name: String,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    /// Restraints `[U1, U2, U3, R1, R2, R3]` in joint local coordinate system
    pub restraint: [bool; 6],
    /// Uncoupled spring stiffness `[k_u1, k_u2, k_u3, k_r1, k_r2, k_r3]`
    pub spring: [f64; 6],
    /// Coupled 6x6 upper-triangular spring stiffness (21 terms: U1U1, U1U2..R3R3) if coupled
    pub spring_coupled: Option<[f64; 21]>,
    pub spring_is_local: bool,
    /// Additional lumped joint mass `[m_u1, m_u2, m_u3, m_r1, m_r2, m_r3]`
    pub mass: [f64; 6],
    /// Local axes Euler angles `(A, B, C)` in degrees (`Z -> Y' -> X''` or 3D local orientation matrix)
    pub local_axes_deg: [f64; 3],
    /// Optional explicit $3 \times 3$ local-to-global direction cosine matrix (columns are $\mathbf{e}_1, \mathbf{e}_2, \mathbf{e}_3$)
    pub local_axes_matrix: Option<[[f64; 3]; 3]>,
    /// Assigned constraint names
    pub constraints: Vec<String>,
    /// Joint pattern values (`pattern_name -> value`)
    pub pattern_values: BTreeMap<String, f64>,
    pub loads_force: Vec<PointLoadForce>,
    pub loads_displ: Vec<PointLoadDispl>,
    pub special_point: bool,
    pub selected: bool,
}

impl PointObj {
    pub fn new(name: impl Into<String>, x: f64, y: f64, z: f64) -> Self {
        Self {
            name: name.into(),
            x,
            y,
            z,
            restraint: [false; 6],
            spring: [0.0; 6],
            spring_coupled: None,
            spring_is_local: false,
            mass: [0.0; 6],
            local_axes_deg: [0.0; 3],
            local_axes_matrix: None,
            constraints: Vec::new(),
            pattern_values: BTreeMap::new(),
            loads_force: Vec::new(),
            loads_displ: Vec::new(),
            special_point: false,
            selected: false,
        }
    }
}

/// Distributed frame load (`SetLoadDistributed`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FrameLoadDistributed {
    pub load_pat: String,
    /// `1` = Force/length, `2` = Moment/length
    pub my_type: i32,
    /// `1..3` = Local 1..3, `4..6` = Global X, Y, Z, `7..9` = Projected X, Y, Z, `10` = Gravity (-Z), `11` = Projected Gravity
    pub dir: i32,
    pub csys: String,
    /// Relative start and end distances $s_1, s_2 \in [0, 1]$
    pub rel_dist1: f64,
    pub rel_dist2: f64,
    pub val1: f64,
    pub val2: f64,
}

/// Point span load on a frame element (`SetLoadPoint`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FrameLoadPoint {
    pub load_pat: String,
    /// `1` = Force, `2` = Moment
    pub my_type: i32,
    /// `1..3` = Local 1..3, `4..6` = Global X, Y, Z, `7..9` = Projected X, Y, Z, `10` = Gravity (-Z), `11` = Projected Gravity
    pub dir: i32,
    pub csys: String,
    /// Relative distance $s \in [0, 1]$ from I-end
    pub rel_dist: f64,
    pub val: f64,
}

/// Temperature load on a frame element (`SetLoadTemperature`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FrameLoadTemperature {
    pub load_pat: String,
    /// `1` = Uniform Temp Change $\Delta T$, `2` = Temp Gradient along local 2-axis $\Delta T_2$ (per unit depth), `3` = Temp Gradient along local 3-axis $\Delta T_3$
    pub my_type: i32,
    pub val: f64,
    /// Optional Joint Pattern name multiplying `val` at joints $I$ and $J$
    pub pattern_name: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FrameObj {
    pub name: String,
    pub point_i: String,
    pub point_j: String,
    pub section: String,
    /// Local axis rotation angle $\beta$ in degrees
    pub angle_deg: f64,
    /// Property modifiers `[Area, As2, As3, J, I22, I33, Mass, Weight]`
    pub modifiers: [f64; 8],
    /// End releases at I-end `[U1, U2, U3, R1, R2, R3]` (`P, V2, V3, T, M2, M3`)
    pub release_i: [bool; 6],
    /// End releases at J-end `[U1, U2, U3, R1, R2, R3]`
    pub release_j: [bool; 6],
    /// Partial-fixity spring stiffness at I-end when `release_i[k]` is true (`0.0` = full release)
    pub start_spring: [f64; 6],
    /// Partial-fixity spring stiffness at J-end when `release_j[k]` is true (`0.0` = full release)
    pub end_spring: [f64; 6],
    /// End length offsets `[offset_i, offset_j]` and rigid-zone factor `rz` $\in [0, 1]$
    pub end_offset_i: f64,
    pub end_offset_j: f64,
    pub rigid_zone_factor: f64,
    /// Cardinal point (`1..11`, `10` = Centroid) and end joint offsets in local/global coordinates
    pub cardinal_point: i32,
    pub joint_offset_i: [f64; 3],
    pub joint_offset_j: [f64; 3],
    pub joint_offset_is_global: bool,
    pub stiff_transform: bool,
    /// Tension/compression limits `(has_comp_limit, comp_limit, has_tens_limit, tens_limit)`
    pub tc_limits: Option<(bool, f64, bool, f64)>,
    /// Initial P-Delta axial force (or target cable force)
    pub pdelta_force: f64,
    /// Is this a catenary cable element? (If `Some(undeformed_length_or_sag)`)
    pub cable_params: Option<CableParams>,
    /// Minimum number of output stations along element
    pub min_stations: usize,
    pub loads_dist: Vec<FrameLoadDistributed>,
    pub loads_point: Vec<FrameLoadPoint>,
    pub loads_temp: Vec<FrameLoadTemperature>,
    /// Gravity load multiplier `[gx, gy, gz]` per load pattern (`load_pat -> [gx, gy, gz]`)
    pub loads_gravity: BTreeMap<String, [f64; 3]>,
    pub selected: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CableParams {
    pub undeformed_length: Option<f64>,
    pub target_tension_i: Option<f64>,
    pub horizontal_tension: Option<f64>,
    pub num_segments: usize,
}

impl FrameObj {
    pub fn new(
        name: impl Into<String>,
        point_i: impl Into<String>,
        point_j: impl Into<String>,
        section: impl Into<String>,
    ) -> Self {
        Self {
            name: name.into(),
            point_i: point_i.into(),
            point_j: point_j.into(),
            section: section.into(),
            angle_deg: 0.0,
            modifiers: [1.0; 8],
            release_i: [false; 6],
            release_j: [false; 6],
            start_spring: [0.0; 6],
            end_spring: [0.0; 6],
            end_offset_i: 0.0,
            end_offset_j: 0.0,
            rigid_zone_factor: 0.0,
            cardinal_point: 10,
            joint_offset_i: [0.0; 3],
            joint_offset_j: [0.0; 3],
            joint_offset_is_global: false,
            stiff_transform: true,
            tc_limits: None,
            pdelta_force: 0.0,
            cable_params: None,
            min_stations: 9,
            loads_dist: Vec::new(),
            loads_point: Vec::new(),
            loads_temp: Vec::new(),
            loads_gravity: BTreeMap::new(),
            selected: false,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AreaLoadUniform {
    pub load_pat: String,
    pub csys: String,
    /// `1..3` = Local 1..3, `4..6` = Global X..Z, `7..9` = Projected X..Z, `10` = Gravity (-Z), `11` = Projected Gravity
    pub dir: i32,
    pub val: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AreaLoadSurfacePressure {
    pub load_pat: String,
    /// `-1` = Bottom, `-2` = Top, `1..4` = Edge 1..4
    pub face: i32,
    pub val: f64,
    pub pattern_name: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AreaLoadTemperature {
    pub load_pat: String,
    /// `1` = Uniform $\Delta T$, `2` = Through-thickness gradient $\Delta T_3$
    pub my_type: i32,
    pub val: f64,
    pub pattern_name: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AreaObj {
    pub name: String,
    /// 3 or 4 (or up to 9 for higher-order plane/asolid) joint names in counter-clockwise order
    pub points: Vec<String>,
    pub section: String,
    pub angle_deg: f64,
    pub modifiers: [f64; 10],
    /// Auto-mesh divisions `(n1, n2)` (`(1, 1)` = unmeshed single element)
    pub auto_mesh: (usize, usize),
    pub edge_constraint: bool,
    pub loads_uniform: Vec<AreaLoadUniform>,
    pub loads_pressure: Vec<AreaLoadSurfacePressure>,
    pub loads_temp: Vec<AreaLoadTemperature>,
    pub selected: bool,
}

impl AreaObj {
    pub fn new(name: impl Into<String>, points: Vec<String>, section: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            points,
            section: section.into(),
            angle_deg: 0.0,
            modifiers: [1.0; 10],
            auto_mesh: (1, 1),
            edge_constraint: false,
            loads_uniform: Vec::new(),
            loads_pressure: Vec::new(),
            loads_temp: Vec::new(),
            selected: false,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SolidSurfaceLoad {
    pub load_pat: String,
    pub face: i32, // 1..6
    pub val: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SolidObj {
    pub name: String,
    /// 8 joint names `[p1..p8]`
    pub points: [String; 8],
    pub section: String,
    pub loads_surface: Vec<SolidSurfaceLoad>,
    pub loads_temp: BTreeMap<String, f64>,
    pub selected: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LinkObj {
    pub name: String,
    pub point_i: String,
    /// `None` if 1-joint grounded link
    pub point_j: Option<String>,
    pub prop: String,
    pub angle_deg: f64,
    pub selected: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ConstraintDef {
    pub name: String,
    pub constraint_type: ConstraintType,
    pub axis: ConstraintAxis,
    /// Active constrained DOFs `[UX, UY, UZ, RX, RY, RZ]`
    pub dofs: [bool; 6],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LoadPattern {
    pub name: String,
    pub pat_type: LoadPatternType,
    pub self_wt_multiplier: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LoadCaseAssignment {
    pub load_name: String, // LoadPattern name or Accel UX/UY/UZ
    pub scale_factor: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum ModalSubType {
    #[default]
    Eigen,
    Ritz,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum ModalComboMethod {
    #[default]
    Cqc,
    Srss,
    AbsSum,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum NLGeomType {
    #[default]
    None,
    PDelta,
    LargeDisplacements,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum TimeIntegrationMethod {
    #[default]
    NewmarkAverage,
    NewmarkLinear,
    HilberHughesTaylor,
    WilsonTheta,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LoadCase {
    pub name: String,
    pub case_type: LoadCaseType,
    /// Initial stiffness case name (`None` = unstressed zero initial conditions, or P-Delta case)
    pub initial_case: Option<String>,
    pub loads: Vec<LoadCaseAssignment>,
    /// Modal parameters
    pub modal_sub_type: ModalSubType,
    pub max_modes: usize,
    pub min_modes: usize,
    pub shift_freq_hz: f64,
    /// Nonlinear static parameters
    pub nl_geom: NLGeomType,
    pub max_iterations: usize,
    pub convergence_tol: f64,
    /// Response spectrum parameters: `(function_name, dir "U1"/"U2"/"U3", scale_factor)`
    pub spectrum_loads: Vec<(String, String, f64)>,
    pub modal_combo: ModalComboMethod,
    pub modal_damping: f64,
    /// Time-history parameters: `(function_name, load_pat_or_accel, scale_factor)`
    pub history_loads: Vec<(String, String, f64)>,
    pub num_time_steps: usize,
    pub time_step_dt: f64,
    pub time_integration: TimeIntegrationMethod,
    pub hht_alpha: f64,
    #[serde(default = "default_wilson_theta")]
    pub wilson_theta: f64,
    pub rayleigh_damping: (f64, f64), // (alpha_mass, beta_stiff)
    /// Buckling parameters
    pub num_buckling_modes: usize,
    pub run_flag: bool,
}

impl LoadCase {
    pub fn linear_static(name: impl Into<String>, pat: impl Into<String>, sf: f64) -> Self {
        let name_s = name.into();
        let pat_s = pat.into();
        Self {
            name: name_s,
            case_type: LoadCaseType::LinearStatic,
            initial_case: None,
            loads: vec![LoadCaseAssignment {
                load_name: pat_s,
                scale_factor: sf,
            }],
            modal_sub_type: ModalSubType::Eigen,
            max_modes: 12,
            min_modes: 1,
            shift_freq_hz: 0.0,
            nl_geom: NLGeomType::None,
            max_iterations: 50,
            convergence_tol: 1e-6,
            spectrum_loads: Vec::new(),
            modal_combo: ModalComboMethod::Cqc,
            modal_damping: 0.05,
            history_loads: Vec::new(),
            num_time_steps: 200,
            time_step_dt: 0.02,
            time_integration: TimeIntegrationMethod::NewmarkAverage,
            hht_alpha: 0.0,
            wilson_theta: default_wilson_theta(),
            rayleigh_damping: (0.0, 0.0),
            num_buckling_modes: 6,
            run_flag: true,
        }
    }

    pub fn modal_eigen(name: impl Into<String>, max_modes: usize) -> Self {
        Self {
            name: name.into(),
            case_type: LoadCaseType::Modal,
            initial_case: None,
            loads: Vec::new(),
            modal_sub_type: ModalSubType::Eigen,
            max_modes,
            min_modes: 1,
            shift_freq_hz: 0.0,
            nl_geom: NLGeomType::None,
            max_iterations: 100,
            convergence_tol: 1e-9,
            spectrum_loads: Vec::new(),
            modal_combo: ModalComboMethod::Cqc,
            modal_damping: 0.05,
            history_loads: Vec::new(),
            num_time_steps: 0,
            time_step_dt: 0.01,
            time_integration: TimeIntegrationMethod::NewmarkAverage,
            hht_alpha: 0.0,
            wilson_theta: default_wilson_theta(),
            rayleigh_damping: (0.0, 0.0),
            num_buckling_modes: 6,
            run_flag: true,
        }
    }
}

fn default_wilson_theta() -> f64 {
    1.4
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum ComboType {
    #[default]
    LinearAdd = 0,
    Envelope = 1,
    AbsAdd = 2,
    Srss = 3,
    RangeAdd = 4,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LoadCombo {
    pub name: String,
    pub combo_type: ComboType,
    /// `(case_or_combo_name, scale_factor)`
    pub cases: Vec<(String, f64)>,
}

/// Response spectrum or Time-history function definition `(x, y)` pairs.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DynamicFunction {
    pub name: String,
    pub is_time_history: bool,
    pub damping_ratio: f64,
    pub points: Vec<(f64, f64)>,
}

// ============================================================================
// ANALYSIS & DESIGN RESULTS
// ============================================================================

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct FrameStationResult {
    /// Distance from I-end in current length units
    pub station: f64,
    /// Relative distance $s \in [0, 1]$
    pub rel_dist: f64,
    /// Axial force $P$ (`+` = tension, `-` = compression, matching SAP2000 convention!)
    pub p: f64,
    /// Major shear $V_2$ along local 2-axis
    pub v2: f64,
    /// Minor shear $V_3$ along local 3-axis
    pub v3: f64,
    /// Torsional moment $T$ about local 1-axis
    pub t: f64,
    /// Minor bending moment $M_2$ about local 2-axis (in 1-3 plane)
    pub m2: f64,
    /// Major bending moment $M_3$ about local 3-axis (in 1-2 plane)
    pub m3: f64,
    /// Local transverse deflection $[u_1, u_2, u_3]$
    pub local_disp: [f64; 3],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct AreaResult {
    /// Membrane forces per unit width $[F_{11}, F_{22}, F_{12}, F_{\max}, F_{\min}]$
    pub f11: f64,
    pub f22: f64,
    pub f12: f64,
    /// Bending moments per unit width $[M_{11}, M_{22}, M_{12}]$
    pub m11: f64,
    pub m22: f64,
    pub m12: f64,
    /// Transverse shear forces per unit width $[V_{13}, V_{23}]$
    pub v13: f64,
    pub v23: f64,
    /// Top/Bottom stresses $[S_{11}, S_{22}, S_{12}, S_{\text{VM}}]$
    pub s11_top: f64,
    pub s22_top: f64,
    pub s12_top: f64,
    pub svm_top: f64,
    /// Nodal forces at each corner joint in global coordinates `[FX, FY, FZ, MX, MY, MZ]`
    pub joint_forces: Vec<(String, [f64; 6])>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct SolidStress {
    pub s11: f64,
    pub s22: f64,
    pub s33: f64,
    pub s12: f64,
    pub s13: f64,
    pub s23: f64,
    pub s_max: f64,
    pub s_mid: f64,
    pub s_min: f64,
    pub von_mises: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct SolidResult {
    /// Stress evaluated at the element natural origin $\boldsymbol{\xi} = (0, 0, 0)$.
    pub center_stress: SolidStress,
    /// Stresses evaluated at the 8 Gauss integration points ($\xi_k = \pm 1/\sqrt{3}$).
    pub gauss_stresses: [SolidStress; 8],
    /// Stresses extrapolated from the 8 Gauss points to the 8 corner nodes.
    pub nodal_stresses: [SolidStress; 8],
    /// Internal elastic nodal forces (`[FX, FY, FZ]` in global coordinates) at each of the 8 nodes.
    pub nodal_forces: [[f64; 3]; 8],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct ModalModeResult {
    pub mode_num: usize,
    pub period_sec: f64,
    pub freq_hz: f64,
    pub circ_freq_rad_s: f64,
    pub eigenvalue: f64,
    /// Modal participation factors `[UX, UY, UZ, RX, RY, RZ]`
    pub participation_factors: [f64; 6],
    /// Participating modal mass ratios `[UX, UY, UZ, RX, RY, RZ]`
    pub modal_mass_ratios: [f64; 6],
    /// Mode shape joint displacements `joint_name -> [U1, U2, U3, R1, R2, R3]`
    pub mode_shape: IndexMap<String, [f64; 6]>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct NonlinearLinkForceState {
    pub link: String,
    pub dof: usize,
    pub force: f64,
}

/// Persisted terminal state of a direct-integration time-history case for exact continuation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct TransientTerminalState {
    pub point_order: Vec<String>,
    pub u: Vec<f64>,
    pub v: Vec<f64>,
    pub a: Vec<f64>,
    pub previous_force: Vec<f64>,
    pub nonlinear_link_forces: Vec<NonlinearLinkForceState>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct CaseResults {
    pub case_name: String,
    /// Joint displacements `joint_name -> [U1, U2, U3, R1, R2, R3]`
    pub joint_displacements: IndexMap<String, [f64; 6]>,
    /// Support/Spring reactions `joint_name -> [F1, F2, F3, M1, M2, M3]`
    pub joint_reactions: IndexMap<String, [f64; 6]>,
    /// Frame internal forces & deflections along stations `frame_name -> Vec<FrameStationResult>`
    pub frame_forces: IndexMap<String, Vec<FrameStationResult>>,
    /// Area/Shell internal forces & stresses `area_name -> AreaResult`
    pub area_results: IndexMap<String, AreaResult>,
    /// 3D Solid internal stresses & nodal forces `solid_name -> SolidResult`
    #[serde(default)]
    pub solid_results: IndexMap<String, SolidResult>,
    /// Link forces `link_name -> [P, V2, V3, T, M2, M3]`
    pub link_forces: IndexMap<String, [f64; 6]>,
    /// Modal results (populated for Modal cases)
    pub modes: Vec<ModalModeResult>,
    /// Buckling load factors $\lambda_{cr}$ (populated for Buckling cases)
    pub buckling_factors: Vec<f64>,
    /// Time-history step traces `time_steps` and `joint_name -> Vec<[f64; 6]>`
    pub time_steps: Vec<f64>,
    pub time_history_disp: IndexMap<String, Vec<[f64; 6]>>,
    pub base_reactions: [f64; 6],
    /// Terminal dynamic state for chained `initial_case` continuation
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub transient_state: Option<TransientTerminalState>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SteelDesignResult {
    pub frame_name: String,
    pub section_name: String,
    pub design_code: String,
    pub governing_combo: String,
    pub governing_station: f64,
    /// Governing demand/capacity ratio: maximum of PMM interaction and both shear checks.
    pub total_ratio: f64,
    pub pr_ratio: f64,
    pub mr_major_ratio: f64,
    pub mr_minor_ratio: f64,
    pub shear_major_ratio: f64,
    #[serde(default)]
    pub shear_minor_ratio: f64,
    /// Available strengths: factored capacities for LRFD or allowable capacities for ASD.
    pub phi_pnc: f64,
    pub phi_pnt: f64,
    pub phi_mn33: f64,
    pub phi_mn22: f64,
    pub phi_vn2: f64,
    #[serde(default)]
    pub phi_vn3: f64,
    pub status: String, // "OK", "OVERSTRESSED", "SLENDER"
    pub eq_name: String, // e.g. "H1-1a", "H1-1b", "EC3-6.2"
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ConcreteDesignResult {
    pub frame_name: String,
    pub section_name: String,
    pub design_code: String,
    pub governing_combo: String,
    pub is_column: bool,
    /// Top longitudinal rebar area required $A_{s,\text{top}}$
    pub as_top: f64,
    /// Bottom longitudinal rebar area required $A_{s,\text{bot}}$
    pub as_bot: f64,
    /// Shear stirrup area per unit spacing $A_v / s$
    pub av_over_s: f64,
    /// Column PMM interaction capacity ratio (if column)
    pub pmm_ratio: f64,
    pub min_rebar_area: f64,
    pub status: String,
}

/// Original interchange text and its supported canonical projection. This is
/// preservation evidence, not a model semantic/physics implementation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct S2kPreservation {
    pub source_text: String,
    pub canonical_baseline: String,
}

/// Central SAP2000 Structural Model (`cSapModel`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SapModel {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub s2k_preservation: Option<S2kPreservation>,
    pub model_name: String,
    pub program_version: String,
    pub present_units: Units,
    pub database_units: Units,
    /// Active global degrees of freedom `[UX, UY, UZ, RX, RY, RZ]`
    pub active_dofs: [bool; 6],
    pub steel_design_code: String,
    pub concrete_design_code: String,
    pub project_info: IndexMap<String, String>,

    pub coord_systems: IndexMap<String, CoordSystem>,
    pub materials: IndexMap<String, Material>,
    pub frame_sections: IndexMap<String, FrameSection>,
    pub area_sections: IndexMap<String, AreaSection>,
    pub solid_sections: IndexMap<String, SolidSection>,
    pub link_props: IndexMap<String, LinkProp>,
    pub constraints: IndexMap<String, ConstraintDef>,

    pub points: IndexMap<String, PointObj>,
    pub frames: IndexMap<String, FrameObj>,
    pub areas: IndexMap<String, AreaObj>,
    pub solids: IndexMap<String, SolidObj>,
    pub links: IndexMap<String, LinkObj>,

    pub functions: IndexMap<String, DynamicFunction>,
    pub load_patterns: IndexMap<String, LoadPattern>,
    pub load_cases: IndexMap<String, LoadCase>,
    pub load_combos: IndexMap<String, LoadCombo>,

    pub is_locked: bool,
    pub case_results: IndexMap<String, CaseResults>,
    pub steel_design_results: IndexMap<String, SteelDesignResult>,
    pub concrete_design_results: IndexMap<String, ConcreteDesignResult>,
}

impl Default for SapModel {
    fn default() -> Self {
        Self::new(Units::KipInF)
    }
}

impl SapModel {
    pub fn new(units: Units) -> Self {
        let mut coord_systems = IndexMap::new();
        coord_systems.insert("GLOBAL".to_string(), CoordSystem::default());

        let mut materials = IndexMap::new();
        let mut steel = Material::default_steel_kip_in();
        let mut conc = Material::default_concrete_kip_in();
        // Scale default materials from KipInF to requested `units`
        let f_stress = Units::KipInF.stress_factor_to(units);
        let f_uw = Units::KipInF.unit_weight_factor_to(units);
        let f_um = Units::KipInF.mass_density_factor_to(units);
        let f_alpha = Units::KipInF.thermal_expansion_factor_to(units);
        for m in [&mut steel, &mut conc] {
            m.e *= f_stress;
            m.g *= f_stress;
            m.fy *= f_stress;
            m.fu *= f_stress;
            m.fc *= f_stress;
            m.fys *= f_stress;
            m.unit_weight *= f_uw;
            m.unit_mass *= f_um;
            m.a *= f_alpha;
        }
        materials.insert(steel.name.clone(), steel);
        materials.insert(conc.name.clone(), conc);

        let mut frame_sections = IndexMap::new();
        let mut fsec = FrameSection::default();
        let fl = Units::KipInF.length_factor_to(units);
        fsec.t3 *= fl;
        fsec.t2 *= fl;
        fsec.tf *= fl;
        fsec.tw *= fl;
        fsec.t2b *= fl;
        fsec.tfb *= fl;
        fsec.area *= fl * fl;
        fsec.as2 *= fl * fl;
        fsec.as3 *= fl * fl;
        fsec.r33 *= fl;
        fsec.r22 *= fl;
        fsec.s33 *= fl.powi(3);
        fsec.s22 *= fl.powi(3);
        fsec.z33 *= fl.powi(3);
        fsec.z22 *= fl.powi(3);
        fsec.torsional_j *= fl.powi(4);
        fsec.i33 *= fl.powi(4);
        fsec.i22 *= fl.powi(4);
        frame_sections.insert(fsec.name.clone(), fsec);

        let mut area_sections = IndexMap::new();
        let mut asec = AreaSection::default();
        asec.thickness_membrane *= fl;
        asec.thickness_bending *= fl;
        area_sections.insert(asec.name.clone(), asec);

        let mut load_patterns = IndexMap::new();
        load_patterns.insert(
            "DEAD".to_string(),
            LoadPattern {
                name: "DEAD".to_string(),
                pat_type: LoadPatternType::Dead,
                self_wt_multiplier: 1.0,
            },
        );

        let mut load_cases = IndexMap::new();
        load_cases.insert(
            "DEAD".to_string(),
            LoadCase::linear_static("DEAD", "DEAD", 1.0),
        );
        load_cases.insert("MODAL".to_string(), LoadCase::modal_eigen("MODAL", 12));

        Self {
            s2k_preservation: None,
            model_name: "Untitled".to_string(),
            program_version: "19.0.0".to_string(),
            present_units: units,
            database_units: units,
            active_dofs: [true; 6],
            steel_design_code: "AISC 360-10".to_string(),
            concrete_design_code: "ACI 318-14".to_string(),
            project_info: IndexMap::new(),
            coord_systems,
            materials,
            frame_sections,
            area_sections,
            solid_sections: IndexMap::new(),
            link_props: IndexMap::new(),
            constraints: IndexMap::new(),
            points: IndexMap::new(),
            frames: IndexMap::new(),
            areas: IndexMap::new(),
            solids: IndexMap::new(),
            links: IndexMap::new(),
            functions: IndexMap::new(),
            load_patterns,
            load_cases,
            load_combos: IndexMap::new(),
            is_locked: false,
            case_results: IndexMap::new(),
            steel_design_results: IndexMap::new(),
            concrete_design_results: IndexMap::new(),
        }
    }

    pub fn add_point(&mut self, name: impl Into<String>, x: f64, y: f64, z: f64) -> String {
        let name = name.into();
        self.points
            .entry(name.clone())
            .or_insert_with(|| PointObj::new(name.clone(), x, y, z));
        name
    }

    pub fn add_frame(
        &mut self,
        name: impl Into<String>,
        point_i: impl Into<String>,
        point_j: impl Into<String>,
        section: impl Into<String>,
    ) -> String {
        let name = name.into();
        self.frames.insert(
            name.clone(),
            FrameObj::new(name.clone(), point_i, point_j, section),
        );
        name
    }

    pub fn add_area(
        &mut self,
        name: impl Into<String>,
        points: Vec<String>,
        section: impl Into<String>,
    ) -> String {
        let name = name.into();
        self.areas
            .insert(name.clone(), AreaObj::new(name.clone(), points, section));
        name
    }

    /// Converts all model geometry, material properties, section dimensions, and loads
    /// in-place from `self.present_units` to `target_units` (matching `cSapModel.SetPresentUnits`).
    pub fn convert_units_in_place(&mut self, target_units: Units) {
        let src = self.present_units;
        if src == target_units {
            return;
        }
        let fl = src.length_factor_to(target_units);
        let ff = src.force_factor_to(target_units);
        let fm = src.moment_factor_to(target_units);
        let f_stress = src.stress_factor_to(target_units);
        let f_line = src.line_load_factor_to(target_units);
        let f_uw = src.unit_weight_factor_to(target_units);
        let f_um = src.mass_density_factor_to(target_units);
        let f_mass = src.mass_factor_to(target_units);
        let f_temp = src.temp_delta_factor_to(target_units);
        let f_alpha = src.thermal_expansion_factor_to(target_units);

        for cs in self.coord_systems.values_mut() {
            for c in &mut cs.origin {
                *c *= fl;
            }
            for g in &mut cs.grids {
                g.coordinate *= fl;
            }
        }

        for m in self.materials.values_mut() {
            m.e *= f_stress;
            m.g *= f_stress;
            m.fy *= f_stress;
            m.fu *= f_stress;
            m.fc *= f_stress;
            m.fys *= f_stress;
            m.unit_weight *= f_uw;
            m.unit_mass *= f_um;
            m.a *= f_alpha;
            if let Some(eo) = &mut m.e_ortho {
                for v in eo {
                    *v *= f_stress;
                }
            }
            if let Some(go) = &mut m.g_ortho {
                for v in go {
                    *v *= f_stress;
                }
            }
            if let Some(ao) = &mut m.a_ortho {
                for v in ao {
                    *v *= f_alpha;
                }
            }
        }

        for s in self.frame_sections.values_mut() {
            s.t3 *= fl;
            s.t2 *= fl;
            s.tf *= fl;
            s.tw *= fl;
            s.t2b *= fl;
            s.tfb *= fl;
            s.dis *= fl;
            s.area *= fl * fl;
            s.as2 *= fl * fl;
            s.as3 *= fl * fl;
            s.r33 *= fl;
            s.r22 *= fl;
            s.s33 *= fl.powi(3);
            s.s22 *= fl.powi(3);
            s.z33 *= fl.powi(3);
            s.z22 *= fl.powi(3);
            s.torsional_j *= fl.powi(4);
            s.i33 *= fl.powi(4);
            s.i22 *= fl.powi(4);
            s.i23 *= fl.powi(4);
            for seg in &mut s.non_prismatic {
                if !seg.is_variable_length {
                    seg.length *= fl;
                }
            }
            if let Some(rebar) = &mut s.concrete_rebar {
                rebar.cover_top *= fl;
                rebar.cover_bot *= fl;
                rebar.total_rebar_area *= fl * fl;
            }
        }

        for a in self.area_sections.values_mut() {
            if a.prop_type != AreaPropType::ASolid {
                a.thickness_membrane *= fl;
                a.thickness_bending *= fl;
            }
        }

        for lp in self.link_props.values_mut() {
            lp.mass *= f_mass;
            lp.weight *= ff;
            lp.dj2 *= fl;
            lp.dj3 *= fl;
            let is_gap_or_hook = matches!(lp.link_type, LinkPropType::Gap | LinkPropType::Hook);
            let is_damper = lp.link_type == LinkPropType::Damper;
            for i in 0..3 {
                lp.ke[i] *= ff / fl;
                lp.ke[i + 3] *= fm;
                lp.ce[i] *= ff / fl;
                lp.ce[i + 3] *= fm;
                lp.k_nl[i] *= ff / fl;
                lp.k_nl[i + 3] *= fm;
                if is_gap_or_hook {
                    lp.param1[i] *= fl;
                } else if is_damper {
                    let exp_t = if lp.param2[i] > 0.0 { lp.param2[i] } else { 1.0 };
                    lp.param1[i] *= ff / fl.powf(exp_t);
                    lp.param1[i + 3] *= fm;
                } else {
                    lp.param1[i] *= ff;
                    lp.param1[i + 3] *= fm;
                }
            }
        }

        for p in self.points.values_mut() {
            p.x *= fl;
            p.y *= fl;
            p.z *= fl;
            for i in 0..3 {
                p.spring[i] *= ff / fl;
                p.spring[i + 3] *= fm;
                p.mass[i] *= f_mass;
                p.mass[i + 3] *= f_mass * fl * fl;
            }
            if let Some(sc) = &mut p.spring_coupled {
                let mut idx = 0;
                for c in 0..6 {
                    for r in 0..=c {
                        let scale = match (r < 3, c < 3) {
                            (true, true) => ff / fl,
                            (false, false) => fm,
                            _ => ff,
                        };
                        sc[idx] *= scale;
                        idx += 1;
                    }
                }
            }
            for lf in &mut p.loads_force {
                for i in 0..3 {
                    lf.force[i] *= ff;
                    lf.force[i + 3] *= fm;
                }
            }
            for ld in &mut p.loads_displ {
                for i in 0..3 {
                    ld.displ[i] *= fl;
                }
            }
        }

        for fr in self.frames.values_mut() {
            fr.end_offset_i *= fl;
            fr.end_offset_j *= fl;
            for i in 0..3 {
                fr.joint_offset_i[i] *= fl;
                fr.joint_offset_j[i] *= fl;
                fr.start_spring[i] *= ff / fl;
                fr.start_spring[i + 3] *= fm;
                fr.end_spring[i] *= ff / fl;
                fr.end_spring[i + 3] *= fm;
            }
            fr.pdelta_force *= ff;
            if let Some((_, cl, _, tl)) = &mut fr.tc_limits {
                *cl *= ff;
                *tl *= ff;
            }
            if let Some(cp) = &mut fr.cable_params {
                if let Some(l0) = &mut cp.undeformed_length {
                    *l0 *= fl;
                }
                if let Some(ti) = &mut cp.target_tension_i {
                    *ti *= ff;
                }
                if let Some(ht) = &mut cp.horizontal_tension {
                    *ht *= ff;
                }
            }
            for ld in &mut fr.loads_dist {
                let factor = if ld.my_type == 2 {
                    fm / fl // moment per unit length = force
                } else {
                    f_line
                };
                ld.val1 *= factor;
                ld.val2 *= factor;
            }
            for lp in &mut fr.loads_point {
                let factor = if lp.my_type == 2 { fm } else { ff };
                lp.val *= factor;
            }
            for lt in &mut fr.loads_temp {
                if lt.my_type == 1 {
                    lt.val *= f_temp;
                } else {
                    lt.val *= f_temp / fl;
                }
            }
        }

        for ar in self.areas.values_mut() {
            for lu in &mut ar.loads_uniform {
                lu.val *= f_stress;
            }
            for lp in &mut ar.loads_pressure {
                lp.val *= f_stress;
            }
            for lt in &mut ar.loads_temp {
                if lt.my_type == 1 {
                    lt.val *= f_temp;
                } else {
                    lt.val *= f_temp / fl;
                }
            }
        }

        for sol in self.solids.values_mut() {
            for v in sol.loads_temp.values_mut() {
                *v *= f_temp;
            }
            for sl in &mut sol.loads_surface {
                sl.val *= f_stress;
            }
        }

        for lc in self.load_cases.values_mut() {
            for (_, dir, sf) in &mut lc.spectrum_loads {
                let upper = dir.trim().to_ascii_uppercase();
                if !matches!(
                    upper.as_str(),
                    "R1" | "RX" | "4" | "R2" | "RY" | "5" | "R3" | "RZ" | "6"
                ) {
                    *sf *= fl;
                }
            }
            for (_, pat_or_acc, sf) in &mut lc.history_loads {
                let upper = pat_or_acc.trim().to_ascii_uppercase();
                if matches!(
                    upper.as_str(),
                    "U1" | "UX" | "X" | "U2" | "UY" | "Y" | "U3" | "UZ" | "Z"
                ) {
                    *sf *= fl;
                }
            }
        }

        self.present_units = target_units;
        self.database_units = target_units;
        // Stored outputs use the previous units. Until every result component has
        // a dimensional conversion, require analysis again rather than relabeling it.
        self.case_results.clear();
        self.steel_design_results.clear();
        self.concrete_design_results.clear();
        self.is_locked = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn changing_units_invalidates_outputs_instead_of_relabeling_them() {
        let mut model = SapModel::new(Units::KipInF);
        model.add_point("1", 12.0, 0.0, 0.0);
        model.case_results.insert("DEAD".into(), CaseResults::default());
        model.is_locked = true;
        model.convert_units_in_place(Units::KipInF);
        assert!(!model.case_results.is_empty());
        model.convert_units_in_place(Units::KipFtF);
        assert!((model.points["1"].x - 1.0).abs() < 1e-12);
        assert!(model.case_results.is_empty());
        assert!(!model.is_locked);
    }

    #[test]
    fn new_template_drops_links_to_previous_geometry() {
        let mut model = SapModel::new(Units::KipInF);
        model.links.insert("old".into(), LinkObj {
            name: "old".into(), point_i: "deleted".into(), point_j: None,
            prop: "old".into(), angle_deg: 0.0, selected: false,
        });
        model.generate_beam(1, 120.0, true, "FSEC1");
        assert!(model.links.is_empty());
        assert_eq!(model.frames.len(), 1);
    }

    #[test]
    fn test_finding_6_pinned_truss_and_brace_m3_releases() {
        let mut truss = SapModel::new(Units::KipInF);
        truss.generate_2d_truss(4, 120.0, 120.0, true, "FSEC1", "FSEC1");
        let diag = truss
            .frames
            .values()
            .find(|f| f.release_i.iter().any(|&r| r))
            .expect("Expected pinned truss web diagonal");
        assert!(
            diag.release_i[5] && diag.release_j[5],
            "2D XZ truss diagonals must release M3 (index 5), got {:?}",
            diag.release_i
        );
        assert!(!diag.release_i[4] && !diag.release_j[4]);

        let mut frame = SapModel::new(Units::KipInF);
        frame.generate_2d_frame(
            FrameType2D::ConcentricBraced,
            2,
            144.0,
            2,
            240.0,
            true,
            "FSEC1",
            "FSEC1",
            "FSEC1",
            0.0,
        );
        let brace = frame
            .frames
            .values()
            .find(|f| f.release_i.iter().any(|&r| r))
            .expect("Expected pinned concentric brace");
        assert!(
            brace.release_i[5] && brace.release_j[5],
            "2D XZ frame braces must release M3 (index 5), got {:?}",
            brace.release_i
        );
        assert!(!brace.release_i[4] && !brace.release_j[4]);
    }

    #[test]
    fn test_finding_10_convert_units_in_place_rebar_and_links() {
        let mut model = SapModel::new(Units::KipFtF);
        let fsec = model.frame_sections.get_mut("FSEC1").unwrap();
        let rebar = ConcreteRebarConfig {
            is_column: true,
            rebar_mat_long: "4000Psi".into(),
            rebar_mat_confine: "4000Psi".into(),
            cover_top: 0.25, // 0.25 ft = 3.0 in
            cover_bot: 0.25,
            num_bars_3dir: 3,
            num_bars_2dir: 3,
            total_rebar_area: 0.05, // 0.05 ft^2 = 7.2 in^2
        };
        fsec.concrete_rebar = Some(rebar);

        let mut lp = LinkProp {
            name: "L1".into(),
            link_type: LinkPropType::Gap,
            dof_active: [true; 6],
            dof_fixed: [false; 6],
            dof_nonlinear: [true; 6],
            nonlinear_data_complete: true,
            ke: [120.0, 120.0, 120.0, 100.0, 100.0, 100.0],
            ce: [0.0; 6],
            k_nl: [240.0, 0.0, 0.0, 50.0, 0.0, 0.0],
            param1: [0.1; 6],
            param2: [0.0; 6],
            dj2: 0.5,
            dj3: 0.5,
            mass: 1.0,
            weight: 10.0,
        };
        model.link_props.insert("L1".into(), lp.clone());

        model.convert_units_in_place(Units::KipInF);
        let rb = model.frame_sections["FSEC1"].concrete_rebar.as_ref().unwrap();
        assert!((rb.cover_top - 3.0).abs() < 1e-9);
        assert!((rb.total_rebar_area - 7.2).abs() < 1e-9);

        lp = model.link_props["L1"].clone();
        assert!((lp.ke[0] - 10.0).abs() < 1e-9); // 120 kip/ft = 10 kip/in
        assert!((lp.ke[3] - 1200.0).abs() < 1e-9); // 100 kip-ft/rad = 1200 kip-in/rad
        assert!((lp.param1[0] - 1.2).abs() < 1e-9); // 0.1 ft gap opening = 1.2 in
        assert!((lp.dj2 - 6.0).abs() < 1e-9); // 0.5 ft = 6.0 in
    }

    #[test]
    fn convert_units_scales_dynamic_accelerations_and_column_major_coupled_springs() {
        let mut model = SapModel::new(Units::KipFtF);
        model.add_point("1", 0.0, 0.0, 0.0);
        // Column-major upper triangle: 0=U1U1, 3=U1U3 (F/L), 6=U1R1 (F), 9=R1R1 (F*L), 20=R3R3 (F*L)
        let mut sc = [0.0; 21];
        sc[0] = 120.0; // 120 kip/ft -> 10 kip/in
        sc[3] = 240.0; // 240 kip/ft -> 20 kip/in
        sc[6] = 50.0;  // 50 kip/rad -> 50 kip/rad
        sc[9] = 100.0; // 100 kip-ft/rad -> 1200 kip-in/rad
        sc[20] = 30.0; // 30 kip-ft/rad -> 360 kip-in/rad
        model.points.get_mut("1").unwrap().spring_coupled = Some(sc);

        let mut spec = LoadCase::linear_static("SPEC", "DEAD", 1.0);
        spec.case_type = LoadCaseType::ResponseSpectrum;
        spec.spectrum_loads = vec![
            ("RS1".into(), "U1".into(), 32.174),
            ("RS1".into(), "R3".into(), 1.5),
        ];
        model.load_cases.insert("SPEC".into(), spec);

        let mut hist = LoadCase::linear_static("HIST", "DEAD", 1.0);
        hist.case_type = LoadCaseType::LinearHistory;
        hist.history_loads = vec![
            ("TH1".into(), "UZ".into(), 32.174),
            ("TH1".into(), "DEAD".into(), 2.0),
        ];
        model.load_cases.insert("HIST".into(), hist);

        model.convert_units_in_place(Units::KipInF);

        let out_sc = model.points["1"].spring_coupled.unwrap();
        assert!((out_sc[0] - 10.0).abs() < 1e-12);
        assert!((out_sc[3] - 20.0).abs() < 1e-12);
        assert!((out_sc[6] - 50.0).abs() < 1e-12);
        assert!((out_sc[9] - 1200.0).abs() < 1e-12);
        assert!((out_sc[20] - 360.0).abs() < 1e-12);

        assert!((model.load_cases["SPEC"].spectrum_loads[0].2 - 32.174 * 12.0).abs() < 1e-9);
        assert!((model.load_cases["SPEC"].spectrum_loads[1].2 - 1.5).abs() < 1e-12);
        assert!((model.load_cases["HIST"].history_loads[0].2 - 32.174 * 12.0).abs() < 1e-9);
        assert!((model.load_cases["HIST"].history_loads[1].2 - 2.0).abs() < 1e-12);
    }

    #[test]
    fn template_generation_clears_stale_s2k_preservation_and_transient_json_roundtrips() {
        let mut model = SapModel::new(Units::KipInF);
        model.s2k_preservation = Some(S2kPreservation {
            source_text: "TABLE: \"OPAQUE\"".into(),
            canonical_baseline: String::new(),
        });
        model.generate_beam(2, 120.0, true, "FSEC1");
        assert!(model.s2k_preservation.is_none());

        let mut cr = CaseResults::default();
        cr.transient_state = Some(TransientTerminalState {
            point_order: vec!["1".into()],
            u: vec![1.0; 6],
            v: vec![2.0; 6],
            a: vec![3.0; 6],
            previous_force: vec![4.0; 6],
            nonlinear_link_forces: vec![NonlinearLinkForceState {
                link: "L1".into(),
                dof: 0,
                force: -5.5,
            }],
        });
        let json = serde_json::to_string(&cr).unwrap();
        let back: CaseResults = serde_json::from_str(&json).unwrap();
        assert_eq!(back, cr);
    }
}

