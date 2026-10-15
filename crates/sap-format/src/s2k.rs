mod preservation;

use std::collections::BTreeMap;

use sap_model::{
    AreaLoadSurfacePressure, AreaLoadTemperature, AreaLoadUniform, AreaObj, AreaPropType,
    AreaSection, CableParams, ComboType, ConcreteRebarConfig, ConstraintAxis, ConstraintDef,
    ConstraintType, CoordSystem, DynamicFunction, FrameLoadDistributed, FrameLoadPoint,
    FrameLoadTemperature, FrameObj, FramePropType, FrameSection, GridLine, LinkObj, LinkProp,
    LinkPropType, LoadCase, LoadCaseAssignment, LoadCaseType, LoadCombo, LoadPattern,
    LoadPatternType, MatType, Material, ModalComboMethod, ModalSubType, NLGeomType,
    NonPrismaticSegment, PlaneType, PointLoadDispl, PointLoadForce, PointObj, SapModel, ShellType,
    SolidObj, SolidSection, SolidSurfaceLoad, TimeIntegrationMethod,
};
use sap_units::Units;

use crate::{FormatError, ImportReport};

thread_local! {
    static COLUMN_READS: std::cell::RefCell<(String, BTreeMap<String, std::collections::BTreeSet<String>>)> = const { std::cell::RefCell::new((String::new(), BTreeMap::new())) };
}

/// Parse a `Key=Value` record line (after joining ` _` line continuations) into case-insensitive
/// uppercase keys mapped to unquoted string values.
fn parse_kv_row(line: &str) -> Result<BTreeMap<String, String>, FormatError> {
    let mut map = BTreeMap::new();
    let bytes = line.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        while i < bytes.len() && bytes[i].is_ascii_whitespace() {
            i += 1;
        }
        if i == bytes.len() {
            break;
        }
        let start = i;
        while i < bytes.len() && bytes[i] != b'=' && !bytes[i].is_ascii_whitespace() {
            i += 1;
        }
        let key = line[start..i].to_ascii_uppercase();
        while i < bytes.len() && bytes[i].is_ascii_whitespace() {
            i += 1;
        }
        if key.is_empty() || i == bytes.len() || bytes[i] != b'=' {
            return Err(FormatError::ParseS2k(format!(
                "Malformed Key=Value record: {}",
                line
            )));
        }
        i += 1;
        while i < bytes.len() && bytes[i].is_ascii_whitespace() {
            i += 1;
        }
        let value = if i < bytes.len() && bytes[i] == b'"' {
            i += 1;
            let mut value = String::new();
            let mut start = i;
            loop {
                while i < bytes.len() && bytes[i] != b'"' {
                    i += 1;
                }
                if i == bytes.len() {
                    return Err(FormatError::ParseS2k(format!(
                        "Unterminated quoted value for {}",
                        key
                    )));
                }
                value.push_str(&line[start..i]);
                i += 1;
                if i < bytes.len() && bytes[i] == b'"' {
                    value.push('"');
                    i += 1;
                    start = i;
                } else {
                    break;
                }
            }
            if i < bytes.len() && !bytes[i].is_ascii_whitespace() {
                return Err(FormatError::ParseS2k(format!(
                    "Missing separator after quoted value for {}",
                    key
                )));
            }
            value
        } else {
            let start = i;
            while i < bytes.len() && !bytes[i].is_ascii_whitespace() {
                i += 1;
            }
            line[start..i].to_string()
        };
        if map.insert(key.clone(), value).is_some() {
            return Err(FormatError::ParseS2k(format!(
                "Duplicate field {} in record",
                key
            )));
        }
    }
    Ok(map)
}

fn get_str<'a>(row: &'a BTreeMap<String, String>, key: &str) -> Option<&'a str> {
    let key = key.to_ascii_uppercase();
    COLUMN_READS.with(|capture| {
        let mut capture = capture.borrow_mut();
        let table = capture.0.clone();
        if !table.is_empty() {
            capture.1.entry(table).or_default().insert(key.clone());
        }
    });
    row.get(&key).map(|s| s.as_str())
}

fn get_f64(row: &BTreeMap<String, String>, key: &str) -> Option<f64> {
    get_str(row, key).and_then(|s| s.replace(',', ".").parse::<f64>().ok())
}

fn get_i32(row: &BTreeMap<String, String>, key: &str) -> Option<i32> {
    get_str(row, key).and_then(|s| s.parse::<i32>().ok())
}

fn get_bool(row: &BTreeMap<String, String>, key: &str) -> Option<bool> {
    get_str(row, key).map(|s| {
        matches!(
            s.trim().to_ascii_uppercase().as_str(),
            "YES" | "TRUE" | "1" | "Y" | "ON"
        )
    })
}

// Do not turn malformed physical input into zero/default values. Missing optional fields
// still retain their existing defaults; a supplied invalid number is an import error.
fn validate_physical_fields(
    table: &str,
    row: &BTreeMap<String, String>,
) -> Result<(), FormatError> {
    let keys: &[&str] = match table {
        "JOINT COORDINATES" => &["GlobalX", "GlobalY", "GlobalZ", "XorR", "Y", "Z"],
        "LINK PROPERTY DEFINITIONS 01 - GENERAL" => &["TotalMass", "TotalWt", "DJ2", "DJ3"],
        "LINK PROPERTY DEFINITIONS 02 - LINEAR" => &["Ke", "Ce", "Knl", "Param1", "Param2"],
        "JOINT LOADS - FORCE" => &["F1", "F2", "F3", "M1", "M2", "M3"],
        "JOINT LOADS - GROUND DISPLACEMENT" | "JOINT SPRING ASSIGNMENTS 1 - UNCOUPLED" => {
            &["U1", "U2", "U3", "R1", "R2", "R3"]
        }
        "FRAME LOADS - DISTRIBUTED" => &[
            "RelDistA", "RelDistB", "AbsDistA", "AbsDistB", "FOverLA", "FOverLB", "MOverLA",
            "MOverLB",
        ],
        "FRAME LOADS - POINT" => &["RelDist", "AbsDist", "Force", "Moment"],
        "FRAME SECTION PROPERTIES 01 - GENERAL" => &[
            "t3",
            "t2",
            "tf",
            "tw",
            "t2b",
            "tfb",
            "dis",
            "Area",
            "TorsConst",
            "I33",
            "I22",
            "I23",
            "AS2",
            "AS3",
            "S33",
            "S22",
            "Z33",
            "Z22",
            "R33",
            "R22",
            "AMod",
            "A2Mod",
            "A3Mod",
            "JMod",
            "I2Mod",
            "I3Mod",
            "MMod",
            "WMod",
        ],
        "MATERIAL PROPERTIES 02 - BASIC MECHANICAL PROPERTIES" => &[
            "UnitWeight",
            "UnitMass",
            "E1",
            "E2",
            "E3",
            "G12",
            "G13",
            "G23",
            "U12",
            "U13",
            "U23",
            "A1",
            "A2",
            "A3",
            "Fy",
            "Fu",
            "Fc",
            "Fys",
        ],
        "LOAD PATTERN DEFINITIONS" => &["SelfWtMult"],
        "CASE - STATIC 1 - LOAD ASSIGNMENTS" => &["LoadSF"],
        "COMBINATION DEFINITIONS" => &["ScaleFactor"],
        _ => &[],
    };
    for key in keys {
        if get_str(row, key).is_some() && !get_f64(row, key).is_some_and(f64::is_finite) {
            return Err(FormatError::ParseS2k(format!(
                "Invalid finite number for {} in {}",
                key, table
            )));
        }
    }
    if table == "JOINT RESTRAINT ASSIGNMENTS" || table == "ACTIVE DEGREES OF FREEDOM" {
        let keys = if table == "JOINT RESTRAINT ASSIGNMENTS" {
            ["U1", "U2", "U3", "R1", "R2", "R3"]
        } else {
            ["UX", "UY", "UZ", "RX", "RY", "RZ"]
        };
        for key in keys {
            if let Some(value) = get_str(row, key) {
                if !matches!(
                    value.to_ascii_uppercase().as_str(),
                    "YES" | "TRUE" | "1" | "Y" | "ON" | "NO" | "FALSE" | "0" | "N" | "OFF"
                ) {
                    return Err(FormatError::ParseS2k(format!(
                        "Invalid boolean for {} in {}",
                        key, table
                    )));
                }
            }
        }
    }
    Ok(())
}

fn format_bool(b: bool) -> &'static str {
    if b {
        "Yes"
    } else {
        "No"
    }
}

fn format_val(s: &str) -> String {
    if s.is_empty()
        || s.chars().any(char::is_whitespace)
        || s.contains(',')
        || s.contains('"')
        || s.contains('-')
    {
        format!("\"{}\"", s.replace('"', "\"\""))
    } else {
        s.to_string()
    }
}

fn parse_mat_type(s: &str) -> MatType {
    match s.trim().to_ascii_uppercase().as_str() {
        "STEEL" => MatType::Steel,
        "CONCRETE" | "CONC" => MatType::Concrete,
        "ALUMINUM" | "ALUM" => MatType::Aluminum,
        "COLDFORMED" | "COLD FORMED" => MatType::ColdFormed,
        "REBAR" => MatType::Rebar,
        "TENDON" => MatType::Tendon,
        _ => MatType::NoDesign,
    }
}

fn mat_type_str(t: MatType) -> &'static str {
    match t {
        MatType::Steel => "Steel",
        MatType::Concrete => "Concrete",
        MatType::NoDesign => "NoDesign",
        MatType::Aluminum => "Aluminum",
        MatType::ColdFormed => "ColdFormed",
        MatType::Rebar => "Rebar",
        MatType::Tendon => "Tendon",
    }
}

fn parse_frame_prop_type(s: &str) -> FramePropType {
    match s.trim().to_ascii_uppercase().as_str() {
        "I/WIDE FLANGE" | "I" | "W" => FramePropType::I,
        "CHANNEL" | "C" => FramePropType::Channel,
        "TEE" | "T" => FramePropType::T,
        "ANGLE" | "L" => FramePropType::Angle,
        "DOUBLE ANGLE" | "2L" | "DBLANGLE" => FramePropType::DblAngle,
        "BOX/TUBE" | "BOX" | "TUBE" | "B" => FramePropType::Box,
        "PIPE" | "P" => FramePropType::Pipe,
        "RECTANGULAR" | "RECT" | "R" => FramePropType::Rectangular,
        "CIRCLE" | "CIRCULAR" | "CR" => FramePropType::Circle,
        "GENERAL" | "G" => FramePropType::General,
        "DOUBLE CHANNEL" | "2C" | "DBCHANNEL" => FramePropType::DbChannel,
        "AUTO SELECT" | "AUTO" => FramePropType::Auto,
        "SECTION DESIGNER" | "SD" => FramePropType::SD,
        "NONPRISMATIC" | "VARIABLE" => FramePropType::Variable,
        "JOIST" => FramePropType::Joist,
        "BRIDGE" => FramePropType::Bridge,
        "COLD FORMED C" | "COLDC" => FramePropType::ColdC,
        "COLD FORMED 2C" | "COLD2C" => FramePropType::Cold2C,
        "COLD FORMED Z" | "COLDZ" => FramePropType::ColdZ,
        "COLD FORMED L" | "COLDL" => FramePropType::ColdL,
        "COLD FORMED 2L" | "COLD2L" => FramePropType::Cold2L,
        "COLD FORMED HAT" | "COLDHAT" => FramePropType::ColdHat,
        "BUILT UP I COVERPLATE" | "BUILTUPICOVERPLATE" => FramePropType::BuiltUpICoverplate,
        "PCC GIRDER I" | "PCCGIRDERI" => FramePropType::PCCGirderI,
        "PCC GIRDER U" | "PCCGIRDERU" => FramePropType::PCCGirderU,
        "BUILT UP I HYBRID" | "BUILTUPIHYBRID" => FramePropType::BuiltUpIHybrid,
        "BUILT UP U HYBRID" | "BUILTUPUHYBRID" => FramePropType::BuiltUpUHybrid,
        "CONCRETE L" | "CONCRETEL" => FramePropType::ConcreteL,
        "COLD FORMED BOX" | "COLDBOX" => FramePropType::ColdBox,
        "COLD FORMED I" | "COLDI" => FramePropType::ColdI,
        "COLD FORMED PIPE" | "COLDPIPE" => FramePropType::ColdPipe,
        "COLD FORMED T" | "COLDT" => FramePropType::ColdT,
        "TRAPEZOIDAL" => FramePropType::Trapezoidal,
        "PCC GIRDER SUPER T" | "PCCGIRDERSUPERT" => FramePropType::PCCGirderSuperT,
        _ => FramePropType::General,
    }
}

fn frame_prop_type_str(t: FramePropType) -> &'static str {
    match t {
        FramePropType::I => "I/Wide Flange",
        FramePropType::Channel => "Channel",
        FramePropType::T => "Tee",
        FramePropType::Angle => "Angle",
        FramePropType::DblAngle => "Double Angle",
        FramePropType::Box => "Box/Tube",
        FramePropType::Pipe => "Pipe",
        FramePropType::Rectangular => "Rectangular",
        FramePropType::Circle => "Circle",
        FramePropType::General => "General",
        FramePropType::DbChannel => "Double Channel",
        FramePropType::Auto => "Auto Select",
        FramePropType::SD => "Section Designer",
        FramePropType::Variable => "Nonprismatic",
        FramePropType::Joist => "Joist",
        FramePropType::Bridge => "Bridge",
        FramePropType::ColdC => "Cold Formed C",
        FramePropType::Cold2C => "Cold Formed 2C",
        FramePropType::ColdZ => "Cold Formed Z",
        FramePropType::ColdL => "Cold Formed L",
        FramePropType::Cold2L => "Cold Formed 2L",
        FramePropType::ColdHat => "Cold Formed Hat",
        FramePropType::BuiltUpICoverplate => "Built Up I Coverplate",
        FramePropType::PCCGirderI => "PCC Girder I",
        FramePropType::PCCGirderU => "PCC Girder U",
        FramePropType::BuiltUpIHybrid => "Built Up I Hybrid",
        FramePropType::BuiltUpUHybrid => "Built Up U Hybrid",
        FramePropType::ConcreteL => "Concrete L",
        FramePropType::ColdBox => "Cold Formed Box",
        FramePropType::ColdI => "Cold Formed I",
        FramePropType::ColdPipe => "Cold Formed Pipe",
        FramePropType::ColdT => "Cold Formed T",
        FramePropType::Trapezoidal => "Trapezoidal",
        FramePropType::PCCGirderSuperT => "PCC Girder Super T",
    }
}

fn parse_load_pat_type(s: &str) -> LoadPatternType {
    match s.trim().to_ascii_uppercase().as_str() {
        "DEAD" => LoadPatternType::Dead,
        "SUPER DEAD" | "SUPERDEAD" => LoadPatternType::SuperDead,
        "LIVE" => LoadPatternType::Live,
        "REDUCIBLE LIVE" | "REDUCELIVE" => LoadPatternType::ReduceLive,
        "QUAKE" | "SEISMIC" => LoadPatternType::Quake,
        "WIND" => LoadPatternType::Wind,
        "SNOW" => LoadPatternType::Snow,
        "TEMP" | "TEMPERATURE" => LoadPatternType::Temperature,
        "MOVE" | "MOVING" => LoadPatternType::Move,
        "ROOFLIVE" | "ROOF LIVE" => LoadPatternType::Rooflive,
        "NOTIONAL" => LoadPatternType::Notional,
        "PATTERN LIVE" | "PATTERNLIVE" => LoadPatternType::PatternLive,
        "WAVE" => LoadPatternType::Wave,
        "PRESTRESS" => LoadPatternType::Prestress,
        _ => LoadPatternType::Other,
    }
}

fn load_pat_type_str(t: LoadPatternType) -> &'static str {
    match t {
        LoadPatternType::Dead => "Dead",
        LoadPatternType::SuperDead => "Super Dead",
        LoadPatternType::Live => "Live",
        LoadPatternType::ReduceLive => "Reducible Live",
        LoadPatternType::Quake => "Quake",
        LoadPatternType::Wind => "Wind",
        LoadPatternType::Snow => "Snow",
        LoadPatternType::Other => "Other",
        LoadPatternType::Move => "Move",
        LoadPatternType::Temperature => "Temp",
        LoadPatternType::Rooflive => "Roof Live",
        LoadPatternType::Notional => "Notional",
        LoadPatternType::PatternLive => "Pattern Live",
        LoadPatternType::Wave => "Wave",
        LoadPatternType::Prestress => "Prestress",
    }
}

fn parse_load_dir(s: &str) -> i32 {
    let su = s.trim().to_ascii_uppercase();
    match su.as_str() {
        "LOCAL 1" | "LOCAL1" | "1" => 1,
        "LOCAL 2" | "LOCAL2" | "2" => 2,
        "LOCAL 3" | "LOCAL3" | "3" => 3,
        "X" | "GLOBAL X" | "4" => 4,
        "Y" | "GLOBAL Y" | "5" => 5,
        "Z" | "GLOBAL Z" | "6" => 6,
        "PROJ X" | "PROJECTED X" | "7" => 7,
        "PROJ Y" | "PROJECTED Y" | "8" => 8,
        "PROJ Z" | "PROJECTED Z" | "9" => 9,
        "GRAVITY" | "GRAV" | "10" => 10,
        "PROJ GRAVITY" | "PROJ GRAV" | "11" => 11,
        _ => su.parse::<i32>().unwrap_or(10),
    }
}

fn load_dir_str(dir: i32) -> String {
    match dir {
        1 => "Local 1".to_string(),
        2 => "Local 2".to_string(),
        3 => "Local 3".to_string(),
        4 => "X".to_string(),
        5 => "Y".to_string(),
        6 => "Z".to_string(),
        7 => "Proj X".to_string(),
        8 => "Proj Y".to_string(),
        9 => "Proj Z".to_string(),
        10 => "Gravity".to_string(),
        11 => "Proj Gravity".to_string(),
        other => other.to_string(),
    }
}

// Definitions must exist before assignments even when a valid text export rearranges tables.
fn table_priority(name: &str) -> u8 {
    match name {
        "PROGRAM CONTROL" => 0,
        "COORDINATE SYSTEMS" => 1,
        "GRID LINES" => 2,
        "MATERIAL PROPERTIES 01 - GENERAL" => 3,
        "FRAME SECTION PROPERTIES 01 - GENERAL" => 5,
        "AREA SECTION PROPERTIES"
        | "SOLID PROPERTY DEFINITIONS"
        | "LINK PROPERTY DEFINITIONS 01 - GENERAL"
        | "JOINT CONSTRAINT DEFINITIONS"
        | "LOAD PATTERN DEFINITIONS"
        | "LOAD CASE DEFINITIONS" => 7,
        "JOINT COORDINATES" => 9,
        _ if name.starts_with("MATERIAL PROPERTIES") => 4,
        _ if name.starts_with("FRAME SECTION PROPERTIES") => 6,
        _ if name.starts_with("LINK PROPERTY DEFINITIONS") => 8,
        _ if name.starts_with("CONNECTIVITY -") => 10,
        _ => 11,
    }
}

fn order_tables(lines: Vec<String>) -> Vec<String> {
    let mut blocks: Vec<(u8, Vec<String>)> = vec![(0, Vec::new())];
    for line in lines {
        if line.eq_ignore_ascii_case("END TABLE DATA") {
            break;
        }
        if line
            .get(..6)
            .is_some_and(|p| p.eq_ignore_ascii_case("TABLE:"))
        {
            let name = line[6..].trim().trim_matches('"').to_ascii_uppercase();
            blocks.push((table_priority(&name), vec![line]));
        } else {
            blocks.last_mut().unwrap().1.push(line);
        }
    }
    blocks.sort_by_key(|(priority, _)| *priority);
    blocks.into_iter().flat_map(|(_, lines)| lines).collect()
}

fn load_relative_distance(
    row: &BTreeMap<String, String>,
    model: &SapModel,
    frame_name: &str,
    relative_key: &str,
    absolute_key: &str,
    default: f64,
) -> Result<f64, FormatError> {
    let dist_type = get_str(row, "DistType").unwrap_or("RelDist");
    let value = if dist_type.eq_ignore_ascii_case("AbsDist") {
        let frame = model.frames.get(frame_name).ok_or_else(|| {
            FormatError::ParseS2k(format!("Load references unknown frame {}", frame_name))
        })?;
        let a = model.points.get(&frame.point_i);
        let b = model.points.get(&frame.point_j);
        let length = a
            .zip(b)
            .map(|(a, b)| ((b.x - a.x).powi(2) + (b.y - a.y).powi(2) + (b.z - a.z).powi(2)).sqrt())
            .filter(|l| l.is_finite() && *l > 0.0)
            .ok_or_else(|| {
                FormatError::ParseS2k(format!(
                    "Cannot normalize absolute load distance on frame {}",
                    frame_name
                ))
            })?;
        get_f64(row, absolute_key).ok_or_else(|| {
            FormatError::ParseS2k(format!(
                "Missing or invalid {} on frame {}",
                absolute_key, frame_name
            ))
        })? / length
    } else if dist_type.eq_ignore_ascii_case("RelDist") {
        get_f64(row, relative_key).unwrap_or(default)
    } else {
        return Err(FormatError::ParseS2k(format!(
            "Unknown DistType {}",
            dist_type
        )));
    };
    if !value.is_finite() || !(0.0..=1.0).contains(&value) {
        return Err(FormatError::ParseS2k(format!(
            "Load distance {} outside frame {}",
            value, frame_name
        )));
    }
    Ok(value)
}

/// Parses a SAP2000 `.s2k` / `.$2k` text model into a [`SapModel`].
pub fn parse_s2k(input: &str) -> Result<SapModel, FormatError> {
    Ok(parse_s2k_with_report(input)?.model)
}

/// Parse supported model data and retain original text for checked edit writeback.
/// Unsupported tables/columns remain opaque, with explicit import diagnostics.
pub fn parse_s2k_with_report(input: &str) -> Result<ImportReport, FormatError> {
    parse_s2k_internal(input, true).map(|(report, _)| report)
}

pub(super) fn parse_s2k_internal(
    input: &str,
    preserve: bool,
) -> Result<
    (
        ImportReport,
        BTreeMap<String, std::collections::BTreeSet<String>>,
    ),
    FormatError,
> {
    COLUMN_READS.with(|capture| *capture.borrow_mut() = (String::new(), BTreeMap::new()));
    if input.trim().is_empty() {
        return Err(FormatError::ParseS2k("Empty .s2k input".into()));
    }
    let mut model = SapModel::new(Units::KipInF);
    // Clear pre-populated defaults once we encounter tables in the file
    let mut cleared_csys = false;
    let mut cleared_materials = false;
    let mut cleared_fsecs = false;
    let mut cleared_asecs = false;
    let mut cleared_lpats = false;
    let mut cleared_lcases = false;
    let mut saw_table = false;
    let mut saw_case_loads_table = false;
    let mut unsupported_tables = std::collections::BTreeSet::new();
    let mut explicit_nonlinear_rows: BTreeMap<String, std::collections::BTreeSet<usize>> =
        BTreeMap::new();

    // 1. Join line continuations ending with ` _`
    let mut logical_lines: Vec<String> = Vec::new();
    let mut current_line = String::new();
    for raw_line in input.lines() {
        let trimmed_end = raw_line.trim_start_matches('\u{feff}').trim_end();
        if trimmed_end.is_empty() {
            if !current_line.is_empty() {
                logical_lines.push(std::mem::take(&mut current_line));
            }
            continue;
        }
        let check_comment = trimmed_end.trim_start();
        if check_comment.starts_with('$') || check_comment.starts_with(';') {
            continue;
        }
        if let Some(prefix) = trimmed_end.strip_suffix(" _") {
            if !current_line.is_empty() {
                current_line.push(' ');
            }
            current_line.push_str(prefix.trim());
        } else if !current_line.is_empty() {
            current_line.push(' ');
            current_line.push_str(trimmed_end.trim());
            logical_lines.push(std::mem::take(&mut current_line));
        } else {
            logical_lines.push(trimmed_end.trim().to_string());
        }
    }
    if !current_line.is_empty() {
        logical_lines.push(current_line);
    }

    let mut current_table = String::new();
    for line in order_tables(logical_lines) {
        if line.eq_ignore_ascii_case("END TABLE DATA") {
            break;
        }
        if line
            .get(..6)
            .is_some_and(|prefix| prefix.eq_ignore_ascii_case("TABLE:"))
        {
            let rest = &line[6..];
            current_table = rest.trim().trim_matches('"').to_ascii_uppercase();
            saw_table = true;
            match current_table.as_str() {
                "COORDINATE SYSTEMS" if !cleared_csys => {
                    model.coord_systems.clear();
                    cleared_csys = true;
                }
                "MATERIAL PROPERTIES 01 - GENERAL" if !cleared_materials => {
                    model.materials.clear();
                    cleared_materials = true;
                }
                "FRAME SECTION PROPERTIES 01 - GENERAL" if !cleared_fsecs => {
                    model.frame_sections.clear();
                    cleared_fsecs = true;
                }
                "AREA SECTION PROPERTIES" if !cleared_asecs => {
                    model.area_sections.clear();
                    cleared_asecs = true;
                }
                "LOAD PATTERN DEFINITIONS" if !cleared_lpats => {
                    model.load_patterns.clear();
                    cleared_lpats = true;
                }
                "LOAD CASE DEFINITIONS" if !cleared_lcases => {
                    model.load_cases.clear();
                    cleared_lcases = true;
                }
                "CASE - STATIC 1 - LOAD ASSIGNMENTS" => {
                    saw_case_loads_table = true;
                }
                _ => {}
            }
            continue;
        }
        if current_table.is_empty() {
            // Allow standard SAP2000 banner line `File ... was saved ...`
            let lu = line.to_ascii_uppercase();
            if lu.starts_with("FILE ") || lu.starts_with("PROGRAM ") {
                continue;
            }
            return Err(FormatError::ParseS2k(format!(
                "Unexpected line before any TABLE header: '{}'",
                line
            )));
        }
        if !preservation::supported_table(&current_table) {
            unsupported_tables.insert(current_table.clone());
            continue;
        }
        if !line.contains('=') {
            return Err(FormatError::ParseS2k(format!(
                "Invalid record line without Key=Value in table '{}': '{}'",
                current_table, line
            )));
        }
        let row = parse_kv_row(&line)?;
        if row.is_empty() {
            continue;
        }
        COLUMN_READS.with(|capture| capture.borrow_mut().0 = current_table.clone());
        validate_physical_fields(&current_table, &row)?;

        match current_table.as_str() {
            "PROGRAM CONTROL" => {
                if let Some(mname) = get_str(&row, "ModelName") {
                    model.model_name = mname.to_string();
                }
                if let Some(ver) = get_str(&row, "Version") {
                    model.program_version = ver.to_string();
                }
                if let Some(u_str) = get_str(&row, "CurrUnits") {
                    let u = u_str.parse::<Units>()?;
                    model.present_units = u;
                    model.database_units = u;
                }
                if let Some(du_str) = get_str(&row, "DataBaseUnits") {
                    model.database_units = du_str.parse::<Units>()?;
                }
                if let Some(sc) = get_str(&row, "SteelCode") {
                    model.steel_design_code = sc.to_string();
                }
                if let Some(cc) = get_str(&row, "ConcCode") {
                    model.concrete_design_code = cc.to_string();
                }
                if let Some(lk) = get_bool(&row, "IsLocked") {
                    model.is_locked = lk;
                }
            }
            "PROJECT INFORMATION" => {
                if let (Some(item), Some(data)) = (get_str(&row, "Item"), get_str(&row, "Data")) {
                    model
                        .project_info
                        .insert(item.to_string(), data.to_string());
                }
            }
            "ACTIVE DEGREES OF FREEDOM" => {
                for (idx, k) in ["UX", "UY", "UZ", "RX", "RY", "RZ"].iter().enumerate() {
                    if let Some(b) = get_bool(&row, k) {
                        model.active_dofs[idx] = b;
                    }
                }
            }
            "COORDINATE SYSTEMS" => {
                if let Some(name) = get_str(&row, "Name") {
                    let cs = model
                        .coord_systems
                        .entry(name.to_string())
                        .or_insert_with(|| CoordSystem {
                            name: name.to_string(),
                            ..Default::default()
                        });
                    cs.origin[0] = get_f64(&row, "X").unwrap_or(0.0);
                    cs.origin[1] = get_f64(&row, "Y").unwrap_or(0.0);
                    cs.origin[2] = get_f64(&row, "Z").unwrap_or(0.0);
                    cs.angles_deg[0] = get_f64(&row, "AboutZ").unwrap_or(0.0);
                    cs.angles_deg[1] = get_f64(&row, "AboutY").unwrap_or(0.0);
                    cs.angles_deg[2] = get_f64(&row, "AboutX").unwrap_or(0.0);
                }
            }
            "GRID LINES" => {
                let csys = get_str(&row, "CoordSys").unwrap_or("GLOBAL").to_string();
                let cs = model
                    .coord_systems
                    .entry(csys.clone())
                    .or_insert_with(|| CoordSystem {
                        name: csys,
                        ..Default::default()
                    });
                cs.grids.push(GridLine {
                    axis: get_str(&row, "AxisDir").unwrap_or("X").to_string(),
                    id: get_str(&row, "GridID").unwrap_or("1").to_string(),
                    coordinate: get_f64(&row, "XRYZCoord").unwrap_or(0.0),
                    visible: get_bool(&row, "Visible").unwrap_or(true),
                    bubble_loc: get_str(&row, "BubbleLoc").unwrap_or("End").to_string(),
                });
            }
            "MATERIAL PROPERTIES 01 - GENERAL" => {
                if !cleared_materials {
                    model.materials.clear();
                    cleared_materials = true;
                }
                if let Some(name) = get_str(&row, "Material") {
                    let mat = model
                        .materials
                        .entry(name.to_string())
                        .or_insert_with(|| Material {
                            name: name.to_string(),
                            ..Material::default_steel_kip_in()
                        });
                    if let Some(t) = get_str(&row, "Type") {
                        mat.mat_type = parse_mat_type(t);
                    }
                    if let Some(c) = get_str(&row, "Color") {
                        mat.color = c.to_string();
                    }
                }
            }
            "MATERIAL PROPERTIES 02 - BASIC MECHANICAL PROPERTIES" => {
                if let Some(name) = get_str(&row, "Material") {
                    let mat = model
                        .materials
                        .entry(name.to_string())
                        .or_insert_with(|| Material {
                            name: name.to_string(),
                            ..Material::default_steel_kip_in()
                        });
                    if let Some(uw) = get_f64(&row, "UnitWeight") {
                        mat.unit_weight = uw;
                    }
                    if let Some(um) = get_f64(&row, "UnitMass") {
                        mat.unit_mass = um;
                    }
                    if let Some(e1) = get_f64(&row, "E1") {
                        mat.e = e1;
                        if let (Some(e2), Some(e3)) = (get_f64(&row, "E2"), get_f64(&row, "E3")) {
                            mat.e_ortho = Some([e1, e2, e3]);
                        }
                    }
                    if let Some(u12) = get_f64(&row, "U12") {
                        mat.u = u12;
                        if let (Some(u13), Some(u23)) = (get_f64(&row, "U13"), get_f64(&row, "U23"))
                        {
                            mat.u_ortho = Some([u12, u13, u23]);
                        }
                    }
                    if let Some(g12) = get_f64(&row, "G12") {
                        mat.g = g12;
                        if let (Some(g13), Some(g23)) = (get_f64(&row, "G13"), get_f64(&row, "G23"))
                        {
                            mat.g_ortho = Some([g12, g13, g23]);
                        }
                    } else if mat.e > 0.0 {
                        mat.g = mat.e / (2.0 * (1.0 + mat.u));
                    }
                    if let Some(a1) = get_f64(&row, "A1") {
                        mat.a = a1;
                        if let (Some(a2), Some(a3)) = (get_f64(&row, "A2"), get_f64(&row, "A3")) {
                            mat.a_ortho = Some([a1, a2, a3]);
                        }
                    }
                    if let Some(fy) = get_f64(&row, "Fy") {
                        mat.fy = fy;
                    }
                    if let Some(fu) = get_f64(&row, "Fu") {
                        mat.fu = fu;
                    }
                    if let Some(fc) = get_f64(&row, "Fc") {
                        mat.fc = fc;
                    }
                    if let Some(fys) = get_f64(&row, "Fys") {
                        mat.fys = fys;
                    }
                }
            }
            "MATERIAL PROPERTIES 03A - STEEL DATA" => {
                if let Some(name) = get_str(&row, "Material") {
                    if let Some(mat) = model.materials.get_mut(name) {
                        if let Some(fy) = get_f64(&row, "Fy") {
                            mat.fy = fy;
                        }
                        if let Some(fu) = get_f64(&row, "Fu") {
                            mat.fu = fu;
                        }
                    }
                }
            }
            "MATERIAL PROPERTIES 03B - CONCRETE DATA" => {
                if let Some(name) = get_str(&row, "Material") {
                    if let Some(mat) = model.materials.get_mut(name) {
                        if let Some(fc) = get_f64(&row, "Fc") {
                            mat.fc = fc;
                        }
                    }
                }
            }
            "FRAME SECTION PROPERTIES 01 - GENERAL" => {
                if !cleared_fsecs {
                    model.frame_sections.clear();
                    cleared_fsecs = true;
                }
                if let Some(name) = get_str(&row, "SectionName") {
                    let has_explicit_area = get_f64(&row, "Area").is_some();
                    let mut sec = FrameSection {
                        name: name.to_string(),
                        material: get_str(&row, "Material").unwrap_or("A992Fy50").to_string(),
                        prop_type: get_str(&row, "Shape")
                            .map(parse_frame_prop_type)
                            .unwrap_or(FramePropType::General),
                        t3: get_f64(&row, "t3").unwrap_or(0.0),
                        t2: get_f64(&row, "t2").unwrap_or(0.0),
                        tf: get_f64(&row, "tf").unwrap_or(0.0),
                        tw: get_f64(&row, "tw").unwrap_or(0.0),
                        t2b: get_f64(&row, "t2b")
                            .unwrap_or_else(|| get_f64(&row, "t2").unwrap_or(0.0)),
                        tfb: get_f64(&row, "tfb")
                            .unwrap_or_else(|| get_f64(&row, "tf").unwrap_or(0.0)),
                        dis: get_f64(&row, "dis").unwrap_or(0.0),
                        area: get_f64(&row, "Area").unwrap_or(0.0),
                        torsional_j: get_f64(&row, "TorsConst").unwrap_or(0.0),
                        i33: get_f64(&row, "I33").unwrap_or(0.0),
                        i22: get_f64(&row, "I22").unwrap_or(0.0),
                        i23: get_f64(&row, "I23").unwrap_or(0.0),
                        as2: get_f64(&row, "AS2").unwrap_or(0.0),
                        as3: get_f64(&row, "AS3").unwrap_or(0.0),
                        s33: get_f64(&row, "S33").unwrap_or(0.0),
                        s22: get_f64(&row, "S22").unwrap_or(0.0),
                        z33: get_f64(&row, "Z33").unwrap_or(0.0),
                        z22: get_f64(&row, "Z22").unwrap_or(0.0),
                        r33: get_f64(&row, "R33").unwrap_or(0.0),
                        r22: get_f64(&row, "R22").unwrap_or(0.0),
                        modifiers: [
                            get_f64(&row, "AMod").unwrap_or(1.0),
                            get_f64(&row, "A2Mod").unwrap_or(1.0),
                            get_f64(&row, "A3Mod").unwrap_or(1.0),
                            get_f64(&row, "JMod").unwrap_or(1.0),
                            get_f64(&row, "I2Mod").unwrap_or(1.0),
                            get_f64(&row, "I3Mod").unwrap_or(1.0),
                            get_f64(&row, "MMod").unwrap_or(1.0),
                            get_f64(&row, "WMod").unwrap_or(1.0),
                        ],
                        non_prismatic: Vec::new(),
                        auto_select_list: Vec::new(),
                        concrete_rebar: None,
                        file_name: get_str(&row, "FileName").map(|s| s.to_string()),
                        name_in_file: get_str(&row, "SectInFile").map(|s| s.to_string()),
                    };
                    // If rectangular or circular without explicit Area/I33, compute them
                    if !has_explicit_area && sec.area == 0.0 && sec.t3 > 0.0 {
                        match sec.prop_type {
                            FramePropType::Rectangular => {
                                let d = sec.t3;
                                let b = sec.t2;
                                sec.area = b * d;
                                sec.i33 = b * d.powi(3) / 12.0;
                                sec.i22 = d * b.powi(3) / 12.0;
                                sec.as2 = 5.0 / 6.0 * sec.area;
                                sec.as3 = 5.0 / 6.0 * sec.area;
                                sec.s33 = sec.i33 / (0.5 * d);
                                sec.s22 = sec.i22 / (0.5 * b);
                                sec.z33 = b * d * d / 4.0;
                                sec.z22 = d * b * b / 4.0;
                                sec.r33 = (sec.i33 / sec.area).sqrt();
                                sec.r22 = (sec.i22 / sec.area).sqrt();
                                let a_long = d.max(b);
                                let b_short = d.min(b);
                                let r = b_short / a_long;
                                sec.torsional_j = a_long
                                    * b_short.powi(3)
                                    * (1.0 / 3.0 - 0.21 * r * (1.0 - r.powi(4) / 12.0));
                            }
                            FramePropType::Circle => {
                                let d = sec.t3;
                                let r = 0.5 * d;
                                sec.area = std::f64::consts::PI * r * r;
                                sec.i33 = std::f64::consts::PI * r.powi(4) / 4.0;
                                sec.i22 = sec.i33;
                                sec.torsional_j = 2.0 * sec.i33;
                                sec.as2 = 0.9 * sec.area;
                                sec.as3 = 0.9 * sec.area;
                                sec.s33 = sec.i33 / r;
                                sec.s22 = sec.s33;
                                sec.z33 = d.powi(3) / 6.0;
                                sec.z22 = sec.z33;
                                sec.r33 = 0.5 * r;
                                sec.r22 = 0.5 * r;
                            }
                            _ => {}
                        }
                    }
                    model.frame_sections.insert(name.to_string(), sec);
                }
            }
            "FRAME SECTION PROPERTIES 02 - CONCRETE COLUMN"
            | "FRAME SECTION PROPERTIES 03 - CONCRETE BEAM" => {
                if let Some(name) = get_str(&row, "SectionName") {
                    if let Some(sec) = model.frame_sections.get_mut(name) {
                        let is_col = current_table.contains("COLUMN")
                            || get_bool(&row, "IsColumn").unwrap_or(false);
                        sec.concrete_rebar = Some(ConcreteRebarConfig {
                            is_column: is_col,
                            rebar_mat_long: get_str(&row, "RebarMatL")
                                .unwrap_or("A615Gr60")
                                .to_string(),
                            rebar_mat_confine: get_str(&row, "RebarMatC")
                                .unwrap_or("A615Gr60")
                                .to_string(),
                            cover_top: get_f64(&row, "CoverTop")
                                .or_else(|| get_f64(&row, "Cover"))
                                .unwrap_or(1.5),
                            cover_bot: get_f64(&row, "CoverBot")
                                .or_else(|| get_f64(&row, "Cover"))
                                .unwrap_or(1.5),
                            num_bars_3dir: get_i32(&row, "NumBars3Dir").unwrap_or(4).max(0)
                                as usize,
                            num_bars_2dir: get_i32(&row, "NumBars2Dir").unwrap_or(4).max(0)
                                as usize,
                            total_rebar_area: get_f64(&row, "TotalRebarArea").unwrap_or(0.0),
                        });
                    }
                }
            }
            "FRAME SECTION PROPERTIES 04 - AUTO SELECT" => {
                if let (Some(lname), Some(asec)) =
                    (get_str(&row, "ListName"), get_str(&row, "AutoSection"))
                {
                    let sec = model
                        .frame_sections
                        .entry(lname.to_string())
                        .or_insert_with(|| FrameSection {
                            name: lname.to_string(),
                            prop_type: FramePropType::Auto,
                            ..Default::default()
                        });
                    sec.auto_select_list.push(asec.to_string());
                }
            }
            "FRAME SECTION PROPERTIES 05 - NONPRISMATIC" => {
                if let Some(name) = get_str(&row, "SectionName") {
                    let sec = model
                        .frame_sections
                        .entry(name.to_string())
                        .or_insert_with(|| FrameSection {
                            name: name.to_string(),
                            prop_type: FramePropType::Variable,
                            ..Default::default()
                        });
                    sec.prop_type = FramePropType::Variable;
                    let ei33 = match get_str(&row, "EI33Var")
                        .unwrap_or("Linear")
                        .to_ascii_uppercase()
                        .as_str()
                    {
                        "PARABOLIC" | "2" => 2,
                        "CUBIC" | "3" => 3,
                        s => s.parse::<i32>().unwrap_or(1),
                    };
                    let ei22 = match get_str(&row, "EI22Var")
                        .unwrap_or("Linear")
                        .to_ascii_uppercase()
                        .as_str()
                    {
                        "PARABOLIC" | "2" => 2,
                        "CUBIC" | "3" => 3,
                        s => s.parse::<i32>().unwrap_or(1),
                    };
                    sec.non_prismatic.push(NonPrismaticSegment {
                        start_sec: get_str(&row, "StartSect").unwrap_or("").to_string(),
                        end_sec: get_str(&row, "EndSect").unwrap_or("").to_string(),
                        length: get_f64(&row, "SegLength").unwrap_or(1.0),
                        is_variable_length: !matches!(
                            get_str(&row, "LengthType")
                                .unwrap_or("Variable")
                                .to_ascii_uppercase()
                                .as_str(),
                            "ABSOLUTE"
                        ),
                        ei33_variation: ei33,
                        ei22_variation: ei22,
                    });
                }
            }
            "AREA SECTION PROPERTIES" => {
                if !cleared_asecs {
                    model.area_sections.clear();
                    cleared_asecs = true;
                }
                if let Some(name) = get_str(&row, "Section") {
                    let prop_type = match get_str(&row, "AreaType")
                        .unwrap_or("Shell")
                        .to_ascii_uppercase()
                        .as_str()
                    {
                        "PLANE" => AreaPropType::Plane,
                        "ASOLID" => AreaPropType::ASolid,
                        _ => AreaPropType::Shell,
                    };
                    let t_str = get_str(&row, "Type")
                        .unwrap_or("Shell-Thin")
                        .to_ascii_uppercase();
                    let shell_type = match t_str.as_str() {
                        "SHELL-THICK" | "SHELLTHICK" => ShellType::ShellThick,
                        "PLATE-THIN" | "PLATETHIN" => ShellType::PlateThin,
                        "PLATE-THICK" | "PLATETHICK" => ShellType::PlateThick,
                        "MEMBRANE" => ShellType::Membrane,
                        "SHELL-LAYERED" | "SHELLLAYERED" => ShellType::ShellLayered,
                        _ => ShellType::ShellThin,
                    };
                    let plane_type = if t_str.contains("STRAIN") {
                        PlaneType::PlaneStrain
                    } else {
                        PlaneType::PlaneStress
                    };
                    let tm = get_f64(&row, "Thickness").unwrap_or(6.0);
                    let tb = get_f64(&row, "BendThick").unwrap_or(tm);
                    model.area_sections.insert(
                        name.to_string(),
                        AreaSection {
                            name: name.to_string(),
                            material: get_str(&row, "Material").unwrap_or("4000Psi").to_string(),
                            prop_type,
                            shell_type,
                            plane_type,
                            mat_angle_deg: get_f64(&row, "MatAngle").unwrap_or(0.0),
                            thickness_membrane: tm,
                            thickness_bending: tb,
                            incompatible_modes: get_bool(&row, "InComp").unwrap_or(true),
                            drilling_dof: get_bool(&row, "DrillDOF").unwrap_or(true),
                            modifiers: [
                                get_f64(&row, "F11Mod").unwrap_or(1.0),
                                get_f64(&row, "F22Mod").unwrap_or(1.0),
                                get_f64(&row, "F12Mod").unwrap_or(1.0),
                                get_f64(&row, "M11Mod").unwrap_or(1.0),
                                get_f64(&row, "M22Mod").unwrap_or(1.0),
                                get_f64(&row, "M12Mod").unwrap_or(1.0),
                                get_f64(&row, "V13Mod").unwrap_or(1.0),
                                get_f64(&row, "V23Mod").unwrap_or(1.0),
                                get_f64(&row, "MMod").unwrap_or(1.0),
                                get_f64(&row, "WMod").unwrap_or(1.0),
                            ],
                        },
                    );
                }
            }
            "SOLID PROPERTY DEFINITIONS" => {
                if let Some(name) = get_str(&row, "SolidProp") {
                    model.solid_sections.insert(
                        name.to_string(),
                        SolidSection {
                            name: name.to_string(),
                            material: get_str(&row, "Material").unwrap_or("4000Psi").to_string(),
                            mat_angle_deg: [
                                get_f64(&row, "MatAngleA").unwrap_or(0.0),
                                get_f64(&row, "MatAngleB").unwrap_or(0.0),
                                get_f64(&row, "MatAngleC").unwrap_or(0.0),
                            ],
                            incompatible_modes: get_bool(&row, "InComp").unwrap_or(true),
                            modifiers: [
                                get_f64(&row, "S11Mod").unwrap_or(1.0),
                                get_f64(&row, "S22Mod").unwrap_or(1.0),
                                get_f64(&row, "S33Mod").unwrap_or(1.0),
                                get_f64(&row, "S12Mod").unwrap_or(1.0),
                                get_f64(&row, "S13Mod").unwrap_or(1.0),
                                get_f64(&row, "S23Mod").unwrap_or(1.0),
                                get_f64(&row, "MMod").unwrap_or(1.0),
                                get_f64(&row, "WMod").unwrap_or(1.0),
                            ],
                        },
                    );
                }
            }
            "LINK PROPERTY DEFINITIONS 01 - GENERAL" => {
                if let Some(name) = get_str(&row, "Link") {
                    let ltype = match get_str(&row, "LinkType")
                        .unwrap_or("Linear")
                        .to_ascii_uppercase()
                        .as_str()
                    {
                        "DAMPER" => LinkPropType::Damper,
                        "GAP" => LinkPropType::Gap,
                        "HOOK" => LinkPropType::Hook,
                        "PLASTIC (WEN)" | "PLASTICWEN" => LinkPropType::PlasticWen,
                        "RUBBER ISOLATOR" | "RUBBERISOLATOR" => LinkPropType::RubberIsolator,
                        "FRICTION ISOLATOR" | "FRICTIONISOLATOR" => LinkPropType::FrictionIsolator,
                        "MULTILINEAR ELASTIC" | "MULTILINEARELASTIC" => {
                            LinkPropType::MultiLinearElastic
                        }
                        "MULTILINEAR PLASTIC" | "MULTILINEARPLASTIC" => {
                            LinkPropType::MultiLinearPlastic
                        }
                        "T/C FRICTION ISOLATOR" | "TCFRICTIONISOLATOR" => {
                            LinkPropType::TCFrictionIsolator
                        }
                        "DAMPER - BILINEAR" | "DAMPERBILINEAR" => LinkPropType::DamperBilinear,
                        "DAMPER - FRICTION SPRING" | "DAMPERFRICTIONSPRING" => {
                            LinkPropType::DamperFrictionSpring
                        }
                        "TRIPLE PENDULUM ISOLATOR" | "TRIPLEPENDULUMISOLATOR" => {
                            LinkPropType::TriplePendulumIsolator
                        }
                        _ => LinkPropType::Linear,
                    };
                    let lp = model
                        .link_props
                        .entry(name.to_string())
                        .or_insert_with(|| LinkProp {
                            name: name.to_string(),
                            dof_active: [false; 6],
                            ke: [0.0; 6],
                            ..Default::default()
                        });
                    lp.link_type = ltype;
                    if let Some(value) = get_str(&row, "RustNonlinearDataComplete") {
                        if !matches!(
                            value.to_ascii_uppercase().as_str(),
                            "YES" | "NO" | "TRUE" | "FALSE" | "1" | "0"
                        ) {
                            return Err(FormatError::ParseS2k(
                                "Invalid RustNonlinearDataComplete boolean".into(),
                            ));
                        }
                    }
                    lp.nonlinear_data_complete = get_bool(&row, "RustNonlinearDataComplete")
                        .unwrap_or(ltype == LinkPropType::Linear);
                    if let Some(m) = get_f64(&row, "TotalMass") {
                        lp.mass = m;
                    }
                    if let Some(w) = get_f64(&row, "TotalWt") {
                        lp.weight = w;
                    }
                    if let Some(dj2) = get_f64(&row, "DJ2") {
                        lp.dj2 = dj2;
                    }
                    if let Some(dj3) = get_f64(&row, "DJ3") {
                        lp.dj3 = dj3;
                    }
                }
            }
            "LINK PROPERTY DEFINITIONS 02 - LINEAR" => {
                if let Some(name) = get_str(&row, "Link") {
                    let lp = model
                        .link_props
                        .entry(name.to_string())
                        .or_insert_with(|| LinkProp {
                            name: name.to_string(),
                            dof_active: [false; 6],
                            ke: [0.0; 6],
                            ..Default::default()
                        });
                    if let Some(dof) = get_str(&row, "DOF") {
                        let idx = match dof.to_ascii_uppercase().as_str() {
                            "U1" => Some(0),
                            "U2" => Some(1),
                            "U3" => Some(2),
                            "R1" => Some(3),
                            "R2" => Some(4),
                            "R3" => Some(5),
                            _ => None,
                        };
                        if let Some(i) = idx {
                            lp.dof_active[i] = get_bool(&row, "Active").unwrap_or(true);
                            lp.dof_fixed[i] = get_bool(&row, "Fixed").unwrap_or(false);
                            if let Some(value) = get_str(&row, "RustNonlinear") {
                                if !matches!(
                                    value.to_ascii_uppercase().as_str(),
                                    "YES" | "NO" | "TRUE" | "FALSE" | "1" | "0"
                                ) {
                                    return Err(FormatError::ParseS2k(
                                        "Invalid RustNonlinear boolean".into(),
                                    ));
                                }
                                lp.dof_nonlinear[i] =
                                    get_bool(&row, "RustNonlinear").unwrap_or(false);
                                if !lp.dof_nonlinear[i]
                                    || (get_f64(&row, "Knl").is_some()
                                        && get_f64(&row, "Param1").is_some())
                                {
                                    explicit_nonlinear_rows
                                        .entry(name.into())
                                        .or_default()
                                        .insert(i);
                                }
                            }
                            lp.ke[i] = get_f64(&row, "Ke").unwrap_or(0.0);
                            lp.ce[i] = get_f64(&row, "Ce").unwrap_or(0.0);
                            if let Some(knl) = get_f64(&row, "Knl") {
                                lp.k_nl[i] = knl;
                            }
                            if let Some(p1) = get_f64(&row, "Param1") {
                                lp.param1[i] = p1;
                            }
                            if let Some(p2) = get_f64(&row, "Param2") {
                                lp.param2[i] = p2;
                            }
                        }
                    }
                }
            }
            "JOINT CONSTRAINT DEFINITIONS" => {
                if let Some(name) = get_str(&row, "Name") {
                    let ctype = match get_str(&row, "Type")
                        .unwrap_or("Body")
                        .to_ascii_uppercase()
                        .as_str()
                    {
                        "DIAPHRAGM" => ConstraintType::Diaphragm,
                        "PLATE" => ConstraintType::Plate,
                        "ROD" => ConstraintType::Rod,
                        "BEAM" => ConstraintType::Beam,
                        "EQUAL" => ConstraintType::Equal,
                        "LOCAL" => ConstraintType::Local,
                        "WELD" => ConstraintType::Weld,
                        "LINE" => ConstraintType::Line,
                        _ => ConstraintType::Body,
                    };
                    let axis = match get_str(&row, "Axis")
                        .unwrap_or("Z")
                        .to_ascii_uppercase()
                        .as_str()
                    {
                        "X" => ConstraintAxis::X,
                        "Y" => ConstraintAxis::Y,
                        "AUTO" | "AUTOAXIS" => ConstraintAxis::AutoAxis,
                        _ => ConstraintAxis::Z,
                    };
                    let mut dofs = [false; 6];
                    for (idx, k) in ["UX", "UY", "UZ", "RX", "RY", "RZ"].iter().enumerate() {
                        dofs[idx] = get_bool(&row, k).unwrap_or(false);
                    }
                    model.constraints.insert(
                        name.to_string(),
                        ConstraintDef {
                            name: name.to_string(),
                            constraint_type: ctype,
                            axis,
                            dofs,
                        },
                    );
                }
            }
            "JOINT COORDINATES" => {
                if let Some(jname) = get_str(&row, "Joint") {
                    let has_global = ["GlobalX", "GlobalY", "GlobalZ"]
                        .iter()
                        .all(|k| get_f64(&row, k).is_some());
                    if !has_global
                        && (!get_str(&row, "CoordSys")
                            .unwrap_or("GLOBAL")
                            .eq_ignore_ascii_case("GLOBAL")
                            || !get_str(&row, "CoordType")
                                .unwrap_or("Cartesian")
                                .eq_ignore_ascii_case("Cartesian"))
                    {
                        return Err(FormatError::ParseS2k(format!("Joint {} needs GlobalX/GlobalY/GlobalZ for non-global Cartesian coordinates", jname)));
                    }
                    let x = get_f64(&row, "GlobalX")
                        .or_else(|| get_f64(&row, "XorR"))
                        .unwrap_or(0.0);
                    let y = get_f64(&row, "GlobalY")
                        .or_else(|| get_f64(&row, "Y"))
                        .unwrap_or(0.0);
                    let z = get_f64(&row, "GlobalZ")
                        .or_else(|| get_f64(&row, "Z"))
                        .unwrap_or(0.0);
                    let pt = model
                        .points
                        .entry(jname.to_string())
                        .or_insert_with(|| PointObj::new(jname, x, y, z));
                    pt.x = x;
                    pt.y = y;
                    pt.z = z;
                    if let Some(sp) = get_bool(&row, "SpecialJt") {
                        pt.special_point = sp;
                    }
                    if let Some(sel) = get_bool(&row, "Selected") {
                        pt.selected = sel;
                    }
                }
            }
            "JOINT RESTRAINT ASSIGNMENTS" => {
                if let Some(jname) = get_str(&row, "Joint") {
                    if let Some(pt) = model.points.get_mut(jname) {
                        for (idx, k) in ["U1", "U2", "U3", "R1", "R2", "R3"].iter().enumerate() {
                            if let Some(b) = get_bool(&row, k) {
                                pt.restraint[idx] = b;
                            }
                        }
                    }
                }
            }
            "JOINT SPRING ASSIGNMENTS 1 - UNCOUPLED" => {
                if let Some(jname) = get_str(&row, "Joint") {
                    if let Some(pt) = model.points.get_mut(jname) {
                        pt.spring_is_local = get_str(&row, "CoordSys")
                            .map(|s| s.eq_ignore_ascii_case("LOCAL"))
                            .unwrap_or(false);
                        for (idx, k) in ["U1", "U2", "U3", "R1", "R2", "R3"].iter().enumerate() {
                            if let Some(v) = get_f64(&row, k) {
                                pt.spring[idx] = v;
                            }
                        }
                    }
                }
            }
            "JOINT SPRING ASSIGNMENTS 2 - COUPLED" => {
                if let Some(jname) = get_str(&row, "Joint") {
                    if let Some(pt) = model.points.get_mut(jname) {
                        pt.spring_is_local = get_str(&row, "CoordSys")
                            .map(|s| s.eq_ignore_ascii_case("LOCAL"))
                            .unwrap_or(false);
                        let keys = [
                            "U1U1", "U1U2", "U2U2", "U1U3", "U2U3", "U3U3", "U1R1", "U2R1", "U3R1",
                            "R1R1", "U1R2", "U2R2", "U3R2", "R1R2", "R2R2", "U1R3", "U2R3", "U3R3",
                            "R1R3", "R2R3", "R3R3",
                        ];
                        let mut c = [0.0; 21];
                        for (idx, k) in keys.iter().enumerate() {
                            c[idx] = get_f64(&row, k).unwrap_or(0.0);
                        }
                        pt.spring_coupled = Some(c);
                    }
                }
            }
            "JOINT ADDED MASS ASSIGNMENTS" => {
                if let Some(jname) = get_str(&row, "Joint") {
                    if let Some(pt) = model.points.get_mut(jname) {
                        for (idx, k) in ["Mass1", "Mass2", "Mass3", "MMI1", "MMI2", "MMI3"]
                            .iter()
                            .enumerate()
                        {
                            if let Some(v) = get_f64(&row, k) {
                                pt.mass[idx] = v;
                            }
                        }
                    }
                }
            }
            "JOINT LOCAL AXES ASSIGNMENTS 1 - STANDARD"
            | "JOINT LOCAL AXES ASSIGNMENTS 1 - TYPICAL" => {
                if let Some(jname) = get_str(&row, "Joint") {
                    if let Some(pt) = model.points.get_mut(jname) {
                        pt.local_axes_deg[0] = get_f64(&row, "AngleA").unwrap_or(0.0);
                        pt.local_axes_deg[1] = get_f64(&row, "AngleB").unwrap_or(0.0);
                        pt.local_axes_deg[2] = get_f64(&row, "AngleC").unwrap_or(0.0);
                    }
                }
            }
            "JOINT LOCAL AXES ASSIGNMENTS 2 - ADVANCED" => {
                if let Some(jname) = get_str(&row, "Joint") {
                    if let Some(pt) = model.points.get_mut(jname) {
                        let m = [
                            [
                                get_f64(&row, "R11").unwrap_or(1.0),
                                get_f64(&row, "R12").unwrap_or(0.0),
                                get_f64(&row, "R13").unwrap_or(0.0),
                            ],
                            [
                                get_f64(&row, "R21").unwrap_or(0.0),
                                get_f64(&row, "R22").unwrap_or(1.0),
                                get_f64(&row, "R23").unwrap_or(0.0),
                            ],
                            [
                                get_f64(&row, "R31").unwrap_or(0.0),
                                get_f64(&row, "R32").unwrap_or(0.0),
                                get_f64(&row, "R33").unwrap_or(1.0),
                            ],
                        ];
                        pt.local_axes_matrix = Some(m);
                    }
                }
            }
            "JOINT CONSTRAINT ASSIGNMENTS" => {
                if let (Some(jname), Some(cname)) =
                    (get_str(&row, "Joint"), get_str(&row, "Constraint"))
                {
                    if let Some(pt) = model.points.get_mut(jname) {
                        pt.constraints.push(cname.to_string());
                    }
                }
            }
            "JOINT PATTERN DEFINITIONS" => {
                // Pattern names are stored on PointObj when assigned
            }
            "JOINT PATTERN ASSIGNMENTS" => {
                if let (Some(jname), Some(pname)) =
                    (get_str(&row, "Joint"), get_str(&row, "PatternName"))
                {
                    if let Some(pt) = model.points.get_mut(jname) {
                        let v = get_f64(&row, "Value").unwrap_or(0.0);
                        pt.pattern_values.insert(pname.to_string(), v);
                    }
                }
            }
            "JOINT LOADS - FORCE" => {
                if let (Some(jname), Some(lpat)) =
                    (get_str(&row, "Joint"), get_str(&row, "LoadPat"))
                {
                    if let Some(pt) = model.points.get_mut(jname) {
                        pt.loads_force.push(PointLoadForce {
                            load_pat: lpat.to_string(),
                            csys: get_str(&row, "CoordSys").unwrap_or("GLOBAL").to_string(),
                            force: [
                                get_f64(&row, "F1").unwrap_or(0.0),
                                get_f64(&row, "F2").unwrap_or(0.0),
                                get_f64(&row, "F3").unwrap_or(0.0),
                                get_f64(&row, "M1").unwrap_or(0.0),
                                get_f64(&row, "M2").unwrap_or(0.0),
                                get_f64(&row, "M3").unwrap_or(0.0),
                            ],
                        });
                    }
                }
            }
            "JOINT LOADS - GROUND DISPLACEMENT" => {
                if let (Some(jname), Some(lpat)) =
                    (get_str(&row, "Joint"), get_str(&row, "LoadPat"))
                {
                    if let Some(pt) = model.points.get_mut(jname) {
                        pt.loads_displ.push(PointLoadDispl {
                            load_pat: lpat.to_string(),
                            csys: get_str(&row, "CoordSys").unwrap_or("GLOBAL").to_string(),
                            displ: [
                                get_f64(&row, "U1").unwrap_or(0.0),
                                get_f64(&row, "U2").unwrap_or(0.0),
                                get_f64(&row, "U3").unwrap_or(0.0),
                                get_f64(&row, "R1").unwrap_or(0.0),
                                get_f64(&row, "R2").unwrap_or(0.0),
                                get_f64(&row, "R3").unwrap_or(0.0),
                            ],
                        });
                    }
                }
            }
            "CONNECTIVITY - FRAME" => {
                if let (Some(fname), Some(ji), Some(jj)) = (
                    get_str(&row, "Frame"),
                    get_str(&row, "JointI"),
                    get_str(&row, "JointJ"),
                ) {
                    let default_sec = model
                        .frame_sections
                        .keys()
                        .next()
                        .cloned()
                        .unwrap_or_else(|| "FSEC1".to_string());
                    let fr = model
                        .frames
                        .entry(fname.to_string())
                        .or_insert_with(|| FrameObj::new(fname, ji, jj, default_sec));
                    if let Some(sel) = get_bool(&row, "Selected") {
                        fr.selected = sel;
                    }
                    if let Some(ms) = get_i32(&row, "MinSta") {
                        fr.min_stations = ms.max(1) as usize;
                    }
                }
            }
            "FRAME SECTION ASSIGNMENTS" => {
                if let Some(fname) = get_str(&row, "Frame") {
                    if let Some(fr) = model.frames.get_mut(fname) {
                        if let Some(sec) = get_str(&row, "AnalSect") {
                            fr.section = sec.to_string();
                        }
                    }
                }
            }
            "FRAME PROPERTY MODIFIERS" => {
                if let Some(fname) = get_str(&row, "Frame") {
                    if let Some(fr) = model.frames.get_mut(fname) {
                        fr.modifiers = [
                            get_f64(&row, "AMod").unwrap_or(1.0),
                            get_f64(&row, "A2Mod").unwrap_or(1.0),
                            get_f64(&row, "A3Mod").unwrap_or(1.0),
                            get_f64(&row, "JMod").unwrap_or(1.0),
                            get_f64(&row, "I2Mod").unwrap_or(1.0),
                            get_f64(&row, "I3Mod").unwrap_or(1.0),
                            get_f64(&row, "MMod").unwrap_or(1.0),
                            get_f64(&row, "WMod").unwrap_or(1.0),
                        ];
                    }
                }
            }
            "FRAME RELEASE ASSIGNMENTS 1 - GENERAL" => {
                if let Some(fname) = get_str(&row, "Frame") {
                    if let Some(fr) = model.frames.get_mut(fname) {
                        for (idx, k) in ["PI", "V2I", "V3I", "TI", "M2I", "M3I"].iter().enumerate()
                        {
                            if let Some(b) = get_bool(&row, k) {
                                fr.release_i[idx] = b;
                            }
                        }
                        for (idx, k) in ["PJ", "V2J", "V3J", "TJ", "M2J", "M3J"].iter().enumerate()
                        {
                            if let Some(b) = get_bool(&row, k) {
                                fr.release_j[idx] = b;
                            }
                        }
                    }
                }
            }
            "FRAME RELEASE ASSIGNMENTS 2 - PARTIAL FIXITY" => {
                if let Some(fname) = get_str(&row, "Frame") {
                    if let Some(fr) = model.frames.get_mut(fname) {
                        for (idx, k) in ["PI", "V2I", "V3I", "TI", "M2I", "M3I"].iter().enumerate()
                        {
                            if let Some(v) = get_f64(&row, k) {
                                fr.start_spring[idx] = v;
                            }
                        }
                        for (idx, k) in ["PJ", "V2J", "V3J", "TJ", "M2J", "M3J"].iter().enumerate()
                        {
                            if let Some(v) = get_f64(&row, k) {
                                fr.end_spring[idx] = v;
                            }
                        }
                    }
                }
            }
            "FRAME OFFSET ALONG LENGTH ASSIGNMENTS" => {
                if let Some(fname) = get_str(&row, "Frame") {
                    if let Some(fr) = model.frames.get_mut(fname) {
                        fr.end_offset_i = get_f64(&row, "LengthI").unwrap_or(0.0);
                        fr.end_offset_j = get_f64(&row, "LengthJ").unwrap_or(0.0);
                        fr.rigid_zone_factor = get_f64(&row, "RZ").unwrap_or(0.0);
                    }
                }
            }
            "FRAME INSERTION POINT ASSIGNMENTS" => {
                if let Some(fname) = get_str(&row, "Frame") {
                    if let Some(fr) = model.frames.get_mut(fname) {
                        if let Some(cp) = get_i32(&row, "CardinalPt") {
                            fr.cardinal_point = cp;
                        }
                        fr.stiff_transform = get_bool(&row, "StiffTrans").unwrap_or(true);
                        fr.joint_offset_is_global = get_str(&row, "CoordSys")
                            .map(|s| !s.eq_ignore_ascii_case("LOCAL"))
                            .unwrap_or(true);
                        fr.joint_offset_i = [
                            get_f64(&row, "EndIOffsetX")
                                .or_else(|| get_f64(&row, "EndIOffset1"))
                                .unwrap_or(0.0),
                            get_f64(&row, "EndIOffsetY")
                                .or_else(|| get_f64(&row, "EndIOffset2"))
                                .unwrap_or(0.0),
                            get_f64(&row, "EndIOffsetZ")
                                .or_else(|| get_f64(&row, "EndIOffset3"))
                                .unwrap_or(0.0),
                        ];
                        fr.joint_offset_j = [
                            get_f64(&row, "EndJOffsetX")
                                .or_else(|| get_f64(&row, "EndJOffset1"))
                                .unwrap_or(0.0),
                            get_f64(&row, "EndJOffsetY")
                                .or_else(|| get_f64(&row, "EndJOffset2"))
                                .unwrap_or(0.0),
                            get_f64(&row, "EndJOffsetZ")
                                .or_else(|| get_f64(&row, "EndJOffset3"))
                                .unwrap_or(0.0),
                        ];
                    }
                }
            }
            "FRAME LOCAL AXES ASSIGNMENTS 1 - STANDARD"
            | "FRAME LOCAL AXES ASSIGNMENTS 1 - TYPICAL" => {
                if let Some(fname) = get_str(&row, "Frame") {
                    if let Some(fr) = model.frames.get_mut(fname) {
                        fr.angle_deg = get_f64(&row, "Angle").unwrap_or(0.0);
                    }
                }
            }
            "FRAME TENSION AND COMPRESSION LIMITS" => {
                if let Some(fname) = get_str(&row, "Frame") {
                    if let Some(fr) = model.frames.get_mut(fname) {
                        let hc = get_bool(&row, "CLimitExists").unwrap_or(false);
                        let cl = get_f64(&row, "CLimit").unwrap_or(0.0);
                        let ht = get_bool(&row, "TLimitExists").unwrap_or(false);
                        let tl = get_f64(&row, "TLimit").unwrap_or(0.0);
                        fr.tc_limits = Some((hc, cl, ht, tl));
                    }
                }
            }
            "FRAME PDELTA FORCE ASSIGNMENTS" => {
                if let Some(fname) = get_str(&row, "Frame") {
                    if let Some(fr) = model.frames.get_mut(fname) {
                        fr.pdelta_force = get_f64(&row, "Force").unwrap_or(0.0);
                    }
                }
            }
            "CABLE SHAPE DATA" => {
                if let Some(fname) = get_str(&row, "Frame") {
                    if let Some(fr) = model.frames.get_mut(fname) {
                        fr.cable_params = Some(CableParams {
                            undeformed_length: get_f64(&row, "UndeformedLength"),
                            target_tension_i: get_f64(&row, "TensionI"),
                            horizontal_tension: get_f64(&row, "HorizTension"),
                            num_segments: get_i32(&row, "NumSegs").unwrap_or(1).max(1) as usize,
                        });
                    }
                }
            }
            "FRAME LOADS - DISTRIBUTED" => {
                if let (Some(fname), Some(lpat)) =
                    (get_str(&row, "Frame"), get_str(&row, "LoadPat"))
                {
                    let d1 =
                        load_relative_distance(&row, &model, fname, "RelDistA", "AbsDistA", 0.0)?;
                    let d2 =
                        load_relative_distance(&row, &model, fname, "RelDistB", "AbsDistB", 1.0)?;
                    if d2 <= d1 {
                        return Err(FormatError::ParseS2k(format!(
                            "Distributed load has reversed or empty span on {}",
                            fname
                        )));
                    }
                    if let Some(fr) = model.frames.get_mut(fname) {
                        let is_moment = get_str(&row, "Type")
                            .map(|s| s.eq_ignore_ascii_case("Moment"))
                            .unwrap_or(false);
                        let my_type =
                            get_i32(&row, "MyType").unwrap_or(if is_moment { 2 } else { 1 });
                        let val1 = if my_type == 2 {
                            get_f64(&row, "MOverLA")
                                .or_else(|| get_f64(&row, "FOverLA"))
                                .unwrap_or(0.0)
                        } else {
                            get_f64(&row, "FOverLA").unwrap_or(0.0)
                        };
                        let val2 = if my_type == 2 {
                            get_f64(&row, "MOverLB")
                                .or_else(|| get_f64(&row, "FOverLB"))
                                .unwrap_or(val1)
                        } else {
                            get_f64(&row, "FOverLB").unwrap_or(val1)
                        };
                        fr.loads_dist.push(FrameLoadDistributed {
                            load_pat: lpat.to_string(),
                            my_type,
                            dir: get_str(&row, "Dir").map(parse_load_dir).unwrap_or(10),
                            csys: get_str(&row, "CoordSys").unwrap_or("GLOBAL").to_string(),
                            rel_dist1: d1,
                            rel_dist2: d2,
                            val1,
                            val2,
                        });
                    }
                }
            }
            "FRAME LOADS - POINT" => {
                if let (Some(fname), Some(lpat)) =
                    (get_str(&row, "Frame"), get_str(&row, "LoadPat"))
                {
                    let distance =
                        load_relative_distance(&row, &model, fname, "RelDist", "AbsDist", 0.5)?;
                    if let Some(fr) = model.frames.get_mut(fname) {
                        let is_moment = get_str(&row, "Type")
                            .map(|s| s.eq_ignore_ascii_case("Moment"))
                            .unwrap_or(false);
                        let my_type =
                            get_i32(&row, "MyType").unwrap_or(if is_moment { 2 } else { 1 });
                        let val = if my_type == 2 {
                            get_f64(&row, "Moment")
                                .or_else(|| get_f64(&row, "Force"))
                                .unwrap_or(0.0)
                        } else {
                            get_f64(&row, "Force").unwrap_or(0.0)
                        };
                        fr.loads_point.push(FrameLoadPoint {
                            load_pat: lpat.to_string(),
                            my_type,
                            dir: get_str(&row, "Dir").map(parse_load_dir).unwrap_or(10),
                            csys: get_str(&row, "CoordSys").unwrap_or("GLOBAL").to_string(),
                            rel_dist: distance,
                            val,
                        });
                    }
                }
            }
            "FRAME LOADS - TEMPERATURE" => {
                if let (Some(fname), Some(lpat)) =
                    (get_str(&row, "Frame"), get_str(&row, "LoadPat"))
                {
                    if let Some(fr) = model.frames.get_mut(fname) {
                        let t_str = get_str(&row, "Type")
                            .unwrap_or("Temperature")
                            .to_ascii_uppercase();
                        let (my_type, val) = if let Some(mt) = get_i32(&row, "MyType") {
                            (
                                mt,
                                get_f64(&row, "Temp")
                                    .or_else(|| get_f64(&row, "TempGrad2"))
                                    .or_else(|| get_f64(&row, "TempGrad3"))
                                    .unwrap_or(0.0),
                            )
                        } else if t_str.contains('2') {
                            (
                                2,
                                get_f64(&row, "TempGrad2")
                                    .or_else(|| get_f64(&row, "Temp"))
                                    .unwrap_or(0.0),
                            )
                        } else if t_str.contains('3') {
                            (
                                3,
                                get_f64(&row, "TempGrad3")
                                    .or_else(|| get_f64(&row, "Temp"))
                                    .unwrap_or(0.0),
                            )
                        } else {
                            (1, get_f64(&row, "Temp").unwrap_or(0.0))
                        };
                        let pat = get_str(&row, "JtPattern").and_then(|s| {
                            if s.eq_ignore_ascii_case("None") || s.is_empty() {
                                None
                            } else {
                                Some(s.to_string())
                            }
                        });
                        fr.loads_temp.push(FrameLoadTemperature {
                            load_pat: lpat.to_string(),
                            my_type,
                            val,
                            pattern_name: pat,
                        });
                    }
                }
            }
            "FRAME LOADS - GRAVITY" => {
                if let (Some(fname), Some(lpat)) =
                    (get_str(&row, "Frame"), get_str(&row, "LoadPat"))
                {
                    if let Some(fr) = model.frames.get_mut(fname) {
                        fr.loads_gravity.insert(
                            lpat.to_string(),
                            [
                                get_f64(&row, "MultiplierX").unwrap_or(0.0),
                                get_f64(&row, "MultiplierY").unwrap_or(0.0),
                                get_f64(&row, "MultiplierZ").unwrap_or(0.0),
                            ],
                        );
                    }
                }
            }
            "CONNECTIVITY - AREA" => {
                if let Some(aname) = get_str(&row, "Area") {
                    let num_j = get_i32(&row, "NumJoints").unwrap_or(4).max(0) as usize;
                    let mut pts = Vec::new();
                    let max_j = num_j.max(9);
                    for k_idx in 1..=max_j {
                        let k = format!("Joint{}", k_idx);
                        if let Some(j) = get_str(&row, &k) {
                            if !j.is_empty() && !j.eq_ignore_ascii_case("None") {
                                pts.push(j.to_string());
                            }
                        }
                    }
                    let default_asec = model
                        .area_sections
                        .keys()
                        .next()
                        .cloned()
                        .unwrap_or_else(|| "ASEC1".to_string());
                    let ar = model
                        .areas
                        .entry(aname.to_string())
                        .or_insert_with(|| AreaObj::new(aname, pts, default_asec));
                    if let Some(sel) = get_bool(&row, "Selected") {
                        ar.selected = sel;
                    }
                    if let Some(ang) = get_f64(&row, "Angle") {
                        ar.angle_deg = ang;
                    }
                    if let (Some(n1), Some(n2)) = (get_i32(&row, "MeshN1"), get_i32(&row, "MeshN2"))
                    {
                        ar.auto_mesh = (n1.max(1) as usize, n2.max(1) as usize);
                    }
                    if let Some(ec) = get_bool(&row, "EdgeConstraint") {
                        ar.edge_constraint = ec;
                    }
                }
            }
            "AREA SECTION ASSIGNMENTS" => {
                if let Some(aname) = get_str(&row, "Area") {
                    if let Some(ar) = model.areas.get_mut(aname) {
                        if let Some(sec) = get_str(&row, "Section") {
                            ar.section = sec.to_string();
                        }
                    }
                }
            }
            "AREA PROPERTY MODIFIERS" => {
                if let Some(aname) = get_str(&row, "Area") {
                    if let Some(ar) = model.areas.get_mut(aname) {
                        ar.modifiers = [
                            get_f64(&row, "F11Mod").unwrap_or(1.0),
                            get_f64(&row, "F22Mod").unwrap_or(1.0),
                            get_f64(&row, "F12Mod").unwrap_or(1.0),
                            get_f64(&row, "M11Mod").unwrap_or(1.0),
                            get_f64(&row, "M22Mod").unwrap_or(1.0),
                            get_f64(&row, "M12Mod").unwrap_or(1.0),
                            get_f64(&row, "V13Mod").unwrap_or(1.0),
                            get_f64(&row, "V23Mod").unwrap_or(1.0),
                            get_f64(&row, "MMod").unwrap_or(1.0),
                            get_f64(&row, "WMod").unwrap_or(1.0),
                        ];
                    }
                }
            }
            "AREA LOADS - UNIFORM" => {
                if let (Some(aname), Some(lpat)) = (get_str(&row, "Area"), get_str(&row, "LoadPat"))
                {
                    if let Some(ar) = model.areas.get_mut(aname) {
                        ar.loads_uniform.push(AreaLoadUniform {
                            load_pat: lpat.to_string(),
                            csys: get_str(&row, "CoordSys").unwrap_or("GLOBAL").to_string(),
                            dir: get_str(&row, "Dir").map(parse_load_dir).unwrap_or(10),
                            val: get_f64(&row, "UnifLoad").unwrap_or(0.0),
                        });
                    }
                }
            }
            "AREA LOADS - SURFACE PRESSURE" => {
                if let (Some(aname), Some(lpat)) = (get_str(&row, "Area"), get_str(&row, "LoadPat"))
                {
                    if let Some(ar) = model.areas.get_mut(aname) {
                        let face = match get_str(&row, "Face")
                            .unwrap_or("Bottom")
                            .to_ascii_uppercase()
                            .as_str()
                        {
                            "BOTTOM" | "-1" => -1,
                            "TOP" | "-2" => -2,
                            s => s
                                .strip_prefix("EDGE")
                                .unwrap_or(s)
                                .trim()
                                .parse::<i32>()
                                .unwrap_or(-1),
                        };
                        let pat = get_str(&row, "JtPattern").and_then(|s| {
                            if s.eq_ignore_ascii_case("None") || s.is_empty() {
                                None
                            } else {
                                Some(s.to_string())
                            }
                        });
                        ar.loads_pressure.push(AreaLoadSurfacePressure {
                            load_pat: lpat.to_string(),
                            face,
                            val: get_f64(&row, "Pressure").unwrap_or(0.0),
                            pattern_name: pat,
                        });
                    }
                }
            }
            "AREA LOADS - TEMPERATURE" => {
                if let (Some(aname), Some(lpat)) = (get_str(&row, "Area"), get_str(&row, "LoadPat"))
                {
                    if let Some(ar) = model.areas.get_mut(aname) {
                        let is_grad = get_str(&row, "Type")
                            .map(|s| s.to_ascii_uppercase().contains("GRAD"))
                            .unwrap_or(false);
                        let (my_type, val) = if let Some(mt) = get_i32(&row, "MyType") {
                            (
                                mt,
                                get_f64(&row, "Temp")
                                    .or_else(|| get_f64(&row, "TempGrad3"))
                                    .unwrap_or(0.0),
                            )
                        } else if is_grad {
                            (
                                2,
                                get_f64(&row, "TempGrad3")
                                    .or_else(|| get_f64(&row, "Temp"))
                                    .unwrap_or(0.0),
                            )
                        } else {
                            (1, get_f64(&row, "Temp").unwrap_or(0.0))
                        };
                        let pat = get_str(&row, "JtPattern").and_then(|s| {
                            if s.eq_ignore_ascii_case("None") || s.is_empty() {
                                None
                            } else {
                                Some(s.to_string())
                            }
                        });
                        ar.loads_temp.push(AreaLoadTemperature {
                            load_pat: lpat.to_string(),
                            my_type,
                            val,
                            pattern_name: pat,
                        });
                    }
                }
            }
            "CONNECTIVITY - SOLID" => {
                if let Some(sname) = get_str(&row, "Solid") {
                    let mut pts = [
                        String::new(),
                        String::new(),
                        String::new(),
                        String::new(),
                        String::new(),
                        String::new(),
                        String::new(),
                        String::new(),
                    ];
                    for (idx, p) in pts.iter_mut().enumerate() {
                        let k = format!("Joint{}", idx + 1);
                        if let Some(j) = get_str(&row, &k) {
                            *p = j.to_string();
                        }
                    }
                    let sec = get_str(&row, "SolidProp").unwrap_or("SSEC1").to_string();
                    let sel = get_bool(&row, "Selected").unwrap_or(false);
                    model.solids.insert(
                        sname.to_string(),
                        SolidObj {
                            name: sname.to_string(),
                            points: pts,
                            section: sec,
                            loads_surface: Vec::new(),
                            loads_temp: BTreeMap::new(),
                            selected: sel,
                        },
                    );
                }
            }
            "SOLID LOADS - SURFACE PRESSURE" => {
                if let (Some(sname), Some(lpat)) =
                    (get_str(&row, "Solid"), get_str(&row, "LoadPat"))
                {
                    if let Some(sol) = model.solids.get_mut(sname) {
                        sol.loads_surface.push(SolidSurfaceLoad {
                            load_pat: lpat.to_string(),
                            face: get_i32(&row, "Face").unwrap_or(1),
                            val: get_f64(&row, "Pressure").unwrap_or(0.0),
                        });
                    }
                }
            }
            "SOLID LOADS - TEMPERATURE" => {
                if let (Some(sname), Some(lpat)) =
                    (get_str(&row, "Solid"), get_str(&row, "LoadPat"))
                {
                    if let Some(sol) = model.solids.get_mut(sname) {
                        sol.loads_temp
                            .insert(lpat.to_string(), get_f64(&row, "Temp").unwrap_or(0.0));
                    }
                }
            }
            "CONNECTIVITY - LINK" => {
                if let (Some(lname), Some(ji)) = (get_str(&row, "Link"), get_str(&row, "JointI")) {
                    let jj = get_str(&row, "JointJ").and_then(|s| {
                        if s.is_empty() || s.eq_ignore_ascii_case("None") {
                            None
                        } else {
                            Some(s.to_string())
                        }
                    });
                    let prop = get_str(&row, "LinkProp")
                        .map(|s| s.to_string())
                        .or_else(|| model.link_props.keys().next().cloned())
                        .unwrap_or_else(|| "LIN1".to_string());
                    let angle_deg = get_f64(&row, "Angle").unwrap_or(0.0);
                    let selected = get_bool(&row, "Selected").unwrap_or(false);
                    model.links.insert(
                        lname.to_string(),
                        LinkObj {
                            name: lname.to_string(),
                            point_i: ji.to_string(),
                            point_j: jj,
                            prop,
                            angle_deg,
                            selected,
                        },
                    );
                }
            }
            "LINK PROPERTY ASSIGNMENTS" => {
                if let (Some(lname), Some(prop)) =
                    (get_str(&row, "Link"), get_str(&row, "LinkProp"))
                {
                    if let Some(lk) = model.links.get_mut(lname) {
                        lk.prop = prop.to_string();
                    }
                }
            }
            "FUNCTION - RESPONSE SPECTRUM - USER" | "FUNCTION - TIME HISTORY - USER" => {
                if let Some(fname) = get_str(&row, "Name") {
                    let is_th = current_table.contains("TIME HISTORY")
                        || get_bool(&row, "IsTimeHistory").unwrap_or(false);
                    let damp = get_f64(&row, "DampRatio").unwrap_or(0.05);
                    let func = model.functions.entry(fname.to_string()).or_insert_with(|| {
                        DynamicFunction {
                            name: fname.to_string(),
                            is_time_history: is_th,
                            damping_ratio: damp,
                            points: Vec::new(),
                        }
                    });
                    func.is_time_history = is_th;
                    if get_f64(&row, "DampRatio").is_some() {
                        func.damping_ratio = damp;
                    }
                    if let (Some(x), Some(y)) = (
                        get_f64(&row, "Period").or_else(|| get_f64(&row, "Time")),
                        get_f64(&row, "Accel").or_else(|| get_f64(&row, "Value")),
                    ) {
                        func.points.push((x, y));
                    }
                }
            }
            "LOAD PATTERN DEFINITIONS" => {
                if !cleared_lpats {
                    model.load_patterns.clear();
                    cleared_lpats = true;
                }
                if let Some(name) = get_str(&row, "LoadPat") {
                    model.load_patterns.insert(
                        name.to_string(),
                        LoadPattern {
                            name: name.to_string(),
                            pat_type: get_str(&row, "DesignType")
                                .map(parse_load_pat_type)
                                .unwrap_or(LoadPatternType::Dead),
                            self_wt_multiplier: get_f64(&row, "SelfWtMult").unwrap_or(0.0),
                        },
                    );
                }
            }
            "LOAD CASE DEFINITIONS" => {
                if !cleared_lcases {
                    model.load_cases.clear();
                    cleared_lcases = true;
                }
                if let Some(name) = get_str(&row, "Case") {
                    let ctype = match get_str(&row, "Type")
                        .unwrap_or("LinStatic")
                        .to_ascii_uppercase()
                        .as_str()
                    {
                        "NONSTATIC" | "NONLINEAR STATIC" => LoadCaseType::NonlinearStatic,
                        "LINMODAL" | "MODAL" => LoadCaseType::Modal,
                        "LINRESPSPEC" | "RESPONSE SPECTRUM" => LoadCaseType::ResponseSpectrum,
                        "LINHIST" => LoadCaseType::LinearHistory,
                        "NONHIST" => LoadCaseType::NonlinearHistory,
                        "LINDYNAMIC" => LoadCaseType::LinearDynamic,
                        "NONDYNAMIC" => LoadCaseType::NonlinearDynamic,
                        "LINBUCKLING" | "BUCKLING" => LoadCaseType::Buckling,
                        "MOVE" | "MOVING LOAD" => LoadCaseType::MovingLoad,
                        "STEADYSTATE" => LoadCaseType::SteadyState,
                        "PSD" => LoadCaseType::PowerSpectralDensity,
                        "LINSTATICMULTISTEP" => LoadCaseType::LinearStaticMultiStep,
                        "HYPERSTATIC" => LoadCaseType::HyperStatic,
                        _ => LoadCaseType::LinearStatic,
                    };
                    let mut lc = LoadCase::linear_static(name, name, 1.0);
                    lc.case_type = ctype;
                    lc.loads.clear();
                    lc.run_flag = get_bool(&row, "RunCase").unwrap_or(true);
                    if let Some(ic) = get_str(&row, "InitialCond") {
                        if !ic.eq_ignore_ascii_case("Zero") && !ic.eq_ignore_ascii_case("None") {
                            lc.initial_case = Some(ic.to_string());
                        }
                    }
                    if let Some(ms) = get_str(&row, "ModeType") {
                        lc.modal_sub_type = if ms.eq_ignore_ascii_case("Ritz") {
                            ModalSubType::Ritz
                        } else {
                            ModalSubType::Eigen
                        };
                    }
                    if let Some(mx) = get_i32(&row, "MaxNumModes") {
                        lc.max_modes = mx.max(0) as usize;
                    }
                    if let Some(mn) = get_i32(&row, "MinNumModes") {
                        lc.min_modes = mn.max(0) as usize;
                    }
                    if let Some(sh) = get_f64(&row, "EigenShift") {
                        lc.shift_freq_hz = sh;
                    }
                    if let Some(nlg) = get_str(&row, "GeomNonLin") {
                        lc.nl_geom = match nlg.to_ascii_uppercase().as_str() {
                            "PDELTA" => NLGeomType::PDelta,
                            "LARGEDISPLACEMENTS" | "LARGE DISPLACEMENTS" => {
                                NLGeomType::LargeDisplacements
                            }
                            _ => NLGeomType::None,
                        };
                    }
                    if let Some(mi) = get_i32(&row, "MaxIter") {
                        lc.max_iterations = mi.max(0) as usize;
                    }
                    if let Some(ct) = get_f64(&row, "ConvTol") {
                        lc.convergence_tol = ct;
                    }
                    if let Some(mc) = get_str(&row, "ModalCombo") {
                        lc.modal_combo = match mc.to_ascii_uppercase().as_str() {
                            "SRSS" => ModalComboMethod::Srss,
                            "ABS" | "ABSSUM" => ModalComboMethod::AbsSum,
                            _ => ModalComboMethod::Cqc,
                        };
                    }
                    if let Some(md) = get_f64(&row, "ConstDamp") {
                        lc.modal_damping = md;
                    }
                    if let Some(nts) = get_i32(&row, "NumSteps") {
                        lc.num_time_steps = nts.max(0) as usize;
                    }
                    if let Some(dt) = get_f64(&row, "StepSize") {
                        lc.time_step_dt = dt;
                    }
                    if let Some(ti) = get_str(&row, "TimeInteg") {
                        lc.time_integration = match ti.to_ascii_uppercase().as_str() {
                            "NEWMARKLINEAR" => TimeIntegrationMethod::NewmarkLinear,
                            "HHT" | "HILBERHUGHESTAYLOR" => {
                                TimeIntegrationMethod::HilberHughesTaylor
                            }
                            "WILSONTHETA" => TimeIntegrationMethod::WilsonTheta,
                            _ => TimeIntegrationMethod::NewmarkAverage,
                        };
                    }
                    if let Some(theta) = get_f64(&row, "WilsonTheta") {
                        lc.wilson_theta = theta;
                    }
                    if let Some(ha) = get_f64(&row, "HHTAlpha") {
                        lc.hht_alpha = ha;
                    }
                    if let (Some(rm), Some(rk)) = (
                        get_f64(&row, "RayleighMass"),
                        get_f64(&row, "RayleighStiff"),
                    ) {
                        lc.rayleigh_damping = (rm, rk);
                    }
                    if let Some(nbm) = get_i32(&row, "NumBucklingModes") {
                        lc.num_buckling_modes = nbm.max(0) as usize;
                    }
                    model.load_cases.insert(name.to_string(), lc);
                }
            }
            "CASE - STATIC 1 - LOAD ASSIGNMENTS" => {
                if let (Some(cname), Some(lname)) =
                    (get_str(&row, "Case"), get_str(&row, "LoadName"))
                {
                    let lc = model
                        .load_cases
                        .entry(cname.to_string())
                        .or_insert_with(|| {
                            let mut c = LoadCase::linear_static(cname, lname, 1.0);
                            c.loads.clear();
                            c
                        });
                    lc.loads.push(LoadCaseAssignment {
                        load_name: lname.to_string(),
                        scale_factor: get_f64(&row, "LoadSF").unwrap_or(1.0),
                    });
                }
            }
            "CASE - MODAL 1 - GENERAL" => {
                if let Some(cname) = get_str(&row, "Case") {
                    let lc = model
                        .load_cases
                        .entry(cname.to_string())
                        .or_insert_with(|| LoadCase::modal_eigen(cname, 12));
                    lc.case_type = LoadCaseType::Modal;
                    lc.modal_sub_type = if get_str(&row, "ModeType")
                        .map(|s| s.eq_ignore_ascii_case("Ritz"))
                        .unwrap_or(false)
                    {
                        ModalSubType::Ritz
                    } else {
                        ModalSubType::Eigen
                    };
                    if let Some(mx) = get_i32(&row, "MaxNumModes") {
                        lc.max_modes = mx.max(1) as usize;
                    }
                    if let Some(mn) = get_i32(&row, "MinNumModes") {
                        lc.min_modes = mn.max(1) as usize;
                    }
                    if let Some(sh) = get_f64(&row, "EigenShift") {
                        lc.shift_freq_hz = sh;
                    }
                }
            }
            "CASE - RESPONSE SPECTRUM 1 - GENERAL" => {
                if let Some(cname) = get_str(&row, "Case") {
                    if let Some(lc) = model.load_cases.get_mut(cname) {
                        lc.case_type = LoadCaseType::ResponseSpectrum;
                        if let Some(mc) = get_str(&row, "ModalCombo") {
                            lc.modal_combo = match mc.to_ascii_uppercase().as_str() {
                                "SRSS" => ModalComboMethod::Srss,
                                "ABS" | "ABSSUM" => ModalComboMethod::AbsSum,
                                _ => ModalComboMethod::Cqc,
                            };
                        }
                        if let Some(d) = get_f64(&row, "ConstDamp") {
                            lc.modal_damping = d;
                        }
                    }
                }
            }
            "CASE - RESPONSE SPECTRUM 2 - LOAD ASSIGNMENTS" => {
                if let (Some(cname), Some(func)) =
                    (get_str(&row, "Case"), get_str(&row, "Function"))
                {
                    if let Some(lc) = model.load_cases.get_mut(cname) {
                        lc.spectrum_loads.push((
                            func.to_string(),
                            get_str(&row, "CoordDir").unwrap_or("U1").to_string(),
                            get_f64(&row, "TransAccSF")
                                .or_else(|| get_f64(&row, "SF"))
                                .unwrap_or(1.0),
                        ));
                    }
                }
            }
            "CASE - TIME HISTORY 2 - LOAD ASSIGNMENTS" => {
                if let (Some(cname), Some(func), Some(lname)) = (
                    get_str(&row, "Case"),
                    get_str(&row, "Function"),
                    get_str(&row, "LoadName"),
                ) {
                    if let Some(lc) = model.load_cases.get_mut(cname) {
                        lc.history_loads.push((
                            func.to_string(),
                            lname.to_string(),
                            get_f64(&row, "SF").unwrap_or(1.0),
                        ));
                    }
                }
            }
            "COMBINATION DEFINITIONS" => {
                if let Some(cname) = get_str(&row, "ComboName") {
                    let combo = model
                        .load_combos
                        .entry(cname.to_string())
                        .or_insert_with(|| LoadCombo {
                            name: cname.to_string(),
                            combo_type: ComboType::LinearAdd,
                            cases: Vec::new(),
                        });
                    if let Some(ct) = get_str(&row, "ComboType") {
                        combo.combo_type = match ct.to_ascii_uppercase().as_str() {
                            "ENVELOPE" | "ENV" => ComboType::Envelope,
                            "ABS ADD" | "ABS" => ComboType::AbsAdd,
                            "SRSS" => ComboType::Srss,
                            "RANGE ADD" | "RANGE" => ComboType::RangeAdd,
                            _ => ComboType::LinearAdd,
                        };
                    }
                    if let Some(case_name) = get_str(&row, "CaseName") {
                        if !case_name.is_empty() && !case_name.eq_ignore_ascii_case("None") {
                            let sf = get_f64(&row, "ScaleFactor").unwrap_or(1.0);
                            combo.cases.push((case_name.to_string(), sf));
                        }
                    }
                }
            }
            _ => {
                unsupported_tables.insert(current_table.clone());
            }
        }
    }

    if !saw_table {
        return Err(FormatError::ParseS2k(
            "No valid TABLE headers found in .s2k input".into(),
        ));
    }

    // If CASE - STATIC 1 - LOAD ASSIGNMENTS was not present in the file, default LinearStatic cases to matching pattern
    if !saw_case_loads_table {
        for (cname, lc) in &mut model.load_cases {
            if lc.case_type == LoadCaseType::LinearStatic
                && lc.loads.is_empty()
                && model.load_patterns.contains_key(cname)
            {
                lc.loads.push(LoadCaseAssignment {
                    load_name: cname.clone(),
                    scale_factor: 1.0,
                });
            }
        }
    }

    for (name, prop) in &mut model.link_props {
        if prop.link_type != LinkPropType::Linear
            && explicit_nonlinear_rows
                .get(name)
                .is_none_or(|rows| rows.len() != 6)
        {
            prop.nonlinear_data_complete = false;
        }
    }
    let columns = COLUMN_READS.with(|capture| {
        let mut capture = capture.borrow_mut();
        capture.0.clear();
        std::mem::take(&mut capture.1)
    });
    let canonical = write_s2k_canonical(&model);
    let mut warnings = if preserve && input != canonical {
        preservation::import_warnings(input, &canonical, &columns)?
    } else {
        Vec::new()
    };
    for prop in model
        .link_props
        .values()
        .filter(|p| !p.nonlinear_data_complete)
    {
        warnings.push(format!("Link '{}' nonlinear source activation/constitutive data is incomplete; only effective linear analyses are supported until explicit law data is supplied", prop.name));
    }
    if input.contains("RustNonlinear") {
        warnings.push("RustNonlinear/RustNonlinearDataComplete are reconstruction extensions, not verified vendor S2K fields.".into());
    }
    if preserve && input != canonical {
        model.s2k_preservation = Some(sap_model::S2kPreservation {
            source_text: input.into(),
            canonical_baseline: canonical,
        });
    }
    Ok((ImportReport { model, warnings }, columns))
}

/// Serializes a [`SapModel`] into a SAP2000 v19 `.s2k` / `.$2k` text model string.
pub(super) fn write_s2k_canonical(model: &SapModel) -> String {
    let mut out = String::with_capacity(16384);
    out.push_str(&format!(
        "File {}.$2k was saved by StructuraX\n\n",
        model.model_name
    ));

    // 1. PROGRAM CONTROL
    out.push_str("TABLE:  \"PROGRAM CONTROL\"\n");
    out.push_str(&format!(
        "   ProgramName=SAP2000   Version={}   ProgLevel=Advanced   ModelName={} _\n     CurrUnits=\"{}\"   DataBaseUnits=\"{}\"   SteelCode={}   ConcCode={}   IsLocked={}\n\n",
        format_val(&model.program_version),
        format_val(&model.model_name),
        model.present_units.s2k_label(),
        model.database_units.s2k_label(),
        format_val(&model.steel_design_code),
        format_val(&model.concrete_design_code),
        format_bool(model.is_locked),
    ));

    if !model.project_info.is_empty() {
        out.push_str("TABLE:  \"PROJECT INFORMATION\"\n");
        for (k, v) in &model.project_info {
            out.push_str(&format!(
                "   Item={}   Data={}\n",
                format_val(k),
                format_val(v),
            ));
        }
        out.push('\n');
    }

    // 2. ACTIVE DEGREES OF FREEDOM
    out.push_str("TABLE:  \"ACTIVE DEGREES OF FREEDOM\"\n");
    out.push_str(&format!(
        "   UX={}   UY={}   UZ={}   RX={}   RY={}   RZ={}\n\n",
        format_bool(model.active_dofs[0]),
        format_bool(model.active_dofs[1]),
        format_bool(model.active_dofs[2]),
        format_bool(model.active_dofs[3]),
        format_bool(model.active_dofs[4]),
        format_bool(model.active_dofs[5]),
    ));

    // 3. COORDINATE SYSTEMS & GRID LINES
    out.push_str("TABLE:  \"COORDINATE SYSTEMS\"\n");
    for cs in model.coord_systems.values() {
        out.push_str(&format!(
            "   Name={}   Type=Cartesian   X={}   Y={}   Z={}   AboutZ={}   AboutY={}   AboutX={}\n",
            format_val(&cs.name),
            cs.origin[0],
            cs.origin[1],
            cs.origin[2],
            cs.angles_deg[0],
            cs.angles_deg[1],
            cs.angles_deg[2],
        ));
    }
    out.push('\n');

    let has_grids = model.coord_systems.values().any(|cs| !cs.grids.is_empty());
    if has_grids {
        out.push_str("TABLE:  \"GRID LINES\"\n");
        for cs in model.coord_systems.values() {
            for g in &cs.grids {
                out.push_str(&format!(
                    "   CoordSys={}   AxisDir={}   GridID={}   XRYZCoord={}   Visible={}   BubbleLoc={}\n",
                    format_val(&cs.name),
                    format_val(&g.axis),
                    format_val(&g.id),
                    g.coordinate,
                    format_bool(g.visible),
                    format_val(&g.bubble_loc),
                ));
            }
        }
        out.push('\n');
    }

    // 4. MATERIALS
    out.push_str("TABLE:  \"MATERIAL PROPERTIES 01 - GENERAL\"\n");
    for m in model.materials.values() {
        let sym = if m.e_ortho.is_some() {
            "Orthotropic"
        } else {
            "Isotropic"
        };
        out.push_str(&format!(
            "   Material={}   Type={}   SymType={}   TempDepend=No   Color={}\n",
            format_val(&m.name),
            mat_type_str(m.mat_type),
            sym,
            format_val(&m.color),
        ));
    }
    out.push('\n');

    if !model.materials.is_empty() {
        out.push_str("TABLE:  \"MATERIAL PROPERTIES 02 - BASIC MECHANICAL PROPERTIES\"\n");
        for m in model.materials.values() {
            let mut line = format!(
                "   Material={}   UnitWeight={}   UnitMass={}   E1={}   G12={}   U12={}   A1={}",
                format_val(&m.name),
                m.unit_weight,
                m.unit_mass,
                m.e,
                m.g,
                m.u,
                m.a,
            );
            if let Some(eo) = m.e_ortho {
                line.push_str(&format!("   E2={}   E3={}", eo[1], eo[2]));
            }
            if let Some(go) = m.g_ortho {
                line.push_str(&format!("   G13={}   G23={}", go[1], go[2]));
            }
            if let Some(uo) = m.u_ortho {
                line.push_str(&format!("   U13={}   U23={}", uo[1], uo[2]));
            }
            if let Some(ao) = m.a_ortho {
                line.push_str(&format!("   A2={}   A3={}", ao[1], ao[2]));
            }
            line.push_str(&format!(
                "   Fy={}   Fu={}   Fc={}   Fys={}\n",
                m.fy, m.fu, m.fc, m.fys
            ));
            out.push_str(&line);
        }
        out.push('\n');

        let steels: Vec<&Material> = model
            .materials
            .values()
            .filter(|m| m.mat_type == MatType::Steel || m.mat_type == MatType::Rebar)
            .collect();
        if !steels.is_empty() {
            out.push_str("TABLE:  \"MATERIAL PROPERTIES 03A - STEEL DATA\"\n");
            for m in steels {
                out.push_str(&format!(
                    "   Material={}   Fy={}   Fu={}   EffFy={}   EffFu={}\n",
                    format_val(&m.name),
                    m.fy,
                    m.fu,
                    m.fy * 1.1,
                    m.fu * 1.1,
                ));
            }
            out.push('\n');
        }

        let concs: Vec<&Material> = model
            .materials
            .values()
            .filter(|m| m.mat_type == MatType::Concrete)
            .collect();
        if !concs.is_empty() {
            out.push_str("TABLE:  \"MATERIAL PROPERTIES 03B - CONCRETE DATA\"\n");
            for m in concs {
                out.push_str(&format!(
                    "   Material={}   Fc={}   eFc={}   LtWtConc=No\n",
                    format_val(&m.name),
                    m.fc,
                    m.fc,
                ));
            }
            out.push('\n');
        }
    }

    // 5. FRAME SECTIONS
    out.push_str("TABLE:  \"FRAME SECTION PROPERTIES 01 - GENERAL\"\n");
    for s in model.frame_sections.values() {
        let mut line = format!(
            "   SectionName={}   Material={}   Shape={}   t3={}   t2={}   tf={}   tw={}   t2b={}   tfb={} _\n     dis={}   Area={}   TorsConst={}   I33={}   I22={}   I23={}   AS2={}   AS3={} _\n     S33={}   S22={}   Z33={}   Z22={}   R33={}   R22={}   AMod={}   A2Mod={}   A3Mod={} _\n     JMod={}   I2Mod={}   I3Mod={}   MMod={}   WMod={}",
            format_val(&s.name),
            format_val(&s.material),
            format_val(frame_prop_type_str(s.prop_type)),
            s.t3,
            s.t2,
            s.tf,
            s.tw,
            s.t2b,
            s.tfb,
            s.dis,
            s.area,
            s.torsional_j,
            s.i33,
            s.i22,
            s.i23,
            s.as2,
            s.as3,
            s.s33,
            s.s22,
            s.z33,
            s.z22,
            s.r33,
            s.r22,
            s.modifiers[0],
            s.modifiers[1],
            s.modifiers[2],
            s.modifiers[3],
            s.modifiers[4],
            s.modifiers[5],
            s.modifiers[6],
            s.modifiers[7],
        );
        if let Some(ref fnm) = s.file_name {
            line.push_str(&format!("   FileName={}", format_val(fnm)));
        }
        if let Some(ref sif) = s.name_in_file {
            line.push_str(&format!("   SectInFile={}", format_val(sif)));
        }
        line.push('\n');
        out.push_str(&line);
    }
    out.push('\n');

    let conc_cols: Vec<&FrameSection> = model
        .frame_sections
        .values()
        .filter(|s| {
            s.concrete_rebar
                .as_ref()
                .map(|r| r.is_column)
                .unwrap_or(false)
        })
        .collect();
    if !conc_cols.is_empty() {
        out.push_str("TABLE:  \"FRAME SECTION PROPERTIES 02 - CONCRETE COLUMN\"\n");
        for s in conc_cols {
            if let Some(ref rb) = s.concrete_rebar {
                out.push_str(&format!(
                    "   SectionName={}   RebarMatL={}   RebarMatC={}   CoverTop={}   CoverBot={}   NumBars3Dir={}   NumBars2Dir={}   TotalRebarArea={}   IsColumn=Yes\n",
                    format_val(&s.name),
                    format_val(&rb.rebar_mat_long),
                    format_val(&rb.rebar_mat_confine),
                    rb.cover_top,
                    rb.cover_bot,
                    rb.num_bars_3dir,
                    rb.num_bars_2dir,
                    rb.total_rebar_area,
                ));
            }
        }
        out.push('\n');
    }

    let conc_beams: Vec<&FrameSection> = model
        .frame_sections
        .values()
        .filter(|s| {
            s.concrete_rebar
                .as_ref()
                .map(|r| !r.is_column)
                .unwrap_or(false)
        })
        .collect();
    if !conc_beams.is_empty() {
        out.push_str("TABLE:  \"FRAME SECTION PROPERTIES 03 - CONCRETE BEAM\"\n");
        for s in conc_beams {
            if let Some(ref rb) = s.concrete_rebar {
                out.push_str(&format!(
                    "   SectionName={}   RebarMatL={}   RebarMatC={}   CoverTop={}   CoverBot={}   NumBars3Dir={}   NumBars2Dir={}   TotalRebarArea={}   IsColumn=No\n",
                    format_val(&s.name),
                    format_val(&rb.rebar_mat_long),
                    format_val(&rb.rebar_mat_confine),
                    rb.cover_top,
                    rb.cover_bot,
                    rb.num_bars_3dir,
                    rb.num_bars_2dir,
                    rb.total_rebar_area,
                ));
            }
        }
        out.push('\n');
    }

    let auto_sects: Vec<&FrameSection> = model
        .frame_sections
        .values()
        .filter(|s| !s.auto_select_list.is_empty())
        .collect();
    if !auto_sects.is_empty() {
        out.push_str("TABLE:  \"FRAME SECTION PROPERTIES 04 - AUTO SELECT\"\n");
        for s in auto_sects {
            for item in &s.auto_select_list {
                out.push_str(&format!(
                    "   ListName={}   AutoSection={}\n",
                    format_val(&s.name),
                    format_val(item),
                ));
            }
        }
        out.push('\n');
    }

    let nonpris: Vec<&FrameSection> = model
        .frame_sections
        .values()
        .filter(|s| !s.non_prismatic.is_empty())
        .collect();
    if !nonpris.is_empty() {
        out.push_str("TABLE:  \"FRAME SECTION PROPERTIES 05 - NONPRISMATIC\"\n");
        for s in nonpris {
            for seg in &s.non_prismatic {
                let ltype = if seg.is_variable_length {
                    "Variable"
                } else {
                    "Absolute"
                };
                out.push_str(&format!(
                    "   SectionName={}   StartSect={}   EndSect={}   LengthType={}   SegLength={}   EI33Var={}   EI22Var={}\n",
                    format_val(&s.name),
                    format_val(&seg.start_sec),
                    format_val(&seg.end_sec),
                    ltype,
                    seg.length,
                    seg.ei33_variation,
                    seg.ei22_variation,
                ));
            }
        }
        out.push('\n');
    }

    // 6. AREA SECTIONS & SOLID SECTIONS
    out.push_str("TABLE:  \"AREA SECTION PROPERTIES\"\n");
    for a in model.area_sections.values() {
        let atype = match a.prop_type {
            AreaPropType::Shell => "Shell",
            AreaPropType::Plane => "Plane",
            AreaPropType::ASolid => "ASolid",
        };
        let stype = match a.shell_type {
            ShellType::ShellThin => "Shell-Thin",
            ShellType::ShellThick => "Shell-Thick",
            ShellType::PlateThin => "Plate-Thin",
            ShellType::PlateThick => "Plate-Thick",
            ShellType::Membrane => "Membrane",
            ShellType::ShellLayered => "Shell-Layered",
        };
        out.push_str(&format!(
            "   Section={}   Material={}   MatAngle={}   AreaType={}   Type={}   DrillDOF={} _\n     Thickness={}   BendThick={}   InComp={}   F11Mod={}   F22Mod={}   F12Mod={} _\n     M11Mod={}   M22Mod={}   M12Mod={}   V13Mod={}   V23Mod={}   MMod={}   WMod={}\n",
            format_val(&a.name),
            format_val(&a.material),
            a.mat_angle_deg,
            atype,
            stype,
            format_bool(a.drilling_dof),
            a.thickness_membrane,
            a.thickness_bending,
            format_bool(a.incompatible_modes),
            a.modifiers[0],
            a.modifiers[1],
            a.modifiers[2],
            a.modifiers[3],
            a.modifiers[4],
            a.modifiers[5],
            a.modifiers[6],
            a.modifiers[7],
            a.modifiers[8],
            a.modifiers[9],
        ));
    }
    out.push('\n');

    if !model.solid_sections.is_empty() {
        out.push_str("TABLE:  \"SOLID PROPERTY DEFINITIONS\"\n");
        for sp in model.solid_sections.values() {
            out.push_str(&format!(
                "   SolidProp={}   Material={}   MatAngleA={}   MatAngleB={}   MatAngleC={}   InComp={} _\n     S11Mod={}   S22Mod={}   S33Mod={}   S12Mod={}   S13Mod={}   S23Mod={}   MMod={}   WMod={}\n",
                format_val(&sp.name),
                format_val(&sp.material),
                sp.mat_angle_deg[0],
                sp.mat_angle_deg[1],
                sp.mat_angle_deg[2],
                format_bool(sp.incompatible_modes),
                sp.modifiers[0],
                sp.modifiers[1],
                sp.modifiers[2],
                sp.modifiers[3],
                sp.modifiers[4],
                sp.modifiers[5],
                sp.modifiers[6],
                sp.modifiers[7],
            ));
        }
        out.push('\n');
    }

    // 7. LINK PROPERTIES & CONSTRAINTS
    if !model.link_props.is_empty() {
        out.push_str("TABLE:  \"LINK PROPERTY DEFINITIONS 01 - GENERAL\"\n");
        for lp in model.link_props.values() {
            let ltype = match lp.link_type {
                LinkPropType::Linear => "Linear",
                LinkPropType::Damper => "Damper",
                LinkPropType::Gap => "Gap",
                LinkPropType::Hook => "Hook",
                LinkPropType::MultiLinearElastic => "MultiLinear Elastic",
                LinkPropType::MultiLinearPlastic => "MultiLinear Plastic",
                LinkPropType::PlasticWen => "Plastic (Wen)",
                LinkPropType::RubberIsolator => "Rubber Isolator",
                LinkPropType::FrictionIsolator => "Friction Isolator",
                LinkPropType::TCFrictionIsolator => "T/C Friction Isolator",
                LinkPropType::DamperBilinear => "Damper - Bilinear",
                LinkPropType::DamperFrictionSpring => "Damper - Friction Spring",
                LinkPropType::TriplePendulumIsolator => "Triple Pendulum Isolator",
            };
            out.push_str(&format!(
                "   Link={}   LinkType={}   TotalMass={}   TotalWt={}   DJ2={}   DJ3={}   RustNonlinearDataComplete={}\n",
                format_val(&lp.name),
                format_val(ltype),
                lp.mass,
                lp.weight,
                lp.dj2,
                lp.dj3,
                format_bool(lp.nonlinear_data_complete),
            ));
        }
        out.push('\n');

        out.push_str("TABLE:  \"LINK PROPERTY DEFINITIONS 02 - LINEAR\"\n");
        let dof_names = ["U1", "U2", "U3", "R1", "R2", "R3"];
        for lp in model.link_props.values() {
            for i in 0..6 {
                out.push_str(&format!(
                    "   Link={}   DOF={}   Active={}   Fixed={}   RustNonlinear={}   Ke={}   Ce={}   Knl={}   Param1={}   Param2={}\n",
                    format_val(&lp.name),
                    dof_names[i],
                    format_bool(lp.dof_active[i]),
                    format_bool(lp.dof_fixed[i]),
                    format_bool(lp.dof_nonlinear[i]),
                    lp.ke[i],
                    lp.ce[i],
                    lp.k_nl[i],
                    lp.param1[i],
                    lp.param2[i],
                ));
            }
        }
        out.push('\n');
    }

    if !model.constraints.is_empty() {
        out.push_str("TABLE:  \"JOINT CONSTRAINT DEFINITIONS\"\n");
        for c in model.constraints.values() {
            let ctype = match c.constraint_type {
                ConstraintType::Body => "Body",
                ConstraintType::Diaphragm => "Diaphragm",
                ConstraintType::Plate => "Plate",
                ConstraintType::Rod => "Rod",
                ConstraintType::Beam => "Beam",
                ConstraintType::Equal => "Equal",
                ConstraintType::Local => "Local",
                ConstraintType::Weld => "Weld",
                ConstraintType::Line => "Line",
            };
            let ax = match c.axis {
                ConstraintAxis::X => "X",
                ConstraintAxis::Y => "Y",
                ConstraintAxis::Z => "Z",
                ConstraintAxis::AutoAxis => "Auto",
            };
            out.push_str(&format!(
                "   Name={}   Type={}   Axis={}   UX={}   UY={}   UZ={}   RX={}   RY={}   RZ={}\n",
                format_val(&c.name),
                ctype,
                ax,
                format_bool(c.dofs[0]),
                format_bool(c.dofs[1]),
                format_bool(c.dofs[2]),
                format_bool(c.dofs[3]),
                format_bool(c.dofs[4]),
                format_bool(c.dofs[5]),
            ));
        }
        out.push('\n');
    }

    // 8. JOINT COORDINATES, RESTRAINTS, SPRINGS, MASSES, AXES, CONSTRAINTS, PATTERNS, LOADS
    if !model.points.is_empty() {
        out.push_str("TABLE:  \"JOINT COORDINATES\"\n");
        for p in model.points.values() {
            out.push_str(&format!(
                "   Joint={}   CoordSys=GLOBAL   CoordType=Cartesian   XorR={}   Y={}   Z={}   SpecialJt={}   Selected={}   GlobalX={}   GlobalY={}   GlobalZ={}\n",
                format_val(&p.name),
                p.x,
                p.y,
                p.z,
                format_bool(p.special_point),
                format_bool(p.selected),
                p.x,
                p.y,
                p.z,
            ));
        }
        out.push('\n');

        let restrained: Vec<&PointObj> = model
            .points
            .values()
            .filter(|p| p.restraint.iter().any(|&r| r))
            .collect();
        if !restrained.is_empty() {
            out.push_str("TABLE:  \"JOINT RESTRAINT ASSIGNMENTS\"\n");
            for p in restrained {
                out.push_str(&format!(
                    "   Joint={}   U1={}   U2={}   U3={}   R1={}   R2={}   R3={}\n",
                    format_val(&p.name),
                    format_bool(p.restraint[0]),
                    format_bool(p.restraint[1]),
                    format_bool(p.restraint[2]),
                    format_bool(p.restraint[3]),
                    format_bool(p.restraint[4]),
                    format_bool(p.restraint[5]),
                ));
            }
            out.push('\n');
        }

        let sprung: Vec<&PointObj> = model
            .points
            .values()
            .filter(|p| p.spring.iter().any(|&k| k != 0.0) || p.spring_is_local)
            .collect();
        if !sprung.is_empty() {
            out.push_str("TABLE:  \"JOINT SPRING ASSIGNMENTS 1 - UNCOUPLED\"\n");
            for p in sprung {
                let csys = if p.spring_is_local { "LOCAL" } else { "GLOBAL" };
                out.push_str(&format!(
                    "   Joint={}   CoordSys={}   U1={}   U2={}   U3={}   R1={}   R2={}   R3={}\n",
                    format_val(&p.name),
                    csys,
                    p.spring[0],
                    p.spring[1],
                    p.spring[2],
                    p.spring[3],
                    p.spring[4],
                    p.spring[5],
                ));
            }
            out.push('\n');
        }

        let coupled_sprung: Vec<&PointObj> = model
            .points
            .values()
            .filter(|p| p.spring_coupled.is_some())
            .collect();
        if !coupled_sprung.is_empty() {
            out.push_str("TABLE:  \"JOINT SPRING ASSIGNMENTS 2 - COUPLED\"\n");
            for p in coupled_sprung {
                if let Some(ref k) = p.spring_coupled {
                    let csys = if p.spring_is_local { "LOCAL" } else { "GLOBAL" };
                    out.push_str(&format!(
                        "   Joint={}   CoordSys={}   U1U1={}   U1U2={}   U2U2={}   U1U3={}   U2U3={}   U3U3={} _\n     U1R1={}   U2R1={}   U3R1={}   R1R1={}   U1R2={}   U2R2={}   U3R2={}   R1R2={}   R2R2={} _\n     U1R3={}   U2R3={}   U3R3={}   R1R3={}   R2R3={}   R3R3={}\n",
                        format_val(&p.name),
                        csys,
                        k[0], k[1], k[2], k[3], k[4], k[5],
                        k[6], k[7], k[8], k[9], k[10], k[11], k[12], k[13], k[14],
                        k[15], k[16], k[17], k[18], k[19], k[20],
                    ));
                }
            }
            out.push('\n');
        }

        let massed: Vec<&PointObj> = model
            .points
            .values()
            .filter(|p| p.mass.iter().any(|&m| m != 0.0))
            .collect();
        if !massed.is_empty() {
            out.push_str("TABLE:  \"JOINT ADDED MASS ASSIGNMENTS\"\n");
            for p in massed {
                out.push_str(&format!(
                    "   Joint={}   CoordSys=GLOBAL   Mass1={}   Mass2={}   Mass3={}   MMI1={}   MMI2={}   MMI3={}\n",
                    format_val(&p.name),
                    p.mass[0],
                    p.mass[1],
                    p.mass[2],
                    p.mass[3],
                    p.mass[4],
                    p.mass[5],
                ));
            }
            out.push('\n');
        }

        let rot_pts: Vec<&PointObj> = model
            .points
            .values()
            .filter(|p| p.local_axes_deg.iter().any(|&a| a != 0.0))
            .collect();
        if !rot_pts.is_empty() {
            out.push_str("TABLE:  \"JOINT LOCAL AXES ASSIGNMENTS 1 - STANDARD\"\n");
            for p in rot_pts {
                out.push_str(&format!(
                    "   Joint={}   AngleA={}   AngleB={}   AngleC={}\n",
                    format_val(&p.name),
                    p.local_axes_deg[0],
                    p.local_axes_deg[1],
                    p.local_axes_deg[2],
                ));
            }
            out.push('\n');
        }

        let adv_pts: Vec<&PointObj> = model
            .points
            .values()
            .filter(|p| p.local_axes_matrix.is_some())
            .collect();
        if !adv_pts.is_empty() {
            out.push_str("TABLE:  \"JOINT LOCAL AXES ASSIGNMENTS 2 - ADVANCED\"\n");
            for p in adv_pts {
                if let Some(ref m) = p.local_axes_matrix {
                    out.push_str(&format!(
                        "   Joint={}   R11={}   R12={}   R13={}   R21={}   R22={}   R23={}   R31={}   R32={}   R33={}\n",
                        format_val(&p.name),
                        m[0][0], m[0][1], m[0][2],
                        m[1][0], m[1][1], m[1][2],
                        m[2][0], m[2][1], m[2][2],
                    ));
                }
            }
            out.push('\n');
        }

        let has_jconst = model.points.values().any(|p| !p.constraints.is_empty());
        if has_jconst {
            out.push_str("TABLE:  \"JOINT CONSTRAINT ASSIGNMENTS\"\n");
            for p in model.points.values() {
                for cname in &p.constraints {
                    out.push_str(&format!(
                        "   Joint={}   Constraint={}\n",
                        format_val(&p.name),
                        format_val(cname),
                    ));
                }
            }
            out.push('\n');
        }

        let has_jpat = model.points.values().any(|p| !p.pattern_values.is_empty());
        if has_jpat {
            out.push_str("TABLE:  \"JOINT PATTERN ASSIGNMENTS\"\n");
            for p in model.points.values() {
                for (pat, val) in &p.pattern_values {
                    out.push_str(&format!(
                        "   Joint={}   PatternName={}   Value={}\n",
                        format_val(&p.name),
                        format_val(pat),
                        val,
                    ));
                }
            }
            out.push('\n');
        }

        let has_jforce = model.points.values().any(|p| !p.loads_force.is_empty());
        if has_jforce {
            out.push_str("TABLE:  \"JOINT LOADS - FORCE\"\n");
            for p in model.points.values() {
                for lf in &p.loads_force {
                    out.push_str(&format!(
                        "   Joint={}   LoadPat={}   CoordSys={}   F1={}   F2={}   F3={}   M1={}   M2={}   M3={}\n",
                        format_val(&p.name),
                        format_val(&lf.load_pat),
                        format_val(&lf.csys),
                        lf.force[0],
                        lf.force[1],
                        lf.force[2],
                        lf.force[3],
                        lf.force[4],
                        lf.force[5],
                    ));
                }
            }
            out.push('\n');
        }

        let has_jdispl = model.points.values().any(|p| !p.loads_displ.is_empty());
        if has_jdispl {
            out.push_str("TABLE:  \"JOINT LOADS - GROUND DISPLACEMENT\"\n");
            for p in model.points.values() {
                for ld in &p.loads_displ {
                    out.push_str(&format!(
                        "   Joint={}   LoadPat={}   CoordSys={}   U1={}   U2={}   U3={}   R1={}   R2={}   R3={}\n",
                        format_val(&p.name),
                        format_val(&ld.load_pat),
                        format_val(&ld.csys),
                        ld.displ[0],
                        ld.displ[1],
                        ld.displ[2],
                        ld.displ[3],
                        ld.displ[4],
                        ld.displ[5],
                    ));
                }
            }
            out.push('\n');
        }
    }

    // 9. FRAMES & FRAME ASSIGNMENTS/LOADS
    if !model.frames.is_empty() {
        out.push_str("TABLE:  \"CONNECTIVITY - FRAME\"\n");
        for f in model.frames.values() {
            out.push_str(&format!(
                "   Frame={}   JointI={}   JointJ={}   IsCurved=No   Selected={}   MinSta={}\n",
                format_val(&f.name),
                format_val(&f.point_i),
                format_val(&f.point_j),
                format_bool(f.selected),
                f.min_stations,
            ));
        }
        out.push('\n');

        out.push_str("TABLE:  \"FRAME SECTION ASSIGNMENTS\"\n");
        for f in model.frames.values() {
            out.push_str(&format!(
                "   Frame={}   AnalSect={}   DesignSect={}   MatProp=Default\n",
                format_val(&f.name),
                format_val(&f.section),
                format_val(&f.section),
            ));
        }
        out.push('\n');

        let rot_frames: Vec<&FrameObj> = model
            .frames
            .values()
            .filter(|f| f.angle_deg.abs() > 1e-12)
            .collect();
        if !rot_frames.is_empty() {
            out.push_str("TABLE:  \"FRAME LOCAL AXES ASSIGNMENTS 1 - TYPICAL\"\n");
            for f in rot_frames {
                out.push_str(&format!(
                    "   Frame={}   Angle={}   AdvanceAxes=No\n",
                    format_val(&f.name),
                    f.angle_deg,
                ));
            }
            out.push('\n');
        }

        let mod_frames: Vec<&FrameObj> = model
            .frames
            .values()
            .filter(|f| f.modifiers.iter().any(|&m| (m - 1.0).abs() > 1e-12))
            .collect();
        if !mod_frames.is_empty() {
            out.push_str("TABLE:  \"FRAME PROPERTY MODIFIERS\"\n");
            for f in mod_frames {
                out.push_str(&format!(
                    "   Frame={}   AMod={}   A2Mod={}   A3Mod={}   JMod={}   I2Mod={}   I3Mod={}   MMod={}   WMod={}\n",
                    format_val(&f.name),
                    f.modifiers[0],
                    f.modifiers[1],
                    f.modifiers[2],
                    f.modifiers[3],
                    f.modifiers[4],
                    f.modifiers[5],
                    f.modifiers[6],
                    f.modifiers[7],
                ));
            }
            out.push('\n');
        }

        let rel_frames: Vec<&FrameObj> = model
            .frames
            .values()
            .filter(|f| f.release_i.iter().any(|&r| r) || f.release_j.iter().any(|&r| r))
            .collect();
        if !rel_frames.is_empty() {
            out.push_str("TABLE:  \"FRAME RELEASE ASSIGNMENTS 1 - GENERAL\"\n");
            for f in &rel_frames {
                let pfix = f.start_spring.iter().any(|&k| k != 0.0)
                    || f.end_spring.iter().any(|&k| k != 0.0);
                out.push_str(&format!(
                    "   Frame={}   PI={}   V2I={}   V3I={}   TI={}   M2I={}   M3I={} _\n     PJ={}   V2J={}   V3J={}   TJ={}   M2J={}   M3J={}   PartialFix={}\n",
                    format_val(&f.name),
                    format_bool(f.release_i[0]),
                    format_bool(f.release_i[1]),
                    format_bool(f.release_i[2]),
                    format_bool(f.release_i[3]),
                    format_bool(f.release_i[4]),
                    format_bool(f.release_i[5]),
                    format_bool(f.release_j[0]),
                    format_bool(f.release_j[1]),
                    format_bool(f.release_j[2]),
                    format_bool(f.release_j[3]),
                    format_bool(f.release_j[4]),
                    format_bool(f.release_j[5]),
                    format_bool(pfix),
                ));
            }
            out.push('\n');
        }

        let pfix_frames: Vec<&FrameObj> = model
            .frames
            .values()
            .filter(|f| {
                f.start_spring.iter().any(|&k| k != 0.0) || f.end_spring.iter().any(|&k| k != 0.0)
            })
            .collect();
        if !pfix_frames.is_empty() {
            out.push_str("TABLE:  \"FRAME RELEASE ASSIGNMENTS 2 - PARTIAL FIXITY\"\n");
            for f in pfix_frames {
                out.push_str(&format!(
                    "   Frame={}   PI={}   V2I={}   V3I={}   TI={}   M2I={}   M3I={} _\n     PJ={}   V2J={}   V3J={}   TJ={}   M2J={}   M3J={}\n",
                    format_val(&f.name),
                    f.start_spring[0],
                    f.start_spring[1],
                    f.start_spring[2],
                    f.start_spring[3],
                    f.start_spring[4],
                    f.start_spring[5],
                    f.end_spring[0],
                    f.end_spring[1],
                    f.end_spring[2],
                    f.end_spring[3],
                    f.end_spring[4],
                    f.end_spring[5],
                ));
            }
            out.push('\n');
        }

        let off_frames: Vec<&FrameObj> = model
            .frames
            .values()
            .filter(|f| {
                f.end_offset_i != 0.0 || f.end_offset_j != 0.0 || f.rigid_zone_factor != 0.0
            })
            .collect();
        if !off_frames.is_empty() {
            out.push_str("TABLE:  \"FRAME OFFSET ALONG LENGTH ASSIGNMENTS\"\n");
            for f in off_frames {
                out.push_str(&format!(
                    "   Frame={}   AutoOffset=No   LengthI={}   LengthJ={}   RZ={}\n",
                    format_val(&f.name),
                    f.end_offset_i,
                    f.end_offset_j,
                    f.rigid_zone_factor,
                ));
            }
            out.push('\n');
        }

        let ins_frames: Vec<&FrameObj> = model
            .frames
            .values()
            .filter(|f| {
                f.cardinal_point != 10
                    || !f.stiff_transform
                    || !f.joint_offset_is_global
                    || f.joint_offset_i != [0.0; 3]
                    || f.joint_offset_j != [0.0; 3]
            })
            .collect();
        if !ins_frames.is_empty() {
            out.push_str("TABLE:  \"FRAME INSERTION POINT ASSIGNMENTS\"\n");
            for f in ins_frames {
                let csys = if f.joint_offset_is_global {
                    "GLOBAL"
                } else {
                    "LOCAL"
                };
                out.push_str(&format!(
                    "   Frame={}   CardinalPt={}   StiffTrans={}   CoordSys={} _\n     EndIOffsetX={}   EndIOffsetY={}   EndIOffsetZ={}   EndJOffsetX={}   EndJOffsetY={}   EndJOffsetZ={}\n",
                    format_val(&f.name),
                    f.cardinal_point,
                    format_bool(f.stiff_transform),
                    csys,
                    f.joint_offset_i[0],
                    f.joint_offset_i[1],
                    f.joint_offset_i[2],
                    f.joint_offset_j[0],
                    f.joint_offset_j[1],
                    f.joint_offset_j[2],
                ));
            }
            out.push('\n');
        }

        let tc_frames: Vec<&FrameObj> = model
            .frames
            .values()
            .filter(|f| f.tc_limits.is_some())
            .collect();
        if !tc_frames.is_empty() {
            out.push_str("TABLE:  \"FRAME TENSION AND COMPRESSION LIMITS\"\n");
            for f in tc_frames {
                if let Some((hc, cl, ht, tl)) = f.tc_limits {
                    out.push_str(&format!(
                        "   Frame={}   CLimitExists={}   CLimit={}   TLimitExists={}   TLimit={}\n",
                        format_val(&f.name),
                        format_bool(hc),
                        cl,
                        format_bool(ht),
                        tl,
                    ));
                }
            }
            out.push('\n');
        }

        let pd_frames: Vec<&FrameObj> = model
            .frames
            .values()
            .filter(|f| f.pdelta_force != 0.0)
            .collect();
        if !pd_frames.is_empty() {
            out.push_str("TABLE:  \"FRAME PDELTA FORCE ASSIGNMENTS\"\n");
            for f in pd_frames {
                out.push_str(&format!(
                    "   Frame={}   Force={}\n",
                    format_val(&f.name),
                    f.pdelta_force,
                ));
            }
            out.push('\n');
        }

        let cable_frames: Vec<&FrameObj> = model
            .frames
            .values()
            .filter(|f| f.cable_params.is_some())
            .collect();
        if !cable_frames.is_empty() {
            out.push_str("TABLE:  \"CABLE SHAPE DATA\"\n");
            for f in cable_frames {
                if let Some(ref cp) = f.cable_params {
                    let mut line = format!(
                        "   Frame={}   NumSegs={}",
                        format_val(&f.name),
                        cp.num_segments
                    );
                    if let Some(ul) = cp.undeformed_length {
                        line.push_str(&format!("   UndeformedLength={}", ul));
                    }
                    if let Some(ti) = cp.target_tension_i {
                        line.push_str(&format!("   TensionI={}", ti));
                    }
                    if let Some(ht) = cp.horizontal_tension {
                        line.push_str(&format!("   HorizTension={}", ht));
                    }
                    line.push('\n');
                    out.push_str(&line);
                }
            }
            out.push('\n');
        }

        let has_fdist = model.frames.values().any(|f| !f.loads_dist.is_empty());
        if has_fdist {
            out.push_str("TABLE:  \"FRAME LOADS - DISTRIBUTED\"\n");
            for f in model.frames.values() {
                for ld in &f.loads_dist {
                    let t_str = if ld.my_type == 2 { "Moment" } else { "Force" };
                    let (fa, fb, ma, mb) = if ld.my_type == 2 {
                        (0.0, 0.0, ld.val1, ld.val2)
                    } else {
                        (ld.val1, ld.val2, 0.0, 0.0)
                    };
                    out.push_str(&format!(
                        "   Frame={}   LoadPat={}   CoordSys={}   Type={}   MyType={}   Dir={}   DistType=RelDist _\n     RelDistA={}   RelDistB={}   FOverLA={}   FOverLB={}   MOverLA={}   MOverLB={}\n",
                        format_val(&f.name),
                        format_val(&ld.load_pat),
                        format_val(&ld.csys),
                        t_str,
                        ld.my_type,
                        format_val(&load_dir_str(ld.dir)),
                        ld.rel_dist1,
                        ld.rel_dist2,
                        fa,
                        fb,
                        ma,
                        mb,
                    ));
                }
            }
            out.push('\n');
        }

        let has_fpt = model.frames.values().any(|f| !f.loads_point.is_empty());
        if has_fpt {
            out.push_str("TABLE:  \"FRAME LOADS - POINT\"\n");
            for f in model.frames.values() {
                for lp in &f.loads_point {
                    let t_str = if lp.my_type == 2 { "Moment" } else { "Force" };
                    let (fv, mv) = if lp.my_type == 2 {
                        (0.0, lp.val)
                    } else {
                        (lp.val, 0.0)
                    };
                    out.push_str(&format!(
                        "   Frame={}   LoadPat={}   CoordSys={}   Type={}   MyType={}   Dir={}   DistType=RelDist   RelDist={}   Force={}   Moment={}\n",
                        format_val(&f.name),
                        format_val(&lp.load_pat),
                        format_val(&lp.csys),
                        t_str,
                        lp.my_type,
                        format_val(&load_dir_str(lp.dir)),
                        lp.rel_dist,
                        fv,
                        mv,
                    ));
                }
            }
            out.push('\n');
        }

        let has_ftemp = model.frames.values().any(|f| !f.loads_temp.is_empty());
        if has_ftemp {
            out.push_str("TABLE:  \"FRAME LOADS - TEMPERATURE\"\n");
            for f in model.frames.values() {
                for lt in &f.loads_temp {
                    let t_str = match lt.my_type {
                        2 => "Gradient2",
                        3 => "Gradient3",
                        _ => "Temperature",
                    };
                    out.push_str(&format!(
                        "   Frame={}   LoadPat={}   Type={}   MyType={}   Temp={}   JtPattern={}\n",
                        format_val(&f.name),
                        format_val(&lt.load_pat),
                        t_str,
                        lt.my_type,
                        lt.val,
                        format_val(lt.pattern_name.as_deref().unwrap_or("None")),
                    ));
                }
            }
            out.push('\n');
        }

        let has_fgrav = model.frames.values().any(|f| !f.loads_gravity.is_empty());
        if has_fgrav {
            out.push_str("TABLE:  \"FRAME LOADS - GRAVITY\"\n");
            for f in model.frames.values() {
                for (lpat, mult) in &f.loads_gravity {
                    out.push_str(&format!(
                        "   Frame={}   LoadPat={}   MultiplierX={}   MultiplierY={}   MultiplierZ={}\n",
                        format_val(&f.name),
                        format_val(lpat),
                        mult[0],
                        mult[1],
                        mult[2],
                    ));
                }
            }
            out.push('\n');
        }
    }

    // 10. AREAS & AREA LOADS
    if !model.areas.is_empty() {
        out.push_str("TABLE:  \"CONNECTIVITY - AREA\"\n");
        for a in model.areas.values() {
            let mut line = format!(
                "   Area={}   NumJoints={}",
                format_val(&a.name),
                a.points.len()
            );
            for (idx, pt) in a.points.iter().enumerate() {
                line.push_str(&format!("   Joint{}={}", idx + 1, format_val(pt)));
            }
            line.push_str(&format!(
                "   Selected={}   Angle={}   MeshN1={}   MeshN2={}   EdgeConstraint={}\n",
                format_bool(a.selected),
                a.angle_deg,
                a.auto_mesh.0,
                a.auto_mesh.1,
                format_bool(a.edge_constraint),
            ));
            out.push_str(&line);
        }
        out.push('\n');

        out.push_str("TABLE:  \"AREA SECTION ASSIGNMENTS\"\n");
        for a in model.areas.values() {
            out.push_str(&format!(
                "   Area={}   Section={}   MatProp=Default\n",
                format_val(&a.name),
                format_val(&a.section),
            ));
        }
        out.push('\n');

        let mod_areas: Vec<&AreaObj> = model
            .areas
            .values()
            .filter(|a| a.modifiers.iter().any(|&m| (m - 1.0).abs() > 1e-12))
            .collect();
        if !mod_areas.is_empty() {
            out.push_str("TABLE:  \"AREA PROPERTY MODIFIERS\"\n");
            for a in mod_areas {
                out.push_str(&format!(
                    "   Area={}   F11Mod={}   F22Mod={}   F12Mod={}   M11Mod={}   M22Mod={}   M12Mod={}   V13Mod={}   V23Mod={}   MMod={}   WMod={}\n",
                    format_val(&a.name),
                    a.modifiers[0],
                    a.modifiers[1],
                    a.modifiers[2],
                    a.modifiers[3],
                    a.modifiers[4],
                    a.modifiers[5],
                    a.modifiers[6],
                    a.modifiers[7],
                    a.modifiers[8],
                    a.modifiers[9],
                ));
            }
            out.push('\n');
        }

        let has_aunif = model.areas.values().any(|a| !a.loads_uniform.is_empty());
        if has_aunif {
            out.push_str("TABLE:  \"AREA LOADS - UNIFORM\"\n");
            for a in model.areas.values() {
                for lu in &a.loads_uniform {
                    out.push_str(&format!(
                        "   Area={}   LoadPat={}   CoordSys={}   Dir={}   UnifLoad={}\n",
                        format_val(&a.name),
                        format_val(&lu.load_pat),
                        format_val(&lu.csys),
                        format_val(&load_dir_str(lu.dir)),
                        lu.val,
                    ));
                }
            }
            out.push('\n');
        }

        let has_apress = model.areas.values().any(|a| !a.loads_pressure.is_empty());
        if has_apress {
            out.push_str("TABLE:  \"AREA LOADS - SURFACE PRESSURE\"\n");
            for a in model.areas.values() {
                for lp in &a.loads_pressure {
                    let face_str = match lp.face {
                        -1 => "Bottom".to_string(),
                        -2 => "Top".to_string(),
                        n => format!("Edge{}", n),
                    };
                    out.push_str(&format!(
                        "   Area={}   LoadPat={}   Face={}   Pressure={}   JtPattern={}\n",
                        format_val(&a.name),
                        format_val(&lp.load_pat),
                        face_str,
                        lp.val,
                        format_val(lp.pattern_name.as_deref().unwrap_or("None")),
                    ));
                }
            }
            out.push('\n');
        }

        let has_atemp = model.areas.values().any(|a| !a.loads_temp.is_empty());
        if has_atemp {
            out.push_str("TABLE:  \"AREA LOADS - TEMPERATURE\"\n");
            for a in model.areas.values() {
                for lt in &a.loads_temp {
                    let t_str = if lt.my_type == 2 {
                        "Gradient"
                    } else {
                        "Temperature"
                    };
                    out.push_str(&format!(
                        "   Area={}   LoadPat={}   Type={}   MyType={}   Temp={}   JtPattern={}\n",
                        format_val(&a.name),
                        format_val(&lt.load_pat),
                        t_str,
                        lt.my_type,
                        lt.val,
                        format_val(lt.pattern_name.as_deref().unwrap_or("None")),
                    ));
                }
            }
            out.push('\n');
        }
    }

    // 10B. SOLIDS & LINKS & FUNCTIONS
    if !model.solids.is_empty() {
        out.push_str("TABLE:  \"CONNECTIVITY - SOLID\"\n");
        for s in model.solids.values() {
            out.push_str(&format!(
                "   Solid={}   SolidProp={}   Joint1={}   Joint2={}   Joint3={}   Joint4={} _\n     Joint5={}   Joint6={}   Joint7={}   Joint8={}   Selected={}\n",
                format_val(&s.name),
                format_val(&s.section),
                format_val(&s.points[0]),
                format_val(&s.points[1]),
                format_val(&s.points[2]),
                format_val(&s.points[3]),
                format_val(&s.points[4]),
                format_val(&s.points[5]),
                format_val(&s.points[6]),
                format_val(&s.points[7]),
                format_bool(s.selected),
            ));
        }
        out.push('\n');

        let has_spress = model.solids.values().any(|s| !s.loads_surface.is_empty());
        if has_spress {
            out.push_str("TABLE:  \"SOLID LOADS - SURFACE PRESSURE\"\n");
            for s in model.solids.values() {
                for lp in &s.loads_surface {
                    out.push_str(&format!(
                        "   Solid={}   LoadPat={}   Face={}   Pressure={}\n",
                        format_val(&s.name),
                        format_val(&lp.load_pat),
                        lp.face,
                        lp.val,
                    ));
                }
            }
            out.push('\n');
        }

        let has_stemp = model.solids.values().any(|s| !s.loads_temp.is_empty());
        if has_stemp {
            out.push_str("TABLE:  \"SOLID LOADS - TEMPERATURE\"\n");
            for s in model.solids.values() {
                for (lpat, &val) in &s.loads_temp {
                    out.push_str(&format!(
                        "   Solid={}   LoadPat={}   Temp={}\n",
                        format_val(&s.name),
                        format_val(lpat),
                        val,
                    ));
                }
            }
            out.push('\n');
        }
    }

    if !model.links.is_empty() {
        out.push_str("TABLE:  \"CONNECTIVITY - LINK\"\n");
        for l in model.links.values() {
            out.push_str(&format!(
                "   Link={}   JointI={}   JointJ={}   LinkProp={}   Angle={}   Selected={}\n",
                format_val(&l.name),
                format_val(&l.point_i),
                format_val(l.point_j.as_deref().unwrap_or("None")),
                format_val(&l.prop),
                l.angle_deg,
                format_bool(l.selected),
            ));
        }
        out.push('\n');
    }

    let rs_funcs: Vec<&DynamicFunction> = model
        .functions
        .values()
        .filter(|f| !f.is_time_history)
        .collect();
    if !rs_funcs.is_empty() {
        out.push_str("TABLE:  \"FUNCTION - RESPONSE SPECTRUM - USER\"\n");
        for f in rs_funcs {
            if f.points.is_empty() {
                out.push_str(&format!(
                    "   Name={}   DampRatio={}\n",
                    format_val(&f.name),
                    f.damping_ratio,
                ));
            } else {
                for &(x, y) in &f.points {
                    out.push_str(&format!(
                        "   Name={}   Period={}   Accel={}   DampRatio={}\n",
                        format_val(&f.name),
                        x,
                        y,
                        f.damping_ratio,
                    ));
                }
            }
        }
        out.push('\n');
    }

    let th_funcs: Vec<&DynamicFunction> = model
        .functions
        .values()
        .filter(|f| f.is_time_history)
        .collect();
    if !th_funcs.is_empty() {
        out.push_str("TABLE:  \"FUNCTION - TIME HISTORY - USER\"\n");
        for f in th_funcs {
            if f.points.is_empty() {
                out.push_str(&format!(
                    "   Name={}   DampRatio={}\n",
                    format_val(&f.name),
                    f.damping_ratio
                ));
            } else {
                for &(x, y) in &f.points {
                    out.push_str(&format!(
                        "   Name={}   Time={}   Value={}   DampRatio={}\n",
                        format_val(&f.name),
                        x,
                        y,
                        f.damping_ratio,
                    ));
                }
            }
        }
        out.push('\n');
    }

    // 11. LOAD PATTERNS, LOAD CASES, COMBINATIONS
    out.push_str("TABLE:  \"LOAD PATTERN DEFINITIONS\"\n");
    for lp in model.load_patterns.values() {
        out.push_str(&format!(
            "   LoadPat={}   DesignType={}   SelfWtMult={}\n",
            format_val(&lp.name),
            format_val(load_pat_type_str(lp.pat_type)),
            lp.self_wt_multiplier,
        ));
    }
    out.push('\n');

    out.push_str("TABLE:  \"LOAD CASE DEFINITIONS\"\n");
    for lc in model.load_cases.values() {
        let ctype = match lc.case_type {
            LoadCaseType::LinearStatic => "LinStatic",
            LoadCaseType::NonlinearStatic => "NonStatic",
            LoadCaseType::Modal => "LinModal",
            LoadCaseType::ResponseSpectrum => "LinRespSpec",
            LoadCaseType::LinearHistory => "LinHist",
            LoadCaseType::NonlinearHistory => "NonHist",
            LoadCaseType::LinearDynamic => "LinDynamic",
            LoadCaseType::NonlinearDynamic => "NonDynamic",
            LoadCaseType::Buckling => "LinBuckling",
            LoadCaseType::MovingLoad => "Move",
            LoadCaseType::SteadyState => "SteadyState",
            LoadCaseType::PowerSpectralDensity => "PSD",
            LoadCaseType::LinearStaticMultiStep => "LinStaticMultiStep",
            LoadCaseType::HyperStatic => "HyperStatic",
        };
        let mtype = match lc.modal_sub_type {
            ModalSubType::Eigen => "Eigen",
            ModalSubType::Ritz => "Ritz",
        };
        let gnl = match lc.nl_geom {
            NLGeomType::None => "None",
            NLGeomType::PDelta => "PDelta",
            NLGeomType::LargeDisplacements => "LargeDisplacements",
        };
        let mcombo = match lc.modal_combo {
            ModalComboMethod::Cqc => "CQC",
            ModalComboMethod::Srss => "SRSS",
            ModalComboMethod::AbsSum => "ABS",
        };
        let tint = match lc.time_integration {
            TimeIntegrationMethod::NewmarkAverage => "NewmarkAverage",
            TimeIntegrationMethod::NewmarkLinear => "NewmarkLinear",
            TimeIntegrationMethod::HilberHughesTaylor => "HHT",
            TimeIntegrationMethod::WilsonTheta => "WilsonTheta",
        };
        let ic = lc.initial_case.as_deref().unwrap_or("Zero");
        out.push_str(&format!(
            "   Case={}   Type={}   InitialCond={}   RunCase={}   ModeType={}   MaxNumModes={}   MinNumModes={}   EigenShift={} _\n     GeomNonLin={}   MaxIter={}   ConvTol={}   ModalCombo={}   ConstDamp={}   NumSteps={}   StepSize={} _\n     TimeInteg={}   HHTAlpha={}   WilsonTheta={}   RayleighMass={}   RayleighStiff={}   NumBucklingModes={}\n",
            format_val(&lc.name),
            ctype,
            format_val(ic),
            format_bool(lc.run_flag),
            mtype,
            lc.max_modes,
            lc.min_modes,
            lc.shift_freq_hz,
            gnl,
            lc.max_iterations,
            lc.convergence_tol,
            mcombo,
            lc.modal_damping,
            lc.num_time_steps,
            lc.time_step_dt,
            tint,
            lc.hht_alpha,
            lc.wilson_theta,
            lc.rayleigh_damping.0,
            lc.rayleigh_damping.1,
            lc.num_buckling_modes,
        ));
    }
    out.push('\n');

    out.push_str("TABLE:  \"CASE - STATIC 1 - LOAD ASSIGNMENTS\"\n");
    for lc in model.load_cases.values() {
        for asgn in &lc.loads {
            out.push_str(&format!(
                "   Case={}   LoadType=\"Load pattern\"   LoadName={}   LoadSF={}\n",
                format_val(&lc.name),
                format_val(&asgn.load_name),
                asgn.scale_factor,
            ));
        }
    }
    out.push('\n');

    let modal_cases: Vec<&LoadCase> = model
        .load_cases
        .values()
        .filter(|lc| lc.case_type == LoadCaseType::Modal)
        .collect();
    if !modal_cases.is_empty() {
        out.push_str("TABLE:  \"CASE - MODAL 1 - GENERAL\"\n");
        for lc in modal_cases {
            let mtype = match lc.modal_sub_type {
                ModalSubType::Eigen => "Eigen",
                ModalSubType::Ritz => "Ritz",
            };
            out.push_str(&format!(
                "   Case={}   ModeType={}   MaxNumModes={}   MinNumModes={}   EigenShift={}\n",
                format_val(&lc.name),
                mtype,
                lc.max_modes,
                lc.min_modes,
                lc.shift_freq_hz,
            ));
        }
        out.push('\n');
    }

    let has_spec_loads = model
        .load_cases
        .values()
        .any(|lc| !lc.spectrum_loads.is_empty());
    if has_spec_loads {
        out.push_str("TABLE:  \"CASE - RESPONSE SPECTRUM 2 - LOAD ASSIGNMENTS\"\n");
        for lc in model.load_cases.values() {
            for (func, cdir, sf) in &lc.spectrum_loads {
                out.push_str(&format!(
                    "   Case={}   Function={}   CoordDir={}   TransAccSF={}\n",
                    format_val(&lc.name),
                    format_val(func),
                    format_val(cdir),
                    sf,
                ));
            }
        }
        out.push('\n');
    }

    let has_hist_loads = model
        .load_cases
        .values()
        .any(|lc| !lc.history_loads.is_empty());
    if has_hist_loads {
        out.push_str("TABLE:  \"CASE - TIME HISTORY 2 - LOAD ASSIGNMENTS\"\n");
        for lc in model.load_cases.values() {
            for (func, lname, sf) in &lc.history_loads {
                out.push_str(&format!(
                    "   Case={}   Function={}   LoadName={}   SF={}\n",
                    format_val(&lc.name),
                    format_val(func),
                    format_val(lname),
                    sf,
                ));
            }
        }
        out.push('\n');
    }

    if !model.load_combos.is_empty() {
        out.push_str("TABLE:  \"COMBINATION DEFINITIONS\"\n");
        for cb in model.load_combos.values() {
            let ctype = match cb.combo_type {
                ComboType::LinearAdd => "Linear Add",
                ComboType::Envelope => "Envelope",
                ComboType::AbsAdd => "Abs Add",
                ComboType::Srss => "SRSS",
                ComboType::RangeAdd => "Range Add",
            };
            if cb.cases.is_empty() {
                out.push_str(&format!(
                    "   ComboName={}   ComboType={}\n",
                    format_val(&cb.name),
                    format_val(ctype),
                ));
            } else {
                for (cname, sf) in &cb.cases {
                    out.push_str(&format!(
                        "   ComboName={}   ComboType={}   CaseName={}   ScaleFactor={}\n",
                        format_val(&cb.name),
                        format_val(ctype),
                        format_val(cname),
                        sf,
                    ));
                }
            }
        }
        out.push('\n');
    }

    out.push_str("END TABLE DATA\n");
    out
}

/// Export with conservative source preservation and actionable conflict errors.
pub fn try_write_s2k_with_report(model: &SapModel) -> Result<crate::ExportReport, FormatError> {
    preservation::write(model)
}

/// Convenience text serializer. Panics only when preserved opaque data cannot
/// be associated safely after edits. File exports use the checked API instead.
/// Prefer [`try_write_s2k_with_report`] for imported/user-edited models.
pub fn write_s2k(model: &SapModel) -> String {
    try_write_s2k_with_report(model)
        .expect(
            "S2K source preservation conflict; use try_write_s2k_with_report for checked export",
        )
        .text
}
