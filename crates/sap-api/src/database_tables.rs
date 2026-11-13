use std::collections::{BTreeMap, HashSet};

use sap_model::{
    ComboType, FramePropType, FrameSection, LoadCase, LoadCaseAssignment, LoadCaseType, LoadCombo,
    LoadPattern, LoadPatternType, MatType, Material, PointObj, SapModel,
};

use crate::{cSapModel, is_valid_name, OAPI_ERR, OAPI_OK};

/// Staged table entry stored in memory prior to `apply_edited_tables`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StagedTable {
    pub table_key: String,
    pub table_version: i32,
    pub fields: Vec<String>,
    pub num_records: usize,
    pub data: Vec<String>,
}

/// Catalog entry for available database tables.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TableCatalogEntry {
    pub key: String,
    pub name: String,
    pub import_type: i32, // 1 = editable definition, 2 = read-only result/design
    pub is_empty: bool,
}

pub struct FieldDef {
    pub key: &'static str,
    pub name: &'static str,
    pub description: &'static str,
    pub units: &'static str,
    pub is_importable: bool,
}

pub struct TableDef {
    pub key: &'static str,
    pub name: &'static str,
    pub import_type: i32,
    pub fields: &'static [FieldDef],
}

macro_rules! field {
    ($key:expr, $name:expr, $desc:expr, $units:expr, $import:expr) => {
        FieldDef {
            key: $key,
            name: $name,
            description: $desc,
            units: $units,
            is_importable: $import,
        }
    };
}

static TABLES: &[TableDef] = &[
    TableDef {
        key: "JOINT COORDINATES",
        name: "Joint Coordinates",
        import_type: 1,
        fields: &[
            field!("Joint", "Joint", "Joint Name", "", true),
            field!("CoordSys", "CoordSys", "Coordinate System", "", true),
            field!("CoordType", "CoordType", "Coordinate Type", "", true),
            field!("XorR", "XorR", "X Coordinate", "Length", true),
            field!("Y", "Y", "Y Coordinate", "Length", true),
            field!("Z", "Z", "Z Coordinate", "Length", true),
            field!("SpecialJt", "SpecialJt", "Special Joint", "", true),
            field!("GlobalX", "GlobalX", "Global X Coordinate", "Length", false),
            field!("GlobalY", "GlobalY", "Global Y Coordinate", "Length", false),
            field!("GlobalZ", "GlobalZ", "Global Z Coordinate", "Length", false),
        ],
    },
    TableDef {
        key: "CONNECTIVITY - FRAME",
        name: "Connectivity - Frame",
        import_type: 1,
        fields: &[
            field!("Frame", "Frame", "Frame Name", "", true),
            field!("JointI", "JointI", "Joint I", "", true),
            field!("JointJ", "JointJ", "Joint J", "", true),
            field!("IsCurved", "IsCurved", "Is Curved", "", false),
            field!("MinSta", "MinSta", "Minimum Output Stations", "", true),
        ],
    },
    TableDef {
        key: "FRAME SECTION ASSIGNMENTS",
        name: "Frame Section Assignments",
        import_type: 1,
        fields: &[
            field!("Frame", "Frame", "Frame Name", "", true),
            field!("AnalSect", "AnalSect", "Analysis Section", "", true),
            field!("DesignSect", "DesignSect", "Design Section", "", true),
            field!("MatProp", "MatProp", "Material Property", "", false),
        ],
    },
    TableDef {
        key: "MATERIAL PROPERTIES 01 - GENERAL",
        name: "Material Properties 01 - General",
        import_type: 1,
        fields: &[
            field!("Material", "Material", "Material Name", "", true),
            field!("Type", "Type", "Material Type", "", true),
            field!("SymType", "SymType", "Symmetry Type", "", true),
            field!(
                "TempDepend",
                "TempDepend",
                "Temperature Dependent",
                "",
                false
            ),
            field!("Color", "Color", "Display Color", "", true),
        ],
    },
    TableDef {
        key: "MATERIAL PROPERTIES 02 - BASIC MECHANICAL PROPERTIES",
        name: "Material Properties 02 - Basic Mechanical Properties",
        import_type: 1,
        fields: &[
            field!("Material", "Material", "Material Name", "", true),
            field!(
                "UnitWeight",
                "UnitWeight",
                "Unit Weight",
                "Force/Length^3",
                true
            ),
            field!("UnitMass", "UnitMass", "Unit Mass", "Mass/Length^3", true),
            field!("E1", "E1", "Modulus of Elasticity", "Force/Length^2", true),
            field!("G12", "G12", "Shear Modulus", "Force/Length^2", true),
            field!("U12", "U12", "Poisson's Ratio", "", true),
            field!("A1", "A1", "Thermal Coefficient", "1/Temp", true),
            field!("Fy", "Fy", "Yield Strength", "Force/Length^2", true),
            field!("Fu", "Fu", "Tensile Strength", "Force/Length^2", true),
            field!(
                "Fc",
                "Fc",
                "Concrete Compressive Strength",
                "Force/Length^2",
                true
            ),
            field!(
                "Fys",
                "Fys",
                "Shear Rebar Yield Strength",
                "Force/Length^2",
                true
            ),
        ],
    },
    TableDef {
        key: "FRAME SECTION PROPERTIES 01 - GENERAL",
        name: "Frame Section Properties 01 - General",
        import_type: 1,
        fields: &[
            field!("SectionName", "SectionName", "Section Name", "", true),
            field!("Material", "Material", "Material Name", "", true),
            field!("Shape", "Shape", "Section Shape", "", true),
            field!("t3", "t3", "Depth / t3", "Length", true),
            field!("t2", "t2", "Width / t2", "Length", true),
            field!("tf", "tf", "Flange Thickness", "Length", true),
            field!("tw", "tw", "Web Thickness", "Length", true),
            field!("Area", "Area", "Cross-Sectional Area", "Length^2", true),
            field!(
                "TorsConst",
                "TorsConst",
                "Torsional Constant",
                "Length^4",
                true
            ),
            field!("I33", "I33", "Moment of Inertia I33", "Length^4", true),
            field!("I22", "I22", "Moment of Inertia I22", "Length^4", true),
            field!("AS2", "AS2", "Shear Area AS2", "Length^2", true),
            field!("AS3", "AS3", "Shear Area AS3", "Length^2", true),
            field!("S33", "S33", "Section Modulus S33", "Length^3", true),
            field!("S22", "S22", "Section Modulus S22", "Length^3", true),
            field!("Z33", "Z33", "Plastic Modulus Z33", "Length^3", true),
            field!("Z22", "Z22", "Plastic Modulus Z22", "Length^3", true),
            field!("R33", "R33", "Radius of Gyration R33", "Length", true),
            field!("R22", "R22", "Radius of Gyration R22", "Length", true),
        ],
    },
    TableDef {
        key: "LOAD PATTERN DEFINITIONS",
        name: "Load Pattern Definitions",
        import_type: 1,
        fields: &[
            field!("LoadPat", "LoadPat", "Load Pattern Name", "", true),
            field!("DesignType", "DesignType", "Design Type", "", true),
            field!(
                "SelfWtMult",
                "SelfWtMult",
                "Self Weight Multiplier",
                "",
                true
            ),
        ],
    },
    TableDef {
        key: "LOAD CASE DEFINITIONS",
        name: "Load Case Definitions",
        import_type: 1,
        fields: &[
            field!("Case", "Case", "Load Case Name", "", true),
            field!("Type", "Type", "Case Type", "", true),
            field!(
                "InitialCond",
                "InitialCond",
                "Initial Conditions",
                "",
                false
            ),
            field!("DesignType", "DesignType", "Design Type", "", true),
        ],
    },
    TableDef {
        key: "JOINT RESTRAINT ASSIGNMENTS",
        name: "Joint Restraint Assignments",
        import_type: 1,
        fields: &[
            field!("Joint", "Joint", "Joint Name", "", true),
            field!("U1", "U1", "U1 Restraint", "", true),
            field!("U2", "U2", "U2 Restraint", "", true),
            field!("U3", "U3", "U3 Restraint", "", true),
            field!("R1", "R1", "R1 Restraint", "", true),
            field!("R2", "R2", "R2 Restraint", "", true),
            field!("R3", "R3", "R3 Restraint", "", true),
        ],
    },
    TableDef {
        key: "COMBINATION DEFINITIONS",
        name: "Combination Definitions",
        import_type: 1,
        fields: &[
            field!("ComboName", "ComboName", "Combination Name", "", true),
            field!("ComboType", "ComboType", "Combination Type", "", true),
            field!("CaseName", "CaseName", "Case or Pattern Name", "", true),
            field!("ScaleFactor", "ScaleFactor", "Scale Factor", "", true),
        ],
    },
    TableDef {
        key: "JOINT DISPLACEMENTS",
        name: "Joint Displacements",
        import_type: 2,
        fields: &[
            field!("Joint", "Joint", "Joint Name", "", false),
            field!("OutputCase", "OutputCase", "Output Case", "", false),
            field!("StepType", "StepType", "Step Type", "", false),
            field!("StepNum", "StepNum", "Step Number", "", false),
            field!("U1", "U1", "Displacement U1", "Length", false),
            field!("U2", "U2", "Displacement U2", "Length", false),
            field!("U3", "U3", "Displacement U3", "Length", false),
            field!("R1", "R1", "Rotation R1", "Radians", false),
            field!("R2", "R2", "Rotation R2", "Radians", false),
            field!("R3", "R3", "Rotation R3", "Radians", false),
        ],
    },
    TableDef {
        key: "JOINT REACTIONS",
        name: "Joint Reactions",
        import_type: 2,
        fields: &[
            field!("Joint", "Joint", "Joint Name", "", false),
            field!("OutputCase", "OutputCase", "Output Case", "", false),
            field!("StepType", "StepType", "Step Type", "", false),
            field!("StepNum", "StepNum", "Step Number", "", false),
            field!("F1", "F1", "Reaction Force F1", "Force", false),
            field!("F2", "F2", "Reaction Force F2", "Force", false),
            field!("F3", "F3", "Reaction Force F3", "Force", false),
            field!("M1", "M1", "Reaction Moment M1", "Force-Length", false),
            field!("M2", "M2", "Reaction Moment M2", "Force-Length", false),
            field!("M3", "M3", "Reaction Moment M3", "Force-Length", false),
        ],
    },
    TableDef {
        key: "BASE REACTIONS",
        name: "Base Reactions",
        import_type: 2,
        fields: &[
            field!("OutputCase", "OutputCase", "Output Case", "", false),
            field!("StepType", "StepType", "Step Type", "", false),
            field!("StepNum", "StepNum", "Step Number", "", false),
            field!("GlobalFX", "GlobalFX", "Global FX", "Force", false),
            field!("GlobalFY", "GlobalFY", "Global FY", "Force", false),
            field!("GlobalFZ", "GlobalFZ", "Global FZ", "Force", false),
            field!("GlobalMX", "GlobalMX", "Global MX", "Force-Length", false),
            field!("GlobalMY", "GlobalMY", "Global MY", "Force-Length", false),
            field!("GlobalMZ", "GlobalMZ", "Global MZ", "Force-Length", false),
        ],
    },
    TableDef {
        key: "ELEMENT FORCES - FRAMES",
        name: "Element Forces - Frames",
        import_type: 2,
        fields: &[
            field!("Frame", "Frame", "Frame Name", "", false),
            field!("Station", "Station", "Station Distance", "Length", false),
            field!("OutputCase", "OutputCase", "Output Case", "", false),
            field!("StepType", "StepType", "Step Type", "", false),
            field!("StepNum", "StepNum", "Step Number", "", false),
            field!("P", "P", "Axial Force P", "Force", false),
            field!("V2", "V2", "Shear Force V2", "Force", false),
            field!("V3", "V3", "Shear Force V3", "Force", false),
            field!("T", "T", "Torsional Moment T", "Force-Length", false),
            field!("M2", "M2", "Bending Moment M2", "Force-Length", false),
            field!("M3", "M3", "Bending Moment M3", "Force-Length", false),
        ],
    },
    TableDef {
        key: "MODAL PERIODS AND FREQUENCIES",
        name: "Modal Periods and Frequencies",
        import_type: 2,
        fields: &[
            field!("OutputCase", "OutputCase", "Output Case", "", false),
            field!("StepType", "StepType", "Step Type", "", false),
            field!("StepNum", "StepNum", "Mode Number", "", false),
            field!("Period", "Period", "Modal Period", "Time", false),
            field!("Frequency", "Frequency", "Frequency", "1/Time", false),
            field!(
                "CircFreq",
                "CircFreq",
                "Circular Frequency",
                "Radians/Time",
                false
            ),
            field!(
                "Eigenvalue",
                "Eigenvalue",
                "Eigenvalue",
                "Radians^2/Time^2",
                false
            ),
        ],
    },
    TableDef {
        key: "BUCKLING FACTORS",
        name: "Buckling Factors",
        import_type: 2,
        fields: &[
            field!("OutputCase", "OutputCase", "Output Case", "", false),
            field!("StepType", "StepType", "Step Type", "", false),
            field!("StepNum", "StepNum", "Mode Number", "", false),
            field!("Factor", "Factor", "Buckling Factor Lambda", "", false),
        ],
    },
    TableDef {
        key: "STEEL DESIGN 1 - SUMMARY DATA",
        name: "Steel Design 1 - Summary Data",
        import_type: 2,
        fields: &[
            field!("Frame", "Frame", "Frame Name", "", false),
            field!("DesignSect", "DesignSect", "Design Section", "", false),
            field!("DesignCode", "DesignCode", "Design Code", "", false),
            field!("Combo", "Combo", "Governing Combo", "", false),
            field!("Station", "Station", "Governing Station", "Length", false),
            field!("Ratio", "Ratio", "Demand/Capacity Ratio", "", false),
            field!("RatioType", "RatioType", "Ratio Type", "", false),
            field!("Status", "Status", "Design Status", "", false),
        ],
    },
    TableDef {
        key: "CONCRETE DESIGN 1 - COLUMN SUMMARY DATA",
        name: "Concrete Design 1 - Column Summary Data",
        import_type: 2,
        fields: &[
            field!("Frame", "Frame", "Frame Name", "", false),
            field!("DesignSect", "DesignSect", "Design Section", "", false),
            field!("DesignCode", "DesignCode", "Design Code", "", false),
            field!("Combo", "Combo", "Governing Combo", "", false),
            field!("Station", "Station", "Governing Station", "Length", false),
            field!(
                "TotalAs",
                "TotalAs",
                "Total Longitudinal Rebar",
                "Length^2",
                false
            ),
            field!(
                "TotalAv",
                "TotalAv",
                "Transverse Rebar / Length",
                "Length^2/Length",
                false
            ),
            field!("PMMRatio", "PMMRatio", "PMM Interaction Ratio", "", false),
            field!("Status", "Status", "Design Status", "", false),
        ],
    },
    TableDef {
        key: "CONCRETE DESIGN 2 - BEAM SUMMARY DATA",
        name: "Concrete Design 2 - Beam Summary Data",
        import_type: 2,
        fields: &[
            field!("Frame", "Frame", "Frame Name", "", false),
            field!("DesignSect", "DesignSect", "Design Section", "", false),
            field!("DesignCode", "DesignCode", "Design Code", "", false),
            field!("Combo", "Combo", "Governing Combo", "", false),
            field!("Station", "Station", "Governing Station", "Length", false),
            field!(
                "TopAs",
                "TopAs",
                "Top Longitudinal Rebar",
                "Length^2",
                false
            ),
            field!(
                "BotAs",
                "BotAs",
                "Bottom Longitudinal Rebar",
                "Length^2",
                false
            ),
            field!(
                "SideAs",
                "SideAs",
                "Side Longitudinal Rebar",
                "Length^2",
                false
            ),
            field!(
                "TotalAv",
                "TotalAv",
                "Transverse Rebar / Length",
                "Length^2/Length",
                false
            ),
            field!("Status", "Status", "Design Status", "", false),
        ],
    },
];

fn canonical_table_key(key: &str) -> String {
    let mut s = key.trim();
    if s.to_ascii_uppercase().starts_with("TABLE:") {
        s = s[6..].trim();
    }
    s = s.trim_matches('"').trim();
    let upper = s.to_ascii_uppercase();
    match upper.as_str() {
        "JOINT RESTRAINTS" => "JOINT RESTRAINT ASSIGNMENTS".to_string(),
        "FRAME FORCES" => "ELEMENT FORCES - FRAMES".to_string(),
        "MODAL PERIODS" => "MODAL PERIODS AND FREQUENCIES".to_string(),
        _ => upper,
    }
}

fn find_table_def(key: &str) -> Option<&'static TableDef> {
    let canon = canonical_table_key(key);
    TABLES.iter().find(|t| t.key == canon)
}

fn is_table_empty(model: &SapModel, key: &str) -> bool {
    match key {
        "JOINT COORDINATES" => model.points.is_empty(),
        "CONNECTIVITY - FRAME" | "FRAME SECTION ASSIGNMENTS" => model.frames.is_empty(),
        "MATERIAL PROPERTIES 01 - GENERAL"
        | "MATERIAL PROPERTIES 02 - BASIC MECHANICAL PROPERTIES" => model.materials.is_empty(),
        "FRAME SECTION PROPERTIES 01 - GENERAL" => model.frame_sections.is_empty(),
        "LOAD PATTERN DEFINITIONS" => model.load_patterns.is_empty(),
        "LOAD CASE DEFINITIONS" => model.load_cases.is_empty(),
        "JOINT RESTRAINT ASSIGNMENTS" => !model
            .points
            .values()
            .any(|p| p.restraint.iter().any(|&r| r)),
        "COMBINATION DEFINITIONS" => model.load_combos.is_empty(),
        "JOINT DISPLACEMENTS" => !model
            .case_results
            .values()
            .any(|cr| !cr.joint_displacements.is_empty()),
        "JOINT REACTIONS" => !model
            .case_results
            .values()
            .any(|cr| !cr.joint_reactions.is_empty()),
        "BASE REACTIONS" => !model
            .case_results
            .values()
            .any(|cr| cr.base_reactions.iter().any(|&v| v.abs() > 1e-12)),
        "ELEMENT FORCES - FRAMES" => !model
            .case_results
            .values()
            .any(|cr| !cr.frame_forces.is_empty()),
        "MODAL PERIODS AND FREQUENCIES" => {
            !model.case_results.values().any(|cr| !cr.modes.is_empty())
        }
        "BUCKLING FACTORS" => !model
            .case_results
            .values()
            .any(|cr| !cr.buckling_factors.is_empty()),
        "STEEL DESIGN 1 - SUMMARY DATA" => model.steel_design_results.is_empty(),
        "CONCRETE DESIGN 1 - COLUMN SUMMARY DATA" => {
            !model.concrete_design_results.values().any(|r| r.is_column)
        }
        "CONCRETE DESIGN 2 - BEAM SUMMARY DATA" => {
            !model.concrete_design_results.values().any(|r| !r.is_column)
        }
        _ => true,
    }
}

/// Interactive Database Tables facade (`SAP2000v19.cDatabaseTables`).
pub struct cDatabaseTables<'a> {
    api: &'a mut cSapModel,
}

impl<'a> cDatabaseTables<'a> {
    pub fn new(api: &'a mut cSapModel) -> Self {
        Self { api }
    }

    /// Queries the catalog of available database tables (`cDatabaseTables.GetAvailableTables`).
    pub fn get_available_tables(
        &self,
        number_tables: &mut i32,
        table_key: &mut Vec<String>,
        table_name: &mut Vec<String>,
        import_type: &mut Vec<i32>,
    ) -> i32 {
        table_key.clear();
        table_name.clear();
        import_type.clear();

        for t in TABLES {
            table_key.push(t.key.to_string());
            table_name.push(t.name.to_string());
            import_type.push(t.import_type);
        }
        *number_tables = TABLES.len() as i32;
        OAPI_OK
    }

    /// Convenience query returning tables as a tuple.
    pub fn get_available_tables_vec(&self) -> (i32, Vec<String>, Vec<String>, Vec<i32>) {
        let mut n = 0;
        let mut keys = Vec::new();
        let mut names = Vec::new();
        let mut types = Vec::new();
        let ret = self.get_available_tables(&mut n, &mut keys, &mut names, &mut types);
        (ret, keys, names, types)
    }

    /// Queries the full catalog of database tables including empty status (`cDatabaseTables.GetAllTables`).
    pub fn get_all_tables(
        &self,
        number_tables: &mut i32,
        table_key: &mut Vec<String>,
        table_name: &mut Vec<String>,
        import_type: &mut Vec<i32>,
        is_empty: &mut Vec<bool>,
    ) -> i32 {
        self.get_available_tables(number_tables, table_key, table_name, import_type);
        is_empty.clear();
        for key in table_key.iter() {
            is_empty.push(is_table_empty(&self.api.inner, key));
        }
        OAPI_OK
    }

    /// Alias for `get_all_tables` matching CSI OAPI naming variants (`cDatabaseTables.GetAllTablesKeyInfo`).
    pub fn get_all_tables_key_info(
        &self,
        number_tables: &mut i32,
        table_key: &mut Vec<String>,
        table_name: &mut Vec<String>,
        import_type: &mut Vec<i32>,
        is_empty: &mut Vec<bool>,
    ) -> i32 {
        self.get_all_tables(number_tables, table_key, table_name, import_type, is_empty)
    }

    /// Convenience query returning all tables info as a tuple.
    pub fn get_all_tables_vec(&self) -> (i32, Vec<String>, Vec<String>, Vec<i32>, Vec<bool>) {
        let mut n = 0;
        let mut keys = Vec::new();
        let mut names = Vec::new();
        let mut types = Vec::new();
        let mut empty = Vec::new();
        let ret = self.get_all_tables(&mut n, &mut keys, &mut names, &mut types, &mut empty);
        (ret, keys, names, types, empty)
    }

    /// Queries column schema and metadata for a specified table key (`cDatabaseTables.GetAllFieldsInTable`).
    #[allow(clippy::too_many_arguments)]
    pub fn get_all_fields_in_table(
        &self,
        table_key: &str,
        table_version: &mut i32,
        number_fields: &mut i32,
        field_key: &mut Vec<String>,
        field_name: &mut Vec<String>,
        description: &mut Vec<String>,
        units_string: &mut Vec<String>,
        is_importable: &mut Vec<bool>,
    ) -> i32 {
        let Some(t) = find_table_def(table_key) else {
            return OAPI_ERR;
        };
        *table_version = 1;
        field_key.clear();
        field_name.clear();
        description.clear();
        units_string.clear();
        is_importable.clear();

        for f in t.fields {
            field_key.push(f.key.to_string());
            field_name.push(f.name.to_string());
            description.push(f.description.to_string());
            units_string.push(f.units.to_string());
            is_importable.push(f.is_importable);
        }
        *number_fields = t.fields.len() as i32;
        OAPI_OK
    }

    /// Extracts table data for display or post-processing (`cDatabaseTables.GetTableForDisplayArray`).
    /// Row-major layout: TableData[row * num_fields + col].
    pub fn get_table_for_display_array(
        &self,
        table_key: &str,
        field_key_list: &[String],
        group_name: &str,
        table_version: &mut i32,
        fields_keys_included: &mut Vec<String>,
        number_records: &mut i32,
        table_data: &mut Vec<String>,
    ) -> i32 {
        let Some(t) = find_table_def(table_key) else {
            return OAPI_ERR;
        };
        *table_version = 1;
        fields_keys_included.clear();
        table_data.clear();
        *number_records = 0;

        // Group filtering: "" and "ALL" match all; any other non-existent group returns 0 records.
        let group_upper = group_name.trim().to_ascii_uppercase();
        let matches_all = group_upper.is_empty() || group_upper == "ALL";

        // Determine fields included
        if field_key_list.is_empty() {
            for f in t.fields {
                fields_keys_included.push(f.key.to_string());
            }
        } else {
            for req in field_key_list {
                if let Some(f) = t.fields.iter().find(|fd| fd.key.eq_ignore_ascii_case(req)) {
                    fields_keys_included.push(f.key.to_string());
                }
            }
            if fields_keys_included.is_empty() {
                return OAPI_ERR;
            }
        }

        if !matches_all {
            *number_records = 0;
            return OAPI_OK;
        }

        let rows = self.extract_table_rows(t.key);
        *number_records = rows.len() as i32;
        table_data.reserve(rows.len() * fields_keys_included.len());

        for row in &rows {
            for col in fields_keys_included.iter() {
                let val = row.get(col.as_str()).cloned().unwrap_or_default();
                table_data.push(val);
            }
        }

        OAPI_OK
    }

    /// Convenience query returning display array as tuple.
    pub fn get_table_for_display_array_vec(
        &self,
        table_key: &str,
        field_key_list: &[&str],
        group_name: &str,
    ) -> (i32, Vec<String>, i32, Vec<String>) {
        let mut ver = 1;
        let mut included = Vec::new();
        let mut num = 0;
        let mut data = Vec::new();
        let req_fields: Vec<String> = field_key_list.iter().map(|s| s.to_string()).collect();
        let ret = self.get_table_for_display_array(
            table_key,
            &req_fields,
            group_name,
            &mut ver,
            &mut included,
            &mut num,
            &mut data,
        );
        (ret, included, num, data)
    }

    /// Extracts an editable model definition table into array format (`cDatabaseTables.GetTableForEditingArray`).
    pub fn get_table_for_editing_array(
        &self,
        table_key: &str,
        group_name: &str,
        table_version: &mut i32,
        fields_keys_included: &mut Vec<String>,
        number_records: &mut i32,
        table_data: &mut Vec<String>,
    ) -> i32 {
        let Some(t) = find_table_def(table_key) else {
            return OAPI_ERR;
        };
        if t.import_type != 1 {
            // Read-only / result tables cannot be extracted for editing
            return OAPI_ERR;
        }
        self.get_table_for_display_array(
            table_key,
            &[],
            group_name,
            table_version,
            fields_keys_included,
            number_records,
            table_data,
        )
    }

    /// Convenience query returning editing array as tuple.
    pub fn get_table_for_editing_array_vec(
        &self,
        table_key: &str,
        group_name: &str,
    ) -> (i32, Vec<String>, i32, Vec<String>) {
        let mut ver = 1;
        let mut included = Vec::new();
        let mut num = 0;
        let mut data = Vec::new();
        let ret = self.get_table_for_editing_array(
            table_key,
            group_name,
            &mut ver,
            &mut included,
            &mut num,
            &mut data,
        );
        (ret, included, num, data)
    }

    /// Stages an edited table in the internal memory buffer (`cDatabaseTables.SetTableForEditingArray`).
    pub fn set_table_for_editing_array(
        &mut self,
        table_key: &str,
        table_version: &mut i32,
        fields_keys_included: &[String],
        number_records: i32,
        table_data: &[String],
    ) -> i32 {
        if self.api.inner.is_locked || number_records < 0 {
            return OAPI_ERR;
        }
        let Some(t) = find_table_def(table_key) else {
            return OAPI_ERR;
        };
        if t.import_type != 1 {
            return OAPI_ERR;
        }

        let num_rec = number_records as usize;
        let num_fields = fields_keys_included.len();
        if num_rec > 0 && num_fields == 0 {
            return OAPI_ERR;
        }
        if table_data.len() != num_rec * num_fields {
            return OAPI_ERR;
        }

        // Validate primary key exists in fields_keys_included
        let primary_key = match t.key {
            "JOINT COORDINATES" => "Joint",
            "CONNECTIVITY - FRAME" | "FRAME SECTION ASSIGNMENTS" => "Frame",
            "MATERIAL PROPERTIES 01 - GENERAL"
            | "MATERIAL PROPERTIES 02 - BASIC MECHANICAL PROPERTIES" => "Material",
            "FRAME SECTION PROPERTIES 01 - GENERAL" => "SectionName",
            "LOAD PATTERN DEFINITIONS" => "LoadPat",
            "LOAD CASE DEFINITIONS" => "Case",
            "JOINT RESTRAINT ASSIGNMENTS" => "Joint",
            "COMBINATION DEFINITIONS" => "ComboName",
            _ => "",
        };

        if !primary_key.is_empty()
            && !fields_keys_included
                .iter()
                .any(|f| f.eq_ignore_ascii_case(primary_key))
        {
            return OAPI_ERR;
        }

        let staged = StagedTable {
            table_key: t.key.to_string(),
            table_version: *table_version,
            fields: fields_keys_included.to_vec(),
            num_records: num_rec,
            data: table_data.to_vec(),
        };

        self.api.staged_tables.insert(t.key.to_string(), staged);
        OAPI_OK
    }

    /// Convenience wrapper accepting `&str` slices for table staging.
    pub fn set_table_for_editing_array_str(
        &mut self,
        table_key: &str,
        fields_keys_included: &[&str],
        number_records: i32,
        table_data: &[&str],
    ) -> i32 {
        let mut ver = 1;
        let fields: Vec<String> = fields_keys_included.iter().map(|s| s.to_string()).collect();
        let data: Vec<String> = table_data.iter().map(|s| s.to_string()).collect();
        self.set_table_for_editing_array(table_key, &mut ver, &fields, number_records, &data)
    }

    /// Commits all staged database table edits into the model (`cDatabaseTables.ApplyEditedTables`).
    pub fn apply_edited_tables(
        &mut self,
        fill_import_log: bool,
        num_fatal_errors: &mut i32,
        num_error_msgs: &mut i32,
        num_warn_msgs: &mut i32,
        num_info_msgs: &mut i32,
        import_log: &mut String,
    ) -> i32 {
        *num_fatal_errors = 0;
        *num_error_msgs = 0;
        *num_warn_msgs = 0;
        *num_info_msgs = 0;

        if self.api.inner.is_locked {
            *num_fatal_errors += 1;
            if fill_import_log {
                import_log.push_str("FATAL ERROR: Model is locked. Edits cannot be applied.\n");
            }
            return OAPI_ERR;
        }

        if self.api.staged_tables.is_empty() {
            if fill_import_log {
                import_log.push_str("INFO: No staged tables to apply.\n");
            }
            *num_info_msgs += 1;
            return OAPI_OK;
        }

        // Dependency order: Materials -> Sections -> Joints -> Frames -> Assignments -> Loads -> Combos
        let phase_order = [
            "MATERIAL PROPERTIES 01 - GENERAL",
            "MATERIAL PROPERTIES 02 - BASIC MECHANICAL PROPERTIES",
            "FRAME SECTION PROPERTIES 01 - GENERAL",
            "JOINT COORDINATES",
            "JOINT RESTRAINT ASSIGNMENTS",
            "CONNECTIVITY - FRAME",
            "FRAME SECTION ASSIGNMENTS",
            "LOAD PATTERN DEFINITIONS",
            "LOAD CASE DEFINITIONS",
            "COMBINATION DEFINITIONS",
        ];

        let mut ordered_tables: Vec<StagedTable> = Vec::new();
        for key in phase_order {
            if let Some(t) = self.api.staged_tables.get(key) {
                ordered_tables.push(t.clone());
            }
        }
        // Add any staged tables not in the standard order
        for (k, t) in &self.api.staged_tables {
            if !phase_order.contains(&k.as_str()) {
                ordered_tables.push(t.clone());
            }
        }

        let mut model_clone = self.api.inner.clone();

        for staged in &ordered_tables {
            let res = apply_single_table(&mut model_clone, staged, fill_import_log, import_log);
            if res != OAPI_OK {
                *num_fatal_errors += 1;
            }
        }

        if *num_fatal_errors > 0 {
            self.api.staged_tables.clear();
            return OAPI_ERR;
        }

        self.api.inner = model_clone;
        self.api.inner.invalidate_results();
        self.api.staged_tables.clear();
        if fill_import_log {
            import_log.push_str("INFO: Staged database tables committed successfully.\n");
        }
        *num_info_msgs += 1;
        OAPI_OK
    }

    /// Convenience wrapper applying staged tables and returning status code.
    pub fn apply_edited_tables_simple(&mut self, fill_import_log: bool) -> i32 {
        let mut fatal = 0;
        let mut err = 0;
        let mut warn = 0;
        let mut info = 0;
        let mut log = String::new();
        self.apply_edited_tables(
            fill_import_log,
            &mut fatal,
            &mut err,
            &mut warn,
            &mut info,
            &mut log,
        )
    }

    /// Discards all staged table edits without modifying the model (`cDatabaseTables.CancelTableEditing`).
    pub fn cancel_table_editing(&mut self) -> i32 {
        self.api.staged_tables.clear();
        OAPI_OK
    }

    // --- Private Row Extraction ---

    fn extract_table_rows(&self, key: &str) -> Vec<BTreeMap<&'static str, String>> {
        let mut rows = Vec::new();
        let model = &self.api.inner;
        let selected: Vec<String> =
            if self.api.selected_cases.is_empty() && self.api.selected_combos.is_empty() {
                model.case_results.keys().cloned().collect()
            } else {
                self.api
                    .selected_cases
                    .iter()
                    .chain(self.api.selected_combos.iter())
                    .cloned()
                    .collect()
            };

        match key {
            "JOINT COORDINATES" => {
                for p in model.points.values() {
                    let mut r = BTreeMap::new();
                    r.insert("Joint", p.name.clone());
                    r.insert("CoordSys", "GLOBAL".to_string());
                    r.insert("CoordType", "Cartesian".to_string());
                    r.insert("XorR", format!("{:.6}", p.x));
                    r.insert("Y", format!("{:.6}", p.y));
                    r.insert("Z", format!("{:.6}", p.z));
                    r.insert(
                        "SpecialJt",
                        if p.special_point { "Yes" } else { "No" }.to_string(),
                    );
                    r.insert("GlobalX", format!("{:.6}", p.x));
                    r.insert("GlobalY", format!("{:.6}", p.y));
                    r.insert("GlobalZ", format!("{:.6}", p.z));
                    rows.push(r);
                }
            }
            "CONNECTIVITY - FRAME" => {
                for f in model.frames.values() {
                    let mut r = BTreeMap::new();
                    r.insert("Frame", f.name.clone());
                    r.insert("JointI", f.point_i.clone());
                    r.insert("JointJ", f.point_j.clone());
                    r.insert("IsCurved", "No".to_string());
                    r.insert("MinSta", format!("{}", f.min_stations));
                    rows.push(r);
                }
            }
            "FRAME SECTION ASSIGNMENTS" => {
                for f in model.frames.values() {
                    let mut r = BTreeMap::new();
                    r.insert("Frame", f.name.clone());
                    r.insert("AnalSect", f.section.clone());
                    r.insert("DesignSect", f.section.clone());
                    r.insert("MatProp", "Default".to_string());
                    rows.push(r);
                }
            }
            "MATERIAL PROPERTIES 01 - GENERAL" => {
                for m in model.materials.values() {
                    let mut r = BTreeMap::new();
                    r.insert("Material", m.name.clone());
                    r.insert("Type", format!("{:?}", m.mat_type));
                    r.insert(
                        "SymType",
                        if m.e_ortho.is_some() {
                            "Orthotropic"
                        } else {
                            "Isotropic"
                        }
                        .to_string(),
                    );
                    r.insert("TempDepend", "No".to_string());
                    r.insert("Color", m.color.clone());
                    rows.push(r);
                }
            }
            "MATERIAL PROPERTIES 02 - BASIC MECHANICAL PROPERTIES" => {
                for m in model.materials.values() {
                    let mut r = BTreeMap::new();
                    r.insert("Material", m.name.clone());
                    r.insert("UnitWeight", format!("{:.6}", m.unit_weight));
                    r.insert("UnitMass", format!("{:.6}", m.unit_mass));
                    r.insert("E1", format!("{:.6}", m.e));
                    r.insert("G12", format!("{:.6}", m.g));
                    r.insert("U12", format!("{:.6}", m.u));
                    r.insert("A1", format!("{:.6e}", m.a));
                    r.insert("Fy", format!("{:.6}", m.fy));
                    r.insert("Fu", format!("{:.6}", m.fu));
                    r.insert("Fc", format!("{:.6}", m.fc));
                    r.insert("Fys", format!("{:.6}", m.fys));
                    rows.push(r);
                }
            }
            "FRAME SECTION PROPERTIES 01 - GENERAL" => {
                for s in model.frame_sections.values() {
                    let mut r = BTreeMap::new();
                    r.insert("SectionName", s.name.clone());
                    r.insert("Material", s.material.clone());
                    r.insert("Shape", format!("{:?}", s.prop_type));
                    r.insert("t3", format!("{:.6}", s.t3));
                    r.insert("t2", format!("{:.6}", s.t2));
                    r.insert("tf", format!("{:.6}", s.tf));
                    r.insert("tw", format!("{:.6}", s.tw));
                    r.insert("Area", format!("{:.6}", s.area));
                    r.insert("TorsConst", format!("{:.6}", s.torsional_j));
                    r.insert("I33", format!("{:.6}", s.i33));
                    r.insert("I22", format!("{:.6}", s.i22));
                    r.insert("AS2", format!("{:.6}", s.as2));
                    r.insert("AS3", format!("{:.6}", s.as3));
                    r.insert("S33", format!("{:.6}", s.s33));
                    r.insert("S22", format!("{:.6}", s.s22));
                    r.insert("Z33", format!("{:.6}", s.z33));
                    r.insert("Z22", format!("{:.6}", s.z22));
                    r.insert("R33", format!("{:.6}", s.r33));
                    r.insert("R22", format!("{:.6}", s.r22));
                    rows.push(r);
                }
            }
            "LOAD PATTERN DEFINITIONS" => {
                for lp in model.load_patterns.values() {
                    let mut r = BTreeMap::new();
                    r.insert("LoadPat", lp.name.clone());
                    r.insert("DesignType", format!("{:?}", lp.pat_type));
                    r.insert("SelfWtMult", format!("{:.6}", lp.self_wt_multiplier));
                    rows.push(r);
                }
            }
            "LOAD CASE DEFINITIONS" => {
                for lc in model.load_cases.values() {
                    let mut r = BTreeMap::new();
                    r.insert("Case", lc.name.clone());
                    r.insert("Type", format!("{:?}", lc.case_type));
                    r.insert("InitialCond", "Zero".to_string());
                    r.insert("DesignType", "DEAD".to_string());
                    rows.push(r);
                }
            }
            "JOINT RESTRAINT ASSIGNMENTS" => {
                for p in model
                    .points
                    .values()
                    .filter(|p| p.restraint.iter().any(|&r| r))
                {
                    let mut r = BTreeMap::new();
                    r.insert("Joint", p.name.clone());
                    r.insert("U1", if p.restraint[0] { "Yes" } else { "No" }.to_string());
                    r.insert("U2", if p.restraint[1] { "Yes" } else { "No" }.to_string());
                    r.insert("U3", if p.restraint[2] { "Yes" } else { "No" }.to_string());
                    r.insert("R1", if p.restraint[3] { "Yes" } else { "No" }.to_string());
                    r.insert("R2", if p.restraint[4] { "Yes" } else { "No" }.to_string());
                    r.insert("R3", if p.restraint[5] { "Yes" } else { "No" }.to_string());
                    rows.push(r);
                }
            }
            "COMBINATION DEFINITIONS" => {
                for cb in model.load_combos.values() {
                    let ctype = format!("{:?}", cb.combo_type);
                    if cb.cases.is_empty() {
                        let mut r = BTreeMap::new();
                        r.insert("ComboName", cb.name.clone());
                        r.insert("ComboType", ctype.clone());
                        r.insert("CaseName", String::new());
                        r.insert("ScaleFactor", "1.0".to_string());
                        rows.push(r);
                    } else {
                        for (cname, sf) in &cb.cases {
                            let mut r = BTreeMap::new();
                            r.insert("ComboName", cb.name.clone());
                            r.insert("ComboType", ctype.clone());
                            r.insert("CaseName", cname.clone());
                            r.insert("ScaleFactor", format!("{:.6}", sf));
                            rows.push(r);
                        }
                    }
                }
            }
            "JOINT DISPLACEMENTS" => {
                for cname in &selected {
                    if let Some(cr) = model.case_results.get(cname) {
                        for (jid, u) in &cr.joint_displacements {
                            let mut r = BTreeMap::new();
                            r.insert("Joint", jid.clone());
                            r.insert("OutputCase", cname.clone());
                            r.insert("StepType", "Static".to_string());
                            r.insert("StepNum", "1".to_string());
                            r.insert("U1", format!("{:.6}", u[0]));
                            r.insert("U2", format!("{:.6}", u[1]));
                            r.insert("U3", format!("{:.6}", u[2]));
                            r.insert("R1", format!("{:.6}", u[3]));
                            r.insert("R2", format!("{:.6}", u[4]));
                            r.insert("R3", format!("{:.6}", u[5]));
                            rows.push(r);
                        }
                    }
                }
            }
            "JOINT REACTIONS" => {
                for cname in &selected {
                    if let Some(cr) = model.case_results.get(cname) {
                        for (jid, re) in &cr.joint_reactions {
                            let mut r = BTreeMap::new();
                            r.insert("Joint", jid.clone());
                            r.insert("OutputCase", cname.clone());
                            r.insert("StepType", "Static".to_string());
                            r.insert("StepNum", "1".to_string());
                            r.insert("F1", format!("{:.4}", re[0]));
                            r.insert("F2", format!("{:.4}", re[1]));
                            r.insert("F3", format!("{:.4}", re[2]));
                            r.insert("M1", format!("{:.4}", re[3]));
                            r.insert("M2", format!("{:.4}", re[4]));
                            r.insert("M3", format!("{:.4}", re[5]));
                            rows.push(r);
                        }
                    }
                }
            }
            "BASE REACTIONS" => {
                for cname in &selected {
                    if let Some(cr) = model.case_results.get(cname) {
                        let mut r = BTreeMap::new();
                        r.insert("OutputCase", cname.clone());
                        r.insert("StepType", "Static".to_string());
                        r.insert("StepNum", "1".to_string());
                        r.insert("GlobalFX", format!("{:.4}", cr.base_reactions[0]));
                        r.insert("GlobalFY", format!("{:.4}", cr.base_reactions[1]));
                        r.insert("GlobalFZ", format!("{:.4}", cr.base_reactions[2]));
                        r.insert("GlobalMX", format!("{:.4}", cr.base_reactions[3]));
                        r.insert("GlobalMY", format!("{:.4}", cr.base_reactions[4]));
                        r.insert("GlobalMZ", format!("{:.4}", cr.base_reactions[5]));
                        rows.push(r);
                    }
                }
            }
            "ELEMENT FORCES - FRAMES" => {
                for cname in &selected {
                    if let Some(cr) = model.case_results.get(cname) {
                        for (fid, stations) in &cr.frame_forces {
                            for st in stations {
                                let mut r = BTreeMap::new();
                                r.insert("Frame", fid.clone());
                                r.insert("Station", format!("{:.4}", st.station));
                                r.insert("OutputCase", cname.clone());
                                r.insert("StepType", "Static".to_string());
                                r.insert("StepNum", "1".to_string());
                                r.insert("P", format!("{:.4}", st.p));
                                r.insert("V2", format!("{:.4}", st.v2));
                                r.insert("V3", format!("{:.4}", st.v3));
                                r.insert("T", format!("{:.4}", st.t));
                                r.insert("M2", format!("{:.4}", st.m2));
                                r.insert("M3", format!("{:.4}", st.m3));
                                rows.push(r);
                            }
                        }
                    }
                }
            }
            "MODAL PERIODS AND FREQUENCIES" => {
                for (cname, cr) in &model.case_results {
                    for m in &cr.modes {
                        let mut r = BTreeMap::new();
                        r.insert("OutputCase", cname.clone());
                        r.insert("StepType", "Mode".to_string());
                        r.insert("StepNum", format!("{}", m.mode_num));
                        r.insert("Period", format!("{:.6}", m.period_sec));
                        r.insert("Frequency", format!("{:.4}", m.freq_hz));
                        r.insert(
                            "CircFreq",
                            format!("{:.4}", m.freq_hz * 2.0 * std::f64::consts::PI),
                        );
                        r.insert("Eigenvalue", format!("{:.4}", m.eigenvalue));
                        rows.push(r);
                    }
                }
            }
            "BUCKLING FACTORS" => {
                for (cname, cr) in &model.case_results {
                    for (idx, &factor) in cr.buckling_factors.iter().enumerate() {
                        let mut r = BTreeMap::new();
                        r.insert("OutputCase", cname.clone());
                        r.insert("StepType", "Buckling".to_string());
                        r.insert("StepNum", format!("{}", idx + 1));
                        r.insert("Factor", format!("{:.6}", factor));
                        rows.push(r);
                    }
                }
            }
            "STEEL DESIGN 1 - SUMMARY DATA" => {
                for res in model.steel_design_results.values() {
                    let mut r = BTreeMap::new();
                    r.insert("Frame", res.frame_name.clone());
                    r.insert("DesignSect", res.section_name.clone());
                    r.insert("DesignCode", res.design_code.clone());
                    r.insert("Combo", res.governing_combo.clone());
                    r.insert("Station", format!("{:.4}", res.governing_station));
                    r.insert("Ratio", format!("{:.4}", res.total_ratio));
                    r.insert("RatioType", "PMM".to_string());
                    r.insert("Status", res.status.clone());
                    rows.push(r);
                }
            }
            "CONCRETE DESIGN 1 - COLUMN SUMMARY DATA" => {
                for res in model
                    .concrete_design_results
                    .values()
                    .filter(|r| r.is_column)
                {
                    let mut r = BTreeMap::new();
                    r.insert("Frame", res.frame_name.clone());
                    r.insert("DesignSect", res.section_name.clone());
                    r.insert("DesignCode", res.design_code.clone());
                    r.insert("Combo", res.governing_combo.clone());
                    r.insert("Station", "0.0000".to_string());
                    r.insert(
                        "TotalAs",
                        format!("{:.6}", res.as_top + res.as_bot + res.as_side),
                    );
                    r.insert("TotalAv", format!("{:.6}", res.av_over_s));
                    r.insert("PMMRatio", format!("{:.4}", res.pmm_ratio));
                    r.insert("Status", res.status.clone());
                    rows.push(r);
                }
            }
            "CONCRETE DESIGN 2 - BEAM SUMMARY DATA" => {
                for res in model
                    .concrete_design_results
                    .values()
                    .filter(|r| !r.is_column)
                {
                    let mut r = BTreeMap::new();
                    r.insert("Frame", res.frame_name.clone());
                    r.insert("DesignSect", res.section_name.clone());
                    r.insert("DesignCode", res.design_code.clone());
                    r.insert("Combo", res.governing_combo.clone());
                    r.insert("Station", "0.0000".to_string());
                    r.insert("TopAs", format!("{:.6}", res.as_top));
                    r.insert("BotAs", format!("{:.6}", res.as_bot));
                    r.insert("SideAs", format!("{:.6}", res.as_side));
                    r.insert("TotalAv", format!("{:.6}", res.av_over_s));
                    r.insert("Status", res.status.clone());
                    rows.push(r);
                }
            }
            _ => {}
        }

        rows
    }
}

// --- Application logic ---

fn apply_single_table(
    model: &mut SapModel,
    table: &StagedTable,
    fill_log: bool,
    log: &mut String,
) -> i32 {
    let k = table.fields.len();
    if k == 0 {
        return OAPI_OK;
    }

    let get_cell = |row_idx: usize, col_name: &str| -> Option<&str> {
        table
            .fields
            .iter()
            .position(|f| f.eq_ignore_ascii_case(col_name))
            .map(|col_idx| table.data[row_idx * k + col_idx].as_str())
    };

    match table.table_key.as_str() {
        "JOINT COORDINATES" => {
            for row in 0..table.num_records {
                let Some(name) = get_cell(row, "Joint") else {
                    if fill_log {
                        log.push_str("ERROR: Missing 'Joint' field in Joint Coordinates\n");
                    }
                    return OAPI_ERR;
                };
                if !is_valid_name(name) {
                    if fill_log {
                        log.push_str(&format!("ERROR: Invalid joint name '{}'\n", name));
                    }
                    return OAPI_ERR;
                }
                let x_str = get_cell(row, "XorR").or_else(|| get_cell(row, "GlobalX"));
                let y_str = get_cell(row, "Y").or_else(|| get_cell(row, "GlobalY"));
                let z_str = get_cell(row, "Z").or_else(|| get_cell(row, "GlobalZ"));

                let (Some(xs), Some(ys), Some(zs)) = (x_str, y_str, z_str) else {
                    if fill_log {
                        log.push_str(&format!(
                            "ERROR: Incomplete coordinates for joint '{}'\n",
                            name
                        ));
                    }
                    return OAPI_ERR;
                };

                let (Ok(x), Ok(y), Ok(z)) = (
                    xs.trim().parse::<f64>(),
                    ys.trim().parse::<f64>(),
                    zs.trim().parse::<f64>(),
                ) else {
                    if fill_log {
                        log.push_str(&format!(
                            "ERROR: Non-numeric coordinate for joint '{}'\n",
                            name
                        ));
                    }
                    return OAPI_ERR;
                };

                if !x.is_finite() || !y.is_finite() || !z.is_finite() {
                    if fill_log {
                        log.push_str(&format!(
                            "ERROR: Non-finite coordinate for joint '{}'\n",
                            name
                        ));
                    }
                    return OAPI_ERR;
                }

                if let Some(p) = model.points.get_mut(name) {
                    p.x = x;
                    p.y = y;
                    p.z = z;
                } else {
                    model
                        .points
                        .insert(name.to_string(), PointObj::new(name, x, y, z));
                }
            }
            OAPI_OK
        }
        "CONNECTIVITY - FRAME" => {
            for row in 0..table.num_records {
                let (Some(name), Some(ji), Some(jj)) = (
                    get_cell(row, "Frame"),
                    get_cell(row, "JointI"),
                    get_cell(row, "JointJ"),
                ) else {
                    if fill_log {
                        log.push_str(
                            "ERROR: Missing Frame/JointI/JointJ fields in Connectivity - Frame\n",
                        );
                    }
                    return OAPI_ERR;
                };
                if !is_valid_name(name) || ji == jj {
                    if fill_log {
                        log.push_str(&format!(
                            "ERROR: Invalid frame name or coincident joints for '{}'\n",
                            name
                        ));
                    }
                    return OAPI_ERR;
                }
                if !model.points.contains_key(ji) || !model.points.contains_key(jj) {
                    if fill_log {
                        log.push_str(&format!(
                            "ERROR: Endpoints '{}' / '{}' not found for frame '{}'\n",
                            ji, jj, name
                        ));
                    }
                    return OAPI_ERR;
                }
                if let Some(f) = model.frames.get_mut(name) {
                    f.point_i = ji.to_string();
                    f.point_j = jj.to_string();
                } else {
                    let sec = model
                        .frame_sections
                        .keys()
                        .next()
                        .cloned()
                        .unwrap_or_else(|| "FSEC1".to_string());
                    model.frames.insert(
                        name.to_string(),
                        sap_model::FrameObj::new(name, ji, jj, &sec),
                    );
                }
            }
            OAPI_OK
        }
        "FRAME SECTION ASSIGNMENTS" => {
            for row in 0..table.num_records {
                let (Some(name), Some(sec)) = (get_cell(row, "Frame"), get_cell(row, "AnalSect"))
                else {
                    if fill_log {
                        log.push_str(
                            "ERROR: Missing Frame or AnalSect in Frame Section Assignments\n",
                        );
                    }
                    return OAPI_ERR;
                };
                if !model.frame_sections.contains_key(sec) {
                    if fill_log {
                        log.push_str(&format!(
                            "ERROR: Section '{}' does not exist in model\n",
                            sec
                        ));
                    }
                    return OAPI_ERR;
                }
                let Some(f) = model.frames.get_mut(name) else {
                    if fill_log {
                        log.push_str(&format!(
                            "ERROR: Frame '{}' does not exist in model\n",
                            name
                        ));
                    }
                    return OAPI_ERR;
                };
                f.section = sec.to_string();
            }
            OAPI_OK
        }
        "MATERIAL PROPERTIES 01 - GENERAL" => {
            for row in 0..table.num_records {
                let Some(name) = get_cell(row, "Material") else {
                    if fill_log {
                        log.push_str("ERROR: Missing Material field in Material Properties 01\n");
                    }
                    return OAPI_ERR;
                };
                if !is_valid_name(name) {
                    return OAPI_ERR;
                }
                let type_str = get_cell(row, "Type").unwrap_or("Steel");
                let mat_type = match type_str.to_ascii_uppercase().as_str() {
                    "STEEL" => MatType::Steel,
                    "CONCRETE" => MatType::Concrete,
                    "NODESIGN" => MatType::NoDesign,
                    "ALUMINUM" => MatType::Aluminum,
                    "COLDFORMED" => MatType::ColdFormed,
                    "REBAR" => MatType::Rebar,
                    "TENDON" => MatType::Tendon,
                    _ => MatType::Steel,
                };
                if let Some(m) = model.materials.get_mut(name) {
                    m.mat_type = mat_type;
                } else {
                    let mut m = Material::default_steel_kip_in();
                    m.name = name.to_string();
                    m.mat_type = mat_type;
                    model.materials.insert(name.to_string(), m);
                }
            }
            OAPI_OK
        }
        "MATERIAL PROPERTIES 02 - BASIC MECHANICAL PROPERTIES" => {
            for row in 0..table.num_records {
                let Some(name) = get_cell(row, "Material") else {
                    return OAPI_ERR;
                };
                let Some(m) = model.materials.get_mut(name) else {
                    if fill_log {
                        log.push_str(&format!("ERROR: Material '{}' not found\n", name));
                    }
                    return OAPI_ERR;
                };
                if let Some(s) = get_cell(row, "E1") {
                    let Ok(v) = s.trim().parse::<f64>() else {
                        if fill_log {
                            log.push_str(&format!("ERROR: Invalid float for E1: '{}'\n", s));
                        }
                        return OAPI_ERR;
                    };
                    if !v.is_finite() || v <= 0.0 {
                        return OAPI_ERR;
                    }
                    m.e = v;
                }
                if let Some(s) = get_cell(row, "G12") {
                    let Ok(v) = s.trim().parse::<f64>() else {
                        if fill_log {
                            log.push_str(&format!("ERROR: Invalid float for G12: '{}'\n", s));
                        }
                        return OAPI_ERR;
                    };
                    if !v.is_finite() || v <= 0.0 {
                        return OAPI_ERR;
                    }
                    m.g = v;
                }
                if let Some(s) = get_cell(row, "U12") {
                    let Ok(v) = s.trim().parse::<f64>() else {
                        if fill_log {
                            log.push_str(&format!("ERROR: Invalid float for U12: '{}'\n", s));
                        }
                        return OAPI_ERR;
                    };
                    if !v.is_finite() || !(0.0..0.5).contains(&v) {
                        return OAPI_ERR;
                    }
                    m.u = v;
                }
                if let Some(s) = get_cell(row, "UnitWeight") {
                    let Ok(v) = s.trim().parse::<f64>() else {
                        if fill_log {
                            log.push_str(&format!(
                                "ERROR: Invalid float for UnitWeight: '{}'\n",
                                s
                            ));
                        }
                        return OAPI_ERR;
                    };
                    if !v.is_finite() || v < 0.0 {
                        return OAPI_ERR;
                    }
                    m.unit_weight = v;
                }
                if let Some(s) = get_cell(row, "UnitMass") {
                    let Ok(v) = s.trim().parse::<f64>() else {
                        if fill_log {
                            log.push_str(&format!("ERROR: Invalid float for UnitMass: '{}'\n", s));
                        }
                        return OAPI_ERR;
                    };
                    if !v.is_finite() || v < 0.0 {
                        return OAPI_ERR;
                    }
                    m.unit_mass = v;
                }
                if let Some(s) = get_cell(row, "A1") {
                    let Ok(v) = s.trim().parse::<f64>() else {
                        if fill_log {
                            log.push_str(&format!("ERROR: Invalid float for A1: '{}'\n", s));
                        }
                        return OAPI_ERR;
                    };
                    if !v.is_finite() || v < 0.0 {
                        return OAPI_ERR;
                    }
                    m.a = v;
                }
                if let Some(s) = get_cell(row, "Fy") {
                    let Ok(v) = s.trim().parse::<f64>() else {
                        if fill_log {
                            log.push_str(&format!("ERROR: Invalid float for Fy: '{}'\n", s));
                        }
                        return OAPI_ERR;
                    };
                    if !v.is_finite() || v < 0.0 {
                        return OAPI_ERR;
                    }
                    m.fy = v;
                }
                if let Some(s) = get_cell(row, "Fu") {
                    let Ok(v) = s.trim().parse::<f64>() else {
                        if fill_log {
                            log.push_str(&format!("ERROR: Invalid float for Fu: '{}'\n", s));
                        }
                        return OAPI_ERR;
                    };
                    if !v.is_finite() || v < 0.0 {
                        return OAPI_ERR;
                    }
                    m.fu = v;
                }
                if let Some(s) = get_cell(row, "Fc") {
                    let Ok(v) = s.trim().parse::<f64>() else {
                        if fill_log {
                            log.push_str(&format!("ERROR: Invalid float for Fc: '{}'\n", s));
                        }
                        return OAPI_ERR;
                    };
                    if !v.is_finite() || v < 0.0 {
                        return OAPI_ERR;
                    }
                    m.fc = v;
                }
                if let Some(s) = get_cell(row, "Fys") {
                    let Ok(v) = s.trim().parse::<f64>() else {
                        if fill_log {
                            log.push_str(&format!("ERROR: Invalid float for Fys: '{}'\n", s));
                        }
                        return OAPI_ERR;
                    };
                    if !v.is_finite() || v < 0.0 {
                        return OAPI_ERR;
                    }
                    m.fys = v;
                }
            }
            OAPI_OK
        }
        "FRAME SECTION PROPERTIES 01 - GENERAL" => {
            for row in 0..table.num_records {
                let Some(name) = get_cell(row, "SectionName").or_else(|| get_cell(row, "Section"))
                else {
                    return OAPI_ERR;
                };
                if !is_valid_name(name) {
                    return OAPI_ERR;
                }
                let mat = get_cell(row, "Material").unwrap_or("A992Fy50");
                if !model.materials.contains_key(mat) {
                    if fill_log {
                        log.push_str(&format!(
                            "ERROR: Material '{}' not found for section '{}'\n",
                            mat, name
                        ));
                    }
                    return OAPI_ERR;
                }

                let parse_f64 = |field_name: &str, default_val: f64| -> Result<f64, ()> {
                    if let Some(s) = get_cell(row, field_name) {
                        let v = s.trim().parse::<f64>().map_err(|_| ())?;
                        if v.is_finite() {
                            Ok(v)
                        } else {
                            Err(())
                        }
                    } else {
                        Ok(default_val)
                    }
                };

                let parse_opt = |field_name: &str| -> Result<Option<f64>, ()> {
                    if let Some(s) = get_cell(row, field_name) {
                        let v = s.trim().parse::<f64>().map_err(|_| ())?;
                        if v.is_finite() && v >= 0.0 {
                            Ok(Some(v))
                        } else {
                            Err(())
                        }
                    } else {
                        Ok(None)
                    }
                };

                let Ok(area) = parse_f64("Area", 10.0) else {
                    return OAPI_ERR;
                };
                let Ok(i33) = parse_f64("I33", 100.0) else {
                    return OAPI_ERR;
                };
                let Ok(i22) = parse_f64("I22", 50.0) else {
                    return OAPI_ERR;
                };
                let Ok(j) = parse_f64("TorsConst", 1.0) else {
                    return OAPI_ERR;
                };
                let Ok(t3) = parse_f64("t3", 10.0) else {
                    return OAPI_ERR;
                };
                let Ok(t2) = parse_f64("t2", 10.0) else {
                    return OAPI_ERR;
                };
                let Ok(tf) = parse_f64("tf", 0.5) else {
                    return OAPI_ERR;
                };
                let Ok(tw) = parse_f64("tw", 0.5) else {
                    return OAPI_ERR;
                };

                let Ok(opt_as2) = parse_opt("AS2") else {
                    return OAPI_ERR;
                };
                let Ok(opt_as3) = parse_opt("AS3") else {
                    return OAPI_ERR;
                };
                let Ok(opt_s33) = parse_opt("S33") else {
                    return OAPI_ERR;
                };
                let Ok(opt_s22) = parse_opt("S22") else {
                    return OAPI_ERR;
                };
                let Ok(opt_z33) = parse_opt("Z33") else {
                    return OAPI_ERR;
                };
                let Ok(opt_z22) = parse_opt("Z22") else {
                    return OAPI_ERR;
                };
                let Ok(opt_r33) = parse_opt("R33") else {
                    return OAPI_ERR;
                };
                let Ok(opt_r22) = parse_opt("R22") else {
                    return OAPI_ERR;
                };

                if area <= 0.0 || i33 <= 0.0 || i22 <= 0.0 {
                    return OAPI_ERR;
                }

                if let Some(s) = model.frame_sections.get_mut(name) {
                    s.material = mat.to_string();
                    s.area = area;
                    s.i33 = i33;
                    s.i22 = i22;
                    s.torsional_j = j;
                    s.t3 = t3;
                    s.t2 = t2;
                    s.tf = tf;
                    s.tw = tw;
                    if let Some(v) = opt_as2 {
                        s.as2 = v;
                    }
                    if let Some(v) = opt_as3 {
                        s.as3 = v;
                    }
                    if let Some(v) = opt_s33 {
                        s.s33 = v;
                    }
                    if let Some(v) = opt_s22 {
                        s.s22 = v;
                    }
                    if let Some(v) = opt_z33 {
                        s.z33 = v;
                    }
                    if let Some(v) = opt_z22 {
                        s.z22 = v;
                    }
                    if let Some(v) = opt_r33 {
                        s.r33 = v;
                    }
                    if let Some(v) = opt_r22 {
                        s.r22 = v;
                    }
                } else {
                    let mut sec = FrameSection::default();
                    sec.name = name.to_string();
                    sec.material = mat.to_string();
                    sec.prop_type = FramePropType::I;
                    sec.area = area;
                    sec.i33 = i33;
                    sec.i22 = i22;
                    sec.torsional_j = j;
                    sec.t3 = t3;
                    sec.t2 = t2;
                    sec.tf = tf;
                    sec.tw = tw;
                    if let Some(v) = opt_as2 {
                        sec.as2 = v;
                    }
                    if let Some(v) = opt_as3 {
                        sec.as3 = v;
                    }
                    if let Some(v) = opt_s33 {
                        sec.s33 = v;
                    }
                    if let Some(v) = opt_s22 {
                        sec.s22 = v;
                    }
                    if let Some(v) = opt_z33 {
                        sec.z33 = v;
                    }
                    if let Some(v) = opt_z22 {
                        sec.z22 = v;
                    }
                    if let Some(v) = opt_r33 {
                        sec.r33 = v;
                    }
                    if let Some(v) = opt_r22 {
                        sec.r22 = v;
                    }
                    model.frame_sections.insert(name.to_string(), sec);
                }
            }
            OAPI_OK
        }
        "LOAD PATTERN DEFINITIONS" => {
            for row in 0..table.num_records {
                let Some(name) = get_cell(row, "LoadPat") else {
                    return OAPI_ERR;
                };
                if !is_valid_name(name) {
                    return OAPI_ERR;
                };
                let sw_mult = get_cell(row, "SelfWtMult")
                    .and_then(|s| s.trim().parse::<f64>().ok())
                    .unwrap_or(0.0);
                if !sw_mult.is_finite() {
                    return OAPI_ERR;
                };

                if let Some(lp) = model.load_patterns.get_mut(name) {
                    lp.self_wt_multiplier = sw_mult;
                } else {
                    model.load_patterns.insert(
                        name.to_string(),
                        LoadPattern {
                            name: name.to_string(),
                            pat_type: LoadPatternType::Dead,
                            self_wt_multiplier: sw_mult,
                        },
                    );
                }
            }
            OAPI_OK
        }
        "LOAD CASE DEFINITIONS" => {
            for row in 0..table.num_records {
                let Some(name) = get_cell(row, "Case") else {
                    return OAPI_ERR;
                };
                if !is_valid_name(name) {
                    return OAPI_ERR;
                };

                let pattern = get_cell(row, "Pattern")
                    .or_else(|| get_cell(row, "LoadPat"))
                    .or_else(|| get_cell(row, "DesignType"))
                    .filter(|p| !p.trim().is_empty())
                    .map(|s| s.trim().to_string())
                    .or_else(|| {
                        if model.load_patterns.contains_key(name) {
                            Some(name.to_string())
                        } else {
                            let mut pat_names: Vec<&String> = model.load_patterns.keys().collect();
                            pat_names.sort_by_key(|p| std::cmp::Reverse(p.len()));
                            let upper_name = name.to_ascii_uppercase();
                            pat_names
                                .into_iter()
                                .find(|pat| {
                                    let p_up = pat.to_ascii_uppercase();
                                    p_up != "DEAD"
                                        && (upper_name.contains(&p_up)
                                            || p_up.contains(&upper_name))
                                })
                                .cloned()
                        }
                    })
                    .or_else(|| {
                        let non_dead: Vec<&String> = model
                            .load_patterns
                            .keys()
                            .filter(|k| k.as_str() != "DEAD")
                            .collect();
                        if non_dead.len() == 1 {
                            Some(non_dead[0].clone())
                        } else {
                            None
                        }
                    })
                    .unwrap_or_else(|| "DEAD".to_string());

                let case_type = if let Some(t) = get_cell(row, "Type") {
                    match t.to_ascii_uppercase().as_str() {
                        "NONLINEARSTATIC" | "NONLINEAR_STATIC" => LoadCaseType::NonlinearStatic,
                        "MODAL" => LoadCaseType::Modal,
                        "RESPONSESPECTRUM" | "RESPONSE_SPECTRUM" => LoadCaseType::ResponseSpectrum,
                        "LINEARHISTORY" | "LINEAR_HISTORY" => LoadCaseType::LinearHistory,
                        "NONLINEARHISTORY" | "NONLINEAR_HISTORY" => LoadCaseType::NonlinearHistory,
                        _ => LoadCaseType::LinearStatic,
                    }
                } else {
                    LoadCaseType::LinearStatic
                };

                if let Some(lc) = model.load_cases.get_mut(name) {
                    lc.case_type = case_type;
                    if lc.loads.is_empty() && model.load_patterns.contains_key(&pattern) {
                        lc.loads.push(LoadCaseAssignment {
                            load_name: pattern.clone(),
                            scale_factor: 1.0,
                        });
                    }
                } else {
                    let mut lc = LoadCase::linear_static(name, &pattern, 1.0);
                    lc.case_type = case_type;
                    model.load_cases.insert(name.to_string(), lc);
                }
            }
            OAPI_OK
        }
        "JOINT RESTRAINT ASSIGNMENTS" => {
            let parse_bool = |s: &str| -> bool {
                let t = s.trim();
                t.eq_ignore_ascii_case("yes") || t.eq_ignore_ascii_case("true") || t == "1"
            };

            for row in 0..table.num_records {
                let Some(name) = get_cell(row, "Joint") else {
                    return OAPI_ERR;
                };
                let Some(p) = model.points.get_mut(name) else {
                    if fill_log {
                        log.push_str(&format!(
                            "ERROR: Joint '{}' not found for restraint assignment\n",
                            name
                        ));
                    }
                    return OAPI_ERR;
                };
                let u1 = get_cell(row, "U1")
                    .map(parse_bool)
                    .unwrap_or(p.restraint[0]);
                let u2 = get_cell(row, "U2")
                    .map(parse_bool)
                    .unwrap_or(p.restraint[1]);
                let u3 = get_cell(row, "U3")
                    .map(parse_bool)
                    .unwrap_or(p.restraint[2]);
                let r1 = get_cell(row, "R1")
                    .map(parse_bool)
                    .unwrap_or(p.restraint[3]);
                let r2 = get_cell(row, "R2")
                    .map(parse_bool)
                    .unwrap_or(p.restraint[4]);
                let r3 = get_cell(row, "R3")
                    .map(parse_bool)
                    .unwrap_or(p.restraint[5]);
                p.restraint = [u1, u2, u3, r1, r2, r3];
            }
            OAPI_OK
        }
        "COMBINATION DEFINITIONS" => {
            let mut seen_combos = HashSet::new();
            for row in 0..table.num_records {
                let Some(name) = get_cell(row, "ComboName") else {
                    return OAPI_ERR;
                };
                if !is_valid_name(name) {
                    return OAPI_ERR;
                };
                let case_name = get_cell(row, "CaseName").unwrap_or("");
                let sf = get_cell(row, "ScaleFactor")
                    .and_then(|s| s.trim().parse::<f64>().ok())
                    .unwrap_or(1.0);
                if !sf.is_finite() {
                    return OAPI_ERR;
                };

                let combo = model
                    .load_combos
                    .entry(name.to_string())
                    .or_insert_with(|| LoadCombo {
                        name: name.to_string(),
                        combo_type: ComboType::LinearAdd,
                        cases: Vec::new(),
                    });

                if seen_combos.insert(name.to_string()) {
                    combo.cases.clear();
                }

                if let Some(ct) = get_cell(row, "ComboType") {
                    combo.combo_type = match ct.to_ascii_uppercase().as_str() {
                        "ENVELOPE" => ComboType::Envelope,
                        "ABSOLUTEADD" | "ABSADD" => ComboType::AbsAdd,
                        "SRSS" => ComboType::Srss,
                        _ => ComboType::LinearAdd,
                    };
                }

                if !case_name.is_empty() {
                    combo.cases.push((case_name.to_string(), sf));
                }
            }
            OAPI_OK
        }
        _ => OAPI_OK,
    }
}
