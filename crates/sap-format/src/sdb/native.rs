//! Lossless native v19 stream inventory and source-preserving serialization.
//!
//! This is deliberately separate from `pack_model_to_sdb`: preserving a vendor document
//! is not the same operation as initializing all native records from a `SapModel`.

mod class_versions;
mod export_guard;
mod verified_file;

use super::*;
use serde::Serialize;
use std::collections::BTreeMap;

#[derive(Debug, Clone, Serialize)]
pub struct NativeFieldSpan {
    pub name: Option<String>,
    pub wire_type: String,
    pub offset: usize,
    pub length: usize,
}

#[derive(Debug, Clone, Serialize)]
pub struct NativeRecord {
    pub class_id: String,
    pub offset: usize,
    pub length: usize,
    pub fields: Vec<NativeFieldSpan>,
}

#[derive(Debug, Clone, Serialize)]
pub struct NativeTopLevelField {
    pub name: String,
    pub wire_type: String,
    pub offset: usize,
    pub length: usize,
}

/// Compact inventory suitable for the CLI. Nested record fields remain available separately.
#[derive(Debug, Clone, Serialize)]
pub struct NativeSdbInventory {
    pub stream_bytes: usize,
    pub parsed_bytes: usize,
    pub top_level: Vec<NativeTopLevelField>,
    pub record_count: usize,
    pub classes: BTreeMap<String, usize>,
}

#[derive(Debug, Clone)]
pub struct NativeSdbDocument {
    source: SdbFile,
    original_bytes: Option<Vec<u8>>,
    inventory: NativeSdbInventory,
    records: Vec<NativeRecord>,
}

impl NativeSdbDocument {
    pub fn inventory(&self) -> &NativeSdbInventory {
        &self.inventory
    }
    pub fn records(&self) -> &[NativeRecord] {
        &self.records
    }
    pub fn to_sdb(&self) -> SdbFile {
        self.source.clone()
    }

    /// Without edits, documents opened from bytes/files retain the exact original container,
    /// including gzip metadata and compressed bytes. An edited document preserves the complete
    /// decompressed stream except for explicit field patches and regenerates its gzip container.
    pub fn to_bytes(&self) -> Result<Vec<u8>, FormatError> {
        self.original_bytes
            .clone()
            .map(Ok)
            .unwrap_or_else(|| write_sdb_bytes(&self.source))
    }

    /// Low-level scalar edit using a verified native field span. This does not invalidate
    /// analysis results or recompute geometry caches. It is not a validated model-editing API.
    pub fn set_f64(
        &mut self,
        record_index: usize,
        field: &str,
        value: f64,
    ) -> Result<(), FormatError> {
        if !value.is_finite() {
            return Err(FormatError::InvalidSdb(
                "Native scalar patch must be finite".into(),
            ));
        }
        let record = self
            .records
            .get(record_index)
            .ok_or_else(|| FormatError::InvalidSdb("Native record index is out of range".into()))?;
        let span = record
            .fields
            .iter()
            .find(|span| span.name.as_deref() == Some(field))
            .ok_or_else(|| {
                FormatError::InvalidSdb(format!(
                    "Native field {} not found in {}",
                    field, record.class_id
                ))
            })?;
        if span.wire_type != "r8" || span.length != 8 {
            return Err(FormatError::InvalidSdb(format!(
                "Native field {} is not an r8 scalar",
                field
            )));
        }
        self.source.decompressed_stream[span.offset..span.offset + 8]
            .copy_from_slice(&value.to_le_bytes());
        self.original_bytes = None;
        Ok(())
    }
}

pub fn inspect_native_sdb_file(path: impl AsRef<Path>) -> Result<NativeSdbDocument, FormatError> {
    inspect_native_sdb_bytes(&std::fs::read(path)?)
}

pub fn inspect_native_sdb_bytes(bytes: &[u8]) -> Result<NativeSdbDocument, FormatError> {
    let mut document = inspect_native_sdb(&read_sdb_bytes(bytes)?)?;
    document.original_bytes = Some(bytes.to_vec());
    Ok(document)
}

/// Traverse every native top-level field and nested schema-known structure to EOF.
/// Model-unsupported records are inventoried and preserved verbatim. Unknown class IDs
/// fail decoding rather than guessing a boundary; the caller's original input remains untouched.
pub fn inspect_native_sdb(source: &SdbFile) -> Result<NativeSdbDocument, FormatError> {
    if source.decompressed_stream.starts_with(RUST_SDB_S2K_MAGIC) {
        return Err(FormatError::InvalidSdb(
            "Private reconstruction payload is not a native SDB stream".into(),
        ));
    }
    let mut reader = BinarySdbReader::new(&source.decompressed_stream);
    reader.inventory = Some(Vec::new());
    let top_level = traverse(&mut reader)?;
    if reader.pos != source.decompressed_stream.len() {
        return Err(FormatError::InvalidSdb(format!(
            "Native traversal left {} bytes unread",
            source.decompressed_stream.len() - reader.pos
        )));
    }
    let records = reader.inventory.take().unwrap();
    let mut classes = BTreeMap::new();
    for record in &records {
        *classes.entry(record.class_id.clone()).or_insert(0) += 1;
    }
    let inventory = NativeSdbInventory {
        stream_bytes: source.decompressed_stream.len(),
        parsed_bytes: reader.pos,
        top_level,
        record_count: records.len(),
        classes,
    };
    Ok(NativeSdbDocument {
        source: source.clone(),
        original_bytes: None,
        inventory,
        records,
    })
}

fn consume(
    reader: &mut BinarySdbReader<'_>,
    fields: &mut Vec<NativeTopLevelField>,
    name: &str,
    kind: &str,
) -> Result<bool, FormatError> {
    let offset = reader.pos;
    let flag = match kind {
        "string" => {
            reader.read_string()?;
            false
        }
        "i4" => {
            reader.read_i32()?;
            false
        }
        "bool" => reader.read_bool()?,
        "struct" => {
            reader.read_struct(false)?;
            false
        }
        "struct[]" => {
            reader.read_struct_array(false)?;
            false
        }
        _ => {
            return Err(FormatError::InvalidSdb(format!(
                "Unknown top-level kind {}",
                kind
            )))
        }
    };
    fields.push(NativeTopLevelField {
        name: name.into(),
        wire_type: kind.into(),
        offset,
        length: reader.pos - offset,
    });
    Ok(flag)
}

// Exact top-level sequence recovered by the original full-stream decoder.
// Unlike docs/re/formats_and_sections.md section 2.3, the supplied files use a string
// FileVersionInFile and this sequence. Fixture traversal tests enforce EOF consumption.
fn traverse(reader: &mut BinarySdbReader<'_>) -> Result<Vec<NativeTopLevelField>, FormatError> {
    let mut fields = Vec::new();
    let mut flags = HashMap::new();
    consume(reader, &mut fields, "NameInFile", "string")?;
    consume(reader, &mut fields, "ProgramVersionInFile", "string")?;
    consume(reader, &mut fields, "FileVersionInFile", "string")?;
    consume(reader, &mut fields, "ProjectStartUp", "struct")?;
    consume(reader, &mut fields, "ProjectInformation", "struct")?;
    consume(reader, &mut fields, "Standards", "struct[]")?;
    consume(reader, &mut fields, "AnalysisModelDateTimeStamp", "struct")?;
    consume(reader, &mut fields, "RunParameters", "struct")?;
    consume(reader, &mut fields, "PresentCoordSystem", "i4")?;
    consume(reader, &mut fields, "CoordSystems", "struct[]")?;
    consume(reader, &mut fields, "DevElev", "struct[]")?;
    consume(reader, &mut fields, "ReferenceLine", "struct[]")?;
    consume(reader, &mut fields, "ReferencePlane", "struct[]")?;
    consume(reader, &mut fields, "GenRefLine", "struct[]")?;
    consume(reader, &mut fields, "Materials", "struct[]")?;
    consume(reader, &mut fields, "Section", "struct[]")?;
    consume(reader, &mut fields, "Auto", "struct[]")?;
    consume(reader, &mut fields, "VS", "struct[]")?;
    consume(reader, &mut fields, "Rebar", "struct[]")?;
    consume(reader, &mut fields, "SDSection", "struct[]")?;
    consume(reader, &mut fields, "JoistData", "struct[]")?;
    consume(reader, &mut fields, "ShellSection", "struct[]")?;
    consume(reader, &mut fields, "SolidProp", "struct[]")?;
    consume(reader, &mut fields, "NLLinkProp", "struct[]")?;
    consume(reader, &mut fields, "NLLinkFDProp", "struct[]")?;
    consume(reader, &mut fields, "CableProp", "struct[]")?;
    consume(reader, &mut fields, "TendonProp", "struct[]")?;
    consume(reader, &mut fields, "NamedFModifier", "struct[]")?;
    consume(reader, &mut fields, "NamedFRelease", "struct[]")?;
    consume(reader, &mut fields, "NamedCModifier", "struct[]")?;
    consume(reader, &mut fields, "NamedShModifier", "struct[]")?;
    consume(reader, &mut fields, "PointObjects", "struct[]")?;
    consume(reader, &mut fields, "LineObjects", "struct[]")?;
    consume(reader, &mut fields, "FrameExternalResult", "struct[]")?;
    consume(reader, &mut fields, "AreaObjects", "struct[]")?;
    consume(reader, &mut fields, "SolidObjects", "struct[]")?;
    consume(reader, &mut fields, "NLLinkObjects", "struct[]")?;
    consume(reader, &mut fields, "Restraint", "struct[]")?;
    consume(reader, &mut fields, "Constraint", "struct[]")?;
    consume(reader, &mut fields, "CJoint", "struct[]")?;
    consume(reader, &mut fields, "Pattern", "struct[]")?;
    consume(reader, &mut fields, "PJoint", "struct[]")?;
    consume(reader, &mut fields, "Spring", "struct[]")?;
    consume(reader, &mut fields, "Mass", "struct[]")?;
    consume(reader, &mut fields, "PanelZone", "struct[]")?;
    consume(reader, &mut fields, "GenDispl", "struct[]")?;
    consume(reader, &mut fields, "GD", "struct[]")?;
    consume(reader, &mut fields, "MatTemp", "struct[]")?;
    consume(reader, &mut fields, "PDForce", "struct[]")?;
    consume(reader, &mut fields, "FModifier", "struct[]")?;
    consume(reader, &mut fields, "FFixity", "struct[]")?;
    consume(reader, &mut fields, "FOffset", "struct[]")?;
    consume(reader, &mut fields, "FSpring", "struct[]")?;
    consume(reader, &mut fields, "FMass", "struct[]")?;
    consume(reader, &mut fields, "FFireProofing", "struct[]")?;
    consume(reader, &mut fields, "ShModifier", "struct[]")?;
    consume(reader, &mut fields, "ShSpring", "struct[]")?;
    consume(reader, &mut fields, "ShMass", "struct[]")?;
    consume(reader, &mut fields, "ShMesh", "struct[]")?;
    consume(reader, &mut fields, "SSpring", "struct[]")?;
    consume(reader, &mut fields, "LoadPattern", "struct[]")?;
    consume(reader, &mut fields, "Seismic", "struct[]")?;
    consume(reader, &mut fields, "Wind", "struct[]")?;
    consume(reader, &mut fields, "MultiStepMove", "struct[]")?;
    consume(reader, &mut fields, "UndeformedGeometryData", "struct")?;
    consume(reader, &mut fields, "AnalysisCase", "struct[]")?;
    consume(reader, &mut fields, "Schedule", "struct[]")?;
    consume(reader, &mut fields, "MassSource", "struct[]")?;
    consume(reader, &mut fields, "Combo", "struct[]")?;
    consume(reader, &mut fields, "AutoComboOptions", "struct[]")?;
    consume(reader, &mut fields, "JFLoad", "struct[]")?;
    consume(reader, &mut fields, "JGDLoad", "struct[]")?;
    consume(reader, &mut fields, "FGLoad", "struct[]")?;
    consume(reader, &mut fields, "FCLoad", "struct[]")?;
    consume(reader, &mut fields, "FDLoad", "struct[]")?;
    consume(reader, &mut fields, "FTLoad", "struct[]")?;
    consume(reader, &mut fields, "FSLoad", "struct[]")?;
    consume(reader, &mut fields, "FDefLoad", "struct[]")?;
    consume(reader, &mut fields, "FTFLoad", "struct[]")?;
    consume(reader, &mut fields, "PTLoad", "struct[]")?;
    consume(reader, &mut fields, "WindOverwriteLine", "struct[]")?;
    consume(reader, &mut fields, "ShGLoad", "struct[]")?;
    consume(reader, &mut fields, "ShULoad", "struct[]")?;
    consume(reader, &mut fields, "ShUFLoad", "struct[]")?;
    consume(reader, &mut fields, "ShSPLoad", "struct[]")?;
    consume(reader, &mut fields, "ShPPLoad", "struct[]")?;
    consume(reader, &mut fields, "ShTLoad", "struct[]")?;
    consume(reader, &mut fields, "ShSLoad", "struct[]")?;
    consume(reader, &mut fields, "ShWPLoad", "struct[]")?;
    consume(reader, &mut fields, "AsoRtLoad", "struct[]")?;
    consume(reader, &mut fields, "SoGLoad", "struct[]")?;
    consume(reader, &mut fields, "SoPPLoad", "struct[]")?;
    consume(reader, &mut fields, "SoSPLoad", "struct[]")?;
    consume(reader, &mut fields, "SoTLoad", "struct[]")?;
    consume(reader, &mut fields, "SoSLoad", "struct[]")?;
    consume(reader, &mut fields, "NLGLoad", "struct[]")?;
    consume(reader, &mut fields, "NLDefLoad", "struct[]")?;
    consume(reader, &mut fields, "NLTFLoad", "struct[]")?;
    consume(reader, &mut fields, "Seastate", "struct[]")?;
    consume(reader, &mut fields, "Wave", "struct[]")?;
    consume(reader, &mut fields, "WaveCharacteristic", "struct[]")?;
    consume(reader, &mut fields, "WaveCurrent", "struct[]")?;
    consume(reader, &mut fields, "WaveMarineGrowth", "struct[]")?;
    consume(reader, &mut fields, "WaveDICoeff", "struct[]")?;
    consume(reader, &mut fields, "WaveWind", "struct[]")?;
    consume(reader, &mut fields, "WaveOverwriteLine", "struct[]")?;
    consume(reader, &mut fields, "WaveOverwriteArea", "struct[]")?;
    consume(reader, &mut fields, "WaveOverwriteSolid", "struct[]")?;
    consume(reader, &mut fields, "Funcshun", "struct[]")?;
    consume(reader, &mut fields, "TV", "struct[]")?;
    consume(reader, &mut fields, "BridgeMiscParameters", "struct")?;
    consume(reader, &mut fields, "BridgeLayout", "struct[]")?;
    consume(reader, &mut fields, "BridgeAbutmentNew", "struct[]")?;
    consume(reader, &mut fields, "BridgeFoundationSpring", "struct[]")?;
    consume(reader, &mut fields, "BridgeSection", "struct[]")?;
    consume(reader, &mut fields, "BridgeBent", "struct[]")?;
    consume(reader, &mut fields, "BridgeBearing", "struct[]")?;
    consume(reader, &mut fields, "BridgeRestrainer", "struct[]")?;
    consume(reader, &mut fields, "BridgeDiaphragm", "struct[]")?;
    consume(reader, &mut fields, "BridgeSplice", "struct[]")?;
    consume(reader, &mut fields, "Variation", "struct[]")?;
    consume(reader, &mut fields, "BridgeSuperstructureLoad", "struct[]")?;
    consume(reader, &mut fields, "BridgeTempGradLoad", "struct[]")?;
    consume(reader, &mut fields, "BridgeObject", "struct[]")?;
    consume(reader, &mut fields, "BridgeSCGroup", "struct[]")?;
    consume(reader, &mut fields, "Lane", "struct[]")?;
    consume(reader, &mut fields, "Vehicle", "struct[]")?;
    consume(reader, &mut fields, "VClass", "struct[]")?;
    consume(reader, &mut fields, "BR", "struct")?;
    consume(reader, &mut fields, "VehRespCompPoint", "struct[]")?;
    consume(reader, &mut fields, "VehRespCompLine", "struct[]")?;
    consume(reader, &mut fields, "VehRespCompArea", "struct[]")?;
    consume(reader, &mut fields, "VehRespCompSolid", "struct[]")?;
    consume(reader, &mut fields, "VehRespCompNLLink", "struct[]")?;
    consume(reader, &mut fields, "HingeProp", "struct[]")?;
    flags.insert(
        "FrameHingeAssignDimed",
        consume(reader, &mut fields, "FrameHingeAssignDimed", "bool")?,
    );
    if flags.get("FrameHingeAssignDimed").copied().unwrap_or(false) {
        consume(reader, &mut fields, "FrameHingeAssign", "struct[]")?;
    }
    consume(reader, &mut fields, "POParamVD", "struct[]")?;
    consume(reader, &mut fields, "POParamATC40", "struct[]")?;
    consume(reader, &mut fields, "POParamFEMA356", "struct[]")?;
    consume(reader, &mut fields, "POParamFEMA440DM", "struct[]")?;
    consume(reader, &mut fields, "POParamFEMA440EL", "struct[]")?;
    consume(reader, &mut fields, "FLateralBrace", "struct[]")?;
    consume(reader, &mut fields, "SteelPreferences", "struct")?;
    flags.insert(
        "SteelResultsAvailable",
        consume(reader, &mut fields, "SteelResultsAvailable", "bool")?,
    );
    flags.insert(
        "SteelDefaultsSet",
        consume(reader, &mut fields, "SteelDefaultsSet", "bool")?,
    );
    consume(reader, &mut fields, "SteelData", "struct[]")?;
    if flags.get("SteelResultsAvailable").copied().unwrap_or(false) {
        consume(reader, &mut fields, "SteelEnvelopes", "struct[]")?;
    }
    consume(reader, &mut fields, "ConcretePreferences", "struct")?;
    flags.insert(
        "ConcreteResultsAvailable",
        consume(reader, &mut fields, "ConcreteResultsAvailable", "bool")?,
    );
    flags.insert(
        "ConcreteEnvelopesAvailable",
        consume(reader, &mut fields, "ConcreteEnvelopesAvailable", "bool")?,
    );
    flags.insert(
        "ConcreteDefaultsSet",
        consume(reader, &mut fields, "ConcreteDefaultsSet", "bool")?,
    );
    if flags.get("ConcreteDefaultsSet").copied().unwrap_or(false) {
        consume(reader, &mut fields, "ConcreteData", "struct[]")?;
    }
    if flags
        .get("ConcreteEnvelopesAvailable")
        .copied()
        .unwrap_or(false)
    {
        consume(reader, &mut fields, "ConcreteEnvelopes", "struct[]")?;
    }
    consume(reader, &mut fields, "AluminumPreferences", "struct")?;
    flags.insert(
        "AluminumResultsAvailable",
        consume(reader, &mut fields, "AluminumResultsAvailable", "bool")?,
    );
    flags.insert(
        "AluminumDefaultsSet",
        consume(reader, &mut fields, "AluminumDefaultsSet", "bool")?,
    );
    if flags.get("AluminumDefaultsSet").copied().unwrap_or(false) {
        consume(reader, &mut fields, "AluminumData", "struct[]")?;
    }
    if flags
        .get("AluminumResultsAvailable")
        .copied()
        .unwrap_or(false)
    {
        consume(reader, &mut fields, "AluminumEnvelopes", "struct[]")?;
    }
    consume(reader, &mut fields, "ColdFormedPreferences", "struct")?;
    flags.insert(
        "ColdFormedResultsAvailable",
        consume(reader, &mut fields, "ColdFormedResultsAvailable", "bool")?,
    );
    flags.insert(
        "ColdFormedDefaultsSet",
        consume(reader, &mut fields, "ColdFormedDefaultsSet", "bool")?,
    );
    if flags.get("ColdFormedDefaultsSet").copied().unwrap_or(false) {
        consume(reader, &mut fields, "ColdFormedData", "struct[]")?;
    }
    if flags
        .get("ColdFormedResultsAvailable")
        .copied()
        .unwrap_or(false)
    {
        consume(reader, &mut fields, "ColdFormedEnvelopes", "struct[]")?;
    }
    consume(reader, &mut fields, "OptimizationD", "struct[]")?;
    consume(reader, &mut fields, "OptimizationT", "struct[]")?;
    flags.insert(
        "PunchAPIResultsAvailable",
        consume(reader, &mut fields, "PunchAPIResultsAvailable", "bool")?,
    );
    flags.insert(
        "PunchAPIDefaultsSet",
        consume(reader, &mut fields, "PunchAPIDefaultsSet", "bool")?,
    );
    if flags.get("PunchAPIDefaultsSet").copied().unwrap_or(false) {
        consume(reader, &mut fields, "PunchAPIData", "struct[]")?;
    }
    if flags.get("PunchAPIDefaultsSet").copied().unwrap_or(false)
        && flags
            .get("PunchAPIResultsAvailable")
            .copied()
            .unwrap_or(false)
    {
        consume(reader, &mut fields, "PunchAPIEnvelopes", "struct[]")?;
    }
    flags.insert(
        "ConsiderFatigue",
        consume(reader, &mut fields, "ConsiderFatigue", "bool")?,
    );
    if flags.get("ConsiderFatigue").copied().unwrap_or(false) {
        consume(reader, &mut fields, "APIFatigueParameters", "struct")?;
        flags.insert(
            "FatigueAPIResultsAvailable",
            consume(reader, &mut fields, "FatigueAPIResultsAvailable", "bool")?,
        );
        flags.insert(
            "FatigueAPIDefaultsSet",
            consume(reader, &mut fields, "FatigueAPIDefaultsSet", "bool")?,
        );
        if flags.get("FatigueAPIDefaultsSet").copied().unwrap_or(false) {
            consume(reader, &mut fields, "FatigueAPIData", "struct[]")?;
        }
        if flags.get("FatigueAPIDefaultsSet").copied().unwrap_or(false)
            && flags
                .get("FatigueAPIResultsAvailable")
                .copied()
                .unwrap_or(false)
        {
            consume(reader, &mut fields, "FatigueAPIEnvelopes", "struct[]")?;
        }
    }
    consume(reader, &mut fields, "ChineseSpecialItems", "struct")?;
    consume(reader, &mut fields, "BridgeDesignPreferences", "struct")?;
    consume(reader, &mut fields, "DesignRequests", "struct[]")?;
    consume(reader, &mut fields, "DesignRequestSupers", "struct[]")?;
    consume(reader, &mut fields, "DesignRequestSeismics", "struct[]")?;
    consume(reader, &mut fields, "Groups", "struct[]")?;
    consume(reader, &mut fields, "SectionCuts", "struct[]")?;
    consume(reader, &mut fields, "WantedArrangement", "i4")?;
    consume(reader, &mut fields, "ws", "struct[]")?;
    consume(reader, &mut fields, "InDBFormat", "struct")?;
    consume(reader, &mut fields, "InDBZeroFormat", "struct")?;
    consume(reader, &mut fields, "InDBNamedFormat", "struct[]")?;
    flags.insert(
        "DBAlwaysUseCurrentUnits",
        consume(reader, &mut fields, "DBAlwaysUseCurrentUnits", "bool")?,
    );
    flags.insert(
        "DBOverrideAutoEFormat",
        consume(reader, &mut fields, "DBOverrideAutoEFormat", "bool")?,
    );
    flags.insert(
        "DBRegenerateHingeAfterImport",
        consume(reader, &mut fields, "DBRegenerateHingeAfterImport", "bool")?,
    );
    consume(reader, &mut fields, "RSNamedSet", "struct[]")?;
    consume(reader, &mut fields, "NLStaticNamedSet", "struct[]")?;
    consume(reader, &mut fields, "PONamedSet", "struct[]")?;
    consume(reader, &mut fields, "THTraceNamedSet", "struct[]")?;
    consume(reader, &mut fields, "ElemEnergyNamedSet", "struct[]")?;
    consume(reader, &mut fields, "DBTableNamedSet", "struct[]")?;
    consume(reader, &mut fields, "NamedReport", "struct[]")?;
    consume(reader, &mut fields, "NamedDisplay", "struct[]")?;
    consume(reader, &mut fields, "NamedPicture", "struct[]")?;
    consume(reader, &mut fields, "QuickReport", "struct")?;
    flags.insert(
        "SaveDBAfterAnalysis",
        consume(reader, &mut fields, "SaveDBAfterAnalysis", "bool")?,
    );
    consume(reader, &mut fields, "AutoDB", "struct")?;
    consume(reader, &mut fields, "ProjectEnvironment", "struct")?;
    consume(reader, &mut fields, "Dimensions", "struct")?;
    consume(reader, &mut fields, "ScreenColors", "struct")?;
    consume(reader, &mut fields, "PrinterColors", "struct")?;
    consume(reader, &mut fields, "ScreenContourColors", "struct")?;
    consume(reader, &mut fields, "PrinterContourColors", "struct")?;
    consume(reader, &mut fields, "ScreenSteelColors", "struct")?;
    consume(reader, &mut fields, "PrinterSteelColors", "struct")?;
    consume(reader, &mut fields, "ColorPrinterColors", "struct")?;
    consume(reader, &mut fields, "ColorPrinterContourColors", "struct")?;
    consume(reader, &mut fields, "ColorPrinterSteelColors", "struct")?;
    consume(reader, &mut fields, "PrinterInfo", "struct")?;
    consume(reader, &mut fields, "DecimalPoint", "struct")?;
    consume(reader, &mut fields, "MiscOptions", "struct")?;
    consume(reader, &mut fields, "TimeFunction", "struct[]")?;
    consume(reader, &mut fields, "CalculatorTendon", "struct")?;
    consume(reader, &mut fields, "SASSIControl", "struct")?;
    consume(reader, &mut fields, "SASSISoilLayer", "struct[]")?;
    consume(reader, &mut fields, "FWPFileiDts", "i4")?;
    consume(reader, &mut fields, "FWPMember", "struct[]")?;
    flags.insert(
        "ProSteelModel",
        consume(reader, &mut fields, "ProSteelModel", "bool")?,
    );
    if flags.get("ProSteelModel").copied().unwrap_or(false) {
        consume(reader, &mut fields, "PSElemRecs", "struct[]")?;
        consume(reader, &mut fields, "PSMaterials", "struct[]")?;
        consume(reader, &mut fields, "PSShapes", "struct[]")?;
    }
    let offset = reader.pos;
    let raw_apps = reader.read_i32()?;
    let apps = reader.check_count(raw_apps, 1)?;
    for _ in 0..apps {
        reader.read_string()?;
        let raw_entries = reader.read_i32()?;
        let entries = reader.check_count(raw_entries, 1)?;
        for _ in 0..entries {
            reader.read_string()?;
            let raw_values = reader.read_i32()?;
            let values = reader.check_count(raw_values, 1)?;
            for _ in 0..values {
                reader.read_string()?;
            }
        }
    }
    fields.push(NativeTopLevelField {
        name: "APIExtendedEntityData".into(),
        wire_type: "application-map".into(),
        offset,
        length: reader.pos - offset,
    });
    consume(reader, &mut fields, "ModelLog", "string")?;
    consume(reader, &mut fields, "DateTimeStamp", "i4")?;
    Ok(fields)
}

struct BinarySdbWriter {
    data: Vec<u8>,
    schemas: &'static HashMap<String, Vec<SchemaStep>>,
}

impl BinarySdbWriter {
    fn new() -> Self {
        Self {
            data: Vec::with_capacity(64 * 1024),
            schemas: get_schemas(),
        }
    }

    fn write_u8(&mut self, v: u8) {
        self.data.push(v);
    }

    fn write_bool(&mut self, v: bool) {
        self.write_u8(u8::from(v));
    }

    fn write_i16(&mut self, v: i16) {
        self.data.extend_from_slice(&v.to_le_bytes());
    }

    fn write_i32(&mut self, v: i32) {
        self.data.extend_from_slice(&v.to_le_bytes());
    }

    fn write_i64(&mut self, v: i64) {
        self.data.extend_from_slice(&v.to_le_bytes());
    }

    fn write_f32(&mut self, v: f32) {
        self.data.extend_from_slice(&v.to_le_bytes());
    }

    fn write_f64(&mut self, v: f64) {
        self.data.extend_from_slice(&v.to_le_bytes());
    }

    fn write_7bit_int(&mut self, mut v: usize) {
        loop {
            let mut b = (v & 0x7f) as u8;
            v >>= 7;
            if v != 0 {
                b |= 0x80;
            }
            self.write_u8(b);
            if v == 0 {
                break;
            }
        }
    }

    fn write_string(&mut self, s: &str) {
        let bytes = s.as_bytes();
        self.write_7bit_int(bytes.len());
        self.data.extend_from_slice(bytes);
    }

    fn write_empty_struct(&mut self) {
        self.write_bool(false);
    }

    fn write_empty_struct_array(&mut self) {
        self.write_bool(false);
    }

    fn write_prim(
        &mut self,
        ptype: &str,
        _prop_name: Option<&str>,
        val: Option<&SdbVal>,
    ) -> Result<(), FormatError> {
        match ptype {
            "bool" => self.write_bool(val.and_then(SdbVal::as_bool).unwrap_or(false)),
            "u1" => self.write_u8(val.and_then(SdbVal::as_i32).unwrap_or(0) as u8),
            "i2" => self.write_i16(val.and_then(SdbVal::as_i32).unwrap_or(0) as i16),
            "i4" => {
                self.write_i32(val.and_then(SdbVal::as_i32).unwrap_or(0));
            }
            "i8" => {
                let v = match val {
                    Some(SdbVal::I64(x)) => *x,
                    Some(SdbVal::I32(x)) => *x as i64,
                    _ => 0,
                };
                self.write_i64(v);
            }
            "r4" => self.write_f32(val.and_then(SdbVal::as_f64).unwrap_or(0.0) as f32),
            "r8" => self.write_f64(val.and_then(SdbVal::as_f64).unwrap_or(0.0)),
            "string" => match val {
                Some(SdbVal::Str(s)) => self.write_string(s),
                Some(SdbVal::Null) => self.write_string("~NothingString~"),
                _ => self.write_string(""),
            },
            _ if ptype.starts_with("System.Nullable`1<") => {
                let inner = &ptype["System.Nullable`1<".len()..ptype.len() - 1];
                match val {
                    None | Some(SdbVal::Null) => self.write_i32(-1),
                    Some(v) => {
                        self.write_i32(0);
                        self.write_prim(inner, None, Some(v))?;
                    }
                }
            }
            _ if ptype.endswith_arr_or_list() => {
                let inner = if let Some(s) = ptype.strip_suffix("[]") {
                    s
                } else {
                    &ptype["System.Collections.Generic.List`1<".len()..ptype.len() - 1]
                };
                match (inner, val) {
                    ("r8", Some(SdbVal::ArrF64(arr))) if !arr.is_empty() => {
                        self.write_bool(true);
                        self.write_i32(arr.len() as i32);
                        for &x in arr {
                            self.write_f64(x);
                        }
                    }
                    ("r4", Some(SdbVal::ArrF64(arr))) if !arr.is_empty() => {
                        self.write_bool(true);
                        self.write_i32(arr.len() as i32);
                        for &x in arr {
                            self.write_f32(x as f32);
                        }
                    }
                    ("i4", Some(SdbVal::ArrI32(arr))) if !arr.is_empty() => {
                        self.write_bool(true);
                        self.write_i32(arr.len() as i32);
                        for &x in arr {
                            self.write_i32(x);
                        }
                    }
                    ("i2", Some(SdbVal::ArrI32(arr))) if !arr.is_empty() => {
                        self.write_bool(true);
                        self.write_i32(arr.len() as i32);
                        for &x in arr {
                            self.write_i16(x as i16);
                        }
                    }
                    ("u1", Some(SdbVal::ArrI32(arr))) if !arr.is_empty() => {
                        self.write_bool(true);
                        self.write_i32(arr.len() as i32);
                        for &x in arr {
                            self.write_u8(x as u8);
                        }
                    }
                    ("bool", Some(SdbVal::ArrBool(arr))) if !arr.is_empty() => {
                        self.write_bool(true);
                        self.write_i32(arr.len() as i32);
                        for &x in arr {
                            self.write_bool(x);
                        }
                    }
                    _ => self.write_bool(false),
                }
            }
            _ => {
                return Err(FormatError::InvalidSdb(format!(
                    "Unknown cBinaryStream primitive type during write: {}",
                    ptype
                )))
            }
        }
        Ok(())
    }

    fn write_struct(
        &mut self,
        class_id: &str,
        overrides: &HashMap<String, SdbVal>,
    ) -> Result<(), FormatError> {
        let steps = self.schemas.get(class_id).ok_or_else(|| {
            FormatError::InvalidSdb(format!(
                "Unknown SAP2000 ClassID '{}' during native write",
                class_id
            ))
        })?;
        self.write_bool(true);
        self.write_string(class_id);
        for st in steps {
            if st.prop.as_deref() == Some("ClassVersion") {
                let version =
                    class_versions::observed_class_version(class_id).ok_or_else(|| {
                        export_guard::unsupported(
                            format!("class_versions.{class_id}"),
                            "no unambiguous observed v19 class tag; export will not guess",
                        )
                    })?;
                self.write_i32(version);
                continue;
            }
            let val = st.prop.as_deref().and_then(|p| overrides.get(p));
            match st.kind.as_str() {
                "struct" => {
                    if let Some(SdbVal::Obj(nested)) = val {
                        if let Some(cid) = nested.get("__class_id").and_then(SdbVal::as_str) {
                            self.write_struct(cid, nested)?;
                        } else {
                            self.write_bool(false);
                        }
                    } else {
                        self.write_bool(false);
                    }
                }
                "struct[]" => {
                    if let Some(SdbVal::ArrObj(items)) = val {
                        if items.is_empty() {
                            self.write_bool(false);
                        } else {
                            self.write_bool(true);
                            self.write_i32(items.len() as i32);
                            for item in items {
                                if let Some(cid) = item.get("__class_id").and_then(SdbVal::as_str) {
                                    self.write_struct(cid, item)?;
                                } else {
                                    self.write_bool(false);
                                }
                            }
                        }
                    } else {
                        self.write_bool(false);
                    }
                }
                "struct[,]" => self.write_bool(false),
                "WriteDouble2" => {
                    if let Some(SdbVal::ArrF64(arr)) = val {
                        if !arr.is_empty() {
                            // Shapes observed in the corpus: per-end values
                            // are stored [component][end], a nine-value
                            // array is a 3 x 3 matrix, and every other
                            // two-dimensional array is a single column.
                            let (rows, columns) = match (class_id, arr.len()) {
                                ("TYFFixity", 12) => (6, 2),
                                ("TYFOffset", 6) => (3, 2),
                                (_, 9) => (3, 3),
                                (_, len) => (len as i32, 1),
                            };
                            self.write_bool(true);
                            self.write_i32(rows);
                            self.write_i32(columns);
                            for &x in arr {
                                self.write_f64(x);
                            }
                        } else {
                            self.write_bool(false);
                        }
                    } else {
                        self.write_bool(false);
                    }
                }
                "WriteShort2" | "WriteInt2" | "WriteSingle2" | "WriteString2" | "WriteByte2"
                | "WriteDouble3" | "WriteDouble4" => self.write_bool(false),
                prim => self.write_prim(prim, st.prop.as_deref(), val)?,
            }
        }
        Ok(())
    }

    fn write_struct_array(
        &mut self,
        class_id: &str,
        items: &[HashMap<String, SdbVal>],
    ) -> Result<(), FormatError> {
        if items.is_empty() {
            self.write_bool(false);
        } else {
            self.write_bool(true);
            self.write_i32(items.len() as i32);
            for item in items {
                let cid = item
                    .get("__class_id")
                    .and_then(SdbVal::as_str)
                    .unwrap_or(class_id);
                self.write_struct(cid, item)?;
            }
        }
        Ok(())
    }
}

fn csys_index(coord_index: &HashMap<String, i32>, name: &str) -> i32 {
    if name.eq_ignore_ascii_case("LOCAL") {
        0
    } else {
        coord_index.get(name).copied().unwrap_or(1)
    }
}

/// Experimental synthesis of a schema-checked v19-style native stream. Vendor
/// application acceptance is unverified. Unsupported or lossy state is rejected;
/// analysis/design results and lock state are intentionally not serialized.
pub fn pack_model_to_native_sdb(model: &SapModel) -> Result<SdbFile, FormatError> {
    export_guard::validate(model)?;
    let mut w = BinarySdbWriter::new();

    // 1. Header strings
    w.write_string(&model.model_name);
    w.write_string(&model.program_version);
    w.write_string("19.00");

    // 2. ProjectStartUp..PresentCoordSystem
    let startup = HashMap::from([
        ("m_Units".into(), SdbVal::I32(model.present_units as i32)),
        ("m_DBUnits".into(), SdbVal::I32(model.database_units as i32)),
    ]);
    w.write_struct("S_ProjectStartUp", &startup)?;
    let mut user_infos = Vec::new();
    for (k, v) in &model.project_info {
        match k.as_str() {
            "Company Name" | "Client Name" | "Project Name" | "Project Number" | "Model Name"
            | "Model Description" | "Revision Number" | "Frame Type" | "Engineer" | "Checker"
            | "Supervisor" | "Issue Code" | "Design Code" => {}
            _ => {
                user_infos.push(HashMap::from([
                    (
                        "__class_id".into(),
                        SdbVal::Str("S_ProjectInfoByUser".into()),
                    ),
                    ("m_Item".into(), SdbVal::Str(k.clone())),
                    ("m_data".into(), SdbVal::Str(v.clone())),
                ]));
            }
        }
    }
    let proj_info = HashMap::from([
        (
            "m_CompanyName".into(),
            SdbVal::Str(
                model
                    .project_info
                    .get("Company Name")
                    .cloned()
                    .unwrap_or_default(),
            ),
        ),
        (
            "m_ClientName".into(),
            SdbVal::Str(
                model
                    .project_info
                    .get("Client Name")
                    .cloned()
                    .unwrap_or_default(),
            ),
        ),
        (
            "m_ProjectName".into(),
            SdbVal::Str(
                model
                    .project_info
                    .get("Project Name")
                    .cloned()
                    .unwrap_or_default(),
            ),
        ),
        (
            "m_ProjectNumber".into(),
            SdbVal::Str(
                model
                    .project_info
                    .get("Project Number")
                    .cloned()
                    .unwrap_or_default(),
            ),
        ),
        (
            "m_ModelName".into(),
            SdbVal::Str(
                model
                    .project_info
                    .get("Model Name")
                    .cloned()
                    .unwrap_or_default(),
            ),
        ),
        (
            "m_ModelDescription".into(),
            SdbVal::Str(
                model
                    .project_info
                    .get("Model Description")
                    .cloned()
                    .unwrap_or_default(),
            ),
        ),
        (
            "m_RevisionNumber".into(),
            SdbVal::Str(
                model
                    .project_info
                    .get("Revision Number")
                    .cloned()
                    .unwrap_or_default(),
            ),
        ),
        (
            "m_FrameType".into(),
            SdbVal::Str(
                model
                    .project_info
                    .get("Frame Type")
                    .cloned()
                    .unwrap_or_default(),
            ),
        ),
        (
            "m_EngineerName".into(),
            SdbVal::Str(
                model
                    .project_info
                    .get("Engineer")
                    .cloned()
                    .unwrap_or_default(),
            ),
        ),
        (
            "m_CheckerName".into(),
            SdbVal::Str(
                model
                    .project_info
                    .get("Checker")
                    .cloned()
                    .unwrap_or_default(),
            ),
        ),
        (
            "m_SupervisorName".into(),
            SdbVal::Str(
                model
                    .project_info
                    .get("Supervisor")
                    .cloned()
                    .unwrap_or_default(),
            ),
        ),
        (
            "m_IssueCode".into(),
            SdbVal::Str(
                model
                    .project_info
                    .get("Issue Code")
                    .cloned()
                    .unwrap_or_default(),
            ),
        ),
        (
            "m_designcode".into(),
            SdbVal::Str(
                model
                    .project_info
                    .get("Design Code")
                    .cloned()
                    .unwrap_or_default(),
            ),
        ),
        (
            "m_NumberUserInfos".into(),
            SdbVal::I32(user_infos.len() as i32),
        ),
        ("m_UserInfo".into(), SdbVal::ArrObj(user_infos)),
    ]);
    w.write_struct("S_ProjectInfo", &proj_info)?;
    w.write_empty_struct_array(); // Standards
    w.write_empty_struct(); // AnalysisModelDateTimeStamp
    let run_params = HashMap::from([(
        "m_dof".into(),
        SdbVal::ArrI32(model.active_dofs.iter().map(|&b| i32::from(b)).collect()),
    )]);
    w.write_struct("RUNARRAY", &run_params)?;
    w.write_i32(1); // PresentCoordSystem

    // 3. CoordSystems + 4 empty arrays
    let mut coord_entries: Vec<&sap_model::CoordSystem> = Vec::new();
    if let Some(global_cs) = model.coord_systems.get("GLOBAL") {
        coord_entries.push(global_cs);
    }
    for (name, cs) in &model.coord_systems {
        if name != "GLOBAL" {
            coord_entries.push(cs);
        }
    }
    let mut coord_index = HashMap::new();
    let mut raw_coords = Vec::with_capacity(coord_entries.len());
    for (idx, cs) in coord_entries.iter().enumerate() {
        coord_index.insert(cs.name.clone(), (idx + 1) as i32);
        let mut guide_x = Vec::new();
        let mut guide_y = Vec::new();
        let mut guide_z = Vec::new();
        for g in &cs.grids {
            let (target, dir) = match g.axis.to_ascii_uppercase().as_str() {
                "X" => (&mut guide_x, 0),
                "Y" => (&mut guide_y, 1),
                "Z" => (&mut guide_z, 2),
                _ => continue,
            };
            let bubble_switch = g.bubble_loc.eq_ignore_ascii_case("Start");
            let guide_map = HashMap::from([
                ("__class_id".into(), SdbVal::Str("S_GUIDE".into())),
                ("m_id".into(), SdbVal::Str(g.id.clone())),
                ("m_GuideNo".into(), SdbVal::I32(target.len() as i32)),
                ("m_GuideDir".into(), SdbVal::I32(dir)),
                ("m_Ordinate".into(), SdbVal::F64(g.coordinate)),
                ("m_xa".into(), SdbVal::F64(0.0)),
                ("m_ya".into(), SdbVal::F64(0.0)),
                ("m_xb".into(), SdbVal::F64(0.0)),
                ("m_Yb".into(), SdbVal::F64(0.0)),
                ("m_xc".into(), SdbVal::F64(0.0)),
                ("m_yc".into(), SdbVal::F64(0.0)),
                ("m_ltype".into(), SdbVal::I32(0)),
                ("m_type".into(), SdbVal::I32(1)),
                ("m_color".into(), SdbVal::I32(8421504)),
                ("m_ColorByUser".into(), SdbVal::Bool(false)),
                ("m_BubbleSize".into(), SdbVal::F64(0.0)),
                ("m_BubbleSwitch".into(), SdbVal::Bool(bubble_switch)),
                ("m_ZTop".into(), SdbVal::I32(0)),
                ("m_ZBot".into(), SdbVal::I32(0)),
                ("m_Visible".into(), SdbVal::Bool(g.visible)),
                ("m_Selected".into(), SdbVal::Bool(false)),
                ("m_alive".into(), SdbVal::Bool(true)),
            ]);
            target.push(guide_map);
        }
        raw_coords.push(HashMap::from([
            ("m_alive".into(), SdbVal::Bool(true)),
            ("m_csname".into(), SdbVal::Str(cs.name.clone())),
            ("m_CoordType".into(), SdbVal::I32(1)),
            ("m_TGLX".into(), SdbVal::I32(guide_x.len() as i32)),
            ("m_TGLY".into(), SdbVal::I32(guide_y.len() as i32)),
            ("m_TGLZ".into(), SdbVal::I32(guide_z.len() as i32)),
            ("m_GuideX".into(), SdbVal::ArrObj(guide_x)),
            ("m_GuideY".into(), SdbVal::ArrObj(guide_y)),
            ("m_GuideZ".into(), SdbVal::ArrObj(guide_z)),
            ("m_Ux".into(), SdbVal::F64(cs.origin[0])),
            ("m_Uy".into(), SdbVal::F64(cs.origin[1])),
            ("m_UZ".into(), SdbVal::F64(cs.origin[2])),
            ("m_RZ".into(), SdbVal::F64(cs.angles_deg[0])),
            ("m_Ry".into(), SdbVal::F64(cs.angles_deg[1])),
            ("m_RX".into(), SdbVal::F64(cs.angles_deg[2])),
        ]));
    }
    w.write_struct_array("COORDSYSTEMARRAY", &raw_coords)?;
    for _ in 0..4 {
        w.write_empty_struct_array();
    }

    // 4. Materials & Sections
    let mut mat_index = HashMap::new();
    let mut raw_materials = Vec::with_capacity(model.materials.len());
    for (idx, mat) in model.materials.values().enumerate() {
        mat_index.insert(mat.name.clone(), (idx + 1) as i32);
        let mtype = match mat.mat_type {
            MatType::Steel => 1,
            MatType::Concrete => 2,
            MatType::NoDesign => 3,
            MatType::Aluminum => 4,
            MatType::ColdFormed => 5,
            MatType::Rebar => 6,
            MatType::Tendon => 7,
        };
        let steel_props = HashMap::from([
            (
                "__class_id".into(),
                SdbVal::Str("S_MaterialPropertiesSteel".into()),
            ),
            ("m_fy".into(), SdbVal::F64(mat.fy)),
            ("m_Fu".into(), SdbVal::F64(mat.fu)),
        ]);
        let conc_props = HashMap::from([
            (
                "__class_id".into(),
                SdbVal::Str("S_MaterialPropertiesConcrete".into()),
            ),
            ("m_fc".into(), SdbVal::F64(mat.fc)),
        ]);
        let rebar_props = HashMap::from([
            (
                "__class_id".into(),
                SdbVal::Str("S_MaterialPropertiesRebar".into()),
            ),
            ("m_fy".into(), SdbVal::F64(mat.fys)),
            ("m_Fu".into(), SdbVal::F64(mat.fu)),
        ]);
        let is_ortho = mat.e_ortho.is_some()
            || mat.u_ortho.is_some()
            || mat.g_ortho.is_some()
            || mat.a_ortho.is_some();
        let sym_type = if is_ortho {
            1
        } else if matches!(mat.mat_type, MatType::Rebar | MatType::Tendon) {
            3
        } else {
            0
        };
        let e_arr = mat.e_ortho.map_or_else(|| vec![mat.e; 3], |v| v.to_vec());
        let g_arr = mat.g_ortho.map_or_else(|| vec![mat.g; 3], |v| v.to_vec());
        let mut u_arr = vec![0.0; 15];
        if let Some(uo) = mat.u_ortho {
            u_arr[..3].copy_from_slice(&uo);
        } else {
            u_arr[..3].fill(mat.u);
        }
        let mut a_arr = vec![0.0; 6];
        if let Some(ao) = mat.a_ortho {
            a_arr[..3].copy_from_slice(&ao);
        } else {
            a_arr[..3].fill(mat.a);
        }
        let mt0 = HashMap::from([
            ("__class_id".into(), SdbVal::Str("S_MT".into())),
            ("m_alive".into(), SdbVal::Bool(true)),
            ("m_w".into(), SdbVal::F64(mat.unit_weight)),
            ("m_m".into(), SdbVal::F64(mat.unit_mass)),
            ("m_e".into(), SdbVal::ArrF64(e_arr)),
            ("m_g".into(), SdbVal::ArrF64(g_arr)),
            ("m_u".into(), SdbVal::ArrF64(u_arr)),
            ("m_a".into(), SdbVal::ArrF64(a_arr)),
            ("m_SteelProps".into(), SdbVal::Obj(steel_props)),
            ("m_ConcreteProps".into(), SdbVal::Obj(conc_props)),
            ("m_RebarProps".into(), SdbVal::Obj(rebar_props)),
        ]);
        raw_materials.push(HashMap::from([
            ("m_alive".into(), SdbVal::Bool(true)),
            ("m_Name".into(), SdbVal::Str(mat.name.clone())),
            (
                "m_color".into(),
                SdbVal::I32(material_colorref(&mat.color).ok_or_else(|| {
                    export_guard::unsupported(
                        format!("materials.{}.color", mat.name),
                        "color has no supported exact COLORREF encoding",
                    )
                })?),
            ),
            ("m_mtype".into(), SdbVal::I32(mtype)),
            ("m_SymType".into(), SdbVal::I32(sym_type)),
            ("m_NumberMTs".into(), SdbVal::I32(1)),
            ("m_Mt".into(), SdbVal::ArrObj(vec![mt0])),
        ]));
    }
    w.write_struct_array("S_Material", &raw_materials)?;

    let mut fsec_index = HashMap::new();
    let mut raw_sections = Vec::with_capacity(model.frame_sections.len());
    for (idx, sec) in model.frame_sections.values().enumerate() {
        fsec_index.insert(sec.name.clone(), (idx + 1) as i32);
        let sh = match sec.prop_type {
            FramePropType::I => 1,
            FramePropType::Channel => 2,
            FramePropType::T => 3,
            FramePropType::Angle => 4,
            FramePropType::DblAngle => 5,
            FramePropType::Box => 6,
            FramePropType::Pipe => 7,
            FramePropType::Rectangular => 8,
            FramePropType::Circle => 9,
            FramePropType::General => 10,
            FramePropType::DbChannel => 11,
            FramePropType::Auto => 12,
            FramePropType::SD => 13,
            FramePropType::Variable => 14,
            _ => 10,
        };
        let mindex = mat_index.get(&sec.material).copied().unwrap_or(1);
        let mut s_map = HashMap::from([
            ("m_alive".into(), SdbVal::Bool(true)),
            ("m_sname".into(), SdbVal::Str(sec.name.clone())),
            ("m_Mindex".into(), SdbVal::I32(mindex)),
            ("m_sh".into(), SdbVal::I32(sh)),
            ("m_t3".into(), SdbVal::F64(sec.t3)),
            ("m_t2".into(), SdbVal::F64(sec.t2)),
            ("m_TF".into(), SdbVal::F64(sec.tf)),
            ("m_TW".into(), SdbVal::F64(sec.tw)),
            ("m_t2b".into(), SdbVal::F64(sec.t2b)),
            ("m_tfb".into(), SdbVal::F64(sec.tfb)),
            ("m_dis".into(), SdbVal::F64(sec.dis)),
            ("m_Area".into(), SdbVal::F64(sec.area)),
            ("m_Torsion".into(), SdbVal::F64(sec.torsional_j)),
            ("m_I33".into(), SdbVal::F64(sec.i33)),
            ("m_I22".into(), SdbVal::F64(sec.i22)),
            ("m_I23".into(), SdbVal::F64(sec.i23)),
            ("m_As2".into(), SdbVal::F64(sec.as2)),
            ("m_As3".into(), SdbVal::F64(sec.as3)),
            ("m_S33".into(), SdbVal::F64(sec.s33)),
            ("m_S22".into(), SdbVal::F64(sec.s22)),
            ("m_Z33".into(), SdbVal::F64(sec.z33)),
            ("m_Z22".into(), SdbVal::F64(sec.z22)),
            ("m_R33".into(), SdbVal::F64(sec.r33)),
            ("m_R22".into(), SdbVal::F64(sec.r22)),
            ("m_msf".into(), SdbVal::ArrF64(sec.modifiers.to_vec())),
        ]);
        if let Some(ref fnm) = sec.file_name {
            s_map.insert("m_PropertyFile".into(), SdbVal::Str(fnm.clone()));
        }
        if let Some(ref sif) = sec.name_in_file {
            s_map.insert("m_Sectname".into(), SdbVal::Str(sif.clone()));
        }
        raw_sections.push(s_map);
    }
    w.write_struct_array("SECTIONARRAY", &raw_sections)?;

    let mut raw_auto = Vec::new();
    let mut raw_vs = Vec::new();
    for sec in model.frame_sections.values() {
        if let Some(&p_idx) = fsec_index.get(&sec.name) {
            for child in &sec.auto_select_list {
                if let Some(&c_idx) = fsec_index.get(child) {
                    raw_auto.push(HashMap::from([
                        ("m_parentSection".into(), SdbVal::I32(p_idx)),
                        ("m_childSection".into(), SdbVal::I32(c_idx)),
                    ]));
                }
            }
            for seg in &sec.non_prismatic {
                if let (Some(&st_idx), Some(&en_idx)) =
                    (fsec_index.get(&seg.start_sec), fsec_index.get(&seg.end_sec))
                {
                    raw_vs.push(HashMap::from([
                        ("m_snumber".into(), SdbVal::I32(p_idx)),
                        ("m_Start".into(), SdbVal::I32(st_idx)),
                        ("m_end".into(), SdbVal::I32(en_idx)),
                        ("m_Length".into(), SdbVal::F64(seg.length)),
                        (
                            "m_type".into(),
                            SdbVal::I32(if seg.is_variable_length { 1 } else { 2 }),
                        ),
                        ("m_EI33".into(), SdbVal::I32(seg.ei33_variation)),
                        ("m_EI22".into(), SdbVal::I32(seg.ei22_variation)),
                    ]));
                }
            }
        }
    }
    w.write_struct_array("AUTOARRAY", &raw_auto)?;
    w.write_struct_array("VSARRAY", &raw_vs)?;
    for _ in 0..3 {
        w.write_empty_struct_array(); // Rebar, SDSection, JoistData
    }

    let mut asec_index = HashMap::new();
    let mut raw_shell_sections = Vec::with_capacity(model.area_sections.len());
    for (idx, asec) in model.area_sections.values().enumerate() {
        asec_index.insert(asec.name.clone(), (idx + 1) as i32);
        // Mirror of the importer's table; `export_guard` refuses the section
        // types whose native code has not been observed.
        let (element_type, stype) = match (asec.prop_type, asec.shell_type, asec.plane_type) {
            (AreaPropType::Shell, ShellType::ShellThick, _) => (0, 1),
            (AreaPropType::Shell, _, _) => (0, 4),
            (AreaPropType::Plane, _, PlaneType::PlaneStress) => (1, 1),
            (AreaPropType::Plane, _, PlaneType::PlaneStrain) => (1, 2),
            (AreaPropType::ASolid, _, _) => (2, 0),
        };
        let mindex = mat_index.get(&asec.material).copied().unwrap_or(1);
        raw_shell_sections.push(HashMap::from([
            ("m_alive".into(), SdbVal::Bool(true)),
            ("m_name".into(), SdbVal::Str(asec.name.clone())),
            ("m_Mindex".into(), SdbVal::I32(mindex)),
            ("m_SType".into(), SdbVal::I32(stype)),
            ("m_ElementType".into(), SdbVal::I32(element_type)),
            ("m_Ang".into(), SdbVal::F64(asec.mat_angle_deg)),
            ("m_Thickness".into(), SdbVal::F64(asec.thickness_membrane)),
            ("m_bending".into(), SdbVal::F64(asec.thickness_bending)),
            (
                "m_Incompatible".into(),
                // Native polarity: 1 = incompatible modes off.
                SdbVal::I32(i32::from(!asec.incompatible_modes)),
            ),
            (
                "m_IncludeDrillingDOF".into(),
                SdbVal::Bool(asec.drilling_dof),
            ),
            ("m_msf".into(), SdbVal::ArrF64(asec.modifiers.to_vec())),
        ]));
    }
    w.write_struct_array("SHELLSECTIONARRAY", &raw_shell_sections)?;

    let mut solid_prop_index = HashMap::new();
    let mut raw_solid_props = Vec::with_capacity(model.solid_sections.len());
    for (idx, ssec) in model.solid_sections.values().enumerate() {
        solid_prop_index.insert(ssec.name.clone(), (idx + 1) as i32);
        let mindex = mat_index.get(&ssec.material).copied().unwrap_or(1);
        raw_solid_props.push(HashMap::from([
            ("m_alive".into(), SdbVal::Bool(true)),
            ("m_name".into(), SdbVal::Str(ssec.name.clone())),
            ("m_Mindex".into(), SdbVal::I32(mindex)),
            (
                "m_Matang".into(),
                SdbVal::ArrF64(ssec.mat_angle_deg.to_vec()),
            ),
            (
                "m_Incompatible".into(),
                SdbVal::I32(i32::from(!ssec.incompatible_modes)),
            ),
        ]));
    }
    w.write_struct_array("SOLIDPROPERTYARRAY", &raw_solid_props)?;

    let mut link_prop_index = HashMap::new();
    let mut raw_link_props = Vec::with_capacity(model.link_props.len());
    for (idx, lprop) in model.link_props.values().enumerate() {
        link_prop_index.insert(lprop.name.clone(), (idx + 1) as i32);
        let ltype = match lprop.link_type {
            LinkPropType::Linear => 1,
            LinkPropType::Damper => 2,
            LinkPropType::Gap => 3,
            LinkPropType::Hook => 4,
            LinkPropType::PlasticWen => 5,
            LinkPropType::RubberIsolator => 6,
            LinkPropType::FrictionIsolator => 7,
            LinkPropType::MultiLinearElastic => 8,
            LinkPropType::MultiLinearPlastic => 9,
            LinkPropType::TCFrictionIsolator => 10,
            LinkPropType::DamperBilinear => 11,
            LinkPropType::DamperFrictionSpring => 12,
            LinkPropType::TriplePendulumIsolator => 13,
        };
        let mut lp_map = HashMap::from([
            ("m_alive".into(), SdbVal::Bool(true)),
            ("m_name".into(), SdbVal::Str(lprop.name.clone())),
            ("m_type".into(), SdbVal::I32(ltype)),
            ("m_dof".into(), SdbVal::ArrBool(lprop.dof_active.to_vec())),
            ("m_Fixed".into(), SdbVal::ArrBool(lprop.dof_fixed.to_vec())),
            (
                "m_Nonlinear".into(),
                SdbVal::ArrBool(lprop.dof_nonlinear.to_vec()),
            ),
            ("m_ke".into(), SdbVal::ArrF64(lprop.ke.to_vec())),
            ("m_ce".into(), SdbVal::ArrF64(lprop.ce.to_vec())),
            ("m_k".into(), SdbVal::ArrF64(lprop.k_nl.to_vec())),
            ("m_dj2".into(), SdbVal::F64(lprop.dj2)),
            ("m_dj3".into(), SdbVal::F64(lprop.dj3)),
            ("m_m".into(), SdbVal::F64(lprop.mass)),
            ("m_w".into(), SdbVal::F64(lprop.weight)),
        ]);
        if lprop.nonlinear_data_complete
            || lprop.param1.iter().any(|&v| v != 0.0)
            || lprop.param2.iter().any(|&v| v != 0.0)
        {
            let mut m_param = vec![0.0; 30];
            m_param[0..6].copy_from_slice(&lprop.param1);
            m_param[6..12].copy_from_slice(&lprop.param2);
            if lprop.link_type == LinkPropType::PlasticWen {
                m_param[12..18].copy_from_slice(&lprop.param3);
            }
            lp_map.insert("m_param".into(), SdbVal::ArrF64(m_param));
        }
        raw_link_props.push(lp_map);
    }
    w.write_struct_array("NLLINKPROPARRAY", &raw_link_props)?;
    w.write_empty_struct_array(); // NLLinkFDProp

    let mut cable_prop_index = HashMap::new();
    let mut raw_cable_props = Vec::new();
    for fr in model.frames.values() {
        if fr.cable_params.is_some() && !cable_prop_index.contains_key(&fr.section) {
            let idx = (raw_cable_props.len() + 1) as i32;
            cable_prop_index.insert(fr.section.clone(), idx);
            let (mindex, area, msf) = if let Some(sec) = model.frame_sections.get(&fr.section) {
                (
                    mat_index.get(&sec.material).copied().unwrap_or(1),
                    sec.area,
                    sec.modifiers.to_vec(),
                )
            } else {
                (1, 0.0, vec![1.0; 8])
            };
            raw_cable_props.push(HashMap::from([
                ("m_alive".into(), SdbVal::Bool(true)),
                ("m_name".into(), SdbVal::Str(fr.section.clone())),
                ("m_Mindex".into(), SdbVal::I32(mindex)),
                ("m_Area".into(), SdbVal::F64(area)),
                ("m_SpecifyArea".into(), SdbVal::Bool(true)),
                ("m_msf".into(), SdbVal::ArrF64(msf)),
            ]));
        }
    }
    w.write_struct_array("CABLEPROPERTYARRAY", &raw_cable_props)?;
    w.write_empty_struct_array(); // TendonProp
    for _ in 0..4 {
        w.write_empty_struct_array(); // NamedFModifier..NamedShModifier
    }

    // 5. PointObjects, LineObjects, AreaObjects, SolidObjects, NLLinkObjects
    let mut pt_index = HashMap::new();
    let mut raw_points = Vec::with_capacity(model.points.len());
    for (idx, pt) in model.points.values().enumerate() {
        pt_index.insert(pt.name.clone(), (idx + 1) as i32);
        let mut p_map = HashMap::from([
            ("m_alive".into(), SdbVal::Bool(true)),
            ("m_Name".into(), SdbVal::Str(pt.name.clone())),
            ("m_gx".into(), SdbVal::F64(pt.x)),
            ("m_gy".into(), SdbVal::F64(pt.y)),
            ("m_gz".into(), SdbVal::F64(pt.z)),
            ("m_x".into(), SdbVal::F64(pt.x)),
            ("m_y".into(), SdbVal::F64(pt.y)),
            ("m_z".into(), SdbVal::F64(pt.z)),
            ("m_a".into(), SdbVal::F64(pt.local_axes_deg[0])),
            ("m_B".into(), SdbVal::F64(pt.local_axes_deg[1])),
            ("m_c".into(), SdbVal::F64(pt.local_axes_deg[2])),
            ("m_Selected".into(), SdbVal::Bool(pt.selected)),
        ]);
        if let Some(mat) = pt.local_axes_matrix {
            let t_vec = vec![
                mat[0][0], mat[0][1], mat[0][2], mat[1][0], mat[1][1], mat[1][2], mat[2][0],
                mat[2][1], mat[2][2],
            ];
            p_map.insert("m_UserAxes".into(), SdbVal::I32(1));
            p_map.insert("m_T".into(), SdbVal::ArrF64(t_vec.clone()));
            p_map.insert("m_tg".into(), SdbVal::ArrF64(t_vec));
        }
        raw_points.push(p_map);
    }
    w.write_struct_array("S_POINTOBJECTS", &raw_points)?;

    let mut fr_index = HashMap::new();
    let mut raw_lines = Vec::with_capacity(model.frames.len());
    for (idx, fr) in model.frames.values().enumerate() {
        fr_index.insert(fr.name.clone(), (idx + 1) as i32);
        let p1 = pt_index.get(&fr.point_i).copied().unwrap_or(1);
        let p2 = pt_index.get(&fr.point_j).copied().unwrap_or(1);
        let (line_type, san) = if fr.cable_params.is_some() {
            (
                1,
                cable_prop_index
                    .get(&fr.section)
                    .or_else(|| fsec_index.get(&fr.section))
                    .copied()
                    .unwrap_or(1),
            )
        } else {
            (0, fsec_index.get(&fr.section).copied().unwrap_or(1))
        };
        let mut ir = vec![0i32; 6];
        for i in 0..6 {
            ir[i] = i32::from(fr.release_i[i]) | (i32::from(fr.release_j[i]) << 1);
        }
        let mut l_map = HashMap::from([
            ("m_alive".into(), SdbVal::Bool(true)),
            ("m_Name".into(), SdbVal::Str(fr.name.clone())),
            ("m_point".into(), SdbVal::ArrI32(vec![p1, p2])),
            ("m_LineType".into(), SdbVal::I32(line_type)),
            ("m_sanalysis".into(), SdbVal::I32(san)),
            ("m_ang".into(), SdbVal::F64(fr.angle_deg)),
            ("m_ai".into(), SdbVal::F64(fr.end_offset_i)),
            ("m_aj".into(), SdbVal::F64(fr.end_offset_j)),
            ("m_RZ".into(), SdbVal::F64(fr.rigid_zone_factor)),
            ("m_CardinalPoint".into(), SdbVal::I32(fr.cardinal_point)),
            ("m_StiffTransform".into(), SdbVal::Bool(fr.stiff_transform)),
            ("m_IR".into(), SdbVal::ArrI32(ir)),
        ]);
        // Automatic subdivision for analysis, as the vendor stores it: the
        // enabling flag (twice), subdivision at joints, and the segment count.
        let mesh_segments = fr.auto_mesh_segments;
        if mesh_segments > 0 || fr.auto_mesh_at_joints {
            l_map.insert("m_AutoMesh".into(), SdbVal::Bool(true));
            l_map.insert("m_automesh2".into(), SdbVal::Bool(true));
            l_map.insert("m_AutoMeshAtPoints".into(), SdbVal::Bool(fr.auto_mesh_at_joints));
            l_map.insert("m_automeshn".into(), SdbVal::I32(mesh_segments as i32));
        }
        if let Some((lim_c_exists, lim_c, lim_t_exists, lim_t)) = fr.tc_limits {
            l_map.insert(
                "m_LimitCompressionExists".into(),
                SdbVal::Bool(lim_c_exists),
            );
            l_map.insert("m_LimitCompression".into(), SdbVal::F64(lim_c));
            l_map.insert("m_LimitTensionExists".into(), SdbVal::Bool(lim_t_exists));
            l_map.insert("m_LimitTension".into(), SdbVal::F64(lim_t));
        }
        if let Some(ref cp) = fr.cable_params {
            let cable_type = if cp.undeformed_length.is_some() {
                8
            } else if cp.target_tension_i.is_some() {
                3
            } else if cp.horizontal_tension.is_some() {
                5
            } else {
                8
            };
            let cd_map = HashMap::from([
                ("__class_id".into(), SdbVal::Str("S_CableData".into())),
                ("m_ModelUsingFrames".into(), SdbVal::Bool(false)),
                ("m_CableType".into(), SdbVal::I32(cable_type)),
                ("m_NumSegs".into(), SdbVal::I32(cp.num_segments as i32)),
                (
                    "m_UndeformedLength".into(),
                    SdbVal::F64(cp.undeformed_length.unwrap_or(0.0)),
                ),
                (
                    "m_TensionI".into(),
                    SdbVal::F64(cp.target_tension_i.unwrap_or(0.0)),
                ),
                (
                    "m_HorizTensionComp".into(),
                    SdbVal::F64(cp.horizontal_tension.unwrap_or(0.0)),
                ),
            ]);
            l_map.insert("m_CableData".into(), SdbVal::Obj(cd_map));
        }
        raw_lines.push(l_map);
    }
    w.write_struct_array("S_LINEOBJECTS", &raw_lines)?;
    w.write_empty_struct_array(); // FrameExternalResult

    let mut ar_index = HashMap::new();
    let mut raw_areas = Vec::with_capacity(model.areas.len());
    for (idx, ar) in model.areas.values().enumerate() {
        ar_index.insert(ar.name.clone(), (idx + 1) as i32);
        let pts: Vec<i32> = ar
            .points
            .iter()
            .map(|p| pt_index.get(p).copied().unwrap_or(1))
            .collect();
        let prop_idx = asec_index.get(&ar.section).copied().unwrap_or(1);
        raw_areas.push(HashMap::from([
            ("m_alive".into(), SdbVal::Bool(true)),
            ("m_name".into(), SdbVal::Str(ar.name.clone())),
            ("m_numpnts".into(), SdbVal::I32(pts.len() as i32)),
            ("m_point".into(), SdbVal::ArrI32(pts)),
            ("m_prop".into(), SdbVal::I32(prop_idx)),
            ("m_Ang".into(), SdbVal::F64(ar.angle_deg)),
        ]));
    }
    w.write_struct_array("S_AREAOBJECTS", &raw_areas)?;

    let mut solid_index = HashMap::new();
    let mut raw_solids = Vec::with_capacity(model.solids.len());
    for (idx, so) in model.solids.values().enumerate() {
        solid_index.insert(so.name.clone(), (idx + 1) as i32);
        let pts: Vec<i32> = so
            .points
            .iter()
            .map(|p| pt_index.get(p).copied().unwrap_or(1))
            .collect();
        let prop_idx = solid_prop_index.get(&so.section).copied().unwrap_or(1);
        raw_solids.push(HashMap::from([
            ("m_alive".into(), SdbVal::Bool(true)),
            ("m_name".into(), SdbVal::Str(so.name.clone())),
            ("m_point".into(), SdbVal::ArrI32(pts)),
            ("m_prop".into(), SdbVal::I32(prop_idx)),
            ("m_Selected".into(), SdbVal::Bool(so.selected)),
        ]));
    }
    w.write_struct_array("S_SOLIDOBJECTS", &raw_solids)?;

    let mut raw_links = Vec::with_capacity(model.links.len());
    for lk in model.links.values() {
        let p1 = pt_index.get(&lk.point_i).copied().unwrap_or(1);
        let p2 = lk
            .point_j
            .as_ref()
            .and_then(|pj| pt_index.get(pj).copied())
            .unwrap_or(0);
        let npr = link_prop_index.get(&lk.prop).copied().unwrap_or(1);
        raw_links.push(HashMap::from([
            ("m_alive".into(), SdbVal::Bool(true)),
            ("m_name".into(), SdbVal::Str(lk.name.clone())),
            ("m_point".into(), SdbVal::ArrI32(vec![p1, p2])),
            ("m_NPR".into(), SdbVal::I32(npr)),
            ("m_Ang".into(), SdbVal::F64(lk.angle_deg)),
            ("m_Selected".into(), SdbVal::Bool(lk.selected)),
        ]));
    }
    w.write_struct_array("S_NLLINKOBJECTS", &raw_links)?;

    // 6. Restraint, Constraint, CJoint, Pattern, PJoint, Spring, Mass
    let mut raw_restraints = Vec::new();
    let mut raw_cjoint = Vec::new();
    let mut raw_pjoint = Vec::new();
    let mut raw_springs = Vec::new();
    let mut raw_masses = Vec::new();

    let mut constraint_index = HashMap::new();
    let mut raw_constraints = Vec::with_capacity(model.constraints.len());
    for (idx, c) in model.constraints.values().enumerate() {
        constraint_index.insert(c.name.clone(), (idx + 1) as i32);
        let ctype = match c.constraint_type {
            ConstraintType::Body => 1,
            ConstraintType::Diaphragm => 2,
            ConstraintType::Plate => 3,
            ConstraintType::Rod => 4,
            ConstraintType::Beam => 5,
            ConstraintType::Equal => 6,
            ConstraintType::Local => 7,
            ConstraintType::Weld => 8,
            ConstraintType::Line => 9,
        };
        let axis = match c.axis {
            ConstraintAxis::X => 1,
            ConstraintAxis::Y => 2,
            ConstraintAxis::Z => 3,
            ConstraintAxis::AutoAxis => 4,
        };
        raw_constraints.push(HashMap::from([
            ("m_alive".into(), SdbVal::Bool(true)),
            ("m_Name".into(), SdbVal::Str(c.name.clone())),
            ("m_type".into(), SdbVal::I32(ctype)),
            ("m_axis".into(), SdbVal::I32(axis)),
            (
                "m_DOFs".into(),
                SdbVal::ArrI32(c.dofs.iter().map(|&b| i32::from(b)).collect()),
            ),
        ]));
    }

    let mut pattern_set = std::collections::BTreeSet::new();
    for pt in model.points.values() {
        for pname in pt.pattern_values.keys() {
            pattern_set.insert(pname.clone());
        }
    }
    for fr in model.frames.values() {
        for lt in &fr.loads_temp {
            if let Some(ref pname) = lt.pattern_name {
                pattern_set.insert(pname.clone());
            }
        }
    }
    for ar in model.areas.values() {
        for lp in &ar.loads_pressure {
            if let Some(ref pname) = lp.pattern_name {
                pattern_set.insert(pname.clone());
            }
        }
        for lt in &ar.loads_temp {
            if let Some(ref pname) = lt.pattern_name {
                pattern_set.insert(pname.clone());
            }
        }
    }
    let mut pat_index = HashMap::new();
    let mut raw_patterns = Vec::with_capacity(pattern_set.len());
    for (idx, pname) in pattern_set.into_iter().enumerate() {
        pat_index.insert(pname.clone(), (idx + 1) as i32);
        raw_patterns.push(HashMap::from([
            ("m_alive".into(), SdbVal::Bool(true)),
            ("m_Name".into(), SdbVal::Str(pname)),
        ]));
    }

    for pt in model.points.values() {
        let jn = pt_index.get(&pt.name).copied().unwrap_or(1);
        if pt.restraint.iter().any(|&r| r) {
            raw_restraints.push(HashMap::from([
                ("m_alive".into(), SdbVal::Bool(true)),
                ("m_jn".into(), SdbVal::I32(jn)),
                ("m_DOFs".into(), SdbVal::ArrBool(pt.restraint.to_vec())),
            ]));
        }
        for cname in &pt.constraints {
            if let Some(&cn) = constraint_index.get(cname) {
                raw_cjoint.push(HashMap::from([
                    ("m_alive".into(), SdbVal::Bool(true)),
                    ("m_jn".into(), SdbVal::I32(jn)),
                    ("m_ConstraintN".into(), SdbVal::I32(cn)),
                ]));
            }
        }
        for (pname, &pval) in &pt.pattern_values {
            if let Some(&pn) = pat_index.get(pname) {
                raw_pjoint.push(HashMap::from([
                    ("m_alive".into(), SdbVal::Bool(true)),
                    ("m_jn".into(), SdbVal::I32(jn)),
                    ("m_PatternN".into(), SdbVal::I32(pn)),
                    ("m_P".into(), SdbVal::F64(pval)),
                ]));
            }
        }
        if pt.spring.iter().any(|&k| k != 0.0) || pt.spring_coupled.is_some() || pt.spring_is_local
        {
            let xjps = if let Some(coupled) = pt.spring_coupled {
                coupled.to_vec()
            } else {
                let mut arr = vec![0.0; 21];
                for (d, &idx) in SPRING_DIAG_INDICES.iter().enumerate() {
                    arr[idx] = pt.spring[d];
                }
                arr
            };
            let csys = if pt.spring_is_local { -1 } else { 1 };
            raw_springs.push(HashMap::from([
                ("m_alive".into(), SdbVal::Bool(true)),
                ("m_jn".into(), SdbVal::I32(jn)),
                ("m_XJPS".into(), SdbVal::ArrF64(xjps)),
                ("m_csys".into(), SdbVal::I32(csys)),
            ]));
        }
        if pt.mass.iter().any(|&m| m != 0.0) {
            raw_masses.push(HashMap::from([
                ("m_alive".into(), SdbVal::Bool(true)),
                ("m_jn".into(), SdbVal::I32(jn)),
                ("m_XJPM".into(), SdbVal::ArrF64(pt.mass.to_vec())),
            ]));
        }
    }

    w.write_struct_array("TYRestraint", &raw_restraints)?;
    w.write_struct_array("CONSTRAINTARRAY", &raw_constraints)?;
    w.write_struct_array("TYCJoint", &raw_cjoint)?;
    w.write_struct_array("PATTERNARRAY", &raw_patterns)?;
    w.write_struct_array("TYPJoint", &raw_pjoint)?;
    w.write_struct_array("TYSpring", &raw_springs)?;
    w.write_struct_array("TYMass", &raw_masses)?;
    for _ in 0..3 {
        w.write_empty_struct_array(); // PanelZone, GenDispl, GD
    }

    // 7. Frame/Area modifiers & fixity
    w.write_empty_struct_array(); // MatTemp
    let mut raw_pd_force = Vec::new();
    let mut raw_fmodifier = Vec::new();
    let mut raw_ffixity = Vec::new();
    let mut raw_foffset = Vec::new();
    for fr in model.frames.values() {
        let fn_idx = fr_index.get(&fr.name).copied().unwrap_or(1);
        if fr.pdelta_force != 0.0 {
            raw_pd_force.push(HashMap::from([
                ("m_alive".into(), SdbVal::Bool(true)),
                ("m_n".into(), SdbVal::I32(fn_idx)),
                ("m_P".into(), SdbVal::F64(fr.pdelta_force)),
            ]));
        }
        if fr.modifiers.iter().any(|&m| (m - 1.0).abs() > 1e-12) {
            raw_fmodifier.push(HashMap::from([
                ("m_alive".into(), SdbVal::Bool(true)),
                ("m_n".into(), SdbVal::I32(fn_idx)),
                ("m_sf".into(), SdbVal::ArrF64(fr.modifiers.to_vec())),
            ]));
        }
        if fr.start_spring.iter().any(|&k| k != 0.0) || fr.end_spring.iter().any(|&k| k != 0.0) {
            let mut p = Vec::with_capacity(12);
            for i in 0..6 {
                p.push(fr.start_spring[i]);
                p.push(fr.end_spring[i]);
            }
            raw_ffixity.push(HashMap::from([
                ("m_alive".into(), SdbVal::Bool(true)),
                ("m_n".into(), SdbVal::I32(fn_idx)),
                ("m_P".into(), SdbVal::ArrF64(p)),
            ]));
        }
        // Insertion-point joint offsets in the element local system, the one
        // form the corpus shows: [component][end], coordinate system -1.
        if fr.joint_offset_i != [0.0; 3] || fr.joint_offset_j != [0.0; 3] {
            let mut p = Vec::with_capacity(6);
            for c in 0..3 {
                p.push(fr.joint_offset_i[c]);
                p.push(fr.joint_offset_j[c]);
            }
            raw_foffset.push(HashMap::from([
                ("m_alive".into(), SdbVal::Bool(true)),
                ("m_n".into(), SdbVal::I32(fn_idx)),
                ("m_P".into(), SdbVal::ArrF64(p)),
                ("m_csys".into(), SdbVal::I32(-1)),
            ]));
        }
    }
    w.write_struct_array("TYPDFORCE", &raw_pd_force)?;
    w.write_struct_array("TYFModifier", &raw_fmodifier)?;
    w.write_struct_array("TYFFixity", &raw_ffixity)?;
    w.write_struct_array("TYFOffset", &raw_foffset)?;
    for _ in 0..3 {
        w.write_empty_struct_array(); // FSpring, FMass, FFireProofing
    }
    // Object-level area assignments, in the sections where the corpus keeps
    // them (Example 2-008): property modifiers, added mass, N x N auto-mesh.
    let mut raw_shmodifier = Vec::new();
    let mut raw_shmass = Vec::new();
    let mut raw_shmesh = Vec::new();
    for ar in model.areas.values() {
        let an_idx = ar_index.get(&ar.name).copied().unwrap_or(1);
        if ar.modifiers.iter().any(|&m| m != 1.0) {
            raw_shmodifier.push(HashMap::from([
                ("m_alive".into(), SdbVal::Bool(true)),
                ("m_n".into(), SdbVal::I32(an_idx)),
                ("m_sf".into(), SdbVal::ArrF64(ar.modifiers.to_vec())),
            ]));
        }
        if ar.mass_per_area != 0.0 {
            raw_shmass.push(HashMap::from([
                ("m_alive".into(), SdbVal::Bool(true)),
                ("m_n".into(), SdbVal::I32(an_idx)),
                ("m_mpa".into(), SdbVal::F64(ar.mass_per_area)),
            ]));
        }
        if ar.auto_mesh != (1, 1) {
            raw_shmesh.push(HashMap::from([
                ("m_alive".into(), SdbVal::Bool(true)),
                ("m_n".into(), SdbVal::I32(an_idx)),
                ("m_MeshType".into(), SdbVal::I32(1)),
                ("m_n1".into(), SdbVal::I32(ar.auto_mesh.0 as i32)),
                ("m_n2".into(), SdbVal::I32(ar.auto_mesh.1 as i32)),
                // Group 1 as in every corpus record.
                ("m_Group".into(), SdbVal::I32(1)),
                // What the solver does: restrain a new edge joint in the DOFs
                // restrained at both ends of the edge.
                ("m_RestraintsOnEdge".into(), SdbVal::Bool(true)),
                ("m_RestraintsOnFace".into(), SdbVal::Bool(true)),
            ]));
        }
    }
    w.write_struct_array("TYShModifier", &raw_shmodifier)?;
    w.write_empty_struct_array(); // ShSpring
    w.write_struct_array("TYSMass", &raw_shmass)?;
    w.write_struct_array("TYShMesh", &raw_shmesh)?;
    w.write_empty_struct_array(); // SSpring

    // 8. LoadPattern, AnalysisCase, Combo, Loads, Functions
    let mut func_index = HashMap::new();
    for (idx, func) in model.functions.values().enumerate() {
        func_index.insert(func.name.clone(), (idx + 1) as i32);
    }

    let mut lpat_index = HashMap::new();
    let mut raw_load_patterns = Vec::with_capacity(model.load_patterns.len());
    for (idx, lp) in model.load_patterns.values().enumerate() {
        lpat_index.insert(lp.name.clone(), (idx + 1) as i32);
        let ptype = match lp.pat_type {
            LoadPatternType::Dead => 1,
            LoadPatternType::SuperDead => 2,
            LoadPatternType::Live => 3,
            LoadPatternType::ReduceLive => 4,
            LoadPatternType::Quake => 5,
            LoadPatternType::Wind => 6,
            LoadPatternType::Snow => 7,
            LoadPatternType::Other => 8,
            LoadPatternType::Move => 9,
            LoadPatternType::Temperature => 10,
            LoadPatternType::Rooflive => 11,
            LoadPatternType::Notional => 12,
            LoadPatternType::PatternLive => 13,
            LoadPatternType::Wave => 14,
            LoadPatternType::Prestress => 34,
        };
        raw_load_patterns.push(HashMap::from([
            ("m_alive".into(), SdbVal::Bool(true)),
            ("m_Name".into(), SdbVal::Str(lp.name.clone())),
            ("m_type".into(), SdbVal::I32(ptype)),
            (
                "m_SelfWTMultiplier".into(),
                SdbVal::F64(lp.self_wt_multiplier),
            ),
        ]));
    }
    w.write_struct_array("LOADPATTERNARRAY", &raw_load_patterns)?;
    for _ in 0..3 {
        w.write_empty_struct_array(); // Seismic, Wind, MultiStepMove
    }
    w.write_empty_struct(); // UndeformedGeometryData

    let mut ac_index = HashMap::new();
    for (idx, lc) in model.load_cases.values().enumerate() {
        ac_index.insert(lc.name.clone(), (idx + 1) as i32);
    }
    let mut raw_analysis_cases = Vec::with_capacity(model.load_cases.len());
    for lc in model.load_cases.values() {
        let ctype_code = match lc.case_type {
            LoadCaseType::LinearStatic => 1,
            LoadCaseType::NonlinearStatic => 2,
            LoadCaseType::Modal => 3,
            LoadCaseType::ResponseSpectrum => 4,
            LoadCaseType::LinearHistory => 5,
            LoadCaseType::NonlinearHistory => 6,
            LoadCaseType::LinearDynamic => 7,
            LoadCaseType::NonlinearDynamic => 8,
            LoadCaseType::MovingLoad => 9,
            LoadCaseType::Buckling => 10,
            LoadCaseType::SteadyState => 11,
            LoadCaseType::PowerSpectralDensity => 12,
            LoadCaseType::LinearStaticMultiStep => 13,
            LoadCaseType::HyperStatic => 14,
        };
        let prev_idx = lc
            .initial_case
            .as_ref()
            .and_then(|ic| ac_index.get(ic).copied())
            .unwrap_or(0);
        let mut las = Vec::new();
        if ctype_code == 4 {
            for (func_name, dir_str, sf) in &lc.spectrum_loads {
                let f_idx = func_index.get(func_name).copied().unwrap_or(0);
                let l_idx = dir_str
                    .strip_prefix('U')
                    .or_else(|| dir_str.strip_prefix('u'))
                    .and_then(|s| s.parse::<i32>().ok())
                    .unwrap_or(1);
                las.push(HashMap::from([
                    ("__class_id".into(), SdbVal::Str("LAARRAY".into())),
                    ("m_ltype".into(), SdbVal::I32(2)),
                    ("m_load".into(), SdbVal::I32(l_idx)),
                    ("m_Func".into(), SdbVal::I32(f_idx)),
                    ("m_sf".into(), SdbVal::F64(*sf)),
                    ("m_csys".into(), SdbVal::I32(1)),
                    ("m_TF".into(), SdbVal::F64(1.0)),
                ]));
            }
        } else if (5..=8).contains(&ctype_code) {
            for (func_name, load_name, sf) in &lc.history_loads {
                let f_idx = func_index.get(func_name).copied().unwrap_or(0);
                let (ltype, l_idx) = if let Some(u_idx) = load_name
                    .strip_prefix('U')
                    .or_else(|| load_name.strip_prefix('u'))
                    .and_then(|s| s.parse::<i32>().ok())
                    .filter(|&i| (1..=6).contains(&i))
                    .filter(|_| !lpat_index.contains_key(load_name))
                {
                    (2, u_idx)
                } else {
                    (1, lpat_index.get(load_name).copied().unwrap_or(0))
                };
                las.push(HashMap::from([
                    ("__class_id".into(), SdbVal::Str("LAARRAY".into())),
                    ("m_ltype".into(), SdbVal::I32(ltype)),
                    ("m_load".into(), SdbVal::I32(l_idx)),
                    ("m_Func".into(), SdbVal::I32(f_idx)),
                    ("m_sf".into(), SdbVal::F64(*sf)),
                    ("m_csys".into(), SdbVal::I32(1)),
                    ("m_TF".into(), SdbVal::F64(1.0)),
                ]));
            }
        } else {
            for asgn in &lc.loads {
                if let Some(u_idx) = asgn
                    .load_name
                    .strip_prefix('U')
                    .or_else(|| asgn.load_name.strip_prefix('u'))
                    .and_then(|s| s.parse::<i32>().ok())
                    .filter(|&i| (1..=6).contains(&i))
                    .filter(|_| !lpat_index.contains_key(&asgn.load_name))
                {
                    las.push(HashMap::from([
                        ("__class_id".into(), SdbVal::Str("LAARRAY".into())),
                        ("m_ltype".into(), SdbVal::I32(2)),
                        ("m_load".into(), SdbVal::I32(u_idx)),
                        ("m_sf".into(), SdbVal::F64(asgn.scale_factor)),
                        ("m_csys".into(), SdbVal::I32(1)),
                        ("m_TF".into(), SdbVal::F64(1.0)),
                    ]));
                } else if let Some(&l_idx) = lpat_index.get(&asgn.load_name) {
                    las.push(HashMap::from([
                        ("__class_id".into(), SdbVal::Str("LAARRAY".into())),
                        ("m_ltype".into(), SdbVal::I32(1)),
                        ("m_load".into(), SdbVal::I32(l_idx)),
                        ("m_sf".into(), SdbVal::F64(asgn.scale_factor)),
                        ("m_csys".into(), SdbVal::I32(1)),
                        ("m_TF".into(), SdbVal::F64(1.0)),
                    ]));
                }
            }
        }
        let modc = match lc.modal_combo {
            ModalComboMethod::Cqc => 1,
            ModalComboMethod::Srss => 2,
            ModalComboMethod::AbsSum => 3,
        };
        let mut sol_control = HashMap::from([
            (
                "__class_id".into(),
                SdbVal::Str("SOLUTIONCONTROLARRAY".into()),
            ),
            ("m_nstep".into(), SdbVal::I32(lc.num_time_steps as i32)),
            ("m_DT".into(), SdbVal::F64(lc.time_step_dt)),
            ("m_MaxModes".into(), SdbVal::I32(lc.max_modes as i32)),
            ("m_MinModes".into(), SdbVal::I32(lc.min_modes as i32)),
            (
                "m_ModeType".into(),
                SdbVal::I32(match lc.modal_sub_type {
                    ModalSubType::Eigen => 1,
                    ModalSubType::Ritz => 2,
                }),
            ),
            ("m_EigenShiftFreq".into(), SdbVal::F64(lc.shift_freq_hz)),
            (
                "m_NumBucklingModes".into(),
                SdbVal::I32(lc.num_buckling_modes as i32),
            ),
            ("m_itmax".into(), SdbVal::I32(lc.max_iterations as i32)),
            ("m_TolConvD".into(), SdbVal::F64(lc.convergence_tol)),
        ]);
        let mut monitored_joints = Vec::new();
        if let Some(control) = &lc.displacement_control {
            sol_control.insert("m_control".into(), SdbVal::I32(2));
            sol_control.insert("m_TargetDisp".into(), SdbVal::F64(control.target));
            sol_control.insert(
                "m_DispType".into(),
                SdbVal::I32(if control.conjugate { 1 } else { 2 }),
            );
            if let Some((point, dof)) = &control.monitored {
                monitored_joints.push(HashMap::from([
                    ("__class_id".into(), SdbVal::Str("TYNLCDisp".into())),
                    ("m_DType".into(), SdbVal::I32(1)),
                    (
                        "m_jn".into(),
                        SdbVal::I32(pt_index.get(point).copied().unwrap_or(0)),
                    ),
                    ("m_dof".into(), SdbVal::I32(*dof as i32 + 1)),
                    ("m_sf".into(), SdbVal::F64(1.0)),
                ]));
            }
        }
        let nonlinearity = HashMap::from([
            ("__class_id".into(), SdbVal::Str("NONLINEARITYARRAY".into())),
            (
                "m_nlgeomtype".into(),
                SdbVal::I32(match lc.nl_geom {
                    NLGeomType::None => 0,
                    NLGeomType::PDelta => 1,
                    NLGeomType::LargeDisplacements => 2,
                }),
            ),
        ]);
        let dd = HashMap::from([
            ("__class_id".into(), SdbVal::Str("DDARRAY".into())),
            ("m_Dampa".into(), SdbVal::F64(lc.rayleigh_damping.0)),
            ("m_Dampb".into(), SdbVal::F64(lc.rayleigh_damping.1)),
        ]);
        raw_analysis_cases.push(HashMap::from([
            ("m_alive".into(), SdbVal::Bool(true)),
            ("m_name".into(), SdbVal::Str(lc.name.clone())),
            ("m_casetype".into(), SdbVal::I32(ctype_code)),
            ("m_SelectedForRun".into(), SdbVal::Bool(lc.run_flag)),
            ("m_previous".into(), SdbVal::I32(prev_idx)),
            ("m_NumberLAs".into(), SdbVal::I32(las.len() as i32)),
            ("m_LA".into(), SdbVal::ArrObj(las)),
            ("m_Damp".into(), SdbVal::F64(lc.modal_damping)),
            // 4 (constant ratio) is the value on every default case in the
            // corpus. A modal history whose damping is carried as Rayleigh
            // coefficients is written as 1 (direct coefficients, as in Example
            // 6-011b) so the coefficients are not read as inactive.
            (
                "m_DampType".into(),
                SdbVal::I32(
                    if matches!(
                        lc.case_type,
                        LoadCaseType::LinearHistory | LoadCaseType::NonlinearHistory
                    ) && lc.rayleigh_damping != (0.0, 0.0)
                    {
                        1
                    } else {
                        4
                    },
                ),
            ),
            ("m_modc".into(), SdbVal::I32(modc)),
            (
                "m_ndjoints".into(),
                SdbVal::I32(monitored_joints.len() as i32),
            ),
            ("m_djoints".into(), SdbVal::ArrObj(monitored_joints)),
            ("m_SolControl".into(), SdbVal::Obj(sol_control)),
            ("m_Nonlinearity".into(), SdbVal::Obj(nonlinearity)),
            ("m_dd".into(), SdbVal::Obj(dd)),
        ]));
    }
    w.write_struct_array("ANALYSISCASEARRAY", &raw_analysis_cases)?;
    w.write_empty_struct_array(); // Schedule
    w.write_empty_struct_array(); // MassSource

    let mut combo_index = HashMap::new();
    for (idx, cb) in model.load_combos.values().enumerate() {
        combo_index.insert(cb.name.clone(), (idx + 1) as i32);
    }
    let mut raw_combos = Vec::with_capacity(model.load_combos.len());
    for cb in model.load_combos.values() {
        let ctype = match cb.combo_type {
            ComboType::LinearAdd => 0,
            ComboType::Envelope => 1,
            ComboType::AbsAdd => 2,
            ComboType::Srss => 3,
            ComboType::RangeAdd => 4,
        };
        let mut cls = Vec::with_capacity(cb.cases.len());
        for (ref_name, sf) in &cb.cases {
            let (cl_type, ncase) = if let Some(&c_idx) = ac_index.get(ref_name) {
                (0, c_idx)
            } else if let Some(&cb_idx) = combo_index.get(ref_name) {
                (1, cb_idx)
            } else {
                (0, 0)
            };
            cls.push(HashMap::from([
                ("__class_id".into(), SdbVal::Str("CLARRAY".into())),
                ("m_CType".into(), SdbVal::I32(cl_type)),
                ("m_nCase".into(), SdbVal::I32(ncase)),
                ("m_sf".into(), SdbVal::F64(*sf)),
            ]));
        }
        raw_combos.push(HashMap::from([
            ("m_alive".into(), SdbVal::Bool(true)),
            ("m_Name".into(), SdbVal::Str(cb.name.clone())),
            ("m_type".into(), SdbVal::I32(ctype)),
            ("m_NumberCls".into(), SdbVal::I32(cls.len() as i32)),
            ("m_CL".into(), SdbVal::ArrObj(cls)),
        ]));
    }
    w.write_struct_array("COMBOARRAY", &raw_combos)?;
    w.write_empty_struct_array(); // AutoComboOptions

    let mut raw_jfload = Vec::new();
    let mut raw_jgdload = Vec::new();
    for pt in model.points.values() {
        let jn = pt_index.get(&pt.name).copied().unwrap_or(1);
        for lf in &pt.loads_force {
            if let Some(&lc) = lpat_index.get(&lf.load_pat) {
                raw_jfload.push(HashMap::from([
                    ("m_alive".into(), SdbVal::Bool(true)),
                    ("m_jn".into(), SdbVal::I32(jn)),
                    ("m_LC".into(), SdbVal::I32(lc)),
                    (
                        "m_csys".into(),
                        SdbVal::I32(csys_index(&coord_index, &lf.csys)),
                    ),
                    ("m_F".into(), SdbVal::ArrF64(lf.force.to_vec())),
                ]));
            }
        }
        for ld in &pt.loads_displ {
            if let Some(&lc) = lpat_index.get(&ld.load_pat) {
                raw_jgdload.push(HashMap::from([
                    ("m_alive".into(), SdbVal::Bool(true)),
                    ("m_jn".into(), SdbVal::I32(jn)),
                    ("m_LC".into(), SdbVal::I32(lc)),
                    (
                        "m_csys".into(),
                        SdbVal::I32(csys_index(&coord_index, &ld.csys)),
                    ),
                    ("m_d".into(), SdbVal::ArrF64(ld.displ.to_vec())),
                ]));
            }
        }
    }
    w.write_struct_array("TYJFLoad", &raw_jfload)?;
    w.write_struct_array("TYJGDLoad", &raw_jgdload)?;

    let mut raw_fgload = Vec::new();
    let mut raw_fcload = Vec::new();
    let mut raw_fdload = Vec::new();
    let mut raw_ftload = Vec::new();
    for fr in model.frames.values() {
        let fn_idx = fr_index.get(&fr.name).copied().unwrap_or(1);
        for (lpat, &[ux, uy, uz]) in &fr.loads_gravity {
            if let Some(&lc) = lpat_index.get(lpat) {
                raw_fgload.push(HashMap::from([
                    ("m_alive".into(), SdbVal::Bool(true)),
                    ("m_n".into(), SdbVal::I32(fn_idx)),
                    ("m_lc".into(), SdbVal::I32(lc)),
                    ("m_Ux".into(), SdbVal::F64(ux)),
                    ("m_Uy".into(), SdbVal::F64(uy)),
                    ("m_UZ".into(), SdbVal::F64(uz)),
                ]));
            }
        }
        for lp in &fr.loads_point {
            if let Some(&lc) = lpat_index.get(&lp.load_pat) {
                raw_fcload.push(HashMap::from([
                    ("m_alive".into(), SdbVal::Bool(true)),
                    ("m_n".into(), SdbVal::I32(fn_idx)),
                    ("m_lc".into(), SdbVal::I32(lc)),
                    (
                        "m_csys".into(),
                        SdbVal::I32(csys_index(&coord_index, &lp.csys)),
                    ),
                    ("m_kt".into(), SdbVal::I32(lp.my_type)),
                    ("m_Kd".into(), SdbVal::I32(lp.dir)),
                    ("m_RD".into(), SdbVal::F64(lp.rel_dist)),
                    ("m_UK".into(), SdbVal::F64(lp.val)),
                ]));
            }
        }
        for ld in &fr.loads_dist {
            if let Some(&lc) = lpat_index.get(&ld.load_pat) {
                raw_fdload.push(HashMap::from([
                    ("m_alive".into(), SdbVal::Bool(true)),
                    ("m_n".into(), SdbVal::I32(fn_idx)),
                    ("m_lc".into(), SdbVal::I32(lc)),
                    (
                        "m_csys".into(),
                        SdbVal::I32(csys_index(&coord_index, &ld.csys)),
                    ),
                    ("m_kt".into(), SdbVal::I32(ld.my_type)),
                    ("m_Kd".into(), SdbVal::I32(ld.dir)),
                    ("m_RDA".into(), SdbVal::F64(ld.rel_dist1)),
                    ("m_RDB".into(), SdbVal::F64(ld.rel_dist2)),
                    ("m_UKA".into(), SdbVal::F64(ld.val1)),
                    ("m_UKB".into(), SdbVal::F64(ld.val2)),
                ]));
            }
        }
        for lt in &fr.loads_temp {
            if let Some(&lc) = lpat_index.get(&lt.load_pat) {
                let p_idx = lt
                    .pattern_name
                    .as_ref()
                    .and_then(|pn| pat_index.get(pn).copied())
                    .unwrap_or(0);
                raw_ftload.push(HashMap::from([
                    ("m_alive".into(), SdbVal::Bool(true)),
                    ("m_n".into(), SdbVal::I32(fn_idx)),
                    ("m_lc".into(), SdbVal::I32(lc)),
                    ("m_LT".into(), SdbVal::I32(lt.my_type)),
                    ("m_T".into(), SdbVal::F64(lt.val)),
                    ("m_Pattern".into(), SdbVal::I32(p_idx)),
                ]));
            }
        }
    }
    w.write_struct_array("TYGLoad", &raw_fgload)?;
    w.write_struct_array("TYFCLoad", &raw_fcload)?;
    w.write_struct_array("TYFDLoad", &raw_fdload)?;
    w.write_struct_array("TYTLoad", &raw_ftload)?;
    for _ in 0..6 {
        w.write_empty_struct_array(); // FSLoad..ShGLoad
    }

    let mut raw_shuload = Vec::new();
    let mut raw_shspload = Vec::new();
    let mut raw_shtload = Vec::new();
    for ar in model.areas.values() {
        let an_idx = ar_index.get(&ar.name).copied().unwrap_or(1);
        for lu in &ar.loads_uniform {
            if let Some(&lc) = lpat_index.get(&lu.load_pat) {
                raw_shuload.push(HashMap::from([
                    ("m_alive".into(), SdbVal::Bool(true)),
                    ("m_n".into(), SdbVal::I32(an_idx)),
                    ("m_LC".into(), SdbVal::I32(lc)),
                    (
                        "m_csys".into(),
                        SdbVal::I32(csys_index(&coord_index, &lu.csys)),
                    ),
                    ("m_Kd".into(), SdbVal::I32(lu.dir)),
                    ("m_P".into(), SdbVal::F64(lu.val)),
                ]));
            }
        }
        for lp in &ar.loads_pressure {
            if let Some(&lc) = lpat_index.get(&lp.load_pat) {
                let p_idx = lp
                    .pattern_name
                    .as_ref()
                    .and_then(|pn| pat_index.get(pn).copied())
                    .unwrap_or(0);
                raw_shspload.push(HashMap::from([
                    ("m_alive".into(), SdbVal::Bool(true)),
                    ("m_n".into(), SdbVal::I32(an_idx)),
                    ("m_LC".into(), SdbVal::I32(lc)),
                    ("m_Face".into(), SdbVal::I32(lp.face)),
                    ("m_P".into(), SdbVal::F64(lp.val)),
                    ("m_Pattern".into(), SdbVal::I32(p_idx)),
                ]));
            }
        }
        for lt in &ar.loads_temp {
            if let Some(&lc) = lpat_index.get(&lt.load_pat) {
                let p_idx = lt
                    .pattern_name
                    .as_ref()
                    .and_then(|pn| pat_index.get(pn).copied())
                    .unwrap_or(0);
                raw_shtload.push(HashMap::from([
                    ("m_alive".into(), SdbVal::Bool(true)),
                    ("m_n".into(), SdbVal::I32(an_idx)),
                    ("m_lc".into(), SdbVal::I32(lc)),
                    ("m_LT".into(), SdbVal::I32(lt.my_type)),
                    ("m_T".into(), SdbVal::F64(lt.val)),
                    ("m_Pattern".into(), SdbVal::I32(p_idx)),
                ]));
            }
        }
    }
    w.write_struct_array("TYShULoad", &raw_shuload)?;
    w.write_empty_struct_array(); // ShUFLoad
    w.write_struct_array("TYSPLoad", &raw_shspload)?;
    w.write_empty_struct_array(); // ShPPLoad
    w.write_struct_array("TYTLoad", &raw_shtload)?;
    for _ in 0..5 {
        w.write_empty_struct_array(); // ShSLoad..SoPPLoad
    }

    let mut raw_sospload = Vec::new();
    let mut raw_sotload = Vec::new();
    for so in model.solids.values() {
        let sn_idx = solid_index.get(&so.name).copied().unwrap_or(1);
        for lp in &so.loads_surface {
            if let Some(&lc) = lpat_index.get(&lp.load_pat) {
                raw_sospload.push(HashMap::from([
                    ("m_alive".into(), SdbVal::Bool(true)),
                    ("m_n".into(), SdbVal::I32(sn_idx)),
                    ("m_LC".into(), SdbVal::I32(lc)),
                    ("m_Face".into(), SdbVal::I32(lp.face)),
                    ("m_P".into(), SdbVal::F64(lp.val)),
                    ("m_Pattern".into(), SdbVal::I32(0)),
                ]));
            }
        }
        for (lpat, &tval) in &so.loads_temp {
            if let Some(&lc) = lpat_index.get(lpat) {
                raw_sotload.push(HashMap::from([
                    ("m_alive".into(), SdbVal::Bool(true)),
                    ("m_n".into(), SdbVal::I32(sn_idx)),
                    ("m_lc".into(), SdbVal::I32(lc)),
                    ("m_LT".into(), SdbVal::I32(1)),
                    ("m_T".into(), SdbVal::F64(tval)),
                    ("m_Pattern".into(), SdbVal::I32(0)),
                ]));
            }
        }
    }
    w.write_struct_array("TYSPLoad", &raw_sospload)?;
    w.write_struct_array("TYTLoad", &raw_sotload)?;
    for _ in 0..5 {
        w.write_empty_struct_array(); // SoSLoad..Seastate
    }
    for _ in 0..9 {
        w.write_empty_struct_array(); // Wave..WaveOverwriteSolid
    }

    let mut raw_functions = Vec::with_capacity(model.functions.len());
    let mut raw_tv = Vec::new();
    for (idx, func) in model.functions.values().enumerate() {
        let fnum = (idx + 1) as i32;
        raw_functions.push(HashMap::from([
            ("m_alive".into(), SdbVal::Bool(true)),
            ("m_Name".into(), SdbVal::Str(func.name.clone())),
            (
                "m_type".into(),
                SdbVal::I32(if func.is_time_history { 2 } else { 1 }),
            ),
            ("m_dampratio".into(), SdbVal::F64(func.damping_ratio)),
        ]));
        for &(t, val) in &func.points {
            raw_tv.push(HashMap::from([
                ("m_fnumber".into(), SdbVal::I32(fnum)),
                ("m_time".into(), SdbVal::F64(t)),
                ("m_Value".into(), SdbVal::F64(val)),
            ]));
        }
    }
    w.write_struct_array("FUNCSHUNARRAY", &raw_functions)?;
    w.write_struct_array("TVARRAY", &raw_tv)?;

    // 9. Tail records matching traverse() lines 318..568
    w.write_empty_struct(); // BridgeMiscParameters
    for _ in 0..17 {
        w.write_empty_struct_array(); // BridgeLayout..VClass
    }
    w.write_empty_struct(); // BR
    for _ in 0..6 {
        w.write_empty_struct_array(); // VehRespCompPoint..HingeProp
    }
    w.write_bool(false); // FrameHingeAssignDimed
    for _ in 0..6 {
        w.write_empty_struct_array(); // POParamVD..FLateralBrace
    }
    w.write_empty_struct(); // SteelPreferences
    w.write_bool(false); // SteelResultsAvailable
    w.write_bool(false); // SteelDefaultsSet
    w.write_empty_struct_array(); // SteelData
    w.write_empty_struct(); // ConcretePreferences
    w.write_bool(false); // ConcreteResultsAvailable
    w.write_bool(false); // ConcreteEnvelopesAvailable
    w.write_bool(false); // ConcreteDefaultsSet
    w.write_empty_struct(); // AluminumPreferences
    w.write_bool(false); // AluminumResultsAvailable
    w.write_bool(false); // AluminumDefaultsSet
    w.write_empty_struct(); // ColdFormedPreferences
    w.write_bool(false); // ColdFormedResultsAvailable
    w.write_bool(false); // ColdFormedDefaultsSet
    w.write_empty_struct_array(); // OptimizationD
    w.write_empty_struct_array(); // OptimizationT
    w.write_bool(false); // PunchAPIResultsAvailable
    w.write_bool(false); // PunchAPIDefaultsSet
    w.write_bool(false); // ConsiderFatigue
    w.write_empty_struct(); // ChineseSpecialItems
    w.write_empty_struct(); // BridgeDesignPreferences
    for _ in 0..5 {
        w.write_empty_struct_array(); // DesignRequests..SectionCuts
    }
    w.write_i32(0); // WantedArrangement
    w.write_empty_struct_array(); // ws
    w.write_empty_struct(); // InDBFormat
    w.write_empty_struct(); // InDBZeroFormat
    w.write_empty_struct_array(); // InDBNamedFormat
    w.write_bool(false); // DBAlwaysUseCurrentUnits
    w.write_bool(false); // DBOverrideAutoEFormat
    w.write_bool(false); // DBRegenerateHingeAfterImport
    for _ in 0..9 {
        w.write_empty_struct_array(); // RSNamedSet..NamedPicture
    }
    w.write_empty_struct(); // QuickReport
    w.write_bool(false); // SaveDBAfterAnalysis
    for _ in 0..15 {
        w.write_empty_struct(); // AutoDB..MiscOptions
    }
    w.write_empty_struct_array(); // TimeFunction
    w.write_empty_struct(); // CalculatorTendon
    w.write_empty_struct(); // SASSIControl
    w.write_empty_struct_array(); // SASSISoilLayer
    w.write_i32(0); // FWPFileiDts
    w.write_empty_struct_array(); // FWPMember
    w.write_bool(false); // ProSteelModel
    w.write_i32(0); // APIExtendedEntityData (0 apps)
    w.write_string(""); // ModelLog
    w.write_i32(0); // DateTimeStamp

    let sdb = SdbFile {
        program_name: "SAP2000".to_string(),
        program_version: model.program_version.clone(),
        file_version: "19.00".to_string(),
        thumbnail_png: DEFAULT_THUMBNAIL_PNG.to_vec(),
        decompressed_stream: w.data,
    };
    let projected = extract_sdb_to_model(&sdb)?;
    export_guard::verify_projection(model, &projected)?;
    Ok(sdb)
}

/// Save an experimental checked native projection to a new destination.
/// Validation and packing finish before creation; existing files are never
/// overwritten. Written bytes are synchronized, read back and inspected before
/// success. Failed writes/checks remove the newly created incomplete output.
/// No vendor runtime compatibility or arbitrary-model serialization is claimed.
pub fn save_native_sdb_file(model: &SapModel, path: impl AsRef<Path>) -> Result<(), FormatError> {
    let sdb = pack_model_to_native_sdb(model)?;
    let bytes = write_sdb_bytes(&sdb)?;
    verified_file::write_new(path.as_ref(), &bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture_directory() -> std::path::PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../extracted/files")
    }

    #[test]
    fn entire_vendor_corpus_native_inventory_and_copy_are_lossless() {
        let directory = fixture_directory();
        if !directory.exists() {
            return;
        }
        let mut checked = 0;
        let mut verified_classes = std::collections::BTreeSet::new();
        for entry in std::fs::read_dir(directory).unwrap() {
            let path = entry.unwrap().path();
            if path.extension().and_then(|p| p.to_str()) != Some("sdb") {
                continue;
            }
            let bytes = std::fs::read(&path).unwrap();
            let document = inspect_native_sdb_bytes(&bytes)
                .unwrap_or_else(|error| panic!("{}: {}", path.display(), error));
            assert_eq!(
                document.to_bytes().unwrap(),
                bytes,
                "compressed container copy changed"
            );
            for record in document.records() {
                let Some(expected) = class_versions::observed_class_version(&record.class_id)
                else {
                    continue;
                };
                let field = record
                    .fields
                    .iter()
                    .find(|f| f.name.as_deref() == Some("ClassVersion"))
                    .unwrap_or_else(|| {
                        panic!("{}: {} lacks class tag", path.display(), record.class_id)
                    });
                let actual = i32::from_le_bytes(
                    document.source.decompressed_stream[field.offset..field.offset + 4]
                        .try_into()
                        .unwrap(),
                );
                assert_eq!(
                    actual,
                    expected,
                    "{}: {} tag mismatch",
                    path.display(),
                    record.class_id
                );
                verified_classes.insert(record.class_id.clone());
            }
            let inventory = document.inventory();
            assert_eq!(inventory.parsed_bytes, inventory.stream_bytes);
            assert_eq!(inventory.top_level.first().unwrap().offset, 0);
            for window in inventory.top_level.windows(2) {
                assert_eq!(window[0].offset + window[0].length, window[1].offset);
            }
            let last = inventory.top_level.last().unwrap();
            assert_eq!(last.offset + last.length, inventory.stream_bytes);
            assert_eq!(last.name, "DateTimeStamp");
            // Regenerating gzip still preserves every native record, including structures
            // outside SapModel's projection and all API extended entity data.
            let regenerated =
                read_sdb_bytes(&write_sdb_bytes(&document.to_sdb()).unwrap()).unwrap();
            assert_eq!(
                regenerated.decompressed_stream,
                document.source.decompressed_stream
            );
            checked += 1;
        }
        assert!(checked >= 336, "Only {} native fixtures checked", checked);
        assert_eq!(
            verified_classes.len(),
            57,
            "Every emitted class tag needs source evidence"
        );
    }

    #[test]
    fn typed_native_scalar_patch_changes_only_the_verified_field_bytes() {
        let path = fixture_directory().join("example_1_001.sdb");
        if !path.exists() {
            return;
        }
        let mut document = inspect_native_sdb_file(path).unwrap();
        let original = document.to_sdb().decompressed_stream;
        let index = document
            .records()
            .iter()
            .position(|r| r.class_id == "S_POINTOBJECTS")
            .unwrap();
        let offset = document.records()[index]
            .fields
            .iter()
            .find(|f| f.name.as_deref() == Some("m_gx"))
            .unwrap()
            .offset;
        assert!(document.set_f64(index, "m_Name", 10.0).is_err());
        assert!(document.set_f64(index, "m_gx", f64::NAN).is_err());
        document.set_f64(index, "m_gx", 42.0).unwrap();
        let bytes = document.to_bytes().unwrap();
        let reloaded = inspect_native_sdb_bytes(&bytes).unwrap();
        let modified = reloaded.to_sdb().decompressed_stream;
        assert_eq!(&original[..offset], &modified[..offset]);
        assert_eq!(&original[offset + 8..], &modified[offset + 8..]);
        assert_eq!(&modified[offset..offset + 8], &42.0_f64.to_le_bytes());
        let model = extract_sdb_to_model(&reloaded.to_sdb()).unwrap();
        assert!(model.points.values().any(|point| point.x == 42.0));
    }

    #[test]
    fn private_payload_is_never_classified_as_native() {
        let source = pack_model_to_sdb(&SapModel::new(Units::KipInF)).unwrap();
        assert!(inspect_native_sdb(&source).is_err());
    }

    #[test]
    fn pack_model_to_native_sdb_produces_valid_cbinarystream_and_roundtrips() {
        let mut model = SapModel::new(Units::KnMC);
        model.active_dofs = [true, true, true, false, true, false];

        let mut p1 = PointObj::new("1", 0.0, 0.0, 0.0);
        p1.restraint = [true, true, true, true, true, true];
        let mut p2 = PointObj::new("2", 6.0, 0.0, 0.0);
        p2.spring = [150.0, 0.0, 250.0, 0.0, 0.0, 0.0];
        p2.mass = [2.5, 2.5, 2.5, 0.0, 0.0, 0.0];
        p2.loads_force.push(PointLoadForce {
            load_pat: "DEAD".into(),
            csys: "GLOBAL".into(),
            force: [10.0, 0.0, -50.0, 0.0, 0.0, 0.0],
        });
        let p3 = PointObj::new("3", 6.0, 4.0, 0.0);
        model.points.insert("1".into(), p1);
        model.points.insert("2".into(), p2);
        model.points.insert("3".into(), p3);

        let mut cab_sec = FrameSection::default();
        cab_sec.name = "CAB1".into();
        cab_sec.material = "A992Fy50".into();
        cab_sec.prop_type = FramePropType::Circle;
        cab_sec.area = 0.01;
        cab_sec.t3 = (4.0 * 0.01 / std::f64::consts::PI).sqrt();
        model.frame_sections.insert("CAB1".into(), cab_sec);

        let mut f1 = FrameObj::new("F1", "1", "2", "FSEC1");
        f1.release_j[4] = true;
        f1.pdelta_force = 12.5;
        f1.loads_dist.push(FrameLoadDistributed {
            load_pat: "DEAD".into(),
            my_type: 1,
            dir: 10,
            csys: "GLOBAL".into(),
            rel_dist1: 0.0,
            rel_dist2: 1.0,
            val1: 5.0,
            val2: 5.0,
        });
        model.frames.insert("F1".into(), f1);

        let mut c1 = FrameObj::new("C1", "2", "3", "CAB1");
        c1.cable_params = Some(sap_model::CableParams {
            undeformed_length: Some(3.98),
            target_tension_i: None,
            horizontal_tension: None,
            num_segments: 8,
        });
        model.frames.insert("C1".into(), c1);

        let mut lprop = LinkProp::default();
        lprop.name = "GAP1".into();
        lprop.link_type = LinkPropType::Gap;
        lprop.dof_active[0] = true;
        lprop.dof_nonlinear[0] = true;
        lprop.nonlinear_data_complete = true;
        lprop.ke[0] = 1000.0;
        lprop.k_nl[0] = 5000.0;
        lprop.param1[0] = 0.02;
        lprop.param2[0] = 1.5;
        model.link_props.insert("GAP1".into(), lprop);
        model.links.insert(
            "L1".into(),
            LinkObj {
                name: "L1".into(),
                point_i: "1".into(),
                point_j: Some("2".into()),
                prop: "GAP1".into(),
                angle_deg: 0.0,
                selected: false,
            },
        );

        let mut nl_case = LoadCase::linear_static("NL1", "DEAD", 1.0);
        nl_case.case_type = LoadCaseType::NonlinearStatic;
        nl_case.nl_geom = NLGeomType::LargeDisplacements;
        nl_case.max_iterations = 250;
        nl_case.convergence_tol = 1e-6;
        nl_case.rayleigh_damping = (0.02, 0.005);
        model.load_cases.insert("NL1".into(), nl_case);

        let sdb = pack_model_to_native_sdb(&model).unwrap();
        let doc = inspect_native_sdb(&sdb).unwrap();
        let inv = doc.inventory();
        assert_eq!(inv.parsed_bytes, inv.stream_bytes);
        assert!(inv.classes.contains_key("CABLEPROPERTYARRAY"));
        assert!(inv.classes.contains_key("S_CableData"));
        assert!(inv.classes.contains_key("NLLINKPROPARRAY"));

        let recovered = extract_sdb_to_model(&sdb).unwrap();
        assert_eq!(recovered, model);
    }

    #[test]
    fn vendor_corpus_models_round_trip_through_s2k_and_native_sdb() {
        let directory = fixture_directory();
        if !directory.exists() {
            return;
        }
        let mut checked = 0usize;
        let mut rejected_incomplete = 0usize;
        const EXPECTED_UNMODELED: usize = 46;
        let mut rejected_unmodeled: Vec<String> = Vec::new();
        let mut native_checked = 0usize;
        let mut nonlinear_links_checked = 0usize;
        let mut cables_checked = 0usize;
        for entry in std::fs::read_dir(directory).unwrap() {
            let path = entry.unwrap().path();
            if path.extension().and_then(|p| p.to_str()) != Some("sdb") {
                continue;
            }
            let sdb = read_sdb_file(&path).unwrap();
            let model = match extract_sdb_to_model(&sdb) {
                Ok(m) => m,
                Err(FormatError::UnsupportedNativeCase { .. }) => continue,
                Err(err) => panic!("{}: {}", path.display(), err),
            };
            if model
                .link_props
                .values()
                .any(|lp| lp.dof_nonlinear.iter().any(|&v| v))
            {
                nonlinear_links_checked += 1;
            }
            if model.frames.values().any(|f| f.cable_params.is_some()) {
                cables_checked += 1;
            }
            let s2k_roundtrip = parse_s2k(&write_s2k(&model))
                .unwrap_or_else(|e| panic!("{} S2K roundtrip failed: {}", path.display(), e));
            assert_eq!(
                s2k_roundtrip,
                model,
                "{}: S2K round-trip mismatch",
                path.display()
            );

            checked += 1;
            let native_sdb = match pack_model_to_native_sdb(&model) {
                Ok(sdb) => sdb,
                Err(FormatError::UnsupportedNativeExport { field, reason })
                    if model.link_props.iter().any(|(name, prop)| {
                        !prop.nonlinear_data_complete
                            && field == format!("link_props.{name}.nonlinear_data_complete")
                    }) => {
                        eprintln!("{}: correctly rejected incomplete native source law ({reason})", path.display());
                        rejected_incomplete += 1;
                        continue;
                    }
                // Hinges, link gravity loads, advanced link axes and per-mode
                // damping are recorded but not representable; synthesis would
                // silently omit them, so it must refuse.
                Err(FormatError::UnsupportedNativeExport { field, .. })
                    if field == "unmodeled_source" && !model.unmodeled_source.is_empty() =>
                {
                    rejected_unmodeled.push(
                        path.file_name().unwrap().to_string_lossy().into_owned(),
                    );
                    continue;
                }
                Err(e) => panic!("{} native pack failed: {}", path.display(), e),
            };
            let doc = inspect_native_sdb(&native_sdb)
                .unwrap_or_else(|e| panic!("{} native inspect failed: {}", path.display(), e));
            assert_eq!(
                doc.inventory().parsed_bytes,
                doc.inventory().stream_bytes,
                "{}: native stream byte mismatch",
                path.display()
            );
            let native_roundtrip = extract_sdb_to_model(&native_sdb)
                .unwrap_or_else(|e| panic!("{} native extract failed: {}", path.display(), e));
            assert_eq!(
                native_roundtrip,
                model,
                "{}: native SDB round-trip mismatch",
                path.display()
            );
            native_checked += 1;
        }
        rejected_unmodeled.sort();
        eprintln!("Vendor projection: {checked} S2K roundtrips, {native_checked} checked native roundtrips, {rejected_incomplete} incomplete-source native rejections, unmodeled-source native rejections: {rejected_unmodeled:?}");
        assert_eq!(
            checked,
            native_checked + rejected_incomplete + rejected_unmodeled.len()
        );
        if checked >= 334 {
            // One file per kind of recorded data; the full list is printed above.
            // Files that also have an incomplete link law are counted there.
            for expected in [
                "example_1_009a.sdb",       // tendon object and prestress load
                "example_1_013a.sdb",       // frame line springs
                "example_1_015.sdb",        // frame added mass
                "example_1_026a.sdb",       // frame hinges
                "example_2_006_thick.sdb",  // area gravity load
                "example_2_016a_thick.sdb", // area auto-mesh options not modeled
                "example_2_009a_thick.sdb", // area springs
                "example_3_005.sdb",        // area pore pressure
                "example_4_003.sdb",        // axisymmetric-solid rotate load
                "example_5_006.sdb",        // solid gravity load
                "example_5_012a.sdb",       // solid surface springs
                "example_5_013.sdb",        // solid pore pressure
                "example_6_006.sdb",        // per-mode modal damping
                "example_6_008.sdb",        // link gravity load and advanced axes
            ] {
                assert!(
                    rejected_unmodeled.iter().any(|name| name == expected),
                    "{expected} should be refused for unmodeled source data"
                );
            }
            assert_eq!(rejected_unmodeled.len(), EXPECTED_UNMODELED);
            assert_eq!(rejected_incomplete, 5);
            assert_eq!(native_checked, 334 - 5 - EXPECTED_UNMODELED);
        }
        assert!(native_checked > 0, "No supported native models checked");
        assert!(checked >= 334, "Only {} vendor models checked", checked);
        assert!(
            nonlinear_links_checked > 0,
            "Expected nonlinear link fixtures"
        );
        assert!(cables_checked > 0, "Expected cable fixtures");
    }
}
