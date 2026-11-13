//! Raw S2K preservation and conservative, identity-based edit writeback.
use super::{parse_kv_row, write_s2k_canonical};
use crate::{ExportReport, FormatError};
use sap_model::SapModel;
use std::collections::{BTreeMap, BTreeSet};

type Row = BTreeMap<String, String>;
pub(super) fn supported_table(name: &str) -> bool {
    matches!(
        name,
        "PROGRAM CONTROL"
            | "PROJECT INFORMATION"
            | "ACTIVE DEGREES OF FREEDOM"
            | "COORDINATE SYSTEMS"
            | "GRID LINES"
            | "MATERIAL PROPERTIES 01 - GENERAL"
            | "MATERIAL PROPERTIES 02 - BASIC MECHANICAL PROPERTIES"
            | "MATERIAL PROPERTIES 03A - STEEL DATA"
            | "MATERIAL PROPERTIES 03B - CONCRETE DATA"
            | "FRAME SECTION PROPERTIES 01 - GENERAL"
            | "FRAME SECTION PROPERTIES 02 - CONCRETE COLUMN"
            | "FRAME SECTION PROPERTIES 03 - CONCRETE BEAM"
            | "FRAME SECTION PROPERTIES 04 - AUTO SELECT"
            | "FRAME SECTION PROPERTIES 05 - NONPRISMATIC"
            | "AREA SECTION PROPERTIES"
            | "SOLID PROPERTY DEFINITIONS"
            | "LINK PROPERTY DEFINITIONS 01 - GENERAL"
            | "LINK PROPERTY DEFINITIONS 02 - LINEAR"
            | "JOINT CONSTRAINT DEFINITIONS"
            | "JOINT COORDINATES"
            | "JOINT RESTRAINT ASSIGNMENTS"
            | "JOINT SPRING ASSIGNMENTS 1 - UNCOUPLED"
            | "JOINT SPRING ASSIGNMENTS 2 - COUPLED"
            | "JOINT ADDED MASS ASSIGNMENTS"
            | "JOINT LOCAL AXES ASSIGNMENTS 1 - STANDARD"
            | "JOINT LOCAL AXES ASSIGNMENTS 1 - TYPICAL"
            | "JOINT LOCAL AXES ASSIGNMENTS 2 - ADVANCED"
            | "JOINT CONSTRAINT ASSIGNMENTS"
            | "JOINT PATTERN DEFINITIONS"
            | "JOINT PATTERN ASSIGNMENTS"
            | "JOINT LOADS - FORCE"
            | "JOINT LOADS - GROUND DISPLACEMENT"
            | "CONNECTIVITY - FRAME"
            | "FRAME SECTION ASSIGNMENTS"
            | "FRAME PROPERTY MODIFIERS"
            | "FRAME RELEASE ASSIGNMENTS 1 - GENERAL"
            | "FRAME RELEASE ASSIGNMENTS 2 - PARTIAL FIXITY"
            | "FRAME OFFSET ALONG LENGTH ASSIGNMENTS"
            | "FRAME INSERTION POINT ASSIGNMENTS"
            | "FRAME LOCAL AXES ASSIGNMENTS 1 - STANDARD"
            | "FRAME LOCAL AXES ASSIGNMENTS 1 - TYPICAL"
            | "FRAME TENSION AND COMPRESSION LIMITS"
            | "FRAME PDELTA FORCE ASSIGNMENTS"
            | "CABLE SHAPE DATA"
            | "FRAME LOADS - DISTRIBUTED"
            | "FRAME LOADS - POINT"
            | "FRAME LOADS - TEMPERATURE"
            | "FRAME LOADS - GRAVITY"
            | "CONNECTIVITY - AREA"
            | "AREA SECTION ASSIGNMENTS"
            | "AREA PROPERTY MODIFIERS"
            | "AREA LOADS - UNIFORM"
            | "AREA LOADS - SURFACE PRESSURE"
            | "AREA LOADS - TEMPERATURE"
            | "CONNECTIVITY - SOLID"
            | "SOLID LOADS - SURFACE PRESSURE"
            | "SOLID LOADS - TEMPERATURE"
            | "CONNECTIVITY - LINK"
            | "LINK PROPERTY ASSIGNMENTS"
            | "FUNCTION - RESPONSE SPECTRUM - USER"
            | "FUNCTION - TIME HISTORY - USER"
            | "LOAD PATTERN DEFINITIONS"
            | "LOAD CASE DEFINITIONS"
            | "CASE - STATIC 1 - LOAD ASSIGNMENTS"
            | "CASE - MODAL 1 - GENERAL"
            | "CASE - RESPONSE SPECTRUM 1 - GENERAL"
            | "CASE - RESPONSE SPECTRUM 2 - LOAD ASSIGNMENTS"
            | "CASE - TIME HISTORY 2 - LOAD ASSIGNMENTS"
            | "COMBINATION DEFINITIONS"
            | "RUST UNMODELED SOURCE DATA"
    )
}

#[derive(Debug, Clone)]
struct Block {
    name: String,
    raw: String,
}
#[derive(Debug)]
struct Document {
    prefix: String,
    blocks: Vec<Block>,
    suffix: String,
}
fn document(text: &str) -> Document {
    let mut doc = Document {
        prefix: String::new(),
        blocks: Vec::new(),
        suffix: String::new(),
    };
    let mut ended = false;
    for line in text.split_inclusive('\n') {
        let trimmed = line.trim_start_matches('\u{feff}').trim();
        if ended || trimmed.eq_ignore_ascii_case("END TABLE DATA") {
            ended = true;
            doc.suffix.push_str(line);
            continue;
        }
        if trimmed
            .get(..6)
            .is_some_and(|s| s.eq_ignore_ascii_case("TABLE:"))
        {
            doc.blocks.push(Block {
                name: trimmed[6..].trim().trim_matches('"').to_ascii_uppercase(),
                raw: line.into(),
            });
        } else if let Some(block) = doc.blocks.last_mut() {
            block.raw.push_str(line);
        } else {
            doc.prefix.push_str(line);
        }
    }
    doc
}
fn records(block: &Block) -> Result<Vec<(String, Row)>, FormatError> {
    let mut records = Vec::new();
    let mut pending = String::new();
    for line in block.raw.lines().skip(1) {
        let line = line.trim();
        if line.is_empty() || line.starts_with('$') || line.starts_with(';') {
            continue;
        }
        if let Some(part) = line.strip_suffix(" _") {
            pending.push_str(part);
            pending.push(' ');
            continue;
        }
        pending.push_str(line);
        let raw = std::mem::take(&mut pending);
        records.push((raw.clone(), parse_kv_row(&raw)?));
    }
    if !pending.is_empty() {
        records.push((pending.clone(), parse_kv_row(&pending)?));
    }
    Ok(records)
}
fn grouped(doc: &Document) -> Result<BTreeMap<String, Vec<(String, Row)>>, FormatError> {
    let mut out: BTreeMap<String, Vec<(String, Row)>> = BTreeMap::new();
    for block in &doc.blocks {
        if supported_table(&block.name) {
            out.entry(block.name.clone())
                .or_default()
                .extend(records(block)?);
        }
    }
    Ok(out)
}
fn known_columns(
    table: &str,
    baseline: &[(String, Row)],
    schema: &BTreeMap<String, BTreeSet<String>>,
) -> BTreeSet<String> {
    let mut columns = schema.get(table).cloned().unwrap_or_default();
    let _ = baseline;
    // Aliases consumed by the projection must never be replayed beside edited
    // canonical fields, where they could override them on the next import.
    if table == "JOINT COORDINATES" {
        columns.extend(["GLOBALX", "GLOBALY", "GLOBALZ", "XORR", "Y", "Z"].map(str::to_owned));
    }
    if matches!(table, "FRAME LOADS - DISTRIBUTED" | "FRAME LOADS - POINT") {
        columns.extend(["ABSDIST", "ABSDISTA", "ABSDISTB"].map(str::to_owned));
    }
    if table.starts_with("FUNCTION -") {
        columns.extend(["PERIOD", "TIME", "ACCEL", "VALUE"].map(str::to_owned));
    }
    if matches!(table, "FRAME LOADS - DISTRIBUTED" | "FRAME LOADS - POINT") {
        columns.extend(["TYPE", "MYTYPE"].map(str::to_owned));
    }
    columns
}
fn identity(table: &str, row: &Row) -> Vec<(String, String)> {
    if table.starts_with("FUNCTION -") {
        let mut id = Vec::new();
        if let Some(name) = row.get("NAME") {
            id.push(("NAME".into(), name.clone()));
        }
        if let Some(x) = row.get("PERIOD").or_else(|| row.get("TIME")) {
            id.push((
                "SAMPLE".into(),
                x.replace(',', ".")
                    .parse::<f64>()
                    .map(|v| v.to_string())
                    .unwrap_or_else(|_| x.clone()),
            ));
        }
        return id;
    }
    let keys: &[&str] = match table {
        "PROGRAM CONTROL" | "ACTIVE DEGREES OF FREEDOM" => &[],
        "GRID LINES" => &["COORDSYS", "AXISDIR", "GRIDID"],
        "FRAME SECTION PROPERTIES 04 - AUTO SELECT" => &["LISTNAME", "AUTOSECTION"],
        _ => &[
            "JOINT",
            "FRAME",
            "AREA",
            "SOLID",
            "LINK",
            "MATERIAL",
            "SECTIONNAME",
            "SECTION",
            "SOLIDPROP",
            "NAME",
            "LOADPAT",
            "LOADCASE",
            "LOADNAME",
            "CASE",
            "CASENAME",
            "COMBONAME",
            "ITEM",
            "DOF",
            "PATTERN",
            "FUNCTION",
            "TIME",
            "PERIOD",
        ],
    };
    keys.iter()
        .filter_map(|key| {
            row.get(*key).map(|value| {
                (
                    (*key).into(),
                    if matches!(*key, "TIME" | "PERIOD") {
                        value
                            .parse::<f64>()
                            .map(|v| v.to_string())
                            .unwrap_or_else(|_| value.clone())
                    } else {
                        value.clone()
                    },
                )
            })
        })
        .collect()
}
// Raw unknown tokens retain key spelling, quoted strings and escaped double
// quotes. Known fields are regenerated, so they cannot compete with stale values.
fn extra_tokens(raw: &str, known: &BTreeSet<String>) -> Vec<String> {
    let bytes = raw.as_bytes();
    let mut i = 0;
    let mut out = Vec::new();
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
        let key = raw[start..i].to_ascii_uppercase();
        while i < bytes.len() && bytes[i] != b'=' {
            i += 1;
        }
        i += 1;
        while i < bytes.len() && bytes[i].is_ascii_whitespace() {
            i += 1;
        }
        if i < bytes.len() && bytes[i] == b'"' {
            i += 1;
            while i < bytes.len() {
                if bytes[i] == b'"' {
                    i += 1;
                    if i < bytes.len() && bytes[i] == b'"' {
                        i += 1;
                    } else {
                        break;
                    }
                } else {
                    i += 1;
                }
            }
        } else {
            while i < bytes.len() && !bytes[i].is_ascii_whitespace() {
                i += 1;
            }
        }
        if !known.contains(&key) {
            out.push(raw[start..i].into());
        }
    }
    out
}
fn conflict(table: &str, reason: &str) -> FormatError {
    FormatError::PreservationConflict(format!("table '{table}': {reason}; retain the original source, or explicitly clear s2k_preservation to export only supported data"))
}

pub(super) fn import_warnings(
    source: &str,
    canonical: &str,
    schema: &BTreeMap<String, BTreeSet<String>>,
) -> Result<Vec<String>, FormatError> {
    let original = document(source);
    let baseline = grouped(&document(canonical))?;
    let mut warnings = BTreeSet::new();
    for block in &original.blocks {
        if !supported_table(&block.name) {
            warnings.insert(format!(
                "Unsupported S2K table preserved verbatim, not interpreted: {}",
                block.name
            ));
            continue;
        }
        let base = baseline.get(&block.name).map(Vec::as_slice).unwrap_or(&[]);
        let known = known_columns(&block.name, base, schema);
        let unknown: BTreeSet<_> = records(block)?
            .iter()
            .flat_map(|(_, row)| row.keys().filter(|key| !known.contains(*key)).cloned())
            .collect();
        if !unknown.is_empty() {
            warnings.insert(format!(
                "S2K unknown columns preserved in '{}': {}",
                block.name,
                unknown.into_iter().collect::<Vec<_>>().join(", ")
            ));
        }
        let identities: BTreeSet<_> = base
            .iter()
            .map(|(_, row)| identity(&block.name, row))
            .collect();
        if base.is_empty()
            || records(block)?
                .iter()
                .any(|(_, row)| !identities.contains(&identity(&block.name, row)))
        {
            warnings.insert(format!("S2K table '{}' has source records absent from the supported export projection; records retained verbatim and edits to this table require explicit resolution", block.name));
        }
    }
    Ok(warnings.into_iter().collect())
}

/// Check whether the saved source contains anything beyond the supported model
/// projection. Native synthesis may discard source spelling, never opaque data.
pub(crate) fn native_projection_issues(
    saved: &sap_model::S2kPreservation,
) -> Result<Vec<String>, FormatError> {
    let (parsed, mut schema) = super::parse_s2k_internal(&saved.source_text, false)?;
    let canonical = write_s2k_canonical(&parsed.model);
    if canonical != saved.canonical_baseline {
        return Ok(vec![
            "S2K source and its saved projection baseline disagree".into(),
        ]);
    }
    let source_document = document(&saved.source_text);
    let canonical_document = document(&canonical);
    let mut issues = Vec::new();
    let comment_or_blank = |line: &str| {
        let line = line.trim_start_matches('\u{feff}').trim();
        line.is_empty() || line.starts_with('$') || line.starts_with(';')
    };
    if source_document.prefix.lines().any(|line| {
        !comment_or_blank(line)
            && !canonical_document
                .prefix
                .lines()
                .any(|known| known.trim() == line.trim())
    }) {
        issues.push("Uninterpreted text before the first S2K table".into());
    }
    if source_document
        .suffix
        .lines()
        .any(|line| !comment_or_blank(line) && !line.trim().eq_ignore_ascii_case("END TABLE DATA"))
    {
        issues.push("Uninterpreted text after END TABLE DATA".into());
    }
    let original = grouped(&source_document)?;
    let baseline = grouped(&canonical_document)?;
    let baseline_by_owner: BTreeMap<_, BTreeMap<_, Vec<_>>> = baseline
        .iter()
        .map(|(table, rows)| {
            let mut owners: BTreeMap<_, Vec<_>> = BTreeMap::new();
            for (_, row) in rows {
                owners.entry(identity(table, row)).or_default().push(row);
            }
            (table, owners)
        })
        .collect();
    // Some canonical fields are generated constants, not parser inputs. Their
    // exact projected value is harmless (e.g. SymType=Isotropic), but a different
    // supplied value must remain opaque (e.g. SymType=Orthotropic).
    for (table, rows) in &original {
        let mut constant_columns: BTreeMap<String, bool> = BTreeMap::new();
        for (_, row) in rows {
            for (key, value) in row {
                let represented = baseline_by_owner
                    .get(table)
                    .and_then(|owners| owners.get(&identity(table, row)))
                    .into_iter()
                    .flatten()
                    .any(|base| {
                        base.get(key).is_some_and(|projected| {
                            same_projected_cell(table, key, value, projected)
                        })
                    });
                *constant_columns.entry(key.clone()).or_insert(true) &= represented;
            }
        }
        for (key, represented) in constant_columns {
            if represented {
                schema.entry(table.clone()).or_default().insert(key);
            }
        }
        if table == "JOINT COORDINATES"
            && rows
                .iter()
                .any(|(_, row)| translated_coordinates(row, &parsed.model))
        {
            schema
                .entry(table.clone())
                .or_default()
                .extend(["COORDSYS", "COORDTYPE"].map(str::to_owned));
        }
    }
    // Empty recognized blocks are syntax, not omitted source records. Keep
    // unknown blocks in the check even when empty: their meaning is unknown.
    let checked_source = source_document
        .blocks
        .iter()
        .filter_map(|block| match records(block) {
            Ok(rows) if supported_table(&block.name) && rows.is_empty() => None,
            _ => Some(block.raw.as_str()),
        })
        .collect::<String>();
    issues.extend(import_warnings(&checked_source, &canonical, &schema)?);
    for (table, rows) in original {
        let mut source_counts = BTreeMap::new();
        for (_, row) in rows {
            let translated =
                table == "JOINT COORDINATES" && translated_coordinates(&row, &parsed.model);
            for (key, value) in &row {
                if translated
                    && matches!(key.as_str(), "COORDSYS" | "COORDTYPE" | "XORR" | "Y" | "Z")
                {
                    continue;
                }
                let candidates: Vec<_> = baseline_by_owner
                    .get(&table)
                    .and_then(|owners| owners.get(&identity(&table, &row)))
                    .into_iter()
                    .flatten()
                    .filter_map(|base| base.get(key))
                    .collect();
                if !candidates.is_empty()
                    && !candidates
                        .iter()
                        .any(|other| same_projected_cell(&table, key, value, other))
                {
                    issues.push(format!("S2K table '{table}' field '{key}' is not represented by its projected value"));
                }
            }
            let alias_groups: &[&[&str]] = if table == "JOINT COORDINATES" && !translated {
                &[&["GLOBALX", "XORR"], &["GLOBALY", "Y"], &["GLOBALZ", "Z"]]
            } else if table.starts_with("FUNCTION -") {
                &[&["PERIOD", "TIME"], &["ACCEL", "VALUE"]]
            } else {
                &[]
            };
            for aliases in alias_groups {
                let supplied: Vec<_> = aliases.iter().filter_map(|key| row.get(*key)).collect();
                if supplied.windows(2).any(|pair| {
                    pair[0] != pair[1]
                        && match (
                            pair[0].replace(',', ".").parse::<f64>(),
                            pair[1].replace(',', ".").parse::<f64>(),
                        ) {
                            (Ok(a), Ok(b)) => a != b,
                            _ => true,
                        }
                }) {
                    issues.push(format!(
                        "S2K table '{table}' contains conflicting aliases: {}",
                        aliases.join("/")
                    ));
                }
            }
            *source_counts
                .entry(identity(&table, &row))
                .or_insert(0usize) += 1;
        }
        let mut baseline_counts = BTreeMap::new();
        for (_, row) in baseline.get(&table).into_iter().flatten() {
            *baseline_counts
                .entry(identity(&table, row))
                .or_insert(0usize) += 1;
        }
        if source_counts
            .iter()
            .any(|(id, count)| *count > baseline_counts.get(id).copied().unwrap_or(0))
        {
            issues.push(format!("S2K table '{table}' contains source records merged or omitted by the supported projection"));
        }
    }
    Ok(issues)
}

fn translated_coordinates(row: &Row, model: &SapModel) -> bool {
    if !row
        .get("COORDTYPE")
        .is_none_or(|kind| kind.eq_ignore_ascii_case("Cartesian"))
    {
        return false;
    }
    let csys = row.get("COORDSYS").map(String::as_str).unwrap_or("GLOBAL");
    let Some(system) = model
        .coord_systems
        .get(if csys.eq_ignore_ascii_case("GLOBAL") {
            "GLOBAL"
        } else {
            csys
        })
    else {
        return false;
    };
    if system.angles_deg != [0.; 3] {
        return false;
    }
    ["XORR", "Y", "Z"]
        .into_iter()
        .zip(["GLOBALX", "GLOBALY", "GLOBALZ"])
        .enumerate()
        .all(|(axis, (local, global))| {
            let number = |key: &str| row.get(key)?.replace(',', ".").parse::<f64>().ok();
            match (number(local), number(global)) {
                (Some(local), Some(global)) => {
                    (local + system.origin[axis] - global).abs()
                        <= 1e-12 * global.abs().max(local.abs()).max(1.)
                }
                _ => false,
            }
        })
}

fn same_projected_cell(table: &str, key: &str, source: &str, projected: &str) -> bool {
    if same_cell(source, projected) {
        return true;
    }
    if table == "PROGRAM CONTROL" && matches!(key, "CURRUNITS" | "DATABASEUNITS") {
        return matches!((source.parse::<sap_units::Units>(), projected.parse::<sap_units::Units>()), (Ok(a), Ok(b)) if a == b);
    }
    if table == "MATERIAL PROPERTIES 01 - GENERAL" && key == "TYPE" {
        return matches!(
            source.to_ascii_uppercase().as_str(),
            "STEEL"
                | "CONCRETE"
                | "CONC"
                | "ALUMINUM"
                | "ALUM"
                | "COLDFORMED"
                | "COLD FORMED"
                | "REBAR"
                | "TENDON"
                | "NODESIGN"
        ) && super::parse_mat_type(source) == super::parse_mat_type(projected);
    }
    if table == "LOAD PATTERN DEFINITIONS" && key == "TYPE" {
        return super::parse_load_pat_type(source) != sap_model::LoadPatternType::Other
            && super::parse_load_pat_type(source) == super::parse_load_pat_type(projected);
    }
    if key == "DIR" {
        let name = source.to_ascii_uppercase();
        let recognized = matches!(
            name.as_str(),
            "LOCAL 1"
                | "LOCAL1"
                | "LOCAL 2"
                | "LOCAL2"
                | "LOCAL 3"
                | "LOCAL3"
                | "X"
                | "GLOBAL X"
                | "Y"
                | "GLOBAL Y"
                | "Z"
                | "GLOBAL Z"
                | "PROJ X"
                | "PROJECTED X"
                | "PROJ Y"
                | "PROJECTED Y"
                | "PROJ Z"
                | "PROJECTED Z"
                | "GRAVITY"
                | "GRAV"
                | "PROJ GRAVITY"
                | "PROJ GRAV"
        ) || name
            .parse::<i32>()
            .is_ok_and(|value| (1..=11).contains(&value));
        return recognized && super::parse_load_dir(source) == super::parse_load_dir(projected);
    }
    false
}

fn same_cell(source: &str, projected: &str) -> bool {
    if source.eq_ignore_ascii_case(projected) {
        return true;
    }
    if let (Ok(a), Ok(b)) = (
        source.replace(',', ".").parse::<f64>(),
        projected.replace(',', ".").parse::<f64>(),
    ) {
        return a == b;
    }
    let boolean = |value: &str| match value.to_ascii_uppercase().as_str() {
        "YES" | "TRUE" | "1" | "Y" | "ON" => Some(true),
        "NO" | "FALSE" | "0" | "N" | "OFF" => Some(false),
        _ => None,
    };
    matches!((boolean(source), boolean(projected)), (Some(a), Some(b)) if a == b)
}

pub(super) fn write(model: &SapModel) -> Result<ExportReport, FormatError> {
    let canonical = write_s2k_canonical(model);
    let mut warnings = Vec::new();
    if model
        .link_props
        .values()
        .any(|p| p.link_type != sap_model::LinkPropType::Linear)
    {
        warnings.push("RustNonlinear is a reconstruction extension, not a verified SAP2000 v19 S2K nonlinear flag contract; vendor nonlinear serialization is not claimed.".into());
    }
    let Some(saved) = &model.s2k_preservation else {
        return Ok(ExportReport {
            text: canonical,
            warnings,
        });
    };
    let original = document(&saved.source_text);
    let baseline = grouped(&document(&saved.canonical_baseline))?;
    let current_doc = document(&canonical);
    let current = grouped(&current_doc)?;
    let (_, schema) = super::parse_s2k_internal(&saved.source_text, false)?;
    let preserved = import_warnings(&saved.source_text, &saved.canonical_baseline, &schema)?;
    let has_opaque = !preserved.is_empty();
    warnings.extend(preserved);
    if canonical == saved.canonical_baseline {
        return Ok(ExportReport {
            text: saved.source_text.clone(),
            warnings,
        });
    }
    let units = |tables: &BTreeMap<String, Vec<(String, Row)>>| {
        tables
            .get("PROGRAM CONTROL")
            .and_then(|rows| rows.first())
            .map(|(_, row)| {
                (
                    row.get("CURRUNITS").cloned(),
                    row.get("DATABASEUNITS").cloned(),
                )
            })
    };
    if units(&baseline) != units(&current) && has_opaque {
        return Err(conflict(
            "PROGRAM CONTROL",
            "unit changes cannot safely scale preserved unknown columns/tables",
        ));
    }
    let mut out = original.prefix.clone();
    let mut emitted = BTreeSet::new();
    for block in &original.blocks {
        if !supported_table(&block.name) || !baseline.contains_key(&block.name) {
            if supported_table(&block.name) && current.contains_key(&block.name) {
                return Err(conflict(
                    &block.name,
                    "new generated table would collide with previously unprojected source records",
                ));
            }
            out.push_str(&block.raw);
            continue;
        }
        let base = &baseline[&block.name];
        let now = current.get(&block.name).map(Vec::as_slice).unwrap_or(&[]);
        let unchanged = base.iter().map(|(_, r)| r).eq(now.iter().map(|(_, r)| r));
        if unchanged {
            out.push_str(&block.raw);
            emitted.insert(block.name.clone());
            continue;
        }
        if !emitted.insert(block.name.clone()) {
            continue;
        }
        let known = known_columns(&block.name, base, &schema);
        let mut base_counts: BTreeMap<Vec<(String, String)>, usize> = BTreeMap::new();
        for (_, row) in base {
            *base_counts.entry(identity(&block.name, row)).or_default() += 1;
        }
        let mut now_counts: BTreeMap<Vec<(String, String)>, usize> = BTreeMap::new();
        for (_, row) in now {
            *now_counts.entry(identity(&block.name, row)).or_default() += 1;
        }
        let mut extras: BTreeMap<Vec<(String, String)>, Vec<String>> = BTreeMap::new();
        for original_block in original.blocks.iter().filter(|b| b.name == block.name) {
            for (raw, row) in records(original_block)? {
                let id = identity(&block.name, &row);
                if !base_counts.contains_key(&id) {
                    return Err(conflict(&block.name,"source contains an unprojected record that would be lost by regenerating the edited table"));
                }
                let tokens = extra_tokens(&raw, &known);
                if tokens.is_empty() {
                    continue;
                }
                let count = base_counts[&id];
                if count != 1 {
                    return Err(conflict(&block.name, "unknown-column owner cannot be associated with exactly one baseline record"));
                }
                if extras.insert(id, tokens).is_some() {
                    return Err(conflict(
                        &block.name,
                        "multiple original records share an unknown-column owner identity",
                    ));
                }
            }
        }
        for (id, tokens) in &extras {
            let count = now_counts.get(id).copied().unwrap_or(0);
            if count > 1 {
                return Err(conflict(
                    &block.name,
                    "edited records have ambiguous identities for preserved columns",
                ));
            }
            if count == 0 {
                warnings.push(format!(
                    "Removed preserved columns with deleted owner in '{}': {:?} ({})",
                    block.name,
                    id,
                    tokens.len()
                ));
            }
        }
        if !now.is_empty() {
            out.push_str(&format!("TABLE:  \"{}\"\n", block.name));
            for (raw, row) in now {
                let tokens = extras.get(&identity(&block.name, row));
                let shadowed: BTreeSet<_> = tokens
                    .into_iter()
                    .flatten()
                    .filter_map(|token| {
                        token
                            .split_once('=')
                            .map(|(k, _)| k.trim().to_ascii_uppercase())
                    })
                    .collect();
                let retained: BTreeSet<_> = row
                    .keys()
                    .filter(|k| !shadowed.contains(*k))
                    .cloned()
                    .collect();
                let rendered = if shadowed.is_empty() {
                    raw.clone()
                } else {
                    let omitted: BTreeSet<_> = row
                        .keys()
                        .filter(|k| !retained.contains(*k))
                        .cloned()
                        .collect();
                    extra_tokens(raw, &omitted).join("   ")
                };
                out.push_str("   ");
                out.push_str(&rendered);
                if let Some(tokens) = tokens {
                    for token in tokens {
                        out.push_str("   ");
                        out.push_str(token);
                    }
                }
                out.push('\n');
            }
            out.push('\n');
        }
    }
    for block in &current_doc.blocks {
        if emitted.insert(block.name.clone()) {
            out.push_str(&block.raw);
        }
    }
    if has_opaque {
        warnings.push("Preserved opaque records were not interpreted or updated; verify references to renamed/deleted objects and unsupported assignments before vendor use.".into());
    }
    if original.suffix.is_empty() {
        out.push_str("END TABLE DATA\n");
    } else {
        out.push_str(&original.suffix);
    }
    Ok(ExportReport {
        text: out,
        warnings,
    })
}
