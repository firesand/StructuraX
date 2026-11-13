//! Native sections that carry analysis-relevant data but that the model
//! importer does not interpret. Their live records are counted from the
//! full-stream inventory and recorded as unmodeled source data, so analysis
//! and native synthesis refuse instead of silently working on a different
//! model. See docs/reconstruction/unmodeled-source.md.
use super::native::{inspect_native_sdb, NativeRecord, NativeTopLevelField};
use super::SdbFile;
use sap_model::{SapModel, UnmodeledScope, UnmodeledSourceData};
use std::collections::BTreeMap;

#[derive(Clone, Copy, PartialEq)]
enum Affects {
    /// Stiffness, geometry or weight: every case.
    AllCases,
    /// Mass only: cases that use mass.
    DynamicCases,
    /// A load: cases applying the record's load pattern (`m_lc` / `m_LC`).
    LoadPattern,
}

/// `(section, what one live record is, what it affects)`.
///
/// The first group is non-empty in the supplied corpus only in files that
/// really make the assignment, and every record there carries `m_alive`. The
/// second group is empty in all 336 corpus files; its entries follow the
/// section names in the vendor schema and have no fixture.
///
/// The object-level area modifiers, added mass and auto-mesh (`ShModifier`,
/// `ShMass`, `ShMesh`) are imported by the model reader and are not listed.
const SKIPPED_SECTIONS: &[(&str, &str, Affects)] = &[
    ("FSpring", "frame line-spring assignment", Affects::AllCases),
    ("FMass", "frame added-mass assignment", Affects::DynamicCases),
    ("ShSpring", "area spring assignment", Affects::AllCases),
    ("SSpring", "solid surface-spring assignment", Affects::AllCases),
    ("PTLoad", "tendon prestress load", Affects::LoadPattern),
    ("ShGLoad", "area gravity load", Affects::LoadPattern),
    ("ShPPLoad", "area pore-pressure load", Affects::LoadPattern),
    ("AsoRtLoad", "axisymmetric-solid rotate load", Affects::LoadPattern),
    ("SoGLoad", "solid gravity load", Affects::LoadPattern),
    ("SoPPLoad", "solid pore-pressure load", Affects::LoadPattern),
    // No corpus fixture below this line.
    ("PanelZone", "panel-zone assignment", Affects::AllCases),
    ("MatTemp", "material-temperature assignment", Affects::AllCases),
    ("FFireProofing", "frame fireproofing assignment", Affects::AllCases),
    ("Seismic", "auto seismic load definition", Affects::LoadPattern),
    ("Wind", "auto wind load definition", Affects::LoadPattern),
    ("FSLoad", "frame strain load", Affects::LoadPattern),
    ("FDefLoad", "frame deformation load", Affects::LoadPattern),
    ("FTFLoad", "frame target-force load", Affects::LoadPattern),
    ("ShUFLoad", "area uniform-to-frame load", Affects::LoadPattern),
    ("ShSLoad", "area strain load", Affects::LoadPattern),
    ("ShWPLoad", "area wind-pressure load", Affects::LoadPattern),
    ("SoSLoad", "solid strain load", Affects::LoadPattern),
    ("NLDefLoad", "link deformation load", Affects::LoadPattern),
    ("NLTFLoad", "link target-force load", Affects::LoadPattern),
];

fn field<'a>(record: &NativeRecord, bytes: &'a [u8], name: &str) -> Option<&'a [u8]> {
    record
        .fields
        .iter()
        .find(|f| f.name.as_deref() == Some(name))
        .and_then(|f| bytes.get(f.offset..f.offset + f.length))
}

fn flag(record: &NativeRecord, bytes: &[u8], name: &str) -> Option<bool> {
    field(record, bytes, name)
        .filter(|b| b.len() == 1)
        .map(|b| b[0] != 0)
}

fn integer(record: &NativeRecord, bytes: &[u8], name: &str) -> Option<i32> {
    field(record, bytes, name)
        .and_then(|b| <[u8; 4]>::try_from(b).ok())
        .map(i32::from_le_bytes)
}

/// Records directly inside a top-level section (not nested in another record
/// of that section) whose `m_alive` flag is absent or true.
fn live_records<'a>(
    section: &NativeTopLevelField,
    records: &'a [NativeRecord],
    bytes: &[u8],
) -> Vec<&'a NativeRecord> {
    let end = section.offset + section.length;
    let mut inside: Vec<&NativeRecord> = records
        .iter()
        .filter(|r| r.offset >= section.offset && r.offset + r.length <= end)
        .collect();
    inside.sort_by_key(|r| (r.offset, std::cmp::Reverse(r.length)));
    let mut cursor = section.offset;
    let mut top = Vec::new();
    for record in inside {
        if record.offset >= cursor {
            cursor = record.offset + record.length;
            if flag(record, bytes, "m_alive") != Some(false) {
                top.push(record);
            }
        }
    }
    top
}

fn plural(count: usize, what: &str) -> String {
    format!("{count} {what}{}", if count == 1 { "" } else { "s" })
}

/// Unmodeled-source entries for the skipped sections of one native stream.
pub(super) fn collect(
    top_level: &[NativeTopLevelField],
    records: &[NativeRecord],
    bytes: &[u8],
    load_patterns: &[String],
) -> Vec<UnmodeledSourceData> {
    let section = |name: &str| {
        top_level
            .iter()
            .find(|t| t.name == name && t.wire_type == "struct[]")
    };
    let mut entries = Vec::new();
    for (name, what, affects) in SKIPPED_SECTIONS {
        let Some(span) = section(name) else { continue };
        let live = live_records(span, records, bytes);
        if live.is_empty() {
            continue;
        }
        match affects {
            Affects::AllCases | Affects::DynamicCases => entries.push(UnmodeledSourceData {
                scope: if *affects == Affects::AllCases {
                    UnmodeledScope::AllCases
                } else {
                    UnmodeledScope::DynamicCases
                },
                description: plural(live.len(), what),
            }),
            Affects::LoadPattern => {
                // `None` collects records whose pattern cannot be resolved.
                let mut by_pattern: BTreeMap<Option<&str>, usize> = BTreeMap::new();
                for record in live {
                    let pattern = integer(record, bytes, "m_lc")
                        .or_else(|| integer(record, bytes, "m_LC"))
                        .and_then(|index| usize::try_from(index).ok())
                        .and_then(|index| load_patterns.get(index))
                        .map(String::as_str)
                        .filter(|pattern| !pattern.is_empty());
                    *by_pattern.entry(pattern).or_default() += 1;
                }
                for (pattern, count) in by_pattern {
                    entries.push(match pattern {
                        Some(pattern) => UnmodeledSourceData {
                            scope: UnmodeledScope::LoadPattern(pattern.to_string()),
                            description: format!(
                                "{} in load pattern '{pattern}'",
                                plural(count, what)
                            ),
                        },
                        // An unscoped load must refuse everything.
                        None => UnmodeledSourceData {
                            scope: UnmodeledScope::AllCases,
                            description: format!(
                                "{} with an unresolved load pattern",
                                plural(count, what)
                            ),
                        },
                    });
                }
            }
        }
    }

    // Every corpus file has exactly one mass source: element masses plus
    // additional masses, nothing from loads. Anything else changes the mass.
    if let Some(span) = section("MassSource") {
        let sources = live_records(span, records, bytes);
        let default_only = sources.len() == 1
            && sources.iter().all(|source| {
                flag(source, bytes, "m_MassFromElements") == Some(true)
                    && flag(source, bytes, "m_MassFromMasses") == Some(true)
                    && flag(source, bytes, "m_MassFromLoads") == Some(false)
            });
        if !sources.is_empty() && !default_only {
            entries.push(UnmodeledSourceData {
                scope: UnmodeledScope::DynamicCases,
                description: "a mass source other than element masses plus additional masses"
                    .into(),
            });
        }
    }

    // Frame hinge assignments. The `FrameHingeAssign` section holds one record
    // per frame even when no hinge is assigned (non-empty in 144 corpus files),
    // so the hinge records themselves are counted. Hinges act in nonlinear cases.
    let hinges = records.iter().filter(|r| r.class_id == "S_Hinge").count();
    if hinges > 0 {
        entries.push(UnmodeledSourceData {
            scope: UnmodeledScope::NonlinearCases,
            description: format!("{hinges} frame hinge assignment(s)"),
        });
    }
    entries
}

/// Adds the entries for `sdb` to `model`. A stream that cannot be inventoried
/// is recorded as unknown, not as free of such data.
pub(super) fn record(sdb: &SdbFile, load_patterns: &[String], model: &mut SapModel) {
    match inspect_native_sdb(sdb) {
        Ok(document) => model.unmodeled_source.extend(collect(
            &document.inventory().top_level,
            document.records(),
            &sdb.decompressed_stream,
            load_patterns,
        )),
        Err(_) => model.unmodeled_source.push(UnmodeledSourceData {
            scope: UnmodeledScope::AllCases,
            description: "native sections outside the imported subset, which could not be inspected"
                .into(),
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::super::native::NativeFieldSpan;
    use super::*;

    /// Builds a stream of records, each a list of `(name, bytes)` fields, laid
    /// out back to back inside named top-level sections.
    struct Stream {
        bytes: Vec<u8>,
        top_level: Vec<NativeTopLevelField>,
        records: Vec<NativeRecord>,
    }

    impl Stream {
        fn new() -> Self {
            Self {
                bytes: Vec::new(),
                top_level: Vec::new(),
                records: Vec::new(),
            }
        }
        fn section(&mut self, name: &str, class: &str, records: &[Vec<(&str, Vec<u8>)>]) {
            let start = self.bytes.len();
            // Non-empty marker and count, as in a native struct array.
            self.bytes.push(1);
            self.bytes.extend((records.len() as i32).to_le_bytes());
            for fields in records {
                let offset = self.bytes.len();
                let mut spans = Vec::new();
                for (name, value) in fields {
                    spans.push(NativeFieldSpan {
                        name: Some((*name).into()),
                        wire_type: match value.len() {
                            1 => "bool",
                            4 => "i4",
                            _ => "r8",
                        }
                        .into(),
                        offset: self.bytes.len(),
                        length: value.len(),
                    });
                    self.bytes.extend(value);
                }
                self.records.push(NativeRecord {
                    class_id: class.into(),
                    offset,
                    length: self.bytes.len() - offset,
                    fields: spans,
                });
            }
            self.top_level.push(NativeTopLevelField {
                name: name.into(),
                wire_type: "struct[]".into(),
                offset: start,
                length: self.bytes.len() - start,
            });
        }
        fn entries(&self) -> Vec<(UnmodeledScope, String)> {
            let patterns = ["".to_string(), "DEAD".to_string(), "PT".to_string()];
            collect(&self.top_level, &self.records, &self.bytes, &patterns)
                .into_iter()
                .map(|e| (e.scope, e.description))
                .collect()
        }
    }

    fn alive(value: bool) -> (&'static str, Vec<u8>) {
        ("m_alive", vec![u8::from(value)])
    }
    fn int(name: &'static str, value: i32) -> (&'static str, Vec<u8>) {
        (name, value.to_le_bytes().to_vec())
    }
    fn bit(name: &'static str, value: bool) -> (&'static str, Vec<u8>) {
        (name, vec![u8::from(value)])
    }
    fn mass_source(elements: bool, masses: bool, loads: bool) -> Vec<(&'static str, Vec<u8>)> {
        vec![
            alive(true),
            bit("m_MassFromElements", elements),
            bit("m_MassFromMasses", masses),
            bit("m_MassFromLoads", loads),
        ]
    }

    #[test]
    fn live_records_are_counted_and_scoped_by_what_they_affect() {
        let mut s = Stream::new();
        // Two live line springs and one deleted one.
        s.section(
            "FSpring",
            "TYFSpring",
            &[
                vec![alive(true), int("m_n", 1)],
                vec![alive(false), int("m_n", 2)],
                vec![alive(true), int("m_n", 3)],
            ],
        );
        s.section("FMass", "TYFMass", &[vec![alive(true), int("m_n", 1)]]);
        // Loads in two patterns, one unresolved index, both field spellings.
        s.section(
            "PTLoad",
            "TYPTLoad",
            &[
                vec![alive(true), int("m_n", 1), int("m_lc", 2)],
                vec![alive(true), int("m_n", 2), int("m_lc", 2)],
                vec![alive(true), int("m_n", 3), int("m_lc", 1)],
                vec![alive(true), int("m_n", 4), int("m_lc", 9)],
                vec![alive(false), int("m_n", 5), int("m_lc", 1)],
            ],
        );
        s.section(
            "ShPPLoad",
            "TYPPLoad",
            &[vec![alive(true), int("m_n", 1), int("m_LC", 1)]],
        );
        // A section that is not in the table is not reported.
        s.section("Groups", "GROUPARRAY", &[vec![alive(true)]]);
        s.section("MassSource", "S_MassSource", &[mass_source(true, true, false)]);
        assert_eq!(
            s.entries(),
            [
                (UnmodeledScope::AllCases, "2 frame line-spring assignments".to_string()),
                (UnmodeledScope::DynamicCases, "1 frame added-mass assignment".to_string()),
                (
                    UnmodeledScope::AllCases,
                    "1 tendon prestress load with an unresolved load pattern".to_string()
                ),
                (
                    UnmodeledScope::LoadPattern("DEAD".into()),
                    "1 tendon prestress load in load pattern 'DEAD'".to_string()
                ),
                (
                    UnmodeledScope::LoadPattern("PT".into()),
                    "2 tendon prestress loads in load pattern 'PT'".to_string()
                ),
                (
                    UnmodeledScope::LoadPattern("DEAD".into()),
                    "1 area pore-pressure load in load pattern 'DEAD'".to_string()
                ),
            ]
        );
    }

    #[test]
    fn deleted_records_default_mass_source_and_empty_sections_record_nothing() {
        let mut s = Stream::new();
        s.section("ShSpring", "TYSSpring", &[vec![alive(false), int("m_n", 1)]]);
        s.section("FSpring", "TYFSpring", &[]);
        s.section("MassSource", "S_MassSource", &[mass_source(true, true, false)]);
        assert!(s.entries().is_empty());
    }

    #[test]
    fn non_default_mass_sources_and_hinge_records_are_reported() {
        for sources in [
            vec![mass_source(true, true, true)],
            vec![mass_source(false, true, false)],
            vec![mass_source(true, false, false)],
            vec![mass_source(true, true, false), mass_source(true, true, false)],
        ] {
            let mut s = Stream::new();
            s.section("MassSource", "S_MassSource", &sources);
            assert_eq!(
                s.entries(),
                [(
                    UnmodeledScope::DynamicCases,
                    "a mass source other than element masses plus additional masses".to_string()
                )]
            );
        }
        // One assignment record per frame; only the nested hinge records count.
        let mut s = Stream::new();
        s.section(
            "FrameHingeAssign",
            "S_FrameHingeAssign",
            &[vec![int("m_nhinge", 0)], vec![int("m_nhinge", 0)]],
        );
        assert!(s.entries().is_empty());
        s.section("Elsewhere", "S_Hinge", &[vec![int("m_iprop", 1)]]);
        assert_eq!(
            s.entries(),
            [(
                UnmodeledScope::NonlinearCases,
                "1 frame hinge assignment(s)".to_string()
            )]
        );
    }

    #[test]
    fn records_nested_inside_a_section_record_are_not_counted_separately() {
        let mut s = Stream::new();
        s.section("FSpring", "TYFSpring", &[vec![alive(true), int("m_n", 1), int("m_csys", 0)]]);
        // A nested record occupying the tail of the section's only record.
        let outer = s.records[0].clone();
        s.records.push(NativeRecord {
            class_id: "TYNested".into(),
            offset: outer.offset + 1,
            length: outer.length - 1,
            fields: Vec::new(),
        });
        assert_eq!(
            s.entries(),
            [(UnmodeledScope::AllCases, "1 frame line-spring assignment".to_string())]
        );
    }
}
