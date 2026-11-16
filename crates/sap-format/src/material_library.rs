//! Materials read from the vendor's material library files
//! (`CSiMaterialLibrary*.xml`), which are looked up at run time like the
//! `.pro` section files. Nothing from those files ships with the program.
//!
//! # Files
//!
//! The SAP2000 help topic "Define Materials" says that the program reads the
//! files named `CSiMaterialLibrary*.xml` of its folder in alphabetical order;
//! when two files define the same material, the later one wins. A material is
//! identified by (region, standard, type, grade), the type without regard to
//! case (the vendor files write both `Tendon` and `tendon`).
//!
//! [`read_material_library_dir`] follows that rule. It accepts one more kind of
//! file: a file whose name starts with `csimateriallibrary` and has **no
//! extension at all**. The reason is practical: unpacking the vendor's
//! installer truncates long file names and drops their extension, which leaves
//! `csimateriallibraryunitedstat` and `csimateriallibrarynewzealand`. The
//! schema (`.xsd`) and every other extension are never read.
//! [`read_material_library_file`] reads one file by its path, whatever its name.
//!
//! # Units
//!
//! The `control` element gives the units of the file; a region, collection,
//! standard or material element may override them for its own block. A
//! (force, length) pair that has no [`Units`] variant (for example Kip and mm)
//! is refused with the file and line of the element that names it.
//!
//! # The XML reader
//!
//! The reader is a hand-written subset reader, not a general XML parser. It
//! supports the XML declaration and other processing instructions, comments,
//! attributes in either quote style, self-closing tags, the five named
//! entities and numeric character references, and UTF-8 text. Every namespace
//! attribute is read and ignored. It refuses, with a message naming the line:
//! a DTD or any other `<!` declaration, CDATA sections, element names with a
//! prefix, unknown entities, mismatched or missing end tags, text outside the
//! root element, more than one root, and unquoted or duplicate attributes. It
//! never panics on malformed input.
//!
//! # What is kept
//!
//! Weight density, mass density, E, nu, alpha, the minimum yield and tensile
//! stress, the effective ("expected") yield and tensile stress, f'c, expected
//! f'c, the lightweight flag and the shear strength reduction factor. Strains
//! and the aluminum and tendon curve data are read over and not kept.

use sap_model::{MatType, Material};
use sap_units::{ForceUnit, LengthUnit, Units};
use std::fmt;
use std::path::{Path, PathBuf};

/// Why a library could not be read. Line numbers start at 1.
#[derive(Debug, Clone, PartialEq)]
pub enum MaterialLibraryError {
    Io {
        path: String,
        reason: String,
    },
    /// The text is not in the supported XML subset.
    Syntax {
        source: String,
        line: usize,
        message: String,
    },
    /// The XML is readable but does not describe a library we can use.
    Invalid {
        source: String,
        line: usize,
        message: String,
    },
    /// A folder without any library file.
    NoFiles {
        dir: String,
    },
}

impl fmt::Display for MaterialLibraryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io { path, reason } => write!(f, "cannot read {path}: {reason}"),
            Self::Syntax {
                source,
                line,
                message,
            }
            | Self::Invalid {
                source,
                line,
                message,
            } => write!(f, "{source}: line {line}: {message}"),
            Self::NoFiles { dir } => write!(
                f,
                "no material library file (CSiMaterialLibrary*.xml) in {dir}"
            ),
        }
    }
}

impl std::error::Error for MaterialLibraryError {}

/// The numbers of one library material, in the units of its entry.
/// `None` means the file does not give the value.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct LibraryValues {
    pub weight_density: Option<f64>,
    pub mass_density: Option<f64>,
    pub e: Option<f64>,
    pub nu: Option<f64>,
    pub alpha: Option<f64>,
    /// Minimum yield stress.
    pub fy: Option<f64>,
    /// Minimum tensile stress.
    pub fu: Option<f64>,
    /// Effective yield stress (not kept in a model).
    pub fye: Option<f64>,
    /// Effective tensile stress (not kept in a model).
    pub fue: Option<f64>,
    /// Specified compressive strength.
    pub fc: Option<f64>,
    /// Expected compressive strength (not kept in a model).
    pub fce: Option<f64>,
    pub lightweight: bool,
    pub shear_reduction: Option<f64>,
}

/// One material of a library file.
#[derive(Debug, Clone, PartialEq)]
pub struct LibraryEntry {
    pub region: String,
    pub standard: String,
    /// The `type` attribute as written in the file.
    pub type_label: String,
    /// The model type of `type_label`; `None` for a type this program does not know.
    pub mat_type: Option<MatType>,
    pub grade: String,
    pub name: String,
    pub is_default: bool,
    /// The units of `values` (control, region, collection, standard or material).
    pub units: Units,
    /// The file the entry came from (its name as given).
    pub source: String,
    /// The line of the `material` element in `source`.
    pub line: usize,
    pub values: LibraryValues,
}

/// The type of a `type` attribute without regard to case.
pub fn fold_type(label: &str) -> String {
    label.trim().to_lowercase()
}

/// The model type of a library `type` attribute, if there is one.
pub fn mat_type_of(label: &str) -> Option<MatType> {
    match fold_type(label).as_str() {
        "steel" => Some(MatType::Steel),
        "concrete" => Some(MatType::Concrete),
        "rebar" => Some(MatType::Rebar),
        "tendon" => Some(MatType::Tendon),
        "aluminum" => Some(MatType::Aluminum),
        "coldformed" => Some(MatType::ColdFormed),
        _ => None,
    }
}

impl LibraryEntry {
    fn key_matches(&self, other: &LibraryEntry) -> bool {
        self.region == other.region
            && self.standard == other.standard
            && self.grade == other.grade
            && fold_type(&self.type_label) == fold_type(&other.type_label)
    }
}

/// All entries of the files read, without duplicate keys.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MaterialLibrary {
    pub entries: Vec<LibraryEntry>,
}

impl MaterialLibrary {
    /// Adds `entries`; an entry with the key of an earlier one replaces it
    /// (in the earlier one's position), as the later file wins.
    pub fn merge(&mut self, entries: Vec<LibraryEntry>) {
        for entry in entries {
            match self.entries.iter().position(|e| e.key_matches(&entry)) {
                Some(i) => self.entries[i] = entry,
                None => self.entries.push(entry),
            }
        }
    }

    /// The regions, sorted, without repeats.
    pub fn regions(&self) -> Vec<String> {
        let mut regions: Vec<String> = self.entries.iter().map(|e| e.region.clone()).collect();
        regions.sort();
        regions.dedup();
        regions
    }

    /// The type labels of a region (as the first entry of each type wrote them), in file order.
    pub fn type_labels(&self, region: &str) -> Vec<String> {
        let mut out: Vec<String> = Vec::new();
        for e in self.entries.iter().filter(|e| e.region == region) {
            if !out.iter().any(|t| fold_type(t) == fold_type(&e.type_label)) {
                out.push(e.type_label.clone());
            }
        }
        out
    }

    /// The standards of a region that have materials of `type_label`, in file order.
    pub fn standards(&self, region: &str, type_label: &str) -> Vec<String> {
        let mut out: Vec<String> = Vec::new();
        for e in self
            .entries
            .iter()
            .filter(|e| e.region == region && fold_type(&e.type_label) == fold_type(type_label))
        {
            if !out.contains(&e.standard) {
                out.push(e.standard.clone());
            }
        }
        out
    }

    /// The entries of one region, type and standard, in file order.
    pub fn grades(&self, region: &str, type_label: &str, standard: &str) -> Vec<&LibraryEntry> {
        self.entries
            .iter()
            .filter(|e| {
                e.region == region
                    && e.standard == standard
                    && fold_type(&e.type_label) == fold_type(type_label)
            })
            .collect()
    }

    /// The entry with this key; the type is compared without regard to case.
    pub fn find(
        &self,
        region: &str,
        type_label: &str,
        standard: &str,
        grade: &str,
    ) -> Option<&LibraryEntry> {
        self.entries.iter().find(|e| {
            e.region == region
                && e.standard == standard
                && e.grade == grade
                && fold_type(&e.type_label) == fold_type(type_label)
        })
    }
}

// ---------------------------------------------------------------------------
// Units

fn force_unit_of(name: &str) -> Option<ForceUnit> {
    match name.trim().to_lowercase().as_str() {
        "lb" => Some(ForceUnit::Lb),
        "kip" => Some(ForceUnit::Kip),
        "n" => Some(ForceUnit::N),
        "kn" => Some(ForceUnit::Kn),
        "kgf" => Some(ForceUnit::Kgf),
        "tonf" => Some(ForceUnit::Tonf),
        _ => None,
    }
}

fn length_unit_of(name: &str) -> Option<LengthUnit> {
    match name.trim().to_lowercase().as_str() {
        "in" => Some(LengthUnit::In),
        "ft" => Some(LengthUnit::Ft),
        "mm" => Some(LengthUnit::Mm),
        "cm" => Some(LengthUnit::Cm),
        "m" => Some(LengthUnit::M),
        _ => None,
    }
}

/// The [`Units`] variant with these force and length units, if one exists.
pub fn units_for(force: ForceUnit, length: LengthUnit) -> Option<Units> {
    (1..=16)
        .filter_map(|code| Units::from_code(code).ok())
        .find(|u| u.force_unit() == force && u.length_unit() == length)
}

// ---------------------------------------------------------------------------
// The XML subset reader

#[derive(Debug)]
struct Element {
    name: String,
    attrs: Vec<(String, String)>,
    text: String,
    children: Vec<Element>,
    line: usize,
}

impl Element {
    fn attr(&self, name: &str) -> Option<&str> {
        self.attrs
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, v)| v.as_str())
    }
}

const MAX_DEPTH: usize = 64;

struct XmlReader<'a> {
    text: &'a str,
    pos: usize,
    line: usize,
    source: &'a str,
}

impl<'a> XmlReader<'a> {
    fn err<T>(&self, line: usize, message: impl Into<String>) -> Result<T, MaterialLibraryError> {
        Err(MaterialLibraryError::Syntax {
            source: self.source.to_string(),
            line,
            message: message.into(),
        })
    }

    fn rest(&self) -> &'a str {
        &self.text[self.pos..]
    }

    /// Moves to byte offset `to` (a character boundary), counting lines.
    fn advance_to(&mut self, to: usize) {
        let to = to.min(self.text.len());
        self.line += self.text[self.pos..to].matches('\n').count();
        self.pos = to;
    }

    fn skip_ws(&mut self) {
        let n = self
            .rest()
            .find(|c: char| !c.is_whitespace())
            .unwrap_or(self.rest().len());
        self.advance_to(self.pos + n);
    }

    fn name(&mut self, allow_colon: bool) -> Result<String, MaterialLibraryError> {
        let rest = self.rest();
        let end = rest
            .find(|c: char| c.is_whitespace() || matches!(c, '<' | '>' | '/' | '=' | '"' | '\''))
            .unwrap_or(rest.len());
        let name = &rest[..end];
        let first = name.chars().next();
        if !first.is_some_and(|c| c.is_alphabetic() || c == '_') {
            let shown: String = rest.chars().take(12).collect();
            return self.err(self.line, format!("expected a name, found '{shown}'"));
        }
        if !allow_colon && name.contains(':') {
            return self.err(
                self.line,
                format!("prefixed element name '{name}' is not supported"),
            );
        }
        let name = name.to_string();
        self.advance_to(self.pos + end);
        Ok(name)
    }

    /// Replaces entity references in `raw`, which starts on line `line`.
    fn decode(&self, raw: &str, line: usize) -> Result<String, MaterialLibraryError> {
        if !raw.contains('&') {
            return Ok(raw.to_string());
        }
        let mut out = String::with_capacity(raw.len());
        let mut rest = raw;
        let mut consumed = 0usize;
        while let Some(i) = rest.find('&') {
            out.push_str(&rest[..i]);
            let here = line + raw[..consumed + i].matches('\n').count();
            let after = &rest[i + 1..];
            let Some(semi) = after.find(';').filter(|&s| s <= 10) else {
                return self.err(here, "unterminated entity reference");
            };
            let body = &after[..semi];
            let ch = match body {
                "lt" => Some('<'),
                "gt" => Some('>'),
                "amp" => Some('&'),
                "quot" => Some('"'),
                "apos" => Some('\''),
                _ => {
                    let code = body
                        .strip_prefix("#x")
                        .or_else(|| body.strip_prefix("#X"))
                        .and_then(|h| u32::from_str_radix(h, 16).ok())
                        .or_else(|| {
                            body.strip_prefix('#')
                                .filter(|d| !d.is_empty() && d.bytes().all(|b| b.is_ascii_digit()))
                                .and_then(|d| d.parse::<u32>().ok())
                        });
                    code.and_then(char::from_u32)
                }
            };
            match ch {
                Some(c) => out.push(c),
                None => return self.err(here, format!("unknown entity '&{body};'")),
            }
            consumed += i + 1 + semi + 1;
            rest = &after[semi + 1..];
        }
        out.push_str(rest);
        Ok(out)
    }

    fn attribute(&mut self) -> Result<(String, String), MaterialLibraryError> {
        let name = self.name(true)?;
        self.skip_ws();
        if !self.rest().starts_with('=') {
            return self.err(self.line, format!("attribute '{name}' has no value"));
        }
        self.advance_to(self.pos + 1);
        self.skip_ws();
        let quote = match self.rest().chars().next() {
            Some(q @ ('"' | '\'')) => q,
            _ => {
                return self.err(
                    self.line,
                    format!("the value of attribute '{name}' is not quoted"),
                )
            }
        };
        let start_line = self.line;
        let body = &self.rest()[1..];
        let Some(end) = body.find(quote) else {
            return self.err(
                start_line,
                format!("the value of attribute '{name}' is not closed"),
            );
        };
        let raw = &body[..end];
        if raw.contains('<') {
            return self.err(
                start_line,
                format!("'<' is not allowed in the value of attribute '{name}'"),
            );
        }
        let value = self.decode(raw, start_line)?;
        self.advance_to(self.pos + 1 + end + 1);
        Ok((name, value))
    }

    fn parse(mut self) -> Result<Element, MaterialLibraryError> {
        if self.text.starts_with('\u{feff}') {
            self.pos = '\u{feff}'.len_utf8();
        }
        let mut stack: Vec<Element> = Vec::new();
        let mut root: Option<Element> = None;
        while self.pos < self.text.len() {
            if !self.rest().starts_with('<') {
                let n = self.rest().find('<').unwrap_or(self.rest().len());
                let raw = &self.rest()[..n];
                let line = self.line;
                if stack.is_empty() {
                    if !raw.trim().is_empty() {
                        let skipped = raw.len() - raw.trim_start().len();
                        let at = line + raw[..skipped].matches('\n').count();
                        return self.err(at, "text outside the root element");
                    }
                } else {
                    let decoded = self.decode(raw, line)?;
                    if let Some(top) = stack.last_mut() {
                        top.text.push_str(&decoded);
                    }
                }
                self.advance_to(self.pos + n);
                continue;
            }
            let rest = self.rest();
            if rest.starts_with("<?") {
                let Some(end) = rest.find("?>") else {
                    return self.err(self.line, "processing instruction is not closed");
                };
                self.advance_to(self.pos + end + 2);
            } else if let Some(body) = rest.strip_prefix("<!--") {
                let Some(end) = body.find("-->") else {
                    return self.err(self.line, "comment is not closed");
                };
                self.advance_to(self.pos + 4 + end + 3);
            } else if rest.starts_with("<![CDATA[") {
                return self.err(self.line, "CDATA sections are not supported");
            } else if rest.starts_with("<!") {
                return self.err(
                    self.line,
                    "DTDs and other '<!' declarations are not supported",
                );
            } else if rest.starts_with("</") {
                let line = self.line;
                self.advance_to(self.pos + 2);
                let name = self.name(true)?;
                self.skip_ws();
                if !self.rest().starts_with('>') {
                    return self.err(self.line, format!("end tag </{name}> is not closed"));
                }
                self.advance_to(self.pos + 1);
                let Some(done) = stack.pop() else {
                    return self.err(line, format!("end tag </{name}> has no start tag"));
                };
                if done.name != name {
                    return self.err(
                        line,
                        format!(
                            "end tag </{name}> does not match <{}> opened at line {}",
                            done.name, done.line
                        ),
                    );
                }
                match stack.last_mut() {
                    Some(parent) => parent.children.push(done),
                    None => root = Some(done),
                }
            } else {
                let line = self.line;
                if stack.is_empty() && root.is_some() {
                    return self.err(line, "more than one root element");
                }
                self.advance_to(self.pos + 1);
                let name = self.name(false)?;
                let mut attrs: Vec<(String, String)> = Vec::new();
                let self_closing;
                loop {
                    self.skip_ws();
                    if self.rest().starts_with("/>") {
                        self.advance_to(self.pos + 2);
                        self_closing = true;
                        break;
                    }
                    if self.rest().starts_with('>') {
                        self.advance_to(self.pos + 1);
                        self_closing = false;
                        break;
                    }
                    if self.pos >= self.text.len() {
                        return self.err(line, format!("start tag <{name}> is not closed"));
                    }
                    let attr_line = self.line;
                    let (an, av) = self.attribute()?;
                    if attrs.iter().any(|(n, _)| *n == an) {
                        return self.err(attr_line, format!("duplicate attribute '{an}'"));
                    }
                    attrs.push((an, av));
                }
                let element = Element {
                    name,
                    attrs,
                    text: String::new(),
                    children: Vec::new(),
                    line,
                };
                if self_closing {
                    match stack.last_mut() {
                        Some(parent) => parent.children.push(element),
                        None => root = Some(element),
                    }
                } else {
                    if stack.len() >= MAX_DEPTH {
                        return self.err(line, "elements are nested too deeply");
                    }
                    stack.push(element);
                }
            }
        }
        if let Some(open) = stack.last() {
            return self.err(
                self.line,
                format!(
                    "unexpected end of the file: <{}> opened at line {} is not closed",
                    open.name, open.line
                ),
            );
        }
        match root {
            Some(root) => Ok(root),
            None => self.err(self.line, "the file has no root element"),
        }
    }
}

// ---------------------------------------------------------------------------
// From the element tree to entries

#[derive(Clone, Copy, Default)]
struct UnitScope {
    force: Option<ForceUnit>,
    length: Option<LengthUnit>,
}

struct Interpreter<'a> {
    source: &'a str,
}

impl Interpreter<'_> {
    fn err<T>(&self, line: usize, message: impl Into<String>) -> Result<T, MaterialLibraryError> {
        Err(MaterialLibraryError::Invalid {
            source: self.source.to_string(),
            line,
            message: message.into(),
        })
    }

    /// The units of `el` given those of its container; the pair is checked
    /// whenever the element names a unit.
    fn scope(&self, el: &Element, outer: UnitScope) -> Result<UnitScope, MaterialLibraryError> {
        let mut scope = outer;
        let force = el.attr("forceUnits");
        let length = el.attr("lengthUnits");
        if let Some(f) = force {
            match force_unit_of(f) {
                Some(u) => scope.force = Some(u),
                None => return self.err(el.line, format!("unknown force unit '{f}'")),
            }
        }
        if let Some(l) = length {
            match length_unit_of(l) {
                Some(u) => scope.length = Some(u),
                None => return self.err(el.line, format!("unknown length unit '{l}'")),
            }
        }
        if force.is_some() || length.is_some() {
            if let (Some(f), Some(l)) = (scope.force, scope.length) {
                if units_for(f, l).is_none() {
                    return self.err(
                        el.line,
                        format!(
                            "force unit {} with length unit {} is not a SAP2000 unit \
                             system; the supported pairs are those of the unit list",
                            f.symbol(),
                            l.symbol()
                        ),
                    );
                }
            }
        }
        Ok(scope)
    }

    fn required<'e>(&self, el: &'e Element, attr: &str) -> Result<&'e str, MaterialLibraryError> {
        match el.attr(attr) {
            Some(v) if !v.trim().is_empty() => Ok(v),
            _ => self.err(el.line, format!("<{}> has no '{attr}' attribute", el.name)),
        }
    }

    fn number(&self, el: &Element) -> Result<f64, MaterialLibraryError> {
        match el.text.trim().parse::<f64>() {
            Ok(v) if v.is_finite() => Ok(v),
            _ => self.err(
                el.line,
                format!("<{}>: '{}' is not a finite number", el.name, el.text.trim()),
            ),
        }
    }

    fn boolean(&self, el: &Element) -> Result<bool, MaterialLibraryError> {
        match el.text.trim().to_lowercase().as_str() {
            "true" | "1" => Ok(true),
            "false" | "0" => Ok(false),
            other => self.err(
                el.line,
                format!("<{}>: '{other}' is not true or false", el.name),
            ),
        }
    }

    fn values(&self, material: &Element) -> Result<LibraryValues, MaterialLibraryError> {
        let mut v = LibraryValues::default();
        let mut seen: Vec<&str> = Vec::new();
        for child in &material.children {
            let slot: Option<&mut Option<f64>> = match child.name.as_str() {
                "weightDensity" => Some(&mut v.weight_density),
                "massDensity" => Some(&mut v.mass_density),
                "modulusOfElasticity" => Some(&mut v.e),
                "poissonsRatio" => Some(&mut v.nu),
                "coefficientOfThermalExpansion" => Some(&mut v.alpha),
                "minimumYieldStress" => Some(&mut v.fy),
                "minimumTensileStress" => Some(&mut v.fu),
                "effectiveYieldStress" => Some(&mut v.fye),
                "effectiveTensileStress" => Some(&mut v.fue),
                "compressiveStrength" => Some(&mut v.fc),
                "expectedCompressiveStrength" => Some(&mut v.fce),
                "shearStrengthReductionFactor" => Some(&mut v.shear_reduction),
                _ => None,
            };
            let known = slot.is_some() || child.name == "isLightWeight";
            if known {
                if seen.contains(&child.name.as_str()) {
                    return self.err(
                        child.line,
                        format!("<{}> appears twice in one material", child.name),
                    );
                }
                seen.push(&child.name);
            }
            if let Some(slot) = slot {
                *slot = Some(self.number(child)?);
            } else if child.name == "isLightWeight" {
                v.lightweight = self.boolean(child)?;
            }
        }
        Ok(v)
    }

    fn entries(&self, root: &Element) -> Result<Vec<LibraryEntry>, MaterialLibraryError> {
        if root.name != "propertyFile" {
            return self.err(
                root.line,
                format!("the root element is <{}>, not <propertyFile>", root.name),
            );
        }
        let Some(control) = root.children.iter().find(|c| c.name == "control") else {
            return self.err(root.line, "the file has no <control> element");
        };
        let file_scope = self.scope(control, UnitScope::default())?;
        let mut out = Vec::new();
        for region in root.children.iter().filter(|c| c.name != "control") {
            if region.name != "region" {
                return self.err(region.line, format!("unexpected <{}>", region.name));
            }
            let region_name = self.required(region, "name")?.to_string();
            let region_scope = self.scope(region, file_scope)?;
            for collection in &region.children {
                if collection.name != "collection" {
                    return self.err(collection.line, format!("unexpected <{}>", collection.name));
                }
                let collection_scope = self.scope(collection, region_scope)?;
                for standard in &collection.children {
                    if standard.name != "standard" {
                        return self.err(standard.line, format!("unexpected <{}>", standard.name));
                    }
                    let standard_name = self.required(standard, "name")?.to_string();
                    let standard_scope = self.scope(standard, collection_scope)?;
                    for material in &standard.children {
                        if material.name != "material" {
                            return self
                                .err(material.line, format!("unexpected <{}>", material.name));
                        }
                        let scope = self.scope(material, standard_scope)?;
                        let (Some(force), Some(length)) = (scope.force, scope.length) else {
                            return self.err(
                                material.line,
                                "the material has no force and length units (control, region, \
                                 collection, standard or material)",
                            );
                        };
                        let Some(units) = units_for(force, length) else {
                            return self.err(material.line, "unsupported force and length units");
                        };
                        let type_label = self.required(material, "type")?.to_string();
                        out.push(LibraryEntry {
                            region: region_name.clone(),
                            standard: standard_name.clone(),
                            mat_type: mat_type_of(&type_label),
                            type_label,
                            grade: self.required(material, "grade")?.to_string(),
                            name: self.required(material, "name")?.to_string(),
                            is_default: material
                                .attr("isDefault")
                                .is_some_and(|v| v.trim().eq_ignore_ascii_case("true")),
                            units,
                            source: self.source.to_string(),
                            line: material.line,
                            values: self.values(material)?,
                        });
                    }
                }
            }
        }
        Ok(out)
    }
}

/// Parses the text of one library file. `source` names the file in errors and
/// in each entry. A file without materials gives an empty list.
pub fn parse_material_library(
    text: &str,
    source: &str,
) -> Result<Vec<LibraryEntry>, MaterialLibraryError> {
    let root = XmlReader {
        text,
        pos: 0,
        line: 1,
        source,
    }
    .parse()?;
    Interpreter { source }.entries(&root)
}

/// Reads one library file by its path, whatever its name.
pub fn read_material_library_file(path: &Path) -> Result<Vec<LibraryEntry>, MaterialLibraryError> {
    let shown = path
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_else(|| path.display().to_string());
    let bytes = std::fs::read(path).map_err(|e| MaterialLibraryError::Io {
        path: path.display().to_string(),
        reason: e.to_string(),
    })?;
    let text = String::from_utf8(bytes).map_err(|e| {
        let valid = e.utf8_error().valid_up_to();
        let line = 1 + e.as_bytes()[..valid]
            .iter()
            .filter(|&&b| b == b'\n')
            .count();
        MaterialLibraryError::Syntax {
            source: shown.clone(),
            line,
            message: "the file is not valid UTF-8".into(),
        }
    })?;
    parse_material_library(&text, &shown)
}

/// Whether a file name is read by [`read_material_library_dir`]: the vendor
/// pattern `CSiMaterialLibrary*.xml` without regard to case, or a name that
/// starts with `csimateriallibrary` and has no extension at all (extracting
/// the vendor's installer truncates long names and drops the extension, which
/// leaves `csimateriallibraryunitedstat`). Other extensions, `.xsd` included,
/// never qualify.
pub fn is_library_file_name(name: &str) -> bool {
    let lower = name.to_lowercase();
    lower.starts_with("csimateriallibrary") && (lower.ends_with(".xml") || !lower.contains('.'))
}

/// The library files of `dir`, in alphabetical order without regard to case.
pub fn library_files_in(dir: &Path) -> Result<Vec<PathBuf>, MaterialLibraryError> {
    let io = |e: std::io::Error| MaterialLibraryError::Io {
        path: dir.display().to_string(),
        reason: e.to_string(),
    };
    let mut files: Vec<(String, PathBuf)> = Vec::new();
    for entry in std::fs::read_dir(dir).map_err(io)? {
        let entry = entry.map_err(io)?;
        let name = entry.file_name().to_string_lossy().into_owned();
        let path = entry.path();
        if is_library_file_name(&name) && path.is_file() {
            files.push((name, path));
        }
    }
    files.sort_by(|a, b| {
        a.0.to_lowercase()
            .cmp(&b.0.to_lowercase())
            .then_with(|| a.0.cmp(&b.0))
    });
    Ok(files.into_iter().map(|(_, p)| p).collect())
}

/// Reads every library file of `dir` (see [`is_library_file_name`]) in
/// alphabetical order; a later entry with the key (region, standard, type
/// without regard to case, grade) of an earlier one replaces it. A file that
/// cannot be read is an error naming the file and line, never skipped.
pub fn read_material_library_dir(dir: &Path) -> Result<MaterialLibrary, MaterialLibraryError> {
    let files = library_files_in(dir)?;
    if files.is_empty() {
        return Err(MaterialLibraryError::NoFiles {
            dir: dir.display().to_string(),
        });
    }
    let mut library = MaterialLibrary::default();
    for file in files {
        library.merge(read_material_library_file(&file)?);
    }
    Ok(library)
}

// ---------------------------------------------------------------------------
// Conversion

/// A material of the library as a model material in `units`.
///
/// E, nu, alpha, unit weight and mass density come from the file, each
/// converted with its own factor of `entry.units`; weight and mass density are
/// never derived from each other. G = E / (2 (1 + nu)). Steel, rebar and
/// tendon take Fy and Fu from the minimum yield and tensile stress; concrete
/// takes f'c and has Fy = Fys = 0. The effective strengths, the expected f'c
/// and the strains are not kept (the model has no field for them).
///
/// Refused, naming the material: aluminum, cold-formed and unknown types;
/// lightweight concrete and a shear strength reduction factor other than 1
/// (the concrete design would ignore them); a missing E, nu, alpha, unit
/// weight, mass density or strength; an invalid E or nu; an empty `name`.
pub fn library_material(
    entry: &LibraryEntry,
    units: Units,
    name: &str,
) -> Result<Material, String> {
    let who = format!(
        "library material '{}' ({} / {} / {} / {})",
        entry.name, entry.region, entry.standard, entry.type_label, entry.grade
    );
    if name.trim().is_empty() || name.chars().any(char::is_control) {
        return Err("The material name must be nonempty and contain no control characters.".into());
    }
    let mat_type = match entry.mat_type {
        Some(t @ (MatType::Steel | MatType::Concrete | MatType::Rebar | MatType::Tendon)) => t,
        _ => {
            return Err(format!(
                "{who}: the type '{}' is not supported; only steel, concrete, rebar and tendon \
                 are",
                entry.type_label
            ))
        }
    };
    let v = &entry.values;
    let need = |value: Option<f64>, what: &str| -> Result<f64, String> {
        value.ok_or_else(|| format!("{who}: the file gives no {what}"))
    };
    if mat_type == MatType::Concrete {
        if v.lightweight {
            return Err(format!(
                "{who}: lightweight concrete is not supported (the concrete design ignores it)"
            ));
        }
        if let Some(r) = v.shear_reduction {
            if r != 1.0 {
                return Err(format!(
                    "{who}: a shear strength reduction factor of {r} is not supported \
                     (only 1 is)"
                ));
            }
        }
    }
    let e = need(v.e, "modulus of elasticity")?;
    let nu = need(v.nu, "Poisson's ratio")?;
    let alpha = need(v.alpha, "coefficient of thermal expansion")?;
    let weight = need(v.weight_density, "weight density")?;
    let mass = need(v.mass_density, "mass density")?;
    if e <= 0.0 || !(-1.0 < nu && nu < 0.5) {
        return Err(format!(
            "{who}: E = {e} and Poisson's ratio = {nu} are not valid"
        ));
    }
    if weight < 0.0 || mass < 0.0 {
        return Err(format!("{who}: a density is negative"));
    }
    let (fy, fu, fc) = if mat_type == MatType::Concrete {
        let fc = need(v.fc, "compressive strength")?;
        if fc <= 0.0 {
            return Err(format!("{who}: the compressive strength is not positive"));
        }
        (0.0, 0.0, fc)
    } else {
        let fy = need(v.fy, "minimum yield stress")?;
        let fu = need(v.fu, "minimum tensile stress")?;
        if fy <= 0.0 || fu < fy {
            return Err(format!("{who}: requires Fu >= Fy > 0"));
        }
        (fy, fu, 0.0)
    };
    let stress = entry.units.stress_factor_to(units);
    let material = Material {
        name: name.to_string(),
        mat_type,
        e: e * stress,
        e_ortho: None,
        u: nu,
        u_ortho: None,
        g: e * stress / (2.0 * (1.0 + nu)),
        g_ortho: None,
        a: alpha * entry.units.thermal_expansion_factor_to(units),
        a_ortho: None,
        unit_weight: weight * entry.units.unit_weight_factor_to(units),
        unit_mass: mass * entry.units.mass_density_factor_to(units),
        fy: fy * stress,
        fu: fu * stress,
        fc: fc * stress,
        // The model's convention for a material without a separate shear
        // reinforcement grade (the built-in steel, and native import): equal
        // to the yield stress. Zero for concrete, which has none here.
        fys: fy * stress,
        color: if mat_type == MatType::Concrete {
            "Green".to_string()
        } else {
            "Blue".to_string()
        },
    };
    let finite = [
        material.e,
        material.g,
        material.a,
        material.unit_weight,
        material.unit_mass,
        material.fy,
        material.fu,
        material.fc,
    ]
    .iter()
    .all(|x| x.is_finite());
    if !finite {
        return Err(format!("{who}: a converted value is not finite"));
    }
    Ok(material)
}
