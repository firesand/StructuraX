use std::collections::HashMap;
use std::io::{Read, Write};
use std::path::Path;
use std::sync::OnceLock;

use flate2::read::GzDecoder;
use flate2::write::GzEncoder;
use flate2::Compression;

use sap_model::{
    AreaLoadSurfacePressure, AreaLoadTemperature, AreaLoadUniform, AreaObj, AreaPropType,
    AreaSection, ComboType, ConstraintAxis, ConstraintDef, ConstraintType, DynamicFunction,
    FrameLoadDistributed, FrameLoadPoint, FrameLoadTemperature, FrameObj, FramePropType,
    FrameSection, LinkObj, LinkProp, LinkPropType, LoadCase, LoadCaseAssignment, LoadCaseType,
    LoadCombo, LoadPattern, LoadPatternType, MatType, Material, ModalComboMethod, ModalSubType,
    NLGeomType, NonPrismaticSegment, PlaneType, PointLoadDispl, PointLoadForce, PointObj, SapModel,
    ShellType, SolidObj, SolidSection, SolidSurfaceLoad,
};
use sap_units::Units;

#[cfg(test)]
use crate::s2k::write_s2k;
use crate::s2k::{parse_s2k, try_write_s2k_with_report};
use crate::FormatError;
use sap_model::{UnmodeledScope, UnmodeledSourceData};

pub mod native;
mod skipped;
pub use native::{pack_model_to_native_sdb, save_native_sdb_file};

// Native materials store Windows COLORREF integers. Keep the common canonical
// names and preserve every other integer exactly rather than guessing a name.
fn native_material_color(value: i32) -> String {
    match value {
        0x000000 => "Black".into(),
        0x0000ff => "Red".into(),
        0x00ff00 => "Green".into(),
        0xff0000 => "Blue".into(),
        0x00ffff => "Yellow".into(),
        0xff00ff => "Magenta".into(),
        0xffff00 => "Cyan".into(),
        0xffffff => "White".into(),
        0x808080 => "Gray".into(),
        _ => format!("ColorRef:{value}"),
    }
}

fn material_colorref(value: &str) -> Option<i32> {
    match value {
        "Black" => Some(0x000000),
        "Red" => Some(0x0000ff),
        "Green" => Some(0x00ff00),
        "Blue" => Some(0xff0000),
        "Yellow" => Some(0x00ffff),
        "Magenta" => Some(0xff00ff),
        "Cyan" => Some(0xffff00),
        "White" => Some(0xffffff),
        "Gray" => Some(0x808080),
        _ => value.strip_prefix("ColorRef:")?.parse().ok(),
    }
}

const SDB_SCHEMAS_TXT: &str = include_str!("sdb_schemas.txt");

/// Maximum allowed decompressed `.sdb` stream size (128 MiB) to prevent decompression bombs.
const MAX_DECOMPRESSED_SDB_BYTES: u64 = 128 * 1024 * 1024;

/// Maximum struct nesting depth in `cBinaryStream` to prevent stack overflow on corrupt inputs.
const MAX_RECURSION_DEPTH: usize = 64;

/// Maximum number of elements in any single `cBinaryStream` array.
const MAX_ARRAY_LEN: usize = 1_000_000;

/// Minimal 1x1 transparent PNG used as default thumbnail when writing new `.sdb` files.
const DEFAULT_THUMBNAIL_PNG: &[u8] = &[
    0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A, 0x00, 0x00, 0x00, 0x0D, 0x49, 0x48, 0x44, 0x52,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x08, 0x06, 0x00, 0x00, 0x00, 0x1F, 0x15, 0xC4,
    0x89, 0x00, 0x00, 0x00, 0x0A, 0x49, 0x44, 0x41, 0x54, 0x78, 0x9C, 0x63, 0x00, 0x01, 0x00, 0x00,
    0x05, 0x00, 0x01, 0x0D, 0x0A, 0x2D, 0xB4, 0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4E, 0x44, 0xAE,
    0x42, 0x60, 0x82,
];

/// Magic prefix inside `decompressed_stream` when serialized from `SapModel` by `pack_model_to_sdb`.
pub(crate) const RUST_SDB_S2K_MAGIC: &[u8] = b"SAP2000_S2K_STREAM\n";

/// Parsed outer container of a SAP2000 v19 `.sdb` file.
#[derive(Debug, Clone)]
pub struct SdbFile {
    pub program_name: String,
    pub program_version: String,
    pub file_version: String,
    pub thumbnail_png: Vec<u8>,
    pub decompressed_stream: Vec<u8>,
}

/// Reads a SAP2000 `.sdb` container from raw file bytes.
pub fn read_sdb_bytes(bytes: &[u8]) -> Result<SdbFile, FormatError> {
    if bytes.len() < 0x34 {
        return Err(FormatError::InvalidSdb(format!(
            "File too short for .sdb header ({} bytes)",
            bytes.len()
        )));
    }
    let program_name = String::from_utf8_lossy(&bytes[0x00..0x10])
        .trim()
        .to_string();
    let program_version = String::from_utf8_lossy(&bytes[0x10..0x20])
        .trim()
        .to_string();
    let file_version = String::from_utf8_lossy(&bytes[0x20..0x30])
        .trim()
        .to_string();
    if !program_name.starts_with("SAP2000") {
        return Err(FormatError::InvalidSdb(format!(
            "Unexpected .sdb header magic: '{}'",
            program_name
        )));
    }
    let png_len = u32::from_le_bytes(bytes[0x30..0x34].try_into().unwrap()) as usize;
    let gz_offset = 0x34usize
        .checked_add(png_len)
        .ok_or_else(|| FormatError::InvalidSdb(format!("PNG length overflow ({})", png_len)))?;
    if gz_offset >= bytes.len() {
        return Err(FormatError::InvalidSdb(format!(
            "Invalid PNG length {} in .sdb of size {}",
            png_len,
            bytes.len()
        )));
    }
    let thumbnail_png = bytes[0x34..gz_offset].to_vec();
    let gz_slice = &bytes[gz_offset..];
    let decoder = GzDecoder::new(gz_slice);
    let mut limited = decoder.take(MAX_DECOMPRESSED_SDB_BYTES + 1);
    let mut decompressed_stream = Vec::new();
    limited
        .read_to_end(&mut decompressed_stream)
        .map_err(|e| FormatError::InvalidSdb(format!("Gzip decompression failed: {}", e)))?;
    if decompressed_stream.len() as u64 > MAX_DECOMPRESSED_SDB_BYTES {
        return Err(FormatError::InvalidSdb(format!(
            "Decompressed .sdb payload exceeds {} MiB safety limit",
            MAX_DECOMPRESSED_SDB_BYTES / (1024 * 1024)
        )));
    }

    Ok(SdbFile {
        program_name,
        program_version,
        file_version,
        thumbnail_png,
        decompressed_stream,
    })
}

/// Reads a SAP2000 `.sdb` file from disk.
pub fn read_sdb_file(path: impl AsRef<Path>) -> Result<SdbFile, FormatError> {
    let bytes = std::fs::read(path)?;
    read_sdb_bytes(&bytes)
}

/// Serializes an [`SdbFile`] into raw `.sdb` container bytes (48-byte header + PNG + gzip stream).
pub fn write_sdb_bytes(sdb: &SdbFile) -> Result<Vec<u8>, FormatError> {
    let mut out =
        Vec::with_capacity(0x34 + sdb.thumbnail_png.len() + sdb.decompressed_stream.len());
    let mut pad16 = |s: &str| {
        let mut buf = [b' '; 16];
        let b = s.as_bytes();
        let n = b.len().min(16);
        buf[..n].copy_from_slice(&b[..n]);
        out.extend_from_slice(&buf);
    };
    pad16(&sdb.program_name);
    pad16(&sdb.program_version);
    pad16(&sdb.file_version);
    out.extend_from_slice(&(sdb.thumbnail_png.len() as u32).to_le_bytes());
    out.extend_from_slice(&sdb.thumbnail_png);

    let mut encoder = GzEncoder::new(&mut out, Compression::default());
    encoder.write_all(&sdb.decompressed_stream)?;
    encoder.finish()?;
    Ok(out)
}

/// Writes an [`SdbFile`] to disk.
pub fn write_sdb_file(path: impl AsRef<Path>, sdb: &SdbFile) -> Result<(), FormatError> {
    let bytes = write_sdb_bytes(sdb)?;
    std::fs::write(path, bytes)?;
    Ok(())
}

/// Packs a [`SapModel`] into an [`SdbFile`] container (using the gzip-compressed `.s2k` table
/// stream format recognized by `extract_sdb_to_model`). To export a file for native import into
/// CSI SAP2000, use [`crate::save_s2k_file`].
pub fn pack_model_to_sdb(model: &SapModel) -> Result<SdbFile, FormatError> {
    let report = try_write_s2k_with_report(model)?;
    Ok(pack_projected_text(model, &report.text))
}

pub(crate) fn pack_projected_text(model: &SapModel, s2k_text: &str) -> SdbFile {
    let mut decompressed_stream = Vec::with_capacity(RUST_SDB_S2K_MAGIC.len() + s2k_text.len());
    decompressed_stream.extend_from_slice(RUST_SDB_S2K_MAGIC);
    decompressed_stream.extend_from_slice(s2k_text.as_bytes());
    SdbFile {
        program_name: "SAP2000".to_string(),
        program_version: model.program_version.clone(),
        file_version: "19.00".to_string(),
        thumbnail_png: DEFAULT_THUMBNAIL_PNG.to_vec(),
        decompressed_stream,
    }
}

// ============================================================================
// Native SAP2000 v19 cBinaryStream Parser (`CSI.CSICommon.cBinaryStream`)
// ============================================================================

#[derive(Debug, Clone)]
struct SchemaStep {
    prop: Option<String>,
    kind: String,
}

static SCHEMAS: OnceLock<HashMap<String, Vec<SchemaStep>>> = OnceLock::new();

fn get_schemas() -> &'static HashMap<String, Vec<SchemaStep>> {
    SCHEMAS.get_or_init(|| {
        let mut map = HashMap::with_capacity(900);
        for line in SDB_SCHEMAS_TXT.lines() {
            if let Some((cid, rest)) = line.split_once('|') {
                let mut steps = Vec::new();
                if !rest.is_empty() {
                    for item in rest.split(',') {
                        if let Some((prop, kind)) = item.split_once(':') {
                            steps.push(SchemaStep {
                                prop: Some(prop.to_string()),
                                kind: kind.to_string(),
                            });
                        } else {
                            steps.push(SchemaStep {
                                prop: None,
                                kind: item.to_string(),
                            });
                        }
                    }
                }
                map.insert(cid.to_string(), steps);
            }
        }
        map
    })
}

#[derive(Debug, Clone)]
enum SdbVal {
    Null,
    Bool(bool),
    I32(i32),
    I64(i64),
    F64(f64),
    Str(String),
    ArrBool(Vec<bool>),
    ArrI32(Vec<i32>),
    ArrF64(Vec<f64>),
    Obj(HashMap<String, SdbVal>),
    ArrObj(Vec<HashMap<String, SdbVal>>),
}

impl SdbVal {
    fn as_bool(&self) -> Option<bool> {
        match self {
            SdbVal::Bool(b) => Some(*b),
            _ => None,
        }
    }
    fn as_i32(&self) -> Option<i32> {
        match self {
            SdbVal::I32(v) => Some(*v),
            SdbVal::I64(v) => Some(*v as i32),
            _ => None,
        }
    }
    fn as_f64(&self) -> Option<f64> {
        match self {
            SdbVal::F64(v) => Some(*v),
            SdbVal::I32(v) => Some(*v as f64),
            _ => None,
        }
    }
    fn as_str(&self) -> Option<&str> {
        match self {
            SdbVal::Str(s) => Some(s.as_str()),
            _ => None,
        }
    }
    fn as_obj(&self) -> Option<&HashMap<String, SdbVal>> {
        match self {
            SdbVal::Obj(o) => Some(o),
            _ => None,
        }
    }
    fn as_arr_obj(&self) -> Option<&[HashMap<String, SdbVal>]> {
        match self {
            SdbVal::ArrObj(a) => Some(a.as_slice()),
            _ => None,
        }
    }
    fn as_arr_f64(&self) -> Option<&[f64]> {
        match self {
            SdbVal::ArrF64(a) => Some(a.as_slice()),
            _ => None,
        }
    }
    fn as_arr_i32(&self) -> Option<&[i32]> {
        match self {
            SdbVal::ArrI32(a) => Some(a.as_slice()),
            _ => None,
        }
    }
    fn as_arr_bool(&self) -> Option<&[bool]> {
        match self {
            SdbVal::ArrBool(a) => Some(a.as_slice()),
            _ => None,
        }
    }
}

struct BinarySdbReader<'a> {
    data: &'a [u8],
    pos: usize,
    depth: usize,
    schemas: &'static HashMap<String, Vec<SchemaStep>>,
    inventory: Option<Vec<native::NativeRecord>>,
}

impl<'a> BinarySdbReader<'a> {
    fn new(data: &'a [u8]) -> Self {
        Self {
            data,
            pos: 0,
            depth: 0,
            schemas: get_schemas(),
            inventory: None,
        }
    }

    fn ensure(&self, n: usize) -> Result<(), FormatError> {
        let end = self.pos.checked_add(n).ok_or_else(|| {
            FormatError::InvalidSdb(format!(
                "Stream offset overflow at byte {} + {}",
                self.pos, n
            ))
        })?;
        if end > self.data.len() {
            Err(FormatError::InvalidSdb(format!(
                "Unexpected EOF at byte {} (needed {} more bytes, total {})",
                self.pos,
                n,
                self.data.len()
            )))
        } else {
            Ok(())
        }
    }

    fn check_count(&self, raw_cnt: i32, min_elem_bytes: usize) -> Result<usize, FormatError> {
        if raw_cnt < 0 {
            return Err(FormatError::InvalidSdb(format!(
                "Negative array length {} at stream offset {}",
                raw_cnt, self.pos
            )));
        }
        let cnt = raw_cnt as usize;
        if cnt > MAX_ARRAY_LEN {
            return Err(FormatError::InvalidSdb(format!(
                "Array length {} exceeds safety limit ({}) at stream offset {}",
                cnt, MAX_ARRAY_LEN, self.pos
            )));
        }
        let min_bytes = cnt.checked_mul(min_elem_bytes).ok_or_else(|| {
            FormatError::InvalidSdb(format!("Array byte size overflow for count {}", cnt))
        })?;
        self.ensure(min_bytes)?;
        Ok(cnt)
    }

    fn read_u8(&mut self) -> Result<u8, FormatError> {
        self.ensure(1)?;
        let v = self.data[self.pos];
        self.pos += 1;
        Ok(v)
    }

    fn read_bool(&mut self) -> Result<bool, FormatError> {
        Ok(self.read_u8()? != 0)
    }

    fn read_i16(&mut self) -> Result<i16, FormatError> {
        self.ensure(2)?;
        let v = i16::from_le_bytes(self.data[self.pos..self.pos + 2].try_into().unwrap());
        self.pos += 2;
        Ok(v)
    }

    fn read_i32(&mut self) -> Result<i32, FormatError> {
        self.ensure(4)?;
        let v = i32::from_le_bytes(self.data[self.pos..self.pos + 4].try_into().unwrap());
        self.pos += 4;
        Ok(v)
    }

    fn read_i64(&mut self) -> Result<i64, FormatError> {
        self.ensure(8)?;
        let v = i64::from_le_bytes(self.data[self.pos..self.pos + 8].try_into().unwrap());
        self.pos += 8;
        Ok(v)
    }

    fn read_f32(&mut self) -> Result<f32, FormatError> {
        self.ensure(4)?;
        let v = f32::from_le_bytes(self.data[self.pos..self.pos + 4].try_into().unwrap());
        self.pos += 4;
        Ok(v)
    }

    fn read_f64(&mut self) -> Result<f64, FormatError> {
        self.ensure(8)?;
        let v = f64::from_le_bytes(self.data[self.pos..self.pos + 8].try_into().unwrap());
        self.pos += 8;
        Ok(v)
    }

    fn read_7bit_int(&mut self) -> Result<usize, FormatError> {
        let mut res = 0usize;
        let mut shift = 0u32;
        loop {
            if shift >= 35 {
                return Err(FormatError::InvalidSdb(format!(
                    "Corrupt 7-bit integer at stream offset {}",
                    self.pos
                )));
            }
            let b = self.read_u8()?;
            res |= ((b & 0x7f) as usize) << shift;
            if (b & 0x80) == 0 {
                break;
            }
            shift += 7;
        }
        Ok(res)
    }

    fn read_string(&mut self) -> Result<Option<String>, FormatError> {
        let len = self.read_7bit_int()?;
        self.ensure(len)?;
        let s = String::from_utf8_lossy(&self.data[self.pos..self.pos + len]).to_string();
        self.pos += len;
        if s == "~NothingString~" {
            Ok(None)
        } else {
            Ok(Some(s))
        }
    }

    fn read_prim(&mut self, ptype: &str, keep: bool) -> Result<SdbVal, FormatError> {
        if self.depth >= MAX_RECURSION_DEPTH {
            return Err(FormatError::InvalidSdb(format!(
                "Maximum recursion depth ({}) exceeded at offset {}",
                MAX_RECURSION_DEPTH, self.pos
            )));
        }
        self.depth += 1;
        let res = self.read_prim_inner(ptype, keep);
        self.depth -= 1;
        res
    }

    fn read_prim_inner(&mut self, ptype: &str, keep: bool) -> Result<SdbVal, FormatError> {
        match ptype {
            "bool" => Ok(SdbVal::Bool(self.read_bool()?)),
            "u1" => Ok(SdbVal::I32(self.read_u8()? as i32)),
            "i2" => Ok(SdbVal::I32(self.read_i16()? as i32)),
            "i4" => Ok(SdbVal::I32(self.read_i32()?)),
            "i8" => Ok(SdbVal::I64(self.read_i64()?)),
            "r4" => Ok(SdbVal::F64(self.read_f32()? as f64)),
            "r8" => Ok(SdbVal::F64(self.read_f64()?)),
            "string" => Ok(match self.read_string()? {
                Some(s) => SdbVal::Str(s),
                None => SdbVal::Null,
            }),
            _ if ptype.starts_with("System.Nullable`1<") => {
                let inner = &ptype["System.Nullable`1<".len()..ptype.len() - 1];
                let flag = self.read_i32()?;
                if flag == -1 {
                    Ok(SdbVal::Null)
                } else {
                    self.read_prim(inner, keep)
                }
            }
            _ if ptype.endswith_arr_or_list() => {
                let inner = if let Some(s) = ptype.strip_suffix("[]") {
                    s
                } else {
                    &ptype["System.Collections.Generic.List`1<".len()..ptype.len() - 1]
                };
                let non_empty = self.read_bool()?;
                if !non_empty {
                    return Ok(match inner {
                        "r8" | "r4" => SdbVal::ArrF64(Vec::new()),
                        "i4" | "i2" | "u1" => SdbVal::ArrI32(Vec::new()),
                        "bool" => SdbVal::ArrBool(Vec::new()),
                        _ => SdbVal::Null,
                    });
                }
                let raw_cnt = self.read_i32()?;
                let min_elem = match inner {
                    "r8" | "i8" => 8,
                    "r4" | "i4" => 4,
                    "i2" => 2,
                    _ => 1,
                };
                let cnt = self.check_count(raw_cnt, min_elem)?;
                if !keep {
                    self.skip_prim_n(inner, cnt)?;
                    return Ok(SdbVal::Null);
                }
                match inner {
                    "r8" => {
                        let mut v = Vec::with_capacity(cnt);
                        for _ in 0..cnt {
                            v.push(self.read_f64()?);
                        }
                        Ok(SdbVal::ArrF64(v))
                    }
                    "r4" => {
                        let mut v = Vec::with_capacity(cnt);
                        for _ in 0..cnt {
                            v.push(self.read_f32()? as f64);
                        }
                        Ok(SdbVal::ArrF64(v))
                    }
                    "i4" => {
                        let mut v = Vec::with_capacity(cnt);
                        for _ in 0..cnt {
                            v.push(self.read_i32()?);
                        }
                        Ok(SdbVal::ArrI32(v))
                    }
                    "i2" => {
                        let mut v = Vec::with_capacity(cnt);
                        for _ in 0..cnt {
                            v.push(self.read_i16()? as i32);
                        }
                        Ok(SdbVal::ArrI32(v))
                    }
                    "bool" => {
                        let mut v = Vec::with_capacity(cnt);
                        for _ in 0..cnt {
                            v.push(self.read_bool()?);
                        }
                        Ok(SdbVal::ArrBool(v))
                    }
                    _ => {
                        self.skip_prim_n(inner, cnt)?;
                        Ok(SdbVal::Null)
                    }
                }
            }
            _ => Err(FormatError::InvalidSdb(format!(
                "Unknown cBinaryStream primitive type: {}",
                ptype
            ))),
        }
    }

    fn skip_prim_n(&mut self, inner: &str, cnt: usize) -> Result<(), FormatError> {
        match inner {
            "bool" | "u1" => {
                self.ensure(cnt)?;
                self.pos += cnt;
            }
            "i2" => {
                let bytes = cnt.checked_mul(2).ok_or_else(|| {
                    FormatError::InvalidSdb("Integer overflow skipping i2 array".into())
                })?;
                self.ensure(bytes)?;
                self.pos += bytes;
            }
            "i4" | "r4" => {
                let bytes = cnt.checked_mul(4).ok_or_else(|| {
                    FormatError::InvalidSdb("Integer overflow skipping i4/r4 array".into())
                })?;
                self.ensure(bytes)?;
                self.pos += bytes;
            }
            "i8" | "r8" => {
                let bytes = cnt.checked_mul(8).ok_or_else(|| {
                    FormatError::InvalidSdb("Integer overflow skipping i8/r8 array".into())
                })?;
                self.ensure(bytes)?;
                self.pos += bytes;
            }
            "string" => {
                for _ in 0..cnt {
                    let _ = self.read_string()?;
                }
            }
            _ => {
                for _ in 0..cnt {
                    let _ = self.read_prim(inner, false)?;
                }
            }
        }
        Ok(())
    }

    fn read_nd_array(
        &mut self,
        inner: &str,
        dims: usize,
        keep: bool,
    ) -> Result<SdbVal, FormatError> {
        let non_empty = self.read_bool()?;
        if !non_empty {
            return Ok(SdbVal::ArrF64(Vec::new()));
        }
        let mut total = 1usize;
        for _ in 0..dims {
            let raw_c = self.read_i32()?;
            if raw_c < 0 {
                return Err(FormatError::InvalidSdb(format!(
                    "Negative ND-array dimension {} at offset {}",
                    raw_c, self.pos
                )));
            }
            total = total
                .checked_mul(raw_c as usize)
                .ok_or_else(|| FormatError::InvalidSdb("ND-array dimension overflow".into()))?;
        }
        let min_elem = match inner {
            "r8" | "i8" => 8,
            "r4" | "i4" => 4,
            "i2" => 2,
            _ => 1,
        };
        if total > MAX_ARRAY_LEN {
            return Err(FormatError::InvalidSdb(format!(
                "ND-array total elements {} exceeds safety limit ({})",
                total, MAX_ARRAY_LEN
            )));
        }
        let min_bytes = total
            .checked_mul(min_elem)
            .ok_or_else(|| FormatError::InvalidSdb("ND-array byte size overflow".into()))?;
        self.ensure(min_bytes)?;
        if keep && inner == "r8" {
            let mut v = Vec::with_capacity(total);
            for _ in 0..total {
                v.push(self.read_f64()?);
            }
            Ok(SdbVal::ArrF64(v))
        } else {
            self.skip_prim_n(inner, total)?;
            Ok(SdbVal::Null)
        }
    }

    fn read_struct(&mut self, keep: bool) -> Result<Option<HashMap<String, SdbVal>>, FormatError> {
        if self.depth >= MAX_RECURSION_DEPTH {
            return Err(FormatError::InvalidSdb(format!(
                "Maximum struct recursion depth ({}) exceeded at offset {}",
                MAX_RECURSION_DEPTH, self.pos
            )));
        }
        self.depth += 1;
        let res = self.read_struct_inner(keep);
        self.depth -= 1;
        res
    }

    fn read_struct_inner(
        &mut self,
        keep: bool,
    ) -> Result<Option<HashMap<String, SdbVal>>, FormatError> {
        let record_start = self.pos;
        let non_empty = self.read_bool()?;
        if !non_empty {
            return Ok(None);
        }
        let class_id = self.read_string()?.unwrap_or_default();
        let steps = self.schemas.get(&class_id).ok_or_else(|| {
            FormatError::InvalidSdb(format!(
                "Unknown SAP2000 ClassID '{}' at stream offset {}",
                class_id, self.pos
            ))
        })?;
        let mut obj = if keep {
            HashMap::with_capacity(steps.len())
        } else {
            HashMap::new()
        };
        let mut spans = Vec::new();
        for st in steps {
            let field_start = self.pos;
            let keep_field = keep && st.prop.is_some();
            let val = match st.kind.as_str() {
                "struct" => match self.read_struct(keep_field)? {
                    Some(o) => SdbVal::Obj(o),
                    None => SdbVal::Null,
                },
                "struct[]" => SdbVal::ArrObj(self.read_struct_array(keep_field)?),
                "struct[,]" => SdbVal::ArrObj(self.read_struct_array2(keep_field)?),
                "WriteShort2" => self.read_nd_array("i2", 2, keep_field)?,
                "WriteInt2" => self.read_nd_array("i4", 2, keep_field)?,
                "WriteSingle2" => self.read_nd_array("r4", 2, keep_field)?,
                "WriteDouble2" => self.read_nd_array("r8", 2, keep_field)?,
                "WriteString2" => self.read_nd_array("string", 2, keep_field)?,
                "WriteByte2" => self.read_nd_array("u1", 2, keep_field)?,
                "WriteDouble3" => self.read_nd_array("r8", 3, keep_field)?,
                "WriteDouble4" => self.read_nd_array("r8", 4, keep_field)?,
                prim => self.read_prim(prim, keep_field)?,
            };
            if self.inventory.is_some() {
                spans.push(native::NativeFieldSpan {
                    name: st.prop.clone(),
                    wire_type: st.kind.clone(),
                    offset: field_start,
                    length: self.pos - field_start,
                });
            }
            if let Some(prop) = &st.prop {
                if keep {
                    obj.insert(prop.clone(), val);
                }
            }
        }
        if let Some(inventory) = &mut self.inventory {
            inventory.push(native::NativeRecord {
                class_id,
                offset: record_start,
                length: self.pos - record_start,
                fields: spans,
            });
        }
        Ok(Some(obj))
    }

    fn read_struct_array(
        &mut self,
        keep: bool,
    ) -> Result<Vec<HashMap<String, SdbVal>>, FormatError> {
        let non_empty = self.read_bool()?;
        if !non_empty {
            return Ok(Vec::new());
        }
        let raw_cnt = self.read_i32()?;
        let cnt = self.check_count(raw_cnt, 1)?;
        let mut out = if keep {
            Vec::with_capacity(cnt)
        } else {
            Vec::new()
        };
        for _ in 0..cnt {
            let object = self.read_struct(keep)?;
            if keep {
                out.push(
                    object.unwrap_or_else(|| {
                        HashMap::from([("m_alive".into(), SdbVal::Bool(false))])
                    }),
                );
            }
        }
        Ok(out)
    }

    fn read_struct_array2(
        &mut self,
        keep: bool,
    ) -> Result<Vec<HashMap<String, SdbVal>>, FormatError> {
        let non_empty = self.read_bool()?;
        if !non_empty {
            return Ok(Vec::new());
        }
        let raw_c0 = self.read_i32()?;
        let raw_c1 = self.read_i32()?;
        if raw_c0 < 0 || raw_c1 < 0 {
            return Err(FormatError::InvalidSdb(format!(
                "Negative 2D struct array dimensions ({}, {}) at offset {}",
                raw_c0, raw_c1, self.pos
            )));
        }
        let total_i64 = (raw_c0 as i64).saturating_mul(raw_c1 as i64);
        if total_i64 > MAX_ARRAY_LEN as i64 {
            return Err(FormatError::InvalidSdb(format!(
                "2D struct array total {} exceeds safety limit ({})",
                total_i64, MAX_ARRAY_LEN
            )));
        }
        let total = self.check_count(total_i64 as i32, 1)?;
        let mut out = if keep {
            Vec::with_capacity(total)
        } else {
            Vec::new()
        };
        for _ in 0..total {
            let object = self.read_struct(keep)?;
            if keep {
                out.push(
                    object.unwrap_or_else(|| {
                        HashMap::from([("m_alive".into(), SdbVal::Bool(false))])
                    }),
                );
            }
        }
        Ok(out)
    }
}

trait StrArrExt {
    fn endswith_arr_or_list(&self) -> bool;
}

impl StrArrExt for str {
    fn endswith_arr_or_list(&self) -> bool {
        self.ends_with("[]") || self.starts_with("System.Collections.Generic.List`1<")
    }
}

fn map_frame_shape(sh: i32) -> FramePropType {
    match sh {
        1 => FramePropType::I,
        2 => FramePropType::Channel,
        3 => FramePropType::T,
        4 => FramePropType::Angle,
        5 => FramePropType::DblAngle,
        6 => FramePropType::Box,
        7 => FramePropType::Pipe,
        8 => FramePropType::Rectangular,
        9 => FramePropType::Circle,
        10 => FramePropType::General,
        11 => FramePropType::DbChannel,
        12 => FramePropType::Auto,
        13 => FramePropType::SD,
        14 => FramePropType::Variable,
        _ => FramePropType::General,
    }
}

fn map_load_pat_type(t: i32) -> LoadPatternType {
    match t {
        1 => LoadPatternType::Dead,
        2 => LoadPatternType::SuperDead,
        3 => LoadPatternType::Live,
        4 => LoadPatternType::ReduceLive,
        5 => LoadPatternType::Quake,
        6 => LoadPatternType::Wind,
        7 => LoadPatternType::Snow,
        8 => LoadPatternType::Other,
        9 => LoadPatternType::Move,
        10 => LoadPatternType::Temperature,
        11 => LoadPatternType::Rooflive,
        12 => LoadPatternType::Notional,
        13 => LoadPatternType::PatternLive,
        14 => LoadPatternType::Wave,
        34 => LoadPatternType::Prestress,
        _ => LoadPatternType::Other,
    }
}

fn native_alive(object: &HashMap<String, SdbVal>) -> bool {
    object
        .get("m_alive")
        .and_then(SdbVal::as_bool)
        .unwrap_or(true)
}
fn native_name(object: &HashMap<String, SdbVal>, key: &str, fallback: &str) -> String {
    object
        .get(key)
        .and_then(SdbVal::as_str)
        .filter(|name| !name.is_empty())
        .unwrap_or(fallback)
        .to_owned()
}
fn native_floats<const N: usize>(object: &HashMap<String, SdbVal>, key: &str) -> [f64; N] {
    let mut values = [0.0; N];
    if let Some(raw) = object.get(key).and_then(SdbVal::as_arr_f64) {
        for (out, input) in values.iter_mut().zip(raw) {
            *out = *input;
        }
    }
    values
}
fn native_flags(object: &HashMap<String, SdbVal>, key: &str) -> [bool; 6] {
    let mut values = [false; 6];
    if let Some(raw) = object.get(key).and_then(SdbVal::as_arr_bool) {
        for (out, input) in values.iter_mut().zip(raw) {
            *out = *input;
        }
    }
    values
}
fn native_case_type(code: i32) -> Result<LoadCaseType, FormatError> {
    Ok(match code {
        1 => LoadCaseType::LinearStatic,
        2 => LoadCaseType::NonlinearStatic,
        3 => LoadCaseType::Modal,
        4 => LoadCaseType::ResponseSpectrum,
        5 => LoadCaseType::LinearHistory,
        6 => LoadCaseType::NonlinearHistory,
        7 => LoadCaseType::LinearDynamic,
        8 => LoadCaseType::NonlinearDynamic,
        9 => LoadCaseType::MovingLoad,
        10 => LoadCaseType::Buckling,
        11 => LoadCaseType::SteadyState,
        12 => LoadCaseType::PowerSpectralDensity,
        13 => LoadCaseType::LinearStaticMultiStep,
        14 => LoadCaseType::HyperStatic,
        _ => {
            return Err(FormatError::InvalidSdb(format!(
                "Unknown native analysis case type {}",
                code
            )))
        }
    })
}
fn native_link_type(code: i32) -> Result<LinkPropType, FormatError> {
    Ok(match code {
        1 => LinkPropType::Linear,
        2 => LinkPropType::Damper,
        3 => LinkPropType::Gap,
        4 => LinkPropType::Hook,
        5 => LinkPropType::PlasticWen,
        6 => LinkPropType::RubberIsolator,
        7 => LinkPropType::FrictionIsolator,
        8 => LinkPropType::MultiLinearElastic,
        9 => LinkPropType::MultiLinearPlastic,
        10 => LinkPropType::TCFrictionIsolator,
        11 => LinkPropType::DamperBilinear,
        12 => LinkPropType::DamperFrictionSpring,
        13 => LinkPropType::TriplePendulumIsolator,
        _ => {
            return Err(FormatError::InvalidSdb(format!(
                "Unknown native link property type {}",
                code
            )))
        }
    })
}

fn check_native_dynamic_assignment(
    case: &str,
    case_type: i32,
    load: &HashMap<String, SdbVal>,
) -> Result<(), FormatError> {
    let mut unsupported = Vec::new();
    let angle = load.get("m_Ang").and_then(SdbVal::as_f64).unwrap_or(0.0);
    let csys = load.get("m_csys").and_then(SdbVal::as_i32).unwrap_or(1);
    if angle != 0.0 {
        unsupported.push(format!("rotated excitation angle {}", angle));
    }
    if csys != 1 {
        unsupported.push(format!("excitation coordinate system index {}", csys));
    }
    if matches!(case_type, 5..=8) {
        let scale = load.get("m_TF").and_then(SdbVal::as_f64).unwrap_or(1.0);
        let arrival = load.get("m_at").and_then(SdbVal::as_f64).unwrap_or(0.0);
        let phase = load
            .get("m_PhaseAngle")
            .and_then(SdbVal::as_f64)
            .unwrap_or(0.0);
        if scale != 1.0 {
            unsupported.push(format!("function time scale {}", scale));
        }
        if arrival != 0.0 {
            unsupported.push(format!("arrival time {}", arrival));
        }
        if phase != 0.0 {
            unsupported.push(format!("phase angle {}", phase));
        }
    }
    if unsupported.is_empty() {
        Ok(())
    } else {
        Err(FormatError::UnsupportedNativeCase {
            case_name: case.to_owned(),
            reason: unsupported.join(", "),
        })
    }
}

/// Diagonal entry positions in SAP2000's 21-term column-wise upper-triangular
/// joint spring array (`[U1U1, U1U2, U2U2, U1U3, U2U3, U3U3, U1R1, ..., R3R3]`).
pub(crate) const SPRING_DIAG_INDICES: [usize; 6] = [0, 2, 5, 9, 14, 20];

fn coordinate_name(names: &[String], index: i32) -> Result<String, FormatError> {
    if index <= 0 {
        return Ok("LOCAL".to_string());
    }
    names
        .get(index as usize)
        .filter(|name| !name.is_empty())
        .cloned()
        .ok_or_else(|| {
            FormatError::InvalidSdb(format!("Unknown coordinate system index {}", index))
        })
}

/// Extracts a populated [`SapModel`] from a parsed [`SdbFile`].
pub fn extract_sdb_to_model(sdb: &SdbFile) -> Result<SapModel, FormatError> {
    if let Some(s2k_bytes) = sdb.decompressed_stream.strip_prefix(RUST_SDB_S2K_MAGIC) {
        let s2k_str = std::str::from_utf8(s2k_bytes).map_err(|e| {
            FormatError::InvalidSdb(format!("Invalid UTF-8 text model stream: {}", e))
        })?;
        return parse_s2k(s2k_str);
    }

    let mut r = BinarySdbReader::new(&sdb.decompressed_stream);
    let _name_in_file = r.read_string()?;
    let prog_ver = r
        .read_string()?
        .unwrap_or_else(|| sdb.program_version.clone());
    let _file_ver = r.read_string()?;

    // z (0x06001dcb)
    let proj_startup = r.read_struct(true)?;
    let _proj_info = r.read_struct(false)?;
    let _standards = r.read_struct_array(false)?;
    let _am_timestamp = r.read_struct(false)?;
    let run_params = r.read_struct(true)?;
    let _present_csys = r.read_i32()?;

    let units_code = proj_startup
        .as_ref()
        .and_then(|o| o.get("m_Units"))
        .and_then(|v| v.as_i32())
        .unwrap_or(3);
    let units = Units::from_code(units_code)?;
    let mut model = SapModel::new(units);
    model.program_version = prog_ver;
    if let Some(dofs) = run_params
        .as_ref()
        .and_then(|o| o.get("m_dof"))
        .and_then(|v| v.as_arr_i32())
    {
        for i in 0..6.min(dofs.len()) {
            model.active_dofs[i] = dofs[i] != 0;
        }
    }
    model.materials.clear();
    model.frame_sections.clear();
    model.area_sections.clear();
    model.load_patterns.clear();
    model.load_cases.clear();

    // y (0x06001dcc)
    let raw_coord_systems = r.read_struct_array(true)?;
    let mut coord_names = vec![String::new()];
    for (index, coord) in raw_coord_systems.iter().enumerate() {
        let name = coord
            .get("m_csname")
            .and_then(SdbVal::as_str)
            .filter(|name| !name.is_empty())
            .map(str::to_owned)
            .unwrap_or_else(|| format!("CSYS{}", index + 1));
        coord_names.push(name.clone());
        if !coord
            .get("m_alive")
            .and_then(SdbVal::as_bool)
            .unwrap_or(true)
        {
            continue;
        }
        model.coord_systems.insert(
            name.clone(),
            sap_model::CoordSystem {
                name,
                origin: ["m_Ux", "m_Uy", "m_UZ"]
                    .map(|key| coord.get(key).and_then(SdbVal::as_f64).unwrap_or(0.0)),
                angles_deg: ["m_RZ", "m_Ry", "m_RX"]
                    .map(|key| coord.get(key).and_then(SdbVal::as_f64).unwrap_or(0.0)),
                grids: Vec::new(),
            },
        );
    }
    for _ in 0..4 {
        let _ = r.read_struct_array(false)?;
    }

    // x (0x06001dcd): Materials & Sections
    let raw_materials = r.read_struct_array(true)?;
    let raw_sections = r.read_struct_array(true)?;
    let raw_auto = r.read_struct_array(true)?;
    let raw_vs = r.read_struct_array(true)?;
    let _rebar = r.read_struct_array(false)?;
    let _sd_sec = r.read_struct_array(false)?;
    let _joist = r.read_struct_array(false)?;
    let raw_shell_sections = r.read_struct_array(true)?;
    let raw_solid_props = r.read_struct_array(true)?;
    let raw_link_props = r.read_struct_array(true)?;
    let _nllink_fd_props = r.read_struct_array(false)?;
    let raw_cable_props = r.read_struct_array(true)?;
    let _tendon_props = r.read_struct_array(false)?;

    // w (0x06001dce)
    for _ in 0..4 {
        let _ = r.read_struct_array(false)?;
    }

    // v (0x06001dcf): PointObjects, LineObjects, AreaObjects, SolidObjects, NLLinkObjects
    let raw_points = r.read_struct_array(true)?;
    let raw_lines = r.read_struct_array(true)?;
    let _frame_ext = r.read_struct_array(false)?;
    let raw_areas = r.read_struct_array(true)?;
    let raw_solids = r.read_struct_array(true)?;
    let raw_links = r.read_struct_array(true)?;

    // u (0x06001dd0): Restraint, Constraint, CJoint, Pattern, PJoint, Spring, Mass...
    let raw_restraints = r.read_struct_array(true)?;
    let raw_constraints = r.read_struct_array(true)?;
    let raw_cjoint = r.read_struct_array(true)?;
    let raw_patterns = r.read_struct_array(true)?;
    let raw_pjoint = r.read_struct_array(true)?;
    let raw_springs = r.read_struct_array(true)?;
    let raw_masses = r.read_struct_array(true)?;
    for _ in 0..3 {
        let _ = r.read_struct_array(false)?;
    }

    // t (0x06001dd1): Frame/Area modifiers & fixity
    let _mat_temp = r.read_struct_array(false)?;
    let raw_pd_force = r.read_struct_array(true)?;
    let raw_fmodifier = r.read_struct_array(true)?;
    let raw_ffixity = r.read_struct_array(true)?;
    let raw_foffset = r.read_struct_array(true)?;
    let _fspring = r.read_struct_array(false)?;
    let _fmass = r.read_struct_array(false)?;
    let _ffireproofing = r.read_struct_array(false)?;
    let raw_shmodifier = r.read_struct_array(true)?;
    let _shspring = r.read_struct_array(false)?;
    let raw_shmass = r.read_struct_array(true)?;
    let raw_shmesh = r.read_struct_array(true)?;
    let _sspring = r.read_struct_array(false)?;

    // s (0x06001dd2): LoadPattern, AnalysisCase, Combo, Loads
    let raw_load_patterns = r.read_struct_array(true)?;
    for _ in 0..3 {
        let _ = r.read_struct_array(false)?;
    }
    let _undef_geom = r.read_struct(false)?;
    let raw_analysis_cases = r.read_struct_array(true)?;
    let _schedule = r.read_struct_array(false)?;
    let _mass_source = r.read_struct_array(false)?;
    let raw_combos = r.read_struct_array(true)?;
    let _auto_combo_opts = r.read_struct_array(false)?;
    let raw_jfload = r.read_struct_array(true)?;
    let raw_jgdload = r.read_struct_array(true)?;
    let raw_fgload = r.read_struct_array(true)?;
    let raw_fcload = r.read_struct_array(true)?;
    let raw_fdload = r.read_struct_array(true)?;
    let raw_ftload = r.read_struct_array(true)?;
    let _fsload = r.read_struct_array(false)?;
    let _fdefload = r.read_struct_array(false)?;
    let _ftfload = r.read_struct_array(false)?;
    let _ptload = r.read_struct_array(false)?;
    let _wind_ow = r.read_struct_array(false)?;
    let _shgload = r.read_struct_array(false)?;
    let raw_shuload = r.read_struct_array(true)?;
    let _shufload = r.read_struct_array(false)?;
    let raw_shspload = r.read_struct_array(true)?;
    let _shppload = r.read_struct_array(false)?;
    let raw_shtload = r.read_struct_array(true)?;
    for _ in 0..5 {
        let _ = r.read_struct_array(false)?;
    } // ShSLoad..SoPPLoad
    let raw_sospload = r.read_struct_array(true)?;
    let raw_sotload = r.read_struct_array(true)?;
    let _sosload = r.read_struct_array(false)?;
    // Link gravity loads are not modeled; they are read so their presence is
    // recorded instead of the load silently disappearing.
    let raw_nlgload = r.read_struct_array(true)?;
    for _ in 0..3 {
        let _ = r.read_struct_array(false)?;
    } // NLDefLoad..Seastate
    for _ in 0..9 {
        let _ = r.read_struct_array(false)?;
    } // wave records
    let raw_functions = r.read_struct_array(true)?;
    let raw_function_values = r.read_struct_array(true)?;

    // Convert Materials (1-based index -> Material name)
    let mut mat_names = Vec::with_capacity(raw_materials.len() + 1);
    mat_names.push(String::new());
    for (idx, m) in raw_materials.iter().enumerate() {
        let name = m
            .get("m_Name")
            .and_then(|v| v.as_str())
            .filter(|s| !s.is_empty())
            .map(|s| s.to_string())
            .unwrap_or_else(|| format!("MAT{}", idx + 1));
        mat_names.push(name.clone());
        let alive = m.get("m_alive").and_then(|v| v.as_bool()).unwrap_or(true);
        if !alive {
            continue;
        }
        let mtype_code = m.get("m_mtype").and_then(|v| v.as_i32()).unwrap_or(1);
        let mat_type = match mtype_code {
            1 => MatType::Steel,
            2 => MatType::Concrete,
            3 => MatType::NoDesign,
            4 => MatType::Aluminum,
            5 => MatType::ColdFormed,
            6 => MatType::Rebar,
            7 => MatType::Tendon,
            _ => MatType::Steel,
        };
        let sym_type = m.get("m_SymType").and_then(SdbVal::as_i32).unwrap_or(0);
        match sym_type {
            0 | 1 => {}
            3 if matches!(mat_type, MatType::Rebar | MatType::Tendon) => {}
            other => {
                return Err(FormatError::InvalidSdb(format!(
                    "Unsupported material symmetry type m_SymType={other} on material '{name}' ({mat_type:?})"
                )));
            }
        }
        let mut mat = Material {
            name: name.clone(),
            mat_type,
            ..Material::default_steel_kip_in()
        };
        if let Some(color) = m.get("m_color").and_then(SdbVal::as_i32) {
            mat.color = native_material_color(color);
        }
        let mt0 = m
            .get("m_Mt")
            .and_then(SdbVal::as_arr_obj)
            .and_then(|mts| mts.first());
        if sym_type == 1 {
            let Some(mt0) = mt0 else {
                return Err(FormatError::InvalidSdb(format!(
                    "Orthotropic material '{name}' lacks S_MT mechanical properties"
                )));
            };
            let take3 = |field: &str| -> Result<[f64; 3], FormatError> {
                let arr = mt0.get(field).and_then(SdbVal::as_arr_f64).ok_or_else(|| {
                    FormatError::InvalidSdb(format!(
                        "Orthotropic material '{name}' lacks '{field}' array"
                    ))
                })?;
                if arr.len() < 3 {
                    return Err(FormatError::InvalidSdb(format!(
                        "Orthotropic material '{name}' field '{field}' has length {} (expected >= 3)",
                        arr.len()
                    )));
                }
                Ok([arr[0], arr[1], arr[2]])
            };
            let eo = take3("m_e")?;
            let go = take3("m_g")?;
            let uo = take3("m_u")?;
            let ao = take3("m_a")?;
            mat.e = eo[0];
            mat.e_ortho = Some(eo);
            mat.g = go[0];
            mat.g_ortho = Some(go);
            mat.u = uo[0];
            mat.u_ortho = Some(uo);
            mat.a = ao[0];
            mat.a_ortho = Some(ao);
        }
        if let Some(mt0) = mt0 {
            if let Some(w) = mt0.get("m_w").and_then(|v| v.as_f64()) {
                mat.unit_weight = w;
            }
            if let Some(mass) = mt0.get("m_m").and_then(|v| v.as_f64()) {
                mat.unit_mass = mass;
            }
            if sym_type != 1 {
                if let Some(es) = mt0.get("m_e").and_then(|v| v.as_arr_f64()) {
                    if let Some(&e1) = es.first() {
                        mat.e = e1;
                    }
                }
                if let Some(gs) = mt0.get("m_g").and_then(|v| v.as_arr_f64()) {
                    if let Some(&g1) = gs.first() {
                        mat.g = g1;
                    }
                }
                if let Some(us) = mt0.get("m_u").and_then(|v| v.as_arr_f64()) {
                    if let Some(&u1) = us.first() {
                        mat.u = u1;
                    }
                }
                if let Some(as_) = mt0.get("m_a").and_then(|v| v.as_arr_f64()) {
                    if let Some(&a1) = as_.first() {
                        mat.a = a1;
                    }
                }
            }
            if let Some(sp) = mt0.get("m_SteelProps").and_then(|v| v.as_obj()) {
                if let Some(fy) = sp.get("m_fy").and_then(|v| v.as_f64()) {
                    mat.fy = fy;
                }
                if let Some(fu) = sp.get("m_Fu").and_then(|v| v.as_f64()) {
                    mat.fu = fu;
                }
            }
            if let Some(cp) = mt0.get("m_ConcreteProps").and_then(|v| v.as_obj()) {
                if let Some(fc) = cp.get("m_fc").and_then(|v| v.as_f64()) {
                    mat.fc = fc;
                }
            }
        }
        // SapModel's independent shear-rebar Fys has no observed native scalar.
        // Use the material yield as a reconstruction fallback; native synthesis
        // rejects an independent Fys which this projection cannot preserve.
        mat.fys = mat.fy;
        model.materials.insert(name, mat);
    }

    // Convert Frame Sections (1-based index -> Section name)
    let mut fsec_names = Vec::with_capacity(raw_sections.len() + 1);
    fsec_names.push(String::new());
    for (idx, s) in raw_sections.iter().enumerate() {
        let name = s
            .get("m_sname")
            .and_then(|v| v.as_str())
            .filter(|n| !n.is_empty())
            .map(|n| n.to_string())
            .unwrap_or_else(|| format!("FSEC{}", idx + 1));
        fsec_names.push(name.clone());
        let alive = s.get("m_alive").and_then(|v| v.as_bool()).unwrap_or(true);
        if !alive {
            continue;
        }
        let mindex = s.get("m_Mindex").and_then(|v| v.as_i32()).unwrap_or(1) as usize;
        let mat_name = mat_names
            .get(mindex)
            .cloned()
            .filter(|n| !n.is_empty())
            .unwrap_or_else(|| "STEEL".to_string());
        let sh = s.get("m_sh").and_then(|v| v.as_i32()).unwrap_or(10);
        let mut modifiers = [1.0; 8];
        if let Some(msf) = s.get("m_msf").and_then(|v| v.as_arr_f64()) {
            for i in 0..8.min(msf.len()) {
                modifiers[i] = msf[i];
            }
        }
        let sec = FrameSection {
            name: name.clone(),
            material: mat_name,
            prop_type: map_frame_shape(sh),
            t3: s.get("m_t3").and_then(|v| v.as_f64()).unwrap_or(0.0),
            t2: s.get("m_t2").and_then(|v| v.as_f64()).unwrap_or(0.0),
            tf: s.get("m_TF").and_then(|v| v.as_f64()).unwrap_or(0.0),
            tw: s.get("m_TW").and_then(|v| v.as_f64()).unwrap_or(0.0),
            t2b: s.get("m_t2b").and_then(|v| v.as_f64()).unwrap_or(0.0),
            tfb: s.get("m_tfb").and_then(|v| v.as_f64()).unwrap_or(0.0),
            dis: s.get("m_dis").and_then(|v| v.as_f64()).unwrap_or(0.0),
            area: s.get("m_Area").and_then(|v| v.as_f64()).unwrap_or(0.0),
            torsional_j: s.get("m_Torsion").and_then(|v| v.as_f64()).unwrap_or(0.0),
            i33: s.get("m_I33").and_then(|v| v.as_f64()).unwrap_or(0.0),
            i22: s.get("m_I22").and_then(|v| v.as_f64()).unwrap_or(0.0),
            i23: s.get("m_I23").and_then(|v| v.as_f64()).unwrap_or(0.0),
            as2: s.get("m_As2").and_then(|v| v.as_f64()).unwrap_or(0.0),
            as3: s.get("m_As3").and_then(|v| v.as_f64()).unwrap_or(0.0),
            s33: s.get("m_S33").and_then(|v| v.as_f64()).unwrap_or(0.0),
            s22: s.get("m_S22").and_then(|v| v.as_f64()).unwrap_or(0.0),
            z33: s.get("m_Z33").and_then(|v| v.as_f64()).unwrap_or(0.0),
            z22: s.get("m_Z22").and_then(|v| v.as_f64()).unwrap_or(0.0),
            r33: s.get("m_R33").and_then(|v| v.as_f64()).unwrap_or(0.0),
            r22: s.get("m_R22").and_then(|v| v.as_f64()).unwrap_or(0.0),
            modifiers,
            non_prismatic: Vec::new(),
            auto_select_list: Vec::new(),
            concrete_rebar: None,
            file_name: s
                .get("m_PropertyFile")
                .and_then(|v| v.as_str())
                .filter(|s| !s.is_empty())
                .map(|s| s.to_string()),
            name_in_file: s
                .get("m_Sectname")
                .and_then(|v| v.as_str())
                .filter(|s| !s.is_empty())
                .map(|s| s.to_string()),
        };
        model.frame_sections.insert(name, sec);
    }

    // Populate Auto-Select lists (`AUTOARRAY`) & Non-Prismatic segments (`VSARRAY`)
    for au in &raw_auto {
        let p_idx = au
            .get("m_parentSection")
            .and_then(|v| v.as_i32())
            .unwrap_or(0) as usize;
        let c_idx = au
            .get("m_childSection")
            .and_then(|v| v.as_i32())
            .unwrap_or(0) as usize;
        if let (Some(pname), Some(cname)) = (
            fsec_names.get(p_idx).filter(|s| !s.is_empty()),
            fsec_names.get(c_idx).filter(|s| !s.is_empty()),
        ) {
            if let Some(sec) = model.frame_sections.get_mut(pname) {
                sec.auto_select_list.push(cname.clone());
            }
        }
    }
    for vs in &raw_vs {
        let s_idx = vs.get("m_snumber").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        let st_idx = vs.get("m_Start").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        let en_idx = vs.get("m_end").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        if let (Some(sname), Some(st_name), Some(en_name)) = (
            fsec_names.get(s_idx).filter(|s| !s.is_empty()),
            fsec_names.get(st_idx).filter(|s| !s.is_empty()),
            fsec_names.get(en_idx).filter(|s| !s.is_empty()),
        ) {
            if let Some(sec) = model.frame_sections.get_mut(sname) {
                let ltype = vs.get("m_type").and_then(|v| v.as_i32()).unwrap_or(1);
                sec.non_prismatic.push(NonPrismaticSegment {
                    start_sec: st_name.clone(),
                    end_sec: en_name.clone(),
                    length: vs.get("m_Length").and_then(|v| v.as_f64()).unwrap_or(1.0),
                    is_variable_length: ltype == 1,
                    ei33_variation: vs.get("m_EI33").and_then(|v| v.as_i32()).unwrap_or(1),
                    ei22_variation: vs.get("m_EI22").and_then(|v| v.as_i32()).unwrap_or(1),
                });
            }
        }
    }

    // Convert Area Sections (1-based index -> AreaSection name)
    let mut asec_names = Vec::with_capacity(raw_shell_sections.len() + 1);
    asec_names.push(String::new());
    for (idx, a) in raw_shell_sections.iter().enumerate() {
        let name = a
            .get("m_name")
            .and_then(|v| v.as_str())
            .filter(|n| !n.is_empty())
            .map(|n| n.to_string())
            .unwrap_or_else(|| format!("ASEC{}", idx + 1));
        asec_names.push(name.clone());
        let alive = a.get("m_alive").and_then(|v| v.as_bool()).unwrap_or(true);
        if !alive {
            continue;
        }
        let mindex = a.get("m_Mindex").and_then(|v| v.as_i32()).unwrap_or(1) as usize;
        let mat_name = mat_names
            .get(mindex)
            .cloned()
            .filter(|n| !n.is_empty())
            .unwrap_or_else(|| "CONC".to_string());
        let mut modifiers = [1.0; 10];
        if let Some(msf) = a.get("m_msf").and_then(|v| v.as_arr_f64()) {
            for i in 0..10.min(msf.len()) {
                modifiers[i] = msf[i];
            }
        }
        // Native area-section type. `m_ElementType` is 0 shell, 1 plane,
        // 2 axisymmetric solid. For shells the native `m_SType` is not the OAPI
        // enumeration: every "thin" verification file and every untouched
        // default section (the vendor default is Shell-Thin) stores 4, and
        // every "thick" verification file stores 1. For planes the six
        // "plane strain" verification files store 2 and the others 1. No other
        // code occurs in the corpus; an unknown one is recorded, not guessed.
        let stype = a.get("m_SType").and_then(|v| v.as_i32()).unwrap_or(4);
        let element_type = a.get("m_ElementType").and_then(|v| v.as_i32()).unwrap_or(0);
        let (prop_type, shell_type, plane_type, known) = match (element_type, stype) {
            (0, 4) => (AreaPropType::Shell, ShellType::ShellThin, PlaneType::PlaneStress, true),
            (0, 1) => (AreaPropType::Shell, ShellType::ShellThick, PlaneType::PlaneStress, true),
            (1, 1) => (AreaPropType::Plane, ShellType::ShellThin, PlaneType::PlaneStress, true),
            (1, 2) => (AreaPropType::Plane, ShellType::ShellThin, PlaneType::PlaneStrain, true),
            (2, _) => (AreaPropType::ASolid, ShellType::ShellThin, PlaneType::PlaneStress, true),
            _ => (AreaPropType::Shell, ShellType::ShellThin, PlaneType::PlaneStress, false),
        };
        if !known && native_alive(a) {
            model.unmodeled_source.push(UnmodeledSourceData {
                scope: UnmodeledScope::AllCases,
                description: format!(
                    "area section '{name}' has element type {element_type} and section type {stype}, whose meaning is not established (imported as a thin shell)"
                ),
            });
        }
        let asec = AreaSection {
            name: name.clone(),
            material: mat_name,
            prop_type,
            shell_type,
            plane_type,
            mat_angle_deg: a.get("m_Ang").and_then(|v| v.as_f64()).unwrap_or(0.0),
            thickness_membrane: a.get("m_Thickness").and_then(|v| v.as_f64()).unwrap_or(1.0),
            thickness_bending: a.get("m_bending").and_then(|v| v.as_f64()).unwrap_or(1.0),
            // The native flag is 1 when incompatible modes are OFF: every
            // "comp" verification file stores 1, every "incomp" file and every
            // untouched default (the vendor default is on) stores 0.
            incompatible_modes: a
                .get("m_Incompatible")
                .and_then(|v| v.as_i32())
                .unwrap_or(0)
                == 0,
            drilling_dof: a
                .get("m_IncludeDrillingDOF")
                .and_then(|v| v.as_bool())
                .unwrap_or(true),
            modifiers,
        };
        model.area_sections.insert(name, asec);
    }

    let mut solid_prop_names = vec![String::new()];
    for (index, prop) in raw_solid_props.iter().enumerate() {
        let name = native_name(prop, "m_name", &format!("SOLID{}", index + 1));
        solid_prop_names.push(name.clone());
        if !native_alive(prop) {
            continue;
        }
        let material_index = prop.get("m_Mindex").and_then(SdbVal::as_i32).unwrap_or(0) as usize;
        let material = mat_names
            .get(material_index)
            .filter(|name| !name.is_empty())
            .cloned()
            .ok_or_else(|| {
                FormatError::InvalidSdb(format!(
                    "Solid property {} has invalid material index {}",
                    name, material_index
                ))
            })?;
        model.solid_sections.insert(
            name.clone(),
            SolidSection {
                name,
                material,
                mat_angle_deg: native_floats(prop, "m_Matang"),
                // 1 means incompatible modes are off, as for area sections:
                // Example 5-002 "comp" files store 1 and "incomp" files 0.
                incompatible_modes: prop
                    .get("m_Incompatible")
                    .and_then(SdbVal::as_i32)
                    .unwrap_or(0)
                    == 0,
                modifiers: [1.0; 8],
            },
        );
    }
    let mut link_prop_names = vec![String::new()];
    for (index, prop) in raw_link_props.iter().enumerate() {
        let name = native_name(prop, "m_name", &format!("LINK{}", index + 1));
        link_prop_names.push(name.clone());
        if !native_alive(prop) {
            continue;
        }
        let link_type = native_link_type(
            prop.get("m_type").and_then(SdbVal::as_i32).unwrap_or(1),
        )?;
        let dof_nonlinear = native_flags(prop, "m_Nonlinear");
        let raw_param = prop
            .get("m_param")
            .and_then(SdbVal::as_arr_f64)
            .unwrap_or(&[]);
        let mut param1 = [0.0; 6];
        let mut param2 = [0.0; 6];
        // Third `m_param` row: the Wen yield exponent (Example 6-008 stores
        // yield 50, ratio 0.1 and exponent 1 for U2 in rows 1-3). It is kept
        // only for that law so other families keep their existing projection.
        let mut param3 = [0.0; 6];
        if link_type == LinkPropType::PlasticWen && raw_param.len() >= 18 {
            param3.copy_from_slice(&raw_param[12..18]);
        }
        if raw_param.len() >= 12 {
            param1.copy_from_slice(&raw_param[0..6]);
            param2.copy_from_slice(&raw_param[6..12]);
        } else if raw_param.len() >= 6 {
            param1.copy_from_slice(&raw_param[0..6]);
        }
        let nonlinear_data_complete = link_type == LinkPropType::Linear
            || (prop
                .get("m_Nonlinear")
                .and_then(SdbVal::as_arr_bool)
                .is_some_and(|v| v.len() == 6)
                && (!dof_nonlinear.iter().any(|v| *v)
                    || (matches!(
                        link_type,
                        LinkPropType::Linear
                            | LinkPropType::Gap
                            | LinkPropType::Hook
                            | LinkPropType::Damper
                    ) && raw_param.len() >= 12)
                    || (link_type == LinkPropType::PlasticWen && raw_param.len() >= 18)));
        model.link_props.insert(
            name.clone(),
            LinkProp {
                name,
                link_type,
                dof_active: native_flags(prop, "m_dof"),
                dof_fixed: native_flags(prop, "m_Fixed"),
                dof_nonlinear,
                nonlinear_data_complete,
                ke: native_floats(prop, "m_ke"),
                ce: native_floats(prop, "m_ce"),
                k_nl: native_floats(prop, "m_k"),
                param1,
                param2,
                param3,
                dj2: prop.get("m_dj2").and_then(SdbVal::as_f64).unwrap_or(0.0),
                dj3: prop.get("m_dj3").and_then(SdbVal::as_f64).unwrap_or(0.0),
                mass: prop.get("m_m").and_then(SdbVal::as_f64).unwrap_or(0.0),
                weight: prop.get("m_w").and_then(SdbVal::as_f64).unwrap_or(0.0),
            },
        );
    }

    let mut cable_prop_names = vec![String::new()];
    for (idx, cp) in raw_cable_props.iter().enumerate() {
        let name = native_name(cp, "m_name", &format!("CAB{}", idx + 1));
        cable_prop_names.push(name.clone());
        if !native_alive(cp) {
            continue;
        }
        let mindex = cp.get("m_Mindex").and_then(SdbVal::as_i32).unwrap_or(1) as usize;
        let mat_name = mat_names
            .get(mindex)
            .cloned()
            .filter(|n| !n.is_empty())
            .unwrap_or_else(|| "STEEL".to_string());
        let area = cp.get("m_Area").and_then(SdbVal::as_f64).unwrap_or(0.0);
        let t3 = if area > 0.0 {
            (4.0 * area / std::f64::consts::PI).sqrt()
        } else {
            0.0
        };
        let mut modifiers = [1.0; 8];
        if let Some(msf) = cp.get("m_msf").and_then(SdbVal::as_arr_f64) {
            for i in 0..8.min(msf.len()) {
                modifiers[i] = msf[i];
            }
        }
        model
            .frame_sections
            .entry(name.clone())
            .or_insert(FrameSection {
                name,
                material: mat_name,
                prop_type: FramePropType::Circle,
                t3,
                t2: 0.0,
                tf: 0.0,
                tw: 0.0,
                t2b: 0.0,
                tfb: 0.0,
                dis: 0.0,
                area,
                torsional_j: 0.0,
                i33: 0.0,
                i22: 0.0,
                i23: 0.0,
                as2: 0.0,
                as3: 0.0,
                s33: 0.0,
                s22: 0.0,
                z33: 0.0,
                z22: 0.0,
                r33: 0.0,
                r22: 0.0,
                modifiers,
                non_prismatic: Vec::new(),
                auto_select_list: Vec::new(),
                concrete_rebar: None,
                file_name: None,
                name_in_file: None,
            });
    }

    // Convert Points (1-based index -> Point name)
    let mut pt_names = Vec::with_capacity(raw_points.len() + 1);
    pt_names.push(String::new());
    for (idx, p) in raw_points.iter().enumerate() {
        let name = p
            .get("m_Name")
            .and_then(|v| v.as_str())
            .filter(|n| !n.is_empty())
            .map(|n| n.to_string())
            .unwrap_or_else(|| (idx + 1).to_string());
        pt_names.push(name.clone());
        let alive = p.get("m_alive").and_then(|v| v.as_bool()).unwrap_or(true);
        if !alive {
            continue;
        }
        let x = p
            .get("m_gx")
            .or_else(|| p.get("m_x"))
            .and_then(|v| v.as_f64())
            .unwrap_or(0.0);
        let y = p
            .get("m_gy")
            .or_else(|| p.get("m_y"))
            .and_then(|v| v.as_f64())
            .unwrap_or(0.0);
        let z = p
            .get("m_gz")
            .or_else(|| p.get("m_z"))
            .and_then(|v| v.as_f64())
            .unwrap_or(0.0);
        let mut pt = PointObj::new(
            name.clone(),
            if x.abs() < 1e-13 { 0.0 } else { x },
            if y.abs() < 1e-13 { 0.0 } else { y },
            if z.abs() < 1e-13 { 0.0 } else { z },
        );
        pt.local_axes_deg = [
            p.get("m_a").and_then(|v| v.as_f64()).unwrap_or(0.0),
            p.get("m_B").and_then(|v| v.as_f64()).unwrap_or(0.0),
            p.get("m_c").and_then(|v| v.as_f64()).unwrap_or(0.0),
        ];
        pt.selected = p
            .get("m_Selected")
            .and_then(|v| v.as_bool())
            .unwrap_or(false);
        model.points.insert(name, pt);
    }

    // Apply Restraints, Constraints, Joint Patterns, Springs, Masses to Points
    for res in &raw_restraints {
        if !res.get("m_alive").and_then(|v| v.as_bool()).unwrap_or(true) {
            continue;
        }
        let jn = res.get("m_jn").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        if let Some(jname) = pt_names.get(jn).filter(|s| !s.is_empty()) {
            if let (Some(pt), Some(dofs)) = (
                model.points.get_mut(jname),
                res.get("m_DOFs").and_then(|v| v.as_arr_bool()),
            ) {
                for i in 0..6.min(dofs.len()) {
                    pt.restraint[i] = dofs[i];
                }
            }
        }
    }

    let mut constraint_names = Vec::with_capacity(raw_constraints.len() + 1);
    constraint_names.push(String::new());
    for (idx, c) in raw_constraints.iter().enumerate() {
        let cname = c
            .get("m_Name")
            .and_then(|v| v.as_str())
            .filter(|n| !n.is_empty())
            .map(|n| n.to_string())
            .unwrap_or_else(|| format!("CONST{}", idx + 1));
        constraint_names.push(cname.clone());
        if !c.get("m_alive").and_then(|v| v.as_bool()).unwrap_or(true) {
            continue;
        }
        let ctype = match c.get("m_type").and_then(|v| v.as_i32()).unwrap_or(1) {
            1 => ConstraintType::Body,
            2 => ConstraintType::Diaphragm,
            3 => ConstraintType::Plate,
            4 => ConstraintType::Rod,
            5 => ConstraintType::Beam,
            6 => ConstraintType::Equal,
            7 => ConstraintType::Local,
            8 => ConstraintType::Weld,
            9 => ConstraintType::Line,
            _ => ConstraintType::Body,
        };
        let axis = match c.get("m_axis").and_then(|v| v.as_i32()).unwrap_or(3) {
            1 => ConstraintAxis::X,
            2 => ConstraintAxis::Y,
            3 => ConstraintAxis::Z,
            _ => ConstraintAxis::AutoAxis,
        };
        let mut dofs = [true; 6];
        if let Some(raw_dofs) = c.get("m_DOFs").and_then(|v| v.as_arr_i32()) {
            for i in 0..6.min(raw_dofs.len()) {
                dofs[i] = raw_dofs[i] != 0;
            }
        }
        model.constraints.insert(
            cname.clone(),
            ConstraintDef {
                name: cname,
                constraint_type: ctype,
                axis,
                dofs,
            },
        );
    }
    for cj in &raw_cjoint {
        if !cj.get("m_alive").and_then(|v| v.as_bool()).unwrap_or(true) {
            continue;
        }
        let jn = cj.get("m_jn").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        let cn = cj
            .get("m_ConstraintN")
            .and_then(|v| v.as_i32())
            .unwrap_or(0) as usize;
        if let (Some(jname), Some(cname)) = (
            pt_names.get(jn).filter(|s| !s.is_empty()),
            constraint_names.get(cn).filter(|s| !s.is_empty()),
        ) {
            if let Some(pt) = model.points.get_mut(jname) {
                if !pt.constraints.contains(cname) {
                    pt.constraints.push(cname.clone());
                }
            }
        }
    }

    let mut pat_names = Vec::with_capacity(raw_patterns.len() + 1);
    pat_names.push(String::new());
    for (idx, pat) in raw_patterns.iter().enumerate() {
        let pname = pat
            .get("m_Name")
            .and_then(|v| v.as_str())
            .filter(|n| !n.is_empty())
            .map(|n| n.to_string())
            .unwrap_or_else(|| format!("PAT{}", idx + 1));
        pat_names.push(pname);
    }
    for pj in &raw_pjoint {
        if !pj.get("m_alive").and_then(|v| v.as_bool()).unwrap_or(true) {
            continue;
        }
        let jn = pj.get("m_jn").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        let pn = pj.get("m_PatternN").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        let pval = pj.get("m_P").and_then(|v| v.as_f64()).unwrap_or(0.0);
        if let (Some(jname), Some(pname)) = (
            pt_names.get(jn).filter(|s| !s.is_empty()),
            pat_names.get(pn).filter(|s| !s.is_empty()),
        ) {
            if let Some(pt) = model.points.get_mut(jname) {
                pt.pattern_values.insert(pname.clone(), pval);
            }
        }
    }

    for sp in &raw_springs {
        if !sp.get("m_alive").and_then(|v| v.as_bool()).unwrap_or(true) {
            continue;
        }
        let jn = sp.get("m_jn").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        if let Some(jname) = pt_names.get(jn).filter(|s| !s.is_empty()) {
            let csys = sp.get("m_csys").and_then(SdbVal::as_i32).unwrap_or(1);
            let spring_is_local = if csys <= 0 {
                true
            } else if csys == 1
                && coord_names
                    .get(1)
                    .filter(|s| !s.is_empty())
                    .is_none_or(|s| s.eq_ignore_ascii_case("GLOBAL"))
            {
                false
            } else {
                let csys_name = coordinate_name(&coord_names, csys)?;
                if csys_name.eq_ignore_ascii_case("GLOBAL") {
                    false
                } else {
                    return Err(FormatError::InvalidSdb(format!(
                        "Unsupported joint spring coordinate system '{csys_name}' (m_csys={csys}) on joint '{jname}'"
                    )));
                }
            };
            if let Some(pt) = model.points.get_mut(jname) {
                pt.spring_is_local = spring_is_local;
                if let Some(xjps) = sp.get("m_XJPS").and_then(|v| v.as_arr_f64()) {
                    match xjps.len() {
                        21 => {
                            let has_off_diag = xjps.iter().enumerate().any(|(idx, &val)| {
                                !SPRING_DIAG_INDICES.contains(&idx) && val != 0.0
                            });
                            if has_off_diag {
                                let mut coupled = [0.0; 21];
                                coupled.copy_from_slice(xjps);
                                pt.spring = [0.0; 6];
                                pt.spring_coupled = Some(coupled);
                            } else {
                                for (d, &idx) in SPRING_DIAG_INDICES.iter().enumerate() {
                                    pt.spring[d] = xjps[idx];
                                }
                                pt.spring_coupled = None;
                            }
                        }
                        6 => {
                            pt.spring.copy_from_slice(xjps);
                            pt.spring_coupled = None;
                        }
                        other => {
                            return Err(FormatError::InvalidSdb(format!(
                                "Unexpected TYSpring m_XJPS length {other} on joint '{jname}' (expected 21 or 6)"
                            )));
                        }
                    }
                }
            }
        }
    }
    for ms in &raw_masses {
        if !ms.get("m_alive").and_then(|v| v.as_bool()).unwrap_or(true) {
            continue;
        }
        let jn = ms.get("m_jn").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        if let Some(jname) = pt_names.get(jn).filter(|s| !s.is_empty()) {
            if let (Some(pt), Some(xjpm)) = (
                model.points.get_mut(jname),
                ms.get("m_XJPM").and_then(|v| v.as_arr_f64()),
            ) {
                for i in 0..6.min(xjpm.len()) {
                    pt.mass[i] = xjpm[i];
                }
            }
        }
    }

    // Convert Lines / Frames (1-based index -> Frame name)
    let mut cables_meshed_at_joints: Vec<String> = Vec::new();
    let mut lines_meshed_at_crossings: Vec<String> = Vec::new();
    let mut fr_names = Vec::with_capacity(raw_lines.len() + 1);
    fr_names.push(String::new());
    for (idx, l) in raw_lines.iter().enumerate() {
        let name = l
            .get("m_Name")
            .and_then(|v| v.as_str())
            .filter(|n| !n.is_empty())
            .map(|n| n.to_string())
            .unwrap_or_else(|| (idx + 1).to_string());
        fr_names.push(name.clone());
        let alive = l.get("m_alive").and_then(|v| v.as_bool()).unwrap_or(true);
        if !alive {
            continue;
        }
        let pts = l.get("m_point").and_then(|v| v.as_arr_i32()).unwrap_or(&[]);
        let p1_idx = pts.first().copied().unwrap_or(1) as usize;
        let p2_idx = pts.get(1).copied().unwrap_or(1) as usize;
        let p1 = pt_names
            .get(p1_idx)
            .cloned()
            .unwrap_or_else(|| p1_idx.to_string());
        let p2 = pt_names
            .get(p2_idx)
            .cloned()
            .unwrap_or_else(|| p2_idx.to_string());
        let line_type = l.get("m_LineType").and_then(SdbVal::as_i32).unwrap_or(0);
        let is_cable = matches!(line_type, 1 | 2);
        // Type 3 is a tendon object (the nine corpus files with tendon loads
        // are the nine with this type). Reading it as a frame member with
        // whatever frame section its index selects (a W18X35 beam in Example
        // 1-009) would invent structure, so it is recorded and not imported.
        if !matches!(line_type, 0..=2) {
            model.unmodeled_source.push(UnmodeledSourceData {
                scope: UnmodeledScope::AllCases,
                description: if line_type == 3 {
                    format!("tendon object '{name}' (not imported)")
                } else {
                    format!("line object '{name}' of unrecognized type {line_type} (not imported)")
                },
            });
            continue;
        }
        // A mirrored insertion point swaps the sides the cardinal point is
        // counted from. No corpus line sets either flag; it is not modeled.
        if insertion_is_mirrored(l) {
            model.unmodeled_source.push(UnmodeledSourceData {
                scope: UnmodeledScope::AllCases,
                description: format!("mirrored insertion point of frame '{name}'"),
            });
        }
        let mesh = line_auto_mesh(l, &name, is_cable);
        model.unmodeled_source.extend(mesh.records);
        let (mesh_segments, mesh_at_joints) = (mesh.segments, mesh.at_joints);
        if is_cable && mesh_at_joints {
            cables_meshed_at_joints.push(name.clone());
        }
        if mesh.at_crossings {
            lines_meshed_at_crossings.push(name.clone());
        }
        let san = l.get("m_sanalysis").and_then(|v| v.as_i32()).unwrap_or(1) as usize;
        let sec_name = if is_cable {
            cable_prop_names
                .get(san)
                .filter(|s| !s.is_empty())
                .or_else(|| fsec_names.get(san).filter(|s| !s.is_empty()))
                .cloned()
                .unwrap_or_else(|| "FSEC1".to_string())
        } else {
            fsec_names
                .get(san)
                .cloned()
                .filter(|s| !s.is_empty())
                .unwrap_or_else(|| "FSEC1".to_string())
        };
        let mut fr = FrameObj::new(name.clone(), p1, p2, sec_name);
        if !is_cable {
            fr.auto_mesh_segments = mesh_segments;
            fr.auto_mesh_at_joints = mesh_at_joints;
        }
        if is_cable {
            if let Some(cd) = l.get("m_CableData").and_then(SdbVal::as_obj) {
                let model_using_frames = cd
                    .get("m_ModelUsingFrames")
                    .and_then(SdbVal::as_bool)
                    .unwrap_or(false);
                if !model_using_frames {
                    let cable_type = cd.get("m_CableType").and_then(SdbVal::as_i32).unwrap_or(0);
                    let undef_len = cd
                        .get("m_UndeformedLength")
                        .and_then(SdbVal::as_f64)
                        .filter(|&v| v > 0.0);
                    let tension_i = cd
                        .get("m_TensionI")
                        .and_then(SdbVal::as_f64)
                        .filter(|&v| v > 0.0);
                    let horiz_tension = cd
                        .get("m_HorizTensionComp")
                        .and_then(SdbVal::as_f64)
                        .filter(|&v| v > 0.0);
                    let (undeformed_length, target_tension_i, horizontal_tension) = match cable_type
                    {
                        3 => (None, tension_i, None),
                        5 => (None, None, horiz_tension),
                        8 => (undef_len, None, None),
                        _ => {
                            if undef_len.is_some() {
                                (undef_len, None, None)
                            } else if tension_i.is_some() {
                                (None, tension_i, None)
                            } else {
                                (None, None, horiz_tension)
                            }
                        }
                    };
                    fr.cable_params = Some(sap_model::CableParams {
                        undeformed_length,
                        target_tension_i,
                        horizontal_tension,
                        num_segments: cd
                            .get("m_NumSegs")
                            .and_then(SdbVal::as_i32)
                            .unwrap_or(1)
                            .max(1) as usize,
                    });
                }
            }
        }
        fr.angle_deg = l.get("m_ang").and_then(|v| v.as_f64()).unwrap_or(0.0);
        fr.end_offset_i = l.get("m_ai").and_then(|v| v.as_f64()).unwrap_or(0.0);
        fr.end_offset_j = l.get("m_aj").and_then(|v| v.as_f64()).unwrap_or(0.0);
        fr.rigid_zone_factor = l.get("m_RZ").and_then(|v| v.as_f64()).unwrap_or(0.0);
        fr.cardinal_point = l
            .get("m_CardinalPoint")
            .and_then(|v| v.as_i32())
            .unwrap_or(10);
        fr.stiff_transform = l
            .get("m_StiffTransform")
            .and_then(|v| v.as_bool())
            .unwrap_or(true);
        let lim_c_exists = l
            .get("m_LimitCompressionExists")
            .and_then(|v| v.as_bool())
            .unwrap_or(false);
        let lim_t_exists = l
            .get("m_LimitTensionExists")
            .and_then(|v| v.as_bool())
            .unwrap_or(false);
        if lim_c_exists || lim_t_exists {
            let lim_c = l
                .get("m_LimitCompression")
                .and_then(|v| v.as_f64())
                .unwrap_or(0.0);
            let lim_t = l
                .get("m_LimitTension")
                .and_then(|v| v.as_f64())
                .unwrap_or(0.0);
            fr.tc_limits = Some((lim_c_exists, lim_c, lim_t_exists, lim_t));
        }
        if let Some(ir) = l.get("m_IR").and_then(|v| v.as_arr_i32()) {
            for i in 0..6.min(ir.len()) {
                // In S_LINEOBJECTS.m_IR[0..5]: bit 0 (1) = I-end released, bit 1 (2) = J-end released
                fr.release_i[i] = (ir[i] & 1) != 0;
                fr.release_j[i] = (ir[i] & 2) != 0;
            }
        }
        model.frames.insert(name, fr);
    }

    for pdf in &raw_pd_force {
        if !pdf.get("m_alive").and_then(|v| v.as_bool()).unwrap_or(true) {
            continue;
        }
        let fn_idx = pdf.get("m_n").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        if let Some(fname) = fr_names.get(fn_idx).filter(|s| !s.is_empty()) {
            if let (Some(fr), Some(p)) = (
                model.frames.get_mut(fname),
                pdf.get("m_P").and_then(|v| v.as_f64()),
            ) {
                fr.pdelta_force = p;
            }
        }
    }
    // Insertion-point joint offsets (`FOffset`). One corpus record exists
    // (Example 1-011); see `frame_joint_offset` for what is read.
    for record in raw_foffset.iter().filter(|r| native_alive(r)) {
        let index = record.get("m_n").and_then(SdbVal::as_i32).unwrap_or(0) as usize;
        let Some(name) = fr_names.get(index).filter(|s| !s.is_empty()).cloned() else {
            continue;
        };
        match frame_joint_offset(record) {
            Ok(offset) => {
                if let Some(frame) = model.frames.get_mut(&name) {
                    frame.joint_offset_i = offset;
                    frame.joint_offset_j = offset;
                    frame.joint_offset_is_global = false;
                }
            }
            Err(reason) => model.unmodeled_source.push(UnmodeledSourceData {
                scope: UnmodeledScope::AllCases,
                description: format!("joint offsets of frame '{name}' ({reason})"),
            }),
        }
    }

    for fmod in &raw_fmodifier {
        if !fmod
            .get("m_alive")
            .and_then(|v| v.as_bool())
            .unwrap_or(true)
        {
            continue;
        }
        let fn_idx = fmod.get("m_n").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        if let Some(fname) = fr_names.get(fn_idx).filter(|s| !s.is_empty()) {
            if let (Some(fr), Some(sf)) = (
                model.frames.get_mut(fname),
                fmod.get("m_sf").and_then(|v| v.as_arr_f64()),
            ) {
                for i in 0..8.min(sf.len()) {
                    fr.modifiers[i] = sf[i];
                }
            }
        }
    }
    for ffix in &raw_ffixity {
        if !ffix
            .get("m_alive")
            .and_then(|v| v.as_bool())
            .unwrap_or(true)
        {
            continue;
        }
        let fn_idx = ffix.get("m_n").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        if let Some(fname) = fr_names.get(fn_idx).filter(|s| !s.is_empty()) {
            if let (Some(fr), Some(p)) = (
                model.frames.get_mut(fname),
                ffix.get("m_P").and_then(|v| v.as_arr_f64()),
            ) {
                if p.len() >= 12 {
                    for i in 0..6 {
                        fr.start_spring[i] = p[i * 2];
                        fr.end_spring[i] = p[i * 2 + 1];
                    }
                }
            }
        }
    }

    // Convert Areas (1-based index -> Area name)
    let mut ar_names = Vec::with_capacity(raw_areas.len() + 1);
    ar_names.push(String::new());
    for (idx, a) in raw_areas.iter().enumerate() {
        let name = a
            .get("m_name")
            .and_then(|v| v.as_str())
            .filter(|n| !n.is_empty())
            .map(|n| n.to_string())
            .unwrap_or_else(|| (idx + 1).to_string());
        ar_names.push(name.clone());
        let alive = a.get("m_alive").and_then(|v| v.as_bool()).unwrap_or(true);
        if !alive {
            continue;
        }
        let num_pts = a.get("m_numpnts").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        let raw_pts = a.get("m_point").and_then(|v| v.as_arr_i32()).unwrap_or(&[]);
        let mut pts = Vec::with_capacity(num_pts);
        for &p_idx in raw_pts.iter().take(num_pts.max(raw_pts.len())) {
            if p_idx > 0 {
                if let Some(pname) = pt_names.get(p_idx as usize).filter(|s| !s.is_empty()) {
                    pts.push(pname.clone());
                }
            }
        }
        let prop_idx = a.get("m_prop").and_then(|v| v.as_i32()).unwrap_or(1) as usize;
        let sec_name = asec_names
            .get(prop_idx)
            .cloned()
            .filter(|s| !s.is_empty())
            .unwrap_or_else(|| "ASEC1".to_string());
        let mut ar = AreaObj::new(name.clone(), pts, sec_name);
        ar.angle_deg = a.get("m_Ang").and_then(|v| v.as_f64()).unwrap_or(0.0);
        let n1 = a
            .get("m_meshn1")
            .and_then(|v| v.as_i32())
            .unwrap_or(1)
            .max(1) as usize;
        let n2 = a
            .get("m_meshn2")
            .and_then(|v| v.as_i32())
            .unwrap_or(1)
            .max(1) as usize;
        ar.auto_mesh = (n1, n2);
        model.areas.insert(name, ar);
    }

    let mut solid_names = vec![String::new()];
    for (index, solid) in raw_solids.iter().enumerate() {
        let name = native_name(solid, "m_name", &(index + 1).to_string());
        solid_names.push(name.clone());
        if !native_alive(solid) {
            continue;
        }
        let raw_points = solid
            .get("m_point")
            .and_then(SdbVal::as_arr_i32)
            .unwrap_or(&[]);
        if raw_points.len() < 8 {
            return Err(FormatError::InvalidSdb(format!(
                "Solid {} needs eight native point indices",
                name
            )));
        }
        let mut points = std::array::from_fn(|_| String::new());
        for (out, index) in points.iter_mut().zip(raw_points) {
            *out = pt_names
                .get(*index as usize)
                .filter(|n| model.points.contains_key(*n))
                .cloned()
                .ok_or_else(|| {
                    FormatError::InvalidSdb(format!(
                        "Solid {} has invalid point index {}",
                        name, index
                    ))
                })?;
        }
        let prop_index = solid.get("m_prop").and_then(SdbVal::as_i32).unwrap_or(0) as usize;
        let section = solid_prop_names
            .get(prop_index)
            .filter(|n| model.solid_sections.contains_key(*n))
            .cloned()
            .ok_or_else(|| {
                FormatError::InvalidSdb(format!(
                    "Solid {} has invalid property index {}",
                    name, prop_index
                ))
            })?;
        model.solids.insert(
            name.clone(),
            SolidObj {
                name,
                points,
                section,
                loads_surface: Vec::new(),
                loads_temp: std::collections::BTreeMap::new(),
                selected: solid
                    .get("m_Selected")
                    .and_then(SdbVal::as_bool)
                    .unwrap_or(false),
            },
        );
    }
    let mut link_names = vec![String::new()];
    for (index, link) in raw_links.iter().enumerate() {
        let name = native_name(link, "m_name", &(index + 1).to_string());
        link_names.push(if native_alive(link) {
            name.clone()
        } else {
            String::new()
        });
        if !native_alive(link) {
            continue;
        }
        // Example 6-008 orients its link with this flag and plane/axis vectors
        // that are not interpreted; only the roll angle is imported.
        if link.get("m_UserAxes").and_then(SdbVal::as_bool) == Some(true) {
            model.unmodeled_source.push(UnmodeledSourceData {
                scope: UnmodeledScope::AllCases,
                description: format!(
                    "advanced local axes of link '{name}' (only the roll angle is imported)"
                ),
            });
        }
        let raw_points = link
            .get("m_point")
            .and_then(SdbVal::as_arr_i32)
            .unwrap_or(&[]);
        let p_i_raw = raw_points.first().copied().unwrap_or(0);
        let p_j_raw = raw_points.get(1).copied().unwrap_or(0);
        let point_i = pt_names
            .get(p_i_raw as usize)
            .filter(|n| model.points.contains_key(*n))
            .cloned()
            .ok_or_else(|| FormatError::InvalidSdb(format!("Link {} has invalid I-point", name)))?;
        let is_single_joint = link
            .get("m_SingleJoint")
            .and_then(SdbVal::as_bool)
            .unwrap_or(false)
            || p_j_raw == 0
            || p_j_raw == p_i_raw;
        let point_j = if is_single_joint {
            None
        } else {
            Some(
                pt_names
                    .get(p_j_raw as usize)
                    .filter(|n| model.points.contains_key(*n))
                    .cloned()
                    .ok_or_else(|| {
                        FormatError::InvalidSdb(format!("Link {} has invalid J-point", name))
                    })?,
            )
        };
        let prop_index = link.get("m_NPR").and_then(SdbVal::as_i32).unwrap_or(0) as usize;
        let prop = link_prop_names
            .get(prop_index)
            .filter(|n| model.link_props.contains_key(*n))
            .cloned()
            .ok_or_else(|| {
                FormatError::InvalidSdb(format!(
                    "Link {} has invalid property index {}",
                    name, prop_index
                ))
            })?;
        model.links.insert(
            name.clone(),
            LinkObj {
                name,
                point_i,
                point_j,
                prop,
                angle_deg: link.get("m_Ang").and_then(SdbVal::as_f64).unwrap_or(0.0),
                selected: link
                    .get("m_Selected")
                    .and_then(SdbVal::as_bool)
                    .unwrap_or(false),
            },
        );
    }
    let mut function_names = vec![String::new()];
    for (index, function) in raw_functions.iter().enumerate() {
        let name = native_name(function, "m_Name", &format!("FUNC{}", index + 1));
        function_names.push(name.clone());
        if !native_alive(function) {
            continue;
        }
        let function_type = function.get("m_type").and_then(SdbVal::as_i32).unwrap_or(0);
        if !matches!(function_type, 1 | 2) {
            continue;
        }
        let mut points: Vec<(f64, f64)> = raw_function_values
            .iter()
            .filter(|point| {
                point.get("m_fnumber").and_then(SdbVal::as_i32) == Some((index + 1) as i32)
            })
            .filter_map(|point| {
                Some((
                    point.get("m_time")?.as_f64()?,
                    point.get("m_Value")?.as_f64()?,
                ))
            })
            .collect();
        if points.is_empty() && function_type == 2 {
            let fname = function
                .get("m_fname")
                .and_then(SdbVal::as_str)
                .unwrap_or("");
            let sine_p = function.get("m_SineP").and_then(SdbVal::as_f64).unwrap_or(0.0);
            let sine_steps = function.get("m_SineSteps").and_then(SdbVal::as_i32).unwrap_or(0);
            let sine_cycles = function.get("m_SineCycles").and_then(SdbVal::as_i32).unwrap_or(0);
            let sine_amp = function.get("m_SineAmp").and_then(SdbVal::as_f64).unwrap_or(0.0);
            if fname.is_empty() && sine_p > 0.0 && sine_steps > 0 && sine_cycles > 0 && sine_amp != 0.0
            {
                let steps_per_cycle = (sine_steps as usize).max(64);
                let total_steps = (sine_cycles as usize) * steps_per_cycle;
                let dt = sine_p / (steps_per_cycle as f64);
                for k in 0..=total_steps {
                    let t = (k as f64) * dt;
                    let phase =
                        2.0 * std::f64::consts::PI * (k as f64) / (steps_per_cycle as f64);
                    points.push((t, sine_amp * phase.sin()));
                }
            }
        }
        model.functions.insert(
            name.clone(),
            DynamicFunction {
                name,
                is_time_history: function_type == 2,
                damping_ratio: function
                    .get("m_dampratio")
                    .and_then(SdbVal::as_f64)
                    .unwrap_or(0.0),
                points,
            },
        );
    }

    // Convert Load Patterns (1-based index -> LoadPattern name)
    let mut lpat_names = Vec::with_capacity(raw_load_patterns.len() + 1);
    lpat_names.push(String::new());
    for (idx, lp) in raw_load_patterns.iter().enumerate() {
        let name = lp
            .get("m_Name")
            .and_then(|v| v.as_str())
            .filter(|n| !n.is_empty())
            .map(|n| n.to_string())
            .unwrap_or_else(|| (idx + 1).to_string());
        lpat_names.push(name.clone());
        if !lp.get("m_alive").and_then(|v| v.as_bool()).unwrap_or(true) {
            continue;
        }
        let ptype = lp.get("m_type").and_then(|v| v.as_i32()).unwrap_or(1);
        let sw = lp
            .get("m_SelfWTMultiplier")
            .and_then(|v| v.as_f64())
            .unwrap_or(0.0);
        model.load_patterns.insert(
            name.clone(),
            LoadPattern {
                name,
                pat_type: map_load_pat_type(ptype),
                self_wt_multiplier: sw,
            },
        );
    }

    // Internal native case codes differ from OAPI eLoadCaseType.
    let ac_names: Vec<String> = std::iter::once(String::new())
        .chain(
            raw_analysis_cases
                .iter()
                .enumerate()
                .map(|(index, ac)| native_name(ac, "m_name", &(index + 1).to_string())),
        )
        .collect();
    for (idx, ac) in raw_analysis_cases.iter().enumerate() {
        let name = ac_names[idx + 1].clone();
        if !native_alive(ac) {
            continue;
        }
        let ctype_code = ac.get("m_casetype").and_then(SdbVal::as_i32).unwrap_or(1);
        let case_type = native_case_type(ctype_code)?;
        let mut lc = LoadCase::linear_static(name.clone(), name.clone(), 1.0);
        lc.case_type = case_type;
        lc.loads.clear();
        lc.run_flag = ac
            .get("m_SelectedForRun")
            .and_then(|v| v.as_bool())
            .unwrap_or(true);
        let prev_idx = ac.get("m_previous").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        if prev_idx > 0 {
            if let Some(pname) = ac_names.get(prev_idx).filter(|s| !s.is_empty()) {
                lc.initial_case = Some(pname.clone());
            }
        }
        if let Some(las) = ac.get("m_LA").and_then(SdbVal::as_arr_obj) {
            for la in las {
                let load_idx = la.get("m_load").and_then(SdbVal::as_i32).unwrap_or(0) as usize;
                let function_idx = la.get("m_Func").and_then(SdbVal::as_i32).unwrap_or(0) as usize;
                let load_type = la.get("m_ltype").and_then(SdbVal::as_i32).unwrap_or(1);
                let sf = la.get("m_sf").and_then(SdbVal::as_f64).unwrap_or(1.0);
                let load_name = if load_type == 2 && (1..=6).contains(&load_idx) {
                    Some(format!("U{}", load_idx))
                } else if load_type == 1 {
                    lpat_names.get(load_idx).filter(|n| !n.is_empty()).cloned()
                } else {
                    None
                };
                let function = function_names
                    .get(function_idx)
                    .filter(|n| model.functions.contains_key(*n));
                if matches!(ctype_code, 4..=8) && function_idx > 0 {
                    check_native_dynamic_assignment(&name, ctype_code, la)?;
                }
                if matches!(ctype_code, 4) {
                    if let (Some(name), Some(direction)) = (function, load_name) {
                        lc.spectrum_loads.push((name.clone(), direction, sf));
                    }
                } else if matches!(ctype_code, 5..=8) {
                    if let (Some(function), Some(load_name)) = (function, load_name) {
                        lc.history_loads.push((function.clone(), load_name, sf));
                    }
                } else if let Some(load_name) = load_name {
                    lc.loads.push(LoadCaseAssignment {
                        load_name,
                        scale_factor: sf,
                    });
                }
            }
        }
        lc.modal_damping = ac.get("m_Damp").and_then(SdbVal::as_f64).unwrap_or(0.05);
        if matches!(ctype_code, 5 | 6) {
            // Modal history damping. Type 4 is a constant ratio in m_Damp. Types 1
            // and 2 are mass/stiffness proportional: Example 6-011a stores type 2
            // with f1 = 1, f2 = 10, ratio 0.0059 and exactly the Rayleigh
            // coefficients those give in m_dd, which are imported below. Their
            // m_Damp is then not the active damping.
            if matches!(ac.get("m_DampType").and_then(SdbVal::as_i32), Some(1 | 2)) {
                lc.modal_damping = 0.0;
            }
            let count = |key: &str| ac.get(key).and_then(SdbVal::as_i32).unwrap_or(0);
            if count("m_numberMDs") > 0 || count("m_numberFDs") > 0 {
                model.unmodeled_source.push(UnmodeledSourceData {
                    scope: UnmodeledScope::Case(name.clone()),
                    description: format!(
                        "per-mode or interpolated modal damping of case '{name}'"
                    ),
                });
            }
        }
        lc.modal_combo = match ac.get("m_modc").and_then(SdbVal::as_i32).unwrap_or(1) {
            2 => ModalComboMethod::Srss,
            3 => ModalComboMethod::AbsSum,
            _ => ModalComboMethod::Cqc,
        };
        if let Some(sc) = ac.get("m_SolControl").and_then(|v| v.as_obj()) {
            // Nonlinear static load application: m_control 2 is displacement
            // control (Example 6-008 stores control 2, target 10 and one
            // monitored joint record for its "push to 10 in" case).
            if ctype_code == 2 && sc.get("m_control").and_then(SdbVal::as_i32) == Some(2) {
                let monitored = match ac.get("m_djoints") {
                    Some(SdbVal::ArrObj(items)) if items.len() == 1 => {
                        let item = &items[0];
                        let joint = item.get("m_jn").and_then(SdbVal::as_i32).unwrap_or(0);
                        let dof = item.get("m_dof").and_then(SdbVal::as_i32).unwrap_or(0);
                        let plain_joint = item.get("m_DType").and_then(SdbVal::as_i32) == Some(1)
                            && item.get("m_sf").and_then(SdbVal::as_f64) == Some(1.0);
                        usize::try_from(joint)
                            .ok()
                            .and_then(|index| pt_names.get(index))
                            .filter(|name| !name.is_empty() && plain_joint && (1..=6).contains(&dof))
                            .map(|name| (name.clone(), (dof - 1) as usize))
                    }
                    _ => None,
                };
                lc.displacement_control = Some(sap_model::DisplacementControl {
                    target: sc.get("m_TargetDisp").and_then(SdbVal::as_f64).unwrap_or(0.0),
                    monitored,
                    // 1 is the value every default case carries; the vendor
                    // default is conjugate-displacement stepping.
                    conjugate: sc.get("m_DispType").and_then(SdbVal::as_i32) != Some(2),
                });
            }
            if let Some(steps) = sc.get("m_nstep").and_then(SdbVal::as_i32) {
                lc.num_time_steps = steps.max(0) as usize;
            }
            if let Some(dt) = sc.get("m_DT").and_then(SdbVal::as_f64) {
                lc.time_step_dt = dt;
            }
            if let Some(mx) = sc.get("m_MaxModes").and_then(|v| v.as_i32()) {
                lc.max_modes = mx.max(1) as usize;
            }
            if let Some(mn) = sc.get("m_MinModes").and_then(|v| v.as_i32()) {
                lc.min_modes = mn.max(1) as usize;
            }
            if let Some(mt) = sc.get("m_ModeType").and_then(|v| v.as_i32()) {
                lc.modal_sub_type = if mt == 2 {
                    ModalSubType::Ritz
                } else {
                    ModalSubType::Eigen
                };
            }
            if let Some(sh) = sc.get("m_EigenShiftFreq").and_then(|v| v.as_f64()) {
                lc.shift_freq_hz = sh;
            }
            if let Some(nb) = sc.get("m_NumBucklingModes").and_then(|v| v.as_i32()) {
                lc.num_buckling_modes = nb.max(1) as usize;
            }
            if let Some(itmax) = sc.get("m_itmax").and_then(SdbVal::as_i32).filter(|&v| v > 0) {
                lc.max_iterations = itmax as usize;
            }
            if let Some(ittol) = sc
                .get("m_ittol")
                .or_else(|| sc.get("m_TolConvD"))
                .and_then(SdbVal::as_f64)
                .filter(|&v| v > 0.0)
            {
                lc.convergence_tol = ittol;
            }
        }
        if let Some(intg) = ac.get("m_Integration").and_then(|v| v.as_obj()) {
            let am = intg.get("m_am").and_then(SdbVal::as_f64).unwrap_or(0.0);
            let bk = intg.get("m_bk").and_then(SdbVal::as_f64).unwrap_or(0.0);
            if am != 0.0 || bk != 0.0 {
                lc.rayleigh_damping = (am, bk);
            }
        }
        if let Some(dd) = ac.get("m_dd").and_then(|v| v.as_obj()) {
            let am = dd.get("m_Dampa").and_then(SdbVal::as_f64).unwrap_or(0.0);
            let bk = dd.get("m_Dampb").and_then(SdbVal::as_f64).unwrap_or(0.0);
            if am != 0.0 || bk != 0.0 {
                lc.rayleigh_damping = (am, bk);
            }
        }
        if let Some(nl) = ac.get("m_Nonlinearity").and_then(|v| v.as_obj()) {
            if let Some(nlg) = nl.get("m_nlgeomtype").and_then(|v| v.as_i32()) {
                lc.nl_geom = match nlg {
                    1 => NLGeomType::PDelta,
                    2 => NLGeomType::LargeDisplacements,
                    _ => NLGeomType::None,
                };
            }
            if let Some(itmax) = nl.get("m_itmax").and_then(SdbVal::as_i32).filter(|&v| v > 0) {
                lc.max_iterations = itmax as usize;
            }
            if let Some(ittol) = nl.get("m_ittol").and_then(SdbVal::as_f64).filter(|&v| v > 0.0) {
                lc.convergence_tol = ittol;
            }
        }
        model.load_cases.insert(name, lc);
    }

    // Convert Load Combos (1-based index -> LoadCombo name)
    let mut combo_names = Vec::with_capacity(raw_combos.len() + 1);
    combo_names.push(String::new());
    for (idx, cb) in raw_combos.iter().enumerate() {
        let name = cb
            .get("m_Name")
            .and_then(|v| v.as_str())
            .filter(|n| !n.is_empty())
            .map(|n| n.to_string())
            .unwrap_or_else(|| format!("COMBO{}", idx + 1));
        combo_names.push(name);
    }
    for (idx, cb) in raw_combos.iter().enumerate() {
        if !cb.get("m_alive").and_then(|v| v.as_bool()).unwrap_or(true) {
            continue;
        }
        let name = combo_names[idx + 1].clone();
        if name.is_empty() {
            continue;
        }
        let ctype = match cb.get("m_type").and_then(|v| v.as_i32()).unwrap_or(0) {
            1 => ComboType::Envelope,
            2 => ComboType::AbsAdd,
            3 => ComboType::Srss,
            4 => ComboType::RangeAdd,
            _ => ComboType::LinearAdd,
        };
        let mut cases = Vec::new();
        if let Some(cls) = cb.get("m_CL").and_then(|v| v.as_arr_obj()) {
            for cl in cls {
                let cl_type = cl.get("m_CType").and_then(|v| v.as_i32()).unwrap_or(0);
                let ncase = cl.get("m_nCase").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
                let sf = cl.get("m_sf").and_then(|v| v.as_f64()).unwrap_or(1.0);
                let ref_name = if cl_type == 1 {
                    combo_names
                        .get(ncase)
                        .filter(|s| !s.is_empty())
                        .or_else(|| ac_names.get(ncase).filter(|s| !s.is_empty()))
                } else {
                    ac_names
                        .get(ncase)
                        .filter(|s| !s.is_empty())
                        .or_else(|| combo_names.get(ncase).filter(|s| !s.is_empty()))
                };
                if let Some(cname) = ref_name {
                    cases.push((cname.clone(), sf));
                }
            }
        }
        model.load_combos.insert(
            name.clone(),
            LoadCombo {
                name,
                combo_type: ctype,
                cases,
            },
        );
    }

    // Apply Joint Loads (JFLoad & JGDLoad)
    for jf in &raw_jfload {
        if !jf.get("m_alive").and_then(|v| v.as_bool()).unwrap_or(true) {
            continue;
        }
        let jn = jf.get("m_jn").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        let lc = jf.get("m_LC").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        if let (Some(jname), Some(lpat)) = (
            pt_names.get(jn).filter(|s| !s.is_empty()),
            lpat_names.get(lc).filter(|s| !s.is_empty()),
        ) {
            if let (Some(pt), Some(f)) = (
                model.points.get_mut(jname),
                jf.get("m_F").and_then(|v| v.as_arr_f64()),
            ) {
                let mut force = [0.0; 6];
                for i in 0..6.min(f.len()) {
                    force[i] = f[i];
                }
                let csys_idx = jf.get("m_csys").and_then(|v| v.as_i32()).unwrap_or(1);
                pt.loads_force.push(PointLoadForce {
                    load_pat: lpat.clone(),
                    csys: coordinate_name(&coord_names, csys_idx)?,
                    force,
                });
            }
        }
    }
    for jgd in &raw_jgdload {
        if !jgd.get("m_alive").and_then(|v| v.as_bool()).unwrap_or(true) {
            continue;
        }
        let jn = jgd.get("m_jn").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        let lc = jgd.get("m_LC").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        if let (Some(jname), Some(lpat)) = (
            pt_names.get(jn).filter(|s| !s.is_empty()),
            lpat_names.get(lc).filter(|s| !s.is_empty()),
        ) {
            if let (Some(pt), Some(d)) = (
                model.points.get_mut(jname),
                jgd.get("m_D")
                    .or_else(|| jgd.get("m_U"))
                    .or_else(|| jgd.get("m_d"))
                    .and_then(|v| v.as_arr_f64()),
            ) {
                let mut displ = [0.0; 6];
                for i in 0..6.min(d.len()) {
                    displ[i] = d[i];
                }
                let csys_idx = jgd.get("m_csys").and_then(|v| v.as_i32()).unwrap_or(1);
                pt.loads_displ.push(PointLoadDispl {
                    load_pat: lpat.clone(),
                    csys: coordinate_name(&coord_names, csys_idx)?,
                    displ,
                });
            }
        }
    }

    // Apply Frame Gravity, Distributed, Concentrated & Temperature Loads
    for fg in &raw_fgload {
        if !fg.get("m_alive").and_then(|v| v.as_bool()).unwrap_or(true) {
            continue;
        }
        let fn_idx = fg.get("m_n").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        let lc = fg.get("m_lc").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        if let (Some(fname), Some(lpat)) = (
            fr_names.get(fn_idx).filter(|s| !s.is_empty()),
            lpat_names.get(lc).filter(|s| !s.is_empty()),
        ) {
            if let Some(fr) = model.frames.get_mut(fname) {
                let ux = fg.get("m_Ux").and_then(|v| v.as_f64()).unwrap_or(0.0);
                let uy = fg.get("m_Uy").and_then(|v| v.as_f64()).unwrap_or(0.0);
                let uz = fg.get("m_UZ").and_then(|v| v.as_f64()).unwrap_or(0.0);
                fr.loads_gravity.insert(lpat.clone(), [ux, uy, uz]);
            }
        }
    }
    for load in &raw_nlgload {
        if !native_alive(load) {
            continue;
        }
        let multiplier = |key: &str| load.get(key).and_then(SdbVal::as_f64).unwrap_or(0.0);
        if ["m_Ux", "m_Uy", "m_UZ"].iter().all(|key| multiplier(key) == 0.0) {
            continue;
        }
        let index = |key: &str| load.get(key).and_then(SdbVal::as_i32).unwrap_or(0) as usize;
        let link = link_names.get(index("m_n")).filter(|n| !n.is_empty());
        let pattern = lpat_names.get(index("m_lc")).filter(|n| !n.is_empty());
        model.unmodeled_source.push(match (link, pattern) {
            (Some(link), Some(pattern)) => UnmodeledSourceData {
                scope: UnmodeledScope::LoadPattern(pattern.clone()),
                description: format!(
                    "gravity load on link '{link}' in load pattern '{pattern}'"
                ),
            },
            // An unresolved reference cannot be scoped; refuse everything.
            _ => UnmodeledSourceData {
                scope: UnmodeledScope::AllCases,
                description: "a link gravity load with an unresolved link or load pattern".into(),
            },
        });
    }
    for fd in &raw_fdload {
        if !fd.get("m_alive").and_then(|v| v.as_bool()).unwrap_or(true) {
            continue;
        }
        let fn_idx = fd.get("m_n").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        let lc = fd.get("m_lc").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        if let (Some(fname), Some(lpat)) = (
            fr_names.get(fn_idx).filter(|s| !s.is_empty()),
            lpat_names.get(lc).filter(|s| !s.is_empty()),
        ) {
            if let Some(fr) = model.frames.get_mut(fname) {
                let csys_idx = fd.get("m_csys").and_then(|v| v.as_i32()).unwrap_or(1);
                let kd = fd.get("m_Kd").and_then(|v| v.as_i32()).unwrap_or(10);
                fr.loads_dist.push(FrameLoadDistributed {
                    load_pat: lpat.clone(),
                    my_type: fd.get("m_kt").and_then(|v| v.as_i32()).unwrap_or(1),
                    dir: kd,
                    csys: coordinate_name(&coord_names, csys_idx)?,
                    rel_dist1: fd.get("m_RDA").and_then(|v| v.as_f64()).unwrap_or(0.0),
                    rel_dist2: fd.get("m_RDB").and_then(|v| v.as_f64()).unwrap_or(1.0),
                    val1: fd.get("m_UKA").and_then(|v| v.as_f64()).unwrap_or(0.0),
                    val2: fd.get("m_UKB").and_then(|v| v.as_f64()).unwrap_or(0.0),
                });
            }
        }
    }
    for fc in &raw_fcload {
        if !fc.get("m_alive").and_then(|v| v.as_bool()).unwrap_or(true) {
            continue;
        }
        let fn_idx = fc.get("m_n").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        let lc = fc.get("m_lc").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        if let (Some(fname), Some(lpat)) = (
            fr_names.get(fn_idx).filter(|s| !s.is_empty()),
            lpat_names.get(lc).filter(|s| !s.is_empty()),
        ) {
            if let Some(fr) = model.frames.get_mut(fname) {
                let csys_idx = fc.get("m_csys").and_then(|v| v.as_i32()).unwrap_or(1);
                let kd = fc.get("m_Kd").and_then(|v| v.as_i32()).unwrap_or(10);
                fr.loads_point.push(FrameLoadPoint {
                    load_pat: lpat.clone(),
                    my_type: fc.get("m_kt").and_then(|v| v.as_i32()).unwrap_or(1),
                    dir: kd,
                    csys: coordinate_name(&coord_names, csys_idx)?,
                    rel_dist: fc.get("m_RD").and_then(|v| v.as_f64()).unwrap_or(0.5),
                    val: fc.get("m_UK").and_then(|v| v.as_f64()).unwrap_or(0.0),
                });
            }
        }
    }
    for ft in &raw_ftload {
        if !ft.get("m_alive").and_then(|v| v.as_bool()).unwrap_or(true) {
            continue;
        }
        let fn_idx = ft.get("m_n").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        let lc = ft.get("m_lc").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        if let (Some(fname), Some(lpat)) = (
            fr_names.get(fn_idx).filter(|s| !s.is_empty()),
            lpat_names.get(lc).filter(|s| !s.is_empty()),
        ) {
            if let Some(fr) = model.frames.get_mut(fname) {
                let lt = ft.get("m_LT").and_then(|v| v.as_i32()).unwrap_or(1);
                let tval = ft.get("m_T").and_then(|v| v.as_f64()).unwrap_or(0.0);
                let pat_idx = ft.get("m_Pattern").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
                let pattern_name = if pat_idx > 0 {
                    pat_names.get(pat_idx).filter(|s| !s.is_empty()).cloned()
                } else {
                    None
                };
                fr.loads_temp.push(FrameLoadTemperature {
                    load_pat: lpat.clone(),
                    my_type: lt,
                    val: tval,
                    pattern_name,
                });
            }
        }
    }

    // Apply Area Uniform Loads (ShULoad)
    for shu in &raw_shuload {
        if !shu.get("m_alive").and_then(|v| v.as_bool()).unwrap_or(true) {
            continue;
        }
        let an_idx = shu.get("m_n").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        let lc = shu.get("m_LC").and_then(|v| v.as_i32()).unwrap_or(0) as usize;
        if let (Some(aname), Some(lpat)) = (
            ar_names.get(an_idx).filter(|s| !s.is_empty()),
            lpat_names.get(lc).filter(|s| !s.is_empty()),
        ) {
            if let Some(ar) = model.areas.get_mut(aname) {
                let csys_idx = shu.get("m_csys").and_then(|v| v.as_i32()).unwrap_or(1);
                ar.loads_uniform.push(AreaLoadUniform {
                    load_pat: lpat.clone(),
                    csys: coordinate_name(&coord_names, csys_idx)?,
                    dir: shu.get("m_Kd").and_then(|v| v.as_i32()).unwrap_or(10),
                    val: shu.get("m_P").and_then(|v| v.as_f64()).unwrap_or(0.0),
                });
            }
        }
    }

    for load in &raw_shspload {
        if !native_alive(load) {
            continue;
        }
        let index = load.get("m_n").and_then(SdbVal::as_i32).unwrap_or(0) as usize;
        let pattern = load.get("m_LC").and_then(SdbVal::as_i32).unwrap_or(0) as usize;
        if let (Some(area), Some(load_pat)) = (
            ar_names
                .get(index)
                .and_then(|name| model.areas.get_mut(name)),
            lpat_names.get(pattern).filter(|name| !name.is_empty()),
        ) {
            let pindex = load.get("m_Pattern").and_then(SdbVal::as_i32).unwrap_or(0) as usize;
            area.loads_pressure.push(AreaLoadSurfacePressure {
                load_pat: load_pat.clone(),
                face: load.get("m_Face").and_then(SdbVal::as_i32).unwrap_or(-1),
                val: load.get("m_P").and_then(SdbVal::as_f64).unwrap_or(0.0),
                pattern_name: pat_names
                    .get(pindex)
                    .filter(|name| !name.is_empty())
                    .cloned(),
            });
        }
    }
    for load in &raw_shtload {
        if !native_alive(load) {
            continue;
        }
        let index = load.get("m_n").and_then(SdbVal::as_i32).unwrap_or(0) as usize;
        let pattern = load.get("m_lc").and_then(SdbVal::as_i32).unwrap_or(0) as usize;
        if let (Some(area), Some(load_pat)) = (
            ar_names
                .get(index)
                .and_then(|name| model.areas.get_mut(name)),
            lpat_names.get(pattern).filter(|name| !name.is_empty()),
        ) {
            let pindex = load.get("m_Pattern").and_then(SdbVal::as_i32).unwrap_or(0) as usize;
            area.loads_temp.push(AreaLoadTemperature {
                load_pat: load_pat.clone(),
                my_type: load.get("m_LT").and_then(SdbVal::as_i32).unwrap_or(1),
                val: load.get("m_T").and_then(SdbVal::as_f64).unwrap_or(0.0),
                pattern_name: pat_names
                    .get(pindex)
                    .filter(|name| !name.is_empty())
                    .cloned(),
            });
        }
    }
    for load in &raw_sospload {
        if !native_alive(load) || load.get("m_Pattern").and_then(SdbVal::as_i32).unwrap_or(0) != 0 {
            continue;
        }
        let index = load.get("m_n").and_then(SdbVal::as_i32).unwrap_or(0) as usize;
        let pattern = load.get("m_LC").and_then(SdbVal::as_i32).unwrap_or(0) as usize;
        if let (Some(solid), Some(load_pat)) = (
            solid_names
                .get(index)
                .and_then(|name| model.solids.get_mut(name)),
            lpat_names.get(pattern).filter(|name| !name.is_empty()),
        ) {
            solid.loads_surface.push(SolidSurfaceLoad {
                load_pat: load_pat.clone(),
                face: load.get("m_Face").and_then(SdbVal::as_i32).unwrap_or(1),
                val: load.get("m_P").and_then(SdbVal::as_f64).unwrap_or(0.0),
            });
        }
    }
    for load in &raw_sotload {
        if !native_alive(load)
            || load.get("m_Pattern").and_then(SdbVal::as_i32).unwrap_or(0) != 0
            || load.get("m_LT").and_then(SdbVal::as_i32).unwrap_or(1) != 1
        {
            continue;
        }
        let index = load.get("m_n").and_then(SdbVal::as_i32).unwrap_or(0) as usize;
        let pattern = load.get("m_lc").and_then(SdbVal::as_i32).unwrap_or(0) as usize;
        if let (Some(solid), Some(load_pat)) = (
            solid_names
                .get(index)
                .and_then(|name| model.solids.get_mut(name)),
            lpat_names.get(pattern).filter(|name| !name.is_empty()),
        ) {
            *solid.loads_temp.entry(load_pat.clone()).or_default() +=
                load.get("m_T").and_then(SdbVal::as_f64).unwrap_or(0.0);
        }
    }

    // Object-level area assignments: property modifiers, added mass and the
    // N x N auto-mesh. They live in their own sections, keyed by area index.
    let assigned_area = |record: &HashMap<String, SdbVal>| -> Option<String> {
        let index = record.get("m_n").and_then(SdbVal::as_i32).unwrap_or(0);
        usize::try_from(index)
            .ok()
            .and_then(|index| ar_names.get(index))
            .filter(|name| !name.is_empty())
            .cloned()
    };
    let unassigned = |what: &str| UnmodeledSourceData {
        scope: UnmodeledScope::AllCases,
        description: format!("an area {what} assignment that could not be read"),
    };
    for record in raw_shmodifier.iter().filter(|r| native_alive(r)) {
        let factors = record.get("m_sf").and_then(SdbVal::as_arr_f64);
        match (
            assigned_area(record).and_then(|name| model.areas.get_mut(&name)),
            factors,
        ) {
            (Some(area), Some(factors)) if factors.len() == 10 => {
                area.modifiers.copy_from_slice(factors);
            }
            _ => model.unmodeled_source.push(unassigned("property-modifier")),
        }
    }
    for record in raw_shmass.iter().filter(|r| native_alive(r)) {
        let mass = record.get("m_mpa").and_then(SdbVal::as_f64);
        match (
            assigned_area(record).and_then(|name| model.areas.get_mut(&name)),
            mass,
        ) {
            (Some(area), Some(mass)) if mass.is_finite() && mass >= 0.0 => {
                area.mass_per_area = mass;
            }
            _ => model.unmodeled_source.push(unassigned("added-mass")),
        }
    }
    for record in raw_shmesh.iter().filter(|r| native_alive(r)) {
        let Some(name) = assigned_area(record).filter(|n| model.areas.contains_key(n)) else {
            model.unmodeled_source.push(unassigned("auto-mesh"));
            continue;
        };
        let integer = |key: &str| record.get(key).and_then(SdbVal::as_i32).unwrap_or(0);
        let flag = |key: &str| record.get(key).and_then(SdbVal::as_bool).unwrap_or(false);
        let (mesh_type, n1, n2) = (integer("m_MeshType"), integer("m_n1"), integer("m_n2"));
        let corners: Vec<[bool; 6]> = model.areas[&name]
            .points
            .iter()
            .filter_map(|point| model.points.get(point))
            .map(|point| point.restraint)
            .collect();
        // The solver restrains a mesh joint on an edge in the DOFs restrained
        // at both ends of that edge, and an interior joint in the DOFs
        // restrained at all four corners. That equals the vendor options only
        // when they are on, or when there is nothing for them to act on.
        let shared = |a: usize, b: usize| (0..6).any(|d| corners[a][d] && corners[b][d]);
        let representable = mesh_type == 1
            // Unequal counts need the vendor edge order, which no corpus file
            // with restraints on edges exercises.
            && n1 == n2
            && n1 >= 1
            && corners.len() == 4
            && !flag("m_PointOnEdgeFromLine")
            && !flag("m_PointOnEdgeFromPoint")
            && !flag("m_ExtendCookieCutLines")
            && !flag("m_SubMesh")
            && !flag("m_LocalAxesOnEdge")
            && !flag("m_LocalAxesOnFace")
            && (flag("m_RestraintsOnEdge") || !(0..4).any(|c| shared(c, (c + 1) % 4)))
            && (flag("m_RestraintsOnFace")
                || !(0..6).any(|d| corners.iter().all(|corner| corner[d])));
        if representable {
            if let Some(area) = model.areas.get_mut(&name) {
                area.auto_mesh = (n1 as usize, n2 as usize);
            }
        } else {
            model.unmodeled_source.push(UnmodeledSourceData {
                scope: UnmodeledScope::AllCases,
                description: format!(
                    "auto-mesh assignment of area '{name}' ({n1} x {n2}, type {mesh_type}) with options that are not modeled"
                ),
            });
        }
    }

    // A cable is solved between its end joints only. If it is flagged for
    // subdivision at joints and a joint lies on its chord, the source connects
    // them and this model does not.
    for name in &cables_meshed_at_joints {
        if let Some(joint) = model.frames.get(name).and_then(|f| joint_on_chord(&model, f)) {
            model.unmodeled_source.push(UnmodeledSourceData {
                scope: UnmodeledScope::AllCases,
                description: format!(
                    "auto-subdivision of cable '{name}' at joint '{joint}', which lies on it"
                ),
            });
        }
    }

    // Subdivision where two line objects cross away from a joint would create
    // a joint connecting them. That is not modeled; it is recorded when such
    // a crossing exists.
    for name in &lines_meshed_at_crossings {
        if let Some(other) = model.frames.get(name).and_then(|f| crossing_line(&model, f)) {
            model.unmodeled_source.push(UnmodeledSourceData {
                scope: UnmodeledScope::AllCases,
                description: format!(
                    "auto-subdivision of frame '{name}' where it crosses line object '{other}'"
                ),
            });
        }
    }

    // Sections that carry analysis-relevant data but are not interpreted
    // above (line springs, added masses, auto-mesh, hinges, several load
    // types, ...) are recorded so they are not silently dropped.
    skipped::record(sdb, &lpat_names, &mut model);

    Ok(model)
}

/// Whether a native line object mirrors its insertion point about local 2 or 3.
fn insertion_is_mirrored(line: &HashMap<String, SdbVal>) -> bool {
    ["m_Mirror2", "m_mirror3"]
        .iter()
        .any(|flag| line.get(*flag).and_then(SdbVal::as_bool) == Some(true))
}

/// The joint offset of a native `FOffset` record, in the element local system
/// and equal at both ends.
///
/// The one corpus record (Example 1-011, +12 in along global Z at both ends of
/// a member whose local 2 axis is global Z) stores a 3 x 2 array
/// `[0, 0, 12, 12, 0, 0]` with coordinate system code -1. That fixes the
/// layout as [component][end] and the code -1 as the local system. It does not
/// show which column is which end, or any other coordinate system code, so
/// unequal ends and other codes are not read.
fn frame_joint_offset(record: &HashMap<String, SdbVal>) -> Result<[f64; 3], String> {
    let values = record
        .get("m_P")
        .and_then(SdbVal::as_arr_f64)
        .filter(|values| values.len() == 6)
        .ok_or_else(|| "not three components at two ends".to_string())?;
    let csys = record.get("m_csys").and_then(SdbVal::as_i32).unwrap_or(0);
    if csys != -1 {
        return Err(format!("coordinate system code {csys} is not the local system"));
    }
    if (0..3).any(|c| values[2 * c] != values[2 * c + 1]) {
        return Err("they differ between the ends, whose order in the record is not established".to_string());
    }
    Ok([values[0], values[2], values[4]])
}

/// The automatic-subdivision assignment of a native line object.
struct LineAutoMesh {
    /// Number of segments (`0` where none is asked for).
    segments: usize,
    /// Subdivision at joints lying on the object.
    at_joints: bool,
    /// Subdivision where the object crosses other line objects.
    at_crossings: bool,
    /// Parts of the assignment that are not modeled whatever the geometry.
    records: Vec<UnmodeledSourceData>,
}

/// Reads `m_AutoMesh`, `m_automeshn`, `m_AutoMeshAtPoints`, `m_AutoMeshAtLines`
/// and `m_AutoMeshMaxLength` of a line object. The solver models a segment
/// count and subdivision at joints for frame objects; a maximum segment length
/// and a segment count on a cable are recorded instead.
fn line_auto_mesh(line: &HashMap<String, SdbVal>, name: &str, is_cable: bool) -> LineAutoMesh {
    let flag = |field: &str| line.get(field).and_then(SdbVal::as_bool) == Some(true);
    let mut mesh = LineAutoMesh {
        segments: 0,
        at_joints: false,
        at_crossings: false,
        records: Vec::new(),
    };
    if !flag("m_AutoMesh") {
        return mesh;
    }
    mesh.segments = line
        .get("m_automeshn")
        .and_then(SdbVal::as_i32)
        .unwrap_or(0)
        .max(0) as usize;
    mesh.at_joints = flag("m_AutoMeshAtPoints");
    mesh.at_crossings = flag("m_AutoMeshAtLines");
    let max_length = line
        .get("m_AutoMeshMaxLength")
        .and_then(SdbVal::as_f64)
        .unwrap_or(0.0);
    if max_length != 0.0 {
        mesh.records.push(UnmodeledSourceData {
            scope: UnmodeledScope::AllButLinearStatic,
            description: format!(
                "auto-subdivision of frame '{name}' to a maximum segment length of {max_length}"
            ),
        });
    }
    if is_cable && mesh.segments > 1 {
        mesh.records.push(UnmodeledSourceData {
            scope: UnmodeledScope::AllButLinearStatic,
            description: format!(
                "auto-subdivision of cable '{name}' into {} segments",
                mesh.segments
            ),
        });
    }
    mesh
}

/// A joint lying on the straight line between the ends of `frame`, strictly
/// inside it (one millionth of its length from the line and from both ends).
fn joint_on_chord(model: &SapModel, frame: &FrameObj) -> Option<String> {
    let (pi, pj) = (
        model.points.get(&frame.point_i)?,
        model.points.get(&frame.point_j)?,
    );
    let axis = [pj.x - pi.x, pj.y - pi.y, pj.z - pi.z];
    let length2: f64 = axis.iter().map(|c| c * c).sum();
    if length2 <= 0.0 {
        return None;
    }
    model
        .points
        .values()
        .find(|p| {
            if p.name == frame.point_i || p.name == frame.point_j {
                return false;
            }
            let rel = [p.x - pi.x, p.y - pi.y, p.z - pi.z];
            let t = (0..3).map(|c| rel[c] * axis[c]).sum::<f64>() / length2;
            let off2: f64 = (0..3).map(|c| (rel[c] - t * axis[c]).powi(2)).sum();
            t > 1e-6 && t < 1.0 - 1e-6 && off2 <= 1e-12 * length2
        })
        .map(|p| p.name.clone())
}

/// Another line object that crosses `frame` strictly inside both, so at a
/// joint of neither. Parallel and collinear objects do not cross.
fn crossing_line(model: &SapModel, frame: &FrameObj) -> Option<String> {
    let ends = |f: &FrameObj| {
        let (a, b) = (model.points.get(&f.point_i)?, model.points.get(&f.point_j)?);
        Some(([a.x, a.y, a.z], [b.x, b.y, b.z]))
    };
    let (a0, a1) = ends(frame)?;
    let dot = |x: &[f64; 3], y: &[f64; 3]| (0..3).map(|c| x[c] * y[c]).sum::<f64>();
    model
        .frames
        .values()
        .find(|other| {
            if other.name == frame.name {
                return false;
            }
            let Some((b0, b1)) = ends(other) else {
                return false;
            };
            let d1: [f64; 3] = std::array::from_fn(|c| a1[c] - a0[c]);
            let d2: [f64; 3] = std::array::from_fn(|c| b1[c] - b0[c]);
            let r: [f64; 3] = std::array::from_fn(|c| a0[c] - b0[c]);
            let (a, e, f) = (dot(&d1, &d1), dot(&d2, &d2), dot(&d2, &r));
            let (b, c) = (dot(&d1, &d2), dot(&d1, &r));
            let denom = a * e - b * b;
            if a <= 0.0 || e <= 0.0 || denom <= 1e-12 * a * e {
                return false;
            }
            // Closest points: a0 + s d1 and b0 + t d2.
            let s = (b * f - c * e) / denom;
            let t = (a * f - b * c) / denom;
            let gap2: f64 = (0..3)
                .map(|k| (r[k] + s * d1[k] - t * d2[k]).powi(2))
                .sum();
            let inside = |v: f64| v > 1e-6 && v < 1.0 - 1e-6;
            inside(s) && inside(t) && gap2 <= 1e-12 * a.max(e)
        })
        .map(|other| other.name.clone())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_joint_offsets_are_read_only_in_the_one_verified_form() {
        let record = |values: &[f64], csys: i32| -> HashMap<String, SdbVal> {
            HashMap::from([
                ("m_P".to_string(), SdbVal::ArrF64(values.to_vec())),
                ("m_csys".to_string(), SdbVal::I32(csys)),
            ])
        };
        // The Example 1-011 record: +12 along local 2 at both ends.
        assert_eq!(
            frame_joint_offset(&record(&[0.0, 0.0, 12.0, 12.0, 0.0, 0.0], -1)),
            Ok([0.0, 12.0, 0.0])
        );
        assert_eq!(
            frame_joint_offset(&record(&[1.0, 1.0, -2.0, -2.0, 3.5, 3.5], -1)),
            Ok([1.0, -2.0, 3.5])
        );
        // Another coordinate system, unequal ends and a wrong size are not read.
        assert!(frame_joint_offset(&record(&[0.0, 0.0, 12.0, 12.0, 0.0, 0.0], 0))
            .unwrap_err()
            .contains("coordinate system code 0"));
        assert!(frame_joint_offset(&record(&[0.0, 0.0, 12.0, 6.0, 0.0, 0.0], -1))
            .unwrap_err()
            .contains("differ between the ends"));
        assert!(frame_joint_offset(&record(&[0.0, 12.0, 0.0], -1)).is_err());

        // Either mirror flag marks the insertion point as mirrored.
        let line = |a: bool, b: bool| -> HashMap<String, SdbVal> {
            HashMap::from([
                ("m_Mirror2".to_string(), SdbVal::Bool(a)),
                ("m_mirror3".to_string(), SdbVal::Bool(b)),
            ])
        };
        assert!(!insertion_is_mirrored(&line(false, false)));
        assert!(!insertion_is_mirrored(&HashMap::new()));
        assert!(insertion_is_mirrored(&line(true, false)));
        assert!(insertion_is_mirrored(&line(false, true)));
    }

    #[test]
    fn line_auto_mesh_reads_what_is_modeled_and_records_the_rest() {
        let line = |fields: &[(&str, SdbVal)]| -> HashMap<String, SdbVal> {
            fields.iter().map(|(k, v)| (k.to_string(), v.clone())).collect()
        };
        // Not enabled: stale values behind the flag are ignored.
        let off = line(&[
            ("m_AutoMesh", SdbVal::Bool(false)),
            ("m_automeshn", SdbVal::I32(8)),
            ("m_AutoMeshAtPoints", SdbVal::Bool(true)),
            ("m_AutoMeshMaxLength", SdbVal::F64(24.0)),
        ]);
        let mesh = line_auto_mesh(&off, "1", false);
        assert_eq!((mesh.segments, mesh.at_joints, mesh.at_crossings), (0, false, false));
        assert!(mesh.records.is_empty());

        let on = line(&[
            ("m_AutoMesh", SdbVal::Bool(true)),
            ("m_automeshn", SdbVal::I32(16)),
            ("m_AutoMeshAtPoints", SdbVal::Bool(true)),
            ("m_AutoMeshAtLines", SdbVal::Bool(true)),
        ]);
        let mesh = line_auto_mesh(&on, "1", false);
        assert_eq!((mesh.segments, mesh.at_joints, mesh.at_crossings), (16, true, true));
        assert!(mesh.records.is_empty());

        // A negative count is no count.
        let negative = line(&[("m_AutoMesh", SdbVal::Bool(true)), ("m_automeshn", SdbVal::I32(-3))]);
        assert_eq!(line_auto_mesh(&negative, "1", false).segments, 0);

        // A maximum segment length is not modeled.
        let limited = line(&[
            ("m_AutoMesh", SdbVal::Bool(true)),
            ("m_AutoMeshMaxLength", SdbVal::F64(24.0)),
        ]);
        let mesh = line_auto_mesh(&limited, "B7", false);
        assert_eq!(
            mesh.records,
            [UnmodeledSourceData {
                scope: UnmodeledScope::AllButLinearStatic,
                description: "auto-subdivision of frame 'B7' to a maximum segment length of 24"
                    .to_string(),
            }]
        );

        // Neither is a segment count on a cable object.
        let mesh = line_auto_mesh(&on, "C1", true);
        assert_eq!(
            mesh.records,
            [UnmodeledSourceData {
                scope: UnmodeledScope::AllButLinearStatic,
                description: "auto-subdivision of cable 'C1' into 16 segments".to_string(),
            }]
        );
    }

    #[test]
    fn joints_on_a_chord_and_line_crossings_are_found_by_geometry() {
        let mut m = SapModel::default();
        m.add_point("A", 0.0, 0.0, 0.0);
        m.add_point("B", 10.0, 0.0, 0.0);
        m.add_frame("AB", "A", "B", "FSEC1");
        assert_eq!(joint_on_chord(&m, &m.frames["AB"]), None);
        assert_eq!(crossing_line(&m, &m.frames["AB"]), None);

        // Beyond the end, off the line, and on it.
        m.add_point("BEYOND", 12.0, 0.0, 0.0);
        m.add_point("OFF", 4.0, 0.001, 0.0);
        assert_eq!(joint_on_chord(&m, &m.frames["AB"]), None);
        m.add_point("ON", 4.0, 0.0, 0.0);
        assert_eq!(joint_on_chord(&m, &m.frames["AB"]), Some("ON".to_string()));

        // A T-junction ends at a joint of the other object: not a crossing.
        m.add_point("T", 4.0, 5.0, 0.0);
        m.add_frame("TEE", "ON", "T", "FSEC1");
        assert_eq!(crossing_line(&m, &m.frames["AB"]), None);
        // Parallel, collinear and skew objects do not cross either.
        m.add_point("P1", 0.0, 3.0, 0.0);
        m.add_point("P2", 10.0, 3.0, 0.0);
        m.add_frame("PARALLEL", "P1", "P2", "FSEC1");
        m.add_frame("COLLINEAR", "ON", "BEYOND", "FSEC1");
        m.add_point("S1", 6.0, -5.0, 2.0);
        m.add_point("S2", 6.0, 5.0, 2.0);
        m.add_frame("SKEW", "S1", "S2", "FSEC1");
        assert_eq!(crossing_line(&m, &m.frames["AB"]), None);
        // A true crossing inside both, with no joint there.
        m.add_point("X1", 7.0, -5.0, 0.0);
        m.add_point("X2", 7.0, 5.0, 0.0);
        m.add_frame("CROSS", "X1", "X2", "FSEC1");
        assert_eq!(crossing_line(&m, &m.frames["AB"]), Some("CROSS".to_string()));
        assert_eq!(crossing_line(&m, &m.frames["CROSS"]), Some("AB".to_string()));
    }

    #[test]
    fn unsupported_native_history_or_spectrum_excitation_is_an_explicit_error() {
        for (case_type, field, value) in [(5, "m_TF", 20.0), (7, "m_at", 2.0), (4, "m_Ang", 30.0)] {
            let assignment = HashMap::from([(field.into(), SdbVal::F64(value))]);
            let error = check_native_dynamic_assignment(
                "CASE_WITH_UNSUPPORTED_LOAD",
                case_type,
                &assignment,
            )
            .unwrap_err();
            assert!(matches!(error, FormatError::UnsupportedNativeCase { .. }));
            assert!(error.to_string().contains("CASE_WITH_UNSUPPORTED_LOAD"));
        }
        let assignment = HashMap::from([("m_csys".into(), SdbVal::I32(2))]);
        assert!(check_native_dynamic_assignment("ROTATED_SPEC", 4, &assignment).is_err());
        let directory = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../extracted/files");
        let path = directory.join("example_6_003a.sdb");
        if path.exists() {
            let source = read_sdb_file(path).unwrap();
            assert!(matches!(
                extract_sdb_to_model(&source),
                Err(FormatError::UnsupportedNativeCase { .. })
            ));
            assert!(native::inspect_native_sdb(&source).is_ok());
        }
    }

    #[test]
    fn native_case_codes_are_not_oapi_enum_codes() {
        let directory = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../extracted/files");
        if !directory.exists() {
            return;
        }
        let spectral =
            extract_sdb_to_model(&read_sdb_file(directory.join("example_1_022.sdb")).unwrap())
                .unwrap();
        assert_eq!(spectral.load_cases["MODAL"].case_type, LoadCaseType::Modal); // native 3
        assert_eq!(
            spectral.load_cases["SPECSRSS"].case_type,
            LoadCaseType::ResponseSpectrum
        ); // native 4
        assert_eq!(
            spectral.load_cases["MHIST1"].case_type,
            LoadCaseType::LinearHistory
        ); // native 5
        assert_eq!(
            spectral.load_cases["DHIST1"].case_type,
            LoadCaseType::LinearDynamic
        ); // native 7
        assert_eq!(
            spectral.load_cases["SPECSRSS"].modal_combo,
            ModalComboMethod::Srss
        );
        assert_eq!(spectral.load_cases["MHIST1"].num_time_steps, 400);
        assert_eq!(spectral.load_cases["MHIST1"].time_step_dt, 0.02);
        assert_eq!(
            spectral.load_cases["SPECSRSS"].spectrum_loads[0],
            ("ELC_RS".into(), "U1".into(), 386.4)
        );
        let nonlinear =
            extract_sdb_to_model(&read_sdb_file(directory.join("example_1_016e.sdb")).unwrap())
                .unwrap();
        assert_eq!(
            nonlinear.load_cases["NLSTAT1"].case_type,
            LoadCaseType::NonlinearStatic
        ); // native 2
        let buckling =
            extract_sdb_to_model(&read_sdb_file(directory.join("example_1_019a.sdb")).unwrap())
                .unwrap();
        assert_eq!(
            buckling.load_cases["BUCK1"].case_type,
            LoadCaseType::Buckling
        ); // native 10
    }

    #[test]
    fn previously_omitted_native_solids_links_functions_and_loads_are_imported() {
        let directory = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../extracted/files");
        if !directory.exists() {
            return;
        }
        let load = |name: &str| {
            extract_sdb_to_model(&read_sdb_file(directory.join(name)).unwrap()).unwrap()
        };
        let solid = load("example_5_005a.sdb");
        assert_eq!(solid.solids.len(), 16);
        assert_eq!(
            solid
                .solids
                .values()
                .map(|s| s.loads_surface.len())
                .sum::<usize>(),
            16
        );
        assert!(solid
            .solids
            .values()
            .all(|s| s.loads_surface[0].face == 6 && s.loads_surface[0].val == -0.0001));
        let temperature = load("example_5_011.sdb");
        assert_eq!(temperature.solids.len(), 100);
        assert!(temperature
            .solids
            .values()
            .all(|s| s.loads_temp.values().any(|t| *t == 100.0)));
        let link = load("example_6_001.sdb");
        assert_eq!(link.links.len(), 1);
        assert_eq!(link.links["1"].point_i, "1");
        assert_eq!(link.links["1"].point_j.as_deref(), Some("2"));
        assert_eq!(link.links["1"].prop, "LIN1");
        let pressure = load("example_2_010a_thick.sdb");
        assert_eq!(
            pressure
                .areas
                .values()
                .map(|a| a.loads_pressure.len())
                .sum::<usize>(),
            128
        );
        let area_temperature = load("example_2_013a_thick.sdb");
        assert_eq!(
            area_temperature
                .areas
                .values()
                .map(|a| a.loads_temp.len())
                .sum::<usize>(),
            5
        );
        let functions = load("example_1_022.sdb");
        assert_eq!(functions.functions["ELC_RS"].points.len(), 33);
        assert!(!functions.functions["ELC_RS"].is_time_history);
        assert_eq!(functions, parse_s2k(&write_s2k(&functions)).unwrap());
    }

    #[test]
    fn absent_struct_array_slots_preserve_native_reference_indices() {
        let class = b"S_ProjectStartUp";
        let mut bytes = vec![1];
        bytes.extend_from_slice(&3_i32.to_le_bytes());
        bytes.extend_from_slice(&[0, 1, class.len() as u8]);
        bytes.extend_from_slice(class);
        for value in [1900_i32, 6, 3] {
            bytes.extend_from_slice(&value.to_le_bytes());
        }
        bytes.push(0);
        let mut reader = BinarySdbReader::new(&bytes);
        let entries = reader.read_struct_array(true).unwrap();
        assert_eq!(entries.len(), 3);
        assert_eq!(entries[0]["m_alive"].as_bool(), Some(false));
        assert_eq!(entries[1]["m_Units"].as_i32(), Some(6));
        assert_eq!(entries[2]["m_alive"].as_bool(), Some(false));
        assert_eq!(reader.pos, bytes.len());
    }

    #[test]
    fn named_load_coordinates_never_become_global() {
        let names = vec![String::new(), "GLOBAL".into(), "ROTATED".into()];
        assert_eq!(coordinate_name(&names, 0).unwrap(), "LOCAL");
        assert_eq!(coordinate_name(&names, 1).unwrap(), "GLOBAL");
        assert_eq!(coordinate_name(&names, 2).unwrap(), "ROTATED");
        assert!(coordinate_name(&names, 3).is_err());
    }

    #[test]
    fn unrepresented_material_symtype_or_spring_csys_is_rejected() {
        let mut model = SapModel::new(Units::KipInF);
        model.coord_systems.insert(
            "ROTATED".into(),
            sap_model::CoordSystem {
                name: "ROTATED".into(),
                origin: [0.0; 3],
                angles_deg: [45.0, 0.0, 0.0],
                grids: Vec::new(),
            },
        );
        let mut p1 = PointObj::new("1", 0.0, 0.0, 0.0);
        p1.spring[2] = 50.0;
        model.points.insert("1".into(), p1);

        let base = native::pack_model_to_native_sdb(&model).unwrap();
        let doc = native::inspect_native_sdb(&base).unwrap();

        let mat_span = doc
            .records()
            .iter()
            .find(|r| r.class_id == "S_Material")
            .unwrap()
            .fields
            .iter()
            .find(|f| f.name.as_deref() == Some("m_SymType"))
            .unwrap();
        let mut bad_mat = base.clone();
        bad_mat.decompressed_stream[mat_span.offset..mat_span.offset + 4]
            .copy_from_slice(&2_i32.to_le_bytes());
        let err = extract_sdb_to_model(&bad_mat).unwrap_err();
        assert!(matches!(err, FormatError::InvalidSdb(_)), "{err}");

        let sp_span = doc
            .records()
            .iter()
            .find(|r| r.class_id == "TYSpring")
            .unwrap()
            .fields
            .iter()
            .find(|f| f.name.as_deref() == Some("m_csys"))
            .unwrap();
        let mut bad_sp = base.clone();
        bad_sp.decompressed_stream[sp_span.offset..sp_span.offset + 4]
            .copy_from_slice(&2_i32.to_le_bytes());
        let err = extract_sdb_to_model(&bad_sp).unwrap_err();
        assert!(matches!(err, FormatError::InvalidSdb(_)), "{err}");
    }

    #[test]
    fn vendor_springs_orthotropic_materials_and_cables_extract_and_roundtrip() {
        let directory = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../extracted/files");
        if !directory.exists() {
            return;
        }
        let load = |name: &str| {
            extract_sdb_to_model(&read_sdb_file(directory.join(name)).unwrap()).unwrap()
        };

        // 1. 21-term TYSpring fixtures (m_csys = -1 LOCAL)
        let s1005b = load("example_1_005b.sdb");
        let sprung_b: Vec<_> = s1005b
            .points
            .values()
            .filter(|p| p.spring.iter().any(|&k| k != 0.0) || p.spring_coupled.is_some())
            .collect();
        assert_eq!(sprung_b.len(), 1);
        assert!(sprung_b[0].spring_is_local);
        assert_eq!(sprung_b[0].spring, [0.0, 0.0, 10.0, 0.0, 0.0, 0.0]);
        assert_eq!(sprung_b[0].spring_coupled, None);
        assert_eq!(
            extract_sdb_to_model(&native::pack_model_to_native_sdb(&s1005b).unwrap()).unwrap(),
            s1005b
        );

        let s1005d = load("example_1_005d.sdb");
        let sprung_d: Vec<_> = s1005d
            .points
            .values()
            .filter(|p| p.spring.iter().any(|&k| k != 0.0) || p.spring_coupled.is_some())
            .collect();
        assert_eq!(sprung_d.len(), 1);
        assert!(sprung_d[0].spring_is_local);
        assert_eq!(sprung_d[0].spring, [0.0, 0.0, 0.0, 0.0, 80000.0, 0.0]);
        assert_eq!(sprung_d[0].spring_coupled, None);
        assert_eq!(
            extract_sdb_to_model(&native::pack_model_to_native_sdb(&s1005d).unwrap()).unwrap(),
            s1005d
        );

        // 2. Orthotropic S_Material (m_SymType = 1) fixtures
        for fixture in [
            "example_2_015a_thick.sdb",
            "example_2_015a_thin.sdb",
            "example_2_015b_thick.sdb",
            "example_2_015b_thin.sdb",
            "example_2_015c_thick.sdb",
            "example_2_015d_thick.sdb",
        ] {
            let m = load(fixture);
            let ortho: Vec<_> = m
                .materials
                .values()
                .filter(|mat| mat.e_ortho.is_some())
                .collect();
            assert!(!ortho.is_empty(), "{fixture} expected orthotropic material");
            for mat in ortho {
                assert!(mat.g_ortho.is_some(), "{fixture}: {}", mat.name);
                assert!(mat.u_ortho.is_some(), "{fixture}: {}", mat.name);
                assert!(mat.a_ortho.is_some(), "{fixture}: {}", mat.name);
                assert_eq!(mat.e, mat.e_ortho.unwrap()[0]);
                assert_eq!(mat.g, mat.g_ortho.unwrap()[0]);
                assert_eq!(mat.u, mat.u_ortho.unwrap()[0]);
                assert_eq!(mat.a, mat.a_ortho.unwrap()[0]);
            }
            assert_eq!(
                extract_sdb_to_model(&native::pack_model_to_native_sdb(&m).unwrap()).unwrap(),
                m,
                "{fixture}"
            );
        }

        // 3. S_CableData m_CableType governing specification fixtures
        for fixture in ["example_7_001.sdb", "example_7_002.sdb"] {
            let m = load(fixture);
            let cables: Vec<_> = m
                .frames
                .values()
                .filter_map(|f| f.cable_params.as_ref())
                .collect();
            assert!(!cables.is_empty(), "{fixture}");
            for cp in cables {
                assert!(cp.undeformed_length.is_some(), "{fixture}");
                assert!(cp.target_tension_i.is_none(), "{fixture}");
                assert!(cp.horizontal_tension.is_none(), "{fixture}");
            }
            assert_eq!(
                extract_sdb_to_model(&native::pack_model_to_native_sdb(&m).unwrap()).unwrap(),
                m,
                "{fixture}"
            );
        }

        let c7003 = load("example_7_003.sdb");
        let cables_7003: Vec<_> = c7003
            .frames
            .values()
            .filter_map(|f| f.cable_params.as_ref())
            .collect();
        assert_eq!(cables_7003.len(), 12);
        for cp in cables_7003 {
            assert!(cp.undeformed_length.is_none());
            assert!(cp.target_tension_i.is_some());
            assert!(cp.horizontal_tension.is_none());
        }
        assert_eq!(
            extract_sdb_to_model(&native::pack_model_to_native_sdb(&c7003).unwrap()).unwrap(),
            c7003
        );
    }
}
