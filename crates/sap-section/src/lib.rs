use sap_model::{FramePropType, FrameSection};
use sap_units::{LengthUnit, Units};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;
use std::fs;
use std::path::Path;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum SectionError {
    #[error("I/O error reading .pro file: {0}")]
    Io(#[from] std::io::Error),
    #[error("Invalid .pro file size {0} (must be a multiple of 120 bytes and >= 240)")]
    InvalidFileSize(usize),
    #[error("Invalid .pro header: {0}")]
    InvalidHeader(String),
    #[error("Non-finite .pro property at record {record}, field {field}")]
    InvalidProperty { record: usize, field: usize },
    #[error("Invalid .pro header unit code {0}")]
    InvalidUnitCode(f32),
    #[error("Section '{0}' not found in database")]
    SectionNotFound(String),
}

/// Represents a parsed CSI `.pro` binary section property database (`aisc14.pro`, `euro.pro`, etc.).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProDatabase {
    pub file_name: String,
    pub version: f32,
    pub length_unit: LengthUnit,
    pub sections: Vec<FrameSection>,
}

impl ProDatabase {
    /// Parses a binary CSI `.pro` file according to the exact 120-byte (`0x78`) layout
    /// reverse-engineered from `sap2000.exe` (`0x0600a614`–`0x0600a61b`).
    pub fn from_file(path: impl AsRef<Path>) -> Result<Self, SectionError> {
        let path_ref = path.as_ref();
        let bytes = fs::read(path_ref)?;
        let file_name = path_ref
            .file_name()
            .and_then(|s| s.to_str())
            .unwrap_or("unknown.pro")
            .to_string();
        Self::from_bytes(&bytes, file_name)
    }

    pub fn from_bytes(bytes: &[u8], file_name: String) -> Result<Self, SectionError> {
        if bytes.len() < 240 || bytes.len() % 120 != 0 {
            return Err(SectionError::InvalidFileSize(bytes.len()));
        }
        let read_f32 = |off: usize| -> f32 {
            f32::from_le_bytes([bytes[off], bytes[off + 1], bytes[off + 2], bytes[off + 3]])
        };

        let version = read_f32(0x28);
        if !version.is_finite() || !matches!(version, 6.0 | 7.0) {
            return Err(SectionError::InvalidHeader(format!(
                "Unsupported version {}",
                version
            )));
        }
        let unit_code = read_f32(0x2c);
        if !unit_code.is_finite() || unit_code.fract() != 0.0 {
            return Err(SectionError::InvalidUnitCode(unit_code));
        }
        let declared_records = read_f32(0x30);
        let actual_records = bytes.len() / 120 - 1;
        if !declared_records.is_finite()
            || declared_records < 0.0
            || declared_records.fract() != 0.0
            || declared_records > actual_records as f32
        {
            return Err(SectionError::InvalidHeader(format!(
                "Declared {} records, found {}",
                declared_records, actual_records
            )));
        }
        let length_unit =
            LengthUnit::from_pro_code(unit_code).ok_or(SectionError::InvalidUnitCode(unit_code))?;

        // Legacy vendor libraries can contain complete records beyond the declared count.
        // The vendor reader imports the declared records only.
        let num_records = declared_records as usize;
        let mut sections = Vec::with_capacity(num_records);

        for idx in 1..=num_records {
            let base = idx * 120;
            let raw_name = if version >= 6.5 {
                String::from_utf8_lossy(&bytes[base..base + 36])
                    .trim()
                    .to_string()
            } else {
                // Version 6.0 stores 18-char US name + 18-char SI name
                let n1 = String::from_utf8_lossy(&bytes[base..base + 18])
                    .trim()
                    .to_string();
                if !n1.is_empty() {
                    n1
                } else {
                    String::from_utf8_lossy(&bytes[base + 18..base + 36])
                        .trim()
                        .to_string()
                }
            };
            if raw_name.is_empty() {
                continue;
            }

            let shape_str = String::from_utf8_lossy(&bytes[base + 36..base + 40]);
            let shape_tok = shape_str.split_whitespace().next().unwrap_or("W");
            let prop_type = match shape_tok {
                "W" | "I" | "S" | "M" | "HP" => FramePropType::I,
                "C" | "MC" => FramePropType::Channel,
                "T" | "WT" | "MT" | "ST" | "TW" => FramePropType::T,
                "L" => FramePropType::Angle,
                "2L" => FramePropType::DblAngle,
                "B" | "TS" | "HSS" => FramePropType::Box,
                "P" => FramePropType::Pipe,
                "R" => FramePropType::Rectangular,
                "CR" => FramePropType::Circle,
                "2C" => FramePropType::DbChannel,
                _ => FramePropType::General,
            };

            let mut f = [0.0f64; 20];
            for (k, slot) in f.iter_mut().enumerate() {
                *slot = read_f32(base + 40 + k * 4) as f64;
                if !slot.is_finite() {
                    return Err(SectionError::InvalidProperty {
                        record: idx,
                        field: k,
                    });
                }
            }

            let t3 = f[0];
            let t2 = f[1];
            let tf = f[2];
            let tw = f[3];
            let area = f[5];
            let torsional_j = f[6];
            let i33 = f[7];
            let i22 = f[8];
            let as2 = f[9];
            let as3 = f[10];
            let z33 = f[11];
            let z22 = f[12];
            let x_bar = f[13];
            let y_bar = f[14];
            let dis = f[16];
            let t2b = if f[18] > 0.0 { f[18] } else { t2 };
            let tfb = if f[19] > 0.0 { f[19] } else { tf };

            let c3 = if y_bar > 0.0 && y_bar < t3 {
                y_bar.max(t3 - y_bar)
            } else {
                0.5 * t3
            };
            let width = t2.max(t2b);
            let c2 = if x_bar > 0.0 && x_bar < width {
                x_bar.max(width - x_bar)
            } else {
                0.5 * width
            };
            let s33 = if c3 > 1e-12 { i33 / c3 } else { 0.0 };
            let s22 = if c2 > 1e-12 { i22 / c2 } else { 0.0 };
            let r33 = if area > 1e-12 {
                (i33 / area).sqrt()
            } else {
                0.0
            };
            let r22 = if area > 1e-12 {
                (i22 / area).sqrt()
            } else {
                0.0
            };

            sections.push(FrameSection {
                name: raw_name.clone(),
                material: "A992Fy50".to_string(),
                prop_type,
                t3,
                t2,
                tf,
                tw,
                t2b,
                tfb,
                dis,
                area,
                torsional_j,
                i33,
                i22,
                i23: 0.0,
                as2,
                as3,
                s33,
                s22,
                z33,
                z22,
                r33,
                r22,
                modifiers: [1.0; 8],
                non_prismatic: Vec::new(),
                auto_select_list: Vec::new(),
                concrete_rebar: None,
                file_name: Some(file_name.clone()),
                name_in_file: Some(raw_name),
            });
        }

        Ok(Self {
            file_name,
            version,
            length_unit,
            sections,
        })
    }

    /// Finds a section by name (case-insensitive) and scales its geometric properties
    /// to `target_units`.
    pub fn get_section_scaled(
        &self,
        section_name: &str,
        target_units: Units,
        material: &str,
    ) -> Result<FrameSection, SectionError> {
        let target_upper = section_name.trim().to_ascii_uppercase();
        let sec = self
            .sections
            .iter()
            .find(|s| s.name.to_ascii_uppercase() == target_upper)
            .ok_or_else(|| SectionError::SectionNotFound(section_name.to_string()))?;

        let fl = self.length_unit.to_meters() / target_units.length_unit().to_meters();
        let mut out = sec.clone();
        out.material = material.to_string();
        out.t3 *= fl;
        out.t2 *= fl;
        out.tf *= fl;
        out.tw *= fl;
        out.t2b *= fl;
        out.tfb *= fl;
        out.dis *= fl;
        out.area *= fl * fl;
        out.as2 *= fl * fl;
        out.as3 *= fl * fl;
        out.r33 *= fl;
        out.r22 *= fl;
        out.s33 *= fl.powi(3);
        out.s22 *= fl.powi(3);
        out.z33 *= fl.powi(3);
        out.z22 *= fl.powi(3);
        out.torsional_j *= fl.powi(4);
        out.i33 *= fl.powi(4);
        out.i22 *= fl.powi(4);
        out.i23 *= fl.powi(4);
        Ok(out)
    }
}

// ============================================================================
// PARAMETRIC SECTION PROPERTY CALCULATORS (Matching `cPropFrame.Set*`)
// ============================================================================

pub fn make_rectangular(
    name: impl Into<String>,
    material: impl Into<String>,
    t3: f64,
    t2: f64,
) -> FrameSection {
    let area = t2 * t3;
    let i33 = t2 * t3.powi(3) / 12.0;
    let i22 = t3 * t2.powi(3) / 12.0;
    let as2 = (5.0 / 6.0) * area;
    let as3 = (5.0 / 6.0) * area;
    let s33 = t2 * t3.powi(2) / 6.0;
    let s22 = t3 * t2.powi(2) / 6.0;
    let z33 = t2 * t3.powi(2) / 4.0;
    let z22 = t3 * t2.powi(2) / 4.0;
    let r33 = (i33 / area).sqrt();
    let r22 = (i22 / area).sqrt();
    // St. Venant torsional constant for rectangular bar (Roark / SAP2000 formula)
    let a = 0.5 * t3.max(t2);
    let b = 0.5 * t3.min(t2);
    let ratio = if a > 1e-12 { b / a } else { 1.0 };
    let torsional_j = a * b.powi(3) * (16.0 / 3.0 - 3.36 * ratio * (1.0 - ratio.powi(4) / 12.0));

    FrameSection {
        name: name.into(),
        material: material.into(),
        prop_type: FramePropType::Rectangular,
        t3,
        t2,
        tf: 0.0,
        tw: 0.0,
        t2b: t2,
        tfb: 0.0,
        dis: 0.0,
        area,
        torsional_j,
        i33,
        i22,
        i23: 0.0,
        as2,
        as3,
        s33,
        s22,
        z33,
        z22,
        r33,
        r22,
        modifiers: [1.0; 8],
        non_prismatic: Vec::new(),
        auto_select_list: Vec::new(),
        concrete_rebar: None,
        file_name: None,
        name_in_file: None,
    }
}

pub fn make_circle(name: impl Into<String>, material: impl Into<String>, d: f64) -> FrameSection {
    let r = 0.5 * d;
    let area = PI * r * r;
    let i33 = PI * d.powi(4) / 64.0;
    let i22 = i33;
    let torsional_j = PI * d.powi(4) / 32.0;
    let as2 = 0.9 * area;
    let as3 = 0.9 * area;
    let s33 = PI * d.powi(3) / 32.0;
    let z33 = d.powi(3) / 6.0;
    let r33 = 0.25 * d;

    FrameSection {
        name: name.into(),
        material: material.into(),
        prop_type: FramePropType::Circle,
        t3: d,
        t2: d,
        tf: 0.0,
        tw: 0.0,
        t2b: d,
        tfb: 0.0,
        dis: 0.0,
        area,
        torsional_j,
        i33,
        i22,
        i23: 0.0,
        as2,
        as3,
        s33,
        s22: s33,
        z33,
        z22: z33,
        r33,
        r22: r33,
        modifiers: [1.0; 8],
        non_prismatic: Vec::new(),
        auto_select_list: Vec::new(),
        concrete_rebar: None,
        file_name: None,
        name_in_file: None,
    }
}

pub fn make_pipe(
    name: impl Into<String>,
    material: impl Into<String>,
    d_out: f64,
    tw: f64,
) -> FrameSection {
    let d_in = (d_out - 2.0 * tw).max(0.0);
    let area = 0.25 * PI * (d_out.powi(2) - d_in.powi(2));
    let i33 = PI * (d_out.powi(4) - d_in.powi(4)) / 64.0;
    let torsional_j = 2.0 * i33;
    let as2 = 0.5 * area;
    let s33 = if d_out > 1e-12 {
        2.0 * i33 / d_out
    } else {
        0.0
    };
    let z33 = (d_out.powi(3) - d_in.powi(3)) / 6.0;
    let r33 = if area > 1e-12 {
        (i33 / area).sqrt()
    } else {
        0.0
    };

    FrameSection {
        name: name.into(),
        material: material.into(),
        prop_type: FramePropType::Pipe,
        t3: d_out,
        t2: d_out,
        tf: tw,
        tw,
        t2b: d_out,
        tfb: tw,
        dis: 0.0,
        area,
        torsional_j,
        i33,
        i22: i33,
        i23: 0.0,
        as2,
        as3: as2,
        s33,
        s22: s33,
        z33,
        z22: z33,
        r33,
        r22: r33,
        modifiers: [1.0; 8],
        non_prismatic: Vec::new(),
        auto_select_list: Vec::new(),
        concrete_rebar: None,
        file_name: None,
        name_in_file: None,
    }
}

pub fn make_box_tube(
    name: impl Into<String>,
    material: impl Into<String>,
    t3: f64,
    t2: f64,
    tf: f64,
    tw: f64,
) -> FrameSection {
    let h_in = (t3 - 2.0 * tf).max(0.0);
    let b_in = (t2 - 2.0 * tw).max(0.0);
    let area = t2 * t3 - b_in * h_in;
    let i33 = (t2 * t3.powi(3) - b_in * h_in.powi(3)) / 12.0;
    let i22 = (t3 * t2.powi(3) - h_in * b_in.powi(3)) / 12.0;
    // Bredt's thin-walled closed tube formula for J
    let am = (t3 - tf).max(0.0) * (t2 - tw).max(0.0);
    let denom = 2.0 * ((t3 - tf) / tw.max(1e-9) + (t2 - tw) / tf.max(1e-9));
    let torsional_j = if denom > 1e-12 {
        4.0 * am * am / denom
    } else {
        0.0
    };
    let as2 = 2.0 * t3 * tw;
    let as3 = 2.0 * t2 * tf;
    let s33 = 2.0 * i33 / t3.max(1e-12);
    let s22 = 2.0 * i22 / t2.max(1e-12);
    let z33 = 0.25 * (t2 * t3.powi(2) - b_in * h_in.powi(2));
    let z22 = 0.25 * (t3 * t2.powi(2) - h_in * b_in.powi(2));
    let r33 = (i33 / area.max(1e-12)).sqrt();
    let r22 = (i22 / area.max(1e-12)).sqrt();

    FrameSection {
        name: name.into(),
        material: material.into(),
        prop_type: FramePropType::Box,
        t3,
        t2,
        tf,
        tw,
        t2b: t2,
        tfb: tf,
        dis: 0.0,
        area,
        torsional_j,
        i33,
        i22,
        i23: 0.0,
        as2,
        as3,
        s33,
        s22,
        z33,
        z22,
        r33,
        r22,
        modifiers: [1.0; 8],
        non_prismatic: Vec::new(),
        auto_select_list: Vec::new(),
        concrete_rebar: None,
        file_name: None,
        name_in_file: None,
    }
}

pub fn make_i_section(
    name: impl Into<String>,
    material: impl Into<String>,
    t3: f64,
    t2: f64,
    tf: f64,
    tw: f64,
    t2b: f64,
    tfb: f64,
) -> FrameSection {
    let hw = (t3 - tf - tfb).max(0.0);
    let a_top = t2 * tf;
    let a_bot = t2b * tfb;
    let a_web = hw * tw;
    let area = a_top + a_bot + a_web;

    let y_bot = 0.5 * tfb;
    let y_web = tfb + 0.5 * hw;
    let y_top = t3 - 0.5 * tf;
    let y_bar = (a_bot * y_bot + a_web * y_web + a_top * y_top) / area.max(1e-12);

    let i33 = (t2b * tfb.powi(3) / 12.0 + a_bot * (y_bot - y_bar).powi(2))
        + (tw * hw.powi(3) / 12.0 + a_web * (y_web - y_bar).powi(2))
        + (t2 * tf.powi(3) / 12.0 + a_top * (y_top - y_bar).powi(2));
    let i22 = (tfb * t2b.powi(3) + hw * tw.powi(3) + tf * t2.powi(3)) / 12.0;
    let torsional_j = (t2 * tf.powi(3) + t2b * tfb.powi(3) + hw * tw.powi(3)) / 3.0;
    let as2 = t3 * tw;
    let as3 = (5.0 / 6.0) * (a_top + a_bot);
    let c3 = y_bar.max(t3 - y_bar);
    let c2 = 0.5 * t2.max(t2b);
    let s33 = i33 / c3.max(1e-12);
    let s22 = i22 / c2.max(1e-12);
    // Symmetric approximation for Z33, Z22
    let z33 = a_top * (t3 - y_bar - 0.5 * tf).abs()
        + a_bot * (y_bar - 0.5 * tfb).abs()
        + 0.25 * tw * hw.powi(2);
    let z22 = 0.25 * (tf * t2.powi(2) + tfb * t2b.powi(2) + hw * tw.powi(2));
    let r33 = (i33 / area.max(1e-12)).sqrt();
    let r22 = (i22 / area.max(1e-12)).sqrt();

    FrameSection {
        name: name.into(),
        material: material.into(),
        prop_type: FramePropType::I,
        t3,
        t2,
        tf,
        tw,
        t2b,
        tfb,
        dis: 0.0,
        area,
        torsional_j,
        i33,
        i22,
        i23: 0.0,
        as2,
        as3,
        s33,
        s22,
        z33,
        z22,
        r33,
        r22,
        modifiers: [1.0; 8],
        non_prismatic: Vec::new(),
        auto_select_list: Vec::new(),
        concrete_rebar: None,
        file_name: None,
        name_in_file: None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn synthetic_channel() -> Vec<u8> {
        let mut bytes = vec![b' '; 240];
        for (offset, value) in [(40, 7.0_f32), (44, 1.0), (48, 1.0)] {
            bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        }
        bytes[120..127].copy_from_slice(b"CHANNEL");
        bytes[156..160].copy_from_slice(b"C   ");
        let mut values = [0.0_f32; 20];
        values[0] = 10.0;
        values[1] = 6.0;
        values[5] = 10.0;
        values[7] = 200.0;
        values[8] = 100.0;
        values[13] = 1.0;
        for (index, value) in values.iter().enumerate() {
            bytes[160 + index * 4..164 + index * 4].copy_from_slice(&value.to_le_bytes());
        }
        bytes
    }

    #[test]
    fn asymmetric_section_modulus_uses_horizontal_centroid_and_scales_cubically() {
        let db = ProDatabase::from_bytes(&synthetic_channel(), "test.pro".into()).unwrap();
        assert_eq!(db.sections[0].s22, 20.0); // I22 / max(xbar, width-xbar)
        let scaled = db
            .get_section_scaled("channel", Units::KnMmC, "steel")
            .unwrap();
        assert!((scaled.s22 - 20.0 * 25.4_f64.powi(3)).abs() < 1e-8);
        assert!((scaled.i22 - 100.0 * 25.4_f64.powi(4)).abs() < 1e-6);
    }

    #[test]
    fn malformed_pro_headers_and_nonfinite_properties_are_rejected() {
        for (offset, value) in [
            (40, f32::NAN),
            (40, 8.0),
            (44, 1.2),
            (48, 2.0),
            (48, 0.5),
            (180, f32::INFINITY),
        ] {
            let mut bytes = synthetic_channel();
            bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
            assert!(
                ProDatabase::from_bytes(&bytes, "corrupt.pro".into()).is_err(),
                "offset {}",
                offset
            );
        }
        let mut extra_records = synthetic_channel();
        extra_records.extend_from_slice(&[0; 120]);
        assert_eq!(
            ProDatabase::from_bytes(&extra_records, "legacy.pro".into())
                .unwrap()
                .sections
                .len(),
            1
        );
    }

    #[test]
    fn vendor_pro_libraries_parse_with_declared_record_counts() {
        let directory = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../extracted/files");
        if !directory.exists() {
            return;
        }
        let mut checked = 0;
        for entry in fs::read_dir(directory).unwrap() {
            let path = entry.unwrap().path();
            if path.extension().and_then(|p| p.to_str()) != Some("pro")
                || path.file_name().unwrap() == "joists.pro"
            {
                continue;
            }
            ProDatabase::from_file(&path).unwrap_or_else(|e| panic!("{}: {}", path.display(), e));
            checked += 1;
        }
        assert!(checked >= 28, "Only {} vendor databases checked", checked);
    }

    #[test]
    fn test_aisc14_pro_database_parsing() {
        let pro_path = Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../../extracted/files/aisc14.pro"));
        if pro_path.exists() {
            let db = ProDatabase::from_file(pro_path).expect("Failed to parse aisc14.pro");
            assert_eq!(db.length_unit, LengthUnit::In);
            assert!(db.sections.len() >= 1900);
            let w14x90 = db
                .get_section_scaled("W14X90", Units::KipInF, "A992Fy50")
                .expect("W14X90 should exist in aisc14.pro");
            assert!((w14x90.t3 - 14.0).abs() < 0.05);
            assert!((w14x90.area - 26.5).abs() < 0.2);
            assert!((w14x90.i33 - 999.0).abs() < 2.0);
        }
    }
}
