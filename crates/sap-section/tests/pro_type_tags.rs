//! D98: the 4-byte type tag of a `.pro` record is read by its first two characters
//! (docs/re/formats_and_sections.md: the vendor reads the first two characters), so the doubled
//! tags "2L2L", "2C2C", "CRCR", "WW  " are recognised. The buffers are built here; no vendor
//! bytes are in the repository.
use sap_model::FramePropType;
use sap_section::ProDatabase;
use std::path::Path;

/// A version 7 library, inch units, with one record per tag. Every record has the same
/// property values, so that only the tag can make two records differ.
fn library(tags: &[&[u8; 4]]) -> Vec<u8> {
    let mut bytes = vec![b' '; 120 * (tags.len() + 1)];
    for (offset, value) in [(40, 7.0_f32), (44, 1.0), (48, tags.len() as f32)] {
        bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }
    for (i, tag) in tags.iter().enumerate() {
        let base = 120 * (i + 1);
        let name = format!("S{i}");
        bytes[base..base + name.len()].copy_from_slice(name.as_bytes());
        bytes[base + 36..base + 40].copy_from_slice(*tag);
        let mut values = [0.0_f32; 20];
        values[0] = 10.0; // t3
        values[1] = 6.0; // t2
        values[2] = 0.5; // tf
        values[3] = 0.3; // tw
        values[5] = 10.0; // area
        values[7] = 200.0; // i33
        values[8] = 100.0; // i22
        values[16] = 0.25; // dis
        for (k, value) in values.iter().enumerate() {
            let at = base + 40 + k * 4;
            bytes[at..at + 4].copy_from_slice(&value.to_le_bytes());
        }
    }
    bytes
}

#[test]
fn d98_two_character_tags_give_the_vendor_types() {
    let tags: [(&[u8; 4], FramePropType); 9] = [
        (b"2L2L", FramePropType::DblAngle),
        (b"2C2C", FramePropType::DbChannel),
        (b"CRCR", FramePropType::Circle),
        (b"WW  ", FramePropType::I),
        (b"G G ", FramePropType::General),
        (b"W W ", FramePropType::I),
        (b"2L  ", FramePropType::DblAngle),
        (b"TW  ", FramePropType::T),
        (b"B   ", FramePropType::Box),
    ];
    let list: Vec<&[u8; 4]> = tags.iter().map(|(t, _)| *t).collect();
    let db = ProDatabase::from_bytes(&library(&list), "synthetic.pro".into()).unwrap();
    assert_eq!(db.sections.len(), tags.len());
    for ((tag, expected), section) in tags.iter().zip(&db.sections) {
        assert_eq!(
            section.prop_type,
            *expected,
            "tag {:?}",
            String::from_utf8_lossy(*tag)
        );
    }
}

#[test]
fn d98_w_w_and_w_records_are_bitwise_the_same_as_before() {
    // "W W " and "W   " already parsed as W; the records differ only in the name
    let db = ProDatabase::from_bytes(&library(&[b"W W ", b"W   "]), "synthetic.pro".into())
        .unwrap();
    let (a, b) = (&db.sections[0], &db.sections[1]);
    assert_eq!(a.prop_type, FramePropType::I);
    assert_eq!(b.prop_type, FramePropType::I);
    let mut a2 = a.clone();
    a2.name = b.name.clone();
    a2.name_in_file = b.name_in_file.clone();
    assert_eq!(&a2, b);
    // and an unknown two-character tag is General, as an unknown tag always was
    let db = ProDatabase::from_bytes(&library(&[b"ZZ  ", b"Q Q "]), "synthetic.pro".into())
        .unwrap();
    assert!(db.sections.iter().all(|s| s.prop_type == FramePropType::General));
}

/// Optional vendor corpus (skips when `extracted` is absent): the counts of the audit.
#[test]
fn d98_vendor_corpus_type_counts() {
    let directory = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../extracted/files");
    if !directory.exists() {
        return;
    }
    let (mut dbl_angle, mut db_channel, mut circle, mut general) = (0, 0, 0, 0);
    for entry in std::fs::read_dir(directory).unwrap() {
        let path = entry.unwrap().path();
        if path.extension().and_then(|p| p.to_str()) != Some("pro")
            || path.file_name().unwrap() == "joists.pro"
        {
            continue;
        }
        let db = ProDatabase::from_file(&path).unwrap();
        for s in &db.sections {
            match s.prop_type {
                FramePropType::DblAngle => dbl_angle += 1,
                FramePropType::DbChannel => db_channel += 1,
                FramePropType::Circle => circle += 1,
                FramePropType::General => general += 1,
                _ => {}
            }
        }
    }
    // tags "2L2L" 12873 + "2L  " 1710, "2C2C" 1792, "CRCR" 13, "G G " 13 (28 parseable files)
    assert_eq!(dbl_angle, 12_873 + 1_710);
    assert_eq!(db_channel, 1_792);
    assert_eq!(circle, 13);
    assert_eq!(general, 13);
}
