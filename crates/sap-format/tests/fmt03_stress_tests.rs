//! Empirical stress-test suite for FMT-03: Native SDB project_info and grids.
use sap_format::sdb::native::{inspect_native_sdb, pack_model_to_native_sdb};
use sap_format::{extract_sdb_to_model, parse_s2k, write_s2k, FormatError};
use sap_model::*;

fn base_model() -> SapModel {
    let mut m = SapModel::default();
    m.add_point("P1", 0.0, 0.0, 0.0);
    m.add_point("P2", 10.0, 0.0, 0.0);
    m.add_frame("F1", "P1", "P2", "FSEC1");
    m
}

fn assert_unsupported(m: &SapModel, expected_err_substring: &str) {
    let err = pack_model_to_native_sdb(m).expect_err("expected export to fail");
    match err {
        FormatError::UnsupportedNativeExport { field, reason } => {
            let combined = format!("{field}: {reason}");
            assert!(
                combined.contains(expected_err_substring)
                    || field.contains(expected_err_substring)
                    || reason.contains(expected_err_substring),
                "Expected error substring '{expected_err_substring}', but got '{combined}'"
            );
        }
        other => panic!("Expected UnsupportedNativeExport, got: {other:?}"),
    }
}

// =========================================================================
// FMT-03 Project Info Tests
// =========================================================================

#[test]
fn test_project_info_all_13_standard_keys() {
    let mut m = base_model();
    let std_keys = [
        ("Company Name", "ACME Engineering Corp."),
        ("Client Name", "Metropolis Infrastructure Dept."),
        ("Project Name", "Skyline Tower Expansion"),
        ("Project Number", "PRJ-2026-X88"),
        ("Model Name", "Tower_Final_REV4"),
        ("Model Description", "3D framed tower with lateral bracing"),
        ("Revision Number", "4.2.1"),
        ("Frame Type", "Space Frame"),
        ("Engineer", "Dr. Jane Doe, P.E."),
        ("Checker", "John Smith, S.E."),
        ("Supervisor", "E. Vance"),
        ("Issue Code", "FINAL-ISSUE-B"),
        ("Design Code", "AISC 360-16 / ACI 318-19"),
    ];

    for (k, v) in std_keys {
        m.project_info.insert(k.to_string(), v.to_string());
    }

    let native = pack_model_to_native_sdb(&m).expect("native export with 13 standard keys failed");
    let doc = inspect_native_sdb(&native).expect("inspect native sdb failed");
    assert_eq!(doc.inventory().parsed_bytes, doc.inventory().stream_bytes);

    let restored = extract_sdb_to_model(&native).expect("native import failed");
    for (k, v) in std_keys {
        assert_eq!(
            restored.project_info.get(k).map(|s| s.as_str()),
            Some(v),
            "Mismatch for standard key '{k}'"
        );
    }
    assert_eq!(restored.project_info.len(), 13);
    assert_eq!(restored, m);

    // Also verify S2K fidelity
    let s2k_text = write_s2k(&restored);
    let s2k_restored = parse_s2k(&s2k_text).expect("S2K parse failed");
    assert_eq!(s2k_restored, m);
}

#[test]
fn test_project_info_sparse_standard_keys() {
    let mut m = base_model();
    m.project_info
        .insert("Company Name".to_string(), "Solo Dev".to_string());
    m.project_info
        .insert("Engineer".to_string(), "Self".to_string());

    let native = pack_model_to_native_sdb(&m).expect("native export with sparse keys failed");
    let restored = extract_sdb_to_model(&native).expect("native import failed");

    assert_eq!(restored.project_info.len(), 2);
    assert_eq!(restored.project_info["Company Name"], "Solo Dev");
    assert_eq!(restored.project_info["Engineer"], "Self");
    assert!(!restored.project_info.contains_key("Project Name"));
    assert!(!restored.project_info.contains_key("Client Name"));
    assert_eq!(restored, m);
}

#[test]
fn test_project_info_user_defined_keys_only() {
    let mut m = base_model();
    m.project_info
        .insert("CustomKey1".to_string(), "Value1".to_string());
    m.project_info
        .insert("CustomKey2".to_string(), "Value2".to_string());
    m.project_info.insert(
        "AuditHash".to_string(),
        "SHA256-abcdef0123456789".to_string(),
    );

    let native = pack_model_to_native_sdb(&m).expect("native export with user keys only failed");
    let restored = extract_sdb_to_model(&native).expect("native import failed");

    assert_eq!(restored.project_info.len(), 3);
    assert_eq!(restored.project_info["CustomKey1"], "Value1");
    assert_eq!(restored.project_info["CustomKey2"], "Value2");
    assert_eq!(
        restored.project_info["AuditHash"],
        "SHA256-abcdef0123456789"
    );
    assert_eq!(restored, m);
}

#[test]
fn test_project_info_100_user_defined_keys_stress() {
    let mut m = base_model();
    m.project_info
        .insert("Company Name".to_string(), "Big Corporate".to_string());
    for i in 0..100 {
        m.project_info.insert(
            format!("CustomField_{i:04}"),
            format!("DynamicValue_{i:04}_x99283471029384710293847"),
        );
    }

    let native = pack_model_to_native_sdb(&m).expect("native export with 100 user keys failed");
    let doc = inspect_native_sdb(&native).expect("inspect failed");
    assert_eq!(doc.inventory().parsed_bytes, doc.inventory().stream_bytes);

    let restored = extract_sdb_to_model(&native).expect("native import failed");
    assert_eq!(restored.project_info.len(), 101);
    for i in 0..100 {
        let k = format!("CustomField_{i:04}");
        let expected_v = format!("DynamicValue_{i:04}_x99283471029384710293847");
        assert_eq!(restored.project_info.get(&k), Some(&expected_v));
    }
    assert_eq!(restored, m);
}

#[test]
fn test_project_info_unicode_and_emojis() {
    let mut m = base_model();
    m.project_info.insert(
        "Company Name".to_string(),
        "株式会社 構造計画研究所 🏢".to_string(),
    );
    m.project_info.insert(
        "Project Name".to_string(),
        "Стадион Олимпийский 🏆 — Проект №12".to_string(),
    );
    m.project_info.insert(
        "Engineer".to_string(),
        "Müller & Söhne Ingenieure (öäüß éèç ñ)".to_string(),
    );
    m.project_info.insert(
        "UnicodeTag_🦀".to_string(),
        "Rustaceans 🚀 100% α=0.05 β=1.234 ± 0.01".to_string(),
    );
    m.project_info.insert(
        "MultilingualKey_你好世界".to_string(),
        "مرحبا بالعالم - שלום עולם".to_string(),
    );

    let native = pack_model_to_native_sdb(&m).expect("native export with unicode failed");
    let doc = inspect_native_sdb(&native).expect("inspect native sdb failed");
    assert_eq!(doc.inventory().parsed_bytes, doc.inventory().stream_bytes);

    let restored = extract_sdb_to_model(&native).expect("native import failed");
    assert_eq!(
        restored.project_info["Company Name"],
        "株式会社 構造計画研究所 🏢"
    );
    assert_eq!(
        restored.project_info["Project Name"],
        "Стадион Олимпийский 🏆 — Проект №12"
    );
    assert_eq!(
        restored.project_info["Engineer"],
        "Müller & Söhne Ingenieure (öäüß éèç ñ)"
    );
    assert_eq!(
        restored.project_info["UnicodeTag_🦀"],
        "Rustaceans 🚀 100% α=0.05 β=1.234 ± 0.01"
    );
    assert_eq!(
        restored.project_info["MultilingualKey_你好世界"],
        "مرحبا بالعالم - שלום עולם"
    );
    assert_eq!(restored, m);
}

#[test]
fn test_project_info_punctuation_spaces_and_quotes() {
    let mut m = base_model();
    m.project_info.insert(
        "Company Name".to_string(),
        "A & B \"Designers\" / Architects, LLC; [Ref #99.99]".to_string(),
    );
    m.project_info.insert(
        "Complex_Key: {a=1, b=2}; // test".to_string(),
        "Value with spaces, tabs\tand symbols: !@#$%^&*()_+-=[]{}|;':,./<>?".to_string(),
    );

    let native = pack_model_to_native_sdb(&m).expect("native export with punctuation failed");
    let doc = inspect_native_sdb(&native).expect("inspect failed");
    assert_eq!(doc.inventory().parsed_bytes, doc.inventory().stream_bytes);

    let restored = extract_sdb_to_model(&native).expect("native import failed");
    assert_eq!(
        restored.project_info["Company Name"],
        "A & B \"Designers\" / Architects, LLC; [Ref #99.99]"
    );
    assert_eq!(
        restored.project_info["Complex_Key: {a=1, b=2}; // test"],
        "Value with spaces, tabs\tand symbols: !@#$%^&*()_+-=[]{}|;':,./<>?"
    );
    assert_eq!(restored, m);
}

#[test]
fn test_project_info_empty_key_rejection() {
    let mut m = base_model();
    m.project_info
        .insert("".to_string(), "Some Value".to_string());
    assert_unsupported(&m, "empty project_info key or value is not representable");
}

#[test]
fn test_project_info_empty_value_rejection() {
    let mut m = base_model();
    m.project_info
        .insert("CustomKey".to_string(), "".to_string());
    assert_unsupported(&m, "empty project_info key or value is not representable");

    let mut m2 = base_model();
    m2.project_info
        .insert("Company Name".to_string(), "".to_string());
    assert_unsupported(&m2, "empty project_info key or value is not representable");
}

// =========================================================================
// FMT-03 Grids Tests
// =========================================================================

#[test]
fn test_grids_multi_axis_100_plus_lines_stress() {
    let mut m = base_model();
    let expected_grids: Vec<GridLine> = {
        // 120 lines on X, 120 lines on Y, 80 lines on Z = 320 grid lines total!
        let mut grids = Vec::with_capacity(320);
        for i in 0..120 {
            grids.push(GridLine {
                axis: "X".to_string(),
                id: format!("GX_{i}"),
                coordinate: i as f64 * 3.5 - 200.0,
                visible: i % 2 == 0,
                bubble_loc: if i % 3 == 0 { "Start" } else { "End" }.to_string(),
            });
        }
        for i in 0..120 {
            grids.push(GridLine {
                axis: "Y".to_string(),
                id: format!("GY_{i}"),
                coordinate: i as f64 * 4.0 - 240.0,
                visible: true,
                bubble_loc: if i % 2 == 0 { "End" } else { "Start" }.to_string(),
            });
        }
        for i in 0..80 {
            grids.push(GridLine {
                axis: "Z".to_string(),
                id: format!("GZ_{i}"),
                coordinate: i as f64 * 3.0,
                visible: false,
                bubble_loc: "End".to_string(),
            });
        }
        grids
    };
    m.coord_systems.get_mut("GLOBAL").unwrap().grids = expected_grids.clone();

    let native = pack_model_to_native_sdb(&m).expect("native export of 320 grid lines failed");
    let doc = inspect_native_sdb(&native).expect("inspect native sdb failed");
    assert_eq!(doc.inventory().parsed_bytes, doc.inventory().stream_bytes);

    let restored = extract_sdb_to_model(&native).expect("native import failed");
    let r_grids = &restored.coord_systems["GLOBAL"].grids;
    assert_eq!(r_grids.len(), 320);

    for (idx, (orig, rest)) in expected_grids.iter().zip(r_grids.iter()).enumerate() {
        assert_eq!(orig.axis, rest.axis, "axis mismatch at index {idx}");
        assert_eq!(orig.id, rest.id, "id mismatch at index {idx}");
        assert_eq!(
            orig.coordinate, rest.coordinate,
            "coordinate mismatch at index {idx}"
        );
        assert_eq!(
            orig.visible, rest.visible,
            "visible mismatch at index {idx}"
        );
        assert_eq!(
            orig.bubble_loc, rest.bubble_loc,
            "bubble_loc mismatch at index {idx}"
        );
    }
    assert_eq!(restored, m);
}

#[test]
fn test_grids_negative_and_extreme_coordinates() {
    let mut m = base_model();
    let expected_grids: Vec<GridLine> = {
        let coords_x = [
            -1_000_000.5,
            -9999.123456,
            -0.000001,
            0.0,
            0.000001,
            1234.5678,
            5_000_000.75,
        ];
        let mut grids = Vec::new();
        for (i, &coord) in coords_x.iter().enumerate() {
            grids.push(GridLine {
                axis: "X".to_string(),
                id: format!("X{i}"),
                coordinate: coord,
                visible: true,
                bubble_loc: "Start".to_string(),
            });
        }
        grids.push(GridLine {
            axis: "Y".to_string(),
            id: "Y_neg".to_string(),
            coordinate: -45000.875,
            visible: false,
            bubble_loc: "End".to_string(),
        });
        grids.push(GridLine {
            axis: "Z".to_string(),
            id: "Z_subgrade".to_string(),
            coordinate: -12.5,
            visible: true,
            bubble_loc: "Start".to_string(),
        });
        grids
    };
    m.coord_systems.get_mut("GLOBAL").unwrap().grids = expected_grids.clone();

    let native = pack_model_to_native_sdb(&m).expect("native export with extreme coords failed");
    let doc = inspect_native_sdb(&native).expect("inspect failed");
    assert_eq!(doc.inventory().parsed_bytes, doc.inventory().stream_bytes);

    let restored = extract_sdb_to_model(&native).expect("native import failed");
    let r_grids = &restored.coord_systems["GLOBAL"].grids;
    assert_eq!(r_grids.len(), expected_grids.len());

    for (orig, rest) in expected_grids.iter().zip(r_grids.iter()) {
        assert_eq!(orig.id, rest.id);
        assert_eq!(
            orig.coordinate.to_bits(),
            rest.coordinate.to_bits(),
            "Float exact bit match failed for {}",
            orig.id
        );
    }
    assert_eq!(restored, m);
}

#[test]
fn test_grids_bubble_location_supported() {
    let mut m = base_model();
    m.coord_systems.get_mut("GLOBAL").unwrap().grids = vec![
        GridLine {
            axis: "X".to_string(),
            id: "X1".to_string(),
            coordinate: 0.0,
            visible: true,
            bubble_loc: "Start".to_string(),
        },
        GridLine {
            axis: "Y".to_string(),
            id: "Y1".to_string(),
            coordinate: 0.0,
            visible: true,
            bubble_loc: "End".to_string(),
        },
    ];

    let native = pack_model_to_native_sdb(&m)
        .expect("native export with 'Start' and 'End' bubble_loc must succeed");
    let restored = extract_sdb_to_model(&native).expect("native import failed");
    let r_grids = &restored.coord_systems["GLOBAL"].grids;
    assert_eq!(r_grids[0].bubble_loc, "Start");
    assert_eq!(r_grids[1].bubble_loc, "End");
    assert_eq!(restored, m);
}

#[test]
fn test_grids_bubble_location_unrepresentable_values() {
    // Native SDB represents bubble switch as a single bool (Start vs End).
    // Representing "Both" or "None" is impossible without data loss,
    // so verify_projection must reject them.
    for unsupported_loc in ["Both", "None", "start", "end", "START", "END", "Center"] {
        let mut m = base_model();
        m.coord_systems.get_mut("GLOBAL").unwrap().grids = vec![GridLine {
            axis: "X".to_string(),
            id: "X1".to_string(),
            coordinate: 0.0,
            visible: true,
            bubble_loc: unsupported_loc.to_string(),
        }];

        let err = pack_model_to_native_sdb(&m).expect_err(&format!(
            "bubble_loc '{unsupported_loc}' should be rejected"
        ));
        match err {
            FormatError::UnsupportedNativeExport { field, reason } => {
                assert!(
                    field.contains("bubble_loc"),
                    "field should mention bubble_loc: {field}"
                );
                assert!(
                    reason.contains(
                        "native serialization/projection would change represented model data"
                    ) || reason.contains("bubble location"),
                    "unexpected reason: {reason}"
                );
            }
            other => panic!("expected UnsupportedNativeExport, got: {other:?}"),
        }
    }
}

#[test]
fn test_grids_out_of_order_coordinate_values_within_axis() {
    let mut m = base_model();
    // Out-of-order coordinate values within an axis are legal in SAP2000
    // (e.g., user defined grids out of spatial order or descending order)
    let expected_grids = vec![
        GridLine {
            axis: "X".to_string(),
            id: "X_high".to_string(),
            coordinate: 100.0,
            visible: true,
            bubble_loc: "Start".to_string(),
        },
        GridLine {
            axis: "X".to_string(),
            id: "X_neg".to_string(),
            coordinate: -50.0,
            visible: true,
            bubble_loc: "End".to_string(),
        },
        GridLine {
            axis: "X".to_string(),
            id: "X_zero".to_string(),
            coordinate: 0.0,
            visible: true,
            bubble_loc: "Start".to_string(),
        },
        GridLine {
            axis: "X".to_string(),
            id: "X_mid".to_string(),
            coordinate: 25.0,
            visible: false,
            bubble_loc: "End".to_string(),
        },
        GridLine {
            axis: "Y".to_string(),
            id: "Y_desc3".to_string(),
            coordinate: 30.0,
            visible: true,
            bubble_loc: "End".to_string(),
        },
        GridLine {
            axis: "Y".to_string(),
            id: "Y_desc2".to_string(),
            coordinate: 20.0,
            visible: true,
            bubble_loc: "End".to_string(),
        },
        GridLine {
            axis: "Y".to_string(),
            id: "Y_desc1".to_string(),
            coordinate: 10.0,
            visible: true,
            bubble_loc: "End".to_string(),
        },
        GridLine {
            axis: "Z".to_string(),
            id: "Z_roof".to_string(),
            coordinate: 45.0,
            visible: true,
            bubble_loc: "End".to_string(),
        },
        GridLine {
            axis: "Z".to_string(),
            id: "Z_ground".to_string(),
            coordinate: 0.0,
            visible: true,
            bubble_loc: "Start".to_string(),
        },
        GridLine {
            axis: "Z".to_string(),
            id: "Z_floor2".to_string(),
            coordinate: 15.0,
            visible: true,
            bubble_loc: "End".to_string(),
        },
    ];
    m.coord_systems.get_mut("GLOBAL").unwrap().grids = expected_grids.clone();

    let native = pack_model_to_native_sdb(&m)
        .expect("out-of-order coordinate values within an axis must succeed");
    let doc = inspect_native_sdb(&native).expect("inspect failed");
    assert_eq!(doc.inventory().parsed_bytes, doc.inventory().stream_bytes);

    let restored = extract_sdb_to_model(&native).expect("native import failed");
    let r_grids = &restored.coord_systems["GLOBAL"].grids;
    assert_eq!(r_grids.len(), 10);

    // Verify EXACT original sequence is preserved (no unexpected spatial sorting)
    for (i, (orig, rest)) in expected_grids.iter().zip(r_grids.iter()).enumerate() {
        assert_eq!(orig.id, rest.id, "ID mismatch at index {i}");
        assert_eq!(
            orig.coordinate, rest.coordinate,
            "Coordinate mismatch at index {i}"
        );
        assert_eq!(orig.axis, rest.axis, "Axis mismatch at index {i}");
    }
    assert_eq!(restored, m);
}

#[test]
fn test_grids_out_of_order_axes_rejection() {
    let mut m = base_model();
    // Grouping by Y then X violates X <= Y <= Z order requirement
    m.coord_systems.get_mut("GLOBAL").unwrap().grids = vec![
        GridLine {
            axis: "Y".to_string(),
            id: "Y1".to_string(),
            coordinate: 0.0,
            visible: true,
            bubble_loc: "Start".to_string(),
        },
        GridLine {
            axis: "X".to_string(),
            id: "X1".to_string(),
            coordinate: 0.0,
            visible: true,
            bubble_loc: "End".to_string(),
        },
    ];

    assert_unsupported(&m, "grid lines must be grouped by axis in X, Y, Z order");

    // Interleaved axes (X then Y then X) must also be rejected
    m.coord_systems.get_mut("GLOBAL").unwrap().grids = vec![
        GridLine {
            axis: "X".to_string(),
            id: "X1".to_string(),
            coordinate: 0.0,
            visible: true,
            bubble_loc: "Start".to_string(),
        },
        GridLine {
            axis: "Y".to_string(),
            id: "Y1".to_string(),
            coordinate: 0.0,
            visible: true,
            bubble_loc: "Start".to_string(),
        },
        GridLine {
            axis: "X".to_string(),
            id: "X2".to_string(),
            coordinate: 10.0,
            visible: true,
            bubble_loc: "End".to_string(),
        },
    ];
    assert_unsupported(&m, "grid lines must be grouped by axis in X, Y, Z order");
}

#[test]
fn test_grids_invalid_axis_rejection() {
    let mut m = base_model();

    m.coord_systems.get_mut("GLOBAL").unwrap().grids = vec![GridLine {
        axis: "W".to_string(),
        id: "W1".to_string(),
        coordinate: 0.0,
        visible: true,
        bubble_loc: "Start".to_string(),
    }];
    assert_unsupported(&m, "grid axis must be 'X', 'Y', or 'Z'");

    m.coord_systems.get_mut("GLOBAL").unwrap().grids = vec![GridLine {
        axis: "".to_string(),
        id: "EmptyAxis".to_string(),
        coordinate: 0.0,
        visible: true,
        bubble_loc: "Start".to_string(),
    }];
    assert_unsupported(&m, "grid axis must be 'X', 'Y', or 'Z'");
}

#[test]
fn test_grids_nonfinite_coordinate_rejection() {
    let mut m = base_model();

    m.coord_systems.get_mut("GLOBAL").unwrap().grids = vec![GridLine {
        axis: "X".to_string(),
        id: "NaN_Grid".to_string(),
        coordinate: f64::NAN,
        visible: true,
        bubble_loc: "Start".to_string(),
    }];
    assert_unsupported(&m, "grid coordinate must be a finite number");

    m.coord_systems.get_mut("GLOBAL").unwrap().grids = vec![GridLine {
        axis: "X".to_string(),
        id: "Inf_Grid".to_string(),
        coordinate: f64::INFINITY,
        visible: true,
        bubble_loc: "Start".to_string(),
    }];
    assert_unsupported(&m, "grid coordinate must be a finite number");
}

#[test]
fn test_project_info_large_string_64kb() {
    let mut m = base_model();
    // 64 KiB string to stress variable-length 7-bit integer encoding
    let big_val = "A".repeat(65536);
    m.project_info
        .insert("LargePayload".to_string(), big_val.clone());

    let native = pack_model_to_native_sdb(&m).expect("native export of 64KB string failed");
    let doc = inspect_native_sdb(&native).expect("inspect failed");
    assert_eq!(doc.inventory().parsed_bytes, doc.inventory().stream_bytes);

    let restored = extract_sdb_to_model(&native).expect("native import failed");
    assert_eq!(restored.project_info["LargePayload"].len(), 65536);
    assert_eq!(restored.project_info["LargePayload"], big_val);
    assert_eq!(restored, m);
}

#[test]
fn test_project_info_whitespace_and_newlines() {
    let mut m = base_model();
    m.project_info
        .insert("SpacesOnlyKey".to_string(), "     ".to_string());
    m.project_info.insert(
        "MultilineNote".to_string(),
        "Line 1: Initial design\r\nLine 2: Peer reviewed\nLine 3: Approved with conditions.\tTabbed.".to_string(),
    );

    let native =
        pack_model_to_native_sdb(&m).expect("native export of whitespace and newlines failed");
    let restored = extract_sdb_to_model(&native).expect("native import failed");
    assert_eq!(restored.project_info["SpacesOnlyKey"], "     ");
    assert_eq!(
        restored.project_info["MultilineNote"],
        "Line 1: Initial design\r\nLine 2: Peer reviewed\nLine 3: Approved with conditions.\tTabbed."
    );
    assert_eq!(restored, m);
}

#[test]
fn test_grids_duplicate_ids_across_axes() {
    let mut m = base_model();
    // Grid ID "A" reused on X, Y, and Z axes (common engineering practice)
    m.coord_systems.get_mut("GLOBAL").unwrap().grids = vec![
        GridLine {
            axis: "X".to_string(),
            id: "A".to_string(),
            coordinate: 0.0,
            visible: true,
            bubble_loc: "Start".to_string(),
        },
        GridLine {
            axis: "Y".to_string(),
            id: "A".to_string(),
            coordinate: 0.0,
            visible: true,
            bubble_loc: "Start".to_string(),
        },
        GridLine {
            axis: "Z".to_string(),
            id: "A".to_string(),
            coordinate: 0.0,
            visible: true,
            bubble_loc: "End".to_string(),
        },
    ];

    let native =
        pack_model_to_native_sdb(&m).expect("grids with duplicate IDs across axes must succeed");
    let restored = extract_sdb_to_model(&native).expect("native import failed");
    let r_grids = &restored.coord_systems["GLOBAL"].grids;
    assert_eq!(r_grids.len(), 3);
    assert_eq!(r_grids[0].id, "A");
    assert_eq!(r_grids[0].axis, "X");
    assert_eq!(r_grids[1].id, "A");
    assert_eq!(r_grids[1].axis, "Y");
    assert_eq!(r_grids[2].id, "A");
    assert_eq!(r_grids[2].axis, "Z");
    assert_eq!(restored, m);
}

#[test]
fn test_grids_coincident_coordinates() {
    let mut m = base_model();
    // Multiple grid lines at the exact same spatial coordinate (construction / phased lines)
    m.coord_systems.get_mut("GLOBAL").unwrap().grids = vec![
        GridLine {
            axis: "X".to_string(),
            id: "Phase1_Boundary".to_string(),
            coordinate: 15.0,
            visible: true,
            bubble_loc: "Start".to_string(),
        },
        GridLine {
            axis: "X".to_string(),
            id: "Phase2_Boundary".to_string(),
            coordinate: 15.0,
            visible: false,
            bubble_loc: "End".to_string(),
        },
    ];

    let native = pack_model_to_native_sdb(&m).expect("coincident grid coordinates must succeed");
    let restored = extract_sdb_to_model(&native).expect("native import failed");
    let r_grids = &restored.coord_systems["GLOBAL"].grids;
    assert_eq!(r_grids.len(), 2);
    assert_eq!(r_grids[0].coordinate, 15.0);
    assert_eq!(r_grids[1].coordinate, 15.0);
    assert_eq!(r_grids[0].id, "Phase1_Boundary");
    assert_eq!(r_grids[1].id, "Phase2_Boundary");
    assert_eq!(restored, m);
}

#[test]
fn test_grids_unicode_ids_and_spaces() {
    let mut m = base_model();
    m.coord_systems.get_mut("GLOBAL").unwrap().grids = vec![
        GridLine {
            axis: "X".to_string(),
            id: "Grid 1 (East Wing / Phase A)".to_string(),
            coordinate: 0.0,
            visible: true,
            bubble_loc: "Start".to_string(),
        },
        GridLine {
            axis: "X".to_string(),
            id: "轴线-A1 📐".to_string(),
            coordinate: 6.0,
            visible: true,
            bubble_loc: "End".to_string(),
        },
        GridLine {
            axis: "Y".to_string(),
            id: "Ось 1.0 (Главная)".to_string(),
            coordinate: 0.0,
            visible: true,
            bubble_loc: "End".to_string(),
        },
    ];

    let native = pack_model_to_native_sdb(&m).expect("grid unicode IDs must succeed");
    let restored = extract_sdb_to_model(&native).expect("native import failed");
    let r_grids = &restored.coord_systems["GLOBAL"].grids;
    assert_eq!(r_grids[0].id, "Grid 1 (East Wing / Phase A)");
    assert_eq!(r_grids[1].id, "轴线-A1 📐");
    assert_eq!(r_grids[2].id, "Ось 1.0 (Главная)");
    assert_eq!(restored, m);
}

#[test]
fn test_grids_1000_lines_stress() {
    let mut m = base_model();
    let mut expected_grids = Vec::with_capacity(1000);
    for i in 0..400 {
        expected_grids.push(GridLine {
            axis: "X".to_string(),
            id: format!("X_{i}"),
            coordinate: (i as f64) * 0.5 - 100.0,
            visible: i % 2 == 0,
            bubble_loc: if i % 3 == 0 { "Start" } else { "End" }.to_string(),
        });
    }
    for i in 0..400 {
        expected_grids.push(GridLine {
            axis: "Y".to_string(),
            id: format!("Y_{i}"),
            coordinate: (i as f64) * 0.5 - 100.0,
            visible: true,
            bubble_loc: "End".to_string(),
        });
    }
    for i in 0..200 {
        expected_grids.push(GridLine {
            axis: "Z".to_string(),
            id: format!("Z_{i}"),
            coordinate: (i as f64) * 0.25,
            visible: false,
            bubble_loc: "Start".to_string(),
        });
    }
    m.coord_systems.get_mut("GLOBAL").unwrap().grids = expected_grids.clone();

    let native = pack_model_to_native_sdb(&m).expect("native export of 1000 grid lines failed");
    let doc = inspect_native_sdb(&native).expect("inspect failed");
    assert_eq!(doc.inventory().parsed_bytes, doc.inventory().stream_bytes);

    let restored = extract_sdb_to_model(&native).expect("native import failed");
    let r_grids = &restored.coord_systems["GLOBAL"].grids;
    assert_eq!(r_grids.len(), 1000);
    for (i, (orig, rest)) in expected_grids.iter().zip(r_grids.iter()).enumerate() {
        assert_eq!(orig.id, rest.id, "ID mismatch at index {i}");
        assert_eq!(
            orig.coordinate, rest.coordinate,
            "Coord mismatch at index {i}"
        );
        assert_eq!(orig.axis, rest.axis, "Axis mismatch at index {i}");
        assert_eq!(orig.visible, rest.visible, "Visible mismatch at index {i}");
        assert_eq!(
            orig.bubble_loc, rest.bubble_loc,
            "Bubble mismatch at index {i}"
        );
    }
    assert_eq!(restored, m);
}

#[test]
fn test_grids_multiple_coordinate_systems() {
    let mut m = base_model();
    let global_cs = m.coord_systems.get_mut("GLOBAL").unwrap();
    global_cs.grids = vec![
        GridLine {
            axis: "X".to_string(),
            id: "GX1".to_string(),
            coordinate: 0.0,
            visible: true,
            bubble_loc: "Start".to_string(),
        },
        GridLine {
            axis: "Y".to_string(),
            id: "GY1".to_string(),
            coordinate: 5.0,
            visible: true,
            bubble_loc: "End".to_string(),
        },
    ];

    m.coord_systems.insert(
        "CSYS_TOWER".to_string(),
        CoordSystem {
            name: "CSYS_TOWER".to_string(),
            origin: [100.0, 200.0, 50.0],
            angles_deg: [45.0, 0.0, 0.0],
            grids: vec![
                GridLine {
                    axis: "X".to_string(),
                    id: "TX1".to_string(),
                    coordinate: 0.0,
                    visible: true,
                    bubble_loc: "Start".to_string(),
                },
                GridLine {
                    axis: "X".to_string(),
                    id: "TX2".to_string(),
                    coordinate: 12.0,
                    visible: true,
                    bubble_loc: "End".to_string(),
                },
                GridLine {
                    axis: "Z".to_string(),
                    id: "TZ1".to_string(),
                    coordinate: 3.5,
                    visible: false,
                    bubble_loc: "End".to_string(),
                },
            ],
        },
    );

    let native =
        pack_model_to_native_sdb(&m).expect("native export with multiple coord systems failed");
    let doc = inspect_native_sdb(&native).expect("inspect failed");
    assert_eq!(doc.inventory().parsed_bytes, doc.inventory().stream_bytes);

    let restored = extract_sdb_to_model(&native).expect("native import failed");
    assert_eq!(restored.coord_systems.len(), 2);

    let r_global = &restored.coord_systems["GLOBAL"];
    assert_eq!(r_global.grids.len(), 2);
    assert_eq!(r_global.grids[0].id, "GX1");
    assert_eq!(r_global.grids[1].id, "GY1");

    let r_tower = &restored.coord_systems["CSYS_TOWER"];
    assert_eq!(r_tower.origin, [100.0, 200.0, 50.0]);
    assert_eq!(r_tower.angles_deg, [45.0, 0.0, 0.0]);
    assert_eq!(r_tower.grids.len(), 3);
    assert_eq!(r_tower.grids[0].id, "TX1");
    assert_eq!(r_tower.grids[1].id, "TX2");
    assert_eq!(r_tower.grids[2].id, "TZ1");

    assert_eq!(restored, m);
}
