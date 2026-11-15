//! Function-file reader. Expected values are typed from the in-test text or,
//! for the opt-in vendor files, from what the files themselves state.

use sap_format::function_file::{
    parse_function_text, read_function_file, FunctionFileError, FunctionFileOptions, ItemFormat,
    ValueLayout,
};
use std::path::PathBuf;

fn options(
    header_lines: usize,
    prefix_chars: usize,
    points_per_line: usize,
    format: ItemFormat,
    layout: ValueLayout,
) -> FunctionFileOptions {
    FunctionFileOptions {
        header_lines,
        prefix_chars,
        points_per_line,
        format,
        layout,
    }
}

const EQUAL: ValueLayout = ValueLayout::EqualInterval { dt: 0.02 };

#[test]
fn equal_interval_file_with_two_header_lines_and_eight_values_per_line() {
    // Two header lines, then eight F10.3 values per line; last line has three.
    let text = "HEADER LINE ONE 1 2 3\n\
                10 POINTS OF ACCEL DATA EQUALLY SPACED AT .020 SEC.\n\
                \x20   -3.035    -1.032    -1.224    -3.023    -5.213     1.536     7.311     5.159\n\
                \x20     .633    -2.967     1.503\n";
    let expected = [
        -3.035, -1.032, -1.224, -3.023, -5.213, 1.536, 7.311, 5.159, 0.633, -2.967, 1.503,
    ];
    for format in [ItemFormat::Free, ItemFormat::Fixed { chars_per_item: 10 }] {
        let points = parse_function_text(text, &options(2, 0, 8, format, EQUAL)).unwrap();
        assert_eq!(points.len(), expected.len());
        for (i, (t, v)) in points.iter().enumerate() {
            assert_eq!(*v, expected[i]);
            assert!((t - i as f64 * 0.02).abs() <= 1e-15, "t[{i}] = {t}");
        }
    }
}

#[test]
fn pair_file_with_crlf_line_ends_and_trailing_ctrl_z() {
    let text = "   0.00000    .01080    .04200    .00100    .09700    .01590\r\n\
                \x20   .16100   -.00010    .22100    .01890\r\n\x1a";
    let points = parse_function_text(
        text,
        &options(0, 0, 3, ItemFormat::Free, ValueLayout::Pairs),
    )
    .unwrap();
    assert_eq!(
        points,
        vec![
            (0.0, 0.0108),
            (0.042, 0.001),
            (0.097, 0.0159),
            (0.161, -0.0001),
            (0.221, 0.0189)
        ]
    );
}

#[test]
fn extra_items_on_a_line_are_ignored() {
    // Two pairs are read, the third pair (and a word) is ignored.
    let text = "0 1 1 2 2 3 junk\n3 4 4 5 5 6 junk\n";
    let points = parse_function_text(
        text,
        &options(0, 0, 2, ItemFormat::Free, ValueLayout::Pairs),
    )
    .unwrap();
    assert_eq!(points, vec![(0.0, 1.0), (1.0, 2.0), (3.0, 4.0), (4.0, 5.0)]);
}

#[test]
fn dollar_lines_are_skipped_and_a_blank_line_stops_reading() {
    let text = "$ comment\n0 1\n$ another comment 7 7\n1 2\n\n2 3\n";
    let points = parse_function_text(
        text,
        &options(0, 0, 1, ItemFormat::Free, ValueLayout::Pairs),
    )
    .unwrap();
    assert_eq!(points, vec![(0.0, 1.0), (1.0, 2.0)]);
    // A header line may itself start with $ or be blank: it is skipped first.
    let text = "\n$ header\n0 5\n";
    let points = parse_function_text(
        text,
        &options(2, 0, 1, ItemFormat::Free, ValueLayout::Pairs),
    )
    .unwrap();
    assert_eq!(points, vec![(0.0, 5.0)]);
    // A line holding only ^Z ends the data.
    let points = parse_function_text(
        "0 1\n\x1a\n5 5\n",
        &options(0, 0, 1, ItemFormat::Free, ValueLayout::Pairs),
    )
    .unwrap();
    assert_eq!(points, vec![(0.0, 1.0)]);
}

#[test]
fn prefix_characters_are_skipped_before_free_and_fixed_items() {
    // Five-character line numbers in front of the data.
    let text = "  001 0.5 1.5\n  002 1.0 2.5\n";
    let free = parse_function_text(
        text,
        &options(0, 5, 1, ItemFormat::Free, ValueLayout::Pairs),
    )
    .unwrap();
    assert_eq!(free, vec![(0.5, 1.5), (1.0, 2.5)]);
    // Fixed format counts characters after the prefix; adjacent numbers with
    // no blank between them are separated only by position.
    let text = "AB-100.123-200.456\nAB 300.789 400.012\n";
    let fixed = parse_function_text(
        text,
        &options(
            0,
            2,
            1,
            ItemFormat::Fixed { chars_per_item: 8 },
            ValueLayout::Pairs,
        ),
    );
    // Items: "-100.123", "-200.456" -> time -100.123, value -200.456, then
    // " 300.789", " 400.012".
    assert_eq!(
        fixed.unwrap(),
        vec![(-100.123, -200.456), (300.789, 400.012)]
    );
}

#[test]
fn fortran_d_exponent_and_commas_read() {
    let points = parse_function_text(
        "0.0, 1.0D+01\n1.0,2.5d-1\n",
        &options(0, 0, 1, ItemFormat::Free, ValueLayout::Pairs),
    )
    .unwrap();
    assert_eq!(points, vec![(0.0, 10.0), (1.0, 0.25)]);
}

#[test]
fn errors_carry_the_line_number() {
    let free_pairs = options(1, 0, 2, ItemFormat::Free, ValueLayout::Pairs);
    // Non-numeric item on line 3 (the header line counts).
    let error = parse_function_text("title\n0 1\n1 abc\n", &free_pairs).unwrap_err();
    assert_eq!(
        error,
        FunctionFileError::NonNumeric {
            line: 3,
            item: "abc".into()
        }
    );
    // Odd number of items among the ones read.
    let error = parse_function_text("title\n0 1\n1 2 3\n", &free_pairs).unwrap_err();
    assert_eq!(error, FunctionFileError::OddItemCount { line: 3, count: 3 });
    // Equal and decreasing times.
    let error = parse_function_text("title\n0 1 1 2\n1 5\n", &free_pairs).unwrap_err();
    assert_eq!(
        error,
        FunctionFileError::NonIncreasingTime {
            line: 3,
            previous: 1.0,
            time: 1.0
        }
    );
    let error = parse_function_text("title\n0 1 2 2\n1 5\n", &free_pairs).unwrap_err();
    assert!(matches!(
        error,
        FunctionFileError::NonIncreasingTime { line: 3, .. }
    ));
    // Not-a-number and infinity are not accepted as values.
    for bad in ["nan", "inf", "-inf"] {
        let text = format!("title\n0 {bad}\n");
        assert!(
            matches!(
                parse_function_text(&text, &free_pairs),
                Err(FunctionFileError::NonNumeric { line: 2, .. })
            ),
            "{bad}"
        );
    }
    // No values at all: only a header, only comments, or a blank first line.
    for text in ["title\n", "title\n$ c\n", "title\n\n0 1\n", ""] {
        assert_eq!(
            parse_function_text(text, &free_pairs).unwrap_err(),
            FunctionFileError::NoValues,
            "{text:?}"
        );
    }
    // A data line shorter than the prefix has nothing to read.
    let error = parse_function_text(
        "abc\n",
        &options(0, 5, 1, ItemFormat::Free, ValueLayout::Pairs),
    )
    .unwrap_err();
    assert_eq!(error, FunctionFileError::EmptyLine { line: 1 });
    // Fixed format: an interior blank item is an error, not a skipped value.
    let error = parse_function_text(
        "1.0       2.0\n",
        &options(0, 0, 3, ItemFormat::Fixed { chars_per_item: 5 }, EQUAL),
    )
    .unwrap_err();
    assert_eq!(
        error,
        FunctionFileError::NonNumeric {
            line: 1,
            item: String::new()
        }
    );
}

#[test]
fn invalid_options_are_refused() {
    let bad = [
        options(0, 0, 0, ItemFormat::Free, ValueLayout::Pairs),
        options(
            0,
            0,
            1,
            ItemFormat::Fixed { chars_per_item: 0 },
            ValueLayout::Pairs,
        ),
        options(
            0,
            0,
            1,
            ItemFormat::Free,
            ValueLayout::EqualInterval { dt: 0.0 },
        ),
        options(
            0,
            0,
            1,
            ItemFormat::Free,
            ValueLayout::EqualInterval { dt: f64::NAN },
        ),
        options(
            0,
            0,
            1,
            ItemFormat::Free,
            ValueLayout::EqualInterval { dt: -0.01 },
        ),
    ];
    for o in &bad {
        assert!(
            matches!(
                parse_function_text("1 2\n", o),
                Err(FunctionFileError::InvalidOptions(_))
            ),
            "{o:?}"
        );
    }
}

#[test]
fn missing_file_is_an_io_error() {
    let error = read_function_file(
        std::path::Path::new("/nonexistent/structurax/function.th"),
        &FunctionFileOptions::default(),
    )
    .unwrap_err();
    assert!(matches!(error, FunctionFileError::Io(_)));
}

// ---------------------------------------------------------------------------
// Opt-in vendor files (STRUCTURAX_VENDOR_FIXTURES = directory with the files)
// ---------------------------------------------------------------------------

fn vendor_file(name: &str) -> Option<PathBuf> {
    let dir = PathBuf::from(std::env::var_os("STRUCTURAX_VENDOR_FIXTURES")?);
    let path = dir.join(name);
    if path.is_file() {
        Some(path)
    } else {
        eprintln!("skipped: {} not found", path.display());
        None
    }
}

/// The count and interval a `.th` file states in its second header line
/// ("2000 POINTS OF ACCEL DATA EQUALLY SPACED AT  .020 SEC.").
fn header_count_and_interval(text: &str) -> (usize, f64) {
    let line = text.lines().nth(1).expect("second header line");
    let words: Vec<&str> = line.split_whitespace().collect();
    let count: usize = words[0].parse().expect("point count");
    let at = words.iter().position(|w| *w == "AT").expect("AT");
    let dt: f64 = words[at + 1].parse().expect("interval");
    (count, dt)
}

#[test]
fn vendor_th_files_have_the_point_count_and_interval_their_header_states() {
    let Some(dir) = std::env::var_os("STRUCTURAX_VENDOR_FIXTURES").map(PathBuf::from) else {
        eprintln!("skipped: STRUCTURAX_VENDOR_FIXTURES is not set");
        return;
    };
    let mut checked = 0;
    let mut names: Vec<_> = std::fs::read_dir(&dir)
        .expect("vendor directory")
        .filter_map(|e| e.ok())
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .filter(|n| n.ends_with(".th"))
        .collect();
    names.sort();
    for name in names {
        let path = dir.join(&name);
        let text = String::from_utf8_lossy(&std::fs::read(&path).unwrap()).into_owned();
        let (count, dt) = header_count_and_interval(&text);
        // Fixed format (10 characters per value) and free format must agree.
        let fixed = read_function_file(
            &path,
            &options(
                2,
                0,
                8,
                ItemFormat::Fixed { chars_per_item: 10 },
                ValueLayout::EqualInterval { dt },
            ),
        )
        .unwrap_or_else(|e| panic!("{name}: {e}"));
        let free = read_function_file(
            &path,
            &options(2, 0, 8, ItemFormat::Free, ValueLayout::EqualInterval { dt }),
        )
        .unwrap_or_else(|e| panic!("{name}: {e}"));
        assert_eq!(fixed.len(), count, "{name} point count");
        assert_eq!(fixed, free, "{name} fixed vs free");
        let (t_last, _) = fixed[count - 1];
        assert!(
            (t_last - (count - 1) as f64 * dt).abs() <= 1e-9 * t_last.max(1.0),
            "{name} duration"
        );
        // The number of data items in the body, counted without the reader.
        let items = text
            .lines()
            .skip(2)
            .flat_map(|l| l.split_whitespace())
            .filter(|w| !w.contains('\x1a'))
            .count();
        assert_eq!(items, count, "{name} items in file");
        checked += 1;
    }
    eprintln!("checked {checked} .th files");
    assert!(checked > 0, "no .th files in the vendor directory");
}

#[test]
fn vendor_elcentro_reads_pairs_with_increasing_times() {
    for name in ["elcentro", "elcentro1"] {
        let Some(path) = vendor_file(name) else {
            return;
        };
        let text = String::from_utf8_lossy(&std::fs::read(&path).unwrap()).into_owned();
        let points = read_function_file(
            &path,
            &options(0, 0, 3, ItemFormat::Free, ValueLayout::Pairs),
        )
        .unwrap_or_else(|e| panic!("{name}: {e}"));
        let items = text
            .lines()
            .flat_map(|l| l.split_whitespace())
            .filter(|w| !w.contains('\x1a'))
            .count();
        assert_eq!(points.len() * 2, items, "{name}");
        assert!(points.windows(2).all(|w| w[0].0 < w[1].0), "{name}");
        // First line of the file: 0.00000 .01080 .04200 .00100 .09700 .01590
        assert_eq!(points[0], (0.0, 0.0108), "{name}");
        assert_eq!(points[1], (0.042, 0.001), "{name}");
        assert_eq!(points[2], (0.097, 0.0159), "{name}");
        // Example 1-022 runs 8 s of this record (CSI verification manual).
        assert!(points.last().unwrap().0 >= 8.0, "{name}");
    }
}
