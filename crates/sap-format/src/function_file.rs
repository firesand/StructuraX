//! Time-history and response-spectrum function values read from a text file.
//!
//! The reading rules are those of the SAP2000 help topic "Time History
//! Function from File" (Define > Functions > Time History):
//!
//! 1. the given number of header lines is skipped;
//! 2. a line whose first character is `$` is skipped;
//! 3. a blank line, or the end of the file, stops the reading;
//! 4. on every other line the given number of prefix characters is skipped,
//!    then at most N values (equal-interval files) or N (time, value) pairs
//!    (paired files) are read; further items on the line are ignored;
//! 5. in free format the items are separated by blanks or tabs (commas are
//!    accepted as separators too); in fixed format every item occupies the
//!    same number of characters, counted after the prefix.
//!
//! Carriage returns and the DOS end-of-file character (`^Z`, `0x1A`) are
//! removed from every line before it is interpreted, so a file written with
//! CR LF line ends and closed with `^Z` reads like a plain one.
//!
//! Assumption not stated in the help topic: for equal-interval files the first
//! value belongs to time `t = 0`, the next to `Δt`, and so on. The result is a
//! list of `(time, value)` points, the form of `DynamicFunction::points`.

use std::fmt;
use std::path::Path;

/// How the items of a line are separated.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ItemFormat {
    /// Items separated by blanks, tabs or commas.
    Free,
    /// Every item takes `chars_per_item` characters (after the prefix).
    Fixed { chars_per_item: usize },
}

/// What the numbers of the file mean.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ValueLayout {
    /// Pairs of time and function value.
    Pairs,
    /// Function values at equal time intervals `dt`, the first one at `t = 0`.
    EqualInterval { dt: f64 },
}

/// Reading options, the fields of the "Time History Function Definition -
/// from File" form.
#[derive(Debug, Clone, PartialEq)]
pub struct FunctionFileOptions {
    /// Lines skipped at the top of the file.
    pub header_lines: usize,
    /// Characters skipped at the start of every data line.
    pub prefix_chars: usize,
    /// Values (or pairs) read from each line; further items are ignored.
    pub points_per_line: usize,
    pub format: ItemFormat,
    pub layout: ValueLayout,
}

impl Default for FunctionFileOptions {
    fn default() -> Self {
        Self {
            header_lines: 0,
            prefix_chars: 0,
            points_per_line: 1,
            format: ItemFormat::Free,
            layout: ValueLayout::Pairs,
        }
    }
}

/// Why a function file could not be read. Line numbers start at 1 and count
/// the header lines.
#[derive(Debug, Clone, PartialEq)]
pub enum FunctionFileError {
    Io(String),
    InvalidOptions(String),
    NonNumeric {
        line: usize,
        item: String,
    },
    /// A pair file line with an odd number of items among the ones read.
    OddItemCount {
        line: usize,
        count: usize,
    },
    NonIncreasingTime {
        line: usize,
        previous: f64,
        time: f64,
    },
    /// A data line from which no item could be read.
    EmptyLine {
        line: usize,
    },
    NoValues,
}

impl fmt::Display for FunctionFileError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(reason) => write!(f, "cannot read function file: {reason}"),
            Self::InvalidOptions(reason) => write!(f, "invalid function file options: {reason}"),
            Self::NonNumeric { line, item } => {
                write!(f, "line {line}: '{item}' is not a finite number")
            }
            Self::OddItemCount { line, count } => write!(
                f,
                "line {line}: {count} items cannot form (time, value) pairs"
            ),
            Self::NonIncreasingTime {
                line,
                previous,
                time,
            } => write!(
                f,
                "line {line}: time {time} does not exceed the previous time {previous}"
            ),
            Self::EmptyLine { line } => write!(f, "line {line}: no values after the prefix"),
            Self::NoValues => write!(f, "the file contains no function values"),
        }
    }
}

impl std::error::Error for FunctionFileError {}

fn validate_options(options: &FunctionFileOptions) -> Result<(), FunctionFileError> {
    let invalid = |reason: &str| Err(FunctionFileError::InvalidOptions(reason.to_string()));
    if options.points_per_line == 0 {
        return invalid("points per line must be at least 1");
    }
    if let ItemFormat::Fixed { chars_per_item } = options.format {
        if chars_per_item == 0 {
            return invalid("characters per item must be at least 1");
        }
    }
    if let ValueLayout::EqualInterval { dt } = options.layout {
        if !dt.is_finite() || dt <= 0.0 {
            return invalid("the time interval must be finite and positive");
        }
    }
    Ok(())
}

fn parse_item(text: &str, line: usize) -> Result<f64, FunctionFileError> {
    let trimmed = text.trim();
    // Fortran writers use D for the exponent.
    let normalized = trimmed.replace(['D', 'd'], "e");
    match normalized.parse::<f64>() {
        Ok(value) if value.is_finite() => Ok(value),
        _ => Err(FunctionFileError::NonNumeric {
            line,
            item: trimmed.to_string(),
        }),
    }
}

/// The items of one line, at most `limit` of them.
fn line_items(
    text: &str,
    format: ItemFormat,
    limit: usize,
    line: usize,
) -> Result<Vec<f64>, FunctionFileError> {
    let mut values = Vec::new();
    match format {
        ItemFormat::Free => {
            for item in text
                .split(|c: char| c.is_whitespace() || c == ',')
                .filter(|s| !s.is_empty())
                .take(limit)
            {
                values.push(parse_item(item, line)?);
            }
        }
        ItemFormat::Fixed { chars_per_item } => {
            let chars: Vec<char> = text.chars().collect();
            // Blank columns at the end of the line are not items.
            let used = chars
                .iter()
                .rposition(|c| !c.is_whitespace())
                .map_or(0, |p| p + 1);
            for chunk in chars[..used].chunks(chars_per_item).take(limit) {
                let item: String = chunk.iter().collect();
                if item.trim().is_empty() {
                    return Err(FunctionFileError::NonNumeric {
                        line,
                        item: String::new(),
                    });
                }
                values.push(parse_item(&item, line)?);
            }
        }
    }
    Ok(values)
}

/// Reads the `(time, value)` points of function-file text.
pub fn parse_function_text(
    text: &str,
    options: &FunctionFileOptions,
) -> Result<Vec<(f64, f64)>, FunctionFileError> {
    validate_options(options)?;
    let mut points: Vec<(f64, f64)> = Vec::new();
    let mut equal_index = 0usize;
    for (index, raw) in text.split('\n').enumerate() {
        let line = index + 1;
        if line <= options.header_lines {
            continue;
        }
        let cleaned: String = raw.chars().filter(|c| *c != '\r' && *c != '\x1a').collect();
        if cleaned.starts_with('$') {
            continue;
        }
        if cleaned.trim().is_empty() {
            break;
        }
        let body: String = cleaned.chars().skip(options.prefix_chars).collect();
        match options.layout {
            ValueLayout::Pairs => {
                let items = line_items(&body, options.format, 2 * options.points_per_line, line)?;
                if items.is_empty() {
                    return Err(FunctionFileError::EmptyLine { line });
                }
                if items.len() % 2 != 0 {
                    return Err(FunctionFileError::OddItemCount {
                        line,
                        count: items.len(),
                    });
                }
                for pair in items.chunks(2) {
                    if let Some(&(previous, _)) = points.last() {
                        if pair[0] <= previous {
                            return Err(FunctionFileError::NonIncreasingTime {
                                line,
                                previous,
                                time: pair[0],
                            });
                        }
                    }
                    points.push((pair[0], pair[1]));
                }
            }
            ValueLayout::EqualInterval { dt } => {
                let items = line_items(&body, options.format, options.points_per_line, line)?;
                if items.is_empty() {
                    return Err(FunctionFileError::EmptyLine { line });
                }
                for value in items {
                    points.push((equal_index as f64 * dt, value));
                    equal_index += 1;
                }
            }
        }
    }
    if points.is_empty() {
        return Err(FunctionFileError::NoValues);
    }
    Ok(points)
}

/// Reads the function file at `path`. Bytes that are not valid UTF-8 are
/// replaced, so a legacy 8-bit header does not stop the reading.
pub fn read_function_file(
    path: &Path,
    options: &FunctionFileOptions,
) -> Result<Vec<(f64, f64)>, FunctionFileError> {
    let bytes = std::fs::read(path)
        .map_err(|error| FunctionFileError::Io(format!("{}: {error}", path.display())))?;
    parse_function_text(&String::from_utf8_lossy(&bytes), options)
}
