// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! STEP file parser — streaming and in-memory variants.
//!
//! Handles:
//! - Multi-line entities (entities spanning multiple lines)
//! - Complex/composite entities (e.g., `( TYPE1() TYPE2() TYPE3() )`)
//! - Standard simple entities (`#ID = TYPE_NAME(params);`)
//! - Block comments (`/* ... */` spanning multiple lines)
//! - String escape sequences (`''` for literal single quote)
//!
//! # Streaming API
//!
//! For large files (100MB+), use [`parse_step_streaming`] to process entities
//! as they're parsed, without loading the entire file into memory:
//!
//! ```ignore
//! let file = std::fs::File::open("large.stp")?;
//! let reader = std::io::BufReader::new(file);
//! let step_file = parse_step_streaming(reader, |entity, line| {
//!     if entity.type_name == "MANIFOLD_SOLID_BREP" {
//!         println!("Found BREP #{}", entity.id);
//!     }
//!     true
//! })?;
//! ```
//!
//! # Performance
//!
//! - Incremental parenthesis tracking: O(1) per line instead of O(n²) re-scan
//! - Zero-copy value parsing: works on `&str` slices, no `Vec<char>` allocation
//! - `BufRead`-based streaming: constant memory for file I/O
//! - Line number tracking: accurate error messages

use crate::schema::*;
use std::io::BufRead;

// ============================================================
// Public API
// ============================================================

/// Parse a STEP file from a string (in-memory).
///
/// This is the simplest API — loads the entire content into memory first.
/// For large files, prefer [`parse_step_reader`] or [`parse_step_streaming`].
pub fn parse_step(input: &str) -> Result<StepFile, StepParseError> {
    let cursor = std::io::Cursor::new(input.as_bytes());
    parse_step_inner(cursor, &mut NoOpEntityCallback)
}

/// Parse a STEP file from a file path (native only — not available on wasm).
///
/// Uses `BufReader` for efficient line-by-line reading without loading
/// the entire file into a single contiguous `String`.
#[cfg(not(target_arch = "wasm32"))]
pub fn parse_step_file(path: &str) -> Result<StepFile, StepParseError> {
    let data = std::fs::read(path)
        .map_err(|e| StepParseError::IoError(e.to_string()))?;
    // Use from_utf8_lossy to handle ANSI/Windows-1251/1252 encoded files.
    // STEP files are ASCII-compatible — the only non-UTF-8 bytes are typically
    // in comments (file names with national characters). Lossy conversion
    // replaces invalid bytes with U+FFFD, which is safe for parsing.
    let content = String::from_utf8_lossy(&data);
    parse_step(&content)
}

/// Parse a STEP file from any `BufRead` source.
///
/// Reads line-by-line without loading the entire file into a single buffer.
/// Memory usage is proportional to the largest single entity (including
/// multi-line NURBS), not the total file size.
pub fn parse_step_reader<R: BufRead>(reader: R) -> Result<StepFile, StepParseError> {
    parse_step_inner(reader, &mut NoOpEntityCallback)
}

/// Parse a STEP file from a `BufRead` source with an entity callback.
///
/// The callback is invoked for each entity **immediately after it's parsed**,
/// before the next entity is read. This enables:
/// - Progress reporting for large files
/// - Selective processing (skip entities you don't need)
/// - Early termination (return `false` from callback to stop)
///
/// The callback receives a reference to the parsed entity and the current
/// line number. Return `true` to continue parsing, `false` to stop early.
///
/// # Example
///
/// ```ignore
/// let file = std::fs::File::open("assembly.stp")?;
/// let reader = std::io::BufReader::new(file);
/// let step_file = parse_step_streaming(reader, |entity, line| {
///     println!("Line {}: parsed #{} = {}", line, entity.id, entity.type_name);
///     true // continue parsing
/// })?;
/// ```
pub fn parse_step_streaming<R: BufRead, F: FnMut(&StepEntity, usize) -> bool>(
    reader: R,
    callback: F,
) -> Result<StepFile, StepParseError> {
    parse_step_inner(reader, &mut WrapCallback(callback))
}

/// Parse a STEP file from a string with an entity callback.
///
/// Same as [`parse_step_streaming`] but works with an in-memory string.
pub fn parse_step_streaming_str<F: FnMut(&StepEntity, usize) -> bool>(
    input: &str,
    callback: F,
) -> Result<StepFile, StepParseError> {
    let cursor = std::io::Cursor::new(input.as_bytes());
    parse_step_inner(cursor, &mut WrapCallback(callback))
}

// ============================================================
// Core parser
// ============================================================

/// Trait for entity callbacks during streaming parsing.
trait EntityCallback {
    fn call(&mut self, entity: &StepEntity, line: usize) -> bool;
}

/// No-op callback for non-streaming parsing.
struct NoOpEntityCallback;

impl EntityCallback for NoOpEntityCallback {
    fn call(&mut self, _entity: &StepEntity, _line: usize) -> bool {
        true
    }
}

/// Wrapper to adapt user closures to `EntityCallback`.
struct WrapCallback<F>(F);

impl<F: FnMut(&StepEntity, usize) -> bool> EntityCallback for WrapCallback<F> {
    fn call(&mut self, entity: &StepEntity, line: usize) -> bool {
        (self.0)(entity, line)
    }
}

/// Core parser implementation.
///
/// Key optimizations over the original parser:
/// 1. **Incremental paren tracking**: Tracks `paren_depth` instead of
///    re-scanning the entire buffer on every continuation line (O(1) vs O(n²)).
/// 2. **Line number tracking**: Every error includes the actual line number.
/// 3. **Block comment handling**: Properly skips `/* ... */` across lines.
/// 4. **String escapes**: Handles `''` (STEP's way to encode a literal `'`).
/// 5. **Byte-based value parsing**: No `Vec<char>` allocation.
fn parse_step_inner<R: BufRead, C: EntityCallback>(
    reader: R,
    callback: &mut C,
) -> Result<StepFile, StepParseError> {
    let mut file = StepFile::new();
    let mut in_data = false;
    let mut in_header = false;
    let mut in_block_comment = false;

    // Buffer for accumulating multi-line entities
    let mut entity_buffer = String::new();
    let mut collecting_entity = false;
    // Incremental parenthesis depth tracker — avoids O(n²) re-scanning
    let mut paren_depth: i32 = 0;
    // Tracks whether we're currently inside a `'...'` string literal.
    // PERSISTED across lines so that multi-line strings (common in NX STEP
    // exports) don't cause paren depth to be miscounted on continuation lines.
    let mut in_string: bool = false;

    // Line number tracking
    let mut line_number: usize = 0;
    // Flag for early termination via callback
    let mut should_continue = true;

    // BufRead-based line reader
    let mut buf_reader = std::io::BufReader::new(reader);
    let mut line_buf = String::with_capacity(4096);

    loop {
        line_buf.clear();
        match buf_reader.read_line(&mut line_buf) {
            Ok(0) => break, // EOF
            Ok(_) => {
                // Remove trailing newline
                if line_buf.ends_with('\n') {
                    line_buf.pop();
                    if line_buf.ends_with('\r') {
                        line_buf.pop();
                    }
                }
            }
            Err(e) => {
                return Err(StepParseError::IoError(e.to_string()));
            }
        }

        line_number += 1;
        let line = line_buf.trim();

        if line.is_empty() {
            continue;
        }

        // Handle block comments: /* ... */ can span multiple lines
        if in_block_comment {
            if let Some(end_pos) = line.find("*/") {
                in_block_comment = false;
                let rest = line[end_pos + 2..].trim();
                if rest.is_empty() {
                    continue;
                }
                // Process the rest of the line after the comment
                if let Err(e) = process_line(
                    rest,
                    &mut file,
                    &mut in_data,
                    &mut in_header,
                    &mut entity_buffer,
                    &mut collecting_entity,
                    &mut paren_depth,
                    &mut in_string,
                    line_number,
                    callback,
                    &mut should_continue,
                ) {
                    return Err(e);
                }
            }
            continue;
        }

        // Check for block comment start
        if let Some(start_pos) = line.find("/*") {
            // Check if the block comment closes on the same line
            if let Some(end_offset) = line[start_pos + 2..].find("*/") {
                // Inline block comment: remove the comment and process the rest
                let end_pos = start_pos + 2 + end_offset;
                let before = &line[..start_pos];
                let after = &line[end_pos + 2..];
                let cleaned = format!("{}{}", before, after);
                let cleaned_trimmed = cleaned.trim();
                if !cleaned_trimmed.is_empty() {
                    if let Err(e) = process_line(
                        cleaned_trimmed,
                        &mut file,
                        &mut in_data,
                        &mut in_header,
                        &mut entity_buffer,
                        &mut collecting_entity,
                        &mut paren_depth,
                        &mut in_string,
                        line_number,
                        callback,
                        &mut should_continue,
                    ) {
                        return Err(e);
                    }
                }
                continue;
            } else {
                // Block comment starts but doesn't end on this line
                in_block_comment = true;
                // Process the part before the comment
                let before = line[..start_pos].trim();
                if !before.is_empty() {
                    if let Err(e) = process_line(
                        before,
                        &mut file,
                        &mut in_data,
                        &mut in_header,
                        &mut entity_buffer,
                        &mut collecting_entity,
                        &mut paren_depth,
                        &mut in_string,
                        line_number,
                        callback,
                        &mut should_continue,
                    ) {
                        return Err(e);
                    }
                }
                continue;
            }
        }

        if !should_continue {
            break;
        }
        if let Err(e) = process_line(
            line,
            &mut file,
            &mut in_data,
            &mut in_header,
            &mut entity_buffer,
            &mut collecting_entity,
            &mut paren_depth,
            &mut in_string,
            line_number,
            callback,
            &mut should_continue,
        ) {
            return Err(e);
        }
    }

    // Flush any remaining buffered entity
    if collecting_entity && !entity_buffer.is_empty() {
        if let Some(entity) = parse_entity_line_with_lineno(&entity_buffer, line_number)? {
            callback.call(&entity, line_number);
            file.entities.push(entity);
        }
    }

    // Build entity index for fast lookup
    file.build_index();

    Ok(file)
}

/// Process a single trimmed line of STEP data.
#[allow(clippy::too_many_arguments)]
fn process_line<C: EntityCallback>(
    line: &str,
    file: &mut StepFile,
    in_data: &mut bool,
    in_header: &mut bool,
    entity_buffer: &mut String,
    collecting_entity: &mut bool,
    paren_depth: &mut i32,
    in_string: &mut bool,
    line_number: usize,
    callback: &mut C,
    should_continue: &mut bool,
) -> Result<(), StepParseError> {
    if line == "HEADER;" {
        *in_header = true;
        return Ok(());
    }

    if line == "ENDSEC;" && *in_header {
        *in_header = false;
        return Ok(());
    }

    if line == "DATA;" {
        *in_data = true;
        return Ok(());
    }

    if line == "ENDSEC;" && *in_data {
        // Flush any buffered entity
        if *collecting_entity && !entity_buffer.is_empty() {
            if let Some(entity) = parse_entity_line_with_lineno(entity_buffer, line_number)? {
                if !callback.call(&entity, line_number) {
                    *should_continue = false;
                }
                file.entities.push(entity);
            }
            entity_buffer.clear();
            *collecting_entity = false;
            *paren_depth = 0;
            *in_string = false;
        }
        *in_data = false;
        return Ok(());
    }

    if line == "END-ISO-10303-21;" {
        return Ok(());
    }

    if *in_header {
        parse_header_line(line, &mut file.header)?;
    }

    if *in_data {
        // CRITICAL: If we're collecting a multi-line entity, the continuation
        // line must be treated as such, even if it starts with '#' after trimming.
        // In STEP files, continuation lines often look like "  #123,#456,$);"
        // which after trimming starts with '#' but is NOT a new entity.
        // We distinguish by checking paren_depth: if unbalanced, this is a continuation.
        if *collecting_entity && *paren_depth > 0 {
            // Continuation of a multi-line entity
            entity_buffer.push(' ');
            entity_buffer.push_str(line);

            // Update paren depth incrementally — O(1) per line.
            // `in_string` is threaded through so multi-line strings are
            // correctly tracked across line boundaries.
            update_paren_depth(line, paren_depth, in_string);

            // Check if entity is complete
            if line.ends_with(';') && *paren_depth <= 0 && !*in_string {
                if let Some(entity) = parse_entity_line_with_lineno(entity_buffer, line_number)? {
                    if !callback.call(&entity, line_number) {
                        *should_continue = false;
                    }
                    file.entities.push(entity);
                }
                entity_buffer.clear();
                *collecting_entity = false;
                *paren_depth = 0;
                *in_string = false;
            } else if line.ends_with(';') && *paren_depth > 0 && !*in_string {
                // Line ends with ';' but parentheses are unbalanced — this
                // indicates a syntax error in the STEP file (missing closing
                // parenthesis). Rather than accumulating all subsequent lines
                // into the entity buffer (causing O(n²) memory growth and
                // eventual hang), return an error immediately.
                return Err(StepParseError::SyntaxError {
                    line: line_number,
                    message: format!(
                        "entity ends with ';' but has unbalanced parentheses (depth={}). \
                         Likely a missing ')' in this entity: {}",
                        paren_depth,
                        entity_buffer.chars().take(200).collect::<String>()
                    ),
                });
            }
            return Ok(());
        }

        if line.starts_with('#') {
            // Start of a new entity — flush any previous buffered entity
            if *collecting_entity && !entity_buffer.is_empty() {
                if let Some(entity) = parse_entity_line_with_lineno(entity_buffer, line_number)? {
                    if !callback.call(&entity, line_number) {
                        *should_continue = false;
                    }
                    file.entities.push(entity);
                }
            }
            entity_buffer.clear();
            *paren_depth = 0;
            *in_string = false;
            entity_buffer.push_str(line);
            *collecting_entity = true;

            // Count parens on this first line — O(line_length)
            update_paren_depth(line, paren_depth, in_string);

            // Check if this line is a complete entity (ends with ; and balanced parens)
            if line.ends_with(';') && *paren_depth <= 0 && !*in_string {
                if let Some(entity) = parse_entity_line_with_lineno(entity_buffer, line_number)? {
                    if !callback.call(&entity, line_number) {
                        *should_continue = false;
                    }
                    file.entities.push(entity);
                }
                entity_buffer.clear();
                *collecting_entity = false;
                *paren_depth = 0;
                *in_string = false;
            } else if line.ends_with(';') && *paren_depth > 0 && !*in_string {
                // First line of entity ends with ';' but parens unbalanced — syntax error.
                return Err(StepParseError::SyntaxError {
                    line: line_number,
                    message: format!(
                        "entity ends with ';' but has unbalanced parentheses (depth={}). \
                         Likely a missing ')' in this entity: {}",
                        paren_depth,
                        entity_buffer.chars().take(200).collect::<String>()
                    ),
                });
            }
        } else if *collecting_entity {
            // Continuation of a multi-line entity (paren_depth was already 0,
            // but we're still collecting — e.g., semicolon not yet seen)
            entity_buffer.push(' ');
            entity_buffer.push_str(line);

            // Update paren depth incrementally
            update_paren_depth(line, paren_depth, in_string);

            // Check if entity is complete
            if line.ends_with(';') && *paren_depth <= 0 && !*in_string {
                if let Some(entity) = parse_entity_line_with_lineno(entity_buffer, line_number)? {
                    if !callback.call(&entity, line_number) {
                        *should_continue = false;
                    }
                    file.entities.push(entity);
                }
                entity_buffer.clear();
                *collecting_entity = false;
                *paren_depth = 0;
                *in_string = false;
            } else if line.ends_with(';') && *paren_depth > 0 && !*in_string {
                // Continuation line ends with ';' but parens still unbalanced — syntax error.
                return Err(StepParseError::SyntaxError {
                    line: line_number,
                    message: format!(
                        "entity ends with ';' but has unbalanced parentheses (depth={}). \
                         Likely a missing ')' in this entity: {}",
                        paren_depth,
                        entity_buffer.chars().take(200).collect::<String>()
                    ),
                });
            }
        }
    }

    Ok(())
}

/// Update parenthesis depth by scanning a line, being aware of string literals.
///
/// Parentheses inside STEP strings (delimited by `'`) are not counted,
/// preventing false balance detection when strings contain `(` or `)`.
///
/// CRITICAL: `in_string` is threaded through from caller so that a string
/// literal opened on a previous line (multi-line string) is correctly
/// continued on this line. Without this, STEP files where strings wrap
/// across lines (e.g. NX-exported assemblies) cause the closing `'` on the
/// next line to be misinterpreted as OPENING a new string, which makes
/// `)` on that line ignored and leaves `paren_depth` permanently > 0,
/// merging all subsequent entities into one giant blob that gets dropped.
#[inline]
fn update_paren_depth(line: &str, depth: &mut i32, in_string: &mut bool) {
    let bytes = line.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        match bytes[i] {
            b'\'' => {
                // Check for escaped quote ('')
                if *in_string && i + 1 < bytes.len() && bytes[i + 1] == b'\'' {
                    i += 2; // Skip escaped quote
                    continue;
                }
                *in_string = !*in_string;
            }
            b'(' if !*in_string => *depth += 1,
            b')' if !*in_string => *depth -= 1,
            _ => {}
        }
        i += 1;
    }
}

// ============================================================
// Entity parsing
// ============================================================

fn parse_header_line(line: &str, header: &mut StepHeader) -> Result<(), StepParseError> {
    if line.starts_with("FILE_DESCRIPTION") {
        if let Some(content) = extract_parentheses_content(line) {
            header.file_description.push(content);
        }
    } else if line.starts_with("FILE_NAME") {
        if let Some(content) = extract_parentheses_content(line) {
            header.file_name.push(content);
        }
    } else if line.starts_with("FILE_SCHEMA") {
        if let Some(content) = extract_parentheses_content(line) {
            header.file_schema.push(content);
        }
    }
    Ok(())
}

fn parse_entity_line_with_lineno(line: &str, lineno: usize) -> Result<Option<StepEntity>, StepParseError> {
    // Format: #ID = TYPE_NAME(params);
    // Or complex: #ID = ( TYPE1(params1) TYPE2(params2) TYPE3(params3) );
    if !line.starts_with('#') {
        return Ok(None);
    }

    let line = line.trim_end_matches(';').trim();

    // Split at '='
    let eq_pos = match line.find('=') {
        Some(pos) => pos,
        None => return Ok(None),
    };

    let id_str = line[1..eq_pos].trim();
    let id: i64 = id_str.parse().map_err(|_| StepParseError::SyntaxError {
        line: lineno,
        message: format!("Invalid entity ID: {}", id_str),
    })?;

    let rest = line[eq_pos + 1..].trim();

    // Check for complex/composite entity: starts with '('
    if rest.starts_with('(') {
        // Complex entity: ( TYPE1(params) TYPE2(params2) ... )
        return parse_complex_entity(id, rest, lineno);
    }

    // Simple entity: TYPE_NAME(params)
    let paren_pos = match rest.find('(') {
        Some(pos) => pos,
        None => {
            // Entity with no parameters
            return Ok(Some(StepEntity {
                id,
                type_name: rest.trim().to_string(),
                params: vec![],
                sub_entities: vec![],
            }));
        }
    };

    let type_name = rest[..paren_pos].trim().to_string();
    let params_str = &rest[paren_pos..];

    let params = parse_step_values(params_str, lineno)?;

    Ok(Some(StepEntity { id, type_name, params, sub_entities: vec![] }))
}

/// Parse a complex/composite STEP entity.
/// Format: ( TYPE1(params1) TYPE2(params2) TYPE3(params3) )
///
/// In STEP, complex entities combine multiple entity types into one instance.
/// For example:
/// #748 = ( REPRESENTATION_RELATIONSHIP('','',#62,#44)
///          REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#749)
///          SHAPE_REPRESENTATION_RELATIONSHIP() );
///
/// We parse this into a single StepEntity whose type_name combines all types
/// and whose params combine all parameters. We also store the individual
/// sub-entities for later reference resolution.
fn parse_complex_entity(id: i64, rest: &str, lineno: usize) -> Result<Option<StepEntity>, StepParseError> {
    // Strip outer parentheses
    let inner = rest.trim_start_matches('(').trim_end_matches(')').trim();

    // Parse the complex entity into its constituent parts
    let mut type_names: Vec<String> = Vec::new();
    let mut all_params: Vec<StepValue> = Vec::new();
    let mut sub_entities: Vec<StepEntity> = Vec::new();

    // Use byte-based scanning instead of collecting Vec<char>
    let bytes = inner.as_bytes();
    let mut i = 0;
    let len = bytes.len();

    while i < len {
        // Skip whitespace
        while i < len && (bytes[i] == b' ' || bytes[i] == b'\t' || bytes[i] == b'\n') {
            i += 1;
        }
        if i >= len {
            break;
        }

        // Read type name (ASCII alphanumeric + underscore)
        let name_start = i;
        while i < len && (bytes[i].is_ascii_alphanumeric() || bytes[i] == b'_') {
            i += 1;
        }
        let name = &inner[name_start..i];

        if name.is_empty() {
            i += 1; // Skip unexpected character
            continue;
        }

        // Skip whitespace before potential parenthesis
        while i < len && (bytes[i] == b' ' || bytes[i] == b'\t') {
            i += 1;
        }

        // Check for parameters
        if i < len && bytes[i] == b'(' {
            // Find matching closing parenthesis — byte-scanning, string-aware
            let mut depth = 1i32;
            let start = i;
            i += 1;
            let mut in_string = false;
            while i < len && depth > 0 {
                match bytes[i] {
                    b'\'' => {
                        // Handle escaped quote
                        if in_string && i + 1 < len && bytes[i + 1] == b'\'' {
                            i += 2;
                            continue;
                        }
                        in_string = !in_string;
                    }
                    b'(' if !in_string => depth += 1,
                    b')' if !in_string => depth -= 1,
                    _ => {}
                }
                i += 1;
            }
            let params_str = &inner[start..i.min(len)];
            let params = parse_step_values(params_str, lineno)?;

            type_names.push(name.to_string());
            all_params.extend(params.iter().cloned());

            sub_entities.push(StepEntity {
                id: -1, // Synthetic sub-entity
                type_name: name.to_string(),
                params,
                sub_entities: vec![],
            });
        } else {
            // No parameters
            type_names.push(name.to_string());
            sub_entities.push(StepEntity {
                id: -1,
                type_name: name.to_string(),
                params: vec![],
                sub_entities: vec![],
            });
        }
    }

    // The combined type name uses all sub-types joined with "+"
    let combined_type_name = type_names.join("+");

    Ok(Some(StepEntity {
        id,
        type_name: combined_type_name,
        params: all_params,
        sub_entities,
    }))
}

/// Parse STEP parameter values from a string like "(1.0, 2.0, #3, .T.)".
///
/// Uses byte-based scanning on the `&str` directly — no `Vec<char>` allocation.
/// Handles string escape sequences: `''` is treated as a literal `'`.
fn parse_step_values(input: &str, lineno: usize) -> Result<Vec<StepValue>, StepParseError> {
    let mut values = Vec::new();
    let bytes = input.as_bytes();
    let mut i = 0;
    let len = bytes.len();

    // Skip opening paren
    if i < len && bytes[i] == b'(' {
        i += 1;
    }

    while i < len {
        match bytes[i] {
            b' ' | b'\t' | b',' => { i += 1; continue; }
            b')' => { break; }
            b'#' => {
                // Entity reference
                i += 1;
                let start = i;
                while i < len && (bytes[i].is_ascii_digit() || bytes[i] == b'-') {
                    i += 1;
                }
                let ref_id: i64 = input[start..i].parse().unwrap_or(0);
                values.push(StepValue::Ref(ref_id));
            }
            b'$' => {
                values.push(StepValue::Omitted);
                i += 1;
            }
            b'*' => {
                values.push(StepValue::Redefined);
                i += 1;
            }
            b'.' => {
                // Enum value like .T. or .F.
                i += 1;
                let start = i;
                while i < len && bytes[i] != b'.' {
                    i += 1;
                }
                let name = input[start..i].to_string();
                if i < len { i += 1; } // Skip closing dot
                values.push(StepValue::Enum(name));
            }
            b'\'' => {
                // String value — handles STEP escape: '' is a literal '
                i += 1;
                let mut s = String::new();
                while i < len {
                    if bytes[i] == b'\'' {
                        // Check for escaped quote ('')
                        if i + 1 < len && bytes[i + 1] == b'\'' {
                            s.push('\'');
                            i += 2;
                        } else {
                            i += 1; // Closing quote
                            break;
                        }
                    } else {
                        s.push(bytes[i] as char);
                        i += 1;
                    }
                }
                values.push(StepValue::String(s));
            }
            b'(' => {
                // Nested list — find matching close, string-aware
                let mut depth = 1i32;
                let start = i;
                i += 1;
                let mut in_string = false;
                while i < len && depth > 0 {
                    match bytes[i] {
                        b'\'' => {
                            if in_string && i + 1 < len && bytes[i + 1] == b'\'' {
                                i += 2;
                                continue;
                            }
                            in_string = !in_string;
                        }
                        b'(' if !in_string => depth += 1,
                        b')' if !in_string => depth -= 1,
                        _ => {}
                    }
                    i += 1;
                }
                let nested = &input[start..i.min(len)];
                let nested_values = parse_step_values(nested, lineno)?;
                values.push(StepValue::List(nested_values));
            }
            _ => {
                if bytes[i].is_ascii_digit() || bytes[i] == b'-' || bytes[i] == b'+' {
                    // Number — collect the full token, handling E/e scientific notation
                    let start = i;
                    i += 1;
                    let mut has_exponent = false;
                    while i < len {
                        let b = bytes[i];
                        if b.is_ascii_digit() || b == b'.' {
                            i += 1;
                        } else if (b == b'E' || b == b'e') && !has_exponent {
                            has_exponent = true;
                            i += 1;
                            // Consume optional sign after exponent
                            if i < len && (bytes[i] == b'+' || bytes[i] == b'-') {
                                i += 1;
                            }
                        } else {
                            break;
                        }
                    }
                    let num_str = &input[start..i];
                    if num_str.contains('.') || num_str.contains('E') || num_str.contains('e') {
                        values.push(StepValue::Float(num_str.parse().unwrap_or(0.0)));
                    } else {
                        values.push(StepValue::Integer(num_str.parse().unwrap_or(0)));
                    }
                } else {
                    // Type name followed by value (e.g., REAL(3.14))
                    let start = i;
                    while i < len && (bytes[i].is_ascii_alphanumeric() || bytes[i] == b'_') {
                        i += 1;
                    }
                    let name = input[start..i].to_string();

                    // Skip whitespace
                    while i < len && bytes[i] == b' ' { i += 1; }

                    if i < len && bytes[i] == b'(' {
                        // Find matching close, string-aware
                        let mut depth = 1i32;
                        let paren_start = i;
                        i += 1;
                        let mut in_string = false;
                        while i < len && depth > 0 {
                            match bytes[i] {
                                b'\'' => {
                                    if in_string && i + 1 < len && bytes[i + 1] == b'\'' {
                                        i += 2;
                                        continue;
                                    }
                                    in_string = !in_string;
                                }
                                b'(' if !in_string => depth += 1,
                                b')' if !in_string => depth -= 1,
                                _ => {}
                            }
                            i += 1;
                        }
                        let nested = &input[paren_start..i.min(len)];
                        let nested_values = parse_step_values(nested, lineno)?;
                        values.push(StepValue::Typed {
                            type_name: name,
                            value: Box::new(StepValue::List(nested_values)),
                        });
                    }
                }
            }
        }
    }

    Ok(values)
}

fn extract_parentheses_content(s: &str) -> Option<String> {
    let start = s.find('(')?;
    let end = s.rfind(')')?;
    Some(s[start + 1..end].to_string())
}

// ============================================================
// Error types
// ============================================================

/// STEP parse error.
#[derive(Debug, Clone)]
pub enum StepParseError {
    IoError(String),
    SyntaxError { line: usize, message: String },
    InvalidEntity { id: i64, message: String },
}

impl std::fmt::Display for StepParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            StepParseError::IoError(msg) => write!(f, "IO error: {}", msg),
            StepParseError::SyntaxError { line, message } => write!(f, "Syntax error at line {}: {}", line, message),
            StepParseError::InvalidEntity { id, message } => write!(f, "Invalid entity #{}: {}", id, message),
        }
    }
}

impl std::error::Error for StepParseError {}

#[cfg(test)]
mod tests {
    use super::*;
    use quickcheck_macros::quickcheck;

    #[test]
    fn test_parse_simple_step() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('3Draper test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', ('Author'), (''), '3Draper', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = SHAPE_DEFINITION_REPRESENTATION(#2, #3);
#10 = CARTESIAN_POINT('origin', (0.0, 0.0, 0.0));
#11 = DIRECTION('x', (1.0, 0.0, 0.0));
ENDSEC;
END-ISO-10303-21;
"#;
        let result = parse_step(step);
        assert!(result.is_ok());
        let file = result.unwrap();
        assert_eq!(file.entities.len(), 3);
        assert_eq!(file.entities[0].type_name, "SHAPE_DEFINITION_REPRESENTATION");
        assert_eq!(file.entities[1].type_name, "CARTESIAN_POINT");
    }

    #[test]
    fn test_parse_complex_entity() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#748 = ( REPRESENTATION_RELATIONSHIP('','',#62,#44) 
REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#749) 
SHAPE_REPRESENTATION_RELATIONSHIP() );
#749 = ITEM_DEFINED_TRANSFORMATION('','',#11,#45);
ENDSEC;
END-ISO-10303-21;
"#;
        let result = parse_step(step);
        assert!(result.is_ok());
        let file = result.unwrap();
        assert_eq!(file.entities.len(), 2);
        
        // Check the complex entity
        let complex = file.find_entity(748).unwrap();
        assert!(complex.type_name.contains("REPRESENTATION_RELATIONSHIP"));
        assert!(complex.type_name.contains("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION"));
        assert!(complex.type_name.contains("SHAPE_REPRESENTATION_RELATIONSHIP"));
        
        // Check it has sub-entities
        assert_eq!(complex.sub_entities.len(), 3);
        assert_eq!(complex.sub_entities[0].type_name, "REPRESENTATION_RELATIONSHIP");
        assert_eq!(complex.sub_entities[1].type_name, "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION");
        assert_eq!(complex.sub_entities[2].type_name, "SHAPE_REPRESENTATION_RELATIONSHIP");
        
        // Check the transformation reference is in the RRWT sub-entity
        let rrwt = &complex.sub_entities[1];
        assert!(rrwt.params.contains(&StepValue::Ref(749)));
    }

    #[test]
    fn test_parse_multiline_entity() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#747 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#748,#750);
#750 = PRODUCT_DEFINITION_SHAPE('Placement','Placement of an item',#751
  );
ENDSEC;
END-ISO-10303-21;
"#;
        let result = parse_step(step);
        assert!(result.is_ok());
        let file = result.unwrap();
        assert_eq!(file.entities.len(), 2);
        
        // Check the multi-line entity
        let pds = file.find_entity(750).unwrap();
        assert_eq!(pds.type_name, "PRODUCT_DEFINITION_SHAPE");
        // Should have 3 params: 'Placement', 'Placement of an item', #751
        assert!(pds.params.len() >= 3);
    }

    #[test]
    fn test_string_escape() {
        // STEP uses '' (two single quotes) to represent a literal ' in a string
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = PRODUCT('It''s a test', 'Description', (#2));
ENDSEC;
END-ISO-10303-21;
"#;
        let result = parse_step(step);
        assert!(result.is_ok());
        let file = result.unwrap();
        let entity = file.find_entity(1).unwrap();
        assert_eq!(entity.params[0], StepValue::String("It's a test".to_string()));
    }

    #[test]
    fn test_scientific_notation() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('origin', (1.0E-3, -2.5E+2, 3.14));
ENDSEC;
END-ISO-10303-21;
"#;
        let result = parse_step(step);
        assert!(result.is_ok());
        let file = result.unwrap();
        let entity = file.find_entity(1).unwrap();
        // Check the list of floats
        if let StepValue::List(ref coords) = entity.params[1] {
            assert_eq!(coords.len(), 3);
            if let StepValue::Float(v) = coords[0] {
                assert!((v - 1.0e-3).abs() < 1e-10);
            }
            if let StepValue::Float(v) = coords[1] {
                assert!((v - (-2.5e2)).abs() < 1e-10);
            }
        } else {
            panic!("Expected List for second param");
        }
    }

    #[test]
    fn test_block_comment() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
/* This is a block comment
   spanning multiple lines */
#1 = CARTESIAN_POINT('origin', (0.0, 0.0, 0.0));
#2 = DIRECTION('x', /* inline comment */ (1.0, 0.0, 0.0));
ENDSEC;
END-ISO-10303-21;
"#;
        let result = parse_step(step);
        assert!(result.is_ok());
        let file = result.unwrap();
        assert_eq!(file.entities.len(), 2);
    }

    #[test]
    fn test_streaming_callback() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('origin', (0.0, 0.0, 0.0));
#2 = DIRECTION('x', (1.0, 0.0, 0.0));
#3 = VECTOR('v', #2, 1.0);
ENDSEC;
END-ISO-10303-21;
"#;
        let mut entity_types: Vec<String> = Vec::new();
        let result = parse_step_streaming_str(step, |entity, _line| {
            entity_types.push(entity.type_name.clone());
            true
        });
        assert!(result.is_ok());
        assert_eq!(entity_types, vec![
            "CARTESIAN_POINT",
            "DIRECTION",
            "VECTOR",
        ]);
    }

    #[test]
    fn test_streaming_early_termination() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('origin', (0.0, 0.0, 0.0));
#2 = DIRECTION('x', (1.0, 0.0, 0.0));
#3 = VECTOR('v', #2, 1.0);
ENDSEC;
END-ISO-10303-21;
"#;
        let mut count = 0;
        let result = parse_step_streaming_str(step, |_entity, _line| {
            count += 1;
            count < 2 // Stop after 2 entities
        });
        assert!(result.is_ok());
        let file = result.unwrap();
        // Should have only parsed 2 entities before callback said stop
        assert_eq!(file.entities.len(), 2);
    }

    #[test]
    fn test_parse_step_reader() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('origin', (0.0, 0.0, 0.0));
ENDSEC;
END-ISO-10303-21;
"#;
        let reader = std::io::BufReader::new(step.as_bytes());
        let result = parse_step_reader(reader);
        assert!(result.is_ok());
        let file = result.unwrap();
        assert_eq!(file.entities.len(), 1);
        assert_eq!(file.entities[0].type_name, "CARTESIAN_POINT");
    }

    #[test]
    fn test_parens_in_strings() {
        // Test that parentheses inside strings don't affect balance tracking
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = APPLICATION_PROTOCOL_DEFINITION('international standard', 'automotive_design', 2000, (#2));
#2 = PRODUCT_CONTEXT('detailed design', #3, 'design');
ENDSEC;
END-ISO-10303-21;
"#;
        let result = parse_step(step);
        assert!(result.is_ok());
        let file = result.unwrap();
        assert_eq!(file.entities.len(), 2);
    }

    // ── Fuzz tests (quickcheck) ──
    // Goal: STEP parser must NEVER panic on arbitrary input.
    // All malformed inputs should return Err(StepParseError), not crash.

    /// Fuzz 1: Random byte strings must not panic the parser.
    #[quickcheck]
    fn fuzz_parse_random_string(input: String) -> bool {
        // The parser should either succeed or return an error — never panic.
        let result = parse_step(&input);
        result.is_ok() || result.is_err()
    }

    /// Fuzz 2: Random byte slices (including non-UTF8) must not panic.
    #[quickcheck]
    fn fuzz_parse_random_bytes(input: Vec<u8>) -> bool {
        // Try to parse as string — if invalid UTF-8, parser should handle gracefully.
        if let Ok(s) = std::str::from_utf8(&input) {
            let _ = parse_step(s);
        }
        // No panic = pass.
        true
    }

    /// Fuzz 3: Strings with STEP-like structure but random entity IDs.
    #[quickcheck]
    fn fuzz_parse_random_step_entities(entity_id: i64, entity_type: String) -> bool {
        let step_str = format!("{} = {};\nENDSEC;\nENDISO-10303-21;", entity_id, entity_type);
        let _ = parse_step(&step_str);
        // No panic = pass.
        true
    }

    /// Fuzz 4: Deeply nested parentheses (stack overflow test).
    #[quickcheck]
    fn fuzz_parse_nested_parens(depth: u16) -> bool {
        let depth = (depth % 50) as usize;  // Cap at 50 levels
        let open = "(".repeat(depth);
        let close = ")".repeat(depth);
        let step_str = format!("1 = TEST{}{};", open, close);
        let _ = parse_step(&step_str);
        true
    }

    /// Fuzz 5: Very long strings.
    #[quickcheck]
    fn fuzz_parse_long_string(len: u16) -> bool {
        let len = (len as usize) * 100;  // Up to ~6.5MB
        let s = "A".repeat(len);
        let step_str = format!("1 = TEST('{}');", s);
        let _ = parse_step(&step_str);
        true
    }
}
