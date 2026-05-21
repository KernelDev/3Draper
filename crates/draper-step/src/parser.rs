//! STEP (ISO 10303-21) file parser.
//!
//! Hand-written recursive descent parser that handles real-world STEP files
//! including AP203, AP214, AP242 and other schema versions.
//!
//! Handles:
//! - Whitespace (spaces, tabs, newlines) anywhere between tokens
//! - Comments `/* ... */`
//! - Multi-line entities
//! - All parameter types (integer, real, string, enum, ref, typed, list, binary, omitted, redefined)
//! - Composite entity syntax: `TYPE1(...)TYPE2(...)`

use crate::ast::*;
use crate::error::{StepError, StepResult};

/// Parse a complete STEP file from a string.
pub fn parse_step(input: &str) -> StepResult<StepDocument> {
    let mut p = Parser::new(input);
    p.parse()
}

/// Hand-written STEP parser with position tracking.
struct Parser<'a> {
    input: &'a str,
    pos: usize,
    line: usize,
}

impl<'a> Parser<'a> {
    fn new(input: &'a str) -> Self {
        Self { input, pos: 0, line: 1 }
    }

    fn remaining(&self) -> &'a str {
        &self.input[self.pos..]
    }

    fn peek(&self) -> Option<char> {
        self.input[self.pos..].chars().next()
    }

    fn advance(&mut self, n: usize) {
        for ch in self.input[self.pos..self.pos + n].chars() {
            if ch == '\n' {
                self.line += 1;
            }
        }
        self.pos += n;
    }

    fn error(&self, msg: impl Into<String>) -> StepError {
        StepError::Parse {
            line: self.line,
            message: msg.into(),
        }
    }

    /// Skip whitespace and comments.
    fn skip_ws(&mut self) {
        loop {
            let rem = self.remaining();
            // Skip whitespace
            match rem.chars().next() {
                Some(c) if c.is_whitespace() => {
                    let len = c.len_utf8();
                    self.advance(len);
                    continue;
                }
                _ => {}
            }
            // Skip C-style comments  /* ... */
            if rem.starts_with("/*") {
                if let Some(end) = rem[2..].find("*/") {
                    self.advance(2 + end + 2);
                    continue;
                } else {
                    // Unterminated comment — skip to end
                    self.advance(rem.len());
                    return;
                }
            }
            break;
        }
    }

    /// Try to match a literal string at current position.
    fn try_match(&mut self, s: &str) -> bool {
        self.skip_ws();
        if self.remaining().starts_with(s) {
            self.advance(s.len());
            true
        } else {
            false
        }
    }

    /// Match a literal string or error.
    fn expect(&mut self, s: &str) -> StepResult<()> {
        self.skip_ws();
        if self.remaining().starts_with(s) {
            self.advance(s.len());
            Ok(())
        } else {
            let preview: String = self.remaining().chars().take(80).collect();
            Err(self.error(format!("Expected '{}', found: {}...", s, preview)))
        }
    }

    /// Match a single character or error.
    fn expect_char(&mut self, c: char) -> StepResult<()> {
        self.skip_ws();
        if self.peek() == Some(c) {
            self.advance(c.len_utf8());
            Ok(())
        } else {
            let found = self.peek().unwrap_or('\0');
            Err(self.error(format!("Expected '{}', found '{}' at line {}", c, found, self.line)))
        }
    }

    // ---- Top-level parsing ----

    fn parse(&mut self) -> StepResult<StepDocument> {
        self.expect("ISO-10303-21;")?;
        log::info!("STEP file header found");

        let header = self.parse_header_section()?;
        log::info!("HEADER parsed: schema={:?}", header.file_schema.schemas);

        let (entities, entity_order) = self.parse_data_section()?;
        log::info!("DATA parsed: {} entities", entities.len());

        // Try to parse END-ISO-10303-21; (non-fatal if missing)
        let _ = self.try_match("END-ISO-10303-21;");

        Ok(StepDocument {
            header,
            entities,
            entity_order,
        })
    }

    // ---- HEADER section ----

    fn parse_header_section(&mut self) -> StepResult<StepHeader> {
        self.expect("HEADER;")?;

        // Parse the three mandatory header entities.
        // We use a generic approach: read entity name, then parameter list, then ';'
        let file_description = self.parse_file_description()?;
        let file_name = self.parse_file_name()?;
        let file_schema = self.parse_file_schema()?;

        // Skip any additional header entities until ENDSEC
        loop {
            self.skip_ws();
            if self.remaining().starts_with("ENDSEC;") {
                break;
            }
            // Try to skip a header entity: NAME(...);
            if self.skip_entity_line().is_err() {
                break;
            }
        }

        self.expect("ENDSEC;")?;
        Ok(StepHeader { file_description, file_name, file_schema })
    }

    fn parse_file_description(&mut self) -> StepResult<FileDescription> {
        self.expect("FILE_DESCRIPTION")?;
        let params = self.parse_parameter_list()?;
        self.expect(";")?;

        let description = extract_string_list(&params, 0);
        let implementation_level = extract_string(&params, 1).unwrap_or_default();

        log::debug!("FILE_DESCRIPTION: {:?}, level={}", description, implementation_level);
        Ok(FileDescription { description, implementation_level })
    }

    fn parse_file_name(&mut self) -> StepResult<FileName> {
        self.expect("FILE_NAME")?;
        let params = self.parse_parameter_list()?;
        self.expect(";")?;

        Ok(FileName {
            name: extract_string(&params, 0).unwrap_or_default(),
            time_stamp: extract_string(&params, 1).unwrap_or_default(),
            author: extract_string_list(&params, 2),
            organization: extract_string_list(&params, 3),
            preprocessor_version: extract_string(&params, 4).unwrap_or_default(),
            originating_system: extract_string(&params, 5).unwrap_or_default(),
            authorization: extract_string(&params, 6).unwrap_or_default(),
        })
    }

    fn parse_file_schema(&mut self) -> StepResult<FileSchema> {
        self.expect("FILE_SCHEMA")?;
        let params = self.parse_parameter_list()?;
        self.expect(";")?;

        let schemas = extract_string_list(&params, 0);
        Ok(FileSchema { schemas })
    }

    /// Skip an entity-like line in the header (name + params + ;).
    fn skip_entity_line(&mut self) -> StepResult<()> {
        self.skip_ws();
        // Read identifier
        let name_start = self.pos;
        while let Some(c) = self.peek() {
            if c.is_alphanumeric() || c == '_' {
                self.advance(c.len_utf8());
            } else {
                break;
            }
        }
        if self.pos == name_start {
            return Err(self.error("Expected entity name"));
        }
        // Read parameter list (handles nested parens)
        if self.peek() == Some('(') {
            self.skip_balanced_parens()?;
        }
        self.expect(";")?;
        Ok(())
    }

    /// Skip a balanced parentheses group (including nested parens, strings, etc.)
    fn skip_balanced_parens(&mut self) -> StepResult<()> {
        self.expect_char('(')?;
        let mut depth = 1i32;
        let mut in_string = false;
        while depth > 0 {
            let ch = self.peek();
            match ch {
                None => return Err(self.error("Unterminated parentheses")),
                Some('\'') => {
                    self.advance(1);
                    if in_string {
                        // Check for escaped quote ''
                        if self.peek() == Some('\'') {
                            self.advance(1);
                            // Still in string
                        } else {
                            in_string = false;
                        }
                    } else {
                        in_string = true;
                    }
                }
                Some('(') if !in_string => {
                    self.advance(1);
                    depth += 1;
                }
                Some(')') if !in_string => {
                    self.advance(1);
                    depth -= 1;
                }
                Some('/') if !in_string && self.remaining().starts_with("/*") => {
                    self.skip_ws(); // handles comment
                }
                Some(_) => {
                    self.advance(1);
                }
            }
        }
        Ok(())
    }

    // ---- DATA section ----

    fn parse_data_section(&mut self) -> StepResult<(std::collections::HashMap<u64, StepEntity>, Vec<u64>)> {
        self.expect("DATA;")?;

        let mut entities = std::collections::HashMap::new();
        let mut entity_order = Vec::new();

        loop {
            self.skip_ws();
            if self.remaining().starts_with("ENDSEC;") {
                break;
            }
            if self.remaining().is_empty() {
                break;
            }

            match self.parse_entity_instance() {
                Ok(entity) => {
                    log::trace!("Parsed entity #{} = {}", entity.id, entity.type_name);
                    entity_order.push(entity.id);
                    entities.insert(entity.id, entity);
                }
                Err(e) => {
                    log::warn!("Skipping entity near line {}: {}", self.line, e);
                    // Try to recover: skip to next line
                    self.skip_to_next_line();
                }
            }
        }

        self.expect("ENDSEC;")?;
        Ok((entities, entity_order))
    }

    fn parse_entity_instance(&mut self) -> StepResult<StepEntity> {
        self.skip_ws();

        // Parse entity ID: #123
        self.expect_char('#')?;
        let id = self.parse_integer_u64()?;

        // Parse '='
        self.expect_char('=')?;

        // Parse entity type name
        let type_name = self.parse_identifier()?;

        // Parse parameter list
        let parameters = self.parse_parameter_list()?;

        // Handle composite entity syntax:
        //   #23 = ( TYPE1(...)TYPE2(...)REPRESENTATION_CONTEXT(...) );
        // or  #23 = TYPE1(...)TYPE2(...)REPRESENTATION_CONTEXT(...);
        // After the first param list, check for more type(...) sequences
        // We handle this by storing them as additional typed parameters

        // Check for ';' (normal entity) or more TYPE(...) (composite entity)
        self.skip_ws();

        // Check for composite entity: multiple TYPE(...) after first param list
        let mut all_params = parameters;
        loop {
            self.skip_ws();
            if self.peek() == Some(';') {
                break;
            }
            // Might be another TYPE(...) in a composite entity
            if let Some(c) = self.peek() {
                if c.is_alphabetic() || c == '_' {
                    let saved_pos = self.pos;
                    let saved_line = self.line;
                    let name = self.parse_identifier();
                    match name {
                        Ok(n) if self.peek() == Some('(') => {
                            // It's a composite entity continuation
                            let sub_params = self.parse_parameter_list()?;
                            all_params.push(Parameter::Typed {
                                type_name: n,
                                parameters: sub_params,
                            });
                            log::trace!("  Composite part: {}(...)", all_params.last().unwrap().type_name_if_typed().unwrap_or("?"));
                        }
                        _ => {
                            // Not a composite — restore position
                            self.pos = saved_pos;
                            self.line = saved_line;
                            break;
                        }
                    }
                } else {
                    break;
                }
            } else {
                break;
            }
        }

        self.expect(";")?;

        Ok(StepEntity {
            id,
            type_name,
            parameters: all_params,
        })
    }

    /// Parse an unsigned integer.
    fn parse_integer_u64(&mut self) -> StepResult<u64> {
        self.skip_ws();
        let start = self.pos;
        while let Some(c) = self.peek() {
            if c.is_ascii_digit() {
                self.advance(1);
            } else {
                break;
            }
        }
        if self.pos == start {
            return Err(self.error("Expected integer"));
        }
        let s = &self.input[start..self.pos];
        s.parse::<u64>().map_err(|e| self.error(format!("Invalid integer '{}': {}", s, e)))
    }

    /// Parse an identifier (alphanumeric + underscore).
    fn parse_identifier(&mut self) -> StepResult<String> {
        self.skip_ws();
        let start = self.pos;
        while let Some(c) = self.peek() {
            if c.is_alphanumeric() || c == '_' {
                self.advance(1);
            } else {
                break;
            }
        }
        if self.pos == start {
            return Err(self.error("Expected identifier"));
        }
        Ok(self.input[start..self.pos].to_string())
    }

    // ---- Parameter parsing ----

    fn parse_parameter_list(&mut self) -> StepResult<Vec<Parameter>> {
        self.expect_char('(')?;
        let params = self.parse_parameters_until(')')?;
        self.expect_char(')')?;
        Ok(params)
    }

    /// Parse parameters until we hit a closing paren at depth 0.
    fn parse_parameters_until(&mut self, end: char) -> StepResult<Vec<Parameter>> {
        let mut params = Vec::new();
        self.skip_ws();

        // Handle empty list
        if self.peek() == Some(end) {
            return Ok(params);
        }

        loop {
            self.skip_ws();
            let param = self.parse_parameter()?;
            params.push(param);

            self.skip_ws();
            match self.peek() {
                Some(',') => {
                    self.advance(1);
                    // Continue parsing next parameter
                }
                Some(c) if c == end => {
                    break;
                }
                _ => {
                    // In composite entities, we might see TYPE(...) directly
                    // after a parameter list without a comma. Let the caller handle it.
                    break;
                }
            }
        }

        Ok(params)
    }

    fn parse_parameter(&mut self) -> StepResult<Parameter> {
        self.skip_ws();
        match self.peek() {
            Some('$') => {
                self.advance(1);
                Ok(Parameter::Omitted)
            }
            Some('*') => {
                self.advance(1);
                Ok(Parameter::Redefined)
            }
            Some('.') => {
                self.advance(1);
                let name = self.parse_identifier()?;
                self.expect_char('.')?;
                Ok(Parameter::Enumeration(name))
            }
            Some('#') => {
                self.advance(1);
                let id = self.parse_integer_u64()?;
                Ok(Parameter::Reference(id))
            }
            Some('\'') => {
                let s = self.parse_step_string()?;
                Ok(Parameter::String(s))
            }
            Some('"') => {
                let data = self.parse_binary()?;
                Ok(Parameter::Binary(data))
            }
            Some('(') => {
                let params = self.parse_parameter_list()?;
                Ok(Parameter::List(params))
            }
            Some(c) if c.is_ascii_digit() || c == '-' || c == '+' => {
                self.parse_number_parameter()
            }
            Some(c) if c.is_alphabetic() || c == '_' => {
                // Typed parameter: TYPE_NAME(params)
                let type_name = self.parse_identifier()?;
                self.skip_ws();
                if self.peek() == Some('(') {
                    let params = self.parse_parameter_list()?;
                    Ok(Parameter::Typed { type_name, parameters: params })
                } else {
                    // Bare identifier — treat as string (shouldn't happen in valid STEP)
                    Ok(Parameter::String(type_name))
                }
            }
            Some(c) => {
                Err(self.error(format!("Unexpected character '{}' in parameter", c)))
            }
            None => Err(self.error("Unexpected end of input in parameter")),
        }
    }

    fn parse_step_string(&mut self) -> StepResult<String> {
        self.expect_char('\'')?;
        let mut result = String::new();
        loop {
            match self.peek() {
                None => return Err(self.error("Unterminated string")),
                Some('\'') => {
                    self.advance(1);
                    // Check for escaped quote ''
                    if self.peek() == Some('\'') {
                        self.advance(1);
                        result.push('\'');
                    } else {
                        break; // End of string
                    }
                }
                Some(c) => {
                    self.advance(c.len_utf8());
                    result.push(c);
                }
            }
        }
        Ok(result)
    }

    fn parse_binary(&mut self) -> StepResult<Vec<u8>> {
        self.expect_char('"')?;
        let start = self.pos;
        while let Some(c) = self.peek() {
            if c.is_ascii_hexdigit() {
                self.advance(1);
            } else {
                break;
            }
        }
        let hex_str = &self.input[start..self.pos];
        self.expect_char('"')?;

        let mut bytes = Vec::with_capacity(hex_str.len() / 2);
        for chunk in hex_str.as_bytes().chunks(2) {
            let high = from_hex_char(chunk[0]).unwrap_or(0);
            let low = if chunk.len() > 1 { from_hex_char(chunk[1]).unwrap_or(0) } else { 0 };
            bytes.push((high << 4) | low);
        }
        Ok(bytes)
    }

    fn parse_number_parameter(&mut self) -> StepResult<Parameter> {
        self.skip_ws();
        let start = self.pos;

        // Optional sign
        if self.peek() == Some('-') || self.peek() == Some('+') {
            self.advance(1);
        }

        // Digits before decimal point
        let has_int_part = self.consume_digits();
        let has_dot;
        let has_frac_part;
        let has_exp;

        // Decimal point
        if self.peek() == Some('.') {
            // Need to distinguish decimal point from enumeration start
            // If it's "123." followed by a digit, it's a real number
            // If it's "123" followed by ".ENUM.", the caller should have matched the enum first
            // But we can get here for "1.0", "1.", "1.0E-6" etc.
            let after_dot = self.input.get(self.pos + 1..).and_then(|s| s.chars().next());
            if after_dot.map_or(false, |c| c.is_ascii_digit()) {
                self.advance(1); // consume '.'
                has_dot = true;
                has_frac_part = self.consume_digits();
            } else if after_dot.map_or(false, |c| c == 'E' || c == 'e') {
                self.advance(1); // consume '.'
                has_dot = true;
                has_frac_part = false;
            } else if has_int_part && after_dot.is_none() {
                // End of input after digit and dot — treat as real
                self.advance(1);
                has_dot = true;
                has_frac_part = false;
            } else if has_int_part && !after_dot.map_or(false, |c| c.is_alphabetic()) {
                // "1." at end or followed by non-alpha — real number
                self.advance(1);
                has_dot = true;
                has_frac_part = false;
            } else {
                // This is likely an integer followed by an enumeration like ".T."
                // Don't consume the dot
                has_dot = false;
                has_frac_part = false;
            }
        } else {
            has_dot = false;
            has_frac_part = false;
        }

        // Exponent
        if self.peek() == Some('E') || self.peek() == Some('e') {
            self.advance(1);
            has_exp = true;
            // Optional sign
            if self.peek() == Some('-') || self.peek() == Some('+') {
                self.advance(1);
            }
            self.consume_digits();
        } else {
            has_exp = false;
        }

        let num_str = self.input[start..self.pos].to_string();

        if has_dot || has_frac_part || has_exp {
            // Real number
            let val: f64 = num_str.parse().map_err(|_| {
                self.error(format!("Invalid real number: '{}'", num_str))
            })?;
            Ok(Parameter::Real(val))
        } else {
            // Integer
            let val: i64 = num_str.parse().map_err(|_| {
                self.error(format!("Invalid integer: '{}'", num_str))
            })?;
            Ok(Parameter::Integer(val))
        }
    }

    fn consume_digits(&mut self) -> bool {
        let start = self.pos;
        while let Some(c) = self.peek() {
            if c.is_ascii_digit() {
                self.advance(1);
            } else {
                break;
            }
        }
        self.pos > start
    }

    fn skip_to_next_line(&mut self) {
        while let Some(c) = self.peek() {
            self.advance(1);
            if c == '\n' {
                break;
            }
        }
    }
}

fn from_hex_char(c: u8) -> Option<u8> {
    match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'a'..=b'f' => Some(c - b'a' + 10),
        b'A'..=b'F' => Some(c - b'A' + 10),
        _ => None,
    }
}

// ---- Helper functions for extracting data from parameters ----

fn extract_string(params: &[Parameter], index: usize) -> Option<String> {
    match params.get(index)? {
        Parameter::String(s) => Some(s.clone()),
        Parameter::Typed { parameters, .. } => {
            parameters.first().and_then(|p| {
                if let Parameter::String(s) = p {
                    Some(s.clone())
                } else {
                    None
                }
            })
        }
        _ => None,
    }
}

fn extract_string_list(params: &[Parameter], index: usize) -> Vec<String> {
    match params.get(index) {
        Some(Parameter::List(items)) => items
            .iter()
            .filter_map(|p| {
                if let Parameter::String(s) = p {
                    Some(s.clone())
                } else {
                    None
                }
            })
            .collect(),
        Some(Parameter::String(s)) => vec![s.clone()],
        _ => vec![],
    }
}

impl Parameter {
    fn type_name_if_typed(&self) -> Option<&str> {
        if let Parameter::Typed { type_name, .. } = self {
            Some(type_name)
        } else {
            None
        }
    }
}

// ---- Serialization ----

/// Serialize a StepDocument back to STEP exchange format.
pub fn write_step(doc: &StepDocument) -> String {
    let mut output = String::new();

    output.push_str("ISO-10303-21;\n");
    output.push_str("HEADER;\n");

    // FILE_DESCRIPTION
    output.push_str("FILE_DESCRIPTION(");
    output.push_str(&format_string_list(&doc.header.file_description.description));
    output.push_str(&format!(", '{}');\n", doc.header.file_description.implementation_level));

    // FILE_NAME
    output.push_str(&format!(
        "FILE_NAME('{}', '{}', {}, {}, '{}', '{}', '{}');\n",
        doc.header.file_name.name,
        doc.header.file_name.time_stamp,
        format_string_list(&doc.header.file_name.author),
        format_string_list(&doc.header.file_name.organization),
        doc.header.file_name.preprocessor_version,
        doc.header.file_name.originating_system,
        doc.header.file_name.authorization,
    ));

    // FILE_SCHEMA
    output.push_str("FILE_SCHEMA(");
    output.push_str(&format_string_list(&doc.header.file_schema.schemas));
    output.push_str(");\n");

    output.push_str("ENDSEC;\n");

    // DATA
    output.push_str("DATA;\n");
    for id in &doc.entity_order {
        if let Some(entity) = doc.entities.get(id) {
            output.push_str(&format_entity(entity));
            output.push('\n');
        }
    }
    output.push_str("ENDSEC;\n");
    output.push_str("END-ISO-10303-21;\n");

    output
}

fn format_string_list(items: &[String]) -> String {
    let formatted: Vec<String> = items.iter().map(|s| format!("'{}'", s.replace('\'', "''"))).collect();
    format!("({})", formatted.join(", "))
}

fn format_entity(entity: &StepEntity) -> String {
    let params: Vec<String> = entity.parameters.iter().map(format_parameter).collect();
    format!("#{}= {}({});", entity.id, entity.type_name, params.join(", "))
}

fn format_parameter(param: &Parameter) -> String {
    match param {
        Parameter::Integer(v) => format!("{}", v),
        Parameter::Real(v) => format!("{}", v),
        Parameter::String(s) => format!("'{}'", s.replace('\'', "''")),
        Parameter::Enumeration(s) => format!(".{}.", s),
        Parameter::Reference(id) => format!("#{}", id),
        Parameter::Typed { type_name, parameters } => {
            let params: Vec<String> = parameters.iter().map(format_parameter).collect();
            format!("{}({})", type_name, params.join(", "))
        }
        Parameter::List(items) => {
            let items: Vec<String> = items.iter().map(format_parameter).collect();
            format!("({})", items.join(", "))
        }
        Parameter::Omitted => "$".to_string(),
        Parameter::Redefined => "*".to_string(),
        Parameter::Binary(data) => {
            let hex: String = data.iter().map(|b| format!("{:02X}", b)).collect();
            format!("\"{}\"", hex)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_simple_step() {
        let input = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('A test STEP file'),'2;1');
FILE_NAME('test.stp','2024-01-01T00:00:00',('Author'),('Organization'),'preprocessor','3Draper','auth');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1=CARTESIAN_POINT('',(0.0,0.0,0.0));
#2=DIRECTION('',(0.0,0.0,1.0));
#3=AXIS2_PLACEMENT_3D('',#1,#2,$);
ENDSEC;
END-ISO-10303-21;
"#;
        let doc = parse_step(input).unwrap();
        assert_eq!(doc.entities.len(), 3);
        assert_eq!(doc.entities[&1].type_name, "CARTESIAN_POINT");
        assert_eq!(doc.entities[&2].type_name, "DIRECTION");
        assert_eq!(doc.entities[&3].type_name, "AXIS2_PLACEMENT_3D");
    }

    #[test]
    fn test_parse_spaced_format() {
        // Real-world STEP files often have spaces inside parens
        let input = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION( ( '' ), '1' );
FILE_NAME( 'test.stp', '2009-06-02T07:53:49', ( '' ), ( '' ), ' ', ' ', ' ' );
FILE_SCHEMA( ( 'CONFIG_CONTROL_DESIGN' ) );
ENDSEC;
DATA;
#1=CARTESIAN_POINT('',( 0.0, 0.0, 0.0 ));
#2=DIRECTION('',( 0.0, 0.0, 1.0 ));
ENDSEC;
END-ISO-10303-21;
"#;
        let doc = parse_step(input).unwrap();
        assert_eq!(doc.entities.len(), 2);
        assert_eq!(doc.entities[&1].type_name, "CARTESIAN_POINT");
    }

    #[test]
    fn test_parse_comments() {
        let input = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('Test'),'2;1');
FILE_NAME('test.stp','2024-01-01',('A'),('O'),'p','s','a');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
/* This is a comment */
#1=CARTESIAN_POINT('',(0.0,0.0,0.0)); /* inline comment */
#2=DIRECTION('',(0.0,0.0,1.0));
ENDSEC;
END-ISO-10303-21;
"#;
        let doc = parse_step(input).unwrap();
        assert_eq!(doc.entities.len(), 2);
    }

    #[test]
    fn test_parse_oriented_edge_with_star() {
        // ORIENTED_EDGE uses * for unnamed parameters
        let input = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('Test'),'2;1');
FILE_NAME('test.stp','2024-01-01',('A'),('O'),'p','s','a');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#140=ORIENTED_EDGE('',*,*,#217,.F.);
ENDSEC;
END-ISO-10303-21;
"#;
        let doc = parse_step(input).unwrap();
        assert_eq!(doc.entities.len(), 1);
        let e = &doc.entities[&140];
        assert_eq!(e.type_name, "ORIENTED_EDGE");
        assert_eq!(e.parameters.len(), 5);
        assert_eq!(e.parameters[1], Parameter::Redefined);
        assert_eq!(e.parameters[2], Parameter::Redefined);
        assert_eq!(e.parameters[3], Parameter::Reference(217));
    }

    #[test]
    fn test_roundtrip() {
        let input = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('Test'),'2;1');
FILE_NAME('test.stp','2024-01-01',('A'),('O'),'p','s','a');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1=CARTESIAN_POINT('origin',(0.0,0.0,0.0));
ENDSEC;
END-ISO-10303-21;
"#;
        let doc = parse_step(input).unwrap();
        let output = write_step(&doc);
        let doc2 = parse_step(&output).unwrap();
        assert_eq!(doc2.entities.len(), doc.entities.len());
    }
}
