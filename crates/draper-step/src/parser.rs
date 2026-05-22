//! STEP file parser.

use crate::schema::*;
use std::collections::HashMap;

/// Parse a STEP file from a string.
pub fn parse_step(input: &str) -> Result<StepFile, StepParseError> {
    let mut file = StepFile::new();
    let mut in_data = false;
    let mut in_header = false;

    for line in input.lines() {
        let line = line.trim();

        if line.is_empty() || line.starts_with("/*") {
            continue;
        }

        if line == "HEADER;" {
            in_header = true;
            continue;
        }

        if line == "ENDSEC;" && in_header {
            in_header = false;
            continue;
        }

        if line == "DATA;" {
            in_data = true;
            continue;
        }

        if line == "ENDSEC;" && in_data {
            in_data = false;
            continue;
        }

        if line == "END-ISO-10303-21;" {
            break;
        }

        if in_header {
            parse_header_line(line, &mut file.header)?;
        }

        if in_data {
            if let Some(entity) = parse_entity_line(line)? {
                file.entities.push(entity);
            }
        }
    }

    Ok(file)
}

/// Parse a STEP file from a file path.
pub fn parse_step_file(path: &str) -> Result<StepFile, StepParseError> {
    let content = std::fs::read_to_string(path)
        .map_err(|e| StepParseError::IoError(e.to_string()))?;
    parse_step(&content)
}

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

fn parse_entity_line(line: &str) -> Result<Option<StepEntity>, StepParseError> {
    // Format: #ID = TYPE_NAME(params);
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
        line: 0,
        message: format!("Invalid entity ID: {}", id_str),
    })?;

    let rest = line[eq_pos + 1..].trim();

    // Split type name and parameters
    let paren_pos = match rest.find('(') {
        Some(pos) => pos,
        None => {
            // Entity with no parameters
            return Ok(Some(StepEntity {
                id,
                type_name: rest.trim().to_string(),
                params: vec![],
            }));
        }
    };

    let type_name = rest[..paren_pos].trim().to_string();
    let params_str = &rest[paren_pos..];

    let params = parse_step_values(params_str)?;

    Ok(Some(StepEntity { id, type_name, params }))
}

/// Parse STEP parameter values from a string like "(1.0, 2.0, #3, .T.)".
fn parse_step_values(input: &str) -> Result<Vec<StepValue>, StepParseError> {
    let mut values = Vec::new();
    let chars: Vec<char> = input.chars().collect();
    let mut i = 0;

    // Skip opening paren
    if i < chars.len() && chars[i] == '(' {
        i += 1;
    }

    while i < chars.len() {
        match chars[i] {
            ' ' | '\t' | ',' => { i += 1; continue; }
            ')' => { break; }
            '#' => {
                // Entity reference
                i += 1;
                let mut num = String::new();
                while i < chars.len() && (chars[i].is_ascii_digit() || chars[i] == '-') {
                    num.push(chars[i]);
                    i += 1;
                }
                let ref_id: i64 = num.parse().unwrap_or(0);
                values.push(StepValue::Ref(ref_id));
            }
            '$' => {
                values.push(StepValue::Omitted);
                i += 1;
            }
            '*' => {
                values.push(StepValue::Redefined);
                i += 1;
            }
            '.' => {
                // Enum value like .T. or .F.
                i += 1;
                let mut name = String::new();
                while i < chars.len() && chars[i] != '.' {
                    name.push(chars[i]);
                    i += 1;
                }
                if i < chars.len() { i += 1; } // Skip closing dot
                values.push(StepValue::Enum(name));
            }
            '\'' => {
                // String value
                i += 1;
                let mut s = String::new();
                while i < chars.len() && chars[i] != '\'' {
                    s.push(chars[i]);
                    i += 1;
                }
                if i < chars.len() { i += 1; } // Skip closing quote
                values.push(StepValue::String(s));
            }
            '(' => {
                // Nested list
                let mut depth = 1;
                let start = i;
                i += 1;
                while i < chars.len() && depth > 0 {
                    if chars[i] == '(' { depth += 1; }
                    if chars[i] == ')' { depth -= 1; }
                    i += 1;
                }
                let nested = &input[start..i.min(chars.len())];
                let nested_values = parse_step_values(nested)?;
                values.push(StepValue::List(nested_values));
            }
            _ => {
                // Number or type name
                if chars[i].is_ascii_digit() || chars[i] == '-' || chars[i] == '+' {
                    let mut num = String::new();
                    while i < chars.len() && (chars[i].is_ascii_digit() || chars[i] == '.' || chars[i] == 'E' || chars[i] == 'e' || chars[i] == '-' || chars[i] == '+') {
                        num.push(chars[i]);
                        i += 1;
                    }
                    if num.contains('.') || num.contains('E') || num.contains('e') {
                        values.push(StepValue::Float(num.parse().unwrap_or(0.0)));
                    } else {
                        values.push(StepValue::Integer(num.parse().unwrap_or(0)));
                    }
                } else {
                    // Type name followed by value
                    let mut name = String::new();
                    while i < chars.len() && chars[i].is_ascii_alphanumeric() || (i < chars.len() && chars[i] == '_') {
                        name.push(chars[i]);
                        i += 1;
                    }
                    // Skip to the value
                    while i < chars.len() && chars[i] == ' ' { i += 1; }
                    if i < chars.len() && chars[i] == '(' {
                        let mut depth = 1;
                        let start = i;
                        i += 1;
                        while i < chars.len() && depth > 0 {
                            if chars[i] == '(' { depth += 1; }
                            if chars[i] == ')' { depth -= 1; }
                            i += 1;
                        }
                        let nested = &input[start..i.min(chars.len())];
                        let nested_values = parse_step_values(nested)?;
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

#[cfg(test)]
mod tests {
    use super::*;

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
}
