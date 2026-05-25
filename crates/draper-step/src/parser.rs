//! STEP file parser.
//!
//! Handles:
//! - Multi-line entities (entities spanning multiple lines)
//! - Complex/composite entities (e.g., `( TYPE1() TYPE2() TYPE3() )`)
//! - Standard simple entities (`#ID = TYPE_NAME(params);`)

use crate::schema::*;
use std::collections::HashMap;

/// Parse a STEP file from a string.
pub fn parse_step(input: &str) -> Result<StepFile, StepParseError> {
    let mut file = StepFile::new();
    let mut in_data = false;
    let mut in_header = false;

    // Buffer for accumulating multi-line entities
    let mut entity_buffer = String::new();
    let mut collecting_entity = false;

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
            // Flush any buffered entity
            if collecting_entity && !entity_buffer.is_empty() {
                if let Some(entity) = parse_entity_line(&entity_buffer)? {
                    file.entities.push(entity);
                }
                entity_buffer.clear();
                collecting_entity = false;
            }
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
            if line.starts_with('#') {
                // Start of a new entity — flush any previous buffered entity
                if collecting_entity && !entity_buffer.is_empty() {
                    if let Some(entity) = parse_entity_line(&entity_buffer)? {
                        file.entities.push(entity);
                    }
                }
                entity_buffer = line.to_string();
                collecting_entity = true;

                // Check if this line is a complete entity (ends with ;)
                if line.ends_with(';') {
                    // Count parentheses to see if it's balanced
                    let open = line.chars().filter(|c| *c == '(').count();
                    let close = line.chars().filter(|c| *c == ')').count();
                    if open == close {
                        if let Some(entity) = parse_entity_line(&entity_buffer)? {
                            file.entities.push(entity);
                        }
                        entity_buffer.clear();
                        collecting_entity = false;
                    }
                    // If unbalanced, keep collecting
                }
            } else if collecting_entity {
                // Continuation of a multi-line entity
                entity_buffer.push(' ');
                entity_buffer.push_str(line);

                // Check if entity is complete
                if line.ends_with(';') {
                    let open = entity_buffer.chars().filter(|c| *c == '(').count();
                    let close = entity_buffer.chars().filter(|c| *c == ')').count();
                    // Also handle the case where the ; closes a complex entity
                    // like: #748 = ( TYPE1() TYPE2() TYPE3() );
                    if open == close || (open <= close && line.trim().ends_with(");")) {
                        if let Some(entity) = parse_entity_line(&entity_buffer)? {
                            file.entities.push(entity);
                        }
                        entity_buffer.clear();
                        collecting_entity = false;
                    }
                }
            }
        }
    }

    // Flush any remaining buffered entity
    if collecting_entity && !entity_buffer.is_empty() {
        if let Some(entity) = parse_entity_line(&entity_buffer)? {
            file.entities.push(entity);
        }
    }

    // Build entity index for fast lookup
    file.build_index();

    Ok(file)
}

/// Parse a STEP file from a file path (native only — not available on wasm).
#[cfg(not(target_arch = "wasm32"))]
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
        line: 0,
        message: format!("Invalid entity ID: {}", id_str),
    })?;

    let rest = line[eq_pos + 1..].trim();

    // Check for complex/composite entity: starts with '('
    if rest.starts_with('(') {
        // Complex entity: ( TYPE1(params) TYPE2(params2) ... )
        // We need to parse the combined entity and extract all parts
        return parse_complex_entity(id, rest);
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

    let params = parse_step_values(params_str)?;

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
fn parse_complex_entity(id: i64, rest: &str) -> Result<Option<StepEntity>, StepParseError> {
    // Strip outer parentheses
    let inner = rest.trim_start_matches('(').trim_end_matches(')').trim();

    // Parse the complex entity into its constituent parts
    let mut type_names: Vec<String> = Vec::new();
    let mut all_params: Vec<StepValue> = Vec::new();
    let mut sub_entities: Vec<StepEntity> = Vec::new();

    let chars: Vec<char> = inner.chars().collect();
    let mut i = 0;
    let len = chars.len();

    while i < len {
        // Skip whitespace
        while i < len && (chars[i] == ' ' || chars[i] == '\t' || chars[i] == '\n') {
            i += 1;
        }
        if i >= len {
            break;
        }

        // Read type name
        let mut name = String::new();
        while i < len && chars[i].is_ascii_alphanumeric() || (i < len && chars[i] == '_') {
            name.push(chars[i]);
            i += 1;
        }

        if name.is_empty() {
            i += 1; // Skip unexpected character
            continue;
        }

        // Skip whitespace before potential parenthesis
        while i < len && (chars[i] == ' ' || chars[i] == '\t') {
            i += 1;
        }

        // Check for parameters
        if i < len && chars[i] == '(' {
            // Find matching closing parenthesis
            let mut depth = 1;
            let start = i;
            i += 1;
            while i < len && depth > 0 {
                if chars[i] == '(' { depth += 1; }
                if chars[i] == ')' { depth -= 1; }
                i += 1;
            }
            let params_str = &inner[start..i.min(len)];
            let params = parse_step_values(params_str)?;

            type_names.push(name.clone());
            all_params.extend(params.iter().cloned());

            sub_entities.push(StepEntity {
                id: -1, // Synthetic sub-entity
                type_name: name,
                params,
                sub_entities: vec![],
            });
        } else {
            // No parameters
            type_names.push(name.clone());
            sub_entities.push(StepEntity {
                id: -1,
                type_name: name,
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
                    while i < chars.len() && (chars[i].is_ascii_alphanumeric() || chars[i] == '_') {
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
}
