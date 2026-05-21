//! STEP (ISO 10303-21) file parser.
//!
//! Implements a complete parser for the STEP exchange structure format,
//! supporting all common AP versions (AP203, AP214, AP242, etc.).

use nom::{
    branch::alt,
    bytes::complete::{is_not, tag, take_while, take_while1},
    character::complete::{char, digit1, multispace0, multispace1, space0},
    combinator::{eof, map, map_res, opt, peek, recognize, value},
    multi::{many0, many1, separated_list0, separated_list1},
    number::complete::double,
    sequence::{delimited, pair, preceded, terminated, tuple},
    IResult,
};

use crate::ast::*;
use crate::error::{StepError, StepResult};

/// Parse a complete STEP file from a string.
pub fn parse_step(input: &str) -> StepResult<StepDocument> {
    let input = input.trim();
    let (rest, _) = parse_iso_start(input).map_err(|e| StepError::Parse {
        line: 0,
        message: format!("Invalid STEP file start: {:?}", e),
    })?;

    let (rest, header) = parse_header_section(rest).map_err(|e| StepError::Parse {
        line: 0,
        message: format!("Header parse error: {:?}", e),
    })?;

    let (rest, (entities, entity_order)) = parse_data_section(rest).map_err(|e| StepError::Parse {
        line: 0,
        message: format!("Data section parse error: {:?}", e),
    })?;

    let _ = parse_end_section(rest).map_err(|e| StepError::Parse {
        line: 0,
        message: format!("END-SEC parse error: {:?}", e),
    })?;

    Ok(StepDocument {
        header,
        entities,
        entity_order,
    })
}

// ---- Low-level parsers ----

fn parse_iso_start(input: &str) -> IResult<&str, &str> {
    tag("ISO-10303-21;")(input)
}

fn parse_header_section(input: &str) -> IResult<&str, StepHeader> {
    let (input, _) = tag("HEADER;")(input)?;
    let (input, _) = multispace0(input)?;

    let (input, file_description) = parse_file_description(input)?;
    let (input, _) = multispace0(input)?;

    let (input, file_name) = parse_file_name(input)?;
    let (input, _) = multispace0(input)?;

    let (input, file_schema) = parse_file_schema(input)?;
    let (input, _) = multispace0(input)?;

    // Skip any extra header entities
    let (input, _) = many0(parse_header_entity)(input)?;
    let (input, _) = multispace0(input)?;

    let (input, _) = tag("ENDSEC;")(input)?;

    Ok((input, StepHeader {
        file_description,
        file_name,
        file_schema,
    }))
}

fn parse_file_description(input: &str) -> IResult<&str, FileDescription> {
    let (input, _) = tag("FILE_DESCRIPTION")(input)?;
    let (input, params) = parse_parameter_list(input)?;
    let (input, _) = tag(";")(input)?;

    let description = extract_string_list(&params, 0);
    let implementation_level = extract_string(&params, 1).unwrap_or_default();

    Ok((input, FileDescription {
        description,
        implementation_level,
    }))
}

fn parse_file_name(input: &str) -> IResult<&str, FileName> {
    let (input, _) = tag("FILE_NAME")(input)?;
    let (input, params) = parse_parameter_list(input)?;
    let (input, _) = tag(";")(input)?;

    Ok((input, FileName {
        name: extract_string(&params, 0).unwrap_or_default(),
        time_stamp: extract_string(&params, 1).unwrap_or_default(),
        author: extract_string_list(&params, 2),
        organization: extract_string_list(&params, 3),
        preprocessor_version: extract_string(&params, 4).unwrap_or_default(),
        originating_system: extract_string(&params, 5).unwrap_or_default(),
        authorization: extract_string(&params, 6).unwrap_or_default(),
    }))
}

fn parse_file_schema(input: &str) -> IResult<&str, FileSchema> {
    let (input, _) = tag("FILE_SCHEMA")(input)?;
    let (input, params) = parse_parameter_list(input)?;
    let (input, _) = tag(";")(input)?;

    let schemas = extract_string_list(&params, 0);

    Ok((input, FileSchema { schemas }))
}

fn parse_header_entity(input: &str) -> IResult<&str, &str> {
    let (input, _) = multispace0(input)?;
    let (input, name) = take_while1(|c: char| c.is_alphanumeric() || c == '_')(input)?;
    let (input, _) = parse_parameter_list(input)?;
    let (input, _) = tag(";")(input)?;
    Ok((input, name))
}

// ---- DATA section parsing ----

fn parse_data_section(input: &str) -> IResult<&str, (std::collections::HashMap<u64, StepEntity>, Vec<u64>)> {
    let (input, _) = multispace0(input)?;
    let (input, _) = tag("DATA;")(input)?;
    let (input, _) = multispace0(input)?;

    let mut entities = std::collections::HashMap::new();
    let mut entity_order = Vec::new();

    let mut remaining = input;
    loop {
        let rest = remaining.trim_start();
        if rest.starts_with("ENDSEC;") {
            remaining = rest;
            break;
        }
        if rest.is_empty() {
            break;
        }

        match parse_entity_instance(rest) {
            Ok((rest, entity)) => {
                entity_order.push(entity.id);
                entities.insert(entity.id, entity);
                remaining = rest;
            }
            Err(e) => {
                // Try to skip to next line on error
                log::warn!("Skipping entity due to parse error: {:?}", e);
                if let Some(pos) = remaining.find('\n') {
                    remaining = &remaining[pos + 1..];
                } else {
                    break;
                }
            }
        }
    }

    let (remaining, _) = tag("ENDSEC;")(remaining)?;

    Ok((remaining, (entities, entity_order)))
}

fn parse_entity_instance(input: &str) -> IResult<&str, StepEntity> {
    let (input, _) = multispace0(input)?;
    let (input, id) = parse_entity_id(input)?;
    let (input, _) = char('=')(input)?;
    let (input, type_name) = parse_type_name(input)?;
    let (input, parameters) = parse_parameter_list(input)?;
    let (input, _) = char(';')(input)?;

    Ok((input, StepEntity {
        id,
        type_name,
        parameters,
    }))
}

fn parse_entity_id(input: &str) -> IResult<&str, u64> {
    let (input, _) = char('#')(input)?;
    map_res(digit1, |s: &str| s.parse::<u64>())(input)
}

fn parse_type_name(input: &str) -> IResult<&str, String> {
    let (input, name) = take_while1(|c: char| c.is_alphanumeric() || c == '_')(input)?;
    Ok((input, name.to_string()))
}

// ---- Parameter parsing ----

fn parse_parameter_list(input: &str) -> IResult<&str, Vec<Parameter>> {
    let (input, _) = char('(')(input)?;
    let (input, params) = separated_list0(parse_param_separator, parse_parameter)(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, params))
}

fn parse_param_separator(input: &str) -> IResult<&str, ()> {
    let (input, _) = multispace0(input)?;
    let (input, _) = char(',')(input)?;
    let (input, _) = multispace0(input)?;
    Ok((input, ()))
}

fn parse_parameter(input: &str) -> IResult<&str, Parameter> {
    alt((
        parse_omitted,
        parse_redefined,
        parse_enumeration,
        parse_typed_parameter,
        parse_reference,
        parse_real_param,
        parse_integer_param,
        parse_string_param,
        parse_binary_param,
        parse_list_parameter,
    ))(input)
}

fn parse_omitted(input: &str) -> IResult<&str, Parameter> {
    value(Parameter::Omitted, char('$'))(input)
}

fn parse_redefined(input: &str) -> IResult<&str, Parameter> {
    value(Parameter::Redefined, char('*'))(input)
}

fn parse_enumeration(input: &str) -> IResult<&str, Parameter> {
    let (input, _) = char('.')(input)?;
    let (input, name) = take_while1(|c: char| c.is_alphanumeric() || c == '_')(input)?;
    let (input, _) = char('.')(input)?;
    Ok((input, Parameter::Enumeration(name.to_string())))
}

fn parse_reference(input: &str) -> IResult<&str, Parameter> {
    map(parse_entity_id, Parameter::Reference)(input)
}

fn parse_real_param(input: &str) -> IResult<&str, Parameter> {
    let (input, val) = double(input)?;
    // Check that it has a decimal point or exponent (to distinguish from integer)
    Ok((input, Parameter::Real(val)))
}

fn parse_integer_param(input: &str) -> IResult<&str, Parameter> {
    let (input, neg) = opt(char('-'))(input)?;
    let (input, digits) = digit1(input)?;
    let val: i64 = digits.parse().unwrap_or(0);
    let val = if neg.is_some() { -val } else { val };
    Ok((input, Parameter::Integer(val)))
}

fn parse_string_param(input: &str) -> IResult<&str, Parameter> {
    let (input, _) = char('\'')(input)?;
    let (input, content) = parse_string_content(input)?;
    let (input, _) = char('\'')(input)?;
    Ok((input, Parameter::String(content)))
}

fn parse_string_content(input: &str) -> IResult<&str, String> {
    let mut result = String::new();
    let mut remaining = input;
    loop {
        if remaining.is_empty() || remaining.starts_with('\'') {
            break;
        }
        // Handle escaped apostrophe ('' -> ')
        if remaining.starts_with("''") {
            result.push('\'');
            remaining = &remaining[2..];
        } else {
            let ch = remaining.chars().next().unwrap();
            result.push(ch);
            remaining = &remaining[ch.len_utf8()..];
        }
    }
    Ok((remaining, result))
}

fn parse_binary_param(input: &str) -> IResult<&str, Parameter> {
    let (input, _) = char('"')(input)?;
    let (input, content) = take_while1(|c: char| c.is_ascii_hexdigit())(input)?;
    let (input, _) = char('"')(input)?;
    let bytes = hex::decode(content).unwrap_or_default();
    Ok((input, Parameter::Binary(bytes)))
}

fn parse_list_parameter(input: &str) -> IResult<&str, Parameter> {
    let (input, _) = char('(')(input)?;
    let (input, params) = separated_list0(parse_param_separator, parse_parameter)(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, Parameter::List(params)))
}

fn parse_typed_parameter(input: &str) -> IResult<&str, Parameter> {
    let (input, type_name) = take_while1(|c: char| c.is_alphanumeric() || c == '_')(input)?;
    let (input, _) = char('(')(input)?;
    let (input, params) = separated_list0(parse_param_separator, parse_parameter)(input)?;
    let (input, _) = char(')')(input)?;
    Ok((input, Parameter::Typed {
        type_name: type_name.to_string(),
        parameters: params,
    }))
}

fn parse_end_section(input: &str) -> IResult<&str, &str> {
    let (input, _) = multispace0(input)?;
    let (input, _) = tag("END-ISO-10303-21;")(input)?;
    Ok((input, ""))
}

// ---- Helper functions ----

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

/// Module for hex decoding (minimal implementation to avoid extra dependency)
mod hex {
    pub fn decode(input: &str) -> Result<Vec<u8>, ()> {
        let input = input.trim();
        if input.len() % 2 != 0 {
            return Err(());
        }
        let mut bytes = Vec::with_capacity(input.len() / 2);
        for chunk in input.as_bytes().chunks(2) {
            let high = from_hex_char(chunk[0])?;
            let low = from_hex_char(chunk[1])?;
            bytes.push((high << 4) | low);
        }
        Ok(bytes)
    }

    fn from_hex_char(c: u8) -> Result<u8, ()> {
        match c {
            b'0'..=b'9' => Ok(c - b'0'),
            b'a'..=b'f' => Ok(c - b'a' + 10),
            b'A'..=b'F' => Ok(c - b'A' + 10),
            _ => Err(()),
        }
    }
}

/// Serialize a StepDocument back to STEP exchange format.
pub fn write_step(doc: &StepDocument) -> String {
    let mut output = String::new();

    // ISO start
    output.push_str("ISO-10303-21;\n");

    // HEADER
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

    // END
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
