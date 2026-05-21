//! Bridge between step-io's typed IR and our backward-compatible AST types.
//!
//! This module converts `StepModel` (step-io's arena-based IR) into
//! `StepDocument` (our flat HashMap-based AST) for use by the viewer's
//! structure tree and other legacy code paths.

use crate::ast::*;
use crate::error::{StepError, StepResult};
use crate::{parse_step_raw, StepModel, EntityGraph};
use std::collections::HashMap;

/// Parse a STEP file string and return both the raw StepModel and our StepDocument.
///
/// This is the primary entry point for STEP parsing in 3Draper.
/// It uses step-io under the hood for robust real-world file support.
pub fn parse_step(input: &str) -> StepResult<ParsedStep> {
    log::info!("Parsing STEP file ({} bytes)", input.len());

    // Step 1: Parse into EntityGraph (raw entities)
    let graph = parse_step_raw(input).map_err(|e| {
        StepError::Parse {
            line: 0,
            message: format!("step-io parse error: {:?}", e),
        }
    })?;
    log::info!("Parsed EntityGraph: {} entities, schema={:?}", graph.entities.len(), graph.schema);

    // Step 2: Convert to typed StepModel
    use step_io::reader::ReaderContext;
    let convert_result = ReaderContext::convert(&graph);
    let model = convert_result.model;

    if !convert_result.warnings.is_empty() {
        log::warn!("STEP conversion warnings ({}):", convert_result.warnings.len());
        for w in convert_result.warnings.iter().take(10) {
            log::warn!("  {:?}", w);
        }
        if convert_result.warnings.len() > 10 {
            log::warn!("  ... and {} more warnings", convert_result.warnings.len() - 10);
        }
    }

    log::info!(
        "StepModel: {} points, {} directions, {} curves, {} surfaces, {} vertices, {} edges, {} wires, {} faces, {} shells, {} solids",
        model.geometry.points.len(),
        model.geometry.directions.len(),
        model.geometry.curves.len(),
        model.geometry.surfaces.len(),
        model.topology.vertices.len(),
        model.topology.edges.len(),
        model.topology.wires.len(),
        model.topology.faces.len(),
        model.topology.shells.len(),
        model.topology.solids.len(),
    );

    // Step 3: Build backward-compatible StepDocument for the structure tree
    let doc = build_step_document(&graph, &model);

    Ok(ParsedStep {
        model,
        document: doc,
    })
}

/// The result of parsing a STEP file — contains both the typed model
/// and the backward-compatible AST document.
#[derive(Debug, Clone)]
pub struct ParsedStep {
    /// step-io's typed IR with arena-based geometry and topology.
    pub model: StepModel,
    /// Our backward-compatible AST document (for structure tree, etc.).
    pub document: StepDocument,
}

/// Build a StepDocument from the parsed EntityGraph and StepModel.
fn build_step_document(graph: &EntityGraph, model: &StepModel) -> StepDocument {
    let mut entities = HashMap::new();
    let mut entity_order = Vec::new();

    // Convert all raw entities from the graph
    for (&id, raw_entity) in &graph.entities {
        let step_entity = convert_raw_entity(id, raw_entity);
        entity_order.push(id);
        entities.insert(id, step_entity);
    }

    // Build header from model
    let header = if let Some(ref fh) = model.header {
        StepHeader {
            file_description: FileDescription {
                description: fh.description.as_slice().to_vec(),
                implementation_level: fh.implementation_level.as_str().to_string(),
            },
            file_name: FileName {
                name: fh.name.clone(),
                time_stamp: fh.time_stamp.clone(),
                author: fh.author.as_slice().to_vec(),
                organization: fh.organization.as_slice().to_vec(),
                preprocessor_version: fh.preprocessor_version.clone(),
                originating_system: fh.originating_system.clone(),
                authorization: fh.authorization.clone(),
            },
            file_schema: FileSchema {
                schemas: match &model.schema {
                    step_io::StepSchema::Known { raw: Some(raw_list), .. } => {
                        raw_list.as_slice().to_vec()
                    }
                    step_io::StepSchema::Known { raw: None, class } => {
                        vec![format!("{:?}", class)]
                    }
                    step_io::StepSchema::Unknown { raw } => {
                        raw.as_slice().to_vec()
                    }
                },
            },
        }
    } else {
        StepHeader::new_default()
    };

    StepDocument {
        header,
        entities,
        entity_order,
    }
}

/// Convert a raw entity from step-io's parser into our StepEntity type.
fn convert_raw_entity(id: u64, raw: &step_io::RawEntity) -> StepEntity {
    match raw {
        step_io::RawEntity::Simple { name, attributes, .. } => {
            let parameters = attributes.iter().map(convert_attribute).collect();
            StepEntity {
                id,
                type_name: name.clone(),
                parameters,
            }
        }
        step_io::RawEntity::Complex { parts, .. } => {
            let mut type_name = String::from("COMPLEX_ENTITY");
            let mut parameters = Vec::new();

            for (i, part) in parts.iter().enumerate() {
                if i == 0 {
                    type_name = part.name.clone();
                    let part_params: Vec<Parameter> = part.attributes.iter().map(convert_attribute).collect();
                    parameters = part_params;
                } else {
                    let part_params: Vec<Parameter> = part.attributes.iter().map(convert_attribute).collect();
                    parameters.push(Parameter::Typed {
                        type_name: part.name.clone(),
                        parameters: part_params,
                    });
                }
            }

            StepEntity {
                id,
                type_name,
                parameters,
            }
        }
    }
}

/// Convert a step-io Attribute into our Parameter type.
fn convert_attribute(attr: &step_io::Attribute) -> Parameter {
    match attr {
        step_io::Attribute::Integer(v) => Parameter::Integer(*v),
        step_io::Attribute::Real(v) => Parameter::Real(*v),
        step_io::Attribute::String(s) => Parameter::String(s.clone()),
        step_io::Attribute::Enum(s) => Parameter::Enumeration(s.clone()),
        step_io::Attribute::EntityRef(id) => Parameter::Reference(*id),
        step_io::Attribute::Unset => Parameter::Omitted,
        step_io::Attribute::Derived => Parameter::Redefined,
        step_io::Attribute::Binary(hex_str) => {
            // Decode hex string to bytes
            let data = decode_hex(hex_str);
            Parameter::Binary(data)
        }
        step_io::Attribute::List(items) => {
            Parameter::List(items.iter().map(convert_attribute).collect())
        }
        step_io::Attribute::Typed { type_name, value } => {
            Parameter::Typed {
                type_name: type_name.clone(),
                parameters: vec![convert_attribute(value)],
            }
        }
    }
}

/// Write a StepDocument back to STEP exchange format.
pub fn write_step(doc: &StepDocument) -> String {
    let mut output = String::new();

    output.push_str("ISO-10303-21;\n");
    output.push_str("HEADER;\n");

    output.push_str("FILE_DESCRIPTION(");
    output.push_str(&format_string_list(&doc.header.file_description.description));
    output.push_str(&format!(", '{}');\n", doc.header.file_description.implementation_level));

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

    output.push_str("FILE_SCHEMA(");
    output.push_str(&format_string_list(&doc.header.file_schema.schemas));
    output.push_str(");\n");

    output.push_str("ENDSEC;\n");

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

/// Decode a hex string to bytes (simple implementation, no external dep).
fn decode_hex(hex: &str) -> Vec<u8> {
    let hex_bytes: Vec<u8> = hex.bytes().filter_map(|b| {
        match b {
            b'0'..=b'9' => Some(b - b'0'),
            b'a'..=b'f' => Some(b - b'a' + 10),
            b'A'..=b'F' => Some(b - b'A' + 10),
            _ => None,
        }
    }).collect();
    let mut result = Vec::with_capacity(hex_bytes.len() / 2);
    for chunk in hex_bytes.chunks(2) {
        let high = chunk[0];
        let low = if chunk.len() > 1 { chunk[1] } else { 0 };
        result.push((high << 4) | low);
    }
    result
}
