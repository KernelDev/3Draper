//! Abstract Syntax Tree for STEP files.
//!
//! Represents the parsed structure of a STEP exchange file:
//! - HEADER section (file_description, file_name, file_schema)
//! - DATA section (entities with references)

use std::collections::HashMap;

/// A complete STEP document.
#[derive(Debug, Clone)]
pub struct StepDocument {
    pub header: StepHeader,
    pub entities: HashMap<u64, StepEntity>,
    /// Ordered list of entity IDs as they appear in the file.
    pub entity_order: Vec<u64>,
}

/// HEADER section of a STEP file.
#[derive(Debug, Clone)]
pub struct StepHeader {
    pub file_description: FileDescription,
    pub file_name: FileName,
    pub file_schema: FileSchema,
}

#[derive(Debug, Clone)]
pub struct FileDescription {
    pub description: Vec<String>,
    pub implementation_level: String,
}

#[derive(Debug, Clone)]
pub struct FileName {
    pub name: String,
    pub time_stamp: String,
    pub author: Vec<String>,
    pub organization: Vec<String>,
    pub preprocessor_version: String,
    pub originating_system: String,
    pub authorization: String,
}

#[derive(Debug, Clone)]
pub struct FileSchema {
    pub schemas: Vec<String>,
}

/// A single STEP entity instance (one line in DATA section).
#[derive(Debug, Clone)]
pub struct StepEntity {
    /// Instance ID (e.g., #42)
    pub id: u64,
    /// The keyword / type name (e.g., "CARTESIAN_POINT", "MANIFOLD_SOLID_BREP")
    pub type_name: String,
    /// Parameters of the entity.
    pub parameters: Vec<Parameter>,
}

/// A parameter value in a STEP entity.
#[derive(Debug, Clone, PartialEq)]
pub enum Parameter {
    /// An integer literal.
    Integer(i64),
    /// A floating-point literal.
    Real(f64),
    /// A string literal (without enclosing quotes).
    String(String),
    /// An enumeration value (without the dot delimiters).
    Enumeration(String),
    /// A reference to another entity (e.g., #42).
    Reference(u64),
    /// A typed parameter: TYPE_NAME(parameters).
    Typed {
        type_name: String,
        parameters: Vec<Parameter>,
    },
    /// A list of parameters: (p1, p2, ...).
    List(Vec<Parameter>),
    /// Omitted parameter (the $ sign).
    Omitted,
    /// Redefined / inherited parameter (the * sign).
    Redefined,
    /// Binary data.
    Binary(Vec<u8>),
}

impl StepDocument {
    /// Create an empty STEP document.
    pub fn new() -> Self {
        Self {
            header: StepHeader {
                file_description: FileDescription {
                    description: vec![],
                    implementation_level: String::new(),
                },
                file_name: FileName {
                    name: String::new(),
                    time_stamp: String::new(),
                    author: vec![],
                    organization: vec![],
                    preprocessor_version: String::new(),
                    originating_system: String::new(),
                    authorization: String::new(),
                },
                file_schema: FileSchema {
                    schemas: vec![],
                },
            },
            entities: HashMap::new(),
            entity_order: Vec::new(),
        }
    }

    /// Get an entity by its ID.
    pub fn get_entity(&self, id: u64) -> Option<&StepEntity> {
        self.entities.get(&id)
    }

    /// Resolve a parameter to the entity it references, if it is a Reference.
    pub fn resolve(&self, param: &Parameter) -> Option<&StepEntity> {
        if let Parameter::Reference(id) = param {
            self.get_entity(*id)
        } else {
            None
        }
    }

    /// Find all entities of a given type.
    pub fn find_by_type(&self, type_name: &str) -> Vec<&StepEntity> {
        self.entity_order
            .iter()
            .filter_map(|id| {
                let e = self.entities.get(id)?;
                if e.type_name == type_name {
                    Some(e)
                } else {
                    None
                }
            })
            .collect()
    }

    /// Build a structural tree of the document for display.
    /// Returns top-level entities (PRODUCT, SHAPE_DEFINITION_REPRESENTATION, etc.)
    /// grouped for hierarchical display.
    pub fn structure_tree(&self) -> StructureNode {
        let mut root = StructureNode {
            name: "STEP Document".to_string(),
            type_name: "DOCUMENT".to_string(),
            entity_id: None,
            children: Vec::new(),
        };

        // Find products (top-level objects)
        let products = self.find_by_type("PRODUCT");
        for product in &products {
            let mut product_node = StructureNode {
                name: self.extract_entity_name(product),
                type_name: "PRODUCT".to_string(),
                entity_id: Some(product.id),
                children: Vec::new(),
            };

            // Find related shape definitions
            self.build_subtree(product.id, &mut product_node);

            root.children.push(product_node);
        }

        // If no products found, just list all top-level entities
        if products.is_empty() {
            for id in &self.entity_order {
                if let Some(entity) = self.entities.get(id) {
                    let is_top_level = !self.is_referenced_by_others(entity.id);
                    if is_top_level {
                        root.children.push(StructureNode {
                            name: self.extract_entity_name(entity),
                            type_name: entity.type_name.clone(),
                            entity_id: Some(entity.id),
                            children: Vec::new(),
                        });
                    }
                }
            }
        }

        root
    }

    fn build_subtree(&self, parent_id: u64, parent_node: &mut StructureNode) {
        // Find entities that reference parent_id
        for id in &self.entity_order {
            if let Some(entity) = self.entities.get(id) {
                if entity.references(parent_id) {
                    let mut child_node = StructureNode {
                        name: self.extract_entity_name(entity),
                        type_name: entity.type_name.clone(),
                        entity_id: Some(entity.id),
                        children: Vec::new(),
                    };
                    self.build_subtree(entity.id, &mut child_node);
                    parent_node.children.push(child_node);
                }
            }
        }
    }

    fn extract_entity_name(&self, entity: &StepEntity) -> String {
        // Try to find a name parameter
        for param in &entity.parameters {
            if let Parameter::String(s) = param {
                if !s.is_empty() {
                    return s.clone();
                }
            }
            if let Parameter::Typed { type_name, parameters } = param {
                if type_name == "LABEL" || type_name == "NAME" {
                    if let Some(Parameter::String(s)) = parameters.first() {
                        return s.clone();
                    }
                }
            }
        }
        format!("#{} {}", entity.id, entity.type_name)
    }

    fn is_referenced_by_others(&self, id: u64) -> bool {
        self.entities.values().any(|e| e.references(id) && e.id != id)
    }
}

impl StepEntity {
    /// Check if this entity references another entity by ID.
    pub fn references(&self, id: u64) -> bool {
        self.parameters.iter().any(|p| p.references(id))
    }

    /// Get all entity IDs referenced by this entity.
    pub fn all_references(&self) -> Vec<u64> {
        let mut refs = Vec::new();
        for param in &self.parameters {
            param.collect_references(&mut refs);
        }
        refs
    }

    /// Get a parameter by index, if it exists.
    pub fn param(&self, index: usize) -> Option<&Parameter> {
        self.parameters.get(index)
    }

    /// Get a real (f64) parameter by index.
    pub fn real_param(&self, index: usize) -> Option<f64> {
        match self.param(index)? {
            Parameter::Real(v) => Some(*v),
            Parameter::Integer(v) => Some(*v as f64),
            _ => None,
        }
    }

    /// Get a reference parameter by index.
    pub fn ref_param(&self, index: usize) -> Option<u64> {
        match self.param(index)? {
            Parameter::Reference(id) => Some(*id),
            _ => None,
        }
    }

    /// Get a string parameter by index.
    pub fn string_param(&self, index: usize) -> Option<&str> {
        match self.param(index)? {
            Parameter::String(s) => Some(s),
            _ => None,
        }
    }

    /// Get a list parameter by index.
    pub fn list_param(&self, index: usize) -> Option<&Vec<Parameter>> {
        match self.param(index)? {
            Parameter::List(items) => Some(items),
            _ => None,
        }
    }
}

impl Parameter {
    /// Check if this parameter references a given entity ID.
    pub fn references(&self, id: u64) -> bool {
        match self {
            Parameter::Reference(ref_id) => *ref_id == id,
            Parameter::List(items) => items.iter().any(|p| p.references(id)),
            Parameter::Typed { parameters, .. } => parameters.iter().any(|p| p.references(id)),
            _ => false,
        }
    }

    /// Collect all entity references in this parameter.
    pub fn collect_references(&self, refs: &mut Vec<u64>) {
        match self {
            Parameter::Reference(id) => refs.push(*id),
            Parameter::List(items) => items.iter().for_each(|p| p.collect_references(refs)),
            Parameter::Typed { parameters, .. } => {
                parameters.iter().for_each(|p| p.collect_references(refs))
            }
            _ => {}
        }
    }
}

/// A node in the structure tree for UI display.
#[derive(Debug, Clone)]
pub struct StructureNode {
    pub name: String,
    pub type_name: String,
    pub entity_id: Option<u64>,
    pub children: Vec<StructureNode>,
}

impl Default for StepDocument {
    fn default() -> Self {
        Self::new()
    }
}
