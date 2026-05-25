//! STEP schema definitions — entity types and data structures.

use std::collections::HashMap;

/// A STEP entity with its ID, type name, and parameters.
#[derive(Clone, Debug)]
pub struct StepEntity {
    pub id: i64,
    pub type_name: String,
    pub params: Vec<StepValue>,
    /// For complex/composite entities, this contains the individual sub-entities.
    /// For example, `( REPRESENTATION_RELATIONSHIP() REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION() SHAPE_REPRESENTATION_RELATIONSHIP() )`
    /// will have 3 sub-entities.
    /// For simple entities, this is empty.
    pub sub_entities: Vec<StepEntity>,
}

impl StepEntity {
    /// Check if this entity is a complex entity (has sub-entities).
    pub fn is_complex(&self) -> bool {
        !self.sub_entities.is_empty()
    }

    /// Find a sub-entity by type name prefix.
    /// For complex entities, searches sub-entities for one whose type_name contains the given prefix.
    pub fn find_sub_entity(&self, type_name: &str) -> Option<&StepEntity> {
        self.sub_entities.iter().find(|e| e.type_name == type_name || e.type_name.contains(type_name))
    }
}

/// A STEP value.
#[derive(Clone, Debug, PartialEq)]
pub enum StepValue {
    Integer(i64),
    Float(f64),
    String(String),
    Enum(String),
    Ref(i64),         // Reference to another entity by ID
    List(Vec<StepValue>),
    Omitted,          // $ — omitted parameter
    Redefined,        // * — redefined parameter
    Typed { type_name: String, value: Box<StepValue> },
}

/// STEP file header.
#[derive(Clone, Debug, Default)]
pub struct StepHeader {
    pub file_description: Vec<String>,
    pub file_name: Vec<String>,
    pub file_schema: Vec<String>,
}

/// Parsed STEP file.
#[derive(Clone, Debug)]
pub struct StepFile {
    pub header: StepHeader,
    pub entities: Vec<StepEntity>,
    /// Index for fast entity lookup by ID.
    entity_index: HashMap<i64, usize>,
}

impl StepFile {
    pub fn new() -> Self {
        Self {
            header: StepHeader::default(),
            entities: Vec::new(),
            entity_index: HashMap::new(),
        }
    }

    /// Build the entity index for fast lookups. Called automatically after parsing.
    pub fn build_index(&mut self) {
        self.entity_index = self.entities.iter()
            .enumerate()
            .map(|(i, e)| (e.id, i))
            .collect();
    }

    /// Find entity by ID.
    pub fn find_entity(&self, id: i64) -> Option<&StepEntity> {
        self.entity_index.get(&id)
            .map(|&idx| &self.entities[idx])
    }

    /// Find all entities whose type_name contains the given string.
    /// This handles both simple types like "MANIFOLD_SOLID_BREP" and
    /// complex types like "REPRESENTATION_RELATIONSHIP+REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION+SHAPE_REPRESENTATION_RELATIONSHIP".
    pub fn find_entities_by_type(&self, type_name: &str) -> Vec<&StepEntity> {
        self.entities.iter().filter(|e| {
            e.type_name == type_name || e.type_name.contains(type_name)
        }).collect()
    }

    /// Find all entities with exact type name match.
    pub fn find_entities_by_exact_type(&self, type_name: &str) -> Vec<&StepEntity> {
        self.entities.iter().filter(|e| e.type_name == type_name).collect()
    }
}
