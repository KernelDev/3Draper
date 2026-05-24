//! STEP schema definitions — entity types and data structures.

/// A STEP entity with its ID, type name, and parameters.
#[derive(Clone, Debug)]
pub struct StepEntity {
    pub id: i64,
    pub type_name: String,
    pub params: Vec<StepValue>,
}

/// A STEP value.
#[derive(Clone, Debug)]
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
}

impl StepFile {
    pub fn new() -> Self {
        Self {
            header: StepHeader::default(),
            entities: Vec::new(),
        }
    }

    /// Find entity by ID.
    pub fn find_entity(&self, id: i64) -> Option<&StepEntity> {
        self.entities.iter().find(|e| e.id == id)
    }

    /// Find all entities of a given type.
    pub fn find_entities_by_type(&self, type_name: &str) -> Vec<&StepEntity> {
        self.entities.iter().filter(|e| e.type_name == type_name).collect()
    }
}
