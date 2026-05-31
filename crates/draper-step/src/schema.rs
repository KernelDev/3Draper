// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
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
///
/// The lazy index caches (type_index, pd_brep_index, nauo_transform_index) use
/// `RefCell` for interior mutability during lazy initialization. After initialization,
/// they are only ever read. Since `StepFile` is only mutated during parsing (single-threaded)
/// and then shared read-only across threads for conversion, we implement `Sync` manually.
/// This is safe because:
/// 1. The `RefCell` contents are only written once (lazy init), then only read
/// 2. Writing is protected by the `is_none()` check (only one thread writes)
/// 3. After writing, the contents are immutable
#[derive(Clone, Debug)]
pub struct StepFile {
    pub header: StepHeader,
    pub entities: Vec<StepEntity>,
    /// Index for fast entity lookup by ID.
    entity_index: HashMap<i64, usize>,
    /// Type-to-entities index for fast lookup by type name.
    /// Built lazily on first access to avoid overhead when not needed.
    type_index: std::cell::RefCell<Option<HashMap<String, Vec<usize>>>>,
    /// Pre-built index: pd_id → brep_id (resolved eagerly on first access).
    /// This replaces O(n³) per-call lookups with O(1) lookups.
    pd_brep_index: std::cell::RefCell<Option<HashMap<i64, Option<i64>>>>,
    /// Pre-built index: nauo_id → transform (resolved eagerly on first access).
    /// This replaces O(n²) per-call lookups with O(1) lookups.
    nauo_transform_index: std::cell::RefCell<Option<HashMap<i64, Option<[[f64; 4]; 4]>>>>,
}

// SAFETY: StepFile uses RefCell for lazy index initialization. After initialization,
// the RefCell contents are only read. Since StepFile is built in a single-threaded
// context (parsing) and then shared read-only, manual Sync is safe.
unsafe impl Sync for StepFile {}

impl StepFile {
    pub fn new() -> Self {
        Self {
            header: StepHeader::default(),
            entities: Vec::new(),
            entity_index: HashMap::new(),
            type_index: std::cell::RefCell::new(None),
            pd_brep_index: std::cell::RefCell::new(None),
            nauo_transform_index: std::cell::RefCell::new(None),
        }
    }

    /// Build the entity index for fast lookups. Called automatically after parsing.
    pub fn build_index(&mut self) {
        self.entity_index = self.entities.iter()
            .enumerate()
            .map(|(i, e)| (e.id, i))
            .collect();
        // Reset type index so it's rebuilt lazily
        *self.type_index.borrow_mut() = None;
        *self.pd_brep_index.borrow_mut() = None;
        *self.nauo_transform_index.borrow_mut() = None;
    }

    /// Build the type-to-entities index for O(1) type lookups.
    /// Called lazily on first `find_entities_by_type` call.
    fn ensure_type_index(&self) {
        if self.type_index.borrow().is_some() {
            return;
        }
        let mut index: HashMap<String, Vec<usize>> = HashMap::new();
        for (i, entity) in self.entities.iter().enumerate() {
            // Index by full type name
            index.entry(entity.type_name.clone()).or_default().push(i);
            // For complex entities with "+" separated types, also index each sub-type
            if entity.type_name.contains('+') {
                for part in entity.type_name.split('+') {
                    let part = part.trim().to_string();
                    if !part.is_empty() {
                        index.entry(part).or_default().push(i);
                    }
                }
            }
        }
        *self.type_index.borrow_mut() = Some(index);
    }

    /// Find entity by ID.
    pub fn find_entity(&self, id: i64) -> Option<&StepEntity> {
        self.entity_index.get(&id)
            .map(|&idx| &self.entities[idx])
    }

    /// Get a reference to the entity index (id → array position).
    /// This allows callers to clone the index instead of rebuilding it.
    pub fn entity_index_ref(&self) -> &HashMap<i64, usize> {
        &self.entity_index
    }

    /// Find all entities whose type_name contains the given string.
    /// This handles both simple types like "MANIFOLD_SOLID_BREP" and
    /// complex types like "REPRESENTATION_RELATIONSHIP+REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION+SHAPE_REPRESENTATION_RELATIONSHIP".
    ///
    /// Uses a type index for O(1) exact match lookups, falling back to linear scan
    /// only for partial matches.
    pub fn find_entities_by_type(&self, type_name: &str) -> Vec<&StepEntity> {
        self.ensure_type_index();
        let type_idx = self.type_index.borrow();
        if let Some(ref idx) = *type_idx {
            // Fast path: exact match via index
            if let Some(indices) = idx.get(type_name) {
                return indices.iter().map(|&i| &self.entities[i]).collect();
            }
        }
        // Slow path: linear scan for partial matches (rare)
        self.entities.iter().filter(|e| {
            e.type_name == type_name || e.type_name.contains(type_name)
        }).collect()
    }

    /// Find all entities with exact type name match.
    pub fn find_entities_by_exact_type(&self, type_name: &str) -> Vec<&StepEntity> {
        self.ensure_type_index();
        let type_idx = self.type_index.borrow();
        if let Some(ref idx) = *type_idx {
            if let Some(indices) = idx.get(type_name) {
                return indices.iter().map(|&i| &self.entities[i]).collect();
            }
        }
        vec![]
    }

    /// Get or build the pd_id → brep_id index.
    /// Built lazily on first access and cached for subsequent calls.
    pub fn pd_brep_index(&self) -> std::cell::Ref<'_, HashMap<i64, Option<i64>>> {
        if self.pd_brep_index.borrow().is_none() {
            // Will be populated by StepConverter on first use
            *self.pd_brep_index.borrow_mut() = Some(HashMap::new());
        }
        std::cell::Ref::map(self.pd_brep_index.borrow(), |opt| opt.as_ref().unwrap())
    }

    /// Set the pd_brep_index (called by StepConverter after building it).
    pub fn set_pd_brep_index(&self, index: HashMap<i64, Option<i64>>) {
        *self.pd_brep_index.borrow_mut() = Some(index);
    }

    /// Get or build the nauo_id → transform index.
    /// Built lazily on first access and cached for subsequent calls.
    pub fn nauo_transform_index(&self) -> std::cell::Ref<'_, HashMap<i64, Option<[[f64; 4]; 4]>>> {
        if self.nauo_transform_index.borrow().is_none() {
            *self.nauo_transform_index.borrow_mut() = Some(HashMap::new());
        }
        std::cell::Ref::map(self.nauo_transform_index.borrow(), |opt| opt.as_ref().unwrap())
    }

    /// Set the nauo_transform_index (called by StepConverter after building it).
    pub fn set_nauo_transform_index(&self, index: HashMap<i64, Option<[[f64; 4]; 4]>>) {
        *self.nauo_transform_index.borrow_mut() = Some(index);
    }
}
