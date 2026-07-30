// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Core engine — Phase 8.
//!
//! Selection system, undo/redo, parameter system, material system, layers.

use std::collections::HashMap;

// ============================================================
// 8.1. Selection System
// ============================================================

/// Type of selectable entity.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum EntityType {
    Face,
    Edge,
    Vertex,
    Body,
    Sketch,
    Plane,
}

impl EntityType {
    pub fn label(&self) -> &'static str {
        match self {
            EntityType::Face => "Face",
            EntityType::Edge => "Edge",
            EntityType::Vertex => "Vertex",
            EntityType::Body => "Body",
            EntityType::Sketch => "Sketch",
            EntityType::Plane => "Plane",
        }
    }
}

/// A selected entity.
#[derive(Clone, Debug)]
pub struct Selection {
    pub entity_type: EntityType,
    pub entity_id: u64,
    pub name: String,
}

/// Selection manager — tracks current selection.
#[derive(Clone, Debug, Default)]
pub struct SelectionManager {
    pub selected: Vec<Selection>,
    pub hover_id: Option<u64>,
}

impl SelectionManager {
    pub fn select(&mut self, sel: Selection) {
        self.selected.clear();
        self.selected.push(sel);
    }

    pub fn add_select(&mut self, sel: Selection) {
        if !self.selected.iter().any(|s| s.entity_id == sel.entity_id) {
            self.selected.push(sel);
        }
    }

    pub fn toggle_select(&mut self, sel: Selection) {
        if let Some(pos) = self.selected.iter().position(|s| s.entity_id == sel.entity_id) {
            self.selected.remove(pos);
        } else {
            self.selected.push(sel);
        }
    }

    pub fn clear(&mut self) {
        self.selected.clear();
    }

    pub fn count(&self) -> usize {
        self.selected.len()
    }

    pub fn is_empty(&self) -> bool {
        self.selected.is_empty()
    }

    pub fn primary(&self) -> Option<&Selection> {
        self.selected.last()
    }

    pub fn select_by_type(&mut self, etype: EntityType, entities: &[(u64, String)]) {
        self.selected.clear();
        for (id, name) in entities {
            self.selected.push(Selection {
                entity_type: etype,
                entity_id: *id,
                name: name.clone(),
            });
        }
    }

    pub fn invert(&mut self, all_entities: &[(EntityType, u64, String)]) {
        let current_ids: std::collections::HashSet<u64> =
            self.selected.iter().map(|s| s.entity_id).collect();
        self.selected.clear();
        for (etype, id, name) in all_entities {
            if !current_ids.contains(id) {
                self.selected.push(Selection {
                    entity_type: *etype,
                    entity_id: *id,
                    name: name.clone(),
                });
            }
        }
    }
}

// ============================================================
// 8.2. Undo/Redo System
// ============================================================

/// A command that can be undone.
pub trait Command: std::fmt::Debug {
    fn description(&self) -> &str;
    fn undo(&self) -> String;
    fn redo(&self) -> String;
}

/// Simple text-based command (for testing/stubs).
#[derive(Clone, Debug)]
pub struct TextCommand {
    desc: String,
}

impl TextCommand {
    pub fn new(desc: &str) -> Self {
        Self { desc: desc.to_string() }
    }
}

impl Command for TextCommand {
    fn description(&self) -> &str { &self.desc }
    fn undo(&self) -> String { format!("Undo: {}", self.desc) }
    fn redo(&self) -> String { format!("Redo: {}", self.desc) }
}

/// Undo/Redo manager with linear history.
#[derive(Default)]
pub struct UndoManager {
    undo_stack: Vec<Box<dyn Command>>,
    redo_stack: Vec<Box<dyn Command>>,
    max_history: usize,
}

impl UndoManager {
    pub fn new(max_history: usize) -> Self {
        Self {
            undo_stack: Vec::new(),
            redo_stack: Vec::new(),
            max_history,
        }
    }

    pub fn execute(&mut self, cmd: Box<dyn Command>) {
        // Truncate redo stack (branching point)
        self.redo_stack.clear();
        self.undo_stack.push(cmd);
        if self.undo_stack.len() > self.max_history {
            self.undo_stack.remove(0);
        }
    }

    pub fn undo(&mut self) -> Option<String> {
        if let Some(cmd) = self.undo_stack.pop() {
            let desc = cmd.undo();
            self.redo_stack.push(cmd);
            Some(desc)
        } else {
            None
        }
    }

    pub fn redo(&mut self) -> Option<String> {
        if let Some(cmd) = self.redo_stack.pop() {
            let desc = cmd.redo();
            self.undo_stack.push(cmd);
            Some(desc)
        } else {
            None
        }
    }

    pub fn can_undo(&self) -> bool { !self.undo_stack.is_empty() }
    pub fn can_redo(&self) -> bool { !self.redo_stack.is_empty() }

    pub fn history(&self) -> Vec<String> {
        self.undo_stack.iter().map(|c| c.description().to_string()).collect()
    }

    pub fn clear(&mut self) {
        self.undo_stack.clear();
        self.redo_stack.clear();
    }
}

// ============================================================
// 8.3. Parameter System
// ============================================================

/// A named parameter with optional formula.
#[derive(Clone, Debug)]
pub struct Parameter {
    pub name: String,
    pub value: f64,
    pub formula: Option<String>,
    pub unit: String,
    pub comment: String,
}

impl Parameter {
    pub fn new(name: &str, value: f64) -> Self {
        Self {
            name: name.to_string(),
            value,
            formula: None,
            unit: "mm".to_string(),
            comment: String::new(),
        }
    }

    pub fn with_formula(name: &str, formula: &str, unit: &str) -> Self {
        Self {
            name: name.to_string(),
            value: 0.0, // Will be evaluated
            formula: Some(formula.to_string()),
            unit: unit.to_string(),
            comment: String::new(),
        }
    }
}

/// Parameter table — stores all named parameters.
#[derive(Clone, Debug, Default)]
pub struct ParameterTable {
    pub params: HashMap<String, Parameter>,
}

impl ParameterTable {
    pub fn new() -> Self { Self::default() }

    pub fn set(&mut self, name: &str, value: f64) {
        if let Some(p) = self.params.get_mut(name) {
            p.value = value;
            p.formula = None;
        } else {
            self.params.insert(name.to_string(), Parameter::new(name, value));
        }
    }

    pub fn set_formula(&mut self, name: &str, formula: &str, unit: &str) {
        self.params.insert(name.to_string(), Parameter::with_formula(name, formula, unit));
    }

    pub fn get(&self, name: &str) -> Option<f64> {
        self.params.get(name).map(|p| p.value)
    }

    pub fn remove(&mut self, name: &str) -> Option<Parameter> {
        self.params.remove(name)
    }

    pub fn names(&self) -> Vec<String> {
        let mut names: Vec<String> = self.params.keys().cloned().collect();
        names.sort();
        names
    }

    /// Evaluate formulas (simple: supports "name * factor" and "name + value").
    pub fn evaluate(&mut self) {
        let snapshot = self.params.clone();
        for p in self.params.values_mut() {
            if let Some(ref formula) = p.formula {
                p.value = Self::eval_formula(formula, &snapshot);
            }
        }
    }

    fn eval_formula(formula: &str, params: &HashMap<String, Parameter>) -> f64 {
        let formula = formula.trim();
        // Simple evaluation: try to parse as expression
        // Support: "name", "name * 2", "name + 10", "name * name2"
        if let Some(val) = formula.parse::<f64>().ok() {
            return val;
        }

        // Try "a * b"
        if let Some(idx) = formula.find('*') {
            let left = formula[..idx].trim();
            let right = formula[idx + 1..].trim();
            let lv = Self::eval_term(left, params);
            let rv = Self::eval_term(right, params);
            return lv * rv;
        }

        // Try "a + b"
        if let Some(idx) = formula.find('+') {
            let left = formula[..idx].trim();
            let right = formula[idx + 1..].trim();
            let lv = Self::eval_term(left, params);
            let rv = Self::eval_term(right, params);
            return lv + rv;
        }

        // Try "a - b"
        if let Some(idx) = formula.find('-') {
            let left = formula[..idx].trim();
            let right = formula[idx + 1..].trim();
            let lv = Self::eval_term(left, params);
            let rv = Self::eval_term(right, params);
            return lv - rv;
        }

        // Try "a / b"
        if let Some(idx) = formula.find('/') {
            let left = formula[..idx].trim();
            let right = formula[idx + 1..].trim();
            let lv = Self::eval_term(left, params);
            let rv = Self::eval_term(right, params);
            if rv != 0.0 { return lv / rv; }
        }

        Self::eval_term(formula, params)
    }

    fn eval_term(term: &str, params: &HashMap<String, Parameter>) -> f64 {
        if let Some(val) = term.parse::<f64>().ok() {
            return val;
        }
        params.get(term).map(|p| p.value).unwrap_or(0.0)
    }
}

// ============================================================
// 8.4. Material System
// ============================================================

/// Material definition.
#[derive(Clone, Debug)]
pub struct Material {
    pub name: String,
    pub category: String,
    pub density: f64,       // kg/m³
    pub youngs_modulus: f64, // MPa
    pub poissons_ratio: f64,
    pub color: [u8; 3],
    pub thermal_conductivity: f64, // W/(m·K)
    pub specific_heat: f64,        // J/(kg·K)
}

impl Default for Material {
    fn default() -> Self {
        Self {
            name: "Default".to_string(),
            category: "Metals".to_string(),
            density: 7850.0,
            youngs_modulus: 200000.0,
            poissons_ratio: 0.29,
            color: [120, 120, 130],
            thermal_conductivity: 50.0,
            specific_heat: 490.0,
        }
    }
}

impl Material {
    pub fn steel() -> Self {
        Material {
            name: "Steel AISI 1045".to_string(),
            category: "Metals".to_string(),
            density: 7850.0,
            youngs_modulus: 200000.0,
            poissons_ratio: 0.29,
            color: [100, 100, 110],
            thermal_conductivity: 50.0,
            specific_heat: 490.0,
        }
    }

    pub fn aluminum() -> Self {
        Material {
            name: "Aluminum 6061-T6".to_string(),
            category: "Metals".to_string(),
            density: 2700.0,
            youngs_modulus: 68900.0,
            poissons_ratio: 0.33,
            color: [180, 180, 190],
            thermal_conductivity: 167.0,
            specific_heat: 896.0,
        }
    }

    pub fn abs_plastic() -> Self {
        Material {
            name: "ABS Plastic".to_string(),
            category: "Plastics".to_string(),
            density: 1050.0,
            youngs_modulus: 2000.0,
            poissons_ratio: 0.35,
            color: [220, 220, 220],
            thermal_conductivity: 0.18,
            specific_heat: 1500.0,
        }
    }
}

/// Material library.
#[derive(Clone, Debug, Default)]
pub struct MaterialLibrary {
    pub materials: Vec<Material>,
}

impl MaterialLibrary {
    pub fn new() -> Self {
        Self {
            materials: vec![
                Material::steel(),
                Material::aluminum(),
                Material::abs_plastic(),
            ],
        }
    }

    pub fn categories(&self) -> Vec<String> {
        let mut cats: Vec<String> = self.materials.iter().map(|m| m.category.clone()).collect();
        cats.sort();
        cats.dedup();
        cats
    }

    pub fn by_category(&self, cat: &str) -> Vec<&Material> {
        self.materials.iter().filter(|m| m.category == cat).collect()
    }
}

// ============================================================
// 8.5. Layer System
// ============================================================

/// A layer with visibility, color, and line weight.
#[derive(Clone, Debug)]
pub struct Layer {
    pub id: u32,
    pub name: String,
    pub visible: bool,
    pub locked: bool,
    pub color: [u8; 3],
    pub line_weight: f32,
}

impl Default for Layer {
    fn default() -> Self {
        Self {
            id: 0,
            name: "0".to_string(),
            visible: true,
            locked: false,
            color: [255, 255, 255],
            line_weight: 1.0,
        }
    }
}

/// Layer manager.
#[derive(Clone, Debug, Default)]
pub struct LayerManager {
    pub layers: Vec<Layer>,
    pub active_layer: u32,
}

impl LayerManager {
    pub fn new() -> Self {
        Self {
            layers: vec![
                Layer { id: 0, name: "0: Default".to_string(), visible: true, ..Default::default() },
                Layer { id: 1, name: "1: Construction".to_string(), visible: true, color: [100, 200, 100], line_weight: 0.5, ..Default::default() },
                Layer { id: 2, name: "2: Dimensions".to_string(), visible: true, color: [100, 150, 255], line_weight: 0.5, ..Default::default() },
                Layer { id: 3, name: "3: Annotations".to_string(), visible: true, color: [255, 200, 100], line_weight: 0.5, ..Default::default() },
            ],
            active_layer: 0,
        }
    }

    pub fn add(&mut self, name: &str) -> u32 {
        let id = self.layers.len() as u32;
        self.layers.push(Layer {
            id,
            name: format!("{}: {}", id, name),
            ..Default::default()
        });
        id
    }

    pub fn remove(&mut self, id: u32) {
        self.layers.retain(|l| l.id != id);
    }

    pub fn toggle_visible(&mut self, id: u32) {
        if let Some(l) = self.layers.iter_mut().find(|l| l.id == id) {
            l.visible = !l.visible;
        }
    }

    pub fn toggle_lock(&mut self, id: u32) {
        if let Some(l) = self.layers.iter_mut().find(|l| l.id == id) {
            l.locked = !l.locked;
        }
    }

    pub fn set_active(&mut self, id: u32) {
        self.active_layer = id;
    }

    pub fn is_visible(&self, id: u32) -> bool {
        self.layers.iter().find(|l| l.id == id).map(|l| l.visible).unwrap_or(true)
    }
}
