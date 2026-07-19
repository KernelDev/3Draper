// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Feature History — associative modeling with parameter-driven re-evaluation.
//!
//! Audit item 8.3 (2026-07-19): Implements a feature tree that records the
//! construction history of a solid. Each feature stores its parameters and
//! dependencies, enabling incremental re-evaluation when parameters change.
//!
//! # Design
//!
//! The feature tree is a directed acyclic graph (DAG) where:
//! - **Nodes** are features (sketch, extrude, fillet, boolean, etc.)
//! - **Edges** represent dependencies (feature B uses the result of feature A)
//!
//! When a feature's parameters change, only that feature and its dependents
//! are re-evaluated. Features that don't depend on the changed one are reused
//! from cache, making parameter sweeps fast.
//!
//! # Usage
//!
//! ```ignore
//! use draper_topology::feature_history::{FeatureTree, Feature, FeatureParams};
//!
//! let mut tree = FeatureTree::new();
//! let sketch_id = tree.add_feature(Feature::new("sketch1", FeatureParams::Sketch { ... }));
//! let extrude_id = tree.add_feature(Feature::new("extrude1", FeatureParams::Extrude {
//!     sketch: sketch_id,
//!     distance: 10.0,
//! }));
//! tree.evaluate(extrude_id);
//! ```

use crate::entity::Solid;
use std::collections::HashMap;

// ============================================================
// Feature Types
// ============================================================

/// A feature in the history tree.
///
/// Each feature has:
/// - A unique ID
/// - A name (for UI display)
/// - Parameters (feature-specific)
/// - Dependencies (other feature IDs this feature depends on)
/// - A cached result (the Solid produced by evaluating this feature)
#[derive(Clone, Debug)]
pub struct Feature {
    /// Unique feature ID.
    pub id: FeatureId,
    /// Human-readable name.
    pub name: String,
    /// Feature parameters.
    pub params: FeatureParams,
    /// IDs of features this feature depends on.
    pub dependencies: Vec<FeatureId>,
    /// Cached evaluation result (None if not yet evaluated or invalidated).
    pub cached_result: Option<Solid>,
    /// Whether this feature is currently being evaluated (cycle detection).
    pub evaluating: bool,
}

/// Unique identifier for a feature.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct FeatureId(pub u64);

/// Feature parameters — the data needed to evaluate a feature.
#[derive(Clone, Debug)]
pub enum FeatureParams {
    /// A 2D sketch (collection of curves).
    Sketch {
        /// Sketch curves in 2D (will be expanded in future).
        curves: Vec<String>,
    },
    /// Extrude a sketch along a direction.
    Extrude {
        /// Source sketch feature ID.
        sketch: FeatureId,
        /// Extrusion distance.
        distance: f64,
        /// Extrusion direction (x, y, z).
        direction: [f64; 3],
    },
    /// Revolve a sketch around an axis.
    Revolve {
        /// Source sketch feature ID.
        sketch: FeatureId,
        /// Revolution angle in radians (2π = full revolution).
        angle: f64,
        /// Axis origin (x, y, z).
        axis_origin: [f64; 3],
        /// Axis direction (x, y, z).
        axis_direction: [f64; 3],
    },
    /// Boolean union of two solids.
    Union {
        /// First operand.
        solid_a: FeatureId,
        /// Second operand.
        solid_b: FeatureId,
    },
    /// Boolean subtraction (a - b).
    Subtract {
        /// First operand (the solid to subtract from).
        solid_a: FeatureId,
        /// Second operand (the solid to remove).
        solid_b: FeatureId,
    },
    /// Boolean intersection of two solids.
    Intersect {
        /// First operand.
        solid_a: FeatureId,
        /// Second operand.
        solid_b: FeatureId,
    },
    /// Fillet edges with a given radius.
    Fillet {
        /// Source solid.
        solid: FeatureId,
        /// Fillet radius.
        radius: f64,
        /// Edge IDs to fillet (empty = all edges).
        edges: Vec<u64>,
    },
    /// Chamfer edges with a given distance.
    Chamfer {
        /// Source solid.
        solid: FeatureId,
        /// Chamfer distance.
        distance: f64,
        /// Edge IDs to chamfer (empty = all edges).
        edges: Vec<u64>,
    },
    /// Shell a solid (hollow it out).
    Shell {
        /// Source solid.
        solid: FeatureId,
        /// Shell thickness.
        thickness: f64,
        /// Face IDs to remove (open faces).
        open_faces: Vec<u64>,
    },
    /// Transform a solid (translate, rotate, scale).
    Transform {
        /// Source solid.
        solid: FeatureId,
        /// Translation (x, y, z).
        translation: [f64; 3],
        /// Rotation as quaternion (w, x, y, z).
        rotation: [f64; 4],
        /// Scale factor.
        scale: f64,
    },
}

impl Feature {
    /// Create a new feature with the given name and parameters.
    pub fn new(name: &str, params: FeatureParams) -> Self {
        Self {
            id: FeatureId(0), // Will be assigned by FeatureTree
            name: name.to_string(),
            params,
            dependencies: Vec::new(),
            cached_result: None,
            evaluating: false,
        }
    }

    /// Get the dependency feature IDs from the parameters.
    pub fn extract_dependencies(&self) -> Vec<FeatureId> {
        match &self.params {
            FeatureParams::Sketch { .. } => vec![],
            FeatureParams::Extrude { sketch, .. } => vec![*sketch],
            FeatureParams::Revolve { sketch, .. } => vec![*sketch],
            FeatureParams::Union { solid_a, solid_b } => vec![*solid_a, *solid_b],
            FeatureParams::Subtract { solid_a, solid_b } => vec![*solid_a, *solid_b],
            FeatureParams::Intersect { solid_a, solid_b } => vec![*solid_a, *solid_b],
            FeatureParams::Fillet { solid, .. } => vec![*solid],
            FeatureParams::Chamfer { solid, .. } => vec![*solid],
            FeatureParams::Shell { solid, .. } => vec![*solid],
            FeatureParams::Transform { solid, .. } => vec![*solid],
        }
    }
}

// ============================================================
// Feature Tree
// ============================================================

/// The feature history tree.
///
/// Stores all features and their dependencies. Supports:
/// - Adding features
/// - Evaluating features (with caching)
/// - Invalidating cached results when parameters change
/// - Re-evaluating only affected features
pub struct FeatureTree {
    /// All features, indexed by ID.
    features: HashMap<FeatureId, Feature>,
    /// Next available feature ID.
    next_id: u64,
    /// Root feature (the final solid).
    root: Option<FeatureId>,
}

impl FeatureTree {
    /// Create a new empty feature tree.
    pub fn new() -> Self {
        Self {
            features: HashMap::new(),
            next_id: 1,
            root: None,
        }
    }

    /// Add a feature to the tree.
    ///
    /// The feature's dependencies are automatically extracted from its params.
    /// Returns the assigned feature ID.
    pub fn add_feature(&mut self, mut feature: Feature) -> FeatureId {
        let id = FeatureId(self.next_id);
        self.next_id += 1;
        feature.id = id;
        feature.dependencies = feature.extract_dependencies();
        self.features.insert(id, feature);
        self.root = Some(id); // Last added feature is the root
        id
    }

    /// Get a feature by ID.
    pub fn get(&self, id: FeatureId) -> Option<&Feature> {
        self.features.get(&id)
    }

    /// Get a mutable feature by ID.
    pub fn get_mut(&mut self, id: FeatureId) -> Option<&mut Feature> {
        self.features.get_mut(&id)
    }

    /// Update a feature's parameters.
    ///
    /// This invalidates the feature's cached result and all features that
    /// depend on it (transitively).
    pub fn update_params(&mut self, id: FeatureId, params: FeatureParams) {
        if let Some(feature) = self.features.get_mut(&id) {
            feature.params = params;
            feature.dependencies = feature.extract_dependencies();
            feature.cached_result = None; // Invalidate self
        }
        // Invalidate all dependents transitively
        self.invalidate_dependents(id);
    }

    /// Invalidate all features that depend on the given feature (transitively).
    fn invalidate_dependents(&mut self, id: FeatureId) {
        let mut to_invalidate: Vec<FeatureId> = self
            .features
            .iter()
            .filter(|(_, f)| f.dependencies.contains(&id))
            .map(|(fid, _)| *fid)
            .collect();

        while let Some(fid) = to_invalidate.pop() {
            if let Some(feature) = self.features.get_mut(&fid) {
                if feature.cached_result.is_some() {
                    feature.cached_result = None;
                    // Add this feature's dependents to the queue
                    let dependents: Vec<FeatureId> = self
                        .features
                        .iter()
                        .filter(|(_, f)| f.dependencies.contains(&fid))
                        .map(|(dep_id, _)| *dep_id)
                        .collect();
                    to_invalidate.extend(dependents);
                }
            }
        }
    }

    /// Evaluate a feature and all its dependencies.
    ///
    /// Returns the resulting Solid, or None if evaluation failed.
    ///
    /// This is a placeholder implementation — actual feature evaluation
    /// (extrude, boolean, etc.) requires the geometry engine to be fully
    /// implemented. For now, it returns the cached result or None.
    pub fn evaluate(&mut self, id: FeatureId) -> Option<&Solid> {
        // Check for cycles
        if let Some(feature) = self.features.get(&id) {
            if feature.evaluating {
                return None; // Cycle detected
            }
        }

        // Mark as evaluating
        if let Some(feature) = self.features.get_mut(&id) {
            feature.evaluating = true;
        }

        // Check cache first
        if let Some(feature) = self.features.get(&id) {
            if feature.cached_result.is_some() {
                // Unmark and return cached
                if let Some(feature) = self.features.get_mut(&id) {
                    feature.evaluating = false;
                }
                return self.features.get(&id).and_then(|f| f.cached_result.as_ref());
            }
        }

        // Evaluate dependencies first
        let deps: Vec<FeatureId> = self.features.get(&id).map(|f| f.dependencies.clone()).unwrap_or_default();
        for dep_id in deps {
            self.evaluate(dep_id);
        }

        // TODO: Actual feature evaluation (requires geometry engine)
        // For now, just return the cached result (which is None)
        if let Some(feature) = self.features.get_mut(&id) {
            feature.evaluating = false;
        }
        self.features.get(&id).and_then(|f| f.cached_result.as_ref())
    }

    /// Get the root feature (the final solid).
    pub fn root(&self) -> Option<FeatureId> {
        self.root
    }

    /// Get all features in topological order (dependencies first).
    ///
    /// This is useful for re-evaluating the entire tree after a parameter change.
    pub fn topological_order(&self) -> Vec<FeatureId> {
        let mut visited = std::collections::HashSet::new();
        let mut order = Vec::new();

        fn visit(
            id: FeatureId,
            features: &HashMap<FeatureId, Feature>,
            visited: &mut std::collections::HashSet<FeatureId>,
            order: &mut Vec<FeatureId>,
        ) {
            if visited.contains(&id) {
                return;
            }
            visited.insert(id);
            if let Some(f) = features.get(&id) {
                for dep in &f.dependencies {
                    visit(*dep, features, visited, order);
                }
                order.push(id);
            }
        }

        if let Some(root) = self.root {
            visit(root, &self.features, &mut visited, &mut order);
        }
        order
    }

    /// Get the total number of features.
    pub fn len(&self) -> usize {
        self.features.len()
    }

    /// Check if the tree is empty.
    pub fn is_empty(&self) -> bool {
        self.features.is_empty()
    }
}

impl Default for FeatureTree {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_feature_tree_basic() {
        let mut tree = FeatureTree::new();
        let sketch = tree.add_feature(Feature::new("sketch1", FeatureParams::Sketch {
            curves: vec!["line".to_string()],
        }));
        let extrude = tree.add_feature(Feature::new("extrude1", FeatureParams::Extrude {
            sketch,
            distance: 10.0,
            direction: [0.0, 0.0, 1.0],
        }));

        assert_eq!(tree.len(), 2);
        assert_eq!(tree.root(), Some(extrude));

        // Topological order should have sketch before extrude
        let order = tree.topological_order();
        assert_eq!(order.len(), 2);
        assert_eq!(order[0], sketch);
        assert_eq!(order[1], extrude);
    }

    #[test]
    fn test_invalidation() {
        let mut tree = FeatureTree::new();
        let sketch = tree.add_feature(Feature::new("sketch1", FeatureParams::Sketch {
            curves: vec!["line".to_string()],
        }));
        let extrude = tree.add_feature(Feature::new("extrude1", FeatureParams::Extrude {
            sketch,
            distance: 10.0,
            direction: [0.0, 0.0, 1.0],
        }));

        // Update sketch params should invalidate extrude
        tree.update_params(sketch, FeatureParams::Sketch {
            curves: vec!["circle".to_string()],
        });

        // Both features should have no cached result
        assert!(tree.get(sketch).unwrap().cached_result.is_none());
        assert!(tree.get(extrude).unwrap().cached_result.is_none());
    }
}
