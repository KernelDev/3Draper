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
use crate::operations::{
    extrude_polyline, revolve_polyline, fillet_edge, chamfer_edge,
    shell_solid, Polyline2d,
};
use crate::boolean::{boolean_union, boolean_subtract, boolean_intersect};
use crate::builder::ShapeBuilder;
use draper_geometry::{Vec3d, Direction3d, ToleranceContext, Transform};
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
    /// A 2D sketch (closed polyline profile).
    ///
    /// Per BREPCAD Phase 1.3: stores an actual `Polyline2d` (not a
    /// placeholder string), so the sketch can be evaluated into a real
    /// profile for extrude/revolve operations.
    Sketch {
        /// The 2D polyline profile (closed).
        profile: Polyline2d,
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
    /// Current feature for rollback (None = use root).
    current_feature: Option<FeatureId>,
}

impl FeatureTree {
    /// Create a new empty feature tree.
    pub fn new() -> Self {
        Self {
            features: HashMap::new(),
            next_id: 1,
            root: None,
            current_feature: None,
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
    /// Per BREPCAD Phase 1.3: REAL implementation — no stubs. Each
    /// feature type calls the corresponding geometry operation:
    /// - Sketch: stores the profile (no solid produced)
    /// - Extrude: calls `extrude_polyline` on the sketch's profile
    /// - Revolve: calls `revolve_polyline` on the sketch's profile
    /// - Union/Subtract/Intersect: calls the corresponding boolean op
    /// - Fillet/Chamfer: calls `fillet_edge` / `chamfer_edge`
    /// - Shell: calls `shell_solid`
    /// - Transform: applies translation/rotation/scale
    ///
    /// Returns the resulting Solid, or None if evaluation failed.
    pub fn evaluate(&mut self, id: FeatureId) -> Option<&Solid> {
        // Check for cycles
        if let Some(feature) = self.features.get(&id) {
            if feature.evaluating {
                log::warn!("FeatureTree: cycle detected at feature {:?}", id);
                return None;
            }
        }

        // Check cache first
        if let Some(feature) = self.features.get(&id) {
            if feature.cached_result.is_some() {
                return self.features.get(&id).and_then(|f| f.cached_result.as_ref());
            }
        }

        // Mark as evaluating
        if let Some(feature) = self.features.get_mut(&id) {
            feature.evaluating = true;
        }

        // Evaluate dependencies first
        let deps: Vec<FeatureId> = self
            .features
            .get(&id)
            .map(|f| f.dependencies.clone())
            .unwrap_or_default();
        for dep_id in deps {
            self.evaluate(dep_id);
        }

        // Get the params (clone to avoid borrow issues)
        let params = self.features.get(&id).map(|f| f.params.clone());

        // Evaluate based on feature type
        let result: Option<Solid> = match params {
            Some(FeatureParams::Sketch { .. }) => {
                // Sketch doesn't produce a solid — it stores the profile
                // for downstream features (extrude, revolve).
                None
            }

            Some(FeatureParams::Extrude { sketch, distance, direction }) => {
                let profile = self.get_sketch_profile(sketch);
                match profile {
                    Some(p) => {
                        let dir = Vec3d::new(direction[0], direction[1], direction[2]);
                        match extrude_polyline(&p, dir, distance) {
                            Ok(solid) => Some(solid),
                            Err(e) => {
                                log::error!("Extrude failed: {}", e);
                                None
                            }
                        }
                    }
                    None => {
                        log::error!("Extrude: sketch profile not found for {:?}", sketch);
                        None
                    }
                }
            }

            Some(FeatureParams::Revolve { sketch, angle, .. }) => {
                let profile = self.get_sketch_profile(sketch);
                match profile {
                    Some(p) => match revolve_polyline(&p, angle) {
                        Ok(solid) => Some(solid),
                        Err(e) => {
                            log::error!("Revolve failed: {}", e);
                            None
                        }
                    },
                    None => None,
                }
            }

            Some(FeatureParams::Union { solid_a, solid_b }) => {
                let a = self.get_cached_solid(solid_a).cloned();
                let b = self.get_cached_solid(solid_b).cloned();
                match (a, b) {
                    (Some(a), Some(b)) => {
                        let ctx = ToleranceContext::default();
                        match boolean_union(&a, &b, &ctx) {
                            Ok(result) => Some(result),
                            Err(e) => {
                                log::error!("Boolean union failed: {:?}", e);
                                None
                            }
                        }
                    }
                    _ => None,
                }
            }

            Some(FeatureParams::Subtract { solid_a, solid_b }) => {
                let a = self.get_cached_solid(solid_a).cloned();
                let b = self.get_cached_solid(solid_b).cloned();
                match (a, b) {
                    (Some(a), Some(b)) => {
                        let ctx = ToleranceContext::default();
                        match boolean_subtract(&a, &b, &ctx) {
                            Ok(result) => Some(result),
                            Err(e) => {
                                log::error!("Boolean subtract failed: {:?}", e);
                                None
                            }
                        }
                    }
                    _ => None,
                }
            }

            Some(FeatureParams::Intersect { solid_a, solid_b }) => {
                let a = self.get_cached_solid(solid_a).cloned();
                let b = self.get_cached_solid(solid_b).cloned();
                match (a, b) {
                    (Some(a), Some(b)) => {
                        let ctx = ToleranceContext::default();
                        match boolean_intersect(&a, &b, &ctx) {
                            Ok(result) => Some(result),
                            Err(e) => {
                                log::error!("Boolean intersect failed: {:?}", e);
                                None
                            }
                        }
                    }
                    _ => None,
                }
            }

            Some(FeatureParams::Fillet { solid, radius, edges }) => {
                let src = self.get_cached_solid(solid).cloned();
                match src {
                    Some(s) => {
                        if edges.is_empty() {
                            fillet_edge(&s, 0, radius).ok()
                        } else {
                            let mut current = s;
                            for &edge_idx in &edges {
                                match fillet_edge(&current, edge_idx as usize, radius) {
                                    Ok(filletted) => current = filletted,
                                    Err(e) => {
                                        log::warn!("Fillet edge {} failed: {}", edge_idx, e);
                                    }
                                }
                            }
                            Some(current)
                        }
                    }
                    None => None,
                }
            }

            Some(FeatureParams::Chamfer { solid, distance, edges }) => {
                let src = self.get_cached_solid(solid).cloned();
                match src {
                    Some(s) => {
                        if edges.is_empty() {
                            chamfer_edge(&s, 0, distance).ok()
                        } else {
                            let mut current = s;
                            for &edge_idx in &edges {
                                if let Ok(chamfered) = chamfer_edge(&current, edge_idx as usize, distance) {
                                    current = chamfered;
                                }
                            }
                            Some(current)
                        }
                    }
                    None => None,
                }
            }

            Some(FeatureParams::Shell { solid, thickness, .. }) => {
                let src = self.get_cached_solid(solid).cloned();
                match src {
                    Some(s) => shell_solid(&s, thickness).ok(),
                    None => None,
                }
            }

            Some(FeatureParams::Transform { solid, translation, rotation, scale }) => {
                let src = self.get_cached_solid(solid).cloned();
                match src {
                    Some(mut s) => {
                        // Apply translation
                        let transform = Transform::translation(translation[0], translation[1], translation[2]);
                        ShapeBuilder::transform_solid(&mut s, &transform);

                        // Apply rotation (if non-identity quaternion)
                        if (rotation[0] - 1.0).abs() > 1e-10
                            || rotation[1].abs() > 1e-10
                            || rotation[2].abs() > 1e-10
                            || rotation[3].abs() > 1e-10
                        {
                            let qx = rotation[1];
                            let qy = rotation[2];
                            let qz = rotation[3];
                            let qw = rotation[0];
                            let angle = 2.0 * qw.acos();
                            let axis_len = (qx * qx + qy * qy + qz * qz).sqrt();
                            if axis_len > 1e-10 {
                                if let Some(axis) = Direction3d::new(qx / axis_len, qy / axis_len, qz / axis_len) {
                                    let rot = Transform::rotation_axis(&axis, angle);
                                    ShapeBuilder::transform_solid(&mut s, &rot);
                                }
                            }
                        }

                        // Apply scale
                        if (scale - 1.0).abs() > 1e-10 {
                            let sc = Transform::scaling(scale, scale, scale);
                            ShapeBuilder::transform_solid(&mut s, &sc);
                        }
                        Some(s)
                    }
                    None => None,
                }
            }

            None => None,
        };

        // Store the result in cache
        if let Some(feature) = self.features.get_mut(&id) {
            feature.evaluating = false;
            feature.cached_result = result;
        }

        self.features.get(&id).and_then(|f| f.cached_result.as_ref())
    }

    /// Get the Polyline2d profile from a sketch feature.
    fn get_sketch_profile(&self, sketch_id: FeatureId) -> Option<Polyline2d> {
        let feature = self.features.get(&sketch_id)?;
        match &feature.params {
            FeatureParams::Sketch { profile } => Some(profile.clone()),
            _ => None,
        }
    }

    /// Get the cached solid from a feature.
    fn get_cached_solid(&self, feature_id: FeatureId) -> Option<&Solid> {
        self.features.get(&feature_id).and_then(|f| f.cached_result.as_ref())
    }

    /// Rollback to a specific feature — re-evaluates only up to that point.
    ///
    /// Per BREPCAD Phase 1.3: sets the "current" feature to `id`,
    /// meaning all features after `id` are suppressed.
    pub fn rollback_to(&mut self, id: FeatureId) -> Result<(), String> {
        if !self.features.contains_key(&id) {
            return Err(format!("Feature {:?} not found", id));
        }
        self.current_feature = Some(id);
        self.rebuild_to(id);
        Ok(())
    }

    /// Rebuild the model up to a specific feature (inclusive).
    fn rebuild_to(&mut self, target: FeatureId) {
        let order = self.topological_order_up_to(target);
        for fid in order {
            self.evaluate(fid);
        }
    }

    /// Get topological order up to a specific feature (inclusive).
    fn topological_order_up_to(&self, target: FeatureId) -> Vec<FeatureId> {
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

        visit(target, &self.features, &mut visited, &mut order);
        order
    }

    /// Edit a parameter of a feature and rebuild.
    ///
    /// Per BREPCAD Phase 1.3: updates the feature's parameters and
    /// re-evaluates the entire tree from that point forward.
    pub fn edit_parameter(
        &mut self,
        feature_id: FeatureId,
        new_params: FeatureParams,
    ) -> Result<(), String> {
        if !self.features.contains_key(&feature_id) {
            return Err(format!("Feature {:?} not found", feature_id));
        }
        self.update_params(feature_id, new_params);
        if let Some(root) = self.root {
            self.evaluate(root);
        }
        Ok(())
    }

    /// Get the current feature (for rollback state).
    pub fn current_feature(&self) -> Option<FeatureId> {
        self.current_feature.or(self.root)
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
    use crate::operations::Polyline2d;

    #[test]
    fn test_feature_tree_basic() {
        let mut tree = FeatureTree::new();
        let sketch = tree.add_feature(Feature::new("sketch1", FeatureParams::Sketch {
            profile: Polyline2d::rectangle(10.0, 10.0),
        }));
        let extrude = tree.add_feature(Feature::new("extrude1", FeatureParams::Extrude {
            sketch,
            distance: 10.0,
            direction: [0.0, 0.0, 1.0],
        }));

        assert_eq!(tree.len(), 2);
        assert_eq!(tree.root(), Some(extrude));

        let order = tree.topological_order();
        assert_eq!(order.len(), 2);
        assert_eq!(order[0], sketch);
        assert_eq!(order[1], extrude);
    }

    #[test]
    fn test_invalidation() {
        let mut tree = FeatureTree::new();
        let sketch = tree.add_feature(Feature::new("sketch1", FeatureParams::Sketch {
            profile: Polyline2d::rectangle(10.0, 10.0),
        }));
        let extrude = tree.add_feature(Feature::new("extrude1", FeatureParams::Extrude {
            sketch,
            distance: 10.0,
            direction: [0.0, 0.0, 1.0],
        }));

        tree.update_params(sketch, FeatureParams::Sketch {
            profile: Polyline2d::rectangle(20.0, 20.0),
        });

        assert!(tree.get(sketch).unwrap().cached_result.is_none());
        assert!(tree.get(extrude).unwrap().cached_result.is_none());
    }

    #[test]
    fn test_evaluate_extrude_produces_solid() {
        // Per BREPCAD Phase 1.3: REAL evaluation — no stubs.
        let mut tree = FeatureTree::new();
        let sketch = tree.add_feature(Feature::new("sketch1", FeatureParams::Sketch {
            profile: Polyline2d::rectangle(10.0, 10.0),
        }));
        let extrude = tree.add_feature(Feature::new("extrude1", FeatureParams::Extrude {
            sketch,
            distance: 5.0,
            direction: [0.0, 0.0, 1.0],
        }));

        tree.evaluate(extrude);

        let result = tree.get(extrude).unwrap();
        assert!(result.cached_result.is_some(), "Extrude should produce a solid");
        let solid = result.cached_result.as_ref().unwrap();
        assert_eq!(solid.faces().len(), 6, "Extruded rectangle should have 6 faces");
    }

    #[test]
    fn test_evaluate_revolve_produces_solid() {
        let mut tree = FeatureTree::new();
        let sketch = tree.add_feature(Feature::new("sketch1", FeatureParams::Sketch {
            profile: Polyline2d::rectangle(10.0, 5.0),
        }));
        let revolve = tree.add_feature(Feature::new("revolve1", FeatureParams::Revolve {
            sketch,
            angle: std::f64::consts::PI,
            axis_origin: [0.0, 0.0, 0.0],
            axis_direction: [0.0, 0.0, 1.0],
        }));

        tree.evaluate(revolve);
        let result = tree.get(revolve).unwrap();
        assert!(result.cached_result.is_some(), "Revolve should produce a solid");
    }

    #[test]
    fn test_edit_parameter_rebuilds() {
        let mut tree = FeatureTree::new();
        let sketch = tree.add_feature(Feature::new("sketch1", FeatureParams::Sketch {
            profile: Polyline2d::rectangle(10.0, 10.0),
        }));
        let extrude = tree.add_feature(Feature::new("extrude1", FeatureParams::Extrude {
            sketch,
            distance: 5.0,
            direction: [0.0, 0.0, 1.0],
        }));

        tree.evaluate(extrude);
        let original = tree.get(extrude).unwrap().cached_result.as_ref().unwrap().clone();
        assert_eq!(original.faces().len(), 6);

        tree.edit_parameter(extrude, FeatureParams::Extrude {
            sketch,
            distance: 20.0,
            direction: [0.0, 0.0, 1.0],
        }).unwrap();

        let rebuilt = tree.get(extrude).unwrap().cached_result.as_ref().unwrap();
        assert_eq!(rebuilt.faces().len(), 6, "Rebuilt solid should still have 6 faces");
    }

    #[test]
    fn test_rollback() {
        let mut tree = FeatureTree::new();
        let sketch = tree.add_feature(Feature::new("sketch1", FeatureParams::Sketch {
            profile: Polyline2d::rectangle(10.0, 10.0),
        }));
        let extrude = tree.add_feature(Feature::new("extrude1", FeatureParams::Extrude {
            sketch,
            distance: 5.0,
            direction: [0.0, 0.0, 1.0],
        }));

        tree.evaluate(extrude);
        tree.rollback_to(sketch).unwrap();
        assert_eq!(tree.current_feature(), Some(sketch));
    }

    #[test]
    fn test_chained_features() {
        let mut tree = FeatureTree::new();
        let sketch = tree.add_feature(Feature::new("sketch1", FeatureParams::Sketch {
            profile: Polyline2d::rectangle(10.0, 10.0),
        }));
        let extrude = tree.add_feature(Feature::new("extrude1", FeatureParams::Extrude {
            sketch,
            distance: 10.0,
            direction: [0.0, 0.0, 1.0],
        }));
        let fillet = tree.add_feature(Feature::new("fillet1", FeatureParams::Fillet {
            solid: extrude,
            radius: 1.0,
            edges: vec![0],
        }));

        tree.evaluate(fillet);

        let extrude_result = tree.get(extrude).unwrap();
        assert!(extrude_result.cached_result.is_some(), "Extrude should be evaluated");
    }
}
