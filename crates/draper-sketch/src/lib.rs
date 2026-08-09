// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! 2D sketch geometry kernel with constraint solver.
//!
//! Per BREPCAD_IMPLEMENTATION_PLAN.md Phase 1.1: provides the foundation
//! for parametric sketch-based modeling. This crate is the 2D counterpart
//! to `draper-topology` (3D B-Rep) — it handles:
//!
//! - **Sketch entities**: points, lines, circles, arcs (2D primitives).
//! - **Constraints**: geometric (coincident, parallel, tangent) and
//!   dimensional (distance, angle) relationships between entities.
//! - **Constraint solver**: Newton-Raphson with SVD for stability,
//!   solves the system of constraint equations to find a valid
//!   configuration of entity positions.
//! - **Parameters**: named scalar values that can be referenced by
//!   dimensional constraints (e.g., `d1 = 50.0`) and edited
//!   parametrically.
//!
//! # Architecture
//!
//! The solver treats point coordinates as the unknowns. Each constraint
//! contributes one or more residual equations. The Jacobian is the
//! matrix of partial derivatives of residuals w.r.t. coordinates.
//! Newton-Raphson iterates: `x_{n+1} = x_n - J⁺ · r`, where `J⁺` is
//! the pseudo-inverse (via SVD) for robustness against singular Jacobians.
//!
//! # Example
//!
//! ```ignore
//! use draper_sketch::{Sketch2d, Constraint, ConstraintSolver};
//!
//! let mut sketch = Sketch2d::new();
//! let p1 = sketch.add_point(0.0, 0.0);
//! let p2 = sketch.add_point(10.0, 5.0);
//! let line = sketch.add_line(p1, p2);
//!
//! sketch.add_constraint(Constraint::Horizontal { line });
//! sketch.add_constraint(Constraint::Fixed { entity: p1 });
//!
//! let mut solver = ConstraintSolver::new();
//! solver.solve(&mut sketch, 50).unwrap();
//!
//! let p2_final = sketch.get_point(p2).unwrap();
//! assert!((p2_final.y - 0.0).abs() < 1e-6); // p2.y forced to 0
//! ```

pub mod projection;

use nalgebra::{DMatrix, DVector};
use std::collections::HashMap;

// ============================================================
// Error types
// ============================================================

/// Errors that can occur during constraint solving.
#[derive(Debug, Clone, thiserror::Error)]
pub enum SolverError {
    /// The system did not converge to a solution within `max_iter` iterations.
    #[error("Solver did not converge in {max_iter} iterations (final residual: {residual:.2e})")]
    DidNotConverge { max_iter: usize, residual: f64 },

    /// The system is over-constrained: more independent constraints than degrees of freedom.
    #[error("Over-constrained system: {constraint_count} constraints for {dof} degrees of freedom")]
    OverConstrained {
        constraint_count: usize,
        dof: usize,
    },

    /// The system is under-constrained: fewer constraints than degrees of freedom.
    /// The solver can still find a solution, but it may not be unique.
    #[error("Under-constrained system: {constraint_count} constraints for {dof} degrees of freedom (solution may not be unique)")]
    UnderConstrained {
        constraint_count: usize,
        dof: usize,
    },

    /// A constraint references a non-existent entity.
    #[error("Constraint references unknown entity: {entity_id}")]
    UnknownEntity { entity_id: u64 },

    /// A dimensional constraint references a non-existent parameter.
    #[error("Dimensional constraint references unknown parameter: {param_name}")]
    UnknownParameter { param_name: String },

    /// The Jacobian is singular (no solution exists).
    #[error("Singular Jacobian — system has no solution or is degenerate")]
    SingularJacobian,
}

// ============================================================
// Sketch entities
// ============================================================

/// A 2D sketch entity (point, line, circle, arc, spline, polygon).
#[derive(Debug, Clone)]
pub enum SketchEntity {
    /// A 2D point with coordinates (x, y).
    Point { id: u64, x: f64, y: f64 },
    /// A line segment between two points.
    Line { id: u64, start: u64, end: u64 },
    /// A circle with a center point and a radius parameter name.
    /// The actual radius value is stored in `Sketch2d::parameters`.
    Circle { id: u64, center: u64, radius_param: String },
    /// An arc from start to end around a center.
    Arc { id: u64, center: u64, start: u64, end: u64 },
    /// Phase 3.5: A spline through a list of control points (Catmull-Rom).
    Spline { id: u64, points: Vec<u64>, tension: f64 },
    /// Phase 3.5: A regular polygon (N-sided) inscribed in a circle.
    Polygon {
        id: u64,
        center: u64,
        radius_param: String,
        sides: u32,
        rotation_deg: f64,
    },
}

impl SketchEntity {
    /// Get the ID of this entity.
    pub fn id(&self) -> u64 {
        match self {
            SketchEntity::Point { id, .. }
            | SketchEntity::Line { id, .. }
            | SketchEntity::Circle { id, .. }
            | SketchEntity::Arc { id, .. }
            | SketchEntity::Spline { id, .. }
            | SketchEntity::Polygon { id, .. } => *id,
        }
    }

    /// Get all point IDs referenced by this entity.
    pub fn point_refs(&self) -> Vec<u64> {
        match self {
            SketchEntity::Point { id, .. } => vec![*id],
            SketchEntity::Line { start, end, .. } => vec![*start, *end],
            SketchEntity::Circle { center, .. } => vec![*center],
            SketchEntity::Arc { center, start, end, .. } => vec![*center, *start, *end],
            SketchEntity::Spline { points, .. } => points.clone(),
            SketchEntity::Polygon { center, .. } => vec![*center],
        }
    }
}

// ============================================================
// Constraints
// ============================================================

/// A geometric or dimensional constraint between sketch entities.
#[derive(Debug, Clone)]
pub enum Constraint {
    /// Two points must be at the same location.
    Coincident { p1: u64, p2: u64 },
    /// The distance between two points must equal a parameter value.
    Distance { p1: u64, p2: u64, value: String },
    /// A line must be horizontal (both endpoints have the same y).
    Horizontal { line: u64 },
    /// A line must be vertical (both endpoints have the same x).
    Vertical { line: u64 },
    /// Two lines must be parallel.
    Parallel { l1: u64, l2: u64 },
    /// Two lines must be perpendicular.
    Perpendicular { l1: u64, l2: u64 },
    /// Two entities must be tangent (touching at a single point).
    Tangent { e1: u64, e2: u64 },
    /// Two entities must have equal length (lines) or radius (circles).
    Equal { e1: u64, e2: u64 },
    /// An entity is fixed (cannot move during solving).
    Fixed { entity: u64 },
    /// The angle between two lines must equal a parameter value (in degrees).
    Angle { l1: u64, l2: u64, value: String },
    /// A point must lie on a line.
    PointOnLine { point: u64, line: u64 },
}

impl Constraint {
    /// Get all entity IDs referenced by this constraint.
    pub fn entity_refs(&self) -> Vec<u64> {
        match self {
            Constraint::Coincident { p1, p2 } => vec![*p1, *p2],
            Constraint::Distance { p1, p2, .. } => vec![*p1, *p2],
            Constraint::Horizontal { line }
            | Constraint::Vertical { line } => vec![*line],
            Constraint::Parallel { l1, l2 }
            | Constraint::Perpendicular { l1, l2 }
            | Constraint::Angle { l1, l2, .. } => vec![*l1, *l2],
            Constraint::Tangent { e1, e2 } | Constraint::Equal { e1, e2 } => vec![*e1, *e2],
            Constraint::Fixed { entity } => vec![*entity],
            Constraint::PointOnLine { point, line } => vec![*point, *line],
        }
    }
}

// ============================================================
// Sketch2d — the main sketch container
// ============================================================

/// A 2D sketch: a collection of entities, constraints, and parameters.
#[derive(Debug, Clone)]
pub struct Sketch2d {
    /// All entities in the sketch (points, lines, circles, arcs).
    pub entities: Vec<SketchEntity>,
    /// All constraints between entities.
    pub constraints: Vec<Constraint>,
    /// Named scalar parameters used by dimensional constraints.
    /// E.g., `"d1" -> 50.0` for a distance constraint with `value = "d1"`.
    pub parameters: HashMap<String, f64>,
    /// Next entity ID to assign.
    next_id: u64,
}

impl Default for Sketch2d {
    fn default() -> Self {
        Self::new()
    }
}

impl Sketch2d {
    /// Create an empty sketch.
    pub fn new() -> Self {
        Self {
            entities: Vec::new(),
            constraints: Vec::new(),
            parameters: HashMap::new(),
            next_id: 1,
        }
    }

    /// Allocate a unique entity ID.
    pub fn alloc_id(&mut self) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    /// Add a point at (x, y) and return its ID.
    pub fn add_point(&mut self, x: f64, y: f64) -> u64 {
        let id = self.alloc_id();
        self.entities.push(SketchEntity::Point { id, x, y });
        id
    }

    /// Add a line between two existing points and return its ID.
    pub fn add_line(&mut self, start: u64, end: u64) -> u64 {
        let id = self.alloc_id();
        self.entities.push(SketchEntity::Line { id, start, end });
        id
    }

    /// Add a circle with a center point and a radius parameter.
    /// The radius parameter is created in `parameters` with the given initial value.
    pub fn add_circle(&mut self, center: u64, radius_param: &str, radius_value: f64) -> u64 {
        let id = self.alloc_id();
        self.parameters.insert(radius_param.to_string(), radius_value);
        self.entities.push(SketchEntity::Circle {
            id,
            center,
            radius_param: radius_param.to_string(),
        });
        id
    }

    /// Add an arc from start to end around a center.
    pub fn add_arc(&mut self, center: u64, start: u64, end: u64) -> u64 {
        let id = self.alloc_id();
        self.entities.push(SketchEntity::Arc { id, center, start, end });
        id
    }

    /// Phase 3.5: Add a spline through a list of control points.
    /// Uses Catmull-Rom interpolation. Returns u64::MAX if < 2 points.
    pub fn add_spline(&mut self, points: Vec<u64>, tension: f64) -> u64 {
        if points.len() < 2 {
            return u64::MAX;
        }
        let id = self.alloc_id();
        let tension = tension.clamp(0.0, 1.0);
        self.entities.push(SketchEntity::Spline { id, points, tension });
        id
    }

    /// Phase 3.5: Add a regular polygon. Returns u64::MAX if sides < 3.
    pub fn add_polygon(
        &mut self, center: u64, radius_param: &str, radius_value: f64,
        sides: u32, rotation_deg: f64,
    ) -> u64 {
        if sides < 3 {
            return u64::MAX;
        }
        let id = self.alloc_id();
        self.parameters.insert(radius_param.to_string(), radius_value);
        self.entities.push(SketchEntity::Polygon {
            id, center, radius_param: radius_param.to_string(), sides, rotation_deg,
        });
        id
    }

    /// Phase 3.5: Tessellate a spline into a polyline using Catmull-Rom.
    pub fn tessellate_spline(&self, spline_id: u64, segments_per_span: u32) -> Vec<(f64, f64)> {
        let spline = self.entities.iter().find_map(|e| {
            if let SketchEntity::Spline { id, points, tension } = e {
                if *id == spline_id { return Some((points.clone(), *tension)); }
            }
            None
        });
        let (point_ids, tension) = match spline {
            Some(s) => s,
            None => return Vec::new(),
        };
        if point_ids.len() < 2 { return Vec::new(); }

        let mut pts: Vec<(f64, f64)> = Vec::with_capacity(point_ids.len());
        for pid in &point_ids {
            match self.get_point(*pid) {
                Some(p) => pts.push(p),
                None => return Vec::new(),
            }
        }

        let mut result: Vec<(f64, f64)> = Vec::new();
        let s = tension;
        let segs = segments_per_span.max(1) as usize;

        for i in 0..pts.len() - 1 {
            let p0 = if i == 0 { pts[i] } else { pts[i - 1] };
            let p1 = pts[i];
            let p2 = pts[i + 1];
            let p3 = if i + 2 < pts.len() { pts[i + 2] } else { pts[i + 1] };

            for j in 0..segs {
                let t = j as f64 / segs as f64;
                let t2 = t * t;
                let t3 = t2 * t;
                let alpha = 1.0 - s;
                let x = alpha * (0.5 * (2.0 * p1.0
                    + (-p0.0 + p2.0) * t
                    + (2.0 * p0.0 - 5.0 * p1.0 + 4.0 * p2.0 - p3.0) * t2
                    + (-p0.0 + 3.0 * p1.0 - 3.0 * p2.0 + p3.0) * t3))
                    + s * (p1.0 + (p2.0 - p1.0) * t);
                let y = alpha * (0.5 * (2.0 * p1.1
                    + (-p0.1 + p2.1) * t
                    + (2.0 * p0.1 - 5.0 * p1.1 + 4.0 * p2.1 - p3.1) * t2
                    + (-p0.1 + 3.0 * p1.1 - 3.0 * p2.1 + p3.1) * t3))
                    + s * (p1.1 + (p2.1 - p1.1) * t);
                result.push((x, y));
            }
        }
        result.push(*pts.last().unwrap());
        result
    }

    /// Phase 3.5: Tessellate a polygon into a closed vertex list.
    pub fn tessellate_polygon(&self, polygon_id: u64) -> Vec<(f64, f64)> {
        let poly = self.entities.iter().find_map(|e| {
            if let SketchEntity::Polygon { id, center, radius_param, sides, rotation_deg } = e {
                if *id == polygon_id {
                    return Some((*center, radius_param.clone(), *sides, *rotation_deg));
                }
            }
            None
        });
        let (center_id, radius_param, sides, rotation_deg) = match poly {
            Some(p) => p,
            None => return Vec::new(),
        };
        let (cx, cy) = match self.get_point(center_id) { Some(p) => p, None => return Vec::new() };
        let radius = match self.parameters.get(&radius_param) { Some(r) => *r, None => return Vec::new() };
        if sides < 3 || radius <= 0.0 { return Vec::new(); }

        let rot_rad = rotation_deg.to_radians();
        let mut verts: Vec<(f64, f64)> = Vec::with_capacity(sides as usize + 1);
        for i in 0..sides {
            let angle = rot_rad + (i as f64) * std::f64::consts::TAU / sides as f64;
            verts.push((cx + radius * angle.cos(), cy + radius * angle.sin()));
        }
        if let Some(first) = verts.first() { verts.push(*first); }
        verts
    }

    /// Add a constraint to the sketch.
    pub fn add_constraint(&mut self, constraint: Constraint) {
        self.constraints.push(constraint);
    }

    /// Get a point by ID.
    pub fn get_point(&self, id: u64) -> Option<(f64, f64)> {
        for e in &self.entities {
            if let SketchEntity::Point { id: pid, x, y } = e {
                if *pid == id {
                    return Some((*x, *y));
                }
            }
        }
        None
    }

    /// Set a point's coordinates by ID.
    pub fn set_point(&mut self, id: u64, x: f64, y: f64) -> bool {
        for e in &mut self.entities {
            if let SketchEntity::Point { id: pid, x: px, y: py } = e {
                if *pid == id {
                    *px = x;
                    *py = y;
                    return true;
                }
            }
        }
        false
    }

    /// Get a line by ID, returning (start_id, end_id).
    pub fn get_line(&self, id: u64) -> Option<(u64, u64)> {
        for e in &self.entities {
            if let SketchEntity::Line { id: lid, start, end } = e {
                if *lid == id {
                    return Some((*start, *end));
                }
            }
        }
        None
    }

    /// Get all point IDs in the sketch.
    pub fn point_ids(&self) -> Vec<u64> {
        self.entities
            .iter()
            .filter_map(|e| {
                if let SketchEntity::Point { id, .. } = e {
                    Some(*id)
                } else {
                    None
                }
            })
            .collect()
    }

    /// Set a parameter value.
    pub fn set_parameter(&mut self, name: &str, value: f64) {
        self.parameters.insert(name.to_string(), value);
    }

    /// Get a parameter value.
    pub fn get_parameter(&self, name: &str) -> Option<f64> {
        self.parameters.get(name).copied()
    }

    /// Resolve a constraint value that may be a parameter name or a literal number.
    /// E.g., "d1" → looks up parameters["d1"]; "50.0" → parses as 50.0.
    pub fn resolve_value(&self, value: &str) -> Result<f64, SolverError> {
        // First try as a parameter name
        if let Some(v) = self.parameters.get(value) {
            return Ok(*v);
        }
        // Then try as a literal number
        value
            .parse::<f64>()
            .map_err(|_| SolverError::UnknownParameter {
                param_name: value.to_string(),
            })
    }

    /// Count the degrees of freedom (2 per point: x and y).
    pub fn degrees_of_freedom(&self) -> usize {
        self.point_ids().len() * 2
    }

    /// Count the number of scalar equations contributed by all constraints.
    /// Most constraints contribute 1 equation; Coincident contributes 2 (x and y).
    pub fn constraint_equation_count(&self) -> usize {
        self.constraints
            .iter()
            .map(|c| match c {
                Constraint::Coincident { .. } => 2, // x1=x2, y1=y2
                Constraint::Distance { .. } => 1,
                Constraint::Horizontal { .. } => 1,
                Constraint::Vertical { .. } => 1,
                Constraint::Parallel { .. } => 1,
                Constraint::Perpendicular { .. } => 1,
                Constraint::Tangent { .. } => 1,
                Constraint::Equal { .. } => 1,
                Constraint::Fixed { .. } => 2, // fixes both x and y
                Constraint::Angle { .. } => 1,
                Constraint::PointOnLine { .. } => 1,
            })
            .sum()
    }

    /// Check if the sketch is over-constrained (more equations than DOF).
    pub fn is_over_constrained(&self) -> bool {
        self.constraint_equation_count() > self.degrees_of_freedom()
    }

    /// Check if the sketch is under-constrained (fewer equations than DOF).
    pub fn is_under_constrained(&self) -> bool {
        self.constraint_equation_count() < self.degrees_of_freedom()
    }
}

// ============================================================
// ConstraintSolver — Newton-Raphson with SVD
// ============================================================

/// A Newton-Raphson constraint solver with SVD-based pseudo-inverse
/// for numerical stability.
///
/// Per BREPCAD_IMPLEMENTATION_PLAN.md Phase 1.1: the solver treats
/// point coordinates as unknowns. Each constraint contributes residual
/// equations. The Jacobian `J` is the matrix of partial derivatives
/// of residuals w.r.t. coordinates. The Newton step is:
/// `x_{n+1} = x_n - J⁺ · r`, where `J⁺` is the pseudo-inverse.
pub struct ConstraintSolver {
    /// Tolerance for convergence (residual norm must be below this).
    pub tolerance: f64,
    /// Maximum number of iterations.
    pub max_iterations: usize,
    /// Step size damping factor (0 < step ≤ 1). Lower values improve
    /// stability for highly nonlinear constraints.
    pub step_size: f64,
}

impl Default for ConstraintSolver {
    fn default() -> Self {
        Self::new()
    }
}

impl ConstraintSolver {
    /// Create a new solver with default settings:
    /// tolerance = 1e-10, max_iterations = 50, step_size = 1.0.
    pub fn new() -> Self {
        Self {
            tolerance: 1e-10,
            max_iterations: 50,
            step_size: 1.0,
        }
    }

    /// Set the convergence tolerance.
    pub fn with_tolerance(mut self, tol: f64) -> Self {
        self.tolerance = tol;
        self
    }

    /// Set the maximum number of iterations.
    pub fn with_max_iterations(mut self, max: usize) -> Self {
        self.max_iterations = max;
        self
    }

    /// Set the step size damping factor.
    pub fn with_step_size(mut self, step: f64) -> Self {
        self.step_size = step;
        self
    }

    /// Solve the constraint system.
    ///
    /// Modifies point positions in `sketch` to satisfy all constraints.
    /// Returns `Ok(())` on convergence, or an error if the system is
    /// over-constrained, under-constrained, or fails to converge.
    pub fn solve(
        &mut self,
        sketch: &mut Sketch2d,
        max_iter: usize,
    ) -> Result<(), SolverError> {
        // Check for over-constrained system
        let dof = sketch.degrees_of_freedom();
        let n_eq = sketch.constraint_equation_count();
        if n_eq > dof {
            return Err(SolverError::OverConstrained {
                constraint_count: n_eq,
                dof,
            });
        }
        if n_eq < dof {
            // Under-constrained — we can still try to solve, but warn.
            log::debug!(
                "Under-constrained sketch: {} equations for {} DOF — solution may not be unique",
                n_eq,
                dof
            );
        }

        // Map point IDs to indices in the state vector
        let point_ids = sketch.point_ids();
        if point_ids.is_empty() {
            return Ok(()); // Nothing to solve
        }
        let id_to_idx: HashMap<u64, usize> = point_ids
            .iter()
            .enumerate()
            .map(|(i, &id)| (id, i))
            .collect();

        // Build initial state vector: [x0, y0, x1, y1, ...]
        let n = point_ids.len();
        let mut state = DVector::<f64>::zeros(2 * n);
        for (i, &pid) in point_ids.iter().enumerate() {
            if let Some((x, y)) = sketch.get_point(pid) {
                state[2 * i] = x;
                state[2 * i + 1] = y;
            }
        }

        // Identify fixed points (their coordinates should not change)
        let mut fixed_mask = vec![false; 2 * n];
        for c in &sketch.constraints {
            if let Constraint::Fixed { entity } = c {
                if let Some(&idx) = id_to_idx.get(entity) {
                    fixed_mask[2 * idx] = true;
                    fixed_mask[2 * idx + 1] = true;
                }
            }
        }

        // Newton-Raphson iteration
        let mut last_residual = f64::MAX;
        for iter in 0..max_iter {
            // Build residual vector and Jacobian
            let (residual, jacobian) = self.build_system(sketch, &point_ids, &id_to_idx, &state);

            let residual_norm = residual.norm();
            if residual_norm < self.tolerance {
                log::debug!("Solver converged in {} iterations (residual: {:.2e})", iter, residual_norm);
                return Ok(());
            }

            // Check for divergence
            if iter > 0 && residual_norm > last_residual * 1e6 {
                log::warn!("Solver diverging at iter {} (residual: {:.2e})", iter, residual_norm);
                return Err(SolverError::DidNotConverge {
                    max_iter,
                    residual: residual_norm,
                });
            }
            last_residual = residual_norm;

            // Solve J · delta = -residual via SVD pseudo-inverse
            // J may not be square (under/over-determined), so use SVD.
            let delta = if jacobian.nrows() > 0 && jacobian.ncols() > 0 {
                match self.solve_svd(&jacobian, &residual) {
                    Some(d) => d,
                    None => return Err(SolverError::SingularJacobian),
                }
            } else {
                DVector::zeros(2 * n)
            };

            // Apply delta with damping, respecting fixed mask
            for i in 0..2 * n {
                if !fixed_mask[i] {
                    state[i] += self.step_size * delta[i];
                }
            }

            // Write state back to sketch
            for (i, &pid) in point_ids.iter().enumerate() {
                sketch.set_point(pid, state[2 * i], state[2 * i + 1]);
            }
        }

        Err(SolverError::DidNotConverge {
            max_iter,
            residual: last_residual,
        })
    }

    /// Build the residual vector and Jacobian matrix for the current state.
    fn build_system(
        &self,
        sketch: &Sketch2d,
        point_ids: &[u64],
        id_to_idx: &HashMap<u64, usize>,
        state: &DVector<f64>,
    ) -> (DVector<f64>, DMatrix<f64>) {
        let constraints = &sketch.constraints;
        let n_eq: usize = constraints
            .iter()
            .map(|c| match c {
                Constraint::Coincident { .. } => 2,
                Constraint::Fixed { .. } => 2,
                _ => 1,
            })
            .sum();

        let n_vars = 2 * point_ids.len();
        let mut residual = DVector::<f64>::zeros(n_eq);
        let mut jacobian = DMatrix::<f64>::zeros(n_eq, n_vars);

        let mut row = 0;
        for c in constraints {
            let n_rows = match c {
                Constraint::Coincident { .. } => 2,
                Constraint::Fixed { .. } => 2,
                _ => 1,
            };
            self.fill_constraint_residuals(
                c,
                sketch,
                id_to_idx,
                state,
                &mut residual,
                &mut jacobian,
                row,
            );
            row += n_rows;
        }

        (residual, jacobian)
    }

    /// Fill the residual and Jacobian rows for a single constraint.
    fn fill_constraint_residuals(
        &self,
        constraint: &Constraint,
        sketch: &Sketch2d,
        id_to_idx: &HashMap<u64, usize>,
        state: &DVector<f64>,
        residual: &mut DVector<f64>,
        jacobian: &mut DMatrix<f64>,
        row: usize,
    ) {
        let eps = 1e-8;

        // Helper: get point coordinates from state
        let pt = |id: u64| -> (f64, f64, usize) {
            let idx = *id_to_idx.get(&id).unwrap_or(&0);
            (state[2 * idx], state[2 * idx + 1], idx)
        };

        match constraint {
            Constraint::Coincident { p1, p2 } => {
                let (x1, y1, i1) = pt(*p1);
                let (x2, y2, i2) = pt(*p2);
                // Residual: (x1 - x2, y1 - y2)
                residual[row] = x1 - x2;
                residual[row + 1] = y1 - y2;
                // Jacobian: d(x1-x2)/dx1 = 1, d(x1-x2)/dx2 = -1
                jacobian[(row, 2 * i1)] = 1.0;
                jacobian[(row, 2 * i1 + 1)] = 0.0;
                jacobian[(row, 2 * i2)] = -1.0;
                jacobian[(row, 2 * i2 + 1)] = 0.0;
                jacobian[(row + 1, 2 * i1)] = 0.0;
                jacobian[(row + 1, 2 * i1 + 1)] = 1.0;
                jacobian[(row + 1, 2 * i2)] = 0.0;
                jacobian[(row + 1, 2 * i2 + 1)] = -1.0;
            }

            Constraint::Distance { p1, p2, value } => {
                let (x1, y1, i1) = pt(*p1);
                let (x2, y2, i2) = pt(*p2);
                let target = sketch.resolve_value(value).unwrap_or(0.0);
                let dx = x2 - x1;
                let dy = y2 - y1;
                let dist = (dx * dx + dy * dy).sqrt();
                // Residual: dist - target
                residual[row] = dist - target;
                // Jacobian: d(dist)/dx1 = -dx/dist, d(dist)/dx2 = dx/dist
                if dist > eps {
                    jacobian[(row, 2 * i1)] = -dx / dist;
                    jacobian[(row, 2 * i1 + 1)] = -dy / dist;
                    jacobian[(row, 2 * i2)] = dx / dist;
                    jacobian[(row, 2 * i2 + 1)] = dy / dist;
                }
            }

            Constraint::Horizontal { line } => {
                if let Some((start, end)) = sketch.get_line(*line) {
                    let (_, y1, i1) = pt(start);
                    let (_, y2, i2) = pt(end);
                    // Residual: y1 - y2
                    residual[row] = y1 - y2;
                    jacobian[(row, 2 * i1 + 1)] = 1.0;
                    jacobian[(row, 2 * i2 + 1)] = -1.0;
                }
            }

            Constraint::Vertical { line } => {
                if let Some((start, end)) = sketch.get_line(*line) {
                    let (x1, _, i1) = pt(start);
                    let (x2, _, i2) = pt(end);
                    // Residual: x1 - x2
                    residual[row] = x1 - x2;
                    jacobian[(row, 2 * i1)] = 1.0;
                    jacobian[(row, 2 * i2)] = -1.0;
                }
            }

            Constraint::Parallel { l1, l2 } => {
                // For parallel: cross product of direction vectors = 0
                // (dx1 * dy2 - dy1 * dx2 = 0)
                if let (Some((s1, e1)), Some((s2, e2))) =
                    (sketch.get_line(*l1), sketch.get_line(*l2))
                {
                    let (x1, y1, i1) = pt(s1);
                    let (x2, y2, i2) = pt(e1);
                    let (x3, y3, i3) = pt(s2);
                    let (x4, y4, i4) = pt(e2);
                    let dx1 = x2 - x1;
                    let dy1 = y2 - y1;
                    let dx2 = x4 - x3;
                    let dy2 = y4 - y3;
                    residual[row] = dx1 * dy2 - dy1 * dx2;
                    // Jacobian (partial derivatives)
                    jacobian[(row, 2 * i1)] = -dy2;
                    jacobian[(row, 2 * i1 + 1)] = dx2;
                    jacobian[(row, 2 * i2)] = dy2;
                    jacobian[(row, 2 * i2 + 1)] = -dx2;
                    jacobian[(row, 2 * i3)] = dy1;
                    jacobian[(row, 2 * i3 + 1)] = -dx1;
                    jacobian[(row, 2 * i4)] = -dy1;
                    jacobian[(row, 2 * i4 + 1)] = dx1;
                }
            }

            Constraint::Perpendicular { l1, l2 } => {
                // For perpendicular: dot product of direction vectors = 0
                if let (Some((s1, e1)), Some((s2, e2))) =
                    (sketch.get_line(*l1), sketch.get_line(*l2))
                {
                    let (x1, y1, i1) = pt(s1);
                    let (x2, y2, i2) = pt(e1);
                    let (x3, y3, i3) = pt(s2);
                    let (x4, y4, i4) = pt(e2);
                    let dx1 = x2 - x1;
                    let dy1 = y2 - y1;
                    let dx2 = x4 - x3;
                    let dy2 = y4 - y3;
                    residual[row] = dx1 * dx2 + dy1 * dy2;
                    jacobian[(row, 2 * i1)] = -dx2;
                    jacobian[(row, 2 * i1 + 1)] = -dy2;
                    jacobian[(row, 2 * i2)] = dx2;
                    jacobian[(row, 2 * i2 + 1)] = dy2;
                    jacobian[(row, 2 * i3)] = -dx1;
                    jacobian[(row, 2 * i3 + 1)] = -dy1;
                    jacobian[(row, 2 * i4)] = dx1;
                    jacobian[(row, 2 * i4 + 1)] = dy1;
                }
            }

            Constraint::Fixed { entity } => {
                // Fixed constraint: residual = (x - x0, y - y0)
                // The fixed mask prevents delta from being applied, so
                // the residual is always 0 (no-op). We fill it for completeness.
                let (x, y, idx) = pt(*entity);
                residual[row] = 0.0; // already at fixed position
                residual[row + 1] = 0.0;
                // Jacobian is zero — fixed points don't contribute
                let _ = (x, y, idx);
            }

            Constraint::Angle { l1, l2, value } => {
                // Angle between two lines (in radians).
                // residual = atan2(cross, dot) - target_angle
                if let (Some((s1, e1)), Some((s2, e2))) =
                    (sketch.get_line(*l1), sketch.get_line(*l2))
                {
                    let (x1, y1, i1) = pt(s1);
                    let (x2, y2, i2) = pt(e1);
                    let (x3, y3, i3) = pt(s2);
                    let (x4, y4, i4) = pt(e2);
                    let dx1 = x2 - x1;
                    let dy1 = y2 - y1;
                    let dx2 = x4 - x3;
                    let dy2 = y4 - y3;
                    let cross = dx1 * dy2 - dy1 * dx2;
                    let dot = dx1 * dx2 + dy1 * dy2;
                    let angle = cross.atan2(dot);
                    let target_deg = sketch.resolve_value(value).unwrap_or(0.0);
                    let target_rad = target_deg.to_radians();
                    residual[row] = angle - target_rad;
                    // Jacobian of atan2(cross, dot) w.r.t. coordinates
                    // is complex; use finite-difference approximation
                    let denom = cross * cross + dot * dot;
                    if denom > eps {
                        // d(atan2(cross,dot))/d(var) = (dot * d_cross - cross * d_dot) / denom
                        // For dx1: d_cross/dx1 = dy2, d_dot/dx1 = dx2
                        jacobian[(row, 2 * i1)] = (dot * (-dy2) - cross * (-dx2)) / denom;
                        jacobian[(row, 2 * i1 + 1)] = (dot * dx2 - cross * dy2) / denom;
                        jacobian[(row, 2 * i2)] = (dot * dy2 - cross * dx2) / denom;
                        jacobian[(row, 2 * i2 + 1)] = (dot * (-dx2) - cross * (-dy2)) / denom;
                        jacobian[(row, 2 * i3)] = (dot * dy1 - cross * dx1) / denom;
                        jacobian[(row, 2 * i3 + 1)] = (dot * (-dx1) - cross * (-dy1)) / denom;
                        jacobian[(row, 2 * i4)] = (dot * (-dy1) - cross * (-dx1)) / denom;
                        jacobian[(row, 2 * i4 + 1)] = (dot * dx1 - cross * dy1) / denom;
                    }
                }
            }

            Constraint::Tangent { e1: _, e2: _ } => {
                // Tangent constraint — simplified: residual = 0 (placeholder)
                // Full implementation requires entity-type-specific logic.
                residual[row] = 0.0;
            }

            Constraint::Equal { e1: _, e2: _ } => {
                // Equal constraint — simplified: residual = 0 (placeholder)
                residual[row] = 0.0;
            }

            Constraint::PointOnLine { point, line } => {
                // Point on line: cross product of (line_dir) and (point - line_start) = 0
                if let Some((start, end)) = sketch.get_line(*line) {
                    let (x0, y0, i0) = pt(*point);
                    let (x1, y1, i1) = pt(start);
                    let (x2, y2, i2) = pt(end);
                    let dx = x2 - x1;
                    let dy = y2 - y1;
                    let px = x0 - x1;
                    let py = y0 - y1;
                    residual[row] = dx * py - dy * px;
                    jacobian[(row, 2 * i0)] = -dy;
                    jacobian[(row, 2 * i0 + 1)] = dx;
                    jacobian[(row, 2 * i1)] = dy - (-dy);
                    jacobian[(row, 2 * i1 + 1)] = -dx - dx;
                    jacobian[(row, 2 * i2)] = -py;
                    jacobian[(row, 2 * i2 + 1)] = px;
                }
            }
        }
    }

    /// Solve the linear system J · delta = -residual via SVD pseudo-inverse.
    /// Returns None if the SVD fails (singular matrix).
    fn solve_svd(
        &self,
        jacobian: &DMatrix<f64>,
        residual: &DVector<f64>,
    ) -> Option<DVector<f64>> {
        let svd = jacobian.clone().svd(true, true);
        // Solve J · delta = -residual
        // => delta = -J⁺ · residual
        let neg_residual = -residual;
        svd.solve(&neg_residual, 1e-12).ok()
    }
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_solve_horizontal_line() {
        // A line that should become horizontal after solving.
        let mut sketch = Sketch2d::new();
        let p1 = sketch.add_point(0.0, 0.0);
        let p2 = sketch.add_point(10.0, 5.0);
        let line = sketch.add_line(p1, p2);

        sketch.add_constraint(Constraint::Horizontal { line });
        sketch.add_constraint(Constraint::Fixed { entity: p1 });

        let mut solver = ConstraintSolver::new();
        let result = solver.solve(&mut sketch, 50);
        assert!(result.is_ok(), "Solver failed: {:?}", result.err());

        let p2_final = sketch.get_point(p2).unwrap();
        assert!(
            (p2_final.1 - 0.0).abs() < 1e-6,
            "p2.y should be 0 after Horizontal constraint, got {}",
            p2_final.1
        );
    }

    #[test]
    fn test_solve_vertical_line() {
        let mut sketch = Sketch2d::new();
        let p1 = sketch.add_point(0.0, 0.0);
        let p2 = sketch.add_point(5.0, 10.0);
        let line = sketch.add_line(p1, p2);

        sketch.add_constraint(Constraint::Vertical { line });
        sketch.add_constraint(Constraint::Fixed { entity: p1 });

        let mut solver = ConstraintSolver::new();
        solver.solve(&mut sketch, 50).unwrap();

        let p2_final = sketch.get_point(p2).unwrap();
        assert!(
            (p2_final.0 - 0.0).abs() < 1e-6,
            "p2.x should be 0 after Vertical constraint, got {}",
            p2_final.0
        );
    }

    #[test]
    fn test_solve_distance_constraint() {
        let mut sketch = Sketch2d::new();
        let p1 = sketch.add_point(0.0, 0.0);
        let p2 = sketch.add_point(10.0, 0.0);
        let line = sketch.add_line(p1, p2);

        sketch.add_constraint(Constraint::Distance {
            p1,
            p2,
            value: "50.0".to_string(),
        });
        sketch.add_constraint(Constraint::Fixed { entity: p1 });
        sketch.add_constraint(Constraint::Horizontal { line });

        let mut solver = ConstraintSolver::new();
        let result = solver.solve(&mut sketch, 50);
        assert!(result.is_ok(), "Solver failed: {:?}", result.err());

        let p2_final = sketch.get_point(p2).unwrap();
        let distance = ((p2_final.0 - 0.0).powi(2) + (p2_final.1 - 0.0).powi(2)).sqrt();
        assert!(
            (distance - 50.0).abs() < 1e-4,
            "Distance should be 50, got {}",
            distance
        );
    }

    #[test]
    fn test_solve_distance_with_parameter() {
        // Use a named parameter instead of a literal value
        let mut sketch = Sketch2d::new();
        sketch.set_parameter("d1", 25.0);
        let p1 = sketch.add_point(0.0, 0.0);
        let p2 = sketch.add_point(10.0, 0.0);
        let line = sketch.add_line(p1, p2);

        sketch.add_constraint(Constraint::Distance {
            p1,
            p2,
            value: "d1".to_string(),
        });
        sketch.add_constraint(Constraint::Horizontal { line });
        sketch.add_constraint(Constraint::Fixed { entity: p1 });

        let mut solver = ConstraintSolver::new();
        solver.solve(&mut sketch, 50).unwrap();

        let p2_final = sketch.get_point(p2).unwrap();
        let distance = (p2_final.0.powi(2) + p2_final.1.powi(2)).sqrt();
        assert!(
            (distance - 25.0).abs() < 1e-4,
            "Distance should be 25 (from parameter d1), got {}",
            distance
        );
    }

    #[test]
    fn test_solve_coincident() {
        let mut sketch = Sketch2d::new();
        let p1 = sketch.add_point(0.0, 0.0);
        let p2 = sketch.add_point(5.0, 5.0);

        sketch.add_constraint(Constraint::Coincident { p1, p2 });
        sketch.add_constraint(Constraint::Fixed { entity: p1 });

        let mut solver = ConstraintSolver::new();
        solver.solve(&mut sketch, 50).unwrap();

        let p2_final = sketch.get_point(p2).unwrap();
        assert_relative_eq!(p2_final.0, 0.0, epsilon = 1e-6);
        assert_relative_eq!(p2_final.1, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_solve_parallel_lines() {
        let mut sketch = Sketch2d::new();
        let p1 = sketch.add_point(0.0, 0.0);
        let p2 = sketch.add_point(10.0, 5.0);
        let p3 = sketch.add_point(0.0, 10.0);
        let p4 = sketch.add_point(10.0, 13.0);

        let l1 = sketch.add_line(p1, p2);
        let l2 = sketch.add_line(p3, p4);

        sketch.add_constraint(Constraint::Parallel { l1, l2 });
        sketch.add_constraint(Constraint::Fixed { entity: p1 });
        sketch.add_constraint(Constraint::Fixed { entity: p2 });
        sketch.add_constraint(Constraint::Fixed { entity: p3 });

        let mut solver = ConstraintSolver::new();
        solver.solve(&mut sketch, 100).unwrap();

        // After solving, both lines should have the same direction
        let p2_final = sketch.get_point(p2).unwrap();
        let p4_final = sketch.get_point(p4).unwrap();
        let p3_final = sketch.get_point(p3).unwrap();
        let dir1 = (p2_final.0 - 0.0, p2_final.1 - 0.0);
        let dir2 = (p4_final.0 - p3_final.0, p4_final.1 - p3_final.1);
        let cross = dir1.0 * dir2.1 - dir1.1 * dir2.0;
        assert!(
            cross.abs() < 1e-3,
            "Lines should be parallel (cross product ≈ 0), got {}",
            cross
        );
    }

    #[test]
    fn test_solve_perpendicular_lines() {
        let mut sketch = Sketch2d::new();
        let p1 = sketch.add_point(0.0, 0.0);
        let p2 = sketch.add_point(10.0, 0.0);
        let p3 = sketch.add_point(5.0, 0.0);
        let p4 = sketch.add_point(5.0, 1.0);

        let l1 = sketch.add_line(p1, p2);
        let l2 = sketch.add_line(p3, p4);

        // l1 is horizontal, l2 should become vertical
        sketch.add_constraint(Constraint::Horizontal { line: l1 });
        sketch.add_constraint(Constraint::Perpendicular { l1, l2 });
        sketch.add_constraint(Constraint::Fixed { entity: p1 });
        sketch.add_constraint(Constraint::Fixed { entity: p2 });
        sketch.add_constraint(Constraint::Fixed { entity: p3 });

        let mut solver = ConstraintSolver::new();
        solver.solve(&mut sketch, 100).unwrap();

        let p4_final = sketch.get_point(p4).unwrap();
        let p3_final = sketch.get_point(p3).unwrap();
        let dir2 = (p4_final.0 - p3_final.0, p4_final.1 - p3_final.1);
        // Perpendicular to horizontal means vertical: dir2.x ≈ 0
        assert!(
            dir2.0.abs() < 1e-3,
            "l2 should be vertical (dir.x ≈ 0), got {}",
            dir2.0
        );
    }

    #[test]
    fn test_detect_over_constrained() {
        let mut sketch = Sketch2d::new();
        let p1 = sketch.add_point(0.0, 0.0);
        let p2 = sketch.add_point(10.0, 0.0);

        // Contradictory distance constraints
        sketch.add_constraint(Constraint::Distance {
            p1,
            p2,
            value: "10".to_string(),
        });
        sketch.add_constraint(Constraint::Distance {
            p1,
            p2,
            value: "20".to_string(),
        });
        sketch.add_constraint(Constraint::Fixed { entity: p1 });
        sketch.add_constraint(Constraint::Fixed { entity: p2 });

        let mut solver = ConstraintSolver::new();
        let result = solver.solve(&mut sketch, 50);
        // Should fail — over-constrained (4 equations for 4 DOF, but contradictory)
        assert!(
            result.is_err(),
            "Over-constrained system should fail, got {:?}",
            result
        );
    }

    #[test]
    fn test_under_constrained_warns() {
        // Single point, no constraints — under-constrained
        let mut sketch = Sketch2d::new();
        let _p1 = sketch.add_point(10.0, 20.0);

        let mut solver = ConstraintSolver::new();
        // Should succeed (trivially — no constraints to satisfy)
        let result = solver.solve(&mut sketch, 50);
        assert!(result.is_ok(), "Under-constrained should still solve: {:?}", result.err());
        assert!(sketch.is_under_constrained());
    }

    #[test]
    fn test_no_points_succeeds() {
        let mut sketch = Sketch2d::new();
        let mut solver = ConstraintSolver::new();
        let result = solver.solve(&mut sketch, 50);
        assert!(result.is_ok());
    }

    #[test]
    fn test_point_on_line() {
        let mut sketch = Sketch2d::new();
        let p1 = sketch.add_point(0.0, 0.0);
        let p2 = sketch.add_point(10.0, 0.0);
        let p3 = sketch.add_point(5.0, 3.0); // off the line
        let line = sketch.add_line(p1, p2);

        sketch.add_constraint(Constraint::Fixed { entity: p1 });
        sketch.add_constraint(Constraint::Fixed { entity: p2 });
        sketch.add_constraint(Constraint::PointOnLine { point: p3, line });

        let mut solver = ConstraintSolver::new().with_tolerance(1e-6);
        solver.solve(&mut sketch, 200).unwrap();

        let p3_final = sketch.get_point(p3).unwrap();
        // p3 should be on the line y=0
        assert!(
            p3_final.1.abs() < 1e-3,
            "p3 should be on the line (y≈0), got y={}",
            p3_final.1
        );
    }

    #[test]
    fn test_dof_counting() {
        let mut sketch = Sketch2d::new();
        assert_eq!(sketch.degrees_of_freedom(), 0);

        sketch.add_point(0.0, 0.0);
        assert_eq!(sketch.degrees_of_freedom(), 2);

        sketch.add_point(1.0, 1.0);
        assert_eq!(sketch.degrees_of_freedom(), 4);

        // A line doesn't add DOF (it references existing points)
        let p1 = sketch.point_ids()[0];
        let p2 = sketch.point_ids()[1];
        sketch.add_line(p1, p2);
        assert_eq!(sketch.degrees_of_freedom(), 4);
    }

    #[test]
    fn test_constraint_equation_count() {
        let mut sketch = Sketch2d::new();
        let p1 = sketch.add_point(0.0, 0.0);
        let p2 = sketch.add_point(1.0, 1.0);

        sketch.add_constraint(Constraint::Fixed { entity: p1 });
        assert_eq!(sketch.constraint_equation_count(), 2); // Fixed = 2

        sketch.add_constraint(Constraint::Distance {
            p1,
            p2,
            value: "5".to_string(),
        });
        assert_eq!(sketch.constraint_equation_count(), 3); // 2 + 1

        sketch.add_constraint(Constraint::Coincident { p1, p2 });
        assert_eq!(sketch.constraint_equation_count(), 5); // 3 + 2
    }

    // ─── Phase 3.5: Spline + Polygon tests ───

    #[test]
    fn test_add_spline_basic() {
        let mut sketch = Sketch2d::new();
        let p0 = sketch.add_point(0.0, 0.0);
        let p1 = sketch.add_point(10.0, 5.0);
        let p2 = sketch.add_point(20.0, 0.0);
        let id = sketch.add_spline(vec![p0, p1, p2], 0.0);
        assert!(id != u64::MAX);
    }

    #[test]
    fn test_add_spline_too_few_points() {
        let mut sketch = Sketch2d::new();
        let p0 = sketch.add_point(0.0, 0.0);
        assert_eq!(sketch.add_spline(vec![p0], 0.0), u64::MAX);
    }

    #[test]
    fn test_tessellate_spline_endpoints() {
        let mut sketch = Sketch2d::new();
        let p0 = sketch.add_point(0.0, 0.0);
        let p1 = sketch.add_point(10.0, 10.0);
        let p2 = sketch.add_point(20.0, 0.0);
        let id = sketch.add_spline(vec![p0, p1, p2], 0.0);
        let pts = sketch.tessellate_spline(id, 10);
        assert!(pts.len() >= 21);
        assert!((pts[0].0 - 0.0).abs() < 1e-9);
        let last = *pts.last().unwrap();
        assert!((last.0 - 20.0).abs() < 1e-9);
    }

    #[test]
    fn test_tessellate_spline_missing_id() {
        let mut sketch = Sketch2d::new();
        assert!(sketch.tessellate_spline(9999, 10).is_empty());
    }

    #[test]
    fn test_add_polygon_basic() {
        let mut sketch = Sketch2d::new();
        let center = sketch.add_point(0.0, 0.0);
        let id = sketch.add_polygon(center, "r", 10.0, 6, 0.0);
        assert!(id != u64::MAX);
        assert_eq!(sketch.get_parameter("r"), Some(10.0));
    }

    #[test]
    fn test_add_polygon_too_few_sides() {
        let mut sketch = Sketch2d::new();
        let center = sketch.add_point(0.0, 0.0);
        assert_eq!(sketch.add_polygon(center, "r", 10.0, 2, 0.0), u64::MAX);
    }

    #[test]
    fn test_tessellate_polygon_hex() {
        let mut sketch = Sketch2d::new();
        let center = sketch.add_point(5.0, 5.0);
        let id = sketch.add_polygon(center, "r", 10.0, 6, 0.0);
        let verts = sketch.tessellate_polygon(id);
        assert_eq!(verts.len(), 7); // 6 + closing
        assert!((verts[0].0 - verts[6].0).abs() < 1e-9);
        assert!((verts[0].0 - 15.0).abs() < 1e-9); // center + (radius, 0)
    }

    #[test]
    fn test_tessellate_polygon_rotation() {
        let mut sketch = Sketch2d::new();
        let center = sketch.add_point(0.0, 0.0);
        let id = sketch.add_polygon(center, "r", 10.0, 4, 90.0);
        let verts = sketch.tessellate_polygon(id);
        assert!((verts[0].0 - 0.0).abs() < 1e-9);
        assert!((verts[0].1 - 10.0).abs() < 1e-9);
    }

    #[test]
    fn test_spline_point_refs() {
        let mut sketch = Sketch2d::new();
        let p0 = sketch.add_point(0.0, 0.0);
        let p1 = sketch.add_point(10.0, 0.0);
        let p2 = sketch.add_point(20.0, 0.0);
        let id = sketch.add_spline(vec![p0, p1, p2], 0.0);
        let spline = sketch.entities.iter().find(|e| e.id() == id).unwrap();
        assert_eq!(spline.point_refs(), vec![p0, p1, p2]);
    }
}
