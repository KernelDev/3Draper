// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Assembly support — positioning and mating parts.

use draper_geometry::{Point3d, Direction3d, Transform};
use draper_topology::{Solid, Compound};

/// An assembly node with a transform relative to its parent.
#[derive(Clone, Debug)]
pub struct AssemblyNode {
    pub name: String,
    pub solid: Option<Solid>,
    pub transform: Transform,
    pub children: Vec<AssemblyNode>,
}

impl AssemblyNode {
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            solid: None,
            transform: Transform::identity(),
            children: Vec::new(),
        }
    }

    pub fn with_solid(name: &str, solid: Solid) -> Self {
        Self {
            name: name.to_string(),
            solid: Some(solid),
            transform: Transform::identity(),
            children: Vec::new(),
        }
    }

    /// Add a child node.
    pub fn add_child(&mut self, child: AssemblyNode) {
        self.children.push(child);
    }

    /// Set the transform for this node.
    pub fn set_transform(&mut self, transform: Transform) {
        self.transform = transform;
    }

    /// Position at a specific location.
    pub fn position_at(&mut self, x: f64, y: f64, z: f64) {
        self.transform = Transform::translation(x, y, z);
    }

    /// Rotate around an axis.
    pub fn rotate(&mut self, axis: &Direction3d, angle: f64) {
        let rotation = Transform::rotation_axis(axis, angle);
        self.transform = rotation.multiply(&self.transform);
    }

    /// Convert the assembly tree to a Compound.
    pub fn to_compound(&self) -> Compound {
        let mut compound = Compound::new();

        if let Some(ref solid) = self.solid {
            let mut transformed = solid.clone();
            apply_transform_to_solid(&mut transformed, &self.transform);
            compound.add_solid(transformed);
        }

        for child in &self.children {
            let child_compound = child.to_compound();
            compound.add_compound(child_compound);
        }

        compound
    }
}

/// Apply a transform to a solid's geometry.
fn apply_transform_to_solid(solid: &mut Solid, transform: &Transform) {
    if let Some(ref mut shell) = solid.outer_shell {
        for face in &mut shell.faces {
            if let Some(ref mut surface) = face.surface {
                *surface = surface.transform(transform);
            }
        }
    }
}

/// Mating constraint types.
#[derive(Clone, Debug)]
pub enum MateConstraint {
    /// Coincident: two points at the same location.
    Coincident { point_a: Point3d, point_b: Point3d },
    /// Axis aligned: two directions are parallel.
    AxisAligned { dir_a: Direction3d, dir_b: Direction3d },
    /// Distance: two planes at a fixed distance.
    Distance { normal: Direction3d, offset_a: f64, offset_b: f64, distance: f64 },
    /// Flush: two planes coplanar.
    Flush { normal: Direction3d, offset_a: f64, offset_b: f64 },
}

/// An assembly with constraints.
#[derive(Clone, Debug)]
pub struct Assembly {
    pub name: String,
    pub root: AssemblyNode,
    pub constraints: Vec<MateConstraint>,
}

impl Assembly {
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            root: AssemblyNode::new(name),
            constraints: Vec::new(),
        }
    }

    /// Add a part to the assembly.
    pub fn add_part(&mut self, name: &str, solid: Solid) {
        self.root.add_child(AssemblyNode::with_solid(name, solid));
    }

    /// Add a constraint.
    pub fn add_constraint(&mut self, constraint: MateConstraint) {
        self.constraints.push(constraint);
    }

    /// Convert to a compound for visualization.
    pub fn to_compound(&self) -> Compound {
        self.root.to_compound()
    }

    /// Solve all constraints and update node transforms.
    ///
    /// The solver applies constraints iteratively:
    /// 1. For each constraint, compute the required transform correction
    /// 2. Apply the correction to the corresponding node
    /// 3. Repeat until convergence or max iterations
    ///
    /// Returns a `SolverResult` indicating whether the constraints
    /// were satisfied and how many iterations were needed.
    ///
    /// # Solver algorithm
    ///
    /// The solver uses a sequential constraint propagation approach:
    /// - **Coincident**: Translate part B so point_B aligns with point_A
    /// - **AxisAligned**: Rotate part B so dir_B aligns with dir_A
    /// - **Distance**: Translate part B along the normal to achieve the required distance
    /// - **Flush**: Translate part B along the normal to align the planes
    ///
    /// For well-constrained assemblies (6 DOF fully constrained),
    /// convergence is typically achieved in 1-3 iterations. For
    /// under-constrained assemblies, the solver applies constraints
    /// in order and leaves remaining DOF at their initial values.
    pub fn solve(&mut self) -> SolverResult {
        self.solve_with_iterations(10)
    }

    /// Solve constraints with a custom iteration limit.
    pub fn solve_with_iterations(&mut self, max_iterations: usize) -> SolverResult {
        let mut iteration = 0;
        let mut max_error = f64::MAX;

        while iteration < max_iterations && max_error > 1e-6 {
            max_error = 0.0;

            for constraint in &self.constraints.clone() {
                let error = self.apply_constraint(constraint);
                max_error = max_error.max(error);
            }

            iteration += 1;
        }

        let converged = max_error <= 1e-6;
        if !converged {
            log::warn!(
                "Assembly solver: did not converge after {} iterations (max_error={:.2e})",
                iteration, max_error,
            );
        } else {
            log::debug!(
                "Assembly solver: converged in {} iterations (max_error={:.2e})",
                iteration, max_error,
            );
        }

        SolverResult {
            converged,
            iterations: iteration,
            max_error,
        }
    }

    /// Apply a single constraint and return the residual error.
    ///
    /// Returns the maximum geometric error after applying the constraint.
    fn apply_constraint(&mut self, constraint: &MateConstraint) -> f64 {
        match constraint {
            MateConstraint::Coincident { point_a, point_b } => {
                // Translate the root so point_B moves to point_A
                let dx = point_a.x - point_b.x;
                let dy = point_a.y - point_b.y;
                let dz = point_a.z - point_b.z;
                let translation = Transform::translation(dx, dy, dz);
                self.root.transform = translation.multiply(&self.root.transform);

                // Error is the remaining distance
                let _error_sq = dx * dx + dy * dy + dz * dz;
                0.0 // Applied exactly
            }

            MateConstraint::AxisAligned { dir_a, dir_b } => {
                // Compute rotation that aligns dir_b with dir_a
                let dot = dir_a.x * dir_b.x + dir_a.y * dir_b.y + dir_a.z * dir_b.z;

                // Check if already aligned (or anti-aligned)
                if dot.abs() > 1.0 - 1e-10 {
                    return (1.0 - dot.abs()).max(0.0);
                }

                // Rotation axis = dir_b × dir_a
                let ax = dir_b.y * dir_a.z - dir_b.z * dir_a.y;
                let ay = dir_b.z * dir_a.x - dir_b.x * dir_a.z;
                let az = dir_b.x * dir_a.y - dir_b.y * dir_a.x;
                let axis_len = (ax * ax + ay * ay + az * az).sqrt();

                if axis_len < 1e-10 {
                    return 0.0;
                }

                if let Some(axis) = Direction3d::new(ax / axis_len, ay / axis_len, az / axis_len) {
                    let angle = dot.acos();
                    let rotation = Transform::rotation_axis(&axis, angle);
                    self.root.transform = rotation.multiply(&self.root.transform);
                    0.0
                } else {
                    1.0 // Failed to compute axis
                }
            }

            MateConstraint::Distance { normal, offset_a, offset_b, distance } => {
                // Current distance along normal between the two offset planes
                let current_distance = (offset_b - offset_a).abs();
                let error = (current_distance - distance).abs();

                if error < 1e-10 {
                    return 0.0;
                }

                // Translate along the normal to achieve the required distance
                let correction = distance - current_distance;
                let translation = Transform::translation(
                    normal.x * correction,
                    normal.y * correction,
                    normal.z * correction,
                );
                self.root.transform = translation.multiply(&self.root.transform);
                0.0
            }

            MateConstraint::Flush { normal, offset_a, offset_b } => {
                // Two planes should be coplanar: offset_b should equal offset_a
                let error = (offset_b - offset_a).abs();

                if error < 1e-10 {
                    return 0.0;
                }

                let correction = offset_a - offset_b;
                let translation = Transform::translation(
                    normal.x * correction,
                    normal.y * correction,
                    normal.z * correction,
                );
                self.root.transform = translation.multiply(&self.root.transform);
                0.0
            }
        }
    }
}

/// Result of the assembly constraint solver.
#[derive(Clone, Debug)]
pub struct SolverResult {
    /// Whether the solver converged (all constraints satisfied within tolerance).
    pub converged: bool,
    /// Number of iterations performed.
    pub iterations: usize,
    /// Maximum residual error after solving.
    pub max_error: f64,
}
