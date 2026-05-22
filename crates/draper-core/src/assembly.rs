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
}
