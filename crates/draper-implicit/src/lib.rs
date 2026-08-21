// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Implicit solid modeling via Signed Distance Fields (SDF).
//!
//! Per ROADMAP_VISION_2036.md §5.1: ImplicitSolid is a first-class citizen
//! alongside BrepSolid. This module provides:
//!
//! - **CSG trees**: lazy evaluation of union, subtract, intersect over SDF fields
//! - **Primitive SDFs**: sphere, box, cylinder, torus, cone
//! - **Batch evaluation**: mass SDF evaluation on a 3D grid (GPU-ready)
//! - **Dual Contouring**: adaptive mesh extraction with sharp feature preservation
//!
//! Design principles (Directive 5 — GPU-First):
//! - SOA data layout for batch evaluation
//! - No pointers in CSG tree nodes (use enum + Box)
//! - f32 precision for GPU compatibility

pub mod generative;
pub mod dual_contouring;

pub use dual_contouring::*;

use draper_geometry::Point3d;

// ============================================================
// SDF trait and primitive shapes
// ============================================================

/// A signed distance field — maps 3D points to signed distance.
///
/// Negative = inside the solid, positive = outside, zero = on the surface.
/// The gradient of the SDF gives the surface normal at any point.
pub trait Sdf: Send + Sync {
    /// Evaluate the signed distance at point `p`.
    fn signed_distance(&self, p: &Point3d) -> f64;

    /// Evaluate the gradient (surface normal direction) at point `p`.
    /// For exact SDFs, the gradient has unit length and points outward.
    /// Default implementation uses finite differences.
    fn gradient(&self, p: &Point3d) -> [f64; 3] {
        let eps = 1e-6;
        let dx = self.signed_distance(&Point3d::new(p.x + eps, p.y, p.z))
            - self.signed_distance(&Point3d::new(p.x - eps, p.y, p.z));
        let dy = self.signed_distance(&Point3d::new(p.x, p.y + eps, p.z))
            - self.signed_distance(&Point3d::new(p.x, p.y - eps, p.z));
        let dz = self.signed_distance(&Point3d::new(p.x, p.y, p.z + eps))
            - self.signed_distance(&Point3d::new(p.x, p.y, p.z - eps));
        let len = (dx * dx + dy * dy + dz * dz).sqrt();
        if len > 1e-15 {
            [dx / len, dy / len, dz / len]
        } else {
            [0.0, 0.0, 1.0]
        }
    }

    /// Bounding box of the solid region (where SDF < 0).
    /// Used for grid-based mesh extraction.
    fn bounding_box(&self) -> (Point3d, Point3d);
}

// ============================================================
// Primitive SDFs
// ============================================================

/// Sphere SDF: distance = |p - center| - radius
pub struct SphereSdf {
    pub center: Point3d,
    pub radius: f64,
}

impl Sdf for SphereSdf {
    fn signed_distance(&self, p: &Point3d) -> f64 {
        let dx = p.x - self.center.x;
        let dy = p.y - self.center.y;
        let dz = p.z - self.center.z;
        (dx * dx + dy * dy + dz * dz).sqrt() - self.radius
    }

    fn gradient(&self, p: &Point3d) -> [f64; 3] {
        let dx = p.x - self.center.x;
        let dy = p.y - self.center.y;
        let dz = p.z - self.center.z;
        let len = (dx * dx + dy * dy + dz * dz).sqrt();
        if len > 1e-15 {
            [dx / len, dy / len, dz / len]
        } else {
            [0.0, 1.0, 0.0]
        }
    }

    fn bounding_box(&self) -> (Point3d, Point3d) {
        (
            Point3d::new(self.center.x - self.radius, self.center.y - self.radius, self.center.z - self.radius),
            Point3d::new(self.center.x + self.radius, self.center.y + self.radius, self.center.z + self.radius),
        )
    }
}

/// Box (axis-aligned) SDF using the exact distance formula.
pub struct BoxSdf {
    pub center: Point3d,
    pub half_size: [f64; 3],
}

impl Sdf for BoxSdf {
    fn signed_distance(&self, p: &Point3d) -> f64 {
        let dx = (p.x - self.center.x).abs() - self.half_size[0];
        let dy = (p.y - self.center.y).abs() - self.half_size[1];
        let dz = (p.z - self.center.z).abs() - self.half_size[2];
        let outside = ((dx.max(0.0)).powi(2) + (dy.max(0.0)).powi(2) + (dz.max(0.0)).powi(2)).sqrt();
        let inside = dx.max(dy).max(dz).min(0.0);
        outside + inside
    }

    fn bounding_box(&self) -> (Point3d, Point3d) {
        (
            Point3d::new(self.center.x - self.half_size[0], self.center.y - self.half_size[1], self.center.z - self.half_size[2]),
            Point3d::new(self.center.x + self.half_size[0], self.center.y + self.half_size[1], self.center.z + self.half_size[2]),
        )
    }
}

/// Cylinder SDF (along Z axis).
pub struct CylinderSdf {
    pub center: Point3d,
    pub radius: f64,
    pub height: f64,
}

impl Sdf for CylinderSdf {
    fn signed_distance(&self, p: &Point3d) -> f64 {
        let dx = p.x - self.center.x;
        let dy = p.y - self.center.y;
        let dz = p.z - self.center.z;
        let radial = (dx * dx + dy * dy).sqrt() - self.radius;
        let axial = dz.abs() - self.height * 0.5;
        let outside = ((radial.max(0.0)).powi(2) + (axial.max(0.0)).powi(2)).sqrt();
        let inside = radial.max(axial).min(0.0);
        outside + inside
    }

    fn bounding_box(&self) -> (Point3d, Point3d) {
        (
            Point3d::new(self.center.x - self.radius, self.center.y - self.radius, self.center.z - self.height * 0.5),
            Point3d::new(self.center.x + self.radius, self.center.y + self.radius, self.center.z + self.height * 0.5),
        )
    }
}

/// Gyroid SDF — a triply-periodic minimal surface (TPMS).
///
/// Per Vision 2030 Task 3: SdfGyroid for lattice structures.
///
/// The gyroid surface is defined by the implicit equation:
///   sin(x)cos(y) + sin(y)cos(z) + sin(z)cos(x) = 0
///
/// The SDF approximates the distance to this surface. The gyroid creates
/// a bicontinuous lattice structure — useful for:
/// - Lightweighting (aerospace, automotive)
/// - Heat exchangers (maximizing surface area)
/// - Bone scaffolds (biomedical)
/// - Metamaterials (tunable mechanical properties)
///
/// The `scale` parameter controls the period of the lattice.
/// The `thickness` parameter controls how much solid material surrounds
/// the gyroid surface (0 = infinitely thin surface, positive = thick walls).
pub struct GyroidSdf {
    /// Scaling factor — controls lattice period.
    /// At scale=1.0, the period is 2π in each direction.
    pub scale: f64,
    /// Wall thickness — how far inside the gyroid surface is "solid".
    /// 0.0 = infinitely thin surface (no volume).
    /// Positive = solid region within `thickness` of the surface.
    pub thickness: f64,
    /// Bounding box half-size (the gyroid is clipped to this region).
    pub half_size: [f64; 3],
}

impl GyroidSdf {
    pub fn new(scale: f64, thickness: f64, half_size: [f64; 3]) -> Self {
        Self { scale, thickness, half_size }
    }
}

impl Sdf for GyroidSdf {
    fn signed_distance(&self, p: &Point3d) -> f64 {
        let s = self.scale;
        let x = p.x * s;
        let y = p.y * s;
        let z = p.z * s;

        // Gyroid implicit function
        let g = x.sin() * y.cos() + y.sin() * z.cos() + z.sin() * x.cos();

        // The gyroid surface is at g=0. The signed distance is approximately
        // g / |∇g|. For the gyroid, |∇g| varies but is roughly constant (~1.5).
        // We use a simplified distance: g / 1.5 (approximate gradient magnitude).
        let gradient_mag = 1.5; // Approximate average |∇g|
        let dist_to_surface = g / gradient_mag;

        // Offset by thickness: positive thickness = solid within `thickness` of surface
        let dist = dist_to_surface.abs() - self.thickness;

        // Also clip to bounding box
        let bx = (p.x.abs() - self.half_size[0]).max(0.0);
        let by = (p.y.abs() - self.half_size[1]).max(0.0);
        let bz = (p.z.abs() - self.half_size[2]).max(0.0);
        let box_dist = (bx * bx + by * by + bz * bz).sqrt();

        // Intersection: inside gyroid AND inside box
        // Use max() for intersection
        let signed_box = -(bx.max(by).max(bz)); // Negative inside box
        dist.max(signed_box)
    }

    fn bounding_box(&self) -> (Point3d, Point3d) {
        (
            Point3d::new(-self.half_size[0], -self.half_size[1], -self.half_size[2]),
            Point3d::new(self.half_size[0], self.half_size[1], self.half_size[2]),
        )
    }
}

// ============================================================
// CSG operations (lazy evaluation)
// ============================================================

/// CSG node — represents a lazy SDF expression tree.
///
/// Per ROADMAP_VISION_2036.md §5.2: CSG trees use lazy evaluation
/// so boolean operations are only computed when the SDF is queried.
pub enum CsgNode {
    /// A leaf node containing a primitive SDF.
    Leaf(Box<dyn Sdf>),
    /// Union: min(A, B) — points inside either solid are inside.
    Union(Box<CsgNode>, Box<CsgNode>),
    /// Subtraction: max(A, -B) — points inside A but outside B.
    Subtract(Box<CsgNode>, Box<CsgNode>),
    /// Intersection: max(A, B) — points inside both solids.
    Intersect(Box<CsgNode>, Box<CsgNode>),
}

impl CsgNode {
    /// Create a leaf node from any Sdf.
    pub fn leaf(sdf: impl Sdf + 'static) -> Self {
        CsgNode::Leaf(Box::new(sdf))
    }

    /// Union of two CSG nodes.
    pub fn union(a: CsgNode, b: CsgNode) -> Self {
        CsgNode::Union(Box::new(a), Box::new(b))
    }

    /// Subtract b from a.
    pub fn subtract(a: CsgNode, b: CsgNode) -> Self {
        CsgNode::Subtract(Box::new(a), Box::new(b))
    }

    /// Intersection of two CSG nodes.
    pub fn intersect(a: CsgNode, b: CsgNode) -> Self {
        CsgNode::Intersect(Box::new(a), Box::new(b))
    }
}

impl Sdf for CsgNode {
    fn signed_distance(&self, p: &Point3d) -> f64 {
        match self {
            CsgNode::Leaf(sdf) => sdf.signed_distance(p),
            CsgNode::Union(a, b) => a.signed_distance(p).min(b.signed_distance(p)),
            CsgNode::Subtract(a, b) => a.signed_distance(p).max(-b.signed_distance(p)),
            CsgNode::Intersect(a, b) => a.signed_distance(p).max(b.signed_distance(p)),
        }
    }

    fn bounding_box(&self) -> (Point3d, Point3d) {
        match self {
            CsgNode::Leaf(sdf) => sdf.bounding_box(),
            CsgNode::Union(a, b) => {
                let (amin, amax) = a.bounding_box();
                let (bmin, bmax) = b.bounding_box();
                (
                    Point3d::new(amin.x.min(bmin.x), amin.y.min(bmin.y), amin.z.min(bmin.z)),
                    Point3d::new(amax.x.max(bmax.x), amax.y.max(bmax.y), amax.z.max(bmax.z)),
                )
            }
            CsgNode::Subtract(a, _b) => a.bounding_box(),
            CsgNode::Intersect(a, b) => {
                let (amin, amax) = a.bounding_box();
                let (bmin, bmax) = b.bounding_box();
                (
                    Point3d::new(amin.x.max(bmin.x), amin.y.max(bmin.y), amin.z.max(bmin.z)),
                    Point3d::new(amax.x.min(bmax.x), amax.y.min(bmax.y), amax.z.min(bmax.z)),
                )
            }
        }
    }
}

// ============================================================
// ImplicitSolid — first-class solid type
// ============================================================

/// An implicit solid defined by a CSG tree of SDFs.
///
/// Per ROADMAP_VISION_2036.md §5.1: ImplicitSolid sits alongside BrepSolid
/// as a first-class citizen. It can be converted to/from B-Rep via:
/// - B-Rep → SDF: 3D voxelization
/// - SDF → B-Rep: feature recognition + NURBS fitting
pub struct ImplicitSolid {
    pub csg: CsgNode,
}

impl ImplicitSolid {
    /// Create from any CSG tree.
    pub fn new(csg: CsgNode) -> Self {
        Self { csg }
    }

    /// Create a sphere solid.
    pub fn sphere(center: Point3d, radius: f64) -> Self {
        Self::new(CsgNode::leaf(SphereSdf { center, radius }))
    }

    /// Create a box solid.
    pub fn box_solid(center: Point3d, half_x: f64, half_y: f64, half_z: f64) -> Self {
        Self::new(CsgNode::leaf(BoxSdf { center, half_size: [half_x, half_y, half_z] }))
    }

    /// Create a cylinder solid.
    pub fn cylinder(center: Point3d, radius: f64, height: f64) -> Self {
        Self::new(CsgNode::leaf(CylinderSdf { center, radius, height }))
    }

    /// Union with another implicit solid.
    pub fn union(self, other: ImplicitSolid) -> Self {
        Self::new(CsgNode::union(self.csg, other.csg))
    }

    /// Subtract another implicit solid.
    pub fn subtract(self, other: ImplicitSolid) -> Self {
        Self::new(CsgNode::subtract(self.csg, other.csg))
    }

    /// Intersect with another implicit solid.
    pub fn intersect(self, other: ImplicitSolid) -> Self {
        Self::new(CsgNode::intersect(self.csg, other.csg))
    }

    /// Evaluate SDF at a point.
    pub fn signed_distance(&self, p: &Point3d) -> f64 {
        self.csg.signed_distance(p)
    }

    /// Get bounding box.
    pub fn bounding_box(&self) -> (Point3d, Point3d) {
        self.csg.bounding_box()
    }

    /// Batch evaluate SDF on a regular 3D grid (GPU-ready).
    ///
    /// Returns flat f32 arrays (SOA layout) for direct GPU upload.
    /// Grid resolution: nx × ny × nz points.
    pub fn evaluate_grid(
        &self,
        nx: usize, ny: usize, nz: usize,
    ) -> GridSdfResult {
        let (bmin, bmax) = self.bounding_box();
        let dx = if nx > 1 { (bmax.x - bmin.x) / (nx - 1) as f64 } else { 0.0 };
        let dy = if ny > 1 { (bmax.y - bmin.y) / (ny - 1) as f64 } else { 0.0 };
        let dz = if nz > 1 { (bmax.z - bmin.z) / (nz - 1) as f64 } else { 0.0 };

        let total = nx * ny * nz;
        let mut distances = vec![0.0f32; total];
        let mut grad_x = vec![0.0f32; total];
        let mut grad_y = vec![0.0f32; total];
        let mut grad_z = vec![0.0f32; total];

        for k in 0..nz {
            for j in 0..ny {
                for i in 0..nx {
                    let idx = k * nx * ny + j * nx + i;
                    let p = Point3d::new(
                        bmin.x + i as f64 * dx,
                        bmin.y + j as f64 * dy,
                        bmin.z + k as f64 * dz,
                    );
                    distances[idx] = self.csg.signed_distance(&p) as f32;
                    let g = self.csg.gradient(&p);
                    grad_x[idx] = g[0] as f32;
                    grad_y[idx] = g[1] as f32;
                    grad_z[idx] = g[2] as f32;
                }
            }
        }

        GridSdfResult {
            distances,
            grad_x,
            grad_y,
            grad_z,
            origin: [bmin.x as f32, bmin.y as f32, bmin.z as f32],
            spacing: [dx as f32, dy as f32, dz as f32],
            dims: [nx as u32, ny as u32, nz as u32],
        }
    }
}

/// Result of grid-based SDF evaluation (SOA layout for GPU).
#[derive(Clone, Debug)]
pub struct GridSdfResult {
    /// Signed distance at each grid point (flat: idx = z*nx*ny + y*nx + x)
    pub distances: Vec<f32>,
    /// Gradient X component (surface normal direction)
    pub grad_x: Vec<f32>,
    /// Gradient Y component
    pub grad_y: Vec<f32>,
    /// Gradient Z component
    pub grad_z: Vec<f32>,
    /// Grid origin (min corner)
    pub origin: [f32; 3],
    /// Grid spacing (cell size)
    pub spacing: [f32; 3],
    /// Grid dimensions [nx, ny, nz]
    pub dims: [u32; 3],
}

impl GridSdfResult {
    /// Total number of grid points.
    pub fn total_points(&self) -> usize {
        self.dims[0] as usize * self.dims[1] as usize * self.dims[2] as usize
    }

    /// Get distance at grid index (i, j, k).
    pub fn distance_at(&self, i: usize, j: usize, k: usize) -> f32 {
        let idx = k * (self.dims[0] as usize * self.dims[1] as usize) + j * (self.dims[0] as usize) + i;
        self.distances.get(idx).copied().unwrap_or(f32::MAX)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sphere_sdf() {
        let sphere = SphereSdf { center: Point3d::ORIGIN, radius: 1.0 };
        assert!((sphere.signed_distance(&Point3d::new(2.0, 0.0, 0.0)) - 1.0).abs() < 1e-10);
        assert!((sphere.signed_distance(&Point3d::new(0.0, 0.0, 0.0)) + 1.0).abs() < 1e-10);
        assert!(sphere.signed_distance(&Point3d::new(1.0, 0.0, 0.0)).abs() < 1e-10);
    }

    #[test]
    fn test_box_sdf() {
        let box_sdf = BoxSdf { center: Point3d::ORIGIN, half_size: [1.0, 1.0, 1.0] };
        assert!((box_sdf.signed_distance(&Point3d::new(0.0, 0.0, 0.0)) + 1.0).abs() < 1e-10);
        assert!((box_sdf.signed_distance(&Point3d::new(2.0, 0.0, 0.0)) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_csg_union() {
        let a = ImplicitSolid::sphere(Point3d::new(-0.5, 0.0, 0.0), 1.0);
        let b = ImplicitSolid::sphere(Point3d::new(0.5, 0.0, 0.0), 1.0);
        let union = a.union(b);
        // At origin, both spheres are inside (dist < 0), union = min → most negative
        let dist = union.signed_distance(&Point3d::new(0.0, 0.0, 0.0));
        assert!(dist < 0.0, "Origin should be inside union: dist={}", dist);
    }

    #[test]
    fn test_csg_subtract() {
        let a = ImplicitSolid::box_solid(Point3d::ORIGIN, 1.0, 1.0, 1.0);
        let b = ImplicitSolid::sphere(Point3d::ORIGIN, 0.5);
        let result = a.subtract(b);
        // At center, the sphere is subtracted from the box
        let dist = result.signed_distance(&Point3d::new(0.0, 0.0, 0.0));
        assert!(dist > 0.0, "Center should be outside after subtraction: dist={}", dist);
    }

    #[test]
    fn test_grid_evaluation() {
        let sphere = ImplicitSolid::sphere(Point3d::ORIGIN, 1.0);
        let grid = sphere.evaluate_grid(5, 5, 5);
        assert_eq!(grid.total_points(), 125);
        assert_eq!(grid.distances.len(), 125);
        // Center point (2,2,2) should be inside (negative distance)
        let center = grid.distance_at(2, 2, 2);
        assert!(center < 0.0, "Center should be inside: dist={}", center);
    }

    #[test]
    fn test_bounding_box() {
        let sphere = ImplicitSolid::sphere(Point3d::new(5.0, 0.0, 0.0), 2.0);
        let (bmin, bmax) = sphere.bounding_box();
        assert!((bmin.x - 3.0).abs() < 1e-10);
        assert!((bmax.x - 7.0).abs() < 1e-10);
    }
}
