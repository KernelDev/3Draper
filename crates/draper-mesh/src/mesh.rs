//! Mesh data structures.

use draper_geometry::Point3d;
use std::fmt;

/// A 3D triangle mesh.
#[derive(Clone, Debug)]
pub struct TriangleMesh {
    /// Vertex positions.
    pub vertices: Vec<Point3d>,
    /// Triangle indices (3 vertex indices per triangle).
    pub triangles: Vec<[u32; 3]>,
    /// Optional vertex normals.
    pub normals: Option<Vec<[f64; 3]>>,
    /// Optional triangle normals.
    pub face_normals: Option<Vec<[f64; 3]>>,
}

impl TriangleMesh {
    pub fn new() -> Self {
        Self {
            vertices: Vec::new(),
            triangles: Vec::new(),
            normals: None,
            face_normals: None,
        }
    }

    /// Create from vertices and triangle indices.
    pub fn from_data(vertices: Vec<Point3d>, triangles: Vec<[u32; 3]>) -> Self {
        Self {
            vertices,
            triangles,
            normals: None,
            face_normals: None,
        }
    }

    /// Add a vertex and return its index.
    pub fn add_vertex(&mut self, p: Point3d) -> u32 {
        let idx = self.vertices.len() as u32;
        self.vertices.push(p);
        idx
    }

    /// Add a vertex normal. Call after add_vertex with the returned index.
    pub fn add_vertex_normal(&mut self, _idx: u32, normal: [f64; 3]) {
        if self.normals.is_none() {
            self.normals = Some(vec![[0.0, 0.0, 1.0]; self.vertices.len() - 1]);
        }
        if let Some(ref mut normals) = self.normals {
            normals.push(normal);
        }
    }

    /// Add a triangle.
    pub fn add_triangle(&mut self, i: u32, j: u32, k: u32) {
        self.triangles.push([i, j, k]);
    }

    /// Number of vertices.
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    /// Number of triangles.
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    /// Compute face normals.
    pub fn compute_face_normals(&mut self) {
        let mut normals = Vec::with_capacity(self.triangles.len());
        for tri in &self.triangles {
            let v0 = self.vertices[tri[0] as usize];
            let v1 = self.vertices[tri[1] as usize];
            let v2 = self.vertices[tri[2] as usize];

            let e1 = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
            let e2 = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);

            let nx = e1.1 * e2.2 - e1.2 * e2.1;
            let ny = e1.2 * e2.0 - e1.0 * e2.2;
            let nz = e1.0 * e2.1 - e1.1 * e2.0;
            let len = (nx * nx + ny * ny + nz * nz).sqrt();
            if len > 1e-15 {
                normals.push([nx / len, ny / len, nz / len]);
            } else {
                normals.push([0.0, 0.0, 1.0]);
            }
        }
        self.face_normals = Some(normals);
    }

    /// Merge another mesh into this one.
    pub fn merge(&mut self, other: &TriangleMesh) {
        let offset = self.vertices.len() as u32;
        self.vertices.extend(other.vertices.iter().cloned());
        for tri in &other.triangles {
            self.triangles.push([tri[0] + offset, tri[1] + offset, tri[2] + offset]);
        }
        // Merge normals
        match (&mut self.normals, &other.normals) {
            (Some(ref mut self_normals), Some(ref other_normals)) => {
                self_normals.extend(other_normals.iter().cloned());
            }
            (None, Some(ref other_normals)) => {
                // We need to fill in default normals for existing vertices
                let mut combined = vec![[0.0, 0.0, 1.0]; self.vertices.len() - other.vertices.len()];
                combined.extend(other_normals.iter().cloned());
                self.normals = Some(combined);
            }
            _ => {}
        }
    }

    /// Compute bounding box.
    pub fn bounding_box(&self) -> (Point3d, Point3d) {
        if self.vertices.is_empty() {
            return (Point3d::ORIGIN, Point3d::ORIGIN);
        }
        let mut min = self.vertices[0];
        let mut max = self.vertices[0];
        for v in &self.vertices[1..] {
            min.x = min.x.min(v.x);
            min.y = min.y.min(v.y);
            min.z = min.z.min(v.z);
            max.x = max.x.max(v.x);
            max.y = max.y.max(v.y);
            max.z = max.z.max(v.z);
        }
        (min, max)
    }

    /// Total surface area.
    pub fn surface_area(&self) -> f64 {
        let mut area = 0.0;
        for tri in &self.triangles {
            let v0 = self.vertices[tri[0] as usize];
            let v1 = self.vertices[tri[1] as usize];
            let v2 = self.vertices[tri[2] as usize];
            // Cross product of two edges / 2
            let e1x = v1.x - v0.x;
            let e1y = v1.y - v0.y;
            let e1z = v1.z - v0.z;
            let e2x = v2.x - v0.x;
            let e2y = v2.y - v0.y;
            let e2z = v2.z - v0.z;
            let cx = e1y * e2z - e1z * e2y;
            let cy = e1z * e2x - e1x * e2z;
            let cz = e1x * e2y - e1y * e2x;
            area += (cx * cx + cy * cy + cz * cz).sqrt() * 0.5;
        }
        area
    }

    /// Transform all vertices.
    pub fn transform(&mut self, m: &[[f64; 4]; 4]) {
        for v in &mut self.vertices {
            *v = v.transform(m);
        }
    }
}

/// A 2D point for triangulation (in parametric space).
#[derive(Clone, Copy, Debug)]
pub struct Point2dForTriangulation {
    pub x: f64,
    pub y: f64,
    pub original_index: usize,
}

/// Edge constraint for triangulation.
#[derive(Clone, Copy, Debug)]
pub struct ConstraintEdge {
    pub start: usize,
    pub end: usize,
}
