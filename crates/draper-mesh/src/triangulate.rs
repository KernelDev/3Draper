//! Triangle mesh data structure.

use draper_geometry::point::Point3;
use serde::{Deserialize, Serialize};

/// A triangle mesh with vertices and indices.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TriangleMesh {
    /// 3D vertex positions.
    pub vertices: Vec<Point3>,
    /// Normal vectors per vertex (optional).
    pub normals: Vec<[f32; 3]>,
    /// Triangle indices (3 indices per triangle).
    pub indices: Vec<u32>,
    /// UV coordinates per vertex (optional).
    pub uvs: Vec<[f32; 2]>,
    /// Per-face colors (optional).
    pub face_colors: Vec<[f32; 3]>,
}

impl TriangleMesh {
    pub fn new() -> Self {
        Self {
            vertices: Vec::new(),
            normals: Vec::new(),
            indices: Vec::new(),
            uvs: Vec::new(),
            face_colors: Vec::new(),
        }
    }

    /// Add a vertex and return its index.
    pub fn add_vertex(&mut self, point: Point3) -> u32 {
        let idx = self.vertices.len() as u32;
        self.vertices.push(point);
        idx
    }

    /// Add a triangle from vertex indices.
    pub fn add_triangle(&mut self, a: u32, b: u32, c: u32) {
        self.indices.push(a);
        self.indices.push(b);
        self.indices.push(c);
    }

    /// Compute per-vertex normals by averaging face normals.
    pub fn compute_normals(&mut self) {
        let mut normal_accum = vec![glam::Vec3::ZERO; self.vertices.len()];

        for tri in self.indices.chunks(3) {
            let a = self.vertices[tri[0] as usize].to_dvec3();
            let b = self.vertices[tri[1] as usize].to_dvec3();
            let c = self.vertices[tri[2] as usize].to_dvec3();

            let ab = b - a;
            let ac = c - a;
            let face_normal = ab.cross(ac);

            normal_accum[tri[0] as usize] += glam::Vec3::new(
                face_normal.x as f32,
                face_normal.y as f32,
                face_normal.z as f32,
            );
            normal_accum[tri[1] as usize] += glam::Vec3::new(
                face_normal.x as f32,
                face_normal.y as f32,
                face_normal.z as f32,
            );
            normal_accum[tri[2] as usize] += glam::Vec3::new(
                face_normal.x as f32,
                face_normal.y as f32,
                face_normal.z as f32,
            );
        }

        self.normals = normal_accum
            .iter()
            .map(|n| {
                let len = n.length();
                if len > 1e-10 {
                    let n = n / len;
                    [n.x, n.y, n.z]
                } else {
                    [0.0, 0.0, 1.0]
                }
            })
            .collect();
    }

    pub fn triangle_count(&self) -> usize {
        self.indices.len() / 3
    }

    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    /// Compute the bounding box of this mesh.
    pub fn bounding_box(&self) -> draper_geometry::point::BoundingBox3 {
        let mut bb = draper_geometry::point::BoundingBox3::empty();
        for v in &self.vertices {
            bb.extend(*v);
        }
        bb
    }
}

impl Default for TriangleMesh {
    fn default() -> Self {
        Self::new()
    }
}
