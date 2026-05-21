//! Mesh generation from B-rep topology.
//!
//! Converts topological faces into triangle meshes suitable for rendering.

use crate::earcut;
use crate::triangulate::TriangleMesh;
use draper_geometry::point::{Point2, Point3};
use draper_geometry::surface::Surface;
use draper_topology::entity::*;
use draper_topology::shape::Shape;

/// Generate a triangle mesh from a shape.
pub fn generate_mesh(shape: &Shape, u_samples: usize, v_samples: usize) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    for face in shape.faces() {
        generate_face_mesh(shape, face, &mut mesh, u_samples, v_samples);
    }

    mesh.compute_normals();
    mesh
}

/// Generate mesh for a single face.
fn generate_face_mesh(
    shape: &Shape,
    face: &Face,
    mesh: &mut TriangleMesh,
    u_samples: usize,
    v_samples: usize,
) {
    match &face.surface {
        Some(Surface::Plane(plane)) => {
            generate_planar_face_mesh(shape, face, plane, mesh);
        }
        Some(surface) => {
            generate_parametric_face_mesh(shape, face, surface, mesh, u_samples, v_samples);
        }
        None => {
            // No surface — try to generate from wire vertices
            generate_wire_face_mesh(shape, face, mesh);
        }
    }
}

/// Generate mesh for a planar face using ear clipping.
fn generate_planar_face_mesh(
    shape: &Shape,
    face: &Face,
    plane: &draper_geometry::surface::Plane,
    mesh: &mut TriangleMesh,
) {
    let _normal = plane.axis.axis;
    let origin = plane.axis.location;
    let u_dir = plane.axis.ref_direction;
    let v_dir = plane.axis.y_direction();

    // Get the wire and project vertices to 2D
    if let Some(wire_id) = face.outer_wire {
        if let Some(TopoShape::Wire(wire)) = shape.get(wire_id) {
            let mut points_3d = Vec::new();
            for oriented_edge in &wire.edges {
                if let Some(TopoShape::Edge(edge)) = shape.get(oriented_edge.edge_id) {
                    if oriented_edge.orientation {
                        if let Some(TopoShape::Vertex(v)) = shape.get(edge.start_vertex) {
                            points_3d.push(v.point);
                        }
                    } else {
                        if let Some(TopoShape::Vertex(v)) = shape.get(edge.end_vertex) {
                            points_3d.push(v.point);
                        }
                    }
                }
            }

            if points_3d.len() < 3 {
                return;
            }

            // Project to 2D using the plane's coordinate system
            let points_2d: Vec<Point2> = points_3d
                .iter()
                .map(|p| {
                    let rel = p.to_dvec3() - origin.to_dvec3();
                    Point2::new(rel.dot(u_dir.to_dvec3()), rel.dot(v_dir.to_dvec3()))
                })
                .collect();

            // Triangulate using ear clipping
            let tri_indices = earcut::ear_clip_polygon(&points_2d);

            // Add vertices and triangles to mesh
            let base_idx = mesh.vertices.len() as u32;
            for pt in &points_3d {
                mesh.add_vertex(*pt);
            }

            for idx in tri_indices.chunks(3) {
                if idx.len() == 3 {
                    mesh.add_triangle(base_idx + idx[0], base_idx + idx[1], base_idx + idx[2]);
                }
            }
        }
    }
}

/// Generate mesh for a parametric surface by sampling.
fn generate_parametric_face_mesh(
    _shape: &Shape,
    _face: &Face,
    surface: &Surface,
    mesh: &mut TriangleMesh,
    u_samples: usize,
    v_samples: usize,
) {
    let base_idx = mesh.vertices.len() as u32;

    // Sample the surface on a grid
    for j in 0..=v_samples {
        for i in 0..=u_samples {
            let u = i as f64 / u_samples as f64;
            let v = j as f64 / v_samples as f64;
            let pt = surface.point_at(u, v);
            mesh.add_vertex(pt);
        }
    }

    // Create triangles from the grid
    for j in 0..v_samples {
        for i in 0..u_samples {
            let a = base_idx + (j * (u_samples + 1) + i) as u32;
            let b = base_idx + (j * (u_samples + 1) + i + 1) as u32;
            let c = base_idx + ((j + 1) * (u_samples + 1) + i + 1) as u32;
            let d = base_idx + ((j + 1) * (u_samples + 1) + i) as u32;

            mesh.add_triangle(a, b, c);
            mesh.add_triangle(a, c, d);
        }
    }
}

/// Generate mesh from wire vertices (fallback for faces without surface geometry).
fn generate_wire_face_mesh(shape: &Shape, face: &Face, mesh: &mut TriangleMesh) {
    if let Some(wire_id) = face.outer_wire {
        if let Some(TopoShape::Wire(wire)) = shape.get(wire_id) {
            let mut points = Vec::new();
            for oriented_edge in &wire.edges {
                if let Some(TopoShape::Edge(edge)) = shape.get(oriented_edge.edge_id) {
                    let v_id = if oriented_edge.orientation {
                        edge.start_vertex
                    } else {
                        edge.end_vertex
                    };
                    if let Some(TopoShape::Vertex(v)) = shape.get(v_id) {
                        points.push(v.point);
                    }
                }
            }

            if points.len() < 3 {
                return;
            }

            // Simple fan triangulation from the centroid
            let centroid = {
                let sum = points.iter().fold(glam::DVec3::ZERO, |acc, p| acc + p.to_dvec3());
                Point3::from_dvec3(sum / points.len() as f64)
            };

            let center_idx = mesh.add_vertex(centroid);
            let base_idx = mesh.vertices.len() as u32;

            for pt in &points {
                mesh.add_vertex(*pt);
            }

            for i in 0..points.len() {
                let j = (i + 1) % points.len();
                mesh.add_triangle(center_idx, base_idx + i as u32, base_idx + j as u32);
            }
        }
    }
}
