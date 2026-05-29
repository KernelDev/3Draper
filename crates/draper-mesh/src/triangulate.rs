//! Face triangulation — converts B-Rep faces to triangle meshes.
//!
//! Uses ear-clipping for polygons and parametric sampling for curved surfaces.
//! Supports boundary-aware trimming when boundary points are available.

use crate::mesh::TriangleMesh;
use draper_geometry::{
    Point3d, Point2d, Direction3d, Vec3d,
    Surface, Plane, CylinderSurface, SphereSurface, TorusSurface,
    ConeSurface, Curve3d,
};
use draper_topology::{Face, Wire, CoEdge, Edge, Solid, Shell, Compound};
use std::f64::consts::PI;
use std::collections::HashMap;

/// Triangulation parameters.
#[derive(Clone, Debug)]
pub struct TriangulationParams {
    /// Maximum edge length in the triangulation.
    pub max_edge_length: f64,
    /// Maximum deviation from the true surface.
    pub max_deviation: f64,
    /// Number of angular samples for cylindrical/spherical surfaces.
    pub angular_samples: usize,
    /// Number of height samples for cylindrical surfaces.
    pub height_samples: usize,
}

impl Default for TriangulationParams {
    fn default() -> Self {
        Self {
            max_edge_length: 1.0,
            max_deviation: 0.01,
            angular_samples: 48,
            height_samples: 8,
        }
    }
}

/// Triangulate a solid into a triangle mesh.
pub fn triangulate_solid(solid: &Solid, params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    for face in solid.faces() {
        let face_mesh = triangulate_face(face, params);
        mesh.merge(&face_mesh);
    }
    mesh
}

/// Triangulate a shell into a triangle mesh.
pub fn triangulate_shell(shell: &Shell, params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    for face in &shell.faces {
        let face_mesh = triangulate_face(face, params);
        mesh.merge(&face_mesh);
    }
    mesh
}

/// Triangulate a compound into a triangle mesh.
pub fn triangulate_compound(compound: &Compound, params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    for solid in &compound.solids {
        mesh.merge(&triangulate_solid(solid, params));
    }
    for sub in &compound.compounds {
        mesh.merge(&triangulate_compound(sub, params));
    }
    mesh
}

/// Triangulate a single face.
pub fn triangulate_face(face: &Face, params: &TriangulationParams) -> TriangleMesh {
    if let Some(ref surface) = face.surface {
        match surface {
            Surface::Plane(plane) => triangulate_planar_face(face, plane, params),
            Surface::Cylinder(cyl) => triangulate_cylinder_face(face, cyl, params),
            Surface::Sphere(sphere) => triangulate_sphere_face(face, sphere, params),
            Surface::Torus(torus) => triangulate_torus_face(face, torus, params),
            Surface::Cone(cone) => triangulate_cone_face(face, cone, params),
            Surface::Revolution(rev) => triangulate_revolution_face(face, rev, params),
            Surface::Extrusion(ext) => triangulate_extrusion_face(face, ext, params),
            Surface::Nurbs(_) => {
                // Fallback: sample the surface on a grid
                triangulate_generic_surface(face, surface, params)
            }
        }
    } else {
        TriangleMesh::new()
    }
}

/// Triangulate a planar face.
/// Uses simple fan triangulation for convex boundaries (no centroid vertex added)
/// and ear clipping for non-convex.
fn triangulate_planar_face(face: &Face, plane: &Plane, _params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    // Collect boundary points
    let points_3d = collect_face_boundary_points(face);

    if points_3d.is_empty() {
        // No boundary — skip
        return mesh;
    }

    // Use the same logic as triangulate_plane_with_boundary
    let forward = face.forward;

    // Project 3D points onto the plane's 2D coordinate system
    let points_2d: Vec<Point2d> = points_3d.iter().map(|p| {
        let dx = p.x - plane.origin.x;
        let dy = p.y - plane.origin.y;
        let dz = p.z - plane.origin.z;
        Point2d::new(
            dx * plane.u_dir.x + dy * plane.u_dir.y + dz * plane.u_dir.z,
            dx * plane.v_dir.x + dy * plane.v_dir.y + dz * plane.v_dir.z,
        )
    }).collect();

    let is_convex = is_convex_polygon(&points_2d);

    if is_convex && points_3d.len() >= 3 {
        // Simple fan triangulation — uses boundary vertex 0 as the fan center.
        // This produces N-2 triangles for N boundary vertices (minimum for convex polygon).
        for p in &points_3d {
            mesh.add_vertex(*p);
        }
        let n = points_3d.len() as u32;
        for i in 1..n-1 {
            let v0 = 0u32;       // fan center = first boundary vertex
            let v1 = i;          // boundary[i]
            let v2 = i + 1;      // boundary[i+1]
            if forward {
                mesh.add_triangle(v0, v1, v2);
            } else {
                mesh.add_triangle(v0, v2, v1);
            }
        }
    } else {
        // Ear clipping for non-convex polygons
        let triangles = ear_clip(&points_2d);
        for p in &points_3d {
            mesh.add_vertex(*p);
        }
        for tri in &triangles {
            if forward {
                mesh.add_triangle(tri[0], tri[1], tri[2]);
            } else {
                mesh.add_triangle(tri[0], tri[2], tri[1]);
            }
        }
    }

    mesh
}

/// Triangulate a cylinder face.
fn triangulate_cylinder_face(face: &Face, cyl: &CylinderSurface, params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let n_u = params.angular_samples;
    let n_v = params.height_samples.max(2);

    // Determine v range from face boundary or default
    let (v_min, v_max) = estimate_v_range(face).unwrap_or((0.0, 1.0));

    for j in 0..n_v {
        for i in 0..n_u {
            let u = 2.0 * PI * i as f64 / n_u as f64;
            let v = v_min + (v_max - v_min) * j as f64 / (n_v - 1) as f64;
            let p = cyl.point_at(u, v);
            let n = cyl.normal_at(u, v);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
        }
    }

    for j in 0..n_v - 1 {
        for i in 0..n_u {
            let i_next = (i + 1) % n_u;
            let v0 = (j * n_u + i) as u32;
            let v1 = (j * n_u + i_next) as u32;
            let v2 = ((j + 1) * n_u + i_next) as u32;
            let v3 = ((j + 1) * n_u + i) as u32;

            if face.forward {
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            } else {
                mesh.add_triangle(v0, v2, v1);
                mesh.add_triangle(v0, v3, v2);
            }
        }
    }

    mesh
}

/// Triangulate a sphere face.
fn triangulate_sphere_face(face: &Face, sphere: &SphereSurface, params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let n_u = params.angular_samples;
    let n_v = params.angular_samples / 2;

    for j in 0..=n_v {
        for i in 0..n_u {
            let u = 2.0 * PI * i as f64 / n_u as f64;
            let v = PI * j as f64 / n_v as f64;
            let p = sphere.point_at(u, v);
            let n = sphere.normal_at(u, v);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
        }
    }

    for j in 0..n_v {
        for i in 0..n_u {
            let i_next = (i + 1) % n_u;
            let v0 = (j * n_u + i) as u32;
            let v1 = (j * n_u + i_next) as u32;
            let v2 = ((j + 1) * n_u + i_next) as u32;
            let v3 = ((j + 1) * n_u + i) as u32;

            if j == 0 {
                // Top cap — degenerate triangles
                if face.forward {
                    mesh.add_triangle(v0, v2, v3);
                } else {
                    mesh.add_triangle(v0, v3, v2);
                }
            } else if j == n_v - 1 {
                // Bottom cap
                if face.forward {
                    mesh.add_triangle(v0, v1, v2);
                } else {
                    mesh.add_triangle(v0, v2, v1);
                }
            } else {
                if face.forward {
                    mesh.add_triangle(v0, v1, v2);
                    mesh.add_triangle(v0, v2, v3);
                } else {
                    mesh.add_triangle(v0, v2, v1);
                    mesh.add_triangle(v0, v3, v2);
                }
            }
        }
    }

    mesh
}

/// Triangulate a torus face.
fn triangulate_torus_face(face: &Face, torus: &TorusSurface, params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let n_u = params.angular_samples;
    let n_v = params.angular_samples;

    for j in 0..n_v {
        for i in 0..n_u {
            let u = 2.0 * PI * i as f64 / n_u as f64;
            let v = 2.0 * PI * j as f64 / n_v as f64;
            let p = torus.point_at(u, v);
            let n = torus.normal_at(u, v);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
        }
    }

    for j in 0..n_v {
        for i in 0..n_u {
            let i_next = (i + 1) % n_u;
            let j_next = (j + 1) % n_v;
            let v0 = (j * n_u + i) as u32;
            let v1 = (j * n_u + i_next) as u32;
            let v2 = (j_next * n_u + i_next) as u32;
            let v3 = (j_next * n_u + i) as u32;

            if face.forward {
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            } else {
                mesh.add_triangle(v0, v2, v1);
                mesh.add_triangle(v0, v3, v2);
            }
        }
    }

    mesh
}

/// Triangulate a cone face.
fn triangulate_cone_face(face: &Face, cone: &draper_geometry::ConeSurface, params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let n_u = params.angular_samples;
    let n_v = params.height_samples.max(2);

    let (v_min, v_max) = estimate_v_range(face).unwrap_or((0.0, 1.0));

    for j in 0..n_v {
        for i in 0..n_u {
            let u = 2.0 * PI * i as f64 / n_u as f64;
            let v = v_min + (v_max - v_min) * j as f64 / (n_v - 1) as f64;
            let p = cone.point_at(u, v);
            let n = cone.normal_at(u, v);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
        }
    }

    for j in 0..n_v - 1 {
        for i in 0..n_u {
            let i_next = (i + 1) % n_u;
            let v0 = (j * n_u + i) as u32;
            let v1 = (j * n_u + i_next) as u32;
            let v2 = ((j + 1) * n_u + i_next) as u32;
            let v3 = ((j + 1) * n_u + i) as u32;

            if face.forward {
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            } else {
                mesh.add_triangle(v0, v2, v1);
                mesh.add_triangle(v0, v3, v2);
            }
        }
    }

    mesh
}

/// Triangulate a revolution surface face.
fn triangulate_revolution_face(face: &Face, rev: &draper_geometry::RevolutionSurface, params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let n_u = params.angular_samples;
    let n_v = params.angular_samples;

    let (v_min, v_max) = rev.profile.param_range();

    for j in 0..n_v {
        for i in 0..n_u {
            let u = 2.0 * PI * i as f64 / n_u as f64;
            let v = v_min + (v_max - v_min) * j as f64 / (n_v - 1) as f64;
            let p = rev.point_at(u, v);
            // Use numerical normal for revolution surfaces
            let n = face.surface.as_ref().map(|s| s.normal_at(u, v)).unwrap_or(Direction3d::Z);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
        }
    }

    for j in 0..n_v - 1 {
        for i in 0..n_u {
            let i_next = (i + 1) % n_u;
            let v0 = (j * n_u + i) as u32;
            let v1 = (j * n_u + i_next) as u32;
            let v2 = ((j + 1) * n_u + i_next) as u32;
            let v3 = ((j + 1) * n_u + i) as u32;

            if face.forward {
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            } else {
                mesh.add_triangle(v0, v2, v1);
                mesh.add_triangle(v0, v3, v2);
            }
        }
    }

    mesh
}

/// Triangulate an extrusion surface face.
fn triangulate_extrusion_face(face: &Face, ext: &draper_geometry::ExtrusionSurface, params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let n_u = params.angular_samples;
    let n_v = params.height_samples.max(2);

    // Compute v range from boundary points
    let (v_min, v_max) = compute_extrusion_v_range(face, ext);

    // Compute u range from the profile curve's param range
    let (u_min, u_max) = ext.profile.param_range();

    for j in 0..n_v {
        for i in 0..n_u {
            let u = u_min + (u_max - u_min) * i as f64 / (n_u - 1).max(1) as f64;
            let v = v_min + (v_max - v_min) * j as f64 / (n_v - 1).max(1) as f64;
            let p = ext.point_at(u, v);
            mesh.add_vertex(p);
        }
    }

    for j in 0..n_v - 1 {
        for i in 0..n_u - 1 {
            let v0 = (j * n_u + i) as u32;
            let v1 = (j * n_u + i + 1) as u32;
            let v2 = ((j + 1) * n_u + i + 1) as u32;
            let v3 = ((j + 1) * n_u + i) as u32;

            if face.forward {
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            } else {
                mesh.add_triangle(v0, v2, v1);
                mesh.add_triangle(v0, v3, v2);
            }
        }
    }

    mesh
}

/// Compute the v (extrusion) parameter range for an extrusion surface
/// by projecting boundary points onto the extrusion direction.
fn compute_extrusion_v_range(face: &Face, ext: &draper_geometry::ExtrusionSurface) -> (f64, f64) {
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;

    // Sample boundary edges and project onto extrusion direction
    for edge in &face.edges {
        for i in 0..64 {
            let t = i as f64 / 63.0;
            if let Some(p) = edge.point_at(t) {
                let v = (p.x - ext.profile.point_at(0.0).x) * ext.direction.x
                      + (p.y - ext.profile.point_at(0.0).y) * ext.direction.y
                      + (p.z - ext.profile.point_at(0.0).z) * ext.direction.z;
                v_min = v_min.min(v);
                v_max = v_max.max(v);
            }
        }
    }

    // Also try sampling from outer wire
    if v_min >= v_max {
        if let Some(ref wire) = face.outer_wire {
            for coedge in &wire.coedges {
                let edge = face.edges.iter().find(|e| e.id == coedge.edge);
                if let Some(edge) = edge {
                    for i in 0..64 {
                        let t = i as f64 / 63.0;
                        let t_actual = if coedge.forward { t } else { 1.0 - t };
                        if let Some(p) = edge.point_at(t_actual) {
                            let v = (p.x - ext.profile.point_at(0.0).x) * ext.direction.x
                                  + (p.y - ext.profile.point_at(0.0).y) * ext.direction.y
                                  + (p.z - ext.profile.point_at(0.0).z) * ext.direction.z;
                            v_min = v_min.min(v);
                            v_max = v_max.max(v);
                        }
                    }
                }
            }
        }
    }

    if v_min >= v_max {
        (0.0, 1.0)
    } else {
        let margin = (v_max - v_min) * 0.001;
        (v_min - margin, v_max + margin)
    }
}

/// Generic surface triangulation by sampling on a grid.
/// For NURBS surfaces, uses the actual knot range.
fn triangulate_generic_surface(face: &Face, surface: &Surface, params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    // Determine the parametric range based on surface type
    let (u_min, u_max, v_min, v_max) = if let Surface::Nurbs(nurbs) = surface {
        let (u0, u1) = nurbs.u_range();
        let (v0, v1) = nurbs.v_range();
        (u0, u1, v0, v1)
    } else {
        // Fallback for unknown surfaces — try boundary projection
        (0.0, 2.0 * PI, 0.0, PI)
    };

    // If we have boundary edges, use them to refine the parametric range
    let (u_min, u_max, v_min, v_max) = if let Some(ref wire) = face.outer_wire {
        let boundary_pts = collect_face_boundary_points(face);
        if !boundary_pts.is_empty() {
            let mut proj_u_min = f64::MAX;
            let mut proj_u_max = f64::MIN;
            let mut proj_v_min = f64::MAX;
            let mut proj_v_max = f64::MIN;
            for p in &boundary_pts {
                let (u, v) = surface.project_point(p);
                proj_u_min = proj_u_min.min(u);
                proj_u_max = proj_u_max.max(u);
                proj_v_min = proj_v_min.min(v);
                proj_v_max = proj_v_max.max(v);
            }
            // Use the intersection of knot range and projected range
            let u0 = proj_u_min.max(u_min);
            let u1 = proj_u_max.min(u_max);
            let v0 = proj_v_min.max(v_min);
            let v1 = proj_v_max.min(v_max);
            if u0 < u1 && v0 < v1 {
                let margin_u = (u1 - u0) * 0.01;
                let margin_v = (v1 - v0) * 0.01;
                (u0 - margin_u, u1 + margin_u, v0 - margin_v, v1 + margin_v)
            } else {
                // Projected range doesn't overlap with knot range — use knot range
                (u_min, u_max, v_min, v_max)
            }
        } else {
            (u_min, u_max, v_min, v_max)
        }
    } else {
        (u_min, u_max, v_min, v_max)
    };

    // Choose resolution based on surface type and parametric range
    let n_u = if let Surface::Nurbs(_) = surface {
        // For NURBS, use a reasonable number of samples
        params.angular_samples.max(24)
    } else {
        params.angular_samples
    };
    let n_v = if let Surface::Nurbs(_) = surface {
        params.angular_samples.max(24)
    } else {
        params.angular_samples
    };

    for j in 0..n_v {
        for i in 0..n_u {
            let u = u_min + (u_max - u_min) * i as f64 / (n_u - 1).max(1) as f64;
            let v = v_min + (v_max - v_min) * j as f64 / (n_v - 1).max(1) as f64;
            let p = surface.point_at(u, v);
            mesh.add_vertex(p);
        }
    }

    for j in 0..n_v - 1 {
        for i in 0..n_u - 1 {
            let v0 = (j * n_u + i) as u32;
            let v1 = (j * n_u + i + 1) as u32;
            let v2 = ((j + 1) * n_u + i + 1) as u32;
            let v3 = ((j + 1) * n_u + i) as u32;

            if face.forward {
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            } else {
                mesh.add_triangle(v0, v2, v1);
                mesh.add_triangle(v0, v3, v2);
            }
        }
    }

    mesh
}

/// Collect boundary points from a face's wires.
/// Uses the edge geometry stored in `face.edges` to sample actual boundary curves.
fn collect_face_boundary_points(face: &Face) -> Vec<Point3d> {
    let mut points = Vec::new();

    if let Some(ref wire) = face.outer_wire {
        let n_samples = 32; // Points per edge
        for coedge in &wire.coedges {
            // Look up the edge by ID in the face's stored edges
            let edge = face.edges.iter().find(|e| e.id == coedge.edge);
            if let Some(edge) = edge {
                // Sample the edge curve
                for i in 0..n_samples {
                    let t = i as f64 / n_samples as f64;
                    // If coedge is reversed relative to edge, reverse the parameter
                    let t_sampled = if coedge.forward { t } else { 1.0 - t };
                    if let Some(p) = edge.point_at(t_sampled) {
                        points.push(p);
                    }
                }
            } else {
                // Fallback: try to sample the surface boundary
                // This is less accurate but better than nothing
                if let Some(ref surface) = face.surface {
                    for i in 0..n_samples {
                        let t = i as f64 / n_samples as f64;
                        let p = surface.point_at(t * 2.0 * PI, 0.0);
                        points.push(p);
                    }
                }
            }
        }
    }

    // Remove duplicate points (within tolerance)
    if !points.is_empty() {
        let mut unique = vec![points[0]];
        for p in &points[1..] {
            if !unique.last().unwrap().is_coincident_with(p) {
                unique.push(*p);
            }
        }
        // Also check last vs first (closed loop)
        if unique.len() > 1 && unique.last().unwrap().is_coincident_with(&unique[0]) {
            unique.pop();
        }
        points = unique;
    }

    points
}

/// Estimate the v parameter range for a face by sampling its edges
/// and projecting sample points onto the surface's axis direction.
fn estimate_v_range(face: &Face) -> Option<(f64, f64)> {
    if let Some(ref surface) = face.surface {
        match surface {
            Surface::Cylinder(cyl) => {
                let (v_min, v_max) = compute_axis_v_range(face, &cyl.origin, &cyl.axis);
                if v_min < v_max {
                    Some((v_min, v_max))
                } else {
                    // Fallback: try to infer from the surface's bounding box
                    // For a cylinder, v is the height. Use a large default.
                    Some((0.0, 100.0))
                }
            }
            Surface::Cone(cone) => {
                let (v_min, v_max) = compute_axis_v_range(face, &cone.origin, &cone.axis);
                if v_min < v_max {
                    Some((v_min, v_max))
                } else {
                    // Fallback: use cone height as max v
                    Some((0.0, cone.height().min(100.0)))
                }
            }
            Surface::Revolution(rev) => Some(rev.profile.param_range()),
            Surface::Extrusion(ext) => Some(ext.profile.param_range()),
            _ => Some((0.0, 100.0)),
        }
    } else {
        None
    }
}

/// Compute the v parameter range for an axis-based surface (Cylinder, Cone)
/// by sampling the face's edges and projecting each sample point onto the axis.
/// v = dot(point - origin, axis)
fn compute_axis_v_range(face: &Face, origin: &Point3d, axis: &Direction3d) -> (f64, f64) {
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;

    // Sample from face.edges (direct edge geometry)
    for edge in &face.edges {
        for i in 0..64 {
            let t = i as f64 / 63.0;
            if let Some(p) = edge.point_at(t) {
                let v = (p.x - origin.x) * axis.x
                      + (p.y - origin.y) * axis.y
                      + (p.z - origin.z) * axis.z;
                v_min = v_min.min(v);
                v_max = v_max.max(v);
            }
        }
    }

    // Also try sampling from the outer wire coedges (look up edge by ID)
    if v_min >= v_max {
        if let Some(ref wire) = face.outer_wire {
            for coedge in &wire.coedges {
                let edge = face.edges.iter().find(|e| e.id == coedge.edge);
                if let Some(edge) = edge {
                    for i in 0..64 {
                        let t = i as f64 / 63.0;
                        let t_actual = if coedge.forward { t } else { 1.0 - t };
                        if let Some(p) = edge.point_at(t_actual) {
                            let v = (p.x - origin.x) * axis.x
                                  + (p.y - origin.y) * axis.y
                                  + (p.z - origin.z) * axis.z;
                            v_min = v_min.min(v);
                            v_max = v_max.max(v);
                        }
                    }
                }
            }
        }
    }

    if v_min >= v_max {
        // Fallback: compute from bounding box of all sampled 3D boundary points
        let boundary_pts = collect_face_boundary_points(face);
        for p in &boundary_pts {
            let v = (p.x - origin.x) * axis.x
                  + (p.y - origin.y) * axis.y
                  + (p.z - origin.z) * axis.z;
            v_min = v_min.min(v);
            v_max = v_max.max(v);
        }
    }

    if v_min >= v_max {
        (0.0, 100.0)
    } else {
        let margin = (v_max - v_min) * 0.001;
        (v_min - margin, v_max + margin)
    }
}

/// Ear clipping triangulation of a 2D polygon.
/// Returns triangle indices into the original point array.
pub fn ear_clip(points: &[Point2d]) -> Vec<[u32; 3]> {
    let n = points.len();
    if n < 3 {
        return vec![];
    }
    if n == 3 {
        return vec![[0, 1, 2]];
    }

    // Determine winding order
    let mut signed_area = 0.0;
    for i in 0..n {
        let j = (i + 1) % n;
        signed_area += points[i].u * points[j].v - points[j].u * points[i].v;
    }
    let ccw = signed_area > 0.0;

    // Build index list
    let mut indices: Vec<u32> = (0..n as u32).collect();
    let mut triangles = Vec::new();

    let mut attempts = 0;
    let max_attempts = n * n;

    while indices.len() > 3 && attempts < max_attempts {
        attempts += 1;
        let len = indices.len();
        let mut found_ear = false;

        for i in 0..len {
            let i_prev = if i == 0 { len - 1 } else { i - 1 };
            let i_next = (i + 1) % len;

            let a = indices[i_prev];
            let b = indices[i];
            let c = indices[i_next];

            let pa = &points[a as usize];
            let pb = &points[b as usize];
            let pc = &points[c as usize];

            // Check if this is a convex vertex
            let cross = (pb.u - pa.u) * (pc.v - pa.v) - (pb.v - pa.v) * (pc.u - pa.u);
            let is_convex = if ccw { cross > 0.0 } else { cross < 0.0 };

            if !is_convex {
                continue;
            }

            // Check if any other point is inside this triangle
            let mut is_ear = true;
            for j in 0..len {
                if j == i_prev || j == i || j == i_next {
                    continue;
                }
                let p = &points[indices[j] as usize];
                if point_in_triangle(pa, pb, pc, p) {
                    is_ear = false;
                    break;
                }
            }

            if is_ear {
                triangles.push([a, b, c]);
                indices.remove(i);
                found_ear = true;
                break;
            }
        }

        if !found_ear {
            // Degenerate polygon — just fan triangulate
            for i in 1..indices.len() - 1 {
                triangles.push([indices[0], indices[i], indices[i + 1]]);
            }
            break;
        }
    }

    if indices.len() == 3 {
        triangles.push([indices[0], indices[1], indices[2]]);
    }

    triangles
}

/// Check if a 2D point is inside a triangle.
fn point_in_triangle(a: &Point2d, b: &Point2d, c: &Point2d, p: &Point2d) -> bool {
    let d1 = sign_area_2d(p, a, b);
    let d2 = sign_area_2d(p, b, c);
    let d3 = sign_area_2d(p, c, a);

    let has_neg = (d1 < 0.0) || (d2 < 0.0) || (d3 < 0.0);
    let has_pos = (d1 > 0.0) || (d2 > 0.0) || (d3 > 0.0);

    !(has_neg && has_pos)
}

/// Signed area of triangle (p, a, b) × 2.
fn sign_area_2d(p: &Point2d, a: &Point2d, b: &Point2d) -> f64 {
    (p.u - b.u) * (a.v - b.v) - (a.u - b.u) * (p.v - b.v)
}

// ============================================================
// Boundary-aware triangulation (new API)
// ============================================================

/// Triangulate a face with boundary points for proper trimming.
/// This is the preferred entry point when boundary 3D points are available
/// from STEP file topology extraction.
pub fn triangulate_face_with_boundary(
    surface: &Surface,
    boundary_points: &[Point3d],
    forward: bool,
    params: &TriangulationParams,
) -> TriangleMesh {
    if boundary_points.is_empty() {
        // No boundary — fall back to untrimmed triangulation
        let wire = Wire::new(vec![]);
        let mut face = Face::new(surface.clone(), wire);
        face.forward = forward;
        face.edges = vec![];
        return triangulate_face(&face, params);
    }

    match surface {
        Surface::Plane(plane) => {
            triangulate_plane_with_boundary(plane, boundary_points, forward)
        }
        Surface::Cylinder(cyl) => {
            triangulate_cylinder_with_boundary(cyl, boundary_points, forward, params)
        }
        Surface::Cone(cone) => {
            triangulate_cone_with_boundary(cone, boundary_points, forward, params)
        }
        Surface::Sphere(sphere) => {
            triangulate_sphere_with_boundary(sphere, boundary_points, forward, params)
        }
        Surface::Torus(torus) => {
            triangulate_torus_with_boundary(torus, boundary_points, forward, params)
        }
        _ => {
            // For revolution, extrusion, NURBS — sample within boundary range
            triangulate_generic_with_boundary(surface, boundary_points, forward, params)
        }
    }
}

/// Triangulate a plane face with boundary points.
/// Uses simple fan triangulation for convex boundaries (no centroid vertex)
/// and falls back to ear clipping for non-convex boundaries.
fn triangulate_plane_with_boundary(
    plane: &Plane,
    boundary_points: &[Point3d],
    forward: bool,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    if boundary_points.len() < 3 {
        return mesh;
    }

    // Project 3D boundary points onto the plane's 2D coordinate system
    let points_2d: Vec<Point2d> = boundary_points.iter().map(|p| {
        let dx = p.x - plane.origin.x;
        let dy = p.y - plane.origin.y;
        let dz = p.z - plane.origin.z;
        Point2d::new(
            dx * plane.u_dir.x + dy * plane.u_dir.y + dz * plane.u_dir.z,
            dx * plane.v_dir.x + dy * plane.v_dir.y + dz * plane.v_dir.z,
        )
    }).collect();

    // Check if the polygon is convex (most cap faces are convex discs)
    let is_convex = is_convex_polygon(&points_2d);

    if is_convex && boundary_points.len() >= 3 {
        // Simple fan triangulation — uses boundary vertex 0 as the fan center.
        // This produces N-2 triangles for N boundary vertices (minimum for convex polygon).
        for p in boundary_points {
            mesh.add_vertex(*p);
        }
        let n = boundary_points.len() as u32;
        for i in 1..n-1 {
            let v0 = 0u32;       // fan center = first boundary vertex
            let v1 = i;          // boundary[i]
            let v2 = i + 1;      // boundary[i+1]
            if forward {
                mesh.add_triangle(v0, v1, v2);
            } else {
                mesh.add_triangle(v0, v2, v1);
            }
        }
    } else {
        // Non-convex polygon — use ear clipping
        let triangles = ear_clip(&points_2d);

        for p in boundary_points {
            mesh.add_vertex(*p);
        }
        for tri in &triangles {
            if forward {
                mesh.add_triangle(tri[0], tri[1], tri[2]);
            } else {
                mesh.add_triangle(tri[0], tri[2], tri[1]); // Flip winding
            }
        }
    }

    mesh
}

/// Check if a 2D polygon is convex by verifying that all cross products
/// of consecutive edges have the same sign.
fn is_convex_polygon(points: &[Point2d]) -> bool {
    if points.len() < 3 {
        return false;
    }
    let n = points.len();
    let mut sign = 0i32;
    for i in 0..n {
        let a = &points[i];
        let b = &points[(i + 1) % n];
        let c = &points[(i + 2) % n];
        let cross = (b.u - a.u) * (c.v - a.v) - (b.v - a.v) * (c.u - a.u);
        if cross.abs() > 1e-10 {
            let s = if cross > 0.0 { 1 } else { -1 };
            if sign == 0 {
                sign = s;
            } else if sign != s {
                return false;
            }
        }
    }
    sign != 0
}

/// Compute the centroid (geometric center) of a set of 3D points.
fn compute_centroid_3d(points: &[Point3d]) -> Point3d {
    let n = points.len() as f64;
    let mut x = 0.0;
    let mut y = 0.0;
    let mut z = 0.0;
    for p in points {
        x += p.x;
        y += p.y;
        z += p.z;
    }
    Point3d::new(x / n, y / n, z / n)
}

/// Compute parametric (u, v) range from boundary points for a cylinder.
/// Handles the angular wraparound properly: if the u values span the ±π boundary,
/// it adjusts the angles to ensure a contiguous angular range.
fn cylinder_uv_range(cyl: &CylinderSurface, boundary_points: &[Point3d]) -> (f64, f64, f64, f64) {
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;

    // Project all points and collect angles
    let mut angles: Vec<f64> = Vec::with_capacity(boundary_points.len());
    for p in boundary_points {
        let (u, v) = cyl.project_point(p);
        angles.push(u);
        v_min = v_min.min(v);
        v_max = v_max.max(v);
    }

    let (u_min, u_max) = compute_angular_range(&angles);

    // Add a small margin
    let u_margin = (u_max - u_min) * 0.001;
    let v_margin = (v_max - v_min) * 0.001;
    (u_min - u_margin, u_max + u_margin, v_min - v_margin, v_max + v_margin)
}

/// Compute the angular range from a list of angles, handling the ±π wraparound.
/// Returns (angle_min, angle_max) such that all angles lie within the range
/// and the range is the smallest contiguous arc covering all points.
fn compute_angular_range(angles: &[f64]) -> (f64, f64) {
    if angles.is_empty() {
        return (0.0, 2.0 * PI);
    }
    if angles.len() == 1 {
        return (angles[0], angles[0] + 2.0 * PI);
    }

    // Normalize all angles to [0, 2π)
    let mut normalized: Vec<f64> = angles.iter()
        .map(|a| ((a % (2.0 * PI)) + 2.0 * PI) % (2.0 * PI))
        .collect();
    normalized.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    // Find the largest gap between consecutive angles
    let n = normalized.len();
    let mut max_gap = 0.0f64;
    let mut gap_end_idx = 0;
    for i in 0..n {
        let next = if i + 1 < n { normalized[i + 1] } else { normalized[0] + 2.0 * PI };
        let gap = next - normalized[i];
        if gap > max_gap {
            max_gap = gap;
            gap_end_idx = i + 1;
        }
    }

    // The arc goes from the end of the largest gap to the start of the largest gap (wrapped)
    let start_angle = normalized[gap_end_idx % n];
    let end_angle = if gap_end_idx == 0 {
        normalized[n - 1]
    } else if gap_end_idx < n {
        normalized[gap_end_idx - 1]
    } else {
        normalized[n - 1]
    };

    let range = if gap_end_idx == 0 {
        // The gap wraps around 2π
        end_angle - start_angle + 2.0 * PI
    } else {
        end_angle - start_angle
    };

    // If the range is close to 2π, it's a full circle
    if range > 1.99 * PI {
        (0.0, 2.0 * PI)
    } else {
        (start_angle, start_angle + range)
    }
}

/// Triangulate a cylinder face trimmed by boundary points.
fn triangulate_cylinder_with_boundary(
    cyl: &CylinderSurface,
    boundary_points: &[Point3d],
    forward: bool,
    params: &TriangulationParams,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let (u_min, u_max, v_min, v_max) = cylinder_uv_range(cyl, boundary_points);

    let n_u = params.angular_samples;
    let n_v = params.height_samples.max(2);

    // If the u range covers the full circle, render full circle
    let u_range = u_max - u_min;
    let full_circle = u_range > 1.9 * PI;
    let u_start = if full_circle { 0.0 } else { u_min };
    let u_end = if full_circle { 2.0 * PI } else { u_max };

    for j in 0..n_v {
        for i in 0..n_u {
            let u = u_start + (u_end - u_start) * i as f64 / n_u as f64;
            let v = v_min + (v_max - v_min) * j as f64 / (n_v - 1) as f64;
            let p = cyl.point_at(u, v);
            mesh.add_vertex(p);
        }
    }

    let n_u_loop = if full_circle { n_u } else { n_u - 1 };
    for j in 0..n_v - 1 {
        for i in 0..n_u_loop {
            let i_next = (i + 1) % n_u;
            let v0 = (j * n_u + i) as u32;
            let v1 = (j * n_u + i_next) as u32;
            let v2 = ((j + 1) * n_u + i_next) as u32;
            let v3 = ((j + 1) * n_u + i) as u32;

            if forward {
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            } else {
                mesh.add_triangle(v0, v2, v1);
                mesh.add_triangle(v0, v3, v2);
            }
        }
    }

    mesh
}

/// Compute parametric (u, v) range from boundary points for a cone.
/// Handles the angular wraparound properly using the same algorithm as cylinders.
fn cone_uv_range(cone: &ConeSurface, boundary_points: &[Point3d]) -> (f64, f64, f64, f64) {
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;

    // Project all points and collect angles
    let mut angles: Vec<f64> = Vec::with_capacity(boundary_points.len());
    for p in boundary_points {
        let (u, v) = cone.project_point(p);
        angles.push(u);
        v_min = v_min.min(v);
        v_max = v_max.max(v);
    }

    let (u_min, u_max) = compute_angular_range(&angles);

    let u_margin = (u_max - u_min) * 0.001;
    let v_margin = (v_max - v_min) * 0.001;
    (u_min - u_margin, u_max + u_margin, v_min - v_margin, v_max + v_margin)
}

/// Triangulate a cone face trimmed by boundary points.
fn triangulate_cone_with_boundary(
    cone: &ConeSurface,
    boundary_points: &[Point3d],
    forward: bool,
    params: &TriangulationParams,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let (u_min, u_max, v_min, v_max) = cone_uv_range(cone, boundary_points);

    let n_u = params.angular_samples;
    let n_v = params.height_samples.max(2);

    let u_range = u_max - u_min;
    let full_circle = u_range > 1.9 * PI;
    let u_start = if full_circle { 0.0 } else { u_min };
    let u_end = if full_circle { 2.0 * PI } else { u_max };

    for j in 0..n_v {
        for i in 0..n_u {
            let u = u_start + (u_end - u_start) * i as f64 / n_u as f64;
            let v = v_min + (v_max - v_min) * j as f64 / (n_v - 1) as f64;
            let p = cone.point_at(u, v);
            mesh.add_vertex(p);
        }
    }

    let n_u_loop = if full_circle { n_u } else { n_u - 1 };
    for j in 0..n_v - 1 {
        for i in 0..n_u_loop {
            let i_next = (i + 1) % n_u;
            let v0 = (j * n_u + i) as u32;
            let v1 = (j * n_u + i_next) as u32;
            let v2 = ((j + 1) * n_u + i_next) as u32;
            let v3 = ((j + 1) * n_u + i) as u32;

            if forward {
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            } else {
                mesh.add_triangle(v0, v2, v1);
                mesh.add_triangle(v0, v3, v2);
            }
        }
    }

    mesh
}

/// Compute parametric (u, v) range from boundary points for a sphere.
/// Uses proper angular range computation for the u parameter.
fn sphere_uv_range(sphere: &SphereSurface, boundary_points: &[Point3d]) -> (f64, f64, f64, f64) {
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;

    let mut angles: Vec<f64> = Vec::with_capacity(boundary_points.len());
    for p in boundary_points {
        let (u, v) = sphere.project_point(p);
        angles.push(u);
        v_min = v_min.min(v);
        v_max = v_max.max(v);
    }

    let (u_min, u_max) = compute_angular_range(&angles);

    let u_margin = (u_max - u_min) * 0.001;
    let v_margin = (v_max - v_min) * 0.001;
    (u_min - u_margin, u_max + u_margin, v_min - v_margin, v_max + v_margin)
}

/// Triangulate a sphere face trimmed by boundary points.
fn triangulate_sphere_with_boundary(
    sphere: &SphereSurface,
    boundary_points: &[Point3d],
    forward: bool,
    params: &TriangulationParams,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let (u_min, u_max, v_min, v_max) = sphere_uv_range(sphere, boundary_points);

    let n_u = params.angular_samples;
    let n_v = (params.angular_samples / 2).max(4);

    // Check if this is a full sphere
    let u_range = u_max - u_min;
    let v_range = v_max - v_min;
    let full_u = u_range > 1.9 * PI;
    let full_v = v_range > 0.9 * PI;

    let u_start = if full_u { 0.0 } else { u_min };
    let u_end = if full_u { 2.0 * PI } else { u_max };
    let v_start = if full_v { 0.0 } else { v_min };
    let v_end = if full_v { PI } else { v_max };

    for j in 0..=n_v {
        for i in 0..n_u {
            let u = u_start + (u_end - u_start) * i as f64 / n_u as f64;
            let v = v_start + (v_end - v_start) * j as f64 / n_v as f64;
            let p = sphere.point_at(u, v);
            mesh.add_vertex(p);
        }
    }

    for j in 0..n_v {
        for i in 0..n_u {
            let i_next = (i + 1) % n_u;
            let v0 = (j * n_u + i) as u32;
            let v1 = (j * n_u + i_next) as u32;
            let v2 = ((j + 1) * n_u + i_next) as u32;
            let v3 = ((j + 1) * n_u + i) as u32;

            if forward {
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            } else {
                mesh.add_triangle(v0, v2, v1);
                mesh.add_triangle(v0, v3, v2);
            }
        }
    }

    mesh
}

/// Compute parametric (u, v) range from boundary points for a torus.
/// Uses proper angular range computation for both u and v parameters.
fn torus_uv_range(torus: &TorusSurface, boundary_points: &[Point3d]) -> (f64, f64, f64, f64) {
    let mut u_angles: Vec<f64> = Vec::with_capacity(boundary_points.len());
    let mut v_angles: Vec<f64> = Vec::with_capacity(boundary_points.len());

    for p in boundary_points {
        let (u, v) = torus.project_point(p);
        u_angles.push(u);
        v_angles.push(v);
    }

    let (u_min, u_max) = compute_angular_range(&u_angles);
    let (v_min, v_max) = compute_angular_range(&v_angles);

    let u_margin = (u_max - u_min) * 0.001;
    let v_margin = (v_max - v_min) * 0.001;
    (u_min - u_margin, u_max + u_margin, v_min - v_margin, v_max + v_margin)
}

/// Triangulate a torus face trimmed by boundary points.
fn triangulate_torus_with_boundary(
    torus: &TorusSurface,
    boundary_points: &[Point3d],
    forward: bool,
    params: &TriangulationParams,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let (u_min, u_max, v_min, v_max) = torus_uv_range(torus, boundary_points);

    let n_u = params.angular_samples;
    let n_v = params.angular_samples;

    let u_range = u_max - u_min;
    let v_range = v_max - v_min;
    let full_u = u_range > 1.9 * PI;
    let full_v = v_range > 1.9 * PI;

    let u_start = if full_u { 0.0 } else { u_min };
    let u_end = if full_u { 2.0 * PI } else { u_max };
    let v_start = if full_v { 0.0 } else { v_min };
    let v_end = if full_v { 2.0 * PI } else { v_max };

    for j in 0..n_v {
        for i in 0..n_u {
            let u = u_start + (u_end - u_start) * i as f64 / n_u as f64;
            let v = v_start + (v_end - v_start) * j as f64 / n_v as f64;
            let p = torus.point_at(u, v);
            mesh.add_vertex(p);
        }
    }

    for j in 0..n_v {
        for i in 0..n_u {
            let i_next = (i + 1) % n_u;
            let j_next = (j + 1) % n_v;
            let v0 = (j * n_u + i) as u32;
            let v1 = (j * n_u + i_next) as u32;
            let v2 = (j_next * n_u + i_next) as u32;
            let v3 = (j_next * n_u + i) as u32;

            if forward {
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            } else {
                mesh.add_triangle(v0, v2, v1);
                mesh.add_triangle(v0, v3, v2);
            }
        }
    }

    mesh
}

/// Generic surface triangulation with boundary points.
/// Projects boundary points to parametric space, determines the parametric range,
/// then samples within that range.
fn triangulate_generic_with_boundary(
    surface: &Surface,
    boundary_points: &[Point3d],
    forward: bool,
    params: &TriangulationParams,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    // Determine the base parametric range
    let (base_u_min, base_u_max, base_v_min, base_v_max) = if let Surface::Nurbs(nurbs) = surface {
        let (u0, u1) = nurbs.u_range();
        let (v0, v1) = nurbs.v_range();
        (u0, u1, v0, v1)
    } else {
        (0.0, 2.0 * PI, 0.0, PI)
    };

    // Compute parametric range from boundary points
    let mut u_min = f64::MAX;
    let mut u_max = f64::MIN;
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;

    for p in boundary_points {
        let (u, v) = surface.project_point(p);
        u_min = u_min.min(u);
        u_max = u_max.max(u);
        v_min = v_min.min(v);
        v_max = v_max.max(v);
    }

    // Clamp the projected range to the valid parametric domain
    u_min = u_min.max(base_u_min);
    u_max = u_max.min(base_u_max);
    v_min = v_min.max(base_v_min);
    v_max = v_max.min(base_v_max);

    // If we couldn't determine a range, use the base range
    if u_min >= u_max || v_min >= v_max {
        u_min = base_u_min;
        u_max = base_u_max;
        v_min = base_v_min;
        v_max = base_v_max;
    }

    // Add margin
    let u_margin = (u_max - u_min) * 0.01;
    let v_margin = (v_max - v_min) * 0.01;
    u_min = (u_min - u_margin).max(base_u_min);
    u_max = (u_max + u_margin).min(base_u_max);
    v_min = (v_min - v_margin).max(base_v_min);
    v_max = (v_max + v_margin).min(base_v_max);

    // Choose resolution — use more samples for better quality
    let n_u = if let Surface::Nurbs(_) = surface {
        params.angular_samples.max(24)
    } else {
        params.angular_samples
    };
    let n_v = if let Surface::Nurbs(_) = surface {
        params.angular_samples.max(24)
    } else {
        params.angular_samples
    };

    for j in 0..n_v {
        for i in 0..n_u {
            let u = u_min + (u_max - u_min) * i as f64 / (n_u - 1).max(1) as f64;
            let v = v_min + (v_max - v_min) * j as f64 / (n_v - 1).max(1) as f64;
            let p = surface.point_at(u, v);
            mesh.add_vertex(p);
        }
    }

    for j in 0..n_v - 1 {
        for i in 0..n_u - 1 {
            let v0 = (j * n_u + i) as u32;
            let v1 = (j * n_u + i + 1) as u32;
            let v2 = ((j + 1) * n_u + i + 1) as u32;
            let v3 = ((j + 1) * n_u + i) as u32;

            if forward {
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            } else {
                mesh.add_triangle(v0, v2, v1);
                mesh.add_triangle(v0, v3, v2);
            }
        }
    }

    mesh
}

/// Merge coincident vertices in a mesh within the given tolerance.
/// This makes closed solids watertight by ensuring that shared edge
/// vertices between adjacent faces use the same vertex index.
pub fn merge_coincident_vertices(mesh: &mut TriangleMesh, tolerance: f64) {
    if mesh.vertices.is_empty() {
        return;
    }
    
    let tol_sq = tolerance * tolerance;
    let n = mesh.vertices.len();
    
    // Build a mapping: old vertex index -> new vertex index
    let mut remap: Vec<u32> = vec![0; n];
    let mut new_vertices: Vec<Point3d> = Vec::with_capacity(n);
    
    // Spatial hash for fast lookup
    let cell_size = tolerance * 10.0;
    let mut grid: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
    
    for (i, v) in mesh.vertices.iter().enumerate() {
        let cx = (v.x / cell_size).floor() as i64;
        let cy = (v.y / cell_size).floor() as i64;
        let cz = (v.z / cell_size).floor() as i64;
        
        let mut found = false;
        // Check neighboring cells
        for dx in -1i64..=1 {
            for dy in -1i64..=1 {
                for dz in -1i64..=1 {
                    let key = (cx + dx, cy + dy, cz + dz);
                    if let Some(indices) = grid.get(&key) {
                        for &j in indices {
                            let ov = &new_vertices[j as usize];
                            let ddx = v.x - ov.x;
                            let ddy = v.y - ov.y;
                            let ddz = v.z - ov.z;
                            if ddx*ddx + ddy*ddy + ddz*ddz < tol_sq {
                                remap[i] = j;
                                found = true;
                                break;
                            }
                        }
                    }
                    if found { break; }
                }
                if found { break; }
            }
            if found { break; }
        }
        
        if !found {
            let new_idx = new_vertices.len() as u32;
            new_vertices.push(*v);
            remap[i] = new_idx;
            grid.entry((cx, cy, cz)).or_default().push(new_idx);
        }
    }
    
    // Apply remap to triangles and filter degenerate ones
    let old_triangles = std::mem::take(&mut mesh.triangles);
    for tri in &old_triangles {
        let a = remap[tri[0] as usize];
        let b = remap[tri[1] as usize];
        let c = remap[tri[2] as usize];
        // Skip degenerate triangles (collapsed vertices)
        if a != b && b != c && a != c {
            mesh.triangles.push([a, b, c]);
        }
    }
    
    mesh.vertices = new_vertices;
    
    // Rebuild normals if present
    if mesh.normals.is_some() {
        mesh.normals = None; // Normals are no longer valid after vertex merge
    }
}
