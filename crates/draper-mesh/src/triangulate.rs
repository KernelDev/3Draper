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

/// Triangulate a planar face using ear clipping.
fn triangulate_planar_face(face: &Face, plane: &Plane, _params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    // Collect boundary points
    let points_3d = collect_face_boundary_points(face);

    if points_3d.is_empty() {
        // No boundary — skip
        return mesh;
    }

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

    // Ear clipping triangulation
    let triangles = ear_clip(&points_2d);

    // Add vertices and triangles
    for p in &points_3d {
        mesh.add_vertex(*p);
    }
    for tri in &triangles {
        let face_forward = face.forward;
        if face_forward {
            mesh.add_triangle(tri[0], tri[1], tri[2]);
        } else {
            mesh.add_triangle(tri[0], tri[2], tri[1]); // Flip winding
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
            mesh.add_vertex(p);
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
            mesh.add_vertex(p);
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
            mesh.add_vertex(p);
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

    let (v_min, v_max) = estimate_v_range(face).unwrap_or((0.0, 1.0));

    for j in 0..n_v {
        for i in 0..n_u {
            let u = i as f64 / (n_u - 1) as f64;
            let v = v_min + (v_max - v_min) * j as f64 / (n_v - 1) as f64;
            let (u_min, u_max) = ext.profile.param_range();
            let u_param = u_min + u * (u_max - u_min);
            let p = ext.point_at(u_param, v);
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

/// Generic surface triangulation by sampling on a grid.
fn triangulate_generic_surface(face: &Face, surface: &Surface, params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let n = params.angular_samples;

    for j in 0..n {
        for i in 0..n {
            let u = 2.0 * PI * i as f64 / n as f64;
            let v = PI * j as f64 / n as f64;
            let p = surface.point_at(u, v);
            mesh.add_vertex(p);
        }
    }

    for j in 0..n - 1 {
        for i in 0..n {
            let i_next = (i + 1) % n;
            let v0 = (j * n + i) as u32;
            let v1 = (j * n + i_next) as u32;
            let v2 = ((j + 1) * n + i_next) as u32;
            let v3 = ((j + 1) * n + i) as u32;

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
fn ear_clip(points: &[Point2d]) -> Vec<[u32; 3]> {
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

/// Triangulate a plane face with boundary points using ear clipping.
fn triangulate_plane_with_boundary(
    plane: &Plane,
    boundary_points: &[Point3d],
    forward: bool,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

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

    // Ear clipping triangulation
    let triangles = ear_clip(&points_2d);

    // Add vertices and triangles
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

    mesh
}

/// Compute parametric (u, v) range from boundary points for a cylinder.
fn cylinder_uv_range(cyl: &CylinderSurface, boundary_points: &[Point3d]) -> (f64, f64, f64, f64) {
    let mut u_min = f64::MAX;
    let mut u_max = f64::MIN;
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;

    for p in boundary_points {
        let (u, v) = cyl.project_point(p);
        u_min = u_min.min(u);
        u_max = u_max.max(u);
        v_min = v_min.min(v);
        v_max = v_max.max(v);
    }

    // Add a small margin
    let u_margin = (u_max - u_min) * 0.001;
    let v_margin = (v_max - v_min) * 0.001;
    (u_min - u_margin, u_max + u_margin, v_min - v_margin, v_max + v_margin)
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
fn cone_uv_range(cone: &ConeSurface, boundary_points: &[Point3d]) -> (f64, f64, f64, f64) {
    let mut u_min = f64::MAX;
    let mut u_max = f64::MIN;
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;

    for p in boundary_points {
        let (u, v) = cone.project_point(p);
        u_min = u_min.min(u);
        u_max = u_max.max(u);
        v_min = v_min.min(v);
        v_max = v_max.max(v);
    }

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
fn sphere_uv_range(sphere: &SphereSurface, boundary_points: &[Point3d]) -> (f64, f64, f64, f64) {
    let mut u_min = f64::MAX;
    let mut u_max = f64::MIN;
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;

    for p in boundary_points {
        let (u, v) = sphere.project_point(p);
        u_min = u_min.min(u);
        u_max = u_max.max(u);
        v_min = v_min.min(v);
        v_max = v_max.max(v);
    }

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
fn torus_uv_range(torus: &TorusSurface, boundary_points: &[Point3d]) -> (f64, f64, f64, f64) {
    let mut u_min = f64::MAX;
    let mut u_max = f64::MIN;
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;

    for p in boundary_points {
        let (u, v) = torus.project_point(p);
        u_min = u_min.min(u);
        u_max = u_max.max(u);
        v_min = v_min.min(v);
        v_max = v_max.max(v);
    }

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
    let n = params.angular_samples;

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

    // If we couldn't determine a range, use defaults
    if u_min >= u_max || v_min >= v_max {
        // Fall back to generic sampling
        let wire = Wire::new(vec![]);
        let mut face = Face::new(surface.clone(), wire);
        face.forward = forward;
        face.edges = vec![];
        return triangulate_face(&face, params);
    }

    // Add margin
    let u_margin = (u_max - u_min) * 0.001;
    let v_margin = (v_max - v_min) * 0.001;
    u_min -= u_margin;
    u_max += u_margin;
    v_min -= v_margin;
    v_max += v_margin;

    for j in 0..n {
        for i in 0..n {
            let u = u_min + (u_max - u_min) * i as f64 / (n - 1).max(1) as f64;
            let v = v_min + (v_max - v_min) * j as f64 / (n - 1).max(1) as f64;
            let p = surface.point_at(u, v);
            mesh.add_vertex(p);
        }
    }

    for j in 0..n - 1 {
        for i in 0..n - 1 {
            let v0 = (j * n + i) as u32;
            let v1 = (j * n + i + 1) as u32;
            let v2 = ((j + 1) * n + i + 1) as u32;
            let v3 = ((j + 1) * n + i) as u32;

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
