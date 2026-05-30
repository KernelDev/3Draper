//! Face triangulation — converts B-Rep faces to triangle meshes.
//!
//! Design principles:
//! 1. Edge curves are sampled at consistent parameter values so shared edges
//!    between adjacent faces produce identical 3D points (triangulation consistency).
//! 2. Planes use minimum number of triangles (ear-clipping, no interior subdivision).
//! 3. Curved surfaces use edge samples as boundary ring vertices.
//! 4. Post-hoc merge_coincident_vertices ensures watertight closed solids.

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

/// Number of samples per edge curve for boundary discretization.
const EDGE_SAMPLES: usize = 32;

// ============================================================
// Top-level entry points
// ============================================================

/// Triangulate a solid into a triangle mesh.
/// After merging all faces, merges coincident vertices to ensure
/// that shared edges are watertight.
pub fn triangulate_solid(solid: &Solid, params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    for face in solid.faces() {
        let face_mesh = triangulate_face(face, params);
        mesh.merge(&face_mesh);
    }
    // Merge coincident boundary vertices to make the solid watertight
    merge_coincident_vertices(&mut mesh, 1e-6);
    mesh
}

/// Triangulate a shell into a triangle mesh.
pub fn triangulate_shell(shell: &Shell, params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    for face in &shell.faces {
        let face_mesh = triangulate_face(face, params);
        mesh.merge(&face_mesh);
    }
    merge_coincident_vertices(&mut mesh, 1e-6);
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
                triangulate_generic_surface(face, surface, params)
            }
        }
    } else {
        TriangleMesh::new()
    }
}

// ============================================================
// Edge curve sampling — the foundation of consistent triangulation
// ============================================================

/// Sample points along a single edge curve.
/// Returns the sampled 3D points (not including the endpoint to avoid duplicates
/// when chaining edges into a wire).
fn sample_edge_points(edge: &Edge, n_samples: usize) -> Vec<Point3d> {
    let mut pts = Vec::with_capacity(n_samples);
    for i in 0..n_samples {
        let t = i as f64 / n_samples as f64;
        if let Some(p) = edge.point_at(t) {
            pts.push(p);
        }
    }
    pts
}

/// Collect boundary points from a face's outer wire by sampling edge curves.
/// Each edge is sampled at consistent parameter values so that shared edges
/// between adjacent faces produce identical 3D points.
fn collect_face_boundary_points(face: &Face) -> Vec<Point3d> {
    let mut points = Vec::new();

    if let Some(ref wire) = face.outer_wire {
        for coedge in &wire.coedges {
            let edge = face.edges.iter().find(|e| e.id == coedge.edge);
            if let Some(edge) = edge {
                let mut edge_pts = sample_edge_points(edge, EDGE_SAMPLES);
                // If coedge is reversed, reverse the sample order
                if !coedge.forward {
                    edge_pts.reverse();
                }
                points.extend(edge_pts);
            }
        }
    }

    // Remove duplicate consecutive points (within tolerance)
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

/// Collect boundary points from a face's inner wires (holes).
fn collect_face_hole_points(face: &Face) -> Vec<Vec<Point3d>> {
    let mut holes = Vec::new();
    for wire in &face.inner_wires {
        let mut points = Vec::new();
        for coedge in &wire.coedges {
            let edge = face.edges.iter().find(|e| e.id == coedge.edge);
            if let Some(edge) = edge {
                let mut edge_pts = sample_edge_points(edge, EDGE_SAMPLES);
                if !coedge.forward {
                    edge_pts.reverse();
                }
                points.extend(edge_pts);
            }
        }
        // Deduplicate
        if !points.is_empty() {
            let mut unique = vec![points[0]];
            for p in &points[1..] {
                if !unique.last().unwrap().is_coincident_with(p) {
                    unique.push(*p);
                }
            }
            if unique.len() > 1 && unique.last().unwrap().is_coincident_with(&unique[0]) {
                unique.pop();
            }
            holes.push(unique);
        }
    }
    holes
}

// ============================================================
// Planar face triangulation — minimum triangle count
// ============================================================

/// Triangulate a planar face.
/// Uses ear-clipping on the boundary polygon — this produces the minimum
/// number of triangles for a given boundary polygon (N-2 for convex).
/// Supports holes via bridge-edge technique.
fn triangulate_planar_face(face: &Face, plane: &Plane, _params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    let boundary_3d = collect_face_boundary_points(face);
    if boundary_3d.is_empty() {
        return mesh;
    }

    let holes_3d = collect_face_hole_points(face);
    let forward = face.forward;

    // Project 3D boundary points onto the plane's 2D coordinate system
    let project = |p: &Point3d| -> Point2d {
        let dx = p.x - plane.origin.x;
        let dy = p.y - plane.origin.y;
        let dz = p.z - plane.origin.z;
        Point2d::new(
            dx * plane.u_dir.x + dy * plane.u_dir.y + dz * plane.u_dir.z,
            dx * plane.v_dir.x + dy * plane.v_dir.y + dz * plane.v_dir.z,
        )
    };

    let points_2d: Vec<Point2d> = boundary_3d.iter().map(|p| project(p)).collect();

    if holes_3d.is_empty() {
        // No holes — simple polygon triangulation
        let is_convex = is_convex_polygon(&points_2d);

        if is_convex && boundary_3d.len() >= 3 {
            // Fan triangulation — N-2 triangles for N boundary vertices (minimum)
            for p in &boundary_3d {
                mesh.add_vertex(*p);
            }
            let n = boundary_3d.len() as u32;
            for i in 1..n - 1 {
                if forward {
                    mesh.add_triangle(0, i, i + 1);
                } else {
                    mesh.add_triangle(0, i + 1, i);
                }
            }
        } else {
            // Ear clipping for non-convex polygons
            let triangles = ear_clip(&points_2d);
            for p in &boundary_3d {
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
    } else {
        // Has holes — merge holes into outer polygon via bridge edges, then ear-clip
        let mut all_points_3d = boundary_3d.clone();
        let mut all_points_2d = points_2d.clone();

        // Compute the polygon index ranges after each hole insertion
        let mut polygon_indices: Vec<u32> = (0..boundary_3d.len() as u32).collect();

        for hole_3d in &holes_3d {
            let hole_2d: Vec<Point2d> = hole_3d.iter().map(|p| project(p)).collect();
            let hole_start_idx = all_points_3d.len();

            // Find the bridge: rightmost point of the hole, closest point on outer polygon
            let bridge_result = find_bridge_edge(&all_points_2d, &hole_2d);

            // Add hole points to the combined point list
            for p in hole_3d {
                all_points_3d.push(*p);
            }
            all_points_2d.extend(hole_2d);

            // Insert hole into polygon via bridge edge
            let mut new_polygon = Vec::with_capacity(polygon_indices.len() + hole_3d.len() + 2);
            let bridge_outer = bridge_result.outer_idx;
            let bridge_hole = hole_start_idx + bridge_result.hole_idx;

            for &idx in &polygon_indices[..=bridge_outer as usize] {
                new_polygon.push(idx);
            }
            // Bridge: outer → hole → ... hole loop ... → hole → outer
            new_polygon.push(bridge_hole as u32);
            for i in 0..hole_3d.len() {
                let idx = (bridge_hole + i) % hole_3d.len() + hole_start_idx;
                new_polygon.push(idx as u32);
            }
            new_polygon.push(bridge_hole as u32);
            new_polygon.push(polygon_indices[bridge_outer as usize]);
            for &idx in &polygon_indices[bridge_outer as usize + 1..] {
                new_polygon.push(idx);
            }

            polygon_indices = new_polygon;
        }

        // Now ear-clip the merged polygon
        let merged_2d: Vec<Point2d> = polygon_indices.iter()
            .map(|&idx| all_points_2d[idx as usize])
            .collect();

        let triangles = ear_clip(&merged_2d);

        // Add all vertices
        for p in &all_points_3d {
            mesh.add_vertex(*p);
        }

        // Map ear-clip indices back to original vertex indices
        for tri in &triangles {
            let i0 = polygon_indices[tri[0] as usize];
            let i1 = polygon_indices[tri[1] as usize];
            let i2 = polygon_indices[tri[2] as usize];
            if forward {
                mesh.add_triangle(i0, i1, i2);
            } else {
                mesh.add_triangle(i0, i2, i1);
            }
        }
    }

    // Compute face normals for the planar face
    let normal = if forward {
        plane.normal
    } else {
        Direction3d::new(-plane.normal.x, -plane.normal.y, -plane.normal.z).unwrap_or(Direction3d::Z)
    };
    mesh.face_normals = Some(vec![[normal.x, normal.y, normal.z]; mesh.triangles.len()]);

    mesh
}

/// Result of finding a bridge edge between outer polygon and a hole.
struct BridgeResult {
    outer_idx: usize,
    hole_idx: usize,
}

/// Find the best bridge edge between an outer polygon and a hole.
/// Uses the rightmost-hole-point / closest-outer-point technique.
fn find_bridge_edge(outer_2d: &[Point2d], hole_2d: &[Point2d]) -> BridgeResult {
    // Find rightmost point of the hole
    let mut hole_idx = 0;
    let mut max_u = hole_2d[0].u;
    for (i, p) in hole_2d.iter().enumerate() {
        if p.u > max_u {
            max_u = p.u;
            hole_idx = i;
        }
    }

    // Find closest point on outer polygon to the rightmost hole point
    let hole_pt = &hole_2d[hole_idx];
    let mut outer_idx = 0;
    let mut min_dist = f64::MAX;
    for (i, p) in outer_2d.iter().enumerate() {
        let dx = p.u - hole_pt.u;
        let dy = p.v - hole_pt.v;
        let dist = dx * dx + dy * dy;
        if dist < min_dist {
            min_dist = dist;
            outer_idx = i;
        }
    }

    BridgeResult { outer_idx, hole_idx }
}

// ============================================================
// Curved surface triangulation — boundary-consistent
// ============================================================

/// Triangulate a cylinder face.
/// The boundary ring vertices are sampled from edge curves (ensuring consistency
/// with adjacent faces). Interior vertices are sampled from the parametric grid.
/// Top/bottom cap rings are snapped to edge curves when available.
fn triangulate_cylinder_face(face: &Face, cyl: &CylinderSurface, params: &TriangulationParams) -> TriangleMesh {
    let boundary_3d = collect_face_boundary_points(face);

    if boundary_3d.is_empty() {
        // No boundary edges — sample full cylinder
        return triangulate_cylinder_full(face, cyl, params);
    }

    // Determine UV range from boundary
    let (u_min, u_max, v_min, v_max) = cylinder_uv_range(cyl, &boundary_3d);
    let u_range = u_max - u_min;
    let full_circle = u_range > 1.9 * PI;

    // Sample the cylinder surface on a grid, but snap boundary rings to edge curves
    let n_u = if full_circle { params.angular_samples } else { params.angular_samples.min(48) };
    let n_v = params.height_samples.max(2);

    let u_start = if full_circle { 0.0 } else { u_min };
    let u_end = if full_circle { 2.0 * PI } else { u_max };

    // Generate interior grid points (not boundary rings)
    let mut mesh = TriangleMesh::new();

    // Sample rows from j=0 to j=n_v-1
    // For each row, generate points along u
    // j=0 and j=n_v-1 are boundary rows — we'll try to snap them to edge curves
    // Interior rows are purely from surface.point_at

    // Collect edge-projected boundary rings at v_min and v_max
    let bottom_ring = extract_boundary_ring_at_v(&boundary_3d, cyl, v_min, n_u, full_circle, u_start, u_end);
    let top_ring = extract_boundary_ring_at_v(&boundary_3d, cyl, v_max, n_u, full_circle, u_start, u_end);

    for j in 0..n_v {
        for i in 0..n_u {
            let u = u_start + (u_end - u_start) * i as f64 / n_u as f64;
            let v = v_min + (v_max - v_min) * j as f64 / (n_v - 1).max(1) as f64;
            let p = cyl.point_at(u, v);
            let n = cyl.normal_at(u, v);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
        }
    }

    // Snap boundary row vertices to edge curve samples
    if !bottom_ring.is_empty() && n_u == bottom_ring.len() {
        for i in 0..n_u {
            mesh.vertices[i] = bottom_ring[i];
        }
    }
    if !top_ring.is_empty() && n_u == top_ring.len() {
        let offset = (n_v - 1) * n_u;
        for i in 0..n_u {
            mesh.vertices[offset + i] = top_ring[i];
        }
    }

    // Generate triangles
    let n_u_loop = if full_circle { n_u } else { n_u - 1 };
    for j in 0..n_v - 1 {
        for i in 0..n_u_loop {
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

/// Full cylinder triangulation (no boundary edges).
fn triangulate_cylinder_full(face: &Face, cyl: &CylinderSurface, params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let n_u = params.angular_samples;
    let n_v = params.height_samples.max(2);
    let (v_min, v_max) = compute_axis_v_range(face, &cyl.origin, &cyl.axis);
    let (v_min, v_max) = if v_min < v_max { (v_min, v_max) } else { (0.0, 1.0) };

    for j in 0..n_v {
        for i in 0..n_u {
            let u = 2.0 * PI * i as f64 / n_u as f64;
            let v = v_min + (v_max - v_min) * j as f64 / (n_v - 1).max(1) as f64;
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

/// Extract boundary ring points at a specific v value from edge curve samples.
/// Returns points sorted by u parameter, interpolated to n_u evenly spaced angles.
fn extract_boundary_ring_at_v(
    boundary_3d: &[Point3d],
    cyl: &CylinderSurface,
    target_v: f64,
    n_u: usize,
    full_circle: bool,
    u_start: f64,
    u_end: f64,
) -> Vec<Point3d> {
    // Collect boundary points near target_v
    let v_tol = 1e-4;
    let mut ring_pts: Vec<(f64, Point3d)> = Vec::new();

    for p in boundary_3d {
        let (u, v) = cyl.project_point(p);
        if (v - target_v).abs() < v_tol * (target_v.abs().max(1.0)) + 1e-6 {
            ring_pts.push((u, *p));
        }
    }

    if ring_pts.len() < 3 {
        return Vec::new();
    }

    // Sort by u
    ring_pts.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    // Generate n_u points at evenly spaced u values, sampling from the ring
    let mut result = Vec::with_capacity(n_u);
    for i in 0..n_u {
        let u = u_start + (u_end - u_start) * i as f64 / n_u as f64;
        // Find the closest ring point by angle, or evaluate the surface
        let p = cyl.point_at(u, target_v);

        // Snap to the closest boundary point if it's very close
        let mut best_dist = f64::MAX;
        let mut best_pt = p;
        for (ru, rp) in &ring_pts {
            let du = if full_circle {
                let diff = (u - ru).abs();
                diff.min(2.0 * PI - diff)
            } else {
                (u - ru).abs()
            };
            if du < best_dist {
                best_dist = du;
                best_pt = *rp;
            }
        }
        // If there's a boundary point very close in angle, snap to it
        let angle_tol = (u_end - u_start) / n_u as f64 * 0.3;
        if best_dist < angle_tol {
            result.push(best_pt);
        } else {
            result.push(p);
        }
    }

    result
}

/// Triangulate a cone face.
fn triangulate_cone_face(face: &Face, cone: &ConeSurface, params: &TriangulationParams) -> TriangleMesh {
    let boundary_3d = collect_face_boundary_points(face);

    if boundary_3d.is_empty() {
        return triangulate_cone_full(face, cone, params);
    }

    let (u_min, u_max, v_min, v_max) = cone_uv_range(cone, &boundary_3d);
    let u_range = u_max - u_min;
    let full_circle = u_range > 1.9 * PI;

    let n_u = if full_circle { params.angular_samples } else { params.angular_samples.min(48) };
    let n_v = params.height_samples.max(2);

    let u_start = if full_circle { 0.0 } else { u_min };
    let u_end = if full_circle { 2.0 * PI } else { u_max };

    let mut mesh = TriangleMesh::new();

    for j in 0..n_v {
        for i in 0..n_u {
            let u = u_start + (u_end - u_start) * i as f64 / n_u as f64;
            let v = v_min + (v_max - v_min) * j as f64 / (n_v - 1).max(1) as f64;
            let p = cone.point_at(u, v);
            let n = cone.normal_at(u, v);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
        }
    }

    // Snap boundary rings to edge curves
    if let Some(ref surface) = face.surface {
        snap_boundary_rings(&mut mesh, &boundary_3d, surface, n_u, n_v, u_start, u_end, v_min, v_max, full_circle);
    }

    let n_u_loop = if full_circle { n_u } else { n_u - 1 };
    for j in 0..n_v - 1 {
        for i in 0..n_u_loop {
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

/// Full cone triangulation (no boundary edges).
fn triangulate_cone_full(face: &Face, cone: &ConeSurface, params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let n_u = params.angular_samples;
    let n_v = params.height_samples.max(2);
    let (v_min, v_max) = compute_axis_v_range(face, &cone.origin, &cone.axis);
    let (v_min, v_max) = if v_min < v_max { (v_min, v_max) } else { (0.0, cone.height().min(100.0)) };

    for j in 0..n_v {
        for i in 0..n_u {
            let u = 2.0 * PI * i as f64 / n_u as f64;
            let v = v_min + (v_max - v_min) * j as f64 / (n_v - 1).max(1) as f64;
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

/// Triangulate a sphere face.
fn triangulate_sphere_face(face: &Face, sphere: &SphereSurface, params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let n_u = params.angular_samples;
    let n_v = (params.angular_samples / 2).max(4);

    // Determine range from boundary if available
    let boundary_3d = collect_face_boundary_points(face);
    let (u_start, u_end, v_start, v_end) = if !boundary_3d.is_empty() {
        let (u_min, u_max, v_min, v_max) = sphere_uv_range(sphere, &boundary_3d);
        let u_range = u_max - u_min;
        let v_range = v_max - v_min;
        let full_u = u_range > 1.9 * PI;
        let full_v = v_range > 0.9 * PI;
        (
            if full_u { 0.0 } else { u_min },
            if full_u { 2.0 * PI } else { u_max },
            if full_v { 0.0 } else { v_min },
            if full_v { PI } else { v_max },
        )
    } else {
        (0.0, 2.0 * PI, 0.0, PI)
    };

    // Generate vertices including poles
    for j in 0..=n_v {
        for i in 0..n_u {
            let u = u_start + (u_end - u_start) * i as f64 / n_u as f64;
            let v = v_start + (v_end - v_start) * j as f64 / n_v as f64;
            let p = sphere.point_at(u, v);
            let n = sphere.normal_at(u, v);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
        }
    }

    // Generate triangles with proper pole handling
    for j in 0..n_v {
        for i in 0..n_u {
            let i_next = (i + 1) % n_u;
            let v0 = (j * n_u + i) as u32;
            let v1 = (j * n_u + i_next) as u32;
            let v2 = ((j + 1) * n_u + i_next) as u32;
            let v3 = ((j + 1) * n_u + i) as u32;

            if j == 0 {
                // Top cap — degenerate triangles (all v0/v1 share top pole position)
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

    let boundary_3d = collect_face_boundary_points(face);
    let (u_start, u_end, v_start, v_end) = if !boundary_3d.is_empty() {
        let (u_min, u_max, v_min, v_max) = torus_uv_range(torus, &boundary_3d);
        let u_range = u_max - u_min;
        let v_range = v_max - v_min;
        (
            if u_range > 1.9 * PI { 0.0 } else { u_min },
            if u_range > 1.9 * PI { 2.0 * PI } else { u_max },
            if v_range > 1.9 * PI { 0.0 } else { v_min },
            if v_range > 1.9 * PI { 2.0 * PI } else { v_max },
        )
    } else {
        (0.0, 2.0 * PI, 0.0, 2.0 * PI)
    };

    for j in 0..n_v {
        for i in 0..n_u {
            let u = u_start + (u_end - u_start) * i as f64 / n_u as f64;
            let v = v_start + (v_end - v_start) * j as f64 / n_v as f64;
            let p = torus.point_at(u, v);
            let n = torus.normal_at(u, v);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
        }
    }

    let u_periodic = (u_end - u_start) > 1.9 * PI;
    let v_periodic = (v_end - v_start) > 1.9 * PI;

    for j in 0..n_v {
        for i in 0..n_u {
            let i_next = if u_periodic { (i + 1) % n_u } else { (i + 1).min(n_u - 1) };
            let j_next = if v_periodic { (j + 1) % n_v } else { (j + 1).min(n_v - 1) };

            if (!u_periodic && i == n_u - 1) || (!v_periodic && j == n_v - 1) {
                continue;
            }

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

/// Triangulate a revolution surface face.
fn triangulate_revolution_face(face: &Face, rev: &draper_geometry::RevolutionSurface, params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let n_u = params.angular_samples;
    let n_v = params.angular_samples;

    let (v_min, v_max) = rev.profile.param_range();

    for j in 0..n_v {
        for i in 0..n_u {
            let u = 2.0 * PI * i as f64 / n_u as f64;
            let v = v_min + (v_max - v_min) * j as f64 / (n_v - 1).max(1) as f64;
            let p = rev.point_at(u, v);
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

    let (v_min, v_max) = compute_extrusion_v_range(face, ext);
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

/// Generic surface triangulation by sampling on a grid.
/// For NURBS surfaces, uses the actual knot range.
fn triangulate_generic_surface(face: &Face, surface: &Surface, params: &TriangulationParams) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    let (base_u_min, base_u_max, base_v_min, base_v_max) = if let Surface::Nurbs(nurbs) = surface {
        let (u0, u1) = nurbs.u_range();
        let (v0, v1) = nurbs.v_range();
        (u0, u1, v0, v1)
    } else {
        (0.0, 2.0 * PI, 0.0, PI)
    };

    // If we have boundary edges, refine the parametric range
    let (u_min, u_max, v_min, v_max) = if let Some(ref wire) = face.outer_wire {
        if wire.coedges.is_empty() {
            (base_u_min, base_u_max, base_v_min, base_v_max)
        } else {
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
                let u0 = proj_u_min.max(base_u_min);
                let u1 = proj_u_max.min(base_u_max);
                let v0 = proj_v_min.max(base_v_min);
                let v1 = proj_v_max.min(base_v_max);
                if u0 < u1 && v0 < v1 {
                    let margin_u = (u1 - u0) * 0.01;
                    let margin_v = (v1 - v0) * 0.01;
                    (u0 - margin_u, u1 + margin_u, v0 - margin_v, v1 + margin_v)
                } else {
                    (base_u_min, base_u_max, base_v_min, base_v_max)
                }
            } else {
                (base_u_min, base_u_max, base_v_min, base_v_max)
            }
        }
    } else {
        (base_u_min, base_u_max, base_v_min, base_v_max)
    };

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

// ============================================================
// Boundary-aware triangulation (new API for STEP converter)
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
    triangulate_face_with_boundary_and_holes(surface, boundary_points, &[], forward, params)
}

/// Triangulate a face with boundary points and optional hole polygons.
/// For curved surfaces, uses UV-space boundary trimming with proper hole exclusion.
pub fn triangulate_face_with_boundary_and_holes(
    surface: &Surface,
    boundary_points: &[Point3d],
    hole_polylines: &[Vec<Point3d>],
    forward: bool,
    params: &TriangulationParams,
) -> TriangleMesh {
    if boundary_points.is_empty() {
        let wire = Wire::new(vec![]);
        let mut face = Face::new(surface.clone(), wire);
        face.forward = forward;
        face.edges = vec![];
        return triangulate_face(&face, params);
    }

    match surface {
        Surface::Plane(plane) => {
            // Planes: ear-clipping is correct — don't change
            triangulate_plane_with_boundary(plane, boundary_points, forward)
        }
        _ => {
            // All curved surfaces: use UV-space boundary trimming
            triangulate_surface_uv_trimmed(surface, boundary_points, hole_polylines, forward, params)
        }
    }
}

// ============================================================
// UV-space boundary trimming for curved surfaces
// ============================================================

/// Test if a 2D point is inside a closed polygon using ray casting.
fn point_in_polygon_2d(point: &Point2d, polygon: &[Point2d]) -> bool {
    let n = polygon.len();
    if n < 3 { return false; }
    let mut inside = false;
    let px = point.u;
    let py = point.v;
    let mut j = n - 1;
    for i in 0..n {
        let xi = polygon[i].u;
        let yi = polygon[i].v;
        let xj = polygon[j].u;
        let yj = polygon[j].v;
        if ((yi > py) != (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi) {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Normalize UV polygon for periodic surfaces.
/// Handles wrap-around when boundary points cross the ±π seam.
fn normalize_uv_polygon(boundary_uv: &mut [Point2d], u_period: Option<f64>, v_period: Option<f64>) {
    // Handle u-periodicity
    if let Some(period) = u_period {
        // Find the largest gap and normalize
        let mut us: Vec<f64> = boundary_uv.iter().map(|p| p.u).collect();
        us.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        // Check for wrap-around: if the range is close to the period,
        // shift values that are far from the cluster
        let u_range = us.last().unwrap() - us.first().unwrap();
        if u_range > period * 0.5 {
            // Find the largest gap — points on the other side of the gap
            // should be shifted by ±period
            let mut max_gap = 0.0f64;
            let mut gap_idx = 0;
            for i in 0..us.len() - 1 {
                let gap = us[i + 1] - us[i];
                if gap > max_gap {
                    max_gap = gap;
                    gap_idx = i;
                }
            }
            // Points after the gap should be shifted down by period
            // (they wrapped from +period to -period)
            let threshold = us[gap_idx];
            for p in boundary_uv.iter_mut() {
                if p.u > threshold + max_gap * 0.5 {
                    p.u -= period;
                }
            }
        }
    }

    // Handle v-periodicity
    if let Some(period) = v_period {
        let mut vs: Vec<f64> = boundary_uv.iter().map(|p| p.v).collect();
        vs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        let v_range = vs.last().unwrap() - vs.first().unwrap();
        if v_range > period * 0.5 {
            let mut max_gap = 0.0f64;
            let mut gap_idx = 0;
            for i in 0..vs.len() - 1 {
                let gap = vs[i + 1] - vs[i];
                if gap > max_gap {
                    max_gap = gap;
                    gap_idx = i;
                }
            }
            let threshold = vs[gap_idx];
            for p in boundary_uv.iter_mut() {
                if p.v > threshold + max_gap * 0.5 {
                    p.v -= period;
                }
            }
        }
    }
}

/// Get the period of the surface's u parameter, if periodic.
fn surface_u_period(surface: &Surface) -> Option<f64> {
    match surface {
        Surface::Cylinder(_) | Surface::Cone(_) | Surface::Sphere(_) | Surface::Torus(_) | Surface::Revolution(_) => {
            Some(2.0 * PI)
        }
        _ => None,
    }
}

/// Get the period of the surface's v parameter, if periodic.
fn surface_v_period(surface: &Surface) -> Option<f64> {
    match surface {
        Surface::Torus(_) => Some(2.0 * PI),
        _ => None,
    }
}

/// Triangulate a "cap" face on a curved surface — a disc-like face where the boundary
/// projects to a degenerate UV range (e.g., a circular disc on the end of a cylinder,
/// cone, or torus). The boundary points form a closed loop on the surface, and we
/// triangulate using a fan from the centroid of the boundary points, with each triangle's
/// third vertex evaluated on the surface.
fn triangulate_cap_face(
    surface: &Surface,
    boundary_points_3d: &[Point3d],
    forward: bool,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    if boundary_points_3d.len() < 3 {
        return mesh;
    }

    // Compute centroid of boundary points
    let n_pts = boundary_points_3d.len() as f64;
    let centroid = Point3d::new(
        boundary_points_3d.iter().map(|p| p.x).sum::<f64>() / n_pts,
        boundary_points_3d.iter().map(|p| p.y).sum::<f64>() / n_pts,
        boundary_points_3d.iter().map(|p| p.z).sum::<f64>() / n_pts,
    );

    // Project centroid onto the surface to get a more accurate center
    let (cu, cv) = surface.project_point(&centroid);
    let center_3d = surface.point_at(cu, cv);
    let center_normal = surface.normal_at(cu, cv);

    // Add centroid as first vertex
    let center_idx = mesh.add_vertex(center_3d);
    mesh.add_vertex_normal(center_idx, [center_normal.x, center_normal.y, center_normal.z]);

    // Add boundary vertices
    for p in boundary_points_3d {
        let (u, v) = surface.project_point(p);
        let n = surface.normal_at(u, v);
        let idx = mesh.add_vertex(*p);
        mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
    }

    // Fan triangulation: center → boundary[i] → boundary[(i+1) % n]
    let n = boundary_points_3d.len() as u32;
    for i in 0..n {
        let i_next = (i + 1) % n;
        let v1 = center_idx + 1 + i;
        let v2 = center_idx + 1 + i_next;
        if forward {
            mesh.add_triangle(center_idx, v1, v2);
        } else {
            mesh.add_triangle(center_idx, v2, v1);
        }
    }

    mesh
}

/// Triangulate a curved surface with boundary trimming in UV space.
///
/// Algorithm:
/// 1. Project boundary points to UV space → UV polygon
/// 2. Normalize UV polygon for periodic surfaces
/// 3. Compute UV bounding box from the polygon
/// 4. Create a UV grid inside the bounding box
/// 5. For each grid cell, test if the center is inside the UV polygon
/// 6. Generate triangles only for inside cells
/// 7. Add boundary vertices and create boundary strip triangles
fn triangulate_surface_uv_trimmed(
    surface: &Surface,
    boundary_points_3d: &[Point3d],
    hole_polylines_3d: &[Vec<Point3d>],
    forward: bool,
    params: &TriangulationParams,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    // 1. Project boundary to UV space
    let mut boundary_uv: Vec<Point2d> = boundary_points_3d.iter()
        .map(|p| {
            let (u, v) = surface.project_point(p);
            Point2d::new(u, v)
        })
        .collect();

    if boundary_uv.len() < 3 {
        return mesh;
    }

    // Also project hole polylines to UV
    let mut hole_uv_polylines: Vec<Vec<Point2d>> = hole_polylines_3d.iter().map(|hole| {
        hole.iter().map(|p| {
            let (u, v) = surface.project_point(p);
            Point2d::new(u, v)
        }).collect()
    }).collect();

    // 2. Normalize UV polygon for periodic surfaces
    let u_period = surface_u_period(surface);
    let v_period = surface_v_period(surface);
    normalize_uv_polygon(&mut boundary_uv, u_period, v_period);
    // Also normalize hole polygons
    for hole_uv in hole_uv_polylines.iter_mut() {
        normalize_uv_polygon(hole_uv, u_period, v_period);
    }

    // 3. Compute UV bounding box from BOTH outer and inner boundary points
    let mut u_min = f64::MAX; let mut u_max = f64::MIN;
    let mut v_min = f64::MAX; let mut v_max = f64::MIN;
    for p in &boundary_uv {
        u_min = u_min.min(p.u); u_max = u_max.max(p.u);
        v_min = v_min.min(p.v); v_max = v_max.max(p.v);
    }
    for hole_uv in &hole_uv_polylines {
        for p in hole_uv {
            u_min = u_min.min(p.u); u_max = u_max.max(p.u);
            v_min = v_min.min(p.v); v_max = v_max.max(p.v);
        }
    }

    let u_range = u_max - u_min;
    let v_range = v_max - v_min;

    // Handle degenerate UV range — this occurs when the boundary is a "cap" face
    // (a disc on a cylinder/cone/torus where all boundary points project to the
    // same v value). In this case, triangulate as a disc (fan from centroid).
    if u_range < 1e-12 && v_range < 1e-12 {
        // Both ranges degenerate — shouldn't happen
        return mesh;
    }

    // Check for cap faces: if the outer boundary projects to a degenerate UV range
    // (constant v for cylinders/cones, or constant u/v for torus), but holes provide
    // the missing range, we still need to treat this as a cap + annular ring.
    //
    // The key insight: if the outer boundary alone has a degenerate v_range (or u_range),
    // but with holes the full range becomes non-degenerate, the face is an annular band
    // (like the side of a tube between two concentric circles). We should handle this
    // with the UV grid approach.
    //
    // But if the outer boundary has a degenerate range AND holes don't help either
    // (holes are at the same v value), it's truly a flat cap disc.
    {
        let mut outer_u_min = f64::MAX; let mut outer_u_max = f64::MIN;
        let mut outer_v_min = f64::MAX; let mut outer_v_max = f64::MIN;
        for p in &boundary_uv {
            outer_u_min = outer_u_min.min(p.u); outer_u_max = outer_u_max.max(p.u);
            outer_v_min = outer_v_min.min(p.v); outer_v_max = outer_v_max.max(p.v);
        }
        let outer_u_range = outer_u_max - outer_u_min;
        let outer_v_range = outer_v_max - outer_v_min;

        if outer_u_range < 1e-8 && outer_v_range < 1e-8 {
            // Both ranges degenerate — completely degenerate face
            return mesh;
        }

        if outer_u_range < 1e-8 || outer_v_range < 1e-8 {
            // Outer boundary is degenerate in one direction.
            // This is a cap face (disc on cylinder/cone/torus).
            // Even if holes exist, the outer boundary is a flat circle on the surface.
            return triangulate_cap_face(surface, boundary_points_3d, forward);
        }
    }

    // Add small margin
    let margin_u = u_range * 0.001;
    let margin_v = v_range * 0.001;
    u_min -= margin_u; u_max += margin_u;
    v_min -= margin_v; v_max += margin_v;

    // 4. Determine grid resolution
    let n_u = params.angular_samples.max(16);
    let n_v = params.height_samples.max(4);
    let du = (u_max - u_min) / n_u as f64;
    let dv = (v_max - v_min) / n_v as f64;

    // 5. For each grid cell, check if center is inside outer polygon AND NOT inside any hole
    let mut inside = vec![vec![false; n_u]; n_v];
    let mut inside_count = 0usize;
    for j in 0..n_v {
        for i in 0..n_u {
            let cu = u_min + du * (i as f64 + 0.5);
            let cv = v_min + dv * (j as f64 + 0.5);
            let pt = Point2d::new(cu, cv);
            let in_outer = point_in_polygon_2d(&pt, &boundary_uv);
            if !in_outer {
                inside[j][i] = false;
                continue;
            }
            // Check if inside any hole — if so, exclude
            let mut in_hole = false;
            for hole_uv in &hole_uv_polylines {
                if hole_uv.len() >= 3 && point_in_polygon_2d(&pt, hole_uv) {
                    in_hole = true;
                    break;
                }
            }
            inside[j][i] = !in_hole;
            if inside[j][i] { inside_count += 1; }
        }
    }

    if inside_count == 0 {
        // No cells inside the polygon — this likely means the boundary projects
        // to a degenerate shape (a line) in UV space. Fall back to cap face triangulation.
        return triangulate_cap_face(surface, boundary_points_3d, forward);
    }

    // 6. Generate grid vertices (only for cells that are inside or adjacent to inside)
    // Vertex grid is (n_u+1) x (n_v+1)
    let mut vertex_index = vec![vec![None::<u32>; n_u + 1]; n_v + 1];

    for j in 0..=n_v {
        for i in 0..=n_u {
            // Check if this vertex is needed (adjacent to any inside cell)
            let mut needed = false;
            for dj in 0..2usize {
                for di in 0..2usize {
                    let ci = if di == 0 { i.checked_sub(1) } else { if i < n_u { Some(i) } else { None } };
                    let cj = if dj == 0 { j.checked_sub(1) } else { if j < n_v { Some(j) } else { None } };
                    if let (Some(ci), Some(cj)) = (ci, cj) {
                        if inside[cj][ci] { needed = true; }
                    }
                }
            }

            if needed {
                let u = u_min + du * i as f64;
                let v = v_min + dv * j as f64;
                let p3d = surface.point_at(u, v);
                let normal = surface.normal_at(u, v);
                let idx = mesh.add_vertex(p3d);
                mesh.add_vertex_normal(idx, [normal.x, normal.y, normal.z]);
                vertex_index[j][i] = Some(idx);
            }
        }
    }

    // 7. Generate triangles for inside cells
    for j in 0..n_v {
        for i in 0..n_u {
            if !inside[j][i] { continue; }

            let v00 = vertex_index[j][i];
            let v10 = vertex_index[j][i + 1];
            let v01 = vertex_index[j + 1][i];
            let v11 = vertex_index[j + 1][i + 1];

            // Need at least 3 vertices to form a triangle
            match (v00, v10, v01, v11) {
                (Some(i00), Some(i10), Some(i01), Some(i11)) => {
                    // Full quad
                    if forward {
                        mesh.add_triangle(i00, i10, i11);
                        mesh.add_triangle(i00, i11, i01);
                    } else {
                        mesh.add_triangle(i00, i11, i10);
                        mesh.add_triangle(i00, i01, i11);
                    }
                }
                (Some(i00), Some(i10), Some(i01), None) => {
                    if forward {
                        mesh.add_triangle(i00, i10, i01);
                    } else {
                        mesh.add_triangle(i00, i01, i10);
                    }
                }
                (Some(i00), Some(i10), None, Some(i11)) => {
                    if forward {
                        mesh.add_triangle(i00, i10, i11);
                    } else {
                        mesh.add_triangle(i00, i11, i10);
                    }
                }
                (Some(i00), None, Some(i01), Some(i11)) => {
                    if forward {
                        mesh.add_triangle(i00, i01, i11);
                    } else {
                        mesh.add_triangle(i00, i11, i01);
                    }
                }
                (None, Some(i10), Some(i01), Some(i11)) => {
                    if forward {
                        mesh.add_triangle(i10, i11, i01);
                    } else {
                        mesh.add_triangle(i10, i01, i11);
                    }
                }
                _ => {
                    // Fewer than 3 vertices available — skip this cell
                }
            }
        }
    }

    // 8. Add boundary vertices and create boundary strip triangles
    // For each boundary vertex, add it to the mesh and then create
    // triangles connecting it to its neighbors and the nearest grid vertex
    let n_boundary = boundary_points_3d.len();
    if n_boundary < 3 {
        return mesh;
    }

    let boundary_start = mesh.vertices.len() as u32;
    for (idx_3d, p) in boundary_points_3d.iter().enumerate() {
        let (u, v) = surface.project_point(p);
        // Use normalized UV for finding nearest grid vertex
        let mut nu = u;
        let mut nv = v;
        // Apply the same normalization as the boundary_uv
        if let Some(period) = u_period {
            let threshold_u = (u_min + u_max) * 0.5;
            if nu > threshold_u + (u_max - u_min) * 0.5 {
                nu -= period;
            } else if nu < threshold_u - (u_max - u_min) * 0.5 {
                nu += period;
            }
        }
        if let Some(period) = v_period {
            let threshold_v = (v_min + v_max) * 0.5;
            if nv > threshold_v + (v_max - v_min) * 0.5 {
                nv -= period;
            } else if nv < threshold_v - (v_max - v_min) * 0.5 {
                nv += period;
            }
        }

        let normal = surface.normal_at(u, v);
        let vidx = mesh.add_vertex(*p);
        mesh.add_vertex_normal(vidx, [normal.x, normal.y, normal.z]);

        // Find nearest grid vertex that is inside the polygon
        let gi_f = ((nu - u_min) / du).round() as isize;
        let gj_f = ((nv - v_min) / dv).round() as isize;

        // Search in expanding radius for an inside grid vertex
        let mut best_grid: Option<u32> = None;
        let mut best_dist = f64::MAX;
        let search_radius = 3isize;
        for dj in -search_radius..=search_radius {
            for di in -search_radius..=search_radius {
                let gi = gi_f + di;
                let gj = gj_f + dj;
                if gi < 0 || gj < 0 { continue; }
                let gi = gi as usize;
                let gj = gj as usize;
                if gi > n_u || gj > n_v { continue; }
                if let Some(vidx_grid) = vertex_index[gj][gi] {
                    let gp = &mesh.vertices[vidx_grid as usize];
                    let dx = gp.x - p.x;
                    let dy = gp.y - p.y;
                    let dz = gp.z - p.z;
                    let dist = dx * dx + dy * dy + dz * dz;
                    if dist < best_dist {
                        best_dist = dist;
                        best_grid = Some(vidx_grid);
                    }
                }
            }
        }

        // Create triangle: boundary[idx] → boundary[idx+1] → nearest_grid
        if let Some(grid_idx) = best_grid {
            let next_boundary_idx = boundary_start + ((idx_3d as u32 + 1) % n_boundary as u32);
            let cur_boundary_idx = boundary_start + idx_3d as u32;
            if forward {
                mesh.add_triangle(cur_boundary_idx, next_boundary_idx, grid_idx);
            } else {
                mesh.add_triangle(cur_boundary_idx, grid_idx, next_boundary_idx);
            }
        }
    }

    mesh
}

/// Triangulate a plane face with boundary points — minimum triangles.
fn triangulate_plane_with_boundary(
    plane: &Plane,
    boundary_points: &[Point3d],
    forward: bool,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    if boundary_points.len() < 3 {
        return mesh;
    }

    let points_2d: Vec<Point2d> = boundary_points.iter().map(|p| {
        let dx = p.x - plane.origin.x;
        let dy = p.y - plane.origin.y;
        let dz = p.z - plane.origin.z;
        Point2d::new(
            dx * plane.u_dir.x + dy * plane.u_dir.y + dz * plane.u_dir.z,
            dx * plane.v_dir.x + dy * plane.v_dir.y + dz * plane.v_dir.z,
        )
    }).collect();

    let is_convex = is_convex_polygon(&points_2d);

    if is_convex && boundary_points.len() >= 3 {
        // Fan triangulation — N-2 triangles (minimum for convex polygon)
        for p in boundary_points {
            mesh.add_vertex(*p);
        }
        let n = boundary_points.len() as u32;
        for i in 1..n - 1 {
            if forward {
                mesh.add_triangle(0, i, i + 1);
            } else {
                mesh.add_triangle(0, i + 1, i);
            }
        }
    } else {
        let triangles = ear_clip(&points_2d);
        for p in boundary_points {
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

    let normal = if forward {
        plane.normal
    } else {
        Direction3d::new(-plane.normal.x, -plane.normal.y, -plane.normal.z).unwrap_or(Direction3d::Z)
    };
    mesh.face_normals = Some(vec![[normal.x, normal.y, normal.z]; mesh.triangles.len()]);
    mesh
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

    let u_range = u_max - u_min;
    let full_circle = u_range > 1.9 * PI;
    let u_start = if full_circle { 0.0 } else { u_min };
    let u_end = if full_circle { 2.0 * PI } else { u_max };

    for j in 0..n_v {
        for i in 0..n_u {
            let u = u_start + (u_end - u_start) * i as f64 / n_u as f64;
            let v = v_min + (v_max - v_min) * j as f64 / (n_v - 1).max(1) as f64;
            let p = cyl.point_at(u, v);
            let n = cyl.normal_at(u, v);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
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
            let v = v_min + (v_max - v_min) * j as f64 / (n_v - 1).max(1) as f64;
            let p = cone.point_at(u, v);
            let n = cone.normal_at(u, v);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
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
            let n = torus.normal_at(u, v);
            let idx = mesh.add_vertex(p);
            mesh.add_vertex_normal(idx, [n.x, n.y, n.z]);
        }
    }

    let u_periodic = full_u;
    let v_periodic = full_v;

    for j in 0..n_v {
        for i in 0..n_u {
            let i_next = if u_periodic { (i + 1) % n_u } else { (i + 1).min(n_u - 1) };
            let j_next = if v_periodic { (j + 1) % n_v } else { (j + 1).min(n_v - 1) };

            if (!u_periodic && i == n_u - 1) || (!v_periodic && j == n_v - 1) {
                continue;
            }

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
fn triangulate_generic_with_boundary(
    surface: &Surface,
    boundary_points: &[Point3d],
    forward: bool,
    params: &TriangulationParams,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    let (base_u_min, base_u_max, base_v_min, base_v_max) = if let Surface::Nurbs(nurbs) = surface {
        let (u0, u1) = nurbs.u_range();
        let (v0, v1) = nurbs.v_range();
        (u0, u1, v0, v1)
    } else {
        (0.0, 2.0 * PI, 0.0, PI)
    };

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

    u_min = u_min.max(base_u_min);
    u_max = u_max.min(base_u_max);
    v_min = v_min.max(base_v_min);
    v_max = v_max.min(base_v_max);

    if u_min >= u_max || v_min >= v_max {
        u_min = base_u_min;
        u_max = base_u_max;
        v_min = base_v_min;
        v_max = base_v_max;
    }

    let u_margin = (u_max - u_min) * 0.01;
    let v_margin = (v_max - v_min) * 0.01;
    u_min = (u_min - u_margin).max(base_u_min);
    u_max = (u_max + u_margin).min(base_u_max);
    v_min = (v_min - v_margin).max(base_v_min);
    v_max = (v_max + v_margin).min(base_v_max);

    let n_u = if let Surface::Nurbs(_) = surface { params.angular_samples.max(24) } else { params.angular_samples };
    let n_v = if let Surface::Nurbs(_) = surface { params.angular_samples.max(24) } else { params.angular_samples };

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

// ============================================================
// UV range computation for boundary-aware trimming
// ============================================================

/// Compute parametric (u, v) range from boundary points for a cylinder.
fn cylinder_uv_range(cyl: &CylinderSurface, boundary_points: &[Point3d]) -> (f64, f64, f64, f64) {
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;
    let mut angles: Vec<f64> = Vec::with_capacity(boundary_points.len());
    for p in boundary_points {
        let (u, v) = cyl.project_point(p);
        angles.push(u);
        v_min = v_min.min(v);
        v_max = v_max.max(v);
    }
    let (u_min, u_max) = compute_angular_range(&angles);
    let u_margin = (u_max - u_min) * 0.001;
    let v_margin = (v_max - v_min) * 0.001;
    (u_min - u_margin, u_max + u_margin, v_min - v_margin, v_max + v_margin)
}

/// Compute parametric (u, v) range from boundary points for a cone.
fn cone_uv_range(cone: &ConeSurface, boundary_points: &[Point3d]) -> (f64, f64, f64, f64) {
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;
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

/// Compute parametric (u, v) range from boundary points for a sphere.
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

/// Compute parametric (u, v) range from boundary points for a torus.
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

/// Compute the angular range from a list of angles, handling ±π wraparound.
fn compute_angular_range(angles: &[f64]) -> (f64, f64) {
    if angles.is_empty() {
        return (0.0, 2.0 * PI);
    }
    if angles.len() == 1 {
        return (angles[0], angles[0] + 2.0 * PI);
    }

    let mut normalized: Vec<f64> = angles.iter()
        .map(|a| ((a % (2.0 * PI)) + 2.0 * PI) % (2.0 * PI))
        .collect();
    normalized.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

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

    let start_angle = normalized[gap_end_idx % n];
    let end_angle = if gap_end_idx == 0 {
        normalized[n - 1]
    } else if gap_end_idx < n {
        normalized[gap_end_idx - 1]
    } else {
        normalized[n - 1]
    };

    let range = if gap_end_idx == 0 {
        end_angle - start_angle + 2.0 * PI
    } else {
        end_angle - start_angle
    };

    if range > 1.99 * PI {
        (0.0, 2.0 * PI)
    } else {
        (start_angle, start_angle + range)
    }
}

// ============================================================
// V-range estimation for axis-based surfaces
// ============================================================

/// Estimate the v parameter range for a face by sampling its edges
/// and projecting sample points onto the surface's axis direction.
fn estimate_v_range(face: &Face) -> Option<(f64, f64)> {
    if let Some(ref surface) = face.surface {
        match surface {
            Surface::Cylinder(cyl) => {
                let (v_min, v_max) = compute_axis_v_range(face, &cyl.origin, &cyl.axis);
                if v_min < v_max { Some((v_min, v_max)) } else { Some((0.0, 100.0)) }
            }
            Surface::Cone(cone) => {
                let (v_min, v_max) = compute_axis_v_range(face, &cone.origin, &cone.axis);
                if v_min < v_max { Some((v_min, v_max)) } else { Some((0.0, cone.height().min(100.0))) }
            }
            Surface::Revolution(rev) => Some(rev.profile.param_range()),
            Surface::Extrusion(ext) => Some(ext.profile.param_range()),
            _ => Some((0.0, 100.0)),
        }
    } else {
        None
    }
}

/// Compute the v parameter range for axis-based surfaces (Cylinder, Cone).
fn compute_axis_v_range(face: &Face, origin: &Point3d, axis: &Direction3d) -> (f64, f64) {
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;

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
        (0.0, 100.0)
    } else {
        let margin = (v_max - v_min) * 0.001;
        (v_min - margin, v_max + margin)
    }
}

/// Compute the v (extrusion) parameter range for an extrusion surface.
fn compute_extrusion_v_range(face: &Face, ext: &draper_geometry::ExtrusionSurface) -> (f64, f64) {
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;

    for edge in &face.edges {
        for i in 0..64 {
            let t = i as f64 / 63.0;
            if let Some(p) = edge.point_at(t) {
                let origin = ext.profile.point_at(0.0);
                let v = (p.x - origin.x) * ext.direction.x
                      + (p.y - origin.y) * ext.direction.y
                      + (p.z - origin.z) * ext.direction.z;
                v_min = v_min.min(v);
                v_max = v_max.max(v);
            }
        }
    }

    if v_min >= v_max {
        if let Some(ref wire) = face.outer_wire {
            for coedge in &wire.coedges {
                let edge = face.edges.iter().find(|e| e.id == coedge.edge);
                if let Some(edge) = edge {
                    for i in 0..64 {
                        let t = i as f64 / 63.0;
                        let t_actual = if coedge.forward { t } else { 1.0 - t };
                        if let Some(p) = edge.point_at(t_actual) {
                            let origin = ext.profile.point_at(0.0);
                            let v = (p.x - origin.x) * ext.direction.x
                                  + (p.y - origin.y) * ext.direction.y
                                  + (p.z - origin.z) * ext.direction.z;
                            v_min = v_min.min(v);
                            v_max = v_max.max(v);
                        }
                    }
                }
            }
        }
    }

    if v_min >= v_max { (0.0, 1.0) } else {
        let margin = (v_max - v_min) * 0.001;
        (v_min - margin, v_max + margin)
    }
}

// ============================================================
// Boundary ring snapping for curved surfaces
// ============================================================

/// Snap boundary ring vertices of a curved surface mesh to edge curve samples.
/// This is a generic helper that works for any axis-based surface (cylinder, cone).
fn snap_boundary_rings(
    mesh: &mut TriangleMesh,
    boundary_3d: &[Point3d],
    surface: &Surface,
    n_u: usize,
    n_v: usize,
    u_start: f64,
    u_end: f64,
    v_min: f64,
    v_max: f64,
    full_circle: bool,
) {
    // For each boundary ring (top and bottom rows), find edge curve points
    // at the corresponding v values and snap grid vertices to them
    for (row_j, target_v) in [(0, v_min), (n_v - 1, v_max)] {
        let v_tol = (v_max - v_min) * 0.01 + 1e-6;

        // Collect boundary points near this v value
        let mut ring_pts: Vec<(f64, Point3d)> = Vec::new();
        for p in boundary_3d {
            if let Some((u, v)) = surface.project_point_opt(p) {
                if (v - target_v).abs() < v_tol {
                    ring_pts.push((u, *p));
                }
            }
        }

        if ring_pts.is_empty() {
            continue;
        }

        // For each grid vertex in this row, find the closest boundary point
        let offset = row_j * n_u;
        for i in 0..n_u {
            let u = u_start + (u_end - u_start) * i as f64 / n_u as f64;
            let mut best_dist = f64::MAX;
            let mut best_pt = None;
            for (ru, rp) in &ring_pts {
                let du = if full_circle {
                    let diff = (u - ru).abs();
                    diff.min(2.0 * PI - diff)
                } else {
                    (u - ru).abs()
                };
                if du < best_dist {
                    best_dist = du;
                    best_pt = Some(*rp);
                }
            }
            // Only snap if the boundary point is very close in angle
            let angle_tol = (u_end - u_start) / n_u as f64 * 0.5;
            if best_dist < angle_tol {
                if let Some(pt) = best_pt {
                    mesh.vertices[offset + i] = pt;
                }
            }
        }
    }
}

// ============================================================
// Ear clipping triangulation
// ============================================================

/// Ear clipping triangulation of a 2D polygon.
/// Returns triangle indices into the original point array.
/// Produces N-2 triangles for a simple polygon with N vertices (minimum for convex).
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

            let cross = (pb.u - pa.u) * (pc.v - pa.v) - (pb.v - pa.v) * (pc.u - pa.u);
            let is_convex = if ccw { cross > 0.0 } else { cross < 0.0 };

            if !is_convex {
                continue;
            }

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
            // Degenerate polygon — fan triangulate as fallback
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

/// Signed area of triangle (p, a, b) * 2.
fn sign_area_2d(p: &Point2d, a: &Point2d, b: &Point2d) -> f64 {
    (p.u - b.u) * (a.v - b.v) - (a.u - b.u) * (p.v - b.v)
}

/// Check if a 2D polygon is convex.
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
            if sign == 0 { sign = s; } else if sign != s { return false; }
        }
    }
    sign != 0
}

// ============================================================
// Vertex merging for watertight solids
// ============================================================

/// Merge coincident vertices in a mesh within the given tolerance.
/// This makes closed solids watertight by ensuring that shared edge
/// vertices between adjacent faces use the same vertex index.
pub fn merge_coincident_vertices(mesh: &mut TriangleMesh, tolerance: f64) {
    if mesh.vertices.is_empty() {
        return;
    }

    let tol_sq = tolerance * tolerance;
    let n = mesh.vertices.len();

    let mut remap: Vec<u32> = vec![0; n];
    let mut new_vertices: Vec<Point3d> = Vec::with_capacity(n);

    let cell_size = tolerance * 10.0;
    let mut grid: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();

    for (i, v) in mesh.vertices.iter().enumerate() {
        let cx = (v.x / cell_size).floor() as i64;
        let cy = (v.y / cell_size).floor() as i64;
        let cz = (v.z / cell_size).floor() as i64;

        let mut found = false;
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
                            if ddx * ddx + ddy * ddy + ddz * ddz < tol_sq {
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
        if a != b && b != c && a != c {
            mesh.triangles.push([a, b, c]);
        }
    }

    mesh.vertices = new_vertices;

    // Rebuild normals
    if mesh.normals.is_some() {
        mesh.normals = None;
    }
}
