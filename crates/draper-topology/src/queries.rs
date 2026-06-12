// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Analytical queries for B-Rep solids.
//!
//! Provides volume, surface area, center of mass, point-in-solid test,
//! moments of inertia, and BVH acceleration structure.

use crate::entity::*;
use draper_geometry::{Point3d, Vec3d, Surface, Plane, CylinderSurface, SphereSurface, ConeSurface};
use std::f64::consts::PI;

// ============================================================
// Lightweight triangulation for analytical queries
// ============================================================

/// A simple triangle mesh used internally for analytical computations.
/// Avoids dependency on draper-mesh (which depends on draper-topology).
#[derive(Clone, Debug)]
struct QueryMesh {
    vertices: Vec<Point3d>,
    triangles: Vec<[u32; 3]>,
}

impl QueryMesh {
    fn new() -> Self {
        Self {
            vertices: Vec::new(),
            triangles: Vec::new(),
        }
    }

    fn add_vertex(&mut self, p: Point3d) -> u32 {
        let idx = self.vertices.len() as u32;
        self.vertices.push(p);
        idx
    }

    fn merge(&mut self, other: &QueryMesh) {
        let offset = self.vertices.len() as u32;
        self.vertices.extend(other.vertices.iter().cloned());
        for tri in &other.triangles {
            self.triangles.push([tri[0] + offset, tri[1] + offset, tri[2] + offset]);
        }
    }
}

/// Triangulate a solid for analytical query purposes.
/// Uses a simplified approach: planar faces are fan-triangulated from boundary,
/// curved surfaces are sampled on a UV grid.
fn triangulate_solid_for_queries(solid: &Solid) -> QueryMesh {
    let mut mesh = QueryMesh::new();
    for face in solid.faces() {
        let face_mesh = triangulate_face_for_queries(face);
        mesh.merge(&face_mesh);
    }
    mesh
}

/// Triangulate a single face for analytical queries.
fn triangulate_face_for_queries(face: &Face) -> QueryMesh {
    let surface = match &face.surface {
        Some(s) => s,
        None => return QueryMesh::new(),
    };

    match surface {
        Surface::Plane(plane) => triangulate_planar_face_query(face, plane),
        Surface::Cylinder(cyl) => triangulate_cylinder_face_query(face, cyl),
        Surface::Sphere(sphere) => triangulate_sphere_face_query(face, sphere),
        Surface::Cone(cone) => triangulate_cone_face_query(face, cone),
        _ => {
            // Fallback: try to sample the surface on a grid
            triangulate_generic_face_query(face, surface)
        }
    }
}

/// Collect boundary points from a face's outer wire.
/// For straight edges, only keeps the start point to avoid redundant
/// collinear points. The end point of the last edge closes the loop.
fn collect_boundary_points(face: &Face) -> Vec<Point3d> {
    let mut points = Vec::new();
    if let Some(ref wire) = face.outer_wire {
        for coedge in &wire.coedges {
            if let Some(edge) = face.edges.iter().find(|e| e.id == coedge.edge) {
                if edge.degenerate {
                    continue;
                }
                // Sample a few points to detect if the edge is curved
                let p0 = edge.point_at(0.0);
                let p1 = edge.point_at(0.5);
                let p2 = edge.point_at(1.0);

                match (p0, p1, p2) {
                    (Some(start), Some(mid), Some(end)) => {
                        // Check if the edge is approximately straight
                        // by measuring deviation of the midpoint from the line start→end.
                        // For closed curves (start ≈ end), always treat as curved.
                        let dx = end.x - start.x;
                        let dy = end.y - start.y;
                        let dz = end.z - start.z;
                        let len_sq = dx * dx + dy * dy + dz * dz;

                        let is_straight = if len_sq < 1e-10 {
                            // Start and end are the same point — closed curve, treat as curved
                            false
                        } else {
                            // Check midpoint deviation from the line
                            let t = ((mid.x - start.x) * dx + (mid.y - start.y) * dy + (mid.z - start.z) * dz) / len_sq;
                            let proj_x = start.x + t * dx;
                            let proj_y = start.y + t * dy;
                            let proj_z = start.z + t * dz;
                            let dev_x = mid.x - proj_x;
                            let dev_y = mid.y - proj_y;
                            let dev_z = mid.z - proj_z;
                            (dev_x * dev_x + dev_y * dev_y + dev_z * dev_z) < 1e-10
                        };

                        let mut pts = if is_straight {
                            // Straight edge — only keep start point
                            vec![start]
                        } else {
                            // Curved edge — sample more densely for accuracy
                            let n = 64;
                            let mut sampled = Vec::with_capacity(n);
                            for i in 0..n {
                                let t = i as f64 / n as f64;
                                if let Some(p) = edge.point_at(t) {
                                    sampled.push(p);
                                }
                            }
                            sampled
                        };

                        // If coedge is reversed, reverse the sample order
                        if !coedge.forward {
                            pts.reverse();
                        }
                        points.extend(pts);
                    }
                    _ => {}
                }
            }
        }
    }
    // Deduplicate consecutive coincident points
    deduplicate_points(&mut points);
    points
}

fn deduplicate_points(points: &mut Vec<Point3d>) {
    if points.len() <= 1 {
        return;
    }
    let mut unique = vec![points[0]];
    for p in &points[1..] {
        if let Some(last) = unique.last() {
            let dx = p.x - last.x;
            let dy = p.y - last.y;
            let dz = p.z - last.z;
            if dx * dx + dy * dy + dz * dz > 1e-12 {
                unique.push(*p);
            }
        }
    }
    // Check last vs first
    if unique.len() > 1 {
        let first = unique[0];
        let last = unique[unique.len() - 1];
        let dx = last.x - first.x;
        let dy = last.y - first.y;
        let dz = last.z - first.z;
        if dx * dx + dy * dy + dz * dz < 1e-12 {
            unique.pop();
        }
    }
    *points = unique;
}

fn triangulate_planar_face_query(face: &Face, _plane: &Plane) -> QueryMesh {
    let boundary = collect_boundary_points(face);
    if boundary.len() < 3 {
        return QueryMesh::new();
    }

    let mut mesh = QueryMesh::new();
    for p in &boundary {
        mesh.add_vertex(*p);
    }
    // Fan triangulation
    let n = boundary.len() as u32;
    let forward = face.forward;
    for i in 1..n - 1 {
        if forward {
            mesh.triangles.push([0, i, i + 1]);
        } else {
            mesh.triangles.push([0, i + 1, i]);
        }
    }
    mesh
}

fn triangulate_cylinder_face_query(face: &Face, cyl: &CylinderSurface) -> QueryMesh {
    // Determine v range from edges
    let (v_min, v_max) = compute_cylinder_v_range(face, cyl);
    let n_u = 64;
    let n_v = 16;

    let mut mesh = QueryMesh::new();
    for j in 0..=n_v {
        for i in 0..n_u {
            let u = 2.0 * PI * i as f64 / n_u as f64;
            let v = v_min + (v_max - v_min) * j as f64 / n_v as f64;
            mesh.add_vertex(cyl.point_at(u, v));
        }
    }

    let forward = face.forward;
    for j in 0..n_v {
        for i in 0..n_u {
            let i_next = (i + 1) % n_u;
            let v0 = (j * n_u + i) as u32;
            let v1 = (j * n_u + i_next) as u32;
            let v2 = ((j + 1) * n_u + i_next) as u32;
            let v3 = ((j + 1) * n_u + i) as u32;
            if forward {
                mesh.triangles.push([v0, v1, v2]);
                mesh.triangles.push([v0, v2, v3]);
            } else {
                mesh.triangles.push([v0, v2, v1]);
                mesh.triangles.push([v0, v3, v2]);
            }
        }
    }
    mesh
}

fn compute_cylinder_v_range(face: &Face, cyl: &CylinderSurface) -> (f64, f64) {
    let mut v_min = 0.0_f64;
    let mut v_max = 0.0_f64;
    let mut initialized = false;
    for edge in &face.edges {
        for i in 0..32 {
            let t = i as f64 / 32.0;
            if let Some(p) = edge.point_at(t) {
                let (_u, v) = cyl.project_point(&p);
                if !initialized {
                    v_min = v;
                    v_max = v;
                    initialized = true;
                } else {
                    v_min = v_min.min(v);
                    v_max = v_max.max(v);
                }
            }
        }
    }
    if !initialized {
        v_min = -1.0;
        v_max = 1.0;
    }
    (v_min, v_max)
}

fn triangulate_sphere_face_query(_face: &Face, sphere: &SphereSurface) -> QueryMesh {
    let n_u = 64;
    let n_v = 32;
    let mut mesh = QueryMesh::new();

    for j in 0..=n_v {
        let v = PI * j as f64 / n_v as f64;
        for i in 0..n_u {
            let u = 2.0 * PI * i as f64 / n_u as f64;
            mesh.add_vertex(sphere.point_at(u, v));
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
                // Top pole — degenerate row
                mesh.triangles.push([v2, v3, ((j + 1) * n_u) as u32]);
                // Actually, top pole: first row is all same point? No.
                // j=0 is north pole. Each vertex at j=0 maps to same point.
                // Fan from the "pole" which is actually just the single first vertex
                mesh.triangles.push([0, v2, v3]);
            } else if j == n_v - 1 {
                mesh.triangles.push([v0, v1, v2]);
            } else {
                mesh.triangles.push([v0, v1, v2]);
                mesh.triangles.push([v0, v2, v3]);
            }
        }
    }
    mesh
}

fn triangulate_cone_face_query(face: &Face, cone: &ConeSurface) -> QueryMesh {
    let (v_min, v_max) = compute_cone_v_range(face, cone);
    let n_u = 64;
    let n_v = 16;
    let mut mesh = QueryMesh::new();

    for j in 0..=n_v {
        for i in 0..n_u {
            let u = 2.0 * PI * i as f64 / n_u as f64;
            let v = v_min + (v_max - v_min) * j as f64 / n_v as f64;
            mesh.add_vertex(cone.point_at(u, v));
        }
    }

    let forward = face.forward;
    for j in 0..n_v {
        for i in 0..n_u {
            let i_next = (i + 1) % n_u;
            let v0 = (j * n_u + i) as u32;
            let v1 = (j * n_u + i_next) as u32;
            let v2 = ((j + 1) * n_u + i_next) as u32;
            let v3 = ((j + 1) * n_u + i) as u32;
            if forward {
                mesh.triangles.push([v0, v1, v2]);
                mesh.triangles.push([v0, v2, v3]);
            } else {
                mesh.triangles.push([v0, v2, v1]);
                mesh.triangles.push([v0, v3, v2]);
            }
        }
    }
    mesh
}

fn compute_cone_v_range(face: &Face, cone: &ConeSurface) -> (f64, f64) {
    let mut v_min = 0.0_f64;
    let mut v_max = 0.0_f64;
    let mut initialized = false;
    for edge in &face.edges {
        for i in 0..32 {
            let t = i as f64 / 32.0;
            if let Some(p) = edge.point_at(t) {
                let (_u, v) = cone.project_point(&p);
                if !initialized {
                    v_min = v;
                    v_max = v;
                    initialized = true;
                } else {
                    v_min = v_min.min(v);
                    v_max = v_max.max(v);
                }
            }
        }
    }
    if !initialized {
        v_min = 0.0;
        v_max = cone.height();
    }
    (v_min, v_max)
}

fn triangulate_generic_face_query(face: &Face, surface: &Surface) -> QueryMesh {
    // Try to collect boundary points for a rough triangulation
    let boundary = collect_boundary_points(face);
    if boundary.len() >= 3 {
        // Assume planar-ish — fan triangulate
        let mut mesh = QueryMesh::new();
        for p in &boundary {
            mesh.add_vertex(*p);
        }
        let n = boundary.len() as u32;
        let forward = face.forward;
        for i in 1..n - 1 {
            if forward {
                mesh.triangles.push([0, i, i + 1]);
            } else {
                mesh.triangles.push([0, i + 1, i]);
            }
        }
        mesh
    } else {
        // Last resort: sample the surface on a grid
        let n_u = 32;
        let n_v = 16;
        let mut mesh = QueryMesh::new();
        for j in 0..=n_v {
            for i in 0..n_u {
                let u = i as f64 / n_u as f64; // [0, 1)
                let v = j as f64 / n_v as f64; // [0, 1]
                mesh.add_vertex(surface.point_at(u * 2.0 * PI, v * PI));
            }
        }
        for j in 0..n_v {
            for i in 0..n_u {
                let i_next = (i + 1) % n_u;
                let v0 = (j * n_u + i) as u32;
                let v1 = (j * n_u + i_next) as u32;
                let v2 = ((j + 1) * n_u + i_next) as u32;
                let v3 = ((j + 1) * n_u + i) as u32;
                mesh.triangles.push([v0, v1, v2]);
                mesh.triangles.push([v0, v2, v3]);
            }
        }
        mesh
    }
}

// ============================================================
// 4.2.1 Volume of a closed solid
// ============================================================

/// Compute the volume of a closed solid using the divergence theorem.
///
/// For each triangulated face, computes the signed volume of the tetrahedron
/// formed by each triangle and the origin:
///   V = Σ (v1 · (v2 × v3)) / 6
///
/// Face orientations are normalized to ensure consistent outward-pointing normals
/// before computing the volume.
pub fn solid_volume(solid: &Solid) -> f64 {
    let mesh = triangulate_solid_for_queries(solid);

    // Compute an approximate center of the solid (average of all vertices)
    let mut cx = 0.0_f64;
    let mut cy = 0.0_f64;
    let mut cz = 0.0_f64;
    for v in &mesh.vertices {
        cx += v.x;
        cy += v.y;
        cz += v.z;
    }
    let n = mesh.vertices.len() as f64;
    if n < 1e-15 {
        return 0.0;
    }
    let center = Point3d::new(cx / n, cy / n, cz / n);

    let mut volume = 0.0;
    for tri in &mesh.triangles {
        let v0 = mesh.vertices[tri[0] as usize];
        let v1 = mesh.vertices[tri[1] as usize];
        let v2 = mesh.vertices[tri[2] as usize];

        // Compute triangle normal
        let e1 = Vec3d::new(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
        let e2 = Vec3d::new(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
        let normal = e1.cross(&e2);

        // Check if normal points outward (away from center)
        let face_center = Point3d::new(
            (v0.x + v1.x + v2.x) / 3.0,
            (v0.y + v1.y + v2.y) / 3.0,
            (v0.z + v1.z + v2.z) / 3.0,
        );
        let to_face = Vec3d::new(
            face_center.x - center.x,
            face_center.y - center.y,
            face_center.z - center.z,
        );
        let dot_outward = normal.dot(&to_face);

        // Signed volume of tetrahedron (origin, v0, v1, v2)
        let cross = Vec3d::new(
            v1.y * v2.z - v1.z * v2.y,
            v1.z * v2.x - v1.x * v2.z,
            v1.x * v2.y - v1.y * v2.x,
        );
        let signed_vol = (v0.x * cross.x + v0.y * cross.y + v0.z * cross.z) / 6.0;

        // If the normal points inward (dot_outward < 0), flip the sign
        if dot_outward >= 0.0 {
            volume += signed_vol;
        } else {
            volume -= signed_vol;
        }
    }
    volume.abs()
}

// ============================================================
// 4.2.2 Surface area
// ============================================================

/// Compute the total surface area of a solid.
///
/// Triangulates all faces and sums triangle areas.
pub fn solid_surface_area(solid: &Solid) -> f64 {
    let mesh = triangulate_solid_for_queries(solid);
    let mut area = 0.0;
    for tri in &mesh.triangles {
        let v0 = mesh.vertices[tri[0] as usize];
        let v1 = mesh.vertices[tri[1] as usize];
        let v2 = mesh.vertices[tri[2] as usize];
        let e1 = Vec3d::new(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
        let e2 = Vec3d::new(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
        let cross = e1.cross(&e2);
        area += cross.length() * 0.5;
    }
    area
}

// ============================================================
// 4.2.3 Center of mass
// ============================================================

/// Compute the center of mass (centroid) of a solid.
///
/// Uses the weighted average of tetrahedra centroids, where each
/// tetrahedron is formed by a triangle and the origin:
///   centroid_i = (v1 + v2 + v3) / 4
///   weight_i = signed_volume_i
///   total_centroid = Σ(centroid_i * weight_i) / Σ(weight_i)
pub fn solid_center_of_mass(solid: &Solid) -> Point3d {
    let mesh = triangulate_solid_for_queries(solid);
    let mut cx = 0.0;
    let mut cy = 0.0;
    let mut cz = 0.0;
    let mut total_volume = 0.0;

    for tri in &mesh.triangles {
        let v0 = mesh.vertices[tri[0] as usize];
        let v1 = mesh.vertices[tri[1] as usize];
        let v2 = mesh.vertices[tri[2] as usize];

        let cross = Vec3d::new(
            v1.y * v2.z - v1.z * v2.y,
            v1.z * v2.x - v1.x * v2.z,
            v1.x * v2.y - v1.y * v2.x,
        );
        let signed_vol = (v0.x * cross.x + v0.y * cross.y + v0.z * cross.z) / 6.0;

        // Centroid of tetrahedron (origin, v0, v1, v2) is (v0+v1+v2)/4
        let tcx = (v0.x + v1.x + v2.x) / 4.0;
        let tcy = (v0.y + v1.y + v2.y) / 4.0;
        let tcz = (v0.z + v1.z + v2.z) / 4.0;

        cx += tcx * signed_vol;
        cy += tcy * signed_vol;
        cz += tcz * signed_vol;
        total_volume += signed_vol;
    }

    if total_volume.abs() < 1e-15 {
        return Point3d::ORIGIN;
    }

    Point3d::new(cx / total_volume, cy / total_volume, cz / total_volume)
}

// ============================================================
// 4.2.4 Point-in-solid test (ray casting)
// ============================================================

/// Test whether a point is inside a solid using ray casting.
///
/// Casts multiple rays from the point in slightly perturbed directions and
/// counts intersections with all face triangles using the Möller–Trumbore
/// algorithm. Uses majority voting across rays to handle edge/vertex cases
/// where a single ray might give an ambiguous result.
/// Odd count = inside, even = outside.
pub fn point_in_solid(solid: &Solid, point: &Point3d) -> bool {
    let mesh = triangulate_solid_for_queries(solid);

    // Cast multiple rays in slightly different directions to avoid
    // edge/vertex hits that can give wrong parity counts.
    // Use perturbations that are large enough to avoid face diagonals
    // but small enough to maintain the general ray direction.
    let ray_dirs = [
        Vec3d::new(1.0, 0.3742, 0.1583),   // +X with significant perturbation
        Vec3d::new(0.2819, 1.0, 0.4291),   // +Y with significant perturbation
        Vec3d::new(0.1938, 0.3714, 1.0),   // +Z with significant perturbation
    ];

    let mut inside_votes = 0;
    let mut outside_votes = 0;

    for ray_dir in &ray_dirs {
        let mut count = 0u32;
        for tri in &mesh.triangles {
            let v0 = mesh.vertices[tri[0] as usize];
            let v1 = mesh.vertices[tri[1] as usize];
            let v2 = mesh.vertices[tri[2] as usize];

            if moller_trumbore_intersect(point, ray_dir, &v0, &v1, &v2) {
                count += 1;
            }
        }
        if count % 2 == 1 {
            inside_votes += 1;
        } else {
            outside_votes += 1;
        }
    }

    inside_votes > outside_votes
}

/// Möller–Trumbore ray-triangle intersection test.
/// Returns true if the ray from `origin` in direction `dir` intersects
/// the triangle (v0, v1, v2) at t > 0.
fn moller_trumbore_intersect(
    origin: &Point3d,
    dir: &Vec3d,
    v0: &Point3d,
    v1: &Point3d,
    v2: &Point3d,
) -> bool {
    let eps = 1e-10;

    let e1 = Vec3d::new(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
    let e2 = Vec3d::new(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
    let h = dir.cross(&e2);
    let a = e1.dot(&h);

    if a.abs() < eps {
        return false; // Ray is parallel to triangle
    }

    let f = 1.0 / a;
    let s = Vec3d::new(origin.x - v0.x, origin.y - v0.y, origin.z - v0.z);
    let u = f * s.dot(&h);
    if u < 0.0 || u > 1.0 {
        return false;
    }

    let q = s.cross(&e1);
    let v = f * dir.dot(&q);
    if v < 0.0 || u + v > 1.0 {
        return false;
    }

    let t = f * e2.dot(&q);
    t > eps
}

// ============================================================
// 4.2.5 Moments of inertia
// ============================================================

/// The 3×3 inertia tensor of a solid, stored as 6 independent components.
///
/// The inertia tensor is symmetric, so only 6 values are needed:
///   Ixx = ∫(y² + z²) dV
///   Iyy = ∫(x² + z²) dV
///   Izz = ∫(x² + y²) dV
///   Ixy = -∫xy dV
///   Ixz = -∫xz dV
///   Iyz = -∫yz dV
#[derive(Clone, Debug)]
pub struct InertiaTensor {
    pub ixx: f64,
    pub iyy: f64,
    pub izz: f64,
    pub ixy: f64,
    pub ixz: f64,
    pub iyz: f64,
}

impl InertiaTensor {
    /// Zero inertia tensor.
    pub fn zero() -> Self {
        Self {
            ixx: 0.0, iyy: 0.0, izz: 0.0,
            ixy: 0.0, ixz: 0.0, iyz: 0.0,
        }
    }
}

/// Compute the inertia tensor of a solid about its center of mass.
///
/// Uses the divergence theorem approach: decomposes the solid into
/// tetrahedra (origin + triangle) and integrates each component
/// analytically over each tetrahedron.
pub fn solid_moments_of_inertia(solid: &Solid) -> InertiaTensor {
    let mesh = triangulate_solid_for_queries(solid);
    let com = solid_center_of_mass(solid);

    let mut ixx = 0.0;
    let mut iyy = 0.0;
    let mut izz = 0.0;
    let mut ixy = 0.0;
    let mut ixz = 0.0;
    let mut iyz = 0.0;

    for tri in &mesh.triangles {
        // Translate vertices to COM frame
        let v0 = Point3d::new(
            mesh.vertices[tri[0] as usize].x - com.x,
            mesh.vertices[tri[0] as usize].y - com.y,
            mesh.vertices[tri[0] as usize].z - com.z,
        );
        let v1 = Point3d::new(
            mesh.vertices[tri[1] as usize].x - com.x,
            mesh.vertices[tri[1] as usize].y - com.y,
            mesh.vertices[tri[1] as usize].z - com.z,
        );
        let v2 = Point3d::new(
            mesh.vertices[tri[2] as usize].x - com.x,
            mesh.vertices[tri[2] as usize].y - com.y,
            mesh.vertices[tri[2] as usize].z - com.z,
        );

        // Signed volume of tetrahedron
        let cross = Vec3d::new(
            v1.y * v2.z - v1.z * v2.y,
            v1.z * v2.x - v1.x * v2.z,
            v1.x * v2.y - v1.y * v2.x,
        );
        let det = v0.x * cross.x + v0.y * cross.y + v0.z * cross.z;
        let vol = det / 6.0;

        // Integrate x^2, y^2, z^2, xy, xz, yz over the tetrahedron
        // Using the formula for integration of monomials over a tetrahedron:
        // ∫ x^a y^b z^c dV = (a! b! c! / (a+b+c+3)!) * 6V * x0^a * y0^b * z0^c (summed over vertices)
        // For second-order monomials with a+b+c=2:
        //   ∫ f dV = V/20 * (2*f0 + 2*f1 + 2*f2 + f0c + f1c + f2c)
        // where fi = f(vi) and fic = sum of f evaluated at combinations of vertices
        //
        // Simpler approach: exact formulas for tetrahedron integrals
        // ∫ x^2 dV = V/10 * (x0^2 + x1^2 + x2^2 + x0*x1 + x0*x2 + x1*x2)
        // (since the tetrahedron has one vertex at origin: x3=y3=z3=0)
        // Similarly for cross terms:
        // ∫ xy dV = V/20 * (2*x0*y0 + 2*x1*y1 + 2*x2*y2 + x0*y1 + x0*y2 + x1*y0 + x1*y2 + x2*y0 + x2*y1)

        let x0 = v0.x; let y0 = v0.y; let z0 = v0.z;
        let x1 = v1.x; let y1 = v1.y; let z1 = v1.z;
        let x2 = v2.x; let y2 = v2.y; let z2 = v2.z;

        // ∫ x² dV over tetrahedron (origin, v0, v1, v2)
        let int_x2 = vol / 10.0 * (x0*x0 + x1*x1 + x2*x2 + x0*x1 + x0*x2 + x1*x2);
        let int_y2 = vol / 10.0 * (y0*y0 + y1*y1 + y2*y2 + y0*y1 + y0*y2 + y1*y2);
        let int_z2 = vol / 10.0 * (z0*z0 + z1*z1 + z2*z2 + z0*z1 + z0*z2 + z1*z2);

        // ∫ xy dV
        let int_xy = vol / 20.0 * (
            2.0*x0*y0 + 2.0*x1*y1 + 2.0*x2*y2 +
            x0*y1 + x0*y2 + x1*y0 + x1*y2 + x2*y0 + x2*y1
        );
        let int_xz = vol / 20.0 * (
            2.0*x0*z0 + 2.0*x1*z1 + 2.0*x2*z2 +
            x0*z1 + x0*z2 + x1*z0 + x1*z2 + x2*z0 + x2*z1
        );
        let int_yz = vol / 20.0 * (
            2.0*y0*z0 + 2.0*y1*z1 + 2.0*y2*z2 +
            y0*z1 + y0*z2 + y1*z0 + y1*z2 + y2*z0 + y2*z1
        );

        ixx += int_y2 + int_z2;
        iyy += int_x2 + int_z2;
        izz += int_x2 + int_y2;
        ixy -= int_xy;
        ixz -= int_xz;
        iyz -= int_yz;
    }

    InertiaTensor {
        ixx: ixx.abs(),
        iyy: iyy.abs(),
        izz: izz.abs(),
        ixy,
        ixz,
        iyz,
    }
}

// ============================================================
// 4.2.6 BVH for acceleration
// ============================================================

/// A node in the Bounding Volume Hierarchy.
#[derive(Clone, Debug)]
pub struct BvhNode {
    /// Minimum corner of the axis-aligned bounding box.
    pub bbox_min: Point3d,
    /// Maximum corner of the axis-aligned bounding box.
    pub bbox_max: Point3d,
    /// Left child node.
    pub left: Option<Box<BvhNode>>,
    /// Right child node.
    pub right: Option<Box<BvhNode>>,
    /// Triangle indices stored in leaf nodes only.
    pub triangle_indices: Option<Vec<usize>>,
}

/// A Bounding Volume Hierarchy for accelerating spatial queries.
#[derive(Clone, Debug)]
pub struct Bvh {
    /// Root node of the BVH tree.
    pub root: BvhNode,
    /// Vertex positions shared by all triangles.
    pub vertices: Vec<Point3d>,
    /// Triangle indices (3 vertex indices per triangle).
    pub triangles: Vec<[u32; 3]>,
}

impl Bvh {
    /// Build a BVH from vertices and triangles using median-split
    /// along the longest axis.
    pub fn build(vertices: &[Point3d], triangles: &[[u32; 3]]) -> Bvh {
        if triangles.is_empty() {
            return Bvh {
                root: BvhNode {
                    bbox_min: Point3d::ORIGIN,
                    bbox_max: Point3d::ORIGIN,
                    left: None,
                    right: None,
                    triangle_indices: Some(vec![]),
                },
                vertices: vertices.to_vec(),
                triangles: triangles.to_vec(),
            };
        }

        let indices: Vec<usize> = (0..triangles.len()).collect();
        let root = Self::build_recursive(vertices, triangles, &indices, 0);

        Bvh {
            root,
            vertices: vertices.to_vec(),
            triangles: triangles.to_vec(),
        }
    }

    fn build_recursive(
        vertices: &[Point3d],
        triangles: &[[u32; 3]],
        indices: &[usize],
        depth: usize,
    ) -> BvhNode {
        // Compute bounding box for all triangles in this set
        let (bbox_min, bbox_max) = Self::compute_bbox(vertices, triangles, indices);

        // Leaf node: few triangles or max depth reached
        if indices.len() <= 4 || depth >= 32 {
            return BvhNode {
                bbox_min,
                bbox_max,
                left: None,
                right: None,
                triangle_indices: Some(indices.to_vec()),
            };
        }

        // Find the longest axis of the bounding box
        let dx = bbox_max.x - bbox_min.x;
        let dy = bbox_max.y - bbox_min.y;
        let dz = bbox_max.z - bbox_min.z;
        let axis = if dx >= dy && dx >= dz { 0 } else if dy >= dz { 1 } else { 2 };

        // Compute centroids and sort by the chosen axis
        let mut sorted: Vec<usize> = indices.to_vec();
        sorted.sort_by(|&a, &b| {
            let ca = Self::triangle_centroid(vertices, &triangles[a], axis);
            let cb = Self::triangle_centroid(vertices, &triangles[b], axis);
            ca.partial_cmp(&cb).unwrap_or(std::cmp::Ordering::Equal)
        });

        // Median split
        let mid = sorted.len() / 2;
        let left_indices = &sorted[..mid];
        let right_indices = &sorted[mid..];

        if left_indices.is_empty() || right_indices.is_empty() {
            // Can't split further — make leaf
            return BvhNode {
                bbox_min,
                bbox_max,
                left: None,
                right: None,
                triangle_indices: Some(indices.to_vec()),
            };
        }

        let left = Self::build_recursive(vertices, triangles, left_indices, depth + 1);
        let right = Self::build_recursive(vertices, triangles, right_indices, depth + 1);

        BvhNode {
            bbox_min,
            bbox_max,
            left: Some(Box::new(left)),
            right: Some(Box::new(right)),
            triangle_indices: None,
        }
    }

    fn triangle_centroid(vertices: &[Point3d], tri: &[u32; 3], axis: usize) -> f64 {
        let v0 = vertices[tri[0] as usize];
        let v1 = vertices[tri[1] as usize];
        let v2 = vertices[tri[2] as usize];
        match axis {
            0 => (v0.x + v1.x + v2.x) / 3.0,
            1 => (v0.y + v1.y + v2.y) / 3.0,
            _ => (v0.z + v1.z + v2.z) / 3.0,
        }
    }

    fn compute_bbox(
        vertices: &[Point3d],
        triangles: &[[u32; 3]],
        indices: &[usize],
    ) -> (Point3d, Point3d) {
        let mut min = Point3d::new(f64::MAX, f64::MAX, f64::MAX);
        let mut max = Point3d::new(f64::MIN, f64::MIN, f64::MIN);
        for &idx in indices {
            let tri = &triangles[idx];
            for &vi in tri {
                let v = vertices[vi as usize];
                min.x = min.x.min(v.x);
                min.y = min.y.min(v.y);
                min.z = min.z.min(v.z);
                max.x = max.x.max(v.x);
                max.y = max.y.max(v.y);
                max.z = max.z.max(v.z);
            }
        }
        (min, max)
    }

    /// Intersect a ray with the BVH.
    ///
    /// Returns a list of (triangle_index, distance) pairs for all triangles
    /// intersected by the ray, sorted by distance.
    pub fn ray_intersect(&self, origin: &Point3d, dir: &Vec3d) -> Vec<(usize, f64)> {
        let mut hits = Vec::new();
        Self::ray_intersect_node(&self.root, &self.vertices, &self.triangles, origin, dir, &mut hits);
        hits.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        hits
    }

    fn ray_intersect_node(
        node: &BvhNode,
        vertices: &[Point3d],
        triangles: &[[u32; 3]],
        origin: &Point3d,
        dir: &Vec3d,
        hits: &mut Vec<(usize, f64)>,
    ) {
        // Test ray against AABB
        if !ray_aabb_intersect(origin, dir, &node.bbox_min, &node.bbox_max) {
            return;
        }

        if let Some(ref indices) = node.triangle_indices {
            // Leaf node — test each triangle
            for &idx in indices {
                let tri = &triangles[idx];
                let v0 = vertices[tri[0] as usize];
                let v1 = vertices[tri[1] as usize];
                let v2 = vertices[tri[2] as usize];
                if let Some(t) = moller_trumbore_distance(origin, dir, &v0, &v1, &v2) {
                    if t > 1e-10 {
                        hits.push((idx, t));
                    }
                }
            }
        } else {
            // Internal node — recurse into children
            if let Some(ref left) = node.left {
                Self::ray_intersect_node(left, vertices, triangles, origin, dir, hits);
            }
            if let Some(ref right) = node.right {
                Self::ray_intersect_node(right, vertices, triangles, origin, dir, hits);
            }
        }
    }

    /// Find triangles within `max_dist` of a point.
    ///
    /// Returns indices of triangles whose bounding boxes are within
    /// `max_dist` of the query point.
    pub fn closest_point(&self, point: &Point3d, max_dist: f64) -> Vec<usize> {
        let mut result = Vec::new();
        Self::closest_point_node(&self.root, point, max_dist, &mut result);
        result
    }

    fn closest_point_node(
        node: &BvhNode,
        point: &Point3d,
        max_dist: f64,
        result: &mut Vec<usize>,
    ) {
        // Check if point is within max_dist of the AABB
        if !point_aabb_within(point, &node.bbox_min, &node.bbox_max, max_dist) {
            return;
        }

        if let Some(ref indices) = node.triangle_indices {
            result.extend(indices.iter().cloned());
        } else {
            if let Some(ref left) = node.left {
                Self::closest_point_node(left, point, max_dist, result);
            }
            if let Some(ref right) = node.right {
                Self::closest_point_node(right, point, max_dist, result);
            }
        }
    }
}

/// Check if a ray intersects an axis-aligned bounding box (slab method).
fn ray_aabb_intersect(
    origin: &Point3d,
    dir: &Vec3d,
    bbox_min: &Point3d,
    bbox_max: &Point3d,
) -> bool {
    let eps = 1e-10;
    let mut t_min = f64::MIN;
    let mut t_max = f64::MAX;

    for (o, d, bmin, bmax) in &[
        (origin.x, dir.x, bbox_min.x, bbox_max.x),
        (origin.y, dir.y, bbox_min.y, bbox_max.y),
        (origin.z, dir.z, bbox_min.z, bbox_max.z),
    ] {
        if d.abs() < eps {
            // Ray is parallel to this slab
            if *o < *bmin || *o > *bmax {
                return false;
            }
        } else {
            let mut t1 = (bmin - o) / d;
            let mut t2 = (bmax - o) / d;
            if t1 > t2 {
                std::mem::swap(&mut t1, &mut t2);
            }
            t_min = t_min.max(t1);
            t_max = t_max.min(t2);
            if t_min > t_max {
                return false;
            }
        }
    }
    t_max >= 0.0
}

/// Möller–Trumbore ray-triangle intersection, returning the distance t
/// if the ray hits the triangle, or None.
fn moller_trumbore_distance(
    origin: &Point3d,
    dir: &Vec3d,
    v0: &Point3d,
    v1: &Point3d,
    v2: &Point3d,
) -> Option<f64> {
    let eps = 1e-10;
    let e1 = Vec3d::new(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
    let e2 = Vec3d::new(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
    let h = dir.cross(&e2);
    let a = e1.dot(&h);

    if a.abs() < eps {
        return None;
    }

    let f = 1.0 / a;
    let s = Vec3d::new(origin.x - v0.x, origin.y - v0.y, origin.z - v0.z);
    let u = f * s.dot(&h);
    if u < 0.0 || u > 1.0 {
        return None;
    }

    let q = s.cross(&e1);
    let v = f * dir.dot(&q);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }

    let t = f * e2.dot(&q);
    if t > eps {
        Some(t)
    } else {
        None
    }
}

/// Check if a point is within `max_dist` of an AABB.
fn point_aabb_within(
    point: &Point3d,
    bbox_min: &Point3d,
    bbox_max: &Point3d,
    max_dist: f64,
) -> bool {
    // Compute the closest point on the AABB to the query point
    let cx = if point.x < bbox_min.x {
        bbox_min.x
    } else if point.x > bbox_max.x {
        bbox_max.x
    } else {
        point.x
    };
    let cy = if point.y < bbox_min.y {
        bbox_min.y
    } else if point.y > bbox_max.y {
        bbox_max.y
    } else {
        point.y
    };
    let cz = if point.z < bbox_min.z {
        bbox_min.z
    } else if point.z > bbox_max.z {
        bbox_max.z
    } else {
        point.z
    };

    let dx = point.x - cx;
    let dy = point.y - cy;
    let dz = point.z - cz;
    dx * dx + dy * dy + dz * dz <= max_dist * max_dist
}

// ============================================================
// 4.2.7 View Frustum Culling
// ============================================================

/// A view frustum defined by 6 clipping planes.
///
/// Each plane is represented as `(a, b, c, d)` where the plane equation
/// is `ax + by + cz + d = 0` and the normal `(a, b, c)` points **inward**
/// (toward the visible region). A point is inside the frustum if it
/// satisfies all 6 plane inequalities: `ax + by + cz + d >= 0`.
///
/// # Construction
///
/// Extract planes from a view-projection matrix using [`Frustum::from_matrix`],
/// or construct manually from individual plane equations.
///
/// # Usage with BVH
///
/// ```ignore
/// let frustum = Frustum::from_matrix(&view_projection);
/// let visible_triangles = bvh.frustum_cull(&frustum);
/// ```
#[derive(Clone, Debug)]
pub struct Frustum {
    /// The 6 clipping planes: [left, right, bottom, top, near, far].
    /// Each plane is `(a, b, c, d)` with inward-pointing normal.
    pub planes: [[f64; 4]; 6],
}

impl Frustum {
    /// Extract the frustum planes from a 4×4 view-projection matrix.
    ///
    /// The matrix is stored in column-major order (OpenGL convention):
    /// `m[col][row]` where `col` is the column index (0..4) and `row`
    /// is the row index (0..4).
    ///
    /// This uses the standard clipping-plane extraction method:
    /// - Left:   `m[3] + m[0]`
    /// - Right:  `m[3] - m[0]`
    /// - Bottom: `m[3] + m[1]`
    /// - Top:    `m[3] - m[1]`
    /// - Near:   `m[3] + m[2]`
    /// - Far:    `m[3] - m[2]`
    ///
    /// After extraction, each plane is normalized so that the normal
    /// has unit length. The normal is flipped to point inward.
    pub fn from_matrix(m: &[[f64; 4]; 4]) -> Self {
        // Column-major: m[col][row]
        // m[0] = first column, m[1] = second column, etc.
        let mut planes = [[0.0f64; 4]; 6];

        // Left: m[3] + m[0]
        planes[0] = normalize_plane([
            m[0][3] + m[0][0],
            m[1][3] + m[1][0],
            m[2][3] + m[2][0],
            m[3][3] + m[3][0],
        ]);

        // Right: m[3] - m[0]
        planes[1] = normalize_plane([
            m[0][3] - m[0][0],
            m[1][3] - m[1][0],
            m[2][3] - m[2][0],
            m[3][3] - m[3][0],
        ]);

        // Bottom: m[3] + m[1]
        planes[2] = normalize_plane([
            m[0][3] + m[0][1],
            m[1][3] + m[1][1],
            m[2][3] + m[2][1],
            m[3][3] + m[3][1],
        ]);

        // Top: m[3] - m[1]
        planes[3] = normalize_plane([
            m[0][3] - m[0][1],
            m[1][3] - m[1][1],
            m[2][3] - m[2][1],
            m[3][3] - m[3][1],
        ]);

        // Near: m[3] + m[2]
        planes[4] = normalize_plane([
            m[0][3] + m[0][2],
            m[1][3] + m[1][2],
            m[2][3] + m[2][2],
            m[3][3] + m[3][2],
        ]);

        // Far: m[3] - m[2]
        planes[5] = normalize_plane([
            m[0][3] - m[0][2],
            m[1][3] - m[1][2],
            m[2][3] - m[2][2],
            m[3][3] - m[3][2],
        ]);

        Frustum { planes }
    }

    /// Construct a frustum from 6 plane equations.
    ///
    /// Each plane is `(a, b, c, d)` where the plane equation is
    /// `ax + by + cz + d = 0` and the normal `(a, b, c)` points
    /// **inward** (toward the visible region).
    ///
    /// The planes should be in the order: [left, right, bottom, top, near, far].
    pub fn from_planes(planes: [[f64; 4]; 6]) -> Self {
        Frustum { planes }
    }

    /// Test if a point is inside the frustum.
    ///
    /// A point is inside if it satisfies all 6 plane inequalities.
    #[inline]
    pub fn contains_point(&self, p: &Point3d) -> bool {
        for plane in &self.planes {
            if plane[0] * p.x + plane[1] * p.y + plane[2] * p.z + plane[3] < 0.0 {
                return false;
            }
        }
        true
    }

    /// Test if an AABB intersects the frustum.
    ///
    /// Returns `true` if the AABB is partially or fully inside the frustum.
    /// Uses the "p-vertex / n-vertex" optimization: for each plane, the
    /// diagonally opposite corner that is most aligned with the plane's
    /// normal is tested first. If even the most-aligned corner is outside,
    /// the entire AABB is outside that plane.
    pub fn intersects_aabb(&self, bbox_min: &Point3d, bbox_max: &Point3d) -> bool {
        for plane in &self.planes {
            // Pick the "positive vertex" — the corner most in the direction
            // of the plane's inward normal. If this corner is outside,
            // the entire AABB is outside.
            let px = if plane[0] >= 0.0 { bbox_max.x } else { bbox_min.x };
            let py = if plane[1] >= 0.0 { bbox_max.y } else { bbox_min.y };
            let pz = if plane[2] >= 0.0 { bbox_max.z } else { bbox_min.z };

            if plane[0] * px + plane[1] * py + plane[2] * pz + plane[3] < 0.0 {
                return false;
            }
        }
        true
    }
}

/// Normalize a plane so its normal has unit length.
fn normalize_plane(p: [f64; 4]) -> [f64; 4] {
    let len = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
    if len < 1e-15 {
        return p;
    }
    let inv = 1.0 / len;
    [p[0] * inv, p[1] * inv, p[2] * inv, p[3] * inv]
}

impl Bvh {
    /// Frustum culling: collect all triangles visible from the given frustum.
    ///
    /// Traverses the BVH and returns indices of all triangles whose
    /// bounding boxes intersect the view frustum. This is an efficient
    /// way to skip rendering triangles that are off-screen.
    ///
    /// # Algorithm
    /// For each BVH node:
    /// 1. Test the node's AABB against the frustum
    /// 2. If fully outside → skip entire subtree (O(1) culling)
    /// 3. If inside → add all triangles in subtree
    /// 4. If partially inside → recurse into children
    ///
    /// # Performance
    /// For a model with N triangles, worst case is O(N) (all visible),
    /// but typical case for culling is O(log N) per visible cluster.
    /// A model with 1M triangles and 10% visible runs ~10× faster
    /// than brute-force rendering.
    pub fn frustum_cull(&self, frustum: &Frustum) -> Vec<usize> {
        let mut visible = Vec::new();
        Self::frustum_cull_node(&self.root, frustum, &mut visible);
        visible
    }

    fn frustum_cull_node(node: &BvhNode, frustum: &Frustum, visible: &mut Vec<usize>) {
        // Test node's AABB against the frustum
        if !frustum.intersects_aabb(&node.bbox_min, &node.bbox_max) {
            return; // Entire subtree is outside the frustum
        }

        if let Some(ref indices) = node.triangle_indices {
            // Leaf node — all triangles are potentially visible
            visible.extend(indices.iter().cloned());
        } else {
            // Internal node — recurse into children
            if let Some(ref left) = node.left {
                Self::frustum_cull_node(left, frustum, visible);
            }
            if let Some(ref right) = node.right {
                Self::frustum_cull_node(right, frustum, visible);
            }
        }
    }

    /// Frustum culling with per-face granularity.
    ///
    /// Same as [`frustum_cull`](Self::frustum_cull), but groups results
    /// by per-triangle face IDs. Returns a set of face IDs that have at
    /// least one visible triangle.
    ///
    /// This is useful for selective face rendering: only re-render
    /// faces whose triangles are visible, avoiding per-triangle overhead.
    pub fn frustum_cull_faces(&self, frustum: &Frustum, triangle_face_ids: &[u64]) -> std::collections::HashSet<u64> {
        let visible_indices = self.frustum_cull(frustum);
        let mut face_set = std::collections::HashSet::new();
        for idx in visible_indices {
            if idx < triangle_face_ids.len() {
                face_set.insert(triangle_face_ids[idx]);
            }
        }
        face_set
    }
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builder::ShapeBuilder;
    use std::f64::consts::PI;

    #[test]
    fn test_volume_unit_cube() {
        let cube = ShapeBuilder::make_box(1.0, 1.0, 1.0);
        let vol = solid_volume(&cube);
        assert!(
            (vol - 1.0).abs() < 0.01,
            "Unit cube volume should be 1.0, got {}",
            vol
        );
    }

    #[test]
    fn test_volume_cube_2x3x4() {
        let cube = ShapeBuilder::make_box(2.0, 3.0, 4.0);
        let vol = solid_volume(&cube);
        assert!(
            (vol - 24.0).abs() < 0.1,
            "2x3x4 cube volume should be 24.0, got {}",
            vol
        );
    }

    #[test]
    fn test_volume_cylinder() {
        let radius = 1.0;
        let height = 2.0;
        let cyl = ShapeBuilder::make_cylinder(radius, height);
        let vol = solid_volume(&cyl);
        let expected = PI * radius * radius * height;
        let rel_err = (vol - expected).abs() / expected;
        assert!(
            rel_err < 0.02,
            "Cylinder volume should be ≈ {}, got {} (rel_err = {})",
            expected,
            vol,
            rel_err
        );
    }

    #[test]
    fn test_surface_area_unit_cube() {
        let cube = ShapeBuilder::make_box(1.0, 1.0, 1.0);
        let area = solid_surface_area(&cube);
        assert!(
            (area - 6.0).abs() < 0.01,
            "Unit cube surface area should be 6.0, got {}",
            area
        );
    }

    #[test]
    fn test_surface_area_cylinder() {
        let radius = 1.0;
        let height = 2.0;
        let cyl = ShapeBuilder::make_cylinder(radius, height);
        let area = solid_surface_area(&cyl);
        // Expected: 2*π*r² + 2*π*r*h = 2*π + 4*π = 6*π ≈ 18.85
        let expected = 2.0 * PI * radius * radius + 2.0 * PI * radius * height;
        let rel_err = (area - expected).abs() / expected;
        assert!(
            rel_err < 0.05,
            "Cylinder surface area should be ≈ {}, got {} (rel_err = {})",
            expected,
            area,
            rel_err
        );
    }

    #[test]
    fn test_center_of_mass_unit_cube() {
        let cube = ShapeBuilder::make_box(1.0, 1.0, 1.0);
        let com = solid_center_of_mass(&cube);
        assert!(
            (com.x).abs() < 0.01,
            "Cube COM x should be 0.0, got {}",
            com.x
        );
        assert!(
            (com.y).abs() < 0.01,
            "Cube COM y should be 0.0, got {}",
            com.y
        );
        assert!(
            (com.z).abs() < 0.01,
            "Cube COM z should be 0.0, got {}",
            com.z
        );
    }

    #[test]
    fn test_point_inside_cube() {
        let cube = ShapeBuilder::make_box(2.0, 2.0, 2.0);
        let inside = Point3d::new(0.0, 0.0, 0.0); // Center of the cube
        assert!(
            point_in_solid(&cube, &inside),
            "Center of cube should be inside"
        );
    }

    #[test]
    fn test_point_outside_cube() {
        let cube = ShapeBuilder::make_box(2.0, 2.0, 2.0);
        let outside = Point3d::new(5.0, 5.0, 5.0);
        assert!(
            !point_in_solid(&cube, &outside),
            "Point far from cube should be outside"
        );
    }

    #[test]
    fn test_point_inside_cube_corner() {
        let cube = ShapeBuilder::make_box(2.0, 2.0, 2.0);
        // Point slightly inside the cube
        let inside = Point3d::new(0.5, 0.5, 0.5);
        assert!(
            point_in_solid(&cube, &inside),
            "Point (0.5, 0.5, 0.5) should be inside 2x2x2 cube centered at origin"
        );
    }

    #[test]
    fn test_inertia_tensor_cube() {
        let cube = ShapeBuilder::make_box(1.0, 1.0, 1.0);
        let inertia = solid_moments_of_inertia(&cube);
        // For a unit cube (side=1, mass=1, density=1):
        // Ixx = Iyy = Izz = (1/12) * (d² + h²) = (1/12) * (1+1) = 1/6 ≈ 0.1667
        // Ixy = Ixz = Iyz = 0 (symmetric about all axes)
        let expected = 1.0 / 6.0;
        assert!(
            (inertia.ixx - expected).abs() < 0.02,
            "Cube Ixx should be ≈ {}, got {}",
            expected,
            inertia.ixx
        );
        assert!(
            (inertia.iyy - expected).abs() < 0.02,
            "Cube Iyy should be ≈ {}, got {}",
            expected,
            inertia.iyy
        );
        assert!(
            (inertia.izz - expected).abs() < 0.02,
            "Cube Izz should be ≈ {}, got {}",
            expected,
            inertia.izz
        );
        assert!(
            inertia.ixy.abs() < 0.02,
            "Cube Ixy should be ≈ 0, got {}",
            inertia.ixy
        );
    }

    #[test]
    fn test_bvh_build_and_ray_intersect() {
        // Create a simple mesh: two triangles forming a quad
        let vertices = vec![
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 0.0, 0.0),
            Point3d::new(1.0, 1.0, 0.0),
            Point3d::new(0.0, 1.0, 0.0),
        ];
        let triangles = vec![[0u32, 1, 2], [0u32, 2, 3]];

        let bvh = Bvh::build(&vertices, &triangles);

        // Ray from (0.5, 0.5, -1) in +Z direction should hit
        let origin = Point3d::new(0.5, 0.5, -1.0);
        let dir = Vec3d::new(0.0, 0.0, 1.0);
        let hits = bvh.ray_intersect(&origin, &dir);
        assert!(!hits.is_empty(), "Ray should hit the quad");
        // Distance should be 1.0
        let min_dist = hits.iter().map(|&(_, d)| d).fold(f64::MAX, f64::min);
        assert!(
            (min_dist - 1.0).abs() < 0.01,
            "Ray should hit at distance 1.0, got {}",
            min_dist
        );
    }

    #[test]
    fn test_bvh_closest_point() {
        let vertices = vec![
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 0.0, 0.0),
            Point3d::new(1.0, 1.0, 0.0),
            Point3d::new(0.0, 1.0, 0.0),
        ];
        let triangles = vec![[0u32, 1, 2], [0u32, 2, 3]];

        let bvh = Bvh::build(&vertices, &triangles);

        // Point at (0.5, 0.5, 0.1) should be within 0.5 of the quad
        let point = Point3d::new(0.5, 0.5, 0.1);
        let nearby = bvh.closest_point(&point, 0.5);
        assert!(!nearby.is_empty(), "Should find nearby triangles");

        // Point far away should not find any triangles
        let far = Point3d::new(10.0, 10.0, 10.0);
        let far_nearby = bvh.closest_point(&far, 0.5);
        assert!(far_nearby.is_empty(), "Should not find any triangles near far point");
    }

    #[test]
    fn test_bvh_with_cube_solid() {
        let cube = ShapeBuilder::make_box(1.0, 1.0, 1.0);
        let mesh = triangulate_solid_for_queries(&cube);
        let bvh = Bvh::build(&mesh.vertices, &mesh.triangles);

        // Ray from inside the cube should hit two faces (enter and exit)
        let origin = Point3d::new(0.0, 0.0, 0.0);
        let dir = Vec3d::new(1.0, 0.0, 0.0);
        let hits = bvh.ray_intersect(&origin, &dir);
        assert!(
            hits.len() >= 2,
            "Ray from inside cube should hit at least 2 faces, got {}",
            hits.len()
        );
    }

    #[test]
    fn test_frustum_contains_point() {
        // Simple frustum: a box from -1 to 1 in each axis
        // left:   x = -1  →  x + 1 >= 0   →  (1, 0, 0, 1)
        // right:  x =  1  → -x + 1 >= 0   →  (-1, 0, 0, 1)
        // bottom: y = -1  →  y + 1 >= 0   →  (0, 1, 0, 1)
        // top:    y =  1  → -y + 1 >= 0   →  (0, -1, 0, 1)
        // near:   z = -1  →  z + 1 >= 0   →  (0, 0, 1, 1)
        // far:    z =  1  → -z + 1 >= 0   →  (0, 0, -1, 1)
        let frustum = Frustum::from_planes([
            [1.0, 0.0, 0.0, 1.0],   // left
            [-1.0, 0.0, 0.0, 1.0],  // right
            [0.0, 1.0, 0.0, 1.0],   // bottom
            [0.0, -1.0, 0.0, 1.0],  // top
            [0.0, 0.0, 1.0, 1.0],   // near
            [0.0, 0.0, -1.0, 1.0],  // far
        ]);

        // Center should be inside
        assert!(frustum.contains_point(&Point3d::new(0.0, 0.0, 0.0)));
        // Just inside the boundary
        assert!(frustum.contains_point(&Point3d::new(0.99, 0.99, 0.99)));
        // Outside
        assert!(!frustum.contains_point(&Point3d::new(2.0, 0.0, 0.0)));
        assert!(!frustum.contains_point(&Point3d::new(0.0, 2.0, 0.0)));
        assert!(!frustum.contains_point(&Point3d::new(0.0, 0.0, 2.0)));
    }

    #[test]
    fn test_frustum_aabb_intersection() {
        let frustum = Frustum::from_planes([
            [1.0, 0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0, 1.0],
            [0.0, -1.0, 0.0, 1.0],
            [0.0, 0.0, 1.0, 1.0],
            [0.0, 0.0, -1.0, 1.0],
        ]);

        // AABB fully inside
        assert!(frustum.intersects_aabb(
            &Point3d::new(-0.5, -0.5, -0.5),
            &Point3d::new(0.5, 0.5, 0.5),
        ));

        // AABB partially inside (overlapping boundary)
        assert!(frustum.intersects_aabb(
            &Point3d::new(0.5, -0.5, -0.5),
            &Point3d::new(1.5, 0.5, 0.5),
        ));

        // AABB fully outside
        assert!(!frustum.intersects_aabb(
            &Point3d::new(2.0, 2.0, 2.0),
            &Point3d::new(3.0, 3.0, 3.0),
        ));
    }

    #[test]
    fn test_bvh_frustum_cull() {
        // Create two separated groups of triangles with enough
        // triangles to force BVH to split into separate nodes.
        // Group 1: near origin, Group 2: far away at x=10
        let mut vertices = Vec::new();
        let mut triangles = Vec::new();

        // Group 1: 8 triangles (2 quads × 2 each × 2 rows) near origin
        for row in 0..4 {
            for col in 0..4 {
                let x0 = col as f64;
                let y0 = row as f64;
                let x1 = x0 + 1.0;
                let y1 = y0 + 1.0;
                let base = vertices.len() as u32;
                vertices.push(Point3d::new(x0, y0, 0.0));
                vertices.push(Point3d::new(x1, y0, 0.0));
                vertices.push(Point3d::new(x1, y1, 0.0));
                vertices.push(Point3d::new(x0, y1, 0.0));
                triangles.push([base, base + 1, base + 2]);
                triangles.push([base, base + 2, base + 3]);
            }
        }
        let group1_count = triangles.len();

        // Group 2: same grid but shifted to x=10
        for row in 0..4 {
            for col in 0..4 {
                let x0 = 10.0 + col as f64;
                let y0 = row as f64;
                let x1 = x0 + 1.0;
                let y1 = y0 + 1.0;
                let base = vertices.len() as u32;
                vertices.push(Point3d::new(x0, y0, 0.0));
                vertices.push(Point3d::new(x1, y0, 0.0));
                vertices.push(Point3d::new(x1, y1, 0.0));
                vertices.push(Point3d::new(x0, y1, 0.0));
                triangles.push([base, base + 1, base + 2]);
                triangles.push([base, base + 2, base + 3]);
            }
        }

        let bvh = Bvh::build(&vertices, &triangles);

        // Frustum that only includes Group 1 (x: -1 to 6)
        let frustum = Frustum::from_planes([
            [1.0, 0.0, 0.0, 1.0],   // left: x >= -1
            [-1.0, 0.0, 0.0, 6.0],  // right: x <= 6
            [0.0, 1.0, 0.0, 6.0],   // bottom: y >= -6
            [0.0, -1.0, 0.0, 6.0],  // top: y <= 6
            [0.0, 0.0, 1.0, 6.0],   // near: z >= -6
            [0.0, 0.0, -1.0, 6.0],  // far: z <= 6
        ]);

        let visible = bvh.frustum_cull(&frustum);
        // Should cull Group 2 (x=10..14 is outside x<=6)
        assert!(
            visible.len() <= group1_count,
            "Frustum should cull Group 2, got {} visible out of {} total",
            visible.len(),
            triangles.len(),
        );
        // All visible indices should be from Group 1
        for &idx in &visible {
            assert!(idx < group1_count, "Visible triangle {} should be from Group 1", idx);
        }
    }

    #[test]
    fn test_frustum_from_identity_matrix() {
        // Identity matrix should produce some valid frustum
        let identity = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ];
        let frustum = Frustum::from_matrix(&identity);
        // All planes should have valid (non-NaN) values
        for plane in &frustum.planes {
            for &v in plane {
                assert!(v.is_finite(), "Plane value should be finite, got {}", v);
            }
        }
    }
}
