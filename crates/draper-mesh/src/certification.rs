// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Additive Manufacturing Certification.
//!
//! Provides certification checks for 3D-printable meshes:
//! - 5.2.1 Watertightness certification (0 boundary edges for closed solids)
//! - 5.2.2 Wall thickness control
//! - 5.2.3 Self-intersection check
//! - 5.2.4 Mesh quality control (angles, aspect ratios, winding, duplicates)

use crate::manifold::{check_manifold, ManifoldReport};
use crate::mesh::TriangleMesh;
use draper_geometry::{Point3d, Surface};
use draper_topology::Solid;
use std::collections::HashMap;

// ============================================================
// 5.2.1 Watertightness Certification
// ============================================================

/// Result of a watertightness certification check.
#[derive(Clone, Debug)]
pub struct CertificationResult {
    /// Whether the mesh passes the watertightness check.
    pub passed: bool,
    /// Number of boundary edges found (should be 0 for a closed solid).
    pub boundary_edge_count: usize,
    /// Number of non-manifold edges found (should be 0).
    pub non_manifold_edge_count: usize,
    /// Number of degenerate triangles.
    pub degenerate_triangle_count: usize,
    /// Euler characteristic (V - E + F).
    pub euler_characteristic: i64,
    /// Full manifold report with detailed edge information.
    pub manifold_report: ManifoldReport,
    /// Human-readable summary.
    pub summary: String,
}

impl CertificationResult {
    /// Whether the certification passed.
    pub fn is_passed(&self) -> bool {
        self.passed
    }
}

/// Certify that a mesh is watertight (suitable for additive manufacturing).
///
/// A mesh is watertight if:
/// - It has 0 boundary edges (every edge is shared by exactly 2 triangles)
/// - It has 0 non-manifold edges (no edge is shared by more than 2 triangles)
/// - Euler characteristic is consistent with a closed surface
///
/// This is a hard requirement for 3D printing — open meshes or meshes with
/// non-manifold edges cannot be sliced properly.
pub fn certify_watertight(mesh: &TriangleMesh) -> CertificationResult {
    let report = check_manifold(mesh);

    let boundary_ok = report.boundary_edge_count == 0;
    let manifold_ok = report.non_manifold_edge_count == 0;
    let passed = boundary_ok && manifold_ok;

    let mut summary_parts = Vec::new();
    if boundary_ok {
        summary_parts.push("0 boundary edges ✓".to_string());
    } else {
        summary_parts.push(format!("{} boundary edges ✗ (need 0)", report.boundary_edge_count));
    }
    if manifold_ok {
        summary_parts.push("0 non-manifold edges ✓".to_string());
    } else {
        summary_parts.push(format!("{} non-manifold edges ✗ (need 0)", report.non_manifold_edge_count));
    }
    summary_parts.push(format!("Euler χ = {}", report.euler_characteristic));
    if report.degenerate_triangle_count > 0 {
        summary_parts.push(format!("{} degenerate triangles ⚠", report.degenerate_triangle_count));
    }

    let summary = if passed {
        format!("PASSED — {}", summary_parts.join(", "))
    } else {
        format!("FAILED — {}", summary_parts.join(", "))
    };

    CertificationResult {
        passed,
        boundary_edge_count: report.boundary_edge_count,
        non_manifold_edge_count: report.non_manifold_edge_count,
        degenerate_triangle_count: report.degenerate_triangle_count,
        euler_characteristic: report.euler_characteristic,
        manifold_report: report,
        summary,
    }
}

// ============================================================
// 5.2.2 Wall Thickness Control
// ============================================================

/// A thin region detected during wall thickness analysis.
#[derive(Clone, Debug)]
pub struct ThinRegion {
    /// Position of the interior sample point that detected the thin region.
    pub position: Point3d,
    /// The closest surface point to this interior point.
    pub closest_surface_point: Point3d,
    /// The measured wall thickness (distance from interior point to surface).
    pub thickness: f64,
    /// The minimum required thickness.
    pub min_thickness: f64,
    /// The ratio of measured to required thickness (below 1.0 = too thin).
    pub ratio: f64,
}

/// Result of a wall thickness check.
#[derive(Clone, Debug)]
pub struct WallThicknessResult {
    /// All thin regions found.
    pub thin_regions: Vec<ThinRegion>,
    /// Number of sample points tested.
    pub samples_tested: usize,
    /// Minimum wall thickness found across all samples.
    pub min_thickness_found: f64,
    /// Whether the mesh passes the wall thickness check.
    pub passed: bool,
    /// Human-readable summary.
    pub summary: String,
}

/// Check wall thickness of a solid by sampling interior points.
///
/// For each interior sample point, finds the closest surface point.
/// If the distance is less than `min_thickness`, marks it as a thin region.
///
/// **Note:** This is a sampling-based approximation. It may miss thin features
/// that fall between sample points. For critical applications, use a higher
/// sample count or a more rigorous approach.
///
/// # Arguments
/// * `solid` - The solid to check (must have valid surface geometry)
/// * `min_thickness` - Minimum acceptable wall thickness (in model units)
///
/// # Returns
/// A list of thin regions where the wall thickness falls below the minimum.
pub fn check_wall_thickness(solid: &Solid, min_thickness: f64) -> WallThicknessResult {
    // Triangulate the solid to get a mesh for surface distance queries
    let mesh = triangulate_solid_for_thickness(solid);
    if mesh.vertices.is_empty() || mesh.triangles.is_empty() {
        return WallThicknessResult {
            thin_regions: Vec::new(),
            samples_tested: 0,
            min_thickness_found: f64::MAX,
            passed: true,
            summary: "Empty solid — no samples to test".to_string(),
        };
    }

    check_wall_thickness_mesh(&mesh, min_thickness)
}

/// Check wall thickness using a triangulated mesh.
///
/// This is the main implementation that works on a `TriangleMesh`.
/// Interior sample points are generated within the bounding box, tested
/// for inside/outside, and then the distance to the closest surface
/// triangle is computed.
pub fn check_wall_thickness_mesh(mesh: &TriangleMesh, min_thickness: f64) -> WallThicknessResult {
    let (bbox_min, bbox_max) = mesh.bounding_box();

    // Build BVH for closest-point queries
    let bvh = draper_topology::Bvh::build(&mesh.vertices, &mesh.triangles);

    // Determine grid resolution for sampling
    let dx = bbox_max.x - bbox_min.x;
    let dy = bbox_max.y - bbox_min.y;
    let dz = bbox_max.z - bbox_min.z;
    let max_dim = dx.max(dy).max(dz);

    if max_dim < 1e-10 {
        return WallThicknessResult {
            thin_regions: Vec::new(),
            samples_tested: 0,
            min_thickness_found: f64::MAX,
            passed: true,
            summary: "Degenerate bounding box — no samples".to_string(),
        };
    }

    // Shrink the sampling region by min_thickness from each face to avoid
    // sampling points near the surface that would naturally have small
    // wall thickness. We want to check the thickness of interior walls,
    // not measure the distance from near-surface points to the surface.
    let shrink = min_thickness;
    let inner_min = Point3d::new(
        bbox_min.x + shrink,
        bbox_min.y + shrink,
        bbox_min.z + shrink,
    );
    let inner_max = Point3d::new(
        bbox_max.x - shrink,
        bbox_max.y - shrink,
        bbox_max.z - shrink,
    );

    let inner_dx = (inner_max.x - inner_min.x).max(0.0);
    let inner_dy = (inner_max.y - inner_min.y).max(0.0);
    let inner_dz = (inner_max.z - inner_min.z).max(0.0);

    if inner_dx < 1e-10 || inner_dy < 1e-10 || inner_dz < 1e-10 {
        // The solid is thinner than the minimum required wall thickness
        // in at least one dimension — this is a thin region.
        return WallThicknessResult {
            thin_regions: vec![ThinRegion {
                position: Point3d::new(
                    (bbox_min.x + bbox_max.x) * 0.5,
                    (bbox_min.y + bbox_max.y) * 0.5,
                    (bbox_min.z + bbox_max.z) * 0.5,
                ),
                closest_surface_point: Point3d::ORIGIN,
                thickness: max_dim * 0.5, // Approximate
                min_thickness,
                ratio: 0.0,
            }],
            samples_tested: 0,
            min_thickness_found: max_dim * 0.5,
            passed: false,
            summary: format!(
                "FAILED — solid too thin for wall thickness sampling (bbox dimension < {:.4})",
                min_thickness * 2.0
            ),
        };
    }

    // Use a grid spacing based on min_thickness
    let grid_spacing = (min_thickness / 2.0).max(max_dim / 50.0);
    let nx = ((inner_dx / grid_spacing).ceil() as usize).max(2).min(30);
    let ny = ((inner_dy / grid_spacing).ceil() as usize).max(2).min(30);
    let nz = ((inner_dz / grid_spacing).ceil() as usize).max(2).min(30);

    let mut thin_regions = Vec::new();
    let mut samples_tested = 0;
    let mut min_thickness_found = f64::MAX;

    for ix in 0..nx {
        for iy in 0..ny {
            for iz in 0..nz {
                let t_x = (ix as f64 + 0.5) / nx as f64;
                let t_y = (iy as f64 + 0.5) / ny as f64;
                let t_z = (iz as f64 + 0.5) / nz as f64;

                let point = Point3d::new(
                    inner_min.x + t_x * inner_dx,
                    inner_min.y + t_y * inner_dy,
                    inner_min.z + t_z * inner_dz,
                );

                // Check if point is inside the solid using ray casting
                if !point_inside_mesh(&point, mesh) {
                    continue;
                }

                samples_tested += 1;

                // Find closest surface point
                let (closest_point, distance) = find_closest_surface_point(&point, mesh, &bvh);

                if distance < min_thickness_found {
                    min_thickness_found = distance;
                }

                if distance < min_thickness {
                    thin_regions.push(ThinRegion {
                        position: point,
                        closest_surface_point: closest_point,
                        thickness: distance,
                        min_thickness,
                        ratio: distance / min_thickness,
                    });
                }
            }
        }
    }

    if min_thickness_found == f64::MAX {
        min_thickness_found = 0.0;
    }

    let passed = thin_regions.is_empty();
    let summary = if passed {
        format!("PASSED — {} samples tested, min thickness {:.4} ≥ {:.4}", 
            samples_tested, min_thickness_found, min_thickness)
    } else {
        format!("FAILED — {} thin regions out of {} samples, min thickness {:.4} < {:.4}",
            thin_regions.len(), samples_tested, min_thickness_found, min_thickness)
    };

    WallThicknessResult {
        thin_regions,
        samples_tested,
        min_thickness_found,
        passed,
        summary,
    }
}

/// Triangulate a solid for thickness analysis.
fn triangulate_solid_for_thickness(solid: &Solid) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    let faces = solid.faces();
    for face in &faces {
        if let Some(ref surface) = face.surface {
            // Get boundary edge endpoints
            let edge_points: Vec<Point3d> = face.edges.iter()
                .flat_map(|edge| {
                    let mut pts = Vec::new();
                    if let Some(p) = edge.start_point() { pts.push(p); }
                    if let Some(p) = edge.end_point() { pts.push(p); }
                    pts
                })
                .collect();

            // Simple triangulation: project boundary points and triangulate
            if edge_points.len() >= 3 {
                let base_idx = mesh.vertices.len() as u32;
                for &p in &edge_points {
                    mesh.add_vertex(p);
                }
                // Fan triangulation from first vertex
                for i in 1..edge_points.len().saturating_sub(1) {
                    mesh.add_triangle(base_idx, base_idx + i as u32, base_idx + (i + 1) as u32);
                }
            } else {
                // Surface without edges — create a simple quad approximation
                let (u_min, u_max) = surface_param_u_range(surface);
                let (v_min, v_max) = surface_param_v_range(surface);
                let n = 4;
                let base_idx = mesh.vertices.len() as u32;
                for iv in 0..=n {
                    for iu in 0..=n {
                        let u = u_min + (u_max - u_min) * iu as f64 / n as f64;
                        let v = v_min + (v_max - v_min) * iv as f64 / n as f64;
                        mesh.add_vertex(surface.point_at(u, v));
                    }
                }
                for iv in 0..n {
                    for iu in 0..n {
                        let i00 = base_idx + (iv * (n + 1) + iu) as u32;
                        let i10 = base_idx + (iv * (n + 1) + iu + 1) as u32;
                        let i01 = base_idx + ((iv + 1) * (n + 1) + iu) as u32;
                        let i11 = base_idx + ((iv + 1) * (n + 1) + iu + 1) as u32;
                        mesh.add_triangle(i00, i10, i11);
                        mesh.add_triangle(i00, i11, i01);
                    }
                }
            }
        }
    }

    mesh.compute_face_normals();
    mesh
}

/// Get the u parameter range for a Surface variant.
fn surface_param_u_range(surface: &Surface) -> (f64, f64) {
    match surface {
        Surface::Plane(_) => (-1000.0, 1000.0), // Planes are infinite
        Surface::Cylinder(c) => c.u_range(),
        Surface::Cone(_) => (0.0, 2.0 * std::f64::consts::PI),
        Surface::Sphere(_) => (0.0, 2.0 * std::f64::consts::PI),
        Surface::Torus(_) => (0.0, 2.0 * std::f64::consts::PI),
        Surface::Revolution(_) => (0.0, 2.0 * std::f64::consts::PI),
        Surface::Extrusion(_) => (0.0, 1.0),
        Surface::Nurbs(n) => n.u_range(),
        _ => (0.0, 1.0),
    }
}

/// Get the v parameter range for a Surface variant.
fn surface_param_v_range(surface: &Surface) -> (f64, f64) {
    match surface {
        Surface::Plane(_) => (-1000.0, 1000.0), // Planes are infinite
        Surface::Cylinder(_) => (-1000.0, 1000.0), // Cylinders are infinite along axis
        Surface::Cone(_) => (-1000.0, 1000.0),
        Surface::Sphere(_) => (0.0, std::f64::consts::PI),
        Surface::Torus(_) => (0.0, 2.0 * std::f64::consts::PI),
        Surface::Revolution(_) => (-1000.0, 1000.0),
        Surface::Extrusion(_) => (-1000.0, 1000.0),
        Surface::Nurbs(n) => n.v_range(),
        _ => (0.0, 1.0),
    }
}

/// Check if a point is inside a mesh using ray casting.
///
/// Casts rays in multiple directions and uses majority voting
/// to avoid edge/vertex hit artifacts.
fn point_inside_mesh(point: &Point3d, mesh: &TriangleMesh) -> bool {
    let ray_dirs = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ];

    let mut inside_votes = 0u32;
    let mut outside_votes = 0u32;

    for dir in &ray_dirs {
        let mut count = 0u32;
        for tri in &mesh.triangles {
            let v0 = mesh.vertices[tri[0] as usize];
            let v1 = mesh.vertices[tri[1] as usize];
            let v2 = mesh.vertices[tri[2] as usize];

            if let Some(t) = ray_triangle_intersect(point, dir, &v0, &v1, &v2) {
                if t > 1e-10 {
                    count += 1;
                }
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

/// Möller–Trumbore ray-triangle intersection.
/// Returns the ray parameter t if the ray hits the triangle, None otherwise.
fn ray_triangle_intersect(
    origin: &Point3d,
    dir: &[f64; 3],
    v0: &Point3d,
    v1: &Point3d,
    v2: &Point3d,
) -> Option<f64> {
    let e1x = v1.x - v0.x;
    let e1y = v1.y - v0.y;
    let e1z = v1.z - v0.z;
    let e2x = v2.x - v0.x;
    let e2y = v2.y - v0.y;
    let e2z = v2.z - v0.z;

    let hx = dir[1] * e2z - dir[2] * e2y;
    let hy = dir[2] * e2x - dir[0] * e2z;
    let hz = dir[0] * e2y - dir[1] * e2x;

    let a = e1x * hx + e1y * hy + e1z * hz;
    if a.abs() < 1e-10 {
        return None;
    }

    let f = 1.0 / a;
    let sx = origin.x - v0.x;
    let sy = origin.y - v0.y;
    let sz = origin.z - v0.z;

    let u = f * (sx * hx + sy * hy + sz * hz);
    if u < 0.0 || u > 1.0 {
        return None;
    }

    let qx = sy * e1z - sz * e1y;
    let qy = sz * e1x - sx * e1z;
    let qz = sx * e1y - sy * e1x;

    let v = f * (dir[0] * qx + dir[1] * qy + dir[2] * qz);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }

    let t = f * (e2x * qx + e2y * qy + e2z * qz);
    Some(t)
}

/// Find the closest surface point to a given query point.
///
/// Returns the closest point and the distance.
fn find_closest_surface_point(
    point: &Point3d,
    mesh: &TriangleMesh,
    bvh: &draper_topology::Bvh,
) -> (Point3d, f64) {
    // Use BVH to find candidate triangles, then compute exact distance
    let search_radius = {
        let (bmin, bmax) = mesh.bounding_box();
        let dx = bmax.x - bmin.x;
        let dy = bmax.y - bmin.y;
        let dz = bmax.z - bmin.z;
        dx.max(dy).max(dz) // Search the full bounding box
    };

    let candidates = bvh.closest_point(point, search_radius);

    let mut best_dist = f64::MAX;
    let mut best_point = *point;

    for &tri_idx in &candidates {
        let tri = mesh.triangles[tri_idx];
        let v0 = mesh.vertices[tri[0] as usize];
        let v1 = mesh.vertices[tri[1] as usize];
        let v2 = mesh.vertices[tri[2] as usize];

        let (closest, dist) = closest_point_on_triangle(point, &v0, &v1, &v2);
        if dist < best_dist {
            best_dist = dist;
            best_point = closest;
        }
    }

    (best_point, best_dist)
}

/// Find the closest point on a triangle to a query point.
///
/// Uses the Voronoi region method for exact closest-point computation.
fn closest_point_on_triangle(
    point: &Point3d,
    v0: &Point3d,
    v1: &Point3d,
    v2: &Point3d,
) -> (Point3d, f64) {
    let e0x = v1.x - v0.x;
    let e0y = v1.y - v0.y;
    let e0z = v1.z - v0.z;
    let e1x = v2.x - v0.x;
    let e1y = v2.y - v0.y;
    let e1z = v2.z - v0.z;
    let dx = v0.x - point.x;
    let dy = v0.y - point.y;
    let dz = v0.z - point.z;

    let a = e0x * e0x + e0y * e0y + e0z * e0z;
    let b = e0x * e1x + e0y * e1y + e0z * e1z;
    let c = e1x * e1x + e1y * e1y + e1z * e1z;
    let d = e0x * dx + e0y * dy + e0z * dz;
    let e = e1x * dx + e1y * dy + e1z * dz;

    let det = a * c - b * b;
    let s = if det.abs() > 1e-20 { b * e - c * d } else { 0.0 };
    let t = if det.abs() > 1e-20 { b * d - a * e } else { 0.0 };

    // Classify the region and clamp s, t
    let (s, t) = if s + t <= det {
        if s < 0.0 {
            if t < 0.0 {
                // Region 4
                if d < 0.0 {
                    let t_val = (-d / a).clamp(0.0, 1.0);
                    (t_val, 0.0)
                } else {
                    let s_val = (-e / c).clamp(0.0, 1.0);
                    (0.0, s_val)
                }
            } else {
                // Region 3
                (0.0, (-e / c).clamp(0.0, 1.0))
            }
        } else if t < 0.0 {
            // Region 5
            ((-d / a).clamp(0.0, 1.0), 0.0)
        } else {
            // Region 0
            let inv_det = 1.0 / det;
            (s * inv_det, t * inv_det)
        }
    } else {
        if s < 0.0 {
            // Region 2
            let tmp0 = b + d;
            let tmp1 = c + e;
            if tmp1 > tmp0 {
                let numer = tmp1 - tmp0;
                let denom = a - 2.0 * b + c;
                let s_val = if denom.abs() > 1e-20 {
                    (numer / denom).clamp(0.0, 1.0)
                } else {
                    0.5
                };
                (s_val, 1.0 - s_val)
            } else {
                (0.0, (-e / c).clamp(0.0, 1.0))
            }
        } else if t < 0.0 {
            // Region 6
            let tmp0 = b + e;
            let tmp1 = a + d;
            if tmp1 > tmp0 {
                let numer = tmp1 - tmp0;
                let denom = a - 2.0 * b + c;
                let s_val = if denom.abs() > 1e-20 {
                    (numer / denom).clamp(0.0, 1.0)
                } else {
                    0.5
                };
                (s_val, 1.0 - s_val)
            } else {
                ((-d / a).clamp(0.0, 1.0), 0.0)
            }
        } else {
            // Region 1
            let numer = (c + e) - (b + d);
            let denom = a - 2.0 * b + c;
            let s_val = if denom.abs() > 1e-20 {
                (numer / denom).clamp(0.0, 1.0)
            } else {
                0.5
            };
            (s_val, 1.0 - s_val)
        }
    };

    let closest = Point3d::new(
        v0.x + s * e0x + t * e1x,
        v0.y + s * e0y + t * e1y,
        v0.z + s * e0z + t * e1z,
    );

    let dist = {
        let cx = closest.x - point.x;
        let cy = closest.y - point.y;
        let cz = closest.z - point.z;
        (cx * cx + cy * cy + cz * cz).sqrt()
    };

    (closest, dist)
}

// ============================================================
// 5.2.3 Self-Intersection Check
// ============================================================

/// A self-intersection found between two triangles.
#[derive(Clone, Debug)]
pub struct SelfIntersection {
    /// Index of the first triangle.
    pub triangle_a: usize,
    /// Index of the second triangle.
    pub triangle_b: usize,
    /// Intersection line segment (two endpoints), if computable.
    pub intersection_segment: Option<[Point3d; 2]>,
}

/// Result of a self-intersection check.
#[derive(Clone, Debug)]
pub struct SelfIntersectionResult {
    /// All self-intersections found.
    pub intersections: Vec<SelfIntersection>,
    /// Whether the mesh passes (no self-intersections).
    pub passed: bool,
    /// Human-readable summary.
    pub summary: String,
}

/// Check a mesh for self-intersections.
///
/// For each pair of non-adjacent triangles (not sharing an edge),
/// checks if they intersect. Uses a BVH for acceleration.
///
/// This is O(n²) in the worst case but BVH acceleration makes it
/// practical for typical meshes.
pub fn check_self_intersections(mesh: &TriangleMesh) -> SelfIntersectionResult {
    if mesh.triangles.is_empty() {
        return SelfIntersectionResult {
            intersections: Vec::new(),
            passed: true,
            summary: "Empty mesh — no self-intersections possible".to_string(),
        };
    }

    // Build adjacency information: for each edge, which triangles share it
    let mut edge_to_tris: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (ti, tri) in mesh.triangles.iter().enumerate() {
        let v0 = tri[0];
        let v1 = tri[1];
        let v2 = tri[2];
        let e01 = if v0 < v1 { (v0, v1) } else { (v1, v0) };
        let e12 = if v1 < v2 { (v1, v2) } else { (v2, v1) };
        let e20 = if v2 < v0 { (v2, v0) } else { (v0, v2) };
        edge_to_tris.entry(e01).or_default().push(ti);
        edge_to_tris.entry(e12).or_default().push(ti);
        edge_to_tris.entry(e20).or_default().push(ti);
    }

    // Build vertex-to-triangles for adjacency check
    let mut vert_to_tris: HashMap<u32, Vec<usize>> = HashMap::new();
    for (ti, tri) in mesh.triangles.iter().enumerate() {
        vert_to_tris.entry(tri[0]).or_default().push(ti);
        vert_to_tris.entry(tri[1]).or_default().push(ti);
        vert_to_tris.entry(tri[2]).or_default().push(ti);
    }

    // Build BVH for acceleration
    let bvh = draper_topology::Bvh::build(&mesh.vertices, &mesh.triangles);

    let mut intersections = Vec::new();
    let mut checked_pairs: std::collections::HashSet<(usize, usize)> = std::collections::HashSet::new();

    // For each triangle, find potential overlapping triangles via BVH
    for (ti, tri) in mesh.triangles.iter().enumerate() {
        let v0 = mesh.vertices[tri[0] as usize];
        let v1 = mesh.vertices[tri[1] as usize];
        let v2 = mesh.vertices[tri[2] as usize];

        // Compute triangle bounding box
        let tri_bmin = Point3d::new(
            v0.x.min(v1.x).min(v2.x),
            v0.y.min(v1.y).min(v2.y),
            v0.z.min(v1.z).min(v2.z),
        );
        let tri_bmax = Point3d::new(
            v0.x.max(v1.x).max(v2.x),
            v0.y.max(v1.y).max(v2.y),
            v0.z.max(v1.z).max(v2.z),
        );

        // Expand slightly to avoid missing edge-on-edge contacts
        let eps = 1e-10;
        let tri_bmin_expanded = Point3d::new(
            tri_bmin.x - eps, tri_bmin.y - eps, tri_bmin.z - eps,
        );
        let tri_bmax_expanded = Point3d::new(
            tri_bmax.x + eps, tri_bmax.y + eps, tri_bmax.z + eps,
        );

        // Find overlapping triangles via BVH
        let candidates = find_triangles_in_bbox(&bvh, &tri_bmin_expanded, &tri_bmax_expanded);

        for &tj in &candidates {
            if tj <= ti {
                continue; // Avoid duplicate pairs
            }

            let pair = (ti, tj);
            if checked_pairs.contains(&pair) {
                continue;
            }
            checked_pairs.insert(pair);

            // Check if triangles are adjacent (share an edge or vertex)
            // Vertex-adjacent triangles naturally meet at a point and should
            // not be considered self-intersecting
            if are_adjacent(ti, tj, &mesh.triangles) || share_vertex(ti, tj, &mesh.triangles) {
                continue;
            }

            // Test for intersection
            let tri_j = mesh.triangles[tj];
            let v3 = mesh.vertices[tri_j[0] as usize];
            let v4 = mesh.vertices[tri_j[1] as usize];
            let v5 = mesh.vertices[tri_j[2] as usize];

            if triangles_intersect_approx(&v0, &v1, &v2, &v3, &v4, &v5) {
                intersections.push(SelfIntersection {
                    triangle_a: ti,
                    triangle_b: tj,
                    intersection_segment: None, // Exact segment computation is complex
                });
            }
        }
    }

    let passed = intersections.is_empty();
    let summary = if passed {
        "PASSED — no self-intersections found".to_string()
    } else {
        format!("FAILED — {} self-intersections found", intersections.len())
    };

    SelfIntersectionResult {
        intersections,
        passed,
        summary,
    }
}

/// Check if two triangles are adjacent (share an edge).
fn are_adjacent(ti: usize, tj: usize, triangles: &[[u32; 3]]) -> bool {
    let tri_a = triangles[ti];
    let tri_b = triangles[tj];

    // Get edges of tri_a
    let edges_a = [
        (tri_a[0].min(tri_a[1]), tri_a[0].max(tri_a[1])),
        (tri_a[1].min(tri_a[2]), tri_a[1].max(tri_a[2])),
        (tri_a[2].min(tri_a[0]), tri_a[2].max(tri_a[0])),
    ];
    let edges_b = [
        (tri_b[0].min(tri_b[1]), tri_b[0].max(tri_b[1])),
        (tri_b[1].min(tri_b[2]), tri_b[1].max(tri_b[2])),
        (tri_b[2].min(tri_b[0]), tri_b[2].max(tri_b[0])),
    ];

    for ea in &edges_a {
        for eb in &edges_b {
            if ea == eb {
                return true;
            }
        }
    }
    false
}

/// Check if two triangles share a vertex.
fn share_vertex(ti: usize, tj: usize, triangles: &[[u32; 3]]) -> bool {
    let tri_a = triangles[ti];
    let tri_b = triangles[tj];

    for va in &tri_a {
        for vb in &tri_b {
            if va == vb {
                return true;
            }
        }
    }
    false
}

/// Find all triangle indices whose bounding boxes overlap with the given AABB.
fn find_triangles_in_bbox(
    bvh: &draper_topology::Bvh,
    bmin: &Point3d,
    bmax: &Point3d,
) -> Vec<usize> {
    let mut result = Vec::new();
    find_triangles_in_bbox_node(&bvh.root, &bvh.vertices, &bvh.triangles, bmin, bmax, &mut result);
    result
}

fn find_triangles_in_bbox_node(
    node: &draper_topology::BvhNode,
    vertices: &[Point3d],
    triangles: &[[u32; 3]],
    bmin: &Point3d,
    bmax: &Point3d,
    result: &mut Vec<usize>,
) {
    // Check if BVH node AABB overlaps with query AABB
    if node.bbox_max.x < bmin.x || node.bbox_min.x > bmax.x
        || node.bbox_max.y < bmin.y || node.bbox_min.y > bmax.y
        || node.bbox_max.z < bmin.z || node.bbox_min.z > bmax.z
    {
        return;
    }

    if let Some(ref indices) = node.triangle_indices {
        // Leaf node — check each triangle's bbox
        for &idx in indices {
            let tri = triangles[idx];
            let v0 = vertices[tri[0] as usize];
            let v1 = vertices[tri[1] as usize];
            let v2 = vertices[tri[2] as usize];

            let tri_min = Point3d::new(
                v0.x.min(v1.x).min(v2.x),
                v0.y.min(v1.y).min(v2.y),
                v0.z.min(v1.z).min(v2.z),
            );
            let tri_max = Point3d::new(
                v0.x.max(v1.x).max(v2.x),
                v0.y.max(v1.y).max(v2.y),
                v0.z.max(v1.z).max(v2.z),
            );

            if tri_max.x >= bmin.x && tri_min.x <= bmax.x
                && tri_max.y >= bmin.y && tri_min.y <= bmax.y
                && tri_max.z >= bmin.z && tri_min.z <= bmax.z
            {
                result.push(idx);
            }
        }
    } else {
        if let Some(ref left) = node.left {
            find_triangles_in_bbox_node(left, vertices, triangles, bmin, bmax, result);
        }
        if let Some(ref right) = node.right {
            find_triangles_in_bbox_node(right, vertices, triangles, bmin, bmax, result);
        }
    }
}

/// Test if two triangles approximately intersect.
///
/// Uses a vertex-projection approach: checks if any vertex of one triangle
/// penetrates the plane of the other triangle and lies within its projection.
/// Also checks edge-plane intersections for coplanar edge-crossing cases.
fn triangles_intersect_approx(
    v0: &Point3d, v1: &Point3d, v2: &Point3d,
    u0: &Point3d, u1: &Point3d, u2: &Point3d,
) -> bool {
    let eps = 1e-6;

    // Self-intersection is when two triangles physically cross through
    // each other. We detect this by checking if any edge of one triangle
    // crosses through the plane of the other triangle, with the intersection
    // point falling within the triangle's bounds.
    //
    // We do NOT check if vertices are "inside" the other triangle's volume,
    // as this produces false positives for adjacent faces of closed solids
    // where vertices naturally lie behind neighboring faces' planes.

    // Check edges of triangle A crossing triangle B
    if edge_crosses_triangle(v0, v1, u0, u1, u2, eps) { return true; }
    if edge_crosses_triangle(v1, v2, u0, u1, u2, eps) { return true; }
    if edge_crosses_triangle(v2, v0, u0, u1, u2, eps) { return true; }

    // Check edges of triangle B crossing triangle A
    if edge_crosses_triangle(u0, u1, v0, v1, v2, eps) { return true; }
    if edge_crosses_triangle(u1, u2, v0, v1, v2, eps) { return true; }
    if edge_crosses_triangle(u2, u0, v0, v1, v2, eps) { return true; }

    false
}

/// Check if an edge segment crosses through a triangle's plane within the triangle's bounds.
fn edge_crosses_triangle(
    a0: &Point3d, a1: &Point3d,
    t0: &Point3d, t1: &Point3d, t2: &Point3d,
    eps: f64,
) -> bool {
    // Compute triangle plane normal
    let e1 = (t1.x - t0.x, t1.y - t0.y, t1.z - t0.z);
    let e2 = (t2.x - t0.x, t2.y - t0.y, t2.z - t0.z);
    let n = (
        e1.1 * e2.2 - e1.2 * e2.1,
        e1.2 * e2.0 - e1.0 * e2.2,
        e1.0 * e2.1 - e1.1 * e2.0,
    );

    let d0 = n.0 * a0.x + n.1 * a0.y + n.2 * a0.z
           - (n.0 * t0.x + n.1 * t0.y + n.2 * t0.z);
    let d1 = n.0 * a1.x + n.1 * a1.y + n.2 * a1.z
           - (n.0 * t0.x + n.1 * t0.y + n.2 * t0.z);

    // If both endpoints are on the same side of the plane, no crossing
    if d0 > eps && d1 > eps { return false; }
    if d0 < -eps && d1 < -eps { return false; }

    // If both are essentially on the plane (coplanar), skip —
    // this would be handled by coplanar intersection if needed
    if d0.abs() < eps && d1.abs() < eps { return false; }

    // Find the intersection point: interpolate along the edge
    let t = d0 / (d0 - d1);
    if t < -eps || t > 1.0 + eps { return false; }

    let ix = a0.x + t * (a1.x - a0.x);
    let iy = a0.y + t * (a1.y - a0.y);
    let iz = a0.z + t * (a1.z - a0.z);

    // Check if the intersection point is inside the triangle
    point_in_triangle_3d(ix, iy, iz, t0, t1, t2, eps)
}

/// Check if a point is inside a triangle using 3D barycentric coordinates.
fn point_in_triangle_3d(px: f64, py: f64, pz: f64, t0: &Point3d, t1: &Point3d, t2: &Point3d, eps: f64) -> bool {
    // Use the cross-product method
    let v0x = t2.x - t0.x; let v0y = t2.y - t0.y; let v0z = t2.z - t0.z;
    let v1x = t1.x - t0.x; let v1y = t1.y - t0.y; let v1z = t1.z - t0.z;
    let v2x = px - t0.x;   let v2y = py - t0.y;   let v2z = pz - t0.z;

    let dot00 = v0x*v0x + v0y*v0y + v0z*v0z;
    let dot01 = v0x*v1x + v0y*v1y + v0z*v1z;
    let dot02 = v0x*v2x + v0y*v2y + v0z*v2z;
    let dot11 = v1x*v1x + v1y*v1y + v1z*v1z;
    let dot12 = v1x*v2x + v1y*v2y + v1z*v2z;

    let denom = dot00 * dot11 - dot01 * dot01;
    if denom.abs() < eps * eps { return false; }

    let inv = 1.0 / denom;
    let u = (dot11 * dot02 - dot01 * dot12) * inv;
    let v = (dot00 * dot12 - dot01 * dot02) * inv;

    u >= -eps && v >= -eps && (u + v) <= 1.0 + eps
}

/// Check if a vertex penetrates a triangle's plane and is within the triangle.
/// A vertex "penetrates" if it is on the inside (negative) side of the
/// triangle's outward-facing plane and within the triangle's 2D projection.
/// Vertices that are exactly ON the plane (boundary touch) are NOT considered
/// penetrating — this avoids false positives at shared geometric edges.
fn vertex_penetrates_triangle(
    vertex: &Point3d,
    t0: &Point3d, t1: &Point3d, t2: &Point3d,
    eps: f64,
) -> bool {
    // Compute triangle plane normal
    let e1 = (t1.x - t0.x, t1.y - t0.y, t1.z - t0.z);
    let e2 = (t2.x - t0.x, t2.y - t0.y, t2.z - t0.z);
    let n = (
        e1.1 * e2.2 - e1.2 * e2.1,
        e1.2 * e2.0 - e1.0 * e2.2,
        e1.0 * e2.1 - e1.1 * e2.0,
    );
    let n_len = (n.0 * n.0 + n.1 * n.1 + n.2 * n.2).sqrt();
    if n_len < eps {
        return false;
    }

    // Signed distance from vertex to triangle plane (normalized)
    let d = (n.0 * vertex.x + n.1 * vertex.y + n.2 * vertex.z
           - (n.0 * t0.x + n.1 * t0.y + n.2 * t0.z)) / n_len;

    eprintln!("    vertex=({},{},{}) d={:.6} n_len={:.6}", vertex.x, vertex.y, vertex.z, d, n_len);

    // If vertex is outside or ON the plane, it's not penetrating.
    if d > -1e-4 {
        return false;
    }

    // Project vertex onto triangle plane and check if inside
    // Use 3D barycentric coordinates
    let v0x = t2.x - t0.x; let v0y = t2.y - t0.y; let v0z = t2.z - t0.z;
    let v1x = t1.x - t0.x; let v1y = t1.y - t0.y; let v1z = t1.z - t0.z;
    let v2x = vertex.x - t0.x; let v2y = vertex.y - t0.y; let v2z = vertex.z - t0.z;

    let dot00 = v0x*v0x + v0y*v0y + v0z*v0z;
    let dot01 = v0x*v1x + v0y*v1y + v0z*v1z;
    let dot02 = v0x*v2x + v0y*v2y + v0z*v2z;
    let dot11 = v1x*v1x + v1y*v1y + v1z*v1z;
    let dot12 = v1x*v2x + v1y*v2y + v1z*v2z;

    let denom = dot00 * dot11 - dot01 * dot01;
    if denom.abs() < eps * eps { return false; }

    let inv = 1.0 / denom;
    let u = (dot11 * dot02 - dot01 * dot12) * inv;
    let v = (dot00 * dot12 - dot01 * dot02) * inv;

    u >= -eps && v >= -eps && (u + v) <= 1.0 + eps
}

/// Test coplanar triangle intersection (simplified).
fn coplanar_triangles_intersect(
    v0: &Point3d, v1: &Point3d, v2: &Point3d,
    u0: &Point3d, u1: &Point3d, u2: &Point3d,
    _normal: &(f64, f64, f64),
) -> bool {
    // Check if any edge of triangle 1 intersects any edge of triangle 2
    let edges1 = [(v0, v1), (v1, v2), (v2, v0)];
    let edges2 = [(u0, u1), (u1, u2), (u2, u0)];

    for (a0, a1) in &edges1 {
        for (b0, b1) in &edges2 {
            if segments_intersect_3d(a0, a1, b0, b1) {
                return true;
            }
        }
    }

    // Check containment (one triangle inside the other)
    point_in_triangle_2d(v0, u0, u1, u2)
        || point_in_triangle_2d(u0, v0, v1, v2)
}

/// Test if two 3D line segments intersect (approximate).
fn segments_intersect_3d(a0: &Point3d, a1: &Point3d, b0: &Point3d, b1: &Point3d) -> bool {
    // Compute shortest distance between the two line segments
    let (dist, _) = segment_segment_distance(a0, a1, b0, b1);
    dist < 1e-8
}

/// Compute the shortest distance between two line segments.
fn segment_segment_distance(
    p1: &Point3d, p2: &Point3d,
    p3: &Point3d, p4: &Point3d,
) -> (f64, (f64, f64)) {
    let d13x = p3.x - p1.x;
    let d13y = p3.y - p1.y;
    let d13z = p3.z - p1.z;
    let d43x = p4.x - p3.x;
    let d43y = p4.y - p3.y;
    let d43z = p4.z - p3.z;
    let d21x = p2.x - p1.x;
    let d21y = p2.y - p1.y;
    let d21z = p2.z - p1.z;

    let d1343 = d13x * d43x + d13y * d43y + d13z * d43z;
    let d4321 = d43x * d21x + d43y * d21y + d43z * d21z;
    let d1321 = d13x * d21x + d13y * d21y + d13z * d21z;
    let d4343 = d43x * d43x + d43y * d43y + d43z * d43z;
    let d2121 = d21x * d21x + d21y * d21y + d21z * d21z;

    let denom = d2121 * d4343 - d4321 * d4321;
    let numer = d1343 * d4321 - d1321 * d4343;

    let (mu_a, mu_b) = if denom.abs() < 1e-20 {
        (0.0, if d4343.abs() < 1e-20 { 0.0 } else { d1343 / d4343 })
    } else {
        let ma = numer / denom;
        let mb = (d1343 + ma * d4321) / if d4343.abs() < 1e-20 { 1.0 } else { d4343 };
        (ma, mb)
    };

    let mu_a = mu_a.clamp(0.0, 1.0);
    let mu_b = mu_b.clamp(0.0, 1.0);

    let cx = p1.x + mu_a * d21x - p3.x - mu_b * d43x;
    let cy = p1.y + mu_a * d21y - p3.y - mu_b * d43y;
    let cz = p1.z + mu_a * d21z - p3.z - mu_b * d43z;

    ((cx * cx + cy * cy + cz * cz).sqrt(), (mu_a, mu_b))
}

/// Check if a point is inside a triangle (2D projection).
fn point_in_triangle_2d(p: &Point3d, a: &Point3d, b: &Point3d, c: &Point3d) -> bool {
    // Use the sign of cross products in the most dominant projection plane
    let v0x = c.x - a.x;
    let v0y = c.y - a.y;
    let v1x = b.x - a.x;
    let v1y = b.y - a.y;
    let v2x = p.x - a.x;
    let v2y = p.y - a.y;

    let dot00 = v0x * v0x + v0y * v0y;
    let dot01 = v0x * v1x + v0y * v1y;
    let dot02 = v0x * v2x + v0y * v2y;
    let dot11 = v1x * v1x + v1y * v1y;
    let dot12 = v1x * v2x + v1y * v2y;

    let inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    let u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
    let v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

    u >= 0.0 && v >= 0.0 && (u + v) <= 1.0
}

// ============================================================
// 5.2.4 Mesh Quality Control
// ============================================================

/// Quality criterion result.
#[derive(Clone, Debug)]
pub struct QualityCriterion {
    /// Name of the criterion.
    pub name: String,
    /// Whether this criterion passed.
    pub passed: bool,
    /// The measured value.
    pub measured_value: f64,
    /// The required threshold.
    pub threshold: f64,
    /// Human-readable description.
    pub description: String,
}

/// Report from a mesh quality check.
#[derive(Clone, Debug)]
pub struct MeshQualityReport {
    /// Individual quality criteria.
    pub criteria: Vec<QualityCriterion>,
    /// Whether all criteria passed.
    pub passed: bool,
    /// Number of triangles with minimum angle below threshold.
    pub degenerate_triangles: usize,
    /// Number of sliver triangles (high aspect ratio).
    pub sliver_triangles: usize,
    /// Number of duplicate vertex pairs.
    pub duplicate_vertices: usize,
    /// Number of inverted triangles (inconsistent winding).
    pub inverted_triangles: usize,
    /// Total number of triangles.
    pub triangle_count: usize,
    /// Total number of vertices.
    pub vertex_count: usize,
    /// Human-readable summary.
    pub summary: String,
}

/// Parameters for mesh quality checking.
#[derive(Clone, Debug)]
pub struct MeshQualityParams {
    /// Minimum angle in degrees (default: 5°).
    pub min_angle_degrees: f64,
    /// Maximum aspect ratio (default: 100:1).
    pub max_aspect_ratio: f64,
    /// Minimum edge length (default: 1e-6).
    pub min_edge_length: f64,
    /// Tolerance for duplicate vertex detection (default: 1e-6).
    pub duplicate_tolerance: f64,
}

impl Default for MeshQualityParams {
    fn default() -> Self {
        Self {
            min_angle_degrees: 5.0,
            max_aspect_ratio: 100.0,
            min_edge_length: 1e-6,
            duplicate_tolerance: 1e-6,
        }
    }
}

/// Check mesh quality for additive manufacturing.
///
/// Verifies:
/// - Minimum angle > 5° (no degenerate triangles)
/// - Maximum aspect ratio < 100:1 (no slivers)
/// - Minimum edge length > tolerance
/// - Consistent winding order (all normals outward)
/// - No duplicate vertices within tolerance
///
/// Returns a detailed report with pass/fail for each criterion.
pub fn check_mesh_quality(mesh: &TriangleMesh) -> MeshQualityReport {
    check_mesh_quality_with_params(mesh, &MeshQualityParams::default())
}

/// Check mesh quality with custom parameters.
pub fn check_mesh_quality_with_params(
    mesh: &TriangleMesh,
    params: &MeshQualityParams,
) -> MeshQualityReport {
    let mut criteria = Vec::new();
    let triangle_count = mesh.triangles.len();
    let vertex_count = mesh.vertices.len();

    if triangle_count == 0 || vertex_count == 0 {
        return MeshQualityReport {
            criteria: vec![QualityCriterion {
                name: "Non-empty mesh".to_string(),
                passed: false,
                measured_value: triangle_count as f64,
                threshold: 1.0,
                description: "Mesh must have at least one triangle".to_string(),
            }],
            passed: false,
            degenerate_triangles: 0,
            sliver_triangles: 0,
            duplicate_vertices: 0,
            inverted_triangles: 0,
            triangle_count,
            vertex_count,
            summary: "FAILED — empty mesh".to_string(),
        };
    }

    // Ensure face normals are computed
    let mut mesh_owned = mesh.clone();
    if mesh_owned.face_normals.is_none() {
        mesh_owned.compute_face_normals();
    }
    let face_normals = mesh_owned.face_normals.as_ref().unwrap();

    // ---- Criterion 1: Minimum angle ----
    let mut min_angle = f64::MAX;
    let mut degenerate_count = 0;

    for tri in &mesh.triangles {
        let v0 = mesh.vertices[tri[0] as usize];
        let v1 = mesh.vertices[tri[1] as usize];
        let v2 = mesh.vertices[tri[2] as usize];

        let angles = triangle_angles(&v0, &v1, &v2);
        let tri_min = angles[0].min(angles[1]).min(angles[2]);

        if tri_min < min_angle {
            min_angle = tri_min;
        }
        if tri_min < params.min_angle_degrees {
            degenerate_count += 1;
        }
    }

    criteria.push(QualityCriterion {
        name: "Minimum angle".to_string(),
        passed: degenerate_count == 0,
        measured_value: min_angle,
        threshold: params.min_angle_degrees,
        description: format!(
            "{} triangles with angle < {:.1}° (min angle: {:.2}°)",
            degenerate_count, params.min_angle_degrees, min_angle
        ),
    });

    // ---- Criterion 2: Maximum aspect ratio ----
    let mut max_aspect_ratio = 0.0f64;
    let mut sliver_count = 0;

    for tri in &mesh.triangles {
        let v0 = mesh.vertices[tri[0] as usize];
        let v1 = mesh.vertices[tri[1] as usize];
        let v2 = mesh.vertices[tri[2] as usize];

        let ar = triangle_aspect_ratio(&v0, &v1, &v2);
        if ar > max_aspect_ratio {
            max_aspect_ratio = ar;
        }
        if ar > params.max_aspect_ratio {
            sliver_count += 1;
        }
    }

    criteria.push(QualityCriterion {
        name: "Maximum aspect ratio".to_string(),
        passed: sliver_count == 0,
        measured_value: max_aspect_ratio,
        threshold: params.max_aspect_ratio,
        description: format!(
            "{} sliver triangles with ratio > {:.0}:1 (max ratio: {:.1}:1)",
            sliver_count, params.max_aspect_ratio, max_aspect_ratio
        ),
    });

    // ---- Criterion 3: Minimum edge length ----
    let mut min_edge_length = f64::MAX;
    let mut short_edge_count = 0;

    for tri in &mesh.triangles {
        let edges = [
            edge_length(&mesh.vertices[tri[0] as usize], &mesh.vertices[tri[1] as usize]),
            edge_length(&mesh.vertices[tri[1] as usize], &mesh.vertices[tri[2] as usize]),
            edge_length(&mesh.vertices[tri[2] as usize], &mesh.vertices[tri[0] as usize]),
        ];

        for &len in &edges {
            if len < min_edge_length {
                min_edge_length = len;
            }
            if len < params.min_edge_length {
                short_edge_count += 1;
            }
        }
    }

    criteria.push(QualityCriterion {
        name: "Minimum edge length".to_string(),
        passed: short_edge_count == 0,
        measured_value: min_edge_length,
        threshold: params.min_edge_length,
        description: format!(
            "{} edges shorter than {:.1e} (min: {:.2e})",
            short_edge_count, params.min_edge_length, min_edge_length
        ),
    });

    // ---- Criterion 4: Consistent winding order ----
    let inverted_indices = count_inverted_triangles(mesh, face_normals);
    let inverted_count = inverted_indices.len();

    criteria.push(QualityCriterion {
        name: "Consistent winding".to_string(),
        passed: inverted_count == 0,
        measured_value: inverted_count as f64,
        threshold: 0.0,
        description: format!(
            "{} triangles with inconsistent winding order",
            inverted_count
        ),
    });

    // ---- Criterion 5: Duplicate vertices ----
    let duplicate_count = count_duplicate_vertices(mesh, params.duplicate_tolerance);

    criteria.push(QualityCriterion {
        name: "No duplicate vertices".to_string(),
        passed: duplicate_count == 0,
        measured_value: duplicate_count as f64,
        threshold: 0.0,
        description: format!(
            "{} duplicate vertex pairs (tolerance: {:.1e})",
            duplicate_count, params.duplicate_tolerance
        ),
    });

    let passed = criteria.iter().all(|c| c.passed);

    let summary = if passed {
        format!(
            "PASSED — {} vertices, {} triangles, all quality criteria met",
            vertex_count, triangle_count
        )
    } else {
        let failed: Vec<&str> = criteria.iter()
            .filter(|c| !c.passed)
            .map(|c| c.name.as_str())
            .collect();
        format!(
            "FAILED — {} vertices, {} triangles, failed: [{}]",
            vertex_count, triangle_count, failed.join(", ")
        )
    };

    MeshQualityReport {
        criteria,
        passed,
        degenerate_triangles: degenerate_count,
        sliver_triangles: sliver_count,
        duplicate_vertices: duplicate_count,
        inverted_triangles: inverted_count,
        triangle_count,
        vertex_count,
        summary,
    }
}

/// Compute the three angles of a triangle in degrees.
fn triangle_angles(v0: &Point3d, v1: &Point3d, v2: &Point3d) -> [f64; 3] {
    let a = edge_length(v1, v2); // side opposite v0
    let b = edge_length(v0, v2); // side opposite v1
    let c = edge_length(v0, v1); // side opposite v2

    if a < 1e-20 || b < 1e-20 || c < 1e-20 {
        return [0.0, 0.0, 0.0]; // Degenerate
    }

    // Law of cosines
    let cos_a = ((b * b + c * c - a * a) / (2.0 * b * c)).clamp(-1.0, 1.0);
    let cos_b = ((a * a + c * c - b * b) / (2.0 * a * c)).clamp(-1.0, 1.0);
    let cos_c = ((a * a + b * b - c * c) / (2.0 * a * b)).clamp(-1.0, 1.0);

    [
        cos_a.acos().to_degrees(),
        cos_b.acos().to_degrees(),
        cos_c.acos().to_degrees(),
    ]
}

/// Compute the aspect ratio of a triangle.
///
/// Defined as the longest edge divided by the shortest altitude.
/// A perfectly equilateral triangle has aspect ratio ≈ 1.15.
fn triangle_aspect_ratio(v0: &Point3d, v1: &Point3d, v2: &Point3d) -> f64 {
    let a = edge_length(v1, v2);
    let b = edge_length(v0, v2);
    let c = edge_length(v0, v1);

    let longest_edge = a.max(b).max(c);

    // Compute altitudes
    let area = triangle_area(v0, v1, v2);
    if area < 1e-20 {
        return f64::MAX; // Degenerate triangle
    }

    let h_a = 2.0 * area / a;
    let h_b = 2.0 * area / b;
    let h_c = 2.0 * area / c;

    let shortest_altitude = h_a.min(h_b).min(h_c);

    if shortest_altitude < 1e-20 {
        return f64::MAX;
    }

    longest_edge / shortest_altitude
}

/// Compute the area of a triangle.
fn triangle_area(v0: &Point3d, v1: &Point3d, v2: &Point3d) -> f64 {
    let e1x = v1.x - v0.x;
    let e1y = v1.y - v0.y;
    let e1z = v1.z - v0.z;
    let e2x = v2.x - v0.x;
    let e2y = v2.y - v0.y;
    let e2z = v2.z - v0.z;
    let cx = e1y * e2z - e1z * e2y;
    let cy = e1z * e2x - e1x * e2z;
    let cz = e1x * e2y - e1y * e2x;
    (cx * cx + cy * cy + cz * cz).sqrt() * 0.5
}

/// Compute the length of an edge.
fn edge_length(a: &Point3d, b: &Point3d) -> f64 {
    ((b.x - a.x).powi(2) + (b.y - a.y).powi(2) + (b.z - a.z).powi(2)).sqrt()
}

/// Count triangles with inconsistent winding order.
///
/// Uses the face normals: if a triangle's face normal points in the opposite
/// direction from the average normal of its edge-neighbor triangles, it has
/// inconsistent winding.
fn count_inverted_triangles(mesh: &TriangleMesh, face_normals: &[[f64; 3]]) -> Vec<usize> {
    if mesh.triangles.len() < 2 {
        return Vec::new();
    }

    // Build triangle adjacency: for each edge, which triangles share it
    let mut edge_to_tris: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (ti, tri) in mesh.triangles.iter().enumerate() {
        let e01 = if tri[0] < tri[1] { (tri[0], tri[1]) } else { (tri[1], tri[0]) };
        let e12 = if tri[1] < tri[2] { (tri[1], tri[2]) } else { (tri[2], tri[1]) };
        let e20 = if tri[2] < tri[0] { (tri[2], tri[0]) } else { (tri[0], tri[2]) };
        edge_to_tris.entry(e01).or_default().push(ti);
        edge_to_tris.entry(e12).or_default().push(ti);
        edge_to_tris.entry(e20).or_default().push(ti);
    }

    let mut inverted = Vec::new();

    for (ti, tri) in mesh.triangles.iter().enumerate() {
        let ni = face_normals[ti];

        // Find neighbor triangles
        let edges = [
            if tri[0] < tri[1] { (tri[0], tri[1]) } else { (tri[1], tri[0]) },
            if tri[1] < tri[2] { (tri[1], tri[2]) } else { (tri[2], tri[1]) },
            if tri[2] < tri[0] { (tri[2], tri[0]) } else { (tri[0], tri[2]) },
        ];

        let mut neighbor_count = 0;
        let mut consistent_count = 0;

        for edge in &edges {
            if let Some(neighbors) = edge_to_tris.get(edge) {
                for &nj in neighbors {
                    if nj != ti {
                        let nj_normal = face_normals[nj];
                        // Two neighboring triangles should have normals that
                        // roughly point in the same hemisphere (dot product > 0)
                        let dot = ni[0] * nj_normal[0] + ni[1] * nj_normal[1] + ni[2] * nj_normal[2];
                        if dot > 0.0 {
                            consistent_count += 1;
                        }
                        neighbor_count += 1;
                    }
                }
            }
        }

        // If most neighbors have consistent normals, this triangle is fine
        // If most neighbors have inconsistent normals, this triangle is inverted
        if neighbor_count > 0 && consistent_count < neighbor_count / 2 {
            inverted.push(ti);
        }
    }

    inverted
}

/// Count duplicate vertex pairs within the given tolerance.
fn count_duplicate_vertices(mesh: &TriangleMesh, tolerance: f64) -> usize {
    let tol_sq = tolerance * tolerance;
    let mut count = 0;

    // For performance, use spatial hashing
    let cell_size = tolerance * 2.0;
    let mut grid: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();

    for (i, v) in mesh.vertices.iter().enumerate() {
        let cx = (v.x / cell_size).floor() as i64;
        let cy = (v.y / cell_size).floor() as i64;
        let cz = (v.z / cell_size).floor() as i64;
        grid.entry((cx, cy, cz)).or_default().push(i);
    }

    for (i, v) in mesh.vertices.iter().enumerate() {
        let cx = (v.x / cell_size).floor() as i64;
        let cy = (v.y / cell_size).floor() as i64;
        let cz = (v.z / cell_size).floor() as i64;

        // Check 27 neighboring cells
        for dx in -1i64..=1 {
            for dy in -1i64..=1 {
                for dz in -1i64..=1 {
                    if let Some(indices) = grid.get(&(cx + dx, cy + dy, cz + dz)) {
                        for &j in indices {
                            if j > i {
                                let w = &mesh.vertices[j];
                                let dist_sq = (v.x - w.x).powi(2)
                                    + (v.y - w.y).powi(2)
                                    + (v.z - w.z).powi(2);
                                if dist_sq < tol_sq {
                                    count += 1;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    count
}

// ============================================================
// Comprehensive certification
// ============================================================

/// Full additive manufacturing certification result.
#[derive(Clone, Debug)]
pub struct AmCertificationReport {
    /// Watertightness certification (5.2.1).
    pub watertight: CertificationResult,
    /// Wall thickness result (5.2.2).
    pub wall_thickness: WallThicknessResult,
    /// Self-intersection check result (5.2.3).
    pub self_intersections: SelfIntersectionResult,
    /// Mesh quality report (5.2.4).
    pub quality: MeshQualityReport,
    /// Whether the mesh passes ALL certification checks.
    pub passed: bool,
    /// Human-readable summary.
    pub summary: String,
}

/// Run all additive manufacturing certification checks on a mesh.
///
/// This is the main entry point for AM certification. It runs:
/// 1. Watertightness check
/// 2. Wall thickness check (with given minimum thickness)
/// 3. Self-intersection check
/// 4. Mesh quality check
///
/// Returns a comprehensive report with pass/fail for each check.
pub fn certify_additive_manufacturing(
    mesh: &TriangleMesh,
    min_wall_thickness: f64,
) -> AmCertificationReport {
    let watertight = certify_watertight(mesh);
    let wall_thickness = check_wall_thickness_mesh(mesh, min_wall_thickness);
    let self_intersections = check_self_intersections(mesh);
    let quality = check_mesh_quality(mesh);

    let passed = watertight.passed
        && wall_thickness.passed
        && self_intersections.passed
        && quality.passed;

    let summary = if passed {
        "PASSED — mesh is certified for additive manufacturing".to_string()
    } else {
        let mut failures = Vec::new();
        if !watertight.passed { failures.push("watertightness"); }
        if !wall_thickness.passed { failures.push("wall thickness"); }
        if !self_intersections.passed { failures.push("self-intersections"); }
        if !quality.passed { failures.push("mesh quality"); }
        format!("FAILED — certification failed for: [{}]", failures.join(", "))
    };

    AmCertificationReport {
        watertight,
        wall_thickness,
        self_intersections,
        quality,
        passed,
        summary,
    }
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: create a unit cube mesh (closed, watertight).
    fn make_cube_mesh() -> TriangleMesh {
        let mut mesh = TriangleMesh::new();
        let v = [
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 0.0, 0.0),
            Point3d::new(1.0, 1.0, 0.0),
            Point3d::new(0.0, 1.0, 0.0),
            Point3d::new(0.0, 0.0, 1.0),
            Point3d::new(1.0, 0.0, 1.0),
            Point3d::new(1.0, 1.0, 1.0),
            Point3d::new(0.0, 1.0, 1.0),
        ];
        for p in &v {
            mesh.add_vertex(*p);
        }
        // Bottom (z=0) — outward normal -Z
        mesh.add_triangle(0, 2, 1);
        mesh.add_triangle(0, 3, 2);
        // Top (z=1) — outward normal +Z
        mesh.add_triangle(4, 5, 6);
        mesh.add_triangle(4, 6, 7);
        // Front (y=0) — outward normal -Y
        mesh.add_triangle(0, 1, 5);
        mesh.add_triangle(0, 5, 4);
        // Back (y=1) — outward normal +Y
        mesh.add_triangle(3, 7, 6);
        mesh.add_triangle(3, 6, 2);
        // Left (x=0) — outward normal -X
        mesh.add_triangle(0, 4, 7);
        mesh.add_triangle(0, 7, 3);
        // Right (x=1) — outward normal +X
        mesh.add_triangle(1, 2, 6);
        mesh.add_triangle(1, 6, 5);
        mesh.compute_face_normals();
        mesh
    }

    #[test]
    fn test_certify_watertight_cube() {
        let mesh = make_cube_mesh();
        let result = certify_watertight(&mesh);
        assert!(result.passed, "Cube should be watertight: {}", result.summary);
        assert_eq!(result.boundary_edge_count, 0);
        assert_eq!(result.non_manifold_edge_count, 0);
    }

    #[test]
    fn test_certify_watertight_open_mesh() {
        let mut mesh = TriangleMesh::new();
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(0.5, 1.0, 0.0));
        mesh.add_triangle(0, 1, 2);

        let result = certify_watertight(&mesh);
        assert!(!result.passed, "Open mesh should not be watertight");
        assert_eq!(result.boundary_edge_count, 3);
    }

    #[test]
    fn test_wall_thickness_cube() {
        let mesh = make_cube_mesh();
        // A 1x1x1 cube with min_thickness=0.2: interior should pass since
        // the minimum wall thickness from center to face is 0.5.
        // The sampling is shrunk by min_thickness*0.5 from each face,
        // so we sample points at least 0.1 from the surface.
        let result = check_wall_thickness_mesh(&mesh, 0.2);
        assert!(result.passed, "Cube wall thickness should be >= 0.2: {}", result.summary);
    }

    #[test]
    fn test_wall_thickness_thin_shell() {
        // Create a thin shell: two parallel squares close together
        let mut mesh = TriangleMesh::new();
        let thickness = 0.1;
        // Bottom face
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(1.0, 1.0, 0.0));
        mesh.add_vertex(Point3d::new(0.0, 1.0, 0.0));
        // Top face
        mesh.add_vertex(Point3d::new(0.0, 0.0, thickness));
        mesh.add_vertex(Point3d::new(1.0, 0.0, thickness));
        mesh.add_vertex(Point3d::new(1.0, 1.0, thickness));
        mesh.add_vertex(Point3d::new(0.0, 1.0, thickness));

        // Bottom
        mesh.add_triangle(0, 2, 1);
        mesh.add_triangle(0, 3, 2);
        // Top
        mesh.add_triangle(4, 5, 6);
        mesh.add_triangle(4, 6, 7);
        // Sides
        mesh.add_triangle(0, 1, 5);
        mesh.add_triangle(0, 5, 4);
        mesh.add_triangle(1, 2, 6);
        mesh.add_triangle(1, 6, 5);
        mesh.add_triangle(2, 3, 7);
        mesh.add_triangle(2, 7, 6);
        mesh.add_triangle(3, 0, 4);
        mesh.add_triangle(3, 4, 7);

        mesh.compute_face_normals();

        // Check with minimum thickness of 0.2 (should fail - shell is only 0.1 thick)
        let result = check_wall_thickness_mesh(&mesh, 0.2);
        assert!(!result.passed, "Thin shell should fail thickness check: {}", result.summary);
    }

    #[test]
    fn test_self_intersection_no_intersections() {
        let mesh = make_cube_mesh();
        let result = check_self_intersections(&mesh);
        assert!(result.passed, "Cube should have no self-intersections: {}", result.summary);
        assert!(result.intersections.is_empty());
    }

    #[test]
    fn test_self_intersection_with_penetration() {
        // Create two cubes that overlap
        let mut mesh = TriangleMesh::new();

        // Cube 1: 0,0,0 → 1,1,1
        let v = [
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 0.0, 0.0),
            Point3d::new(1.0, 1.0, 0.0),
            Point3d::new(0.0, 1.0, 0.0),
            Point3d::new(0.0, 0.0, 1.0),
            Point3d::new(1.0, 0.0, 1.0),
            Point3d::new(1.0, 1.0, 1.0),
            Point3d::new(0.0, 1.0, 1.0),
        ];
        for p in &v { mesh.add_vertex(*p); }
        mesh.add_triangle(0, 2, 1);
        mesh.add_triangle(0, 3, 2);
        mesh.add_triangle(4, 5, 6);
        mesh.add_triangle(4, 6, 7);
        mesh.add_triangle(0, 1, 5);
        mesh.add_triangle(0, 5, 4);
        mesh.add_triangle(3, 7, 6);
        mesh.add_triangle(3, 6, 2);
        mesh.add_triangle(0, 4, 7);
        mesh.add_triangle(0, 7, 3);
        mesh.add_triangle(1, 2, 6);
        mesh.add_triangle(1, 6, 5);

        // Cube 2: 0.5,0.5,0.5 → 1.5,1.5,1.5 (overlapping)
        let offset = 8u32;
        let v2 = [
            Point3d::new(0.5, 0.5, 0.5),
            Point3d::new(1.5, 0.5, 0.5),
            Point3d::new(1.5, 1.5, 0.5),
            Point3d::new(0.5, 1.5, 0.5),
            Point3d::new(0.5, 0.5, 1.5),
            Point3d::new(1.5, 0.5, 1.5),
            Point3d::new(1.5, 1.5, 1.5),
            Point3d::new(0.5, 1.5, 1.5),
        ];
        for p in &v2 { mesh.add_vertex(*p); }
        mesh.add_triangle(offset + 0, offset + 2, offset + 1);
        mesh.add_triangle(offset + 0, offset + 3, offset + 2);
        mesh.add_triangle(offset + 4, offset + 5, offset + 6);
        mesh.add_triangle(offset + 4, offset + 6, offset + 7);
        mesh.add_triangle(offset + 0, offset + 1, offset + 5);
        mesh.add_triangle(offset + 0, offset + 5, offset + 4);
        mesh.add_triangle(offset + 3, offset + 7, offset + 6);
        mesh.add_triangle(offset + 3, offset + 6, offset + 2);
        mesh.add_triangle(offset + 0, offset + 4, offset + 7);
        mesh.add_triangle(offset + 0, offset + 7, offset + 3);
        mesh.add_triangle(offset + 1, offset + 2, offset + 6);
        mesh.add_triangle(offset + 1, offset + 6, offset + 5);

        mesh.compute_face_normals();

        let result = check_self_intersections(&mesh);
        assert!(!result.passed, "Overlapping cubes should have self-intersections: {}", result.summary);
    }

    #[test]
    fn test_mesh_quality_cube() {
        let mesh = make_cube_mesh();
        let report = check_mesh_quality(&mesh);
        assert!(report.passed, "Cube should pass quality check: {}", report.summary);
    }

    #[test]
    fn test_mesh_quality_degenerate_triangle() {
        let mut mesh = TriangleMesh::new();
        // Create a degenerate triangle (all points collinear)
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(0.5, 0.0, 0.0)); // Collinear
        mesh.add_triangle(0, 1, 2);
        mesh.compute_face_normals();

        let report = check_mesh_quality(&mesh);
        assert!(!report.passed, "Degenerate triangle should fail quality check");
        assert!(report.degenerate_triangles > 0);
    }

    #[test]
    fn test_mesh_quality_custom_params() {
        let mesh = make_cube_mesh();
        let params = MeshQualityParams {
            min_angle_degrees: 30.0, // Very strict
            max_aspect_ratio: 5.0,    // Very strict
            min_edge_length: 0.01,
            duplicate_tolerance: 1e-6,
        };
        let report = check_mesh_quality_with_params(&mesh, &params);
        // A cube has 45° angles, so 30° should be OK, but aspect ratio might fail
        // depending on the specific triangulation
        assert!(report.triangle_count > 0);
    }

    #[test]
    fn test_certify_additive_manufacturing_cube() {
        let mesh = make_cube_mesh();
        let report = certify_additive_manufacturing(&mesh, 0.2);
        assert!(report.watertight.passed, "Cube should be watertight");
        assert!(report.self_intersections.passed, "Cube should have no self-intersections");
        assert!(report.quality.passed, "Cube should pass quality check");
        assert!(report.passed, "Cube should pass AM certification: {}", report.summary);
    }

    #[test]
    fn test_empty_mesh_certification() {
        let mesh = TriangleMesh::new();
        let watertight = certify_watertight(&mesh);
        assert!(watertight.passed, "Empty mesh has 0 boundary edges");
        assert_eq!(watertight.euler_characteristic, 0);
    }

    #[test]
    fn test_triangle_angles() {
        // Equilateral triangle: all angles 60°
        let v0 = Point3d::new(0.0, 0.0, 0.0);
        let v1 = Point3d::new(1.0, 0.0, 0.0);
        let v2 = Point3d::new(0.5, 3.0f64.sqrt() / 2.0, 0.0);
        let angles = triangle_angles(&v0, &v1, &v2);
        for angle in &angles {
            assert!((angle - 60.0).abs() < 1.0, "Expected ~60°, got {:.1}°", angle);
        }
    }

    #[test]
    fn test_aspect_ratio_equilateral() {
        let v0 = Point3d::new(0.0, 0.0, 0.0);
        let v1 = Point3d::new(1.0, 0.0, 0.0);
        let v2 = Point3d::new(0.5, 3.0f64.sqrt() / 2.0, 0.0);
        let ar = triangle_aspect_ratio(&v0, &v1, &v2);
        // Equilateral aspect ratio ≈ 1.15
        assert!(ar < 2.0, "Equilateral aspect ratio should be < 2, got {:.2}", ar);
    }

    #[test]
    fn test_closest_point_on_triangle() {
        let v0 = Point3d::new(0.0, 0.0, 0.0);
        let v1 = Point3d::new(1.0, 0.0, 0.0);
        let v2 = Point3d::new(0.0, 1.0, 0.0);

        // Point directly above the centroid
        let centroid = Point3d::new(1.0 / 3.0, 1.0 / 3.0, 0.0);
        let point = Point3d::new(1.0 / 3.0, 1.0 / 3.0, 1.0);
        let (closest, dist) = closest_point_on_triangle(&point, &v0, &v1, &v2);
        assert!((closest.x - centroid.x).abs() < 1e-6);
        assert!((closest.y - centroid.y).abs() < 1e-6);
        assert!((closest.z - 0.0).abs() < 1e-6);
        assert!((dist - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_duplicate_vertices() {
        let mut mesh = TriangleMesh::new();
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0)); // Duplicate of vertex 0
        mesh.add_triangle(0, 1, 2);

        let count = count_duplicate_vertices(&mesh, 1e-6);
        assert_eq!(count, 1, "Should find 1 duplicate vertex pair");
    }
}
