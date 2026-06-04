// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Parametric domain representation for trimmed surface triangulation.
//!
//! A ParametricDomain represents the 2D region in UV-parameter space
//! that defines the valid area of a trimmed surface. It consists of:
//! - An outer boundary (the trimming loop)
//! - Optional inner boundaries (holes)
//!
//! The domain is triangulated using ear-clipping with bridge-edge hole
//! insertion, which is GUARANTEED to terminate in O(n²) — unlike
//! Constrained Delaunay Triangulation (spade CDT) which can hang
//! inside a single `add_constraint` call on degenerate inputs.

use draper_geometry::{Point2d, Point3d, Surface};
use crate::mesh::TriangleMesh;
use std::f64::consts::PI;

/// A closed polygon in UV parameter space.
pub type UVPolygon = Vec<Point2d>;

/// The parametric domain of a trimmed surface face.
///
/// Defines the valid 2D region in UV space that should be triangulated.
/// The outer boundary defines the exterior contour, and inner boundaries
/// define holes that should be excluded from the triangulation.
#[derive(Clone, Debug)]
pub struct ParametricDomain {
    /// The outer boundary of the domain (counter-clockwise in UV space).
    pub outer_boundary: UVPolygon,
    /// Inner boundaries (holes) — each is a clockwise polygon in UV space.
    pub holes: Vec<UVPolygon>,
    /// The UV range of the surface: (u_min, u_max).
    pub u_range: (f64, f64),
    /// The V range of the surface: (v_min, v_max).
    pub v_range: (f64, f64),
}

impl ParametricDomain {
    /// Create a new parametric domain from an outer boundary.
    pub fn new(outer_boundary: UVPolygon, u_range: (f64, f64), v_range: (f64, f64)) -> Self {
        Self {
            outer_boundary,
            holes: Vec::new(),
            u_range,
            v_range,
        }
    }

    /// Add a hole (inner boundary) to the domain.
    pub fn with_hole(mut self, hole: UVPolygon) -> Self {
        self.holes.push(hole);
        self
    }

    /// Compute the bounding box of the domain.
    pub fn bounding_box(&self) -> (f64, f64, f64, f64) {
        let mut u_min = f64::MAX;
        let mut u_max = f64::MIN;
        let mut v_min = f64::MAX;
        let mut v_max = f64::MIN;

        for p in &self.outer_boundary {
            u_min = u_min.min(p.u);
            u_max = u_max.max(p.u);
            v_min = v_min.min(p.v);
            v_max = v_max.max(p.v);
        }
        for hole in &self.holes {
            for p in hole {
                u_min = u_min.min(p.u);
                u_max = u_max.max(p.u);
                v_min = v_min.min(p.v);
                v_max = v_max.max(p.v);
            }
        }

        (u_min, u_max, v_min, v_max)
    }

    /// Check if a UV point is inside the domain (inside outer boundary, outside all holes).
    pub fn contains(&self, point: &Point2d) -> bool {
        if !point_in_polygon(point, &self.outer_boundary) {
            return false;
        }
        for hole in &self.holes {
            if point_in_polygon(point, hole) {
                return false;
            }
        }
        true
    }
}

/// Test if a 2D point is inside a closed polygon using ray casting.
fn point_in_polygon(point: &Point2d, polygon: &[Point2d]) -> bool {
    let n = polygon.len();
    if n < 3 {
        return false;
    }
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

// ============================================================
// Ear-clipping triangulation (GUARANTEED to terminate)
// ============================================================

/// Triangulate a parametric domain using ear-clipping with bridge-edge
/// hole insertion.
///
/// This approach is GUARANTEED to terminate in O(n²) worst case,
/// unlike spade's Constrained Delaunay Triangulation which can hang
/// inside a single `add_constraint` call when constraint edges intersect
/// or when near-coincident points create degenerate configurations.
///
/// The algorithm:
/// 1. Collect all boundary and hole points
/// 2. Insert each hole into the outer polygon using bridge edges
/// 3. Ear-clip the merged polygon (always terminates)
/// 4. Add interior grid points by subdividing containing triangles
/// 5. Filter triangles by domain containment
/// 6. Map UV vertices to 3D
pub fn triangulate_cdt(
    domain: &ParametricDomain,
    surface: &Surface,
    forward: bool,
    interior_uv_points: &[Point2d],
) -> TriangleMesh {
    // If no holes and no interior points, use simple ear-clip of outer boundary
    if domain.holes.is_empty() && interior_uv_points.is_empty() {
        return triangulate_simple_domain(domain, surface, forward);
    }

    // If no holes but has interior points, still use ear-clip + point insertion
    if domain.holes.is_empty() {
        return triangulate_simple_domain_with_interior(domain, surface, forward, interior_uv_points);
    }

    // Has holes: use bridge-edge + ear-clipping
    triangulate_domain_with_holes(domain, surface, forward, interior_uv_points)
}

/// Triangulate a simple domain (no holes) using ear-clipping.
fn triangulate_simple_domain(
    domain: &ParametricDomain,
    surface: &Surface,
    forward: bool,
) -> TriangleMesh {
    let outer = &domain.outer_boundary;
    if outer.len() < 3 {
        return TriangleMesh::new();
    }

    // Ear-clip the outer boundary directly
    let triangles_2d = crate::ear_clip(outer);

    // Map UV to 3D
    uv_triangles_to_3d(&triangles_2d, outer, surface, forward)
}

/// Triangulate a simple domain with interior points using ear-clip + subdivision.
fn triangulate_simple_domain_with_interior(
    domain: &ParametricDomain,
    surface: &Surface,
    forward: bool,
    interior_uv_points: &[Point2d],
) -> TriangleMesh {
    let outer = &domain.outer_boundary;
    if outer.len() < 3 {
        return TriangleMesh::new();
    }

    let triangles_2d = crate::ear_clip(outer);
    let mut all_points: Vec<Point2d> = outer.clone();
    let mut result_triangles = triangles_2d;

    // Insert interior points by subdividing containing triangles
    for &pt in interior_uv_points {
        if !domain.contains(&pt) {
            continue;
        }
        let new_idx = all_points.len() as u32;
        all_points.push(pt);

        // Find a triangle that contains this point and subdivide it
        let mut found = false;
        for tri in &mut result_triangles {
            let a = all_points[tri[0] as usize];
            let b = all_points[tri[1] as usize];
            let c = all_points[tri[2] as usize];
            if point_in_triangle_2d(&pt, &a, &b, &c) {
                let old = *tri;
                *tri = [old[0], old[1], new_idx];
                result_triangles.push([old[1], old[2], new_idx]);
                result_triangles.push([old[2], old[0], new_idx]);
                found = true;
                break;
            }
        }

        if !found {
            // Point not in any triangle — skip it (shouldn't happen if domain is valid)
            all_points.pop();
        }
    }

    uv_triangles_to_3d(&result_triangles, &all_points, surface, forward)
}

/// Triangulate a domain with holes using bridge-edge + ear-clipping.
///
/// This is the key algorithm that replaces CDT. It works by:
/// 1. Finding a "bridge edge" from each hole to the outer polygon
/// 2. Merging the hole into the outer polygon via the bridge
/// 3. Ear-clipping the resulting single polygon (guaranteed O(n²))
/// 4. Adding interior points via triangle subdivision
/// 5. Filtering triangles by domain containment
fn triangulate_domain_with_holes(
    domain: &ParametricDomain,
    surface: &Surface,
    forward: bool,
    interior_uv_points: &[Point2d],
) -> TriangleMesh {
    let outer = &domain.outer_boundary;
    if outer.len() < 3 {
        return TriangleMesh::new();
    }

    // Downsample holes if too many points (prevents O(n²) blowup in ear-clip)
    // Use same limits on all platforms for consistent results
    let max_hole_points = 200;

    let downsampled_holes: Vec<Vec<Point2d>> = domain.holes.iter()
        .map(|hole| {
            if hole.len() > max_hole_points {
                let step = hole.len() as f64 / max_hole_points as f64;
                (0..max_hole_points)
                    .map(|i| hole[((i as f64 * step) as usize).min(hole.len() - 1)])
                    .collect()
            } else {
                hole.clone()
            }
        })
        .collect();

    // Also downsample outer boundary if too large
    // Use same limits on all platforms for consistent results
    let max_outer_points = 500;

    let outer_downsampled: Vec<Point2d> = if outer.len() > max_outer_points {
        let step = outer.len() as f64 / max_outer_points as f64;
        (0..max_outer_points)
            .map(|i| outer[((i as f64 * step) as usize).min(outer.len() - 1)])
            .collect()
    } else {
        outer.clone()
    };

    // Collect all UV points: outer boundary first, then holes, then interior
    let mut all_points: Vec<Point2d> = outer_downsampled.clone();
    let mut hole_start_indices: Vec<usize> = Vec::new();

    for hole in &downsampled_holes {
        if hole.len() < 3 {
            continue;
        }
        hole_start_indices.push(all_points.len());
        all_points.extend_from_slice(hole);
    }

    // Add interior grid points
    let mut interior_point_indices: Vec<u32> = Vec::new();
    for &pt in interior_uv_points {
        if !domain.contains(&pt) {
            continue;
        }
        let idx = all_points.len() as u32;
        all_points.push(pt);
        interior_point_indices.push(idx);
    }

    // Use earcutr for triangulation — it natively handles holes without bridge-edge tricks
    let mut coords: Vec<f64> = Vec::with_capacity(all_points.len() * 2);
    for p in &all_points {
        coords.push(p.u);
        coords.push(p.v);
    }

    let earcutr_hole_indices: Vec<usize> = hole_start_indices.iter()
        .map(|&idx| idx) // hole indices already point into all_points
        .collect();

    let triangle_indices = earcutr::earcut(&coords, &earcutr_hole_indices, 2);

    let mut result_triangles: Vec<[u32; 3]> = Vec::new();
    for chunk in triangle_indices.chunks(3) {
        if chunk.len() < 3 { break; }
        let a = chunk[0] as u32;
        let b = chunk[1] as u32;
        let c = chunk[2] as u32;
        if a == b || b == c || a == c { continue; }
        result_triangles.push([a, b, c]);
    }

    // Insert interior points by subdividing containing triangles
    for &uv_idx in &interior_point_indices {
        let pt = all_points[uv_idx as usize];

        let mut found = false;
        for tri in &mut result_triangles {
            let a = all_points[tri[0] as usize];
            let b = all_points[tri[1] as usize];
            let c = all_points[tri[2] as usize];
            if point_in_triangle_2d(&pt, &a, &b, &c) {
                let old = *tri;
                *tri = [old[0], old[1], uv_idx];
                result_triangles.push([old[1], old[2], uv_idx]);
                result_triangles.push([old[2], old[0], uv_idx]);
                found = true;
                break;
            }
        }
        // If not found in any triangle, skip (harmless)
    }

    // Filter triangles by domain containment and map to 3D
    let mut mesh = TriangleMesh::new();
    let mut vertex_map: std::collections::HashMap<u32, u32> = std::collections::HashMap::new();

    for tri in &result_triangles {
        // Get UV positions for the three vertices
        let a_uv = all_points[tri[0] as usize];
        let b_uv = all_points[tri[1] as usize];
        let c_uv = all_points[tri[2] as usize];

        // Check if triangle centroid is inside the domain
        let centroid = Point2d::new(
            (a_uv.u + b_uv.u + c_uv.u) / 3.0,
            (a_uv.v + b_uv.v + c_uv.v) / 3.0,
        );
        if !domain.contains(&centroid) {
            continue;
        }

        // Check for degenerate triangle (near-zero area)
        let area = triangle_area_2d(a_uv.u, a_uv.v, b_uv.u, b_uv.v, c_uv.u, c_uv.v);
        if area < 1e-20 {
            continue;
        }

        // Add vertices and triangle
        let mut tri_indices = [0u32; 3];
        for (k, &idx) in tri.iter().enumerate() {
            let entry = vertex_map.entry(idx).or_insert_with(|| {
                let uv = all_points[idx as usize];
                let p3d = surface.point_at(uv.u, uv.v);
                let n = surface.normal_at(uv.u, uv.v);
                let vi = mesh.add_vertex(p3d);
                mesh.add_vertex_normal(vi, [n.x, n.y, n.z]);
                vi
            });
            tri_indices[k] = *entry;
        }

        if forward {
            mesh.add_triangle(tri_indices[0], tri_indices[1], tri_indices[2]);
        } else {
            mesh.add_triangle(tri_indices[0], tri_indices[2], tri_indices[1]);
        }
    }

    mesh
}

/// Find bridge edge between outer polygon and a hole for ear-clipping.
///
/// The bridge connects the rightmost point of the hole to the closest
/// point on the outer polygon. This is a standard technique for
/// converting a polygon-with-holes into a single simple polygon.
struct BridgeResult {
    outer_idx: usize,
    hole_idx: usize,
}

fn find_bridge_edge(
    all_points: &[Point2d],
    polygon_indices: &[u32],
    hole: &[Point2d],
) -> BridgeResult {
    // Find rightmost point of the hole (most positive u)
    let mut hole_idx = 0;
    let mut max_u = hole[0].u;
    for (i, p) in hole.iter().enumerate() {
        if p.u > max_u {
            max_u = p.u;
            hole_idx = i;
        }
    }

    // Find closest point on outer polygon to the rightmost hole point
    let hole_pt = &hole[hole_idx];
    let mut outer_idx = 0;
    let mut min_dist = f64::MAX;
    for (i, &idx) in polygon_indices.iter().enumerate() {
        let p = &all_points[idx as usize];
        let du = p.u - hole_pt.u;
        let dv = p.v - hole_pt.v;
        let dist = du * du + dv * dv;
        if dist < min_dist {
            min_dist = dist;
            outer_idx = i;
        }
    }

    BridgeResult { outer_idx, hole_idx }
}

/// Check if a 2D point is inside a triangle (barycentric coordinates).
fn point_in_triangle_2d(p: &Point2d, a: &Point2d, b: &Point2d, c: &Point2d) -> bool {
    let d1 = sign_2d(p, a, b);
    let d2 = sign_2d(p, b, c);
    let d3 = sign_2d(p, c, a);
    let has_neg = (d1 < 0.0) || (d2 < 0.0) || (d3 < 0.0);
    let has_pos = (d1 > 0.0) || (d2 > 0.0) || (d3 > 0.0);
    !(has_neg && has_pos)
}

/// Compute signed area for point-in-triangle test.
fn sign_2d(p1: &Point2d, p2: &Point2d, p3: &Point2d) -> f64 {
    (p1.u - p3.u) * (p2.v - p3.v) - (p2.u - p3.u) * (p1.v - p3.v)
}

/// Map 2D UV triangles to 3D using the surface evaluation.
fn uv_triangles_to_3d(
    triangles: &[[u32; 3]],
    points: &[Point2d],
    surface: &Surface,
    forward: bool,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let mut vertex_map: std::collections::HashMap<u32, u32> = std::collections::HashMap::new();

    for tri in triangles {
        let mut tri_indices = [0u32; 3];
        for (k, &idx) in tri.iter().enumerate() {
            let entry = vertex_map.entry(idx).or_insert_with(|| {
                let uv = points[idx as usize];
                let p3d = surface.point_at(uv.u, uv.v);
                let n = surface.normal_at(uv.u, uv.v);
                let vi = mesh.add_vertex(p3d);
                mesh.add_vertex_normal(vi, [n.x, n.y, n.z]);
                vi
            });
            tri_indices[k] = *entry;
        }

        if forward {
            mesh.add_triangle(tri_indices[0], tri_indices[1], tri_indices[2]);
        } else {
            mesh.add_triangle(tri_indices[0], tri_indices[2], tri_indices[1]);
        }
    }

    mesh
}

/// Compute the area of a 2D triangle.
fn triangle_area_2d(x0: f64, y0: f64, x1: f64, y1: f64, x2: f64, y2: f64) -> f64 {
    ((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)).abs() * 0.5
}

/// Generate interior UV grid points for a parametric domain.
///
/// Creates a regular grid of points within the domain's bounding box,
/// excluding points that are outside the domain or too close to boundaries.
pub fn generate_interior_points(
    domain: &ParametricDomain,
    n_u: usize,
    n_v: usize,
    boundary_margin: f64,
) -> Vec<Point2d> {
    let (u_min, u_max, v_min, v_max) = domain.bounding_box();
    let mut points = Vec::new();

    for j in 1..n_v {
        for i in 1..n_u {
            let u = u_min + (u_max - u_min) * i as f64 / n_u as f64;
            let v = v_min + (v_max - v_min) * j as f64 / n_v as f64;
            let pt = Point2d::new(u, v);

            // Check that the point is inside the domain
            if domain.contains(&pt) {
                // Check distance to boundary (skip points too close)
                let mut min_dist_sq = f64::MAX;
                for p in &domain.outer_boundary {
                    let du = u - p.u;
                    let dv = v - p.v;
                    min_dist_sq = min_dist_sq.min(du * du + dv * dv);
                }
                if min_dist_sq > boundary_margin * boundary_margin {
                    points.push(pt);
                }
            }
        }
    }

    points
}

/// Generate interior UV points for NURBS surfaces, respecting knot ranges.
///
/// Places additional sample points at knot boundaries to capture
/// surface features that occur at knot spans.
pub fn generate_nurbs_interior_points(
    domain: &ParametricDomain,
    u_knots: &[f64],
    v_knots: &[f64],
    n_sub: usize,
) -> Vec<Point2d> {
    let (u_min, u_max, v_min, v_max) = domain.bounding_box();
    let mut points = Vec::new();

    // Filter knots within domain
    let u_knots_in_range: Vec<f64> = u_knots
        .iter()
        .filter(|&&k| k > u_min && k < u_max)
        .cloned()
        .collect();
    let v_knots_in_range: Vec<f64> = v_knots
        .iter()
        .filter(|&&k| k > v_min && k < v_max)
        .cloned()
        .collect();

    // Generate grid points: knot boundaries + interior subdivisions
    let mut u_values: Vec<f64> = vec![u_min];
    for k in &u_knots_in_range {
        u_values.push(*k);
    }
    u_values.push(u_max);

    let mut v_values: Vec<f64> = vec![v_min];
    for k in &v_knots_in_range {
        v_values.push(*k);
    }
    v_values.push(v_max);

    // Subdivide each knot span
    let mut u_grid: Vec<f64> = Vec::new();
    for i in 0..u_values.len() - 1 {
        for j in 0..n_sub {
            let t = j as f64 / n_sub as f64;
            u_grid.push(u_values[i] + t * (u_values[i + 1] - u_values[i]));
        }
    }
    u_grid.push(u_max);

    let mut v_grid: Vec<f64> = Vec::new();
    for i in 0..v_values.len() - 1 {
        for j in 0..n_sub {
            let t = j as f64 / n_sub as f64;
            v_grid.push(v_values[i] + t * (v_values[i + 1] - v_values[i]));
        }
    }
    v_grid.push(v_max);

    // Filter points inside domain
    for &u in &u_grid {
        for &v in &v_grid {
            let pt = Point2d::new(u, v);
            if domain.contains(&pt) {
                points.push(pt);
            }
        }
    }

    points
}

// ============================================================
// Integration: ear-clipping based surface triangulation
// ============================================================

/// Triangulate a curved surface using UV-space ear-clipping with bridge edges.
///
/// This replaces the previous CDT-based approach (spade) which could hang
/// indefinitely inside `add_constraint` on degenerate inputs. The ear-clipping
/// approach is GUARANTEED to terminate in O(n²) worst case.
///
/// For surfaces without holes, this uses simple ear-clipping of the outer boundary.
/// For surfaces with holes, each hole is connected to the outer boundary via a
/// bridge edge, forming a single polygon that is then ear-clipped.
///
/// Interior grid points are inserted by subdividing containing triangles,
/// which is O(interior_points × triangles) but always terminates.
pub fn triangulate_surface_uv_cdt(
    surface: &Surface,
    boundary_points: &[Point3d],
    hole_polylines: &[Vec<Point3d>],
    forward: bool,
    params: &crate::triangulate::TriangulationParams,
) -> TriangleMesh {
    if boundary_points.is_empty() {
        return TriangleMesh::new();
    }

    // Downsample boundary points to prevent O(n²) blowup in ear-clipping.
    // Use same limits on all platforms for consistent results
    let max_boundary_points = 500;

    let boundary_points = if boundary_points.len() > max_boundary_points {
        let step = boundary_points.len() as f64 / max_boundary_points as f64;
        let sampled: Vec<Point3d> = (0..max_boundary_points)
            .map(|i| boundary_points[((i as f64 * step) as usize).min(boundary_points.len() - 1)])
            .collect();
        log::info!(
            "Ear-clip: downsampled boundary from {} to {} points",
            boundary_points.len(), sampled.len()
        );
        sampled
    } else {
        boundary_points.to_vec()
    };

    // Also downsample hole polylines
    // Use same limits on all platforms for consistent results
    let max_hole_points = 200;

    let hole_polylines_downsampled: Vec<Vec<Point3d>> = hole_polylines.iter()
        .map(|hole| {
            if hole.len() > max_hole_points {
                let step = hole.len() as f64 / max_hole_points as f64;
                let sampled: Vec<Point3d> = (0..max_hole_points)
                    .map(|i| hole[((i as f64 * step) as usize).min(hole.len() - 1)])
                    .collect();
                sampled
            } else {
                hole.clone()
            }
        })
        .collect();

    // Project 3D boundary to UV
    let mut outer_uv: Vec<Point2d> = boundary_points
        .iter()
        .map(|p| {
            let (u, v) = surface.project_point(p);
            Point2d::new(u, v)
        })
        .collect();

    // Normalize UV for periodic surfaces
    let u_period = if surface.is_u_periodic() {
        Some(2.0 * PI)
    } else {
        None
    };
    let v_period = if surface.is_v_periodic() {
        Some(2.0 * PI)
    } else {
        None
    };
    crate::triangulate::normalize_uv_polygon(&mut outer_uv, u_period, v_period);

    // Compute UV range
    let mut u_min = f64::MAX;
    let mut u_max = f64::MIN;
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;
    for p in &outer_uv {
        u_min = u_min.min(p.u);
        u_max = u_max.max(p.u);
        v_min = v_min.min(p.v);
        v_max = v_max.max(p.v);
    }
    let margin_u = (u_max - u_min) * 0.01;
    let margin_v = (v_max - v_min) * 0.01;

    // Project holes to UV (use downsampled holes)
    let holes_uv: Vec<Vec<Point2d>> = hole_polylines_downsampled
        .iter()
        .map(|hole| {
            let mut huv: Vec<Point2d> = hole
                .iter()
                .map(|p| {
                    let (u, v) = surface.project_point(p);
                    Point2d::new(u, v)
                })
                .collect();
            crate::triangulate::normalize_uv_polygon(&mut huv, u_period, v_period);
            huv
        })
        .collect();

    // Create parametric domain
    let mut domain = ParametricDomain::new(
        outer_uv,
        (u_min - margin_u, u_max + margin_u),
        (v_min - margin_v, v_max + margin_v),
    );
    for hole in &holes_uv {
        domain = domain.with_hole(hole.clone());
    }

    // Generate interior points using adaptive sampling (capped by max_face_triangles)
    let (n_u, n_v) = if params.adaptive {
        crate::adaptive::required_samples_capped(
            surface,
            u_min, u_max, v_min, v_max,
            params.max_deviation, params.detail_level,
            params.max_face_triangles,
        )
    } else {
        let mut n_u = params.angular_samples;
        let mut n_v = params.height_samples;
        let approx_tris = 2 * n_u * n_v;
        if approx_tris > params.max_face_triangles {
            let scale = (params.max_face_triangles as f64 / approx_tris as f64).sqrt();
            n_u = ((n_u as f64 * scale).ceil() as usize).max(4);
            n_v = ((n_v as f64 * scale).ceil() as usize).max(2);
        }
        (n_u, n_v)
    };

    // Compute margin for interior points
    let u_step = (u_max - u_min) / n_u.max(1) as f64;
    let v_step = (v_max - v_min) / n_v.max(1) as f64;
    let boundary_margin = u_step.min(v_step) * 0.3;

    let interior_points = generate_interior_points(&domain, n_u, n_v, boundary_margin);

    // Triangulate using ear-clipping (replaces CDT — guaranteed to terminate)
    triangulate_cdt(&domain, surface, forward, &interior_points)
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_domain_contains_square() {
        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(1.0, 0.0),
            Point2d::new(1.0, 1.0),
            Point2d::new(0.0, 1.0),
        ];
        let domain = ParametricDomain::new(outer, (0.0, 1.0), (0.0, 1.0));
        assert!(domain.contains(&Point2d::new(0.5, 0.5)));
        assert!(!domain.contains(&Point2d::new(1.5, 0.5)));
    }

    #[test]
    fn test_domain_with_hole() {
        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(2.0, 0.0),
            Point2d::new(2.0, 2.0),
            Point2d::new(0.0, 2.0),
        ];
        let hole = vec![
            Point2d::new(0.5, 0.5),
            Point2d::new(1.5, 0.5),
            Point2d::new(1.5, 1.5),
            Point2d::new(0.5, 1.5),
        ];
        let domain = ParametricDomain::new(outer, (0.0, 2.0), (0.0, 2.0)).with_hole(hole);
        assert!(domain.contains(&Point2d::new(0.25, 0.25))); // Outside hole
        assert!(!domain.contains(&Point2d::new(1.0, 1.0))); // Inside hole
    }

    #[test]
    fn test_cylinder_with_hole() {
        use draper_geometry::{CylinderSurface, Point3d, Surface};

        let cyl = CylinderSurface::new_z(5.0);
        let surface = Surface::Cylinder(cyl);

        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(PI, 0.0),
            Point2d::new(PI, 10.0),
            Point2d::new(0.0, 10.0),
        ];
        let hole = vec![
            Point2d::new(1.0, 3.0),
            Point2d::new(2.0, 3.0),
            Point2d::new(2.0, 7.0),
            Point2d::new(1.0, 7.0),
        ];
        let domain = ParametricDomain::new(outer, (0.0, PI), (0.0, 10.0)).with_hole(hole);

        let interior = generate_interior_points(&domain, 10, 10, 0.1);
        for p in &interior {
            assert!(domain.contains(p), "Interior point {:?} should be inside domain", p);
        }

        let mesh = triangulate_cdt(&domain, &surface, true, &interior);
        assert!(!mesh.triangles.is_empty(), "Should generate triangles with holes");
    }

    #[test]
    fn test_sphere_band() {
        use draper_geometry::{SphereSurface, Point3d, Surface};

        let sphere = SphereSurface::new(Point3d::ORIGIN, 10.0);
        let surface = Surface::Sphere(sphere);

        let n_pts = 20;
        let mut outer: Vec<Point2d> = Vec::new();
        for i in 0..n_pts {
            let u = 2.0 * PI * i as f64 / n_pts as f64;
            outer.push(Point2d::new(u, PI / 4.0));
        }
        outer.push(Point2d::new(2.0 * PI, PI / 2.0));
        for i in (0..n_pts).rev() {
            let u = 2.0 * PI * i as f64 / n_pts as f64;
            outer.push(Point2d::new(u, PI / 2.0));
        }
        outer.push(Point2d::new(0.0, PI / 4.0));

        let domain = ParametricDomain::new(outer, (0.0, 2.0 * PI), (PI / 4.0, PI / 2.0));
        let interior = generate_interior_points(&domain, 10, 5, 0.01);
        let mesh = triangulate_cdt(&domain, &surface, true, &interior);
        assert!(!mesh.triangles.is_empty(), "Sphere band should generate triangles");
    }

    #[test]
    fn test_nurbs_interior_points() {
        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(4.0, 0.0),
            Point2d::new(4.0, 4.0),
            Point2d::new(0.0, 4.0),
        ];
        let domain = ParametricDomain::new(outer, (0.0, 4.0), (0.0, 4.0));

        let u_knots = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let v_knots = vec![0.0, 2.0, 4.0];

        let points = generate_nurbs_interior_points(&domain, &u_knots, &v_knots, 2);
        for p in &points {
            assert!(domain.contains(p), "NURBS interior point {:?} should be inside domain", p);
        }
        assert!(!points.is_empty(), "Should generate NURBS interior points");
    }

    #[test]
    fn test_earclip_with_holes_no_hang() {
        // Test that ear-clipping with multiple holes completes quickly
        // (previously CDT could hang on this)
        use draper_geometry::{SphereSurface, Point3d, Surface};

        let sphere = SphereSurface::new(Point3d::ORIGIN, 10.0);
        let surface = Surface::Sphere(sphere);

        // Outer boundary: a rectangle in UV space
        let outer = vec![
            Point2d::new(0.0, 0.5),
            Point2d::new(3.0, 0.5),
            Point2d::new(3.0, 2.5),
            Point2d::new(0.0, 2.5),
        ];

        // Multiple holes (simulating text cutouts)
        let hole1 = vec![
            Point2d::new(0.5, 1.0),
            Point2d::new(1.0, 1.0),
            Point2d::new(1.0, 2.0),
            Point2d::new(0.5, 2.0),
        ];
        let hole2 = vec![
            Point2d::new(1.5, 1.0),
            Point2d::new(2.0, 1.0),
            Point2d::new(2.0, 2.0),
            Point2d::new(1.5, 2.0),
        ];
        let hole3 = vec![
            Point2d::new(2.2, 0.8),
            Point2d::new(2.8, 0.8),
            Point2d::new(2.8, 1.5),
            Point2d::new(2.2, 1.5),
        ];

        let domain = ParametricDomain::new(outer, (0.0, 3.0), (0.5, 2.5))
            .with_hole(hole1)
            .with_hole(hole2)
            .with_hole(hole3);

        let start = std::time::Instant::now();
        let mesh = triangulate_cdt(&domain, &surface, true, &[]);
        let elapsed = start.elapsed();

        assert!(!mesh.triangles.is_empty(), "Should generate triangles with 3 holes");
        assert!(elapsed.as_millis() < 100, "Ear-clip should be fast, took {}ms", elapsed.as_millis());
    }
}
