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

use draper_geometry::{Point2d, Point3d, Surface, DegeneracyFlags};
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

    /// Add multiple holes (inner boundaries) to the domain.
    pub fn with_holes_from<I: IntoIterator<Item = UVPolygon>>(mut self, holes: I) -> Self {
        self.holes.extend(holes);
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
// Consistent (watertight) triangulation — Phase 2.2
// ============================================================

/// Triangulate a curved surface with **consistent** boundary vertices.
///
/// This function is the key to producing watertight meshes where shared edges
/// between adjacent faces have **bit-identical** 3D vertex positions. It works by:
///
/// 1. Accepting pre-computed UV coordinates for boundary and hole points
///    (from PCURVE or EdgeDiscretizationCache), avoiding the inaccurate
///    and slow `surface.project_point()` re-projection.
/// 2. Using boundary 3D points **directly** (not re-projected from UV) as
///    the 3D positions for boundary vertices. This ensures that two faces
///    sharing an edge receive the exact same 3D points from StepEdgeCache.
/// 3. Including boundary UV points as vertices in the earcutr triangulation,
///    so boundary edges become triangle edges (constraints in the mesh).
/// 4. Adding interior grid points to earcutr's input as additional vertices,
///    then mapping interior vertices to 3D via `surface.point_at(u,v)`.
///
/// # Arguments
/// * `surface` — The parametric surface to triangulate.
/// * `boundary_points_3d` — 3D points along the outer boundary (from cache).
/// * `boundary_uvs` — Pre-computed UV coordinates for boundary points.
/// * `hole_polylines_3d` — 3D points along each hole boundary.
/// * `hole_uvs` — Pre-computed UV coordinates for hole points (same structure as hole_polylines_3d).
/// * `forward` — Whether face normal matches surface normal.
/// * `params` — Triangulation parameters (for grid resolution).
///
/// # Returns
/// A `TriangleMesh` where boundary vertices use the cached 3D positions directly.
pub fn triangulate_surface_consistent(
    surface: &Surface,
    boundary_points_3d: &[Point3d],
    boundary_uvs: &[Point2d],
    hole_polylines_3d: &[Vec<Point3d>],
    hole_uvs: &[Vec<Point2d>],
    forward: bool,
    params: &crate::triangulate::TriangulationParams,
) -> TriangleMesh {
    if boundary_points_3d.is_empty() || boundary_uvs.len() < 3 {
        return TriangleMesh::new();
    }

    // Downsample boundary if too many points (prevents O(n²) blowup in earcutr)
    let max_boundary_points = 500;
    let max_hole_points = 200;

    let (boundary_3d, boundary_uv) = if boundary_points_3d.len() > max_boundary_points {
        let step = boundary_points_3d.len() as f64 / max_boundary_points as f64;
        let sampled_3d: Vec<Point3d> = (0..max_boundary_points)
            .map(|i| boundary_points_3d[((i as f64 * step) as usize).min(boundary_points_3d.len() - 1)])
            .collect();
        let sampled_uv: Vec<Point2d> = (0..max_boundary_points)
            .map(|i| boundary_uvs[((i as f64 * step) as usize).min(boundary_uvs.len() - 1)])
            .collect();
        (sampled_3d, sampled_uv)
    } else {
        (boundary_points_3d.to_vec(), boundary_uvs.to_vec())
    };

    // Downsample holes
    let (holes_3d, holes_uv): (Vec<Vec<Point3d>>, Vec<Vec<Point2d>>) = hole_polylines_3d.iter()
        .zip(hole_uvs.iter())
        .map(|(h3d, huv)| {
            if h3d.len() > max_hole_points {
                let step = h3d.len() as f64 / max_hole_points as f64;
                let s3d: Vec<Point3d> = (0..max_hole_points)
                    .map(|i| h3d[((i as f64 * step) as usize).min(h3d.len() - 1)])
                    .collect();
                let suv: Vec<Point2d> = (0..max_hole_points)
                    .map(|i| huv[((i as f64 * step) as usize).min(huv.len() - 1)])
                    .collect();
                (s3d, suv)
            } else {
                (h3d.clone(), huv.clone())
            }
        })
        .unzip();

    // Normalize UV for periodic surfaces
    let u_period = if surface.is_u_periodic() { Some(2.0 * PI) } else { None };
    let v_period = if surface.is_v_periodic() { Some(2.0 * PI) } else { None };

    let mut outer_uv = boundary_uv.clone();
    crate::triangulate::normalize_uv_polygon(&mut outer_uv, u_period, v_period);

    let mut normalized_holes_uv: Vec<Vec<Point2d>> = Vec::new();
    for huv in &holes_uv {
        let mut nhuv = huv.clone();
        crate::triangulate::normalize_uv_polygon(&mut nhuv, u_period, v_period);
        normalized_holes_uv.push(nhuv);
    }

    // ============================================================
    // Phase 2: Pole/apex degeneracy handling
    //
    // Detect degenerate boundary vertices where multiple UV points
    // map to the same 3D point (sphere poles, cone apex). These
    // must be collapsed to a single vertex BEFORE earcutr runs,
    // otherwise earcutr creates degenerate triangles with zero-area.
    // ============================================================
    let (boundary_3d, outer_uv, holes_3d, normalized_holes_uv) =
        collapse_degenerate_boundary_vertices(
            boundary_3d, outer_uv, holes_3d, normalized_holes_uv, surface,
        );

    if outer_uv.len() < 3 {
        return TriangleMesh::new();
    }

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
    for huv in &normalized_holes_uv {
        for p in huv {
            u_min = u_min.min(p.u);
            u_max = u_max.max(p.u);
            v_min = v_min.min(p.v);
            v_max = v_max.max(p.v);
        }
    }

    let margin_u = (u_max - u_min) * 0.01;
    let margin_v = (v_max - v_min) * 0.01;

    // Build parametric domain for containment checks
    let domain = ParametricDomain::new(
        outer_uv.clone(),
        (u_min - margin_u, u_max + margin_u),
        (v_min - margin_v, v_max + margin_v),
    ).with_holes_from(normalized_holes_uv.iter().cloned());

    // Generate interior grid points
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

    let boundary_margin_u = (u_max - u_min) / n_u.max(1) as f64 * 0.3;
    let boundary_margin_v = (v_max - v_min) / n_v.max(1) as f64 * 0.3;
    let boundary_margin = boundary_margin_u.min(boundary_margin_v);

    // ============================================================
    // Phase 2: Adaptive interior point generation
    //
    // Generate interior points, adding extra samples near degenerate
    // regions (poles, apex) to ensure good triangulation quality.
    // ============================================================
    let mut interior_uv_points = generate_interior_points(&domain, n_u, n_v, boundary_margin);

    // Add extra interior points near degenerate boundary vertices
    // (poles/apexes) for better fan triangulation quality
    let degenerate_tol = 1e-3;
    for (i, uv) in outer_uv.iter().enumerate() {
        let flags = surface.is_degenerate_at(uv.u, uv.v, degenerate_tol);
        if flags.contains(DegeneracyFlags::DU_ZERO) {
            // This is a pole/apex — add nearby interior points to help
            // earcutr create good fan triangulation
            let step_u = (u_max - u_min) / n_u.max(1) as f64;
            let step_v = (v_max - v_min) / n_v.max(1) as f64;
            let small = step_u.min(step_v) * 0.5;

            // Add points slightly away from the pole in the non-degenerate direction
            if flags.contains(DegeneracyFlags::DV_ZERO) {
                // Fully degenerate (apex) — add ring of points around it
                for k in 0..6 {
                    let angle = 2.0 * PI * k as f64 / 6.0;
                    let du = small * angle.cos();
                    let dv = small * angle.sin();
                    let pt = Point2d::new(uv.u + du, uv.v + dv);
                    if domain.contains(&pt) {
                        interior_uv_points.push(pt);
                    }
                }
            } else {
                // DU_ZERO only (sphere pole) — add ring of points below/above
                let v_dir = if uv.v < (v_min + v_max) * 0.5 { 1.0 } else { -1.0 };
                let n_ring = n_u.max(6);
                let ring_v = uv.v + v_dir * small;
                for k in 0..n_ring {
                    let u_val = u_min + (u_max - u_min) * k as f64 / n_ring as f64;
                    let pt = Point2d::new(u_val, ring_v);
                    if domain.contains(&pt) {
                        interior_uv_points.push(pt);
                    }
                }
            }
        }
    }

    // ============================================================
    // Build earcutr input: [boundary_uv...][hole_uv...]
    // Interior points are added later via subdivision for better mesh quality.
    // ============================================================

    let n_boundary = outer_uv.len();

    let mut coords: Vec<f64> = Vec::with_capacity((n_boundary + normalized_holes_uv.iter().map(|h| h.len()).sum::<usize>()) * 2);

    // Outer boundary UV
    for p in &outer_uv {
        coords.push(p.u);
        coords.push(p.v);
    }

    // Hole UVs
    let mut hole_start_indices: Vec<usize> = Vec::new();
    for huv in &normalized_holes_uv {
        if huv.len() < 3 {
            continue;
        }
        hole_start_indices.push(coords.len() / 2);
        for p in huv {
            coords.push(p.u);
            coords.push(p.v);
        }
    }

    // Run earcutr triangulation (boundary + holes only, no interior yet)
    let triangle_indices = earcutr::earcut(&coords, &hole_start_indices, 2);

    // Collect raw triangles
    let mut result_triangles: Vec<[u32; 3]> = Vec::new();
    for chunk in triangle_indices.chunks(3) {
        if chunk.len() < 3 { break; }
        let a = chunk[0] as u32;
        let b = chunk[1] as u32;
        let c = chunk[2] as u32;
        if a == b || b == c || a == c { continue; }
        result_triangles.push([a, b, c]);
    }

    // Build all_uv: [boundary_uv...][hole_uv...]
    let mut all_uv: Vec<Point2d> = outer_uv.clone();
    for huv in &normalized_holes_uv {
        all_uv.extend_from_slice(huv);
    }

    // Insert interior points by subdividing containing triangles.
    // This ensures interior points are connected to the mesh properly.
    for &pt in interior_uv_points.iter() {
        let uv_idx = all_uv.len() as u32;
        all_uv.push(pt);

        // Find a triangle that contains this point and subdivide it
        let mut found = false;
        for tri in &mut result_triangles {
            let a = all_uv[tri[0] as usize];
            let b = all_uv[tri[1] as usize];
            let c = all_uv[tri[2] as usize];
            if point_in_triangle_2d(&pt, &a, &b, &c) {
                let old = *tri;
                *tri = [old[0], old[1], uv_idx];
                result_triangles.push([old[1], old[2], uv_idx]);
                result_triangles.push([old[2], old[0], uv_idx]);
                found = true;
                break;
            }
        }
        if !found {
            // Point not in any triangle — skip it
            all_uv.pop();
        }
    }

    // ============================================================
    // Phase 2: Adaptive refinement
    //
    // After initial triangulation, check surface deviation of each
    // triangle. If the midpoint of any edge deviates from the surface
    // by more than max_deviation, subdivide that triangle by inserting
    // the midpoint and splitting into sub-triangles.
    // ============================================================
    if params.adaptive && params.max_deviation > 0.0 {
        adaptive_refine_triangles(
            &mut result_triangles, &mut all_uv, surface,
            params.max_deviation, params.max_face_triangles,
            &domain,
        );
    }

    // ============================================================
    // Build 3D mesh: boundary vertices use cached 3D, interior use surface.point_at
    // ============================================================

    // Build combined 3D point array for boundary + hole vertices
    let mut all_boundary_3d: Vec<Point3d> = boundary_3d.clone();
    for h3d in &holes_3d {
        all_boundary_3d.extend_from_slice(h3d);
    }
    let n_boundary_and_holes = all_boundary_3d.len();

    // Filter triangles by domain containment and build mesh
    let mut mesh = TriangleMesh::new();
    let mut vertex_map: std::collections::HashMap<u32, u32> = std::collections::HashMap::new();

    for tri in &result_triangles {
        // Bounds check — skip triangles with invalid indices
        if tri[0] as usize >= all_uv.len() || tri[1] as usize >= all_uv.len() || tri[2] as usize >= all_uv.len() {
            continue;
        }

        // Get UV positions
        let a_uv = all_uv[tri[0] as usize];
        let b_uv = all_uv[tri[1] as usize];
        let c_uv = all_uv[tri[2] as usize];

        // Check if triangle centroid is inside the domain
        let centroid = Point2d::new(
            (a_uv.u + b_uv.u + c_uv.u) / 3.0,
            (a_uv.v + b_uv.v + c_uv.v) / 3.0,
        );
        if !domain.contains(&centroid) {
            continue;
        }

        // Check for degenerate triangle
        let area = triangle_area_2d(a_uv.u, a_uv.v, b_uv.u, b_uv.v, c_uv.u, c_uv.v);
        if area < 1e-20 {
            continue;
        }

        // Add vertices and triangle
        let mut tri_indices = [0u32; 3];
        for (k, &idx) in tri.iter().enumerate() {
            let idx_usize = idx as usize;
            let entry = vertex_map.entry(idx).or_insert_with(|| {
                if idx_usize < n_boundary_and_holes {
                    // Boundary/hole vertex: use cached 3D point directly
                    let p3d = all_boundary_3d[idx_usize];
                    let uv = all_uv[idx_usize];
                    let n = surface.normal_at(uv.u, uv.v);
                    let vi = mesh.add_vertex(p3d);
                    mesh.add_vertex_normal(vi, [n.x, n.y, n.z]);
                    vi
                } else {
                    // Interior vertex: compute 3D from UV via surface.point_at
                    let uv = all_uv[idx_usize];
                    let p3d = surface.point_at(uv.u, uv.v);
                    let n = surface.normal_at(uv.u, uv.v);
                    let vi = mesh.add_vertex(p3d);
                    mesh.add_vertex_normal(vi, [n.x, n.y, n.z]);
                    vi
                }
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

// ============================================================
// Phase 2: Pole/apex degeneracy handling
// ============================================================

/// Collapse degenerate boundary vertices where multiple UV points
/// map to the same 3D point (sphere poles, cone apex).
///
/// When a boundary passes through a degenerate parametric point (where the
/// surface parameterization collapses, e.g. a sphere pole where all u-values
/// map to the same 3D point), the edge cache produces many boundary points
/// with different UV coordinates but identical 3D positions. If we feed all
/// of these to earcutr, it creates degenerate (zero-area) triangles.
///
/// This function detects such clusters and collapses them to a single vertex.
/// It uses `Surface::is_degenerate_at()` to identify degenerate UV regions
/// and groups nearby degenerate boundary points into a single representative.
///
/// # Algorithm
/// 1. For each boundary point, check if it lies in a degenerate UV region.
/// 2. Group consecutive degenerate points into clusters.
/// 3. Replace each cluster with a single representative point (the first in the group).
/// 4. Return the collapsed boundary arrays.
fn collapse_degenerate_boundary_vertices(
    boundary_3d: Vec<Point3d>,
    boundary_uv: Vec<Point2d>,
    holes_3d: Vec<Vec<Point3d>>,
    holes_uv: Vec<Vec<Point2d>>,
    surface: &Surface,
) -> (Vec<Point3d>, Vec<Point2d>, Vec<Vec<Point3d>>, Vec<Vec<Point2d>>) {
    let degenerate_tol = 1e-3; // Tolerance for degeneracy detection

    // Process outer boundary
    let (b3d, buv) = collapse_degenerate_ring(&boundary_3d, &boundary_uv, surface, degenerate_tol);

    // Process holes
    let mut h3d_result = Vec::new();
    let mut huv_result = Vec::new();
    for (h3d, huv) in holes_3d.into_iter().zip(holes_uv.into_iter()) {
        let (collapsed_3d, collapsed_uv) = collapse_degenerate_ring(&h3d, &huv, surface, degenerate_tol);
        h3d_result.push(collapsed_3d);
        huv_result.push(collapsed_uv);
    }

    (b3d, buv, h3d_result, huv_result)
}

/// Collapse degenerate vertices in a single ring (outer boundary or hole).
///
/// Detects runs of consecutive degenerate boundary vertices and replaces
/// each run with a single representative. The representative is the vertex
/// closest to the midpoint of the run's 3D positions.
fn collapse_degenerate_ring(
    ring_3d: &[Point3d],
    ring_uv: &[Point2d],
    surface: &Surface,
    degenerate_tol: f64,
) -> (Vec<Point3d>, Vec<Point2d>) {
    if ring_3d.len() < 3 {
        return (ring_3d.to_vec(), ring_uv.to_vec());
    }

    // Mark which vertices are degenerate
    let is_degenerate: Vec<bool> = ring_uv.iter()
        .map(|uv| surface.is_degenerate_at(uv.u, uv.v, degenerate_tol).contains(DegeneracyFlags::DU_ZERO))
        .collect();

    // Count degenerate vertices — if none, return as-is
    let n_degenerate = is_degenerate.iter().filter(|&&d| d).count();
    if n_degenerate == 0 {
        return (ring_3d.to_vec(), ring_uv.to_vec());
    }

    // Group consecutive degenerate vertices into clusters.
    // A cluster is a maximal run of degenerate vertices.
    // Each cluster is replaced by a single representative vertex.
    let mut clusters: Vec<Vec<usize>> = Vec::new();
    let mut current_cluster: Vec<usize> = Vec::new();
    let n = ring_3d.len();

    for i in 0..n {
        if is_degenerate[i] {
            current_cluster.push(i);
        } else {
            if !current_cluster.is_empty() {
                clusters.push(std::mem::take(&mut current_cluster));
            }
        }
    }
    // Handle wrap-around: if first and last vertices are both degenerate,
    // merge the last cluster into the first
    if !current_cluster.is_empty() {
        if !clusters.is_empty() && is_degenerate[0] {
            // Wrap-around: merge last partial cluster into first cluster
            let last_cluster = std::mem::take(&mut current_cluster);
            clusters[0].extend(last_cluster);
        } else {
            clusters.push(current_cluster);
        }
    }

    // Build index map: old index → new index (after collapsing)
    let mut index_map: Vec<Option<usize>> = vec![None; n];
    let mut new_3d: Vec<Point3d> = Vec::new();
    let mut new_uv: Vec<Point2d> = Vec::new();

    // For each cluster, choose a representative (the one whose 3D point
    // is closest to the centroid of all cluster 3D points)
    for cluster in &clusters {
        // Compute centroid of cluster's 3D points
        let mut cx = 0.0_f64;
        let mut cy = 0.0_f64;
        let mut cz = 0.0_f64;
        for &idx in cluster {
            cx += ring_3d[idx].x;
            cy += ring_3d[idx].y;
            cz += ring_3d[idx].z;
        }
        let n_pts = cluster.len() as f64;
        cx /= n_pts;
        cy /= n_pts;
        cz /= n_pts;

        // Find the vertex closest to the centroid
        let mut best_idx = cluster[0];
        let mut best_dist = f64::MAX;
        for &idx in cluster {
            let dx = ring_3d[idx].x - cx;
            let dy = ring_3d[idx].y - cy;
            let dz = ring_3d[idx].z - cz;
            let dist = dx * dx + dy * dy + dz * dz;
            if dist < best_dist {
                best_dist = dist;
                best_idx = idx;
            }
        }

        // Use the representative vertex for the whole cluster.
        // For the UV, use the centroid UV (average) — this gives a
        // more stable position for earcutr to work with.
        let rep_3d = ring_3d[best_idx];
        let mut avg_u = 0.0_f64;
        let mut avg_v = 0.0_f64;
        for &idx in cluster {
            avg_u += ring_uv[idx].u;
            avg_v += ring_uv[idx].v;
        }
        avg_u /= n_pts;
        avg_v /= n_pts;
        let rep_uv = Point2d::new(avg_u, avg_v);

        let new_idx = new_3d.len();
        new_3d.push(rep_3d);
        new_uv.push(rep_uv);

        for &idx in cluster {
            index_map[idx] = Some(new_idx);
        }
    }

    // Add non-degenerate vertices
    for i in 0..n {
        if !is_degenerate[i] {
            let new_idx = new_3d.len();
            new_3d.push(ring_3d[i]);
            new_uv.push(ring_uv[i]);
            index_map[i] = Some(new_idx);
        }
    }

    // If we collapsed everything to < 3 vertices, return the original
    if new_3d.len() < 3 {
        return (ring_3d.to_vec(), ring_uv.to_vec());
    }

    (new_3d, new_uv)
}

// ============================================================
// Phase 2: Adaptive refinement
// ============================================================

/// Adaptively refine triangles by subdividing those whose chord deviation
/// from the surface exceeds `max_deviation`.
///
/// For each triangle, checks the deviation of edge midpoints from the
/// true surface. If any edge midpoint deviates too much, the triangle
/// is subdivided by inserting midpoints and splitting into sub-triangles.
///
/// This is an iterative process with a maximum number of refinement passes
/// to prevent infinite loops on pathological surfaces.
///
/// # Algorithm
/// 1. For each triangle edge, compute the UV midpoint.
/// 2. Evaluate the surface at the midpoint → p_mid_3d.
/// 3. Compute the chord midpoint (average of endpoints' 3D positions).
/// 4. If |p_mid_3d - chord_mid| > max_deviation, mark the edge for subdivision.
/// 5. Subdivide marked triangles using midpoint insertion (Longest-edge bisection).
fn adaptive_refine_triangles(
    triangles: &mut Vec<[u32; 3]>,
    all_uv: &mut Vec<Point2d>,
    surface: &Surface,
    max_deviation: f64,
    max_face_triangles: usize,
    domain: &ParametricDomain,
) {
    let max_passes = 3; // Maximum refinement iterations
    let deviation_sq_threshold = max_deviation * max_deviation;

    for _pass in 0..max_passes {
        if triangles.len() >= max_face_triangles {
            break;
        }

        // Find triangles that need refinement
        let mut edges_to_split: std::collections::HashSet<(u32, u32)> = std::collections::HashSet::new();

        for tri in triangles.iter() {
            let edges = [
                (tri[0].min(tri[1]), tri[0].max(tri[1])),
                (tri[1].min(tri[2]), tri[1].max(tri[2])),
                (tri[2].min(tri[0]), tri[2].max(tri[0])),
            ];

            for (a, b) in &edges {
                if edges_to_split.contains(&(*a, *b)) {
                    continue; // Already marked
                }

                let uv_a = all_uv[*a as usize];
                let uv_b = all_uv[*b as usize];

                // UV midpoint
                let mid_u = (uv_a.u + uv_b.u) * 0.5;
                let mid_v = (uv_a.v + uv_b.v) * 0.5;

                // Evaluate surface at midpoint
                let p_mid_surface = surface.point_at(mid_u, mid_v);

                // Chord midpoint: average of endpoint 3D positions
                let p_a = surface.point_at(uv_a.u, uv_a.v);
                let p_b = surface.point_at(uv_b.u, uv_b.v);
                let chord_mid = Point3d::new(
                    (p_a.x + p_b.x) * 0.5,
                    (p_a.y + p_b.y) * 0.5,
                    (p_a.z + p_b.z) * 0.5,
                );

                // Check deviation
                let dx = p_mid_surface.x - chord_mid.x;
                let dy = p_mid_surface.y - chord_mid.y;
                let dz = p_mid_surface.z - chord_mid.z;
                let deviation_sq = dx * dx + dy * dy + dz * dz;

                if deviation_sq > deviation_sq_threshold {
                    edges_to_split.insert((*a, *b));
                }
            }
        }

        if edges_to_split.is_empty() {
            break; // No more refinement needed
        }

        // Compute midpoint UVs for each edge to split
        let mut edge_midpoint: std::collections::HashMap<(u32, u32), u32> = std::collections::HashMap::new();

        for &(a, b) in &edges_to_split {
            let uv_a = all_uv[a as usize];
            let uv_b = all_uv[b as usize];
            let mid_uv = Point2d::new(
                (uv_a.u + uv_b.u) * 0.5,
                (uv_a.v + uv_b.v) * 0.5,
            );

            // Only add if the midpoint is inside the domain
            if domain.contains(&mid_uv) {
                let mid_idx = all_uv.len() as u32;
                all_uv.push(mid_uv);
                edge_midpoint.insert((a, b), mid_idx);
            }
        }

        // Subdivide triangles
        let mut new_triangles: Vec<[u32; 3]> = Vec::with_capacity(triangles.len());

        for tri in triangles.drain(..) {
            let a = tri[0];
            let b = tri[1];
            let c = tri[2];

            // Get midpoint indices (if edge was split)
            let ab_key = (a.min(b), a.max(b));
            let bc_key = (b.min(c), b.max(c));
            let ca_key = (c.min(a), c.max(a));

            let ab_mid = edge_midpoint.get(&ab_key).copied();
            let bc_mid = edge_midpoint.get(&bc_key).copied();
            let ca_mid = edge_midpoint.get(&ca_key).copied();

            match (ab_mid, bc_mid, ca_mid) {
                // No edges split — keep the triangle
                (None, None, None) => {
                    new_triangles.push(tri);
                }
                // One edge split — split into 2 triangles
                (Some(m), None, None) => {
                    // AB split: vertex M on AB
                    // Triangle 1: A, M, C
                    // Triangle 2: M, B, C
                    new_triangles.push([a, m, c]);
                    new_triangles.push([m, b, c]);
                }
                (None, Some(m), None) => {
                    // BC split: vertex M on BC
                    new_triangles.push([a, b, m]);
                    new_triangles.push([a, m, c]);
                }
                (None, None, Some(m)) => {
                    // CA split: vertex M on CA
                    new_triangles.push([a, b, m]);
                    new_triangles.push([b, c, m]);
                }
                // Two edges split — split into 3 triangles
                (Some(mab), Some(mbc), None) => {
                    // AB and BC split
                    new_triangles.push([a, mab, mbc]);
                    new_triangles.push([mab, b, mbc]);
                    new_triangles.push([a, mbc, c]);
                }
                (Some(mab), None, Some(mca)) => {
                    // AB and CA split
                    new_triangles.push([a, mab, mca]);
                    new_triangles.push([mab, b, c]);
                    new_triangles.push([mab, c, mca]);
                }
                (None, Some(mbc), Some(mca)) => {
                    // BC and CA split
                    new_triangles.push([a, b, mbc]);
                    new_triangles.push([a, mbc, mca]);
                    new_triangles.push([mbc, c, mca]);
                }
                // All three edges split — split into 4 triangles (central triangle)
                (Some(mab), Some(mbc), Some(mca)) => {
                    // 4 sub-triangles
                    new_triangles.push([a, mab, mca]);
                    new_triangles.push([mab, b, mbc]);
                    new_triangles.push([mca, mbc, c]);
                    new_triangles.push([mab, mbc, mca]); // Central triangle
                }
            }
        }

        *triangles = new_triangles;

        // Check triangle budget
        if triangles.len() >= max_face_triangles {
            break;
        }
    }
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
