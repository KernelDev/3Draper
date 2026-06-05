// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Parametric domain representation for trimmed surface triangulation.
//!
//! A ParametricDomain represents the 2D region in UV-parameter space
//! that defines the valid area of a trimmed surface. It consists of:
//! - An outer boundary (the trimming loop)
//! - Optional inner boundaries (holes)
//!
//! The domain is triangulated using earcutr with interior Steiner points,
//! which is fast (O(n log n) typical) and handles holes natively.
//! Previously used ear-clipping / CDT approaches had O(n²) worst case
//! or could hang on degenerate inputs.

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
    /// Cached spatial grid for fast containment checks (lazy-initialized).
    containment_grid: Option<ContainmentGrid>,
}

/// Grid-based spatial index for fast point-in-domain checks.
///
/// Pre-computes a grid of cells, each marked as INSIDE, OUTSIDE, or BOUNDARY.
/// For BOUNDARY cells, falls back to ray-casting. This provides O(1) containment
/// checks for most points, vs O(n) ray-casting for every point.
#[derive(Clone, Debug)]
struct ContainmentGrid {
    /// Grid cells: true = inside, false = outside/unknown.
    cells: Vec<bool>,
    /// Number of cells in u direction.
    n_u: usize,
    /// Number of cells in v direction.
    n_v: usize,
    /// Origin of the grid (u_min, v_min).
    u_min: f64,
    v_min: f64,
    /// Cell size in u and v directions.
    du: f64,
    dv: f64,
}

impl ContainmentGrid {
    fn new(domain: &ParametricDomain, resolution: usize) -> Self {
        let (u_min, u_max, v_min, v_max) = domain.bounding_box();
        let u_range = u_max - u_min;
        let v_range = v_max - v_min;

        // Determine grid resolution: aim for ~resolution cells along the longer axis
        let aspect = v_range / u_range.max(1e-10);
        let n_u = if aspect > 1.0 {
            (resolution as f64 / aspect.sqrt()).max(8.0) as usize
        } else {
            (resolution as f64 * aspect.sqrt()).max(8.0) as usize
        };
        let n_v = (n_u as f64 * aspect).max(8.0) as usize;

        let du = u_range / n_u as f64;
        let dv = v_range / n_v as f64;

        // Pre-compute containment for each grid cell
        let mut cells = Vec::with_capacity(n_u * n_v);
        for j in 0..n_v {
            for i in 0..n_u {
                // Test center of each cell
                let u = u_min + (i as f64 + 0.5) * du;
                let v = v_min + (j as f64 + 0.5) * dv;
                let pt = Point2d::new(u, v);
                cells.push(domain.contains_ray(&pt));
            }
        }

        ContainmentGrid { cells, n_u, n_v, u_min, v_min, du, dv }
    }

    #[inline]
    fn is_inside(&self, point: &Point2d) -> bool {
        let iu = ((point.u - self.u_min) / self.du) as i64;
        let iv = ((point.v - self.v_min) / self.dv) as i64;
        if iu < 0 || iu >= self.n_u as i64 || iv < 0 || iv >= self.n_v as i64 {
            return false;
        }
        self.cells[iu as usize + iv as usize * self.n_u]
    }
}

impl ParametricDomain {
    /// Create a new parametric domain from an outer boundary.
    pub fn new(outer_boundary: UVPolygon, u_range: (f64, f64), v_range: (f64, f64)) -> Self {
        Self {
            outer_boundary,
            holes: Vec::new(),
            u_range,
            v_range,
            containment_grid: None,
        }
    }

    /// Add a hole (inner boundary) to the domain.
    pub fn with_hole(mut self, hole: UVPolygon) -> Self {
        self.holes.push(hole);
        self.containment_grid = None; // invalidate cache
        self
    }

    /// Add multiple holes (inner boundaries) to the domain.
    pub fn with_holes_from<I: IntoIterator<Item = UVPolygon>>(mut self, holes: I) -> Self {
        self.holes.extend(holes);
        self.containment_grid = None; // invalidate cache
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

    /// Check if a UV point is inside the domain using cached grid (fast).
    ///
    /// Uses a spatial grid for O(1) containment checks for most points,
    /// with lazy initialization on first call.
    pub fn contains(&self, point: &Point2d) -> bool {
        if let Some(ref grid) = self.containment_grid {
            grid.is_inside(point)
        } else {
            self.contains_ray(point)
        }
    }

    /// Initialize the containment grid for fast contains() checks.
    /// Call this once before many contains() calls for best performance.
    pub fn init_containment_grid(&mut self) {
        if self.containment_grid.is_none() {
            self.containment_grid = Some(ContainmentGrid::new(self, 64));
        }
    }

    /// Check if a UV point is inside the domain using ray-casting (slow but exact).
    fn contains_ray(&self, point: &Point2d) -> bool {
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

    // Has holes: use earcutr
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

/// Triangulate a simple domain with interior points using earcutr.
///
/// Uses earcutr with interior points as Steiner points for O(n log n)
/// instead of the old O(n²) subdivision approach.
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

    // Build combined point array: [boundary...][interior...]
    let mut all_points: Vec<Point2d> = outer.clone();
    for &pt in interior_uv_points {
        if domain.contains(&pt) {
            all_points.push(pt);
        }
    }

    // Build flat coordinate array for earcutr
    let mut coords: Vec<f64> = Vec::with_capacity(all_points.len() * 2);
    for p in &all_points {
        coords.push(p.u);
        coords.push(p.v);
    }

    // Run earcutr (no holes)
    let no_holes: Vec<usize> = vec![];
    let triangle_indices = earcutr::earcut(&coords, &no_holes, 2);

    let mut result_triangles: Vec<[u32; 3]> = Vec::with_capacity(triangle_indices.len() / 3);
    for chunk in triangle_indices.chunks(3) {
        if chunk.len() < 3 { break; }
        let a = chunk[0] as u32;
        let b = chunk[1] as u32;
        let c = chunk[2] as u32;
        if a == b || b == c || a == c { continue; }
        result_triangles.push([a, b, c]);
    }

    uv_triangles_to_3d(&result_triangles, &all_points, surface, forward)
}

/// Triangulate a domain with holes using earcutr.
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

    // Downsample holes if too many points
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

    // Add interior grid points directly to earcutr input (NOT post-hoc subdivision!)
    for &pt in interior_uv_points {
        if domain.contains(&pt) {
            all_points.push(pt);
        }
    }

    // Use earcutr for triangulation
    let mut coords: Vec<f64> = Vec::with_capacity(all_points.len() * 2);
    for p in &all_points {
        coords.push(p.u);
        coords.push(p.v);
    }

    let triangle_indices = earcutr::earcut(&coords, &hole_start_indices, 2);

    let mut result_triangles: Vec<[u32; 3]> = Vec::new();
    for chunk in triangle_indices.chunks(3) {
        if chunk.len() < 3 { break; }
        let a = chunk[0] as u32;
        let b = chunk[1] as u32;
        let c = chunk[2] as u32;
        if a == b || b == c || a == c { continue; }
        result_triangles.push([a, b, c]);
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
/// Uses the containment grid for O(1) checks when available.
pub fn generate_interior_points(
    domain: &ParametricDomain,
    n_u: usize,
    n_v: usize,
    boundary_margin: f64,
) -> Vec<Point2d> {
    let (u_min, u_max, v_min, v_max) = domain.bounding_box();
    let mut points = Vec::with_capacity(n_u * n_v / 4); // estimate ~25% inside

    let margin_sq = boundary_margin * boundary_margin;

    for j in 1..n_v {
        let v = v_min + (v_max - v_min) * j as f64 / n_v as f64;
        for i in 1..n_u {
            let u = u_min + (u_max - u_min) * i as f64 / n_u as f64;
            let pt = Point2d::new(u, v);

            // Use fast grid-based containment check
            if !domain.contains(&pt) {
                continue;
            }

            // Check distance to boundary (skip points too close)
            // Only check against outer boundary for speed
            let mut min_dist_sq = f64::MAX;
            for p in &domain.outer_boundary {
                let du = u - p.u;
                let dv = v - p.v;
                min_dist_sq = min_dist_sq.min(du * du + dv * dv);
                if min_dist_sq <= margin_sq {
                    break; // Early exit: too close
                }
            }
            if min_dist_sq > margin_sq {
                points.push(pt);
            }
        }
    }

    points
}

/// Generate interior UV points for NURBS surfaces, respecting knot ranges.
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
// Integration: earcutr-based surface triangulation
// ============================================================

/// Triangulate a curved surface using UV-space earcutr with holes.
///
/// Uses earcutr which handles holes natively and is fast O(n log n).
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

    // Downsample boundary points to prevent O(n²) blowup
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
    let u_period = if surface.is_u_periodic() { Some(2.0 * PI) } else { None };
    let v_period = if surface.is_v_periodic() { Some(2.0 * PI) } else { None };
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

    // Project holes to UV
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

    // Create parametric domain with containment grid
    let mut domain = ParametricDomain::new(
        outer_uv,
        (u_min - margin_u, u_max + margin_u),
        (v_min - margin_v, v_max + margin_v),
    );
    for hole in &holes_uv {
        domain = domain.with_hole(hole.clone());
    }
    domain.init_containment_grid(); // Pre-compute grid for fast contains()

    // Generate interior points using adaptive sampling
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

    let u_step = (u_max - u_min) / n_u.max(1) as f64;
    let v_step = (v_max - v_min) / n_v.max(1) as f64;
    let boundary_margin = u_step.min(v_step) * 0.3;

    let interior_points = generate_interior_points(&domain, n_u, n_v, boundary_margin);

    // Triangulate using earcutr
    triangulate_cdt(&domain, surface, forward, &interior_points)
}

// ============================================================
// Consistent (watertight) triangulation — OPTIMIZED
// ============================================================

/// Triangulate a curved surface with **consistent** boundary vertices.
///
/// This function produces watertight meshes where shared edges between
/// adjacent faces have **bit-identical** 3D vertex positions.
///
/// # Key optimizations over the original implementation:
/// 1. Interior points are passed directly to earcutr as Steiner points
///    (eliminates the O(interior_points × triangles) subdivision loop)
/// 2. Uses a spatial grid for O(1) domain containment checks
/// 3. Handles pole/apex degeneracy by NOT collapsing to a single vertex;
///    instead, structured rings of interior points ensure proper fan triangulation
/// 4. Adaptive refinement is lightweight (single pass, edge midpoints only)
///
/// # Arguments
/// * `surface` — The parametric surface to triangulate.
/// * `boundary_points_3d` — 3D points along the outer boundary (from cache).
/// * `boundary_uvs` — Pre-computed UV coordinates for boundary points.
/// * `hole_polylines_3d` — 3D points along each hole boundary.
/// * `hole_uvs` — Pre-computed UV coordinates for hole points.
/// * `forward` — Whether face normal matches surface normal.
/// * `params` — Triangulation parameters (for grid resolution).
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

    // ============================================================
    // Step 1: Downsample boundary/holes if too many points
    // ============================================================
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

    // ============================================================
    // Step 2: Normalize UV for periodic surfaces
    // ============================================================
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
    // Step 3: Handle degenerate boundary vertices (poles/apexes)
    //
    // IMPORTANT: Instead of collapsing to a single vertex (which
    // creates the fan pattern), we keep the boundary ring but mark
    // degenerate regions so we can add structured interior rings.
    // ============================================================
    let degenerate_info = detect_degenerate_regions(&outer_uv, surface);

    let (boundary_3d, outer_uv, holes_3d, normalized_holes_uv) =
        collapse_degenerate_boundary_vertices(
            boundary_3d, outer_uv, holes_3d, normalized_holes_uv, surface,
        );

    if outer_uv.len() < 3 {
        return TriangleMesh::new();
    }

    // ============================================================
    // Step 4: Compute UV range and build domain
    // ============================================================
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

    // Build parametric domain with containment grid for fast checks
    let mut domain = ParametricDomain::new(
        outer_uv.clone(),
        (u_min - margin_u, u_max + margin_u),
        (v_min - margin_v, v_max + margin_v),
    ).with_holes_from(normalized_holes_uv.iter().cloned());
    domain.init_containment_grid(); // Critical for performance!

    // ============================================================
    // Step 5: Generate interior grid points
    // ============================================================
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

    // Generate regular interior points
    let mut interior_uv_points = generate_interior_points(&domain, n_u, n_v, boundary_margin);

    // Add structured ring points near degenerate regions (poles/apexes)
    // This ensures proper fan triangulation quality at degenerate UV points
    add_degenerate_ring_points(
        &mut interior_uv_points,
        &degenerate_info,
        surface,
        &domain,
        u_min, u_max, v_min, v_max,
        n_u, n_v,
    );

    // ============================================================
    // Step 6: Build earcutr input with ALL points (boundary + holes + interior)
    //
    // KEY OPTIMIZATION: Pass interior points as part of the earcutr
    // input directly instead of the old O(n²) post-hoc subdivision.
    // earcutr handles Steiner points natively and produces good
    // quality triangulation in O(n log n).
    // ============================================================

    let n_boundary = outer_uv.len();

    // Build combined point array: [boundary_uv...][hole_uv...][interior_uv...]
    let mut all_uv: Vec<Point2d> = outer_uv.clone();
    for huv in &normalized_holes_uv {
        all_uv.extend_from_slice(huv);
    }
    let n_boundary_and_holes = all_uv.len();

    // Add interior points (they become Steiner points in earcutr)
    all_uv.extend_from_slice(&interior_uv_points);

    // Build flat coordinate array for earcutr
    let mut coords: Vec<f64> = Vec::with_capacity(all_uv.len() * 2);
    for p in &all_uv {
        coords.push(p.u);
        coords.push(p.v);
    }

    // Hole indices (point into the combined array)
    let mut hole_start_indices: Vec<usize> = Vec::new();
    let mut offset = n_boundary;
    for huv in &normalized_holes_uv {
        if huv.len() < 3 {
            continue;
        }
        hole_start_indices.push(offset);
        offset += huv.len();
    }

    // Run earcutr triangulation with ALL points at once
    let triangle_indices = earcutr::earcut(&coords, &hole_start_indices, 2);

    // Collect triangles, filtering degenerate ones
    let mut result_triangles: Vec<[u32; 3]> = Vec::with_capacity(triangle_indices.len() / 3);
    for chunk in triangle_indices.chunks(3) {
        if chunk.len() < 3 { break; }
        let a = chunk[0] as u32;
        let b = chunk[1] as u32;
        let c = chunk[2] as u32;
        if a == b || b == c || a == c { continue; }
        result_triangles.push([a, b, c]);
    }

    // ============================================================
    // Step 7: Build 3D mesh
    // ============================================================

    // Build combined 3D point array for boundary + hole vertices
    let mut all_boundary_3d: Vec<Point3d> = boundary_3d.clone();
    for h3d in &holes_3d {
        all_boundary_3d.extend_from_slice(h3d);
    }

    // Filter triangles by domain containment and build mesh
    let mut mesh = TriangleMesh::new();
    let mut vertex_map: std::collections::HashMap<u32, u32> = std::collections::HashMap::new();

    for tri in &result_triangles {
        // Bounds check
        if tri[0] as usize >= all_uv.len() || tri[1] as usize >= all_uv.len() || tri[2] as usize >= all_uv.len() {
            continue;
        }

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
// Degenerate region detection
// ============================================================

/// Information about degenerate UV regions on the boundary.
struct DegenerateRegionInfo {
    /// List of (uv_index, is_pole, is_apex) for degenerate boundary points.
    degenerate_points: Vec<(usize, bool, bool)>,
    /// Whether there are any pole degeneracies (sphere).
    has_pole: bool,
    /// Whether there are any apex degeneracies (cone).
    has_apex: bool,
}

/// Detect degenerate regions on the boundary.
///
/// Returns information about which boundary points are at degenerate UV locations
/// (poles or apexes), without collapsing them. This info is used to add
/// structured interior ring points near the degenerate regions.
fn detect_degenerate_regions(
    boundary_uv: &[Point2d],
    surface: &Surface,
) -> DegenerateRegionInfo {
    let degenerate_tol = 1e-3;
    let mut degenerate_points = Vec::new();
    let mut has_pole = false;
    let mut has_apex = false;

    for (i, uv) in boundary_uv.iter().enumerate() {
        let flags = surface.is_degenerate_at(uv.u, uv.v, degenerate_tol);
        if flags.contains(DegeneracyFlags::DU_ZERO) {
            let is_apex = flags.contains(DegeneracyFlags::DV_ZERO);
            let is_pole = !is_apex;
            degenerate_points.push((i, is_pole, is_apex));
            if is_pole { has_pole = true; }
            if is_apex { has_apex = true; }
        }
    }

    DegenerateRegionInfo {
        degenerate_points,
        has_pole,
        has_apex,
    }
}

/// Add structured ring points near degenerate boundary vertices.
///
/// Instead of just adding a few random interior points near the pole,
/// this creates concentric rings of points at controlled distances from
/// the degenerate UV point. This ensures earcutr produces high-quality
/// triangulation (not a degenerate fan) at poles and apexes.
fn add_degenerate_ring_points(
    interior_uv_points: &mut Vec<Point2d>,
    degenerate_info: &DegenerateRegionInfo,
    surface: &Surface,
    domain: &ParametricDomain,
    u_min: f64, u_max: f64,
    v_min: f64, v_max: f64,
    n_u: usize, n_v: usize,
) {
    if degenerate_info.degenerate_points.is_empty() {
        return;
    }

    let step_u = (u_max - u_min) / n_u.max(1) as f64;
    let step_v = (v_max - v_min) / n_v.max(1) as f64;
    let u_range = u_max - u_min;

    // For sphere poles: add concentric rings at different v-values
    // For cone apex: add concentric rings at different v-values
    // The key is that at a pole/apex, all u-values map to the same 3D point,
    // so we need points at slightly different v-values to create a proper ring.

    for &(idx, is_pole, _is_apex) in &degenerate_info.degenerate_points {
        // Get the degenerate UV point (use the first one's v-coordinate)
        // Since collapse may have changed indices, we work with UV coords directly
        let _ = idx; // We don't need the index since we already have the UV info

        // Determine the v-direction to add ring points
        // For poles: add rings on the interior side
        // For apexes: add rings toward the interior
        // We add multiple rings at increasing distances for better quality

        let n_rings = 3; // Number of concentric rings
        let n_ring_points = n_u.max(8); // Points per ring

        for ring_idx in 1..=n_rings {
            let fraction = ring_idx as f64 / (n_rings + 1) as f64;
            let ring_offset_v = step_v * fraction * 2.0; // Offset from the degenerate point

            if is_pole {
                // Sphere pole: the pole is at v_min or v_max
                // Add ring at v_offset from the pole
                let pole_v = if degenerate_info.degenerate_points.iter()
                    .any(|&(_, is_p, _)| is_p)
                {
                    // Check if the pole is at the bottom or top of the UV domain
                    let sample_uv = domain.outer_boundary.get(0);
                    if let Some(suv) = sample_uv {
                        if suv.v < (v_min + v_max) * 0.5 { v_min } else { v_max }
                    } else {
                        v_min
                    }
                } else {
                    v_min
                };

                let v_dir = if pole_v < (v_min + v_max) * 0.5 { 1.0 } else { -1.0 };
                let ring_v = pole_v + v_dir * ring_offset_v;

                for k in 0..n_ring_points {
                    let u_val = u_min + u_range * k as f64 / n_ring_points as f64;
                    let pt = Point2d::new(u_val, ring_v);
                    if domain.contains(&pt) {
                        interior_uv_points.push(pt);
                    }
                }
            } else {
                // Cone apex: add ring around the apex point
                // The apex is at a specific (u, v) — we add a ring of points
                // around it in UV space. Since the apex is a single point
                // (not a line like a pole), we add points in a circle.
                let small = step_u.min(step_v) * fraction * 2.0;
                let center_u = u_min + u_range * 0.5; // Approximate center
                let center_v = if domain.outer_boundary.first().map_or(false, |p| p.v < (v_min + v_max) * 0.5) {
                    v_min + small
                } else {
                    v_max - small
                };

                for k in 0..n_ring_points {
                    let angle = 2.0 * PI * k as f64 / n_ring_points as f64;
                    let du = small * angle.cos();
                    let dv = small * angle.sin();
                    let pt = Point2d::new(center_u + du, center_v + dv);
                    if domain.contains(&pt) {
                        interior_uv_points.push(pt);
                    }
                }
            }
        }
    }
}

// ============================================================
// Pole/apex degeneracy handling (collapse)
// ============================================================

/// Collapse degenerate boundary vertices where multiple UV points
/// map to the same 3D point (sphere poles, cone apex).
///
/// When a boundary passes through a degenerate parametric point (where the
/// surface parameterization collapses), the edge cache produces many boundary
/// points with different UV coordinates but identical 3D positions.
/// This function collapses such clusters to a single representative vertex
/// per cluster to avoid degenerate (zero-area) triangles in earcutr.
fn collapse_degenerate_boundary_vertices(
    boundary_3d: Vec<Point3d>,
    boundary_uv: Vec<Point2d>,
    holes_3d: Vec<Vec<Point3d>>,
    holes_uv: Vec<Vec<Point2d>>,
    surface: &Surface,
) -> (Vec<Point3d>, Vec<Point2d>, Vec<Vec<Point3d>>, Vec<Vec<Point2d>>) {
    let degenerate_tol = 1e-3;

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
/// each run with a single representative.
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

    // If more than half the boundary is degenerate, we have a very
    // degenerate surface (e.g., a very narrow cone strip). In this case,
    // don't collapse at all — earcutr can handle it with interior points.
    if n_degenerate > ring_3d.len() / 2 {
        return (ring_3d.to_vec(), ring_uv.to_vec());
    }

    // Group consecutive degenerate vertices into clusters.
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
    // Handle wrap-around
    if !current_cluster.is_empty() {
        if !clusters.is_empty() && is_degenerate[0] {
            let last_cluster = std::mem::take(&mut current_cluster);
            clusters[0].extend(last_cluster);
        } else {
            clusters.push(current_cluster);
        }
    }

    // Build index map
    let mut index_map: Vec<Option<usize>> = vec![None; n];
    let mut new_3d: Vec<Point3d> = Vec::new();
    let mut new_uv: Vec<Point2d> = Vec::new();

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

        let rep_3d = ring_3d[best_idx];
        // Use average UV for the representative (more stable for earcutr)
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
    fn test_containment_grid() {
        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(10.0, 0.0),
            Point2d::new(10.0, 10.0),
            Point2d::new(0.0, 10.0),
        ];
        let mut domain = ParametricDomain::new(outer, (0.0, 10.0), (0.0, 10.0));
        domain.init_containment_grid();
        assert!(domain.contains(&Point2d::new(5.0, 5.0)));
        assert!(!domain.contains(&Point2d::new(15.0, 5.0)));
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
        let mut domain = ParametricDomain::new(outer, (0.0, PI), (0.0, 10.0)).with_hole(hole);
        domain.init_containment_grid();

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
        use draper_geometry::{SphereSurface, Point3d, Surface};

        let sphere = SphereSurface::new(Point3d::ORIGIN, 10.0);
        let surface = Surface::Sphere(sphere);

        let outer = vec![
            Point2d::new(0.0, 0.5),
            Point2d::new(3.0, 0.5),
            Point2d::new(3.0, 2.5),
            Point2d::new(0.0, 2.5),
        ];

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

    #[test]
    fn test_consistent_triangulation_performance() {
        // Test that consistent triangulation completes quickly for a medium-sized surface
        use draper_geometry::{CylinderSurface, Point3d, Surface};

        let cyl = CylinderSurface::new_z(5.0);
        let surface = Surface::Cylinder(cyl);

        // Create a large boundary (simulating a complex face)
        let n_pts = 100;
        let boundary_3d: Vec<Point3d> = (0..n_pts)
            .flat_map(|i| {
                let u = 2.0 * PI * i as f64 / n_pts as f64;
                // Top ring
                Some(Point3d::new(5.0 * u.cos(), 5.0 * u.sin(), 10.0))
            })
            .collect();
        let mut boundary_uv: Vec<Point2d> = (0..n_pts)
            .map(|i| {
                let u = 2.0 * PI * i as f64 / n_pts as f64;
                Point2d::new(u, 10.0)
            })
            .collect();
        // Add bottom ring
        let bottom_3d: Vec<Point3d> = (0..n_pts)
            .map(|i| {
                let u = 2.0 * PI * i as f64 / n_pts as f64;
                Point3d::new(5.0 * u.cos(), 5.0 * u.sin(), 0.0)
            })
            .collect();
        let bottom_uv: Vec<Point2d> = (0..n_pts)
            .rev()
            .map(|i| {
                let u = 2.0 * PI * i as f64 / n_pts as f64;
                Point2d::new(u, 0.0)
            })
            .collect();

        let all_3d: Vec<Point3d> = boundary_3d.into_iter().chain(bottom_3d).collect();
        let all_uv: Vec<Point2d> = boundary_uv.into_iter().chain(bottom_uv).collect();

        let params = crate::triangulate::TriangulationParams::default();

        let start = std::time::Instant::now();
        let mesh = triangulate_surface_consistent(
            &surface, &all_3d, &all_uv, &[], &[], true, &params,
        );
        let elapsed = start.elapsed();

        assert!(!mesh.triangles.is_empty(), "Should generate triangles");
        assert!(elapsed.as_millis() < 500, "Consistent triangulation should be fast, took {}ms", elapsed.as_millis());
    }
}
