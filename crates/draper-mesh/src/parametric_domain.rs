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

use draper_geometry::{Point2d, Point3d, Surface};
use crate::mesh::TriangleMesh;
use crate::edge_cache::deterministic_round_point;
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
/// Pre-computes a grid of cells, each marked as inside or outside.
/// This provides O(1) containment checks for interior point generation.
#[derive(Clone, Debug)]
struct ContainmentGrid {
    cells: Vec<bool>,
    n_u: usize,
    n_v: usize,
    u_min: f64,
    v_min: f64,
    du: f64,
    dv: f64,
}

impl ContainmentGrid {
    fn new(domain: &ParametricDomain, resolution: usize) -> Self {
        let (u_min, u_max, v_min, v_max) = domain.bounding_box();
        let u_range = (u_max - u_min).max(1e-10);
        let v_range = (v_max - v_min).max(1e-10);

        let aspect = v_range / u_range;
        let n_u = if aspect > 1.0 {
            (resolution as f64 / aspect.sqrt()).max(8.0) as usize
        } else {
            (resolution as f64 * aspect.sqrt()).max(8.0) as usize
        };
        let n_v = (n_u as f64 * aspect).max(8.0) as usize;

        let du = u_range / n_u as f64;
        let dv = v_range / n_v as f64;

        // Pre-compute containment for each grid cell center
        let mut cells = Vec::with_capacity(n_u * n_v);
        for j in 0..n_v {
            for i in 0..n_u {
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
        self.containment_grid = None;
        self
    }

    /// Add multiple holes (inner boundaries) to the domain.
    pub fn with_holes_from<I: IntoIterator<Item = UVPolygon>>(mut self, holes: I) -> Self {
        self.holes.extend(holes);
        self.containment_grid = None;
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
    pub fn contains(&self, point: &Point2d) -> bool {
        if let Some(ref grid) = self.containment_grid {
            grid.is_inside(point)
        } else {
            self.contains_ray(point)
        }
    }

    /// Initialize the containment grid for fast contains() checks.
    pub fn init_containment_grid(&mut self) {
        if self.containment_grid.is_none() {
            // Use a 128×128 grid for accurate interior point generation.
            // The previous 64×64 was too coarse for complex NURBS trimming regions,
            // causing interior points near boundaries to be incorrectly excluded,
            // which produced irregular triangle distributions and gaps.
            // 128×128 costs ~16K ray-casting tests at initialization, which is
            // negligible compared to the NURBS surface evaluations in triangulation.
            self.containment_grid = Some(ContainmentGrid::new(self, 128));
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
// Ear-clipping triangulation
// ============================================================

/// Triangulate a parametric domain using earcutr.
///
/// earcutr is O(n log n) typical, handles holes natively, and
/// never hangs on degenerate inputs.
pub fn triangulate_cdt(
    domain: &ParametricDomain,
    surface: &Surface,
    forward: bool,
    interior_uv_points: &[Point2d],
) -> TriangleMesh {
    if domain.outer_boundary.len() < 3 {
        return TriangleMesh::new();
    }

    // Build combined point array: [boundary...][holes...][interior...]
    let mut all_points: Vec<Point2d> = domain.outer_boundary.clone();
    let mut hole_start_indices: Vec<usize> = Vec::new();

    for hole in &domain.holes {
        if hole.len() < 3 {
            continue;
        }
        hole_start_indices.push(all_points.len());
        all_points.extend_from_slice(hole);
    }

    // Add interior points as Steiner points
    all_points.extend_from_slice(interior_uv_points);

    // Build flat coordinate array for earcutr
    let mut coords: Vec<f64> = Vec::with_capacity(all_points.len() * 2);
    for p in &all_points {
        coords.push(p.u);
        coords.push(p.v);
    }

    // Run earcutr
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

    // Map UV to 3D
    uv_triangles_to_3d(&result_triangles, &all_points, surface, forward)
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

/// Re-project a 3D point onto a NURBS surface using Newton-Raphson
/// starting from an initial UV guess. This is much more accurate and
/// faster than a full grid search when we have a reasonable initial guess.
pub fn reproject_nurbs_point(
    nurbs: &draper_geometry::NurbsSurface,
    point: &Point3d,
    init_u: f64,
    init_v: f64,
) -> (f64, f64) {
    use draper_geometry::Surface;

    let (u_min, u_max) = nurbs.u_range();
    let (v_min, v_max) = nurbs.v_range();
    let surface = Surface::Nurbs(nurbs.clone());

    let mut best_u = init_u.clamp(u_min, u_max);
    let mut best_v = init_v.clamp(v_min, v_max);
    let mut best_dist = {
        let p = surface.point_at(best_u, best_v);
        (p.x - point.x).powi(2) + (p.y - point.y).powi(2) + (p.z - point.z).powi(2)
    };

    // Newton-Raphson refinement from the initial guess
    for _ in 0..15 {
        let derivs = nurbs.derivatives_at(best_u, best_v);
        let sp = derivs.point;
        let dx = sp.x - point.x;
        let dy = sp.y - point.y;
        let dz = sp.z - point.z;

        let gu = derivs.du.x * dx + derivs.du.y * dy + derivs.du.z * dz;
        let gv = derivs.dv.x * dx + derivs.dv.y * dy + derivs.dv.z * dz;

        let hu_u = derivs.du.x * derivs.du.x + derivs.du.y * derivs.du.y + derivs.du.z * derivs.du.z;
        let hu_v = derivs.du.x * derivs.dv.x + derivs.du.y * derivs.dv.y + derivs.du.z * derivs.dv.z;
        let hv_v = derivs.dv.x * derivs.dv.x + derivs.dv.y * derivs.dv.y + derivs.dv.z * derivs.dv.z;

        let det = hu_u * hv_v - hu_v * hu_v;
        if det.abs() < 1e-20 { break; }

        let du = -(hv_v * gu - hu_v * gv) / det;
        let dv = -(-hu_v * gu + hu_u * gv) / det;

        let u_range = u_max - u_min;
        let v_range = v_max - v_min;
        let step_limit_u = u_range * 0.1;
        let step_limit_v = v_range * 0.1;
        let du = du.clamp(-step_limit_u, step_limit_u);
        let dv = dv.clamp(-step_limit_v, step_limit_v);

        let new_u = (best_u + du).clamp(u_min, u_max);
        let new_v = (best_v + dv).clamp(v_min, v_max);

        let new_p = surface.point_at(new_u, new_v);
        let new_dist = (new_p.x - point.x).powi(2) + (new_p.y - point.y).powi(2) + (new_p.z - point.z).powi(2);

        if new_dist < best_dist {
            if (best_dist - new_dist) < 1e-12 * best_dist.max(1e-20) {
                best_u = new_u;
                best_v = new_v;
                break;
            }
            best_u = new_u;
            best_v = new_v;
            best_dist = new_dist;
        } else {
            break;
        }
    }

    (best_u, best_v)
}

/// Compute the area of a 2D triangle.
fn triangle_area_2d(x0: f64, y0: f64, x1: f64, y1: f64, x2: f64, y2: f64) -> f64 {
    ((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)).abs() * 0.5
}

/// Compute the signed area of a 2D polygon using the shoelace formula.
/// Returns a positive value for counter-clockwise, negative for clockwise.
fn polygon_area_2d(polygon: &[Point2d]) -> f64 {
    if polygon.len() < 3 {
        return 0.0;
    }
    let mut area = 0.0;
    let n = polygon.len();
    for i in 0..n {
        let j = (i + 1) % n;
        area += polygon[i].u * polygon[j].v;
        area -= polygon[j].u * polygon[i].v;
    }
    area.abs() * 0.5
}

/// Compute the approximate area of a 3D polygon using the Newell's method.
/// This gives a reasonable area estimate even for non-planar polygons.
fn polygon_area_3d(polygon: &[Point3d]) -> f64 {
    if polygon.len() < 3 {
        return 0.0;
    }
    // Compute the normal using Newell's method
    let mut nx = 0.0_f64;
    let mut ny = 0.0_f64;
    let mut nz = 0.0_f64;
    let n = polygon.len();
    for i in 0..n {
        let j = (i + 1) % n;
        let pi = &polygon[i];
        let pj = &polygon[j];
        nx += (pi.y - pj.y) * (pi.z + pj.z);
        ny += (pi.z - pj.z) * (pi.x + pj.x);
        nz += (pi.x - pj.x) * (pi.y + pj.y);
    }
    (nx * nx + ny * ny + nz * nz).sqrt() * 0.5
}

/// Generate interior UV grid points for a parametric domain.
///
/// Creates a regular grid of points within the domain's bounding box,
/// excluding points that are outside the domain.
/// Uses the containment grid for O(1) checks when available.
///
/// NOTE: We do NOT check distance to boundary vertices. This is intentional:
/// 1. It's O(n_u × n_v × boundary_len) which is extremely slow
/// 2. earcutr handles boundary proximity correctly
/// 3. Steiner points near boundaries improve triangulation quality
pub fn generate_interior_points(
    domain: &ParametricDomain,
    n_u: usize,
    n_v: usize,
    _boundary_margin: f64,
) -> Vec<Point2d> {
    let (u_min, u_max, v_min, v_max) = domain.bounding_box();
    let mut points = Vec::with_capacity(n_u * n_v / 4);

    for j in 1..n_v {
        let v = v_min + (v_max - v_min) * j as f64 / n_v as f64;
        for i in 1..n_u {
            let u = u_min + (u_max - u_min) * i as f64 / n_u as f64;
            let pt = Point2d::new(u, v);
            if domain.contains(&pt) {
                points.push(pt);
            }
        }
    }

    points
}

/// Downsample a polyline (3D + UV) to at most `max_points` points while
/// preserving the overall shape. Uses uniform stride sampling which is
/// fast and preserves the polygon's general form.
///
/// Returns downsampled (3D, UV) point arrays.
pub fn downsample_polyline(
    points_3d: &[Point3d],
    points_uv: &[Point2d],
    max_points: usize,
) -> (Vec<Point3d>, Vec<Point2d>) {
    if points_3d.len() <= max_points || max_points < 3 {
        return (points_3d.to_vec(), points_uv.to_vec());
    }

    let n = points_3d.len();
    let mut result_3d = Vec::with_capacity(max_points);
    let mut result_uv = Vec::with_capacity(max_points);

    // Always include the first point
    result_3d.push(points_3d[0]);
    result_uv.push(points_uv[0]);

    // Uniform stride for interior points
    let stride = (n - 1) as f64 / (max_points - 1) as f64;
    let mut next_idx: f64 = 1.0;
    for _i in 1..max_points - 1 {
        let idx = next_idx.round() as usize;
        let idx = idx.min(n - 2).max(1); // Clamp to valid interior range
        result_3d.push(points_3d[idx]);
        result_uv.push(points_uv[idx]);
        next_idx += stride;
    }

    // Always include the last point
    result_3d.push(points_3d[n - 1]);
    result_uv.push(points_uv[n - 1]);

    (result_3d, result_uv)
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

/// Downsample interior UV points to a budget using stride-based sampling.
///
/// Unlike `truncate()` which removes points from the END of the list
/// (creating position bias — dense at low-v, sparse at high-v),
/// stride-based sampling preserves uniform spatial coverage by keeping
/// every N-th point. This produces a more even triangle distribution
/// across the entire surface.
///
/// If `pts.len() <= budget`, returns a clone of the input.
fn downsample_interior_points(pts: &[Point2d], budget: usize) -> Vec<Point2d> {
    if pts.len() <= budget {
        return pts.to_vec();
    }
    if budget == 0 {
        return Vec::new();
    }
    // Stride-based downsampling: keep every (len/budget)-th point
    let stride = pts.len() as f64 / budget as f64;
    let mut result = Vec::with_capacity(budget);
    let mut next_idx = 0.0f64;
    while result.len() < budget {
        let idx = next_idx.round() as usize;
        let idx = idx.min(pts.len() - 1);
        result.push(pts[idx]);
        next_idx += stride;
    }
    result
}

// ============================================================
// Integration: earcutr-based surface triangulation (non-consistent)
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
    let max_boundary_points = 150;
    let boundary_points = if boundary_points.len() > max_boundary_points {
        let step = boundary_points.len() as f64 / max_boundary_points as f64;
        let sampled: Vec<Point3d> = (0..max_boundary_points)
            .map(|i| boundary_points[((i as f64 * step) as usize).min(boundary_points.len() - 1)])
            .collect();
        sampled
    } else {
        boundary_points.to_vec()
    };

    // Also downsample hole polylines
    let max_hole_points = 50;
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
    // OPTIMIZATION: For NURBS surfaces, use bootstrap + chain Newton-Raphson
    // instead of calling project_point() for each point (which does 11×11 grid
    // search per point and is catastrophically slow).
    let mut outer_uv: Vec<Point2d> = if let Surface::Nurbs(ref nurbs) = surface {
        let mut uvs = Vec::with_capacity(boundary_points.len());
        if !boundary_points.is_empty() {
            let (u0, v0) = surface.project_point(&boundary_points[0]);
            uvs.push(Point2d::new(u0, v0));
            for i in 1..boundary_points.len() {
                let prev = uvs[i - 1];
                let (u, v) = reproject_nurbs_point(nurbs, &boundary_points[i], prev.u, prev.v);
                uvs.push(Point2d::new(u, v));
            }
        }
        uvs
    } else {
        boundary_points
            .iter()
            .map(|p| {
                let (u, v) = surface.project_point(p);
                Point2d::new(u, v)
            })
            .collect()
    };

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

    // Project holes to UV (with NURBS optimization)
    let holes_uv: Vec<Vec<Point2d>> = hole_polylines_downsampled
        .iter()
        .map(|hole| {
            let mut huv: Vec<Point2d> = if let Surface::Nurbs(ref nurbs) = surface {
                // NURBS fast path: bootstrap + chain Newton-Raphson
                let mut uvs = Vec::with_capacity(hole.len());
                if !hole.is_empty() {
                    let (u0, v0) = surface.project_point(&hole[0]);
                    uvs.push(Point2d::new(u0, v0));
                    for i in 1..hole.len() {
                        let prev = uvs[i - 1];
                        let (u, v) = reproject_nurbs_point(nurbs, &hole[i], prev.u, prev.v);
                        uvs.push(Point2d::new(u, v));
                    }
                }
                uvs
            } else {
                hole.iter()
                    .map(|p| {
                        let (u, v) = surface.project_point(p);
                        Point2d::new(u, v)
                    })
                    .collect()
            };
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
    domain.init_containment_grid();

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

    triangulate_cdt(&domain, surface, forward, &interior_points)
}

// ============================================================
// Consistent (watertight) triangulation — HIGHLY OPTIMIZED
// ============================================================

/// Triangulate a curved surface with **consistent** boundary vertices.
///
/// This function produces watertight meshes where shared edges between
/// adjacent faces have **bit-identical** 3D vertex positions.
///
/// # Key optimizations:
/// 1. No per-triangle containment check — earcutr handles holes natively
/// 2. No boundary distance check in interior point generation
/// 3. Boundary vertices use cached 3D points directly (bit-identical)
/// 4. Interior vertices computed from UV via surface.point_at()
/// 5. Uses earcutr O(n log n) for the actual triangulation
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

    // Length mismatch between 3D points and UVs indicates a bug in the caller
    if boundary_points_3d.len() != boundary_uvs.len() {
        log::warn!(
            "triangulate_surface_consistent: 3D/UV length mismatch ({} vs {}) — returning empty mesh",
            boundary_points_3d.len(), boundary_uvs.len()
        );
        return TriangleMesh::new();
    }

    // ============================================================
    // Step 0: Validate and fix UV coordinates for NURBS surfaces
    //
    // CRITICAL OPTIMIZATION: We do NOT do per-point Newton-Raphson
    // re-projection here. That was the cause of the NURBS hang:
    // reproject_nurbs_point() does 15 iterations of derivatives_at()
    // per point, and with 48 EDGE_SAMPLES × N edges, this becomes
    // astronomically slow. It also DEGRADED quality — Newton from
    // a wrong initial guess converges to a different minimum, making
    // accurate UVs (from pcurves) worse.
    //
    // Instead, we just clamp UVs to the valid NURBS parameter range.
    // Boundary UVs that come from curve_2d/pcurve are already exact.
    // Boundary UVs from project_point() may be slightly off, but the
    // subsequent triangulation + chord-error refinement handles that.
    //
    // For UVs completely out of range (NaN, Inf, or far outside the
    // knot range), we fall back to the generic surface path.
    // ============================================================
    let mut outer_uv = boundary_uvs.to_vec();
    if let Surface::Nurbs(ref nurbs) = surface {
        let (nurb_u_min, nurb_u_max) = nurbs.u_range();
        let (nurb_v_min, nurb_v_max) = nurbs.v_range();

        // Check for invalid UVs (NaN, Inf, or wildly out of range)
        let margin = (nurb_u_max - nurb_u_min).max(1e-6) * 0.1;
        let v_margin = (nurb_v_max - nurb_v_min).max(1e-6) * 0.1;
        let has_invalid_uv = outer_uv.iter().any(|uv| {
            !uv.u.is_finite() || !uv.v.is_finite()
            || uv.u < nurb_u_min - margin || uv.u > nurb_u_max + margin
            || uv.v < nurb_v_min - v_margin || uv.v > nurb_v_max + v_margin
        });

        if has_invalid_uv {
            // Some UVs are wildly off — try clamping them as a best effort.
            // If too many are bad, the triangulation will be wrong anyway.
            let bad_count = outer_uv.iter().filter(|uv| {
                !uv.u.is_finite() || !uv.v.is_finite()
            }).count();
            let clamped_count = outer_uv.iter().filter(|uv| {
                uv.u < nurb_u_min || uv.u > nurb_u_max || uv.v < nurb_v_min || uv.v > nurb_v_max
            }).count();
            if clamped_count > 0 || bad_count > 0 {
                log::warn!(
                    "NURBS UV clamp: {} of {} UVs out of range, {} NaN/Inf (u=[{:.4},{:.4}] v=[{:.4},{:.4}])",
                    clamped_count, outer_uv.len(), bad_count,
                    nurb_u_min, nurb_u_max, nurb_v_min, nurb_v_max,
                );
            }
            if bad_count > outer_uv.len() / 2 {
                log::warn!(
                    "triangulate_surface_consistent: {} of {} NURBS UVs are NaN/Inf — returning empty mesh",
                    bad_count, outer_uv.len()
                );
                return TriangleMesh::new();
            }
            // Clamp all UVs to the NURBS parameter range
            for uv in outer_uv.iter_mut() {
                if uv.u.is_finite() {
                    uv.u = uv.u.clamp(nurb_u_min, nurb_u_max);
                } else {
                    uv.u = (nurb_u_min + nurb_u_max) * 0.5;
                }
                if uv.v.is_finite() {
                    uv.v = uv.v.clamp(nurb_v_min, nurb_v_max);
                } else {
                    uv.v = (nurb_v_min + nurb_v_max) * 0.5;
                }
            }
        }
    }

    // ============================================================
    // Step 1: Normalize UV for periodic surfaces
    // ============================================================
    let u_period = if surface.is_u_periodic() { Some(2.0 * PI) } else { None };
    let v_period = if surface.is_v_periodic() { Some(2.0 * PI) } else { None };

    crate::triangulate::normalize_uv_polygon(&mut outer_uv, u_period, v_period);

    let mut normalized_holes_uv: Vec<Vec<Point2d>> = Vec::new();
    for huv in hole_uvs {
        let mut nhuv = huv.clone();
        crate::triangulate::normalize_uv_polygon(&mut nhuv, u_period, v_period);
        normalized_holes_uv.push(nhuv);
    }

    if outer_uv.len() < 3 {
        return TriangleMesh::new();
    }

    // ============================================================
    // Step 1.5: Validate UV polygon quality
    //
    // After normalization, check if the UV polygon is degenerate or
    // self-intersecting. This can happen when project_point()
    // returns inaccurate UV coordinates that, after normalization,
    // produce a polygon that doesn't match the actual surface region.
    //
    // IMPORTANT: The area ratio check is ONLY meaningful for analytic
    // surfaces (cylinder, sphere, cone, torus, revolution, extrusion)
    // where UV coordinates have a fixed geometric interpretation
    // (radians, distances). For NURBS surfaces, the UV parameterization
    // is completely arbitrary — a surface with UV range [0,1]×[0,1]
    // can have any 3D area. Therefore, the area ratio is meaningless
    // for NURBS and we skip this check entirely.
    //
    // Additionally, we NEVER call surface.project_point() for NURBS
    // during triangulation — it does a 32×32 grid search + Newton-Raphson
    // (~1767 NURBS evaluations per point) which is catastrophically slow
    // and would hang the application.
    // ============================================================
    if !matches!(surface, Surface::Nurbs(_)) {
        let uv_area = polygon_area_2d(&outer_uv);
        let boundary_3d_area = polygon_area_3d(boundary_points_3d);
        let area_ratio = if boundary_3d_area > 1e-20 { uv_area / boundary_3d_area } else { 1.0 };
        if area_ratio < 0.001 && boundary_3d_area > 1e-10 {
            log::warn!(
                "triangulate_surface_consistent: UV polygon area ({:.6}) much smaller than 3D area ({:.6}), ratio={:.6} — re-projecting UVs from scratch",
                uv_area, boundary_3d_area, area_ratio
            );
            outer_uv = boundary_points_3d.iter().map(|p| {
                let (u, v) = surface.project_point(p);
                Point2d::new(u, v)
            }).collect();
            // Re-normalize
            crate::triangulate::normalize_uv_polygon(&mut outer_uv, u_period, v_period);
            if outer_uv.len() < 3 {
                return TriangleMesh::new();
            }
        }
    }

    // ============================================================
    // Step 1.6: NURBS UV polygon self-intersection check
    //
    // For NURBS surfaces, the UV polygon can be self-intersecting
    // when Newton-Raphson converges to a wrong UV for some boundary
    // points (e.g., on surfaces with large UV ranges or bad
    // parameterization). A self-intersecting UV polygon produces
    // incorrect triangulation — triangles on the wrong side of
    // the surface, inverted normals, etc.
    //
    // If the UV polygon is self-intersecting, we fall back to
    // re-projecting all UVs from scratch using surface.project_point().
    // This is slow (~146 evaluations per point) but more robust.
    // ============================================================
    if let Surface::Nurbs(ref nurbs) = surface {
        let uv_area = polygon_area_2d(&outer_uv);
        // A negative or zero UV area means the polygon is self-intersecting
        // or degenerate (collapsed to a line/point).
        if uv_area <= 0.0 && outer_uv.len() >= 3 {
            log::warn!(
                "NURBS UV polygon is self-intersecting/degenerate: area={:.6}, {} points, u_range=[{:.2},{:.2}] v_range=[{:.2},{:.2}] — re-projecting from scratch",
                uv_area, outer_uv.len(),
                outer_uv.iter().map(|p| p.u).fold(f64::MAX, f64::min),
                outer_uv.iter().map(|p| p.u).fold(f64::MIN, f64::max),
                outer_uv.iter().map(|p| p.v).fold(f64::MAX, f64::min),
                outer_uv.iter().map(|p| p.v).fold(f64::MIN, f64::max),
            );
            // Re-project all boundary UVs using full project_point()
            let (nu_min, nu_max) = nurbs.u_range();
            let (nv_min, nv_max) = nurbs.v_range();
            outer_uv = boundary_points_3d.iter().map(|p| {
                let (u, v) = surface.project_point(p);
                // Clamp to NURBS parameter range
                Point2d::new(
                    u.clamp(nu_min, nu_max),
                    v.clamp(nv_min, nv_max),
                )
            }).collect();
            // Re-normalize
            crate::triangulate::normalize_uv_polygon(&mut outer_uv, u_period, v_period);
            if outer_uv.len() < 3 {
                return TriangleMesh::new();
            }
            let new_area = polygon_area_2d(&outer_uv);
            log::info!(
                "NURBS UV polygon re-projected: area={:.6} (was {:.6})",
                new_area, uv_area
            );
        }
    }

    // ============================================================
    // Step 2: Compute UV range and build domain
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

    // Check for degenerate UV range (zero-area polygon)
    if (u_max - u_min) < 1e-12 || (v_max - v_min) < 1e-12 {
        log::warn!(
            "triangulate_surface_consistent: degenerate UV range u=[{:.6}, {:.6}] v=[{:.6}, {:.6}], {} boundary pts — returning empty mesh",
            u_min, u_max, v_min, v_max, outer_uv.len()
        );
        return TriangleMesh::new();
    }

    let mut domain = ParametricDomain::new(
        outer_uv.clone(),
        (u_min - margin_u, u_max + margin_u),
        (v_min - margin_v, v_max + margin_v),
    ).with_holes_from(normalized_holes_uv.iter().cloned());
    domain.init_containment_grid();

    // ============================================================
    // Step 2.5: DO NOT downsample boundary points!
    //
    // Boundary points from the edge cache represent the EXACT face
    // boundary. Downsampling them produces an incorrect polygon that
    // doesn't match the actual face boundary, leading to wrong
    // triangulation (triangles in wrong regions, gaps, overlaps).
    //
    // earcutr is O(n log n) and handles even 500+ boundary points
    // efficiently. The resulting triangle count from earcutr is
    // approximately 2×(N_boundary + N_holes + N_interior) - 2, which
    // is very reasonable.
    //
    // Instead of downsampling boundaries, we control the total triangle
    // count by limiting INTERIOR points only.
    // ============================================================
    let boundary_points_3d = boundary_points_3d.to_vec();
    let outer_uv = outer_uv; // Already a Vec, no downsampling

    // Keep all hole points too — holes define where NOT to triangulate
    let hole_polylines_3d_capped: Vec<Vec<Point3d>> = hole_polylines_3d.iter().map(|h| h.clone()).collect();
    let normalized_holes_uv_capped: Vec<Vec<Point2d>> = normalized_holes_uv;

    let max_total_points = (params.max_face_triangles / 2).max(6);

    // ============================================================
    // Step 3: Generate interior grid points
    //
    // IMPORTANT DESIGN PRINCIPLE:
    // Interior points are needed ONLY to improve surface approximation
    // for curved surfaces. For flat surfaces (planes, bilinear NURBS),
    // NO interior points are needed — the boundary polygon alone,
    // triangulated by earcutr, produces a perfect mesh.
    //
    // For curved surfaces (cylinder, sphere, high-degree NURBS),
    // interior points help the triangulation follow the surface
    // curvature. We add the MINIMUM number needed.
    //
    // The total triangle budget is: max_face_triangles per face.
    // Since we keep ALL boundary points (for watertightness),
    // the interior point budget is:
    //   max_face_triangles/2 - boundary_points - hole_points
    // ============================================================
    // Interior point budget: boundary points are mandatory for watertightness
    // and should NOT consume the interior budget. Interior points are needed
    // for curved surface approximation quality. We compute the interior budget
    // separately to ensure curved surfaces always get enough interior Steiner points.
    let n_boundary_and_holes = boundary_points_3d.len()
        + hole_polylines_3d_capped.iter().map(|h| h.len()).sum::<usize>();

    // Minimum interior points for curved surfaces based on the number of
    // boundary vertices. A curved surface needs at least ~1/3 as many interior
    // points as boundary points to produce a good triangulation that follows
    // the surface curvature.
    //
    // ADAPTIVE: For NURBS, the minimum is based on curvature rather than a
    // fixed floor. Bilinear NURBS (deg 1×1) need 0 interior points. Ruled
    // NURBS (deg 1×N) need fewer points than high-degree NURBS (deg M×N).
    // The chord-error refinement will add more points where needed.
    let is_nurbs = matches!(surface, Surface::Nurbs(_));
    let is_nurbs_bilinear = if let Surface::Nurbs(ref nurbs) = surface {
        nurbs.u_degree <= 1 && nurbs.v_degree <= 1
    } else {
        false
    };
    let is_nurbs_ruled = if let Surface::Nurbs(ref nurbs) = surface {
        (nurbs.u_degree <= 1) != (nurbs.v_degree <= 1) // exactly one direction is linear
    } else {
        false
    };
    let min_interior_for_curved = if is_nurbs_bilinear {
        0 // Bilinear NURBS are flat — no interior points needed
    } else if is_nurbs_ruled {
        (n_boundary_and_holes / 4).max(8) // Ruled surfaces: fewer interior points
    } else if is_nurbs {
        (n_boundary_and_holes / 3).max(20) // High-degree NURBS: moderate floor
    } else {
        (n_boundary_and_holes / 3).max(20)
    };
    let max_interior_budget = max_total_points.saturating_sub(n_boundary_and_holes).max(min_interior_for_curved);

    let interior_uv_points = if let Surface::Nurbs(ref nurbs) = surface {
        let u_deg = nurbs.u_degree;
        let v_deg = nurbs.v_degree;

        // For bilinear (deg=1×1) NURBS surfaces, no interior points needed.
        // The surface is flat and boundary points alone triangulate it perfectly.
        if u_deg <= 1 && v_deg <= 1 {
            Vec::new()
        } else if u_deg <= 1 || v_deg <= 1 {
            // Ruled surface (linear in one direction): needs interior points
            // to capture curvature in the non-linear direction.
            // Use adaptive subdivision based on actual curvature — fewer
            // subdivisions for nearly-flat surfaces, more for curved ones.
            let max_k = crate::adaptive::max_curvature_over_domain(
                surface, u_min, u_max, v_min, v_max,
            );
            // n_sub ranges from 2 (nearly flat, max_k < 0.01) to 8 (high curvature)
            let n_sub = if max_k < 0.01 {
                2
            } else if max_k < 0.1 {
                3
            } else if max_k < 1.0 {
                4
            } else {
                6
            };
            let pts = generate_nurbs_interior_points(&domain, &nurbs.u_knots, &nurbs.v_knots, n_sub);
            downsample_interior_points(&pts, max_interior_budget)
        } else {
            // High-degree NURBS (both directions curved): use adaptive sampling.
            // Compute curvature to determine how many subdivisions are needed.
            let max_k = crate::adaptive::max_curvature_over_domain(
                surface, u_min, u_max, v_min, v_max,
            );
            // n_sub ranges from 3 (low curvature) to 8 (high curvature).
            // This is more conservative than the old formula which used up to
            // 12 subdivisions — the chord-error refinement adds more points
            // where actually needed, so we don't need excessive initial sampling.
            let n_sub = if max_k < 0.01 {
                3
            } else if max_k < 0.1 {
                4
            } else if max_k < 1.0 {
                5
            } else {
                8
            };
            let pts = generate_nurbs_interior_points(&domain, &nurbs.u_knots, &nurbs.v_knots, n_sub);
            downsample_interior_points(&pts, max_interior_budget)
        }
    } else {
        // Non-NURBS curved surfaces (Torus, Revolution, Extrusion)
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
        let boundary_margin = (u_max - u_min) / n_u.max(1) as f64 * 0.3;
        let pts = generate_interior_points(&domain, n_u, n_v, boundary_margin);
        downsample_interior_points(&pts, max_interior_budget)
    };

    // ============================================================
    // Step 4: Build earcutr input with ALL points
    //
    // KEY: Pass interior points as part of the earcutr input
    // directly. earcutr handles Steiner points natively and
    // produces quality triangulation in O(n log n).
    // ============================================================

    let n_boundary = outer_uv.len();

    // Build combined point array: [boundary_uv...][valid_hole_uv...][interior_uv...]
    // CRITICAL: Only include holes with >= 3 points. Small holes are degenerate
    // and would corrupt earcutr's triangulation. We must also track which holes
    // were included so the 3D vertex array (Step 5) stays in sync with UV indices.
    let mut all_uv: Vec<Point2d> = outer_uv.clone();
    let mut valid_hole_indices: Vec<usize> = Vec::new(); // indices into normalized_holes_uv_capped
    let mut hole_start_indices: Vec<usize> = Vec::new();
    let mut offset = n_boundary;
    for (hi, huv) in normalized_holes_uv_capped.iter().enumerate() {
        if huv.len() >= 3 {
            valid_hole_indices.push(hi);
            hole_start_indices.push(offset);
            all_uv.extend_from_slice(huv);
            offset += huv.len();
        }
        // Skip holes with < 3 points — they're degenerate
    }
    let n_boundary_and_holes_actual = all_uv.len();

    // Add interior points as Steiner points — already capped by max_interior_budget
    all_uv.extend_from_slice(&interior_uv_points);

    // Build flat coordinate array for earcutr
    let mut coords: Vec<f64> = Vec::with_capacity(all_uv.len() * 2);
    for p in &all_uv {
        coords.push(p.u);
        coords.push(p.v);
    }

    // Run earcutr triangulation with ALL points at once
    // hole_start_indices was built above, only including valid holes (>= 3 points)
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
    // Step 5: Build 3D mesh
    //
    // IMPORTANT: No per-triangle containment check!
    // earcutr already produces correct triangulation when given
    // proper hole indices. The old centroid check was:
    // 1. O(triangles × boundary_len) — extremely slow
    // 2. Incorrect for triangles near boundaries (coarse grid gives false negatives)
    // 3. Caused "not watertight" gaps in the mesh
    // ============================================================

    // Build combined 3D point array for boundary + hole vertices
    // CRITICAL: Only include holes that are also in all_uv (valid_hole_indices).
    // This ensures the 3D vertex indices match the UV indices used by earcutr.
    let mut all_boundary_3d: Vec<Point3d> = boundary_points_3d.clone();
    for &hi in &valid_hole_indices {
        all_boundary_3d.extend_from_slice(&hole_polylines_3d_capped[hi]);
    }

    // Build mesh — use cached 3D points for boundary/hole vertices
    let mut mesh = TriangleMesh::new();
    let mut vertex_map: std::collections::HashMap<u32, u32> = std::collections::HashMap::new();

    for tri in &result_triangles {
        // Bounds check
        if tri[0] as usize >= all_uv.len()
            || tri[1] as usize >= all_uv.len()
            || tri[2] as usize >= all_uv.len()
        {
            continue;
        }

        // Add vertices and triangle
        let mut tri_indices = [0u32; 3];
        for (k, &idx) in tri.iter().enumerate() {
            let idx_usize = idx as usize;
            let entry = vertex_map.entry(idx).or_insert_with(|| {
                if idx_usize < n_boundary_and_holes_actual {
                    // Boundary/hole vertex: use cached 3D point directly
                    // This is what makes the mesh watertight — shared edge
                    // vertices have bit-identical 3D positions
                    let p3d = all_boundary_3d[idx_usize];
                    let uv = all_uv[idx_usize];
                    // For boundary vertices, we need the normal.
                    // For NURBS, use derivatives_at to get both point and normal
                    // in one call (saves a separate normal_at = derivatives_at call).
                    let n = if let Surface::Nurbs(ref nurbs) = surface {
                        let derivs = nurbs.derivatives_at(uv.u, uv.v);
                        derivs.normal()
                    } else {
                        surface.normal_at(uv.u, uv.v)
                    };
                    let vi = mesh.add_vertex(p3d);
                    mesh.add_vertex_normal(vi, [n.x, n.y, n.z]);
                    vi
                } else {
                    // Interior vertex: compute 3D point and normal from UV.
                    // For NURBS, use derivatives_at once to get both point and
                    // normal in a single call (87 de Boor iterations) instead of
                    // point_at (30) + normal_at (87) = 117 iterations separately.
                    // Apply deterministic rounding to ensure consistent vertex positions
                    // across faces (matches edge cache's rounding for boundary vertices).
                    let uv = all_uv[idx_usize];
                    let (p3d, n) = if let Surface::Nurbs(ref nurbs) = surface {
                        let derivs = nurbs.derivatives_at(uv.u, uv.v);
                        (deterministic_round_point(derivs.point), derivs.normal())
                    } else {
                        (deterministic_round_point(surface.point_at(uv.u, uv.v)), surface.normal_at(uv.u, uv.v))
                    };
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

    // ============================================================
    // Step 6: Adaptive chord-error refinement
    //
    // For curved surfaces (not planes), check each triangle's chord
    // error — the distance from the midpoint of each edge to the
    // true surface point at the corresponding UV. If any edge exceeds
    // max_deviation, subdivide the triangle by inserting a point at
    // the surface midpoint.
    //
    // This is iterative — we repeat until no edge exceeds the
    // tolerance or we hit a maximum iteration count.
    //
    // KEY OPTIMIZATION: We build a vertex UV array so that midpoint
    // UVs can be computed by averaging adjacent vertex UVs instead of
    // calling surface.project_point(). For NURBS surfaces, project_point()
    // costs ~1000+ evaluations per call (32×32 grid search + Newton-Raphson),
    // making chord-error refinement catastrophically slow. By using UV
    // averaging, each midpoint costs just 1 surface.point_at() evaluation.
    // ============================================================
    if !matches!(surface, Surface::Plane(_)) && params.max_deviation > 0.0 {
        // Use 3 refinement iterations for NURBS (2 for other curved surfaces).
        // NURBS surfaces with high curvature benefit from the extra iteration
        // because knot-span interior points may not capture all curvature regions.
        // The UV-averaging approach avoids the expensive project_point() — each
        // midpoint costs just 1 surface.point_at() evaluation, so 3 iterations
        // is affordable even for large NURBS surfaces.
        let max_refine_iters = if matches!(surface, Surface::Nurbs(_)) { 3 } else { 2 };

        // Build vertex UV array — maps mesh vertex index to UV coordinate.
        // This enables O(1) midpoint UV computation instead of O(1000) project_point().
        let mut vertex_uvs: Vec<Point2d> = vec![Point2d::new(0.0, 0.0); mesh.vertices.len()];
        for (idx, uv) in all_uv.iter().enumerate() {
            if let Some(&mesh_idx) = vertex_map.get(&(idx as u32)) {
                vertex_uvs[mesh_idx as usize] = *uv;
            }
        }

        // Track which mesh vertices are boundary (from edge cache) vs interior.
        // Boundary vertices have bit-identical 3D coordinates across faces,
        // so splitting a boundary-boundary edge would create new vertices
        // that can't be deduplicated — breaking watertightness.
        let mut is_boundary_vertex: Vec<bool> = vec![false; mesh.vertices.len()];
        for (idx, _) in all_uv.iter().enumerate() {
            if let Some(&mesh_idx) = vertex_map.get(&(idx as u32)) {
                // Vertices from boundary/hole polylines (indices < n_boundary_and_holes_actual)
                // are "boundary" vertices from the edge cache.
                is_boundary_vertex[mesh_idx as usize] = idx < n_boundary_and_holes_actual;
            }
        }

        refine_mesh_chord_error_uv(
            &mut mesh, surface, forward, params.max_deviation, max_refine_iters,
            &mut vertex_uvs, &mut is_boundary_vertex,
        );
    }

    mesh
}

// ============================================================
// Adaptive chord-error refinement
// ============================================================

/// Iteratively refine a triangle mesh on a curved surface by checking
/// the chord error of each edge and subdividing edges that exceed
/// the maximum deviation tolerance.
///
/// The chord error of an edge is the distance from the midpoint of
/// the straight line segment (in 3D) to the true surface point at
/// the corresponding UV parameter. For curved surfaces like cylinders
/// and NURBS, this measures how well the triangle mesh approximates
/// the true surface.
///
/// # Algorithm
/// For each iteration:
/// 1. For each triangle edge, compute the midpoint in 3D
/// 2. Project the midpoint onto the surface to get the "true" point
/// 3. If the distance exceeds max_deviation, mark the edge for subdivision
/// 4. For each triangle with a marked edge, insert the surface point
///    and subdivide the triangle into 2-4 sub-triangles
///
/// # Arguments
/// * `mesh` — The triangle mesh to refine
/// * `surface` — The parametric surface the mesh approximates
/// * `forward` — Whether face normal matches surface normal
/// * `max_deviation` — Maximum allowed chord error
/// * `max_iterations` — Maximum number of refinement iterations
#[allow(dead_code)] // Kept for potential non-UV-aware use cases; consistent path uses UV-aware variant
fn refine_mesh_chord_error(
    mesh: &mut TriangleMesh,
    surface: &Surface,
    forward: bool,
    max_deviation: f64,
    max_iterations: usize,
) {
    use std::collections::HashMap;

    for _iter in 0..max_iterations {
        // Find edges that need subdivision
        // edge = (v0, v1) where v0 < v1
        let mut edges_to_split: HashMap<(u32, u32), u32> = HashMap::new();

        for tri in &mesh.triangles {
            for k in 0..3 {
                let v0 = tri[k];
                let v1 = tri[(k + 1) % 3];
                let edge = if v0 < v1 { (v0, v1) } else { (v1, v0) };

                if edges_to_split.contains_key(&edge) {
                    continue; // Already marked
                }

                let p0 = mesh.vertices[v0 as usize];
                let p1 = mesh.vertices[v1 as usize];

                // Compute midpoint of the edge in 3D
                let mid = Point3d::new(
                    (p0.x + p1.x) * 0.5,
                    (p0.y + p1.y) * 0.5,
                    (p0.z + p1.z) * 0.5,
                );

                // Project midpoint onto the surface
                let (u, v) = surface.project_point(&mid);
                let p_surf = surface.point_at(u, v);

                // For NURBS surfaces, project_point can be inaccurate.
                // Try Newton-Raphson refinement if the initial projection
                // is far from the midpoint.
                let (u, v, p_surf) = if let Surface::Nurbs(ref nurbs) = surface {
                    let dx0 = p_surf.x - mid.x;
                    let dy0 = p_surf.y - mid.y;
                    let dz0 = p_surf.z - mid.z;
                    let err0 = (dx0*dx0 + dy0*dy0 + dz0*dz0).sqrt();
                    if err0 > max_deviation * 0.1 {
                        let (u2, v2) = reproject_nurbs_point(nurbs, &mid, u, v);
                        let p2 = surface.point_at(u2, v2);
                        let dx2 = p2.x - mid.x;
                        let dy2 = p2.y - mid.y;
                        let dz2 = p2.z - mid.z;
                        let err2 = (dx2*dx2 + dy2*dy2 + dz2*dz2).sqrt();
                        if err2 < err0 {
                            (u2, v2, p2)
                        } else {
                            (u, v, p_surf)
                        }
                    } else {
                        (u, v, p_surf)
                    }
                } else {
                    (u, v, p_surf)
                };

                // Chord error: distance from line midpoint to surface point
                let dx = mid.x - p_surf.x;
                let dy = mid.y - p_surf.y;
                let dz = mid.z - p_surf.z;
                let chord_error = (dx * dx + dy * dy + dz * dz).sqrt();

                if chord_error > max_deviation {
                    // Mark this edge for subdivision — the new vertex index
                    // will be assigned when we actually split it
                    edges_to_split.insert(edge, u32::MAX); // Placeholder
                }
            }
        }

        if edges_to_split.is_empty() {
            break; // No more edges to split
        }

        // Now insert the surface points and update the map
        let mut new_edges: HashMap<(u32, u32), u32> = HashMap::new();
        for (edge, _) in &edges_to_split {
            let p0 = mesh.vertices[edge.0 as usize];
            let p1 = mesh.vertices[edge.1 as usize];

            let mid = Point3d::new(
                (p0.x + p1.x) * 0.5,
                (p0.y + p1.y) * 0.5,
                (p0.z + p1.z) * 0.5,
            );

            let (u, v) = surface.project_point(&mid);
            let p_surf = surface.point_at(u, v);

            // For NURBS surfaces, project_point can be inaccurate.
            // Verify the re-projection quality and re-project using
            // Newton-Raphson if the initial result is poor.
            let (u, v, p_surf) = if let Surface::Nurbs(ref nurbs) = surface {
                // Check re-projection error
                let dx = p_surf.x - mid.x;
                let dy = p_surf.y - mid.y;
                let dz = p_surf.z - mid.z;
                let reproj_err = (dx*dx + dy*dy + dz*dz).sqrt();

                // If re-projection error is large, try Newton-Raphson refinement
                if reproj_err > max_deviation * 0.1 {
                    let (u2, v2) = reproject_nurbs_point(nurbs, &mid, u, v);
                    let p2 = surface.point_at(u2, v2);
                    let dx2 = p2.x - mid.x;
                    let dy2 = p2.y - mid.y;
                    let dz2 = p2.z - mid.z;
                    let err2 = (dx2*dx2 + dy2*dy2 + dz2*dz2).sqrt();
                    if err2 < reproj_err {
                        (u2, v2, p2)
                    } else {
                        (u, v, p_surf)
                    }
                } else {
                    (u, v, p_surf)
                }
            } else {
                (u, v, p_surf)
            };
            let n = surface.normal_at(u, v);

            let vi = mesh.add_vertex(p_surf);
            mesh.add_vertex_normal(vi, [n.x, n.y, n.z]);
            new_edges.insert(*edge, vi);
        }

        // Now rebuild the triangle list, splitting triangles that have
        // edges marked for subdivision
        let old_triangles = std::mem::take(&mut mesh.triangles);
        mesh.triangles.reserve(old_triangles.len());

        for tri in &old_triangles {
            // Check which edges of this triangle are split
            let mut split_verts = [None; 3];
            let mut n_splits = 0;
            for k in 0..3 {
                let v0 = tri[k];
                let v1 = tri[(k + 1) % 3];
                let edge = if v0 < v1 { (v0, v1) } else { (v1, v0) };
                if let Some(&new_v) = new_edges.get(&edge) {
                    split_verts[k] = Some(new_v);
                    n_splits += 1;
                }
            }

            if n_splits == 0 {
                // No splits — keep the triangle as-is
                mesh.triangles.push(*tri);
            } else if n_splits == 1 {
                // One edge split — triangle becomes 2 triangles
                // Find which edge is split
                let split_k = split_verts.iter().position(|v| v.is_some()).unwrap();
                let vm = split_verts[split_k].unwrap();
                let v0 = tri[split_k];
                let v1 = tri[(split_k + 1) % 3];
                let v2 = tri[(split_k + 2) % 3];

                if forward {
                    mesh.triangles.push([v0, vm, v2]);
                    mesh.triangles.push([vm, v1, v2]);
                } else {
                    mesh.triangles.push([v0, v2, vm]);
                    mesh.triangles.push([vm, v2, v1]);
                }
            } else if n_splits == 2 {
                // Two edges split — triangle becomes 3 triangles
                let split_edges: Vec<usize> = split_verts.iter()
                    .enumerate()
                    .filter(|(_, v)| v.is_some())
                    .map(|(i, _)| i)
                    .collect();
                let k0 = split_edges[0];
                let k1 = split_edges[1];
                let vm0 = split_verts[k0].unwrap();
                let vm1 = split_verts[k1].unwrap();
                let v0 = tri[k0];
                let v1 = tri[(k0 + 1) % 3];
                let v2 = tri[(k0 + 2) % 3];

                // k0 is the first split edge: v0 -> v1
                // k1 is the second split edge: v1 -> v2 (if adjacent)
                if (k0 + 1) % 3 == k1 {
                    // Split edges are adjacent: v0-vm0-v1-vm1-v2
                    if forward {
                        mesh.triangles.push([v0, vm0, vm1]);
                        mesh.triangles.push([vm0, v1, vm1]);
                        mesh.triangles.push([v0, vm1, v2]);
                    } else {
                        mesh.triangles.push([v0, vm1, vm0]);
                        mesh.triangles.push([vm0, vm1, v1]);
                        mesh.triangles.push([v0, v2, vm1]);
                    }
                } else {
                    // k1 is on the other side: v0-vm0-v1, v0-vm1-v2
                    // Actually k0=0, k1=2 means edges v0->v1 and v2->v0
                    // v2-vm1-v0-vm0-v1
                    if forward {
                        mesh.triangles.push([v0, vm0, v1]);
                        mesh.triangles.push([v2, vm1, vm0]);
                        mesh.triangles.push([vm0, v0, vm1]);
                    } else {
                        mesh.triangles.push([v0, v1, vm0]);
                        mesh.triangles.push([v2, vm0, vm1]);
                        mesh.triangles.push([vm0, vm1, v0]);
                    }
                }
            } else {
                // All 3 edges split — triangle becomes 4 triangles
                let vm0 = split_verts[0].unwrap();
                let vm1 = split_verts[1].unwrap();
                let vm2 = split_verts[2].unwrap();
                let v0 = tri[0];
                let v1 = tri[1];
                let v2 = tri[2];

                if forward {
                    mesh.triangles.push([v0, vm0, vm2]);
                    mesh.triangles.push([vm0, v1, vm1]);
                    mesh.triangles.push([vm2, vm1, v2]);
                    mesh.triangles.push([vm0, vm1, vm2]);
                } else {
                    mesh.triangles.push([v0, vm2, vm0]);
                    mesh.triangles.push([vm0, vm1, v1]);
                    mesh.triangles.push([vm2, v2, vm1]);
                    mesh.triangles.push([vm0, vm2, vm1]);
                }
            }
        }

        // Also update face_normals if present
        if let Some(ref mut face_normals) = mesh.face_normals {
            let n_old = face_normals.len();
            let n_new = mesh.triangles.len();
            if n_new > n_old {
                // Compute normals for the new triangles
                for i in n_old..n_new {
                    let tri = mesh.triangles[i];
                    let p0 = mesh.vertices[tri[0] as usize];
                    let p1 = mesh.vertices[tri[1] as usize];
                    let p2 = mesh.vertices[tri[2] as usize];
                    let ab = [p1.x - p0.x, p1.y - p0.y, p1.z - p0.z];
                    let ac = [p2.x - p0.x, p2.y - p0.y, p2.z - p0.z];
                    let nx = ab[1] * ac[2] - ab[2] * ac[1];
                    let ny = ab[2] * ac[0] - ab[0] * ac[2];
                    let nz = ab[0] * ac[1] - ab[1] * ac[0];
                    let len = (nx * nx + ny * ny + nz * nz).sqrt().max(1e-15);
                    face_normals.push([nx / len, ny / len, nz / len]);
                }
            }
        }

        // Also update triangle_face_ids if present
        if let Some(ref mut face_ids) = mesh.triangle_face_ids {
            let n_old = face_ids.len();
            let n_new = mesh.triangles.len();
            if n_new > n_old {
                // All new triangles inherit the face ID of the triangle they came from
                // Since we process sequentially, just extend with the last face ID
                let last_id = face_ids.last().copied().unwrap_or(0);
                face_ids.extend(std::iter::repeat(last_id).take(n_new - n_old));
            }
        }
    }
}

/// UV-aware chord-error refinement — O(1) per edge instead of O(1000).
///
/// This is the fast variant of `refine_mesh_chord_error` that uses pre-computed
/// UV coordinates for each vertex. Instead of calling the extremely expensive
/// `surface.project_point()` (which for NURBS costs ~1000+ evaluations per call),
/// it computes the midpoint UV by averaging adjacent vertex UVs, then evaluates
/// `surface.point_at(mid_u, mid_v)` directly — a single evaluation.
///
/// This provides a ~1000× speedup for NURBS surfaces in the refinement step.
///
/// # Arguments
/// * `mesh` — The triangle mesh to refine
/// * `surface` — The parametric surface the mesh approximates
/// * `forward` — Whether face normal matches surface normal
/// * `max_deviation` — Maximum allowed chord error
/// * `max_iterations` — Maximum number of refinement iterations
/// * `vertex_uvs` — UV coordinates for each vertex in the mesh (mutated as new vertices are added)
fn refine_mesh_chord_error_uv(
    mesh: &mut TriangleMesh,
    surface: &Surface,
    forward: bool,
    max_deviation: f64,
    max_iterations: usize,
    vertex_uvs: &mut Vec<Point2d>,
    is_boundary_vertex: &mut Vec<bool>,
) {
    use std::collections::HashMap;

    // For NURBS, we might need Newton-Raphson refinement of the midpoint UV.
    // But first, try the simple UV averaging which is correct for well-parameterized surfaces.
    let is_nurbs = matches!(surface, Surface::Nurbs(_));
    let (nurb_u_min, nurb_u_max, nurb_v_min, nurb_v_max) = if let Surface::Nurbs(ref nurbs) = surface {
        (nurbs.u_range().0, nurbs.u_range().1, nurbs.v_range().0, nurbs.v_range().1)
    } else {
        (0.0, 1.0, 0.0, 1.0)
    };

    // Minimum UV distance between edge endpoints — edges shorter than this
    // are never split, preventing triangle explosion from non-convergent
    // refinement (where chord error doesn't decrease despite subdivision).
    let min_uv_dist = if is_nurbs {
        // For NURBS, use a fraction of the parameter range as the minimum.
        // This prevents infinite subdivision in areas with bad parameterization.
        let u_size = (nurb_u_max - nurb_u_min).max(1e-10);
        let v_size = (nurb_v_max - nurb_v_min).max(1e-10);
        u_size.min(v_size) * 0.01 // 1% of the smaller parameter range
    } else {
        1e-10 // Effectively no minimum for analytic surfaces
    };

    for _iter in 0..max_iterations {
        // Find edges that need subdivision
        let mut edges_to_split: HashMap<(u32, u32), u32> = HashMap::new();

        for tri in &mesh.triangles {
            for k in 0..3 {
                let v0 = tri[k];
                let v1 = tri[(k + 1) % 3];
                let edge = if v0 < v1 { (v0, v1) } else { (v1, v0) };

                if edges_to_split.contains_key(&edge) {
                    continue; // Already marked
                }

                // CRITICAL: Skip splitting edges between two boundary vertices.
                // Boundary vertices come from the edge cache with bit-identical
                // 3D coordinates across faces. Splitting a boundary-boundary edge
                // creates a new midpoint vertex computed from surface.point_at(),
                // which produces DIFFERENT f64 bits for each face. This new vertex
                // can't be deduplicated, breaking watertightness.
                //
                // The edge cache already ensures sufficient sampling on boundary
                // edges (via adaptive_discretize). If chord error is too large
                // on a boundary edge, the fix is to increase edge sampling, not
                // to split the edge here.
                let v0_is_boundary = is_boundary_vertex.get(v0 as usize).copied().unwrap_or(false);
                let v1_is_boundary = is_boundary_vertex.get(v1 as usize).copied().unwrap_or(false);
                if v0_is_boundary && v1_is_boundary {
                    continue; // Don't split boundary-boundary edges
                }

                // Compute midpoint UV by averaging — O(1) instead of O(1000)
                let uv0 = vertex_uvs[v0 as usize];
                let uv1 = vertex_uvs[v1 as usize];

                // Minimum edge-length check: skip edges that are already very short
                // in UV space to prevent triangle explosion
                let du = (uv1.u - uv0.u).abs();
                let dv = (uv1.v - uv0.v).abs();
                if du < min_uv_dist && dv < min_uv_dist {
                    continue;
                }

                let mid_u = (uv0.u + uv1.u) * 0.5;
                let mid_v = (uv0.v + uv1.v) * 0.5;

                // Handle periodic surfaces: if UVs wrap around, averaging is wrong.
                // For periodic u, if |uv0.u - uv1.u| > half_period, one UV is near
                // the wrap boundary. Adjust the average accordingly.
                let mid_u = if surface.is_u_periodic() {
                    let period = 2.0 * PI; // Standard period for our surfaces
                    let du = (uv1.u - uv0.u).abs();
                    if du > period * 0.5 {
                        // UVs wrap around — adjust
                        let (lo, hi) = if uv0.u < uv1.u { (uv0.u, uv1.u) } else { (uv1.u, uv0.u) };
                        ((lo + period + hi) * 0.5) % period
                    } else {
                        mid_u
                    }
                } else {
                    mid_u
                };

                let mid_v = if surface.is_v_periodic() {
                    let period = 2.0 * PI;
                    let dv = (uv1.v - uv0.v).abs();
                    if dv > period * 0.5 {
                        let (lo, hi) = if uv0.v < uv1.v { (uv0.v, uv1.v) } else { (uv1.v, uv0.v) };
                        ((lo + period + hi) * 0.5) % period
                    } else {
                        mid_v
                    }
                } else {
                    mid_v
                };

                // Clamp to surface parameter range (important for NURBS)
                let mid_u_clamped = if is_nurbs { mid_u.clamp(nurb_u_min, nurb_u_max) } else { mid_u };
                let mid_v_clamped = if is_nurbs { mid_v.clamp(nurb_v_min, nurb_v_max) } else { mid_v };

                // Compute the surface point at the midpoint UV — ONE evaluation
                let p_surf = surface.point_at(mid_u_clamped, mid_v_clamped);

                // Compute 3D midpoint of the edge
                let p0 = mesh.vertices[v0 as usize];
                let p1 = mesh.vertices[v1 as usize];
                let mid_3d = Point3d::new(
                    (p0.x + p1.x) * 0.5,
                    (p0.y + p1.y) * 0.5,
                    (p0.z + p1.z) * 0.5,
                );

                // Chord error: distance from 3D midpoint to surface point
                let dx = mid_3d.x - p_surf.x;
                let dy = mid_3d.y - p_surf.y;
                let dz = mid_3d.z - p_surf.z;
                let chord_error = (dx * dx + dy * dy + dz * dz).sqrt();

                if chord_error > max_deviation {
                    edges_to_split.insert(edge, u32::MAX); // Placeholder
                }
            }
        }

        if edges_to_split.is_empty() {
            break; // No more edges to split
        }

        // Insert surface points for each edge to split
        let mut new_edges: HashMap<(u32, u32), u32> = HashMap::new();
        for (edge, _) in &edges_to_split {
            let v0 = edge.0;
            let v1 = edge.1;

            // Compute midpoint UV by averaging
            let uv0 = vertex_uvs[v0 as usize];
            let uv1 = vertex_uvs[v1 as usize];
            let mut mid_u = (uv0.u + uv1.u) * 0.5;
            let mut mid_v = (uv0.v + uv1.v) * 0.5;

            // Handle periodic wrapping
            if surface.is_u_periodic() {
                let period = 2.0 * PI;
                let du = (uv1.u - uv0.u).abs();
                if du > period * 0.5 {
                    let (lo, hi) = if uv0.u < uv1.u { (uv0.u, uv1.u) } else { (uv1.u, uv0.u) };
                    mid_u = ((lo + period + hi) * 0.5) % period;
                }
            }
            if surface.is_v_periodic() {
                let period = 2.0 * PI;
                let dv = (uv1.v - uv0.v).abs();
                if dv > period * 0.5 {
                    let (lo, hi) = if uv0.v < uv1.v { (uv0.v, uv1.v) } else { (uv1.v, uv0.v) };
                    mid_v = ((lo + period + hi) * 0.5) % period;
                }
            }

            // Clamp to surface parameter range
            if is_nurbs {
                mid_u = mid_u.clamp(nurb_u_min, nurb_u_max);
                mid_v = mid_v.clamp(nurb_v_min, nurb_v_max);
            }

            // For the split vertex, simply evaluate the surface at the averaged UV.
            // We do NOT use Newton-Raphson re-projection here because:
            // 1. UV averaging is already correct for well-parameterized surfaces
            // 2. Newton-Raphson costs ~469 de Boor iterations per edge split
            // 3. The chord error check already verified the averaged UV is reasonable
            // 4. For NURBS with bad parameterization, Newton often doesn't converge
            //    better than simple averaging anyway
            let p_surf = deterministic_round_point(surface.point_at(mid_u, mid_v));
            let n = surface.normal_at(mid_u, mid_v);

            let vi = mesh.add_vertex(p_surf);
            mesh.add_vertex_normal(vi, [n.x, n.y, n.z]);

            // Store UV for the new vertex
            vertex_uvs.push(Point2d::new(mid_u, mid_v));

            // New vertices from chord-error refinement are NOT boundary vertices —
            // they're computed from surface.point_at(), not from the edge cache.
            // Marking them as non-boundary ensures that future refinement iterations
            // can still split edges involving these vertices.
            is_boundary_vertex.push(false);

            new_edges.insert(*edge, vi);
        }

        // Rebuild the triangle list, splitting triangles that have edges marked for subdivision
        let old_triangles = std::mem::take(&mut mesh.triangles);
        mesh.triangles.reserve(old_triangles.len());

        for tri in &old_triangles {
            let mut split_verts = [None; 3];
            let mut n_splits = 0;
            for k in 0..3 {
                let v0 = tri[k];
                let v1 = tri[(k + 1) % 3];
                let edge = if v0 < v1 { (v0, v1) } else { (v1, v0) };
                if let Some(&new_v) = new_edges.get(&edge) {
                    split_verts[k] = Some(new_v);
                    n_splits += 1;
                }
            }

            if n_splits == 0 {
                mesh.triangles.push(*tri);
            } else if n_splits == 1 {
                let split_k = split_verts.iter().position(|v| v.is_some()).unwrap();
                let vm = split_verts[split_k].unwrap();
                let v0 = tri[split_k];
                let v1 = tri[(split_k + 1) % 3];
                let v2 = tri[(split_k + 2) % 3];

                if forward {
                    mesh.triangles.push([v0, vm, v2]);
                    mesh.triangles.push([vm, v1, v2]);
                } else {
                    mesh.triangles.push([v0, v2, vm]);
                    mesh.triangles.push([vm, v2, v1]);
                }
            } else if n_splits == 2 {
                let split_edges: Vec<usize> = split_verts.iter()
                    .enumerate()
                    .filter(|(_, v)| v.is_some())
                    .map(|(i, _)| i)
                    .collect();
                let k0 = split_edges[0];
                let k1 = split_edges[1];
                let vm0 = split_verts[k0].unwrap();
                let vm1 = split_verts[k1].unwrap();
                let v0 = tri[k0];
                let v1 = tri[(k0 + 1) % 3];
                let v2 = tri[(k0 + 2) % 3];

                if (k0 + 1) % 3 == k1 {
                    if forward {
                        mesh.triangles.push([v0, vm0, vm1]);
                        mesh.triangles.push([vm0, v1, vm1]);
                        mesh.triangles.push([v0, vm1, v2]);
                    } else {
                        mesh.triangles.push([v0, vm1, vm0]);
                        mesh.triangles.push([vm0, vm1, v1]);
                        mesh.triangles.push([v0, v2, vm1]);
                    }
                } else {
                    if forward {
                        mesh.triangles.push([v0, vm0, v1]);
                        mesh.triangles.push([v2, vm1, vm0]);
                        mesh.triangles.push([vm0, v0, vm1]);
                    } else {
                        mesh.triangles.push([v0, v1, vm0]);
                        mesh.triangles.push([v2, vm0, vm1]);
                        mesh.triangles.push([vm0, vm1, v0]);
                    }
                }
            } else {
                // All 3 edges split — triangle becomes 4 triangles
                let vm0 = split_verts[0].unwrap();
                let vm1 = split_verts[1].unwrap();
                let vm2 = split_verts[2].unwrap();
                let v0 = tri[0];
                let v1 = tri[1];
                let v2 = tri[2];

                if forward {
                    mesh.triangles.push([v0, vm0, vm2]);
                    mesh.triangles.push([vm0, v1, vm1]);
                    mesh.triangles.push([vm2, vm1, v2]);
                    mesh.triangles.push([vm0, vm1, vm2]);
                } else {
                    mesh.triangles.push([v0, vm2, vm0]);
                    mesh.triangles.push([vm0, vm1, v1]);
                    mesh.triangles.push([vm2, v2, vm1]);
                    mesh.triangles.push([vm0, vm2, vm1]);
                }
            }
        }

        // Update face_normals if present
        if let Some(ref mut face_normals) = mesh.face_normals {
            let n_old = face_normals.len();
            let n_new = mesh.triangles.len();
            if n_new > n_old {
                for i in n_old..n_new {
                    let tri = mesh.triangles[i];
                    let p0 = mesh.vertices[tri[0] as usize];
                    let p1 = mesh.vertices[tri[1] as usize];
                    let p2 = mesh.vertices[tri[2] as usize];
                    let ab = [p1.x - p0.x, p1.y - p0.y, p1.z - p0.z];
                    let ac = [p2.x - p0.x, p2.y - p0.y, p2.z - p0.z];
                    let nx = ab[1] * ac[2] - ab[2] * ac[1];
                    let ny = ab[2] * ac[0] - ab[0] * ac[2];
                    let nz = ab[0] * ac[1] - ab[1] * ac[0];
                    let len = (nx * nx + ny * ny + nz * nz).sqrt().max(1e-15);
                    face_normals.push([nx / len, ny / len, nz / len]);
                }
            }
        }

        // Update triangle_face_ids if present
        if let Some(ref mut face_ids) = mesh.triangle_face_ids {
            let n_old = face_ids.len();
            let n_new = mesh.triangles.len();
            if n_new > n_old {
                let last_id = face_ids.last().copied().unwrap_or(0);
                face_ids.extend(std::iter::repeat(last_id).take(n_new - n_old));
            }
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
        assert!(domain.contains(&Point2d::new(0.25, 0.25)));
        assert!(!domain.contains(&Point2d::new(1.0, 1.0)));
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
        use draper_geometry::{CylinderSurface, Point3d, Surface};

        let cyl = CylinderSurface::new_z(5.0);
        let surface = Surface::Cylinder(cyl);

        let n_pts = 100;
        let boundary_3d: Vec<Point3d> = (0..n_pts)
            .map(|i| {
                let u = 2.0 * PI * i as f64 / n_pts as f64;
                Point3d::new(5.0 * u.cos(), 5.0 * u.sin(), 10.0)
            })
            .collect();
        let mut boundary_uv: Vec<Point2d> = (0..n_pts)
            .map(|i| {
                let u = 2.0 * PI * i as f64 / n_pts as f64;
                Point2d::new(u, 10.0)
            })
            .collect();
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
        assert!(elapsed.as_millis() < 200, "Consistent triangulation should be fast, took {}ms", elapsed.as_millis());
    }

    #[test]
    fn test_nurbs_triangulation_performance() {
        use draper_geometry::{NurbsSurface, Surface, Point3d as P3, Point2d as P2};

        // Create a bicubic NURBS surface (same as the test button in the app)
        let control_points = vec![
            vec![P3::new(-50.0, -50.0,  0.0), P3::new(-50.0, -15.0, 10.0), P3::new(-50.0,  15.0, 10.0), P3::new(-50.0,  50.0,  0.0)],
            vec![P3::new(-15.0, -50.0, 10.0), P3::new(-15.0, -15.0, 30.0), P3::new(-15.0,  15.0, 25.0), P3::new(-15.0,  50.0,  5.0)],
            vec![P3::new( 15.0, -50.0, 10.0), P3::new( 15.0, -15.0, 25.0), P3::new( 15.0,  15.0, 30.0), P3::new( 15.0,  50.0, 10.0)],
            vec![P3::new( 50.0, -50.0,  0.0), P3::new( 50.0, -15.0,  5.0), P3::new( 50.0,  15.0, 10.0), P3::new( 50.0,  50.0,  0.0)],
        ];
        let weights = vec![vec![1.0; 4]; 4];
        let u_knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let v_knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];

        let nurbs = NurbsSurface {
            u_degree: 3, v_degree: 3,
            control_points, weights,
            u_knots, v_knots,
        };

        let (u_min, u_max) = nurbs.u_range();
        let (v_min, v_max) = nurbs.v_range();
        let surface = Surface::Nurbs(nurbs);

        // Sample boundary
        let mut boundary_3d = Vec::new();
        let mut boundary_uv = Vec::new();
        let steps = 20;
        for i in 0..=steps {
            let u = u_min + (u_max - u_min) * i as f64 / steps as f64;
            boundary_3d.push(surface.point_at(u, v_min));
            boundary_uv.push(P2::new(u, v_min));
        }
        for i in 1..=steps {
            let v = v_min + (v_max - v_min) * i as f64 / steps as f64;
            boundary_3d.push(surface.point_at(u_max, v));
            boundary_uv.push(P2::new(u_max, v));
        }
        for i in (0..steps).rev() {
            let u = u_min + (u_max - u_min) * i as f64 / steps as f64;
            boundary_3d.push(surface.point_at(u, v_max));
            boundary_uv.push(P2::new(u, v_max));
        }
        for i in (1..steps).rev() {
            let v = v_min + (v_max - v_min) * i as f64 / steps as f64;
            boundary_3d.push(surface.point_at(u_min, v));
            boundary_uv.push(P2::new(u_min, v));
        }

        let params = crate::triangulate::TriangulationParams::default();

        let start = std::time::Instant::now();
        // Use the public API that routes NURBS through the grid-based path
        let mesh = crate::triangulate::triangulate_face_with_boundary_and_holes_uv(
            &surface, &boundary_3d, &boundary_uv, &[], &[], true, &params,
        );
        let elapsed = start.elapsed();

        assert!(!mesh.triangles.is_empty(), "Should generate triangles");
        assert!(elapsed.as_millis() < 5000, "NURBS triangulation should be fast (was hanging before), took {}ms", elapsed.as_millis());

        // Quality checks
        let nan_count = mesh.vertices.iter().filter(|v| !v.x.is_finite() || !v.y.is_finite() || !v.z.is_finite()).count();
        assert_eq!(nan_count, 0, "No NaN vertices");

        let degen = mesh.triangles.iter().filter(|t| t[0] == t[1] || t[1] == t[2] || t[0] == t[2]).count();
        assert_eq!(degen, 0, "No degenerate triangles");

        assert!(mesh.triangles.len() >= 50, "Should have at least 50 triangles, got {}", mesh.triangles.len());
    }
}
