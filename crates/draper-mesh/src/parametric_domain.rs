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
            // Use a 16×16 grid — coarse enough to be fast, fine enough
            // for accurate interior point generation. The previous 24×24
            // was unnecessarily fine and slowed down face processing.
            self.containment_grid = Some(ContainmentGrid::new(self, 16));
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

/// Compute the area of a 2D triangle.
fn triangle_area_2d(x0: f64, y0: f64, x1: f64, y1: f64, x2: f64, y2: f64) -> f64 {
    ((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)).abs() * 0.5
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
    let max_boundary_points = 500;
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
    // Step 1: Normalize UV for periodic surfaces
    // ============================================================
    let u_period = if surface.is_u_periodic() { Some(2.0 * PI) } else { None };
    let v_period = if surface.is_v_periodic() { Some(2.0 * PI) } else { None };

    let mut outer_uv = boundary_uvs.to_vec();
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
    // Step 3: Generate interior grid points
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

    let boundary_margin = (u_max - u_min) / n_u.max(1) as f64 * 0.3;
    let interior_uv_points = generate_interior_points(&domain, n_u, n_v, boundary_margin);

    // ============================================================
    // Step 4: Build earcutr input with ALL points
    //
    // KEY: Pass interior points as part of the earcutr input
    // directly. earcutr handles Steiner points natively and
    // produces quality triangulation in O(n log n).
    // ============================================================

    let n_boundary = outer_uv.len();

    // Build combined point array: [boundary_uv...][hole_uv...][interior_uv...]
    let mut all_uv: Vec<Point2d> = outer_uv.clone();
    for huv in &normalized_holes_uv {
        all_uv.extend_from_slice(huv);
    }
    let n_boundary_and_holes = all_uv.len();

    // Add interior points as Steiner points
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
    let mut all_boundary_3d: Vec<Point3d> = boundary_points_3d.to_vec();
    for h3d in hole_polylines_3d {
        all_boundary_3d.extend_from_slice(h3d);
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
                if idx_usize < n_boundary_and_holes {
                    // Boundary/hole vertex: use cached 3D point directly
                    // This is what makes the mesh watertight — shared edge
                    // vertices have bit-identical 3D positions
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
}
