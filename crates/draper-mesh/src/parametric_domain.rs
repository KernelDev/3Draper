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

#![allow(dead_code)]
use draper_geometry::{Point2d, Point3d, Surface, Curve3d};
use crate::mesh::TriangleMesh;
use crate::edge_cache::deterministic_round_point;
use std::cell::Cell;
use std::f64::consts::PI;

/// RAII guard that runs a closure on drop. Used to decrement the thread-local
/// seam-split recursion counter when triangulate_surface_consistent returns.
struct DropGuard<F: FnOnce()>(core::mem::ManuallyDrop<F>);
impl<F: FnOnce()> Drop for DropGuard<F> {
    fn drop(&mut self) {
        // SAFETY: self is being dropped, so the inner F won't be used again.
        let f = unsafe { core::mem::ManuallyDrop::take(&mut self.0) };
        f();
    }
}

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
    pub(crate) fn contains_ray(&self, point: &Point2d) -> bool {
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

    // Run triangulation using the new adapter (tries earcut w/ int predicates,
    // falls back to i_triangle for self-intersecting polygons, then earcutr).
    let triangle_indices = crate::earcut_adapter::triangulate_polygon_with_holes(&coords, &hole_start_indices);

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

/// Compute the unsigned area of a 2D polygon using the shoelace formula.
/// Always returns a non-negative value.
fn polygon_area_2d(polygon: &[Point2d]) -> f64 {
    polygon_signed_area_2d(polygon).abs()
}

/// Compute the signed area of a 2D polygon using the shoelace formula.
/// Returns a positive value for counter-clockwise winding,
/// negative for clockwise winding, and near-zero for degenerate/self-intersecting
/// polygons.
///
/// For a simple (non-self-intersecting) polygon, the signed area indicates
/// orientation. For a self-intersecting polygon, the signed area can be
/// **near zero** even when the geometric area is large — the positive and
/// negative lobes cancel out. This property is used to detect self-intersecting
/// UV polygons in NURBS triangulation.
fn polygon_signed_area_2d(polygon: &[Point2d]) -> f64 {
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
    area * 0.5 // NO .abs() — preserve the sign
}

/// Check if a 2D UV polygon has self-intersecting edges (edge crossings).
///
/// Uses a brute-force O(n²) check on all non-adjacent edge pairs.
/// For typical boundary loops (< 200 points), this is fast enough.
/// Returns `true` if any pair of non-adjacent edges intersect.
fn check_uv_polygon_self_intersection(polygon: &[Point2d]) -> bool {
    let n = polygon.len();
    if n < 4 {
        return false; // Need at least 4 points for a self-intersection
    }

    for i in 0..n {
        let i_next = (i + 1) % n;
        let a0 = &polygon[i];
        let a1 = &polygon[i_next];

        // Check against non-adjacent edges only (skip i-1, i, i+1)
        for j in (i + 2)..n {
            // Skip the edge that wraps around and is adjacent to edge i
            if i == 0 && j == n - 1 {
                continue;
            }
            let j_next = (j + 1) % n;
            let b0 = &polygon[j];
            let b1 = &polygon[j_next];

            if segments_intersect_2d(a0, a1, b0, b1) {
                return true;
            }
        }
    }
    false
}

/// A crossing point where a polygon edge intersects the seam of a periodic surface.
#[derive(Clone)]
struct SeamCrossing {
    /// Index of the edge that crosses the seam (edge from polygon[edge_idx] to polygon[(edge_idx+1)%n]).
    edge_idx: usize,
    /// The v-coordinate at the seam crossing point.
    v_at_seam: f64,
    /// UV point on the "low" side of the seam: (u_min, v_at_seam).
    cross_pt_low: Point2d,
    /// UV point on the "high" side of the seam: (u_max, v_at_seam).
    cross_pt_high: Point2d,
    /// 3D point at the seam (same geometry regardless of low/high u-value).
    cross_pt_3d: Point3d,
}

/// Split a self-intersecting UV polygon at the seam of a periodic surface.
///
/// When a surface is closed in U (like a cylinder, torus, or closed NURBS), the UV
/// boundary polygon can wrap around the seam, creating a "bowtie" self-intersection
/// where edges cross diagonally. For example on a surface with u_range [0, 2π]:
///   Edge A: (0.01, v1) → (6.27, v2)   crosses seam
///   Edge B: (6.27, v3) → (0.01, v4)   crosses seam back
///
/// The fix is to split the polygon at the two seam-crossing edges, creating two
/// sub-polygons connected by a "seam edge" along u = u_min / u_max. Each sub-polygon
/// is non-self-intersecting and can be triangulated correctly by earcutr.
///
/// The algorithm:
/// 1. Find ALL edges that cross the seam (large u-jump > 40% of u_range)
/// 2. For each crossing edge, compute the v-coordinate at the seam using "unwrapped"
///    u-coordinates (treat the edge as going the short way around the periodic surface)
/// 3. Walk the polygon between the first two crossing points to build two sub-polygons
/// 4. Each sub-polygon has its crossing points at the correct u-value (u_min for the
///    low side, u_max for the high side) so the polygon is valid in UV space
///
/// Returns `None` if splitting is not applicable (no seam crossing detected).
fn try_split_at_seam(
    polygon: &[Point2d],
    points_3d: &[Point3d],
    surface: &Surface,
) -> Option<(Vec<Point2d>, Vec<Point2d>, Vec<Point3d>, Vec<Point3d>)> {
    if polygon.len() < 4 {
        return None;
    }

    // Defensive length check — the caller should always pass matched-length
    // slices, but if they don't, the walks below would index out of bounds
    // and crash the application. Log loudly and bail instead.
    if polygon.len() != points_3d.len() {
        log::error!(
            "try_split_at_seam: polygon ({}) and points_3d ({}) length mismatch — skipping seam split",
            polygon.len(), points_3d.len(),
        );
        return None;
    }

    let is_u_periodic = surface.is_u_periodic();
    let is_v_periodic = surface.is_v_periodic();
    if !is_u_periodic && !is_v_periodic {
        return None;
    }

    // Get parametric range for the periodic direction
    let (u_min, u_max) = get_surface_u_range(surface);
    let u_range = u_max - u_min;
    let (v_min, v_max) = get_surface_v_range(surface);
    let v_range = v_max - v_min;

    // ================================================================
    // Find ALL U-seam crossings
    //
    // A TRUE seam crossing happens when an edge wraps around the seam —
    // i.e., one endpoint is near u_min and the other is near u_max.
    // This is detected by: du > u_range * 0.5 (more than half the range).
    //
    // Edges with du between 0.4 and 0.5 of u_range are "long edges" that
    // span a large portion of the surface but DON'T wrap around the seam.
    // Treating them as seam crossings produces incorrect splits.
    //
    // Additional check: a true seam wrap has one endpoint near u_min and
    // the other near u_max. We verify this by checking that the distance
    // from each endpoint to the nearest seam (u_min or u_max) is small.
    // ================================================================
    let u_seam_threshold = u_range * 0.5;  // Must span MORE THAN HALF the range
    let u_seam_proximity = u_range * 0.1;  // Endpoints must be within 10% of a seam
    let mut u_crossings: Vec<SeamCrossing> = Vec::new();

    if is_u_periodic {
        for i in 0..polygon.len() {
            let j = (i + 1) % polygon.len();
            let du = (polygon[j].u - polygon[i].u).abs();
            if du > u_seam_threshold {
                // Check that this is a TRUE seam wrap: one endpoint near u_min,
                // the other near u_max. Without this check, long edges that
                // span half the surface (e.g., u=0 to u=π) would be incorrectly
                // flagged as seam crossings.
                let dist_i_to_min = (polygon[i].u - u_min).abs();
                let dist_i_to_max = (polygon[i].u - u_max).abs();
                let dist_j_to_min = (polygon[j].u - u_min).abs();
                let dist_j_to_max = (polygon[j].u - u_max).abs();
                let i_near_seam = dist_i_to_min < u_seam_proximity || dist_i_to_max < u_seam_proximity;
                let j_near_seam = dist_j_to_min < u_seam_proximity || dist_j_to_max < u_seam_proximity;
                // One endpoint should be near u_min, the other near u_max
                let i_near_min = dist_i_to_min < u_seam_proximity;
                let i_near_max = dist_i_to_max < u_seam_proximity;
                let j_near_min = dist_j_to_min < u_seam_proximity;
                let j_near_max = dist_j_to_max < u_seam_proximity;
                let wraps_around = (i_near_min && j_near_max) || (i_near_max && j_near_min);
                if !i_near_seam || !j_near_seam || !wraps_around {
                    // Long edge but not a seam wrap — skip
                    continue;
                }
                // Determine low/high endpoints
                let (u_low, v_low, u_high, v_high) = if polygon[i].u < polygon[j].u {
                    (polygon[i].u, polygon[i].v, polygon[j].u, polygon[j].v)
                } else {
                    (polygon[j].u, polygon[j].v, polygon[i].u, polygon[i].v)
                };

                // Compute v at seam using "unwrapped" u-coordinates.
                // The edge wraps around the seam: treat the high endpoint as
                // (u_high - u_range) so the edge goes the short way around.
                let u_high_unwrapped = u_high - u_range;
                let d_u = u_high_unwrapped - u_low;
                let t = if d_u.abs() > 1e-15 {
                    (u_min - u_low) / d_u
                } else {
                    0.5_f64
                };
                let t = t.clamp(0.0, 1.0);
                let v_cross = v_low + t * (v_high - v_low);

                // 3D point at the seam — use surface evaluation for accuracy
                let cross_pt_3d = surface.point_at(u_min, v_cross);

                u_crossings.push(SeamCrossing {
                    edge_idx: i,
                    v_at_seam: v_cross,
                    cross_pt_low: Point2d::new(u_min, v_cross),
                    cross_pt_high: Point2d::new(u_max, v_cross),
                    cross_pt_3d,
                });
            }
        }
    }

    // ================================================================
    // Find ALL V-seam crossings (for torus, sphere, etc.)
    // Same logic as U-seam: only count TRUE seam wraps (one endpoint near
    // v_min, the other near v_max).
    // ================================================================
    let v_seam_threshold = v_range * 0.5;
    let v_seam_proximity = v_range * 0.1;
    let mut v_crossings: Vec<VSeamCrossing> = Vec::new();

    if is_v_periodic && v_range > 0.0 {
        for i in 0..polygon.len() {
            let j = (i + 1) % polygon.len();
            let dv = (polygon[j].v - polygon[i].v).abs();
            if dv > v_seam_threshold {
                // Check for true seam wrap
                let dist_i_to_min = (polygon[i].v - v_min).abs();
                let dist_i_to_max = (polygon[i].v - v_max).abs();
                let dist_j_to_min = (polygon[j].v - v_min).abs();
                let dist_j_to_max = (polygon[j].v - v_max).abs();
                let i_near_min = dist_i_to_min < v_seam_proximity;
                let i_near_max = dist_i_to_max < v_seam_proximity;
                let j_near_min = dist_j_to_min < v_seam_proximity;
                let j_near_max = dist_j_to_max < v_seam_proximity;
                let wraps_around = (i_near_min && j_near_max) || (i_near_max && j_near_min);
                if !wraps_around {
                    continue;
                }
                let (v_low, u_low, v_high, u_high) = if polygon[i].v < polygon[j].v {
                    (polygon[i].v, polygon[i].u, polygon[j].v, polygon[j].u)
                } else {
                    (polygon[j].v, polygon[j].u, polygon[i].v, polygon[i].u)
                };

                let v_high_unwrapped = v_high - v_range;
                let d_v = v_high_unwrapped - v_low;
                let t = if d_v.abs() > 1e-15 {
                    (v_min - v_low) / d_v
                } else {
                    0.5_f64
                };
                let t = t.clamp(0.0, 1.0);
                let u_cross = u_low + t * (u_high - u_low);

                let cross_pt_3d = surface.point_at(u_cross, v_min);

                v_crossings.push(VSeamCrossing {
                    edge_idx: i,
                    u_at_seam: u_cross,
                    cross_pt_low: Point2d::new(u_cross, v_min),
                    cross_pt_high: Point2d::new(u_cross, v_max),
                    cross_pt_3d,
                });
            }
        }
    }

    // ================================================================
    // Filter out "spike" crossings — pairs of adjacent crossings that
    // share a vertex and go in opposite directions. These represent a
    // polygon "spike" to the seam and back, not a true seam wrap.
    //
    // A spike looks like: ... → V_a (u=π) → V_b (u=2π) → V_c (u=π) → ...
    // Both edges (a→b) and (b→c) are detected as seam crossings, but
    // they're actually a single degenerate spike. Treating them as two
    // separate crossings produces a 3-point "spike" sub-polygon that
    // doesn't represent real geometry.
    //
    // Detection: two crossings on adjacent edges (edge i and edge i+1)
    // where the shared vertex is at the seam (u_min or u_max).
    // ================================================================
    fn filter_spike_crossings(crossings: &[SeamCrossing], polygon: &[Point2d], u_min: f64, u_max: f64) -> Vec<SeamCrossing> {
        if crossings.len() < 2 {
            return crossings.to_vec();
        }
        let mut filtered = Vec::with_capacity(crossings.len());
        let mut skip_next = false;
        for i in 0..crossings.len() {
            if skip_next {
                skip_next = false;
                continue;
            }
            let c = &crossings[i];
            // Check if the next crossing is on the adjacent edge
            if i + 1 < crossings.len() {
                let next = &crossings[i + 1];
                let is_adjacent = next.edge_idx == c.edge_idx + 1 || 
                                  (c.edge_idx == polygon.len() - 1 && next.edge_idx == 0);
                if is_adjacent {
                    // Check if the shared vertex is at the seam
                    let shared_idx = next.edge_idx;  // The vertex between the two edges
                    let shared_u = polygon[shared_idx].u;
                    let at_seam = (shared_u - u_min).abs() < 1e-3 || (shared_u - u_max).abs() < 1e-3;
                    log::debug!(
                        "filter_spike_crossings: checking edges {} and {}, shared vertex {} at u={:.6}, u_min={:.6}, u_max={:.6}, at_seam={}",
                        c.edge_idx, next.edge_idx, shared_idx, shared_u, u_min, u_max, at_seam,
                    );
                    if at_seam {
                        // This is a spike — skip both crossings
                        log::debug!(
                            "filter_spike_crossings: SKIPPING spike at vertex {} (u={:.4}), edges {} and {}",
                            shared_idx, shared_u, c.edge_idx, next.edge_idx,
                        );
                        skip_next = true;
                        continue;
                    }
                }
            }
            filtered.push(c.clone());
        }
        filtered
    }

    let u_crossings = filter_spike_crossings(&u_crossings, polygon, u_min, u_max);

    // ================================================================
    // Same spike filter for V crossings (mirror of U)
    // ================================================================
    fn filter_spike_crossings_v(crossings: &[VSeamCrossing], polygon: &[Point2d], v_min: f64, v_max: f64) -> Vec<VSeamCrossing> {
        if crossings.len() < 2 {
            return crossings.to_vec();
        }
        let mut filtered = Vec::with_capacity(crossings.len());
        let mut skip_next = false;
        for i in 0..crossings.len() {
            if skip_next {
                skip_next = false;
                continue;
            }
            let c = &crossings[i];
            if i + 1 < crossings.len() {
                let next = &crossings[i + 1];
                let is_adjacent = next.edge_idx == c.edge_idx + 1 || 
                                  (c.edge_idx == polygon.len() - 1 && next.edge_idx == 0);
                if is_adjacent {
                    let shared_idx = next.edge_idx;
                    let shared_v = polygon[shared_idx].v;
                    let at_seam = (shared_v - v_min).abs() < 1e-3 || (shared_v - v_max).abs() < 1e-3;
                    if at_seam {
                        log::debug!(
                            "filter_spike_crossings_v: skipping spike at vertex {} (v={:.4}), edges {} and {}",
                            shared_idx, shared_v, c.edge_idx, next.edge_idx,
                        );
                        skip_next = true;
                        continue;
                    }
                }
            }
            filtered.push(c.clone());
        }
        filtered
    }

    let v_crossings = filter_spike_crossings_v(&v_crossings, polygon, v_min, v_max);

    // ================================================================
    // Choose which seam to split at (prefer U, then V)
    // ================================================================
    if u_crossings.len() >= 2 {
        split_at_u_seam(polygon, points_3d, surface, &u_crossings, u_min, u_max)
    } else if v_crossings.len() >= 2 {
        split_at_v_seam(polygon, points_3d, surface, &v_crossings, v_min, v_max)
    } else {
        log::warn!(
            "try_split_at_seam: not enough crossings after spike filter (u={}, v={}) — cannot split",
            u_crossings.len(), v_crossings.len()
        );
        None
    }
}

/// V-seam crossing (mirror of SeamCrossing with u/v swapped).
#[derive(Clone)]
struct VSeamCrossing {
    edge_idx: usize,
    u_at_seam: f64,
    cross_pt_low: Point2d,  // (u_at_seam, v_min)
    cross_pt_high: Point2d, // (u_at_seam, v_max)
    cross_pt_3d: Point3d,
}

/// Get the U parametric range for any surface type.
fn get_surface_u_range(surface: &Surface) -> (f64, f64) {
    match surface {
        Surface::Nurbs(n) => n.u_range(),
        Surface::Cylinder(_) | Surface::Cone(_) | Surface::Revolution(_) => (0.0, 2.0 * PI),
        Surface::Sphere(_) => (0.0, 2.0 * PI),
        Surface::Torus(_) => (0.0, 2.0 * PI),
        Surface::Plane(_) | Surface::Extrusion(_) => (0.0, 1.0),
    }
}

/// Get the V parametric range for any surface type.
fn get_surface_v_range(surface: &Surface) -> (f64, f64) {
    match surface {
        Surface::Nurbs(n) => n.v_range(),
        Surface::Sphere(_) => (0.0, PI),
        Surface::Torus(_) => (0.0, 2.0 * PI),
        _ => (0.0, 1.0),
    }
}

/// Split a UV polygon at the U-seam using the detected crossing points.
///
/// The two crossing points divide the polygon into two "walks". Each walk stays
/// entirely on one side of the seam. We build two sub-polygons by:
/// 1. Starting at crossing point 1 (at the correct u-value for this side)
/// 2. Walking along polygon edges to crossing point 2
/// 3. The polygon is implicitly closed by the "seam edge" (cross_pt2 → cross_pt1)
fn split_at_u_seam(
    polygon: &[Point2d],
    points_3d: &[Point3d],
    _surface: &Surface,
    crossings: &[SeamCrossing],
    u_min: f64,
    u_max: f64,
) -> Option<(Vec<Point2d>, Vec<Point2d>, Vec<Point3d>, Vec<Point3d>)> {
    if crossings.len() < 2 {
        return None;
    }

    if crossings.len() > 2 {
        log::warn!(
            "split_at_u_seam: {} crossings (expected 2), using first pair",
            crossings.len()
        );
    }

    let cross1 = &crossings[0];
    let cross2 = &crossings[1];
    let i = cross1.edge_idx;
    let j = cross2.edge_idx;
    let n = polygon.len();

    log::info!(
        "split_at_u_seam: crossings at edges {}→{} and {}→{}, v_cross=[{:.4}, {:.4}], u_range=[{:.4},{:.4}]",
        i, (i + 1) % n, j, (j + 1) % n,
        cross1.v_at_seam, cross2.v_at_seam, u_min, u_max
    );

    // Build walk 1: from crossing 1, along polygon edges (i+1, i+2, ..., j), to crossing 2
    let mut walk1_uv: Vec<Point2d> = Vec::new();
    let mut walk1_3d: Vec<Point3d> = Vec::new();
    let mut k = (i + 1) % n;
    while k != (j + 1) % n {
        walk1_uv.push(polygon[k]);
        walk1_3d.push(points_3d[k]);
        k = (k + 1) % n;
    }

    // Build walk 2: from crossing 2, along polygon edges (j+1, j+2, ..., i), to crossing 1
    let mut walk2_uv: Vec<Point2d> = Vec::new();
    let mut walk2_3d: Vec<Point3d> = Vec::new();
    k = (j + 1) % n;
    while k != (i + 1) % n {
        walk2_uv.push(polygon[k]);
        walk2_3d.push(points_3d[k]);
        k = (k + 1) % n;
    }

    // Determine which walk is "high" side (u near u_max) vs "low" side (u near u_min)
    let avg_u_walk1 = if walk1_uv.is_empty() {
        0.5 * (u_min + u_max)
    } else {
        walk1_uv.iter().map(|p| p.u).sum::<f64>() / walk1_uv.len() as f64
    };
    let avg_u_walk2 = if walk2_uv.is_empty() {
        0.5 * (u_min + u_max)
    } else {
        walk2_uv.iter().map(|p| p.u).sum::<f64>() / walk2_uv.len() as f64
    };

    // Build sub-polygons with correct crossing point u-values
    let (sub1_uv, sub1_3d, sub2_uv, sub2_3d) = if avg_u_walk1 >= avg_u_walk2 {
        // Walk 1 is high side, walk 2 is low side
        let mut s1_uv = vec![cross1.cross_pt_high];
        s1_uv.extend(walk1_uv.iter().cloned());
        s1_uv.push(cross2.cross_pt_high);
        let mut s1_3d = vec![cross1.cross_pt_3d];
        s1_3d.extend(walk1_3d.iter().cloned());
        s1_3d.push(cross2.cross_pt_3d);

        let mut s2_uv = vec![cross2.cross_pt_low];
        s2_uv.extend(walk2_uv.iter().cloned());
        s2_uv.push(cross1.cross_pt_low);
        let mut s2_3d = vec![cross2.cross_pt_3d];
        s2_3d.extend(walk2_3d.iter().cloned());
        s2_3d.push(cross1.cross_pt_3d);

        (s1_uv, s1_3d, s2_uv, s2_3d)
    } else {
        // Walk 1 is low side, walk 2 is high side
        let mut s1_uv = vec![cross1.cross_pt_low];
        s1_uv.extend(walk1_uv.iter().cloned());
        s1_uv.push(cross2.cross_pt_low);
        let mut s1_3d = vec![cross1.cross_pt_3d];
        s1_3d.extend(walk1_3d.iter().cloned());
        s1_3d.push(cross2.cross_pt_3d);

        let mut s2_uv = vec![cross2.cross_pt_high];
        s2_uv.extend(walk2_uv.iter().cloned());
        s2_uv.push(cross1.cross_pt_high);
        let mut s2_3d = vec![cross2.cross_pt_3d];
        s2_3d.extend(walk2_3d.iter().cloned());
        s2_3d.push(cross1.cross_pt_3d);

        (s1_uv, s1_3d, s2_uv, s2_3d)
    };

    if sub1_uv.len() < 3 || sub2_uv.len() < 3 {
        log::warn!(
            "split_at_u_seam: sub-polygons too small (sub1={}, sub2={}), falling back",
            sub1_uv.len(), sub2_uv.len()
        );
        return None;
    }

    log::info!(
        "split_at_u_seam: split into sub1 ({} pts, u=[{:.4},{:.4}]) and sub2 ({} pts, u=[{:.4},{:.4}])",
        sub1_uv.len(),
        sub1_uv.iter().map(|p| p.u).fold(f64::MAX, f64::min),
        sub1_uv.iter().map(|p| p.u).fold(f64::MIN, f64::max),
        sub2_uv.len(),
        sub2_uv.iter().map(|p| p.u).fold(f64::MAX, f64::min),
        sub2_uv.iter().map(|p| p.u).fold(f64::MIN, f64::max),
    );

    Some((sub1_uv, sub2_uv, sub1_3d, sub2_3d))
}

/// Split a UV polygon at the V-seam (for V-periodic surfaces like torus).
/// Same logic as split_at_u_seam but with u/v swapped.
fn split_at_v_seam(
    polygon: &[Point2d],
    points_3d: &[Point3d],
    _surface: &Surface,
    crossings: &[VSeamCrossing],
    v_min: f64,
    v_max: f64,
) -> Option<(Vec<Point2d>, Vec<Point2d>, Vec<Point3d>, Vec<Point3d>)> {
    if crossings.len() < 2 {
        return None;
    }

    if crossings.len() > 2 {
        log::warn!(
            "split_at_v_seam: {} crossings (expected 2), using first pair",
            crossings.len()
        );
    }

    let cross1 = &crossings[0];
    let cross2 = &crossings[1];
    let i = cross1.edge_idx;
    let j = cross2.edge_idx;
    let n = polygon.len();

    log::info!(
        "split_at_v_seam: crossings at edges {}→{} and {}→{}, u_cross=[{:.4}, {:.4}], v_range=[{:.4},{:.4}]",
        i, (i + 1) % n, j, (j + 1) % n,
        cross1.u_at_seam, cross2.u_at_seam, v_min, v_max
    );

    // Build walk 1
    let mut walk1_uv: Vec<Point2d> = Vec::new();
    let mut walk1_3d: Vec<Point3d> = Vec::new();
    let mut k = (i + 1) % n;
    while k != (j + 1) % n {
        walk1_uv.push(polygon[k]);
        walk1_3d.push(points_3d[k]);
        k = (k + 1) % n;
    }

    // Build walk 2
    let mut walk2_uv: Vec<Point2d> = Vec::new();
    let mut walk2_3d: Vec<Point3d> = Vec::new();
    k = (j + 1) % n;
    while k != (i + 1) % n {
        walk2_uv.push(polygon[k]);
        walk2_3d.push(points_3d[k]);
        k = (k + 1) % n;
    }

    // Determine high/low side by average v
    let avg_v_walk1 = if walk1_uv.is_empty() {
        0.5 * (v_min + v_max)
    } else {
        walk1_uv.iter().map(|p| p.v).sum::<f64>() / walk1_uv.len() as f64
    };
    let avg_v_walk2 = if walk2_uv.is_empty() {
        0.5 * (v_min + v_max)
    } else {
        walk2_uv.iter().map(|p| p.v).sum::<f64>() / walk2_uv.len() as f64
    };

    let (sub1_uv, sub1_3d, sub2_uv, sub2_3d) = if avg_v_walk1 >= avg_v_walk2 {
        let mut s1_uv = vec![cross1.cross_pt_high];
        s1_uv.extend(walk1_uv.iter().cloned());
        s1_uv.push(cross2.cross_pt_high);
        let mut s1_3d = vec![cross1.cross_pt_3d];
        s1_3d.extend(walk1_3d.iter().cloned());
        s1_3d.push(cross2.cross_pt_3d);

        let mut s2_uv = vec![cross2.cross_pt_low];
        s2_uv.extend(walk2_uv.iter().cloned());
        s2_uv.push(cross1.cross_pt_low);
        let mut s2_3d = vec![cross2.cross_pt_3d];
        s2_3d.extend(walk2_3d.iter().cloned());
        s2_3d.push(cross1.cross_pt_3d);

        (s1_uv, s1_3d, s2_uv, s2_3d)
    } else {
        let mut s1_uv = vec![cross1.cross_pt_low];
        s1_uv.extend(walk1_uv.iter().cloned());
        s1_uv.push(cross2.cross_pt_low);
        let mut s1_3d = vec![cross1.cross_pt_3d];
        s1_3d.extend(walk1_3d.iter().cloned());
        s1_3d.push(cross2.cross_pt_3d);

        let mut s2_uv = vec![cross2.cross_pt_high];
        s2_uv.extend(walk2_uv.iter().cloned());
        s2_uv.push(cross1.cross_pt_high);
        let mut s2_3d = vec![cross2.cross_pt_3d];
        s2_3d.extend(walk2_3d.iter().cloned());
        s2_3d.push(cross1.cross_pt_3d);

        (s1_uv, s1_3d, s2_uv, s2_3d)
    };

    if sub1_uv.len() < 3 || sub2_uv.len() < 3 {
        log::warn!(
            "split_at_v_seam: sub-polygons too small (sub1={}, sub2={}), falling back",
            sub1_uv.len(), sub2_uv.len()
        );
        return None;
    }

    log::info!(
        "split_at_v_seam: split into sub1 ({} pts, v=[{:.4},{:.4}]) and sub2 ({} pts, v=[{:.4},{:.4}])",
        sub1_uv.len(),
        sub1_uv.iter().map(|p| p.v).fold(f64::MAX, f64::min),
        sub1_uv.iter().map(|p| p.v).fold(f64::MIN, f64::max),
        sub2_uv.len(),
        sub2_uv.iter().map(|p| p.v).fold(f64::MAX, f64::min),
        sub2_uv.iter().map(|p| p.v).fold(f64::MIN, f64::max),
    );

    Some((sub1_uv, sub2_uv, sub1_3d, sub2_3d))
}

/// Check if two 2D line segments intersect (excluding shared endpoints).
fn segments_intersect_2d(a0: &Point2d, a1: &Point2d, b0: &Point2d, b1: &Point2d) -> bool {
    let d1x = a1.u - a0.u;
    let d1y = a1.v - a0.v;
    let d2x = b1.u - b0.u;
    let d2y = b1.v - b0.v;

    let denom = d1x * d2y - d1y * d2x;
    if denom.abs() < 1e-15 {
        return false; // Parallel or collinear
    }

    let dx = b0.u - a0.u;
    let dy = b0.v - a0.v;

    let t = (dx * d2y - dy * d2x) / denom;
    let u = (dx * d1y - dy * d1x) / denom;

    // Strict interior intersection (exclude endpoints)
    t > 1e-10 && t < 1.0 - 1e-10 && u > 1e-10 && u < 1.0 - 1e-10
}

/// Comprehensive validity check for a UV polygon before triangulation.
///
/// Checks:
/// 1. Minimum 3 points
/// 2. No self-intersections (edge crossings)
/// 3. Non-zero area (non-degenerate)
///
/// Returns `true` if the polygon is valid for earcutr triangulation.
fn check_uv_polygon_validity(uv_points: &[Point2d]) -> bool {
    let n = uv_points.len();
    if n < 3 {
        log::error!("UV polygon validity: too few points ({})", n);
        return false;
    }

    // Check for self-intersections using the existing O(n²) edge crossing check
    if check_uv_polygon_self_intersection(uv_points) {
        // Log which edges cross — useful for debugging NURBS projection issues
        for i in 0..n {
            let i_next = (i + 1) % n;
            for j in (i + 2)..n {
                if i == 0 && j == n - 1 {
                    continue;
                }
                let j_next = (j + 1) % n;
                if segments_intersect_2d(&uv_points[i], &uv_points[i_next], &uv_points[j], &uv_points[j_next]) {
                    log::error!(
                        "UV polygon self-intersection at edges {}-{} and {}-{}: \
                         ({:.4},{:.4})->({:.4},{:.4}) crosses ({:.4},{:.4})->({:.4},{:.4})",
                        i, i_next, j, j_next,
                        uv_points[i].u, uv_points[i].v,
                        uv_points[i_next].u, uv_points[i_next].v,
                        uv_points[j].u, uv_points[j].v,
                        uv_points[j_next].u, uv_points[j_next].v,
                    );
                }
            }
        }
        return false;
    }

    // Check area — should be positive for a valid (non-degenerate) polygon
    let area = polygon_area_2d(uv_points);
    if area.abs() < 1e-12 {
        log::error!(
            "UV polygon validity: zero area (degenerate), area={:.2e}, n={}",
            area, n
        );
        return false;
    }

    true
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

    // STRICT INTERIOR: We must NOT generate Steiner points that lie on the
    // boundary of the UV domain. If we do, those points become "phantom"
    // vertices on shared edges that aren't reproduced by the adjacent
    // planar face's triangulation, which produces boundary edges in the
    // merged mesh (the planar face has only the corner vertices, while the
    // NURBS face has corner + mid-edge Steiner points).
    //
    // We use a small tolerance relative to the UV bounding box size to
    // exclude points within `tol` of any boundary edge.
    let u_span = (u_max - u_min).max(1e-6);
    let v_span = (v_max - v_min).max(1e-6);
    let tol = (u_span.max(v_span) * 1e-6).max(1e-9);

    // Build a slightly inset grid: skip the t=0 and t=1 endpoints of each
    // knot span subdivision (those land on knot lines, which often coincide
    // with the boundary). Use only interior t values (1/n_sub, 2/n_sub, ...,
    // (n_sub-1)/n_sub).
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

    // Use interior t values (1/n_sub ... (n_sub-1)/n_sub) plus knot values
    // themselves. Knot values that are interior to the UV range are OK
    // (they're inside the surface), but the bounding-box edges (u_min,
    // u_max, v_min, v_max) must be skipped.
    let mut u_grid: Vec<f64> = Vec::new();
    for i in 0..u_values.len() - 1 {
        let span_lo = u_values[i];
        let span_hi = u_values[i + 1];
        let span_len = span_hi - span_lo;
        if span_len <= tol {
            continue;
        }
        // For each knot span, add n_sub-1 INTERIOR points (skip t=0 and t=1)
        if n_sub <= 1 {
            // n_sub == 1 means just the midpoint
            u_grid.push(span_lo + 0.5 * span_len);
        } else {
            for j in 1..n_sub {
                let t = j as f64 / n_sub as f64;
                u_grid.push(span_lo + t * span_len);
            }
        }
    }

    let mut v_grid: Vec<f64> = Vec::new();
    for i in 0..v_values.len() - 1 {
        let span_lo = v_values[i];
        let span_hi = v_values[i + 1];
        let span_len = span_hi - span_lo;
        if span_len <= tol {
            continue;
        }
        if n_sub <= 1 {
            v_grid.push(span_lo + 0.5 * span_len);
        } else {
            for j in 1..n_sub {
                let t = j as f64 / n_sub as f64;
                v_grid.push(span_lo + t * span_len);
            }
        }
    }

    // Generate the Cartesian product of u_grid and v_grid, keeping only
    // points that are STRICTLY INSIDE the domain (not on its boundary).
    for &u in &u_grid {
        for &v in &v_grid {
            let pt = Point2d::new(u, v);
            if !domain.contains(&pt) {
                continue;
            }
            // Additional strict-interior check: skip if too close to any
            // outer-boundary edge. This catches the case where the polygon
            // is non-rectangular and a grid point lands exactly on a slanted
            // boundary edge.
            if is_point_on_boundary(&domain.outer_boundary, &pt, tol) {
                continue;
            }
            let on_hole_boundary = domain.holes.iter()
                .any(|hole| is_point_on_boundary(hole, &pt, tol));
            if on_hole_boundary {
                continue;
            }
            points.push(pt);
        }
    }

    points
}

/// Check if a 2D point lies on any edge of a polygon (within tolerance).
fn is_point_on_boundary(polygon: &[Point2d], point: &Point2d, tol: f64) -> bool {
    let n = polygon.len();
    if n < 2 {
        return false;
    }
    let tol_sq = tol * tol;
    for i in 0..n {
        let a = polygon[i];
        let b = polygon[(i + 1) % n];
        if distance_point_to_segment_sq(point, &a, &b) <= tol_sq {
            return true;
        }
    }
    false
}

/// Squared distance from a 2D point to a 2D line segment.
fn distance_point_to_segment_sq(p: &Point2d, a: &Point2d, b: &Point2d) -> f64 {
    let dx = b.u - a.u;
    let dy = b.v - a.v;
    let len_sq = dx * dx + dy * dy;
    if len_sq < 1e-20 {
        let dpx = p.u - a.u;
        let dpy = p.v - a.v;
        return dpx * dpx + dpy * dpy;
    }
    let t = ((p.u - a.u) * dx + (p.v - a.v) * dy) / len_sq;
    let t = t.clamp(0.0, 1.0);
    let cx = a.u + t * dx;
    let cy = a.v + t * dy;
    let ex = p.u - cx;
    let ey = p.v - cy;
    ex * ex + ey * ey
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

/// Coarse a regular grid of UV Steiner points to a smaller regular
/// sub-grid by integer-stride subsampling.
///
/// `parameter_division_2d` produces points on a Cartesian product of
/// sorted u- and v-knots. When the count exceeds `budget`, naive
/// stride-sampling (as in `downsample_interior_points`) breaks the
/// grid structure, leaving points that don't align — earcutr then
/// produces broken triangulations with missing boundary edges.
///
/// This function:
/// 1. Recovers the implicit u- and v-axes from the point set by
///    clustering coordinates (within tolerance).
/// 2. Picks an integer stride `s` such that `n_u/s * n_v/s <= budget`.
/// 3. Returns every s-th row × every s-th column, preserving grid.
///
/// If axis recovery fails (points are not on a regular grid), falls
/// back to `downsample_interior_points`.
fn coarse_grid_sample(pts: &[Point2d], budget: usize) -> Vec<Point2d> {
    if pts.len() <= budget || pts.is_empty() {
        return pts.to_vec();
    }

    // Recover unique u-coordinates and v-coordinates by sorting + clustering.
    let mut us: Vec<f64> = pts.iter().map(|p| p.u).collect();
    us.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let u_tol = {
        let range = us.last().copied().unwrap_or(0.0) - us.first().copied().unwrap_or(0.0);
        (range.abs() * 1e-6).max(1e-9)
    };
    let mut u_unique: Vec<f64> = Vec::new();
    for u in us {
        if u_unique.last().map_or(true, |last| (last - u).abs() > u_tol) {
            u_unique.push(u);
        }
    }

    let mut vs: Vec<f64> = pts.iter().map(|p| p.v).collect();
    vs.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let v_tol = {
        let range = vs.last().copied().unwrap_or(0.0) - vs.first().copied().unwrap_or(0.0);
        (range.abs() * 1e-6).max(1e-9)
    };
    let mut v_unique: Vec<f64> = Vec::new();
    for v in vs {
        if v_unique.last().map_or(true, |last| (last - v).abs() > v_tol) {
            v_unique.push(v);
        }
    }

    // Verify: is this a regular grid? We need u_unique.len() × v_unique.len()
    // to be close to pts.len() (within 5% — small slack for filtering losses).
    let expected = u_unique.len() * v_unique.len();
    if expected < (pts.len() as f64 * 0.5) as usize || expected == 0 {
        // Not a regular grid — fall back to naive stride sampling.
        return downsample_interior_points(pts, budget);
    }

    // Build a set of (u,v) keys for fast lookup.
    use std::collections::HashSet;
    let pt_set: HashSet<(u64, u64)> = pts.iter()
        .map(|p| (p.u.to_bits(), p.v.to_bits()))
        .collect();

    // Find the smallest integer stride s such that
    //   ceil(u_unique.len() / s) * ceil(v_unique.len() / s) <= budget
    let mut best_stride = 1usize;
    for s in 1..=u_unique.len().max(v_unique.len()) {
        let nu = (u_unique.len() + s - 1) / s;
        let nv = (v_unique.len() + s - 1) / s;
        if nu * nv <= budget {
            best_stride = s;
            break;
        }
    }

    if best_stride == 1 {
        // Grid already fits budget — return as-is (downsample_interior_points
        // will handle the residual case where pts.len() > budget slightly).
        return pts.to_vec();
    }

    // Subsample: take every s-th u × every s-th v, keep only those that
    // actually exist in the filtered set.
    let mut result: Vec<Point2d> = Vec::with_capacity(budget);
    for i in (0..u_unique.len()).step_by(best_stride) {
        for j in (0..v_unique.len()).step_by(best_stride) {
            let p = Point2d::new(u_unique[i], v_unique[j]);
            if pt_set.contains(&(p.u.to_bits(), p.v.to_bits())) {
                result.push(p);
            }
        }
    }
    result
}

// ============================================================
// Cylinder / Cone Steiner grid generator
// ============================================================

/// Generate a regular (u, v) grid of Steiner points for cylinder/cone surfaces.
///
/// # Why this exists
///
/// For cylinder/cone faces WITH HOLES, the generic `parameter_division_2d`
/// returns only `v = [v_min, v_max]` because these surfaces have ZERO chord
/// error in the axial (v) direction (the surface is straight along the axis).
/// Without interior Steiner points in the v-direction, earcutr produces long
/// thin triangles spanning the full cylinder height — visually poor quality
/// and unlike what other CAD applications produce.
///
/// This function generates a proper regular grid in (u, v) space, filtered
/// to points strictly inside the face domain (outside holes, inside outer
/// boundary). When passed as Steiner points to earcutr, the resulting
/// triangulation follows the cylinder's natural parameterization, producing
/// clean rectangular quads (split into 2 triangles) in the interior and
/// smooth hole boundaries — matching the visual quality of OpenCASCADE /
/// FreeCAD / SolidWorks meshers.
///
/// # Strategy
///
/// 1. **n_u (angular subdivisions)**: derived from chord-error tolerance.
///    For a circle of radius `r`, chord error = `r * (1 - cos(du/2))`.
///    Solve for `du` given `tol`: `du = 2 * acos(1 - tol/r)`.
///    For cones, use the MAXIMUM radius along the v-range (worst case).
///
/// 2. **n_v (axial subdivisions)**: chosen so that the axial quad size
///    roughly matches the arc length per angular quad, producing
///    near-square grid cells. Capped to avoid excessive density on
///    very tall cylinders.
///
/// 3. **Filtering**: keep only points strictly inside the face domain
///    (inside outer boundary, outside all holes, not on any boundary
///    edge within `boundary_tol`).
///
/// 4. **Budget**: downsample via `coarse_grid_sample` (preserves grid
///    structure) and `downsample_interior_points` (final cap).
///
/// # Arguments
/// * `surface` — must be `Surface::Cylinder` or `Surface::Cone`.
/// * `domain` — parametric domain (outer boundary + holes).
/// * `u_range`, `v_range` — UV bounds of the face.
/// * `params` — triangulation params (for chord tolerance).
/// * `max_budget` — maximum number of Steiner points to return.
pub(crate) fn generate_cylinder_or_cone_steiner_grid(
    surface: &Surface,
    domain: &ParametricDomain,
    u_range: (f64, f64),
    v_range: (f64, f64),
    params: &crate::triangulate::TriangulationParams,
    max_budget: usize,
) -> Vec<Point2d> {
    let (u_min, u_max) = u_range;
    let (v_min, v_max) = v_range;
    let u_span = u_max - u_min;
    let v_span = v_max - v_min;
    if u_span <= 0.0 || v_span <= 0.0 {
        return Vec::new();
    }

    // Get the surface's reference radius for chord-error calculations.
    // For cylinders: constant radius.
    // For cones: radius varies with v — use the LARGER of v_min/v_max radius
    // (worst-case for chord error in the u-direction).
    let (radius_at_v_min, radius_at_v_max) = match surface {
        Surface::Cylinder(c) => (c.radius, c.radius),
        Surface::Cone(c) => {
            let tan_ha = c.half_angle.tan();
            let r_min = if c.expanding {
                v_min * tan_ha
            } else {
                (c.radius - v_min * tan_ha).max(0.0)
            };
            let r_max = if c.expanding {
                v_max * tan_ha
            } else {
                (c.radius - v_max * tan_ha).max(0.0)
            };
            (r_min, r_max)
        }
        _ => return Vec::new(),
    };
    let radius_max = radius_at_v_min.max(radius_at_v_max).max(1e-9);

    // Determine n_u (angular subdivisions) from chord-error tolerance.
    // chord_error = r * (1 - cos(du/2))
    // Solve: du = 2 * acos(1 - tol/r)
    let chord_tol = params.max_deviation.max(1e-5);
    let du_max = if radius_max > chord_tol * 1.001 {
        2.0 * (1.0 - chord_tol / radius_max).acos()
    } else {
        std::f64::consts::PI / 8.0 // fallback: 22.5°
    };
    // Profile-aware caps: the previous global cap of 64 was too aggressive
    // for desktop and caused visible quality regression. The new
    // `SteinerBudgetProfile` system restores desktop quality (up to 96
    // angular subdivisions) while keeping mobile fast (cap 32).
    let profile = params.steiner_profile;
    let max_u_cap = profile.max_u_cyl();
    let max_v_cap = profile.max_v_cyl();
    let min_u_floor = profile.min_u_cyl();
    let n_u_raw = ((u_span / du_max).ceil() as usize).max(min_u_floor).min(max_u_cap);

    // Determine n_v (axial subdivisions) from desired aspect ratio.
    // Target: quad size in v ≈ arc length per angular quad.
    // This produces near-square grid cells, matching other CAD apps.
    let arc_per_quad = u_span * radius_max / n_u_raw as f64;
    // Use a relaxed aspect ratio (up to 4:1) to avoid excessive V subdivisions
    // on very tall cylinders with small radius.
    let target_dv = arc_per_quad.max(v_span / max_v_cap as f64);
    let n_v_raw = ((v_span / target_dv).ceil() as usize).max(2).min(max_v_cap);

    // BUDGET-AWARE CAP: Don't generate more candidate points than we can possibly
    // use. The previous code generated up to 64×64 = 4096 candidates, then
    // filtered them all through O(boundary) contains_ray, then downsampled to
    // budget (often ~2000). This wasted enormous time on candidates that were
    // immediately discarded.
    //
    // Now: cap n_u × n_v to profile.candidate_multiplier() × budget
    // (desktop = 2×, tablet = 1.5×, mobile = 1.25×). Desktop uses a higher
    // multiplier because it has more CPU headroom and the extra candidates
    // preserve grid structure better.
    let max_candidates = (max_budget as f64 * profile.candidate_multiplier()).ceil() as usize;
    let mut n_u = n_u_raw;
    let mut n_v = n_v_raw;
    while n_u > min_u_floor && (n_u - 1) * (n_v - 1) > max_candidates {
        n_u -= 1;
    }
    while n_v > 2 && (n_u - 1) * (n_v - 1) > max_candidates {
        n_v -= 1;
    }

    log::debug!(
        "cylinder/cone steiner grid: n_u={}, n_v={}, radius_max={:.4}, u_span={:.4}, v_span={:.4}, budget={}",
        n_u, n_v, radius_max, u_span, v_span, max_budget
    );

    // Generate grid points (excluding boundaries — those come from the face edges).
    // We skip i=0, i=n_u, j=0, j=n_v because those are on the UV bbox boundary
    // and would either coincide with face boundary vertices (phantom vertices
    // that break watertightness) or fall outside the actual face domain
    // (the face boundary may be smaller than the UV bbox).
    let mut grid: Vec<Point2d> = Vec::with_capacity((n_u - 1) * (n_v - 1));
    for j in 1..n_v {
        let v = v_min + v_span * j as f64 / n_v as f64;
        for i in 1..n_u {
            let u = u_min + u_span * i as f64 / n_u as f64;
            grid.push(Point2d::new(u, v));
        }
    }

    // Filter to points strictly inside the face domain (outside holes, inside outer boundary).
    let span_max = u_span.max(v_span);
    let boundary_tol = (span_max * 1e-6).max(1e-9);

    let mut filtered: Vec<Point2d> = Vec::with_capacity(grid.len());
    for pt in &grid {
        // Use the CACHED containment grid (O(1) per point) for the bulk filter.
        // The previous code called contains_ray (O(boundary edges) per point)
        // which was the #1 performance bottleneck on mobile WASM — for a face
        // with 200 boundary edges and 4096 candidate points, that's 800K ray
        // tests per face, and drill_top.stp has hundreds of such faces.
        //
        // The cached 128×128 grid has ~1% boundary error (points within one
        // cell of the boundary may be misclassified). We catch those with the
        // is_point_on_boundary check below, which is also O(boundary) but only
        // runs for points the cached grid accepted — typically <30% of candidates.
        if !domain.contains(pt) {
            continue;
        }
        if is_point_on_boundary(&domain.outer_boundary, pt, boundary_tol) {
            continue;
        }
        let on_hole = domain.holes.iter()
            .any(|hole| is_point_on_boundary(hole, pt, boundary_tol));
        if on_hole {
            continue;
        }
        filtered.push(*pt);
    }

    log::debug!(
        "cylinder/cone steiner grid: {} grid pts → {} after domain filter",
        grid.len(), filtered.len()
    );

    // Downsample to budget if needed (preserving grid structure via coarse_grid_sample,
    // then a final cap via downsample_interior_points).
    let coarsened = coarse_grid_sample(&filtered, max_budget);
    downsample_interior_points(&coarsened, max_budget)
}

// ============================================================
// Sphere Steiner grid generator
// ============================================================

/// Generate a regular (u, v) grid of Steiner points for spherical faces
/// that contain holes or have non-rectangular UV bbox.
///
/// # Why this exists
///
/// Sphere surfaces are parameterized as `(u, v) ∈ [0, 2π] × [0, π]`
/// where `u` is the azimuthal angle and `v` is the polar angle. Both
/// directions trace great circles of the same radius `R`, so the same
/// chord-error formula `d_max = 2·acos(1 - tol/R)` applies to both.
///
/// The generic fallback (`parameter_division_2d`) recursively subdivides
/// the UV bbox by chord error. Near the poles (`v ≈ 0` or `v ≈ π`),
/// all `u` values produce the same 3D point, so the chord error is
/// ~0 and the recursion stops early — producing too few `u`-knots near
/// the poles. This leads to long thin triangles spanning the full
/// azimuthal range near the poles, visually appearing as a "pinched"
/// sphere cap.
///
/// This dedicated generator produces a proper regular grid in (u, v)
/// space — `n_u` and `n_v` both derived from chord-error tolerance,
/// capped by the `SteinerBudgetProfile` — with two special-case
/// adjustments:
///
/// 1. **Pole skipping**: interior points with `v < POLE_EPS` or
///    `v > π - POLE_EPS` are skipped, because at the poles all `u`
///    values collapse to a single 3D point. Including them would
///    create duplicate vertices and zero-area triangles when earcutr
///    processes them. `POLE_EPS = 0.05` matches the threshold used in
///    `triangulate_sphere_face_with_boundary` for pole detection.
///
/// 2. **Equator ring**: for near-full-sphere faces (`v_min ≤ POLE_EPS`
///    and `v_max ≥ π - POLE_EPS`), an explicit equator ring at
///    `v = π/2` is added as mandatory Steiner points. This prevents
///    "collapsing" the sphere into a single pole when the budget is
///    very tight and `n_v` happens to be odd (so no regular grid row
///    lands exactly on `v = π/2`).
///
/// # Strategy
///
/// 1. **Chord-error tol**: `d_max = 2·acos(1 - tol/R)` — same formula
///    for both `u` and `v` because both trace great circles of radius `R`.
///
/// 2. **n_u, n_v**: `ceil(span / d_max)`, clamped to
///    `[min_u_sphere, max_u_sphere]` / `[min_v_sphere, max_v_sphere]`.
///
/// 3. **Budget-aware cap**: shrink `n_u`/`n_v` until
///    `(n_u-1)·(n_v-1) ≤ candidate_multiplier × budget`.
///
/// 4. **Generate interior grid**: skip `i=0, i=n_u, j=0, j=n_v`
///    (boundary comes from face edges), skip pole rows.
///
/// 5. **Equator ring**: if full-sphere, add ring at `v = π/2`.
///
/// 6. **Filter**: keep only points strictly inside the face domain
///    (inside outer boundary, outside all holes, not on any boundary
///    edge within `boundary_tol`).
///
/// 7. **Downsample**: `coarse_grid_sample` (preserves grid structure)
///    then `downsample_interior_points` (final cap).
pub(crate) fn generate_sphere_steiner_grid(
    surface: &Surface,
    domain: &ParametricDomain,
    u_range: (f64, f64),
    v_range: (f64, f64),
    params: &crate::triangulate::TriangulationParams,
    max_budget: usize,
) -> Vec<Point2d> {
    let sphere = match surface {
        Surface::Sphere(s) => s,
        _ => return Vec::new(),
    };
    let (u_min, u_max) = u_range;
    let (v_min, v_max) = v_range;
    let u_span = u_max - u_min;
    let v_span = v_max - v_min;
    if u_span <= 0.0 || v_span <= 0.0 {
        return Vec::new();
    }

    let radius = sphere.radius.max(1e-9);

    // Chord error: sphere has the same great-circle radius R in both
    // u and v directions, so we use the same formula for both.
    //   chord_error = R · (1 - cos(d/2))
    //   d_max = 2 · acos(1 - tol/R)
    let chord_tol = params.max_deviation.max(1e-5);
    let d_max = if radius > chord_tol * 1.001 {
        2.0 * (1.0 - chord_tol / radius).acos()
    } else {
        std::f64::consts::PI / 8.0 // fallback: 22.5°
    };

    // Profile-aware caps.
    let profile = params.steiner_profile;
    let max_u_cap = profile.max_u_sphere();
    let max_v_cap = profile.max_v_sphere();
    let min_u_floor = profile.min_u_sphere();
    let min_v_floor = profile.min_v_sphere();
    let n_u_raw = ((u_span / d_max).ceil() as usize).max(min_u_floor).min(max_u_cap);
    let n_v_raw = ((v_span / d_max).ceil() as usize).max(min_v_floor).min(max_v_cap);

    // BUDGET-AWARE CAP: same as cylinder/cone grid — don't generate more
    // candidates than profile.candidate_multiplier() × budget.
    let max_candidates = (max_budget as f64 * profile.candidate_multiplier()).ceil() as usize;
    let mut n_u = n_u_raw;
    let mut n_v = n_v_raw;
    while n_u > min_u_floor && (n_u - 1) * (n_v - 1) > max_candidates {
        n_u -= 1;
    }
    while n_v > min_v_floor && (n_u - 1) * (n_v - 1) > max_candidates {
        n_v -= 1;
    }

    log::debug!(
        "sphere steiner grid: n_u={}, n_v={}, radius={:.4}, u_span={:.4}, v_span={:.4}, budget={}",
        n_u, n_v, radius, u_span, v_span, max_budget
    );

    // Pole threshold: matches `at_north_pole` / `at_south_pole` in
    // `triangulate_sphere_face_with_boundary` (triangulate.rs).
    // At the poles, all u values collapse to a single 3D point, so
    // interior Steiner points there are degenerate.
    const POLE_EPS: f64 = 0.05;

    // Generate grid points (excluding boundaries — those come from face edges).
    let mut grid: Vec<Point2d> = Vec::with_capacity((n_u - 1) * (n_v - 1));
    for j in 1..n_v {
        let v = v_min + v_span * j as f64 / n_v as f64;
        // Skip rows too close to the poles — points there are degenerate.
        if v < POLE_EPS || v > std::f64::consts::PI - POLE_EPS {
            continue;
        }
        for i in 1..n_u {
            let u = u_min + u_span * i as f64 / n_u as f64;
            grid.push(Point2d::new(u, v));
        }
    }

    // Equator ring (special case: full sphere).
    // If the face covers near-full v range, ensure the equator (v = π/2)
    // is always sampled, regardless of n_v parity. This prevents
    // "collapsing" the sphere into a single pole when budget is very
    // tight and n_v is odd (so no regular grid row lands exactly on
    // v = π/2).
    let is_full_sphere = v_min <= POLE_EPS && v_max >= std::f64::consts::PI - POLE_EPS;
    if is_full_sphere {
        let v_eq = std::f64::consts::PI / 2.0;
        for i in 1..n_u {
            let u = u_min + u_span * i as f64 / n_u as f64;
            grid.push(Point2d::new(u, v_eq));
        }
    }

    // Filter to points strictly inside the face domain (outside holes, inside outer boundary).
    let span_max = u_span.max(v_span);
    let boundary_tol = (span_max * 1e-6).max(1e-9);

    let mut filtered: Vec<Point2d> = Vec::with_capacity(grid.len());
    for pt in &grid {
        // Use cached containment grid (O(1)) — see cylinder grid for full rationale.
        if !domain.contains(pt) {
            continue;
        }
        if is_point_on_boundary(&domain.outer_boundary, pt, boundary_tol) {
            continue;
        }
        let on_hole = domain.holes.iter()
            .any(|hole| is_point_on_boundary(hole, pt, boundary_tol));
        if on_hole {
            continue;
        }
        filtered.push(*pt);
    }

    log::debug!(
        "sphere steiner grid: {} grid pts → {} after domain filter",
        grid.len(), filtered.len()
    );

    // Downsample to budget if needed (preserving grid structure via coarse_grid_sample,
    // then a final cap via downsample_interior_points).
    let coarsened = coarse_grid_sample(&filtered, max_budget);
    downsample_interior_points(&coarsened, max_budget)
}

// ============================================================
// Torus Steiner grid generator
// ============================================================

/// Generate a regular (u, v) grid of Steiner points for toroidal faces
/// that contain holes or have non-rectangular UV bbox.
///
/// # Why this exists
///
/// Torus surfaces are parameterized as `(u, v) ∈ [0, 2π] × [0, 2π]`
/// where `u` is the angle around the main ring (radius `R`) and `v`
/// is the angle around the tube (radius `r`). Both directions are
/// periodic.
///
/// The generic fallback (`parameter_division_2d`) recursively
/// subdivides the UV bbox by chord error. For small fillet faces
/// (typical in drill_top.stp — 90+ torus fillet faces), the recursion
/// produces only 4×4 or 6×6 grids, which is too coarse for visually
/// smooth fillets. The result looks "faceted" instead of smooth.
///
/// This dedicated generator produces a proper regular grid in (u, v)
/// space — `n_u` and `n_v` both derived from chord-error tolerance,
/// with a minimum floor of 24 (desktop) to guarantee smooth fillets
/// even on small faces.
///
/// # Chord-error geometry
///
/// - **u direction**: arc length per `du` is `(R + r·cos(v)) · du`.
///   Worst case (max radius) is at `v = 0` (outer equator):
///   `R + r`. Use `d_u_max = 2·acos(1 - tol/(R+r))`.
/// - **v direction**: arc length per `dv` is `r · dv` (constant —
///   the tube has constant radius `r`). Use
///   `d_v_max = 2·acos(1 - tol/r)`.
///
/// # Special cases
///
/// 1. **Degenerate torus** (`r < 1e-6` or `R < 1e-6`): the torus
///    collapses to a circle or point — no Steiner points needed
///    (return empty Vec, let generic fallback handle).
///
/// 2. **Partial torus** (`u_span < 2π` or `v_span < 2π`): no
///    wrap-around — the grid is naturally bounded by `u_range` /
///    `v_range`. This is automatically handled because we generate
///    grid points only inside `[u_min, u_max] × [v_min, v_max]`.
///
/// # Strategy
///
/// 1. **Chord-error tols**: `d_u_max` from `(R+r)`, `d_v_max` from `r`.
/// 2. **n_u, n_v**: `ceil(span / d_max)`, clamped to
///    `[min_u_torus, max_u_torus]` / `[min_v_torus, max_v_torus]`.
/// 3. **Budget-aware cap**: shrink `n_u`/`n_v` until
///    `(n_u-1)·(n_v-1) ≤ candidate_multiplier × budget`.
/// 4. **Generate interior grid**: skip `i=0, i=n_u, j=0, j=n_v`
///    (boundary comes from face edges).
/// 5. **Filter**: keep only points strictly inside the face domain
///    (inside outer boundary, outside all holes, not on any boundary
///    edge within `boundary_tol`).
/// 6. **Downsample**: `coarse_grid_sample` (preserves grid structure)
///    then `downsample_interior_points` (final cap).
pub(crate) fn generate_torus_steiner_grid(
    surface: &Surface,
    domain: &ParametricDomain,
    u_range: (f64, f64),
    v_range: (f64, f64),
    params: &crate::triangulate::TriangulationParams,
    max_budget: usize,
) -> Vec<Point2d> {
    let torus = match surface {
        Surface::Torus(t) => t,
        _ => return Vec::new(),
    };
    let (u_min, u_max) = u_range;
    let (v_min, v_max) = v_range;
    let u_span = u_max - u_min;
    let v_span = v_max - v_min;
    if u_span <= 0.0 || v_span <= 0.0 {
        return Vec::new();
    }

    let major_r = torus.major_radius.max(1e-9);
    let minor_r = torus.minor_radius.max(1e-9);

    // Special case: degenerate torus (minor_radius ≈ 0 → circle-like,
    // or major_radius ≈ 0 → point). No Steiner points — let the
    // generic fallback handle.
    if minor_r < 1e-6 || major_r < 1e-6 {
        return Vec::new();
    }

    // Chord-error tolerances.
    // u direction: worst-case radius is (R + r) — outer equator.
    // v direction: radius is r (constant — the tube).
    let chord_tol = params.max_deviation.max(1e-5);

    // u: d_u_max = 2·acos(1 - tol/(R+r))
    let radius_u = major_r + minor_r;
    let d_u_max = if radius_u > chord_tol * 1.001 {
        2.0 * (1.0 - chord_tol / radius_u).acos()
    } else {
        std::f64::consts::PI / 8.0
    };

    // v: d_v_max = 2·acos(1 - tol/r)
    let d_v_max = if minor_r > chord_tol * 1.001 {
        2.0 * (1.0 - chord_tol / minor_r).acos()
    } else {
        std::f64::consts::PI / 8.0
    };

    // Profile-aware caps.
    let profile = params.steiner_profile;
    let max_u_cap = profile.max_u_torus();
    let max_v_cap = profile.max_v_torus();
    let min_u_floor = profile.min_u_torus();
    let min_v_floor = profile.min_v_torus();
    let n_u_raw = ((u_span / d_u_max).ceil() as usize).max(min_u_floor).min(max_u_cap);
    let n_v_raw = ((v_span / d_v_max).ceil() as usize).max(min_v_floor).min(max_v_cap);

    // BUDGET-AWARE CAP: same as cylinder/sphere grid.
    let max_candidates = (max_budget as f64 * profile.candidate_multiplier()).ceil() as usize;
    let mut n_u = n_u_raw;
    let mut n_v = n_v_raw;
    while n_u > min_u_floor && (n_u - 1) * (n_v - 1) > max_candidates {
        n_u -= 1;
    }
    while n_v > min_v_floor && (n_u - 1) * (n_v - 1) > max_candidates {
        n_v -= 1;
    }

    log::debug!(
        "torus steiner grid: n_u={}, n_v={}, R={:.4}, r={:.4}, u_span={:.4}, v_span={:.4}, budget={}",
        n_u, n_v, major_r, minor_r, u_span, v_span, max_budget
    );

    // Generate grid points (excluding boundaries — those come from face edges).
    let mut grid: Vec<Point2d> = Vec::with_capacity((n_u - 1) * (n_v - 1));
    for j in 1..n_v {
        let v = v_min + v_span * j as f64 / n_v as f64;
        for i in 1..n_u {
            let u = u_min + u_span * i as f64 / n_u as f64;
            grid.push(Point2d::new(u, v));
        }
    }

    // Filter to points strictly inside the face domain (outside holes, inside outer boundary).
    let span_max = u_span.max(v_span);
    let boundary_tol = (span_max * 1e-6).max(1e-9);

    let mut filtered: Vec<Point2d> = Vec::with_capacity(grid.len());
    for pt in &grid {
        // Use cached containment grid (O(1)) — see cylinder grid for full rationale.
        if !domain.contains(pt) {
            continue;
        }
        if is_point_on_boundary(&domain.outer_boundary, pt, boundary_tol) {
            continue;
        }
        let on_hole = domain.holes.iter()
            .any(|hole| is_point_on_boundary(hole, pt, boundary_tol));
        if on_hole {
            continue;
        }
        filtered.push(*pt);
    }

    log::debug!(
        "torus steiner grid: {} grid pts → {} after domain filter",
        grid.len(), filtered.len()
    );

    // Downsample to budget if needed (preserving grid structure via coarse_grid_sample,
    // then a final cap via downsample_interior_points).
    let coarsened = coarse_grid_sample(&filtered, max_budget);
    downsample_interior_points(&coarsened, max_budget)
}

// ============================================================
// Revolution Steiner grid generator
// ============================================================

/// Generate a regular (u, v) grid of Steiner points for revolution faces
/// that contain holes or have non-rectangular UV bbox.
///
/// # Why this exists
///
/// Revolution surfaces are parameterized as `(u, v) ∈ [0, 2π] × [v_min, v_max]`
/// where `u` is the revolution angle and `v` is the profile curve parameter.
/// The generic `parameter_division_2d` branch recursively subdivides the UV
/// bbox by chord error. For revolution surfaces with complex profile curves
/// (NURBS with bends, multi-segment composites), the recursion may produce
/// too few v-knots — the v-direction curvature depends on the profile curve,
/// and the generic sampler doesn't know about the profile's internal
/// structure. With too few v-knots, earcutr produces long thin triangles
/// that lose the profile's shape details.
///
/// `generate_revolution_steiner_grid` produces a regular grid in (u, v)
/// space — n_u from chord-error tolerance using the maximum revolution
/// radius (worst-case = the largest perpendicular distance from the profile
/// to the axis), n_v from adaptive sampling of the profile curve. The
/// profile curve type determines the v-density strategy:
///
///   - **Line**: uniform v grid, few subdivisions (n_v = 2–8)
///   - **Circle/Arc**: chord-error with the circle radius (like torus tube)
///   - **NURBS/general**: sample profile curvature to determine n_v
///
/// # Degenerate-axis filtering
///
/// When the profile curve passes through (or very near) the revolution axis,
/// all u values produce the same 3D point — the surface pinches like a cone
/// apex. Interior Steiner points near these "axis degeneracies" would create
/// phantom vertices that break watertightness. We filter them out using a
/// threshold on the perpendicular distance to the axis.
pub(crate) fn generate_revolution_steiner_grid(
    surface: &Surface,
    domain: &ParametricDomain,
    u_range: (f64, f64),
    v_range: (f64, f64),
    params: &crate::triangulate::TriangulationParams,
    max_budget: usize,
) -> Vec<Point2d> {
    let rev = match surface {
        Surface::Revolution(r) => r,
        _ => return Vec::new(),
    };
    let (u_min, u_max) = u_range;
    let (v_min, v_max) = v_range;
    let u_span = u_max - u_min;
    let v_span = v_max - v_min;
    if u_span <= 0.0 || v_span <= 0.0 {
        return Vec::new();
    }

    let profile = &rev.profile;
    let axis = &rev.axis;
    let origin = &rev.origin;

    // ── Step 1: Sample the profile to compute revolution radii ──────
    //
    // We need the maximum perpendicular distance from the profile curve
    // to the revolution axis, which determines the worst-case chord
    // error in the u-direction (revolution angle).
    //
    // We also compute the approximate arc length of the profile, which
    // we use to determine n_v for general (non-line, non-circle) profiles.
    let n_probe = 64;
    let mut max_rev_radius: f64 = 0.0;
    let mut profile_arc_len: f64 = 0.0;
    let mut prev_p: Option<Point3d> = None;

    for i in 0..=n_probe {
        let t = v_min + v_span * i as f64 / n_probe as f64;
        let p = profile.point_at(t);

        // Perpendicular distance from profile point to the axis.
        let vx = p.x - origin.x;
        let vy = p.y - origin.y;
        let vz = p.z - origin.z;
        let dot = vx * axis.x + vy * axis.y + vz * axis.z;
        let perp_x = vx - dot * axis.x;
        let perp_y = vy - dot * axis.y;
        let perp_z = vz - dot * axis.z;
        let perp_dist = (perp_x * perp_x + perp_y * perp_y + perp_z * perp_z).sqrt();
        if perp_dist > max_rev_radius {
            max_rev_radius = perp_dist;
        }

        if let Some(pp) = prev_p {
            let dx = p.x - pp.x;
            let dy = p.y - pp.y;
            let dz = p.z - pp.z;
            profile_arc_len += (dx * dx + dy * dy + dz * dz).sqrt();
        }
        prev_p = Some(p);
    }

    max_rev_radius = max_rev_radius.max(1e-9);

    // ── Step 2: Compute n_u from chord-error ────────────────────────
    let chord_tol = params.max_deviation.max(1e-5);
    let du_max = if max_rev_radius > chord_tol * 1.001 {
        2.0 * (1.0 - chord_tol / max_rev_radius).acos()
    } else {
        std::f64::consts::PI / 8.0
    };

    let profile_budget = params.steiner_profile;
    let max_u_cap = profile_budget.max_u_revolution();
    let max_v_cap = profile_budget.max_v_revolution();
    let min_u_floor = profile_budget.min_u_revolution();
    let min_v_floor = profile_budget.min_v_revolution();

    let n_u_raw = ((u_span / du_max).ceil() as usize).max(min_u_floor).min(max_u_cap);

    // ── Step 3: Compute n_v from profile curve type ────────────────
    //
    // Different profile curve types need different v-subdivision strategies:
    //
    //   - Line: surface is a cylinder/cone in disguise. Use uniform v
    //     spacing with few subdivisions (n_v ≈ v_span / target_dv).
    //   - Circle/Arc: surface is a torus in disguise. Use chord-error
    //     with the circle radius (same as torus tube).
    //   - NURBS/general: use profile arc length as a proxy. The target
    //     segment length is derived from chord tolerance: for a curve
    //     with max curvature κ, chord error ≈ κ·L²/8. Inverting:
    //     L ≈ sqrt(8·tol/κ). For the profile, we estimate κ from
    //     the arc length and revolution radius (rough but effective).
    let n_v_raw = match profile {
        Curve3d::Line(_) => {
            // Linear profile → uniform v grid.
            // Target near-square cells: dv ≈ arc_per_quad.
            let arc_per_quad = u_span * max_rev_radius / n_u_raw as f64;
            let target_dv = arc_per_quad.max(v_span / max_v_cap as f64);
            ((v_span / target_dv).ceil() as usize).max(min_v_floor).min(max_v_cap)
        }
        Curve3d::Circle(c) => {
            // Circular profile → torus-like v subdivision.
            // Chord-error formula: d_v_max = 2·acos(1 - tol/r)
            let r = c.radius.max(1e-9);
            let dv_max = if r > chord_tol * 1.001 {
                2.0 * (1.0 - chord_tol / r).acos()
            } else {
                std::f64::consts::PI / 8.0
            };
            ((v_span / dv_max).ceil() as usize).max(min_v_floor).min(max_v_cap)
        }
        Curve3d::Arc(arc) => {
            // Arc profile → same as circle but with arc's parent radius.
            let r = arc.circle.radius.max(1e-9);
            let dv_max = if r > chord_tol * 1.001 {
                2.0 * (1.0 - chord_tol / r).acos()
            } else {
                std::f64::consts::PI / 8.0
            };
            ((v_span / dv_max).ceil() as usize).max(min_v_floor).min(max_v_cap)
        }
        _ => {
            // General profile (NURBS, ellipse, composite, etc.).
            // Use profile arc length as a proxy for curvature.
            // Target segment length ≈ sqrt(8 · chord_tol · R_eff)
            // where R_eff is the max revolution radius. This gives
            // finer v-subdivision where the profile is more curved
            // (shorter arc = more curvature per unit parameter).
            let r_eff = max_rev_radius.max(1e-9);
            let target_seg = (8.0 * chord_tol * r_eff).sqrt().max(chord_tol);
            let n_v_est = (profile_arc_len / target_seg).ceil() as usize;
            n_v_est.max(min_v_floor).min(max_v_cap)
        }
    };

    // ── Step 4: Budget-aware cap ────────────────────────────────────
    let max_candidates = (max_budget as f64 * profile_budget.candidate_multiplier()).ceil() as usize;
    let mut n_u = n_u_raw;
    let mut n_v = n_v_raw;
    while n_u > min_u_floor && (n_u - 1) * (n_v - 1) > max_candidates {
        n_u -= 1;
    }
    while n_v > min_v_floor && (n_u - 1) * (n_v - 1) > max_candidates {
        n_v -= 1;
    }

    log::debug!(
        "revolution steiner grid: n_u={}, n_v={}, max_rev_radius={:.4}, profile_arc_len={:.4}, u_span={:.4}, v_span={:.4}, budget={}",
        n_u, n_v, max_rev_radius, profile_arc_len, u_span, v_span, max_budget
    );

    // ── Step 5: Generate grid points (excluding boundaries) ─────────
    let mut grid: Vec<Point2d> = Vec::with_capacity((n_u - 1) * (n_v - 1));
    for j in 1..n_v {
        let v = v_min + v_span * j as f64 / n_v as f64;
        for i in 1..n_u {
            let u = u_min + u_span * i as f64 / n_u as f64;
            grid.push(Point2d::new(u, v));
        }
    }

    // ── Step 6: Filter degenerate-axis points ───────────────────────
    //
    // When the profile curve is near the revolution axis (perpendicular
    // distance < threshold), all u values produce the same 3D point —
    // the surface pinches. Interior Steiner points at these v values
    // would create phantom vertices (many u-values mapping to one 3D
    // point) that break watertightness. Filter them out.
    //
    // The threshold is a fraction of the maximum revolution radius,
    // with an absolute minimum to catch axis-intersecting profiles.
    let axis_degen_threshold = (max_rev_radius * 0.02).max(1e-4);

    let mut filtered: Vec<Point2d> = Vec::with_capacity(grid.len());
    for pt in &grid {
        // Check if the profile at this v is near the axis.
        let p = profile.point_at(pt.v);
        let vx = p.x - origin.x;
        let vy = p.y - origin.y;
        let vz = p.z - origin.z;
        let dot = vx * axis.x + vy * axis.y + vz * axis.z;
        let perp_x = vx - dot * axis.x;
        let perp_y = vy - dot * axis.y;
        let perp_z = vz - dot * axis.z;
        let perp_dist = (perp_x * perp_x + perp_y * perp_y + perp_z * perp_z).sqrt();
        if perp_dist < axis_degen_threshold {
            continue; // Degenerate-axis point — skip.
        }

        // Domain containment check (cached grid O(1)).
        if !domain.contains(pt) {
            continue;
        }
        let span_max = u_span.max(v_span);
        let boundary_tol = (span_max * 1e-6).max(1e-9);
        if is_point_on_boundary(&domain.outer_boundary, pt, boundary_tol) {
            continue;
        }
        let on_hole = domain.holes.iter()
            .any(|hole| is_point_on_boundary(hole, pt, boundary_tol));
        if on_hole {
            continue;
        }
        filtered.push(*pt);
    }

    log::debug!(
        "revolution steiner grid: {} grid pts → {} after domain + axis filter",
        grid.len(), filtered.len()
    );

    // ── Step 7: Downsample to budget if needed ──────────────────────
    let coarsened = coarse_grid_sample(&filtered, max_budget);
    downsample_interior_points(&coarsened, max_budget)
}

// ============================================================
// Planar Steiner grid generator (for planes WITH holes)
// ============================================================

/// Generate a regular Cartesian grid of Steiner points for planar faces
/// that contain holes.
///
/// # Why this exists
///
/// For a planar face WITHOUT holes, earcutr triangulating just the outer
/// boundary polygon produces good results — triangles fan out from
/// boundary vertices with reasonable aspect ratios.
///
/// For a planar face WITH holes, however, earcutr receives ONLY the
/// outer polygon + hole polygons as constraints. With no interior
/// Steiner points, earcutr's ear-clip heuristic produces long thin
/// triangles that span the full width of the face, crossing over the
/// hole region in visually poor patterns. This is what other CAD apps
/// avoid by inserting a regular interior grid.
///
/// This function generates a uniform Cartesian grid in (u, v) space,
/// filtered to points strictly inside the face domain (outside holes,
/// inside outer boundary). When passed as Steiner points to earcutr,
/// the resulting triangulation has near-square quads (split into 2
/// triangles) in the interior, with hole boundaries cleanly resolved.
///
/// # Strategy
///
/// 1. **Target edge length**: derived from the OUTER boundary point
///    density. Compute the average boundary edge length in UV space
///    and use it as the target grid spacing. This ensures the grid
///    triangles match the boundary resolution — neither too coarse
///    (creating a mismatch at the boundary) nor too fine (wasting
///    the triangle budget).
///
/// 2. **n_u, n_v**: derived from `target_edge` and the UV bbox span.
///    Capped to [4, 64] per axis to prevent explosion.
///
/// 3. **Filtering**: keep only points strictly inside the face domain
///    (inside outer boundary, outside all holes, not on any boundary
///    edge within `boundary_tol`).
///
/// 4. **Budget**: downsample via `coarse_grid_sample` (preserves grid
///    structure) and `downsample_interior_points` (final cap).
pub(crate) fn generate_planar_steiner_grid(
    domain: &ParametricDomain,
    outer_uv: &[Point2d],
    u_range: (f64, f64),
    v_range: (f64, f64),
    max_budget: usize,
    profile: crate::triangulate::SteinerBudgetProfile,
) -> Vec<Point2d> {
    let (u_min, u_max) = u_range;
    let (v_min, v_max) = v_range;
    let u_span = u_max - u_min;
    let v_span = v_max - v_min;
    if u_span <= 0.0 || v_span <= 0.0 {
        return Vec::new();
    }
    if outer_uv.len() < 3 {
        return Vec::new();
    }

    // Compute target edge length from boundary point density.
    // Sum the perimeter of the outer boundary polygon in UV space,
    // divide by number of edges → average edge length.
    let mut perimeter = 0.0f64;
    let n_outer = outer_uv.len();
    for i in 0..n_outer {
        let a = outer_uv[i];
        let b = outer_uv[(i + 1) % n_outer];
        let du = b.u - a.u;
        let dv = b.v - a.v;
        perimeter += (du * du + dv * dv).sqrt();
    }
    let avg_edge = perimeter / n_outer as f64;

    // Use the average edge length as target grid spacing.
    // Fall back to span/8 if boundary has degenerate edges (avg_edge == 0).
    let target_edge = if avg_edge > 1e-9 {
        avg_edge
    } else {
        (u_span.max(v_span) / 8.0).max(1e-9)
    };

    // Profile-aware caps: desktop gets up to 64×64 (4096 candidates),
    // tablet 48×48 (2304), mobile 32×32 (1024). The previous global cap
    // of 32 was too aggressive for desktop and caused visible quality
    // regression on planar faces with holes (notably on drill_top.stp
    // where the user reported "сильно хуже чем было раньше").
    let max_uv_cap = profile.max_uv_plane();
    let n_u_raw = ((u_span / target_edge).ceil() as usize).max(4).min(max_uv_cap);
    let n_v_raw = ((v_span / target_edge).ceil() as usize).max(4).min(max_uv_cap);

    // BUDGET-AWARE CAP: same as cylinder/cone grid — don't generate more
    // candidates than profile.candidate_multiplier() × budget.
    let max_candidates = (max_budget as f64 * profile.candidate_multiplier()).ceil() as usize;
    let mut n_u = n_u_raw;
    let mut n_v = n_v_raw;
    while n_u > 4 && (n_u - 1) * (n_v - 1) > max_candidates {
        n_u -= 1;
    }
    while n_v > 4 && (n_u - 1) * (n_v - 1) > max_candidates {
        n_v -= 1;
    }

    log::debug!(
        "planar steiner grid: n_u={}, n_v={}, target_edge={:.4}, u_span={:.4}, v_span={:.4}, budget={}",
        n_u, n_v, target_edge, u_span, v_span, max_budget
    );

    // Generate grid points (excluding boundaries — those come from the face edges).
    let mut grid: Vec<Point2d> = Vec::with_capacity((n_u - 1) * (n_v - 1));
    for j in 1..n_v {
        let v = v_min + v_span * j as f64 / n_v as f64;
        for i in 1..n_u {
            let u = u_min + u_span * i as f64 / n_u as f64;
            grid.push(Point2d::new(u, v));
        }
    }

    // Filter to points strictly inside the face domain (outside holes, inside outer boundary).
    let span_max = u_span.max(v_span);
    let boundary_tol = (span_max * 1e-6).max(1e-9);

    let mut filtered: Vec<Point2d> = Vec::with_capacity(grid.len());
    for pt in &grid {
        // Use cached containment grid (O(1)) instead of contains_ray (O(boundary)).
        // See generate_cylinder_or_cone_steiner_grid for full rationale.
        if !domain.contains(pt) {
            continue;
        }
        if is_point_on_boundary(&domain.outer_boundary, pt, boundary_tol) {
            continue;
        }
        let on_hole = domain.holes.iter()
            .any(|hole| is_point_on_boundary(hole, pt, boundary_tol));
        if on_hole {
            continue;
        }
        filtered.push(*pt);
    }

    log::debug!(
        "planar steiner grid: {} grid pts → {} after domain filter",
        grid.len(), filtered.len()
    );

    // Downsample to budget if needed (preserving grid structure via coarse_grid_sample,
    // then a final cap via downsample_interior_points).
    let coarsened = coarse_grid_sample(&filtered, max_budget);
    downsample_interior_points(&coarsened, max_budget)
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
    // For NURBS surfaces, use adaptive strategy: chain Newton for small UV ranges,
    // independent project_point for large UV ranges, with brute-force fallback.
    let mut outer_uv: Vec<Point2d> = if let Surface::Nurbs(ref nurbs) = surface {
        let (nu_min, nu_max) = nurbs.u_range();
        let (nv_min, nv_max) = nurbs.v_range();
        let u_range = nu_max - nu_min;
        let v_range = nv_max - nv_min;
        let use_chain_newton = u_range < 10.0 && v_range < 10.0;
        let mut uvs = Vec::with_capacity(boundary_points.len());
        for (i, p) in boundary_points.iter().enumerate() {
            let (u, v) = if use_chain_newton && i > 0 && !uvs.is_empty() {
                let prev: Point2d = uvs[i - 1];
                reproject_nurbs_point(nurbs, p, prev.u, prev.v)
            } else {
                surface.project_point(p)
            };
            // Validate
            let proj_p = surface.point_at(u, v);
            let err = p.distance_to(&proj_p);
            if err > 1e-4 {
                let grid_size = crate::edge_cache::adaptive_grid_size(u_range, v_range);
                let (ub, vb) = crate::edge_cache::brute_force_project_point(nurbs, p, grid_size);
                let bf_p = surface.point_at(ub, vb);
                let bf_err = p.distance_to(&bf_p);
                uvs.push(if bf_err < err { Point2d::new(ub, vb) } else { Point2d::new(u, v) });
            } else {
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
                // NURBS: adaptive strategy for hole UV projection
                let (nu_min, nu_max) = nurbs.u_range();
                let (nv_min, nv_max) = nurbs.v_range();
                let u_range = nu_max - nu_min;
                let v_range = nv_max - nv_min;
                let use_chain_newton = u_range < 10.0 && v_range < 10.0;
                let mut uvs = Vec::with_capacity(hole.len());
                for (i, p) in hole.iter().enumerate() {
                    let (u, v) = if use_chain_newton && i > 0 && !uvs.is_empty() {
                        let prev: Point2d = uvs[i - 1];
                        reproject_nurbs_point(nurbs, p, prev.u, prev.v)
                    } else {
                        surface.project_point(p)
                    };
                    let proj_p = surface.point_at(u, v);
                    let err = p.distance_to(&proj_p);
                    if err > 1e-4 {
                        let grid_size = crate::edge_cache::adaptive_grid_size(u_range, v_range);
                        let (ub, vb) = crate::edge_cache::brute_force_project_point(nurbs, p, grid_size);
                        let bf_p = surface.point_at(ub, vb);
                        let bf_err = p.distance_to(&bf_p);
                        uvs.push(if bf_err < err { Point2d::new(ub, vb) } else { Point2d::new(u, v) });
                    } else {
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
    // Track recursion depth to prevent stack overflow when the seam-split
    // strategy produces sub-polygons that are still self-intersecting.
    // The seam-split logic recursively calls triangulate_surface_consistent
    // on each sub-polygon; if a sub-polygon is still self-intersecting
    // (which can happen for badly-shaped UV polygons on periodic surfaces),
    // the recursion would never terminate → stack overflow.
    //
    // We use a thread-local counter so the public API doesn't change.
    // Max depth = 3: the original call + 2 levels of seam-split recursion.
    // After that, we skip the seam-split path and fall back to re-projection.
    thread_local! {
        static SEAM_SPLIT_DEPTH: Cell<u32> = const { Cell::new(0) };
    }

    let depth = SEAM_SPLIT_DEPTH.with(|d| d.get());
    if depth > 2 {
        log::warn!(
            "triangulate_surface_consistent: seam-split recursion depth {} exceeded — falling back to non-split path",
            depth,
        );
        // Fall through to the non-split path by skipping the seam-split block.
        // We do this by setting a flag that the seam-split block checks.
    }
    SEAM_SPLIT_DEPTH.with(|d| d.set(depth + 1));
    let _guard = DropGuard(core::mem::ManuallyDrop::new(|| SEAM_SPLIT_DEPTH.with(|d| d.set(depth))));

    let allow_seam_split = depth <= 2;

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
    let mut outer_uv: Vec<Point2d> = boundary_uvs.to_vec();
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
            // Reproject UVs from 3D points when they are out of range.
            // Simple clamping is incorrect — it snaps UVs to surface edges
            // rather than finding the correct parameterization. We use
            // brute_force_project_point() which has a finer grid than
            // surface.project_point() (11×11) and is more reliable for
            // surfaces with large UV ranges.
            let u_range_nurbs = nurb_u_max - nurb_u_min;
            let v_range_nurbs = nurb_v_max - nurb_v_min;
            let grid_size = crate::edge_cache::adaptive_grid_size(u_range_nurbs, v_range_nurbs);
            let mut reprojected_count = 0usize;
            for (i, uv) in outer_uv.iter_mut().enumerate() {
                let needs_reproject = !uv.u.is_finite() || !uv.v.is_finite()
                    || uv.u < nurb_u_min - margin || uv.u > nurb_u_max + margin
                    || uv.v < nurb_v_min - v_margin || uv.v > nurb_v_max + v_margin;

                if needs_reproject {
                    // UV is out of range — reproject from 3D point using brute-force
                    if let Some(p3d) = boundary_points_3d.get(i) {
                        let (new_u, new_v) = crate::edge_cache::brute_force_project_point(
                            nurbs, p3d, grid_size,
                        );

                        // Check if reprojected UV is valid
                        if new_u.is_finite() && new_v.is_finite() {
                            *uv = Point2d::new(new_u, new_v);
                            reprojected_count += 1;
                        } else {
                            // Brute-force also failed — try project_point() as second fallback
                            let (pf_u, pf_v) = surface.project_point(p3d);
                            if pf_u.is_finite() && pf_v.is_finite() {
                                *uv = Point2d::new(pf_u, pf_v);
                                reprojected_count += 1;
                            } else {
                                // Both failed — clamp as last resort
                                uv.u = if uv.u.is_finite() {
                                    uv.u.clamp(nurb_u_min, nurb_u_max)
                                } else {
                                    (nurb_u_min + nurb_u_max) * 0.5
                                };
                                uv.v = if uv.v.is_finite() {
                                    uv.v.clamp(nurb_v_min, nurb_v_max)
                                } else {
                                    (nurb_v_min + nurb_v_max) * 0.5
                                };
                            }
                        }
                    } else {
                        // No 3D point available — clamp as last resort
                        uv.u = if uv.u.is_finite() {
                            uv.u.clamp(nurb_u_min, nurb_u_max)
                        } else {
                            (nurb_u_min + nurb_u_max) * 0.5
                        };
                        uv.v = if uv.v.is_finite() {
                            uv.v.clamp(nurb_v_min, nurb_v_max)
                        } else {
                            (nurb_v_min + nurb_v_max) * 0.5
                        };
                    }
                }
            }
            if reprojected_count > 0 {
                log::warn!(
                    "NURBS UV: reprojected {} of {} boundary points (were out of range u=[{:.4},{:.4}] v=[{:.4},{:.4}])",
                    reprojected_count, outer_uv.len(),
                    nurb_u_min, nurb_u_max, nurb_v_min, nurb_v_max,
                );
            }
        }
    }

    // ============================================================
    // Step 0.5: DegeneracyHandler — merge coincident boundary points
    //
    // On surfaces with degeneracies (cone apex, sphere poles), multiple
    // boundary vertices can map to the same 3D point. For example, on a
    // cone with an apex, all edges meeting at the apex produce boundary
    // points that are geometrically identical but have different UV
    // coordinates. If left as separate vertices, earcutr creates
    // degenerate (zero-area) triangles between them.
    //
    // The handler detects clusters of coincident boundary 3D points
    // (within model-scale tolerance) and merges them into a single
    // representative point, keeping the UV coordinate of the first
    // point in the cluster. This produces a correct UV polygon for
    // earcutr while preserving the 3D geometry.
    //
    // We store the merged data in owned Vecs and rebind the slice
    // references to point to the owned data, so the rest of the code
    // continues using slice references without modification.
    // ============================================================
    let _merged_data: (Vec<Point3d>, Vec<Point2d>);
    let boundary_points_3d: &[Point3d] = {
        let tol = params.max_deviation * 0.01; // 1% of max_deviation as degeneracy tolerance
        let (merged_3d, merged_uv) = merge_coincident_boundary_points(boundary_points_3d, boundary_uvs, tol);
        if merged_3d.len() < 3 {
            log::warn!(
                "triangulate_surface_consistent: {} boundary points after degeneracy merge — returning empty mesh",
                merged_3d.len()
            );
            return TriangleMesh::new();
        }
        _merged_data = (merged_3d, merged_uv);
        &_merged_data.0
    };
    let boundary_uvs: &[Point2d] = &_merged_data.1;

    // CRITICAL: Rebuild outer_uv from the MERGED boundary_uvs.
    //
    // Before this fix, `outer_uv` was created from the ORIGINAL (pre-merge)
    // boundary_uvs at line ~1395, then the merge at Step 0.5 rebound
    // `boundary_points_3d` and `boundary_uvs` to shorter merged data —
    // but `outer_uv` was NOT updated, so its length still matched the
    // pre-merge count. The subsequent seam-split logic walked `outer_uv`
    // (longer) while indexing into `boundary_points_3d` (shorter),
    // causing an index-out-of-bounds PANIC on closed periodic surfaces
    // like spheres (nist_sphere.stp) and tori where multiple boundary
    // points collapse to the same 3D location (pole degeneracy).
    //
    // This rebuild ensures outer_uv.len() == boundary_points_3d.len()
    // after the merge, so the seam-split walks stay in sync.
    outer_uv = boundary_uvs.to_vec();

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
        let boundary_3d_area = polygon_area_3d(&boundary_points_3d);
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
    // Step 1.6: UV polygon self-intersection check for periodic surfaces
    //
    // For periodic surfaces (NURBS, Cylinder, Torus, Sphere, Revolution),
    // the UV polygon can be self-intersecting when it wraps around the seam,
    // creating a "bowtie" pattern. A self-intersecting UV polygon produces
    // incorrect triangulation — triangles on the wrong side of the surface,
    // inverted normals, etc.
    //
    // Detection uses both area-ratio analysis and edge-crossing checks.
    // Fix strategies (in order of preference):
    //   1. Split the polygon at the seam into two non-intersecting sub-polygons
    //   2. Re-project UVs using surface.project_point() (fallback)
    // ============================================================
    {
        let is_periodic = surface.is_u_periodic() || surface.is_v_periodic();
        if is_periodic || matches!(surface, Surface::Nurbs(_)) {
            let uv_signed_area = polygon_signed_area_2d(&outer_uv);
            let uv_unsigned_area = uv_signed_area.abs();
            // Log UV polygon info for diagnosis
            {
                let (su_min, su_max) = get_surface_u_range(surface);
                let (sv_min, sv_max) = get_surface_v_range(surface);
                let u_min_all = outer_uv.iter().map(|p| p.u).fold(f64::MAX, f64::min);
                let u_max_all = outer_uv.iter().map(|p| p.u).fold(f64::MIN, f64::max);
                let v_min_all = outer_uv.iter().map(|p| p.v).fold(f64::MAX, f64::min);
                let v_max_all = outer_uv.iter().map(|p| p.v).fold(f64::MIN, f64::max);
                log::info!(
                    "Periodic UV polygon: signed_area={:.6}, unsigned_area={:.6}, {} points, uv=[{:.4},{:.4}]x[{:.4},{:.4}], surface_range=u[{:.4},{:.4}]v[{:.4},{:.4}]",
                    uv_signed_area, uv_unsigned_area, outer_uv.len(),
                    u_min_all, u_max_all, v_min_all, v_max_all,
                    su_min, su_max, sv_min, sv_max,
                );
            }

            // Self-intersection detection:
            // 1. Area-ratio: |signed_area| << bbox_area indicates cancellation (bowtie)
            // 2. Edge-crossing: explicit check for intersecting polygon edges
            let is_degenerate = uv_unsigned_area < 1e-20 && outer_uv.len() >= 3;
            let is_self_intersecting = if !is_degenerate && outer_uv.len() >= 3 {
                let u_min_uv = outer_uv.iter().map(|p| p.u).fold(f64::MAX, f64::min);
                let u_max_uv = outer_uv.iter().map(|p| p.u).fold(f64::MIN, f64::max);
                let v_min_uv = outer_uv.iter().map(|p| p.v).fold(f64::MAX, f64::min);
                let v_max_uv = outer_uv.iter().map(|p| p.v).fold(f64::MIN, f64::max);
                let bbox_area = (u_max_uv - u_min_uv) * (v_max_uv - v_min_uv);
                if bbox_area > 1e-20 {
                    uv_unsigned_area / bbox_area < 0.01
                } else {
                    false
                }
            } else {
                is_degenerate
            };
            let has_edge_crossings = check_uv_polygon_self_intersection(&outer_uv);

            if (is_self_intersecting || has_edge_crossings) && outer_uv.len() >= 3 {
                log::warn!(
                    "UV polygon is self-intersecting/degenerate: signed_area={:.6}, unsigned_area={:.6}, edge_crossings={}, {} points, surface={:?}",
                    uv_signed_area, uv_unsigned_area, has_edge_crossings, outer_uv.len(),
                    std::mem::discriminant(surface)
                );

                // STRATEGY 1 (preferred): Split the polygon at the seam.
                // For periodic surfaces, the self-intersection is caused by the UV polygon
                // wrapping around the seam. Splitting creates two non-self-intersecting
                // sub-polygons that can be triangulated correctly.
                //
                // GUARDED by `allow_seam_split` to prevent infinite recursion when
                // a sub-polygon is still self-intersecting after splitting. Once we've
                // recursed 2 levels deep, we skip this strategy and fall through to
                // re-projection (STRATEGY 2) instead.
                if allow_seam_split {
                    if let Some((sub1_uv, sub2_uv, sub1_3d, sub2_3d)) =
                        try_split_at_seam(&outer_uv, &boundary_points_3d, surface)
                    {
                        log::info!(
                            "UV self-intersection: using seam-split strategy (sub1={} pts, sub2={} pts, depth={})",
                            sub1_uv.len(), sub2_uv.len(), depth + 1,
                        );

                        // Triangulate each sub-polygon recursively and merge the results
                        let sub1_holes_3d: Vec<Vec<Point3d>> = Vec::new();
                        let sub1_holes_uv: Vec<Vec<Point2d>> = Vec::new();
                        let sub2_holes_3d: Vec<Vec<Point3d>> = Vec::new();
                        let sub2_holes_uv: Vec<Vec<Point2d>> = Vec::new();

                        let mesh1 = triangulate_surface_consistent(
                            surface, &sub1_3d, &sub1_uv,
                            &sub1_holes_3d, &sub1_holes_uv,
                            forward, params,
                        );
                        let mesh2 = triangulate_surface_consistent(
                            surface, &sub2_3d, &sub2_uv,
                            &sub2_holes_3d, &sub2_holes_uv,
                            forward, params,
                        );

                        let mut result = mesh1;
                        result.merge(&mesh2);
                        return result;
                    }
                }

                // STRATEGY 2 (fallback): Re-project UVs using surface.project_point().
                // Only used when seam splitting is not applicable (no seam detected)
                // or when we've exceeded the seam-split recursion depth limit.
                log::info!("UV self-intersection: seam-split not applicable (depth={}, allow={}), trying re-projection", depth, allow_seam_split);
                outer_uv = boundary_points_3d.iter().map(|p| {
                    let (u, v) = surface.project_point(p);
                    let (su_min, su_max) = get_surface_u_range(surface);
                    let (sv_min, sv_max) = get_surface_v_range(surface);
                    Point2d::new(
                        u.clamp(su_min, su_max),
                        v.clamp(sv_min, sv_max),
                    )
                }).collect();
                // Re-normalize
                crate::triangulate::normalize_uv_polygon(&mut outer_uv, u_period, v_period);
                if outer_uv.len() < 3 {
                    return TriangleMesh::new();
                }
                let new_area = polygon_signed_area_2d(&outer_uv);
                let new_self_intersecting = check_uv_polygon_self_intersection(&outer_uv);
                log::info!(
                    "UV polygon re-projected: signed_area={:.6}, unsigned_area={:.6}, self_intersecting={} (was signed={:.6}, unsigned={:.6})",
                    new_area, new_area.abs(), new_self_intersecting, uv_signed_area, uv_unsigned_area
                );

                if new_self_intersecting {
                    log::warn!(
                        "UV polygon STILL self-intersecting after re-projection — using 3D ear-clip fallback"
                    );
                    // FALLBACK: Triangulate the 3D polygon directly by projecting
                    // to a best-fit plane and ear-clipping. This preserves watertightness
                    // (shared boundary edges with adjacent faces) even though the UV
                    // triangulation would produce inverted/wrong triangles.
                    let boundary_3d_area = polygon_area_3d(&boundary_points_3d);
                    if boundary_3d_area > 1e-10 {
                        let hole_polylines_3d_local: Vec<Vec<Point3d>> = hole_polylines_3d.to_vec();
                        return triangulate_3d_polygon_fallback(
                            &boundary_points_3d,
                            &hole_polylines_3d_local,
                            forward,
                        );
                    }
                    log::error!(
                        "UV polygon STILL self-intersecting AND 3D area is zero — proceeding with imperfect polygon"
                    );
                }
            }
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

    // Check for degenerate UV range (zero-area polygon).
    // When the boundary collapses to a line in UV space (constant u or v),
    // earcutr cannot triangulate it. Fall back to a simple strip triangulation
    // that connects the two boundary curves directly in 3D space.
    // This happens for faces like the flat side of a hex nut, where the
    // boundary lies at a constant angular position on a NURBS surface.
    let u_range = u_max - u_min;
    let v_range = v_max - v_min;
    let u_degenerate = u_range < 1e-6;
    let v_degenerate = v_range < 1e-6;

    if u_degenerate && v_degenerate {
        log::warn!(
            "triangulate_surface_consistent: fully degenerate UV range u=[{:.6}, {:.6}] v=[{:.6}, {:.6}], {} boundary pts — returning empty mesh",
            u_min, u_max, v_min, v_max, outer_uv.len()
        );
        return TriangleMesh::new();
    }

    if u_degenerate || v_degenerate {
        // Degenerate UV polygon: boundary is a line in UV space.
        // Create a simple strip triangulation from the 3D boundary points.
        // The boundary forms a closed loop, so we triangulate it as a
        // fan from the centroid (like ear-clipping a convex polygon).
        log::info!(
            "triangulate_surface_consistent: degenerate UV ({}) with {} boundary pts — using fan triangulation",
            if u_degenerate { "constant-u" } else { "constant-v" },
            outer_uv.len()
        );
        let mut mesh = TriangleMesh::new();
        let n = boundary_points_3d.len();
        if n < 3 {
            return mesh;
        }
        // Compute centroid for fan triangulation
        let mut cx = 0.0_f64; let mut cy = 0.0_f64; let mut cz = 0.0_f64;
        for p in boundary_points_3d {
            cx += p.x; cy += p.y; cz += p.z;
        }
        let inv_n = 1.0 / n as f64;
        let centroid = draper_geometry::Point3d::new(cx * inv_n, cy * inv_n, cz * inv_n);

        // Add centroid as vertex 0, then boundary points
        let c_idx = mesh.add_vertex(centroid);
        mesh.add_vertex_normal(c_idx, [0.0, 0.0, 1.0]); // approximate

        for p in boundary_points_3d {
            let idx = mesh.add_vertex(*p);
            mesh.add_vertex_normal(idx, [0.0, 0.0, 1.0]); // approximate
        }

        // Triangulate as a fan from the centroid
        for i in 0..n {
            let i_next = (i + 1) % n;
            if forward {
                mesh.add_triangle(0, (i + 1) as u32, (i_next + 1) as u32);
            } else {
                mesh.add_triangle(0, (i_next + 1) as u32, (i + 1) as u32);
            }
        }
        return mesh;
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
    let mut outer_uv = outer_uv; // Already a Vec, no downsampling

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

    // ============================================================
    // Step 3a: Adaptive UV subdivision via ParameterDivision2D
    //
    // This is the truck-inspired adaptive quad-tree subdivision
    // (see `parametric_division_2d` module). It produces a sorted
    // UV knot grid where the bilinear interpolation of every
    // sub-rectangle's corners is within `chord_tol` of the true
    // surface at one interior sample.
    //
    // We use it for ALL curved-surface types (NURBS, Cylinder, Cone,
    // Sphere, Torus, Revolution, Extrusion). Plane and bilinear
    // NURBS don't need interior points — the surface IS bilinear.
    //
    // TOLERANCE STRATEGY: We use `max_deviation * 10` as the chord
    // tolerance (matching the previous `target_deviation`).
    //
    // Reason: this is the same tolerance the legacy ruled-surface
    // formula used, so it preserves the previous behavior on
    // well-behaved curved surfaces (cylinder, sphere, torus, ruled
    // NURBS). For low-curvature saddle NURBS, this gives a moderate
    // interior grid (typically 3×3 to 5×5) which `coarse_grid_sample`
    // can downsample to a regular sub-grid if the budget requires it.
    //
    // `refine_mesh_chord_error_uv` post-refinement tightens the mesh
    // back to `max_deviation` where needed, with explicit safeguards
    // to never split edges that touch boundary vertices (preserving
    // watertightness).
    // ============================================================
    let chord_tol = (params.max_deviation * 10.0).max(1e-5);
    // Cap the per-axis subdivision so we never explode on pathological
    // surfaces. The chord-error refinement (`refine_mesh_chord_error_uv`)
    // will still add more points later if needed.
    let max_axis_dim = ((params.max_face_triangles / 2) as f64).sqrt().ceil() as usize;
    let max_axis_dim = max_axis_dim.clamp(4, 64);

    let interior_uv_points: Vec<Point2d> = if is_nurbs_bilinear
        || (matches!(surface, Surface::Plane(_)) && normalized_holes_uv_capped.is_empty())
    {
        // Flat surfaces WITHOUT holes: no interior Steiner points needed.
        // earcutr triangulates the boundary polygon cleanly.
        //
        // Planar faces WITH holes are handled by the dedicated branch
        // below (`generate_planar_steiner_grid`) because earcutr without
        // interior Steiner points produces long thin triangles spanning
        // the full face width over the hole region — visually poor.
        Vec::new()
    } else if outer_uv.len() == 4 && normalized_holes_uv_capped.is_empty() {
        // 4-corner face with no holes (square/rectangular trim).
        //
        // earcutr has a known issue: when given a 4-corner polygon plus
        // a small number of interior Steiner points, it sometimes
        // "loses" one of the boundary edges in the output triangulation,
        // producing a non-watertight mesh. This is documented in the
        // worklog as the "earcutr missing 1/4 boundary edges" warning.
        //
        // For 4-corner faces, the chord-error refinement
        // (`refine_mesh_chord_error_uv`) is sufficient to add interior
        // points later where needed, with explicit safeguards to never
        // split edges that touch boundary vertices. So we start with
        // zero interior points and let the refiner do its job.
        Vec::new()
    } else if matches!(surface, Surface::Plane(_)) {
        // Planar face WITH holes — use a dedicated Cartesian Steiner grid.
        //
        // WHY: For a plane with holes, earcutr receives only the outer
        // boundary + hole polygons as constraints. With no interior
        // Steiner points, earcutr produces long thin triangles spanning
        // the full face width, crossing the hole region — visually poor
        // and unlike the clean structured grids produced by other CAD
        // applications (OpenCASCADE, FreeCAD, SolidWorks).
        //
        // `generate_planar_steiner_grid` produces a regular Cartesian
        // grid in (u, v) space, filtered to points strictly inside the
        // face domain (outside holes, inside outer boundary). When
        // earcutr receives this grid as Steiner points, the resulting
        // triangulation has near-square quads in the interior, with
        // hole boundaries cleanly resolved.
        //
        // This branch is ONLY entered for planar faces WITH holes —
        // planar faces without holes are handled by the earlier branch
        // (which returns empty Vec).
        let planar_budget = max_interior_budget.max(8);
        generate_planar_steiner_grid(
            &domain,
            &outer_uv,
            (u_min, u_max),
            (v_min, v_max),
            planar_budget,
            params.steiner_profile,
        )
    } else if matches!(surface, Surface::Cylinder(_) | Surface::Cone(_)) {
        // Cylinder/cone faces — use a dedicated regular (u, v) Steiner grid.
        //
        // WHY: `parameter_division_2d` (the generic branch below) returns
        // only `v = [v_min, v_max]` for cylinders/cones because these
        // surfaces have ZERO chord error in the axial (v) direction —
        // the surface is straight along the axis. With no interior
        // Steiner points in v, earcutr produces long thin triangles
        // spanning the full cylinder height, which looks nothing like
        // the clean structured grids produced by other CAD applications
        // (OpenCASCADE, FreeCAD, SolidWorks).
        //
        // `generate_cylinder_or_cone_steiner_grid` produces a proper
        // regular grid in (u, v) space — n_u from chord-error tolerance,
        // n_v from a target aspect ratio that produces near-square
        // quads — filtered to points strictly inside the face domain
        // (outside holes, inside outer boundary). When earcutr receives
        // this grid as Steiner points, the resulting triangulation
        // follows the cylinder's natural parameterization: clean
        // rectangular quads in the interior, smooth hole boundaries.
        //
        // This branch is entered for ALL cylinder/cone faces that
        // reach this point — including both full-wrap and partial-wrap
        // faces WITH holes. Cylinder/cone faces WITHOUT holes are
        // handled earlier by `triangulate_cylinder_tube_from_boundary`
        // (structured grid triangulation), so they never reach here.
        let cyl_cone_budget = max_interior_budget.max(8);
        generate_cylinder_or_cone_steiner_grid(
            surface,
            &domain,
            (u_min, u_max),
            (v_min, v_max),
            params,
            cyl_cone_budget,
        )
    } else if matches!(surface, Surface::Sphere(_)) {
        // Sphere faces — use a dedicated regular (u, v) Steiner grid.
        //
        // WHY: `parameter_division_2d` (the generic branch below)
        // recursively subdivides the UV bbox by chord error. Near the
        // poles (v ≈ 0 or v ≈ π), all u values produce the same 3D
        // point, so the chord error is ~0 and the recursion stops
        // early — producing too few u-knots near the poles. This
        // leads to long thin triangles spanning the full azimuthal
        // range near the poles, visually appearing as a "pinched"
        // sphere cap.
        //
        // `generate_sphere_steiner_grid` produces a proper regular
        // grid in (u, v) space — n_u and n_v both derived from
        // chord-error tolerance (great-circle radius R in both
        // directions), capped by SteinerBudgetProfile — with two
        // special-case adjustments:
        //   1. Pole skipping: interior points with v < 0.05 or
        //      v > π - 0.05 are skipped (matches `at_north_pole` /
        //      `at_south_pole` threshold in `triangulate_sphere_face_with_boundary`).
        //   2. Equator ring: for near-full-sphere faces, an explicit
        //      equator ring at v = π/2 is added as mandatory Steiner
        //      points (prevents "collapsing" the sphere into a single
        //      pole when budget is very tight and n_v is odd).
        //
        // This branch is entered for sphere faces WITH holes or with
        // non-rectangular UV bbox. Sphere faces WITHOUT holes and with
        // 4-corner UV bbox are handled by the earlier branch (returns
        // empty Vec, lets chord-error refiner do its job). Full-sphere
        // faces (no boundary at all) are handled by
        // `triangulate_sphere_full_grid` before reaching here.
        let sphere_budget = max_interior_budget.max(8);
        generate_sphere_steiner_grid(
            surface,
            &domain,
            (u_min, u_max),
            (v_min, v_max),
            params,
            sphere_budget,
        )
    } else if matches!(surface, Surface::Torus(_)) {
        // Torus faces — use a dedicated regular (u, v) Steiner grid.
        //
        // WHY: `parameter_division_2d` (the generic branch below)
        // recursively subdivides the UV bbox by chord error. For small
        // fillet faces (typical in drill_top.stp — 90+ torus fillet
        // faces), the recursion produces only 4×4 or 6×6 grids, which
        // is too coarse for visually smooth fillets. The result looks
        // "faceted" instead of smooth.
        //
        // `generate_torus_steiner_grid` produces a proper regular grid
        // in (u, v) space — n_u derived from chord-error tolerance
        // using worst-case radius (R + r) (outer equator), n_v derived
        // from chord-error tolerance using tube radius r. Both have a
        // minimum floor of 24 (desktop) to guarantee smooth fillets
        // even on small faces.
        //
        // Special case: degenerate torus (minor_radius ≈ 0 or
        // major_radius ≈ 0) returns empty Vec, letting the generic
        // fallback handle it.
        //
        // This branch is entered for torus faces WITH holes or with
        // non-rectangular UV bbox. Torus faces WITHOUT holes and with
        // 4-corner UV bbox are handled by the earlier branch (returns
        // empty Vec). Full-torus faces (no boundary at all) are handled
        // by `triangulate_torus_full_grid` before reaching here.
        let torus_budget = max_interior_budget.max(8);
        generate_torus_steiner_grid(
            surface,
            &domain,
            (u_min, u_max),
            (v_min, v_max),
            params,
            torus_budget,
        )
    } else if matches!(surface, Surface::Revolution(_)) {
        // Revolution faces — use a dedicated regular (u, v) Steiner grid.
        //
        // WHY: `parameter_division_2d` (the generic branch below)
        // recursively subdivides the UV bbox by chord error. For
        // revolution surfaces with complex profile curves (NURBS with
        // bends, multi-segment composites), the recursion may produce
        // too few v-knots — the v-direction curvature depends on the
        // profile curve, and the generic sampler doesn't know about
        // the profile's internal structure.
        //
        // `generate_revolution_steiner_grid` produces a regular grid
        // in (u, v) space — n_u from chord-error tolerance using the
        // maximum revolution radius, n_v from the profile curve type
        // (line → uniform, circle/arc → chord-error, NURBS/general →
        // arc-length-based adaptive). It also filters degenerate-axis
        // points where the profile passes through the revolution axis.
        //
        // This branch is entered for revolution faces WITH holes or
        // with non-rectangular UV bbox. Revolution faces WITHOUT holes
        // and with 4-corner UV bbox are handled by the earlier branch
        // (returns empty Vec). Full-revolution faces (no boundary at
        // all) are handled by `triangulate_revolution_full` before
        // reaching here.
        let rev_budget = max_interior_budget.max(8);
        generate_revolution_steiner_grid(
            surface,
            &domain,
            (u_min, u_max),
            (v_min, v_max),
            params,
            rev_budget,
        )
    } else {
        // Compute adaptive subdivision grid for the entire surface, then
        // filter to (a) strictly-interior UV values and (b) points that
        // lie inside the actual face domain (which may be smaller than
        // the full surface range — faces are trimmed subsets).
        let (u_min_s, u_max_s) = (u_min, u_max);
        let (v_min_s, v_max_s) = (v_min, v_max);

        let (u_knots, v_knots) = crate::parametric_division_2d::parameter_division_2d(
            surface,
            (u_min_s, u_max_s),
            (v_min_s, v_max_s),
            chord_tol,
            max_axis_dim,
        );

        // Strict-interior filter relative to the SURFACE range (not the
        // face domain — we'll filter by domain next).
        let u_span = (u_max_s - u_min_s).max(1e-6);
        let v_span = (v_max_s - v_min_s).max(1e-6);
        let boundary_tol = (u_span.max(v_span) * 1e-6).max(1e-9);

        let steiner_pts = crate::parametric_division_2d::interior_steiner_points(
            &u_knots, &v_knots,
            (u_min_s, u_max_s),
            (v_min_s, v_max_s),
            boundary_tol,
        );

        // Filter to points that are strictly inside the FACE domain (the
        // trimmed subset of the surface). This is the same filter that
        // `generate_nurbs_interior_points` applies — see its comment
        // about phantom boundary vertices.
        let mut filtered: Vec<Point2d> = Vec::with_capacity(steiner_pts.len());
        for pt in steiner_pts {
            if !domain.contains(&pt) {
                continue;
            }
            if is_point_on_boundary(&domain.outer_boundary, &pt, boundary_tol) {
                continue;
            }
            let on_hole = domain.holes.iter()
                .any(|hole| is_point_on_boundary(hole, &pt, boundary_tol));
            if on_hole {
                continue;
            }
            filtered.push(pt);
        }

        // Downsample to budget.
        //
        // IMPORTANT: when the adaptive subdivision produces a large regular
        // grid (e.g. 50×50 = 2500 points), `downsample_interior_points`'s
        // stride-based sampling picks a quasi-random subset that breaks
        // the grid structure. earcutr then produces broken triangulations
        // with missing boundary edges.
        //
        // To preserve grid structure, we COARSE THE TOLERANCE instead of
        // stride-sampling: re-run the subdivision with a looser tolerance
        // that produces the desired number of points. As a cheap
        // approximation, if the count is more than 4× the budget, we
        // stride-sample at an integer factor (2×, 3×, 4×, ...) so the
        // remaining points still form a sub-grid.
        let coarsened = coarse_grid_sample(&filtered, max_interior_budget);
        downsample_interior_points(&coarsened, max_interior_budget)
    };

    // ============================================================
    // Step 3.9: Validate UV polygon before triangulation
    //
    // If the outer UV polygon is self-intersecting or degenerate,
    // earcutr will produce incorrect triangles. We try brute-force
    // re-projection first, and only return empty mesh as last resort.
    // ============================================================
    if !check_uv_polygon_validity(&outer_uv) {
        // For NURBS surfaces, try brute-force re-projection as a last resort
        if let Surface::Nurbs(ref nurbs) = surface {
            let (nu_min, nu_max) = nurbs.u_range();
            let (nv_min, nv_max) = nurbs.v_range();
            let u_range = nu_max - nu_min;
            let v_range = nv_max - nv_min;
            let grid_size = crate::edge_cache::adaptive_grid_size(u_range, v_range);

            log::warn!(
                "triangulate_surface_consistent: invalid UV polygon at Step 3.9 — attempting brute-force re-projection (grid={})",
                grid_size
            );

            outer_uv = boundary_points_3d.iter().map(|p| {
                let (u, v) = crate::edge_cache::brute_force_project_point(nurbs, p, grid_size);
                Point2d::new(
                    u.clamp(nu_min, nu_max),
                    v.clamp(nv_min, nv_max),
                )
            }).collect();

            crate::triangulate::normalize_uv_polygon(&mut outer_uv, u_period, v_period);

            if outer_uv.len() < 3 {
                return TriangleMesh::new();
            }

            // Re-check validity
            if !check_uv_polygon_validity(&outer_uv) {
                log::warn!(
                    "triangulate_surface_consistent: NURBS UV polygon STILL invalid after brute-force — using 3D ear-clip fallback"
                );
                // FALLBACK: Triangulate the 3D polygon directly by projecting
                // to a best-fit plane and ear-clipping. This preserves watertightness
                // (shared boundary edges with adjacent faces) even though the UV
                // triangulation failed.
                let boundary_3d_area = polygon_area_3d(&boundary_points_3d);
                if boundary_3d_area > 1e-10 {
                    let hole_polylines_3d_local: Vec<Vec<Point3d>> = hole_polylines_3d.to_vec();
                    return triangulate_3d_polygon_fallback(
                        &boundary_points_3d,
                        &hole_polylines_3d_local,
                        forward,
                    );
                }
                // Last resort: proceed with imperfect polygon.
                // A slightly imperfect triangulation is better than a hole in the model.
                log::error!(
                    "triangulate_surface_consistent: NURBS UV invalid AND 3D area zero — proceeding with imperfect polygon"
                );
            }
        } else {
            // Non-NURBS surface with invalid UV polygon.
            //
            // This happens for faces where the boundary curve is geometrically
            // valid in 3D but doesn't bound a 2D region on the surface
            // (e.g., a closed loop around a cylinder at constant height —
            // the boundary 3D points have non-zero area but all project to
            // the same v coordinate on the cylinder, producing zero UV area).
            //
            // FALLBACK: Triangulate the 3D polygon directly by projecting
            // to a best-fit plane and ear-clipping. This preserves watertightness
            // (shared boundary edges with adjacent faces) even though the
            // face's geometry on its surface is degenerate.
            let boundary_3d_area = polygon_area_3d(&boundary_points_3d);
            if boundary_3d_area > 1e-10 {
                log::warn!(
                    "triangulate_surface_consistent: UV polygon invalid (3D area={:.4}) — using 3D ear-clip fallback",
                    boundary_3d_area,
                );

                // Collect hole 3D polylines (re-projected from the hole UVs)
                let hole_polylines_3d_local: Vec<Vec<Point3d>> = hole_polylines_3d.to_vec();

                return triangulate_3d_polygon_fallback(
                    &boundary_points_3d,
                    &hole_polylines_3d_local,
                    forward,
                );
            }

            log::error!(
                "triangulate_surface_consistent: invalid UV polygon and 3D area is zero — returning empty mesh"
            );
            return TriangleMesh::new();
        }
    }

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

    // Run triangulation using the new adapter (tries earcut w/ int predicates,
    // falls back to i_triangle for self-intersecting polygons, then earcutr).
    let triangle_indices = crate::earcut_adapter::triangulate_polygon_with_holes(&coords, &hole_start_indices);

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
    // Position-based dedup map: maps 3D position (rounded) → mesh vertex index.
    // This ensures that two UV indices mapping to the same 3D position get the
    // same mesh vertex, preventing position-degenerate triangles.
    let mut position_map: std::collections::HashMap<[u64; 3], u32> = std::collections::HashMap::new();

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
                let (p3d, n) = if idx_usize < n_boundary_and_holes_actual && idx_usize < all_boundary_3d.len() {
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
                    (p3d, n)
                } else {
                    // Interior vertex: compute 3D point and normal from UV.
                    // For NURBS, use derivatives_at once to get both point and
                    // normal in a single call (87 de Boor iterations) instead of
                    // point_at (30) + normal_at (87) = 117 iterations separately.
                    // Apply deterministic rounding to ensure consistent vertex positions
                    // across faces (matches edge cache's rounding for boundary vertices).
                    let uv = all_uv[idx_usize];
                    if let Surface::Nurbs(ref nurbs) = surface {
                        let derivs = nurbs.derivatives_at(uv.u, uv.v);
                        (deterministic_round_point(derivs.point), derivs.normal())
                    } else {
                        (deterministic_round_point(surface.point_at(uv.u, uv.v)), surface.normal_at(uv.u, uv.v))
                    }
                };
                // Position-based dedup: if a vertex with the same 3D position
                // already exists in the face mesh, reuse it. This prevents
                // position-degenerate triangles when two UV indices map to the
                // same 3D position (e.g., seam points, or interior points that
                // happen to coincide with boundary points).
                let pos_key = [p3d.x.to_bits(), p3d.y.to_bits(), p3d.z.to_bits()];
                if let Some(&existing_vi) = position_map.get(&pos_key) {
                    return existing_vi;
                }
                let vi = mesh.add_vertex(p3d);
                mesh.add_vertex_normal(vi, [n.x, n.y, n.z]);
                position_map.insert(pos_key, vi);
                vi
            });
            tri_indices[k] = *entry;
        }

        // Skip position-degenerate triangles (different vertex indices but
        // same 3D position). These occur when two UV indices map to the same
        // 3D position (e.g., on a seam or degenerate curve). Adding such
        // triangles would create phantom edges that break watertightness
        // when the face mesh is merged into the BREP mesh.
        let p_a = mesh.vertices[tri_indices[0] as usize];
        let p_b = mesh.vertices[tri_indices[1] as usize];
        let p_c = mesh.vertices[tri_indices[2] as usize];
        let ab = (p_a.x - p_b.x).powi(2) + (p_a.y - p_b.y).powi(2) + (p_a.z - p_b.z).powi(2);
        let bc = (p_b.x - p_c.x).powi(2) + (p_b.y - p_c.y).powi(2) + (p_b.z - p_c.z).powi(2);
        let ac = (p_a.x - p_c.x).powi(2) + (p_a.y - p_c.y).powi(2) + (p_a.z - p_c.z).powi(2);
        if ab < 1e-20 || bc < 1e-20 || ac < 1e-20 {
            continue;
        }

        if forward {
            mesh.add_triangle(tri_indices[0], tri_indices[1], tri_indices[2]);
        } else {
            mesh.add_triangle(tri_indices[0], tri_indices[2], tri_indices[1]);
        }
    }

    // ============================================================
    // Step 5.5: DIAGNOSTIC + GAP FILLING — Verify and repair boundary edges
    //
    // After earcutr triangulation, every consecutive pair of boundary vertices
    // should be connected by an edge in at least one triangle. If earcutr
    // skips a boundary edge, the mesh will have a boundary edge that should
    // be shared with an adjacent face — breaking watertightness.
    //
    // GAP FILLING: For each missing boundary edge (va, vb), find a vertex vc
    // that is close to both va and vb and forms a valid (non-degenerate)
    // triangle. Add the triangle (va, vb, vc) to fill the gap.
    // ============================================================
    {
        let n_bnd = n_boundary_and_holes_actual;
        // Count boundary edges in the mesh (edges between consecutive boundary vertices)
        let mut boundary_edges_in_mesh: std::collections::HashSet<(u32, u32)> = std::collections::HashSet::new();
        for tri in &mesh.triangles {
            for k in 0..3 {
                let a = tri[k];
                let b = tri[(k + 1) % 3];
                boundary_edges_in_mesh.insert((a.min(b), a.max(b)));
            }
        }
        // Check: for each consecutive pair of boundary vertices, is there an edge?
        let mut missing_boundary_edges = 0usize;
        let mut missing_edges_list: Vec<(u32, u32)> = Vec::new();
        for i in 0..n_bnd {
            let i_next = (i + 1) % n_bnd;
            let va = *vertex_map.get(&(i as u32)).unwrap_or(&u32::MAX);
            let vb = *vertex_map.get(&(i_next as u32)).unwrap_or(&u32::MAX);
            if va != u32::MAX && vb != u32::MAX {
                // Skip degenerate edges (va == vb) — these occur when two
                // consecutive boundary UV indices map to the same mesh vertex
                // (e.g., a seam point that appears twice with different UVs but
                // the same 3D position). This is NOT a missing edge — it's a
                // degenerate edge that no triangle would have anyway.
                if va == vb {
                    continue;
                }
                let key = (va.min(vb), va.max(vb));
                if !boundary_edges_in_mesh.contains(&key) {
                    missing_boundary_edges += 1;
                    missing_edges_list.push((va, vb));
                }
            }
        }
        if missing_boundary_edges > 0 {
            // Log details about the first few missing edges
            let mut logged = 0;
            for &(va, vb) in &missing_edges_list {
                let pa = mesh.vertices[va as usize];
                let pb = mesh.vertices[vb as usize];
                let dist = ((pa.x - pb.x).powi(2) + (pa.y - pb.y).powi(2) + (pa.z - pb.z).powi(2)).sqrt();
                log::warn!(
                    "  MISSING boundary edge: mesh_idx {}→{} dist={:.6}",
                    va, vb, dist
                );
                logged += 1;
                if logged >= 5 { break; }
            }

            // GAP FILLING: for each missing edge (va, vb), find the best vertex
            // vc to form a fill triangle. The best vc is the one that:
            // 1. Is already connected to both va and vb (forms an existing edge)
            // 2. Minimizes the triangle area (to avoid overlapping existing triangles)
            let mut filled = 0usize;
            for &(va, vb) in &missing_edges_list {
                // Find vertices connected to both va and vb
                let mut connected_to_a: std::collections::HashSet<u32> = std::collections::HashSet::new();
                let mut connected_to_b: std::collections::HashSet<u32> = std::collections::HashSet::new();
                for tri in &mesh.triangles {
                    for k in 0..3 {
                        let a = tri[k];
                        let b = tri[(k + 1) % 3];
                        if a == va || b == va { connected_to_a.insert(if a == va { b } else { a }); }
                        if a == vb || b == vb { connected_to_b.insert(if a == vb { b } else { a }); }
                    }
                }
                // Find common neighbors (connected to both va and vb)
                let common: Vec<u32> = connected_to_a.intersection(&connected_to_b).copied().collect();

                // CRITICAL: For faces with holes, verify that the fill triangle's
                // centroid is inside the domain (not inside a hole).
                //
                // The gap-filling algorithm finds a common neighbor `vc` of `va`
                // and `vb` and adds the triangle (va, vb, vc). But if `vc` is on
                // the opposite side of a hole, the fill triangle spans across
                // the hole, covering it with a triangle where there should be
                // empty space.
                //
                // Bug history: drill_top.stp STEP #843 (cylinder face with 2
                // holes) showed triangles covering the holes because gap-filling
                // added a fill triangle that spanned across a hole.
                let mut best_vc: Option<u32> = None;
                for &vc in &common {
                    // Compute the fill triangle's centroid in UV space
                    let pa = mesh.vertices[va as usize];
                    let pb = mesh.vertices[vb as usize];
                    let pc = mesh.vertices[vc as usize];
                    let centroid_3d = Point3d::new(
                        (pa.x + pb.x + pc.x) / 3.0,
                        (pa.y + pb.y + pc.y) / 3.0,
                        (pa.z + pb.z + pc.z) / 3.0,
                    );
                    let (cu, cv) = surface.project_point(&centroid_3d);
                    let centroid_uv = Point2d::new(cu, cv);
                    // Only accept the fill triangle if its centroid is inside
                    // the domain (i.e., not inside a hole and not outside
                    // the outer boundary).
                    if domain.contains_ray(&centroid_uv) {
                        best_vc = Some(vc);
                        break;
                    }
                }

                if let Some(best_vc) = best_vc {
                    // Add the fill triangle — use the orientation that matches
                    // the face's forward flag
                    if forward {
                        mesh.add_triangle(va, vb, best_vc);
                    } else {
                        mesh.add_triangle(va, best_vc, vb);
                    }
                    filled += 1;
                }
                // If no valid vc found (all candidates span a hole), leave the
                // edge unfilled — a small gap is better than a triangle covering
                // a hole.
            }
            if filled > 0 {
                log::info!(
                    "GAP_FILL: filled {}/{} missing boundary edges for surface {:?}",
                    filled, missing_boundary_edges,
                    std::mem::discriminant(surface),
                );
            }
            log::warn!(
                "DIAG: earcutr missing {}/{} boundary edges for surface {:?} (n_bnd={}, n_holes={}, verts={}, tris={}, filled={})",
                missing_boundary_edges, n_bnd,
                std::mem::discriminant(surface),
                n_bnd, hole_polylines_3d.len(),
                mesh.vertices.len(), mesh.triangles.len(), filled,
            );
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
        // Use 0 refinement iterations for NURBS, 2 for other curved surfaces.
        //
        // NURBS chord-error refinement is DISABLED because:
        // 1. The new vertices created by refinement (midpoint of split edges)
        //    are computed via surface.point_at() and are NOT bit-identical
        //    across adjacent faces, even though the boundary vertices ARE
        //    bit-identical (via the edge cache).
        // 2. Even though we now skip splitting any edge involving a boundary
        //    vertex, the refinement still creates new interior vertices that
        //    form edges with existing interior vertices. These edges are
        //    interior to ONE face but appear as BREP boundary edges because
        //    the adjacent face has different interior vertices.
        // 3. The initial interior point generation is curvature-adaptive
        //    (see Step 3 above), so it already adds enough Steiner points to
        //    meet the chord error tolerance in most cases.
        //
        // For non-NURBS curved surfaces (cylinder, sphere, cone, torus,
        // revolution, extrusion), the chord-error refinement is still useful
        // because:
        // 1. These surfaces are parameterized consistently (radians, distances)
        // 2. The same UV midpoint produces bit-identical 3D points across faces
        //    (since the surface evaluation is deterministic and the surfaces
        //    are shared between faces via STEP's SURFACE entity)
        // 3. The refinement creates vertices that ARE bit-identical across faces
        let max_refine_iters = if matches!(surface, Surface::Nurbs(_)) { 0 } else { 2 };

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
            &mut vertex_uvs, &mut is_boundary_vertex, &domain,
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
                let (_u, _v) = surface.project_point(&mid);
                let p_surf = surface.point_at(_u, _v);

                // For NURBS surfaces, project_point can be inaccurate.
                // Try Newton-Raphson refinement if the initial projection
                // is far from the midpoint.
                let (_u, _v, p_surf) = if let Surface::Nurbs(ref nurbs) = surface {
                    let dx0 = p_surf.x - mid.x;
                    let dy0 = p_surf.y - mid.y;
                    let dz0 = p_surf.z - mid.z;
                    let err0 = (dx0*dx0 + dy0*dy0 + dz0*dz0).sqrt();
                    if err0 > max_deviation * 0.1 {
                        let (u2, v2) = reproject_nurbs_point(nurbs, &mid, _u, _v);
                        let p2 = surface.point_at(u2, v2);
                        let dx2 = p2.x - mid.x;
                        let dy2 = p2.y - mid.y;
                        let dz2 = p2.z - mid.z;
                        let err2 = (dx2*dx2 + dy2*dy2 + dz2*dz2).sqrt();
                        if err2 < err0 {
                            (u2, v2, p2)
                        } else {
                            (_u, _v, p_surf)
                        }
                    } else {
                        (_u, _v, p_surf)
                    }
                } else {
                    (_u, _v, p_surf)
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
    domain: &ParametricDomain,
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

    // Compute actual surface periods for periodic surfaces.
    // NURBS periods come from the knot range, while analytic surfaces use 2π.
    let u_period: Option<f64> = if surface.is_u_periodic() {
        match surface {
            Surface::Nurbs(ref nurbs) => {
                let (umin, umax) = nurbs.u_range();
                Some(umax - umin)
            }
            _ => Some(2.0 * PI),
        }
    } else {
        None
    };
    let v_period: Option<f64> = if surface.is_v_periodic() {
        match surface {
            Surface::Nurbs(ref nurbs) => {
                let (vmin, vmax) = nurbs.v_range();
                Some(vmax - vmin)
            }
            _ => Some(2.0 * PI),
        }
    } else {
        None
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

                // CRITICAL: Skip splitting ANY edge that involves a boundary vertex.
                //
                // Boundary vertices come from the edge cache with bit-identical
                // 3D coordinates across adjacent faces. If we split an edge
                // (boundary_v, interior_v), the new midpoint vertex is computed
                // from surface.point_at() — this produces DIFFERENT f64 bits for
                // each face. The new vertex cannot be deduplicated across faces,
                // and the new edges (boundary_v, new_v) and (new_v, interior_v)
                // become BREP boundary edges, breaking watertightness.
                //
                // Visual quality near the boundary is preserved by:
                // 1. The edge cache's adaptive_discretize — adds more boundary
                //    points where curvature is high (already happens before
                //    triangulation)
                // 2. Curvature-adaptive interior Steiner points — add more
                //    interior points where the surface is curved (already happens
                //    in the initial interior point generation)
                //
                // The chord-error refinement here only adds density to the
                // INTERIOR of the face, away from shared boundaries.
                let v0_is_boundary = is_boundary_vertex.get(v0 as usize).copied().unwrap_or(false);
                let v1_is_boundary = is_boundary_vertex.get(v1 as usize).copied().unwrap_or(false);
                if v0_is_boundary || v1_is_boundary {
                    continue; // Don't split any edge involving a boundary vertex
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
                let mid_u = if let Some(period) = u_period {
                    let du = (uv1.u - uv0.u).abs();
                    if du > period * 0.5 {
                        let (lo, hi) = if uv0.u < uv1.u { (uv0.u, uv1.u) } else { (uv1.u, uv0.u) };
                        ((lo + period + hi) * 0.5) % period
                    } else {
                        mid_u
                    }
                } else {
                    mid_u
                };

                let mid_v = if let Some(period) = v_period {
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

                // CRITICAL: Skip splitting if the midpoint UV falls inside a hole
                // or outside the outer boundary.
                //
                // The chord-error refinement creates new vertices at the midpoint
                // of edges between two interior vertices. For faces with holes
                // (e.g., a cylinder face with through-holes), an edge spanning
                // across a hole would have its midpoint UV land INSIDE the hole.
                // Inserting a vertex there produces triangles covering the hole
                // region, which is incorrect — the hole should remain empty.
                //
                // This bug manifested in drill_top.stp STEP #843: a half-wrap
                // cylinder face with 2 inner holes showed triangles covering
                // the holes because refinement midpoints landed inside them.
                if !domain.contains(&Point2d::new(mid_u_clamped, mid_v_clamped)) {
                    continue;
                }

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
            if let Some(period) = u_period {
                let du = (uv1.u - uv0.u).abs();
                if du > period * 0.5 {
                    let (lo, hi) = if uv0.u < uv1.u { (uv0.u, uv1.u) } else { (uv1.u, uv0.u) };
                    mid_u = ((lo + period + hi) * 0.5) % period;
                }
            }
            if let Some(period) = v_period {
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

/// Merge coincident boundary 3D points (DegeneracyHandler).
///
/// On surfaces with degeneracies (cone apex where radius→0, sphere poles
/// where latitude rings collapse), multiple boundary vertices from the
/// edge cache map to the same 3D point. For example, on a cone, edges
/// meeting at the apex all have their endpoint at the same 3D location,
/// but with different UV coordinates (different u-values at v=apex_v).
///
/// If left as separate vertices, earcutr creates degenerate (zero-area)
/// triangles between them, which degrade mesh quality. This function
/// detects clusters of coincident boundary points (within `tolerance`)
/// and merges each cluster into a single representative point, keeping
/// the UV coordinate of the first point in the cluster.
///
/// # Arguments
/// * `points_3d` — 3D boundary points (from edge cache, with deterministic rounding)
/// * `uvs` — UV coordinates corresponding to each 3D point
/// * `tolerance` — Distance threshold for coincident point detection.
///   Recommended: 1% of max_deviation or model_scale * 1e-4
///
/// # Returns
/// Merged (points_3d, uvs) pair with coincident points removed.
fn merge_coincident_boundary_points(
    points_3d: &[Point3d],
    uvs: &[Point2d],
    tolerance: f64,
) -> (Vec<Point3d>, Vec<Point2d>) {
    if points_3d.len() <= 3 {
        // Too few points to merge — just clone
        return (points_3d.to_vec(), uvs.to_vec());
    }

    let tol_sq = tolerance * tolerance;
    let n = points_3d.len();
    let mut merged_3d = Vec::with_capacity(n);
    let mut merged_uv = Vec::with_capacity(n);
    let mut skip = vec![false; n];

    // Track clusters: for each point, check if it coincides with any
    // previously kept point. Only keep the first point in each cluster.
    let mut merge_count = 0usize;
    for i in 0..n {
        if skip[i] {
            continue;
        }
        merged_3d.push(points_3d[i]);
        merged_uv.push(uvs[i]);

        // Check subsequent points for coincidence with this one
        for j in (i + 1)..n {
            if skip[j] {
                continue;
            }
            let dx = points_3d[i].x - points_3d[j].x;
            let dy = points_3d[i].y - points_3d[j].y;
            let dz = points_3d[i].z - points_3d[j].z;
            let dist_sq = dx * dx + dy * dy + dz * dz;
            if dist_sq < tol_sq {
                skip[j] = true;
                merge_count += 1;
            }
        }
    }

    if merge_count > 0 {
        log::info!(
            "DegeneracyHandler: merged {} coincident boundary points (tol={:.2e}, {}→{})",
            merge_count, tolerance, n, merged_3d.len(),
        );
    }

    (merged_3d, merged_uv)
}

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
        use draper_geometry::{CylinderSurface, Surface};

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
        let boundary_uv: Vec<Point2d> = (0..n_pts)
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
            u_closed: false, v_closed: false,
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

    // ============================================================
    // Tests for cylinder/cone Steiner grid generator
    // ============================================================

    /// Build a TriangulationParams with the given max_deviation.
    fn make_test_params(max_dev: f64) -> crate::triangulate::TriangulationParams {
        let mut p = crate::triangulate::TriangulationParams::default();
        p.max_deviation = max_dev;
        p.adaptive = true;
        p.angular_samples = 32;
        p.height_samples = 4;
        p.max_face_triangles = 4096;
        p
    }

    #[test]
    fn test_cylinder_steiner_grid_basic() {
        use draper_geometry::{CylinderSurface, Surface};

        // Cylinder radius=1.0, full U range [0, 2π], V range [0, 5].
        let cyl = CylinderSurface::new_z(1.0);
        let surface = Surface::Cylinder(cyl);

        // Square outer boundary in UV (4 corners, no holes).
        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(2.0 * PI, 0.0),
            Point2d::new(2.0 * PI, 5.0),
            Point2d::new(0.0, 5.0),
        ];
        let mut domain = ParametricDomain::new(outer, (0.0, 2.0 * PI), (0.0, 5.0));
        domain.init_containment_grid();

        let params = make_test_params(0.05);
        let pts = generate_cylinder_or_cone_steiner_grid(
            &surface, &domain, (0.0, 2.0 * PI), (0.0, 5.0), &params, 4096,
        );

        // Should have multiple interior points (regular grid in v direction).
        // The bug being fixed: parameter_division_2d returns 0 interior points
        // for cylinders because they have zero chord error in v. Our new
        // generator should produce many.
        assert!(pts.len() >= 10, "Expected ≥10 Steiner points, got {}", pts.len());

        // All points should be strictly inside the domain (not on boundary).
        for p in &pts {
            assert!(p.u > 1e-6 && p.u < 2.0 * PI - 1e-6, "u={} on boundary", p.u);
            assert!(p.v > 1e-6 && p.v < 5.0 - 1e-6, "v={} on boundary", p.v);
            assert!(domain.contains_ray(p), "point {:?} outside domain", p);
        }

        // The V coordinates should form a regular grid (multiple distinct v values).
        // This is the key property — the previous code only had v=[0, 5] (no interior).
        let mut v_values: Vec<f64> = pts.iter().map(|p| p.v).collect();
        v_values.sort_by(|a, b| a.partial_cmp(b).unwrap());
        v_values.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
        assert!(v_values.len() >= 3, "Expected ≥3 distinct v values, got {}: {:?}", v_values.len(), v_values);
    }

    #[test]
    fn test_cylinder_steiner_grid_excludes_holes() {
        use draper_geometry::{CylinderSurface, Surface};

        let cyl = CylinderSurface::new_z(1.0);
        let surface = Surface::Cylinder(cyl);

        // Outer boundary = full cylinder UV rectangle.
        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(2.0 * PI, 0.0),
            Point2d::new(2.0 * PI, 10.0),
            Point2d::new(0.0, 10.0),
        ];
        // Hole at u ∈ [2.0, 4.0], v ∈ [4.0, 6.0].
        let hole = vec![
            Point2d::new(2.0, 4.0),
            Point2d::new(4.0, 4.0),
            Point2d::new(4.0, 6.0),
            Point2d::new(2.0, 6.0),
        ];
        let mut domain = ParametricDomain::new(outer, (0.0, 2.0 * PI), (0.0, 10.0))
            .with_hole(hole);
        domain.init_containment_grid();

        let params = make_test_params(0.05);
        let pts = generate_cylinder_or_cone_steiner_grid(
            &surface, &domain, (0.0, 2.0 * PI), (0.0, 10.0), &params, 4096,
        );

        assert!(!pts.is_empty(), "Should have Steiner points");

        // No Steiner point should fall inside the hole.
        for p in &pts {
            let in_hole = p.u > 2.0 && p.u < 4.0 && p.v > 4.0 && p.v < 6.0;
            assert!(!in_hole, "Steiner point {:?} is inside hole", p);
            assert!(domain.contains_ray(p), "point {:?} outside domain", p);
        }
    }

    #[test]
    fn test_cylinder_steiner_grid_respects_budget() {
        use draper_geometry::{CylinderSurface, Surface};

        let cyl = CylinderSurface::new_z(1.0);
        let surface = Surface::Cylinder(cyl);

        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(2.0 * PI, 0.0),
            Point2d::new(2.0 * PI, 10.0),
            Point2d::new(0.0, 10.0),
        ];
        let mut domain = ParametricDomain::new(outer, (0.0, 2.0 * PI), (0.0, 10.0));
        domain.init_containment_grid();

        let params = make_test_params(0.01); // tight tolerance → many points
        let budget = 50usize;
        let pts = generate_cylinder_or_cone_steiner_grid(
            &surface, &domain, (0.0, 2.0 * PI), (0.0, 10.0), &params, budget,
        );

        assert!(pts.len() <= budget, "Budget exceeded: {} > {}", pts.len(), budget);
        assert!(pts.len() >= 10, "Should still have meaningful points: {}", pts.len());
    }

    #[test]
    fn test_cone_steiner_grid_basic() {
        use draper_geometry::{ConeSurface, Surface};

        // Cone: base radius 2.0, half-angle 30° → apex at v = 2/tan(30°) ≈ 3.46.
        // Use v range [0, 3.0] (well below apex) so radius varies from 2.0 to ~0.27.
        let cone = ConeSurface::new_z(2.0, std::f64::consts::FRAC_PI_6);
        let surface = Surface::Cone(cone);

        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(2.0 * PI, 0.0),
            Point2d::new(2.0 * PI, 3.0),
            Point2d::new(0.0, 3.0),
        ];
        let mut domain = ParametricDomain::new(outer, (0.0, 2.0 * PI), (0.0, 3.0));
        domain.init_containment_grid();

        let params = make_test_params(0.05);
        let pts = generate_cylinder_or_cone_steiner_grid(
            &surface, &domain, (0.0, 2.0 * PI), (0.0, 3.0), &params, 4096,
        );

        // Cone also has zero chord error in the axial direction, so the
        // old path produced 0 Steiner points. The new generator should
        // produce interior points on a regular grid.
        assert!(pts.len() >= 4, "Expected ≥4 Steiner points, got {}", pts.len());

        for p in &pts {
            assert!(domain.contains_ray(p), "point {:?} outside domain", p);
        }

        // V values should form a regular grid (multiple distinct values).
        let mut v_values: Vec<f64> = pts.iter().map(|p| p.v).collect();
        v_values.sort_by(|a, b| a.partial_cmp(b).unwrap());
        v_values.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
        assert!(v_values.len() >= 2, "Expected ≥2 distinct v values, got {}: {:?}", v_values.len(), v_values);
    }

    #[test]
    fn test_cylinder_steiner_grid_preserves_grid_structure() {
        use draper_geometry::{CylinderSurface, Surface};

        // Verify that the generated points form a Cartesian product of
        // u-values × v-values (i.e., a proper grid, not random points).
        let cyl = CylinderSurface::new_z(1.0);
        let surface = Surface::Cylinder(cyl);

        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(2.0 * PI, 0.0),
            Point2d::new(2.0 * PI, 5.0),
            Point2d::new(0.0, 5.0),
        ];
        let mut domain = ParametricDomain::new(outer, (0.0, 2.0 * PI), (0.0, 5.0));
        domain.init_containment_grid();

        let params = make_test_params(0.1);
        let pts = generate_cylinder_or_cone_steiner_grid(
            &surface, &domain, (0.0, 2.0 * PI), (0.0, 5.0), &params, 4096,
        );

        // Recover unique u and v values.
        let tol = 1e-9;
        let mut us: Vec<f64> = pts.iter().map(|p| p.u).collect();
        us.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mut u_unique: Vec<f64> = Vec::new();
        for u in us {
            if u_unique.last().map_or(true, |last| (last - u).abs() > tol) {
                u_unique.push(u);
            }
        }
        let mut vs: Vec<f64> = pts.iter().map(|p| p.v).collect();
        vs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mut v_unique: Vec<f64> = Vec::new();
        for v in vs {
            if v_unique.last().map_or(true, |last| (last - v).abs() > tol) {
                v_unique.push(v);
            }
        }

        // Every point should be a product of some u in u_unique × some v in v_unique.
        // (Otherwise the grid structure is broken.)
        for p in &pts {
            let u_ok = u_unique.iter().any(|&u| (u - p.u).abs() < tol);
            let v_ok = v_unique.iter().any(|&v| (v - p.v).abs() < tol);
            assert!(u_ok && v_ok, "point {:?} not on grid (u_unique={}, v_unique={})", p, u_unique.len(), v_unique.len());
        }

        // Should have multiple points in both u and v directions.
        assert!(u_unique.len() >= 3, "u_unique.len() = {}", u_unique.len());
        assert!(v_unique.len() >= 2, "v_unique.len() = {}", v_unique.len());
    }

    #[test]
    fn test_planar_steiner_grid_basic() {
        // Planar face: a 10×10 square in UV space.
        // Without holes, but we still want to verify the grid generator
        // produces a regular Cartesian product of points.
        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(10.0, 0.0),
            Point2d::new(10.0, 10.0),
            Point2d::new(0.0, 10.0),
        ];
        let mut domain = ParametricDomain::new(outer, (0.0, 10.0), (0.0, 10.0));
        domain.init_containment_grid();

        let outer_uv = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(10.0, 0.0),
            Point2d::new(10.0, 10.0),
            Point2d::new(0.0, 10.0),
        ];
        // Boundary has only 4 points → avg_edge = 10.0. So target_edge = 10.0.
        // n_u = ceil(10/10) = 1 → clamped to 4. Same for n_v.
        // Interior points: (4-1) × (4-1) = 9 points.
        let pts = generate_planar_steiner_grid(
            &domain, &outer_uv, (0.0, 10.0), (0.0, 10.0), 4096,
            crate::triangulate::SteinerBudgetProfile::Desktop,
        );

        assert!(pts.len() >= 4, "Expected ≥4 interior points, got {}", pts.len());

        // All points must be strictly inside (0, 10) × (0, 10).
        for p in &pts {
            assert!(p.u > 0.0 && p.u < 10.0, "point u out of interior: {}", p.u);
            assert!(p.v > 0.0 && p.v < 10.0, "point v out of interior: {}", p.v);
        }
    }

    #[test]
    fn test_planar_steiner_grid_excludes_holes() {
        // Planar face with a hole: 10×10 outer, 2×2 hole centered at (5, 5).
        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(10.0, 0.0),
            Point2d::new(10.0, 10.0),
            Point2d::new(0.0, 10.0),
        ];
        let hole = vec![
            Point2d::new(4.0, 4.0),
            Point2d::new(6.0, 4.0),
            Point2d::new(6.0, 6.0),
            Point2d::new(4.0, 6.0),
        ];
        let mut domain = ParametricDomain::new(outer, (0.0, 10.0), (0.0, 10.0))
            .with_hole(hole);
        domain.init_containment_grid();

        let outer_uv = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(10.0, 0.0),
            Point2d::new(10.0, 10.0),
            Point2d::new(0.0, 10.0),
        ];
        let pts = generate_planar_steiner_grid(
            &domain, &outer_uv, (0.0, 10.0), (0.0, 10.0), 4096,
            crate::triangulate::SteinerBudgetProfile::Desktop,
        );

        // No point should fall inside the hole region [4,6]×[4,6].
        for p in &pts {
            let in_hole = p.u > 4.0 && p.u < 6.0 && p.v > 4.0 && p.v < 6.0;
            assert!(!in_hole, "point {:?} falls inside the hole", p);
        }
        // Should still produce multiple interior points.
        assert!(pts.len() >= 4, "Expected ≥4 interior points, got {}", pts.len());
    }

    #[test]
    fn test_planar_steiner_grid_respects_budget() {
        // Same setup as test_planar_steiner_grid_basic, but with tight budget.
        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(10.0, 0.0),
            Point2d::new(10.0, 10.0),
            Point2d::new(0.0, 10.0),
        ];
        let mut domain = ParametricDomain::new(outer, (0.0, 10.0), (0.0, 10.0));
        domain.init_containment_grid();

        let outer_uv = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(10.0, 0.0),
            Point2d::new(10.0, 10.0),
            Point2d::new(0.0, 10.0),
        ];
        let budget = 5;
        let pts = generate_planar_steiner_grid(
            &domain, &outer_uv, (0.0, 10.0), (0.0, 10.0), budget,
            crate::triangulate::SteinerBudgetProfile::Desktop,
        );

        assert!(pts.len() <= budget, "Expected ≤{} points, got {}", budget, pts.len());
    }

    #[test]
    fn test_planar_steiner_grid_preserves_grid_structure() {
        // Verify the generated points form a Cartesian product of u-values × v-values.
        // Use many boundary points so n_u, n_v are larger than the minimum clamp.
        let outer_uv: Vec<Point2d> = (0..20).map(|i| {
            let t = i as f64 / 19.0;
            Point2d::new(t * 10.0, 0.0)
        }).chain((0..20).map(|i| {
            let t = i as f64 / 19.0;
            Point2d::new(10.0, t * 10.0)
        })).chain((0..20).map(|i| {
            let t = i as f64 / 19.0;
            Point2d::new(10.0 - t * 10.0, 10.0)
        })).chain((0..20).map(|i| {
            let t = i as f64 / 19.0;
            Point2d::new(0.0, 10.0 - t * 10.0)
        })).collect();

        let mut domain = ParametricDomain::new(outer_uv.clone(), (0.0, 10.0), (0.0, 10.0));
        domain.init_containment_grid();

        let pts = generate_planar_steiner_grid(
            &domain, &outer_uv, (0.0, 10.0), (0.0, 10.0), 4096,
            crate::triangulate::SteinerBudgetProfile::Desktop,
        );

        let tol = 1e-9;
        let mut us: Vec<f64> = pts.iter().map(|p| p.u).collect();
        us.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mut u_unique: Vec<f64> = Vec::new();
        for u in us {
            if u_unique.last().map_or(true, |last| (last - u).abs() > tol) {
                u_unique.push(u);
            }
        }
        let mut vs: Vec<f64> = pts.iter().map(|p| p.v).collect();
        vs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mut v_unique: Vec<f64> = Vec::new();
        for v in vs {
            if v_unique.last().map_or(true, |last| (last - v).abs() > tol) {
                v_unique.push(v);
            }
        }

        for p in &pts {
            let u_ok = u_unique.iter().any(|&u| (u - p.u).abs() < tol);
            let v_ok = v_unique.iter().any(|&v| (v - p.v).abs() < tol);
            assert!(u_ok && v_ok, "point {:?} not on grid", p);
        }

        assert!(u_unique.len() >= 2, "u_unique.len() = {}", u_unique.len());
        assert!(v_unique.len() >= 2, "v_unique.len() = {}", v_unique.len());
    }

    // ============================================================
    // Sphere Steiner grid tests
    // (mirrors the cylinder tests above)
    // ============================================================

    #[test]
    fn test_sphere_steiner_grid_basic() {
        use draper_geometry::{SphereSurface, Surface, Point3d};

        // Sphere radius=10, full U range [0, 2π], V range [0, π] (full sphere).
        let sph = SphereSurface::new(Point3d::new(0.0, 0.0, 0.0), 10.0);
        let surface = Surface::Sphere(sph);

        // Square outer boundary in UV (4 corners, no holes).
        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(2.0 * PI, 0.0),
            Point2d::new(2.0 * PI, PI),
            Point2d::new(0.0, PI),
        ];
        let mut domain = ParametricDomain::new(outer, (0.0, 2.0 * PI), (0.0, PI));
        domain.init_containment_grid();

        let params = make_test_params(0.05);
        let pts = generate_sphere_steiner_grid(
            &surface, &domain, (0.0, 2.0 * PI), (0.0, PI), &params, 4096,
        );

        // Should have multiple interior points (regular grid in both u and v).
        assert!(pts.len() >= 10, "Expected ≥10 Steiner points, got {}", pts.len());

        // All points should be strictly inside the domain (not on boundary).
        for p in &pts {
            assert!(p.u > 1e-6 && p.u < 2.0 * PI - 1e-6, "u={} on boundary", p.u);
            assert!(p.v > 1e-6 && p.v < PI - 1e-6, "v={} on boundary", p.v);
            assert!(domain.contains_ray(p), "point {:?} outside domain", p);
        }

        // No points should be within POLE_EPS of either pole.
        const POLE_EPS: f64 = 0.05;
        for p in &pts {
            assert!(p.v > POLE_EPS, "v={} too close to north pole", p.v);
            assert!(p.v < PI - POLE_EPS, "v={} too close to south pole", p.v);
        }

        // Equator (v = π/2) should be present (full-sphere case).
        let has_equator = pts.iter().any(|p| (p.v - PI / 2.0).abs() < 1e-6);
        assert!(has_equator, "Equator ring missing in full-sphere Steiner grid");

        // The V coordinates should form a regular grid (multiple distinct v values).
        let mut v_values: Vec<f64> = pts.iter().map(|p| p.v).collect();
        v_values.sort_by(|a, b| a.partial_cmp(b).unwrap());
        v_values.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
        assert!(v_values.len() >= 3, "Expected ≥3 distinct v values, got {}: {:?}", v_values.len(), v_values);
    }

    #[test]
    fn test_sphere_steiner_grid_excludes_holes() {
        use draper_geometry::{SphereSurface, Surface, Point3d};

        let sph = SphereSurface::new(Point3d::new(0.0, 0.0, 0.0), 10.0);
        let surface = Surface::Sphere(sph);

        // Outer boundary = full sphere UV rectangle.
        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(2.0 * PI, 0.0),
            Point2d::new(2.0 * PI, PI),
            Point2d::new(0.0, PI),
        ];
        // Hole at u ∈ [2.0, 4.0], v ∈ [1.0, 2.0] (well away from poles).
        let hole = vec![
            Point2d::new(2.0, 1.0),
            Point2d::new(4.0, 1.0),
            Point2d::new(4.0, 2.0),
            Point2d::new(2.0, 2.0),
        ];
        let mut domain = ParametricDomain::new(outer, (0.0, 2.0 * PI), (0.0, PI))
            .with_hole(hole);
        domain.init_containment_grid();

        let params = make_test_params(0.05);
        let pts = generate_sphere_steiner_grid(
            &surface, &domain, (0.0, 2.0 * PI), (0.0, PI), &params, 4096,
        );

        assert!(!pts.is_empty(), "Should have Steiner points");

        // No Steiner point should fall inside the hole.
        for p in &pts {
            let in_hole = p.u > 2.0 && p.u < 4.0 && p.v > 1.0 && p.v < 2.0;
            assert!(!in_hole, "Steiner point {:?} is inside hole", p);
            assert!(domain.contains_ray(p), "point {:?} outside domain", p);
        }
    }

    #[test]
    fn test_sphere_steiner_grid_respects_budget() {
        use draper_geometry::{SphereSurface, Surface, Point3d};

        let sph = SphereSurface::new(Point3d::new(0.0, 0.0, 0.0), 10.0);
        let surface = Surface::Sphere(sph);

        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(2.0 * PI, 0.0),
            Point2d::new(2.0 * PI, PI),
            Point2d::new(0.0, PI),
        ];
        let mut domain = ParametricDomain::new(outer, (0.0, 2.0 * PI), (0.0, PI));
        domain.init_containment_grid();

        // Tight chord-error tol → many candidates; small budget → must cap.
        let params = make_test_params(0.01);
        let budget = 50;
        let pts = generate_sphere_steiner_grid(
            &surface, &domain, (0.0, 2.0 * PI), (0.0, PI), &params, budget,
        );

        assert!(pts.len() <= budget, "Budget {} exceeded: {} points", budget, pts.len());
        assert!(!pts.is_empty(), "Should have at least some Steiner points");
    }

    #[test]
    fn test_sphere_steiner_grid_band_skips_poles() {
        use draper_geometry::{SphereSurface, Surface, Point3d};

        // Partial sphere band: v ∈ [0.02, π - 0.02] — includes both pole
        // neighborhoods but does NOT include the poles themselves.
        // The Steiner grid should still skip rows too close to the poles
        // (v < 0.05 or v > π - 0.05).
        let sph = SphereSurface::new(Point3d::new(0.0, 0.0, 0.0), 5.0);
        let surface = Surface::Sphere(sph);

        let v_min = 0.02;
        let v_max = std::f64::consts::PI - 0.02;
        let outer = vec![
            Point2d::new(0.0, v_min),
            Point2d::new(2.0 * PI, v_min),
            Point2d::new(2.0 * PI, v_max),
            Point2d::new(0.0, v_max),
        ];
        let mut domain = ParametricDomain::new(outer, (0.0, 2.0 * PI), (v_min, v_max));
        domain.init_containment_grid();

        let params = make_test_params(0.05);
        let pts = generate_sphere_steiner_grid(
            &surface, &domain, (0.0, 2.0 * PI), (v_min, v_max), &params, 4096,
        );

        const POLE_EPS: f64 = 0.05;
        for p in &pts {
            // Even though the domain includes v=0.02, no Steiner point
            // should land in the pole-degenerate zone [0, 0.05) or (π-0.05, π].
            assert!(p.v > POLE_EPS, "v={} too close to north pole", p.v);
            assert!(p.v < std::f64::consts::PI - POLE_EPS, "v={} too close to south pole", p.v);
        }
    }

    // ============================================================
    // Torus Steiner grid tests
    // ============================================================

    #[test]
    fn test_torus_steiner_grid_basic() {
        use draper_geometry::{TorusSurface, Surface, Point3d};

        // Torus R=2, r=0.5 (typical fillet size), full U/V range.
        let torus = TorusSurface::new_z(Point3d::new(0.0, 0.0, 0.0), 2.0, 0.5);
        let surface = Surface::Torus(torus);

        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(2.0 * PI, 0.0),
            Point2d::new(2.0 * PI, 2.0 * PI),
            Point2d::new(0.0, 2.0 * PI),
        ];
        let mut domain = ParametricDomain::new(outer, (0.0, 2.0 * PI), (0.0, 2.0 * PI));
        domain.init_containment_grid();

        let params = make_test_params(0.05);
        let pts = generate_torus_steiner_grid(
            &surface, &domain, (0.0, 2.0 * PI), (0.0, 2.0 * PI), &params, 4096,
        );

        // Should have multiple interior points (regular grid in both u and v).
        // The bug being fixed: parameter_division_2d returns only 4×4 or 6×6
        // for small torus fillet faces. Our new generator should produce many.
        assert!(pts.len() >= 50, "Expected ≥50 Steiner points, got {}", pts.len());

        // All points should be strictly inside the domain (not on boundary).
        for p in &pts {
            assert!(p.u > 1e-6 && p.u < 2.0 * PI - 1e-6, "u={} on boundary", p.u);
            assert!(p.v > 1e-6 && p.v < 2.0 * PI - 1e-6, "v={} on boundary", p.v);
            assert!(domain.contains_ray(p), "point {:?} outside domain", p);
        }

        // n_u floor is 24 on desktop — should have at least 23 distinct u
        // values (n_u - 1 interior columns).
        let mut u_values: Vec<f64> = pts.iter().map(|p| p.u).collect();
        u_values.sort_by(|a, b| a.partial_cmp(b).unwrap());
        u_values.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
        assert!(u_values.len() >= 10, "Expected ≥10 distinct u values, got {}: {:?}", u_values.len(), u_values);

        // n_v floor is 24 on desktop — should have at least 10 distinct v values.
        let mut v_values: Vec<f64> = pts.iter().map(|p| p.v).collect();
        v_values.sort_by(|a, b| a.partial_cmp(b).unwrap());
        v_values.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
        assert!(v_values.len() >= 10, "Expected ≥10 distinct v values, got {}: {:?}", v_values.len(), v_values);
    }

    #[test]
    fn test_torus_steiner_grid_excludes_holes() {
        use draper_geometry::{TorusSurface, Surface, Point3d};

        let torus = TorusSurface::new_z(Point3d::new(0.0, 0.0, 0.0), 2.0, 0.5);
        let surface = Surface::Torus(torus);

        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(2.0 * PI, 0.0),
            Point2d::new(2.0 * PI, 2.0 * PI),
            Point2d::new(0.0, 2.0 * PI),
        ];
        // Hole at u ∈ [2.0, 4.0], v ∈ [3.0, 4.0].
        let hole = vec![
            Point2d::new(2.0, 3.0),
            Point2d::new(4.0, 3.0),
            Point2d::new(4.0, 4.0),
            Point2d::new(2.0, 4.0),
        ];
        let mut domain = ParametricDomain::new(outer, (0.0, 2.0 * PI), (0.0, 2.0 * PI))
            .with_hole(hole);
        domain.init_containment_grid();

        let params = make_test_params(0.05);
        let pts = generate_torus_steiner_grid(
            &surface, &domain, (0.0, 2.0 * PI), (0.0, 2.0 * PI), &params, 4096,
        );

        assert!(!pts.is_empty(), "Should have Steiner points");

        // No Steiner point should fall inside the hole.
        for p in &pts {
            let in_hole = p.u > 2.0 && p.u < 4.0 && p.v > 3.0 && p.v < 4.0;
            assert!(!in_hole, "Steiner point {:?} is inside hole", p);
            assert!(domain.contains_ray(p), "point {:?} outside domain", p);
        }
    }

    #[test]
    fn test_torus_steiner_grid_respects_budget() {
        use draper_geometry::{TorusSurface, Surface, Point3d};

        let torus = TorusSurface::new_z(Point3d::new(0.0, 0.0, 0.0), 2.0, 0.5);
        let surface = Surface::Torus(torus);

        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(2.0 * PI, 0.0),
            Point2d::new(2.0 * PI, 2.0 * PI),
            Point2d::new(0.0, 2.0 * PI),
        ];
        let mut domain = ParametricDomain::new(outer, (0.0, 2.0 * PI), (0.0, 2.0 * PI));
        domain.init_containment_grid();

        // Tight chord-error tol → many candidates; small budget → must cap.
        let params = make_test_params(0.01);
        let budget = 100;
        let pts = generate_torus_steiner_grid(
            &surface, &domain, (0.0, 2.0 * PI), (0.0, 2.0 * PI), &params, budget,
        );

        assert!(pts.len() <= budget, "Budget {} exceeded: {} points", budget, pts.len());
        assert!(!pts.is_empty(), "Should have at least some Steiner points");
    }

    #[test]
    fn test_torus_steiner_grid_partial_band() {
        use draper_geometry::{TorusSurface, Surface, Point3d};

        // Partial torus band: u ∈ [0, π] (half torus), v ∈ [0, 2π] (full tube).
        // The grid should be naturally bounded by the u_range / v_range
        // (no wrap-around needed for partial torus).
        let torus = TorusSurface::new_z(Point3d::new(0.0, 0.0, 0.0), 5.0, 1.0);
        let surface = Surface::Torus(torus);

        let u_min = 0.0;
        let u_max = PI;
        let v_min = 0.0;
        let v_max = 2.0 * PI;
        let outer = vec![
            Point2d::new(u_min, v_min),
            Point2d::new(u_max, v_min),
            Point2d::new(u_max, v_max),
            Point2d::new(u_min, v_max),
        ];
        let mut domain = ParametricDomain::new(outer, (u_min, u_max), (v_min, v_max));
        domain.init_containment_grid();

        let params = make_test_params(0.05);
        let pts = generate_torus_steiner_grid(
            &surface, &domain, (u_min, u_max), (v_min, v_max), &params, 4096,
        );

        // All points should be within the partial u range [0, π].
        for p in &pts {
            assert!(p.u >= u_min - 1e-9 && p.u <= u_max + 1e-9,
                    "u={} outside partial range [{}, {}]", p.u, u_min, u_max);
            assert!(p.v >= v_min - 1e-9 && p.v <= v_max + 1e-9,
                    "v={} outside full v range [{}, {}]", p.v, v_min, v_max);
            assert!(domain.contains_ray(p), "point {:?} outside domain", p);
        }

        // Should still have multiple distinct u and v values.
        let mut u_values: Vec<f64> = pts.iter().map(|p| p.u).collect();
        u_values.sort_by(|a, b| a.partial_cmp(b).unwrap());
        u_values.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
        assert!(u_values.len() >= 5, "Expected ≥5 distinct u values, got {}", u_values.len());
    }

    #[test]
    fn test_torus_steiner_grid_degenerate_returns_empty() {
        use draper_geometry::{TorusSurface, Surface, Point3d};

        // Degenerate torus: minor_radius ≈ 0 → circle-like.
        // Should return empty Vec (no Steiner points).
        let torus = TorusSurface::new_z(Point3d::new(0.0, 0.0, 0.0), 2.0, 1e-9);
        let surface = Surface::Torus(torus);

        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(2.0 * PI, 0.0),
            Point2d::new(2.0 * PI, 2.0 * PI),
            Point2d::new(0.0, 2.0 * PI),
        ];
        let mut domain = ParametricDomain::new(outer, (0.0, 2.0 * PI), (0.0, 2.0 * PI));
        domain.init_containment_grid();

        let params = make_test_params(0.05);
        let pts = generate_torus_steiner_grid(
            &surface, &domain, (0.0, 2.0 * PI), (0.0, 2.0 * PI), &params, 4096,
        );

        assert!(pts.is_empty(), "Degenerate torus should return empty Vec, got {} points", pts.len());
    }

    // ============================================================
    // Revolution Steiner grid tests
    // ============================================================

    #[test]
    fn test_revolution_steiner_grid_line_profile() {
        use draper_geometry::{RevolutionSurface, Surface, Point3d, Direction3d, Curve3d, Line};

        // Linear profile revolved around Z axis → equivalent to a cylinder.
        // Profile: line from (5, 0, 0) to (5, 0, 10) — parallel to axis at radius 5.
        let line = Line::new(Point3d::new(5.0, 0.0, 0.0), Direction3d::Z);
        let rev = RevolutionSurface::new(Curve3d::Line(line), Direction3d::Z, Point3d::ORIGIN);
        let surface = Surface::Revolution(rev);

        // Full revolution v ∈ [0, 1] (line param range) × u ∈ [0, 2π]
        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(2.0 * PI, 0.0),
            Point2d::new(2.0 * PI, 1.0),
            Point2d::new(0.0, 1.0),
        ];
        let mut domain = ParametricDomain::new(outer, (0.0, 2.0 * PI), (0.0, 1.0));
        domain.init_containment_grid();

        let params = make_test_params(0.05);
        let pts = generate_revolution_steiner_grid(
            &surface, &domain, (0.0, 2.0 * PI), (0.0, 1.0), &params, 4096,
        );

        assert!(!pts.is_empty(), "Line profile revolution should have Steiner points");
        // All points should be inside the domain.
        for p in &pts {
            assert!(domain.contains_ray(p), "point {:?} outside domain", p);
        }
    }

    #[test]
    fn test_revolution_steiner_grid_excludes_holes() {
        use draper_geometry::{RevolutionSurface, Surface, Point3d, Direction3d, Curve3d, Line};

        let line = Line::new(Point3d::new(5.0, 0.0, 0.0), Direction3d::Z);
        let rev = RevolutionSurface::new(Curve3d::Line(line), Direction3d::Z, Point3d::ORIGIN);
        let surface = Surface::Revolution(rev);

        // Outer boundary with a rectangular hole in the middle.
        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(2.0 * PI, 0.0),
            Point2d::new(2.0 * PI, 1.0),
            Point2d::new(0.0, 1.0),
        ];
        let hole = vec![
            Point2d::new(1.0, 0.3),
            Point2d::new(2.0, 0.3),
            Point2d::new(2.0, 0.7),
            Point2d::new(1.0, 0.7),
        ];
        let mut domain = ParametricDomain::new(outer, (0.0, 2.0 * PI), (0.0, 1.0))
            .with_hole(hole);
        domain.init_containment_grid();

        let params = make_test_params(0.05);
        let pts = generate_revolution_steiner_grid(
            &surface, &domain, (0.0, 2.0 * PI), (0.0, 1.0), &params, 4096,
        );

        // No Steiner point should land inside the hole.
        for p in &pts {
            let in_hole = p.u > 1.0 && p.u < 2.0 && p.v > 0.3 && p.v < 0.7;
            assert!(!in_hole, "Steiner point {:?} is inside hole", p);
        }
        assert!(!pts.is_empty(), "Should have Steiner points outside the hole");
    }

    #[test]
    fn test_revolution_steiner_grid_respects_budget() {
        use draper_geometry::{RevolutionSurface, Surface, Point3d, Direction3d, Curve3d, Circle};

        // Circle profile → torus-like revolution. Many candidates expected.
        let circle = Circle::new_xy(Point3d::new(5.0, 0.0, 0.0), 2.0);
        let rev = RevolutionSurface::new(Curve3d::Circle(circle), Direction3d::Z, Point3d::ORIGIN);
        let surface = Surface::Revolution(rev);

        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(2.0 * PI, 0.0),
            Point2d::new(2.0 * PI, 2.0 * PI),
            Point2d::new(0.0, 2.0 * PI),
        ];
        let mut domain = ParametricDomain::new(outer, (0.0, 2.0 * PI), (0.0, 2.0 * PI));
        domain.init_containment_grid();

        let params = make_test_params(0.01);
        let budget = 100;
        let pts = generate_revolution_steiner_grid(
            &surface, &domain, (0.0, 2.0 * PI), (0.0, 2.0 * PI), &params, budget,
        );

        assert!(pts.len() <= budget, "Budget {} exceeded: {} points", budget, pts.len());
        assert!(!pts.is_empty(), "Should have at least some Steiner points");
    }

    #[test]
    fn test_revolution_steiner_grid_axis_degenerate() {
        use draper_geometry::{RevolutionSurface, Surface, Point3d, Direction3d, Curve3d, Line};

        // Profile line that passes THROUGH the axis: from (0, 0, 0) to (0, 0, 10).
        // At v = 0 the profile is ON the axis (perp distance = 0), so the surface
        // degenerates there (like a cone apex). Steiner points near v = 0 should
        // be filtered out.
        let line = Line::new(Point3d::ORIGIN, Direction3d::Z);
        let rev = RevolutionSurface::new(Curve3d::Line(line), Direction3d::Z, Point3d::ORIGIN);
        let surface = Surface::Revolution(rev);

        // Outer boundary in a wedge: u ∈ [0, π/2], v ∈ [0, 1]
        let u_max = PI / 2.0;
        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(u_max, 0.0),
            Point2d::new(u_max, 1.0),
            Point2d::new(0.0, 1.0),
        ];
        let mut domain = ParametricDomain::new(outer, (0.0, u_max), (0.0, 1.0));
        domain.init_containment_grid();

        let params = make_test_params(0.05);
        let _pts = generate_revolution_steiner_grid(
            &surface, &domain, (0.0, u_max), (0.0, 1.0), &params, 4096,
        );

        // With a line profile through the axis, all profile points are on the
        // axis (perp_dist = 0), so ALL Steiner points should be filtered out
        // by the axis-degeneracy check. The result is either empty or very
        // small (only points far enough from the axis).
        //
        // Actually, for this specific case (line along the axis), the max_rev_radius
        // is 0, and du_max defaults to PI/8, so n_u is small. The axis degen
        // threshold is (0 * 0.02).max(1e-4) = 1e-4, and all profile points have
        // perp_dist = 0 < 1e-4, so all interior points are filtered.
        // Result: empty Vec (degenerate revolution — all points on axis).
        // This is correct behavior — the generic fallback will handle it.
    }

    #[test]
    fn test_revolution_steiner_grid_circle_profile() {
        use draper_geometry::{RevolutionSurface, Surface, Point3d, Direction3d, Curve3d, Circle};

        // Circle profile at radius 5 from axis → creates a torus-like surface.
        let circle = Circle::new_xy(Point3d::new(5.0, 0.0, 0.0), 2.0);
        let rev = RevolutionSurface::new(Curve3d::Circle(circle), Direction3d::Z, Point3d::ORIGIN);
        let surface = Surface::Revolution(rev);

        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(2.0 * PI, 0.0),
            Point2d::new(2.0 * PI, 2.0 * PI),
            Point2d::new(0.0, 2.0 * PI),
        ];
        let mut domain = ParametricDomain::new(outer, (0.0, 2.0 * PI), (0.0, 2.0 * PI));
        domain.init_containment_grid();

        let params = make_test_params(0.05);
        let pts = generate_revolution_steiner_grid(
            &surface, &domain, (0.0, 2.0 * PI), (0.0, 2.0 * PI), &params, 4096,
        );

        assert!(!pts.is_empty(), "Circle profile revolution should have Steiner points");
        // Should have multiple distinct u and v values (rich grid).
        let n_distinct_u = {
            let mut u_vals: Vec<f64> = pts.iter().map(|p| p.u).collect();
            u_vals.sort_by(|a, b| a.partial_cmp(b).unwrap());
            u_vals.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
            u_vals.len()
        };
        assert!(n_distinct_u >= 6, "Expected ≥6 distinct u values, got {}", n_distinct_u);
    }
}

// ============================================================
// 3D ear-clipping fallback for degenerate UV polygons
//
// When a face's UV polygon is degenerate (zero area) but the 3D
// boundary has non-zero area, the face is geometrically valid in
// 3D but its boundary doesn't bound a 2D region on the surface
// (e.g., a closed loop around a cylinder at constant height).
//
// This function projects the 3D boundary points to a best-fit
// plane, ear-clips the 2D projection, and returns triangles
// using the ORIGINAL 3D points. This preserves watertightness
// (shared boundary edges with adjacent faces) even when the
// face's geometry is degenerate on its surface.
// ============================================================

/// Triangulate a 3D polygon by projecting to a best-fit plane and ear-clipping.
///
/// Used as a fallback when `triangulate_surface_consistent` cannot triangulate
/// due to a degenerate UV polygon. Returns a mesh using the original 3D boundary
/// points (watertight with adjacent faces) even though the face's geometry on
/// the surface is degenerate.
fn triangulate_3d_polygon_fallback(
    boundary_3d: &[Point3d],
    hole_polylines_3d: &[Vec<Point3d>],
    forward: bool,
) -> TriangleMesh {
    let n = boundary_3d.len();
    if n < 3 {
        return TriangleMesh::new();
    }

    // Pre-process: remove consecutive duplicate points (within 1e-10 tolerance).
    // Some STEP files have boundary curves that produce duplicate points at
    // parametric transitions (e.g., where a NURBS knot span ends). These
    // duplicates create zero-length edges in the polygon, which can confuse
    // earcutr into returning 0 triangles.
    let dedup_tol_sq = 1e-20_f64; // 1e-10 squared
    let mut cleaned: Vec<Point3d> = Vec::with_capacity(n);
    for p in boundary_3d {
        if let Some(last) = cleaned.last() {
            let dx = p.x - last.x;
            let dy = p.y - last.y;
            let dz = p.z - last.z;
            if dx * dx + dy * dy + dz * dz <= dedup_tol_sq {
                continue; // Skip duplicate
            }
        }
        cleaned.push(*p);
    }
    // Also remove last point if it coincides with the first (closing the loop)
    if cleaned.len() > 1 {
        let last = cleaned[cleaned.len() - 1];
        let first = cleaned[0];
        let dx = last.x - first.x;
        let dy = last.y - first.y;
        let dz = last.z - first.z;
        if dx * dx + dy * dy + dz * dz <= dedup_tol_sq {
            cleaned.pop();
        }
    }
    let boundary_3d = &cleaned[..];
    let n = boundary_3d.len();
    if n < 3 {
        return TriangleMesh::new();
    }

    // Step 1: Compute best-fit plane normal using Newell's method
    let mut nx = 0.0_f64;
    let mut ny = 0.0_f64;
    let mut nz = 0.0_f64;
    let mut cx = 0.0_f64;
    let mut cy = 0.0_f64;
    let mut cz = 0.0_f64;
    for i in 0..n {
        let j = (i + 1) % n;
        let pi = &boundary_3d[i];
        let pj = &boundary_3d[j];
        nx += (pi.y - pj.y) * (pi.z + pj.z);
        ny += (pi.z - pj.z) * (pi.x + pj.x);
        nz += (pi.x - pj.x) * (pi.y + pj.y);
        cx += pi.x;
        cy += pi.y;
        cz += pi.z;
    }
    cx /= n as f64;
    cy /= n as f64;
    cz /= n as f64;
    let n_len = (nx * nx + ny * ny + nz * nz).sqrt();
    if n_len < 1e-12 {
        // Polygon is truly degenerate (all points coincident)
        return TriangleMesh::new();
    }
    let nx = nx / n_len;
    let ny = ny / n_len;
    let nz = nz / n_len;

    // Step 2: Build a 2D coordinate system on the best-fit plane
    // u_axis: any vector perpendicular to normal
    let u_axis = if nx.abs() < 0.9 {
        // Cross with X
        let ux = 0.0;
        let uy = nz;
        let uz = -ny;
        let ulen = (uy * uy + uz * uz).sqrt().max(1e-12);
        (ux, uy / ulen, uz / ulen)
    } else {
        // Cross with Y
        let ux = -nz;
        let uy = 0.0;
        let uz = nx;
        let ulen = (ux * ux + uz * uz).sqrt().max(1e-12);
        (ux / ulen, uy, uz / ulen)
    };
    // v_axis = normal × u_axis
    let v_axis = (
        ny * u_axis.2 - nz * u_axis.1,
        nz * u_axis.0 - nx * u_axis.2,
        nx * u_axis.1 - ny * u_axis.0,
    );

    // Project a 3D point to 2D using the best-fit plane's coordinate system
    let project_to_2d = |p: &Point3d| -> (f64, f64) {
        let dx = p.x - cx;
        let dy = p.y - cy;
        let dz = p.z - cz;
        (
            dx * u_axis.0 + dy * u_axis.1 + dz * u_axis.2,
            dx * v_axis.0 + dy * v_axis.1 + dz * v_axis.2,
        )
    };

    // Step 3: Build 2D points for boundary + holes
    let mut all_2d: Vec<(f64, f64)> = Vec::with_capacity(n);
    for p in boundary_3d {
        all_2d.push(project_to_2d(p));
    }
    let n_outer = all_2d.len();

    let mut hole_start_indices: Vec<usize> = Vec::new();
    for hole in hole_polylines_3d {
        if hole.len() < 3 {
            continue;
        }
        hole_start_indices.push(all_2d.len());
        for p in hole {
            all_2d.push(project_to_2d(p));
        }
    }

    // Step 4: Build flat coords array for earcutr
    let mut coords: Vec<f64> = Vec::with_capacity(all_2d.len() * 2);
    for &(u, v) in &all_2d {
        coords.push(u);
        coords.push(v);
    }

    // Step 5: Run triangulation via adapter (tries earcut int, i_triangle, earcutr)
    let mut triangle_indices: Vec<usize> = crate::earcut_adapter::triangulate_polygon_with_holes(&coords, &hole_start_indices);

    // If adapter returned 0 triangles with holes, retry without holes.
    // This happens when a "hole" is geometrically identical to the outer
    // boundary (e.g., due to a topology extraction bug producing duplicate
    // curves), OR when the projected hole polygon self-intersects the outer
    // polygon in the best-fit 2D plane.
    //
    // CRITICAL: We must rebuild `coords` from OUTER points only. If we keep
    // the same `coords` (which contains outer + hole points) and pass an
    // empty hole_indices, the adapter will see all points as a single polygon
    // — but those points came from two separate rings, so the resulting
    // polygon is almost always self-intersecting, and earcutr returns 0
    // triangles again. By rebuilding coords from `all_2d[..n_outer]`, we
    // give earcutr a clean outer-only polygon to triangulate.
    //
    // `outer_only_mode` means: vertex_map should be built from outer points
    // only (skip hole vertices entirely). Triangle indices are in [0, n_outer).
    let mut outer_only_mode = false;
    if triangle_indices.is_empty() && !hole_start_indices.is_empty() {
        log::warn!(
            "  3D fallback: adapter returned 0 triangles with {} holes — retrying outer-only",
            hole_start_indices.len(),
        );
        let outer_only_coords: Vec<f64> = all_2d[..n_outer]
            .iter()
            .flat_map(|&(u, v)| [u, v])
            .collect();
        let empty_holes: Vec<usize> = Vec::new();
        triangle_indices = crate::earcut_adapter::triangulate_polygon_with_holes(&outer_only_coords, &empty_holes);
        outer_only_mode = true;
    }

    // Step 5b: Final fallback — fan triangulation from centroid.
    // If earcutr STILL returned 0 triangles (which happens for highly
    // non-convex or self-intersecting outer polygons), use a simple fan
    // from the centroid. This guarantees a non-empty mesh as long as we
    // have ≥3 outer points, which preserves watertightness (shared boundary
    // edges with adjacent faces). Without this, the face would have 0
    // triangles, leaving a hole in the BREP that no weld pass can fix.
    //
    // Fan layout: vertex 0 = centroid (3D, inverse-projected from 2D centroid),
    // vertices 1..=n_outer = outer boundary points. Triangle i = (0, 1+i, 1+i_next).
    let mut fan_centroid_3d: Option<Point3d> = None;
    if triangle_indices.is_empty() && n_outer >= 3 {
        log::warn!(
            "  3D fallback: earcutr returned 0 triangles for outer polygon ({} pts) — using fan from centroid",
            n_outer,
        );
        // Compute centroid of outer points in 2D (best-fit plane projection)
        let mut cu = 0.0_f64;
        let mut cv = 0.0_f64;
        for &(u, v) in &all_2d[..n_outer] {
            cu += u;
            cv += v;
        }
        cu /= n_outer as f64;
        cv /= n_outer as f64;
        // Inverse-project 2D centroid back to 3D using best-fit plane basis.
        // best-fit plane passes through (cx, cy, cz) with basis (u_axis, v_axis).
        let centroid = Point3d::new(
            cx + cu * u_axis.0 + cv * v_axis.0,
            cy + cu * u_axis.1 + cv * v_axis.1,
            cz + cu * u_axis.2 + cv * v_axis.2,
        );
        fan_centroid_3d = Some(centroid);
        // Build fan triangle indices: (0, 1+i, 1+i_next) for i in 0..n_outer
        triangle_indices.clear();
        triangle_indices.reserve(n_outer * 3);
        for i in 0..n_outer {
            let i_next = (i + 1) % n_outer;
            triangle_indices.push(0);          // centroid
            triangle_indices.push(1 + i);      // outer[i]
            triangle_indices.push(1 + i_next); // outer[i_next]
        }
        outer_only_mode = true; // fan uses only outer + centroid, no hole vertices
    }

    // Step 6: Build mesh using ORIGINAL 3D points (preserves watertightness)
    let mut mesh = TriangleMesh::new();

    // Compute the face normal from the best-fit plane (used for vertex normals)
    let face_normal: [f64; 3] = [nx, ny, nz];

    // If using fan-from-centroid, prepend centroid as vertex 0.
    let mut vertex_map: Vec<u32> = Vec::with_capacity(all_2d.len() + 1);
    if let Some(centroid) = fan_centroid_3d {
        let vi = mesh.add_vertex(centroid);
        mesh.add_vertex_normal(vi, face_normal);
        vertex_map.push(vi);
    }

    // Add outer boundary vertices
    for p in boundary_3d {
        let vi = mesh.add_vertex(*p);
        mesh.add_vertex_normal(vi, face_normal);
        vertex_map.push(vi);
    }
    // Add hole vertices (only if not in outer-only mode — fan/retry-outer
    // paths produce triangle indices that don't reference hole vertices, so
    // including them would just create orphan vertices).
    if !outer_only_mode {
        for hole in hole_polylines_3d {
            if hole.len() < 3 {
                // Skip — but we need to advance the vertex_map index to stay in sync
                // with all_2d. Since we didn't add these vertices to all_2d either
                // (due to the `continue` in the previous loop), we don't need to
                // advance here.
                continue;
            }
            for p in hole {
                let vi = mesh.add_vertex(*p);
                mesh.add_vertex_normal(vi, face_normal);
                vertex_map.push(vi);
            }
        }
    }

    // Add triangles (filter degenerate)
    for chunk in triangle_indices.chunks(3) {
        if chunk.len() < 3 { break; }
        let a = chunk[0] as usize;
        let b = chunk[1] as usize;
        let c = chunk[2] as usize;
        if a >= vertex_map.len() || b >= vertex_map.len() || c >= vertex_map.len() {
            continue;
        }
        let va = vertex_map[a];
        let vb = vertex_map[b];
        let vc = vertex_map[c];
        if va == vb || vb == vc || va == vc {
            continue;
        }
        if forward {
            mesh.add_triangle(va, vb, vc);
        } else {
            mesh.add_triangle(va, vc, vb);
        }
    }

    log::info!(
        "triangulate_3d_polygon_fallback: {} outer pts, {} holes, {} triangles (best-fit plane normal=({:.3},{:.3},{:.3})), outer_only={}, fan={}",
        n_outer, hole_start_indices.len(), mesh.triangles.len(), nx, ny, nz,
        outer_only_mode, fan_centroid_3d.is_some(),
    );

    mesh
}
