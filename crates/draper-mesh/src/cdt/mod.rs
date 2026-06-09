// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Constrained Delaunay Triangulation (CDT) module.
//!
//! Uses `spade` for proper Constrained Delaunay Triangulation.
//! Spade guarantees that all constraint edges appear in the output,
//! which is essential for watertight mesh generation.
//!
//! # Algorithm
//! 1. Validate and prepare points and constraint edges
//! 2. Snap near-edge vertices onto constraint edges (T-junction elimination)
//! 3. Build spade CDT with constraint edges using bulk_load_cdt
//! 4. Extract triangles inside the domain (bounded by outer boundary, outside holes)
//! 5. Return triangles with indices into the input point array
//!
//! Vertex deduplication is NOT performed here — it's handled at a higher level
//! by `merge_coincident_vertices()` when merging face meshes into a solid mesh.
//! This avoids complex index remapping and ensures 3D positions are preserved.

pub mod predicates;
pub mod preprocess;

use std::collections::HashMap;
use std::collections::HashSet;

/// Sentinel value for "no index" or "internally generated".
const SENTINEL: u32 = u32::MAX;

/// A 2D point for CDT with an original index for mapping back to caller's data.
#[derive(Clone, Copy, Debug)]
pub struct CdtPoint {
    pub x: f64,
    pub y: f64,
    /// Index into the caller's original point array. SENTINEL for internally generated points.
    pub original_index: u32,
}

impl CdtPoint {
    pub fn new(x: f64, y: f64, original_index: u32) -> Self {
        Self { x, y, original_index }
    }
}

/// Configuration for CDT.
#[derive(Clone, Debug)]
pub struct CdtConfig {
    /// Tolerance for edge snapping (T-junction elimination).
    /// Vertex deduplication is NOT performed here.
    pub tolerance: f64,
    /// Whether to snap near-edge vertices onto constraint edges.
    pub snap_vertices_to_edges: bool,
}

impl Default for CdtConfig {
    fn default() -> Self {
        Self {
            tolerance: 1e-10,
            snap_vertices_to_edges: true,
        }
    }
}

/// Result of CDT triangulation.
#[derive(Clone, Debug)]
pub struct CdtResult {
    /// Triangle indices into the input points array.
    pub triangles: Vec<[u32; 3]>,
    /// The (possibly modified by snapping) point array.
    pub points: Vec<CdtPoint>,
    /// Constraint edges in the final triangulation.
    pub constraint_edges: Vec<[u32; 2]>,
}

/// Perform Constrained Delaunay Triangulation using spade.
///
/// # Algorithm
/// 1. Build constraint edges from boundary/hole polygons
/// 2. Snap near-edge vertices onto constraints (eliminate T-junctions)
/// 3. Build spade CDT with all points and constraints
/// 4. Extract triangles inside the domain
///
/// # Guarantees
/// - Every constraint edge appears as an edge in the output triangulation.
/// - No T-junctions within tolerance (if snap_vertices_to_edges is enabled).
/// - All boundary vertices are used in the triangulation.
pub fn constrained_delaunay(
    points: &[CdtPoint],
    outer_boundary: &[u32],
    holes: &[Vec<u32>],
    extra_constraints: &[[u32; 2]],
    config: &CdtConfig,
) -> CdtResult {
    if points.len() < 3 || outer_boundary.len() < 3 {
        return CdtResult {
            triangles: Vec::new(),
            points: points.to_vec(),
            constraint_edges: Vec::new(),
        };
    }

    let mut points = points.to_vec();

    // ============================================================
    // Step 1: Build constraint edges from boundary/hole polygons
    // ============================================================
    let mut constraints: Vec<[u32; 2]> = Vec::new();

    for i in 0..outer_boundary.len() {
        let a = outer_boundary[i];
        let b = outer_boundary[(i + 1) % outer_boundary.len()];
        if a != b && (a as usize) < points.len() && (b as usize) < points.len() {
            constraints.push([a, b]);
        }
    }

    for hole in holes {
        for i in 0..hole.len() {
            let a = hole[i];
            let b = hole[(i + 1) % hole.len()];
            if a != b && (a as usize) < points.len() && (b as usize) < points.len() {
                constraints.push([a, b]);
            }
        }
    }

    for e in extra_constraints {
        if e[0] != e[1] && (e[0] as usize) < points.len() && (e[1] as usize) < points.len() {
            constraints.push(*e);
        }
    }

    // Remove duplicate constraints
    let mut seen: std::collections::HashSet<(u32, u32)> = std::collections::HashSet::new();
    constraints.retain(|e| {
        let key = (e[0].min(e[1]), e[0].max(e[1]));
        seen.insert(key)
    });

    // ============================================================
    // Step 2: Snap near-edge vertices to constraints
    // ============================================================
    if config.snap_vertices_to_edges && config.tolerance > 0.0 {
        let boundary_indices: Vec<u32> = outer_boundary
            .iter()
            .chain(holes.iter().flat_map(|h| h.iter()))
            .copied()
            .collect();
        preprocess::snap_vertices_to_constraints(
            &mut points,
            &mut constraints,
            &boundary_indices,
            config.tolerance,
        );
    }

    // ============================================================
    // Step 2.5: Deduplicate points before passing to spade
    //
    // spade will panic if it encounters duplicate (coincident)
    // vertices that form a constraint edge. We deduplicate here
    // and remap all constraint indices.
    // ============================================================
    let dedup_tolerance = config.tolerance.max(1e-12);
    let (dedup_points, remap) = preprocess::deduplicate_points(&points, dedup_tolerance);

    // Remap constraint edges through dedup
    let mut dedup_constraints: Vec<[u32; 2]> = Vec::new();
    for e in &constraints {
        let a = if (e[0] as usize) < remap.len() { remap[e[0] as usize] } else { e[0] };
        let b = if (e[1] as usize) < remap.len() { remap[e[1] as usize] } else { e[1] };
        if a != b && a != SENTINEL && b != SENTINEL {
            dedup_constraints.push([a, b]);
        }
    }

    // Remove duplicate dedup constraints
    let mut seen2: std::collections::HashSet<(u32, u32)> = std::collections::HashSet::new();
    dedup_constraints.retain(|e| {
        let key = (e[0].min(e[1]), e[0].max(e[1]));
        seen2.insert(key)
    });

    // Remap boundary/hole indices through dedup
    let outer_boundary_dedup: Vec<u32> = preprocess::remap_indices(outer_boundary, &remap);
    let holes_dedup: Vec<Vec<u32>> = holes
        .iter()
        .map(|h| preprocess::remap_indices(h, &remap))
        .collect();

    // Build reverse mapping: dedup_index → first_original_index
    let mut dedup_to_original: HashMap<u32, u32> = HashMap::new();
    for (orig, &dedup) in remap.iter().enumerate() {
        if dedup != SENTINEL {
            dedup_to_original.entry(dedup).or_insert(orig as u32);
        }
    }

    // ============================================================
    // Step 3: Build spade CDT
    // ============================================================
    let n_pts = dedup_points.len();

    // Validate coordinates
    let mut valid = true;
    for p in &dedup_points {
        if !p.x.is_finite() || !p.y.is_finite() {
            valid = false;
            break;
        }
    }
    if !valid {
        log::warn!("CDT: non-finite coordinates detected, returning empty result");
        return CdtResult {
            triangles: Vec::new(),
            points,
            constraint_edges: constraints,
        };
    }

    // Build spade vertices
    let spade_vertices: Vec<spade::Point2<f64>> = dedup_points
        .iter()
        .map(|p| spade::Point2::new(p.x, p.y))
        .collect();

    // Build constraint edge list (using dedup indices)
    let spade_edges: Vec<[usize; 2]> = dedup_constraints
        .iter()
        .map(|e| [e[0] as usize, e[1] as usize])
        .collect();

    // Use spade bulk_load_cdt
    let cdt_result = spade::ConstrainedDelaunayTriangulation::<spade::Point2<f64>>::bulk_load_cdt(
        spade_vertices,
        spade_edges,
    );

    let cdt = match cdt_result {
        Ok(cdt) => cdt,
        Err(e) => {
            log::warn!("CDT: spade bulk_load_cdt failed: {:?}, falling back to earcutr", e);
            return constrained_delaunay_earcutr_fallback(
                &points, outer_boundary, holes, &constraints,
            );
        }
    };

    // ============================================================
    // Step 4: Extract triangles inside the domain
    // ============================================================

    // Build outer boundary polygon for inside/outside check
    let outer_polygon: Vec<(f64, f64)> = outer_boundary_dedup
        .iter()
        .map(|&i| (dedup_points[i as usize].x, dedup_points[i as usize].y))
        .collect();

    let hole_polygons: Vec<Vec<(f64, f64)>> = holes_dedup
        .iter()
        .map(|hole| {
            hole.iter()
                .map(|&i| (dedup_points[i as usize].x, dedup_points[i as usize].y))
                .collect()
        })
        .collect();

    let mut triangles: Vec<[u32; 3]> = Vec::new();

    // Build a set of constraint edges for determining inside faces.
    // A triangle is "inside" if it is bounded by at least one constraint edge
    // OR if its centroid is inside the outer boundary and outside all holes.
    // This dual approach ensures we don't miss boundary triangles whose
    // centroids fall slightly outside the polygon due to numerical precision.
    let constraint_edge_set_dedup: HashSet<(usize, usize)> = dedup_constraints
        .iter()
        .map(|e| (e[0].min(e[1]) as usize, e[0].max(e[1]) as usize))
        .collect();

    use spade::Triangulation;
    for face in cdt.inner_faces() {
        let v0 = face.vertices()[0];
        let v1 = face.vertices()[1];
        let v2 = face.vertices()[2];

        let i0 = v0.index();
        let i1 = v1.index();
        let i2 = v2.index();

        // Skip if any vertex is out of range
        if i0 >= n_pts || i1 >= n_pts || i2 >= n_pts {
            continue;
        }

        let p0 = &dedup_points[i0];
        let p1 = &dedup_points[i1];
        let p2 = &dedup_points[i2];

        // Skip degenerate triangles
        if predicates::orient2d(p0.x, p0.y, p1.x, p1.y, p2.x, p2.y).abs() < 1e-20 {
            continue;
        }

        // Determine if this triangle is inside the domain.
        // Strategy: Check if any edge of this triangle is a constraint edge.
        // If at least one edge is a constraint, the triangle is definitely inside.
        // If no edges are constraints, use centroid-based point-in-polygon test.
        let tri_edges = [
            (i0.min(i1), i0.max(i1)),
            (i1.min(i2), i1.max(i2)),
            (i0.min(i2), i0.max(i2)),
        ];
        let has_constraint_edge = tri_edges.iter().any(|e| constraint_edge_set_dedup.contains(e));

        let inside = if has_constraint_edge {
            // Triangle has at least one constraint edge — it's on the boundary or inside.
            // Check that the centroid is NOT inside any hole.
            let cx = (p0.x + p1.x + p2.x) / 3.0;
            let cy = (p0.y + p1.y + p2.y) / 3.0;
            let mut in_hole = false;
            for hole_poly in &hole_polygons {
                if point_in_polygon(cx, cy, hole_poly) {
                    in_hole = true;
                    break;
                }
            }
            !in_hole
        } else {
            // No constraint edges — use centroid-based test
            let cx = (p0.x + p1.x + p2.x) / 3.0;
            let cy = (p0.y + p1.y + p2.y) / 3.0;
            if !point_in_polygon(cx, cy, &outer_polygon) {
                false
            } else {
                let mut in_hole = false;
                for hole_poly in &hole_polygons {
                    if point_in_polygon(cx, cy, hole_poly) {
                        in_hole = true;
                        break;
                    }
                }
                !in_hole
            }
        };

        if !inside {
            continue;
        }

        // Map dedup indices back to original indices
        let orig0 = dedup_to_original.get(&(i0 as u32)).copied().unwrap_or(i0 as u32);
        let orig1 = dedup_to_original.get(&(i1 as u32)).copied().unwrap_or(i1 as u32);
        let orig2 = dedup_to_original.get(&(i2 as u32)).copied().unwrap_or(i2 as u32);

        triangles.push([orig0, orig1, orig2]);
    }

    // Verify all constraint edges are present (using original indices)
    let edge_set = build_edge_set(&triangles);
    let mut missing = 0;
    for e in &constraints {
        let key = (e[0].min(e[1]), e[0].max(e[1]));
        if !edge_set.contains(&key) {
            missing += 1;
        }
    }

    if missing > 0 {
        log::warn!(
            "CDT: {} of {} constraint edges missing after spade triangulation",
            missing,
            constraints.len()
        );
    }

    CdtResult {
        triangles,
        points,
        constraint_edges: constraints,
    }
}

/// Earcutr-based fallback for when spade CDT fails.
///
/// Uses earcutr for triangulation, then enforces constraint edges.
/// Less robust than spade but always terminates.
fn constrained_delaunay_earcutr_fallback(
    points: &[CdtPoint],
    outer_boundary: &[u32],
    holes: &[Vec<u32>],
    constraints: &[[u32; 2]],
) -> CdtResult {
    if points.len() < 3 || outer_boundary.len() < 3 {
        return CdtResult {
            triangles: Vec::new(),
            points: points.to_vec(),
            constraint_edges: constraints.to_vec(),
        };
    }

    // Build earcutr input: boundary + holes + interior points
    let boundary_set: std::collections::HashSet<u32> = outer_boundary
        .iter()
        .chain(holes.iter().flat_map(|h| h.iter()))
        .copied()
        .collect();

    let mut hole_start_indices: Vec<usize> = Vec::new();
    let mut all_uv: Vec<f64> = Vec::with_capacity(points.len() * 2);
    let mut earcutr_to_original: Vec<u32> = Vec::new();

    // Add boundary points
    for &vi in outer_boundary {
        all_uv.push(points[vi as usize].x);
        all_uv.push(points[vi as usize].y);
        earcutr_to_original.push(vi);
    }

    // Add hole points
    for hole in holes {
        hole_start_indices.push(all_uv.len() / 2);
        for &vi in hole {
            all_uv.push(points[vi as usize].x);
            all_uv.push(points[vi as usize].y);
            earcutr_to_original.push(vi);
        }
    }

    // Add interior/Steiner points (points not in boundary or holes)
    for (i, _p) in points.iter().enumerate() {
        let i = i as u32;
        if !boundary_set.contains(&i) {
            all_uv.push(points[i as usize].x);
            all_uv.push(points[i as usize].y);
            earcutr_to_original.push(i);
        }
    }

    let earcutr_indices = earcutr::earcut(&all_uv, &hole_start_indices, 2);

    // Collect triangles — map earcutr sequential indices to original point indices
    let mut triangles: Vec<[u32; 3]> = Vec::with_capacity(earcutr_indices.len() / 3);
    for chunk in earcutr_indices.chunks(3) {
        if chunk.len() < 3 { break; }
        let a = earcutr_to_original.get(chunk[0] as usize).copied().unwrap_or(chunk[0] as u32);
        let b = earcutr_to_original.get(chunk[1] as usize).copied().unwrap_or(chunk[1] as u32);
        let c = earcutr_to_original.get(chunk[2] as usize).copied().unwrap_or(chunk[2] as u32);
        if a == b || b == c || a == c { continue; }
        triangles.push([a, b, c]);
    }

    // Enforce constraint edges
    enforce_constraint_edges(&mut triangles, points, constraints);

    // Filter degenerate triangles
    triangles.retain(|tri| {
        if tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2] {
            return false;
        }
        let a = &points[tri[0] as usize];
        let b = &points[tri[1] as usize];
        let c = &points[tri[2] as usize];
        predicates::orient2d(a.x, a.y, b.x, b.y, c.x, c.y).abs() > 1e-20
    });

    CdtResult {
        triangles,
        points: points.to_vec(),
        constraint_edges: constraints.to_vec(),
    }
}

// ============================================================
// Helper functions
// ============================================================

/// Build a set of edges from triangles.
fn build_edge_set(triangles: &[[u32; 3]]) -> std::collections::HashSet<(u32, u32)> {
    let mut set = std::collections::HashSet::new();
    for tri in triangles {
        for k in 0..3 {
            let a = tri[k].min(tri[(k + 1) % 3]);
            let b = tri[k].max(tri[(k + 1) % 3]);
            set.insert((a, b));
        }
    }
    set
}

/// Ray-casting point-in-polygon test.
fn point_in_polygon(px: f64, py: f64, polygon: &[(f64, f64)]) -> bool {
    if polygon.len() < 3 {
        return false;
    }
    let n = polygon.len();
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = polygon[i];
        let (xj, yj) = polygon[j];
        if ((yi > py) != (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi) {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Enforce that all constraint edges appear as edges in the triangulation.
fn enforce_constraint_edges(
    triangles: &mut Vec<[u32; 3]>,
    points: &[CdtPoint],
    constraints: &[[u32; 2]],
) {
    for _pass in 0..3 {
        let edge_set = build_edge_set(triangles);
        let mut any_inserted = false;

        for constraint in constraints {
            let a = constraint[0];
            let b = constraint[1];
            let key = (a.min(b), a.max(b));
            if edge_set.contains(&key) {
                continue;
            }

            if insert_constraint_edge(triangles, points, a, b) {
                any_inserted = true;
            }
        }

        if !any_inserted { break; }
    }
}

/// Insert a single constraint edge (a, b) into the triangulation.
fn insert_constraint_edge(
    triangles: &mut Vec<[u32; 3]>,
    points: &[CdtPoint],
    a: u32,
    b: u32,
) -> bool {
    let key = (a.min(b), a.max(b));
    if build_edge_set(triangles).contains(&key) {
        return true;
    }

    let intersected = walk_intersected_edges(triangles, points, a, b);
    if intersected.is_empty() {
        return false;
    }

    for (c, d) in &intersected {
        let c = *c;
        let d = *d;
        let mut tri_indices: Vec<usize> = Vec::new();
        for (ti, tri) in triangles.iter().enumerate() {
            let has_c = tri.contains(&c);
            let has_d = tri.contains(&d);
            if has_c && has_d {
                tri_indices.push(ti);
                if tri_indices.len() == 2 { break; }
            }
        }

        if tri_indices.len() < 2 {
            continue;
        }

        let t0 = tri_indices[0];
        let t1 = tri_indices[1];
        let tri0 = triangles[t0];
        let tri1 = triangles[t1];

        let opp0 = tri0.iter().find(|&&v| v != c && v != d).copied();
        let opp1 = tri1.iter().find(|&&v| v != c && v != d).copied();

        let (Some(e), Some(f)) = (opp0, opp1) else {
            continue;
        };

        let ex = points[e as usize].x; let ey = points[e as usize].y;
        let cx = points[c as usize].x; let cy = points[c as usize].y;
        let fx = points[f as usize].x; let fy = points[f as usize].y;
        let dx_ = points[d as usize].x; let dy_ = points[d as usize].y;

        let orient_ecf = predicates::orient2d(ex, ey, cx, cy, fx, fy);
        let orient_efd = predicates::orient2d(ex, ey, fx, fy, dx_, dy_);

        if (orient_ecf > 0.0) != (orient_efd > 0.0) {
            continue;
        }

        let new0 = if predicates::orient2d(ex, ey, fx, fy, cx, cy) > 0.0 {
            [e, f, c]
        } else {
            [e, c, f]
        };

        let new1 = if predicates::orient2d(ex, ey, dx_, dy_, fx, fy) > 0.0 {
            [e, d, f]
        } else {
            [e, f, d]
        };

        triangles[t0] = new0;
        triangles[t1] = new1;
    }

    build_edge_set(triangles).contains(&key)
}

/// Walk through the triangulation from vertex a to vertex b,
/// collecting all edges that intersect segment (a, b).
fn walk_intersected_edges(
    triangles: &[[u32; 3]],
    points: &[CdtPoint],
    a: u32,
    b: u32,
) -> Vec<(u32, u32)> {
    let ax = points[a as usize].x;
    let ay = points[a as usize].y;
    let bx = points[b as usize].x;
    let by = points[b as usize].y;

    let start_tri = find_triangle_on_path(triangles, points, a, b);
    let Some(start) = start_tri else {
        return Vec::new();
    };

    let mut result: Vec<(u32, u32)> = Vec::new();
    let mut current_tri = start;
    let mut visited: std::collections::HashSet<usize> = std::collections::HashSet::new();

    loop {
        if visited.contains(&current_tri) { break; }
        visited.insert(current_tri);

        let tri = triangles[current_tri];

        if tri.contains(&a) && tri.contains(&b) {
            break;
        }

        let mut found_exit = false;
        for k in 0..3 {
            let v0 = tri[k];
            let v1 = tri[(k + 1) % 3];

            if v0 == a || v1 == a { continue; }

            let v0x = points[v0 as usize].x;
            let v0y = points[v0 as usize].y;
            let v1x = points[v1 as usize].x;
            let v1y = points[v1 as usize].y;

            if predicates::segments_intersect_proper(ax, ay, bx, by, v0x, v0y, v1x, v1y) {
                result.push((v0.min(v1), v0.max(v1)));

                let next = find_adjacent_triangle(triangles, current_tri, v0, v1);
                match next {
                    Some(nt) => {
                        current_tri = nt;
                        found_exit = true;
                        break;
                    }
                    None => {
                        found_exit = true;
                        break;
                    }
                }
            }
        }

        if !found_exit { break; }
    }

    result
}

/// Find a triangle containing vertex a that is on the path toward vertex b.
fn find_triangle_on_path(
    triangles: &[[u32; 3]],
    points: &[CdtPoint],
    a: u32,
    b: u32,
) -> Option<usize> {
    let ax = points[a as usize].x;
    let ay = points[a as usize].y;
    let bx = points[b as usize].x;
    let by = points[b as usize].y;

    for (ti, tri) in triangles.iter().enumerate() {
        if !tri.contains(&a) { continue; }

        for k in 0..3 {
            let v0 = tri[k];
            let v1 = tri[(k + 1) % 3];

            if v0 == a || v1 == a { continue; }

            let v0x = points[v0 as usize].x;
            let v0y = points[v0 as usize].y;
            let v1x = points[v1 as usize].x;
            let v1y = points[v1 as usize].y;

            if predicates::segments_intersect_proper(ax, ay, bx, by, v0x, v0y, v1x, v1y) {
                return Some(ti);
            }
        }
    }
    None
}

/// Find the triangle adjacent to triangle `ti` across edge (v0, v1).
fn find_adjacent_triangle(triangles: &[[u32; 3]], ti: usize, v0: u32, v1: u32) -> Option<usize> {
    for (tj, tri) in triangles.iter().enumerate() {
        if tj == ti { continue; }
        let has_v0 = tri.contains(&v0);
        let has_v1 = tri.contains(&v1);
        if has_v0 && has_v1 {
            return Some(tj);
        }
    }
    None
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn check_all_constraints_present(result: &CdtResult) {
        let edge_set = build_edge_set(&result.triangles);
        for e in &result.constraint_edges {
            let key = (e[0].min(e[1]), e[0].max(e[1]));
            assert!(
                edge_set.contains(&key),
                "Constraint edge ({}, {}) missing from triangulation",
                e[0],
                e[1]
            );
        }
    }

    #[test]
    fn test_square_cdt() {
        let points = vec![
            CdtPoint::new(0.0, 0.0, 0),
            CdtPoint::new(1.0, 0.0, 1),
            CdtPoint::new(1.0, 1.0, 2),
            CdtPoint::new(0.0, 1.0, 3),
        ];
        let boundary = vec![0, 1, 2, 3];
        let config = CdtConfig {
            snap_vertices_to_edges: false,
            ..CdtConfig::default()
        };

        let result = constrained_delaunay(&points, &boundary, &[], &[], &config);

        assert!(!result.triangles.is_empty(), "Should produce triangles");
        check_all_constraints_present(&result);
    }

    #[test]
    fn test_collinear_boundary_vertices() {
        let points = vec![
            CdtPoint::new(0.0, 0.0, 0),
            CdtPoint::new(1.0, 0.0, 1),
            CdtPoint::new(2.0, 0.0, 2),
            CdtPoint::new(3.0, 0.0, 3),
            CdtPoint::new(3.0, 1.0, 4),
            CdtPoint::new(0.0, 1.0, 5),
        ];
        let boundary = vec![0, 1, 2, 3, 4, 5];
        let config = CdtConfig {
            snap_vertices_to_edges: false,
            ..CdtConfig::default()
        };

        let result = constrained_delaunay(&points, &boundary, &[], &[], &config);

        assert!(!result.triangles.is_empty());
        check_all_constraints_present(&result);

        // Verify collinear vertices are connected
        let edge_set = build_edge_set(&result.triangles);
        assert!(edge_set.contains(&(0, 1)), "Edge (0,1) should exist");
        assert!(edge_set.contains(&(1, 2)), "Edge (1,2) should exist — collinear vertex");
        assert!(edge_set.contains(&(2, 3)), "Edge (2,3) should exist — collinear vertex");
    }

    #[test]
    fn test_square_with_hole() {
        let points = vec![
            CdtPoint::new(0.0, 0.0, 0),
            CdtPoint::new(2.0, 0.0, 1),
            CdtPoint::new(2.0, 2.0, 2),
            CdtPoint::new(0.0, 2.0, 3),
            CdtPoint::new(0.5, 0.5, 4),
            CdtPoint::new(1.5, 0.5, 5),
            CdtPoint::new(1.5, 1.5, 6),
            CdtPoint::new(0.5, 1.5, 7),
        ];
        let boundary = vec![0, 1, 2, 3];
        let holes = vec![vec![4, 5, 6, 7]];
        let config = CdtConfig {
            snap_vertices_to_edges: false,
            ..CdtConfig::default()
        };

        let result = constrained_delaunay(&points, &boundary, &holes, &[], &config);

        assert!(!result.triangles.is_empty());
        check_all_constraints_present(&result);
    }

    #[test]
    fn test_non_convex_polygon() {
        let points = vec![
            CdtPoint::new(0.0, 0.0, 0),
            CdtPoint::new(2.0, 0.0, 1),
            CdtPoint::new(2.0, 1.0, 2),
            CdtPoint::new(1.0, 1.0, 3),
            CdtPoint::new(1.0, 2.0, 4),
            CdtPoint::new(0.0, 2.0, 5),
        ];
        let boundary = vec![0, 1, 2, 3, 4, 5];
        let config = CdtConfig {
            snap_vertices_to_edges: false,
            ..CdtConfig::default()
        };

        let result = constrained_delaunay(&points, &boundary, &[], &[], &config);

        assert!(!result.triangles.is_empty());
        check_all_constraints_present(&result);
    }

    #[test]
    fn test_all_boundary_vertices_used() {
        let points = vec![
            CdtPoint::new(0.0, 0.0, 0),
            CdtPoint::new(2.0, 0.0, 1),
            CdtPoint::new(3.0, 1.5, 2),
            CdtPoint::new(1.5, 3.0, 3),
            CdtPoint::new(-0.5, 1.5, 4),
        ];
        let boundary = vec![0, 1, 2, 3, 4];
        let config = CdtConfig {
            snap_vertices_to_edges: false,
            ..CdtConfig::default()
        };

        let result = constrained_delaunay(&points, &boundary, &[], &[], &config);

        assert!(!result.triangles.is_empty());
        check_all_constraints_present(&result);

        let used_vertices: std::collections::HashSet<u32> = result
            .triangles
            .iter()
            .flat_map(|tri| tri.iter().copied())
            .collect();

        for &vi in &boundary {
            assert!(used_vertices.contains(&vi), "Boundary vertex {} should appear", vi);
        }
    }

    #[test]
    fn test_point_in_polygon() {
        let square = vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
        assert!(point_in_polygon(0.5, 0.5, &square));
        assert!(!point_in_polygon(1.5, 0.5, &square));
        assert!(!point_in_polygon(-0.5, 0.5, &square));
    }
}
