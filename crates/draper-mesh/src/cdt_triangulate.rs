// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! CDT-based solid triangulation with tolerance-aware vertex unification.
//!
//! This module provides watertight mesh generation by:
//! 1. Pre-discretizing ALL edges consistently (shared edges produce identical 3D points)
//! 2. For each face, collecting boundary points from the consistent edge cache
//! 3. Projecting boundary points to 2D and triangulating with custom CDT
//! 4. Mapping CDT triangles back to 3D using the surface parameterization
//! 5. Since shared edges use identical vertex indices, the result is watertight by construction

use crate::mesh::TriangleMesh;
use crate::triangulate::{
    TriangulationParams, triangulate_face, filter_degenerate_triangles,
    sample_edge_points,
};
use crate::custom_cdt;
use crate::watertight::stitch_boundary_edges;
use draper_geometry::{
    Point3d, Point2d, Direction3d,
    Surface, Plane, CylinderSurface, SphereSurface,
    TorusSurface, ConeSurface,
};
use draper_topology::{Face, Solid, TopoId};
use spade::{ConstrainedDelaunayTriangulation, Point2, HasPosition, Triangulation as SpadeTriangulation};
use spade::handles::FixedVertexHandle;
use std::collections::HashMap;

/// Number of samples per edge curve for boundary discretization.
const CDT_EDGE_SAMPLES: usize = 32;

/// A consistent edge discretization cache that ensures shared edges produce
/// identical 3D vertex positions across all faces.
#[derive(Clone, Debug)]
pub struct ConsistentEdgeCache {
    /// Maps edge TopoId → sampled 3D points (in canonical direction).
    entries: HashMap<TopoId, Vec<Point3d>>,
    /// Number of samples per edge.
    n_samples: usize,
}

impl ConsistentEdgeCache {
    /// Build a cache by pre-computing discretizations for all edges in a solid.
    pub fn build_from_solid(solid: &Solid, n_samples: usize) -> Self {
        let mut entries = HashMap::new();
        for face in solid.faces() {
            for edge in &face.edges {
                if edge.degenerate {
                    continue;
                }
                if !entries.contains_key(&edge.id) {
                    let pts = sample_edge_points(edge, n_samples);
                    entries.insert(edge.id, pts);
                }
            }
        }
        Self { entries, n_samples }
    }

    /// Get or compute edge points.
    fn get_or_compute(&self, edge: &draper_topology::Edge) -> Vec<Point3d> {
        if let Some(cached) = self.entries.get(&edge.id) {
            cached.clone()
        } else {
            sample_edge_points(edge, self.n_samples)
        }
    }
}

/// Collect face boundary points using the consistent edge cache.
fn collect_cached_boundary_points(face: &Face, cache: &ConsistentEdgeCache) -> Vec<Point3d> {
    let mut points = Vec::new();

    if let Some(ref wire) = face.outer_wire {
        for coedge in &wire.coedges {
            let edge = face.edges.iter().find(|e| e.id == coedge.edge);
            if let Some(edge) = edge {
                if edge.degenerate {
                    continue;
                }
                let mut edge_pts = cache.get_or_compute(edge);
                let edge_is_reversed = edge.param_range.0 > edge.param_range.1;
                let should_reverse = !coedge.forward != edge_is_reversed;
                if should_reverse {
                    edge_pts.reverse();
                }
                points.extend(edge_pts);
            }
        }
    }

    deduplicate_consecutive(&mut points);
    points
}

/// Collect face hole boundary points using the consistent edge cache.
fn collect_cached_hole_points(face: &Face, cache: &ConsistentEdgeCache) -> Vec<Vec<Point3d>> {
    let mut holes = Vec::new();
    for wire in &face.inner_wires {
        let mut points = Vec::new();
        for coedge in &wire.coedges {
            let edge = face.edges.iter().find(|e| e.id == coedge.edge);
            if let Some(edge) = edge {
                if edge.degenerate {
                    continue;
                }
                let mut edge_pts = cache.get_or_compute(edge);
                let edge_is_reversed = edge.param_range.0 > edge.param_range.1;
                let should_reverse = !coedge.forward != edge_is_reversed;
                if should_reverse {
                    edge_pts.reverse();
                }
                points.extend(edge_pts);
            }
        }
        deduplicate_consecutive(&mut points);
        if !points.is_empty() {
            holes.push(points);
        }
    }
    holes
}

/// Remove duplicate consecutive points (within tolerance) and close the loop.
fn deduplicate_consecutive(points: &mut Vec<Point3d>) {
    if points.is_empty() {
        return;
    }
    let mut unique = vec![points[0]];
    for p in &points[1..] {
        if let Some(last) = unique.last() {
            if !last.is_coincident_with(p) {
                unique.push(*p);
            }
        }
    }
    if unique.len() > 1 {
        if let Some(last) = unique.last() {
            if last.is_coincident_with(&unique[0]) {
                unique.pop();
            }
        }
    }
    *points = unique;
}

// ============================================================
// 2D projection and CDT helpers
// ============================================================

/// A 2D vertex for spade CDT that carries the unified 3D vertex pool index.
#[derive(Clone, Debug)]
struct CdtVertex {
    pos: [f64; 2],
    /// Index into the unified 3D vertex pool.
    index_3d: u32,
}

impl HasPosition for CdtVertex {
    type Scalar = f64;

    fn position(&self) -> Point2<Self::Scalar> {
        Point2::new(self.pos[0], self.pos[1])
    }
}

/// Project a 3D point onto a plane's 2D coordinate system.
fn project_to_plane(p: &Point3d, plane: &Plane) -> [f64; 2] {
    let dx = p.x - plane.origin.x;
    let dy = p.y - plane.origin.y;
    let dz = p.z - plane.origin.z;
    let u = dx * plane.u_dir.x + dy * plane.u_dir.y + dz * plane.u_dir.z;
    let v = dx * plane.v_dir.x + dy * plane.v_dir.y + dz * plane.v_dir.z;
    [u, v]
}

/// Triangulate a planar face using spade CDT with consistent boundary vertices.
fn triangulate_planar_cdt(
    face: &Face,
    plane: &Plane,
    boundary_3d: &[Point3d],
    holes_3d: &[Vec<Point3d>],
    forward: bool,
    pool: &mut UnifiedVertexPool,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    if boundary_3d.len() < 3 {
        return mesh;
    }

    let face_id = face.id.to_u64();

    // Insert all 3D boundary vertices into the unified pool FIRST.
    let boundary_indices: Vec<u32> = boundary_3d.iter().map(|p| pool.insert_with_face(*p, face_id)).collect();
    let hole_indices: Vec<Vec<u32>> = holes_3d.iter()
        .map(|hole| hole.iter().map(|p| pool.insert_with_face(*p, face_id)).collect())
        .collect();

    // Collect all 3D points and their pool indices
    let mut all_3d: Vec<Point3d> = boundary_3d.to_vec();
    let mut all_pool_indices: Vec<u32> = boundary_indices.clone();
    for hole in holes_3d {
        all_3d.extend_from_slice(hole);
    }
    for hole_idx in &hole_indices {
        all_pool_indices.extend_from_slice(hole_idx);
    }

    // Project all points to 2D
    let all_2d: Vec<[f64; 2]> = all_3d.iter().map(|p| project_to_plane(p, plane)).collect();

    // Build CDT vertices with their pool indices
    let cdt_vertices: Vec<CdtVertex> = all_2d.iter().zip(all_pool_indices.iter())
        .map(|(pos, &idx)| CdtVertex { pos: *pos, index_3d: idx })
        .collect();

    // Build spade CDT
    let mut cdt: ConstrainedDelaunayTriangulation<CdtVertex> = ConstrainedDelaunayTriangulation::new();

    // Insert all vertices and remember their CDT handles
    let mut cdt_handle_map: Vec<usize> = Vec::with_capacity(cdt_vertices.len());
    for v in &cdt_vertices {
        match cdt.insert(v.clone()) {
            Ok(handle) => cdt_handle_map.push(handle.index()),
            Err(_) => {
                // Duplicate vertex — find the existing one
                // Search for a vertex with the same pool index
                let mut found_idx = 0;
                for (i, v2) in cdt.vertices().enumerate() {
                    if v2.data().index_3d == v.index_3d {
                        found_idx = i;
                        break;
                    }
                }
                cdt_handle_map.push(found_idx);
            }
        }
    }

    // Add boundary constraints (outer ring)
    let n_boundary = boundary_3d.len();
    for i in 0..n_boundary {
        let j = (i + 1) % n_boundary;
        let hi = FixedVertexHandle::from_index(cdt_handle_map[i]);
        let hj = FixedVertexHandle::from_index(cdt_handle_map[j]);
        if hi != hj {
            let _ = cdt.add_constraint(hi, hj);
        }
    }

    // Add hole constraints
    let mut offset = n_boundary;
    for hole_idx in &hole_indices {
        let n_hole = hole_idx.len();
        for i in 0..n_hole {
            let j = (i + 1) % n_hole;
            let gi = offset + i;
            let gj = offset + j;
            if gi < cdt_handle_map.len() && gj < cdt_handle_map.len() {
                let hi = FixedVertexHandle::from_index(cdt_handle_map[gi]);
                let hj = FixedVertexHandle::from_index(cdt_handle_map[gj]);
                if hi != hj {
                    let _ = cdt.add_constraint(hi, hj);
                }
            }
        }
        offset += n_hole;
    }

    // Extract triangles from the CDT
    let normal = if forward {
        plane.normal
    } else {
        Direction3d::new(-plane.normal.x, -plane.normal.y, -plane.normal.z).unwrap_or(Direction3d::Z)
    };

    // Build a mapping from CDT vertex index → pool index
    let cdt_to_pool: Vec<u32> = cdt.vertices().map(|v| v.data().index_3d).collect();

    for face_handle in cdt.inner_faces() {
        // Get the 3 vertices of this triangle face via adjacent edges
        let edges = face_handle.adjacent_edges();
        let vs = [edges[0].from(), edges[1].from(), edges[2].from()];

        let a = cdt_to_pool[vs[0].fix().index()];
        let b = cdt_to_pool[vs[1].fix().index()];
        let c = cdt_to_pool[vs[2].fix().index()];

        // Skip degenerate triangles
        if a == b || b == c || a == c {
            continue;
        }

        // Check if the triangle centroid is inside the polygon
        let v0 = vs[0].data();
        let v1 = vs[1].data();
        let v2 = vs[2].data();
        let centroid_u = (v0.pos[0] + v1.pos[0] + v2.pos[0]) / 3.0;
        let centroid_v = (v0.pos[1] + v1.pos[1] + v2.pos[1]) / 3.0;
        let centroid = Point2d::new(centroid_u, centroid_v);

        let boundary_2d: Vec<Point2d> = all_2d[..n_boundary].iter()
            .map(|p| Point2d::new(p[0], p[1]))
            .collect();

        if !point_in_polygon(&centroid, &boundary_2d) {
            continue;
        }

        // Check if centroid is inside any hole
        let mut in_hole = false;
        let mut hole_offset = n_boundary;
        for hole_idx in &hole_indices {
            if hole_offset + hole_idx.len() <= all_2d.len() {
                let hole_2d: Vec<Point2d> = all_2d[hole_offset..hole_offset + hole_idx.len()].iter()
                    .map(|p| Point2d::new(p[0], p[1]))
                    .collect();
                if point_in_polygon(&centroid, &hole_2d) {
                    in_hole = true;
                    break;
                }
            }
            hole_offset += hole_idx.len();
        }
        if in_hole {
            continue;
        }

        if forward {
            mesh.add_triangle(a, b, c);
        } else {
            mesh.add_triangle(a, c, b);
        }
        mesh.triangle_face_ids.get_or_insert_with(Vec::new).push(face_id);
        mesh.face_normals.get_or_insert_with(Vec::new).push([normal.x, normal.y, normal.z]);
    }

    mesh
}

/// Triangulate a curved surface face with consistent edge boundary vertices.
fn triangulate_curved_face_consistent(
    face: &Face,
    boundary_3d: &[Point3d],
    params: &TriangulationParams,
    pool: &mut UnifiedVertexPool,
) -> TriangleMesh {
    let face_id = face.id.to_u64();

    // Insert boundary vertices into the unified pool first.
    // These come from the consistent edge cache, so shared edges produce
    // the same pool indices across different faces.
    let cache_boundary_indices: Vec<u32> = boundary_3d.iter().map(|p| pool.insert_with_face(*p, face_id)).collect();

    // Build a spatial index of cache boundary vertices for snapping
    let snap_tolerance = {
        // Use a generous snap tolerance: the maximum distance between adjacent
        // edge sample points. This ensures that face_mesh boundary vertices
        // snap to the correct cache vertex.
        let mut max_spacing = 0.0f64;
        for i in 1..boundary_3d.len() {
            let dx = boundary_3d[i].x - boundary_3d[i-1].x;
            let dy = boundary_3d[i].y - boundary_3d[i-1].y;
            let dz = boundary_3d[i].z - boundary_3d[i-1].z;
            let dist = (dx*dx + dy*dy + dz*dz).sqrt();
            max_spacing = max_spacing.max(dist);
        }
        // Also check last to first (closed loop)
        if boundary_3d.len() > 2 {
            let dx = boundary_3d[0].x - boundary_3d.last().unwrap().x;
            let dy = boundary_3d[0].y - boundary_3d.last().unwrap().y;
            let dz = boundary_3d[0].z - boundary_3d.last().unwrap().z;
            let dist = (dx*dx + dy*dy + dz*dz).sqrt();
            max_spacing = max_spacing.max(dist);
        }
        // Snap tolerance = 80% of max spacing (to avoid snapping to wrong vertex)
        max_spacing * 0.8
    };

    // Build spatial index of cache boundary vertices
    let snap_cell_size = snap_tolerance * 10.0;
    let snap_tol_sq = snap_tolerance * snap_tolerance;
    let mut snap_grid: HashMap<(i64, i64, i64), Vec<(Point3d, u32)>> = HashMap::new();
    for (i, &p) in boundary_3d.iter().enumerate() {
        let cx = (p.x / snap_cell_size).floor() as i64;
        let cy = (p.y / snap_cell_size).floor() as i64;
        let cz = (p.z / snap_cell_size).floor() as i64;
        snap_grid.entry((cx, cy, cz)).or_default().push((p, cache_boundary_indices[i]));
    }

    // Use standard triangulation as fallback for curved surfaces
    let face_mesh = triangulate_face(face, params);

    // For each vertex in the face mesh, try to snap to a cache boundary vertex.
    // If a vertex is close to a cache boundary vertex (within snap_tolerance),
    // use the cache vertex's pool index instead. This ensures that shared
    // edge vertices are unified across faces.
    let remap: Vec<u32> = face_mesh.vertices.iter().map(|p| {
        let cx = (p.x / snap_cell_size).floor() as i64;
        let cy = (p.y / snap_cell_size).floor() as i64;
        let cz = (p.z / snap_cell_size).floor() as i64;

        let mut best_match: Option<(u32, f64)> = None;
        for dx in -1i64..=1 {
            for dy in -1i64..=1 {
                for dz in -1i64..=1 {
                    let key = (cx + dx, cy + dy, cz + dz);
                    if let Some(entries) = snap_grid.get(&key) {
                        for &(ref cp, pool_idx) in entries {
                            let ddx = p.x - cp.x;
                            let ddy = p.y - cp.y;
                            let ddz = p.z - cp.z;
                            let dist_sq = ddx*ddx + ddy*ddy + ddz*ddz;
                            if dist_sq < snap_tol_sq {
                                match best_match {
                                    None => best_match = Some((pool_idx, dist_sq)),
                                    Some((_, best_dist)) => {
                                        if dist_sq < best_dist {
                                            best_match = Some((pool_idx, dist_sq));
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // If we found a snap match, use the cache boundary vertex's pool index.
        // Otherwise, insert as a new vertex into the pool.
        if let Some((pool_idx, _)) = best_match {
            pool_idx
        } else {
            pool.insert_with_face(*p, face_id)
        }
    }).collect();

    // Build the face mesh using the remapped indices
    let mut mesh = TriangleMesh::new();

    for (tri_idx, tri) in face_mesh.triangles.iter().enumerate() {
        let a = remap[tri[0] as usize];
        let b = remap[tri[1] as usize];
        let c = remap[tri[2] as usize];

        if a != b && b != c && a != c {
            mesh.add_triangle(a, b, c);
            mesh.triangle_face_ids.get_or_insert_with(Vec::new).push(face_id);

            if let Some(ref face_normals) = face_mesh.face_normals {
                if let Some(&n) = face_normals.get(tri_idx) {
                    mesh.face_normals.get_or_insert_with(Vec::new).push(n);
                }
            }
        }
    }

    mesh
}

/// Check if a 2D point is inside a polygon using the winding number algorithm.
fn point_in_polygon(point: &Point2d, polygon: &[Point2d]) -> bool {
    if polygon.len() < 3 {
        return false;
    }
    let n = polygon.len();
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let xi = polygon[i].u;
        let yi = polygon[i].v;
        let xj = polygon[j].u;
        let yj = polygon[j].v;

        if ((yi > point.v) != (yj > point.v))
            && (point.u < (xj - xi) * (point.v - yi) / (yj - yi) + xi)
        {
            inside = !inside;
        }
        j = i;
    }
    inside
}

// ============================================================
// CDT triangulation for curved surfaces
// ============================================================

/// Generic CDT-based surface triangulation with consistent boundary vertices.
///
/// Projects boundary points to 2D parametric space, builds a CDT with
/// boundary constraints, optionally adds interior grid points, then maps
/// triangles back to 3D using the surface parameterization.
fn triangulate_surface_cdt_generic(
    face: &Face,
    surface: &Surface,
    boundary_3d: &[Point3d],
    holes_3d: &[Vec<Point3d>],
    forward: bool,
    pool: &mut UnifiedVertexPool,
    params: &TriangulationParams,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    if boundary_3d.len() < 3 {
        return mesh;
    }

    let face_id = face.id.to_u64();

    // Insert boundary vertices into the unified pool
    let boundary_indices: Vec<u32> = boundary_3d.iter().map(|p| pool.insert_with_face(*p, face_id)).collect();
    let hole_indices: Vec<Vec<u32>> = holes_3d.iter()
        .map(|hole| hole.iter().map(|p| pool.insert_with_face(*p, face_id)).collect())
        .collect();

    // Project boundary points to 2D UV space
    let boundary_2d: Vec<[f64; 2]> = boundary_3d.iter()
        .map(|p| project_to_surface_uv(p, surface))
        .collect();

    // Check if UV projection is valid (no NaN/Inf)
    if boundary_2d.iter().any(|uv| !uv[0].is_finite() || !uv[1].is_finite()) {
        // Fall back to snap-boundary approach
        log::warn!("CDT surface fallback: UV projection produced NaN/Inf for face {}", face.id);
        return triangulate_curved_face_consistent(face, boundary_3d, params, pool);
    }

    log::info!(
        "CDT surface: face {} type={:?}, {} boundary pts, {} holes, UV range [{:.2},{:.2}]x[{:.2},{:.2}]",
        face.id,
        std::mem::discriminant(surface),
        boundary_3d.len(),
        holes_3d.len(),
        boundary_2d.iter().map(|uv| uv[0]).fold(f64::MAX, f64::min),
        boundary_2d.iter().map(|uv| uv[0]).fold(f64::MIN, f64::max),
        boundary_2d.iter().map(|uv| uv[1]).fold(f64::MAX, f64::min),
        boundary_2d.iter().map(|uv| uv[1]).fold(f64::MIN, f64::max),
    );

    // Project hole points to 2D
    let holes_2d: Vec<Vec<[f64; 2]>> = holes_3d.iter()
        .map(|hole| hole.iter()
            .map(|p| project_to_surface_uv(p, surface))
            .collect())
        .collect();

    // Collect all vertices for CDT
    let mut all_2d: Vec<[f64; 2]> = boundary_2d.clone();
    let mut all_pool_indices: Vec<u32> = boundary_indices.clone();
    let mut all_3d: Vec<Point3d> = boundary_3d.to_vec();

    for (hi, hole_3d) in holes_3d.iter().enumerate() {
        all_3d.extend_from_slice(hole_3d);
        if holes_2d.len() > hi {
            all_2d.extend_from_slice(&holes_2d[hi]);
        }
        if hole_indices.len() > hi {
            all_pool_indices.extend_from_slice(&hole_indices[hi]);
        }
    }

    // Add interior grid points for better surface approximation
    let n_interior = add_interior_grid_points(
        &boundary_2d, surface, &all_3d, &mut all_2d, &mut all_pool_indices, pool, params, face_id,
    );

    let n_boundary = boundary_3d.len();
    let n_holes: usize = hole_indices.iter().map(|h| h.len()).sum();

    // Build CDT
    let cdt_vertices: Vec<CdtVertex> = all_2d.iter().zip(all_pool_indices.iter())
        .map(|(pos, &idx)| CdtVertex { pos: *pos, index_3d: idx })
        .collect();

    let mut cdt: ConstrainedDelaunayTriangulation<CdtVertex> = ConstrainedDelaunayTriangulation::new();

    // Insert all vertices
    let mut cdt_handle_map: Vec<usize> = Vec::with_capacity(cdt_vertices.len());
    for v in &cdt_vertices {
        match cdt.insert(v.clone()) {
            Ok(handle) => cdt_handle_map.push(handle.index()),
            Err(_) => {
                // Duplicate — find existing
                let mut found = 0;
                for (i, v2) in cdt.vertices().enumerate() {
                    if v2.data().index_3d == v.index_3d {
                        found = i;
                        break;
                    }
                }
                cdt_handle_map.push(found);
            }
        }
    }

    // Add boundary constraints (outer ring)
    for i in 0..n_boundary {
        let j = (i + 1) % n_boundary;
        let hi = FixedVertexHandle::from_index(cdt_handle_map[i]);
        let hj = FixedVertexHandle::from_index(cdt_handle_map[j]);
        if hi != hj {
            let _ = cdt.add_constraint(hi, hj);
        }
    }

    // Add hole constraints
    let mut offset = n_boundary;
    for hole_idx in &hole_indices {
        let n_hole = hole_idx.len();
        for i in 0..n_hole {
            let j = (i + 1) % n_hole;
            let gi = offset + i;
            let gj = offset + j;
            if gi < cdt_handle_map.len() && gj < cdt_handle_map.len() {
                let hi = FixedVertexHandle::from_index(cdt_handle_map[gi]);
                let hj = FixedVertexHandle::from_index(cdt_handle_map[gj]);
                if hi != hj {
                    let _ = cdt.add_constraint(hi, hj);
                }
            }
        }
        offset += n_hole;
    }

    // Extract triangles from CDT
    let cdt_to_pool: Vec<u32> = cdt.vertices().map(|v| v.data().index_3d).collect();

    let boundary_2d_pts: Vec<Point2d> = boundary_2d.iter()
        .map(|uv| Point2d::new(uv[0], uv[1]))
        .collect();

    for face_handle in cdt.inner_faces() {
        let edges = face_handle.adjacent_edges();
        let vs = [edges[0].from(), edges[1].from(), edges[2].from()];

        let a = cdt_to_pool[vs[0].fix().index()];
        let b = cdt_to_pool[vs[1].fix().index()];
        let c = cdt_to_pool[vs[2].fix().index()];

        if a == b || b == c || a == c {
            continue;
        }

        // Check if triangle is inside the polygon
        let v0 = vs[0].data();
        let v1 = vs[1].data();
        let v2 = vs[2].data();
        let centroid_u = (v0.pos[0] + v1.pos[0] + v2.pos[0]) / 3.0;
        let centroid_v = (v0.pos[1] + v1.pos[1] + v2.pos[1]) / 3.0;
        let centroid = Point2d::new(centroid_u, centroid_v);

        if !point_in_polygon(&centroid, &boundary_2d_pts) {
            continue;
        }

        // Check if centroid is inside any hole
        let mut in_hole = false;
        let mut hole_offset = n_boundary;
        for (hi, hole_idx) in hole_indices.iter().enumerate() {
            if hi < holes_2d.len() {
                let hole_2d_pts: Vec<Point2d> = holes_2d[hi].iter()
                    .map(|uv| Point2d::new(uv[0], uv[1]))
                    .collect();
                if point_in_polygon(&centroid, &hole_2d_pts) {
                    in_hole = true;
                    break;
                }
            }
            hole_offset += hole_idx.len();
        }
        if in_hole {
            continue;
        }

        if forward {
            mesh.add_triangle(a, b, c);
        } else {
            mesh.add_triangle(a, c, b);
        }
        mesh.triangle_face_ids.get_or_insert_with(Vec::new).push(face_id);
    }

    mesh
}

/// Project a 3D point to a surface's 2D UV parametric space.
///
/// Uses `Surface::project_point` which supports ALL surface types
/// including NURBS, Revolution, and Extrusion.
fn project_to_surface_uv(p: &Point3d, surface: &Surface) -> [f64; 2] {
    let (u, v) = surface.project_point(p);
    [u, v]
}

/// Triangulate a curved surface using our custom CDT implementation.
///
/// This replaces spade CDT with our custom earcutr + Bowyer-Watson CDT
/// that guarantees ALL boundary vertices appear as triangle vertices
/// and ALL boundary edges appear as triangle edges.
///
/// Algorithm:
/// 1. Project boundary points to 2D UV space
/// 2. Generate interior Steiner points on the surface
/// 3. Run custom CDT (earcutr for boundary + Bowyer-Watson for interior)
/// 4. Map 2D triangles back to 3D vertex pool indices
fn triangulate_surface_custom_cdt(
    face: &Face,
    surface: &Surface,
    boundary_3d: &[Point3d],
    holes_3d: &[Vec<Point3d>],
    forward: bool,
    pool: &mut UnifiedVertexPool,
    params: &TriangulationParams,
) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    if boundary_3d.len() < 3 {
        return mesh;
    }

    let face_id = face.id.to_u64();

    // Insert boundary vertices into the unified pool
    let boundary_indices: Vec<u32> = boundary_3d.iter().map(|p| pool.insert_with_face(*p, face_id)).collect();
    let hole_indices: Vec<Vec<u32>> = holes_3d.iter()
        .map(|hole| hole.iter().map(|p| pool.insert_with_face(*p, face_id)).collect())
        .collect();

    // Project boundary points to 2D UV space
    let boundary_2d: Vec<[f64; 2]> = boundary_3d.iter()
        .map(|p| project_to_surface_uv(p, surface))
        .collect();

    // Check if UV projection is valid (no NaN/Inf)
    if boundary_2d.iter().any(|uv| !uv[0].is_finite() || !uv[1].is_finite()) {
        log::warn!("Custom CDT fallback: UV projection produced NaN/Inf for face {}", face.id);
        return triangulate_curved_face_consistent(face, boundary_3d, params, pool);
    }

    log::info!(
        "Custom CDT: face {} type={:?}, {} boundary pts, {} holes",
        face.id,
        std::mem::discriminant(surface),
        boundary_3d.len(),
        holes_3d.len(),
    );

    // Project hole points to 2D
    let holes_2d: Vec<Vec<[f64; 2]>> = holes_3d.iter()
        .map(|hole| hole.iter()
            .map(|p| project_to_surface_uv(p, surface))
            .collect())
        .collect();

    // Check hole UV validity
    if holes_2d.iter().any(|h| h.iter().any(|uv| !uv[0].is_finite() || !uv[1].is_finite())) {
        log::warn!("Custom CDT fallback: hole UV projection NaN/Inf for face {}", face.id);
        return triangulate_curved_face_consistent(face, boundary_3d, params, pool);
    }

    // Generate interior Steiner points on the surface
    let mut interior_2d: Vec<[f64; 2]> = Vec::new();
    let mut interior_pool_indices: Vec<u32> = Vec::new();
    generate_interior_points(
        &boundary_2d, surface, &mut interior_2d, &mut interior_pool_indices, pool, params, face_id,
    );

    // Run custom CDT triangulation
    let cdt_triangles = custom_cdt::triangulate_polygon_cdt(
        &boundary_2d, &holes_2d, &interior_2d,
    );

    // Build the combined index map:
    // CDT indices reference: [boundary][hole0][hole1]...[interior]
    // We need to map these to pool indices
    let mut cdt_to_pool: Vec<u32> = boundary_indices.clone();
    for hole_idx in &hole_indices {
        cdt_to_pool.extend_from_slice(hole_idx);
    }
    cdt_to_pool.extend_from_slice(&interior_pool_indices);

    // Convert CDT triangles to mesh triangles using pool indices
    for tri in &cdt_triangles {
        let a = cdt_to_pool.get(tri[0] as usize).copied().unwrap_or(tri[0]);
        let b = cdt_to_pool.get(tri[1] as usize).copied().unwrap_or(tri[1]);
        let c = cdt_to_pool.get(tri[2] as usize).copied().unwrap_or(tri[2]);

        if a == b || b == c || a == c {
            continue;
        }

        if forward {
            mesh.add_triangle(a, b, c);
        } else {
            mesh.add_triangle(a, c, b);
        }
        mesh.triangle_face_ids.get_or_insert_with(Vec::new).push(face_id);
    }

    mesh
}

/// Generate interior Steiner points on a surface for better curvature approximation.
///
/// Creates a grid of interior points in UV space that lie inside the boundary polygon.
/// These points help the CDT better approximate the surface curvature.
fn generate_interior_points(
    boundary_2d: &[[f64; 2]],
    surface: &Surface,
    interior_2d: &mut Vec<[f64; 2]>,
    interior_pool_indices: &mut Vec<u32>,
    pool: &mut UnifiedVertexPool,
    params: &TriangulationParams,
    face_id: u64,
) {
    // Compute UV bounding box from boundary
    let mut u_min = f64::MAX;
    let mut u_max = f64::MIN;
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;
    for uv in boundary_2d.iter() {
        u_min = u_min.min(uv[0]);
        u_max = u_max.max(uv[0]);
        v_min = v_min.min(uv[1]);
        v_max = v_max.max(uv[1]);
    }

    // Determine grid resolution based on surface type and params
    let (n_u, n_v) = match surface {
        Surface::Cylinder(_) | Surface::Cone(_) => {
            (params.angular_samples.min(48), params.height_samples.max(4))
        }
        Surface::Sphere(_) => {
            (params.angular_samples.min(48), params.angular_samples.min(24))
        }
        Surface::Torus(_) => {
            (params.angular_samples.min(48), params.angular_samples.min(24))
        }
        Surface::Revolution(_) => {
            (params.angular_samples.min(48), params.height_samples.max(4))
        }
        Surface::Extrusion(_) => {
            (params.height_samples.max(4), params.angular_samples.min(16))
        }
        Surface::Nurbs(_) => {
            // NURBS: higher resolution for better surface quality
            (params.angular_samples.max(12).min(48), params.angular_samples.max(12).min(48))
        }
        _ => (16, 8),
    };

    // Cap grid resolution
    let (n_u, n_v) = {
        let max_tris = params.max_face_triangles;
        let approx_tris = 2 * n_u * n_v;
        if approx_tris > max_tris {
            let scale = (max_tris as f64 / approx_tris as f64).sqrt();
            let nu = ((n_u as f64 * scale).ceil() as usize).max(4);
            let nv = ((n_v as f64 * scale).ceil() as usize).max(2);
            (nu, nv)
        } else {
            (n_u, n_v)
        }
    };

    let du = (u_max - u_min) / n_u as f64;
    let dv = (v_max - v_min) / n_v as f64;

    let boundary_pts: Vec<[f64; 2]> = boundary_2d.to_vec();

    for i in 1..n_u {
        for j in 1..n_v {
            let u = u_min + i as f64 * du;
            let v = v_min + j as f64 * dv;

            // Check if (u, v) is inside the boundary polygon
            if !custom_cdt::point_in_polygon([u, v], &boundary_pts) {
                continue;
            }

            // Get 3D point on surface
            let p3d = surface.point_at(u, v);
            if !p3d.x.is_finite() || !p3d.y.is_finite() || !p3d.z.is_finite() {
                continue;
            }

            let pool_idx = pool.insert_with_face(p3d, face_id);
            interior_2d.push([u, v]);
            interior_pool_indices.push(pool_idx);
        }
    }
}

/// Add interior grid points for curved surface approximation.
///
/// For curved surfaces, we need interior points to properly approximate
/// the curvature. This function generates a grid of interior points
/// in UV space and adds them to the CDT.
fn add_interior_grid_points(
    boundary_2d: &[[f64; 2]],
    surface: &Surface,
    all_3d: &[Point3d],
    all_2d: &mut Vec<[f64; 2]>,
    all_pool_indices: &mut Vec<u32>,
    pool: &mut UnifiedVertexPool,
    params: &TriangulationParams,
    face_id: u64,
) -> usize {
    // Compute UV bounding box from boundary
    let mut u_min = f64::MAX;
    let mut u_max = f64::MIN;
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;
    for uv in boundary_2d.iter() {
        u_min = u_min.min(uv[0]);
        u_max = u_max.max(uv[0]);
        v_min = v_min.min(uv[1]);
        v_max = v_max.max(uv[1]);
    }

    // Determine grid resolution based on surface type and params
    let (n_u, n_v) = match surface {
        Surface::Cylinder(_) | Surface::Cone(_) => {
            let n_u = params.angular_samples.min(64);
            let n_v = params.height_samples.max(4);
            (n_u, n_v)
        }
        Surface::Sphere(_) => {
            let n_u = params.angular_samples.min(64);
            let n_v = params.angular_samples.min(32);
            (n_u, n_v)
        }
        Surface::Torus(_) => {
            let n_u = params.angular_samples.min(64);
            let n_v = params.angular_samples.min(32);
            (n_u, n_v)
        }
        Surface::Revolution(_) => {
            let n_u = params.angular_samples.min(64);
            let n_v = params.height_samples.max(4);
            (n_u, n_v)
        }
        Surface::Extrusion(_) => {
            let n_u = params.height_samples.max(4);
            let n_v = params.angular_samples.min(16);
            (n_u, n_v)
        }
        Surface::Nurbs(_) => {
            // NURBS: moderate resolution grid
            let n_u = params.height_samples.max(8).min(32);
            let n_v = params.height_samples.max(8).min(32);
            (n_u, n_v)
        }
        _ => (16, 8),
    };

    // Cap grid resolution
    let (n_u, n_v) = {
        let max_tris = params.max_face_triangles;
        let approx_tris = 2 * n_u * n_v;
        if approx_tris > max_tris {
            let scale = (max_tris as f64 / approx_tris as f64).sqrt();
            let nu = ((n_u as f64 * scale).ceil() as usize).max(4);
            let nv = ((n_v as f64 * scale).ceil() as usize).max(2);
            (nu, nv)
        } else {
            (n_u, n_v)
        }
    };

    let mut count = 0;
    let du = (u_max - u_min) / n_u as f64;
    let dv = (v_max - v_min) / n_v as f64;

    for i in 1..n_u {
        for j in 1..n_v {
            let u = u_min + i as f64 * du;
            let v = v_min + j as f64 * dv;

            // Get 3D point on surface
            let p3d = surface_point_at(surface, u, v);
            if !p3d.x.is_finite() || !p3d.y.is_finite() || !p3d.z.is_finite() {
                continue;
            }

            // Check if (u, v) is inside the boundary polygon
            let pt = Point2d::new(u, v);
            let boundary_pts: Vec<Point2d> = boundary_2d.iter()
                .map(|uv| Point2d::new(uv[0], uv[1]))
                .collect();

            if !point_in_polygon(&pt, &boundary_pts) {
                continue;
            }

            let pool_idx = pool.insert_with_face(p3d, face_id);
            all_2d.push([u, v]);
            all_pool_indices.push(pool_idx);
            count += 1;
        }
    }

    count
}

/// Evaluate a 3D point on a surface at UV parameters.
/// Uses `Surface::point_at` which supports ALL surface types.
fn surface_point_at(surface: &Surface, u: f64, v: f64) -> Point3d {
    surface.point_at(u, v)
}

/// CDT triangulation for cylindrical surfaces.
fn triangulate_cylinder_cdt(
    face: &Face,
    cyl: &CylinderSurface,
    boundary_3d: &[Point3d],
    holes_3d: &[Vec<Point3d>],
    forward: bool,
    pool: &mut UnifiedVertexPool,
    params: &TriangulationParams,
) -> TriangleMesh {
    triangulate_surface_custom_cdt(face, &Surface::Cylinder(cyl.clone()), boundary_3d, holes_3d, forward, pool, params)
}

/// CDT triangulation for conical surfaces.
fn triangulate_cone_cdt(
    face: &Face,
    cone: &ConeSurface,
    boundary_3d: &[Point3d],
    holes_3d: &[Vec<Point3d>],
    forward: bool,
    pool: &mut UnifiedVertexPool,
    params: &TriangulationParams,
) -> TriangleMesh {
    triangulate_surface_custom_cdt(face, &Surface::Cone(cone.clone()), boundary_3d, holes_3d, forward, pool, params)
}

/// CDT triangulation for spherical surfaces.
fn triangulate_sphere_cdt(
    face: &Face,
    sphere: &SphereSurface,
    boundary_3d: &[Point3d],
    holes_3d: &[Vec<Point3d>],
    forward: bool,
    pool: &mut UnifiedVertexPool,
    params: &TriangulationParams,
) -> TriangleMesh {
    triangulate_surface_custom_cdt(face, &Surface::Sphere(sphere.clone()), boundary_3d, holes_3d, forward, pool, params)
}

/// CDT triangulation for toroidal surfaces.
fn triangulate_torus_cdt(
    face: &Face,
    torus: &TorusSurface,
    boundary_3d: &[Point3d],
    holes_3d: &[Vec<Point3d>],
    forward: bool,
    pool: &mut UnifiedVertexPool,
    params: &TriangulationParams,
) -> TriangleMesh {
    triangulate_surface_custom_cdt(face, &Surface::Torus(torus.clone()), boundary_3d, holes_3d, forward, pool, params)
}

// ============================================================
// Face-aware boundary closing and non-manifold resolution
// ============================================================

/// Close boundary edges by merging nearby vertices, but ONLY if merging
/// won't create a non-manifold edge.
///
/// The algorithm:
/// 1. Find all boundary edges (edges with count == 1)
/// 2. For each pair of nearby boundary vertices:
///    - Check if merging would create a non-manifold edge
///    - If safe, merge them
/// 3. Apply the merge remap and filter degenerate triangles
///
/// A merge creates a non-manifold edge when the merged vertex participates
/// in an edge that already has 2 adjacent triangles. After merging, that
/// edge would have 3+ triangles = non-manifold.
fn face_aware_close_boundary(mesh: &mut TriangleMesh, base_tolerance: f64) {
    let tolerances = [
        base_tolerance * 2.0,
        base_tolerance * 5.0,
        base_tolerance * 10.0,
        base_tolerance * 50.0,
        base_tolerance * 100.0,
    ];

    for &tol in &tolerances {
        let report = crate::watertight::validate_watertight(mesh, false);
        if report.is_watertight() {
            return;
        }
        if report.boundary_edge_count == 0 {
            return;
        }

        log::info!(
            "Face-aware closing: {} boundary edges remaining, trying tol={:.6}",
            report.boundary_edge_count, tol,
        );

        // Build edge count map for non-manifold checking
        let edge_counts = build_edge_count_map(mesh);

        // Collect boundary vertices
        let boundary_verts = collect_boundary_vertex_set(mesh);

        // Build spatial index of boundary vertices
        let cell_size = tol * 10.0;
        let tol_sq = tol * tol;
        let mut grid: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
        for &vidx in &boundary_verts {
            let p = mesh.vertices[vidx as usize];
            let cx = (p.x / cell_size).floor() as i64;
            let cy = (p.y / cell_size).floor() as i64;
            let cz = (p.z / cell_size).floor() as i64;
            grid.entry((cx, cy, cz)).or_default().push(vidx);
        }

        // For each boundary vertex, find the closest other boundary vertex
        let mut remap: Vec<u32> = (0..mesh.vertices.len() as u32).collect();
        let mut merged_any = false;

        for &vidx in &boundary_verts {
            // Follow remap chain
            let mut v1 = vidx;
            while remap[v1 as usize] != v1 {
                v1 = remap[v1 as usize];
            }
            let p = mesh.vertices[v1 as usize];
            let cx = (p.x / cell_size).floor() as i64;
            let cy = (p.y / cell_size).floor() as i64;
            let cz = (p.z / cell_size).floor() as i64;

            let mut best_match: Option<(u32, f64)> = None;

            for dx in -1i64..=1 {
                for dy in -1i64..=1 {
                    for dz in -1i64..=1 {
                        let key = (cx + dx, cy + dy, cz + dz);
                        if let Some(indices) = grid.get(&key) {
                            for &other_idx in indices {
                                // Follow remap chain
                                let mut v2 = other_idx;
                                while remap[v2 as usize] != v2 {
                                    v2 = remap[v2 as usize];
                                }
                                if v2 == v1 {
                                    continue; // Already merged
                                }

                                let other_p = mesh.vertices[v2 as usize];
                                let ddx = p.x - other_p.x;
                                let ddy = p.y - other_p.y;
                                let ddz = p.z - other_p.z;
                                let dist_sq = ddx * ddx + ddy * ddy + ddz * ddz;

                                if dist_sq < tol_sq {
                                    // Check if merging v1 into v2 would create a non-manifold edge.
                                    // This happens when an edge from v1 to some neighbor Vn
                                    // would overlap with an existing edge from v2 to Vn
                                    // that already has count == 2.
                                    if would_create_nonmanifold(v1, v2, &edge_counts, mesh) {
                                        continue;
                                    }

                                    match best_match {
                                        None => best_match = Some((v2, dist_sq)),
                                        Some((_, best_dist)) => {
                                            if dist_sq < best_dist {
                                                best_match = Some((v2, dist_sq));
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            if let Some((target, _)) = best_match {
                remap[v1 as usize] = target;
                merged_any = true;
            }
        }

        if !merged_any {
            continue;
        }

        // Apply remap
        apply_face_aware_remap(mesh, &remap);
        filter_degenerate_triangles(mesh, 1e-10);
    }
}

/// Build a map from edge (canonical vertex pair) → count of adjacent triangles.
fn build_edge_count_map(mesh: &TriangleMesh) -> HashMap<(u32, u32), u32> {
    let mut edge_counts: HashMap<(u32, u32), u32> = HashMap::new();
    for tri in &mesh.triangles {
        let edges = [
            (tri[0].min(tri[1]), tri[0].max(tri[1])),
            (tri[1].min(tri[2]), tri[1].max(tri[2])),
            (tri[2].min(tri[0]), tri[2].max(tri[0])),
        ];
        for edge in &edges {
            *edge_counts.entry(*edge).or_insert(0) += 1;
        }
    }
    edge_counts
}

/// Check if merging vertex v1 into v2 would create a non-manifold edge.
///
/// After merging, all triangles that used v1 will now use v2. This means
/// all edges of the form (v1, neighbor) become edges (v2, neighbor).
/// If an edge (v2, neighbor) already exists with count == 2, the merged
/// edge would have count > 2 → non-manifold.
fn would_create_nonmanifold(
    v1: u32,
    v2: u32,
    edge_counts: &HashMap<(u32, u32), u32>,
    mesh: &TriangleMesh,
) -> bool {
    // Find all neighbors of v1 (vertices connected to v1 by an edge)
    let v1_neighbors: std::collections::HashSet<u32> = mesh.triangles.iter()
        .flat_map(|tri| {
            let mut neighbors = Vec::new();
            if tri[0] == v1 { neighbors.push(tri[1]); neighbors.push(tri[2]); }
            if tri[1] == v1 { neighbors.push(tri[0]); neighbors.push(tri[2]); }
            if tri[2] == v1 { neighbors.push(tri[0]); neighbors.push(tri[1]); }
            neighbors
        })
        .filter(|&n| n != v1 && n != v2)
        .collect();

    // Check if any edge (v2, neighbor) already has count >= 2
    for &neighbor in &v1_neighbors {
        let edge = (v2.min(neighbor), v2.max(neighbor));
        if let Some(&count) = edge_counts.get(&edge) {
            if count >= 2 {
                return true; // Would create non-manifold edge
            }
        }
    }

    false
}

/// Build a map from vertex index to set of face IDs that use that vertex.
fn build_vertex_face_map(mesh: &TriangleMesh) -> Vec<std::collections::HashSet<u64>> {
    let n = mesh.vertices.len();
    let mut map: Vec<std::collections::HashSet<u64>> = vec![std::collections::HashSet::new(); n];

    for (tri_idx, tri) in mesh.triangles.iter().enumerate() {
        let face_id = mesh.triangle_face_ids.as_ref()
            .and_then(|ids| ids.get(tri_idx).copied())
            .unwrap_or(0);

        for &v in tri.iter() {
            if (v as usize) < n {
                map[v as usize].insert(face_id);
            }
        }
    }

    map
}

/// Collect the set of vertex indices that appear on boundary edges.
fn collect_boundary_vertex_set(mesh: &TriangleMesh) -> std::collections::HashSet<u32> {
    let mut edge_count: HashMap<(u32, u32), u32> = HashMap::new();
    for tri in &mesh.triangles {
        let edges = [
            (tri[0].min(tri[1]), tri[0].max(tri[1])),
            (tri[1].min(tri[2]), tri[1].max(tri[2])),
            (tri[2].min(tri[0]), tri[2].max(tri[0])),
        ];
        for edge in &edges {
            *edge_count.entry(*edge).or_insert(0) += 1;
        }
    }

    let mut boundary = std::collections::HashSet::new();
    for (edge, count) in &edge_count {
        if *count == 1 {
            boundary.insert(edge.0);
            boundary.insert(edge.1);
        }
    }
    boundary
}

/// Apply a vertex remap to all triangles, preserving face_ids and normals.
fn apply_face_aware_remap(mesh: &mut TriangleMesh, remap: &[u32]) {
    // First, follow remap chains to get final targets
    let mut final_remap: Vec<u32> = Vec::with_capacity(remap.len());
    for &target in remap {
        let mut current = target;
        loop {
            let next = remap[current as usize];
            if next == current {
                break;
            }
            current = next;
        }
        final_remap.push(current);
    }

    // Apply remap to triangles, filtering degenerate ones
    let old_triangles = std::mem::take(&mut mesh.triangles);
    let old_face_ids = mesh.triangle_face_ids.take();
    let old_face_normals = mesh.face_normals.take();
    let old_triangle_colors = mesh.triangle_colors.take();

    for (i, tri) in old_triangles.iter().enumerate() {
        let a = final_remap[tri[0] as usize];
        let b = final_remap[tri[1] as usize];
        let c = final_remap[tri[2] as usize];

        if a != b && b != c && a != c {
            mesh.triangles.push([a, b, c]);
            if let Some(ref ids) = old_face_ids {
                if let Some(&fid) = ids.get(i) {
                    mesh.triangle_face_ids.get_or_insert_with(Vec::new).push(fid);
                }
            }
            if let Some(ref normals) = old_face_normals {
                if let Some(&n) = normals.get(i) {
                    mesh.face_normals.get_or_insert_with(Vec::new).push(n);
                }
            }
            if let Some(ref colors) = old_triangle_colors {
                if let Some(&col) = colors.get(i) {
                    mesh.triangle_colors.get_or_insert_with(Vec::new).push(col);
                }
            }
        }
    }

    // Compact vertices
    crate::watertight::compact_vertices(mesh);
}

/// Resolve non-manifold edges by splitting vertices.
///
/// A non-manifold edge is shared by more than 2 triangles. This function
/// resolves them by:
/// 1. Finding all non-manifold edges (count > 2)
/// 2. For each non-manifold edge, identifying which triangles share it
/// 3. Pairing triangles from different faces and keeping those pairs
/// 4. For excess triangles (from the same face as an existing one),
///   duplicating the edge vertices
///
/// After splitting, boundary edges may appear where split vertices
/// couldn't be merged. The face-aware closing step handles these.
fn resolve_non_manifold_edges(mesh: &mut TriangleMesh) {
    for _iteration in 0..10 {
        let report = crate::watertight::validate_watertight(mesh, false);
        if report.non_manifold_edge_count == 0 {
            return;
        }

        log::info!(
            "Non-manifold resolution: {} non-manifold edges, {} boundary edges, {} vertices, {} triangles",
            report.non_manifold_edge_count, report.boundary_edge_count,
            report.vertex_count, report.triangle_count,
        );

        // Build edge → triangle indices map
        let mut edge_triangles: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
        for (tri_idx, tri) in mesh.triangles.iter().enumerate() {
            let edges = [
                (tri[0].min(tri[1]), tri[0].max(tri[1])),
                (tri[1].min(tri[2]), tri[1].max(tri[2])),
                (tri[2].min(tri[0]), tri[2].max(tri[0])),
            ];
            for edge in &edges {
                edge_triangles.entry(*edge).or_default().push(tri_idx);
            }
        }

        // Find non-manifold edges (count > 2)
        let non_manifold: Vec<((u32, u32), Vec<usize>)> = edge_triangles.iter()
            .filter(|(_, tris)| tris.len() > 2)
            .map(|(&edge, tris)| (edge, tris.clone()))
            .collect();

        if non_manifold.is_empty() {
            return;
        }

        // For each non-manifold edge, split excess triangles by
        // duplicating their vertices
        let mut split_count = 0;
        for (edge, tri_indices) in &non_manifold {
            if tri_indices.len() <= 2 {
                continue;
            }

            // Group triangles by face_id, keeping track of which face each belongs to
            let tri_faces: Vec<(usize, u64)> = tri_indices.iter().map(|&tri_idx| {
                let face_id = mesh.triangle_face_ids.as_ref()
                    .and_then(|ids| ids.get(tri_idx).copied())
                    .unwrap_or(0);
                (tri_idx, face_id)
            }).collect();

            // Strategy: keep one triangle per unique face, up to 2 total.
            // For any additional triangles (from a face that already has a
            // triangle on this edge), duplicate their edge vertices.
            let mut seen_faces: std::collections::HashMap<u64, usize> = std::collections::HashMap::new();
            let mut kept_tri_indices: Vec<usize> = Vec::new();

            for &(tri_idx, face_id) in &tri_faces {
                if seen_faces.len() < 2 && !seen_faces.contains_key(&face_id) {
                    // Keep this triangle — it's from a new face
                    seen_faces.insert(face_id, tri_idx);
                    kept_tri_indices.push(tri_idx);
                }
                // Otherwise, it will be split below
            }

            // If we couldn't find 2 different faces, just keep first 2 triangles
            if kept_tri_indices.len() < 2 {
                kept_tri_indices.clear();
                kept_tri_indices.push(tri_indices[0]);
                if tri_indices.len() > 1 {
                    kept_tri_indices.push(tri_indices[1]);
                }
            }

            // Split all triangles NOT in kept_tri_indices
            for &(tri_idx, _face_id) in &tri_faces {
                if kept_tri_indices.contains(&tri_idx) {
                    continue;
                }

                // Duplicate edge vertices for this triangle
                let tri = &mut mesh.triangles[tri_idx];
                let (ea, eb) = *edge;
                for v_idx in tri.iter_mut() {
                    let v = *v_idx;
                    if v == ea || v == eb {
                        // Create a new vertex copy at the same position
                        let new_idx = mesh.vertices.len() as u32;
                        mesh.vertices.push(mesh.vertices[v as usize]);
                        *v_idx = new_idx;
                        split_count += 1;
                    }
                }
            }
        }

        if split_count == 0 {
            // No progress — break to avoid infinite loop
            break;
        }

        filter_degenerate_triangles(mesh, 1e-10);
    }
}


/// Triangulate a face using CDT with consistent edge sampling.
fn triangulate_face_cdt(
    face: &Face,
    cache: &ConsistentEdgeCache,
    params: &TriangulationParams,
    pool: &mut UnifiedVertexPool,
) -> TriangleMesh {
    let boundary_3d = collect_cached_boundary_points(face, cache);
    if boundary_3d.is_empty() {
        return TriangleMesh::new();
    }

    let holes_3d = collect_cached_hole_points(face, cache);
    let forward = face.forward;

    let surface_type = match &face.surface {
        Some(s) => match s {
            Surface::Plane(_) => "Plane",
            Surface::Cylinder(_) => "Cylinder",
            Surface::Cone(_) => "Cone",
            Surface::Sphere(_) => "Sphere",
            Surface::Torus(_) => "Torus",
            Surface::Revolution(_) => "Revolution",
            Surface::Extrusion(_) => "Extrusion",
            Surface::Nurbs(_) => "Nurbs",
        },
        None => "None",
    };
    log::info!("CDT face {}: type={}, {} boundary pts, {} holes, forward={}", 
        face.id, surface_type, boundary_3d.len(), holes_3d.len(), forward);

    match &face.surface {
        Some(Surface::Plane(plane)) => {
            triangulate_planar_cdt(face, plane, &boundary_3d, &holes_3d, forward, pool)
        }
        Some(Surface::Cylinder(cyl)) => {
            triangulate_cylinder_cdt(face, cyl, &boundary_3d, &holes_3d, forward, pool, params)
        }
        Some(Surface::Cone(cone)) => {
            triangulate_cone_cdt(face, cone, &boundary_3d, &holes_3d, forward, pool, params)
        }
        Some(Surface::Sphere(sphere)) => {
            triangulate_sphere_cdt(face, sphere, &boundary_3d, &holes_3d, forward, pool, params)
        }
        Some(Surface::Torus(torus)) => {
            triangulate_torus_cdt(face, torus, &boundary_3d, &holes_3d, forward, pool, params)
        }
        Some(Surface::Revolution(_)) | Some(Surface::Extrusion(_)) | Some(Surface::Nurbs(_)) => {
            // For NURBS, revolution, extrusion surfaces:
            // Try CDT first; if UV projection fails, fall back to snap-boundary
            if let Some(ref surface) = face.surface {
                // Check if UV projection works for this surface
                let test_uv: Vec<[f64; 2]> = boundary_3d.iter()
                    .take(5)
                    .map(|p| project_to_surface_uv(p, surface))
                    .collect();
                let uv_valid = test_uv.iter().all(|uv| uv[0].is_finite() && uv[1].is_finite());

                if uv_valid {
                    triangulate_surface_cdt_generic(face, surface, &boundary_3d, &holes_3d, forward, pool, params)
                } else {
                    triangulate_curved_face_consistent(face, &boundary_3d, params, pool)
                }
            } else {
                triangulate_curved_face_consistent(face, &boundary_3d, params, pool)
            }
        }
        _ => {
            // For unknown surfaces: use snap-boundary approach
            triangulate_curved_face_consistent(face, &boundary_3d, params, pool)
        }
    }
}

/// A unified vertex pool that merges vertices across all faces of a solid.
///
/// When two faces share an edge, their boundary vertices should be the same.
/// This pool ensures that by:
/// 1. Inserting all vertices with 3D spatial hashing
/// 2. Merging vertices within the given tolerance, but ONLY if they belong
///    to DIFFERENT faces (face-aware merging prevents non-manifold edges)
/// 3. Mapping each face's local vertex indices to unified indices
#[derive(Clone, Debug)]
pub struct UnifiedVertexPool {
    /// All unique vertices (after merging).
    pub vertices: Vec<Point3d>,
    /// Spatial hash grid for fast near-neighbor lookup.
    grid: HashMap<(i64, i64, i64), Vec<u32>>,
    /// Cell size for the spatial hash grid.
    cell_size: f64,
    /// Tolerance for vertex merging (squared).
    tolerance_sq: f64,
    /// Face ID for each vertex (used for face-aware merging).
    vertex_face_ids: Vec<u64>,
    /// Whether to use face-aware merging (only merge vertices from different faces).
    face_aware: bool,
}

impl UnifiedVertexPool {
    /// Create a new vertex pool with the given merge tolerance.
    pub fn new(tolerance: f64) -> Self {
        Self {
            vertices: Vec::new(),
            grid: HashMap::new(),
            cell_size: tolerance * 10.0,
            tolerance_sq: tolerance * tolerance,
            vertex_face_ids: Vec::new(),
            face_aware: false,
        }
    }

    /// Create a new vertex pool with face-aware merging.
    ///
    /// When `face_aware` is true, vertices from the same face will NOT be
    /// merged even if they are within tolerance. This prevents non-manifold
    /// edges caused by over-aggressive vertex merging within a single face.
    pub fn new_face_aware(tolerance: f64) -> Self {
        Self {
            vertices: Vec::new(),
            grid: HashMap::new(),
            cell_size: tolerance * 10.0,
            tolerance_sq: tolerance * tolerance,
            vertex_face_ids: Vec::new(),
            face_aware: true,
        }
    }

    /// Insert a vertex and return its unified index.
    pub fn insert(&mut self, point: Point3d) -> u32 {
        self.insert_with_face(point, 0)
    }

    /// Insert a vertex with a face ID and return its unified index.
    ///
    /// When face-aware merging is enabled, only merges with vertices
    /// from a DIFFERENT face ID. This prevents vertices within the
    /// same face from being merged (which causes non-manifold edges).
    pub fn insert_with_face(&mut self, point: Point3d, face_id: u64) -> u32 {
        let cx = (point.x / self.cell_size).floor() as i64;
        let cy = (point.y / self.cell_size).floor() as i64;
        let cz = (point.z / self.cell_size).floor() as i64;

        // Collect all matching candidates, then pick the one with the
        // smallest index (deterministic choice regardless of hash order)
        let mut best_match: Option<(u32, f64)> = None;

        for dx in -1i64..=1 {
            for dy in -1i64..=1 {
                for dz in -1i64..=1 {
                    let key = (cx + dx, cy + dy, cz + dz);
                    if let Some(indices) = self.grid.get(&key) {
                        for &idx in indices {
                            let existing = &self.vertices[idx as usize];

                            // Face-aware check: only merge vertices from DIFFERENT faces
                            if self.face_aware && face_id != 0 {
                                if let Some(&existing_face) = self.vertex_face_ids.get(idx as usize) {
                                    if existing_face == face_id {
                                        // Same face — don't merge (prevents non-manifold edges)
                                        continue;
                                    }
                                }
                            }

                            let ddx = point.x - existing.x;
                            let ddy = point.y - existing.y;
                            let ddz = point.z - existing.z;
                            let dist_sq = ddx * ddx + ddy * ddy + ddz * ddz;
                            if dist_sq < self.tolerance_sq {
                                // Pick the candidate with the smallest index for determinism
                                match best_match {
                                    None => best_match = Some((idx, dist_sq)),
                                    Some((best_idx, best_dist)) => {
                                        // Prefer closer match; if equal distance, prefer smaller index
                                        if dist_sq < best_dist || (dist_sq == best_dist && idx < best_idx) {
                                            best_match = Some((idx, dist_sq));
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        if let Some((best_idx, _)) = best_match {
            return best_idx;
        }

        let new_idx = self.vertices.len() as u32;
        self.vertices.push(point);
        self.vertex_face_ids.push(face_id);
        self.grid.entry((cx, cy, cz)).or_default().push(new_idx);
        new_idx
    }

    /// Number of unique vertices.
    pub fn len(&self) -> usize {
        self.vertices.len()
    }

    /// Whether the pool is empty.
    pub fn is_empty(&self) -> bool {
        self.vertices.len() == 0
    }
}

/// Triangulate a solid into a watertight mesh using CDT with consistent edge
/// discretization and unified vertex merging.
pub fn triangulate_solid_watertight(
    solid: &Solid,
    params: &TriangulationParams,
    merge_tolerance: f64,
) -> TriangleMesh {
    let faces = solid.faces();
    if faces.is_empty() {
        return TriangleMesh::new();
    }

    // Phase 1: Build consistent edge discretization cache
    let edge_cache = ConsistentEdgeCache::build_from_solid(solid, CDT_EDGE_SAMPLES);

    log::info!(
        "triangulate_solid_watertight: {} faces, {} unique edges cached",
        faces.len(),
        edge_cache.entries.len(),
    );

    // Phase 2+3: Triangulate each face with consistent boundary vertices
    let mut pool = UnifiedVertexPool::new_face_aware(merge_tolerance);
    let mut all_triangles: Vec<[u32; 3]> = Vec::new();
    let mut all_face_ids: Vec<u64> = Vec::new();
    let mut all_face_normals: Vec<[f64; 3]> = Vec::new();

    for face in &faces {
        let face_mesh = triangulate_face_cdt(face, &edge_cache, params, &mut pool);
        for (tri_idx, tri) in face_mesh.triangles.iter().enumerate() {
            all_triangles.push(*tri);
            if let Some(ref ids) = face_mesh.triangle_face_ids {
                if let Some(&id) = ids.get(tri_idx) {
                    all_face_ids.push(id);
                }
            }
            if let Some(ref normals) = face_mesh.face_normals {
                if let Some(&n) = normals.get(tri_idx) {
                    all_face_normals.push(n);
                }
            }
        }
    }

    // Build final mesh from the unified vertex pool
    let mut mesh = TriangleMesh::new();
    for &point in &pool.vertices {
        mesh.add_vertex(point);
    }
    for tri in &all_triangles {
        if (tri[0] as usize) < pool.len()
            && (tri[1] as usize) < pool.len()
            && (tri[2] as usize) < pool.len()
        {
            if tri[0] != tri[1] && tri[1] != tri[2] && tri[0] != tri[2] {
                mesh.add_triangle(tri[0], tri[1], tri[2]);
            }
        }
    }

    if !all_face_ids.is_empty() {
        mesh.triangle_face_ids = Some(all_face_ids);
    }
    if !all_face_normals.is_empty() {
        mesh.face_normals = Some(all_face_normals);
    }

    // Ensure per-triangle arrays are consistent
    if let Some(ref mut ids) = mesh.triangle_face_ids {
        while ids.len() < mesh.triangles.len() {
            ids.push(0);
        }
    }
    if let Some(ref mut normals) = mesh.face_normals {
        while normals.len() < mesh.triangles.len() {
            normals.push([0.0, 0.0, 1.0]);
        }
    }

    log::info!(
        "triangulate_solid_watertight: {} unified vertices, {} triangles (merge_tol={:.6})",
        pool.len(),
        mesh.triangles.len(),
        merge_tolerance,
    );

    // Filter degenerate triangles
    filter_degenerate_triangles(&mut mesh, 1e-10);

    // Phase 5: Progressive edge stitching for remaining gaps
    // Use the standard stitch approach first, then fix any non-manifold edges
    let stitch_tolerances = [
        merge_tolerance * 2.0,
        merge_tolerance * 5.0,
        merge_tolerance * 10.0,
    ];

    for &stitch_tol in &stitch_tolerances {
        let report = crate::watertight::validate_watertight(&mesh, false);
        if report.is_watertight() {
            break;
        }
        if report.boundary_edge_count == 0 {
            break;
        }
        log::info!(
            "Edge stitching: {} boundary edges remaining, trying stitch_tol={:.6}",
            report.boundary_edge_count, stitch_tol,
        );
        stitch_boundary_edges(&mut mesh, stitch_tol, 3);
        filter_degenerate_triangles(&mut mesh, 1e-10);
    }

    // Phase 6: Resolve any non-manifold edges by splitting
    resolve_non_manifold_edges(&mut mesh);

    // Phase 7: Close boundary edges created by non-manifold splitting
    // using face-aware vertex merging (which won't create new non-manifold edges)
    // Rebuild edge counts after the non-manifold resolution
    face_aware_close_boundary(&mut mesh, merge_tolerance);

    // Phase 8: If still not watertight, iterate non-manifold resolution + closing
    for _round in 0..5 {
        let report = crate::watertight::validate_watertight(&mesh, false);
        if report.is_watertight() {
            break;
        }
        if report.non_manifold_edge_count == 0 && report.boundary_edge_count == 0 {
            break;
        }

        if report.non_manifold_edge_count > 0 {
            resolve_non_manifold_edges(&mut mesh);
        }
        face_aware_close_boundary(&mut mesh, merge_tolerance);

        // If face-aware closing couldn't close all edges, try aggressive
        // stitching which may create non-manifold edges, then resolve those
        let report2 = crate::watertight::validate_watertight(&mesh, false);
        if report2.boundary_edge_count > 0 && report2.boundary_edge_count <= 100 {
            stitch_boundary_edges(&mut mesh, merge_tolerance * 10.0, 3);
            filter_degenerate_triangles(&mut mesh, 1e-10);
            // Resolve any non-manifold edges created by the stitching
            resolve_non_manifold_edges(&mut mesh);
            face_aware_close_boundary(&mut mesh, merge_tolerance);
        }
    }

    // Final validation
    let report = crate::watertight::validate_watertight(&mesh, false);
    if !report.is_watertight() {
        log::warn!(
            "triangulate_solid_watertight: mesh still has {} boundary edges, {} non-manifold edges after processing",
            report.boundary_edge_count, report.non_manifold_edge_count,
        );
    } else {
        log::info!(
            "triangulate_solid_watertight: MESH IS WATERTIGHT! ({} vertices, {} triangles, χ={})",
            report.vertex_count, report.triangle_count, report.euler_characteristic,
        );
    }

    mesh
}

/// Compute an adaptive merge tolerance based on the solid's bounding box.
pub fn adaptive_merge_tolerance(
    solid: &Solid,
    factor: f64,
    min_tol: f64,
    max_tol: f64,
) -> f64 {
    let faces = solid.faces();
    let mut min_pt = Point3d::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max_pt = Point3d::new(f64::MIN, f64::MIN, f64::MIN);

    for face in &faces {
        for edge in &face.edges {
            if let Some(ref curve) = edge.curve {
                let (tmin, tmax) = edge.param_range;
                let (pmin, pmax) = if tmin <= tmax { (tmin, tmax) } else { (tmax, tmin) };
                let p0 = curve.point_at(pmin);
                let p1 = curve.point_at(pmax);
                let pm = curve.point_at((pmin + pmax) * 0.5);

                for p in &[p0, p1, pm] {
                    min_pt.x = min_pt.x.min(p.x);
                    min_pt.y = min_pt.y.min(p.y);
                    min_pt.z = min_pt.z.min(p.z);
                    max_pt.x = max_pt.x.max(p.x);
                    max_pt.y = max_pt.y.max(p.y);
                    max_pt.z = max_pt.z.max(p.z);
                }
            }
        }
    }

    let dx = max_pt.x - min_pt.x;
    let dy = max_pt.y - min_pt.y;
    let dz = max_pt.z - min_pt.z;
    let diagonal = (dx * dx + dy * dy + dz * dz).sqrt();

    let tol = diagonal * factor;
    tol.clamp(min_tol, max_tol)
}

#[cfg(test)]
mod tests {
    use super::*;
    use draper_geometry::Point3d;

    #[test]
    fn test_unified_vertex_pool_merge() {
        let mut pool = UnifiedVertexPool::new(1e-3);
        let a = pool.insert(Point3d::new(0.0, 0.0, 0.0));
        let b = pool.insert(Point3d::new(1.0, 0.0, 0.0));
        let c = pool.insert(Point3d::new(0.0001, 0.0001, 0.0001));
        assert_eq!(pool.len(), 2, "Should have 2 unique vertices (a≈c merged)");
        assert_eq!(a, c, "Close vertices should get the same index");
        assert_ne!(a, b, "Distinct vertices should get different indices");
    }

    #[test]
    fn test_unified_vertex_pool_no_false_merge() {
        let mut pool = UnifiedVertexPool::new(1e-3);
        let a = pool.insert(Point3d::new(0.0, 0.0, 0.0));
        let b = pool.insert(Point3d::new(0.1, 0.0, 0.0));
        assert_eq!(pool.len(), 2, "Should have 2 unique vertices");
        assert_ne!(a, b, "Vertices 0.1 apart should not merge at 1e-3 tolerance");
    }

    #[test]
    fn test_point_in_polygon() {
        let polygon = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(10.0, 0.0),
            Point2d::new(10.0, 10.0),
            Point2d::new(0.0, 10.0),
        ];
        assert!(point_in_polygon(&Point2d::new(5.0, 5.0), &polygon));
        assert!(!point_in_polygon(&Point2d::new(15.0, 5.0), &polygon));
        assert!(!point_in_polygon(&Point2d::new(-1.0, 5.0), &polygon));
    }
}
