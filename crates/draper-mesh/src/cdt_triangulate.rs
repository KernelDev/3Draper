// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! CDT-based solid triangulation with tolerance-aware vertex unification.
//!
//! This module provides watertight mesh generation by:
//! 1. Pre-discretizing ALL edges consistently (shared edges produce identical 3D points)
//! 2. For each face, collecting boundary points from the consistent edge cache
//! 3. Projecting boundary points to 2D and triangulating with CDT (spade)
//! 4. Mapping CDT triangles back to 3D using the surface parameterization
//! 5. Since shared edges use identical vertex indices, the result is watertight by construction

use crate::mesh::TriangleMesh;
use crate::triangulate::{
    TriangulationParams, triangulate_face, filter_degenerate_triangles,
    sample_edge_points,
};
use crate::watertight::{stitch_boundary_edges, zipper_stitch_boundary_edges};
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
fn project_to_surface_uv(p: &Point3d, surface: &Surface) -> [f64; 2] {
    match surface {
        Surface::Plane(plane) => project_to_plane(p, plane),
        Surface::Cylinder(cyl) => {
            let (u, v) = cyl.project_point(p);
            [u, v]
        }
        Surface::Cone(cone) => {
            let (u, v) = cone.project_point(p);
            [u, v]
        }
        Surface::Sphere(sphere) => {
            let (u, v) = sphere.project_point(p);
            [u, v]
        }
        Surface::Torus(torus) => {
            let (u, v) = torus.project_point(p);
            [u, v]
        }
        _ => {
            // Fallback: use XY projection (not great but prevents crash)
            [p.x, p.y]
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
        Surface::Cylinder(_) => {
            let n_u = params.angular_samples.min(64);
            let n_v = params.height_samples.max(4);
            (n_u, n_v)
        }
        Surface::Cone(_) => {
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
fn surface_point_at(surface: &Surface, u: f64, v: f64) -> Point3d {
    match surface {
        Surface::Plane(plane) => {
            Point3d::new(
                plane.origin.x + u * plane.u_dir.x + v * plane.v_dir.x,
                plane.origin.y + u * plane.u_dir.y + v * plane.v_dir.y,
                plane.origin.z + u * plane.u_dir.z + v * plane.v_dir.z,
            )
        }
        Surface::Cylinder(cyl) => cyl.point_at(u, v),
        Surface::Cone(cone) => cone.point_at(u, v),
        Surface::Sphere(sphere) => sphere.point_at(u, v),
        Surface::Torus(torus) => torus.point_at(u, v),
        _ => Point3d::new(u, v, 0.0),
    }
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
    triangulate_surface_cdt_generic(face, &Surface::Cylinder(cyl.clone()), boundary_3d, holes_3d, forward, pool, params)
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
    triangulate_surface_cdt_generic(face, &Surface::Cone(cone.clone()), boundary_3d, holes_3d, forward, pool, params)
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
    triangulate_surface_cdt_generic(face, &Surface::Sphere(sphere.clone()), boundary_3d, holes_3d, forward, pool, params)
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
    triangulate_surface_cdt_generic(face, &Surface::Torus(torus.clone()), boundary_3d, holes_3d, forward, pool, params)
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
        _ => {
            // For NURBS, revolution, extrusion surfaces: use snap-boundary approach
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
                            if ddx * ddx + ddy * ddy + ddz * ddz < self.tolerance_sq {
                                return idx;
                            }
                        }
                    }
                }
            }
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
    let stitch_tolerances = [
        merge_tolerance * 2.0,
        merge_tolerance * 5.0,
        merge_tolerance * 10.0,
        merge_tolerance * 50.0,
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

    // If still not watertight, try zipper stitching with conservative tolerance
    // Only use zipper stitch when there are few remaining boundary edges
    // (to avoid creating non-manifold edges from over-aggressive stitching)
    {
        let report = crate::watertight::validate_watertight(&mesh, false);
        if !report.is_watertight() && report.boundary_edge_count > 0 && report.boundary_edge_count <= 50 {
            log::info!(
                "Trying conservative zipper stitch for {} remaining boundary edges",
                report.boundary_edge_count,
            );
            zipper_stitch_boundary_edges(&mut mesh, merge_tolerance * 20.0, 3);
            filter_degenerate_triangles(&mut mesh, 1e-10);
        }
    }

    // Final validation
    let report = crate::watertight::validate_watertight(&mesh, false);
    if !report.is_watertight() {
        log::warn!(
            "triangulate_solid_watertight: mesh still has {} boundary edges, {} non-manifold edges after stitching",
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
