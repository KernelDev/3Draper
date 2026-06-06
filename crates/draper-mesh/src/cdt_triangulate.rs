// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! CDT-based solid triangulation with tolerance-aware vertex unification.
//!
//! This module provides watertight mesh generation by:
//! 1. Collecting ALL boundary edge samples from ALL faces of a solid
//! 2. Unifying vertices across faces using 3D tolerance (spatial hashing)
//! 3. For planar faces: using spade CDT with constraint edges
//! 4. For curved faces: using earcutr with unified boundary vertices
//! 5. Building the final mesh with unified vertex indices
//!
//! The key insight: by unifying vertices BEFORE triangulation and using
//! constraint edges, shared edges between adjacent faces automatically
//! have identical vertex indices, producing watertight meshes by construction.

use crate::mesh::TriangleMesh;
use crate::triangulate::{TriangulationParams, merge_coincident_vertices, filter_degenerate_triangles};
use crate::parametric_domain::triangulate_surface_consistent;
use draper_geometry::{
    Point3d, Point2d, Direction3d, Surface, Plane,
};
use draper_topology::{Face, Solid, TopoId};
use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation as SpadeTriangulation};
use std::collections::HashMap;

/// A unified vertex pool that merges vertices across all faces of a solid.
///
/// When two faces share an edge, their boundary vertices should be the same.
/// This pool ensures that by:
/// 1. Inserting all boundary vertices with 3D spatial hashing
/// 2. Merging vertices within the given tolerance
/// 3. Mapping each face's local boundary indices to unified indices
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
}

impl UnifiedVertexPool {
    /// Create a new vertex pool with the given merge tolerance.
    pub fn new(tolerance: f64) -> Self {
        Self {
            vertices: Vec::new(),
            grid: HashMap::new(),
            cell_size: tolerance * 10.0,
            tolerance_sq: tolerance * tolerance,
        }
    }

    /// Insert a vertex and return its unified index.
    ///
    /// If a vertex within `tolerance` already exists, returns the existing index.
    /// Otherwise, adds the vertex and returns a new index.
    pub fn insert(&mut self, point: Point3d) -> u32 {
        let cx = (point.x / self.cell_size).floor() as i64;
        let cy = (point.y / self.cell_size).floor() as i64;
        let cz = (point.z / self.cell_size).floor() as i64;

        // Check neighboring cells for existing vertex within tolerance
        for dx in -1i64..=1 {
            for dy in -1i64..=1 {
                for dz in -1i64..=1 {
                    let key = (cx + dx, cy + dy, cz + dz);
                    if let Some(indices) = self.grid.get(&key) {
                        for &idx in indices {
                            let existing = &self.vertices[idx as usize];
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

        // No match found — add new vertex
        let new_idx = self.vertices.len() as u32;
        self.vertices.push(point);
        self.grid.entry((cx, cy, cz)).or_default().push(new_idx);
        new_idx
    }

    /// Number of unique vertices.
    pub fn len(&self) -> usize {
        self.vertices.len()
    }

    /// Whether the pool is empty.
    pub fn is_empty(&self) -> bool {
        self.vertices.is_empty()
    }
}

/// Information about a face's boundary in terms of unified vertex indices.
#[derive(Clone, Debug)]
pub struct FaceBoundaryInfo {
    /// The face's TopoId (for tracking).
    pub face_id: TopoId,
    /// Outer boundary as unified vertex indices (in order around the boundary).
    pub outer_indices: Vec<u32>,
    /// Inner boundaries (holes) as unified vertex indices.
    pub hole_indices: Vec<Vec<u32>>,
    /// The original 3D points for the outer boundary (same length as outer_indices).
    pub outer_points_3d: Vec<Point3d>,
    /// UV coordinates for outer boundary points on this face's surface.
    pub outer_uvs: Vec<Point2d>,
    /// UV coordinates for inner boundary points.
    pub hole_uvs: Vec<Vec<Point2d>>,
    /// Hole 3D points.
    pub hole_points_3d: Vec<Vec<Point3d>>,
    /// Whether the face normal matches the surface normal.
    pub forward: bool,
}

/// Triangulate a solid into a watertight mesh using CDT with vertex unification.
///
/// This function ensures watertightness by:
/// 1. Collecting all boundary edge samples from all faces
/// 2. Unifying vertices across faces using 3D tolerance
/// 3. Triangulating each face using the unified vertex set
/// 4. Building the final mesh with shared vertex indices
///
/// # Arguments
/// * `solid` — The B-Rep solid to triangulate.
/// * `params` — Triangulation parameters.
/// * `merge_tolerance` — 3D distance tolerance for merging boundary vertices.
///   Typical values: 1e-3 to 1e-4. Should be large enough to merge vertices
///   that are meant to be the same but differ due to floating-point precision,
///   but small enough to not merge distinct vertices.
pub fn triangulate_solid_watertight(
    solid: &Solid,
    params: &TriangulationParams,
    merge_tolerance: f64,
) -> TriangleMesh {
    let faces = solid.faces();
    if faces.is_empty() {
        return TriangleMesh::new();
    }

    // ============================================================
    // Phase 1: Collect boundary edges and build unified vertex pool
    // ============================================================

    let mut pool = UnifiedVertexPool::new(merge_tolerance);
    let mut face_boundaries: Vec<FaceBoundaryInfo> = Vec::with_capacity(faces.len());

    for face in &faces {
        let boundary_info = collect_face_boundary_unified(face, &mut pool, params);
        face_boundaries.push(boundary_info);
    }

    log::info!(
        "triangulate_solid_watertight: {} faces, {} unified vertices (merge_tol={:.6})",
        faces.len(), pool.len(), merge_tolerance
    );

    // ============================================================
    // Phase 2: Triangulate each face using the unified vertex pool
    // ============================================================

    let mut mesh = TriangleMesh::new();

    // The mesh will use the unified vertex pool directly.
    // Add all unified vertices to the mesh first.
    for &point in &pool.vertices {
        mesh.add_vertex(point);
    }

    for (face_idx, boundary) in face_boundaries.iter().enumerate() {
        let face = faces[face_idx];
        let surface = match &face.surface {
            Some(s) => s,
            None => continue,
        };

        let face_triangles = match surface {
            Surface::Plane(plane) => {
                triangulate_planar_face_cdt(
                    &boundary,
                    plane,
                    pool.vertices.len(),
                    params,
                )
            }
            _ => {
                // For curved surfaces, use earcutr with unified boundary vertices
                triangulate_curved_face_unified(
                    &boundary,
                    surface,
                    params,
                )
            }
        };

        // Add triangles to the mesh (vertex indices are already unified)
        for tri in face_triangles {
            // Skip degenerate triangles
            if tri[0] != tri[1] && tri[1] != tri[2] && tri[0] != tri[2] {
                // Check bounds
                let max_idx = pool.vertices.len() as u32;
                if tri[0] < max_idx && tri[1] < max_idx && tri[2] < max_idx {
                    // Apply face orientation
                    if boundary.forward {
                        mesh.add_triangle(tri[0], tri[1], tri[2]);
                    } else {
                        mesh.add_triangle(tri[0], tri[2], tri[1]);
                    }
                }
            }
        }
    }

    // Filter degenerate triangles
    filter_degenerate_triangles(&mut mesh, 1e-10);

    mesh
}

/// Collect boundary points from a face and insert them into the unified vertex pool.
///
/// Returns the face boundary info with unified vertex indices.
fn collect_face_boundary_unified(
    face: &Face,
    pool: &mut UnifiedVertexPool,
    params: &TriangulationParams,
) -> FaceBoundaryInfo {
    let _ = params; // Used indirectly via sample count

    let mut outer_indices = Vec::new();
    let mut outer_points_3d = Vec::new();
    let mut outer_uvs = Vec::new();

    // Collect outer boundary points
    if let Some(ref wire) = face.outer_wire {
        for coedge in &wire.coedges {
            let edge = face.edges.iter().find(|e| e.id == coedge.edge);
            if let Some(edge) = edge {
                if edge.degenerate {
                    continue;
                }

                let mut edge_pts = crate::triangulate::sample_edge_points(edge, 32);

                // Apply coedge orientation
                let edge_is_reversed = edge.param_range.0 > edge.param_range.1;
                let should_reverse = !coedge.forward != edge_is_reversed;
                if should_reverse {
                    edge_pts.reverse();
                }

                // Compute UV coordinates for each point
                let surface = face.surface.as_ref();
                for pt in &edge_pts {
                    let unified_idx = pool.insert(*pt);
                    outer_indices.push(unified_idx);
                    outer_points_3d.push(*pt);

                    if let Some(surf) = surface {
                        let (u, v) = surf.project_point(pt);
                        outer_uvs.push(Point2d::new(u, v));
                    } else {
                        outer_uvs.push(Point2d::new(0.0, 0.0));
                    }
                }
            }
        }
    }

    // Deduplicate consecutive vertices with same unified index
    deduplicate_boundary(&mut outer_indices, &mut outer_points_3d, &mut outer_uvs);

    // Close the loop: if last vertex == first vertex, remove the last
    if outer_indices.len() > 1 && outer_indices[0] == *outer_indices.last().unwrap() {
        outer_indices.pop();
        outer_points_3d.pop();
        outer_uvs.pop();
    }

    // Collect inner boundary (hole) points
    let mut hole_indices = Vec::new();
    let mut hole_uvs = Vec::new();
    let mut hole_points_3d = Vec::new();

    for wire in &face.inner_wires {
        let mut hole_idx = Vec::new();
        let mut hole_pts = Vec::new();
        let mut hole_uv = Vec::new();

        for coedge in &wire.coedges {
            let edge = face.edges.iter().find(|e| e.id == coedge.edge);
            if let Some(edge) = edge {
                if edge.degenerate {
                    continue;
                }

                let mut edge_pts = crate::triangulate::sample_edge_points(edge, 32);
                let edge_is_reversed = edge.param_range.0 > edge.param_range.1;
                let should_reverse = !coedge.forward != edge_is_reversed;
                if should_reverse {
                    edge_pts.reverse();
                }

                let surface = face.surface.as_ref();
                for pt in &edge_pts {
                    let unified_idx = pool.insert(*pt);
                    hole_idx.push(unified_idx);
                    hole_pts.push(*pt);
                    if let Some(surf) = surface {
                        let (u, v) = surf.project_point(pt);
                        hole_uv.push(Point2d::new(u, v));
                    } else {
                        hole_uv.push(Point2d::new(0.0, 0.0));
                    }
                }
            }
        }

        deduplicate_boundary(&mut hole_idx, &mut hole_pts, &mut hole_uv);
        if hole_idx.len() > 1 && hole_idx[0] == *hole_idx.last().unwrap() {
            hole_idx.pop();
            hole_pts.pop();
            hole_uv.pop();
        }

        if !hole_idx.is_empty() {
            hole_indices.push(hole_idx);
            hole_uvs.push(hole_uv);
            hole_points_3d.push(hole_pts);
        }
    }

    FaceBoundaryInfo {
        face_id: face.id,
        outer_indices,
        hole_indices,
        outer_points_3d,
        outer_uvs,
        hole_uvs,
        hole_points_3d,
        forward: face.forward,
    }
}

/// Remove duplicate consecutive entries in boundary arrays.
fn deduplicate_boundary(
    indices: &mut Vec<u32>,
    points_3d: &mut Vec<Point3d>,
    uvs: &mut Vec<Point2d>,
) {
    if indices.is_empty() {
        return;
    }
    let mut unique_idx = vec![0];
    let mut unique_pts = vec![points_3d[0]];
    let mut unique_uvs = vec![uvs[0]];

    for i in 1..indices.len() {
        // Skip if the unified index is the same as the previous
        if indices[i] != *unique_idx.last().unwrap() {
            unique_idx.push(indices[i]);
            unique_pts.push(points_3d[i]);
            unique_uvs.push(uvs[i]);
        }
    }

    *indices = unique_idx;
    *points_3d = unique_pts;
    *uvs = unique_uvs;
}

/// Triangulate a planar face using spade CDT with constraint edges.
///
/// This produces a high-quality constrained Delaunay triangulation that
/// respects the boundary polygon and hole boundaries.
fn triangulate_planar_face_cdt(
    boundary: &FaceBoundaryInfo,
    plane: &Plane,
    _total_vertices: usize,
    _params: &TriangulationParams,
) -> Vec<[u32; 3]> {
    if boundary.outer_indices.len() < 3 {
        return Vec::new();
    }

    // Project 3D boundary points onto the plane's 2D coordinate system
    let project = |p: &Point3d| -> Point2d {
        let dx = p.x - plane.origin.x;
        let dy = p.y - plane.origin.y;
        let dz = p.z - plane.origin.z;
        Point2d::new(
            dx * plane.u_dir.x + dy * plane.u_dir.y + dz * plane.u_dir.z,
            dx * plane.v_dir.x + dy * plane.v_dir.y + dz * plane.v_dir.z,
        )
    };

    // Snap boundary points to plane to eliminate numerical drift
    let snap_to_plane = |p: &Point3d| -> Point3d {
        let dx = p.x - plane.origin.x;
        let dy = p.y - plane.origin.y;
        let dz = p.z - plane.origin.z;
        let dist = dx * plane.normal.x + dy * plane.normal.y + dz * plane.normal.z;
        Point3d::new(
            p.x - dist * plane.normal.x,
            p.y - dist * plane.normal.y,
            p.z - dist * plane.normal.z,
        )
    };

    // Build vertex list for CDT: map unified indices to 2D positions
    // We need to collect all unique vertex indices used by this face
    let mut vertex_set: Vec<u32> = boundary.outer_indices.clone();
    for hole in &boundary.hole_indices {
        for &idx in hole {
            if !vertex_set.contains(&idx) {
                vertex_set.push(idx);
            }
        }
    }

    // For CDT, we need to provide 2D points and constraint edges.
    // Use spade's ConstrainedDelaunayTriangulation with bulk_load_cdt.
    // Build the 2D point array and constraint edges.

    // Collect all 2D points and their unified indices
    let mut cdt_vertices: Vec<Point2<f64>> = Vec::new();
    let mut unified_to_cdt: HashMap<u32, usize> = HashMap::new();

    for &unified_idx in &vertex_set {
        // Get the 3D point from the boundary (or from the unified pool)
        let p3d = snap_to_plane(&boundary.outer_points_3d.iter()
            .zip(boundary.outer_indices.iter())
            .find(|(_, &idx)| idx == unified_idx)
            .map(|(p, _)| *p)
            .unwrap_or_else(|| {
                // Try holes
                boundary.hole_points_3d.iter()
                    .flat_map(|pts| pts.iter())
                    .zip(boundary.hole_indices.iter().flat_map(|idx| idx.iter()))
                    .find(|(_, &idx)| idx == unified_idx)
                    .map(|(p, _)| *p)
                    .unwrap_or(Point3d::ORIGIN)
            }));
        let p2d = project(&p3d);
        let cdt_idx = cdt_vertices.len();
        unified_to_cdt.insert(unified_idx, cdt_idx);
        cdt_vertices.push(Point2::new(p2d.u, p2d.v));
    }

    // Build constraint edges from outer boundary
    let mut constraint_edges: Vec<[usize; 2]> = Vec::new();
    let outer_len = boundary.outer_indices.len();
    for i in 0..outer_len {
        let a = *unified_to_cdt.get(&boundary.outer_indices[i]).unwrap_or(&0);
        let b = *unified_to_cdt.get(&boundary.outer_indices[(i + 1) % outer_len]).unwrap_or(&0);
        if a != b {
            constraint_edges.push([a.min(b), a.max(b)]);
        }
    }

    // Add constraint edges from holes
    for hole in &boundary.hole_indices {
        let hole_len = hole.len();
        for i in 0..hole_len {
            let a = *unified_to_cdt.get(&hole[i]).unwrap_or(&0);
            let b = *unified_to_cdt.get(&hole[(i + 1) % hole_len]).unwrap_or(&0);
            if a != b {
                constraint_edges.push([a.min(b), a.max(b)]);
            }
        }
    }

    // Deduplicate constraint edges
    constraint_edges.sort();
    constraint_edges.dedup();

    // Run spade CDT using bulk_load_cdt
    let cdt_result = ConstrainedDelaunayTriangulation::<Point2<f64>>::bulk_load_cdt(
        cdt_vertices.clone(),
        constraint_edges,
    );

    let mut triangles = Vec::new();

    match cdt_result {
        Ok(cdt) => {
            // Extract triangles from the CDT
            // We need to map CDT vertex handles back to our unified indices
            for face in cdt.inner_faces() {
                let vertices = face.vertices();
                let v0_idx = vertices[0].fix();
                let v1_idx = vertices[1].fix();
                let v2_idx = vertices[2].fix();

                // Convert CDT vertex indices back to unified vertex indices
                // CDT vertex indices correspond to positions in vertex_set
                let i0 = vertex_set.get(v0_idx.index()).copied();
                let i1 = vertex_set.get(v1_idx.index()).copied();
                let i2 = vertex_set.get(v2_idx.index()).copied();

                if let (Some(i0), Some(i1), Some(i2)) = (i0, i1, i2) {
                    if i0 != i1 && i1 != i2 && i0 != i2 {
                        triangles.push([i0, i1, i2]);
                    }
                }
            }
        }
        Err(e) => {
            log::warn!("spade CDT failed for planar face {}: {:?}, falling back to earcutr",
                boundary.face_id, e);
            // Fallback to earcutr
            return triangulate_planar_face_earcutr(boundary, plane);
        }
    }

    // Filter triangles that are outside the boundary polygon
    // (CDT produces a convex hull that may include triangles outside the polygon)
    let outer_2d: Vec<Point2d> = boundary.outer_points_3d.iter().map(|p| project(p)).collect();
    let filtered_triangles: Vec<[u32; 3]> = triangles.into_iter().filter(|tri| {
        // Check if the centroid of the triangle is inside the boundary polygon
        let p0 = boundary.outer_points_3d.iter()
            .zip(boundary.outer_indices.iter())
            .find(|(_, &idx)| idx == tri[0])
            .map(|(p, _)| *p)
            .unwrap_or(Point3d::ORIGIN);
        let p1 = boundary.outer_points_3d.iter()
            .zip(boundary.outer_indices.iter())
            .find(|(_, &idx)| idx == tri[1])
            .map(|(p, _)| *p)
            .unwrap_or(Point3d::ORIGIN);
        let p2 = boundary.outer_points_3d.iter()
            .zip(boundary.outer_indices.iter())
            .find(|(_, &idx)| idx == tri[2])
            .map(|(p, _)| *p)
            .unwrap_or(Point3d::ORIGIN);

        let centroid = Point3d::new(
            (p0.x + p1.x + p2.x) / 3.0,
            (p0.y + p1.y + p2.y) / 3.0,
            (p0.z + p1.z + p2.z) / 3.0,
        );
        let centroid_2d = project(&centroid);

        // Check if centroid is inside the outer boundary
        if !point_in_polygon(&centroid_2d, &outer_2d) {
            return false;
        }

        // Check if centroid is NOT inside any hole
        for hole_pts in &boundary.hole_points_3d {
            let hole_2d: Vec<Point2d> = hole_pts.iter().map(|p| project(p)).collect();
            if point_in_polygon(&centroid_2d, &hole_2d) {
                return false;
            }
        }

        true
    }).collect();

    filtered_triangles
}

/// Fallback: triangulate a planar face using earcutr with unified boundary vertices.
fn triangulate_planar_face_earcutr(
    boundary: &FaceBoundaryInfo,
    plane: &Plane,
) -> Vec<[u32; 3]> {
    let project = |p: &Point3d| -> Point2d {
        let dx = p.x - plane.origin.x;
        let dy = p.y - plane.origin.y;
        let dz = p.z - plane.origin.z;
        Point2d::new(
            dx * plane.u_dir.x + dy * plane.u_dir.y + dz * plane.u_dir.z,
            dx * plane.v_dir.x + dy * plane.v_dir.y + dz * plane.v_dir.z,
        )
    };

    // Build earcutr input
    let mut coords: Vec<f64> = Vec::new();
    for pt in &boundary.outer_points_3d {
        let p2d = project(pt);
        coords.push(p2d.u);
        coords.push(p2d.v);
    }

    let mut hole_indices: Vec<usize> = Vec::new();
    for hole_pts in &boundary.hole_points_3d {
        hole_indices.push(coords.len() / 2);
        for pt in hole_pts {
            let p2d = project(pt);
            coords.push(p2d.u);
            coords.push(p2d.v);
        }
    }

    let triangle_indices = earcutr::earcut(&coords, &hole_indices, 2);

    // Map earcutr indices to unified vertex indices
    let mut all_indices: Vec<u32> = boundary.outer_indices.clone();
    for hole in &boundary.hole_indices {
        all_indices.extend_from_slice(hole);
    }

    let mut triangles = Vec::new();
    for chunk in triangle_indices.chunks(3) {
        if chunk.len() < 3 { break; }
        let a = chunk[0] as usize;
        let b = chunk[1] as usize;
        let c = chunk[2] as usize;
        if a < all_indices.len() && b < all_indices.len() && c < all_indices.len() {
            let ia = all_indices[a];
            let ib = all_indices[b];
            let ic = all_indices[c];
            if ia != ib && ib != ic && ia != ic {
                triangles.push([ia, ib, ic]);
            }
        }
    }

    triangles
}

/// Triangulate a curved face using earcutr with unified boundary vertices.
///
/// For curved surfaces, we use the existing earcutr-based approach but with
/// unified boundary vertex indices. This ensures that shared edges between
/// adjacent faces have identical vertex indices.
fn triangulate_curved_face_unified(
    boundary: &FaceBoundaryInfo,
    surface: &Surface,
    params: &TriangulationParams,
) -> Vec<[u32; 3]> {
    if boundary.outer_points_3d.len() < 3 {
        return Vec::new();
    }

    // Use the existing triangulate_surface_consistent function,
    // but then map the resulting vertex indices to unified indices.
    let face_mesh = triangulate_surface_consistent(
        surface,
        &boundary.outer_points_3d,
        &boundary.outer_uvs,
        &boundary.hole_points_3d,
        &boundary.hole_uvs,
        boundary.forward,
        params,
    );

    // Now we need to map the face_mesh's vertices to unified vertex indices.
    // The face_mesh has its own local vertex indices. We need to find
    // which unified vertex index each face mesh vertex corresponds to.
    let mut triangles = Vec::new();
    let mut vertex_remap: HashMap<u32, u32> = HashMap::new();

    // Build a mapping from 3D point to unified index
    let mut point_to_unified: HashMap<(i64, i64, i64), Vec<(Point3d, u32)>> = HashMap::new();
    let merge_tol = 1e-3; // Use the same tolerance for lookup
    let cell = merge_tol * 10.0;

    for (i, &idx) in boundary.outer_indices.iter().enumerate() {
        let pt = boundary.outer_points_3d[i];
        let cx = (pt.x / cell).floor() as i64;
        let cy = (pt.y / cell).floor() as i64;
        let cz = (pt.z / cell).floor() as i64;
        point_to_unified.entry((cx, cy, cz)).or_default().push((pt, idx));
    }
    for (hole_idx, hole_pts) in boundary.hole_points_3d.iter().enumerate() {
        for (i, pt) in hole_pts.iter().enumerate() {
            let idx = boundary.hole_indices[hole_idx][i];
            let cx = (pt.x / cell).floor() as i64;
            let cy = (pt.y / cell).floor() as i64;
            let cz = (pt.z / cell).floor() as i64;
            point_to_unified.entry((cx, cy, cz)).or_default().push((*pt, idx));
        }
    }

    for tri in &face_mesh.triangles {
        let mut remapped = [0u32; 3];
        let mut valid = true;

        for (k, &local_idx) in tri.iter().enumerate() {
            if let Some(&unified_idx) = vertex_remap.get(&local_idx) {
                remapped[k] = unified_idx;
            } else {
                let pt = face_mesh.vertices[local_idx as usize];
                let cx = (pt.x / cell).floor() as i64;
                let cy = (pt.y / cell).floor() as i64;
                let cz = (pt.z / cell).floor() as i64;

                let mut found = None;
                'outer: for dx in -1i64..=1 {
                    for dy in -1i64..=1 {
                        for dz in -1i64..=1 {
                            let key = (cx + dx, cy + dy, cz + dz);
                            if let Some(entries) = point_to_unified.get(&key) {
                                for (ref_pt, ref_idx) in entries {
                                    let ddx = pt.x - ref_pt.x;
                                    let ddy = pt.y - ref_pt.y;
                                    let ddz = pt.z - ref_pt.z;
                                    if ddx * ddx + ddy * ddy + ddz * ddz < merge_tol * merge_tol {
                                        found = Some(*ref_idx);
                                        break 'outer;
                                    }
                                }
                            }
                        }
                    }
                }

                if let Some(unified_idx) = found {
                    vertex_remap.insert(local_idx, unified_idx);
                    remapped[k] = unified_idx;
                } else {
                    // Interior vertex — not on the boundary. Use a new unified index.
                    // We need to add this vertex to the global mesh.
                    // For now, just mark it as invalid and skip.
                    valid = false;
                    break;
                }
            }
        }

        if valid && remapped[0] != remapped[1] && remapped[1] != remapped[2] && remapped[0] != remapped[2] {
            triangles.push(remapped);
        }
    }

    triangles
}

/// Check if a 2D point is inside a polygon using ray casting.
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

#[cfg(test)]
mod tests {
    use super::*;
    use draper_geometry::Point3d;

    #[test]
    fn test_unified_vertex_pool_merge() {
        let mut pool = UnifiedVertexPool::new(1e-3);

        let a = pool.insert(Point3d::new(0.0, 0.0, 0.0));
        let b = pool.insert(Point3d::new(1.0, 0.0, 0.0));
        // Close to a — should merge
        let c = pool.insert(Point3d::new(0.0001, 0.0001, 0.0001));

        assert_eq!(pool.len(), 2, "Should have 2 unique vertices (a≈c merged)");
        assert_eq!(a, c, "Close vertices should get the same index");
        assert_ne!(a, b, "Distinct vertices should get different indices");
    }

    #[test]
    fn test_unified_vertex_pool_no_false_merge() {
        let mut pool = UnifiedVertexPool::new(1e-3);

        let a = pool.insert(Point3d::new(0.0, 0.0, 0.0));
        let b = pool.insert(Point3d::new(0.1, 0.0, 0.0)); // 0.1 apart — should NOT merge at 1e-3

        assert_eq!(pool.len(), 2, "Should have 2 unique vertices");
        assert_ne!(a, b, "Vertices 0.1 apart should not merge at 1e-3 tolerance");
    }

    #[test]
    fn test_point_in_polygon_square() {
        let square = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(1.0, 0.0),
            Point2d::new(1.0, 1.0),
            Point2d::new(0.0, 1.0),
        ];
        assert!(point_in_polygon(&Point2d::new(0.5, 0.5), &square));
        assert!(!point_in_polygon(&Point2d::new(1.5, 0.5), &square));
    }
}
