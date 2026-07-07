// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Watertight mesh validation for B-Rep solid triangulation.
//!
//! After triangulating all faces of a solid and merging them into a single mesh,
//! this module checks that the result is **watertight**: every edge must be shared
//! by exactly 2 triangles. If any edge has count != 2, the mesh has gaps (count=1)
//! or self-intersections (count>2).
//!
//! # Usage
//! ```rust,ignore
//! use draper_mesh::watertight::validate_watertight;
//!
//! let report = validate_watertight(&merged_mesh, true);
//! if report.is_watertight() {
//!     println!("Mesh is watertight!");
//! } else {
//!     println!("Mesh has {} boundary edges, {} non-manifold edges",
//!              report.boundary_edge_count, report.non_manifold_edge_count);
//! }
//! ```

use crate::mesh::TriangleMesh;
use crate::edge_cache::compute_adaptive_crease_angle;
use draper_geometry::{Point3d};
use draper_topology::Solid;
use std::collections::HashMap;

/// Result of watertight validation on a merged solid mesh.
#[derive(Clone, Debug)]
pub struct WatertightReport {
    /// Total number of unique edges in the mesh.
    pub edge_count: usize,
    /// Number of edges shared by exactly 2 triangles (good — interior edges).
    pub interior_edge_count: usize,
    /// Number of edges with only 1 adjacent triangle (boundary/gap edges).
    pub boundary_edge_count: usize,
    /// Number of edges with more than 2 adjacent triangles (non-manifold).
    pub non_manifold_edge_count: usize,
    /// Total number of triangles.
    pub triangle_count: usize,
    /// Total number of vertices.
    pub vertex_count: usize,
    /// Euler characteristic: V - E + F.
    pub euler_characteristic: i64,
    /// Number of degenerate triangles (zero area).
    pub degenerate_triangle_count: usize,
    /// Number of duplicate triangles (identical vertex sets).
    pub duplicate_triangle_count: usize,
    /// Boundary edges as vertex pairs (for debugging).
    pub boundary_edges: Vec<(u32, u32)>,
    /// Non-manifold edges as (vertex_a, vertex_b, face_count).
    pub non_manifold_edges: Vec<(u32, u32, u32)>,
    /// Per-face-id watertight summary (if face_ids are available).
    pub per_face_summary: HashMap<u64, FaceWatertightSummary>,
}

/// Watertight summary for a single face's triangles.
#[derive(Clone, Debug, Default)]
pub struct FaceWatertightSummary {
    /// Number of triangles in this face.
    pub triangle_count: usize,
    /// Number of boundary edges (edges only in this face, not shared with another face).
    pub boundary_edge_count: usize,
}

impl WatertightReport {
    /// Check if the mesh is watertight: no boundary edges, no non-manifold edges.
    pub fn is_watertight(&self) -> bool {
        self.boundary_edge_count == 0 && self.non_manifold_edge_count == 0
    }

    /// Check if the mesh is a 2-manifold: no non-manifold edges (boundary is OK).
    pub fn is_manifold(&self) -> bool {
        self.non_manifold_edge_count == 0
    }
}

/// Result of edge consistency validation.
///
/// After topology-first triangulation, shared edges between faces should have
/// **bit-identical** vertex positions. If they don't, the edge cache is not
/// working correctly and the mesh will have gaps/cracks.
#[derive(Clone, Debug)]
pub struct EdgeConsistencyReport {
    /// Total number of interior edges (shared by 2+ faces) examined.
    pub shared_edges_checked: usize,
    /// Number of shared edges where all incident vertices are bit-identical.
    pub consistent_edges: usize,
    /// Number of shared edges where incident vertices differ (BUG in edge cache!).
    pub inconsistent_edges: usize,
    /// Maximum distance between corresponding vertices on shared edges.
    /// Should be 0.0 if the edge cache works correctly.
    pub max_vertex_distance: f64,
    /// Details of the worst inconsistencies (limited to 10 for log readability).
    pub worst_inconsistencies: Vec<EdgeInconsistency>,
}

/// Details of an inconsistent shared edge.
#[derive(Clone, Debug)]
pub struct EdgeInconsistency {
    /// Vertex indices that should be the same but aren't.
    pub vertex_indices: (u32, u32),
    /// Distance between the two vertices.
    pub distance: f64,
    /// Face IDs of the triangles sharing this edge (if available).
    pub face_ids: Vec<u64>,
}

impl EdgeConsistencyReport {
    /// Check if all shared edges are consistent (bit-identical vertices).
    pub fn is_consistent(&self) -> bool {
        self.inconsistent_edges == 0
    }

    /// Percentage of shared edges that are inconsistent.
    pub fn inconsistency_rate(&self) -> f64 {
        if self.shared_edges_checked == 0 {
            0.0
        } else {
            self.inconsistent_edges as f64 / self.shared_edges_checked as f64 * 100.0
        }
    }
}

/// Validate that shared edges in the mesh have bit-identical vertex positions.
///
/// This is the key diagnostic for the topology-first approach: if the unified
/// edge cache works correctly, two faces sharing an edge should produce the
/// exact same vertex indices in the merged mesh. If they produce different
/// vertex indices, the corresponding Point3d values should at least be
/// bit-identical (distance = 0).
///
/// # Algorithm
/// 1. Build the edge→face-count map (same as `validate_watertight`)
/// 2. For each interior edge (shared by 2 triangles), check that the
///    two vertex indices are the SAME. If not, compute the distance
///    between the two Point3d values.
/// 3. Report the number of inconsistent edges and the maximum distance.
///
/// # Arguments
/// * `mesh` — The triangle mesh to validate.
/// * `tolerance` — Distance threshold for flagging inconsistencies.
///   Use 0.0 for strict bit-identity, or a small value (e.g., 1e-12) for
///   floating-point tolerance.
pub fn validate_edge_consistency(mesh: &TriangleMesh, tolerance: f64) -> EdgeConsistencyReport {
    let mut report = EdgeConsistencyReport {
        shared_edges_checked: 0,
        consistent_edges: 0,
        inconsistent_edges: 0,
        max_vertex_distance: 0.0,
        worst_inconsistencies: Vec::new(),
    };

    if mesh.vertices.is_empty() || mesh.triangles.is_empty() {
        return report;
    }

    // Build edge → list of (triangle_index, vertex_a, vertex_b) map.
    // Key: canonical sorted pair (lo, hi).
    // Value: original unsorted pair (a, b) from the triangle — needed to
    // detect when two different vertex indices map to the same geometric edge.
    //
    // When merge_deduplicating works correctly, two faces sharing an edge
    // produce the SAME vertex indices (e.g., both have edge (5,8)). When it
    // fails, they produce DIFFERENT indices that happen to have the same
    // 3D positions (e.g., one has (5,8) and another has (12,15)), but
    // those different indices will NOT share an edge_map key because
    // (5,8) ≠ (12,15). They'll appear as two separate boundary edges instead.
    //
    // So actually, if two triangles share an edge_map key (lo, hi), they
    // necessarily use the SAME vertex indices. The "inconsistency" scenario
    // (different indices, same positions) manifests as boundary edges, not
    // as interior edges with mismatched indices.
    //
    // This validation checks interior edges for a subtler issue: when a
    // single face's triangulation produces an edge where two vertices that
    // SHOULD be the same point have different indices but close 3D positions.
    // This can happen when the edge cache returns slightly different points
    // for the same edge at different parametric locations.
    let mut edge_map: HashMap<(u32, u32), Vec<(usize, u32, u32)>> = HashMap::new();
    for (ti, tri) in mesh.triangles.iter().enumerate() {
        // Skip degenerate triangles — they contribute phantom edges
        if tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2] {
            continue;
        }
        let edges = [
            (tri[0].min(tri[1]), tri[0].max(tri[1]), tri[0], tri[1]),
            (tri[1].min(tri[2]), tri[1].max(tri[2]), tri[1], tri[2]),
            (tri[2].min(tri[0]), tri[2].max(tri[0]), tri[2], tri[0]),
        ];
        for (lo, hi, a, b) in &edges {
            edge_map.entry((*lo, *hi)).or_default().push((ti, *a, *b));
        }
    }

    // For interior edges (count >= 2), check vertex consistency
    let _tol_sq = tolerance * tolerance;
    let face_ids = mesh.triangle_face_ids.as_deref();

    for ((_lo, _hi), entries) in &edge_map {
        if entries.len() < 2 {
            continue; // Boundary edge — skip
        }

        report.shared_edges_checked += 1;

        // Since the edge_map key is the canonical (lo, hi), all entries
        // sharing this key use the same two vertex indices (by definition:
        // min(a,b)=lo and max(a,b)=hi). Therefore, if dedup worked, all
        // entries have identical (lo, hi) and the edge is consistent.
        //
        // The only way to get genuinely different indices for the same
        // geometric edge is if the vertices have different index values but
        // happen to have the same 3D positions. But then they'd produce
        // different (lo, hi) keys and wouldn't be grouped together.
        //
        // So for interior edges grouped by canonical key, they are ALWAYS
        // consistent by construction. The real diagnostic is the boundary
        // edge count from validate_watertight().
        report.consistent_edges += 1;
    }

    // The real edge cache diagnostic: count boundary edges that should be
    // interior (shared between faces). This is done by validate_watertight(),
    // not here. But we also check for "near-miss" edges: boundary edges
    // from different faces whose endpoints are close in 3D space but
    // weren't merged by dedup (indicating the edge cache produced slightly
    // different positions for the same logical edge).
    let boundary_edges: Vec<((u32, u32), Vec<(usize, u32, u32)>)> = edge_map
        .iter()
        .filter(|(_, entries)| entries.len() == 1)
        .map(|(k, v)| (*k, v.clone()))
        .collect();

    if !boundary_edges.is_empty() {
        // Build spatial index of boundary vertex positions for near-miss detection
        let mut vertex_positions: HashMap<u32, Point3d> = HashMap::new();
        for &(lo, hi) in boundary_edges.iter().map(|(k, _)| k) {
            if !vertex_positions.contains_key(&lo) {
                vertex_positions.insert(lo, mesh.vertices[lo as usize]);
            }
            if !vertex_positions.contains_key(&hi) {
                vertex_positions.insert(hi, mesh.vertices[hi as usize]);
            }
        }

        // For each boundary edge, check if there's another boundary edge
        // from a DIFFERENT face with close but not identical endpoints.
        // This indicates the edge cache produced slightly different positions.
        let near_miss_tol = tolerance.max(1e-6);
        let _near_miss_tol_sq = near_miss_tol * near_miss_tol;

        for (i, (edge_i, entries_i)) in boundary_edges.iter().enumerate() {
            let face_id_i = face_ids
                .and_then(|ids| ids.get(entries_i[0].0).copied())
                .unwrap_or(0);
            let p_lo_i = vertex_positions[&edge_i.0];
            let p_hi_i = vertex_positions[&edge_i.1];

            for (j, (edge_j, entries_j)) in boundary_edges.iter().enumerate() {
                if j <= i { continue; } // Avoid duplicate checks

                let face_id_j = face_ids
                    .and_then(|ids| ids.get(entries_j[0].0).copied())
                    .unwrap_or(0);

                // Skip edges from the same face (they're just face boundaries)
                if face_id_i == face_id_j && face_id_i != 0 { continue; }

                let p_lo_j = vertex_positions[&edge_j.0];
                let p_hi_j = vertex_positions[&edge_j.1];

                // Check both alignment options (lo↔lo,hi↔hi or lo↔hi,hi↔lo)
                let d_ll = dist_sq(&p_lo_i, &p_lo_j);
                let d_hh = dist_sq(&p_hi_i, &p_hi_j);
                let d_lh = dist_sq(&p_lo_i, &p_hi_j);
                let d_hl = dist_sq(&p_hi_i, &p_lo_j);

                let aligned_dist = (d_ll + d_hh).sqrt();
                let flipped_dist = (d_lh + d_hl).sqrt();
                let best_dist = aligned_dist.min(flipped_dist);

                // Only count as near-miss if vertices are close but NOT identical
                // (identical would mean they should have been merged by dedup)
                let is_close = best_dist < near_miss_tol * 100.0;
                let is_not_identical = best_dist > 0.0;

                if is_close && is_not_identical {
                    report.inconsistent_edges += 1;
                    report.max_vertex_distance = report.max_vertex_distance.max(best_dist);

                    if report.worst_inconsistencies.len() < 10 {
                        report.worst_inconsistencies.push(EdgeInconsistency {
                            vertex_indices: (edge_i.0, edge_j.0),
                            distance: best_dist,
                            face_ids: vec![face_id_i, face_id_j],
                        });
                    }
                }
            }
        }

        // Adjust shared_edges_checked to include near-miss pairs
        report.shared_edges_checked += report.inconsistent_edges;

        // Vertex-level near-miss diagnostic: find boundary vertices from different faces
        // that are close in 3D space but not bit-identical (would indicate dedup failure)
        let mut vertex_near_misses = 0usize;
        let mut vertex_near_miss_max_dist = 0.0f64;
        let vertex_nm_tol = tolerance.max(1e-6);
        let vertex_nm_tol_sq = vertex_nm_tol * vertex_nm_tol;

        // Collect unique boundary vertex (index, position, face_id) tuples
        let mut boundary_vertex_info: Vec<(u32, Point3d, u64)> = Vec::new();
        for ((lo, hi), entries) in &edge_map {
            if entries.len() == 1 {
                let face_id = face_ids.and_then(|ids| ids.get(entries[0].0).copied()).unwrap_or(0);
                if !boundary_vertex_info.iter().any(|(idx, _, _)| *idx == *lo) {
                    boundary_vertex_info.push((*lo, mesh.vertices[*lo as usize], face_id));
                }
                if !boundary_vertex_info.iter().any(|(idx, _, _)| *idx == *hi) {
                    boundary_vertex_info.push((*hi, mesh.vertices[*hi as usize], face_id));
                }
            }
        }

        // Check pairs of boundary vertices from different faces
        for i in 0..boundary_vertex_info.len() {
            for j in (i+1)..boundary_vertex_info.len() {
                let (_vi, pi, fi) = &boundary_vertex_info[i];
                let (_vj, pj, fj) = &boundary_vertex_info[j];
                if fi == fj && *fi != 0 { continue; } // Same face
                let dx = pi.x - pj.x;
                let dy = pi.y - pj.y;
                let dz = pi.z - pj.z;
                let dist_sq = dx*dx + dy*dy + dz*dz;
                if dist_sq > 0.0 && dist_sq < vertex_nm_tol_sq * 10000.0 {
                    vertex_near_misses += 1;
                    let dist = dist_sq.sqrt();
                    vertex_near_miss_max_dist = vertex_near_miss_max_dist.max(dist);
                }
            }
        }
        if vertex_near_misses > 0 {
            log::warn!("Vertex near-miss diagnostic: {} boundary vertex pairs from different faces are close but not bit-identical (max_dist={:.2e}, tol={:.2e})",
                vertex_near_misses, vertex_near_miss_max_dist, vertex_nm_tol);
        }
    }

    // Sort by distance (worst first) and keep top 10
    report.worst_inconsistencies.sort_by(|a, b| {
        b.distance.partial_cmp(&a.distance).unwrap_or(std::cmp::Ordering::Equal)
    });
    report.worst_inconsistencies.truncate(10);

    report
}

/// Validate that a merged solid mesh is watertight.
///
/// For a closed solid, every edge should be shared by exactly 2 triangles.
/// This function counts the number of adjacent triangles for each edge
/// and classifies edges as:
/// - Interior (count == 2): correct for a closed solid
/// - Boundary (count == 1): gap/crack — mesh is not watertight
/// - Non-manifold (count > 2): self-intersection or T-junction
///
/// # Arguments
/// * `mesh` — The merged triangle mesh from all faces of the solid.
/// * `verbose` — If true, collect per-face summaries and boundary edge lists
///   (slightly slower due to extra bookkeeping).
pub fn validate_watertight(mesh: &TriangleMesh, verbose: bool) -> WatertightReport {
    let vertex_count = mesh.vertices.len();
    let triangle_count = mesh.triangles.len();

    // Build edge → (face_count, list of face_ids) map
    // Key: canonical edge (smaller vertex index first)
    let mut edge_face_count: HashMap<(u32, u32), EdgeInfo> = HashMap::new();

    for (tri_idx, tri) in mesh.triangles.iter().enumerate() {
        let v0 = tri[0];
        let v1 = tri[1];
        let v2 = tri[2];

        // Skip degenerate triangles — they contribute phantom edges
        // (e.g., self-loop (a,a) and double-counted (a,c)) that inflate
        // boundary/non-manifold edge counts and break watertight validation.
        if v0 == v1 || v1 == v2 || v0 == v2 {
            continue;
        }

        // Also skip POSITION-degenerate triangles (different vertex indices
        // but same 3D position). These occur when merge_deduplicating maps
        // two distinct face-mesh vertices to the same BREP-mesh vertex
        // (because they have the same 3D position). The resulting triangle
        // has zero area and its edges are phantom (e.g., an edge between
        // two coincident points) — they shouldn't be counted.
        let p0 = mesh.vertices[v0 as usize];
        let p1 = mesh.vertices[v1 as usize];
        let p2 = mesh.vertices[v2 as usize];
        let area = ((p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x)).abs()
                 + ((p1.y - p0.y) * (p2.z - p0.z) - (p1.z - p0.z) * (p2.y - p0.y)).abs()
                 + ((p1.z - p0.z) * (p2.x - p0.x) - (p1.x - p0.x) * (p2.z - p0.z)).abs();
        if area < 1e-20 {
            continue;
        }

        let edges = [
            (v0.min(v1), v0.max(v1)),
            (v1.min(v2), v1.max(v2)),
            (v2.min(v0), v2.max(v0)),
        ];

        let face_id = mesh.triangle_face_ids.as_ref()
            .and_then(|ids| ids.get(tri_idx).copied())
            .unwrap_or(0);

        for edge in &edges {
            let info = edge_face_count.entry(*edge).or_insert(EdgeInfo::default());
            info.count += 1;
            if verbose && face_id != 0 {
                info.face_ids.push(face_id);
            }
        }
    }

    // Count degenerate triangles
    let mut degenerate_triangle_count = 0;
    for tri in &mesh.triangles {
        if tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2] {
            degenerate_triangle_count += 1;
            continue;
        }
        let v0 = mesh.vertices[tri[0] as usize];
        let v1 = mesh.vertices[tri[1] as usize];
        let v2 = mesh.vertices[tri[2] as usize];
        let area = triangle_area_3d(&v0, &v1, &v2);
        if area < 1e-20 {
            degenerate_triangle_count += 1;
        }
    }

    // Count duplicate triangles
    let mut duplicate_triangle_count = 0;
    {
        let mut tri_set: HashMap<[u32; 3], usize> = HashMap::new();
        for tri in &mesh.triangles {
            let mut sorted = [tri[0], tri[1], tri[2]];
            sorted.sort();
            *tri_set.entry(sorted).or_insert(0) += 1;
        }
        for &count in tri_set.values() {
            if count > 1 {
                duplicate_triangle_count += count - 1;
            }
        }
    }

    // Classify edges
    let edge_count = edge_face_count.len();
    let mut interior_edge_count = 0;
    let mut boundary_edge_count = 0;
    let mut non_manifold_edge_count = 0;
    let mut boundary_edges = Vec::new();
    let mut non_manifold_edges = Vec::new();

    for (edge, info) in &edge_face_count {
        match info.count {
            1 => {
                boundary_edge_count += 1;
                if verbose {
                    boundary_edges.push(*edge);
                }
            }
            2 => {
                interior_edge_count += 1;
            }
            _ => {
                non_manifold_edge_count += 1;
                if verbose {
                    non_manifold_edges.push((edge.0, edge.1, info.count));
                }
            }
        }
    }

    // Euler characteristic: V - E + F
    let euler = vertex_count as i64 - edge_count as i64 + triangle_count as i64;

    // Per-face summary (if face_ids available)
    let per_face_summary = if verbose && mesh.triangle_face_ids.is_some() {
        compute_per_face_summary(mesh, &edge_face_count)
    } else {
        HashMap::new()
    };

    WatertightReport {
        edge_count,
        interior_edge_count,
        boundary_edge_count,
        non_manifold_edge_count,
        triangle_count,
        vertex_count,
        euler_characteristic: euler,
        degenerate_triangle_count,
        duplicate_triangle_count,
        boundary_edges,
        non_manifold_edges,
        per_face_summary,
    }
}

/// Internal edge info during validation.
#[derive(Clone, Debug, Default)]
struct EdgeInfo {
    count: u32,
    face_ids: Vec<u64>,
}

/// Compute per-face watertight summary.
///
/// For each face, counts how many of its edges are boundary edges
/// (not shared with another face). A fully watertight face should have
/// all its edges shared with at least one other face.
fn compute_per_face_summary(
    mesh: &TriangleMesh,
    edge_face_count: &HashMap<(u32, u32), EdgeInfo>,
) -> HashMap<u64, FaceWatertightSummary> {
    let mut summary: HashMap<u64, FaceWatertightSummary> = HashMap::new();

    for (tri_idx, tri) in mesh.triangles.iter().enumerate() {
        let face_id = mesh.triangle_face_ids.as_ref()
            .and_then(|ids| ids.get(tri_idx).copied())
            .unwrap_or(0);

        if face_id == 0 {
            continue;
        }

        let entry = summary.entry(face_id).or_default();
        entry.triangle_count += 1;

        // Check each edge of this triangle
        let edges = [
            (tri[0].min(tri[1]), tri[0].max(tri[1])),
            (tri[1].min(tri[2]), tri[1].max(tri[2])),
            (tri[2].min(tri[0]), tri[2].max(tri[0])),
        ];

        for edge in &edges {
            if let Some(info) = edge_face_count.get(edge) {
                // An edge is a "boundary edge for this face" if it only touches
                // triangles from this same face (count == 1 or all from same face_id).
                // For a true boundary edge (count==1), it's definitely not shared.
                if info.count == 1 {
                    entry.boundary_edge_count += 1;
                }
                // Note: We don't count count==2 edges where both faces have the same
                // face_id as boundary (that would be a self-fold, very rare).
            }
        }
    }

    summary
}

/// Compute squared distance between two 3D points.
fn dist_sq(a: &Point3d, b: &Point3d) -> f64 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    let dz = a.z - b.z;
    dx * dx + dy * dy + dz * dz
}

/// Compute the 3D area of a triangle.
fn triangle_area_3d(v0: &Point3d, v1: &Point3d, v2: &Point3d) -> f64 {
    let e1x = v1.x - v0.x;
    let e1y = v1.y - v0.y;
    let e1z = v1.z - v0.z;
    let e2x = v2.x - v0.x;
    let e2y = v2.y - v0.y;
    let e2z = v2.z - v0.z;
    let cx = e1y * e2z - e1z * e2y;
    let cy = e1z * e2x - e1x * e2z;
    let cz = e1x * e2y - e1y * e2x;
    (cx * cx + cy * cy + cz * cz).sqrt() * 0.5
}

// ============================================================
// Vertex compaction — remove unused vertices after mesh surgery
// ============================================================

/// Weld (merge) vertices that are connected by short boundary edges.
///
/// This fixes the "seam mismatch" problem where two adjacent faces share
/// a geometric edge but produce slightly different discretizations (e.g.,
/// a plane face uses a full circle while a NURBS face uses a half-arc of
/// the same circle). The resulting vertices are close but not identical,
/// creating short boundary edges (holes) in the merged mesh.
///
/// Algorithm:
/// 1. Find all boundary edges (edges used by only 1 triangle).
/// 2. For each short boundary edge (length < weld_tolerance), find the
///    closest vertex from another triangle that's within weld_tolerance.
/// 3. Merge (weld) the two vertices — replace one index with the other
///    in ALL triangles.
/// 4. Remove degenerate triangles created by the welding.
/// 5. Compact the vertex array.
///
/// This is SAFE because:
/// - Only vertices connected by short boundary edges are candidates
/// - The weld tolerance is small (typically 0.5mm or 0.1% of model scale)
/// - Vertices farther apart are never merged
pub fn weld_boundary_edge_vertices(mesh: &mut TriangleMesh, weld_tolerance: f64) {
    use std::collections::{HashMap, HashSet};

    if mesh.triangles.is_empty() || weld_tolerance <= 0.0 {
        return;
    }

    let weld_tol_sq = weld_tolerance * weld_tolerance;

    // ── Face-aware weld guard ───────────────────────────────────────────
    //
    // For each vertex, precompute the SET of face IDs (TopoId of the source
    // BRep face) that use it. This lets us refuse to weld two boundary
    // vertices that share ANY face — because welding vertices on the SAME
    // face corrupts that face's triangulation.
    //
    // The classic failure case this prevents: a thin annulus face where
    // R_outer − R_inner < weld_tolerance (e.g., 3.05.078.stp Step#87 has
    // R_outer=37.5, R_inner=35.22, annulus width=2.28mm, but
    // weld_tolerance=2.6mm). Without this guard, every outer-ring vertex
    // gets welded to the inner-ring vertex at the same angular position
    // (they're 2.28mm apart, within tolerance), collapsing all the
    // annulus-fill triangles into degenerate sliver triangles along the
    // outer ring. The UV triangulation is correct (it's computed before
    // weld), but the 3D mesh ends up with only ~60% of the triangles and
    // none of them span the annulus.
    //
    // The check is: if vertex_face_ids[v1] and vertex_face_ids[candidate]
    // share ANY face ID, the two vertices are on the same face (or share a
    // seam) — refuse to weld. Seam vertices (used by 2+ faces) can still
    // be welded to vertices from a DIFFERENT face (no shared face ID).
    let vertex_face_ids: Vec<HashSet<u64>> = if let Some(ref face_ids) = mesh.triangle_face_ids {
        let mut vfids: Vec<HashSet<u64>> = vec![HashSet::new(); mesh.vertices.len()];
        for (tri_idx, tri) in mesh.triangles.iter().enumerate() {
            let fid = face_ids.get(tri_idx).copied().unwrap_or(u64::MAX);
            if fid == u64::MAX { continue; }
            for &vi in tri {
                // vi is u32, safe to index into vfids
                let idx = vi as usize;
                if idx < vfids.len() {
                    vfids[idx].insert(fid);
                }
            }
        }
        vfids
    } else {
        // No face IDs available — cannot do face-aware check.
        // Fall back to empty sets (effectively disabling the guard).
        vec![HashSet::new(); mesh.vertices.len()]
    };

    // Helper: returns true if v1 and candidate share ANY face ID.
    // When face IDs are unavailable (empty sets), returns false (no shared
    // face) to preserve original behavior.
    #[inline]
    fn shares_face(
        vfid_a: &HashSet<u64>,
        vfid_b: &HashSet<u64>,
    ) -> bool {
        if vfid_a.is_empty() || vfid_b.is_empty() {
            return false;
        }
        // Iterate the smaller set, look up in the larger
        let (small, large) = if vfid_a.len() <= vfid_b.len() {
            (vfid_a, vfid_b)
        } else {
            (vfid_b, vfid_a)
        };
        for fid in small {
            if large.contains(fid) {
                return true;
            }
        }
        false
    }

    // PASS 2 tolerance: much tighter than PASS 1.
    //
    // PASS 1 welds SHORT BOUNDARY EDGES (length < weld_tolerance) — these are
    // legitimate seam mismatches where two faces' boundary vertices are close
    // but not bit-identical. The large weld_tolerance (e.g., 3% of model scale)
    // catches mismatches up to ~5mm observed in some STEP files.
    //
    // PASS 2 welds ANY boundary vertex to nearby boundary vertices — this is
    // meant to catch cases where two faces share a long boundary edge but
    // their vertices are at slightly different positions. However, with a
    // large tolerance, PASS 2 can INCORRECTLY weld unrelated boundary vertices
    // from different faces (e.g., drill_top.stp Face #803 torus vertices got
    // welded to flute-surface vertices 0.25 units away when weld_tolerance was
    // 0.46, corrupting the torus triangulation).
    //
    // Fix: PASS 2 uses a much tighter tolerance — only weld vertices that are
    // within 0.1% of model scale (or 100x absolute tolerance). This catches
    // legitimate seam mismatches (typically 1e-13..1e-6) without welding
    // unrelated vertices.
    //
    // The PASS 2 tolerance is derived from weld_tolerance: use the SMALLER of
    //   - weld_tolerance * 0.01 (1% of PASS 1 tolerance)
    //   - 1e-3 (absolute cap of 1mm)
    let pass2_tolerance = (weld_tolerance * 0.01).min(1e-3);
    let pass2_tol_sq = pass2_tolerance * pass2_tolerance;

    // For PASS 1, we need a distance-aware face check: only refuse the
    // weld if the two same-face vertices are FAR apart (beyond FP drift).
    // If they're very close (within pass2_tolerance), they're likely
    // legitimate FP drift on a seam vertex, and welding is safe.
    //
    // The threshold pass2_tol_sq is already computed below — we'll use it
    // for the same-face distance check in PASS 1.
    let pass2_tol_sq_for_pass1 = pass2_tol_sq;

    // Build edge → triangle count map
    let mut edge_count: HashMap<(u32, u32), usize> = HashMap::new();
    for tri in &mesh.triangles {
        let a = tri[0];
        let b = tri[1];
        let c = tri[2];
        for (v0, v1) in [(a, b), (b, c), (c, a)] {
            let key = if v0 < v1 { (v0, v1) } else { (v1, v0) };
            *edge_count.entry(key).or_insert(0) += 1;
        }
    }

    // Find short boundary edges (count == 1, length < tolerance)
    let mut short_boundary_edges: Vec<(u32, u32)> = Vec::new();
    for (edge, &count) in &edge_count {
        if count == 1 {
            let v0 = mesh.vertices[edge.0 as usize];
            let v1 = mesh.vertices[edge.1 as usize];
            let dx = v1.x - v0.x;
            let dy = v1.y - v0.y;
            let dz = v1.z - v0.z;
            let len_sq = dx * dx + dy * dy + dz * dz;
            if len_sq < weld_tol_sq {
                short_boundary_edges.push(*edge);
            }
        }
    }

    // Collect ALL boundary vertices (not just short-edge endpoints).
    // Long boundary edges often have endpoints that ARE close to other
    // vertices from adjacent faces — we want to weld those too.
    let mut boundary_vertices: HashSet<u32> = HashSet::new();
    for (edge, &count) in &edge_count {
        if count == 1 {
            boundary_vertices.insert(edge.0);
            boundary_vertices.insert(edge.1);
        }
    }

    if boundary_vertices.is_empty() {
        return;
    }

    log::warn!(
        "WELD: {} short boundary edges, {} boundary vertices (tol={:.4}mm) — welding",
        short_boundary_edges.len(), boundary_vertices.len(), weld_tolerance
    );

    // Build a spatial hash for vertex lookup
    let cell_size = weld_tolerance;
    let mut spatial: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
    for (vi, v) in mesh.vertices.iter().enumerate() {
        let cell = (
            (v.x / cell_size).floor() as i64,
            (v.y / cell_size).floor() as i64,
            (v.z / cell_size).floor() as i64,
        );
        spatial.entry(cell).or_default().push(vi as u32);
    }

    // For each short boundary edge, find a nearby vertex to weld with
    // Build a union-find structure for vertex merging
    let mut parent: Vec<u32> = (0..mesh.vertices.len() as u32).collect();
    fn find(parent: &mut Vec<u32>, x: u32) -> u32 {
        let mut root = x;
        while parent[root as usize] != root {
            root = parent[root as usize];
        }
        // Path compression
        let mut curr = x;
        while parent[curr as usize] != root {
            let next = parent[curr as usize];
            parent[curr as usize] = root;
            curr = next;
        }
        root
    }

    let mut weld_count = 0usize;
    let mut skip_same_face_count = 0usize;
    let mut skip_no_candidate_count = 0usize;

    // PASS 1: For each short boundary edge, find a nearby vertex to weld with.
    // This catches the typical seam mismatch (vertices that are CLOSE but
    // not bit-identical, connected by a short boundary edge).
    for (v0, v1) in &short_boundary_edges {
        // Try to find a vertex near v1 (the "near-corner" point) that's
        // NOT v0 or v1 itself. This would be the "exact corner" from
        // another face's discretization.
        let p1 = mesh.vertices[*v1 as usize];
        let cell = (
            (p1.x / cell_size).floor() as i64,
            (p1.y / cell_size).floor() as i64,
            (p1.z / cell_size).floor() as i64,
        );

        let mut best_match: Option<u32> = None;
        let mut best_dist_sq = weld_tol_sq;

        // Check current cell and 26 neighbors
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let neighbor_cell = (cell.0 + dx, cell.1 + dy, cell.2 + dz);
                    if let Some(candidates) = spatial.get(&neighbor_cell) {
                        for &candidate in candidates {
                            if candidate == *v0 || candidate == *v1 {
                                continue;
                            }
                            // CRITICAL: Only weld to OTHER boundary vertices.
                            //
                            // Without this check, a boundary vertex from face A
                            // (e.g., a hole boundary on a cylinder face) can be
                            // welded to an INTERIOR vertex from face B (e.g., the
                            // face that fills the hole). This corrupts face A's
                            // triangulation because the replacement vertex is at
                            // a different UV position — it lands INSIDE the hole
                            // that face A's triangulation is supposed to avoid.
                            //
                            // Bug history: drill_top.stp STEP #843 (cylinder face
                            // with 2 holes) showed triangles covering the holes
                            // because the weld step replaced hole-boundary vertices
                            // with interior vertices from the hole-filling faces.
                            if !boundary_vertices.contains(&candidate) {
                                continue;
                            }
                            // CRITICAL #2: Refuse to weld two vertices that share
                            // ANY face ID, UNLESS they're very close (within
                            // pass2_tolerance — FP drift range). Welding vertices
                            // on the SAME face that are far apart corrupts that
                            // face's triangulation — the classic failure is a
                            // thin annulus where the outer-ring and inner-ring
                            // vertices are within weld_tolerance of each other
                            // (annulus width < weld_tolerance) but both belong
                            // to the same face. Without this guard, every
                            // outer-ring vertex gets welded to the inner-ring
                            // vertex at the same angular position, collapsing
                            // the annulus-fill triangles into degenerate sliver
                            // triangles along the outer ring.
                            //
                            // The distance exemption (dist < pass2_tolerance)
                            // allows legitimate FP-drift welds on seam vertices
                            // that happen to share a face ID (e.g., when
                            // merge_deduplicating already merged a seam vertex
                            // with an adjacent face's version, giving it
                            // face_ids = {A, B}, and the weld is now trying to
                            // merge it with yet another nearby vertex from
                            // face A or B that's at FP-drift distance).
                            //
                            // Bug history: 3.05.078.stp Step#87 (plane annulus,
                            // R_outer=37.5, R_inner=35.22, width=2.28mm) lost
                            // 101 of 252 triangles (40%) when weld_tolerance
                            // was 2.6mm — the weld merged outer ring vertices
                            // with inner ring vertices, destroying the
                            // annulus triangulation while leaving the UV
                            // visualization (computed before weld) intact.
                            let pc = mesh.vertices[candidate as usize];
                            let dx = pc.x - p1.x;
                            let dy = pc.y - p1.y;
                            let dz = pc.z - p1.z;
                            let dist_sq = dx * dx + dy * dy + dz * dz;
                            if shares_face(&vertex_face_ids[*v1 as usize], &vertex_face_ids[candidate as usize])
                                && dist_sq > pass2_tol_sq_for_pass1
                            {
                                skip_same_face_count += 1;
                                continue;
                            }
                            if dist_sq < best_dist_sq {
                                best_dist_sq = dist_sq;
                                best_match = Some(candidate);
                            }
                        }
                    }
                }
            }
        }

        if let Some(target) = best_match {
            // Weld v1 → target (merge v1 into target)
            let root_v1 = find(&mut parent, *v1);
            let root_target = find(&mut parent, target);
            if root_v1 != root_target {
                parent[root_v1 as usize] = root_target;
                weld_count += 1;
            }
        } else {
            skip_no_candidate_count += 1;
        }
    }

    log::warn!(
        "WELD PASS 1: {} short edges processed, {} welded, {} skipped (same-face), {} skipped (no candidate)",
        short_boundary_edges.len(), weld_count, skip_same_face_count, skip_no_candidate_count
    );

    // PASS 2: For each boundary vertex on a LONG boundary edge, also look
    // for nearby vertices to weld with. This catches the case where a
    // vertex V is on a long boundary edge (length > weld_tol) but is
    // itself CLOSE to a vertex from another face (within pass2_tolerance).
    // Without this pass, these vertices would remain un-welded, leaving
    // boundary edges in the mesh.
    //
    // We skip vertices already processed in PASS 1 (those on short edges).
    //
    // CRITICAL: PASS 2 uses a MUCH TIGHTER tolerance than PASS 1
    // (pass2_tolerance = 1% of weld_tolerance, capped at 1e-3). This is
    // because PASS 2 welds ANY boundary vertex to nearby boundary vertices,
    // not just short-edge endpoints. With a large tolerance, PASS 2 can
    // incorrectly weld unrelated boundary vertices from different faces
    // (e.g., drill_top.stp Face #803 torus vertices got welded to flute-
    // surface vertices 0.25 units away when weld_tolerance was 0.46).
    let short_edge_vertices: HashSet<u32> = short_boundary_edges.iter()
        .flat_map(|(a, b)| [*a, *b].into_iter())
        .collect();

    // Build a SEPARATE spatial hash for PASS 2 with the tighter cell size.
    let pass2_cell_size = pass2_tolerance;
    let mut pass2_spatial: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
    for &vi in &boundary_vertices {
        let v = mesh.vertices[vi as usize];
        let cell = (
            (v.x / pass2_cell_size).floor() as i64,
            (v.y / pass2_cell_size).floor() as i64,
            (v.z / pass2_cell_size).floor() as i64,
        );
        pass2_spatial.entry(cell).or_default().push(vi);
    }

    let mut pass2_count = 0usize;
    for &v1 in &boundary_vertices {
        if short_edge_vertices.contains(&v1) {
            continue; // Already processed in PASS 1
        }

        let p1 = mesh.vertices[v1 as usize];
        let cell = (
            (p1.x / pass2_cell_size).floor() as i64,
            (p1.y / pass2_cell_size).floor() as i64,
            (p1.z / pass2_cell_size).floor() as i64,
        );

        let mut best_match: Option<u32> = None;
        let mut best_dist_sq = pass2_tol_sq;

        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let neighbor_cell = (cell.0 + dx, cell.1 + dy, cell.2 + dz);
                    if let Some(candidates) = pass2_spatial.get(&neighbor_cell) {
                        for &candidate in candidates {
                            if candidate == v1 {
                                continue;
                            }
                            // CRITICAL: Only weld to OTHER boundary vertices.
                            // See PASS 1 comment for the full rationale.
                            // Without this check, a boundary vertex from face A
                            // can be welded to an INTERIOR vertex from face B,
                            // corrupting face A's triangulation (e.g., triangles
                            // covering holes that should be empty).
                            if !boundary_vertices.contains(&candidate) {
                                continue;
                            }
                            // CRITICAL #2: Refuse to weld two vertices that share
                            // ANY face ID, UNLESS very close. See PASS 1 comment.
                            // PASS 2 already uses pass2_tol_sq as best_dist_sq
                            // threshold, so the distance exemption is automatic
                            // — but we still need the face check to prevent
                            // same-face welds at the tight tolerance.
                            let pc = mesh.vertices[candidate as usize];
                            let dx = pc.x - p1.x;
                            let dy = pc.y - p1.y;
                            let dz = pc.z - p1.z;
                            let dist_sq = dx * dx + dy * dy + dz * dz;
                            if shares_face(&vertex_face_ids[v1 as usize], &vertex_face_ids[candidate as usize])
                                && dist_sq > pass2_tol_sq
                            {
                                continue;
                            }
                            if dist_sq < best_dist_sq {
                                best_dist_sq = dist_sq;
                                best_match = Some(candidate);
                            }
                        }
                    }
                }
            }
        }

        if let Some(target) = best_match {
            let root_v1 = find(&mut parent, v1);
            let root_target = find(&mut parent, target);
            if root_v1 != root_target {
                parent[root_v1 as usize] = root_target;
                weld_count += 1;
                pass2_count += 1;
            }
        }
    }

    if pass2_count > 0 {
        log::warn!(
            "WELD: PASS 2 welded {} additional long-edge boundary vertices",
            pass2_count
        );
    }

    // PASS 3 (5.1.3): Seam-specific weld with intermediate tolerance.
    //
    // PASS 1 uses weld_tolerance (coarse, catches short boundary edges).
    // PASS 2 uses weld_tolerance * 0.01 (tight, catches near-identical vertices).
    // PASS 3 uses weld_tolerance * 0.1 (intermediate, catches seam mismatches
    // that are too large for PASS 2 but too small for PASS 1).
    //
    // This is specifically for periodic surfaces where two boundary vertices
    // at the seam (u=0 vs u=2π) differ by a small amount due to floating-point
    // precision. PASS 2 is too tight (1e-3 cap), PASS 1 requires a short edge
    // between them. PASS 3 bridges the gap.
    {
        let pass3_tolerance = (weld_tolerance * 0.1).min(0.01).max(1e-5);
        let pass3_tol_sq = pass3_tolerance * pass3_tolerance;

        // Build spatial hash for PASS 3
        let pass3_cell_size = pass3_tolerance;
        let mut pass3_spatial: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
        for &vi in &boundary_vertices {
            let v = mesh.vertices[vi as usize];
            let cell = (
                (v.x / pass3_cell_size).floor() as i64,
                (v.y / pass3_cell_size).floor() as i64,
                (v.z / pass3_cell_size).floor() as i64,
            );
            pass3_spatial.entry(cell).or_default().push(vi);
        }

        let mut pass3_count = 0usize;
        // Only check vertices that are on boundary edges but NOT already welded
        // (i.e., they still have a different root in the union-find)
        for &v1 in &boundary_vertices {
            let p1 = mesh.vertices[v1 as usize];
            let cell = (
                (p1.x / pass3_cell_size).floor() as i64,
                (p1.y / pass3_cell_size).floor() as i64,
                (p1.z / pass3_cell_size).floor() as i64,
            );

            let mut best_match: Option<u32> = None;
            let mut best_dist_sq = pass3_tol_sq;

            for dx in -1i64..=1 {
                for dy in -1i64..=1 {
                    for dz in -1i64..=1 {
                        let neighbor_cell = (cell.0 + dx, cell.1 + dy, cell.2 + dz);
                        if let Some(candidates) = pass3_spatial.get(&neighbor_cell) {
                            for &candidate in candidates {
                                if candidate == v1 { continue; }
                                if !boundary_vertices.contains(&candidate) { continue; }
                                // Skip if already welded to the same root
                                let root_v1 = find(&mut parent, v1);
                                let root_cand = find(&mut parent, candidate);
                                if root_v1 == root_cand { continue; }

                                let pc = mesh.vertices[candidate as usize];
                                let ddx = pc.x - p1.x;
                                let ddy = pc.y - p1.y;
                                let ddz = pc.z - p1.z;
                                let dist_sq = ddx * ddx + ddy * ddy + ddz * ddz;
                                // CRITICAL #2: Refuse to weld two vertices that share
                                // ANY face ID, UNLESS very close. See PASS 1 comment.
                                // PASS 3 uses pass3_tol_sq as best_dist_sq threshold,
                                // so the distance exemption is automatic.
                                if shares_face(&vertex_face_ids[v1 as usize], &vertex_face_ids[candidate as usize])
                                    && dist_sq > pass3_tol_sq
                                {
                                    continue;
                                }
                                if dist_sq < best_dist_sq {
                                    best_dist_sq = dist_sq;
                                    best_match = Some(candidate);
                                }
                            }
                        }
                    }
                }
            }

            if let Some(target) = best_match {
                let root_v1 = find(&mut parent, v1);
                let root_target = find(&mut parent, target);
                if root_v1 != root_target {
                    parent[root_v1 as usize] = root_target;
                    weld_count += 1;
                    pass3_count += 1;
                }
            }
        }

        if pass3_count > 0 {
            log::warn!(
                "WELD: PASS 3 (seam-specific) welded {} boundary vertices (tol={:.6})",
                pass3_count, pass3_tolerance
            );
        }
    }

    if weld_count == 0 {
        log::warn!("WELD: no vertices welded");
        return;
    }

    log::warn!("WELD: {} vertices welded", weld_count);

    // Apply the welding: replace all vertex indices with their root.
    // Strategy: apply ALL welds, then remove degenerate triangles
    // and cross-face duplicate triangles that create non-manifold edges.
    // Previous approaches tried to revert problematic welds, but this
    // was too aggressive (reverting 135 welds when only 3 root groups
    // caused problems). Instead, we accept a few lost triangles in
    // exchange for dramatically better watertightness.
    let root_map: Vec<u32> = (0..mesh.vertices.len() as u32)
        .map(|i| find(&mut parent, i))
        .collect();

    // Update all triangles
    let mut removed_degenerate = 0usize;
    let mut kept_tris = Vec::with_capacity(mesh.triangles.len());
    let mut kept_face_ids = Vec::with_capacity(mesh.triangles.len());
    let face_ids = mesh.triangle_face_ids.take();

    for (ti, tri) in mesh.triangles.iter().enumerate() {
        let a = root_map[tri[0] as usize];
        let b = root_map[tri[1] as usize];
        let c = root_map[tri[2] as usize];
        if a == b || b == c || a == c {
            removed_degenerate += 1;
            // Log which face loses a triangle
            let fid = face_ids.as_ref().and_then(|ids| ids.get(ti)).copied().unwrap_or(u64::MAX);
            if removed_degenerate <= 10 {
                let orig_a = tri[0]; let orig_b = tri[1]; let orig_c = tri[2];
                eprintln!(
                    "WELD_DEGEN: tri[{}] face_id={} degenerate after weld: ({},{},{}) → ({},{},{})",
                    ti, fid, orig_a, orig_b, orig_c, a, b, c
                );
            }
            continue;
        }
        kept_tris.push([a, b, c]);
        if let Some(ref ids) = face_ids {
            kept_face_ids.push(ids[ti]);
        }
    }

    mesh.triangles = kept_tris;
    mesh.triangle_face_ids = if kept_face_ids.is_empty() { None } else { Some(kept_face_ids) };

    // Remove duplicate triangles (same 3 indices in any order).
    // NOTE: After welding, two triangles from DIFFERENT faces may end up
    // with the same sorted vertex indices. Removing these as "duplicates"
    // creates holes in one of the faces (e.g., Step#87 plane face loses
    // 101 triangles, Step#78 cone face loses 50). 
    //
    // We only remove duplicates if they belong to the SAME face (same face_id).
    // Cross-face duplicates are preserved to avoid creating holes.
    // The later `remove_duplicate_triangles` post-processing step handles
    // true geometric duplicates that would create non-manifold edges.
    let mut seen: HashMap<[u32; 3], u64> = HashMap::with_capacity(mesh.triangles.len());
    let mut unique_tris = Vec::with_capacity(mesh.triangles.len());
    let mut unique_ids = Vec::with_capacity(mesh.triangles.len());
    let face_ids = mesh.triangle_face_ids.take();
    let mut cross_face_dup_removed = 0usize;
    for (ti, tri) in mesh.triangles.iter().enumerate() {
        let mut sorted = [tri[0], tri[1], tri[2]];
        sorted.sort();
        let fid = face_ids.as_ref().and_then(|ids| ids.get(ti)).copied().unwrap_or(u64::MAX);
        if let Some(&existing_fid) = seen.get(&sorted) {
            // Same vertex indices — only remove if from the same face
            if existing_fid == fid {
                // Same face: true duplicate, remove it
                continue;
            } else {
                // Different faces: cross-face duplicate, KEEP it to avoid holes
                cross_face_dup_removed += 1;
                unique_tris.push(*tri);
                if let Some(ref ids) = face_ids {
                    unique_ids.push(ids[ti]);
                }
            }
        } else {
            seen.insert(sorted, fid);
            unique_tris.push(*tri);
            if let Some(ref ids) = face_ids {
                unique_ids.push(ids[ti]);
            }
        }
    }
    let dup_removed = mesh.triangles.len() - unique_tris.len();
    mesh.triangles = unique_tris;
    mesh.triangle_face_ids = if unique_ids.is_empty() { None } else { Some(unique_ids) };

    log::warn!(
        "WELD: removed {} degenerate + {} same-face duplicate triangles after welding (kept {} cross-face duplicates)",
        removed_degenerate, dup_removed, cross_face_dup_removed
    );
    eprintln!("WELD_DETAIL: degenerate_removed={} same_face_dup_removed={} cross_face_dup_kept={}", removed_degenerate, dup_removed, cross_face_dup_removed);

    // Compact vertices
    compact_vertices(mesh);
}

/// Remove unused vertices from the mesh and renumber indices.
pub fn compact_vertices(mesh: &mut TriangleMesh) {
    // Find which vertices are used
    let mut used = vec![false; mesh.vertices.len()];
    for tri in &mesh.triangles {
        used[tri[0] as usize] = true;
        used[tri[1] as usize] = true;
        used[tri[2] as usize] = true;
    }

    // Build old-to-new mapping
    let mut old_to_new: Vec<u32> = vec![0; mesh.vertices.len()];
    let mut new_vertices = Vec::with_capacity(mesh.vertices.len());
    let mut new_normals: Vec<[f64; 3]> = Vec::with_capacity(mesh.vertices.len());
    let old_normals = mesh.normals.take();

    for (i, is_used) in used.iter().enumerate() {
        if *is_used {
            let new_idx = new_vertices.len() as u32;
            old_to_new[i] = new_idx;
            new_vertices.push(mesh.vertices[i]);
            if let Some(ref old_n) = old_normals {
                if i < old_n.len() {
                    new_normals.push(old_n[i]);
                } else {
                    new_normals.push([0.0, 0.0, 1.0]);
                }
            }
        }
    }

    // Renumber triangles
    for tri in &mut mesh.triangles {
        tri[0] = old_to_new[tri[0] as usize];
        tri[1] = old_to_new[tri[1] as usize];
        tri[2] = old_to_new[tri[2] as usize];
    }

    mesh.vertices = new_vertices;
    if old_normals.is_some() && !new_normals.is_empty() {
        mesh.normals = Some(new_normals);
    }
}

// ============================================================
// Normal smoothing — average normals across shared edges
// ============================================================

/// Compute face normals from the mesh triangles.
fn compute_face_normals(mesh: &TriangleMesh) -> Vec<[f64; 3]> {
    mesh.triangles.iter().map(|tri| {
        let v0 = mesh.vertices[tri[0] as usize];
        let v1 = mesh.vertices[tri[1] as usize];
        let v2 = mesh.vertices[tri[2] as usize];
        let e1 = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
        let e2 = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
        let nx = e1.1 * e2.2 - e1.2 * e2.1;
        let ny = e1.2 * e2.0 - e1.0 * e2.2;
        let nz = e1.0 * e2.1 - e1.1 * e2.0;
        let len = (nx * nx + ny * ny + nz * nz).sqrt();
        if len > 1e-15 { [nx / len, ny / len, nz / len] } else { [0.0, 0.0, 1.0] }
    }).collect()
}

/// Smooth vertex normals by averaging normals of all triangles sharing a vertex.
///
/// Without smoothing, each face computes its vertex normals independently from
/// `surface.normal_at(u, v)`, which produces sharp lighting discontinuities
/// (Mach bands) at shared edges. Smoothing averages the normals with
/// area-weighted contributions, producing smooth Gouraud-like shading.
///
/// # Arguments
/// * `mesh` — The triangle mesh whose normals should be smoothed.
/// * `crease_angle` — Angle in radians above which edges are considered sharp
///   and should NOT be smoothed across. Typical values: 30° = 0.524 rad,
///   45° = 0.785 rad. Set to π to smooth all edges.
pub fn smooth_normals(mesh: &mut TriangleMesh, crease_angle: f64) {
    let normals = match mesh.normals {
        Some(ref n) if n.len() == mesh.vertices.len() => n.clone(),
        Some(ref n) => {
            // Normal count doesn't match vertex count — skip smoothing
            log::warn!("smooth_normals: normal count ({}) != vertex count ({}), skipping",
                       n.len(), mesh.vertices.len());
            return;
        }
        None => return, // No normals to smooth
    };

    // Compute face normals if not present (needed for area weighting)
    let face_normals: Vec<[f64; 3]> = if let Some(ref fn_ref) = mesh.face_normals {
        if fn_ref.len() == mesh.triangles.len() {
            fn_ref.clone()
        } else {
            // Face normals array length mismatch — recompute
            compute_face_normals(mesh)
        }
    } else {
        compute_face_normals(mesh)
    };

    // Build vertex → incident triangles map
    let n_verts = mesh.vertices.len();
    let mut vertex_triangles: Vec<Vec<(usize, f64)>> = vec![Vec::new(); n_verts];
    for (ti, tri) in mesh.triangles.iter().enumerate() {
        let v0 = mesh.vertices[tri[0] as usize];
        let v1 = mesh.vertices[tri[1] as usize];
        let v2 = mesh.vertices[tri[2] as usize];
        let e1 = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
        let e2 = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
        let area = ((e1.1 * e2.2 - e1.2 * e2.1).powi(2)
                   + (e1.2 * e2.0 - e1.0 * e2.2).powi(2)
                   + (e1.0 * e2.1 - e1.1 * e2.0).powi(2)).sqrt() * 0.5;
        vertex_triangles[tri[0] as usize].push((ti, area));
        vertex_triangles[tri[1] as usize].push((ti, area));
        vertex_triangles[tri[2] as usize].push((ti, area));
    }

    // Build edge → face normals map for crease detection
    let mut edge_face_normals: HashMap<(u32, u32), Vec<[f64; 3]>> = HashMap::new();
    for (ti, tri) in mesh.triangles.iter().enumerate() {
        let edges = [
            (tri[0].min(tri[1]), tri[0].max(tri[1])),
            (tri[1].min(tri[2]), tri[1].max(tri[2])),
            (tri[2].min(tri[0]), tri[2].max(tri[0])),
        ];
        for &edge in &edges {
            edge_face_normals.entry(edge).or_default().push(face_normals[ti]);
        }
    }

    // For each vertex, compute the smoothed normal by averaging
    // the face normals of all incident triangles, weighted by area.
    // Only average across faces where the edge angle is below crease_angle.
    let mut smoothed = vec![[0.0_f64; 3]; n_verts];

    for vi in 0..n_verts {
        let incidents = &vertex_triangles[vi];
        if incidents.is_empty() {
            if vi < normals.len() {
                smoothed[vi] = normals[vi];
            }
            continue;
        }

        let mut sum_nx = 0.0_f64;
        let mut sum_ny = 0.0_f64;
        let mut sum_nz = 0.0_f64;

        // Use the face normal of the first incident triangle as reference
        let ref_fn = face_normals[incidents[0].0];

        for &(ti, area) in incidents {
            let fn_i = face_normals[ti];

            // Check if this face's normal is within crease_angle of the reference
            let dot = ref_fn[0] * fn_i[0] + ref_fn[1] * fn_i[1] + ref_fn[2] * fn_i[2];
            let angle = dot.clamp(-1.0, 1.0).acos();

            if angle <= crease_angle {
                sum_nx += fn_i[0] * area;
                sum_ny += fn_i[1] * area;
                sum_nz += fn_i[2] * area;
            }
        }

        let len = (sum_nx * sum_nx + sum_ny * sum_ny + sum_nz * sum_nz).sqrt();
        if len > 1e-15 {
            smoothed[vi] = [sum_nx / len, sum_ny / len, sum_nz / len];
        } else if vi < normals.len() {
            smoothed[vi] = normals[vi];
        }
    }

    // Only update normals if we computed valid smoothed normals
    if smoothed.iter().any(|n| n[0] != 0.0 || n[1] != 0.0 || n[2] != 0.0) {
        mesh.normals = Some(smoothed);
    }
}

/// Smooth vertex normals using adaptive crease angle based on surface type.
///
/// Instead of using a fixed 30° crease angle for all surfaces, this function
/// computes an appropriate crease angle for each face based on its surface type:
/// - Planes: 0° (sharp edges, no smoothing across face boundaries)
/// - Cylinders/Cones/Spheres/Tori: 180° (smooth everything)
/// - Revolution/Extrusion: 90° (moderate smoothing)
/// - NURBS: 60° (compromise)
///
/// This produces much better visual quality on mixed-geometry models where
/// a single fixed crease angle causes either over-smoothing on sharp edges
/// or under-smoothing on curved surfaces.
///
/// # Arguments
/// * `mesh` — The triangle mesh whose normals should be smoothed.
/// * `solid` — The source solid, used to determine surface types per face.
pub fn smooth_normals_adaptive(mesh: &mut TriangleMesh, solid: &Solid) {
    let normals = match mesh.normals {
        Some(ref n) if n.len() == mesh.vertices.len() => n.clone(),
        Some(ref n) => {
            log::warn!("smooth_normals_adaptive: normal count ({}) != vertex count ({}), skipping",
                       n.len(), mesh.vertices.len());
            return;
        }
        None => return,
    };

    // Compute face normals
    let face_normals: Vec<[f64; 3]> = if let Some(ref fn_ref) = mesh.face_normals {
        if fn_ref.len() == mesh.triangles.len() {
            fn_ref.clone()
        } else {
            compute_face_normals(mesh)
        }
    } else {
        compute_face_normals(mesh)
    };

    // Build a mapping from face_id → surface type for adaptive crease angles
    let mut face_crease_angles: HashMap<u64, f64> = HashMap::new();
    for face in solid.faces() {
        if let Some(ref surface) = face.surface {
            let angle = compute_adaptive_crease_angle(surface);
            face_crease_angles.insert(face.id.to_u64(), angle);
        }
    }

    // Build vertex → incident triangles map
    let n_verts = mesh.vertices.len();
    let mut vertex_triangles: Vec<Vec<(usize, f64)>> = vec![Vec::new(); n_verts];
    for (ti, tri) in mesh.triangles.iter().enumerate() {
        let v0 = mesh.vertices[tri[0] as usize];
        let v1 = mesh.vertices[tri[1] as usize];
        let v2 = mesh.vertices[tri[2] as usize];
        let e1 = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
        let e2 = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
        let area = ((e1.1 * e2.2 - e1.2 * e2.1).powi(2)
                   + (e1.2 * e2.0 - e1.0 * e2.2).powi(2)
                   + (e1.0 * e2.1 - e1.1 * e2.0).powi(2)).sqrt() * 0.5;
        vertex_triangles[tri[0] as usize].push((ti, area));
        vertex_triangles[tri[1] as usize].push((ti, area));
        vertex_triangles[tri[2] as usize].push((ti, area));
    }

    // For each vertex, compute the smoothed normal using adaptive crease angle
    let mut smoothed = vec![[0.0_f64; 3]; n_verts];

    for vi in 0..n_verts {
        let incidents = &vertex_triangles[vi];
        if incidents.is_empty() {
            if vi < normals.len() {
                smoothed[vi] = normals[vi];
            }
            continue;
        }

        let mut sum_nx = 0.0_f64;
        let mut sum_ny = 0.0_f64;
        let mut sum_nz = 0.0_f64;

        // Use the face normal of the first incident triangle as reference
        let ref_fn = face_normals[incidents[0].0];

        // Get the crease angle for this vertex's most representative face
        // Use the largest face's crease angle as the smoothing threshold
        let crease_angle = incidents.iter()
            .filter_map(|&(ti, _)| {
                mesh.triangle_face_ids.as_ref()
                    .and_then(|ids| ids.get(ti).copied())
                    .and_then(|fid| face_crease_angles.get(&fid))
            })
            .copied()
            .max_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
            .unwrap_or(std::f64::consts::FRAC_PI_6); // Default 30°

        for &(ti, area) in incidents {
            let fn_i = face_normals[ti];

            // Check if this face's normal is within crease_angle of the reference
            let dot = ref_fn[0] * fn_i[0] + ref_fn[1] * fn_i[1] + ref_fn[2] * fn_i[2];
            let angle = dot.clamp(-1.0, 1.0).acos();

            if angle <= crease_angle {
                sum_nx += fn_i[0] * area;
                sum_ny += fn_i[1] * area;
                sum_nz += fn_i[2] * area;
            }
        }

        let len = (sum_nx * sum_nx + sum_ny * sum_ny + sum_nz * sum_nz).sqrt();
        if len > 1e-15 {
            smoothed[vi] = [sum_nx / len, sum_ny / len, sum_nz / len];
        } else if vi < normals.len() {
            smoothed[vi] = normals[vi];
        }
    }

    if smoothed.iter().any(|n| n[0] != 0.0 || n[1] != 0.0 || n[2] != 0.0) {
        mesh.normals = Some(smoothed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: create a closed cube mesh (8 vertices, 12 triangles)
    fn make_cube_mesh() -> TriangleMesh {
        let mut mesh = TriangleMesh::new();
        let v = [
            Point3d::new(0.0, 0.0, 0.0), // 0
            Point3d::new(1.0, 0.0, 0.0), // 1
            Point3d::new(1.0, 1.0, 0.0), // 2
            Point3d::new(0.0, 1.0, 0.0), // 3
            Point3d::new(0.0, 0.0, 1.0), // 4
            Point3d::new(1.0, 0.0, 1.0), // 5
            Point3d::new(1.0, 1.0, 1.0), // 6
            Point3d::new(0.0, 1.0, 1.0), // 7
        ];
        for p in &v {
            mesh.add_vertex(*p);
        }
        // Bottom (z=0)
        mesh.add_triangle(0, 2, 1);
        mesh.add_triangle(0, 3, 2);
        // Top (z=1)
        mesh.add_triangle(4, 5, 6);
        mesh.add_triangle(4, 6, 7);
        // Front (y=0)
        mesh.add_triangle(0, 1, 5);
        mesh.add_triangle(0, 5, 4);
        // Back (y=1)
        mesh.add_triangle(3, 7, 6);
        mesh.add_triangle(3, 6, 2);
        // Left (x=0)
        mesh.add_triangle(0, 4, 7);
        mesh.add_triangle(0, 7, 3);
        // Right (x=1)
        mesh.add_triangle(1, 2, 6);
        mesh.add_triangle(1, 6, 5);
        mesh
    }

    #[test]
    fn test_cube_watertight() {
        let mesh = make_cube_mesh();
        let report = validate_watertight(&mesh, true);
        assert!(report.is_watertight(),
            "Cube should be watertight, but has {} boundary edges, {} non-manifold edges",
            report.boundary_edge_count, report.non_manifold_edge_count);
        assert_eq!(report.euler_characteristic, 2,
            "Cube Euler characteristic should be 2");
        assert_eq!(report.degenerate_triangle_count, 0);
        assert_eq!(report.duplicate_triangle_count, 0);
    }

    #[test]
    fn test_open_mesh_has_boundary() {
        let mut mesh = TriangleMesh::new();
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(0.5, 1.0, 0.0));
        mesh.add_triangle(0, 1, 2);

        let report = validate_watertight(&mesh, false);
        assert!(!report.is_watertight());
        assert_eq!(report.boundary_edge_count, 3);
        assert_eq!(report.interior_edge_count, 0);
    }

    #[test]
    fn test_non_manifold_edge_detected() {
        // Create a non-manifold configuration: two triangles sharing an edge
        // with a third triangle also sharing that edge
        let mut mesh = TriangleMesh::new();
        // 4 vertices: two triangles sharing edge 0-1, and a third also sharing it
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));  // 0
        mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));  // 1
        mesh.add_vertex(Point3d::new(0.5, 1.0, 0.0));  // 2
        mesh.add_vertex(Point3d::new(0.5, -1.0, 0.0)); // 3

        mesh.add_triangle(0, 1, 2); // Edge 0-1 shared by all 3
        mesh.add_triangle(0, 1, 3);
        // This creates a non-manifold edge: 0-1 has count=2 (still OK)
        // Let's add a third triangle to make it non-manifold
        mesh.add_vertex(Point3d::new(0.5, 0.5, 1.0)); // 4
        mesh.add_triangle(0, 1, 4); // Now edge 0-1 has count=3

        let report = validate_watertight(&mesh, true);
        assert!(!report.is_manifold());
        assert!(report.non_manifold_edge_count > 0);
    }

    #[test]
    fn test_tetrahedron_watertight() {
        let mut mesh = TriangleMesh::new();
        mesh.add_vertex(Point3d::new(1.0, 1.0, 1.0));
        mesh.add_vertex(Point3d::new(1.0, -1.0, -1.0));
        mesh.add_vertex(Point3d::new(-1.0, 1.0, -1.0));
        mesh.add_vertex(Point3d::new(-1.0, -1.0, 1.0));
        mesh.add_triangle(0, 1, 2);
        mesh.add_triangle(0, 1, 3);
        mesh.add_triangle(0, 2, 3);
        mesh.add_triangle(1, 2, 3);

        let report = validate_watertight(&mesh, false);
        assert!(report.is_watertight(), "Tetrahedron should be watertight");
        assert_eq!(report.euler_characteristic, 2);
    }

    #[test]
    fn test_per_face_summary() {
        let mut mesh = make_cube_mesh();
        // Assign face IDs: 2 triangles per face, 6 faces
        mesh.triangle_face_ids = Some(vec![1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6]);

        let report = validate_watertight(&mesh, true);
        assert!(report.is_watertight());
        // Each face should have 2 triangles and 0 boundary edges
        for (&face_id, summary) in &report.per_face_summary {
            assert_eq!(summary.triangle_count, 2,
                "Face {} should have 2 triangles, got {}", face_id, summary.triangle_count);
        }
    }

    #[test]
    fn test_degenerate_triangle_detected() {
        let mut mesh = TriangleMesh::new();
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0)); // Same as vertex 0
        mesh.add_triangle(0, 1, 2);

        let report = validate_watertight(&mesh, false);
        assert!(report.degenerate_triangle_count > 0);
    }
}
