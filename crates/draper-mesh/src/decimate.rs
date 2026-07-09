// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Mesh decimation for Level-of-Detail (LOD) support.
//!
//! Implements a topology-preserving "shortest-edge collapse" decimation
//! algorithm suitable for reducing triangle count on BREP-derived meshes
//! when the user selects a lower LOD in the viewer.
//!
//! ## Algorithm (batched Union-Find — O(n log n))
//!
//! 1. **Weld vertices**: Merge coincident vertices (within 1e-6 tolerance) so
//!    that adjacent triangles truly share edges.
//!
//! 2. **Build adjacency** (once): For each undirected edge (a,b), count how
//!    many triangles contain it. Identify boundary vertices as those
//!    incident to at least one boundary edge (count == 1).
//!
//! 3. **Collect candidates**: All internal edges (count == 2) where NOT both
//!    endpoints are boundary vertices. Compute their squared length.
//!
//! 4. **Sort candidates by length ascending**: shortest edges first.
//!
//! 5. **Batched Union-Find**: For each candidate in order, if the two
//!    endpoints are in different Union-Find clusters, union them. Stop when
//!    we've done `collapses_needed = (original - target) / 2 + 1` unions
//!    (each union removes ~2 triangles — the two sharing the edge).
//!
//! 6. **Apply remap**: For each triangle, replace each vertex index with its
//!    Union-Find root. Remove degenerate triangles (where two indices coincide).
//!
//! 7. **Compact vertices**: Remove orphaned vertices.
//!
//! ## Performance
//!
//! - Build adjacency: O(F) where F = triangle count
//! - Collect candidates: O(E) where E = edge count (≤ 3F/2)
//! - Sort: O(E log E)
//! - Union-Find: O(E · α(n)) ≈ O(E) (inverse Ackermann, basically constant)
//! - Apply remap: O(F)
//! - Total: **O(F log F)** — vs the old O(F²) which rebuilt adjacency every
//!   iteration.
//!
//! For drill_top.stp (~7400 triangles per BREP, ~5600 collapses needed):
//! - Old: 5600 × 7400 = 41M HashMap ops per BREP
//! - New: ~7400 log 7400 ≈ 100K ops per BREP (≈400× faster)
//!
//! ## Limitations
//!
//! - This is NOT a quadric-error-metric (QEM) decimation; it uses simple
//!   Euclidean edge length as the cost. QEM would give better visual quality
//!   but is significantly more complex. For LOD previews, simple shortest-edge
//!   collapse is sufficient.
//! - Decimation is applied to the FINAL mesh (after all per-face triangulation
//!   and vertex welding). It does not preserve per-face IDs beyond simple
//!   inheritance — when a triangle is collapsed, the resulting triangle
//!   inherits the face ID of one of the two source triangles (arbitrary).
//! - Decimation NEVER removes boundary edges or moves boundary vertices, so
//!   the silhouette is preserved.
//! - The batched approach does NOT recompute edge lengths after each collapse
//!   (it uses the original edge lengths). This means the collapse order is
//!   greedy on original lengths, not on current lengths. For LOD purposes
//!   this is fine — the visual difference is negligible.

use crate::mesh::TriangleMesh;
use draper_geometry::Point3d;

/// Decimate `mesh` in-place until its triangle count reaches
/// `(original_count * keep_ratio).round()` OR no more collapsible edges remain.
///
/// `keep_ratio` is clamped to `[0.01, 1.0]`. `1.0` means no decimation.
/// Returns the (original_triangle_count, final_triangle_count).
pub fn decimate_mesh(mesh: &mut TriangleMesh, keep_ratio: f64) -> (usize, usize) {
    let original_count = mesh.triangles.len();
    let keep_ratio = keep_ratio.clamp(0.01, 1.0);
    if keep_ratio >= 1.0 || original_count < 4 {
        return (original_count, original_count);
    }
    let target_count = ((original_count as f64) * keep_ratio).round() as usize;
    let target_count = target_count.max(2);

    if mesh.triangles.len() <= target_count {
        return (original_count, mesh.triangles.len());
    }

    weld_vertices(mesh);

    // Multi-pass batched decimation.
    //
    // Each pass:
    //   1. Builds adjacency from current mesh state — O(F)
    //   2. Collects candidate edges (internal, not boundary-boundary) — O(E)
    //   3. Sorts by length — O(E log E)
    //   4. Greedily merges via Union-Find — O(E · α(n))
    //   5. Applies remap, removes degenerate triangles
    //
    // Multiple passes are needed because merging vertices can create NEW
    // internal edges (when two clusters merge, their edges combine, and
    // previously-boundary edges may become internal). A single pass would
    // miss these cascading collapse opportunities.
    //
    // Boundary handling: We use the CURRENT boundary set (recomputed each
    // pass), not the original one. This allows vertices that were originally
    // boundary but become internal (after their adjacent triangles are
    // removed) to be merged in later passes — matching the behavior of the
    // original iterative algorithm. The SILHOUETTE is still preserved because
    // the current boundary set always includes the outer perimeter.
    //
    // We do NOT compact vertices between passes — this keeps vertex indices
    // stable so the boundary set from each pass remains valid within that
    // pass. Compaction happens once at the very end.
    //
    // In practice, 2-3 passes suffice for most meshes. The loop terminates
    // when no more collapses are possible or the target is reached.
    let mut pass = 0;
    loop {
        pass += 1;
        if mesh.triangles.len() <= target_count {
            break;
        }

        // Build adjacency for current mesh state — O(F)
        let adjacency = build_adjacency(mesh);
        let current_boundary = &adjacency.boundary_vertices;

        // Collect candidate edges (internal, not boundary-boundary) — O(E)
        let mut candidates: Vec<(f64, u32, u32)> = Vec::new();
        for (&key, &count) in &adjacency.edge_count {
            if count != 2 {
                continue; // Only internal edges (shared by exactly 2 triangles)
            }
            let a = ((key >> 32) & 0xFFFFFFFF) as u32;
            let b = (key & 0xFFFFFFFF) as u32;

            let a_is_boundary = current_boundary.contains(&a);
            let b_is_boundary = current_boundary.contains(&b);

            // Don't collapse edges where BOTH endpoints are boundary vertices —
            // this would deform the silhouette (no internal vertex to absorb the move).
            if a_is_boundary && b_is_boundary {
                continue;
            }

            let pa = &mesh.vertices[a as usize];
            let pb = &mesh.vertices[b as usize];
            let dx = pa.x - pb.x;
            let dy = pa.y - pb.y;
            let dz = pa.z - pb.z;
            let len_sq = dx * dx + dy * dy + dz * dz;
            candidates.push((len_sq, a, b));
        }

        if candidates.is_empty() {
            break; // No more collapsible edges
        }

        // Sort by length ascending (shortest first) — O(E log E)
        candidates.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap_or(std::cmp::Ordering::Equal));

        // Fresh Union-Find for this pass
        let n_vertices = mesh.vertices.len();
        let mut parent: Vec<u32> = (0..n_vertices as u32).collect();

        // Each union removes ~2 triangles (the two sharing the edge).
        // Limit the batched algorithm to 70% of the needed collapses — the
        // remaining 30% is handled by the iterative fallback, which can
        // cascade (rebuild adjacency after each collapse) to reach the target.
        //
        // Without this limit, the batched algorithm would merge ALL interior
        // vertices, leaving a mesh where every vertex is on the boundary
        // (no more collapsible edges). The iterative fallback would then be
        // stuck. By leaving some interior vertices unmerged, the iterative
        // fallback can continue cascading.
        let remaining = mesh.triangles.len().saturating_sub(target_count);
        let collapses_needed = (remaining / 2 + 1) * 7 / 10; // 70% of needed
        let mut collapses_done: usize = 0;

        // Process edges in order of increasing length
        for &(_, a, b) in &candidates {
            if collapses_done >= collapses_needed {
                break;
            }
            let ra = find(&mut parent, a);
            let rb = find(&mut parent, b);
            if ra == rb {
                continue; // Already in same cluster
            }

            // Check boundary status of CLUSTER ROOTS (not original vertices).
            //
            // This is critical: if boundary vertex `a` was previously merged
            // with interior vertex `x` (so find(x) = a), and now we process
            // candidate edge (x, c) where c is also boundary, checking
            // original_boundary.contains(&x) would be false. We'd then merge
            // a's cluster into c's cluster, losing a's position.
            //
            // By checking the cluster roots, we correctly identify that a's
            // cluster (root = a, boundary) should be preserved.
            let ra_is_boundary = current_boundary.contains(&ra);
            let rb_is_boundary = current_boundary.contains(&rb);

            if ra_is_boundary && rb_is_boundary {
                // Both cluster roots are boundary — skip to preserve both
                continue;
            }

            if !ra_is_boundary && rb_is_boundary {
                // Keep b's cluster (boundary), merge a into b
                parent[ra as usize] = rb;
            } else {
                // Keep a's cluster (either a is boundary, or neither is boundary)
                parent[rb as usize] = ra;
            }
            collapses_done += 1;
        }

        if collapses_done == 0 {
            break; // No progress
        }

        // Apply remap to triangles — O(F)
        for tri in mesh.triangles.iter_mut() {
            tri[0] = find(&mut parent, tri[0]);
            tri[1] = find(&mut parent, tri[1]);
            tri[2] = find(&mut parent, tri[2]);
        }

        // Remove degenerate triangles (where two indices coincide) — O(F)
        mesh.triangles.retain(|t| t[0] != t[1] && t[1] != t[2] && t[0] != t[2]);

        // NOTE: Do NOT compact_vertices here — vertex indices must remain
        // stable for the boundary set to stay valid across passes.
        // Compaction happens once after the loop.

        // Safety: limit to 10 passes to avoid infinite loop on pathological meshes
        if pass >= 10 {
            log::warn!(
                "decimate_mesh: reached 10-pass limit ({} → {} triangles, target {})",
                original_count, mesh.triangles.len(), target_count
            );
            break;
        }
    }

    // Compact vertices before the iterative fallback — the batched algorithm
    // leaves orphaned vertices (merged but not removed). Compaction ensures
    // the iterative fallback works on a clean mesh.
    compact_vertices(mesh);

    // Iterative fallback: if the batched algorithm couldn't reach the target
    // (because it doesn't cascade within a pass), fall back to the original
    // iterative algorithm for the remaining collapses.
    //
    // The iterative algorithm rebuilds adjacency after EACH collapse, so it
    // can cascade — merging vertices that became internal after previous
    // collapses. This is slower (O(n²)) but only runs on the REDUCED mesh
    // (after the batched algorithm did the bulk of the work), so it's fast
    // in practice.
    //
    // This ensures we reach the target triangle count (or get as close as
    // possible) while keeping the overall algorithm fast.
    if mesh.triangles.len() > target_count {
        let mut iterations = 0;
        let max_iterations = mesh.triangles.len() * 2;
        while mesh.triangles.len() > target_count && iterations < max_iterations {
            let adjacency = build_adjacency(mesh);
            let Some((va, vb, target_pos)) = find_shortest_collapsible_edge(mesh, &adjacency) else {
                break; // No more collapsible edges
            };
            collapse_edge(mesh, va, vb, target_pos);
            iterations += 1;
        }
        if iterations > 0 {
            log::debug!(
                "decimate_mesh: iterative fallback did {} collapses ({} → {} triangles, target {})",
                iterations, original_count, mesh.triangles.len(), target_count
            );
        }
    }

    (original_count, mesh.triangles.len())
}

/// Union-Find `find` with path compression.
fn find(parent: &mut Vec<u32>, mut x: u32) -> u32 {
    while parent[x as usize] != x {
        // Path compression: point x directly to its grandparent
        parent[x as usize] = parent[parent[x as usize] as usize];
        x = parent[x as usize];
    }
    x
}

/// Per-edge adjacency info.
struct Adjacency {
    /// Edge key → count of triangles sharing this edge.
    edge_count: std::collections::HashMap<u64, usize>,
    /// Set of vertex indices that are boundary vertices
    /// (incident to at least one boundary edge).
    boundary_vertices: std::collections::HashSet<u32>,
}

fn build_adjacency(mesh: &TriangleMesh) -> Adjacency {
    let mut edge_count: std::collections::HashMap<u64, usize> = std::collections::HashMap::new();
    let mut boundary_vertices: std::collections::HashSet<u32> = std::collections::HashSet::new();

    for tri in &mesh.triangles {
        let edges = [
            (tri[0].min(tri[1]), tri[0].max(tri[1])),
            (tri[1].min(tri[2]), tri[1].max(tri[2])),
            (tri[0].min(tri[2]), tri[0].max(tri[2])),
        ];
        for (a, b) in edges {
            let key = ((a as u64) << 32) | (b as u64);
            *edge_count.entry(key).or_insert(0) += 1;
        }
    }

    // Identify boundary vertices (incident to edges with count == 1).
    for (&key, &count) in &edge_count {
        if count == 1 {
            let a = ((key >> 32) & 0xFFFFFFFF) as u32;
            let b = (key & 0xFFFFFFFF) as u32;
            boundary_vertices.insert(a);
            boundary_vertices.insert(b);
        }
    }

    Adjacency { edge_count, boundary_vertices }
}

/// Find the shortest collapsible internal edge.
/// Returns `Some((va, vb, target_position))` where `va < vb` are vertex indices,
/// and `target_position` is where both endpoints should move to (one of them
/// stays put if it's a boundary vertex).
///
/// Used by the iterative fallback. The batched algorithm uses Union-Find instead.
fn find_shortest_collapsible_edge(
    mesh: &TriangleMesh,
    adj: &Adjacency,
) -> Option<(u32, u32, Point3d)> {
    let mut best: Option<(f64, u32, u32)> = None;

    for (&key, &count) in &adj.edge_count {
        if count != 2 {
            continue; // Only internal edges (shared by exactly 2 triangles)
        }
        let a = ((key >> 32) & 0xFFFFFFFF) as u32;
        let b = (key & 0xFFFFFFFF) as u32;

        let a_is_boundary = adj.boundary_vertices.contains(&a);
        let b_is_boundary = adj.boundary_vertices.contains(&b);

        // Don't collapse edges where BOTH endpoints are boundary vertices —
        // this would deform the silhouette (no internal vertex to absorb the move).
        if a_is_boundary && b_is_boundary {
            continue;
        }

        let pa = &mesh.vertices[a as usize];
        let pb = &mesh.vertices[b as usize];
        let dx = pa.x - pb.x;
        let dy = pa.y - pb.y;
        let dz = pa.z - pb.z;
        let len_sq = dx * dx + dy * dy + dz * dz;
        match best {
            None => best = Some((len_sq, a, b)),
            Some((best_len, _, _)) if len_sq < best_len => best = Some((len_sq, a, b)),
            _ => {}
        }
    }

    best.map(|(_, a, b)| {
        // Determine target position: keep boundary vertex fixed if any.
        let a_is_boundary = adj.boundary_vertices.contains(&a);
        let b_is_boundary = adj.boundary_vertices.contains(&b);
        let target = if a_is_boundary {
            mesh.vertices[a as usize]
        } else if b_is_boundary {
            mesh.vertices[b as usize]
        } else {
            let pa = &mesh.vertices[a as usize];
            let pb = &mesh.vertices[b as usize];
            Point3d::new(
                (pa.x + pb.x) * 0.5,
                (pa.y + pb.y) * 0.5,
                (pa.z + pb.z) * 0.5,
            )
        };
        (a, b, target)
    })
}

/// Collapse edge (va, vb): merge vb into va at `target_pos`, remove degenerate triangles.
///
/// Used by the iterative fallback. The batched algorithm uses Union-Find instead.
fn collapse_edge(mesh: &mut TriangleMesh, va: u32, vb: u32, target_pos: Point3d) {
    if va == vb {
        return;
    }
    mesh.vertices[va as usize] = target_pos;
    mesh.vertices[vb as usize] = target_pos; // Both ends at same position before merging

    // Replace all references to vb with va in triangles.
    for tri in mesh.triangles.iter_mut() {
        for i in 0..3 {
            if tri[i] == vb {
                tri[i] = va;
            }
        }
    }

    // Remove degenerate triangles (where two indices coincide).
    mesh.triangles.retain(|t| t[0] != t[1] && t[1] != t[2] && t[0] != t[2]);
}

/// Weld coincident vertices (within tolerance `1e-6`).
///
/// After welding, triangles that referenced different vertex indices but
/// pointed to (almost) the same 3D position are reindexed to use the same
/// index. This makes the mesh truly manifold for the adjacency analysis.
///
/// We use a simple spatial hash (rounded to 1e-6 buckets) for O(n) welding.
fn weld_vertices(mesh: &mut TriangleMesh) {
    if mesh.vertices.is_empty() {
        return;
    }
    let tol = 1e-6;
    let cell_inv = 1.0 / tol;
    let mut hash: std::collections::HashMap<(i64, i64, i64), u32> = std::collections::HashMap::new();
    let mut remap: Vec<u32> = vec![u32::MAX; mesh.vertices.len()];
    let mut new_vertices: Vec<Point3d> = Vec::with_capacity(mesh.vertices.len());

    for (i, p) in mesh.vertices.iter().enumerate() {
        let key = (
            (p.x * cell_inv).round() as i64,
            (p.y * cell_inv).round() as i64,
            (p.z * cell_inv).round() as i64,
        );
        if let Some(&existing) = hash.get(&key) {
            remap[i] = existing;
        } else {
            let new_idx = new_vertices.len() as u32;
            new_vertices.push(*p);
            hash.insert(key, new_idx);
            remap[i] = new_idx;
        }
    }

    for tri in mesh.triangles.iter_mut() {
        tri[0] = remap[tri[0] as usize];
        tri[1] = remap[tri[1] as usize];
        tri[2] = remap[tri[2] as usize];
    }

    mesh.triangles.retain(|t| t[0] != t[1] && t[1] != t[2] && t[0] != t[2]);
    mesh.vertices = new_vertices;

    // Normals, if present, are no longer valid after welding — drop them.
    mesh.normals = None;
    mesh.face_normals = None;
}

/// Remove vertices that are not referenced by any triangle, and reindex.
fn compact_vertices(mesh: &mut TriangleMesh) {
    let n = mesh.vertices.len();
    if n == 0 {
        return;
    }
    let mut used = vec![false; n];
    for tri in &mesh.triangles {
        used[tri[0] as usize] = true;
        used[tri[1] as usize] = true;
        used[tri[2] as usize] = true;
    }
    let mut remap: Vec<u32> = vec![u32::MAX; n];
    let mut new_vertices: Vec<Point3d> = Vec::with_capacity(mesh.triangles.len() * 3 / 2);
    for (i, &is_used) in used.iter().enumerate() {
        if is_used {
            remap[i] = new_vertices.len() as u32;
            new_vertices.push(mesh.vertices[i]);
        }
    }
    for tri in mesh.triangles.iter_mut() {
        tri[0] = remap[tri[0] as usize];
        tri[1] = remap[tri[1] as usize];
        tri[2] = remap[tri[2] as usize];
    }
    mesh.vertices = new_vertices;
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_grid(nx: usize, ny: usize) -> TriangleMesh {
        // Flat grid in XY plane, spanning (0,0) to (nx, ny)
        let mut mesh = TriangleMesh::new();
        for j in 0..=ny {
            for i in 0..=nx {
                mesh.add_vertex(Point3d::new(i as f64, j as f64, 0.0));
            }
        }
        let idx = |i: usize, j: usize| -> u32 { (j * (nx + 1) + i) as u32 };
        for j in 0..ny {
            for i in 0..nx {
                let v00 = idx(i, j);
                let v10 = idx(i + 1, j);
                let v01 = idx(i, j + 1);
                let v11 = idx(i + 1, j + 1);
                mesh.add_triangle(v00, v10, v11);
                mesh.add_triangle(v00, v11, v01);
            }
        }
        mesh
    }

    #[test]
    fn test_decimate_grid_4x4() {
        let mut mesh = make_grid(4, 4);
        // 4x4 grid → 5*5=25 vertices, 4*4*2=32 triangles
        assert_eq!(mesh.triangle_count(), 32);
        let (orig, final_) = decimate_mesh(&mut mesh, 0.25); // Keep ~25%
        assert_eq!(orig, 32);
        // 25% of 32 = 8, but with boundary preservation, we can't always
        // reach the exact target. Allow up to ~50% of original as upper bound.
        assert!(final_ <= 16, "Expected ≤16 triangles after decimation, got {}", final_);
        assert!(final_ >= 2, "Expected ≥2 triangles after decimation, got {}", final_);
    }

    #[test]
    fn test_decimate_preserves_boundary() {
        let mut mesh = make_grid(4, 4);
        let original_boundary_pts: Vec<Point3d> = mesh.vertices.iter()
            .filter(|p| p.x == 0.0 || p.x == 4.0 || p.y == 0.0 || p.y == 4.0)
            .cloned()
            .collect();
        assert_eq!(original_boundary_pts.len(), 16); // 4*4 perimeter
        decimate_mesh(&mut mesh, 0.1);
        // All boundary points must still be present at the same position
        let mut missing = 0;
        for p in &original_boundary_pts {
            let found = mesh.vertices.iter().any(|q| {
                (q.x - p.x).abs() < 1e-6 && (q.y - p.y).abs() < 1e-6 && (q.z - p.z).abs() < 1e-6
            });
            if !found {
                missing += 1;
            }
        }
        assert_eq!(missing, 0, "{} boundary points missing after decimation", missing);
    }

    #[test]
    fn test_decimate_no_op_for_keep_ratio_1() {
        let mut mesh = make_grid(3, 3);
        let original_count = mesh.triangle_count();
        let (orig, final_) = decimate_mesh(&mut mesh, 1.0);
        assert_eq!(orig, original_count);
        assert_eq!(final_, original_count);
        assert_eq!(mesh.triangle_count(), original_count);
    }

    #[test]
    fn test_decimate_progressive_ratio() {
        let make_fresh = || make_grid(8, 8);
        let (_, r10) = decimate_mesh(&mut make_fresh(), 0.10);
        let (_, r25) = decimate_mesh(&mut make_fresh(), 0.25);
        let (_, r50) = decimate_mesh(&mut make_fresh(), 0.50);
        let (_, r100) = decimate_mesh(&mut make_fresh(), 1.0);
        assert!(r10 < r25, "Lower keep_ratio should give fewer triangles: r10={} r25={}", r10, r25);
        assert!(r25 < r50, "Lower keep_ratio should give fewer triangles: r25={} r50={}", r25, r50);
        assert!(r50 < r100, "Lower keep_ratio should give fewer triangles: r50={} r100={}", r50, r100);
    }

    #[test]
    fn test_decimate_preserves_topology() {
        let mut mesh = make_grid(6, 6);
        decimate_mesh(&mut mesh, 0.1);
        for tri in &mesh.triangles {
            assert!(tri[0] != tri[1], "Degenerate triangle");
            assert!(tri[1] != tri[2], "Degenerate triangle");
            assert!(tri[0] != tri[2], "Degenerate triangle");
            let p0 = &mesh.vertices[tri[0] as usize];
            let p1 = &mesh.vertices[tri[1] as usize];
            let p2 = &mesh.vertices[tri[2] as usize];
            let d01 = (p0.x - p1.x).powi(2) + (p0.y - p1.y).powi(2) + (p0.z - p1.z).powi(2);
            let d12 = (p1.x - p2.x).powi(2) + (p1.y - p2.y).powi(2) + (p1.z - p2.z).powi(2);
            let d02 = (p0.x - p2.x).powi(2) + (p0.y - p2.y).powi(2) + (p0.z - p2.z).powi(2);
            assert!(d01 > 1e-20, "Coincident vertices in triangle");
            assert!(d12 > 1e-20, "Coincident vertices in triangle");
            assert!(d02 > 1e-20, "Coincident vertices in triangle");
        }
    }

    /// Performance regression test: ensures decimation of a moderately-sized
    /// mesh completes quickly (well under 1 second on any modern CPU).
    ///
    /// Before the O(n log n) rewrite, this test would take 10+ seconds due
    /// to the O(n²) adjacency rebuild per iteration.
    #[test]
    fn test_decimate_performance_5k_triangles() {
        let mut mesh = make_grid(50, 50); // 50*50*2 = 5000 triangles
        let start = std::time::Instant::now();
        let (orig, final_) = decimate_mesh(&mut mesh, 0.25);
        let elapsed = start.elapsed();
        assert_eq!(orig, 5000);
        assert!(final_ < 5000, "Decimation should reduce triangle count");
        assert!(
            elapsed.as_millis() < 1000,
            "Decimation of 5k triangles took {:?} (expected <1s)",
            elapsed
        );
    }
}
