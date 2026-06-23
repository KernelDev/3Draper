// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Mesh decimation for Level-of-Detail (LOD) support.
//!
//! Implements a simple, topology-preserving "shortest-edge collapse" decimation
//! algorithm suitable for reducing triangle count on BREP-derived meshes
//! when the user selects a lower LOD in the viewer.
//!
//! ## Algorithm
//!
//! 1. **Weld vertices**: Merge coincident vertices (within 1e-6 tolerance) so
//!    that adjacent triangles truly share edges. Without welding, the adjacency
//!    analysis sees far fewer internal edges than it should.
//!
//! 2. **Build adjacency**: For each undirected edge (a,b), count how many
//!    triangles contain it. An edge shared by exactly 2 triangles is "internal"
//!    and is a candidate for collapse. Boundary edges (1 triangle) are NEVER
//!    collapsed — this preserves the silhouette of the model.
//!
//! 3. **Identify boundary vertices**: A vertex is a "boundary vertex" if it is
//!    incident to at least one boundary edge. Boundary vertices must NEVER be
//!    moved — moving them would deform the silhouette.
//!
//! 4. **Edge cost**: The cost of collapsing edge (a,b) is its length. Shorter
//!    edges have lower cost and are collapsed first. This naturally preserves
//!    the shape: large features (long edges) survive, small details (short
//!    edges) are removed.
//!
//! 5. **Edge collapse**: To collapse edge (a,b):
//!    - Compute the target position:
//!      - If `a` is a boundary vertex → target = `a` (don't move `a`)
//!      - Else if `b` is a boundary vertex → target = `b` (don't move `b`)
//!      - Else → target = midpoint of `a` and `b`
//!    - Move both endpoints to the target position (one of them stays put).
//!    - For every triangle that contained `b`, replace `b` with `a`.
//!    - Remove now-degenerate triangles (those where two vertices coincide).
//!
//! 6. **Termination**: Stop when the triangle count reaches
//!    `(original_count * keep_ratio).round()` OR no more collapsible edges remain.
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

    // Weld vertices first — for STEP-derived meshes, per-face triangulation
    // may produce duplicate vertices on shared edges even after the edge cache.
    // Without welding, the adjacency map sees far fewer internal edges than
    // it should, and decimation gets stuck early.
    weld_vertices(mesh);

    // Iterate: build adjacency, find shortest collapsible internal edge,
    // collapse it. We re-build the adjacency map every iteration — for the
    // mesh sizes we deal with (≤ ~50k triangles per BREP instance), this is
    // fast enough.
    let mut iterations = 0;
    let max_iterations = original_count * 2; // Safety valve
    while mesh.triangles.len() > target_count && iterations < max_iterations {
        let adjacency = build_adjacency(mesh);
        let Some((va, vb, target_pos)) = find_shortest_collapsible_edge(mesh, &adjacency) else {
            // No more collapsible edges — stop early.
            break;
        };
        collapse_edge(mesh, va, vb, target_pos);
        iterations += 1;
    }

    // Compact the vertex array (remove dead/orphaned vertices)
    compact_vertices(mesh);

    (original_count, mesh.triangles.len())
}

/// Per-edge adjacency info.
struct Adjacency {
    /// Edge key → (count, last_triangle_index)
    /// We don't need triangle indices, just the count.
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

/// Collapse edge (va, vb): merge vb into va at `target_pos`, remove degenerate triangles.
///
/// `target_pos` is computed by the caller to preserve boundary vertices:
/// - If `va` is a boundary vertex → `target_pos = va` (don't move va)
/// - Else if `vb` is a boundary vertex → `target_pos = vb` (don't move vb)
/// - Else → `target_pos = midpoint(va, vb)`
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
}
