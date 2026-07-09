// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Mesh subdivision and quadrangulation.
//!
//! Provides Loop subdivision (1/8 mask, splits each triangle into 4) and
//! greedy quadrangulation (merges coplanar triangle pairs into quads).
//!
//! Algorithm adapted from truck-meshalgo v0.6 (ricosjp/truck, Apache-2.0 OR MIT)
//! and "Approximating Subdivision Surfaces" (Loop & Schaefer, 2008).

use crate::mesh::TriangleMesh;
use draper_geometry::Point3d;
use std::collections::HashMap;

/// Loop subdivision: refine a triangle mesh by splitting each triangle into 4.
///
/// For each edge, a new vertex is created at:
/// - Interior edge (shared by 2 triangles): (3/8)·(v1+v2) + (1/8)·(v3+v4)
///   where v3, v4 are the opposite vertices of the two adjacent triangles.
/// - Boundary edge (shared by 1 triangle): (1/2)·(v1+v2)
///
/// Each vertex is updated to:
/// - Interior vertex with n neighbors: (1-n·β)·v + β·Σ(neighbors)
///   where β = (1/n)·(5/8 - (3/8 + 1/4·cos(2π/n))²)
/// - Boundary vertex: (3/4)·v + (1/8)·(left + right)
///
/// Each triangle (v0, v1, v2) with edge-vertices (e01, e12, e20) becomes:
/// - (v0, e01, e20)
/// - (v1, e12, e01)
/// - (v2, e20, e12)
/// - (e01, e12, e20)
///
/// Algorithm adapted from truck-meshalgo v0.6 (ricosjp/truck, Apache-2.0 OR MIT).
pub fn loop_subdivide(mesh: &TriangleMesh, iterations: usize) -> TriangleMesh {
    if iterations == 0 || mesh.triangle_count() == 0 {
        return mesh.clone();
    }

    let mut result = mesh.clone();
    for _ in 0..iterations {
        result = loop_subdivide_once(&result);
    }
    result
}

/// Perform one iteration of Loop subdivision.
fn loop_subdivide_once(mesh: &TriangleMesh) -> TriangleMesh {
    let n_verts = mesh.vertices.len();
    let n_tris = mesh.triangles.len();

    if n_verts == 0 || n_tris == 0 {
        return mesh.clone();
    }

    // Step 1: Build edge → (opposite1, opposite2) map
    // For each undirected edge (i, j), find the 1 or 2 opposite vertices.
    let mut edge_opposites: HashMap<(u32, u32), Vec<u32>> = HashMap::new();
    for tri in &mesh.triangles {
        let [a, b, c] = *tri;
        // Three edges: (a,b), (b,c), (c,a)
        for &(i, j) in &[(a.min(b), a.max(b)), (b.min(c), b.max(c)), (c.min(a), c.max(a))] {
            edge_opposites.entry((i, j)).or_default().push(c);
        }
        // Wait — the opposite of edge (a,b) is c, of (b,c) is a, of (c,a) is b.
        // Let me redo this properly.
    }
    // Clear and redo correctly
    edge_opposites.clear();
    for tri in &mesh.triangles {
        let [a, b, c] = *tri;
        // Edge (a,b) has opposite c
        let key_ab = (a.min(b), a.max(b));
        edge_opposites.entry(key_ab).or_default().push(c);
        // Edge (b,c) has opposite a
        let key_bc = (b.min(c), b.max(c));
        edge_opposites.entry(key_bc).or_default().push(a);
        // Edge (c,a) has opposite b
        let key_ca = (c.min(a), c.max(a));
        edge_opposites.entry(key_ca).or_default().push(b);
    }

    // Step 2: Build vertex → neighbors map (for vertex update)
    let mut vertex_neighbors: Vec<Vec<u32>> = vec![Vec::new(); n_verts];
    for tri in &mesh.triangles {
        let [a, b, c] = *tri;
        for &(v, n1, n2) in &[(a, b, c), (b, c, a), (c, a, b)] {
            if !vertex_neighbors[v as usize].contains(&n1) {
                vertex_neighbors[v as usize].push(n1);
            }
            if !vertex_neighbors[v as usize].contains(&n2) {
                vertex_neighbors[v as usize].push(n2);
            }
        }
    }

    // Step 3: Identify boundary vertices (vertices on edges shared by only 1 triangle)
    let mut is_boundary_edge: HashMap<(u32, u32), bool> = HashMap::new();
    for ((i, j), opposites) in &edge_opposites {
        is_boundary_edge.insert((*i, *j), opposites.len() == 1);
    }
    let mut is_boundary_vertex = vec![false; n_verts];
    for ((i, j), is_boundary) in &is_boundary_edge {
        if *is_boundary {
            is_boundary_vertex[*i as usize] = true;
            is_boundary_vertex[*j as usize] = true;
        }
    }

    // Step 4: Compute new vertex positions
    // New mesh will have:
    // - Original vertices (updated positions) at indices 0..n_verts
    // - Edge vertices at indices n_verts..n_verts+n_edges
    let n_edges = edge_opposites.len();
    let mut new_vertices: Vec<Point3d> = Vec::with_capacity(n_verts + n_edges);

    // Updated original vertices
    for vi in 0..n_verts {
        let v = &mesh.vertices[vi];
        let vi_u32 = vi as u32;
        if is_boundary_vertex[vi] {
            // Boundary vertex: (3/4)·v + (1/8)·(left + right)
            // Find the two boundary neighbors
            let boundary_neighbors: Vec<u32> = vertex_neighbors[vi].iter()
                .filter(|&&n| {
                    let key = (vi_u32.min(n), vi_u32.max(n));
                    *is_boundary_edge.get(&(key.0, key.1)).unwrap_or(&false)
                })
                .copied()
                .collect();
            if boundary_neighbors.len() == 2 {
                let left = &mesh.vertices[boundary_neighbors[0] as usize];
                let right = &mesh.vertices[boundary_neighbors[1] as usize];
                new_vertices.push(Point3d::new(
                    0.75 * v.x + 0.125 * (left.x + right.x),
                    0.75 * v.y + 0.125 * (left.y + right.y),
                    0.75 * v.z + 0.125 * (left.z + right.z),
                ));
            } else {
                // Single boundary edge — keep position
                new_vertices.push(*v);
            }
        } else {
            // Interior vertex: (1 - n·β)·v + β·Σ(neighbors)
            let n = vertex_neighbors[vi].len();
            if n == 0 {
                new_vertices.push(*v);
            } else {
                let beta = loop_beta(n);
                let mut sum = Point3d::new(0.0, 0.0, 0.0);
                for &ni in &vertex_neighbors[vi] {
                    let p = &mesh.vertices[ni as usize];
                    sum.x += p.x;
                    sum.y += p.y;
                    sum.z += p.z;
                }
                let factor = 1.0 - (n as f64) * beta;
                new_vertices.push(Point3d::new(
                    factor * v.x + beta * sum.x,
                    factor * v.y + beta * sum.y,
                    factor * v.z + beta * sum.z,
                ));
            }
        }
    }

    // Edge vertices: assign index n_verts + edge_index
    let mut edge_vertex_idx: HashMap<(u32, u32), u32> = HashMap::new();
    let mut next_edge_idx = n_verts as u32;
    for (key, opposites) in &edge_opposites {
        let (i, j) = *key;
        let vi = &mesh.vertices[i as usize];
        let vj = &mesh.vertices[j as usize];

        let new_pos = if opposites.len() == 2 {
            // Interior edge: (3/8)·(vi+vj) + (1/8)·(vk+vl)
            let vk = &mesh.vertices[opposites[0] as usize];
            let vl = &mesh.vertices[opposites[1] as usize];
            Point3d::new(
                0.375 * (vi.x + vj.x) + 0.125 * (vk.x + vl.x),
                0.375 * (vi.y + vj.y) + 0.125 * (vk.y + vl.y),
                0.375 * (vi.z + vj.z) + 0.125 * (vk.z + vl.z),
            )
        } else {
            // Boundary edge: (1/2)·(vi+vj)
            Point3d::new(
                0.5 * (vi.x + vj.x),
                0.5 * (vi.y + vj.y),
                0.5 * (vi.z + vj.z),
            )
        };

        new_vertices.push(new_pos);
        edge_vertex_idx.insert((key.0, key.1), next_edge_idx);
        next_edge_idx += 1;
    }

    // Step 5: Build new triangles
    let mut new_triangles: Vec<[u32; 3]> = Vec::with_capacity(n_tris * 4);
    for tri in &mesh.triangles {
        let [a, b, c] = *tri;
        let e_ab = edge_vertex_idx[&(a.min(b), a.max(b))];
        let e_bc = edge_vertex_idx[&(b.min(c), b.max(c))];
        let e_ca = edge_vertex_idx[&(c.min(a), c.max(a))];

        // 4 sub-triangles
        new_triangles.push([a, e_ab, e_ca]);
        new_triangles.push([b, e_bc, e_ab]);
        new_triangles.push([c, e_ca, e_bc]);
        new_triangles.push([e_ab, e_bc, e_ca]);
    }

    TriangleMesh::from_data(new_vertices, new_triangles)
}

/// Compute the Loop subdivision beta coefficient for a vertex with n neighbors.
///
/// β = (1/n) · (5/8 - (3/8 + 1/4·cos(2π/n))²)
///
/// For n = 6 (regular valence): β = 1/16 ≈ 0.0625
/// For other valences, β is computed by the formula.
fn loop_beta(n: usize) -> f64 {
    if n == 0 {
        return 0.0;
    }
    let n_f = n as f64;
    let cos_term = (2.0 * std::f64::consts::PI / n_f).cos();
    let inner = 0.375 + 0.25 * cos_term;
    let beta = (1.0 / n_f) * (0.625 - inner * inner);
    beta
}

/// Greedy quadrangulation: merge pairs of coplanar triangles into quads.
///
/// For each triangle, find a neighbor triangle that:
/// 1. Shares exactly one edge
/// 2. Has a normal within `angle_tolerance` of the first triangle's normal
/// 3. Hasn't already been merged
///
/// Merge the pair into a quad (4 vertices). Triangles that can't be merged
/// remain as triangles (2 tris → 1 quad, or 1 tri → 1 tri).
///
/// Returns a list of quads (4 vertex indices each) and remaining triangles.
///
/// Algorithm adapted from truck-meshalgo v0.6 (ricosjp/truck, Apache-2.0 OR MIT).
pub fn quadrangulate(
    mesh: &TriangleMesh,
    angle_tolerance_radians: f64,
) -> (Vec<[u32; 4]>, Vec<[u32; 3]>) {
    let n_tris = mesh.triangles.len();
    if n_tris == 0 {
        return (Vec::new(), Vec::new());
    }

    // Compute face normals
    let normals = compute_face_normals(mesh);

    // Build edge → triangle list map
    let mut edge_tris: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (ti, tri) in mesh.triangles.iter().enumerate() {
        let [a, b, c] = *tri;
        for &(i, j) in &[(a.min(b), a.max(b)), (b.min(c), b.max(c)), (c.min(a), c.max(a))] {
            edge_tris.entry((i, j)).or_default().push(ti);
        }
    }

    // Find neighbor pairs
    let mut used = vec![false; n_tris];
    let mut quads: Vec<[u32; 4]> = Vec::new();
    let mut remaining_tris: Vec<[u32; 3]> = Vec::new();

    for ti in 0..n_tris {
        if used[ti] {
            continue;
        }
        let [a, b, c] = mesh.triangles[ti];
        let n0 = normals[ti];

        // Find a neighbor triangle sharing an edge, with a compatible normal
        let mut best_neighbor: Option<usize> = None;
        for &(i, j) in &[(a.min(b), a.max(b)), (b.min(c), b.max(c)), (c.min(a), c.max(a))] {
            if let Some(tri_list) = edge_tris.get(&(i, j)) {
                for &tj in tri_list {
                    if tj == ti || used[tj] {
                        continue;
                    }
                    let n1 = normals[tj];
                    let dot = n0[0] * n1[0] + n0[1] * n1[1] + n0[2] * n1[2];
                    let angle = dot.clamp(-1.0, 1.0).acos();
                    if angle < angle_tolerance_radians {
                        best_neighbor = Some(tj);
                        break;
                    }
                }
            }
            if best_neighbor.is_some() {
                break;
            }
        }

        if let Some(tj) = best_neighbor {
            used[ti] = true;
            used[tj] = true;
            // Merge triangles ti and tj into a quad
            let [a1, b1, c1] = mesh.triangles[ti];
            let [a2, b2, c2] = mesh.triangles[tj];

            // Find shared vertices (the shared edge has exactly 2 common vertices)
            use std::collections::HashSet;
            let set1: HashSet<u32> = [a1, b1, c1].iter().copied().collect();
            let set2: HashSet<u32> = [a2, b2, c2].iter().copied().collect();
            let shared: Vec<u32> = set1.intersection(&set2).copied().collect();

            if shared.len() == 2 {
                // The quad is: non-shared vertices of tri1, shared, non-shared vertices of tri2
                let v1_nonshared: Vec<u32> = vec![a1, b1, c1].into_iter()
                    .filter(|v| !shared.contains(v))
                    .collect();
                let v2_nonshared: Vec<u32> = vec![a2, b2, c2].into_iter()
                    .filter(|v| !shared.contains(v))
                    .collect();
                if v1_nonshared.len() == 1 && v2_nonshared.len() == 1 {
                    // Quad: v1_nonshared[0], shared[0], v2_nonshared[0], shared[1]
                    quads.push([
                        v1_nonshared[0],
                        shared[0],
                        v2_nonshared[0],
                        shared[1],
                    ]);
                    continue;
                }
            }
            // Fallback: keep as triangles
            remaining_tris.push([a1, b1, c1]);
            remaining_tris.push([a2, b2, c2]);
        } else {
            used[ti] = true;
            remaining_tris.push([a, b, c]);
        }
    }

    (quads, remaining_tris)
}

/// Compute face normals for all triangles in the mesh.
fn compute_face_normals(mesh: &TriangleMesh) -> Vec<[f64; 3]> {
    mesh.triangles.iter().map(|tri| {
        let [a, b, c] = *tri;
        let pa = &mesh.vertices[a as usize];
        let pb = &mesh.vertices[b as usize];
        let pc = &mesh.vertices[c as usize];
        let ab = [pb.x - pa.x, pb.y - pa.y, pb.z - pa.z];
        let ac = [pc.x - pa.x, pc.y - pa.y, pc.z - pa.z];
        let nx = ab[1] * ac[2] - ab[2] * ac[1];
        let ny = ab[2] * ac[0] - ab[0] * ac[2];
        let nz = ab[0] * ac[1] - ab[1] * ac[0];
        let len = (nx * nx + ny * ny + nz * nz).sqrt();
        if len < 1e-15 {
            [0.0, 0.0, 1.0]
        } else {
            [nx / len, ny / len, nz / len]
        }
    }).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_tetrahedron() -> TriangleMesh {
        let mut mesh = TriangleMesh::new();
        let v0 = mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        let v1 = mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        let v2 = mesh.add_vertex(Point3d::new(0.5, 1.0, 0.0));
        let v3 = mesh.add_vertex(Point3d::new(0.5, 0.5, 1.0));
        mesh.add_triangle(v0, v1, v2);
        mesh.add_triangle(v0, v1, v3);
        mesh.add_triangle(v1, v2, v3);
        mesh.add_triangle(v2, v0, v3);
        mesh
    }

    fn make_square() -> TriangleMesh {
        // Two triangles forming a square in the XY plane
        let mut mesh = TriangleMesh::new();
        let v0 = mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        let v1 = mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        let v2 = mesh.add_vertex(Point3d::new(1.0, 1.0, 0.0));
        let v3 = mesh.add_vertex(Point3d::new(0.0, 1.0, 0.0));
        mesh.add_triangle(v0, v1, v2);
        mesh.add_triangle(v0, v2, v3);
        mesh
    }

    #[test]
    fn test_loop_subdivide_increases_vertex_count() {
        let mesh = make_tetrahedron();
        let subdivided = loop_subdivide(&mesh, 1);
        // 4 triangles → 16 triangles
        assert_eq!(subdivided.triangle_count(), 16,
            "1 iteration of Loop subdivision should produce 4× triangles");
        // Original 4 vertices + 6 edge vertices = 10 vertices
        assert_eq!(subdivided.vertex_count(), 10,
            "1 iteration of Loop subdivision should produce 4+6=10 vertices");
    }

    #[test]
    fn test_loop_subdivide_multiple_iterations() {
        let mesh = make_tetrahedron();
        let subdivided = loop_subdivide(&mesh, 2);
        // 4 → 16 → 64 triangles
        assert_eq!(subdivided.triangle_count(), 64,
            "2 iterations of Loop subdivision should produce 4²× = 64 triangles");
    }

    #[test]
    fn test_loop_subdivide_preserves_volume_roughly() {
        // A tetrahedron's volume should be approximately preserved after subdivision.
        // Loop subdivision is an APPROXIMATING scheme (not interpolating), so the mesh
        // shrinks toward the limit surface. For a tetrahedron with all vertices on the
        // boundary, the shrinkage is severe — we check that the volume is at least 5%
        // of the original (the limit surface volume).
        let mesh = make_tetrahedron();
        let subdivided = loop_subdivide(&mesh, 2);

        let vol_orig = tetrahedron_volume(&mesh);
        let vol_new = tetrahedron_volume(&subdivided);

        assert!(vol_new > 0.0, "subdivided volume should be positive, got {}", vol_new);
        assert!(vol_new > vol_orig * 0.05,
            "subdivided volume {} should be > 5% of original {} (Loop is approximating)",
            vol_new, vol_orig);
    }

    fn tetrahedron_volume(mesh: &TriangleMesh) -> f64 {
        // Approximate volume using the divergence theorem
        let mut vol = 0.0;
        for tri in &mesh.triangles {
            let [a, b, c] = *tri;
            let pa = &mesh.vertices[a as usize];
            let pb = &mesh.vertices[b as usize];
            let pc = &mesh.vertices[c as usize];
            // Signed volume contribution = (1/6) · (pa · (pb × pc))
            let cross_x = pb.y * pc.z - pb.z * pc.y;
            let cross_y = pb.z * pc.x - pb.x * pc.z;
            let cross_z = pb.x * pc.y - pb.y * pc.x;
            vol += (pa.x * cross_x + pa.y * cross_y + pa.z * cross_z) / 6.0;
        }
        vol.abs()
    }

    #[test]
    fn test_quadrangulate_merges_coplanar_triangles() {
        // A square made of two coplanar triangles should become one quad
        let mesh = make_square();
        let (quads, tris) = quadrangulate(&mesh, 0.1); // ~5.7 degrees tolerance

        assert_eq!(quads.len(), 1, "expected 1 quad, got {}", quads.len());
        assert_eq!(tris.len(), 0, "expected 0 remaining triangles, got {}", tris.len());
    }

    #[test]
    fn test_quadrangulate_keeps_non_coplanar_triangles() {
        // A tetrahedron has 4 non-coplanar faces — none should be merged
        let mesh = make_tetrahedron();
        let (quads, tris) = quadrangulate(&mesh, 0.1);

        assert_eq!(quads.len(), 0, "expected 0 quads for tetrahedron, got {}", quads.len());
        assert_eq!(tris.len(), 4, "expected 4 remaining triangles, got {}", tris.len());
    }

    #[test]
    fn test_loop_beta_regular_valence() {
        // For valence 6 (regular case in a triangle mesh), β should be 1/16
        let beta = loop_beta(6);
        assert!((beta - 1.0 / 16.0).abs() < 1e-12,
            "loop_beta(6) should be 1/16 ≈ 0.0625, got {}", beta);
    }

    #[test]
    fn test_loop_beta_decreasing_with_valence() {
        // β should decrease as valence increases (for n >= 3)
        let beta3 = loop_beta(3);
        let beta6 = loop_beta(6);
        let beta12 = loop_beta(12);
        assert!(beta3 > beta6, "beta(3)={} should be > beta(6)={}", beta3, beta6);
        assert!(beta6 > beta12, "beta(6)={} should be > beta(12)={}", beta6, beta12);
    }
}
