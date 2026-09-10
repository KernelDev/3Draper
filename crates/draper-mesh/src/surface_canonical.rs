// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Surface-level canonical triangulation (ROADMAP_VISION_2036 Phase 1 —
//! "100% watertight", the session-28/29 line).
//!
//! ONE constrained Delaunay triangulation per NURBS surface shared by
//! multiple faces, with per-face sub-triangulation extraction.
//!
//! # Why
//!
//! The per-face CDT (`custom_cdt::triangulate_polygon_cdt`, gated by
//! `TriangulationParams::use_cdt_steiner`) produces provably hole-free
//! PER-FACE meshes, but faces sharing one NURBS surface receive the same
//! shared Steiner points (MS-2) while building DIFFERENT CDT connectivity
//! per face — Steiner-to-rim edges mismatch across faces and every such
//! edge lands in the merged mesh with a single adjacent triangle
//! (HOUSING #47598: 6035 → 14292 boundary edges when enabled). Interior
//! Steiner holes could therefore never be filled without regressing
//! cross-face watertightness.
//!
//! # How
//!
//! 1. Collect ALL faces' rim loops (outer + holes) on the shared surface,
//!    as UV + bit-identical 3D polylines from the shared edge cache.
//! 2. Build ONE triangulation seeded over the whole UV window:
//!    rim vertices + the surface-level Steiner grid, inserted greedily.
//! 3. Enforce every rim loop edge as a CONSTRAINT edge (Sloan-style
//!    crossing walk + convex swap/point split, winding-preserving).
//! 4. Extract per face: canonical triangles whose centroid lies inside
//!    the face's trimmed UV domain (outer polygon minus holes).
//!
//! Connectivity is shared by construction: an interior canonical edge
//! either lies between two triangles of the SAME face (interior to that
//! face's mesh) or between triangles assigned to DIFFERENT faces (present
//! in both face meshes → 2-adjacent in the merged position-dedup'd mesh).
//! No new boundary edges can appear, while interior Steiner coverage
//! fills the per-face holes the legacy earcutr spike-chain path leaks.
//!
//! # Never-worsen
//!
//! `build_canonical_surface_cdt` validates the result (all enforceable
//! constraint edges present, no edge with >2 adjacent triangles) and
//! returns `None` on failure — callers fall back to the legacy per-face
//! path. `extract_face_mesh` returns `None` when the caller-supplied loops
//! do not match a canonical face entry (e.g. the edge cache produced
//! different discretizations) — again legacy fallback, face by face.

use std::collections::HashMap;

use draper_geometry::{NurbsSurface, Point2d, Point3d};

use crate::custom_cdt::{orient2d, point_in_polygon};
use crate::edge_cache::deterministic_round_point;
use crate::mesh::TriangleMesh;

/// Numeric tolerance for geometric predicates (matches `custom_cdt::EPS`
/// semantics; kept local so this module never drifts from it).
const EPS: f64 = 1e-10;

/// One face's boundary loops, collected by the converter from the shared
/// edge cache (bit-identical across faces sharing an EDGE_CURVE).
#[derive(Clone, Debug)]
pub struct CanonicalFaceLoops {
    pub step_face_id: i64,
    pub forward: bool,
    pub outer_3d: Vec<Point3d>,
    pub outer_uv: Vec<Point2d>,
    pub holes_3d: Vec<Vec<Point3d>>,
    pub holes_uv: Vec<Vec<Point2d>>,
}

/// The canonical CDT for one shared NURBS surface.
///
/// Vertex layout: `[rim vertices (dedup'd by 3D bits)][steiner grid]
/// [constraint-split points][super-quad corners (removed after build)]`.
#[derive(Clone, Debug)]
pub struct CanonicalSurfaceCdt {
    /// Canonical UV vertices (window frame of the surface).
    uv: Vec<[f64; 2]>,
    /// Canonical 3D positions: rims use the edge-cache polyline positions
    /// (bit-identical to the per-face legacy path); interior/split points
    /// use `deterministic_round_point(derivatives_at(uv).point)`.
    p3d: Vec<Point3d>,
    /// Canonical triangles (CCW in UV), after super-quad removal.
    tris: Vec<[u32; 3]>,
    /// Per input face: indices into `tris` assigned by centroid extraction.
    face_tris: Vec<Vec<usize>>,
    /// The input faces (for caller-side loop matching at extraction).
    faces: Vec<CanonicalFaceLoops>,
}

impl CanonicalSurfaceCdt {
    /// Number of canonical triangles (union region, all faces).
    pub fn canonical_triangle_count(&self) -> usize {
        self.tris.len()
    }

    /// Number of input faces captured by this canonical CDT.
    pub fn face_count(&self) -> usize {
        self.faces.len()
    }

    /// Extract one face's sub-triangulation as a mesh.
    ///
    /// `boundary_3d`/`boundary_uvs`/`holes_*` must be the SAME loops the
    /// converter collected for this face (they are matched against the
    /// loops captured at build time — exact or reversed sequence of 3D
    /// position bits). Returns `None` on mismatch → legacy fallback.
    pub fn extract_face_mesh(
        &self,
        nurbs: &NurbsSurface,
        boundary_3d: &[Point3d],
        boundary_uvs: &[Point2d],
        holes_3d: &[Vec<Point3d>],
        holes_uvs: &[Vec<Point2d>],
        forward: bool,
    ) -> Option<TriangleMesh> {
        let face_idx = self.match_face(boundary_3d, holes_3d)?;
        let tri_idxs = &self.face_tris[face_idx];

        // ── Rim-contract validation (never-worsen) ──
        // Every consecutive pair of the face's rim polyline (outer + holes,
        // by 3D position bits) must be an edge of the extracted mesh: the
        // neighboring face's mesh contains exactly these edges, and a
        // missing one becomes a boundary edge in the merged BREP mesh.
        // Centroid misclassification near self-intersecting/sliver UV
        // domains can drop rim-adjacent triangles — detected here, the
        // face falls back to the legacy path instead of regressing.
        {
            let mut mesh_edges: std::collections::HashSet<(u32, u32)> =
                std::collections::HashSet::with_capacity(tri_idxs.len() * 3);
            let mut pos_index: HashMap<[u64; 3], u32> = HashMap::with_capacity(64);
            for &ti in tri_idxs {
                for &v in &self.tris[ti] {
                    let p = self.p3d[v as usize];
                    pos_index
                        .entry([p.x.to_bits(), p.y.to_bits(), p.z.to_bits()])
                        .or_insert(v);
                }
            }
            for &ti in tri_idxs {
                let tr = self.tris[ti];
                for i in 0..3 {
                    let a = pos_index
                        [&bits3(&self.p3d[tr[i] as usize])]
                        .min(pos_index[&bits3(&self.p3d[tr[(i + 1) % 3] as usize])]);
                    let b = pos_index
                        [&bits3(&self.p3d[tr[i] as usize])]
                        .max(pos_index[&bits3(&self.p3d[tr[(i + 1) % 3] as usize])]);
                    mesh_edges.insert((a, b));
                }
            }
            let check_loop = |loop3d: &[Point3d],
                              mesh_edges: &std::collections::HashSet<(u32, u32)>|
             -> bool {
                let n = loop3d.len();
                for i in 0..n {
                    let j = (i + 1) % n;
                    let (Some(&ia), Some(&ib)) = (
                        pos_index.get(&bits3(&loop3d[i])),
                        pos_index.get(&bits3(&loop3d[j])),
                    ) else {
                        return false; // rim vertex missing from the extraction
                    };
                    if !mesh_edges.contains(&(ia.min(ib), ia.max(ib))) {
                        return false;
                    }
                }
                true
            };
            if !check_loop(boundary_3d, &mesh_edges) {
                log::debug!(
                    "canonical CDT: rim contract violated for face — legacy fallback"
                );
                return None;
            }
            for h in holes_3d {
                if !check_loop(h, &mesh_edges) {
                    log::debug!(
                        "canonical CDT: hole rim contract violated — legacy fallback"
                    );
                    return None;
                }
            }
        }

        // Local emission mirrors the legacy Step 5 exactly:
        // position-based dedup within the face mesh (seam duplicates of
        // one 3D point collapse here), normals flipped for !forward,
        // position-degenerate triangle filter.
        let mut mesh = TriangleMesh::new();
        let mut position_map: HashMap<[u64; 3], u32> = HashMap::with_capacity(64);

        for &ti in tri_idxs {
            let t = self.tris[ti];
            let mut tri_indices = [0u32; 3];
            for (k, &vi) in t.iter().enumerate() {
                let p = self.p3d[vi as usize];
                let uv = self.uv[vi as usize];
                let pos_key = [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
                let entry = if let Some(&existing) = position_map.get(&pos_key) {
                    existing
                } else {
                    let derivs = nurbs.derivatives_at(uv[0], uv[1]);
                    let n = if forward {
                        derivs.normal()
                    } else {
                        let n = derivs.normal();
                        draper_geometry::Direction3d::new(-n.x, -n.y, -n.z)
                            .unwrap_or(n)
                    };
                    let li = mesh.vertices.len() as u32;
                    mesh.vertices.push(p);
                    mesh.add_vertex_normal(li, [n.x, n.y, n.z]);
                    position_map.insert(pos_key, li);
                    li
                };
                tri_indices[k] = entry;
            }
            // Position-degenerate filter (same threshold as legacy Step 5).
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
        let _ = (boundary_uvs, holes_uvs);
        Some(mesh)
    }

    /// Match caller loops against the stored faces (exact or reversed
    /// outer-loop 3D sequence; hole count must agree).
    fn match_face(
        &self,
        boundary_3d: &[Point3d],
        holes_3d: &[Vec<Point3d>],
    ) -> Option<usize> {
        for (i, f) in self.faces.iter().enumerate() {
            if f.holes_3d.len() != holes_3d.len() {
                continue;
            }
            if seq_bits_eq(&f.outer_3d, boundary_3d)
                || seq_bits_eq(&f.outer_3d, &boundary_3d.iter().rev().copied().collect::<Vec<_>>())
            {
                return Some(i);
            }
        }
        None
    }
}

fn bits3(p: &Point3d) -> [u64; 3] {
    [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]
}

fn seq_bits_eq(a: &[Point3d], b: &[Point3d]) -> bool {
    a.len() == b.len() && a.iter().zip(b.iter()).all(|(x, y)| bits3(x) == bits3(y))
}

// ============================================================
// Incremental triangulation with adjacency maintenance
// ============================================================

/// Triangulation state with incrementally-maintained edge and vertex
/// adjacency (edge → incident triangles, vertex → incident triangles).
struct Triangulation {
    verts: Vec<[f64; 2]>,
    tris: Vec<[u32; 3]>,
    /// (min, max) vertex pair → triangles using it as an edge.
    edge_map: HashMap<(u32, u32), Vec<usize>>,
    /// vertex → triangles containing it.
    vert_tris: HashMap<u32, Vec<usize>>,
}

impl Triangulation {
    fn new(verts: Vec<[f64; 2]>, seeds: impl IntoIterator<Item = [u32; 3]>) -> Self {
        let mut t = Triangulation {
            verts,
            tris: Vec::new(),
            edge_map: HashMap::new(),
            vert_tris: HashMap::new(),
        };
        for tri in seeds {
            t.add_tri(tri);
        }
        t
    }

    fn add_tri(&mut self, mut tri: [u32; 3]) -> usize {
        self.normalize_ccw(&mut tri);
        let idx = self.tris.len();
        self.tris.push(tri);
        self.index_tri(idx, tri);
        idx
    }

    fn index_tri(&mut self, idx: usize, tri: [u32; 3]) {
        for i in 0..3 {
            let a = tri[i].min(tri[(i + 1) % 3]);
            let b = tri[i].max(tri[(i + 1) % 3]);
            self.edge_map.entry((a, b)).or_default().push(idx);
        }
        for &v in &tri {
            self.vert_tris.entry(v).or_default().push(idx);
        }
    }

    fn deindex_tri(&mut self, idx: usize, tri: [u32; 3]) {
        for i in 0..3 {
            let a = tri[i].min(tri[(i + 1) % 3]);
            let b = tri[i].max(tri[(i + 1) % 3]);
            if let Some(list) = self.edge_map.get_mut(&(a, b)) {
                list.retain(|&t| t != idx);
                if list.is_empty() {
                    self.edge_map.remove(&(a, b));
                }
            }
        }
        if let Some(list) = self.vert_tris.get_mut(&tri[0]) {
            list.retain(|&t| t != idx);
        }
        if let Some(list) = self.vert_tris.get_mut(&tri[1]) {
            list.retain(|&t| t != idx);
        }
        if let Some(list) = self.vert_tris.get_mut(&tri[2]) {
            list.retain(|&t| t != idx);
        }
    }

    /// Replace `tris[idx]` with `new` (same winding), updating adjacency.
    fn replace_tri(&mut self, idx: usize, mut new: [u32; 3]) {
        self.normalize_ccw(&mut new);
        let old = self.tris[idx];
        self.deindex_tri(idx, old);
        self.tris[idx] = new;
        self.index_tri(idx, new);
    }

    /// Orientation normalization: swap the last two indices when the
    /// triangle is CW, so every triangle in the structure is CCW. Flip
    /// operations rely on this invariant (their raw index order is
    /// derived combinatorially, not geometrically).
    fn normalize_ccw(&self, tri: &mut [u32; 3]) {
        let a = self.verts[tri[0] as usize];
        let b = self.verts[tri[1] as usize];
        let c = self.verts[tri[2] as usize];
        if orient2d(a, b, c) < 0.0 {
            tri.swap(1, 2);
        }
    }

    /// Triangles adjacent across edge (a, b), other than `not_this`.
    fn neighbors_across(&self, a: u32, b: u32, not_this: usize) -> Vec<usize> {
        let key = (a.min(b), a.max(b));
        self.edge_map
            .get(&key)
            .map(|v| v.iter().copied().filter(|&t| t != not_this).collect())
            .unwrap_or_default()
    }

    fn edge_exists(&self, a: u32, b: u32) -> bool {
        self.edge_map.contains_key(&(a.min(b), a.max(b)))
    }

    /// Insert an existing vertex `vi` into the triangulation (greedy
    /// Bowyer-Watson-lite point insertion, winding-preserving).
    /// `hint` = a triangle index to start the locate walk from.
    fn insert_vertex(&mut self, vi: u32, hint: usize) -> usize {
        let p = self.verts[vi as usize];
        match self.locate(hint, p) {
            Some((ti, false)) => {
                // Interior: split into 3.
                let [a, b, c] = self.tris[ti];
                self.replace_tri(ti, [a, b, vi]);
                let t1 = self.add_tri([b, c, vi]);
                let t2 = self.add_tri([c, a, vi]);
                t1.min(t2)
            }
            Some((ti, true)) => {
                // On an edge — or coincident with an existing vertex.
                let tri = self.tris[ti];
                // Coincidence guard: a point equal to a triangle vertex
                // is a duplicate — splitting an edge at it would create
                // zero-area triangles. Skip the insertion entirely.
                if tri.iter().any(|&v| {
                    let q = self.verts[v as usize];
                    (q[0] - p[0]).abs() < 1e-12 && (q[1] - p[1]).abs() < 1e-12
                }) {
                    return ti;
                }
                let edge = nearest_edge(self.verts.as_ref(), tri, p);
                if let Some((v1, v2)) = edge {
                    self.split_edge(v1, v2, vi);
                }
                ti
            }
            None => hint, // outside the current hull (shouldn't happen with super-quad)
        }
    }

    /// Constraint-edge-protected insertion (Steiner phase): a point that
    /// lands ON a constraint edge is SKIPPED (returns `usize::MAX`) —
    /// splitting a constraint would T-junction the neighbor's rim
    /// polyline. Mirrors `custom_cdt::insert_interior_points`'s ring
    /// protection.
    fn insert_vertex_protected(
        &mut self,
        vi: u32,
        hint: usize,
        constraints: &std::collections::HashSet<(u32, u32)>,
    ) -> usize {
        let p = self.verts[vi as usize];
        match self.locate(hint, p) {
            Some((ti, false)) => {
                let [a, b, c] = self.tris[ti];
                self.replace_tri(ti, [a, b, vi]);
                let t1 = self.add_tri([b, c, vi]);
                let t2 = self.add_tri([c, a, vi]);
                t1.min(t2)
            }
            Some((ti, true)) => {
                let tri = self.tris[ti];
                if tri.iter().any(|&v| {
                    let q = self.verts[v as usize];
                    (q[0] - p[0]).abs() < 1e-12 && (q[1] - p[1]).abs() < 1e-12
                }) {
                    return usize::MAX; // duplicate vertex — skip
                }
                let edge = nearest_edge(self.verts.as_ref(), tri, p);
                if let Some((v1, v2)) = edge {
                    let key = (v1.min(v2), v1.max(v2));
                    if constraints.contains(&key) {
                        return usize::MAX; // on a constraint edge — skip
                    }
                    self.split_edge(v1, v2, vi);
                }
                ti
            }
            None => usize::MAX, // outside hull — skip
        }
    }

    /// Split edge (v1, v2) at existing vertex `p_idx` in ALL adjacent
    /// triangles (winding-preserving), updating adjacency.
    fn split_edge(&mut self, v1: u32, v2: u32, p_idx: u32) {
        let key = (v1.min(v2), v1.max(v2));
        let adjacent: Vec<usize> = match self.edge_map.get(&key) {
            Some(v) => v.clone(),
            None => return,
        };
        for ti in adjacent {
            let [a, b, c] = self.tris[ti];
            // Winding-preserving split, mirroring custom_cdt's
            // split_edge_with_vertex (see its comment for the ordering
            // derivation — emitting the wrong pair inverts the normal).
            let opposite = if a != v1 && a != v2 {
                a
            } else if b != v1 && b != v2 {
                b
            } else {
                c
            };
            let pos = |x: u32| -> usize {
                if x == a { 0 } else if x == b { 1 } else { 2 }
            };
            let (pv1, pv2, pop) = (pos(v1), pos(v2), pos(opposite));
            let v1_before_v2 = (pv1 + 3 - pop) % 3 < (pv2 + 3 - pop) % 3;
            let (t1, t2) = if v1_before_v2 {
                ([opposite, v1, p_idx], [opposite, p_idx, v2])
            } else {
                ([opposite, v2, p_idx], [opposite, p_idx, v1])
            };
            self.replace_tri(ti, t1);
            self.add_tri(t2);
        }
    }

    /// Locate `p`: visibility walk from `hint`.
    /// Returns `(triangle_idx, on_edge)`; `None` if outside the hull.
    fn locate(&self, hint: usize, p: [f64; 2]) -> Option<(usize, bool)> {
        // Bounded visibility walk.
        let mut t = hint.min(self.tris.len().saturating_sub(1));
        let max_steps = self.tris.len() + 8;
        for _ in 0..max_steps {
            let tri = self.tris[t];
            let d = [
                orient2d(self.verts[tri[0] as usize], self.verts[tri[1] as usize], p),
                orient2d(self.verts[tri[1] as usize], self.verts[tri[2] as usize], p),
                orient2d(self.verts[tri[2] as usize], self.verts[tri[0] as usize], p),
            ];
            let has_neg = d.iter().any(|&x| x < -EPS);
            let has_pos = d.iter().any(|&x| x > EPS);
            if !has_neg && !has_pos {
                return Some((t, true)); // on a vertex/edge
            }
            if !has_neg || !has_pos {
                let on_edge = d.iter().any(|&x| x.abs() <= EPS);
                return Some((t, on_edge)); // strictly inside or on edge
            }
            // p is outside one of the edges → walk across the FIRST such
            // edge that has a neighbor triangle.
            let mut moved = false;
            for i in 0..3 {
                if d[i] < -EPS {
                    let (a, b) = (tri[i], tri[(i + 1) % 3]);
                    if let Some(&next) = self
                        .neighbors_across(a, b, t)
                        .first()
                    {
                        t = next;
                        moved = true;
                        break;
                    }
                }
            }
            if !moved {
                return None; // hull boundary → outside
            }
        }
        // Walk failed (numeric degeneracy) → linear fallback.
        linear_locate(self, p)
    }
}

fn linear_locate(t: &Triangulation, p: [f64; 2]) -> Option<(usize, bool)> {
    for (i, tri) in t.tris.iter().enumerate() {
        let d = [
            orient2d(t.verts[tri[0] as usize], t.verts[tri[1] as usize], p),
            orient2d(t.verts[tri[1] as usize], t.verts[tri[2] as usize], p),
            orient2d(t.verts[tri[2] as usize], t.verts[tri[0] as usize], p),
        ];
        let has_neg = d.iter().any(|&x| x < -EPS);
        let has_pos = d.iter().any(|&x| x > EPS);
        if !has_neg && !has_pos {
            return Some((i, true));
        }
        if !(has_neg && has_pos) {
            let on_edge = d.iter().any(|&x| x.abs() <= EPS);
            return Some((i, on_edge));
        }
    }
    None
}

fn nearest_edge(
    verts: &[[f64; 2]],
    tri: [u32; 3],
    p: [f64; 2],
) -> Option<(u32, u32)> {
    let d1 = orient2d(p, verts[tri[0] as usize], verts[tri[1] as usize]).abs();
    let d2 = orient2d(p, verts[tri[1] as usize], verts[tri[2] as usize]).abs();
    let d3 = orient2d(p, verts[tri[2] as usize], verts[tri[0] as usize]).abs();
    Some(if d1 <= d2 && d1 <= d3 {
        (tri[0], tri[1])
    } else if d2 <= d3 {
        (tri[1], tri[2])
    } else {
        (tri[2], tri[0])
    })
}

/// Proper segment crossing: strictly interior intersection of (a,b) and
/// (c,d) — shared endpoints do NOT count.
fn segments_properly_cross(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> bool {
    let d1 = orient2d(c, d, a);
    let d2 = orient2d(c, d, b);
    let d3 = orient2d(a, b, c);
    let d4 = orient2d(a, b, d);
    ((d1 > EPS && d2 < -EPS) || (d1 < -EPS && d2 > EPS))
        && ((d3 > EPS && d4 < -EPS) || (d3 < -EPS && d4 > EPS))
}


// ============================================================
// Canonical build
// ============================================================

/// Build the canonical CDT for one shared NURBS surface.
///
/// `faces` = all faces referencing this surface (their loops collected
/// from the shared edge cache); `steiner_uv` = the surface-level
/// refinement grid (MS-2, `EdgeDiscretizationCache::get_nurbs_refinement_grid`).
///
/// Returns `None` when validation fails (never-worsen: callers fall back
/// to the legacy per-face path).
pub fn build_canonical_surface_cdt(
    nurbs: &NurbsSurface,
    faces: Vec<CanonicalFaceLoops>,
    steiner_uv: &[Point2d],
) -> Option<CanonicalSurfaceCdt> {
    if faces.is_empty() {
        return None;
    }

    // ── 1. Canonical vertex array (3D-bits dedup with UV-collision guard) ──
    let mut uv: Vec<[f64; 2]> = Vec::with_capacity(256);
    let mut p3d: Vec<Point3d> = Vec::with_capacity(256);
    // 3D position bits → canonical vertex indices at that position
    // (a Vec: periodic seams map one 3D point to several UV vertices).
    let mut by_pos: HashMap<[u64; 3], Vec<u32>> = HashMap::with_capacity(256);
    // UV bits → canonical vertex index (exact-UV dedup for Steiner).
    let mut by_uv: HashMap<(u64, u64), u32> = HashMap::with_capacity(256);

    let intern_rim = |p: &Point3d,
                          q: &Point2d,
                          uv: &mut Vec<[f64; 2]>,
                          p3d: &mut Vec<Point3d>,
                          by_pos: &mut HashMap<[u64; 3], Vec<u32>>,
                          by_uv: &mut HashMap<(u64, u64), u32>|
     -> u32 {
        let key = bits3(p);
        let quv = [q.u, q.v];
        // Reuse an existing vertex at the same 3D position whose UV is
        // numerically indistinguishable — bit-identical rims of faces
        // sharing an EDGE_CURVE collapse to one vertex here (the whole
        // point of the canonical array). A same-3D-different-UV point is
        // a seam/period duplicate and MUST stay separate.
        if let Some(ids) = by_pos.get(&key) {
            for &id in ids {
                if (uv[id as usize][0] - quv[0]).abs() < 1e-9
                    && (uv[id as usize][1] - quv[1]).abs() < 1e-9
                {
                    return id;
                }
            }
        }
        let id = uv.len() as u32;
        uv.push(quv);
        p3d.push(*p);
        by_pos.entry(key).or_default().push(id);
        by_uv.insert((quv[0].to_bits(), quv[1].to_bits()), id);
        id
    };

    // Rim constraints in insertion order: (face_idx, loop_idx, v_a, v_b).
    // loop_idx: 0 = outer, 1..=holes = hole loops.
    let mut constraints: Vec<(usize, usize, u32, u32)> = Vec::with_capacity(512);
    let mut face_loops_uv: Vec<(Vec<Vec<[f64; 2]>>)> = Vec::with_capacity(faces.len());

    for (fi, f) in faces.iter().enumerate() {
        if f.outer_3d.len() != f.outer_uv.len() || f.outer_3d.len() < 3 {
            // Malformed loops — the whole surface falls back to legacy.
            return None;
        }
        let mut loops: Vec<Vec<[f64; 2]>> = Vec::with_capacity(1 + f.holes_uv.len());
        let mut outer_ids: Vec<u32> = Vec::with_capacity(f.outer_3d.len());
        for (p, q) in f.outer_3d.iter().zip(f.outer_uv.iter()) {
            let id = intern_rim(p, q, &mut uv, &mut p3d, &mut by_pos, &mut by_uv);
            if outer_ids.last() != Some(&id) {
                outer_ids.push(id);
            }
        }
        // Drop the closing duplicate (first == last on closed loops).
        if outer_ids.len() > 1 && outer_ids[0] == *outer_ids.last().unwrap() {
            outer_ids.pop();
        }
        if outer_ids.len() < 3 {
            return None;
        }
        for w in 0..outer_ids.len() {
            constraints.push((fi, 0, outer_ids[w], outer_ids[(w + 1) % outer_ids.len()]));
        }
        loops.push(outer_ids.iter().map(|&id| uv[id as usize]).collect());

        for (hi, (h3d, huv)) in f.holes_3d.iter().zip(f.holes_uv.iter()).enumerate() {
            if h3d.len() != huv.len() || h3d.len() < 3 {
                continue; // degenerate hole — skip (matches legacy path)
            }
            let mut hole_ids: Vec<u32> = Vec::with_capacity(h3d.len());
            for (p, q) in h3d.iter().zip(huv.iter()) {
                let id = intern_rim(p, q, &mut uv, &mut p3d, &mut by_pos, &mut by_uv);
                if hole_ids.last() != Some(&id) {
                    hole_ids.push(id);
                }
            }
            if hole_ids.len() > 1 && hole_ids[0] == *hole_ids.last().unwrap() {
                hole_ids.pop();
            }
            if hole_ids.len() < 3 {
                continue;
            }
            for w in 0..hole_ids.len() {
                constraints.push((fi, hi + 1, hole_ids[w], hole_ids[(w + 1) % hole_ids.len()]));
            }
            loops.push(hole_ids.iter().map(|&id| uv[id as usize]).collect());
        }
        face_loops_uv.push(loops);
    }

    // ── 3. Seed triangulation over the CONVEX HULL of the rim vertices ──
    // Canonical order (mirroring the proven `custom_cdt` two-phase design):
    //   (a) hull fan (covers the hull; every face domain lies inside it),
    //   (b) insert interior rim vertices,
    //   (c) enforce rim constraints FLIP-ONLY (no new vertices — every rim
    //       edge is a cross-face contract: a split vertex on it would
    //       T-junction the neighbor's polyline),
    //   (d) insert Steiner points with constraint-edge protection.
    // NO super-quad frame: frame edges (corner→interior) were the blocked
    // configurations of the Sloan walk — the hull fan has none, and flips
    // on hull-fan edges are valid by construction (strictly convex quads).
    let steiner_uv_pts: Vec<Point2d> = steiner_uv.to_vec();
    let n_rim_vertices = uv.len();

    // Hull (strictly-convex, CCW) of the rim vertices.
    let hull_ids = convex_hull_ids(&uv);
    if hull_ids.len() < 3 {
        log::warn!("canonical CDT: rim hull degenerate — legacy path");
        return None;
    }

    // Seed: fan from hull_ids[0] — triangles (h0, hi, hi+1), CCW.
    let mut tri = Triangulation::new(uv.clone(), []);
    for w in 1..hull_ids.len() - 1 {
        tri.add_tri([hull_ids[0], hull_ids[w], hull_ids[w + 1]]);
    }

    // Insert the NON-HULL rim vertices (deterministic id order).
    let hull_set: std::collections::HashSet<u32> = hull_ids.iter().copied().collect();
    let mut hint = 0usize;
    for vi in 0..n_rim_vertices {
        if hull_set.contains(&(vi as u32)) {
            continue;
        }
        hint = tri.insert_vertex(vi as u32, hint);
    }

    // ── 4. Constraint enforcement — FLIP-ONLY (contract-safe) ──
    // A rim edge is a cross-face contract: the neighbor's mesh contains
    // exactly (v_i, v_{i+1}) as one edge. Splitting the constraint (or a
    // crossed edge at a NEW vertex) would introduce vertices the neighbor
    // does not have → T-junctions → boundary edges — the exact regression
    // this module exists to prevent. Flips mutate connectivity only; a
    // blocked constraint (no flippable crossing) fails the build → the
    // surface takes the legacy per-face path (never-worsen).
    for &(fi, loop_idx, a, b) in constraints.iter() {
        let _ = (fi, loop_idx);
        if a == b {
            continue;
        }
        if !tri.edge_exists(a, b) && !enforce_constraint(&mut tri, a, b) {
            log::warn!(
                "canonical CDT: constraint edge ({}, {}) could not be enforced \
                 flip-only — dropping canonical surface triangulation (legacy fallback)",
                a, b
            );
            return None;
        }
    }
    let constraint_edges: std::collections::HashSet<(u32, u32)> = constraints
        .iter()
        .filter_map(|&(_, _, a, b)| if a == b { None } else { Some((a.min(b), a.max(b))) })
        .collect();

    // ── 4.5 Insert Steiner points — CONSTRAINT-EDGE PROTECTED ──
    // Mirrors `custom_cdt::insert_interior_points`'s ring protection: a
    // Steiner landing on a constraint edge is redundant (the rim already
    // discretizes the surface there) and splitting the edge would break
    // the cross-face contract — skip it.
    for g in &steiner_uv_pts {
        let quv = [g.u, g.v];
        let pi = tri.verts.len() as u32;
        tri.verts.push(quv);
        // 3D position mirrors the legacy pipeline (deterministic rounding
        // of the surface evaluation).
        p3d.push(deterministic_round_point(nurbs.derivatives_at(quv[0], quv[1]).point));
        let hint_new = tri.insert_vertex_protected(pi, hint, &constraint_edges);
        if hint_new == usize::MAX {
            // Skipped (on a constraint edge / duplicate) — remove the
            // unused vertex so uv/p3d stay dense.
            tri.verts.pop();
            p3d.pop();
        } else {
            hint = hint_new;
        }
    }


    // ── 6. Validation: no edge with >2 adjacent triangles ──
    {
        let mut usage: HashMap<(u32, u32), usize> = HashMap::with_capacity(tri.tris.len() * 3);
        for t in &tri.tris {
            for i in 0..3 {
                let key = (t[i].min(t[(i + 1) % 3]), t[i].max(t[(i + 1) % 3]));
                *usage.entry(key).or_insert(0) += 1;
            }
        }
        if let Some(&(a, b)) = usage.iter().find(|(_, &n)| n > 2).map(|(k, _)| k) {
            log::warn!(
                "canonical CDT: edge ({}, {}) has >2 adjacent triangles — dropping \
                 canonical surface triangulation (legacy fallback)",
                a, b
            );
            return None;
        }
        if tri.tris.is_empty() {
            return None;
        }
    }

    // ── 7. Per-face extraction (centroid classification) ──
    let mut face_tris: Vec<Vec<usize>> = vec![Vec::new(); faces.len()];
    let mut taken: Vec<bool> = vec![false; tri.tris.len()];
    for (ti, t) in tri.tris.iter().enumerate() {
        let c = [
            (tri.verts[t[0] as usize][0] + tri.verts[t[1] as usize][0] + tri.verts[t[2] as usize][0])
                / 3.0,
            (tri.verts[t[0] as usize][1] + tri.verts[t[1] as usize][1] + tri.verts[t[2] as usize][1])
                / 3.0,
        ];
        for (fi, loops) in face_loops_uv.iter().enumerate() {
            if loops.is_empty() || taken[ti] {
                continue;
            }
            // Outer polygon must contain the centroid…
            if !point_in_polygon(c, &loops[0]) {
                continue;
            }
            // …and no hole may contain it.
            let in_hole = loops.iter().skip(1).any(|h| point_in_polygon(c, h));
            if in_hole {
                continue;
            }
            face_tris[fi].push(ti);
            taken[ti] = true;
        }
    }

    Some(CanonicalSurfaceCdt {
        uv: tri.verts,
        p3d,
        tris: tri.tris,
        face_tris,
        faces,
    })
}

/// Enforce constraint edge (a, b) in the triangulation — FLIP-ONLY.
///
/// Sloan-style visibility walk from `a` towards `b`: each properly
/// crossed edge whose quad is STRICTLY CONVEX and whose flip diagonal
/// does not re-cross the constraint is FLIPPED (connectivity-only
/// mutation); the walk restarts after every flip. The crossing count
/// strictly decreases per flip → termination. A constraint passing
/// through an existing vertex recurses on the two sub-segments (vertex
/// splits are contract-safe: both halves end at real vertices).
///
/// NO new vertices are ever created: rim constraint edges are
/// cross-face contracts (the neighboring face's mesh contains exactly
/// `(v_i, v_{i+1})`), and a split vertex on the edge would T-junction
/// the neighbor. A blocked constraint (no flippable crossing) returns
/// false → the canonical build fails → legacy per-face path.
fn enforce_constraint(tri: &mut Triangulation, a: u32, b: u32) -> bool {
    // Iteration cap: each flip strictly decreases the number of edges
    // properly crossing (a, b), so |tris| flips is a generous bound.
    let max_iterations = tri.tris.len().saturating_mul(2).saturating_add(16);
    for _iter in 0..max_iterations {
        if tri.edge_exists(a, b) {
            return true;
        }
        // Vertex-on-edge shortcuts (contract-safe: the split vertex is an
        // EXISTING canonical vertex, present in the neighbor's polyline).
        // Case 1: `b` lies ON an existing edge (a, c) — the constraint is
        // a sub-segment of a longer collinear edge (common after greedy
        // insertion skips intermediate rim vertices).
        if let Some(c) = edge_from_containing(tri, a, b) {
            tri.split_edge(a, c, b);
            continue;
        }
        // Case 2: `a` lies ON an existing edge (c, b).
        if let Some(c) = edge_from_containing(tri, b, a) {
            tri.split_edge(c, b, a);
            continue;
        }
        match walk_crossings(tri, a, b) {
            WalkOutcome::Done => {
                // Reached b without crossings — the edge either exists
                // (checked above) or the walk degenerated.
                return tri.edge_exists(a, b);
            }
            WalkOutcome::Crossed(edges) => {
                // Flip the FIRST flippable crossed edge (closest to a).
                let mut flipped = false;
                for &(u, v) in &edges {
                    if let Some((o1, o2)) = flip_is_valid(tri, u, v, a, b) {
                        flip_edge(tri, u, v, o1, o2);
                        flipped = true;
                        break;
                    }
                }
                if !flipped {
                    // Blocked by a non-convex configuration — Sloan would
                    // split here; we fail (never-worsen).
                    return false;
                }
                // re-walk
            }
            WalkOutcome::ThroughVertex(v) => {
                // The constraint passes through existing vertex v —
                // enforce the two sub-segments (contract-safe).
                return enforce_constraint(tri, a, v) && enforce_constraint(tri, v, b);
            }
            WalkOutcome::Valley(v) => {
                // The segment leaves a along an existing edge chain.
                return enforce_constraint(tri, a, v) && enforce_constraint(tri, v, b);
            }
            WalkOutcome::Failed => return false,
        }
    }
    false
}

enum WalkOutcome {
    /// Reached b; the edge now exists (or the walk found no crossings).
    Done,
    /// Properly crossed edges, in walk order from a to b.
    Crossed(Vec<(u32, u32)>),
    /// The segment passes through an existing vertex (index).
    ThroughVertex(u32),
    /// The segment leaves `a` along an existing edge to this vertex.
    Valley(u32),
    /// Walk degenerated irrecoverably.
    Failed,
}

/// Visibility walk from `a` towards `b`, collecting properly crossed
/// edges in order. Initial-triangle selection verifies the crossing is
/// FORWARD (the fan around `a` also contains triangles whose opposite
/// edges cross the LINE behind `a`).
fn walk_crossings(tri: &Triangulation, a: u32, b: u32) -> WalkOutcome {
    let pa = tri.verts[a as usize];
    let pb = tri.verts[b as usize];
    let dir = [pb[0] - pa[0], pb[1] - pa[1]];
    let dir2 = dir[0] * dir[0] + dir[1] * dir[1];

    // Initial triangle: incident to `a`, straddling the RAY a→b forward,
    // or containing b.
    let start = {
        let mut found = None;
        if let Some(ts) = tri.vert_tris.get(&a) {
            for &ti in ts {
                let t = tri.tris[ti];
                if t.contains(&b) {
                    found = Some(ti);
                    break;
                }
                let (x, y) = opposite_vertices(t, a);
                let ox = orient2d(pa, pb, tri.verts[x as usize]);
                let oy = orient2d(pa, pb, tri.verts[y as usize]);
                if (ox > EPS && oy < -EPS) || (ox < -EPS && oy > EPS) {
                    let s = ox / (ox - oy);
                    let cx = tri.verts[x as usize][0]
                        + s * (tri.verts[y as usize][0] - tri.verts[x as usize][0]);
                    let cy = tri.verts[x as usize][1]
                        + s * (tri.verts[y as usize][1] - tri.verts[x as usize][1]);
                    let t_fwd = (cx - pa[0]) * dir[0] + (cy - pa[1]) * dir[1];
                    if t_fwd > EPS * dir2.max(1.0) {
                        found = Some(ti);
                        break;
                    }
                }
            }
        }
        match found {
            Some(ti) => ti,
            None => match vertex_on_segment_from(tri, a, pa, pb) {
                Some(v) => return WalkOutcome::Valley(v),
                None => return WalkOutcome::Failed,
            },
        }
    };

    // Walk from a to b, COLLECTING every properly crossed edge. Reaching
    // a triangle containing b TERMINATES the walk — it is NOT success:
    // the collected crossings must still be flipped away before the
    // constraint edge (a, b) exists. (Returning Done at reach-time threw
    // the crossings away — the constraint silently stayed unenforced,
    // the exact L-shape regression this fixes.)
    let mut crossed: Vec<(u32, u32)> = Vec::new();
    let mut t = start;
    let mut reached_b = false;
    // Entry edge (the edge crossed to ENTER the current triangle): the
    // visibility walk must never exit through it — two triangles sharing
    // a constraint-crossing edge would otherwise oscillate forever
    // (twin-rims regression: [12,1,2] ⇄ [1,12,15] across edge (12,1)).
    let mut entry_edge: Option<(u32, u32)> = None;
    let max_steps = tri.tris.len() + 8;
    for _step in 0..max_steps {
        let tri_cur = tri.tris[t];
        if tri_cur.contains(&b) {
            reached_b = true;
            break;
        }
        // A vertex of this triangle lying ON the line a→b (not a/b):
        // split the constraint there (contract-safe, existing vertex).
        if let Some(v) = tri_cur.iter().copied().find(|&v| {
            v != a && v != b && point_on_line(pa, pb, tri.verts[v as usize])
        }) {
            return WalkOutcome::ThroughVertex(v);
        }
        // First properly crossed edge of this triangle (never the entry
        // edge; never edges incident to a in the start triangle).
        let mut crossed_edge: Option<(u32, u32)> = None;
        for i in 0..3 {
            let (p, q) = (tri_cur[i], tri_cur[(i + 1) % 3]);
            if t == start && (p == a || q == a) {
                continue; // edges incident to a cannot properly cross
            }
            if let Some((ep, eq)) = entry_edge {
                if (p == ep && q == eq) || (p == eq && q == ep) {
                    continue; // never exit through the entry edge
                }
            }
            if segments_properly_cross(pa, pb, tri.verts[p as usize], tri.verts[q as usize]) {
                crossed_edge = Some((p, q));
                break;
            }
        }
        if let Some((p, q)) = crossed_edge {
            crossed.push((p, q));
            match tri.neighbors_across(p, q, t).first() {
                Some(&next) => {
                    entry_edge = Some((p, q));
                    t = next;
                }
                None => return WalkOutcome::Failed, // hull boundary — b unreachable
            }
        } else {
            // No crossing in this triangle but b not reached: walk
            // across the edge facing b (not the entry edge).
            let mut moved = false;
            for i in 0..3 {
                let (p, q) = (tri_cur[i], tri_cur[(i + 1) % 3]);
                if let Some((ep, eq)) = entry_edge {
                    if (p == ep && q == eq) || (p == eq && q == ep) {
                        continue;
                    }
                }
                if orient2d(tri.verts[p as usize], tri.verts[q as usize], pb) < -EPS {
                    if let Some(&next) = tri.neighbors_across(p, q, t).first() {
                        entry_edge = Some((p, q));
                        t = next;
                        moved = true;
                        break;
                    }
                }
            }
            if !moved {
                return WalkOutcome::Failed;
            }
        }
    }
    if !reached_b {
        return WalkOutcome::Failed;
    }
    if crossed.is_empty() {
        // The segment crossed nothing: the edge must already exist
        // (checked by the caller) or the geometry is degenerate.
        return WalkOutcome::Done;
    }
    WalkOutcome::Crossed(crossed)
}

/// Flip validity for constraint insertion: edge (u, v) with opposite
/// vertices o1/o2 — valid iff the quad {u, o1, v, o2} is STRICTLY
/// convex AND the new diagonal (o1, o2) does not properly re-cross the
/// constraint (a, b) (monotone termination). Returns the (o1, o2) pair.
fn flip_is_valid(
    tri: &Triangulation,
    u: u32,
    v: u32,
    a: u32,
    b: u32,
) -> Option<(u32, u32)> {
    let key = (u.min(v), u.max(v));
    let adjacent = tri.edge_map.get(&key)?;
    if adjacent.len() != 2 {
        return None; // hull edge or degenerate fan — not flippable
    }
    let (t1, t2) = (adjacent[0], adjacent[1]);
    let tri1 = tri.tris[t1];
    let tri2 = tri.tris[t2];
    let opposite_of = |t: [u32; 3]| -> Option<u32> {
        t.iter().copied().find(|&x| x != u && x != v)
    };
    let o1 = opposite_of(tri1)?;
    let o2 = opposite_of(tri2)?;
    let pu = tri.verts[u as usize];
    let pv = tri.verts[v as usize];
    let p1 = tri.verts[o1 as usize];
    let p2 = tri.verts[o2 as usize];
    // Strictly convex quad = the four points are in convex position with
    // (u, v) and (o1, o2) as the two diagonals: o1/o2 are strictly on
    // opposite sides of line(u, v) (always true for two CCW triangles
    // sharing the edge), and u/v must be STRICTLY on opposite sides of
    // line(o1, o2). (The earlier corner-turn test assumed a fixed quad
    // cycle u→o1→v→o2 — wrong under CCW winding normalization, it
    // rejected perfectly valid flips.)
    let d_u = orient2d(p1, p2, pu);
    let d_v = orient2d(p1, p2, pv);
    if !((d_u > EPS && d_v < -EPS) || (d_u < -EPS && d_v > EPS)) {
        return None;
    }
    // The new diagonal must not properly re-cross the constraint.
    let pa = tri.verts[a as usize];
    let pb = tri.verts[b as usize];
    if segments_properly_cross(pa, pb, p1, p2) {
        return None;
    }
    Some((o1, o2))
}

/// Flip edge (u, v) to the diagonal (o1, o2) in both adjacent triangles.
/// Winding: replaces CCW triangles (u,v,o1)/(v,u,o2) with CCW triangles
/// (u,o1,o2)/(o1,v,o2) — verified by orientation normalization in
/// `replace_tri`/`add_tri`.
fn flip_edge(tri: &mut Triangulation, u: u32, v: u32, o1: u32, o2: u32) {
    let key = (u.min(v), u.max(v));
    let adjacent: Vec<usize> = match tri.edge_map.get(&key) {
        Some(list) => list.clone(),
        None => return,
    };
    if adjacent.len() != 2 {
        return;
    }
    let (t1, t2) = (adjacent[0], adjacent[1]);
    // Replace the FIRST triangle with (u, o1, o2) and the SECOND with
    // (o1, v, o2); add_tri/replace_tri normalize winding (CCW).
    let new1 = [u, o1, o2];
    let new2 = [o1, v, o2];
    tri.replace_tri(t1, new1);
    tri.replace_tri(t2, new2);
}

fn opposite_vertices(t: [u32; 3], v: u32) -> (u32, u32) {
    let others: Vec<u32> = t.iter().copied().filter(|&x| x != v).collect();
    (others[0], others[1])
}

/// Is point `p` (numerically) on the line through a→b?
fn point_on_line(a: [f64; 2], b: [f64; 2], p: [f64; 2]) -> bool {
    orient2d(a, b, p).abs() <= EPS * (b[0] - a[0]).hypot(b[1] - a[1]).max(1.0) * 10.0
}

/// Find a vertex connected to `a` by an existing edge and lying on the
/// segment a→b (valley case).
fn vertex_on_segment_from(tri: &Triangulation, a: u32, pa: [f64; 2], pb: [f64; 2]) -> Option<u32> {
    // Gather neighbors of a via its incident triangles.
    let mut seen: Vec<u32> = Vec::new();
    if let Some(ts) = tri.vert_tris.get(&a) {
        for &ti in ts {
            for &v in &tri.tris[ti] {
                if v != a && !seen.contains(&v) {
                    seen.push(v);
                }
            }
        }
    }
    let mut best: Option<(f64, u32)> = None;
    for v in seen {
        let pv = tri.verts[v as usize];
        if !point_on_line(pa, pb, pv) {
            continue;
        }
        // Must lie between a and b.
        let t = ((pv[0] - pa[0]) * (pb[0] - pa[0]) + (pv[1] - pa[1]) * (pb[1] - pa[1]))
            / ((pb[0] - pa[0]).powi(2) + (pb[1] - pa[1]).powi(2));
        if t > 1e-12 && t < 1.0 - 1e-12 {
            let d = (pv[0] - pa[0]).hypot(pv[1] - pa[1]);
            if best.map(|(bd, _)| d < bd).unwrap_or(true) {
                best = Some((d, v));
            }
        }
    }
    best.map(|(_, v)| v)
}
/// Find a neighbor `c` of `a` (connected by an existing edge) such that
/// vertex `b` lies ON the segment a→c (strictly between, or at c).
/// The caller splits edge (a, c) at the existing vertex `b`.
fn edge_from_containing(tri: &Triangulation, a: u32, b: u32) -> Option<u32> {
    let pa = tri.verts[a as usize];
    let pb = tri.verts[b as usize];
    let mut seen: Vec<u32> = Vec::new();
    if let Some(ts) = tri.vert_tris.get(&a) {
        for &ti in ts {
            for &v in &tri.tris[ti] {
                if v != a && v != b && !seen.contains(&v) {
                    seen.push(v);
                }
            }
        }
    }
    for c in seen {
        let pc = tri.verts[c as usize];
        // b collinear with (a, c)…
        if orient2d(pa, pc, pb).abs() > EPS * (pc[0] - pa[0]).hypot(pc[1] - pa[1]).max(1.0) * 10.0 {
            continue;
        }
        // …and inside the segment bounds (t in (0, 1]).
        let ab = pb[0] - pa[0];
        let acx = pc[0] - pa[0];
        let aby = pb[1] - pa[1];
        let acy = pc[1] - pa[1];
        let len2 = acx * acx + acy * acy;
        if len2 < 1e-24 {
            continue;
        }
        let tpar = (ab * acx + aby * acy) / len2;
        if tpar > 1e-9 && tpar <= 1.0 + 1e-9 {
            return Some(c);
        }
    }
    None
}

// ============================================================
// Tests
// ============================================================

/// Strictly-convex hull (CCW) of the canonical UV vertices, as vertex
/// indices. Andrew's monotone chain with strict turns (collinear points
/// dropped — the fan seed needs strict convexity).
fn convex_hull_ids(pts: &[[f64; 2]]) -> Vec<u32> {
    let n = pts.len();
    if n < 3 {
        return (0..n as u32).collect();
    }
    let mut idx: Vec<u32> = (0..n as u32).collect();
    idx.sort_by(|&a, &b| {
        let pa = pts[a as usize];
        let pb = pts[b as usize];
        pa[0].partial_cmp(&pb[0])
            .unwrap()
            .then(pa[1].partial_cmp(&pb[1]).unwrap())
    });

    let cross = |o: u32, a: u32, b: u32| -> f64 {
        let po = pts[o as usize];
        let pa = pts[a as usize];
        let pb = pts[b as usize];
        (pa[0] - po[0]) * (pb[1] - po[1]) - (pa[1] - po[1]) * (pb[0] - po[0])
    };

    let mut lower: Vec<u32> = Vec::new();
    for &id in &idx {
        while lower.len() >= 2
            && cross(lower[lower.len() - 2], lower[lower.len() - 1], id) <= 0.0
        {
            lower.pop();
        }
        lower.push(id);
    }
    let mut upper: Vec<u32> = Vec::new();
    for &id in idx.iter().rev() {
        while upper.len() >= 2
            && cross(upper[upper.len() - 2], upper[upper.len() - 1], id) <= 0.0
        {
            upper.pop();
        }
        upper.push(id);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

#[cfg(test)]
mod tests {
    use super::*;
    use draper_geometry::{NurbsSurface, Point2d, Point3d};

    /// Bilinear NURBS patch S(u, v) = (u, v, 0) over [u0,u1]×[v0,v1].
    fn bilinear_patch(u0: f64, u1: f64, v0: f64, v1: f64) -> NurbsSurface {
        NurbsSurface::from_v_rows(
            1,
            1,
            vec![
                vec![Point3d::new(u0, v0, 0.0), Point3d::new(u1, v0, 0.0)],
                vec![Point3d::new(u0, v1, 0.0), Point3d::new(u1, v1, 0.0)],
            ],
            vec![vec![1.0, 1.0], vec![1.0, 1.0]],
            vec![u0, u0, u1, u1],
            vec![v0, v0, v1, v1],
            false,
            false,
        )
    }

    /// Rectangular rim loop: ALL 4 corners + `n_mid` interior points per
    /// side (discretized rim, as the edge cache would produce).
    fn rect_loop(u0: f64, u1: f64, v0: f64, v1: f64, n_mid: usize) -> (Vec<Point3d>, Vec<Point2d>) {
        let mut p3d = Vec::new();
        let mut puv = Vec::new();
        let mut push = |u: f64, v: f64| {
            p3d.push(Point3d::new(u, v, 0.0));
            puv.push(Point2d::new(u, v));
        };
        push(u0, v0); // bottom-left corner
        // bottom: (u0,v0) → (u1,v0)
        for i in 1..=n_mid {
            push(u0 + (u1 - u0) * i as f64 / (n_mid + 1) as f64, v0);
        }
        push(u1, v0); // bottom-right corner
        // right: (u1,v0) → (u1,v1)
        for i in 1..=n_mid {
            push(u1, v0 + (v1 - v0) * i as f64 / (n_mid + 1) as f64);
        }
        push(u1, v1); // top-right corner
        // top: (u1,v1) → (u0,v1)
        for i in 1..=n_mid {
            push(u1 - (u1 - u0) * i as f64 / (n_mid + 1) as f64, v1);
        }
        push(u0, v1); // top-left corner
        // left: (u0,v1) → (u0,v0)
        for i in 1..=n_mid {
            push(u0, v1 - (v1 - v0) * i as f64 / (n_mid + 1) as f64);
        }
        (p3d, puv)
    }

    /// Merge face meshes by bit-identical 3D positions (the converter's
    /// merged-mesh semantics) and return the edge→usage map.
    fn merge_by_position(meshes: &[TriangleMesh]) -> (Vec<Point3d>, Vec<[u32; 3]>) {
        let mut verts: Vec<Point3d> = Vec::new();
        let mut index_of: HashMap<[u64; 3], u32> = HashMap::new();
        let mut tris: Vec<[u32; 3]> = Vec::new();
        for m in meshes {
            let mut remap = HashMap::new();
            for (vi, p) in m.vertices.iter().enumerate() {
                let key = [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
                if let Some(&g) = index_of.get(&key) {
                    remap.insert(vi as u32, g);
                } else {
                    let g = verts.len() as u32;
                    verts.push(*p);
                    index_of.insert(key, g);
                    remap.insert(vi as u32, g);
                }
            }
            for t in &m.triangles {
                tris.push([
                    *remap.get(&t[0]).unwrap(),
                    *remap.get(&t[1]).unwrap(),
                    *remap.get(&t[2]).unwrap(),
                ]);
            }
        }
        (verts, tris)
    }

    /// Edge usage in a merged mesh: (a, b) → adjacent triangle count.
    fn edge_usage(tris: &[[u32; 3]]) -> HashMap<(u32, u32), usize> {
        let mut usage: HashMap<(u32, u32), usize> = HashMap::new();
        for t in tris {
            for i in 0..3 {
                let a = t[i].min(t[(i + 1) % 3]);
                let b = t[i].max(t[(i + 1) % 3]);
                *usage.entry((a, b)).or_insert(0) += 1;
            }
        }
        usage
    }

    /// Two adjacent rectangular faces on ONE shared NURBS surface, with
    /// interior Steiner points on both sides of the shared rim: the
    /// canonical extraction must produce a merged mesh whose ONLY boundary
    /// edges lie on the OUTER rim of the union (the shared rim is interior
    /// by construction — the failure mode of the per-face CDT).
    #[test]
    fn canonical_cdt_two_faces_shared_surface_watertight() {
        let nurbs = bilinear_patch(0.0, 4.0, 0.0, 2.0);

        // Face A: [0,2]×[0,2]; Face B: [2,4]×[0,2] (shared rim at u=2).
        // Rims discretized identically on the shared edge (edge-cache
        // contract): the shared side has an intermediate point at v=1.
        let (a3d, auv) = rect_loop(0.0, 2.0, 0.0, 2.0, 1);
        let (b3d, buv) = rect_loop(2.0, 4.0, 0.0, 2.0, 1);

        // Steiner grid straddling the shared rim (surface-level grid).
        let steiner = vec![
            Point2d::new(1.0, 1.0),
            Point2d::new(3.0, 1.0),
            Point2d::new(1.5, 0.5),
            Point2d::new(2.5, 0.5),
        ];

        let faces = vec![
            CanonicalFaceLoops {
                step_face_id: 101,
                forward: true,
                outer_3d: a3d.clone(),
                outer_uv: auv.clone(),
                holes_3d: Vec::new(),
                holes_uv: Vec::new(),
            },
            CanonicalFaceLoops {
                step_face_id: 102,
                forward: true,
                outer_3d: b3d.clone(),
                outer_uv: buv.clone(),
                holes_3d: Vec::new(),
                holes_uv: Vec::new(),
            },
        ];

        let cdt = build_canonical_surface_cdt(&nurbs, faces, &steiner)
            .expect("canonical build must succeed on clean shared surface");
        assert_eq!(cdt.face_count(), 2);
        assert!(cdt.canonical_triangle_count() > 0, "canonical must have triangles");

        let mesh_a = cdt
            .extract_face_mesh(&nurbs, &a3d, &auv, &[], &[], true)
            .expect("face A must match");
        let mesh_b = cdt
            .extract_face_mesh(&nurbs, &b3d, &buv, &[], &[], true)
            .expect("face B must match");
        assert!(!mesh_a.triangles.is_empty());
        assert!(!mesh_b.triangles.is_empty());

        // Steiner fill: the interior grid points appear as mesh vertices.
        let has = |m: &TriangleMesh, x: f64, y: f64| {
            m.vertices.iter().any(|p| p.x == x && p.y == y && p.z == 0.0)
        };
        assert!(has(&mesh_a, 1.0, 1.0), "face A must contain interior Steiner (1,1)");
        assert!(has(&mesh_b, 3.0, 1.0), "face B must contain interior Steiner (3,1)");

        // Merge by position (converter semantics) and audit edges.
        let (verts, tris) = merge_by_position(&[mesh_a, mesh_b]);
        let usage = edge_usage(&tris);
        let boundary: Vec<(u32, u32)> = usage
            .iter()
            .filter(|(_, &n)| n == 1)
            .map(|(k, _)| *k)
            .collect();
        // Precise check: no boundary edge may lie on the shared rim u=2
        // (interior of the union). All boundary edges must lie on the OUTER
        // rim of the [0,4]×[0,2] rectangle.
        for &(a, b) in &boundary {
            let pa = verts[a as usize];
            let pb = verts[b as usize];
            let mx = (pa.x + pb.x) / 2.0;
            let my = (pa.y + pb.y) / 2.0;
            assert!(
                !((mx - 2.0).abs() < 1e-9 && my > 1e-9 && my < 2.0 - 1e-9),
                "boundary edge on the SHARED rim: ({:.3},{:.3})-({:.3},{:.3})",
                pa.x, pa.y, pb.x, pb.y
            );
        }
        // The union covers the full rectangle → total triangle area = 8.
        let area: f64 = tris
            .iter()
            .map(|t| {
                let a = verts[t[0] as usize];
                let b = verts[t[1] as usize];
                let c = verts[t[2] as usize];
                ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)).abs() / 2.0
            })
            .sum();
        assert!((area - 8.0).abs() < 1e-9, "union area must be 8.0 (got {:.4})", area);
    }

    /// Single face with a non-convex (L-shaped) rim + interior Steiner
    /// points: every rim edge must appear as a mesh edge in the extracted
    /// sub-triangulation (the constraint-edge contract) and the interior
    /// must be filled (triangle count well above the earcutr-only floor).
    #[test]
    fn canonical_cdt_l_shaped_face_rim_fidelity() {
        let nurbs = bilinear_patch(0.0, 4.0, 0.0, 4.0);

        // L-shape: (0,0) (4,0) (4,1) (1,1) (1,4) (0,4).
        let uv_pts = [
            [0.0, 0.0],
            [4.0, 0.0],
            [4.0, 1.0],
            [1.0, 1.0],
            [1.0, 4.0],
            [0.0, 4.0],
        ];
        let outer_3d: Vec<Point3d> = uv_pts.iter().map(|p| Point3d::new(p[0], p[1], 0.0)).collect();
        let outer_uv: Vec<Point2d> = uv_pts.iter().map(|p| Point2d::new(p[0], p[1])).collect();

        // Steiner points in the L interior, including near the reflex
        // corner (1,1) — where a greedy Delaunay would happily connect
        // across the notch unless constraints are enforced.
        let steiner = vec![
            Point2d::new(2.0, 0.5),
            Point2d::new(0.5, 2.0),
            Point2d::new(2.5, 0.5),
            Point2d::new(0.5, 2.5),
            Point2d::new(3.0, 0.5),
        ];

        let faces = vec![CanonicalFaceLoops {
            step_face_id: 201,
            forward: true,
            outer_3d: outer_3d.clone(),
            outer_uv: outer_uv.clone(),
            holes_3d: Vec::new(),
            holes_uv: Vec::new(),
        }];

        let cdt = build_canonical_surface_cdt(&nurbs, faces, &steiner)
            .expect("canonical build must succeed on L-shaped domain");
        let mesh = cdt
            .extract_face_mesh(&nurbs, &outer_3d, &outer_uv, &[], &[], true)
            .expect("face must match");

        // Rim fidelity: every consecutive rim pair is a mesh edge
        // (by 3D positions).
        let pos_index: HashMap<[u64; 3], u32> = mesh
            .vertices
            .iter()
            .enumerate()
            .map(|(i, p)| ([p.x.to_bits(), p.y.to_bits(), p.z.to_bits()], i as u32))
            .collect();
        let mesh_edges: std::collections::HashSet<(u32, u32)> = mesh
            .triangles
            .iter()
            .flat_map(|t| {
                [
                    (t[0].min(t[1]), t[0].max(t[1])),
                    (t[1].min(t[2]), t[1].max(t[2])),
                    (t[2].min(t[0]), t[2].max(t[0])),
                ]
            })
            .collect();
        let n = outer_3d.len();
        for i in 0..n {
            let j = (i + 1) % n;
            let ka = [outer_3d[i].x.to_bits(), outer_3d[i].y.to_bits(), outer_3d[i].z.to_bits()];
            let kb = [outer_3d[j].x.to_bits(), outer_3d[j].y.to_bits(), outer_3d[j].z.to_bits()];
            let (ia, ib) = (pos_index[&ka], pos_index[&kb]);
            let key = (ia.min(ib), ia.max(ib));
            assert!(
                mesh_edges.contains(&key),
                "rim edge {}→{} ({:?}→{:?}) missing from extracted mesh",
                i, j, uv_pts[i], uv_pts[j]
            );
        }

        // Interior fill: Steiner (2.0, 0.5) must appear as a vertex.
        assert!(
            mesh.vertices.iter().any(|p| p.x == 2.0 && p.y == 0.5),
            "interior Steiner point must be present"
        );
        // L-area sanity: the L (bottom strip [0,4]×[0,1] + left strip
        // [0,1]×[0,4]) has area 7 — neither the bounding box (16) nor
        // less than the L (holes).
        let area: f64 = mesh
            .triangles
            .iter()
            .map(|t| {
                let a = mesh.vertices[t[0] as usize];
                let b = mesh.vertices[t[1] as usize];
                let c = mesh.vertices[t[2] as usize];
                ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)).abs() / 2.0
            })
            .sum();
        assert!((area - 7.0).abs() < 1e-9, "union area must be 7.0 (got {:.4})", area);
    }

    /// Twinned rims (separate near-coincident EDGE_CURVEs — the dirty-file
    /// case): the canonical must still build, both faces extract, and each
    /// face's rim edges remain present in its own mesh (rim fidelity on
    /// both chains — never-worse vs legacy).
    #[test]
    fn canonical_cdt_twin_rims_never_worse() {
        let nurbs = bilinear_patch(0.0, 4.0, 0.0, 2.0);

        // Face A: [0,2]×[0,2]. Face B: rim offset by 1e-6 in u (twin of the
        // shared edge — distinct vertices, geometrically coincident).
        let (a3d, auv) = rect_loop(0.0, 2.0, 0.0, 2.0, 1);
        let (mut b3d, mut buv) = rect_loop(2.0, 4.0, 0.0, 2.0, 1);
        for p in b3d.iter_mut() {
            if (p.x - 2.0).abs() < 1e-12 {
                p.x = 2.0 + 1e-6;
            }
        }
        for p in buv.iter_mut() {
            if (p.u - 2.0).abs() < 1e-12 {
                p.u = 2.0 + 1e-6;
            }
        }

        let steiner = vec![Point2d::new(1.0, 1.0), Point2d::new(3.0, 1.0)];

        let faces = vec![
            CanonicalFaceLoops {
                step_face_id: 301,
                forward: true,
                outer_3d: a3d.clone(),
                outer_uv: auv.clone(),
                holes_3d: Vec::new(),
                holes_uv: Vec::new(),
            },
            CanonicalFaceLoops {
                step_face_id: 302,
                forward: true,
                outer_3d: b3d.clone(),
                outer_uv: buv.clone(),
                holes_3d: Vec::new(),
                holes_uv: Vec::new(),
            },
        ];

        // The two rims do not cross each other (offset strictly one-sided),
        // so the canonical build must succeed.
        let cdt = build_canonical_surface_cdt(&nurbs, faces, &steiner)
            .expect("canonical build must tolerate twin rims");
        let mesh_a = cdt
            .extract_face_mesh(&nurbs, &a3d, &auv, &[], &[], true)
            .expect("face A must match");
        let mesh_b = cdt
            .extract_face_mesh(&nurbs, &b3d, &buv, &[], &[], true)
            .expect("face B must match");
        assert!(!mesh_a.triangles.is_empty());
        assert!(!mesh_b.triangles.is_empty());

        // Rim fidelity for BOTH chains: every rim edge of each face is an
        // edge of that face's mesh.
        for (mesh, loop3d) in [(&mesh_a, &a3d), (&mesh_b, &b3d)] {
            let pos_index: HashMap<[u64; 3], u32> = mesh
                .vertices
                .iter()
                .enumerate()
                .map(|(i, p)| ([p.x.to_bits(), p.y.to_bits(), p.z.to_bits()], i as u32))
                .collect();
            let mesh_edges: std::collections::HashSet<(u32, u32)> = mesh
                .triangles
                .iter()
                .flat_map(|t| {
                    [
                        (t[0].min(t[1]), t[0].max(t[1])),
                        (t[1].min(t[2]), t[1].max(t[2])),
                        (t[2].min(t[0]), t[2].max(t[0])),
                    ]
                })
                .collect();
            let n = loop3d.len();
            for i in 0..n {
                let j = (i + 1) % n;
                let ka = [loop3d[i].x.to_bits(), loop3d[i].y.to_bits(), loop3d[i].z.to_bits()];
                let kb = [loop3d[j].x.to_bits(), loop3d[j].y.to_bits(), loop3d[j].z.to_bits()];
                if let (Some(&ia), Some(&ib)) = (pos_index.get(&ka), pos_index.get(&kb)) {
                    let key = (ia.min(ib), ia.max(ib));
                    assert!(
                        mesh_edges.contains(&key),
                        "twin rim edge {}→{} missing from its face mesh",
                        i, j
                    );
                }
            }
        }
    }

    /// Degenerate input guard: no faces → None; malformed loops → None
    /// (legacy fallback contract).
    #[test]
    fn canonical_cdt_degenerate_inputs_rejected() {
        let nurbs = bilinear_patch(0.0, 1.0, 0.0, 1.0);
        assert!(build_canonical_surface_cdt(&nurbs, Vec::new(), &[]).is_none());

        let bad = vec![CanonicalFaceLoops {
            step_face_id: 1,
            forward: true,
            outer_3d: vec![Point3d::new(0.0, 0.0, 0.0)],
            outer_uv: vec![Point2d::new(0.0, 0.0)],
            holes_3d: Vec::new(),
            holes_uv: Vec::new(),
        }];
        assert!(build_canonical_surface_cdt(&nurbs, bad, &[]).is_none());
    }
}

