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

/// A triangle whose UV vertices are collinear within this orient2d
/// magnitude covers zero area: its predicates are meaningless (every
/// point off the line reads as "strictly inside" — all three orient
/// signs collapse to one side). Session-35: iso-parametric boundary
/// chains (v=0 / v=1 lines) and sqrt-singular micro-sliver strips
/// (chains 1e-6 apart) produced such triangles; treating them as
/// containers made `insert_vertex` split them into OVERLAPPING
/// triangles — the source of the >2-adjacency validation failures.
const DEGENERATE_AREA_EPS: f64 = 1e-14;

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
    /// Session-37: built via the insertion-legalization retry. The
    /// extraction's session-37 acceptance criteria (zero-length rim
    /// segment skip, emitted-mesh manifold check) are calibrated for
    /// these builds and apply ONLY to them — plain builds keep the
    /// exact pre-session-37 extraction semantics (never-worsen:
    /// previously-working AND previously-locked faces behave
    /// identically).
    legalized: bool,
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
            let check_loop = |label: &str,
                              loop3d: &[Point3d],
                              mesh_edges: &std::collections::HashSet<(u32, u32)>|
             -> bool {
                let n = loop3d.len();
                for i in 0..n {
                    let j = (i + 1) % n;
                    let (Some(&ia), Some(&ib)) = (
                        pos_index.get(&bits3(&loop3d[i])),
                        pos_index.get(&bits3(&loop3d[j])),
                    ) else {
                        if debug_enabled() {
                            eprintln!(
                                "CANON-DEBUG: rim contract[{}] {}: vertex {:?} ({:?}) \
                                 MISSING from extraction",
                                label,
                                i,
                                bits3(&loop3d[i]).map(|b| b as i64),
                                loop3d[i]
                            );
                        }
                        return false; // rim vertex missing from the extraction
                    };
                    if ia == ib && self.legalized {
                        // Zero-length rim segment: consecutive duplicate
                        // 3D position in the caller's loop (closed-circle
                        // seam vertex / seam pinch). The build's intern
                        // stage collapsed it (consecutive-dedup) — no
                        // mesh edge can or should exist for it. Skipping
                        // aligns the extraction check with the build
                        // semantics (session-37: the 337 drill_top
                        // legalization-rescued groups all tripped on
                        // exactly this). LEGALIZED BUILDS ONLY — plain
                        // builds keep the strict pre-session-37 check
                        // (never-worsen).
                        continue;
                    }
                    if !mesh_edges.contains(&(ia.min(ib), ia.max(ib))) {
                        if debug_enabled() {
                            eprintln!(
                                "CANON-DEBUG: rim contract[{}] {}: edge ({}, {}) — 3d \
                                 ({:.6},{:.6},{:.6})-({:.6},{:.6},{:.6}) uv ({:.6e},{:.6e})-\
                                 ({:.6e},{:.6e}) NOT an extracted edge",
                                label,
                                i,
                                ia,
                                ib,
                                loop3d[i].x,
                                loop3d[i].y,
                                loop3d[i].z,
                                loop3d[j].x,
                                loop3d[j].y,
                                loop3d[j].z,
                                self.uv[ia as usize][0],
                                self.uv[ia as usize][1],
                                self.uv[ib as usize][0],
                                self.uv[ib as usize][1],
                            );
                        }
                        return false;
                    }
                }
                true
            };
            if !check_loop("outer", boundary_3d, &mesh_edges) {
                log::debug!(
                    "canonical CDT: rim contract violated for face — legacy fallback"
                );
                return None;
            }
            for h in holes_3d {
                if !check_loop("hole", h, &mesh_edges) {
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

        // ── Manifold check over the EMITTED mesh (session-37,
        //    LEGALIZED BUILDS ONLY) ──
        // Every edge that is NOT a segment of the face's rim polyline
        // (outer + holes, position-deduped, zero-length segments
        // skipped) must be 2-adjacent within the face's own mesh. Rim
        // edges are 1-adjacent here and matched by the neighboring
        // face's bit-identical rim in the merged BREP mesh. A PARTIALLY
        // emitted degenerate rim fan leaves its chord edges 1-adjacent,
        // and a filtered seam-pinch triangle leaves its side edges
        // 1-adjacent — both surface as boundary edges in the merged
        // mesh (the as1-oc-214 regression: faces unlocked by the
        // zero-length-rim-segment fix carried exactly such half fans).
        // Running the check POST position-dedup and degenerate filter
        // makes the accounting exact. Failing here routes the face to
        // the legacy path — never-worsen. Deterministic: the
        // lexicographically smallest offending edge.
        if self.legalized {
            let mut usage: HashMap<(u32, u32), usize> =
                HashMap::with_capacity(mesh.triangles.len() * 3);
            for t in &mesh.triangles {
                for i in 0..3 {
                    let a = t[i].min(t[(i + 1) % 3]);
                    let b = t[i].max(t[(i + 1) % 3]);
                    *usage.entry((a, b)).or_insert(0) += 1;
                }
            }
            let mut pos_id: HashMap<[u64; 3], u32> = HashMap::with_capacity(mesh.vertices.len());
            for (vi, p) in mesh.vertices.iter().enumerate() {
                pos_id.insert([p.x.to_bits(), p.y.to_bits(), p.z.to_bits()], vi as u32);
            }
            let mut rim_edges: std::collections::HashSet<(u32, u32)> =
                std::collections::HashSet::with_capacity(64);
            {
                let mut collect = |loop3d: &[Point3d]| {
                    let n = loop3d.len();
                    for i in 0..n {
                        let j = (i + 1) % n;
                        if let (Some(&ia), Some(&ib)) = (
                            pos_id.get(&bits3(&loop3d[i])),
                            pos_id.get(&bits3(&loop3d[j])),
                        ) {
                            if ia != ib {
                                rim_edges.insert((ia.min(ib), ia.max(ib)));
                            }
                        }
                    }
                };
                collect(boundary_3d);
                for h in holes_3d {
                    collect(h);
                }
            }
            let mut offending: Vec<(&(u32, u32), &usize)> = usage
                .iter()
                .filter(|(&(a, b), &n)| n != 2 && !rim_edges.contains(&(a, b)))
                .collect();
            offending.sort_unstable();
            if let Some((&(a, b), &n)) = offending.first() {
                log::debug!(
                    "canonical CDT: emitted face mesh non-manifold at edge ({}, {}) \
                     usage {} — legacy fallback",
                    a,
                    b,
                    n
                );
                if debug_enabled() {
                    eprintln!(
                        "CANON-DEBUG: manifold check FAILED edge ({}, {}) usage {} \
                         (rim edge: {})",
                        a,
                        b,
                        n,
                        rim_edges.contains(&(a, b))
                    );
                }
                return None;
            }
        }

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
// Build failure attribution + hostile-face screening (session-36)
// ============================================================

/// Why a canonical build failed, and — when attributable — which input
/// face (index into the `faces` argument) owns the failure.
///
/// The converter's group rescue (`build_canonical_surface_cdt_resilient`)
/// uses `failed_face` to drop the culprit face and retry the build with
/// the survivors, converting whole-group legacy fallbacks into per-face
/// ones (never-worsen: a dropped face takes exactly the legacy path it
/// takes today when the entire group fails).
#[derive(Clone, Debug)]
pub struct CanonicalBuildFailure {
    /// Index into the `faces` slice of the face whose rim geometry broke
    /// the build: the constraint that could not be enforced flip-only,
    /// or a face whose loops own the duplicated edge. `None` when the
    /// failure is not attributable to a single face (degenerate hull,
    /// empty triangulation, unowned edge).
    pub failed_face: Option<usize>,
    /// Short machine-readable cause: "no_faces", "malformed_loops",
    /// "degenerate_loop", "hull_degenerate", "constraint_unenforced",
    /// "edge_overused", "empty_triangulation".
    pub cause: &'static str,
}

/// UV-domain micro-sliver threshold: minimum-width ratio
/// (2·|area| / diameter²) below which a loop is considered hostile.
///
/// Session-35 measured the failing groups' strips at 1e-6..1e-4 width
/// against ~0.7-long chains (ratio ≈ 3e-6..3e-4); legitimate thin faces
/// sit orders of magnitude above. A rectangle with side ratio b/a reads
/// ≈ 2b/a, so `1e-3` flags strips thinner than ~1:2000.
pub const UV_SLIVER_RATIO: f64 = 1e-3;

/// Minimum-width ratio of a closed UV loop: `2·|signed area| / diameter²`.
///
/// Scale-free sliver metric: a rectangle a×b (a ≥ b) reads `2ab/(a²+b²)`
/// (≈ 2b/a for thin strips); an equilateral triangle reads ≈ 0.77.
/// Degenerate loops (fewer than 3 distinct points, zero diameter) read
/// as slivers (0.0).
pub fn uv_sliver_ratio(puv: &[Point2d]) -> f64 {
    let n = puv.len();
    if n < 3 {
        return 0.0;
    }
    // 2·signed area (shoelace).
    let mut area2 = 0.0f64;
    for i in 0..n {
        let a = &puv[i];
        let b = &puv[(i + 1) % n];
        area2 += a.u * b.v - b.u * a.v;
    }
    let area2 = area2.abs();
    // Squared diameter (exact pair scan — loops are at most a few
    // hundred points, and this only runs for failing groups).
    let mut d2 = 0.0f64;
    for i in 0..n {
        for j in (i + 1)..n {
            let du = puv[i].u - puv[j].u;
            let dv = puv[i].v - puv[j].v;
            let dd = du * du + dv * dv;
            if dd > d2 {
                d2 = dd;
            }
        }
    }
    if d2 <= 0.0 {
        return 0.0;
    }
    area2 / d2
}

/// Indices (into `faces`) of faces whose outer or hole UV loops are
/// micro-slivers — the session-35 root cause of the EdgeOverused
/// duplicate-triangle failures (sqrt-singular strips with UV chains
/// 1e-6..1e-4 apart).
pub fn hostile_face_indices(faces: &[CanonicalFaceLoops]) -> Vec<usize> {
    faces
        .iter()
        .enumerate()
        .filter(|(_, f)| {
            uv_sliver_ratio(&f.outer_uv) < UV_SLIVER_RATIO
                || f.holes_uv.iter().any(|h| uv_sliver_ratio(h) < UV_SLIVER_RATIO)
        })
        .map(|(i, _)| i)
        .collect()
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
        if trace_enabled() {
            self.trace_duplicate("add_tri", tri);
        }
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
        if trace_enabled() {
            self.trace_duplicate("replace_tri", new);
        }
        let old = self.tris[idx];
        self.deindex_tri(idx, old);
        self.tris[idx] = new;
        self.index_tri(idx, new);
    }

    /// Trace helper (DRAPER_CANON_TRACE=1): fire when a triangle with
    /// the same vertex set as `tri` already exists — the duplication
    /// mechanism behind the >2-adjacency failures (session-35).
    fn trace_duplicate(&self, label: &str, tri: [u32; 3]) {
        let key = (tri[0].min(tri[1]), tri[0].max(tri[1]));
        if let Some(list) = self.edge_map.get(&key) {
            for &ti in list {
                let t = self.tris[ti];
                let same_set = (t.contains(&tri[0]) && t.contains(&tri[1]) && t.contains(&tri[2]))
                    || (t[0] == t[1] || t[1] == t[2] || t[0] == t[2]);
                if same_set {
                    eprintln!(
                        "CANON-TRACE: {} DUPLICATE {:?} (existing tri[{}] = {:?})",
                        label, tri, ti, t
                    );
                    eprintln!("{}", std::backtrace::Backtrace::force_capture());
                    return;
                }
            }
        }
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
                // Prefer the edge that CONTAINS p (parametrically) —
                // nearest_edge can pick a wrong sub-segment on
                // degenerate/zero-area chains (session-35).
                let edge = self
                    .containing_edge(tri, p)
                    .or_else(|| nearest_edge(self.verts.as_ref(), tri, p));
                if let Some((v1, v2)) = edge {
                    // vi is brand-new: the duplicate guard cannot fire.
                    let _ = self.split_edge(v1, v2, vi);
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
                // Containing edge first (session-35); constraint check
                // applies to whichever edge is chosen.
                let edge = self
                    .containing_edge(tri, p)
                    .or_else(|| nearest_edge(self.verts.as_ref(), tri, p));
                if let Some((v1, v2)) = edge {
                    let key = (v1.min(v2), v1.max(v2));
                    if constraints.contains(&key) {
                        return usize::MAX; // on a constraint edge — skip
                    }
                    // vi is brand-new: the duplicate guard cannot fire.
                    let _ = self.split_edge(v1, v2, vi);
                }
                ti
            }
            None => usize::MAX, // outside hull — skip
        }
    }

    /// Split edge (v1, v2) at existing vertex `p_idx` in ALL adjacent
    /// triangles (winding-preserving), updating adjacency.
    ///
    /// Returns `false` WITHOUT modifying anything when edge (v1, p_idx)
    /// or (p_idx, v2) already exists — the split would re-create a
    /// triangle whose vertex set is already present (p_idx connected to
    /// an endpoint through a different path): the exact duplication
    /// mechanism behind the >2-adjacency failures (session-35 observed
    /// tri[295] == tri[320] on the micro-sliver chains).
    fn split_edge(&mut self, v1: u32, v2: u32, p_idx: u32) -> bool {
        let key = (v1.min(v2), v1.max(v2));
        if self.edge_exists(v1, p_idx) || self.edge_exists(p_idx, v2) {
            return false; // would duplicate an existing triangle
        }
        let mut adjacent: Vec<usize> = match self.edge_map.get(&key) {
            Some(v) => v.clone(),
            None => return false,
        };
        // Defense against duplicate listings: a combinatorially
        // degenerate triangle registers its repeated edge twice, and
        // processing the same index twice duplicates triangles on
        // every subsequent split (session-35 observed tri[295]==tri[320]).
        adjacent.sort_unstable();
        adjacent.dedup();
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
        true
    }

    /// Repair a SPANNING edge (v1, v2) that passes over vertex `p` (p
    /// lies ON the segment and is already connected to v1 or v2 through
    /// other paths — which is why `split_edge` refused). For each
    /// triangle [v1, v2, opp] on the spanning edge: when the half
    /// triangle {p, v2, opp} already exists, replace with [v1, p, opp]
    /// (the region is preserved — the dropped half is exactly the
    /// existing one); symmetrically, when {v1, p, opp} exists, replace
    /// with [p, v2, opp]. Removes the spanning edge and creates the
    /// missing connection through p (session-35).
    ///
    /// Returns false WITHOUT modifying anything when any triangle on
    /// the edge has no clean replacement (ambiguous or missing halves).
    fn repair_spanning_edge(&mut self, v1: u32, v2: u32, p: u32) -> bool {
        let key = (v1.min(v2), v1.max(v2));
        let adjacent: Vec<usize> = match self.edge_map.get(&key) {
            Some(v) => v.clone(),
            None => return false,
        };
        if adjacent.is_empty() || adjacent.len() > 2 {
            return false; // corrupted edge (>2) — refuse to touch
        }
        let opp_of = |t: [u32; 3]| -> u32 {
            if t[0] != v1 && t[0] != v2 {
                t[0]
            } else if t[1] != v1 && t[1] != v2 {
                t[1]
            } else {
                t[2]
            }
        };
        let has_half = |x: u32, y: u32, opp: u32| -> bool {
            let hk = (x.min(y), x.max(y));
            self.edge_map
                .get(&hk)
                .map(|l| l.iter().any(|&ti| self.tris[ti].contains(&opp)))
                .unwrap_or(false)
        };
        // Pre-check every triangle for a clean replacement.
        let mut plan: Vec<(usize, [u32; 3])> = Vec::with_capacity(adjacent.len());
        for &ti in &adjacent {
            let t = self.tris[ti];
            let opp = opp_of(t);
            let half_p_v2 = has_half(p, v2, opp);
            let half_v1_p = has_half(v1, p, opp);
            // The replacement triangle introduces edge (p, opp): the
            // matching half already contributes one triangle there — a
            // second pre-existing one would make three (>2).
            let third_count = self
                .edge_map
                .get(&(p.min(opp), p.max(opp)))
                .map(|l| l.len())
                .unwrap_or(0);
            if third_count > 1 {
                return false;
            }
            if half_p_v2 && !half_v1_p {
                plan.push((ti, [v1, p, opp]));
            } else if half_v1_p && !half_p_v2 {
                plan.push((ti, [p, v2, opp]));
            } else {
                return false; // ambiguous or missing halves — refuse
            }
        }
        for (ti, new) in plan {
            self.replace_tri(ti, new);
        }
        true
    }

    /// Lawson legalization after inserting vertex `vi` (session-37 —
    /// the drill_top collinear-chain cure): restore local Delaunayhood
    /// around the insertion by flipping strictly non-Delaunay edges
    /// outward, so the later flip-only constraint enforcement meets
    /// convex, well-shaped quads instead of the blocked non-convex
    /// configurations greedy insertion leaves behind on iso-parametric
    /// chains.
    ///
    /// GUARDS (never-worsen, see [`Self::delaunay_flip_target`]):
    /// strictly convex quad, strict incircle (cocircular ties stay),
    /// non-degenerate triangles, non-spanning new diagonal, interior
    /// edges only, deterministic stack order, flip cap (a partially
    /// legalized triangulation is still a VALID triangulation).
    ///
    /// Returns the number of flips performed.
    fn legalize_local(&mut self, vi: u32) -> usize {
        // Deterministic seeds: the edges OPPOSITE vi in its incident
        // triangles (sorted triangle order — vert_tris lists can carry
        // stale/duplicated indices across replace_tri churn).
        let mut stack: Vec<(u32, u32)> = Vec::new();
        if let Some(ts) = self.vert_tris.get(&vi) {
            let mut tris: Vec<usize> = ts.clone();
            tris.sort_unstable();
            tris.dedup();
            for &ti in &tris {
                let t = self.tris[ti];
                let (x, y) = opposite_vertices(t, vi);
                stack.push((x.min(y), x.max(y)));
            }
        }
        self.legalize_stack(stack)
    }

    /// Flip non-locally-Delaunay edges from a seed stack (Lawson). Each
    /// flip pushes the four outer edges of the flipped quad in a fixed
    /// order — the legalization front propagates outward; locally
    /// Delaunay edges pop as no-ops, so revisits are harmless.
    fn legalize_stack(&mut self, stack: Vec<(u32, u32)>) -> usize {
        // Generous cap: expected O(1) flips per insertion; the cap only
        // guards pathological tie storms (strict incircle makes A→B→A
        // ping-pong impossible, but near-tie cascades are bounded anyway).
        let max_flips = self.verts.len().saturating_mul(6).saturating_add(32);
        let mut flips = 0usize;
        let mut stack = stack;
        while let Some((u, v)) = stack.pop() {
            if flips >= max_flips {
                break;
            }
            if let Some((o1, o2)) = self.delaunay_flip_target(u, v) {
                flip_edge(self, u, v, o1, o2);
                flips += 1;
                // Outer edges of the flipped quad {u, o1, v, o2} minus
                // the removed diagonal (u, v) — fixed push order for
                // determinism (pop is LIFO).
                let outer = [
                    (u.min(o1), u.max(o1)),
                    (o1.min(v), o1.max(v)),
                    (v.min(o2), v.max(o2)),
                    (o2.min(u), o2.max(u)),
                ];
                stack.extend_from_slice(&outer);
            }
        }
        flips
    }

    /// Delaunay legalization flip check for internal edge (u, v):
    /// returns the new diagonal (o1, o2) when the edge is STRICTLY
    /// non-locally-Delaunay AND the flip is geometrically safe.
    ///
    /// Refused (None): hull edges (1 adjacent triangle) and corrupted
    /// edges (>2), degenerate triangles (undefined circumcircle),
    /// non-convex quads (the flip would produce overlapping triangles —
    /// the `custom_cdt::lawson_flip` lesson of 2026-09-09), cocircular
    /// ties (strict incircle — determinism), and diagonals that would
    /// SPAN an existing vertex (session-35: edges like (99,101) passing
    /// over vertex 100 are the exact configurations that later block
    /// constraint enforcement and inflate edge usage).
    fn delaunay_flip_target(&self, u: u32, v: u32) -> Option<(u32, u32)> {
        let key = (u.min(v), u.max(v));
        let adjacent = self.edge_map.get(&key)?;
        if adjacent.len() != 2 {
            return None; // hull edge or degenerate fan — not flippable
        }
        let (t1, t2) = (adjacent[0], adjacent[1]);
        let tri1 = self.tris[t1];
        let tri2 = self.tris[t2];
        let opposite_of = |t: [u32; 3]| -> Option<u32> {
            t.iter().copied().find(|&x| x != u && x != v)
        };
        let o1 = opposite_of(tri1)?;
        let o2 = opposite_of(tri2)?;
        let pu = self.verts[u as usize];
        let pv = self.verts[v as usize];
        let p1 = self.verts[o1 as usize];
        let p2 = self.verts[o2 as usize];
        // Non-degenerate triangles: a zero-area triangle's circumcircle
        // is undefined — the incircle predicate reads garbage.
        if orient2d(pu, pv, p1).abs() <= DEGENERATE_AREA_EPS
            || orient2d(pu, pv, p2).abs() <= DEGENERATE_AREA_EPS
        {
            return None;
        }
        // Strictly convex quad: u/v STRICTLY on opposite sides of the
        // line (o1, o2). (o1/o2 strictly on opposite sides of (u, v) is
        // inherent in two non-degenerate CCW triangles sharing the
        // edge.) A non-convex quad flip produces overlaps.
        let d_u = orient2d(p1, p2, pu);
        let d_v = orient2d(p1, p2, pv);
        if !((d_u > EPS && d_v < -EPS) || (d_u < -EPS && d_v > EPS)) {
            return None;
        }
        // Strictly non-locally-Delaunay: o2 strictly inside the
        // circumcircle of (u, v, o1), orientation- and scale-aware.
        if !incircle_strict(pu, pv, p1, p2) {
            return None;
        }
        // Spanning-guard on the new diagonal (shared with the constraint
        // `flip_is_valid` — one helper, one lesson).
        if diagonal_spans_vertex(self.verts.as_ref(), o1, o2, u, v) {
            return None;
        }
        Some((o1, o2))
    }

    /// A combinatorially (repeated vertex) or geometrically (collinear
    /// UVs) degenerate triangle. Covers zero area, contains nothing.
    fn tri_is_degenerate(&self, t: [u32; 3]) -> bool {
        if t[0] == t[1] || t[1] == t[2] || t[0] == t[2] {
            return true;
        }
        let (a, b, c) = (
            self.verts[t[0] as usize],
            self.verts[t[1] as usize],
            self.verts[t[2] as usize],
        );
        orient2d(a, b, c).abs() <= DEGENERATE_AREA_EPS
    }

    /// Debug statistics: (repeated-vertex triangles, collinear triangles).
    fn degenerate_stats(&self) -> (usize, usize) {
        let mut repeated = 0usize;
        let mut collinear = 0usize;
        for t in &self.tris {
            if t[0] == t[1] || t[1] == t[2] || t[0] == t[2] {
                repeated += 1;
            } else {
                let (a, b, c) = (
                    self.verts[t[0] as usize],
                    self.verts[t[1] as usize],
                    self.verts[t[2] as usize],
                );
                if orient2d(a, b, c).abs() <= DEGENERATE_AREA_EPS {
                    collinear += 1;
                }
            }
        }
        (repeated, collinear)
    }

    /// Edge of `tri` whose segment CONTAINS `p` (collinear within a
    /// scale-relative tolerance, parametric t in [-eps, 1+eps]).
    /// Unlike `nearest_edge` (pure distance), this never picks an edge
    /// the point is merely CLOSE to — on zero-area collinear chains the
    /// nearest-edge split landed on the wrong sub-segment and created
    /// triangles spanning other rim vertices (session-35).
    fn containing_edge(&self, tri: [u32; 3], p: [f64; 2]) -> Option<(u32, u32)> {
        for i in 0..3 {
            let (a, b) = (tri[i], tri[(i + 1) % 3]);
            let (pa, pb) = (self.verts[a as usize], self.verts[b as usize]);
            let ab = [pb[0] - pa[0], pb[1] - pa[1]];
            let len2 = ab[0] * ab[0] + ab[1] * ab[1];
            if len2 < 1e-24 {
                continue;
            }
            // |orient2d| = |ab| * dist(p, line) → collinear within
            // 1e-9 * |ab| iff |orient2d| <= 1e-9 * len2.
            if orient2d(pa, pb, p).abs() > 1e-9 * len2 {
                continue;
            }
            let t = ((p[0] - pa[0]) * ab[0] + (p[1] - pa[1]) * ab[1]) / len2;
            if (-1e-9..=1.0 + 1e-9).contains(&t) {
                return Some((a, b));
            }
        }
        None
    }

    /// Locate `p`: visibility walk from `hint`.
    /// Returns `(triangle_idx, on_edge)`; `None` if outside the hull.
    fn locate(&self, hint: usize, p: [f64; 2]) -> Option<(usize, bool)> {
        // Bounded visibility walk.
        let mut t = hint.min(self.tris.len().saturating_sub(1));
        let max_steps = self.tris.len() + 8;
        for _ in 0..max_steps {
            let tri = self.tris[t];
            if self.tri_is_degenerate(tri) {
                // Degenerate triangles are transparent: their orient
                // predicates collapse (all one sign) and would claim
                // any point on one side of their line as "inside".
                return linear_locate(self, p);
            }
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
        if t.tri_is_degenerate(*tri) {
            continue; // zero-area triangles are transparent
        }
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
/// Returns `Err(CanonicalBuildFailure)` when validation fails, carrying
/// the face that owns the failure when attributable — the group rescue
/// (`build_canonical_surface_cdt_resilient`) drops that face and retries
/// (never-worsen: dropped faces fall back to the legacy per-face path).
///
/// Single PLAIN attempt (bit-identical to the pre-session-37 build);
/// the Lawson insertion-legalization variant
/// ([`Triangulation::legalize_local`]) is driven by the group rescue,
/// AFTER its micro-sliver screening — legalizing around a hostile
/// sliver poisons the healthy neighbors' rim-contract extraction
/// (legalized Delaunay hugs the strip, the shared rim edges end up in
/// sliver-classified triangles). See `build_canonical_surface_cdt_resilient`.
pub fn build_canonical_surface_cdt_detailed(
    nurbs: &NurbsSurface,
    faces: Vec<CanonicalFaceLoops>,
    steiner_uv: &[Point2d],
) -> Result<CanonicalSurfaceCdt, CanonicalBuildFailure> {
    build_canonical_surface_cdt_inner(nurbs, &faces, steiner_uv, false)
}

/// The single-attempt build: the pre-session-37 algorithm, plus the
/// `legalize_insertions` variant (Lawson legalization after every rim
/// vertex insertion — driven by `build_canonical_surface_cdt_resilient`).
fn build_canonical_surface_cdt_inner(
    nurbs: &NurbsSurface,
    faces: &[CanonicalFaceLoops],
    steiner_uv: &[Point2d],
    legalize_insertions: bool,
) -> Result<CanonicalSurfaceCdt, CanonicalBuildFailure> {
    if faces.is_empty() {
        return Err(CanonicalBuildFailure {
            failed_face: None,
            cause: "no_faces",
        });
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
    // Interned vertex-id loops per face (diagnostics + pinch splitting).
    let mut face_loop_ids: Vec<Vec<Vec<u32>>> = Vec::with_capacity(faces.len());

    for (fi, f) in faces.iter().enumerate() {
        if f.outer_3d.len() != f.outer_uv.len() || f.outer_3d.len() < 3 {
            // Malformed loops — attributed so the group rescue can drop
            // just this face (the converter filters these upstream, this
            // is defense in depth).
            return Err(CanonicalBuildFailure {
                failed_face: Some(fi),
                cause: "malformed_loops",
            });
        }
        let mut loops: Vec<Vec<[f64; 2]>> = Vec::with_capacity(1 + f.holes_uv.len());
        let mut loop_ids: Vec<Vec<u32>> = Vec::with_capacity(1 + f.holes_uv.len());
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
            return Err(CanonicalBuildFailure {
                failed_face: Some(fi),
                cause: "degenerate_loop",
            });
        }
        for w in 0..outer_ids.len() {
            constraints.push((fi, 0, outer_ids[w], outer_ids[(w + 1) % outer_ids.len()]));
        }
        loops.push(outer_ids.iter().map(|&id| uv[id as usize]).collect());
        loop_ids.push(outer_ids.clone());

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
            loop_ids.push(hole_ids.clone());
        }
        face_loops_uv.push(loops);
        face_loop_ids.push(loop_ids);
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
        return Err(CanonicalBuildFailure {
            failed_face: None,
            cause: "hull_degenerate",
        });
    }

    // Seed: fan from hull_ids[0] — triangles (h0, hi, hi+1), CCW.
    let mut tri = Triangulation::new(uv.clone(), []);
    for w in 1..hull_ids.len() - 1 {
        tri.add_tri([hull_ids[0], hull_ids[w], hull_ids[w + 1]]);
    }

    // Insert the NON-HULL rim vertices (deterministic id order).
    // Session-37 variant: legalize after every insertion (Lawson flips,
    // guarded) — the plain path is kept bit-identical and only taken on
    // the retry attempt (see build_canonical_surface_cdt_detailed).
    let hull_set: std::collections::HashSet<u32> = hull_ids.iter().copied().collect();
    let mut hint = 0usize;
    let mut legalize_flips = 0usize;
    for vi in 0..n_rim_vertices {
        if hull_set.contains(&(vi as u32)) {
            continue;
        }
        hint = tri.insert_vertex(vi as u32, hint);
        if legalize_insertions {
            legalize_flips += tri.legalize_local(vi as u32);
        }
    }
    if legalize_insertions && debug_enabled() {
        eprintln!(
            "CANON-DEBUG: rim-insert legalization: {} flips ({} tris, {} verts)",
            legalize_flips,
            tri.tris.len(),
            tri.verts.len()
        );
    }
    if debug_enabled() {
        let (r, c) = tri.degenerate_stats();
        if r + c > 0 {
            eprintln!(
                "CANON-DEBUG: phase 'rim-insert' degenerates: repeated={} collinear={} ({} tris)",
                r,
                c,
                tri.tris.len()
            );
        }
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
        if a == b {
            continue;
        }
        if !tri.edge_exists(a, b) && !enforce_constraint(&mut tri, a, b) {
            log::warn!(
                "canonical CDT: constraint edge ({}, {}) of face {} loop {} could \
                 not be enforced flip-only — attributed failure (group rescue \
                 may drop the face)",
                a, b, fi, loop_idx
            );
            if debug_enabled() {
                dump_build_debug(
                    &tri, &p3d, &constraints, &face_loop_ids, &faces, a, b,
                    DebugCause::ConstraintUnenforced,
                );
            }
            if !legalize_insertions && std::env::var("DRAPER_CANON_DUMP_FACES").is_ok() {
                dump_face_literals(faces, steiner_uv, 0);
            }
            return Err(CanonicalBuildFailure {
                failed_face: Some(fi),
                cause: "constraint_unenforced",
            });
        }
    }
    let constraint_edges: std::collections::HashSet<(u32, u32)> = constraints
        .iter()
        .filter_map(|&(_, _, a, b)| if a == b { None } else { Some((a.min(b), a.max(b))) })
        .collect();
    if debug_enabled() {
        let (r, c) = tri.degenerate_stats();
        if r + c > 0 {
            eprintln!(
                "CANON-DEBUG: phase 'enforce' degenerates: repeated={} collinear={} ({} tris)",
                r,
                c,
                tri.tris.len()
            );
        }
    }

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
    if debug_enabled() {
        let (r, c) = tri.degenerate_stats();
        if r + c > 0 {
            eprintln!(
                "CANON-DEBUG: phase 'steiner' degenerates: repeated={} collinear={} ({} tris)",
                r,
                c,
                tri.tris.len()
            );
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
        // Deterministic pick: the lexicographically smallest overused
        // edge (HashMap iteration order is randomized — the group rescue
        // must drop faces deterministically).
        let mut overused: Vec<(u32, u32)> = usage
            .iter()
            .filter(|(_, &n)| n > 2)
            .map(|(&(a, b), _)| (a, b))
            .collect();
        overused.sort_unstable();
        if let Some(&(a, b)) = overused.first() {
            log::warn!(
                "canonical CDT: edge ({}, {}) has >2 adjacent triangles — attributed \
                 failure (group rescue may drop the owning face)",
                a, b
            );
            if debug_enabled() {
                dump_build_debug(
                    &tri, &p3d, &constraints, &face_loop_ids, &faces, a, b,
                    DebugCause::EdgeOverused,
                );
            }
            if !legalize_insertions && std::env::var("DRAPER_CANON_DUMP_FACES").is_ok() {
                dump_face_literals(faces, steiner_uv, 0);
            }
            // Attribute to a face whose loops own the duplicated edge:
            // prefer a loop containing BOTH endpoints, else either; the
            // lowest face index wins (deterministic). Unowned edges stay
            // unattributed (whole-group failure).
            let owner = face_loop_ids
                .iter()
                .enumerate()
                .find(|(_, loops)| {
                    loops.iter().any(|l| l.contains(&a) && l.contains(&b))
                })
                .map(|(fi, _)| fi)
                .or_else(|| {
                    face_loop_ids
                        .iter()
                        .enumerate()
                        .find(|(_, loops)| {
                            loops.iter().any(|l| l.contains(&a) || l.contains(&b))
                        })
                        .map(|(fi, _)| fi)
                });
            return Err(CanonicalBuildFailure {
                failed_face: owner,
                cause: "edge_overused",
            });
        }
        if tri.tris.is_empty() {
            return Err(CanonicalBuildFailure {
                failed_face: None,
                cause: "empty_triangulation",
            });
        }
    }

    // ── 7. Per-face extraction: centroid baseline + additive flood ──
    // Pass 1 is the pre-session-37 centroid classification VERBATIM —
    // including its emission of degenerate rim-fan triangles (whose
    // chord edges match the neighbors' meshes on files like
    // as1-oc-214), so currently-working groups stay bit-identical.
    //
    // Pass 2 (session-37, STRICTLY ADDITIVE) recovers the triangles
    // the centroid misclassifies on boundary-noisy domains: an
    // iso-parametric rim chain wiggling at 1e-15 makes a rim-adjacent
    // triangle's centroid land on the wrong side of the polygon, the
    // face then fails its rim contract and falls back to legacy
    // despite a successfully enforced build. The flood starts from the
    // face's assigned NON-degenerate triangles and claims only
    // UNASSIGNED NON-degenerate triangles across non-constraint edges:
    // it can never remove or steal (claimed triangles and rim
    // constraints are hard barriers, degenerate fans are never entered
    // — their only passable edges are chords to each other), so the
    // result is the baseline plus provably-interior stragglers.
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
    // Pass 2 (LEGALIZED BUILDS ONLY — never-worsen: plain builds keep
    // the exact pre-session-37 classification): connectivity recovery
    // of non-degenerate stragglers.
    let degenerate: Vec<bool> = tri
        .tris
        .iter()
        .map(|&t| tri.tri_is_degenerate(t))
        .collect();
    for fi in 0..faces.len() {
        if !legalize_insertions {
            break;
        }
        if face_loops_uv[fi].is_empty() {
            continue;
        }
        let mut queue: Vec<usize> = face_tris[fi]
            .iter()
            .copied()
            .filter(|&ti| !degenerate[ti])
            .collect();
        let mut added: Vec<usize> = Vec::new();
        while let Some(ti) = queue.pop() {
            let t = tri.tris[ti];
            for i in 0..3 {
                let a = t[i].min(t[(i + 1) % 3]);
                let b = t[i].max(t[(i + 1) % 3]);
                if constraint_edges.contains(&(a, b)) {
                    continue; // rim constraint — the domain boundary
                }
                if let Some(neighbors) = tri.edge_map.get(&(a, b)) {
                    for &nti in neighbors {
                        if !taken[nti] && !degenerate[nti] {
                            taken[nti] = true;
                            added.push(nti);
                            queue.push(nti);
                        }
                    }
                }
            }
        }
        if !added.is_empty() {
            face_tris[fi].extend(added);
            face_tris[fi].sort_unstable();
        }
    }

    // Session-37 diagnostic (DRAPER_CANON_DEBUG=1): compare the flood
    // classification against the pure-centroid one — any symmetric
    // difference on previously-working files is a red flag for
    // cross-face theft through barrier gaps (pinch vertices).
    if debug_enabled() {
        let mut centroid_tris: Vec<Vec<usize>> = vec![Vec::new(); faces.len()];
        let mut ctaken: Vec<bool> = vec![false; tri.tris.len()];
        for (ti, t) in tri.tris.iter().enumerate() {
            let c = [
                (tri.verts[t[0] as usize][0]
                    + tri.verts[t[1] as usize][0]
                    + tri.verts[t[2] as usize][0])
                    / 3.0,
                (tri.verts[t[0] as usize][1]
                    + tri.verts[t[1] as usize][1]
                    + tri.verts[t[2] as usize][1])
                    / 3.0,
            ];
            for (fi, loops) in face_loops_uv.iter().enumerate() {
                if loops.is_empty() || ctaken[ti] {
                    continue;
                }
                if !point_in_polygon(c, &loops[0]) {
                    continue;
                }
                if loops.iter().skip(1).any(|h| point_in_polygon(c, h)) {
                    continue;
                }
                centroid_tris[fi].push(ti);
                ctaken[ti] = true;
            }
        }
        for (fi, (flood, centroid)) in face_tris.iter().zip(centroid_tris.iter()).enumerate() {
            let stolen: Vec<usize> = flood
                .iter()
                .filter(|t| !centroid.contains(t))
                .copied()
                .collect();
            let lost: Vec<usize> = centroid
                .iter()
                .filter(|t| !flood.contains(t))
                .copied()
                .collect();
            if !stolen.is_empty() || !lost.is_empty() {
                eprintln!(
                    "CANON-DEBUG: face {} flood-vs-centroid: +{} stolen {:?} / -{} lost {:?}",
                    fi,
                    stolen.len(),
                    &stolen[..stolen.len().min(6)],
                    lost.len(),
                    &lost[..lost.len().min(6)],
                );
                for &ti in stolen.iter().take(4) {
                    let t = tri.tris[ti];
                    eprintln!(
                        "  stolen tri[{}] = {:?} uv ({:.5},{:.5}) ({:.5},{:.5}) ({:.5},{:.5})",
                        ti,
                        t,
                        tri.verts[t[0] as usize][0],
                        tri.verts[t[0] as usize][1],
                        tri.verts[t[1] as usize][0],
                        tri.verts[t[1] as usize][1],
                        tri.verts[t[2] as usize][0],
                        tri.verts[t[2] as usize][1],
                    );
                }
                for &ti in lost.iter().take(4) {
                    let t = tri.tris[ti];
                    eprintln!(
                        "  lost tri[{}] degen={} = {:?} uv ({:.5},{:.5}) ({:.5},{:.5}) ({:.5},{:.5})",
                        ti,
                        tri.tri_is_degenerate(t),
                        t,
                        tri.verts[t[0] as usize][0],
                        tri.verts[t[0] as usize][1],
                        tri.verts[t[1] as usize][0],
                        tri.verts[t[1] as usize][1],
                        tri.verts[t[2] as usize][0],
                        tri.verts[t[2] as usize][1],
                    );
                }
            }
        }
    }

    Ok(CanonicalSurfaceCdt {
        uv: tri.verts,
        p3d,
        tris: tri.tris,
        face_tris,
        faces: faces.to_vec(),
        legalized: legalize_insertions,
    })
}

/// Compatibility wrapper over [`build_canonical_surface_cdt_detailed`]
/// (pre-session-36 signature; existing callers and tests unchanged).
pub fn build_canonical_surface_cdt(
    nurbs: &NurbsSurface,
    faces: Vec<CanonicalFaceLoops>,
    steiner_uv: &[Point2d],
) -> Option<CanonicalSurfaceCdt> {
    build_canonical_surface_cdt_detailed(nurbs, faces, steiner_uv).ok()
}

/// Never-worsen group rescue (session-36, extended in session-37).
///
/// The FIRST attempt builds the FULL group — bit-identical to
/// [`build_canonical_surface_cdt`], so currently-succeeding groups are
/// unaffected. On the first failure a static micro-sliver screening
/// pass ([`hostile_face_indices`], the session-35 root cause) drops UV
/// micro-sliver faces and retries. Next (session-37), ONE
/// insertion-legalization attempt rebuilds the surviving group with
/// Lawson Delaunay flips ([`Triangulation::legalize_local`]) — the
/// drill_top collinear-chain cure; it runs AFTER screening because a
/// legalized build around a hostile sliver hugs the strip and pushes
/// the shared rim edges into sliver-classified triangles, breaking the
/// healthy neighbor's rim-contract extraction. On later failures the
/// attributed face (a rim constraint that cannot be enforced flip-only,
/// or a face owning the duplicated edge) is dropped — it takes the
/// legacy per-face path, exactly the path it takes today when the whole
/// group fails — and the build retries with the survivors.
///
/// Returns the built CDT (if any) plus the ORIGINAL indices of the
/// dropped faces (ascending), for caller-side logging.
pub fn build_canonical_surface_cdt_resilient(
    nurbs: &NurbsSurface,
    faces: Vec<CanonicalFaceLoops>,
    steiner_uv: &[Point2d],
) -> (Option<CanonicalSurfaceCdt>, Vec<usize>) {
    if faces.is_empty() {
        return (None, Vec::new());
    }
    // (original index, loops) — removals keep original ids for reporting.
    let mut live: Vec<(usize, CanonicalFaceLoops)> =
        faces.into_iter().enumerate().collect();
    let mut dropped: Vec<usize> = Vec::new();
    let mut screened = false;
    let mut legalized = false;
    let mut attempts = 0usize;

    loop {
        attempts += 1;
        let attempt_faces: Vec<CanonicalFaceLoops> =
            live.iter().map(|(_, f)| f.clone()).collect();
        match build_canonical_surface_cdt_inner(nurbs, &attempt_faces, steiner_uv, false) {
            Ok(cdt) => {
                dropped.sort_unstable();
                return (Some(cdt), dropped);
            }
            Err(failure) => {
                if attempts > 32 {
                    // Pathological ping-pong guard (each retry drops at
                    // least one face, so this is unreachable in practice).
                    dropped.sort_unstable();
                    return (None, dropped);
                }
                // 1) First failure → static micro-sliver screening pass
                //    (session-35 root cause). Hostile faces are dropped
                //    BEFORE attributed ones so a sliver cannot shadow the
                //    real culprit of an unrelated constraint failure.
                if !screened {
                    screened = true;
                    let live_faces: Vec<CanonicalFaceLoops> =
                        live.iter().map(|(_, f)| f.clone()).collect();
                    let hostile = hostile_face_indices(&live_faces);
                    if !hostile.is_empty() && hostile.len() < live.len() {
                        for hi in hostile.into_iter().rev() {
                            let (orig, _) = live.remove(hi);
                            dropped.push(orig);
                        }
                        log::info!(
                            "canonical CDT group rescue: screened {} micro-sliver \
                             face(s) — retrying with {} faces",
                            dropped.len(),
                            live.len()
                        );
                        continue;
                    }
                }
                // 2) ONE insertion-legalization attempt on the surviving
                //    group (session-37): the drill_top cure. Runs after
                //    screening (see doc comment) and before the
                //    attributed-drop loop; its failure does not consume
                //    the plain attempt's attribution.
                if !legalized {
                    legalized = true;
                    if failure.cause == "constraint_unenforced"
                        || failure.cause == "edge_overused"
                    {
                        match build_canonical_surface_cdt_inner(
                            nurbs,
                            &attempt_faces,
                            steiner_uv,
                            true,
                        ) {
                            Ok(cdt) => {
                                log::info!(
                                    "canonical CDT group rescue: insertion-legalization \
                                     rescued the group ({} faces, plain cause: {})",
                                    live.len(),
                                    failure.cause
                                );
                                if std::env::var("DRAPER_CANON_DUMP_RESCUED").is_ok() {
                                    dump_face_literals(&attempt_faces, steiner_uv, 1);
                                }
                                dropped.sort_unstable();
                                return (Some(cdt), dropped);
                            }
                            Err(_) => {} // fall through to attribution
                        }
                    }
                }
                // 3) Attributed face → drop it, retry. The failed build's
                //    face index refers to the CURRENT live set (screening
                //    retries rebuild before any attribution is consumed).
                if let Some(fi) = failure.failed_face {
                    if live.len() > 1 && fi < live.len() {
                        let (orig, _) = live.remove(fi);
                        log::info!(
                            "canonical CDT group rescue: dropped face {} ({}) — \
                             retrying with {} faces",
                            orig,
                            failure.cause,
                            live.len()
                        );
                        dropped.push(orig);
                        continue;
                    }
                    if live.len() <= 1 {
                        // The only surviving face IS the failure — nothing
                        // left to rescue.
                        dropped.sort_unstable();
                        return (None, dropped);
                    }
                    // Out-of-range attribution (defensive) → give up below.
                }
                // 4) Nothing left to try — whole-group legacy (today's
                //    behavior).
                dropped.sort_unstable();
                return (None, dropped);
            }
        }
    }
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
        // insertion skips intermediate rim vertices). DUPLICATE-GUARDED
        // (session-35): when b is already connected to a or c through
        // another path, the split would re-create an existing triangle —
        // refuse and let the visibility walk handle the constraint.
        if let Some(c) = edge_from_containing(tri, a, b) {
            if tri.split_edge(a, c, b) {
                continue;
            }
            // Blocked by the duplicate guard: b already connected to a
            // or c. The edge (a, c) SPANS b — repair it by re-routing
            // through b (region-preserving triangle replacement).
            if tri.repair_spanning_edge(a, c, b) {
                continue;
            }
        }
        // Case 2: `a` lies ON an existing edge (c, b).
        if let Some(c) = edge_from_containing(tri, b, a) {
            if tri.split_edge(c, b, a) {
                continue;
            }
            if tri.repair_spanning_edge(c, b, a) {
                continue;
            }
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
            WalkOutcome::BlockedOnEdge(p, q) => {
                // b lies ON edge (p, q): split it at b — guarded against
                // duplication (b already connected to p or q through
                // another path → refuse, the constraint is blocked).
                if tri.split_edge(p, q, b) {
                    continue; // edge (a, b) may exist now — re-check
                }
                return false;
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
    /// `b` lies ON an edge (p, q) of a walk triangle: the constraint
    /// is a sub-segment of that edge — the walk can neither properly
    /// cross it nor step past it. The caller splits (p, q) at b
    /// (duplicate-guarded).
    BlockedOnEdge(u32, u32),
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
                None => {
                    if trace_enabled() {
                        eprintln!(
                            "CANON-TRACE: initial-selection FAILED a={} b={} uv a=({:.6},{:.6}) b=({:.6},{:.6})",
                            a, b, pa[0], pa[1], pb[0], pb[1]
                        );
                        if let Some(ts) = tri.vert_tris.get(&a) {
                            eprintln!("  {} incident tris:", ts.len());
                            for &ti in ts.iter().take(14) {
                                let t = tri.tris[ti];
                                eprintln!(
                                    "    tri[{}] = {:?} uv ({:.5},{:.5}) ({:.5},{:.5}) ({:.5},{:.5})",
                                    ti, t,
                                    tri.verts[t[0] as usize][0], tri.verts[t[0] as usize][1],
                                    tri.verts[t[1] as usize][0], tri.verts[t[1] as usize][1],
                                    tri.verts[t[2] as usize][0], tri.verts[t[2] as usize][1],
                                );
                            }
                        } else {
                            eprintln!("  a has NO incident triangles (orphan)!");
                        }
                    }
                    return WalkOutcome::Failed;
                }
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
        // `b` lying ON an edge of this triangle (not as its endpoint):
        // the constraint is a sub-segment of that edge — no proper
        // crossing is possible and the facing-side step cannot move.
        // Split the edge at b (caller-side, duplicate-guarded).
        for i in 0..3 {
            let (p, q) = (tri_cur[i], tri_cur[(i + 1) % 3]);
            if p == b || q == b {
                continue;
            }
            let (pp, pq2) = (tri.verts[p as usize], tri.verts[q as usize]);
            let eq = [pq2[0] - pp[0], pq2[1] - pp[1]];
            let len2 = eq[0] * eq[0] + eq[1] * eq[1];
            if len2 < 1e-24 {
                continue;
            }
            if orient2d(pp, pq2, pb).abs() > 1e-9 * len2 {
                continue; // not collinear with the edge
            }
            let t = ((pb[0] - pp[0]) * eq[0] + (pb[1] - pp[1]) * eq[1]) / len2;
            if t > 1e-9 && t < 1.0 - 1e-9 {
                return WalkOutcome::BlockedOnEdge(p, q);
            }
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

/// Strict in-circumcircle test: is `p` STRICTLY inside the circumcircle
/// of (a, b, c)? Orientation-aware (the triangle may be wound either
/// way) and scale-aware (the incircle determinant scales as length^4 —
/// a flat epsilon would misjudge large UV domains). Cocircular ties are
/// NOT inside: flips only fire on unambiguous violations, which keeps
/// legalization deterministic under floating-point noise.
fn incircle_strict(a: [f64; 2], b: [f64; 2], c: [f64; 2], p: [f64; 2]) -> bool {
    let ax = a[0] - p[0];
    let ay = a[1] - p[1];
    let bx = b[0] - p[0];
    let by = b[1] - p[1];
    let cx = c[0] - p[0];
    let cy = c[1] - p[1];
    let det = (ax * ax + ay * ay) * (bx * cy - cx * by)
        - (bx * bx + by * by) * (ax * cy - cx * ay)
        + (cx * cx + cy * cy) * (ax * by - bx * ay);
    let scale = a[0]
        .abs()
        .max(a[1].abs())
        .max(b[0].abs())
        .max(b[1].abs())
        .max(c[0].abs())
        .max(c[1].abs())
        .max(p[0].abs())
        .max(p[1].abs())
        .max(1.0);
    let eps = 1e-12 * scale * scale * scale * scale;
    let orient = orient2d(a, b, c);
    if orient > 0.0 {
        det > eps
    } else {
        det < -eps
    }
}

/// True when segment (o1, o2) passes STRICTLY over some other vertex
/// (bbox-pruned full scan; vertices are a few hundred per surface).
/// The flip-spanning lesson of session-35: a flipped-in diagonal that
/// passes over an existing vertex keeps that vertex's incident
/// triangles, overlapping the flipped pair — the exact configuration
/// that later blocks constraint enforcement and inflates edge usage.
fn diagonal_spans_vertex(verts: &[[f64; 2]], o1: u32, o2: u32, u: u32, v: u32) -> bool {
    let p1 = verts[o1 as usize];
    let p2 = verts[o2 as usize];
    let (minx, maxx) = (p1[0].min(p2[0]), p1[0].max(p2[0]));
    let (miny, maxy) = (p1[1].min(p2[1]), p1[1].max(p2[1]));
    for (wi, w) in verts.iter().enumerate() {
        let wi = wi as u32;
        if wi == o1 || wi == o2 || wi == u || wi == v {
            continue;
        }
        if w[0] < minx - 1e-12
            || w[0] > maxx + 1e-12
            || w[1] < miny - 1e-12
            || w[1] > maxy + 1e-12
        {
            continue;
        }
        if point_on_line(p1, p2, *w) {
            // strictly between o1 and o2?
            let t = ((w[0] - p1[0]) * (p2[0] - p1[0]) + (w[1] - p1[1]) * (p2[1] - p1[1]))
                / ((p2[0] - p1[0]).powi(2) + (p2[1] - p1[1]).powi(2));
            if t > 1e-9 && t < 1.0 - 1e-9 {
                return true;
            }
        }
    }
    false
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
    // The new diagonal must not SPAN an existing vertex (session-35) —
    // shared guard with the Delaunay legalization path.
    if diagonal_spans_vertex(tri.verts.as_ref(), o1, o2, u, v) {
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
    // NEAREST candidate (session-35): the closest c minimizes the
    // spanning window — the split edge (a, c) is as short as the
    // neighborhood allows, so fewer intermediate vertices can be
    // jumped over.
    let mut best: Option<(f64, u32)> = None;
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
            if best.map(|(bd, _)| len2 < bd).unwrap_or(true) {
                best = Some((len2, c));
            }
        }
    }
    best.map(|(_, c)| c)
}

// ============================================================
// Build diagnostics (env-gated: DRAPER_CANON_DEBUG=1)
// ============================================================

fn debug_enabled() -> bool {
    // Cached via std::env (cheap enough per build; builds are per-surface).
    std::env::var("DRAPER_CANON_DEBUG").is_ok()
}

/// Fine-grained per-triangle tracing (DRAPER_CANON_TRACE=1): duplicate
/// and degenerate triangle creation with backtraces. OnceLock-cached —
/// checked on EVERY add_tri/replace_tri call.
fn trace_enabled() -> bool {
    static TRACE: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *TRACE.get_or_init(|| std::env::var("DRAPER_CANON_TRACE").is_ok())
}

#[derive(Clone, Copy, Debug)]
enum DebugCause {
    ConstraintUnenforced,
    EdgeOverused,
}

/// Dump the failing canonical build state to stderr: the offending edge
/// (UV + 3D), its adjacent triangles, repeated-vertex triangles, the
/// constraints touching either endpoint, and a per-face pinch report
/// (non-consecutive revisits of an interned rim vertex).
#[allow(clippy::too_many_arguments)]
fn dump_build_debug(
    tri: &Triangulation,
    p3d: &[Point3d],
    constraints: &[(usize, usize, u32, u32)],
    face_loop_ids: &[Vec<Vec<u32>>],
    faces: &[CanonicalFaceLoops],
    a: u32,
    b: u32,
    cause: DebugCause,
) {
    let key = (a.min(b), a.max(b));
    eprintln!("CANON-DEBUG: cause={:?} edge=({},{})", cause, key.0, key.1);
    eprintln!(
        "  uv a=({:.6},{:.6}) b=({:.6},{:.6})  3d a=({:.4},{:.4},{:.4}) b=({:.4},{:.4},{:.4})",
        tri.verts[a as usize][0],
        tri.verts[a as usize][1],
        tri.verts[b as usize][0],
        tri.verts[b as usize][1],
        p3d[a as usize].x,
        p3d[a as usize].y,
        p3d[a as usize].z,
        p3d[b as usize].x,
        p3d[b as usize].y,
        p3d[b as usize].z,
    );
    if let Some(ts) = tri.edge_map.get(&key) {
        eprintln!("  edge_map[({},{})] = {} tris {:?}", key.0, key.1, ts.len(), ts);
        for &ti in ts.iter().take(6) {
            let t = tri.tris[ti];
            eprintln!(
                "    tri[{}] = {:?}  uv ({:.5},{:.5}) ({:.5},{:.5}) ({:.5},{:.5})",
                ti,
                t,
                tri.verts[t[0] as usize][0],
                tri.verts[t[0] as usize][1],
                tri.verts[t[1] as usize][0],
                tri.verts[t[1] as usize][1],
                tri.verts[t[2] as usize][0],
                tri.verts[t[2] as usize][1],
            );
        }
    } else {
        eprintln!("  edge_map[({},{})] = ABSENT", key.0, key.1);
    }
    // Repeated-vertex triangles (degenerate combinatorics inflate usage).
    let mut degenerate = 0usize;
    for (ti, t) in tri.tris.iter().enumerate() {
        if t[0] == t[1] || t[1] == t[2] || t[0] == t[2] {
            if degenerate < 6 {
                eprintln!(
                    "  REPEATED-VERTEX tri[{}] = {:?} uv ({:.5},{:.5}) ({:.5},{:.5})",
                    ti,
                    t,
                    tri.verts[t[0] as usize][0],
                    tri.verts[t[0] as usize][1],
                    tri.verts[t[1] as usize][0],
                    tri.verts[t[1] as usize][1],
                );
            }
            degenerate += 1;
        }
    }
    if degenerate > 0 {
        eprintln!("  repeated-vertex triangles: {}", degenerate);
    }
    // Constraints touching either endpoint.
    for &(fi, li, ca, cb) in constraints.iter() {
        if ca == a || cb == a || ca == b || cb == b {
            eprintln!("  constraint face#{} loop{} = ({},{})", fi, li, ca, cb);
        }
    }
    // Pinch report: per face+loop, non-consecutive revisits.
    for (fi, loops) in face_loop_ids.iter().enumerate() {
        for (li, ids) in loops.iter().enumerate() {
            for i in 0..ids.len() {
                for j in (i + 2)..ids.len() {
                    if ids[i] == ids[j] && !(i == 0 && j == ids.len() - 1) {
                        eprintln!(
                            "  PINCH face#{} loop{} revisits vertex {} at {} and {} (loop len {})",
                            fi,
                            li,
                            ids[i],
                            i,
                            j,
                            ids.len()
                        );
                    }
                }
            }
        }
    }
    let _ = faces;
}

/// DRAPER_CANON_DUMP_FACES=1 / DRAPER_CANON_DUMP_RESCUED=1: dump the
/// group's face loops (3D + UV) and Steiner points as Rust literals —
/// for building regression tests from real production data
/// (session-37 drill_top capture). DUMP_FACES fires on the PLAIN
/// attempt's connectivity failures; DUMP_RESCUED fires on groups the
/// legalization retry actually rescued. Capped at the first 3 groups
/// each to keep log files sane.
fn dump_face_literals(faces: &[CanonicalFaceLoops], steiner_uv: &[Point2d], slot: usize) {
    use std::sync::atomic::{AtomicUsize, Ordering};
    static DUMPS: [AtomicUsize; 2] = [AtomicUsize::new(0), AtomicUsize::new(0)];
    if DUMPS[slot].fetch_add(1, Ordering::SeqCst) >= 3 {
        return;
    }
    eprintln!(
        "CANON-DUMP-FACES: BEGIN group ({} faces, {} steiner)",
        faces.len(),
        steiner_uv.len()
    );
    eprintln!("    let faces = vec![");
    for f in faces {
        eprintln!("        CanonicalFaceLoops {{");
        eprintln!("            step_face_id: {},", f.step_face_id);
        eprintln!("            forward: {},", f.forward);
        eprint!("            outer_3d: vec![");
        for p in &f.outer_3d {
            eprint!("Point3d::new({:?}, {:?}, {:?}), ", p.x, p.y, p.z);
        }
        eprintln!("],");
        eprint!("            outer_uv: vec![");
        for p in &f.outer_uv {
            eprint!("Point2d::new({:?}, {:?}), ", p.u, p.v);
        }
        eprintln!("],");
        eprint!("            holes_3d: vec![");
        for h in &f.holes_3d {
            eprint!("vec![");
            for p in h {
                eprint!("Point3d::new({:?}, {:?}, {:?}), ", p.x, p.y, p.z);
            }
            eprint!("], ");
        }
        eprintln!("],");
        eprint!("            holes_uv: vec![");
        for h in &f.holes_uv {
            eprint!("vec![");
            for p in h {
                eprint!("Point2d::new({:?}, {:?}), ", p.u, p.v);
            }
            eprint!("], ");
        }
        eprintln!("],");
        eprintln!("        }},");
    }
    eprintln!("    ];");
    eprint!("    let steiner = vec![");
    for p in steiner_uv {
        eprint!("Point2d::new({:?}, {:?}), ", p.u, p.v);
    }
    eprintln!("];");
    eprintln!("CANON-DUMP-FACES: END group");
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

    /// Session-35 regression: `split_edge` must REFUSE to split an edge
    /// at a vertex already connected to one of its endpoints — the
    /// unguarded split re-created existing triangles (tri[295]==tri[320]
    /// on the HOUSING micro-sliver chains), inflating edge usage to 4
    /// and failing the whole canonical group with >2 adjacency.
    #[test]
    fn canonical_cdt_split_edge_duplicate_guard() {
        // Square 0(0,0) 1(2,0) 2(2,2) 3(0,2); vertices 5=(0.5,0) and
        // 6=(1.5,0) on the bottom edge (0,1).
        let verts = vec![
            [0.0, 0.0],
            [2.0, 0.0],
            [2.0, 2.0],
            [0.0, 2.0],
            [1.0, 1.0],
            [0.5, 0.0],
            [1.5, 0.0],
        ];
        let mut t = Triangulation::new(verts, [[0, 1, 2], [0, 2, 3]]);
        // Split (0,1) at 6, then (0,6) at 5: both fresh — must succeed.
        assert!(t.split_edge(0, 1, 6));
        assert!(t.split_edge(0, 6, 5));
        assert!(t.edge_exists(0, 5) && t.edge_exists(5, 6) && t.edge_exists(6, 1));
        let tris_before = t.tris.len();
        // Re-splitting (0,6) at 5 would re-create {0,5,opp} — REFUSED.
        assert!(!t.split_edge(0, 6, 5));
        assert_eq!(t.tris.len(), tris_before);
        // Same for (5,6) at ... (0,5) now exists: splitting (0,5) at 6
        // is geometrically wrong anyway (6 beyond 5) — the guard keys on
        // connectivity: edge (6,0) exists → refuse.
        assert!(!t.split_edge(0, 5, 6));
        assert_eq!(t.tris.len(), tris_before);
        // Manifold invariant after all operations.
        let mut usage: HashMap<(u32, u32), usize> = HashMap::new();
        for tri in &t.tris {
            for i in 0..3 {
                let k = (tri[i].min(tri[(i + 1) % 3]), tri[i].max(tri[(i + 1) % 3]));
                *usage.entry(k).or_insert(0) += 1;
            }
        }
        assert!(
            usage.values().all(|&n| n <= 2),
            "edge usage >2 after guarded splits"
        );
    }

    /// Session-35 regression: `repair_spanning_edge` re-routes a
    /// spanning edge through the vertex it jumps over, preserving the
    /// region (the dropped half already exists) and manifoldness.
    #[test]
    fn canonical_cdt_repair_spanning_edge() {
        // 0(0,0) 1(2,0) 2(2,2) 3(0,2); 4=(1,0) ON edge (0,1) with the
        // half {4,1,2} present — the corrupt "spanning" configuration
        // the duplicate-guarded split refuses to touch.
        let verts = vec![[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0], [1.0, 0.0]];
        let mut t = Triangulation::new(verts, [[0, 1, 2], [0, 2, 3], [4, 1, 2]]);
        assert!(t.edge_exists(0, 1)); // the spanning edge
        assert!(!t.split_edge(0, 1, 4)); // guard: edge (4,1) exists
        assert!(t.repair_spanning_edge(0, 1, 4));
        assert!(!t.edge_exists(0, 1)); // spanning edge removed
        assert!(t.edge_exists(0, 4)); // re-routed connection created
        // Manifold invariant.
        let mut usage: HashMap<(u32, u32), usize> = HashMap::new();
        for tri in &t.tris {
            for i in 0..3 {
                let k = (tri[i].min(tri[(i + 1) % 3]), tri[i].max(tri[(i + 1) % 3]));
                *usage.entry(k).or_insert(0) += 1;
            }
        }
        assert!(
            usage.values().all(|&n| n <= 2),
            "edge usage >2 after spanning repair"
        );
        // Idempotence: repairing a non-spanning/gone edge refuses.
        assert!(!t.repair_spanning_edge(0, 1, 4)); // edge gone
    }

    /// Session-35 regression: a zero-area (collinear) triangle is
    /// TRANSPARENT to `locate` — every point off its line used to read
    /// as "strictly inside" (all orient signs collapse), and the
    /// interior split produced overlapping triangles.
    #[test]
    fn canonical_cdt_locate_degenerate_transparent() {
        // 0(0,0) 1(2,0) 2(1,0): collinear "triangle" (zero area);
        // 3(0,2) 4(2,2) above.
        let verts = vec![[0.0, 0.0], [2.0, 0.0], [1.0, 0.0], [0.0, 2.0], [2.0, 2.0]];
        let t = Triangulation::new(verts, [[0, 1, 2], [0, 3, 4], [0, 4, 1]]);
        // A point strictly inside the healthy region: the degenerate
        // [0,1,2] must NOT be returned as its container (linear
        // fallback finds the healthy triangle instead).
        let found = t.locate(0, [0.25, 1.0]);
        let (ti, on_edge) = found.expect("point inside the healthy region");
        assert!(!t.tri_is_degenerate(t.tris[ti]));
        assert!(!on_edge);
        // A point ON the collinear line between 0 and 1, e.g. (0.5, 0):
        // must resolve to a HEALTHY triangle having it on an edge —
        // never the degenerate one.
        let (ti2, on_edge2) = t.locate(0, [0.5, 0.0]).expect("on-edge point located");
        assert!(!t.tri_is_degenerate(t.tris[ti2]));
        assert!(on_edge2);
    }

    // ── Session-36: hostile-face screening + group rescue ──

    /// Face factory mirroring the converter's CanonicalFaceLoops.
    fn face(
        id: i64,
        outer: (Vec<Point3d>, Vec<Point2d>),
    ) -> CanonicalFaceLoops {
        CanonicalFaceLoops {
            step_face_id: id,
            forward: true,
            outer_3d: outer.0,
            outer_uv: outer.1,
            holes_3d: Vec::new(),
            holes_uv: Vec::new(),
        }
    }

    #[test]
    fn canonical_cdt_uv_sliver_ratio_metrics() {
        // Rectangle 2×1 → 2·area/d² = 4/5 = 0.8 — not a sliver.
        let (_, puv) = rect_loop(0.0, 2.0, 0.0, 1.0, 0);
        assert!(uv_sliver_ratio(&puv) > UV_SLIVER_RATIO);
        // Thin-but-legitimate strip 1:100 (0.02 wide × 2 long) — 0.04,
        // comfortably above the threshold.
        let (_, puv) = rect_loop(0.0, 2.0, 0.0, 0.02, 0);
        assert!(uv_sliver_ratio(&puv) > UV_SLIVER_RATIO);
        // Session-35 hostile geometry: 1e-6-wide strip of length 2
        // (sqrt-singular micro-sliver) → ~1e-6 — flagged.
        let (_, puv) = rect_loop(0.0, 2.0, 0.0, 1e-6, 0);
        assert!(uv_sliver_ratio(&puv) < UV_SLIVER_RATIO);
        // 1e-4-wide strip (upper end of the measured hostile range).
        let (_, puv) = rect_loop(0.0, 2.0, 0.0, 1e-4, 0);
        assert!(uv_sliver_ratio(&puv) < UV_SLIVER_RATIO);
        // Degenerate inputs read as slivers.
        assert!(uv_sliver_ratio(&[Point2d::new(0.0, 0.0)]) < UV_SLIVER_RATIO);
        let z = Point2d::new(1.0, 1.0);
        assert!(uv_sliver_ratio(&[z, z, z, z]) < UV_SLIVER_RATIO);
    }

    #[test]
    fn canonical_cdt_hostile_face_indices_screening() {
        // Normal face + micro-sliver face → only the sliver flagged.
        let faces = vec![
            face(1, rect_loop(0.0, 2.0, 0.0, 2.0, 1)),
            face(2, rect_loop(2.0, 4.0, 0.0, 1e-6, 1)),
        ];
        assert_eq!(hostile_face_indices(&faces), vec![1]);
        // A micro-sliver HOLE also flags the owning face.
        let (outer3d, outeruv) = rect_loop(0.0, 2.0, 0.0, 2.0, 1);
        let (hole3d, holeuv) = rect_loop(0.5, 1.5, 0.5, 0.5 + 1e-7, 0);
        let faces_hole = vec![CanonicalFaceLoops {
            step_face_id: 3,
            forward: true,
            outer_3d: outer3d,
            outer_uv: outeruv,
            holes_3d: vec![hole3d],
            holes_uv: vec![holeuv],
        }];
        assert_eq!(hostile_face_indices(&faces_hole), vec![0]);
        // Clean group → nothing flagged.
        let faces_clean = vec![
            face(4, rect_loop(0.0, 2.0, 0.0, 2.0, 1)),
            face(5, rect_loop(2.0, 4.0, 0.0, 2.0, 1)),
        ];
        assert!(hostile_face_indices(&faces_clean).is_empty());
    }

    #[test]
    fn canonical_cdt_resilient_drops_failing_face_and_rescues_group() {
        // Face B is malformed (3D/UV length mismatch on a HEALTHY UV
        // triangle — not a sliver, so the screening pass will not take
        // it): the plain build fails the WHOLE group; the resilient
        // build attributes B (malformed_loops), drops it, and survivor
        // A triangulates canonically.
        let nurbs = bilinear_patch(0.0, 4.0, 0.0, 2.0);
        let (a3d, auv) = rect_loop(0.0, 2.0, 0.0, 2.0, 1);
        let faces = vec![
            face(201, (a3d.clone(), auv.clone())),
            CanonicalFaceLoops {
                step_face_id: 202,
                forward: true,
                outer_3d: vec![
                    Point3d::new(3.0, 0.0, 0.0),
                    Point3d::new(4.0, 0.0, 0.0),
                    Point3d::new(4.0, 1.0, 0.0),
                    Point3d::new(3.0, 1.0, 0.0),
                ],
                outer_uv: vec![
                    Point2d::new(3.0, 0.0),
                    Point2d::new(4.0, 0.0),
                    Point2d::new(3.5, 1.0),
                ],
                holes_3d: Vec::new(),
                holes_uv: Vec::new(),
            },
        ];
        // Plain build (compat wrapper): whole-group failure (today's
        // behavior).
        assert!(build_canonical_surface_cdt(&nurbs, faces.clone(), &[]).is_none());
        // The malformed face is NOT width-hostile — the screening pass
        // finds nothing, so the rescue must proceed via attribution.
        assert!(hostile_face_indices(&faces).is_empty());
        // Resilient: B dropped (original index 1), A captured.
        let (cdt, dropped) = build_canonical_surface_cdt_resilient(&nurbs, faces, &[]);
        assert_eq!(dropped, vec![1], "the malformed face must be dropped");
        let cdt = cdt.expect("survivor face must build");
        assert_eq!(cdt.face_count(), 1);
        let mesh = cdt
            .extract_face_mesh(&nurbs, &a3d, &auv, &[], &[], true)
            .expect("face A extraction");
        assert!(!mesh.triangles.is_empty());
        // The dropped face no longer matches any canonical entry — the
        // caller routes it to the legacy path (extract returns None).
        assert!(cdt
            .extract_face_mesh(
                &nurbs,
                &[
                    Point3d::new(3.0, 0.0, 0.0),
                    Point3d::new(4.0, 0.0, 0.0),
                    Point3d::new(4.0, 1.0, 0.0),
                    Point3d::new(3.0, 1.0, 0.0),
                ],
                &[
                    Point2d::new(3.0, 0.0),
                    Point2d::new(4.0, 0.0),
                    Point2d::new(3.5, 1.0),
                ],
                &[],
                &[],
                true
            )
            .is_none());
    }

    #[test]
    fn canonical_cdt_resilient_sliver_group_never_regresses() {
        // Session-35-shaped group: a normal face plus a 1e-6 UV strip
        // face. Whatever the predicates do with the strip, the resilient
        // contract holds: face A is either captured together with B
        // (build succeeded) or rescued alone (B dropped → legacy), and
        // the extracted A mesh stays manifold (no regression vs the
        // whole-group legacy fallback of today).
        let nurbs = bilinear_patch(0.0, 4.0, 0.0, 2.0);
        let (a3d, auv) = rect_loop(0.0, 2.0, 0.0, 2.0, 1);
        let (s3d, suv) = rect_loop(2.0, 4.0, 0.0, 1e-6, 1);
        let faces = vec![
            face(301, (a3d.clone(), auv.clone())),
            face(302, (s3d, suv)),
        ];
        let (cdt, dropped) = build_canonical_surface_cdt_resilient(&nurbs, faces, &[]);
        if dropped.is_empty() {
            // The build survived the strip: both faces captured.
            assert_eq!(cdt.as_ref().expect("build reported success").face_count(), 2);
        } else {
            // Rescue path: exactly the sliver dropped, A captured.
            assert_eq!(dropped, vec![1]);
            assert_eq!(cdt.as_ref().expect("survivor face must build").face_count(), 1);
        }
        // In BOTH branches A must extract cleanly (rim contract intact).
        let cdt = cdt.as_ref().unwrap();
        let mesh = cdt
            .extract_face_mesh(&nurbs, &a3d, &auv, &[], &[], true)
            .expect("face A extraction in every branch");
        assert!(!mesh.triangles.is_empty());
        // Rim edges of A are interior to the extracted mesh (closed loop).
        let mut usage: HashMap<(u32, u32), usize> = HashMap::new();
        for t in &mesh.triangles {
            for i in 0..3 {
                let k = (t[i].min(t[(i + 1) % 3]), t[i].max(t[(i + 1) % 3]));
                *usage.entry(k).or_insert(0) += 1;
            }
        }
        assert!(
            usage.values().all(|&n| n <= 2),
            "extracted face mesh must stay manifold"
        );
    }

    #[test]
    fn canonical_cdt_resilient_clean_group_untouched() {
        // Never-worsen: a clean group's resilient result is bit-equal to
        // the plain build (first attempt = full group, zero drops).
        let nurbs = bilinear_patch(0.0, 4.0, 0.0, 2.0);
        let steiner = vec![Point2d::new(1.0, 1.0), Point2d::new(3.0, 1.0)];
        let faces = vec![
            face(401, rect_loop(0.0, 2.0, 0.0, 2.0, 1)),
            face(402, rect_loop(2.0, 4.0, 0.0, 2.0, 1)),
        ];
        let (cdt, dropped) =
            build_canonical_surface_cdt_resilient(&nurbs, faces.clone(), &steiner);
        assert!(dropped.is_empty(), "clean group must not lose faces");
        let cdt = cdt.expect("clean group must build");
        let plain =
            build_canonical_surface_cdt(&nurbs, faces, &steiner).expect("plain build");
        assert_eq!(
            cdt.canonical_triangle_count(),
            plain.canonical_triangle_count()
        );
        assert_eq!(cdt.face_count(), plain.face_count());
    }

    #[test]
    fn canonical_cdt_resilient_all_hostile_group_fails_gracefully() {
        // A group whose ONLY face is hostile: the rescue must not loop
        // forever and must not panic — either a lone buildable sliver
        // succeeds (Some) or the group returns None (whole-group legacy,
        // today's behavior). The contract under test is termination.
        let nurbs = bilinear_patch(0.0, 4.0, 0.0, 2.0);
        let faces = vec![face(501, rect_loop(0.0, 2.0, 0.0, 1e-6, 1))];
        let (cdt, _dropped) = build_canonical_surface_cdt_resilient(&nurbs, faces, &[]);
        if let Some(cdt) = cdt {
            assert_eq!(cdt.face_count(), 1);
        }
    }
    // ── Session-37: Lawson insertion legalization ──

    /// A strictly non-Delaunay convex quad must flip: verts
    /// 0=(0,0), 1=(5,0), 2=(5,2), 3=(0,4) with diagonal (1,3) — the
    /// circumcircle of (1,2,3) strictly contains 0, so edge (1,3) is
    /// not locally Delaunay and the flip to (0,2) is legal (convex,
    /// non-spanning).
    #[test]
    fn canonical_cdt_legalize_flips_non_delaunay_edge() {
        let mut tri = Triangulation::new(
            vec![[0.0, 0.0], [5.0, 0.0], [5.0, 2.0], [0.0, 4.0]],
            [[0, 1, 3], [1, 2, 3]],
        );
        let flips = tri.legalize_stack(vec![(1, 3)]);
        assert_eq!(flips, 1, "the non-Delaunay diagonal must flip exactly once");
        assert!(tri.edge_exists(0, 2), "new diagonal (0,2) must exist");
        assert!(!tri.edge_exists(1, 3), "old diagonal (1,3) must be gone");
        // Structural integrity: 2 triangles, every edge ≤ 2-adjacent.
        assert_eq!(tri.tris.len(), 2);
        let mut usage: HashMap<(u32, u32), usize> = HashMap::new();
        for t in &tri.tris {
            for i in 0..3 {
                let k = (t[i].min(t[(i + 1) % 3]), t[i].max(t[(i + 1) % 3]));
                *usage.entry(k).or_insert(0) += 1;
            }
        }
        assert!(usage.values().all(|&n| n <= 2));
    }

    /// Spanning-guard: edge (0,1) with opposites 2=(3,0), 3=(6,6) — the
    /// quad is strictly convex and o2 lies strictly inside the
    /// circumcircle of (0,1,2) (a flip WANTS to fire), but the new
    /// diagonal (2,3) passes strictly over vertex 4=(4.5,3) — the exact
    /// session-35 configuration (edges like (99,101) over vertex 100).
    /// The flip must be refused; removing the spanning vertex must make
    /// it legal again (positive control isolating the guard).
    #[test]
    fn canonical_cdt_legalize_spanning_guard() {
        // Positive control (no spanning vertex): flip is valid.
        let tri = Triangulation::new(
            vec![[0.0, 5.0], [10.0, 5.0], [3.0, 0.0], [6.0, 6.0]],
            [[0, 1, 2], [0, 1, 3]],
        );
        assert_eq!(
            tri.delaunay_flip_target(0, 1),
            Some((2, 3)),
            "without the middle vertex the flip must be valid"
        );
        // Spanning vertex on the would-be diagonal (3,0)-(6,6).
        let tri = Triangulation::new(
            vec![[0.0, 5.0], [10.0, 5.0], [3.0, 0.0], [6.0, 6.0], [4.5, 3.0]],
            [[0, 1, 2], [0, 1, 3]],
        );
        assert_eq!(
            tri.delaunay_flip_target(0, 1),
            None,
            "the diagonal spanning vertex 4 must be refused"
        );
        // legalize_stack over the same configuration: no flip, structure
        // untouched.
        let mut tri = Triangulation::new(
            vec![[0.0, 5.0], [10.0, 5.0], [3.0, 0.0], [6.0, 6.0], [4.5, 3.0]],
            [[0, 1, 2], [0, 1, 3]],
        );
        assert_eq!(tri.legalize_stack(vec![(0, 1)]), 0);
        assert!(tri.edge_exists(0, 1));
        assert!(!tri.edge_exists(2, 3));
    }

    /// Cocircular ties stay untouched (strict incircle — determinism):
    /// a square's diagonal is exactly cocircular; no flip may fire.
    #[test]
    fn canonical_cdt_legalize_cocircular_tie_untouched() {
        let mut tri = Triangulation::new(
            vec![[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]],
            [[0, 1, 3], [1, 2, 3]],
        );
        assert_eq!(tri.legalize_stack(vec![(1, 3)]), 0);
        assert!(tri.edge_exists(1, 3));
        assert!(!tri.edge_exists(0, 2));
    }

    /// Hull edges (single adjacent triangle) are never flippable, and a
    /// legalization seed on them is a harmless no-op.
    #[test]
    fn canonical_cdt_legalize_hull_edge_never_flips() {
        let mut tri = Triangulation::new(
            vec![[0.0, 0.0], [10.0, 0.0], [0.0, 10.0]],
            [[0, 1, 2]],
        );
        assert_eq!(tri.delaunay_flip_target(0, 1), None);
        assert_eq!(tri.legalize_stack(vec![(0, 1), (1, 2), (0, 2)]), 0);
        assert_eq!(tri.tris.len(), 1);
    }

    /// Legalized builds are deterministic: the same input produces the
    /// identical triangle list across two independent builds.
    #[test]
    fn canonical_cdt_legalized_build_deterministic() {
        let nurbs = bilinear_patch(0.0, 4.0, 0.0, 2.0);
        let steiner = vec![
            Point2d::new(1.0, 1.0),
            Point2d::new(3.0, 1.0),
            Point2d::new(2.0, 0.5),
        ];
        let faces = vec![
            face(601, rect_loop(0.0, 2.0, 0.0, 2.0, 3)),
            face(602, rect_loop(2.0, 4.0, 0.0, 2.0, 3)),
        ];
        let a = build_canonical_surface_cdt_inner(&nurbs, &faces, &steiner, true)
            .expect("legalized build");
        let b = build_canonical_surface_cdt_inner(&nurbs, &faces, &steiner, true)
            .expect("legalized build (repeat)");
        assert_eq!(a.tris, b.tris);
        assert_eq!(a.uv, b.uv);
        assert_eq!(a.face_tris, b.face_tris);
        assert!(a.legalized && b.legalized);
    }

    // Session-37 e2e regression: a REAL drill_top cone face (STEP face
    // #39037, captured via DRAPER_CANON_DUMP_FACES). The rim is a
    // triangle-like domain with two iso-parametric chains: the right
    // u~0.833 seam chain (wiggle ~1e-5) and the bottom v~1e-14 chain
    // (wiggle ~1e-15), plus a seam pinch (the closing point repeats the
    // 3D position of the first). The PLAIN build fails (blocked
    // non-convex quads on the collinear chains); the legalization retry
    // rescues the group; the lenient (legalized-only) extraction
    // accepts the zero-length raw rim segment and emits a manifold mesh.
    #[test]
    fn canonical_cdt_legalization_rescues_drill_top_cone_face() {
        let outer_3d = vec![
            Point3d::new(0.0, 0.6859999999999999, -0.5),
            Point3d::new(0.0, 0.6860577455020316, -0.4939006445193552),
            Point3d::new(0.0, 0.6862307358963271, -0.4878035027234153),
            Point3d::new(0.0, 0.6865186020151874, -0.48171069540104394),
            Point3d::new(0.0, 0.6869209746909171, -0.47562434334110826),
            Point3d::new(0.0, 0.6874374847558169, -0.4695465673324746),
            Point3d::new(0.0, 0.6880677630421896, -0.4634794881640092),
            Point3d::new(0.0, 0.6888114403823344, -0.4574252266245793),
            Point3d::new(0.0, 0.6896681476085558, -0.4513859035030494),
            Point3d::new(0.0, 0.6906375155531528, -0.44536363958828673),
            Point3d::new(0.0, 0.6917191750484317, -0.43936055566915844),
            Point3d::new(0.0, 0.69291275692669, -0.43337877253452994),
            Point3d::new(0.0, 0.694217892020232, -0.42742041097326755),
            Point3d::new(0.0, 0.6956342111613587, -0.42148759177423667),
            Point3d::new(0.0, 0.6971613451823728, -0.4155824357263054),
            Point3d::new(0.0, 0.6987989249155753, -0.40970706361834),
            Point3d::new(0.0, 0.7005465811932687, -0.403863596239205),
            Point3d::new(0.0, 0.7024039448477559, -0.3980541543777676),
            Point3d::new(0.0, 0.704370646711336, -0.392280858822895),
            Point3d::new(0.0, 0.7064462831495462, -0.38654581850180847),
            Point3d::new(0.0, 0.7086300498517328, -0.38085100445012454),
            Point3d::new(0.0, 0.7109208840064856, -0.37519829874113153),
            Point3d::new(0.0, 0.7133177184940465, -0.36958958196541225),
            Point3d::new(0.0, 0.7158194861946576, -0.3640267347135495),
            Point3d::new(0.0, 0.7184251199885576, -0.35851163757612703),
            Point3d::new(0.0, 0.7211335527559886, -0.3530461711437276),
            Point3d::new(0.0, 0.7239437173771925, -0.34763221600693406),
            Point3d::new(0.0, 0.7268545467324117, -0.3422716527563301),
            Point3d::new(0.0, 0.7298649737018881, -0.33696636198249763),
            Point3d::new(0.0, 0.7329739311658603, -0.33171822427602127),
            Point3d::new(0.0, 0.7361803520045704, -0.32652912022748204),
            Point3d::new(0.0, 0.7394831690982606, -0.32140093042746454),
            Point3d::new(0.0, 0.7428813153271729, -0.3163355354665516),
            Point3d::new(0.0, 0.7463737235715495, -0.31133481593532597),
            Point3d::new(0.0, 0.7499593267116289, -0.30640065242436965),
            Point3d::new(0.0, 0.7536370576276532, -0.3015349255242681),
            Point3d::new(0.0, 0.7574058491998645, -0.2967395158256023),
            Point3d::new(0.0, 0.761264630826016, -0.2920163009808405),
            Point3d::new(0.0, 0.7652121229544608, -0.2873669823554952),
            Point3d::new(0.0, 0.7692467221619932, -0.28279298807029996),
            Point3d::new(0.0, 0.773366797165485, -0.2782957227410616),
            Point3d::new(0.0, 0.7775707166818098, -0.27387659098358785),
            Point3d::new(0.0, 0.7818568494278413, -0.26953699741368453),
            Point3d::new(0.0, 0.7862235641204514, -0.26527834664715755),
            Point3d::new(0.0, 0.7906692294765154, -0.26110204329981546),
            Point3d::new(0.0, 0.7951922142129035, -0.25700949198746326),
            Point3d::new(0.0, 0.7997908870464912, -0.2530020973259095),
            Point3d::new(0.0, 0.8044636166941519, -0.24908126393095875),
            Point3d::new(0.0, 0.8092087718727559, -0.24524839641841867),
            Point3d::new(0.0, 0.8140247212991802, -0.24150489940409603),
            Point3d::new(0.0, 0.8189098336902951, -0.2378521775037976),
            Point3d::new(0.0, 0.8238624777629742, -0.23429163533332975),
            Point3d::new(0.0, 0.828881022234091, -0.23082467750849966),
            Point3d::new(0.0, 0.8339638358205193, -0.22745270864511324),
            Point3d::new(0.0, 0.8391092872391308, -0.2241771333589777),
            Point3d::new(0.0, 0.8443157452067993, -0.22099935626589984),
            Point3d::new(0.0, 0.8443157452067993, -0.22099935626589984),
            Point3d::new(-0.013297735181704684, 0.84426138965169, -0.22096907256178744),
            Point3d::new(-0.026596918012880832, 0.8440978363869309, -0.22087874803329255),
            Point3d::new(-0.03989891153566605, 0.8438243555133749, -0.2207291725561804),
            Point3d::new(-0.053205078792197824, 0.8434402171318709, -0.22052113600621537),
            Point3d::new(-0.06651678282461382, 0.8429446913432734, -0.22025542825916133),
            Point3d::new(-0.07983538667505163, 0.8423370482484334, -0.21993283919078266),
            Point3d::new(-0.09316225338564865, 0.8416165579482033, -0.21955415867684458),
            Point3d::new(-0.10649874567114015, 0.840782489760862, -0.2191201772954292),
            Point3d::new(-0.11984605992573605, 0.8398337154579778, -0.21863204240228606),
            Point3d::new(-0.13320495611597138, 0.8387680636422843, -0.2180918375433789),
            Point3d::new(-0.14657612348941695, 0.83758319388091, -0.2175017979654128),
            Point3d::new(-0.159960251293644, 0.836276765740978, -0.21686415891509192),
            Point3d::new(-0.17335802877622397, 0.8348464387896151, -0.21618115563912133),
            Point3d::new(-0.1867701451847279, 0.8332898725939444, -0.2154550233842043),
            Point3d::new(-0.20019728976672768, 0.8316047267210944, -0.21468799739704636),
            Point3d::new(-0.21364014545225718, 0.8297886543012574, -0.21388231639534983),
            Point3d::new(-0.2270988447559712, 0.8278387476469966, -0.2130405215075557),
            Point3d::new(-0.24057255045065018, 0.8257511110019298, -0.21216568666036295),
            Point3d::new(-0.25406032659756406, 0.8235217480326273, -0.21126094001482043),
            Point3d::new(-0.26756123725798364, 0.821146662405658, -0.21032940973197833),
            Point3d::new(-0.2810743464931784, 0.8186218577875906, -0.20937422397288508),
            Point3d::new(-0.2945987183644183, 0.8159433378449954, -0.20839851089859085),
            Point3d::new(-0.308133416932975, 0.8131071062444395, -0.20740539867014451),
            Point3d::new(-0.3216774742110564, 0.8101091552318955, -0.20639801383968592),
            Point3d::new(-0.33522886340488967, 0.8069450997505783, -0.2053794298057392),
            Point3d::new(-0.3487842816933089, 0.8036101000346747, -0.20435265590838103),
            Point3d::new(-0.3623403502870044, 0.8000992892473135, -0.20332069767397565),
            Point3d::new(-0.3758936903966692, 0.7964078005516324, -0.2022865606288864),
            Point3d::new(-0.3894409232329936, 0.7925307671107618, -0.2012532502994775),
            Point3d::new(-0.40297867000666976, 0.7884633220878356, -0.20022377221211363),
            Point3d::new(-0.41650355192838884, 0.7842005986459881, -0.19920113189315813),
            Point3d::new(-0.4300121188190067, 0.7797377581236873, -0.19818829974706675),
            Point3d::new(-0.4434997213732279, 0.7750704351169766, -0.19718765623998769),
            Point3d::new(-0.45696071528991666, 0.7701946569156259, -0.19620109232646854),
            Point3d::new(-0.4703894261503532, 0.7651064626958757, -0.19523048414400312),
            Point3d::new(-0.4837801795358141, 0.7598018916339679, -0.19427770783008302),
            Point3d::new(-0.49712730102757785, 0.7542769829061431, -0.1933446395222016),
            Point3d::new(-0.5104251162069229, 0.7485277756886397, -0.19243315535785221),
            Point3d::new(-0.5236679506551276, 0.7425503091576999, -0.1915451314745269),
            Point3d::new(-0.536850060785806, 0.7363407702457678, -0.19068235591815563),
            Point3d::new(-0.5499650235094542, 0.7298967974422439, -0.1898457513231504),
            Point3d::new(-0.5630060300576751, 0.7232168531251268, -0.1890357491253667),
            Point3d::new(-0.5759662672353425, 0.7162994091288102, -0.1882527751227996),
            Point3d::new(-0.5888389218473318, 0.709142937287691, -0.18749725511344462),
            Point3d::new(-0.6016171806985149, 0.7017459094361627, -0.1867696148952973),
            Point3d::new(-0.6142942305937673, 0.6941067974086206, -0.1860702802663523),
            Point3d::new(-0.6268632583379627, 0.68622407303946, -0.18539967702460558),
            Point3d::new(-0.6393175097724182, 0.6780965181159839, -0.18475812493251675),
            Point3d::new(-0.6516505950698335, 0.6697248272367169, -0.18414528937583574),
            Point3d::new(-0.6638562632478742, 0.6611104239635051, -0.18356058636044326),
            Point3d::new(-0.675928263597525, 0.6522547332931623, -0.1830034314013127),
            Point3d::new(-0.6878603454097689, 0.6431591802224972, -0.18247324001341836),
            Point3d::new(-0.6996462579755907, 0.6338251897483236, -0.1819694277117354),
            Point3d::new(-0.7112797505859731, 0.6242541868674536, -0.1814914100112377),
            Point3d::new(-0.7227545725318993, 0.6144475965766993, -0.18103860242689995),
            Point3d::new(-0.7227545725318993, 0.6144475965766993, -0.18103860242689995),
            Point3d::new(-0.716684032589054, 0.609286745549845, -0.18270184306543547),
            Point3d::new(-0.7106475883912697, 0.6041548809167203, -0.18456652763755343),
            Point3d::new(-0.7046487940166966, 0.5990550241660255, -0.1866305693308199),
            Point3d::new(-0.6986912035434809, 0.5939901967864678, -0.18889188133280088),
            Point3d::new(-0.6927783710497746, 0.5889634202667491, -0.19134837683106243),
            Point3d::new(-0.686913850613724, 0.5839777160955713, -0.19399796901317057),
            Point3d::new(-0.6811011963134757, 0.5790361057616398, -0.19683857106669134),
            Point3d::new(-0.6753439622271831, 0.5741416107536583, -0.1998680961791912),
            Point3d::new(-0.6696457024329892, 0.569297252560327, -0.20308445753823579),
            Point3d::new(-0.6640099743589989, 0.5645060555183044, -0.2064855645547672),
            Point3d::new(-0.6584403577037872, 0.559771062897422, -0.21006930153272796),
            Point3d::new(-0.6529404412510047, 0.555095325691159, -0.21383354253385622),
            Point3d::new(-0.647513813811095, 0.5504818949157748, -0.2177761615896765),
            Point3d::new(-0.642164064194505, 0.545933821587532, -0.2218950327317133),
            Point3d::new(-0.6368947812116854, 0.5414541567226898, -0.226188029991492),
            Point3d::new(-0.6317095536730797, 0.5370459513375092, -0.23065302740053717),
            Point3d::new(-0.6266119703891349, 0.5327122564482529, -0.23528789899037328),
            Point3d::new(-0.6216056201702997, 0.528456123071182, -0.24009051879252574),
            Point3d::new(-0.6166940373935699, 0.5242805559461239, -0.24505866916427532),
            Point3d::new(-0.6118801236470635, 0.5201880218493748, -0.25018906674982944),
            Point3d::new(-0.6071663722680167, 0.5161806404839826, -0.25547774063657425),
            Point3d::new(-0.6025552697894838, 0.5122605257684434, -0.2609207084526126),
            Point3d::new(-0.5980493027445153, 0.5084297916212517, -0.2665139878260492),
            Point3d::new(-0.5936509576661653, 0.5046905519609055, -0.2722535963849877),
            Point3d::new(-0.5893627210874879, 0.5010449207058958, -0.27813555175753457),
            Point3d::new(-0.5851870795415337, 0.4974950117747188, -0.2841558715717927),
            Point3d::new(-0.5811265195613586, 0.4940429390858707, -0.29031057345586575),
            Point3d::new(-0.5771835279737996, 0.49069081680760807, -0.29659567535517706),
            Point3d::new(-0.573360598362795, 0.4874407648527139, -0.30300720251344515),
            Point3d::new(-0.5696602310693795, 0.48429490887849447, -0.30954118747268744),
            Point3d::new(-0.5660849267283705, 0.4812553747920205, -0.3161936630922373),
            Point3d::new(-0.5626371859745891, 0.478324288500362, -0.32296066223142805),
            Point3d::new(-0.559319509442858, 0.4755037759105871, -0.32983821774959576),
            Point3d::new(-0.5561343977679964, 0.47279596292976755, -0.3368223625060729),
            Point3d::new(-0.5530843515848236, 0.4702029754649715, -0.3439091293601946),
            Point3d::new(-0.5501718715281587, 0.4677269394232688, -0.3510945511712942),
            Point3d::new(-0.547399448358826, 0.4653699723173794, -0.3583746542327102),
            Point3d::new(-0.5447689803976346, 0.46313368799895205, -0.3657450708780363),
            Point3d::new(-0.5422814476833775, 0.46101891964498165, -0.3732008228032555),
            Point3d::new(-0.5399377512628512, 0.4590264332776517, -0.3807368791763901),
            Point3d::new(-0.5377387921828483, 0.4571569949191492, -0.38834820916545887),
            Point3d::new(-0.5356854714901651, 0.4554113705916576, -0.3960297819384815),
            Point3d::new(-0.5337786902315926, 0.45379032631736216, -0.4037765666634776),
            Point3d::new(-0.5320193494539289, 0.4522946281184481, -0.4115835325084678),
            Point3d::new(-0.5304083502039649, 0.4509250420171007, -0.4194456486414708),
            Point3d::new(-0.5289465935110496, 0.4496823340206717, -0.4273578842708199),
            Point3d::new(-0.5276349744902582, 0.4485672651085144, -0.4353152222712584),
            Point3d::new(-0.5264743737588429, 0.4475805839347018, -0.443312679018371),
            Point3d::new(-0.5254656697532774, 0.44672303729932406, -0.45134527592697804),
            Point3d::new(-0.5246097409100372, 0.4459953720024732, -0.4594080344119016),
            Point3d::new(-0.5239074656655962, 0.4453983348442403, -0.4674959758879629),
            Point3d::new(-0.5233597224564264, 0.4449326726247147, -0.47560412176998135),
            Point3d::new(-0.522967389719005, 0.4445991321439893, -0.48372749347278),
            Point3d::new(-0.5227313458898042, 0.44439846020215423, -0.49186111241117914),
            Point3d::new(-0.5226524694052994, 0.4443314035992998, -0.5),
            Point3d::new(-0.5226524694052994, 0.4443314035992998, -0.5),
            Point3d::new(-0.5100343369307634, 0.4587602589061026, -0.5),
            Point3d::new(-0.4970180045706858, 0.47283094561647854, -0.5),
            Point3d::new(-0.48361363458608153, 0.4865324783017462, -0.5),
            Point3d::new(-0.46983169219117027, 0.49985415974339986, -0.5),
            Point3d::new(-0.45568293738284993, 0.5127855892847784, -0.5),
            Point3d::new(-0.4411784165400352, 0.5253166709511756, -0.5),
            Point3d::new(-0.4263294537993998, 0.5374376213320993, -0.5),
            Point3d::new(-0.4111476422142637, 0.5491389772194744, -0.5),
            Point3d::new(-0.3956448347035346, 0.5604116029958792, -0.5),
            Point3d::new(-0.3798331347977584, 0.5712466977670037, -0.5),
            Point3d::new(-0.3637248871895098, 0.5816358022327872, -0.5),
            Point3d::new(-0.3473326680955049, 0.5915708052918571, -0.5),
            Point3d::new(-0.3306692754379519, 0.6010439503741285, -0.5),
            Point3d::new(-0.31374771885280595, 0.6100478414965984, -0.5),
            Point3d::new(-0.29658120953274025, 0.6185754490376212, -0.5),
            Point3d::new(-0.27918314991275217, 0.62662011522516, -0.5),
            Point3d::new(-0.26156712320645514, 0.6341755593347145, -0.5),
            Point3d::new(-0.24374688280123902, 0.6412358825928859, -0.5),
            Point3d::new(-0.22573634152056643, 0.6477955727827336, -0.5),
            Point3d::new(-0.20754956076179232, 0.6538495085473315, -0.5),
            Point3d::new(-0.18920073951798866, 0.6593929633881803, -0.5),
            Point3d::new(-0.17070420329234226, 0.6644216093553297, -0.5),
            Point3d::new(-0.15207439291378222, 0.6689315204263462, -0.5),
            Point3d::new(-0.13332585326257096, 0.6729191755714847, -0.5),
            Point3d::new(-0.11447322191465692, 0.6763814615026611, -0.5),
            Point3d::new(-0.09553121771365713, 0.679315675104105, -0.5),
            Point3d::new(-0.07651462927939279, 0.6817195255427535, -0.5),
            Point3d::new(-0.05743830346194745, 0.6835911360567888, -0.5),
            Point3d::new(-0.03831713375026258, 0.6849290454208834, -0.5),
            Point3d::new(-0.01916604864432181, 0.6857322090870177, -0.5),
            Point3d::new(0.0, 0.6859999999999999, -0.5),
        ];
        let outer_uv = vec![
            Point2d::new(-3.266491523407789e-14, 3.83128173528171e-15),
            Point2d::new(0.01818181818178114, 3.8313706505801146e-15),
            Point2d::new(0.036363636363591016, 3.830153122874718e-15),
            Point2d::new(0.05454545454540484, 3.8393702132844996e-15),
            Point2d::new(0.07272727272722078, 3.878967053160334e-15),
            Point2d::new(0.09090909090903722, 3.90543168945638e-15),
            Point2d::new(0.10909090909085499, 3.9100988896415996e-15),
            Point2d::new(0.12727272727267058, 3.9744042723040025e-15),
            Point2d::new(0.14545454545448994, 3.976988564416318e-15),
            Point2d::new(0.16363636363630904, 4.053927877425321e-15),
            Point2d::new(0.1818181818181269, 4.099075862615511e-15),
            Point2d::new(0.1999999999999446, 4.092673976253245e-15),
            Point2d::new(0.21818181818176177, 4.187217515536753e-15),
            Point2d::new(0.23636363636358293, 4.16999867480379e-15),
            Point2d::new(0.2545454545454011, 4.205820444053153e-15),
            Point2d::new(0.27272727272721625, 4.241078239344783e-15),
            Point2d::new(0.29090909090903383, 4.272673807923062e-15),
            Point2d::new(0.30909090909085224, 4.300388749340173e-15),
            Point2d::new(0.3272727272726665, 4.3235107372748624e-15),
            Point2d::new(0.3454545454544843, 4.39067590873114e-15),
            Point2d::new(0.3636363636362989, 4.396239464888493e-15),
            Point2d::new(0.3818181818181129, 4.397621824105978e-15),
            Point2d::new(0.3999999999999269, 4.362592424478279e-15),
            Point2d::new(0.4181818181817445, 4.3892165621079844e-15),
            Point2d::new(0.4363636363635594, 4.35429617125559e-15),
            Point2d::new(0.45454545454537404, 4.3660512007997304e-15),
            Point2d::new(0.47272727272718873, 4.3280245682600415e-15),
            Point2d::new(0.4909090909090043, 4.323095867901138e-15),
            Point2d::new(0.5090909090908237, 4.289645252897791e-15),
            Point2d::new(0.5272727272726396, 4.262241953727767e-15),
            Point2d::new(0.5454545454544578, 4.2307718847476595e-15),
            Point2d::new(0.5636363636362762, 4.207000597475681e-15),
            Point2d::new(0.5818181818180959, 4.173113454899973e-15),
            Point2d::new(0.5999999999999184, 4.102455792197897e-15),
            Point2d::new(0.6181818181817436, 4.101766074083912e-15),
            Point2d::new(0.6363636363635655, 4.065165472385022e-15),
            Point2d::new(0.6545454545453897, 4.025622399352476e-15),
            Point2d::new(0.6727272727272176, 3.918471953740358e-15),
            Point2d::new(0.6909090909090461, 3.873464094703693e-15),
            Point2d::new(0.7090909090908772, 3.833669393877603e-15),
            Point2d::new(0.7272727272727089, 3.869905607392966e-15),
            Point2d::new(0.7454545454545407, 3.759872915795566e-15),
            Point2d::new(0.7636363636363744, 3.810511568759938e-15),
            Point2d::new(0.7818181818182094, 3.698021256682738e-15),
            Point2d::new(0.8000000000000475, 3.680445607629041e-15),
            Point2d::new(0.818181818181884, 3.669320435363064e-15),
            Point2d::new(0.8363636363637209, 3.670528033185704e-15),
            Point2d::new(0.8545454545455637, 3.681598303354775e-15),
            Point2d::new(0.8727272727274035, 3.843830292073948e-15),
            Point2d::new(0.8909090909092493, 3.750877122372139e-15),
            Point2d::new(0.9090909090910942, 3.9604502842406865e-15),
            Point2d::new(0.9272727272729399, 4.041594198631444e-15),
            Point2d::new(0.9454545454547857, 3.980988986184576e-15),
            Point2d::new(0.963636363636636, 4.099973744578067e-15),
            Point2d::new(0.9818181818184857, 4.4016679521317585e-15),
            Point2d::new(1.000000000000336, 4.564715226367851e-15),
            Point2d::new(1.000000000000336, 4.564715226367851e-15),
            Point2d::new(0.999999068638362, 0.01818176457185888),
            Point2d::new(0.999997356264596, 0.03636352643455933),
            Point2d::new(0.9999953745394531, 0.05454531936798146),
            Point2d::new(0.9999936354709118, 0.07272716647783287),
            Point2d::new(0.9999926494483355, 0.09090908011937503),
            Point2d::new(0.999992923300113, 0.1090910618587116),
            Point2d::new(0.9999949583641368, 0.12727310247014068),
            Point2d::new(0.9999992454815548, 0.14545518166009394),
            Point2d::new(1.0000047042491957, 0.1636371208861581),
            Point2d::new(1.0000068743114758, 0.18181848735638811),
            Point2d::new(1.000005390611427, 0.1999994231685952),
            Point2d::new(1.0000018291474186, 0.2181802787405809),
            Point2d::new(0.999997775001678, 0.23636136338609245),
            Point2d::new(0.9999948137098025, 0.2545429438603678),
            Point2d::new(0.9999945282600832, 0.27272524369656204),
            Point2d::new(0.9999984732846473, 0.2909084365444309),
            Point2d::new(1.0000062142080979, 0.3090921381193811),
            Point2d::new(1.000013799050933, 0.32727510380837704),
            Point2d::new(1.0000171782001452, 0.3454562415575639),
            Point2d::new(1.0000157819272502, 0.3636358278000511),
            Point2d::new(1.0000113438055085, 0.3818149244424347),
            Point2d::new(1.0000057038643695, 0.3999945328442241),
            Point2d::new(1.0000007746561501, 0.4181755777550603),
            Point2d::new(0.9999985022243678, 0.4363588712973996),
            Point2d::new(0.9999995096190865, 0.45454404623308614),
            Point2d::new(1.0000026651808116, 0.47272942499226694),
            Point2d::new(1.0000065539823435, 0.490913339546462),
            Point2d::new(1.0000095936295195, 0.5090942899258444),
            Point2d::new(1.0000106690393191, 0.5272725147653784),
            Point2d::new(1.0000094006769293, 0.545449898482987),
            Point2d::new(1.0000056923495593, 0.5636283405579078),
            Point2d::new(0.999999765680506, 0.581809579940407),
            Point2d::new(0.9999928501602008, 0.5999936418926538),
            Point2d::new(0.999986744632409, 0.618179156061113),
            Point2d::new(0.9999830443114283, 0.6363647072215636),
            Point2d::new(0.999983109511214, 0.6545488812803304),
            Point2d::new(0.9999880238457075, 0.6727302924189673),
            Point2d::new(0.9999966504377478, 0.690908932667234),
            Point2d::new(1.0000048930682293, 0.7090869474487608),
            Point2d::new(1.000009125735603, 0.7272665333641206),
            Point2d::new(1.000009156016947, 0.7454481395216419),
            Point2d::new(1.0000065404442355, 0.7636311913389432),
            Point2d::new(1.000002741812932, 0.7818150776125228),
            Point2d::new(0.9999991049486988, 0.7999991637346021),
            Point2d::new(0.9999968535124049, 0.8181827935642866),
            Point2d::new(0.9999970821392278, 0.8363652932671043),
            Point2d::new(0.9999991135709506, 0.8545465945378945),
            Point2d::new(0.9999990362820962, 0.8727279879995387),
            Point2d::new(0.9999970418029728, 0.8909097639899299),
            Point2d::new(0.9999950297249336, 0.9090916834743638),
            Point2d::new(0.9999946425907618, 0.92727348107164),
            Point2d::new(0.9999958150630823, 0.9454551074828347),
            Point2d::new(0.9999976696617701, 0.9636366526659041),
            Point2d::new(0.9999993442039657, 0.9818182350275175),
            Point2d::new(1.0000000000002462, 1.0000000000001505),
            Point2d::new(1.0000000000002462, 1.0000000000001505),
            Point2d::new(0.9818208420128224, 0.9999999999994983),
            Point2d::new(0.9636414348975575, 0.9999999999963166),
            Point2d::new(0.9454617564812444, 0.9999999999910796),
            Point2d::new(0.9272817819268151, 0.9999999999858056),
            Point2d::new(0.9091014838884371, 0.9999999999826895),
            Point2d::new(0.8909208326701813, 0.999999999983133),
            Point2d::new(0.8727397963925551, 0.9999999999871663),
            Point2d::new(0.8545583411632752, 0.9999999999933983),
            Point2d::new(0.8363764312535604, 0.9999999999994064),
            Point2d::new(0.8181940417778509, 1.0000000000023395),
            Point2d::new(0.800011230293992, 1.000000000000005),
            Point2d::new(0.7818280896368851, 0.9999999999931547),
            Point2d::new(0.7636447140988587, 0.9999999999849556),
            Point2d::new(0.7454611988039215, 0.9999999999793784),
            Point2d::new(0.7272776391404538, 0.9999999999793737),
            Point2d::new(0.7090941302239594, 0.9999999999852297),
            Point2d::new(0.6909107662999772, 0.999999999993922),
            Point2d::new(0.6727276401402915, 0.9999999999998537),
            Point2d::new(0.6545448358486995, 0.9999999999979544),
            Point2d::new(0.6363623592888382, 0.9999999999906602),
            Point2d::new(0.6181801671489873, 0.9999999999833166),
            Point2d::new(0.5999982162044218, 0.999999999979357),
            Point2d::new(0.5818164644272859, 0.9999999999799989),
            Point2d::new(0.5636348712453557, 0.9999999999844947),
            Point2d::new(0.5454533977805879, 0.9999999999908338),
            Point2d::new(0.527272007076246, 0.9999999999965944),
            Point2d::new(0.5090906643141162, 0.99999999999982),
            Point2d::new(0.49090933700928513, 0.9999999999999242),
            Point2d::new(0.47272799417289485, 0.9999999999988052),
            Point2d::new(0.45454660333590646, 0.9999999999988254),
            Point2d::new(0.43636512969661234, 1.0000000000007),
            Point2d::new(0.41818353631589616, 1.0000000000036797),
            Point2d::new(0.4000017843323116, 1.0000000000061782),
            Point2d::new(0.3818198331918234, 1.0000000000066631),
            Point2d::new(0.36363764088321326, 1.0000000000046125),
            Point2d::new(0.34545516420030636, 1.0000000000013582),
            Point2d::new(0.327272359847332, 1.000000000000535),
            Point2d::new(0.30908923369390373, 1.000000000005162),
            Point2d::new(0.29090586982969263, 1.0000000000134757),
            Point2d::new(0.2727223610131545, 1.00000000002211),
            Point2d::new(0.254538801478752, 1.000000000027911),
            Point2d::new(0.2363552863230969, 1.000000000028926),
            Point2d::new(0.2181719109287249, 1.0000000000248155),
            Point2d::new(0.19998877040043295, 1.0000000000168119),
            Point2d::new(0.1818059590234126, 1.0000000000072757),
            Point2d::new(0.16362356961827682, 0.9999999999990284),
            Point2d::new(0.14544165973585668, 0.9999999999940536),
            Point2d::new(0.1272602044926333, 0.9999999999922861),
            Point2d::new(0.10907916816533804, 0.9999999999928981),
            Point2d::new(0.09089851686449903, 0.9999999999948203),
            Point2d::new(0.07271821871542036, 0.9999999999970244),
            Point2d::new(0.054538244026244305, 0.9999999999987789),
            Point2d::new(0.03635856545551152, 0.9999999999997761),
            Point2d::new(0.018179158170108813, 1.0000000000001252),
            Point2d::new(-2.4612319357597036e-14, 1.00000000000017),
            Point2d::new(-2.4612319357597036e-14, 1.00000000000017),
            Point2d::new(-1.2041782628909486e-13, 0.9677419298206387),
            Point2d::new(-8.466880111483037e-14, 0.9354838666352288),
            Point2d::new(-2.331261263357152e-14, 0.903225810305769),
            Point2d::new(1.0427752020944005e-15, 0.8709677578789627),
            Point2d::new(2.4663644019816522e-15, 0.8387096817192615),
            Point2d::new(2.9345149202159837e-15, 0.806451601284247),
            Point2d::new(7.151023767848713e-15, 0.7741935747446488),
            Point2d::new(1.3707284295657025e-14, 0.7419355756459546),
            Point2d::new(2.103657015383944e-14, 0.7096775295456993),
            Point2d::new(2.8735409283919994e-14, 0.67741939255183),
            Point2d::new(3.443596368155901e-14, 0.6451612251896028),
            Point2d::new(3.801768856856942e-14, 0.6129031378867323),
            Point2d::new(4.0196032155779875e-14, 0.5806451307317771),
            Point2d::new(4.166123174614647e-14, 0.5483871287287642),
            Point2d::new(4.182735157649713e-14, 0.516129060643475),
            Point2d::new(4.1093041537711883e-14, 0.48387093714556356),
            Point2d::new(4.12321801960199e-14, 0.45161285638252757),
            Point2d::new(3.961205750693926e-14, 0.4193548407247218),
            Point2d::new(3.6247497680606905e-14, 0.38709682906625786),
            Point2d::new(3.0252075469751405e-14, 0.3548387542461352),
            Point2d::new(2.1664415522230703e-14, 0.3225806215823751),
            Point2d::new(8.587941507642446e-15, 0.29032253103406874),
            Point2d::new(-6.695418855918243e-15, 0.2580645167643862),
            Point2d::new(-2.322030580200793e-14, 0.22580651778917862),
            Point2d::new(-4.05107180527367e-14, 0.1935484540821129),
            Point2d::new(-5.826040795922995e-14, 0.16129030541888453),
            Point2d::new(-7.304547589376469e-14, 0.12903216440302212),
            Point2d::new(-8.180420812411391e-14, 0.09677410455521873),
            Point2d::new(-8.102564998253571e-14, 0.06451609487376742),
            Point2d::new(-6.529334065451125e-14, 0.03225806821897282),
            Point2d::new(-3.2622564498347626e-14, 3.8303081554017496e-15),
        ];
        let steiner = vec![
            Point2d::new(-0.01637721993029, -0.008417508642905),
            Point2d::new(-0.01637721993029, 0.08333333333335),
            Point2d::new(-0.01637721993029, 0.25),
            Point2d::new(-0.01637721993029, 0.41666666666665),
            Point2d::new(-0.01637721993029, 0.58333333333335),
            Point2d::new(-0.01637721993029, 0.75),
            Point2d::new(-0.01637721993029, 0.8733333333333),
            Point2d::new(-0.01637721993029, 0.95666666666665),
            Point2d::new(-0.01637721993029, 1.0084178848585),
            Point2d::new(0.16666666666665, -0.008417508642905),
            Point2d::new(0.16666666666665, 0.08333333333335),
            Point2d::new(0.16666666666665, 0.25),
            Point2d::new(0.16666666666665, 0.41666666666665),
            Point2d::new(0.16666666666665, 0.58333333333335),
            Point2d::new(0.16666666666665, 0.75),
            Point2d::new(0.16666666666665, 0.8733333333333),
            Point2d::new(0.16666666666665, 0.95666666666665),
            Point2d::new(0.16666666666665, 1.0084178848585),
            Point2d::new(0.5, -0.008417508642905),
            Point2d::new(0.5, 0.08333333333335),
            Point2d::new(0.5, 0.25),
            Point2d::new(0.5, 0.41666666666665),
            Point2d::new(0.5, 0.58333333333335),
            Point2d::new(0.5, 0.75),
            Point2d::new(0.5, 0.8733333333333),
            Point2d::new(0.5, 0.95666666666665),
            Point2d::new(0.5, 1.0084178848585),
            Point2d::new(0.83333333333335, -0.008417508642905),
            Point2d::new(0.83333333333335, 0.08333333333335),
            Point2d::new(0.83333333333335, 0.25),
            Point2d::new(0.83333333333335, 0.41666666666665),
            Point2d::new(0.83333333333335, 0.58333333333335),
            Point2d::new(0.83333333333335, 0.75),
            Point2d::new(0.83333333333335, 0.8733333333333),
            Point2d::new(0.83333333333335, 0.95666666666665),
            Point2d::new(0.83333333333335, 1.0084178848585),
            Point2d::new(1.011829368196, -0.008417508642905),
            Point2d::new(1.011829368196, 0.08333333333335),
            Point2d::new(1.011829368196, 0.25),
            Point2d::new(1.011829368196, 0.41666666666665),
            Point2d::new(1.011829368196, 0.58333333333335),
            Point2d::new(1.011829368196, 0.75),
            Point2d::new(1.011829368196, 0.8733333333333),
            Point2d::new(1.011829368196, 0.95666666666665),
            Point2d::new(1.011829368196, 1.0084178848585),
        ];
        // Bilinear patch over the UV window: the failure is purely
        // UV-topological; 3D positions only feed dedup keys.
        let nurbs = bilinear_patch(0.0, 1.0, 0.0, 1.0);
        let faces = vec![CanonicalFaceLoops {
            step_face_id: 39037,
            forward: false,
            outer_3d,
            outer_uv,
            holes_3d: Vec::new(),
            holes_uv: Vec::new(),
        }];

        // Plain build: fails (the pre-session-37 behavior — legacy path).
        assert!(
            build_canonical_surface_cdt_inner(&nurbs, &faces, &steiner, false).is_err(),
            "plain build must fail on the collinear chains"
        );
        // Legalized build: succeeds.
        let cdt = build_canonical_surface_cdt_inner(&nurbs, &faces, &steiner, true)
            .expect("legalized build must rescue the group");
        assert!(cdt.legalized);
        // Resilient (the production entry point): rescued, no drops.
        let (cdt, dropped) = build_canonical_surface_cdt_resilient(&nurbs, faces.clone(), &steiner);
        assert!(dropped.is_empty());
        let cdt = cdt.expect("resilient must capture the group");
        assert!(cdt.legalized);
        // Extraction with the RAW caller loops (seam-pinch duplicate
        // included): lenient rim contract, manifold-checked emission.
        let mesh = cdt
            .extract_face_mesh(
                &nurbs,
                &faces[0].outer_3d,
                &faces[0].outer_uv,
                &[],
                &[],
                false,
            )
            .expect("legalized extraction must succeed");
        assert!(!mesh.triangles.is_empty());
        let mut usage: HashMap<(u32, u32), usize> = HashMap::new();
        for t in &mesh.triangles {
            for i in 0..3 {
                let k = (t[i].min(t[(i + 1) % 3]), t[i].max(t[(i + 1) % 3]));
                *usage.entry(k).or_insert(0) += 1;
            }
        }
        assert!(
            usage.values().all(|&n| n <= 2),
            "extracted face mesh must stay manifold"
        );
    }
}
