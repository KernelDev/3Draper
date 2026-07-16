// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Mesh data structures.

use draper_geometry::Point3d;
use std::cell::Cell;
use std::collections::HashMap;

/// A bit-exact hash key for a 3D point, used for vertex deduplication.
///
/// Since the edge cache applies deterministic rounding (48-bit mantissa precision),
/// shared-edge vertices produce bit-identical f64 values. Using the raw bit
/// representation as a hash key is both correct and fast — no epsilon comparison
/// is needed because the rounding already guarantees consistent bit patterns.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct VertexKey(u64, u64, u64);

impl VertexKey {
    /// Create a bit-exact key from a Point3d.
    ///
    /// Applies `deterministic_round_point` BEFORE hashing. The edge cache
    /// already applies this rounding when storing points, so cached boundary
    /// points will match. Non-cached points (interior, fallback sampling)
    /// also get rounded, ensuring bit-exact matching across paths.
    ///
    /// The 48-bit mantissa rounding (~1e-14 relative precision) does NOT
    /// collapse distinct features in normal models. For extremely small
    /// models (sub-micron), the merge_tol floor is the separate problem
    /// (fixed by removing the .max(0.005) floor).
    #[inline]
    pub fn from_point(p: &Point3d) -> Self {
        let rounded = crate::edge_cache::deterministic_round_point(*p);
        VertexKey(rounded.x.to_bits(), rounded.y.to_bits(), rounded.z.to_bits())
    }
}

/// A vertex deduplication map that tracks which 3D points have already been
/// added to a mesh, mapping them to vertex indices.
///
/// Used during the "topology-first" merge step to ensure that shared-edge
/// vertices from different faces get the same vertex index in the final mesh,
/// making it watertight by construction.
///
/// # Two-tier lookup strategy:
///
/// 1. **Bit-exact**: First tries `VertexKey` (bit-exact f64 comparison).
///    This is O(1) and handles the common case where the edge cache produces
///    bit-identical coordinates for shared edges.
///
/// 2. **Tolerance-based**: Falls back to spatial hash grid for near-miss
///    vertices that are geometrically close but not bit-identical. This
///    handles the case where adjacent faces use different STEP EDGE_CURVE
///    entities on their common geometric boundary (the edge cache can't
///    produce bit-identical points because the STEP entities differ).
pub struct VertexDedupMap {
    /// Bit-exact vertex → index mapping (fast path).
    exact: HashMap<VertexKey, u32>,
    /// Spatial hash grid for tolerance-based near-miss lookups (slow path).
    /// Maps (cell_x, cell_y, cell_z) → list of (vertex_index, Point3d).
    spatial: HashMap<(i64, i64, i64), Vec<(u32, Point3d)>>,
    /// Cell size for spatial hash (= merge tolerance).
    tolerance: f64,
    /// Whether tolerance-based fallback is enabled.
    use_tolerance: bool,
    /// Number of bit-exact matches (edge cache working correctly).
    exact_hits: Cell<usize>,
    /// Number of tolerance-based matches (near-miss from non-cached edges).
    tolerance_hits: Cell<usize>,
    /// Number of new vertex insertions.
    misses: Cell<usize>,
}

impl VertexDedupMap {
    /// Create a new dedup map with bit-exact comparison only.
    /// Use this when edge cache guarantees bit-identical 3D coordinates.
    ///
    /// Alias for [`bit_exact()`](Self::bit_exact) — prefer using `bit_exact()`
    /// for clarity in production code to make the dedup strategy explicit.
    pub fn new() -> Self {
        Self::bit_exact()
    }

    /// Create a new dedup map with **bit-exact** comparison only.
    ///
    /// This is the recommended dedup strategy when the edge cache uses
    /// deterministic rounding (48-bit mantissa), guaranteeing that shared
    /// edges between adjacent faces produce bit-identical 3D coordinates.
    /// Vertices that are not bit-exact will NOT be merged — they represent
    /// genuine geometry differences that should be caught by watertight
    /// validation, not silently merged.
    ///
    /// Use [`with_tolerance()`](Self::with_tolerance) only as a diagnostic
    /// tool to identify edges where the cache is not producing bit-identical
    /// coordinates (e.g., STEP edges without step_id).
    pub fn bit_exact() -> Self {
        Self {
            exact: HashMap::new(),
            spatial: HashMap::new(),
            tolerance: 0.0,
            use_tolerance: false,
            exact_hits: Cell::new(0),
            tolerance_hits: Cell::new(0),
            misses: Cell::new(0),
        }
    }

    /// Create a new dedup map with tolerance-based fallback.
    /// Use this when some edges lack step_id (step_id==0) and their
    /// discretization may produce near-identical but not bit-identical vertices.
    /// The tolerance should be small enough to not collapse distinct features
    /// but large enough to catch edge cache near-misses.
    /// Recommended: model_scale * 1e-6 (1 PPM) for production,
    ///              model_scale * 1e-4 (100 PPM) as diagnostic.
    pub fn with_tolerance(tolerance: f64) -> Self {
        Self {
            exact: HashMap::new(),
            spatial: HashMap::new(),
            tolerance: tolerance.max(1e-15),
            use_tolerance: tolerance > 0.0,
            exact_hits: Cell::new(0),
            tolerance_hits: Cell::new(0),
            misses: Cell::new(0),
        }
    }

    /// Look up a vertex. Returns Some(index) if found, None otherwise.
    /// First tries bit-exact match, then tolerance-based spatial lookup.
    /// Tracks hit statistics: exact_hits (bit-identical), tolerance_hits (near-miss).
    pub fn get(&self, p: &Point3d) -> Option<u32> {
        // Fast path: bit-exact match
        let key = VertexKey::from_point(p);
        if let Some(&idx) = self.exact.get(&key) {
            self.exact_hits.set(self.exact_hits.get() + 1);
            return Some(idx);
        }

        // Slow path: tolerance-based spatial lookup
        if self.use_tolerance {
            let cell = self.cell_key(p);
            if let Some(candidates) = self.spatial.get(&cell) {
                let tol_sq = self.tolerance * self.tolerance;
                for &(idx, ref vp) in candidates {
                    let dx = p.x - vp.x;
                    let dy = p.y - vp.y;
                    let dz = p.z - vp.z;
                    if dx * dx + dy * dy + dz * dz <= tol_sq {
                        self.tolerance_hits.set(self.tolerance_hits.get() + 1);
                        return Some(idx);
                    }
                }
            }
            // Also check neighboring cells (vertex may be near cell boundary)
            let cx = (p.x / self.tolerance).floor() as i64;
            let cy = (p.y / self.tolerance).floor() as i64;
            let cz = (p.z / self.tolerance).floor() as i64;
            for dx in -1i64..=1 {
                for dy in -1i64..=1 {
                    for dz in -1i64..=1 {
                        if dx == 0 && dy == 0 && dz == 0 { continue; }
                        let neighbor = (cx + dx, cy + dy, cz + dz);
                        if let Some(candidates) = self.spatial.get(&neighbor) {
                            let tol_sq = self.tolerance * self.tolerance;
                            for &(idx, ref vp) in candidates {
                                let ddx = p.x - vp.x;
                                let ddy = p.y - vp.y;
                                let ddz = p.z - vp.z;
                                if ddx * ddx + ddy * ddy + ddz * ddz <= tol_sq {
                                    self.tolerance_hits.set(self.tolerance_hits.get() + 1);
                                    return Some(idx);
                                }
                            }
                        }
                    }
                }
            }
        }

        None
    }

    /// Insert a vertex with its index.
    pub fn insert(&mut self, p: &Point3d, idx: u32) {
        let key = VertexKey::from_point(p);
        self.exact.insert(key, idx);
        self.misses.set(self.misses.get() + 1);

        if self.use_tolerance {
            let cell = self.cell_key(p);
            self.spatial.entry(cell).or_default().push((idx, *p));
        }
    }

    /// Compute spatial hash cell key for a point.
    #[inline]
    fn cell_key(&self, p: &Point3d) -> (i64, i64, i64) {
        (
            (p.x / self.tolerance).floor() as i64,
            (p.y / self.tolerance).floor() as i64,
            (p.z / self.tolerance).floor() as i64,
        )
    }

    /// Return dedup statistics: (exact_hits, tolerance_hits, misses).
    /// - exact_hits: bit-identical matches (edge cache working correctly)
    /// - tolerance_hits: near-miss matches (non-cached edges or FP drift)
    /// - misses: new vertex insertions
    pub fn stats(&self) -> (usize, usize, usize) {
        (self.exact_hits.get(), self.tolerance_hits.get(), self.misses.get())
    }

    /// Returns the number of bit-exact entries.
    pub fn len(&self) -> usize {
        self.exact.len()
    }

    /// Returns whether the map is empty.
    pub fn is_empty(&self) -> bool {
        self.exact.is_empty()
    }
}

/// A 3D triangle mesh.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TriangleMesh {
    /// Vertex positions.
    pub vertices: Vec<Point3d>,
    /// Triangle indices (3 vertex indices per triangle).
    pub triangles: Vec<[u32; 3]>,
    /// Optional vertex normals.
    pub normals: Option<Vec<[f64; 3]>>,
    /// Optional triangle normals.
    pub face_normals: Option<Vec<[f64; 3]>>,
    /// Optional per-triangle RGBA colors (0..1 range).
    pub triangle_colors: Option<Vec<[f32; 4]>>,
    /// Optional per-triangle face ID (TopoId of the source BRep face).
    /// Used for selection, highlighting, and UV grid display.
    pub triangle_face_ids: Option<Vec<u64>>,
}

impl TriangleMesh {
    pub fn new() -> Self {
        Self {
            vertices: Vec::new(),
            triangles: Vec::new(),
            normals: None,
            face_normals: None,
            triangle_colors: None,
            triangle_face_ids: None,
        }
    }

    /// Create from vertices and triangle indices.
    pub fn from_data(vertices: Vec<Point3d>, triangles: Vec<[u32; 3]>) -> Self {
        Self {
            vertices,
            triangles,
            normals: None,
            face_normals: None,
            triangle_colors: None,
            triangle_face_ids: None,
        }
    }

    /// Add a vertex and return its index.
    pub fn add_vertex(&mut self, p: Point3d) -> u32 {
        let idx = self.vertices.len() as u32;
        self.vertices.push(p);
        idx
    }

    /// Add a vertex normal. Call after add_vertex with the returned index.
    pub fn add_vertex_normal(&mut self, _idx: u32, normal: [f64; 3]) {
        if self.normals.is_none() {
            self.normals = Some(vec![[0.0, 0.0, 1.0]; self.vertices.len() - 1]);
        }
        if let Some(ref mut normals) = self.normals {
            normals.push(normal);
        }
    }

    /// Add a triangle.
    pub fn add_triangle(&mut self, i: u32, j: u32, k: u32) {
        self.triangles.push([i, j, k]);
    }

    /// Add a triangle from an array of 3 vertex indices (convenience overload).
    pub fn add_triangle_arr(&mut self, tri: [u32; 3]) {
        self.triangles.push(tri);
    }

    /// Number of vertices.
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    /// Number of triangles.
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    /// Remove duplicate triangles (triangles with the same 3 vertex indices,
    /// regardless of order). Duplicate triangles create non-manifold edges
    /// (each edge of the duplicate adds an extra face).
    ///
    /// Returns the number of duplicates removed.
    /// Also filters degenerate triangles (where two vertex indices are equal).
    pub fn remove_duplicate_triangles(&mut self) -> usize {
        use std::collections::HashMap;
        let old_len = self.triangles.len();
        let old_triangles = std::mem::take(&mut self.triangles);
        let old_face_ids = self.triangle_face_ids.take();
        let old_face_normals = self.face_normals.take();
        let old_triangle_colors = self.triangle_colors.take();

        // Face-aware dedup: track (sorted_key → face_id) so that triangles
        // from *different* faces sharing the same vertex indices are BOTH
        // kept.  Only same-face duplicates are removed.  This prevents holes
        // in the 3D view (e.g., Step#87 plane face, Step#78 cone face) that
        // were caused by aggressively removing cross-face duplicates.
        let mut seen: HashMap<[u32; 3], u64> = HashMap::with_capacity(old_len);
        let mut removed = 0usize;
        let mut cross_face_kept = 0usize;

        for (i, tri) in old_triangles.iter().enumerate() {
            // Skip degenerate triangles (two equal indices)
            if tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2] {
                removed += 1;
                continue;
            }
            // Canonical sorted key so (a,b,c) and (c,b,a) match
            let mut key = [tri[0], tri[1], tri[2]];
            key.sort_unstable();
            let fid = old_face_ids.as_ref().and_then(|ids| ids.get(i)).copied().unwrap_or(u64::MAX);
            if let Some(&existing_fid) = seen.get(&key) {
                if existing_fid == fid {
                    // Same face: true duplicate, remove it
                    removed += 1;
                    continue;
                } else {
                    // Different faces: cross-face duplicate, KEEP to avoid holes
                    cross_face_kept += 1;
                }
            } else {
                seen.insert(key, fid);
            }
            self.triangles.push(*tri);
            // Move per-triangle attributes in sync (use get() to handle
            // any length mismatch from prior filter_degenerate_triangles)
            if let Some(ref src) = old_face_ids {
                if let Some(&fid) = src.get(i) {
                    self.triangle_face_ids.get_or_insert_with(Vec::new).push(fid);
                }
            }
            if let Some(ref src) = old_face_normals {
                if let Some(&n) = src.get(i) {
                    self.face_normals.get_or_insert_with(Vec::new).push(n);
                }
            }
            if let Some(ref src) = old_triangle_colors {
                if let Some(&col) = src.get(i) {
                    self.triangle_colors.get_or_insert_with(Vec::new).push(col);
                }
            }
        }
        if cross_face_kept > 0 {
            log::debug!(
                "remove_duplicate_triangles: kept {} cross-face duplicates (different faces sharing same vertices)",
                cross_face_kept
            );
        }
        let _ = old_len;
        removed
    }

    /// Fill boundary edges by adding triangles that connect boundary vertices
    /// to nearby interior vertices.
    ///
    /// After merging face meshes, some edges may have count=1 (boundary) even
    /// though they should be shared between adjacent faces. This happens when
    /// one face's triangulation doesn't create a triangle using a particular
    /// rim edge. This function finds such edges and adds fill triangles.
    ///
    /// Algorithm:
    /// 1. Find all boundary edges (count=1)
    /// 2. For each boundary edge (va, vb), find the best vertex vc:
    ///    - vc must be connected to either va or vb (share an existing edge)
    ///    - vc should be on the same side as the missing triangle
    ///    - The triangle (va, vb, vc) must not be degenerate
    /// 3. Add the fill triangle
    ///
    /// Returns the number of fill triangles added.
    pub fn fill_boundary_edges(&mut self, max_fill: usize) -> usize {
        use std::collections::HashMap;
        if self.vertices.is_empty() || self.triangles.is_empty() {
            return 0;
        }

        let mut filled = 0usize;
        for _iteration in 0..5 {
            // Build edge → list of (triangle_index, edge_orientation) map
            // edge_orientation: +1 if triangle has edge as (a, b), -1 if (b, a)
            // This tells us which side of the edge the triangle is on.
            let mut edge_info: HashMap<(u32, u32), Vec<(usize, i32)>> = HashMap::new();
            for (ti, tri) in self.triangles.iter().enumerate() {
                if tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2] {
                    continue;
                }
                for k in 0..3 {
                    let a = tri[k];
                    let b = tri[(k + 1) % 3];
                    let key = (a.min(b), a.max(b));
                    // +1 if a < b (edge goes a→b in CCW order), -1 if reversed
                    let orient = if a < b { 1 } else { -1 };
                    edge_info.entry(key).or_default().push((ti, orient));
                }
            }

            // Find boundary edges (used by exactly 1 triangle)
            let boundary_edges: Vec<(u32, u32)> = edge_info.iter()
                .filter(|(_, tris)| tris.len() == 1)
                .map(|(&(a, b), _)| (a, b))
                .collect();

            if boundary_edges.is_empty() {
                break;
            }

            if filled == 0 {
                log::info!(
                    "fill_boundary_edges: iteration starting with {} boundary edges ({} tris, {} verts)",
                    boundary_edges.len(), self.triangles.len(), self.vertices.len(),
                );
            }

            // Build vertex → neighbors map
            let mut vertex_neighbors: HashMap<u32, std::collections::HashSet<u32>> = HashMap::new();
            for tri in &self.triangles {
                if tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2] {
                    continue;
                }
                for k in 0..3 {
                    let a = tri[k];
                    let b = tri[(k + 1) % 3];
                    vertex_neighbors.entry(a).or_default().insert(b);
                    vertex_neighbors.entry(b).or_default().insert(a);
                }
            }

            let mut added_this_iter = 0usize;
            let mut no_common_neighbor = 0usize;
            let mut no_opposite_normal = 0usize;
            let mut already_exists = 0usize;
            let mut degenerate = 0usize;

            // Build a spatial hash for efficient nearest-vertex queries.
            // This allows the fill function to find vertices from ADJACENT faces
            // (which don't share edges with the current face) to use as fill candidates.
            //
            // The cell size is based on the model's bounding box diagonal,
            // ensuring we get a reasonable number of vertices per cell.
            let bbox_min = self.vertices.iter().fold(
                [f64::INFINITY; 3], |acc, v| [
                    acc[0].min(v.x), acc[1].min(v.y), acc[2].min(v.z),
                ]);
            let bbox_max = self.vertices.iter().fold(
                [f64::NEG_INFINITY; 3], |acc, v| [
                    acc[0].max(v.x), acc[1].max(v.y), acc[2].max(v.z),
                ]);
            let diagonal = ((bbox_max[0] - bbox_min[0]).powi(2)
                + (bbox_max[1] - bbox_min[1]).powi(2)
                + (bbox_max[2] - bbox_min[2]).powi(2)).sqrt();
            // Cell size: 5% of diagonal — small enough to find nearby vertices,
            // large enough to keep the hash table small.
            let cell_size = (diagonal * 0.05).max(1e-6);
            let mut spatial: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
            for (i, v) in self.vertices.iter().enumerate() {
                let key = (
                    (v.x / cell_size).floor() as i64,
                    (v.y / cell_size).floor() as i64,
                    (v.z / cell_size).floor() as i64,
                );
                spatial.entry(key).or_default().push(i as u32);
            }

            for &(va, vb) in &boundary_edges {
                if filled + added_this_iter >= max_fill {
                    break;
                }

                // Get the existing triangle that uses this edge
                let key = (va.min(vb), va.max(vb));
                let existing = match edge_info.get(&key) {
                    Some(v) if v.len() == 1 => v[0],
                    _ => continue,
                };
                let (existing_ti, existing_orient) = existing;
                let existing_tri = self.triangles[existing_ti];

                // Find the third vertex of the existing triangle
                let third = existing_tri.iter().copied().find(|&v| v != va && v != vb).unwrap_or(va);
                let p_third = self.vertices[third as usize];

                // Compute the normal of the existing triangle
                let pa = self.vertices[va as usize];
                let pb = self.vertices[vb as usize];
                let existing_normal = {
                    let e1 = (pb.x - pa.x, pb.y - pa.y, pb.z - pa.z);
                    let e2 = (p_third.x - pa.x, p_third.y - pa.y, p_third.z - pa.z);
                    (e1.1 * e2.2 - e1.2 * e2.1,
                     e1.2 * e2.0 - e1.0 * e2.2,
                     e1.0 * e2.1 - e1.1 * e2.0)
                };

                // Find candidate fill vertices.
                // Strategy: search the spatial hash for vertices near the midpoint
                // of the boundary edge. These are likely from the adjacent face.
                let mid = Point3d::new(
                    (pa.x + pb.x) * 0.5,
                    (pa.y + pb.y) * 0.5,
                    (pa.z + pb.z) * 0.5,
                );
                let mid_key = (
                    (mid.x / cell_size).floor() as i64,
                    (mid.y / cell_size).floor() as i64,
                    (mid.z / cell_size).floor() as i64,
                );

                // Search a 5x5x5 neighborhood of cells around the midpoint
                let mut candidates: Vec<u32> = Vec::new();
                for dx in -2i64..=2 {
                    for dy in -2i64..=2 {
                        for dz in -2i64..=2 {
                            let nk = (mid_key.0 + dx, mid_key.1 + dy, mid_key.2 + dz);
                            if let Some(verts) = spatial.get(&nk) {
                                candidates.extend(verts.iter().copied());
                            }
                        }
                    }
                }

                // Filter candidates: exclude va, vb, and the existing third vertex
                candidates.retain(|&v| v != va && v != vb && v != third);

                if candidates.is_empty() {
                    no_common_neighbor += 1;
                    continue;
                }

                // Choose the vc whose triangle (va, vb, vc) has normal OPPOSITE
                // to the existing triangle's normal AND is closest to the edge midpoint.
                let mut best_vc: Option<u32> = None;
                let mut best_score = f64::NEG_INFINITY; // Higher score = better

                for &vc in &candidates {
                    let pc = self.vertices[vc as usize];
                    let fill_normal = {
                        let e1 = (pb.x - pa.x, pb.y - pa.y, pb.z - pa.z);
                        let e2 = (pc.x - pa.x, pc.y - pa.y, pc.z - pa.z);
                        (e1.1 * e2.2 - e1.2 * e2.1,
                         e1.2 * e2.0 - e1.0 * e2.2,
                         e1.0 * e2.1 - e1.1 * e2.0)
                    };
                    // Dot product: if negative, normals are opposite (good)
                    let dot = existing_normal.0 * fill_normal.0
                            + existing_normal.1 * fill_normal.1
                            + existing_normal.2 * fill_normal.2;

                    // Only consider candidates with opposite normals
                    if dot >= 0.0 {
                        continue;
                    }

                    // Distance from vc to the edge midpoint (closer = better)
                    let dist_sq = (pc.x - mid.x).powi(2)
                        + (pc.y - mid.y).powi(2)
                        + (pc.z - mid.z).powi(2);

                    // Score: prefer opposite normals (more negative dot) and closer vertices
                    // Use -dot (positive) - dist_sq (normalized)
                    let score = -dot - dist_sq * 0.001; // Weight: normal direction matters more

                    if score > best_score {
                        best_score = score;
                        best_vc = Some(vc);
                    }
                }

                let vc = match best_vc {
                    Some(v) => v,
                    None => {
                        no_opposite_normal += 1;
                        continue;
                    }
                };

                // Check the triangle is not degenerate
                let pc = self.vertices[vc as usize];
                let area = ((pb.x - pa.x) * (pc.y - pa.y) - (pb.y - pa.y) * (pc.x - pa.x)).abs()
                         + ((pb.y - pa.y) * (pc.z - pa.z) - (pb.z - pa.z) * (pc.y - pa.y)).abs()
                         + ((pb.z - pa.z) * (pc.x - pa.x) - (pb.x - pa.x) * (pc.z - pa.z)).abs();
                if area < 1e-20 {
                    degenerate += 1;
                    continue;
                }

                // Check the triangle doesn't already exist
                let mut sorted = [va, vb, vc];
                sorted.sort();
                let exists = self.triangles.iter().any(|t| {
                    let mut s = [t[0], t[1], t[2]];
                    s.sort();
                    s == sorted
                });
                if exists {
                    already_exists += 1;
                    continue;
                }

                // Add the fill triangle with correct orientation.
                if existing_orient > 0 {
                    self.triangles.push([vb, va, vc]);
                } else {
                    self.triangles.push([va, vb, vc]);
                }
                // Update neighbor maps
                vertex_neighbors.entry(va).or_default().insert(vb);
                vertex_neighbors.entry(vb).or_default().insert(va);
                vertex_neighbors.entry(va).or_default().insert(vc);
                vertex_neighbors.entry(vc).or_default().insert(va);
                vertex_neighbors.entry(vb).or_default().insert(vc);
                vertex_neighbors.entry(vc).or_default().insert(vb);
                added_this_iter += 1;
            }

            log::info!(
                "fill_boundary_edges: iter added {} (no_common={}, no_opposite={}, exists={}, degen={})",
                added_this_iter, no_common_neighbor, no_opposite_normal, already_exists, degenerate,
            );

            filled += added_this_iter;
            if added_this_iter == 0 {
                break;
            }
        }

        if filled > 0 {
            log::info!(
                "fill_boundary_edges: added {} fill triangles total ({} verts, {} tris)",
                filled, self.vertices.len(), self.triangles.len(),
            );
        }
        filled
    }

    /// Compute face normals.
    pub fn compute_face_normals(&mut self) {
        // Don't overwrite existing face normals — they may have been set
        // analytically (e.g., from plane.normal) and are more accurate
        // than cross-product normals derived from possibly-inconsistent
        // winding order.
        if self.face_normals.is_some() {
            return;
        }
        let mut normals = Vec::with_capacity(self.triangles.len());
        for tri in &self.triangles {
            let v0 = self.vertices[tri[0] as usize];
            let v1 = self.vertices[tri[1] as usize];
            let v2 = self.vertices[tri[2] as usize];

            let e1 = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
            let e2 = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);

            let nx = e1.1 * e2.2 - e1.2 * e2.1;
            let ny = e1.2 * e2.0 - e1.0 * e2.2;
            let nz = e1.0 * e2.1 - e1.1 * e2.0;
            let len = (nx * nx + ny * ny + nz * nz).sqrt();
            if len > 1e-15 {
                normals.push([nx / len, ny / len, nz / len]);
            } else {
                normals.push([0.0, 0.0, 1.0]);
            }
        }
        self.face_normals = Some(normals);
    }

    /// Compute face normals only for triangles that have placeholder [0,0,1]
    /// normals (set during merge when one mesh had face_normals and the other
    /// didn't).  Preserves analytically-set normals (e.g., from plane.normal).
    pub fn fill_missing_face_normals(&mut self) {
        if let Some(ref mut normals) = self.face_normals {
            for (tri_idx, tri) in self.triangles.iter().enumerate() {
                let n = normals[tri_idx];
                // Skip if normal is already properly set (not the placeholder)
                if n[0].abs() > 1e-10 || n[1].abs() > 1e-10 || (n[2] - 1.0).abs() > 1e-10 {
                    continue;
                }
                // Compute from cross product
                let v0 = self.vertices[tri[0] as usize];
                let v1 = self.vertices[tri[1] as usize];
                let v2 = self.vertices[tri[2] as usize];
                let e1 = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
                let e2 = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
                let nx = e1.1 * e2.2 - e1.2 * e2.1;
                let ny = e1.2 * e2.0 - e1.0 * e2.2;
                let nz = e1.0 * e2.1 - e1.1 * e2.0;
                let len = (nx * nx + ny * ny + nz * nz).sqrt();
                if len > 1e-15 {
                    normals[tri_idx] = [nx / len, ny / len, nz / len];
                }
            }
        } else {
            // No face normals at all — compute from scratch
            self.compute_face_normals();
        }
    }

    /// Merge another mesh into this one.
    pub fn merge(&mut self, other: &TriangleMesh) {
        let offset = self.vertices.len() as u32;
        self.vertices.extend(other.vertices.iter().cloned());
        for tri in &other.triangles {
            self.triangles.push([tri[0] + offset, tri[1] + offset, tri[2] + offset]);
        }
        // Merge vertex normals
        match (&mut self.normals, &other.normals) {
            (Some(ref mut self_normals), Some(ref other_normals)) => {
                self_normals.extend(other_normals.iter().cloned());
            }
            (None, Some(ref other_normals)) => {
                // We need to fill in default normals for existing vertices
                let mut combined = vec![[0.0, 0.0, 1.0]; self.vertices.len() - other.vertices.len()];
                combined.extend(other_normals.iter().cloned());
                self.normals = Some(combined);
            }
            _ => {}
        }
        // Merge face normals (per-triangle).
        // Same three-case logic as merge_deduplicating: must keep
        // face_normals.len() == triangles.len() after the merge.
        let other_tri_count = other.triangles.len();
        if self.face_normals.is_none() && other.face_normals.is_some() {
            let existing_count = self.triangles.len().saturating_sub(other_tri_count);
            self.face_normals = Some(vec![[0.0, 0.0, 1.0]; existing_count]);
        }
        match (&mut self.face_normals, &other.face_normals) {
            (Some(ref mut dest), Some(ref src)) => {
                dest.extend(src.iter().cloned());
            }
            (Some(ref mut dest), None) => {
                for _ in 0..other_tri_count {
                    dest.push([0.0, 0.0, 1.0]);
                }
            }
            _ => {}
        }
        // Merge triangle colors (per-triangle)
        if self.triangle_colors.is_none() && other.triangle_colors.is_some() {
            let existing_count = self.triangles.len().saturating_sub(other_tri_count);
            self.triangle_colors = Some(vec![[0.62, 0.65, 0.70, 1.0]; existing_count]);
        }
        match (&mut self.triangle_colors, &other.triangle_colors) {
            (Some(ref mut dest), Some(ref src)) => {
                dest.extend(src.iter().cloned());
            }
            (Some(ref mut dest), None) => {
                for _ in 0..other_tri_count {
                    dest.push([0.62, 0.65, 0.70, 1.0]);
                }
            }
            _ => {}
        }
        // Merge face IDs
        if self.triangle_face_ids.is_none() && other.triangle_face_ids.is_some() {
            let existing_count = self.triangles.len().saturating_sub(other_tri_count);
            self.triangle_face_ids = Some(vec![0; existing_count]);
        }
        match (&mut self.triangle_face_ids, &other.triangle_face_ids) {
            (Some(ref mut ids), Some(ref other_ids)) => {
                ids.extend(other_ids.iter().cloned());
            }
            (Some(ref mut ids), None) => {
                for _ in 0..other_tri_count {
                    ids.push(0);
                }
            }
            _ => {}
        }
    }

    /// Merge another mesh into this one with vertex deduplication.
    ///
    /// This is the **topology-first** merge: when two faces share an edge,
    /// their boundary vertices have identical 3D coordinates (guaranteed by
    /// the `EdgeDiscretizationCache` with deterministic rounding). Instead of
    /// blindly appending all vertices with an offset (like `merge()`), this
    /// method reuses existing vertex indices for points that are already
    /// present in the accumulated mesh.
    ///
    /// # How it works
    ///
    /// 1. For each vertex in `other`, compute a `VertexKey` from its bit-exact
    ///    coordinates.
    /// 2. If the key exists in `dedup_map`, reuse the existing vertex index.
    /// 3. Otherwise, add the vertex and store its index in `dedup_map`.
    /// 4. Remap triangle indices from `other`'s local indices to the
    ///    (deduplicated) global indices.
    ///
    /// # Arguments
    ///
    /// * `other` — The per-face mesh to merge in.
    /// * `dedup_map` — A mutable vertex deduplication map that persists
    ///   across calls. Must be created once before the first call and reused
    ///   for all subsequent face merges in the same solid.
    ///
    /// # Why this matters
    ///
    /// Without deduplication, shared-edge vertices get different indices in
    /// the final mesh, producing boundary edges. The mesh is NOT watertight
    /// even though the edge cache guarantees bit-identical 3D coordinates.
    /// With deduplication, shared vertices get the same index, making the
    /// mesh watertight **by construction** — no post-hoc repair needed.
    pub fn merge_deduplicating(&mut self, other: &TriangleMesh, dedup_map: &mut VertexDedupMap) {
        // Save pre-merge counts for correct normals/triangle-attribute sizing
        let old_vertex_count = self.vertices.len();
        let _old_triangle_count = self.triangles.len();

        // Build index remapping: other's local vertex index → global index
        let mut index_map: Vec<u32> = Vec::with_capacity(other.vertices.len());
        let mut reuse_count = 0usize;
        let mut new_count = 0usize;

        for vertex in &other.vertices {
            if let Some(existing_idx) = dedup_map.get(vertex) {
                // Vertex already exists — reuse its index
                index_map.push(existing_idx);
                reuse_count += 1;
            } else {
                // New vertex — add to mesh and record in dedup map
                let new_idx = self.vertices.len() as u32;
                self.vertices.push(*vertex);
                dedup_map.insert(vertex, new_idx);
                index_map.push(new_idx);
                new_count += 1;
            }
        }

        // Add triangles with remapped indices, filtering out:
        // - Degenerate triangles (a==b or b==c or a==c) — they contribute
        //   phantom edges to the edge map.
        // - Duplicate triangles (same 3 vertex indices as an existing triangle,
        //   in any order) — they create non-manifold edges (count=3+).
        //
        // We track which source triangles are kept (via kept_src_indices) so
        // that the per-triangle attribute arrays (face_normals, triangle_colors,
        // triangle_face_ids) stay aligned with self.triangles.
        let mut became_degenerate = 0usize;
        let mut duplicate_count = 0usize;
        let mut cross_face_kept = 0usize;
        let mut kept_src_indices: Vec<usize> = Vec::with_capacity(other.triangles.len());
        {
            // Face-aware duplicate tracking: keyed by (sorted_indices → face_id).
            // Cross-face duplicates (same vertex indices, different face) are KEPT
            // because removing them creates visible holes in the 3D view
            // (e.g., Step#87 plane face, Step#78 cone face).  Only same-face
            // duplicates are removed.
            let mut existing_tris: std::collections::HashMap<[u32; 3], u64> = std::collections::HashMap::with_capacity(self.triangles.len());
            // Get the face_id for other's triangles — per-face meshes set all
            // triangle_face_ids to the same value (the face's ID).
            let other_face_id = other.triangle_face_ids.as_ref()
                .and_then(|ids| ids.first().copied())
                .unwrap_or(u64::MAX);
            for (ti, tri) in self.triangles.iter().enumerate() {
                let mut sorted = [tri[0], tri[1], tri[2]];
                sorted.sort();
                let fid = self.triangle_face_ids.as_ref()
                    .and_then(|ids| ids.get(ti).copied())
                    .unwrap_or(u64::MAX);
                existing_tris.entry(sorted).or_insert(fid);
            }

            for (src_idx, tri) in other.triangles.iter().enumerate() {
                let a = index_map[tri[0] as usize];
                let b = index_map[tri[1] as usize];
                let c = index_map[tri[2] as usize];
                if a == b || b == c || a == c {
                    became_degenerate += 1;
                    continue;
                }
                // Face-aware duplicate check
                let mut sorted = [a, b, c];
                sorted.sort();
                if let Some(&existing_fid) = existing_tris.get(&sorted) {
                    if existing_fid == other_face_id {
                        // Same face: true duplicate, remove it
                        duplicate_count += 1;
                        continue;
                    } else {
                        // Different faces: cross-face duplicate, KEEP to avoid holes
                        cross_face_kept += 1;
                    }
                } else {
                    existing_tris.insert(sorted, other_face_id);
                }
                self.triangles.push([a, b, c]);
                kept_src_indices.push(src_idx);
            }
        }
        if became_degenerate > 0 || duplicate_count > 0 || cross_face_kept > 0 {
            log::warn!(
                "MERGE_DEGEN: other has {} verts ({} new, {} reused), {} tris — {} degenerate, {} same-face dup skipped, {} cross-face dup kept",
                other.vertices.len(), new_count, reuse_count, other.triangles.len(), became_degenerate, duplicate_count, cross_face_kept,
            );
        }

        // Handle vertex normals: when deduplicating, the first face's normal
        // wins for shared vertices. This is acceptable because normals are
        // later smoothed by `smooth_normals_adaptive` which computes proper
        // averaged normals across shared edges.
        //
        // When `other` lacks vertex normals, we derive them from the face
        // normals of the triangles in `other` (per-triangle normal assigned
        // to each vertex of that triangle). This avoids leaving vertices
        // with (0,0,1) defaults that cause incorrect smooth shading.
        match (&mut self.normals, &other.normals) {
            (Some(ref mut self_normals), Some(ref other_normals)) => {
                // For new vertices, add their normals. For reused vertices,
                // skip (keep the first face's normal).
                for (i, _vertex) in other.vertices.iter().enumerate() {
                    let global_idx = index_map[i] as usize;
                    // Only add normal if this is a new vertex
                    if global_idx >= self_normals.len() {
                        // Bounds check: other_normals might be shorter than
                        // other.vertices if a face mesh was built without
                        // calling add_vertex_normal for every add_vertex.
                        // Use a default normal in that case.
                        let n = other_normals.get(i).copied().unwrap_or([0.0, 0.0, 1.0]);
                        self_normals.push(n);
                    }
                }
            }
            (Some(ref mut self_normals), None) => {
                // `other` has no vertex normals — derive from face geometry.
                // Build a per-vertex normal by averaging face normals of
                // all triangles in `other` that reference that vertex.
                let mut vertex_face_normals: Vec<Vec<[f64; 3]>> = vec![Vec::new(); other.vertices.len()];
                for &src_idx in &kept_src_indices {
                    let tri = other.triangles[src_idx];
                    // Compute face normal from cross product
                    let v0 = other.vertices[tri[0] as usize];
                    let v1 = other.vertices[tri[1] as usize];
                    let v2 = other.vertices[tri[2] as usize];
                    let e1 = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
                    let e2 = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
                    let nx = e1.1 * e2.2 - e1.2 * e2.1;
                    let ny = e1.2 * e2.0 - e1.0 * e2.2;
                    let nz = e1.0 * e2.1 - e1.1 * e2.0;
                    let len = (nx * nx + ny * ny + nz * nz).sqrt();
                    let fn_normal = if len > 1e-15 {
                        [nx / len, ny / len, nz / len]
                    } else {
                        [0.0, 0.0, 1.0]
                    };
                    vertex_face_normals[tri[0] as usize].push(fn_normal);
                    vertex_face_normals[tri[1] as usize].push(fn_normal);
                    vertex_face_normals[tri[2] as usize].push(fn_normal);
                }
                for (i, _vertex) in other.vertices.iter().enumerate() {
                    let global_idx = index_map[i] as usize;
                    if global_idx >= self_normals.len() {
                        // Average face normals for this vertex
                        let fns = &vertex_face_normals[i];
                        let n = if fns.is_empty() {
                            [0.0, 0.0, 1.0]
                        } else {
                            let mut sum = [0.0_f64; 3];
                            for &f in fns {
                                sum[0] += f[0];
                                sum[1] += f[1];
                                sum[2] += f[2];
                            }
                            let len = (sum[0]*sum[0] + sum[1]*sum[1] + sum[2]*sum[2]).sqrt();
                            if len > 1e-15 {
                                [sum[0]/len, sum[1]/len, sum[2]/len]
                            } else {
                                [0.0, 0.0, 1.0]
                            }
                        };
                        self_normals.push(n);
                    }
                }
            }
            (None, Some(ref other_normals)) => {
                // Fill default normals for pre-existing vertices that don't
                // have normals yet, then add normals for new vertices.
                // Use old_vertex_count (saved before merge) to avoid underflow.
                // Derive normals from face geometry for pre-existing vertices.
                let mut combined: Vec<[f64; 3]> = Vec::with_capacity(self.vertices.len());
                // Compute per-vertex normals for existing mesh by averaging face normals
                let mut existing_vfn: Vec<Vec<[f64; 3]>> = vec![Vec::new(); old_vertex_count];
                for tri in &self.triangles {
                    let v0 = self.vertices[tri[0] as usize];
                    let v1 = self.vertices[tri[1] as usize];
                    let v2 = self.vertices[tri[2] as usize];
                    let e1 = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
                    let e2 = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
                    let nx = e1.1 * e2.2 - e1.2 * e2.1;
                    let ny = e1.2 * e2.0 - e1.0 * e2.2;
                    let nz = e1.0 * e2.1 - e1.1 * e2.0;
                    let len = (nx * nx + ny * ny + nz * nz).sqrt();
                    let fn_n = if len > 1e-15 { [nx/len, ny/len, nz/len] } else { [0.0, 0.0, 1.0] };
                    if (tri[0] as usize) < old_vertex_count { existing_vfn[tri[0] as usize].push(fn_n); }
                    if (tri[1] as usize) < old_vertex_count { existing_vfn[tri[1] as usize].push(fn_n); }
                    if (tri[2] as usize) < old_vertex_count { existing_vfn[tri[2] as usize].push(fn_n); }
                }
                for i in 0..old_vertex_count {
                    let fns = &existing_vfn[i];
                    if fns.is_empty() {
                        combined.push([0.0, 0.0, 1.0]);
                    } else {
                        let mut sum = [0.0_f64; 3];
                        for &f in fns { sum[0] += f[0]; sum[1] += f[1]; sum[2] += f[2]; }
                        let len = (sum[0]*sum[0] + sum[1]*sum[1] + sum[2]*sum[2]).sqrt();
                        if len > 1e-15 { combined.push([sum[0]/len, sum[1]/len, sum[2]/len]); }
                        else { combined.push([0.0, 0.0, 1.0]); }
                    }
                }
                for (i, _vertex) in other.vertices.iter().enumerate() {
                    let global_idx = index_map[i] as usize;
                    if global_idx >= combined.len() {
                        let n = other_normals.get(i).copied().unwrap_or([0.0, 0.0, 1.0]);
                        combined.push(n);
                    }
                }
                self.normals = Some(combined);
            }
            _ => {}
        }

        // Merge face normals (per-triangle, only for kept triangles).
        // We must keep face_normals.len() == triangles.len() after the merge.
        // Three cases:
        //   (None, Some) → seed self with defaults for pre-existing triangles,
        //                  then copy other's kept triangles.
        //   (Some, Some) → copy other's kept triangles.
        //   (Some, None) → push defaults for the new triangles from `other`
        //                  so the array stays in sync with self.triangles.
        //   (None, None) → nothing to do.
        let kept_len = kept_src_indices.len();
        if self.face_normals.is_none() && other.face_normals.is_some() {
            // Compute face normals for pre-existing triangles from geometry,
            // instead of using wrong (0,0,1) defaults.
            let existing_count = self.triangles.len().saturating_sub(kept_len);
            let mut init_normals = Vec::with_capacity(existing_count);
            for i in 0..existing_count {
                let tri = self.triangles[i];
                let v0 = self.vertices[tri[0] as usize];
                let v1 = self.vertices[tri[1] as usize];
                let v2 = self.vertices[tri[2] as usize];
                let e1 = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
                let e2 = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
                let nx = e1.1 * e2.2 - e1.2 * e2.1;
                let ny = e1.2 * e2.0 - e1.0 * e2.2;
                let nz = e1.0 * e2.1 - e1.1 * e2.0;
                let len = (nx * nx + ny * ny + nz * nz).sqrt();
                if len > 1e-15 {
                    init_normals.push([nx / len, ny / len, nz / len]);
                } else {
                    init_normals.push([0.0, 0.0, 1.0]);
                }
            }
            self.face_normals = Some(init_normals);
        } else if self.face_normals.is_none() && other.face_normals.is_none() && kept_len > 0 {
            // Both self and other lack face normals — compute from geometry
            // for ALL triangles (existing + newly-added).
            let total_count = self.triangles.len();
            let mut all_normals = Vec::with_capacity(total_count);
            for i in 0..total_count {
                let tri = self.triangles[i];
                let v0 = self.vertices[tri[0] as usize];
                let v1 = self.vertices[tri[1] as usize];
                let v2 = self.vertices[tri[2] as usize];
                let e1 = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
                let e2 = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
                let nx = e1.1 * e2.2 - e1.2 * e2.1;
                let ny = e1.2 * e2.0 - e1.0 * e2.2;
                let nz = e1.0 * e2.1 - e1.1 * e2.0;
                let len = (nx * nx + ny * ny + nz * nz).sqrt();
                if len > 1e-15 {
                    all_normals.push([nx / len, ny / len, nz / len]);
                } else {
                    all_normals.push([0.0, 0.0, 1.0]);
                }
            }
            self.face_normals = Some(all_normals);
        }
        match (&mut self.face_normals, &other.face_normals) {
            (Some(ref mut dest), Some(ref src)) => {
                for &src_idx in &kept_src_indices {
                    dest.push(src[src_idx]);
                }
            }
            (Some(ref mut dest), None) => {
                // `other` has no face normals — compute them from the
                // triangle geometry instead of using a wrong (0,0,1) default.
                // This is critical for correct lighting: structured grid
                // triangulation functions (cone tube, cylinder tube, etc.)
                // don't set face_normals, so the merge must derive them.
                for &src_idx in &kept_src_indices {
                    let tri = other.triangles[src_idx];
                    let v0 = other.vertices[tri[0] as usize];
                    let v1 = other.vertices[tri[1] as usize];
                    let v2 = other.vertices[tri[2] as usize];
                    let e1 = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
                    let e2 = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
                    let nx = e1.1 * e2.2 - e1.2 * e2.1;
                    let ny = e1.2 * e2.0 - e1.0 * e2.2;
                    let nz = e1.0 * e2.1 - e1.1 * e2.0;
                    let len = (nx * nx + ny * ny + nz * nz).sqrt();
                    if len > 1e-15 {
                        dest.push([nx / len, ny / len, nz / len]);
                    } else {
                        dest.push([0.0, 0.0, 1.0]);
                    }
                }
            }
            _ => {}
        }

        // Merge triangle colors (per-triangle, only for kept triangles)
        if self.triangle_colors.is_none() && other.triangle_colors.is_some() {
            let existing_count = self.triangles.len().saturating_sub(kept_len);
            self.triangle_colors = Some(vec![[0.62, 0.65, 0.70, 1.0]; existing_count]);
        }
        match (&mut self.triangle_colors, &other.triangle_colors) {
            (Some(ref mut dest), Some(ref src)) => {
                for &src_idx in &kept_src_indices {
                    dest.push(src[src_idx]);
                }
            }
            (Some(ref mut dest), None) => {
                for _ in 0..kept_len {
                    dest.push([0.62, 0.65, 0.70, 1.0]);
                }
            }
            _ => {}
        }

        // Merge face IDs (per-triangle, only for kept triangles)
        if self.triangle_face_ids.is_none() && other.triangle_face_ids.is_some() {
            let existing_count = self.triangles.len().saturating_sub(kept_len);
            self.triangle_face_ids = Some(vec![0; existing_count]);
        }
        match (&mut self.triangle_face_ids, &other.triangle_face_ids) {
            (Some(ref mut ids), Some(ref other_ids)) => {
                for &src_idx in &kept_src_indices {
                    ids.push(other_ids[src_idx]);
                }
            }
            (Some(ref mut ids), None) => {
                for _ in 0..kept_len {
                    ids.push(0);
                }
            }
            _ => {}
        }
    }

    /// Merge another mesh with a uniform color applied to all its triangles.
    pub fn merge_with_color(&mut self, other: &TriangleMesh, color: [f32; 4]) {
        let offset = self.vertices.len() as u32;
        self.vertices.extend(other.vertices.iter().cloned());
        for tri in &other.triangles {
            self.triangles.push([tri[0] + offset, tri[1] + offset, tri[2] + offset]);
        }
        if self.triangle_colors.is_none() {
            let existing_count = self.triangles.len() - other.triangles.len();
            self.triangle_colors = Some(vec![[0.62, 0.65, 0.70, 1.0]; existing_count]);
        }
        if let Some(ref mut colors) = self.triangle_colors {
            for _ in 0..other.triangles.len() {
                colors.push(color);
            }
        }
        // Merge vertex normals
        match (&mut self.normals, &other.normals) {
            (Some(ref mut self_normals), Some(ref other_normals)) => {
                self_normals.extend(other_normals.iter().cloned());
            }
            (None, Some(ref other_normals)) => {
                // Fill in default normals for existing vertices
                let mut combined = vec![[0.0, 0.0, 1.0]; self.vertices.len() - other.vertices.len()];
                combined.extend(other_normals.iter().cloned());
                self.normals = Some(combined);
            }
            _ => {}
        }
        // Merge face normals (per-triangle) — same three-case logic as merge()
        let other_tri_count = other.triangles.len();
        if self.face_normals.is_none() && other.face_normals.is_some() {
            let existing_count = self.triangles.len().saturating_sub(other_tri_count);
            self.face_normals = Some(vec![[0.0, 0.0, 1.0]; existing_count]);
        }
        match (&mut self.face_normals, &other.face_normals) {
            (Some(ref mut dest), Some(ref src)) => {
                dest.extend(src.iter().cloned());
            }
            (Some(ref mut dest), None) => {
                for _ in 0..other_tri_count {
                    dest.push([0.0, 0.0, 1.0]);
                }
            }
            _ => {}
        }
        // Merge face IDs
        if self.triangle_face_ids.is_none() && other.triangle_face_ids.is_some() {
            let existing_count = self.triangles.len().saturating_sub(other_tri_count);
            self.triangle_face_ids = Some(vec![0; existing_count]);
        }
        match (&mut self.triangle_face_ids, &other.triangle_face_ids) {
            (Some(ref mut ids), Some(ref other_ids)) => {
                ids.extend(other_ids.iter().cloned());
            }
            (Some(ref mut ids), None) => {
                for _ in 0..other_tri_count {
                    ids.push(0);
                }
            }
            _ => {}
        }
    }

    /// Snap boundary vertices to nearby vertices (shared or boundary).
    ///
    /// After merging face meshes with bit-exact deduplication, some vertices
    /// from different faces may be geometrically close but not bit-identical
    /// (because the STEP file uses different VERTEX_POINT entities for the
    /// same geometric boundary). This method finds such pairs and snaps the
    /// boundary vertex to the nearby vertex, effectively "welding" them
    /// together and reducing the number of boundary edges.
    ///
    /// # Algorithm
    /// 1. Build a spatial hash of ALL vertices
    /// 2. Find boundary edges (edges appearing in only one triangle)
    /// 3. For each boundary vertex, check if there's a nearby vertex
    ///    within `snap_tolerance`
    /// 4. If found, remap the boundary vertex to the nearby one
    ///
    /// Runs multiple iterations because snapping boundary vertices can
    /// create new shared vertices that enable further snapping.
    pub fn snap_boundary_vertices(&mut self, snap_tolerance: f64) -> usize {
        if snap_tolerance <= 0.0 || self.vertices.is_empty() || self.triangles.is_empty() {
            return 0;
        }

        // Diagnostic: count boundary vertices up front
        let mut edge_count_diag: HashMap<(u32, u32), u32> = HashMap::new();
        for tri in &self.triangles {
            if tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2] {
                continue;
            }
            for k in 0..3 {
                let a = tri[k].min(tri[(k + 1) % 3]);
                let b = tri[k].max(tri[(k + 1) % 3]);
                *edge_count_diag.entry((a, b)).or_insert(0) += 1;
            }
        }
        let boundary_edges_diag = edge_count_diag.values().filter(|&&c| c == 1).count();
        let non_manifold_edges_diag = edge_count_diag.values().filter(|&&c| c >= 3).count();
        log::info!(
            "snap_boundary_vertices: tol={:.2e}, {} verts, {} tris, {} boundary edges, {} non-manifold edges",
            snap_tolerance, self.vertices.len(), self.triangles.len(),
            boundary_edges_diag, non_manifold_edges_diag,
        );

        let mut total_snapped = 0usize;
        // Iterate: each round may create new shared vertices that enable
        // further snapping in the next round.
        for iteration in 0..5 {
            let snapped = self.snap_boundary_vertices_once(snap_tolerance);
            total_snapped += snapped;
            log::debug!(
                "snap_boundary_vertices iteration {}: snapped {} vertices (total={})",
                iteration, snapped, total_snapped,
            );
            if snapped == 0 {
                break;
            }
            log::debug!("snap_boundary_vertices iteration {}: snapped {} vertices", iteration, snapped);
        }
        total_snapped
    }

    /// Single iteration of boundary vertex snapping.
    ///
    /// CRITICAL FIX: Only snap boundary→boundary or boundary→shared vertices.
    /// NEVER snap to interior (non-boundary, non-shared) vertices — that
    /// corrupts valid triangulations by collapsing interior structure.
    fn snap_boundary_vertices_once(&mut self, snap_tolerance: f64) -> usize {
        // Step 1: Count edge occurrences to find boundary vertices
        let mut edge_count: HashMap<(u32, u32), u32> = HashMap::new();
        for tri in &self.triangles {
            // Skip degenerate triangles — they contribute phantom edges
            if tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2] {
                continue;
            }
            for k in 0..3 {
                let a = tri[k].min(tri[(k + 1) % 3]);
                let b = tri[k].max(tri[(k + 1) % 3]);
                *edge_count.entry((a, b)).or_insert(0) += 1;
            }
        }

        // Find vertices that are on boundary edges (count==1)
        let mut boundary_vertex_set: std::collections::HashSet<u32> = std::collections::HashSet::new();
        for (&(a, b), &count) in &edge_count {
            if count == 1 {
                boundary_vertex_set.insert(a);
                boundary_vertex_set.insert(b);
            }
        }

        if boundary_vertex_set.is_empty() {
            return 0;
        }

        // Count how many triangles each vertex appears in.
        // Vertices in 2+ triangles are "shared" (interior of an edge between faces).
        // Vertices in 1 triangle only are "leaf" boundary vertices (corners).
        let mut vert_tri_count = vec![0u32; self.vertices.len()];
        for tri in &self.triangles {
            // Skip degenerate triangles
            if tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2] {
                continue;
            }
            for &v in tri {
                vert_tri_count[v as usize] += 1;
            }
        }

        // Step 2: Build spatial hash of BOUNDARY and SHARED vertices only.
        // We DO NOT include pure interior vertices (count >= 2 but not on
        // any boundary edge) as snap targets — that was the bug that
        // corrupted triangulations by snapping boundary vertices to
        // nearby interior vertices.
        //
        // Allowed snap targets:
        //   - Boundary vertices from OTHER faces (the desired case)
        //   - Shared vertices (interior edge junctions — count >= 2)
        // Disallowed snap targets:
        //   - Pure interior vertices that are not on any boundary edge
        let cell_size = snap_tolerance;
        let mut spatial: HashMap<(i64, i64, i64), Vec<(u32, Point3d)>> = HashMap::new();

        // Collect target candidates: boundary vertices + shared vertices
        for (idx, p) in self.vertices.iter().enumerate() {
            let is_boundary = boundary_vertex_set.contains(&(idx as u32));
            let is_shared = vert_tri_count[idx] >= 2;
            if is_boundary || is_shared {
                let cell = (
                    (p.x / cell_size).floor() as i64,
                    (p.y / cell_size).floor() as i64,
                    (p.z / cell_size).floor() as i64,
                );
                spatial.entry(cell).or_default().push((idx as u32, *p));
            }
        }

        // Step 3: For each boundary vertex, find nearby target vertices.
        // Prefer shared vertices (count >= 2) as targets, fall back to
        // other boundary vertices only if no shared vertex is in range.
        let mut remap: Vec<u32> = (0..self.vertices.len() as u32).collect();
        let mut snap_count = 0usize;
        let tol_sq = snap_tolerance * snap_tolerance;

        for &bv in boundary_vertex_set.iter() {
            let p = self.vertices[bv as usize];
            let cell = (
                (p.x / cell_size).floor() as i64,
                (p.y / cell_size).floor() as i64,
                (p.z / cell_size).floor() as i64,
            );

            let mut best_dist_sq_shared = tol_sq;
            let mut best_target_shared: Option<u32> = None;
            let mut best_dist_sq_boundary = tol_sq;
            let mut best_target_boundary: Option<u32> = None;

            // Check current cell and 26 neighbors
            for dx in -1i64..=1 {
                for dy in -1i64..=1 {
                    for dz in -1i64..=1 {
                        let neighbor = (cell.0 + dx, cell.1 + dy, cell.2 + dz);
                        if let Some(candidates) = spatial.get(&neighbor) {
                            for &(idx, ref vp) in candidates {
                                if idx == bv { continue; }
                                let ddx = p.x - vp.x;
                                let ddy = p.y - vp.y;
                                let ddz = p.z - vp.z;
                                let dist_sq = ddx * ddx + ddy * ddy + ddz * ddz;
                                if dist_sq >= tol_sq { continue; }
                                if vert_tri_count[idx as usize] >= 2 {
                                    // Shared vertex target
                                    if dist_sq < best_dist_sq_shared {
                                        best_dist_sq_shared = dist_sq;
                                        best_target_shared = Some(idx);
                                    }
                                } else {
                                    // Boundary-only target
                                    if dist_sq < best_dist_sq_boundary {
                                        best_dist_sq_boundary = dist_sq;
                                        best_target_boundary = Some(idx);
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // Prefer shared target; fall back to boundary target
            let target = best_target_shared.or(best_target_boundary);
            if let Some(target) = target {
                remap[bv as usize] = target;
                snap_count += 1;
            }
        }

        // Step 4: Apply remapping to all triangles
        //
        // NOTE: We do NOT pre-check for degenerate triangles here.
        // Aggressive snapping (snapping all near-miss boundary vertices)
        // produces more watertight results than conservative snapping,
        // even though some triangles become degenerate. The degenerate
        // triangles are filtered out by `filter_degenerate_triangles`
        // in the converter after snapping completes.
        //
        // Pre-checking each remap for degeneration (the "smart snap"
        // approach) was too conservative — it skipped 98% of remaps,
        // leaving most boundary edges unwelded.
        if snap_count > 0 {
            // Resolve chains (if A→B and B→C, then A→C)
            for i in 0..remap.len() {
                let mut current = remap[i];
                let mut seen = std::collections::HashSet::new();
                while remap[current as usize] != current {
                    if !seen.insert(current) { break; } // cycle protection
                    current = remap[current as usize];
                }
                remap[i] = current;
            }

            // Apply remap and track which triangles become degenerate
            let mut degen_count = 0;
            for tri in &mut self.triangles {
                tri[0] = remap[tri[0] as usize];
                tri[1] = remap[tri[1] as usize];
                tri[2] = remap[tri[2] as usize];
                if tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2] {
                    degen_count += 1;
                }
            }
            if degen_count > 0 {
                log::debug!(
                    "snap_boundary_vertices_once: {} triangles became degenerate after snapping {} vertices (will be filtered later)",
                    degen_count, snap_count,
                );
            }
        }

        snap_count
    }

    /// Ensure triangle_colors matches triangles length, filling with default color if needed.
    pub fn ensure_colors(&mut self, default: [f32; 4]) {
        if self.triangle_colors.is_none() {
            self.triangle_colors = Some(vec![default; self.triangles.len()]);
        } else if let Some(ref mut colors) = self.triangle_colors {
            while colors.len() < self.triangles.len() {
                colors.push(default);
            }
        }
    }

    /// Compute bounding box.
    pub fn bounding_box(&self) -> (Point3d, Point3d) {
        if self.vertices.is_empty() {
            return (Point3d::ORIGIN, Point3d::ORIGIN);
        }
        let mut min = self.vertices[0];
        let mut max = self.vertices[0];
        for v in &self.vertices[1..] {
            min.x = min.x.min(v.x);
            min.y = min.y.min(v.y);
            min.z = min.z.min(v.z);
            max.x = max.x.max(v.x);
            max.y = max.y.max(v.y);
            max.z = max.z.max(v.z);
        }
        (min, max)
    }

    /// Total surface area.
    pub fn surface_area(&self) -> f64 {
        let mut area = 0.0;
        for tri in &self.triangles {
            let v0 = self.vertices[tri[0] as usize];
            let v1 = self.vertices[tri[1] as usize];
            let v2 = self.vertices[tri[2] as usize];
            // Cross product of two edges / 2
            let e1x = v1.x - v0.x;
            let e1y = v1.y - v0.y;
            let e1z = v1.z - v0.z;
            let e2x = v2.x - v0.x;
            let e2y = v2.y - v0.y;
            let e2z = v2.z - v0.z;
            let cx = e1y * e2z - e1z * e2y;
            let cy = e1z * e2x - e1x * e2z;
            let cz = e1x * e2y - e1y * e2x;
            area += (cx * cx + cy * cy + cz * cz).sqrt() * 0.5;
        }
        area
    }

    /// Transform all vertices and normals.
    ///
    /// Vertices are transformed by the full 4×4 matrix (including translation).
    /// Normals are transformed by the inverse-transpose of the upper-left 3×3
    /// submatrix — this preserves correct lighting/backface-culling for
    /// non-uniform scaling and reflection transforms.
    pub fn transform(&mut self, m: &[[f64; 4]; 4]) {
        for v in &mut self.vertices {
            *v = v.transform(m);
        }
        // Transform normals by inverse-transpose of 3×3 rotation
        if let Some(ref mut normals) = self.normals {
            let inv_transpose = compute_normal_transform(m);
            for n in normals.iter_mut() {
                let nx = inv_transpose[0][0] * n[0] + inv_transpose[0][1] * n[1] + inv_transpose[0][2] * n[2];
                let ny = inv_transpose[1][0] * n[0] + inv_transpose[1][1] * n[1] + inv_transpose[1][2] * n[2];
                let nz = inv_transpose[2][0] * n[0] + inv_transpose[2][1] * n[1] + inv_transpose[2][2] * n[2];
                let len = (nx * nx + ny * ny + nz * nz).sqrt();
                if len > 1e-15 {
                    *n = [nx / len, ny / len, nz / len];
                }
            }
        }
        // Face normals also need to be transformed
        if let Some(ref mut face_normals) = self.face_normals {
            let inv_transpose = compute_normal_transform(m);
            for n in face_normals.iter_mut() {
                let nx = inv_transpose[0][0] * n[0] + inv_transpose[0][1] * n[1] + inv_transpose[0][2] * n[2];
                let ny = inv_transpose[1][0] * n[0] + inv_transpose[1][1] * n[1] + inv_transpose[1][2] * n[2];
                let nz = inv_transpose[2][0] * n[0] + inv_transpose[2][1] * n[1] + inv_transpose[2][2] * n[2];
                let len = (nx * nx + ny * ny + nz * nz).sqrt();
                if len > 1e-15 {
                    *n = [nx / len, ny / len, nz / len];
                }
            }
        }
    }
}

/// A 2D point for triangulation (in parametric space).
#[derive(Clone, Copy, Debug)]
pub struct Point2dForTriangulation {
    pub x: f64,
    pub y: f64,
    pub original_index: usize,
}

/// Edge constraint for triangulation.
#[derive(Clone, Copy, Debug)]
pub struct ConstraintEdge {
    pub start: usize,
    pub end: usize,
}

/// Compute the inverse-transpose of the upper-left 3×3 submatrix of a 4×4 matrix.
///
/// This is used for transforming normals: if vertices are transformed by M,
/// then normals must be transformed by (M⁻¹)ᵀ to remain correct under
/// non-uniform scaling and reflection transforms.
fn compute_normal_transform(m: &[[f64; 4]; 4]) -> [[f64; 3]; 3] {
    // Extract 3×3 submatrix
    let a = m[0][0]; let b = m[0][1]; let c = m[0][2];
    let d = m[1][0]; let e = m[1][1]; let f = m[1][2];
    let g = m[2][0]; let h = m[2][1]; let i = m[2][2];

    // Compute determinant of 3×3
    let det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);

    if det.abs() < 1e-15 {
        // Degenerate matrix — return identity
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    }

    let inv_det = 1.0 / det;

    // Compute inverse of 3×3 (cofactor matrix transposed, divided by det)
    let inv = [
        [(e * i - f * h) * inv_det, (c * h - b * i) * inv_det, (b * f - c * e) * inv_det],
        [(f * g - d * i) * inv_det, (a * i - c * g) * inv_det, (c * d - a * f) * inv_det],
        [(d * h - e * g) * inv_det, (b * g - a * h) * inv_det, (a * e - b * d) * inv_det],
    ];

    // Transpose the inverse to get (M⁻¹)ᵀ
    [
        [inv[0][0], inv[1][0], inv[2][0]],
        [inv[0][1], inv[1][1], inv[2][1]],
        [inv[0][2], inv[1][2], inv[2][2]],
    ]
}
