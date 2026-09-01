// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! # Mesh-based Boolean Operations
//!
//! Boolean operations (union, subtract, intersect) on triangle meshes.
//! Unlike B-Rep boolean operations which require exact surface-surface
//! intersection curves and topology stitching, mesh-based booleans operate
//! on the discretized geometry and are therefore more robust for complex
//! geometries (cylinders through boxes, fillets, etc.).
//!
//! ## Algorithm (D5 rewrite: Möller triangle-triangle)
//!
//! 1. **Broad phase** — a uniform spatial grid over triangle AABBs
//!    produces candidate pairs (one triangle from A, one from B).
//!
//! 2. **Narrow phase (Möller triangle-triangle)** — for each candidate
//!    pair the intersection line `L = plane(A) ∩ plane(B)` and the
//!    interval endpoints of *both* triangles on `L` are computed:
//!    - non-coplanar crossing pair → both triangles receive `L` as a
//!      cutting constraint, and each side receives the *other* side's
//!      interval endpoints as shared split points;
//!    - coplanar pair → each triangle receives the partner's edge lines
//!      as cutting constraints (a 2D arrangement in the shared plane).
//!
//! 3. **Decomposition** — every triangle with constraints is split in a
//!    local 2D frame by the cutting lines (convex polygon splitting).
//!    Because the split points along the intersection curve are copied
//!    verbatim between the two meshes, both sides produce the *same*
//!    edge structure along the curve — the result boundary follows the
//!    true intersection curve and is watertight by construction there.
//!
//! 4. **Classification** — each cell (triangle fragment) is classified:
//!    - coplanar-covered cells follow the orientation rule table below;
//!    - all other cells use a 3-axis majority ray-cast (point-in-mesh)
//!      from a slightly perturbed centroid (avoids on-axis grazing).
//!
//! 5. **Assembly + weld + cleanup** — kept cells are assembled with
//!    quantized vertex deduplication, welded at fp-level tolerance and
//!    cleaned of degenerate triangles. **No gap filling** is performed:
//!    boundary gaps can no longer be patched over silently (D3/D4/D5
//!    policy — fallbacks mask bugs).
//!
//! ## Coplanar rule table
//!
//! A cell of A covered by a coplanar face of B (and vice versa):
//!
//! | op        | A-cell, same orient | A-cell, opposite | B-cell, same | B-cell, opposite |
//! |-----------|--------------------|------------------|--------------|------------------|
//! | Union     | discard (B copy)   | discard          | keep         | discard          |
//! | Intersect | discard (B copy)   | discard          | keep         | discard          |
//! | Subtract  | discard            | keep (open B)    | discard      | discard          |
//!
//! ("same orient" = outward normals point the same way, i.e. both solids
//! lie on the same side of the shared plane; "opposite" = the faces kiss,
//! solids on opposite sides — for subtract, B is treated as open.)

use crate::mesh::TriangleMesh;
use draper_geometry::{Direction3d, Point3d, Vec2d, Vec3d};
use std::collections::{HashMap, HashSet};

// ============================================================
// Public API
// ============================================================

/// Compute boolean union of two triangle meshes.
///
/// Result = A ∪ B (all cells from A outside B, plus all cells
/// from B outside A).
pub fn mesh_union(a: &TriangleMesh, b: &TriangleMesh) -> TriangleMesh {
    mesh_boolean(a, b, MeshBooleanOp::Union)
}

/// Compute boolean subtract (a - b) of two triangle meshes.
///
/// Result = A - B (all cells from A outside B, plus the cells
/// from B that are inside A, with reversed winding to form the cavity).
pub fn mesh_subtract(a: &TriangleMesh, b: &TriangleMesh) -> TriangleMesh {
    mesh_boolean(a, b, MeshBooleanOp::Subtract)
}

/// Compute boolean intersect of two triangle meshes.
///
/// Result = A ∩ B (all cells from A inside B, plus all cells
/// from B inside A).
pub fn mesh_intersect(a: &TriangleMesh, b: &TriangleMesh) -> TriangleMesh {
    mesh_boolean(a, b, MeshBooleanOp::Intersect)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum MeshBooleanOp {
    Union,
    Subtract,
    Intersect,
}

// ============================================================
// Small vector helpers (Point3d has no operator overloads)
// ============================================================

#[inline]
fn vsub(a: &Point3d, b: &Point3d) -> Vec3d {
    Vec3d::new(a.x - b.x, a.y - b.y, a.z - b.z)
}

#[inline]
fn padd(a: &Point3d, v: &Vec3d) -> Point3d {
    Point3d::new(a.x + v.x, a.y + v.y, a.z + v.z)
}

#[inline]
fn tri_points(mesh: &TriangleMesh, i: usize) -> [Point3d; 3] {
    let t = mesh.triangles[i];
    [
        mesh.vertices[t[0] as usize],
        mesh.vertices[t[1] as usize],
        mesh.vertices[t[2] as usize],
    ]
}

// ============================================================
// Möller triangle-triangle intersection
// ============================================================

/// Result of a triangle-triangle intersection test.
enum TriTri {
    /// No intersection (including parallel distinct planes).
    Disjoint,
    /// The two triangles share a plane.
    Coplanar,
    /// The triangles cross; `L(t) = origin + t·dir` is the intersection
    /// line of both planes. `a_pts` / `b_pts` are the interval endpoints
    /// of each triangle on `L` (points where its edges cross the other
    /// plane). Both lie exactly on `L`, so they can be copied verbatim
    /// into the other triangle's decomposition as shared split points.
    Segment {
        origin: Point3d,
        dir: Vec3d,
        a_pts: Vec<Point3d>,
        b_pts: Vec<Point3d>,
    },
}

/// Compute the (unnormalized) normal of a triangle (cross product of two edges).
fn triangle_normal(t: &[Point3d; 3]) -> Vec3d {
    let e1 = vsub(&t[1], &t[0]);
    let e2 = vsub(&t[2], &t[0]);
    e1.cross(&e2)
}

/// Möller-style triangle-triangle intersection.
///
/// For non-parallel planes the test computes, for each triangle, the
/// points where its edges cross the *other* triangle's plane (all such
/// points lie on the line `L = plane_a ∩ plane_b`, because each triangle
/// lies inside its own plane). The triangles intersect iff their
/// intervals on `L` overlap.
fn tri_tri_intersect(ta: &[Point3d; 3], tb: &[Point3d; 3], eps: f64) -> TriTri {
    let na = triangle_normal(ta);
    let nb = triangle_normal(tb);
    let na_len = na.length();
    let nb_len = nb.length();
    if na_len <= eps || nb_len <= eps {
        // Degenerate (zero-area) triangle — nothing to intersect.
        return TriTri::Disjoint;
    }
    let nua = Vec3d::new(na.x / na_len, na.y / na_len, na.z / na_len);
    let nub = Vec3d::new(nb.x / nb_len, nb.y / nb_len, nb.z / nb_len);

    // Signed distances of B's vertices to A's plane and vice versa
    // (unit normals → world units).
    let db = [
        nua.dot(&vsub(&tb[0], &ta[0])),
        nua.dot(&vsub(&tb[1], &ta[0])),
        nua.dot(&vsub(&tb[2], &ta[0])),
    ];
    let da = [
        nub.dot(&vsub(&ta[0], &tb[0])),
        nub.dot(&vsub(&ta[1], &tb[0])),
        nub.dot(&vsub(&ta[2], &tb[0])),
    ];

    let line_dir = na.cross(&nb);
    let sin_theta = line_dir.length() / (na_len * nb_len);
    if sin_theta < 1e-7 {
        // Parallel planes: coplanar iff B's vertices lie in A's plane.
        let coplanar = db.iter().all(|d| d.abs() <= eps);
        return if coplanar { TriTri::Coplanar } else { TriTri::Disjoint };
    }

    let a_pts = plane_crossing_points(ta, &da, eps);
    let b_pts = plane_crossing_points(tb, &db, eps);
    if a_pts.is_empty() || b_pts.is_empty() {
        // One of the triangles does not reach the other's plane.
        return TriTri::Disjoint;
    }

    let origin = a_pts[0];
    let tval = |p: &Point3d| line_dir.dot(&vsub(p, &origin));
    let mut a_lo = f64::MAX;
    let mut a_hi = f64::MIN;
    for p in &a_pts {
        let t = tval(p);
        a_lo = a_lo.min(t);
        a_hi = a_hi.max(t);
    }
    let mut b_lo = f64::MAX;
    let mut b_hi = f64::MIN;
    for p in &b_pts {
        let t = tval(p);
        b_lo = b_lo.min(t);
        b_hi = b_hi.max(t);
    }
    if a_lo.max(b_lo) <= a_hi.min(b_hi) + eps {
        TriTri::Segment {
            origin,
            dir: line_dir,
            a_pts,
            b_pts,
        }
    } else {
        TriTri::Disjoint
    }
}

/// Points of `tri` that lie on a plane given by per-vertex signed
/// distances `dist` (on-plane vertices plus edge crossings). All returned
/// points lie on `tri` *and* on the plane.
fn plane_crossing_points(tri: &[Point3d; 3], dist: &[f64; 3], eps: f64) -> Vec<Point3d> {
    let mut pts: Vec<Point3d> = Vec::with_capacity(4);
    for i in 0..3 {
        if dist[i].abs() <= eps {
            pts.push(tri[i]);
        }
        let j = (i + 1) % 3;
        if (dist[i] > eps && dist[j] < -eps) || (dist[i] < -eps && dist[j] > eps) {
            let t = dist[i] / (dist[i] - dist[j]);
            pts.push(tri[i].lerp(&tri[j], t));
        }
    }
    // Dedup near-equal points (on-plane vertex + crossing at the same spot).
    let mut out: Vec<Point3d> = Vec::with_capacity(pts.len());
    for p in pts {
        if !out.iter().any(|q| q.distance_sq_to(&p) < eps * eps) {
            out.push(p);
        }
    }
    out
}

// ============================================================
// Constraint representation
// ============================================================

/// One cutting line on a triangle's plane, produced by an intersecting
/// partner triangle.
struct LineConstraint {
    /// Any 3D point on the line (lies on the triangle's plane up to fp).
    origin: Point3d,
    /// Line direction (3D, arbitrary length; lies in the plane).
    dir: Vec3d,
    /// Extra points on the line to insert as vertices. These are the
    /// partner's interval endpoints, copied verbatim from the pair
    /// computation so that both meshes share identical split points
    /// along the intersection curve.
    insert: Vec<Point3d>,
}

/// A coplanar partner triangle (its plane coincides with this triangle's).
struct CoplanarHit {
    tri: [Point3d; 3],
    /// Outward normals of the two triangles point the same way.
    same_orientation: bool,
}

// ============================================================
// Broad phase (spatial grid over triangle AABBs)
// ============================================================

fn triangle_aabbs(mesh: &TriangleMesh) -> Vec<([f64; 3], [f64; 3])> {
    mesh.triangles
        .iter()
        .map(|tri| {
            let a = mesh.vertices[tri[0] as usize];
            let b = mesh.vertices[tri[1] as usize];
            let c = mesh.vertices[tri[2] as usize];
            (
                [
                    a.x.min(b.x).min(c.x),
                    a.y.min(b.y).min(c.y),
                    a.z.min(b.z).min(c.z),
                ],
                [
                    a.x.max(b.x).max(c.x),
                    a.y.max(b.y).max(c.y),
                    a.z.max(b.z).max(c.z),
                ],
            )
        })
        .collect()
}

fn mesh_aabb(mesh: &TriangleMesh) -> ([f64; 3], [f64; 3]) {
    let mut mn = [f64::MAX; 3];
    let mut mx = [f64::MIN; 3];
    for v in &mesh.vertices {
        mn[0] = mn[0].min(v.x);
        mn[1] = mn[1].min(v.y);
        mn[2] = mn[2].min(v.z);
        mx[0] = mx[0].max(v.x);
        mx[1] = mx[1].max(v.y);
        mx[2] = mx[2].max(v.z);
    }
    (mn, mx)
}

/// Combined scene scale: the largest dimension spanned by both meshes.
fn combined_scene_scale(a: &TriangleMesh, b: &TriangleMesh) -> f64 {
    let (mn_a, mx_a) = mesh_aabb(a);
    let (mn_b, mx_b) = mesh_aabb(b);
    let mn = [
        mn_a[0].min(mn_b[0]),
        mn_a[1].min(mn_b[1]),
        mn_a[2].min(mn_b[2]),
    ];
    let mx = [
        mx_a[0].max(mx_b[0]),
        mx_a[1].max(mx_b[1]),
        mx_a[2].max(mx_b[2]),
    ];
    let d = (mx[0] - mn[0])
        .max(mx[1] - mn[1])
        .max(mx[2] - mn[2]);
    d.max(1e-12)
}

/// Candidate triangle pairs (index into A, index into B) whose AABBs
/// overlap. Uses a uniform grid built over B's triangles.
fn broad_phase_pairs(a: &TriangleMesh, b: &TriangleMesh, scale: f64) -> Vec<(usize, usize)> {
    let aabbs_a = triangle_aabbs(a);
    let aabbs_b = triangle_aabbs(b);
    let cell = (scale / 32.0).max(1e-12);
    let cell_key = |x: f64| (x / cell).floor() as i64;

    let mut grid: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
    for (j, (mn, mx)) in aabbs_b.iter().enumerate() {
        let (x0, x1) = (cell_key(mn[0]), cell_key(mx[0]));
        let (y0, y1) = (cell_key(mn[1]), cell_key(mx[1]));
        let (z0, z1) = (cell_key(mn[2]), cell_key(mx[2]));
        for cx in x0..=x1 {
            for cy in y0..=y1 {
                for cz in z0..=z1 {
                    grid.entry((cx, cy, cz)).or_default().push(j);
                }
            }
        }
    }

    let mut pairs: Vec<(usize, usize)> = Vec::new();
    let mut seen: HashSet<(usize, usize)> = HashSet::new();
    for (i, (mn, mx)) in aabbs_a.iter().enumerate() {
        let (x0, x1) = (cell_key(mn[0]), cell_key(mx[0]));
        let (y0, y1) = (cell_key(mn[1]), cell_key(mx[1]));
        let (z0, z1) = (cell_key(mn[2]), cell_key(mx[2]));
        for cx in x0..=x1 {
            for cy in y0..=y1 {
                for cz in z0..=z1 {
                    if let Some(list) = grid.get(&(cx, cy, cz)) {
                        for &j in list {
                            if seen.insert((i, j)) {
                                pairs.push((i, j));
                            }
                        }
                    }
                }
            }
        }
    }
    pairs
}

// ============================================================
// Triangle decomposition (cell generation)
// ============================================================

/// Split `tri` by all cutting lines into convex cells.
///
/// Works in a local 2D frame on the triangle's plane; every output
/// triangle is lifted back to 3D. Split points are exact line-edge
/// crossings; the constraint's `insert` points (the partner's interval
/// endpoints) are added as vertices on the new boundary edges that lie
/// on the line.
fn decompose_triangle(
    tri: &[Point3d; 3],
    constraints: &[LineConstraint],
    eps: f64,
) -> Vec<[Point3d; 3]> {
    if constraints.is_empty() {
        return vec![*tri];
    }

    // Orthonormal frame (u, v) on the triangle's plane; (u, v, n) is
    // right-handed with n the triangle normal, so CCW 2D polygons fan
    // back to the original winding.
    let e1 = vsub(&tri[1], &tri[0]);
    let e2 = vsub(&tri[2], &tri[0]);
    let n = e1.cross(&e2);
    let e1_len = e1.length();
    let n_len = n.length();
    if e1_len <= eps || n_len <= eps * eps.max(1e-30) {
        return vec![*tri]; // degenerate
    }
    let u = Vec3d::new(e1.x / e1_len, e1.y / e1_len, e1.z / e1_len);
    let n_unit = Vec3d::new(n.x / n_len, n.y / n_len, n.z / n_len);
    let v = n_unit.cross(&u);

    let origin = tri[0];
    let to2 = |p: &Point3d| -> Vec2d {
        let d = vsub(p, &origin);
        Vec2d::new(d.dot(&u), d.dot(&v))
    };
    let to3 = |p: &Vec2d| -> Point3d {
        Point3d::new(
            origin.x + p.u * u.x + p.v * v.x,
            origin.y + p.u * u.y + p.v * v.y,
            origin.z + p.u * u.z + p.v * v.z,
        )
    };

    let mut polys: Vec<Vec<Vec2d>> = vec![vec![to2(&tri[0]), to2(&tri[1]), to2(&tri[2])]];

    for c in constraints {
        let o2 = to2(&c.origin);
        let p2 = to2(&padd(&c.origin, &c.dir));
        let draw = Vec2d::new(p2.u - o2.u, p2.v - o2.v);
        let dlen = draw.length();
        if dlen <= eps {
            continue; // line perpendicular to the plane (invalid here) — skip
        }
        let d2 = Vec2d::new(draw.u / dlen, draw.v / dlen);
        // Only insert points that actually lie on the line (they should,
        // up to fp drift).
        let mut ins2: Vec<Vec2d> = Vec::with_capacity(c.insert.len());
        for p in &c.insert {
            let q = to2(p);
            if (d2.u * (q.v - o2.v) - d2.v * (q.u - o2.u)).abs() <= eps {
                ins2.push(q);
            }
        }
        polys = split_polys_by_line(polys, &o2, &d2, &ins2, eps);
    }

    // Fan-triangulate convex pieces. For polygons with more than 3
    // vertices the fan is built from the polygon *centroid*: a plain
    // (p0, pk, pk+1) fan would emit degenerate triangles whenever
    // consecutive boundary vertices are collinear (inserted shared
    // points), and clean_mesh would drop them — silently removing the
    // shared vertex from the edge structure and creating T-junctions.
    // The centroid fan keeps every boundary edge (v_i, v_i+1) intact.
    let mut out = Vec::new();
    for poly in &polys {
        if poly.len() < 3 {
            continue;
        }
        if poly.len() == 3 {
            out.push([to3(&poly[0]), to3(&poly[1]), to3(&poly[2])]);
        } else {
            let mut c = Vec2d::new(0.0, 0.0);
            for p in poly {
                c.u += p.u;
                c.v += p.v;
            }
            let inv = 1.0 / poly.len() as f64;
            let c = Vec2d::new(c.u * inv, c.v * inv);
            let c3 = to3(&c);
            for k in 0..poly.len() {
                out.push([to3(&poly[k]), to3(&poly[(k + 1) % poly.len()]), c3]);
            }
        }
    }
    out
}

/// Signed side of point `p` w.r.t. the oriented line (o, d) — equals the
/// signed distance times |d| (d is unit here, so it *is* the distance).
#[inline]
fn line_side(p: &Vec2d, o: &Vec2d, d: &Vec2d) -> f64 {
    d.u * (p.v - o.v) - d.v * (p.u - o.u)
}

fn split_polys_by_line(
    polys: Vec<Vec<Vec2d>>,
    o: &Vec2d,
    d: &Vec2d,
    ins: &[Vec2d],
    eps: f64,
) -> Vec<Vec<Vec2d>> {
    polys
        .into_iter()
        .flat_map(|p| split_poly_by_line(p, o, d, ins, eps))
        .collect()
}

/// Split one convex polygon by a line. Vertices exactly on the line go to
/// both sides. New boundary edges lying on the line receive the `ins`
/// points that fall strictly inside their span.
fn split_poly_by_line(
    poly: Vec<Vec2d>,
    o: &Vec2d,
    d: &Vec2d,
    ins: &[Vec2d],
    eps: f64,
) -> Vec<Vec<Vec2d>> {
    let n = poly.len();
    if n < 3 {
        return vec![poly];
    }
    let sides: Vec<f64> = poly.iter().map(|p| line_side(p, o, d)).collect();
    let has_pos = sides.iter().any(|&s| s > eps);
    let has_neg = sides.iter().any(|&s| s < -eps);
    if !has_pos || !has_neg {
        // No crossing — only insert points on on-line edges.
        let mut single = poly;
        insert_points_on_line_edges(&mut single, o, d, ins, eps);
        return vec![single];
    }

    let mut pos: Vec<Vec2d> = Vec::with_capacity(n + 2);
    let mut neg: Vec<Vec2d> = Vec::with_capacity(n + 2);
    for i in 0..n {
        let j = (i + 1) % n;
        let p = poly[i];
        let q = poly[j];
        let (sp, sq) = (sides[i], sides[j]);
        if sp > eps {
            pos.push(p);
        } else if sp < -eps {
            neg.push(p);
        } else {
            // On the line — belongs to both halves.
            pos.push(p);
            neg.push(p);
        }
        if (sp > eps && sq < -eps) || (sp < -eps && sq > eps) {
            let t = sp / (sp - sq);
            let c = Vec2d::new(p.u + t * (q.u - p.u), p.v + t * (q.v - p.v));
            pos.push(c);
            neg.push(c);
        }
    }

    let mut out: Vec<Vec<Vec2d>> = Vec::new();
    for mut side_poly in [pos, neg] {
        dedup_consecutive(&mut side_poly, eps);
        if side_poly.len() >= 3 {
            insert_points_on_line_edges(&mut side_poly, o, d, ins, eps);
            dedup_consecutive(&mut side_poly, eps);
            if side_poly.len() >= 3 {
                out.push(side_poly);
            }
        }
    }
    if out.is_empty() {
        // Defensive: never lose the polygon entirely.
        let mut single = poly;
        insert_points_on_line_edges(&mut single, o, d, ins, eps);
        out.push(single);
    }
    out
}

/// Remove consecutive duplicate vertices (including wrap-around).
fn dedup_consecutive(poly: &mut Vec<Vec2d>, eps: f64) {
    if poly.len() < 2 {
        return;
    }
    let mut out: Vec<Vec2d> = Vec::with_capacity(poly.len());
    for p in poly.iter() {
        if let Some(last) = out.last() {
            if (p.u - last.u).abs() <= eps && (p.v - last.v).abs() <= eps {
                continue;
            }
        }
        out.push(*p);
    }
    while out.len() > 1 {
        let first = out[0];
        let last = *out.last().unwrap();
        if (first.u - last.u).abs() <= eps && (first.v - last.v).abs() <= eps {
            out.pop();
        } else {
            break;
        }
    }
    *poly = out;
}

/// Insert the given on-line points into the polygon's edges that lie on
/// the line (subdividing those edges).
fn insert_points_on_line_edges(poly: &mut Vec<Vec2d>, o: &Vec2d, d: &Vec2d, ins: &[Vec2d], eps: f64) {
    if ins.is_empty() || poly.len() < 3 {
        return;
    }
    let t_of = |p: &Vec2d| d.u * (p.u - o.u) + d.v * (p.v - o.v);
    let mut rebuilt: Vec<Vec2d> = Vec::with_capacity(poly.len() + ins.len());
    let n = poly.len();
    for i in 0..n {
        let a = poly[i];
        let b = poly[(i + 1) % n];
        rebuilt.push(a);
        let on_line = line_side(&a, o, d).abs() <= eps && line_side(&b, o, d).abs() <= eps;
        if on_line {
            let (ta, tb) = (t_of(&a), t_of(&b));
            let (lo, hi) = if ta < tb { (ta, tb) } else { (tb, ta) };
            let mut mid: Vec<(f64, Vec2d)> = ins
                .iter()
                .map(|p| (t_of(p), *p))
                .filter(|(t, _)| *t > lo + eps && *t < hi - eps)
                .collect();
            mid.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap());
            // The edge is traversed a -> b; if t decreases along that
            // traversal, the points must be inserted in decreasing t
            // order, otherwise the polygon self-intersects (butterfly).
            if ta > tb {
                mid.reverse();
            }
            for (_, p) in mid {
                if let Some(last) = rebuilt.last() {
                    if (p.u - last.u).abs() <= eps && (p.v - last.v).abs() <= eps {
                        continue;
                    }
                }
                rebuilt.push(p);
            }
        }
    }
    *poly = rebuilt;
}

// ============================================================
// Cell classification
// ============================================================

/// Decide whether a cell (triangle fragment) belongs to the result.
///
/// Coplanar-covered cells follow the rule table in the module docs; all
/// other cells are classified by a 3-axis majority ray-cast against the
/// other mesh.
fn classify_cell(
    cell: &[Point3d; 3],
    coplanar_hits: &[CoplanarHit],
    other: &TriangleMesh,
    other_aabb: &([f64; 3], [f64; 3]),
    op: MeshBooleanOp,
    is_a: bool,
    eps: f64,
    perturb: f64,
) -> bool {
    let centroid = triangle_centroid(cell);

    if !coplanar_hits.is_empty() {
        let mut covered_same = false;
        let mut covered_opposite = false;
        for hit in coplanar_hits {
            if point_in_triangle_3d(&centroid, &hit.tri, eps) {
                if hit.same_orientation {
                    covered_same = true;
                } else {
                    covered_opposite = true;
                }
            }
        }
        if covered_same || covered_opposite {
            // Rule table (module docs). "same" has priority — a face
            // pair that both covers and kisses is pathological.
            return match (op, is_a, covered_same) {
                (MeshBooleanOp::Union, false, true)
                | (MeshBooleanOp::Intersect, false, true) => true, // B's copy is the kept one
                (MeshBooleanOp::Subtract, true, false) => true, // A's face, B treated as open
                _ => false,
            };
        }
    }

    let inside = point_in_mesh_robust(&centroid, other, other_aabb, perturb);
    match (op, is_a) {
        (MeshBooleanOp::Union, _) => !inside,
        (MeshBooleanOp::Subtract, true) => !inside,
        (MeshBooleanOp::Subtract, false) => inside,
        (MeshBooleanOp::Intersect, _) => inside,
    }
}

/// Barycentric containment test for a (near-)coplanar point in a 3D
/// triangle.
fn point_in_triangle_3d(p: &Point3d, tri: &[Point3d; 3], eps: f64) -> bool {
    let e0 = vsub(&tri[1], &tri[0]);
    let e1 = vsub(&tri[2], &tri[0]);
    let vp = vsub(p, &tri[0]);
    let d00 = e0.dot(&e0);
    let d01 = e0.dot(&e1);
    let d11 = e1.dot(&e1);
    let d20 = vp.dot(&e0);
    let d21 = vp.dot(&e1);
    let denom = d00 * d11 - d01 * d01;
    if denom.abs() <= eps * eps {
        return false; // degenerate triangle
    }
    let v = (d11 * d20 - d01 * d21) / denom;
    let w = (d00 * d21 - d01 * d20) / denom;
    let u = 1.0 - v - w;
    u >= -1e-9 && v >= -1e-9 && w >= -1e-9
}

/// Robust point-in-mesh: AABB prefilter, then a 3-axis majority vote of
/// parity ray casts from a slightly perturbed point (avoids on-axis
/// grazing of axis-aligned geometry).
fn point_in_mesh_robust(
    p: &Point3d,
    mesh: &TriangleMesh,
    aabb: &([f64; 3], [f64; 3]),
    perturb: f64,
) -> bool {
    let m = perturb;
    if p.x < aabb.0[0] - m
        || p.x > aabb.1[0] + m
        || p.y < aabb.0[1] - m
        || p.y > aabb.1[1] + m
        || p.z < aabb.0[2] - m
        || p.z > aabb.1[2] + m
    {
        return false;
    }
    let q = Point3d::new(
        p.x + perturb * 0.123,
        p.y + perturb * 0.456,
        p.z + perturb * 0.789,
    );
    let mut votes = 0u8;
    for dir in [Direction3d::X, Direction3d::Y, Direction3d::Z] {
        if point_in_mesh_along(&q, &dir, mesh) {
            votes += 1;
        }
    }
    votes >= 2
}

fn point_in_mesh_along(p: &Point3d, dir: &Direction3d, mesh: &TriangleMesh) -> bool {
    let mut count = 0u32;
    for tri in &mesh.triangles {
        let a = mesh.vertices[tri[0] as usize];
        let b = mesh.vertices[tri[1] as usize];
        let c = mesh.vertices[tri[2] as usize];
        if ray_triangle_intersect(p, dir, &a, &b, &c).is_some() {
            count += 1;
        }
    }
    count % 2 == 1
}

/// Test if a point is inside a triangle mesh using ray-casting along +X.
/// Returns `true` if the point is inside (odd number of ray/triangle
/// intersections along +X axis).
fn point_in_mesh(point: &Point3d, mesh: &TriangleMesh) -> bool {
    point_in_mesh_along(point, &Direction3d::X, mesh)
}

// ============================================================
// Boundary split propagation
// ============================================================

/// Is point `x` strictly inside segment (p, q) (within `eps`)?
fn point_strictly_on_segment(x: &Point3d, p: &Point3d, q: &Point3d, eps: f64) -> bool {
    let d = vsub(q, p);
    let len = d.length();
    if len <= eps {
        return false;
    }
    let ex = vsub(x, p);
    // Projection parameter along the segment.
    let t = ex.dot(&d) / (len * len);
    if t <= eps / len || t >= 1.0 - eps / len {
        return false;
    }
    // Perpendicular distance to the segment line.
    let perp = ex.cross(&d).length() / len;
    perp <= eps
}

/// Ordered (quantized) key for an undirected edge.
fn edge_key(p: &Point3d, q: &Point3d) -> ((u64, u64, u64), (u64, u64, u64)) {
    let ka = quantize_point(p);
    let kb = quantize_point(q);
    if (ka.0, ka.1, ka.2) <= (kb.0, kb.1, kb.2) {
        (ka, kb)
    } else {
        (kb, ka)
    }
}

/// Propagate boundary-edge split points to neighbor triangles.
///
/// A constraint-line cut of triangle T can subdivide T's *boundary* edges
/// (edges shared with neighboring triangles of the same mesh) even when
/// the neighbor itself has no constraints there. The neighbors' cells
/// must be subdivided at exactly the same points, otherwise the shared
/// edge structure mismatches on the two sides (T-junctions → boundary
/// edges in the result).
fn propagate_edge_splits(mesh: &TriangleMesh, cells: &mut Vec<Vec<[Point3d; 3]>>, eps: f64) {
    // 1. Collect split points per original mesh edge.
    let mut edge_splits_map: HashMap<((u64, u64, u64), (u64, u64, u64)), ((Point3d, Point3d), Vec<Point3d>)> = HashMap::new();
    for (i, tris_cells) in cells.iter().enumerate() {
        let tri = mesh.triangles[i];
        let va = [
            mesh.vertices[tri[0] as usize],
            mesh.vertices[tri[1] as usize],
            mesh.vertices[tri[2] as usize],
        ];
        for cell in tris_cells {
            for x in cell {
                if x.distance_to(&va[0]) <= eps
                    || x.distance_to(&va[1]) <= eps
                    || x.distance_to(&va[2]) <= eps
                {
                    continue; // original vertex, nothing to propagate
                }
                for e in 0..3 {
                    let p = va[e];
                    let q = va[(e + 1) % 3];
                    if point_strictly_on_segment(x, &p, &q, eps) {
                        let key = edge_key(&p, &q);
                        let entry = edge_splits_map.entry(key).or_insert_with(|| ((p, q), Vec::new()));
                        if !entry.1.iter().any(|y| y.distance_sq_to(x) < eps * eps) {
                            entry.1.push(*x);
                        }
                    }
                }
            }
        }
    }
    if edge_splits_map.is_empty() {
        return;
    }
    let edge_splits: Vec<((Point3d, Point3d), Vec<Point3d>)> =
        edge_splits_map.into_values().collect();

    // 2. Subdivide every cell whose boundary edge lies on one of the split
    //    edges and contains split points strictly inside it. Cell edges are
    //    matched by *collinearity* (a cut cell's edge is a fragment of the
    //    original edge, not the full corner-to-corner edge).
    for tris_cells in cells.iter_mut() {
        let mut i = 0;
        while i < tris_cells.len() {
            let cell = tris_cells[i];
            let new_tris = apply_cell_edge_splits(&cell, &edge_splits, eps);
            if new_tris.len() == 1 {
                i += 1;
            } else {
                tris_cells.remove(i);
                for (offset, nt) in new_tris.iter().enumerate() {
                    tris_cells.insert(i + offset, *nt);
                }
                i += new_tris.len();
            }
        }
    }
}

/// Distance from point `x` to the line through `a` and `b`.
fn dist_point_line(x: &Point3d, a: &Point3d, b: &Point3d) -> f64 {
    let ab = vsub(b, a);
    let len = ab.length();
    if len <= f64::MIN_POSITIVE {
        return f64::MAX;
    }
    vsub(x, a).cross(&ab).length() / len
}

/// Subdivide a single cell triangle at the propagated split points that
/// fall strictly inside its boundary edges (matched by collinearity with
/// the split edges' supporting lines).
fn apply_cell_edge_splits(
    cell: &[Point3d; 3],
    edge_splits: &[((Point3d, Point3d), Vec<Point3d>)],
    eps: f64,
) -> Vec<[Point3d; 3]> {
    let mut tris = vec![*cell];
    for e in 0..3 {
        let (p, q) = (cell[e], cell[(e + 1) % 3]);
        let d = vsub(&q, &p);
        let len = d.length();
        if len <= eps {
            continue;
        }
        // Gather split points (from any split edge whose line contains
        // this cell edge) strictly inside (p, q).
        let mut mid: Vec<(f64, Point3d)> = Vec::new();
        for ((a, b), pts) in edge_splits {
            if dist_point_line(&p, a, b) > eps || dist_point_line(&q, a, b) > eps {
                continue;
            }
            for x in pts {
                let t = vsub(x, &p).dot(&d) / (len * len);
                if t > eps / len && t < 1.0 - eps / len {
                    mid.push((t, *x));
                }
            }
        }
        if mid.is_empty() {
            continue;
        }
        mid.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap());
        // Dedup near-identical points.
        let mut deduped: Vec<(f64, Point3d)> = Vec::with_capacity(mid.len());
        for m in mid {
            if let Some(last) = deduped.last() {
                if (m.0 - last.0).abs() <= eps / len {
                    continue;
                }
            }
            deduped.push(m);
        }
        mid = deduped;

        // Find the current triangle that still owns the full edge (p, q).
        let mut target: Option<usize> = None;
        'outer: for (ti, t) in tris.iter().enumerate() {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                let same = (a.distance_to(&p) <= eps && b.distance_to(&q) <= eps)
                    || (a.distance_to(&q) <= eps && b.distance_to(&p) <= eps);
                if same {
                    target = Some(ti);
                    break 'outer;
                }
            }
        }
        let ti = match target {
            Some(ti) => ti,
            None => continue, // edge already consumed by an earlier split
        };
        let t = tris.remove(ti);

        // Orient the chain along the triangle's own winding of (p, q).
        let mut forward = false;
        for k in 0..3 {
            if t[k].distance_to(&p) <= eps && t[(k + 1) % 3].distance_to(&q) <= eps {
                forward = true;
            }
        }
        let (sp, sq) = if forward { (p, q) } else { (q, p) };
        let mut chain_mids: Vec<Point3d> = mid.iter().map(|(_, x)| *x).collect();
        if !forward {
            chain_mids.reverse();
        }
        // Third vertex (not on the edge).
        let r = match t
            .iter()
            .find(|v| v.distance_to(&p) > eps && v.distance_to(&q) > eps)
            .copied()
        {
            Some(r) => r,
            None => {
                tris.push(t);
                continue;
            }
        };
        let mut chain = vec![sp];
        chain.extend(chain_mids);
        chain.push(sq);
        for w in 0..chain.len() - 1 {
            tris.push([chain[w], chain[w + 1], r]);
        }
    }
    tris
}

// ============================================================
// Main boolean pipeline
// ============================================================

/// Main entry point for mesh boolean operations.
///
/// See the module docs for the algorithm. The result boundary follows the
/// true intersection curve; **no gap filling is performed** — residual
/// boundary edges (possible only in non-convex tangential configurations)
/// are a measurable defect, not something to patch silently.
fn mesh_boolean(a: &TriangleMesh, b: &TriangleMesh, op: MeshBooleanOp) -> TriangleMesh {
    if a.triangles.is_empty() {
        return match op {
            MeshBooleanOp::Subtract => TriangleMesh::new(),
            _ => b.clone(),
        };
    }
    if b.triangles.is_empty() {
        return match op {
            MeshBooleanOp::Intersect => TriangleMesh::new(),
            _ => a.clone(),
        };
    }

    let scale = combined_scene_scale(a, b);
    let eps = scale * 1e-9;
    let perturb = scale * 1e-7;

    // ---- Phase 1: broad phase ------------------------------------------
    let pairs = broad_phase_pairs(a, b, scale);

    // ---- Phase 2: narrow phase (Möller triangle-triangle) --------------
    let mut lines_a: Vec<Vec<LineConstraint>> = (0..a.triangles.len()).map(|_| Vec::new()).collect();
    let mut lines_b: Vec<Vec<LineConstraint>> = (0..b.triangles.len()).map(|_| Vec::new()).collect();
    let mut cop_a: Vec<Vec<CoplanarHit>> = (0..a.triangles.len()).map(|_| Vec::new()).collect();
    let mut cop_b: Vec<Vec<CoplanarHit>> = (0..b.triangles.len()).map(|_| Vec::new()).collect();

    for (ia, ib) in pairs {
        let ta = tri_points(a, ia);
        let tb = tri_points(b, ib);
        match tri_tri_intersect(&ta, &tb, eps) {
            TriTri::Disjoint => {}
            TriTri::Coplanar => {
                let same = triangle_normal(&ta).dot(&triangle_normal(&tb)) > 0.0;
                // 2D-arrangement constraints: each side is cut by the
                // partner's edge lines. Crossings of two such lines are
                // computed identically from both sides (line × line), so
                // the arrangements stay consistent.
                for k in 0..3 {
                    let p = tb[k];
                    let q = tb[(k + 1) % 3];
                    lines_a[ia].push(LineConstraint {
                        origin: p,
                        dir: vsub(&q, &p),
                        insert: Vec::new(),
                    });
                }
                for k in 0..3 {
                    let p = ta[k];
                    let q = ta[(k + 1) % 3];
                    lines_b[ib].push(LineConstraint {
                        origin: p,
                        dir: vsub(&q, &p),
                        insert: Vec::new(),
                    });
                }
                cop_a[ia].push(CoplanarHit {
                    tri: tb,
                    same_orientation: same,
                });
                cop_b[ib].push(CoplanarHit {
                    tri: ta,
                    same_orientation: same,
                });
            }
            TriTri::Segment {
                origin,
                dir,
                a_pts,
                b_pts,
            } => {
                // Both triangles are cut along the same 3D line; each side
                // receives the other's interval endpoints as shared split
                // points, so the edge structure along the intersection
                // curve matches exactly on both sides.
                lines_a[ia].push(LineConstraint {
                    origin,
                    dir,
                    insert: b_pts,
                });
                lines_b[ib].push(LineConstraint {
                    origin,
                    dir,
                    insert: a_pts,
                });
            }
        }
    }

    // ---- Phase 3: decompose into cells ----------------------------------
    let mut cells_a: Vec<Vec<[Point3d; 3]>> = (0..a.triangles.len())
        .map(|i| decompose_triangle(&tri_points(a, i), &lines_a[i], eps))
        .collect();
    let mut cells_b: Vec<Vec<[Point3d; 3]>> = (0..b.triangles.len())
        .map(|j| decompose_triangle(&tri_points(b, j), &lines_b[j], eps))
        .collect();

    // ---- Phase 3b: propagate boundary-edge splits to neighbors ----------
    // Constraint-line cuts can subdivide a triangle's boundary edges while
    // the neighbor across that edge is uncut; insert the same split points
    // into the neighbor's cells so the shared edge structure matches.
    propagate_edge_splits(a, &mut cells_a, eps);
    propagate_edge_splits(b, &mut cells_b, eps);

    // ---- Phase 4+5: classify + assemble ---------------------------------
    let aabb_a = mesh_aabb(a);
    let aabb_b = mesh_aabb(b);

    let mut result = TriangleMesh::new();
    let mut vertex_map: HashMap<(u64, u64, u64), u32> = HashMap::new();

    for (i, cells) in cells_a.iter().enumerate() {
        for cell in cells {
            if classify_cell(cell, &cop_a[i], b, &aabb_b, op, true, eps, perturb) {
                add_triangle(&mut result, &mut vertex_map, cell);
            }
        }
    }
    for (j, cells) in cells_b.iter().enumerate() {
        for cell in cells {
            if classify_cell(cell, &cop_b[j], a, &aabb_a, op, false, eps, perturb) {
                if op == MeshBooleanOp::Subtract {
                    add_triangle(&mut result, &mut vertex_map, &[cell[0], cell[2], cell[1]]);
                } else {
                    add_triangle(&mut result, &mut vertex_map, cell);
                }
            }
        }
    }

    // ---- Phase 6: weld fp-level duplicates + cleanup ---------------------
    // (No fill_boundary_gaps: the boundary is exact on the intersection
    // curve; the weld only merges fp-noise duplicates.)
    if !result.vertices.is_empty() {
        let bbox = compute_bounding_box(&result);
        let rscale = bbox_size(&bbox).max(scale);
        weld_vertices(&mut result, rscale * 1e-6);
    }
    clean_mesh(result)
}

/// Add a triangle to the result mesh, deduplicating vertices.
fn add_triangle(
    mesh: &mut TriangleMesh,
    vertex_map: &mut HashMap<(u64, u64, u64), u32>,
    tri: &[Point3d; 3],
) {
    let mut indices = [0u32; 3];
    for (i, p) in tri.iter().enumerate() {
        let key = quantize_point(p);
        if let Some(&idx) = vertex_map.get(&key) {
            indices[i] = idx;
        } else {
            let idx = mesh.vertices.len() as u32;
            mesh.vertices.push(*p);
            vertex_map.insert(key, idx);
            indices[i] = idx;
        }
    }
    mesh.triangles.push(indices);
}

/// Möller-Trumbore ray-triangle intersection along an axis direction.
/// Returns `Some(t)` (distance from ray origin to hit point) if intersection
/// exists in `t > 1e-9` and barycentric coordinates are within [0,1].
fn ray_triangle_intersect(
    origin: &Point3d,
    direction: &Direction3d,
    a: &Point3d,
    b: &Point3d,
    c: &Point3d,
) -> Option<f64> {
    let edge1 = Vec3d::new(b.x - a.x, b.y - a.y, b.z - a.z);
    let edge2 = Vec3d::new(c.x - a.x, c.y - a.y, c.z - a.z);
    let h = Vec3d::new(
        direction.y * edge2.z - direction.z * edge2.y,
        direction.z * edge2.x - direction.x * edge2.z,
        direction.x * edge2.y - direction.y * edge2.x,
    );
    let det = edge1.x * h.x + edge1.y * h.y + edge1.z * h.z;
    if det.abs() < 1e-12 {
        return None;
    }
    let inv_det = 1.0 / det;
    let s = Vec3d::new(origin.x - a.x, origin.y - a.y, origin.z - a.z);
    let u = inv_det * (s.x * h.x + s.y * h.y + s.z * h.z);
    if u < 0.0 || u > 1.0 {
        return None;
    }
    let q = Vec3d::new(
        s.y * edge1.z - s.z * edge1.y,
        s.z * edge1.x - s.x * edge1.z,
        s.x * edge1.y - s.y * edge1.x,
    );
    let v = inv_det * (direction.x * q.x + direction.y * q.y + direction.z * q.z);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = inv_det * (edge2.x * q.x + edge2.y * q.y + edge2.z * q.z);
    if t > 1e-9 {
        Some(t)
    } else {
        None
    }
}

fn quantize_point(p: &Point3d) -> (u64, u64, u64) {
    const SCALE: f64 = 1e7;
    let qx = (p.x * SCALE).round() as i64 as u64;
    let qy = (p.y * SCALE).round() as i64 as u64;
    let qz = (p.z * SCALE).round() as i64 as u64;
    (qx, qy, qz)
}

/// Centroid of a triangle.
fn triangle_centroid(tri: &[Point3d; 3]) -> Point3d {
    Point3d::new(
        (tri[0].x + tri[1].x + tri[2].x) / 3.0,
        (tri[0].y + tri[1].y + tri[2].y) / 3.0,
        (tri[0].z + tri[1].z + tri[2].z) / 3.0,
    )
}

// ============================================================
// Mesh cleanup
// ============================================================

/// Remove degenerate triangles and empty vertices.
fn clean_mesh(mut mesh: TriangleMesh) -> TriangleMesh {
    // Remove degenerate triangles (zero area or duplicate vertices).
    // Index-preserving filter so the per-triangle attribute arrays
    // (face_normals, triangle_colors, triangle_face_ids) stay in sync —
    // `Vec::retain` on `triangles` alone would leave them stale and break
    // every downstream consumer that zips them with `triangles`.
    let keep: Vec<bool> = mesh
        .triangles
        .iter()
        .map(|tri| {
            let a = &mesh.vertices[tri[0] as usize];
            let b = &mesh.vertices[tri[1] as usize];
            let c = &mesh.vertices[tri[2] as usize];
            let area_sq = triangle_normal(&[*a, *b, *c]).length_sq();
            area_sq > 1e-18
        })
        .collect();

    let keep_idx = |i: usize| keep.get(i).copied().unwrap_or(true);
    mesh.triangles = mesh
        .triangles
        .iter()
        .enumerate()
        .filter(|(i, _)| keep_idx(*i))
        .map(|(_, t)| *t)
        .collect();
    // Filter each per-triangle attribute with the same predicate.
    if let Some(ref mut face_normals) = mesh.face_normals {
        *face_normals = face_normals
            .iter()
            .enumerate()
            .filter(|(i, _)| keep_idx(*i))
            .map(|(_, v)| *v)
            .collect();
    }
    if let Some(ref mut colors) = mesh.triangle_colors {
        *colors = colors
            .iter()
            .enumerate()
            .filter(|(i, _)| keep_idx(*i))
            .map(|(_, v)| *v)
            .collect();
    }
    if let Some(ref mut ids) = mesh.triangle_face_ids {
        *ids = ids
            .iter()
            .enumerate()
            .filter(|(i, _)| keep_idx(*i))
            .map(|(_, v)| *v)
            .collect();
    }

    // Compact vertices (remove unused)
    let mut used = vec![false; mesh.vertices.len()];
    for tri in &mesh.triangles {
        used[tri[0] as usize] = true;
        used[tri[1] as usize] = true;
        used[tri[2] as usize] = true;
    }
    let mut new_vertices = Vec::new();
    let mut old_to_new: Vec<u32> = vec![0; mesh.vertices.len()];
    for (i, &u) in used.iter().enumerate() {
        if u {
            old_to_new[i] = new_vertices.len() as u32;
            new_vertices.push(mesh.vertices[i]);
        }
    }
    for tri in &mut mesh.triangles {
        tri[0] = old_to_new[tri[0] as usize];
        tri[1] = old_to_new[tri[1] as usize];
        tri[2] = old_to_new[tri[2] as usize];
    }
    mesh.vertices = new_vertices;

    mesh
}

// ============================================================
// Vertex welding (for watertightness)
// ============================================================

/// Compute the axis-aligned bounding box of a mesh.
fn compute_bounding_box(mesh: &TriangleMesh) -> (Point3d, Point3d) {
    let mut min = Point3d::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max = Point3d::new(f64::MIN, f64::MIN, f64::MIN);
    for v in &mesh.vertices {
        min.x = min.x.min(v.x);
        min.y = min.y.min(v.y);
        min.z = min.z.min(v.z);
        max.x = max.x.max(v.x);
        max.y = max.y.max(v.y);
        max.z = max.z.max(v.z);
    }
    (min, max)
}

/// Size of a bounding box (max dimension).
fn bbox_size(bbox: &(Point3d, Point3d)) -> f64 {
    let dx = bbox.1.x - bbox.0.x;
    let dy = bbox.1.y - bbox.0.y;
    let dz = bbox.1.z - bbox.0.z;
    dx.max(dy).max(dz)
}

/// Weld (merge) vertices that are within `tolerance` of each other.
/// This makes meshes watertight by merging duplicate/near-duplicate vertices.
fn weld_vertices(mesh: &mut TriangleMesh, tolerance: f64) {
    if mesh.vertices.is_empty() {
        return;
    }

    let tol_sq = tolerance * tolerance;

    // Build a spatial hash for fast neighbor lookup
    let cell_size = tolerance.max(1e-15);
    let mut grid: std::collections::HashMap<(i64, i64, i64), Vec<u32>> = std::collections::HashMap::new();

    let mut new_indices: Vec<u32> = vec![u32::MAX; mesh.vertices.len()];
    let mut new_vertices: Vec<Point3d> = Vec::new();

    for (i, v) in mesh.vertices.iter().enumerate() {
        let cx = (v.x / cell_size).floor() as i64;
        let cy = (v.y / cell_size).floor() as i64;
        let cz = (v.z / cell_size).floor() as i64;

        // Check this cell and 26 neighbors
        let mut found: Option<u32> = None;
        for dx in -1i64..=1 {
            for dy in -1i64..=1 {
                for dz in -1i64..=1 {
                    let key = (cx + dx, cy + dy, cz + dz);
                    if let Some(candidates) = grid.get(&key) {
                        for &idx in candidates {
                            let w = &new_vertices[idx as usize];
                            let d = (v.x - w.x).powi(2) + (v.y - w.y).powi(2) + (v.z - w.z).powi(2);
                            if d <= tol_sq {
                                found = Some(idx);
                                break;
                            }
                        }
                    }
                    if found.is_some() { break; }
                }
                if found.is_some() { break; }
            }
            if found.is_some() { break; }
        }

        let idx = match found {
            Some(i) => i,
            None => {
                let new_idx = new_vertices.len() as u32;
                new_vertices.push(*v);
                grid.entry((cx, cy, cz)).or_default().push(new_idx);
                new_idx
            }
        };
        new_indices[i] = idx;
    }

    // Remap triangles
    for tri in &mut mesh.triangles {
        tri[0] = new_indices[tri[0] as usize];
        tri[1] = new_indices[tri[1] as usize];
        tri[2] = new_indices[tri[2] as usize];
    }
    mesh.vertices = new_vertices;
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Number of edges not shared by exactly two triangles (0 = watertight
    /// 2-manifold edge structure).
    fn count_boundary_edges(mesh: &TriangleMesh) -> usize {
        let mut edges: HashMap<[u32; 2], u32> = HashMap::new();
        for tri in &mesh.triangles {
            for (i, j) in [(0usize, 1usize), (1, 2), (2, 0)] {
                let key = if tri[i] < tri[j] {
                    [tri[i], tri[j]]
                } else {
                    [tri[j], tri[i]]
                };
                *edges.entry(key).or_insert(0) += 1;
            }
        }
        edges.values().filter(|&&c| c != 2).count()
    }

    /// Signed volume via the divergence theorem (Σ det(p0,p1,p2)/6).
    /// Positive for consistently outward-oriented watertight meshes.
    fn signed_volume(mesh: &TriangleMesh) -> f64 {
        let mut vol = 0.0;
        for tri in &mesh.triangles {
            let p0 = mesh.vertices[tri[0] as usize];
            let p1 = mesh.vertices[tri[1] as usize];
            let p2 = mesh.vertices[tri[2] as usize];
            vol += (p0.x * (p1.y * p2.z - p1.z * p2.y)
                + p0.y * (p1.z * p2.x - p1.x * p2.z)
                + p0.z * (p1.x * p2.y - p1.y * p2.x))
                / 6.0;
        }
        vol
    }

    fn assert_watertight_and_volume(mesh: &TriangleMesh, expected: f64, what: &str) {
        let boundary = count_boundary_edges(mesh);
        assert_eq!(
            boundary, 0,
            "{what}: expected watertight result, got {boundary} boundary edges"
        );
        let vol = signed_volume(mesh);
        assert!(
            (vol - expected).abs() / expected.max(1e-9) < 1e-4,
            "{what}: expected volume {expected}, got {vol}"
        );
    }

    fn make_box_mesh(dx: f64, dy: f64, dz: f64) -> TriangleMesh {
        let hx = dx / 2.0;
        let hy = dy / 2.0;
        let hz = dz / 2.0;
        let v = vec![
            Point3d::new(-hx, -hy, -hz),
            Point3d::new(hx, -hy, -hz),
            Point3d::new(hx, hy, -hz),
            Point3d::new(-hx, hy, -hz),
            Point3d::new(-hx, -hy, hz),
            Point3d::new(hx, -hy, hz),
            Point3d::new(hx, hy, hz),
            Point3d::new(-hx, hy, hz),
        ];
        let triangles = vec![
            [0, 2, 1], [0, 3, 2], // bottom
            [4, 5, 6], [4, 6, 7], // top
            [0, 1, 5], [0, 5, 4], // front
            [3, 6, 2], [3, 7, 6], // back
            [0, 7, 3], [0, 4, 7], // left
            [1, 2, 6], [1, 6, 5], // right
        ];
        TriangleMesh { vertices: v, triangles, normals: None, face_normals: None, triangle_colors: None, triangle_face_ids: None }
    }

    /// Outward-oriented prism over a regular 32-gon (approximating a
    /// cylinder), centered on the z-axis, z from 0 to `height`.
    fn make_cylinder_mesh(radius: f64, height: f64, segments: usize) -> TriangleMesh {
        let mut vertices = Vec::new();
        let mut triangles: Vec<[u32; 3]> = Vec::new();

        // Bottom center vertex
        vertices.push(Point3d::new(0.0, 0.0, 0.0));
        // Top center vertex
        vertices.push(Point3d::new(0.0, 0.0, height));

        // Ring vertices
        for i in 0..segments {
            let angle = 2.0 * std::f64::consts::PI * (i as f64) / (segments as f64);
            let x = radius * angle.cos();
            let y = radius * angle.sin();
            vertices.push(Point3d::new(x, y, 0.0)); // bottom ring
            vertices.push(Point3d::new(x, y, height)); // top ring
        }

        // Bottom cap triangles (fan from center, clockwise seen from +z
        // → outward normal −z)
        for i in 0..segments {
            let next = (i + 1) % segments;
            triangles.push([0, (2 + 2 * next) as u32, (2 + 2 * i) as u32]);
        }

        // Top cap triangles (fan from center, CCW → outward normal +z)
        for i in 0..segments {
            let next = (i + 1) % segments;
            triangles.push([1, (3 + 2 * i) as u32, (3 + 2 * next) as u32]);
        }

        // Side quads (2 triangles each, outward radial normals)
        for i in 0..segments {
            let next = (i + 1) % segments;
            let b0 = (2 + 2 * i) as u32;
            let b1 = (2 + 2 * next) as u32;
            let t0 = (3 + 2 * i) as u32;
            let t1 = (3 + 2 * next) as u32;
            triangles.push([b0, b1, t0]);
            triangles.push([b1, t1, t0]);
        }

        TriangleMesh { vertices, triangles, normals: None, face_normals: None, triangle_colors: None, triangle_face_ids: None }
    }

    /// Area of a regular n-gon with circumradius R.
    fn ngon_area(n: usize, r: f64) -> f64 {
        0.5 * (n as f64) * r * r * (2.0 * std::f64::consts::PI / n as f64).sin()
    }

    fn rotate_z(mesh: &TriangleMesh, angle_rad: f64) -> TriangleMesh {
        let (s, c) = (angle_rad.sin(), angle_rad.cos());
        let mut out = mesh.clone();
        for v in &mut out.vertices {
            let (x, y) = (v.x, v.y);
            v.x = c * x - s * y;
            v.y = s * x + c * y;
        }
        out
    }

    fn translate(mesh: &TriangleMesh, dx: f64, dy: f64, dz: f64) -> TriangleMesh {
        let mut out = mesh.clone();
        for v in &mut out.vertices {
            v.x += dx;
            v.y += dy;
            v.z += dz;
        }
        out
    }

    // ------------------------------------------------------------
    // Möller triangle-triangle unit tests
    // ------------------------------------------------------------

    #[test]
    fn test_tri_tri_crossing_segment() {
        // Two triangles crossing like an X; both contain the z-axis
        // segment from (0,0,-0.5) to (0,0,0.5).
        let ta = [
            Point3d::new(-1.0, 0.0, -1.0),
            Point3d::new(-1.0, 0.0, 1.0),
            Point3d::new(1.0, 0.0, 0.0),
        ];
        let tb = [
            Point3d::new(0.0, -1.0, -1.0),
            Point3d::new(0.0, -1.0, 1.0),
            Point3d::new(0.0, 1.0, 0.0),
        ];
        match tri_tri_intersect(&ta, &tb, 1e-9) {
            TriTri::Segment { a_pts, b_pts, .. } => {
                assert_eq!(a_pts.len(), 2, "A should have 2 interval endpoints");
                assert_eq!(b_pts.len(), 2, "B should have 2 interval endpoints");
                for p in a_pts.iter().chain(b_pts.iter()) {
                    assert!(p.x.abs() < 1e-9 && p.y.abs() < 1e-9, "points on z-axis");
                    assert!(p.z.abs() - 0.5 < 1e-9, "z within interval, got {:?}", p);
                }
            }
            other => panic!("expected Segment, got {:?}", matches!(other, TriTri::Disjoint)),
        }
    }

    #[test]
    fn test_tri_tri_disjoint() {
        let ta = [
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 0.0, 0.0),
            Point3d::new(0.0, 1.0, 0.0),
        ];
        let tb = [
            Point3d::new(5.0, 5.0, 5.0),
            Point3d::new(6.0, 5.0, 5.0),
            Point3d::new(5.0, 6.0, 5.0),
        ];
        assert!(matches!(
            tri_tri_intersect(&ta, &tb, 1e-9),
            TriTri::Disjoint
        ));
    }

    #[test]
    fn test_tri_tri_coplanar() {
        let ta = [
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(2.0, 0.0, 0.0),
            Point3d::new(0.0, 2.0, 0.0),
        ];
        let tb = [
            Point3d::new(1.0, 1.0, 0.0),
            Point3d::new(3.0, 1.0, 0.0),
            Point3d::new(1.0, 3.0, 0.0),
        ];
        assert!(matches!(
            tri_tri_intersect(&ta, &tb, 1e-9),
            TriTri::Coplanar
        ));
    }

    #[test]
    fn test_tri_tri_parallel_distinct_planes() {
        let ta = [
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 0.0, 0.0),
            Point3d::new(0.0, 1.0, 0.0),
        ];
        let tb = [
            Point3d::new(0.0, 0.0, 5.0),
            Point3d::new(1.0, 0.0, 5.0),
            Point3d::new(0.0, 1.0, 5.0),
        ];
        assert!(matches!(
            tri_tri_intersect(&ta, &tb, 1e-9),
            TriTri::Disjoint
        ));
    }

    // ------------------------------------------------------------
    // Decomposition unit tests
    // ------------------------------------------------------------

    #[test]
    fn test_decompose_split_by_line() {
        // Triangle in the z=0 plane cut by the line y=1.
        let tri = [
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(4.0, 0.0, 0.0),
            Point3d::new(2.0, 3.0, 0.0),
        ];
        let constraints = [LineConstraint {
            origin: Point3d::new(0.0, 1.0, 0.0),
            dir: Vec3d::new(1.0, 0.0, 0.0),
            insert: Vec::new(),
        }];
        let cells = decompose_triangle(&tri, &constraints, 1e-9);
        // Line y=1 crosses edges (0,0)-(2,3) and (4,0)-(2,3): lower part
        // is a quad (centroid-fanned into 4 triangles), upper part a
        // triangle → 5 cells in total.
        assert_eq!(cells.len(), 5, "quad + tri = 5 cells, got {}", cells.len());
        // Total area must be preserved.
        let area = cells
            .iter()
            .map(|c| triangle_normal(c).length() * 0.5)
            .sum::<f64>();
        let orig = triangle_normal(&tri).length() * 0.5;
        assert!((area - orig).abs() < 1e-9, "area {area} != {orig}");
        // Every cell must be entirely on one side of the line.
        for c in &cells {
            let ys: Vec<f64> = c.iter().map(|p| p.y).collect();
            let all_below = ys.iter().all(|&y| y <= 1.0 + 1e-9);
            let all_above = ys.iter().all(|&y| y >= 1.0 - 1e-9);
            assert!(all_below || all_above, "cell straddles the line: {ys:?}");
        }
    }

    #[test]
    fn test_decompose_insert_points() {
        // Same split, but with a shared point (2, 1, 0) on the line that
        // must become a vertex of the on-line edges.
        let tri = [
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(4.0, 0.0, 0.0),
            Point3d::new(2.0, 3.0, 0.0),
        ];
        let constraints = [LineConstraint {
            origin: Point3d::new(0.0, 1.0, 0.0),
            dir: Vec3d::new(1.0, 0.0, 0.0),
            insert: vec![Point3d::new(2.0, 1.0, 0.0)],
        }];
        let cells = decompose_triangle(&tri, &constraints, 1e-9);
        let has_vertex = cells.iter().any(|c| {
            c.iter()
                .any(|p| p.distance_to(&Point3d::new(2.0, 1.0, 0.0)) < 1e-6)
        });
        assert!(has_vertex, "insert point must appear as a vertex");
        let area = cells
            .iter()
            .map(|c| triangle_normal(c).length() * 0.5)
            .sum::<f64>();
        let orig = triangle_normal(&tri).length() * 0.5;
        assert!((area - orig).abs() < 1e-9, "area {area} != {orig}");
    }

    #[test]
    fn test_decompose_no_constraints() {
        let tri = [
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 0.0, 0.0),
            Point3d::new(0.0, 1.0, 0.0),
        ];
        let cells = decompose_triangle(&tri, &[], 1e-9);
        assert_eq!(cells.len(), 1);
        assert_eq!(cells[0], tri);
    }

    // ------------------------------------------------------------
    // Boolean integration tests (watertightness + volume)
    // ------------------------------------------------------------

    #[test]
    fn test_box_minus_box_subtract() {
        // B fully inside A: result is a box with an internal cubic cavity.
        let a = make_box_mesh(100.0, 100.0, 100.0);
        let b = make_box_mesh(30.0, 30.0, 30.0);

        let result = mesh_subtract(&a, &b);
        println!(
            "Subtract result: {} vertices, {} triangles",
            result.vertices.len(),
            result.triangles.len()
        );

        assert!(result.triangles.len() >= 24, "outer shell + cavity walls");
        assert_watertight_and_volume(&result, 100.0f64.powi(3) - 30.0f64.powi(3), "box-box");
    }

    #[test]
    fn test_box_union_box() {
        // Two overlapping boxes (coplanar top/bottom/front/back faces).
        let a = make_box_mesh(40.0, 40.0, 40.0);
        let b = translate(&make_box_mesh(40.0, 40.0, 40.0), 20.0, 0.0, 0.0);

        let result = mesh_union(&a, &b);
        println!("Union result: {} triangles", result.triangles.len());
        // Union = box [-20,40] x [-20,20] x [-20,20] → 60·40·40.
        assert_watertight_and_volume(&result, 60.0 * 40.0 * 40.0, "union box-box");
    }

    #[test]
    fn test_box_union_disjoint_boxes() {
        let a = make_box_mesh(40.0, 40.0, 40.0);
        let b = translate(&make_box_mesh(40.0, 40.0, 40.0), 200.0, 0.0, 0.0);
        let result = mesh_union(&a, &b);
        assert_watertight_and_volume(&result, 2.0 * 40.0f64.powi(3), "union disjoint");
    }

    #[test]
    fn test_box_intersect_box() {
        // A = [-20,20]^3, B = [0,40]x[-20,20]x[-20,20]:
        // intersection = [0,20] x [-20,20] x [-20,20].
        let a = make_box_mesh(40.0, 40.0, 40.0);
        let b = translate(&make_box_mesh(40.0, 40.0, 40.0), 20.0, 0.0, 0.0);
        let result = mesh_intersect(&a, &b);
        assert_watertight_and_volume(&result, 20.0 * 40.0 * 40.0, "intersect box-box");
    }

    #[test]
    fn test_box_minus_overlapping_box() {
        // A = [-20,20]^3, B = [0,40]^3 shifted by 20 in x:
        // A - B keeps the left half of A plus B's cut wall.
        let a = make_box_mesh(40.0, 40.0, 40.0);
        let b = translate(&make_box_mesh(40.0, 40.0, 40.0), 20.0, 0.0, 0.0);
        let result = mesh_subtract(&a, &b);
        assert_watertight_and_volume(&result, 40.0f64.powi(3) - 20.0 * 40.0 * 40.0, "subtract overlap");
    }

    #[test]
    fn test_box_union_rotated_box() {
        // Non-axis-aligned configuration: no coplanar faces, all
        // crossings generic — a strong robustness test.
        let a = make_box_mesh(40.0, 40.0, 40.0);
        let b = translate(&rotate_z(&make_box_mesh(40.0, 40.0, 40.0), 0.5236), 15.0, 5.0, 0.0);

        let result = mesh_union(&a, &b);
        let boundary = count_boundary_edges(&result);
        assert_eq!(
            boundary, 0,
            "rotated union: expected watertight, got {boundary} boundary edges"
        );
        let vol = signed_volume(&result);
        // V(A∪B) ∈ (max(VA,VB), VA+VB) with a real overlap.
        let (va, vb) = (40.0f64.powi(3), 40.0f64.powi(3));
        assert!(
            vol > va && vol < va + vb,
            "rotated union volume {vol} out of ({va}, {})",
            va + vb
        );
    }

    #[test]
    fn test_box_intersect_rotated_box() {
        // Rotated intersect: A-cells inside B + B-cells inside A, all
        // crossings generic (no coplanar faces).
        let a = make_box_mesh(40.0, 40.0, 40.0);
        let b = translate(&rotate_z(&make_box_mesh(40.0, 40.0, 40.0), 0.5236), 15.0, 5.0, 0.0);
        let result = mesh_intersect(&a, &b);
        let boundary = count_boundary_edges(&result);
        assert_eq!(
            boundary, 0,
            "rotated intersect: expected watertight, got {boundary} boundary edges"
        );
        // V(A∩B) ∈ (0, min(VA,VB)); the overlap is substantial (~half).
        let vol = signed_volume(&result);
        assert!(
            vol > 0.2 * 40.0f64.powi(3) && vol < 0.9 * 40.0f64.powi(3),
            "rotated intersect volume {vol} out of expected range"
        );
    }

    #[test]
    fn test_point_in_box() {
        let box_mesh = make_box_mesh(100.0, 100.0, 100.0);
        // Off-center point should be inside (avoid axis-aligned rays that
        // hit edges/vertices exactly)
        assert!(point_in_mesh(&Point3d::new(1.0, 2.0, 3.0), &box_mesh));
        // Outside point
        assert!(!point_in_mesh(&Point3d::new(200.0, 100.0, 50.0), &box_mesh));
    }

    #[test]
    fn test_box_minus_cylinder_mesh_boolean() {
        // Cylinder pokes through the box on both z sides: the result is a
        // box with a through hole of 32-gon cross-section.
        let box_mesh = make_box_mesh(100.0, 80.0, 50.0);
        let cyl = translate(&make_cylinder_mesh(20.0, 100.0, 32), 0.0, 0.0, -50.0);

        let result = mesh_subtract(&box_mesh, &cyl);
        println!(
            "Box-Cylinder subtract: {} vertices, {} triangles",
            result.vertices.len(),
            result.triangles.len()
        );

        let hole_volume = ngon_area(32, 20.0) * 50.0;
        let expected = 100.0 * 80.0 * 50.0 - hole_volume;
        assert_watertight_and_volume(&result, expected, "box-cylinder");
    }

    #[test]
    fn test_box_union_cylinder_through_faces() {
        // Cylinder pokes through the box: union = box + two cylindrical
        // studs sticking out (both caps kept outside the box).
        let box_mesh = make_box_mesh(100.0, 80.0, 50.0);
        let cyl = translate(&make_cylinder_mesh(20.0, 100.0, 32), 0.0, 0.0, -50.0);

        let result = mesh_union(&box_mesh, &cyl);
        let cyl_vol = ngon_area(32, 20.0) * 100.0;
        let inside_part = ngon_area(32, 20.0) * 50.0;
        let expected = 100.0 * 80.0 * 50.0 + cyl_vol - inside_part;
        assert_watertight_and_volume(&result, expected, "union box-cylinder");
    }
}
