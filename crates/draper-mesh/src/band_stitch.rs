// ============================================================
// Session-45: degenerate constant-v outer ring → band stitch
// ============================================================

use draper_geometry::{Point2d, Point3d, Surface};
use crate::mesh::TriangleMesh;
use std::f64::consts::PI;

/// True when a UV polygon is a degenerate ring: every point at (nearly)
/// the same v while u wraps a substantial arc of the period. On a
/// u-periodic surface such a "polygon" has zero area and cannot bound a
/// 2D region — it is the UV image of a closed constant-v loop (e.g. a
/// full tangency circle stored as a self-loop EDGE_CURVE on a cylinder
/// or torus).
pub fn is_degenerate_v_ring(uv: &[Point2d]) -> bool {
    if uv.len() < 8 {
        return false;
    }
    let (mut u_min, mut u_max) = (f64::MAX, f64::MIN);
    let (mut v_min, mut v_max) = (f64::MAX, f64::MIN);
    for p in uv {
        u_min = u_min.min(p.u);
        u_max = u_max.max(p.u);
        v_min = v_min.min(p.v);
        v_max = v_max.max(p.v);
    }
    // Constant v (a line in UV) wrapping more than half the period —
    // only a genuine closed ring achieves this on a periodic surface
    // (an open arc would need a non-constant-v chord to close).
    v_max - v_min <= 1e-6 && u_max - u_min > PI
}

/// Stitch the "degenerate outer ring" band pattern (session-45).
///
/// A face whose OUTER boundary is a closed loop lying entirely at
/// constant v on a u-periodic surface — e.g. the exact-G1-tangency
/// circles of Zentralstaender's TRANSPORTROLLE, where the cylinder and
/// the torus fillet are both trimmed by the SAME self-loop circle — has
/// a zero-area UV polygon. The old pipeline fell into seam-split
/// (dropping the holes) and then the constant-v fan fallback, emitting
/// a FLAT centroid fan per arc: off-surface geometry that duplicates
/// the identical fan of the neighbouring face across the shared circle
/// (565 FAT fold-over pairs across 16 BREPs; sessions 42–45).
///
/// The correct interpretation is a BAND between the degenerate outer
/// loop and a u-wrapping hole loop. This function stitches the two
/// rings directly:
///   - boundary rows use ONLY the original boundary points
///     (bit-identical with the neighbouring faces through the shared
///     edge cache → watertight by construction);
///   - intermediate rows subdivide each column chord and project onto
///     the surface via `point_at` (on-surface by construction);
///   - the two-pointer walk is the session-43 radial-zipper walk
///     generalized to arbitrary (possibly meandering) hole loops;
///   - winding is derived from `surface.normal_at` and the face's
///     `forward` flag.
///
/// Returns `None` when the inputs do not match the pattern (the caller
/// falls through to the old path unchanged).
#[allow(clippy::too_many_arguments)]
pub fn try_band_stitch_degenerate_outer(
    surface: &Surface,
    outer_uv: &[Point2d],
    outer_3d: &[Point3d],
    holes_3d: &[Vec<Point3d>],
    holes_uv: &[Vec<Point2d>],
    forward: bool,
    params: &crate::triangulate::TriangulationParams,
) -> Option<TriangleMesh> {
    if !surface.is_u_periodic() {
        return None;
    }
    if !is_degenerate_v_ring(outer_uv) {
        return None;
    }
    let n_out = outer_uv.len();
    if outer_3d.len() != n_out {
        return None;
    }
    let v_outer = 0.5
        * (outer_uv.iter().map(|p| p.v).fold(f64::MAX, f64::min)
            + outer_uv.iter().map(|p| p.v).fold(f64::MIN, f64::max));

    // ── Pick the stitching hole: a wrapping closed loop strictly on one
    //    side of the outer ring in v (the band boundary). If several
    //    qualify, bail — multi-band topologies are out of scope.
    let mut picked: Option<usize> = None;
    for (hi, huv) in holes_uv.iter().enumerate() {
        let h3d = holes_3d.get(hi)?;
        if huv.len() < 8 || h3d.len() != huv.len() {
            continue;
        }
        let hu_min = huv.iter().map(|p| p.u).fold(f64::MAX, f64::min);
        let hu_max = huv.iter().map(|p| p.u).fold(f64::MIN, f64::max);
        let hv_min = huv.iter().map(|p| p.v).fold(f64::MAX, f64::min);
        let hv_max = huv.iter().map(|p| p.v).fold(f64::MIN, f64::max);
        if hu_max - hu_min <= PI {
            continue; // does not wrap
        }
        // Strictly on one side in v.
        if !(hv_max < v_outer - 1e-9 || hv_min > v_outer + 1e-9) {
            continue;
        }
        // Short way only for angular v (v-periodic surfaces): a band
        // spanning more than half the v-period would be the "long way"
        // around the tube — reject and keep the old behaviour.
        if surface.is_v_periodic() && (hv_max - v_outer).abs().max((hv_min - v_outer).abs()) > PI
        {
            continue;
        }
        if picked.is_some() {
            return None; // multiple wrapping holes — out of scope
        }
        picked = Some(hi);
    }

    // ── Orient both rings u-increasing (net u trend around the loop). ──
    let orient_u_inc = |uv: &[Point2d], pts: &[Point3d]| -> (Vec<Point2d>, Vec<Point3d>) {
        let n = uv.len();
        let mut net = 0.0_f64;
        for i in 0..n {
            let j = (i + 1) % n;
            // Single-step wrap: genuine ring steps are far below π; a
            // while-loop would double-wrap FP noise around the 2π seam
            // (observed: du = -2π+1ulp → +2π → ≈2π → false "wobble").
            let mut du = uv[j].u - uv[i].u;
            if du > PI {
                du -= 2.0 * PI;
            } else if du < -PI {
                du += 2.0 * PI;
            }
            net += du;
        }
        if net < 0.0 {
            (
                uv.iter().rev().copied().collect(),
                pts.iter().rev().copied().collect(),
            )
        } else {
            (uv.to_vec(), pts.to_vec())
        }
    };

    let (o_uv, o_3d) = orient_u_inc(outer_uv, outer_3d);
    let hi = picked?;
    let (h_uv, h_3d) = orient_u_inc(&holes_uv[hi], &holes_3d[hi]);
    let n_hole = h_uv.len();
    if n_hole < 3 {
        return None;
    }

    // ── Rotate the outer walk to its minimum u; rotate the hole walk to
    //    the vertex CYCLICALLY JUST BEFORE the outer's start meridian.
    //    Both rings live on the SAME surface, so absolute u is a shared
    //    meridian coordinate: points with equal u (mod 2π) lie in the
    //    same half-plane through the axis. Pairing by per-ring min-u
    //    would ignore the pcurve phase difference between the two edges
    //    and twist the band by a constant angle (observed −128° on the
    //    TRANSPORTROLLE fillet). The hole's start vertex is the one with
    //    the LARGEST phase distance rel = phase(β − u₀) (< 2π) — i.e.
    //    the last vertex counterclockwise before the outer's start —
    //    and its walk position is the negative offset rel − 2π, so the
    //    staircase begins meridian-aligned. ────────────────────────────
    let rotate_to_min_u = |uv: &[Point2d], pts: &[Point3d]| -> (Vec<Point2d>, Vec<Point3d>) {
        let mut best = 0usize;
        for (i, p) in uv.iter().enumerate() {
            if p.u < uv[best].u {
                best = i;
            }
        }
        let n = uv.len();
        let rot_uv: Vec<Point2d> = (0..n).map(|k| uv[(best + k) % n]).collect();
        let rot_pts: Vec<Point3d> = (0..n).map(|k| pts[(best + k) % n]).collect();
        (rot_uv, rot_pts)
    };
    let (o_uv, o_3d) = rotate_to_min_u(&o_uv, &o_3d);

    let phase = |x: f64| -> f64 {
        let mut p = x % (2.0 * PI);
        if p < 0.0 {
            p += 2.0 * PI;
        }
        p
    };
    let o0_phase = o_uv[0].u;
    let mut j_init = 0usize;
    let mut max_rel = -1.0_f64;
    for (j, p) in h_uv.iter().enumerate() {
        let rel = phase(p.u - o0_phase);
        if rel > max_rel {
            max_rel = rel;
            j_init = j;
        }
    }
    let n_h_raw = h_uv.len();
    let h_uv: Vec<Point2d> = (0..n_h_raw).map(|k| h_uv[(j_init + k) % n_h_raw]).collect();
    let h_3d: Vec<Point3d> = (0..n_h_raw)
        .map(|k| h_3d[(j_init + k) % n_h_raw])
        .collect();

    // ── Turn-position walks. The outer's positions are cumulative u
    //    from its start (fraction of 2π). The hole's positions carry
    //    the NEGATIVE PHASE OFFSET of its rotated start (absolute u in
    //    the outer's turn frame), so a shared position = the same
    //    meridian half-plane: the two-pointer then pairs radially-
    //    aligned vertices instead of twisting the band by the pcurve
    //    phase difference. ────────────────────────────────────────────
    let walk_positions = |uv: &[Point2d], offset: f64| -> Option<Vec<f64>> {
        let n = uv.len();
        let mut f = Vec::with_capacity(n);
        let mut acc = 0.0_f64;
        f.push(offset);
        for k in 1..n {
            // Single-step wrap (see orient_u_inc): a while-loop
            // double-wraps FP noise at the 2π seam.
            let mut du = uv[k].u - uv[k - 1].u;
            if du < -PI {
                du += 2.0 * PI;
            } else if du > PI {
                du -= 2.0 * PI;
            }
            if du > PI {
                return None; // non-monotone (wobble) — not a clean ring
            }
            acc += du;
            f.push(offset + acc / (2.0 * PI));
        }
        // Closing step: the CYCLIC forward step from the last point to
        // the first. (The outer walk starts at its min-u, where
        // uv[0]+2π−uv[n−1] would also work, but the hole walk starts at
        // its phase-aligned vertex — NOT its min-u — so the closing step
        // must be computed cyclically.)
        let mut closing = uv[0].u - uv[n - 1].u;
        if closing < -PI {
            closing += 2.0 * PI;
        } else if closing > PI {
            closing -= 2.0 * PI;
        }
        if closing > PI {
            return None; // the closing gap is huge — not a wrapping ring
        }
        Some(f)
    };
    let hole_off = (max_rel - 2.0 * PI) / (2.0 * PI); // ∈ (−1, 0)
    let fr_o = walk_positions(&o_uv, 0.0)?;
    let fr_h = walk_positions(&h_uv, hole_off)?;

    // ── Column walk (session-43 zipper two-pointer, wrap-inclusive). ──
    // Columns are (i, j) pairs; consecutive columns bound one strip.
    // The hole's wrap position is 1.0 + hole_off (its phase-aligned
    // turn end), the outer's is 1.0 — the walk closes on the shared
    // meridian of the start column.
    let f_out = |k: usize| -> f64 {
        if k >= n_out {
            1.0
        } else {
            fr_o[k]
        }
    };
    let f_hole = |k: usize| -> f64 {
        if k >= n_hole {
            1.0 + hole_off
        } else {
            fr_h[k]
        }
    };
    let eps = 0.25 / (n_out.max(n_hole)) as f64;
    // Each column stores (outer idx, hole idx, u_outer, u_hole) — the u
    // values are the ABSOLUTE turn positions from the wrap accessors,
    // so columns past a walk's wrap carry the +1 turn: lerping them
    // stays LOCAL. Recomputing u from the ring arrays would lose the
    // turn and sweep the lerp nearly a full revolution (the spiral
    // fold observed on the BEVORRICHTUNG cone bands).
    let mut cols: Vec<(usize, usize, f64, f64)> =
        Vec::with_capacity(n_out + n_hole + 2);
    let (mut i, mut j) = (0usize, 0usize);
    cols.push((
        0,
        0,
        o_uv[0].u + 2.0 * PI * f_out(0),
        o_uv[0].u + 2.0 * PI * f_hole(0),
    ));
    while i < n_out || j < n_hole {
        let (adv_a, adv_b) = if i < n_out && j < n_hole {
            let (da, db) = (f_out(i + 1), f_hole(j + 1));
            // session-47: hole JUMP TWIN — the next hole vertex sits at
            // the SAME turn position as the current one (a vertical slit
            // edge of a meander/castellation hole: du = 0, dv > 0).
            // Pairing BOTH twins with the same outer vertex (the old
            // hole-alone advance, chosen whenever da > db) crosses the
            // two column chords — the ray to the slit end that lies
            // closer to the outer ring falls INSIDE the fan sector of
            // the opposite run, the slit region gets covered TWICE and
            // the overlap folds back at 172-180° (measured on the final
            // mesh: TRANSP face-26 slit 174.06/175.21/176.47/179.84°,
            // BNO face-2 castellation 172.75/174.87/180.00°). Pair the
            // second twin with the NEXT outer vertex instead: the chords
            // (outer_i → slit end A) and (outer_{i+1} → slit end B)
            // preserve the u-order at every level once the next outer is
            // at/beyond the slit meridian (da ≥ db) — exactly the case
            // where the old rule advanced the hole alone. When the next
            // outer is still before the slit (da < db), the plain
            // outer-alone advance already defers the twin correctly
            // (same-hole fan columns), so only the (false, true) branch
            // changes.
            let jump_twin = j + 1 < n_hole
                && (f_hole(j + 1) - f_hole(j)).abs() <= 1e-12
                && (h_uv[j + 1].v - h_uv[j].v).abs() > 1e-9;
            if jump_twin && da >= db - eps {
                (true, true)
            } else if (da - db).abs() <= eps {
                (true, true)
            } else if da < db {
                (true, false)
            } else {
                (false, true)
            }
        } else if i < n_out {
            (true, false)
        } else if j < n_hole {
            (false, true)
        } else {
            break;
        };
        if adv_a {
            i += 1;
        }
        if adv_b {
            j += 1;
        }
        cols.push((
            i % n_out,
            j % n_hole,
            o_uv[0].u + 2.0 * PI * f_out(i),
            o_uv[0].u + 2.0 * PI * f_hole(j),
        ));
    }
    // The walk ends at the wrap column == the first column; drop it if
    // it duplicates the start (it always does for a full walk).
    if cols.len() >= 2
        && (cols[cols.len() - 1].0, cols[cols.len() - 1].1) == (cols[0].0, cols[0].1)
    {
        cols.pop();
    }
    if cols.len() < 2 {
        return None;
    }

    // ── Intermediate-row density from the adaptive sampler. ──────────
    let v_lo = v_outer.min(h_uv.iter().map(|p| p.v).fold(f64::MAX, f64::min));
    let v_hi = v_outer.max(h_uv.iter().map(|p| p.v).fold(f64::MIN, f64::max));
    let (_, n_v) = crate::adaptive::required_samples(
        surface,
        0.0,
        2.0 * PI,
        v_lo,
        v_hi,
        params.max_deviation,
        params.detail_level,
    );
    let k_rows = n_v.clamp(1, 64);

    // ── Column polylines: level 0 = original outer vertex, level k_rows
    //    = original hole vertex, between = point_at on the lerped chord.
    //    Boundary vertices may be added several times (a ring vertex is
    //    referenced by consecutive columns) — the converter's position
    //    dedup unifies them; the zero-area filter drops the degenerate
    //    wedges that coincidence produces. ─────────────────────────────
    let mut mesh = TriangleMesh::new();
    let mut uvs: Vec<Point2d> = Vec::new();
    let mut outer_used = vec![false; n_out];
    let mut hole_used = vec![false; n_hole];
    let mut column_pts: Vec<Vec<u32>> = Vec::with_capacity(cols.len());

    for &(ci, cj, u_o, u_h) in &cols {
        let v_h = h_uv[cj].v;
        let mut pts = Vec::with_capacity(k_rows + 1);
        let io = mesh.add_vertex(o_3d[ci]);
        uvs.push(Point2d::new(u_o, v_outer));
        outer_used[ci] = true;
        pts.push(io);
        for l in 1..k_rows {
            let t = l as f64 / k_rows as f64;
            let p = surface.point_at(u_o + t * (u_h - u_o), v_outer + t * (v_h - v_outer));
            let idx = mesh.add_vertex(p);
            uvs.push(Point2d::new(
                u_o + t * (u_h - u_o),
                v_outer + t * (v_h - v_outer),
            ));
            pts.push(idx);
        }
        let ih = mesh.add_vertex(h_3d[cj]);
        uvs.push(Point2d::new(u_h, v_h));
        hole_used[cj] = true;
        pts.push(ih);
        column_pts.push(pts);
    }

    // ── Strips between consecutive columns (cyclic). ─────────────────
    // session-46: PER-TRIANGLE orientation against the surface normal.
    // The (P_l, Q_l, Q_{l+1})/(P_l, Q_{l+1}, P_{l+1}) patterns assume
    // every strip quad has the same UV orientation — true for normal
    // bands, FALSE at hole-meander vertical jumps: there the two columns
    // share the outer vertex and their holes sit at the same u, shearing
    // the quad past vertical so its UV signed area flips sign. The
    // emitted triangles then have MIXED winding (measured: jump-strip
    // triangle +radial, adjacent deep-strip triangle −radial on BNO
    // face 2), and the single global flip below cannot fix both classes
    // — the result is back-to-back fold pairs at ~179.6° along the
    // lower half of each jump column (88 intra-face Cyl|Cyl pairs on
    // Zentralstaender). Orienting every triangle individually makes the
    // winding consistent everywhere; for all-normal bands the outcome
    // is bit-identical to the previous uniform global flip.
    // session-47: SEAM-AWARE UV centroid. The wrap strip (last column
    // → first column, cyclic) stores the first column's vertices at
    // their raw turn-0 u while the last column's sit at turn ≈1 (u+2π−δ):
    // the RAW mean of such a triangle's u lands ~π away, on the opposite
    // meridian, where `normal_at` returns a normal unrelated to the
    // triangle's true location — the winding decision then flips
    // incorrectly and the whole wrap strip comes out inverted. Measured:
    // 171-177° WINDING-FLIP fold pairs along the first and last columns
    // of EVERY band-stitched face (the remaining 28 Cyl|Cyl intra-face
    // pairs: TRANSP 16 + BNO 12). Unwrapping each vertex's u onto the
    // branch continuous with the first vertex (single ±2π step — band
    // steps are ≪ π) puts the centroid back inside the strip. For every
    // non-wrap triangle the unwrap is the identity → bit-identical
    // winding decisions, unchanged meshes everywhere else.
    let centroid_uv = |x: u32, y: u32, z: u32| -> (f64, f64) {
        let p0 = uvs[x as usize];
        let p1 = uvs[y as usize];
        let p2 = uvs[z as usize];
        let unwrap = |u: f64| -> f64 {
            let mut d = u - p0.u;
            if d > PI {
                d -= 2.0 * PI;
            } else if d < -PI {
                d += 2.0 * PI;
            }
            p0.u + d
        };
        let u1 = unwrap(p1.u);
        let u2 = unwrap(p2.u);
        ((p0.u + u1 + u2) / 3.0, (p0.v + p1.v + p2.v) / 3.0)
    };

    let emit = |mesh: &mut TriangleMesh, x: u32, y: u32, z: u32| {
        if x == y || y == z || x == z {
            return;
        }
        // Zero-area filter (coincident columns / degenerate wedges).
        let a = mesh.vertices[x as usize];
        let b = mesh.vertices[y as usize];
        let c = mesh.vertices[z as usize];
        let e1 = (b.x - a.x, b.y - a.y, b.z - a.z);
        let e2 = (c.x - a.x, c.y - a.y, c.z - a.z);
        let n = (
            e1.1 * e2.2 - e1.2 * e2.1,
            e1.2 * e2.0 - e1.0 * e2.2,
            e1.0 * e2.1 - e1.1 * e2.0,
        );
        let n2 = n.0 * n.0 + n.1 * n.1 + n.2 * n.2;
        if n2 < 1e-20 {
            return;
        }
        // Surface normal at the triangle's (seam-aware) UV centroid.
        let (mu, mv) = centroid_uv(x, y, z);
        let sn = surface.normal_at(mu, mv);
        let mut dot = n.0 * sn.x + n.1 * sn.y + n.2 * sn.z;
        if !forward {
            dot = -dot;
        }
        if dot < 0.0 {
            mesh.add_triangle(x, z, y);
        } else {
            mesh.add_triangle(x, y, z);
        }
    };

    let ncols = cols.len();
    // session-47: slit-strip triangulation. Two emission modes:
    //
    // • Level-tied strips (both columns end on the same hole run —
    //   v_hole identical): the CLASSIC fixed pattern
    //   (P_l, Q_l, Q_{l+1}) / (P_l, Q_{l+1}, P_{l+1}) — bit-identical
    //   to the session-45 emission for every normal band.
    //
    // • Slit strips (the columns end at the two ends of a vertical
    //   slit edge — v_hole differs, e.g. the spread strips): the old
    //   fixed pattern emits sheared/bowtie quads whose winding flips
    //   against the neighbours (172-180° folds, TRANSP/BNO). The
    //   correct triangulation of the two-chain region (outer edge on
    //   top, the two column chords on the sides, the vertical slit at
    //   the bottom) is the TWO-FAN decomposition:
    //     (1) fan from the DEEP column's TOP (its outer vertex) over
    //         the SHALLOW column's polyline, and
    //     (2) fan from the SHALLOW column's BOTTOM (the near slit
    //         twin) over the DEEP column's polyline,
    //   split by the single internal diagonal deep_top→shallow_bottom.
    //   Validity: the deep-top diagonals to the shallow chain stay
    //   inside (the deep top lies beyond the shallow chord's top in u,
    //   and two straight lines from it can only meet the shallow chord
    //   at their shared endpoint); the near-twin diagonals to the deep
    //   chain stay inside (they never cross to the far side of the
    //   slit or the deep chord). Both fans are provably inside for
    //   both slit orientations (P-deep and Q-deep). Long diagonals
    //   from the SHALLOW top to the deep chain — the fold driver
    //   (172-180°, measured) — are never emitted.
    for c in 0..ncols {
        let p = &column_pts[c];
        let q = &column_pts[(c + 1) % ncols];
        let v_end_p = uvs[p[p.len() - 1] as usize].v;
        let v_end_q = uvs[q[q.len() - 1] as usize].v;
        if (v_end_p - v_end_q).abs() <= 1e-12 {
            for l in 0..k_rows {
                emit(&mut mesh, p[l], q[l], q[l + 1]);
                emit(&mut mesh, p[l], q[l + 1], p[l + 1]);
            }
        } else {
            // slit strip: pick the deep column (hole end farther from
            // the outer ring)
            let dp = (v_end_p - v_outer).abs();
            let dq = (v_end_q - v_outer).abs();
            let (deep, shallow) = if dp >= dq { (p, q) } else { (q, p) };
            let dtop = deep[0];
            let sbot = shallow[shallow.len() - 1];
            for i in 0..shallow.len() - 1 {
                emit(&mut mesh, dtop, shallow[i], shallow[i + 1]);
            }
            for i in 0..deep.len() - 1 {
                emit(&mut mesh, sbot, deep[i], deep[i + 1]);
            }
        }
    }

    if mesh.triangles.is_empty() {
        return None;
    }

    // Every boundary vertex must be used (watertightness contract).
    if outer_used.iter().any(|&u| !u) || hole_used.iter().any(|&u| !u) {
        log::warn!(
            "BAND_STITCH: boundary vertices unused (outer {}/{}, hole {}/{}) — rejecting",
            outer_used.iter().filter(|&&u| u).count(),
            n_out,
            hole_used.iter().filter(|&&u| u).count(),
            n_hole
        );
        return None;
    }

    // ── Orientation: align the first triangle with the surface normal
    //    (accounting for `forward`). ──────────────────────────────────
    {
        let t0 = mesh.triangles[0];
        let (a, b, c) = (
            mesh.vertices[t0[0] as usize],
            mesh.vertices[t0[1] as usize],
            mesh.vertices[t0[2] as usize],
        );
        let e1 = (b.x - a.x, b.y - a.y, b.z - a.z);
        let e2 = (c.x - a.x, c.y - a.y, c.z - a.z);
        let n = (
            e1.1 * e2.2 - e1.2 * e2.1,
            e1.2 * e2.0 - e1.0 * e2.2,
            e1.0 * e2.1 - e1.1 * e2.0,
        );
        // session-47: seam-aware centroid here too (t0 is normally a
        // strip-0 triangle where the unwrap is the identity; kept for
        // robustness so this safety flip can never misfire on a wrap
        // triangle if emission order ever changes).
        let (mu, mv) = centroid_uv(t0[0], t0[1], t0[2]);
        let sn = surface.normal_at(mu, mv);
        let mut dot = n.0 * sn.x + n.1 * sn.y + n.2 * sn.z;
        if !forward {
            dot = -dot;
        }
        if dot < 0.0 {
            for t in mesh.triangles.iter_mut() {
                *t = [t[0], t[2], t[1]];
            }
        }
    }

    log::info!(
        "BAND_STITCH: degenerate constant-v ring stitched (outer {} + hole {} pts, {} columns × {} rows → {} tris, forward={})",
        n_out,
        n_hole,
        ncols,
        k_rows,
        mesh.triangle_count(),
        forward
    );

    // session-46 diagnostics (read-only, env-gated): meander shape of the
    // stitching hole (v-jumps between consecutive hole vertices are the
    // wedge-fold driver) and the band parameters.
    if std::env::var("DRAPPER_DUMP_BAND").is_ok() {
        eprintln!(
            "BAND: outer={} hole={} cols={} k_rows={} tris={} v_outer={:.4}",
            n_out,
            n_hole,
            ncols,
            k_rows,
            mesh.triangle_count(),
            v_outer
        );
        let mut hv = String::new();
        for p in h_uv.iter() {
            hv.push_str(&format!("({:.4},{:.4})", p.u, p.v));
        }
        eprintln!("BANDHOLE: {}", hv);
        let mut max_jump = 0.0f64;
        for k in 0..n_hole {
            let a = h_uv[k];
            let b = h_uv[(k + 1) % n_hole];
            let mut du = b.u - a.u;
            if du > PI {
                du -= 2.0 * PI;
            } else if du < -PI {
                du += 2.0 * PI;
            }
            let jump = (du * du + (b.v - a.v) * (b.v - a.v)).sqrt();
            max_jump = max_jump.max(jump);
        }
        eprintln!("BANDHOLEMAXJUMP: {:.4}", max_jump);
    }
    // session-46 diagnostics (read-only, env-gated): full band topology —
    // vertices (with uv), columns (outer/hole idx + turn positions) and
    // emitted triangles (band-local indices), for offline reconstruction
    // of wedge-fold provenance at hole-meander corners.
    if std::env::var("DRAPPER_DUMP_BAND2").is_ok() {
        eprintln!("BANDBEGIN: outer={} hole={} k_rows={}", n_out, n_hole, k_rows);
        for (vi, p) in mesh.vertices.iter().enumerate() {
            eprintln!(
                "BANDVERT: {} ({:.6},{:.6},{:.6}) uv=({:.6},{:.6})",
                vi, p.x, p.y, p.z, uvs[vi].u, uvs[vi].v
            );
        }
        for (ci, &(oi, hj, uo, uh)) in cols.iter().enumerate() {
            let vp = &column_pts[ci];
            eprintln!(
                "BANDCOL: {} outer={} hole={} uo={:.6} uh={:.6} pts={:?}",
                ci, oi, hj, uo, uh, vp
            );
        }
        for (ti, t) in mesh.triangles.iter().enumerate() {
            eprintln!("BANDTRI: {} {},{},{}", ti, t[0], t[1], t[2]);
        }
    }

    Some(mesh)
}

#[cfg(test)]
mod tests {
    use super::*;
    use draper_geometry::surface::{CylinderSurface, TorusSurface};
    use draper_geometry::{Direction3d, Point3d};

    /// Ring on a Z-axis cylinder at height h: n points from phase u0.
    fn cyl_ring(s: &Surface, h: f64, n: usize, u0: f64) -> (Vec<Point3d>, Vec<Point2d>) {
        let mut pts = Vec::with_capacity(n);
        let mut uvs = Vec::with_capacity(n);
        for k in 0..n {
            let u = u0 + 2.0 * PI * k as f64 / n as f64;
            pts.push(s.point_at(u, h));
            uvs.push(Point2d::new(u, h));
        }
        (pts, uvs)
    }

    fn params() -> crate::triangulate::TriangulationParams {
        crate::triangulate::TriangulationParams::default()
    }

    fn contains(mesh: &TriangleMesh, p: &Point3d) -> bool {
        mesh.vertices.iter().any(|q| {
            (q.x - p.x).abs() < 1e-12
                && (q.y - p.y).abs() < 1e-12
                && (q.z - p.z).abs() < 1e-12
        })
    }

    #[test]
    fn test_degenerate_v_ring_detector() {
        let s = Surface::Cylinder(CylinderSurface::new_z(5.0));
        // Full circle at constant v → degenerate ring.
        let (_, uvs) = cyl_ring(&s, 3.0, 16, 0.0);
        assert!(is_degenerate_v_ring(&uvs));
        // Constant-u line (hex-nut side face) → NOT the pattern.
        let uvs_cu: Vec<Point2d> = (0..16).map(|k| Point2d::new(1.0, k as f64 * 0.1)).collect();
        assert!(!is_degenerate_v_ring(&uvs_cu));
        // Partial arc at constant v (u span < π) → NOT the pattern.
        let uvs_arc: Vec<Point2d> = (0..8).map(|k| Point2d::new(k as f64 * 0.2, 1.0)).collect();
        assert!(!is_degenerate_v_ring(&uvs_arc));
        // Varying v → NOT the pattern.
        let uvs_2d: Vec<Point2d> =
            (0..16).map(|k| Point2d::new(k as f64 * 0.4, (k % 4) as f64)).collect();
        assert!(!is_degenerate_v_ring(&uvs_2d));
        // Too few points → NOT the pattern.
        let uvs_small = vec![
            Point2d::new(0.0, 1.0),
            Point2d::new(1.0, 1.0),
            Point2d::new(2.0, 1.0),
        ];
        assert!(!is_degenerate_v_ring(&uvs_small));
    }

    #[test]
    fn test_band_stitch_cylinder_phase_aligned() {
        // Outer ring at h=5 (32 pts) and hole ring at h=1 with a
        // DIFFERENT sampling count and a pcurve phase offset — the exact
        // conditions that twisted the TRANSPORTROLLE fillet by −128°
        // when rings were paired by per-ring min-u instead of absolute
        // u phase.
        let s = Surface::Cylinder(CylinderSurface::new_z(5.0));
        let (o3, ouv) = cyl_ring(&s, 5.0, 32, 0.0);
        let (h3, huv) = cyl_ring(&s, 1.0, 16, 0.11);
        let mesh = try_band_stitch_degenerate_outer(
            &s, &ouv, &o3, &[h3.clone()], &[huv.clone()], true, &params(),
        )
        .expect("band stitch must fire for the cylinder band");
        assert!(!mesh.triangles.is_empty());
        // Every original boundary point survives (watertightness contract).
        for p in o3.iter().chain(h3.iter()) {
            assert!(contains(&mesh, p), "boundary point missing: {p:?}");
        }
        // Phase alignment: every outer vertex's nearest hole vertex by
        // XY angle is within one hole step (2π/16) — no band twist.
        let step = 2.0 * PI / 16.0;
        for p in &o3 {
            let ang = p.y.atan2(p.x);
            let best = h3
                .iter()
                .map(|q| {
                    let a = q.y.atan2(q.x);
                    let mut d = (a - ang).abs();
                    if d > PI {
                        d = 2.0 * PI - d;
                    }
                    d
                })
                .fold(f64::MAX, f64::min);
            assert!(
                best <= step + 1e-9,
                "column twist: nearest hole vertex {best} rad away (step {step})"
            );
        }
    }

    #[test]
    fn test_band_stitch_seam_wrap_consistency() {
        // session-47 regression: the wrap strip (last column → first
        // column) has triangles whose RAW u values straddle the 2π turn
        // (last column at u0+2π−δ, first at u0). With the raw UV
        // centroid the surface normal is sampled on the opposite
        // meridian, the winding decision flips, and the seam columns
        // come out inverted (171-177° WINDING-FLIP folds). Verify:
        // (a) every interior edge traversed in OPPOSITE directions by
        //     its two triangles (topological winding consistency),
        // (b) no interior-edge dihedral above 170° (no folds).
        let s = Surface::Cylinder(CylinderSurface::new_z(5.0));
        let (o3, ouv) = cyl_ring(&s, 5.0, 32, 0.0);
        // Phase-offset hole with a different sampling count (the exact
        // conditions of the TRANSPORTROLLE/BNO seam folds).
        let (h3, huv) = cyl_ring(&s, 1.0, 24, 0.3);
        let mesh = try_band_stitch_degenerate_outer(
            &s, &ouv, &o3, &[h3.clone()], &[huv.clone()], true, &params(),
        )
        .expect("band must stitch");
        assert!(mesh.triangles.len() >= 2 * 32);

        use std::collections::HashMap;
        let mut edges: HashMap<(u32, u32), Vec<(usize, u32, u32)>> = HashMap::new();
        for (ti, t) in mesh.triangles.iter().enumerate() {
            for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                let key = if a < b { (a, b) } else { (b, a) };
                edges.entry(key).or_default().push((ti, a, b));
            }
        }
        let tri_normal = |t: &[u32; 3]| -> Option<(f64, f64, f64)> {
            let a = mesh.vertices[t[0] as usize];
            let b = mesh.vertices[t[1] as usize];
            let c = mesh.vertices[t[2] as usize];
            let e1 = (b.x - a.x, b.y - a.y, b.z - a.z);
            let e2 = (c.x - a.x, c.y - a.y, c.z - a.z);
            let n = (
                e1.1 * e2.2 - e1.2 * e2.1,
                e1.2 * e2.0 - e1.0 * e2.2,
                e1.0 * e2.1 - e1.1 * e2.0,
            );
            let l = (n.0 * n.0 + n.1 * n.1 + n.2 * n.2).sqrt();
            if l < 1e-12 {
                None
            } else {
                Some((n.0 / l, n.1 / l, n.2 / l))
            }
        };
        let mut interior = 0usize;
        for (_e, ts) in &edges {
            if ts.len() != 2 {
                continue;
            }
            interior += 1;
            let (t0, a0, b0) = ts[0];
            let (t1, a1, b1) = ts[1];
            assert!(
                a0 == b1 && b0 == a1,
                "winding-flip on interior edge ({a0},{b0}): tris {t0} and {t1} traverse it in the same direction"
            );
            let (Some(n0), Some(n1)) = (tri_normal(&mesh.triangles[t0]), tri_normal(&mesh.triangles[t1]))
            else {
                continue;
            };
            let dot = n0.0 * n1.0 + n0.1 * n1.1 + n0.2 * n1.2;
            let ang = dot.clamp(-1.0, 1.0).acos().to_degrees();
            assert!(
                ang <= 170.0,
                "fold {ang:.2}° on interior edge ({a0},{b0}) between tris {t0} and {t1}"
            );
        }
        // A stitched band of 32 columns must be mostly interior —
        // sanity that the edge map is not degenerate.
        assert!(interior > 100, "interior edge count {interior} too small");
    }

    #[test]
    fn test_band_stitch_castellation_high_slits() {
        // session-47: BNO face-2 proportions — the outer ring sits CLOSE
        // to the near run and FAR from the deep run (band height 0.2 vs
        // 4.1), with several slit edges in sequence (castellation teeth).
        // The spread + internal-diagonal emission must keep the winding
        // topologically consistent and fold-free through every tooth.
        let s = Surface::Cylinder(CylinderSurface::new_z(5.0));
        let (o3, ouv) = cyl_ring(&s, 5.0, 32, 0.0);
        let (v_near, v_deep) = (4.8, 0.9);
        let mut h3 = Vec::new();
        let mut huv = Vec::new();
        let mut push = |u: f64, v: f64| {
            h3.push(s.point_at(u, v));
            huv.push(Point2d::new(u, v));
        };
        // Comb: deep teeth with short near bridges over the gaps; the
        // slit edges are EXACTLY vertical (du = 0 — as measured on the
        // BNO castellation: BANDHOLEMAXJUMP=4.1 at du=0); the deep
        // background wraps around and closes the loop.
        //   deep [0.4, 1.6] | up@1.6 (twin) | near (1.8, 2.0) |
        //   down@2.0 (twin) | deep [2.2, 3.2] | up@3.2 (twin) |
        //   near (3.4, 3.6) | down@3.6 (twin) | deep [3.725, 6.6]
        let n_arc = 6usize;
        for k in 0..=n_arc {
            push(0.4 + 1.2 * k as f64 / n_arc as f64, v_deep);
        }
        push(1.6, v_near); // vertical slit up (twin of (1.6, deep))
        push(1.8, v_near);
        push(2.0, v_near);
        push(2.0, v_deep); // vertical slit down (twin of (2.0, near))
        for k in 1..=n_arc {
            push(2.0 + 1.2 * k as f64 / n_arc as f64, v_deep);
        }
        push(3.2, v_near); // vertical slit up
        push(3.4, v_near);
        push(3.6, v_near);
        push(3.6, v_deep); // vertical slit down
        for k in 1..=24 {
            push(3.6 + 3.0 * k as f64 / 24.0, v_deep);
        }
        let mesh = try_band_stitch_degenerate_outer(
            &s, &ouv, &o3, &[h3.clone()], &[huv.clone()], true, &params(),
        )
        .expect("castellation band must stitch");
        assert!(!mesh.triangles.is_empty());
        // consistency + fold checks (same as the meander test)
        use std::collections::HashMap;
        let mut edges: HashMap<(u32, u32), Vec<(usize, u32, u32)>> = HashMap::new();
        for (ti, t) in mesh.triangles.iter().enumerate() {
            for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                let key = if a < b { (a, b) } else { (b, a) };
                edges.entry(key).or_default().push((ti, a, b));
            }
        }
        let tri_normal = |t: &[u32; 3]| -> Option<(f64, f64, f64)> {
            let a = mesh.vertices[t[0] as usize];
            let b = mesh.vertices[t[1] as usize];
            let c = mesh.vertices[t[2] as usize];
            let e1 = (b.x - a.x, b.y - a.y, b.z - a.z);
            let e2 = (c.x - a.x, c.y - a.y, c.z - a.z);
            let n = (
                e1.1 * e2.2 - e1.2 * e2.1,
                e1.2 * e2.0 - e1.0 * e2.2,
                e1.0 * e2.1 - e1.1 * e2.0,
            );
            let l = (n.0 * n.0 + n.1 * n.1 + n.2 * n.2).sqrt();
            if l < 1e-12 {
                None
            } else {
                Some((n.0 / l, n.1 / l, n.2 / l))
            }
        };
        let mut interior = 0usize;
        for (_e, ts) in &edges {
            if ts.len() != 2 {
                continue;
            }
            interior += 1;
            let (t0, a0, b0) = ts[0];
            let (t1, a1, b1) = ts[1];
            assert!(
                a0 == b1 && b0 == a1,
                "castellation winding-flip on interior edge ({a0},{b0}): tris {t0}/{t1}"
            );
            let (Some(n0), Some(n1)) =
                (tri_normal(&mesh.triangles[t0]), tri_normal(&mesh.triangles[t1]))
            else {
                continue;
            };
            let dot = n0.0 * n1.0 + n0.1 * n1.1 + n0.2 * n1.2;
            let ang = dot.clamp(-1.0, 1.0).acos().to_degrees();
            assert!(
                ang <= 170.0,
                "castellation fold {ang:.2}° on interior edge ({a0},{b0}): tris {t0}/{t1}"
            );
        }
        assert!(interior > 100, "interior edge count {interior} too small");
    }

    #[test]
    fn test_band_stitch_meander_hole() {
        // Hole wraps with a square-wave v profile plus two vertical
        // jumps (the TRANSPORTROLLE cuff window topology).
        let s = Surface::Cylinder(CylinderSurface::new_z(5.0));
        let (o3, ouv) = cyl_ring(&s, 5.0, 32, 0.7);
        let u_start = 2.3_f64;
        let n_arc = 12usize;
        let mut h3 = Vec::new();
        let mut huv = Vec::new();
        for k in 0..=n_arc {
            let u = u_start + PI * k as f64 / n_arc as f64;
            h3.push(s.point_at(u, 2.0));
            huv.push(Point2d::new(u, 2.0));
        }
        // vertical jump down at u_start + π
        h3.push(s.point_at(u_start + PI, 1.0));
        huv.push(Point2d::new(u_start + PI, 1.0));
        for k in 1..=n_arc {
            let u = u_start + PI + PI * k as f64 / n_arc as f64;
            h3.push(s.point_at(u, 1.0));
            huv.push(Point2d::new(u, 1.0));
        }
        let mesh = try_band_stitch_degenerate_outer(
            &s, &ouv, &o3, &[h3.clone()], &[huv.clone()], true, &params(),
        )
        .expect("meander band must stitch");
        assert!(!mesh.triangles.is_empty());
        for p in h3.iter() {
            assert!(contains(&mesh, p), "meander point missing: {p:?}");
        }
        for p in o3.iter() {
            assert!(contains(&mesh, p), "outer point missing: {p:?}");
        }
        // session-47: with the jump-twin spread the vertical slit edges
        // (du = 0, dv > 0) must no longer double-cover the slit region:
        // winding stays topologically consistent and no interior edge
        // folds past 170°.
        {
            use std::collections::HashMap;
            let mut edges: HashMap<(u32, u32), Vec<(usize, u32, u32)>> = HashMap::new();
            for (ti, t) in mesh.triangles.iter().enumerate() {
                for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                    let key = if a < b { (a, b) } else { (b, a) };
                    edges.entry(key).or_default().push((ti, a, b));
                }
            }
            let tri_normal = |t: &[u32; 3]| -> Option<(f64, f64, f64)> {
                let a = mesh.vertices[t[0] as usize];
                let b = mesh.vertices[t[1] as usize];
                let c = mesh.vertices[t[2] as usize];
                let e1 = (b.x - a.x, b.y - a.y, b.z - a.z);
                let e2 = (c.x - a.x, c.y - a.y, c.z - a.z);
                let n = (
                    e1.1 * e2.2 - e1.2 * e2.1,
                    e1.2 * e2.0 - e1.0 * e2.2,
                    e1.0 * e2.1 - e1.1 * e2.0,
                );
                let l = (n.0 * n.0 + n.1 * n.1 + n.2 * n.2).sqrt();
                if l < 1e-12 {
                    None
                } else {
                    Some((n.0 / l, n.1 / l, n.2 / l))
                }
            };
            let mut interior = 0usize;
            for (_e, ts) in &edges {
                if ts.len() != 2 {
                    continue;
                }
                interior += 1;
                let (t0, a0, b0) = ts[0];
                let (t1, a1, b1) = ts[1];
                assert!(
                    a0 == b1 && b0 == a1,
                    "meander winding-flip on interior edge ({a0},{b0}): tris {t0}/{t1}"
                );
                let (Some(n0), Some(n1)) =
                    (tri_normal(&mesh.triangles[t0]), tri_normal(&mesh.triangles[t1]))
                else {
                    continue;
                };
                let dot = n0.0 * n1.0 + n0.1 * n1.1 + n0.2 * n1.2;
                let ang = dot.clamp(-1.0, 1.0).acos().to_degrees();
                assert!(
                    ang <= 170.0,
                    "meander fold {ang:.2}° on interior edge ({a0},{b0}): tris {t0}/{t1}"
                );
            }
            assert!(interior > 100, "interior edge count {interior} too small");
        }
    }

    #[test]
    fn test_band_stitch_torus_on_surface() {
        // Torus fillet: outer = tangency circle (v=0), hole = tube
        // equator (v=π/2). Intermediate rows must lie ON the torus.
        let t = Surface::Torus(TorusSurface {
            center: Point3d::ORIGIN,
            axis: Direction3d::Z,
            major_radius: 14.0,
            minor_radius: 5.0,
            x_dir: Direction3d::X,
        });
        let ring = |v: f64, n: usize, u0: f64| -> (Vec<Point3d>, Vec<Point2d>) {
            let mut pts = Vec::with_capacity(n);
            let mut uvs = Vec::with_capacity(n);
            for k in 0..n {
                let u = u0 + 2.0 * PI * k as f64 / n as f64;
                pts.push(t.point_at(u, v));
                uvs.push(Point2d::new(u, v));
            }
            (pts, uvs)
        };
        let (o3, ouv) = ring(0.0, 24, 0.0);
        let (h3, huv) = ring(PI / 2.0, 17, 1.3);
        let mesh = try_band_stitch_degenerate_outer(
            &t, &ouv, &o3, &[h3.clone()], &[huv.clone()], true, &params(),
        )
        .expect("torus band must stitch");
        assert!(!mesh.triangles.is_empty());
        for p in &mesh.vertices {
            let r_xy = (p.x * p.x + p.y * p.y).sqrt();
            let d = ((r_xy - 14.0).powi(2) + p.z * p.z).sqrt() - 5.0;
            assert!(
                d.abs() < 1e-9,
                "vertex off the torus by {d:.3e}: {p:?}"
            );
        }
    }

    #[test]
    fn test_band_stitch_rejects_non_pattern() {
        // Non-degenerate outer (varying v) → None (old path unchanged).
        let s = Surface::Cylinder(CylinderSurface::new_z(5.0));
        let (o3, ouv) = cyl_ring(&s, 5.0, 32, 0.0);
        let mut uv2 = ouv.clone();
        for (k, p) in uv2.iter_mut().enumerate() {
            p.v += 0.5 * (k as f64).sin().abs();
        }
        let (h3, huv) = cyl_ring(&s, 1.0, 16, 0.0);
        assert!(try_band_stitch_degenerate_outer(
            &s, &uv2, &o3, &[h3], &[huv], true, &params(),
        )
        .is_none());
    }

    #[test]
    fn test_degenerate_ring_no_hole_yields_empty() {
        // A closed constant-v ring with NO holes on a cylinder is a
        // zero-extent degenerate face — the old flat centroid fan
        // duplicated the neighbour's coverage (the July family); the
        // pipeline must emit NOTHING for it.
        let s = Surface::Cylinder(CylinderSurface::new_z(5.0));
        let (o3, ouv) = cyl_ring(&s, 3.0, 32, 0.0);
        let mesh = crate::parametric_domain::triangulate_surface_consistent(
            &s, &o3, &ouv, &[], &[], true, &params(),
        );
        assert!(
            mesh.triangles.is_empty(),
            "degenerate zero-extent face must emit nothing, got {} tris",
            mesh.triangles.len()
        );
    }
}
