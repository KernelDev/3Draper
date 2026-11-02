// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Shared surface-justified dihedral exemption (session-46, direction (c)
//! of session-44, generalized).
//!
//! The >170° dihedral gate flags pairs of adjacent triangles whose raw
//! normals are near-opposite. But when the two underlying SURFACES meet
//! at a shallow angle — exact G1 tangency (tangent double-back of the
//! boundary, e.g. a fillet tangent to a flange plane) gives 180°, a
//! shallow knife edge gives e.g. 172° — a FAITHFUL mesh necessarily
//! reproduces that dihedral. Such pairs are not mesh bugs.
//!
//! A pair is exempt only when ALL guards hold:
//!   (a) topologically consistent winding (shared edge traversed in
//!       opposite directions — WINDING-FLIP pairs stay flagged)
//!   (b) NOT coincident (both centroids within 1e-6 of the other
//!       surface — parallel-plane flange duplicates stay flagged)
//!   (c) different faces with geometrically distinct surfaces (intra-
//!       face / same-surface winding bugs stay flagged)
//!   (d) mesh dihedral <= surface-surface angle at the rim + 2° — the
//!       mesh is not folded more than the surfaces actually are
//!       (Plane|Cone needles: mesh 180° vs surface 149° stay flagged)
//!   (e) glue guard: each centroid stays near the other surface
//!       (session-44 ear-fan duplicate caps at a tangency circle had
//!       cross-distances 1.6-3.7 — they stay flagged)
//!
//! Surface normals are evaluated at the shared-edge ENDPOINTS (bit-exact
//! rim points ON both surfaces). The midpoint of a chord of the tangency
//! circle sits inside the circle and the projected torus/cone normal
//! there picks up a chord-sagitta tilt (measured 3.8° on B_WELLE),
//! masking exact tangency.
//!
//! Shared by tools/src/bin/fold_face_probe.rs (diagnostic classification)
//! and tools/src/bin/angle_check.rs (the release gate) via
//! `#[path = "../surf_exempt.rs"] mod surf_exempt;`.

use draper_geometry::{Point3d, Surface};

/// Distance from a point to an analytic surface (plane/cylinder/cone/
/// torus; None for everything else). The surface must ALREADY be in the
/// same space as the point (world).
pub fn surface_distance(surf: &Surface, p: &Point3d) -> Option<f64> {
    match surf {
        Surface::Plane(pl) => {
            let d = [p.x - pl.origin.x, p.y - pl.origin.y, p.z - pl.origin.z];
            Some((d[0] * pl.normal.x + d[1] * pl.normal.y + d[2] * pl.normal.z).abs())
        }
        Surface::Cylinder(cy) => {
            let d = [p.x - cy.origin.x, p.y - cy.origin.y, p.z - cy.origin.z];
            let a = [cy.axis.x, cy.axis.y, cy.axis.z];
            let t = d[0] * a[0] + d[1] * a[1] + d[2] * a[2];
            let perp = [d[0] - t * a[0], d[1] - t * a[1], d[2] - t * a[2]];
            Some(
                ((perp[0] * perp[0] + perp[1] * perp[1] + perp[2] * perp[2]).sqrt()
                    - cy.radius)
                    .abs(),
            )
        }
        Surface::Cone(co) => {
            let d = [p.x - co.origin.x, p.y - co.origin.y, p.z - co.origin.z];
            let a = [co.axis.x, co.axis.y, co.axis.z];
            let t = d[0] * a[0] + d[1] * a[1] + d[2] * a[2];
            let perp = [d[0] - t * a[0], d[1] - t * a[1], d[2] - t * a[2]];
            let r = (perp[0] * perp[0] + perp[1] * perp[1] + perp[2] * perp[2]).sqrt();
            let tan = co.half_angle.tan();
            let expect = if co.expanding {
                co.radius + t * tan
            } else {
                co.radius - t * tan
            };
            Some((r - expect).abs())
        }
        Surface::Torus(to) => {
            let d = [p.x - to.center.x, p.y - to.center.y, p.z - to.center.z];
            let a = [to.axis.x, to.axis.y, to.axis.z];
            let h = d[0] * a[0] + d[1] * a[1] + d[2] * a[2];
            let q = [d[0] - h * a[0], d[1] - h * a[1], d[2] - h * a[2]];
            let ql = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2]).sqrt();
            if ql < 1e-12 {
                return Some(to.minor_radius.abs());
            }
            let ring_dist = ((ql - to.major_radius).powi(2) + h * h).sqrt();
            Some((ring_dist - to.minor_radius).abs())
        }
        _ => None,
    }
}

/// Unit surface normal of an analytic surface at (or near) a point.
/// Returns None for non-analytic surfaces and at singular loci
/// (cylinder/cone axis, cone apex, torus ring circle) where the normal
/// is undefined. Sign convention is irrelevant for the exemption (the
/// criterion only compares |n0·n1| against the mesh dihedral).
pub fn surface_unit_normal(surf: &Surface, p: &Point3d) -> Option<[f64; 3]> {
    let unit = |v: [f64; 3]| -> Option<[f64; 3]> {
        let l = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
        if l < 1e-15 {
            None
        } else {
            Some([v[0] / l, v[1] / l, v[2] / l])
        }
    };
    match surf {
        Surface::Plane(pl) => unit([pl.normal.x, pl.normal.y, pl.normal.z]),
        Surface::Cylinder(cy) => {
            let d = [p.x - cy.origin.x, p.y - cy.origin.y, p.z - cy.origin.z];
            let a = [cy.axis.x, cy.axis.y, cy.axis.z];
            let t = d[0] * a[0] + d[1] * a[1] + d[2] * a[2];
            unit([d[0] - t * a[0], d[1] - t * a[1], d[2] - t * a[2]])
        }
        Surface::Cone(co) => {
            let d = [p.x - co.origin.x, p.y - co.origin.y, p.z - co.origin.z];
            let a = [co.axis.x, co.axis.y, co.axis.z];
            let t = d[0] * a[0] + d[1] * a[1] + d[2] * a[2];
            let perp = [d[0] - t * a[0], d[1] - t * a[1], d[2] - t * a[2]];
            let Some(pr) = unit(perp) else { return None }; // on the axis
            let tan = co.half_angle.tan();
            let r_t = if co.expanding {
                co.radius + t * tan
            } else {
                co.radius - t * tan
            };
            if r_t <= 1e-9 {
                return None; // apex singularity
            }
            let ch = co.half_angle.cos();
            let sh = co.half_angle.sin();
            // outward normal: cos(h)·radial ∓ sin(h)·axis (expanding → −)
            let s = if co.expanding { -1.0 } else { 1.0 };
            unit([
                ch * pr[0] + s * sh * a[0],
                ch * pr[1] + s * sh * a[1],
                ch * pr[2] + s * sh * a[2],
            ])
        }
        Surface::Torus(to) => {
            let d = [p.x - to.center.x, p.y - to.center.y, p.z - to.center.z];
            let a = [to.axis.x, to.axis.y, to.axis.z];
            let h = d[0] * a[0] + d[1] * a[1] + d[2] * a[2];
            let q = [d[0] - h * a[0], d[1] - h * a[1], d[2] - h * a[2]];
            let ql = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2]).sqrt();
            if ql < 1e-12 {
                return None; // on the torus axis
            }
            // ring-circle point at p's angular position; p − ring ≈
            // ±minor_radius·n at the nearest surface point
            let ring = [
                to.center.x + to.major_radius * q[0] / ql,
                to.center.y + to.major_radius * q[1] / ql,
                to.center.z + to.major_radius * q[2] / ql,
            ];
            unit([p.x - ring[0], p.y - ring[1], p.z - ring[2]])
        }
        _ => None,
    }
}

/// Are the two analytic surfaces geometrically IDENTICAL (same plane /
/// cylinder / cone / torus)? Tangency between a surface and itself is
/// trivially true and must NOT exempt pairs — those are intra-surface
/// winding/fold bugs. Returns None if either surface is non-analytic.
pub fn same_analytic_surface(s0: &Surface, s1: &Surface) -> Option<bool> {
    let ax_eq = |a: &draper_geometry::Direction3d, b: &draper_geometry::Direction3d| -> bool {
        let cross = [
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x,
        ];
        let cl = (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt();
        cl < 1e-9
    };
    let pt_eq = |a: &Point3d, b: &Point3d| -> bool {
        (a.x - b.x).abs() < 1e-9 && (a.y - b.y).abs() < 1e-9 && (a.z - b.z).abs() < 1e-9
    };
    Some(match (s0, s1) {
        (Surface::Plane(p0), Surface::Plane(p1)) => {
            ax_eq(&p0.normal, &p1.normal)
                && ((p0.normal.x * (p1.origin.x - p0.origin.x)
                    + p0.normal.y * (p1.origin.y - p0.origin.y)
                    + p0.normal.z * (p1.origin.z - p0.origin.z))
                    .abs()
                    < 1e-9)
        }
        (Surface::Cylinder(c0), Surface::Cylinder(c1)) => {
            let d = [
                c1.origin.x - c0.origin.x,
                c1.origin.y - c0.origin.y,
                c1.origin.z - c0.origin.z,
            ];
            let a = [c0.axis.x, c0.axis.y, c0.axis.z];
            let t = d[0] * a[0] + d[1] * a[1] + d[2] * a[2];
            let q = [d[0] - t * a[0], d[1] - t * a[1], d[2] - t * a[2]];
            let perp_l = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2]).sqrt();
            ax_eq(&c0.axis, &c1.axis)
                && t.abs() < 1e-9
                && perp_l < 1e-9
                && (c0.radius - c1.radius).abs() < 1e-9
        }
        (Surface::Cone(c0), Surface::Cone(c1)) => {
            pt_eq(&c0.origin, &c1.origin)
                && ax_eq(&c0.axis, &c1.axis)
                && (c0.half_angle - c1.half_angle).abs() < 1e-9
                && (c0.radius - c1.radius).abs() < 1e-9
                && c0.expanding == c1.expanding
        }
        (Surface::Torus(t0), Surface::Torus(t1)) => {
            pt_eq(&t0.center, &t1.center)
                && ax_eq(&t0.axis, &t1.axis)
                && (t0.major_radius - t1.major_radius).abs() < 1e-9
                && (t0.minor_radius - t1.minor_radius).abs() < 1e-9
        }
        _ => false,
    })
}

/// Chord-noise tolerance: the mesh dihedral may slightly overshoot the
/// true surface-surface angle due to chord approximation.
pub const SN_TOL_DEG: f64 = 2.0;

/// Full per-pair exemption evaluation.
pub struct PairVerdict {
    /// shared edge traversed in opposite directions by the two triangles
    pub topo_consistent: bool,
    /// both centroids within 1e-6 of the other face's surface
    pub coincide: bool,
    /// apex heights over the shared edge (sliver metric)
    pub h0: f64,
    pub h1: f64,
    /// centroid → other-surface distances
    pub d01: Option<f64>,
    pub d10: Option<f64>,
    /// glue tolerance actually used
    pub glue_tol: f64,
    /// surface-surface normal angle at the shared-edge midpoint
    /// (diagnostics only — chord-sagitta contaminated)
    pub sn_mid: Option<f64>,
    /// max surface-surface normal angle over the shared-edge endpoints
    /// (bit-exact rim points — the true junction angle)
    pub sn_rim: Option<f64>,
    /// exemption decision
    pub exempt: bool,
    /// human-readable reason (first failing guard, or "ok")
    pub reason: String,
}

/// Evaluate the surface-justified exemption for a pair of triangles
/// sharing exactly one edge. `mesh_ang_deg` is the raw dihedral angle
/// between the two triangle normals (0..180). `fid0`/`fid1` are the
/// owning face ids (same face ⇒ never exempt). Surfaces must already be
/// in world space (apply `transform_surface` first).
pub fn evaluate(
    verts: &[Point3d],
    tri0: &[u32; 3],
    tri1: &[u32; 3],
    mesh_ang_deg: f64,
    fid0: u64,
    fid1: u64,
    surf0: Option<&Surface>,
    surf1: Option<&Surface>,
) -> PairVerdict {
    // --- shared edge extraction + topological winding consistency ---
    let dirs = |t: &[u32; 3]| -> [(u32, u32); 3] {
        [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])]
    };
    let d0 = dirs(tri0);
    let d1 = dirs(tri1);
    let mut shared: Option<((u32, u32), (u32, u32))> = None;
    for e0 in &d0 {
        for e1 in &d1 {
            if e0.0 == e1.0 && e0.1 == e1.1 {
                // same-direction duplicate edge — degenerate overlap
                shared = Some((*e0, *e1));
            } else if e0.0 == e1.1 && e0.1 == e1.0 && shared.is_none() {
                shared = Some((*e0, *e1));
            }
        }
    }
    let Some(((ea0, eb0), (ea1, eb1))) = shared else {
        return PairVerdict {
            topo_consistent: false,
            coincide: false,
            h0: 0.0,
            h1: 0.0,
            d01: None,
            d10: None,
            glue_tol: 0.0,
            sn_mid: None,
            sn_rim: None,
            exempt: false,
            reason: "no-shared-edge".to_string(),
        };
    };
    // opposite traversal = consistent manifold winding
    let topo_consistent = ea0 == eb1 && eb0 == ea1;

    let a = verts[ea0 as usize];
    let b = verts[eb0 as usize];
    let apex_of = |t: &[u32; 3]| -> Point3d {
        for &v in t {
            if v != ea0 && v != eb0 {
                return verts[v as usize];
            }
        }
        verts[t[0] as usize]
    };
    let c0 = apex_of(tri0);
    let c1 = apex_of(tri1);

    // --- sliver heights over the shared edge ---
    let e = [b.x - a.x, b.y - a.y, b.z - a.z];
    let base_len =
        (e[0] * e[0] + e[1] * e[1] + e[2] * e[2]).sqrt().max(1e-30);
    let point_line_dist = |p: &Point3d| -> f64 {
        let ap = [p.x - a.x, p.y - a.y, p.z - a.z];
        let cr = [
            ap[1] * e[2] - ap[2] * e[1],
            ap[2] * e[0] - ap[0] * e[2],
            ap[0] * e[1] - ap[1] * e[0],
        ];
        let l = (cr[0] * cr[0] + cr[1] * cr[1] + cr[2] * cr[2]).sqrt();
        l / base_len
    };
    let h0 = point_line_dist(&c0);
    let h1 = point_line_dist(&c1);

    // --- centroid cross-distances (coincidence + glue) ---
    let centroid = |t: &[u32; 3]| -> Point3d {
        let (va, vb, vc) = (
            verts[t[0] as usize],
            verts[t[1] as usize],
            verts[t[2] as usize],
        );
        Point3d::new(
            (va.x + vb.x + vc.x) / 3.0,
            (va.y + vb.y + vc.y) / 3.0,
            (va.z + vb.z + vc.z) / 3.0,
        )
    };
    let g0 = centroid(tri0);
    let g1 = centroid(tri1);
    let d01 = surf1.and_then(|s| surface_distance(s, &g0));
    let d10 = surf0.and_then(|s| surface_distance(s, &g1));
    let coincide = match (d01, d10) {
        (Some(x), Some(y)) => x < 1e-6 && y < 1e-6,
        _ => false,
    };

    // --- surface-surface normal angles at rim endpoints + midpoint ---
    let sn_ang_at = |p: &Point3d| -> Option<f64> {
        let n0 = surf0.and_then(|s| surface_unit_normal(s, p))?;
        let n1 = surf1.and_then(|s| surface_unit_normal(s, p))?;
        Some(
            (n0[0] * n1[0] + n0[1] * n1[1] + n0[2] * n1[2])
                .clamp(-1.0, 1.0)
                .acos()
                .to_degrees(),
        )
    };
    let mid = Point3d::new(
        (a.x + b.x) / 2.0,
        (a.y + b.y) / 2.0,
        (a.z + b.z) / 2.0,
    );
    let sn_a = sn_ang_at(&a);
    let sn_b = sn_ang_at(&b);
    let sn_mid = sn_ang_at(&mid);
    let sn_rim = match (sn_a, sn_b) {
        (Some(x), Some(y)) => Some(x.max(y)),
        (Some(x), None) | (None, Some(x)) => Some(x),
        (None, None) => None,
    };

    // --- guards ---
    let glue_tol = 1e-3 + 0.1 * h0.max(h1);
    let glued = match (d01, d10) {
        (Some(x), Some(y)) => x <= glue_tol && y <= glue_tol,
        _ => false,
    };
    let surfaces_distinct = match (surf0, surf1) {
        (Some(s0), Some(s1)) => !matches!(same_analytic_surface(s0, s1), Some(true)),
        _ => false,
    };
    let sn_justifies = match sn_rim {
        Some(s) => mesh_ang_deg <= s + SN_TOL_DEG,
        None => false,
    };

    let exempt = topo_consistent
        && !coincide
        && fid0 != fid1
        && surfaces_distinct
        && sn_justifies
        && glued;

    let reason = if exempt {
        "ok".to_string()
    } else if !topo_consistent {
        "winding".to_string()
    } else if fid0 == fid1 {
        "same-face".to_string()
    } else if coincide {
        "coincident".to_string()
    } else if !surfaces_distinct {
        "same-surface".to_string()
    } else if !sn_justifies {
        match sn_rim {
            Some(s) => format!("mesh-{}>surf-{:.1}", mesh_ang_deg, s),
            None => "no-surf-normal".to_string(),
        }
    } else if !glued {
        "not-glued".to_string()
    } else {
        "n/a".to_string()
    };

    PairVerdict {
        topo_consistent,
        coincide,
        h0,
        h1,
        d01,
        d10,
        glue_tol,
        sn_mid,
        sn_rim,
        exempt,
        reason,
    }
}

// ─────────────────────────────────────────────────────────────────
// session-94: KNIFE-FIN (thin-wall pinch) test.
//
// A FOLD-OVER-class pair (topo-consistent manifold winding + BOTH
// apexes on the SAME side of the shared edge) whose two triangles are
// near-coincident OFFSET COPIES — apexes a wall-thickness apart along
// the surface normal, at the same hinge distance — is the two sides
// of a thin solid wall pinched at the shared edge (the flattened-tube
// pinch / knife edge). A faithful mesh of that geometry NECESSARILY
// shows ~180° between the raw triangle normals: not a bug.
// Measured origin: transmission SPRING f2/f3 (two Nurbs 9x69 halves
// of a coiled 0.75mm × 0.043mm flattened-tube ribbon sharing both
// pinch curves) — the sail-lattice corner needles at the end-cap
// corners pair up across the pinch with apex gaps exactly the wall
// thickness (0.0427mm vs eff_tol 0.0043mm) and equal hinge distances.
//
// Guards (all must hold):
//   1. topo-consistent: the shared edge is traversed in opposite
//      directions (a manifold pair — a duplicated emission traverses
//      it the same way and stays WINDING-FLIP);
//   2. same-side apexes (the FOLD-OVER class gate);
//   3. offset copies: |c0−c1| ≤ 0.1 · min(hinge distance of each
//      apex to the shared-edge SEGMENT) — the apexes sit at the same
//      reach from the pinch, a wall-thickness apart, not at unrelated
//      spots of a hovering flap;
//   4. resolvable perpendicular gaps: both apexes strictly off the
//      other triangle's plane by ≥ min_gap (the caller passes the
//      BREP eff_tol). Note this also PROVES disjointness: any point
//      of tri0's open interior has signed distance γ·g0 (γ>0) to
//      tri1's plane, so with g0>0 no interior point can lie IN tri1 —
//      the only possible interior contact is the coplanar g=0 case,
//      excluded here;
//   5. normal-dominant offset: |c0−c1| ≤ 3·max(g0,g1) — the copies
//      are STACKED across the wall, not shifted along the surface.
//
// Callers gate on: cross-face (fid0 != fid1), >170°, and the
// DRAPPER_KNIFE_FIN=0 kill-switch. Same-face fins (a face folding
// onto itself) stay flagged.
pub fn knife_fin(
    verts: &[Point3d],
    tri0: &[u32; 3],
    tri1: &[u32; 3],
    min_gap: f64,
) -> (bool, String) {
    // --- shared edge + apexes (same extraction as evaluate) ---
    let dirs = |t: &[u32; 3]| -> [(u32, u32); 3] {
        [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])]
    };
    let d0 = dirs(tri0);
    let d1 = dirs(tri1);
    let mut shared: Option<((u32, u32), (u32, u32))> = None;
    for e0 in &d0 {
        for e1 in &d1 {
            if e0.0 == e1.0 && e0.1 == e1.1 {
                shared = Some((*e0, *e1));
            } else if e0.0 == e1.1 && e0.1 == e1.0 && shared.is_none() {
                shared = Some((*e0, *e1));
            }
        }
    }
    let Some(((ea0, eb0), (ea1, eb1))) = shared else {
        return (false, "no-shared-edge".to_string());
    };
    // guard 1: opposite traversal (manifold winding)
    if !(ea0 == eb1 && eb0 == ea1) {
        return (false, "winding".to_string());
    }
    let a = verts[ea0 as usize];
    let b = verts[eb0 as usize];
    let apex_of = |t: &[u32; 3]| -> Point3d {
        for &v in t {
            if v != ea0 && v != eb0 {
                return verts[v as usize];
            }
        }
        verts[t[0] as usize]
    };
    let c0 = apex_of(tri0);
    let c1 = apex_of(tri1);

    // guard 2: same-side apexes (the FOLD-OVER class shape)
    let e = [b.x - a.x, b.y - a.y, b.z - a.z];
    let base_len = (e[0] * e[0] + e[1] * e[1] + e[2] * e[2]).sqrt();
    if base_len <= 1e-30 {
        return (false, "degenerate-edge".to_string());
    }
    let side = |q: &Point3d| -> [f64; 3] {
        let d = [q.x - a.x, q.y - a.y, q.z - a.z];
        [
            e[1] * d[2] - e[2] * d[1],
            e[2] * d[0] - e[0] * d[2],
            e[0] * d[1] - e[1] * d[0],
        ]
    };
    let s0 = side(&c0);
    let s1 = side(&c1);
    if s0[0] * s1[0] + s0[1] * s1[1] + s0[2] * s1[2] <= 0.0 {
        return (false, "opposite-side".to_string());
    }

    // guard 3: offset copies — apex-apex distance vs hinge reach
    let seg_dist = |q: &Point3d| -> f64 {
        let ap = [q.x - a.x, q.y - a.y, q.z - a.z];
        let t =
            ((ap[0] * e[0] + ap[1] * e[1] + ap[2] * e[2]) / (base_len * base_len)).clamp(0.0, 1.0);
        let cx = a.x + t * e[0] - q.x;
        let cy = a.y + t * e[1] - q.y;
        let cz = a.z + t * e[2] - q.z;
        (cx * cx + cy * cy + cz * cz).sqrt()
    };
    let d_h0 = seg_dist(&c0);
    let d_h1 = seg_dist(&c1);
    let d_min = d_h0.min(d_h1).max(1e-30);
    let c01 = ((c0.x - c1.x).powi(2) + (c0.y - c1.y).powi(2) + (c0.z - c1.z).powi(2)).sqrt();
    if c01 > 0.1 * d_min {
        return (false, format!("apex-offset {:.4} > 0.1·hinge {:.4}", c01, 0.1 * d_min));
    }

    // guard 4: resolvable perpendicular gaps (also proves disjointness)
    let plane_dist = |q: &Point3d, p: &Point3d, u: &Point3d, v: &Point3d| -> f64 {
        let e1 = [u.x - p.x, u.y - p.y, u.z - p.z];
        let e2 = [v.x - p.x, v.y - p.y, v.z - p.z];
        let n = [
            e1[1] * e2[2] - e1[2] * e2[1],
            e1[2] * e2[0] - e1[0] * e2[2],
            e1[0] * e2[1] - e1[1] * e2[0],
        ];
        let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        if l <= 1e-30 {
            return f64::INFINITY;
        }
        let dq = [q.x - p.x, q.y - p.y, q.z - p.z];
        (n[0] * dq[0] + n[1] * dq[1] + n[2] * dq[2]) / l
    };
    let (t1a, t1b, t1c) = (
        verts[tri1[0] as usize],
        verts[tri1[1] as usize],
        verts[tri1[2] as usize],
    );
    let (t0a, t0b, t0c) = (
        verts[tri0[0] as usize],
        verts[tri0[1] as usize],
        verts[tri0[2] as usize],
    );
    let g0 = plane_dist(&c0, &t1a, &t1b, &t1c).abs();
    let g1 = plane_dist(&c1, &t0a, &t0b, &t0c).abs();
    let gap_floor = min_gap.max(1e-9);
    if g0 < gap_floor || g1 < gap_floor {
        return (false, format!("gap {:.6}/{:.6} < floor {:.6}", g0, g1, gap_floor));
    }

    // guard 5: normal-dominant offset (stacked, not shifted along)
    if c01 > 3.0 * g0.max(g1) {
        return (false, format!("apex-offset {:.4} not wall-normal (g {:.4}/{:.4})", c01, g0, g1));
    }

    (
        true,
        format!(
            "knife-fin: gap {:.4}/{:.4} hinge {:.3}/{:.3} apex-d {:.4}",
            g0, g1, d_h0, d_h1, c01
        ),
    )
}

// ─────────────────────────────────────────────────────────────────
// session-121: COIL-TURN-WELD exemption (the census layer of the s120
// verdict on the residual zipped-face REAL pairs).
//
// A same-face >170° pair on a coiled ribbon (helical band, Nurbs with
// a degree-1 direction) whose fold is rooted in coil geometry that
// the merge tolerance cannot represent:
//   - adjacent turns of the tangle sit closer than the tolerance and
//     the merge welds them into one seam (s120 measured: f11 x f13
//     gap 0.0079 < merge_tol 0.0156, point k=253 of f13 welded onto
//     f11's rail vertex), so fix_inconsistent_winding's greedy flood
//     pits two individually-correct emissions against each other and
//     manufactures thin same-face pairs (BFS collateral); or
//   - the band / turn gap of the ribbon is itself narrower than the
//     tolerance (s18: f28 ruling width 8.9e-3 < 2e-2 "merge-doomed by
//     geometry"), so the turns physically self-overlap (intrinsic
//     FOLD-OVER, identical in default and zipper modes).
// A faithful mesh necessarily folds there. Not debt.
//
// Guards (all must hold):
//   1. thin: max(h0,h1) < 2×eff_tol — a near-miss REAL by the eff_tol
//      ruler (at least one apex >= eff_tol; both under it is sub-tol
//      weld noise, which callers check BEFORE this class);
//   2. ruled ribbon: the owning face's surface is a Nurbs with a
//      degree-1 direction (deg 3/1: iso-curves of the other parameter
//      are straight rulings). Analytic faces (the Cylinder/Torus/
//      Plane chronics) and Nurbs deg 2/3 (#27977 f30 SLIVER 1.2x,
//      #31356 f3) stay flagged;
//   3. weld witness (either):
//      (a) STAR — a cross-face interior edge at one of the pair's
//          four triad vertices: the welded rail is in the merged mesh
//          at the pair (f11: e(594,595) f=(11,13); f26: e(1358,1404)
//          f=(24,26)); or
//      (b) RIM-GAP — the owning face's boundary polyline returns
//          within eff_tol of itself at a FAR position of the loop
//          (index distance >= n/4; wrap-around neighbours and the
//          closing duplicate excluded): the other rail of the same
//          turn or an adjacent turn sits closer than the tolerance
//          (f25 far=0.0135 < 0.0179; f28 far=0.0111 < 0.0200).
//          Ribbon faces whose rim stays apart (Vulcan #10220 f25
//          far=0.039, f27 far=0.051 vs tol 0.02) stay flagged — their
//          folds are emission debt (the s118 projection zigzag),
//          fixable, not exemptible.
//
// Callers gate on: same-face (fid0 == fid1), >170°, non-subtol,
// non-knife, and the DRAPPER_COIL_TURN_EXEMPT=0 kill-switch.

/// Weld-witness facts for `coil_turn_weld`, computed by the caller
/// from the merged mesh (star) and the face's boundary polylines
/// (rim gap). Both are diagnostics-grade inputs; the guard only
/// needs `star` OR `rim_far < eff_tol`.
pub struct CoilWitness {
    /// (a) a cross-face interior edge touches one of the pair's four
    /// triad vertices (the welded seam in the merged mesh)
    pub star: bool,
    /// which edge (v0, v1)
    pub star_edge: (u32, u32),
    /// which two faces share it (fid_a, fid_b), fid_a != fid_b
    pub star_faces: (u64, u64),
    /// (b) the owning face's rim polyline far-loop min distance
    /// (world space; None if the face has no usable polyline)
    pub rim_far: Option<f64>,
}

/// Evaluate the coil-turn-weld exemption. `h0`/`h1` are the apex
/// heights of the two triangles over the shared edge (line distances,
/// same metric as the sub-tol classification). `eff_tol` is the BREP
/// effective tolerance (merge + every weld pass). `surface` is the
/// OWNING face's surface (same-face pair: one surface), already in
/// world space.
pub fn coil_turn_weld(
    h0: f64,
    h1: f64,
    eff_tol: f64,
    surface: Option<&Surface>,
    w: Option<&CoilWitness>,
) -> (bool, String) {
    // guard 1: thin near-miss by the ruler
    if h0.max(h1) >= 2.0 * eff_tol {
        return (
            false,
            format!("not-thin h={:.4}/{:.4} vs 2tol={:.4}", h0, h1, 2.0 * eff_tol),
        );
    }
    // guard 2: ruled ribbon (Nurbs with a degree-1 direction)
    let ruled = match surface {
        Some(Surface::Nurbs(n)) => n.u_degree == 1 || n.v_degree == 1,
        _ => false,
    };
    if !ruled {
        return (false, "not-ruled".to_string());
    }
    // guard 3: weld witness — star OR rim-gap
    let Some(w) = w else {
        return (false, "no-witness".to_string());
    };
    if w.star {
        return (
            true,
            format!(
                " star e({},{}) f=({},{})",
                w.star_edge.0, w.star_edge.1, w.star_faces.0, w.star_faces.1
            ),
        );
    }
    match w.rim_far {
        Some(gap) if gap < eff_tol => {
            (true, format!(" rim-gap {:.5} < tol {:.4}", gap, eff_tol))
        }
        Some(gap) => (
            false,
            format!("rim-gap {:.5} >= tol {:.4}", gap, eff_tol),
        ),
        None => (false, "no-witness".to_string()),
    }
}

// ── session-123: junction-sliver (cross-face multi-face-hub
// duplicate-coverage) exemption ─────────────────────────────────────
//
// s122-п.3 decoded the last #31359 REAL pair (f(7,11), Cone×Nurbs,
// ×2 instances): BOTH triangles lie in ONE plane — the runout plane
// where every coil winding meets the cone (a MULTI-FACE junction: the
// pair's apex is a hub vertex of 4 faces; 20 junction-plane points
// shared by 5 faces) — and the edge geometry is WELD-ASSEMBLED (the
// ribbon's own runout points sit 0.003–0.016 off the junction curve,
// within eff_tol 0.0156, and weld onto it). A thin coplanar duplicate
// coverage whose knife-edge REAL exists only through the hub apex
// (~1× tol): not representable at the merge tolerance, not debt. The
// s123 recon measured the same class on Vulcan #10203 f(108,1118)
// (Torus×Plane, plane_dev 3.8e-15, hub 3) and the negative controls:
// #10220 f(8,27) is thick (h 0.2627 ≫ 2×tol) AND not coplanar
// (3.0e-3) AND hub 2 — stays REAL.
//
// Callers gate on: cross-face (fid0 != fid1), >170°, non-subtol,
// non-knife, non-coil, and the DRAPPER_JUNCTION_SLIDER=0 kill-switch.

/// Witness facts for `junction_sliver`, computed by the caller from
/// the merged mesh.
pub struct JunctionWitness {
    /// max over the pair's four vertices of the number of DISTINCT
    /// faces whose triangles use that vertex (3+ = a multi-face hub —
    /// the runout/junction structure; a plain two-face adjacency
    /// scores 2)
    pub hub_max: usize,
    /// which vertex reaches `hub_max`
    pub hub_vertex: u32,
    /// two-sided plane deviations `[d10, d01]`: max distance of t1's
    /// vertices from t0's plane and vice versa (None if either
    /// triangle is degenerate)
    pub plane_dev: Option<[f64; 2]>,
}

/// Evaluate the junction-sliver exemption. `h0`/`h1` are the apex
/// heights of the two triangles over the shared edge (same metric as
/// the sub-tol classification); `eff_tol` is the BREP effective
/// tolerance (merge + every weld pass).
pub fn junction_sliver(
    h0: f64,
    h1: f64,
    eff_tol: f64,
    w: Option<&JunctionWitness>,
) -> (bool, String) {
    // guard 1: thin by the ruler (both apexes under 2×eff_tol — the
    // coil-turn metric; measured negative control: the #10220 f(8,27)
    // fold at h=0.2627 vs 2×tol=0.0400 stays REAL)
    if h0.max(h1) >= 2.0 * eff_tol {
        return (
            false,
            format!("not-thin h={:.4}/{:.4} vs 2tol={:.4}", h0, h1, 2.0 * eff_tol),
        );
    }
    let Some(w) = w else {
        return (false, "no-witness".to_string());
    };
    // guard 2: coplanar — both triangles in ONE plane. Measured
    // witnesses sit at 5.3e-5 (#31359, tol 0.0156) and 3.8e-15
    // (#10203, tol 0.0158) while the rejected #10220 fold deviates
    // 3.0e-3 with tol 0.0200; the 10%-of-ruler band separates the
    // classes on both sides.
    let Some([d10, d01]) = w.plane_dev else {
        return (false, "no-plane".to_string());
    };
    if d10.max(d01) >= 0.1 * eff_tol {
        return (
            false,
            format!(
                "not-coplanar dev=({:.5},{:.5}) vs 0.1tol={:.5}",
                d10,
                d01,
                0.1 * eff_tol
            ),
        );
    }
    // guard 3: multi-face hub — one of the pair's vertices is used by
    // 3+ distinct faces (the junction/runout structure)
    if w.hub_max < 3 {
        return (false, format!("no-hub max={}", w.hub_max));
    }
    (
        true,
        format!(
            " hub v{}×{}faces dev=({:.2e},{:.2e})",
            w.hub_vertex, w.hub_max, d10, d01
        ),
    )
}

#[cfg(test)]
mod junction_sliver_tests {
    use super::*;

    fn witness(hub: usize, d10: f64, d01: f64) -> JunctionWitness {
        JunctionWitness {
            hub_max: hub,
            hub_vertex: 314,
            plane_dev: Some([d10, d01]),
        }
    }

    /// The #31359 f(7,11) pair (measured): thin (0.0163 < 2×0.0156),
    /// coplanar (5.3e-5 / 3.6e-6), hub 4 — exempt.
    #[test]
    fn accepts_31359_pair() {
        let w = witness(4, 5.33e-5, 3.63e-6);
        let (ok, why) = junction_sliver(0.0011, 0.0163, 0.0156, Some(&w));
        assert!(ok, "why={why}");
        assert!(why.contains("hub v314×4faces"));
    }

    /// The #10203 f(108,1118) pair (measured): thin (0.0212 <
    /// 2×0.0158), exactly coplanar (3.8e-15 / 4.0e-16), hub 3 — exempt.
    #[test]
    fn accepts_10203_pair() {
        let w = witness(3, 3.81e-15, 4.04e-16);
        let (ok, why) = junction_sliver(0.0022, 0.0212, 0.0158, Some(&w));
        assert!(ok, "why={why}");
    }

    /// The #10220 f(8,27) negative control (measured): h=0.2627 ≫
    /// 2×tol=0.0400 — stays REAL (not-thin fires before the plane /
    /// hub guards even run).
    #[test]
    fn rejects_thick_fold() {
        let w = witness(2, 1.9e-5, 3.05e-3);
        let (ok, why) = junction_sliver(0.2627, 0.0017, 0.0200, Some(&w));
        assert!(!ok);
        assert!(why.starts_with("not-thin"));
    }

    /// A thin coplanar pair WITHOUT a hub (plain two-face adjacency,
    /// hub 2) — the coil-turn class already owns same-face pairs, and
    /// a cross-face pair without the junction structure stays REAL.
    #[test]
    fn rejects_no_hub() {
        let w = witness(2, 1e-6, 1e-6);
        let (ok, why) = junction_sliver(0.001, 0.002, 0.0156, Some(&w));
        assert!(!ok);
        assert!(why.starts_with("no-hub"));
    }

    /// A thin hub pair that is NOT coplanar (a genuine fold-back —
    /// the #10220 plane deviation 3.0e-3 vs 0.1×tol band) — stays
    /// REAL.
    #[test]
    fn rejects_not_coplanar() {
        let w = witness(4, 1.9e-5, 3.05e-3);
        let (ok, why) = junction_sliver(0.0011, 0.0163, 0.0200, Some(&w));
        assert!(!ok);
        assert!(why.starts_with("not-coplanar"));
    }

    /// Degenerate triangle → no plane → not exempt.
    #[test]
    fn rejects_no_plane() {
        let w = JunctionWitness {
            hub_max: 4,
            hub_vertex: 314,
            plane_dev: None,
        };
        let (ok, why) = junction_sliver(0.0011, 0.0163, 0.0156, Some(&w));
        assert!(!ok);
        assert_eq!(why, "no-plane");
    }

    /// Missing witness (e.g. mesh without face ids) — not exempt.
    #[test]
    fn rejects_no_witness() {
        let (ok, why) = junction_sliver(0.0011, 0.0163, 0.0156, None);
        assert!(!ok);
        assert_eq!(why, "no-witness");
    }
}

#[cfg(test)]
mod coil_turn_weld_tests {
    use super::*;
    use draper_geometry::NurbsSurface;

    fn ruled_nurbs() -> Surface {
        // A deg 3/1 ribbon: u cubic (4 cps), v linear (2 cps) —
        // the helical-band class (ruled in v).
        let cps = vec![
            vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.02, 0.0, 0.0)],
            vec![Point3d::new(1.0, 0.5, 0.0), Point3d::new(1.02, 0.5, 0.0)],
            vec![Point3d::new(2.0, 0.0, 0.0), Point3d::new(2.02, 0.0, 0.0)],
            vec![Point3d::new(3.0, 0.5, 0.0), Point3d::new(3.02, 0.5, 0.0)],
        ];
        Surface::Nurbs(NurbsSurface {
            u_degree: 3,
            v_degree: 1,
            control_points: cps,
            weights: vec![vec![1.0, 1.0]; 4],
            u_knots: vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            v_knots: vec![0.0, 0.0, 1.0, 1.0],
            u_closed: false,
            v_closed: false,
        })
    }

    fn witness(star: bool, rim: Option<f64>) -> CoilWitness {
        CoilWitness {
            star,
            star_edge: (594, 595),
            star_faces: (11, 13),
            rim_far: rim,
        }
    }

    /// The f11 pair: thin (0.021 < 2x0.0156), ruled, star witness.
    #[test]
    fn accepts_star_witness() {
        let s = ruled_nurbs();
        let (ok, why) = coil_turn_weld(
            0.0207,
            0.0209,
            0.0156,
            Some(&s),
            Some(&witness(true, Some(0.021))),
        );
        assert!(ok, "should accept: {}", why);
        assert!(why.contains("star"), "reason: {}", why);
    }

    /// The f25 pair: thin, ruled, no star but the rim returns at
    /// 0.0135 < tol 0.0179 (the #31360 remainder).
    #[test]
    fn accepts_rim_gap_witness() {
        let s = ruled_nurbs();
        let (ok, why) =
            coil_turn_weld(0.0265, 0.0000, 0.0179, Some(&s), Some(&witness(false, Some(0.0135))));
        assert!(ok, "should accept: {}", why);
        assert!(why.contains("rim-gap"), "reason: {}", why);
    }

    /// Vulcan #10220 f25/f27: thin, ruled, but the rim stays apart
    /// (0.039/0.051 vs tol 0.02) — emission debt stays flagged.
    #[test]
    fn rejects_rim_apart() {
        let s = ruled_nurbs();
        let (ok, why) =
            coil_turn_weld(0.0230, 0.0010, 0.0200, Some(&s), Some(&witness(false, Some(0.039))));
        assert!(!ok, "must reject: {}", why);
        assert!(why.contains("rim-gap"), "reason: {}", why);
    }

    /// A FAT pair (default-mode #31359 f13 h=0.06/0.27) — the fold is
    /// well above the ruler, real debt.
    #[test]
    fn rejects_fat() {
        let s = ruled_nurbs();
        let (ok, why) = coil_turn_weld(0.0603, 0.0560, 0.0156, Some(&s), Some(&witness(true, None)));
        assert!(!ok, "must reject: {}", why);
        assert!(why.contains("not-thin"), "reason: {}", why);
    }

    /// Analytic faces (Cylinder/Torus chronics) and Nurbs deg 2/3
    /// (#27977 f30, #31356 f3) stay flagged even when thin with a
    /// witness.
    #[test]
    fn rejects_not_ruled() {
        let cyl = Surface::Cylinder(draper_geometry::CylinderSurface {
            origin: Point3d::new(0.0, 0.0, 0.0),
            axis: draper_geometry::Direction3d::new(0.0, 0.0, 1.0).unwrap(),
            radius: 1.0,
            x_dir: draper_geometry::Direction3d::new(1.0, 0.0, 0.0).unwrap(),
        });
        let (ok, why) = coil_turn_weld(0.0279, 0.0118, 0.0161, Some(&cyl), Some(&witness(true, Some(0.0107))));
        assert!(!ok, "must reject the Cylinder: {}", why);
        assert!(why.contains("not-ruled"), "reason: {}", why);

        // Nurbs deg 2/3 (f30-class): thin + rim witness, still flagged
        let mut n = ruled_nurbs();
        if let Surface::Nurbs(ref mut nn) = n {
            nn.u_degree = 2;
            nn.v_degree = 3;
        }
        let (ok, why) = coil_turn_weld(0.0239, 0.0027, 0.0167, Some(&n), Some(&witness(false, Some(0.0117))));
        assert!(!ok, "must reject the deg-2/3 Nurbs: {}", why);
        assert!(why.contains("not-ruled"), "reason: {}", why);
    }

    /// No witness at all → flagged.
    #[test]
    fn rejects_no_witness() {
        let s = ruled_nurbs();
        let (ok, why) = coil_turn_weld(0.0207, 0.0209, 0.0156, Some(&s), Some(&witness(false, None)));
        assert!(!ok, "must reject: {}", why);
        assert!(why.contains("no-witness"), "reason: {}", why);
    }
}

#[cfg(test)]
mod knife_fin_tests {
    use super::*;

    fn v(x: f64, y: f64, z: f64) -> Point3d {
        Point3d::new(x, y, z)
    }

    /// The SPRING shape: shared edge along +X, both apexes on the same
    /// side (+Y reach), stacked across a small wall gap in Z — the two
    /// sides of a pinched thin wall. Accepted at any resolvable gap.
    #[test]
    fn accepts_offset_copy_pair() {
        let verts = vec![
            v(0.0, 0.0, 0.0),  // 0: edge start
            v(1.0, 0.0, 0.0),  // 1: edge end
            v(0.5, 2.5, 0.0),  // 2: tri0 apex (lower copy)
            v(0.5, 2.5, 0.04), // 3: tri1 apex (wall gap 0.04)
        ];
        // topo-consistent: the shared edge traversed in OPPOSITE directions
        let tri0 = [0u32, 1, 2];
        let tri1 = [1u32, 0, 3];
        let (ok, why) = knife_fin(&verts, &tri0, &tri1, 0.001);
        assert!(ok, "should accept the knife-fin: {}", why);
    }

    /// A duplicated emission (same-direction traversal) stays WINDING-FLIP.
    #[test]
    fn rejects_same_direction_winding() {
        let verts = vec![
            v(0.0, 0.0, 0.0),
            v(1.0, 0.0, 0.0),
            v(0.5, 2.5, 0.0),
            v(0.5, 2.5, 0.04),
        ];
        let tri0 = [0u32, 1, 2];
        let tri1 = [0u32, 1, 3]; // edge 0→1 SAME direction
        let (ok, why) = knife_fin(&verts, &tri0, &tri1, 0.001);
        assert!(!ok, "must reject: {}", why);
        assert!(why.contains("winding"), "reason: {}", why);
    }

    /// A smooth continuation (apexes on opposite sides of the hinge) is
    /// the CURVED-180 class, not a fin.
    #[test]
    fn rejects_opposite_side_apexes() {
        let verts = vec![
            v(0.0, 0.0, 0.0),
            v(1.0, 0.0, 0.0),
            v(0.5, 2.5, 0.0),
            v(0.5, -2.5, 0.04),
        ];
        let tri0 = [0u32, 1, 2];
        let tri1 = [1u32, 0, 3];
        let (ok, why) = knife_fin(&verts, &tri0, &tri1, 0.001);
        assert!(!ok, "must reject: {}", why);
        assert!(why.contains("opposite-side"), "reason: {}", why);
    }

    /// A hovering flap: the other face's apex at a DIFFERENT hinge
    /// reach — not an offset copy of this triangle.
    #[test]
    fn rejects_hovering_flap() {
        let verts = vec![
            v(0.0, 0.0, 0.0),
            v(1.0, 0.0, 0.0),
            v(0.5, 2.5, 0.0),
            v(0.5, 0.25, 0.04), // apex at hinge reach 0.25 vs 2.5
        ];
        let tri0 = [0u32, 1, 2];
        let tri1 = [1u32, 0, 3];
        let (ok, why) = knife_fin(&verts, &tri0, &tri1, 0.001);
        assert!(!ok, "must reject: {}", why);
        assert!(why.contains("apex-offset"), "reason: {}", why);
    }

    /// Coincident copies (zero gap) are the double-emission signature,
    /// not a wall — coplanar g=0 also admits interior overlap.
    #[test]
    fn rejects_coincident_copies() {
        let verts = vec![
            v(0.0, 0.0, 0.0),
            v(1.0, 0.0, 0.0),
            v(0.5, 2.5, 0.0),
            v(0.5, 2.5, 0.0), // gap 0
        ];
        let tri0 = [0u32, 1, 2];
        let tri1 = [1u32, 0, 3];
        let (ok, why) = knife_fin(&verts, &tri0, &tri1, 0.001);
        assert!(!ok, "must reject: {}", why);
        assert!(why.contains("gap"), "reason: {}", why);
    }

    /// A wall thinner than the mesh's vertex resolution (eff_tol)
    /// cannot be represented — not a resolvable fin.
    #[test]
    fn rejects_subresolution_gap() {
        let verts = vec![
            v(0.0, 0.0, 0.0),
            v(1.0, 0.0, 0.0),
            v(0.5, 2.5, 0.0),
            v(0.5, 2.5, 0.000001),
        ];
        let tri0 = [0u32, 1, 2];
        let tri1 = [1u32, 0, 3];
        let (ok, why) = knife_fin(&verts, &tri0, &tri1, 0.01);
        assert!(!ok, "must reject: {}", why);
        assert!(why.contains("gap"), "reason: {}", why);
    }

    /// Same hinge reach but a LATERAL shift dominating the offset —
    /// the copies slide along the surface, they are not stacked across
    /// a wall.
    #[test]
    fn rejects_lateral_shift() {
        // |c0−c1| = 0.2 (lateral 0.196 + gap 0.04): passes the
        // 0.1·hinge gate (0.2 ≤ 0.25) but 0.2 > 3·gap = 0.12.
        let verts = vec![
            v(0.0, 0.0, 0.0),
            v(1.0, 0.0, 0.0),
            v(0.5, 2.5, 0.0),
            v(0.5, 2.696, 0.04),
        ];
        let tri0 = [0u32, 1, 2];
        let tri1 = [1u32, 0, 3];
        let (ok, why) = knife_fin(&verts, &tri0, &tri1, 0.001);
        assert!(!ok, "must reject: {}", why);
        assert!(why.contains("wall-normal"), "reason: {}", why);
    }

    /// Degenerate shared edge (zero length) — no fin decision.
    #[test]
    fn rejects_degenerate_edge() {
        let verts = vec![
            v(0.0, 0.0, 0.0),
            v(0.0, 0.0, 0.0), // zero-length edge
            v(0.5, 2.5, 0.0),
            v(0.5, 2.5, 0.04),
        ];
        let tri0 = [0u32, 1, 2];
        let tri1 = [1u32, 0, 3];
        let (ok, why) = knife_fin(&verts, &tri0, &tri1, 0.001);
        assert!(!ok, "must reject: {}", why);
        assert!(why.contains("degenerate"), "reason: {}", why);
    }
}
