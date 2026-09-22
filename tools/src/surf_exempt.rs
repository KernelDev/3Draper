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
