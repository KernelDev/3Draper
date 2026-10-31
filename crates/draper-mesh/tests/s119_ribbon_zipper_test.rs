//! s119: monotone-band zipper tests for coiled thin ruled ribbons.
//!
//! The ribbon faces accepted by `nurbs_ribbon_chain_projection` are
//! helical bands — UV PARALLELOGRAMS (two rails at (nearly) constant v,
//! closed by slanted runout sides), not axis-aligned sub-rectangles.
//! `try_ribbon_monotone_zipper` triangulates them by sweeping two
//! u-monotone chains; every crossbar connects points at the same u
//! (a ruling) so nothing cuts across the coil bundle.
//!
//! All phases run inside ONE test function: the env gates
//! (DRAPPER_RIBBON_CHAIN / DRAPPER_RIBBON_ZIPPER) are process-global,
//! so parallel #[test] fns would race on them.

use draper_geometry::{Point3d, Point2d, NurbsSurface};
use draper_mesh::triangulate::try_ribbon_monotone_zipper;

/// Flat v-ruled ribbon: S(u,v) = (u, v, 0), u ∈ [0,100], v ∈ [0,1].
/// u_span 100 >= 10, v_span 1 < 10, v_degree 1 → the criterion's
/// ruled_along_v class (with the default thread-local
/// vertex_resolution_tol of 0, the width check 1.0 > 0 passes).
fn flat_band_surface() -> NurbsSurface {
    NurbsSurface {
        u_degree: 1,
        v_degree: 1,
        control_points: vec![
            vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.0, 1.0, 0.0)],
            vec![Point3d::new(100.0, 0.0, 0.0), Point3d::new(100.0, 1.0, 0.0)],
        ],
        weights: vec![vec![1.0, 1.0], vec![1.0, 1.0]],
        u_knots: vec![0.0, 0.0, 100.0, 100.0],
        v_knots: vec![0.0, 0.0, 1.0, 1.0],
        u_closed: false,
        v_closed: false,
    }
}

/// Helical-band shaped boundary (parallelogram in UV): top rail at v=1
/// (u 10→90), bottom rail at v=0 (u 13→93), slanted runout sides with
/// a lead of 3 in u. The side interiors are CURVED (like real helical
/// runouts) — a straight-side interior point would be collinear with
/// the side endpoints, produce a degenerate sweep triangle and get the
/// face conservatively rejected (documented in the collinear phase
/// below). Loop order: A → top → B → side → C → bottom → D → side → A.
/// 3D points come from the surface itself, so the reprojection gate
/// measures exactly 0.
fn helical_band_boundary() -> (Vec<Point3d>, Vec<Point2d>) {
    let nurbs = flat_band_surface();
    let mut pts = Vec::new();
    let mut uvs = Vec::new();
    let mut push = |u: f64, v: f64| {
        let p = nurbs.point_at(u, v);
        pts.push(Point3d::new(p.x, p.y, p.z));
        uvs.push(Point2d::new(u, v));
    };
    // Top rail A(10,1) → B(90,1): 11 points.
    for k in 0..=10 {
        push(10.0 + 8.0 * k as f64, 1.0);
    }
    // Side B(90,1) → C(93,0): 1 curved interior point (off the chord).
    push(91.5, 0.35);
    // Bottom rail C(93,0) → D(13,0): 11 points.
    for k in 0..=10 {
        push(93.0 - 8.0 * k as f64, 0.0);
    }
    // Side D(13,0) → A(10,1): 1 curved interior point (off the chord).
    push(11.5, 0.62);
    (pts, uvs)
}

/// Same band but with STRAIGHT side interiors: the interior side point
/// is exactly on the chord, the sweep triangle covering its boundary
/// edges is UV-collinear (and 3D-collinear on this flat surface) and
/// gets skipped → boundary coverage fails → the zipper must REJECT the
/// face (conservative never-worsen: it falls back to the legacy path).
fn collinear_side_band_boundary() -> (Vec<Point3d>, Vec<Point2d>) {
    let nurbs = flat_band_surface();
    let mut pts = Vec::new();
    let mut uvs = Vec::new();
    let mut push = |u: f64, v: f64| {
        let p = nurbs.point_at(u, v);
        pts.push(Point3d::new(p.x, p.y, p.z));
        uvs.push(Point2d::new(u, v));
    };
    for k in 0..=10 {
        push(10.0 + 8.0 * k as f64, 1.0);
    }
    push(91.5, 0.5); // exactly on the B–C chord
    for k in 0..=10 {
        push(93.0 - 8.0 * k as f64, 0.0);
    }
    push(11.5, 0.5); // exactly on the D–A chord
    (pts, uvs)
}

/// Same band, but the bottom rail has a NOTCH that doubles back in u
/// (the Vulcan #10220 f25 failure shape): walking the bottom from
/// C(93) toward D, at u=45 the boundary detours BACK to u=70 and then
/// forward again — the lower chain is not u-monotone → the zipper must
/// reject the face (None), leaving it to the legacy path.
fn notched_band_boundary() -> (Vec<Point3d>, Vec<Point2d>) {
    let nurbs = flat_band_surface();
    let mut pts = Vec::new();
    let mut uvs = Vec::new();
    let mut push = |u: f64, v: f64| {
        let p = nurbs.point_at(u, v);
        pts.push(Point3d::new(p.x, p.y, p.z));
        uvs.push(Point2d::new(u, v));
    };
    // Top rail A(10,1) → B(90,1).
    for k in 0..=10 {
        push(10.0 + 8.0 * k as f64, 1.0);
    }
    push(91.5, 0.35); // side B → C (curved runout)
    // Bottom rail C(93,0) → 45, then notch back to 70, then to D(13).
    push(93.0, 0.0);
    push(85.0, 0.0);
    push(77.0, 0.0);
    push(69.0, 0.0);
    push(61.0, 0.0);
    push(53.0, 0.0);
    push(45.0, 0.0);
    // Notch: double back (u decreases along the walk no more — it
    // INCREASES again, breaking monotonicity of the chain).
    push(53.0, 0.0);
    push(61.0, 0.0);
    push(69.0, 0.0);
    // Continue toward D (decreasing u).
    push(37.0, 0.0);
    push(29.0, 0.0);
    push(21.0, 0.0);
    push(13.0, 0.0);
    push(11.5, 0.62); // side D → A (curved runout)
    (pts, uvs)
}

fn boundary_edges_present(
    mesh: &draper_mesh::mesh::TriangleMesh,
    pts: &[Point3d],
) -> usize {
    use std::collections::HashMap;
    // Map each 3D point to its mesh vertex index (the zipper adds every
    // boundary point exactly once, in order).
    let mut vmap: HashMap<(u64, u64, u64), u32> = HashMap::new();
    for (i, v) in mesh.vertices.iter().enumerate() {
        vmap.insert((v.x.to_bits(), v.y.to_bits(), v.z.to_bits()), i as u32);
    }
    let mut mesh_edges: std::collections::HashSet<(u32, u32)> = std::collections::HashSet::new();
    for tri in &mesh.triangles {
        for k in 0..3 {
            let a = tri[k];
            let b = tri[(k + 1) % 3];
            mesh_edges.insert((a.min(b), a.max(b)));
        }
    }
    let n = pts.len();
    let mut missing = 0;
    for k in 0..n {
        let pa = &pts[k];
        let pb = &pts[(k + 1) % n];
        let ka = (
            pa.x.to_bits(),
            pa.y.to_bits(),
            pa.z.to_bits(),
        );
        let kb = (
            pb.x.to_bits(),
            pb.y.to_bits(),
            pb.z.to_bits(),
        );
        let (Some(va), Some(vb)) = (vmap.get(&ka), vmap.get(&kb)) else {
            missing += 1;
            continue;
        };
        if va == vb {
            continue;
        }
        if !mesh_edges.contains(&(*va.min(vb), *va.max(vb))) {
            missing += 1;
        }
    }
    missing
}

#[test]
fn test_ribbon_monotone_zipper_phases() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Debug)
        .is_test(true)
        .try_init();
    let nurbs = flat_band_surface();

    // ── Phase A: chain mode ON → the helical band is zipped ──────────
    std::env::set_var("DRAPPER_RIBBON_CHAIN", "1");
    std::env::remove_var("DRAPPER_RIBBON_ZIPPER");

    let (pts, uvs) = helical_band_boundary();
    let n = pts.len();
    let mesh = try_ribbon_monotone_zipper(&nurbs, &pts, &uvs, true)
        .expect("zipper must accept the clean helical band");
    assert_eq!(
        mesh.vertices.len(),
        n,
        "zipper must use every boundary point exactly once"
    );
    assert_eq!(
        mesh.triangle_count(),
        n - 2,
        "monotone sweep of an n-gon must emit exactly n-2 triangles (got {} for n={})",
        mesh.triangle_count(),
        n
    );
    let missing = boundary_edges_present(&mesh, &pts);
    assert_eq!(missing, 0, "{} boundary edges missing after zipper", missing);
    // Every triangle must be non-degenerate in 3D and the total area
    // must equal the UV polygon's shoelace area (the surface maps
    // (u,v) → (x,y) isometrically).
    let mut shoelace = 0.0f64;
    for k in 0..uvs.len() {
        let a = &uvs[k];
        let b = &uvs[(k + 1) % uvs.len()];
        shoelace += a.u * b.v - b.u * a.v;
    }
    let expected_area = shoelace.abs() * 0.5;
    let total_area: f64 = mesh
        .triangles
        .iter()
        .map(|t| {
            let a = mesh.vertices[t[0] as usize];
            let b = mesh.vertices[t[1] as usize];
            let c = mesh.vertices[t[2] as usize];
            let e1 = (b.x - a.x, b.y - a.y, b.z - a.z);
            let e2 = (c.x - a.x, c.y - a.y, c.z - a.z);
            let cr = (
                e1.1 * e2.2 - e1.2 * e2.1,
                e1.2 * e2.0 - e1.0 * e2.2,
                e1.0 * e2.1 - e1.1 * e2.0,
            );
            0.5 * (cr.0 * cr.0 + cr.1 * cr.1 + cr.2 * cr.2).sqrt()
        })
        .sum();
    assert!(
        (total_area - expected_area).abs() < 1e-6,
        "zipped area must equal the polygon area {:.4} (got {:.4})",
        expected_area,
        total_area
    );

    // ── Phase B: the notch face must be REJECTED (non-monotone) ──────
    let (npts, nuvs) = notched_band_boundary();
    assert!(
        try_ribbon_monotone_zipper(&nurbs, &npts, &nuvs, true).is_none(),
        "non-monotone notched band must be rejected"
    );

    // ── Phase B2: straight-side interiors are conservatively rejected ─
    // (the sweep triangle covering a collinear side point is degenerate;
    // the face falls back to the legacy path — never-worsen).
    let (cpts, cuvs) = collinear_side_band_boundary();
    assert!(
        try_ribbon_monotone_zipper(&nurbs, &cpts, &cuvs, true).is_none(),
        "collinear-side band must be rejected (conservative)"
    );

    // ── Phase C: kill-switch DRAPPER_RIBBON_ZIPPER=0 → no-op ─────────
    std::env::set_var("DRAPPER_RIBBON_ZIPPER", "0");
    assert!(
        try_ribbon_monotone_zipper(&nurbs, &pts, &uvs, true).is_none(),
        "kill-switch must disable the zipper"
    );
    std::env::remove_var("DRAPPER_RIBBON_ZIPPER");

    // ── Phase D: default mode (no DRAPPER_RIBBON_CHAIN) → no-op ──────
    std::env::remove_var("DRAPPER_RIBBON_CHAIN");
    assert!(
        try_ribbon_monotone_zipper(&nurbs, &pts, &uvs, true).is_none(),
        "zipper must be inert without the opt-in chain env"
    );
}
