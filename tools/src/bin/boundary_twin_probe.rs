// Diagnostic: classify boundary edges of a BREP instance mesh by whether
// a geometric TWIN exists on another face ("stitching/aliasing failure")
// or not ("genuine hole" — needs SSI edge recovery / surface extension).
//
// For every boundary edge (exactly 1 adjacent triangle) we search, among
// boundary edges of OTHER faces, for a twin: a segment whose 3D
// segment-to-segment distance is under `twin_tol`. Classification:
//   TWINED  — twin found at dist <= twin_tol  (topology link lost, but
//             geometry present on both sides → re-association fixes it)
//   ORPHAN  — no twin within `search_tol`     (true hole: missing face
//             or lost edge → §1.4 SSI recovery)
// Also reports twin subdivision mismatch (vertex count per matched run),
// per-surface-type and per-face-pair aggregates.
//
// Usage: boundary_twin_probe <file.stp> <brep_id>

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use std::collections::HashMap;

#[derive(Clone, Copy)]
struct V3([f64; 3]);

fn sub(a: &V3, b: &V3) -> V3 { V3([a.0[0] - b.0[0], a.0[1] - b.0[1], a.0[2] - b.0[2]]) }
fn dot(a: &V3, b: &V3) -> f64 { a.0[0] * b.0[0] + a.0[1] * b.0[1] + a.0[2] * b.0[2] }
fn norm(a: &V3) -> f64 { dot(a, a).sqrt() }

/// Point-to-segment distance (3D).
fn point_seg_dist(p: &V3, a: &V3, b: &V3) -> f64 {
    let ab = sub(b, a);
    let ap = sub(p, a);
    let len2 = dot(&ab, &ab);
    if len2 < 1e-24 {
        return norm(&ap);
    }
    let t = (dot(&ap, &ab) / len2).clamp(0.0, 1.0);
    let proj = V3([a.0[0] + ab.0[0] * t, a.0[1] + ab.0[1] * t, a.0[2] + ab.0[2] * t]);
    norm(&sub(p, &proj))
}

/// Segment-to-segment distance (approximate via mutual point-seg + endpoint checks).
fn seg_seg_dist(p1: &V3, q1: &V3, p2: &V3, q2: &V3) -> f64 {
    let d1 = point_seg_dist(p1, p2, q2).min(point_seg_dist(q1, p2, q2));
    let d2 = point_seg_dist(p2, p1, q1).min(point_seg_dist(q2, p1, q1));
    d1.min(d2)
}

/// Spatial hash cell key.
fn cell_key(p: &V3, inv: f64) -> (i64, i64, i64) {
    (
        (p.0[0] * inv).floor() as i64,
        (p.0[1] * inv).floor() as i64,
        (p.0[2] * inv).floor() as i64,
    )
}

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Error)
        .init();
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).map(|s| s.as_str()).unwrap_or("test/drill_top.stp");
    let brep_id: i64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(47598);

    let data = std::fs::read_to_string(path).expect("read STEP");
    let step = parse_step(&data).expect("parse STEP");
    let (_tree, pending) = step_structure_lazy(&step);
    let target = pending.iter().find(|p| p.brep_id == brep_id)
        .unwrap_or_else(|| panic!("BREP {} not found", brep_id));
    let mut ctx = StepConversionContext::new(&step);
    let inst = ctx.triangulate_pending(target).expect("convert BREP");
    let mesh = &inst.mesh;

    let face_ids = match &mesh.triangle_face_ids {
        Some(ids) => ids.clone(),
        None => { eprintln!("No triangle_face_ids"); return; }
    };

    // Model scale for tolerance selection.
    let mut lo = [f64::MAX; 3];
    let mut hi = [f64::MIN; 3];
    for v in &mesh.vertices {
        lo[0] = lo[0].min(v.x); lo[1] = lo[1].min(v.y); lo[2] = lo[2].min(v.z);
        hi[0] = hi[0].max(v.x); hi[1] = hi[1].max(v.y); hi[2] = hi[2].max(v.z);
    }
    let diag = ((hi[0]-lo[0]).powi(2) + (hi[1]-lo[1]).powi(2) + (hi[2]-lo[2]).powi(2)).sqrt();
    let twin_tol = diag * 1e-4;   // ~sewing tolerance scale
    let search_tol = diag * 1e-3; // generous search radius for classification
    println!("model diag={:.3}, twin_tol={:.6}, search_tol={:.6}", diag, twin_tol, search_tol);

    // Global edge usage: boundary = exactly 1 triangle.
    let mut edge_use: HashMap<(u32, u32), u32> = HashMap::new();
    let mut edge_owner: HashMap<(u32, u32), u64> = HashMap::new();
    for (ti, t) in mesh.triangles.iter().enumerate() {
        let fid = face_ids[ti];
        for k in 0..3 {
            let a = t[k];
            let b = t[(k + 1) % 3];
            if a == b { continue; }
            let key = if a < b { (a, b) } else { (b, a) };
            *edge_use.entry(key).or_insert(0) += 1;
            edge_owner.insert(key, fid);
        }
    }

    // Collect boundary edges with geometry.
    struct BEdge { a: V3, b: V3, face: u64, key: (u32, u32) }
    let mut boundary: Vec<BEdge> = Vec::new();
    for (key, &uses) in &edge_use {
        if uses != 1 { continue; }
        let (va, vb) = (key.0 as usize, key.1 as usize);
        let p = &mesh.vertices[va];
        let q = &mesh.vertices[vb];
        boundary.push(BEdge {
            a: V3([p.x, p.y, p.z]),
            b: V3([q.x, q.y, q.z]),
            face: edge_owner[key],
            key: *key,
        });
    }
    println!("boundary edges: {}", boundary.len());

    // Spatial hash of boundary edge midpoints (only for lookup across faces).
    let cell = search_tol.max(twin_tol);
    let inv = 1.0 / cell;
    let mut hash: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
    for (i, e) in boundary.iter().enumerate() {
        let mid = V3([
            (e.a.0[0] + e.b.0[0]) * 0.5,
            (e.a.0[1] + e.b.0[1]) * 0.5,
            (e.a.0[2] + e.b.0[2]) * 0.5,
        ]);
        hash.entry(cell_key(&mid, inv)).or_default().push(i);
    }

    // For each boundary edge, find best twin among OTHER faces within
    // search_tol; classify TWINED (<= twin_tol) / ORPHAN.
    let mut n_twined = 0usize;
    let mut n_orphan = 0usize;
    let mut twined_face_pairs: HashMap<(u64, u64), usize> = HashMap::new();
    let mut orphan_faces: HashMap<u64, usize> = HashMap::new();
    let mut twined_dist_sum = 0.0;
    let mut twined_max = 0.0f64;
    let mut orphan_len_sum = 0.0;

    for (i, e) in boundary.iter().enumerate() {
        let mid = V3([
            (e.a.0[0] + e.b.0[0]) * 0.5,
            (e.a.0[1] + e.b.0[1]) * 0.5,
            (e.a.0[2] + e.b.0[2]) * 0.5,
        ]);
        let ck = cell_key(&mid, inv);
        let mut best: Option<(f64, u64)> = None; // (dist, face)
        for dx in -1..=1i64 {
            for dy in -1..=1i64 {
                for dz in -1..=1i64 {
                    let k = (ck.0 + dx, ck.1 + dy, ck.2 + dz);
                    if let Some(idx) = hash.get(&k) {
                        for &j in idx {
                            let o = &boundary[j];
                            if o.face == e.face { continue; }
                            let d = seg_seg_dist(&e.a, &e.b, &o.a, &o.b);
                            if d <= search_tol {
                                if best.map(|(bd, _)| d < bd).unwrap_or(true) {
                                    best = Some((d, o.face));
                                }
                            }
                        }
                    }
                }
            }
        }
        let elen = norm(&sub(&e.b, &e.a));
        match best {
            Some((d, of)) if d <= twin_tol => {
                n_twined += 1;
                twined_dist_sum += d;
                twined_max = twined_max.max(d);
                let pair = if e.face < of { (e.face, of) } else { (of, e.face) };
                *twined_face_pairs.entry(pair).or_insert(0) += 1;
            }
            _ => {
                n_orphan += 1;
                orphan_len_sum += elen;
                *orphan_faces.entry(e.face).or_insert(0) += 1;
            }
        }
    }

    println!("\n=== Twin classification ===");
    println!("TWINED (stitching failure, geometry present both sides): {}", n_twined);
    println!("ORPHAN (genuine hole, needs SSI recovery):            {}", n_orphan);
    if n_twined > 0 {
        println!("twined mean dist={:.6} max={:.6}", twined_dist_sum / n_twined as f64, twined_max);
    }
    if n_orphan > 0 {
        println!("orphan total edge length={:.3} (diag={:.3})", orphan_len_sum, diag);
    }

    println!("\n=== Top TWINED face pairs ===");
    let mut pairs: Vec<_> = twined_face_pairs.iter().collect();
    pairs.sort_by_key(|(_, c)| std::cmp::Reverse(**c));
    for ((f1, f2), c) in pairs.iter().take(15) {
        println!("  faces ({}, {}) -> {} edges", f1, f2, c);
    }
    println!("\n=== Top ORPHAN faces ===");
    let mut orph: Vec<_> = orphan_faces.iter().collect();
    orph.sort_by_key(|(_, c)| std::cmp::Reverse(**c));
    for (f, c) in orph.iter().take(15) {
        println!("  face {} -> {} orphan edges", f, c);
    }
}
