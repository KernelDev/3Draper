// Diagnostic: verify EdgeDiscretizationCache unification for shared STEP edges.
// Loads a STEP file, emulates the sequential pre-population path, then
// compares the discretization entries of edges that share the same
// step_entity_id but have different TopoIds.
// Run: cargo run --release --bin cache_unify_diag -- test/nist_cone.stp

use draper_step::{parse_step_file, extract_solids};
use draper_mesh::edge_cache::EdgeDiscretizationCache;
use draper_geometry::Point3d;

fn main() {
    let path = std::env::args().nth(1).unwrap_or_else(||
        "test/nist_cone.stp".to_string()
    );
    println!("Loading: {}", path);
    let step = match parse_step_file(&path) {
        Ok(s) => s,
        Err(e) => { eprintln!("Parse error: {}", e); return; }
    };
    let (solids, _) = extract_solids(&step);

    for (si, solid) in solids.iter().enumerate() {
        println!("\n=== Solid #{} ===", si);

        // Group edges by cache key (step_entity_id or TopoId) across faces.
        let mut groups: std::collections::HashMap<i64, Vec<(usize, draper_topology::TopoId)>> =
            std::collections::HashMap::new();
        // Compute bbox from all edge endpoints (emulates solid_bounding_box).
        let mut bmin = Point3d::new(f64::MAX, f64::MAX, f64::MAX);
        let mut bmax = Point3d::new(f64::MIN, f64::MIN, f64::MIN);
        for (fi, face) in solid.faces().iter().enumerate() {
            for edge in &face.edges {
                let key = edge.step_entity_id.unwrap_or_else(|| edge.id.to_u64() as i64);
                groups.entry(key).or_default().push((fi, edge.id));
                for p in [edge.start_point(), edge.end_point()] {
                    if let Some(p) = p {
                        bmin.x = bmin.x.min(p.x); bmin.y = bmin.y.min(p.y); bmin.z = bmin.z.min(p.z);
                        bmax.x = bmax.x.max(p.x); bmax.y = bmax.y.max(p.y); bmax.z = bmax.z.max(p.z);
                    }
                }
            }
        }

        // Build cache with same params as triangulate_solid_with_report
        let mut cache = EdgeDiscretizationCache::with_adaptive_tolerance(
            &bmin, &bmax, 64,
        );

        // Discretize every face's edges in face order (sequential path emulation)
        for (_fi, face) in solid.faces().iter().enumerate() {
            if let Some(ref surface) = face.surface {
                if let Some(ref wire) = face.outer_wire {
                    for coedge in &wire.coedges {
                        if let Some(edge) = face.edges.iter().find(|e| e.id == coedge.edge) {
                            if edge.degenerate { continue; }
                            cache.discretize_edge(edge, face.id, surface, 64, coedge.curve_2d.as_ref());
                        }
                    }
                }
                for wire in &face.inner_wires {
                    for coedge in &wire.coedges {
                        if let Some(edge) = face.edges.iter().find(|e| e.id == coedge.edge) {
                            if edge.degenerate { continue; }
                            cache.discretize_edge(edge, face.id, surface, 64, coedge.curve_2d.as_ref());
                        }
                    }
                }
            }
        }

        // For each shared group, compare point sequences
        let mut keys: Vec<_> = groups.iter().collect();
        keys.sort_by_key(|(k, _)| **k);
        for (key, members) in &keys {
            if members.len() < 2 { continue; }
            println!("\nShared key step#{} ({} edge copies): {:?}", key, members.len(),
                members.iter().map(|(f, id)| format!("face{}:{}", f, id)).collect::<Vec<_>>());
            for w in members.windows(2) {
                let (f0, id0) = w[0];
                let (f1, id1) = w[1];
                let d0 = cache.get(id0);
                let d1 = cache.get(id1);
                match (d0, d1) {
                    (Some(a), Some(b)) => {
                        let same_ptr = std::ptr::eq(a, b);
                        let n0 = a.points_3d.len();
                        let n1 = b.points_3d.len();
                        let identical = n0 == n1 && a.points_3d.iter().zip(b.points_3d.iter())
                            .all(|(p, q)| p.x == q.x && p.y == q.y && p.z == q.z);
                        println!("  face{}:{} ({} pts) vs face{}:{} ({} pts) -> same_entry={} identical_points={}",
                            f0, id0, n0, f1, id1, n1, same_ptr, identical);
                        if !identical {
                            println!("    face{} first 3: {:?}", f0, &a.points_3d.iter().take(3).collect::<Vec<_>>());
                            println!("    face{} first 3: {:?}", f1, &b.points_3d.iter().take(3).collect::<Vec<_>>());
                        }
                    }
                    _ => println!("  MISSING cache entry for one of the copies!"),
                }
            }
        }
        let stats = cache.stats();
        println!("\nCache stats: total={}, hits={}, misses={}, shared={}",
            stats.total_edges, stats.cache_hits, stats.cache_misses, stats.shared_edges);
    }
}
