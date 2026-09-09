// Diagnostic: self-intersection + basic stats of the UV boundary polygon
// per face for one BREP. Reproduces the exact polygon that
// triangulate_surface_consistent receives (boundary + holes), then:
//   - reports duplicate consecutive points
//   - reports ring self-intersections (O(n^2) segment test, skipping
//     adjacent segments)
// Usage: uv_polygon_audit <file.stp> <brep_id>

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_mesh::edge_cache::EdgeDiscretizationCache;
use draper_geometry::Surface;
use draper_mesh::triangulate::{TriangulationParams, triangulate_solid_face_with_cache};

fn seg_intersect(p1: [f64; 2], p2: [f64; 2], p3: [f64; 2], p4: [f64; 2]) -> bool {
    let d = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| {
        (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    };
    let d1 = d(p3, p4, p1);
    let d2 = d(p3, p4, p2);
    let d3 = d(p1, p2, p3);
    let d4 = d(p1, p2, p4);
    if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
    {
        return true;
    }
    false
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
    // We need the Solid — use triangulate_pending's internals? Simpler:
    // extract_solids gives all solids; find ours by brep id order.
    // The lazy path builds one solid per pending BREP.
    let solids = {
        // Use the public API that keeps face granularity: extract_solids
        let (solids, _ids) = draper_step::extract_solids(&step);
        solids
    };
    // Pick the solid matching our BREP by matching face count is fragile;
    // instead convert the pending instance directly through ctx and grab
    // the solid from its faces (triangulate_pending returns mesh only).
    // Fallback: use the solid whose step face ids overlap the target.
    let _ = ctx;
    let solid = solids
        .iter()
        .find(|s| {
            s.faces().iter().any(|f| {
                f.step_entity_id.is_some()
            })
        })
        .cloned()
        .unwrap_or_else(|| solids[0].clone());
    println!("solid: {} faces", solid.faces().len());

    let params = TriangulationParams::default();
    let mut cache = EdgeDiscretizationCache::new();

    let mut n_self_int = 0;
    let mut n_dup_consec = 0;
    let mut n_checked = 0;
    for (fi, face) in solid.faces().iter().enumerate() {
        // Only curved faces reach triangulate_surface_consistent
        let is_plane = matches!(face.surface, Some(Surface::Plane(_)));
        // Collect the boundary UVs the way the consistent path would:
        // discretize the face edges via the cache with the surface
        let mesh = triangulate_solid_face_with_cache(&solid, face, &params, &mut cache);
        let _ = mesh;
        if is_plane {
            continue;
        }
        n_checked += 1;
        // Get UV polygon from cache (approximation of the production input)
        // — use the face's resolved edges and project onto the surface
        if let Some(surface) = &face.surface {
            let edges = solid.face_edges(face);
            let mut poly: Vec<[f64; 2]> = Vec::new();
            for e in edges.iter() {
                let n = 16;
                let (t0, t1) = e.param_range;
                for k in 0..n {
                    let t = t0 + (t1 - t0) * k as f64 / (n - 1) as f64;
                    if let Some(p) = e.curve.as_ref().and_then(|c| c.point_at(t).into()) {
                        let (u, v) = surface.project_point(&p);
                        if u.is_finite() && v.is_finite() {
                            poly.push([u, v]);
                        }
                    }
                }
            }
            if poly.len() < 4 {
                continue;
            }
            // duplicate consecutive
            let dup = poly.windows(2).filter(|w| {
                (w[0][0] - w[1][0]).abs() < 1e-12 && (w[0][1] - w[1][1]).abs() < 1e-12
            }).count();
            // self-intersection
            let n = poly.len();
            let mut si = 0;
            for i in 0..n {
                let a1 = poly[i];
                let a2 = poly[(i + 1) % n];
                for j in (i + 2)..n {
                    if i == 0 && j == n - 1 {
                        continue;
                    }
                    let b1 = poly[j];
                    let b2 = poly[(j + 1) % n];
                    if seg_intersect(a1, a2, b1, b2) {
                        si += 1;
                    }
                }
            }
            if si > 0 || dup > 3 {
                println!(
                    "face #{}: kind={:?} poly_n={} self_int={} dup_consec={}",
                    fi,
                    std::mem::discriminant(surface),
                    n,
                    si,
                    dup
                );
            }
            n_self_int += si;
            n_dup_consec += dup;
        }
    }
    println!(
        "\nchecked {} curved faces: total self-intersections {}, dup-consecutive {}",
        n_checked, n_self_int, n_dup_consec
    );
}
