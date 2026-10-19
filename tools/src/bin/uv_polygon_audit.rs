// uv_polygon_audit v2 — s102 emission slice (chord-fan root cause).
//
// For a target BREP (selected properly by brep_id) and an optional face
// filter, this reproduces the EXACT production emission inputs/outputs:
//   1. runs the production per-face triangulation (which populates the
//      shared edge-discretization cache — same call the pipeline uses),
//   2. rebuilds the exact production UV polygon + holes from the cache
//      (collect_face_boundary_with_uv_from_cache — the same function the
//      sphere/cylinder drivers call),
//   3. dumps: polygon vertices (u, v, 3D), domain-edge structure
//      (u=0 / u=PI / u=2PI cut edges, their v-ranges),
//   4. dumps emitted triangles and classifies them:
//      - 3-ON-BND: all 3 verts on the outer polygon (chord-fan family)
//      - BRIDGE: verts spanning two different domain cut edges
//        (domain-edge bridge = the s101 addendum signature)
//
// Usage: uv_polygon_audit <file.stp> <brep_id> [face_idx]

use draper_step::{parse_step, extract_solids};
use draper_geometry::Surface;
use draper_mesh::edge_cache::EdgeDiscretizationCache;
use draper_mesh::triangulate::{
    TriangulationParams,
    triangulate_solid_face_with_cache,
    stage_solid_face,
    collect_face_boundary_with_uv_from_cache,
    collect_face_holes_with_uv_from_cache,
};

fn dist2(a: [f64; 3], b: [f64; 3]) -> f64 {
    let d0 = a[0] - b[0];
    let d1 = a[1] - b[1];
    let d2 = a[2] - b[2];
    d0 * d0 + d1 * d1 + d2 * d2
}

fn p3(p: &draper_geometry::Point3d) -> [f64; 3] {
    [p.x, p.y, p.z]
}

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Error)
        .init();
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).map(|s| s.as_str()).unwrap_or("test/drill_top.stp");
    let brep_id: i64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(47598);
    let face_filter: Option<usize> = args.get(3).and_then(|s| s.parse().ok());

    let data = std::fs::read_to_string(path).expect("read STEP");
    let step = parse_step(&data).expect("parse STEP");
    let (solids, ids) = extract_solids(&step);
    let sidx = ids
        .iter()
        .position(|&id| id == brep_id)
        .unwrap_or_else(|| panic!("BREP {} not found among {} solids", brep_id, ids.len()));
    let solid = &solids[sidx];
    println!(
        "# solid for BREP#{}: {} faces (of {} solids)",
        brep_id,
        solid.faces().len(),
        solids.len()
    );

    let params = TriangulationParams::default();
    let mut cache = EdgeDiscretizationCache::new();
    let pi = std::f64::consts::PI;
    let tau = 2.0 * std::f64::consts::PI;

    for (fi, face) in solid.faces().iter().enumerate() {
        if let Some(want) = face_filter {
            if fi != want {
                continue;
            }
        }
        let surface = match &face.surface {
            Some(s) => s.clone(),
            None => continue,
        };
        if matches!(surface, Surface::Plane(_)) {
            continue; // planes use a different driver
        }
        let step_ids: Vec<i64> = solid
            .face_edges(face)
            .iter()
            .filter_map(|e| e.step_entity_id)
            .collect();

        // 1. production triangulation for this face (populates cache)
        let mesh = triangulate_solid_face_with_cache(solid, face, &params, &mut cache);
        // 2. exact production polygon from the cache
        let staged = stage_solid_face(solid, face);
        let (b3, buv) = collect_face_boundary_with_uv_from_cache(&staged, &cache, &surface);
        let (h3, huv) = collect_face_holes_with_uv_from_cache(&staged, &cache, &surface);

        println!(
            "\n=== FACE #{} kind={:?} step_ids={:?} bnd={} holes={} tris={} verts={}",
            fi,
            std::mem::discriminant(&surface),
            step_ids,
            b3.len(),
            h3.len(),
            mesh.triangles.len(),
            mesh.vertices.len(),
        );

        for (i, (p, uv)) in b3.iter().zip(buv.iter()).enumerate() {
            println!(
                "POLY {:4} u={:9.4} v={:9.4} xyz=({:9.4},{:9.4},{:9.4})",
                i, uv.u, uv.v, p.x, p.y, p.z
            );
        }
        for (hi, (hp, hu)) in h3.iter().zip(huv.iter()).enumerate() {
            println!("HOLE {} n={}", hi, hp.len());
            for (i, (p, uv)) in hp.iter().zip(hu.iter()).enumerate() {
                println!(
                    "  HP{:4} u={:9.4} v={:9.4} xyz=({:9.4},{:9.4},{:9.4})",
                    i, uv.u, uv.v, p.x, p.y, p.z
                );
            }
        }

        // domain-edge structure
        let on = |u: f64, x: f64| (u - x).abs() < 0.03;
        let mut e0 = Vec::new();
        let mut epi = Vec::new();
        let mut etau = Vec::new();
        for (i, uv) in buv.iter().enumerate() {
            if on(uv.u, 0.0) {
                e0.push(i);
            }
            if on(uv.u, pi) {
                epi.push(i);
            }
            if on(uv.u, tau) {
                etau.push(i);
            }
        }
        let vrange = |idx: &[usize]| -> (f64, f64) {
            idx.iter()
                .map(|&i| buv[i].v)
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), v| {
                    (a.min(v), b.max(v))
                })
        };
        if !e0.is_empty() {
            let (a, b) = vrange(&e0);
            println!(
                "CUT u=0   : n={} v-range=[{:.4},{:.4}] idx={:?}",
                e0.len(), a, b, e0
            );
        }
        if !epi.is_empty() {
            let (a, b) = vrange(&epi);
            println!(
                "CUT u=PI  : n={} v-range=[{:.4},{:.4}] idx={:?}",
                epi.len(), a, b, epi
            );
        }
        if !etau.is_empty() {
            let (a, b) = vrange(&etau);
            println!(
                "CUT u=2PI : n={} v-range=[{:.4},{:.4}] idx={:?}",
                etau.len(), a, b, etau
            );
        }

        // classify emitted triangles: 3-on-boundary + domain-edge bridges
        let scale = mesh
            .vertices
            .iter()
            .map(|v| v.x.abs().max(v.y.abs()).max(v.z.abs()))
            .fold(1.0_f64, f64::max)
            .max(1.0);
        let eps2 = (1e-7 * scale) * (1e-7 * scale);
        let poly_xyz: Vec<[f64; 3]> = b3.iter().map(|p| p3(p)).collect();
        let mut n3bnd = 0;
        let mut nbridge = 0;
        let mut chords: Vec<String> = Vec::new();
        for (ti, tri) in mesh.triangles.iter().enumerate() {
            let hits: Vec<Option<usize>> = tri
                .iter()
                .map(|&vi| {
                    let mv = p3(&mesh.vertices[vi as usize]);
                    poly_xyz.iter().position(|px| dist2(*px, mv) < eps2)
                })
                .collect();
            let n_on = hits.iter().filter(|h| h.is_some()).count();
            if n_on < 3 {
                continue;
            }
            n3bnd += 1;
            let cut_of = |uv: &draper_geometry::Point2d| -> &'static str {
                if on(uv.u, 0.0) {
                    "u0"
                } else if on(uv.u, pi) {
                    "uPI"
                } else if on(uv.u, tau) {
                    "u2PI"
                } else {
                    "bnd"
                }
            };
            let cuts: Vec<&'static str> = hits
                .iter()
                .map(|h| cut_of(&buv[h.unwrap()]))
                .collect();
            let uniq: std::collections::HashSet<&'static str> =
                cuts.iter().copied().collect();
            if uniq.len() >= 2 {
                nbridge += 1;
            }
            chords.push(format!(
                "CHORD tri#{} verts=({},{},{}) poly=({},{},{}) uv=({:.4},{:.4})({:.4},{:.4})({:.4},{:.4}) cuts={:?}",
                ti,
                tri[0],
                tri[1],
                tri[2],
                hits[0].unwrap(),
                hits[1].unwrap(),
                hits[2].unwrap(),
                buv[hits[0].unwrap()].u,
                buv[hits[0].unwrap()].v,
                buv[hits[1].unwrap()].u,
                buv[hits[1].unwrap()].v,
                buv[hits[2].unwrap()].u,
                buv[hits[2].unwrap()].v,
                cuts
            ));
        }
        for line in chords.iter() {
            println!("{}", line);
        }
        println!("FACE #{} SUMMARY: 3-on-bnd={} bridges={}", fi, n3bnd, nbridge);
    }
    println!("\n# done");
}
