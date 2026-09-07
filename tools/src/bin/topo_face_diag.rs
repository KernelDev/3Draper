// Diagnostic: per-face triangulation details for small solids.
// Prints each face's boundary points and resulting triangle count
// to find which face emits extra/missing triangles.
// Run: cargo run --release --bin topo_face_diag -- test/nist_cube.stp

use draper_step::{parse_step_file, extract_solids};
use draper_mesh::triangulate::{TriangulationParams, triangulate_solid_face_with_cache};
use draper_mesh::edge_cache::EdgeDiscretizationCache;
use draper_geometry::Surface;

fn main() {
    let path = std::env::args().nth(1).unwrap_or_else(||
        "test/nist_cube.stp".to_string()
    );
    println!("Loading: {}", path);
    let step = match parse_step_file(&path) {
        Ok(s) => s,
        Err(e) => { eprintln!("Parse error: {}", e); return; }
    };
    let (solids, _) = extract_solids(&step);

    for (si, solid) in solids.iter().enumerate() {
        println!("\n=== Solid #{} ===", si);
        for (fi, face) in solid.faces().iter().enumerate() {
            let surf_name = face.surface.as_ref().map(|s| match s {
                Surface::Plane(_) => "Plane",
                Surface::Cone(_) => "Cone",
                Surface::Cylinder(_) => "Cylinder",
                _ => "Other",
            }).unwrap_or("None");
            println!("\nFace {} [{}] edges={} forward={}", fi, surf_name, solid.face_edges(face).len(), face.forward);
            if let Some(ref wire) = face.outer_wire {
                println!("  outer wire: {} coedges", wire.coedges.len());
                for (ci, coedge) in wire.coedges.iter().enumerate() {
                    let face_edges = solid.face_edges(face);
                    let e = face_edges.iter().find(|e| e.id == coedge.edge);
                    if let Some(e) = e {
                        println!("    coedge {}: edge={} fwd={} range=({:.4},{:.4}) step_id={:?}",
                            ci, e.id, coedge.forward, e.param_range.0, e.param_range.1,
                            e.step_entity_id);
                    } else {
                        println!("    coedge {}: edge={} — NOT FOUND in edge store!", ci, coedge.edge);
                    }
                }
            }
            // Triangulate the face standalone (store-resolved boundary)
            let params = TriangulationParams::default();
            let mut cache = EdgeDiscretizationCache::new();
            let mesh = triangulate_solid_face_with_cache(solid, face, &params, &mut cache);
            println!("  standalone triangulation: {} vertices, {} triangles", mesh.vertices.len(), mesh.triangles.len());
            for (ti, t) in mesh.triangles.iter().enumerate() {
                let p = |i: u32| {
                    let v = mesh.vertices[i as usize];
                    format!("({:.2},{:.2},{:.2})", v.x, v.y, v.z)
                };
                println!("    tri {}: {} {} {}", ti, p(t[0]), p(t[1]), p(t[2]));
            }
        }
    }
}
