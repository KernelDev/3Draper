// Diagnostic: dump per-face edge identity (TopoId, step_entity_id, type, param_range)
// to verify whether shared STEP EDGE_CURVEs produce shared Edge identities (C5 analysis).
// Run: cargo run --release --bin edge_id_diag -- test/nist_cone.stp

use draper_step::{parse_step_file, extract_solids};
use draper_geometry::Surface;

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
    println!("Solids: {}", solids.len());

    for (si, solid) in solids.iter().enumerate() {
        println!("\n=== Solid #{} ===", si);
        // Collect (edge_key, face_idx) occurrences across all faces.
        // edge_key = step_entity_id when present, else TopoId (marked).
        let mut occurrences: std::collections::HashMap<String, Vec<(usize, usize)>> =
            std::collections::HashMap::new();

        for (fi, face) in solid.faces().iter().enumerate() {
            let surf_name = face.surface.as_ref().map(|s| {
                match s {
                    Surface::Plane(_) => "Plane",
                    Surface::Sphere(_) => "Sphere",
                    Surface::Cylinder(_) => "Cylinder",
                    Surface::Cone(_) => "Cone",
                    Surface::Torus(_) => "Torus",
                    _ => "Other",
                }
            }).unwrap_or("None");

            // Walk the outer wire: coedge -> edge lookup shows what triangulation sees.
            if let Some(ref wire) = face.outer_wire {
                for (ci, coedge) in wire.coedges.iter().enumerate() {
                    if let Some(edge) = face.edges.iter().find(|e| e.id == coedge.edge) {
                        let et = edge.curve.as_ref().map(|c| match c {
                            draper_geometry::Curve3d::Line(_) => "Line",
                            draper_geometry::Curve3d::Circle(_) => "Circle",
                            draper_geometry::Curve3d::Nurbs(_) => "Nurbs",
                            _ => "Other",
                        }).unwrap_or("None");
                        let key = match edge.step_entity_id {
                            Some(sid) => format!("step#{}", sid),
                            None => format!("topo{}", edge.id),
                        };
                        println!("Face {} [{}] coedge {} -> edge {} key={} type={} range=({:.4},{:.4}) fwd={}",
                            fi, surf_name, ci, edge.id, key, et,
                            edge.param_range.0, edge.param_range.1, coedge.forward);
                        occurrences.entry(key).or_default().push((fi, ci));
                    }
                }
            }
        }

        println!("\n--- Shared edge keys (used by 2+ coedges) ---");
        let mut shared: Vec<_> = occurrences.iter()
            .filter(|(_, v)| v.len() > 1)
            .collect();
        shared.sort_by_key(|(k, _)| (*k).clone());
        for (k, v) in &shared {
            println!("  {} -> {} uses: {:?}", k, v.len(), v);
        }
        let total_keys = occurrences.len();
        let shared_keys = shared.len();
        println!("\nTotal distinct edge keys: {}, shared: {}", total_keys, shared_keys);
    }
}
