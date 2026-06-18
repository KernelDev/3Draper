// Diagnostic: print edge step_ids per face to understand sharing.

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let path = std::env::args().nth(1).unwrap_or("test/as1-oc-214_bolt.stp".to_string());
    let data = std::fs::read_to_string(&path).expect("read");
    let step = parse_step(&data).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);

    let p = &pending[0];
    let ctx = StepConversionContext::new(&step);
    let inst = match ctx.triangulate_pending(p) {
        Some(i) => i,
        None => { eprintln!("Triangulation failed"); return; }
    };

    println!("BREP #{}: {} faces", p.brep_id, inst.faces.len());

    for (fi, face) in inst.faces.iter().enumerate() {
        let tris = face.triangle_range.1 - face.triangle_range.0;
        println!("\n=== Face {} (STEP #{}, surf={}) === tris={}, forward={}",
            fi + 1, face.step_face_id, face.surface_type, tris, face.forward);

        println!("  Outer edges ({}):", face.outer_edges.len());
        for (ei, _edge) in face.outer_edges.iter().enumerate() {
            let sid = face.outer_edge_step_ids.get(ei).copied().unwrap_or(0);
            println!("    edge {}: step_id={}", ei, sid);
        }

        for (li, inner_edges) in face.inner_edges.iter().enumerate() {
            println!("  Inner loop {} ({} edges):", li, inner_edges.len());
            let step_ids = face.inner_edge_step_ids.get(li);
            for (ei, _edge) in inner_edges.iter().enumerate() {
                let sid = step_ids.and_then(|ids| ids.get(ei).copied()).unwrap_or(0);
                println!("    edge {}: step_id={}", ei, sid);
            }
        }
    }

    // Collect all step_ids used and count faces sharing each
    use std::collections::HashMap;
    let mut step_id_faces: HashMap<i64, Vec<usize>> = HashMap::new();
    for (fi, face) in inst.faces.iter().enumerate() {
        let mut seen = std::collections::HashSet::new();
        for &sid in &face.outer_edge_step_ids {
            if sid != 0 && seen.insert(sid) {
                step_id_faces.entry(sid).or_default().push(fi + 1);
            }
        }
        for ids in &face.inner_edge_step_ids {
            for &sid in ids {
                if sid != 0 && seen.insert(sid) {
                    step_id_faces.entry(sid).or_default().push(fi + 1);
                }
            }
        }
    }

    println!("\n\nEdge step_id sharing (only shared edges):");
    let mut shared: Vec<_> = step_id_faces.iter().filter(|(_, v)| v.len() > 1).collect();
    shared.sort_by_key(|(sid, _)| **sid);
    for (sid, faces) in shared {
        println!("  step_id {} shared by faces {:?}", sid, faces);
    }
}
