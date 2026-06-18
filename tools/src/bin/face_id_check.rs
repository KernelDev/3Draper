// Print face_id distribution in the mesh.
use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use std::collections::HashMap;

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

    println!("BREP has {} faces (from inst.faces)", inst.faces.len());
    for (fi, face) in inst.faces.iter().enumerate() {
        let tris = face.triangle_range.1 - face.triangle_range.0;
        println!("  Face index {} (1-based {}): STEP #{}, surf={}, tris={}, range={:?}",
            fi, fi + 1, face.step_face_id, face.surface_type, tris, face.triangle_range);
    }

    // Check mesh's triangle_face_ids
    let mesh = &inst.mesh;
    println!("\nMesh has {} triangles, {} vertices",
        mesh.triangles.len(), mesh.vertices.len());

    if let Some(face_ids) = mesh.triangle_face_ids.as_ref() {
        println!("triangle_face_ids: {} entries", face_ids.len());

        // Count per face_id
        let mut counts: HashMap<u64, usize> = HashMap::new();
        for &fid in face_ids {
            *counts.entry(fid).or_insert(0) += 1;
        }
        let mut sorted: Vec<_> = counts.iter().collect();
        sorted.sort_by_key(|(k, _)| **k);
        for (fid, count) in sorted {
            println!("  face_id {}: {} triangles", fid, count);
        }

        // Check the range of triangle indices per face_id
        let mut ranges: HashMap<u64, (usize, usize)> = HashMap::new();
        for (ti, &fid) in face_ids.iter().enumerate() {
            let e = ranges.entry(fid).or_insert((usize::MAX, 0));
            e.0 = e.0.min(ti);
            e.1 = e.1.max(ti);
        }
        println!("\nTriangle index ranges per face_id:");
        let mut sorted_ranges: Vec<_> = ranges.iter().collect();
        sorted_ranges.sort_by_key(|(k, _)| **k);
        for (fid, (start, end)) in sorted_ranges {
            println!("  face_id {}: tri indices {} to {} ({} tris)", fid, start, end, end - start + 1);
        }
    }
}
