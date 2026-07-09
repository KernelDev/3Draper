// Show triangles that use specific vertices.
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

    let verts = &inst.mesh.vertices;
    let tris = &inst.mesh.triangles;
    let face_ids = inst.mesh.triangle_face_ids.as_ref();

    // Find vertices at the boundary edge positions
    let target_positions: Vec<(f64, f64, f64)> = vec![
        (88.15, 52.50, -4.0),  // vertex 127
        (87.99, 52.50, -4.0),  // vertex 126
        (87.83, 42.50, -4.0),  // vertex 190
        (87.99, 42.50, -4.0),  // vertex 189
        (87.99, 52.50, 30.0),  // vertex 438
        (87.99, 42.50, 30.0),  // vertex 501
    ];

    let tol = 0.05;
    let mut target_indices: HashMap<usize, (f64,f64,f64)> = HashMap::new();
    for (vi, v) in verts.iter().enumerate() {
        for (tx, ty, tz) in &target_positions {
            let dx = v.x - tx;
            let dy = v.y - ty;
            let dz = v.z - tz;
            if dx*dx + dy*dy + dz*dz < tol * tol {
                target_indices.insert(vi, (*tx, *ty, *tz));
                break;
            }
        }
    }

    println!("Found {} target vertices:", target_indices.len());
    for (vi, (tx, ty, tz)) in &target_indices {
        let v = &verts[*vi];
        println!("  vertex {}: pos=({:.4}, {:.4}, {:.4}) target=({:.2},{:.2},{:.2})",
            vi, v.x, v.y, v.z, tx, ty, tz);
    }

    // For each target vertex, find all triangles using it
    for (vi, target) in &target_indices {
        println!("\n=== Vertex {} (target=({:.2},{:.2},{:.2})) — triangles using it ===", vi, target.0, target.1, target.2);
        let mut count_per_face: HashMap<u64, usize> = HashMap::new();
        for (ti, tri) in tris.iter().enumerate() {
            if tri.iter().any(|&idx| idx as usize == *vi) {
                let fid = face_ids.and_then(|ids| ids.get(ti).copied()).unwrap_or(0);
                *count_per_face.entry(fid).or_insert(0) += 1;
                let other: Vec<u32> = tri.iter().filter(|&&idx| idx as usize != *vi).copied().collect();
                let p0 = verts[other[0] as usize];
                let p1 = verts[other[1] as usize];
                println!("  tri {} face={}: other verts=({}, {}) at ({:.2},{:.2},{:.2}) and ({:.2},{:.2},{:.2})",
                    ti, fid, other[0], other[1],
                    p0.x, p0.y, p0.z, p1.x, p1.y, p1.z);
            }
        }
        println!("  Summary: {} triangles, by face: {:?}", count_per_face.values().sum::<usize>(), count_per_face);
    }
}
