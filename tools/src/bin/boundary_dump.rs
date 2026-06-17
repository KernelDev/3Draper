//! Dump boundary points for each face of a BREP.

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let path = std::env::args().nth(1).unwrap_or("test/nist_chamfer_block.stp".to_string());
    let data = std::fs::read_to_string(&path).expect("read");
    let step = parse_step(&data).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);

    let p = &pending[0];
    // Use the conversion context to triangulate, then inspect the result
    let ctx = StepConversionContext::new(&step);
    let inst = match ctx.triangulate_pending(p) {
        Some(i) => i,
        None => { eprintln!("Triangulation failed"); return; }
    };

    println!("BREP #{} has {} faces, {} verts, {} tris",
        p.brep_id, inst.faces.len(), inst.mesh.vertex_count(), inst.mesh.triangle_count());

    let face_ids = inst.mesh.triangle_face_ids.as_ref();
    for (fi, face) in inst.faces.iter().enumerate().take(7) {
        let tris = face.triangle_range.1 - face.triangle_range.0;
        println!("\n=== Face {} (STEP #{}, surf={}) === tris={}, forward={}",
            fi + 1, face.step_face_id, face.surface_type, tris, face.forward);

        // Print outer boundary 3D points
        for (li, loop_pts) in face.outer_boundary.iter().enumerate() {
            println!("  outer loop {}: {} points", li, loop_pts.len());
            for (pi, pt) in loop_pts.iter().enumerate().take(10) {
                println!("    p{} = ({:.4}, {:.4}, {:.4})", pi, pt.x, pt.y, pt.z);
            }
            if loop_pts.len() > 10 {
                println!("    ... ({} more)", loop_pts.len() - 10);
            }
        }

        // Print UV boundary
        for (li, loop_uvs) in face.outer_uv_boundary.iter().enumerate() {
            println!("  outer UV loop {}: {} points", li, loop_uvs.len());
            for (pi, uv) in loop_uvs.iter().enumerate().take(5) {
                println!("    uv{} = ({:.4}, {:.4})", pi, uv.u, uv.v);
            }
        }
    }
}
