// Show full boundary loop for each face of a BREP.
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

    // Only print face 5 and 6 (or whatever user wants)
    let target_faces: Vec<usize> = std::env::args().nth(2)
        .map(|s| s.split(',').filter_map(|n| n.trim().parse::<usize>().ok()).collect())
        .unwrap_or_default();

    for (fi, face) in inst.faces.iter().enumerate() {
        let tris = face.triangle_range.1 - face.triangle_range.0;
        if !target_faces.is_empty() && !target_faces.contains(&(fi + 1)) {
            continue;
        }
        println!("\n=== Face {} (STEP #{}, surf={}) === tris={}, forward={}",
            fi + 1, face.step_face_id, face.surface_type, tris, face.forward);

        for (li, loop_pts) in face.outer_boundary.iter().enumerate() {
            println!("\n  outer loop {}: {} points", li, loop_pts.len());
            // Find z range and corner points
            let (min_z, max_z) = loop_pts.iter().fold((f64::MAX, f64::MIN), |(mn, mx), p| {
                (mn.min(p.z), mx.max(p.z))
            });
            println!("    z range: [{:.4}, {:.4}]", min_z, max_z);

            // Print points where z changes (likely corners)
            let mut prev_z = loop_pts[0].z;
            for (pi, pt) in loop_pts.iter().enumerate() {
                if (pt.z - prev_z).abs() > 0.001 || pi == 0 || pi == loop_pts.len() - 1 {
                    println!("    p{} = ({:.4}, {:.4}, {:.4})  uv=({:.4}, {:.4})",
                        pi, pt.x, pt.y, pt.z,
                        face.outer_uv_boundary[li][pi].u, face.outer_uv_boundary[li][pi].v);
                    prev_z = pt.z;
                }
            }
        }
    }
}
