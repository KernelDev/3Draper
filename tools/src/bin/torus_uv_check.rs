//! Check that face #803 UVs are computed correctly.
//! For each 3D boundary point, compute the expected (u,v) via project_point
//! and compare with the stored UV in face.outer_uv_boundary.

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let path = std::env::args().nth(1).unwrap_or("test/drill_top.stp".to_string());
    let face_idx: usize = std::env::args().nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(4); // 1-based face index → STEP #803

    let data = std::fs::read_to_string(&path).expect("read");
    let step = parse_step(&data).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);

    let p = &pending[0];
    let ctx = StepConversionContext::new(&step);
    let inst = match ctx.triangulate_pending(p) {
        Some(i) => i,
        None => { eprintln!("Triangulation failed"); return; }
    };

    if face_idx < 1 || face_idx > inst.faces.len() {
        eprintln!("Face index {} out of range", face_idx);
        return;
    }

    let face = &inst.faces[face_idx - 1];
    println!("Face {} (STEP #{}, surf={}), forward={}",
        face_idx, face.step_face_id, face.surface_type, face.forward);

    // Print surface info
    let surface_ref = &face.surface;
    if let draper_geometry::Surface::Torus(torus) = surface_ref {
        println!("Torus: center=({:.4},{:.4},{:.4}) axis=({:.4},{:.4},{:.4}) x_dir=({:.4},{:.4},{:.4}) R={:.4} r={:.4}",
            torus.center.x, torus.center.y, torus.center.z,
            torus.axis.x, torus.axis.y, torus.axis.z,
            torus.x_dir.x, torus.x_dir.y, torus.x_dir.z,
            torus.major_radius, torus.minor_radius);
        let y_dir = torus.axis.cross(&torus.x_dir);
        println!("y_dir (axis × x_dir) = ({:.4},{:.4},{:.4})", y_dir.x, y_dir.y, y_dir.z);
    }

    // For each 3D boundary point and stored UV, compare with re-projected UV
    for (li, loop_uv) in face.outer_uv_boundary.iter().enumerate() {
        let loop_3d = face.outer_boundary.get(li).cloned().unwrap_or_default();
        println!("\nouter loop {}: {} UV pts, {} 3D pts", li, loop_uv.len(), loop_3d.len());

        let n = loop_uv.len().min(loop_3d.len());
        let mut mismatches = 0;
        for pi in 0..n {
            let uv_stored = loop_uv[pi];
            let p3d = loop_3d[pi];
            let (u_proj, v_proj) = face.surface.project_point(&p3d);
            let du = (uv_stored.u - u_proj).abs();
            let dv = (uv_stored.v - v_proj).abs();
            let is_mismatch = du > 1e-3 || dv > 1e-3;
            if is_mismatch { mismatches += 1; }
            // Print every 5th + first/last + mismatches
            if pi % 5 == 0 || pi == n - 1 || is_mismatch {
                let flag = if is_mismatch { " *** MISMATCH ***" } else { "" };
                println!("  p{}: 3d=({:.4},{:.4},{:.4}) stored=({:.4},{:.4}) proj=({:.4},{:.4}) du={:.4} dv={:.4}{}",
                    pi, p3d.x, p3d.y, p3d.z,
                    uv_stored.u, uv_stored.v, u_proj, v_proj, du, dv, flag);
            }
        }
        println!("Total mismatches: {}/{}", mismatches, n);
    }
}
