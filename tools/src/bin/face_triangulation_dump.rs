//! Dump actual triangulation (vertices + triangles in UV) for a specific face.
//! Usage: face_triangulation_dump <file.stp> <face_index_1based>

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_geometry::{Point3d, Point2d};

fn main() {
    let path = std::env::args().nth(1).unwrap_or("test/drill_top.stp".to_string());
    let face_idx: usize = std::env::args().nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(4);

    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

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

    if face_idx < 1 || face_idx > inst.faces.len() {
        eprintln!("Face index {} out of range (1..={})", face_idx, inst.faces.len());
        return;
    }

    let face = &inst.faces[face_idx - 1];
    let tris = face.triangle_range.1 - face.triangle_range.0;
    println!("\n=== Face {} (STEP #{}, surf={}) === tris={}, forward={}",
        face_idx, face.step_face_id, face.surface_type, tris, face.forward);

    // Print boundary UVs
    for (li, loop_uv) in face.outer_uv_boundary.iter().enumerate() {
        println!("\nouter UV loop {}: {} points", li, loop_uv.len());
        // Print all UV points with index
        let loop_3d = face.outer_boundary.get(li).cloned().unwrap_or_default();
        for (pi, uv) in loop_uv.iter().enumerate() {
            // Print only every 5th point + first/last to keep output manageable
            if pi % 5 == 0 || pi == loop_uv.len() - 1 {
                let p3d_str = if pi < loop_3d.len() {
                    let p3d = loop_3d[pi];
                    format!("({:.4}, {:.4}, {:.4})", p3d.x, p3d.y, p3d.z)
                } else {
                    "(no 3d)".to_string()
                };
                println!("  p{}: uv=({:.4}, {:.4})  3d={}",
                    pi, uv.u, uv.v, p3d_str);
            }
        }
    }

    // Print hole UVs
    for (hi, hole_loops) in face.inner_uv_boundaries.iter().enumerate() {
        for (li, hole_uv) in hole_loops.iter().enumerate() {
            println!("\nhole {} UV loop {}: {} points", hi, li, hole_uv.len());
            for (pi, uv) in hole_uv.iter().enumerate() {
                if pi % 5 == 0 || pi == hole_uv.len() - 1 {
                    println!("  hp{}: uv=({:.4}, {:.4})", pi, uv.u, uv.v);
                }
            }
        }
    }

    // Print UV triangles
    println!("\nUV triangles ({} total):", face.uv_triangles.len());
    let mut printed = 0;
    for (ti, tri) in face.uv_triangles.iter().enumerate() {
        if printed < 30 || ti < 5 {
            println!("  t{}: ({:.4},{:.4}) ({:.4},{:.4}) ({:.4},{:.4})",
                ti, tri[0].u, tri[0].v, tri[1].u, tri[1].v, tri[2].u, tri[2].v);
            printed += 1;
        }
        if printed >= 30 && ti >= 5 && ti < face.uv_triangles.len() - 5 {
            if printed == 30 {
                println!("  ... (showing first 30 and last 5) ...");
                printed += 1;
            }
            continue;
        }
    }

    // Compute UV bbox of triangles
    let (u_min, u_max, v_min, v_max) = face.uv_triangles.iter().flat_map(|t| t.iter()).fold(
        (f64::MAX, f64::MIN, f64::MAX, f64::MIN),
        |(umn, umx, vmn, vmx), p| {
            (umn.min(p.u), umx.max(p.u), vmn.min(p.v), vmx.max(p.v))
        }
    );
    println!("\nUV bbox of triangles: u=[{:.4}, {:.4}] v=[{:.4}, {:.4}]",
        u_min, u_max, v_min, v_max);

    // Detect wrap: are there UV values close to 0 AND close to 2π?
    let near_zero = face.uv_triangles.iter().flat_map(|t| t.iter())
        .filter(|p| p.u.abs() < 0.1 || p.v.abs() < 0.1).count();
    let near_2pi = face.uv_triangles.iter().flat_map(|t| t.iter())
        .filter(|p| (p.u - 2.0 * std::f64::consts::PI).abs() < 0.1 ||
                     (p.v - 2.0 * std::f64::consts::PI).abs() < 0.1).count();
    println!("UV near 0: {} verts, near 2π: {} verts", near_zero, near_2pi);
}
