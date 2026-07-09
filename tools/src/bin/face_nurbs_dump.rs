// Dump NURBS surface definition and sample points for a specific face.
use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_geometry::Surface;
use draper_geometry::surface::NurbsSurface;

fn point_at_nurbs(nurbs: &NurbsSurface, u: f64, v: f64) -> draper_geometry::Point3d {
    Surface::Nurbs(nurbs.clone()).point_at(u, v)
}

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let path = std::env::args().nth(1).unwrap_or("test/as1-oc-214_bolt.stp".to_string());
    let target_face: usize = std::env::args().nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(5);

    let data = std::fs::read_to_string(&path).expect("read");
    let step = parse_step(&data).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);

    let p = &pending[0];
    let ctx = StepConversionContext::new(&step);
    let inst = match ctx.triangulate_pending(p) {
        Some(i) => i,
        None => { eprintln!("Triangulation failed"); return; }
    };

    if target_face > inst.faces.len() {
        eprintln!("Face {} not found (only {} faces)", target_face, inst.faces.len());
        return;
    }

    let face = &inst.faces[target_face - 1];
    println!("Face {} (STEP #{}): {} tris, surf={:?}",
        target_face, face.step_face_id, face.surface_type, face.surface);

    if let Surface::Nurbs(ref nurbs) = face.surface {
        let (u_min, u_max) = nurbs.u_range();
        let (v_min, v_max) = nurbs.v_range();
        println!("\nNURBS: u_degree={}, v_degree={}, u_range=[{:.4},{:.4}], v_range=[{:.4},{:.4}]",
            nurbs.u_degree, nurbs.v_degree, u_min, u_max, v_min, v_max);
        println!("  u_knots: {:?}", nurbs.u_knots);
        println!("  v_knots: {:?}", nurbs.v_knots);
        println!("  control_points dimensions: {} x {}", nurbs.control_points.len(),
            nurbs.control_points.first().map(|c| c.len()).unwrap_or(0));
        for (i, row) in nurbs.control_points.iter().enumerate() {
            for (j, cp) in row.iter().enumerate() {
                println!("    CP[{}][{}] = ({:.4}, {:.4}, {:.4})", i, j, cp.x, cp.y, cp.z);
            }
        }

        // Sample the surface at the midpoint of each parameter
        println!("\nSurface samples:");
        for ui in 0..=4 {
            for vi in 0..=4 {
                let u = u_min + (u_max - u_min) * (ui as f64) / 4.0;
                let v = v_min + (v_max - v_min) * (vi as f64) / 4.0;
                let p = point_at_nurbs(nurbs, u, v);
                println!("  u={:.2}, v={:.2} → ({:.4}, {:.4}, {:.4})", u, v, p.x, p.y, p.z);
            }
        }

        // Also evaluate at boundary UV points
        if let Some(uv_loop) = face.outer_uv_boundary.first() {
            println!("\nBoundary UV → 3D (first 5 and middle):");
            for i in [0, 1, 2, uv_loop.len()/2, uv_loop.len()-1] {
                let uv = uv_loop[i];
                let p = point_at_nurbs(nurbs, uv.u, uv.v);
                let p3d = &face.outer_boundary[0][i];
                let dist = ((p.x - p3d.x).powi(2) + (p.y - p3d.y).powi(2) + (p.z - p3d.z).powi(2)).sqrt();
                println!("  uv=({:.4},{:.4}) → nurbs=({:.4},{:.4},{:.4}) | boundary=({:.4},{:.4},{:.4}) | dist={:.6}",
                    uv.u, uv.v, p.x, p.y, p.z, p3d.x, p3d.y, p3d.z, dist);
            }
        }
    }
}
