//! Dump the UV coordinates and triangulation for a specific face of a STEP file.
//!
//! Usage: torus_earcutr_dump [path] [face_index]
//!
//! This tool loads a STEP file, triangulates it, and prints detailed
//! boundary/triangulation information for a specific face — useful for
//! debugging artifacts like "bridge" triangles on periodic surfaces.

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let path = std::env::args().nth(1).unwrap_or("test/drill_top.stp".to_string());
    let face_idx: usize = std::env::args().nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(4);

    let data = std::fs::read_to_string(&path).expect("read");
    let step = parse_step(&data).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);

    let p = &pending[0];

    // Use the public API from StepConversionContext
    let ctx = StepConversionContext::new(&step);
    let inst = match ctx.triangulate_pending(p) {
        Some(i) => i,
        None => { eprintln!("Triangulation failed"); return; }
    };

    if face_idx < 1 || face_idx > inst.faces.len() {
        eprintln!("Face index {} out of range (1..{})", face_idx, inst.faces.len());
        return;
    }

    let face = &inst.faces[face_idx - 1];
    println!("Face {} (STEP #{}, surf={}), forward={}",
        face_idx, face.step_face_id, face.surface_type, face.forward);

    // Print surface details
    match &face.surface {
        draper_geometry::Surface::Torus(t) => {
            println!("  Torus: major_radius={:.4}, minor_radius={:.4}", t.major_radius, t.minor_radius);
        }
        draper_geometry::Surface::Cylinder(c) => {
            println!("  Cylinder: radius={:.4}", c.radius);
        }
        draper_geometry::Surface::Sphere(s) => {
            println!("  Sphere: radius={:.4}", s.radius);
        }
        draper_geometry::Surface::Cone(c) => {
            println!("  Cone: radius={:.4}, half_angle={:.4}", c.radius, c.half_angle);
        }
        draper_geometry::Surface::Plane(_) => {
            println!("  Plane");
        }
        draper_geometry::Surface::Revolution(r) => {
            println!("  Revolution (axis-based)");
            let _ = r;
        }
        draper_geometry::Surface::Extrusion(e) => {
            println!("  Extrusion");
            let _ = e;
        }
        draper_geometry::Surface::Nurbs(n) => {
            println!("  NURBS: u_degree={}, v_degree={}", n.u_degree, n.v_degree);
        }
        draper_geometry::Surface::Offset(_) => {
            println!("  Offset surface");
        }
        draper_geometry::Surface::Ruled(_) => {
            println!("  Ruled surface");
        }
    }

    // Print boundary info
    println!("\nOuter boundary: {} polylines", face.outer_boundary.len());
    for (i, polyline) in face.outer_boundary.iter().enumerate() {
        println!("  polyline {}: {} points", i, polyline.len());
    }
    println!("Inner boundaries (holes): {}", face.inner_boundaries.len());
    for (i, hole) in face.inner_boundaries.iter().enumerate() {
        println!("  hole {}: {} polylines", i, hole.len());
    }

    // Print UV boundary info
    println!("\nOuter UV boundary: {} polylines", face.outer_uv_boundary.len());
    for (i, polyline) in face.outer_uv_boundary.iter().enumerate() {
        if !polyline.is_empty() {
            let u_range = polyline.iter().map(|p| p.u)
                .fold((f64::MAX, f64::MIN), |(min, max), u| (min.min(u), max.max(u)));
            let v_range = polyline.iter().map(|p| p.v)
                .fold((f64::MAX, f64::MIN), |(min, max), v| (min.min(v), max.max(v)));
            println!("  uv_polyline {}: {} points, u=[{:.4},{:.4}], v=[{:.4},{:.4}]",
                i, polyline.len(), u_range.0, u_range.1, v_range.0, v_range.1);
        }
    }

    // Print triangle range and check for bridge triangles
    let (tri_start, tri_end) = face.triangle_range;
    let mesh = &inst.mesh;
    let num_face_tris = tri_end.saturating_sub(tri_start);
    println!("\nFace {} triangles: range [{}, {}) = {} triangles",
        face_idx, tri_start, tri_end, num_face_tris);
    println!("Mesh total: {} vertices, {} triangles",
        mesh.vertices.len(), mesh.triangles.len());

    // Check for bridge triangles (large index span between vertices)
    let mut bridge_count = 0usize;
    for ti in tri_start..tri_end {
        if ti >= mesh.triangles.len() {
            break;
        }
        let [i0, i1, i2] = mesh.triangles[ti];
        let idx_span = (i0 as i32 - i1 as i32).abs()
            .max((i1 as i32 - i2 as i32).abs())
            .max((i0 as i32 - i2 as i32).abs());

        if idx_span > 20 {
            if bridge_count < 10 {
                let _v0 = &mesh.vertices[i0 as usize];
                let _v1 = &mesh.vertices[i1 as usize];
                let _v2 = &mesh.vertices[i2 as usize];
                println!("  BRIDGE t{}: idx=({},{},{}) span={}",
                    ti, i0, i1, i2, idx_span);
            }
            bridge_count += 1;
        }
    }
    println!("\nTotal bridge triangles: {} (out of {})", bridge_count, num_face_tris);

    // Print UV triangles
    if !face.uv_triangles.is_empty() {
        println!("\nUV triangles: {}", face.uv_triangles.len());
        for (i, tri) in face.uv_triangles.iter().enumerate().take(20) {
            println!("  uv_tri[{}]: ({:.4},{:.4}) ({:.4},{:.4}) ({:.4},{:.4})",
                i, tri[0].u, tri[0].v, tri[1].u, tri[1].v, tri[2].u, tri[2].v);
        }
        if face.uv_triangles.len() > 20 {
            println!("  ... and {} more", face.uv_triangles.len() - 20);
        }
    }
}
