//! Dump the actual UVs passed to earcutr for face #803.
//! This calls the same code path as triangulation to get the paired 3D/UV arrays.

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_mesh::triangulate::{collect_face_boundary_with_uv_from_cache, EdgeDiscretizationCache};
use draper_mesh::triangulate::TriangulationParams;
use draper_geometry::Surface;

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
    // Build solid and edge cache the same way triangulation does
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

    // We need to rebuild the solid+cache to call collect_face_boundary_with_uv_from_cache
    // Use the public API from draper_mesh
    let solid = draper_step::converter::face_data_list_to_solid(&p.faces);
    let mut cache = EdgeDiscretizationCache::new();
    cache.pre_populate_for_solid(&solid, 21);

    // Get the face from solid (it should match face_idx - 1)
    let solid_faces = solid.faces();
    if face_idx - 1 >= solid_faces.len() {
        eprintln!("Solid face index out of range");
        return;
    }
    let solid_face = &solid_faces[face_idx - 1];

    let surface = face.surface.clone();
    let (boundary_3d, boundary_uvs) = collect_face_boundary_with_uv_from_cache(
        solid_face, &cache, &surface);

    println!("\nBoundary: {} points", boundary_3d.len());
    println!("\nAll UVs:");
    for (i, uv) in boundary_uvs.iter().enumerate() {
        let p3d = boundary_3d[i];
        // Mark corners and transitions
        let mark = if i > 0 {
            let prev = boundary_uvs[i-1];
            let du = (uv.u - prev.u).abs();
            let dv = (uv.v - prev.v).abs();
            if du > 0.1 || dv > 0.1 { " <<JUMP>>" } else { "" }
        } else { "" };
        if i < 10 || i > boundary_uvs.len() - 10 || i % 10 == 0 || !mark.is_empty() {
            println!("  [{}]: uv=({:.4},{:.4}) 3d=({:.4},{:.4},{:.4}){}",
                i, uv.u, uv.v, p3d.x, p3d.y, p3d.z, mark);
        }
    }

    // Run earcutr directly to see what triangles it produces
    use earcutr;
    let mut flat_uvs: Vec<f64> = Vec::with_capacity(boundary_uvs.len() * 2);
    for uv in &boundary_uvs {
        flat_uvs.push(uv.u);
        flat_uvs.push(uv.v);
    }
    let hole_indices: Vec<u32> = vec![];
    let tris = earcutr::earcut(&flat_uvs, &hole_indices, 2);
    println!("\nearcutr produced {} triangles (= {} indices)",
        tris.len() / 3, tris.len());

    // Check for "bridge" triangles — those with vertices far apart in index
    let mut bridge_count = 0;
    for (ti, chunk) in tris.chunks(3).enumerate() {
        let i0 = chunk[0] as usize;
        let i1 = chunk[1] as usize;
        let i2 = chunk[2] as usize;
        let idx_span = (i0 as i32 - i1 as i32).abs().max((i1 as i32 - i2 as i32).abs()).max((i0 as i32 - i2 as i32).abs());
        if idx_span > 20 {
            if bridge_count < 10 {
                let u0 = boundary_uvs[i0];
                let u1 = boundary_uvs[i1];
                let u2 = boundary_uvs[i2];
                println!("  BRIDGE t{}: idx=({},{},{}) span={} uv=({:.3},{:.3})({:.3},{:.3})({:.3},{:.3})",
                    ti, i0, i1, i2, idx_span, u0.u, u0.v, u1.u, u1.v, u2.u, u2.v);
            }
            bridge_count += 1;
        }
    }
    println!("\nTotal bridge triangles: {} (out of {})", bridge_count, tris.len() / 3);
}
