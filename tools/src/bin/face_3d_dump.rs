//! Dump 3D vertices and triangles for a specific face.
//! Usage: face_3d_dump <file.stp> <face_index_1based>

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

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

    println!("BREP #{}: {} faces, {} verts, {} tris",
        p.brep_id, inst.faces.len(), inst.mesh.vertices.len(), inst.mesh.triangles.len());

    if face_idx < 1 || face_idx > inst.faces.len() {
        eprintln!("Face index {} out of range (1..={})", face_idx, inst.faces.len());
        return;
    }

    let face = &inst.faces[face_idx - 1];
    let tri_start = face.triangle_range.0 as usize;
    let tri_end = face.triangle_range.1 as usize;
    let tris = tri_end - tri_start;
    println!("\n=== Face {} (STEP #{}, surf={}) === tris={}, forward={}",
        face_idx, face.step_face_id, face.surface_type, tris, face.forward);

    // Print 3D triangles
    println!("\n3D triangles (first 30):");
    for ti in tri_start..tri_end.min(tri_start + 30) {
        let tri = inst.mesh.triangles[ti];
        let v0 = inst.mesh.vertices[tri[0] as usize];
        let v1 = inst.mesh.vertices[tri[1] as usize];
        let v2 = inst.mesh.vertices[tri[2] as usize];
        println!("  t{}: v{}({:.4},{:.4},{:.4}) v{}({:.4},{:.4},{:.4}) v{}({:.4},{:.4},{:.4})",
            ti - tri_start, tri[0], v0.x, v0.y, v0.z,
            tri[1], v1.x, v1.y, v1.z,
            tri[2], v2.x, v2.y, v2.z);
    }

    // Compute 3D bbox of face's triangles
    let (min_x, max_x, min_y, max_y, min_z, max_z) = (tri_start..tri_end).fold(
        (f64::MAX, f64::MIN, f64::MAX, f64::MIN, f64::MAX, f64::MIN),
        |(mnx, mxx, mny, mxy, mnz, mxz), ti| {
            let tri = inst.mesh.triangles[ti];
            for &vi in &tri {
                let v = inst.mesh.vertices[vi as usize];
                let mnx = mnx.min(v.x); let mxx = mxx.max(v.x);
                let mny = mny.min(v.y); let mxy = mxy.max(v.y);
                let mnz = mnz.min(v.z); let mxz = mxz.max(v.z);
                return (mnx, mxx, mny, mxy, mnz, mxz);
            }
            (mnx, mxx, mny, mxy, mnz, mxz)
        }
    );
    println!("\n3D bbox: x=[{:.4}, {:.4}] y=[{:.4}, {:.4}] z=[{:.4}, {:.4}]",
        min_x, max_x, min_y, max_y, min_z, max_z);

    // Compute triangle areas
    let mut min_area = f64::MAX;
    let mut max_area: f64 = 0.0;
    let mut zero_area_count = 0;
    for ti in tri_start..tri_end {
        let tri = inst.mesh.triangles[ti];
        let v0 = inst.mesh.vertices[tri[0] as usize];
        let v1 = inst.mesh.vertices[tri[1] as usize];
        let v2 = inst.mesh.vertices[tri[2] as usize];
        let area = 0.5 * ((v1.x - v0.x) * (v2.y - v0.y) - (v2.x - v0.x) * (v1.y - v0.y)).abs();
        if area < 1e-12 {
            zero_area_count += 1;
        } else {
            min_area = min_area.min(area);
            max_area = max_area.max(area);
        }
    }
    println!("\nTriangle areas: min={:.6}, max={:.6}, zero_area_count={}",
        min_area, max_area, zero_area_count);

    // Compute edge lengths
    let mut min_edge = f64::MAX;
    let mut max_edge: f64 = 0.0;
    let mut long_edge_count = 0; // edges > 0.5 (suspicious for torus with r=0.015)
    for ti in tri_start..tri_end {
        let tri = inst.mesh.triangles[ti];
        let vs = [
            inst.mesh.vertices[tri[0] as usize],
            inst.mesh.vertices[tri[1] as usize],
            inst.mesh.vertices[tri[2] as usize],
        ];
        for k in 0..3 {
            let a = vs[k]; let b = vs[(k+1)%3];
            let len = ((a.x - b.x).powi(2) + (a.y - b.y).powi(2) + (a.z - b.z).powi(2)).sqrt();
            min_edge = min_edge.min(len);
            max_edge = max_edge.max(len);
            if len > 0.5 { long_edge_count += 1; }
        }
    }
    println!("\nEdge lengths: min={:.6}, max={:.6}, long_edges(>0.5)={}",
        min_edge, max_edge, long_edge_count);

    // Print triangles with long edges (suspicious)
    println!("\nTriangles with long edges (>0.3):");
    let mut count = 0;
    for ti in tri_start..tri_end {
        let tri = inst.mesh.triangles[ti];
        let vs = [
            inst.mesh.vertices[tri[0] as usize],
            inst.mesh.vertices[tri[1] as usize],
            inst.mesh.vertices[tri[2] as usize],
        ];
        let mut max_len = 0.0;
        let mut long_pair = (0u32, 0u32);
        for k in 0..3 {
            let a = vs[k]; let b = vs[(k+1)%3];
            let len = ((a.x - b.x).powi(2) + (a.y - b.y).powi(2) + (a.z - b.z).powi(2)).sqrt();
            if len > max_len {
                max_len = len;
                long_pair = (tri[k], tri[(k+1)%3]);
            }
        }
        if max_len > 0.3 {
            println!("  t{}: v{}({:.4},{:.4},{:.4}) v{}({:.4},{:.4},{:.4}) v{}({:.4},{:.4},{:.4}) max_edge={:.4} (v{}-v{})",
                ti - tri_start,
                tri[0], vs[0].x, vs[0].y, vs[0].z,
                tri[1], vs[1].x, vs[1].y, vs[1].z,
                tri[2], vs[2].x, vs[2].y, vs[2].z,
                max_len, long_pair.0, long_pair.1);
            count += 1;
            if count >= 20 { break; }
        }
    }
    if count == 0 {
        println!("  (none)");
    }
}
