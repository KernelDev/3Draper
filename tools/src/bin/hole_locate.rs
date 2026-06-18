// SPDX-License-Identifier: GPL-3.0-or-later
// Locate exact boundary edges (holes) in a triangulated BRep.
//
// Usage: cargo run --bin hole_locate [file.stp]

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_mesh::validate_watertight;
use std::collections::HashMap;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let path = std::env::args().nth(1).unwrap_or("test/as1-oc-214_bolt.stp".to_string());
    let data = std::fs::read_to_string(&path).expect("Failed to read STEP file");
    let step = parse_step(&data).expect("Failed to parse STEP file");
    let (_tree, pending) = step_structure_lazy(&step);

    let p = &pending[0];
    let ctx = StepConversionContext::new(&step);
    let inst = match ctx.triangulate_pending(p) {
        Some(i) => i,
        None => { eprintln!("Triangulation failed"); return; }
    };

    println!("BREP #{}: {} verts, {} tris, {} faces",
        p.brep_id, inst.mesh.vertex_count(), inst.mesh.triangle_count(), inst.faces.len());

    let report = validate_watertight(&inst.mesh, true);
    println!("Watertight: {} (boundary={}, non-manifold={}, degen={})",
        report.is_watertight(), report.boundary_edge_count,
        report.non_manifold_edge_count, report.degenerate_triangle_count);

    // Build edge -> triangle count map
    let verts = &inst.mesh.vertices;
    let tris = &inst.mesh.triangles;
    let face_ids = inst.mesh.triangle_face_ids.as_ref();

    let mut edge_count: HashMap<(u32, u32), usize> = HashMap::new();
    let mut edge_face: HashMap<(u32, u32), u64> = HashMap::new();
    for (ti, tri) in tris.iter().enumerate() {
        let a = tri[0] as u32;
        let b = tri[1] as u32;
        let c = tri[2] as u32;
        for (v0, v1) in [(a, b), (b, c), (c, a)] {
            let key = if v0 < v1 { (v0, v1) } else { (v1, v0) };
            *edge_count.entry(key).or_insert(0) += 1;
            let fid = face_ids.and_then(|ids| ids.get(ti).copied()).unwrap_or(0);
            edge_face.insert(key, fid);
        }
    }

    // Find boundary edges (count == 1)
    let mut boundary_edges: Vec<((u32, u32), u64)> = Vec::new();
    for (edge, &count) in &edge_count {
        if count == 1 {
            let fid = edge_face.get(edge).copied().unwrap_or(0);
            boundary_edges.push((*edge, fid));
        }
    }

    println!("\n{} boundary edges:", boundary_edges.len());
    for (i, ((v0, v1), fid)) in boundary_edges.iter().enumerate() {
        let p0 = verts[*v0 as usize];
        let p1 = verts[*v1 as usize];
        let mid_x = (p0.x + p1.x) * 0.5;
        let mid_y = (p0.y + p1.y) * 0.5;
        let mid_z = (p0.z + p1.z) * 0.5;
        let len = ((p0.x - p1.x).powi(2) + (p0.y - p1.y).powi(2) + (p0.z - p1.z).powi(2)).sqrt();
        println!("  {}: face={} edge=({},{}) len={:.4}", i, fid, v0, v1, len);
        println!("     p0=({:.4}, {:.4}, {:.4})", p0.x, p0.y, p0.z);
        println!("     p1=({:.4}, {:.4}, {:.4})", p1.x, p1.y, p1.z);
        println!("     mid=({:.4}, {:.4}, {:.4})", mid_x, mid_y, mid_z);
    }

    // Group boundary edges by face
    let mut by_face: HashMap<u64, Vec<((u32, u32), (f64,f64,f64))>> = HashMap::new();
    for ((v0, v1), fid) in &boundary_edges {
        let p0 = verts[*v0 as usize];
        let p1 = verts[*v1 as usize];
        let mid = ((p0.x+p1.x)*0.5, (p0.y+p1.y)*0.5, (p0.z+p1.z)*0.5);
        by_face.entry(*fid).or_default().push(((*v0, *v1), mid));
    }
    println!("\nBoundary edges by face:");
    for (fid, edges) in by_face.iter() {
        println!("  Face {}: {} edges", fid, edges.len());
        for (i, ((v0, v1), mid)) in edges.iter().enumerate() {
            println!("    {}: ({},{}) mid=({:.4}, {:.4}, {:.4})", i, v0, v1, mid.0, mid.1, mid.2);
        }
    }
}
