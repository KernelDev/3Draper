//! Dump all triangles and vertices for a STEP file's first BREP.

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let path = std::env::args().nth(1).unwrap_or("test/nist_cube.stp".to_string());
    let data = std::fs::read_to_string(&path).expect("read");
    let step = parse_step(&data).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);

    let p = &pending[0];
    let ctx = StepConversionContext::new(&step);
    let inst = ctx.triangulate_pending(p).expect("triangulate");

    println!("=== {} (BREP #{}) ===", inst.name, p.brep_id);
    println!("Vertices: {} (8 for a cube)", inst.mesh.vertex_count());
    println!();

    for (i, v) in inst.mesh.vertices.iter().enumerate() {
        println!("  v{} = ({:.4}, {:.4}, {:.4})", i, v.x, v.y, v.z);
    }
    println!();

    println!("Triangles: {} (should be 12 for a cube: 2 per face × 6 faces)", inst.mesh.triangles.len());
    let face_ids = inst.mesh.triangle_face_ids.as_ref();
    for (i, tri) in inst.mesh.triangles.iter().enumerate() {
        let fid = face_ids.map(|ids| ids[i]).unwrap_or(0);
        let v0 = inst.mesh.vertices[tri[0] as usize];
        let v1 = inst.mesh.vertices[tri[1] as usize];
        let v2 = inst.mesh.vertices[tri[2] as usize];
        println!("  tri {}: face={} indices=({},{},{}) verts=({:.1},{:.1},{:.1}) ({:.1},{:.1},{:.1}) ({:.1},{:.1},{:.1})",
            i, fid, tri[0], tri[1], tri[2],
            v0.x, v0.y, v0.z, v1.x, v1.y, v1.z, v2.x, v2.y, v2.z);
    }

    // Count edge occurrences
    use std::collections::HashMap;
    let mut edge_count: HashMap<(u32, u32, u64), usize> = HashMap::new();
    for (i, tri) in inst.mesh.triangles.iter().enumerate() {
        let fid = face_ids.map(|ids| ids[i]).unwrap_or(0);
        let edges = [
            (tri[0].min(tri[1]), tri[0].max(tri[1])),
            (tri[1].min(tri[2]), tri[1].max(tri[2])),
            (tri[2].min(tri[0]), tri[2].max(tri[0])),
        ];
        for (a, b) in edges {
            *edge_count.entry((a, b, fid)).or_insert(0) += 1;
        }
    }
    // Now count by (a,b) only across faces
    let mut edge_faces: HashMap<(u32, u32), std::collections::HashSet<u64>> = HashMap::new();
    for (i, tri) in inst.mesh.triangles.iter().enumerate() {
        let fid = face_ids.map(|ids| ids[i]).unwrap_or(0);
        let edges = [
            (tri[0].min(tri[1]), tri[0].max(tri[1])),
            (tri[1].min(tri[2]), tri[1].max(tri[2])),
            (tri[2].min(tri[0]), tri[2].max(tri[0])),
        ];
        for (a, b) in edges {
            edge_faces.entry((a, b)).or_default().insert(fid);
        }
    }
    println!();
    println!("=== Edges shared across faces ===");
    let mut multi_face_edges: Vec<_> = edge_faces.iter()
        .filter(|(_, faces)| faces.len() > 1)
        .collect();
    multi_face_edges.sort_by_key(|(_, faces)| std::cmp::Reverse(faces.len()));
    for ((a, b), faces) in multi_face_edges.iter().take(20) {
        let pa = inst.mesh.vertices[*a as usize];
        let pb = inst.mesh.vertices[*b as usize];
        println!("  edge ({}, {}): {} faces {:?} — ({:.1},{:.1},{:.1})↔({:.1},{:.1},{:.1})",
            a, b, faces.len(), faces.iter().collect::<Vec<_>>(),
            pa.x, pa.y, pa.z, pb.x, pb.y, pb.z);
    }
}
