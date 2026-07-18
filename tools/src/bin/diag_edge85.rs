// Directly test: call discretize_step_edge for step_id=85 twice and compare
use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_topology::Edge as TopoEdge;
use draper_geometry::Curve3d;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let content = std::fs::read_to_string("/home/z/my-project/3Draper/test/as1-oc-214_bolt.stp")
        .expect("read");
    let step = parse_step(&content).expect("parse");

    // Get the StepConverter to access resolve_edge_curve
    let ctx = StepConversionContext::new(&step);

    // Resolve edge #85 (B_SPLINE) and check its param_range
    // We need to use the internal API — but it's private.
    // Instead, let's triangulate and check boundary points.

    let (_tree, pending) = step_structure_lazy(&step);
    let inst = ctx.triangulate_pending(&pending[0]).unwrap();

    // Get face boundaries for Plane (Step#96) and NURBS (Step#192)
    let f96 = inst.faces.iter().find(|f| f.step_face_id == 96).unwrap();
    let f192 = inst.faces.iter().find(|f| f.step_face_id == 192).unwrap();

    println!("Face #96 (Plane) outer_boundary[0] first 5:");
    for (i, p) in f96.outer_boundary[0].iter().take(5).enumerate() {
        println!("  [{}]: ({:.10}, {:.10}, {:.10})", i, p.x, p.y, p.z);
    }

    println!("\nFace #192 (NURBS) outer_boundary[0] first 5:");
    for (i, p) in f192.outer_boundary[0].iter().take(5).enumerate() {
        println!("  [{}]: ({:.10}, {:.10}, {:.10})", i, p.x, p.y, p.z);
    }

    // Check which edges each face uses
    // Face #96 uses edges #85 and #92 (from FACE_DIAG log)
    // Face #192 uses edges #177, #182, #85, #188
    // Both use edge #85

    // The boundary points for edge #85 should be the SAME in both faces.
    // But the FaceInfo.outer_boundary is built by sample_edges_to_polylines
    // which uses its OWN sampling (not the edge cache). Let's check the
    // actual mesh vertices instead.

    // Get the actual mesh vertices for each face
    let fids = inst.mesh.triangle_face_ids.as_ref().unwrap();
    let (s96, e96) = f96.triangle_range;
    let (s192, e192) = f192.triangle_range;

    let mut v96: std::collections::HashSet<u32> = std::collections::HashSet::new();
    for i in s96..e96 {
        let tri = inst.mesh.triangles[i];
        for &vi in &tri { v96.insert(vi); }
    }
    let mut v192: std::collections::HashSet<u32> = std::collections::HashSet::new();
    for i in s192..e192 {
        let tri = inst.mesh.triangles[i];
        for &vi in &tri { v192.insert(vi); }
    }

    // Find shared vertices (same index)
    let shared: Vec<u32> = v96.intersection(&v192).cloned().collect();
    println!("\nShared vertex indices: {}", shared.len());
    for &vi in shared.iter().take(5) {
        let v = inst.mesh.vertices[vi as usize];
        println!("  vi={}: ({:.10}, {:.10}, {:.10})", vi, v.x, v.y, v.z);
    }

    // Find close vertices (different index, close 3D position)
    println!("\nClose but different vertex indices:");
    let mut found = 0;
    for &vi1 in v96.iter() {
        let v1 = inst.mesh.vertices[vi1 as usize];
        for &vi2 in v192.iter() {
            if vi1 == vi2 { continue; }
            let v2 = inst.mesh.vertices[vi2 as usize];
            let d = ((v1.x-v2.x).powi(2) + (v1.y-v2.y).powi(2) + (v1.z-v2.z).powi(2)).sqrt();
            if d < 0.5 {
                println!("  v96[{}] ({:.6},{:.6},{:.6}) → v192[{}] ({:.6},{:.6},{:.6}) dist={:.6}",
                    vi1, v1.x, v1.y, v1.z, vi2, v2.x, v2.y, v2.z, d);
                found += 1;
                if found >= 10 { break; }
            }
        }
        if found >= 10 { break; }
    }
}
