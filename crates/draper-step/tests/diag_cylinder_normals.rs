use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

/// Resolve a test-data file relative to the workspace `test/` dir.
/// Robust to repo relocation: derived from CARGO_MANIFEST_DIR
/// (crates/draper-step -> workspace root), not from a hardcoded sandbox path.
fn test_file(name: &str) -> std::path::PathBuf {
    let mut dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    dir.pop(); // crates/draper-step -> crates
    dir.pop(); // crates -> workspace root
    dir.join("test").join(name)
}


/// Check normal consistency for brick_thin.stp - focus on cylinder faces
#[test]
fn diag_brick_thin_cylinders() {
    let content = std::fs::read_to_string(test_file("brick_thin.stp"))
        .expect("Failed to read");
    let step = parse_step(&content).expect("parse");
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);
    
    for p in &pending {
        if let Some(inst) = ctx.triangulate_pending(p) {
            eprintln!("BREP: {} verts, {} tris", inst.mesh.vertices.len(), inst.mesh.triangles.len());
            
            let mut total_match = 0usize;
            let mut total_mismatch = 0usize;
            
            for fi in &inst.faces {
                let (start, end) = fi.triangle_range;
                if start >= end { continue; }
                
                let face_normal = inst.mesh.face_normals.as_ref().and_then(|n| n.get(start)).copied();
                let Some(an) = face_normal else { continue; };
                
                let mut match_count = 0usize;
                let mut mismatch_count = 0usize;
                
                for i in start..end {
                    let tri = inst.mesh.triangles[i];
                    let v0 = inst.mesh.vertices[tri[0] as usize];
                    let v1 = inst.mesh.vertices[tri[1] as usize];
                    let v2 = inst.mesh.vertices[tri[2] as usize];
                    
                    let e1 = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
                    let e2 = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
                    let nx = e1.1 * e2.2 - e1.2 * e2.1;
                    let ny = e1.2 * e2.0 - e1.0 * e2.2;
                    let nz = e1.0 * e2.1 - e1.1 * e2.0;
                    
                    let dot = nx * an[0] + ny * an[1] + nz * an[2];
                    if dot > 0.0 { match_count += 1; }
                    else if dot < 0.0 { mismatch_count += 1; }
                }
                
                total_match += match_count;
                total_mismatch += mismatch_count;
                
                if mismatch_count > 0 {
                    // Detailed analysis for mismatched faces
                    let has_vertex_normals = inst.mesh.normals.is_some();
                    eprintln!("\n  MISMATCH Face Step#{} {} forward={} tris={}: match={} mismatch={} vertex_normals={}",
                        fi.step_face_id, fi.surface_type, fi.forward, end-start, match_count, mismatch_count, has_vertex_normals);
                    
                    // Print first 3 triangles with details
                    for i in start..end.min(start + 3) {
                        let tri = inst.mesh.triangles[i];
                        let v0 = inst.mesh.vertices[tri[0] as usize];
                        let v1 = inst.mesh.vertices[tri[1] as usize];
                        let v2 = inst.mesh.vertices[tri[2] as usize];
                        
                        let e1 = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
                        let e2 = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
                        let nx = e1.1 * e2.2 - e1.2 * e2.1;
                        let ny = e1.2 * e2.0 - e1.0 * e2.2;
                        let nz = e1.0 * e2.1 - e1.1 * e2.0;
                        
                        // Get vertex normals for these vertices
                        let vn0 = inst.mesh.normals.as_ref().and_then(|ns| ns.get(tri[0] as usize));
                        let vn1 = inst.mesh.normals.as_ref().and_then(|ns| ns.get(tri[1] as usize));
                        let vn2 = inst.mesh.normals.as_ref().and_then(|ns| ns.get(tri[2] as usize));
                        
                        eprintln!("    tri[{}]: idx=({},{},{})", i, tri[0], tri[1], tri[2]);
                        eprintln!("      v0=({:.2},{:.2},{:.2}) vn0={:?}", v0.x, v0.y, v0.z,
                            vn0.map(|n| format!("({:.3},{:.3},{:.3})", n[0], n[1], n[2])));
                        eprintln!("      v1=({:.2},{:.2},{:.2}) vn1={:?}", v1.x, v1.y, v1.z,
                            vn1.map(|n| format!("({:.3},{:.3},{:.3})", n[0], n[1], n[2])));
                        eprintln!("      v2=({:.2},{:.2},{:.2}) vn2={:?}", v2.x, v2.y, v2.z,
                            vn2.map(|n| format!("({:.3},{:.3},{:.3})", n[0], n[1], n[2])));
                        eprintln!("      cross=({:.4},{:.4},{:.4}) face_normal=({:.4},{:.4},{:.4})",
                            nx, ny, nz, an[0], an[1], an[2]);
                    }
                }
            }
            
            eprintln!("\n  Total: match={}, mismatch={}", total_match, total_mismatch);
            eprintln!("  Percentage correct: {:.1}%", 
                total_match as f64 / (total_match + total_mismatch).max(1) as f64 * 100.0);
        }
    }
}
