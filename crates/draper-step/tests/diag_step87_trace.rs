use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

/// Deep diagnostic: for the Step#87 plane face in 3.05.078.stp,
/// trace EXACTLY what happens from boundary points → earcutr → triangle winding
#[test]
fn diag_step87_plane_tracing() {
    let content = std::fs::read_to_string("/home/z/my-project/test/3.05.078.stp")
        .expect("Failed to read 3.05.078.stp");
    
    let step = parse_step(&content).expect("Failed to parse STEP file");
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);
    
    for p in &pending {
        if let Some(inst) = ctx.triangulate_pending(p) {
            for fi in &inst.faces {
                if fi.step_face_id != 87 { continue; }
                
                let (start, end) = fi.triangle_range;
                eprintln!("\n=== Face Step#87 (Plane, forward={}) ===", fi.forward);
                eprintln!("  triangle_range: [{}, {})", start, end);
                eprintln!("  vertices in mesh: {}", inst.mesh.vertices.len());
                eprintln!("  triangles in mesh: {}", inst.mesh.triangles.len());
                
                // Check analytical face normals
                if let Some(ref face_normals) = inst.mesh.face_normals {
                    let n = face_normals[start];
                    eprintln!("  Analytical face normal: ({:.6}, {:.6}, {:.6})", n[0], n[1], n[2]);
                }
                
                // Check if mesh has per-vertex normals (smooth shading)
                if let Some(ref vertex_normals) = inst.mesh.normals {
                    eprintln!("  Has per-vertex normals: {} (smooth shading path)", vertex_normals.len());
                    // Print first few vertex normals
                    for i in start..end.min(start + 2) {
                        let tri = inst.mesh.triangles[i];
                        for &vi in &tri {
                            let v = inst.mesh.vertices[vi as usize];
                            let vn = vertex_normals.get(vi as usize);
                            eprintln!("    vi={}: pos=({:.2},{:.2},{:.2}) normal={:?}",
                                vi, v.x, v.y, v.z,
                                vn.map(|n| format!("({:.4},{:.4},{:.4})", n[0], n[1], n[2])));
                        }
                    }
                } else {
                    eprintln!("  No per-vertex normals (flat shading path)");
                }
                
                // Check cross-product for first 10 triangles
                eprintln!("\n  Cross-product analysis:");
                let mut all_match = true;
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
                    
                    if let Some(ref face_normals) = inst.mesh.face_normals {
                        let an = face_normals[i];
                        let dot = nx * an[0] + ny * an[1] + nz * an[2];
                        if dot < 0.0 {
                            all_match = false;
                            if i < start + 5 {
                                eprintln!("    tri[{}]: cross=({:.4},{:.4},{:.4}) analytical=({:.4},{:.4},{:.4}) OPPOSITE (dot={:.4})",
                                    i, nx, ny, nz, an[0], an[1], an[2], dot);
                            }
                        }
                    }
                }
                
                eprintln!("\n  All cross-product normals match analytical: {}", all_match);
                if !all_match {
                    eprintln!("  *** ROOT CAUSE: Triangle winding does not match analytical normal direction!");
                    eprintln!("  This means the earcutr output + forward:winding swap produces CW triangles");
                    eprintln!("  when they should be CCW (or vice versa).");
                }
            }
        }
    }
}

/// Check normal consistency for a simple cube (should be perfect)
#[test]
fn diag_cube_normal_consistency() {
    let content = std::fs::read_to_string("/home/z/my-project/test/nist_cube.stp")
        .expect("Failed to read nist_cube.stp");
    
    let step = parse_step(&content).expect("Failed to parse STEP file");
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);
    
    for p in &pending {
        if let Some(inst) = ctx.triangulate_pending(p) {
            eprintln!("\n=== CUBE: {} vertices, {} triangles ===", 
                inst.mesh.vertices.len(), inst.mesh.triangles.len());
            
            let face_normals = inst.mesh.face_normals.as_ref();
            let vertex_normals = inst.mesh.normals.as_ref();
            
            let mut total_match = 0usize;
            let mut total_mismatch = 0usize;
            
            for fi in &inst.faces {
                let (start, end) = fi.triangle_range;
                if start >= end { continue; }
                
                let Some(an) = face_normals.and_then(|n| n.get(start)) else { continue; };
                
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
                    eprintln!("  Face Step#{} {} forward={}: match={}, mismatch={}",
                        fi.step_face_id, fi.surface_type, fi.forward, match_count, mismatch_count);
                }
            }
            
            eprintln!("\n  Total: match={}, mismatch={}", total_match, total_mismatch);
        }
    }
}
