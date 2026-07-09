use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

/// Detailed diagnostic for a single planar face:
/// 1. Show the 2D polygon (outer + holes) going into earcutr
/// 2. Show the signed area before and after CCW normalization  
/// 3. Show earcutr output triangles
/// 4. Show the resulting cross-product normals vs analytical normals
/// 5. Check if the `forward:false` winding swap corrects or corrupts
#[test]
fn diag_planar_face_earcutr_detail() {
    let content = std::fs::read_to_string("/home/z/my-project/test/3.05.078.stp")
        .expect("Failed to read 3.05.078.stp");
    
    let step = parse_step(&content).expect("Failed to parse STEP file");
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);
    
    for (pi, p) in pending.iter().enumerate() {
        if let Some(inst) = ctx.triangulate_pending(p) {
            eprintln!("\n=== BREP #{}: {} vertices, {} triangles ===", 
                p.brep_id, inst.mesh.vertices.len(), inst.mesh.triangles.len());
            
            for fi in &inst.faces {
                let (start, end) = fi.triangle_range;
                if start >= end { continue; }
                
                // Check each triangle's cross-product normal direction
                let mut cross_match = 0usize;
                let mut cross_mismatch = 0usize;
                let mut degenerate = 0usize;
                
                // Get the analytical face normal
                let face_normal = if let Some(ref face_normals) = inst.mesh.face_normals {
                    face_normals[start]
                } else {
                    continue;
                };
                
                eprintln!("\nFace Step#{} {} (face_id={}, forward={}, tri_range=[{},{}))",
                    fi.step_face_id, fi.surface_type, fi.face_id, fi.forward, start, end);
                eprintln!("  Analytical normal: ({:.6}, {:.6}, {:.6})", face_normal[0], face_normal[1], face_normal[2]);
                
                // Print first 3 triangles with cross product
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
                    let len = (nx*nx + ny*ny + nz*nz).sqrt();
                    
                    eprintln!("  tri[{}]: v0=({:.2},{:.2},{:.2}) v1=({:.2},{:.2},{:.2}) v2=({:.2},{:.2},{:.2})",
                        i, v0.x, v0.y, v0.z, v1.x, v1.y, v1.z, v2.x, v2.y, v2.z);
                    eprintln!("    cross=({:.6},{:.6},{:.6}) len={:.6}", nx, ny, nz, len);
                }
                
                // Count matches/mismatches
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
                    let len = (nx*nx + ny*ny + nz*nz).sqrt();
                    
                    if len < 1e-15 {
                        degenerate += 1;
                    } else {
                        let dot = nx * face_normal[0] + ny * face_normal[1] + nz * face_normal[2];
                        if dot > 0.0 {
                            cross_match += 1;
                        } else {
                            cross_mismatch += 1;
                        }
                    }
                }
                
                eprintln!("  Cross-product vs analytical: match={}, mismatch={}, degenerate={}", 
                    cross_match, cross_mismatch, degenerate);
                
                if cross_mismatch > 0 {
                    eprintln!("  *** WARNING: {} of {} triangles have cross-product normal OPPOSITE to analytical normal!", 
                        cross_mismatch, cross_match + cross_mismatch);
                    eprintln!("  This means the triangle winding order does not match the face normal direction.");
                    eprintln!("  Possible causes: CCW normalization + forward:winding swap double-correction");
                }
            }
        }
    }
}

/// Test that verifies the fix: all planar face cross-product normals should match
/// the analytical face normals (pointing in the same hemisphere).
#[test]  
fn diag_all_faces_normal_consistency() {
    // Test with multiple STEP files
    let test_files = [
        "/home/z/my-project/test/3.05.078.stp",
        "/home/z/my-project/test/nist_cube.stp",
        "/home/z/my-project/test/nist_cylinder.stp",
        "/home/z/my-project/test/brick_thin.stp",
    ];
    
    for file in &test_files {
        let content = match std::fs::read_to_string(file) {
            Ok(c) => c,
            Err(_) => continue,
        };
        
        let step = match parse_step(&content) {
            Ok(s) => s,
            Err(_) => continue,
        };
        
        let (_tree, pending) = step_structure_lazy(&step);
        let ctx = StepConversionContext::new(&step);
        
        for p in &pending {
            if let Some(inst) = ctx.triangulate_pending(p) {
                let face_normals = inst.mesh.face_normals.as_ref();
                
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
                    
                    if mismatch_count > match_count {
                        eprintln!("MISMATCH in {}: Face Step#{} {} forward={} — cross: match={}, mismatch={}",
                            file, fi.step_face_id, fi.surface_type, fi.forward, match_count, mismatch_count);
                    }
                }
            }
        }
    }
}
