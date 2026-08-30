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


#[test]
fn diag_3d_view_detail() {
    let content = std::fs::read_to_string(test_file("3.05.078.stp"))
        .expect("Failed to read 3.05.078.stp");
    
    let step = parse_step(&content).expect("Failed to parse STEP file");
    eprintln!("Parsed STEP file: {} entities", step.entities.len());
    
    let (_tree, pending) = step_structure_lazy(&step);
    eprintln!("Pending BREPs: {}", pending.len());
    
    let ctx = StepConversionContext::new(&step);
    
    for (pi, p) in pending.iter().enumerate() {
        eprintln!("\n=== Pending #{}: brep_id={}, name={} ===", pi, p.brep_id, p.name);
        if let Some(inst) = ctx.triangulate_pending(p) {
            eprintln!("Mesh: {} vertices, {} triangles", inst.mesh.vertices.len(), inst.mesh.triangles.len());
            
            let target_step_ids: Vec<i64> = vec![78, 87];
            
            for fi in &inst.faces {
                if !target_step_ids.contains(&fi.step_face_id) {
                    continue;
                }
                
                eprintln!("\n--- Face Step#{} {} (face_id={}, forward={}) ---",
                    fi.step_face_id, fi.surface_type, fi.face_id, fi.forward);
                
                let (start, end) = fi.triangle_range;
                if start >= end {
                    eprintln!("  *** EMPTY triangle_range ({}, {})", start, end);
                    continue;
                }
                if end > inst.mesh.triangles.len() {
                    eprintln!("  *** OUT OF BOUNDS: triangle_range ({}, {}) but mesh has {} triangles",
                        start, end, inst.mesh.triangles.len());
                    continue;
                }
                
                // Check 3D triangle orientations
                eprintln!("  Checking {} 3D triangles for normal consistency...", end - start);
                let mut normal_positive = 0usize;
                let mut normal_negative = 0usize;
                let mut degenerate_count = 0usize;
                
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
                        degenerate_count += 1;
                    } else {
                        if fi.step_face_id == 87 {
                            // Plane at x=0, forward=true → normal should be +x
                            if nx > 0.0 { normal_positive += 1; }
                            else { normal_negative += 1; }
                        } else {
                            // General
                            if nz > 0.0 { normal_positive += 1; }
                            else { normal_negative += 1; }
                        }
                    }
                }
                eprintln!("  normal_positive: {}, normal_negative: {}, degenerate: {}",
                    normal_positive, normal_negative, degenerate_count);
                
                if fi.step_face_id == 87 && normal_negative > normal_positive {
                    eprintln!("  *** BUG: Most normals point in -x but forward=true → face inside-out!");
                }
                
                // Check vertex sharing with adjacent faces
                let face_verts: std::collections::HashSet<u32> = (start..end)
                    .flat_map(|i| {
                        let tri = inst.mesh.triangles[i];
                        [tri[0], tri[1], tri[2]]
                    })
                    .collect();
                eprintln!("  Unique vertex indices: {}", face_verts.len());
                
                // Find adjacent faces sharing vertices
                for other_fi in &inst.faces {
                    if other_fi.face_id == fi.face_id { continue; }
                    let (os, oe) = other_fi.triangle_range;
                    if os >= oe || oe > inst.mesh.triangles.len() { continue; }
                    let other_verts: std::collections::HashSet<u32> = (os..oe)
                        .flat_map(|i| {
                            let tri = inst.mesh.triangles[i];
                            [tri[0], tri[1], tri[2]]
                        })
                        .collect();
                    let shared = face_verts.intersection(&other_verts).count();
                    if shared > 0 {
                        eprintln!("    Adjacent: face_id={} (Step#{}, {}): {} shared vertices",
                            other_fi.face_id, other_fi.step_face_id, other_fi.surface_type, shared);
                    }
                }
                
                // UV vs 3D triangle count
                eprintln!("  UV triangles: {}, 3D triangles: {}", fi.uv_triangles.len(), end - start);
                if fi.uv_triangles.len() != end - start {
                    eprintln!("  *** MISMATCH: UV and 3D triangle counts differ!");
                }
                
                // For Step#78 (cone)
                if fi.step_face_id == 78 {
                    if let draper_geometry::Surface::Cone(ref cone) = fi.surface {
                        eprintln!("  Cone: radius={:.6}, half_angle={:.4}deg, expanding={}, apex_v={:.6}",
                            cone.radius, cone.half_angle.to_degrees(), cone.expanding, cone.apex_v());
                        eprintln!("  origin=({:.4},{:.4},{:.4}), axis=({:.4},{:.4},{:.4})",
                            cone.origin.x, cone.origin.y, cone.origin.z,
                            cone.axis.x, cone.axis.y, cone.axis.z);
                        
                        // Check that the ring vertices at the boundary are at correct positions
                        let base_verts: Vec<_> = face_verts.iter()
                            .map(|&vi| (vi, inst.mesh.vertices[vi as usize]))
                            .filter(|(_, p)| {
                                let v = (p.x - cone.origin.x) * cone.axis.x
                                      + (p.y - cone.origin.y) * cone.axis.y
                                      + (p.z - cone.origin.z) * cone.axis.z;
                                (v - 2.43).abs() < 0.5
                            })
                            .collect();
                        eprintln!("  Base ring (v≈2.43): {} vertices", base_verts.len());
                        for &(vi, p) in base_verts.iter().take(5) {
                            eprintln!("    vi={}: ({:.4}, {:.4}, {:.4})", vi, p.x, p.y, p.z);
                        }
                        
                        let apex_verts: Vec<_> = face_verts.iter()
                            .map(|&vi| (vi, inst.mesh.vertices[vi as usize]))
                            .filter(|(_, p)| {
                                let v = (p.x - cone.origin.x) * cone.axis.x
                                      + (p.y - cone.origin.y) * cone.axis.y
                                      + (p.z - cone.origin.z) * cone.axis.z;
                                (v - (-2.43)).abs() < 0.5
                            })
                            .collect();
                        eprintln!("  Apex ring (v≈-2.43): {} vertices", apex_verts.len());
                        for &(vi, p) in apex_verts.iter().take(5) {
                            eprintln!("    vi={}: ({:.4}, {:.4}, {:.4})", vi, p.x, p.y, p.z);
                        }
                    }
                }
                
                // For Step#87 (plane)
                if fi.step_face_id == 87 {
                    let x_min = face_verts.iter().map(|&vi| inst.mesh.vertices[vi as usize].x)
                        .fold(f64::MAX, f64::min);
                    let x_max = face_verts.iter().map(|&vi| inst.mesh.vertices[vi as usize].x)
                        .fold(f64::MIN, f64::max);
                    eprintln!("  Vertex x range: [{:.6}, {:.6}]", x_min, x_max);
                    
                    if x_max.abs() > 0.001 || x_min.abs() > 0.001 {
                        eprintln!("  *** WARNING: Not all vertices at x=0!");
                    }
                    
                    // Print first few 3D triangles
                    eprintln!("  First 5 3D triangles:");
                    for i in start..std::cmp::min(start + 5, end) {
                        let tri = inst.mesh.triangles[i];
                        let v0 = inst.mesh.vertices[tri[0] as usize];
                        let v1 = inst.mesh.vertices[tri[1] as usize];
                        let v2 = inst.mesh.vertices[tri[2] as usize];
                        eprintln!("    tri[{}]: ({:.4},{:.4},{:.4}) ({:.4},{:.4},{:.4}) ({:.4},{:.4},{:.4})",
                            i, v0.x, v0.y, v0.z, v1.x, v1.y, v1.z, v2.x, v2.y, v2.z);
                    }
                }
            }
            
            // Overall mesh watertightness
            eprintln!("\n=== Mesh watertightness check ===");
            let mut edge_count: std::collections::HashMap<[u32; 2], usize> = std::collections::HashMap::new();
            for tri in &inst.mesh.triangles {
                let mut edges = [
                    [tri[0], tri[1]],
                    [tri[1], tri[2]],
                    [tri[2], tri[0]],
                ];
                for e in &mut edges {
                    e.sort();
                    *edge_count.entry(*e).or_insert(0) += 1;
                }
            }
            let boundary_edges = edge_count.iter().filter(|(_, &count)| count == 1).count();
            let manifold_edges = edge_count.iter().filter(|(_, &count)| count == 2).count();
            let non_manifold_edges = edge_count.iter().filter(|(_, &count)| count > 2).count();
            eprintln!("  Total edges: {}", edge_count.len());
            eprintln!("  Boundary edges (1 face): {}", boundary_edges);
            eprintln!("  Manifold edges (2 faces): {}", manifold_edges);
            eprintln!("  Non-manifold edges (>2 faces): {}", non_manifold_edges);
            
            if non_manifold_edges > 0 {
                eprintln!("  WARNING: Non-manifold edges detected!");
                let mut count = 0;
                for (e, c) in edge_count.iter() {
                    if *c > 2 {
                        let v0 = inst.mesh.vertices[e[0] as usize];
                        let v1 = inst.mesh.vertices[e[1] as usize];
                        eprintln!("    vi={},vj={}, count={}, pos=({:.4},{:.4},{:.4})-({:.4},{:.4},{:.4})",
                            e[0], e[1], c, v0.x, v0.y, v0.z, v1.x, v1.y, v1.z);
                        count += 1;
                        if count > 10 { break; }
                    }
                }
            }
        }
    }
}
