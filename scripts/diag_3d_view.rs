// Diagnostic script to investigate Step#78 (cone) and Step#87 (plane) 3D view issues.
// Run with: cargo run --example diag_3d_view -p draper-step

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};

fn main() {
    let content = std::fs::read_to_string("/home/z/my-project/test/3.05.078.stp")
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
            
            // Find Step#78 and Step#87 faces
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
                
                // Check 3D triangle orientations (normal direction)
                eprintln!("  Checking {} 3D triangles for normal consistency...", end - start);
                let mut normal_positive_z = 0usize;
                let mut normal_negative_z = 0usize;
                let mut normal_other = 0usize;
                let mut degenerate_count = 0usize;
                
                for i in start..end {
                    let tri = inst.mesh.triangles[i];
                    let v0 = inst.mesh.vertices[tri[0] as usize];
                    let v1 = inst.mesh.vertices[tri[1] as usize];
                    let v2 = inst.mesh.vertices[tri[2] as usize];
                    
                    // Cross product for triangle normal
                    let e1 = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
                    let e2 = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
                    let nx = e1.1 * e2.2 - e1.2 * e2.1;
                    let ny = e1.2 * e2.0 - e1.0 * e2.2;
                    let nz = e1.0 * e2.1 - e1.1 * e2.0;
                    let len = (nx*nx + ny*ny + nz*nz).sqrt();
                    
                    if len < 1e-15 {
                        degenerate_count += 1;
                    } else {
                        // For a cone (Step#78): normal should point outward
                        // For a plane at x=0 (Step#87): normal should point in +x direction (forward=true)
                        if fi.step_face_id == 87 {
                            if nx > 0.0 { normal_positive_z += 1; }
                            else if nx < 0.0 { normal_negative_z += 1; }
                        } else {
                            // General orientation
                            if nz > 0.0 { normal_positive_z += 1; }
                            else if nz < 0.0 { normal_negative_z += 1; }
                        }
                    }
                }
                eprintln!("  normal_positive: {}, normal_negative: {}, degenerate: {}",
                    normal_positive_z, normal_negative_z, degenerate_count);
                
                // For Step#87 (plane at x=0, forward=true): normals should point in +x
                if fi.step_face_id == 87 {
                    if normal_negative_z > normal_positive_z {
                        eprintln!("  *** WARNING: Most normals point in -x direction but forward=true!");
                        eprintln!("  *** This means the face appears inside-out in 3D view.");
                    }
                }
                
                // Check boundary vertices sharing with adjacent faces
                eprintln!("\n  Checking shared boundary vertices with adjacent faces...");
                let face_verts: std::collections::HashSet<u32> = (start..end)
                    .flat_map(|i| {
                        let tri = inst.mesh.triangles[i];
                        [tri[0], tri[1], tri[2]]
                    })
                    .collect();
                eprintln!("  This face uses {} unique vertex indices", face_verts.len());
                
                // Find adjacent faces that share vertices
                let mut adjacent_faces: std::collections::HashMap<u64, usize> = std::collections::HashMap::new();
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
                        adjacent_faces.insert(other_fi.face_id, shared);
                    }
                }
                eprintln!("  Adjacent faces sharing vertices:");
                let mut sorted_adj: Vec<_> = adjacent_faces.iter().collect();
                sorted_adj.sort_by_key(|(_, count)| std::cmp::Reverse(**count));
                for (fid, count) in sorted_adj.iter().take(10) {
                    // Find the face info for this face_id
                    if let Some(other_fi) = inst.faces.iter().find(|f| f.face_id == **fid) {
                        eprintln!("    face_id={} (Step#{}, {}): {} shared vertices",
                            fid, other_fi.step_face_id, other_fi.surface_type, count);
                    }
                }
                
                // Check if boundary vertices from the edge cache are shared
                eprintln!("\n  Boundary edge vertex positions:");
                let mut boundary_vert_positions: Vec<(u32, draper_geometry::Point3d)> = Vec::new();
                for &vi in &face_verts {
                    let v = inst.mesh.vertices[vi as usize];
                    // For cone: boundary is at v_min and v_max rings
                    // For plane: boundary is the outer and hole edges
                    // Just collect all vertices for now
                    boundary_vert_positions.push((vi, v));
                }
                
                // For Step#78 (cone): check if the base ring vertices match the base cap face
                if fi.step_face_id == 78 {
                    if let draper_geometry::Surface::Cone(ref cone) = fi.surface {
                        eprintln!("  Cone: radius={:.6}, half_angle={:.4}deg, expanding={}, apex_v={:.6}",
                            cone.radius, cone.half_angle.to_degrees(), cone.expanding, cone.apex_v());
                        
                        // Find vertices at v_max (base ring)
                        let apex_v = cone.apex_v();
                        eprintln!("  apex_v = {:.6}", apex_v);
                        
                        // Print some vertex positions along the base ring
                        let base_ring_verts: Vec<(u32, draper_geometry::Point3d)> = boundary_vert_positions.iter()
                            .filter(|(_, p)| {
                                let v = (p.x - cone.origin.x) * cone.axis.x
                                      + (p.y - cone.origin.y) * cone.axis.y
                                      + (p.z - cone.origin.z) * cone.axis.z;
                                (v - 2.43).abs() < 0.1
                            })
                            .cloned()
                            .collect();
                        eprintln!("  Base ring (v≈2.43): {} vertices", base_ring_verts.len());
                        for (vi, p) in base_ring_verts.iter().take(5) {
                            eprintln!("    vi={}: ({:.4}, {:.4}, {:.4})", vi, p.x, p.y, p.z);
                        }
                        
                        // Also check apex ring (v≈-2.43)
                        let apex_ring_verts: Vec<(u32, draper_geometry::Point3d)> = boundary_vert_positions.iter()
                            .filter(|(_, p)| {
                                let v = (p.x - cone.origin.x) * cone.axis.x
                                      + (p.y - cone.origin.y) * cone.axis.y
                                      + (p.z - cone.origin.z) * cone.axis.z;
                                (v - (-2.43)).abs() < 0.1
                            })
                            .cloned()
                            .collect();
                        eprintln!("  Apex ring (v≈-2.43): {} vertices", apex_ring_verts.len());
                        for (vi, p) in apex_ring_verts.iter().take(5) {
                            eprintln!("    vi={}: ({:.4}, {:.4}, {:.4})", vi, p.x, p.y, p.z);
                        }
                    }
                }
                
                // For Step#87 (plane): check that all vertices lie on x=0
                if fi.step_face_id == 87 {
                    let x_values: Vec<f64> = boundary_vert_positions.iter()
                        .map(|(_, p)| p.x)
                        .collect();
                    let x_min = x_values.iter().cloned().fold(f64::MAX, f64::min);
                    let x_max = x_values.iter().cloned().fold(f64::MIN, f64::max);
                    eprintln!("  All vertex x range: [{:.6}, {:.6}]", x_min, x_max);
                    if x_max.abs() > 0.001 || x_min.abs() > 0.001 {
                        eprintln!("  *** WARNING: Not all vertices at x=0! Vertices are off the plane.");
                        eprintln!("  *** This would cause the face to not align with adjacent faces.");
                    }
                    
                    // Check if the normal is in the +x direction (forward=true)
                    eprintln!("  Checking if normals point in +x direction...");
                }
                
                // Check 3D triangle positions vs UV triangle positions
                eprintln!("\n  UV vs 3D triangle count: UV={}, 3D={}", 
                    fi.uv_triangles.len(), end - start);
                if fi.uv_triangles.len() != end - start {
                    eprintln!("  *** MISMATCH: UV and 3D triangle counts differ!");
                    eprintln!("  *** This means the UV view shows different triangles than the 3D view.");
                }
            }
            
            // Also check overall mesh watertightness
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
            
            // Find non-manifold edges
            if non_manifold_edges > 0 {
                eprintln!("  Non-manifold edges:");
                for (e, count) in edge_count.iter() {
                    if *count > 2 {
                        let v0 = inst.mesh.vertices[e[0] as usize];
                        let v1 = inst.mesh.vertices[e[1] as usize];
                        eprintln!("    vi={},vj={}, count={}, pos=({:.4},{:.4},{:.4})-({:.4},{:.4},{:.4})",
                            e[0], e[1], count, v0.x, v0.y, v0.z, v1.x, v1.y, v1.z);
                    }
                }
            }
        }
    }
}
