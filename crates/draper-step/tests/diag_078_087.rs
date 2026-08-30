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
fn diag_step_078_087() {
    let content = std::fs::read_to_string(test_file("3.05.078.stp"))
        .expect("Failed to read 3.05.078.stp");
    
    let step = parse_step(&content).expect("Failed to parse STEP file");
    
    eprintln!("Parsed STEP file: {} entities", step.entities.len());
    
    let (_tree, pending) = step_structure_lazy(&step);
    
    eprintln!("Pending BREPs: {}", pending.len());
    
    let ctx = StepConversionContext::new(&step);
    
    for (pi, p) in pending.iter().enumerate() {
        eprintln!("\n  Pending #{}: brep_id={}, name={}", pi, p.brep_id, p.name);
        if let Some(inst) = ctx.triangulate_pending(p) {
            eprintln!("  Mesh: {} vertices, {} triangles", inst.mesh.vertices.len(), inst.mesh.triangles.len());
            
            for fi in &inst.faces {
                let step_id = fi.step_face_id;
                if step_id == 87 || step_id == 78 {
                    eprintln!("\n  --- Face Step#{} {} (face_id={}, forward={}) ---",
                        step_id, fi.surface_type, fi.face_id, fi.forward);
                    
                    // Cone-specific diagnostics
                    if let draper_geometry::Surface::Cone(ref cone) = fi.surface {
                        eprintln!("  CONE: radius={:.6}, half_angle={:.4}deg, expanding={}, apex_v={:.6}",
                            cone.radius, cone.half_angle.to_degrees(), cone.expanding, cone.apex_v());
                        for vt in [-2.43, -1.0, 0.0, 1.0, 2.43] {
                            let r = if cone.expanding { vt * cone.half_angle.tan() } else { (cone.radius + vt * cone.half_angle.tan()).max(0.0) };
                            eprintln!("    v={:.2}: radius={:.6}", vt, r);
                        }
                    }
                    
                    eprintln!("  triangle_range: {:?}", fi.triangle_range);
                    eprintln!("  mesh total triangles: {}", inst.mesh.triangles.len());
                    eprintln!("  mesh.triangle_face_ids is_some: {}", inst.mesh.triangle_face_ids.is_some());
                    if let Some(ref fids) = inst.mesh.triangle_face_ids {
                        eprintln!("  triangle_face_ids length: {}", fids.len());
                        eprintln!("  triangles length: {}", inst.mesh.triangles.len());
                        // Count per face_id
                        let mut face_counts: std::collections::HashMap<u64, usize> = std::collections::HashMap::new();
                        for &fid in fids { *face_counts.entry(fid).or_insert(0) += 1; }
                        let mut sorted: Vec<_> = face_counts.iter().collect();
                        sorted.sort_by_key(|(fid, _)| **fid);
                        for (fid, count) in sorted.iter().take(15) {
                            eprintln!("    face_id={}: {} triangles", fid, count);
                        }
                    }
                    if fi.triangle_range.1 > inst.mesh.triangles.len() {
                        eprintln!("  *** BUG: triangle_range END exceeds mesh size! ***");
                        // Count how many triangles have this face_id
                        if let Some(ref fids) = inst.mesh.triangle_face_ids {
                            let count = fids.iter().filter(|&&fid| fid == fi.face_id).count();
                            eprintln!("  face_id={} has {} triangles in triangle_face_ids", fi.face_id, count);
                            // Find actual range
                            let mut actual_start = usize::MAX;
                            let mut actual_end = 0usize;
                            for (ti, &fid) in fids.iter().enumerate() {
                                if fid == fi.face_id {
                                    actual_start = actual_start.min(ti);
                                    actual_end = actual_end.max(ti + 1);
                                }
                            }
                            eprintln!("  actual range from triangle_face_ids: ({}, {})", actual_start, actual_end);
                        }
                    }
                    eprintln!("  outer_uv_boundary: {} polylines", fi.outer_uv_boundary.len());
                    for (i, poly) in fi.outer_uv_boundary.iter().enumerate() {
                        eprintln!("    polyline {}: {} points", i, poly.len());
                        if poly.len() <= 10 {
                            for p in poly {
                                eprintln!("      ({:.6}, {:.6})", p.u, p.v);
                            }
                        } else {
                            eprintln!("      first: ({:.6}, {:.6}), last: ({:.6}, {:.6})",
                                poly[0].u, poly[0].v, poly.last().unwrap().u, poly.last().unwrap().v);
                        }
                    }
                    eprintln!("  inner_uv_boundaries: {} hole groups", fi.inner_uv_boundaries.len());
                    for (i, hole_group) in fi.inner_uv_boundaries.iter().enumerate() {
                        for (j, poly) in hole_group.iter().enumerate() {
                            eprintln!("    hole {}-{}: {} points", i, j, poly.len());
                        }
                    }
                    eprintln!("  uv_triangles: {} triangles", fi.uv_triangles.len());
                    
                    let mut degenerate_count = 0;
                    let mut zero_area_count = 0;
                    let mut total_area = 0.0_f64;
                    for tri in &fi.uv_triangles {
                        let dx1 = tri[1].u - tri[0].u;
                        let dy1 = tri[1].v - tri[0].v;
                        let dx2 = tri[2].u - tri[0].u;
                        let dy2 = tri[2].v - tri[0].v;
                        let area = (dx1 * dy2 - dx2 * dy1).abs() / 2.0;
                        total_area += area;
                        if area < 1e-10 {
                            degenerate_count += 1;
                        }
                        if area < 1e-20 {
                            zero_area_count += 1;
                        }
                    }
                    eprintln!("  total UV area: {:.6}", total_area);
                    eprintln!("  degenerate_triangles (area < 1e-10): {}", degenerate_count);
                    eprintln!("  zero_area_triangles (area < 1e-20): {}", zero_area_count);
                    
                    // Check UV range
                    let mut u_min = f64::MAX;
                    let mut u_max = f64::MIN;
                    let mut v_min = f64::MAX;
                    let mut v_max = f64::MIN;
                    for tri in &fi.uv_triangles {
                        for p in tri {
                            u_min = u_min.min(p.u);
                            u_max = u_max.max(p.u);
                            v_min = v_min.min(p.v);
                            v_max = v_max.max(p.v);
                        }
                    }
                    eprintln!("  UV range: u=[{:.6}, {:.6}], v=[{:.6}, {:.6}]", u_min, u_max, v_min, v_max);
                    
                    // Print first few UV triangles
                    let print_limit = 20.min(fi.uv_triangles.len());
                    eprintln!("  first {} UV triangles:", print_limit);
                    for (i, tri) in fi.uv_triangles.iter().take(print_limit).enumerate() {
                        eprintln!("    tri[{}]: ({:.4},{:.4}) ({:.4},{:.4}) ({:.4},{:.4})",
                            i, tri[0].u, tri[0].v, tri[1].u, tri[1].v, tri[2].u, tri[2].v);
                    }
                    
                    // Print 3D triangles 
                    let (start, end) = fi.triangle_range;
                    if start < end && end <= inst.mesh.triangles.len() {
                        eprintln!("  3D triangles ({} total):", end - start);
                        let tri3d_print = 5.min(end - start);
                        for i in start..start+tri3d_print {
                            let tri = inst.mesh.triangles[i];
                            let v0 = inst.mesh.vertices[tri[0] as usize];
                            let v1 = inst.mesh.vertices[tri[1] as usize];
                            let v2 = inst.mesh.vertices[tri[2] as usize];
                            eprintln!("    tri3d[{}]: ({:.4},{:.4},{:.4}) ({:.4},{:.4},{:.4}) ({:.4},{:.4},{:.4})",
                                i, v0.x, v0.y, v0.z, v1.x, v1.y, v1.z, v2.x, v2.y, v2.z);
                        }
                    }
                    
                    // Check if any UV triangle has NaN or Inf
                    let mut nan_count = 0;
                    for tri in &fi.uv_triangles {
                        for p in tri {
                            if !p.u.is_finite() || !p.v.is_finite() {
                                nan_count += 1;
                            }
                        }
                    }
                    eprintln!("  NaN/Inf UV points: {}", nan_count);
                }
            }
        }
    }
}
