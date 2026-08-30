use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_mesh::{TriangleMesh, filter_degenerate_triangles, weld_boundary_edge_vertices, validate_watertight};

/// Resolve a test-data file relative to the workspace `test/` dir.
/// Robust to repo relocation: derived from CARGO_MANIFEST_DIR
/// (crates/draper-step -> workspace root), not from a hardcoded sandbox path.
fn test_file(name: &str) -> std::path::PathBuf {
    let mut dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    dir.pop(); // crates/draper-step -> crates
    dir.pop(); // crates -> workspace root
    dir.join("test").join(name)
}


/// Track triangle counts per face_id through each post-processing step
fn count_face_triangles(mesh: &TriangleMesh) -> std::collections::HashMap<u64, usize> {
    let mut counts = std::collections::HashMap::new();
    if let Some(ref fids) = mesh.triangle_face_ids {
        for &fid in fids {
            *counts.entry(fid).or_insert(0) += 1;
        }
    }
    counts
}

#[test]
fn diag_postprocessing_steps() {
    let content = std::fs::read_to_string(test_file("3.05.078.stp"))
        .expect("Failed to read 3.05.078.stp");
    
    let step = parse_step(&content).expect("Failed to parse STEP file");
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);
    
    // We need to manually replicate the post-processing steps to track what happens
    // For now, let's just use the public API and check the final result
    
    for (pi, p) in pending.iter().enumerate() {
        if let Some(inst) = ctx.triangulate_pending(p) {
            eprintln!("=== Instance #{} ===", pi);
            
            // Check face triangle counts
            for fi in &inst.faces {
                if fi.step_face_id == 78 || fi.step_face_id == 87 {
                    let (start, end) = fi.triangle_range;
                    let count_3d = if end <= inst.mesh.triangles.len() { end - start } else { 0 };
                    let count_uv = fi.uv_triangles.len();
                    eprintln!("  Step#{} (face_id={}): UV={} 3D={} diff={}",
                        fi.step_face_id, fi.face_id, count_uv, count_3d,
                        count_uv as isize - count_3d as isize);
                    
                    // Count how many triangles with this face_id exist in triangle_face_ids
                    if let Some(ref fids) = inst.mesh.triangle_face_ids {
                        let actual_count = fids.iter().filter(|&&fid| fid == fi.face_id).count();
                        eprintln!("    triangle_face_ids count for face_id={}: {}", fi.face_id, actual_count);
                        
                        // Find the actual range in triangle_face_ids
                        let mut actual_start = usize::MAX;
                        let mut actual_end = 0usize;
                        for (ti, &fid) in fids.iter().enumerate() {
                            if fid == fi.face_id {
                                actual_start = actual_start.min(ti);
                                actual_end = actual_end.max(ti + 1);
                            }
                        }
                        eprintln!("    actual range in triangle_face_ids: [{}, {})", actual_start, actual_end);
                    }
                }
            }
            
            // Now let's check what the pre-postprocessing mesh would look like
            // We can't easily get that, but we can check the merge_deduplicating behavior
            
            // Check for triangles that have all 3 vertices on the same boundary edge
            // (these might be removed as duplicates by adjacent faces)
            eprintln!("\n  Checking for potential duplicate triangles between faces...");
            
            // Group triangles by sorted vertex indices
            let mut tri_by_verts: std::collections::HashMap<[u32; 3], Vec<(usize, u64)>> = std::collections::HashMap::new();
            for (ti, tri) in inst.mesh.triangles.iter().enumerate() {
                let mut key = [tri[0], tri[1], tri[2]];
                key.sort_unstable();
                let fid = inst.mesh.triangle_face_ids.as_ref()
                    .and_then(|fids| fids.get(ti))
                    .copied()
                    .unwrap_or(u64::MAX);
                tri_by_verts.entry(key).or_default().push((ti, fid));
            }
            
            // Find triangles that share the same 3 vertices (these would be removed as duplicates)
            let mut duplicate_count = 0;
            let mut cross_face_duplicates = 0;
            for (key, entries) in &tri_by_verts {
                if entries.len() > 1 {
                    duplicate_count += entries.len() - 1;
                    let face_ids: std::collections::HashSet<u64> = entries.iter().map(|(_, fid)| *fid).collect();
                    if face_ids.len() > 1 {
                        cross_face_duplicates += entries.len() - 1;
                        if cross_face_duplicates <= 10 {
                            eprintln!("    CROSS-FACE DUPLICATE: vertices [{}, {}, {}], faces: {:?}",
                                key[0], key[1], key[2],
                                entries.iter().map(|(_, fid)| *fid).collect::<Vec<_>>());
                        }
                    }
                }
            }
            eprintln!("  Total potential duplicate triangles: {}", duplicate_count);
            eprintln!("  Cross-face duplicate triangles: {}", cross_face_duplicates);
            
            if cross_face_duplicates > 0 {
                eprintln!("  *** BUG: {} triangles from different faces share the same vertex indices!", cross_face_duplicates);
                eprintln!("  *** These are removed by remove_duplicate_triangles, creating holes in the 3D view.");
            }
        }
    }
}
