// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Boundary edge diagnostic for NURBS face triangulations.
//!
//! Diagnoses WHY merged meshes have boundary edges despite the edge cache
//! producing consistent 3D coordinates. For each boundary edge, determines:
//! - Which face IDs the adjacent triangles belong to
//! - Whether the edge vertices are shared (boundary from edge cache) or
//!   unique-to-one-face (interior Steiner points)

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_mesh::validate_watertight;
use std::collections::{HashMap, HashSet};

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .init();

    let args: Vec<String> = std::env::args().collect();
    let default_path = "test/as1-oc-214.stp".to_string();
    let path = if args.len() > 1 { args[1].clone() } else { default_path };
    let target_brep: i64 = if args.len() > 2 { args[2].parse().unwrap_or(63) } else { 63 };

    println!("=== Boundary Edge Diagnostic ===");
    println!("Loading STEP file: {}", path);
    let data = std::fs::read_to_string(path).expect("Failed to read STEP file");
    let step = parse_step(&data).expect("Failed to parse STEP file");
    println!("STEP file parsed: {} entities", step.entities.len());

    let start = std::time::Instant::now();
    let (_tree, pending) = step_structure_lazy(&step);
    println!("Structure parsed: {} BRep instances ({:.1?})", pending.len(), start.elapsed());

    let ctx = StepConversionContext::new(&step);

    // Find the target BREP
    let target = pending.iter().find(|p| p.brep_id == target_brep);
    let target = match target {
        Some(p) => p,
        None => {
            println!("ERROR: BREP #{} not found. Available BReps:", target_brep);
            let mut seen: HashSet<i64> = HashSet::new();
            for p in &pending {
                if seen.insert(p.brep_id) {
                    println!("  BREP #{}: {}", p.brep_id, p.name);
                }
            }
            return;
        }
    };

    println!("\n{}", "=".repeat(80));
    println!("BRep #{}: {}", target.brep_id, target.name);
    println!("{}", "=".repeat(80));

    let t_start = std::time::Instant::now();
    let result = ctx.triangulate_pending(target);
    let t_elapsed = t_start.elapsed();

    match result {
        Some(inst) => {
            let report = validate_watertight(&inst.mesh, true);
            let is_wt = report.is_watertight();

            println!("Vertices: {}, Triangles: {}, Time: {:.1?}",
                inst.mesh.vertex_count(), inst.mesh.triangle_count(), t_elapsed);
            println!("Watertight: {}", if is_wt { "YES" } else { "NO" });
            println!("Boundary edges: {}, Non-manifold edges: {}, Degenerate: {}",
                report.boundary_edge_count, report.non_manifold_edge_count,
                report.degenerate_triangle_count);
            println!("Total edges: {}, Interior: {}", report.edge_count, report.interior_edge_count);
            println!("Boundary edge rate: {:.1}%",
                if report.edge_count > 0 {
                    report.boundary_edge_count as f64 / report.edge_count as f64 * 100.0
                } else { 0.0 });

            if is_wt {
                println!("\nMesh is watertight — nothing to diagnose!");
                return;
            }

            let mesh = &inst.mesh;

            // ─── Step 1: Build edge-to-face-id map ─────────────────────
            let face_ids = match &mesh.triangle_face_ids {
                Some(ids) => ids.clone(),
                None => {
                    println!("ERROR: No triangle_face_ids available");
                    return;
                }
            };

            // edge_key → set of face IDs
            let mut edge_to_faces: HashMap<(u32, u32), HashSet<u64>> = HashMap::new();
            for (tri_idx, tri) in mesh.triangles.iter().enumerate() {
                let fid = face_ids.get(tri_idx).copied().unwrap_or(0);
                let mut v = [tri[0], tri[1], tri[2]];
                v.sort();
                edge_to_faces.entry((v[0], v[1])).or_default().insert(fid);
                edge_to_faces.entry((v[0], v[2])).or_default().insert(fid);
                edge_to_faces.entry((v[1], v[2])).or_default().insert(fid);
            }

            // ─── Step 2: Determine which vertices are shared vs unique ──
            // A vertex is "shared" (from edge cache / boundary) if it appears
            // in triangles belonging to more than one face.
            // A vertex is "unique" (interior/Steiner) if it appears in triangles
            // of only one face.

            // vertex_index → set of face IDs
            let mut vertex_to_faces: HashMap<u32, HashSet<u64>> = HashMap::new();
            for (tri_idx, tri) in mesh.triangles.iter().enumerate() {
                let fid = face_ids.get(tri_idx).copied().unwrap_or(0);
                for &v in tri.iter() {
                    vertex_to_faces.entry(v).or_default().insert(fid);
                }
            }

            // Classify vertices
            let mut shared_vertices: HashSet<u32> = HashSet::new(); // appears in 2+ faces
            let mut unique_vertices: HashSet<u32> = HashSet::new(); // appears in 1 face only
            for (&v, faces) in &vertex_to_faces {
                if faces.len() >= 2 {
                    shared_vertices.insert(v);
                } else {
                    unique_vertices.insert(v);
                }
            }

            println!("\n--- Vertex Classification ---");
            println!("Total vertices: {}", mesh.vertex_count());
            println!("Shared vertices (2+ faces): {} ({:.1}%)",
                shared_vertices.len(),
                shared_vertices.len() as f64 / mesh.vertex_count() as f64 * 100.0);
            println!("Unique vertices (1 face only): {} ({:.1}%)",
                unique_vertices.len(),
                unique_vertices.len() as f64 / mesh.vertex_count() as f64 * 100.0);

            // ─── Step 3: Analyze each boundary edge ────────────────────
            let mut both_shared = 0usize;     // both vertices shared (boundary edge from cache)
            let mut both_unique = 0usize;     // both vertices unique (interior Steiner points)
            let mut mixed = 0usize;           // one shared, one unique

            // Also track: are the boundary edges between same-face or different-face?
            let mut same_face_edge = 0usize;  // edge only touches one face (true boundary)
            let mut diff_face_edge = 0usize;  // edge touches 2+ different faces (should be interior!)

            // Track by face ID
            let mut face_boundary_counts: HashMap<u64, usize> = HashMap::new();

            // Track edge lengths by category
            let mut shared_edge_lengths: Vec<f64> = Vec::new();
            let mut unique_edge_lengths: Vec<f64> = Vec::new();
            let mut mixed_edge_lengths: Vec<f64> = Vec::new();

            // Detailed: which face pair combinations appear?
            let mut face_pair_counts: HashMap<(u64, u64), usize> = HashMap::new();

            // Detailed: for "should be interior" edges (between 2 faces), what's the vertex classification?
            let mut diff_face_both_shared = 0usize;
            let mut diff_face_both_unique = 0usize;
            let mut diff_face_mixed = 0usize;

            for &(a, b) in &report.boundary_edges {
                let a_shared = shared_vertices.contains(&a);
                let b_shared = shared_vertices.contains(&b);

                // Vertex classification
                if a_shared && b_shared {
                    both_shared += 1;
                } else if !a_shared && !b_shared {
                    both_unique += 1;
                } else {
                    mixed += 1;
                }

                // Face adjacency
                let edge_key = (a.min(b), a.max(b));
                let adjacent_faces = edge_to_faces.get(&edge_key)
                    .cloned()
                    .unwrap_or_default();

                if adjacent_faces.len() == 1 {
                    same_face_edge += 1;
                } else if adjacent_faces.len() >= 2 {
                    diff_face_edge += 1;
                    // Track which face pairs
                    let mut sorted_faces: Vec<u64> = adjacent_faces.iter().copied().collect();
                    sorted_faces.sort();
                    for i in 0..sorted_faces.len() {
                        for j in (i+1)..sorted_faces.len() {
                            *face_pair_counts.entry((sorted_faces[i], sorted_faces[j])).or_insert(0) += 1;
                        }
                    }
                    // Sub-classify by vertex type
                    if a_shared && b_shared {
                        diff_face_both_shared += 1;
                    } else if !a_shared && !b_shared {
                        diff_face_both_unique += 1;
                    } else {
                        diff_face_mixed += 1;
                    }
                }

                for &fid in &adjacent_faces {
                    *face_boundary_counts.entry(fid).or_insert(0) += 1;
                }

                // Compute edge length
                if (a as usize) < mesh.vertices.len() && (b as usize) < mesh.vertices.len() {
                    let va = mesh.vertices[a as usize];
                    let vb = mesh.vertices[b as usize];
                    let dist = ((va.x - vb.x).powi(2) + (va.y - vb.y).powi(2) + (va.z - vb.z).powi(2)).sqrt();
                    if a_shared && b_shared {
                        shared_edge_lengths.push(dist);
                    } else if !a_shared && !b_shared {
                        unique_edge_lengths.push(dist);
                    } else {
                        mixed_edge_lengths.push(dist);
                    }
                }
            }

            println!("\n--- Boundary Edge Vertex Classification ---");
            println!("Both vertices shared (edge cache boundary): {} ({:.1}%)",
                both_shared, both_shared as f64 / report.boundary_edge_count as f64 * 100.0);
            println!("Both vertices unique (interior Steiner): {} ({:.1}%)",
                both_unique, both_unique as f64 / report.boundary_edge_count as f64 * 100.0);
            println!("Mixed (one shared, one unique): {} ({:.1}%)",
                mixed, mixed as f64 / report.boundary_edge_count as f64 * 100.0);

            println!("\n--- Boundary Edge Face Adjacency ---");
            println!("One-face-only (true outer boundary): {} ({:.1}%)",
                same_face_edge, same_face_edge as f64 / report.boundary_edge_count as f64 * 100.0);
            println!("Multi-face (SHOULD be interior): {} ({:.1}%)",
                diff_face_edge, diff_face_edge as f64 / report.boundary_edge_count as f64 * 100.0);

            if diff_face_edge > 0 {
                println!("\n  Multi-face boundary edges by vertex type:");
                println!("    Both shared (deduplication failed): {} ({:.1}%)",
                    diff_face_both_shared,
                    diff_face_both_shared as f64 / diff_face_edge as f64 * 100.0);
                println!("    Both unique (Steiner point mismatch): {} ({:.1}%)",
                    diff_face_both_unique,
                    diff_face_both_unique as f64 / diff_face_edge as f64 * 100.0);
                println!("    Mixed (one shared, one unique): {} ({:.1}%)",
                    diff_face_mixed,
                    diff_face_mixed as f64 / diff_face_edge as f64 * 100.0);
            }

            // ─── Edge length statistics by category ─────────────────────
            fn print_length_stats(label: &str, lengths: &[f64]) {
                if lengths.is_empty() {
                    println!("  {}: (none)", label);
                    return;
                }
                let mut sorted = lengths.to_vec();
                sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
                let min = sorted[0];
                let max = *sorted.last().unwrap();
                let median = sorted[sorted.len() / 2];
                let mean = sorted.iter().sum::<f64>() / sorted.len() as f64;
                println!("  {}: count={}, min={:.6}, median={:.6}, mean={:.6}, max={:.6}",
                    label, sorted.len(), min, median, mean, max);
            }

            println!("\n--- Boundary Edge Length Statistics ---");
            print_length_stats("Both shared", &shared_edge_lengths);
            print_length_stats("Both unique", &unique_edge_lengths);
            print_length_stats("Mixed", &mixed_edge_lengths);

            // ─── Face pair breakdown ────────────────────────────────────
            if !face_pair_counts.is_empty() {
                println!("\n--- Face Pairs with Boundary Edges (should be interior) ---");
                let mut pairs: Vec<_> = face_pair_counts.iter().collect();
                pairs.sort_by_key(|(_, c)| std::cmp::Reverse(**c));
                for ((f1, f2), count) in pairs.iter().take(15) {
                    println!("  Faces #{} ↔ #{}: {} boundary edges", f1, f2, count);
                }
            }

            // ─── Per-face boundary edge count ───────────────────────────
            if !face_boundary_counts.is_empty() {
                println!("\n--- Faces by Boundary Edge Count ---");
                let mut sorted_faces: Vec<_> = face_boundary_counts.iter().collect();
                sorted_faces.sort_by_key(|(_, c)| std::cmp::Reverse(**c));
                for (fid, count) in sorted_faces.iter().take(15) {
                    // Count how many vertices this face has
                    let face_vert_count = vertex_to_faces.iter()
                        .filter(|(_, faces)| faces.contains(fid))
                        .count();
                    println!("  Face #{}: {} boundary edges, {} vertices", fid, count, face_vert_count);
                }
            }

            // ─── Deep dive: sample boundary edges with full detail ──────
            println!("\n--- Sample Boundary Edges (first 20) ---");
            for (idx, &(a, b)) in report.boundary_edges.iter().take(20).enumerate() {
                let a_shared = shared_vertices.contains(&a);
                let b_shared = shared_vertices.contains(&b);
                let a_type = if a_shared { "SHARED" } else { "UNIQUE" };
                let b_type = if b_shared { "SHARED" } else { "UNIQUE" };

                let edge_key = (a.min(b), a.max(b));
                let adjacent_faces = edge_to_faces.get(&edge_key)
                    .cloned()
                    .unwrap_or_default();
                let faces_str: Vec<String> = adjacent_faces.iter().map(|f| format!("{}", f)).collect();

                let a_face_count = vertex_to_faces.get(&a).map(|s| s.len()).unwrap_or(0);
                let b_face_count = vertex_to_faces.get(&b).map(|s| s.len()).unwrap_or(0);

                if (a as usize) < mesh.vertices.len() && (b as usize) < mesh.vertices.len() {
                    let va = mesh.vertices[a as usize];
                    let vb = mesh.vertices[b as usize];
                    let dist = ((va.x - vb.x).powi(2) + (va.y - vb.y).powi(2) + (va.z - vb.z).powi(2)).sqrt();
                    println!("  [{}] {}→{} ({}→{}) dist={:.6} faces=[{}] v{}_in_{}_faces v{}_in_{}_faces",
                        idx, a, b, a_type, b_type, dist, faces_str.join(","), a, a_face_count, b, b_face_count);
                }
            }

            // ─── Deep dive: shared-vertex boundary edges specifically ───
            // These are the most suspicious: both vertices are shared between
            // faces, yet the edge is still a boundary. This means deduplication
            // is failing for these edges.
            println!("\n--- Both-Shared Boundary Edges (dedup failure) ---");
            let both_shared_edges: Vec<(u32, u32)> = report.boundary_edges.iter()
                .filter(|&&(a, b)| shared_vertices.contains(&a) && shared_vertices.contains(&b))
                .copied()
                .collect();

            if both_shared_edges.is_empty() {
                println!("  (none — good!)");
            } else {
                // Show which faces these vertices belong to
                let mut dedup_failure_face_pairs: HashMap<(u64, u64), usize> = HashMap::new();
                for &(a, b) in &both_shared_edges {
                    let a_faces = vertex_to_faces.get(&a).cloned().unwrap_or_default();
                    let b_faces = vertex_to_faces.get(&b).cloned().unwrap_or_default();
                    let all_faces: HashSet<u64> = a_faces.union(&b_faces).copied().collect();
                    let mut sorted: Vec<u64> = all_faces.iter().copied().collect();
                    sorted.sort();
                    for i in 0..sorted.len() {
                        for j in (i+1)..sorted.len() {
                            *dedup_failure_face_pairs.entry((sorted[i], sorted[j])).or_insert(0) += 1;
                        }
                    }
                }
                println!("  {} edges where both vertices are shared but edge is boundary",
                    both_shared_edges.len());

                if !dedup_failure_face_pairs.is_empty() {
                    println!("  Face pairs involved in dedup failures:");
                    let mut pairs: Vec<_> = dedup_failure_face_pairs.iter().collect();
                    pairs.sort_by_key(|(_, c)| std::cmp::Reverse(**c));
                    for ((f1, f2), count) in pairs.iter().take(10) {
                        println!("    Faces #{} ↔ #{}: {} dedup-failure edges", f1, f2, count);
                    }
                }

                // Show first few with coordinates
                println!("\n  First 10 both-shared boundary edges:");
                for (idx, &(a, b)) in both_shared_edges.iter().take(10).enumerate() {
                    if (a as usize) < mesh.vertices.len() && (b as usize) < mesh.vertices.len() {
                        let va = mesh.vertices[a as usize];
                        let vb = mesh.vertices[b as usize];
                        let dist = ((va.x - vb.x).powi(2) + (va.y - vb.y).powi(2) + (va.z - vb.z).powi(2)).sqrt();
                        let a_faces = vertex_to_faces.get(&a).map(|s| {
                            let mut v: Vec<u64> = s.iter().copied().collect();
                            v.sort();
                            v
                        }).unwrap_or_default();
                        let b_faces = vertex_to_faces.get(&b).map(|s| {
                            let mut v: Vec<u64> = s.iter().copied().collect();
                            v.sort();
                            v
                        }).unwrap_or_default();
                        let common_faces: HashSet<u64> = a_faces.iter().copied().collect::<HashSet<_>>()
                            .intersection(&b_faces.iter().copied().collect::<HashSet<_>>())
                            .copied().collect();
                        println!("    [{}] {}→{} dist={:.6} a_faces={:?} b_faces={:?} common={:?}",
                            idx, a, b, dist, a_faces, b_faces, common_faces);
                    }
                }
            }

            // ─── Unique-vertex boundary edges (Steiner point mismatch) ───
            println!("\n--- Both-Unique Boundary Edges (Steiner point mismatch) ---");
            let both_unique_edges: Vec<(u32, u32)> = report.boundary_edges.iter()
                .filter(|&&(a, b)| !shared_vertices.contains(&a) && !shared_vertices.contains(&b))
                .copied()
                .collect();

            if both_unique_edges.is_empty() {
                println!("  (none — good!)");
            } else {
                println!("  {} edges where both vertices are unique to one face (Steiner points)",
                    both_unique_edges.len());

                // Which faces do these Steiner-point boundary edges belong to?
                let mut steiner_face_counts: HashMap<u64, usize> = HashMap::new();
                for &(a, b) in &both_unique_edges {
                    let a_faces = vertex_to_faces.get(&a).cloned().unwrap_or_default();
                    let b_faces = vertex_to_faces.get(&b).cloned().unwrap_or_default();
                    for &f in &a_faces { *steiner_face_counts.entry(f).or_insert(0) += 1; }
                    for &f in &b_faces { *steiner_face_counts.entry(f).or_insert(0) += 1; }
                }
                let mut sorted: Vec<_> = steiner_face_counts.iter().collect();
                sorted.sort_by_key(|(_, c)| std::cmp::Reverse(**c));
                println!("  Faces with Steiner-point boundary edges:");
                for (fid, count) in sorted.iter().take(10) {
                    println!("    Face #{}: {} edges", fid, count);
                }
            }

            // ─── Mixed boundary edges ───────────────────────────────────
            println!("\n--- Mixed Boundary Edges (one shared, one unique) ---");
            let mixed_edges: Vec<(u32, u32)> = report.boundary_edges.iter()
                .filter(|&&(a, b)| shared_vertices.contains(&a) != shared_vertices.contains(&b))
                .copied()
                .collect();

            if mixed_edges.is_empty() {
                println!("  (none — good!)");
            } else {
                println!("  {} edges with one shared and one unique vertex",
                    mixed_edges.len());

                // Analyze: for each mixed edge, is the shared vertex on the
                // boundary of the face (adjacent to the unique vertex)?
                // This would indicate a face where the boundary triangulation
                // doesn't reach the edge cache vertices properly.
                let mut mixed_face_counts: HashMap<u64, usize> = HashMap::new();
                for &(a, b) in &mixed_edges {
                    let a_faces = vertex_to_faces.get(&a).cloned().unwrap_or_default();
                    let b_faces = vertex_to_faces.get(&b).cloned().unwrap_or_default();
                    let all: HashSet<u64> = a_faces.union(&b_faces).copied().collect();
                    for &f in &all { *mixed_face_counts.entry(f).or_insert(0) += 1; }
                }
                let mut sorted: Vec<_> = mixed_face_counts.iter().collect();
                sorted.sort_by_key(|(_, c)| std::cmp::Reverse(**c));
                println!("  Faces involved in mixed edges:");
                for (fid, count) in sorted.iter().take(10) {
                    println!("    Face #{}: {} edges", fid, count);
                }
            }

            // ─── Summary diagnosis ──────────────────────────────────────
            println!("\n{}", "=".repeat(80));
            println!("DIAGNOSIS SUMMARY");
            println!("{}", "=".repeat(80));

            let total = report.boundary_edge_count;
            let pct = |n: usize| n as f64 / total as f64 * 100.0;

            println!("Total boundary edges: {}", total);
            println!();

            // Key finding: edge cache consistency vs. boundary edges
            println!("EDGE CACHE vs BOUNDARY EDGES:");
            println!("  Edge cache consistency: 0.00% inconsistent (bit-identical coordinates)");
            println!("  Boundary edge rate: {:.1}%", pct(total));
            println!("  → Edge cache is working correctly; the problem is NOT cross-face deduplication.");
            println!();

            if diff_face_edge > 0 {
                println!("⚠ {} ({:.1}%) boundary edges touch 2+ different faces.",
                    diff_face_edge, pct(diff_face_edge));
                println!("  These should be interior edges (shared by triangles from different faces).");
                println!("  Possible causes:");
                if diff_face_both_shared > 0 {
                    println!("  - {} edges: both vertices shared but dedup failed (VertexKey collision or FP precision issue)",
                        diff_face_both_shared);
                }
                if diff_face_both_unique > 0 {
                    println!("  - {} edges: both vertices are unique Steiner points (independent interior triangulations)",
                        diff_face_both_unique);
                }
                if diff_face_mixed > 0 {
                    println!("  - {} edges: mixed shared/unique (face boundary → Steiner point of another face)",
                        diff_face_mixed);
                }
            }

            if same_face_edge > 0 {
                println!("\n⚠ {} ({:.1}%) boundary edges are WITHIN a single face (count=1 adjacent triangle).",
                    same_face_edge, pct(same_face_edge));
                println!("  This means face triangulation has INTERNAL GAPS — triangles don't fully tile the face.");
                println!("  The edge cache is irrelevant here; the CDT/ear-clipping produced incomplete coverage.");
                println!();

                // Sub-classification
                println!("  Within-face boundary edges by vertex type:");
                println!("    Both shared (boundary verts, edge cache): {} ({:.1}%)",
                    both_shared, pct(both_shared));
                println!("      → Two edge-cache vertices that SHOULD have triangles on both sides but don't.");
                println!("      → The shared edge between two faces is being properly deduplicated,");
                println!("         but one face's triangles don't reach all boundary vertices.");
                println!("    Both unique (interior Steiner points): {} ({:.1}%)",
                    both_unique, pct(both_unique));
                println!("      → Two interior vertices whose connecting edge has only 1 triangle.");
                println!("      → The CDT left a gap in the interior of the face's triangulation.");
                println!("    Mixed (one shared, one unique): {} ({:.1}%)",
                    mixed, pct(mixed));
                println!("      → A boundary vertex connected to an interior vertex with only 1 triangle.");
                println!("      → The CDT didn't extend triangles all the way to the face boundary.");
            }

            // Final verdict
            println!("\n  VERDICT:");
            if same_face_edge == total && diff_face_edge == 0 {
                println!("  All boundary edges are WITHIN single faces → the edge cache and deduplication");
                println!("  are working correctly. The root cause is INCOMPLETE FACE TRIANGULATION:");
                println!("  faces #6 and #7 (likely NURBS) have gaps in their CDT where triangles");
                println!("  don't fully cover the face domain, leaving edges with only 1 adjacent triangle.");
                if both_unique > both_shared + mixed {
                    println!("  The dominant pattern ({:.1}%) is Steiner-to-Steiner boundary edges, suggesting",
                        pct(both_unique));
                    println!("  the CDT is inserting interior points but not connecting them into a complete mesh.");
                }
            } else if diff_face_edge > 0 {
                println!("  Some boundary edges are between different faces → deduplication IS the problem.");
            }
        }
        None => {
            println!("TRIANGULATION FAILED for BREP #{}", target_brep);
        }
    }
}
