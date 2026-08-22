//! Diagnostic test for boolean subtract (Box - Cylinder).
//!
//! This test reproduces the exact VP pipeline that was reported broken:
//! Box(100,80,50) - Cylinder(R=20, H=100) should produce a box with a
//! cylindrical hole through it — a watertight closed solid.

use draper_geometry::ToleranceContext;
use draper_topology::builder::ShapeBuilder;
use draper_topology::boolean::{boolean_subtract, boolean_union, boolean_intersect};

#[test]
fn test_box_minus_cylinder_subtract() {
    eprintln!("\n=== Test: Box(100,80,50) - Cylinder(R=20, H=100) ===");

    let box_solid = ShapeBuilder::make_box(100.0, 80.0, 50.0);
    let cyl_solid = ShapeBuilder::make_cylinder(20.0, 100.0);

    eprintln!("Box faces: {}", box_solid.faces().len());
    eprintln!("Cylinder faces: {}", cyl_solid.faces().len());

    // Use model-scale tolerance (box diagonal ~133)
    let tol_ctx = ToleranceContext::from_model_scale(133.0);
    eprintln!("ToleranceContext: coincidence={:e}", tol_ctx.coincidence_tolerance());

    match boolean_subtract(&box_solid, &cyl_solid, &tol_ctx) {
        Ok(result) => {
            let result_faces = result.faces();
            eprintln!("SUCCESS: result has {} faces", result_faces.len());

            if let Some(ref shell) = result.outer_shell {
                eprintln!("Shell is_closed flag: {}", shell.closed);
            }

            for (i, face) in result_faces.iter().enumerate() {
                let has_surface = face.surface.is_some();
                let edge_count = face.edges.len();
                let has_outer_wire = face.outer_wire.is_some();
                eprintln!("  Face[{}]: surface={}, edges={}, outer_wire={}",
                    i, has_surface, edge_count, has_outer_wire);
            }

            // Triangulate and check watertightness
            let params = draper_mesh::TriangulationParams::default();
            let mesh = draper_mesh::triangulate_solid(&result, &params);
            eprintln!("Mesh: {} vertices, {} triangles",
                mesh.vertices.len(), mesh.triangles.len());

            let edges = count_edges(&mesh);
            let boundary_count = edges.iter().filter(|(_, &c)| c != 2).count();
            eprintln!("Boundary edges (count != 2): {}", boundary_count);
            if boundary_count > 0 {
                eprintln!("  NOT WATERTIGHT!");
                let samples: Vec<_> = edges.iter().filter(|(_, &c)| c != 2).take(5).collect();
                for (edge, count) in samples {
                    eprintln!("    {:?} → count={}", edge, count);
                }

                // Find the actual distance between boundary vertex pairs
                // to understand the scale of the gap
                let boundary_pairs: Vec<_> = edges.iter()
                    .filter(|(_, &c)| c != 2)
                    .map(|(&[i, j], &c)| (i, j, c))
                    .collect();
                let mut min_dist = f64::MAX;
                let mut max_dist = 0.0f64;
                for (i, _, _) in &boundary_pairs {
                    for (j, _, _) in &boundary_pairs {
                        if i >= j { continue; }
                        let pi = &mesh.vertices[*i as usize];
                        let pj = &mesh.vertices[*j as usize];
                        let d = ((pi.x - pj.x).powi(2) + (pi.y - pj.y).powi(2) + (pi.z - pj.z).powi(2)).sqrt();
                        if d > 0.0 && d < min_dist { min_dist = d; }
                        if d > max_dist { max_dist = d; }
                    }
                }
                eprintln!("  Boundary vertex distances: min={:.2e}, max={:.2e}", min_dist, max_dist);
                eprintln!("  (if min > 0 and small, welding with that tolerance would help)");

                // Try welding with a larger tolerance
                let weld_tol = 1e-3; // 1mm — much larger than default
                eprintln!("\n  Trying weld_boundary_edge_vertices_aggressive(tol={})...", weld_tol);
                let mut mesh2 = mesh.clone();
                draper_mesh::watertight::weld_boundary_edge_vertices_aggressive(&mut mesh2, weld_tol);
                let edges2 = count_edges(&mesh2);
                let boundary2 = edges2.iter().filter(|(_, &c)| c != 2).count();
                eprintln!("  After weld: {} boundary edges (was {})", boundary2, boundary_count);

                // Try fill_boundary_gaps
                eprintln!("\n  Trying fill_boundary_gaps(max_loop=64)...");
                let filled = draper_mesh::watertight::fill_boundary_gaps(&mut mesh2, 64);
                let edges3 = count_edges(&mesh2);
                let boundary3 = edges3.iter().filter(|(_, &c)| c != 2).count();
                eprintln!("  After fill: {} boundary edges (filled {} gaps)", boundary3, filled);

                // Combined: weld with model-scale tol + fill
                eprintln!("\n  Trying combined: weld(scale*1e-3) + fill...");
                let scale = 133.0; // box diagonal
                let mut mesh3 = mesh.clone();
                draper_mesh::watertight::weld_boundary_edge_vertices_aggressive(&mut mesh3, scale * 1e-3);
                let filled3 = draper_mesh::watertight::fill_boundary_gaps(&mut mesh3, 64);
                let edges4 = count_edges(&mesh3);
                let boundary4 = edges4.iter().filter(|(_, &c)| c != 2).count();
                eprintln!("  After combined: {} boundary edges (filled {} gaps)", boundary4, filled3);

                // Aggressive: weld with large tolerance (5 units) + fill
                eprintln!("\n  Trying aggressive: weld(5.0) + fill...");
                let mut mesh4 = mesh.clone();
                draper_mesh::watertight::weld_boundary_edge_vertices_aggressive(&mut mesh4, 5.0);
                let filled4 = draper_mesh::watertight::fill_boundary_gaps(&mut mesh4, 256);
                let edges5 = count_edges(&mesh4);
                let boundary5 = edges5.iter().filter(|(_, &c)| c != 2).count();
                eprintln!("  After aggressive: {} boundary edges (filled {} gaps), {} triangles",
                    boundary5, filled4, mesh4.triangles.len());

                // Multi-pass with moderate tolerance
                eprintln!("\n  Trying multi-pass: weld(2.5)+fill+weld(2.0)+fill...");
                let mut mesh5 = mesh.clone();
                for (pass, wt) in [2.5_f64, 2.0, 1.5, 1.0, 0.5].iter().enumerate() {
                    draper_mesh::watertight::weld_boundary_edge_vertices_aggressive(&mut mesh5, *wt);
                    let f = draper_mesh::watertight::fill_boundary_gaps(&mut mesh5, 256);
                    let e = count_edges(&mesh5);
                    let b = e.iter().filter(|(_, &c)| c != 2).count();
                    eprintln!("  Pass {} (wt={}): {} boundary edges (filled {} gaps), {} triangles",
                        pass, wt, b, f, mesh5.triangles.len());
                    if b == 0 || f == 0 { break; }
                }

                // Multi-pass with moderate tolerance, decreasing
                eprintln!("\n  Trying multi-pass: weld(scale*0.02)+fill repeated...");
                let mut mesh6 = mesh.clone();
                let wt = 133.0 * 0.02; // ~2.66
                for pass in 0..5 {
                    draper_mesh::watertight::weld_boundary_edge_vertices_aggressive(&mut mesh6, wt);
                    let f = draper_mesh::watertight::fill_boundary_gaps(&mut mesh6, 256);
                    let e = count_edges(&mesh6);
                    let b = e.iter().filter(|(_, &c)| c != 2).count();
                    eprintln!("  Pass {} (wt={:.2}): {} boundary edges (filled {} gaps), {} triangles",
                        pass, wt, b, f, mesh6.triangles.len());
                    if b == 0 { eprintln!("  ✓ WATERTIGHT after {} passes", pass + 1); break; }
                    if f == 0 && pass > 0 { break; }
                }
            } else {
                eprintln!("  ✓ WATERTIGHT");
            }
        }
        Err(e) => {
            eprintln!("FAILED: {:?}", e);
            panic!("boolean_subtract failed: {:?}", e);
        }
    }
}

#[test]
fn test_box_minus_cylinder_default_tolerance() {
    eprintln!("\n=== Test: default tolerance (current VP code path) ===");

    let box_solid = ShapeBuilder::make_box(100.0, 80.0, 50.0);
    let cyl_solid = ShapeBuilder::make_cylinder(20.0, 100.0);

    let tol_ctx = ToleranceContext::default();
    eprintln!("Default ToleranceContext: coincidence={:e}", tol_ctx.coincidence_tolerance());

    match boolean_subtract(&box_solid, &cyl_solid, &tol_ctx) {
        Ok(result) => {
            eprintln!("SUCCESS with default tolerance: {} faces", result.faces().len());
        }
        Err(e) => {
            eprintln!("FAILED with default tolerance: {:?}", e);
        }
    }
}

#[test]
fn test_union_and_intersect_are_not_stubs() {
    eprintln!("\n=== Test: Union and Intersect actually compute ===");

    let box_a = ShapeBuilder::make_box(40.0, 40.0, 40.0);
    let box_b = ShapeBuilder::make_box_at(20.0, 0.0, 0.0, 40.0, 40.0, 40.0);

    let tol_ctx = ToleranceContext::from_model_scale(80.0);

    match boolean_union(&box_a, &box_b, &tol_ctx) {
        Ok(result) => {
            let face_count = result.faces().len();
            eprintln!("Union result: {} faces (two boxes=12, merged should be ≤10)", face_count);

            // Check watertightness
            let params = draper_mesh::TriangulationParams::default();
            let mesh = draper_mesh::triangulate_solid(&result, &params);
            let edges = count_edges(&mesh);
            let boundary = edges.iter().filter(|(_, &c)| c != 2).count();
            eprintln!("  Union mesh: {} triangles, {} boundary edges",
                mesh.triangles.len(), boundary);
        }
        Err(e) => eprintln!("Union failed: {:?}", e),
    }

    match boolean_intersect(&box_a, &box_b, &tol_ctx) {
        Ok(result) => {
            let face_count = result.faces().len();
            eprintln!("Intersect result: {} faces (overlap=20×40×40 box=6 faces)", face_count);

            let params = draper_mesh::TriangulationParams::default();
            let mesh = draper_mesh::triangulate_solid(&result, &params);
            let edges = count_edges(&mesh);
            let boundary = edges.iter().filter(|(_, &c)| c != 2).count();
            eprintln!("  Intersect mesh: {} triangles, {} boundary edges",
                mesh.triangles.len(), boundary);
        }
        Err(e) => eprintln!("Intersect failed: {:?}", e),
    }
}

#[test]
fn test_box_minus_box_subtract() {
    eprintln!("\n=== Test: Box(100,100,100) - Box(30,30,30) at center ===");

    let box_a = ShapeBuilder::make_box(100.0, 100.0, 100.0);
    let box_b = ShapeBuilder::make_box(30.0, 30.0, 30.0);

    let tol_ctx = ToleranceContext::from_model_scale(173.0);

    match boolean_subtract(&box_a, &box_b, &tol_ctx) {
        Ok(result) => {
            eprintln!("SUCCESS: {} faces", result.faces().len());

            let params = draper_mesh::TriangulationParams::default();
            let mesh = draper_mesh::triangulate_solid(&result, &params);
            let edges = count_edges(&mesh);
            let boundary = edges.iter().filter(|(_, &c)| c != 2).count();
            eprintln!("  Mesh: {} triangles, {} boundary edges",
                mesh.triangles.len(), boundary);

            if boundary > 0 {
                // Try repair
                let mut mesh2 = mesh.clone();
                let scale = 173.0;
                for (pass, wt) in [scale * 0.02, scale * 0.01, scale * 0.005].iter().enumerate() {
                    draper_mesh::watertight::weld_boundary_edge_vertices_aggressive(&mut mesh2, *wt);
                    for _ in 0..3 {
                        let f = draper_mesh::watertight::fill_boundary_gaps(&mut mesh2, 512);
                        if f == 0 { break; }
                    }
                    let e = count_edges(&mesh2);
                    let b = e.iter().filter(|(_, &c)| c != 2).count();
                    eprintln!("  Repair pass {} (wt={:.2}): {} boundary edges, {} triangles",
                        pass, wt, b, mesh2.triangles.len());
                    if b == 0 { eprintln!("  ✓ WATERTIGHT after repair"); break; }
                }
            }
        }
        Err(e) => eprintln!("FAILED: {:?}", e),
    }
}

/// Count edges in a triangle mesh and return a map of edge → occurrence count.
fn count_edges(mesh: &draper_mesh::TriangleMesh) -> std::collections::HashMap<[u32; 2], u32> {
    let mut edges: std::collections::HashMap<[u32; 2], u32> = std::collections::HashMap::new();
    for tri in &mesh.triangles {
        let a = tri[0];
        let b = tri[1];
        let c = tri[2];
        for (i, j) in [(a, b), (b, c), (c, a)] {
            let key = if i < j { [i, j] } else { [j, i] };
            *edges.entry(key).or_insert(0) += 1;
        }
    }
    edges
}
