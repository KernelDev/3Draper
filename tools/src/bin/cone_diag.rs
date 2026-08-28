// Diagnostic: dump cone face triangulation details
// Run: cargo run --release --bin cone_diag -- test/nist_cone.stp

use draper_step::{parse_step_file, extract_solids};
use draper_mesh::triangulate_solid_with_report;
use draper_mesh::triangulate::TriangulationParams;
use draper_geometry::Surface;

fn main() {
    let path = std::env::args().nth(1).unwrap_or_else(||
        "test/nist_cone.stp".to_string()
    );
    println!("Loading: {}", path);
    let step = match parse_step_file(&path) {
        Ok(s) => s,
        Err(e) => { eprintln!("Parse error: {}", e); return; }
    };
    let (solids, _) = extract_solids(&step);
    println!("Solids: {}", solids.len());

    for (si, solid) in solids.iter().enumerate() {
        println!("\n=== Solid #{} ===", si);
        println!("Faces: {}", solid.faces().len());

        for (fi, face) in solid.faces().iter().enumerate() {
            let surf_name = face.surface.as_ref().map(|s| {
                match s {
                    Surface::Plane(_) => "Plane",
                    Surface::Sphere(_) => "Sphere",
                    Surface::Cylinder(_) => "Cylinder",
                    Surface::Cone(_) => "Cone",
                    Surface::Torus(_) => "Torus",
                    _ => "Other",
                }
            }).unwrap_or("None");
            println!("\nFace {}: surface={}, edges={}", fi, surf_name, face.edges.len());

            if let Some(Surface::Cone(cone)) = &face.surface {
                println!("  Cone: origin=({:.4},{:.4},{:.4}) axis=({:.4},{:.4},{:.4})",
                    cone.origin.x, cone.origin.y, cone.origin.z,
                    cone.axis.x, cone.axis.y, cone.axis.z);
                println!("  Cone: radius={} half_angle={} expanding={}",
                    cone.radius, cone.half_angle, cone.expanding);
                println!("  Cone: apex_v={} height={}", cone.apex_v(), cone.height());

                // Print edge details
                for (ei, edge) in face.edges.iter().enumerate() {
                    let edge_type = edge.curve.as_ref().map(|c| {
                        match c {
                            draper_geometry::Curve3d::Line(_) => "Line",
                            draper_geometry::Curve3d::Circle(_) => "Circle",
                            draper_geometry::Curve3d::Nurbs(_) => "Nurbs",
                            _ => "Other",
                        }
                    }).unwrap_or("None");
                    let sp = edge.start_vertex_point;
                    let ep = edge.end_vertex_point;
                    println!("    Edge {}: type={} param_range=({:.4},{:.4}) start={:?} end={:?}",
                        ei, edge_type, edge.param_range.0, edge.param_range.1, sp, ep);
                }
            }
        }

        // Triangulate solid
        let params = TriangulationParams::default();
        let result = triangulate_solid_with_report(solid, &params);
        let r = result.report;
        println!("\nSolid #{} triangulation:", si);
        println!("  Vertices: {}", r.vertex_count);
        println!("  Triangles: {}", r.triangle_count);
        println!("  Edges: {}", r.edge_count);
        println!("  Boundary edges: {} ({:.2}%)", r.boundary_edge_count, r.boundary_pct);
        println!("  Non-manifold edges: {}", r.non_manifold_edge_count);
        println!("  Degenerate triangles: {}", r.degenerate_triangle_count);
        println!("  Euler characteristic: {}", r.euler_characteristic);
        println!("  Watertight: {}", r.is_watertight);

        // Check vertex distances — find min distance between distinct vertices
        let mesh = &result.mesh;
        if mesh.vertices.len() > 1 {
            let mut min_dist_sq = f64::MAX;
            let mut min_pair = (0, 0);
            for i in 0..mesh.vertices.len() {
                for j in (i+1)..mesh.vertices.len() {
                    let dx = mesh.vertices[i].x - mesh.vertices[j].x;
                    let dy = mesh.vertices[i].y - mesh.vertices[j].y;
                    let dz = mesh.vertices[i].z - mesh.vertices[j].z;
                    let d_sq = dx*dx + dy*dy + dz*dz;
                    if d_sq < min_dist_sq {
                        min_dist_sq = d_sq;
                        min_pair = (i, j);
                    }
                }
            }
            let min_dist = min_dist_sq.sqrt();
            println!("  Min vertex distance: {:.6e} (between vertex {} and {})", min_dist, min_pair.0, min_pair.1);
            println!("  V[{}]: {:?}", min_pair.0, mesh.vertices[min_pair.0]);
            println!("  V[{}]: {:?}", min_pair.1, mesh.vertices[min_pair.1]);
        }

        // Print first few boundary edges with vertex coords
        let report_w = draper_mesh::watertight::validate_watertight(mesh, true);
        if !report_w.boundary_edges.is_empty() {
            println!("\n  First 5 boundary edges (vertex pairs):");
            for (i, (v0, v1)) in report_w.boundary_edges.iter().take(5).enumerate() {
                let p0 = &mesh.vertices[*v0 as usize];
                let p1 = &mesh.vertices[*v1 as usize];
                println!("    {}: ({},{}) = ({:.4},{:.4},{:.4}) - ({:.4},{:.4},{:.4})",
                    i, v0, v1, p0.x, p0.y, p0.z, p1.x, p1.y, p1.z);
            }
        }
    }
}
