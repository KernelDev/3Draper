// Diagnostic: investigate sphere triangulation boundary edges
// Run: cargo run --release --bin sphere_diag

use draper_step::{parse_step_file, extract_solids};
use draper_mesh::triangulate_solid_with_report;
use draper_mesh::triangulate::TriangulationParams;

fn main() {
    let path = std::env::args().nth(1).unwrap_or_else(||
        "test/synthetic/synth_sphere.stp".to_string()
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
                    draper_geometry::Surface::Plane(_) => "Plane",
                    draper_geometry::Surface::Sphere(_) => "Sphere",
                    draper_geometry::Surface::Cylinder(_) => "Cylinder",
                    draper_geometry::Surface::Cone(_) => "Cone",
                    draper_geometry::Surface::Torus(_) => "Torus",
                    draper_geometry::Surface::Nurbs(_) => "Nurbs",
                    draper_geometry::Surface::Revolution(_) => "Revolution",
                    draper_geometry::Surface::Extrusion(_) => "Extrusion",
                    _ => "Other",
                }
            }).unwrap_or("None");
            let n_edges = face.edges.len();
            let n_inner_wires = face.inner_wires.len();
            let has_outer = face.outer_wire.is_some();
            let n_coedges = face.outer_wire.as_ref().map(|w| w.coedges.len()).unwrap_or(0);
            println!("  Face {}: surface={}, edges={}, outer_wire={}, inner_wires={}, coedges={}",
                fi, surf_name, n_edges, has_outer, n_inner_wires, n_coedges);
        }

        // Triangulate just this solid and report
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
        println!("  Euler characteristic: {} (should be 2 for genus 0)", r.euler_characteristic);
        println!("  Watertight: {}", r.is_watertight);
    }
}
