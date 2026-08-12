//! Diagnostic test for boolean subtract — vertex comparison.
//! Compares the cylinder face's boundary vertices with the plane face's
//! hole vertices to identify the mismatch.

use draper_geometry::ToleranceContext;
use draper_topology::builder::ShapeBuilder;
use draper_topology::boolean::boolean_subtract;

#[test]
fn test_cylinder_plane_vertex_match() {
    eprintln!("\n=== Vertex Match Test ===");

    let box_solid = ShapeBuilder::make_box(100.0, 80.0, 50.0);
    let cyl_solid = ShapeBuilder::make_cylinder(20.0, 100.0);
    let tol_ctx = ToleranceContext::from_model_scale(133.0);

    let result = boolean_subtract(&box_solid, &cyl_solid, &tol_ctx).unwrap();
    eprintln!("Result: {} faces", result.faces().len());

    // Print each face's surface type and edge count
    for (i, face) in result.faces().iter().enumerate() {
        let surf_type = match &face.surface {
            Some(draper_geometry::Surface::Plane(_)) => "Plane",
            Some(draper_geometry::Surface::Cylinder(_)) => "Cylinder",
            _ => "Other",
        };
        let edge_count = face.edges.len();
        let has_outer = face.outer_wire.is_some();
        let hole_count = face.inner_wires.len();
        eprintln!("  Face[{}]: {} (edges={}, outer={}, holes={})",
            i, surf_type, edge_count, has_outer, hole_count);

        // Print edge curve types
        for (j, edge) in face.edges.iter().enumerate() {
            let curve_type = match &edge.curve {
                Some(draper_geometry::Curve3d::Circle(_)) => "Circle",
                Some(draper_geometry::Curve3d::Line(_)) => "Line",
                Some(draper_geometry::Curve3d::Ellipse(_)) => "Ellipse",
                Some(_) => "Other",
                None => "None",
            };
            eprintln!("    Edge[{}]: {} param_range={:?}", j, curve_type, edge.param_range);
        }
    }

    // Triangulate and check watertightness
    let params = draper_mesh::TriangulationParams::default();
    let mesh = draper_mesh::triangulate_solid(&result, &params);
    eprintln!("\nMesh: {} vertices, {} triangles", mesh.vertices.len(), mesh.triangles.len());

    // Count boundary edges
    let mut edges: std::collections::HashMap<[u32; 2], u32> = std::collections::HashMap::new();
    for tri in &mesh.triangles {
        for (i, j) in [(0, 1), (1, 2), (2, 0)] {
            let key = if tri[i] < tri[j] { [tri[i], tri[j]] } else { [tri[j], tri[i]] };
            *edges.entry(key).or_insert(0) += 1;
        }
    }
    let boundary: Vec<_> = edges.iter().filter(|(_, &c)| c != 2).collect();
    eprintln!("Boundary edges: {}", boundary.len());

    // Find boundary vertex pairs and their distances
    if !boundary.is_empty() {
        // Get unique boundary vertices
        let mut bverts: std::collections::HashSet<u32> = std::collections::HashSet::new();
        for (&[a, b], _) in &boundary {
            bverts.insert(a);
            bverts.insert(b);
        }
        let bverts: Vec<u32> = bverts.into_iter().collect();
        eprintln!("\n{} unique boundary vertices", bverts.len());

        // For each boundary vertex, find the nearest OTHER boundary vertex
        eprintln!("\nNearest neighbor distances (first 15):");
        for i in 0..bverts.len().min(15) {
            let p = &mesh.vertices[bverts[i] as usize];
            let mut min_d = f64::MAX;
            let mut min_j = 0;
            for j in 0..bverts.len() {
                if i == j { continue; }
                let q = &mesh.vertices[bverts[j] as usize];
                let d = ((p.x - q.x).powi(2) + (p.y - q.y).powi(2) + (p.z - q.z).powi(2)).sqrt();
                if d < min_d {
                    min_d = d;
                    min_j = j;
                }
            }
            let q = &mesh.vertices[bverts[min_j] as usize];
            eprintln!("  v[{}] ({:.3},{:.3},{:.3}) → nearest v[{}] ({:.3},{:.3},{:.3}) dist={:.6}",
                bverts[i], p.x, p.y, p.z, bverts[min_j], q.x, q.y, q.z, min_d);
        }

        // Check if boundary vertices come from different faces
        // by looking at their positions — cylinder vertices have x²+y²≈R²
        eprintln!("\nVertex classification (cylinder R=20, so x²+y²≈400):");
        let mut on_cyl = 0;
        let mut on_plane = 0;
        for &v in &bverts {
            let p = &mesh.vertices[v as usize];
            let r_sq = p.x * p.x + p.y * p.y;
            if (r_sq - 400.0).abs() < 5.0 {
                on_cyl += 1;
            } else {
                on_plane += 1;
            }
        }
        eprintln!("  On cylinder surface: {}", on_cyl);
        eprintln!("  Not on cylinder: {}", on_plane);
    }
}
