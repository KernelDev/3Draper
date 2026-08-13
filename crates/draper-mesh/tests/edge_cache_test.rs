//! Test: verify that cylinder and plane faces get the same number of
//! boundary points from the edge cache for the same shared Circle edge.

use draper_geometry::ToleranceContext;
use draper_topology::builder::ShapeBuilder;
use draper_topology::boolean::boolean_subtract;

#[test]
fn test_edge_cache_consistency() {
    eprintln!("\n=== Edge Cache Consistency Test ===");

    let box_solid = ShapeBuilder::make_box(100.0, 80.0, 50.0);
    let cyl_solid = ShapeBuilder::make_cylinder(20.0, 100.0);
    let tol_ctx = ToleranceContext::from_model_scale(133.0);

    let result = boolean_subtract(&box_solid, &cyl_solid, &tol_ctx).unwrap();

    // Find the cylinder face and the plane face with the shared hole
    let mut cyl_face = None;
    let mut plane_face_with_hole_123 = None;
    let mut plane_face_with_hole_124 = None;

    let faces = result.faces();
    for face in &faces {
        // Check if this face has a Circle edge with TopoId(123) or TopoId(124)
        for edge in &face.edges {
            let id_str = format!("{:?}", edge.id);
            let is_cyl = matches!(face.surface, Some(draper_geometry::Surface::Cylinder(_)));
            let is_plane = matches!(face.surface, Some(draper_geometry::Surface::Plane(_)));
            if id_str.contains("123") {
                if is_cyl { cyl_face = Some(face); }
                else if is_plane { plane_face_with_hole_123 = Some(face); }
            }
            if id_str.contains("124") {
                if is_plane { plane_face_with_hole_124 = Some(face); }
            }
        }
    }

    // Triangulate and check vertex counts
    let params = draper_mesh::TriangulationParams::default();
    let mesh = draper_mesh::triangulate_solid(&result, &params);
    
    // Count vertices on the cylinder surface (z≈0 or z≈±25, x²+y²≈400)
    let mut on_circle_z0 = 0;
    let mut on_circle_z25 = 0;
    let mut on_circle_zn25 = 0;
    for v in &mesh.vertices {
        let r_sq = v.x * v.x + v.y * v.y;
        if (r_sq - 400.0).abs() < 5.0 {
            if v.z.abs() < 1.0 { on_circle_z0 += 1; }
            else if (v.z - 25.0).abs() < 1.0 { on_circle_z25 += 1; }
            else if (v.z + 25.0).abs() < 1.0 { on_circle_zn25 += 1; }
        }
    }
    eprintln!("Vertices on circle at z=0: {}", on_circle_z0);
    eprintln!("Vertices on circle at z=25: {}", on_circle_z25);
    eprintln!("Vertices on circle at z=-25: {}", on_circle_zn25);
    eprintln!("Total mesh vertices: {}", mesh.vertices.len());
    eprintln!("Total triangles: {}", mesh.triangles.len());

    // Check if the cylinder face has an outer_wire with coedges
    if let Some(cyl) = cyl_face {
        eprintln!("\nCylinder face:");
        eprintln!("  edges: {}", cyl.edges.len());
        if let Some(ref wire) = cyl.outer_wire {
            eprintln!("  outer_wire coedges: {}", wire.coedges.len());
            for (i, coedge) in wire.coedges.iter().enumerate() {
                eprintln!("    coedge[{}]: edge={:?} forward={}", i, coedge.edge, coedge.forward);
            }
        }
        for (i, edge) in cyl.edges.iter().enumerate() {
            eprintln!("  edge[{}]: id={:?}", i, edge.id);
        }
    }

    // Check boundary edges
    let mut edges: std::collections::HashMap<[u32; 2], u32> = std::collections::HashMap::new();
    for tri in &mesh.triangles {
        for (i, j) in [(0, 1), (1, 2), (2, 0)] {
            let key = if tri[i] < tri[j] { [tri[i], tri[j]] } else { [tri[j], tri[i]] };
            *edges.entry(key).or_insert(0) += 1;
        }
    }
    let boundary: Vec<_> = edges.iter().filter(|(_, &c)| c != 2).collect();
    eprintln!("\nBoundary edges: {}", boundary.len());
    
    // Count boundary edges on each circle
    let mut bnd_z0 = 0;
    let mut bnd_z25 = 0;
    let mut bnd_zn25 = 0;
    let mut bnd_other = 0;
    for (&[a, b], _) in &boundary {
        let pa = &mesh.vertices[a as usize];
        let pb = &mesh.vertices[b as usize];
        let r_a = pa.x * pa.x + pa.y * pa.y;
        let r_b = pb.x * pb.x + pb.y * pb.y;
        if (r_a - 400.0).abs() < 5.0 && (r_b - 400.0).abs() < 5.0 {
            if pa.z.abs() < 1.0 && pb.z.abs() < 1.0 { bnd_z0 += 1; }
            else if (pa.z - 25.0).abs() < 1.0 && (pb.z - 25.0).abs() < 1.0 { bnd_z25 += 1; }
            else if (pa.z + 25.0).abs() < 1.0 && (pb.z + 25.0).abs() < 1.0 { bnd_zn25 += 1; }
            else { bnd_other += 1; }
        } else {
            bnd_other += 1;
        }
    }
    eprintln!("  Boundary on circle z=0: {}", bnd_z0);
    eprintln!("  Boundary on circle z=25: {}", bnd_z25);
    eprintln!("  Boundary on circle z=-25: {}", bnd_zn25);
    eprintln!("  Boundary other: {}", bnd_other);
}
