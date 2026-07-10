// Diagnostic: understand why cone has 20 boundary edges at LOD 0.10
use draper_mesh::{TriangulationParams, check_manifold, triangulate_solid};
use draper_topology::ShapeBuilder;

fn main() {
    let radius = 40.0_f64;
    let height = 80.0_f64;
    let half_angle = (radius / height).atan();
    let solid = ShapeBuilder::make_cone(radius, height, half_angle);

    println!("=== Cone (R={}, h={}, half_angle={:.4}) ===", radius, height, half_angle);
    println!("Faces: {}", solid.faces().len());
    for (i, face) in solid.faces().iter().enumerate() {
        println!(
            "  Face {}: {} edges, surface={}",
            i,
            face.edges.len(),
            face.surface.as_ref().map_or("None", |s| s.type_name())
        );
        for (j, edge) in face.edges.iter().enumerate() {
            let curve_type = match &edge.curve {
                Some(draper_geometry::Curve3d::Circle(c)) => format!("Circle(R={:.2})", c.radius),
                Some(c) => format!("{:?}", c).split('(').next().unwrap_or("?").to_string(),
                None => "None".to_string(),
            };
            println!(
                "    Edge {}: id={:?} curve={}, param=[{:.2},{:.2}]",
                j, edge.id, curve_type, edge.param_range.0, edge.param_range.1
            );
        }
    }

    println!();
    for &lod in &[0.1_f64, 0.5, 1.0] {
        let params = TriangulationParams::for_lod(lod);
        let mesh = triangulate_solid(&solid, &params);
        let report = check_manifold(&mesh);
        println!(
            "LOD {:.2}: max_dev={:.4}, {} verts, {} tris, watertight={}, boundary={}, non_manifold={}, euler={}",
            lod, params.max_deviation, mesh.vertex_count(), mesh.triangle_count(),
            report.is_watertight(), report.boundary_edge_count, report.non_manifold_edge_count,
            report.euler_characteristic
        );
    }
}
