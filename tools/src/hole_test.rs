use draper_topology::ShapeBuilder;
use draper_mesh::{triangulate_solid, TriangleMesh, TriangulationParams, cut_text_holes_in_mesh, TextSurface};

fn main() {
    env_logger::init();
    
    let solid = ShapeBuilder::make_box(100.0, 80.0, 60.0);
    let params = TriangulationParams::default();
    let base_mesh = triangulate_solid(&solid, &params);
    println!("Base mesh: {} vertices, {} triangles", base_mesh.vertex_count(), base_mesh.triangle_count());

    let mesh = cut_text_holes_in_mesh(
        &base_mesh,
        "3",
        &TextSurface::Plane { z: 30.0 },
        3.0,
        5.0,
        [0.15, 0.15, 0.2, 1.0],
    );
    println!("Hole mesh: {} vertices, {} triangles", mesh.vertex_count(), mesh.triangle_count());

    if mesh.triangle_count() == base_mesh.triangle_count() {
        println!("BUG: No triangles removed! Holes not cutting.");
    }
}
