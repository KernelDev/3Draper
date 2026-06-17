// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Test torus triangulation.

use draper_geometry::{Point3d, Surface, TorusSurface};
use draper_topology::ShapeBuilder;
use draper_mesh::{triangulate_solid, validate_watertight};

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .init();

    println!("=== Torus Triangulation Test ===\n");

    // Create a torus
    let major_r = 10.0;
    let minor_r = 2.0;
    let solid = ShapeBuilder::make_torus(major_r, minor_r);
    println!("Created torus: major_r={}, minor_r={}", major_r, minor_r);

    // Triangulate with default params
    let params = draper_mesh::TriangulationParams::default();
    let mesh = triangulate_solid(&solid, &params);

    println!("\nMesh: {} vertices, {} triangles", mesh.vertex_count(), mesh.triangle_count());

    // Check watertightness
    let report = validate_watertight(&mesh, true);
    println!("\nWatertight: {}", if report.is_watertight() { "YES" } else { "NO" });
    println!("  Boundary edges: {}", report.boundary_edge_count);
    println!("  Non-manifold edges: {}", report.non_manifold_edge_count);
    println!("  Degenerate triangles: {}", report.degenerate_triangle_count);
    println!("  Euler characteristic: {}", report.euler_characteristic);

    // Compute volume (should be 2*pi^2 * R * r^2 for a torus)
    let expected_volume = 2.0 * std::f64::consts::PI * std::f64::consts::PI * major_r * minor_r * minor_r;
    let mesh_volume = compute_volume(&mesh);
    println!("\nVolume: {:.4} (expected: {:.4})", mesh_volume, expected_volume);
    println!("  Volume error: {:.4}%", (mesh_volume - expected_volume).abs() / expected_volume * 100.0);

    // Check surface area (should be 4*pi^2 * R * r)
    let expected_area = 4.0 * std::f64::consts::PI * std::f64::consts::PI * major_r * minor_r;
    let mesh_area = mesh.surface_area();
    println!("\nSurface area: {:.4} (expected: {:.4})", mesh_area, expected_area);
    println!("  Area error: {:.4}%", (mesh_area - expected_area).abs() / expected_area * 100.0);

    // Print some sample triangles
    println!("\nFirst 5 triangles:");
    for (i, tri) in mesh.triangles.iter().take(5).enumerate() {
        let v0 = mesh.vertices[tri[0] as usize];
        let v1 = mesh.vertices[tri[1] as usize];
        let v2 = mesh.vertices[tri[2] as usize];
        println!("  tri {}: ({:.2},{:.2},{:.2}) ({:.2},{:.2},{:.2}) ({:.2},{:.2},{:.2})",
            i, v0.x, v0.y, v0.z, v1.x, v1.y, v1.z, v2.x, v2.y, v2.z);
    }
}

fn compute_volume(mesh: &draper_mesh::TriangleMesh) -> f64 {
    let mut volume = 0.0;
    for tri in &mesh.triangles {
        let v0 = mesh.vertices[tri[0] as usize];
        let v1 = mesh.vertices[tri[1] as usize];
        let v2 = mesh.vertices[tri[2] as usize];
        // Signed volume of tetrahedron (origin, v0, v1, v2) = (v0 · (v1 × v2)) / 6
        let cross_x = v1.y * v2.z - v1.z * v2.y;
        let cross_y = v1.z * v2.x - v1.x * v2.z;
        let cross_z = v1.x * v2.y - v1.y * v2.x;
        volume += (v0.x * cross_x + v0.y * cross_y + v0.z * cross_z) / 6.0;
    }
    volume.abs()
}
