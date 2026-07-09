// Diagnostic: build the NURBS Saddle mesh using the same code path as
// build_nurbs_surface_mesh in the viewer, and inspect the resulting
// TriangleMesh for degenerate triangles, holes, and correctness.

use draper_geometry::{NurbsSurface, Surface, Point3d as P3, Point2d};
use draper_mesh::{TriangleMesh, TriangulationParams, check_manifold, triangulate_face_with_boundary_and_holes_uv};

fn main() {
    // ---- Replicate load_nurbs_saddle exactly ----
    let control_points = vec![
        vec![P3::new(-50.0, -50.0,   0.0), P3::new(-17.0, -50.0, -28.0), P3::new( 17.0, -50.0, -28.0), P3::new( 50.0, -50.0,   0.0)],
        vec![P3::new(-50.0, -17.0,  28.0), P3::new(-17.0, -17.0,  -6.0), P3::new( 17.0, -17.0,  -6.0), P3::new( 50.0, -17.0,  28.0)],
        vec![P3::new(-50.0,  17.0,  28.0), P3::new(-17.0,  17.0,  -6.0), P3::new( 17.0,  17.0,  -6.0), P3::new( 50.0,  17.0,  28.0)],
        vec![P3::new(-50.0,  50.0,   0.0), P3::new(-17.0,  50.0, -28.0), P3::new( 17.0,  50.0, -28.0), P3::new( 50.0,  50.0,   0.0)],
    ];
    let weights = vec![vec![1.0; 4]; 4];
    let u_knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let v_knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let nurbs_surface = NurbsSurface::from_v_rows(
        3, 3, control_points, weights, u_knots, v_knots, false, false,
    );

    // ---- Replicate build_nurbs_surface_mesh ----
    let (u_min, u_max) = nurbs_surface.u_range();
    let (v_min, v_max) = nurbs_surface.v_range();
    let surface = Surface::Nurbs(nurbs_surface.clone());
    let steps = 30;
    let mut boundary: Vec<P3> = Vec::new();
    let mut boundary_uvs: Vec<Point2d> = Vec::new();
    // Bottom edge (v = v_min)
    for i in 0..=steps {
        let u = u_min + (u_max - u_min) * i as f64 / steps as f64;
        boundary.push(surface.point_at(u, v_min));
        boundary_uvs.push(Point2d::new(u, v_min));
    }
    // Right edge (u = u_max)
    for i in 1..=steps {
        let v = v_min + (v_max - v_min) * i as f64 / steps as f64;
        boundary.push(surface.point_at(u_max, v));
        boundary_uvs.push(Point2d::new(u_max, v));
    }
    // Top edge (v = v_max), reversed
    for i in (0..steps).rev() {
        let u = u_min + (u_max - u_min) * i as f64 / steps as f64;
        boundary.push(surface.point_at(u, v_max));
        boundary_uvs.push(Point2d::new(u, v_max));
    }
    // Left edge (u = u_min), reversed
    for i in (1..steps).rev() {
        let v = v_min + (v_max - v_min) * i as f64 / steps as f64;
        boundary.push(surface.point_at(u_min, v));
        boundary_uvs.push(Point2d::new(u_min, v));
    }

    println!("Boundary: {} points", boundary.len());
    println!("  First: ({:.3}, {:.3}, {:.3})", boundary[0].x, boundary[0].y, boundary[0].z);
    println!("  Last:  ({:.3}, {:.3}, {:.3})", boundary.last().unwrap().x, boundary.last().unwrap().y, boundary.last().unwrap().z);
    let bbox = boundary.iter().fold(
        (P3::new(f64::MAX, f64::MAX, f64::MAX), P3::new(f64::MIN, f64::MIN, f64::MIN)),
        |(mn, mx), p| (
            P3::new(mn.x.min(p.x), mn.y.min(p.y), mn.z.min(p.z)),
            P3::new(mx.x.max(p.x), mx.y.max(p.y), mx.z.max(p.z)),
        ),
    );
    println!("  bbox: min=({:.1},{:.1},{:.1}) max=({:.1},{:.1},{:.1})",
             bbox.0.x, bbox.0.y, bbox.0.z, bbox.1.x, bbox.1.y, bbox.1.z);

    // ---- Triangulate using the same path as the viewer ----
    let params = TriangulationParams::default();
    println!("\nCalling triangulate_face_with_boundary_and_holes_uv...");
    let mesh = triangulate_face_with_boundary_and_holes_uv(
        &surface, &boundary, &boundary_uvs, &[], &[], true, &params,
    );
    println!("\nResult: {} vertices, {} triangles", mesh.vertices.len(), mesh.triangles.len());

    if mesh.triangles.is_empty() {
        println!("  ✗ EMPTY mesh — no triangles produced!");
        return;
    }

    // ---- Check for degenerate (zero-area) triangles ----
    let mut degen_count = 0;
    let mut tiny_count = 0;
    let mut total_area = 0.0_f64;
    let mut max_area = 0.0_f64;
    let mut min_area = f64::MAX;
    for (i, tri) in mesh.triangles.iter().enumerate() {
        let v0 = mesh.vertices[tri[0] as usize];
        let v1 = mesh.vertices[tri[1] as usize];
        let v2 = mesh.vertices[tri[2] as usize];
        let e1 = P3::new(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
        let e2 = P3::new(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
        let cross = P3::new(
            e1.y * e2.z - e1.z * e2.y,
            e1.z * e2.x - e1.x * e2.z,
            e1.x * e2.y - e1.y * e2.x,
        );
        let area = 0.5 * (cross.x * cross.x + cross.y * cross.y + cross.z * cross.z).sqrt();
        if area < 1e-12 { degen_count += 1; }
        else if area < 0.01 { tiny_count += 1; }
        total_area += area;
        if area > max_area { max_area = area; }
        if area < min_area && area > 1e-12 { min_area = area; }
        if i < 5 || (degen_count > 0 && i < 50) {
            // print first few + first few degenerate
        }
    }
    println!("\nTriangle quality:");
    println!("  Total area: {:.2}", total_area);
    println!("  Max area:   {:.4}", max_area);
    println!("  Min area:   {:.6} (excludes zero-area)", min_area);
    println!("  Degenerate (area<1e-12): {} / {}", degen_count, mesh.triangles.len());
    println!("  Tiny (area<0.01):        {} / {}", tiny_count, mesh.triangles.len());

    // ---- Check for out-of-bounds triangle indices ----
    let mut oob_count = 0;
    for tri in &mesh.triangles {
        for &idx in tri {
            if idx as usize >= mesh.vertices.len() { oob_count += 1; }
        }
    }
    println!("\n  Out-of-bounds indices: {}", oob_count);

    // ---- Check for NaN/Inf in vertices ----
    let nan_count = mesh.vertices.iter().filter(|p|
        !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite()
    ).count();
    println!("  NaN/Inf vertices: {}", nan_count);

    // ---- Check vertex bounds (should be within [-50,+50]^3 roughly) ----
    let vbbox = mesh.vertices.iter().fold(
        (P3::new(f64::MAX, f64::MAX, f64::MAX), P3::new(f64::MIN, f64::MIN, f64::MIN)),
        |(mn, mx), p| (
            P3::new(mn.x.min(p.x), mn.y.min(p.y), mn.z.min(p.z)),
            P3::new(mx.x.max(p.x), mx.y.max(p.y), mx.z.max(p.z)),
        ),
    );
    println!("\n  Vertex bbox: min=({:.2},{:.2},{:.2}) max=({:.2},{:.2},{:.2})",
             vbbox.0.x, vbbox.0.y, vbbox.0.z, vbbox.1.x, vbbox.1.y, vbbox.1.z);

    // ---- Manifold check ----
    let report = check_manifold(&mesh);
    println!("\nManifold report:");
    println!("  Watertight: {}", report.is_watertight());
    println!("  Total edges: {}", report.edge_count);
    println!("  Boundary edges: {}", report.boundary_edge_count);
    println!("  Non-manifold edges: {}", report.non_manifold_edge_count);
    println!("  Degenerate triangles: {}", report.degenerate_triangle_count);

    // ---- Now compare to a simple regular grid mesh ----
    println!("\n--- Comparison: simple regular grid mesh (30x30) ---");
    let gn = 30;
    let mut grid_mesh = TriangleMesh::new();
    let du = (u_max - u_min) / gn as f64;
    let dv = (v_max - v_min) / gn as f64;
    for j in 0..=gn {
        for i in 0..=gn {
            let u = u_min + i as f64 * du;
            let v = v_min + j as f64 * dv;
            grid_mesh.vertices.push(surface.point_at(u, v));
        }
    }
    let row_stride = (gn + 1) as u32;
    for j in 0..gn {
        for i in 0..gn {
            let v00 = j * row_stride + i;
            let v10 = v00 + 1;
            let v01 = v00 + row_stride;
            let v11 = v01 + 1;
            grid_mesh.triangles.push([v00, v10, v11]);
            grid_mesh.triangles.push([v00, v11, v01]);
        }
    }
    println!("Grid mesh: {} vertices, {} triangles", grid_mesh.vertices.len(), grid_mesh.triangles.len());
    let greport = check_manifold(&grid_mesh);
    println!("  Watertight: {}", greport.is_watertight());
    println!("  Boundary edges: {}", greport.boundary_edge_count);
    let garea: f64 = grid_mesh.triangles.iter().map(|tri| {
        let v0 = grid_mesh.vertices[tri[0] as usize];
        let v1 = grid_mesh.vertices[tri[1] as usize];
        let v2 = grid_mesh.vertices[tri[2] as usize];
        let e1 = P3::new(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
        let e2 = P3::new(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
        let cross = P3::new(
            e1.y * e2.z - e1.z * e2.y,
            e1.z * e2.x - e1.x * e2.z,
            e1.x * e2.y - e1.y * e2.x,
        );
        0.5 * (cross.x * cross.x + cross.y * cross.y + cross.z * cross.z).sqrt()
    }).sum();
    println!("  Total area: {:.2}", garea);

    println!("\n=== DIAGNOSIS ===");
    if mesh.triangles.is_empty() {
        println!("  ✗ triangulate_face_with_boundary_and_holes_uv returns EMPTY mesh.");
        println!("    → This is why the 3D view shows nothing/broken for NURBS.");
        println!("    → FIX: replace with simple regular grid mesh in build_nurbs_surface_mesh.");
    } else if report.boundary_edge_count > 0 || degen_count > mesh.triangles.len() / 10 {
        println!("  ✗ Complex triangulation produces broken mesh:");
        println!("    - {} boundary edges (holes), {} degenerate triangles", report.boundary_edge_count, degen_count);
        println!("    → FIX: replace with simple regular grid mesh in build_nurbs_surface_mesh.");
    } else {
        println!("  ✓ Complex triangulation produces a valid mesh ({} triangles, {} boundary edges).",
                 mesh.triangles.len(), report.boundary_edge_count);
        println!("    → The 3D rendering bug is elsewhere (shader, normals, etc.).");
    }
}
