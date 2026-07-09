//! Diagnostic: print detailed triangulation info for a test cylinder
//! at LOD 0.1 (Preview) to understand why it looks "broken/inconsistent".
use draper_geometry::{Curve3d, Point3d};
use draper_mesh::{EdgeDiscretizationCache, TriangulationParams, triangulate_solid, check_manifold};
use draper_topology::ShapeBuilder;

fn main() {
    println!("=== Cylinder triangulation diagnostic ===\n");

    let solid = ShapeBuilder::make_cylinder(40.0, 100.0);
    let params = TriangulationParams::for_lod(0.1);
    println!("LOD 0.1 params: max_deviation={:.4}, angular_samples={}, height_samples={}",
             params.max_deviation, params.angular_samples, params.height_samples);
    println!("  adaptive={}, keep_ratio={:.3}\n", params.adaptive, params.keep_ratio);

    // Print all faces
    let faces = solid.faces();
    println!("Solid has {} faces:\n", faces.len());
    for (i, face) in faces.iter().enumerate() {
        println!("--- Face #{} (id={:?}) ---", i, face.id);
        println!("  surface: {:?}", face.surface);
        println!("  forward: {}", face.forward);
        println!("  edges: {}", face.edges.len());
        for (j, e) in face.edges.iter().enumerate() {
            println!("    edge[{}]: id={:?}, curve={:?}, param_range={:?}", j, e.id, e.curve.as_ref().map(|c| match c {
                Curve3d::Line(_) => "Line",
                Curve3d::Circle(_) => "Circle",
                _ => "Other",
            }), e.param_range);
        }
        if let Some(wire) = &face.outer_wire {
            println!("  outer_wire coedges: {}", wire.coedges.len());
            for (j, ce) in wire.coedges.iter().enumerate() {
                println!("    coedge[{}]: edge={:?}, forward={}", j, ce.edge, ce.forward);
            }
        } else {
            println!("  outer_wire: NONE");
        }
        println!();
    }

    // Now triangulate the full solid
    let mesh = triangulate_solid(&solid, &params);
    println!("=== Full mesh ===");
    println!("vertices: {}", mesh.vertex_count());
    println!("triangles: {}", mesh.triangle_count());

    // Print all vertices grouped by z
    let mut by_z: std::collections::BTreeMap<i64, Vec<(usize, Point3d)>> = std::collections::BTreeMap::new();
    for (i, v) in mesh.vertices.iter().enumerate() {
        let z_bucket = (v.z * 100.0).round() as i64; // round to 0.01mm
        by_z.entry(z_bucket).or_default().push((i, *v));
    }
    println!("\nVertices grouped by z (rounded to 0.01mm):");
    for (z_bucket, verts) in &by_z {
        let z = *z_bucket as f64 / 100.0;
        println!("  z={:.2}: {} vertices", z, verts.len());
        if verts.len() <= 20 {
            for (i, v) in verts {
                println!("    [{}] = ({:.3}, {:.3}, {:.3})", i, v.x, v.y, v.z);
            }
        } else {
            // Print first 5 and last 5
            for (i, v) in verts.iter().take(5) {
                println!("    [{}] = ({:.3}, {:.3}, {:.3})", i, v.x, v.y, v.z);
            }
            println!("    ... ({} more)", verts.len() - 10);
            for (i, v) in verts.iter().skip(verts.len() - 5) {
                println!("    [{}] = ({:.3}, {:.3}, {:.3})", i, v.x, v.y, v.z);
            }
        }
    }

    // Manifold check
    let report = check_manifold(&mesh);
    println!("\n=== Manifold report ===");
    println!("watertight: {}", report.is_watertight());
    println!("vertex_count: {}", report.vertex_count);
    println!("edge_count: {}", report.edge_count);
    println!("triangle_count: {}", report.triangle_count);
    println!("boundary_edge_count: {}", report.boundary_edge_count);
    println!("non_manifold_edge_count: {}", report.non_manifold_edge_count);
    println!("degenerate_triangle_count: {}", report.degenerate_triangle_count);

    // Now check edge cache behavior — triangulate each face separately
    println!("\n=== Per-face triangulation (separate) ===");
    for (i, face) in faces.iter().enumerate() {
        let mut cache = EdgeDiscretizationCache::with_adaptive_tolerance(
            &Point3d::new(-40.0, -40.0, 0.0),
            &Point3d::new(40.0, 40.0, 100.0),
            64,
        );
        cache.set_chord_tolerance_override(Some(params.max_deviation));
        let face_mesh = draper_mesh::triangulate_face_with_cache(face, &params, &mut cache);
        println!("Face #{}: {} vertices, {} triangles", i, face_mesh.vertex_count(), face_mesh.triangle_count());
    }
}
