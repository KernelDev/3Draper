// Diagnostic: per-face vertex counts with SHARED cache (emulates sequential path).
// Prints top faces by vertex count to find what inflates the mesh.
// Run: cargo run --release --bin face_size_diag -- test/8500-02_Vulcan.STEP [max_faces]

use draper_step::{parse_step_file, extract_solids};
use draper_mesh::triangulate::{TriangulationParams, triangulate_face_with_cache};
use draper_mesh::edge_cache::EdgeDiscretizationCache;
use draper_geometry::{Point3d, Surface};

fn main() {
    let path = std::env::args().nth(1).unwrap_or_else(||
        "test/8500-02_Vulcan.STEP".to_string()
    );
    let max_faces: usize = std::env::args().nth(2)
        .and_then(|s| s.parse().ok()).unwrap_or(usize::MAX);
    println!("Loading: {}", path);
    let step = match parse_step_file(&path) {
        Ok(s) => s,
        Err(e) => { eprintln!("Parse error: {}", e); return; }
    };
    let (solids, _) = extract_solids(&step);
    println!("Solids: {}", solids.len());

    let solid = &solids[0];
    let faces = solid.faces();
    println!("Solid #0: {} faces", faces.len());

    // Emulate triangulate_solid_with_report cache setup
    let mut bmin = Point3d::new(f64::MAX, f64::MAX, f64::MAX);
    let mut bmax = Point3d::new(f64::MIN, f64::MIN, f64::MIN);
    for face in faces.iter() {
        for e in &face.edges {
            for p in [e.start_point(), e.end_point()] {
                if let Some(p) = p {
                    bmin.x = bmin.x.min(p.x); bmin.y = bmin.y.min(p.y); bmin.z = bmin.z.min(p.z);
                    bmax.x = bmax.x.max(p.x); bmax.y = bmax.y.max(p.y); bmax.z = bmax.z.max(p.z);
                }
            }
        }
    }
    let mut cache = EdgeDiscretizationCache::with_adaptive_tolerance(&bmin, &bmax, 64);
    let params = TriangulationParams::default();

    let mut total = 0usize;
    for (fi, face) in faces.iter().enumerate() {
        if fi >= max_faces { break; }
        let t0 = std::time::Instant::now();
        let mesh = triangulate_face_with_cache(face, &params, &mut cache);
        total += mesh.vertices.len();
        let surf = face.surface.as_ref().map(|s| match s {
            Surface::Plane(_) => "Plane",
            Surface::Cylinder(_) => "Cyl",
            Surface::Cone(_) => "Cone",
            Surface::Sphere(_) => "Sph",
            Surface::Torus(_) => "Tor",
            Surface::Nurbs(_) => "Nurbs",
            _ => "Other",
        }).unwrap_or("None");
        println!("face {:4} [{}] edges={:3} bnd_edges={:3} -> {:6} verts, {:6} tris ({:?})",
            fi, surf, face.edges.len(),
            face.outer_wire.as_ref().map(|w| w.coedges.len()).unwrap_or(0),
            mesh.vertices.len(), mesh.triangles.len(), t0.elapsed());
    }
    println!("TOTAL (first {} faces): {} vertices", max_faces, total);
}
