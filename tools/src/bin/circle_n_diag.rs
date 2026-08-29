// Diagnostic: replicate pre_populate_for_solid for the first solid and print
// the per-circle-edge sample counts (n) to compare alignment strategies.
// Run: cargo run --release --bin circle_n_diag -- test/8500-02_Vulcan.STEP [face_idx]

use draper_step::{parse_step_file, extract_solids};
use draper_mesh::edge_cache::EdgeDiscretizationCache;
use draper_geometry::{Point3d, Curve3d};

fn main() {
    let path = std::env::args().nth(1).unwrap_or_else(||
        "test/8500-02_Vulcan.STEP".to_string()
    );
    let face_idx: usize = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(0);
    println!("Loading: {}", path);
    let step = match parse_step_file(&path) {
        Ok(s) => s,
        Err(e) => { eprintln!("Parse error: {}", e); return; }
    };
    let (solids, _) = extract_solids(&step);
    let solid = &solids[0];
    let faces = solid.faces();
    println!("Solid #0: {} faces; inspecting face {}", faces.len(), face_idx);

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
    // Same setup as triangulate_solid_with_report
    let mut cache = EdgeDiscretizationCache::with_adaptive_tolerance(&bmin, &bmax, 64);
    cache.set_chord_tolerance_override(Some(0.01)); // params.max_deviation default
    // Emulate pre_populate_for_solid (EDGE_SAMPLES = 20)
    cache.pre_populate_for_solid(solid, 20);

    // Dump circle edge point counts for the requested face
    let face = &faces[face_idx];
    println!("\nFace {} circle edges:", face_idx);
    let mut total_pts = 0usize;
    let mut n_edges = 0usize;
    for (i, edge) in face.edges.iter().enumerate() {
        let is_circle = matches!(edge.curve, Some(Curve3d::Circle(_)));
        if let Some(disc) = cache.get(edge.id) {
            let n = disc.points_3d.len();
            total_pts += n;
            n_edges += 1;
            if i < 24 || is_circle {
                let r = match &edge.curve {
                    Some(Curve3d::Circle(c)) => format!("r={:.3}", c.radius),
                    _ => "non-circle".to_string(),
                };
                println!("  edge {:2}: key={:?} {} range=({:.4},{:.4}) -> {} pts",
                    i, edge.step_entity_id, r, edge.param_range.0, edge.param_range.1, n);
            }
        }
    }
    println!("TOTAL: {} edges, {} points", n_edges, total_pts);
}
