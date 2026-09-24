//! Session-52 diagnostic: minimal reproducer for the T-junction explosion
//! non-determinism on drill_top HOUSING_MIRROR.
//!
//! Loads a dumped `d-after-weld` OBJ (+ optional .fmap), rebuilds a
//! TriangleMesh, computes the pipeline's tj_tol (bbox diagonal × 1e-9,
//! exactly as converter.rs:5219/6105), and calls repair_t_junctions.
//!
//! Run multiple times (per-process HashMap seed differs) to prove or
//! refute non-determinism with a bit-identical input mesh.
//!
//! Usage: tj_repro <mesh.obj> [repeats-in-child-note]

use draper_geometry::Point3d;
use draper_mesh::TriangleMesh;

fn load_obj(path: &str) -> TriangleMesh {
    let data = std::fs::read_to_string(path).expect("read obj");
    let mut mesh = TriangleMesh::new();
    for line in data.lines() {
        if let Some(rest) = line.strip_prefix("v ") {
            let mut it = rest.split_whitespace();
            let x: f64 = it.next().unwrap().parse().unwrap();
            let y: f64 = it.next().unwrap().parse().unwrap();
            let z: f64 = it.next().unwrap().parse().unwrap();
            mesh.vertices.push(Point3d { x, y, z });
        } else if let Some(rest) = line.strip_prefix("f ") {
            let mut idx = [0u32; 3];
            for (i, tok) in rest.split_whitespace().enumerate().take(3) {
                // OBJ may carry "a/b/c" or "a//c" forms — take vertex index.
                let v: u32 = tok.split('/').next().unwrap().parse().unwrap();
                idx[i] = v - 1;
            }
            mesh.triangles.push(idx);
        }
    }
    // Face ids (optional, for parity with the pipeline mesh).
    let fmap_path = path.replace(".obj", ".fmap");
    if let Ok(fmap) = std::fs::read_to_string(&fmap_path) {
        let mut ids = Vec::with_capacity(mesh.triangles.len());
        for line in fmap.lines() {
            if let Some(rest) = line.strip_prefix("t ") {
                if let Some(fid) = rest.split_whitespace().nth(1) {
                    ids.push(fid.parse::<u64>().unwrap_or(u64::MAX));
                }
            }
        }
        if ids.len() == mesh.triangles.len() {
            mesh.triangle_face_ids = Some(ids);
        }
    }
    mesh
}

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .parse_default_env()
        .init();

    let args: Vec<String> = std::env::args().collect();
    let path = args
        .iter()
        .skip(1)
        .find(|a| !a.starts_with('-'))
        .expect("usage: tj_repro <mesh.obj>");

    let mesh = load_obj(path);
    let n_verts = mesh.vertices.len();
    let n_tris = mesh.triangles.len();

    // bbox diagonal (exactly as ToleranceContext::from_bounding_box)
    let mut bmin = Point3d {
        x: f64::INFINITY,
        y: f64::INFINITY,
        z: f64::INFINITY,
    };
    let mut bmax = Point3d {
        x: f64::NEG_INFINITY,
        y: f64::NEG_INFINITY,
        z: f64::NEG_INFINITY,
    };
    for v in &mesh.vertices {
        bmin.x = bmin.x.min(v.x);
        bmin.y = bmin.y.min(v.y);
        bmin.z = bmin.z.min(v.z);
        bmax.x = bmax.x.max(v.x);
        bmax.y = bmax.y.max(v.y);
        bmax.z = bmax.z.max(v.z);
    }
    let dx = bmax.x - bmin.x;
    let dy = bmax.y - bmin.y;
    let dz = bmax.z - bmin.z;
    let model_scale = (dx * dx + dy * dy + dz * dz).sqrt().max(1e-10);
    let tj_tol = (model_scale * 1e-9).max(1e-10);

    println!(
        "tj_repro: {} verts / {} tris, bbox diag={:.6}, tj_tol={:.3e}",
        n_verts, n_tris, model_scale, tj_tol
    );

    // Determinism check inside ONE process: run twice on clones.
    for run in 0..2 {
        let mut m = mesh.clone();
        let t0 = std::time::Instant::now();
        let n = draper_mesh::repair_t_junctions(&mut m, tj_tol);
        println!(
            "run#{}: splits={} tris {} -> {} ({:.1}s)",
            run,
            n,
            n_tris,
            m.triangles.len(),
            t0.elapsed().as_secs_f64()
        );
    }
}
