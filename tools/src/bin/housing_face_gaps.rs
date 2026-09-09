// Diagnostic: per-face gap profile for one BREP instance.
// Groups the instance mesh by triangle face id; for each face counts
// boundary edges (1 adjacent triangle) and classifies their vertices
// (shared across faces = true boundary verts vs interior Steiner).
// Usage: housing_face_gaps <file.stp> <brep_id>

use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use std::collections::{HashMap, HashSet};

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Error)
        .init();
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).map(|s| s.as_str()).unwrap_or("test/drill_top.stp");
    let brep_id: i64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(47598);

    let data = std::fs::read_to_string(path).expect("read STEP");
    let step = parse_step(&data).expect("parse STEP");
    let (_tree, pending) = step_structure_lazy(&step);
    let target = pending.iter().find(|p| p.brep_id == brep_id)
        .unwrap_or_else(|| panic!("BREP {} not found", brep_id));
    let mut ctx = StepConversionContext::new(&step);
    let inst = ctx.triangulate_pending(target).expect("convert BREP");
    let mesh = &inst.mesh;

    let face_ids = match &mesh.triangle_face_ids {
        Some(ids) => ids.clone(),
        None => {
            eprintln!("No triangle_face_ids");
            return;
        }
    };

    // GLOBAL edge usage across all faces: boundary = exactly 1 triangle
    // in the whole instance mesh (true mesh hole).
    let mut edge_use: HashMap<(u32, u32), u32> = HashMap::new();
    let mut edge_owner: HashMap<(u32, u32), u64> = HashMap::new();
    for (ti, t) in mesh.triangles.iter().enumerate() {
        let fid = face_ids[ti];
        for k in 0..3 {
            let a = t[k];
            let b = t[(k + 1) % 3];
            if a == b {
                continue;
            }
            let key = if a < b { (a, b) } else { (b, a) };
            *edge_use.entry(key).or_insert(0) += 1;
            edge_owner.insert(key, fid);
        }
    }

    // vertex -> owning faces
    let mut vert_owner: HashMap<u32, HashSet<u64>> = HashMap::new();
    for (ti, t) in mesh.triangles.iter().enumerate() {
        let fid = face_ids[ti];
        for &v in t {
            vert_owner.entry(v).or_default().insert(fid);
        }
    }

    // face id -> FaceInfo lookup
    let finfo: HashMap<u64, &draper_step::FaceInfo> =
        inst.faces.iter().map(|f| (f.face_id, f)).collect();

    let mut total_boundary = 0usize;
    let mut bad_faces = 0usize;
    let mut rows: Vec<(u64, usize, usize, usize, usize)> = Vec::new();
    let mut per_face: HashMap<u64, (usize, usize, usize, HashSet<u32>)> = HashMap::new();
    for (key, &count) in &edge_use {
        if count != 1 {
            continue;
        }
        let fid = edge_owner[key];
        let (a, b) = *key;
        let entry = per_face.entry(fid).or_insert((0, 0, 0, HashSet::new()));
        entry.0 += 1;
        for v in [a, b] {
            if entry.3.insert(v) {
                if vert_owner.get(&v).map(|s| s.len()).unwrap_or(0) >= 2 {
                    entry.1 += 1;
                } else {
                    entry.2 += 1;
                }
            }
        }
    }
    let n_faces_triangulated = {
        let mut s: HashSet<u64> = HashSet::new();
        for f in &face_ids {
            s.insert(*f);
        }
        s.len()
    };
    for (fid, (nb, sv, uv, _)) in &per_face {
        bad_faces += 1;
        total_boundary += nb;
        rows.push((*fid, *nb, *sv, *uv, edge_use.len()));
    }
    rows.sort_by(|x, y| y.1.cmp(&x.1));

    println!("BREP #{}: {} faces triangulated, {} vertices, {} triangles",
             brep_id, n_faces_triangulated, mesh.vertices.len(), mesh.triangles.len());
    for (fid, nb, sv, uv, _) in rows.iter().take(40) {
        let info = finfo.get(fid);
        let name = info.map(|f| format!("{} step_face={}", f.surface_type, f.step_face_id)).unwrap_or_default();
        println!("face_id {:<6} boundary_edges={:<5} shared_v={:<4} unique_v={:<4} {}",
                 fid, nb, sv, uv, name);
    }
    println!("\nTOTAL: {} gapped faces, {} boundary edges, {} clean faces",
             bad_faces, total_boundary, n_faces_triangulated - bad_faces);
}
