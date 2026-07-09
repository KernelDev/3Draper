// Diagnostic: dump all boundary edges for 3.05.078.stp BREP and check
// whether they come from alias misses or real topology gaps.
use draper_step::{parse_step, step_structure_lazy, StepConversionContext};
use draper_mesh::validate_watertight;
use std::collections::HashMap;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .init();

    let content = std::fs::read_to_string("/home/z/my-project/test/3.05.078.stp")
        .expect("read 3.05.078.stp");
    let step = parse_step(&content).expect("parse step");
    let (_tree, pending) = step_structure_lazy(&step);
    let ctx = StepConversionContext::new(&step);

    for p in &pending {
        let inst = match ctx.triangulate_pending(p) {
            Some(i) => i,
            None => continue,
        };

        let report = validate_watertight(&inst.mesh, true);
        println!("\n=== BREP #{}: v={} t={} ===", p.brep_id, inst.mesh.vertex_count(), inst.mesh.triangle_count());
        println!("  watertight: {}", report.is_watertight());
        println!("  boundary edges: {}", report.boundary_edge_count);

        // Group boundary edges by face pair
        let fids = inst.mesh.triangle_face_ids.as_ref();
        if report.boundary_edge_count == 0 || fids.is_none() {
            continue;
        }
        let fids = fids.unwrap();

        // Build edge → (triangle_idx, face_id) map
        let mut edge_to_face: HashMap<(u32, u32), u64> = HashMap::new();
        for (ti, tri) in inst.mesh.triangles.iter().enumerate() {
            let fid = fids.get(ti).copied().unwrap_or(u64::MAX);
            let edges = [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])];
            for (a, b) in edges {
                let key = if a < b { (a, b) } else { (b, a) };
                edge_to_face.insert(key, fid);
            }
        }

        // For each boundary edge (count==1), find which face it belongs to
        // and compute its length + 3D position
        let mut edge_count_map: HashMap<(u32, u32), usize> = HashMap::new();
        for tri in &inst.mesh.triangles {
            let edges = [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])];
            for (a, b) in edges {
                let key = if a < b { (a, b) } else { (b, a) };
                *edge_count_map.entry(key).or_insert(0) += 1;
            }
        }

        // Collect boundary edges with their face_id and 3D info
        let mut boundary_by_face: HashMap<u64, Vec<(f64, [f64; 6])>> = HashMap::new();
        let mut boundary_lengths: Vec<f64> = Vec::new();
        for (edge, &count) in &edge_count_map {
            if count == 1 {
                let v0 = inst.mesh.vertices[edge.0 as usize];
                let v1 = inst.mesh.vertices[edge.1 as usize];
                let dx = v1.x - v0.x;
                let dy = v1.y - v0.y;
                let dz = v1.z - v0.z;
                let len = (dx*dx + dy*dy + dz*dz).sqrt();
                boundary_lengths.push(len);
                let fid = edge_to_face.get(edge).copied().unwrap_or(u64::MAX);
                boundary_by_face.entry(fid).or_default().push((len, [v0.x, v0.y, v0.z, v1.x, v1.y, v1.z]));
            }
        }

        // Length statistics
        boundary_lengths.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let n = boundary_lengths.len();
        if n > 0 {
            println!("\n  Boundary edge lengths ({} edges):", n);
            println!("    min:    {:.6e} mm", boundary_lengths[0]);
            println!("    25%:    {:.6e} mm", boundary_lengths[n / 4]);
            println!("    median: {:.6e} mm", boundary_lengths[n / 2]);
            println!("    75%:    {:.6e} mm", boundary_lengths[3 * n / 4]);
            println!("    max:    {:.6e} mm", boundary_lengths[n - 1]);

            // Categorize
            let fp_drift = boundary_lengths.iter().filter(|&&l| l < 1e-6).count();
            let sub_micron = boundary_lengths.iter().filter(|&&l| l >= 1e-6 && l < 1e-3).count();
            let sub_mm = boundary_lengths.iter().filter(|&&l| l >= 1e-3 && l < 0.1).count();
            let small = boundary_lengths.iter().filter(|&&l| l >= 0.1 && l < 1.0).count();
            let medium = boundary_lengths.iter().filter(|&&l| l >= 1.0 && l < 5.0).count();
            let large = boundary_lengths.iter().filter(|&&l| l >= 5.0).count();
            println!("    FP drift (<1e-6 mm):    {}", fp_drift);
            println!("    sub-micron (1e-6..1e-3): {}", sub_micron);
            println!("    sub-mm (1e-3..0.1):      {}", sub_mm);
            println!("    small (0.1..1.0):        {}", small);
            println!("    medium (1.0..5.0):       {}", medium);
            println!("    large (>=5.0):           {}", large);
        }

        // Per-face boundary edges
        println!("\n  Boundary edges by face:");
        let mut sorted_faces: Vec<_> = boundary_by_face.iter().collect();
        sorted_faces.sort_by_key(|(_, v)| std::cmp::Reverse(v.len()));
        for (fid, edges) in sorted_faces.iter().take(15) {
            // Find step_face_id for this face_id
            let step_fid = inst.faces.iter()
                .find(|fi| &fi.face_id == *fid)
                .map(|fi| fi.step_face_id)
                .unwrap_or(0);
            let surface = inst.faces.iter()
                .find(|fi| &fi.face_id == *fid)
                .map(|fi| &fi.surface_type)
                .cloned()
                .unwrap_or_default();
            let max_len = edges.iter().map(|(l, _)| *l).fold(0.0f64, f64::max);
            let avg_len = edges.iter().map(|(l, _)| *l).sum::<f64>() / edges.len() as f64;
            println!("    face_id={} (Step#{}, {}): {} edges, avg_len={:.4}mm, max_len={:.4}mm",
                fid, step_fid, surface, edges.len(), avg_len, max_len);
        }

        break; // Only first BREP
    }
}
