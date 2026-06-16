//! Quick volume check for as1-oc-214.stp
use draper_step::converter::step_to_detailed_instances;
use draper_mesh::TriangleMesh;
use std::collections::HashMap;

fn mesh_volume(mesh: &TriangleMesh) -> f64 {
    let mut volume = 0.0;
    for tri in &mesh.triangles {
        let v0 = mesh.vertices[tri[0] as usize];
        let v1 = mesh.vertices[tri[1] as usize];
        let v2 = mesh.vertices[tri[2] as usize];
        let cross_x = v1.y * v2.z - v1.z * v2.y;
        let cross_y = v1.z * v2.x - v1.x * v2.z;
        let cross_z = v1.x * v2.y - v1.y * v2.x;
        volume += v0.x * cross_x + v0.y * cross_y + v0.z * cross_z;
    }
    volume / 6.0
}

fn count_boundary_edges(mesh: &TriangleMesh) -> usize {
    let mut edge_count: HashMap<(u32, u32), usize> = HashMap::new();
    for tri in &mesh.triangles {
        for k in 0..3 {
            let a = tri[k].min(tri[(k+1)%3]);
            let b = tri[k].max(tri[(k+1)%3]);
            *edge_count.entry((a, b)).or_insert(0) += 1;
        }
    }
    edge_count.values().filter(|&&c| c == 1).count()
}

fn main() {
    env_logger::init();
    
    let step_path = "test/as1-oc-214.stp";
    println!("Loading STEP file: {}", step_path);
    
    let content = std::fs::read_to_string(step_path).expect("Failed to read STEP file");
    let step_file = draper_step::parse_step(&content).expect("Failed to parse STEP file");
    
    println!("STEP file loaded, converting...");
    
    let instances = step_to_detailed_instances(&step_file)
        .expect("Failed to convert STEP file");
    
    println!("\n{} instances generated", instances.len());
    
    let mut total_volume = 0.0;
    for (i, inst) in instances.iter().enumerate() {
        let vol = mesh_volume(&inst.mesh);
        let abs_vol = vol.abs();
        let verts = inst.mesh.vertex_count();
        let tris = inst.mesh.triangle_count();
        let boundary = count_boundary_edges(&inst.mesh);
        let boundary_pct = if tris == 0 { 0.0 } else { boundary as f64 / (tris * 3) as f64 * 100.0 };
        println!(
            "  Instance {}: name='{}', brep_id={}, verts={}, tris={}, volume={:.6}, boundary_edges={} ({:.1}%)",
            i, inst.name, inst.brep_id, verts, tris, abs_vol, boundary, boundary_pct
        );
        total_volume += abs_vol;
    }
    
    println!("\nTotal volume: {:.6}", total_volume);
}
