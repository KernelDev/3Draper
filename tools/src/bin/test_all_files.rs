// Comprehensive test: load ALL STEP files in test/ directory and report
// watertightness, surface types, triangle count, boundary edges, and LOD behavior
use draper_step::{parse_step, step_structure_lazy, OwnedStepConversionContext};
use draper_mesh::{validate_watertight, TriangulationParams};
use std::collections::HashMap;
use std::path::Path;

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Error)
        .init();

    let test_dirs = [
        "/home/z/my-project/3Draper/test",
        "/home/z/my-project/3Draper/test/synthetic",
    ];

    let mut step_files: Vec<String> = Vec::new();

    for test_dir in &test_dirs {
        if let Ok(entries) = std::fs::read_dir(test_dir) {
            for entry in entries.flatten() {
                let path = entry.path();
                if let Some(ext) = path.extension() {
                    let ext = ext.to_string_lossy().to_lowercase();
                    if ext == "stp" || ext == "step" {
                        let p = path.to_string_lossy().to_string();
                        if !step_files.contains(&p) {
                            step_files.push(p);
                        }
                    }
                }
            }
        }
    }
    step_files.sort();

    println!("Found {} STEP files\n", step_files.len());
    println!("{:<40} {:>6} {:>6} {:>8} {:>8} {:>6} {:>6} {:>6}",
        "File", "BREPs", "Verts", "Tris", "BndEdges", "Plane", "Cyl", "Cone");
    println!("{}", "-".repeat(100));

    let mut total_boundary = 0;
    let mut total_tris = 0;
    let mut watertight_count = 0;
    let mut total_files = 0;

    for file_path in &step_files {
        let file_name = Path::new(file_path)
            .file_name()
            .unwrap_or_default()
            .to_string_lossy()
            .to_string();

        let content = match std::fs::read_to_string(file_path) {
            Ok(c) => c,
            Err(e) => {
                println!("{:<40} ERROR: {}", file_name, e);
                continue;
            }
        };

        let step = match parse_step(&content) {
            Ok(s) => s,
            Err(e) => {
                println!("{:<40} PARSE ERROR: {}", file_name, e);
                continue;
            }
        };

        let (_tree, pending) = step_structure_lazy(&step);
        let params = TriangulationParams::for_lod(1.0);
        let mut ctx = OwnedStepConversionContext::new_with_params(step, params);

        let mut file_verts = 0;
        let mut file_tris = 0;
        let mut file_boundary = 0;
        let mut file_breps = 0;
        let mut surface_counts: HashMap<String, usize> = HashMap::new();

        for p in &pending {
            if let Some(inst) = ctx.triangulate_pending(p) {
                file_breps += 1;
                file_verts += inst.mesh.vertex_count();
                file_tris += inst.mesh.triangle_count();

                let report = validate_watertight(&inst.mesh, false);
                file_boundary += report.boundary_edge_count;

                for fi in &inst.faces {
                    let st = fi.surface_type.clone();
                    *surface_counts.entry(st).or_insert(0) += 1;
                }
            }
        }

        let wt = if file_boundary == 0 { "✓" } else { "✗" };
        total_boundary += file_boundary;
        total_tris += file_tris;
        total_files += 1;
        if file_boundary == 0 { watertight_count += 1; }

        let plane = surface_counts.get("Plane").copied().unwrap_or(0);
        let cyl = surface_counts.get("Cylinder").copied().unwrap_or(0);
        let cone = surface_counts.get("Cone").copied().unwrap_or(0);

        println!("{:<40} {:>6} {:>6} {:>8} {:>8}{} {:>6} {:>6} {:>6}",
            file_name, file_breps, file_verts, file_tris, file_boundary, wt,
            plane, cyl, cone);
    }

    println!("{}", "-".repeat(100));
    println!("TOTAL: {} files, {} tris, {} boundary edges", total_files, total_tris, total_boundary);
    println!("Watertight: {}/{} files", watertight_count, total_files);
}
