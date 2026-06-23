fn main() {
    let path = "test/Zentralstaender.stp";
    let content = std::fs::read_to_string(&path).expect("read");
    let step_file = draper_step::parser::parse_step(&content).expect("parse");
    
    // Count plane vs curved surfaces in STEP file
    println!("=== Surface types in STEP file ===");
    let mut counts: std::collections::HashMap<String, usize> = std::collections::HashMap::new();
    for e in &step_file.entities {
        match e.type_name.as_str() {
            "PLANE" | "CYLINDRICAL_SURFACE" | "CONICAL_SURFACE" | "SPHERICAL_SURFACE" | "TOROIDAL_SURFACE" => {
                *counts.entry(e.type_name.clone()).or_default() += 1;
            }
            _ if e.type_name.contains("SURFACE") => {
                *counts.entry(e.type_name.clone()).or_default() += 1;
            }
            _ => {}
        }
    }
    for (k, v) in counts.iter() {
        println!("  {}: {}", k, v);
    }
    
    // Compute bbox manually by scanning CARTESIAN_POINT entities
    let mut min = (f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut max = (-f64::INFINITY, -f64::INFINITY, -f64::INFINITY);
    let mut point_count = 0;
    for e in &step_file.entities {
        if e.type_name == "CARTESIAN_POINT" && e.params.len() >= 2 {
            if let Some(coords) = e.params.get(1) {
                if let draper_step::schema::StepValue::List(coords_list) = coords {
                    if coords_list.len() >= 3 {
                        let x = match &coords_list[0] { draper_step::schema::StepValue::Float(x) => *x, _ => 0.0 };
                        let y = match &coords_list[1] { draper_step::schema::StepValue::Float(x) => *x, _ => 0.0 };
                        let z = match &coords_list[2] { draper_step::schema::StepValue::Float(x) => *x, _ => 0.0 };
                        if x.abs() < 1e6 && y.abs() < 1e6 && z.abs() < 1e6 {
                            min.0 = min.0.min(x); min.1 = min.1.min(y); min.2 = min.2.min(z);
                            max.0 = max.0.max(x); max.1 = max.1.max(y); max.2 = max.2.max(z);
                            point_count += 1;
                        }
                    }
                }
            }
        }
    }
    if point_count > 0 {
        let dx = max.0 - min.0;
        let dy = max.1 - min.1;
        let dz = max.2 - min.2;
        let diag = (dx*dx + dy*dy + dz*dz).sqrt();
        println!("\nBBox from CARTESIAN_POINT ({} pts): ({:.2},{:.2},{:.2}) to ({:.2},{:.2},{:.2})",
            point_count, min.0, min.1, min.2, max.0, max.1, max.2);
        println!("Diagonal: {:.2} mm", diag);
        println!("bbox-floor (diag*0.0002): {:.6}", diag * 0.0002);
        
        println!("\nEffective max_deviation per LOD (after floor):");
        for lod in [0.05, 0.1, 0.2, 0.3, 0.5, 0.75, 1.0] {
            let mut p = draper_mesh::TriangulationParams::for_lod(lod);
            let raw_dev = p.max_deviation;
            if diag > 1.0 {
                p.max_deviation = p.max_deviation.max(diag * 0.0002);
            }
            let ratio = p.max_deviation / raw_dev.max(1e-10);
            println!("  LOD {:.2}: raw_dev={:.6} → effective_dev={:.6} (×{:.1} clamp-up)",
                lod, raw_dev, p.max_deviation, ratio);
        }
    }
}
