//! Debug tool to trace STEP assembly transforms

fn main() {
    let path = std::env::args().nth(1).unwrap_or("test/as1-oc-214.stp".to_string());
    let content = std::fs::read_to_string(&path).expect("read step file");
    let step_file = draper_step::parser::parse_step(&content).expect("parse step file");
    
    // Use the lazy path (same as the viewer)
    let (tree, pending) = draper_step::converter::step_structure_lazy(&step_file);
    
    println!("=== Assembly Tree (root) ===");
    println!("Root: {} (PD #{})", tree.name, tree.pd_id);
    println!();
    
    println!("=== Pending BREP Instances ===");
    println!("Total: {}", pending.len());
    for (i, p) in pending.iter().enumerate() {
        let tf_str = match &p.transform {
            Some(tf) => {
                let tx = tf[0][3]; let ty = tf[1][3]; let tz = tf[2][3];
                // Check if rotation is identity
                let is_identity_rot = (tf[0][0] - 1.0).abs() < 1e-10 && (tf[1][1] - 1.0).abs() < 1e-10 && (tf[2][2] - 1.0).abs() < 1e-10
                    && tf[0][1].abs() < 1e-10 && tf[0][2].abs() < 1e-10
                    && tf[1][0].abs() < 1e-10 && tf[1][2].abs() < 1e-10
                    && tf[2][0].abs() < 1e-10 && tf[2][1].abs() < 1e-10;
                if is_identity_rot {
                    format!("translate({:.1},{:.1},{:.1})", tx, ty, tz)
                } else {
                    format!("translate({:.1},{:.1},{:.1}) + rotation\n  [{:.3},{:.3},{:.3}]\n  [{:.3},{:.3},{:.3}]\n  [{:.3},{:.3},{:.3}]",
                        tx, ty, tz,
                        tf[0][0], tf[0][1], tf[0][2],
                        tf[1][0], tf[1][1], tf[1][2],
                        tf[2][0], tf[2][1], tf[2][2])
                }
            }
            None => "NO TRANSFORM".to_string(),
        };
        println!("[{:2}] {} BREP#{} transform={}", i, p.name, p.brep_id, tf_str);
    }
    
    // Now triangulate and check per-instance bounding boxes
    println!();
    println!("=== Per-Instance Bounding Boxes ===");
    let mut ctx = draper_step::OwnedStepConversionContext::new(step_file);
    for (i, p) in pending.iter().enumerate() {
        if let Some(inst) = ctx.triangulate_pending(p) {
            let (bmin, bmax) = inst.mesh.bounding_box();
            let color_str = inst.color.map_or("none".to_string(), |c| format!("{:.2},{:.2},{:.2}", c[0], c[1], c[2]));
            println!("[{:2}] {} BREP#{} color={} bbox=({:.1},{:.1},{:.1})-({:.1},{:.1},{:.1}) tris={}",
                i, inst.name, inst.brep_id, color_str,
                bmin.x, bmin.y, bmin.z, bmax.x, bmax.y, bmax.z,
                inst.mesh.triangle_count());
        } else {
            println!("[{:2}] {} BREP#{} — FAILED TRIANGULATION", i, p.name, p.brep_id);
        }
    }
    
    // Also dump the NAUO transform index
    println!();
    println!("=== Assembly Tree Structure ===");
    print_tree(&tree, 0);
}

fn print_tree(node: &draper_step::AssemblyNode, depth: usize) {
    let indent = "  ".repeat(depth);
    let tf_str = match &node.transform {
        Some(tf) => {
            let tx = tf[0][3]; let ty = tf[1][3]; let tz = tf[2][3];
            let is_identity = (tf[0][0] - 1.0).abs() < 1e-10 && (tf[1][1] - 1.0).abs() < 1e-10 && (tf[2][2] - 1.0).abs() < 1e-10
                && tf[0][1].abs() < 1e-10 && tf[0][2].abs() < 1e-10 && tf[0][3].abs() < 1e-10
                && tf[1][0].abs() < 1e-10 && tf[1][2].abs() < 1e-10 && tf[1][3].abs() < 1e-10
                && tf[2][0].abs() < 1e-10 && tf[2][1].abs() < 1e-10 && tf[2][3].abs() < 1e-10;
            if is_identity {
                "identity".to_string()
            } else if tx.abs() < 1e-10 && ty.abs() < 1e-10 && tz.abs() < 1e-10 {
                format!("rotation [{:.3},{:.3},{:.3};{:.3},{:.3},{:.3};{:.3},{:.3},{:.3}]",
                    tf[0][0], tf[0][1], tf[0][2],
                    tf[1][0], tf[1][1], tf[1][2],
                    tf[2][0], tf[2][1], tf[2][2])
            } else {
                format!("translate({:.1},{:.1},{:.1}) [{:.3},{:.3},{:.3};{:.3},{:.3},{:.3};{:.3},{:.3},{:.3}]",
                    tx, ty, tz,
                    tf[0][0], tf[0][1], tf[0][2],
                    tf[1][0], tf[1][1], tf[1][2],
                    tf[2][0], tf[2][1], tf[2][2])
            }
        }
        None => "NO-TF".to_string(),
    };
    let brep_str = node.brep_id.map_or("".to_string(), |id| format!(" BREP#{}", id));
    let inst_str = node.instance_index.map_or("".to_string(), |idx| format!(" inst={}", idx));
    println!("{}{} (PD#{}){} tf={}{}", indent, node.name, node.pd_id, brep_str, tf_str, inst_str);
    for child in &node.children {
        print_tree(child, depth + 1);
    }
}
