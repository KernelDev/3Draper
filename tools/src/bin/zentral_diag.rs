//! Diagnostic for Zentralstaender.stp opening regression

fn get_ref(p: &draper_step::StepValue) -> Option<i64> {
    match p {
        draper_step::StepValue::Ref(id) => Some(*id),
        _ => None,
    }
}

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    
    let path = std::env::args().nth(1).unwrap_or("test/Zentralstaender.stp".to_string());
    println!("Loading: {}", path);
    let content = std::fs::read_to_string(&path).expect("read step file");
    let step_file = draper_step::parser::parse_step(&content).expect("parse step file");
    
    // Count entity types
    let mut type_counts: std::collections::HashMap<&str, usize> = std::collections::HashMap::new();
    for e in &step_file.entities {
        *type_counts.entry(e.type_name.as_str()).or_default() += 1;
    }
    let mut sorted: Vec<_> = type_counts.into_iter().collect();
    sorted.sort_by(|a, b| b.1.cmp(&a.1));
    println!("\n=== Entity Type Counts (top 20) ===");
    for (t, c) in sorted.iter().take(20) {
        println!("  {:50} {}", t, c);
    }
    
    let breps = step_file.find_entities_by_type("MANIFOLD_SOLID_BREP");
    let faceted = step_file.find_entities_by_type("FACETED_BREP");
    let nauos = step_file.find_entities_by_type("NEXT_ASSEMBLY_USAGE_OCCURRENCE");
    let pds_all = step_file.find_entities_by_type("PRODUCT_DEFINITION");
    let absrs = step_file.find_entities_by_type("ADVANCED_BREP_SHAPE_REPRESENTATION");
    let srs = step_file.find_entities_by_type("SHAPE_REPRESENTATION");
    
    println!("\n=== Key Counts ===");
    println!("MANIFOLD_SOLID_BREP:        {}", breps.len());
    println!("FACETED_BREP:               {}", faceted.len());
    println!("ADVANCED_BREP_SHAPE_REP:    {}", absrs.len());
    println!("SHAPE_REPRESENTATION:       {}", srs.len());
    println!("NEXT_ASSEMBLY_USAGE_OCCUR:  {}", nauos.len());
    println!("PRODUCT_DEFINITION:         {}", pds_all.len());
    
    println!("\n=== step_structure_lazy ===");
    let (tree, pending) = draper_step::step_structure_lazy(&step_file);
    println!("Root: '{}' ({} children)", tree.name, tree.children.len());
    println!("Pending BREP instances: {}", pending.len());
    for (i, p) in pending.iter().enumerate().take(10) {
        println!("  [{}] BREP#{} '{}' transform:{}", i, p.brep_id, p.name, p.transform.is_some());
    }
    
    // Show first few NAUO entity raw params
    println!("\n=== NAUO samples (first 3) ===");
    for n in nauos.iter().take(3) {
        println!("  NAUO #{}: type={} params={:?}", n.id, n.type_name, n.params);
    }
    
    println!("\n=== BREP samples (first 3) ===");
    for b in breps.iter().take(3) {
        println!("  BREP #{}: type={} params={:?}", b.id, b.type_name, b.params);
    }
    
    // Print tree structure (first 3 levels)
    println!("\n=== Assembly Tree (depth 3) ===");
    print_tree(&tree, 0, 3);
    
    // Manually walk: for each PD (parent of NAUO tree), find PD → PDS → SDR → BREP
    // First, manually extract NAUO (parent, child) pairs
    let mut parent_to_children: std::collections::HashMap<i64, Vec<i64>> = std::collections::HashMap::new();
    let mut child_set: std::collections::HashSet<i64> = std::collections::HashSet::new();
    let mut nauo_pairs: Vec<(i64, i64)> = Vec::new();
    for n in &nauos {
        let mut refs: Vec<i64> = Vec::new();
        for p in &n.params {
            if let Some(r) = get_ref(p) { refs.push(r); }
        }
        // NAUO format: 'name', relating PRODUCT_DEFINITION (parent), related PRODUCT_DEFINITION (child)
        if refs.len() >= 2 {
            let parent = refs[refs.len() - 2];
            let child = refs[refs.len() - 1];
            parent_to_children.entry(parent).or_default().push(child);
            child_set.insert(child);
            nauo_pairs.push((parent, child));
        }
    }
    println!("\n=== NAUO pairs found: {} ===", nauo_pairs.len());
    let parent_set: std::collections::HashSet<i64> = parent_to_children.keys().copied().collect();
    let roots: Vec<i64> = parent_set.difference(&child_set).copied().collect();
    println!("Root PDs: {:?}", roots);
    
    // For each root, walk down and count leaves
    let mut leaves: Vec<i64> = Vec::new();
    for r in &roots {
        collect_leaves(*r, &parent_to_children, &mut leaves, 0);
    }
    println!("Total leaves: {}", leaves.len());
    println!("Sample leaves: {:?}", leaves.iter().take(10).collect::<Vec<_>>());
}

fn collect_leaves(pd: i64, m: &std::collections::HashMap<i64, Vec<i64>>, out: &mut Vec<i64>, depth: usize) {
    if depth > 30 { return; }
    match m.get(&pd) {
        None => out.push(pd),
        Some(children) => {
            for c in children { collect_leaves(*c, m, out, depth + 1); }
        }
    }
}

fn print_tree(node: &draper_step::AssemblyNode, depth: usize, max_depth: usize) {
    if depth > max_depth { return; }
    let indent = "  ".repeat(depth);
    let brep_str = match node.brep_id {
        Some(id) => format!(" BREP#{}", id),
        None => String::new(),
    };
    let inst_str = match node.instance_index {
        Some(idx) => format!(" [inst:{}]", idx),
        None => String::new(),
    };
    let child_str = if node.children.is_empty() { "" } else { &format!(" ({} children)", node.children.len()) };
    println!("{}{}{}{}{}", indent, node.name, brep_str, inst_str, child_str);
    for child in &node.children {
        print_tree(child, depth + 1, max_depth);
    }
}
