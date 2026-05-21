fn main() {
    env_logger::init();
    
    let files = ["test/3.05.078.stp", "test/SampleCube.step"];
    
    for file in &files {
        println!("\n=== Testing {} ===", file);
        let content = std::fs::read_to_string(file).unwrap();
        println!("File size: {} bytes", content.len());
        
        match draper_step::parse_step(&content) {
            Ok(doc) => {
                println!("✓ Parsed successfully!");
                println!("  Header schema: {:?}", doc.header.file_schema.schemas);
                println!("  Entities: {}", doc.entities.len());
                
                // Show some entity types
                let mut type_counts = std::collections::HashMap::new();
                for e in doc.entities.values() {
                    *type_counts.entry(e.type_name.clone()).or_insert(0) += 1;
                }
                let mut types: Vec<_> = type_counts.iter().collect();
                types.sort_by(|a, b| b.1.cmp(a.1));
                println!("  Entity types:");
                for (t, c) in types.iter().take(15) {
                    println!("    {} x{}", t, c);
                }
                
                // Show structure tree
                let tree = doc.structure_tree();
                fn show_tree(node: &draper_step::ast::StructureNode, depth: usize) {
                    let indent = "  ".repeat(depth);
                    println!("{}{} [{}]", indent, node.name, node.type_name);
                    for child in &node.children {
                        show_tree(child, depth + 1);
                    }
                }
                println!("  Structure tree:");
                show_tree(&tree, 2);
            }
            Err(e) => {
                println!("✗ Parse error: {}", e);
            }
        }
    }
}
