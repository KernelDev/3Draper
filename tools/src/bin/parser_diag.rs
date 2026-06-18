//! Check what parser captures

fn main() {
    let path = std::env::args().nth(1).unwrap_or("test/Zentralstaender.stp".to_string());
    let content = std::fs::read_to_string(&path).expect("read");
    let step_file = draper_step::parser::parse_step(&content).expect("parse");
    
    println!("Total entities: {}", step_file.entities.len());
    
    let mut ids: Vec<i64> = step_file.entities.iter().map(|e| e.id).collect();
    ids.sort();
    
    // Find gaps in ID sequence
    println!("\nFirst 20 IDs: {:?}", &ids.iter().take(20).copied().collect::<Vec<_>>());
    println!("Last 20 IDs:  {:?}", &ids.iter().rev().take(20).copied().collect::<Vec<_>>());
    
    // Show what types appear in first 30 and last 30 entities
    println!("\nFirst 30 (id, type):");
    for e in step_file.entities.iter().take(30) {
        println!("  #{} = {}", e.id, e.type_name);
    }
    
    println!("\nLast 30 (id, type):");
    for e in step_file.entities.iter().rev().take(30) {
        println!("  #{} = {}", e.id, e.type_name);
    }
    
    // Show range
    let min_id = ids.first().copied().unwrap_or(0);
    let max_id = ids.last().copied().unwrap_or(0);
    println!("\nID range: {} to {} (span = {})", min_id, max_id, max_id - min_id);
    
    // Check the raw file - what's the last ID?
    let mut max_file_id = 0i64;
    for line in content.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with('#') {
            if let Some(end) = trimmed.find('=') {
                let id_str = trimmed[1..end].trim();
                if let Ok(id) = id_str.parse::<i64>() {
                    if id > max_file_id { max_file_id = id; }
                }
            }
        }
    }
    println!("Max ID in raw file: {}", max_file_id);
}
