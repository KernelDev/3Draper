// Standalone test to check STEP uncertainty extraction
use draper_step::{parser::parse_step, extract_step_tolerance};

fn main() {
    let content = std::fs::read_to_string("test/brick_thin_round.stp").expect("read");
    let step_file = parse_step(&content).expect("parse");
    let tol = extract_step_tolerance(&step_file);
    println!("brick_thin_round uncertainty: {:?}", tol);

    let content = std::fs::read_to_string("test/brick_thin.stp").expect("read");
    let step_file = parse_step(&content).expect("parse");
    let tol = extract_step_tolerance(&step_file);
    println!("brick_thin uncertainty: {:?}", tol);

    let content = std::fs::read_to_string("test/3.05.078.stp").expect("read");
    let step_file = parse_step(&content).expect("parse");
    let tol = extract_step_tolerance(&step_file);
    println!("3.05.078 uncertainty: {:?}", tol);
}
