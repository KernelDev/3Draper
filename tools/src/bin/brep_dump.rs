//! Dump the complete face/loop/edge/vertex structure of a small BREP for
//! manual inspection: every face, every loop with its ORIENTED_EDGEs
//! (traversal direction + which endpoint of the EDGE_CURVE), every
//! EDGE_CURVE with its vertices and geometry type, every VERTEX_POINT.

use draper_step::parser::parse_step;
use draper_step::schema::{StepEntity, StepFile, StepValue};
use std::collections::HashMap;

fn refs_of(e: &StepEntity) -> Vec<i64> {
    let mut out = Vec::new();
    for p in &e.params {
        collect_refs(p, &mut out);
    }
    out
}

fn collect_refs(v: &StepValue, out: &mut Vec<i64>) {
    if let StepValue::Ref(r) = v {
        out.push(*r);
    } else if let StepValue::List(items) = v {
        for it in items {
            collect_refs(it, out);
        }
    }
}

fn vertex_position(step: &StepFile, vp_id: i64) -> Option<[f64; 3]> {
    let vp = step.find_entity(vp_id)?;
    for r in refs_of(vp) {
        if let Some(cp) = step.find_entity(r) {
            if cp.type_name == "CARTESIAN_POINT" {
                let mut xyz = [0.0f64; 3];
                let mut n = 0;
                for p in &cp.params {
                    if let StepValue::List(items) = p {
                        for it in items {
                            if let StepValue::Float(f) = it {
                                if n < 3 {
                                    xyz[n] = *f;
                                    n += 1;
                                }
                            }
                        }
                    }
                }
                if n >= 3 {
                    return Some(xyz);
                }
            }
        }
    }
    None
}

fn brep_face_ids(step: &StepFile, brep_id: i64) -> Vec<i64> {
    let brep = step.find_entity(brep_id).expect("brep entity");
    let mut faces = Vec::new();
    for shell_ref in refs_of(brep) {
        if let Some(shell) = step.find_entity(shell_ref) {
            if shell.type_name.contains("SHELL") {
                for f in refs_of(shell) {
                    if let Some(face) = step.find_entity(f) {
                        if face.type_name == "ADVANCED_FACE" {
                            faces.push(f);
                        }
                    }
                }
            }
        }
    }
    faces
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let file = args.iter().find(|s| s.ends_with(".stp") || s.ends_with(".step"))
        .cloned().unwrap_or_else(|| "test/as1-oc-214.stp".to_string());
    let brep_id: i64 = args.iter().filter(|s| !s.contains('.'))
        .filter_map(|s| s.parse().ok()).next().unwrap_or(1190);

    let content = std::fs::read_to_string(&file).expect("read");
    let step = parse_step(&content).expect("parse");
    let faces = brep_face_ids(&step, brep_id);
    println!("BREP #{} — {} faces: {:?}", brep_id, faces.len(), faces);

    // Dump each face's bounds and loops
    for &face_id in &faces {
        let face = step.find_entity(face_id).expect("face");
        // surface
        let mut surf_desc = String::new();
        for r in refs_of(face) {
            if let Some(s) = step.find_entity(r) {
                if s.type_name.contains("SURFACE") || s.type_name.contains("PLANE") || s.type_name.contains("CYLINDR") {
                    surf_desc = format!("#{} {}", r, s.type_name);
                }
            }
        }
        println!("\nFACE #{} surface=[{}] same_sense={:?}", face_id, surf_desc,
            face.params.iter().rev().find_map(|p| if let StepValue::Enum(e) = p { Some(e.clone()) } else { None }));
        for bound_ref in refs_of(face) {
            let bound = match step.find_entity(bound_ref) { Some(b) => b, None => continue };
            if bound.type_name != "FACE_BOUND" && bound.type_name != "FACE_OUTER_BOUND" { continue; }
            let orientation = bound.params.iter().rev().find_map(|p| if let StepValue::Enum(e) = p { Some(e.clone()) } else { None });
            println!("  {} #{} orient={:?}", bound.type_name, bound_ref, orientation);
            for loop_ref in refs_of(bound) {
                let lp = match step.find_entity(loop_ref) { Some(l) => l, None => continue };
                println!("    {} #{} ({} oriented edges)", lp.type_name, loop_ref, refs_of(lp).len());
                for oe_ref in refs_of(lp) {
                    let oe = match step.find_entity(oe_ref) { Some(o) => o, None => continue };
                    let mut orient = "?".to_string();
                    for p in &oe.params {
                        if let StepValue::Enum(e) = p { orient = e.clone(); }
                    }
                    for r in refs_of(oe) {
                        if let Some(ec) = step.find_entity(r) {
                            if ec.type_name == "EDGE_CURVE" {
                                // vertices
                                let mut vs = Vec::new();
                                for p in &ec.params {
                                    if let StepValue::Ref(vr) = p {
                                        if let Some(v) = step.find_entity(*vr) {
                                            if v.type_name == "VERTEX_POINT" {
                                                let pos = vertex_position(&step, *vr)
                                                    .map(|p| format!("({:.2},{:.2},{:.2})", p[0], p[1], p[2]))
                                                    .unwrap_or_else(|| "?".into());
                                                vs.push(format!("#{}{}", vr, pos));
                                            }
                                        }
                                    }
                                }
                                // geometry type
                                let mut geo = "?".to_string();
                                for p in &ec.params {
                                    if let StepValue::Ref(gr) = p {
                                        if let Some(g) = step.find_entity(*gr) {
                                            if !g.type_name.is_empty() && g.type_name != "VERTEX_POINT" {
                                                geo = format!("#{} {}", gr, g.type_name);
                                            }
                                        }
                                    }
                                }
                                println!("      ORIENTED_EDGE #{} orient={} → EDGE_CURVE #{} [{}] v: {}",
                                    oe_ref, orient, r, geo, vs.join(" → "));
                            }
                        }
                    }
                }
            }
        }
    }
}
