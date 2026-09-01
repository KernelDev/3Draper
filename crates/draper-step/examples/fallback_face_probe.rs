// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! D4 diagnostics — dump the full boundary/UV structure of a specific face
//! that hit the 3-tier fallback ("FallbackSurface" in logs), so the root
//! cause of the empty primary triangulation can be pinned down.
//!
//! ```text
//! cargo run -p draper-step --release --example fallback_face_probe -- \
//!     test/8500-02_Vulcan.STEP --solid 6 --face 40443
//! ```

use draper_mesh::{triangulate_face_with_cache, EdgeDiscretizationCache, TriangulationParams};
use draper_step::{extract_solids, parse_step_file};

fn curve_name(c: &draper_geometry::Curve3d) -> &'static str {
    use draper_geometry::Curve3d::*;
    match c {
        Line(_) => "Line",
        Circle(_) => "Circle",
        Ellipse(_) => "Ellipse",
        Nurbs(_) => "Nurbs",
        _ => "Other",
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let path = args
        .iter()
        .find(|a| !a.starts_with("--"))
        .cloned()
        .expect("usage: fallback_face_probe <file.stp> --solid N --face ID");
    let solid_idx: usize = args
        .iter()
        .position(|a| a == "--solid")
        .and_then(|p| args.get(p + 1))
        .and_then(|v| v.parse().ok())
        .expect("--solid N required");
    let face_id: u64 = args
        .iter()
        .position(|a| a == "--face")
        .and_then(|p| args.get(p + 1))
        .and_then(|v| v.parse().ok())
        .expect("--face ID required");

    let step_file = parse_step_file(&path).expect("parse failed");
    let (solids, _ids) = extract_solids(&step_file);
    // Auto-locate the solid containing the requested face id.
    let solid_idx = match solids
        .iter()
        .position(|s| s.faces().iter().any(|f| f.id.to_u64() == face_id))
    {
        Some(found) if found != solid_idx => {
            println!("(face {face_id} not in solid {solid_idx} — auto-located in solid {found})");
            found
        }
        _ => solid_idx,
    };
    println!("{}: {} solids, probing solid {} for face {}", path, solids.len(), solid_idx, face_id);
    let solid = &solids[solid_idx];

    let params = TriangulationParams::default();
    // Replicate `triangulate_solid_with_report` EXACTLY:
    // adaptive tolerance from solid bbox + chord tolerance override,
    // then pre_populate_for_solid(EDGE_SAMPLES=20) — sequential path.
    let mut min = draper_geometry::Point3d::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max = draper_geometry::Point3d::new(f64::MIN, f64::MIN, f64::MIN);
    for face in solid.faces() {
        for edge in &face.edges {
            if edge.degenerate { continue; }
            if let Some(p) = edge.start_point() {
                min.x = min.x.min(p.x); min.y = min.y.min(p.y); min.z = min.z.min(p.z);
                max.x = max.x.max(p.x); max.y = max.y.max(p.y); max.z = max.z.max(p.z);
            }
            if let Some(p) = edge.end_point() {
                min.x = min.x.min(p.x); min.y = min.y.min(p.y); min.z = min.z.min(p.z);
                max.x = max.x.max(p.x); max.y = max.y.max(p.y); max.z = max.z.max(p.z);
            }
        }
    }
    let mut cache = EdgeDiscretizationCache::with_adaptive_tolerance(&min, &max, 64);
    cache.set_chord_tolerance_override(Some(params.max_deviation));
    cache.pre_populate_for_solid(solid, 20);
    let mu = cache.adaptive_tolerance().merge_tolerance();
    println!("cache: merge_tolerance={mu:.3e}");

    let faces = solid.faces();
    let face = faces
        .iter()
        .find(|f| f.id.to_u64() == face_id)
        .unwrap_or_else(|| panic!("face {face_id} not found in solid (ids: {:?})", faces.iter().map(|f| f.id.to_u64()).take(20).collect::<Vec<_>>()));
    println!("\n=== FACE #{} ===", face_id);
    println!("surface: {}", face.surface.as_ref().map(|s| s.type_name()).unwrap_or("None"));
    println!("forward: {}", face.forward);
    if let Some(draper_geometry::Surface::Cone(c)) = &face.surface {
        println!(
            "  cone: origin=({:.4},{:.4},{:.4}) axis=({:.4},{:.4},{:.4}) half_angle={:.6} rad ({:.3}°) radius={:.4} expanding={}",
            c.origin.x, c.origin.y, c.origin.z, c.axis.x, c.axis.y, c.axis.z,
            c.half_angle, c.half_angle.to_degrees(), c.radius, c.expanding
        );
    }
    println!("inner_wires: {}", face.inner_wires.len());

    let wire = face.outer_wire.as_ref().unwrap_or_else(|| panic!("face has NO outer_wire"));
    println!("outer_wire: {} coedges", wire.coedges.len());

    for (ci, co) in wire.coedges.iter().enumerate() {
        let edge = face.edge_by_id(co.edge);
        let Some(edge) = edge else {
            println!("  coedge {ci}: edge id {} NOT RESOLVED via face.edge_by_id", co.edge);
            continue;
        };
        let disc = cache.get(edge.id);
        let uv = disc.and_then(|d| d.uv_per_face.get(&face.id));
        println!(
            "  coedge {ci}: edge={} degenerate={} curve={} param_range=({:.4},{:.4}) forward={} \
             cache_pts={} uv_pts(for this face)={}",
            edge.id,
            edge.degenerate,
            edge.curve.as_ref().map(|c| curve_name(c)).unwrap_or("None"),
            edge.param_range.0,
            edge.param_range.1,
            co.forward,
            disc.map(|d| d.points_3d.len()).unwrap_or(0),
            uv.map(|u| u.len()).unwrap_or(0),
        );
        if let Some(d) = disc {
            let p0 = &d.points_3d[0];
            let pn = &d.points_3d[d.points_3d.len() - 1];
            println!(
            "             3d: first=({:.4},{:.4},{:.4}) last=({:.4},{:.4},{:.4})",
            p0.x, p0.y, p0.z, pn.x, pn.y, pn.z
            );
            if let Some(uvs) = uv {
                let mut umin = f64::MAX; let mut umax = f64::MIN;
                let mut vmin = f64::MAX; let mut vmax = f64::MIN;
                for u in uvs {
                    umin = umin.min(u.u); umax = umax.max(u.u);
                    vmin = vmin.min(u.v); vmax = vmax.max(u.v);
                }
                println!(
                    "             uv: u=[{:.4},{:.4}] (span {:.4}) v=[{:.4},{:.4}] (span {:.4})",
                    umin, umax, umax - umin, vmin, vmax, vmax - vmin
                );
            } else {
                println!("             uv: NO UV FOR THIS FACE (will trigger compute_uvs fallback)");
            }
        }
    }

    // Now triangulate the face the same way the pipeline does:
    let m = triangulate_face_with_cache(face, &params, &mut cache);
    println!(
        "\ntriangulate_face_with_cache → {} vertices, {} triangles",
        m.vertices.len(),
        m.triangles.len()
    );
}
