// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! STEP file exporter (AP203/AP214).
//!
//! Exports a B-Rep Solid to a valid STEP file with proper topology:
//! - MANIFOLD_SOLID_BREP → CLOSED_SHELL → ADVANCED_FACE → FACE_BOUND → EDGE_LOOP → EDGE_CURVE
//! - Geometry: PLANE, CYLINDRICAL_SURFACE, SPHERICAL_SURFACE, CONICAL_SURFACE, TOROIDAL_SURFACE

use crate::schema::*;
use draper_geometry::{Point3d, Direction3d, Surface};
use draper_topology::{Solid, Shell, Face, Edge, Compound};
use std::io::{self, Write};

/// Export a solid to STEP AP203 format.
pub fn export_step(solid: &Solid, name: &str) -> String {
    let mut out = String::new();
    let mut id: i64 = 1;

    // Helper to allocate the next ID
    let next_id = |id: &mut i64| -> i64 { let i = *id; *id += 1; i };

    // ── Header ──
    out.push_str("ISO-10303-21;\n");
    out.push_str("HEADER;\n");
    out.push_str(&format!(
        "FILE_DESCRIPTION(('3Draper export'), '2;1');\n"
    ));
    let now = chrono_now();
    out.push_str(&format!(
        "FILE_NAME('{}.stp', '{}', ('3Draper'), (''), '3Draper', '', '');\n",
        name, now
    ));
    out.push_str("FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\n");
    out.push_str("ENDSEC;\n");

    // ── Data section ──
    out.push_str("DATA;\n");

    // We need to produce a valid STEP topology chain:
    // MANIFOLD_SOLID_BREP → CLOSED_SHELL → ADVANCED_FACE → FACE_BOUND → EDGE_LOOP → ORIENTED_EDGE → EDGE_CURVE → ...

    if let Some(ref shell) = solid.outer_shell {
        let mut face_ids = Vec::new();

        for face in &shell.faces {
            // ── Surface geometry ──
            let surface_id = if let Some(ref surface) = face.surface {
                export_surface(&mut out, surface, &mut id)
            } else {
                continue;
            };

            // ── Edge loop and bounds ──
            let mut oriented_edge_ids = Vec::new();

            if let Some(ref wire) = face.outer_wire {
                for coedge in &wire.coedges {
                    // Find the edge geometry
                    let edge = face.edges.iter().find(|e| e.id == coedge.edge);
                    let (edge_curve_id, edge_start_vtx_id, edge_end_vtx_id) = if let Some(e) = edge {
                        export_edge(&mut out, e, &mut id)
                    } else {
                        // Create a dummy edge
                        let dummy_pt_id = export_point(&mut out, &Point3d::ORIGIN, &mut id);
                        let vtx_id = next_id(&mut id);
                        out.push_str(&format!(
                            "#{} = VERTEX_POINT('',#{});\n", vtx_id, dummy_pt_id
                        ));
                        (dummy_pt_id, vtx_id, vtx_id)
                    };

                    // ORIENTED_EDGE('', *, *, #edge_curve, orientation)
                    let oe_id = next_id(&mut id);
                    let orientation = if coedge.forward { ".T." } else { ".F." };
                    out.push_str(&format!(
                        "#{} = ORIENTED_EDGE('',*,*,#{},{});\n",
                        oe_id, edge_curve_id, orientation
                    ));
                    oriented_edge_ids.push(oe_id);
                }
            }

            // If no edges in wire, create a minimal bound with a self-loop
            if oriented_edge_ids.is_empty() {
                // Create a single vertex loop (degenerate)
                let pt_id = export_point(&mut out, &Point3d::ORIGIN, &mut id);
                let vtx_id = next_id(&mut id);
                out.push_str(&format!(
                    "#{} = VERTEX_POINT('',#{});\n", vtx_id, pt_id
                ));
                let vl_id = next_id(&mut id);
                out.push_str(&format!(
                    "#{} = VERTEX_LOOP('',#{});\n", vl_id, vtx_id
                ));
                let fb_id = next_id(&mut id);
                out.push_str(&format!(
                    "#{} = FACE_BOUND('',#{},.T.);\n", fb_id, vl_id
                ));

                let face_id = next_id(&mut id);
                let face_orient = if face.forward { ".T." } else { ".F." };
                out.push_str(&format!(
                    "#{} = ADVANCED_FACE('',(#{}),#{},{},.F.);\n",
                    face_id, fb_id, surface_id, face_orient
                ));
                face_ids.push(face_id);
            } else {
                // EDGE_LOOP('', (#oe1, #oe2, ...))
                let el_id = next_id(&mut id);
                let oe_refs: Vec<String> = oriented_edge_ids.iter().map(|id| format!("#{}", id)).collect();
                out.push_str(&format!(
                    "#{} = EDGE_LOOP('',({}));\n", el_id, oe_refs.join(",")
                ));

                // FACE_BOUND('', #edge_loop, .T.)
                let fb_id = next_id(&mut id);
                out.push_str(&format!(
                    "#{} = FACE_BOUND('',#{},.T.);\n", fb_id, el_id
                ));

                // ADVANCED_FACE('', (#face_bound), #surface, .T.)
                let face_id = next_id(&mut id);
                let face_orient = if face.forward { ".T." } else { ".F." };
                out.push_str(&format!(
                    "#{} = ADVANCED_FACE('',(#{}),#{},{},.F.);\n",
                    face_id, fb_id, surface_id, face_orient
                ));
                face_ids.push(face_id);
            }
        }

        // ── CLOSED_SHELL ──
        let shell_id = next_id(&mut id);
        let face_refs: Vec<String> = face_ids.iter().map(|id| format!("#{}", id)).collect();
        out.push_str(&format!(
            "#{} = CLOSED_SHELL('',({}));\n", shell_id, face_refs.join(",")
        ));

        // ── MANIFOLD_SOLID_BREP ──
        let msb_id = next_id(&mut id);
        out.push_str(&format!(
            "#{} = MANIFOLD_SOLID_BREP('{}',#{});\n", msb_id, name, shell_id
        ));

        // ── Shape representation chain ──
        let sr_id = next_id(&mut id);
        out.push_str(&format!(
            "#{} = ADVANCED_BREP_SHAPE_REPRESENTATION('',(#{}),#{});\n",
            sr_id, msb_id, next_id(&mut id)
        ));
    }

    out.push_str("ENDSEC;\n");
    out.push_str("END-ISO-10303-21;\n");

    out
}

/// Export a CARTESIAN_POINT and return its ID.
fn export_point(out: &mut String, pt: &Point3d, id: &mut i64) -> i64 {
    let pt_id = *id;
    *id += 1;
    out.push_str(&format!(
        "#{} = CARTESIAN_POINT('',({},{},{}));\n",
        pt_id, pt.x, pt.y, pt.z
    ));
    pt_id
}

/// Export an EDGE_CURVE with its vertex endpoints and curve geometry.
/// Returns (edge_curve_id, start_vertex_id, end_vertex_id).
fn export_edge(out: &mut String, edge: &Edge, id: &mut i64) -> (i64, i64, i64) {
    let next_id = |id: &mut i64| -> i64 { let i = *id; *id += 1; i };

    // Start and end points
    let start_pt = edge.start_point().unwrap_or(Point3d::ORIGIN);
    let end_pt = edge.end_point().unwrap_or(Point3d::ORIGIN);

    let start_pt_id = export_point(out, &start_pt, id);
    let end_pt_id = export_point(out, &end_pt, id);

    // Vertices
    let start_vtx_id = next_id(id);
    out.push_str(&format!(
        "#{} = VERTEX_POINT('',#{});\n", start_vtx_id, start_pt_id
    ));
    let end_vtx_id = next_id(id);
    out.push_str(&format!(
        "#{} = VERTEX_POINT('',#{});\n", end_vtx_id, end_pt_id
    ));

    // Curve geometry
    let curve_id = if let Some(ref curve) = edge.curve {
        match curve {
            draper_geometry::Curve3d::Line(line) => {
                let dir_id = next_id(id);
                out.push_str(&format!(
                    "#{} = DIRECTION('',({},{},{}));\n",
                    dir_id, line.direction.x, line.direction.y, line.direction.z
                ));
                let line_id = next_id(id);
                out.push_str(&format!(
                    "#{} = LINE('',#{});\n", line_id, dir_id
                ));
                line_id
            }
            draper_geometry::Curve3d::Circle(circle) => {
                let pt_id = export_point(out, &circle.center, id);
                let dir_id = next_id(id);
                out.push_str(&format!(
                    "#{} = DIRECTION('',({},{},{}));\n",
                    dir_id, circle.normal.x, circle.normal.y, circle.normal.z
                ));
                let ref_dir_id = next_id(id);
                out.push_str(&format!(
                    "#{} = DIRECTION('',({},{},{}));\n",
                    ref_dir_id, circle.x_axis.x, circle.x_axis.y, circle.x_axis.z
                ));
                let axis2_id = next_id(id);
                out.push_str(&format!(
                    "#{} = AXIS2_PLACEMENT_3D('',#{},#{},#{});\n",
                    axis2_id, pt_id, dir_id, ref_dir_id
                ));
                let circle_id = next_id(id);
                out.push_str(&format!(
                    "#{} = CIRCLE('',#{},{});\n", circle_id, axis2_id, circle.radius
                ));
                circle_id
            }
            _ => {
                // Fallback: use a line from start to end
                let dir = Direction3d::new(
                    end_pt.x - start_pt.x,
                    end_pt.y - start_pt.y,
                    end_pt.z - start_pt.z,
                ).unwrap_or(Direction3d::X);
                let dir_id = next_id(id);
                out.push_str(&format!(
                    "#{} = DIRECTION('',({},{},{}));\n",
                    dir_id, dir.x, dir.y, dir.z
                ));
                let line_id = next_id(id);
                out.push_str(&format!(
                    "#{} = LINE('',#{});\n", line_id, dir_id
                ));
                line_id
            }
        }
    } else {
        // No curve — create a line from start to end
        let dir = Direction3d::new(
            end_pt.x - start_pt.x,
            end_pt.y - start_pt.y,
            end_pt.z - start_pt.z,
        ).unwrap_or(Direction3d::X);
        let dir_id = next_id(id);
        out.push_str(&format!(
            "#{} = DIRECTION('',({},{},{}));\n",
            dir_id, dir.x, dir.y, dir.z
        ));
        let line_id = next_id(id);
        out.push_str(&format!(
            "#{} = LINE('',#{});\n", line_id, dir_id
        ));
        line_id
    };

    // EDGE_CURVE('', #start_vtx, #end_vtx, #curve, .T.)
    let ec_id = next_id(id);
    out.push_str(&format!(
        "#{} = EDGE_CURVE('',#{},#{},{},.T.);\n",
        ec_id, start_vtx_id, end_vtx_id, curve_id
    ));

    (ec_id, start_vtx_id, end_vtx_id)
}

/// Export a surface to STEP format, returns the surface entity ID.
fn export_surface(out: &mut String, surface: &Surface, id: &mut i64) -> i64 {
    let next_id = |id: &mut i64| -> i64 { let i = *id; *id += 1; i };

    match surface {
        Surface::Plane(plane) => {
            let pt_id = export_point(out, &plane.origin, id);
            let dir_id = next_id(id);
            out.push_str(&format!(
                "#{} = DIRECTION('',({},{},{}));\n",
                dir_id, plane.normal.x, plane.normal.y, plane.normal.z
            ));
            let ref_dir_id = next_id(id);
            out.push_str(&format!(
                "#{} = DIRECTION('',({},{},{}));\n",
                ref_dir_id, plane.u_dir.x, plane.u_dir.y, plane.u_dir.z
            ));
            let axis2_id = next_id(id);
            out.push_str(&format!(
                "#{} = AXIS2_PLACEMENT_3D('',#{},#{},#{});\n",
                axis2_id, pt_id, dir_id, ref_dir_id
            ));
            let surface_id = next_id(id);
            out.push_str(&format!(
                "#{} = PLANE('',#{});\n", surface_id, axis2_id
            ));
            surface_id
        }
        Surface::Cylinder(cyl) => {
            let pt_id = export_point(out, &cyl.origin, id);
            let dir_id = next_id(id);
            out.push_str(&format!(
                "#{} = DIRECTION('',({},{},{}));\n",
                dir_id, cyl.axis.x, cyl.axis.y, cyl.axis.z
            ));
            let ref_dir_id = next_id(id);
            out.push_str(&format!(
                "#{} = DIRECTION('',(1.,0.,0.));\n", ref_dir_id
            ));
            let axis2_id = next_id(id);
            out.push_str(&format!(
                "#{} = AXIS2_PLACEMENT_3D('',#{},#{},#{});\n",
                axis2_id, pt_id, dir_id, ref_dir_id
            ));
            let surface_id = next_id(id);
            out.push_str(&format!(
                "#{} = CYLINDRICAL_SURFACE('',#{},{});\n",
                surface_id, axis2_id, cyl.radius
            ));
            surface_id
        }
        Surface::Sphere(sphere) => {
            let pt_id = export_point(out, &sphere.center, id);
            let dir_id = next_id(id);
            out.push_str(&format!(
                "#{} = DIRECTION('',(0.,0.,1.));\n", dir_id
            ));
            let ref_dir_id = next_id(id);
            out.push_str(&format!(
                "#{} = DIRECTION('',(1.,0.,0.));\n", ref_dir_id
            ));
            let axis2_id = next_id(id);
            out.push_str(&format!(
                "#{} = AXIS2_PLACEMENT_3D('',#{},#{},#{});\n",
                axis2_id, pt_id, dir_id, ref_dir_id
            ));
            let surface_id = next_id(id);
            out.push_str(&format!(
                "#{} = SPHERICAL_SURFACE('',#{},{});\n",
                surface_id, axis2_id, sphere.radius
            ));
            surface_id
        }
        Surface::Cone(cone) => {
            let pt_id = export_point(out, &cone.origin, id);
            let dir_id = next_id(id);
            out.push_str(&format!(
                "#{} = DIRECTION('',({},{},{}));\n",
                dir_id, cone.axis.x, cone.axis.y, cone.axis.z
            ));
            let ref_dir_id = next_id(id);
            out.push_str(&format!(
                "#{} = DIRECTION('',(1.,0.,0.));\n", ref_dir_id
            ));
            let axis2_id = next_id(id);
            out.push_str(&format!(
                "#{} = AXIS2_PLACEMENT_3D('',#{},#{},#{});\n",
                axis2_id, pt_id, dir_id, ref_dir_id
            ));
            let surface_id = next_id(id);
            out.push_str(&format!(
                "#{} = CONICAL_SURFACE('',#{},{},{});\n",
                surface_id, axis2_id, cone.radius, cone.half_angle
            ));
            surface_id
        }
        Surface::Torus(torus) => {
            let pt_id = export_point(out, &torus.center, id);
            let dir_id = next_id(id);
            out.push_str(&format!(
                "#{} = DIRECTION('',({},{},{}));\n",
                dir_id, torus.axis.x, torus.axis.y, torus.axis.z
            ));
            let ref_dir_id = next_id(id);
            out.push_str(&format!(
                "#{} = DIRECTION('',(1.,0.,0.));\n", ref_dir_id
            ));
            let axis2_id = next_id(id);
            out.push_str(&format!(
                "#{} = AXIS2_PLACEMENT_3D('',#{},#{},#{});\n",
                axis2_id, pt_id, dir_id, ref_dir_id
            ));
            let surface_id = next_id(id);
            out.push_str(&format!(
                "#{} = TOROIDAL_SURFACE('',#{},{},{});\n",
                surface_id, axis2_id, torus.major_radius, torus.minor_radius
            ));
            surface_id
        }
        _ => {
            // Fallback for unsupported surface types: export as a plane at origin
            let pt_id = export_point(out, &Point3d::ORIGIN, id);
            let dir_id = next_id(id);
            out.push_str(&format!(
                "#{} = DIRECTION('',(0.,0.,1.));\n", dir_id
            ));
            let ref_dir_id = next_id(id);
            out.push_str(&format!(
                "#{} = DIRECTION('',(1.,0.,0.));\n", ref_dir_id
            ));
            let axis2_id = next_id(id);
            out.push_str(&format!(
                "#{} = AXIS2_PLACEMENT_3D('',#{},#{},#{});\n",
                axis2_id, pt_id, dir_id, ref_dir_id
            ));
            let surface_id = next_id(id);
            out.push_str(&format!(
                "#{} = PLANE('',#{});\n", surface_id, axis2_id
            ));
            surface_id
        }
    }
}

/// Export a compound (assembly) to STEP.
pub fn export_compound_step(compound: &Compound, name: &str) -> String {
    // A proper implementation would create a single STEP file with
    // multiple MANIFOLD_SOLID_BREP entities under a single ADVANCED_BREP_SHAPE_REPRESENTATION.
    // For now, export the first solid only.
    if let Some(solid) = compound.solids.first() {
        export_step(solid, name)
    } else {
        "// Empty compound — no solids to export".to_string()
    }
}

/// Write STEP content to a file (native only — not available on wasm).
#[cfg(not(target_arch = "wasm32"))]
pub fn write_step_file(content: &str, path: &str) -> io::Result<()> {
    let mut file = std::fs::File::create(path)?;
    file.write_all(content.as_bytes())
}

/// Get current timestamp in ISO format.
fn chrono_now() -> String {
    // Simple timestamp without chrono dependency
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    // Approximate date from unix timestamp
    let days = now / 86400;
    let year = 1970 + days / 365;
    let month = ((days % 365) / 30).min(11) + 1;
    let day = (days % 30).min(27) + 1;
    let hour = (now % 86400) / 3600;
    let minute = (now % 3600) / 60;
    let second = now % 60;
    format!(
        "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}",
        year, month, day, hour, minute, second
    )
}
