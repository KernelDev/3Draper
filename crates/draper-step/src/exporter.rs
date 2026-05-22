//! STEP file exporter.

use crate::schema::*;
use draper_geometry::{Point3d, Direction3d, Surface};
use draper_topology::{Solid, Shell, Face, Edge, Compound};
use std::io::{self, Write};

/// Export a solid to STEP AP203 format.
pub fn export_step(solid: &Solid, name: &str) -> String {
    let mut out = String::new();
    let mut id_counter: i64 = 1;

    // Header
    out.push_str("ISO-10303-21;\n");
    out.push_str("HEADER;\n");
    out.push_str(&format!("FILE_DESCRIPTION(('{}'), '2;1');\n", name));
    out.push_str(&format!("FILE_NAME('{}.stp', '2024-01-01T00:00:00', ('3Draper'), (''), '3Draper', '', '');\n", name));
    out.push_str("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 103 214 2 1 1 }'));\n");
    out.push_str("ENDSEC;\n");

    // Data section
    out.push_str("DATA;\n");

    // Root entities
    let shape_def_id = alloc_id(&mut id_counter);
    let _shape_rep_id = alloc_id(&mut id_counter);

    // Shape definition
    let pd_id = alloc_id(&mut id_counter);
    out.push_str(&format!("#{} = PRODUCT_DEFINITION_SHAPE('',$,#{});\n", shape_def_id, pd_id));

    // Shell
    let shell_id = alloc_id(&mut id_counter);
    let mut face_ids = Vec::new();

    if let Some(ref shell) = solid.outer_shell {
        for face in &shell.faces {
            let face_id = alloc_id(&mut id_counter);
            face_ids.push(face_id);

            if let Some(ref surface) = face.surface {
                let surface_id = export_surface(&mut out, surface, &mut id_counter);

                // Advanced face
                let orientation = if face.forward { ".T." } else { ".F." };
                out.push_str(&format!(
                    "#{} = ADVANCED_FACE('',({}),#{},{},.F.);\n",
                    face_id, "", surface_id, orientation
                ));
            }
        }
    }

    // Closed shell
    let face_refs: Vec<String> = face_ids.iter().map(|id| format!("#{}", id)).collect();
    out.push_str(&format!(
        "#{} = CLOSED_SHELL('',({}));\n",
        shell_id, face_refs.join(",")
    ));

    // Manifold solid brep
    let msb_id = alloc_id(&mut id_counter);
    out.push_str(&format!(
        "#{} = MANIFOLD_SOLID_BREP('{}',#{});\n",
        msb_id, name, shell_id
    ));

    out.push_str("ENDSEC;\n");
    out.push_str("END-ISO-10303-21;\n");

    out
}

fn alloc_id(counter: &mut i64) -> i64 {
    let id = *counter;
    *counter += 1;
    id
}

/// Export a surface to STEP format, returns the surface entity ID.
fn export_surface(out: &mut String, surface: &Surface, id_counter: &mut i64) -> i64 {
    match surface {
        Surface::Plane(plane) => {
            let pt_id = alloc_id(id_counter);
            out.push_str(&format!(
                "#{} = CARTESIAN_POINT('',({},{},{}));\n",
                pt_id, plane.origin.x, plane.origin.y, plane.origin.z
            ));

            let dir_id = alloc_id(id_counter);
            out.push_str(&format!(
                "#{} = DIRECTION('',({},{},{}));\n",
                dir_id, plane.normal.x, plane.normal.y, plane.normal.z
            ));

            let ref_dir_id = alloc_id(id_counter);
            out.push_str(&format!(
                "#{} = DIRECTION('',({},{},{}));\n",
                ref_dir_id, plane.u_dir.x, plane.u_dir.y, plane.u_dir.z
            ));

            let axis2_id = alloc_id(id_counter);
            out.push_str(&format!(
                "#{} = AXIS2_PLACEMENT_3D('',#{},#{},#{});\n",
                axis2_id, pt_id, dir_id, ref_dir_id
            ));

            let surface_id = alloc_id(id_counter);
            out.push_str(&format!(
                "#{} = PLANE('',#{});\n",
                surface_id, axis2_id
            ));

            surface_id
        }
        Surface::Cylinder(cyl) => {
            let pt_id = alloc_id(id_counter);
            out.push_str(&format!(
                "#{} = CARTESIAN_POINT('',({},{},{}));\n",
                pt_id, cyl.origin.x, cyl.origin.y, cyl.origin.z
            ));

            let dir_id = alloc_id(id_counter);
            out.push_str(&format!(
                "#{} = DIRECTION('',({},{},{}));\n",
                dir_id, cyl.axis.x, cyl.axis.y, cyl.axis.z
            ));

            let ref_dir_id = alloc_id(id_counter);
            out.push_str(&format!(
                "#{} = DIRECTION('',(1.,0.,0.));\n", ref_dir_id
            ));

            let axis2_id = alloc_id(id_counter);
            out.push_str(&format!(
                "#{} = AXIS2_PLACEMENT_3D('',#{},#{},#{});\n",
                axis2_id, pt_id, dir_id, ref_dir_id
            ));

            let surface_id = alloc_id(id_counter);
            out.push_str(&format!(
                "#{} = CYLINDRICAL_SURFACE('',#{},{});\n",
                surface_id, axis2_id, cyl.radius
            ));

            surface_id
        }
        Surface::Sphere(sphere) => {
            let pt_id = alloc_id(id_counter);
            out.push_str(&format!(
                "#{} = CARTESIAN_POINT('',({},{},{}));\n",
                pt_id, sphere.center.x, sphere.center.y, sphere.center.z
            ));

            let dir_id = alloc_id(id_counter);
            out.push_str(&format!(
                "#{} = DIRECTION('',(0.,0.,1.));\n", dir_id
            ));

            let ref_dir_id = alloc_id(id_counter);
            out.push_str(&format!(
                "#{} = DIRECTION('',(1.,0.,0.));\n", ref_dir_id
            ));

            let axis2_id = alloc_id(id_counter);
            out.push_str(&format!(
                "#{} = AXIS2_PLACEMENT_3D('',#{},#{},#{});\n",
                axis2_id, pt_id, dir_id, ref_dir_id
            ));

            let surface_id = alloc_id(id_counter);
            out.push_str(&format!(
                "#{} = SPHERICAL_SURFACE('',#{},{});\n",
                surface_id, axis2_id, sphere.radius
            ));

            surface_id
        }
        Surface::Torus(torus) => {
            let pt_id = alloc_id(id_counter);
            out.push_str(&format!(
                "#{} = CARTESIAN_POINT('',({},{},{}));\n",
                pt_id, torus.center.x, torus.center.y, torus.center.z
            ));

            let dir_id = alloc_id(id_counter);
            out.push_str(&format!(
                "#{} = DIRECTION('',({},{},{}));\n",
                dir_id, torus.axis.x, torus.axis.y, torus.axis.z
            ));

            let ref_dir_id = alloc_id(id_counter);
            out.push_str(&format!(
                "#{} = DIRECTION('',(1.,0.,0.));\n", ref_dir_id
            ));

            let axis2_id = alloc_id(id_counter);
            out.push_str(&format!(
                "#{} = AXIS2_PLACEMENT_3D('',#{},#{},#{});\n",
                axis2_id, pt_id, dir_id, ref_dir_id
            ));

            let surface_id = alloc_id(id_counter);
            out.push_str(&format!(
                "#{} = TOROIDAL_SURFACE('',#{},{},{});\n",
                surface_id, axis2_id, torus.major_radius, torus.minor_radius
            ));

            surface_id
        }
        Surface::Cone(cone) => {
            let pt_id = alloc_id(id_counter);
            out.push_str(&format!(
                "#{} = CARTESIAN_POINT('',({},{},{}));\n",
                pt_id, cone.origin.x, cone.origin.y, cone.origin.z
            ));

            let dir_id = alloc_id(id_counter);
            out.push_str(&format!(
                "#{} = DIRECTION('',({},{},{}));\n",
                dir_id, cone.axis.x, cone.axis.y, cone.axis.z
            ));

            let ref_dir_id = alloc_id(id_counter);
            out.push_str(&format!(
                "#{} = DIRECTION('',(1.,0.,0.));\n", ref_dir_id
            ));

            let axis2_id = alloc_id(id_counter);
            out.push_str(&format!(
                "#{} = AXIS2_PLACEMENT_3D('',#{},#{},#{});\n",
                axis2_id, pt_id, dir_id, ref_dir_id
            ));

            let surface_id = alloc_id(id_counter);
            out.push_str(&format!(
                "#{} = CONICAL_SURFACE('',#{},{},{});\n",
                surface_id, axis2_id, cone.radius, cone.half_angle
            ));

            surface_id
        }
        _ => {
            // Fallback: treat as plane
            let surface_id = alloc_id(id_counter);
            out.push_str(&format!(
                "#{} = PLANE('',#1);\n", surface_id
            ));
            surface_id
        }
    }
}

/// Export a compound (assembly) to STEP.
pub fn export_compound_step(compound: &Compound, name: &str) -> String {
    let mut parts = Vec::new();
    for solid in &compound.solids {
        parts.push(export_step(solid, name));
    }
    // Simplified: just concatenate
    // A proper implementation would create a single STEP file with multiple shapes
    parts.join("\n")
}

/// Write STEP to file.
pub fn write_step_file(content: &str, path: &str) -> io::Result<()> {
    let mut file = std::fs::File::create(path)?;
    file.write_all(content.as_bytes())
}
