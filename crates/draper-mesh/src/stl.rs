//! STL file export.

use crate::mesh::TriangleMesh;
use std::io::{self, Write};

/// Export a triangle mesh to ASCII STL format.
pub fn export_stl_ascii(mesh: &TriangleMesh, name: &str) -> String {
    let mut out = String::new();
    out.push_str(&format!("solid {}\n", name));

    for (i, tri) in mesh.triangles.iter().enumerate() {
        let normal = mesh.face_normals.as_ref()
            .map(|normals| normals[i])
            .unwrap_or([0.0, 0.0, 1.0]);

        out.push_str(&format!("  facet normal {} {} {}\n", normal[0], normal[1], normal[2]));
        out.push_str("    outer loop\n");

        for &idx in tri {
            let v = &mesh.vertices[idx as usize];
            out.push_str(&format!("      vertex {} {} {}\n", v.x, v.y, v.z));
        }

        out.push_str("    endloop\n");
        out.push_str("  endfacet\n");
    }

    out.push_str(&format!("endsolid {}\n", name));
    out
}

/// Export a triangle mesh to binary STL format.
pub fn export_stl_binary(mesh: &TriangleMesh, name: &str) -> Vec<u8> {
    let mut buf = Vec::new();

    // Header: 80 bytes
    let header = format!("3Draper STL - {}", name);
    let header_bytes = header.as_bytes();
    buf.extend_from_slice(header_bytes);
    buf.resize(80, 0);

    // Number of triangles (u32 LE)
    buf.extend_from_slice(&(mesh.triangles.len() as u32).to_le_bytes());

    for (i, tri) in mesh.triangles.iter().enumerate() {
        let normal = mesh.face_normals.as_ref()
            .map(|normals| normals[i])
            .unwrap_or([0.0, 0.0, 1.0]);

        // Normal (3 x f32 LE)
        buf.extend_from_slice(&(normal[0] as f32).to_le_bytes());
        buf.extend_from_slice(&(normal[1] as f32).to_le_bytes());
        buf.extend_from_slice(&(normal[2] as f32).to_le_bytes());

        // Vertices (3 x 3 f32 LE)
        for &idx in tri {
            let v = &mesh.vertices[idx as usize];
            buf.extend_from_slice(&(v.x as f32).to_le_bytes());
            buf.extend_from_slice(&(v.y as f32).to_le_bytes());
            buf.extend_from_slice(&(v.z as f32).to_le_bytes());
        }

        // Attribute byte count (u16 LE) — always 0
        buf.extend_from_slice(&0u16.to_le_bytes());
    }

    buf
}

/// Write STL to a file.
pub fn write_stl_file(mesh: &TriangleMesh, path: &str, binary: bool) -> io::Result<()> {
    let mut file = std::fs::File::create(path)?;
    if binary {
        let data = export_stl_binary(mesh, "3Draper");
        file.write_all(&data)?;
    } else {
        let data = export_stl_ascii(mesh, "3Draper");
        file.write_all(data.as_bytes())?;
    }
    Ok(())
}
