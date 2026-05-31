// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! STL file import and export.

use crate::mesh::TriangleMesh;
use draper_geometry::Point3d;
use std::io::{self, Read, Write};

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
#[cfg(not(target_arch = "wasm32"))]
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

/// Import an STL file from raw bytes (works on all platforms including wasm).
///
/// This is the primary API for web/WASM builds where filesystem access
/// is not available. Pass the raw bytes of the STL file.
pub fn import_stl_from_bytes(data: &[u8]) -> io::Result<TriangleMesh> {
    if data.len() < 84 {
        return Err(io::Error::new(io::ErrorKind::InvalidData, "STL file too small"));
    }

    // Check if it's ASCII or binary
    let header = String::from_utf8_lossy(&data[..80]);
    if header.trim().starts_with("solid") && !is_likely_binary(data) {
        return import_stl_ascii_from_data(data);
    }

    // Binary STL
    let num_triangles = u32::from_le_bytes(
        data[80..84].try_into().map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "Invalid STL header"))?
    );

    let expected_size = 84 + num_triangles as usize * 50;
    if data.len() < expected_size {
        return Err(io::Error::new(io::ErrorKind::InvalidData,
            format!("STL file truncated: expected {} bytes, got {}", expected_size, data.len())));
    }

    let mut mesh = TriangleMesh::new();
    let mut offset = 84;

    for _ in 0..num_triangles {
        // Skip normal (3 x f32)
        offset += 12;

        // Read 3 vertices
        let mut tri_indices = [0u32; 3];
        for v in 0..3 {
            let vx_bytes: [u8; 4] = data[offset..offset+4].try_into()
                .map_err(|_| io::Error::new(io::ErrorKind::InvalidData,
                    format!("Truncated vertex data at offset {}", offset)))?;
            let vy_bytes: [u8; 4] = data[offset+4..offset+8].try_into()
                .map_err(|_| io::Error::new(io::ErrorKind::InvalidData,
                    format!("Truncated vertex data at offset {}", offset+4)))?;
            let vz_bytes: [u8; 4] = data[offset+8..offset+12].try_into()
                .map_err(|_| io::Error::new(io::ErrorKind::InvalidData,
                    format!("Truncated vertex data at offset {}", offset+8)))?;
            let vx = f32::from_le_bytes(vx_bytes) as f64;
            let vy = f32::from_le_bytes(vy_bytes) as f64;
            let vz = f32::from_le_bytes(vz_bytes) as f64;
            tri_indices[v] = mesh.add_vertex(Point3d::new(vx, vy, vz));
            offset += 12;
        }

        mesh.add_triangle(tri_indices[0], tri_indices[1], tri_indices[2]);

        // Attribute byte count
        offset += 2;
    }

    mesh.compute_face_normals();
    Ok(mesh)
}

/// Import a binary STL file from a filesystem path (native only).
#[cfg(not(target_arch = "wasm32"))]
pub fn import_stl_binary(path: &str) -> io::Result<TriangleMesh> {
    let mut file = std::fs::File::open(path)?;
    let mut data = Vec::new();
    file.read_to_end(&mut data)?;
    import_stl_from_bytes(&data)
}

/// Check if data is likely binary STL (even if header starts with 'solid').
fn is_likely_binary(data: &[u8]) -> bool {
    if data.len() < 84 {
        return false;
    }
    let num_triangles: u32 = match data[80..84].try_into() {
        Ok(bytes) => u32::from_le_bytes(bytes),
        Err(_) => return false,
    };
    let expected_size = 84 + num_triangles as usize * 50;
    // If the size matches binary format exactly, it's binary
    data.len() == expected_size || (num_triangles > 0 && num_triangles < 10_000_000)
}

/// Import an ASCII STL file from raw data.
fn import_stl_ascii_from_data(data: &[u8]) -> io::Result<TriangleMesh> {
    let text = String::from_utf8_lossy(data);
    let mut mesh = TriangleMesh::new();

    let mut current_vertices: Vec<u32> = Vec::new();

    for line in text.lines() {
        let line = line.trim();
        if line.starts_with("vertex") {
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() >= 4 {
                if let (Ok(x), Ok(y), Ok(z)) = (
                    parts[1].parse::<f64>(),
                    parts[2].parse::<f64>(),
                    parts[3].parse::<f64>(),
                ) {
                    let idx = mesh.add_vertex(Point3d::new(x, y, z));
                    current_vertices.push(idx);
                }
            }
        } else if line.starts_with("endfacet") {
            if current_vertices.len() >= 3 {
                mesh.add_triangle(current_vertices[0], current_vertices[1], current_vertices[2]);
            }
            current_vertices.clear();
        }
    }

    mesh.compute_face_normals();
    Ok(mesh)
}
