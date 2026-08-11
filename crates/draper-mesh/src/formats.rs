// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! File format importers/exporters — Phase 5.

use crate::mesh::TriangleMesh;
use draper_geometry::Point3d;
use std::io::{self, Write, Read, BufRead, BufReader};
use std::fs::File;

/// Import a Wavefront OBJ file.
pub fn import_obj(path: &str) -> io::Result<TriangleMesh> {
    let file = File::open(path)?;
    let reader = BufReader::new(file);
    import_obj_from_reader(reader)
}

pub fn import_obj_from_reader<R: BufRead>(reader: R) -> io::Result<TriangleMesh> {
    let mut vertices: Vec<Point3d> = Vec::new();
    let mut normals: Vec<[f64; 3]> = Vec::new();
    let mut triangles: Vec<[u32; 3]> = Vec::new();
    let mut has_normals = false;

    for line in reader.lines() {
        let line = line?;
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with('#') { continue; }
        let mut parts = trimmed.split_whitespace();
        let tag = parts.next().unwrap_or("");
        match tag {
            "v" => {
                let coords: Vec<f64> = parts.take(3).map(|s| s.parse().unwrap_or(0.0)).collect();
                if coords.len() >= 3 {
                    vertices.push(Point3d::new(coords[0], coords[1], coords[2]));
                }
            }
            "vn" => {
                let n: Vec<f64> = parts.take(3).map(|s| s.parse().unwrap_or(0.0)).collect();
                if n.len() >= 3 {
                    normals.push([n[0], n[1], n[2]]);
                    has_normals = true;
                }
            }
            "f" => {
                let indices: Vec<i64> = parts.map(|tok| {
                    tok.split('/').next().and_then(|s| s.parse().ok()).unwrap_or(0)
                }).collect();
                let resolved: Vec<u32> = indices.iter().map(|&i| {
                    if i > 0 { (i - 1) as u32 }
                    else if i < 0 {
                        let abs = (-i) as usize;
                        if abs <= vertices.len() { (vertices.len() - abs) as u32 } else { 0 }
                    } else { 0 }
                }).collect();
                if resolved.len() >= 3 {
                    for i in 1..resolved.len() - 1 {
                        triangles.push([resolved[0], resolved[i], resolved[i + 1]]);
                    }
                }
            }
            _ => {}
        }
    }

    if vertices.is_empty() {
        return Err(io::Error::new(io::ErrorKind::InvalidData, "OBJ: no vertices"));
    }

    Ok(TriangleMesh {
        vertices,
        triangles,
        normals: if has_normals && !normals.is_empty() { Some(normals) } else { None },
        face_normals: None,
        triangle_colors: None,
        triangle_face_ids: None,
    })
}

/// Import a Stanford PLY file (ASCII or binary_little_endian).
pub fn import_ply(path: &str) -> io::Result<TriangleMesh> {
    let mut file = File::open(path)?;
    let mut data = Vec::new();
    file.read_to_end(&mut data)?;
    import_ply_from_bytes(&data)
}

pub fn import_ply_from_bytes(data: &[u8]) -> io::Result<TriangleMesh> {
    let header_end = data.windows(11).position(|w| w == b"end_header\n")
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "PLY: end_header not found"))?;
    let header_str = std::str::from_utf8(&data[..header_end])
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "PLY: header not UTF-8"))?;

    let mut format = "ascii";
    let mut vertex_count: usize = 0;
    let mut face_count: usize = 0;
    let mut has_normals = false;
    let mut current_element = "";

    for line in header_str.lines() {
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.is_empty() { continue; }
        match parts[0] {
            "format" if parts.len() >= 2 => format = parts[1],
            "element" if parts.len() >= 3 => {
                current_element = parts[1];
                let count: usize = parts[2].parse().unwrap_or(0);
                match current_element {
                    "vertex" => vertex_count = count,
                    "face" => face_count = count,
                    _ => {}
                }
            }
            "property" if current_element == "vertex" => {
                if parts.contains(&"nx") || parts.contains(&"ny") || parts.contains(&"nz") {
                    has_normals = true;
                }
            }
            _ => {}
        }
    }

    let body = &data[header_end + 11..];
    match format {
        "ascii" => parse_ply_ascii(body, vertex_count, face_count, has_normals),
        "binary_little_endian" => parse_ply_binary_le(body, vertex_count, face_count, has_normals),
        _ => Err(io::Error::new(io::ErrorKind::InvalidData,
            format!("PLY: unsupported format '{}'", format))),
    }
}

fn parse_ply_ascii(body: &[u8], vc: usize, fc: usize, has_normals: bool) -> io::Result<TriangleMesh> {
    let text = std::str::from_utf8(body)
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "PLY ASCII: not UTF-8"))?;
    let mut vertices = Vec::with_capacity(vc);
    let mut normals = if has_normals { Some(Vec::with_capacity(vc)) } else { None };
    let mut triangles = Vec::with_capacity(fc);
    let mut vr = 0; let mut fr = 0;

    for line in text.lines() {
        let t = line.trim();
        if t.is_empty() || t.starts_with("comment") { continue; }
        let parts: Vec<&str> = t.split_whitespace().collect();
        if vr < vc {
            if parts.len() >= 3 {
                let x: f64 = parts[0].parse().unwrap_or(0.0);
                let y: f64 = parts[1].parse().unwrap_or(0.0);
                let z: f64 = parts[2].parse().unwrap_or(0.0);
                vertices.push(Point3d::new(x, y, z));
                if let Some(ref mut ns) = normals {
                    if parts.len() >= 6 {
                        ns.push([parts[3].parse().unwrap_or(0.0),
                                 parts[4].parse().unwrap_or(0.0),
                                 parts[5].parse().unwrap_or(0.0)]);
                    }
                }
                vr += 1;
            }
        } else if fr < fc {
            if parts.len() >= 4 {
                let count: u32 = parts[0].parse().unwrap_or(0);
                let idx: Vec<u32> = parts[1..].iter().map(|s| s.parse().unwrap_or(0)).collect();
                if count as usize == idx.len() && count >= 3 {
                    for i in 1..count - 1 {
                        triangles.push([idx[0], idx[i as usize], idx[(i + 1) as usize]]);
                    }
                }
                fr += 1;
            }
        }
    }

    Ok(TriangleMesh {
        vertices, triangles, normals,
        face_normals: None, triangle_colors: None, triangle_face_ids: None,
    })
}

fn parse_ply_binary_le(body: &[u8], vc: usize, fc: usize, has_normals: bool) -> io::Result<TriangleMesh> {
    use std::convert::TryInto;
    let mut offset = 0;
    let mut vertices = Vec::with_capacity(vc);
    let mut normals = if has_normals { Some(Vec::with_capacity(vc)) } else { None };
    let v_size = if has_normals { 24 } else { 12 };

    for _ in 0..vc {
        if offset + v_size > body.len() {
            return Err(io::Error::new(io::ErrorKind::UnexpectedEof, "PLY binary: vertex truncated"));
        }
        let c = &body[offset..offset + v_size];
        let x = f32::from_le_bytes(c[0..4].try_into().unwrap()) as f64;
        let y = f32::from_le_bytes(c[4..8].try_into().unwrap()) as f64;
        let z = f32::from_le_bytes(c[8..12].try_into().unwrap()) as f64;
        vertices.push(Point3d::new(x, y, z));
        if let Some(ref mut ns) = normals {
            ns.push([
                f32::from_le_bytes(c[12..16].try_into().unwrap()) as f64,
                f32::from_le_bytes(c[16..20].try_into().unwrap()) as f64,
                f32::from_le_bytes(c[20..24].try_into().unwrap()) as f64,
            ]);
        }
        offset += v_size;
    }

    let mut triangles = Vec::with_capacity(fc);
    for _ in 0..fc {
        if offset + 1 > body.len() {
            return Err(io::Error::new(io::ErrorKind::UnexpectedEof, "PLY binary: face count truncated"));
        }
        let count = body[offset] as usize;
        offset += 1;
        if offset + count * 4 > body.len() {
            return Err(io::Error::new(io::ErrorKind::UnexpectedEof, "PLY binary: face idx truncated"));
        }
        let mut idx: Vec<u32> = Vec::with_capacity(count);
        for _ in 0..count {
            idx.push(u32::from_le_bytes(body[offset..offset + 4].try_into().unwrap()));
            offset += 4;
        }
        if count >= 3 {
            for i in 1..count - 1 {
                triangles.push([idx[0], idx[i], idx[i + 1]]);
            }
        }
    }

    Ok(TriangleMesh {
        vertices, triangles, normals,
        face_normals: None, triangle_colors: None, triangle_face_ids: None,
    })
}

/// Export to Stanford PLY (ASCII).
pub fn export_ply(mesh: &TriangleMesh, path: &str) -> io::Result<()> {
    let content = build_ply_ascii(mesh);
    File::create(path)?.write_all(content.as_bytes())?;
    Ok(())
}

pub fn build_ply_ascii(mesh: &TriangleMesh) -> String {
    let mut out = String::new();
    out.push_str("ply\nformat ascii 1.0\ncomment Exported by 3Draper\n");
    out.push_str(&format!("element vertex {}\n", mesh.vertices.len()));
    out.push_str("property float x\nproperty float y\nproperty float z\n");
    let has_normals = mesh.normals.as_ref().map_or(false, |n| !n.is_empty());
    if has_normals {
        out.push_str("property float nx\nproperty float ny\nproperty float nz\n");
    }
    out.push_str(&format!("element face {}\n", mesh.triangles.len()));
    out.push_str("property list uchar int vertex_indices\nend_header\n");
    for (i, v) in mesh.vertices.iter().enumerate() {
        if has_normals {
            let n = &mesh.normals.as_ref().unwrap()[i];
            out.push_str(&format!("{} {} {} {} {} {}\n", v.x, v.y, v.z, n[0], n[1], n[2]));
        } else {
            out.push_str(&format!("{} {} {}\n", v.x, v.y, v.z));
        }
    }
    for tri in &mesh.triangles {
        out.push_str(&format!("3 {} {} {}\n", tri[0], tri[1], tri[2]));
    }
    out
}

/// Export to DXF (2D flat pattern, projected to XY).
pub fn export_dxf(mesh: &TriangleMesh, path: &str) -> io::Result<()> {
    let content = build_dxf(mesh);
    File::create(path)?.write_all(content.as_bytes())?;
    Ok(())
}

pub fn build_dxf(mesh: &TriangleMesh) -> String {
    let mut out = String::new();
    out.push_str("0\nSECTION\n2\nHEADER\n9\n$ACADVER\n1\nAC1009\n0\nENDSEC\n");
    out.push_str("0\nSECTION\n2\nENTITIES\n");
    for tri in &mesh.triangles {
        let v0 = &mesh.vertices[tri[0] as usize];
        let v1 = &mesh.vertices[tri[1] as usize];
        let v2 = &mesh.vertices[tri[2] as usize];
        out.push_str("0\nPOLYLINE\n8\n0\n66\n1\n70\n1\n");
        out.push_str(&format!("0\nVERTEX\n8\n0\n10\n{}\n20\n{}\n30\n0.0\n", v0.x, v0.y));
        out.push_str(&format!("0\nVERTEX\n8\n0\n10\n{}\n20\n{}\n30\n0.0\n", v1.x, v1.y));
        out.push_str(&format!("0\nVERTEX\n8\n0\n10\n{}\n20\n{}\n30\n0.0\n", v2.x, v2.y));
        out.push_str("0\nSEQEND\n");
    }
    out.push_str("0\nENDSEC\n0\nEOF\n");
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_test_mesh() -> TriangleMesh {
        TriangleMesh {
            vertices: vec![
                Point3d::new(0.0, 0.0, 0.0),
                Point3d::new(1.0, 0.0, 0.0),
                Point3d::new(0.0, 1.0, 0.0),
                Point3d::new(1.0, 1.0, 0.0),
            ],
            triangles: vec![[0, 1, 2], [1, 3, 2]],
            normals: None, face_normals: None,
            triangle_colors: None, triangle_face_ids: None,
        }
    }

    fn build_obj_for_test(mesh: &TriangleMesh) -> String {
        let mut out = String::new();
        for v in &mesh.vertices {
            out.push_str(&format!("v {} {} {}\n", v.x, v.y, v.z));
        }
        for tri in &mesh.triangles {
            out.push_str(&format!("f {} {} {}\n", tri[0] + 1, tri[1] + 1, tri[2] + 1));
        }
        out
    }

    #[test]
    fn test_obj_roundtrip() {
        let mesh = make_test_mesh();
        let obj = build_obj_for_test(&mesh);
        let cursor = io::Cursor::new(obj.into_bytes());
        let imported = import_obj_from_reader(BufReader::new(cursor)).unwrap();
        assert_eq!(imported.vertices.len(), 4);
        assert_eq!(imported.triangles.len(), 2);
        for (i, v) in mesh.vertices.iter().enumerate() {
            assert!((imported.vertices[i].x - v.x).abs() < 1e-9);
        }
    }

    #[test]
    fn test_obj_quad_triangulation() {
        let obj = "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf 1 2 3 4\n";
        let cursor = io::Cursor::new(obj.as_bytes().to_vec());
        let mesh = import_obj_from_reader(BufReader::new(cursor)).unwrap();
        assert_eq!(mesh.triangles.len(), 2);
    }

    #[test]
    fn test_obj_empty_errors() {
        let cursor = io::Cursor::new(b"# comment\n".to_vec());
        assert!(import_obj_from_reader(BufReader::new(cursor)).is_err());
    }

    #[test]
    fn test_ply_ascii_roundtrip() {
        let mesh = make_test_mesh();
        let ply = build_ply_ascii(&mesh);
        let imported = import_ply_from_bytes(ply.as_bytes()).unwrap();
        assert_eq!(imported.vertices.len(), 4);
        assert_eq!(imported.triangles.len(), 2);
    }

    #[test]
    fn test_ply_with_normals() {
        let mut mesh = make_test_mesh();
        mesh.normals = Some(vec![[0.0, 0.0, 1.0]; 4]);
        let ply = build_ply_ascii(&mesh);
        let imported = import_ply_from_bytes(ply.as_bytes()).unwrap();
        assert!(imported.normals.is_some());
        assert_eq!(imported.normals.as_ref().unwrap().len(), 4);
    }

    #[test]
    fn test_dxf_basic() {
        let mesh = make_test_mesh();
        let dxf = build_dxf(&mesh);
        assert!(dxf.contains("POLYLINE"));
        assert!(dxf.contains("EOF"));
        assert_eq!(dxf.matches("POLYLINE").count(), 2);
    }
}
