// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Export triangle meshes to modern file formats.
//!
//! Supports: glTF 2.0 (GLB), 3MF, OBJ, STL (ASCII & binary), and USD (stub).

use crate::mesh::TriangleMesh;
use std::io::Write;
use thiserror::Error;
use zip::write::SimpleFileOptions;

// ============================================================
// Error type
// ============================================================

/// Errors that can occur during mesh export.
///
/// This is a local error type within `draper-mesh` to avoid a circular
/// dependency on `draper-core::KernelError`. It can be mapped to
/// `KernelError` in the integration layer.
#[derive(Error, Debug)]
pub enum ExportError {
    /// The requested export format is not yet implemented.
    #[error("Unsupported format: {0}")]
    UnsupportedFormat(String),

    /// The mesh is not valid for export (e.g., zero vertices).
    #[error("Invalid mesh: {0}")]
    InvalidMesh(String),

    /// An I/O error occurred during export.
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// A ZIP archive error (3MF packaging).
    #[error("ZIP error: {0}")]
    Zip(#[from] zip::result::ZipError),
}

impl From<ExportError> for std::io::Error {
    fn from(e: ExportError) -> Self {
        match e {
            ExportError::Io(io) => io,
            other => std::io::Error::new(std::io::ErrorKind::Other, other.to_string()),
        }
    }
}

// ============================================================
// 4.4.1 glTF 2.0 Export (GLB)
// ============================================================

/// Export a triangle mesh as a glTF 2.0 Binary (GLB) file.
///
/// The output is a single-mesh, single-primitive GLB with:
/// - POSITION attribute (vec3, float32)
/// - NORMAL attribute (vec3, float32), using face normals expanded per-vertex
/// - COLOR_0 attribute (vec4, float32) if `triangle_colors` is present
/// - Indices (unsigned short or unsigned int depending on vertex count)
///
/// See: <https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html>
pub fn export_gltf(mesh: &TriangleMesh, path: &str) -> Result<(), ExportError> {
    if mesh.vertices.is_empty() || mesh.triangles.is_empty() {
        return Err(ExportError::InvalidMesh("Mesh has no vertices or triangles".into()));
    }

    let glb_data = build_glb(mesh)?;
    let mut file = std::fs::File::create(path)?;
    file.write_all(&glb_data)?;
    Ok(())
}

/// Build the GLB binary in memory and return the bytes.
pub fn build_glb(mesh: &TriangleMesh) -> Result<Vec<u8>, ExportError> {
    if mesh.vertices.is_empty() || mesh.triangles.is_empty() {
        return Err(ExportError::InvalidMesh("Mesh has no vertices or triangles".into()));
    }

    let has_colors = mesh.triangle_colors.is_some();

    // Expand face normals to per-vertex normals.
    // glTF expects per-vertex normals; if we only have face normals,
    // we expand each triangle into 3 separate vertices with its face normal.
    let vertex_normals: Vec<[f32; 3]>;
    let expanded_positions: Vec<[f32; 3]>;
    let expanded_colors: Vec<[f32; 4]>;
    let indices: Vec<u32>;

    if mesh.normals.is_some() {
        // Per-vertex normals available — we can share vertices.
        let normals = mesh.normals.as_ref().unwrap();
        expanded_positions = mesh
            .vertices
            .iter()
            .map(|v| [v.x as f32, v.y as f32, v.z as f32])
            .collect();
        vertex_normals = normals
            .iter()
            .map(|n| [n[0] as f32, n[1] as f32, n[2] as f32])
            .collect();
        indices = mesh
            .triangles
            .iter()
            .flat_map(|t| [t[0], t[1], t[2]])
            .collect();

        // Colors per-vertex (expand from per-triangle)
        if has_colors {
            let colors = mesh.triangle_colors.as_ref().unwrap();
            // We need to map per-triangle colors to per-vertex.
            // For simplicity, average the colors of all triangles sharing a vertex.
            let mut vert_colors = vec![[0.0f32; 4]; mesh.vertices.len()];
            let mut vert_counts = vec![0u32; mesh.vertices.len()];
            for (ti, tri) in mesh.triangles.iter().enumerate() {
                for &vi in tri {
                    let vi = vi as usize;
                    vert_colors[vi][0] += colors[ti][0];
                    vert_colors[vi][1] += colors[ti][1];
                    vert_colors[vi][2] += colors[ti][2];
                    vert_colors[vi][3] += colors[ti][3];
                    vert_counts[vi] += 1;
                }
            }
            for (i, c) in vert_colors.iter_mut().enumerate() {
                if vert_counts[i] > 0 {
                    let inv = 1.0f32 / vert_counts[i] as f32;
                    c[0] *= inv;
                    c[1] *= inv;
                    c[2] *= inv;
                    c[3] *= inv;
                }
            }
            expanded_colors = vert_colors;
        } else {
            expanded_colors = Vec::new();
        }
    } else {
        // No per-vertex normals — expand each triangle into 3 separate vertices.
        let num_expanded = mesh.triangles.len() * 3;
        let mut pos = Vec::with_capacity(num_expanded);
        let mut col = if has_colors {
            Vec::with_capacity(num_expanded)
        } else {
            Vec::new()
        };
        let mut norms = Vec::with_capacity(num_expanded);

        for (ti, tri) in mesh.triangles.iter().enumerate() {
            let n = if let Some(ref fns) = mesh.face_normals {
                [fns[ti][0] as f32, fns[ti][1] as f32, fns[ti][2] as f32]
            } else {
                // Compute face normal on the fly
                let v0 = &mesh.vertices[tri[0] as usize];
                let v1 = &mesh.vertices[tri[1] as usize];
                let v2 = &mesh.vertices[tri[2] as usize];
                let e1 = (v1.x - v0.x, v1.y - v0.y, v1.z - v0.z);
                let e2 = (v2.x - v0.x, v2.y - v0.y, v2.z - v0.z);
                let nx = e1.1 * e2.2 - e1.2 * e2.1;
                let ny = e1.2 * e2.0 - e1.0 * e2.2;
                let nz = e1.0 * e2.1 - e1.1 * e2.0;
                let len = (nx * nx + ny * ny + nz * nz).sqrt();
                if len > 1e-15 {
                    [nx as f32 / len as f32, ny as f32 / len as f32, nz as f32 / len as f32]
                } else {
                    [0.0f32, 0.0f32, 1.0f32]
                }
            };

            for &vi in tri {
                let v = &mesh.vertices[vi as usize];
                pos.push([v.x as f32, v.y as f32, v.z as f32]);
                norms.push(n);
                if has_colors {
                    let colors = mesh.triangle_colors.as_ref().unwrap();
                    col.push(colors[ti]);
                }
            }
        }

        expanded_positions = pos;
        vertex_normals = norms;
        expanded_colors = col;
        indices = (0..mesh.triangles.len() as u32 * 3).collect();
    }

    // ---- Build binary buffer ----
    let mut bin_data = Vec::new();

    // Positions
    let positions_byte_len = expanded_positions.len() * 12; // 3 x f32
    let positions_offset = 0u32;

    for p in &expanded_positions {
        bin_data.extend_from_slice(&p[0].to_le_bytes());
        bin_data.extend_from_slice(&p[1].to_le_bytes());
        bin_data.extend_from_slice(&p[2].to_le_bytes());
    }

    // Normals
    let normals_byte_len = vertex_normals.len() * 12;
    let normals_offset = positions_offset + positions_byte_len as u32;

    for n in &vertex_normals {
        bin_data.extend_from_slice(&n[0].to_le_bytes());
        bin_data.extend_from_slice(&n[1].to_le_bytes());
        bin_data.extend_from_slice(&n[2].to_le_bytes());
    }

    // Colors (optional)
    let colors_byte_len = if has_colors { expanded_colors.len() * 16 } else { 0 }; // 4 x f32
    let colors_offset = normals_offset + normals_byte_len as u32;

    if has_colors {
        for c in &expanded_colors {
            bin_data.extend_from_slice(&c[0].to_le_bytes());
            bin_data.extend_from_slice(&c[1].to_le_bytes());
            bin_data.extend_from_slice(&c[2].to_le_bytes());
            bin_data.extend_from_slice(&c[3].to_le_bytes());
        }
    }

    // Indices
    let use_u16 = expanded_positions.len() <= 65535;
    let indices_byte_len = if use_u16 {
        indices.len() * 2
    } else {
        indices.len() * 4
    };
    let indices_offset = colors_offset + colors_byte_len as u32;

    if use_u16 {
        for &idx in &indices {
            bin_data.extend_from_slice(&(idx as u16).to_le_bytes());
        }
    } else {
        for &idx in &indices {
            bin_data.extend_from_slice(&idx.to_le_bytes());
        }
    }

    // Pad bin_data to 4-byte alignment
    while bin_data.len() % 4 != 0 {
        bin_data.push(0);
    }

    let total_bin_length = bin_data.len() as u32;

    // ---- Build glTF JSON ----
    let vertex_count = expanded_positions.len() as u32;
    let index_count = indices.len() as u32;
    let index_component_type = if use_u16 { 5123u32 } else { 5125 }; // UNSIGNED_SHORT or UNSIGNED_INT
    let mut attributes_json = format!(
        r#"      "POSITION": {{
        "bufferView": 0,
        "byteOffset": 0,
        "componentType": 5126,
        "count": {vertex_count},
        "type": "VEC3"
      }},
      "NORMAL": {{
        "bufferView": 1,
        "byteOffset": 0,
        "componentType": 5126,
        "count": {vertex_count},
        "type": "VEC3"
      }}"#
    );

    if has_colors {
        attributes_json.push_str(&format!(
            r#",
      "COLOR_0": {{
        "bufferView": 2,
        "byteOffset": 0,
        "componentType": 5126,
        "count": {vertex_count},
        "type": "VEC4"
      }}"#
        ));
    }

    let indices_buffer_view_index = if has_colors { 3 } else { 2 };

    let mut buffer_views_json = format!(
        r#"    {{
      "buffer": 0,
      "byteOffset": {positions_offset},
      "byteLength": {positions_byte_len},
      "target": 34962
    }},
    {{
      "buffer": 0,
      "byteOffset": {normals_offset},
      "byteLength": {normals_byte_len},
      "target": 34962
    }}"#
    );

    if has_colors {
        buffer_views_json.push_str(&format!(
            r#",
    {{
      "buffer": 0,
      "byteOffset": {colors_offset},
      "byteLength": {colors_byte_len},
      "target": 34962
    }}"#
        ));
    }

    buffer_views_json.push_str(&format!(
        r#",
    {{
      "buffer": 0,
      "byteOffset": {indices_offset},
      "byteLength": {indices_byte_len},
      "target": 34963
    }}"#
    ));

    let accessors_json = format!(
        r#"    {{
      "bufferView": 0,
      "byteOffset": 0,
      "componentType": 5126,
      "count": {vertex_count},
      "type": "VEC3",
      "max": [{xmax}, {ymax}, {zmax}],
      "min": [{xmin}, {ymin}, {zmin}]
    }},
    {{
      "bufferView": 1,
      "byteOffset": 0,
      "componentType": 5126,
      "count": {vertex_count},
      "type": "VEC3"
    }},
    {{
      "bufferView": {indices_buffer_view_index},
      "byteOffset": 0,
      "componentType": {index_component_type},
      "count": {index_count},
      "type": "SCALAR"
    }}"#,
        xmax = expanded_positions
            .iter()
            .map(|p| p[0])
            .fold(f32::NEG_INFINITY, f32::max),
        ymax = expanded_positions
            .iter()
            .map(|p| p[1])
            .fold(f32::NEG_INFINITY, f32::max),
        zmax = expanded_positions
            .iter()
            .map(|p| p[2])
            .fold(f32::NEG_INFINITY, f32::max),
        xmin = expanded_positions
            .iter()
            .map(|p| p[0])
            .fold(f32::INFINITY, f32::min),
        ymin = expanded_positions
            .iter()
            .map(|p| p[1])
            .fold(f32::INFINITY, f32::min),
        zmin = expanded_positions
            .iter()
            .map(|p| p[2])
            .fold(f32::INFINITY, f32::min),
    );

    let json_str = format!(
        r#"{{
  "asset": {{
    "version": "2.0",
    "generator": "3Draper"
  }},
  "scene": 0,
  "scenes": [
    {{
      "nodes": [0]
    }}
  ],
  "nodes": [
    {{
      "mesh": 0
    }}
  ],
  "meshes": [
    {{
      "primitives": [
        {{
          "attributes": {{
{attributes_json}
          }},
          "indices": 2,
          "mode": 4
        }}
      ]
    }}
  ],
  "accessors": [
{accessors_json}
  ],
  "bufferViews": [
{buffer_views_json}
  ],
  "buffers": [
    {{
      "byteLength": {total_bin_length}
    }}
  ]
}}"#
    );

    // ---- Assemble GLB ----
    // Pad JSON to 4-byte alignment (with spaces, not nulls, as per spec)
    let mut json_bytes = json_str.into_bytes();
    while json_bytes.len() % 4 != 0 {
        json_bytes.push(b' ');
    }

    let json_length = json_bytes.len() as u32;
    let bin_length = bin_data.len() as u32;
    let total_length = 12u32 + 8 + json_length + 8 + bin_length; // header + json chunk header + json + bin chunk header + bin

    let mut glb = Vec::with_capacity(total_length as usize);

    // GLB Header (12 bytes)
    glb.extend_from_slice(b"glTF"); // magic
    glb.extend_from_slice(&2u32.to_le_bytes()); // version
    glb.extend_from_slice(&total_length.to_le_bytes()); // total length

    // JSON chunk
    glb.extend_from_slice(&json_length.to_le_bytes()); // chunk length
    glb.extend_from_slice(b"JSON"); // chunk type
    glb.extend_from_slice(&json_bytes);

    // Binary chunk
    glb.extend_from_slice(&bin_length.to_le_bytes()); // chunk length
    glb.extend_from_slice(b"BIN\0"); // chunk type
    glb.extend_from_slice(&bin_data);

    Ok(glb)
}

// ============================================================
// 4.4.2 USD/USDZ Export (Stub)
// ============================================================

/// Export a triangle mesh to USD format.
///
/// **Not yet implemented** — requires the USD SDK which is a heavy dependency.
/// Returns `ExportError::UnsupportedFormat`.
pub fn export_usd(_mesh: &TriangleMesh, _path: &str) -> Result<(), ExportError> {
    Err(ExportError::UnsupportedFormat(
        "USD export not yet implemented".into(),
    ))
}

// ============================================================
// 4.4.3 3MF Export
// ============================================================

/// Export a triangle mesh to 3MF (3D Manufacturing Format).
///
/// 3MF is an XML-based format wrapped in a ZIP archive. This function
/// creates a minimal valid 3MF file with the required entries:
/// - `[Content_Types].xml`
/// - `_rels/.rels`
/// - `3D/3dmodel.model`
pub fn export_3mf(mesh: &TriangleMesh, path: &str) -> Result<(), ExportError> {
    if mesh.vertices.is_empty() || mesh.triangles.is_empty() {
        return Err(ExportError::InvalidMesh("Mesh has no vertices or triangles".into()));
    }

    let file = std::fs::File::create(path)?;
    let mut zip = zip::ZipWriter::new(file);
    let options = SimpleFileOptions::default()
        .compression_method(zip::CompressionMethod::Deflated);

    // [Content_Types].xml
    let content_types = r#"<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
</Types>"#;
    zip.start_file("[Content_Types].xml", options)?;
    zip.write_all(content_types.as_bytes())?;

    // _rels/.rels
    let rels = r#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
</Relationships>"#;
    zip.start_file("_rels/.rels", options)?;
    zip.write_all(rels.as_bytes())?;

    // 3D/3dmodel.model
    let model_xml = build_3mf_model(mesh);
    zip.start_file("3D/3dmodel.model", options)?;
    zip.write_all(model_xml.as_bytes())?;

    zip.finish()?;
    Ok(())
}

/// Build the 3MF model XML content.
fn build_3mf_model(mesh: &TriangleMesh) -> String {
    let mut xml = String::new();
    xml.push_str(r#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
  <resources>
    <object id="1" type="model">
      <mesh>
"#);

    // Vertices
    xml.push_str("        <vertices>\n");
    for v in &mesh.vertices {
        xml.push_str(&format!(
            "          <vertex x=\"{}\" y=\"{}\" z=\"{}\" />\n",
            v.x, v.y, v.z
        ));
    }
    xml.push_str("        </vertices>\n");

    // Triangles
    xml.push_str("        <triangles>\n");
    for tri in &mesh.triangles {
        xml.push_str(&format!(
            "          <triangle v1=\"{}\" v2=\"{}\" v3=\"{}\" />\n",
            tri[0], tri[1], tri[2]
        ));
    }
    xml.push_str("        </triangles>\n");

    xml.push_str(r#"      </mesh>
    </object>
  </resources>
  <build>
    <item objectid="1" />
  </build>
</model>"#);

    xml
}

// ============================================================
// 4.4.4 OBJ Export
// ============================================================

/// Export a triangle mesh to Wavefront OBJ format.
///
/// Writes vertex positions (`v`), optional vertex normals (`vn`),
/// and face definitions (`f`) with normal indices.
pub fn export_obj(mesh: &TriangleMesh, path: &str) -> Result<(), ExportError> {
    if mesh.vertices.is_empty() || mesh.triangles.is_empty() {
        return Err(ExportError::InvalidMesh("Mesh has no vertices or triangles".into()));
    }

    let content = build_obj(mesh);
    let mut file = std::fs::File::create(path)?;
    file.write_all(content.as_bytes())?;
    Ok(())
}

/// Build OBJ content as a string.
pub fn build_obj(mesh: &TriangleMesh) -> String {
    let mut out = String::new();

    // Header comment
    out.push_str("# 3Draper OBJ Export\n");

    // Vertices (1-indexed in OBJ)
    for v in &mesh.vertices {
        out.push_str(&format!("v {} {} {}\n", v.x, v.y, v.z));
    }

    // Vertex normals (if available)
    let has_normals = mesh.normals.is_some();
    if has_normals {
        let normals = mesh.normals.as_ref().unwrap();
        for n in normals {
            out.push_str(&format!("vn {} {} {}\n", n[0], n[1], n[2]));
        }
    }

    // Faces
    if has_normals {
        // f v//vn v//vn v//vn  (OBJ is 1-indexed)
        for tri in &mesh.triangles {
            out.push_str(&format!(
                "f {}//{} {}//{} {}//{}\n",
                tri[0] + 1,
                tri[0] + 1,
                tri[1] + 1,
                tri[1] + 1,
                tri[2] + 1,
                tri[2] + 1,
            ));
        }
    } else {
        // No normals — just vertex indices
        for tri in &mesh.triangles {
            out.push_str(&format!(
                "f {} {} {}\n",
                tri[0] + 1,
                tri[1] + 1,
                tri[2] + 1,
            ));
        }
    }

    out
}

// ============================================================
// 4.4.5 STL Export (improved wrapper)
// ============================================================

/// Export a triangle mesh to STL format (binary or ASCII).
///
/// This is a unified wrapper around the existing STL export functions
/// that returns a `Result` and provides proper error handling.
pub fn export_stl(mesh: &TriangleMesh, path: &str, binary: bool) -> Result<(), ExportError> {
    if mesh.vertices.is_empty() || mesh.triangles.is_empty() {
        return Err(ExportError::InvalidMesh("Mesh has no vertices or triangles".into()));
    }

    let mut file = std::fs::File::create(path)?;
    if binary {
        let data = crate::stl::export_stl_binary(mesh, "3Draper");
        file.write_all(&data)?;
    } else {
        let data = crate::stl::export_stl_ascii(mesh, "3Draper");
        file.write_all(data.as_bytes())?;
    }
    Ok(())
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::TriangleMesh;
    use draper_geometry::Point3d;
    use std::io::Read;

    /// Create a simple unit cube mesh for testing.
    fn make_cube() -> TriangleMesh {
        let mut mesh = TriangleMesh::new();
        // 8 vertices of a unit cube
        let v0 = mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        let v1 = mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        let v2 = mesh.add_vertex(Point3d::new(1.0, 1.0, 0.0));
        let v3 = mesh.add_vertex(Point3d::new(0.0, 1.0, 0.0));
        let v4 = mesh.add_vertex(Point3d::new(0.0, 0.0, 1.0));
        let v5 = mesh.add_vertex(Point3d::new(1.0, 0.0, 1.0));
        let v6 = mesh.add_vertex(Point3d::new(1.0, 1.0, 1.0));
        let v7 = mesh.add_vertex(Point3d::new(0.0, 1.0, 1.0));

        // 12 triangles (2 per face)
        // Front (z=0)
        mesh.add_triangle(v0, v1, v2);
        mesh.add_triangle(v0, v2, v3);
        // Back (z=1)
        mesh.add_triangle(v4, v6, v5);
        mesh.add_triangle(v4, v7, v6);
        // Bottom (y=0)
        mesh.add_triangle(v0, v5, v1);
        mesh.add_triangle(v0, v4, v5);
        // Top (y=1)
        mesh.add_triangle(v3, v2, v6);
        mesh.add_triangle(v3, v6, v7);
        // Left (x=0)
        mesh.add_triangle(v0, v3, v7);
        mesh.add_triangle(v0, v7, v4);
        // Right (x=1)
        mesh.add_triangle(v1, v5, v6);
        mesh.add_triangle(v1, v6, v2);

        mesh.compute_face_normals();
        mesh
    }

    #[test]
    fn test_obj_export_content() {
        let mesh = make_cube();
        let obj = build_obj(&mesh);

        // Check header
        assert!(obj.starts_with("# 3Draper OBJ Export\n"));

        // Check vertex lines
        let v_lines: Vec<&str> = obj.lines().filter(|l| l.starts_with("v ")).collect();
        assert_eq!(v_lines.len(), 8);

        // Check face lines
        let f_lines: Vec<&str> = obj.lines().filter(|l| l.starts_with("f ")).collect();
        assert_eq!(f_lines.len(), 12);

        // No normals in this mesh (normals field is None), so no vn lines
        assert!(obj.lines().all(|l| !l.starts_with("vn ")));

        // Face indices should be 1-based
        assert!(obj.contains("f 1 2 3"));
    }

    #[test]
    fn test_obj_export_with_normals() {
        let mut mesh = make_cube();
        // Add vertex normals
        mesh.normals = Some(vec![
            [0.0, 0.0, -1.0], // v0
            [0.0, 0.0, -1.0], // v1
            [0.0, 0.0, -1.0], // v2
            [0.0, 0.0, -1.0], // v3
            [0.0, 0.0, 1.0],  // v4
            [0.0, 0.0, 1.0],  // v5
            [0.0, 0.0, 1.0],  // v6
            [0.0, 0.0, 1.0],  // v7
        ]);

        let obj = build_obj(&mesh);

        // Check vn lines
        let vn_lines: Vec<&str> = obj.lines().filter(|l| l.starts_with("vn ")).collect();
        assert_eq!(vn_lines.len(), 8);

        // Face format should include normals: f v//vn
        let f_lines: Vec<&str> = obj.lines().filter(|l| l.starts_with("f ")).collect();
        assert!(f_lines[0].contains("//"), "Expected f v//vn format, got: {}", f_lines[0]);
    }

    #[test]
    fn test_obj_export_to_file() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("test.obj");
        let path_str = path.to_str().unwrap();

        let mesh = make_cube();
        export_obj(&mesh, path_str).unwrap();

        let content = std::fs::read_to_string(path).unwrap();
        assert!(content.starts_with("# 3Draper OBJ Export"));
        assert!(content.contains("v 0 0 0"));
        assert!(content.contains("v 1 0 0"));
    }

    #[test]
    fn test_gltf_export_glb_structure() {
        let mesh = make_cube();
        let glb = build_glb(&mesh).unwrap();

        // Check GLB header
        assert_eq!(&glb[0..4], b"glTF"); // magic
        assert_eq!(u32::from_le_bytes(glb[4..8].try_into().unwrap()), 2); // version
        let total_len = u32::from_le_bytes(glb[8..12].try_into().unwrap());
        assert_eq!(total_len as usize, glb.len());

        // Check JSON chunk
        let json_len = u32::from_le_bytes(glb[12..16].try_into().unwrap());
        assert_eq!(&glb[16..20], b"JSON");
        let json_str = String::from_utf8_lossy(&glb[20..(20 + json_len as usize)]);
        assert!(json_str.contains("\"asset\""));
        assert!(json_str.contains("\"version\": \"2.0\""));
        assert!(json_str.contains("\"POSITION\""));
        assert!(json_str.contains("\"NORMAL\""));

        // Check BIN chunk
        let bin_start = 20 + json_len as usize;
        let bin_len = u32::from_le_bytes(glb[bin_start..(bin_start + 4)].try_into().unwrap());
        assert_eq!(&glb[(bin_start + 4)..(bin_start + 8)], b"BIN\0");
        assert!(bin_len > 0);
    }

    #[test]
    fn test_gltf_export_to_file() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("test.glb");
        let path_str = path.to_str().unwrap();

        let mesh = make_cube();
        export_gltf(&mesh, path_str).unwrap();

        let data = std::fs::read(path).unwrap();
        assert_eq!(&data[0..4], b"glTF");
    }

    #[test]
    fn test_gltf_export_with_colors() {
        let mut mesh = make_cube();
        mesh.ensure_colors([0.8, 0.2, 0.2, 1.0]);

        let glb = build_glb(&mesh).unwrap();

        // JSON should contain COLOR_0
        let json_len = u32::from_le_bytes(glb[12..16].try_into().unwrap());
        let json_str = String::from_utf8_lossy(&glb[20..(20 + json_len as usize)]);
        assert!(json_str.contains("COLOR_0"));
    }

    #[test]
    fn test_3mf_export_zip_structure() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("test.3mf");
        let path_str = path.to_str().unwrap();

        let mesh = make_cube();
        export_3mf(&mesh, path_str).unwrap();

        // Verify it's a valid ZIP
        let file = std::fs::File::open(path).unwrap();
        let mut archive = zip::ZipArchive::new(file).unwrap();

        // Check required entries
        assert!(archive.by_name("[Content_Types].xml").is_ok());
        assert!(archive.by_name("_rels/.rels").is_ok());
        assert!(archive.by_name("3D/3dmodel.model").is_ok());

        // Check model XML content
        let mut model_file = archive.by_name("3D/3dmodel.model").unwrap();
        let mut model_xml = String::new();
        model_file.read_to_string(&mut model_xml).unwrap();
        assert!(model_xml.contains("<vertex"));
        assert!(model_xml.contains("<triangle"));
        assert!(model_xml.contains("xmlns="));
    }

    #[test]
    fn test_3mf_model_xml_content() {
        let mesh = make_cube();
        let xml = build_3mf_model(&mesh);

        // Should contain all 8 vertices (<vertex x=... not <vertices>)
        let vertex_count = xml.matches("<vertex ").count();
        assert_eq!(vertex_count, 8);

        // Should contain all 12 triangles
        let triangle_count = xml.matches("<triangle ").count();
        assert_eq!(triangle_count, 12);
    }

    #[test]
    fn test_usd_export_stub() {
        let mesh = make_cube();
        let result = export_usd(&mesh, "test.usda");
        assert!(result.is_err());
        match result {
            Err(ExportError::UnsupportedFormat(msg)) => {
                assert!(msg.contains("USD"));
            }
            _ => panic!("Expected UnsupportedFormat error"),
        }
    }

    #[test]
    fn test_stl_export_binary() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("test.stl");
        let path_str = path.to_str().unwrap();

        let mesh = make_cube();
        export_stl(&mesh, path_str, true).unwrap();

        let data = std::fs::read(path).unwrap();
        // Binary STL: 80 byte header + 4 byte triangle count + 50 bytes per triangle
        assert!(data.len() >= 84);
        let num_triangles = u32::from_le_bytes(data[80..84].try_into().unwrap());
        assert_eq!(num_triangles, 12);
        assert_eq!(data.len(), 84 + 12 * 50);
    }

    #[test]
    fn test_stl_export_ascii() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("test.stl");
        let path_str = path.to_str().unwrap();

        let mesh = make_cube();
        export_stl(&mesh, path_str, false).unwrap();

        let content = std::fs::read_to_string(path).unwrap();
        assert!(content.starts_with("solid"));
        assert!(content.contains("endsolid"));
        assert!(content.contains("facet normal"));
        assert!(content.contains("vertex"));
    }

    #[test]
    fn test_export_empty_mesh_errors() {
        let empty = TriangleMesh::new();

        assert!(export_obj(&empty, "test.obj").is_err());
        assert!(export_gltf(&empty, "test.glb").is_err());
        assert!(export_3mf(&empty, "test.3mf").is_err());
        assert!(export_stl(&empty, "test.stl", true).is_err());
    }

    #[test]
    fn test_stl_binary_header() {
        let mesh = make_cube();
        let data = crate::stl::export_stl_binary(&mesh, "TestHeader");

        // Header should start with the name
        let header = String::from_utf8_lossy(&data[..80]);
        assert!(header.starts_with("3Draper STL - TestHeader"));

        // Triangle count
        let num_tri = u32::from_le_bytes(data[80..84].try_into().unwrap());
        assert_eq!(num_tri, 12);

        // Total size
        assert_eq!(data.len(), 84 + 12 * 50);
    }
}
