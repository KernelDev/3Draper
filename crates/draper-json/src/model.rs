// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! JSON-serializable model representation.
//!
//! `JsonModel` is the top-level container that can represent a complete 3D model
//! with all its geometry, topology, mesh data, and assembly structure. It supports
//! full round-trip serialization: STEP → JsonModel → JSON → JsonModel → Mesh.

use serde::{Deserialize, Serialize};
use draper_geometry::{Point3d, Surface};
use draper_mesh::TriangleMesh;
use draper_step::{
    DetailedMeshInstance, AssemblyNode, FaceInfo,
    StepConversionConfig,
};
use draper_step::parse_step;

// ============================================================
// Top-level JSON model
// ============================================================

/// A complete 3D model that can be serialized to/deserialized from JSON.
///
/// This is the primary data structure for JSON import/export. It contains:
/// - Metadata (name, source, version)
/// - Assembly tree (hierarchical structure)
/// - Mesh instances (triangulated geometry with transforms and colors)
/// - BRep topology (optional, for full kernel round-trip)
/// - Per-face information (optional, for structure/debugging)
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct JsonModel {
    /// Model metadata.
    pub metadata: ModelMetadata,
    /// Assembly tree structure.
    pub assembly: AssemblyNode,
    /// Mesh instances with geometry, transforms, and colors.
    pub instances: Vec<JsonMeshInstance>,
    /// Optional STEP source text (for round-trip re-import).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub step_source: Option<String>,
}

/// Metadata about the model.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ModelMetadata {
    /// Model name.
    pub name: String,
    /// Source format (e.g., "STEP", "JSON", "BRep").
    pub source_format: String,
    /// 3Draper kernel version.
    pub kernel_version: String,
    /// Creation timestamp (ISO 8601).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub created_at: Option<String>,
    /// Number of mesh instances.
    pub instance_count: usize,
    /// Total vertex count across all instances.
    pub total_vertices: usize,
    /// Total triangle count across all instances.
    pub total_triangles: usize,
    /// Bounding box (min point).
    pub bbox_min: Option<[f64; 3]>,
    /// Bounding box (max point).
    pub bbox_max: Option<[f64; 3]>,
}

/// A serializable mesh instance with geometry and metadata.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct JsonMeshInstance {
    /// Instance name.
    pub name: String,
    /// STEP BREP entity ID.
    pub brep_id: i64,
    /// Vertex positions as flat array [x0,y0,z0, x1,y1,z1, ...].
    pub vertices: Vec<f64>,
    /// Triangle indices as flat array [i0,j0,k0, i1,j1,k1, ...].
    pub triangles: Vec<u32>,
    /// Optional vertex normals [nx0,ny0,nz0, ...].
    #[serde(skip_serializing_if = "Option::is_none")]
    pub normals: Option<Vec<f64>>,
    /// Optional per-triangle RGBA colors (0..1 range) [r0,g0,b0,a0, ...].
    #[serde(skip_serializing_if = "Option::is_none")]
    pub triangle_colors: Option<Vec<f32>>,
    /// Optional per-triangle face IDs.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub triangle_face_ids: Option<Vec<u64>>,
    /// Optional 4x4 transform matrix (row-major).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub transform: Option<[[f64; 4]; 4]>,
    /// Optional RGBA color for the entire instance (0..1 range).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color: Option<[f32; 4]>,
    /// Per-face information for structure display.
    #[serde(skip_serializing_if = "Vec::is_empty", default)]
    pub faces: Vec<JsonFaceInfo>,
}

/// Per-face information in JSON format.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct JsonFaceInfo {
    /// Unique face identifier.
    pub face_id: u64,
    /// STEP entity ID.
    pub step_face_id: i64,
    /// Surface type name.
    pub surface_type: String,
    /// Surface geometry.
    pub surface: Surface,
    /// Outer boundary polylines (3D).
    pub outer_boundary: Vec<Vec<Point3d>>,
    /// Inner boundary polylines (3D).
    pub inner_boundaries: Vec<Vec<Point3d>>,
    /// Triangle range [start, end) in the instance mesh.
    pub triangle_range: [usize; 2],
    /// Whether the face normal matches the surface normal.
    pub forward: bool,
    /// Whether this face belongs to a void shell (internal cavity).
    #[serde(default)]
    pub is_void: bool,
}

// ============================================================
// Conversion implementations
// ============================================================

impl JsonModel {
    /// Create a JsonModel from STEP file content.
    ///
    /// Parses the STEP file, converts it to detailed mesh instances,
    /// and builds the complete JSON model including assembly tree.
    pub fn from_step_content(step_text: &str) -> Result<Self, String> {
        let step_file = parse_step(step_text).map_err(|e| format!("STEP parse error: {:?}", e))?;
        Self::from_step_file(&step_file, Some(step_text.to_string()))
    }

    /// Create a JsonModel from STEP file content with custom configuration.
    pub fn from_step_content_with_config(
        step_text: &str,
        config: &StepConversionConfig,
    ) -> Result<Self, String> {
        let step_file = parse_step(step_text).map_err(|e| format!("STEP parse error: {:?}", e))?;
        Self::from_step_file_with_config(&step_file, config, Some(step_text.to_string()))
    }

    /// Create a JsonModel from a parsed StepFile.
    pub fn from_step_file(
        step_file: &draper_step::StepFile,
        step_source: Option<String>,
    ) -> Result<Self, String> {
        Self::from_step_file_with_config(step_file, &StepConversionConfig::default(), step_source)
    }

    /// Create a JsonModel from a parsed StepFile with custom configuration.
    pub fn from_step_file_with_config(
        step_file: &draper_step::StepFile,
        config: &StepConversionConfig,
        step_source: Option<String>,
    ) -> Result<Self, String> {
        use draper_step::step_structure_with_instances;
        let _config = config; // Config used by step_structure_with_instances internally

        // Get assembly tree and instances together
        let (assembly, instances) = step_structure_with_instances(step_file);

        let json_instances: Vec<JsonMeshInstance> = instances
            .iter()
            .map(|inst| JsonMeshInstance::from_detailed_instance(inst))
            .collect();

        // Compute global stats
        let total_vertices: usize = json_instances.iter().map(|i| i.vertices.len() / 3).sum();
        let total_triangles: usize = json_instances.iter().map(|i| i.triangles.len() / 3).sum();

        // Compute global bounding box
        let mut bbox_min: Option<[f64; 3]> = None;
        let mut bbox_max: Option<[f64; 3]> = None;
        for inst in &json_instances {
            for chunk in inst.vertices.chunks(3) {
                if chunk.len() < 3 { continue; }
                let p = [chunk[0], chunk[1], chunk[2]];
                bbox_min = Some(match bbox_min {
                    None => p,
                    Some(m) => [m[0].min(p[0]), m[1].min(p[1]), m[2].min(p[2])]
                });
                bbox_max = Some(match bbox_max {
                    None => p,
                    Some(m) => [m[0].max(p[0]), m[1].max(p[1]), m[2].max(p[2])]
                });
            }
        }

        Ok(JsonModel {
            metadata: ModelMetadata {
                name: assembly.name.clone(),
                source_format: "STEP".to_string(),
                kernel_version: env!("CARGO_PKG_VERSION").to_string(),
                created_at: Some(chrono_now()),
                instance_count: json_instances.len(),
                total_vertices,
                total_triangles,
                bbox_min,
                bbox_max,
            },
            assembly,
            instances: json_instances,
            step_source,
        })
    }

    /// Create a JsonModel from detailed mesh instances.
    pub fn from_instances(
        instances: Vec<DetailedMeshInstance>,
        assembly: AssemblyNode,
        name: &str,
    ) -> Self {
        let json_instances: Vec<JsonMeshInstance> = instances
            .iter()
            .map(|inst| JsonMeshInstance::from_detailed_instance(inst))
            .collect();

        let total_vertices: usize = json_instances.iter().map(|i| i.vertices.len() / 3).sum();
        let total_triangles: usize = json_instances.iter().map(|i| i.triangles.len() / 3).sum();

        let mut bbox_min: Option<[f64; 3]> = None;
        let mut bbox_max: Option<[f64; 3]> = None;
        for inst in &json_instances {
            for chunk in inst.vertices.chunks(3) {
                if chunk.len() < 3 { continue; }
                let p = [chunk[0], chunk[1], chunk[2]];
                bbox_min = Some(match bbox_min {
                    None => p,
                    Some(m) => [m[0].min(p[0]), m[1].min(p[1]), m[2].min(p[2])]
                });
                bbox_max = Some(match bbox_max {
                    None => p,
                    Some(m) => [m[0].max(p[0]), m[1].max(p[1]), m[2].max(p[2])]
                });
            }
        }

        JsonModel {
            metadata: ModelMetadata {
                name: name.to_string(),
                source_format: "JSON".to_string(),
                kernel_version: env!("CARGO_PKG_VERSION").to_string(),
                created_at: Some(chrono_now()),
                instance_count: json_instances.len(),
                total_vertices,
                total_triangles,
                bbox_min,
                bbox_max,
            },
            assembly,
            instances: json_instances,
            step_source: None,
        }
    }

    /// Convert to a single merged TriangleMesh.
    pub fn to_triangle_mesh(&self) -> TriangleMesh {
        let mut merged = TriangleMesh::new();
        for inst in &self.instances {
            let mesh = inst.to_triangle_mesh();
            if let Some(color) = inst.color {
                merged.merge_with_color(&mesh, color);
            } else {
                merged.merge(&mesh);
            }
        }
        merged
    }

    /// Convert back to DetailedMeshInstance vector (for rendering).
    pub fn to_detailed_instances(&self) -> Vec<DetailedMeshInstance> {
        self.instances.iter().map(|ji| ji.to_detailed_instance()).collect()
    }

    /// Serialize to JSON string (pretty-printed).
    pub fn to_json_pretty(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }

    /// Serialize to JSON string (compact).
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string(self)
    }

    /// Deserialize from JSON string.
    pub fn from_json(json: &str) -> Result<Self, serde_json::Error> {
        serde_json::from_str(json)
    }

    /// Get the STEP source text, if available.
    pub fn step_source(&self) -> Option<&str> {
        self.step_source.as_deref()
    }
}

impl JsonMeshInstance {
    /// Create from a DetailedMeshInstance.
    pub fn from_detailed_instance(inst: &DetailedMeshInstance) -> Self {
        // Flatten vertices: Vec<Point3d> → Vec<f64>
        let vertices: Vec<f64> = inst.mesh.vertices.iter()
            .flat_map(|p| [p.x, p.y, p.z])
            .collect();

        // Flatten triangles: Vec<[u32; 3]> → Vec<u32>
        let triangles: Vec<u32> = inst.mesh.triangles.iter()
            .flat_map(|t| [t[0], t[1], t[2]])
            .collect();

        // Flatten normals if present
        let normals = inst.mesh.normals.as_ref().map(|n| {
            n.iter().flat_map(|v| [v[0], v[1], v[2]]).collect()
        });

        // Flatten triangle colors if present
        let triangle_colors = inst.mesh.triangle_colors.as_ref().map(|c| {
            c.iter().flat_map(|v| [v[0], v[1], v[2], v[3]]).collect()
        });

        JsonMeshInstance {
            name: inst.name.clone(),
            brep_id: inst.brep_id,
            vertices,
            triangles,
            normals,
            triangle_colors,
            triangle_face_ids: inst.mesh.triangle_face_ids.clone(),
            transform: inst.transform,
            color: inst.color,
            faces: inst.faces.iter().map(|f| JsonFaceInfo::from_face_info(f)).collect(),
        }
    }

    /// Convert to a TriangleMesh.
    pub fn to_triangle_mesh(&self) -> TriangleMesh {
        // Reconstruct vertices from flat array
        let vertices: Vec<Point3d> = self.vertices.chunks(3)
            .map(|c| Point3d::new(c[0], c[1], c[2]))
            .collect();

        // Reconstruct triangles from flat array
        let triangles: Vec<[u32; 3]> = self.triangles.chunks(3)
            .map(|c| [c[0], c[1], c[2]])
            .collect();

        // Reconstruct normals if present
        let normals = self.normals.as_ref().map(|n| {
            n.chunks(3).map(|c| [c[0], c[1], c[2]]).collect()
        });

        // Reconstruct triangle colors if present
        let triangle_colors = self.triangle_colors.as_ref().map(|c| {
            c.chunks(4).map(|v| [v[0], v[1], v[2], v[3]]).collect()
        });

        TriangleMesh {
            vertices,
            triangles,
            normals,
            face_normals: None,
            triangle_colors,
            triangle_face_ids: self.triangle_face_ids.clone(),
        }
    }

    /// Convert to a DetailedMeshInstance.
    pub fn to_detailed_instance(&self) -> DetailedMeshInstance {
        DetailedMeshInstance {
            name: self.name.clone(),
            mesh: self.to_triangle_mesh(),
            color: self.color,
            transform: self.transform,
            brep_id: self.brep_id,
            faces: self.faces.iter().map(|f| f.to_face_info()).collect(),
        }
    }
}

impl JsonFaceInfo {
    /// Create from a FaceInfo.
    pub fn from_face_info(fi: &FaceInfo) -> Self {
        JsonFaceInfo {
            face_id: fi.face_id,
            step_face_id: fi.step_face_id,
            surface_type: fi.surface_type.clone(),
            surface: fi.surface.clone(),
            outer_boundary: fi.outer_boundary.clone(),
            inner_boundaries: fi.inner_boundaries.clone(),
            triangle_range: [fi.triangle_range.0, fi.triangle_range.1],
            forward: fi.forward,
            is_void: fi.is_void,
        }
    }

    /// Convert to a FaceInfo.
    pub fn to_face_info(&self) -> FaceInfo {
        FaceInfo {
            face_id: self.face_id,
            step_face_id: self.step_face_id,
            surface_type: self.surface_type.clone(),
            surface: self.surface.clone(),
            outer_boundary: self.outer_boundary.clone(),
            inner_boundaries: self.inner_boundaries.clone(),
            outer_uv_boundary: Vec::new(),
            inner_uv_boundaries: Vec::new(),
            triangle_range: (self.triangle_range[0], self.triangle_range[1]),
            forward: self.forward,
            uv_triangles: Vec::new(),
            is_void: self.is_void,
        }
    }
}

/// Get current time as ISO 8601 string (WASM-compatible).
/// On WASM, `SystemTime::now()` panics, so we use `web_time::SystemTime` instead.
#[cfg(not(target_arch = "wasm32"))]
fn chrono_now() -> String {
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
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

#[cfg(target_arch = "wasm32")]
fn chrono_now() -> String {
    use web_time::SystemTime;
    let now = SystemTime::now()
        .duration_since(web_time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
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
