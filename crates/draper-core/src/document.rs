//! Document — the main high-level container for a 3D model.
//!
//! A Document wraps a STEP model, a B-rep Shape, and generated meshes.

use crate::error::{CoreError, CoreResult};
use crate::scene::Scene;
use crate::step_bridge;
use draper_mesh::generate::generate_mesh;
use draper_mesh::triangulate::TriangleMesh;
use draper_step::ast::{StepDocument, StructureNode};
use draper_step::bridge::{parse_step, write_step};
use draper_step::StepModel;
use draper_topology::shape::Shape;
use std::path::Path;

/// The main document type for 3Draper.
pub struct Document {
    /// The STEP model (step-io typed IR).
    pub step_model: Option<StepModel>,
    /// The STEP document (backward-compatible AST for structure tree).
    pub step_doc: Option<StepDocument>,
    /// The B-rep shape (constructed from STEP or built programmatically).
    pub shape: Shape,
    /// The generated triangle meshes (one per part/solid).
    pub meshes: Vec<TriangleMesh>,
    /// The scene graph.
    pub scene: Scene,
    /// The file path this document was loaded from (if any).
    pub file_path: Option<String>,
}

impl Document {
    /// Create a new empty document.
    pub fn new() -> Self {
        Self {
            step_model: None,
            step_doc: None,
            shape: Shape::new(),
            meshes: Vec::new(),
            scene: Scene::new(),
            file_path: None,
        }
    }

    /// Load a STEP file.
    pub fn open_step(path: &Path) -> CoreResult<Self> {
        let content = std::fs::read_to_string(path)?;
        log::info!("Read {} bytes from {}", content.len(), path.display());

        let parsed = parse_step(&content)?;

        // Convert step-io's StepModel to our B-rep Shape
        let shape = step_bridge::step_model_to_shape(&parsed.model);

        let mut doc = Self {
            step_model: Some(parsed.model),
            step_doc: Some(parsed.document.clone()),
            shape,
            meshes: Vec::new(),
            scene: Scene::new(),
            file_path: Some(path.to_string_lossy().to_string()),
        };

        // Generate meshes from the shape
        doc.regenerate_meshes();

        // Build scene from the shape
        doc.scene = Scene::from_shape(&doc.shape);

        Ok(doc)
    }

    /// Save the document as a STEP file.
    pub fn save_step(&self, path: &Path) -> CoreResult<()> {
        // If we have the original StepModel, use step-io's writer
        if let Some(ref model) = self.step_model {
            let model_clone = model.clone();
            let mut output = Vec::new();
            model_clone.write_to(&mut output).map_err(|e| {
                CoreError::InvalidOperation(format!("STEP write error: {:?}", e))
            })?;
            std::fs::write(path, &output)?;
            return Ok(());
        }

        // Fallback: use our backward-compatible writer
        if let Some(ref doc) = self.step_doc {
            let content = write_step(doc);
            std::fs::write(path, content)?;
            return Ok(());
        }

        Err(CoreError::InvalidOperation(
            "No STEP data to save. Open a STEP file first.".to_string(),
        ))
    }

    /// Save as STEP to the same file it was loaded from.
    pub fn save(&self) -> CoreResult<()> {
        if let Some(ref path) = self.file_path {
            self.save_step(Path::new(path))
        } else {
            Err(CoreError::InvalidOperation(
                "No file path set. Use save_step() with a path.".to_string(),
            ))
        }
    }

    /// Regenerate all meshes from the current shape.
    pub fn regenerate_meshes(&mut self) {
        self.meshes.clear();

        // For now, generate a single mesh from all faces
        let mesh = generate_mesh(&self.shape, 32, 32);
        if !mesh.vertices.is_empty() {
            self.meshes.push(mesh);
        }
    }

    /// Get the structure tree for display in the UI.
    pub fn structure_tree(&self) -> Option<StructureNode> {
        self.step_doc.as_ref().map(|doc| doc.structure_tree())
    }

    /// Get statistics about the document.
    pub fn statistics(&self) -> DocumentStatistics {
        let total_vertices = self.shape.vertices().len();
        let total_edges = self.shape.edges().len();
        let total_faces = self.shape.faces().len();
        let total_solids = self.shape.solids().len();
        let total_triangles: usize = self.meshes.iter().map(|m| m.triangle_count()).sum();
        let total_mesh_vertices: usize = self.meshes.iter().map(|m| m.vertex_count()).sum();

        DocumentStatistics {
            total_vertices,
            total_edges,
            total_faces,
            total_solids,
            total_triangles,
            total_mesh_vertices,
        }
    }
}

impl Default for Document {
    fn default() -> Self {
        Self::new()
    }
}

/// Statistics about the document.
#[derive(Debug, Clone)]
pub struct DocumentStatistics {
    pub total_vertices: usize,
    pub total_edges: usize,
    pub total_faces: usize,
    pub total_solids: usize,
    pub total_triangles: usize,
    pub total_mesh_vertices: usize,
}
