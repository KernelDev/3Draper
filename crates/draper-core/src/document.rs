//! Document — the main high-level container for a 3D model.
//!
//! A Document wraps a STEP document, a B-rep Shape, and generated meshes.

use crate::error::{CoreError, CoreResult};
use crate::scene::Scene;
use crate::step_bridge;
use draper_mesh::generate::generate_mesh;
use draper_mesh::triangulate::TriangleMesh;
use draper_step::ast::StepDocument;
use draper_step::parser::{parse_step, write_step};
use draper_topology::shape::Shape;
use std::path::Path;

/// The main document type for 3Draper.
pub struct Document {
    /// The STEP document (raw parsed data).
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
        let step_doc = parse_step(&content)?;

        let mut doc = Self {
            step_doc: Some(step_doc.clone()),
            shape: Shape::new(),
            meshes: Vec::new(),
            scene: Scene::new(),
            file_path: Some(path.to_string_lossy().to_string()),
        };

        // Convert STEP entities to B-rep shape
        step_bridge::step_to_shape(&step_doc, &mut doc.shape);

        // Generate meshes from the shape
        doc.regenerate_meshes();

        // Build scene from the shape
        doc.scene = Scene::from_shape(&doc.shape);

        Ok(doc)
    }

    /// Save the document as a STEP file.
    pub fn save_step(&self, path: &Path) -> CoreResult<()> {
        let step_doc = if let Some(ref doc) = self.step_doc {
            doc.clone()
        } else {
            // Generate a STEP document from the shape
            step_bridge::shape_to_step(&self.shape)?
        };

        let content = write_step(&step_doc);
        std::fs::write(path, content)?;
        Ok(())
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

use draper_step::ast::StructureNode;

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
