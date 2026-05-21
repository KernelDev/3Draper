//! Scene graph for organizing 3D objects.
//!
//! The scene provides a hierarchical structure for rendering and
//! interaction, mapping to the topological shape structure.

use draper_geometry::point::{BoundingBox3, Point3};
use draper_geometry::transform::Transform3;
use draper_topology::entity::*;
use draper_topology::shape::Shape;

/// A node in the scene graph.
#[derive(Debug, Clone)]
pub struct SceneNode {
    pub id: u64,
    pub name: String,
    pub node_type: SceneNodeType,
    pub transform: Transform3,
    pub children: Vec<SceneNode>,
    pub visible: bool,
    pub selected: bool,
    /// Reference to the topological entity ID.
    pub topo_id: Option<TopoId>,
    /// Color for rendering [r, g, b] in 0..1.
    pub color: [f32; 3],
}

/// Type of scene node.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SceneNodeType {
    Root,
    Product,
    Part,
    Solid,
    Face,
    Edge,
    Vertex,
    Group,
}

/// The scene graph.
#[derive(Debug, Clone)]
pub struct Scene {
    pub root: SceneNode,
    pub next_id: u64,
}

impl Scene {
    pub fn new() -> Self {
        Self {
            root: SceneNode {
                id: 0,
                name: "Scene".to_string(),
                node_type: SceneNodeType::Root,
                transform: Transform3::IDENTITY,
                children: Vec::new(),
                visible: true,
                selected: false,
                topo_id: None,
                color: [0.7, 0.7, 0.7],
            },
            next_id: 1,
        }
    }

    /// Build a scene from a topological shape.
    pub fn from_shape(shape: &Shape) -> Self {
        let mut scene = Self::new();

        for &solid_id in shape.roots() {
            if let Some(TopoShape::Solid(solid)) = shape.get(solid_id) {
                let mut solid_node = SceneNode {
                    id: scene.alloc_id(),
                    name: format!("Solid #{}", solid.id),
                    node_type: SceneNodeType::Solid,
                    transform: Transform3::IDENTITY,
                    children: Vec::new(),
                    visible: true,
                    selected: false,
                    topo_id: Some(solid.id),
                    color: [0.6, 0.7, 0.85],
                };

                if let Some(TopoShape::Shell(shell)) = shape.get(solid.outer_shell) {
                    for &face_id in &shell.faces {
                        if let Some(TopoShape::Face(face)) = shape.get(face_id) {
                            let face_node = SceneNode {
                                id: scene.alloc_id(),
                                name: format!("Face #{}", face.id),
                                node_type: SceneNodeType::Face,
                                transform: Transform3::IDENTITY,
                                children: Vec::new(),
                                visible: true,
                                selected: false,
                                topo_id: Some(face.id),
                                color: [0.6, 0.7, 0.85],
                            };
                            solid_node.children.push(face_node);
                        }
                    }
                }

                scene.root.children.push(solid_node);
            }
        }

        scene
    }

    /// Build a scene from the STEP structure tree.
    pub fn from_step_structure(tree: &draper_step::ast::StructureNode) -> Self {
        let mut scene = Self::new();
        fn convert_node(node: &draper_step::ast::StructureNode, scene: &mut Scene) -> SceneNode {
            let scene_node = SceneNode {
                id: scene.alloc_id(),
                name: node.name.clone(),
                node_type: match node.type_name.as_str() {
                    "PRODUCT" => SceneNodeType::Product,
                    "MANIFOLD_SOLID_BREP" => SceneNodeType::Solid,
                    "ADVANCED_FACE" => SceneNodeType::Face,
                    _ => SceneNodeType::Group,
                },
                transform: Transform3::IDENTITY,
                children: node.children.iter().map(|c| convert_node(c, scene)).collect(),
                visible: true,
                selected: false,
                topo_id: node.entity_id,
                color: [0.6, 0.7, 0.85],
            };
            scene_node
        }

        scene.root = SceneNode {
            id: 0,
            name: tree.name.clone(),
            node_type: SceneNodeType::Root,
            transform: Transform3::IDENTITY,
            children: tree.children.iter().map(|c| convert_node(c, &mut scene)).collect(),
            visible: true,
            selected: false,
            topo_id: None,
            color: [0.7, 0.7, 0.7],
        };

        scene
    }

    fn alloc_id(&mut self) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    /// Compute the bounding box of all visible nodes.
    pub fn bounding_box(&self) -> BoundingBox3 {
        let mut bb = BoundingBox3::empty();
        fn collect_visible_points(node: &SceneNode, bb: &mut BoundingBox3) {
            if !node.visible {
                return;
            }
            // For now, we don't store geometry in the scene —
            // this will be enriched when we have meshes per node.
            for child in &node.children {
                collect_visible_points(child, bb);
            }
        }
        collect_visible_points(&self.root, &mut bb);
        bb
    }

    /// Find a node by its ID.
    pub fn find_node(&self, id: u64) -> Option<&SceneNode> {
        fn find(node: &SceneNode, id: u64) -> Option<&SceneNode> {
            if node.id == id {
                return Some(node);
            }
            for child in &node.children {
                if let Some(found) = find(child, id) {
                    return Some(found);
                }
            }
            None
        }
        find(&self.root, id)
    }

    /// Find a mutable node by its ID.
    pub fn find_node_mut(&mut self, id: u64) -> Option<&mut SceneNode> {
        fn find(node: &mut SceneNode, id: u64) -> Option<&mut SceneNode> {
            if node.id == id {
                return Some(node);
            }
            for child in &mut node.children {
                if let Some(found) = find(child, id) {
                    return Some(found);
                }
            }
            None
        }
        find(&mut self.root, id)
    }
}

impl Default for Scene {
    fn default() -> Self {
        Self::new()
    }
}
