//! STEP-to-mesh converter.
//!
//! Converts parsed STEP entities into a triangle mesh by:
//! 1. Resolving entity references to build geometry
//! 2. Creating B-Rep faces from STEP surface entities with boundary edges
//! 3. Triangulating using the existing mesh pipeline (ear-clipping for planar faces)
//!
//! Supported surface types:
//! - PLANE
//! - CYLINDRICAL_SURFACE
//! - SPHERICAL_SURFACE
//! - CONICAL_SURFACE
//! - TOROIDAL_SURFACE
//! - SURFACE_OF_REVOLUTION
//! - SURFACE_OF_LINEAR_EXTRUSION
//! - B_SPLINE_SURFACE_WITH_KNOTS / B_SPLINE_SURFACE / BEZIER_SURFACE
//!
//! Boundary extraction:
//! - ADVANCED_FACE → FACE_BOUND → EDGE_LOOP → ORIENTED_EDGE → EDGE_CURVE
//! - EDGE_CURVE → SURFACE_CURVE → 3D curve + vertex endpoints
//! - Boundary edges enable proper ear-clipping triangulation of planar faces

use crate::schema::{StepFile, StepValue};
use draper_geometry::{
    Point3d, Point2d, Direction3d, Surface, Plane, CylinderSurface, SphereSurface,
    ConeSurface, TorusSurface, RevolutionSurface, ExtrusionSurface,
    NurbsSurface, Curve3d, Line, Circle, NurbsCurve,
};
use draper_mesh::{TriangleMesh, TriangulationParams, triangulate_face, triangulate_face_with_boundary, ear_clip};
use draper_topology::{Face, Wire, CoEdge, Edge as TopoEdge};
use std::collections::HashMap;
use log::{info, warn};

/// Extracted face data with surface and boundary edges.
struct FaceData {
    surface: Surface,
    /// Edges from the outer boundary loop (FACE_OUTER_BOUND)
    outer_edges: Vec<TopoEdge>,
    /// Edges from inner boundary loops (FACE_BOUND = holes)
    inner_edges: Vec<Vec<TopoEdge>>,
    /// All edges combined (for backward compat with surface_to_mesh)
    edges: Vec<TopoEdge>,
    forward: bool,
}

/// A mesh instance to be rendered — the mesh geometry is transformed by the given matrix
/// and painted with the given color. Multiple instances can reference the same BREP geometry
/// but with different transforms (e.g., a bolt inserted 6 times at different positions).
#[derive(Clone, Debug)]
pub struct MeshInstance {
    /// Human-readable name (from STEP PRODUCT or NAUO).
    pub name: String,
    /// The triangulated mesh (already transformed to world space).
    pub mesh: TriangleMesh,
    /// Optional RGBA color (0..1 range).
    pub color: Option<[f32; 4]>,
    /// The 4×4 transform that was applied to get this instance into world space.
    pub transform: Option<[[f64; 4]; 4]>,
    /// The STEP entity ID of the source MANIFOLD_SOLID_BREP.
    pub brep_id: i64,
}

/// A node in the STEP assembly tree (for structure display).
#[derive(Clone, Debug)]
pub struct AssemblyNode {
    /// Name of this assembly node.
    pub name: String,
    /// STEP entity ID of the PRODUCT_DEFINITION.
    pub pd_id: i64,
    /// STEP entity ID of the MANIFOLD_SOLID_BREP (if leaf).
    pub brep_id: Option<i64>,
    /// Transform from parent to this node.
    pub transform: Option<[[f64; 4]; 4]>,
    /// Color for this node.
    pub color: Option<[f32; 4]>,
    /// Child nodes.
    pub children: Vec<AssemblyNode>,
}

/// Convert a parsed STEP file to a single merged triangle mesh.
pub fn step_to_mesh(step_file: &StepFile) -> Result<TriangleMesh, String> {
    let converter = StepConverter::new(step_file);
    converter.convert()
}

/// Convert a parsed STEP file to mesh instances (one per assembly leaf occurrence).
/// Each instance has its own transform and color — the same BREP can appear
/// multiple times with different transforms (e.g., bolt inserted 6 times).
pub fn step_to_mesh_instances(step_file: &StepFile) -> Result<Vec<MeshInstance>, String> {
    let converter = StepConverter::new(step_file);
    converter.convert_instances()
}

/// Get the assembly tree structure of a STEP file for display/debugging.
pub fn step_structure(step_file: &StepFile) -> AssemblyNode {
    let converter = StepConverter::new(step_file);
    converter.build_assembly_tree()
}

/// Get a detailed text dump of the STEP file structure, including:
/// - All NAUO (assembly) relationships with transforms
/// - All PD -> BREP mappings
/// - The full assembly tree
/// - The mesh rendering tree (which BREPs are drawn how many times with which transforms)
pub fn step_structure_detailed(step_file: &StepFile) -> String {
    let converter = StepConverter::new(step_file);
    converter.build_detailed_structure()
}

/// Get a text representation of the STEP file structure.
pub fn step_structure_text(step_file: &StepFile) -> String {
    let converter = StepConverter::new(step_file);
    let tree = converter.build_assembly_tree();
    let mut text = String::new();
    format_assembly_node(&tree, 0, &mut text);
    text
}

fn format_assembly_node(node: &AssemblyNode, depth: usize, out: &mut String) {
    let indent = "  ".repeat(depth);
    let brep_str = match node.brep_id {
        Some(id) => format!(" BREP=#{}", id),
        None => String::new(),
    };
    let color_str = match node.color {
        Some(c) => format!(" color=({:.2},{:.2},{:.2})", c[0], c[1], c[2]),
        None => String::new(),
    };
    let tf_str = match node.transform {
        Some(_) => " [T]".to_string(),
        None => String::new(),
    };
    out.push_str(&format!("{}{} (PD=#{}){}{}{}\n", indent, node.name, node.pd_id, brep_str, color_str, tf_str));
    for child in &node.children {
        format_assembly_node(child, depth + 1, out);
    }
}

/// Format assembly node with detailed transform information.
fn format_assembly_node_detailed(node: &AssemblyNode, depth: usize, out: &mut String) {
    let indent = "  ".repeat(depth);
    let node_type = if node.brep_id.is_some() { "part" } else if !node.children.is_empty() { "assembly" } else { "empty" };
    let brep_str = match node.brep_id {
        Some(id) => format!(" BREP=#{}", id),
        None => String::new(),
    };
    let color_str = match node.color {
        Some(c) => format!(" color=({:.2},{:.2},{:.2})", c[0], c[1], c[2]),
        None => String::new(),
    };
    let tf_str = match node.transform {
        Some(tf) => {
            let tx = tf[0][3]; let ty = tf[1][3]; let tz = tf[2][3];
            if tx.abs() < 1e-10 && ty.abs() < 1e-10 && tz.abs() < 1e-10 {
                " [rotation]".to_string()
            } else {
                format!(" [T:({:.1},{:.1},{:.1})]", tx, ty, tz)
            }
        }
        None => String::new(),
    };
    out.push_str(&format!(
        "{}{} [{}] (PD=#{}){}{}{}\n",
        indent, node.name, node_type, node.pd_id, brep_str, color_str, tf_str
    ));
    for child in &node.children {
        format_assembly_node_detailed(child, depth + 1, out);
    }
}

struct StepConverter<'a> {
    step: &'a StepFile,
    _entity_map: HashMap<i64, usize>,
}

impl<'a> StepConverter<'a> {
    fn new(step: &'a StepFile) -> Self {
        let entity_map: HashMap<i64, usize> = step.entities.iter()
            .enumerate()
            .map(|(i, e)| (e.id, i))
            .collect();
        Self { step, _entity_map: entity_map }
    }

    fn convert(&self) -> Result<TriangleMesh, String> {
        let instances = self.convert_instances()?;
        let mut mesh = TriangleMesh::new();
        for inst in &instances {
            if let Some(color) = inst.color {
                mesh.merge_with_color(&inst.mesh, color);
            } else {
                mesh.merge_with_color(&inst.mesh, [0.48, 0.52, 0.58, 1.0]);
            }
        }
        Ok(mesh)
    }

    /// Convert STEP to mesh instances, walking the assembly tree from root.
    /// Each leaf BREP produces one mesh instance per assembly occurrence,
    /// with the composed transform from root → leaf applied.
    fn convert_instances(&self) -> Result<Vec<MeshInstance>, String> {
        let params = TriangulationParams::default();
        let bbox = self.compute_bounding_box();
        let color_map = self.extract_color_map();
        let mut brep_mesh_cache: HashMap<i64, TriangleMesh> = HashMap::new();
        let mut results: Vec<MeshInstance> = Vec::new();

        // ─── Phase 1: Assembly-based conversion via NAUO tree walk ────────
        let nauos = self.step.find_entities_by_type("NEXT_ASSEMBLY_USAGE_OCCURRENCE");
        if !nauos.is_empty() {
            info!("Found {} NAUO assembly instances — walking assembly tree", nauos.len());

            // Build: parent_pd_id → Vec<(nauo_id, child_pd_id, nauo_name)>
            let mut parent_pd_to_children: HashMap<i64, Vec<(i64, i64, String)>> = HashMap::new();
            for nauo in &nauos {
                let (relating_pd, related_pd) = self.extract_nauo_pd_refs(nauo);
                if let (Some(parent_pd), Some(child_pd)) = (relating_pd, related_pd) {
                    let name = self.extract_nauo_name(nauo);
                    parent_pd_to_children.entry(parent_pd).or_default().push((nauo.id, child_pd, name));
                }
            }

            // Find root PD(s): PDs that are parents but are never children
            let parent_pds: std::collections::HashSet<i64> = parent_pd_to_children.keys().copied().collect();
            let child_pds: std::collections::HashSet<i64> = nauos.iter()
                .filter_map(|n| self.extract_nauo_pd_refs(n).1)
                .collect();
            let roots: Vec<i64> = parent_pds.difference(&child_pds).copied().collect();

            if roots.is_empty() {
                info!("No root assembly found, falling back to direct BREP conversion");
            } else {
                for root_pd in &roots {
                    let root_name = self.get_product_name(*root_pd);
                    info!("Root assembly: PD=#{} name='{}'", root_pd, root_name);
                    self.walk_assembly_tree(
                        *root_pd,
                        &root_name,
                        &None,
                        &color_map,
                        &mut brep_mesh_cache,
                        &params,
                        &bbox,
                        &parent_pd_to_children,
                        &mut results,
                        &mut std::collections::HashSet::new(),
                    );
                }
            }

            if !results.is_empty() {
                info!("Assembly conversion: {} mesh instances", results.len());
                return Ok(results);
            }
        }

        // ─── Phase 2: No assembly structure — try direct BREP conversion ───
        let breps = self.step.find_entities_by_type("MANIFOLD_SOLID_BREP");
        if !breps.is_empty() {
            for brep in &breps {
                let name = self.get_brep_name(brep.id);
                if let Some(mesh) = self.triangulate_brep_cached(brep.id, &mut brep_mesh_cache, &params, &bbox) {
                    let color = color_map.get(&brep.id).copied();
                    results.push(MeshInstance {
                        name,
                        mesh,
                        color,
                        transform: None,
                        brep_id: brep.id,
                    });
                }
            }
        }

        if !results.is_empty() {
            return Ok(results);
        }

        // FACETED_BREP
        let faceted = self.step.find_entities_by_type("FACETED_BREP");
        if !faceted.is_empty() {
            for fb in &faceted {
                let name = self.get_brep_name(fb.id);
                if let Some(mesh) = self.triangulate_brep_cached(fb.id, &mut brep_mesh_cache, &params, &bbox) {
                    let color = color_map.get(&fb.id).copied();
                    results.push(MeshInstance {
                        name,
                        mesh,
                        color,
                        transform: None,
                        brep_id: fb.id,
                    });
                }
            }
        }

        if !results.is_empty() {
            return Ok(results);
        }

        // ADVANCED_BREP_SHAPE_REPRESENTATION
        let abrep = self.step.find_entities_by_type("ADVANCED_BREP_SHAPE_REPRESENTATION");
        for ab in &abrep {
            for param in &ab.params {
                if let Some(ref_id) = self.get_ref(param) {
                    if let Some(entity) = self.step.find_entity(ref_id) {
                        if entity.type_name == "MANIFOLD_SOLID_BREP" {
                            let name = self.get_brep_name(entity.id);
                            if let Some(mesh) = self.triangulate_brep_cached(entity.id, &mut brep_mesh_cache, &params, &bbox) {
                                let color = color_map.get(&entity.id).copied();
                                results.push(MeshInstance {
                                    name,
                                    mesh,
                                    color,
                                    transform: None,
                                    brep_id: entity.id,
                                });
                            }
                        }
                    }
                }
                if let StepValue::List(items) = param {
                    for item in items {
                        if let Some(ref_id) = self.get_ref(item) {
                            if let Some(entity) = self.step.find_entity(ref_id) {
                                if entity.type_name == "MANIFOLD_SOLID_BREP" {
                                    let name = self.get_brep_name(entity.id);
                                    if let Some(mesh) = self.triangulate_brep_cached(entity.id, &mut brep_mesh_cache, &params, &bbox) {
                                        let color = color_map.get(&entity.id).copied();
                                        results.push(MeshInstance {
                                            name,
                                            mesh,
                                            color,
                                            transform: None,
                                            brep_id: entity.id,
                                        });
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        if !results.is_empty() {
            return Ok(results);
        }

        // ─── Phase 3: SHELL_BASED_SURFACE_MODEL / MANIFOLD_SURFACE_SHAPE_REPRESENTATION ──
        // Some STEP files use surface models instead of solid BREP models
        let shell_models = self.step.find_entities_by_type("SHELL_BASED_SURFACE_MODEL");
        for sm in &shell_models {
            for param in &sm.params {
                // Look for shell references (OPEN_SHELL, CLOSED_SHELL)
                if let Some(shell_id) = self.get_ref(param) {
                    if let Some(mesh) = self.triangulate_shell_by_id(shell_id, &params, &bbox) {
                        results.push(MeshInstance {
                            name: format!("ShellModel#{}", sm.id),
                            mesh,
                            color: None,
                            transform: None,
                            brep_id: sm.id,
                        });
                    }
                }
                if let StepValue::List(items) = param {
                    for item in items {
                        if let Some(shell_id) = self.get_ref(item) {
                            if let Some(mesh) = self.triangulate_shell_by_id(shell_id, &params, &bbox) {
                                results.push(MeshInstance {
                                    name: format!("ShellModel#{}", sm.id),
                                    mesh,
                                    color: None,
                                    transform: None,
                                    brep_id: sm.id,
                                });
                            }
                        }
                    }
                }
            }
        }

        // Also try MANIFOLD_SURFACE_SHAPE_REPRESENTATION
        let msr = self.step.find_entities_by_type("MANIFOLD_SURFACE_SHAPE_REPRESENTATION");
        for ms in &msr {
            for param in &ms.params {
                if let Some(ref_id) = self.get_ref(param) {
                    if let Some(entity) = self.step.find_entity(ref_id) {
                        if entity.type_name.contains("SHELL") {
                            if let Some(mesh) = self.triangulate_shell_by_id(ref_id, &params, &bbox) {
                                results.push(MeshInstance {
                                    name: format!("SurfaceModel#{}", ms.id),
                                    mesh,
                                    color: None,
                                    transform: None,
                                    brep_id: ms.id,
                                });
                            }
                        }
                    }
                }
                if let StepValue::List(items) = param {
                    for item in items {
                        if let Some(ref_id) = self.get_ref(item) {
                            if let Some(entity) = self.step.find_entity(ref_id) {
                                if entity.type_name.contains("SHELL") {
                                    if let Some(mesh) = self.triangulate_shell_by_id(ref_id, &params, &bbox) {
                                        results.push(MeshInstance {
                                            name: format!("SurfaceModel#{}", ms.id),
                                            mesh,
                                            color: None,
                                            transform: None,
                                            brep_id: ms.id,
                                        });
                                    }
                                } else if entity.type_name.contains("SHELL_BASED_SURFACE_MODEL") {
                                    // Follow the reference chain
                                    for sp in &entity.params {
                                        if let Some(shell_id) = self.get_ref(sp) {
                                            if let Some(mesh) = self.triangulate_shell_by_id(shell_id, &params, &bbox) {
                                                results.push(MeshInstance {
                                                    name: format!("SurfaceModel#{}", ms.id),
                                                    mesh,
                                                    color: None,
                                                    transform: None,
                                                    brep_id: ms.id,
                                                });
                                            }
                                        }
                                        if let StepValue::List(inner) = sp {
                                            for inner_item in inner {
                                                if let Some(shell_id) = self.get_ref(inner_item) {
                                                    if let Some(mesh) = self.triangulate_shell_by_id(shell_id, &params, &bbox) {
                                                        results.push(MeshInstance {
                                                            name: format!("SurfaceModel#{}", ms.id),
                                                            mesh,
                                                            color: None,
                                                            transform: None,
                                                            brep_id: ms.id,
                                                        });
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        if !results.is_empty() {
            return Ok(results);
        }

        // Direct surface extraction fallback
        let surface_types = [
            "PLANE", "CYLINDRICAL_SURFACE", "SPHERICAL_SURFACE",
            "CONICAL_SURFACE", "TOROIDAL_SURFACE", "SURFACE_OF_REVOLUTION",
            "SURFACE_OF_LINEAR_EXTRUSION", "B_SPLINE_SURFACE_WITH_KNOTS",
            "B_SPLINE_SURFACE", "BEZIER_SURFACE",
        ];
        for type_name in &surface_types {
            for entity in self.step.find_entities_by_type(type_name) {
                if let Some(surface) = self.extract_surface(entity.id) {
                    let face_data = FaceData { surface, outer_edges: vec![], inner_edges: vec![], edges: vec![], forward: true };
                    let mesh = self.surface_to_mesh(&face_data, &params, &bbox);
                    results.push(MeshInstance {
                        name: entity.type_name.clone(),
                        mesh,
                        color: None,
                        transform: None,
                        brep_id: entity.id,
                    });
                }
            }
        }

        if !results.is_empty() {
            return Ok(results);
        }

        // Point cloud fallback
        let points: Vec<Point3d> = self.step.find_entities_by_type("CARTESIAN_POINT")
            .iter()
            .filter_map(|e| self.resolve_cartesian_point(e.id))
            .collect();
        if points.len() >= 3 {
            let mut mesh = TriangleMesh::new();
            for p in &points { mesh.add_vertex(*p); }
            for i in 1..points.len().saturating_sub(1) {
                mesh.add_triangle(0, i as u32, (i + 1) as u32);
            }
            if mesh.triangle_count() > 0 {
                results.push(MeshInstance {
                    name: "Point Cloud".to_string(),
                    mesh,
                    color: None,
                    transform: None,
                    brep_id: 0,
                });
                return Ok(results);
            }
        }

        Err("No convertible surface geometry found in STEP file".to_string())
    }

    /// Recursively walk the assembly tree from a parent PD, creating mesh instances
    /// for each leaf BREP occurrence with the correct composed transform.
    fn walk_assembly_tree(
        &self,
        parent_pd_id: i64,
        _parent_name: &str,
        parent_transform: &Option<[[f64; 4]; 4]>,
        color_map: &HashMap<i64, [f32; 4]>,
        brep_mesh_cache: &mut HashMap<i64, TriangleMesh>,
        params: &TriangulationParams,
        bbox: &Option<(Point3d, Point3d)>,
        parent_pd_to_children: &HashMap<i64, Vec<(i64, i64, String)>>,
        results: &mut Vec<MeshInstance>,
        // Track (pd_id, nauo_id) pairs to detect cycles
        visited: &mut std::collections::HashSet<(i64, i64)>,
    ) {
        let children = match parent_pd_to_children.get(&parent_pd_id) {
            Some(c) => c,
            None => return, // Leaf with no children in the NAUO tree
        };

        for &(nauo_id, child_pd_id, ref nauo_name) in children {
            // Cycle detection: same PD via same NAUO = infinite loop
            if visited.contains(&(child_pd_id, nauo_id)) {
                warn!("Cycle detected: PD=#{} via NAUO=#{}", child_pd_id, nauo_id);
                continue;
            }
            visited.insert((child_pd_id, nauo_id));

            // Get this NAUO's transform (CDSR → SRR → ITEM_DEFINED_TRANSFORMATION)
            let nauo_transform = self.find_nauo_transform(nauo_id, child_pd_id);

            // Compose: parent_transform * nauo_transform
            let composed = match (parent_transform, nauo_transform) {
                (Some(pt), Some(nt)) => Some(mat4_mul(pt, &nt)),
                (Some(pt), None) => Some(*pt),
                (None, Some(nt)) => Some(nt),
                (None, None) => None,
            };

            // Check if this child PD has NAUO children (sub-assembly) FIRST.
            // A sub-assembly can have BOTH a BREP and children. In that case,
            // the BREP represents the assembly's own geometry (which duplicates
            // the children), so we should recurse into children, not add the BREP.
            let has_nauo_children = parent_pd_to_children.contains_key(&child_pd_id);

            if has_nauo_children {
                // Sub-assembly — recurse into it (even if it has a BREP)
                let child_name = self.get_product_name(child_pd_id);
                self.walk_assembly_tree(
                    child_pd_id,
                    &child_name,
                    &composed,
                    color_map,
                    brep_mesh_cache,
                    params,
                    bbox,
                    parent_pd_to_children,
                    results,
                    visited,
                );
            } else if let Some(brep_id) = self.find_pd_brep(child_pd_id) {
                // Leaf node — triangulate BREP and create instance
                if let Some(mesh) = self.triangulate_brep_cached(brep_id, brep_mesh_cache, params, bbox) {
                    let mut instance_mesh = mesh.clone();
                    if let Some(ref tf) = composed {
                        instance_mesh.transform(tf);
                    }
                    let color = color_map.get(&brep_id).copied();
                    let name = format!("{} (BREP=#{})", nauo_name, brep_id);
                    results.push(MeshInstance {
                        name,
                        mesh: instance_mesh,
                        color,
                        transform: composed,
                        brep_id,
                    });
                    info!("Instance: {} PD=#{} BREP=#{} color={:?} transform={}",
                        nauo_name, child_pd_id, brep_id, color, composed.is_some());
                }
            } else {
                warn!("PD=#{} has no NAUO children and no BREP — skipped", child_pd_id);
            }

            visited.remove(&(child_pd_id, nauo_id));
        }
    }

    /// Build the assembly tree for display/debugging purposes.
    fn build_assembly_tree(&self) -> AssemblyNode {
        let nauos = self.step.find_entities_by_type("NEXT_ASSEMBLY_USAGE_OCCURRENCE");
        let color_map = self.extract_color_map();

        // Build: parent_pd_id → Vec<(nauo_id, child_pd_id, nauo_name)>
        let mut parent_pd_to_children: HashMap<i64, Vec<(i64, i64, String)>> = HashMap::new();
        for nauo in &nauos {
            let (relating_pd, related_pd) = self.extract_nauo_pd_refs(nauo);
            if let (Some(parent_pd), Some(child_pd)) = (relating_pd, related_pd) {
                let name = self.extract_nauo_name(nauo);
                parent_pd_to_children.entry(parent_pd).or_default().push((nauo.id, child_pd, name));
            }
        }

        // Find root(s)
        let parent_pds: std::collections::HashSet<i64> = parent_pd_to_children.keys().copied().collect();
        let child_pds: std::collections::HashSet<i64> = nauos.iter()
            .filter_map(|n| self.extract_nauo_pd_refs(n).1)
            .collect();
        let roots: Vec<i64> = parent_pds.difference(&child_pds).copied().collect();

        if let Some(&root_pd) = roots.first() {
            let root_name = self.get_product_name(root_pd);
            self.build_assembly_node_recursive(
                root_pd, &root_name, &color_map, &parent_pd_to_children,
                &mut std::collections::HashSet::new(),
            )
        } else {
            // No assembly — build flat tree from BREPs
            let mut node = AssemblyNode {
                name: "No Assembly".to_string(),
                pd_id: 0,
                brep_id: None,
                transform: None,
                color: None,
                children: Vec::new(),
            };
            for brep in self.step.find_entities_by_type("MANIFOLD_SOLID_BREP") {
                let name = self.get_brep_name(brep.id);
                let color = color_map.get(&brep.id).copied();
                node.children.push(AssemblyNode {
                    name,
                    pd_id: 0,
                    brep_id: Some(brep.id),
                    transform: None,
                    color,
                    children: Vec::new(),
                });
            }
            node
        }
    }

    fn build_assembly_node_recursive(
        &self,
        pd_id: i64,
        name: &str,
        color_map: &HashMap<i64, [f32; 4]>,
        parent_pd_to_children: &HashMap<i64, Vec<(i64, i64, String)>>,
        visited: &mut std::collections::HashSet<i64>,
    ) -> AssemblyNode {
        let has_nauo_children = parent_pd_to_children.contains_key(&pd_id);
        // Only set brep_id for leaf nodes (no NAUO children)
        let brep_id = if has_nauo_children { None } else { self.find_pd_brep(pd_id) };
        let color = brep_id.and_then(|id| color_map.get(&id).copied());

        let mut node = AssemblyNode {
            name: name.to_string(),
            pd_id,
            brep_id,
            transform: None, // Transforms are per-NAUO, set in children
            color,
            children: Vec::new(),
        };

        if visited.contains(&pd_id) {
            return node; // Cycle protection
        }
        visited.insert(pd_id);

        if let Some(children) = parent_pd_to_children.get(&pd_id) {
            for &(nauo_id, child_pd_id, ref nauo_name) in children {
                let nauo_transform = self.find_nauo_transform(nauo_id, child_pd_id);
                let child_has_nauo_children = parent_pd_to_children.contains_key(&child_pd_id);
                let child_brep_id = if child_has_nauo_children { None } else { self.find_pd_brep(child_pd_id) };
                let child_color = child_brep_id.and_then(|id| color_map.get(&id).copied());
                let child_name = self.get_product_name(child_pd_id);

                let mut child_node = self.build_assembly_node_recursive(
                    child_pd_id, &child_name, color_map, parent_pd_to_children, visited,
                );
                child_node.name = format!("{} ({})", nauo_name, child_name);
                child_node.transform = nauo_transform;
                if child_color.is_some() {
                    child_node.color = child_color;
                }
                node.children.push(child_node);
            }
        }

        visited.remove(&pd_id);
        node
    }

    /// Build a detailed text representation of the STEP file structure.
    fn build_detailed_structure(&self) -> String {
        let mut out = String::new();

        // ── Section 1: Raw NAUO relationships ──
        let nauos = self.step.find_entities_by_type("NEXT_ASSEMBLY_USAGE_OCCURRENCE");
        out.push_str(&format!("== NAUO Relationships ({} total) ==\n", nauos.len()));
        for nauo in &nauos {
            let (relating_pd, related_pd) = self.extract_nauo_pd_refs(nauo);
            let nauo_name = self.extract_nauo_name(nauo);
            let parent_name = relating_pd.map(|id| self.get_product_name(id)).unwrap_or_else(|| "?".to_string());
            let child_name = related_pd.map(|id| self.get_product_name(id)).unwrap_or_else(|| "?".to_string());
            let transform = relating_pd.and_then(|_| {
                related_pd.and_then(|cpid| self.find_nauo_transform(nauo.id, cpid))
            });
            let tf_str = match transform {
                Some(tf) => {
                    let tx = tf[0][3]; let ty = tf[1][3]; let tz = tf[2][3];
                    if tx.abs() < 1e-10 && ty.abs() < 1e-10 && tz.abs() < 1e-10 {
                        "rotation only".to_string()
                    } else {
                        format!("translate({:.1},{:.1},{:.1})", tx, ty, tz)
                    }
                }
                None => "NO TRANSFORM".to_string(),
            };
            out.push_str(&format!(
                "  NAUO#{} '{}' : {}(PD#{}) → {}(PD#{}) [{}]\n",
                nauo.id, nauo_name,
                parent_name, relating_pd.unwrap_or(0),
                child_name, related_pd.unwrap_or(0),
                tf_str
            ));
        }
        out.push('\n');

        // ── Section 2: PD → BREP mappings ──
        let pds = self.step.find_entities_by_type("PRODUCT_DEFINITION");
        out.push_str(&format!("== PD → BREP Mappings ({} PDs) ==\n", pds.len()));
        for pd in &pds {
            let name = self.get_product_name(pd.id);
            let brep_id = self.find_pd_brep(pd.id);
            match brep_id {
                Some(bid) => out.push_str(&format!("  PD#{} ({}) → BREP#{}\n", pd.id, name, bid)),
                None => out.push_str(&format!("  PD#{} ({}) → no BREP (assembly)\n", pd.id, name)),
            }
        }
        out.push('\n');

        // ── Section 3: Assembly Tree ──
        let tree = self.build_assembly_tree();
        out.push_str("== STEP Assembly Tree ==\n");
        format_assembly_node_detailed(&tree, 0, &mut out);
        out.push('\n');

        out
    }

    /// Triangulate a BREP, using cache to avoid re-triangulating the same BREP multiple times.
    fn triangulate_brep_cached(
        &self,
        brep_id: i64,
        cache: &mut HashMap<i64, TriangleMesh>,
        params: &TriangulationParams,
        bbox: &Option<(Point3d, Point3d)>,
    ) -> Option<TriangleMesh> {
        if let Some(mesh) = cache.get(&brep_id) {
            return Some(mesh.clone());
        }
        let mesh = self.triangulate_brep(brep_id, params, bbox)?;
        cache.insert(brep_id, mesh.clone());
        Some(mesh)
    }

    /// Triangulate a single BREP entity.
    fn triangulate_brep(
        &self,
        brep_id: i64,
        params: &TriangulationParams,
        bbox: &Option<(Point3d, Point3d)>,
    ) -> Option<TriangleMesh> {
        let shell_id = self.find_shell_ref_by_brep_id(brep_id)?;
        let face_data_list = self.extract_shell_faces(shell_id)?;
        let mut mesh = TriangleMesh::new();
        for (fi, face_data) in face_data_list.iter().enumerate() {
            let surface_type = match &face_data.surface {
                Surface::Plane(_) => "Plane",
                Surface::Cylinder(_) => "Cylinder",
                Surface::Cone(_) => "Cone",
                Surface::Sphere(_) => "Sphere",
                Surface::Torus(_) => "Torus",
                Surface::Revolution(_) => "Revolution",
                Surface::Extrusion(_) => "Extrusion",
                Surface::Nurbs(n) => {
                    let (u0, u1) = n.u_range();
                    let (v0, v1) = n.v_range();
                    &format!("Nurbs(deg={}/{}, cps={}x{}, knots_u={}({:.2}..{:.2}), knots_v={}({:.2}..{:.2}))",
                        n.u_degree, n.v_degree, n.control_points.len(), 
                        n.control_points.first().map(|r| r.len()).unwrap_or(0),
                        n.u_knots.len(), u0, u1, n.v_knots.len(), v0, v1)
                }
            };
            let n_outer = face_data.outer_edges.len();
            let n_inner = face_data.inner_edges.len();

            // Sample the surface at a few points to check if evaluation works
            if let Surface::Nurbs(ref n) = face_data.surface {
                let (u0, u1) = n.u_range();
                let (v0, v1) = n.v_range();
                let p00 = face_data.surface.point_at(u0, v0);
                let p_mid = face_data.surface.point_at((u0+u1)/2.0, (v0+v1)/2.0);
                let p11 = face_data.surface.point_at(u1, v1);
                eprintln!("BREP #{} face[{}]: {} outer={} inner={} ur={:.4}..{:.4} vr={:.4}..{:.4}", 
                    brep_id, fi, surface_type, n_outer, n_inner, u0, u1, v0, v1);
                eprintln!("  sample(0,0)=({:.4},{:.4},{:.4}) mid=({:.4},{:.4},{:.4}) end=({:.4},{:.4},{:.4})",
                    p00.x, p00.y, p00.z, p_mid.x, p_mid.y, p_mid.z, p11.x, p11.y, p11.z);
                // Print first 3 control points
                for (ri, row) in n.control_points.iter().enumerate().take(3) {
                    for (ci, cp) in row.iter().enumerate().take(3) {
                        eprintln!("  cp[{}][{}]=({:.4},{:.4},{:.4})", ri, ci, cp.x, cp.y, cp.z);
                    }
                }
                eprintln!("  u_knots={:?}", &n.u_knots);
                eprintln!("  v_knots={:?}", &n.v_knots);
            } else {
                eprintln!("BREP #{} face[{}]: {} outer={} inner={}", brep_id, fi, surface_type, n_outer, n_inner);
            }

            let face_mesh = self.surface_to_mesh(face_data, params, bbox);
            let (fbmin, fbmax) = face_mesh.bounding_box();
            eprintln!("  -> v={} t={} bbox=({:.2},{:.2},{:.2})..({:.2},{:.2},{:.2})",
                face_mesh.vertex_count(), face_mesh.triangle_count(),
                fbmin.x, fbmin.y, fbmin.z, fbmax.x, fbmax.y, fbmax.z);
            mesh.merge(&face_mesh);
        }
        Some(mesh)
    }

    /// Triangulate a shell entity (CLOSED_SHELL, OPEN_SHELL) directly by its ID.
    fn triangulate_shell_by_id(
        &self,
        shell_id: i64,
        params: &TriangulationParams,
        bbox: &Option<(Point3d, Point3d)>,
    ) -> Option<TriangleMesh> {
        let face_data_list = self.extract_shell_faces(shell_id)?;
        let mut mesh = TriangleMesh::new();
        for face_data in &face_data_list {
            let face_mesh = self.surface_to_mesh(face_data, params, bbox);
            mesh.merge(&face_mesh);
        }
        if mesh.vertex_count() == 0 { None } else { Some(mesh) }
    }

    /// Extract the NAUO instance name (e.g., "nut_1", "bolt_2").
    fn extract_nauo_name(&self, nauo: &crate::schema::StepEntity) -> String {
        // NEXT_ASSEMBLY_USAGE_OCCURRENCE('id','name','description',#relating,#related,$)
        // The name is typically the 2nd parameter
        for (i, param) in nauo.params.iter().enumerate() {
            if i == 0 { continue; } // Skip ID
            if let StepValue::String(s) = param {
                if !s.is_empty() {
                    return s.clone();
                }
            }
        }
        format!("NAUO_{}", nauo.id)
    }

    /// Get a human-readable product name from a PRODUCT_DEFINITION.
    fn get_product_name(&self, pd_id: i64) -> String {
        // PRODUCT_DEFINITION('design','',#product_formation,#context)
        // → #product_formation is PRODUCT_DEFINITION_FORMATION('','',#product)
        // → #product is PRODUCT('id', 'name', ...)
        // The chain is: PD → PDF → PRODUCT → name
        let pd = match self.step.find_entity(pd_id) {
            Some(e) => e,
            None => return format!("PD#{}", pd_id),
        };

        // Search for PRODUCT_DEFINITION_FORMATION reference in PD params
        for param in &pd.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(pdf) = self.step.find_entity(ref_id) {
                    // Direct PRODUCT reference
                    if pdf.type_name == "PRODUCT" {
                        for p in &pdf.params {
                            if let StepValue::String(s) = p {
                                if !s.is_empty() {
                                    return s.clone();
                                }
                            }
                        }
                    }
                    // Follow PD → PDF → PRODUCT chain
                    if pdf.type_name == "PRODUCT_DEFINITION_FORMATION" {
                        for p in &pdf.params {
                            if let Some(product_id) = self.get_ref(p) {
                                if let Some(product) = self.step.find_entity(product_id) {
                                    if product.type_name == "PRODUCT" {
                                        for pp in &product.params {
                                            if let StepValue::String(s) = pp {
                                                if !s.is_empty() {
                                                    return s.clone();
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        format!("PD#{}", pd_id)
    }

    /// Get a name for a BREP from its first parameter.
    fn get_brep_name(&self, brep_id: i64) -> String {
        if let Some(brep) = self.step.find_entity(brep_id) {
            for param in &brep.params {
                if let StepValue::String(s) = param {
                    if !s.is_empty() {
                        return format!("{} (#{})", s, brep_id);
                    }
                }
            }
        }
        format!("BREP#{}", brep_id)
    }

    // ─── Assembly tree traversal ────────────────────────────────────────────

    /// Extract relating and related PRODUCT_DEFINITION IDs from a NAUO entity.
    fn extract_nauo_pd_refs(&self, nauo: &crate::schema::StepEntity) -> (Option<i64>, Option<i64>) {
        let mut pd_refs: Vec<i64> = Vec::new();
        for param in &nauo.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(entity) = self.step.find_entity(ref_id) {
                    if entity.type_name == "PRODUCT_DEFINITION" {
                        pd_refs.push(ref_id);
                    }
                }
            }
        }
        (pd_refs.get(0).copied(), pd_refs.get(1).copied())
    }

    /// Find the transform for a NAUO instance by walking CDSR → SRR → ITEM_DEFINED_TRANSFORMATION.
    fn find_nauo_transform(&self, nauo_id: i64, _related_pd_id: i64) -> Option<[[f64; 4]; 4]> {
        let cdsrs = self.step.find_entities_by_type("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION");
        for cdsr in &cdsrs {
            let linked = self.cdsr_links_to_nauo(cdsr, nauo_id);
            if !linked { continue; }

            if let Some(srr_id) = self.get_ref(cdsr.params.first()?) {
                if let Some(srr_entity) = self.step.find_entity(srr_id) {
                    return self.extract_transform_from_srr(&srr_entity);
                }
            }
        }
        None
    }

    /// Check if a CDSR links to a specific NAUO through PRODUCT_DEFINITION_SHAPE.
    fn cdsr_links_to_nauo(&self, cdsr: &crate::schema::StepEntity, nauo_id: i64) -> bool {
        for (i, param) in cdsr.params.iter().enumerate() {
            if i == 0 { continue; }
            if let Some(pds_id) = self.get_ref(param) {
                if let Some(pds) = self.step.find_entity(pds_id) {
                    for p in &pds.params {
                        if let Some(nid) = self.get_ref(p) {
                            if nid == nauo_id { return true; }
                            if let Some(inner) = self.step.find_entity(nid) {
                                for ip in &inner.params {
                                    if let Some(ref_id) = self.get_ref(ip) {
                                        if ref_id == nauo_id { return true; }
                                    }
                                    if let StepValue::List(items) = ip {
                                        for item in items {
                                            if let Some(ref_id) = self.get_ref(item) {
                                                if ref_id == nauo_id { return true; }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        false
    }

    /// Extract the 4x4 transform from a SHAPE_REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION.
    /// Handles both simple SRR entities and complex/composite entities
    /// (e.g., REPRESENTATION_RELATIONSHIP+REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION+SHAPE_REPRESENTATION_RELATIONSHIP).
    fn extract_transform_from_srr(&self, srr: &crate::schema::StepEntity) -> Option<[[f64; 4]; 4]> {
        // First: check if this is a complex entity with a REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION sub-entity
        if srr.is_complex() {
            if let Some(rrwt_sub) = srr.find_sub_entity("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION") {
                // The RRWT sub-entity has a single parameter: the reference to ITEM_DEFINED_TRANSFORMATION
                for param in &rrwt_sub.params {
                    if let Some(ref_id) = self.get_ref(param) {
                        if let Some(entity) = self.step.find_entity(ref_id) {
                            if entity.type_name == "ITEM_DEFINED_TRANSFORMATION" {
                                return self.compute_item_defined_transform(&entity);
                            }
                        }
                    }
                }
            }
        }

        // Fallback: search all params for direct reference to ITEM_DEFINED_TRANSFORMATION
        for param in &srr.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(entity) = self.step.find_entity(ref_id) {
                    if entity.type_name == "ITEM_DEFINED_TRANSFORMATION" {
                        return self.compute_item_defined_transform(&entity);
                    }
                    // Also search inside nested entities
                    for inner_param in &entity.params {
                        if let Some(inner_id) = self.get_ref(inner_param) {
                            if let Some(inner_entity) = self.step.find_entity(inner_id) {
                                if inner_entity.type_name == "ITEM_DEFINED_TRANSFORMATION" {
                                    return self.compute_item_defined_transform(&inner_entity);
                                }
                            }
                        }
                    }
                }
            }
        }
        None
    }

    /// Compute a 4x4 transform from ITEM_DEFINED_TRANSFORMATION(origin_axis2, target_axis2).
    fn compute_item_defined_transform(&self, idt: &crate::schema::StepEntity) -> Option<[[f64; 4]; 4]> {
        let mut axis2_ids: Vec<i64> = Vec::new();
        for (i, param) in idt.params.iter().enumerate() {
            if i < 2 { continue; }
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(entity) = self.step.find_entity(ref_id) {
                    if entity.type_name == "AXIS2_PLACEMENT_3D" {
                        axis2_ids.push(ref_id);
                    }
                }
            }
        }

        if axis2_ids.len() < 2 {
            warn!("ITEM_DEFINED_TRANSFORMATION has {} axis2 refs (need 2)", axis2_ids.len());
            return None;
        }

        let (origin_pt, origin_z, origin_x) = self.resolve_axis2(axis2_ids[0])?;
        let (target_pt, target_z, target_x) = self.resolve_axis2(axis2_ids[1])?;

        let origin_y = origin_z.cross(&origin_x);
        let target_y = target_z.cross(&target_x);

        let o = [
            [origin_x.x, origin_y.x, origin_z.x, origin_pt.x],
            [origin_x.y, origin_y.y, origin_z.y, origin_pt.y],
            [origin_x.z, origin_y.z, origin_z.z, origin_pt.z],
            [0.0, 0.0, 0.0, 1.0],
        ];

        let t = [
            [target_x.x, target_y.x, target_z.x, target_pt.x],
            [target_x.y, target_y.y, target_z.y, target_pt.y],
            [target_x.z, target_y.z, target_z.z, target_pt.z],
            [0.0, 0.0, 0.0, 1.0],
        ];

        let o_inv = mat4_inverse(&o)?;
        let result = mat4_mul(&t, &o_inv);
        Some(result)
    }

    /// Find the MANIFOLD_SOLID_BREP associated with a PRODUCT_DEFINITION.
    fn find_pd_brep(&self, pd_id: i64) -> Option<i64> {
        let _pd = self.step.find_entity(pd_id)?;

        for pds in self.step.find_entities_by_type("PRODUCT_DEFINITION_SHAPE") {
            let mut refs_our_pd = false;
            for param in &pds.params {
                if let Some(ref_id) = self.get_ref(param) {
                    if ref_id == pd_id { refs_our_pd = true; break; }
                }
            }
            if !refs_our_pd { continue; }

            for sdr in self.step.find_entities_by_type("SHAPE_DEFINITION_REPRESENTATION") {
                let mut refs_our_pds = false;
                for param in &sdr.params {
                    if let Some(ref_id) = self.get_ref(param) {
                        if ref_id == pds.id { refs_our_pds = true; break; }
                    }
                }
                if !refs_our_pds { continue; }

                for param in &sdr.params {
                    if let Some(sr_id) = self.get_ref(param) {
                        // Direct: SR is an ADVANCED_BREP_SHAPE_REPRESENTATION
                        if let Some(brep_id) = self.find_brep_in_representation(sr_id) {
                            return Some(brep_id);
                        }
                        // Indirect: SR is a SHAPE_REPRESENTATION linked to ABSR via SRR
                        if let Some(brep_id) = self.find_brep_via_srr(sr_id) {
                            return Some(brep_id);
                        }
                    }
                }
            }
        }
        None
    }

    /// Find a MANIFOLD_SOLID_BREP inside a SHAPE_REPRESENTATION or ADVANCED_BREP_SHAPE_REPRESENTATION.
    fn find_brep_in_representation(&self, sr_id: i64) -> Option<i64> {
        let sr = self.step.find_entity(sr_id)?;
        if sr.type_name.contains("ADVANCED_BREP_SHAPE_REPRESENTATION") {
            for sp in &sr.params {
                if let Some(brep_id) = self.get_ref(sp) {
                    if let Some(brep) = self.step.find_entity(brep_id) {
                        if brep.type_name == "MANIFOLD_SOLID_BREP" {
                            return Some(brep_id);
                        }
                    }
                }
                if let StepValue::List(items) = sp {
                    for item in items {
                        if let Some(brep_id) = self.get_ref(item) {
                            if let Some(brep) = self.step.find_entity(brep_id) {
                                if brep.type_name == "MANIFOLD_SOLID_BREP" {
                                    return Some(brep_id);
                                }
                            }
                        }
                    }
                }
            }
        }
        // Also check FACETED_BREP
        if sr.type_name.contains("FACETED_BREP_SHAPE_REPRESENTATION") {
            for sp in &sr.params {
                if let Some(brep_id) = self.get_ref(sp) {
                    if let Some(brep) = self.step.find_entity(brep_id) {
                        if brep.type_name == "FACETED_BREP" || brep.type_name == "MANIFOLD_SOLID_BREP" {
                            return Some(brep_id);
                        }
                    }
                }
                if let StepValue::List(items) = sp {
                    for item in items {
                        if let Some(brep_id) = self.get_ref(item) {
                            if let Some(brep) = self.step.find_entity(brep_id) {
                                if brep.type_name == "FACETED_BREP" || brep.type_name == "MANIFOLD_SOLID_BREP" {
                                    return Some(brep_id);
                                }
                            }
                        }
                    }
                }
            }
        }
        None
    }

    /// Find a BREP by following SHAPE_REPRESENTATION_RELATIONSHIP links from a SHAPE_REPRESENTATION.
    /// Many STEP files use: SR → SRR → ABSR → BREP
    fn find_brep_via_srr(&self, sr_id: i64) -> Option<i64> {
        let sr = self.step.find_entity(sr_id)?;
        if !sr.type_name.contains("SHAPE_REPRESENTATION") { return None; }

        // Find all SRR entities that reference this SR
        for srr in self.step.find_entities_by_type("SHAPE_REPRESENTATION_RELATIONSHIP") {
            let mut refs_our_sr = false;
            let mut other_sr_id: Option<i64> = None;

            for (i, param) in srr.params.iter().enumerate() {
                if let Some(ref_id) = self.get_ref(param) {
                    if ref_id == sr_id {
                        refs_our_sr = true;
                    } else if i >= 2 {
                        // The first two params are typically strings (name, description)
                        // The 3rd and 4th are the two SR references
                        if let Some(entity) = self.step.find_entity(ref_id) {
                            if entity.type_name.contains("SHAPE_REPRESENTATION") {
                                other_sr_id = Some(ref_id);
                            }
                        }
                    }
                }
            }

            if refs_our_sr {
                if let Some(other_id) = other_sr_id {
                    // Try to find a BREP in the other representation
                    if let Some(brep_id) = self.find_brep_in_representation(other_id) {
                        return Some(brep_id);
                    }
                    // Recurse one level
                    if let Some(brep_id) = self.find_brep_via_srr(other_id) {
                        return Some(brep_id);
                    }
                }
            }
        }

        // Also check complex SRR entities
        for srr in self.step.find_entities_by_type("REPRESENTATION_RELATIONSHIP") {
            let mut refs_our_sr = false;
            let mut other_sr_id: Option<i64> = None;

            for (i, param) in srr.params.iter().enumerate() {
                if let Some(ref_id) = self.get_ref(param) {
                    if ref_id == sr_id {
                        refs_our_sr = true;
                    } else if i >= 2 {
                        if let Some(entity) = self.step.find_entity(ref_id) {
                            if entity.type_name.contains("SHAPE_REPRESENTATION") {
                                other_sr_id = Some(ref_id);
                            }
                        }
                    }
                }
            }

            if refs_our_sr {
                if let Some(other_id) = other_sr_id {
                    if let Some(brep_id) = self.find_brep_in_representation(other_id) {
                        return Some(brep_id);
                    }
                }
            }
        }

        None
    }

    /// Check if an ADVANCED_BREP_SHAPE_REPRESENTATION belongs to a PRODUCT_DEFINITION.
    fn absr_belongs_to_pd(&self, absr: &crate::schema::StepEntity, pd_id: i64) -> bool {
        for sdr in self.step.find_entities_by_type("SHAPE_DEFINITION_REPRESENTATION") {
            let mut refs_absr = false;
            let mut refs_pds_id: Option<i64> = None;
            for (i, param) in sdr.params.iter().enumerate() {
                if let Some(ref_id) = self.get_ref(param) {
                    if ref_id == absr.id { refs_absr = true; }
                    if i == 0 { refs_pds_id = Some(ref_id); }
                }
            }
            if !refs_absr { continue; }
            if let Some(pds_id) = refs_pds_id {
                if let Some(pds) = self.step.find_entity(pds_id) {
                    for param in &pds.params {
                        if let Some(ref_id) = self.get_ref(param) {
                            if ref_id == pd_id { return true; }
                        }
                    }
                }
            }
        }
        false
    }

    /// Find the shell ref given a BREP entity ID.
    fn find_shell_ref_by_brep_id(&self, brep_id: i64) -> Option<i64> {
        let brep = self.step.find_entity(brep_id)?;
        self.find_shell_ref(&brep)
    }

    // ─── Color extraction ───────────────────────────────────────────────────

    /// Build a map from BREP entity ID → RGBA color from STYLED_ITEM chain.
    fn extract_color_map(&self) -> HashMap<i64, [f32; 4]> {
        let mut color_map: HashMap<i64, [f32; 4]> = HashMap::new();

        let styled_items = self.step.find_entities_by_type("STYLED_ITEM");
        for styled in &styled_items {
            let mut item_id: Option<i64> = None;
            let mut style_ids: Vec<i64> = Vec::new();

            for (i, param) in styled.params.iter().enumerate() {
                if i == 0 { continue; }
                if let Some(ref_id) = self.get_ref(param) {
                    if let Some(entity) = self.step.find_entity(ref_id) {
                        if entity.type_name == "MANIFOLD_SOLID_BREP" {
                            item_id = Some(ref_id);
                        } else if entity.type_name == "ADVANCED_BREP_SHAPE_REPRESENTATION" {
                            for p in &entity.params {
                                if let Some(brep_id) = self.get_ref(p) {
                                    if let Some(brep) = self.step.find_entity(brep_id) {
                                        if brep.type_name == "MANIFOLD_SOLID_BREP" {
                                            item_id = Some(brep_id);
                                        }
                                    }
                                }
                            }
                        } else if entity.type_name == "PRESENTATION_STYLE_ASSIGNMENT" {
                            style_ids.push(ref_id);
                        }
                    }
                }
                if let StepValue::List(items) = param {
                    for item in items {
                        if let Some(ref_id) = self.get_ref(item) {
                            if let Some(entity) = self.step.find_entity(ref_id) {
                                if entity.type_name == "PRESENTATION_STYLE_ASSIGNMENT" {
                                    style_ids.push(ref_id);
                                }
                            }
                        }
                    }
                }
            }

            if item_id.is_none() {
                for param in styled.params.iter().rev() {
                    if let Some(ref_id) = self.get_ref(param) {
                        if let Some(entity) = self.step.find_entity(ref_id) {
                            if entity.type_name == "MANIFOLD_SOLID_BREP" {
                                item_id = Some(ref_id);
                                break;
                            }
                        }
                    }
                }
            }

            let color = self.resolve_color_from_styles(&style_ids);
            if let (Some(brep_id), Some(col)) = (item_id, color) {
                color_map.insert(brep_id, col);
            }
        }

        if !color_map.is_empty() {
            info!("Extracted {} colors from STYLED_ITEMs", color_map.len());
        }
        color_map
    }

    /// Resolve color from PRESENTATION_STYLE_ASSIGNMENT chain.
    fn resolve_color_from_styles(&self, style_ids: &[i64]) -> Option<[f32; 4]> {
        for style_id in style_ids {
            if let Some(psa) = self.step.find_entity(*style_id) {
                if psa.type_name != "PRESENTATION_STYLE_ASSIGNMENT" { continue; }
                for param in &psa.params {
                    if let StepValue::List(items) = param {
                        for item in items {
                            if let Some(ref_id) = self.get_ref(item) {
                                if let Some(color) = self.walk_style_chain(ref_id) {
                                    return Some(color);
                                }
                            }
                        }
                    }
                    if let Some(ref_id) = self.get_ref(param) {
                        if let Some(color) = self.walk_style_chain(ref_id) {
                            return Some(color);
                        }
                    }
                }
            }
        }
        None
    }

    /// Walk the style chain from SURFACE_STYLE_USAGE down to COLOUR_RGB.
    fn walk_style_chain(&self, entity_id: i64) -> Option<[f32; 4]> {
        let entity = self.step.find_entity(entity_id)?;

        match entity.type_name.as_str() {
            "SURFACE_STYLE_USAGE" | "SURFACE_SIDE_STYLE" | "SURFACE_STYLE_FILL_AREA" | "FILL_AREA_STYLE" | "FILL_AREA_STYLE_COLOUR" => {
                for param in &entity.params {
                    if let Some(ref_id) = self.get_ref(param) {
                        if let Some(color) = self.walk_style_chain(ref_id) {
                            return Some(color);
                        }
                    }
                    if let StepValue::List(items) = param {
                        for item in items {
                            if let Some(ref_id) = self.get_ref(item) {
                                if let Some(color) = self.walk_style_chain(ref_id) {
                                    return Some(color);
                                }
                            }
                        }
                    }
                }
                None
            }
            "COLOUR_RGB" => {
                let mut rgb = [0.5f32, 0.5, 0.5];
                let mut idx = 0;
                for param in &entity.params {
                    if let Some(f) = self.get_float(param) {
                        if idx < 3 {
                            rgb[idx] = f as f32;
                            idx += 1;
                        }
                    }
                }
                Some([rgb[0], rgb[1], rgb[2], 1.0])
            }
            "DRAUGHTING_PRE_DEFINED_COLOUR" => {
                // Named colors like 'red', 'green', 'blue', etc.
                for param in &entity.params {
                    if let StepValue::String(name) = param {
                        return Some(resolve_predefined_colour(&name));
                    }
                    if let StepValue::Enum(name) = param {
                        return Some(resolve_predefined_colour(&name));
                    }
                }
                None
            }
            _ => {
                // For unknown types, try to walk deeper
                for param in &entity.params {
                    if let Some(ref_id) = self.get_ref(param) {
                        if let Some(color) = self.walk_style_chain(ref_id) {
                            return Some(color);
                        }
                    }
                    if let StepValue::List(items) = param {
                        for item in items {
                            if let Some(ref_id) = self.get_ref(item) {
                                if let Some(color) = self.walk_style_chain(ref_id) {
                                    return Some(color);
                                }
                            }
                        }
                    }
                }
                None
            }
        }
    }

    /// Find the shell reference from a BREP entity.
    fn find_shell_ref(&self, brep: &crate::schema::StepEntity) -> Option<i64> {
        // MANIFOLD_SOLID_BREP('name', #shell_ref) — shell ref is usually 2nd param
        // But some files have it as the first Ref parameter
        for param in &brep.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(entity) = self.step.find_entity(ref_id) {
                    if entity.type_name.contains("SHELL") {
                        return Some(ref_id);
                    }
                }
            }
        }
        // If not found by type name, try the second parameter
        if let Some(param) = brep.params.get(1) {
            if let Some(ref_id) = self.get_ref(param) {
                return Some(ref_id);
            }
        }
        None
    }

    /// Compute a bounding box from all CARTESIAN_POINT entities.
    fn compute_bounding_box(&self) -> Option<(Point3d, Point3d)> {
        let points: Vec<Point3d> = self.step.find_entities_by_type("CARTESIAN_POINT")
            .iter()
            .filter_map(|e| self.resolve_cartesian_point(e.id))
            .collect();

        if points.is_empty() {
            return None;
        }

        let mut min = points[0];
        let mut max = points[0];
        for p in &points[1..] {
            min.x = min.x.min(p.x);
            min.y = min.y.min(p.y);
            min.z = min.z.min(p.z);
            max.x = max.x.max(p.x);
            max.y = max.y.max(p.y);
            max.z = max.z.max(p.z);
        }

        // Expand the box slightly
        let margin = 0.001;
        min.x -= margin; min.y -= margin; min.z -= margin;
        max.x += margin; max.y += margin; max.z += margin;

        Some((min, max))
    }

    /// Extract FaceData (surface + boundary edges) from a CLOSED_SHELL or OPEN_SHELL entity.
    fn extract_shell_faces(&self, shell_id: i64) -> Option<Vec<FaceData>> {
        let shell = self.step.find_entity(shell_id)?;
        let mut face_data_list = Vec::new();

        // CLOSED_SHELL('', (#face1, #face2, ...))
        for param in &shell.params {
            match param {
                StepValue::List(items) => {
                    for item in items {
                        if let Some(face_id) = self.get_ref(item) {
                            if let Some(face_data) = self.extract_face_data(face_id) {
                                face_data_list.push(face_data);
                            }
                        }
                    }
                }
                StepValue::Ref(face_id) => {
                    if let Some(face_data) = self.extract_face_data(*face_id) {
                        face_data_list.push(face_data);
                    }
                }
                _ => {}
            }
        }

        if face_data_list.is_empty() { None } else { Some(face_data_list) }
    }

    /// Extract both surface geometry and boundary edges from an ADVANCED_FACE or FACE_SURFACE entity.
    fn extract_face_data(&self, face_id: i64) -> Option<FaceData> {
        let face_entity = self.step.find_entity(face_id)?;

        match face_entity.type_name.as_str() {
            "ADVANCED_FACE" | "FACE_SURFACE" => {
                // Format: #N = ADVANCED_FACE('', (bounds), #surface_ref, .T.);
                // params: [name, bounds_list, surface_ref, orientation]

                // Extract surface
                let surface = self.extract_face_surface_from_entity(face_entity)?;
                
                // Extract boundary edges with inner/outer distinction
                let (outer_edges, inner_edges) = self.extract_face_bounds_separated(face_entity);

                // All edges combined for backward compat
                let mut all_edges = outer_edges.clone();
                for inner in &inner_edges {
                    all_edges.extend(inner.clone());
                }

                // Extract face orientation (last param, typically .T. or .F.)
                let forward = self.extract_face_orientation(face_entity);

                Some(FaceData {
                    surface,
                    outer_edges,
                    inner_edges,
                    edges: all_edges,
                    forward,
                })
            }
            _ => {
                // Try to extract directly as a surface (no boundary info)
                if let Some(surface) = self.extract_surface(face_id) {
                    Some(FaceData {
                        surface,
                        outer_edges: vec![],
                        inner_edges: vec![],
                        edges: vec![],
                        forward: true,
                    })
                } else {
                    None
                }
            }
        }
    }

    /// Extract the surface geometry from an ADVANCED_FACE or FACE_SURFACE entity.
    /// This is the surface-only extraction logic (previously extract_face_surface).
    fn extract_face_surface_from_entity(&self, face: &crate::schema::StepEntity) -> Option<Surface> {
        // Format: #N = ADVANCED_FACE('', (bounds), #surface_ref, .T.);
        // The surface reference is typically the 3rd parameter (index 2).
        // But bounds can be complex (lists of lists), so we need to be smart.

        // Try parameter index 2 first (the typical position for surface ref)
        if let Some(param) = face.params.get(2) {
            if let Some(surface_id) = self.get_ref(param) {
                if let Some(surface) = self.extract_surface(surface_id) {
                    return Some(surface);
                }
            }
        }

        // If index 2 didn't work, scan all params for the surface ref
        // Skip the first param (usually a string name)
        for (i, param) in face.params.iter().enumerate() {
            if i == 0 { continue; } // Skip name
            if let Some(surface_id) = self.get_ref(param) {
                // Check if this ref points to a surface entity (not a bound)
                if let Some(entity) = self.step.find_entity(surface_id) {
                    let tn = entity.type_name.as_str();
                    let is_surface = matches!(
                        tn,
                        "PLANE" | "CYLINDRICAL_SURFACE" | "SPHERICAL_SURFACE" |
                        "CONICAL_SURFACE" | "TOROIDAL_SURFACE" |
                        "SURFACE_OF_REVOLUTION" | "SURFACE_OF_LINEAR_EXTRUSION" |
                        "B_SPLINE_SURFACE_WITH_KNOTS" | "B_SPLINE_SURFACE" |
                        "BEZIER_SURFACE" | "RECTANGULAR_TRIMMED_SURFACE" |
                        "OFFSET_SURFACE" | "SWEPT_SURFACE"
                    ) || tn.contains("B_SPLINE_SURFACE") // Handle complex entities like "BOUNDED_SURFACE+B_SPLINE_SURFACE+..."
                    || tn.contains("SURFACE") && !tn.contains("CURVE"); // Handle other complex surface entities
                    if is_surface {
                        if let Some(surface) = self.extract_surface(surface_id) {
                            return Some(surface);
                        }
                    }
                }
            }
        }

        None
    }

    /// Extract the face orientation from an ADVANCED_FACE entity.
    /// The orientation is the last parameter, typically .T. or .F.
    fn extract_face_orientation(&self, face: &crate::schema::StepEntity) -> bool {
        if let Some(last_param) = face.params.last() {
            match last_param {
                StepValue::Enum(e) => return e == "T",
                StepValue::Float(f) => return *f != 0.0,
                StepValue::Integer(i) => return *i != 0,
                _ => {}
            }
        }
        // Default to true if not found
        true
    }

    /// Extract boundary edges from an ADVANCED_FACE entity.
    /// Traverses: ADVANCED_FACE → bounds_list → FACE_BOUND/FACE_OUTER_BOUND →
    ///            EDGE_LOOP → ORIENTED_EDGE → EDGE_CURVE → curve + vertices
    fn extract_face_bounds(&self, face: &crate::schema::StepEntity) -> Vec<TopoEdge> {
        let (outer, inner) = self.extract_face_bounds_separated(face);
        let mut all_edges = outer;
        for loop_edges in inner {
            all_edges.extend(loop_edges);
        }
        all_edges
    }

    /// Extract boundary edges from an ADVANCED_FACE entity, separating outer and inner loops.
    /// FACE_OUTER_BOUND → outer loop (the main boundary)
    /// FACE_BOUND → inner loop (a hole)
    /// Returns (outer_edges, inner_loops) where inner_loops is a Vec of edge loops.
    fn extract_face_bounds_separated(&self, face: &crate::schema::StepEntity) -> (Vec<TopoEdge>, Vec<Vec<TopoEdge>>) {
        let mut outer_edges: Vec<TopoEdge> = Vec::new();
        let mut inner_loops: Vec<Vec<TopoEdge>> = Vec::new();

        // ADVANCED_FACE params: [name, (bounds_list), surface_ref, orientation]
        // The bounds are in params[1], which is a List of references to FACE_BOUND/FACE_OUTER_BOUND
        for param in &face.params {
            // Look for the bounds list — it's a StepValue::List containing references
            if let StepValue::List(items) = param {
                let mut found_bound = false;
                for item in items {
                    if let Some(bound_id) = self.get_ref(item) {
                        if let Some(bound_entity) = self.step.find_entity(bound_id) {
                            if bound_entity.type_name == "FACE_OUTER_BOUND" {
                                found_bound = true;
                                if let Some(loop_edges) = self.resolve_face_bound(bound_entity) {
                                    outer_edges = loop_edges;
                                }
                            } else if bound_entity.type_name == "FACE_BOUND" {
                                found_bound = true;
                                if let Some(loop_edges) = self.resolve_face_bound(bound_entity) {
                                    inner_loops.push(loop_edges);
                                }
                            }
                        }
                    }
                }
                // If we found bounds in this list, don't process it again
                if found_bound {
                    // CRITICAL: If no FACE_OUTER_BOUND was found but FACE_BOUND entries exist,
                    // the first FACE_BOUND is the outer boundary (not a hole).
                    // Many STEP files use only FACE_BOUND (no FACE_OUTER_BOUND at all).
                    if outer_edges.is_empty() && !inner_loops.is_empty() {
                        outer_edges = inner_loops.remove(0);
                    }
                    return (outer_edges, inner_loops);
                }
            }
        }

        // Fallback: if no FACE_OUTER_BOUND found but FACE_BOUND exists,
        // treat the first FACE_BOUND as the outer boundary
        // (some STEP files use FACE_BOUND for both outer and inner)
        if outer_edges.is_empty() && !inner_loops.is_empty() {
            outer_edges = inner_loops.remove(0);
        }

        (outer_edges, inner_loops)
    }

    /// Resolve a FACE_BOUND or FACE_OUTER_BOUND entity to a list of Edge objects.
    /// FACE_BOUND params: [name, loop_ref, orientation]
    fn resolve_face_bound(&self, bound_entity: &crate::schema::StepEntity) -> Option<Vec<TopoEdge>> {
        // FACE_BOUND('', #loop_ref, .T.)
        // The loop reference is typically the 2nd parameter (index 1)
        for (i, param) in bound_entity.params.iter().enumerate() {
            if i == 0 { continue; } // Skip name
            if let Some(loop_id) = self.get_ref(param) {
                if let Some(loop_entity) = self.step.find_entity(loop_id) {
                    if loop_entity.type_name == "EDGE_LOOP" {
                        return Some(self.resolve_edge_loop(loop_id));
                    }
                }
            }
        }
        None
    }

    /// Resolve an EDGE_LOOP entity to a list of Edge objects.
    /// EDGE_LOOP params: [name, (oriented_edge_refs)]
    fn resolve_edge_loop(&self, loop_id: i64) -> Vec<TopoEdge> {
        let loop_entity = match self.step.find_entity(loop_id) {
            Some(e) => e,
            None => return vec![],
        };

        let mut edges = Vec::new();

        // EDGE_LOOP('', (#oe1, #oe2, ...))
        for param in &loop_entity.params {
            if let StepValue::List(items) = param {
                for item in items {
                    if let Some(oe_id) = self.get_ref(item) {
                        if let Some(edge) = self.resolve_oriented_edge(oe_id) {
                            edges.push(edge);
                        }
                    }
                }
            }
        }

        edges
    }

    /// Resolve an ORIENTED_EDGE entity to an Edge object.
    /// ORIENTED_EDGE params: [name, *, *, edge_curve_ref, orientation]
    fn resolve_oriented_edge(&self, oe_id: i64) -> Option<TopoEdge> {
        let oe_entity = self.step.find_entity(oe_id)?;

        // ORIENTED_EDGE('', *, *, #edge_curve_ref, .T./.F.)
        // The edge_curve_ref is typically the 4th parameter (index 3)
        // The orientation is typically the 5th parameter (index 4)
        let mut edge_curve_id: Option<i64> = None;
        let mut orientation = true;

        // Find the edge curve reference and orientation
        for (_i, param) in oe_entity.params.iter().enumerate() {
            if let Some(ref_id) = self.get_ref(param) {
                // Check if this reference points to an EDGE_CURVE
                if let Some(entity) = self.step.find_entity(ref_id) {
                    if entity.type_name == "EDGE_CURVE" {
                        edge_curve_id = Some(ref_id);
                    }
                }
            }
            // Check for orientation enum
            if let StepValue::Enum(e) = param {
                orientation = e == "T";
            }
        }

        let edge_curve_id = edge_curve_id?;
        let mut edge = self.resolve_edge_curve(edge_curve_id)?;

        // If the oriented edge is reversed relative to the edge curve, reverse it
        if !orientation {
            edge = edge.reversed();
        }

        Some(edge)
    }

    /// Resolve an EDGE_CURVE entity to an Edge object.
    /// EDGE_CURVE params: [name, vertex1_ref, vertex2_ref, curve_ref, orientation]
    /// Some files omit the name: EDGE_CURVE(#v1, #v2, #curve, .T.)
    /// We handle both cases by scanning parameters for their entity types.
    fn resolve_edge_curve(&self, edge_curve_id: i64) -> Option<TopoEdge> {
        let ec_entity = self.step.find_entity(edge_curve_id)?;

        let mut vertex_ids: Vec<i64> = Vec::new();
        let mut curve_ref_id: Option<i64> = None;

        for param in &ec_entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(entity) = self.step.find_entity(ref_id) {
                    if entity.type_name == "VERTEX_POINT" {
                        vertex_ids.push(ref_id);
                    } else if entity.type_name == "EDGE_CURVE" {
                        // Nested edge curve (shouldn't happen, but handle gracefully)
                    } else if self.is_curve_type(&entity.type_name)
                        || entity.type_name == "SURFACE_CURVE"
                    {
                        curve_ref_id = Some(ref_id);
                    }
                }
            }
        }

        // Resolve vertex points
        let p1 = vertex_ids.get(0).and_then(|id| self.resolve_vertex_point(*id));
        let p2 = vertex_ids.get(1).and_then(|id| self.resolve_vertex_point(*id));

        // If we have both vertex points but no curve ref, create a line edge
        if curve_ref_id.is_none() {
            if let (Some(p1), Some(p2)) = (&p1, &p2) {
                return Some(TopoEdge::new_line(*p1, *p2));
            }
            return None;
        }

        let curve_ref_id = curve_ref_id.unwrap();

        // Resolve the 3D curve (possibly through SURFACE_CURVE)
        let resolved_curve_id = self.resolve_3d_curve_ref(curve_ref_id);
        let curve = match resolved_curve_id {
            Some(id) => self.resolve_curve(id),
            None => self.resolve_curve(curve_ref_id),
        };

        match (curve, &p1, &p2) {
            (Some(curve), Some(p1), Some(p2)) => {
                // We have both curve and vertex points — create edge with vertex info
                // Use vertex points to determine param_range for the curve
                let edge = if let Curve3d::Line(ref line) = curve {
                    // For lines, compute param range from vertex projections
                    let t1 = project_point_on_line(line, p1);
                    let t2 = project_point_on_line(line, p2);
                    let mut edge = TopoEdge::new(curve, (t1, t2));
                    edge.vertex_start = Some(draper_topology::TopoId::new());
                    edge.vertex_end = Some(draper_topology::TopoId::new());
                    edge
                } else if let Curve3d::Circle(ref circle) = curve {
                    // For circles, compute angular range from vertex projections
                    let (t1, t2) = project_points_on_circle(circle, p1, p2);
                    let mut edge = TopoEdge::new(curve, (t1, t2));
                    edge.vertex_start = Some(draper_topology::TopoId::new());
                    edge.vertex_end = Some(draper_topology::TopoId::new());
                    edge
                } else {
                    // For other curves, use the default param range
                    let param_range = curve.param_range();
                    let mut edge = TopoEdge::new(curve, param_range);
                    edge.vertex_start = Some(draper_topology::TopoId::new());
                    edge.vertex_end = Some(draper_topology::TopoId::new());
                    edge
                };
                Some(edge)
            }
            (Some(curve), _, _) => {
                // Curve but missing vertex points — use default param range
                let param_range = curve.param_range();
                Some(TopoEdge::new(curve, param_range))
            }
            (None, Some(p1), Some(p2)) => {
                // No curve but have vertex points — create a line edge
                Some(TopoEdge::new_line(*p1, *p2))
            }
            _ => None,
        }
    }

    /// Resolve a SURFACE_CURVE entity to get the 3D curve reference.
    /// SURFACE_CURVE params: [name, curve3d_ref, (pcurve_refs), .PCURVE_S1.]
    fn resolve_3d_curve_ref(&self, surface_curve_id: i64) -> Option<i64> {
        let sc_entity = self.step.find_entity(surface_curve_id)?;
        
        if sc_entity.type_name != "SURFACE_CURVE" {
            return Some(surface_curve_id); // Not a surface curve, return as-is
        }

        // SURFACE_CURVE('', #curve3d_ref, (#pcurve1, #pcurve2), .PCURVE_S1.)
        // The 3D curve is the 2nd parameter (index 1)
        if let Some(param) = sc_entity.params.get(1) {
            if let Some(curve3d_id) = self.get_ref(param) {
                if let Some(curve_entity) = self.step.find_entity(curve3d_id) {
                    if self.is_curve_type(&curve_entity.type_name) {
                        return Some(curve3d_id);
                    }
                }
            }
        }

        // Fallback: search all params for a curve reference
        for param in &sc_entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(entity) = self.step.find_entity(ref_id) {
                    if self.is_curve_type(&entity.type_name) {
                        return Some(ref_id);
                    }
                }
            }
            // Also check inside lists (pcurve refs might be in a list)
            if let StepValue::List(items) = param {
                for item in items {
                    if let Some(ref_id) = self.get_ref(item) {
                        if let Some(entity) = self.step.find_entity(ref_id) {
                            if self.is_curve_type(&entity.type_name) {
                                return Some(ref_id);
                            }
                        }
                    }
                }
            }
        }

        None
    }

    /// Resolve a VERTEX_POINT entity to a 3D point.
    /// VERTEX_POINT params: [name, point_ref]
    fn resolve_vertex_point(&self, vertex_id: i64) -> Option<Point3d> {
        let vertex_entity = self.step.find_entity(vertex_id)?;
        
        if vertex_entity.type_name != "VERTEX_POINT" {
            return None;
        }

        // VERTEX_POINT('', #point_ref)
        for param in &vertex_entity.params {
            if let Some(point_id) = self.get_ref(param) {
                if let Some(point) = self.resolve_cartesian_point(point_id) {
                    return Some(point);
                }
            }
        }

        None
    }

    /// Extract a Surface from a STEP surface entity.
    fn extract_surface(&self, surface_id: i64) -> Option<Surface> {
        let entity = self.step.find_entity(surface_id)?;
        let type_name = entity.type_name.as_str();

        // Handle complex entity types (e.g., "BOUNDED_SURFACE+B_SPLINE_SURFACE+B_SPLINE_SURFACE_WITH_KNOTS+RATIONAL_B_SPLINE_SURFACE+GEOMETRIC_REPRESENTATION_ITEM+REPRESENTATION_ITEM+SURFACE")
        // by checking if the type_name contains known surface type keywords
        if type_name.contains("B_SPLINE_SURFACE") {
            return self.extract_bspline_surface(entity);
        }

        match type_name {
            "PLANE" => self.extract_plane(entity),
            "CYLINDRICAL_SURFACE" => self.extract_cylinder(entity),
            "SPHERICAL_SURFACE" => self.extract_sphere(entity),
            "CONICAL_SURFACE" => self.extract_cone(entity),
            "TOROIDAL_SURFACE" => self.extract_torus(entity),
            "SURFACE_OF_REVOLUTION" => self.extract_revolution(entity),
            "SURFACE_OF_LINEAR_EXTRUSION" => self.extract_extrusion(entity),
            "B_SPLINE_SURFACE_WITH_KNOTS" | "B_SPLINE_SURFACE" | "BEZIER_SURFACE" => {
                self.extract_bspline_surface(entity)
            }
            "RECTANGULAR_TRIMMED_SURFACE" => self.extract_trimmed_surface(entity),
            "SWEPT_SURFACE" => self.extract_swept_surface(entity),
            _ => None,
        }
    }

    /// Extract a PLANE surface.
    fn extract_plane(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        let axis2_id = self.find_axis2_ref(entity)?;
        let (origin, normal, u_dir) = self.resolve_axis2(axis2_id)?;
        let v_dir = normal.cross(&u_dir);
        Some(Surface::Plane(Plane { origin, u_dir, v_dir, normal }))
    }

    /// Extract a CYLINDRICAL_SURFACE.
    fn extract_cylinder(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        let axis2_id = self.find_axis2_ref(entity)?;
        let (origin, axis, u_dir) = self.resolve_axis2(axis2_id)?;
        let radius = self.find_float_param(entity, 0)?;
        Some(Surface::Cylinder(CylinderSurface::new_with_frame(origin, axis, radius, u_dir)))
    }

    /// Extract a SPHERICAL_SURFACE.
    fn extract_sphere(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        let axis2_id = self.find_axis2_ref(entity)?;
        let (center, _axis, _u_dir) = self.resolve_axis2(axis2_id)?;
        let radius = self.find_float_param(entity, 0)?;
        Some(Surface::Sphere(SphereSurface::new(center, radius)))
    }

    /// Extract a CONICAL_SURFACE.
    fn extract_cone(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        let axis2_id = self.find_axis2_ref(entity)?;
        let (origin, axis, u_dir) = self.resolve_axis2(axis2_id)?;
        let radius = self.find_float_param(entity, 0)?;
        let half_angle = self.find_float_param(entity, 1)?;
        Some(Surface::Cone(ConeSurface::new_with_frame(origin, axis, radius, half_angle.abs(), u_dir)))
    }

    /// Extract a TOROIDAL_SURFACE.
    fn extract_torus(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        let axis2_id = self.find_axis2_ref(entity)?;
        let (center, axis, u_dir) = self.resolve_axis2(axis2_id)?;
        let major_radius = self.find_float_param(entity, 0)?;
        let minor_radius = self.find_float_param(entity, 1)?;
        Some(Surface::Torus(TorusSurface::new_with_frame(center, axis, major_radius, minor_radius, u_dir)))
    }

    /// Extract a SURFACE_OF_REVOLUTION.
    /// Format: #N = SURFACE_OF_REVOLUTION('', #profile_curve, #axis2_placement);
    fn extract_revolution(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        // Find the profile curve (2nd param, index 1)
        let profile_id = self.find_curve_ref(entity, 1)?;
        let profile = self.resolve_curve(profile_id)?;

        // Find the axis placement (3rd param, index 2)
        let axis2_id = self.find_param_ref(entity, 2)?;
        let (origin, axis, _u_dir) = self.resolve_axis2(axis2_id)?;

        Some(Surface::Revolution(RevolutionSurface {
            profile,
            axis,
            origin,
        }))
    }

    /// Extract a SURFACE_OF_LINEAR_EXTRUSION.
    /// Format: #N = SURFACE_OF_LINEAR_EXTRUSION('', #profile_curve, #direction_or_vector);
    /// The 3rd param can be a DIRECTION or a VECTOR(#direction, magnitude).
    fn extract_extrusion(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        // Find the profile curve
        let profile_id = self.find_curve_ref(entity, 1)?;
        let profile = self.resolve_curve(profile_id)?;

        // Find the extrusion direction (3rd param, index 2)
        // Can be a DIRECTION or a VECTOR(#direction, magnitude)
        let dir_id = self.find_param_ref(entity, 2)?;
        let direction = if let Some(dir_entity) = self.step.find_entity(dir_id) {
            if dir_entity.type_name == "DIRECTION" {
                self.resolve_direction(dir_id)?
            } else if dir_entity.type_name == "VECTOR" {
                // VECTOR('', #direction, magnitude) — extract direction from it
                let inner_dir_id = self.find_direction_from_vector(dir_entity)?;
                self.resolve_direction(inner_dir_id)?
            } else {
                // Try to find a direction within the referenced entity
                self.resolve_direction(dir_id)?
            }
        } else {
            self.resolve_direction(dir_id)?
        };

        Some(Surface::Extrusion(ExtrusionSurface {
            profile,
            direction,
        }))
    }

    /// Extract a B_SPLINE_SURFACE_WITH_KNOTS.
    /// Format: #N = B_SPLINE_SURFACE_WITH_KNOTS(degree_u, degree_v,
    ///   ((cp_list_row1), (cp_list_row2), ...), .UNSPECIFIED., .F., .F., .F.,
    ///   knot_count_u, knot_count_v, (knots_u), (knots_v), .UNSPECIFIED.);
    fn extract_bspline_surface(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        // For complex entities, find the B_SPLINE_SURFACE sub-entity for control points
        // and B_SPLINE_SURFACE_WITH_KNOTS sub-entity for knot vectors
        // and RATIONAL_B_SPLINE_SURFACE sub-entity for weights
        let bspline_sub = entity.find_sub_entity("B_SPLINE_SURFACE");
        let knots_sub = entity.find_sub_entity("B_SPLINE_SURFACE_WITH_KNOTS");
        let rational_sub = entity.find_sub_entity("RATIONAL_B_SPLINE_SURFACE");

        // Use the B_SPLINE_SURFACE sub-entity if available, otherwise use the entity itself
        let cp_entity = bspline_sub.unwrap_or(entity);
        let knot_entity = knots_sub.unwrap_or(entity);

        // Degree u and v — use find_float_param to handle name prefix
        let u_degree = self.find_float_param(cp_entity, 0).unwrap_or(1.0) as usize;
        let v_degree = self.find_float_param(cp_entity, 1).unwrap_or(1.0) as usize;

        // Control points: search params for a list-of-lists that contains the control points
        let mut control_points: Vec<Vec<Point3d>> = Vec::new();
        for param in &cp_entity.params {
            if let StepValue::List(rows) = param {
                // Check if this is a list of lists (i.e., a control point grid)
                let mut is_cp_grid = false;
                for row in rows {
                    if let StepValue::List(_) = row {
                        is_cp_grid = true;
                        break;
                    }
                }
                if !is_cp_grid {
                    continue;
                }

                for row in rows {
                    if let StepValue::List(cols) = row {
                        let mut row_pts = Vec::new();
                        for col in cols {
                            if let StepValue::List(coords) = col {
                                // Inline [x, y, z]
                                let x = coords.get(0).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                                let y = coords.get(1).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                                let z = coords.get(2).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                                row_pts.push(Point3d::new(x, y, z));
                            } else if let Some(ref_id) = self.get_ref(col) {
                                // Reference to CARTESIAN_POINT
                                if let Some(pt) = self.resolve_cartesian_point(ref_id) {
                                    row_pts.push(pt);
                                }
                            }
                        }
                        if !row_pts.is_empty() {
                            control_points.push(row_pts);
                        }
                    }
                }
                // Only process the first list-of-lists found (the control point grid)
                break;
            }
        }

        if control_points.is_empty() {
            return None;
        }

        let n_u = control_points.len();
        let n_v = control_points[0].len();

        // Extract weights from RATIONAL_B_SPLINE_SURFACE sub-entity if present
        let weights = if let Some(rational_ent) = rational_sub {
            self.extract_rational_weights(rational_ent, n_u, n_v)
        } else {
            vec![vec![1.0; n_v]; n_u]
        };

        // Find knot vectors — use B_SPLINE_SURFACE_WITH_KNOTS sub-entity if available
        let (u_knots, v_knots) = self.extract_bspline_knots(knot_entity, n_u, n_v, u_degree, v_degree);

        Some(Surface::Nurbs(NurbsSurface {
            u_degree,
            v_degree,
            control_points,
            weights,
            u_knots,
            v_knots,
        }))
    }

    /// Extract weight matrix from a RATIONAL_B_SPLINE_SURFACE sub-entity.
    /// Format: RATIONAL_B_SPLINE_SURFACE(((w11,w12,...),(w21,w22,...),...))
    fn extract_rational_weights(&self, entity: &crate::schema::StepEntity, n_u: usize, n_v: usize) -> Vec<Vec<f64>> {
        for param in &entity.params {
            if let StepValue::List(rows) = param {
                // Check if this is a list of lists of floats (weight matrix)
                let mut is_weight_matrix = false;
                for row in rows {
                    if let StepValue::List(inner) = row {
                        // Check if the inner list contains floats/numbers
                        if !inner.is_empty() && inner.iter().any(|v| matches!(v, StepValue::Float(_) | StepValue::Integer(_))) {
                            is_weight_matrix = true;
                            break;
                        }
                    }
                }

                if is_weight_matrix {
                    let mut weights: Vec<Vec<f64>> = Vec::new();
                    for row in rows {
                        if let StepValue::List(inner) = row {
                            let row_weights: Vec<f64> = inner.iter()
                                .filter_map(|v| self.get_float(v))
                                .collect();
                            if !row_weights.is_empty() {
                                weights.push(row_weights);
                            }
                        }
                    }

                    // Validate dimensions match control points
                    if weights.len() == n_u && weights.iter().all(|r| r.len() == n_v) {
                        return weights;
                    }
                    // If dimensions don't match but we have weights, try to use what we have
                    if !weights.is_empty() {
                        // Resize to match control point dimensions
                        let mut result = vec![vec![1.0; n_v]; n_u];
                        for (i, row) in weights.iter().enumerate() {
                            if i >= n_u { break; }
                            for (j, &w) in row.iter().enumerate() {
                                if j >= n_v { break; }
                                result[i][j] = w;
                            }
                        }
                        return result;
                    }
                }
            }
        }

        // Default to unit weights
        vec![vec![1.0; n_v]; n_u]
    }

    /// Extract knot vectors from a B_SPLINE_SURFACE_WITH_KNOTS entity.
    /// Format: B_SPLINE_SURFACE_WITH_KNOTS(u_mults, v_mults, u_knots, v_knots, knot_type)
    /// The knot values are distinct knots and the multiplicities tell how many times each is repeated.
    /// The full knot vector is: for each distinct knot value, repeat it by its multiplicity.
    fn extract_bspline_knots(
        &self,
        entity: &crate::schema::StepEntity,
        n_u: usize,
        n_v: usize,
        u_degree: usize,
        v_degree: usize,
    ) -> (Vec<f64>, Vec<f64>) {
        let expected_u_knots = n_u + u_degree + 1;
        let expected_v_knots = n_v + v_degree + 1;

        // Strategy 1: Use positional order from B_SPLINE_SURFACE_WITH_KNOTS format.
        // The sub-entity's params are: (u_mults), (v_mults), (u_knot_values), (v_knot_values), knot_type_enum
        // This is the most reliable approach since the STEP format is strictly positional.
        {
            let lists: Vec<&[StepValue]> = entity.params.iter()
                .filter_map(|p| if let StepValue::List(items) = p { Some(items.as_slice()) } else { None })
                .collect();

            if lists.len() >= 4 {
                // First two are multiplicities, next two are knot values
                let u_mults: Vec<usize> = lists[0].iter()
                    .filter_map(|v| self.get_float(v).map(|f| f as usize))
                    .collect();
                let v_mults: Vec<usize> = lists[1].iter()
                    .filter_map(|v| self.get_float(v).map(|f| f as usize))
                    .collect();
                let u_knot_vals: Vec<f64> = lists[2].iter()
                    .filter_map(|v| self.get_float(v))
                    .collect();
                let v_knot_vals: Vec<f64> = lists[3].iter()
                    .filter_map(|v| self.get_float(v))
                    .collect();

                if u_mults.len() == u_knot_vals.len() && v_mults.len() == v_knot_vals.len() {
                    let u_knots = expand_knot_vector(&u_mults, &u_knot_vals);
                    let v_knots = expand_knot_vector(&v_mults, &v_knot_vals);

                    if u_knots.len() == expected_u_knots && v_knots.len() == expected_v_knots {
                        return (u_knots, v_knots);
                    }
                }

                // Try swapped: maybe the entity has a different param layout
                // Some STEP writers put knot values before multiplicities
                if lists.len() >= 4 {
                    let u_knot_vals2: Vec<f64> = lists[0].iter()
                        .filter_map(|v| self.get_float(v))
                        .collect();
                    let v_knot_vals2: Vec<f64> = lists[1].iter()
                        .filter_map(|v| self.get_float(v))
                        .collect();
                    let u_mults2: Vec<usize> = lists[2].iter()
                        .filter_map(|v| self.get_float(v).map(|f| f as usize))
                        .collect();
                    let v_mults2: Vec<usize> = lists[3].iter()
                        .filter_map(|v| self.get_float(v).map(|f| f as usize))
                        .collect();

                    if u_mults2.len() == u_knot_vals2.len() && v_mults2.len() == v_knot_vals2.len() {
                        let u_knots = expand_knot_vector(&u_mults2, &u_knot_vals2);
                        let v_knots = expand_knot_vector(&v_mults2, &v_knot_vals2);

                        if u_knots.len() == expected_u_knots && v_knots.len() == expected_v_knots {
                            return (u_knots, v_knots);
                        }
                    }
                }
            }
        }

        // Strategy 2: Heuristic approach — collect all numeric lists and try to distinguish
        // multiplicities from knot values based on their properties.
        let mut numeric_lists: Vec<Vec<f64>> = Vec::new();
        let mut int_lists: Vec<Vec<usize>> = Vec::new();

        for param in entity.params.iter() {
            if let StepValue::List(items) = param {
                let ints: Vec<usize> = items.iter()
                    .filter_map(|v| {
                        if let StepValue::Integer(i) = v { Some(*i as usize) }
                        else if let StepValue::Float(f) = v { Some(*f as usize) }
                        else { None }
                    })
                    .collect();

                let floats: Vec<f64> = items.iter()
                    .filter_map(|v| self.get_float(v))
                    .collect();

                if floats.len() >= 2 && floats.iter().all(|f| f.is_finite()) {
                    numeric_lists.push(floats);
                }
                if ints.len() >= 2 {
                    int_lists.push(ints);
                }
            }
        }

        // Separate into multiplicities and knot values based on value properties
        let max_mult = (n_u + n_v + u_degree + v_degree + 10).max(20);
        let mut mult_lists: Vec<Vec<usize>> = Vec::new();
        let mut knot_value_lists: Vec<Vec<f64>> = Vec::new();

        for ints in &int_lists {
            if ints.iter().all(|v| *v > 0 && *v <= max_mult) {
                let sum: usize = ints.iter().sum();
                if sum == expected_u_knots || sum == expected_v_knots {
                    mult_lists.push(ints.clone());
                }
            }
        }

        for floats in &numeric_lists {
            if floats.windows(2).all(|w| w[0] <= w[1] + 1e-10) {
                knot_value_lists.push(floats.clone());
            }
        }

        // Try to pair multiplicities with knot values
        if mult_lists.len() >= 2 && knot_value_lists.len() >= 2 {
            let u_mult_idx = mult_lists.iter().position(|m| m.iter().sum::<usize>() == expected_u_knots);
            let v_mult_idx = mult_lists.iter().position(|m| m.iter().sum::<usize>() == expected_v_knots);
            let u_knot_idx = knot_value_lists.iter().position(|k| k.len() > 0 &&
                mult_lists.get(u_mult_idx.unwrap_or(0)).map_or(false, |m| m.len() == k.len()));
            let v_knot_idx = knot_value_lists.iter().position(|k| k.len() > 0 &&
                mult_lists.get(v_mult_idx.unwrap_or(1)).map_or(false, |m| m.len() == k.len()));

            if let (Some(umi), Some(vmi), Some(uki), Some(vki)) = (u_mult_idx, v_mult_idx, u_knot_idx, v_knot_idx) {
                let u_knots = expand_knot_vector(&mult_lists[umi], &knot_value_lists[uki]);
                let v_knots = expand_knot_vector(&mult_lists[vmi], &knot_value_lists[vki]);

                if u_knots.len() == expected_u_knots && v_knots.len() == expected_v_knots {
                    return (u_knots, v_knots);
                }
            }

            // Fallback: try all pairings
            for (mi, m) in mult_lists.iter().enumerate() {
                for (ki, k) in knot_value_lists.iter().enumerate() {
                    if m.len() == k.len() {
                        let expanded = expand_knot_vector(m, k);
                        if expanded.len() == expected_u_knots {
                            for (mi2, m2) in mult_lists.iter().enumerate() {
                                if mi2 == mi { continue; }
                                for (ki2, k2) in knot_value_lists.iter().enumerate() {
                                    if ki2 == ki { continue; }
                                    if m2.len() == k2.len() {
                                        let expanded2 = expand_knot_vector(m2, k2);
                                        if expanded2.len() == expected_v_knots {
                                            return (expanded, expanded2);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // Strategy 3: Try using numeric lists directly (without multiplicity expansion)
        if numeric_lists.len() >= 2 {
            if let Some(k) = numeric_lists.iter().find(|l| l.len() == expected_u_knots) {
                if let Some(k2) = numeric_lists.iter().find(|l| l.len() == expected_v_knots && l.as_ptr() != k.as_ptr()) {
                    return (k.clone(), k2.clone());
                }
            }
        }

        // Strategy 4: Generate uniform knot vectors as fallback
        let u_n = expected_u_knots;
        let v_n = expected_v_knots;
        let u_knots = (0..u_n).map(|i| i as f64 / (u_n - 1).max(1) as f64).collect();
        let v_knots = (0..v_n).map(|i| i as f64 / (v_n - 1).max(1) as f64).collect();
        (u_knots, v_knots)
    }

    /// Extract a RECTANGULAR_TRIMMED_SURFACE (wrapper around another surface).
    fn extract_trimmed_surface(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        // RECTANGULAR_TRIMMED_SURFACE(#basis_surface, u1, u2, v1, v2, .T., .T.)
        let basis_id = self.get_ref(entity.params.first()?)?;
        self.extract_surface(basis_id)
    }

    /// Extract a SWEPT_SURFACE (surface created by sweeping a curve).
    fn extract_swept_surface(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        // SWEPT_SURFACE('', #profile_curve, #swept_curve_or_direction)
        // This is a base type for SURFACE_OF_REVOLUTION and SURFACE_OF_LINEAR_EXTRUSION
        // Try to extract the profile curve and create a revolution or extrusion
        let profile_id = self.find_curve_ref(entity, 1)?;

        // Look for the 3rd param — could be a direction or axis placement
        if let Some(param) = entity.params.get(2) {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(dir_entity) = self.step.find_entity(ref_id) {
                    if dir_entity.type_name == "DIRECTION" {
                        // It's a linear extrusion
                        let profile = self.resolve_curve(profile_id)?;
                        let direction = self.resolve_direction(ref_id)?;
                        return Some(Surface::Extrusion(ExtrusionSurface {
                            profile,
                            direction,
                        }));
                    } else if dir_entity.type_name.contains("AXIS2_PLACEMENT") {
                        // It's a revolution
                        let profile = self.resolve_curve(profile_id)?;
                        let (origin, axis, _u_dir) = self.resolve_axis2(ref_id)?;
                        return Some(Surface::Revolution(RevolutionSurface {
                            profile,
                            axis,
                            origin,
                        }));
                    }
                }
            }
        }
        None
    }

    /// Find a curve reference from an entity's parameters.
    /// Handles both direct references and indirect references through
    /// entities like DEFINITIONAL_REPRESENTATION, GEOMETRIC_REPRESENTATION_ITEM, etc.
    fn find_curve_ref(&self, entity: &crate::schema::StepEntity, param_index: usize) -> Option<i64> {
        if let Some(param) = entity.params.get(param_index) {
            if let Some(ref_id) = self.get_ref(param) {
                // Direct reference — check if it's a curve entity
                if let Some(curve_entity) = self.step.find_entity(ref_id) {
                    if self.is_curve_type(&curve_entity.type_name) {
                        return Some(ref_id);
                    }
                    // Check if it's a SURFACE_CURVE wrapping a curve
                    if curve_entity.type_name == "SURFACE_CURVE" {
                        return self.resolve_3d_curve_ref(ref_id);
                    }
                    // Indirect reference — try to find a curve within this entity
                    return self.find_nested_curve(curve_entity);
                }
            }
        }
        None
    }

    /// Find a curve reference nested inside an entity (e.g., through
    /// DEFINITIONAL_REPRESENTATION, GEOMETRIC_REPRESENTATION_ITEM, etc.)
    fn find_nested_curve(&self, entity: &crate::schema::StepEntity) -> Option<i64> {
        for param in &entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(nested) = self.step.find_entity(ref_id) {
                    if self.is_curve_type(&nested.type_name) {
                        return Some(ref_id);
                    }
                    // Check SURFACE_CURVE
                    if nested.type_name == "SURFACE_CURVE" {
                        return self.resolve_3d_curve_ref(ref_id);
                    }
                    // Go one level deeper
                    let deeper = self.find_nested_curve(nested);
                    if deeper.is_some() {
                        return deeper;
                    }
                }
            }
            // Also check lists
            if let StepValue::List(items) = param {
                for item in items {
                    if let Some(ref_id) = self.get_ref(item) {
                        if let Some(nested) = self.step.find_entity(ref_id) {
                            if self.is_curve_type(&nested.type_name) {
                                return Some(ref_id);
                            }
                        }
                    }
                }
            }
        }
        None
    }

    /// Check if a type name is a known curve type.
    fn is_curve_type(&self, type_name: &str) -> bool {
        matches!(
            type_name,
            "LINE" | "CIRCLE" | "ELLIPSE" | "B_SPLINE_CURVE_WITH_KNOTS" |
            "B_SPLINE_CURVE" | "BEZIER_CURVE" | "POLYLINE" | "TRIMMED_CURVE" |
            "COMPOSITE_CURVE" | "COMPOSITE_CURVE_SEGMENT" | "OFFSET_CURVE_3D" |
            "HYPERBOLA" | "PARABOLA" | "RATIONAL_B_SPLINE_CURVE" |
            "SURFACE_CURVE"
        ) || type_name.contains("B_SPLINE_CURVE") // Handle complex entity types like "BOUNDED_CURVE+B_SPLINE_CURVE_WITH_KNOTS+..."
    }

    /// Find a reference in the entity's parameters at a specific index.
    fn find_param_ref(&self, entity: &crate::schema::StepEntity, index: usize) -> Option<i64> {
        if let Some(param) = entity.params.get(index) {
            self.get_ref(param)
        } else {
            None
        }
    }

    /// Find a reference to an AXIS2_PLACEMENT_3D (or 2D/1D variant) entity in the params list.
    /// Handles the case where the name parameter may or may not be present.
    fn find_axis2_ref(&self, entity: &crate::schema::StepEntity) -> Option<i64> {
        for param in &entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(referenced) = self.step.find_entity(ref_id) {
                    if referenced.type_name == "AXIS2_PLACEMENT_3D"
                        || referenced.type_name == "AXIS2_PLACEMENT_2D"
                        || referenced.type_name == "AXIS1_PLACEMENT"
                    {
                        return Some(ref_id);
                    }
                }
            }
        }
        None
    }

    /// Find a float parameter by searching all params, skipping the first `skip` potential matches.
    /// This handles cases where the name parameter may or may not be present.
    fn find_float_param(&self, entity: &crate::schema::StepEntity, skip: usize) -> Option<f64> {
        let mut found = 0;
        for param in &entity.params {
            if let Some(val) = self.get_float(param) {
                if found >= skip {
                    return Some(val);
                }
                found += 1;
            }
        }
        None
    }

    /// Find a DIRECTION reference nested inside a VECTOR entity.
    /// VECTOR(#direction, magnitude) — extract the direction reference.
    fn find_direction_from_vector(&self, vector_entity: &crate::schema::StepEntity) -> Option<i64> {
        for param in &vector_entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(referenced) = self.step.find_entity(ref_id) {
                    if referenced.type_name == "DIRECTION" {
                        return Some(ref_id);
                    }
                }
            }
        }
        None
    }

    /// Resolve a STEP curve entity to a Curve3d.
    fn resolve_curve(&self, curve_id: i64) -> Option<Curve3d> {
        let entity = self.step.find_entity(curve_id)?;
        let type_name = entity.type_name.as_str();

        // Handle complex entity types (e.g., "BOUNDED_CURVE+B_SPLINE_CURVE_WITH_KNOTS+RATIONAL_B_SPLINE_CURVE+...")
        if type_name.contains("B_SPLINE_CURVE") {
            return self.resolve_bspline_curve(entity);
        }

        match type_name {
            "LINE" => self.resolve_line_curve(entity),
            "CIRCLE" => self.resolve_circle_curve(entity),
            "ELLIPSE" => self.resolve_ellipse_curve(entity),
            "B_SPLINE_CURVE_WITH_KNOTS" | "B_SPLINE_CURVE" | "BEZIER_CURVE" |
            "RATIONAL_B_SPLINE_CURVE" => self.resolve_bspline_curve(entity),
            "POLYLINE" => self.resolve_polyline_curve(entity),
            "TRIMMED_CURVE" => self.resolve_trimmed_curve(entity),
            "COMPOSITE_CURVE" => self.resolve_composite_curve(entity),
            "OFFSET_CURVE_3D" => self.resolve_offset_curve_3d(entity),
            "SURFACE_CURVE" => {
                // Unwrap SURFACE_CURVE to get the 3D curve
                if let Some(curve3d_id) = self.resolve_3d_curve_ref(curve_id) {
                    self.resolve_curve(curve3d_id)
                } else {
                    None
                }
            }
            _ => None,
        }
    }

    /// Resolve a LINE curve entity.
    /// STEP format: `#N = LINE(#point, #direction);` or `#N = LINE('', #point, #direction);`
    /// Also handles VECTOR references: `#N = LINE(#point, #vector);`
    fn resolve_line_curve(&self, entity: &crate::schema::StepEntity) -> Option<Curve3d> {
        let mut point_ref: Option<i64> = None;
        let mut dir_ref: Option<i64> = None;

        for param in &entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(referenced) = self.step.find_entity(ref_id) {
                    if referenced.type_name == "CARTESIAN_POINT" && point_ref.is_none() {
                        point_ref = Some(ref_id);
                    } else if referenced.type_name == "DIRECTION" && dir_ref.is_none() {
                        dir_ref = Some(ref_id);
                    } else if referenced.type_name == "VECTOR" && dir_ref.is_none() {
                        // VECTOR(#direction, magnitude) — extract direction from it
                        dir_ref = self.find_direction_from_vector(referenced);
                    }
                }
            }
        }

        let origin = point_ref.and_then(|id| self.resolve_cartesian_point(id))?;
        let direction = dir_ref.and_then(|id| self.resolve_direction(id))?;
        Some(Curve3d::Line(Line::new(origin, direction)))
    }

    /// Resolve a CIRCLE curve entity.
    /// STEP format: `#N = CIRCLE('', #axis2, radius);` or `#N = CIRCLE(#axis2, radius);`
    fn resolve_circle_curve(&self, entity: &crate::schema::StepEntity) -> Option<Curve3d> {
        let axis2_id = self.find_axis2_ref(entity)?;
        let (center, normal, x_axis) = self.resolve_axis2(axis2_id)?;
        let radius = self.find_float_param(entity, 0)?;
        Some(Curve3d::Circle(Circle {
            center,
            normal,
            radius,
            x_axis,
        }))
    }

    /// Resolve an ELLIPSE curve entity.
    /// STEP format: `#N = ELLIPSE('', #axis2, semi_major, semi_minor);`
    fn resolve_ellipse_curve(&self, entity: &crate::schema::StepEntity) -> Option<Curve3d> {
        let axis2_id = self.find_axis2_ref(entity)?;
        let (center, _normal, _x_axis) = self.resolve_axis2(axis2_id)?;
        let semi_major = self.find_float_param(entity, 0)?;
        let semi_minor = self.find_float_param(entity, 1)?;
        Some(Curve3d::Ellipse(draper_geometry::Ellipse::new_xy(center, semi_major, semi_minor)))
    }

    /// Resolve a B_SPLINE_CURVE_WITH_KNOTS entity (or complex entity containing B_SPLINE_CURVE).
    fn resolve_bspline_curve(&self, entity: &crate::schema::StepEntity) -> Option<Curve3d> {
        // For complex entities, find the B_SPLINE_CURVE sub-entity for control points
        // and B_SPLINE_CURVE_WITH_KNOTS sub-entity for knot vectors
        // and RATIONAL_B_SPLINE_CURVE sub-entity for weights
        let bspline_sub = entity.find_sub_entity("B_SPLINE_CURVE");
        let knots_sub = entity.find_sub_entity("B_SPLINE_CURVE_WITH_KNOTS");
        let rational_sub = entity.find_sub_entity("RATIONAL_B_SPLINE_CURVE");

        // Use the B_SPLINE_CURVE sub-entity if available, otherwise use the entity itself
        let cp_entity = bspline_sub.unwrap_or(entity);
        let knot_entity = knots_sub.unwrap_or(entity);

        let degree = self.get_float(cp_entity.params.first()?).unwrap_or(1.0) as usize;

        // Control points: 2nd param is a list of points
        let mut control_points = Vec::new();
        if let Some(StepValue::List(items)) = cp_entity.params.get(1) {
            for item in items {
                if let Some(ref_id) = self.get_ref(item) {
                    if let Some(pt) = self.resolve_cartesian_point(ref_id) {
                        control_points.push(pt);
                    }
                } else if let StepValue::List(coords) = item {
                    let x = coords.get(0).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                    let y = coords.get(1).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                    let z = coords.get(2).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                    control_points.push(Point3d::new(x, y, z));
                }
            }
        }

        if control_points.is_empty() {
            return None;
        }

        // Extract weights from RATIONAL_B_SPLINE_CURVE sub-entity if present
        let weights = if let Some(rational_ent) = rational_sub {
            self.extract_curve_weights(rational_ent, control_points.len())
        } else {
            vec![1.0; control_points.len()]
        };

        // Extract knots from the B_SPLINE_CURVE_WITH_KNOTS sub-entity if available
        let n = control_points.len();
        let knots = self.extract_curve_knots(knot_entity, n, degree);

        Some(Curve3d::Nurbs(NurbsCurve {
            degree,
            control_points,
            weights,
            knots,
        }))
    }

    /// Extract weight list from a RATIONAL_B_SPLINE_CURVE sub-entity.
    /// Format: RATIONAL_B_SPLINE_CURVE(weights_list) or as part of a complex entity.
    fn extract_curve_weights(&self, entity: &crate::schema::StepEntity, n_cp: usize) -> Vec<f64> {
        // Search for a list of floats in the entity params
        for param in &entity.params {
            if let StepValue::List(items) = param {
                let weights: Vec<f64> = items.iter()
                    .filter_map(|v| self.get_float(v))
                    .collect();
                if weights.len() == n_cp {
                    return weights;
                }
                // If the list has floats but wrong length, try to use what we have
                if !weights.is_empty() && weights.len() >= 2 {
                    let mut result = vec![1.0; n_cp];
                    let len = weights.len().min(n_cp);
                    result[..len].copy_from_slice(&weights[..len]);
                    return result;
                }
            }
        }
        vec![1.0; n_cp]
    }

    /// Extract knot vector from a B_SPLINE_CURVE entity.
    fn extract_curve_knots(&self, entity: &crate::schema::StepEntity, n_cp: usize, degree: usize) -> Vec<f64> {
        let expected_knot_count = n_cp + degree + 1;

        // Search for knot vector in parameters after control points
        // For B_SPLINE_CURVE_WITH_KNOTS: params typically include knot multiplicities and knot values
        let mut knot_lists: Vec<Vec<f64>> = Vec::new();
        for param in entity.params.iter().skip(2) {
            if let StepValue::List(items) = param {
                let floats: Vec<f64> = items.iter()
                    .filter_map(|v| self.get_float(v))
                    .collect();
                if floats.len() >= 2 && floats.iter().all(|f| f.is_finite()) {
                    knot_lists.push(floats);
                }
            }
        }

        // Try to find a knot list matching the expected length
        if let Some(k) = knot_lists.iter().find(|l| l.len() == expected_knot_count) {
            return k.clone();
        }

        // If we found some lists, the longest one might be the knot vector
        if !knot_lists.is_empty() {
            knot_lists.sort_by(|a, b| b.len().cmp(&a.len()));
            // Check if the longest list could be knots (monotonically increasing or non-decreasing)
            let candidate = &knot_lists[0];
            let is_monotonic = candidate.windows(2).all(|w| w[0] <= w[1] + 1e-10);
            if is_monotonic && candidate.len() >= degree + 2 {
                return candidate.clone();
            }
        }

        // Fallback: generate uniform knot vector
        let knot_count = expected_knot_count;
        (0..knot_count).map(|i| i as f64 / (knot_count - 1).max(1) as f64).collect()
    }

    /// Resolve a POLYLINE entity — return as a line segment approximation.
    fn resolve_polyline_curve(&self, entity: &crate::schema::StepEntity) -> Option<Curve3d> {
        // POLYLINE('', (#pt1, #pt2, ...))
        // Use the first and last points to create a line
        let mut points = Vec::new();
        for param in &entity.params {
            if let StepValue::List(items) = param {
                for item in items {
                    if let Some(ref_id) = self.get_ref(item) {
                        if let Some(pt) = self.resolve_cartesian_point(ref_id) {
                            points.push(pt);
                        }
                    }
                }
            }
        }

        if points.len() >= 2 {
            let line = Line::through_points(points[0], *points.last().unwrap())?;
            Some(Curve3d::Line(line))
        } else {
            None
        }
    }

    /// Resolve a TRIMMED_CURVE entity.
    fn resolve_trimmed_curve(&self, entity: &crate::schema::StepEntity) -> Option<Curve3d> {
        // TRIMMED_CURVE(#basis_curve, #trim1, #trim2, .T., .T., .CARTESIAN., .CARTESIAN.)
        let basis_id = self.get_ref(entity.params.first()?)?;
        self.resolve_curve(basis_id)
    }

    /// Resolve a COMPOSITE_CURVE entity.
    fn resolve_composite_curve(&self, entity: &crate::schema::StepEntity) -> Option<Curve3d> {
        // COMPOSITE_CURVE('', (#segment1, #segment2, ...), .U.)
        // Use the first segment as a representative curve
        for param in &entity.params {
            if let StepValue::List(items) = param {
                for item in items {
                    if let Some(ref_id) = self.get_ref(item) {
                        // Each segment is a COMPOSITE_CURVE_SEGMENT
                        if let Some(seg_entity) = self.step.find_entity(ref_id) {
                            if seg_entity.type_name == "COMPOSITE_CURVE_SEGMENT" {
                                // The 2nd param is the parent curve
                                if let Some(curve_id) = self.find_param_ref(seg_entity, 1) {
                                    if let Some(curve) = self.resolve_curve(curve_id) {
                                        return Some(curve);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        None
    }

    /// Resolve an OFFSET_CURVE_3D entity — returns the basis curve (offset is approximated).
    /// Format: OFFSET_CURVE_3D('', #basis_curve, distance, #direction, .F.)
    fn resolve_offset_curve_3d(&self, entity: &crate::schema::StepEntity) -> Option<Curve3d> {
        // Find the basis curve reference
        for param in &entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(curve_entity) = self.step.find_entity(ref_id) {
                    if self.is_curve_type(&curve_entity.type_name) {
                        return self.resolve_curve(ref_id);
                    }
                }
            }
        }
        None
    }

    /// Convert a FaceData (surface + boundary edges) to a mesh by creating a Face
    /// with proper wire/edges and triangulating.
    fn surface_to_mesh(
        &self,
        face_data: &FaceData,
        params: &TriangulationParams,
        bbox: &Option<(Point3d, Point3d)>,
    ) -> TriangleMesh {
        if face_data.edges.is_empty() {
            // No boundary edges — fall back to bounding-box-based triangulation for planes,
            // or standard triangulation for curved surfaces
            if let Surface::Plane(ref plane) = face_data.surface {
                return self.triangulate_unbounded_plane(plane, params, bbox);
            }
            
            let wire = Wire::new(vec![]);
            let mut face = Face::new(face_data.surface.clone(), wire);
            face.forward = face_data.forward;
            face.edges = vec![];
            return triangulate_face(&face, params);
        }

        // For planar faces with inner loops (holes), use the dedicated hole-aware path
        if let Surface::Plane(ref plane) = face_data.surface {
            if !face_data.inner_edges.is_empty() {
                return self.triangulate_planar_face_with_holes(
                    plane, &face_data.outer_edges, &face_data.inner_edges, face_data.forward,
                );
            }
        }

        // Collect 3D boundary points from outer edge curves by sampling each edge.
        // Use adaptive sampling: fewer samples for line edges, more for curved edges.
        let mut boundary_points = Vec::new();
        for edge in &face_data.outer_edges {
            let n_samples = self.edge_sample_count(edge);
            for i in 0..n_samples {
                let t = i as f64 / (n_samples - 1).max(1) as f64;
                if let Some(p) = edge.point_at(t) {
                    boundary_points.push(p);
                }
            }
        }

        // If outer boundary is empty, try all edges
        if boundary_points.is_empty() {
            for edge in &face_data.edges {
                let n_samples = self.edge_sample_count(edge);
                for i in 0..n_samples {
                    let t = i as f64 / (n_samples - 1).max(1) as f64;
                    if let Some(p) = edge.point_at(t) {
                        boundary_points.push(p);
                    }
                }
            }
        }

        // Deduplicate boundary points — critical for ear clipping to work correctly.
        // Without deduplication, shared vertices between edges create zero-area triangles
        // and self-intersecting polygons that break the triangulation.
        boundary_points = deduplicate_points_3d(&boundary_points, 1e-6);

        // If we have boundary points, use boundary-aware triangulation
        if !boundary_points.is_empty() {
            return triangulate_face_with_boundary(
                &face_data.surface,
                &boundary_points,
                face_data.forward,
                params,
            );
        }

        // Fallback: use the old Face-based path
        let coedges: Vec<CoEdge> = face_data.edges.iter().map(|e| {
            CoEdge::new(e.id, true)
        }).collect();
        let wire = Wire::new(coedges);

        let mut face = Face::new(face_data.surface.clone(), wire);
        face.forward = face_data.forward;
        face.edges = face_data.edges.clone();

        triangulate_face(&face, params)
    }

    /// Triangulate a planar face with holes using the bridge-edge technique.
    /// This connects each hole to the outer boundary with a pair of coincident edges,
    /// creating a single polygon that can be ear-clipped.
    fn triangulate_planar_face_with_holes(
        &self,
        plane: &Plane,
        outer_edges: &[TopoEdge],
        inner_loops: &[Vec<TopoEdge>],
        forward: bool,
    ) -> TriangleMesh {
        let mut mesh = TriangleMesh::new();

        // Sample outer boundary points
        let outer_points_3d = self.sample_edges(outer_edges);
        if outer_points_3d.is_empty() {
            return mesh;
        }

        // Project all points onto the plane's 2D coordinate system
        let project = |p: &Point3d| -> Point2d {
            let dx = p.x - plane.origin.x;
            let dy = p.y - plane.origin.y;
            let dz = p.z - plane.origin.z;
            Point2d::new(
                dx * plane.u_dir.x + dy * plane.u_dir.y + dz * plane.u_dir.z,
                dx * plane.v_dir.x + dy * plane.v_dir.y + dz * plane.v_dir.z,
            )
        };

        let outer_2d: Vec<Point2d> = outer_points_3d.iter().map(|p| project(p)).collect();

        // Sample inner loop (hole) points
        let mut hole_points_3d: Vec<Vec<Point3d>> = Vec::new();
        let mut hole_points_2d: Vec<Vec<Point2d>> = Vec::new();
        for inner_edges in inner_loops {
            let pts_3d = self.sample_edges(inner_edges);
            if pts_3d.is_empty() { continue; }
            let pts_2d: Vec<Point2d> = pts_3d.iter().map(|p| project(p)).collect();
            hole_points_3d.push(pts_3d);
            hole_points_2d.push(pts_2d);
        }

        // Use bridge-edge technique to merge holes into the outer polygon
        let (merged_2d, merged_3d) = merge_holes_into_polygon(&outer_2d, &outer_points_3d, &hole_points_2d, &hole_points_3d);

        // Ear clipping triangulation of the merged polygon
        let triangles = ear_clip(&merged_2d);

        // Add vertices and triangles
        for p in &merged_3d {
            mesh.add_vertex(*p);
        }
        for tri in &triangles {
            if forward {
                mesh.add_triangle(tri[0], tri[1], tri[2]);
            } else {
                mesh.add_triangle(tri[0], tri[2], tri[1]);
            }
        }

        mesh
    }

    /// Sample points from a list of edges at uniform parameter intervals.
    /// Determine the number of samples to take from an edge based on its curve type.
    /// Lines need only 2 samples (start and end), while circles/NURBS need more for curvature.
    fn edge_sample_count(&self, edge: &TopoEdge) -> usize {
        match &edge.curve {
            Some(Curve3d::Line(_)) => 2,
            Some(Curve3d::Circle(_)) => 36,
            Some(Curve3d::Ellipse(_)) => 36,
            Some(Curve3d::Arc(_)) => 24,
            Some(Curve3d::Nurbs(_)) => 48,
            None => 2,
        }
    }

    fn sample_edges(&self, edges: &[TopoEdge]) -> Vec<Point3d> {
        let mut points = Vec::new();
        for edge in edges {
            let n_samples = self.edge_sample_count(edge);
            for i in 0..n_samples {
                let t = i as f64 / (n_samples - 1).max(1) as f64;
                if let Some(p) = edge.point_at(t) {
                    points.push(p);
                }
            }
        }

        // Remove near-duplicate consecutive points
        points = deduplicate_points_3d(&points, 1e-6);

        points
    }

    /// Triangulate a PLANE surface with no boundary edges, using the bounding box
    /// to determine a finite extent.
    fn triangulate_unbounded_plane(
        &self,
        plane: &Plane,
        _params: &TriangulationParams,
        bbox: &Option<(Point3d, Point3d)>,
    ) -> TriangleMesh {
        let mut mesh = TriangleMesh::new();

        // Determine the plane's extent from the bounding box
        let (size, center) = if let Some((bmin, bmax)) = bbox {
            let cx = (bmin.x + bmax.x) / 2.0;
            let cy = (bmin.y + bmax.y) / 2.0;
            let cz = (bmin.z + bmax.z) / 2.0;
            let sx = (bmax.x - bmin.x).max(1.0);
            let sy = (bmax.y - bmin.y).max(1.0);
            let sz = (bmax.z - bmin.z).max(1.0);
            let max_dim = sx.max(sy).max(sz);
            (max_dim, Point3d::new(cx, cy, cz))
        } else {
            (100.0, plane.origin)
        };

        // Create a 4x4 grid of points on the plane for better surface representation
        let n = 4;
        let half = size * 0.5;

        for j in 0..=n {
            for i in 0..=n {
                let u = -half + size * i as f64 / n as f64;
                let v = -half + size * j as f64 / n as f64;
                let p = Point3d::new(
                    center.x + u * plane.u_dir.x + v * plane.v_dir.x,
                    center.y + u * plane.u_dir.y + v * plane.v_dir.y,
                    center.z + u * plane.u_dir.z + v * plane.v_dir.z,
                );
                mesh.add_vertex(p);
            }
        }

        let cols = n + 1;
        for j in 0..n {
            for i in 0..n {
                let v0 = (j * cols + i) as u32;
                let v1 = (j * cols + i + 1) as u32;
                let v2 = ((j + 1) * cols + i + 1) as u32;
                let v3 = ((j + 1) * cols + i) as u32;
                mesh.add_triangle(v0, v1, v2);
                mesh.add_triangle(v0, v2, v3);
            }
        }

        mesh
    }

    /// Resolve an AXIS2_PLACEMENT_3D entity to (origin, z_direction, x_direction).
    ///
    /// STEP AP203/AP214 format:
    ///   `#N = AXIS2_PLACEMENT_3D('', #location, #axis, #ref_direction);`
    /// Some files omit the name parameter:
    ///   `#N = AXIS2_PLACEMENT_3D(#location, #axis, #ref_direction);`
    ///
    /// We handle both cases by scanning parameters for their entity types
    /// instead of assuming fixed positional indices.
    fn resolve_axis2(&self, axis2_id: i64) -> Option<(Point3d, Direction3d, Direction3d)> {
        let entity = self.step.find_entity(axis2_id)?;

        let mut origin: Option<Point3d> = None;
        let mut directions: Vec<Direction3d> = Vec::new();

        for param in &entity.params {
            if let Some(ref_id) = self.get_ref(param) {
                if let Some(referenced) = self.step.find_entity(ref_id) {
                    if referenced.type_name == "CARTESIAN_POINT" {
                        if origin.is_none() {
                            origin = self.resolve_cartesian_point(ref_id);
                        }
                    } else if referenced.type_name == "DIRECTION" {
                        if let Some(dir) = self.resolve_direction(ref_id) {
                            directions.push(dir);
                        }
                    }
                }
            }
        }

        let origin = origin?;
        let z_dir = directions.get(0).copied().unwrap_or(Direction3d::Z);
        let x_dir = directions.get(1).copied().unwrap_or_else(|| Self::default_x_dir(&z_dir));

        Some((origin, z_dir, x_dir))
    }

    /// Compute a default x direction given a z direction.
    fn default_x_dir(z_dir: &Direction3d) -> Direction3d {
        if z_dir.is_parallel_to(&Direction3d::Y) {
            Direction3d::X
        } else {
            z_dir.cross(&Direction3d::Y)
        }
    }

    /// Resolve a CARTESIAN_POINT entity.
    fn resolve_cartesian_point(&self, point_id: i64) -> Option<Point3d> {
        let entity = self.step.find_entity(point_id)?;
        for param in &entity.params {
            if let StepValue::List(coords) = param {
                let x = self.get_float(coords.get(0)?)?;
                let y = self.get_float(coords.get(1)?)?;
                let z = coords.get(2).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                return Some(Point3d::new(x, y, z));
            }
        }
        None
    }

    /// Resolve a DIRECTION entity.
    fn resolve_direction(&self, dir_id: i64) -> Option<Direction3d> {
        let entity = self.step.find_entity(dir_id)?;
        for param in &entity.params {
            if let StepValue::List(coords) = param {
                let x = self.get_float(coords.get(0)?)?;
                let y = self.get_float(coords.get(1)?)?;
                let z = coords.get(2).and_then(|v| self.get_float(v)).unwrap_or(0.0);
                return Direction3d::new(x, y, z);
            }
        }
        None
    }

    // Helper methods

    fn get_ref(&self, value: &StepValue) -> Option<i64> {
        match value {
            StepValue::Ref(id) => Some(*id),
            _ => None,
        }
    }

    fn get_float(&self, value: &StepValue) -> Option<f64> {
        match value {
            StepValue::Float(f) => Some(*f),
            StepValue::Integer(i) => Some(*i as f64),
            _ => None,
        }
    }
}

/// Project a 3D point onto a line and return the parameter t.
/// For Line: P(t) = origin + t * direction
/// t = dot(point - origin, direction)
fn project_point_on_line(line: &Line, point: &Point3d) -> f64 {
    let dx = point.x - line.origin.x;
    let dy = point.y - line.origin.y;
    let dz = point.z - line.origin.z;
    dx * line.direction.x + dy * line.direction.y + dz * line.direction.z
}

/// Deduplicate a list of 3D points by removing consecutive points that are within
/// the given tolerance. Also removes the last point if it coincides with the first
/// (closing a loop). This is essential for ear clipping algorithms which produce
/// degenerate triangles on duplicate vertices.
fn deduplicate_points_3d(points: &[Point3d], tolerance: f64) -> Vec<Point3d> {
    if points.is_empty() {
        return Vec::new();
    }

    let tol_sq = tolerance * tolerance;
    let mut unique = vec![points[0]];
    for p in &points[1..] {
        let last = unique.last().unwrap();
        let dx = p.x - last.x;
        let dy = p.y - last.y;
        let dz = p.z - last.z;
        if dx * dx + dy * dy + dz * dz > tol_sq {
            unique.push(*p);
        }
    }
    // Also check last vs first (closed loop)
    if unique.len() > 1 {
        let first = unique[0];
        let last = unique.last().unwrap();
        let dx = first.x - last.x;
        let dy = first.y - last.y;
        let dz = first.z - last.z;
        if dx * dx + dy * dy + dz * dz <= tol_sq {
            unique.pop();
        }
    }
    unique
}

/// Project two 3D points onto a circle and return the angular parameter range (t1, t2).
/// The angles are computed in the circle's local coordinate system.
/// t1 and t2 are in radians and the arc goes from t1 to t2 in the positive direction.
/// For full circles (p1 ≈ p2), returns a full 2π range.
fn project_points_on_circle(circle: &Circle, p1: &Point3d, p2: &Point3d) -> (f64, f64) {
    let y_axis = circle.normal.cross(&circle.x_axis);

    let d1x = p1.x - circle.center.x;
    let d1y = p1.y - circle.center.y;
    let d1z = p1.z - circle.center.z;
    let local1_x = d1x * circle.x_axis.x + d1y * circle.x_axis.y + d1z * circle.x_axis.z;
    let local1_y = d1x * y_axis.x + d1y * y_axis.y + d1z * y_axis.z;
    let t1 = local1_y.atan2(local1_x);

    // Check if p1 and p2 are approximately the same point (full circle)
    let dist_sq = (p2.x - p1.x).powi(2) + (p2.y - p1.y).powi(2) + (p2.z - p1.z).powi(2);
    if dist_sq < 1e-10 {
        // Full circle — use the full 2π range starting from t1
        return (t1, t1 + 2.0 * std::f64::consts::PI);
    }

    let d2x = p2.x - circle.center.x;
    let d2y = p2.y - circle.center.y;
    let d2z = p2.z - circle.center.z;
    let local2_x = d2x * circle.x_axis.x + d2y * circle.x_axis.y + d2z * circle.x_axis.z;
    let local2_y = d2x * y_axis.x + d2y * y_axis.y + d2z * y_axis.z;
    let t2 = local2_y.atan2(local2_x);

    // Ensure t2 > t1 (positive direction arc from t1 to t2)
    let mut t2 = t2;
    while t2 <= t1 {
        t2 += 2.0 * std::f64::consts::PI;
    }

    // Use the positive direction arc from t1 to t2 without assuming shorter arc.
    // The STEP file's orientation flag determines direction; we preserve the
    // natural order from p1 to p2 in the positive (counterclockwise) sense.
    (t1, t2)
}

/// Multiply two 4x4 matrices (row-major storage).
fn mat4_mul(a: &[[f64; 4]; 4], b: &[[f64; 4]; 4]) -> [[f64; 4]; 4] {
    let mut r = [[0.0f64; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            for k in 0..4 {
                r[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    r
}

/// Compute the inverse of a 4x4 matrix using cofactor expansion.
fn mat4_inverse(m: &[[f64; 4]; 4]) -> Option<[[f64; 4]; 4]> {
    let s0 = m[0][0] * m[1][1] - m[1][0] * m[0][1];
    let s1 = m[0][0] * m[1][2] - m[1][0] * m[0][2];
    let s2 = m[0][0] * m[1][3] - m[1][0] * m[0][3];
    let s3 = m[0][1] * m[1][2] - m[1][1] * m[0][2];
    let s4 = m[0][1] * m[1][3] - m[1][1] * m[0][3];
    let s5 = m[0][2] * m[1][3] - m[1][2] * m[0][3];

    let c5 = m[2][2] * m[3][3] - m[3][2] * m[2][3];
    let c4 = m[2][1] * m[3][3] - m[3][1] * m[2][3];
    let c3 = m[2][1] * m[3][2] - m[3][1] * m[2][2];
    let c2 = m[2][0] * m[3][3] - m[3][0] * m[2][3];
    let c1 = m[2][0] * m[3][2] - m[3][0] * m[2][2];
    let c0 = m[2][0] * m[3][1] - m[3][0] * m[2][1];

    let det = s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0;
    if det.abs() < 1e-12 { return None; }
    let inv_det = 1.0 / det;

    Some([
        [( m[1][1]*c5 - m[1][2]*c4 + m[1][3]*c3) * inv_det,
         (-m[0][1]*c5 + m[0][2]*c4 - m[0][3]*c3) * inv_det,
         ( m[3][1]*s5 - m[3][2]*s4 + m[3][3]*s3) * inv_det,
         (-m[2][1]*s5 + m[2][2]*s4 - m[2][3]*s3) * inv_det],
        [(-m[1][0]*c5 + m[1][2]*c2 - m[1][3]*c1) * inv_det,
         ( m[0][0]*c5 - m[0][2]*c2 + m[0][3]*c1) * inv_det,
         (-m[3][0]*s5 + m[3][2]*s2 - m[3][3]*s1) * inv_det,
         ( m[2][0]*s5 - m[2][2]*s2 + m[2][3]*s1) * inv_det],
        [( m[1][0]*c4 - m[1][1]*c2 + m[1][3]*c0) * inv_det,
         (-m[0][0]*c4 + m[0][1]*c2 - m[0][3]*c0) * inv_det,
         ( m[3][0]*s4 - m[3][1]*s2 + m[3][3]*s0) * inv_det,
         (-m[2][0]*s4 + m[2][1]*s2 - m[2][3]*s0) * inv_det],
        [(-m[1][0]*c3 + m[1][1]*c1 - m[1][2]*c0) * inv_det,
         ( m[0][0]*c3 - m[0][1]*c1 + m[0][2]*c0) * inv_det,
         (-m[3][0]*s3 + m[3][1]*s1 - m[3][2]*s0) * inv_det,
         ( m[2][0]*s3 - m[2][1]*s1 + m[2][2]*s0) * inv_det],
    ])
}

/// Resolve STEP predefined colour names to RGBA values.
/// These follow ISO 10209-2:1993 / STEP draughting predefined colours.
fn resolve_predefined_colour(name: &str) -> [f32; 4] {
    match name.to_lowercase().as_str() {
        "red" => [1.0, 0.0, 0.0, 1.0],
        "green" => [0.0, 1.0, 0.0, 1.0],
        "blue" => [0.0, 0.0, 1.0, 1.0],
        "yellow" => [1.0, 1.0, 0.0, 1.0],
        "magenta" => [1.0, 0.0, 1.0, 1.0],
        "cyan" => [0.0, 1.0, 1.0, 1.0],
        "black" => [0.0, 0.0, 0.0, 1.0],
        "white" => [1.0, 1.0, 1.0, 1.0],
        "brown" => [0.6, 0.3, 0.1, 1.0],
        "orange" => [1.0, 0.65, 0.0, 1.0],
        "pink" => [1.0, 0.75, 0.8, 1.0],
        "purple" => [0.5, 0.0, 0.5, 1.0],
        "grey" | "gray" => [0.5, 0.5, 0.5, 1.0],
        "light grey" | "light gray" => [0.75, 0.75, 0.75, 1.0],
        "dark grey" | "dark gray" => [0.25, 0.25, 0.25, 1.0],
        _ => {
            warn!("Unknown predefined colour: {}", name);
            [0.5, 0.5, 0.5, 1.0] // Default grey
        }
    }
}

/// Expand a knot vector from distinct knot values and multiplicities.
/// For example: mults=[2,2], values=[0.0, 1.0] → [0.0, 0.0, 1.0, 1.0]
fn expand_knot_vector(multiplicities: &[usize], knot_values: &[f64]) -> Vec<f64> {
    if multiplicities.len() != knot_values.len() {
        // Mismatch — fall back to just repeating the values
        let mut result = Vec::new();
        for &v in knot_values {
            result.push(v);
            result.push(v);
        }
        return result;
    }
    let mut result = Vec::new();
    for (i, &mult) in multiplicities.iter().enumerate() {
        for _ in 0..mult {
            result.push(knot_values[i]);
        }
    }
    result
}

/// Merge holes into an outer polygon using the bridge-edge technique.
/// For each hole, find the rightmost point of the hole, then find the
/// closest visible point on the outer polygon (or previously merged holes),
/// and insert the hole at that point with a bridge edge.
///
/// The resulting polygon has all holes connected via zero-width bridges,
/// forming a single simple polygon that can be triangulated with ear-clipping.
fn merge_holes_into_polygon(
    outer_2d: &[Point2d],
    outer_3d: &[Point3d],
    holes_2d: &[Vec<Point2d>],
    holes_3d: &[Vec<Point3d>],
) -> (Vec<Point2d>, Vec<Point3d>) {
    if outer_2d.is_empty() {
        return (Vec::new(), Vec::new());
    }
    if holes_2d.is_empty() {
        return (outer_2d.to_vec(), outer_3d.to_vec());
    }

    let mut poly_2d: Vec<Point2d> = outer_2d.to_vec();
    let mut poly_3d: Vec<Point3d> = outer_3d.to_vec();

    // Sort holes by rightmost point (u-coordinate) descending,
    // so we process rightmost holes first for more stable bridge construction
    let mut hole_indices: Vec<usize> = (0..holes_2d.len()).collect();
    hole_indices.sort_by(|&a, &b| {
        let max_u_a = holes_2d[a].iter().map(|p| p.u).fold(f64::NEG_INFINITY, f64::max);
        let max_u_b = holes_2d[b].iter().map(|p| p.u).fold(f64::NEG_INFINITY, f64::max);
        max_u_b.partial_cmp(&max_u_a).unwrap_or(std::cmp::Ordering::Equal)
    });

    for hole_idx in hole_indices {
        let hole_2d = &holes_2d[hole_idx];
        let hole_3d = &holes_3d[hole_idx];
        if hole_2d.is_empty() { continue; }

        // Find the rightmost point of the hole
        let (rightmost_idx, _) = hole_2d.iter().enumerate()
            .max_by(|(_, a), (_, b)| a.u.partial_cmp(&b.u).unwrap_or(std::cmp::Ordering::Equal))
            .unwrap_or((0, &hole_2d[0]));

        // Find the closest point on the outer polygon to the rightmost hole point.
        // For simplicity, find the polygon vertex that is visible from the hole's
        // rightmost point and is closest in the u direction.
        let hole_pt = hole_2d[rightmost_idx];

        let mut best_poly_idx = 0;
        let mut best_dist = f64::MAX;
        for (i, pt) in poly_2d.iter().enumerate() {
            let dx = pt.u - hole_pt.u;
            let dy = pt.v - hole_pt.v;
            let dist = dx * dx + dy * dy;
            if dist < best_dist {
                best_dist = dist;
                best_poly_idx = i;
            }
        }

        // Insert the hole into the polygon at the bridge point
        // The bridge creates: ...poly[best] -> hole[rightmost] -> ...hole -> hole[rightmost] -> poly[best]...
        // This is done by inserting the hole (rotated to start at rightmost_idx)
        // twice at the bridge point, with the rightmost point duplicated.

        // Rotate hole to start at rightmost_idx
        let n_hole = hole_2d.len();
        let mut rotated_hole_2d = Vec::with_capacity(n_hole + 1);
        let mut rotated_hole_3d = Vec::with_capacity(n_hole + 1);
        for i in 0..=n_hole {
            let idx = (rightmost_idx + i) % n_hole;
            rotated_hole_2d.push(hole_2d[idx]);
            rotated_hole_3d.push(hole_3d[idx]);
        }

        // Insert: poly[..best+1] + bridge_point + hole + bridge_point + poly[best..]
        let mut new_poly_2d = Vec::new();
        let mut new_poly_3d = Vec::new();

        // Part 1: outer polygon up to and including the bridge point
        for i in 0..=best_poly_idx {
            new_poly_2d.push(poly_2d[i]);
            new_poly_3d.push(poly_3d[i]);
        }

        // Part 2: bridge to hole (rightmost point)
        new_poly_2d.push(hole_2d[rightmost_idx]);
        new_poly_3d.push(hole_3d[rightmost_idx]);

        // Part 3: hole vertices starting from rightmost+1 going around back to rightmost
        for i in 1..rotated_hole_2d.len() {
            new_poly_2d.push(rotated_hole_2d[i]);
            new_poly_3d.push(rotated_hole_3d[i]);
        }

        // Part 4: bridge back to the same outer polygon point
        new_poly_2d.push(poly_2d[best_poly_idx]);
        new_poly_3d.push(poly_3d[best_poly_idx]);

        // Part 5: rest of outer polygon after bridge point
        for i in (best_poly_idx + 1)..poly_2d.len() {
            new_poly_2d.push(poly_2d[i]);
            new_poly_3d.push(poly_3d[i]);
        }

        poly_2d = new_poly_2d;
        poly_3d = new_poly_3d;
    }

    (poly_2d, poly_3d)
}

#[cfg(test)]
mod diag_tests {
    use super::*;
    use crate::parse_step;

    fn diagnose_file(path: &str) {
        eprintln!("\n========================================");
        eprintln!("FILE: {}", path);
        eprintln!("========================================");
        
        let content = match std::fs::read_to_string(path) {
            Ok(c) => c,
            Err(e) => { eprintln!("ERROR reading: {}", e); return; }
        };
        
        let step = match parse_step(&content) {
            Ok(s) => s,
            Err(e) => { eprintln!("PARSE ERROR: {:?}", e); return; }
        };
        
        // Count surface types found in faces
        let faces = step.find_entities_by_type("ADVANCED_FACE");
        let shells = step.find_entities_by_type("CLOSED_SHELL");
        let open_shells = step.find_entities_by_type("OPEN_SHELL");
        let breps = step.find_entities_by_type("MANIFOLD_SOLID_BREP");
        eprintln!("  ADVANCED_FACE: {}, CLOSED_SHELL: {}, OPEN_SHELL: {}, MANIFOLD_SOLID_BREP: {}",
            faces.len(), shells.len(), open_shells.len(), breps.len());

        // For each face, find its surface type
        let mut surface_types: HashMap<String, usize> = HashMap::new();
        let mut faces_with_no_surface = 0;
        for face in &faces {
            if let Some(surface) = StepConverter::new(&step).extract_face_surface_from_entity(face) {
                let tn = match surface {
                    Surface::Plane(_) => "PLANE",
                    Surface::Cylinder(_) => "CYLINDER",
                    Surface::Cone(_) => "CONE",
                    Surface::Sphere(_) => "SPHERE",
                    Surface::Torus(_) => "TORUS",
                    Surface::Revolution(_) => "REVOLUTION",
                    Surface::Extrusion(_) => "EXTRUSION",
                    Surface::Nurbs(_) => "NURBS",
                };
                *surface_types.entry(tn.to_string()).or_insert(0) += 1;
            } else {
                faces_with_no_surface += 1;
                // Print what entity the surface ref points to
                let converter = StepConverter::new(&step);
                for (i, param) in face.params.iter().enumerate() {
                    if i == 0 { continue; }
                    if let Some(surface_id) = converter.get_ref(param) {
                        if let Some(entity) = step.find_entity(surface_id) {
                            eprintln!("    FACE #{}: ref #{} type='{}'", face.id, surface_id, entity.type_name);
                        }
                    }
                }
            }
        }
        eprintln!("  Surface types extracted: {:?}", surface_types);
        eprintln!("  Faces with NO surface: {}", faces_with_no_surface);

        // Count FACE_BOUND vs FACE_OUTER_BOUND usage
        let outer_bounds = step.find_entities_by_type("FACE_OUTER_BOUND").len();
        let inner_bounds = step.find_entities_by_type("FACE_BOUND").len();
        eprintln!("  FACE_OUTER_BOUND: {}, FACE_BOUND (holes): {}", outer_bounds, inner_bounds);

        // Try full conversion
        let converter = StepConverter::new(&step);
        match converter.convert_instances() {
            Ok(instances) => {
                let total_verts: usize = instances.iter().map(|i| i.mesh.vertex_count()).sum();
                let total_tris: usize = instances.iter().map(|i| i.mesh.triangle_count()).sum();
                eprintln!("  MESH: {} instances, {} verts, {} tris", instances.len(), total_verts, total_tris);
                for inst in &instances {
                    eprintln!("    {} : {}v {}t color={:?}", inst.name, inst.mesh.vertex_count(), inst.mesh.triangle_count(), inst.color);
                }
            }
            Err(e) => eprintln!("  CONVERSION ERROR: {}", e),
        }
    }

    #[test]
    fn test_brick_thin() { diagnose_file("/home/z/my-project/3Draper/test/brick_thin.stp"); }
    #[test]
    fn test_brick_thin_hole() { diagnose_file("/home/z/my-project/3Draper/test/brick_thin_hole.stp"); }
    #[test]
    fn test_brick_thin_round() { diagnose_file("/home/z/my-project/3Draper/test/brick_thin_round.stp"); }
    #[test]
    fn test_compressor() { diagnose_file("/home/z/my-project/3Draper/test/compressor-13920_top.stp"); }
    #[test]
    fn test_drill() { diagnose_file("/home/z/my-project/3Draper/test/drill_top.stp"); }
    #[test]
    fn test_transmission() { diagnose_file("/home/z/my-project/3Draper/test/transmission_top.stp"); }
    #[test]
    fn test_3_05_078() { diagnose_file("/home/z/my-project/3Draper/test/3.05.078.stp"); }
}
