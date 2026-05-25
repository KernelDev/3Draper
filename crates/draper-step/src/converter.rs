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
    Point3d, Direction3d, Surface, Plane, CylinderSurface, SphereSurface,
    ConeSurface, TorusSurface, RevolutionSurface, ExtrusionSurface,
    NurbsSurface, Curve3d, Line, Circle, NurbsCurve,
};
use draper_mesh::{TriangleMesh, TriangulationParams, triangulate_face, triangulate_face_with_boundary};
use draper_topology::{Face, Wire, CoEdge, Edge as TopoEdge};
use std::collections::HashMap;
use log::{info, warn};

/// Extracted face data with surface and boundary edges.
struct FaceData {
    surface: Surface,
    edges: Vec<TopoEdge>,
    forward: bool,
}

/// A colored mesh instance (mesh + optional RGBA color).
struct ColoredMesh {
    mesh: TriangleMesh,
    color: Option<[f32; 4]>, // RGBA, 0..1 range
}

/// Convert a parsed STEP file to a triangle mesh.
pub fn step_to_mesh(step_file: &StepFile) -> Result<TriangleMesh, String> {
    let converter = StepConverter::new(step_file);
    converter.convert()
}

/// Convert a parsed STEP file to colored mesh instances.
/// Returns one mesh per colored part (with per-part RGBA color).
pub fn step_to_colored_meshes(step_file: &StepFile) -> Result<Vec<ColoredMesh>, String> {
    let converter = StepConverter::new(step_file);
    converter.convert_colored()
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
        // Try colored assembly conversion first, then merge all into one mesh
        let colored = self.convert_colored()?;
        let mut mesh = TriangleMesh::new();
        for cm in colored {
            if let Some(color) = cm.color {
                mesh.merge_with_color(&cm.mesh, color);
            } else {
                mesh.merge_with_color(&cm.mesh, [0.48, 0.52, 0.58, 1.0]);
            }
        }
        Ok(mesh)
    }

    /// Convert STEP to colored mesh instances, properly handling assembly structure
    /// with NAUO/CDSR/ITEM_DEFINED_TRANSFORMATION transforms and STYLED_ITEM colors.
    fn convert_colored(&self) -> Result<Vec<ColoredMesh>, String> {
        let params = TriangulationParams::default();
        let bbox = self.compute_bounding_box();
        let mut results: Vec<ColoredMesh> = Vec::new();

        // ─── Phase 1: Try assembly-based conversion via NAUO/CDSR ─────────
        let nauos = self.step.find_entities_by_type("NEXT_ASSEMBLY_USAGE_OCCURRENCE");
        if !nauos.is_empty() {
            info!("Found {} NAUO assembly instances", nauos.len());

            // Build color map: brep_id → [f32;4] color
            let color_map = self.extract_color_map();

            // Build a map: child_pd_id → list of (nauo_id, relating_pd_id)
            let mut pd_to_nauos: HashMap<i64, Vec<(i64, i64)>> = HashMap::new();
            for nauo in &nauos {
                let (relating_pd, related_pd) = self.extract_nauo_pd_refs(nauo);
                if let (Some(parent_pd), Some(child_pd)) = (relating_pd, related_pd) {
                    pd_to_nauos.entry(child_pd).or_default().push((nauo.id, parent_pd));
                }
            }

            // For each NAUO that references a leaf BREP, compute composed transform
            for nauo in &nauos {
                let (relating_pd_id, related_pd_id) = self.extract_nauo_pd_refs(nauo);
                if relating_pd_id.is_none() || related_pd_id.is_none() {
                    continue;
                }
                let related_pd_id = related_pd_id.unwrap();

                // Find the BREP for the related product (skip sub-assemblies)
                let brep_id = self.find_pd_brep(related_pd_id);
                if brep_id.is_none() {
                    continue; // Sub-assembly NAUO — its children are handled separately
                }
                let brep_id = brep_id.unwrap();

                // Compute the composed transform from leaf to root
                let transform = self.compute_composed_transform(related_pd_id, &pd_to_nauos);

                // Triangulate the BREP
                let mesh = if let Some(shell_id) = self.find_shell_ref_by_brep_id(brep_id) {
                    if let Some(face_data_list) = self.extract_shell_faces(shell_id) {
                        let mut m = TriangleMesh::new();
                        for face_data in &face_data_list {
                            let face_mesh = self.surface_to_mesh(face_data, &params, &bbox);
                            m.merge(&face_mesh);
                        }
                        m
                    } else {
                        continue;
                    }
                } else {
                    continue;
                };

                // Apply the composed transform
                let mut mesh = mesh;
                if let Some(tf) = transform {
                    mesh.transform(&tf);
                }

                let color = color_map.get(&brep_id).copied();
                results.push(ColoredMesh { mesh, color });
            }

            if !results.is_empty() {
                info!("Assembly conversion: {} colored mesh instances", results.len());
                return Ok(results);
            }
        }

        // ─── Phase 2: No assembly structure — try direct BREP conversion ───
        let color_map = self.extract_color_map();

        let breps = self.step.find_entities_by_type("MANIFOLD_SOLID_BREP");
        if !breps.is_empty() {
            for brep in &breps {
                let shell_id = self.find_shell_ref(brep);
                if let Some(shell_id) = shell_id {
                    if let Some(face_data_list) = self.extract_shell_faces(shell_id) {
                        let mut mesh = TriangleMesh::new();
                        for face_data in &face_data_list {
                            let face_mesh = self.surface_to_mesh(face_data, &params, &bbox);
                            mesh.merge(&face_mesh);
                        }
                        let color = color_map.get(&brep.id).copied();
                        results.push(ColoredMesh { mesh, color });
                    }
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
                let shell_id = self.find_shell_ref(fb);
                if let Some(shell_id) = shell_id {
                    if let Some(face_data_list) = self.extract_shell_faces(shell_id) {
                        let mut mesh = TriangleMesh::new();
                        for face_data in &face_data_list {
                            let face_mesh = self.surface_to_mesh(face_data, &params, &bbox);
                            mesh.merge(&face_mesh);
                        }
                        let color = color_map.get(&fb.id).copied();
                        results.push(ColoredMesh { mesh, color });
                    }
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
                            let shell_id = self.find_shell_ref(entity);
                            if let Some(shell_id) = shell_id {
                                if let Some(face_data_list) = self.extract_shell_faces(shell_id) {
                                    let mut mesh = TriangleMesh::new();
                                    for face_data in &face_data_list {
                                        let face_mesh = self.surface_to_mesh(face_data, &params, &bbox);
                                        mesh.merge(&face_mesh);
                                    }
                                    let color = color_map.get(&entity.id).copied();
                                    results.push(ColoredMesh { mesh, color });
                                }
                            }
                        }
                    }
                }
                if let StepValue::List(items) = param {
                    for item in items {
                        if let Some(ref_id) = self.get_ref(item) {
                            if let Some(entity) = self.step.find_entity(ref_id) {
                                if entity.type_name == "MANIFOLD_SOLID_BREP" {
                                    let shell_id = self.find_shell_ref(entity);
                                    if let Some(shell_id) = shell_id {
                                        if let Some(face_data_list) = self.extract_shell_faces(shell_id) {
                                            let mut mesh = TriangleMesh::new();
                                            for face_data in &face_data_list {
                                                let face_mesh = self.surface_to_mesh(face_data, &params, &bbox);
                                                mesh.merge(&face_mesh);
                                            }
                                            let color = color_map.get(&entity.id).copied();
                                            results.push(ColoredMesh { mesh, color });
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
                    let face_data = FaceData { surface, edges: vec![], forward: true };
                    let mesh = self.surface_to_mesh(&face_data, &params, &bbox);
                    results.push(ColoredMesh { mesh, color: None });
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
                results.push(ColoredMesh { mesh, color: None });
                return Ok(results);
            }
        }

        Err("No convertible surface geometry found in STEP file".to_string())
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

    /// Compute the composed transform for a leaf product by walking up the assembly tree.
    fn compute_composed_transform(
        &self,
        child_pd_id: i64,
        pd_to_nauos: &HashMap<i64, Vec<(i64, i64)>>,
    ) -> Option<[[f64; 4]; 4]> {
        let mut transforms: Vec<[[f64; 4]; 4]> = Vec::new();
        let mut current_pd = child_pd_id;
        let mut visited = std::collections::HashSet::new();

        while let Some(nauo_list) = pd_to_nauos.get(&current_pd) {
            if visited.contains(&current_pd) {
                warn!("Circular assembly reference detected at PD {}", current_pd);
                break;
            }
            visited.insert(current_pd);

            if let Some(&(nauo_id, parent_pd)) = nauo_list.first() {
                if let Some(tf) = self.find_nauo_transform(nauo_id, current_pd) {
                    transforms.push(tf);
                }
                current_pd = parent_pd;
            } else {
                break;
            }
        }

        if transforms.is_empty() { return None; }

        let mut result = transforms[0];
        for tf in transforms.iter().skip(1) {
            result = mat4_mul(tf, &result);
        }
        Some(result)
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
                        if let Some(sr) = self.step.find_entity(sr_id) {
                            if sr.type_name == "ADVANCED_BREP_SHAPE_REPRESENTATION" {
                                for sp in &sr.params {
                                    if let Some(brep_id) = self.get_ref(sp) {
                                        if let Some(brep) = self.step.find_entity(brep_id) {
                                            if brep.type_name == "MANIFOLD_SOLID_BREP" {
                                                return Some(brep_id);
                                            }
                                        }
                                    }
                                }
                            }
                            if sr.type_name == "SHAPE_REPRESENTATION" {
                                for absr in self.step.find_entities_by_type("ADVANCED_BREP_SHAPE_REPRESENTATION") {
                                    for ap in &absr.params {
                                        if let Some(brep_id) = self.get_ref(ap) {
                                            if let Some(brep) = self.step.find_entity(brep_id) {
                                                if brep.type_name == "MANIFOLD_SOLID_BREP" {
                                                    if self.absr_belongs_to_pd(&absr, pd_id) {
                                                        return Some(brep_id);
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
                
                // Extract boundary edges
                let edges = self.extract_face_bounds(face_entity);

                // Extract face orientation (last param, typically .T. or .F.)
                let forward = self.extract_face_orientation(face_entity);

                Some(FaceData {
                    surface,
                    edges,
                    forward,
                })
            }
            _ => {
                // Try to extract directly as a surface (no boundary info)
                if let Some(surface) = self.extract_surface(face_id) {
                    Some(FaceData {
                        surface,
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
                    let is_surface = matches!(
                        entity.type_name.as_str(),
                        "PLANE" | "CYLINDRICAL_SURFACE" | "SPHERICAL_SURFACE" |
                        "CONICAL_SURFACE" | "TOROIDAL_SURFACE" |
                        "SURFACE_OF_REVOLUTION" | "SURFACE_OF_LINEAR_EXTRUSION" |
                        "B_SPLINE_SURFACE_WITH_KNOTS" | "B_SPLINE_SURFACE" |
                        "BEZIER_SURFACE" | "RECTANGULAR_TRIMMED_SURFACE" |
                        "OFFSET_SURFACE" | "SWEPT_SURFACE"
                    );
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
        let mut all_edges = Vec::new();

        // ADVANCED_FACE params: [name, (bounds_list), surface_ref, orientation]
        // The bounds are in params[1], which is a List of references to FACE_BOUND/FACE_OUTER_BOUND
        for param in &face.params {
            // Look for the bounds list — it's a StepValue::List containing references
            if let StepValue::List(items) = param {
                // Check if this list contains references to FACE_BOUND entities
                let mut found_bound = false;
                for item in items {
                    if let Some(bound_id) = self.get_ref(item) {
                        if let Some(bound_entity) = self.step.find_entity(bound_id) {
                            if bound_entity.type_name == "FACE_BOUND" 
                                || bound_entity.type_name == "FACE_OUTER_BOUND" 
                            {
                                found_bound = true;
                                if let Some(loop_edges) = self.resolve_face_bound(bound_entity) {
                                    all_edges.extend(loop_edges);
                                }
                            }
                        }
                    }
                }
                // If we found bounds in this list, don't process it again as a different list type
                if found_bound {
                    return all_edges;
                }
            }
        }

        all_edges
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
        match entity.type_name.as_str() {
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
    /// Format: #N = SURFACE_OF_LINEAR_EXTRUSION('', #profile_curve, #direction);
    fn extract_extrusion(&self, entity: &crate::schema::StepEntity) -> Option<Surface> {
        // Find the profile curve
        let profile_id = self.find_curve_ref(entity, 1)?;
        let profile = self.resolve_curve(profile_id)?;

        // Find the extrusion direction (3rd param, index 2)
        let dir_id = self.find_param_ref(entity, 2)?;
        let direction = self.resolve_direction(dir_id)?;

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
        // Degree u and v — use find_float_param to handle name prefix
        let u_degree = self.find_float_param(entity, 0).unwrap_or(1.0) as usize;
        let v_degree = self.find_float_param(entity, 1).unwrap_or(1.0) as usize;

        // Control points: search params for a list-of-lists that contains the control points
        let mut control_points: Vec<Vec<Point3d>> = Vec::new();
        for param in &entity.params {
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

        // Weights: default to 1.0 for each control point (rational = false)
        let n_u = control_points.len();
        let n_v = control_points[0].len();
        let weights = vec![vec![1.0; n_v]; n_u];

        // Find knot vectors — search remaining params for lists that could be knot vectors
        // The knot vectors are typically the 10th and 11th parameters for B_SPLINE_SURFACE_WITH_KNOTS
        // but for simpler B_SPLINE_SURFACE, they may be omitted
        let (u_knots, v_knots) = self.extract_bspline_knots(entity, n_u, n_v, u_degree, v_degree);

        Some(Surface::Nurbs(NurbsSurface {
            u_degree,
            v_degree,
            control_points,
            weights,
            u_knots,
            v_knots,
        }))
    }

    /// Extract knot vectors from a B_SPLINE_SURFACE_WITH_KNOTS entity.
    fn extract_bspline_knots(
        &self,
        entity: &crate::schema::StepEntity,
        n_u: usize,
        n_v: usize,
        u_degree: usize,
        v_degree: usize,
    ) -> (Vec<f64>, Vec<f64>) {
        // For B_SPLINE_SURFACE_WITH_KNOTS, the knot vectors are typically at
        // params[9] and params[10] (0-indexed), or params[10] and params[11]
        // depending on the exact format.
        // We'll search for them by looking for lists of numbers in the later params.

        let mut knot_lists: Vec<Vec<f64>> = Vec::new();
        for param in entity.params.iter().skip(3) {
            if let StepValue::List(items) = param {
                // Check if this looks like a knot vector (list of floats)
                let floats: Vec<f64> = items.iter()
                    .filter_map(|v| self.get_float(v))
                    .collect();
                if floats.len() >= 2 && floats.iter().all(|f| f.is_finite()) {
                    knot_lists.push(floats);
                }
            }
        }

        let u_knots = if knot_lists.len() >= 2 {
            // The two longest lists are likely the knot vectors
            knot_lists.sort_by(|a, b| b.len().cmp(&a.len()));
            let expected_u_knots = n_u + u_degree + 1;
            let _expected_v_knots = n_v + v_degree + 1;
            // Try to match by expected length
            if let Some(k) = knot_lists.iter().find(|l| l.len() == expected_u_knots) {
                k.clone()
            } else {
                knot_lists[0].clone()
            }
        } else {
            // Generate uniform knot vector
            let n = n_u + u_degree + 1;
            (0..n).map(|i| i as f64 / (n - 1).max(1) as f64).collect()
        };

        let v_knots = if knot_lists.len() >= 2 {
            let expected_v_knots = n_v + v_degree + 1;
            if let Some(k) = knot_lists.iter().find(|l| l.len() == expected_v_knots) {
                k.clone()
            } else if knot_lists.len() >= 2 {
                knot_lists[1].clone()
            } else {
                let n = n_v + v_degree + 1;
                (0..n).map(|i| i as f64 / (n - 1).max(1) as f64).collect()
            }
        } else {
            let n = n_v + v_degree + 1;
            (0..n).map(|i| i as f64 / (n - 1).max(1) as f64).collect()
        };

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
        )
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
        match entity.type_name.as_str() {
            "LINE" => self.resolve_line_curve(entity),
            "CIRCLE" => self.resolve_circle_curve(entity),
            "ELLIPSE" => self.resolve_ellipse_curve(entity),
            "B_SPLINE_CURVE_WITH_KNOTS" | "B_SPLINE_CURVE" | "BEZIER_CURVE" |
            "RATIONAL_B_SPLINE_CURVE" => self.resolve_bspline_curve(entity),
            "POLYLINE" => self.resolve_polyline_curve(entity),
            "TRIMMED_CURVE" => self.resolve_trimmed_curve(entity),
            "COMPOSITE_CURVE" => self.resolve_composite_curve(entity),
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

    /// Resolve a B_SPLINE_CURVE_WITH_KNOTS entity.
    fn resolve_bspline_curve(&self, entity: &crate::schema::StepEntity) -> Option<Curve3d> {
        let degree = self.get_float(entity.params.first()?).unwrap_or(1.0) as usize;

        // Control points: 2nd param is a list of points
        let mut control_points = Vec::new();
        if let Some(StepValue::List(items)) = entity.params.get(1) {
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

        // Default weights (uniform)
        let weights = vec![1.0; control_points.len()];

        // Extract knots from the STEP entity parameters
        let n = control_points.len();
        let knots = self.extract_curve_knots(entity, n, degree);

        Some(Curve3d::Nurbs(NurbsCurve {
            degree,
            control_points,
            weights,
            knots,
        }))
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

        // Collect 3D boundary points from edge curves by sampling each edge
        let mut boundary_points = Vec::new();
        for edge in &face_data.edges {
            for i in 0..64 {
                let t = i as f64 / 63.0;
                if let Some(p) = edge.point_at(t) {
                    boundary_points.push(p);
                }
            }
        }

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

/// Project two 3D points onto a circle and return the angular parameter range (t1, t2).
/// The angles are computed in the circle's local coordinate system.
/// t1 and t2 are in radians and the arc goes from t1 to t2 in the positive direction.
fn project_points_on_circle(circle: &Circle, p1: &Point3d, p2: &Point3d) -> (f64, f64) {
    let y_axis = circle.normal.cross(&circle.x_axis);

    let d1x = p1.x - circle.center.x;
    let d1y = p1.y - circle.center.y;
    let d1z = p1.z - circle.center.z;
    let local1_x = d1x * circle.x_axis.x + d1y * circle.x_axis.y + d1z * circle.x_axis.z;
    let local1_y = d1x * y_axis.x + d1y * y_axis.y + d1z * y_axis.z;
    let t1 = local1_y.atan2(local1_x);

    let d2x = p2.x - circle.center.x;
    let d2y = p2.y - circle.center.y;
    let d2z = p2.z - circle.center.z;
    let local2_x = d2x * circle.x_axis.x + d2y * circle.x_axis.y + d2z * circle.x_axis.z;
    let local2_y = d2x * y_axis.x + d2y * y_axis.y + d2z * y_axis.z;
    let t2 = local2_y.atan2(local2_x);

    // Ensure t2 > t1 (positive direction arc from t1 to t2)
    let t1 = t1;
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
