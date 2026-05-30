// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! STEP AP242 Product Manufacturing Information (PMI) extraction.
//!
//! Supports:
//! - 5.1.1 Tessellated geometry (TESSELLATED_SHAPE_REPRESENTATION, TRIANGULATED_FACE)
//! - 5.1.2 PMI (PRODUCT_DEFINITION_FORMATION, DRAUGHTING_MODEL, annotations)
//! - 5.1.3 GD&T (GEOMETRIC_TOLERANCE, DATUM_FEATURE, DATUM_REFERENCE)
//! - 5.1.4 Colour and Layer (PRESENTATION_STYLE_ASSIGNMENT, STYLED_ITEM, layers)
//! - 5.1.5 Unit accuracy (LENGTH_UNIT, PLANE_ANGLE_UNIT, SOLID_ANGLE_UNIT, conversion factors)

use crate::schema::{StepEntity, StepFile, StepValue};
use draper_geometry::Point3d;
use std::collections::HashMap;

// ============================================================
// 5.1.1 Tessellated Geometry
// ============================================================

/// A tessellated face extracted from a TESSELLATED_SHAPE_REPRESENTATION.
///
/// AP242 allows pre-tessellated geometry via `TRIANGULATED_FACE` entities,
/// which provide direct triangle data in the STEP file without requiring
/// surface-to-mesh conversion.
#[derive(Clone, Debug)]
pub struct TessellatedFace {
    /// STEP entity ID of the TRIANGULATED_FACE.
    pub step_id: i64,
    /// Triangle indices (3 vertex indices per triangle).
    pub triangles: Vec<[u32; 3]>,
    /// Vertex positions.
    pub vertices: Vec<Point3d>,
    /// Face normal (if specified in the STEP entity).
    pub normal: Option<[f64; 3]>,
}

/// Result of tessellated geometry extraction from a STEP file.
#[derive(Clone, Debug, Default)]
pub struct TessellatedGeometry {
    /// All tessellated faces found.
    pub faces: Vec<TessellatedFace>,
    /// STEP entity IDs of TESSELLATED_SHAPE_REPRESENTATION entities found.
    pub representation_ids: Vec<i64>,
}

/// Extract tessellated geometry from a STEP file.
///
/// Parses `TESSELLATED_SHAPE_REPRESENTATION` and `TRIANGULATED_FACE` entities.
/// AP242 uses these for pre-tessellated models, providing direct triangle data
/// without needing surface-to-mesh conversion.
pub fn extract_tessellated_geometry(step_file: &StepFile) -> TessellatedGeometry {
    let mut result = TessellatedGeometry::default();

    // Find all TESSELLATED_SHAPE_REPRESENTATION entities
    for entity in step_file.find_entities_by_type("TESSELLATED_SHAPE_REPRESENTATION") {
        result.representation_ids.push(entity.id);

        // Walk the entity's parameter list to find TRIANGULATED_FACE references
        let face_refs = collect_all_refs(&entity.params);
        for &face_id in &face_refs {
            if let Some(face_entity) = step_file.find_entity(face_id) {
                let type_upper = face_entity.type_name.to_uppercase();
                if type_upper == "TRIANGULATED_FACE" {
                    if let Some(tess_face) = parse_triangulated_face(face_entity, step_file) {
                        result.faces.push(tess_face);
                    }
                }
            }
        }
    }

    // Also find standalone TRIANGULATED_FACE entities (not referenced by a TSR)
    for entity in step_file.find_entities_by_type("TRIANGULATED_FACE") {
        // Skip if already extracted
        if result.faces.iter().any(|f| f.step_id == entity.id) {
            continue;
        }
        if let Some(tess_face) = parse_triangulated_face(entity, step_file) {
            result.faces.push(tess_face);
        }
    }

    result
}

/// Parse a TRIANGULATED_FACE entity.
///
/// STEP AP242 format:
/// ```text
/// #N = TRIANGULATED_FACE('name', #coord_list_ref, #normal_ref, (triangle_indices), #brep_face_ref);
/// ```
/// Where:
/// - coord_list_ref → Coordinates of vertices
/// - triangle_indices → Flat list of vertex indices (3 per triangle)
fn parse_triangulated_face(entity: &StepEntity, step_file: &StepFile) -> Option<TessellatedFace> {
    let step_id = entity.id;

    // Try to find coordinate list reference (typically first or second ref param)
    let refs: Vec<i64> = entity.params.iter()
        .filter_map(|p| match p {
            StepValue::Ref(id) => Some(*id),
            _ => None,
        })
        .collect();

    // Extract vertices from referenced coordinate list
    let mut vertices = Vec::new();
    for &ref_id in &refs {
        if let Some(coord_entity) = step_file.find_entity(ref_id) {
            if coord_entity.type_name.to_uppercase().contains("COORDINATE") {
                vertices = extract_coordinate_points(&coord_entity.params);
                if !vertices.is_empty() {
                    break;
                }
            }
        }
    }

    // Extract triangle indices from parameter lists
    let mut triangles = Vec::new();
    for param in &entity.params {
        if let StepValue::List(items) = param {
            // Check if this looks like a triangle index list (all integers)
            let ints: Vec<u32> = items.iter()
                .filter_map(|v| match v {
                    StepValue::Integer(i) => Some(*i as u32),
                    StepValue::Float(f) => Some(*f as u32),
                    _ => None,
                })
                .collect();

            if ints.len() >= 3 && ints.len() % 3 == 0 && ints.iter().all(|&i| i < 1_000_000) {
                for chunk in ints.chunks(3) {
                    triangles.push([chunk[0], chunk[1], chunk[2]]);
                }
            }
        }
    }

    // Extract face normal if present
    let normal = extract_face_normal(entity, step_file);

    Some(TessellatedFace {
        step_id,
        triangles,
        vertices,
        normal,
    })
}

/// Extract 3D points from coordinate list parameters.
fn extract_coordinate_points(params: &[StepValue]) -> Vec<Point3d> {
    let mut points = Vec::new();
    for param in params {
        match param {
            StepValue::List(items) => {
                // Could be a list of coordinate lists: ((x1,y1,z1), (x2,y2,z2), ...)
                if items.iter().any(|i| matches!(i, StepValue::List(_))) {
                    for item in items {
                        if let StepValue::List(coords) = item {
                            let nums: Vec<f64> = coords.iter()
                                .filter_map(|v| match v {
                                    StepValue::Float(f) => Some(*f),
                                    StepValue::Integer(i) => Some(*i as f64),
                                    _ => None,
                                })
                                .collect();
                            if nums.len() >= 3 {
                                points.push(Point3d::new(nums[0], nums[1], nums[2]));
                            }
                        }
                    }
                } else {
                    // Flat coordinate list: (x1,y1,z1,x2,y2,z2,...)
                    let nums: Vec<f64> = items.iter()
                        .filter_map(|v| match v {
                            StepValue::Float(f) => Some(*f),
                            StepValue::Integer(i) => Some(*i as f64),
                            _ => None,
                        })
                        .collect();
                    for chunk in nums.chunks(3) {
                        if chunk.len() == 3 {
                            points.push(Point3d::new(chunk[0], chunk[1], chunk[2]));
                        }
                    }
                }
            }
            _ => {}
        }
    }
    points
}

/// Extract the face normal from a TRIANGULATED_FACE entity if available.
fn extract_face_normal(entity: &StepEntity, step_file: &StepFile) -> Option<[f64; 3]> {
    // Look for a DIRECTION reference in the parameters
    for param in &entity.params {
        if let StepValue::Ref(ref_id) = param {
            if let Some(dir_entity) = step_file.find_entity(*ref_id) {
                if dir_entity.type_name.to_uppercase() == "DIRECTION" {
                    let coords = extract_float_list(&dir_entity.params);
                    if coords.len() >= 3 {
                        let len = (coords[0] * coords[0] + coords[1] * coords[1] + coords[2] * coords[2]).sqrt();
                        if len > 1e-10 {
                            return Some([coords[0] / len, coords[1] / len, coords[2] / len]);
                        }
                    }
                }
            }
        }
    }
    None
}

// ============================================================
// 5.1.2 PMI (Product Manufacturing Information)
// ============================================================

/// A PMI annotation extracted from a STEP file.
///
/// PMI includes dimensions, tolerances, notes, and other manufacturing
/// information that is associated with the geometry but not part of the
/// shape itself.
#[derive(Clone, Debug)]
pub struct PmiAnnotation {
    /// STEP entity ID of the source entity.
    pub step_id: i64,
    /// Type of PMI annotation.
    pub annotation_type: PmiAnnotationType,
    /// Human-readable name/label.
    pub name: String,
    /// Description text.
    pub description: String,
    /// Associated shape (STEP entity ID of the geometric entity this PMI refers to).
    pub associated_shape_id: Option<i64>,
    /// Numeric value (e.g., dimension value, tolerance value).
    pub value: Option<f64>,
    /// Unit string (e.g., "mm", "deg").
    pub unit: Option<String>,
}

/// Type of PMI annotation.
#[derive(Clone, Debug, PartialEq)]
pub enum PmiAnnotationType {
    /// A dimensional annotation (e.g., length, diameter).
    Dimension,
    /// A tolerance annotation.
    Tolerance,
    /// A general note or text annotation.
    Note,
    /// A surface finish annotation.
    SurfaceFinish,
    /// A datum reference.
    Datum,
    /// An unknown or unsupported PMI type.
    Other(String),
}

/// Container for all PMI data extracted from a STEP file.
#[derive(Clone, Debug, Default)]
pub struct PmiData {
    /// All PMI annotations.
    pub annotations: Vec<PmiAnnotation>,
    /// STEP entity IDs of DRAUGHTING_MODEL entities found.
    pub draughting_model_ids: Vec<i64>,
    /// STEP entity IDs of PRODUCT_DEFINITION_FORMATION entities found.
    pub product_definition_formation_ids: Vec<i64>,
}

/// Extract PMI (Product Manufacturing Information) from a STEP file.
///
/// Parses:
/// - `PRODUCT_DEFINITION_FORMATION` entities
/// - `DRAUGHTING_MODEL` entities
/// - Dimension annotations (`DIMENSIONAL_SIZE`, `DIMENSIONAL_LOCATION`)
/// - Tolerance annotations
/// - Text annotations (`ANNOTATION_TEXT`, `ANNOTATION_OCCURRENCE`)
pub fn extract_pmi(step_file: &StepFile) -> PmiData {
    let mut pmi = PmiData::default();

    // Extract PRODUCT_DEFINITION_FORMATION entities
    for entity in step_file.find_entities_by_type("PRODUCT_DEFINITION_FORMATION") {
        pmi.product_definition_formation_ids.push(entity.id);

        // Extract formation name/description
        let name = extract_string_param(&entity.params, 0).unwrap_or_default();
        let description = extract_string_param(&entity.params, 1).unwrap_or_default();

        if !name.is_empty() || !description.is_empty() {
            pmi.annotations.push(PmiAnnotation {
                step_id: entity.id,
                annotation_type: PmiAnnotationType::Note,
                name,
                description,
                associated_shape_id: None,
                value: None,
                unit: None,
            });
        }
    }

    // Extract DRAUGHTING_MODEL entities
    for entity in step_file.find_entities_by_type("DRAUGHTING_MODEL") {
        pmi.draughting_model_ids.push(entity.id);

        // Walk the model's items to find annotation entities
        let item_refs = collect_all_refs(&entity.params);
        for &item_id in &item_refs {
            if let Some(item_entity) = step_file.find_entity(item_id) {
                extract_annotation_from_entity(item_entity, step_file, &mut pmi);
            }
        }
    }

    // Also look for standalone annotation entities
    for entity in step_file.find_entities_by_type("DIMENSIONAL_SIZE") {
        let name = extract_string_param(&entity.params, 0).unwrap_or_default();
        let shape_ref = entity.params.iter()
            .filter_map(|p| match p { StepValue::Ref(id) => Some(*id), _ => None })
            .nth(1); // Second reference is typically the shape

        pmi.annotations.push(PmiAnnotation {
            step_id: entity.id,
            annotation_type: PmiAnnotationType::Dimension,
            name,
            description: String::new(),
            associated_shape_id: shape_ref,
            value: None,
            unit: None,
        });
    }

    for entity in step_file.find_entities_by_type("DIMENSIONAL_LOCATION") {
        let name = extract_string_param(&entity.params, 0).unwrap_or_default();
        let shape_ref = entity.params.iter()
            .filter_map(|p| match p { StepValue::Ref(id) => Some(*id), _ => None })
            .nth(1);

        pmi.annotations.push(PmiAnnotation {
            step_id: entity.id,
            annotation_type: PmiAnnotationType::Dimension,
            name,
            description: String::new(),
            associated_shape_id: shape_ref,
            value: None,
            unit: None,
        });
    }

    // Extract annotation text occurrences
    for entity in step_file.find_entities_by_type("ANNOTATION_TEXT") {
        let name = extract_string_param(&entity.params, 0).unwrap_or_default();
        let text_content = extract_string_param(&entity.params, 1).unwrap_or_default();

        pmi.annotations.push(PmiAnnotation {
            step_id: entity.id,
            annotation_type: PmiAnnotationType::Note,
            name,
            description: text_content,
            associated_shape_id: None,
            value: None,
            unit: None,
        });
    }

    // Extract annotation occurrences (which link annotations to geometry)
    for entity in step_file.find_entities_by_type("ANNOTATION_OCCURRENCE") {
        let name = extract_string_param(&entity.params, 0).unwrap_or_default();
        let style_ref = entity.params.iter()
            .filter_map(|p| match p { StepValue::Ref(id) => Some(*id), _ => None })
            .next();

        pmi.annotations.push(PmiAnnotation {
            step_id: entity.id,
            annotation_type: PmiAnnotationType::Note,
            name,
            description: String::new(),
            associated_shape_id: style_ref,
            value: None,
            unit: None,
        });
    }

    pmi
}

/// Extract annotation data from an entity found inside a DRAUGHTING_MODEL.
fn extract_annotation_from_entity(entity: &StepEntity, _step_file: &StepFile, pmi: &mut PmiData) {
    let type_upper = entity.type_name.to_uppercase();

    match type_upper.as_str() {
        "DIMENSIONAL_SIZE" | "DIMENSIONAL_LOCATION" => {
            let name = extract_string_param(&entity.params, 0).unwrap_or_default();
            let shape_ref = entity.params.iter()
                .filter_map(|p| match p { StepValue::Ref(id) => Some(*id), _ => None })
                .nth(1);

            pmi.annotations.push(PmiAnnotation {
                step_id: entity.id,
                annotation_type: PmiAnnotationType::Dimension,
                name,
                description: String::new(),
                associated_shape_id: shape_ref,
                value: None,
                unit: None,
            });
        }
        "ANNOTATION_TEXT" | "ANNOTATION_OCCURRENCE" => {
            let name = extract_string_param(&entity.params, 0).unwrap_or_default();
            let desc = extract_string_param(&entity.params, 1).unwrap_or_default();

            pmi.annotations.push(PmiAnnotation {
                step_id: entity.id,
                annotation_type: PmiAnnotationType::Note,
                name,
                description: desc,
                associated_shape_id: None,
                value: None,
                unit: None,
            });
        }
        _ => {
            // Check for tolerance-related entities
            if type_upper.contains("TOLERANCE") {
                let name = extract_string_param(&entity.params, 0).unwrap_or_default();
                let value = entity.params.iter()
                    .filter_map(|p| match p {
                        StepValue::Float(f) if *f > 0.0 && *f < 1e6 => Some(*f),
                        _ => None,
                    })
                    .next();

                pmi.annotations.push(PmiAnnotation {
                    step_id: entity.id,
                    annotation_type: PmiAnnotationType::Tolerance,
                    name,
                    description: String::new(),
                    associated_shape_id: None,
                    value,
                    unit: None,
                });
            }
        }
    }
}

// ============================================================
// 5.1.3 GD&T (Geometric Dimensioning and Tolerancing)
// ============================================================

/// A geometric tolerance extracted from a STEP file.
#[derive(Clone, Debug)]
pub struct GeometricTolerance {
    /// STEP entity ID.
    pub step_id: i64,
    /// Tolerance name/label.
    pub name: String,
    /// Description.
    pub description: String,
    /// Tolerance value.
    pub tolerance_value: Option<f64>,
    /// Datum references (STEP entity IDs of DATUM_FEATURE entities).
    pub datum_references: Vec<i64>,
    /// Type of geometric tolerance.
    pub tolerance_type: GdtToleranceType,
    /// The shape (entity ID) this tolerance applies to.
    pub applied_to: Option<i64>,
}

/// Type of geometric tolerance (GD&T symbols).
#[derive(Clone, Debug, PartialEq)]
pub enum GdtToleranceType {
    /// Position tolerance.
    Position,
    /// Flatness tolerance.
    Flatness,
    /// Straightness tolerance.
    Straightness,
    /// Circularity / Roundness tolerance.
    Circularity,
    /// Cylindricity tolerance.
    Cylindricity,
    /// Perpendicularity tolerance.
    Perpendicularity,
    /// Parallelism tolerance.
    Parallelism,
    /// Angularity tolerance.
    Angularity,
    /// Concentricity tolerance.
    Concentricity,
    /// Symmetry tolerance.
    Symmetry,
    /// Runout tolerance (circular or total).
    Runout,
    /// Profile of a line tolerance.
    ProfileOfLine,
    /// Profile of a surface tolerance.
    ProfileOfSurface,
    /// Unknown or unsupported tolerance type.
    Other(String),
}

impl GdtToleranceType {
    /// Determine the tolerance type from a STEP entity type name.
    pub fn from_step_type(type_name: &str) -> Self {
        let upper = type_name.to_uppercase();
        if upper.contains("POSITION") {
            GdtToleranceType::Position
        } else if upper.contains("FLATNESS") {
            GdtToleranceType::Flatness
        } else if upper.contains("STRAIGHTNESS") {
            GdtToleranceType::Straightness
        } else if upper.contains("CIRCULARITY") || upper.contains("ROUNDNESS") {
            GdtToleranceType::Circularity
        } else if upper.contains("CYLINDRICITY") {
            GdtToleranceType::Cylindricity
        } else if upper.contains("PERPENDICULARITY") {
            GdtToleranceType::Perpendicularity
        } else if upper.contains("PARALLELISM") {
            GdtToleranceType::Parallelism
        } else if upper.contains("ANGULARITY") {
            GdtToleranceType::Angularity
        } else if upper.contains("CONCENTRICITY") {
            GdtToleranceType::Concentricity
        } else if upper.contains("SYMMETRY") {
            GdtToleranceType::Symmetry
        } else if upper.contains("RUNOUT") {
            GdtToleranceType::Runout
        } else if upper.contains("PROFILE_OF_LINE") {
            GdtToleranceType::ProfileOfLine
        } else if upper.contains("PROFILE_OF_SURFACE") {
            GdtToleranceType::ProfileOfSurface
        } else {
            GdtToleranceType::Other(type_name.to_string())
        }
    }
}

/// A datum feature extracted from a STEP file.
#[derive(Clone, Debug)]
pub struct DatumFeature {
    /// STEP entity ID.
    pub step_id: i64,
    /// Datum name/label (e.g., "A", "B", "C").
    pub name: String,
    /// Description.
    pub description: String,
    /// The shape (entity ID) this datum is applied to.
    pub applied_to: Option<i64>,
}

/// A datum reference (a reference to a datum in a tolerance).
#[derive(Clone, Debug)]
pub struct DatumReference {
    /// STEP entity ID.
    pub step_id: i64,
    /// Name/label.
    pub name: String,
    /// Referenced datum feature entity ID.
    pub datum_feature_id: Option<i64>,
    /// Modifier (e.g., "M" for maximum material condition).
    pub modifier: Option<String>,
}

/// Container for all GD&T data extracted from a STEP file.
#[derive(Clone, Debug, Default)]
pub struct GdtData {
    /// All geometric tolerances.
    pub tolerances: Vec<GeometricTolerance>,
    /// All datum features.
    pub datum_features: Vec<DatumFeature>,
    /// All datum references.
    pub datum_references: Vec<DatumReference>,
}

/// Extract GD&T (Geometric Dimensioning and Tolerancing) data from a STEP file.
///
/// Parses:
/// - `GEOMETRIC_TOLERANCE` and its subtypes (position, flatness, etc.)
/// - `DATUM_FEATURE` entities
/// - `DATUM_REFERENCE` entities
/// - `GEOMETRIC_TOLERANCE_WITH_DATUM_REFERENCE` entities
pub fn extract_gdt(step_file: &StepFile) -> GdtData {
    let mut gdt = GdtData::default();

    for entity in &step_file.entities {
        let type_upper = entity.type_name.to_uppercase();

        // Parse GEOMETRIC_TOLERANCE and all subtypes
        if type_upper.starts_with("GEOMETRIC_TOLERANCE") {
            let name = extract_string_param(&entity.params, 0).unwrap_or_default();
            let description = extract_string_param(&entity.params, 1).unwrap_or_default();

            // Tolerance value — typically the first numeric parameter after strings
            let tolerance_value = entity.params.iter()
                .filter_map(|p| match p {
                    StepValue::Float(f) if *f > 0.0 && *f < 1e6 => Some(*f),
                    _ => None,
                })
                .next();

            // Datum references — references in the entity params
            let datum_refs: Vec<i64> = entity.params.iter()
                .filter_map(|p| match p {
                    StepValue::Ref(id) => Some(*id),
                    _ => None,
                })
                .collect();

            // The applied-to shape is typically one of the references
            let applied_to = datum_refs.last().copied();

            gdt.tolerances.push(GeometricTolerance {
                step_id: entity.id,
                name,
                description,
                tolerance_value,
                datum_references: datum_refs,
                tolerance_type: GdtToleranceType::from_step_type(&entity.type_name),
                applied_to,
            });
        }

        // Parse specific tolerance subtypes by their AP242 names
        if type_upper.contains("POSITION_TOLERANCE")
            || type_upper.contains("FLATNESS_TOLERANCE")
            || type_upper.contains("STRAIGHTNESS_TOLERANCE")
            || type_upper.contains("CIRCULARITY_TOLERANCE")
            || type_upper.contains("CYLINDRICITY_TOLERANCE")
            || type_upper.contains("PERPENDICULARITY_TOLERANCE")
            || type_upper.contains("PARALLELISM_TOLERANCE")
            || type_upper.contains("ANGULARITY_TOLERANCE")
            || type_upper.contains("CONCENTRICITY_TOLERANCE")
            || type_upper.contains("SYMMETRY_TOLERANCE")
            || type_upper.contains("CIRCULAR_RUNOUT_TOLERANCE")
            || type_upper.contains("TOTAL_RUNOUT_TOLERANCE")
            || type_upper.contains("LINE_PROFILE_TOLERANCE")
            || type_upper.contains("SURFACE_PROFILE_TOLERANCE")
        {
            // Skip if already captured as a generic GEOMETRIC_TOLERANCE
            if gdt.tolerances.iter().any(|t| t.step_id == entity.id) {
                continue;
            }

            let name = extract_string_param(&entity.params, 0).unwrap_or_default();
            let description = extract_string_param(&entity.params, 1).unwrap_or_default();
            let tolerance_value = entity.params.iter()
                .filter_map(|p| match p {
                    StepValue::Float(f) if *f > 0.0 && *f < 1e6 => Some(*f),
                    _ => None,
                })
                .next();
            let datum_refs: Vec<i64> = entity.params.iter()
                .filter_map(|p| match p { StepValue::Ref(id) => Some(*id), _ => None })
                .collect();
            let applied_to = datum_refs.last().copied();

            gdt.tolerances.push(GeometricTolerance {
                step_id: entity.id,
                name,
                description,
                tolerance_value,
                datum_references: datum_refs,
                tolerance_type: GdtToleranceType::from_step_type(&entity.type_name),
                applied_to,
            });
        }

        // Parse DATUM_FEATURE
        if type_upper == "DATUM_FEATURE" {
            let name = extract_string_param(&entity.params, 0).unwrap_or_default();
            let description = extract_string_param(&entity.params, 1).unwrap_or_default();
            let applied_to = entity.params.iter()
                .filter_map(|p| match p { StepValue::Ref(id) => Some(*id), _ => None })
                .next();

            gdt.datum_features.push(DatumFeature {
                step_id: entity.id,
                name,
                description,
                applied_to,
            });
        }

        // Parse DATUM_REFERENCE
        if type_upper == "DATUM_REFERENCE" {
            let name = extract_string_param(&entity.params, 0).unwrap_or_default();
            let datum_feature_id = entity.params.iter()
                .filter_map(|p| match p { StepValue::Ref(id) => Some(*id), _ => None })
                .next();

            // Check for modifier enum (e.g., .M., .L., .F.)
            let modifier = entity.params.iter()
                .filter_map(|p| match p {
                    StepValue::Enum(e) => Some(e.clone()),
                    _ => None,
                })
                .next();

            gdt.datum_references.push(DatumReference {
                step_id: entity.id,
                name,
                datum_feature_id,
                modifier,
            });
        }
    }

    gdt
}

// ============================================================
// 5.1.4 Colour and Layer
// ============================================================

/// A colour extracted from a STEP file.
#[derive(Clone, Debug)]
pub struct StepColour {
    /// STEP entity ID of the colour entity.
    pub step_id: i64,
    /// RGB colour values (0..1 range).
    pub rgb: [f64; 3],
    /// Colour name (if specified).
    pub name: String,
}

/// A layer assignment extracted from a STEP file.
#[derive(Clone, Debug)]
pub struct StepLayer {
    /// Layer name.
    pub name: String,
    /// Description.
    pub description: String,
    /// STEP entity IDs of items assigned to this layer.
    pub item_ids: Vec<i64>,
}

/// A colour assignment to a shape.
#[derive(Clone, Debug)]
pub struct ColourAssignment {
    /// The STEP entity ID of the shape (ADVANCED_FACE, MANIFOLD_SOLID_BREP, etc.).
    pub shape_id: i64,
    /// The RGB colour assigned.
    pub colour: [f64; 3],
}

/// A layer assignment to a shape.
#[derive(Clone, Debug)]
pub struct LayerAssignment {
    /// The STEP entity ID of the shape.
    pub shape_id: i64,
    /// The layer name.
    pub layer_name: String,
}

/// Container for all colour and layer data extracted from a STEP file.
#[derive(Clone, Debug, Default)]
pub struct ColourLayerData {
    /// All defined colours.
    pub colours: Vec<StepColour>,
    /// All defined layers.
    pub layers: Vec<StepLayer>,
    /// Colour assignments (shape ID → colour).
    pub colour_assignments: Vec<ColourAssignment>,
    /// Layer assignments (shape ID → layer name).
    pub layer_assignments: Vec<LayerAssignment>,
}

/// Extract colour and layer information from a STEP file.
///
/// Parses:
/// - `PRESENTATION_STYLE_ASSIGNMENT` entities
/// - `STYLED_ITEM` / `OVER_RIDING_STYLED_ITEM` entities
/// - `FILL_AREA_STYLE_COLOUR` entities
/// - `COLOUR_RGB` entities
/// - `LAYERED_ITEM` / `PRESENTATION_LAYER_ASSIGNMENT` entities
pub fn extract_colour_and_layer(step_file: &StepFile) -> ColourLayerData {
    let mut data = ColourLayerData::default();

    // Map: colour entity ID → RGB
    let mut colour_id_to_rgb: HashMap<i64, [f64; 3]> = HashMap::new();

    // Phase 1: Extract COLOUR_RGB entities (define named colours)
    for entity in step_file.find_entities_by_type("COLOUR_RGB") {
        let name = extract_string_param(&entity.params, 0).unwrap_or_default();
        let floats: Vec<f64> = entity.params.iter()
            .filter_map(|p| match p {
                StepValue::Float(f) => Some(*f),
                _ => None,
            })
            .collect();

        let rgb = if floats.len() >= 3 {
            [floats[0].clamp(0.0, 1.0), floats[1].clamp(0.0, 1.0), floats[2].clamp(0.0, 1.0)]
        } else {
            [0.5, 0.5, 0.5] // Default grey
        };

        colour_id_to_rgb.insert(entity.id, rgb);
        data.colours.push(StepColour {
            step_id: entity.id,
            rgb,
            name,
        });
    }

    // Phase 2: Extract FILL_AREA_STYLE_COLOUR entities (references a colour)
    for entity in step_file.find_entities_by_type("FILL_AREA_STYLE_COLOUR") {
        let name = extract_string_param(&entity.params, 0).unwrap_or_default();
        let colour_ref = entity.params.iter()
            .filter_map(|p| match p { StepValue::Ref(id) => Some(*id), _ => None })
            .next();

        if let Some(colour_id) = colour_ref {
            // Look up the referenced colour
            let rgb = colour_id_to_rgb.get(&colour_id).copied().unwrap_or([0.5, 0.5, 0.5]);
            colour_id_to_rgb.insert(entity.id, rgb);
            data.colours.push(StepColour {
                step_id: entity.id,
                rgb,
                name,
            });
        }
    }

    // Phase 3: Extract PRESENTATION_STYLE_ASSIGNMENT entities
    for entity in step_file.find_entities_by_type("PRESENTATION_STYLE_ASSIGNMENT") {
        // Style assignments reference curve/point/surface styles which reference colours
        let style_refs = collect_all_refs(&entity.params);
        for &style_id in &style_refs {
            if let Some(style_entity) = step_file.find_entity(style_id) {
                let inner_refs = collect_all_refs(&style_entity.params);
                for &inner_id in &inner_refs {
                    if let Some(&rgb) = colour_id_to_rgb.get(&inner_id) {
                        colour_id_to_rgb.insert(entity.id, rgb);
                    }
                }
            }
        }
    }

    // Phase 4: Extract STYLED_ITEM entities (link colours to shapes)
    for entity in step_file.find_entities_by_type("STYLED_ITEM") {
        let name = extract_string_param(&entity.params, 0).unwrap_or_default();
        let refs: Vec<i64> = entity.params.iter()
            .filter_map(|p| match p { StepValue::Ref(id) => Some(*id), _ => None })
            .collect();

        // STYLED_ITEM format: ('name', #style, #item)
        // First ref is style, second ref is the shape item
        if refs.len() >= 2 {
            let style_id = refs[0];
            let item_id = refs[1];

            // Try to find the colour for this style
            if let Some(&rgb) = colour_id_to_rgb.get(&style_id) {
                data.colour_assignments.push(ColourAssignment {
                    shape_id: item_id,
                    colour: rgb,
                });
                // Also map the styled_item entity itself
                colour_id_to_rgb.insert(entity.id, rgb);
            } else {
                // Walk the style chain to find a colour
                let found_colour = resolve_colour_chain(style_id, step_file, &colour_id_to_rgb);
                if let Some(rgb) = found_colour {
                    colour_id_to_rgb.insert(entity.id, rgb);
                    data.colour_assignments.push(ColourAssignment {
                        shape_id: item_id,
                        colour: rgb,
                    });
                }
            }
        }
        let _ = name; // name is available for debugging
    }

    // OVER_RIDING_STYLED_ITEM (overrides colour for specific aspects)
    for entity in step_file.find_entities_by_type("OVER_RIDING_STYLED_ITEM") {
        let refs: Vec<i64> = entity.params.iter()
            .filter_map(|p| match p { StepValue::Ref(id) => Some(*id), _ => None })
            .collect();

        if refs.len() >= 2 {
            let style_id = refs[0];
            let item_id = refs[1];

            let found_colour = colour_id_to_rgb.get(&style_id).copied()
                .or_else(|| resolve_colour_chain(style_id, step_file, &colour_id_to_rgb));

            if let Some(rgb) = found_colour {
                colour_id_to_rgb.insert(entity.id, rgb);
                data.colour_assignments.push(ColourAssignment {
                    shape_id: item_id,
                    colour: rgb,
                });
            }
        }
    }

    // Phase 5: Extract layer information
    for entity in step_file.find_entities_by_type("PRESENTATION_LAYER_ASSIGNMENT") {
        let name = extract_string_param(&entity.params, 0).unwrap_or_default();
        let description = extract_string_param(&entity.params, 1).unwrap_or_default();
        let item_refs = collect_all_refs(&entity.params);

        // Skip the first refs that were name/description references
        // The actual items are typically the Ref values in the parameter list
        data.layers.push(StepLayer {
            name,
            description,
            item_ids: item_refs,
        });
    }

    for entity in step_file.find_entities_by_type("LAYERED_ITEM") {
        // LAYERED_ITEM links an item to a layer assignment
        let refs: Vec<i64> = entity.params.iter()
            .filter_map(|p| match p { StepValue::Ref(id) => Some(*id), _ => None })
            .collect();

        if refs.len() >= 2 {
            let layer_assignment_id = refs[0];
            let item_id = refs[1];

            // Find the layer name
            if let Some(layer_entity) = step_file.find_entity(layer_assignment_id) {
                let layer_name = extract_string_param(&layer_entity.params, 0)
                    .unwrap_or_else(|| format!("Layer_{}", layer_assignment_id));

                data.layer_assignments.push(LayerAssignment {
                    shape_id: item_id,
                    layer_name,
                });
            }
        }
    }

    data
}

/// Try to resolve a colour by walking the reference chain.
fn resolve_colour_chain(
    start_id: i64,
    step_file: &StepFile,
    colour_map: &HashMap<i64, [f64; 3]>,
) -> Option<[f64; 3]> {
    let mut visited = std::collections::HashSet::new();
    let mut current_id = start_id;

    for _ in 0..20 { // Max chain length to prevent infinite loops
        if visited.contains(&current_id) {
            break;
        }
        visited.insert(current_id);

        if let Some(&rgb) = colour_map.get(&current_id) {
            return Some(rgb);
        }

        if let Some(entity) = step_file.find_entity(current_id) {
            let refs: Vec<i64> = entity.params.iter()
                .filter_map(|p| match p { StepValue::Ref(id) => Some(*id), _ => None })
                .collect();

            let mut found_next = false;
            for &ref_id in &refs {
                if colour_map.contains_key(&ref_id) {
                    return colour_map.get(&ref_id).copied();
                }
                // Try following the chain
                if step_file.find_entity(ref_id).is_some() && !visited.contains(&ref_id) {
                    current_id = ref_id;
                    found_next = true;
                    break;
                }
            }
            if !found_next {
                break;
            }
        } else {
            break;
        }
    }
    None
}

// ============================================================
// 5.1.5 Unit Accuracy
// ============================================================

/// Unit information extracted from a STEP file.
#[derive(Clone, Debug)]
pub struct UnitInfo {
    /// Length unit name (e.g., "MILLIMETRE", "METRE", "INCH").
    pub length_unit: String,
    /// Plane angle unit name (e.g., "RADIAN", "DEGREE").
    pub plane_angle_unit: String,
    /// Solid angle unit name (e.g., "STERADIAN").
    pub solid_angle_unit: String,
    /// Conversion factor from the file's length unit to metres.
    /// Multiply file values by this factor to get SI metres.
    pub length_to_si: f64,
    /// Conversion factor from the file's plane angle unit to radians.
    pub angle_to_si: f64,
    /// Conversion factor from the file's solid angle unit to steradians.
    pub solid_angle_to_si: f64,
}

impl Default for UnitInfo {
    fn default() -> Self {
        // STEP defaults: mm, radians, steradians
        Self {
            length_unit: "MILLIMETRE".to_string(),
            plane_angle_unit: "RADIAN".to_string(),
            solid_angle_unit: "STERADIAN".to_string(),
            length_to_si: 0.001, // mm → m
            angle_to_si: 1.0,    // radian → radian
            solid_angle_to_si: 1.0, // steradian → steradian
        }
    }
}

impl UnitInfo {
    /// Convert a length value from the file's units to SI metres.
    pub fn length_to_metres(&self, value: f64) -> f64 {
        value * self.length_to_si
    }

    /// Convert a length value from SI metres to the file's units.
    pub fn metres_to_length(&self, value: f64) -> f64 {
        value / self.length_to_si
    }

    /// Convert an angle value from the file's units to radians.
    pub fn angle_to_radians(&self, value: f64) -> f64 {
        value * self.angle_to_si
    }

    /// Convert a solid angle value from the file's units to steradians.
    pub fn solid_angle_to_steradians(&self, value: f64) -> f64 {
        value * self.solid_angle_to_si
    }

    /// Whether the file uses millimetre as length unit.
    pub fn uses_millimetres(&self) -> bool {
        self.length_unit.to_uppercase().contains("MILLIMETRE")
            || self.length_unit.to_uppercase().contains("MM")
    }

    /// Whether the file uses inches as length unit.
    pub fn uses_inches(&self) -> bool {
        self.length_unit.to_uppercase().contains("INCH")
            || self.length_unit.to_uppercase().contains("IN")
    }

    /// Whether the file uses degrees as angle unit.
    pub fn uses_degrees(&self) -> bool {
        self.plane_angle_unit.to_uppercase().contains("DEGREE")
            || self.plane_angle_unit.to_uppercase().contains("DEG")
    }
}

/// Container for all unit-related data.
#[derive(Clone, Debug, Default)]
pub struct UnitData {
    /// Extracted unit information.
    pub units: UnitInfo,
    /// STEP entity IDs of GLOBAL_UNIT_ASSIGNED_CONTEXT entities.
    pub context_ids: Vec<i64>,
    /// Any prefix conversion factors found (e.g., MILLI → 1e-3).
    pub prefix_factors: HashMap<String, f64>,
}

/// Extract unit information from a STEP file.
///
/// Parses:
/// - `LENGTH_UNIT`, `PLANE_ANGLE_UNIT`, `SOLID_ANGLE_UNIT` from HEADER/DATA
/// - `GLOBAL_UNIT_ASSIGNED_CONTEXT` entities
/// - SI prefix conversions (MILLI, CENTI, DECI, KILO, etc.)
///
/// Returns default units (mm, radian, steradian) if none are explicitly specified,
/// since STEP AP203/AP214/AP242 default to mm for length.
pub fn extract_units(step_file: &StepFile) -> UnitData {
    let mut unit_data = UnitData::default();
    let mut length_unit_id: Option<i64> = None;
    let mut angle_unit_id: Option<i64> = None;
    let mut solid_angle_unit_id: Option<i64> = None;

    // Map: unit entity ID → (unit_name, conversion_factor_to_si)
    let mut unit_map: HashMap<i64, (String, f64)> = HashMap::new();

    // Phase 1: Find all LENGTH_UNIT entities (which may be complex entities with SI_UNIT)
    for entity in &step_file.entities {
        let type_upper = entity.type_name.to_uppercase();

        // Simple LENGTH_UNIT
        if type_upper == "LENGTH_UNIT" {
            unit_map.insert(entity.id, ("LENGTH_UNIT".to_string(), 1.0));
            length_unit_id = Some(entity.id);
        }

        // Simple PLANE_ANGLE_UNIT
        if type_upper == "PLANE_ANGLE_UNIT" {
            unit_map.insert(entity.id, ("PLANE_ANGLE_UNIT".to_string(), 1.0));
            angle_unit_id = Some(entity.id);
        }

        // Simple SOLID_ANGLE_UNIT
        if type_upper == "SOLID_ANGLE_UNIT" {
            unit_map.insert(entity.id, ("SOLID_ANGLE_UNIT".to_string(), 1.0));
            solid_angle_unit_id = Some(entity.id);
        }

        // Complex entities: (LENGTH_UNIT() SI_UNIT(...) )
        // or (PLANE_ANGLE_UNIT() SI_UNIT(...) )
        if type_upper.contains("LENGTH_UNIT") && type_upper.contains("SI_UNIT") {
            let (name, factor) = parse_si_unit_from_complex(entity);
            unit_map.insert(entity.id, (name, factor));
            length_unit_id = Some(entity.id);
        }

        if type_upper.contains("PLANE_ANGLE_UNIT") && type_upper.contains("SI_UNIT") {
            let (name, factor) = parse_si_unit_from_complex(entity);
            unit_map.insert(entity.id, (name, factor));
            angle_unit_id = Some(entity.id);
        }

        if type_upper.contains("SOLID_ANGLE_UNIT") && type_upper.contains("SI_UNIT") {
            let (name, factor) = parse_si_unit_from_complex(entity);
            unit_map.insert(entity.id, (name, factor));
            solid_angle_unit_id = Some(entity.id);
        }

        // Standalone SI_UNIT
        if type_upper == "SI_UNIT" {
            let (name, factor) = parse_si_unit_params(entity);
            unit_map.insert(entity.id, (name, factor));
        }

        // CONVERSION_BASED_UNIT (e.g., INCH)
        if type_upper.contains("CONVERSION_BASED_UNIT") {
            let name = extract_string_param(&entity.params, 0).unwrap_or_default();
            // Conversion factor is typically referenced
            let refs: Vec<i64> = entity.params.iter()
                .filter_map(|p| match p { StepValue::Ref(id) => Some(*id), _ => None })
                .collect();

            let factor = if let Some(&conv_ref) = refs.first() {
                resolve_conversion_factor(conv_ref, step_file)
            } else {
                1.0
            };

            unit_map.insert(entity.id, (name, factor));

            if type_upper.contains("LENGTH") {
                length_unit_id = Some(entity.id);
            } else if type_upper.contains("PLANE_ANGLE") {
                angle_unit_id = Some(entity.id);
            } else if type_upper.contains("SOLID_ANGLE") {
                solid_angle_unit_id = Some(entity.id);
            }
        }
    }

    // Phase 2: Find GLOBAL_UNIT_ASSIGNED_CONTEXT entities
    for entity in step_file.find_entities_by_type("GLOBAL_UNIT_ASSIGNED_CONTEXT") {
        unit_data.context_ids.push(entity.id);

        // Extract the unit references from the parameter list
        let unit_refs = collect_all_refs(&entity.params);
        for (i, &unit_id) in unit_refs.iter().enumerate() {
            match i {
                0 => length_unit_id = Some(unit_id),
                1 => angle_unit_id = Some(unit_id),
                2 => solid_angle_unit_id = Some(unit_id),
                _ => {}
            }
        }
    }

    // Phase 3: Build the UnitInfo from collected data
    if let Some(id) = length_unit_id {
        if let Some((name, factor)) = unit_map.get(&id) {
            unit_data.units.length_unit = name.clone();
            unit_data.units.length_to_si = *factor;
        }
    }

    if let Some(id) = angle_unit_id {
        if let Some((name, factor)) = unit_map.get(&id) {
            unit_data.units.plane_angle_unit = name.clone();
            unit_data.units.angle_to_si = *factor;
        }
    }

    if let Some(id) = solid_angle_unit_id {
        if let Some((name, factor)) = unit_map.get(&id) {
            unit_data.units.solid_angle_unit = name.clone();
            unit_data.units.solid_angle_to_si = *factor;
        }
    }

    // Populate prefix factors
    unit_data.prefix_factors.insert("MILLI".to_string(), 1e-3);
    unit_data.prefix_factors.insert("CENTI".to_string(), 1e-2);
    unit_data.prefix_factors.insert("DECI".to_string(), 1e-1);
    unit_data.prefix_factors.insert("KILO".to_string(), 1e3);
    unit_data.prefix_factors.insert("MEGA".to_string(), 1e6);
    unit_data.prefix_factors.insert("MICRO".to_string(), 1e-6);

    unit_data
}

/// Parse SI unit information from a complex entity like `(LENGTH_UNIT() SI_UNIT(MILLI,METRE))`.
fn parse_si_unit_from_complex(entity: &StepEntity) -> (String, f64) {
    let mut unit_name = String::new();
    let mut prefix_factor = 1.0;

    // Check sub-entities for SI_UNIT
    for sub in &entity.sub_entities {
        if sub.type_name.to_uppercase() == "SI_UNIT" {
            let (name, factor) = parse_si_unit_params(sub);
            unit_name = name;
            prefix_factor = factor;
        }
    }

    // If no sub-entities, try to parse from combined params
    if unit_name.is_empty() {
        let (name, factor) = parse_si_unit_params(entity);
        unit_name = name;
        prefix_factor = factor;
    }

    // Determine the dimension type from the main entity type name
    let type_upper = entity.type_name.to_uppercase();
    let dimension = if type_upper.contains("LENGTH") {
        "LENGTH"
    } else if type_upper.contains("PLANE_ANGLE") {
        "ANGLE"
    } else if type_upper.contains("SOLID_ANGLE") {
        "SOLID_ANGLE"
    } else {
        ""
    };

    if !unit_name.is_empty() {
        (format!("{}_{}", dimension, unit_name), prefix_factor)
    } else {
        (dimension.to_string(), prefix_factor)
    }
}

/// Parse SI_UNIT parameters: prefix enum and unit name enum.
fn parse_si_unit_params(entity: &StepEntity) -> (String, f64) {
    let mut prefix = "";
    let mut unit_name = "";
    let mut prefix_factor = 1.0;

    for param in &entity.params {
        if let StepValue::Enum(e) = param {
            let upper = e.to_uppercase();
            match upper.as_str() {
                // SI prefixes
                "EXA" => { prefix = "EXA"; prefix_factor = 1e18; }
                "PETA" => { prefix = "PETA"; prefix_factor = 1e15; }
                "TERA" => { prefix = "TERA"; prefix_factor = 1e12; }
                "GIGA" => { prefix = "GIGA"; prefix_factor = 1e9; }
                "MEGA" => { prefix = "MEGA"; prefix_factor = 1e6; }
                "KILO" => { prefix = "KILO"; prefix_factor = 1e3; }
                "HECTO" => { prefix = "HECTO"; prefix_factor = 1e2; }
                "DECA" => { prefix = "DECA"; prefix_factor = 1e1; }
                "DECI" => { prefix = "DECI"; prefix_factor = 1e-1; }
                "CENTI" => { prefix = "CENTI"; prefix_factor = 1e-2; }
                "MILLI" => { prefix = "MILLI"; prefix_factor = 1e-3; }
                "MICRO" => { prefix = "MICRO"; prefix_factor = 1e-6; }
                "NANO" => { prefix = "NANO"; prefix_factor = 1e-9; }
                "PICO" => { prefix = "PICO"; prefix_factor = 1e-12; }
                "FEMTO" => { prefix = "FEMTO"; prefix_factor = 1e-15; }
                "ATTO" => { prefix = "ATTO"; prefix_factor = 1e-18; }
                // SI units
                "METRE" | "METER" => { unit_name = "METRE"; }
                "RADIAN" => { unit_name = "RADIAN"; prefix_factor = 1.0; }
                "STERADIAN" => { unit_name = "STERADIAN"; prefix_factor = 1.0; }
                "DEGREE" => { unit_name = "DEGREE"; prefix_factor = std::f64::consts::PI / 180.0; }
                "GRAM" => { unit_name = "GRAM"; }
                "SECOND" => { unit_name = "SECOND"; }
                "AMPERE" => { unit_name = "AMPERE"; }
                "KELVIN" => { unit_name = "KELVIN"; }
                "MOLE" => { unit_name = "MOLE"; }
                "CANDELA" => { unit_name = "CANDELA"; }
                _ => {}
            }
        }
    }

    if prefix.is_empty() {
        (unit_name.to_string(), prefix_factor)
    } else {
        (format!("{}_{}", prefix, unit_name), prefix_factor)
    }
}

/// Resolve a conversion factor from a CONVERSION_BASED_UNIT reference chain.
fn resolve_conversion_factor(entity_id: i64, step_file: &StepFile) -> f64 {
    let mut visited = std::collections::HashSet::new();
    let mut current_id = entity_id;

    for _ in 0..10 {
        if visited.contains(&current_id) {
            break;
        }
        visited.insert(current_id);

        if let Some(entity) = step_file.find_entity(current_id) {
            let type_upper = entity.type_name.to_uppercase();

            // MEASURE_WITH_UNIT contains the conversion factor
            if type_upper.contains("MEASURE_WITH_UNIT") {
                // The first numeric parameter is the conversion value
                for param in &entity.params {
                    if let StepValue::List(items) = param {
                        if let Some(StepValue::Float(f)) = items.first() {
                            return *f;
                        }
                        if let Some(StepValue::Integer(i)) = items.first() {
                            return *i as f64;
                        }
                    }
                    if let StepValue::Float(f) = param {
                        return *f;
                    }
                }
            }

            // Follow references
            let refs: Vec<i64> = entity.params.iter()
                .filter_map(|p| match p { StepValue::Ref(id) => Some(*id), _ => None })
                .collect();

            if let Some(&next_id) = refs.first() {
                current_id = next_id;
            } else {
                break;
            }
        } else {
            break;
        }
    }

    1.0 // Default: no conversion
}

// ============================================================
// Helper functions
// ============================================================

/// Extract a string parameter at a given index.
fn extract_string_param(params: &[StepValue], index: usize) -> Option<String> {
    params.get(index).and_then(|p| match p {
        StepValue::String(s) => Some(s.clone()),
        _ => None,
    })
}

/// Extract a flat list of float values from parameters.
fn extract_float_list(params: &[StepValue]) -> Vec<f64> {
    let mut result = Vec::new();
    for param in params {
        match param {
            StepValue::Float(f) => result.push(*f),
            StepValue::Integer(i) => result.push(*i as f64),
            StepValue::List(items) => {
                result.extend(extract_float_list(items));
            }
            _ => {}
        }
    }
    result
}

/// Recursively collect all entity references from a parameter list.
fn collect_all_refs(params: &[StepValue]) -> Vec<i64> {
    let mut refs = Vec::new();
    for param in params {
        match param {
            StepValue::Ref(id) => {
                refs.push(*id);
            }
            StepValue::List(items) => {
                refs.extend(collect_all_refs(items));
            }
            StepValue::Typed { value, .. } => {
                refs.extend(collect_all_refs(std::slice::from_ref(value)));
            }
            _ => {}
        }
    }
    refs
}

// ============================================================
// Comprehensive extraction
// ============================================================

/// All AP242 data extracted from a STEP file.
#[derive(Clone, Debug, Default)]
pub struct Ap242Data {
    /// Tessellated geometry (5.1.1).
    pub tessellated: TessellatedGeometry,
    /// PMI annotations (5.1.2).
    pub pmi: PmiData,
    /// GD&T data (5.1.3).
    pub gdt: GdtData,
    /// Colour and layer data (5.1.4).
    pub colour_layer: ColourLayerData,
    /// Unit information (5.1.5).
    pub units: UnitData,
}

/// Extract all AP242 data from a STEP file.
///
/// Convenience function that runs all extractors in sequence.
pub fn extract_ap242(step_file: &StepFile) -> Ap242Data {
    Ap242Data {
        tessellated: extract_tessellated_geometry(step_file),
        pmi: extract_pmi(step_file),
        gdt: extract_gdt(step_file),
        colour_layer: extract_colour_and_layer(step_file),
        units: extract_units(step_file),
    }
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: create a StepFile from a STEP string.
    fn parse_step_string(input: &str) -> StepFile {
        crate::parser::parse_step(input).expect("Failed to parse STEP")
    }

    #[test]
    fn test_extract_units_default() {
        // A STEP file without explicit units should default to mm
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = SHAPE_DEFINITION_REPRESENTATION(#2, #3);
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let unit_data = extract_units(&file);
        assert_eq!(unit_data.units.length_unit, "MILLIMETRE");
        assert!((unit_data.units.length_to_si - 0.001).abs() < 1e-10);
        assert!(unit_data.units.uses_millimetres());
        assert!(!unit_data.units.uses_inches());
    }

    #[test]
    fn test_extract_units_si_millimetre() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );
#2 = ( PLANE_ANGLE_UNIT() NAMED_UNIT(*) SI_UNIT($,.RADIAN.) );
#3 = ( SOLID_ANGLE_UNIT() NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) );
#4 = UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#1,'distance_accuracy_value','confusion accuracy');
#5 = ( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNIT_ASSIGNED_CONTEXT((#1,#2,#3)) REPRESENTATION_CONTEXT('Context3D','3D Context with WR and GP') );
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let unit_data = extract_units(&file);
        assert!(unit_data.units.length_unit.contains("METRE"), "Got: {}", unit_data.units.length_unit);
        assert!((unit_data.units.length_to_si - 0.001).abs() < 1e-10, "Got factor: {}", unit_data.units.length_to_si);
        assert!(!unit_data.context_ids.is_empty(), "Should find GLOBAL_UNIT_ASSIGNED_CONTEXT");
    }

    #[test]
    fn test_extract_units_degree() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );
#2 = ( PLANE_ANGLE_UNIT() NAMED_UNIT(*) SI_UNIT($,.DEGREE.) );
#3 = ( SOLID_ANGLE_UNIT() NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) );
#4 = ( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNIT_ASSIGNED_CONTEXT((#1,#2,#3)) REPRESENTATION_CONTEXT('Context3D','3D Context') );
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let unit_data = extract_units(&file);
        assert!(unit_data.units.plane_angle_unit.contains("DEGREE"), "Got: {}", unit_data.units.plane_angle_unit);
        assert!(unit_data.units.uses_degrees());
        // PI/180 ≈ 0.01745
        assert!((unit_data.units.angle_to_si - std::f64::consts::PI / 180.0).abs() < 1e-10);
    }

    #[test]
    fn test_extract_colour_rgb() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = COLOUR_RGB('',0.5,0.2,0.8);
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let data = extract_colour_and_layer(&file);
        assert!(!data.colours.is_empty(), "Should find at least one colour");
        let colour = data.colours.iter().find(|c| c.step_id == 1).expect("Should find COLOUR_RGB #1");
        assert!((colour.rgb[0] - 0.5).abs() < 1e-10);
        assert!((colour.rgb[1] - 0.2).abs() < 1e-10);
        assert!((colour.rgb[2] - 0.8).abs() < 1e-10);
    }

    #[test]
    fn test_extract_gdt_tolerance() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = GEOMETRIC_TOLERANCE('position tolerance','pos',0.05,#10,#20);
#2 = DATUM_FEATURE('A','datum A',#30);
#3 = DATUM_REFERENCE('A',#2,.M.);
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let gdt = extract_gdt(&file);
        assert!(!gdt.tolerances.is_empty(), "Should find at least one tolerance");
        assert_eq!(gdt.tolerances[0].tolerance_type, GdtToleranceType::Position);
        assert!((gdt.tolerances[0].tolerance_value.unwrap() - 0.05).abs() < 1e-10);

        assert!(!gdt.datum_features.is_empty(), "Should find at least one datum feature");
        assert_eq!(gdt.datum_features[0].name, "A");

        assert!(!gdt.datum_references.is_empty(), "Should find at least one datum reference");
        assert_eq!(gdt.datum_references[0].modifier.as_deref(), Some("M"));
    }

    #[test]
    fn test_extract_pmi_formation() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = PRODUCT_DEFINITION_FORMATION('design','',#10);
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let pmi = extract_pmi(&file);
        assert!(!pmi.product_definition_formation_ids.is_empty());
        assert_eq!(pmi.product_definition_formation_ids[0], 1);
    }

    #[test]
    fn test_extract_layer() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = PRESENTATION_LAYER_ASSIGNMENT('Layer1','First layer',(#10,#20));
#2 = LAYERED_ITEM(#1,#30);
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let data = extract_colour_and_layer(&file);
        assert!(!data.layers.is_empty(), "Should find at least one layer");
        assert_eq!(data.layers[0].name, "Layer1");
        assert!(!data.layer_assignments.is_empty(), "Should find at least one layer assignment");
        assert_eq!(data.layer_assignments[0].layer_name, "Layer1");
    }

    #[test]
    fn test_extract_ap242_combined() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );
#2 = COLOUR_RGB('red',1.0,0.0,0.0);
#3 = GEOMETRIC_TOLERANCE('flatness','flat',0.01,#10);
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let data = extract_ap242(&file);
        assert!(data.units.units.uses_millimetres());
        assert!(!data.colour_layer.colours.is_empty());
        assert!(!data.gdt.tolerances.is_empty());
    }

    #[test]
    fn test_unit_info_conversions() {
        let units = UnitInfo::default();
        // Default is mm → m: factor 0.001
        assert!((units.length_to_metres(10.0) - 0.01).abs() < 1e-15);
        assert!((units.metres_to_length(0.01) - 10.0).abs() < 1e-15);

        // Degrees
        let mut deg_units = UnitInfo::default();
        deg_units.plane_angle_unit = "DEGREE".to_string();
        deg_units.angle_to_si = std::f64::consts::PI / 180.0;
        assert!((deg_units.angle_to_radians(180.0) - std::f64::consts::PI).abs() < 1e-10);
    }

    #[test]
    fn test_gdt_tolerance_type_from_step_type() {
        assert_eq!(GdtToleranceType::from_step_type("POSITION_TOLERANCE"), GdtToleranceType::Position);
        assert_eq!(GdtToleranceType::from_step_type("FLATNESS_TOLERANCE"), GdtToleranceType::Flatness);
        assert_eq!(GdtToleranceType::from_step_type("CIRCULAR_RUNOUT_TOLERANCE"), GdtToleranceType::Runout);
        assert_eq!(GdtToleranceType::from_step_type("SOME_UNKNOWN_TYPE"), GdtToleranceType::Other("SOME_UNKNOWN_TYPE".to_string()));
    }

    #[test]
    fn test_tessellated_geometry_empty() {
        let step = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('test'), '2;1');
FILE_NAME('test.stp', '2024-01-01', (''), (''), 'test', '', '');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1 = SHAPE_DEFINITION_REPRESENTATION(#2, #3);
ENDSEC;
END-ISO-10303-21;
"#;
        let file = parse_step_string(step);
        let tess = extract_tessellated_geometry(&file);
        assert!(tess.faces.is_empty());
        assert!(tess.representation_ids.is_empty());
    }
}
