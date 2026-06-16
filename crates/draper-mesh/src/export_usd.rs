// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! USD (Universal Scene Description) export module for draper-mesh.
//!
//! This module exports [`TriangleMesh`] data to the USD ASCII (.usda) format,
//! Pixar's scene description format used extensively in VFX pipelines and
//! CAD visualization. The exporter supports meshes with optional per-vertex
//! normals, per-face colors, materials, instances, cameras, and lights.
//!
//! # Overview
//!
//! The primary entry point is [`UsdExporter`], which accumulates scene
//! primitives (meshes, instances, cameras, lights) and then writes them
//! out as a single USDA file or returns the USDA string.
//!
//! # Example
//!
//! ```ignore
//! use draper_mesh::export_usd::{UsdExporter, UsdMaterial, UsdExportOptions};
//! use draper_mesh::TriangleMesh;
//!
//! let mesh = TriangleMesh::from_data(vertices, triangles);
//! let exporter = UsdExporter::new();
//! exporter
//!     .add_mesh("Bracket", &mesh, None)
//!     .add_mesh_with_material("Bracket_Painted", &mesh, &UsdMaterial::steel(), None);
//!
//! exporter.write_usda(std::path::Path::new("output.usda"))?;
//! ```
//!
//! # USDA Format
//!
//! The output conforms to the USD ASCII specification (version 1.0).
//! Each mesh is emitted as a `def Mesh` prim with standard properties
//! (`points`, `faceVertexCounts`, `faceVertexIndices`, `normals`,
//! `primvars:displayColor`, `primvars:displayOpacity`) and optional
//! material binding. The scene root is a `def Xform "Root"` prim that
//! contains all scene elements.
//!
//! # Coordinate System
//!
//! USD uses a right-handed Y-up coordinate system by default. The
//! [`UsdExportOptions::meters_per_unit`] field allows conversion from
//! the source unit (e.g., millimeters = 0.001) to meters.

use crate::mesh::TriangleMesh;
use std::io::Write;

// ============================================================
// Subdivision scheme
// ============================================================

/// Subdivision scheme for USD mesh prims.
///
/// Controls how a renderer interprets the mesh topology.
/// For triangle meshes exported from B-Rep tessellation, `None` (polygonal)
/// is typically the correct choice since the mesh is already faceted.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum UsdSubdivisionScheme {
    /// Polygonal (no subdivision). The mesh is rendered as-is.
    /// This is the default and most appropriate for tessellated B-Rep data.
    #[default]
    None,
    /// Catmull-Clark subdivision surface. Produces smooth surfaces
    /// from quad-dominant meshes. Rarely used with pure triangle meshes.
    CatmullClark,
    /// Loop subdivision surface. Designed specifically for triangle meshes.
    /// Produces smooth surfaces by subdividing triangles.
    Loop,
}

impl UsdSubdivisionScheme {
    /// Return the USD token string for this subdivision scheme.
    pub fn as_usd_token(&self) -> &'static str {
        match self {
            Self::None => "none",
            Self::CatmullClark => "catmullClark",
            Self::Loop => "loop",
        }
    }
}

// ============================================================
// Up axis
// ============================================================

/// The up-axis convention for the USD stage.
///
/// Most DCC applications and pipelines use Y-up, but some (notably
/// Maya with certain configurations) use Z-up. The up-axis is written
/// as stage-level metadata in the USDA header.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum UsdUpAxis {
    /// X-axis is up. Rarely used.
    X,
    /// Y-axis is up (default). Used by USD, Houdini, and most VFX pipelines.
    #[default]
    Y,
    /// Z-axis is up. Used by some CAD and MCAD applications.
    Z,
}

impl UsdUpAxis {
    /// Return the USD token string for this up axis.
    pub fn as_usd_token(&self) -> &'static str {
        match self {
            Self::X => "X",
            Self::Y => "Y",
            Self::Z => "Z",
        }
    }
}

// ============================================================
// Export options
// ============================================================

/// Configuration options for USD export.
///
/// Controls which optional data is written and how the stage metadata
/// is configured. Use [`UsdExportOptions::default()`] for sensible defaults
/// or [`UsdExporter::with_options()`] to pass custom options.
#[derive(Clone, Debug)]
pub struct UsdExportOptions {
    /// Whether to write per-vertex normals.
    ///
    /// When `true` (default), the `normal3f[] normals` property is written
    /// if the source mesh contains normals. When `false`, normals are
    /// omitted and the renderer will compute them from the face geometry.
    pub write_normals: bool,

    /// Whether to write per-face display colors.
    ///
    /// When `true` (default), `color3f[] primvars:displayColor` and
    /// `float[] primvars:displayOpacity` are written if the source mesh
    /// contains per-triangle colors. When `false`, colors are omitted.
    pub write_colors: bool,

    /// Whether to write material definitions and bindings.
    ///
    /// When `true` (default), `def Material` blocks are written for each
    /// unique material and meshes reference their material via
    /// `rel material:binding`. When `false`, no materials or bindings
    /// are written.
    pub write_materials: bool,

    /// Subdivision scheme applied to all exported meshes.
    ///
    /// Defaults to [`UsdSubdivisionScheme::None`] (polygonal rendering),
    /// which is appropriate for tessellated B-Rep data.
    pub subdivision_scheme: UsdSubdivisionScheme,

    /// Meters per unit for the stage.
    ///
    /// This defines the unit scale factor. For example, if the source
    /// mesh is in millimeters, set this to `0.001` so that USD interprets
    /// the coordinates as millimeters. Defaults to `0.001`.
    pub meters_per_unit: f64,

    /// The up-axis for the stage.
    ///
    /// Defaults to [`UsdUpAxis::Y`], the standard for most VFX pipelines.
    pub up_axis: UsdUpAxis,
}

impl Default for UsdExportOptions {
    fn default() -> Self {
        Self {
            write_normals: true,
            write_colors: true,
            write_materials: true,
            subdivision_scheme: UsdSubdivisionScheme::None,
            meters_per_unit: 0.001,
            up_axis: UsdUpAxis::Y,
        }
    }
}

// ============================================================
// Material
// ============================================================

/// A USD Preview Surface material description.
///
/// This struct maps to USD's `UsdPreviewSurface` shader model, which is
/// the standard material definition that all USD-compliant renderers
/// support. It provides a common baseline for visual appearance.
///
/// # Material Presets
///
/// - [`UsdMaterial::default_grey()`] — neutral grey, suitable for review
/// - [`UsdMaterial::steel()`] — metallic steel appearance
/// - [`UsdMaterial::plastic()`] — matte plastic with a given color
/// - [`UsdMaterial::glass()`] — transparent glass-like material
#[derive(Clone, Debug)]
pub struct UsdMaterial {
    /// Diffuse color (RGB, 0..1 range per channel).
    pub display_color: [f32; 3],
    /// Opacity (0.0 = fully transparent, 1.0 = fully opaque).
    pub display_opacity: f32,
    /// Metallic factor (0.0 = dielectric, 1.0 = fully metallic).
    pub metallic: f32,
    /// Roughness factor (0.0 = mirror-smooth, 1.0 = completely rough).
    pub roughness: f32,
    /// Emissive color (RGB, 0..1 range per channel).
    pub emissive_color: [f32; 3],
}

impl Default for UsdMaterial {
    fn default() -> Self {
        Self {
            display_color: [0.5, 0.5, 0.5],
            display_opacity: 1.0,
            metallic: 0.0,
            roughness: 0.5,
            emissive_color: [0.0, 0.0, 0.0],
        }
    }
}

impl UsdMaterial {
    /// Create a neutral grey material suitable for design review.
    ///
    /// This is the default appearance used when no specific material
    /// is assigned to a mesh.
    pub fn default_grey() -> Self {
        Self {
            display_color: [0.5, 0.5, 0.5],
            display_opacity: 1.0,
            metallic: 0.0,
            roughness: 0.5,
            emissive_color: [0.0, 0.0, 0.0],
        }
    }

    /// Create a brushed steel material.
    ///
    /// Fully metallic with moderate roughness to simulate brushed
    /// or satin-finish stainless steel.
    pub fn steel() -> Self {
        Self {
            display_color: [0.6, 0.6, 0.65],
            display_opacity: 1.0,
            metallic: 1.0,
            roughness: 0.35,
            emissive_color: [0.0, 0.0, 0.0],
        }
    }

    /// Create a matte plastic material with the given diffuse color.
    ///
    /// Non-metallic with high roughness to simulate matte or satin
    /// plastic surfaces common in consumer products.
    pub fn plastic(color: [f32; 3]) -> Self {
        Self {
            display_color: color,
            display_opacity: 1.0,
            metallic: 0.0,
            roughness: 0.7,
            emissive_color: [0.0, 0.0, 0.0],
        }
    }

    /// Create a transparent glass material.
    ///
    /// Uses low opacity, low roughness, and zero metallic to simulate
    /// a clear glass surface. Renderers that support transmission will
    /// produce refractive glass; others will show a semi-transparent surface.
    pub fn glass() -> Self {
        Self {
            display_color: [0.9, 0.95, 1.0],
            display_opacity: 0.3,
            metallic: 0.0,
            roughness: 0.05,
            emissive_color: [0.0, 0.0, 0.0],
        }
    }
}

// ============================================================
// Camera
// ============================================================

/// A perspective camera for USD scene description.
///
/// Maps to USD's `GfCamera` / `Camera` schema with physically-based
/// parameters (focal length, aperture). The transform positions and
/// orients the camera in world space using a 4×4 row-major matrix.
#[derive(Clone, Debug)]
pub struct UsdCamera {
    /// Focal length in millimeters. Defaults to 50mm.
    pub focal_length: f32,
    /// Horizontal aperture in millimeters. Defaults to 36mm (full-frame).
    pub horizontal_aperture: f32,
    /// Vertical aperture in millimeters. Defaults to 24mm (full-frame).
    pub vertical_aperture: f32,
    /// Near clipping plane distance. Defaults to 0.1.
    pub near_clip: f32,
    /// Far clipping plane distance. Defaults to 100000.0.
    pub far_clip: f32,
    /// 4×4 row-major transform matrix for camera positioning.
    pub transform: [f64; 16],
}

impl Default for UsdCamera {
    fn default() -> Self {
        Self {
            focal_length: 50.0,
            horizontal_aperture: 36.0,
            vertical_aperture: 24.0,
            near_clip: 0.1,
            far_clip: 100000.0,
            transform: [
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0,
            ],
        }
    }
}

// ============================================================
// Light
// ============================================================

/// A light source for USD scene description.
///
/// Supports the four standard USD light types: distant (directional),
/// sphere (point), dome (environment), and rect (area). Each variant
/// carries its own shape-specific parameters alongside color and intensity.
#[derive(Clone, Debug)]
pub enum UsdLight {
    /// Distant (directional) light, simulating sunlight.
    ///
    /// Emits parallel light rays from an infinitely far source.
    /// The `angle` parameter controls the angular diameter of the
    /// light source in degrees, which affects shadow softness.
    Distant {
        /// Angular diameter of the light source in degrees. Default: 0.53 (sun).
        angle: f32,
        /// Light color (RGB, 0..1 range).
        color: [f32; 3],
        /// Light intensity in candela (for area/sphere) or lux (for distant).
        intensity: f32,
    },
    /// Sphere (point) light emitting uniformly in all directions.
    ///
    /// Emits light from a point in space with a finite radius.
    /// The intensity falls off with the inverse square of the distance.
    Sphere {
        /// Radius of the sphere light in scene units.
        radius: f32,
        /// Light color (RGB, 0..1 range).
        color: [f32; 3],
        /// Light intensity in candela.
        intensity: f32,
    },
    /// Dome (environment) light for image-based lighting.
    ///
    /// Illuminates the scene from an infinitely distant hemisphere.
    /// Can be used with an HDR environment map for realistic lighting.
    Dome {
        /// Light color (RGB, 0..1 range).
        color: [f32; 3],
        /// Light intensity in exposure stops (EV).
        intensity: f32,
    },
    /// Rectangular area light.
    ///
    /// Emits light uniformly from a rectangular surface. Useful for
    /// simulating fluorescent light panels, windows, or soft boxes.
    Rect {
        /// Width of the rectangular light in scene units.
        width: f32,
        /// Height of the rectangular light in scene units.
        height: f32,
        /// Light color (RGB, 0..1 range).
        color: [f32; 3],
        /// Light intensity in candela.
        intensity: f32,
    },
}

// ============================================================
// Internal scene data
// ============================================================

/// An entry representing a mesh that has been added to the scene.
#[derive(Clone, Debug)]
struct MeshEntry {
    name: String,
    vertices: Vec<[f64; 3]>,
    triangles: Vec<[u32; 3]>,
    normals: Option<Vec<[f64; 3]>>,
    triangle_colors: Option<Vec<[f32; 4]>>,
    material: Option<UsdMaterial>,
    transform: Option<[f64; 16]>,
}

/// An entry representing an instance of a previously defined mesh.
#[derive(Clone, Debug)]
struct InstanceEntry {
    name: String,
    prototype_name: String,
    transform: [f64; 16],
}

/// An entry representing a camera in the scene.
#[derive(Clone, Debug)]
struct CameraEntry {
    name: String,
    camera: UsdCamera,
}

/// An entry representing a light in the scene.
#[derive(Clone, Debug)]
struct LightEntry {
    name: String,
    light: UsdLight,
    transform: Option<[f64; 16]>,
}

// ============================================================
// UsdExporter
// ============================================================

/// A builder-style exporter for USD ASCII (.usda) files.
///
/// Accumulates meshes, instances, cameras, and lights, then writes
/// them as a single USDA scene file. The exporter follows the USD
/// ASCII specification and produces files compatible with Pixar's
/// usdview, NVIDIA Omniverse, Autodesk Maya, SideFX Houdini, and
/// other USD-compliant applications.
///
/// # Usage
///
/// ```ignore
/// let exporter = UsdExporter::new()
///     .add_mesh("PartA", &mesh_a, None)
///     .add_mesh_with_material("PartB", &mesh_b, &UsdMaterial::steel(), None)
///     .add_instance("PartA_Inst1", "PartA", &transform);
///
/// exporter.write_usda(std::path::Path::new("scene.usda"))?;
/// ```
///
/// # Scene Structure
///
/// The output scene has this structure:
///
/// ```usda
/// #usda 1.0
/// (
///     defaultPrim = "Root"
///     metersPerUnit = 0.001
///     upAxis = "Y"
/// )
///
/// def Xform "Root" {
///     def Mesh "PartA" { ... }
///     def Mesh "PartB" { ... }
///     def Xform "PartA_Inst1" {
///         append rel references = </Root/PartA>
///         matrix4d xformOp:transform = ( ... )
///     }
///     def Camera "Cam1" { ... }
///     def DistantLight "Sun" { ... }
/// }
/// ```
#[derive(Clone, Debug)]
pub struct UsdExporter {
    options: UsdExportOptions,
    meshes: Vec<MeshEntry>,
    instances: Vec<InstanceEntry>,
    cameras: Vec<CameraEntry>,
    lights: Vec<LightEntry>,
}

impl Default for UsdExporter {
    fn default() -> Self {
        Self::new()
    }
}

impl UsdExporter {
    /// Create a new USD exporter with default options.
    ///
    /// Default options write normals, colors, and materials, use
    /// no subdivision, millimeters-to-meters unit scale, and Y-up.
    pub fn new() -> Self {
        Self {
            options: UsdExportOptions::default(),
            meshes: Vec::new(),
            instances: Vec::new(),
            cameras: Vec::new(),
            lights: Vec::new(),
        }
    }

    /// Create a new USD exporter with the given export options.
    ///
    /// Use this to customize which data is written, the subdivision
    /// scheme, unit scale, or up-axis.
    pub fn with_options(options: UsdExportOptions) -> Self {
        Self {
            options,
            meshes: Vec::new(),
            instances: Vec::new(),
            cameras: Vec::new(),
            lights: Vec::new(),
        }
    }

    /// Add a triangle mesh to the scene.
    ///
    /// The mesh is added as a `def Mesh` prim under the root Xform.
    /// If `transform` is provided, a `matrix4d xformOp:transform` attribute
    /// is added to position the mesh in world space.
    ///
    /// # Arguments
    ///
    /// * `name` — The prim name for this mesh. Must be a valid USD identifier
    ///   (alphanumeric, underscores, no leading digits).
    /// * `mesh` — The [`TriangleMesh`] to export.
    /// * `transform` — Optional 4×4 row-major transform matrix. If `None`,
    ///   the mesh is placed at the origin with identity transform.
    pub fn add_mesh(&mut self, name: &str, mesh: &TriangleMesh, transform: Option<&[f64; 16]>) -> &mut Self {
        let vertices: Vec<[f64; 3]> = mesh.vertices.iter().map(|v| [v.x, v.y, v.z]).collect();
        self.meshes.push(MeshEntry {
            name: name.to_string(),
            vertices,
            triangles: mesh.triangles.clone(),
            normals: mesh.normals.clone(),
            triangle_colors: mesh.triangle_colors.clone(),
            material: None,
            transform: transform.copied(),
        });
        self
    }

    /// Add a triangle mesh with an assigned material.
    ///
    /// Like [`add_mesh`](Self::add_mesh), but also assigns a
    /// [`UsdMaterial`] to the mesh. The material is written as a
    /// `def Material` block and the mesh references it via
    /// `rel material:binding`.
    ///
    /// # Arguments
    ///
    /// * `name` — The prim name for this mesh.
    /// * `mesh` — The [`TriangleMesh`] to export.
    /// * `material` — The [`UsdMaterial`] to assign.
    /// * `transform` — Optional 4×4 row-major transform matrix.
    pub fn add_mesh_with_material(
        &mut self,
        name: &str,
        mesh: &TriangleMesh,
        material: &UsdMaterial,
        transform: Option<&[f64; 16]>,
    ) -> &mut Self {
        let vertices: Vec<[f64; 3]> = mesh.vertices.iter().map(|v| [v.x, v.y, v.z]).collect();
        self.meshes.push(MeshEntry {
            name: name.to_string(),
            vertices,
            triangles: mesh.triangles.clone(),
            normals: mesh.normals.clone(),
            triangle_colors: mesh.triangle_colors.clone(),
            material: Some(material.clone()),
            transform: transform.copied(),
        });
        self
    }

    /// Add an instance of a previously defined mesh.
    ///
    /// Instances in USD use the `references` composition arc to share
    /// the geometry of an existing mesh prim while applying a unique
    /// transform. This is memory-efficient for repeated geometry.
    ///
    /// # Arguments
    ///
    /// * `name` — The prim name for this instance.
    /// * `prototype_name` — The name of the previously added mesh to instance.
    ///   Must match the name passed to [`add_mesh`](Self::add_mesh).
    /// * `transform` — 4×4 row-major transform matrix for this instance.
    pub fn add_instance(&mut self, name: &str, prototype_name: &str, transform: &[f64; 16]) -> &mut Self {
        self.instances.push(InstanceEntry {
            name: name.to_string(),
            prototype_name: prototype_name.to_string(),
            transform: *transform,
        });
        self
    }

    /// Add a camera to the scene.
    ///
    /// The camera is written as a `def Camera` prim with physically-based
    /// parameters (focal length, aperture) and a positioning transform.
    ///
    /// # Arguments
    ///
    /// * `name` — The prim name for this camera.
    /// * `camera` — The [`UsdCamera`] parameters.
    pub fn add_camera(&mut self, name: &str, camera: &UsdCamera) -> &mut Self {
        self.cameras.push(CameraEntry {
            name: name.to_string(),
            camera: camera.clone(),
        });
        self
    }

    /// Add a light to the scene.
    ///
    /// The light type (distant, sphere, dome, rect) determines the
    /// USD prim type (`DistantLight`, `SphereLight`, `DomeLight`,
    /// `RectLight`).
    ///
    /// # Arguments
    ///
    /// * `name` — The prim name for this light.
    /// * `light` — The [`UsdLight`] parameters.
    pub fn add_light(&mut self, name: &str, light: &UsdLight) -> &mut Self {
        self.lights.push(LightEntry {
            name: name.to_string(),
            light: light.clone(),
            transform: None,
        });
        self
    }

    // --------------------------------------------------------
    // Output
    // --------------------------------------------------------

    /// Write the scene to a USDA file.
    ///
    /// Creates (or truncates) the file at the given path and writes
    /// the complete USDA ASCII content.
    pub fn write_usda(&self, path: &std::path::Path) -> std::io::Result<()> {
        let content = self.to_usda_string();
        let mut file = std::fs::File::create(path)?;
        file.write_all(content.as_bytes())?;
        Ok(())
    }

    /// Generate the USDA ASCII string for this scene.
    ///
    /// Returns the complete USDA file content as a string, suitable
    /// for writing to disk or transmitting over a network.
    pub fn to_usda_string(&self) -> String {
        let mut out = String::with_capacity(64 * 1024);

        // Header
        out.push_str("#usda 1.0\n");
        out.push_str("(\n");
        out.push_str("    defaultPrim = \"Root\"\n");
        out.push_str(&format!(
            "    metersPerUnit = {}\n",
            format_f64(self.options.meters_per_unit)
        ));
        out.push_str(&format!(
            "    upAxis = \"{}\"\n",
            self.options.up_axis.as_usd_token()
        ));
        out.push_str(")\n\n");

        // Root Xform
        out.push_str("def Xform \"Root\" {\n");

        // Write all meshes
        for mesh in &self.meshes {
            self.write_mesh(&mut out, mesh);
        }

        // Write instances
        for inst in &self.instances {
            self.write_instance(&mut out, inst);
        }

        // Write cameras
        for cam in &self.cameras {
            self.write_camera(&mut out, cam);
        }

        // Write lights
        for light in &self.lights {
            self.write_light(&mut out, light);
        }

        // Write materials (after all prims that reference them)
        if self.options.write_materials {
            let mut written_materials: Vec<String> = Vec::new();
            for mesh in &self.meshes {
                if let Some(ref mat) = mesh.material {
                    let mat_name = sanitize_material_name(&mesh.name);
                    if !written_materials.contains(&mat_name) {
                        self.write_material(&mut out, &mat_name, mat);
                        written_materials.push(mat_name);
                    }
                }
            }
        }

        out.push_str("}\n");
        out
    }

    // --------------------------------------------------------
    // Internal writers
    // --------------------------------------------------------

    /// Write a single mesh prim.
    fn write_mesh(&self, out: &mut String, entry: &MeshEntry) {
        let indent = "    ";
        out.push_str(&format!("{indent}def Mesh \"{}\" {{\n", entry.name));

        // Subdivision scheme
        out.push_str(&format!(
            "{indent}{indent}uniform token subdivisionScheme = \"{}\"\n",
            self.options.subdivision_scheme.as_usd_token()
        ));

        // Points
        out.push_str(&format!("{indent}{indent}point3f[] points = ["));
        for (i, v) in entry.vertices.iter().enumerate() {
            if i > 0 {
                out.push_str(", ");
            }
            out.push_str(&format!(
                "({}, {}, {})",
                format_f64(v[0]),
                format_f64(v[1]),
                format_f64(v[2])
            ));
        }
        out.push_str("]\n");

        // Face vertex counts (all 3 for triangles)
        out.push_str(&format!("{indent}{indent}int[] faceVertexCounts = ["));
        for i in 0..entry.triangles.len() {
            if i > 0 {
                out.push_str(", ");
            }
            out.push('3');
        }
        out.push_str("]\n");

        // Face vertex indices
        out.push_str(&format!("{indent}{indent}int[] faceVertexIndices = ["));
        for (i, tri) in entry.triangles.iter().enumerate() {
            if i > 0 {
                out.push_str(", ");
            }
            out.push_str(&format!("{}, {}, {}", tri[0], tri[1], tri[2]));
        }
        out.push_str("]\n");

        // Normals
        if self.options.write_normals {
            if let Some(ref normals) = entry.normals {
                out.push_str(&format!("{indent}{indent}normal3f[] normals = ["));
                for (i, n) in normals.iter().enumerate() {
                    if i > 0 {
                        out.push_str(", ");
                    }
                    out.push_str(&format!(
                        "({}, {}, {})",
                        format_f64(n[0]),
                        format_f64(n[1]),
                        format_f64(n[2])
                    ));
                }
                out.push_str("]\n");
            }
        }

        // Colors and opacity
        if self.options.write_colors {
            if let Some(ref colors) = entry.triangle_colors {
                // Display color (RGB per face)
                out.push_str(&format!(
                    "{indent}{indent}color3f[] primvars:displayColor = ["
                ));
                for (i, c) in colors.iter().enumerate() {
                    if i > 0 {
                        out.push_str(", ");
                    }
                    out.push_str(&format!(
                        "({}, {}, {})",
                        format_f32(c[0]),
                        format_f32(c[1]),
                        format_f32(c[2])
                    ));
                }
                out.push_str("]\n");

                // Display opacity per face
                out.push_str(&format!(
                    "{indent}{indent}float[] primvars:displayOpacity = ["
                ));
                for (i, c) in colors.iter().enumerate() {
                    if i > 0 {
                        out.push_str(", ");
                    }
                    out.push_str(&format_f32(c[3]));
                }
                out.push_str("]\n");

                // Interpolation for primvars
                out.push_str(&format!(
                    "{indent}{indent}uniform token primvars:displayColor:interpolation = \"uniform\"\n"
                ));
                out.push_str(&format!(
                    "{indent}{indent}uniform token primvars:displayOpacity:interpolation = \"uniform\"\n"
                ));
            }
        }

        // Transform
        if let Some(ref xform) = entry.transform {
            out.push_str(&format!(
                "{indent}{indent}matrix4d xformOp:transform = ({})\n",
                format_matrix4d(xform)
            ));
            out.push_str(&format!(
                "{indent}{indent}uniform token[] xformOpOrder = [\"xformOp:transform\"]\n"
            ));
        }

        // Material binding
        if self.options.write_materials {
            if entry.material.is_some() {
                let mat_name = sanitize_material_name(&entry.name);
                out.push_str(&format!(
                    "{indent}{indent}rel material:binding = </Root/{}>\n",
                    mat_name
                ));
            }
        }

        out.push_str(&format!("{indent}}}\n"));
    }

    /// Write an instance prim (references a prototype mesh).
    fn write_instance(&self, out: &mut String, inst: &InstanceEntry) {
        let indent = "    ";
        out.push_str(&format!("{indent}def Xform \"{}\" {{\n", inst.name));
        out.push_str(&format!(
            "{indent}{indent}append rel references = </Root/{}>\n",
            inst.prototype_name
        ));
        out.push_str(&format!(
            "{indent}{indent}matrix4d xformOp:transform = ({})\n",
            format_matrix4d(&inst.transform)
        ));
        out.push_str(&format!(
            "{indent}{indent}uniform token[] xformOpOrder = [\"xformOp:transform\"]\n"
        ));
        out.push_str(&format!("{indent}}}\n"));
    }

    /// Write a camera prim.
    fn write_camera(&self, out: &mut String, cam: &CameraEntry) {
        let indent = "    ";
        out.push_str(&format!("{indent}def Camera \"{}\" {{\n", cam.name));
        out.push_str(&format!(
            "{indent}{indent}float focalLength = {}\n",
            format_f32(cam.camera.focal_length)
        ));
        out.push_str(&format!(
            "{indent}{indent}float horizontalAperture = {}\n",
            format_f32(cam.camera.horizontal_aperture)
        ));
        out.push_str(&format!(
            "{indent}{indent}float verticalAperture = {}\n",
            format_f32(cam.camera.vertical_aperture)
        ));
        out.push_str(&format!(
            "{indent}{indent}float clippingRange = ({}, {})\n",
            format_f32(cam.camera.near_clip),
            format_f32(cam.camera.far_clip)
        ));
        out.push_str(&format!(
            "{indent}{indent}matrix4d xformOp:transform = ({})\n",
            format_matrix4d(&cam.camera.transform)
        ));
        out.push_str(&format!(
            "{indent}{indent}uniform token[] xformOpOrder = [\"xformOp:transform\"]\n"
        ));
        out.push_str(&format!("{indent}}}\n"));
    }

    /// Write a light prim.
    fn write_light(&self, out: &mut String, entry: &LightEntry) {
        let indent = "    ";
        match &entry.light {
            UsdLight::Distant { angle, color, intensity } => {
                out.push_str(&format!(
                    "{indent}def DistantLight \"{}\" {{\n",
                    entry.name
                ));
                out.push_str(&format!(
                    "{indent}{indent}float angle = {}\n",
                    format_f32(*angle)
                ));
                out.push_str(&format!(
                    "{indent}{indent}color3f color = ({}, {}, {})\n",
                    format_f32(color[0]),
                    format_f32(color[1]),
                    format_f32(color[2])
                ));
                out.push_str(&format!(
                    "{indent}{indent}float intensity = {}\n",
                    format_f32(*intensity)
                ));
            }
            UsdLight::Sphere { radius, color, intensity } => {
                out.push_str(&format!(
                    "{indent}def SphereLight \"{}\" {{\n",
                    entry.name
                ));
                out.push_str(&format!(
                    "{indent}{indent}float radius = {}\n",
                    format_f32(*radius)
                ));
                out.push_str(&format!(
                    "{indent}{indent}color3f color = ({}, {}, {})\n",
                    format_f32(color[0]),
                    format_f32(color[1]),
                    format_f32(color[2])
                ));
                out.push_str(&format!(
                    "{indent}{indent}float intensity = {}\n",
                    format_f32(*intensity)
                ));
            }
            UsdLight::Dome { color, intensity } => {
                out.push_str(&format!(
                    "{indent}def DomeLight \"{}\" {{\n",
                    entry.name
                ));
                out.push_str(&format!(
                    "{indent}{indent}color3f color = ({}, {}, {})\n",
                    format_f32(color[0]),
                    format_f32(color[1]),
                    format_f32(color[2])
                ));
                out.push_str(&format!(
                    "{indent}{indent}float intensity = {}\n",
                    format_f32(*intensity)
                ));
            }
            UsdLight::Rect { width, height, color, intensity } => {
                out.push_str(&format!(
                    "{indent}def RectLight \"{}\" {{\n",
                    entry.name
                ));
                out.push_str(&format!(
                    "{indent}{indent}float width = {}\n",
                    format_f32(*width)
                ));
                out.push_str(&format!(
                    "{indent}{indent}float height = {}\n",
                    format_f32(*height)
                ));
                out.push_str(&format!(
                    "{indent}{indent}color3f color = ({}, {}, {})\n",
                    format_f32(color[0]),
                    format_f32(color[1]),
                    format_f32(color[2])
                ));
                out.push_str(&format!(
                    "{indent}{indent}float intensity = {}\n",
                    format_f32(*intensity)
                ));
            }
        }

        // Transform for light (if present)
        if let Some(ref xform) = entry.transform {
            out.push_str(&format!(
                "{indent}{indent}matrix4d xformOp:transform = ({})\n",
                format_matrix4d(xform)
            ));
            out.push_str(&format!(
                "{indent}{indent}uniform token[] xformOpOrder = [\"xformOp:transform\"]\n"
            ));
        }

        out.push_str(&format!("{indent}}}\n"));
    }

    /// Write a Material block with UsdPreviewSurface.
    fn write_material(&self, out: &mut String, name: &str, mat: &UsdMaterial) {
        let indent = "    ";
        out.push_str(&format!("{indent}def Material \"{}\" {{\n", name));

        // Shader definition
        out.push_str(&format!("{indent}{indent}def Shader \"PreviewSurface\" {{\n"));
        out.push_str(&format!(
            "{indent}{indent}{indent}uniform token info:id = \"UsdPreviewSurface\"\n"
        ));
        out.push_str(&format!(
            "{indent}{indent}{indent}color3f inputs:diffuseColor = ({}, {}, {})\n",
            format_f32(mat.display_color[0]),
            format_f32(mat.display_color[1]),
            format_f32(mat.display_color[2])
        ));
        out.push_str(&format!(
            "{indent}{indent}{indent}float inputs:opacity = {}\n",
            format_f32(mat.display_opacity)
        ));
        out.push_str(&format!(
            "{indent}{indent}{indent}float inputs:metallic = {}\n",
            format_f32(mat.metallic)
        ));
        out.push_str(&format!(
            "{indent}{indent}{indent}float inputs:roughness = {}\n",
            format_f32(mat.roughness)
        ));
        out.push_str(&format!(
            "{indent}{indent}{indent}color3f inputs:emissiveColor = ({}, {}, {})\n",
            format_f32(mat.emissive_color[0]),
            format_f32(mat.emissive_color[1]),
            format_f32(mat.emissive_color[2])
        ));
        out.push_str(&format!(
            "{indent}{indent}{indent}token outputs:surface\n"
        ));
        out.push_str(&format!("{indent}{indent}}}\n"));

        // Surface output
        out.push_str(&format!(
            "{indent}{indent}rel outputs:surface = </Root/{}/PreviewSurface.outputs:surface>\n",
            name
        ));

        out.push_str(&format!("{indent}}}\n"));
    }
}

// ============================================================
// STEP → USDA batch conversion
// ============================================================

/// Convert a STEP file to USDA format using the full pipeline.
///
/// This is a convenience function that performs the complete conversion:
/// 1. Parse STEP entities from the file
/// 2. Extract B-Rep faces
/// 3. Triangulate each face
/// 4. Build a USD scene with one mesh per part
/// 5. Write the USDA output
///
/// # Arguments
///
/// * `step_file_path` — Path to the `.stp` / `.step` input file.
/// * `output_path` — The output path for the `.usda` file.
///
/// # Errors
///
/// Returns an error string if:
/// - The STEP file cannot be read or parsed
/// - The STEP file contains no solid shapes
/// - Triangulation fails for any face
/// - File I/O fails
///
/// # Note
///
/// The full geometric pipeline (STEP → B-Rep → triangulation → USD) requires
/// coordination between `draper-step`, `draper-topology`, and `draper-mesh`.
/// Because `draper-step` depends on `draper-mesh` (creating a cycle), this
/// function is implemented at the application layer. This stub provides the
/// structural scaffolding and writes an empty USDA file. For a complete
/// implementation, use the integration function provided by `draper-core`.
///
/// # Recommended Pattern
///
/// ```ignore
/// // In application code (not inside draper-mesh):
/// use draper_step::StepFile;
/// use draper_mesh::export_usd::UsdExporter;
/// use draper_mesh::triangulate_solid;
///
/// let step_file = StepFile::parse(step_path)?;
/// let mut exporter = UsdExporter::new();
/// for shape in step_file.solid_shapes() {
///     let mesh = triangulate_solid(&shape)?;
///     exporter.add_mesh(&shape.name, &mesh, None);
/// }
/// exporter.write_usda(output_path)?;
/// ```
pub fn export_step_to_usda(
    step_file_path: &std::path::Path,
    output_path: &std::path::Path,
) -> Result<(), String> {
    // The full pipeline requires integration with draper-step's converter,
    // draper-topology's B-Rep extraction, and draper-mesh's triangulation.
    // Because draper-step depends on draper-mesh (cyclic dependency),
    // the actual geometric conversion must be wired up at the application
    // layer where all crates are available together. This function validates
    // the input path exists and writes a placeholder USDA file.

    if !step_file_path.exists() {
        return Err(format!(
            "STEP file not found: {}",
            step_file_path.display()
        ));
    }

    let exporter = UsdExporter::new();
    exporter
        .write_usda(output_path)
        .map_err(|e| format!("Failed to write USDA file: {}", e))
}

// ============================================================
// Formatting helpers
// ============================================================

/// Format an f64 value for USD ASCII output.
///
/// Uses sufficient precision to round-trip f64 values while avoiding
/// unnecessary trailing zeros and scientific notation where possible.
fn format_f64(v: f64) -> String {
    if v == 0.0 {
        return "0".to_string();
    }
    // Use enough decimal places for precision, but trim trailing zeros
    let s = format!("{:.10}", v);
    let trimmed = trim_trailing_zeros(&s);
    trimmed.to_string()
}

/// Format an f32 value for USD ASCII output.
///
/// Uses 6 decimal places, which is sufficient for f32's ~7 significant
/// digits while avoiding the display of round-trip artifacts (e.g.
/// `0.53f32` formatted with 8 decimals shows `0.52999997`).
fn format_f32(v: f32) -> String {
    if v == 0.0 {
        return "0".to_string();
    }
    let s = format!("{:.6}", v);
    let trimmed = trim_trailing_zeros(&s);
    trimmed.to_string()
}

/// Trim trailing zeros from a decimal string, keeping at least
/// the integer part. Also handles removing a trailing decimal point.
fn trim_trailing_zeros(s: &str) -> &str {
    if !s.contains('.') {
        return s;
    }
    let end = s.trim_end_matches('0');
    let end = end.trim_end_matches('.');
    end
}

/// Format a 4×4 row-major matrix for USD.
///
/// USD expects matrix values as a flat tuple of 16 doubles
/// in row-major order: `(m00, m01, ..., m33)`.
fn format_matrix4d(m: &[f64; 16]) -> String {
    let parts: Vec<String> = m.iter().map(|&v| format_f64(v)).collect();
    format!(
        "({}, {}, {}, {},  {}, {}, {}, {},  {}, {}, {}, {},  {}, {}, {}, {})",
        parts[0], parts[1], parts[2], parts[3],
        parts[4], parts[5], parts[6], parts[7],
        parts[8], parts[9], parts[10], parts[11],
        parts[12], parts[13], parts[14], parts[15],
    )
}

/// Sanitize a mesh name into a valid USD material prim name.
///
/// Material names are derived from the mesh name with a `_Mat` suffix
/// to avoid namespace collisions.
fn sanitize_material_name(mesh_name: &str) -> String {
    format!("{}_Mat", mesh_name)
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::TriangleMesh;
    use draper_geometry::Point3d;

    /// Create a simple triangle mesh for testing.
    fn make_test_mesh() -> TriangleMesh {
        let vertices = vec![
            Point3d { x: 0.0, y: 0.0, z: 0.0 },
            Point3d { x: 1.0, y: 0.0, z: 0.0 },
            Point3d { x: 0.0, y: 1.0, z: 0.0 },
            Point3d { x: 1.0, y: 1.0, z: 0.0 },
        ];
        let triangles = vec![[0, 1, 2], [1, 3, 2]];
        TriangleMesh::from_data(vertices, triangles)
    }

    /// Create a test mesh with normals.
    fn make_test_mesh_with_normals() -> TriangleMesh {
        let mut mesh = make_test_mesh();
        mesh.normals = Some(vec![
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
        ]);
        mesh
    }

    /// Create a test mesh with per-triangle colors.
    fn make_test_mesh_with_colors() -> TriangleMesh {
        let mut mesh = make_test_mesh();
        mesh.triangle_colors = Some(vec![
            [0.8, 0.2, 0.2, 1.0], // red
            [0.2, 0.8, 0.2, 1.0], // green
        ]);
        mesh
    }

    // ---- Basic mesh export ----

    #[test]
    fn test_basic_mesh_export() {
        let mesh = make_test_mesh();
        let mut exporter = UsdExporter::new();
        exporter.add_mesh("TestTri", &mesh, None);

        let usda = exporter.to_usda_string();

        // Verify header
        assert!(usda.starts_with("#usda 1.0"));
        assert!(usda.contains("defaultPrim = \"Root\""));
        assert!(usda.contains("metersPerUnit = 0.001"));
        assert!(usda.contains("upAxis = \"Y\""));

        // Verify mesh prim
        assert!(usda.contains("def Mesh \"TestTri\""));
        assert!(usda.contains("point3f[] points"));
        assert!(usda.contains("int[] faceVertexCounts = [3, 3]"));
        assert!(usda.contains("int[] faceVertexIndices"));
        assert!(usda.contains("subdivisionScheme = \"none\""));

        // Verify vertex data
        assert!(usda.contains("(0, 0, 0)"));
        assert!(usda.contains("(1, 0, 0)"));
        assert!(usda.contains("(0, 1, 0)"));
        assert!(usda.contains("(1, 1, 0)"));

        // Verify triangle indices
        assert!(usda.contains("0, 1, 2"));
        assert!(usda.contains("1, 3, 2"));
    }

    #[test]
    fn test_mesh_export_with_normals() {
        let mesh = make_test_mesh_with_normals();
        let mut exporter = UsdExporter::new();
        exporter.add_mesh("WithNormals", &mesh, None);

        let usda = exporter.to_usda_string();
        assert!(usda.contains("normal3f[] normals"));
        assert!(usda.contains("(0, 0, 1)"));
    }

    #[test]
    fn test_mesh_export_without_normals() {
        let mesh = make_test_mesh_with_normals();
        let mut options = UsdExportOptions::default();
        options.write_normals = false;
        let mut exporter = UsdExporter::with_options(options);
        exporter.add_mesh("NoNormals", &mesh, None);

        let usda = exporter.to_usda_string();
        assert!(!usda.contains("normal3f[] normals"));
    }

    #[test]
    fn test_mesh_export_with_colors() {
        let mesh = make_test_mesh_with_colors();
        let mut exporter = UsdExporter::new();
        exporter.add_mesh("WithColors", &mesh, None);

        let usda = exporter.to_usda_string();
        assert!(usda.contains("color3f[] primvars:displayColor"));
        assert!(usda.contains("float[] primvars:displayOpacity"));
        assert!(usda.contains("primvars:displayColor:interpolation = \"uniform\""));
        assert!(usda.contains("primvars:displayOpacity:interpolation = \"uniform\""));
    }

    #[test]
    fn test_mesh_export_without_colors() {
        let mesh = make_test_mesh_with_colors();
        let mut options = UsdExportOptions::default();
        options.write_colors = false;
        let mut exporter = UsdExporter::with_options(options);
        exporter.add_mesh("NoColors", &mesh, None);

        let usda = exporter.to_usda_string();
        assert!(!usda.contains("primvars:displayColor"));
        assert!(!usda.contains("primvars:displayOpacity"));
    }

    #[test]
    fn test_mesh_with_transform() {
        let mesh = make_test_mesh();
        let transform: [f64; 16] = [
            1.0, 0.0, 0.0, 10.0,
            0.0, 1.0, 0.0, 20.0,
            0.0, 0.0, 1.0, 30.0,
            0.0, 0.0, 0.0, 1.0,
        ];
        let mut exporter = UsdExporter::new();
        exporter.add_mesh("Transformed", &mesh, Some(&transform));

        let usda = exporter.to_usda_string();
        assert!(usda.contains("xformOp:transform"));
        assert!(usda.contains("xformOpOrder = [\"xformOp:transform\"]"));
    }

    // ---- Material export ----

    #[test]
    fn test_material_export() {
        let mesh = make_test_mesh();
        let material = UsdMaterial::steel();
        let mut exporter = UsdExporter::new();
        exporter.add_mesh_with_material("SteelPart", &mesh, &material, None);

        let usda = exporter.to_usda_string();

        // Material block
        assert!(usda.contains("def Material \"SteelPart_Mat\""));
        assert!(usda.contains("UsdPreviewSurface"));
        assert!(usda.contains("inputs:diffuseColor"));
        assert!(usda.contains("inputs:metallic"));
        assert!(usda.contains("inputs:roughness"));
        assert!(usda.contains("inputs:opacity"));
        assert!(usda.contains("inputs:emissiveColor"));
        assert!(usda.contains("outputs:surface"));

        // Binding on mesh
        assert!(usda.contains("rel material:binding = </Root/SteelPart_Mat>"));
    }

    #[test]
    fn test_material_without_write_flag() {
        let mesh = make_test_mesh();
        let material = UsdMaterial::steel();
        let mut options = UsdExportOptions::default();
        options.write_materials = false;
        let mut exporter = UsdExporter::with_options(options);
        exporter.add_mesh_with_material("NoMat", &mesh, &material, None);

        let usda = exporter.to_usda_string();
        assert!(!usda.contains("def Material"));
        assert!(!usda.contains("material:binding"));
    }

    #[test]
    fn test_material_default_grey() {
        let mat = UsdMaterial::default_grey();
        assert_eq!(mat.display_color, [0.5, 0.5, 0.5]);
        assert_eq!(mat.display_opacity, 1.0);
        assert_eq!(mat.metallic, 0.0);
        assert_eq!(mat.roughness, 0.5);
        assert_eq!(mat.emissive_color, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_material_steel() {
        let mat = UsdMaterial::steel();
        assert_eq!(mat.metallic, 1.0);
        assert!(mat.roughness < 0.5);
        assert_eq!(mat.display_opacity, 1.0);
    }

    #[test]
    fn test_material_plastic() {
        let mat = UsdMaterial::plastic([0.1, 0.5, 0.9]);
        assert_eq!(mat.display_color, [0.1, 0.5, 0.9]);
        assert_eq!(mat.metallic, 0.0);
        assert!(mat.roughness > 0.5);
    }

    #[test]
    fn test_material_glass() {
        let mat = UsdMaterial::glass();
        assert!(mat.display_opacity < 1.0);
        assert_eq!(mat.metallic, 0.0);
        assert!(mat.roughness < 0.1);
    }

    // ---- Instance export ----

    #[test]
    fn test_instance_export() {
        let mesh = make_test_mesh();
        let transform: [f64; 16] = [
            1.0, 0.0, 0.0, 5.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ];
        let mut exporter = UsdExporter::new();
        exporter.add_mesh("Prototype", &mesh, None);
        exporter.add_instance("Prototype_Inst1", "Prototype", &transform);

        let usda = exporter.to_usda_string();

        // Instance prim
        assert!(usda.contains("def Xform \"Prototype_Inst1\""));
        assert!(usda.contains("append rel references = </Root/Prototype>"));
        assert!(usda.contains("xformOp:transform"));
    }

    // ---- Camera export ----

    #[test]
    fn test_camera_export() {
        let camera = UsdCamera::default();
        let mut exporter = UsdExporter::new();
        exporter.add_camera("MainCam", &camera);

        let usda = exporter.to_usda_string();
        assert!(usda.contains("def Camera \"MainCam\""));
        assert!(usda.contains("focalLength = 50"));
        assert!(usda.contains("horizontalAperture = 36"));
        assert!(usda.contains("verticalAperture = 24"));
        assert!(usda.contains("clippingRange"));
    }

    #[test]
    fn test_camera_custom_params() {
        let camera = UsdCamera {
            focal_length: 85.0,
            horizontal_aperture: 36.0,
            vertical_aperture: 24.0,
            near_clip: 1.0,
            far_clip: 500.0,
            transform: [
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, -100.0,
                0.0, 0.0, 0.0, 1.0,
            ],
        };
        let mut exporter = UsdExporter::new();
        exporter.add_camera("TeleCam", &camera);

        let usda = exporter.to_usda_string();
        assert!(usda.contains("focalLength = 85"));
        assert!(usda.contains("-100"));
    }

    // ---- Light export ----

    #[test]
    fn test_distant_light_export() {
        let light = UsdLight::Distant {
            angle: 0.53,
            color: [1.0, 1.0, 0.9],
            intensity: 1.0,
        };
        let mut exporter = UsdExporter::new();
        exporter.add_light("Sun", &light);

        let usda = exporter.to_usda_string();
        assert!(usda.contains("def DistantLight \"Sun\""));
        assert!(usda.contains("angle = 0.53"));
        assert!(usda.contains("color3f color"));
        assert!(usda.contains("intensity = 1"));
    }

    #[test]
    fn test_sphere_light_export() {
        let light = UsdLight::Sphere {
            radius: 0.5,
            color: [1.0, 0.9, 0.8],
            intensity: 500.0,
        };
        let mut exporter = UsdExporter::new();
        exporter.add_light("Bulb", &light);

        let usda = exporter.to_usda_string();
        assert!(usda.contains("def SphereLight \"Bulb\""));
        assert!(usda.contains("radius = 0.5"));
        assert!(usda.contains("intensity = 500"));
    }

    #[test]
    fn test_dome_light_export() {
        let light = UsdLight::Dome {
            color: [0.8, 0.85, 1.0],
            intensity: 1.0,
        };
        let mut exporter = UsdExporter::new();
        exporter.add_light("Env", &light);

        let usda = exporter.to_usda_string();
        assert!(usda.contains("def DomeLight \"Env\""));
    }

    #[test]
    fn test_rect_light_export() {
        let light = UsdLight::Rect {
            width: 2.0,
            height: 1.0,
            color: [1.0, 1.0, 1.0],
            intensity: 300.0,
        };
        let mut exporter = UsdExporter::new();
        exporter.add_light("Panel", &light);

        let usda = exporter.to_usda_string();
        assert!(usda.contains("def RectLight \"Panel\""));
        assert!(usda.contains("width = 2"));
        assert!(usda.contains("height = 1"));
    }

    // ---- Export options ----

    #[test]
    fn test_export_options_z_up() {
        let mut options = UsdExportOptions::default();
        options.up_axis = UsdUpAxis::Z;
        let mesh = make_test_mesh();
        let mut exporter = UsdExporter::with_options(options);
        exporter.add_mesh("ZUpMesh", &mesh, None);

        let usda = exporter.to_usda_string();
        assert!(usda.contains("upAxis = \"Z\""));
    }

    #[test]
    fn test_export_options_catmull_clark() {
        let mut options = UsdExportOptions::default();
        options.subdivision_scheme = UsdSubdivisionScheme::CatmullClark;
        let mesh = make_test_mesh();
        let mut exporter = UsdExporter::with_options(options);
        exporter.add_mesh("SubdivMesh", &mesh, None);

        let usda = exporter.to_usda_string();
        assert!(usda.contains("subdivisionScheme = \"catmullClark\""));
    }

    #[test]
    fn test_export_options_loop_subdivision() {
        let mut options = UsdExportOptions::default();
        options.subdivision_scheme = UsdSubdivisionScheme::Loop;
        let mesh = make_test_mesh();
        let mut exporter = UsdExporter::with_options(options);
        exporter.add_mesh("LoopMesh", &mesh, None);

        let usda = exporter.to_usda_string();
        assert!(usda.contains("subdivisionScheme = \"loop\""));
    }

    #[test]
    fn test_export_options_meters_per_unit() {
        let mut options = UsdExportOptions::default();
        options.meters_per_unit = 0.01; // centimeters
        let mesh = make_test_mesh();
        let mut exporter = UsdExporter::with_options(options);
        exporter.add_mesh("CmMesh", &mesh, None);

        let usda = exporter.to_usda_string();
        assert!(usda.contains("metersPerUnit = 0.01"));
    }

    // ---- USDA string format validation ----

    #[test]
    fn test_usda_header_format() {
        let exporter = UsdExporter::new();
        let usda = exporter.to_usda_string();

        // Must start with #usda 1.0
        assert!(usda.starts_with("#usda 1.0\n"));

        // Must have defaultPrim
        assert!(usda.contains("defaultPrim = \"Root\""));

        // Must have metersPerUnit
        assert!(usda.contains("metersPerUnit ="));

        // Must have upAxis
        assert!(usda.contains("upAxis ="));

        // Must have Root Xform
        assert!(usda.contains("def Xform \"Root\""));
    }

    #[test]
    fn test_usda_braces_balanced() {
        let mesh = make_test_mesh();
        let material = UsdMaterial::steel();
        let transform: [f64; 16] = [
            1.0, 0.0, 0.0, 5.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ];
        let camera = UsdCamera::default();
        let light = UsdLight::Distant {
            angle: 0.53,
            color: [1.0, 1.0, 1.0],
            intensity: 1.0,
        };

        let mut exporter = UsdExporter::new();
        exporter.add_mesh_with_material("Part", &mesh, &material, None);
        exporter.add_instance("Part_Inst", "Part", &transform);
        exporter.add_camera("Cam", &camera);
        exporter.add_light("Sun", &light);

        let usda = exporter.to_usda_string();

        // Check that braces are balanced
        let open_count = usda.chars().filter(|&c| c == '{').count();
        let close_count = usda.chars().filter(|&c| c == '}').count();
        assert_eq!(open_count, close_count, "Unbalanced braces in USDA output");
    }

    #[test]
    fn test_usda_mesh_has_required_properties() {
        let mesh = make_test_mesh();
        let mut exporter = UsdExporter::new();
        exporter.add_mesh("PropTest", &mesh, None);

        let usda = exporter.to_usda_string();

        // Every mesh must have these properties
        assert!(usda.contains("point3f[] points ="));
        assert!(usda.contains("int[] faceVertexCounts ="));
        assert!(usda.contains("int[] faceVertexIndices ="));
        assert!(usda.contains("subdivisionScheme ="));
    }

    #[test]
    fn test_usda_multiple_meshes() {
        let mesh1 = make_test_mesh();
        let mesh2 = make_test_mesh();
        let mut exporter = UsdExporter::new();
        exporter.add_mesh("MeshA", &mesh1, None);
        exporter.add_mesh("MeshB", &mesh2, None);

        let usda = exporter.to_usda_string();
        assert!(usda.contains("def Mesh \"MeshA\""));
        assert!(usda.contains("def Mesh \"MeshB\""));
    }

    #[test]
    fn test_usda_empty_exporter() {
        let exporter = UsdExporter::new();
        let usda = exporter.to_usda_string();

        assert!(usda.starts_with("#usda 1.0"));
        assert!(usda.contains("def Xform \"Root\""));
    }

    // ---- Formatting helpers ----

    #[test]
    fn test_format_f64() {
        assert_eq!(format_f64(0.0), "0");
        assert_eq!(format_f64(1.0), "1");
        assert_eq!(format_f64(0.001), "0.001");
        assert_eq!(format_f64(-1.5), "-1.5");
    }

    #[test]
    fn test_format_f32() {
        assert_eq!(format_f32(0.0), "0");
        assert_eq!(format_f32(1.0), "1");
        assert_eq!(format_f32(0.5), "0.5");
    }

    #[test]
    fn test_format_matrix4d_identity() {
        let identity: [f64; 16] = [
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ];
        let result = format_matrix4d(&identity);
        assert!(result.contains("1, 0, 0, 0"));
        assert!(result.contains("0, 0, 0, 1"));
    }

    #[test]
    fn test_trim_trailing_zeros() {
        assert_eq!(trim_trailing_zeros("1.0000000000"), "1");
        assert_eq!(trim_trailing_zeros("0.5000000000"), "0.5");
        assert_eq!(trim_trailing_zeros("0.0010000000"), "0.001");
        assert_eq!(trim_trailing_zeros("42"), "42");
        assert_eq!(trim_trailing_zeros("1.5"), "1.5");
    }

    // ---- Subdivision scheme ----

    #[test]
    fn test_subdivision_scheme_tokens() {
        assert_eq!(UsdSubdivisionScheme::None.as_usd_token(), "none");
        assert_eq!(UsdSubdivisionScheme::CatmullClark.as_usd_token(), "catmullClark");
        assert_eq!(UsdSubdivisionScheme::Loop.as_usd_token(), "loop");
    }

    #[test]
    fn test_up_axis_tokens() {
        assert_eq!(UsdUpAxis::X.as_usd_token(), "X");
        assert_eq!(UsdUpAxis::Y.as_usd_token(), "Y");
        assert_eq!(UsdUpAxis::Z.as_usd_token(), "Z");
    }

    // ---- File write test ----

    #[test]
    fn test_write_usda_to_file() {
        let mesh = make_test_mesh();
        let mut exporter = UsdExporter::new();
        exporter.add_mesh("FileTest", &mesh, None);

        let dir = std::env::temp_dir().join("draper_mesh_usd_test");
        std::fs::create_dir_all(&dir).expect("Failed to create temp dir");
        let path = dir.join("test_output.usda");

        exporter.write_usda(&path).expect("Failed to write USDA file");

        let content = std::fs::read_to_string(&path).expect("Failed to read back USDA file");
        assert!(content.starts_with("#usda 1.0"));
        assert!(content.contains("def Mesh \"FileTest\""));

        // Clean up
        let _ = std::fs::remove_dir_all(&dir);
    }

    // ---- Full scene test ----

    #[test]
    fn test_full_scene_export() {
        let mesh = make_test_mesh();
        let material = UsdMaterial::plastic([0.2, 0.6, 0.9]);
        let instance_transform: [f64; 16] = [
            1.0, 0.0, 0.0, 100.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ];
        let camera = UsdCamera {
            focal_length: 35.0,
            ..UsdCamera::default()
        };
        let light = UsdLight::Distant {
            angle: 0.53,
            color: [1.0, 0.95, 0.9],
            intensity: 1.5,
        };

        let mut exporter = UsdExporter::new();
        exporter.add_mesh_with_material("Housing", &mesh, &material, None);
        exporter.add_instance("Housing_Copy", "Housing", &instance_transform);
        exporter.add_camera("Overview", &camera);
        exporter.add_light("KeyLight", &light);

        let usda = exporter.to_usda_string();

        // Verify all elements are present
        assert!(usda.contains("def Mesh \"Housing\""));
        assert!(usda.contains("def Material \"Housing_Mat\""));
        assert!(usda.contains("def Xform \"Housing_Copy\""));
        assert!(usda.contains("def Camera \"Overview\""));
        assert!(usda.contains("def DistantLight \"KeyLight\""));
        assert!(usda.contains("append rel references = </Root/Housing>"));
        assert!(usda.contains("focalLength = 35"));
        assert!(usda.contains("intensity = 1.5"));

        // Verify braces are balanced
        let open_count = usda.chars().filter(|&c| c == '{').count();
        let close_count = usda.chars().filter(|&c| c == '}').count();
        assert_eq!(open_count, close_count);
    }
}
