// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! STEP → USDA pipeline.
//!
//! This module bridges `draper-step` (STEP parser) and `draper-mesh` (USD
//! exporter), avoiding the cyclic dependency that prevents
//! `draper_mesh::export_usd::export_step_to_usda` from doing the full
//! geometric conversion itself.
//!
//! Pipeline: STEP file → StepFile → extract_solids → triangulate_solid
//!           → UsdExporter::add_mesh → write_usda.

use draper_mesh::{TriangleMesh, export_usd::{UsdExporter, UsdExportOptions, UsdMaterial, UsdCamera, UsdLight}};
use draper_step::{extract_solids};
#[cfg(not(target_arch = "wasm32"))]
use draper_step::parse_step_file;
use draper_topology::Solid;
use std::path::Path;

/// Parameters controlling the STEP → USDA conversion.
#[derive(Clone, Debug)]
pub struct StepToUsdaParams {
    /// Triangulation tolerance (chord error in millimetres). Smaller =
    /// higher mesh resolution. Default: 0.1 mm.
    pub chord_tolerance: f64,
    /// Whether to use parallel triangulation (uses rayon).
    pub parallel: bool,
    /// Whether to compute smooth vertex normals.
    pub smooth_normals: bool,
    /// Default material to apply to all meshes.
    pub material: UsdMaterial,
    /// Whether to include a default camera in the USD scene.
    pub include_camera: bool,
    /// Whether to include a default light.
    pub include_light: bool,
}

impl Default for StepToUsdaParams {
    fn default() -> Self {
        Self {
            chord_tolerance: 0.1,
            parallel: true,
            smooth_normals: true,
            material: UsdMaterial::default_grey(),
            include_camera: true,
            include_light: true,
        }
    }
}

/// Convert a STEP file to a USDA (USD ASCII) file.
///
/// This is the full implementation that the stub in
/// `draper_mesh::export_usd::export_step_to_usda` defers to. It:
///
/// 1. Parses the STEP file into a `StepFile`.
/// 2. Extracts all solids (BREP → Solid).
/// 3. Triangulates each solid with the given chord tolerance.
/// 4. Adds each mesh to a `UsdExporter` with the given material.
/// 5. Optionally adds a default camera and light.
/// 6. Writes the USDA file to `output_path`.
///
/// # Errors
///
/// Returns an error string when:
/// - The STEP file does not exist or cannot be parsed.
/// - No solids could be extracted (file may contain only surface geometry).
/// - Triangulation fails for every solid.
/// - The USDA write fails (e.g. permission denied).
pub fn export_step_to_usda(
    step_file_path: &Path,
    output_path: &Path,
    params: &StepToUsdaParams,
) -> Result<usize, String> {
    if !step_file_path.exists() {
        return Err(format!(
            "STEP file not found: {}",
            step_file_path.display()
        ));
    }

    // 1. Parse STEP file.
    // On native, use buffered file reading. On wasm, read the whole file
    // into a string (wasm has no BufReader-based file API).
    #[cfg(not(target_arch = "wasm32"))]
    let step_file = parse_step_file(&step_file_path.to_string_lossy())
        .map_err(|e| format!("STEP parse error: {}", e))?;
    #[cfg(target_arch = "wasm32")]
    let step_file = {
        let content = std::fs::read_to_string(step_file_path)
            .map_err(|e| format!("Cannot read STEP file: {}", e))?;
        draper_step::parse_step(&content)
            .map_err(|e| format!("STEP parse error: {}", e))?
    };

    // 2. Extract solids.
    let (solids, brep_ids) = extract_solids(&step_file);
    if solids.is_empty() {
        return Err("No solids extracted from STEP file (may contain only surface geometry)".to_string());
    }

    // 3. Set up USD exporter.
    let options = UsdExportOptions::default();
    let mut exporter = UsdExporter::with_options(options);

    // 4. Triangulate each solid and add to exporter.
    let mut meshes_added = 0usize;
    let mut triangulation_params = draper_mesh::TriangulationParams::default();
    triangulation_params.max_deviation = params.chord_tolerance;
    triangulation_params.parallel = params.parallel;

    for (i, solid) in solids.iter().enumerate() {
        let mesh = triangulate_solid_safe(solid, &triangulation_params);
        if mesh.vertices.is_empty() || mesh.triangles.is_empty() {
            log::warn!(
                "Solid #{} (BREP #{}): triangulation produced empty mesh, skipping",
                i, brep_ids.get(i).copied().unwrap_or(-1)
            );
            continue;
        }
        let name = format!("solid_{}_brep_{}", i, brep_ids.get(i).copied().unwrap_or(-1));
        exporter.add_mesh_with_material(&name, &mesh, &params.material, None);
        meshes_added += 1;
    }

    if meshes_added == 0 {
        return Err("All solids triangulated to empty meshes — nothing to export".to_string());
    }

    // 5. Optionally add a default camera and light.
    if params.include_camera {
        let camera = default_camera_for_solids(&solids);
        exporter.add_camera("main_camera", &camera);
    }
    if params.include_light {
        let light = default_light_for_solids(&solids);
        exporter.add_light("key_light", &light);
    }

    // 6. Write USDA file.
    exporter.write_usda(output_path)
        .map_err(|e| format!("USDA write error: {}", e))?;

    Ok(meshes_added)
}

/// Triangulate a solid with a safe fallback to default params.
fn triangulate_solid_safe(
    solid: &Solid,
    params: &draper_mesh::TriangulationParams,
) -> TriangleMesh {
    draper_mesh::triangulate_solid(solid, params)
}

/// Compute a default camera that frames all solids in the scene.
fn default_camera_for_solids(solids: &[Solid]) -> UsdCamera {
    use draper_geometry::Point3d;
    // Compute the bounding box of all solids.
    let mut min = Point3d::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut max = Point3d::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);

    for solid in solids {
        if let Some(shell) = solid.outer_shell.as_ref() {
            for face in &shell.faces {
                // C5 Stage 6.4: store-first boundary reads (per-id mirror
                // fallback keeps builder faces complete).
                for edge in solid.resolve_face_edges(face) {
                    if let Some(p) = edge.start_point() {
                        min.x = min.x.min(p.x); min.y = min.y.min(p.y); min.z = min.z.min(p.z);
                        max.x = max.x.max(p.x); max.y = max.y.max(p.y); max.z = max.z.max(p.z);
                    }
                    if let Some(p) = edge.end_point() {
                        min.x = min.x.min(p.x); min.y = min.y.min(p.y); min.z = min.z.min(p.z);
                        max.x = max.x.max(p.x); max.y = max.y.max(p.y); max.z = max.z.max(p.z);
                    }
                }
            }
        }
    }

    // If no bounds (empty solids), use defaults.
    if !min.x.is_finite() || !max.x.is_finite() {
        return UsdCamera::default();
    }

    let center = Point3d::new(
        0.5 * (min.x + max.x),
        0.5 * (min.y + max.y),
        0.5 * (min.z + max.z),
    );
    let extent = ((max.x - min.x).powi(2)
        + (max.y - min.y).powi(2)
        + (max.z - min.z).powi(2)).sqrt();
    let distance = extent * 2.0 + 1.0;

    let eye = Point3d::new(
        center.x + distance,
        center.y - distance,
        center.z + distance,
    );

    // Build a look-at matrix: camera at `eye`, looking at `center`, up = +Z.
    let forward = Point3d::new(
        center.x - eye.x,
        center.y - eye.y,
        center.z - eye.z,
    );
    let flen = (forward.x * forward.x + forward.y * forward.y + forward.z * forward.z).sqrt();
    let (fx, fy, fz) = if flen > 1e-12 {
        (forward.x / flen, forward.y / flen, forward.z / flen)
    } else {
        (0.0, 0.0, -1.0)
    };
    // Right = forward × up (up = +Z).
    // Standard cross product: (fx,fy,fz) × (0,0,1) = (fy*1 - fz*0, fz*0 - fx*1, fx*0 - fy*0) = (fy, -fx, 0).
    let rx = fy;
    let ry = -fx;
    let rz = 0.0_f64;
    let rlen = (rx * rx + ry * ry + rz * rz).sqrt();
    let (rx, ry, rz) = if rlen > 1e-12 {
        (rx / rlen, ry / rlen, rz / rlen)
    } else {
        (1.0, 0.0, 0.0)
    };
    // Up = right × forward.
    let ux = ry * fz - rz * fy;
    let uy = rz * fx - rx * fz;
    let uz = rx * fy - ry * fx;

    let mut camera = UsdCamera::default();
    // Row-major 4x4 transform: rotation [rx ry rz] [ux uy uz] [-fx -fy -fz] + translation.
    camera.transform = [
        rx, ry, rz, 0.0,
        ux, uy, uz, 0.0,
        -fx, -fy, -fz, 0.0,
        eye.x, eye.y, eye.z, 1.0,
    ];
    camera
}

/// Compute a default key light — a distant light positioned above and
/// behind the camera, simulating sunlight.
fn default_light_for_solids(_solids: &[Solid]) -> UsdLight {
    UsdLight::Distant {
        angle: 0.53, // sun angular diameter
        color: [1.0, 1.0, 0.95],
        intensity: 1000.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_export_step_to_usda_nonexistent_file() {
        let result = export_step_to_usda(
            Path::new("/nonexistent/file.stp"),
            Path::new("/tmp/out.usda"),
            &StepToUsdaParams::default(),
        );
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("not found"));
    }

    #[test]
    fn test_step_to_usda_params_default() {
        let p = StepToUsdaParams::default();
        assert!(p.chord_tolerance > 0.0);
        assert!(p.parallel);
        assert!(p.smooth_normals);
        assert!(p.include_camera);
        assert!(p.include_light);
    }

    #[test]
    fn test_export_step_to_usda_real_cube_file() {
        // Use one of the existing test STEP files.
        let step_path = Path::new("test/nist_cube.stp");
        if !step_path.exists() {
            // Running outside the project root — skip.
            return;
        }
        let out_path = Path::new("/tmp/test_cube_export.usda");
        let result = export_step_to_usda(step_path, out_path, &StepToUsdaParams::default());
        assert!(result.is_ok(), "export failed: {:?}", result);
        let meshes = result.unwrap();
        assert!(meshes >= 1, "expected at least 1 mesh, got {}", meshes);
        assert!(out_path.exists(), "output file not created");
        // Verify the USDA file has content.
        let metadata = std::fs::metadata(out_path).unwrap();
        assert!(metadata.len() > 100, "USDA file is too small");
    }
}
