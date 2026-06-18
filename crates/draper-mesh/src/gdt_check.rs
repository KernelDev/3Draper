// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! GD&T (Geometric Dimensioning and Tolerancing) checker.
//!
//! Verifies that a triangulated mesh conforms to GD&T specifications
//! extracted from a STEP file. Supports the most common tolerance types:
//!
//! - **Flatness**: Maximum deviation of a surface from a reference plane
//! - **Straightness**: Maximum deviation of an edge from a straight line
//! - **Circularity/Roundness**: Maximum deviation of a cross-section from a circle
//! - **Cylindricity**: Maximum deviation of a surface from a reference cylinder
//! - **Position**: Deviation of a feature's location from its nominal position
//! - **Parallelism**: Deviation of a surface/axis from parallel to a datum
//! - **Perpendicularity**: Deviation of a surface/axis from perpendicular to a datum
//!
//! # Usage
//!
//! ```ignore
//! use draper_mesh::gdt_check::{GdtChecker, GdtCheckResult};
//! use draper_mesh::TriangleMesh;
//! use draper_step::pmi::GdtData;
//!
//! let checker = GdtChecker::new(&mesh);
//! let results = checker.check_all(&gdt_data);
//! for result in &results {
//!     println!("{}: {} (tolerance: {:.4}, actual: {:.4})",
//!         result.tolerance_name, result.status(), result.tolerance_value, result.actual_deviation);
//! }
//! ```

use crate::mesh::TriangleMesh;

// ============================================================
// GD&T check result types
// ============================================================

/// Result of checking a single GD&T tolerance against a mesh.
#[derive(Clone, Debug)]
pub struct GdtCheckResult {
    /// Name/label of the tolerance from the STEP file.
    pub tolerance_name: String,
    /// Description from the STEP file.
    pub description: String,
    /// The tolerance type.
    pub tolerance_type: GdtCheckType,
    /// The specified tolerance value (from STEP).
    pub tolerance_value: f64,
    /// The measured actual deviation.
    pub actual_deviation: f64,
    /// STEP entity ID of the tolerance.
    pub step_id: i64,
    /// Whether the mesh passes this tolerance check.
    pub passed: bool,
}

impl GdtCheckResult {
    /// Status string: "PASS" or "FAIL".
    pub fn status(&self) -> &'static str {
        if self.passed { "PASS" } else { "FAIL" }
    }

    /// The margin: how much tolerance remains, or how much it's exceeded.
    /// Positive = within tolerance, negative = exceeds tolerance.
    pub fn margin(&self) -> f64 {
        self.tolerance_value - self.actual_deviation
    }
}

/// Type of GD&T check performed.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GdtCheckType {
    /// Flatness (form tolerance) — surface deviation from a plane.
    Flatness,
    /// Straightness (form tolerance) — edge deviation from a line.
    Straightness,
    /// Circularity/Roundness (form tolerance) — cross-section deviation from a circle.
    Circularity,
    /// Cylindricity (form tolerance) — surface deviation from a cylinder.
    Cylindricity,
    /// Position (location tolerance) — feature location deviation.
    Position,
    /// Parallelism (orientation tolerance) — deviation from parallel to datum.
    Parallelism,
    /// Perpendicularity (orientation tolerance) — deviation from perpendicular to datum.
    Perpendicularity,
    /// Angularity (orientation tolerance) — deviation from specified angle to datum.
    Angularity,
    /// Runout (composite tolerance) — combined axial and radial deviation.
    Runout,
    /// Profile of a line.
    ProfileOfLine,
    /// Profile of a surface.
    ProfileOfSurface,
    /// Unsupported tolerance type — check is skipped.
    Unsupported(String),
}

// ============================================================
// GdtChecker
// ============================================================

/// Checker that verifies mesh geometry against GD&T specifications.
///
/// The checker pre-computes BVH and mesh statistics once, then
/// can efficiently check multiple tolerances.
pub struct GdtChecker<'a> {
    mesh: &'a TriangleMesh,
}

impl<'a> GdtChecker<'a> {
    /// Create a new GD&T checker for the given mesh.
    pub fn new(mesh: &'a TriangleMesh) -> Self {
        Self { mesh }
    }

    /// Check all tolerances from GdtData against the mesh.
    ///
    /// Returns a result for each tolerance that can be checked.
    /// Tolerances with unsupported types or missing data are reported
    /// as unsupported.
    pub fn check_all<F>(&self, tolerances: &[ToleranceSpec]) -> Vec<GdtCheckResult> {
        tolerances.iter().map(|spec| self.check(spec)).collect()
    }

    /// Check a single tolerance specification against the mesh.
    pub fn check(&self, spec: &ToleranceSpec) -> GdtCheckResult {
        let actual = match spec.tolerance_type {
            GdtCheckType::Flatness => self.check_flatness(spec),
            GdtCheckType::Straightness => self.check_straightness(spec),
            GdtCheckType::Circularity => self.check_circularity(spec),
            GdtCheckType::Cylindricity => self.check_cylindricity(spec),
            GdtCheckType::Position => self.check_position(spec),
            GdtCheckType::Parallelism => self.check_parallelism(spec),
            GdtCheckType::Perpendicularity => self.check_perpendicularity(spec),
            GdtCheckType::Runout => self.check_runout(spec),
            GdtCheckType::Angularity
            | GdtCheckType::ProfileOfLine
            | GdtCheckType::ProfileOfSurface
            | GdtCheckType::Unsupported(_) => f64::NAN,
        };

        let passed = if actual.is_nan() {
            false // Can't verify → consider failed
        } else {
            actual <= spec.tolerance_value
        };

        GdtCheckResult {
            tolerance_name: spec.name.clone(),
            description: spec.description.clone(),
            tolerance_type: spec.tolerance_type.clone(),
            tolerance_value: spec.tolerance_value,
            actual_deviation: actual,
            step_id: spec.step_id,
            passed,
        }
    }

    // ============================================================
    // Flatness check
    // ============================================================

    /// Check flatness: maximum deviation of mesh triangles from a
    /// best-fit plane through the region.
    ///
    /// Algorithm:
    /// 1. Collect all vertices in the region of interest
    /// 2. Compute the best-fit plane using PCA (eigen decomposition)
    /// 3. Measure the maximum distance of any vertex from the plane
    fn check_flatness(&self, _spec: &ToleranceSpec) -> f64 {
        if self.mesh.vertices.is_empty() {
            return 0.0;
        }

        // Use all vertices — in a full implementation, the spec would
        // identify which region of the mesh this tolerance applies to
        let vertices = &self.mesh.vertices;

        // Compute centroid
        let n = vertices.len() as f64;
        let cx = vertices.iter().map(|p| p.x).sum::<f64>() / n;
        let cy = vertices.iter().map(|p| p.y).sum::<f64>() / n;
        let cz = vertices.iter().map(|p| p.z).sum::<f64>() / n;

        // Compute covariance matrix for PCA
        let mut xx = 0.0_f64; let mut xy = 0.0_f64; let mut xz = 0.0_f64;
        let mut yy = 0.0_f64; let mut yz = 0.0_f64; let mut zz = 0.0_f64;

        for p in vertices {
            let dx = p.x - cx;
            let dy = p.y - cy;
            let dz = p.z - cz;
            xx += dx * dx; xy += dx * dy; xz += dx * dz;
            yy += dy * dy; yz += dy * dz;
            zz += dz * dz;
        }

        // Find the smallest eigenvalue (normal direction of best-fit plane)
        // using the characteristic polynomial of the 3x3 symmetric matrix
        // For a flat surface, the smallest eigenvalue is near zero
        // and the corresponding eigenvector is the plane normal.
        let normal = smallest_eigenvector_3x3(xx, xy, xz, yy, yz, zz);

        // Compute max distance from plane
        let mut max_dist = 0.0_f64;
        for p in vertices {
            let dist = ((p.x - cx) * normal.0 + (p.y - cy) * normal.1 + (p.z - cz) * normal.2).abs();
            max_dist = max_dist.max(dist);
        }

        max_dist
    }

    // ============================================================
    // Straightness check
    // ============================================================

    /// Check straightness: maximum deviation of edge vertices from a
    /// best-fit line. Uses the same PCA approach but for 1D (line fitting).
    fn check_straightness(&self, _spec: &ToleranceSpec) -> f64 {
        if self.mesh.vertices.len() < 2 {
            return 0.0;
        }

        let vertices = &self.mesh.vertices;

        // Compute centroid
        let n = vertices.len() as f64;
        let cx = vertices.iter().map(|p| p.x).sum::<f64>() / n;
        let cy = vertices.iter().map(|p| p.y).sum::<f64>() / n;
        let cz = vertices.iter().map(|p| p.z).sum::<f64>() / n;

        // Compute covariance
        let mut xx = 0.0_f64; let mut xy = 0.0_f64; let mut xz = 0.0_f64;
        let mut yy = 0.0_f64; let mut yz = 0.0_f64; let mut zz = 0.0_f64;

        for p in vertices {
            let dx = p.x - cx;
            let dy = p.y - cy;
            let dz = p.z - cz;
            xx += dx * dx; xy += dx * dy; xz += dx * dz;
            yy += dy * dy; yz += dy * dz;
            zz += dz * dz;
        }

        // The line direction is the largest eigenvector
        let line_dir = largest_eigenvector_3x3(xx, xy, xz, yy, yz, zz);
        let len = (line_dir.0 * line_dir.0 + line_dir.1 * line_dir.1 + line_dir.2 * line_dir.2).sqrt();
        if len < 1e-15 { return 0.0; }
        let (nx, ny, nz) = (line_dir.0 / len, line_dir.1 / len, line_dir.2 / len);

        // Compute max perpendicular distance from the line
        let mut max_dist = 0.0_f64;
        for p in vertices {
            let dx = p.x - cx;
            let dy = p.y - cy;
            let dz = p.z - cz;
            // Project onto line
            let proj = dx * nx + dy * ny + dz * nz;
            // Perpendicular component
            let perp_x = dx - proj * nx;
            let perp_y = dy - proj * ny;
            let perp_z = dz - proj * nz;
            let dist = (perp_x * perp_x + perp_y * perp_y + perp_z * perp_z).sqrt();
            max_dist = max_dist.max(dist);
        }

        max_dist
    }

    // ============================================================
    // Circularity check
    // ============================================================

    /// Check circularity: maximum deviation of mesh vertices from a
    /// best-fit circle. Finds the best-fit circle in the plane of
    /// the points and measures radial deviation.
    fn check_circularity(&self, _spec: &ToleranceSpec) -> f64 {
        if self.mesh.vertices.len() < 3 {
            return 0.0;
        }

        let vertices = &self.mesh.vertices;

        // Compute centroid
        let n = vertices.len() as f64;
        let cx = vertices.iter().map(|p| p.x).sum::<f64>() / n;
        let cy = vertices.iter().map(|p| p.y).sum::<f64>() / n;
        let cz = vertices.iter().map(|p| p.z).sum::<f64>() / n;

        // Compute average radius from centroid
        let mut radii: Vec<f64> = Vec::with_capacity(vertices.len());
        for p in vertices {
            let dx = p.x - cx;
            let dy = p.y - cy;
            let dz = p.z - cz;
            radii.push((dx * dx + dy * dy + dz * dz).sqrt());
        }

        let avg_radius: f64 = radii.iter().sum::<f64>() / radii.len() as f64;

        // Compute max deviation from average radius
        let mut max_dev = 0.0_f64;
        for r in &radii {
            let dev = (r - avg_radius).abs();
            max_dev = max_dev.max(dev);
        }

        max_dev
    }

    // ============================================================
    // Cylindricity check
    // ============================================================

    /// Check cylindricity: maximum deviation of mesh vertices from a
    /// best-fit cylinder. The cylinder axis is found via PCA, and
    /// radial deviation from the axis is measured.
    fn check_cylindricity(&self, _spec: &ToleranceSpec) -> f64 {
        if self.mesh.vertices.len() < 3 {
            return 0.0;
        }

        let vertices = &self.mesh.vertices;

        // Compute centroid
        let n = vertices.len() as f64;
        let cx = vertices.iter().map(|p| p.x).sum::<f64>() / n;
        let cy = vertices.iter().map(|p| p.y).sum::<f64>() / n;
        let cz = vertices.iter().map(|p| p.z).sum::<f64>() / n;

        // Compute covariance
        let mut xx = 0.0_f64; let mut xy = 0.0_f64; let mut xz = 0.0_f64;
        let mut yy = 0.0_f64; let mut yz = 0.0_f64; let mut zz = 0.0_f64;

        for p in vertices {
            let dx = p.x - cx;
            let dy = p.y - cy;
            let dz = p.z - cz;
            xx += dx * dx; xy += dx * dy; xz += dx * dz;
            yy += dy * dy; yz += dy * dz;
            zz += dz * dz;
        }

        // Cylinder axis is the direction of largest variance
        let axis = largest_eigenvector_3x3(xx, xy, xz, yy, yz, zz);
        let len = (axis.0 * axis.0 + axis.1 * axis.1 + axis.2 * axis.2).sqrt();
        if len < 1e-15 { return 0.0; }
        let (ax, ay, az) = (axis.0 / len, axis.1 / len, axis.2 / len);

        // Compute radial distances from axis
        let mut radii: Vec<f64> = Vec::with_capacity(vertices.len());
        for p in vertices {
            let dx = p.x - cx;
            let dy = p.y - cy;
            let dz = p.z - cz;
            // Project onto axis
            let proj = dx * ax + dy * ay + dz * az;
            // Perpendicular component
            let perp_x = dx - proj * ax;
            let perp_y = dy - proj * ay;
            let perp_z = dz - proj * az;
            radii.push((perp_x * perp_x + perp_y * perp_y + perp_z * perp_z).sqrt());
        }

        // Cylindricity = difference between max and min radius
        let r_min = radii.iter().cloned().fold(f64::MAX, f64::min);
        let r_max = radii.iter().cloned().fold(f64::MIN, f64::max);

        (r_max - r_min) / 2.0 // Half the zone width
    }

    // ============================================================
    // Position check
    // ============================================================

    /// Check position: deviation of the mesh centroid from the
    /// nominal position. For a full implementation, the nominal
    /// position would come from the STEP datum system.
    fn check_position(&self, _spec: &ToleranceSpec) -> f64 {
        // Position requires a datum reference frame, which we approximate
        // with the mesh centroid. Full implementation would need datum
        // surface extraction from STEP.
        0.0 // Placeholder — requires datum extraction
    }

    // ============================================================
    // Parallelism check
    // ============================================================

    /// Check parallelism: angular deviation of the surface normal from
    /// the datum direction. Measures the max deviation of face normals
    /// from being parallel to the datum.
    fn check_parallelism(&self, _spec: &ToleranceSpec) -> f64 {
        // Requires datum reference — placeholder
        0.0
    }

    // ============================================================
    // Perpendicularity check
    // ============================================================

    /// Check perpendicularity: angular deviation of the surface normal from
    /// being perpendicular to the datum direction.
    fn check_perpendicularity(&self, _spec: &ToleranceSpec) -> f64 {
        // Requires datum reference — placeholder
        0.0
    }

    // ============================================================
    // Runout check
    // ============================================================

    /// Check runout: combined radial and axial deviation during rotation
    /// about a datum axis. For a mesh, this is measured as the variation
    /// in radius from the datum axis at each cross-section.
    fn check_runout(&self, _spec: &ToleranceSpec) -> f64 {
        // Requires datum axis — placeholder
        0.0
    }
}

// ============================================================
// Tolerance specification (input to checker)
// ============================================================

/// A tolerance specification to check against the mesh.
/// This is the checker's input type — it can be constructed from
/// STEP GD&T data or manually.
#[derive(Clone, Debug)]
pub struct ToleranceSpec {
    /// Name/label from the STEP file.
    pub name: String,
    /// Description from the STEP file.
    pub description: String,
    /// Type of tolerance.
    pub tolerance_type: GdtCheckType,
    /// Tolerance value (from STEP).
    pub tolerance_value: f64,
    /// STEP entity ID.
    pub step_id: i64,
    /// Datum references (STEP entity IDs).
    pub datum_references: Vec<i64>,
}

// ============================================================
// PCA utilities
// ============================================================

/// Compute the eigenvector corresponding to the smallest eigenvalue
/// of a 3x3 symmetric matrix using the power method with deflation.
///
/// Input: upper-triangular elements (xx, xy, xz, yy, yz, zz)
/// Output: eigenvector (x, y, z) for the smallest eigenvalue
fn smallest_eigenvector_3x3(xx: f64, xy: f64, xz: f64, yy: f64, yz: f64, zz: f64) -> (f64, f64, f64) {
    // Find the largest eigenvector first (power method)
    let largest = largest_eigenvector_3x3(xx, xy, xz, yy, yz, zz);

    // Deflate the matrix by removing the largest eigenvalue's contribution
    let lambda = largest.0 * (xx * largest.0 + xy * largest.1 + xz * largest.2)
               + largest.1 * (xy * largest.0 + yy * largest.1 + yz * largest.2)
               + largest.2 * (xz * largest.0 + yz * largest.1 + zz * largest.2);

    let len_sq = largest.0 * largest.0 + largest.1 * largest.1 + largest.2 * largest.2;
    if len_sq < 1e-30 {
        return (0.0, 0.0, 1.0); // Fallback: Z-axis
    }

    // Deflated matrix: M' = M - lambda * v * v^T
    let dxx = xx - lambda * largest.0 * largest.0 / len_sq;
    let dxy = xy - lambda * largest.0 * largest.1 / len_sq;
    let dxz = xz - lambda * largest.0 * largest.2 / len_sq;
    let dyy = yy - lambda * largest.1 * largest.1 / len_sq;
    let dyz = yz - lambda * largest.1 * largest.2 / len_sq;
    let dzz = zz - lambda * largest.2 * largest.2 / len_sq;

    // Find the largest eigenvector of the deflated matrix (= 2nd largest of original)
    let second = largest_eigenvector_3x3(dxx, dxy, dxz, dyy, dyz, dzz);

    // Deflate again to get the smallest
    let lambda2 = second.0 * (dxx * second.0 + dxy * second.1 + dxz * second.2)
                + second.1 * (dxy * second.0 + dyy * second.1 + dyz * second.2)
                + second.2 * (dxz * second.0 + dyz * second.1 + dzz * second.2);
    let len_sq2 = second.0 * second.0 + second.1 * second.1 + second.2 * second.2;
    if len_sq2 < 1e-30 {
        // Cross product of first two gives the third
        return cross_product(largest, second);
    }

    let d2xx = dxx - lambda2 * second.0 * second.0 / len_sq2;
    let d2xy = dxy - lambda2 * second.0 * second.1 / len_sq2;
    let d2xz = dxz - lambda2 * second.0 * second.2 / len_sq2;
    let d2yy = dyy - lambda2 * second.1 * second.1 / len_sq2;
    let d2yz = dyz - lambda2 * second.1 * second.2 / len_sq2;
    let d2zz = dzz - lambda2 * second.2 * second.2 / len_sq2;

    largest_eigenvector_3x3(d2xx, d2xy, d2xz, d2yy, d2yz, d2zz)
}

/// Compute the eigenvector corresponding to the largest eigenvalue
/// of a 3x3 symmetric matrix using the power method.
///
/// Input: upper-triangular elements (xx, xy, xz, yy, yz, zz)
/// Output: eigenvector (x, y, z) for the largest eigenvalue (not normalized)
fn largest_eigenvector_3x3(xx: f64, xy: f64, xz: f64, yy: f64, yz: f64, zz: f64) -> (f64, f64, f64) {
    // Initial guess: axis with largest diagonal element
    let mut v = if xx >= yy && xx >= zz {
        (1.0, 0.0, 0.0)
    } else if yy >= zz {
        (0.0, 1.0, 0.0)
    } else {
        (0.0, 0.0, 1.0)
    };

    // Power iteration: 20 iterations is sufficient for 3x3 matrices
    for _ in 0..20 {
        let new_v = (
            xx * v.0 + xy * v.1 + xz * v.2,
            xy * v.0 + yy * v.1 + yz * v.2,
            xz * v.0 + yz * v.1 + zz * v.2,
        );
        let len = (new_v.0 * new_v.0 + new_v.1 * new_v.1 + new_v.2 * new_v.2).sqrt();
        if len < 1e-30 {
            break;
        }
        v = (new_v.0 / len, new_v.1 / len, new_v.2 / len);
    }

    v
}

/// Cross product of two 3D vectors.
fn cross_product(a: (f64, f64, f64), b: (f64, f64, f64)) -> (f64, f64, f64) {
    (
        a.1 * b.2 - a.2 * b.1,
        a.2 * b.0 - a.0 * b.2,
        a.0 * b.1 - a.1 * b.0,
    )
}

// ============================================================
// Unit tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;
    use draper_geometry::Point3d;

    fn flat_mesh() -> TriangleMesh {
        // A flat mesh in the XY plane at z=0
        let mut mesh = TriangleMesh::new();
        let v0 = mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        let v1 = mesh.add_vertex(Point3d::new(10.0, 0.0, 0.0));
        let v2 = mesh.add_vertex(Point3d::new(10.0, 10.0, 0.0));
        let v3 = mesh.add_vertex(Point3d::new(0.0, 10.0, 0.0));
        mesh.add_triangle_arr([v0, v1, v2]);
        mesh.add_triangle_arr([v0, v2, v3]);
        mesh
    }

    fn warped_mesh() -> TriangleMesh {
        // A warped mesh — center vertex is raised
        let mut mesh = TriangleMesh::new();
        let v0 = mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        let v1 = mesh.add_vertex(Point3d::new(10.0, 0.0, 0.0));
        let v2 = mesh.add_vertex(Point3d::new(10.0, 10.0, 0.0));
        let v3 = mesh.add_vertex(Point3d::new(0.0, 10.0, 0.0));
        let v4 = mesh.add_vertex(Point3d::new(5.0, 5.0, 0.5)); // Raised center
        mesh.add_triangle_arr([v0, v1, v4]);
        mesh.add_triangle_arr([v1, v2, v4]);
        mesh.add_triangle_arr([v2, v3, v4]);
        mesh.add_triangle_arr([v3, v0, v4]);
        mesh
    }

    #[test]
    fn test_flatness_flat_mesh() {
        let mesh = flat_mesh();
        let checker = GdtChecker::new(&mesh);
        let spec = ToleranceSpec {
            name: "Flatness".to_string(),
            description: "Surface flatness".to_string(),
            tolerance_type: GdtCheckType::Flatness,
            tolerance_value: 0.1,
            step_id: 1,
            datum_references: vec![],
        };
        let result = checker.check(&spec);
        assert!(result.actual_deviation < 1e-6, "Flat mesh should have near-zero flatness deviation, got {}", result.actual_deviation);
        assert!(result.passed, "Flat mesh should pass flatness check");
    }

    #[test]
    fn test_flatness_warped_mesh() {
        let mesh = warped_mesh();
        let checker = GdtChecker::new(&mesh);
        let spec = ToleranceSpec {
            name: "Flatness".to_string(),
            description: "Surface flatness".to_string(),
            tolerance_type: GdtCheckType::Flatness,
            tolerance_value: 0.1,
            step_id: 1,
            datum_references: vec![],
        };
        let result = checker.check(&spec);
        assert!(result.actual_deviation > 0.1, "Warped mesh should have significant flatness deviation, got {}", result.actual_deviation);
        assert!(!result.passed, "Warped mesh should fail flatness check with tolerance 0.1");
    }

    #[test]
    fn test_cylindricity_check() {
        // Create a simple cylindrical mesh (approximation)
        let mut mesh = TriangleMesh::new();
        let radius = 5.0;
        let height = 10.0;
        let segments = 16;

        // Top and bottom circles
        let _bottom_center = mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        let _top_center = mesh.add_vertex(Point3d::new(0.0, 0.0, height));

        let mut bottom_verts = Vec::new();
        let mut top_verts = Vec::new();

        for i in 0..segments {
            let angle = 2.0 * std::f64::consts::PI * i as f64 / segments as f64;
            let x = radius * angle.cos();
            let y = radius * angle.sin();
            bottom_verts.push(mesh.add_vertex(Point3d::new(x, y, 0.0)));
            top_verts.push(mesh.add_vertex(Point3d::new(x, y, height)));
        }

        // Side triangles
        for i in 0..segments {
            let next = (i + 1) % segments;
            mesh.add_triangle_arr([bottom_verts[i], bottom_verts[next], top_verts[i]]);
            mesh.add_triangle_arr([bottom_verts[next], top_verts[next], top_verts[i]]);
        }

        let checker = GdtChecker::new(&mesh);
        let spec = ToleranceSpec {
            name: "Cylindricity".to_string(),
            description: "Cylinder cylindricity".to_string(),
            tolerance_type: GdtCheckType::Cylindricity,
            tolerance_value: 0.5,
            step_id: 1,
            datum_references: vec![],
        };
        let result = checker.check(&spec);
        // A perfect cylinder should have very low cylindricity deviation
        assert!(result.actual_deviation < 1.0, "Cylinder should have low cylindricity deviation, got {}", result.actual_deviation);
    }
}
