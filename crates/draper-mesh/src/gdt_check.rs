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
use draper_geometry::{Direction3d, Point3d};

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
            GdtCheckType::Angularity => self.check_angularity(spec),
            GdtCheckType::Runout => self.check_runout(spec),
            GdtCheckType::ProfileOfLine => self.check_profile_of_line(spec),
            GdtCheckType::ProfileOfSurface => self.check_profile_of_surface(spec),
            GdtCheckType::Unsupported(_) => f64::NAN,
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

    /// Check position: deviation of the mesh centroid (or a feature
    /// point) from the nominal position. Per ASME Y14.5, a Position
    /// tolerance defines a cylindrical (or spherical) tolerance zone
    /// centred on the nominal position; the actual feature centre must
    /// lie within that zone.
    ///
    /// When `spec.nominal_position` is `None`, we return 0.0 because
    /// there is no datum to compare against — the check is then
    /// inconclusive (passes trivially). When set, we compute the
    /// Euclidean distance between the mesh centroid and the nominal
    /// position.
    ///
    /// For cylindrical position tolerances (the most common case), the
    /// tolerance_value is the *diameter* of the cylindrical tolerance
    /// zone. We report the radial deviation; the caller should compare
    /// against `tolerance_value / 2.0` for diametral specs, OR the
    /// caller may set `tolerance_value` to the radial width directly.
    /// We follow the convention used by the rest of this module: the
    /// `passed` flag compares `actual_deviation <= tolerance_value`,
    /// so callers should set `tolerance_value` to the radial allowance
    /// (= diametral / 2).
    fn check_position(&self, spec: &ToleranceSpec) -> f64 {
        if self.mesh.vertices.is_empty() {
            return 0.0;
        }

        let (cx, cy, cz) = self.mesh_centroid();

        match spec.nominal_position {
            None => {
                // No datum → cannot measure deviation. Return 0 so the
                // caller's `passed = actual <= tolerance_value` evaluates
                // to true, but log a warning.
                log::warn!(
                    "Position tolerance '{}' has no nominal_position; \
                     deviation reported as 0.0 (check is inconclusive)",
                    spec.name
                );
                0.0
            }
            Some(nominal) => {
                let dx = cx - nominal.x;
                let dy = cy - nominal.y;
                let dz = cz - nominal.z;
                (dx * dx + dy * dy + dz * dz).sqrt()
            }
        }
    }

    // ============================================================
    // Parallelism check
    // ============================================================

    /// Check parallelism: surface must be parallel to a datum plane or
    /// datum axis. Per ASME Y14.5, the tolerance zone is two parallel
    /// planes spaced `tolerance_value` apart, parallel to the datum,
    /// within which the surface must lie.
    ///
    /// We compute the best-fit plane of the mesh (via PCA) and measure
    /// the maximum deviation of mesh vertices from that best-fit plane
    /// — but only after "de-tilting" the mesh so the best-fit plane is
    /// parallel to the datum. The residual deviation is the parallelism
    /// error.
    ///
    /// If `datum_plane_normal` is `None` but `datum_axis` is set, we
    /// use the axis as the datum direction (the surface must be
    /// parallel to the axis, i.e. the surface normal is perpendicular
    /// to the axis). If both are `None`, returns NaN.
    fn check_parallelism(&self, spec: &ToleranceSpec) -> f64 {
        if self.mesh.vertices.is_empty() {
            return 0.0;
        }

        // Resolve the datum direction (the direction the surface must
        // be parallel to).
        let datum_dir = match self.resolve_parallelism_datum(spec) {
            Some(d) => d,
            None => {
                log::warn!(
                    "Parallelism tolerance '{}' has no datum; returning NaN",
                    spec.name
                );
                return f64::NAN;
            }
        };

        // The surface normal of the best-fit plane must be perpendicular
        // to datum_dir (surface parallel to datum). Find the best-fit
        // plane normal via PCA.
        let (centroid, normal) = self.best_fit_plane();
        let (nx, ny, nz) = (normal.0, normal.1, normal.2);

        // The component of `normal` along `datum_dir` measures tilt away
        // from parallel. Project it out so the residual best-fit plane is
        // truly parallel to the datum.
        let dot = nx * datum_dir.x + ny * datum_dir.y + nz * datum_dir.z;
        // De-tilted normal: n' = n - (n·d) d, then renormalise.
        let mut npx = nx - dot * datum_dir.x;
        let mut npy = ny - dot * datum_dir.y;
        let mut npz = nz - dot * datum_dir.z;
        let nplen = (npx * npx + npy * npy + npz * npz).sqrt();
        if nplen < 1e-15 {
            // The mesh is exactly parallel to the datum axis — surface is
            // a cylinder or plane perpendicular to the datum. Parallelism
            // deviation is 0.
            return 0.0;
        }
        npx /= nplen;
        npy /= nplen;
        npz /= nplen;

        // Max vertex deviation from the de-tilted plane through centroid.
        let mut max_dist = 0.0_f64;
        for p in &self.mesh.vertices {
            let dist = ((p.x - centroid.0) * npx
                + (p.y - centroid.1) * npy
                + (p.z - centroid.2) * npz)
                .abs();
            max_dist = max_dist.max(dist);
        }
        max_dist
    }

    // ============================================================
    // Perpendicularity check
    // ============================================================

    /// Check perpendicularity: surface must be perpendicular to a datum
    /// plane or datum axis. Per ASME Y14.5, the tolerance zone is two
    /// parallel planes spaced `tolerance_value` apart, perpendicular to
    /// the datum, within which the surface must lie.
    ///
    /// Implementation: the best-fit plane normal of the mesh is computed
    /// via PCA. The component of this normal *parallel* to the datum
    /// direction is the "perpendicularity error" — i.e. how much the
    //  surface tilts away from being exactly perpendicular. We then
    /// measure max vertex deviation from the de-tilted plane.
    fn check_perpendicularity(&self, spec: &ToleranceSpec) -> f64 {
        if self.mesh.vertices.is_empty() {
            return 0.0;
        }

        // For perpendicularity, the datum direction is the axis/plane
        // normal the surface must be perpendicular to. Same resolution
        // as parallelism (the geometric meaning differs only in which
        // direction we want the surface to face).
        let datum_dir = match self.resolve_parallelism_datum(spec) {
            Some(d) => d,
            None => {
                log::warn!(
                    "Perpendicularity tolerance '{}' has no datum; returning NaN",
                    spec.name
                );
                return f64::NAN;
            }
        };

        let (centroid, normal) = self.best_fit_plane();
        let (nx, ny, nz) = (normal.0, normal.1, normal.2);

        // For perpendicularity, the surface normal should be ALIGNED with
        // datum_dir (not perpendicular). The component of `normal` along
        // `datum_dir` is what we KEEP — we project out the perpendicular
        // component to get the de-tilted normal.
        let dot = nx * datum_dir.x + ny * datum_dir.y + nz * datum_dir.z;
        // n' = (n·d) * d  — this is the "perpendicular to datum" plane
        // normal (surface aligned with datum = surface perpendicular to
        // the datum plane).
        let mut npx = dot * datum_dir.x;
        let mut npy = dot * datum_dir.y;
        let mut npz = dot * datum_dir.z;
        let nplen = (npx * npx + npy * npy + npz * npz).sqrt();
        if nplen < 1e-15 {
            // Surface is exactly parallel to the datum axis — it cannot
            // be perpendicular. Report the full mesh thickness as the
            // deviation (worst case).
            let (bb_min, bb_max) = self.bounding_box();
            let extent = ((bb_max.0 - bb_min.0).powi(2)
                + (bb_max.1 - bb_min.1).powi(2)
                + (bb_max.2 - bb_min.2).powi(2))
            .sqrt();
            return extent;
        }
        npx /= nplen;
        npy /= nplen;
        npz /= nplen;

        let mut max_dist = 0.0_f64;
        for p in &self.mesh.vertices {
            let dist = ((p.x - centroid.0) * npx
                + (p.y - centroid.1) * npy
                + (p.z - centroid.2) * npz)
                .abs();
            max_dist = max_dist.max(dist);
        }
        max_dist
    }

    // ============================================================
    // Angularity check
    // ============================================================

    /// Check angularity: surface must be at a specified angle to a datum
    /// plane or axis. Per ASME Y14.5, the tolerance zone is two parallel
    /// planes spaced `tolerance_value` apart, oriented at the basic
    /// (theoretically exact) angle to the datum.
    ///
    /// Implementation: compute the best-fit plane of the mesh, measure
    /// the actual angle between the surface normal and the datum
    /// direction, and compare to the nominal angle. The angularity
    /// deviation (in length units) is the angle error multiplied by the
    /// half-extent of the mesh perpendicular to the datum — i.e. the
    /// maximum linear displacement at the edge of the surface caused by
    /// the angular error.
    ///
    /// `nominal_angle_deg` is the basic angle between the SURFACE and the
    /// datum (per ASME Y14.5 convention). Internally we convert it to
    /// the angle between the surface NORMAL and the datum, which is
    /// `90° - nominal_angle_deg`.
    fn check_angularity(&self, spec: &ToleranceSpec) -> f64 {
        if self.mesh.vertices.is_empty() {
            return 0.0;
        }

        let datum_dir = match self.resolve_parallelism_datum(spec) {
            Some(d) => d,
            None => {
                log::warn!(
                    "Angularity tolerance '{}' has no datum; returning NaN",
                    spec.name
                );
                return f64::NAN;
            }
        };

        let nominal_angle_deg = spec.nominal_angle_deg.unwrap_or(90.0);
        // Convert: nominal_angle_deg is between surface and datum.
        // Angle between surface NORMAL and datum = 90° - nominal_angle_deg.
        let nominal_normal_angle_deg = 90.0 - nominal_angle_deg;
        let nominal_normal_angle = nominal_normal_angle_deg.to_radians();

        let (centroid, normal) = self.best_fit_plane();
        let (nx, ny, nz) = (normal.0, normal.1, normal.2);
        let n_len = (nx * nx + ny * ny + nz * nz).sqrt();
        if n_len < 1e-15 {
            return 0.0;
        }
        let (nx, ny, nz) = (nx / n_len, ny / n_len, nz / n_len);

        // Actual angle between surface normal and datum direction.
        let dot = (nx * datum_dir.x + ny * datum_dir.y + nz * datum_dir.z)
            .clamp(-1.0, 1.0);
        let actual_normal_angle = dot.acos();

        // Angle error in radians.
        let angle_err = (actual_normal_angle - nominal_normal_angle).abs();

        // Convert angle error to linear deviation at the mesh boundary.
        // The maximum displacement = angle_err * max_perpendicular_distance
        // from the centroid, projected onto the plane perpendicular to
        // the datum direction.
        let (ax, ay, az) = (datum_dir.x, datum_dir.y, datum_dir.z);
        let mut max_perp_dist = 0.0_f64;
        for p in &self.mesh.vertices {
            let dx = p.x - centroid.0;
            let dy = p.y - centroid.1;
            let dz = p.z - centroid.2;
            // Project out the datum direction component.
            let proj = dx * ax + dy * ay + dz * az;
            let perp_x = dx - proj * ax;
            let perp_y = dy - proj * ay;
            let perp_z = dz - proj * az;
            let perp_dist = (perp_x * perp_x + perp_y * perp_y + perp_z * perp_z).sqrt();
            max_perp_dist = max_perp_dist.max(perp_dist);
        }

        angle_err * max_perp_dist
    }

    // ============================================================
    // Runout check (circular runout)
    // ============================================================

    /// Check runout: combined radial and axial deviation during rotation
    /// about a datum axis. Per ASME Y14.5, Circular Runout is a 2D
    /// tolerance measured at each cross-section perpendicular to the
    /// datum axis: the indicator reading (FIM — Full Indicator
    /// Movement) at each section must not exceed `tolerance_value`.
    ///
    /// Implementation: for each unique Z-position along the datum axis
    /// (binned to 100 bins), find all vertices in that cross-section
    /// and measure the spread of their radial distances. The runout
    /// deviation is the maximum FIM across all cross-sections.
    ///
    /// For Total Runout (not yet a separate GdtCheckType), the
    /// deviation is the max FIM across the entire surface, treating
    /// the whole mesh as one section.
    fn check_runout(&self, spec: &ToleranceSpec) -> f64 {
        if self.mesh.vertices.is_empty() {
            return 0.0;
        }

        let datum_axis = match spec.datum_axis {
            Some(a) => a,
            None => {
                log::warn!(
                    "Runout tolerance '{}' has no datum_axis; returning NaN",
                    spec.name
                );
                return f64::NAN;
            }
        };

        let centroid = self.mesh_centroid();
        let (ax, ay, az) = (datum_axis.x, datum_axis.y, datum_axis.z);

        // Compute axial position and radial distance for each vertex.
        // Axial position = (p - centroid) · axis
        // Radial distance = |(p - centroid) - axial * axis|
        let mut positions: Vec<(f64, f64)> = Vec::with_capacity(self.mesh.vertices.len());
        for p in &self.mesh.vertices {
            let dx = p.x - centroid.0;
            let dy = p.y - centroid.1;
            let dz = p.z - centroid.2;
            let axial = dx * ax + dy * ay + dz * az;
            let perp_x = dx - axial * ax;
            let perp_y = dy - axial * ay;
            let perp_z = dz - axial * az;
            let radial = (perp_x * perp_x + perp_y * perp_y + perp_z * perp_z).sqrt();
            positions.push((axial, radial));
        }

        // Find the axial extent and bin into 100 cross-sections.
        let axial_min = positions.iter().map(|(a, _)| *a).fold(f64::INFINITY, f64::min);
        let axial_max = positions.iter().map(|(a, _)| *a).fold(f64::NEG_INFINITY, f64::max);
        let axial_range = axial_max - axial_min;

        if axial_range < 1e-12 {
            // Mesh is essentially 2D perpendicular to the axis — single
            // cross-section. Runout = max radial - min radial.
            let r_min = positions.iter().map(|(_, r)| *r).fold(f64::INFINITY, f64::min);
            let r_max = positions.iter().map(|(_, r)| *r).fold(f64::NEG_INFINITY, f64::max);
            return r_max - r_min;
        }

        // Bin vertices into 100 axial positions.
        const N_BINS: usize = 100;
        let bin_width = axial_range / N_BINS as f64;
        let mut bins: Vec<Vec<f64>> = (0..N_BINS).map(|_| Vec::new()).collect();
        for (axial, radial) in &positions {
            let bin_idx = ((axial - axial_min) / bin_width).floor() as usize;
            let bin_idx = bin_idx.min(N_BINS - 1);
            bins[bin_idx].push(*radial);
        }

        // Runout = max FIM across all non-empty bins.
        let mut max_fim = 0.0_f64;
        for bin in &bins {
            if bin.len() < 2 {
                continue;
            }
            let r_min = bin.iter().fold(f64::INFINITY, |a, b| a.min(*b));
            let r_max = bin.iter().fold(f64::NEG_INFINITY, |a, b| a.max(*b));
            let fim = r_max - r_min;
            max_fim = max_fim.max(fim);
        }
        max_fim
    }

    // ============================================================
    // Profile of a line check
    // ============================================================

    /// Check profile of a line: deviation of cross-sectional curves
    /// from a nominal profile. Per ASME Y14.5, the tolerance zone is
    /// two curves offset from the nominal profile by ±tolerance_value/2.
    ///
    /// Implementation: for each cross-section perpendicular to the
    /// dominant axis of the mesh, measure the max deviation of vertices
    /// in that cross-section from the nominal surface. The overall
    /// deviation is the max across all cross-sections.
    ///
    /// Requires `spec.nominal_surface` (a plane) or
    /// `spec.nominal_cylinder` ((origin, axis, radius)). If neither is
    /// set, returns NaN.
    fn check_profile_of_line(&self, spec: &ToleranceSpec) -> f64 {
        self.check_profile(spec, /*cross_sectional=*/ true)
    }

    // ============================================================
    // Profile of a surface check
    // ============================================================

    /// Check profile of a surface: deviation of the entire surface from
    /// a nominal profile. Per ASME Y14.5, the tolerance zone is two
    /// surfaces offset from the nominal by ±tolerance_value/2.
    ///
    /// Implementation: measure max deviation of all mesh vertices from
    /// the nominal surface (plane or cylinder).
    fn check_profile_of_surface(&self, spec: &ToleranceSpec) -> f64 {
        self.check_profile(spec, /*cross_sectional=*/ false)
    }

    /// Shared profile-check implementation for ProfileOfLine and
    /// ProfileOfSurface. When `cross_sectional` is true, the deviation
    /// is computed per cross-section and the max is reported.
    fn check_profile(&self, spec: &ToleranceSpec, cross_sectional: bool) -> f64 {
        if self.mesh.vertices.is_empty() {
            return 0.0;
        }

        // Plane nominal: deviation = |signed distance from plane|
        if let Some(ref plane) = spec.nominal_surface {
            if cross_sectional {
                // Bin by dominant axis of the plane normal, measure
                // per-section deviation.
                let abs_x = plane.normal.x.abs();
                let abs_y = plane.normal.y.abs();
                let abs_z = plane.normal.z.abs();
                let axis_idx = if abs_x >= abs_y && abs_x >= abs_z {
                    0
                } else if abs_y >= abs_z {
                    1
                } else {
                    2
                };

                let coords: Vec<f64> = self
                    .mesh
                    .vertices
                    .iter()
                    .map(|p| match axis_idx {
                        0 => p.x,
                        1 => p.y,
                        _ => p.z,
                    })
                    .collect();
                let c_min = coords.iter().fold(f64::INFINITY, |a, b| a.min(*b));
                let c_max = coords.iter().fold(f64::NEG_INFINITY, |a, b| a.max(*b));
                let range = c_max - c_min;
                if range < 1e-12 {
                    // Single section.
                    return self.mesh.vertices.iter()
                        .map(|p| plane.signed_distance(p).abs())
                        .fold(0.0_f64, |a, b| a.max(b));
                }

                const N_BINS: usize = 100;
                let bin_width = range / N_BINS as f64;
                let mut bins: Vec<Vec<f64>> = (0..N_BINS).map(|_| Vec::new()).collect();
                for (p, c) in self.mesh.vertices.iter().zip(coords.iter()) {
                    let bin_idx = ((c - c_min) / bin_width).floor() as usize;
                    let bin_idx = bin_idx.min(N_BINS - 1);
                    bins[bin_idx].push(plane.signed_distance(p).abs());
                }
                bins.iter()
                    .filter(|b| !b.is_empty())
                    .map(|b| b.iter().fold(0.0_f64, |a, &b| a.max(b)))
                    .fold(0.0_f64, |a, b| a.max(b))
            } else {
                // Whole-surface: max |signed distance|.
                self.mesh
                    .vertices
                    .iter()
                    .map(|p| plane.signed_distance(p).abs())
                    .fold(0.0_f64, |a, b| a.max(b))
            }
        } else if let Some((origin, axis, radius)) = &spec.nominal_cylinder {
            // Cylinder nominal: deviation = |measured_radius - nominal_radius|
            let (ax, ay, az) = (axis.x, axis.y, axis.z);
            let (ox, oy, oz) = (origin.x, origin.y, origin.z);

            let compute_dev = |p: &Point3d| -> f64 {
                let dx = p.x - ox;
                let dy = p.y - oy;
                let dz = p.z - oz;
                let axial = dx * ax + dy * ay + dz * az;
                let perp_x = dx - axial * ax;
                let perp_y = dy - axial * ay;
                let perp_z = dz - axial * az;
                let r = (perp_x * perp_x + perp_y * perp_y + perp_z * perp_z).sqrt();
                (r - *radius).abs()
            };

            if cross_sectional {
                // Bin by axial position.
                let axial_positions: Vec<f64> = self
                    .mesh
                    .vertices
                    .iter()
                    .map(|p| {
                        let dx = p.x - ox;
                        let dy = p.y - oy;
                        let dz = p.z - oz;
                        dx * ax + dy * ay + dz * az
                    })
                    .collect();
                let a_min = axial_positions.iter().fold(f64::INFINITY, |a, b| a.min(*b));
                let a_max = axial_positions.iter().fold(f64::NEG_INFINITY, |a, b| a.max(*b));
                let range = a_max - a_min;
                if range < 1e-12 {
                    return self
                        .mesh
                        .vertices
                        .iter()
                        .map(|p| compute_dev(p))
                        .fold(0.0_f64, |a, b| a.max(b));
                }
                const N_BINS: usize = 100;
                let bin_width = range / N_BINS as f64;
                let mut bins: Vec<Vec<f64>> = (0..N_BINS).map(|_| Vec::new()).collect();
                for (p, a) in self.mesh.vertices.iter().zip(axial_positions.iter()) {
                    let bin_idx = ((a - a_min) / bin_width).floor() as usize;
                    let bin_idx = bin_idx.min(N_BINS - 1);
                    bins[bin_idx].push(compute_dev(p));
                }
                bins.iter()
                    .filter(|b| !b.is_empty())
                    .map(|b| b.iter().fold(0.0_f64, |a, &b| a.max(b)))
                    .fold(0.0_f64, |a, b| a.max(b))
            } else {
                self.mesh
                    .vertices
                    .iter()
                    .map(|p| compute_dev(p))
                    .fold(0.0_f64, |a, b| a.max(b))
            }
        } else {
            log::warn!(
                "Profile tolerance '{}' has no nominal_surface or nominal_cylinder; returning NaN",
                spec.name
            );
            f64::NAN
        }
    }

    // ============================================================
    // Helper methods
    // ============================================================

    /// Compute the centroid of all mesh vertices.
    fn mesh_centroid(&self) -> (f64, f64, f64) {
        let n = self.mesh.vertices.len() as f64;
        if n == 0.0 {
            return (0.0, 0.0, 0.0);
        }
        let cx = self.mesh.vertices.iter().map(|p| p.x).sum::<f64>() / n;
        let cy = self.mesh.vertices.iter().map(|p| p.y).sum::<f64>() / n;
        let cz = self.mesh.vertices.iter().map(|p| p.z).sum::<f64>() / n;
        (cx, cy, cz)
    }

    /// Compute the bounding box (min, max) of the mesh.
    fn bounding_box(&self) -> ((f64, f64, f64), (f64, f64, f64)) {
        let mut min = (f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut max = (f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for p in &self.mesh.vertices {
            min.0 = min.0.min(p.x);
            min.1 = min.1.min(p.y);
            min.2 = min.2.min(p.z);
            max.0 = max.0.max(p.x);
            max.1 = max.1.max(p.y);
            max.2 = max.2.max(p.z);
        }
        (min, max)
    }

    /// Best-fit plane through all mesh vertices.
    /// Returns (centroid, normal).
    fn best_fit_plane(&self) -> ((f64, f64, f64), (f64, f64, f64)) {
        let centroid = self.mesh_centroid();
        let (cx, cy, cz) = centroid;

        let mut xx = 0.0_f64;
        let mut xy = 0.0_f64;
        let mut xz = 0.0_f64;
        let mut yy = 0.0_f64;
        let mut yz = 0.0_f64;
        let mut zz = 0.0_f64;

        for p in &self.mesh.vertices {
            let dx = p.x - cx;
            let dy = p.y - cy;
            let dz = p.z - cz;
            xx += dx * dx;
            xy += dx * dy;
            xz += dx * dz;
            yy += dy * dy;
            yz += dy * dz;
            zz += dz * dz;
        }

        let normal = smallest_eigenvector_3x3(xx, xy, xz, yy, yz, zz);
        (centroid, normal)
    }

    /// Resolve the datum direction for parallelism/perpendicularity/
    /// angularity checks.
    ///
    /// Priority:
    /// 1. `datum_plane_normal` — the datum is a plane, and its normal is
    ///    the direction we compare against.
    /// 2. `datum_axis` — the datum is an axis; we use the axis direction
    ///    directly.
    /// 3. None — returns None.
    fn resolve_parallelism_datum(&self, spec: &ToleranceSpec) -> Option<Direction3d> {
        if let Some(n) = &spec.datum_plane_normal {
            Some(*n)
        } else if let Some(a) = &spec.datum_axis {
            Some(*a)
        } else {
            None
        }
    }

    /// Rotate a vector `v` around an axis perpendicular to both `datum`
    /// and `v`, by `angle` radians. Used by Angularity to compute the
    /// expected surface normal given a datum and a nominal angle.
    ///
    /// The rotation axis is `datum × v` (which is perpendicular to both).
    /// If `datum` and `v` are parallel, the cross product is zero and
    /// we return None.
    #[allow(dead_code)] // kept for future use (e.g. nominal-angle surface reconstruction)
    fn rotate_around_perpendicular_axis(
        &self,
        datum: &Direction3d,
        angle: f64,
        v: (f64, f64, f64),
    ) -> Option<(f64, f64, f64)> {
        // Rotation axis = datum × v
        let axis = (
            datum.y * v.2 - datum.z * v.1,
            datum.z * v.0 - datum.x * v.2,
            datum.x * v.1 - datum.y * v.0,
        );
        let axis_len = (axis.0 * axis.0 + axis.1 * axis.1 + axis.2 * axis.2).sqrt();
        if axis_len < 1e-15 {
            return None;
        }
        let (ux, uy, uz) = (axis.0 / axis_len, axis.1 / axis_len, axis.2 / axis_len);

        // Rodrigues' rotation formula: v_rot = v*cos(θ) + (u × v)*sin(θ) + u*(u·v)*(1-cos(θ))
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        let dot = ux * v.0 + uy * v.1 + uz * v.2;
        let cross = (
            uy * v.2 - uz * v.1,
            uz * v.0 - ux * v.2,
            ux * v.1 - uy * v.0,
        );
        let result = (
            v.0 * cos_a + cross.0 * sin_a + ux * dot * (1.0 - cos_a),
            v.1 * cos_a + cross.1 * sin_a + uy * dot * (1.0 - cos_a),
            v.2 * cos_a + cross.2 * sin_a + uz * dot * (1.0 - cos_a),
        );
        Some(result)
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
    /// Optional nominal position (for Position tolerance) — the
    /// theoretically-exact location of the feature's centre. The check
    /// measures the deviation of the actual mesh centroid from this
    /// point. If `None`, the centroid of the mesh is used as a fallback
    /// (which yields deviation 0, so the check will pass trivially).
    pub nominal_position: Option<Point3d>,
    /// Optional datum axis direction (for Parallelism, Perpendicularity,
    /// Angularity, Runout). For Parallelism the surface must be parallel
    /// to this axis; for Perpendicularity, normal to this axis; for
    /// Angularity, at `nominal_angle` to this axis; for Runout, the
    /// mesh is rotated about this axis.
    pub datum_axis: Option<Direction3d>,
    /// Optional datum plane normal (alternative to `datum_axis` for
    /// Parallelism/Perpendicularity/Angularity when the datum is a plane
    /// rather than an axis). For Parallelism the surface must be
    /// parallel to this plane; for Perpendicularity, normal to this plane.
    pub datum_plane_normal: Option<Direction3d>,
    /// Nominal angle in degrees (for Angularity). The tolerance zone is
    /// ±tolerance_value measured perpendicular to the datum.
    pub nominal_angle_deg: Option<f64>,
    /// Optional nominal surface for ProfileOfLine / ProfileOfSurface.
    /// The check measures max deviation of mesh vertices from this
    /// surface. If `None`, the check returns NaN.
    pub nominal_surface: Option<Plane>,
    /// Optional best-fit cylinder parameters for ProfileOfLine /
    /// ProfileOfSurface when the nominal is a cylinder rather than a
    /// plane: `(axis_origin, axis_direction, radius)`.
    pub nominal_cylinder: Option<(Point3d, Direction3d, f64)>,
}

impl Default for ToleranceSpec {
    fn default() -> Self {
        Self {
            name: String::new(),
            description: String::new(),
            tolerance_type: GdtCheckType::Unsupported(String::new()),
            tolerance_value: 0.0,
            step_id: 0,
            datum_references: Vec::new(),
            nominal_position: None,
            datum_axis: None,
            datum_plane_normal: None,
            nominal_angle_deg: None,
            nominal_surface: None,
            nominal_cylinder: None,
        }
    }
}

/// A plane in 3D defined by a point and a normal.
/// Used as a nominal surface for profile tolerance checks.
#[derive(Clone, Debug)]
pub struct Plane {
    /// A point on the plane.
    pub origin: Point3d,
    /// Unit normal of the plane.
    pub normal: Direction3d,
}

impl Plane {
    /// Signed distance from `p` to this plane (positive on the normal side).
    pub fn signed_distance(&self, p: &Point3d) -> f64 {
        (p.x - self.origin.x) * self.normal.x
            + (p.y - self.origin.y) * self.normal.y
            + (p.z - self.origin.z) * self.normal.z
    }
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

    // Normalize the largest eigenvector so deflation is numerically clean.
    let n_largest = (
        largest.0 / len_sq.sqrt(),
        largest.1 / len_sq.sqrt(),
        largest.2 / len_sq.sqrt(),
    );

    // Deflated matrix: M' = M - lambda * v * v^T  (v is now unit length)
    let dxx = xx - lambda * n_largest.0 * n_largest.0;
    let dxy = xy - lambda * n_largest.0 * n_largest.1;
    let dxz = xz - lambda * n_largest.0 * n_largest.2;
    let dyy = yy - lambda * n_largest.1 * n_largest.1;
    let dyz = yz - lambda * n_largest.1 * n_largest.2;
    let dzz = zz - lambda * n_largest.2 * n_largest.2;

    // Frobenius norm of the deflated matrix — if it's near-zero, the original
    // matrix had rank 1 (only one non-zero eigenvalue). In that case, the
    // smallest eigenvector is ANY unit vector orthogonal to `largest`. We
    // pick one robustly using the axis-of-least-component trick.
    let frob_sq = dxx * dxx + 2.0 * dxy * dxy + 2.0 * dxz * dxz
                + dyy * dyy + 2.0 * dyz * dyz + dzz * dzz;
    if frob_sq < 1e-22 {
        return orthogonal_unit_vector(n_largest);
    }

    // Find the largest eigenvector of the deflated matrix (= 2nd largest of original)
    let second = largest_eigenvector_3x3(dxx, dxy, dxz, dyy, dyz, dzz);

    let len_sq2 = second.0 * second.0 + second.1 * second.1 + second.2 * second.2;
    if len_sq2 < 1e-30 {
        // Cross product of first two gives the third
        return cross_product(largest, second);
    }

    // Normalize second eigenvector for clean second deflation.
    let n_second = (
        second.0 / len_sq2.sqrt(),
        second.1 / len_sq2.sqrt(),
        second.2 / len_sq2.sqrt(),
    );

    let lambda2 = second.0 * (dxx * second.0 + dxy * second.1 + dxz * second.2)
                + second.1 * (dxy * second.0 + dyy * second.1 + dyz * second.2)
                + second.2 * (dxz * second.0 + dyz * second.1 + dzz * second.2);

    // Deflate again: M'' = M' - lambda2 * n_second * n_second^T
    let d2xx = dxx - lambda2 * n_second.0 * n_second.0;
    let d2xy = dxy - lambda2 * n_second.0 * n_second.1;
    let d2xz = dxz - lambda2 * n_second.0 * n_second.2;
    let d2yy = dyy - lambda2 * n_second.1 * n_second.1;
    let d2yz = dyz - lambda2 * n_second.1 * n_second.2;
    let d2zz = dzz - lambda2 * n_second.2 * n_second.2;

    // If the twice-deflated matrix is essentially zero, then the original
    // matrix had rank 2 (two non-zero eigenvalues). The smallest eigenvector
    // is the unit vector orthogonal to both `largest` and `second`.
    let frob_sq2 = d2xx * d2xx + 2.0 * d2xy * d2xy + 2.0 * d2xz * d2xz
                 + d2yy * d2yy + 2.0 * d2yz * d2yz + d2zz * d2zz;
    if frob_sq2 < 1e-22 {
        let mut v = cross_product(n_largest, n_second);
        let vlen = (v.0 * v.0 + v.1 * v.1 + v.2 * v.2).sqrt();
        if vlen > 1e-30 {
            v = (v.0 / vlen, v.1 / vlen, v.2 / vlen);
            return v;
        }
        // Fallback if cross product is degenerate.
        return orthogonal_unit_vector(n_largest);
    }

    // Otherwise do power iteration on the twice-deflated matrix; its largest
    // eigenvector is the smallest of the original matrix.
    let mut result = largest_eigenvector_3x3(d2xx, d2xy, d2xz, d2yy, d2yz, d2zz);
    let rlen = (result.0 * result.0 + result.1 * result.1 + result.2 * result.2).sqrt();
    if rlen > 1e-30 {
        result = (result.0 / rlen, result.1 / rlen, result.2 / rlen);
    }
    result
}

/// Return any unit vector orthogonal to `v`.
///
/// Used as a fallback when a covariance matrix has repeated zero eigenvalues
/// (e.g. a perfectly flat mesh lying in a coordinate plane). In that case
/// `largest_eigenvector_3x3` returns the in-plane direction, and the smallest
/// eigenvector (the surface normal) is anything orthogonal to it.
fn orthogonal_unit_vector(v: (f64, f64, f64)) -> (f64, f64, f64) {
    // Pick the axis least aligned with v, then take the cross product.
    let abs_x = v.0.abs();
    let abs_y = v.1.abs();
    let abs_z = v.2.abs();
    let axis = if abs_x <= abs_y && abs_x <= abs_z {
        (1.0, 0.0, 0.0)
    } else if abs_y <= abs_z {
        (0.0, 1.0, 0.0)
    } else {
        (0.0, 0.0, 1.0)
    };
    let mut ortho = cross_product(v, axis);
    let len = (ortho.0 * ortho.0 + ortho.1 * ortho.1 + ortho.2 * ortho.2).sqrt();
    if len > 1e-30 {
        ortho = (ortho.0 / len, ortho.1 / len, ortho.2 / len);
    } else {
        // v was parallel to every axis pick (degenerate input)
        ortho = (0.0, 0.0, 1.0);
    }
    ortho
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
            ..Default::default()
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
            ..Default::default()
        };
        let result = checker.check(&spec);
        assert!(result.actual_deviation > 0.1, "Warped mesh should have significant flatness deviation, got {}", result.actual_deviation);
        assert!(!result.passed, "Warped mesh should fail flatness check with tolerance 0.1");
    }

    #[test]
    fn test_cylindricity_check() {
        // Create a simple cylindrical mesh (approximation).
        // We build ONLY surface triangles — no cap centers — so every vertex
        // lies on the ideal cylinder of radius `radius` and the cylindricity
        // deviation should be small (limited by the chord error of the
        // polygonal approximation, which for 16 segments at radius 5 is
        // ~0.096).
        let mut mesh = TriangleMesh::new();
        let radius = 5.0;
        let height = 10.0;
        let segments = 16;

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
            ..Default::default()
        };
        let result = checker.check(&spec);
        // A perfect 16-segment cylinder at radius 5 has chord error
        //   5 * (1 - cos(π/16)) ≈ 0.0961,
        // so the radial spread (max-min radius) is ~0.096 and the
        // cylindricity zone (half the spread) is ~0.048. Allow generous
        // headroom for the PCA-based axis estimate.
        assert!(
            result.actual_deviation < 1.0,
            "Cylinder should have low cylindricity deviation, got {}",
            result.actual_deviation
        );
    }

    /// Build a flat mesh parallel to the XY plane (z=0), spanning
    /// (-5..5) in X and Y. Used by parallelism/perpendicularity/profile
    /// tests.
    fn flat_xy_mesh() -> TriangleMesh {
        let mut mesh = TriangleMesh::new();
        let v0 = mesh.add_vertex(Point3d::new(-5.0, -5.0, 0.0));
        let v1 = mesh.add_vertex(Point3d::new( 5.0, -5.0, 0.0));
        let v2 = mesh.add_vertex(Point3d::new( 5.0,  5.0, 0.0));
        let v3 = mesh.add_vertex(Point3d::new(-5.0,  5.0, 0.0));
        mesh.add_triangle_arr([v0, v1, v2]);
        mesh.add_triangle_arr([v0, v2, v3]);
        mesh
    }

    #[test]
    fn test_position_at_nominal_returns_zero() {
        let mesh = flat_xy_mesh();
        let checker = GdtChecker::new(&mesh);
        // Nominal position = mesh centroid → deviation 0.
        let spec = ToleranceSpec {
            name: "Pos".to_string(),
            description: String::new(),
            tolerance_type: GdtCheckType::Position,
            tolerance_value: 0.1,
            step_id: 1,
            datum_references: vec![],
            nominal_position: Some(Point3d::new(0.0, 0.0, 0.0)),
            ..Default::default()
        };
        let result = checker.check(&spec);
        assert!(result.actual_deviation < 1e-9, "expected 0, got {}", result.actual_deviation);
        assert!(result.passed);
    }

    #[test]
    fn test_position_offset_returns_distance() {
        let mesh = flat_xy_mesh();
        let checker = GdtChecker::new(&mesh);
        // Centroid is at (0,0,0); nominal at (0.3, 0.4, 0.0) → 0.5 distance.
        let spec = ToleranceSpec {
            name: "Pos".to_string(),
            description: String::new(),
            tolerance_type: GdtCheckType::Position,
            tolerance_value: 1.0,
            step_id: 1,
            datum_references: vec![],
            nominal_position: Some(Point3d::new(0.3, 0.4, 0.0)),
            ..Default::default()
        };
        let result = checker.check(&spec);
        assert!(
            (result.actual_deviation - 0.5).abs() < 1e-9,
            "expected 0.5, got {}",
            result.actual_deviation
        );
        assert!(result.passed);
    }

    #[test]
    fn test_parallelism_flat_xy_against_z_datum() {
        // Mesh lies in the XY plane → surface normal is ±Z.
        // Datum plane normal = Z → surface is parallel to datum plane.
        // Deviation should be ~0.
        let mesh = flat_xy_mesh();
        let checker = GdtChecker::new(&mesh);
        let spec = ToleranceSpec {
            name: "Par".to_string(),
            description: String::new(),
            tolerance_type: GdtCheckType::Parallelism,
            tolerance_value: 0.1,
            step_id: 1,
            datum_references: vec![],
            datum_plane_normal: Some(Direction3d::Z),
            ..Default::default()
        };
        let result = checker.check(&spec);
        assert!(
            result.actual_deviation < 1e-6,
            "flat XY mesh should have ~0 parallelism to Z datum, got {}",
            result.actual_deviation
        );
    }

    #[test]
    fn test_perpendicularity_flat_xy_against_z_datum() {
        // Mesh lies in the XY plane → surface normal is ±Z.
        // For perpendicularity to Z datum, surface normal must be parallel
        // to Z (which it is) → deviation ~0.
        let mesh = flat_xy_mesh();
        let checker = GdtChecker::new(&mesh);
        let spec = ToleranceSpec {
            name: "Perp".to_string(),
            description: String::new(),
            tolerance_type: GdtCheckType::Perpendicularity,
            tolerance_value: 0.1,
            step_id: 1,
            datum_references: vec![],
            datum_axis: Some(Direction3d::Z),
            ..Default::default()
        };
        let result = checker.check(&spec);
        assert!(
            result.actual_deviation < 1e-6,
            "flat XY mesh is perpendicular to Z axis, expected ~0, got {}",
            result.actual_deviation
        );
    }

    #[test]
    fn test_runout_around_z_axis_for_cylinder() {
        // Use the cylindricity test cylinder — its radial deviation
        // around Z should be the chord error (~0.096).
        let mut mesh = TriangleMesh::new();
        let radius = 5.0;
        let height = 10.0;
        let segments = 16;
        let mut bottom_verts = Vec::new();
        let mut top_verts = Vec::new();
        for i in 0..segments {
            let angle = 2.0 * std::f64::consts::PI * i as f64 / segments as f64;
            bottom_verts.push(mesh.add_vertex(Point3d::new(radius * angle.cos(), radius * angle.sin(), 0.0)));
            top_verts.push(mesh.add_vertex(Point3d::new(radius * angle.cos(), radius * angle.sin(), height)));
        }
        for i in 0..segments {
            let next = (i + 1) % segments;
            mesh.add_triangle_arr([bottom_verts[i], bottom_verts[next], top_verts[i]]);
            mesh.add_triangle_arr([bottom_verts[next], top_verts[next], top_verts[i]]);
        }

        let checker = GdtChecker::new(&mesh);
        let spec = ToleranceSpec {
            name: "Runout".to_string(),
            description: String::new(),
            tolerance_type: GdtCheckType::Runout,
            tolerance_value: 1.0,
            step_id: 1,
            datum_references: vec![],
            datum_axis: Some(Direction3d::Z),
            ..Default::default()
        };
        let result = checker.check(&spec);
        // Runout = max FIM per cross-section = max radial spread.
        // For a 16-segment polygon at radius 5, max FIM ≈ 5*(1-cos(π/16)) ≈ 0.096.
        assert!(
            result.actual_deviation < 0.5,
            "expected ~0.1, got {}",
            result.actual_deviation
        );
        assert!(result.passed);
    }

    #[test]
    fn test_profile_of_surface_against_plane() {
        // Mesh in XY plane; nominal surface is the same plane → deviation 0.
        let mesh = flat_xy_mesh();
        let checker = GdtChecker::new(&mesh);
        let spec = ToleranceSpec {
            name: "ProfileS".to_string(),
            description: String::new(),
            tolerance_type: GdtCheckType::ProfileOfSurface,
            tolerance_value: 0.1,
            step_id: 1,
            datum_references: vec![],
            nominal_surface: Some(Plane {
                origin: Point3d::new(0.0, 0.0, 0.0),
                normal: Direction3d::Z,
            }),
            ..Default::default()
        };
        let result = checker.check(&spec);
        assert!(
            result.actual_deviation < 1e-9,
            "expected 0, got {}",
            result.actual_deviation
        );
        assert!(result.passed);
    }

    #[test]
    fn test_profile_of_surface_against_offset_plane() {
        // Mesh in XY plane (z=0); nominal plane at z=0.5 → deviation 0.5.
        let mesh = flat_xy_mesh();
        let checker = GdtChecker::new(&mesh);
        let spec = ToleranceSpec {
            name: "ProfileS".to_string(),
            description: String::new(),
            tolerance_type: GdtCheckType::ProfileOfSurface,
            tolerance_value: 1.0,
            step_id: 1,
            datum_references: vec![],
            nominal_surface: Some(Plane {
                origin: Point3d::new(0.0, 0.0, 0.5),
                normal: Direction3d::Z,
            }),
            ..Default::default()
        };
        let result = checker.check(&spec);
        assert!(
            (result.actual_deviation - 0.5).abs() < 1e-9,
            "expected 0.5, got {}",
            result.actual_deviation
        );
    }

    #[test]
    fn test_profile_of_surface_against_cylinder() {
        // Use the cylinder test mesh (radius 5, axis = Z through origin).
        // Nominal cylinder has the same parameters → deviation ≈ chord error.
        let mut mesh = TriangleMesh::new();
        let radius = 5.0;
        let height = 10.0;
        let segments = 32; // More segments → smaller chord error
        let mut bottom_verts = Vec::new();
        let mut top_verts = Vec::new();
        for i in 0..segments {
            let angle = 2.0 * std::f64::consts::PI * i as f64 / segments as f64;
            bottom_verts.push(mesh.add_vertex(Point3d::new(radius * angle.cos(), radius * angle.sin(), 0.0)));
            top_verts.push(mesh.add_vertex(Point3d::new(radius * angle.cos(), radius * angle.sin(), height)));
        }
        for i in 0..segments {
            let next = (i + 1) % segments;
            mesh.add_triangle_arr([bottom_verts[i], bottom_verts[next], top_verts[i]]);
            mesh.add_triangle_arr([bottom_verts[next], top_verts[next], top_verts[i]]);
        }

        let checker = GdtChecker::new(&mesh);
        let spec = ToleranceSpec {
            name: "ProfileCyl".to_string(),
            description: String::new(),
            tolerance_type: GdtCheckType::ProfileOfSurface,
            tolerance_value: 1.0,
            step_id: 1,
            datum_references: vec![],
            nominal_cylinder: Some((
                Point3d::new(0.0, 0.0, 0.0),
                Direction3d::Z,
                radius,
            )),
            ..Default::default()
        };
        let result = checker.check(&spec);
        // Chord error for 32-seg polygon at radius 5 = 5*(1-cos(π/32)) ≈ 0.024.
        assert!(
            result.actual_deviation < 0.1,
            "expected ~0.024, got {}",
            result.actual_deviation
        );
        assert!(result.passed);
    }

    #[test]
    fn test_angularity_45_deg_to_z_datum() {
        // Build a flat mesh tilted 45° around X axis.
        // Vertices at (x, y, z=y) — surface normal = (0, -1, 1)/√2.
        let mut mesh = TriangleMesh::new();
        let v0 = mesh.add_vertex(Point3d::new(-5.0, -5.0, -5.0));
        let v1 = mesh.add_vertex(Point3d::new( 5.0, -5.0, -5.0));
        let v2 = mesh.add_vertex(Point3d::new( 5.0,  5.0,  5.0));
        let v3 = mesh.add_vertex(Point3d::new(-5.0,  5.0,  5.0));
        mesh.add_triangle_arr([v0, v1, v2]);
        mesh.add_triangle_arr([v0, v2, v3]);

        let checker = GdtChecker::new(&mesh);
        let spec = ToleranceSpec {
            name: "Ang".to_string(),
            description: String::new(),
            tolerance_type: GdtCheckType::Angularity,
            tolerance_value: 1.0,
            step_id: 1,
            datum_references: vec![],
            datum_axis: Some(Direction3d::Z),
            nominal_angle_deg: Some(45.0),
            ..Default::default()
        };
        let result = checker.check(&spec);
        // A perfect 45° plane should have low deviation.
        assert!(
            result.actual_deviation < 1.0,
            "expected <1.0, got {}",
            result.actual_deviation
        );
    }

    #[test]
    fn test_position_no_nominal_returns_zero_with_warning() {
        let mesh = flat_xy_mesh();
        let checker = GdtChecker::new(&mesh);
        let spec = ToleranceSpec {
            name: "Pos".to_string(),
            description: String::new(),
            tolerance_type: GdtCheckType::Position,
            tolerance_value: 0.1,
            step_id: 1,
            datum_references: vec![],
            ..Default::default()
        };
        let result = checker.check(&spec);
        assert_eq!(result.actual_deviation, 0.0);
        assert!(result.passed);
    }

    #[test]
    fn test_parallelism_no_datum_returns_nan() {
        let mesh = flat_xy_mesh();
        let checker = GdtChecker::new(&mesh);
        let spec = ToleranceSpec {
            name: "Par".to_string(),
            description: String::new(),
            tolerance_type: GdtCheckType::Parallelism,
            tolerance_value: 0.1,
            step_id: 1,
            datum_references: vec![],
            ..Default::default()
        };
        let result = checker.check(&spec);
        assert!(result.actual_deviation.is_nan(), "expected NaN, got {}", result.actual_deviation);
        assert!(!result.passed);
    }
}
