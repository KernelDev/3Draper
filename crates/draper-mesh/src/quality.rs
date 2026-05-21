//! Triangle quality metrics and mesh validation.
//!
//! Provides quality metrics for triangle meshes:
//! - Aspect ratio (max edge / min height)
//! - Minimum angle
//! - Area distortion (3D vs 2D)
//! - Jacobian sign check
//!
//! Used by the iterative refinement loop to identify and fix bad triangles.

use draper_geometry::point::Point3;

/// Quality metrics for a single triangle.
#[derive(Debug, Clone, Copy)]
pub struct TriangleQuality {
    /// Aspect ratio: max_edge_length / min_height.
    /// Perfect equilateral = ~1.15. Values > 15 are considered poor.
    pub aspect_ratio: f64,
    /// Minimum interior angle in degrees.
    /// Should be > 10 degrees for acceptable quality.
    pub min_angle_deg: f64,
    /// Maximum interior angle in degrees.
    pub max_angle_deg: f64,
    /// Area of the triangle in 3D.
    pub area: f64,
    /// Whether the triangle is degenerate (zero area).
    pub is_degenerate: bool,
}

impl TriangleQuality {
    /// Check if this triangle meets minimum quality standards.
    pub fn is_acceptable(&self) -> bool {
        !self.is_degenerate && self.aspect_ratio < 20.0 && self.min_angle_deg > 5.0
    }
}

/// Compute quality metrics for a triangle defined by three 3D points.
pub fn triangle_quality(a: Point3, b: Point3, c: Point3) -> TriangleQuality {
    let ab = b - a;
    let ac = c - a;
    let bc = c - b;

    let ab_len = ab.length();
    let ac_len = ac.length();
    let bc_len = bc.length();

    // Area via cross product
    let cross = ab.cross(ac);
    let double_area = cross.length();
    let area = double_area / 2.0;

    if area < 1e-20 || ab_len < 1e-20 || ac_len < 1e-20 || bc_len < 1e-20 {
        return TriangleQuality {
            aspect_ratio: f64::MAX,
            min_angle_deg: 0.0,
            max_angle_deg: 180.0,
            area: 0.0,
            is_degenerate: true,
        };
    }

    // Minimum height = 2 * area / longest_edge
    let max_edge = ab_len.max(ac_len).max(bc_len);
    let min_height = double_area / max_edge;

    let aspect_ratio = max_edge / min_height;

    // Angles using law of cosines
    let angle_a = compute_angle(ab_len, ac_len, bc_len);
    let angle_b = compute_angle(ab_len, bc_len, ac_len);
    let angle_c = compute_angle(ac_len, bc_len, ab_len);

    let min_angle_deg = angle_a.min(angle_b).min(angle_c).to_degrees();
    let max_angle_deg = angle_a.max(angle_b).max(angle_c).to_degrees();

    TriangleQuality {
        aspect_ratio,
        min_angle_deg,
        max_angle_deg,
        area,
        is_degenerate: false,
    }
}

/// Compute angle opposite to side c in a triangle with sides a, b, c.
/// Uses law of cosines: c² = a² + b² - 2ab·cos(C)
fn compute_angle(a_len: f64, b_len: f64, c_len: f64) -> f64 {
    if a_len < 1e-20 || b_len < 1e-20 {
        return 0.0;
    }
    let cos_c = (a_len * a_len + b_len * b_len - c_len * c_len) / (2.0 * a_len * b_len);
    cos_c.clamp(-1.0, 1.0).acos()
}

/// Compute the Jacobian sign at a triangle vertex.
///
/// The Jacobian should have a consistent sign across all triangles
/// in a face. A sign change indicates a mesh inversion.
pub fn jacobian_sign(a: Point3, b: Point3, c: Point3) -> f64 {
    let ab = b - a;
    let ac = c - a;
    let normal = ab.cross(ac);
    // The sign depends on the reference direction
    // We just return the Z component of the cross product
    // (assuming the projection was onto XY plane)
    normal.z
}

/// Compute quality statistics for an entire mesh.
#[derive(Debug, Clone)]
pub struct MeshQualityStats {
    /// Number of triangles.
    pub triangle_count: usize,
    /// Number of degenerate triangles.
    pub degenerate_count: usize,
    /// Minimum aspect ratio across all triangles.
    pub min_aspect_ratio: f64,
    /// Maximum aspect ratio across all triangles.
    pub max_aspect_ratio: f64,
    /// Average aspect ratio.
    pub avg_aspect_ratio: f64,
    /// Minimum angle across all triangles (degrees).
    pub min_angle: f64,
    /// Maximum angle across all triangles (degrees).
    pub max_angle: f64,
    /// Number of triangles with poor quality.
    pub poor_quality_count: usize,
}

impl MeshQualityStats {
    /// Compute quality statistics from a mesh.
    pub fn from_mesh(vertices: &[Point3], indices: &[u32]) -> Self {
        let mut stats = MeshQualityStats {
            triangle_count: 0,
            degenerate_count: 0,
            min_aspect_ratio: f64::MAX,
            max_aspect_ratio: 0.0,
            avg_aspect_ratio: 0.0,
            min_angle: 180.0,
            max_angle: 0.0,
            poor_quality_count: 0,
        };

        let mut aspect_sum = 0.0;

        for tri in indices.chunks(3) {
            if tri.len() < 3 {
                continue;
            }
            let a = vertices[tri[0] as usize];
            let b = vertices[tri[1] as usize];
            let c = vertices[tri[2] as usize];

            let q = triangle_quality(a, b, c);
            stats.triangle_count += 1;

            if q.is_degenerate {
                stats.degenerate_count += 1;
            } else {
                stats.min_aspect_ratio = stats.min_aspect_ratio.min(q.aspect_ratio);
                stats.max_aspect_ratio = stats.max_aspect_ratio.max(q.aspect_ratio);
                stats.min_angle = stats.min_angle.min(q.min_angle_deg);
                stats.max_angle = stats.max_angle.max(q.max_angle_deg);
                aspect_sum += q.aspect_ratio;

                if q.aspect_ratio > 15.0 || q.min_angle_deg < 10.0 {
                    stats.poor_quality_count += 1;
                }
            }
        }

        if stats.triangle_count > stats.degenerate_count {
            stats.avg_aspect_ratio = aspect_sum / (stats.triangle_count - stats.degenerate_count) as f64;
        }

        stats
    }
}
