//! Edge discretization cache for consistent triangulation.
//!
//! Ensures that shared edges between faces produce identical 3D point sequences.
//! This is critical for watertight (gap-free) meshes where adjacent faces must
//! have exactly the same vertices on their common edges.
//!
//! # How it works
//!
//! Without this cache, each face independently samples its boundary edges,
//! resulting in *different* 3D points on shared edges. This creates gaps
//! (cracks) between adjacent faces in the final mesh.
//!
//! With this cache:
//! 1. The first face that references an edge triggers its discretization.
//! 2. The resulting 3D points and curve parameters are cached by edge `TopoId`.
//! 3. Subsequent faces that share the same edge receive the *identical* 3D
//!    point sequence, plus UV coordinates computed for their own surface.
//!
//! # Future work (Phase 2.2)
//!
//! The cached UV coordinates per face will be used directly by UV-space CDT
//! triangulation to produce boundary-conforming triangles.

use draper_geometry::{Point3d, Point2d, Curve3d, Curve2d, Surface, tolerance::ToleranceContext};
use draper_topology::{Edge, TopoId};
use std::collections::HashMap;

/// Cached discretization of a single edge.
#[derive(Clone, Debug)]
pub struct EdgeDiscretization {
    /// 3D points along the edge curve.
    pub points_3d: Vec<Point3d>,
    /// UV coordinates for each incident face.
    /// Maps face TopoId → Vec<Point2d> (same length as points_3d).
    pub uv_per_face: HashMap<TopoId, Vec<Point2d>>,
    /// Curve parameters for each sample point (normalized to [0, 1]).
    pub params: Vec<f64>,
}

/// Cache that ensures each edge is discretized exactly once.
/// When multiple faces share the same edge, they receive identical
/// 3D point sequences and computed UV coordinates.
#[derive(Clone, Debug)]
pub struct EdgeDiscretizationCache {
    /// Maps edge TopoId → its discretization.
    entries: HashMap<TopoId, EdgeDiscretization>,
    /// Tolerance context for adaptive sampling.
    tol_ctx: ToleranceContext,
    /// Maximum number of sample points per edge.
    max_samples: usize,
}

impl EdgeDiscretizationCache {
    /// Create a new cache with default tolerance.
    pub fn new() -> Self {
        Self {
            entries: HashMap::new(),
            tol_ctx: ToleranceContext::new(),
            max_samples: 256,
        }
    }

    /// Create a new cache with custom tolerance and max samples.
    pub fn with_tolerance(tol_ctx: ToleranceContext, max_samples: usize) -> Self {
        Self {
            entries: HashMap::new(),
            tol_ctx,
            max_samples: max_samples.max(4),
        }
    }

    /// Get or compute the discretization for an edge.
    ///
    /// If the edge has already been discretized, returns the cached result
    /// and computes UV coordinates for the given face/surface if not already present.
    ///
    /// # Arguments
    /// * `edge` - The edge to discretize
    /// * `face_id` - The TopoId of the face that needs this edge
    /// * `surface` - The surface of the face (for UV computation)
    /// * `n_samples_hint` - Suggested number of samples (ignored if edge is already cached)
    /// * `curve_2d` - Optional analytical PCURVE in UV space. When present, UV coordinates
    ///   are computed analytically from the curve instead of using surface.project_point().
    pub fn discretize_edge(
        &mut self,
        edge: &Edge,
        face_id: TopoId,
        surface: &Surface,
        n_samples_hint: usize,
        curve_2d: Option<&Curve2d>,
    ) -> &EdgeDiscretization {
        let edge_id = edge.id;

        // If edge already in cache, just add UV if needed
        if self.entries.contains_key(&edge_id) {
            let entry = self.entries.get_mut(&edge_id).unwrap();
            if !entry.uv_per_face.contains_key(&face_id) {
                let uvs = Self::compute_uvs(&entry.points_3d, &entry.params, surface, curve_2d);
                entry.uv_per_face.insert(face_id, uvs);
            }
            return self.entries.get(&edge_id).unwrap();
        }

        // Discretize the edge adaptively
        let (points_3d, params) = self.adaptive_discretize(edge, n_samples_hint);

        // Compute UV for this face
        let uvs = Self::compute_uvs(&points_3d, &params, surface, curve_2d);

        let mut uv_per_face = HashMap::new();
        uv_per_face.insert(face_id, uvs);

        let disc = EdgeDiscretization {
            points_3d,
            uv_per_face,
            params,
        };

        self.entries.insert(edge_id, disc);
        self.entries.get(&edge_id).unwrap()
    }

    /// Get the cached discretization for an edge (if it exists).
    pub fn get(&self, edge_id: TopoId) -> Option<&EdgeDiscretization> {
        self.entries.get(&edge_id)
    }

    /// Get the cached discretization for an edge mutably (if it exists).
    pub fn get_mut(&mut self, edge_id: TopoId) -> Option<&mut EdgeDiscretization> {
        self.entries.get_mut(&edge_id)
    }

    /// Check if an edge is already in the cache.
    pub fn contains(&self, edge_id: TopoId) -> bool {
        self.entries.contains_key(&edge_id)
    }

    /// Number of cached edges.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether the cache is empty.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Adaptively discretize an edge based on curve curvature.
    ///
    /// Starts with uniformly-spaced points based on the hint, then recursively
    /// subdivides where the chord deviation exceeds `max_deviation`.
    fn adaptive_discretize(&self, edge: &Edge, n_samples_hint: usize) -> (Vec<Point3d>, Vec<f64>) {
        let curve = match &edge.curve {
            Some(c) => c,
            None => {
                // No curve geometry — just use start/end points
                let (p0, p1) = match (edge.start_point(), edge.end_point()) {
                    (Some(a), Some(b)) => (a, b),
                    _ => return (vec![], vec![]),
                };
                return (vec![p0, p1], vec![0.0, 1.0]);
            }
        };

        let (t_min, t_max) = edge.param_range;

        // For line edges, just return endpoints
        if matches!(curve, Curve3d::Line(_)) {
            return (
                vec![curve.point_at(t_min), curve.point_at(t_max)],
                vec![0.0, 1.0],
            );
        }

        // Adaptive subdivision threshold: 10× absolute tolerance as chord deviation
        let max_deviation = self.tol_ctx.absolute * 10.0;

        // Start with uniformly spaced points based on hint
        let n_initial = n_samples_hint.min(self.max_samples).max(2);
        let mut t_params: Vec<f64> = vec![0.0]; // Normalized parameter [0, 1]
        let mut points: Vec<Point3d> = vec![curve.point_at(t_min)];

        for i in 1..n_initial {
            let t_norm = i as f64 / (n_initial - 1) as f64;
            let t = t_min + t_norm * (t_max - t_min);
            points.push(curve.point_at(t));
            t_params.push(t_norm);
        }

        // Refine: check chord deviation and subdivide where needed
        let mut refined = true;
        let mut refinement_passes = 0;
        let max_refinement_passes = 5;

        while refined && refinement_passes < max_refinement_passes && points.len() < self.max_samples {
            refined = false;
            refinement_passes += 1;

            let mut i = 0;
            while i < points.len() - 1 && points.len() < self.max_samples {
                let p0 = points[i];
                let p2 = points[i + 1];

                // Compute midpoint parameter
                let t_mid = (t_params[i] + t_params[i + 1]) * 0.5;
                let t_actual = t_min + t_mid * (t_max - t_min);
                let p_mid = curve.point_at(t_actual);

                // Check chord deviation: distance from midpoint to the chord
                let deviation = point_to_chord_distance(&p_mid, &p0, &p2);

                if deviation > max_deviation {
                    // Subdivide: insert midpoint
                    points.insert(i + 1, p_mid);
                    t_params.insert(i + 1, t_mid);
                    refined = true;
                    i += 2; // Skip the newly inserted point
                } else {
                    i += 1;
                }
            }
        }

        (points, t_params)
    }

    /// Compute UV coordinates for a set of 3D points on a surface.
    ///
    /// If a Curve2d (analytical PCURVE) is provided, UV coordinates are computed
    /// by evaluating the curve at the corresponding parameter values. This is more
    /// accurate and faster than surface.project_point().
    ///
    /// If no Curve2d is available, falls back to surface.project_point().
    fn compute_uvs(points_3d: &[Point3d], params: &[f64], surface: &Surface, curve_2d: Option<&Curve2d>) -> Vec<Point2d> {
        if let Some(c2d) = curve_2d {
            // Use analytical PCURVE — evaluate the 2D curve at each parameter value
            let (t_min, t_max) = c2d.param_range();
            params.iter().map(|&t| {
                // Map normalized parameter t ∈ [0, 1] to curve's parameter range
                let curve_t = t_min + t * (t_max - t_min);
                c2d.point_at(curve_t)
            }).collect()
        } else {
            // Fallback: project 3D points onto the surface
            points_3d
                .iter()
                .map(|p| {
                    let (u, v) = surface.project_point(p);
                    Point2d::new(u, v)
                })
                .collect()
        }
    }

    /// Clear the cache.
    pub fn clear(&mut self) {
        self.entries.clear();
    }
}

impl Default for EdgeDiscretizationCache {
    fn default() -> Self {
        Self::new()
    }
}

/// Compute the distance from a point to a line segment (chord).
fn point_to_chord_distance(point: &Point3d, a: &Point3d, b: &Point3d) -> f64 {
    let abx = b.x - a.x;
    let aby = b.y - a.y;
    let abz = b.z - a.z;
    let apx = point.x - a.x;
    let apy = point.y - a.y;
    let apz = point.z - a.z;

    let ab_len_sq = abx * abx + aby * aby + abz * abz;
    if ab_len_sq < 1e-30 {
        return (apx * apx + apy * apy + apz * apz).sqrt();
    }

    let t = (apx * abx + apy * aby + apz * abz) / ab_len_sq;
    let t = t.clamp(0.0, 1.0);

    let cx = a.x + t * abx - point.x;
    let cy = a.y + t * aby - point.y;
    let cz = a.z + t * abz - point.z;
    (cx * cx + cy * cy + cz * cz).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use draper_geometry::{Point3d, Plane, Direction3d, Line2d};
    use draper_topology::Edge;

    #[test]
    fn test_line_edge_cached_once() {
        let mut cache = EdgeDiscretizationCache::new();
        let p1 = Point3d::new(0.0, 0.0, 0.0);
        let p2 = Point3d::new(1.0, 0.0, 0.0);
        let edge = Edge::new_line(p1, p2);

        let surface = Surface::Plane(Plane::xy());
        let face_id = TopoId::new();

        {
            let disc = cache.discretize_edge(&edge, face_id, &surface, 32, None);
            // Line edges should have exactly 2 points (endpoints)
            assert_eq!(disc.points_3d.len(), 2);
        }

        // Verify cache count after the borrow is released
        assert_eq!(cache.len(), 1);
    }

    #[test]
    fn test_shared_edge_same_points() {
        let mut cache = EdgeDiscretizationCache::new();
        let p1 = Point3d::new(0.0, 0.0, 0.0);
        let p2 = Point3d::new(1.0, 0.0, 0.0);
        let edge = Edge::new_line(p1, p2);

        let surface1 = Surface::Plane(Plane::xy());
        let surface2 = Surface::Plane(Plane::xy());
        let face1_id = TopoId::new();
        let face2_id = TopoId::new();

        // Discretize for face1 — clone the points to release the borrow
        let points_face1 = cache.discretize_edge(&edge, face1_id, &surface1, 32, None).points_3d.clone();

        // Discretize for face2 — should return same 3D points
        let (points_face2, has_face1_uv, has_face2_uv) = {
            let disc2 = cache.discretize_edge(&edge, face2_id, &surface2, 32, None);
            let pts = disc2.points_3d.clone();
            let h1 = disc2.uv_per_face.contains_key(&face1_id);
            let h2 = disc2.uv_per_face.contains_key(&face2_id);
            (pts, h1, h2)
        };

        assert_eq!(points_face1, points_face2, "Shared edges must produce identical 3D points");
        assert!(has_face1_uv, "UV for face1 should be present");
        assert!(has_face2_uv, "UV for face2 should be present");

        // Verify cache count after borrows are released
        assert_eq!(cache.len(), 1, "Edge should only be cached once");
    }

    #[test]
    fn test_curve2d_analytical_uv() {
        // Test that a Curve2d produces correct UV coordinates

        let mut cache = EdgeDiscretizationCache::new();
        let p1 = Point3d::new(0.0, 0.0, 0.0);
        let p2 = Point3d::new(1.0, 0.0, 0.0);
        let edge = Edge::new_line(p1, p2);

        let surface = Surface::Plane(Plane::xy());
        let face_id = TopoId::new();

        // Create a Line2d from (0.5, 0.5) to (1.5, 0.5) in UV space
        let curve_2d = Curve2d::Line(Line2d::new(
            Point2d::new(0.5, 0.5),
            Point2d::new(1.5, 0.5),
        ));

        let disc = cache.discretize_edge(&edge, face_id, &surface, 32, Some(&curve_2d));

        // The UV coordinates should come from the Curve2d, not surface.project_point()
        let uvs = disc.uv_per_face.get(&face_id).unwrap();
        // For a line edge with 2 points (t=0 and t=1):
        // t=0 → point_at(0) = (0.5, 0.5)
        // t=1 → point_at(1) = (1.5, 0.5)
        assert!((uvs[0].u - 0.5).abs() < 1e-10, "Expected u=0.5, got {}", uvs[0].u);
        assert!((uvs[0].v - 0.5).abs() < 1e-10, "Expected v=0.5, got {}", uvs[0].v);
        assert!((uvs[1].u - 1.5).abs() < 1e-10, "Expected u=1.5, got {}", uvs[1].u);
        assert!((uvs[1].v - 0.5).abs() < 1e-10, "Expected v=0.5, got {}", uvs[1].v);
    }

    #[test]
    fn test_curve2d_vs_project_point_cylinder() {
        // Test that a cylinder edge with analytical PCURVE produces
        // UV coordinates consistent with surface.project_point()
        use draper_geometry::CylinderSurface;

        // Create a cylinder with radius 5.0, axis along Z
        let center = Point3d::new(0.0, 0.0, 0.0);
        let axis = Direction3d::new(0.0, 0.0, 1.0).unwrap();
        let cylinder = CylinderSurface::new(center, axis, 5.0);
        let surface = Surface::Cylinder(cylinder);

        // Create a line edge along the cylinder axis (constant theta, varying z)
        let p1 = Point3d::new(5.0, 0.0, 0.0); // theta=0, z=0
        let p2 = Point3d::new(5.0, 0.0, 10.0); // theta=0, z=10
        let edge = Edge::new_line(p1, p2);

        let face_id = TopoId::new();

        // Create a Line2d in UV space: u=0 (theta=0), v goes from 0 to 10
        let curve_2d = Curve2d::Line(Line2d::new(
            Point2d::new(0.0, 0.0),
            Point2d::new(0.0, 10.0),
        ));

        // Compute UV using analytical method
        let mut cache_a = EdgeDiscretizationCache::new();
        let uvs_a = cache_a.discretize_edge(&edge, face_id, &surface, 32, Some(&curve_2d))
            .uv_per_face.get(&face_id).unwrap().clone();

        // Compute UV using project_point method
        let mut cache_p = EdgeDiscretizationCache::new();
        let uvs_p = cache_p.discretize_edge(&edge, face_id, &surface, 32, None)
            .uv_per_face.get(&face_id).unwrap().clone();

        // Both should have the same number of UV points
        assert_eq!(uvs_a.len(), uvs_p.len());

        // For a line along the cylinder axis, both methods should produce
        // similar UV coordinates (u≈0, v from 0 to 10)
        for i in 0..uvs_a.len() {
            // Allow some tolerance — project_point is approximate for cylinders
            let du = (uvs_a[i].u - uvs_p[i].u).abs();
            let dv = (uvs_a[i].v - uvs_p[i].v).abs();
            // The analytical curve_2d should give exact UV; project_point is approximate
            // We allow a generous tolerance since project_point can have errors
            assert!(du < 0.5, "u mismatch at point {}: analytical={}, projected={}", i, uvs_a[i].u, uvs_p[i].u);
            assert!(dv < 0.5, "v mismatch at point {}: analytical={}, projected={}", i, uvs_a[i].v, uvs_p[i].v);
        }
    }

    #[test]
    fn test_point_to_chord_distance() {
        let a = Point3d::new(0.0, 0.0, 0.0);
        let b = Point3d::new(1.0, 0.0, 0.0);

        // Point on the chord
        let on_chord = Point3d::new(0.5, 0.0, 0.0);
        assert!(point_to_chord_distance(&on_chord, &a, &b) < 1e-10);

        // Point perpendicular to chord
        let perp = Point3d::new(0.5, 1.0, 0.0);
        let dist = point_to_chord_distance(&perp, &a, &b);
        assert!((dist - 1.0).abs() < 1e-10);
    }
}
