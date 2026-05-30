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

use draper_geometry::{Point3d, Point2d, Curve3d, Surface, tolerance::ToleranceContext};
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
    pub fn discretize_edge(
        &mut self,
        edge: &Edge,
        face_id: TopoId,
        surface: &Surface,
        n_samples_hint: usize,
    ) -> &EdgeDiscretization {
        let edge_id = edge.id;

        // If edge already in cache, just add UV if needed
        if self.entries.contains_key(&edge_id) {
            let entry = self.entries.get_mut(&edge_id).unwrap();
            if !entry.uv_per_face.contains_key(&face_id) {
                let uvs = Self::compute_uvs(&entry.points_3d, surface);
                entry.uv_per_face.insert(face_id, uvs);
            }
            return self.entries.get(&edge_id).unwrap();
        }

        // Discretize the edge adaptively
        let (points_3d, params) = self.adaptive_discretize(edge, n_samples_hint);

        // Compute UV for this face
        let uvs = Self::compute_uvs(&points_3d, surface);

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
    fn compute_uvs(points_3d: &[Point3d], surface: &Surface) -> Vec<Point2d> {
        points_3d
            .iter()
            .map(|p| {
                let (u, v) = surface.project_point(p);
                Point2d::new(u, v)
            })
            .collect()
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
    use draper_geometry::{Point3d, Plane};
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
            let disc = cache.discretize_edge(&edge, face_id, &surface, 32);
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
        let points_face1 = cache.discretize_edge(&edge, face1_id, &surface1, 32).points_3d.clone();

        // Discretize for face2 — should return same 3D points
        let (points_face2, has_face1_uv, has_face2_uv) = {
            let disc2 = cache.discretize_edge(&edge, face2_id, &surface2, 32);
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
