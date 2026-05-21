//! Consistent edge discretization for triangulation.
//!
//! Implements curvature-based adaptive sampling of edges, producing
//! mandatory boundary points in both 3D and UV space. This is the
//! critical preparation step before CDT triangulation.
//!
//! Key principle: "Edge discretization density must never be coarser than
//! the surface discretization in the same region." Edge points become
//! mandatory constraints in the CDT.

use crate::entity::*;
use crate::shape::Shape;
use draper_geometry::curve::Curve;
use draper_geometry::pcurve::PCurve;
use draper_geometry::point::{Point2, Point3};

/// A discretized point on an edge, carrying both 3D and UV information.
#[derive(Debug, Clone)]
pub struct DiscretizedPoint {
    /// 3D position on the curve.
    pub point_3d: Point3,
    /// UV position on the face's surface (if pcurve is available).
    pub point_uv: Option<Point2>,
    /// Parameter value on the edge's curve.
    pub t: f64,
    /// Whether this point is a vertex (start/end of the edge).
    pub is_vertex: bool,
}

/// Result of discretizing all edges of a face's wires.
#[derive(Debug, Clone)]
pub struct FaceDiscretization {
    /// Points and constraints from the outer wire.
    pub outer_contour: WireDiscretization,
    /// Points and constraints from inner wires (holes).
    pub inner_contours: Vec<WireDiscretization>,
    /// All mandatory points (union of all edge discretization points).
    pub mandatory_points: Vec<DiscretizedPoint>,
    /// Constraint edges as pairs of indices into mandatory_points.
    pub constraint_edges: Vec<(usize, usize)>,
}

/// Discretization of a single wire.
#[derive(Debug, Clone)]
pub struct WireDiscretization {
    /// Discretized points in order along the wire.
    pub points: Vec<DiscretizedPoint>,
    /// UV points for CDT (if pcurves are available).
    pub uv_points: Vec<Point2>,
    /// 3D points for mesh generation.
    pub points_3d: Vec<Point3>,
    /// Constraint edge pairs (indices into points array).
    pub constraint_pairs: Vec<(usize, usize)>,
}

/// Discretize all edges of a face, producing boundary points and constraints.
pub fn discretize_face(shape: &Shape, face: &Face) -> FaceDiscretization {
    let mut outer_contour = WireDiscretization {
        points: Vec::new(),
        uv_points: Vec::new(),
        points_3d: Vec::new(),
        constraint_pairs: Vec::new(),
    };
    let mut inner_contours = Vec::new();

    // Discretize outer wire
    if let Some(wire_id) = face.outer_wire {
        outer_contour = discretize_wire(shape, wire_id, face.id);
    }

    // Discretize inner wires
    for &wire_id in &face.inner_wires {
        let disc = discretize_wire(shape, wire_id, face.id);
        inner_contours.push(disc);
    }

    // Build mandatory points and constraint edges
    let mut mandatory_points = Vec::new();
    let mut constraint_edges = Vec::new();

    // Add outer contour points
    let outer_start = mandatory_points.len();
    for pt in &outer_contour.points {
        mandatory_points.push(pt.clone());
    }
    // Add constraint edges for outer contour
    for i in 0..outer_contour.points.len() {
        let j = (i + 1) % outer_contour.points.len();
        constraint_edges.push((outer_start + i, outer_start + j));
    }

    // Add inner contour points
    for inner in &inner_contours {
        let inner_start = mandatory_points.len();
        for pt in &inner.points {
            mandatory_points.push(pt.clone());
        }
        for i in 0..inner.points.len() {
            let j = (i + 1) % inner.points.len();
            constraint_edges.push((inner_start + i, inner_start + j));
        }
    }

    FaceDiscretization {
        outer_contour,
        inner_contours,
        mandatory_points,
        constraint_edges,
    }
}

/// Discretize a single wire into boundary points.
fn discretize_wire(shape: &Shape, wire_id: TopoId, face_id: TopoId) -> WireDiscretization {
    let wire = match shape.get(wire_id) {
        Some(TopoShape::Wire(w)) => w,
        _ => {
            return WireDiscretization {
                points: Vec::new(),
                uv_points: Vec::new(),
                points_3d: Vec::new(),
                constraint_pairs: Vec::new(),
            }
        }
    };

    let mut points = Vec::new();
    let mut uv_points = Vec::new();
    let mut points_3d = Vec::new();
    let mut constraint_pairs = Vec::new();

    for (edge_idx, oriented_edge) in wire.edges.iter().enumerate() {
        let edge = match shape.get(oriented_edge.edge_id) {
            Some(TopoShape::Edge(e)) => e.clone(),
            _ => continue,
        };

        // Get the pcurve for this edge on this face (if available)
        let pcurve = edge.get_pcurve(face_id).map(|pc| &pc.pcurve);

        // Compute discretization points for this edge
        let edge_points = discretize_edge(&edge, pcurve, oriented_edge.orientation);

        // Add edge points to wire discretization
        // Skip the first point if it duplicates the last point of the previous edge
        let start_idx = if edge_idx > 0 && !edge_points.is_empty() { 1 } else { 0 };

        for (i, dp) in edge_points.iter().enumerate() {
            if i < start_idx {
                continue;
            }
            let _idx = points.len();
            points.push(dp.clone());
            points_3d.push(dp.point_3d);
            uv_points.push(dp.point_uv.unwrap_or(Point2::ORIGIN));
        }

        // Add constraint edge pairs within this edge's discretization
        let wire_start = points.len() - (edge_points.len() - start_idx);
        for i in wire_start..points.len() - 1 {
            constraint_pairs.push((i, i + 1));
        }
    }

    // Close the wire with a constraint from last to first
    if points.len() >= 3 {
        constraint_pairs.push((points.len() - 1, 0));
    }

    WireDiscretization {
        points,
        uv_points,
        points_3d,
        constraint_pairs,
    }
}

/// Discretize a single edge using curvature-based adaptive sampling.
///
/// For each edge:
/// 1. If it has a pcurve, sample the pcurve in UV space
/// 2. Always compute 3D positions from the edge's curve
/// 3. Adaptive subdivision based on chord error and angular deviation
fn discretize_edge(
    edge: &Edge,
    pcurve: Option<&PCurve>,
    orientation: bool,
) -> Vec<DiscretizedPoint> {
    let curve = match &edge.curve {
        Some(c) => c,
        None => return discretize_line_edge(edge, pcurve, orientation),
    };

    // Get parameter range
    let (t_min, t_max) = edge
        .parameter_range
        .unwrap_or_else(|| default_param_range(curve));

    // Compute adaptive samples
    let samples = compute_adaptive_samples(curve, t_min, t_max, 1e-3, 48);

    let mut points = Vec::with_capacity(samples.len());

    for &t in &samples {
        // Map parameter to 3D
        let mapped_t = map_param(curve, t, t_min, t_max);
        let point_3d = curve.point_at(mapped_t);

        // Map parameter to UV (via pcurve if available)
        let normalized_t = if (t_max - t_min).abs() > 1e-10 {
            (t - t_min) / (t_max - t_min)
        } else {
            0.0
        };

        let point_uv = pcurve.map(|pc| pc.point_at(normalized_t));

        let is_vertex = (t - t_min).abs() < 1e-10 || (t - t_max).abs() < 1e-10;

        points.push(DiscretizedPoint {
            point_3d,
            point_uv,
            t,
            is_vertex,
        });
    }

    // Reverse if orientation is false
    if !orientation {
        points.reverse();
    }

    // Ensure first and last points are exact vertices
    if let Some(first) = points.first_mut() {
        first.is_vertex = true;
    }
    if let Some(last) = points.last_mut() {
        last.is_vertex = true;
    }

    points
}

/// Discretize a line edge (no curve geometry) from vertex positions.
fn discretize_line_edge(
    _edge: &Edge,
    pcurve: Option<&PCurve>,
    orientation: bool,
) -> Vec<DiscretizedPoint> {
    // For line edges, we just need start and end points
    let mut points = Vec::new();

    // Start point
    let uv_start = pcurve.map(|pc| pc.point_at(0.0));
    points.push(DiscretizedPoint {
        point_3d: Point3::ORIGIN, // Will be filled from vertex
        point_uv: uv_start,
        t: 0.0,
        is_vertex: true,
    });

    // End point
    let uv_end = pcurve.map(|pc| pc.point_at(1.0));
    points.push(DiscretizedPoint {
        point_3d: Point3::ORIGIN,
        point_uv: uv_end,
        t: 1.0,
        is_vertex: true,
    });

    if !orientation {
        points.reverse();
    }

    points
}

/// Compute adaptive parameter samples based on curvature.
///
/// Uses a recursive midpoint subdivision approach:
/// - Start with the endpoints
/// - At each midpoint, check if the chord error exceeds tolerance
/// - If so, subdivide further
/// - Limit total number of samples
fn compute_adaptive_samples(
    curve: &Curve,
    t_min: f64,
    t_max: f64,
    chord_tolerance: f64,
    max_segments: usize,
) -> Vec<f64> {
    let mut samples = vec![t_min, t_max];
    let mut stack = vec![(t_min, t_max)];

    while let Some((a, b)) = stack.pop() {
        if samples.len() >= max_segments + 1 {
            break;
        }

        let mid = (a + b) / 2.0;
        let pa = curve.point_at(map_param(curve, a, t_min, t_max));
        let pb = curve.point_at(map_param(curve, b, t_min, t_max));
        let pm = curve.point_at(map_param(curve, mid, t_min, t_max));

        // Chord error: distance from midpoint to chord
        let chord_error = point_to_segment_distance(pm, pa, pb);

        if chord_error > chord_tolerance {
            // Insert midpoint and continue subdividing
            insert_sorted(&mut samples, mid);
            stack.push((a, mid));
            stack.push((mid, b));
        }
    }

    // If we have very few samples, add more for smoothness
    let min_segments = match curve {
        Curve::Line(_) => 1,
        Curve::Circle(_) => 16,
        Curve::Ellipse(_) => 16,
        Curve::BSplineCurve(_) => 8,
        Curve::OffsetCurve(_) => 8,
        Curve::TrimmedCurve(_) => 8,
    };

    while samples.len() < min_segments + 1 {
        let mut new_samples = Vec::new();
        let mut i = 0;
        while i < samples.len() - 1 {
            new_samples.push(samples[i]);
            let mid = (samples[i] + samples[i + 1]) / 2.0;
            new_samples.push(mid);
            i += 1;
        }
        new_samples.push(*samples.last().unwrap());
        samples = new_samples;
    }

    samples
}

/// Distance from point P to line segment AB.
fn point_to_segment_distance(p: Point3, a: Point3, b: Point3) -> f64 {
    let ab = b - a;
    let ap = p - a;
    let ab_len_sq = ab.dot(ab);

    if ab_len_sq < 1e-20 {
        return p.distance_to(a);
    }

    let t = (ap.dot(ab) / ab_len_sq).clamp(0.0, 1.0);
    let closest = a + ab * t;
    p.distance_to(closest)
}

/// Map a parameter from edge parameter space to curve parameter space.
fn map_param(curve: &Curve, t: f64, _t_min: f64, _t_max: f64) -> f64 {
    // For TrimmedCurve, the parameter mapping is handled internally
    // For other curves, the parameter is used directly
    match curve {
        Curve::TrimmedCurve(tc) => {
            // Map from [0,1] to [trim1, trim2]
            tc.trim1 + t * (tc.trim2 - tc.trim1)
        }
        _ => t,
    }
}

/// Get default parameter range for a curve type.
fn default_param_range(curve: &Curve) -> (f64, f64) {
    match curve {
        Curve::Line(_) => (0.0, 1.0),
        Curve::Circle(_) => (0.0, 2.0 * std::f64::consts::PI),
        Curve::Ellipse(_) => (0.0, 2.0 * std::f64::consts::PI),
        Curve::BSplineCurve(bs) => (
            bs.knots.first().copied().unwrap_or(0.0),
            bs.knots.last().copied().unwrap_or(1.0),
        ),
        Curve::OffsetCurve(_) => (0.0, 1.0),
        Curve::TrimmedCurve(tc) => (tc.trim1, tc.trim2),
    }
}

/// Insert a value into a sorted vector, maintaining order.
fn insert_sorted(vec: &mut Vec<f64>, val: f64) {
    match vec.binary_search_by(|probe| probe.partial_cmp(&val).unwrap()) {
        Ok(_) => {} // Already present
        Err(idx) => vec.insert(idx, val),
    }
}
