//! Delaunay triangulation using the `spade` crate.
//!
//! Provides Constrained Delaunay Triangulation (CDT) for face meshing.
//! Boundary edges from B-rep wires are preserved as constraint edges,
//! ensuring the triangulation respects the original topology.
//!
//! Key function: `cdt_with_constraints` — builds a CDT from a set of
//! points and constraint edges, supporting holes and nested contours.

use draper_geometry::point::Point2;
use spade::{ConstrainedDelaunayTriangulation, Point2 as SpadePoint2, Triangulation};

/// A wrapper type that implements spade's `HasPosition` trait for our `Point2`.
/// Stores the original vertex index for mapping back to the input array.
#[derive(Debug, Clone, Copy)]
struct MeshPoint {
    /// The 2D position in spade's coordinate system.
    pos: SpadePoint2<f64>,
    /// The original index in the input point list.
    original_index: u32,
}

impl spade::HasPosition for MeshPoint {
    type Scalar = f64;

    fn position(&self) -> SpadePoint2<f64> {
        self.pos
    }
}

/// Deduplicate points that are very close together.
/// Returns (deduplicated_points, index_map) where index_map[i] gives the
/// deduplicated index for original index i.
fn deduplicate_points(vertices: &[Point2], eps: f64) -> (Vec<Point2>, Vec<u32>) {
    let mut deduped: Vec<Point2> = Vec::new();
    let mut index_map = Vec::with_capacity(vertices.len());

    for (_i, pt) in vertices.iter().enumerate() {
        // Check if this point is close to any already-added point
        let mut found = None;
        for (j, existing) in deduped.iter().enumerate() {
            if (existing.u - pt.u).abs() < eps && (existing.v - pt.v).abs() < eps {
                found = Some(j as u32);
                break;
            }
        }

        match found {
            Some(dedup_idx) => {
                index_map.push(dedup_idx);
            }
            None => {
                let dedup_idx = deduped.len() as u32;
                deduped.push(*pt);
                index_map.push(dedup_idx);
            }
        }
    }

    (deduped, index_map)
}

/// Build a CDT from a set of 2D points with constraint edges.
///
/// This is the primary CDT entry point for the production pipeline.
/// It:
/// 1. Deduplicates input points
/// 2. Inserts all vertices into the triangulation
/// 3. Adds all constraint edges (preserving topology)
/// 4. Extracts triangle indices mapped back to original indices
///
/// Returns triangle indices (3 per triangle) referencing the input point array.
pub fn cdt_with_constraints(points: &[Point2], constraints: &[(usize, usize)]) -> Vec<u32> {
    if points.len() < 3 {
        return Vec::new();
    }

    // Deduplicate points
    let (deduped, index_map) = deduplicate_points(points, 1e-10);
    if deduped.len() < 3 {
        return Vec::new();
    }

    // Build CDT
    let mut cdt = ConstrainedDelaunayTriangulation::<MeshPoint>::new();

    // Insert deduplicated vertices
    let mut handles: Vec<spade::handles::FixedVertexHandle> = Vec::with_capacity(deduped.len());
    for (i, pt) in deduped.iter().enumerate() {
        let mesh_pt = MeshPoint {
            pos: SpadePoint2::new(pt.u, pt.v),
            original_index: i as u32,
        };
        match cdt.insert(mesh_pt) {
            Ok(handle) => handles.push(handle),
            Err(e) => {
                log::debug!("CDT: failed to insert point at index {}: {:?}", i, e);
                continue;
            }
        }
    }

    if handles.len() < 3 {
        return Vec::new();
    }

    // Add constraint edges
    let mut constraints_added = 0;
    let mut constraints_skipped = 0;
    for &(a, b) in constraints {
        if a >= points.len() || b >= points.len() {
            continue;
        }
        let da = index_map[a] as usize;
        let db = index_map[b] as usize;
        if da == db {
            continue;
        }
        if da >= handles.len() || db >= handles.len() {
            continue;
        }
        let result = cdt.try_add_constraint(handles[da], handles[db]);
        if result.is_empty() {
            constraints_skipped += 1;
        } else {
            constraints_added += 1;
        }
    }

    log::trace!(
        "CDT: {} points, {} constraints added, {} skipped",
        deduped.len(),
        constraints_added,
        constraints_skipped,
    );

    // Extract triangle indices
    let deduped_indices = extract_inner_triangles(&cdt);

    if deduped_indices.is_empty() {
        return Vec::new();
    }

    // Build reverse map: deduplicated index → original index
    let mut reverse_map = vec![0u32; deduped.len()];
    for (orig_idx, &dedup_idx) in index_map.iter().enumerate() {
        reverse_map[dedup_idx as usize] = orig_idx as u32;
    }

    // Map deduplicated indices back to original indices
    deduped_indices
        .into_iter()
        .map(|idx| reverse_map[idx as usize])
        .collect()
}

/// Triangulate a simple polygon (no holes) using Constrained Delaunay Triangulation.
///
/// Boundary edges are added as constraints to preserve the polygon outline.
/// Returns a list of triangle indices (3 indices per triangle), referencing the
/// original input point indices.
pub fn cdt_polygon(vertices: &[Point2]) -> Vec<u32> {
    if vertices.len() < 3 {
        return Vec::new();
    }

    // Build constraints for the polygon boundary
    let constraints: Vec<(usize, usize)> = (0..vertices.len())
        .map(|i| (i, (i + 1) % vertices.len()))
        .collect();

    cdt_with_constraints(vertices, &constraints)
}

/// Triangulate a polygon with holes using Constrained Delaunay Triangulation.
///
/// The outer boundary and each hole boundary are added as constraint edges.
/// Only triangles inside the outer boundary and outside all holes are kept.
pub fn cdt_polygon_with_holes(outer: &[Point2], holes: &[Vec<Point2>]) -> Vec<u32> {
    if outer.len() < 3 {
        return Vec::new();
    }

    // Build combined point array
    let mut all_points: Vec<Point2> = outer.to_vec();
    for hole in holes {
        all_points.extend_from_slice(hole);
    }

    // Build constraints for outer and hole boundaries
    let mut constraints: Vec<(usize, usize)> = Vec::new();

    // Outer boundary
    for i in 0..outer.len() {
        constraints.push((i, (i + 1) % outer.len()));
    }

    // Hole boundaries
    let mut offset = outer.len();
    for hole in holes {
        for i in 0..hole.len() {
            constraints.push((offset + i, offset + (i + 1) % hole.len()));
        }
        offset += hole.len();
    }

    // Build CDT
    let mapped_indices = cdt_with_constraints(&all_points, &constraints);

    if mapped_indices.is_empty() {
        return Vec::new();
    }

    // Filter out triangles outside the outer boundary or inside holes
    filter_triangles_inside_polygon(&mapped_indices, &all_points, outer, holes)
}

/// Extract triangle indices from the CDT, mapping back to deduplicated point indices.
fn extract_inner_triangles(cdt: &ConstrainedDelaunayTriangulation<MeshPoint>) -> Vec<u32> {
    let mut triangles = Vec::new();

    for face in cdt.inner_faces() {
        let verts = face.vertices();
        let a = verts[0].data().original_index;
        let b = verts[1].data().original_index;
        let c = verts[2].data().original_index;

        // Skip degenerate triangles (all same index)
        if a != b && b != c && a != c {
            triangles.push(a);
            triangles.push(b);
            triangles.push(c);
        }
    }

    triangles
}

/// Filter triangles to keep only those inside the polygon boundary and outside holes.
fn filter_triangles_inside_polygon(
    indices: &[u32],
    all_points: &[Point2],
    outer: &[Point2],
    holes: &[Vec<Point2>],
) -> Vec<u32> {
    let mut result = Vec::with_capacity(indices.len());

    for tri in indices.chunks(3) {
        if tri.len() < 3 {
            continue;
        }

        let ai = tri[0] as usize;
        let bi = tri[1] as usize;
        let ci = tri[2] as usize;

        if ai >= all_points.len() || bi >= all_points.len() || ci >= all_points.len() {
            continue;
        }

        let centroid = Point2::new(
            (all_points[ai].u + all_points[bi].u + all_points[ci].u) / 3.0,
            (all_points[ai].v + all_points[bi].v + all_points[ci].v) / 3.0,
        );

        if !point_in_polygon(&centroid, outer) {
            continue;
        }

        let mut inside_hole = false;
        for hole in holes {
            if point_in_polygon(&centroid, hole) {
                inside_hole = true;
                break;
            }
        }
        if inside_hole {
            continue;
        }

        result.push(tri[0]);
        result.push(tri[1]);
        result.push(tri[2]);
    }

    result
}

/// Test if a point is inside a polygon using the ray-casting algorithm.
pub fn point_in_polygon(point: &Point2, polygon: &[Point2]) -> bool {
    if polygon.len() < 3 {
        return false;
    }

    let mut inside = false;
    let n = polygon.len();

    for i in 0..n {
        let j = (i + 1) % n;
        let pi = &polygon[i];
        let pj = &polygon[j];

        if ((pi.v > point.v) != (pj.v > point.v))
            && (point.u < (pj.u - pi.u) * (point.v - pi.v) / (pj.v - pi.v) + pi.u)
        {
            inside = !inside;
        }
    }

    inside
}

/// Legacy entry point for face triangulation using CDT.
pub fn triangulate_face_boundary(
    points_2d: &[Point2],
    holes_2d: &[Vec<Point2>],
) -> Vec<u32> {
    if holes_2d.is_empty() {
        cdt_polygon(points_2d)
    } else {
        cdt_polygon_with_holes(points_2d, holes_2d)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cdt_square() {
        let vertices = vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(1.0, 1.0),
            Point2::new(0.0, 1.0),
        ];

        let triangles = cdt_polygon(&vertices);
        assert_eq!(triangles.len(), 6); // 2 triangles * 3 indices
    }

    #[test]
    fn test_cdt_pentagon() {
        let vertices = vec![
            Point2::new(0.0, 1.0),
            Point2::new(1.0, 0.5),
            Point2::new(0.8, -0.5),
            Point2::new(-0.8, -0.5),
            Point2::new(-1.0, 0.5),
        ];

        let triangles = cdt_polygon(&vertices);
        assert_eq!(triangles.len(), 9); // 3 triangles * 3 indices
    }

    #[test]
    fn test_cdt_square_with_hole() {
        let outer = vec![
            Point2::new(0.0, 0.0),
            Point2::new(4.0, 0.0),
            Point2::new(4.0, 4.0),
            Point2::new(0.0, 4.0),
        ];
        let hole = vec![
            Point2::new(1.0, 1.0),
            Point2::new(1.0, 2.0),
            Point2::new(2.0, 2.0),
            Point2::new(2.0, 1.0),
        ];

        let triangles = cdt_polygon_with_holes(&outer, &[hole]);
        assert!(triangles.len() >= 6, "Expected at least 6 indices, got {}", triangles.len());
        assert!(triangles.len() % 3 == 0, "Triangle indices should be a multiple of 3");
    }

    #[test]
    fn test_cdt_with_constraints() {
        let points = vec![
            Point2::new(0.0, 0.0),
            Point2::new(2.0, 0.0),
            Point2::new(2.0, 2.0),
            Point2::new(0.0, 2.0),
            Point2::new(1.0, 1.0), // Interior point
        ];
        let constraints = vec![
            (0, 1), (1, 2), (2, 3), (3, 0), // Outer boundary
        ];

        let triangles = cdt_with_constraints(&points, &constraints);
        assert!(triangles.len() >= 9, "Expected at least 9 indices, got {}", triangles.len());
        assert!(triangles.len() % 3 == 0);
    }

    #[test]
    fn test_point_in_polygon() {
        let square = vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(1.0, 1.0),
            Point2::new(0.0, 1.0),
        ];

        assert!(point_in_polygon(&Point2::new(0.5, 0.5), &square));
        assert!(!point_in_polygon(&Point2::new(1.5, 0.5), &square));
        assert!(!point_in_polygon(&Point2::new(-0.5, 0.5), &square));
    }
}
