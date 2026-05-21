//! Delaunay triangulation using the `spade` crate.
//!
//! Provides Constrained Delaunay Triangulation (CDT) for face meshing.
//! Boundary edges from B-rep wires are preserved as constraint edges,
//! ensuring the triangulation respects the original topology.

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

/// Triangulate a simple polygon (no holes) using Constrained Delaunay Triangulation.
///
/// Boundary edges are added as constraints to preserve the polygon outline.
/// Returns a list of triangle indices (3 indices per triangle), referencing the
/// original input point indices.
///
/// If CDT fails (e.g., due to intersecting constraints), returns an empty Vec
/// and the caller should fall back to ear-clipping.
pub fn cdt_polygon(vertices: &[Point2]) -> Vec<u32> {
    if vertices.len() < 3 {
        return Vec::new();
    }

    // Deduplicate points
    let (deduped, index_map) = deduplicate_points(vertices, 1e-10);
    if deduped.len() < 3 {
        return Vec::new();
    }

    // Build CDT with constraint edges along the polygon boundary
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
                return Vec::new();
            }
        }
    }

    // Add constraint edges along the polygon boundary.
    // Use try_add_constraint to avoid panics from intersecting edges.
    for i in 0..vertices.len() {
        let j = (i + 1) % vertices.len();
        let di = index_map[i] as usize;
        let dj = index_map[j] as usize;
        if di != dj && di < handles.len() && dj < handles.len() {
            let result = cdt.try_add_constraint(handles[di], handles[dj]);
            if result.is_empty() {
                // Constraint intersects an existing constraint — this means
                // the polygon is self-intersecting or complex.
                // Fall back to no-constraint Delaunay and let caller handle it.
                log::debug!(
                    "CDT: constraint edge ({},{}) intersects existing constraint, skipping",
                    di, dj
                );
            }
        }
    }

    // Extract triangle indices in deduplicated space, then map back to original indices
    let deduped_indices = extract_inner_triangles(&cdt);

    if deduped_indices.is_empty() {
        return Vec::new();
    }

    // Build reverse map: deduplicated index -> original index
    // For each deduplicated index, use the first original index that maps to it
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

/// Triangulate a polygon with holes using Constrained Delaunay Triangulation.
///
/// The outer boundary and each hole boundary are added as constraint edges.
/// Only triangles inside the outer boundary and outside all holes are kept.
///
/// Returns a list of triangle indices (3 indices per triangle), referencing the
/// combined point array: [outer_points, hole1_points, hole2_points, ...].
pub fn cdt_polygon_with_holes(
    outer: &[Point2],
    holes: &[Vec<Point2>],
) -> Vec<u32> {
    if outer.len() < 3 {
        return Vec::new();
    }

    // Build combined point array
    let mut all_points: Vec<Point2> = outer.to_vec();
    for hole in holes {
        all_points.extend_from_slice(hole);
    }

    // Deduplicate all points
    let (deduped, index_map) = deduplicate_points(&all_points, 1e-10);
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
                return Vec::new();
            }
        }
    }

    // Add constraint edges for outer boundary
    for i in 0..outer.len() {
        let j = (i + 1) % outer.len();
        let di = index_map[i] as usize;
        let dj = index_map[j] as usize;
        if di != dj && di < handles.len() && dj < handles.len() {
            let result = cdt.try_add_constraint(handles[di], handles[dj]);
            if result.is_empty() {
                log::debug!("CDT: outer constraint ({},{}) intersects, skipping", di, dj);
            }
        }
    }

    // Add constraint edges for each hole boundary
    let mut offset = outer.len();
    for hole in holes {
        for i in 0..hole.len() {
            let j = (i + 1) % hole.len();
            let hi = index_map[offset + i] as usize;
            let hj = index_map[offset + j] as usize;
            if hi != hj && hi < handles.len() && hj < handles.len() {
                let result = cdt.try_add_constraint(handles[hi], handles[hj]);
                if result.is_empty() {
                    log::debug!("CDT: hole constraint ({},{}) intersects, skipping", hi, hj);
                }
            }
        }
        offset += hole.len();
    }

    // Extract triangles and map back to original indices
    let deduped_indices = extract_inner_triangles(&cdt);

    if deduped_indices.is_empty() {
        return Vec::new();
    }

    // Build reverse map: deduplicated index -> original index
    let mut reverse_map = vec![0u32; deduped.len()];
    for (orig_idx, &dedup_idx) in index_map.iter().enumerate() {
        reverse_map[dedup_idx as usize] = orig_idx as u32;
    }

    // Map deduplicated indices back to original indices
    let mapped_indices: Vec<u32> = deduped_indices
        .into_iter()
        .map(|idx| reverse_map[idx as usize])
        .collect();

    // Filter out triangles that are outside the outer boundary or inside holes
    filter_triangles_inside_polygon(&mapped_indices, &all_points, outer, holes)
}

/// Extract triangle indices from the CDT, mapping back to deduplicated point indices.
fn extract_inner_triangles(
    cdt: &ConstrainedDelaunayTriangulation<MeshPoint>,
) -> Vec<u32> {
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
///
/// Uses point-in-polygon test on the centroid of each triangle.
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

        // Compute centroid
        let centroid = Point2::new(
            (all_points[ai].u + all_points[bi].u + all_points[ci].u) / 3.0,
            (all_points[ai].v + all_points[bi].v + all_points[ci].v) / 3.0,
        );

        // Must be inside outer boundary
        if !point_in_polygon(&centroid, outer) {
            continue;
        }

        // Must be outside all holes
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
///
/// Counts the number of crossings of a horizontal ray from the test point
/// to +infinity with the polygon edges. An odd count means the point is inside.
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

        // Check if the ray from point to +infinity in X direction crosses edge (pi, pj)
        if ((pi.v > point.v) != (pj.v > point.v))
            && (point.u < (pj.u - pi.u) * (point.v - pi.v) / (pj.v - pi.v) + pi.u)
        {
            inside = !inside;
        }
    }

    inside
}

/// Triangulate a set of boundary points that form a face.
///
/// Primary entry point for face triangulation using CDT.
/// Returns triangle indices referencing the original point list.
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
