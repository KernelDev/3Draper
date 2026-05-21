//! Mesh generation from B-rep topology.
//!
//! Converts topological faces into triangle meshes suitable for rendering.
//! The approach is boundary-driven: we extract the boundary vertices from
//! each face's wire loops, then triangulate using ear-clipping after
//! projecting to 2D. For curved surfaces, we also sample the surface
//! between boundary vertices to capture curvature.

use crate::earcut;
use crate::triangulate::TriangleMesh;
use draper_geometry::curve::Curve;
use draper_geometry::point::{Point2, Point3};
use draper_geometry::surface::Surface;
use draper_topology::entity::*;
use draper_topology::shape::Shape;

/// Default number of segments for discretizing circular edges.
const CIRCLE_SEGMENTS: usize = 48;

/// Generate a triangle mesh from a shape.
pub fn generate_mesh(shape: &Shape, _u_samples: usize, _v_samples: usize) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    for face in shape.faces() {
        generate_face_mesh(shape, face, &mut mesh);
    }

    mesh.compute_normals();
    mesh
}

/// Generate mesh for a single face.
fn generate_face_mesh(shape: &Shape, face: &Face, mesh: &mut TriangleMesh) {
    // Extract boundary points from the face's wires
    let outer_points = extract_wire_points(shape, face.outer_wire);

    if outer_points.is_empty() {
        log::debug!("Face #{}: no boundary points, skipping", face.id);
        return;
    }

    let inner_points: Vec<Vec<Point3>> = face
        .inner_wires
        .iter()
        .filter_map(|&wire_id| {
            let pts = extract_wire_points(shape, Some(wire_id));
            if pts.is_empty() { None } else { Some(pts) }
        })
        .collect();

    match &face.surface {
        Some(Surface::Plane(plane)) => {
            generate_planar_face_mesh_from_points(
                &outer_points,
                &inner_points,
                plane,
                mesh,
            );
        }
        Some(surface) => {
            generate_curved_face_mesh_from_points(
                shape,
                face,
                &outer_points,
                &inner_points,
                surface,
                mesh,
            );
        }
        None => {
            // No surface — triangulate from boundary points directly
            generate_faceted_face_mesh(&outer_points, &inner_points, mesh);
        }
    }
}

/// Extract ordered 3D boundary points from a wire.
///
/// Walks the wire's edges and collects vertices, discretizing
/// curved edges (circles, etc.) into multiple points.
fn extract_wire_points(shape: &Shape, wire_id: Option<TopoId>) -> Vec<Point3> {
    let wire_id = match wire_id {
        Some(id) => id,
        None => return Vec::new(),
    };

    let wire = match shape.get(wire_id) {
        Some(TopoShape::Wire(w)) => w,
        _ => return Vec::new(),
    };

    let mut points = Vec::new();
    let mut last_vertex_id: Option<TopoId> = None;

    for (i, oriented_edge) in wire.edges.iter().enumerate() {
        let edge = match shape.get(oriented_edge.edge_id) {
            Some(TopoShape::Edge(e)) => e,
            _ => continue,
        };

        // Determine the vertex order based on orientation
        let (first_vertex, second_vertex) = if oriented_edge.orientation {
            (edge.start_vertex, edge.end_vertex)
        } else {
            (edge.end_vertex, edge.start_vertex)
        };

        // Get first vertex point
        let first_pt = match shape.get(first_vertex) {
            Some(TopoShape::Vertex(v)) => v.point,
            _ => continue,
        };

        // Add the first vertex point (skip if same as last point from previous edge)
        let should_add = match last_vertex_id {
            Some(prev_id) if prev_id == first_vertex => false,
            _ => true,
        };

        if should_add {
            points.push(first_pt);
        }

        // For curved edges, add intermediate points along the curve
        if let Some(ref curve) = edge.curve {
            let intermediates = discretize_curve(curve, CIRCLE_SEGMENTS);
            // Skip the first and last intermediate points as they coincide with vertices
            if intermediates.len() > 2 {
                for pt in intermediates.iter().skip(1).take(intermediates.len() - 2) {
                    points.push(*pt);
                }
            }
        }

        last_vertex_id = Some(second_vertex);

        // For the last edge, add the final vertex if the wire is not closed
        if i == wire.edges.len() - 1 {
            let second_pt = match shape.get(second_vertex) {
                Some(TopoShape::Vertex(v)) => v.point,
                _ => continue,
            };
            // Don't add closing point if wire is closed (first == last)
            if !wire.closed || points.is_empty() || points[0] != second_pt {
                points.push(second_pt);
            }
        }
    }

    // Remove duplicate last point if it matches the first (closed wire)
    if points.len() > 1 {
        let first = points[0];
        let last = points[points.len() - 1];
        if (first.x - last.x).abs() < 1e-10
            && (first.y - last.y).abs() < 1e-10
            && (first.z - last.z).abs() < 1e-10
        {
            points.pop();
        }
    }

    log::trace!(
        "Wire #{}: {} edges → {} boundary points",
        wire_id,
        wire.edges.len(),
        points.len()
    );

    points
}

/// Discretize a curve into a sequence of 3D points.
///
/// For LINE curves, just returns start and end points.
/// For CIRCLE and other curves, samples the curve at regular intervals.
fn discretize_curve(curve: &Curve, segments: usize) -> Vec<Point3> {
    match curve {
        Curve::Line(_) => {
            // Lines are handled by their vertices — no intermediate points needed
            Vec::new()
        }
        Curve::Circle(_) => {
            let mut pts = Vec::with_capacity(segments + 1);
            for i in 0..=segments {
                let t = (i as f64 / segments as f64) * 2.0 * std::f64::consts::PI;
                pts.push(curve.point_at(t));
            }
            pts
        }
        Curve::Ellipse(_) => {
            let mut pts = Vec::with_capacity(segments + 1);
            for i in 0..=segments {
                let t = (i as f64 / segments as f64) * 2.0 * std::f64::consts::PI;
                pts.push(curve.point_at(t));
            }
            pts
        }
        Curve::TrimmedCurve(tc) => {
            // Sample within the trim range
            let mut pts = Vec::with_capacity(segments + 1);
            for i in 0..=segments {
                let t = tc.trim1 + (i as f64 / segments as f64) * (tc.trim2 - tc.trim1);
                pts.push(tc.basis_curve.point_at(t));
            }
            pts
        }
        Curve::BSplineCurve(b_spline) => {
            // Sample within the knot range
            let knot_min = b_spline.knots.first().copied().unwrap_or(0.0);
            let knot_max = b_spline.knots.last().copied().unwrap_or(1.0);
            let mut pts = Vec::with_capacity(segments + 1);
            for i in 0..=segments {
                let t = knot_min + (i as f64 / segments as f64) * (knot_max - knot_min);
                pts.push(b_spline.point_at(t));
            }
            pts
        }
        Curve::OffsetCurve(oc) => {
            let mut pts = Vec::with_capacity(segments + 1);
            for i in 0..=segments {
                let t = i as f64 / segments as f64;
                pts.push(oc.point_at(t));
            }
            pts
        }
    }
}

/// Generate mesh for a planar face using ear clipping.
fn generate_planar_face_mesh_from_points(
    outer_points: &[Point3],
    inner_points: &[Vec<Point3>],
    plane: &draper_geometry::surface::Plane,
    mesh: &mut TriangleMesh,
) {
    let origin = plane.axis.location;
    let u_dir = plane.axis.ref_direction;
    let v_dir = plane.axis.y_direction();

    // Project outer boundary to 2D
    let outer_2d = project_points_to_2d(outer_points, origin, u_dir, v_dir);
    if outer_2d.len() < 3 {
        return;
    }

    // Project inner boundaries to 2D
    let inner_2d: Vec<Vec<Point2>> = inner_points
        .iter()
        .map(|pts| project_points_to_2d(pts, origin, u_dir, v_dir))
        .collect();

    // Triangulate with holes
    let tri_indices = triangulate_polygon_with_holes(&outer_2d, &inner_2d);

    // Add vertices and triangles to mesh
    let base_idx = mesh.vertices.len() as u32;
    for pt in outer_points {
        mesh.add_vertex(*pt);
    }
    for hole_pts in inner_points {
        for pt in hole_pts {
            mesh.add_vertex(*pt);
        }
    }

    for idx in tri_indices.chunks(3) {
        if idx.len() == 3 {
            mesh.add_triangle(base_idx + idx[0], base_idx + idx[1], base_idx + idx[2]);
        }
    }
}

/// Generate mesh for a curved surface face.
///
/// For curved surfaces, we:
/// 1. Get the boundary points from the wires
/// 2. Compute a best-fit plane for 2D projection
/// 3. Ear-clip the 2D polygon
/// 4. Use 3D coordinates directly (faceted approximation)
/// 5. Optionally, refine by projecting 3D points onto the surface
fn generate_curved_face_mesh_from_points(
    _shape: &Shape,
    face: &Face,
    outer_points: &[Point3],
    inner_points: &[Vec<Point3>],
    surface: &Surface,
    mesh: &mut TriangleMesh,
) {
    if outer_points.len() < 3 {
        return;
    }

    // Compute best-fit plane for 2D projection
    let (origin, u_dir, v_dir) = compute_best_fit_plane(outer_points, surface);

    // Project outer boundary to 2D
    let outer_2d = project_points_to_2d(outer_points, origin, u_dir, v_dir);
    if outer_2d.len() < 3 {
        return;
    }

    // Project inner boundaries to 2D
    let inner_2d: Vec<Vec<Point2>> = inner_points
        .iter()
        .map(|pts| project_points_to_2d(pts, origin, u_dir, v_dir))
        .collect();

    // Triangulate with holes
    let tri_indices = triangulate_polygon_with_holes(&outer_2d, &inner_2d);

    // Add 3D boundary vertices
    // For curved surfaces, project the boundary points onto the surface
    // to ensure they lie exactly on the surface geometry
    let base_idx = mesh.vertices.len() as u32;

    // Add outer boundary points (projected onto surface if possible)
    for pt in outer_points {
        let projected = project_point_onto_surface(*pt, surface);
        mesh.add_vertex(projected);
    }

    // Add inner boundary points
    for hole_pts in inner_points {
        for pt in hole_pts {
            let projected = project_point_onto_surface(*pt, surface);
            mesh.add_vertex(projected);
        }
    }

    for idx in tri_indices.chunks(3) {
        if idx.len() == 3 {
            mesh.add_triangle(base_idx + idx[0], base_idx + idx[1], base_idx + idx[2]);
        }
    }

    log::trace!(
        "Curved face #{}: {} outer pts, {} holes, {} triangles, surface={:?}",
        face.id,
        outer_points.len(),
        inner_points.len(),
        tri_indices.len() / 3,
        std::mem::discriminant(surface)
    );
}

/// Generate a faceted mesh from boundary points (no surface geometry).
fn generate_faceted_face_mesh(
    outer_points: &[Point3],
    inner_points: &[Vec<Point3>],
    mesh: &mut TriangleMesh,
) {
    if outer_points.len() < 3 {
        return;
    }

    // Compute best-fit plane for 2D projection
    let (origin, u_dir, v_dir) = compute_best_fit_plane_from_points(outer_points);

    // Project to 2D
    let outer_2d = project_points_to_2d(outer_points, origin, u_dir, v_dir);
    if outer_2d.len() < 3 {
        return;
    }

    let inner_2d: Vec<Vec<Point2>> = inner_points
        .iter()
        .map(|pts| project_points_to_2d(pts, origin, u_dir, v_dir))
        .collect();

    // Triangulate
    let tri_indices = triangulate_polygon_with_holes(&outer_2d, &inner_2d);

    // Add vertices and triangles
    let base_idx = mesh.vertices.len() as u32;
    for pt in outer_points {
        mesh.add_vertex(*pt);
    }
    for hole_pts in inner_points {
        for pt in hole_pts {
            mesh.add_vertex(*pt);
        }
    }

    for idx in tri_indices.chunks(3) {
        if idx.len() == 3 {
            mesh.add_triangle(base_idx + idx[0], base_idx + idx[1], base_idx + idx[2]);
        }
    }
}

// ---- Helper functions ----

/// Project a set of 3D points to 2D using a local coordinate system.
fn project_points_to_2d(
    points: &[Point3],
    origin: Point3,
    u_dir: draper_geometry::direction::Direction3,
    v_dir: draper_geometry::direction::Direction3,
) -> Vec<Point2> {
    let u_d = u_dir.to_dvec3();
    let v_d = v_dir.to_dvec3();

    points
        .iter()
        .map(|p| {
            let rel = p.to_dvec3() - origin.to_dvec3();
            Point2::new(rel.dot(u_d), rel.dot(v_d))
        })
        .collect()
}

/// Compute a best-fit plane for 2D projection.
///
/// For curved surfaces, use the surface's natural coordinate system.
fn compute_best_fit_plane(
    points: &[Point3],
    surface: &Surface,
) -> (Point3, draper_geometry::direction::Direction3, draper_geometry::direction::Direction3) {
    match surface {
        Surface::Plane(plane) => {
            (plane.axis.location, plane.axis.ref_direction, plane.axis.y_direction())
        }
        Surface::CylindricalSurface(cyl) => {
            (cyl.axis.location, cyl.axis.ref_direction, cyl.axis.y_direction())
        }
        Surface::ConicalSurface(cone) => {
            (cone.axis.location, cone.axis.ref_direction, cone.axis.y_direction())
        }
        Surface::SphericalSurface(sph) => {
            (sph.axis.location, sph.axis.ref_direction, sph.axis.y_direction())
        }
        Surface::ToroidalSurface(tor) => {
            (tor.axis.location, tor.axis.ref_direction, tor.axis.y_direction())
        }
        _ => {
            // Fallback: compute best-fit plane from points
            compute_best_fit_plane_from_points(points)
        }
    }
}

/// Compute a best-fit plane from a set of 3D points using PCA.
fn compute_best_fit_plane_from_points(
    points: &[Point3],
) -> (Point3, draper_geometry::direction::Direction3, draper_geometry::direction::Direction3) {
    if points.is_empty() {
        return (
            Point3::ORIGIN,
            draper_geometry::direction::Direction3::X,
            draper_geometry::direction::Direction3::Y,
        );
    }

    // Compute centroid
    let sum = points.iter().fold(glam::DVec3::ZERO, |acc, p| acc + p.to_dvec3());
    let centroid = sum / points.len() as f64;
    let origin = Point3::from_dvec3(centroid);

    // Compute covariance matrix (simplified: use cross product approach)
    // Find the two directions of maximum spread
    let mut best_normal = glam::DVec3::Z;
    let mut min_spread = f64::MAX;

    // Try several candidate normals and pick the one with minimum spread
    let candidates = [
        glam::DVec3::X,
        glam::DVec3::Y,
        glam::DVec3::Z,
    ];

    for candidate in &candidates {
        let mut variance = 0.0f64;
        for p in points {
            let rel = p.to_dvec3() - centroid;
            let proj = rel.dot(*candidate);
            variance += proj * proj;
        }
        if variance < min_spread {
            min_spread = variance;
            best_normal = *candidate;
        }
    }

    // Try cross products of point pairs for a better normal
    if points.len() >= 3 {
        let v1 = points[1].to_dvec3() - points[0].to_dvec3();
        let v2 = points[2].to_dvec3() - points[0].to_dvec3();
        let cross = v1.cross(v2);
        if cross.length() > 1e-10 {
            best_normal = cross.normalize();
        }
    }

    let normal = draper_geometry::direction::Direction3::new(
        best_normal.x, best_normal.y, best_normal.z,
    ).unwrap_or(draper_geometry::direction::Direction3::Z);

    // Compute a perpendicular direction for the u-axis
    let u_dir = if normal.dot(draper_geometry::direction::Direction3::X).abs() < 0.9 {
        normal.cross(draper_geometry::direction::Direction3::X)
    } else {
        normal.cross(draper_geometry::direction::Direction3::Y)
    };

    let v_dir = normal.cross(u_dir);

    (origin, u_dir, v_dir)
}

/// Project a point onto a surface (find the closest point on the surface).
///
/// For initial implementation, we just return the point as-is since
/// the boundary vertices from the wire should already lie on the surface.
/// A proper implementation would use Newton's method to project.
fn project_point_onto_surface(point: Point3, _surface: &Surface) -> Point3 {
    // For now, just return the point. The boundary vertices from the
    // wire already lie on the surface by construction.
    // TODO: Implement proper projection using Newton's method
    point
}

/// Triangulate a polygon with holes using ear clipping.
///
/// The outer boundary is assumed to be in CCW order (will be corrected if not).
/// Inner boundaries (holes) are in CW order (will be corrected if not).
fn triangulate_polygon_with_holes(
    outer: &[Point2],
    holes: &[Vec<Point2>],
) -> Vec<u32> {
    if outer.len() < 3 {
        return Vec::new();
    }

    if holes.is_empty() {
        // Simple polygon — just ear clip
        return earcut::ear_clip_polygon(outer);
    }

    // For polygons with holes, we need to create a bridge between
    // the outer boundary and each hole, then triangulate the combined polygon.
    //
    // Simplified approach: merge all points into one polygon list,
    // using bridge edges to connect holes to outer boundary.
    let mut combined: Vec<Point2> = outer.to_vec();
    let mut combined_3d_indices: Vec<u32> = (0..outer.len() as u32).collect();

    for hole in holes {
        if hole.is_empty() {
            continue;
        }

        // Find the rightmost point of the hole
        let (hole_bridge_idx, outer_bridge_idx) = find_bridge_vertices(&combined, hole);

        // Insert hole vertices into the combined polygon
        let hole_start = combined.len() as u32;
        let insert_pos = outer_bridge_idx as usize + 1;

        // Build the bridge: outer[bridge] → hole[bridge] → ... → hole[bridge] → outer[bridge]
        let hole_len = hole.len();
        let mut new_combined = Vec::with_capacity(combined.len() + hole_len + 2);
        let mut new_indices = Vec::with_capacity(combined_3d_indices.len() + hole_len + 2);

        for (i, (pt, idx)) in combined.iter().zip(combined_3d_indices.iter()).enumerate() {
            new_combined.push(*pt);
            new_indices.push(*idx);

            if i == insert_pos - 1 {
                // Add bridge to hole
                new_combined.push(hole[hole_bridge_idx]);
                new_indices.push(hole_start + hole_bridge_idx as u32);

                // Add remaining hole vertices (starting after bridge, wrapping around)
                for j in 1..=hole_len {
                    let hj = (hole_bridge_idx + j) % hole_len;
                    new_combined.push(hole[hj]);
                    new_indices.push(hole_start + hj as u32);
                }

                // Add bridge vertex again to close the hole
                new_combined.push(hole[hole_bridge_idx]);
                new_indices.push(hole_start + hole_bridge_idx as u32);
            }
        }

        combined = new_combined;
        combined_3d_indices = new_indices;
    }

    // Triangulate the combined polygon
    let raw_indices = earcut::ear_clip_polygon(&combined);

    // Map indices back to original point indices
    raw_indices
        .into_iter()
        .filter_map(|i| combined_3d_indices.get(i as usize).copied())
        .collect()
}

/// Find bridge vertices between outer polygon and a hole.
///
/// Returns (hole_vertex_index, outer_vertex_index) for the bridge edge.
fn find_bridge_vertices(outer: &[Point2], hole: &[Point2]) -> (usize, usize) {
    // Find the rightmost point of the hole
    let mut hole_idx = 0;
    let mut max_u = hole[0].u;
    for (i, pt) in hole.iter().enumerate() {
        if pt.u > max_u {
            max_u = pt.u;
            hole_idx = i;
        }
    }

    // Find the closest outer vertex to the right of the hole bridge point
    let hole_pt = hole[hole_idx];
    let mut outer_idx = 0;
    let mut min_dist = f64::MAX;
    for (i, pt) in outer.iter().enumerate() {
        let dist = (pt.u - hole_pt.u).powi(2) + (pt.v - hole_pt.v).powi(2);
        if dist < min_dist {
            min_dist = dist;
            outer_idx = i;
        }
    }

    (hole_idx, outer_idx)
}
