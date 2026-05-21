//! Mesh generation from B-rep topology.
//!
//! Converts topological faces into triangle meshes suitable for rendering.
//! The approach is boundary-driven: we extract the boundary vertices from
//! each face's wire loops, then triangulate using Constrained Delaunay
//! Triangulation (CDT) via the `spade` crate after projecting to 2D.
//!
//! For curved surfaces, we additionally sample the surface between boundary
//! vertices to capture curvature and project points onto the surface.

use crate::delaunay;
use crate::earcut;
use crate::triangulate::TriangleMesh;
use draper_geometry::curve::Curve;
use draper_geometry::point::{Point2, Point3};
use draper_geometry::surface::Surface;
use draper_topology::entity::*;
use draper_topology::shape::Shape;

/// Default number of segments for discretizing circular edges.
const CIRCLE_SEGMENTS: usize = 48;

/// Default number of interior sample points per direction for curved surfaces.
const SURFACE_SAMPLES: usize = 8;

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

        // Get the first and second vertex points
        let first_pt = match shape.get(first_vertex) {
            Some(TopoShape::Vertex(v)) => v.point,
            _ => continue,
        };
        let second_pt = match shape.get(second_vertex) {
            Some(TopoShape::Vertex(v)) => v.point,
            _ => continue,
        };

        // Add the first vertex point (skip if duplicate of the last point)
        let should_add_first = points.last().map_or(true, |last: &Point3| {
            (last.x - first_pt.x).abs() > 1e-10
                || (last.y - first_pt.y).abs() > 1e-10
                || (last.z - first_pt.z).abs() > 1e-10
        });

        if should_add_first {
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

        // Add the second vertex point.
        // For the last edge of a closed wire, skip it if it equals the first point.
        let is_last = i == wire.edges.len() - 1;
        if is_last && wire.closed && !points.is_empty() {
            let first = points[0];
            if (first.x - second_pt.x).abs() < 1e-10
                && (first.y - second_pt.y).abs() < 1e-10
                && (first.z - second_pt.z).abs() < 1e-10
            {
                continue; // Closing point matches the start — skip
            }
        }

        // Check for duplicate with the last point before adding
        let should_add_second = points.last().map_or(true, |last: &Point3| {
            (last.x - second_pt.x).abs() > 1e-10
                || (last.y - second_pt.y).abs() > 1e-10
                || (last.z - second_pt.z).abs() > 1e-10
        });

        if should_add_second {
            points.push(second_pt);
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
        "Wire #{}: {} edges -> {} boundary points",
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

/// Generate mesh for a planar face using CDT.
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

    // Triangulate with CDT (fall back to ear-clipping if CDT fails)
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
/// 3. Add interior sample points on the surface for better curvature capture
/// 4. CDT the 2D polygon with interior points
/// 5. Project all points onto the surface for correct 3D positions
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

    // Add interior sample points on the surface for better curvature capture
    let (interior_3d, interior_2d) = generate_interior_surface_points(
        surface, &outer_2d, &inner_2d, SURFACE_SAMPLES,
    );

    // Combine boundary + interior points for triangulation
    let mut all_2d = outer_2d.clone();
    let mut all_3d: Vec<Point3> = outer_points.to_vec();

    // Add inner boundary points
    let _inner_offset_start = all_2d.len();
    for hole_pts in inner_points {
        let hole_2d = project_points_to_2d(hole_pts, origin, u_dir, v_dir);
        all_2d.extend(hole_2d);
        all_3d.extend(hole_pts.iter().map(|pt| project_point_onto_surface(*pt, surface)));
    }

    // Add interior surface sample points
    let _interior_offset_start = all_2d.len();
    all_2d.extend(interior_2d);
    all_3d.extend(interior_3d.iter().map(|pt| *pt));

    // Triangulate all points together
    // The boundary edges are constraints, interior points are free
    let tri_indices = delaunay::triangulate_face_boundary(&all_2d, &[]);

    // If CDT produced no results, fall back to ear-clipping with just the boundary
    if tri_indices.is_empty() {
        log::debug!("Curved face #{}: CDT produced no results, falling back to ear-clipping", face.id);
        let boundary_indices = triangulate_polygon_with_holes(&outer_2d, &inner_2d);
        let base_idx = mesh.vertices.len() as u32;

        // Add only boundary vertices
        for pt in outer_points {
            let projected = project_point_onto_surface(*pt, surface);
            mesh.add_vertex(projected);
        }
        for hole_pts in inner_points {
            for pt in hole_pts {
                let projected = project_point_onto_surface(*pt, surface);
                mesh.add_vertex(projected);
            }
        }

        for idx in boundary_indices.chunks(3) {
            if idx.len() == 3 {
                mesh.add_triangle(base_idx + idx[0], base_idx + idx[1], base_idx + idx[2]);
            }
        }
        return;
    }

    // Filter triangles: keep only those inside the polygon boundary
    let filtered_indices = filter_interior_triangles(&tri_indices, &all_2d, &outer_2d, &inner_2d);

    // Add 3D vertices (projected onto surface for curved surfaces)
    let base_idx = mesh.vertices.len() as u32;
    for pt in &all_3d {
        let projected = project_point_onto_surface(*pt, surface);
        mesh.add_vertex(projected);
    }

    for idx in filtered_indices.chunks(3) {
        if idx.len() == 3 {
            mesh.add_triangle(base_idx + idx[0], base_idx + idx[1], base_idx + idx[2]);
        }
    }

    log::trace!(
        "Curved face #{}: {} outer pts, {} holes, {} interior pts, {} triangles, surface={:?}",
        face.id,
        outer_points.len(),
        inner_points.len(),
        interior_3d.len(),
        filtered_indices.len() / 3,
        std::mem::discriminant(surface)
    );
}

/// Generate interior sample points on a surface within the face boundary.
///
/// Samples a grid of points in 2D parameter space, then keeps only those
/// that fall inside the outer boundary and outside holes.
fn generate_interior_surface_points(
    surface: &Surface,
    outer_2d: &[Point2],
    inner_2d: &[Vec<Point2>],
    samples: usize,
) -> (Vec<Point3>, Vec<Point2>) {
    if outer_2d.len() < 3 || samples == 0 {
        return (Vec::new(), Vec::new());
    }

    // Find the bounding box of the 2D points
    let mut min_u = f64::MAX;
    let mut min_v = f64::MAX;
    let mut max_u = f64::MIN;
    let mut max_v = f64::MIN;

    for pt in outer_2d {
        min_u = min_u.min(pt.u);
        min_v = min_v.min(pt.v);
        max_u = max_u.max(pt.u);
        max_v = max_v.max(pt.v);
    }

    let du = max_u - min_u;
    let dv = max_v - min_v;

    if du < 1e-10 || dv < 1e-10 {
        return (Vec::new(), Vec::new());
    }

    let mut points_3d = Vec::new();
    let mut points_2d = Vec::new();

    // Sample a grid and keep points inside the boundary
    for i in 1..samples {
        for j in 1..samples {
            let u = min_u + du * (i as f64 / samples as f64);
            let v = min_v + dv * (j as f64 / samples as f64);
            let pt_2d = Point2::new(u, v);

            // Check if point is inside outer boundary and outside holes
            if !point_in_polygon_2d(&pt_2d, outer_2d) {
                continue;
            }

            let mut in_hole = false;
            for hole in inner_2d {
                if point_in_polygon_2d(&pt_2d, hole) {
                    in_hole = true;
                    break;
                }
            }
            if in_hole {
                continue;
            }

            // Convert 2D back to 3D using the surface's parameterization
            // We need to map from the 2D projection back to surface UV parameters
            let pt_3d = project_2d_to_surface(pt_2d, surface);

            points_3d.push(pt_3d);
            points_2d.push(pt_2d);
        }
    }

    (points_3d, points_2d)
}

/// Map a 2D projected point back onto a surface.
///
/// This converts the 2D point (from the best-fit plane projection) into
/// a 3D point on the surface. The mapping depends on the surface type.
fn project_2d_to_surface(pt_2d: Point2, surface: &Surface) -> Point3 {
    // For each surface type, interpret (u, v) as surface parameters
    match surface {
        Surface::Plane(_) => {
            // For a plane, the 2D projection IS the parameterization
            // We need to reconstruct from the projection, but we don't have
            // the coordinate system here. Use the surface's own parameterization.
            // The pt_2d values are distances in the local plane system.
            surface.point_at(pt_2d.u, pt_2d.v)
        }
        Surface::CylindricalSurface(_) => {
            // Map u to angle (0..2PI), v to height
            let u = pt_2d.u;
            let v = pt_2d.v;
            surface.point_at(u, v)
        }
        Surface::ConicalSurface(_) => {
            let u = pt_2d.u;
            let v = pt_2d.v;
            surface.point_at(u, v)
        }
        Surface::SphericalSurface(_) => {
            let u = pt_2d.u;
            let v = pt_2d.v;
            surface.point_at(u, v)
        }
        Surface::ToroidalSurface(_) => {
            let u = pt_2d.u;
            let v = pt_2d.v;
            surface.point_at(u, v)
        }
        _ => {
            // For unsupported surface types, just evaluate at the 2D parameters
            surface.point_at(pt_2d.u, pt_2d.v)
        }
    }
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
        Surface::SurfaceOfRevolution(rev) => {
            (rev.axis.location, rev.axis.ref_direction, rev.axis.y_direction())
        }
        Surface::SurfaceOfLinearExtrusion(_ext) => {
            // For extrusion surfaces, we don't have a direct axis2 placement.
            // Use the generatrix curve's coordinate system or fall back to PCA.
            compute_best_fit_plane_from_points(points)
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

/// Project a point onto a surface using Newton's method.
///
/// Given a 3D point that is approximately on a surface, find the nearest
/// point on the surface by iterating in UV parameter space.
///
/// For boundary vertices from B-rep wires, these should already lie on or
/// very near the surface by construction, so a few iterations suffice.
fn project_point_onto_surface(point: Point3, surface: &Surface) -> Point3 {
    match surface {
        // For planes, the projection is exact
        Surface::Plane(plane) => {
            let normal = plane.axis.axis.to_dvec3();
            let origin = plane.axis.location.to_dvec3();
            let rel = point.to_dvec3() - origin;
            let dist = rel.dot(normal);
            Point3::from_dvec3(point.to_dvec3() - normal * dist)
        }
        // For curved surfaces, use Newton iteration
        _ => project_onto_curved_surface(point, surface),
    }
}

/// Project a point onto a curved surface using Newton-Raphson iteration in UV space.
///
/// The algorithm:
/// 1. Start with an initial UV guess
/// 2. Compute S(u,v) - point (the residual)
/// 3. Compute the Jacobian (partial derivatives dS/du, dS/dv)
/// 4. Solve for (du, dv) to minimize the residual
/// 5. Update u, v and repeat
fn project_onto_curved_surface(point: Point3, surface: &Surface) -> Point3 {
    let target = point.to_dvec3();
    let eps = 1e-6;
    let max_iter = 10;

    // Initial UV guess: use (0, 0) or try to find a better starting point
    // by sampling the surface at a few locations
    let (mut u, mut v) = find_initial_uv(point, surface);

    for _ in 0..max_iter {
        let s = surface.point_at(u, v).to_dvec3();
        let residual = s - target;

        // Check convergence
        if residual.length() < eps {
            break;
        }

        // Compute partial derivatives via finite differences
        let h = 1e-5;
        let su = (surface.point_at(u + h, v).to_dvec3() - surface.point_at(u - h, v).to_dvec3()) / (2.0 * h);
        let sv = (surface.point_at(u, v + h).to_dvec3() - surface.point_at(u, v - h).to_dvec3()) / (2.0 * h);

        // Solve the 2x2 system: [su.su  su.sv] [du]   [-residual.su]
        //                        [su.sv  sv.sv] [dv] = [-residual.sv]
        let a = su.dot(su);
        let b = su.dot(sv);
        let c = sv.dot(sv);
        let rhs0 = -residual.dot(su);
        let rhs1 = -residual.dot(sv);

        let det = a * c - b * b;
        if det.abs() < 1e-20 {
            // Degenerate — can't solve, return current approximation
            break;
        }

        let du = (c * rhs0 - b * rhs1) / det;
        let dv = (a * rhs1 - b * rhs0) / det;

        // Damped step to prevent divergence
        let step_len = (du * du + dv * dv).sqrt();
        let max_step = 0.5; // Limit step size
        let (du, dv) = if step_len > max_step {
            let scale = max_step / step_len;
            (du * scale, dv * scale)
        } else {
            (du, dv)
        };

        u += du;
        v += dv;
    }

    let projected = surface.point_at(u, v);

    // If the projected point is farther from the original than the original is from
    // the surface's bounding box, the projection may have diverged — return original
    let dist = point.distance_to(projected);
    if dist > 100.0 {
        // Likely diverged, return the original point
        log::debug!(
            "Surface projection diverged (dist={:.3}), returning original point",
            dist
        );
        point
    } else {
        projected
    }
}

/// Find initial UV parameters for a point by sampling the surface.
///
/// Tries a coarse grid and picks the closest surface point.
fn find_initial_uv(point: Point3, surface: &Surface) -> (f64, f64) {
    let target = point.to_dvec3();
    let mut best_u = 0.0;
    let mut best_v = 0.0;
    let mut best_dist = f64::MAX;

    // Sample a coarse grid of UV values
    let u_range = get_surface_u_range(surface);
    let v_range = get_surface_v_range(surface);

    let n_samples = 10;
    for i in 0..=n_samples {
        for j in 0..=n_samples {
            let u = u_range.0 + (u_range.1 - u_range.0) * (i as f64 / n_samples as f64);
            let v = v_range.0 + (v_range.1 - v_range.0) * (j as f64 / n_samples as f64);
            let s = surface.point_at(u, v).to_dvec3();
            let dist = (s - target).length_squared();
            if dist < best_dist {
                best_dist = dist;
                best_u = u;
                best_v = v;
            }
        }
    }

    (best_u, best_v)
}

/// Get the parameter range for the U direction of a surface.
fn get_surface_u_range(surface: &Surface) -> (f64, f64) {
    match surface {
        Surface::Plane(_) => (-1000.0, 1000.0),
        Surface::CylindricalSurface(_) => (0.0, 2.0 * std::f64::consts::PI),
        Surface::ConicalSurface(_) => (0.0, 2.0 * std::f64::consts::PI),
        Surface::SphericalSurface(_) => (0.0, 2.0 * std::f64::consts::PI),
        Surface::ToroidalSurface(_) => (0.0, 2.0 * std::f64::consts::PI),
        Surface::SurfaceOfRevolution(_) => (0.0, 2.0 * std::f64::consts::PI),
        Surface::SurfaceOfLinearExtrusion(_) => (-100.0, 100.0),
        Surface::BSplineSurface(bs) => {
            let min = bs.u_knots.first().copied().unwrap_or(0.0);
            let max = bs.u_knots.last().copied().unwrap_or(1.0);
            (min, max)
        }
        Surface::OffsetSurface(os) => get_surface_u_range(&os.basis_surface),
        Surface::RectangularTrimmedSurface(rs) => (rs.u1, rs.u2),
    }
}

/// Get the parameter range for the V direction of a surface.
fn get_surface_v_range(surface: &Surface) -> (f64, f64) {
    match surface {
        Surface::Plane(_) => (-1000.0, 1000.0),
        Surface::CylindricalSurface(_) => (-1000.0, 1000.0),
        Surface::ConicalSurface(_) => (-1000.0, 1000.0),
        Surface::SphericalSurface(_) => (-std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2),
        Surface::ToroidalSurface(_) => (0.0, 2.0 * std::f64::consts::PI),
        Surface::SurfaceOfRevolution(_) => (-100.0, 100.0),
        Surface::SurfaceOfLinearExtrusion(_) => (-1000.0, 1000.0),
        Surface::BSplineSurface(bs) => {
            let min = bs.v_knots.first().copied().unwrap_or(0.0);
            let max = bs.v_knots.last().copied().unwrap_or(1.0);
            (min, max)
        }
        Surface::OffsetSurface(os) => get_surface_v_range(&os.basis_surface),
        Surface::RectangularTrimmedSurface(rs) => (rs.v1, rs.v2),
    }
}

/// Point-in-polygon test for 2D points.
fn point_in_polygon_2d(point: &Point2, polygon: &[Point2]) -> bool {
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

/// Filter triangles to keep only those inside the outer polygon and outside holes.
fn filter_interior_triangles(
    indices: &[u32],
    all_2d: &[Point2],
    outer_2d: &[Point2],
    inner_2d: &[Vec<Point2>],
) -> Vec<u32> {
    let mut result = Vec::with_capacity(indices.len());

    for tri in indices.chunks(3) {
        if tri.len() < 3 {
            continue;
        }

        let ai = tri[0] as usize;
        let bi = tri[1] as usize;
        let ci = tri[2] as usize;

        if ai >= all_2d.len() || bi >= all_2d.len() || ci >= all_2d.len() {
            continue;
        }

        // Compute centroid
        let centroid = Point2::new(
            (all_2d[ai].u + all_2d[bi].u + all_2d[ci].u) / 3.0,
            (all_2d[ai].v + all_2d[bi].v + all_2d[ci].v) / 3.0,
        );

        // Must be inside outer boundary
        if !point_in_polygon_2d(&centroid, outer_2d) {
            continue;
        }

        // Must be outside all holes
        let mut inside_hole = false;
        for hole in inner_2d {
            if point_in_polygon_2d(&centroid, hole) {
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

/// Triangulate a polygon with holes.
///
/// Uses CDT (spade) as the primary method, with ear-clipping as fallback.
fn triangulate_polygon_with_holes(
    outer: &[Point2],
    holes: &[Vec<Point2>],
) -> Vec<u32> {
    if outer.len() < 3 {
        return Vec::new();
    }

    // Try CDT first
    let cdt_result = delaunay::cdt_polygon_with_holes(outer, holes);
    if !cdt_result.is_empty() {
        return cdt_result;
    }

    // Fallback to ear-clipping
    log::debug!("CDT triangulation produced no results, falling back to ear-clipping");

    if holes.is_empty() {
        return earcut::ear_clip_polygon(outer);
    }

    // For polygons with holes, bridge holes into the outer polygon
    // then ear-clip the combined polygon
    let mut combined: Vec<Point2> = outer.to_vec();
    let mut combined_3d_indices: Vec<u32> = (0..outer.len() as u32).collect();

    for hole in holes {
        if hole.is_empty() {
            continue;
        }

        let (hole_bridge_idx, outer_bridge_idx) = find_bridge_vertices(&combined, hole);

        let hole_start = combined.len() as u32;
        let insert_pos = outer_bridge_idx as usize + 1;

        let hole_len = hole.len();
        let mut new_combined = Vec::with_capacity(combined.len() + hole_len + 2);
        let mut new_indices = Vec::with_capacity(combined_3d_indices.len() + hole_len + 2);

        for (i, (pt, idx)) in combined.iter().zip(combined_3d_indices.iter()).enumerate() {
            new_combined.push(*pt);
            new_indices.push(*idx);

            if i == insert_pos - 1 {
                new_combined.push(hole[hole_bridge_idx]);
                new_indices.push(hole_start + hole_bridge_idx as u32);

                for j in 1..=hole_len {
                    let hj = (hole_bridge_idx + j) % hole_len;
                    new_combined.push(hole[hj]);
                    new_indices.push(hole_start + hj as u32);
                }

                new_combined.push(hole[hole_bridge_idx]);
                new_indices.push(hole_start + hole_bridge_idx as u32);
            }
        }

        combined = new_combined;
        combined_3d_indices = new_indices;
    }

    let raw_indices = earcut::ear_clip_polygon(&combined);

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
