// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Parametric domain representation for trimmed surface triangulation.
//!
//! A ParametricDomain represents the 2D region in UV-parameter space
//! that defines the valid area of a trimmed surface. It consists of:
//! - An outer boundary (the trimming loop)
//! - Optional inner boundaries (holes)
//!
//! The domain is used to construct a constrained Delaunay triangulation
//! that respects the trimming boundaries exactly.

use draper_geometry::{Point2d, Point3d, Surface};
use crate::mesh::TriangleMesh;
use std::f64::consts::PI;

/// A closed polygon in UV parameter space.
pub type UVPolygon = Vec<Point2d>;

/// The parametric domain of a trimmed surface face.
///
/// Defines the valid 2D region in UV space that should be triangulated.
/// The outer boundary defines the exterior contour, and inner boundaries
/// define holes that should be excluded from the triangulation.
#[derive(Clone, Debug)]
pub struct ParametricDomain {
    /// The outer boundary of the domain (counter-clockwise in UV space).
    pub outer_boundary: UVPolygon,
    /// Inner boundaries (holes) — each is a clockwise polygon in UV space.
    pub holes: Vec<UVPolygon>,
    /// The UV range of the surface: (u_min, u_max).
    pub u_range: (f64, f64),
    /// The V range of the surface: (v_min, v_max).
    pub v_range: (f64, f64),
}

impl ParametricDomain {
    /// Create a new parametric domain from an outer boundary.
    pub fn new(outer_boundary: UVPolygon, u_range: (f64, f64), v_range: (f64, f64)) -> Self {
        Self {
            outer_boundary,
            holes: Vec::new(),
            u_range,
            v_range,
        }
    }

    /// Add a hole (inner boundary) to the domain.
    pub fn with_hole(mut self, hole: UVPolygon) -> Self {
        self.holes.push(hole);
        self
    }

    /// Compute the bounding box of the domain.
    pub fn bounding_box(&self) -> (f64, f64, f64, f64) {
        let mut u_min = f64::MAX;
        let mut u_max = f64::MIN;
        let mut v_min = f64::MAX;
        let mut v_max = f64::MIN;

        for p in &self.outer_boundary {
            u_min = u_min.min(p.u);
            u_max = u_max.max(p.u);
            v_min = v_min.min(p.v);
            v_max = v_max.max(p.v);
        }
        for hole in &self.holes {
            for p in hole {
                u_min = u_min.min(p.u);
                u_max = u_max.max(p.u);
                v_min = v_min.min(p.v);
                v_max = v_max.max(p.v);
            }
        }

        (u_min, u_max, v_min, v_max)
    }

    /// Check if a UV point is inside the domain (inside outer boundary, outside all holes).
    pub fn contains(&self, point: &Point2d) -> bool {
        if !point_in_polygon(point, &self.outer_boundary) {
            return false;
        }
        for hole in &self.holes {
            if point_in_polygon(point, hole) {
                return false;
            }
        }
        true
    }
}

/// Test if a 2D point is inside a closed polygon using ray casting.
fn point_in_polygon(point: &Point2d, polygon: &[Point2d]) -> bool {
    let n = polygon.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    let px = point.u;
    let py = point.v;
    let mut j = n - 1;
    for i in 0..n {
        let xi = polygon[i].u;
        let yi = polygon[i].v;
        let xj = polygon[j].u;
        let yj = polygon[j].v;
        if ((yi > py) != (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi) {
            inside = !inside;
        }
        j = i;
    }
    inside
}

// ============================================================
// CDT triangulation using spade
// ============================================================

/// Wrapper point type for spade's Delaunay triangulation.
#[derive(Clone, Debug)]
struct SpadePoint {
    x: f64,
    y: f64,
    index: usize,
}

impl spade::HasPosition for SpadePoint {
    type Scalar = f64;

    fn position(&self) -> spade::Point2<Self::Scalar> {
        spade::Point2::new(self.x, self.y)
    }
}

/// Triangulate a parametric domain using Constrained Delaunay Triangulation.
///
/// This creates a mesh in UV space that respects the boundary and holes,
/// then maps the vertices to 3D using the surface evaluation.
///
/// The algorithm:
/// 1. Collect all boundary edges as constraints
/// 2. Add interior vertices (from adaptive sampling grid)
/// 3. Build a constrained Delaunay triangulation using `spade`
/// 4. Remove triangles outside the domain or inside holes
/// 5. Map UV vertices to 3D
pub fn triangulate_cdt(
    domain: &ParametricDomain,
    surface: &Surface,
    forward: bool,
    interior_uv_points: &[Point2d],
) -> TriangleMesh {
    use spade::ConstrainedDelaunayTriangulation;
    use spade::handles::FixedVertexHandle;
    use spade::Triangulation as _;

    // Collect all constraint edges (outer boundary + holes)
    let mut all_uv_points: Vec<Point2d> = Vec::new();
    let mut constraint_edges: Vec<(usize, usize)> = Vec::new();

    // Outer boundary
    let outer_start = all_uv_points.len();
    for p in &domain.outer_boundary {
        all_uv_points.push(*p);
    }
    for i in 0..domain.outer_boundary.len() {
        let next = (i + 1) % domain.outer_boundary.len();
        constraint_edges.push((outer_start + i, outer_start + next));
    }

    // Holes
    for hole in &domain.holes {
        let hole_start = all_uv_points.len();
        for p in hole {
            all_uv_points.push(*p);
        }
        for i in 0..hole.len() {
            let next = (i + 1) % hole.len();
            constraint_edges.push((hole_start + i, hole_start + next));
        }
    }

    // Interior points
    let _interior_start = all_uv_points.len();
    for p in interior_uv_points {
        all_uv_points.push(*p);
    }

    // Build the constrained Delaunay triangulation
    let mut cdt: ConstrainedDelaunayTriangulation<SpadePoint> =
        ConstrainedDelaunayTriangulation::new();

    // Insert all points
    let mut vertex_handles: Vec<FixedVertexHandle> = Vec::with_capacity(all_uv_points.len());
    for (idx, p) in all_uv_points.iter().enumerate() {
        let spade_pt = SpadePoint {
            x: p.u,
            y: p.v,
            index: idx,
        };
        match cdt.insert(spade_pt) {
            Ok(handle) => vertex_handles.push(handle),
            Err(_) => {
                // Duplicate or degenerate point — skip but push a placeholder
                // Use the last inserted vertex handle as fallback
                if let Some(last) = vertex_handles.last().copied() {
                    vertex_handles.push(last);
                }
                continue;
            }
        }
    }

    // Add constraint edges — catch panics from intersecting constraints
    // and fall back to unconstrained Delaunay if CDT fails
    let mut cdt_ok = true;
    for (i, j) in &constraint_edges {
        if *i < vertex_handles.len() && *j < vertex_handles.len() {
            let h_i = vertex_handles[*i];
            let h_j = vertex_handles[*j];
            // add_constraint can panic if edges intersect — use catch_unwind
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                cdt.add_constraint(h_i, h_j);
            }));
            if result.is_err() {
                cdt_ok = false;
                break;
            }
        }
    }

    // If CDT failed due to intersecting constraints, rebuild as plain Delaunay
    // (without constraints) — this is less accurate but won't panic
    if !cdt_ok {
        log::warn!("CDT constraint edges intersect — falling back to unconstrained Delaunay");
        let mut fallback: spade::DelaunayTriangulation<SpadePoint> = spade::DelaunayTriangulation::new();
        vertex_handles.clear();
        for (idx, p) in all_uv_points.iter().enumerate() {
            let spade_pt = SpadePoint {
                x: p.u,
                y: p.v,
                index: idx,
            };
            match fallback.insert(spade_pt) {
                Ok(handle) => vertex_handles.push(handle),
                Err(_) => {
                    if let Some(last) = vertex_handles.last().copied() {
                        vertex_handles.push(last);
                    }
                }
            }
        }
        // Extract triangles from fallback Delaunay
        let mut mesh = TriangleMesh::new();
        let mut vertex_map: std::collections::HashMap<usize, u32> = std::collections::HashMap::new();

        for face in fallback.inner_faces() {
            let positions = face.positions();
            let verts: Vec<_> = face
                .vertices()
                .iter()
                .map(|v| {
                    let sp = v.position();
                    let data = v.data();
                    (sp.x, sp.y, data.index)
                })
                .collect();

            if verts.len() != 3 { continue; }

            let centroid_u = (verts[0].0 + verts[1].0 + verts[2].0) / 3.0;
            let centroid_v = (verts[0].1 + verts[1].1 + verts[2].1) / 3.0;
            let centroid = Point2d::new(centroid_u, centroid_v);

            if !domain.contains(&centroid) { continue; }

            let area = triangle_area_2d(
                positions[0].x, positions[0].y,
                positions[1].x, positions[1].y,
                positions[2].x, positions[2].y,
            );
            if area < 1e-20 { continue; }

            let mut tri_indices = [0u32; 3];
            for (k, vert) in verts.iter().enumerate() {
                let idx = vert.2;
                let entry = vertex_map.entry(idx).or_insert_with(|| {
                    let uv = Point2d::new(vert.0, vert.1);
                    let p3d = surface.point_at(uv.u, uv.v);
                    let n = surface.normal_at(uv.u, uv.v);
                    let vi = mesh.add_vertex(p3d);
                    mesh.add_vertex_normal(vi, [n.x, n.y, n.z]);
                    vi
                });
                tri_indices[k] = *entry;
            }

            if forward {
                mesh.add_triangle(tri_indices[0], tri_indices[1], tri_indices[2]);
            } else {
                mesh.add_triangle(tri_indices[0], tri_indices[2], tri_indices[1]);
            }
        }
        return mesh;
    }

    // Extract triangles, keeping only those inside the domain
    let mut mesh = TriangleMesh::new();
    let mut vertex_map: std::collections::HashMap<usize, u32> = std::collections::HashMap::new();

    for face in cdt.inner_faces() {
        // Get the three vertex positions and indices
        let positions = face.positions();
        let verts: Vec<_> = face
            .vertices()
            .iter()
            .map(|v| {
                let sp = v.position();
                let data = v.data();
                (sp.x, sp.y, data.index)
            })
            .collect();

        if verts.len() != 3 {
            continue;
        }

        // Check if triangle centroid is inside the domain
        let centroid_u = (verts[0].0 + verts[1].0 + verts[2].0) / 3.0;
        let centroid_v = (verts[0].1 + verts[1].1 + verts[2].1) / 3.0;
        let centroid = Point2d::new(centroid_u, centroid_v);

        if !domain.contains(&centroid) {
            continue;
        }

        // Check for degenerate triangle (near-zero area)
        let area = triangle_area_2d(
            positions[0].x, positions[0].y,
            positions[1].x, positions[1].y,
            positions[2].x, positions[2].y,
        );
        if area < 1e-20 {
            continue;
        }

        // Add vertices and triangle
        let mut tri_indices = [0u32; 3];
        for (k, vert) in verts.iter().enumerate() {
            let idx = vert.2;
            let entry = vertex_map.entry(idx).or_insert_with(|| {
                let uv = Point2d::new(vert.0, vert.1);
                let p3d = surface.point_at(uv.u, uv.v);
                let n = surface.normal_at(uv.u, uv.v);
                let vi = mesh.add_vertex(p3d);
                mesh.add_vertex_normal(vi, [n.x, n.y, n.z]);
                vi
            });
            tri_indices[k] = *entry;
        }

        if forward {
            mesh.add_triangle(tri_indices[0], tri_indices[1], tri_indices[2]);
        } else {
            mesh.add_triangle(tri_indices[0], tri_indices[2], tri_indices[1]);
        }
    }

    mesh
}

/// Compute the signed area of a 2D triangle.
fn triangle_area_2d(x0: f64, y0: f64, x1: f64, y1: f64, x2: f64, y2: f64) -> f64 {
    ((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)).abs() * 0.5
}

/// Generate interior UV grid points for a parametric domain.
///
/// Creates a regular grid of points within the domain's bounding box,
/// excluding points that are outside the domain or too close to boundaries.
pub fn generate_interior_points(
    domain: &ParametricDomain,
    n_u: usize,
    n_v: usize,
    boundary_margin: f64,
) -> Vec<Point2d> {
    let (u_min, u_max, v_min, v_max) = domain.bounding_box();
    let mut points = Vec::new();

    for j in 1..n_v {
        for i in 1..n_u {
            let u = u_min + (u_max - u_min) * i as f64 / n_u as f64;
            let v = v_min + (v_max - v_min) * j as f64 / n_v as f64;
            let pt = Point2d::new(u, v);

            // Check that the point is inside the domain
            if domain.contains(&pt) {
                // Check distance to boundary (skip points too close)
                let mut min_dist_sq = f64::MAX;
                for p in &domain.outer_boundary {
                    let du = u - p.u;
                    let dv = v - p.v;
                    min_dist_sq = min_dist_sq.min(du * du + dv * dv);
                }
                if min_dist_sq > boundary_margin * boundary_margin {
                    points.push(pt);
                }
            }
        }
    }

    points
}

/// Generate interior UV points for NURBS surfaces, respecting knot ranges.
///
/// Places additional sample points at knot boundaries to capture
/// surface features that occur at knot spans.
pub fn generate_nurbs_interior_points(
    domain: &ParametricDomain,
    u_knots: &[f64],
    v_knots: &[f64],
    n_sub: usize,
) -> Vec<Point2d> {
    let (u_min, u_max, v_min, v_max) = domain.bounding_box();
    let mut points = Vec::new();

    // Filter knots within domain
    let u_knots_in_range: Vec<f64> = u_knots
        .iter()
        .filter(|&&k| k > u_min && k < u_max)
        .cloned()
        .collect();
    let v_knots_in_range: Vec<f64> = v_knots
        .iter()
        .filter(|&&k| k > v_min && k < v_max)
        .cloned()
        .collect();

    // Generate grid points: knot boundaries + interior subdivisions
    let mut u_values: Vec<f64> = vec![u_min];
    for k in &u_knots_in_range {
        u_values.push(*k);
    }
    u_values.push(u_max);

    let mut v_values: Vec<f64> = vec![v_min];
    for k in &v_knots_in_range {
        v_values.push(*k);
    }
    v_values.push(v_max);

    // Subdivide each knot span
    let mut u_grid: Vec<f64> = Vec::new();
    for i in 0..u_values.len() - 1 {
        for j in 0..n_sub {
            let t = j as f64 / n_sub as f64;
            u_grid.push(u_values[i] + t * (u_values[i + 1] - u_values[i]));
        }
    }
    u_grid.push(u_max);

    let mut v_grid: Vec<f64> = Vec::new();
    for i in 0..v_values.len() - 1 {
        for j in 0..n_sub {
            let t = j as f64 / n_sub as f64;
            v_grid.push(v_values[i] + t * (v_values[i + 1] - v_values[i]));
        }
    }
    v_grid.push(v_max);

    // Filter points inside domain
    for &u in &u_grid {
        for &v in &v_grid {
            let pt = Point2d::new(u, v);
            if domain.contains(&pt) {
                points.push(pt);
            }
        }
    }

    points
}

// ============================================================
// Integration: CDT-based surface triangulation
// ============================================================

/// Triangulate a curved surface using UV-space Constrained Delaunay Triangulation.
///
/// This is a more accurate alternative to `triangulate_surface_uv_trimmed` that
/// uses CDT to ensure boundary edges are respected exactly and interior triangles
/// satisfy the Delaunay criterion (maximizing minimum angle).
pub fn triangulate_surface_uv_cdt(
    surface: &Surface,
    boundary_points: &[Point3d],
    hole_polylines: &[Vec<Point3d>],
    forward: bool,
    params: &crate::triangulate::TriangulationParams,
) -> TriangleMesh {
    if boundary_points.is_empty() {
        return TriangleMesh::new();
    }

    // Project 3D boundary to UV
    let mut outer_uv: Vec<Point2d> = boundary_points
        .iter()
        .map(|p| {
            let (u, v) = surface.project_point(p);
            Point2d::new(u, v)
        })
        .collect();

    // Normalize UV for periodic surfaces
    let u_period = if surface.is_u_periodic() {
        Some(2.0 * PI)
    } else {
        None
    };
    let v_period = if surface.is_v_periodic() {
        Some(2.0 * PI)
    } else {
        None
    };
    crate::triangulate::normalize_uv_polygon(&mut outer_uv, u_period, v_period);

    // Compute UV range
    let mut u_min = f64::MAX;
    let mut u_max = f64::MIN;
    let mut v_min = f64::MAX;
    let mut v_max = f64::MIN;
    for p in &outer_uv {
        u_min = u_min.min(p.u);
        u_max = u_max.max(p.u);
        v_min = v_min.min(p.v);
        v_max = v_max.max(p.v);
    }
    let margin_u = (u_max - u_min) * 0.01;
    let margin_v = (v_max - v_min) * 0.01;

    // Project holes to UV
    let holes_uv: Vec<Vec<Point2d>> = hole_polylines
        .iter()
        .map(|hole| {
            let mut huv: Vec<Point2d> = hole
                .iter()
                .map(|p| {
                    let (u, v) = surface.project_point(p);
                    Point2d::new(u, v)
                })
                .collect();
            crate::triangulate::normalize_uv_polygon(&mut huv, u_period, v_period);
            huv
        })
        .collect();

    // Create parametric domain
    let mut domain = ParametricDomain::new(
        outer_uv,
        (u_min - margin_u, u_max + margin_u),
        (v_min - margin_v, v_max + margin_v),
    );
    for hole in &holes_uv {
        domain = domain.with_hole(hole.clone());
    }

    // Generate interior points using adaptive sampling
    let (n_u, n_v) = if params.adaptive {
        crate::adaptive::required_samples(
            surface,
            u_min, u_max, v_min, v_max,
            params.max_deviation, params.detail_level,
        )
    } else {
        (params.angular_samples, params.height_samples)
    };

    // Compute margin for interior points (avoid placing too close to boundary)
    let u_step = (u_max - u_min) / n_u.max(1) as f64;
    let v_step = (v_max - v_min) / n_v.max(1) as f64;
    let boundary_margin = u_step.min(v_step) * 0.3;

    let interior_points = generate_interior_points(&domain, n_u, n_v, boundary_margin);

    // Triangulate using CDT
    triangulate_cdt(&domain, surface, forward, &interior_points)
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_domain_contains_square() {
        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(1.0, 0.0),
            Point2d::new(1.0, 1.0),
            Point2d::new(0.0, 1.0),
        ];
        let domain = ParametricDomain::new(outer, (0.0, 1.0), (0.0, 1.0));
        assert!(domain.contains(&Point2d::new(0.5, 0.5)));
        assert!(!domain.contains(&Point2d::new(1.5, 0.5)));
    }

    #[test]
    fn test_domain_with_hole() {
        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(2.0, 0.0),
            Point2d::new(2.0, 2.0),
            Point2d::new(0.0, 2.0),
        ];
        let hole = vec![
            Point2d::new(0.5, 0.5),
            Point2d::new(1.5, 0.5),
            Point2d::new(1.5, 1.5),
            Point2d::new(0.5, 1.5),
        ];
        let domain = ParametricDomain::new(outer, (0.0, 2.0), (0.0, 2.0)).with_hole(hole);
        assert!(domain.contains(&Point2d::new(0.25, 0.25))); // Outside hole
        assert!(!domain.contains(&Point2d::new(1.0, 1.0))); // Inside hole
    }

    #[test]
    fn test_cylinder_with_hole() {
        use draper_geometry::{CylinderSurface, Point3d, Surface};

        // Create a cylinder surface
        let cyl = CylinderSurface::new_z(5.0);
        let surface = Surface::Cylinder(cyl);

        // Create a domain representing a half-cylinder with a rectangular hole
        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(PI, 0.0),
            Point2d::new(PI, 10.0),
            Point2d::new(0.0, 10.0),
        ];
        let hole = vec![
            Point2d::new(1.0, 3.0),
            Point2d::new(2.0, 3.0),
            Point2d::new(2.0, 7.0),
            Point2d::new(1.0, 7.0),
        ];
        let domain = ParametricDomain::new(outer, (0.0, PI), (0.0, 10.0)).with_hole(hole);

        // Generate interior points
        let interior = generate_interior_points(&domain, 10, 10, 0.1);

        // All interior points should be inside domain
        for p in &interior {
            assert!(
                domain.contains(p),
                "Interior point {:?} should be inside domain",
                p
            );
        }

        // Triangulate
        let mesh = triangulate_cdt(&domain, &surface, true, &interior);
        assert!(
            !mesh.triangles.is_empty(),
            "Should generate triangles"
        );
    }

    #[test]
    fn test_sphere_band() {
        use draper_geometry::{SphereSurface, Point3d, Surface};

        let sphere = SphereSurface::new(Point3d::ORIGIN, 10.0);
        let surface = Surface::Sphere(sphere);

        // Create a domain for a spherical band (v from PI/4 to PI/2)
        // This forms a proper closed polygon in UV space
        let n_pts = 20;
        let mut outer: Vec<Point2d> = Vec::new();
        // Bottom edge: v = PI/4, u from 0 to 2*PI
        for i in 0..n_pts {
            let u = 2.0 * PI * i as f64 / n_pts as f64;
            outer.push(Point2d::new(u, PI / 4.0));
        }
        // Right edge: u = 2*PI, v from PI/4 to PI/2
        outer.push(Point2d::new(2.0 * PI, PI / 2.0));
        // Top edge: v = PI/2, u from 2*PI back to 0
        for i in (0..n_pts).rev() {
            let u = 2.0 * PI * i as f64 / n_pts as f64;
            outer.push(Point2d::new(u, PI / 2.0));
        }
        // Left edge: u = 0, v from PI/2 to PI/4
        outer.push(Point2d::new(0.0, PI / 4.0));

        let domain = ParametricDomain::new(outer, (0.0, 2.0 * PI), (PI / 4.0, PI / 2.0));
        let interior = generate_interior_points(&domain, 10, 5, 0.01);
        let mesh = triangulate_cdt(&domain, &surface, true, &interior);
        assert!(
            !mesh.triangles.is_empty(),
            "Sphere band should generate triangles"
        );
    }

    #[test]
    fn test_nurbs_interior_points() {
        // Test NURBS interior point generation with mock knot vectors
        let outer = vec![
            Point2d::new(0.0, 0.0),
            Point2d::new(4.0, 0.0),
            Point2d::new(4.0, 4.0),
            Point2d::new(0.0, 4.0),
        ];
        let domain = ParametricDomain::new(outer, (0.0, 4.0), (0.0, 4.0));

        let u_knots = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let v_knots = vec![0.0, 2.0, 4.0];

        let points = generate_nurbs_interior_points(&domain, &u_knots, &v_knots, 2);

        // All generated points should be inside the domain
        for p in &points {
            assert!(
                domain.contains(p),
                "NURBS interior point {:?} should be inside domain",
                p
            );
        }
        // Should generate some points
        assert!(!points.is_empty(), "Should generate NURBS interior points");
    }
}
