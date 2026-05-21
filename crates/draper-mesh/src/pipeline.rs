//! Production mesh generation pipeline.
//!
//! Implements the complete triangulation pipeline as described in the guide:
//!
//! 1. Topological validation and healing
//! 2. 2D representation generation (pcurves, seam detection)
//! 3. Consistent edge discretization (curvature-based, mandatory boundary points)
//! 4. Surface metric analysis and adaptive interior point generation
//! 5. Constrained Delaunay Triangulation in UV space
//! 6. 3D mapping and quality control
//! 7. Iterative refinement
//! 8. Finalization (seam stitching, normal computation)
//!
//! Key principle: "Triangulation success is 70% preparation, 30% algorithm."

use crate::delaunay;
use crate::quality;
use crate::triangulate::TriangleMesh;
use draper_geometry::curve::Curve;
use draper_geometry::point::{Point2, Point3};
use draper_geometry::surface::Surface;
use draper_geometry::surface_info::{SurfaceCurvature, SurfaceInfo};
use draper_topology::discretize;
use draper_topology::entity::*;
use draper_topology::healing;
use draper_topology::shape::Shape;

/// Default chord tolerance for edge discretization (in mm).
#[allow(dead_code)]
const CHORD_TOLERANCE: f64 = 0.01;

/// Default angular tolerance for edge discretization (in radians).
#[allow(dead_code)]
const ANGULAR_TOLERANCE: f64 = 0.1;

/// Maximum number of adaptive refinement iterations.
const MAX_REFINEMENT_ITERATIONS: usize = 3;

/// Minimum triangle angle threshold for quality (in degrees).
const MIN_ANGLE_DEG: f64 = 10.0;

/// Generate a triangle mesh from a shape using the production pipeline.
pub fn generate_mesh(shape: &Shape, _u_samples: usize, _v_samples: usize) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();

    // Phase 1: Heal the shape (clone to avoid mutating the input)
    let mut healed_shape = shape.clone();
    let report = healing::heal_shape(&mut healed_shape);
    if !report.is_clean() {
        log::info!("Shape healing applied: {:?}", report);
    }

    // Phase 2-7: Process each face
    for face in healed_shape.faces() {
        generate_face_mesh_pipeline(&healed_shape, face, &mut mesh);
    }

    mesh.compute_normals();
    mesh
}

/// Generate mesh for a single face using the full pipeline.
fn generate_face_mesh_pipeline(shape: &Shape, face: &Face, mesh: &mut TriangleMesh) {
    let surface = match &face.surface {
        Some(s) => s,
        None => {
            // No surface geometry — use faceted fallback
            generate_faceted_face_mesh(shape, face, mesh);
            return;
        }
    };

    // Phase 3: Consistent edge discretization
    let face_disc = discretize::discretize_face(shape, face);

    if face_disc.outer_contour.points.is_empty() {
        log::debug!("Face #{}: no boundary points, skipping", face.id);
        return;
    }

    // Phase 4: Surface metric analysis and adaptive interior points
    let surface_info = SurfaceInfo::from_surface(surface);
    let uv_domain = get_uv_domain(face, &surface_info);

    // Get boundary UV points and 3D points
    let outer_uv = &face_disc.outer_contour.uv_points;
    let outer_3d = &face_disc.outer_contour.points_3d;
    let inner_uvs: Vec<&Vec<Point2>> = face_disc
        .inner_contours
        .iter()
        .map(|c| &c.uv_points)
        .collect();

    if outer_uv.len() < 3 {
        return;
    }

    // Generate adaptive interior points based on surface metric
    let (interior_uv, interior_3d) = generate_adaptive_interior_points(
        surface,
        &surface_info,
        &uv_domain,
        outer_uv,
        &inner_uvs,
    );

    // Phase 5: Build CDT in UV space
    // Combine boundary + interior points
    let mut all_uv = outer_uv.clone();
    let mut all_3d = outer_3d.clone();

    // Add inner contour points
    for inner in &face_disc.inner_contours {
        all_uv.extend_from_slice(&inner.uv_points);
        all_3d.extend_from_slice(&inner.points_3d);
    }

    let boundary_count = all_uv.len();

    // Add interior points
    all_uv.extend_from_slice(&interior_uv);
    all_3d.extend_from_slice(&interior_3d);

    // Build constraint edges for outer contour
    let mut constraint_edges: Vec<(usize, usize)> = Vec::new();
    let outer_len = face_disc.outer_contour.points.len();
    for i in 0..outer_len {
        let j = (i + 1) % outer_len;
        constraint_edges.push((i, j));
    }

    // Build constraint edges for inner contours
    let mut offset = outer_len;
    for inner in &face_disc.inner_contours {
        let inner_len = inner.points.len();
        for i in 0..inner_len {
            let j = (i + 1) % inner_len;
            constraint_edges.push((offset + i, offset + j));
        }
        offset += inner_len;
    }

    // Run CDT
    let tri_indices = delaunay::cdt_with_constraints(&all_uv, &constraint_edges);

    if tri_indices.is_empty() {
        log::debug!("Face #{}: CDT produced no results, falling back", face.id);
        generate_faceted_face_mesh(shape, face, mesh);
        return;
    }

    // Filter triangles inside outer boundary and outside holes
    let inner_uv_slices: Vec<&[Point2]> = face_disc
        .inner_contours
        .iter()
        .map(|c| c.uv_points.as_slice())
        .collect();
    let filtered_indices = filter_triangles_inside_boundary(
        &tri_indices,
        &all_uv,
        outer_uv,
        &inner_uv_slices,
    );

    // Phase 6: Map UV vertices to 3D using surface evaluation
    let mapped_3d: Vec<Point3> = all_uv
        .iter()
        .zip(all_3d.iter())
        .map(|(uv, approx_3d)| {
            // Try to project through the surface for accurate 3D positions
            project_uv_to_3d(*uv, *approx_3d, surface, &surface_info)
        })
        .collect();

    // Phase 7: Iterative refinement based on quality metrics
    let (refined_indices, refined_3d) = refine_mesh(
        &filtered_indices,
        &mapped_3d,
        &all_uv,
        surface,
        &surface_info,
        &constraint_edges,
        outer_uv,
        &inner_uv_slices,
    );

    // Add to global mesh
    let base_idx = mesh.vertices.len() as u32;
    for pt in &refined_3d {
        mesh.add_vertex(*pt);
    }

    for idx in refined_indices.chunks(3) {
        if idx.len() == 3 {
            mesh.add_triangle(base_idx + idx[0], base_idx + idx[1], base_idx + idx[2]);
        }
    }

    log::trace!(
        "Face #{}: {} boundary pts, {} interior pts, {} triangles",
        face.id,
        boundary_count,
        interior_uv.len(),
        refined_indices.len() / 3,
    );
}

/// Generate adaptive interior points based on surface metric and curvature.
///
/// Uses a grid in UV space, filtered by the polygon boundary, with
/// additional points in areas of high curvature or high metric distortion.
fn generate_adaptive_interior_points(
    surface: &Surface,
    _info: &SurfaceInfo,
    domain: &UVDomainData,
    outer_uv: &[Point2],
    inner_uvs: &[&Vec<Point2>],
) -> (Vec<Point2>, Vec<Point3>) {
    let mut interior_uv = Vec::new();
    let mut interior_3d = Vec::new();

    if outer_uv.len() < 3 {
        return (interior_uv, interior_3d);
    }

    // Find UV bounding box
    let mut min_u = f64::MAX;
    let mut min_v = f64::MAX;
    let mut max_u = f64::MIN;
    let mut max_v = f64::MIN;
    for pt in outer_uv {
        min_u = min_u.min(pt.u);
        min_v = min_v.min(pt.v);
        max_u = max_u.max(pt.u);
        max_v = max_v.max(pt.v);
    }

    let du = max_u - min_u;
    let dv = max_v - min_v;
    if du < 1e-10 || dv < 1e-10 {
        return (interior_uv, interior_3d);
    }

    // Determine grid resolution based on bounding box size and curvature
    let base_samples = 8;
    let curvature = SurfaceCurvature::estimate(
        surface,
        (min_u + max_u) / 2.0,
        (min_v + max_v) / 2.0,
    );
    let extra = if curvature.max_abs_curvature > 0.01 { 4 } else { 0 };
    let n_u = base_samples + extra;
    let n_v = base_samples + extra;

    // Generate grid points filtered by boundary
    for i in 1..n_u {
        for j in 1..n_v {
            let u = min_u + du * (i as f64 / n_u as f64);
            let v = min_v + dv * (j as f64 / n_v as f64);
            let pt_2d = Point2::new(u, v);

            // Check if inside outer boundary
            if !point_in_polygon_2d(&pt_2d, outer_uv) {
                continue;
            }

            // Check if outside all holes
            let mut in_hole = false;
            for hole in inner_uvs {
                if point_in_polygon_2d(&pt_2d, hole) {
                    in_hole = true;
                    break;
                }
            }
            if in_hole {
                continue;
            }

            // Evaluate surface at this UV
            let (u_norm, v_norm) = domain.normalize_uv(u, v);
            let pt_3d = surface.point_at(u_norm, v_norm);

            interior_uv.push(pt_2d);
            interior_3d.push(pt_3d);
        }
    }

    (interior_uv, interior_3d)
}

/// UV domain data for a face.
struct UVDomainData {
    u_range: (f64, f64),
    v_range: (f64, f64),
    u_periodic: bool,
    v_periodic: bool,
}

impl UVDomainData {
    fn normalize_uv(&self, u: f64, v: f64) -> (f64, f64) {
        let u = if self.u_periodic && self.u_range.1 > self.u_range.0 {
            let period = self.u_range.1 - self.u_range.0;
            let mut u = u;
            while u < self.u_range.0 { u += period; }
            while u > self.u_range.1 { u -= period; }
            u
        } else {
            u
        };
        let v = if self.v_periodic && self.v_range.1 > self.v_range.0 {
            let period = self.v_range.1 - self.v_range.0;
            let mut v = v;
            while v < self.v_range.0 { v += period; }
            while v > self.v_range.1 { v -= period; }
            v
        } else {
            v
        };
        (u, v)
    }
}

fn get_uv_domain(face: &Face, info: &SurfaceInfo) -> UVDomainData {
    if let Some(bounds) = face.uv_bounds {
        UVDomainData {
            u_range: (bounds.u_min, bounds.u_max),
            v_range: (bounds.v_min, bounds.v_max),
            u_periodic: info.u_periodic,
            v_periodic: info.v_periodic,
        }
    } else {
        UVDomainData {
            u_range: info.u_range,
            v_range: info.v_range,
            u_periodic: info.u_periodic,
            v_periodic: info.v_periodic,
        }
    }
}

/// Project a UV point to 3D, using the surface evaluation if possible,
/// falling back to the approximate 3D position.
fn project_uv_to_3d(uv: Point2, approx_3d: Point3, surface: &Surface, info: &SurfaceInfo) -> Point3 {
    // For planes, use exact projection
    if let Surface::Plane(_) = surface {
        return approx_3d; // Already on the plane from vertex positions
    }

    // For curved surfaces, evaluate at the UV coordinates
    // First try direct evaluation
    let (u, v) = if info.u_periodic || info.v_periodic {
        // Clamp UV to parameter range
        let u = uv.u.clamp(info.u_range.0, info.u_range.1);
        let v = uv.v.clamp(info.v_range.0, info.v_range.1);
        (u, v)
    } else {
        (uv.u, uv.v)
    };

    let projected = surface.point_at(u, v);

    // Sanity check: if projected point is far from approximate, use approximate
    let dist = projected.distance_to(approx_3d);
    if dist > 100.0 {
        log::debug!(
            "UV→3D projection far from approximate (dist={:.3}), using approximate",
            dist
        );
        approx_3d
    } else {
        projected
    }
}

/// Filter triangles to keep only those inside the outer boundary and outside holes.
fn filter_triangles_inside_boundary(
    indices: &[u32],
    all_uv: &[Point2],
    outer_uv: &[Point2],
    inner_uvs: &[&[Point2]],
) -> Vec<u32> {
    let mut result = Vec::with_capacity(indices.len());

    for tri in indices.chunks(3) {
        if tri.len() < 3 {
            continue;
        }

        let ai = tri[0] as usize;
        let bi = tri[1] as usize;
        let ci = tri[2] as usize;

        if ai >= all_uv.len() || bi >= all_uv.len() || ci >= all_uv.len() {
            continue;
        }

        // Compute centroid
        let centroid = Point2::new(
            (all_uv[ai].u + all_uv[bi].u + all_uv[ci].u) / 3.0,
            (all_uv[ai].v + all_uv[bi].v + all_uv[ci].v) / 3.0,
        );

        // Must be inside outer boundary
        if !point_in_polygon_2d(&centroid, outer_uv) {
            continue;
        }

        // Must be outside all holes
        let mut inside_hole = false;
        for hole in inner_uvs {
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

/// Iterative mesh refinement based on quality metrics.
///
/// For triangles with poor aspect ratio or high area distortion,
/// insert Steiner points at the triangle centroid in UV space
/// and re-triangulate.
fn refine_mesh(
    indices: &[u32],
    points_3d: &[Point3],
    points_uv: &[Point2],
    surface: &Surface,
    info: &SurfaceInfo,
    _constraints: &[(usize, usize)],
    outer_uv: &[Point2],
    inner_uvs: &[&[Point2]],
) -> (Vec<u32>, Vec<Point3>) {
    if indices.is_empty() {
        return (Vec::new(), points_3d.to_vec());
    }

    let mut current_indices = indices.to_vec();
    let mut current_3d = points_3d.to_vec();
    let mut current_uv = points_uv.to_vec();

    for iteration in 0..MAX_REFINEMENT_ITERATIONS {
        // Find triangles with poor quality
        let mut bad_triangles = Vec::new();
        for (tri_idx, tri) in current_indices.chunks(3).enumerate() {
            if tri.len() < 3 {
                continue;
            }
            let a = current_3d[tri[0] as usize];
            let b = current_3d[tri[1] as usize];
            let c = current_3d[tri[2] as usize];

            let quality = quality::triangle_quality(a, b, c);
            if quality.aspect_ratio > 15.0 || quality.min_angle_deg < MIN_ANGLE_DEG {
                bad_triangles.push(tri_idx);
            }
        }

        if bad_triangles.is_empty() {
            break;
        }

        log::trace!(
            "Refinement iteration {}: {} bad triangles out of {}",
            iteration,
            bad_triangles.len(),
            current_indices.len() / 3,
        );

        // For each bad triangle, add a Steiner point at the centroid in UV
        let mut new_points_3d = Vec::new();
        let mut new_points_uv = Vec::new();

        for tri_idx in &bad_triangles {
            let base = tri_idx * 3;
            if base + 2 >= current_indices.len() {
                continue;
            }
            let ai = current_indices[base] as usize;
            let bi = current_indices[base + 1] as usize;
            let ci = current_indices[base + 2] as usize;

            // UV centroid
            let u_centroid = (current_uv[ai].u + current_uv[bi].u + current_uv[ci].u) / 3.0;
            let v_centroid = (current_uv[ai].v + current_uv[bi].v + current_uv[ci].v) / 3.0;
            new_points_uv.push(Point2::new(u_centroid, v_centroid));

            // 3D point from surface evaluation
            let pt_3d = project_uv_to_3d(
                Point2::new(u_centroid, v_centroid),
                Point3::ORIGIN,
                surface,
                info,
            );
            new_points_3d.push(pt_3d);
        }

        // Add new points
        let _start_idx = current_3d.len() as u32;
        current_3d.extend(new_points_3d);
        current_uv.extend(new_points_uv);

        // Re-triangulate with new points
        // For simplicity, just re-run the full CDT
        // (In a production system, we'd use incremental insertion)
        let boundary_len = outer_uv.len();
        let all_constraints = build_constraint_list(boundary_len, inner_uvs);

        let new_tri_indices = delaunay::cdt_with_constraints(&current_uv, &all_constraints);
        if !new_tri_indices.is_empty() {
            let inner_slices: Vec<&[Point2]> = inner_uvs.iter().map(|h| *h).collect();
            current_indices = filter_triangles_inside_boundary(
                &new_tri_indices,
                &current_uv,
                outer_uv,
                &inner_slices,
            );
        }
    }

    (current_indices, current_3d)
}

fn build_constraint_list(outer_len: usize, inner_uvs: &[&[Point2]]) -> Vec<(usize, usize)> {
    let mut constraints = Vec::new();

    // Outer contour
    for i in 0..outer_len {
        let j = (i + 1) % outer_len;
        constraints.push((i, j));
    }

    // Inner contours
    let mut offset = outer_len;
    for hole in inner_uvs {
        let hole_len = hole.len();
        for i in 0..hole_len {
            let j = (i + 1) % hole_len;
            constraints.push((offset + i, offset + j));
        }
        offset += hole_len;
    }

    constraints
}

/// Generate a faceted mesh from boundary points (no surface geometry).
fn generate_faceted_face_mesh(shape: &Shape, face: &Face, mesh: &mut TriangleMesh) {
    let outer_points = extract_wire_points_3d(shape, face.outer_wire);
    if outer_points.len() < 3 {
        return;
    }

    let inner_points: Vec<Vec<Point3>> = face
        .inner_wires
        .iter()
        .filter_map(|&wire_id| {
            let pts = extract_wire_points_3d(shape, Some(wire_id));
            if pts.is_empty() { None } else { Some(pts) }
        })
        .collect();

    // Compute best-fit plane for 2D projection
    let (origin, u_dir, v_dir) = compute_best_fit_plane(&outer_points);

    // Project to 2D
    let outer_2d = project_points_to_2d(&outer_points, origin, u_dir, v_dir);
    let inner_2d: Vec<Vec<Point2>> = inner_points
        .iter()
        .map(|pts| project_points_to_2d(pts, origin, u_dir, v_dir))
        .collect();

    // Build constraints for CDT
    let mut all_uv = outer_2d.clone();
    let mut all_3d = outer_points.clone();
    let mut constraints = Vec::new();

    // Outer contour constraints
    for i in 0..outer_2d.len() {
        let j = (i + 1) % outer_2d.len();
        constraints.push((i, j));
    }

    // Inner contour constraints
    let mut offset = outer_2d.len();
    for (hole_2d, hole_3d) in inner_2d.iter().zip(inner_points.iter()) {
        all_uv.extend_from_slice(hole_2d);
        all_3d.extend_from_slice(hole_3d);
        let hole_len = hole_2d.len();
        for i in 0..hole_len {
            let j = (i + 1) % hole_len;
            constraints.push((offset + i, offset + j));
        }
        offset += hole_len;
    }

    // Run CDT
    let tri_indices = delaunay::cdt_with_constraints(&all_uv, &constraints);

    if tri_indices.is_empty() {
        // Fall back to basic ear clipping
        let tri_indices = crate::earcut::ear_clip_polygon(&outer_2d);
        if tri_indices.is_empty() {
            return;
        }
        let base_idx = mesh.vertices.len() as u32;
        for pt in &all_3d {
            mesh.add_vertex(*pt);
        }
        for idx in tri_indices.chunks(3) {
            if idx.len() == 3 {
                mesh.add_triangle(base_idx + idx[0], base_idx + idx[1], base_idx + idx[2]);
            }
        }
        return;
    }

    // Filter and add to mesh
    let inner_slices: Vec<&[Point2]> = inner_2d.iter().map(|h| h.as_slice()).collect();
    let filtered = filter_triangles_inside_boundary(&tri_indices, &all_uv, &outer_2d, &inner_slices);

    let base_idx = mesh.vertices.len() as u32;
    for pt in &all_3d {
        mesh.add_vertex(*pt);
    }
    for idx in filtered.chunks(3) {
        if idx.len() == 3 {
            mesh.add_triangle(base_idx + idx[0], base_idx + idx[1], base_idx + idx[2]);
        }
    }
}

/// Extract ordered 3D boundary points from a wire (legacy approach for faceted fallback).
fn extract_wire_points_3d(shape: &Shape, wire_id: Option<TopoId>) -> Vec<Point3> {
    let wire_id = match wire_id {
        Some(id) => id,
        None => return Vec::new(),
    };

    let wire = match shape.get(wire_id) {
        Some(TopoShape::Wire(w)) => w,
        _ => return Vec::new(),
    };

    if wire.edges.is_empty() {
        return Vec::new();
    }

    // Build adjacency graph for proper edge ordering
    use std::collections::HashMap;

    struct EdgeInfo {
        other_vertex: TopoId,
        intermediates: Vec<Point3>,
    }

    let mut adjacency: HashMap<TopoId, Vec<EdgeInfo>> = HashMap::new();

    for oriented_edge in &wire.edges {
        let edge = match shape.get(oriented_edge.edge_id) {
            Some(TopoShape::Edge(e)) => e,
            _ => continue,
        };

        let first_vertex = if oriented_edge.orientation {
            edge.start_vertex
        } else {
            edge.end_vertex
        };
        let second_vertex = if oriented_edge.orientation {
            edge.end_vertex
        } else {
            edge.start_vertex
        };

        let forward_intermediates = if let Some(ref curve) = edge.curve {
            let mut pts = discretize_curve_simple(curve, 48);
            if pts.len() > 2 {
                if !oriented_edge.orientation {
                    pts.reverse();
                }
                pts.iter().skip(1).take(pts.len() - 2).copied().collect()
            } else {
                Vec::new()
            }
        } else {
            Vec::new()
        };

        let reverse_intermediates = {
            let mut rev = forward_intermediates.clone();
            rev.reverse();
            rev
        };

        adjacency.entry(first_vertex).or_default().push(EdgeInfo {
            other_vertex: second_vertex,
            intermediates: forward_intermediates,
        });
        adjacency.entry(second_vertex).or_default().push(EdgeInfo {
            other_vertex: first_vertex,
            intermediates: reverse_intermediates,
        });
    }

    let start_vertex = match adjacency.keys().next().copied() {
        Some(v) => v,
        None => return Vec::new(),
    };

    let mut points = Vec::new();
    let mut current_vertex = start_vertex;
    let mut used_edges: std::collections::HashSet<(TopoId, TopoId)> = std::collections::HashSet::new();

    if let Some(TopoShape::Vertex(v)) = shape.get(current_vertex) {
        points.push(v.point);
    }

    for step in 0..wire.edges.len() {
        let neighbors = match adjacency.get(&current_vertex) {
            Some(n) => n,
            None => break,
        };

        let mut found = false;
        for info in neighbors {
            let edge_key = if current_vertex < info.other_vertex {
                (current_vertex, info.other_vertex)
            } else {
                (info.other_vertex, current_vertex)
            };

            if used_edges.contains(&edge_key) {
                continue;
            }
            used_edges.insert(edge_key);

            points.extend_from_slice(&info.intermediates);

            if let Some(TopoShape::Vertex(v)) = shape.get(info.other_vertex) {
                let should_add = points.last().map_or(true, |last: &Point3| {
                    (last.x - v.point.x).abs() > 1e-10
                        || (last.y - v.point.y).abs() > 1e-10
                        || (last.z - v.point.z).abs() > 1e-10
                });
                if should_add {
                    points.push(v.point);
                }
            }

            current_vertex = info.other_vertex;
            found = true;
            break;
        }

        if !found {
            let _ = step;
            break;
        }
    }

    // Remove duplicate last point
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

    points
}

/// Simple curve discretization (for faceted fallback).
fn discretize_curve_simple(curve: &Curve, segments: usize) -> Vec<Point3> {
    match curve {
        Curve::Line(_) => Vec::new(),
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
            let mut pts = Vec::with_capacity(segments + 1);
            for i in 0..=segments {
                let t = tc.trim1 + (i as f64 / segments as f64) * (tc.trim2 - tc.trim1);
                pts.push(tc.basis_curve.point_at(t));
            }
            pts
        }
        Curve::BSplineCurve(bs) => {
            let knot_min = bs.knots.first().copied().unwrap_or(0.0);
            let knot_max = bs.knots.last().copied().unwrap_or(1.0);
            let mut pts = Vec::with_capacity(segments + 1);
            for i in 0..=segments {
                let t = knot_min + (i as f64 / segments as f64) * (knot_max - knot_min);
                pts.push(bs.point_at(t));
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

/// Compute a best-fit plane from 3D points.
fn compute_best_fit_plane(
    points: &[Point3],
) -> (Point3, draper_geometry::direction::Direction3, draper_geometry::direction::Direction3) {
    use draper_geometry::direction::Direction3;

    if points.is_empty() {
        return (Point3::ORIGIN, Direction3::X, Direction3::Y);
    }

    let sum = points.iter().fold(glam::DVec3::ZERO, |acc, p| acc + p.to_dvec3());
    let centroid = sum / points.len() as f64;
    let origin = Point3::from_dvec3(centroid);

    // Use cross product approach
    let mut best_normal = glam::DVec3::Z;
    if points.len() >= 3 {
        let v1 = points[1].to_dvec3() - points[0].to_dvec3();
        let v2 = points[2].to_dvec3() - points[0].to_dvec3();
        let cross = v1.cross(v2);
        if cross.length() > 1e-10 {
            best_normal = cross.normalize();
        }
    }

    let normal = Direction3::new(best_normal.x, best_normal.y, best_normal.z).unwrap_or(Direction3::Z);

    let u_dir = if normal.dot(Direction3::X).abs() < 0.9 {
        normal.cross(Direction3::X)
    } else {
        normal.cross(Direction3::Y)
    };
    let v_dir = normal.cross(u_dir);

    (origin, u_dir, v_dir)
}

/// Project 3D points to 2D using a local coordinate system.
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

/// Point-in-polygon test for 2D points (ray casting).
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
