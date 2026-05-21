//! Earcut-style triangulation for simple polygons.
//!
//! Implements a 2D ear-clipping algorithm that can be used for
//! triangulating planar faces. For more complex surfaces, use
//! the Delaunay triangulation via the `spade` crate.

use draper_geometry::point::Point2;

/// Triangulate a simple 2D polygon using ear clipping.
/// Returns a list of triangle indices (3 indices per triangle).
pub fn ear_clip_polygon(vertices: &[Point2]) -> Vec<u32> {
    if vertices.len() < 3 {
        return Vec::new();
    }

    let n = vertices.len();
    let mut indices: Vec<u32> = (0..n as u32).collect();
    let mut triangles = Vec::new();

    // Determine winding order
    let mut area = 0.0f64;
    for i in 0..n {
        let j = (i + 1) % n;
        area += vertices[i].u * vertices[j].v;
        area -= vertices[j].u * vertices[i].v;
    }

    if area > 0.0 {
        indices.reverse();
    }

    let mut remaining = n;
    let mut fail_count = 0;
    let mut current = 0;

    while remaining > 2 {
        if fail_count > 2 * remaining {
            // Degenerate polygon
            break;
        }

        let i = current;
        let j = (current + 1) % remaining;
        let k = (current + 2) % remaining;

        let pi = indices[i] as usize;
        let pj = indices[j] as usize;
        let pk = indices[k] as usize;

        if is_ear(vertices, &indices, i, j, k, remaining) {
            triangles.push(indices[i]);
            triangles.push(indices[j]);
            triangles.push(indices[k]);

            // Remove the ear tip
            indices.remove(j);
            remaining -= 1;
            fail_count = 0;
            current = if current > 0 { current - 1 } else { 0 };
        } else {
            fail_count += 1;
            current = (current + 1) % remaining;
        }
    }

    triangles
}

fn is_ear(
    vertices: &[Point2],
    indices: &[u32],
    i: usize,
    j: usize,
    k: usize,
    remaining: usize,
) -> bool {
    let pi = indices[i] as usize;
    let pj = indices[j] as usize;
    let pk = indices[k] as usize;

    let ax = vertices[pi].u;
    let ay = vertices[pi].v;
    let bx = vertices[pj].u;
    let by = vertices[pj].v;
    let cx = vertices[pk].u;
    let cy = vertices[pk].v;

    // Check if the triangle is convex (CCW winding)
    let cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    if cross <= 0.0 {
        return false;
    }

    // Check if any other vertex is inside the triangle
    for m in 0..remaining {
        let pm = indices[m] as usize;
        if pm == pi || pm == pj || pm == pk {
            continue;
        }

        if point_in_triangle(
            vertices[pm].u,
            vertices[pm].v,
            ax, ay,
            bx, by,
            cx, cy,
        ) {
            return false;
        }
    }

    true
}

fn point_in_triangle(px: f64, py: f64, ax: f64, ay: f64, bx: f64, by: f64, cx: f64, cy: f64) -> bool {
    let d1 = sign(px, py, ax, ay, bx, by);
    let d2 = sign(px, py, bx, by, cx, cy);
    let d3 = sign(px, py, cx, cy, ax, ay);

    let has_neg = (d1 < 0.0) || (d2 < 0.0) || (d3 < 0.0);
    let has_pos = (d1 > 0.0) || (d2 > 0.0) || (d3 > 0.0);

    !(has_neg && has_pos)
}

fn sign(px: f64, py: f64, ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    (px - bx) * (ay - by) - (ax - bx) * (py - by)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_square_triangulation() {
        let vertices = vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 0.0),
            Point2::new(1.0, 1.0),
            Point2::new(0.0, 1.0),
        ];

        let triangles = ear_clip_polygon(&vertices);
        assert_eq!(triangles.len(), 6); // 2 triangles * 3 indices
    }

    #[test]
    fn test_pentagon_triangulation() {
        let vertices = vec![
            Point2::new(0.0, 1.0),
            Point2::new(1.0, 0.5),
            Point2::new(0.8, -0.5),
            Point2::new(-0.8, -0.5),
            Point2::new(-1.0, 0.5),
        ];

        let triangles = ear_clip_polygon(&vertices);
        assert_eq!(triangles.len(), 9); // 3 triangles * 3 indices
    }
}
