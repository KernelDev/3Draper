//! Earcut-style triangulation for simple polygons.
//!
//! Implements a 2D ear-clipping algorithm for triangulating planar faces.

use draper_geometry::point::Point2;

/// Triangulate a simple 2D polygon using ear clipping.
/// Returns a list of triangle indices (3 indices per triangle).
pub fn ear_clip_polygon(vertices: &[Point2]) -> Vec<u32> {
    if vertices.len() < 3 {
        return Vec::new();
    }

    let n = vertices.len();

    // Compute signed area to determine winding
    let mut signed_area = 0.0f64;
    for i in 0..n {
        let j = (i + 1) % n;
        signed_area += vertices[i].u * vertices[j].v;
        signed_area -= vertices[j].u * vertices[i].v;
    }

    // Build index list; ensure CCW winding for our algorithm
    let mut indices: Vec<u32> = (0..n as u32).collect();
    if signed_area < 0.0 {
        // CW input — reverse to CCW
        indices.reverse();
    }

    let mut triangles = Vec::new();
    let mut remaining = n;

    while remaining > 3 {
        let mut ear_found = false;

        for i in 0..remaining {
            let prev = if i == 0 { remaining - 1 } else { i - 1 };
            let next = (i + 1) % remaining;

            let pi = indices[prev] as usize;
            let pj = indices[i] as usize;
            let pk = indices[next] as usize;

            // Check convexity (CCW: cross product should be positive)
            let cross = (vertices[pj].u - vertices[pi].u) * (vertices[pk].v - vertices[pi].v)
                      - (vertices[pj].v - vertices[pi].v) * (vertices[pk].u - vertices[pi].u);

            if cross < 0.0 {
                continue; // Reflex vertex, not an ear
            }

            // Check that no other vertex is inside this triangle
            let mut is_ear = true;
            for m in 0..remaining {
                let pm = indices[m] as usize;
                if pm == pi || pm == pj || pm == pk {
                    continue;
                }

                if point_in_triangle(
                    vertices[pm].u, vertices[pm].v,
                    vertices[pi].u, vertices[pi].v,
                    vertices[pj].u, vertices[pj].v,
                    vertices[pk].u, vertices[pk].v,
                ) {
                    is_ear = false;
                    break;
                }
            }

            if is_ear {
                triangles.push(indices[prev]);
                triangles.push(indices[i]);
                triangles.push(indices[next]);
                indices.remove(i);
                remaining -= 1;
                ear_found = true;
                break;
            }
        }

        if !ear_found {
            break; // Degenerate polygon
        }
    }

    // Last triangle
    if remaining == 3 {
        triangles.push(indices[0]);
        triangles.push(indices[1]);
        triangles.push(indices[2]);
    }

    triangles
}

fn point_in_triangle(
    px: f64, py: f64,
    ax: f64, ay: f64,
    bx: f64, by: f64,
    cx: f64, cy: f64,
) -> bool {
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
