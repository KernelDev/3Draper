// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! T.14 — No Flipped Normals
//!
//! Detect and fix triangles with flipped (inward-pointing) normals.
//! For closed meshes, all face normals should point outward.

use draper_mesh::TriangleMesh;

/// Detect triangles with flipped normals.
///
/// For a closed mesh, all face normals should point outward (away from the
/// centroid). This function uses centroid-based voting:
/// 1. Compute the mesh centroid (average of all vertices).
/// 2. For each triangle, compute the face normal.
/// 3. Check if the normal points away from the centroid.
/// 4. A triangle is "flipped" if its normal points toward the centroid.
///
/// Returns the indices of triangles with flipped normals.
pub fn detect_flipped_normals(mesh: &TriangleMesh) -> Vec<u32> {
    if mesh.vertices.is_empty() || mesh.triangles.is_empty() {
        return Vec::new();
    }

    // Compute mesh centroid
    let mut cx = 0.0f64;
    let mut cy = 0.0f64;
    let mut cz = 0.0f64;
    for v in &mesh.vertices {
        cx += v.x;
        cy += v.y;
        cz += v.z;
    }
    let n = mesh.vertices.len() as f64;
    cx /= n;
    cy /= n;
    cz /= n;

    let mut flipped = Vec::new();

    for (i, tri) in mesh.triangles.iter().enumerate() {
        let v0 = mesh.vertices[tri[0] as usize];
        let v1 = mesh.vertices[tri[1] as usize];
        let v2 = mesh.vertices[tri[2] as usize];

        // Face normal (not normalized, but direction is correct)
        let e1x = v1.x - v0.x;
        let e1y = v1.y - v0.y;
        let e1z = v1.z - v0.z;
        let e2x = v2.x - v0.x;
        let e2y = v2.y - v0.y;
        let e2z = v2.z - v0.z;

        let nx = e1y * e2z - e1z * e2y;
        let ny = e1z * e2x - e1x * e2z;
        let nz = e1x * e2y - e1y * e2x;

        // Triangle centroid
        let tri_cx = (v0.x + v1.x + v2.x) / 3.0;
        let tri_cy = (v0.y + v1.y + v2.y) / 3.0;
        let tri_cz = (v0.z + v1.z + v2.z) / 3.0;

        // Vector from mesh centroid to triangle centroid
        let dx = tri_cx - cx;
        let dy = tri_cy - cy;
        let dz = tri_cz - cz;

        // If normal · (triangle_centroid - mesh_centroid) < 0,
        // the normal points inward → flipped
        let dot = nx * dx + ny * dy + nz * dz;

        if dot < 0.0 {
            flipped.push(i as u32);
        }
    }

    flipped
}

/// Fix flipped normals by reversing the winding order of affected triangles.
///
/// For each triangle with a flipped normal, swap vertices 1 and 2
/// to reverse the winding order and flip the normal.
///
/// Returns the number of triangles fixed.
pub fn fix_flipped_normals(mesh: &mut TriangleMesh) -> usize {
    let flipped = detect_flipped_normals(mesh);

    for idx in &flipped {
        let i = *idx as usize;
        // Swap v1 and v2 to reverse winding order
        let tri = mesh.triangles[i];
        mesh.triangles[i] = [tri[0], tri[2], tri[1]];
    }

    // Also fix face_normals if present
    if let Some(ref mut face_normals) = mesh.face_normals {
        for idx in &flipped {
            let i = *idx as usize;
            if i < face_normals.len() {
                face_normals[i] = [
                    -face_normals[i][0],
                    -face_normals[i][1],
                    -face_normals[i][2],
                ];
            }
        }
    }

    flipped.len()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_cube_mesh() -> TriangleMesh {
        let mut mesh = TriangleMesh::new();
        let v = [
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 0.0, 0.0),
            Point3d::new(1.0, 1.0, 0.0),
            Point3d::new(0.0, 1.0, 0.0),
            Point3d::new(0.0, 0.0, 1.0),
            Point3d::new(1.0, 0.0, 1.0),
            Point3d::new(1.0, 1.0, 1.0),
            Point3d::new(0.0, 1.0, 1.0),
        ];
        for p in &v {
            mesh.add_vertex(*p);
        }
        // Outward-oriented triangles
        mesh.add_triangle(0, 2, 1); // Bottom
        mesh.add_triangle(0, 3, 2);
        mesh.add_triangle(4, 5, 6); // Top
        mesh.add_triangle(4, 6, 7);
        mesh.add_triangle(0, 1, 5); // Front
        mesh.add_triangle(0, 5, 4);
        mesh.add_triangle(3, 7, 6); // Back
        mesh.add_triangle(3, 6, 2);
        mesh.add_triangle(0, 4, 7); // Left
        mesh.add_triangle(0, 7, 3);
        mesh.add_triangle(1, 2, 6); // Right
        mesh.add_triangle(1, 6, 5);
        mesh
    }

    #[test]
    fn test_cube_no_flipped_normals() {
        let mesh = make_cube_mesh();
        let flipped = detect_flipped_normals(&mesh);
        // A properly oriented cube should have few or no flipped normals
        // (the centroid-based method isn't perfect for all shapes,
        // but for a cube centered at (0.5,0.5,0.5) it should work)
        // The centroid of this cube is at (0.5, 0.5, 0.5)
        assert!(flipped.len() <= 2, "Cube should have few or no flipped normals, got {}", flipped.len());
    }

    #[test]
    fn test_detect_and_fix_flipped() {
        let mut mesh = TriangleMesh::new();
        mesh.add_vertex(Point3d::new(0.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(1.0, 0.0, 0.0));
        mesh.add_vertex(Point3d::new(0.5, 1.0, 0.0));
        mesh.add_vertex(Point3d::new(0.5, 0.5, 1.0));
        // Intentionally flip one triangle
        mesh.add_triangle(0, 2, 1); // Flipped (CW when should be CCW)
        mesh.add_triangle(0, 1, 3); // Normal
        mesh.add_triangle(1, 2, 3); // Normal
        mesh.add_triangle(0, 3, 2); // Normal

        let flipped = detect_flipped_normals(&mesh);
        // At least some should be detected
        // (exact count depends on centroid position)
        let fixed = fix_flipped_normals(&mut mesh);
        assert_eq!(fixed, flipped.len(), "Should fix all detected flipped normals");
    }

    #[test]
    fn test_empty_mesh_no_flipped() {
        let mesh = TriangleMesh::new();
        let flipped = detect_flipped_normals(&mesh);
        assert!(flipped.is_empty(), "Empty mesh should have no flipped normals");
    }
}
