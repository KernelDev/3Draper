// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Topology traversal utilities.

use crate::entity::*;
use draper_geometry::Point3d;

/// Traverse all faces of a solid.
pub fn solid_faces(solid: &Solid) -> Vec<&Face> {
    solid.faces()
}

/// Traverse all edges of a face (from its wires).
pub fn face_edges(face: &Face) -> Vec<TopoId> {
    let mut edges = Vec::new();
    if let Some(ref wire) = face.outer_wire {
        for coedge in &wire.coedges {
            edges.push(coedge.edge);
        }
    }
    for wire in &face.inner_wires {
        for coedge in &wire.coedges {
            edges.push(coedge.edge);
        }
    }
    edges
}

/// Traverse all faces of a compound.
pub fn compound_faces(compound: &Compound) -> Vec<&Face> {
    let mut faces = Vec::new();
    for solid in &compound.solids {
        faces.extend(solid.faces());
    }
    for sub in &compound.compounds {
        faces.extend(compound_faces(sub));
    }
    faces
}

/// Traverse all solids of a compound (recursively).
pub fn compound_solids(compound: &Compound) -> Vec<&Solid> {
    let mut solids = Vec::new();
    solids.extend(&compound.solids);
    for sub in &compound.compounds {
        solids.extend(compound_solids(sub));
    }
    solids
}

/// Get the bounding box of a solid (approximate).
pub fn solid_bounding_box(solid: &Solid, n_samples: usize) -> (Point3d, Point3d) {
    let mut min = Point3d::new(f64::MAX, f64::MAX, f64::MAX);
    let mut max = Point3d::new(f64::MIN, f64::MIN, f64::MIN);

    for face in solid.faces() {
        if let Some(ref surface) = face.surface {
            // Sample the surface
            for i in 0..=n_samples {
                for j in 0..=n_samples {
                    let u = i as f64 / n_samples as f64;
                    let v = j as f64 / n_samples as f64;
                    let p = surface.point_at(u * 2.0 * std::f64::consts::PI, v * std::f64::consts::PI);
                    min.x = min.x.min(p.x);
                    min.y = min.y.min(p.y);
                    min.z = min.z.min(p.z);
                    max.x = max.x.max(p.x);
                    max.y = max.y.max(p.y);
                    max.z = max.z.max(p.z);
                }
            }
        }
    }

    (min, max)
}
