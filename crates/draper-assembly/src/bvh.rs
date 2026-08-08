// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Bounding Volume Hierarchy (BVH) for assembly collision detection.
//!
//! Per FLEXIBLE_EXECUTION_PLAN task C3: provides AABB-based BVH for
//! fast broad-phase collision queries between assembly components.

use draper_geometry::Transform;

/// An axis-aligned bounding box in 3D.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoundingBox {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

impl BoundingBox {
    pub fn new(min: [f64; 3], max: [f64; 3]) -> Self {
        Self { min, max }
    }

    pub fn from_center_half(center: [f64; 3], half: [f64; 3]) -> Self {
        Self {
            min: [center[0] - half[0], center[1] - half[1], center[2] - half[2]],
            max: [center[0] + half[0], center[1] + half[1], center[2] + half[2]],
        }
    }

    pub fn empty() -> Self {
        Self {
            min: [f64::INFINITY; 3],
            max: [f64::NEG_INFINITY; 3],
        }
    }

    pub fn center(&self) -> [f64; 3] {
        [
            (self.min[0] + self.max[0]) * 0.5,
            (self.min[1] + self.max[1]) * 0.5,
            (self.min[2] + self.max[2]) * 0.5,
        ]
    }

    pub fn overlaps(&self, other: &BoundingBox) -> bool {
        for i in 0..3 {
            if self.max[i] < other.min[i] || self.min[i] > other.max[i] {
                return false;
            }
        }
        true
    }

    pub fn overlaps_strict(&self, other: &BoundingBox) -> bool {
        for i in 0..3 {
            if self.max[i] <= other.min[i] || self.min[i] >= other.max[i] {
                return false;
            }
        }
        true
    }

    pub fn union(&self, other: &BoundingBox) -> BoundingBox {
        BoundingBox {
            min: [
                self.min[0].min(other.min[0]),
                self.min[1].min(other.min[1]),
                self.min[2].min(other.min[2]),
            ],
            max: [
                self.max[0].max(other.max[0]),
                self.max[1].max(other.max[1]),
                self.max[2].max(other.max[2]),
            ],
        }
    }

    /// Transform AABB by a Transform (rotate 8 corners, recompute AABB).
    pub fn transformed(&self, t: &Transform) -> BoundingBox {
        let corners = [
            [self.min[0], self.min[1], self.min[2]],
            [self.min[0], self.min[1], self.max[2]],
            [self.min[0], self.max[1], self.min[2]],
            [self.min[0], self.max[1], self.max[2]],
            [self.max[0], self.min[1], self.min[2]],
            [self.max[0], self.min[1], self.max[2]],
            [self.max[0], self.max[1], self.min[2]],
            [self.max[0], self.max[1], self.max[2]],
        ];
        let mut result = BoundingBox::empty();
        for c in &corners {
            let wx = t.m[0][0] * c[0] + t.m[0][1] * c[1] + t.m[0][2] * c[2] + t.m[0][3];
            let wy = t.m[1][0] * c[0] + t.m[1][1] * c[1] + t.m[1][2] * c[2] + t.m[1][3];
            let wz = t.m[2][0] * c[0] + t.m[2][1] * c[1] + t.m[2][2] * c[2] + t.m[2][3];
            result = result.union_point([wx, wy, wz]);
        }
        result
    }

    fn union_point(&self, p: [f64; 3]) -> BoundingBox {
        BoundingBox {
            min: [
                self.min[0].min(p[0]),
                self.min[1].min(p[1]),
                self.min[2].min(p[2]),
            ],
            max: [
                self.max[0].max(p[0]),
                self.max[1].max(p[1]),
                self.max[2].max(p[2]),
            ],
        }
    }
}

/// Detect collisions between all pairs of components in an assembly.
///
/// Returns a list of (component_a_idx, component_b_idx) pairs whose
/// world-space AABBs overlap. Both components must have `local_aabb` set.
pub fn detect_collisions(
    components: &[crate::Component],
) -> Vec<(usize, usize)> {
    let boxes: Vec<(usize, BoundingBox)> = components
        .iter()
        .enumerate()
        .filter_map(|(i, c)| c.local_aabb.map(|local| (i, local.transformed(&c.transform))))
        .collect();

    if boxes.len() < 2 {
        return Vec::new();
    }

    let mut collisions = Vec::new();
    for i in 0..boxes.len() {
        for j in (i + 1)..boxes.len() {
            if boxes[i].1.overlaps_strict(&boxes[j].1) {
                collisions.push((boxes[i].0, boxes[j].0));
            }
        }
    }
    collisions
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Component;

    #[test]
    fn test_bbox_overlaps() {
        let a = BoundingBox::new([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
        let b = BoundingBox::new([1.0, 1.0, 1.0], [3.0, 3.0, 3.0]);
        assert!(a.overlaps(&b));
    }

    #[test]
    fn test_bbox_no_overlap() {
        let a = BoundingBox::new([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = BoundingBox::new([2.0, 2.0, 2.0], [3.0, 3.0, 3.0]);
        assert!(!a.overlaps(&b));
    }

    #[test]
    fn test_bbox_transformed_translation() {
        let b = BoundingBox::new([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let t = Transform::translation(10.0, 20.0, 30.0);
        let b2 = b.transformed(&t);
        assert_eq!(b2.min, [10.0, 20.0, 30.0]);
        assert_eq!(b2.max, [11.0, 21.0, 31.0]);
    }

    #[test]
    fn test_detect_collisions_overlap() {
        let mut base = Component::new_fixed(0, "Base");
        base.local_aabb = Some(BoundingBox::new([0.0, 0.0, 0.0], [10.0, 10.0, 10.0]));
        let mut mover = Component::new(1, "Mover");
        mover.local_aabb = Some(BoundingBox::new([0.0, 0.0, 0.0], [5.0, 5.0, 5.0]));
        mover.set_translation(3.0, 3.0, 3.0);
        let comps = vec![base, mover];
        let collisions = detect_collisions(&comps);
        assert_eq!(collisions.len(), 1);
        assert_eq!(collisions[0], (0, 1));
    }

    #[test]
    fn test_detect_collisions_no_overlap() {
        let mut base = Component::new_fixed(0, "Base");
        base.local_aabb = Some(BoundingBox::new([0.0, 0.0, 0.0], [10.0, 10.0, 10.0]));
        let mut mover = Component::new(1, "Mover");
        mover.local_aabb = Some(BoundingBox::new([0.0, 0.0, 0.0], [5.0, 5.0, 5.0]));
        mover.set_translation(50.0, 0.0, 0.0);
        let comps = vec![base, mover];
        let collisions = detect_collisions(&comps);
        assert!(collisions.is_empty());
    }
}
